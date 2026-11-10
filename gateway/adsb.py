"""ADS-B receive only (CLAUDE.md X K4, docs/sar_design.md 6, step PF5): manned aircraft near the UAVs.

Pure code (no network, no process, no AURIX, no UAV command): the worker (adsb_worker.py) reads a 1090 MHz receiver
(readsb / dump1090 SBS port 30003, or Beast port 30005) with parse_sbs / BeastParser, keeps the aircraft in
AdsbTargets; the manager (adsbmgr.py) judges them against the UAVs that are IN THE AIR with evaluate().

  parse_sbs(line)        one SBS (BaseStation) line -> {"icao", cs / lat / lon / alt_ft / gs_kn / trk / vs_fpm / gnd}
                         or None (a broken line is counted by the caller)
  BeastParser            Beast binary framing (0x1a escapes) + the Mode S CRC-24 check + DF17 decoding (identity,
                         altitude, velocity, CPR position with an even / odd pair) -> the same dicts
  AdsbTargets            the aircraft: newest values, a short track, stale after 60 s, removed after 180 s
  cpa(ac, uav)           the closest point of approach of an aircraft and a UAV (horizontal AND vertical, like the
                         AIS CPA in avoid.py): t_cpa, d_h, d_v
  evaluate(...)          per (aircraft, flying UAV): CAUTION / WARNING (levels below) + the proposal kind
  proposal_kind(...)     HOLD / DESCEND text of the proposal - a PROPOSAL only, never a command

Levels (TO CONFIRM W132): CAUTION d_h < 1000 m AND d_v < 150 m within 120 s; WARNING d_h < 500 m AND d_v < 100 m
within 60 s. UNKNOWN altitude (owner W137, 2026-11-10; step AB-r): an aircraft without an altitude, or a UAV whose
altitude is unknown (missing / non-finite / stale), is taken as CO-ALTITUDE with the UAV (d_v = 0, never a guess of
"far above"): the horizontal CPA alone decides CAUTION / WARNING, the event says "altitude unknown" (d_v None,
alt_unknown True) and the HOLD / DESCEND proposal is offered as for any conflict (proposal_kind: the unknown side at
the known side's altitude; both unknown = DESCEND). Still only a proposal - nothing is sent without a YES. Barometric altitude
of the aircraft against the UAV's height above the launch point (= the sea): the difference is a few tens of metres
at most near sea level; the limits carry it.
"""
import collections
import math

KN = 1852.0 / 3600.0
FT = 0.3048
FPM = FT / 60.0

DEFAULTS = {
    "max_alt_ft": 3000.0,        # only aircraft below this are kept (TO CONFIRM W132)
    "max_km": 20.0,              # only aircraft within this of the boat / a flying UAV
    "stale_s": 60.0,             # shown grey after this
    "remove_s": 180.0,           # forgotten after this
    "cpa_max_age_s": 30.0,       # a position older than this is not used for a warning
    "caution_h_m": 1000.0, "caution_v_m": 150.0, "caution_t_s": 120.0,
    "warn_h_m": 500.0, "warn_v_m": 100.0, "warn_t_s": 60.0,
    "alt_safe_m": 30.0,          # the DESCEND proposal's height
    "track_s": 120.0,
    "no_data_s": 10.0,           # no frame for this long = NO_DATA
}

LEVEL_NONE, LEVEL_CAUTION, LEVEL_WARNING = 0, 1, 2


def merged(cfg):
    out = dict(DEFAULTS)
    for k, v in (cfg or {}).items():
        if k in out and isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and v > 0:
            out[k] = float(v)
    return out


def _num(s, conv=float):
    s = (s or "").strip()
    if not s:
        return None
    try:
        v = conv(s)
    except ValueError:
        return None
    if isinstance(v, float) and (v != v or abs(v) == float("inf")):
        return None
    return v


# --- SBS (BaseStation) -------------------------------------------------------------------------------------------
MAX_AIRCRAFT = 2000                 # PF5-r LOW: the aircraft table never grows without a limit
UAV_MAX_AGE_S = 5.0                 # PF5-r MED-2: a UAV record older than this = its altitude / speed are unknown


def parse_sbs(line):
    """MSG,<type>,sess,aircraft,<hex>,flight,date,time,date,time,<callsign>,<alt>,<gs>,<trk>,<lat>,<lon>,<vs>,<sq>,
    alert,emerg,spi,<onground>. Types 1 (callsign) 3 (position + altitude) 4 (speed, track, vertical rate) 5 / 6 / 7
    (altitude) 8 (on ground). Anything else / a broken line -> None."""
    if isinstance(line, bytes):
        line = line.decode("ascii", "replace")
    f = line.strip().split(",")
    if len(f) < 11 or f[0] != "MSG":
        return None
    t = _num(f[1], int)
    icao = f[4].strip().upper()
    if t is None or not (1 <= t <= 8) or len(icao) != 6:
        return None
    try:
        int(icao, 16)
    except ValueError:
        return None
    d = {"icao": icao}
    g = lambda i: f[i] if i < len(f) else ""
    if t == 1:
        cs = g(10).strip()
        if cs:
            d["cs"] = cs[:8]
    if t in (2, 3, 5, 6, 7) and _num(g(11)) is not None:
        d["alt_ft"] = _num(g(11))
    if t in (2, 4) and _num(g(12)) is not None:
        d["gs_kn"] = _num(g(12))
    if t in (2, 4) and _num(g(13)) is not None:
        d["trk"] = _num(g(13)) % 360.0
    if t in (2, 3):
        la, lo = _num(g(14)), _num(g(15))
        if la is not None and lo is not None:
            if abs(la) > 90.0 or abs(lo) > 180.0:
                return None
            d["lat"], d["lon"] = la, lo
    if t == 4 and _num(g(16)) is not None:
        d["vs_fpm"] = _num(g(16))
    # PF5-r MED-1: the on-ground flag is read on EVERY type that carries it (not only 2 / 8), and an airborne
    # position (type 3) without the flag clears it - an aircraft that took off is never skipped for ever
    if g(21).strip() in ("0", "-1", "1"):
        d["gnd"] = g(21).strip() in ("-1", "1")
    elif t == 3 and "lat" in d:
        d["gnd"] = False
    if len(d) == 1:
        return None
    return d


# --- Beast + Mode S ----------------------------------------------------------------------------------------------
_GEN = 0xFFF409


def crc24(data):
    """The Mode S CRC remainder of a whole message (112 bits): 0 for a good DF17 / DF18."""
    reg = 0
    for byte in data:
        reg ^= byte << 16
        for _ in range(8):
            reg <<= 1
            if reg & 0x1000000:
                reg ^= (0x1000000 | _GEN)
        reg &= 0xFFFFFF
    return reg


def _bits(v, start, n, total):
    return (v >> (total - start - n)) & ((1 << n) - 1)


_CS = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######"


def _nl(lat):
    lat = abs(lat)
    if lat < 1e-9:
        return 59
    if lat == 87.0:
        return 2
    if lat > 87.0:
        return 1
    nz = 15.0
    a = 1.0 - math.cos(math.pi / (2.0 * nz))
    b = math.cos(math.pi / 180.0 * lat) ** 2
    return int(math.floor(2.0 * math.pi / math.acos(1.0 - a / b)))


def cpr_global(lat0, lon0, lat1, lon1, newest_even):
    """17-bit CPR values (0..131071) of an even (0) and an odd (1) frame -> (lat, lon) or None (zone mismatch)."""
    c = 131072.0
    la0, lo0, la1, lo1 = lat0 / c, lon0 / c, lat1 / c, lon1 / c
    j = int(math.floor(59.0 * la0 - 60.0 * la1 + 0.5))
    lat_e = (360.0 / 60.0) * ((j % 60) + la0)
    lat_o = (360.0 / 59.0) * ((j % 59) + la1)
    if lat_e >= 270.0:
        lat_e -= 360.0
    if lat_o >= 270.0:
        lat_o -= 360.0
    if _nl(lat_e) != _nl(lat_o):
        return None
    if newest_even:
        lat, nl = lat_e, _nl(lat_e)
        ni = max(nl, 1)
        m = int(math.floor(lo0 * (nl - 1) - lo1 * nl + 0.5))
        lon = (360.0 / ni) * ((m % ni) + lo0)
    else:
        lat, nl = lat_o, _nl(lat_o)
        ni = max(nl - 1, 1)
        m = int(math.floor(lo0 * (nl - 1) - lo1 * nl + 0.5))
        lon = (360.0 / ni) * ((m % ni) + lo1)
    if lon >= 180.0:
        lon -= 360.0
    if abs(lat) > 90.0:
        return None
    return lat, lon


class BeastParser:
    """feed(bytes) -> [aircraft dicts]; counts frames / bad (a bad CRC, an unknown frame type)."""
    LEN = {0x31: 2, 0x32: 7, 0x33: 14}

    def __init__(self):
        self.buf = bytearray()
        self.cpr = {}                         # icao -> {0: (lat, lon, t), 1: (...)}
        self.frames = 0
        self.bad = 0
        self.clock = 0.0

    def feed(self, data, now=None):
        self.buf += data
        out = []
        b = self.buf
        i = 0
        while True:
            while i < len(b) and b[i] != 0x1A:
                i += 1
            if i + 1 >= len(b):
                break
            typ = b[i + 1]
            n = self.LEN.get(typ)
            if n is None:
                if typ == 0x1A:                 # an escaped 0x1a outside a frame
                    i += 2
                    continue
                self.bad += 1
                i += 1
                continue
            j, body = i + 2, bytearray()
            need = 7 + n
            partial = False
            while j < len(b) and len(body) < need:
                if b[j] == 0x1A:
                    if j + 1 >= len(b):
                        partial = True          # the escape's second byte has not arrived yet
                        break
                    if b[j + 1] == 0x1A:
                        body.append(0x1A)
                        j += 2
                        continue
                    break                       # a new frame start inside this one = truncated
                body.append(b[j])
                j += 1
            if len(body) < need:
                if j >= len(b) or partial:      # not complete yet
                    break
                self.bad += 1
                i = j
                continue
            i = j
            if typ == 0x33:
                self.frames += 1
                d = self.decode(bytes(body[7:]), now)
                if d:
                    out.append(d)
        del b[:i]
        return out

    def decode(self, msg, now=None):
        if len(msg) != 14:
            return None
        df = msg[0] >> 3
        if df not in (17, 18):
            return None
        if crc24(msg) != 0:
            self.bad += 1
            return None
        icao = "%02X%02X%02X" % (msg[1], msg[2], msg[3])
        me = int.from_bytes(msg[4:11], "big")
        tc = me >> 51
        d = {"icao": icao}
        if 1 <= tc <= 4:
            cs = "".join(_CS[_bits(me, 8 + 6 * k, 6, 56)] for k in range(8)).replace("#", "").strip()
            if cs:
                d["cs"] = cs
        elif 9 <= tc <= 18 or 20 <= tc <= 22:
            ac = _bits(me, 8, 12, 56)
            if ac & 0x10:                       # Q bit: 25 ft steps (Gillham altitudes are not decoded)
                n = ((ac >> 5) << 4) | (ac & 0xF)
                if tc <= 18:
                    d["alt_ft"] = n * 25.0 - 1000.0
            f = _bits(me, 21, 1, 56)
            la, lo = _bits(me, 22, 17, 56), _bits(me, 39, 17, 56)
            t = self.clock if now is None else now
            slot = self.cpr.setdefault(icao, {})
            slot[f] = (la, lo, t)
            o = slot.get(1 - f)
            if o is not None and abs(o[2] - t) <= 10.0:
                e, od = (slot[0], slot[1])
                r = cpr_global(e[0], e[1], od[0], od[1], newest_even=(f == 0))
                if r is not None:
                    d["lat"], d["lon"] = r
        elif tc == 19:
            st = _bits(me, 5, 3, 56)
            if st in (1, 2):
                k = 4.0 if st == 2 else 1.0
                vew, vns = _bits(me, 14, 10, 56) - 1, _bits(me, 25, 10, 56) - 1
                if vew >= 0 and vns >= 0:
                    if _bits(me, 13, 1, 56):
                        vew = -vew
                    if _bits(me, 24, 1, 56):
                        vns = -vns
                    d["gs_kn"] = math.hypot(vew, vns) * k
                    d["trk"] = math.degrees(math.atan2(vew, vns)) % 360.0
                vr = _bits(me, 37, 9, 56) - 1
                if vr >= 0:
                    d["vs_fpm"] = (-1 if _bits(me, 36, 1, 56) else 1) * vr * 64.0
        if len(d) == 1:
            return None
        return d


# --- the aircraft ------------------------------------------------------------------------------------------------
class AdsbTargets:
    def __init__(self, cfg=None):
        self.cfg = merged(cfg)
        self.ac = {}
        self.frames = 0                  # decoded updates
        self.bad = 0
        self.t_last = None               # the last update

    def update(self, d, now):
        icao = d.get("icao")
        if not icao:
            return
        a = self.ac.get(icao)
        if a is None:
            a = self.ac[icao] = {"icao": icao, "cs": "", "lat": None, "lon": None, "alt_ft": None, "gs_kn": None,
                                 "trk": None, "vs_fpm": None, "gnd": False, "t_pos": None, "t_seen": now,
                                 "track": collections.deque(maxlen=600)}
        for k in ("cs", "alt_ft", "gs_kn", "trk", "vs_fpm", "gnd"):
            if k in d:
                a[k] = d[k]
        # PF5-r MED-1: a positive altitude / speed without an on-ground flag in the same report = airborne again
        if "gnd" not in d and a["gnd"] and ((d.get("alt_ft") or 0) > 0 or (d.get("gs_kn") or 0) > 0):
            a["gnd"] = False
        if "lat" in d and "lon" in d:
            a["lat"], a["lon"], a["t_pos"] = d["lat"], d["lon"], now
            tr = a["track"]
            if not tr or now - tr[-1][2] >= 2.0:
                tr.append((d["lat"], d["lon"], now))
            while tr and now - tr[0][2] > self.cfg["track_s"]:
                tr.popleft()
        a["t_seen"] = now
        self.frames += 1
        self.t_last = now
        if len(self.ac) > MAX_AIRCRAFT:
            self.cap()

    def tick(self, now):
        for k in [k for k, a in self.ac.items() if now - a["t_seen"] > self.cfg["remove_s"]]:
            del self.ac[k]
        self.cap()

    def cap(self):
        """PF5-r LOW: at most MAX_AIRCRAFT aircraft are kept (a flood of fake / far ICAOs) - the oldest go first."""
        if len(self.ac) > MAX_AIRCRAFT:
            for k in sorted(self.ac, key=lambda k: self.ac[k]["t_seen"])[:len(self.ac) - MAX_AIRCRAFT]:
                del self.ac[k]

    def snapshot(self, now, max_n=60):
        """The aircraft with a position (a plain list: the worker sends it, the manager filters it)."""
        out = []
        for a in self.ac.values():
            if a["lat"] is None:
                continue
            out.append({"icao": a["icao"], "cs": a["cs"], "lat": a["lat"], "lon": a["lon"], "alt_ft": a["alt_ft"],
                        "gs_kn": a["gs_kn"], "trk": a["trk"], "vs_fpm": a["vs_fpm"], "gnd": a["gnd"],
                        "age": round(now - (a["t_pos"] or a["t_seen"]), 1),
                        "track": [(round(p[0], 5), round(p[1], 5)) for p in list(a["track"])[-30:]]})
        out.sort(key=lambda r: r["age"])
        return out[:max_n]


def _fin(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _f0(v):
    """A UAV velocity part: missing / non-finite = 0 (standing still) - never a NaN that silences every warning."""
    return float(v) if _fin(v) else 0.0


def dist_m(a, b):
    kx = math.cos(math.radians((a[0] + b[0]) / 2.0)) * 111320.0
    return math.hypot((b[1] - a[1]) * kx, (b[0] - a[0]) * 111320.0)


def keep(ac, refs, cfg):
    """The display / warning filter: an aircraft that is not on the ground, below max_alt_ft (an unknown altitude is
    KEPT: unknown is not safe) and within max_km of any reference point (the boat, a UAV in the air; none = all)."""
    if ac.get("gnd") or ac.get("lat") is None:
        return False
    alt = ac.get("alt_ft")
    if alt is not None and alt > cfg["max_alt_ft"]:
        return False
    if refs:
        return any(dist_m((ac["lat"], ac["lon"]), r) <= cfg["max_km"] * 1000.0 for r in refs)
    return True


def _enu(ref, p):
    kx = math.cos(math.radians(ref[0])) * 111320.0
    return (p[1] - ref[1]) * kx, (p[0] - ref[0]) * 111320.0


def cpa(ac, uav, cfg=None):
    """ac {lat, lon, alt_ft, gs_kn, trk, vs_fpm, age}, uav {lat, lon, alt (m), gs (m/s), cog, climb (m/s)} ->
    {"t": s to the horizontal CPA (0..300), "d_h", "d_v" at that time, "d_h0", "d_v0" now (None: no altitude)}.
    The aircraft is moved along its velocity by its age; a missing speed = standing still."""
    age = max(0.0, float(ac.get("age") or 0.0))
    gs = (ac.get("gs_kn") or 0.0) * KN
    trk = math.radians(ac.get("trk") or 0.0)
    avx, avy = gs * math.sin(trk), gs * math.cos(trk)
    avz = (ac.get("vs_fpm") or 0.0) * FPM
    ux, uy = _enu((uav["lat"], uav["lon"]), (ac["lat"], ac["lon"]))
    rx, ry = ux + avx * age, uy + avy * age
    ug = _f0(uav.get("gs"))
    uc = math.radians(_f0(uav.get("cog")))
    uvx, uvy = ug * math.sin(uc), ug * math.cos(uc)
    uvz = _f0(uav.get("climb"))
    vx, vy, vz = avx - uvx, avy - uvy, avz - uvz
    v2 = vx * vx + vy * vy
    t = 0.0
    if v2 > 0.01:
        t = min(300.0, max(0.0, -(rx * vx + ry * vy) / v2))
    alt_m = None if ac.get("alt_ft") is None else ac["alt_ft"] * FT + avz * age
    # PF5-r MED-2: an UNKNOWN UAV altitude is never sea level (0 m): no vertical separation is claimed; W137 (AB-r):
    # level_of then judges it as co-altitude (d_v None = 0 m apart)
    uz = uav.get("alt")
    uz = float(uz) if _fin(uz) else None
    rz = None if alt_m is None or uz is None else alt_m - uz

    def at(tt):
        return (math.hypot(rx + vx * tt, ry + vy * tt), None if rz is None else abs(rz + vz * tt))
    dh, dv = at(t)
    dh0, dv0 = at(0.0)
    return {"t": t, "d_h": dh, "d_v": dv, "d_h0": dh0, "d_v0": dv0, "ac_alt_m": alt_m, "uav_alt_m": uz}


def level_of(c, cfg):
    """The worse of 'now' and 'at the CPA' (an aircraft that is close now counts even when it is moving away).
    W137 (owner 2026-11-10, AB-r): d_v None (the aircraft's or the UAV's altitude unknown) = CO-ALTITUDE (0 m apart):
    the horizontal limits alone decide, CAUTION or WARNING exactly as for an aircraft at the UAV's own level."""
    best, tbest = LEVEL_NONE, c["t"]
    for tt, dh, dv in ((0.0, c["d_h0"], c["d_v0"]), (c["t"], c["d_h"], c["d_v"])):
        lv = LEVEL_NONE
        if dv is None:
            dv = 0.0
        if dh < cfg["warn_h_m"] and dv < cfg["warn_v_m"] and tt <= cfg["warn_t_s"]:
            lv = LEVEL_WARNING
        elif dh < cfg["caution_h_m"] and dv < cfg["caution_v_m"] and tt <= cfg["caution_t_s"]:
            lv = LEVEL_CAUTION
        if lv > best:
            best, tbest = lv, tt
    return best, tbest


def proposal_kind(c, cfg):
    """What to PROPOSE for a UAV near an aircraft: DESCEND (to alt_safe_m) when the aircraft is not clearly below the
    UAV and the UAV is well above the safe height, else HOLD (hover where it is). Never a command.
    W137 (AB-r): an unknown altitude = co-altitude - the unknown side is taken at the known side's altitude (an aircraft
    without an altitude at the UAV's height; a UAV of unknown height at the aircraft's), both unknown = DESCEND (get
    below the manned traffic; offered as for any conflict, the captain answers)."""
    ac, uz = c.get("ac_alt_m"), c.get("uav_alt_m")
    if ac is None and uz is None:
        return "descend"
    if ac is None:
        ac = uz
    if uz is None:
        uz = ac
    if ac is not None and ac >= uz - 15.0 and uz > cfg["alt_safe_m"] + 10.0 and ac > cfg["alt_safe_m"] + 30.0:
        return "descend"
    return "hold"


def evaluate(aircraft, drones, cfg, now_refs=(), unknown_out=None, nopos_out=None):
    """aircraft = snapshot list, drones = {slot: record of the UAV manager's drone_view} (only drones IN THE AIR are
    judged: armed, not landed, a position, not in the slot, a fresh position) -> (list of events worst first,
    list of the flying slots). An event: {icao, cs, slot, level, t, d_h, d_v, ac_alt_m, uav_alt_m, kind}."""
    cfg = merged(cfg) if "caution_h_m" not in cfg else cfg
    cfg = dict(cfg)
    cfg.setdefault("uav_max_age_s", UAV_MAX_AGE_S)
    air, unknown = {}, []
    for s, d in (drones or {}).items():
        try:
            n = int(s)
        except (TypeError, ValueError):
            continue
        if d.get("in_slot") or not d.get("armed") or d.get("landed"):
            continue
        if not _fin(d.get("lat")) or not _fin(d.get("lon")):
            unknown.append(n)                          # PF5-r MED-2 / LOW: in the air, no usable position
            if nopos_out is not None:                  # PF4-r2 LOW-3: NOT judged at all - its own notice
                nopos_out.append(n)
            continue
        d = dict(d)
        age = d.get("alt_age")
        # PF4-r2 LOW-2: a record WITHOUT an altitude age (key missing) = an unknown altitude, never "known"
        if not _fin(d.get("alt")) or "alt_age" not in d or                 (age is not None and (not _fin(age) or age > cfg["uav_max_age_s"])):
            # PF5-r MED-2: altitude missing / non-finite / stale (a lost link keeps the old record): judged with an
            # UNKNOWN altitude = co-altitude (W137: the horizontal CPA decides) and, stale, standing at its last
            # place - never a silent 0 m
            d["alt"] = None
            unknown.append(n)
        if age is not None and (not _fin(age) or age > cfg["uav_max_age_s"]):
            d["gs"] = d["climb"] = 0.0
        air[n] = d
    out = []
    for ac in aircraft:
        if ac.get("gnd") or ac.get("lat") is None or (ac.get("age") or 0.0) > cfg["cpa_max_age_s"]:
            continue
        if ac.get("alt_ft") is not None and ac["alt_ft"] > cfg["max_alt_ft"]:
            continue
        for n, d in air.items():
            c = cpa(ac, d, cfg)
            lv, t = level_of(c, cfg)
            if lv == LEVEL_NONE:
                continue
            out.append({"icao": ac["icao"], "cs": ac.get("cs") or "", "slot": n, "level": lv, "t": round(t, 1),
                        "d_h": round(c["d_h"]), "d_v": None if c["d_v"] is None else round(c["d_v"]),
                        "d_h0": round(c["d_h0"]), "ac_alt_m": None if c["ac_alt_m"] is None else round(c["ac_alt_m"]),
                        "uav_alt_m": None if c["uav_alt_m"] is None else round(c["uav_alt_m"], 1),
                        "alt_unknown": c["d_v"] is None,       # W137: judged as co-altitude, "altitude unknown"
                        "kind": proposal_kind(c, cfg)})
    out.sort(key=lambda e: (-e["level"], e["t"]))
    if unknown_out is not None:                        # PF5-r MED-2: the UAVs without a known altitude / position
        unknown_out.extend(sorted(set(unknown)))
    return out, sorted(air)
