"""AIS on the gateway (section A): RECEIVE ONLY. The AURIX never parses AIS.

Input: NMEA 0183 sentences from a USB AIS receiver (38400 baud, by a stable
name: osdep.resolve_serial), or a TCP source (tests, network receivers):
    !AIVDM  other ships          !AIVDO  our own transponder (later, Class B)
    $xxRMC / $xxGGA              GNSS of an AIS device (second, independent GPS)
Decoded: position reports Class A (types 1, 2, 3) and Class B (18, 19),
static / voyage data (5, 24 A/B, 19). Everything else is counted and ignored.

Targets (by MMSI): last report, a predicted position between reports (speed
and course), age; STALE after stale_a_s / stale_b_s (Class A 3 min, B 6 min),
removed after remove_s. Obviously wrong data is ignored (bad checksum,
no position, impossible MMSI, a jump no ship can make).
Optional online feed (AISHub JSON) merged by MMSI: the newer report wins,
local data wins at equal age.
"""
import json
import math
import re
import threading
import time
import urllib.request

NM = 1852.0
KN = 1852.0 / 3600.0            # m/s per knot

# --- NMEA ---------------------------------------------------------------------------------

def nmea_checksum(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return "%02X" % c


def nmea_ok(line):
    """'!AIVDM,...*5C' with a correct checksum -> body (without ! and *xx), else None."""
    line = line.strip()
    m = re.match(r"^[!$]([^*]+)\*([0-9A-Fa-f]{2})$", line)
    if not m:
        return None
    return m.group(1) if nmea_checksum(m.group(1)) == m.group(2).upper() else None


def armor_to_bits(payload, fill=0):
    bits = []
    for ch in payload:
        v = ord(ch) - 48
        if v > 40:
            v -= 8
        if not 0 <= v < 64:
            raise ValueError("bad payload character %r" % ch)
        bits.append(format(v, "06b"))
    s = "".join(bits)
    return s[:len(s) - fill] if fill else s


def bits_to_armor(bits):
    fill = (-len(bits)) % 6
    bits += "0" * fill
    out = []
    for i in range(0, len(bits), 6):
        v = int(bits[i:i + 6], 2)
        out.append(chr(v + 48 if v < 40 else v + 56))
    return "".join(out), fill


def _u(b, start, n):
    return int(b[start:start + n], 2) if len(b) >= start + n else None


def _i(b, start, n):
    v = _u(b, start, n)
    if v is None:
        return None
    return v - (1 << n) if v & (1 << (n - 1)) else v


SIXBIT = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?"


def _t(b, start, nchars):
    out = []
    for k in range(nchars):
        v = _u(b, start + 6 * k, 6)
        if v is None:
            break
        out.append(SIXBIT[v])
    return "".join(out).replace("@", " ").strip()


def _pos(b, lon_at, lat_at):
    lon = _i(b, lon_at, 28)
    lat = _i(b, lat_at, 27)
    if lon is None or lat is None:
        return None, None
    lon, lat = lon / 600000.0, lat / 600000.0
    if not (-180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0):
        return None, None                           # 181 / 91 = not available
    return lat, lon


def decode(bits):
    """One AIS message (bit string) -> dict, or None if not a type we use."""
    t = _u(bits, 0, 6)
    mmsi = _u(bits, 8, 30)
    if t is None or mmsi is None:
        return None
    d = {"type": t, "mmsi": mmsi}
    if t in (1, 2, 3):
        if len(bits) < 168:
            return None
        lat, lon = _pos(bits, 61, 89)
        sog, cog, hdg = _u(bits, 50, 10), _u(bits, 116, 12), _u(bits, 128, 9)
        d.update(cls="A", nav_status=_u(bits, 38, 4), rot=_i(bits, 42, 8), lat=lat, lon=lon,
                 sog=None if sog == 1023 else sog / 10.0, cog=None if cog >= 3600 else cog / 10.0,
                 heading=None if hdg == 511 else hdg, accuracy=_u(bits, 60, 1), second=_u(bits, 137, 6))
    elif t in (18, 19):
        if len(bits) < 168:
            return None
        lat, lon = _pos(bits, 57, 85)
        sog, cog, hdg = _u(bits, 46, 10), _u(bits, 112, 12), _u(bits, 124, 9)
        d.update(cls="B", lat=lat, lon=lon, sog=None if sog == 1023 else sog / 10.0,
                 cog=None if cog >= 3600 else cog / 10.0, heading=None if hdg == 511 else hdg,
                 accuracy=_u(bits, 56, 1), second=_u(bits, 133, 6))
        if t == 19 and len(bits) >= 312:
            d.update(name=_t(bits, 143, 20), ship_type=_u(bits, 263, 8), to_bow=_u(bits, 271, 9),
                     to_stern=_u(bits, 280, 9), to_port=_u(bits, 289, 6), to_starboard=_u(bits, 295, 6))
    elif t == 5:
        if len(bits) < 420:
            return None
        d.update(cls="A", imo=_u(bits, 40, 30), callsign=_t(bits, 70, 7), name=_t(bits, 112, 20),
                 ship_type=_u(bits, 232, 8), to_bow=_u(bits, 240, 9), to_stern=_u(bits, 249, 9),
                 to_port=_u(bits, 258, 6), to_starboard=_u(bits, 264, 6),
                 draught=_u(bits, 294, 8) / 10.0, destination=_t(bits, 302, 20))
    elif t == 24:
        part = _u(bits, 38, 2)
        d.update(cls="B", part=part)
        if part == 0 and len(bits) >= 160:
            d.update(name=_t(bits, 40, 20))
        elif part == 1 and len(bits) >= 162:
            d.update(ship_type=_u(bits, 40, 8), callsign=_t(bits, 90, 7), to_bow=_u(bits, 132, 9),
                     to_stern=_u(bits, 141, 9), to_port=_u(bits, 150, 6), to_starboard=_u(bits, 156, 6))
        else:
            return None
    else:
        return None
    return d


# --- encoder (simulated traffic, tests) ---------------------------------------------------

def _pu(v, n):
    return format(int(v) & ((1 << n) - 1), "0%db" % n)


def _pt(s, nchars):
    s = (s or "").upper()[:nchars].ljust(nchars, "@")
    return "".join(_pu(SIXBIT.index(c) if c in SIXBIT else 0, 6) for c in s)


def encode(msg):
    """dict (like decode() returns) -> bit string."""
    t = msg["type"]
    lat = msg.get("lat")
    lon = msg.get("lon")
    la = 91 * 600000 if lat is None else round(lat * 600000)
    lo = 181 * 600000 if lon is None else round(lon * 600000)
    sog = 1023 if msg.get("sog") is None else round(msg["sog"] * 10)
    cog = 3600 if msg.get("cog") is None else round(msg["cog"] * 10) % 3600
    hdg = 511 if msg.get("heading") is None else int(msg["heading"]) % 360
    head = _pu(t, 6) + _pu(0, 2) + _pu(msg["mmsi"], 30)
    if t in (1, 2, 3):
        return (head + _pu(msg.get("nav_status", 0), 4) + _pu(msg.get("rot", -128), 8) + _pu(sog, 10) + _pu(1, 1) +
                _pu(lo, 28) + _pu(la, 27) + _pu(cog, 12) + _pu(hdg, 9) + _pu(msg.get("second", 60), 6) +
                _pu(0, 2) + _pu(0, 3) + _pu(0, 1) + _pu(0, 19))
    if t == 18:
        return (head + _pu(0, 8) + _pu(sog, 10) + _pu(1, 1) + _pu(lo, 28) + _pu(la, 27) + _pu(cog, 12) +
                _pu(hdg, 9) + _pu(msg.get("second", 60), 6) + _pu(0, 2) + _pu(1, 1) + _pu(0, 1) + _pu(1, 1) +
                _pu(1, 1) + _pu(1, 1) + _pu(0, 1) + _pu(0, 1) + _pu(0, 20))
    if t == 5:
        return (head + _pu(0, 2) + _pu(msg.get("imo", 0), 30) + _pt(msg.get("callsign"), 7) +
                _pt(msg.get("name"), 20) + _pu(msg.get("ship_type", 0), 8) + _pu(msg.get("to_bow", 0), 9) +
                _pu(msg.get("to_stern", 0), 9) + _pu(msg.get("to_port", 0), 6) + _pu(msg.get("to_starboard", 0), 6) +
                _pu(1, 4) + _pu(0, 20) + _pu(round(msg.get("draught", 0) * 10), 8) + _pt(msg.get("destination"), 20) +
                _pu(0, 1) + _pu(0, 1))
    if t == 24:
        if msg.get("part", 0) == 0:
            return head + _pu(0, 2) + _pt(msg.get("name"), 20)
        return (head + _pu(1, 2) + _pu(msg.get("ship_type", 0), 8) + _pt("", 7) +     # vendor id: 42 bits
                _pt(msg.get("callsign"), 7) + _pu(msg.get("to_bow", 0), 9) + _pu(msg.get("to_stern", 0), 9) +
                _pu(msg.get("to_port", 0), 6) + _pu(msg.get("to_starboard", 0), 6) + _pu(0, 6))
    raise ValueError("type %d not supported by the encoder" % t)


def sentences(msg, channel="A", talker="AIVDM", seq_id=0):
    """dict -> list of NMEA sentences (split into 2 if needed)."""
    payload, fill = bits_to_armor(encode(msg))
    parts = [payload[i:i + 60] for i in range(0, len(payload), 60)] or [""]
    out = []
    for k, p in enumerate(parts):
        f = fill if k == len(parts) - 1 else 0
        body = "%s,%d,%d,%s,%s,%s,%d" % (talker, len(parts), k + 1, str(seq_id % 10) if len(parts) > 1 else "",
                                         channel, p, f)
        out.append("!%s*%s" % (body, nmea_checksum(body)))
    return out


# --- geometry -----------------------------------------------------------------------------

def to_xy(lat, lon, lat0, lon0):
    """metres east / north of (lat0, lon0) (flat earth: fine for a few tens of km)."""
    return ((lon - lon0) * 111320.0 * math.cos(math.radians(lat0)), (lat - lat0) * 111320.0)


def from_xy(x, y, lat0, lon0):
    return lat0 + y / 111320.0, lon0 + x / (111320.0 * math.cos(math.radians(lat0)))


def dist_m(lat1, lon1, lat2, lon2):
    x, y = to_xy(lat2, lon2, lat1, lon1)
    return math.hypot(x, y)


def bearing(lat1, lon1, lat2, lon2):
    x, y = to_xy(lat2, lon2, lat1, lon1)
    return math.degrees(math.atan2(x, y)) % 360.0


def move(lat, lon, cog, sog_kn, dt):
    s = sog_kn * KN * dt
    return from_xy(s * math.sin(math.radians(cog)), s * math.cos(math.radians(cog)), lat, lon)


# --- targets --------------------------------------------------------------------------------

class Target:
    def __init__(self, mmsi):
        self.mmsi = mmsi
        self.cls = "?"
        self.lat = self.lon = None
        self.sog = self.cog = self.heading = None
        self.nav_status = None
        self.t_pos = 0.0                # when the position was reported (our clock)
        self.t_seen = 0.0               # any message
        self.name = ""
        self.ship_type = None
        self.length = self.beam = None
        self.draught = None
        self.destination = ""
        self.callsign = ""
        self.src = "local"              # "local" receiver or "online" feed
        self.msgs = 0
        self.cog_hist = []              # (t, cog): turning detection

    def predict(self, t):
        """Position at time t from the last report (speed / course)."""
        if self.lat is None:
            return None, None
        if not self.sog or self.cog is None:
            return self.lat, self.lon
        return move(self.lat, self.lon, self.cog, self.sog, max(0.0, t - self.t_pos))

    def turn_rate(self):
        """deg/min over the recent course reports (+ = to starboard)."""
        h = self.cog_hist
        if len(h) < 2 or h[-1][0] - h[0][0] < 20:
            return 0.0
        d = (h[-1][1] - h[0][1] + 540.0) % 360.0 - 180.0
        return d / ((h[-1][0] - h[0][0]) / 60.0)

    def info(self, now, stale):
        lat, lon = self.predict(now)
        return {"mmsi": self.mmsi, "cls": self.cls, "name": self.name, "lat": lat, "lon": lon, "sog": self.sog,
                "cog": self.cog, "heading": self.heading, "age_s": round(now - self.t_pos, 1) if self.t_pos else None,
                "stale": stale, "type": self.ship_type, "length": self.length, "beam": self.beam,
                "draught": self.draught, "dest": self.destination, "callsign": self.callsign, "src": self.src,
                "nav_status": self.nav_status, "turn_dpm": round(self.turn_rate(), 1)}


class AisTargets:
    def __init__(self, cfg, clock=time.time):
        self.cfg = cfg
        self.now = clock
        self.targets = {}
        self.frags = {}                 # (seq id, channel) -> (time, parts)
        self.msg_times = []             # last 60 s (rate)
        self.total = self.invalid = self.bad_crc = self.ignored = 0
        self.last_msg = None            # time of the last good AIS message (silent alarm)
        self.own = None                 # AIVDO: our own transponder's report
        self.gps2 = None                # second GPS: {"lat", "lon", "sog", "cog", "t"}
        self.lock = threading.RLock()
        self.stale_a = cfg.get("stale_a_s", 180.0)
        self.stale_b = cfg.get("stale_b_s", 360.0)
        self.remove = cfg.get("remove_s", 900.0)
        self.max_jump_kn = cfg.get("max_jump_kn", 60.0)

    # --- input ------------------------------------------------------------------------
    def feed_line(self, line, src="local"):
        body = nmea_ok(line)
        if body is None:
            if line.strip():
                self.bad_crc += 1
            return None
        f = body.split(",")
        talker = f[0][2:] if len(f[0]) == 5 else f[0]
        if talker in ("RMC", "GGA"):
            return self._gnss(talker, f)
        if talker not in ("VDM", "VDO") or len(f) < 7:
            self.ignored += 1
            return None
        try:
            count, num, seq, chan, payload, fill = int(f[1]), int(f[2]), f[3], f[4], f[5], int(f[6] or 0)
        except ValueError:
            self.invalid += 1
            return None
        if count > 1:
            key = (seq, chan)
            now = self.now()
            t0, parts = self.frags.get(key, (now, {}))
            if now - t0 > 5.0:
                parts = {}
            parts[num] = (payload, fill)
            self.frags[key] = (t0 if parts else now, parts)
            if len(parts) < count:
                return None
            del self.frags[key]
            payload = "".join(parts[k][0] for k in range(1, count + 1))
            fill = parts[count][1]
        try:
            d = decode(armor_to_bits(payload, fill))
        except ValueError:
            self.invalid += 1
            return None
        if d is None:
            self.ignored += 1
            return None
        if talker == "VDO":
            self.own = dict(d, t=self.now())
            if d.get("lat") is not None:
                self.gps2 = {"lat": d["lat"], "lon": d["lon"], "sog": d.get("sog"), "cog": d.get("cog"),
                             "t": self.now(), "src": "AIVDO"}
            return d
        self.apply(d, src)
        return d

    def _gnss(self, talker, f):
        """$..RMC / $..GGA from the AIS device: a second GPS (off unless configured)."""
        try:
            if talker == "RMC" and f[2] == "A":
                lat = int(f[3][:2]) + float(f[3][2:]) / 60.0
                lon = int(f[5][:3]) + float(f[5][3:]) / 60.0
                lat = -lat if f[4] == "S" else lat
                lon = -lon if f[6] == "W" else lon
                self.gps2 = {"lat": lat, "lon": lon, "sog": float(f[7] or 0), "cog": float(f[8] or 0),
                             "t": self.now(), "src": "RMC"}
            elif talker == "GGA" and f[6] not in ("", "0"):
                lat = int(f[2][:2]) + float(f[2][2:]) / 60.0
                lon = int(f[4][:3]) + float(f[4][3:]) / 60.0
                lat = -lat if f[3] == "S" else lat
                lon = -lon if f[5] == "W" else lon
                g = self.gps2 or {}
                self.gps2 = {"lat": lat, "lon": lon, "sog": g.get("sog"), "cog": g.get("cog"), "t": self.now(),
                             "src": "GGA"}
        except (ValueError, IndexError):
            self.invalid += 1
        return None

    def apply(self, d, src="local", t_report=None):
        """A decoded message (or an online record) into the target list."""
        mmsi = d.get("mmsi") or 0
        if not 100000000 <= mmsi <= 999999999:
            self.invalid += 1
            return
        now = self.now()
        t_rep = now if t_report is None else t_report
        with self.lock:
            self.total += 1
            if src == "local":
                self.last_msg = now
                self.msg_times.append(now)
            tg = self.targets.get(mmsi)
            if tg is None:
                tg = self.targets[mmsi] = Target(mmsi)
            tg.msgs += 1
            tg.t_seen = max(tg.t_seen, t_rep)
            if d.get("cls") in ("A", "B"):
                tg.cls = d["cls"]
            for k in ("name", "callsign", "destination"):
                if d.get(k):
                    setattr(tg, k, d[k])
            if d.get("ship_type") is not None:
                tg.ship_type = d["ship_type"]
            if d.get("draught"):
                tg.draught = d["draught"]
            if d.get("to_bow") is not None and d.get("to_stern") is not None:
                tg.length = (d["to_bow"] or 0) + (d["to_stern"] or 0) or tg.length
                tg.beam = (d.get("to_port") or 0) + (d.get("to_starboard") or 0) or tg.beam
            if "lat" in d:
                if d.get("lat") is None:
                    return                                      # no position in this report
                if tg.t_pos and t_rep < tg.t_pos - 0.01:
                    return                                      # older than what we have
                if tg.t_pos and src != "local" and tg.src == "local" and t_rep <= tg.t_pos + 0.01:
                    return                                      # local wins at the same age
                if tg.lat is not None and t_rep > tg.t_pos:
                    plat, plon = tg.predict(t_rep)
                    dt = t_rep - tg.t_pos
                    jump = dist_m(plat, plon, d["lat"], d["lon"])
                    if jump > max(500.0, self.max_jump_kn * KN * dt):
                        self.invalid += 1                       # no ship moves like that
                        return
                tg.lat, tg.lon = d["lat"], d["lon"]
                tg.sog = d.get("sog")
                tg.cog = d.get("cog") if d.get("cog") is not None else tg.cog
                tg.heading = d.get("heading")
                tg.nav_status = d.get("nav_status", tg.nav_status)
                tg.t_pos = t_rep
                tg.src = src
                if tg.cog is not None and (tg.sog or 0) > 1.0:
                    tg.cog_hist = [(t, c) for t, c in tg.cog_hist if t_rep - t < 120.0] + [(t_rep, tg.cog)]

    # --- maintenance --------------------------------------------------------------------
    def stale_limit(self, tg):
        return self.stale_b if tg.cls == "B" else self.stale_a

    def is_stale(self, tg, now=None):
        now = self.now() if now is None else now
        return not tg.t_pos or now - tg.t_pos > self.stale_limit(tg)

    def tick(self):
        now = self.now()
        with self.lock:
            for m in [m for m, tg in self.targets.items() if now - max(tg.t_pos, tg.t_seen) > self.remove]:
                del self.targets[m]
            self.msg_times = [t for t in self.msg_times if now - t < 60.0]
            for k in [k for k, (t0, _p) in self.frags.items() if now - t0 > 5.0]:
                del self.frags[k]

    def silent_for(self, started):
        """Seconds without any AIS message (since the receiver start if none yet)."""
        return self.now() - (self.last_msg or started)

    def list(self):
        now = self.now()
        with self.lock:
            return [tg.info(now, self.is_stale(tg, now)) for tg in self.targets.values() if tg.lat is not None]

    def status(self):
        now = self.now()
        with self.lock:
            n_stale = sum(1 for tg in self.targets.values() if self.is_stale(tg, now))
            return {"targets": len(self.targets), "stale": n_stale, "msgs_per_min": len(self.msg_times),
                    "total": self.total, "invalid": self.invalid, "bad_crc": self.bad_crc, "ignored": self.ignored,
                    "last_msg_s": None if self.last_msg is None else round(now - self.last_msg, 1),
                    "gps2": None if not self.gps2 else dict(self.gps2, age_s=round(now - self.gps2["t"], 1))}


# --- online feed (AISHub) -----------------------------------------------------------------

def parse_aishub(text, now):
    """AISHub JSON (format=1, human readable) -> [(msg dict, report time)]."""
    data = json.loads(text)
    rows = data[1] if isinstance(data, list) and len(data) > 1 else []
    out = []
    for r in rows:
        try:
            mmsi = int(r["MMSI"])
            lat, lon = float(r["LATITUDE"]), float(r["LONGITUDE"])
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                continue
            sog = float(r.get("SOG", 102.3))
            cog = float(r.get("COG", 360))
            t = float(r.get("TIME_UNIX") or r.get("TSTAMP") or now)
            msg = {"mmsi": mmsi, "lat": lat, "lon": lon, "sog": None if sog >= 102.2 else sog,
                   "cog": None if cog >= 360 else cog, "heading": None if int(r.get("HEADING", 511)) == 511
                   else int(r["HEADING"]), "name": (r.get("NAME") or "").strip(),
                   "ship_type": r.get("TYPE"), "cls": "A" if int(r.get("A", 0)) + int(r.get("B", 0)) > 0 else "?"}
            if r.get("A") is not None and r.get("B") is not None:
                msg.update(to_bow=int(r["A"]), to_stern=int(r["B"]), to_port=int(r.get("C", 0)),
                           to_starboard=int(r.get("D", 0)))
            out.append((msg, min(t, now)))
        except (KeyError, ValueError, TypeError):
            continue
    return out


class OnlineFeed:
    """Polls an AISHub-style URL ({lat0} {lat1} {lon0} {lon1} filled in) - only
    when the gateway says the channel allows it (never STARLINK)."""

    def __init__(self, cfg, targets, allowed):
        self.cfg = cfg
        self.targets = targets
        self.allowed = allowed          # () -> (bool, why)
        self.url = cfg.get("url")
        self.period = cfg.get("period_s", 60.0)
        self.next = 0.0
        self.polls = self.records = self.errors = 0
        self.state = "OFF"
        self.box = None                 # (lat0, lat1, lon0, lon1)

    def tick(self, own_lat=None, own_lon=None):
        if not self.cfg.get("enabled") or not self.url:
            self.state = "OFF"
            return
        ok, why = self.allowed()
        if not ok:
            self.state = "PAUSED: " + why
            return
        now = time.time()
        if now < self.next:
            return
        self.next = now + self.period
        r = self.cfg.get("radius_deg", 0.5)
        if own_lat is None:
            return
        self.box = (own_lat - r, own_lat + r, own_lon - r, own_lon + r)
        url = self.url.format(lat0=self.box[0], lat1=self.box[1], lon0=self.box[2], lon1=self.box[3])
        try:
            with urllib.request.urlopen(url, timeout=10) as f:
                rows = parse_aishub(f.read().decode("utf-8", "replace"), time.time())
            for msg, t in rows:
                self.targets.apply(msg, "online", t_report=self.targets.now() - (time.time() - t))
            self.polls += 1
            self.records += len(rows)
            self.state = "OK (%d ships)" % len(rows)
        except Exception as e:                      # network, JSON - never fatal
            self.errors += 1
            self.state = "ERROR: %s" % str(e)[:60]

    def status(self):
        return {"state": self.state, "polls": self.polls, "records": self.records, "errors": self.errors}
