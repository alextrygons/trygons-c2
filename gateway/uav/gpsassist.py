"""GPS warm start of a drone in its slot (CLAUDE.md V "GPS", docs/uav_design.md 7, owner spec H4).

While a drone sits in its slot (USB), the gateway gives its u-blox the time + position as u-blox UBX-MGA messages
through the flight controller's GPS injection (GPS_RTCM_DATA: raw bytes ArduPilot writes to its GPS port - that
ArduPilot passes UBX-MGA bytes on untouched is TO CONFIRM in SITL / on the bench, W56):
  UBX-MGA-INI-TIME_UTC   the gateway's clock (set from the boat's GPS time; never sent while it is not GPS-set)
  UBX-MGA-INI-POS_LLH    the boat's position from FRESH telemetry with a TRUSTED GPS (accuracy = the AURIX's
                         uncertainty, at least 10 m); else no position (time only)
  extra                  ephemeris / almanac bytes (MGA-GPS-EPH, an MGA-DBD dump) from an assist u-blox on the
                         gateway (W56 (b), recommended) - passed through unchanged when given
The boat's own u-blox is wired to the AURIX: the gateway has no ephemeris of its own (W56) - without the assist
receiver this is a WARM start (time + position), not a hot one.
"""
import struct
import time

MGA_CLASS, MGA_INI = 0x13, 0x40
RTCM_MAX = 180                         # GPS_RTCM_DATA data bytes per frame
RTCM_FRAGS = 4                         # at most 4 fragments per message (720 bytes)


def ubx(cls, mid, payload):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def parse_ubx(data):
    """[(cls, id, payload)] of every complete, checksum-correct UBX message in data (tests, the simulator)."""
    out, i = [], 0
    while i + 8 <= len(data):
        if data[i:i + 2] != b"\xb5\x62":
            i += 1
            continue
        ln = struct.unpack_from("<H", data, i + 4)[0]
        if i + 8 + ln > len(data):
            break
        body = data[i + 2:i + 6 + ln]
        a = b = 0
        for x in body:
            a = (a + x) & 0xFF
            b = (b + a) & 0xFF
        if bytes([a, b]) == data[i + 6 + ln:i + 8 + ln]:
            out.append((data[i + 2], data[i + 3], bytes(data[i + 6:i + 6 + ln])))
            i += 8 + ln
        else:
            i += 1
    return out


def mga_ini_time_utc(utc, acc_s=1.0):
    """UBX-MGA-INI-TIME_UTC (24 bytes): type 0x10, version 0, ref 0 (none), leapSecs -128 (unknown)."""
    t = time.gmtime(int(utc))
    ns = int((utc - int(utc)) * 1e9)
    acc_s = max(0.0, float(acc_s))
    p = struct.pack("<BBBbHBBBBBBIHBBI", 0x10, 0, 0, -128, t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min,
                    t.tm_sec, 0, ns, int(acc_s), 0, 0, int((acc_s - int(acc_s)) * 1e9))
    return ubx(MGA_CLASS, MGA_INI, p)


def mga_ini_pos_llh(lat, lon, alt_m, acc_m):
    """UBX-MGA-INI-POS_LLH (20 bytes): type 0x01, version 0, lat / lon 1e-7 deg, alt cm, accuracy cm."""
    p = struct.pack("<BBBBiiiI", 0x01, 0, 0, 0, int(round(lat * 1e7)), int(round(lon * 1e7)), int(round(alt_m * 100)),
                    int(min(4.0e7, max(0.0, acc_m)) * 100))
    return ubx(MGA_CLASS, MGA_INI, p)


def assist_bytes(utc=None, pos=None, extra=b""):
    """The UBX bytes for one warm start: [time] [position] [extra]; what = "time+pos" / "time" / "pos" / ""."""
    out, what = b"", []
    if utc is not None:
        out += mga_ini_time_utc(utc)
        what.append("time")
    if pos is not None:
        lat, lon, acc = pos
        out += mga_ini_pos_llh(lat, lon, 0.0, max(10.0, float(acc)))
        what.append("pos")
    if extra:
        out += bytes(extra)
        what.append("eph")
    return out, "+".join(what)


def rtcm_frames(data, seq):
    """GPS_RTCM_DATA field dicts for one blob (each UBX message its own blob): flags bit 0 fragmented, bits 1-2
    the fragment, bits 3-7 the sequence (5 bits)."""
    data = bytes(data)
    if len(data) > RTCM_MAX * RTCM_FRAGS:
        raise ValueError("assist blob too long (%d bytes)" % len(data))
    seq &= 0x1F
    if len(data) <= RTCM_MAX:
        return [{"flags": seq << 3, "len": len(data), "data": list(data)}]
    out = []
    for k in range(0, len(data), RTCM_MAX):
        part = data[k:k + RTCM_MAX]
        out.append({"flags": 1 | ((k // RTCM_MAX) << 1) | (seq << 3), "len": len(part), "data": list(part)})
    if len(data) % RTCM_MAX == 0:                       # a fragmented message ends with a short (here empty) part
        out.append({"flags": 1 | ((len(data) // RTCM_MAX) << 1) | (seq << 3), "len": 0, "data": []})
    return out[:RTCM_FRAGS]


def split_messages(data):
    """The UBX messages of a blob, each as its own bytes (so each goes into its own GPS_RTCM_DATA)."""
    msgs = []
    for cls, mid, p in parse_ubx(data):
        msgs.append(ubx(cls, mid, p))
    return msgs


def boat_fix(telem, fresh, max_unc_m=200.0):
    """(lat, lon, accuracy m) from the AURIX telemetry, only with fresh telemetry, a GPS position source
    (pos_source 1 = GPS trusted) and a sane uncertainty; else None."""
    if not fresh or not telem:
        return None
    if telem.get("pos_source") != 1:
        return None
    lat, lon, unc = telem.get("est_lat"), telem.get("est_lon"), telem.get("unc_m")
    try:
        lat, lon, unc = float(lat), float(lon), float(unc if unc is not None else 50.0)
    except (TypeError, ValueError):
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0) or (lat == 0.0 and lon == 0.0) or not unc <= max_unc_m:
        return None
    return lat, lon, unc
