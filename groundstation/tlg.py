"""Read and write .TLG log files (the boat's SD card logs, and the ground
station's own copy of the telemetry). Format: app/logrec.h.

    import tlg
    log = tlg.read("B00012_001.TLG")
    log.samples      # list of dicts (one per data record), field names from the header
    log.events       # list of (t_ms, text, code, args)
    log.config       # dict from the config record(s) (last one)
    log.pc_commands  # list of (t_ms, type, seq, result, payload)

Every file describes itself in its header (field names, types, scales,
event names), so this reader needs no changes when fields are added.
"""
import struct

import proto

SYNC = 0xA5
HEADER, CONFIG, DATA, EVENT, PCCMD, TEXT = 1, 2, 0x10, 0x20, 0x22, 0x30
SIZES = {"B": 1, "c": 1, "H": 2, "h": 2, "I": 4, "i": 4, "f": 4}
FMT = {"B": "B", "c": "b", "H": "H", "h": "h", "I": "I", "i": "i", "f": "f"}
POS_SOURCES = ["NO FIX", "GPS", "DEAD RECKONING", "RECOVERING"]


class Field:
    __slots__ = ("name", "type", "scale")

    def __init__(self, name, ftype, scale):
        self.name, self.type, self.scale = name, ftype, scale


class Log:
    def __init__(self, path):
        self.path = path
        self.header = {}
        self.fields = []
        self.events_def = {}            # code -> (name, [arg names])
        self.alarm_names = {}
        self.mode_names = []
        self.why_names = []
        self.samples = []
        self.events = []
        self.config = {}
        self.pc_commands = []
        self.texts = []
        self.crc_errors = 0
        self.tail_bytes = 0
        self.records = 0
        self._struct = None

    # --- header -------------------------------------------------------------------
    def _parse_header(self, text):
        for line in text.splitlines():
            if "=" not in line:
                continue
            k, v = line.split("=", 1)
            self.header[k] = v
        self.fields = []
        for item in self.header.get("fields", "").split(","):
            if item.count(":") == 2:
                n, t, sc = item.split(":")
                self.fields.append(Field(n, t, float(sc)))
        self._struct = struct.Struct("<" + "".join(FMT[f.type] for f in self.fields))
        for item in self.header.get("events", "").split(";"):
            if item.count(":") >= 2:
                code, name, args = item.split(":", 2)
                self.events_def[int(code)] = (name, args.split(","))
        for item in self.header.get("alarms", "").split(","):
            if ":" in item:
                bit, name = item.split(":")
                self.alarm_names[int(bit)] = name
        self.mode_names = self.header.get("modes", "").split(",")
        self.why_names = self.header.get("why", "").split(",")

    # --- decoding -----------------------------------------------------------------
    def _decode_data(self, t, payload):
        if self._struct is None or len(payload) < self._struct.size:
            return None
        vals = self._struct.unpack_from(payload)
        d = {"t_ms": t}
        for f, v in zip(self.fields, vals):
            d[f.name] = v if f.type == "f" else (v / f.scale if f.scale != 1 else v)
        return d

    def alarm_text(self, bits):
        return ",".join(n for b, n in sorted(self.alarm_names.items()) if bits & b) or "-"

    def mode_name(self, m):
        return self.mode_names[m] if 0 <= m < len(self.mode_names) else str(m)

    def event_text(self, code, a):
        name, args = self.events_def.get(code, ("EV%d" % code, ["a", "b", "c", "d"]))
        if name == "MODE":
            v = ["ok", "refused (table)", "refused (conditions)"][a[3]] if 0 <= a[3] < 3 else a[3]
            why = self.why_names[a[2]] if 0 <= a[2] < len(self.why_names) else a[2]
            return "MODE %s -> %s (%s) %s" % (self.mode_name(a[0]), self.mode_name(a[1]), why, v)
        if name in ("ALARM_ON", "ALARM_OFF"):
            return "%s %s" % (name, self.alarm_text(a[0] & 0xFFFFFFFF) + (" + autopilot" if name == "ALARM_ON"
                                                                         and a[2] else ""))
        if name == "POWER_LIMIT":
            src, res, att, dr = a[0] & 0xFF, (a[0] >> 8) & 0xFF, (a[0] >> 16) & 0xFF, (a[0] >> 24) & 0xFF
            rev_old, rev_new = (a[3] >> 16) & 0xFFFF, a[3] & 0xFFFF
            return "POWER LIMIT %s src %d result %d: fwd %.0f -> %.0f %%, rev %.0f -> %.0f %%" % (
                "raise attempt (%s)" % ("fwd", "rev")[dr & 1] if att else "change", src, res, a[1] / 10, a[2] / 10,
                rev_old / 10, rev_new / 10)
        if name == "GPS":
            src = POS_SOURCES[a[0]] if 0 <= a[0] < 4 else a[0]
            return "GPS position source %s (reason %d), uncertainty %.1f m, losses %d" % (src, a[1], a[2] / 10, a[3])
        if name == "SLAM":
            st = ["calm", "YELLOW", "RED"][(a[3] >> 16) & 3] if ((a[3] >> 16) & 3) < 3 else "?"
            return "SLAM %s level %.2f g, %d/min, cap %.0f %%%s, %s" % (
                "impact %.2f g" % (a[0] / 100) if a[0] else "change", a[1] / 100, a[2], (a[3] & 0xFFFF) / 10,
                " OVERRIDE" if (a[3] >> 20) & 1 else "", st)
        if name == "NAV":
            kinds = {0: "none", 1: "speed cap", 2: "avoidance heading", 3: "HDG LOCK heading (accepted)"}
            return "NAV %s%s heading %.0f, cap %s, alarm %d, reason %d%s" % (
                kinds.get(a[0] & 0xFF, a[0] & 0xFF), " APPLIED" if (a[0] >> 25) & 1 else "", a[1] / 100.0,
                "-" if a[2] < 0 else "%.1f kn" % (a[2] / 10.0), (a[0] >> 16) & 0xFF, (a[0] >> 8) & 0xFF,
                " LOST (gateway quiet: slow + hold heading)" if (a[0] >> 24) & 1 else "")
        if name == "SAVE":
            return "VIDEO SAVE request %d: %s" % (a[2], proto.save_trigger_text(a[0], a[1]))
        parts = ["%s=%d" % (n, v) for n, v in zip(args, a) if n != "-"]
        return name + " " + " ".join(parts)


def read(path):
    """Read a whole .TLG file. A record cut off at the end (power cut) is
    ignored; records with a bad CRC are skipped."""
    log = Log(path)
    with open(path, "rb") as fh:
        buf = fh.read()
    if buf[:4] != b"TLG1":
        raise ValueError("%s is not a TLG file" % path)
    i, n = 4, len(buf)
    while i + 10 <= n:
        if buf[i] != SYNC:
            log.crc_errors += 1
            i += 1
            continue
        rtype, ln, t = buf[i + 1], buf[i + 2] | (buf[i + 3] << 8), struct.unpack_from("<I", buf, i + 4)[0]
        if i + 10 + ln > n:
            break                                       # cut off at the end
        crc = proto.crc16(bytes(buf[i + 1:i + 8 + ln]))
        if crc != buf[i + 8 + ln] | (buf[i + 9 + ln] << 8):
            log.crc_errors += 1
            i += 1
            continue
        p = bytes(buf[i + 8:i + 8 + ln])
        log.records += 1
        if rtype == HEADER:
            log._parse_header(p.decode("utf-8", "replace"))
        elif rtype == CONFIG:
            for line in p.decode("utf-8", "replace").splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    log.config[k] = v
        elif rtype == DATA:
            d = log._decode_data(t, p)
            if d is not None:
                log.samples.append(d)
        elif rtype == EVENT and ln >= 18:
            code = p[0] | (p[1] << 8)
            args = list(struct.unpack_from("<iiii", p, 2))
            log.events.append((t, log.event_text(code, args), code, args))
        elif rtype == PCCMD and ln >= 4:
            log.pc_commands.append((t, p[0], p[1], p[2], p[4:4 + p[3]]))
        elif rtype == TEXT:
            log.texts.append((t, p.decode("utf-8", "replace")))
        i += 10 + ln
    log.tail_bytes = n - i
    return log


# --- writing (the ground station's own copy of the telemetry) -----------------------

def record(rtype, t_ms, payload):
    body = bytes([rtype, len(payload) & 0xFF, len(payload) >> 8]) + struct.pack("<I", t_ms & 0xFFFFFFFF) + payload
    return bytes([SYNC]) + body + struct.pack("<H", proto.crc16(body))


def telemetry_fields():
    """The telemetry frame described as TLG fields (same layout as the frame)."""
    fmt = proto._TELEM.format.replace(" ", "").lstrip("<")
    out = []
    for name, ch in zip(proto.TELEM_FIELDS, fmt):
        ftype = {"B": "B", "b": "c", "H": "H", "h": "h", "I": "I", "i": "i", "f": "f"}[ch]
        scale = "1e7" if name in ("gps_lat", "gps_lon", "est_lat", "est_lon", "wp_lat", "wp_lon") else "1"
        out.append("%s:%s:%s" % (name, ftype, scale))
    return ",".join(out)


class Writer:
    """Writes telemetry frames as a .TLG file (DATA = the raw telemetry
    payload, described by the header), PC commands and their answers."""

    EVENTS = ("1:BOOT:reset,raw,boot,format;3:ALARM_ON:new_bits,all_bits,ap_alarm,-;"
              "4:ALARM_OFF:cleared_bits,all_bits,-,-;2:MODE:from,to,why,verdict;")

    def __init__(self, path):
        self.f = open(path, "wb")
        self.f.write(b"TLG1")
        hdr = ("TLG header\nformat=1\nsource=ground station telemetry\nfields=%s\nevents=%s\nalarms=%s\nmodes=%s\n" % (
            telemetry_fields(), self.EVENTS, ",".join("%d:%s" % (b, n.replace(" ", "_")) for b, n in proto.ALARMS),
            ",".join(proto.MODES)))
        self.f.write(record(HEADER, 0, hdr.encode()))
        self.last_mode = None
        self.last_alarms = 0

    def telemetry(self, payload, d):
        t = d["t_ms"]
        if self.last_mode is not None and d["mode"] != self.last_mode:
            self.event(t, 2, self.last_mode, d["mode"], 0, 0)
        if d["alarm_bits"] & ~self.last_alarms:
            self.event(t, 3, d["alarm_bits"] & ~self.last_alarms, d["alarm_bits"], 0, 0)
        if self.last_alarms & ~d["alarm_bits"]:
            self.event(t, 4, self.last_alarms & ~d["alarm_bits"], d["alarm_bits"], 0, 0)
        self.last_mode, self.last_alarms = d["mode"], d["alarm_bits"]
        self.f.write(record(DATA, t, bytes(payload)))

    def event(self, t, code, a, b, c, dd):
        self.f.write(record(EVENT, t, struct.pack("<Hiiii", code, a, b, c, dd)))

    def pc_command(self, t, ftype, seq, result, payload):
        self.f.write(record(PCCMD, t, bytes([ftype, seq, result & 0xFF, len(payload)]) + bytes(payload)))

    def flush(self):
        self.f.flush()

    def close(self):
        self.f.close()
