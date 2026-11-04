"""An OWN minimal MAVLink 2 codec (docs/uav_design.md 4.2, W64: no new dependency; pymavlink (LGPL-3.0) only as an
optional cross-check in gateway/test_uav.py UA2). It knows ONLY the messages of the allow-list (allow.py) plus a few
it must be able to recognise to DROP them by name (they are never encoded by the router: allow.py refuses them).

Frame (MAVLink 2): 0xFD len incompat compat seq sysid compid msgid(3, LE) payload(len) crc(2) [signature(13)]
  - the payload = the fields in WIRE order (largest type first, extensions last), trailing zero bytes cut;
  - crc = X.25 (CRC-16/MCRF4XX) over len .. payload + the message's CRC_EXTRA;
  - signature (incompat bit 0): link_id(1) timestamp(6, 10 us since 2015-01-01) sha256(key + frame + link_id +
    timestamp)[:6].
The field tables below are facts of the official MAVLink XML (common.xml / ardupilotmega.xml: wire order, types,
CRC_EXTRA); UA2 compares every one with pymavlink when it is installed.

Only gateway/uav/router_worker.py (through allow.py) and the simulator gateway/uav/simdrone.py may call encode();
no other module builds MAVLink frames (UA4 checks it, like the servo.c writer check of the firmware).
"""
import hashlib
import struct

STX = 0xFD
HEADER_LEN = 10
SIG_LEN = 13
F_SIGNED = 0x01

# name: (msgid, crc_extra, [(field, type, count)] in wire order)  - type = struct code ("s" = char[count])
MESSAGES = {
    "HEARTBEAT": (0, 50, [("custom_mode", "I", 1), ("type", "B", 1), ("autopilot", "B", 1), ("base_mode", "B", 1),
                          ("system_status", "B", 1), ("mavlink_version", "B", 1)]),
    "SYS_STATUS": (1, 124, [("onboard_control_sensors_present", "I", 1), ("onboard_control_sensors_enabled", "I", 1),
                            ("onboard_control_sensors_health", "I", 1), ("load", "H", 1), ("voltage_battery", "H", 1),
                            ("current_battery", "h", 1), ("drop_rate_comm", "H", 1), ("errors_comm", "H", 1),
                            ("errors_count1", "H", 1), ("errors_count2", "H", 1), ("errors_count3", "H", 1),
                            ("errors_count4", "H", 1), ("battery_remaining", "b", 1),
                            ("onboard_control_sensors_present_extended", "I", 1),
                            ("onboard_control_sensors_enabled_extended", "I", 1),
                            ("onboard_control_sensors_health_extended", "I", 1)]),
    "SYSTEM_TIME": (2, 137, [("time_unix_usec", "Q", 1), ("time_boot_ms", "I", 1)]),
    "SET_MODE": (11, 89, [("custom_mode", "I", 1), ("target_system", "B", 1), ("base_mode", "B", 1)]),
    "PARAM_REQUEST_READ": (20, 214, [("param_index", "h", 1), ("target_system", "B", 1), ("target_component", "B", 1),
                                     ("param_id", "s", 16)]),
    "PARAM_REQUEST_LIST": (21, 159, [("target_system", "B", 1), ("target_component", "B", 1)]),
    "PARAM_VALUE": (22, 220, [("param_value", "f", 1), ("param_count", "H", 1), ("param_index", "H", 1),
                              ("param_id", "s", 16), ("param_type", "B", 1)]),
    "PARAM_SET": (23, 168, [("param_value", "f", 1), ("target_system", "B", 1), ("target_component", "B", 1),
                            ("param_id", "s", 16), ("param_type", "B", 1)]),
    "GPS_RAW_INT": (24, 24, [("time_usec", "Q", 1), ("lat", "i", 1), ("lon", "i", 1), ("alt", "i", 1),
                             ("eph", "H", 1), ("epv", "H", 1), ("vel", "H", 1), ("cog", "H", 1), ("fix_type", "B", 1),
                             ("satellites_visible", "B", 1), ("alt_ellipsoid", "i", 1), ("h_acc", "I", 1),
                             ("v_acc", "I", 1), ("vel_acc", "I", 1), ("hdg_acc", "I", 1), ("yaw", "H", 1)]),
    "ATTITUDE": (30, 39, [("time_boot_ms", "I", 1), ("roll", "f", 1), ("pitch", "f", 1), ("yaw", "f", 1),
                          ("rollspeed", "f", 1), ("pitchspeed", "f", 1), ("yawspeed", "f", 1)]),
    "GLOBAL_POSITION_INT": (33, 104, [("time_boot_ms", "I", 1), ("lat", "i", 1), ("lon", "i", 1), ("alt", "i", 1),
                                      ("relative_alt", "i", 1), ("vx", "h", 1), ("vy", "h", 1), ("vz", "h", 1),
                                      ("hdg", "H", 1)]),
    "SERVO_OUTPUT_RAW": (36, 222, [("time_usec", "I", 1)] + [("servo%d_raw" % i, "H", 1) for i in range(1, 9)] +
                         [("port", "B", 1)] + [("servo%d_raw" % i, "H", 1) for i in range(9, 17)]),
    "MISSION_SET_CURRENT": (41, 28, [("seq", "H", 1), ("target_system", "B", 1), ("target_component", "B", 1)]),
    "MISSION_CURRENT": (42, 28, [("seq", "H", 1), ("total", "H", 1), ("mission_state", "B", 1),
                                 ("mission_mode", "B", 1)]),
    "MISSION_REQUEST_LIST": (43, 132, [("target_system", "B", 1), ("target_component", "B", 1),
                                       ("mission_type", "B", 1)]),
    "MISSION_COUNT": (44, 221, [("count", "H", 1), ("target_system", "B", 1), ("target_component", "B", 1),
                                ("mission_type", "B", 1)]),
    "MISSION_CLEAR_ALL": (45, 232, [("target_system", "B", 1), ("target_component", "B", 1),
                                    ("mission_type", "B", 1)]),
    "MISSION_ITEM_REACHED": (46, 11, [("seq", "H", 1)]),
    "MISSION_ACK": (47, 153, [("target_system", "B", 1), ("target_component", "B", 1), ("type", "B", 1),
                              ("mission_type", "B", 1)]),
    "MISSION_REQUEST_INT": (51, 196, [("seq", "H", 1), ("target_system", "B", 1), ("target_component", "B", 1),
                                      ("mission_type", "B", 1)]),
    "MISSION_ITEM_INT": (73, 38, [("param1", "f", 1), ("param2", "f", 1), ("param3", "f", 1), ("param4", "f", 1),
                                  ("x", "i", 1), ("y", "i", 1), ("z", "f", 1), ("seq", "H", 1), ("command", "H", 1),
                                  ("target_system", "B", 1), ("target_component", "B", 1), ("frame", "B", 1),
                                  ("current", "B", 1), ("autocontinue", "B", 1), ("mission_type", "B", 1)]),
    "VFR_HUD": (74, 20, [("airspeed", "f", 1), ("groundspeed", "f", 1), ("alt", "f", 1), ("climb", "f", 1),
                         ("heading", "h", 1), ("throttle", "H", 1)]),
    "COMMAND_INT": (75, 158, [("param1", "f", 1), ("param2", "f", 1), ("param3", "f", 1), ("param4", "f", 1),
                              ("x", "i", 1), ("y", "i", 1), ("z", "f", 1), ("command", "H", 1),
                              ("target_system", "B", 1), ("target_component", "B", 1), ("frame", "B", 1),
                              ("current", "B", 1), ("autocontinue", "B", 1)]),
    "COMMAND_LONG": (76, 152, [("param1", "f", 1), ("param2", "f", 1), ("param3", "f", 1), ("param4", "f", 1),
                               ("param5", "f", 1), ("param6", "f", 1), ("param7", "f", 1), ("command", "H", 1),
                               ("target_system", "B", 1), ("target_component", "B", 1), ("confirmation", "B", 1)]),
    "COMMAND_ACK": (77, 143, [("command", "H", 1), ("result", "B", 1), ("progress", "B", 1),
                              ("result_param2", "i", 1), ("target_system", "B", 1), ("target_component", "B", 1)]),
    "SET_POSITION_TARGET_LOCAL_NED": (84, 143, [("time_boot_ms", "I", 1), ("x", "f", 1), ("y", "f", 1), ("z", "f", 1),
                                                ("vx", "f", 1), ("vy", "f", 1), ("vz", "f", 1), ("afx", "f", 1),
                                                ("afy", "f", 1), ("afz", "f", 1), ("yaw", "f", 1),
                                                ("yaw_rate", "f", 1), ("type_mask", "H", 1),
                                                ("target_system", "B", 1), ("target_component", "B", 1),
                                                ("coordinate_frame", "B", 1)]),
    "RADIO_STATUS": (109, 185, [("rxerrors", "H", 1), ("fixed", "H", 1), ("rssi", "B", 1), ("remrssi", "B", 1),
                                ("txbuf", "B", 1), ("noise", "B", 1), ("remnoise", "B", 1)]),
    "TIMESYNC": (111, 34, [("tc1", "q", 1), ("ts1", "q", 1)]),
    "GPS_INJECT_DATA": (123, 250, [("target_system", "B", 1), ("target_component", "B", 1), ("len", "B", 1),
                                   ("data", "B", 110)]),
    "BATTERY_STATUS": (147, 154, [("current_consumed", "i", 1), ("energy_consumed", "i", 1), ("temperature", "h", 1),
                                  ("voltages", "H", 10), ("current_battery", "h", 1), ("id", "B", 1),
                                  ("battery_function", "B", 1), ("type", "B", 1), ("battery_remaining", "b", 1),
                                  ("time_remaining", "i", 1), ("charge_state", "B", 1), ("voltages_ext", "H", 4),
                                  ("mode", "B", 1), ("fault_bitmask", "I", 1)]),
    "AUTOPILOT_VERSION": (148, 178, [("capabilities", "Q", 1), ("uid", "Q", 1), ("flight_sw_version", "I", 1),
                                     ("middleware_sw_version", "I", 1), ("os_sw_version", "I", 1),
                                     ("board_version", "I", 1), ("vendor_id", "H", 1), ("product_id", "H", 1),
                                     ("flight_custom_version", "B", 8), ("middleware_custom_version", "B", 8),
                                     ("os_custom_version", "B", 8), ("uid2", "B", 18)]),
    "MOUNT_CONTROL": (157, 21, [("input_a", "i", 1), ("input_b", "i", 1), ("input_c", "i", 1),
                                ("target_system", "B", 1), ("target_component", "B", 1), ("save_position", "B", 1)]),
    "EKF_STATUS_REPORT": (193, 71, [("velocity_variance", "f", 1), ("pos_horiz_variance", "f", 1),
                                    ("pos_vert_variance", "f", 1), ("compass_variance", "f", 1),
                                    ("terrain_alt_variance", "f", 1), ("flags", "H", 1),
                                    ("airspeed_variance", "f", 1)]),
    "EXTENDED_SYS_STATE": (245, 130, [("vtol_state", "B", 1), ("landed_state", "B", 1)]),
    "GPS_RTCM_DATA": (233, 35, [("flags", "B", 1), ("len", "B", 1), ("data", "B", 180)]),
    "HOME_POSITION": (242, 104, [("latitude", "i", 1), ("longitude", "i", 1), ("altitude", "i", 1), ("x", "f", 1),
                                 ("y", "f", 1), ("z", "f", 1), ("q", "f", 4), ("approach_x", "f", 1),
                                 ("approach_y", "f", 1), ("approach_z", "f", 1), ("time_usec", "Q", 1)]),
    "STATUSTEXT": (253, 83, [("severity", "B", 1), ("text", "s", 50), ("id", "H", 1), ("chunk_seq", "B", 1)]),
}
BY_ID = {v[0]: k for k, v in MESSAGES.items()}
_STRUCTS = {}


def _struct(name):
    s = _STRUCTS.get(name)
    if s is None:
        fmt = "<" + "".join(("%d%s" % (n, t)) if n > 1 or t == "s" else t for _f, t, n in MESSAGES[name][2])
        s = _STRUCTS[name] = struct.Struct(fmt)
    return s


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def body_of(name, fields):
    """The payload bytes (zero-truncated) of message `name` from a dict (missing fields = 0)."""
    vals = []
    for f, t, n in MESSAGES[name][2]:
        v = fields.get(f, b"" if t == "s" else (0 if n == 1 else [0] * n))
        if t == "s":
            vals.append(v.encode("ascii", "replace") if isinstance(v, str) else bytes(v))
        elif n > 1:
            v = list(v)[:n]
            vals.extend(v + [0] * (n - len(v)))
        else:
            vals.append(v)
    b = _struct(name).pack(*vals)
    return b.rstrip(b"\x00") or b"\x00"            # MAVLink 2: trailing zeros cut (at least one byte kept)


def sign_tag(key, frame_wo_sig, link_id, timestamp):
    return hashlib.sha256(key + frame_wo_sig + bytes([link_id]) + timestamp.to_bytes(6, "little")).digest()[:6]


def encode(name, fields, seq, sysid, compid, key=None, link_id=0, timestamp=0):
    """One MAVLink 2 frame (bytes). key (32 bytes) = signed (W64)."""
    msgid, extra, _f = MESSAGES[name]
    p = body_of(name, fields)
    inc = F_SIGNED if key else 0
    hdr = bytes([STX, len(p), inc, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF]) + msgid.to_bytes(3, "little")
    crc = x25(bytes([extra]), x25(hdr[1:] + p))
    frame = hdr + p + struct.pack("<H", crc)
    if key:
        frame += bytes([link_id]) + timestamp.to_bytes(6, "little") + sign_tag(key, frame, link_id, timestamp)
    return frame


class Msg:
    __slots__ = ("name", "msgid", "sysid", "compid", "seq", "fields", "signed", "raw")

    def __init__(self, name, msgid, sysid, compid, seq, fields, signed, raw):
        self.name, self.msgid, self.sysid, self.compid, self.seq = name, msgid, sysid, compid, seq
        self.fields, self.signed, self.raw = fields, signed, raw

    def __getitem__(self, k):
        return self.fields[k]

    def get(self, k, d=None):
        return self.fields.get(k, d)

    def __repr__(self):
        return "<%s %d/%d %s>" % (self.name, self.sysid, self.compid, self.fields)


def decode_body(name, payload):
    s = _struct(name)
    vals = s.unpack(payload[:s.size].ljust(s.size, b"\x00"))
    out, i = {}, 0
    for f, t, n in MESSAGES[name][2]:
        if t == "s":
            out[f] = vals[i].split(b"\x00", 1)[0].decode("ascii", "replace")
            i += 1
        elif n > 1:
            out[f] = list(vals[i:i + n])
            i += n
        else:
            out[f] = vals[i]
            i += 1
    return out


class Parser:
    """Bytes in -> Msg out. Counts what it can not use: bad CRC, unknown message id (no table = no CRC_EXTRA = it can
    not even be checked: dropped, counted), bad / missing signature (when a key is set), MAVLink 1 frames."""

    def __init__(self, key=None):
        self.buf = bytearray()
        self.key = key
        self.n = {"ok": 0, "crc": 0, "unknown": 0, "v1": 0, "unsigned": 0, "badsig": 0, "junk": 0}

    def feed(self, data):
        self.buf += data
        out = []
        b = self.buf
        while True:
            i = b.find(bytes([STX]))
            if i < 0:
                if b and b[0] == 0xFE:
                    self.n["v1"] += 1
                self.n["junk"] += len(b) if b else 0
                b.clear()
                break
            if i:
                if 0xFE in b[:i]:
                    self.n["v1"] += 1
                self.n["junk"] += i
                del b[:i]
            if len(b) < HEADER_LEN:
                break
            ln, inc = b[1], b[2]
            total = HEADER_LEN + ln + 2 + (SIG_LEN if inc & F_SIGNED else 0)
            if len(b) < total:
                break
            frame = bytes(b[:total])
            msgid = int.from_bytes(frame[7:10], "little")
            name = BY_ID.get(msgid)
            if name is None:
                self.n["unknown"] += 1
                del b[:1]                          # (can't check its CRC: resync after the STX)
                continue
            crc = x25(bytes([MESSAGES[name][1]]), x25(frame[1:HEADER_LEN + ln]))
            if crc != struct.unpack_from("<H", frame, HEADER_LEN + ln)[0]:
                self.n["crc"] += 1
                del b[:1]
                continue
            del b[:total]
            signed = bool(inc & F_SIGNED)
            if self.key:
                if not signed:
                    self.n["unsigned"] += 1
                    continue
                base = frame[:HEADER_LEN + ln + 2]
                link_id, ts = frame[-13], int.from_bytes(frame[-12:-6], "little")
                if sign_tag(self.key, base, link_id, ts) != frame[-6:]:
                    self.n["badsig"] += 1
                    continue
            self.n["ok"] += 1
            out.append(Msg(name, msgid, frame[5], frame[6], frame[4],
                           decode_body(name, frame[HEADER_LEN:HEADER_LEN + ln]), signed, frame))
        return out


def param_id(s):
    return (s or "")[:16]
