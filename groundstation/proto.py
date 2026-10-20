"""Trygons C2 ground-station protocol. Must match app/proto.h and app/link.c.

Frame:  0xAA 0x55 | len | type | seq | payload[len] | crc_lo crc_hi
CRC-16/CCITT-FALSE over len, type, seq, payload. Little-endian numbers.
"""
import struct

SYNC = b"\xAA\x55"
MAX_PAYLOAD = 200
OVERHEAD = 7

# PC -> boat
T_HEARTBEAT = 0x01
T_SET_MODE = 0x10          # only STANDBY and TRAINING
T_HEADING_ADJ = 0x12       # HDG_LOCK only
T_SEA_STATE = 0x15
T_TRAINING = 0x16
T_PARAM_GET = 0x17
T_PARAM_SET = 0x18
T_JOY_CAL = 0x19
T_ALARM_ACK = 0x1A
T_PROG_BEGIN = 0x20
T_PROG_WP = 0x21
T_PROG_COMMIT = 0x22
T_PROG_GET = 0x23
T_AUTO_START = 0x24        # then joystick CONFIRM within 10 s
T_AUTO_PAUSE = 0x25
T_AUTO_RESUME = 0x26       # then joystick CONFIRM within 10 s
T_AUTO_SKIP = 0x27
T_POWER_CODE = 0x28        # u8 dir (0 fwd, 1 rev): the boat shows a code
T_POWER_RAISE = 0x29       # u8 dir, u16 typed code; then joystick CONFIRM within 10 s
T_POWER_LOWER = 0x2A       # back to 30 % / 5 % at once
T_SIM_JOYSTICK = 0x70   # simulator only
T_SIM_GPS = 0x71        # simulator only
T_SIM_BUTTON = 0x72     # simulator only: panel button
# boat -> PC
T_TELEMETRY = 0x80
T_ACK = 0x81
T_PARAM_VALUE = 0x82
T_PROG_WP_VALUE = 0x83
T_PROG_ERROR = 0x84

TYPE_NAMES = {
    T_HEARTBEAT: "heartbeat", T_SET_MODE: "set mode",
    T_HEADING_ADJ: "heading", T_SEA_STATE: "sea state", T_TRAINING: "training", T_PARAM_GET: "param read",
    T_PARAM_SET: "param write", T_JOY_CAL: "joystick calibration", T_ALARM_ACK: "alarm ack",
    T_PROG_BEGIN: "program begin", T_PROG_WP: "program waypoint", T_PROG_COMMIT: "program upload",
    T_PROG_GET: "program read", T_AUTO_START: "AUTO start", T_AUTO_PAUSE: "AUTO pause",
    T_AUTO_RESUME: "AUTO resume", T_AUTO_SKIP: "AUTO skip",
    T_POWER_CODE: "power code", T_POWER_RAISE: "power raise", T_POWER_LOWER: "power lower",
    T_SIM_JOYSTICK: "sim joystick", T_SIM_GPS: "sim GPS", T_SIM_BUTTON: "sim button",
    T_TELEMETRY: "telemetry", T_ACK: "ack", T_PARAM_VALUE: "param value",
    T_PROG_WP_VALUE: "program waypoint", T_PROG_ERROR: "program error",
}

RESULTS = {
    0: "OK",
    1: "wrong length",
    2: "unknown command",
    3: "value out of range",
    4: "JOYSTICK IS IN CONTROL",
    5: "not allowed (mode table)",
    6: "conditions not met (GPS / waypoint / sensors)",
    7: "not possible in this mode",
    8: "stick not centred",
    9: "only in STANDBY",
    10: "no such parameter",
    11: "refused",
    12: "program refused",
    13: "no program stored",
    14: "joystick not ready (missing / fault / not armed): no CONFIRM possible",
    15: "wrong or expired code (ask for a new one)",
    16: "already at the highest step",
    17: "acknowledge the alarm first",
}

MODES = ["STANDBY", "MANUAL", "HDG_LOCK", "AUTO", "TRAINING"]
MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING = range(5)
AUTO_STATES = ["running", "PAUSED", "STOPPED AT WAYPOINT", "HOLDING (fallback)"]
CONFIRM_KINDS = ["", "AUTO START", "AUTO RESUME", "POWER RAISE FWD", "POWER RAISE REV"]
WP_ACTIONS = ["continue", "slow", "stop"]
PROG_END = ["stop + alarm", "hold heading slow"]
PROG_ERRORS = ["ok", "bad number of waypoints", "bad coordinates", "leg speed out of range",
               "arrival radius out of range", "bad action", "leg too short", "leg too long", "bad checksum"]
BTN_STANDBY, BTN_MINUS_1, BTN_PLUS_1, BTN_MINUS_10, BTN_PLUS_10, BTN_HDG_LOCK = range(6)
AP_MODES = ["STANDBY", "AUTO", "TRAINING", "MANUAL", "GYRO HOLD", "CENTER", "ANGLE"]
GEARS = ["N", "F", "R"]
POS_SOURCES = ["NO FIX", "GPS", "DEAD RECKONING", "RECOVERING"]
DR_REASONS = ["-", "GPS lost", "spoof/jam flag", "position jump", "impossible speed",
              "course mismatch", "slow drag"]
JOY_STATUS = ["not fitted", "NOT CENTRED / not armed", "armed", "FAULT"]
TRAIN_STAGES = ["spin-up", "running", "result ready"]
TRAIN_PHASES = ["idle", "center", "step", "gap", "end right", "end left", "sweep",
                "return", "fit", "result", "aborted"]
TRAIN_RESULTS = ["ok", "aborted by user", "compass lost", "turned too far", "too long",
                 "turning too fast", "no response", "motor reversed", "no end stop",
                 "not centred", "implausible", "rejected"]

ALARMS = [
    (0x001, "COMPASS LOST"),
    (0x002, "GPS LOST"),
    (0x004, "GPS SUSPECT (spoof/jump)"),
    (0x008, "DEAD RECKONING LIMIT"),
    (0x010, "ARRIVAL UNCERTAIN"),
    (0x020, "ARRIVED"),
    (0x040, "GEAR FAULT"),
    (0x080, "TRAINING"),
    (0x100, "JOYSTICK FAULT"),
    (0x200, "JOYSTICK NOT CENTRED"),
    (0x400, "JOYSTICK TOOK CONTROL"),
    (0x800, "PC LINK LOST"),
]
ALM_PC_LINK = 0x800


def alarm_texts(bits):
    return [name for bit, name in ALARMS if bits & bit]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(ftype: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([len(payload), ftype & 0xFF, seq & 0xFF]) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


class Frame:
    __slots__ = ("type", "seq", "payload")

    def __init__(self, ftype, seq, payload):
        self.type, self.seq, self.payload = ftype, seq, payload

    def raw(self):
        return encode(self.type, self.seq, self.payload)


class Receiver:
    """Byte stream -> frames. Same rules as proto_rx_next(): a bad CRC drops
    only the first byte and rescans; stale() drops a byte of an incomplete
    frame that waited too long (corrupted length)."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.junk = 0

    def push(self, data: bytes):
        self.buf += data
        if len(self.buf) > 65536:          # never grows without bound
            del self.buf[: len(self.buf) - 65536]

    def pending(self):
        return len(self.buf)

    def stale(self):
        if self.buf:
            del self.buf[0]
            self.crc_errors += 1

    def next(self):
        b = self.buf
        while True:
            if len(b) < 2:
                return None
            if b[0] != 0xAA or b[1] != 0x55:
                i = b.find(SYNC, 1)
                skip = i if i > 0 else len(b) - 1
                del b[:skip]
                self.junk += skip
                continue
            if len(b) < 5:
                return None
            n = b[2]
            if n > MAX_PAYLOAD:
                del b[0]
                self.crc_errors += 1
                continue
            total = n + OVERHEAD
            if len(b) < total:
                return None
            body = bytes(b[2:5 + n])
            crc = b[5 + n] | (b[6 + n] << 8)
            if crc != crc16(body):
                del b[0]
                self.crc_errors += 1
                continue
            f = Frame(b[3], b[4], bytes(b[5:5 + n]))
            del b[:total]
            self.frames += 1
            return f

    def frames_from(self, data: bytes):
        self.push(data)
        out = []
        while True:
            f = self.next()
            if f is None:
                return out
            out.append(f)


# --- payload builders (PC -> boat) -------------------------------------------

def p_set_mode(mode):
    return struct.pack("<B", mode)


def p_heading(delta):
    if delta not in (-10, -1, 1, 10):
        raise ValueError("heading step must be -10, -1, +1 or +10")
    return struct.pack("<b", delta)


def p_sea_state(ss):
    return struct.pack("<B", ss)


def p_training(action, full_angle=0.0):
    """action: 0 start, 1 accept, 2 reject."""
    return struct.pack("<Bf", action, full_angle)


def p_param_get(pid):
    return struct.pack("<B", pid)


def p_param_set(pid, value):
    return struct.pack("<Bf", pid, value)


def p_joy_cal(action):
    """0 start, 1 finish."""
    return struct.pack("<B", action)


def p_prog_begin(n, final_action=0):
    return struct.pack("<BB", n, final_action)


def p_prog_wp(index, lat, lon, kn, radius_m, action):
    return struct.pack("<BiiffB", index, round(lat * 1e7), round(lon * 1e7), kn, radius_m, action)


def p_sim_joystick(fitted, x, y, take=False, autob=False, confirm=False):
    return struct.pack("<BffBBB", 1 if fitted else 0, x, y, 1 if take else 0, 1 if autob else 0,
                       1 if confirm else 0)


def p_power_code(direction):
    return struct.pack("<B", direction)


def p_power_raise(direction, code):
    return struct.pack("<BH", direction, code)


def p_sim_button(button):
    return struct.pack("<B", button)


def p_sim_gps(fail):
    return struct.pack("<B", 1 if fail else 0)


# --- decoders (boat -> PC) ---------------------------------------------------

_TELEM = struct.Struct(
    "<I BBBB fffff BB iiff BB ffiiff fff I f ff BBB BBB f iif HH BBBBBH ffHBHBf")
TELEM_FIELDS = (
    "t_ms mode ap_mode sea_state flags "
    "heading target yaw_rate outboard throttle "
    "gear gps_fix gps_lat gps_lon gps_sog gps_cog "
    "pos_source dr_reason unc_m dr_time_s est_lat est_lon est_sog est_cog "
    "set_deg drift_kn current_conf alarm_bits motor_current "
    "joy_x joy_y joy_buttons joy_status joy_fault_axis "
    "train_stage train_phase train_result "
    "speed_set_kn wp_lat wp_lon wp_kn rx_frames rx_crc_errors "
    "prog_n auto_wp auto_state auto_flags confirm_kind confirm_left_ds "
    "power_fwd power_rev power_code power_code_dir power_code_left_ds power_flags throttle_demand").split()
TELEM_SIZE = _TELEM.size


def decode_telemetry(payload: bytes) -> dict:
    if len(payload) < TELEM_SIZE:
        raise ValueError("telemetry too short: %d < %d" % (len(payload), TELEM_SIZE))
    d = dict(zip(TELEM_FIELDS, _TELEM.unpack_from(payload)))
    for k in ("gps_lat", "gps_lon", "est_lat", "est_lon", "wp_lat", "wp_lon"):
        d[k] *= 1e-7
    f = d["flags"]
    d["compass_ok"] = bool(f & 0x01)
    d["gps_ok"] = bool(f & 0x02)
    d["alarm"] = bool(f & 0x04)
    d["prog_ok"] = bool(f & 0x08)
    d["wp_set"] = d["prog_ok"] and d["wp_lat"] != 0.0
    d["pc_link_alarm"] = bool(f & 0x10)
    d["stopping"] = bool(f & 0x20)
    d["joy_take"] = bool(d["joy_buttons"] & 1)
    d["joy_auto"] = bool(d["joy_buttons"] & 2)
    d["joy_confirm"] = bool(d["joy_buttons"] & 4)
    d["prog_suspended"] = bool(d["auto_flags"] & 2)
    d["prog_finished"] = bool(d["auto_flags"] & 4)
    d["confirm_left_s"] = d["confirm_left_ds"] / 10.0
    d["power_code_left_s"] = d["power_code_left_ds"] / 10.0
    d["speed_limited"] = bool(d["power_flags"] & 1)
    d["power_raised"] = bool(d["power_flags"] & 2)
    d["mode_name"] = MODES[d["mode"]] if d["mode"] < len(MODES) else "?"
    return d


def decode_ack(payload: bytes) -> dict:
    t, s, r = struct.unpack_from("<BBB", payload)
    return {"cmd": t, "seq": s, "result": r, "ok": r == 0,
            "text": RESULTS.get(r, "error %d" % r), "cmd_name": TYPE_NAMES.get(t, hex(t))}


def decode_prog_wp(payload: bytes) -> dict:
    i, n, fin, lat, lon, kn, r, act = struct.unpack_from("<BBBiiffB", payload)
    return {"index": i, "n": n, "final": fin, "lat": lat * 1e-7, "lon": lon * 1e-7, "kn": kn,
            "radius": r, "action": act}


def decode_prog_error(payload: bytes) -> dict:
    e, idx = struct.unpack_from("<BB", payload)
    return {"error": e, "index": idx, "text": PROG_ERRORS[e] if e < len(PROG_ERRORS) else "?"}


def decode_param(payload: bytes) -> dict:
    pid, v, lo, hi = struct.unpack_from("<Bfff", payload)
    name = payload[13:29].split(b"\0", 1)[0].decode("ascii", "replace")
    return {"id": pid, "value": v, "min": lo, "max": hi, "name": name}
