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
T_JOYMAP_GET = 0x1B        # -> JOYMAP_VALUE
T_JOYMAP_SET = 0x1C        # joymap body + u8 save; STANDBY only
T_JOY_RAW_GET = 0x1D       # -> JOY_RAW (USB bridge frame, for the button mapping)
T_GW_STATUS = 0x31         # gateway -> AURIX, 1 Hz (also the gateway heartbeat)
T_GW_CMD = 0x32            # gateway -> AURIX: a station's command in an envelope
T_HELM = 0x33              # u8 0 take / 1 release
T_GW_STATION = 0x34        # gateway -> AURIX: station joined / left
T_GW_RJOY = 0x36           # gateway -> AURIX: remote joystick packet (30 Hz, no answer)
T_REMOTE = 0x37            # u8 0 ask for remote driving (then CONFIRM on board) / 1 release
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
T_SIM_BRIDGE = 0x73     # simulator only: the USB joysticks behind the simulated bridge
# boat -> PC
T_TELEMETRY = 0x80
T_ACK = 0x81
T_PARAM_VALUE = 0x82
T_PROG_WP_VALUE = 0x83
T_PROG_ERROR = 0x84
T_JOYMAP_VALUE = 0x85
T_JOY_RAW = 0x86
T_LOG_CHUNK = 0x89         # AURIX -> gateway: the full log copy
T_TIME = 0x8A              # AURIX -> gateway: GPS time, 1 Hz
T_TELEM2 = 0x8B            # with every telemetry: gateway / helm / remote / slamming
T_GW_ACK = 0x8C            # AURIX -> gateway: the answer to a station's command

TYPE_NAMES = {
    T_HEARTBEAT: "heartbeat", T_SET_MODE: "set mode",
    T_HEADING_ADJ: "heading", T_SEA_STATE: "sea state", T_TRAINING: "training", T_PARAM_GET: "param read",
    T_PARAM_SET: "param write", T_JOY_CAL: "joystick calibration", T_ALARM_ACK: "alarm ack",
    T_PROG_BEGIN: "program begin", T_PROG_WP: "program waypoint", T_PROG_COMMIT: "program upload",
    T_PROG_GET: "program read", T_AUTO_START: "AUTO start", T_AUTO_PAUSE: "AUTO pause",
    T_AUTO_RESUME: "AUTO resume", T_AUTO_SKIP: "AUTO skip",
    T_POWER_CODE: "power code", T_POWER_RAISE: "power raise", T_POWER_LOWER: "power lower",
    T_SIM_JOYSTICK: "sim joystick", T_SIM_GPS: "sim GPS", T_SIM_BUTTON: "sim button",
    T_JOYMAP_GET: "joystick map read", T_JOYMAP_SET: "joystick map write", T_JOY_RAW_GET: "joystick raw read",
    T_SIM_BRIDGE: "sim USB joystick", T_JOYMAP_VALUE: "joystick map", T_JOY_RAW: "joystick raw",
    T_GW_STATUS: "gateway status", T_LOG_CHUNK: "log chunk", T_TIME: "GPS time", T_TELEM2: "telemetry 2",
    T_GW_CMD: "station command", T_HELM: "HELM", T_GW_STATION: "station event", T_GW_ACK: "station answer",
    T_GW_RJOY: "remote joystick", T_REMOTE: "remote driving",
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
    18: "button map refused: two functions share a button",
    19: "button map refused: TAKE, AUTO and CONFIRM must be mapped",
    20: "button map refused: bad code or neutral zone",
    21: "NOT HELM: take HELM first",
    22: "another station holds HELM (only a station on the boat can take it)",
    # gateway refusals (gateway/stations.py)
    100: "VIEWER: watch only", 101: "duplicate command", 102: "command out of order", 103: "stale command",
    104: "timestamp in the future", 105: "rate limited", 106: "AURIX not connected", 107: "wrong session",
    108: "bad command",
}

MODES = ["STANDBY", "MANUAL", "HDG_LOCK", "AUTO", "TRAINING", "REMOTE_MANUAL"]
MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING, MODE_REMOTE = range(6)
AUTO_STATES = ["running", "PAUSED", "STOPPED AT WAYPOINT", "HOLDING (fallback)"]
CONFIRM_KINDS = ["", "AUTO START", "AUTO RESUME", "POWER RAISE FWD", "POWER RAISE REV", "REMOTE DRIVING"]
REMOTE_STATES = ["off", "DRIVING", "LINK LOST"]
REMOTE_LOST_WHY = ["", "no data (gap > 300 ms)", "data too old", "latency too high"]
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
JOY_SOURCES = ["analog (wired)", "USB bridge"]
# joystick fault / problem reasons (joystick.h JOY_WHY_*)
JOY_WHY = {0: "", 1: "X wire break / jump", 2: "Y wire break / jump", 3: "no frames from the bridge (>150 ms)",
           4: "too many CRC errors", 5: "bridge counter stuck", 6: "STICK UNPLUGGED", 7: "THROTTLE UNPLUGGED",
           8: "axis invalid / out of range", 9: "bridge in learn mode"}
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
    (0x1000, "NOT NEUTRAL AT START (outputs not armed)"),
    (0x2000, "SAFETY MONITOR TRIPPED"),
    (0x4000, "CONTROL LOOP OVERRUN"),
    (0x8000, "LOG FAILURE (SD card)"),
    (0x10000, "GATEWAY LOST"),
    (0x20000, "GATEWAY STORAGE"),
    (0x40000, "REMOTE LINK LOST"),
    (0x80000, "CHANNEL FALLBACK"),
    (0x100000, "REMOTE DRIVING LINK LOST"),
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
    "<I BBBB fffff BB iiff BB ffiiff fff I f ff BBB BBB f iif HH BBBBBH ffHBHBf HHHHHHHBB BHH BBBHBHHIIBBB")
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
    "power_fwd power_rev power_code power_code_dir power_code_left_ds power_flags throttle_demand "
    "t_core0_avg t_core0_max t_mon_avg t_mon_max t_log_avg t_log_max ovr_core0 ovr_mon ovr_log "
    "log_flags log_files log_dropped "
    "joy_source usb_flags usb_fps usb_crc joy_why lever_x1000 pickup_x1000 usb_btn0 usb_btn1 usb_hats "
    "nz_lo_pc nz_hi_pc").split()
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
    u = d["usb_flags"]
    d["usb"] = d["joy_source"] == 1
    d["usb_dev0"] = bool(u & 0x01)
    d["usb_dev1"] = bool(u & 0x02)
    d["usb_learn"] = bool(u & 0x04)
    d["usb_hub"] = bool(u & 0x08)
    d["has_lever"] = bool(u & 0x10)
    d["pickup"] = bool(u & 0x20)
    d["usb_starting"] = bool(u & 0x40)
    d["lever"] = None if d["lever_x1000"] == 0xFFFF else d["lever_x1000"] / 1000.0
    d["pickup_target"] = None if d["pickup_x1000"] == 0xFFFF else d["pickup_x1000"] / 1000.0
    d["nz_lo"] = d["nz_lo_pc"] / 100.0
    d["nz_hi"] = d["nz_hi_pc"] / 100.0
    d["joy_why_text"] = JOY_WHY.get(d["joy_why"], "reason %d" % d["joy_why"])
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


# --- USB joystick: button map, raw frames ---------------------------------------

# Function order in the map (app/joymap.h jm_button_t)
JM_BUTTONS = ["TAKE", "AUTO", "CONFIRM", "HDG LOCK", "STANDBY", "-1", "+1", "-10", "+10"]
JM_TAKE, JM_AUTO, JM_CONFIRM, JM_HDG_LOCK, JM_STANDBY = range(5)
THR_INPUTS = ["stick Y (spring-centred)", "lever (non-returning)"]
HAT_DIRS = ["UP", "RIGHT", "DOWN", "LEFT"]
DEV_NAMES = ["stick", "throttle"]
AXIS_NAMES = ["X", "Y", "Z", "Rx", "Ry", "Rz", "Slider", "Dial"]
_JOYMAP = struct.Struct("<BBBBBB9Bffff")       # 31 bytes, link.c wr_joymap()


def code_btn(dev, index):
    """Control code of USB button `index` (0 = "button 1") on device dev."""
    return 1 + dev * 32 + index


def code_hat(dev, hat, direction):
    return 65 + dev * 8 + hat * 4 + direction


def code_name(code):
    """Readable name of a control code (app/jsframe.h)."""
    if code == 0:
        return "-"
    if 1 <= code <= 64:
        c = code - 1
        return "%s button %d" % (DEV_NAMES[c // 32], c % 32 + 1)
    if 65 <= code <= 80:
        c = code - 65
        return "%s hat %s" % (DEV_NAMES[c // 8], HAT_DIRS[c % 4])
    return "code %d" % code


def axis_name(code):
    if code >= 16:
        return "-"
    return "%s axis %d (%s)" % (DEV_NAMES[code // 8], code % 8, AXIS_NAMES[code % 8])


def default_joymap(source=0):
    """Same as app/joymap.c joymap_default(): T.16000M + TWCS."""
    return {"source": source, "thr_input": 1, "axis_steer": 0, "axis_thr": 8 + 2, "inv_steer": 0, "inv_thr": 1,
            "btn": [code_btn(0, 0), code_btn(1, 0), code_btn(1, 3), code_btn(1, 2), code_btn(0, 1),
                    code_hat(1, 0, 3), code_hat(1, 0, 1), code_hat(1, 0, 2), code_hat(1, 0, 0)],
            "nz_lo": 0.45, "nz_hi": 0.55, "lever_delta": 0.10, "pickup_win": 0.03}


def check_joymap(m):
    """Same rules as app/joymap.c joymap_check(); returns an error text, empty if OK."""
    b = m["btn"]
    if not (b[JM_TAKE] and b[JM_AUTO] and b[JM_CONFIRM]):
        return "TAKE, AUTO and CONFIRM must be mapped"
    used = [c for c in b if c]
    if len(used) != len(set(used)):
        dup = sorted({c for c in used if used.count(c) > 1})
        names = [JM_BUTTONS[i] for i, c in enumerate(b) if c in dup]
        return "two functions share a button: " + ", ".join(names)
    if m["axis_steer"] == m["axis_thr"]:
        return "steering and throttle use the same axis"
    if not (0.2 <= m["nz_lo"] and m["nz_hi"] <= 0.8 and m["nz_hi"] - m["nz_lo"] >= 0.04):
        return "neutral zone must be inside 20..80 % and at least 4 % wide"
    return ""


def p_joymap_set(m, save=True):
    return _JOYMAP.pack(m["source"], m["thr_input"], m["axis_steer"], m["axis_thr"], m["inv_steer"], m["inv_thr"],
                        *m["btn"], m["nz_lo"], m["nz_hi"], m["lever_delta"], m["pickup_win"]) + bytes([1 if save else 0])


def decode_joymap(payload: bytes) -> dict:
    v = _JOYMAP.unpack_from(payload)
    return {"source": v[0], "thr_input": v[1], "axis_steer": v[2], "axis_thr": v[3], "inv_steer": v[4],
            "inv_thr": v[5], "btn": list(v[6:15]), "nz_lo": v[15], "nz_hi": v[16], "lever_delta": v[17],
            "pickup_win": v[18]}


def decode_joy_raw(payload: bytes) -> dict:
    flags, seq = struct.unpack_from("<BB", payload)
    devs = []
    for d in range(2):
        v = struct.unpack_from("<8hIB", payload, 2 + d * 21)
        devs.append({"axes": [None if a == -32768 else a / 32767.0 for a in v[:8]], "buttons": v[8],
                     "hats": [v[9] & 0x0F, v[9] >> 4]})
    return {"flags": flags, "seq": seq, "dev": devs, "connected": [bool(flags & 1), bool(flags & 2)],
            "learn": bool(flags & 4)}


def pressed_codes(raw):
    """Every control code pressed in a raw frame (buttons and exact hat directions)."""
    out = []
    for d in range(2):
        b = raw["dev"][d]["buttons"]
        out += [code_btn(d, i) for i in range(32) if b & (1 << i)]
        for h in range(2):
            v = raw["dev"][d]["hats"][h]
            if v in (0, 2, 4, 6):
                out.append(code_hat(d, h, v // 2))
    return out


def p_sim_bridge(stick_x=0.0, stick_y=0.0, lever=0.5, b0=0, b1=0, hat0=15, hat1=15,
                 stick=True, throttle=True, learn=False, fault=0):
    """Simulated T.16000M (dev 0) + TWCS (dev 1). lever 0..1; fault bits:
    1 no frames, 2 corrupt every frame, 4 counter stuck, 8 corrupt 1 in 50."""
    def ax(v):
        return int(round(max(-1.0, min(1.0, v)) * 32767))
    none = [-32768] * 8
    a0 = none[:]
    a1 = none[:]
    if stick:
        a0[0], a0[1] = ax(stick_x), ax(stick_y)
    if throttle:
        a1[2] = ax(1.0 - 2.0 * lever)          # TWCS: forward = low raw value
    flags = (1 if stick else 0) | (2 if throttle else 0) | (4 if learn else 0) | 8
    out = struct.pack("<B", flags)
    out += struct.pack("<8hIB", *a0, b0 if stick else 0, 0xF0 | (hat0 & 0x0F))
    out += struct.pack("<8hIB", *a1, b1 if throttle else 0, 0xF0 | (hat1 & 0x0F))
    return out + struct.pack("<B", fault)


# --- gateway (Raspberry Pi) ---------------------------------------------------------

GWF_STORAGE, GWF_STORAGE_FULL, GWF_FALLBACK, GWF_STARLINK, GWF_SAVER = 0x01, 0x02, 0x04, 0x08, 0x10
GW_STATES = ["never connected", "OK", "LOST"]


def p_remote(action):
    """0 ask for remote driving (then the joystick CONFIRM on board), 1 release."""
    return struct.pack("<B", action)


def p_helm(action):
    """0 take, 1 release."""
    return struct.pack("<B", action)


def p_gw_status(flags, channel=0, stations=0, pi_log_seq=0, pi_log_gaps=0):
    """GW_STATUS (gateway -> AURIX, 1 Hz): link.c gw_status()."""
    return struct.pack("<BBBBIHH", 1, flags & 0xFF, channel & 0xFF, stations & 0xFF,
                       pi_log_seq & 0xFFFFFFFF, min(pi_log_gaps, 0xFFFF), 0)


def decode_time(payload: bytes) -> dict:
    valid, date, tod, t_ms = struct.unpack_from("<BIII", payload)
    return {"valid": bool(valid), "date": date, "tod_ms": tod, "t_ms": t_ms}


def gps_epoch(date, tod_ms):
    """YYYYMMDD + UTC ms of the day -> Unix time (s)."""
    import calendar
    y, m, d = date // 10000, (date // 100) % 100, date % 100
    return calendar.timegm((y, m, d, 0, 0, 0, 0, 0, 0)) + tod_ms / 1000.0


def decode_log_chunk(payload: bytes):
    seq, off, ln = struct.unpack_from("<IHH", payload)
    return seq, off, ln, payload[8:]


# TELEM2 groups (link.c telemetry2); each group only if the frame is long enough
_T2_GROUPS = [
    (struct.Struct("<IBBBBIIIHI"), "t2_ms gw_state gw_flags gw_channel gw_stations log_sent_seq "
                                   "log_dropped_pi pi_log_seq pi_log_gaps log_skipped".split()),
    (struct.Struct("<BBH"), "helm helm_flags helm_changes".split()),
    (struct.Struct("<BBHHhHBHBBIB"), "remote_state remote_station rjoy_age_ms rjoy_lat_ms rjoy_x1000 rjoy_lever1000 "
                                     "rjoy_flags rjoy_pickup1000 remote_cap_f remote_cap_r rjoy_packets "
                                     "remote_lost_why".split()),
]


def decode_telem2(payload: bytes) -> dict:
    d, off = {}, 0
    for st, names in _T2_GROUPS:
        if len(payload) < off + st.size:
            break
        d.update(zip(names, st.unpack_from(payload, off)))
        off += st.size
    if "gw_state" in d:
        d["gw_state_name"] = GW_STATES[d["gw_state"]] if d["gw_state"] < 3 else "?"
    return d
