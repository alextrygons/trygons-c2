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
T_MANEUVER = 0x38          # u8 0 ask for MANEUVER (captain; then CONFIRM) / 1 release
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
T_BENCH = 0x2B             # u8 action, u16 arg - BENCH TEST, only from the laptop on the USB cable
T_GENERATOR = 0x2C         # u8 0 off / 1 on: generator auto (HELM station; a copilot's ON needs the captain's ACCEPT)
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
T_BENCH_DIAG = 0x8D        # AURIX -> USB laptop, BENCH TEST only: every sensor (link.c bench_diag)

# Commands that change how the boat is driven: through the gateway a copilot's are only
# sent to the AURIX after the captain's ACCEPT on screen 1 (or in SINGLE-OPERATOR mode
# after the own confirmation). Never STANDBY. PROG_COMMIT = a new AUTO route (the frames
# before it only fill the AURIX's staging area; COMMIT makes it the route).
DRIVING_CMDS = {T_SET_MODE, T_AUTO_START, T_AUTO_PAUSE, T_AUTO_RESUME, T_AUTO_SKIP, T_HEADING_ADJ, T_SEA_STATE,
                T_TRAINING, T_PARAM_SET, T_POWER_CODE, T_PROG_COMMIT, T_GENERATOR}


def is_driving_cmd(ftype, payload=b""):
    if ftype not in DRIVING_CMDS:
        return False
    if ftype == T_GENERATOR:
        return payload[:1] == b"\x01"           # switching the generator ON opens the throttle (in neutral): ACCEPT; OFF is free
    return not (ftype == T_SET_MODE and payload[:1] == bytes([MODE_STANDBY]))


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
    T_GW_RJOY: "remote joystick", T_REMOTE: "remote driving", T_MANEUVER: "maneuver", T_BENCH: "bench test", T_BENCH_DIAG: "bench data",
    T_GENERATOR: "generator auto",
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
    8: "centre the joystick (stick not centred)",
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
    108: "bad command", 116: "no manual licence: REMOTE MANUAL / MANEUVER only for a licensed captain",
    117: "only the captain (or a station on the boat) holds HELM", 118: "captain only: REMOTE MANUAL / MANEUVER",
    119: "no captain holds HELM",
}

MODES = ["STANDBY", "MANUAL", "HDG_LOCK", "AUTO", "TRAINING", "REMOTE_MANUAL", "BENCH_TEST", "MANEUVER"]
# mode change reasons (app/modes.h mode_reason_t)
WHY = ["user", "STANDBY button", "compass fault", "GPS lost", "arrived", "training done", "training abort",
       "test", "gear fault", "DR limit", "arrival uncertain", "joystick", "joystick hand-back", "joystick fault",
       "PC", "HDG LOCK button", "CONFIRM", "not armed", "safety monitor", "remote", "remote end", "remote lost",
       "emergency stop", "bench test ended (idle / link / moving / HELM)", "maneuver",
       "profile fault (joystick appeared, UNCREWED)", "link to the HELM station lost"]
MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING, MODE_REMOTE, MODE_BENCH, MODE_MANEUVER = range(8)
# operating profile (app/smallboat.h SB_PROFILE_)
PROFILES = ["DETECTING", "CREWED", "UNCREWED"]
PROFILE_DETECTING, PROFILE_CREWED, PROFILE_UNCREWED = range(3)

# BENCH TEST (app/smallboat.h SB_BENCH_)
BENCH_ENTER, BENCH_LEAVE, BENCH_LEFT, BENCH_RIGHT, BENCH_BUZZER, BENCH_TRIP = range(6)
BENCH_TRIP_CODE = 0xB3E7
BENCH_REFUSE = ["", "at the end stop in that direction", "a pulse is running / too soon", "time out of range"]
BTN_NAMES = ["STANDBY", "-1", "+1", "-10", "+10", "HDG LOCK"]         # hal_button_t order
IMU_ACC = ["unreliable", "low", "medium", "high"]
GPS_SPOOF = ["unknown", "none", "INDICATED", "MULTIPLE"]
GPS_JAM = ["unknown", "ok", "WARNING", "CRITICAL"]
SD_STATES = ["no card", "power", "reset", "check", "init", "OCR", "file system", "folder scan", "ready", "FAILED"]


def p_bench(action, arg=0):
    return struct.pack("<BH", action, arg)


_BENCH = struct.Struct("<BBIH5f5B2ffBBBII4HB6B2HBbfBI")      # 86 bytes, link.c bench_diag()


def decode_bench_diag(payload: bytes) -> dict:
    v = _BENCH.unpack_from(payload)
    (flags, acc, reports, resets, heading, yaw, roll, pitch, accz, fix_type, sats, spoof, jam, fixf, sog, cog,
     current, limits, estop, sd_state, sd_free, uart_err, thr_us, gear_us, thr_idle_us, gear_n_us,
     seen) = v[:27]
    counts = list(v[27:33])
    pulses, refused, last_refuse, pulse, est, mon, idle_left = v[33:]
    return {
        "imu_ok": bool(flags & 1), "gps_ok": bool(flags & 2), "gps_ubx": bool(flags & 4), "sd_card": bool(flags & 8),
        "sd_mounted": bool(flags & 16), "hal_diag": bool(flags & 32), "compass_ok": bool(flags & 64),
        "gyro_ok": bool(flags & 128), "imu_accuracy": acc, "imu_reports": reports, "imu_resets": resets,
        "heading": heading, "yaw": yaw, "roll": roll, "pitch": pitch, "accel_z": accz,
        "gps_fix_type": fix_type, "sats": sats, "spoof": spoof, "jam": jam, "fix": bool(fixf & 1),
        "jam_spoof": bool(fixf & 2), "sog": sog, "cog": cog, "current": current,
        "limits_fitted": limits != 0x80, "limit_left": limits != 0x80 and bool(limits & 1),
        "limit_right": limits != 0x80 and bool(limits & 2), "estop": estop, "sd_state": sd_state,
        "sd_free_kb": None if sd_free == 0xFFFFFFFF else sd_free, "uart_errors": uart_err,
        "thr_us": thr_us, "gear_us": gear_us, "thr_idle_us": thr_idle_us, "gear_n_us": gear_n_us,
        "buttons_seen": seen, "button_counts": counts, "pulses": pulses, "refused": refused,
        "last_refuse": last_refuse, "pulse": pulse, "estimate": est, "mon_tripped": bool(mon),
        "idle_left_s": idle_left / 1000.0,
    }
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
              "course mismatch", "slow drag", "second GPS disagrees", "invalid fix values"]
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
    (0x200000, "SLAMMING"),
    (0x400000, "AIS RECEIVER SILENT"),
    (0x800000, "COLLISION RISK"),
    (0x1000000, "COLLISION RISK - DRIVER DECISION"),
    (0x2000000, "SHALLOW WATER"),
    (0x4000000, "OBSTACLE AHEAD"),
    (0x8000000, "EMERGENCY STOP"),
    (0x10000000, "PROFILE (CREWED / UNCREWED)"),
    (0x20000000, "LINK LOST (UNCREWED)"),
    (0x40000000, "ENGINE KILLED"),
]
ALM_PC_LINK = 0x800
# the second alarm word (app/smallboat.h SB_ALM2_, TELEM2 group L "alarm_bits2"): the first one is full
ALARMS2 = [
    (0x001, "LOW BATTERY"),
]


def alarm_texts(bits):
    return [name for bit, name in ALARMS if bits & bit]


def alarm_texts2(bits2):
    return [name for bit, name in ALARMS2 if bits2 & bit]


def alarm_texts_all(d):
    """Both words of a telemetry dict (TELEM2 merged in)."""
    return alarm_texts(d.get("alarm_bits", 0)) + alarm_texts2(d.get("alarm_bits2", 0))


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
JM_BUTTONS = ["TAKE", "AUTO", "CONFIRM", "HDG LOCK", "STANDBY", "-1", "+1", "-10", "+10", "SAVE"]
JM_TAKE, JM_AUTO, JM_CONFIRM, JM_HDG_LOCK, JM_STANDBY = range(5)
THR_INPUTS = ["stick Y (spring-centred)", "lever (non-returning)"]
HAT_DIRS = ["UP", "RIGHT", "DOWN", "LEFT"]
DEV_NAMES = ["stick", "throttle"]
AXIS_NAMES = ["X", "Y", "Z", "Rx", "Ry", "Rz", "Slider", "Dial"]
_JOYMAP = struct.Struct("<BBBBBB10Bffff")      # 32 bytes, link.c wr_joymap()


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
                    code_hat(1, 0, 3), code_hat(1, 0, 1), code_hat(1, 0, 2), code_hat(1, 0, 0), code_btn(0, 2)],
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
            "inv_thr": v[5], "btn": list(v[6:16]), "nz_lo": v[16], "nz_hi": v[17], "lever_delta": v[18],
            "pickup_win": v[19]}


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
GWF_AIS_SILENT = 0x20
T_GW_GPS2 = 0x39
T_GW_NAVREQ = 0x3A
T_GW_SHORE = 0x3B          # gateway -> AURIX, 1 Hz: distance to the shore (UNCREWED lost-link rule)
NAV_NONE, NAV_CAP, NAV_HEADING, NAV_HDG_SET = 0, 1, 2, 3


def p_gw_navreq(seq, kind, reason=0, alarm=0, heading=None, cap_kn=None, lease_ms=3000):
    """Navigation request for the AURIX (collision avoidance / shallow water), link.c gw_navreq()."""
    return struct.pack("<BBBBHHH", seq & 0xFF, kind, reason, alarm,
                       0 if heading is None else int(round((heading % 360.0) * 100)) % 36000,
                       0xFFFF if cap_kn is None else max(0, min(600, int(round(cap_kn * 10)))), lease_ms)


def p_gw_shore(flags, dist_m):
    """GW_SHORE (gateway -> AURIX, 1 Hz): b0 near shore, b1 valid; distance m (0xFFFF unknown)."""
    return struct.pack("<BH", flags & 0xFF, max(0, min(0xFFFF, int(dist_m))))


def p_gw_gps2(lat, lon, age_ms):
    """Second GPS (AIS device) for the AURIX's GPS integrity check."""
    return struct.pack("<iiH", int(round(lat * 1e7)), int(round(lon * 1e7)), max(0, min(65535, int(age_ms))))
GW_STATES = ["never connected", "OK", "LOST"]


def p_remote(action):
    """0 ask for remote driving (then the joystick CONFIRM on board), 1 release."""
    return struct.pack("<B", action)


def p_helm(action):
    """0 take, 1 release."""
    return struct.pack("<B", action)


def p_helm_hand(station, local=False):
    """HELM action 2: the holder hands HELM to another gateway station in one step (role swap)."""
    return struct.pack("<BBB", 2, station, 1 if local else 0)


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
    (struct.Struct("<HBBIB"), "save_seq save_trig save_arg save_age_ms save_kinds".split()),
    (struct.Struct("<HHHBBHIB"), "slam_peak100 slam_level100 slam_rate slam_cap slam_flags slam_limit100 "
                                 "slam_impacts slam_yellow_pc".split()),
    (struct.Struct("<BBBBHHBH"), "nav_kind nav_flags nav_reason nav_alarm nav_hdg100 nav_cap10 nav_seq "
                                 "nav_age_ms".split()),
    (struct.Struct("<hhH"), "att_roll100 att_pitch100 att_age_ms".split()),
    (struct.Struct("<BBB"), "cfg_flags estop_state moving_ahead".split()),
    (struct.Struct("<BBBH"), "profile probe_flags shore_flags profile_changes".split()),
    (struct.Struct("<BBHHHH"), "ll_state ll_flags coast_m coast_age_ms ll_left10 near_shore_m".split()),
    (struct.Struct("<IHHBBB"), "alarm_bits2 bank_v100 bank_fast100 batt_flags gen_state gen_pct".split()),
]
LL_STATES = ["OK", "LINK LOST", "ENGINE KILLED"]
# generator mode (app/generator.h GEN_, TELEM2 group L "gen_state"): the throttle opens in NEUTRAL to charge the bank
GEN_STATES = ["OFF", "READY", "STARTING", "RUNNING", "STOPPING", "REST", "FAULT", "DONE"]

# video SAVE triggers (app/smallboat.h SB_SAVE_)
SAVE_TRIGGERS = {1: "SAVE button (joystick)", 2: "alarm", 3: "fault", 4: "joystick takeover",
                 5: "STANDBY by fault", 6: "slamming"}
FAULT_KINDS = {1: "gear", 2: "compass", 3: "joystick", 4: "safety monitor", 5: "overrun", 6: "log"}


def save_trigger_text(trig, arg):
    t = SAVE_TRIGGERS.get(trig, "trigger %d" % trig)
    if trig == 2:
        names = dict(ALARMS) if arg < 32 else dict(ALARMS2)     # 32 + bit = the second alarm word
        t += " " + names.get(1 << (arg % 32), "bit %d" % arg)
    elif trig == 3:
        t += " " + FAULT_KINDS.get(arg, str(arg))
    elif trig == 4:
        t += " (from %s)" % (MODES[arg] if arg < len(MODES) else arg)
    elif trig == 5:
        t += " (%s)" % (WHY[arg] if arg < len(WHY) else arg)
    return t


def decode_telem2(payload: bytes) -> dict:
    d, off = {}, 0
    for st, names in _T2_GROUPS:
        if len(payload) < off + st.size:
            break
        d.update(zip(names, st.unpack_from(payload, off)))
        off += st.size
    if "gw_state" in d:
        d["gw_state_name"] = GW_STATES[d["gw_state"]] if d["gw_state"] < 3 else "?"
    if "nav_flags" in d:
        d["nav_fresh"] = bool(d["nav_flags"] & 1)
        d["nav_lost"] = bool(d["nav_flags"] & 2)
        d["nav_applied"] = bool(d["nav_flags"] & 4)
        d["nav_heading"] = d["nav_hdg100"] / 100.0
        d["nav_cap_kn"] = None if d["nav_cap10"] == 0xFFFF else d["nav_cap10"] / 10.0
    if "cfg_flags" in d:
        d["first_test"] = bool(d["cfg_flags"] & 1)
        d["nv_reset"] = bool(d["cfg_flags"] & 2)        # a stored record was unusable: defaults in use
        d["nv_fault"] = bool(d["cfg_flags"] & 4)        # the data flash does not work: nothing is saved
        d["nv_pending"] = bool(d["cfg_flags"] & 8)      # a save waits for STANDBY / the flash
        d["estop_name"] = ["not fitted", "released", "PRESSED"][d["estop_state"]] if d["estop_state"] < 3 else "?"
    if "profile" in d:
        d["profile_name"] = PROFILES[d["profile"]] if d["profile"] < 3 else "?"
        d["joy_detected"] = bool(d["probe_flags"] & 1)
        d["shore_fresh"] = bool(d["shore_flags"] & 1)
        d["shore_centred"] = bool(d["shore_flags"] & 2)
        d["shore_confirm"] = bool(d["shore_flags"] & 4)
    if "ll_state" in d:
        d["ll_name"] = LL_STATES[d["ll_state"]] if d["ll_state"] < 3 else "?"
        d["ll_armed"] = bool(d["ll_flags"] & 1)
        d["near_shore"] = bool(d["ll_flags"] & 2)
        d["ll_left_s"] = d["ll_left10"] / 10.0
        d["coast_valid"] = bool(d["ll_flags"] & 0x20)
        d["coast_dist_m"] = None if d["coast_m"] >= 0xFFFE else d["coast_m"]
    if "alarm_bits2" in d:
        f = d["batt_flags"]
        d["batt_measured"] = bool(f & 1)                # the bank voltage is measured (fresh)
        d["batt_no_data"] = bool(f & 2)                 # notice NO VOLTAGE DATA (not an alarm)
        d["batt_low"] = bool(f & 4)                     # LOW BATTERY condition (alarm bit 1 of word 2)
        d["bank_v"] = None if d["bank_v100"] == 0xFFFF else d["bank_v100"] / 100.0
        d["bank_v_fast"] = None if d["bank_fast100"] == 0xFFFF else d["bank_fast100"] / 100.0
        d["gen_state_name"] = GEN_STATES[d["gen_state"]] if d["gen_state"] < len(GEN_STATES) else "?"
        d["gen_active"] = d["gen_state"] in (3, 4)          # RUNNING / STOPPING: throttle above idle in neutral
    if "att_age_ms" in d:
        ok = d["att_age_ms"] < 1000
        d["att_roll"] = d["att_roll100"] / 100.0 if ok else None
        d["att_pitch"] = d["att_pitch100"] / 100.0 if ok else None
    if "slam_flags" in d:
        f = d["slam_flags"]
        d["slam_peak"] = d["slam_peak100"] / 100.0
        d["slam_level"] = d["slam_level100"] / 100.0
        d["slam_limit"] = d["slam_limit100"] / 100.0
        d["slam_state"] = ["CALM", "YELLOW", "RED"][f & 3] if f & 3 < 3 else "?"
        d["slam_override"] = bool(f & 0x10)
        d["slam_lost"] = bool(f & 0x20)
        d["slam_sensor"] = bool(f & 0x40)
    return d
