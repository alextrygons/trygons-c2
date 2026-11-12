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
T_REMOTE = 0x37            # u8 0 remote driving (CREWED + CONFIRM on board; UNCREWED at once) / 1 withdraw a pending request
T_MANEUVER = 0x38          # u8 0 MANEUVER (captain; as T_REMOTE) / 1 withdraw (13q2: no release)
T_PROG_BEGIN = 0x20
T_PROG_WP = 0x21
T_PROG_COMMIT = 0x22
T_PROG_GET = 0x23
T_AUTO_START = 0x24        # then joystick CONFIRM within 10 s
T_AUTO_PAUSE = 0x25
T_AUTO_RESUME = 0x26       # then joystick CONFIRM within 10 s
T_AUTO_SKIP = 0x27
T_AUTO_RETURN = 0x13       # AUTO RETURN: the program backwards from the current leg to waypoint 1; then CONFIRM
T_POWER_CODE = 0x28        # u8 dir (0 fwd, 1 rev): the boat shows a code
T_POWER_RAISE = 0x29       # u8 dir, u16 typed code; then joystick CONFIRM within 10 s
T_POWER_LOWER = 0x2A       # back to 30 % / 5 % at once
T_BENCH = 0x2B             # u8 action, u16 arg - BENCH TEST, only from the laptop on the USB cable
T_GENERATOR = 0x2C         # u8 0 off / 1 on: generator auto (HELM station; a copilot's ON needs the captain's ACCEPT)
T_RELAY = 0x1E             # u8 n, u8 action (0 off / 1 on / 2 pulse), u16 pulse ms: switch a relay of the table (HELM station)
T_RELAY_CFG = 0x2F         # u8 count, count x {u8 n, u8 type, u8 group, u8 pair, u16 max_s*10, u8 flags, u8 role}: the boat file's
                           # relay table (HELM station; a different table than the stored one: STANDBY + joystick CONFIRM)
T_THRUSTER_CFG = 0x1F      # 12 bytes: bow {u8 max_pct, u16 run_s, u16 rest_s}, stern {...}, u8 jog_m*10, u8 spot_alarm_m:
                           # the boat file's DP thruster settings (HELM station, STANDBY only, RAM only; app/thruster.h)
T_RESTART = 0x2D           # u8 action: 0 RESTART (clear the ENGINE KILL / software reset after a monitor trip), 1 CRANK
                           # (START held, 5 Hz), 2 RELEASE - HELM station; idle + neutral, e-stop released (app/engine.h)
T_DP = 0x2E                # u8 action (0 START, 1 JOG, 2 SPEED, 3 HEADING, 4 STOP), u8 arg, i16 value: DP (app/dp.h;
                           # HELM station; START from STANDBY + joystick CONFIRM; a copilot's needs the captain's ACCEPT
                           # except STOP)
T_CONFIRM_ONLY = 0x30     # u16 token: the academy PRACTICAL TEST's final approval (step 21) - answered only by the
                           # onboard joystick CONFIRM within 10 s, CHANGES NOTHING on the boat (TELEM2 group R);
                           # HELM station, CREWED only, never FIRST TEST, never replaces another pending CONFIRM
T_SLAM_RESTORE = 0x11      # u8 1 ACCEPT / 0 REJECT: the slamming power restore (step 13q4, owner 2026-10-31) - the
                           # HELM station's answer to the boat's question (TELEM2 group T); the gateway: the captain /
                           # a single operator only (a copilot = 118); the reduction never goes away by itself
SLAM_REJECT, SLAM_ACCEPT = 0, 1
T_LINK_TIME = 0x14         # u8 seconds 3..60: LINK TIME = the UNCREWED link-loss detection time (step 13q1): the
                           # HELM captain only (gateway: a copilot = 118), RAM only, back to 3 s at a HELM grant / AURIX
                           # reset; never in force near shore (TELEM2 group S)
LINK_TIME_MIN_S, LINK_TIME_MAX_S = 3, 60
T_SIM_JOYSTICK = 0x70   # simulator only
T_SIM_GPS = 0x71        # simulator only
T_SIM_BUTTON = 0x72     # simulator only: panel button
T_SIM_BRIDGE = 0x73     # simulator only: the USB joysticks behind the simulated bridge
T_SIM_FAULT = 0x74      # simulator only: a world / sensor fault (academy emergency scenarios, step 19)
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
T_CAN_NODES = 0x8E         # AURIX -> USB laptop, 1 Hz while a CAN bus is fitted: DroneCAN bus + node table
T_ACCEL = 0x8F             # AURIX -> both ports, with every telemetry: 3-axis acceleration (step FW-TR, display / log only)
ACCEL_LEN = 28             # = app/proto.h PROTO_ACCEL_LEN
T_HARBOUR = 0x3C           # step HX2: u8 action (0 START, 1 RESUME, 2 HOLD) | u8 corridor 0.1 m | u8 max kn*10 (HELM station;
                           # START / RESUME + the joystick CONFIRM = the captain's YES; a copilot's = the captain's ACCEPT)
T_HARBOUR_STAT = 0x90      # step HX2: AURIX -> both ports, 5 Hz: harbour autonomy (link.c harbour_stat)
HARBOUR_STAT_LEN = 18      # = app/proto.h PROTO_HARBOUR_STAT_LEN
HARB_START, HARB_RESUME, HARB_HOLD = range(3)
# app/harbour.h HARB_ / HARB_WHY_
HARB_STATES = ["OFF", "RUN", "SLOW", "HOLD", "DONE"]
HARB_WHY = ["-", "user HOLD", "lease lapsed", "planner STOP", "corridor", "heading refused", "GPS", "compass",
            "link lost", "fault", "not a harbour route", "route not checked by the gateway", "outside the corridor",
            "too fast", "no planner lease", "arrived"]
NAV_HARBOUR = 4            # GW_NAVREQ kind 4 (13 bytes: + u8 hflags | u8 nearest_m | u8 n_tracks)

# Commands that change how the boat is driven: through the gateway a copilot's are only
# sent to the AURIX after the captain's ACCEPT on screen 1 (or in SINGLE-OPERATOR mode
# after the own confirmation). Never STANDBY. PROG_COMMIT = a new AUTO route (the frames
# before it only fill the AURIX's staging area; COMMIT makes it the route).
DRIVING_CMDS = {T_SET_MODE, T_AUTO_START, T_AUTO_PAUSE, T_AUTO_RESUME, T_AUTO_SKIP, T_AUTO_RETURN, T_HEADING_ADJ, T_SEA_STATE,
                T_TRAINING, T_PARAM_SET, T_POWER_CODE, T_PROG_COMMIT, T_GENERATOR, T_RELAY, T_RELAY_CFG, T_RESTART,
                T_THRUSTER_CFG, T_DP, T_HARBOUR}
# (T_HARBOUR: START / RESUME = the captain's ACCEPT from a copilot; HOLD goes straight, like AUTO PAUSE - CLAUDE.md Z)
# (T_THRUSTER_CFG = the DP thruster settings: a copilot's always needs the captain's ACCEPT, like the relay table)
# (T_DP: START / JOG / SPEED / HEADING move the boat = ACCEPT; STOP (thrusters 0, STANDBY) goes straight - CLAUDE.md P/D)
# (T_RELAY_CFG = the relay table: always the captain's ACCEPT. T_RELAY (CLAUDE.md O): an ENGINE-group relay needs
#  the captain's ACCEPT from a copilot, a trim / extra relay goes straight - but only when the relay's group is KNOWN
#  from the table that is active on the AURIX (relay_groups); unknown relay / unknown table = engine = ACCEPT.)
RELAY_FREE_GROUPS = ("trim", "extra")


def relay_groups(payload):
    """{n: group} of a PROTO_T_RELAY_CFG payload (p_relay_cfg), None if it can't be read. A relay with an engine
    role counts as engine whatever its group byte says."""
    try:
        cnt = payload[0]
        if len(payload) != 1 + 8 * cnt:
            return None
        out = {}
        for i in range(cnt):
            n, _ty, grp, _pair, _ms, _fl, role = struct.unpack_from("<BBBBHBB", payload, 1 + 8 * i)
            out[n] = RELAY_GROUPS[grp] if grp < len(RELAY_GROUPS) and not role else "engine"
        return out
    except (IndexError, struct.error):
        return None


def groups_of(relays):
    """{n: group} of boats.py relay dicts (the same rule as relay_groups: a role = engine)."""
    return {int(r["n"]): ("engine" if r.get("role") else r.get("group")) for r in relays or []}


def is_driving_cmd(ftype, payload=b"", relay_groups=None):
    """relay_groups: {n: group} of the table ACTIVE on the AURIX (None = not known: every relay = ACCEPT)."""
    if ftype not in DRIVING_CMDS:
        return False
    if ftype == T_RELAY:
        g = (relay_groups or {}).get(payload[0]) if payload else None
        return g not in RELAY_FREE_GROUPS
    if ftype == T_GENERATOR:
        return payload[:1] == b"\x01"           # switching the generator ON opens the throttle (in neutral): ACCEPT; OFF is free
    if ftype == T_DP:
        return payload[:1] != bytes([DP_STOP])     # DP STOP: a copilot may always stop (like STANDBY / hold)
    if ftype == T_HARBOUR:
        return payload[:1] != bytes([HARB_HOLD])   # harbour HOLD: free (it only stops the boat)
    if ftype == T_RESTART:
        # RESTART: a copilot's needs the captain's ACCEPT; CRANK / RELEASE (hold-to-crank, 5 Hz) are not asked one by
        # one - the gateway lets only the captain crank (code 118), RELEASE is always free
        return payload[:1] == bytes([ENG_RESTART])
    return not (ftype == T_SET_MODE and payload[:1] == bytes([MODE_STANDBY]))


TYPE_NAMES = {
    T_HEARTBEAT: "heartbeat", T_SET_MODE: "set mode",
    T_HEADING_ADJ: "heading", T_SEA_STATE: "sea state", T_TRAINING: "training", T_PARAM_GET: "param read",
    T_PARAM_SET: "param write", T_JOY_CAL: "joystick calibration", T_ALARM_ACK: "alarm ack",
    T_PROG_BEGIN: "program begin", T_PROG_WP: "program waypoint", T_PROG_COMMIT: "program upload",
    T_PROG_GET: "program read", T_AUTO_START: "AUTO start", T_AUTO_PAUSE: "AUTO pause",
    T_AUTO_RESUME: "AUTO resume", T_AUTO_SKIP: "AUTO skip", T_AUTO_RETURN: "AUTO return",
    T_POWER_CODE: "power code", T_POWER_RAISE: "power raise", T_POWER_LOWER: "power lower",
    T_SIM_JOYSTICK: "sim joystick", T_SIM_GPS: "sim GPS", T_SIM_BUTTON: "sim button",
    T_JOYMAP_GET: "joystick map read", T_JOYMAP_SET: "joystick map write", T_JOY_RAW_GET: "joystick raw read",
    T_SIM_BRIDGE: "sim USB joystick", T_SIM_FAULT: "sim fault", T_JOYMAP_VALUE: "joystick map", T_JOY_RAW: "joystick raw",
    T_GW_STATUS: "gateway status", T_LOG_CHUNK: "log chunk", T_TIME: "GPS time", T_TELEM2: "telemetry 2",
    T_GW_CMD: "station command", T_HELM: "HELM", T_GW_STATION: "station event", T_GW_ACK: "station answer",
    T_GW_RJOY: "remote joystick", T_REMOTE: "remote driving", T_MANEUVER: "maneuver", T_BENCH: "bench test", T_BENCH_DIAG: "bench data", T_CAN_NODES: "CAN nodes", T_ACCEL: "acceleration",
    T_GENERATOR: "generator auto", T_RELAY: "relay", T_RELAY_CFG: "relay table", T_RESTART: "engine restart / start",
    T_THRUSTER_CFG: "thruster settings", T_DP: "DP", T_CONFIRM_ONLY: "confirm-only (practical test)",
    T_LINK_TIME: "LINK TIME", T_SLAM_RESTORE: "slamming power restore", T_HARBOUR: "harbour autonomy",
    T_HARBOUR_STAT: "harbour status",
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
    23: "relay stop pending: the relay table changes after it",
    # gateway refusals (gateway/stations.py)
    100: "VIEWER: watch only", 101: "duplicate command", 102: "command out of order", 103: "stale command",
    104: "timestamp in the future", 105: "rate limited", 106: "AURIX not connected", 107: "wrong session",
    108: "bad command", 116: "no manual licence: REMOTE MANUAL / MANEUVER only for a licensed captain",
    117: "only the captain (or a station on the boat) holds HELM", 118: "captain only: REMOTE MANUAL / MANEUVER / engine START / LINK TIME / power restore",
    119: "no captain holds HELM",
}

MODES = ["STANDBY", "MANUAL", "HDG_LOCK", "AUTO", "TRAINING", "REMOTE_MANUAL", "BENCH_TEST", "MANEUVER", "DP"]
# mode change reasons (app/modes.h mode_reason_t)
WHY = ["user", "STANDBY button", "compass fault", "GPS lost", "arrived", "training done", "training abort",
       "test", "gear fault", "DR limit", "arrival uncertain", "joystick", "joystick hand-back", "joystick fault",
       "PC", "HDG LOCK button", "CONFIRM", "not armed", "safety monitor", "remote", "remote end", "remote lost",
       "emergency stop", "bench test ended (idle / link / moving / HELM)", "maneuver",
       "profile fault (joystick appeared, UNCREWED)", "link to the HELM station lost", "DP (HELM + CONFIRM)", "DP STOP",
       "DP: off the spot", "DP: thruster hot / blocked", "CAN node lost", "CAN bus fault (or our node ID on the bus)",
       "HDG LOCK from the shore (UNCREWED)", "geofence breach (UNCREWED HDG LOCK)"]
MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING, MODE_REMOTE, MODE_BENCH, MODE_MANEUVER, MODE_DP = range(9)
# DP (app/dp.h): sub-states (TELEM2 group N "dp_sub"), PROTO_T_DP actions, why it ended
DP_SUBS = ["off", "SPOT", "JOG", "CRUISE", "TRIP"]
DP_OFF, DP_SPOT, DP_JOG, DP_CRUISE, DP_TRIP = range(5)
DP_TRIP_CONTINUE = 0x80
DP_START, DP_JOG_ACT, DP_SPEED, DP_HEADING, DP_STOP = range(5)
DP_JOG_AHEAD, DP_JOG_ASTERN, DP_JOG_PORT, DP_JOG_STBD = range(4)
DP_STOP_WHY = ["-", "DP STOP", "STANDBY (button / e-stop / link lost)", "joystick took over", "GPS lost / suspect",
               "compass lost", "thruster hot / blocked", "off the spot", "fault"]


def p_link_time(seconds):
    """PROTO_T_LINK_TIME payload: the link-loss detection time in whole seconds (3..60)."""
    return bytes([int(seconds) & 0xFF])


def p_slam_restore(accept):
    """PROTO_T_SLAM_RESTORE payload: 1 ACCEPT (the cap rises at the normal throttle up rate) / 0 REJECT."""
    return bytes([SLAM_ACCEPT if accept else SLAM_REJECT])


def p_confirm_only(token):
    """PROTO_T_CONFIRM_ONLY payload: the question's number (echoed in TELEM2 group R). Changes nothing on the boat."""
    return struct.pack("<H", int(token) & 0xFFFF)


def p_harbour(action, corridor_m=10.0, max_kn=5.0):
    """PROTO_T_HARBOUR payload (step HX2): action HARB_START / RESUME / HOLD, the corridor half-width (0.1 m; the AURIX
    clamps it to 4 m .. harb_corr_m, below 4 m = BAD_VALUE) and the max speed (kn*10; min(this, harb_max_kn, 8 kn))."""
    return bytes([action & 0xFF, max(0, min(255, int(round(corridor_m * 10)))), max(0, min(255, int(round(max_kn * 10))))])


_HSTAT = struct.Struct("<BBhHHHHBBHBB")


def decode_harbour_stat(payload: bytes) -> dict:
    """T_HARBOUR_STAT (link.c harbour_stat, step HX2): harbour autonomy as the AURIX sees it."""
    if len(payload) < _HSTAT.size:
        raise ValueError("harbour status %d bytes, want %d" % (len(payload), _HSTAT.size))
    st, why, xte, corr, cap, hdg, age, refused, flags, left, hfl, kmax = _HSTAT.unpack_from(payload)
    return {"harb_state": st, "harb_state_name": HARB_STATES[st] if st < len(HARB_STATES) else str(st),
            "harb_why": why, "harb_why_text": HARB_WHY[why] if why < len(HARB_WHY) else str(why),
            "harb_xte_m": xte / 10.0, "harb_corr_m": corr / 10.0, "harb_kn": cap / 10.0, "harb_hdg": hdg / 100.0,
            "harb_prop_age_ms": None if age >= 65534 else age, "harb_refused": refused,
            "harb_route": bool(flags & 1), "harb_entry_ok": bool(flags & 2), "harb_lease": bool(flags & 4),
            "harb_hold_pending": bool(flags & 8), "harb_grace": bool(flags & 16), "harb_back": bool(flags & 32),
            "harb_prop_used": bool(flags & 64), "harb_grace_left_s": left / 10.0, "harb_hflags": hfl,
            "harb_max_kn": kmax / 10.0}


def p_dp(action, arg=0, value=0):
    """PROTO_T_DP payload: START (arg sub, value speed kn*100) / JOG (arg dir) / SPEED (value kn*100) /
    HEADING (arg 0 absolute deg*10, 1 change deg*10) / STOP."""
    return struct.pack("<BBh", action, arg, int(value))
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


# relay board (app/relays.h): the table's frame encoding and the relay command
RELAY_TYPES = ["onoff", "momentary", "latching", "timed"]
RELAY_GROUPS = ["engine", "trim", "extra"]
RELAY_ROLES = [None, "starter", "ignition", "shutoff"]     # app/relays.h RELAY_ROLE_ (engine group only)
RELAY_TABLE_MAX = 16
RELAY_CMD_OFF, RELAY_CMD_ON, RELAY_CMD_PULSE = 0, 1, 2


def p_relay_cfg(relays):
    """The relay table as the AURIX takes it (PROTO_T_RELAY_CFG): a list of boats.py relay dicts
    (n, type, group, pair, max_s, show_hud, power_up, role). Names stay on the PC."""
    if len(relays) > RELAY_TABLE_MAX:
        raise ValueError("at most %d relays" % RELAY_TABLE_MAX)
    out = bytearray([len(relays)])
    for r in relays:
        flags = (1 if r.get("show_hud") else 0) | (2 if r.get("power_up") == "on" else 0) | (
            4 if r.get("power_up") == "keep" else 0)
        out += struct.pack("<BBBBHBB", int(r["n"]), RELAY_TYPES.index(r["type"]), RELAY_GROUPS.index(r["group"]),
                           int(r.get("pair") or 0), int(round(float(r.get("max_s") or 0) * 10)), flags,
                           RELAY_ROLES.index(r.get("role")))
    return bytes(out)


def relay_table_hash(relays):
    """What TELEM2 group M "relay_hash" shows for this table (CRC-16 of the frame encoding)."""
    return crc16(p_relay_cfg(relays))


def p_thruster_cfg(thrusters):
    """The DP thruster settings as the AURIX takes them (PROTO_T_THRUSTER_CFG, app/thruster.h): a boats.py
    "thrusters" dict (bow / stern {max_pct, run_s, rest_s}, jog_m, spot_alarm_m). No / missing block = no thruster."""
    t = thrusters or {}
    out = bytearray()
    for k in ("bow", "stern"):
        u = t.get(k) or {}
        out += struct.pack("<BHH", int(round(float(u.get("max_pct", 0)))), int(round(float(u.get("run_s", 0)))),
                           int(round(float(u.get("rest_s", 0)))))
    out += struct.pack("<BB", int(round(float(t.get("jog_m", 1.5)) * 10)), int(round(float(t.get("spot_alarm_m", 15)))))
    return bytes(out)


def thruster_cfg_hash(thrusters):
    """What TELEM2 group N "thr_hash" shows once the AURIX took these settings (CRC-16 of the frame body)."""
    return crc16(p_thruster_cfg(thrusters))


def p_relay(n, action, ms=0):
    return struct.pack("<BBH", n, action, ms)


_BENCH = struct.Struct("<BBIH5f5B2ffBBBII4HB6B2HBbfBI")      # 86 bytes, link.c bench_diag()


BENCH_DIAG_STACK_OFF = 86   # app/proto.h PROTO_BENCH_DIAG_STACK_OFF (step 13s)
BENCH_DIAG_LEN = 114        # app/proto.h PROTO_BENCH_DIAG_LEN
_BENCH_STK_HEAD = struct.Struct("<BBBB")    # count | flags (b0 measured on the board) | known mask | overflow mask
_BENCH_STK = struct.Struct("<HH")           # per stack: reserved bytes, deepest use bytes
STACK_NAMES = ["u0", "i0", "u1", "i1", "u2", "i2"]   # hal.h HAL_STACK_U0 .. HAL_STACK_I2 (user / interrupt x core)


def decode_bench_stacks(payload: bytes):
    """The stack high-water marks after the 86 bytes of sensors (step 13s; None from an older AURIX)."""
    if len(payload) < BENCH_DIAG_STACK_OFF + _BENCH_STK_HEAD.size:
        return None
    n, flags, known, over = _BENCH_STK_HEAD.unpack_from(payload, BENCH_DIAG_STACK_OFF)
    off = BENCH_DIAG_STACK_OFF + _BENCH_STK_HEAD.size
    out = []
    for i in range(n):
        if off + _BENCH_STK.size > len(payload):
            break
        size, used = _BENCH_STK.unpack_from(payload, off)
        off += _BENCH_STK.size
        out.append({"name": STACK_NAMES[i] if i < len(STACK_NAMES) else "s%d" % i, "core": i // 2,
                    "kind": "user" if i % 2 == 0 else "interrupt", "size": size,
                    "used": used if known & (1 << i) else None, "known": bool(known & (1 << i)),
                    "overflow": bool(over & (1 << i))})
    return {"board": bool(flags & 1), "stacks": out}


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
        "idle_left_s": idle_left / 1000.0, "stack": decode_bench_stacks(payload),
    }
# DroneCAN (step 11d; app/link.c can_nodes, hal.h HAL_CAN_): bus states, node roles, why a node is lost
CAN_STATES = ["off", "running", "error passive", "bus-off", "controller fault"]
CAN_ROLES = ["throttle", "steering", "gear", "relay board"]
CAN_WHY = ["ok", "never seen", "silent", "not OPERATIONAL", "health ERROR / CRITICAL", "rebooted", "bus"]
CAN_HEALTH = ["OK", "WARNING", "ERROR", "CRITICAL"]
CAN_MODES = {0: "OPERATIONAL", 1: "INITIALIZATION", 2: "MAINTENANCE", 3: "SOFTWARE_UPDATE", 7: "OFFLINE"}
_CAN_HEAD = struct.Struct("<BBBBIIIIIB")
_CAN_NODE = struct.Struct("<BBBBBBIIHI")


def decode_can_nodes(payload: bytes) -> dict:
    """T_CAN_NODES (link.c can_nodes): the bus + every node of the static node table."""
    st, flags, tec, rec, rx, tx, drops, rxerr, boffs, n = _CAN_HEAD.unpack_from(payload)
    nodes, off = [], _CAN_HEAD.size
    for _ in range(n):
        nid, role, f, why, health, mode, up, age, restarts, statuses = _CAN_NODE.unpack_from(payload, off)
        off += _CAN_NODE.size
        nodes.append({"id": nid, "role": role, "role_name": CAN_ROLES[role] if role < len(CAN_ROLES) else "?",
                      "required": bool(f & 1), "seen": bool(f & 2), "lost": bool(f & 4), "warning": bool(f & 8),
                      "rebooted": bool(f & 16), "why": why,
                      "why_text": CAN_WHY[why] if why < len(CAN_WHY) else "why %d" % why,
                      "health": health, "health_name": CAN_HEALTH[health] if health < 4 else "?", "mode": mode,
                      "mode_name": CAN_MODES.get(mode, "mode %d" % mode), "uptime_s": up,
                      "age_ms": None if age == 0xFFFFFFFF else age, "restarts": restarts, "statuses": statuses})
    return {"state": st, "state_name": CAN_STATES[st] if st < len(CAN_STATES) else "?", "fault": bool(flags & 2),
            "conflict": bool(flags & 4), "anon": bool(flags & 8), "silent": bool(flags & 16), "block": bool(flags & 32),
            "tec": tec, "rec": rec, "rx": rx, "tx": tx, "tx_drops": drops, "rx_errors": rxerr, "busoffs": boffs,
            "nodes": nodes}


ACCEL_AXES = ("lon", "lat", "vert")    # + forward, + starboard, + up (BNO085 linear acceleration, gravity removed)
_ACCEL = struct.Struct("<BB6h6hH")


def decode_accel(payload: bytes) -> dict:
    """T_ACCEL (link.c accel_frame, step FW-TR; DISPLAY / LOGGING ONLY): average + peak (the largest magnitude,
    signed) per axis in g, over the boat's telemetry period (acc_<axis>, acc_<axis>_pk; None = no sample in it) and
    over the last complete 1 s window (acc1_<axis>, acc1_<axis>_pk; None = no fresh second)."""
    if len(payload) < _ACCEL.size:
        raise ValueError("accel frame %d bytes, want %d" % (len(payload), _ACCEL.size))
    v = _ACCEL.unpack_from(payload)
    flags, n, per, sec, n1 = v[0], v[1], v[2:8], v[8:14], v[14]
    d = {"acc_n": n, "acc1_n": n1 if flags & 2 else 0}
    for k, ax in enumerate(ACCEL_AXES):
        d["acc_" + ax] = per[2 * k] / 1000.0 if flags & 1 else None
        d["acc_" + ax + "_pk"] = per[2 * k + 1] / 1000.0 if flags & 1 else None
        d["acc1_" + ax] = sec[2 * k] / 1000.0 if flags & 2 else None
        d["acc1_" + ax + "_pk"] = sec[2 * k + 1] / 1000.0 if flags & 2 else None
    return d


def encode_accel(per=None, sec=None, n=0, n1=0) -> bytes:
    """Test / simulation helper, the inverse of decode_accel: per / sec = ((avg, peak) lon, lat, vert) in g or None."""
    def mg(x):
        return max(-32767, min(32767, int(round(x * 1000.0))))
    flat = []
    for w in (per, sec):
        flat += [mg(x) for pair in (w or ((0, 0),) * 3) for x in pair]
    return _ACCEL.pack((1 if per else 0) | (2 if sec else 0), min(255, n), *flat, min(65535, n1 if sec else 0))


AUTO_STATES = ["running", "PAUSED", "STOPPED AT WAYPOINT", "HOLDING (fallback)"]
CONFIRM_KINDS = ["", "AUTO START", "AUTO RESUME", "POWER RAISE FWD", "POWER RAISE REV", "REMOTE DRIVING", "MANEUVER",
                 "RELAY TABLE", "DP", "THROTTLE TRAVEL", "AUTO RETURN", "CONFIRM ONLY", "HARBOUR"]
# CONFIRM_ONLY answer (TELEM2 group R, app/smallboat.h SB_CO_)
CO_NONE, CO_PENDING, CO_CONFIRMED, CO_LATE, CO_CANCELLED = range(5)
CO_STATES = ["never asked", "PENDING", "CONFIRMED", "LATE", "CANCELLED"]
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
    (0x002, "RELAY BOARD SILENT"),
    (0x004, "DP THRUSTER HOT"),
    (0x008, "DP OFF SPOT"),
    (0x010, "DP OVERPOWER"),
    (0x020, "CAN NODE LOST"),
    (0x040, "CAN BUS FAULT"),
    (0x080, "GEOFENCE BREACH"),
    (0x100, "HARBOUR STOP"),       # step HX2 (the screens' level / text / tour stop: HX6)
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


# PROTO_T_SIM_FAULT faults (sim/sim_server.c; simulator only, the board answers UNKNOWN)
SIM_FAULTS = {"compass": 1, "gps_spoof": 2, "slam": 3, "joy_break": 4}


def p_sim_fault(fault, on):
    return struct.pack("<BB", SIM_FAULTS[fault] if isinstance(fault, str) else fault, 1 if on else 0)


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
    d["prog_gw"] = bool(d["auto_flags"] & 8)        # loaded over the gateway link (its route was checked there)
    d["prog_returning"] = bool(d["auto_flags"] & 16)    # AUTO RETURN: driving the program back (auto_wp = its waypoint)
    d["confirm_left_s"] = d["confirm_left_ds"] / 10.0
    d["power_code_left_s"] = d["power_code_left_ds"] / 10.0
    d["speed_limited"] = bool(d["power_flags"] & 1)
    d["power_raised"] = bool(d["power_flags"] & 2)
    d["mode_name"] = MODES[d["mode"]] if d["mode"] < len(MODES) else "?"
    # DP (step 11): the AURIX's dead-reckoning velocity (est_sog / est_cog) comes from the outboard, which is at idle
    # in DP - the thrusters move the boat. With the GPS trusted, the screens and the gateway (collision avoidance
    # in a DP TRIP) use the GPS speed / course in DP; the DR's own values stay in dr_sog / dr_cog.
    d["dr_sog"], d["dr_cog"] = d["est_sog"], d["est_cog"]
    if d["mode"] == MODE_DP and d["pos_source"] == 1 and d["gps_fix"]:
        d["est_sog"], d["est_cog"] = d["gps_sog"], d["gps_cog"]
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


# --- gateway (gateway PC) -----------------------------------------------------------

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


def p_gw_status(flags, channel=0, stations=0, pi_log_seq=0, pi_log_gaps=0, helm_station=0, helm_age_s=None):
    """GW_STATUS (gateway -> AURIX, 1 Hz): link.c gw_status(). helm_station / helm_age_s (step 13q1): the HELM
    station and the time since the gateway last heard it (None = gone); station 0 = not reported."""
    if not (2 <= int(helm_station or 0) <= 254):
        st, age = 0, 0
    else:
        st = int(helm_station)
        age = 255 if helm_age_s is None else max(0, min(255, int(float(helm_age_s) / 0.25)))
    return struct.pack("<BBBBIHBB", 1, flags & 0xFF, channel & 0xFF, stations & 0xFF,
                       pi_log_seq & 0xFFFFFFFF, min(pi_log_gaps, 0xFFFF), st, age)


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
    (struct.Struct("<IIHBBI"), "relay_states relay_known relay_hash relay_flags relay_count relay_cmd_on".split()),
    (struct.Struct("<BBHHBBHH"), "eng_crank eng_flags eng_hold_ms eng_cool10 eng_action eng_why eng_restarts "
                                 "eng_cranks".split()),
    (struct.Struct("<BbbBBBBHB"), "thr_flags thr_bow thr_stern thr_heat_b thr_heat_s thr_max_b thr_max_s thr_hash "
                                  "thr_why".split()),
    # group N, the DP state (step 10)
    (struct.Struct("<BBhhHBBBB"), "dp_sub dp_flags dp_err_fwd10 dp_err_stbd10 dp_hdg10 dp_set10 dp_trip10 dp_wp "
                                  "dp_stop".split()),
    # group P, the DroneCAN summary (step 11d; the node table: T_CAN_NODES on the USB link)
    (struct.Struct("<BBBBBB"), "can_flags can_state can_lost can_warn can_seen can_reboot".split()),
    # group Q, the DroneCAN steering position servo + the throttle model (step 11e)
    (struct.Struct("<BbbBBBBHBBBBBBH"), "sv_flags sv_cmd100 sv_pos100 sv_r_right100 sv_r_left100 sv_b_stored1000 "
                                         "sv_b_use1000 sv_k100 sv_k_pct sv_tau10 sv_conf sv_reset_why thr_est_pct "
                                         "thr_travel10 sv_train_boot".split()),
    # group R, CONFIRM_ONLY (academy practical test, step 21): the answer only - it changes nothing on the boat
    (struct.Struct("<BH"), "co_state co_token".split()),
    # group S, LINK TIME (step 13q1): the captain's setting + flags (b0 age reported by the gateway, b1 not in force
    # now: near shore / no report, b2..b7 HELM station silent for whole s, 63 = not counting)
    (struct.Struct("<BB"), "lt_set lt_flags".split()),
    # group T, the slamming power restore (step 13q4): b0 question open, b1 ACCEPT possible now (RESTORE POWER
    # button), b2 restoring, b3 declined, b4..b7 seconds left | the throttle % before the reduction
    (struct.Struct("<BB"), "slr_flags slr_before".split()),
]
# app/steermodel.h SM_RESET_: why the steering model went back to the TRAINING values (TELEM2 Q, log event 30)
SM_RESET_WHY = ["-", "model residual too large", "an adapted value reached its bound", "compass / gyro / GPS doubt",
                "steering node lost", "new TRAINING values"]
LL_STATES = ["OK", "LINK LOST", "ENGINE KILLED"]
# engine controls (app/engine.h, TELEM2 group O, log event RESTART)
ENG_RESTART, ENG_CRANK, ENG_RELEASE = 0, 1, 2           # PROTO_T_RESTART actions
ENG_CRANK_STATES = ["OFF", "CRANKING", "COOL-DOWN"]
ENG_ACTIONS = {0: "-", 1: "ENGINE KILL cleared", 2: "software reset after a monitor trip", 3: "START (crank) begins",
               4: "START (crank) stops", 5: "RESTART refused", 6: "START refused", 7: "reset cancelled"}
ENG_WHY = ["-", "FIRST TEST", "throttle not idle", "gear not in neutral", "e-stop pressed", "nothing to restart",
           "engine killed: RESTART first", "monitor tripped: RESTART first", "no STARTER in the relay table",
           "starter cool-down", "relay board silent / refused", "not the HELM station", "link lost", "STANDBY button",
           "released", "5 s limit"]


def eng_why_text(why):
    return ENG_WHY[why] if 0 <= why < len(ENG_WHY) else "why %d" % why


def p_restart(action):
    """PROTO_T_RESTART payload: ENG_RESTART / ENG_CRANK / ENG_RELEASE."""
    return bytes([action])
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
    if "rjoy_flags" in d:
        # step 13q3 (owner 2026-10-31 C3): the shore stick override in UNCREWED AUTO / HDG LOCK (link.c group D flags)
        f = d["rjoy_flags"]
        d["ov_hand"] = bool(f & 8)          # steering by hand with the shore stick (the mode waits)
        d["ov_back"] = bool(f & 16)         # the stick centred: back to the mode's steering in 2 s
        d["ov_newhdg"] = bool(f & 32)       # NEW HEADING (the trigger, 3 s; the value = the target heading)
        d["ov_ceiling"] = bool(f & 64)      # the lever ceiling lowers the throttle now
        d["ov_lost"] = bool(f & 128)        # AUTO: the shore data lost while steering by hand (idle + N, STANDBY 5 s)
        # step 13q3b (review MED-2): in UNCREWED AUTO b1 = the lever is below the throttle but not engaged yet ("move
        # the lever"; the pickup target = where it meets the throttle) - in REMOTE / MANEUVER b1 is the lever pickup
        d["ov_lever_wait"] = bool(f & 2)
        # step 13q5 (owner 2026-10-31 C5): in MANEUVER the u16 after the flags = 0x8000 | the shore stick Y's gear
        # ask (b0-b1: 0 N, 1 F, 2 R) | the full deflection held now (b4-b5: 1 forward, 2 back); else the pickup target
        v = d.get("rjoy_pickup1000", 0xFFFF)
        if v != 0xFFFF and v & 0x8000:
            d["mv_stick_ask"] = v & 3
            d["mv_stick_hold"] = (v >> 4) & 3
            d["rjoy_pickup1000"] = 0xFFFF
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
    if "lt_set" in d:
        f = d["lt_flags"]
        d["lt_reported"] = bool(f & 1)                  # the gateway reports the HELM station's age (else the old rule)
        d["lt_override"] = d["lt_set"] > LINK_TIME_MIN_S  # LINK TIME above 3 s: shown "LINK TIME 30 s (OVERRIDE)"
        d["lt_in_force"] = d["lt_override"] and not (f & 2)   # false near shore / without the report: 3 s
        d["lt_limit_s"] = d["lt_set"] if d["lt_in_force"] else LINK_TIME_MIN_S
        sil = f >> 2
        d["lt_silent_s"] = None if sil == 63 else sil   # the HELM station silent this long (UNCREWED, armed)
    if "alarm_bits2" in d:
        f = d["batt_flags"]
        d["batt_measured"] = bool(f & 1)                # the bank voltage is measured (fresh)
        d["batt_no_data"] = bool(f & 2)                 # notice NO VOLTAGE DATA (not an alarm)
        d["batt_low"] = bool(f & 4)                     # LOW BATTERY condition (alarm bit 1 of word 2)
        d["bank_v"] = None if d["bank_v100"] == 0xFFFF else d["bank_v100"] / 100.0
        d["bank_v_fast"] = None if d["bank_fast100"] == 0xFFFF else d["bank_fast100"] / 100.0
        d["gen_state_name"] = GEN_STATES[d["gen_state"]] if d["gen_state"] < len(GEN_STATES) else "?"
        d["gen_active"] = d["gen_state"] in (3, 4)          # RUNNING / STOPPING: throttle above idle in neutral
    if "relay_flags" in d:
        f = d["relay_flags"]
        d["relay_ok"] = bool(f & 1)                     # the board answers (false = RELAY BOARD SILENT, states unknown)
        d["relay_table"] = bool(f & 2)                  # a relay table is active on the AURIX
        d["relay_from_nv"] = bool(f & 4)                # ... loaded from the data flash at boot
        d["relay_pending"] = bool(f & 8)                # a different table waits for the joystick CONFIRM
        d["relay_power_up"] = bool(f & 16)              # the power_up states were applied (this boot)
        d["relay_first_test"] = bool(f & 32)            # FIRST TEST: no relay changes
        d["relay_no_readback"] = bool(f & 64)           # DroneCAN board without readback: the COMMANDED states (11f)
        d["relay_stop"] = bool(f & 128)                 # a relay stop / timer pending: no table offer now (13g)
        d["relay_on"] = [n for n in range(1, 33) if d["relay_known"] >> (n - 1) & 1 and d["relay_states"] >> (n - 1) & 1]   # lists: the gateway forwards it as JSON
        d["relay_unknown"] = [n for n in range(1, 33) if not d["relay_known"] >> (n - 1) & 1]
    if "eng_flags" in d:
        f = d["eng_flags"]
        d["eng_ready"] = bool(f & 1)                    # idle + neutral confirmed, e-stop released, not FIRST TEST
        d["eng_reset_pending"] = bool(f & 2)            # the software reset after a trip is coming
        d["eng_starter"] = bool(f & 4)                  # a STARTER (the relay with the role "starter") is in the table
        d["eng_killed"] = bool(f & 8)                   # ENGINE KILL latched
        d["eng_tripped"] = bool(f & 16)                 # the safety monitor tripped
        d["eng_crank_name"] = ENG_CRANK_STATES[d["eng_crank"]] if d["eng_crank"] < 3 else "?"
        d["eng_cool_s"] = d["eng_cool10"] / 10.0
    if "thr_flags" in d:
        f = d["thr_flags"]
        d["thr_cfg"] = bool(f & 1)                      # the AURIX has the boat file's thruster settings
        d["thr_active"] = bool(f & 2)                   # DP thrust allowed now
        d["thr_hot_bow"] = bool(f & 4)                  # heat budget used up: held at 0 until rested
        d["thr_hot_stern"] = bool(f & 8)
        d["thr_fitted_bow"] = bool(f & 16)
        d["thr_fitted_stern"] = bool(f & 32)
        if not d["thr_cfg"]:
            d["thr_hash"] = None                        # nothing taken yet (the station pushes its file)
    if "dp_sub" in d:
        f = d["dp_flags"]
        d["dp_sub_name"] = DP_SUBS[d["dp_sub"]] if d["dp_sub"] < len(DP_SUBS) else "?"
        d["dp_offspot"] = bool(f & 1)                   # DP OFF SPOT (advisory)
        d["dp_overpower"] = bool(f & 2)                 # DP OVERPOWER (advisory)
        d["dp_gyro_hold"] = bool(f & 4)                 # compass lost: the gyro holds the heading (STANDBY after 10 s)
        d["dp_trip_done"] = bool(f & 8)
        d["dp_pending"] = bool(f & 16)                  # a DP request waits for the joystick CONFIRM
        d["dp_fight"] = bool(f & 32)                    # the generator check: the stern fights a forward push
        d["dp_err_fwd"] = d["dp_err_fwd10"] / 10.0      # the spot ahead (+) / astern (-), m
        d["dp_err_stbd"] = d["dp_err_stbd10"] / 10.0    # the spot to starboard (+) / port (-), m
        d["dp_err_m"] = (d["dp_err_fwd"] ** 2 + d["dp_err_stbd"] ** 2) ** 0.5
        d["dp_heading"] = d["dp_hdg10"] / 10.0
        d["dp_set_kn"] = d["dp_set10"] / 10.0
        d["dp_trip_kn"] = d["dp_trip10"] / 10.0
        d["dp_stop_text"] = DP_STOP_WHY[d["dp_stop"]] if d["dp_stop"] < len(DP_STOP_WHY) else "why %d" % d["dp_stop"]
    if "can_flags" in d:
        f = d["can_flags"]
        d["can_fitted"] = bool(f & 1)                   # a CAN bus is fitted (else nothing below means anything)
        d["can_fault"] = bool(f & 2)                    # bus off / error passive > 100 ms / controller failed
        d["can_conflict"] = bool(f & 4)                 # OUR node ID 10 seen on the bus (the Pixhawk connected)
        d["can_anon"] = bool(f & 8)                     # notice: a node without a static ID asks for one
        d["can_silent"] = bool(f & 16)                  # nothing received for 1.5 s
        d["can_block"] = bool(f & 32)                   # a required node lost / not seen: STANDBY only
        d["can_state_name"] = CAN_STATES[d["can_state"]] if d["can_state"] < len(CAN_STATES) else "?"
        d["can_warn_nodes"] = [CAN_ROLES[i] for i in range(len(CAN_ROLES)) if d["can_warn"] >> i & 1]
        d["can_lost_nodes"] = [CAN_ROLES[i] for i in range(len(CAN_ROLES)) if d["can_lost"] >> i & 1]
    if "sv_flags" in d:
        f = d["sv_flags"]
        d["sv_servo"] = bool(f & 1)                     # the steering is a DroneCAN position servo
        d["sv_known"] = bool(f & 2)                     # its position is known (after a power-on: worst-case travel)
        d["sv_commanded"] = bool(f & 4)                 # a mode commanded steering since the boot (OD-7)
        d["sv_trained"] = bool(f & 8)                   # TRAINING values in use (else the worst-case defaults)
        d["sv_adapted"] = bool(f & 16)                  # slowly adapted values in use (RAM only)
        d["sv_needs_training"] = bool(f & 32)
        d["sv_reset_notice"] = bool(f & 64)             # STEERING MODEL RESET (30 s notice)
        d["sv_v1_refused"] = bool(f & 128)              # an old (DC-motor) record in slot 5: not used
        d["sv_cmd"] = d["sv_cmd100"] / 100.0
        d["sv_pos"] = None if d["sv_pos100"] == -128 else d["sv_pos100"] / 100.0
        d["sv_r_right"] = d["sv_r_right100"] / 100.0    # servo speed, units (-1..+1) per s
        d["sv_r_left"] = d["sv_r_left100"] / 100.0
        d["sv_play_stored"] = d["sv_b_stored1000"] / 1000.0
        d["sv_play_use"] = d["sv_b_use1000"] / 1000.0
        d["sv_k"] = d["sv_k100"] / 100.0
        d["sv_tau"] = d["sv_tau10"] / 10.0
        d["sv_reset_text"] = SM_RESET_WHY[d["sv_reset_why"]] if d["sv_reset_why"] < len(SM_RESET_WHY) else \
            "why %d" % d["sv_reset_why"]
        d["thr_travel_s"] = d["thr_travel10"] / 10.0
    if "co_state" in d:
        d["co_state_name"] = CO_STATES[d["co_state"]] if d["co_state"] < len(CO_STATES) else "?"
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
    if "slr_flags" in d:                        # group T: the slamming power restore (step 13q4)
        f = d["slr_flags"]
        d["slam_ask"] = bool(f & 1)             # the question is open: ACCEPT / REJECT on the captain's screen 1
        d["slam_restore_ok"] = bool(f & 2)      # an ACCEPT is possible now (the RESTORE POWER button)
        d["slam_restoring"] = bool(f & 4)
        d["slam_declined"] = bool(f & 8)
        d["slam_ask_left"] = f >> 4
    return d
