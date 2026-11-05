"""The UAV registry / magazine (CLAUDE.md V "Registry", docs/uav_design.md 3, owner spec H1).

state/uav_registry.json: one record per slot (1 = top = next out): sysid, board UID, name, firmware, VTX channel,
ELRS binding, the last check. Changed ONLY by "Register" (a station's uav_register, never while a launch runs).
A slot is never taken from the drone: the slot = the fixed USB port (usbslots.py).

check(reading, ...) = the registration check of what the slot's USB cable read (usbslots.SlotProbe), READ ONLY:
the gateway NEVER writes a parameter (W60: the owner sets them with Mission Planner). A drone that fails any of these
is NOT launchable (red, with every reason):
  - an ArduPilot quadrotor (HEARTBEAT type 2, autopilot 3; FRAME_CLASS 1), not armed in the slot;
  - the failsafe set (W60, TO CONFIRM against the drones' ArduPilot version): radio failsafe off (FS_THR_ENABLE 0),
    GCS failsafe off (FS_GCS_ENABLE 0), EKF failsafe AltHold (FS_EKF_ACTION 2), dead-reckoning failsafe off
    (FS_DR_ENABLE 0), fence report only (FENCE_ACTION 0), battery low = warn (BATT_FS_LOW_ACT 0), battery critical =
    Land (BATT_FS_CRT_ACT 1) at a set backstop (BATT_CRT_VOLT > 0 or BATT_CRT_MAH > 0); never RTL / SmartRTL;
  - the RC receiver IGNORED (RC_OPTIONS bit 0 "Ignore RC Receiver", U1 review HIGH-1: in LOITER / ALT_HOLD the RC
    sticks are live inputs - a frozen / low ELRS throttle channel would turn a hold into a descent; W68 bench check);
  - outputs: ONLY the 4 motors (SERVOn_FUNCTION 33-36 each exactly once, every other output 0 - never a GPIO -1, a
    servo, a gripper, a winch, a parachute, a mount); and every other release path off when the parameter exists
    (U1 review MED-1): RELAYn_FUNCTION 0 / RELAYn_PIN / RELAY_PINn -1, GRIP_ENABLE 0, CHUTE_ENABLED 0, WINCH_TYPE 0,
    MNT1_TYPE / MNT_TYPE 0 (no mount output, W60 "no tilt"), SCR_ENABLE 0 (no scripts), RCn_OPTION 0 (no aux
    switch function);
  - sensors: gyro, accelerometer, barometer present + healthy; a compass present; a GPS receiver seen (a fix is NOT
    needed in the slot - the hatch is closed; the launch re-checks before each drone, U3);
  - unique sysid in the registry; the registered UID in a slot = the drone found there (else "unknown drone").
"""
import json
import math
import os
import threading
import time

# (name, rule, required): rule = a value, or a function(value) -> ok; required = the parameter must be read
FAILSAFE = [
    ("FS_THR_ENABLE", 0, True), ("FS_GCS_ENABLE", 0, True), ("FS_EKF_ACTION", 2, True), ("FS_DR_ENABLE", 0, False),
    ("FENCE_ACTION", 0, True), ("BATT_FS_LOW_ACT", 0, True), ("BATT_FS_CRT_ACT", 1, True),
    ("BATT_CRT_VOLT", None, True), ("BATT_CRT_MAH", None, True),
    ("RC_OPTIONS", None, True), ("FRAME_CLASS", 1, True),
    # a second battery monitor (U2 review LOW-6): its failsafes the same as the first, when it exists
    ("BATT2_FS_LOW_ACT", 0, False), ("BATT2_FS_CRT_ACT", 1, False),
]
SERVO_REQUIRED = range(1, 17)                     # SERVO1..16_FUNCTION on every ArduPilot board
SERVO_OPTIONAL = range(17, 33)                    # SERVO17..32 on newer firmware / boards
MOTOR_FUNCS = (33, 34, 35, 36)                    # Motor1..4
OFF_IF_PRESENT = ([("RELAY%d_FUNCTION" % i, 0) for i in range(1, 17)] +          # (U2 review LOW-6: up to 16)
                  [("RELAY%d_PIN" % i, -1) for i in range(1, 17)] +
                  [("RELAY_PIN", -1)] + [("RELAY_PIN%d" % i, -1) for i in range(2, 7)] +
                  [("GRIP_ENABLE", 0), ("CHUTE_ENABLED", 0), ("WINCH_TYPE", 0), ("MNT1_TYPE", 0), ("MNT_TYPE", 0),
                   ("MNT2_TYPE", 0), ("SCR_ENABLE", 0)] +
                  [("RC%d_OPTION" % i, 0) for i in range(1, 17)])
RC_IGNORE_RECEIVER = 1                            # RC_OPTIONS bit 0
# SYS_STATUS sensor bits
S_GYRO, S_ACCEL, S_MAG, S_BARO, S_GPS = 0x1, 0x2, 0x4, 0x8, 0x20
S_PREARM = 0x10000000


def wanted_params():
    """(required names, optional names) the registration reads."""
    req = [n for n, _r, need in FAILSAFE if need] + ["SERVO%d_FUNCTION" % i for i in SERVO_REQUIRED]
    opt = [n for n, _r, need in FAILSAFE if not need] + ["SERVO%d_FUNCTION" % i for i in SERVO_OPTIONAL]
    opt += [n for n, _v in OFF_IF_PRESENT] + ["BATT_CAPACITY"]     # (U3: the route energy check)
    return req, opt


def fw_text(v):
    """AUTOPILOT_VERSION flight_sw_version -> "4.5.7" (major.minor.patch, the type byte = official / beta ...)."""
    if not v:
        return "?"
    return "%d.%d.%d" % ((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF)


def _num(v):
    """A parameter value as a finite float, None when it is not one (U2 review LOW-9: a NaN never raises)."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def check(reading, others=()):
    """Problems (English: protocol / audit; the station translates) of one slot's reading; [] = registrable.
    reading = usbslots.SlotProbe.result(); others = [(slot, sysid)] of the OTHER registered slots."""
    p = []
    if not reading.get("heartbeat"):
        return ["no MAVLink from the slot's USB cable"]
    hb = reading["heartbeat"]
    if hb.get("type") != 2 or hb.get("autopilot") != 3:
        p.append("not an ArduPilot quadcopter (type %s, autopilot %s)" % (hb.get("type"), hb.get("autopilot")))
    if hb.get("base_mode", 0) & 0x80:
        p.append("armed in the slot")
    if not reading.get("version"):
        p.append("no AUTOPILOT_VERSION (firmware / UID unknown)")
    raw = reading.get("params") or {}
    params = {k: _num(v) for k, v in raw.items()} if isinstance(raw, dict) else {}
    bad = sorted(k for k, v in params.items() if v is None)
    if bad:
        p.append("parameters not a number: %s" % ", ".join(bad[:6]))
    params = {k: v for k, v in params.items() if v is not None}
    missing = [n for n in reading.get("required", []) if n not in params]
    if missing:
        p.append("parameters not readable: %s" % ", ".join(missing[:6]) + (" ..." if len(missing) > 6 else ""))
    for name, rule, _need in FAILSAFE:
        if name not in params or rule is None:
            continue
        if int(round(params[name])) != rule:
            p.append("%s is %g (must be %g)" % (name, params[name], rule))
    if "BATT_CRT_VOLT" in params and "BATT_CRT_MAH" in params and \
            not (params["BATT_CRT_VOLT"] > 0 or params["BATT_CRT_MAH"] > 0):
        p.append("no battery critical backstop (BATT_CRT_VOLT and BATT_CRT_MAH are 0)")
    if "RC_OPTIONS" in params and not int(params["RC_OPTIONS"]) & RC_IGNORE_RECEIVER:
        p.append("RC_OPTIONS: the RC receiver is not ignored (bit 0 'Ignore RC Receiver' must be set)")
    motors = []
    for i in list(SERVO_REQUIRED) + list(SERVO_OPTIONAL):
        n = "SERVO%d_FUNCTION" % i
        if n not in params:
            continue
        v = int(round(params[n]))
        if v in MOTOR_FUNCS:
            motors.append(v)
        elif v != 0:
            p.append("output %s = %d: only the 4 motors may be configured" % (n, v))
    if sorted(motors) != list(MOTOR_FUNCS):
        p.append("motor outputs %s (must be Motor1-4 once each)" % sorted(motors))
    for name, off in OFF_IF_PRESENT:
        if name in params and int(round(params[name])) != off:
            p.append("%s = %g (must be %g: no release / aux function)" % (name, params[name], off))
    ss = reading.get("sys_status")
    if not ss:
        p.append("no SYS_STATUS")
    else:
        pres, health = ss.get("onboard_control_sensors_present", 0), ss.get("onboard_control_sensors_health", 0)
        for bit, what in ((S_GYRO, "gyro"), (S_ACCEL, "accelerometer"), (S_BARO, "barometer")):
            if not pres & bit:
                p.append("no %s" % what)
            elif not health & bit:
                p.append("%s not healthy" % what)
        if not pres & S_MAG:
            p.append("no compass")
    gps = reading.get("gps")
    if not gps or gps.get("fix_type", 0) < 1:
        p.append("no GPS receiver")
    sysid = reading.get("sysid")
    if sysid in (0, 255) or (sysid is not None and not 1 <= int(sysid) <= 254):
        p.append("sysid %s is not allowed (0 = broadcast, 255 = a ground station)" % sysid)   # U2 review LOW-6
    for slot, sid in others:
        if sid == sysid:
            p.append("sysid %s is also slot %s" % (sysid, slot))
    return p


def summary(reading):
    """What the station shows of a reading (never the parameters' full list)."""
    v = reading.get("version") or {}
    ss = reading.get("sys_status") or {}
    bs = reading.get("battery") or {}
    gps = reading.get("gps") or {}
    volt = ss.get("voltage_battery")
    pct = bs.get("battery_remaining", ss.get("battery_remaining"))
    return {"sysid": reading.get("sysid"), "compid": reading.get("compid"),
            "uid": ("%016x" % v["uid"]) if v.get("uid") else (bytes(v.get("uid2") or []).hex() or None),
            "fw": fw_text(v.get("flight_sw_version")), "board": v.get("board_version"),
            "battery_pct": pct if pct is not None and pct >= 0 else None,
            "battery_v": round(volt / 1000.0, 2) if volt and volt != 0xFFFF else None,
            "gps_fix": gps.get("fix_type", 0), "gps_sats": gps.get("satellites_visible", 0),
            "prearm": bool(ss.get("onboard_control_sensors_health", 0) & S_PREARM),
            "prearm_text": list(reading.get("prearm", []))[-3:],
            "capacity_mah": _cap((reading.get("params") or {}).get("BATT_CAPACITY"))}


def _cap(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if 100.0 <= v <= 100000.0 else None


class Registry:
    """The registry file (state/uav_registry.json). save() writes in a helper thread (never on the event loop)."""

    def __init__(self, path, slots=6):
        self.path = path
        self.slots = int(slots)
        self.recs = {}
        self.lock = threading.Lock()
        self.error = ""
        if path and os.path.exists(path):
            # U2 review MED-2: a wrongly shaped file (a list, "slots": [], odd keys) never stops the gateway
            try:
                with open(path, encoding="utf-8") as f:
                    d = json.load(f)
                sl = d.get("slots") if isinstance(d, dict) else None
                if not isinstance(sl, dict):
                    raise ValueError("no slots table")
                for k, r in sl.items():
                    try:
                        n = int(k)
                    except (TypeError, ValueError):
                        continue
                    if 1 <= n <= self.slots and isinstance(r, dict):
                        self.recs[n] = r
            except Exception as e:
                self.recs = {}
                self.error = "registry unreadable: %s" % str(e)[:60]

    def others(self, slot):
        return [(s, r.get("sysid")) for s, r in self.recs.items() if s != slot and r.get("ok")]

    def register(self, slot, reading, name=None, vtx=None, elrs=None, now=None):
        """The result of "Register" for one slot -> the record (ok + problems), saved."""
        problems = check(reading, self.others(slot))
        r = dict(summary(reading), slot=slot, ok=not problems, problems=problems,
                 name=name or "UAV-%d" % slot, vtx=vtx, elrs=elrs, checked=round(now or time.time(), 1))
        self.recs[slot] = r
        self.save()
        return r

    def unregister(self, slot):
        self.recs.pop(slot, None)
        self.save()

    def launchable(self, slot, live=None):
        """(ok, why): registered + passed its check + the drone ON its slot cable now (live = what the router sees
        there; None = nothing there: U2 review MED-1) + the same drone (sysid, and the UID once the router knows it)."""
        r = self.recs.get(slot)
        if not r:
            return False, "not registered"
        if not r.get("ok"):
            return False, "check failed: %s" % "; ".join(r.get("problems", [])[:2])
        if any(sid == r.get("sysid") for _s, sid in self.others(slot)):
            return False, "duplicate sysid %s" % r.get("sysid")
        if live is None:
            return False, "no drone on its slot cable"
        if True:
            if live.get("sysid") is not None and live.get("sysid") != r.get("sysid"):
                return False, "unknown drone in the slot (sysid %s, registered %s): register again" % (
                    live.get("sysid"), r.get("sysid"))
            if live.get("uid") and r.get("uid") and live.get("uid") != r.get("uid"):
                return False, "unknown drone in the slot (another UID): register again"
        return True, ""

    def save(self):
        if not self.path:
            return
        data = json.dumps({"version": 1, "slots": {str(k): v for k, v in sorted(self.recs.items())}}, indent=1)
        self._gen = gen = getattr(self, "_gen", 0) + 1

        def write():
            with self.lock:
                if gen < getattr(self, "_written", 0):
                    return                              # a newer save already went to disk
                self._written = gen
                tmp = self.path + ".tmp"
                for k in range(10):
                    # (U6 gate: on Windows os.replace fails while a reader holds the file open - a sharing
                    # violation; retried for ~1 s, never a lost record)
                    try:
                        with open(tmp, "w", encoding="utf-8") as f:
                            f.write(data)
                        os.replace(tmp, self.path)
                        self.error = ""
                        break
                    except OSError as e:
                        self.error = "registry not saved: %s" % str(e)[:60]
                        time.sleep(0.1 * (k + 1))
        threading.Thread(target=write, daemon=True).start()
