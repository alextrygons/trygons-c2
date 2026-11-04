"""UAV magazine on the gateway (CLAUDE.md V, docs/uav_design.md) - step U2: registry + slot USB checks + the read-only
parameter check + the MAVLink allow-list + the own codec + the GPS warm start + the simulated drones.

  UA1  own MAVLink 2 codec: every table message round-trips (wire order, trailing zeros cut, X.25 + CRC_EXTRA),
       the parser counts bad CRC / unknown ids / junk, MAVLink 2 signing (a wrong key / an unsigned frame refused)
  UA2  cross-check with pymavlink (LGPL, optional: this Python or another one on the PATH): every message id,
       CRC_EXTRA, field layout and the encoded bytes are the same; N/A when pymavlink is nowhere
  UA3  ALLOW-list (the scope guard): every other message refused in both directions (PARAM_SET, SET_MODE, mount,
       servo ...), every MAV_CMD 0..65535 but the listed ones refused, mission items only TAKEOFF / WAYPOINT /
       CHANGE_SPEED / LOITER_UNLIM (never DO_SET_SERVO / RELAY / GRIPPER / WINCH / PARACHUTE / MOUNT / DIGICAM), never
       RTL / SmartRTL / LAND mode, force arm / disarm refused, ARM only with the hatch OPEN, DISARM only on the
       ground, LAND only by the battery rule, setpoints = body-frame velocity + yaw rate only, GPS injection only on
       the slot's USB and before ARM, non-finite numbers refused, REQUEST_MESSAGE only for allow-listed messages
  UA4  writer check: only router_worker.py (inside tx(), after allow.check_out) and the simulator call
       mavlink.encode; no other module builds a MAVLink frame; PARAM_SET is never sent anywhere
  UA5  no UAV path to the AURIX: nothing in gateway/uav imports aurix / sends a frame to the AURIX; no payload /
       release / target function name anywhere in the UAV code (station + gateway)
  UA6  the read-only registration check, one finding per case: failsafes (W60), RC receiver not ignored (U1 review
       HIGH-1), every non-motor output / release path (servo function, GPIO, relay function / pin, gripper,
       parachute, winch, mount, scripting, RC aux switch - U1 review MED-1), motors missing, wrong vehicle, armed in
       the slot, parameters not readable, no battery backstop, sensors, duplicate sysid; a good drone passes
  UA7  the registry file: saved + reloaded, launchable rules (not registered, failed, duplicate sysid, an unknown
       drone / UID in a registered slot)
  UA8  GPS warm start bytes: UBX-MGA-INI-TIME_UTC / POS_LLH lengths + checksums, GPS_RTCM_DATA fragments, the boat
       position only from fresh telemetry with a trusted GPS
  UA9  the simulated drone's GPS: no fix in the closed magazine; after the sky: cold > warm (time + position) > hot
       (+ ephemeris); the drone speaks the same MAVLink as the router (arm / takeoff / mission upload + read-back /
       battery backstop LAND / EKF failsafe AltHold)
  UA10 the router process against 6 simulated drones: registration of a good / a bad / a silent slot, the warm
       start reaches the drone, refused frames counted (a drone that sends SERVO_OUTPUT_RAW / MOUNT_CONTROL), a
       frame of another sysid on a registered slot dropped, the GPS injection refused to an armed drone
  UA11 the real gateway + a station: uav_register -> uav_ack (ok / 122 + the problems), a VIEWER refused, gw_status
       "uav" (6 slots, next = slot 1), the registry on the SSD, the warm start sent after the registration, the
       AURIX untouched (same mode, no new alarm), the event loop never blocked, the router killed = OFFLINE notice
       + restarted (no alarm), no process left after the stop
  UA12 osdep: "usbloc:<port path>" = the slot's USB port (never the device's own id), shown by --list-serial

Run:  python test_uav.py      (exit code 0 = all passed)
"""
import ast
import json
import math
import os
import random
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")
os.environ.setdefault("TRYGONS_TEST", "1")
from uav import allow, gpsassist, mavlink, registry, simdrone, usbslots  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
REPO = os.path.dirname(HERE)
UAV_DIR = os.path.join(HERE, "uav")
results = []
LIMIT_S = 900
NA = "N/A"                                       # report(..., NA, ...) = not applicable here (never a falsy result)


def report(name, ok, detail):
    results.append(None if ok is NA else bool(ok))
    print("[%s] %s - %s" % ("N/A " if ok is NA else ("PASS" if ok else "FAIL"), name, detail), flush=True)


def until(cond, timeout, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        v = cond()
        if v:
            return v
        time.sleep(step)
    return None


def free_base():
    """A base port with base + 1..16 and + 30 free (simulated drones)."""
    for _ in range(200):
        b = random.randrange(20000, 40000, 50)
        try:
            socks = []
            for k in list(range(1, 17)) + [30]:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(("127.0.0.1", b + k))
                socks.append(s)
            for s in socks:
                s.close()
            return b
        except OSError:
            for s in socks:
                s.close()
    raise RuntimeError("no free UDP ports")


def sample(name, rnd):
    """Random but valid field values for one message."""
    out = {}
    for f, t, n in mavlink.MESSAGES[name][2]:
        def one():
            if t in "bB":
                return rnd.randrange(-128, 128) if t == "b" else rnd.randrange(256)
            if t in "hH":
                return rnd.randrange(-32768, 32768) if t == "h" else rnd.randrange(65536)
            if t in "iI":
                return rnd.randrange(-2 ** 31, 2 ** 31) if t == "i" else rnd.randrange(2 ** 32)
            if t in "qQ":
                return rnd.randrange(-2 ** 63, 2 ** 63) if t == "q" else rnd.randrange(2 ** 64)
            if t == "f":
                return struct.unpack("<f", struct.pack("<f", rnd.uniform(-1000, 1000)))[0]
            raise ValueError(t)
        if t == "s":
            out[f] = "".join(rnd.choice("ABCDEFGH_0123") for _ in range(rnd.randrange(1, n + 1)))
        elif n > 1:
            out[f] = [one() for _ in range(n)]
        else:
            out[f] = one()
    return out


# ------------------------------------------------------------------------------------------------- UA1 / UA2 -------

def ua1_codec():
    rnd = random.Random(1)
    bad, n = [], 0
    for name in mavlink.MESSAGES:
        for k in range(20):
            f = sample(name, rnd)
            if k == 0:                                       # all zeros = the payload cut to one byte
                f = {x: (0 if c == 1 and t != "s" else ("" if t == "s" else [0] * c))
                     for x, t, c in mavlink.MESSAGES[name][2]}
            fr = mavlink.encode(name, f, k, 7, 1)
            got = mavlink.Parser().feed(fr)
            n += 1
            if len(got) != 1 or got[0].name != name or got[0].fields != f or got[0].sysid != 7:
                bad.append(name)
            if k == 0 and fr[1] != 1:
                bad.append(name + " (zero payload not cut)")
    p = mavlink.Parser()
    fr = bytearray(mavlink.encode("HEARTBEAT", {"type": 2}, 1, 1, 1))
    fr[-1] ^= 0xFF
    unknown = bytes([0xFD, 2, 0, 0, 0, 1, 1, 0x39, 0x30, 0, 1, 2, 0, 0])      # msg id 12345
    out = p.feed(b"junk" + bytes(fr) + unknown + mavlink.encode("HEARTBEAT", {"type": 2}, 2, 1, 1))
    key = bytes(range(32))
    signed = mavlink.encode("HEARTBEAT", {"type": 2}, 3, 1, 1, key=key, link_id=1, timestamp=123456)
    ok_sig = mavlink.Parser(key).feed(signed)
    wrong = mavlink.Parser(bytes(32)).feed(signed)
    pk = mavlink.Parser(key)
    unsigned = pk.feed(mavlink.encode("HEARTBEAT", {"type": 2}, 4, 1, 1))
    report("UA1 own MAVLink 2 codec: every message round-trips (wire order, trailing zeros cut, X.25 + CRC_EXTRA); "
           "bad CRC / unknown id / junk counted; signing: the right key passes, a wrong key and an unsigned frame "
           "are refused",
           not bad and len(out) == 1 and p.n["crc"] >= 1 and p.n["unknown"] >= 1 and len(ok_sig) == 1 and
           ok_sig[0].signed and not wrong and not unsigned and pk.n["unsigned"] == 1,
           "%d frames of %d messages, wrong %s; parser counts %s; signed ok %d, wrong key %d, unsigned %d" % (
               n, len(mavlink.MESSAGES), bad[:4], p.n, len(ok_sig), len(wrong), len(unsigned)))


UA2_SCRIPT = r'''
import json, sys
from pymavlink.dialects.v20 import ardupilotmega as m
req = json.loads(sys.stdin.read())
out = {}
mav = m.MAVLink(None, srcSystem=7, srcComponent=1)
for name, f in req.items():
    c = getattr(m, "MAVLink_%s_message" % name.lower())
    args = []
    for fn in c.fieldnames:
        v = f[fn]
        if isinstance(v, str):
            v = v.encode()
        args.append(v)
    msg = c(*args)
    mav.seq = 5
    b = msg.pack(mav)
    out[name] = {"id": c.id, "crc": c.crc_extra, "fmt": c.unpacker.format if hasattr(c, "unpacker") else c.format,
                 "order": list(c.ordered_fieldnames), "frame": bytes(b).hex()}
print(json.dumps(out))
'''


def ua2_pymavlink():
    rnd = random.Random(2)
    req = {name: sample(name, rnd) for name in mavlink.MESSAGES}
    pys = []
    try:
        import pymavlink  # noqa: F401
        pys.append(sys.executable)
    except ImportError:
        pass
    for cand in ("python", "py", "python3"):
        p = shutil.which(cand)
        if p and p not in pys:
            pys.append(p)
    res, used = None, None
    for py in pys:
        try:
            r = subprocess.run([py, "-c", UA2_SCRIPT], input=json.dumps(req), capture_output=True, text=True,
                               timeout=60)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if r.returncode == 0 and r.stdout.strip().startswith("{"):
            res, used = json.loads(r.stdout), py
            break
    if res is None:
        return report("UA2 codec cross-check with pymavlink", NA, "pymavlink not installed in any Python here "
                      "(optional: pip install pymavlink)")
    bad = []
    for name, (mid, extra, fields) in mavlink.MESSAGES.items():
        r = res[name]
        fmt = "<" + "".join(("%d%s" % (n, t)) if n > 1 or t == "s" else t for _f, t, n in fields)
        mine = mavlink.encode(name, req[name], 5, 7, 1).hex()
        if (r["id"], r["crc"], r["fmt"], r["order"]) != (mid, extra, fmt, [f for f, _t, _n in fields]):
            bad.append(name + " table")
        elif r["frame"] != mine:
            bad.append(name + " bytes")
    report("UA2 the codec = pymavlink (official XML): message ids, CRC_EXTRA, field order / types and the encoded "
           "frames of every table message", not bad,
           "%d messages compared (pymavlink in %s); different: %s" % (len(res), used, bad))


# ------------------------------------------------------------------------------------------------------ UA3 --------

def ua3_allow():
    bad = []

    def want(ok_expected, name, fields, ctx=None, label=""):
        ok, why = allow.check_out(name, fields, ctx or {})
        if ok != ok_expected:
            bad.append("%s %s %s -> %s %s" % (label or name, fields.get("command", ""), ctx, ok, why))
    # messages
    for name in mavlink.MESSAGES:
        if name not in allow.TO_DRONE:
            want(False, name, {}, label="out " + name)
        if name not in allow.FROM_DRONE and allow.check_in(name)[0]:
            bad.append("in " + name)
    for name in ("PARAM_SET", "SET_MODE", "SERVO_OUTPUT_RAW", "MOUNT_CONTROL", "RC_CHANNELS_OVERRIDE",
                 "DIGICAM_CONTROL", "MANUAL_CONTROL"):
        if name in allow.TO_DRONE:
            bad.append("listed " + name)
        want(False, name, {})
    # every command id
    land_ctx = {"battery_land": True}
    allowed_cmds = []
    for cmd in range(65536):
        for nm in ("COMMAND_LONG", "COMMAND_INT"):
            ok, _w = allow.check_out(nm, {"command": cmd, "param1": 1.0, "param2": 4.0},
                                     {"hatch_open": True, "landed": True, "battery_land": True})
            if ok:
                allowed_cmds.append(cmd)
    allowed_cmds = sorted(set(allowed_cmds))
    if not set(allowed_cmds) <= allow.COMMANDS:
        bad.append("commands %s" % allowed_cmds)
    # every mission item command
    m_ok = sorted({c for c in range(65536) for fr in (2, 3, 6)
                   if allow.check_out("MISSION_ITEM_INT", {"command": c, "frame": fr, "param2": 5.0})[0]})
    if set(m_ok) != allow.MISSION_COMMANDS:
        bad.append("mission %s" % m_ok)
    for c in (183, 184, 181, 182, 211, 42600, 42601, 208, 205, 203, 206, 20, 21, 31010, 191, 3000):
        want(False, "MISSION_ITEM_INT", {"command": c, "frame": 3}, label="mission")
    want(False, "MISSION_ITEM_INT", {"command": 16, "frame": 0}, label="wp frame")
    want(True, "MISSION_ITEM_INT", {"command": 16, "frame": 3, "z": 50.0})
    want(False, "MISSION_ITEM_INT", {"command": 16, "frame": 3, "z": float("nan")}, label="nan")
    # modes
    for mode in range(32):
        want(mode in allow.MODES, "COMMAND_LONG", {"command": 176, "param1": 1.0, "param2": float(mode)},
             label="mode %d" % mode)
    for mode in (allow.COPTER_RTL, allow.COPTER_SMART_RTL, allow.COPTER_LAND):
        if mode in allow.MODES:
            bad.append("mode %d listed" % mode)
    # arm / disarm
    want(False, "COMMAND_LONG", {"command": 400, "param1": 1.0}, {}, "arm without the hatch")
    want(True, "COMMAND_LONG", {"command": 400, "param1": 1.0}, {"hatch_open": True})
    want(False, "COMMAND_LONG", {"command": 400, "param1": 1.0, "param2": 21196.0}, {"hatch_open": True}, "force arm")
    want(False, "COMMAND_LONG", {"command": 400, "param1": 0.0, "param2": 21196.0}, {"landed": True}, "force disarm")
    want(False, "COMMAND_LONG", {"command": 400, "param1": 0.0}, {"landed": False}, "disarm flying")
    want(True, "COMMAND_LONG", {"command": 400, "param1": 0.0}, {"landed": True})
    want(False, "COMMAND_LONG", {"command": 21}, {}, "land without the battery rule")
    want(True, "COMMAND_LONG", {"command": 21}, land_ctx)
    want(False, "COMMAND_LONG", {"command": 178, "param2": 99.0}, {}, "speed")
    want(False, "COMMAND_LONG", {"command": 22, "param7": float("inf")}, {}, "inf")
    want(False, "COMMAND_LONG", {"command": 512, "param1": 36.0}, {}, "request SERVO_OUTPUT_RAW")
    want(True, "COMMAND_LONG", {"command": 512, "param1": 148.0})
    want(False, "COMMAND_LONG", {"command": 511, "param1": 157.0}, {}, "interval MOUNT_CONTROL")
    # setpoints
    vel = allow.MASK_MUST_SET
    want(True, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel, "vx": 1.0})
    want(True, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 9, "type_mask": vel | 0x800})
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 1, "type_mask": vel}, {}, "local frame")
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel & ~0x7}, {}, "position")
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel & ~0x1C0}, {}, "accel")
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel | 0x38}, {}, "no velocity")
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel, "vx": float("nan")}, {},
         "nan setpoint")
    # GPS injection
    for nm in ("GPS_RTCM_DATA", "GPS_INJECT_DATA"):
        want(True, nm, {}, {"port": "usb", "armed": False})
        want(False, nm, {}, {"port": "elrs", "armed": False}, nm + " over ELRS")
        want(False, nm, {}, {"port": "usb", "armed": True}, nm + " armed")
    report("UA3 ALLOW-list: every other message refused both ways (PARAM_SET, SET_MODE, mount, servo, RC override), "
           "only the listed MAV_CMDs (all 65536 tried), mission items only TAKEOFF / WAYPOINT / CHANGE_SPEED / "
           "LOITER_UNLIM, never RTL / SmartRTL / LAND mode, no force arm / disarm, ARM only with the hatch OPEN, "
           "DISARM only landed, LAND only by the battery rule, body-frame velocity setpoints only, GPS injection only "
           "on USB before ARM, non-finite numbers refused", not bad,
           "commands that pass: %s; mission commands %s; wrong: %s" % (allowed_cmds, m_ok, bad[:6]))


# ------------------------------------------------------------------------------------------------------ UA4 / 5 ----

def py_files(folders):
    for d in folders:
        for root, _dirs, files in os.walk(d):
            if "__pycache__" in root or os.sep + "bin" in root or "site-packages" in root:
                continue
            for f in files:
                if f.endswith(".py") and not f.startswith("test_"):
                    yield os.path.join(root, f)


def ua4_writer():
    callers, frames, pset = [], [], []
    for p in py_files([HERE, os.path.join(REPO, "groundstation"), os.path.join(REPO, "academy"),
                       os.path.join(REPO, "demo"), os.path.join(REPO, "tools")]):
        rel = os.path.relpath(p, REPO).replace("\\", "/")
        src = open(p, encoding="utf-8", errors="replace").read()
        if re.search(r"mavlink\.encode\s*\(", src) or (rel != "gateway/uav/mavlink.py" and
                                                       re.search(r"\bencode\s*\(\s*\"[A-Z_]+\"", src)):
            callers.append(rel)
        if rel != "gateway/uav/mavlink.py" and re.search(r"0x[fF][dD]\b|\\xfd", src) and "uav" in rel:
            frames.append(rel)
        if '"PARAM_SET"' in src and rel not in ("gateway/uav/mavlink.py",):
            pset.append(rel)
    # inside router_worker.py: encode only in Router.tx, after allow.check_out
    tree = ast.parse(open(os.path.join(UAV_DIR, "router_worker.py"), encoding="utf-8").read())
    in_tx, ok_order = [], False
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            src = ast.get_source_segment(open(os.path.join(UAV_DIR, "router_worker.py"), encoding="utf-8").read(),
                                         node) or ""
            if "mavlink.encode(" in src:
                in_tx.append(node.name)
                if node.name == "tx":
                    ok_order = src.index("allow.check_out(") < src.index("mavlink.encode(")
    report("UA4 writer check: only the router's tx() (after allow.check_out) and the simulator encode MAVLink; no "
           "other module builds a frame; PARAM_SET never sent",
           sorted(callers) == ["gateway/uav/router_worker.py", "gateway/uav/simdrone.py"] and in_tx == ["tx"] and
           ok_order and not frames and not pset,
           "encode callers %s, inside %s (check first %s), frame builders %s, PARAM_SET in %s" % (
               sorted(callers), in_tx, ok_order, frames, pset))


def ua5_no_aurix():
    bad, names = [], []
    files = list(py_files([UAV_DIR])) + [os.path.join(REPO, "groundstation", f) for f in ("uavpanel.py", "uavpics.py")]
    for p in files:
        src = open(p, encoding="utf-8").read()
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                mods = [a.name for a in node.names] + ([node.module] if isinstance(node, ast.ImportFrom) and
                                                       node.module else [])
                if any(m and m.split(".")[0] in ("aurix", "proto", "stations", "link") for m in mods):
                    bad.append("%s imports %s" % (os.path.basename(p), mods))
            if isinstance(node, ast.Attribute) and node.attr in ("aurix",):
                bad.append("%s uses .aurix" % os.path.basename(p))
            if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and re.search(
                    r"payload|release|drop|grip|winch|strike|target|bomb|weapon|chute", node.name, re.I):
                names.append("%s:%s" % (os.path.basename(p), node.name))
    report("UA5 no UAV path to the AURIX (no gateway/uav module imports aurix / proto / stations or touches "
           ".aurix) and no payload / release / target function anywhere in the UAV code", not bad and not names,
           "%d files; AURIX paths %s; suspicious names %s" % (len(files), bad, names))


# ------------------------------------------------------------------------------------------------------ UA6 / 7 ----

def good_reading(sysid=11, params=None, **kw):
    p = simdrone.good_params()
    if params:
        p.update(params)
    req, _opt = registry.wanted_params()
    r = {"slot": 1, "sysid": sysid, "compid": 1,
         "heartbeat": {"type": 2, "autopilot": 3, "base_mode": 1, "custom_mode": 5},
         "version": {"flight_sw_version": simdrone.FW, "uid": 0xABC},
         "sys_status": {"onboard_control_sensors_present": simdrone.SENS,
                        "onboard_control_sensors_health": simdrone.SENS, "voltage_battery": 16000,
                        "battery_remaining": 90},
         "gps": {"fix_type": 1, "satellites_visible": 0}, "params": p, "required": req, "prearm": []}
    r.update(kw)
    return r


def ua6_check():
    cases = [
        ("FS_GCS_ENABLE 1", {"FS_GCS_ENABLE": 1}, "FS_GCS_ENABLE is 1"),
        ("FS_THR_ENABLE 1", {"FS_THR_ENABLE": 1}, "FS_THR_ENABLE is 1"),
        ("FS_EKF_ACTION land", {"FS_EKF_ACTION": 1}, "FS_EKF_ACTION is 1"),
        ("BATT_FS_LOW_ACT RTL", {"BATT_FS_LOW_ACT": 2}, "BATT_FS_LOW_ACT is 2"),
        ("BATT_FS_CRT_ACT SmartRTL", {"BATT_FS_CRT_ACT": 3}, "BATT_FS_CRT_ACT is 3"),
        ("FENCE_ACTION RTL", {"FENCE_ACTION": 1}, "FENCE_ACTION is 1"),
        ("no backstop", {"BATT_CRT_VOLT": 0, "BATT_CRT_MAH": 0}, "no battery critical backstop"),
        ("RC not ignored", {"RC_OPTIONS": 0}, "RC receiver is not ignored"),
        ("gripper output", {"SERVO9_FUNCTION": 28}, "SERVO9_FUNCTION = 28"),
        ("GPIO output", {"SERVO10_FUNCTION": -1}, "SERVO10_FUNCTION = -1"),
        ("servo 17 output", {"SERVO17_FUNCTION": 1}, "SERVO17_FUNCTION = 1"),
        ("relay function", {"RELAY1_FUNCTION": 1}, "RELAY1_FUNCTION = 1"),
        ("relay pin", {"RELAY2_PIN": 13}, "RELAY2_PIN = 13"),
        ("old relay pin", {"RELAY_PIN": 54}, "RELAY_PIN = 54"),
        ("gripper", {"GRIP_ENABLE": 1}, "GRIP_ENABLE = 1"),
        ("parachute", {"CHUTE_ENABLED": 1}, "CHUTE_ENABLED = 1"),
        ("winch", {"WINCH_TYPE": 1}, "WINCH_TYPE = 1"),
        ("mount", {"MNT1_TYPE": 1}, "MNT1_TYPE = 1"),
        ("scripting", {"SCR_ENABLE": 1}, "SCR_ENABLE = 1"),
        ("aux switch", {"RC7_OPTION": 19}, "RC7_OPTION = 19"),
        ("motor missing", {"SERVO4_FUNCTION": 0}, "motor outputs"),
        ("not a quad frame", {"FRAME_CLASS": 2}, "FRAME_CLASS is 2"),
    ]
    bad = []
    base = registry.check(good_reading())
    if base:
        bad.append("good drone: %s" % base)
    for label, params, expect in cases:
        pr = registry.check(good_reading(params=params))
        if not any(expect in p for p in pr):
            bad.append("%s -> %s" % (label, pr))
    p = simdrone.good_params()
    del p["FS_GCS_ENABLE"]
    other = [
        ("missing parameter", dict(good_reading(), params=p), "parameters not readable"),
        ("a plane", good_reading(heartbeat={"type": 1, "autopilot": 3, "base_mode": 1}), "not an ArduPilot quad"),
        ("armed", good_reading(heartbeat={"type": 2, "autopilot": 3, "base_mode": 0x81}), "armed in the slot"),
        ("no version", good_reading(version=None), "no AUTOPILOT_VERSION"),
        ("bad gyro", good_reading(sys_status={"onboard_control_sensors_present": simdrone.SENS,
                                              "onboard_control_sensors_health": simdrone.SENS & ~1}),
         "gyro not healthy"),
        ("no compass", good_reading(sys_status={"onboard_control_sensors_present": simdrone.SENS & ~4,
                                                "onboard_control_sensors_health": simdrone.SENS}), "no compass"),
        ("no GPS", good_reading(gps={"fix_type": 0}), "no GPS receiver"),
        ("silent", {"slot": 1}, "no MAVLink"),
    ]
    for label, rd, expect in other:
        pr = registry.check(rd)
        if not any(expect in x for x in pr):
            bad.append("%s -> %s" % (label, pr))
    dup = registry.check(good_reading(sysid=12), others=[(3, 12)])
    if not any("also slot 3" in x for x in dup):
        bad.append("duplicate -> %s" % dup)
    report("UA6 registration check (read only): failsafes, the RC receiver ignored (U1 review HIGH-1), only the 4 "
           "motors + every release path off (servo / GPIO / relay function + pin / gripper / parachute / winch / "
           "mount / scripting / RC aux switch - U1 review MED-1), vehicle type, armed in the slot, unreadable "
           "parameters, no backstop, sensors, duplicate sysid; a good drone passes",
           not bad, "%d cases; wrong: %s" % (len(cases) + len(other) + 2, bad[:5]))


def ua7_registry():
    tmp = tempfile.mkdtemp(prefix="uavtest_reg_")
    try:
        path = os.path.join(tmp, "uav_registry.json")
        r = registry.Registry(path)
        a = r.register(1, good_reading(sysid=11), name="EAGLE")
        b = r.register(2, good_reading(sysid=12, params={"GRIP_ENABLE": 1}))
        c = r.register(3, good_reading(sysid=11))                    # duplicate sysid of slot 1
        until(lambda: os.path.exists(path) and json.load(open(path, encoding="utf-8")).get("slots", {}).get("3"), 5)
        r2 = registry.Registry(path)
        l1 = r2.launchable(1, {"sysid": 11})
        l2 = r2.launchable(2)
        l3 = r2.launchable(3)
        l4 = r2.launchable(4)
        l5 = r2.launchable(1, {"sysid": 19})
        l6 = r2.launchable(1, {"sysid": 11, "uid": "ffff"})
        ok = (a["ok"] and not b["ok"] and not c["ok"] and r2.recs[1]["name"] == "EAGLE" and l1 == (True, "") and
              not l2[0] and "check failed" in l2[1] and not l3[0] and not l4[0] and l4[1] == "not registered" and
              not l5[0] and "unknown drone" in l5[1] and not l6[0] and "another UID" in l6[1])
        report("UA7 the registry file (state/uav_registry.json): saved + reloaded; launchable only registered + "
               "checked + the same drone (sysid / UID) in the slot; a duplicate sysid never launchable", ok,
               "slot 1 %s, 2 %s, 3 %s, 4 %s, unknown sysid %s, unknown UID %s" % (l1, l2, l3, l4, l5, l6))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------------------------------------------ UA8 --------

def ua8_assist():
    t = 1793000000.25
    tm = gpsassist.mga_ini_time_utc(t)
    pos = gpsassist.mga_ini_pos_llh(37.82, 23.60, 0.0, 15.0)
    msgs = gpsassist.parse_ubx(b"\x00junk" + tm + pos)
    ok_time = msgs and msgs[0][:2] == (0x13, 0x40) and len(msgs[0][2]) == 24 and msgs[0][2][0] == 0x10
    year, mon, day, hh, mm, ss = struct.unpack_from("<HBBBBB", msgs[0][2], 4)
    ns = struct.unpack_from("<I", msgs[0][2], 12)[0]
    g = time.gmtime(int(t))
    ok_tv = (year, mon, day, hh, mm, ss) == (g.tm_year, g.tm_mon, g.tm_mday, g.tm_hour, g.tm_min, g.tm_sec) and \
        abs(ns - 250000000) < 1000
    ok_pos = len(msgs) == 2 and len(msgs[1][2]) == 20 and msgs[1][2][0] == 0x01 and \
        struct.unpack_from("<iiiI", msgs[1][2], 4) == (378200000, 236000000, 0, 1500)
    corrupt = bytearray(tm)
    corrupt[10] ^= 1
    ok_ck = gpsassist.parse_ubx(bytes(corrupt)) == []
    one = gpsassist.rtcm_frames(tm, 3)
    big = gpsassist.rtcm_frames(bytes(400), 5)
    exact = gpsassist.rtcm_frames(bytes(360), 6)
    ok_fr = (len(one) == 1 and one[0]["flags"] == 3 << 3 and one[0]["len"] == len(tm) and
             [f["flags"] & 7 for f in big] == [1, 3, 5] and [f["len"] for f in big] == [180, 180, 40] and
             [f["len"] for f in exact] == [180, 180, 0])
    try:
        gpsassist.rtcm_frames(bytes(800), 1)
        ok_fr = False
    except ValueError:
        pass
    tel = {"pos_source": 1, "est_lat": 37.8, "est_lon": 23.6, "unc_m": 8.0}
    fixes = [gpsassist.boat_fix(tel, True), gpsassist.boat_fix(tel, False), gpsassist.boat_fix(dict(tel, pos_source=2),
             True), gpsassist.boat_fix(dict(tel, unc_m=500.0), True), gpsassist.boat_fix(dict(tel, est_lat=float("nan")),
                                                                                         True)]
    blob, what = gpsassist.assist_bytes(t, (37.8, 23.6, 3.0))
    p2 = gpsassist.parse_ubx(blob)
    acc = struct.unpack_from("<I", p2[1][2], 16)[0]
    report("UA8 GPS warm start bytes: UBX-MGA-INI-TIME_UTC (24 B, the gateway's time) + POS_LLH (20 B, 1e-7 deg, "
           "accuracy >= 10 m), checksums, GPS_RTCM_DATA fragments (<= 4 x 180 B); the boat position only from fresh "
           "telemetry with a trusted GPS and a sane uncertainty",
           ok_time and ok_tv and ok_pos and ok_ck and ok_fr and fixes[0] == (37.8, 23.6, 8.0) and
           fixes[1:] == [None, None, None, None] and what == "time+pos" and acc == 1000,
           "time %s / values %s, pos %s, checksum %s, fragments %s, boat fixes %s, %s acc %d cm" % (
               ok_time, ok_tv, ok_pos, ok_ck, ok_fr, fixes, what, acc))


# ------------------------------------------------------------------------------------------------------ UA9 --------

class Gcs:
    """A test GCS on one simulated drone link (tests only - the gateway's own frames go through the router)."""

    def __init__(self, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", 0))
        self.s.settimeout(0.05)
        self.addr = ("127.0.0.1", port)
        self.p = mavlink.Parser()
        self.seq = 0
        self.msgs = []

    def send(self, name, f):
        self.seq += 1
        self.s.sendto(mavlink.encode(name, f, self.seq, 255, 190), self.addr)

    def pump(self, t=0.2):
        end = time.time() + t
        while time.time() < end:
            try:
                d, _a = self.s.recvfrom(4096)
                self.msgs += self.p.feed(d)
            except socket.timeout:
                pass

    def last(self, name):
        for m in reversed(self.msgs):
            if m.name == name:
                return m
        return None

    def cmd(self, c, *p, t=0.5):
        self.send("COMMAND_LONG", dict({"command": c, "target_system": 0}, **{"param%d" % (i + 1): float(v)
                                                                              for i, v in enumerate(p)}))
        end = time.time() + t
        while time.time() < end:
            self.pump(0.05)
            for m in reversed(self.msgs):
                if m.name == "COMMAND_ACK" and m["command"] == c:
                    self.msgs.remove(m)
                    return m["result"]
        return None


def ua9_simdrone():
    base = free_base()
    fl = simdrone.Fleet(base, 4, ttf=(6.0, 3.0, 1.0)).start()
    try:
        g = [Gcs(base + 1 + i) for i in range(4)]
        for x in g:
            x.send("HEARTBEAT", {"type": 6, "autopilot": 8})
        time.sleep(0.3)
        now = time.time()
        # drone 2: time + position (warm), drone 3: + ephemeris (hot), drone 1: nothing (cold)
        tm, ps = gpsassist.mga_ini_time_utc(now), gpsassist.mga_ini_pos_llh(37.82, 23.6, 0, 20)
        eph = gpsassist.ubx(0x13, 0x00, bytes(68))                     # an MGA-GPS-EPH sized message
        for k, blobs in ((1, [tm, ps]), (2, [tm, ps, eph])):
            for i, b in enumerate(blobs):
                for fr in gpsassist.rtcm_frames(b, i):
                    g[k].send("GPS_RTCM_DATA", fr)
        g[0].pump(0.5)
        in_slot_fix = [d.fix for d in fl.drones[:3]]
        t0 = time.time()
        for d in fl.drones[:3]:
            d.sky = True                                               # the hatch opens
        first = {}
        while time.time() - t0 < 9 and len(first) < 3:
            for i, d in enumerate(fl.drones[:3]):
                if d.fix >= 3 and i not in first:
                    first[i] = round(time.time() - t0, 1)
            time.sleep(0.05)
        ok_gps = (in_slot_fix == [1, 1, 1] and len(first) == 3 and first[2] < first[1] < first[0] and
                  fl.drones[1].assist.get("pos") and fl.drones[2].assist.get("eph"))
        # flight behaviour on drone 1 (direct test frames over its ELRS link: the USB cable goes when it climbs)
        x = Gcs(base + 11)
        x.send("HEARTBEAT", {"type": 6, "autopilot": 8})
        x.pump(0.3)
        mode_ok = x.cmd(176, 1, 4)                                     # GUIDED
        arm = x.cmd(400, 1)
        tko = x.cmd(22, 0, 0, 0, 0, 0, 0, 12)
        climbed = until(lambda: fl.drones[0].alt > 5.0, 8)
        disarm_air = x.cmd(400, 0)                                     # refused in flight (not landed)
        items = [{"seq": 0, "command": 22, "frame": 3, "z": 20.0},
                 {"seq": 1, "command": 16, "frame": 3, "x": 378210000, "y": 236000000, "z": 30.0},
                 {"seq": 2, "command": 17, "frame": 3, "x": 378210000, "y": 236000000, "z": 30.0}]
        x.send("MISSION_COUNT", {"count": len(items)})
        for it in items:
            x.pump(0.2)
            x.send("MISSION_ITEM_INT", it)
        x.pump(0.4)
        ack = x.last("MISSION_ACK")
        x.send("MISSION_REQUEST_LIST", {})
        x.pump(0.3)
        cnt = x.last("MISSION_COUNT")
        x.send("MISSION_REQUEST_INT", {"seq": 1})
        x.pump(0.3)
        rb = x.last("MISSION_ITEM_INT")
        usb_gone = not fl.drones[0].in_slot
        # battery backstop + EKF failsafe on drone 1
        fl.drones[0].used_mah = fl.drones[0].capacity * 0.99
        landing = until(lambda: fl.drones[0].mode == simdrone.M_LAND, 3)
        fl.drones[0].used_mah = 0.0
        d3 = fl.drones[2]                                              # flying in GUIDED, then GPS lost
        y = g[2]
        y.cmd(176, 1, 4)
        y.cmd(400, 1)
        y.cmd(22, 0, 0, 0, 0, 0, 0, 10)
        until(lambda: d3.alt > 3.0, 6)
        d3.fault("no_fix")
        althold = until(lambda: d3.mode == simdrone.M_ALT_HOLD, 3)
        ok_fly = (mode_ok == 0 and arm == 0 and tko == 0 and climbed and disarm_air == 4 and ack and
                  ack["type"] == 0 and cnt and cnt["count"] == 3 and rb and rb["x"] == 378210000 and usb_gone and
                  landing and althold)
        report("UA9 simulated drone: no GPS fix in the closed magazine; after the hatch opens cold > warm (time + "
               "position injected) > hot (+ ephemeris); the same MAVLink behaviour as ArduCopter (GUIDED, ARM, "
               "TAKEOFF, no DISARM in flight, mission upload + read-back, the USB cable gone once airborne, battery "
               "critical = LAND (the backstop), GPS lost in flight = ALT_HOLD (EKF failsafe))", ok_gps and ok_fly,
               "fix in slot %s; first fix after the sky (s) cold %s warm %s hot %s; mode %s arm %s takeoff %s "
               "climbed %s disarm in air %s, mission ack %s count %s read-back %s, USB gone %s, LAND %s, ALT_HOLD %s" % (
                   in_slot_fix, first.get(0), first.get(1), first.get(2), mode_ok, arm, tko, bool(climbed), disarm_air,
                   ack and ack["type"], cnt and cnt["count"], rb and rb["x"], usb_gone, bool(landing), bool(althold)))
    finally:
        fl.stop()


# ------------------------------------------------------------------------------------------------------ UA10 -------

class Worker:
    def __init__(self):
        self.p = subprocess.Popen([sys.executable, os.path.join(UAV_DIR, "router_worker.py")], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, cwd=UAV_DIR)
        self.ev = []
        self.lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            try:
                with self.lock:
                    self.ev.append(json.loads(line))
            except ValueError:
                pass

    def send(self, obj):
        self.p.stdin.write((json.dumps(obj) + "\n").encode())
        self.p.stdin.flush()

    def find(self, cond, timeout):
        def f():
            with self.lock:
                for e in reversed(self.ev):
                    if cond(e):
                        return e
            return None
        return until(f, timeout)

    def close(self):
        try:
            self.p.stdin.close()
        except OSError:
            pass
        try:
            self.p.wait(5)
        except subprocess.TimeoutExpired:
            self.p.kill()
            self.p.wait(5)


def ua10_router():
    base = free_base()
    fl = simdrone.Fleet(base, 6, faults=[(2, "extra_output"), (3, "silent"), (4, "spam_denied"),
                                         (5, "armed")]).start()
    w = Worker()
    try:
        w.send({"op": "config", "slots": [{"slot": i, "usb": "udp:127.0.0.1:%d" % (base + i)} for i in range(1, 7)]})
        w.find(lambda e: e.get("ev") == "status" and e["links"].get("1", {}).get("present"), 6)
        for s in (1, 2, 3):
            w.send({"op": "register", "slot": s, "id": s})
        regs = {}
        for s in (1, 2, 3):
            e = w.find(lambda e, s=s: e.get("ev") == "register" and e.get("slot") == s, 25)
            regs[s] = registry.check(e["reading"]) if e else ["no answer"]
        w.send({"op": "assist", "slot": 1, "utc": time.time(), "pos": [37.8, 23.6, 10.0]})
        a1 = w.find(lambda e: e.get("ev") == "assist" and e.get("slot") == 1, 5)
        got = until(lambda: fl.drones[0].assist.get("time") and fl.drones[0].assist.get("pos"), 3)
        w.send({"op": "assist", "slot": 5, "utc": time.time(), "pos": None})
        a5 = w.find(lambda e: e.get("ev") == "assist" and e.get("slot") == 5, 5)
        time.sleep(1.5)
        st = w.find(lambda e: e.get("ev") == "status" and e["stats"].get("denied_in", 0) > 0, 5)
        # slot 1 registered as sysid 11: another sysid on that cable = dropped
        w.send({"op": "config", "slots": [{"slot": i, "usb": "udp:127.0.0.1:%d" % (base + i),
                                           "sysid": 11 if i == 1 else None} for i in range(1, 7)]})
        time.sleep(0.5)
        fl.drones[0].sysid = 41
        st2 = w.find(lambda e: e.get("ev") == "status" and e["stats"].get("wrong_sysid", 0) > 2, 6)
        fl.drones[0].sysid = 11
        report("UA10 the router process (6 simulated drones over UDP): a good slot registers, a gripper output / a "
               "silent cable are refused with the reason, the warm start (time + position) reaches the drone in its "
               "slot, the GPS injection is refused to an armed drone, frames not on the allow-list (SERVO_OUTPUT_RAW, "
               "MOUNT_CONTROL) and of another sysid on a registered slot are dropped + counted",
               regs[1] == [] and any("SERVO9_FUNCTION = 28" in p for p in regs[2]) and
               any("no MAVLink" in p for p in regs[3]) and a1 and a1["ok"] and a1["what"] == "time+pos" and got and
               a5 and not a5["ok"] and st and st["denied"].get("in SERVO_OUTPUT_RAW") and
               st["denied"].get("in MOUNT_CONTROL") and st2 is not None,
               "slot 1 %s, slot 2 %s, slot 3 %s; assist slot 1 %s (drone got %s), armed slot 5 %s; denied %s; "
               "wrong sysid %s" % (regs[1], regs[2][:1], regs[3], a1 and a1.get("what"), bool(got),
                                   a5 and a5.get("why"), st and st["denied"], st2 and st2["stats"]["wrong_sysid"]))
    finally:
        w.close()
        fl.stop()


# ------------------------------------------------------------------------------------------------------ UA11 -------

def ua11_gateway():
    import test_gateway as tg
    from gwclient import GatewayLink
    if not os.path.exists(tg.EXE):
        return report("UA11 the real gateway + a station", False, "autopilot_sim not built (%s)" % tg.EXE)
    tmp = tempfile.mkdtemp(prefix="uavtest_gw_")
    sim = gw = None
    links = []
    try:
        base = free_base()
        sim = tg.Sim(5.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = tg.gw_cfg_b(sim, root, uav={"enabled": True, "sim": True, "sim_base_port": base,
                                          "sim_args": ["--fault", "2:extra_output"], "assist_s": 5.0})
        gw = tg.GwRunner(cfg)
        st_port = cfg["stations"]["port"]
        a = GatewayLink("127.0.0.1:%d" % st_port, "alice", password=tg.PW["alice"], station="alice")
        a.start()
        v = GatewayLink("127.0.0.1:%d" % st_port, "vic", password=tg.PW["vic"], station="vic")
        v.start()
        links += [a, v]
        seen = {"a": [], "v": []}

        def pump():
            for k, l in (("a", a), ("v", v)):
                while True:
                    try:
                        seen[k].append(l.events.get_nowait())
                    except Exception:
                        break

        def last(k, kind, cond=lambda d: True):
            pump()
            for kd, d in reversed(seen[k]):
                if kd == kind and cond(d):
                    return d
            return None
        tel0 = until(lambda: last("a", "telemetry"), 10)
        up = until(lambda: (last("a", "gw_status", lambda d: (d.get("uav") or {}).get("state") == "ACTIVE" and
                                 all(s["present"] for s in d["uav"]["slots"]))), 20)
        alarms0 = tel0 and tel0.get("alarm_bits")
        if not up:
            sp = gw.gw.uav.sim_proc
            lg = os.path.join(root, "logs", "uav_simdrone.log")
            print("UA11 debug: simulated drones %s exit %s, log %s" % (sp, sp and sp.poll(), open(lg, encoding="utf-8",
                  errors="replace").read()[-800:] if os.path.exists(lg) else "-"), flush=True)
        # the event loop stays fast while 6 registrations run
        for s in range(1, 7):
            a.send_msg({"t": "uav_register", "slot": s})
        lat = []
        for _ in range(20):
            t0 = time.time()
            gw.call(lambda: None)
            lat.append(time.time() - t0)
            time.sleep(0.05)
        acks = until(lambda: (lambda l: l if len(l) >= 6 else None)(
            [d for kd, d in (pump() or seen["a"]) if kd == "uav_ack"]), 40) or []
        by = {d["slot"]: d for d in acks}
        v.send_msg({"t": "uav_register", "slot": 1})
        vref = until(lambda: last("v", "uav_ack"), 5)
        stat = until(lambda: last("a", "gw_status", lambda d: (d.get("uav") or {}).get("slots") and
                                  d["uav"]["slots"][0].get("registered") and
                                  (d["uav"]["slots"][0].get("assist") or {}).get("ok")), 15)
        reg_file = os.path.join(root, "state", "uav_registry.json")
        on_disk = until(lambda: os.path.exists(reg_file) and len(json.load(open(reg_file, encoding="utf-8"))["slots"])
                        == 6, 5)
        drone_assist = None
        u = stat["uav"] if stat else {}
        s1 = (u.get("slots") or [{}])[0]
        drone_assist = (s1.get("assist") or {}).get("what")
        ok_reg = (by.get(1, {}).get("ok") and by.get(2) and not by[2]["ok"] and by[2]["code"] == 122 and
                  any("SERVO9_FUNCTION" in p for p in by[2]["problems"]) and all(by.get(s, {}).get("ok")
                                                                                 for s in (1, 3, 4, 5, 6)))
        report("UA11a the real gateway: uav_register from a station -> uav_ack (5 good drones registered, the "
               "gripper output refused 122 with its problems), a VIEWER refused, gw_status \"uav\" = 6 slots + next "
               "= slot 1 + launchable, the registry on the SSD, the warm start sent after the registration, the "
               "event loop never blocked (max %.0f ms)" % (max(lat) * 1000),
               up and ok_reg and vref and not vref["ok"] and "VIEWER" in vref["text"] and u.get("next") == 1 and
               s1.get("launchable") and not u["slots"][1]["launchable"] and on_disk and
               drone_assist in ("time+pos", "time", "pos") and max(lat) < 0.25,
               "acks %s; viewer %s; next %s; slot 1 launchable %s, slot 2 %s (%s); file %s; warm start %s" % (
                   {s: (d["ok"], d["code"]) for s, d in sorted(by.items())}, vref and vref["text"], u.get("next"),
                   s1.get("launchable"), u.get("slots", [{}, {}])[1].get("launchable"),
                   u.get("slots", [{}, {}])[1].get("why"), bool(on_disk), drone_assist))
        # the router killed: a notice, restarted, the AURIX untouched
        p = gw.gw.uav.proc
        p.kill()
        off = until(lambda: last("a", "gw_status", lambda d: (d.get("uav") or {}).get("state") == "OFFLINE"), 10)
        back = until(lambda: last("a", "gw_status", lambda d: (d.get("uav") or {}).get("state") == "ACTIVE" and
                                  d["uav"]["router"]["starts"] >= 2), 20)
        tel1 = last("a", "telemetry")
        audit = [h[1] for h in gw.gw.uav.history]
        report("UA11b the router process killed = a NOTICE only (gw_status uav OFFLINE, audit uav_state), restarted; "
               "the AURIX untouched (the same mode, no new alarm bit)",
               off and back and tel0 and tel1 and tel1["mode"] == tel0["mode"] and
               (tel1.get("alarm_bits", 0) & ~(alarms0 or 0)) == 0 and "OFFLINE" in audit,
               "offline %s (%s), back %s, mode %s -> %s, alarms %s -> %s, history %s" % (
                   bool(off), off and off["uav"]["why"], bool(back), tel0 and tel0["mode_name"], tel1 and
                   tel1["mode_name"], alarms0, tel1 and tel1.get("alarm_bits"), audit))
        procs = [gw.gw.uav.proc, gw.gw.uav.sim_proc]
        gw.stop()
        gw = None
        ended = until(lambda: all(q is None or q.poll() is not None for q in procs), 10)
        report("UA11c no process left after the gateway stops (the router and the simulated drones end)", ended,
               "exit codes %s" % [q and q.poll() for q in procs])
    finally:
        for l in links:
            try:
                l.stop()
            except Exception:
                pass
        if gw is not None:
            gw.stop()
        if sim is not None:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------------------------------------------ UA12 -------

def ua12_osdep():
    import osdep

    class P:
        def __init__(self, dev, loc, vid=0x1209, pid=0x5741, sn=None):
            self.device, self.location, self.vid, self.pid, self.serial_number = dev, loc, vid, pid, sn
            self.description = "ArduPilot"
    ports = [P("COM7", "1-2.1:x.0", sn="AAA"), P("COM8", "1-2.2:x.0", sn="BBB"), P("COM9", "1-2.3:x.0", sn="CCC")]
    r1 = osdep.resolve_serial("usbloc:1-2.2", ports=ports)
    try:
        osdep.resolve_serial("usbloc:1-2.6", ports=ports)
        missing = False
    except osdep.SerialSpecError as e:
        missing = "nothing plugged" in str(e)
    swapped = [P("COM7", "1-2.1:x.0", sn="CCC"), P("COM8", "1-2.2:x.0", sn="AAA")]   # drones swapped between slots
    r2 = osdep.resolve_serial("usbloc:1-2.2", ports=swapped)
    lines = osdep.list_serial(ports=ports, links={}, platform="win32")
    report("UA12 osdep: usbloc:<USB port path> = the slot (the hub port, never the drone's own id: two drones "
           "swapped between slots stay the slots' devices), missing = refused, --list-serial shows the usbloc names",
           r1 == "COM8" and missing and r2 == "COM8" and any("usbloc:1-2.3" in ln for ln in lines),
           "slot 2 -> %s, empty port refused %s, after a swap %s, list %s" % (
               r1, missing, r2, [ln for ln in lines if "usbloc" in ln][:1]))


def watchdog():
    def fire():
        print("\nSTALLED: test_uav.py still running after %d s - killed" % LIMIT_S, flush=True)
        os._exit(3)
    t = threading.Timer(LIMIT_S, fire)
    t.daemon = True
    t.start()


def main():
    watchdog()
    tests = [ua1_codec, ua2_pymavlink, ua3_allow, ua4_writer, ua5_no_aurix, ua6_check, ua7_registry, ua8_assist,
             ua9_simdrone, ua10_router, ua11_gateway, ua12_osdep]
    want = [a.upper() for a in sys.argv[1:]]
    for t in tests:
        if want and t.__name__.split("_")[0].upper() not in want:
            continue
        try:
            t()
        except Exception:
            import traceback
            traceback.print_exc()
            report(t.__name__, False, "crashed")
    bad = sum(1 for r in results if r is False)
    na = sum(1 for r in results if r is None)
    print("UAV TESTS: %d passed, %d failed%s" % (len(results) - bad - na, bad, ", %d N/A" % na if na else ""),
          flush=True)
    sys.stdout.flush()
    os._exit(1 if bad else 0)                   # (W50: never wait for a gateway loop's teardown)


if __name__ == "__main__":
    main()
