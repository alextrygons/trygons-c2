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
  --- step U3 (route -> mission, the launch sequence, the hatch + its switches) ---
  UA13 route -> mission (HOME slot, TAKEOFF to the slot altitude, speed cap, waypoints with their altitude,
       LOITER_UNLIM at the end), all allow-listed; the read-back comparison
  UA14 the UAV route checks: altitude window, radio range (launch point + the boat now), energy + landing reserve,
       no GPS / unknown battery (123); no-fly zones with altitude bands, the air geofence, a broken file (124)
  UA15 the hatch state + the two limit switches with the simulated hatch: normal travel, every FAULT case
  UA16 the launch state machine (scripted magazine): order, gaps, upload before the hatch, ARM only OPEN, every stop
       case, no climb = DISARM + hatch open, the GPS fallback climb (STANDBY only), a STOP with a drone on deck
  UA17 the router process flying a launch: mission upload + read-back, ARM only with a fresh hatch OPEN held IN the
       router (U2 review LOW-3), DISARM refused in the air (landed from the drone, LOW-4), ELRS link, MISSION START
  UA18 the real gateway + the simulated AURIX relay board + hatch + drones: 125 for a station's hatch relay, routes,
       the copilot's PREPARE -> the captain's YES -> 2 drones launched, the hatch relay through the AURIX, the boat
       untouched; with no HELM nothing switches the hatch (notice) and a launch is refused
  UA19 W55: the gateway's own HELM grant vs TELEM2 for the hatch relay command; 125 for a station's hatch relay

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
        if name in allow.TARGETED and "target_system" not in fields:
            fields = dict(fields, target_system=11)
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
            ok, _w = allow.check_out(nm, {"command": cmd, "param1": 1.0, "param2": 4.0, "param7": 20.0, "z": 20.0,
                                          "target_system": 11},
                                     {"hatch_open": True, "landed": True, "battery_land": True})
            if ok:
                allowed_cmds.append(cmd)
    allowed_cmds = sorted(set(allowed_cmds))
    if not set(allowed_cmds) <= allow.COMMANDS:
        bad.append("commands %s" % allowed_cmds)
    # every mission item command
    m_ok = sorted({c for c in range(65536) for fr in (2, 3, 6)
                   if allow.check_out("MISSION_ITEM_INT", {"command": c, "frame": fr, "param2": 5.0,
                                                           "target_system": 11})[0]})
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
    # U2 review LOW-5: no broadcast (target_system 0), only mission_type 0, ceilings
    want(False, "COMMAND_LONG", {"command": 176, "param1": 1.0, "param2": 4.0, "target_system": 0}, {}, "broadcast")
    want(False, "MISSION_CLEAR_ALL", {"target_system": 11, "mission_type": 1}, {}, "clear the fence")
    want(False, "MISSION_CLEAR_ALL", {"target_system": 11, "mission_type": 255}, {}, "clear all")
    want(True, "MISSION_CLEAR_ALL", {"target_system": 11})
    want(False, "MISSION_COUNT", {"target_system": 11, "count": 3, "mission_type": 2}, {}, "rally upload")
    want(False, "COMMAND_LONG", {"command": 22, "param7": 121.0}, {}, "takeoff too high")
    want(False, "COMMAND_LONG", {"command": 22, "param7": 0.0}, {}, "takeoff to 0 m")
    want(True, "COMMAND_LONG", {"command": 22, "param3": 1.0, "param7": 15.0})
    want(False, "MISSION_ITEM_INT", {"command": 16, "frame": 3, "z": 500.0}, {}, "wp too high")
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel, "vx": 40.0}, {}, "fast")
    want(False, "SET_POSITION_TARGET_LOCAL_NED", {"coordinate_frame": 8, "type_mask": vel, "vz": 5.0}, {}, "fast vz")
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
    # U3 (storm I80 reserved): the ONE boat path = uavlink.hatch_relay -> stations.uav_relay (a T_RELAY of the HELM
    # station); uavlink itself never touches the AURIX link, and the UAV code calls nothing else of uavlink
    ul = open(os.path.join(HERE, "uavlink.py"), encoding="utf-8").read()
    if "aurix" in re.sub(r"#.*|\"\"\"[\s\S]*?\"\"\"", "", ul) or "stations.uav_relay(" not in ul:
        bad.append("uavlink.py touches the AURIX directly")
    used = set()
    for p in py_files([UAV_DIR]):
        used |= set(re.findall(r"uavlink\.(\w+)\(", open(p, encoding="utf-8").read()))
    if not used <= {"boat_state", "relay_on", "hatch_relay"}:
        bad.append("gateway/uav uses uavlink.%s" % sorted(used))
    report("UA5 no UAV path to the AURIX (no gateway/uav module imports aurix / proto / stations or touches "
           ".aurix; the only boat action = uavlink.hatch_relay -> stations.uav_relay) and no payload / release / "
           "target function anywhere in the UAV code", not bad and not names,
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
           "accuracy >= 50 m: sea-level height vs the ellipsoid, U2 review MED-4), checksums, GPS_RTCM_DATA fragments (<= 4 x 180 B); the boat position only from fresh "
           "telemetry with a trusted GPS and a sane uncertainty",
           ok_time and ok_tv and ok_pos and ok_ck and ok_fr and fixes[0] == (37.8, 23.6, 8.0) and
           fixes[1:] == [None, None, None, None] and what == "time+pos" and acc == 5000,
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
               a5 and not a5["ok"] and "armed" in (a5.get("why") or "") and st and st["denied"].get("in SERVO_OUTPUT_RAW") and
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


# ------------------------------------------------------------------------------------------------------ U3 ---------

def ua13_mission():
    from uav import mission
    wps = [{"lat": 37.821, "lon": 23.601, "alt": 50.0}, {"lat": 37.822, "lon": 23.603, "alt": 70.0}]
    it = mission.build(wps, (37.82, 23.60), mission.init_alt(3), 8.0)
    cmds = [i["command"] for i in it]
    allowed = all(allow.check_out("MISSION_ITEM_INT", dict(i, target_system=11))[0] for i in it)
    ok_same = mission.same(it, [dict(i) for i in it]) == (True, "")
    bad_alt = [dict(i) for i in it]
    bad_alt[3]["z"] = 51.0
    other = [dict(i) for i in it]
    other[4]["command"] = 21
    short = mission.same(it, it[:-1])
    home_diff = [dict(i) for i in it]
    home_diff[0]["x"] += 999                                     # the drone's own home in seq 0: not compared
    try:
        mission.build([], (0, 0), 30, 5)
        empty = False
    except ValueError:
        empty = True
    report("UA13 route -> mission: HOME slot, TAKEOFF to the slot-staggered altitude (30 + 5 m x (slot - 1)), the "
           "speed cap, one WAYPOINT per point with ITS altitude, LOITER_UNLIM over the last point (never a landing); "
           "every item passes the allow-list; the read-back comparison finds a changed altitude / command / a "
           "missing item and ignores the drone's own home",
           cmds == [16, 22, 178, 16, 16, 17] and it[1]["z"] == 40.0 and it[2]["param2"] == 8.0 and
           it[3]["z"] == 50.0 and it[4]["z"] == 70.0 and it[5]["x"] == 378220000 and allowed and ok_same and
           not mission.same(it, bad_alt)[0] and not mission.same(it, other)[0] and not short[0] and
           mission.same(it, home_diff)[0] and empty,
           "commands %s, take-off %.0f m, allowed %s, read-back ok %s, altitude %s, command %s, short %s" % (
               cmds, it[1]["z"], allowed, ok_same, mission.same(it, bad_alt), mission.same(it, other)[1], short[1]))


def ua14_checks():
    from uav import nofly, uavcheck
    boat = (37.82, 23.60)
    start = (37.82, 23.60, 0.0)
    cfg = dict(uavcheck.DEFAULTS)
    batt = {"pct": 95, "capacity_mah": 5000}

    def wp(dn, de, alt=50.0):
        return {"lat": boat[0] + dn / 111320.0, "lon": boat[1] + de / (111320.0 * math.cos(math.radians(boat[0]))),
                "alt": alt}
    ok = uavcheck.check([wp(300, 200), wp(500, -100)], start, boat, cfg, batt, take_alt=30)
    high = uavcheck.check([wp(300, 0, 150)], start, boat, cfg, batt)
    low = uavcheck.check([wp(300, 0, 5)], start, boat, cfg, batt)
    far = uavcheck.check([wp(1600, 0)], start, boat, cfg, batt)
    boat_moved = uavcheck.check([wp(1000, 0)], start, (boat[0] - 1000 / 111320.0, boat[1]), cfg, batt)
    weak = uavcheck.check([wp(1400, 0), wp(-1400, 0), wp(1400, 0)], start, boat, cfg, {"pct": 40,
                          "capacity_mah": 3000})
    nogps = uavcheck.check([wp(100, 0)], None, None, cfg, batt)
    unknown_batt = uavcheck.check([wp(100, 0)], start, boat, cfg, {"pct": None})
    route, reserve = uavcheck.energy_mah(start, [wp(0, 0, 100)], cfg)
    try:
        uavcheck.clean([{"lat": float("nan"), "lon": 1}], 50)
        nan = False
    except ValueError:
        nan = True
    cleaned = uavcheck.clean([{"lat": 37.8, "lon": 23.6}], 50)
    tmp = tempfile.mkdtemp(prefix="uavtest_air_")
    try:
        z = {"zones": [{"name": "HELIPAD", "kind": "circle", "center": [boat[0] + 400 / 111320.0, boat[1]],
                        "radius_m": 100, "floor_m": 0, "ceiling_m": 60},
                       {"name": "PORT", "kind": "polygon", "poly": [[37.79, 23.55], [37.79, 23.56], [37.80, 23.56]],
                        "floor_m": 0, "ceiling_m": 9999}]}
        with open(os.path.join(tmp, "uav_nofly.json"), "w", encoding="utf-8") as f:
            json.dump(z, f)
        air = nofly.Airspace(tmp)
        through = uavcheck.check([wp(800, 0, 50)], start, boat, cfg, batt, air)
        above = uavcheck.check([wp(800, 0, 80)], start, boat, cfg, batt, air, take_alt=80)
        climb_into = uavcheck.check([wp(800, 0, 80)], start, boat, cfg, batt, air, take_alt=30)
        with open(os.path.join(tmp, "uav_geofence.json"), "w", encoding="utf-8") as f:
            json.dump({"poly": [[boat[0] - 0.005, boat[1] - 0.005], [boat[0] - 0.005, boat[1] + 0.005],
                                [boat[0] + 0.005, boat[1] + 0.005], [boat[0] + 0.005, boat[1] - 0.005]]}, f)
        air._mt = (None, None)
        air.reload()
        out = uavcheck.check([wp(200, 900, 80)], start, boat, cfg, batt, air, take_alt=80)
        inside_ok = uavcheck.check([wp(-200, 200, 80)], start, boat, cfg, batt, air, take_alt=80)
        with open(os.path.join(tmp, "uav_nofly.json"), "w", encoding="utf-8") as f:
            f.write("{broken")
        air._mt = (None, None)
        air.reload()
        broken = uavcheck.check([wp(-200, 200, 80)], start, boat, cfg, batt, air, take_alt=80)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    report("UA14 UAV route checks (not shallow water): altitude window (10..120 m), radio range from the launch point "
           "AND the boat now, energy with the landing reserve + wind margin above the drone's backstop, no trusted "
           "boat GPS / unknown battery refused (123); air no-fly zones with their altitude band (a climb through the "
           "floor counts), the AIR geofence, a broken airspace file = refused (124); a NaN point refused",
           ok is None and high and high[0] == 123 and low and low[0] == 123 and far and "radio range" in far[1] and
           boat_moved and "the boat" in boat_moved[1] and weak and "battery" in weak[1] and nogps and
           unknown_batt and reserve > 0 and route > 0 and nan and cleaned[0]["alt"] == 50.0 and
           through and through[0] == 124 and "HELIPAD" in through[1] and above is None and climb_into and
           climb_into[0] == 124 and out and "air geofence" in out[1] and inside_ok is None and broken and
           broken[0] == 124 and "unreadable" in broken[1],
           "ok %s; high %s; far %s; boat moved %s; weak %s; zone %s; above it %s; climb into %s; fence %s; "
           "broken %s" % (ok, high, far and far[1][:40], boat_moved and boat_moved[1][:40], weak and weak[1][:50],
                          through, above, climb_into, out, broken))


def ua15_hatch():
    from uav import hatch, simhatch
    out = {}

    def run(fault=None, steps=(), travel=2.0, open_s=4.0):
        """steps: [(t, relay_on, command)]; returns the hatch states seen (deduplicated) + the last."""
        h = hatch.Hatch(open_s, open_s)
        sh = simhatch.SimHatch(travel)
        if fault:
            sh.fault(fault)
        seen, relay = [], False
        t = 0.0
        ev = dict((round(a, 2), (r, c)) for a, r, c in steps)
        while t <= 12.0:
            if round(t, 2) in ev:
                relay, cmd = ev[round(t, 2)]
                if cmd is not None:
                    h.command(cmd, t)
            sh.step(t, relay)
            st = h.update(t, sh.read(t))
            if not seen or seen[-1] != st:
                seen.append(st)
            t += 0.05
        return seen, h
    norm, h1 = run(steps=[(1.0, True, True), (6.0, False, False)])
    out["normal"] = norm
    stuck, h2 = run("stuck", steps=[(1.0, True, True)])
    both, h3 = run("both")
    nosw, h4 = run("closed_sw_broken")
    unpl, h5 = run("unplugged")
    broken_open, h6 = run("open_sw_broken", steps=[(1.0, True, True)])
    selfclose, h7 = run(steps=[(1.0, True, True), (6.0, False, None)])       # relay OFF without a command (reboot)
    why7 = h7.why
    # a new command clears a latched FAULT
    h7.command(False, 12.1)
    cleared = h7.state != hatch.FAULT
    report("UA15 the hatch + its two limit switches (the simulated hatch follows the relay): CLOSED -> OPENING -> OPEN "
           "-> CLOSING -> CLOSED; FAULT for no switch change in time (stuck motor / a broken OPEN wire), BOTH switches, "
           "NONE for longer than the travel time, the cable unplugged, a move WITHOUT a command (the relay switched "
           "off by a board reboot closes the hatch); a new command clears the latched FAULT",
           norm == ["CLOSED", "OPENING", "OPEN", "CLOSING", "CLOSED"] and stuck[-1] == "FAULT" and
           "did not" in h2.why and both[-1] == "FAULT" and "both" in h3.why and nosw[-1] == "FAULT" and
           "no hatch switch" in h4.why and unpl[-1] == "FAULT" and "unplugged" in h5.why and
           broken_open[-1] == "FAULT" and "did not open" in h6.why and selfclose[-1] == "FAULT" and
           "without a command" in why7 and cleared,
           "normal %s; stuck %s (%s); both %s; no switch %s; unplugged %s; OPEN wire broken %s (%s); self-close %s "
           "(%s); cleared %s" % (norm, stuck, h2.why, both, nosw, unpl, broken_open, h6.why, selfclose, why7,
                                 cleared))


class FakeIo:
    """The manager seen by launch.Launch, with a scripted magazine (tests UA16)."""

    def __init__(self, n=3, gps=True, climb=True, standby=True):
        self.t = 0.0
        self.drones = {s: {"usb_present": True, "battery_pct": 95, "gps_fix": 1, "ekf_flags": 0x07, "rel_alt": 0.0,
                           "alt_age": 0.1, "landed": True, "armed": False} for s in range(1, n + 1)}
        self.gps, self.climb = gps, climb
        self.boat_d = {"launch_why": [], "standby": standby, "pos": (37.82, 23.60)}
        self.hatch_st, self.hatch_cmd_open = "CLOSED", False
        self.sent = []                     # (t, slot, what)
        self.relay = []
        self.notices = []
        self.upload_ok = True
        self.captain = True
        self.sky = False
        self.q = []

    def now(self):
        return self.t

    def boat(self):
        return self.boat_d

    def hatch_state(self):
        return self.hatch_st

    def hatch_why(self):
        return "test"

    def hatch_open_s(self):
        return 5.0

    def hatch_commanded_open(self):
        return self.hatch_cmd_open

    def captain_ok(self, sid):
        return self.captain

    def slot(self, n):
        return dict(self.drones[n])

    def next_slot(self):
        return next((s for s, d in sorted(self.drones.items()) if d["usb_present"]), None)

    def check_drone(self, n):
        return None if self.drones[n]["usb_present"] else "not on its slot USB cable"

    def mission_items(self, n, pos, take):
        return [{"seq": 0}], ""

    def upload(self, n, items, via, cb):
        self.sent.append((self.t, n, "upload " + via))
        self.q.append(lambda: cb(self.upload_ok, "" if self.upload_ok else "read-back differs"))

    def mission_loaded(self, n):
        pass

    def cmd(self, n, via, command, params, cb):
        params = list(params) + [0] * 7
        name = {176: "mode %d" % params[1], 400: "arm" if params[0] == 1 else "disarm", 22: "takeoff %g/%g" % (
            params[2], params[6]), 300: "start"}.get(command, str(command))
        self.sent.append((self.t, n, name))
        d = self.drones[n]
        res = 0
        if command == 400 and params[0] == 1:
            res = 0 if self.hatch_st == "OPEN" else 4               # the router's allow-list: ARM only OPEN
            if res == 0:
                d["armed"] = True
        elif command == 400:
            d["armed"] = False
        self.q.append(lambda: cb(res, ""))

    def armed(self, n):
        pass

    def on_mission(self, n):
        pass

    def catchup(self, n):
        self.sent.append((self.t, n, "catchup"))

    def hatch_cmd(self, on, cb):
        self.relay.append((self.t, on))
        self.hatch_cmd_open = on
        self.hatch_t = self.t
        self.q.append(lambda: cb(True, ""))

    def notice(self, text, level="info"):
        self.notices.append(text)

    def world(self, dt):
        """The simulated physics: the hatch moves 2 s, the GPS fix 1 s after the sky, an armed drone climbs."""
        if self.hatch_cmd_open and self.hatch_st in ("CLOSED", "CLOSING") and self.t - self.hatch_t >= 2.0:
            self.hatch_st = "OPEN"
        if not self.hatch_cmd_open and self.hatch_st == "OPEN" and self.t - self.hatch_t >= 2.0:
            self.hatch_st = "CLOSED"
        for s, d in self.drones.items():
            if self.hatch_st == "OPEN" and self.gps and d["usb_present"]:
                d["gps_fix"], d["ekf_flags"] = 3, 0x1F
            if d["armed"] and self.climb:
                d["rel_alt"] += 2.5 * dt
                d["landed"] = d["rel_alt"] < 0.3
                if d["rel_alt"] > 0.3:
                    d["usb_present"] = False


def run_seq(io, seq, until_t, events=()):
    ev = sorted(events)
    while io.t < until_t:
        while ev and ev[0][0] <= io.t:
            ev.pop(0)[1]()
        seq.tick(io.t)
        while io.q:
            io.q.pop(0)()
        io.world(0.25)
        io.t += 0.25
        if seq.state in ("DONE", "STOPPED") and not ev:
            break


def ua16_launch():
    from uav import launch
    # 1) a normal 3-drone sequence
    io = FakeIo(3)
    seq = launch.Launch(io, {"gap_s": 15.0})
    seq.prepare([1, 2, 3], 5, 0.0)
    seq.approved(True, 7, 0.0)
    run_seq(io, seq, 200.0)
    order = [n for _t, n, w in io.sent if w == "arm"]
    arm_t = [t for t, _n, w in io.sent if w == "arm"]
    gaps = [b - a for a, b in zip(arm_t, arm_t[1:])]
    up_first = all(any(w.startswith("upload") and n == k and t < arm_t[i] for t, n, w in io.sent)
                   for i, k in enumerate(order))
    opened_before_arm = io.relay and io.relay[0][1] is True and io.relay[0][0] < arm_t[0]
    closed_after = io.relay[-1][1] is False and io.relay[-1][0] > arm_t[-1]
    seq_ok = (seq.state == "DONE" and order == [1, 2, 3] and min(gaps) >= 15.0 and up_first and opened_before_arm and
              closed_after and len(io.relay) == 2 and [w for _t, n, w in io.sent if n == 1][:4] ==
              ["upload usb", "mode 4", "arm", "takeoff 0/30"])
    # 2) the boat leaves STANDBY after the first drone: stop, no further drone, the hatch not switched
    io2 = FakeIo(3)
    seq2 = launch.Launch(io2, {"gap_s": 15.0})
    seq2.prepare([1, 2, 3], 5, 0.0)
    seq2.approved(True, 7, 0.0)

    def leave():
        io2.boat_d["launch_why"] = ["the boat is not in STANDBY or DP SPOT / JOG (MANUAL)"]
    run_seq(io2, seq2, 200.0, [(10.0, leave)])
    stop_ok = (seq2.state == "STOPPED" and "STANDBY" in seq2.why and
               [n for _t, n, w in io2.sent if w == "arm"] == [1] and len(io2.relay) == 1)
    # 3) a hatch FAULT while opening: never ARM
    io3 = FakeIo(2)
    seq3 = launch.Launch(io3)
    seq3.prepare([1, 2], 5, 0.0)
    seq3.approved(True, 7, 0.0)

    def hfault():
        io3.hatch_st = "FAULT"
        io3.hatch_cmd_open = False                 # (the fake world stops moving it)
    run_seq(io3, seq3, 60.0, [(1.0, hfault)])
    hf_ok = seq3.state == "STOPPED" and "HATCH FAULT" in seq3.why and not any(w == "arm" for _t, _n, w in io3.sent)
    # 4) armed but no climb (motor / prop fault): DISARM at once, stop, the hatch stays OPEN
    io4 = FakeIo(2, climb=False)
    seq4 = launch.Launch(io4)
    seq4.prepare([1, 2], 5, 0.0)
    seq4.approved(True, 7, 0.0)
    run_seq(io4, seq4, 60.0)
    nc = [w for _t, n, w in io4.sent if n == 1]
    nc_ok = (seq4.state == "STOPPED" and "did not leave the hatch" in seq4.why and "disarm" in nc and seq4.keep_open
             and io4.relay == [(io4.relay[0][0], True)] and not any(n == 2 and w == "arm" for _t, n, w in io4.sent))
    # 5) no GPS: STANDBY = the climb fallback (ALT_HOLD + TAKEOFF param3 1, 15 m) -> catch-up; DP = stop, never ARM
    io5 = FakeIo(1, gps=False)
    seq5 = launch.Launch(io5)
    seq5.prepare([1], 5, 0.0)
    seq5.approved(True, 7, 0.0)
    run_seq(io5, seq5, 80.0)
    fb = [w for _t, n, w in io5.sent]
    fb_ok = seq5.state == "DONE" and "mode 2" in fb and "takeoff 1/15" in fb and "catchup" in fb and "start" not in fb
    io6 = FakeIo(1, gps=False, standby=False)
    seq6 = launch.Launch(io6)
    seq6.prepare([1], 5, 0.0)
    seq6.approved(True, 7, 0.0)
    run_seq(io6, seq6, 60.0)
    dp_ok = seq6.state == "STOPPED" and "only in STANDBY" in seq6.why and not any(w == "arm" for _t, _n, w in io6.sent)
    # 6) the read-back differs: stop, never armed; 7) the captain's station lost; 8) slot mismatch; 9) NO / no answer
    io7 = FakeIo(1)
    io7.upload_ok = False
    seq7 = launch.Launch(io7)
    seq7.prepare([1], 5, 0.0)
    seq7.approved(True, 7, 0.0)
    run_seq(io7, seq7, 30.0)
    rb_ok = seq7.state == "STOPPED" and "read-back" in seq7.why and not io7.relay
    io8 = FakeIo(2)
    seq8 = launch.Launch(io8)
    seq8.prepare([1, 2], 5, 0.0)
    seq8.approved(True, 7, 0.0)

    def gone():
        io8.captain = False
    run_seq(io8, seq8, 60.0, [(1.0, gone)])
    cap_ok = seq8.state == "STOPPED" and "captain" in seq8.why and not any(w == "arm" for _t, _n, w in io8.sent)
    io9 = FakeIo(3)
    io9.drones[1]["usb_present"] = False                         # slot 1 empty: the top is slot 2
    seq9 = launch.Launch(io9)
    seq9.prepare([1, 2], 5, 0.0)
    seq9.approved(True, 7, 0.0)
    run_seq(io9, seq9, 10.0)
    mm_ok = seq9.state == "STOPPED" and "mismatch" in seq9.why
    io10 = FakeIo(1)
    seq10 = launch.Launch(io10)
    seq10.prepare([1], 5, 0.0)
    seq10.approved(False, None, 0.0, "the captain said NO")
    seq11 = launch.Launch(io10)
    seq11.prepare([1], 5, 0.0)
    run_seq(io10, seq11, 40.0)
    no_ok = seq10.state == "STOPPED" and seq11.state == "STOPPED" and "no answer" in seq11.why and not io10.sent
    # 10) a STOP while armed on deck (between ARM and the climb) = DISARM
    io12 = FakeIo(1, climb=False)
    seq12 = launch.Launch(io12)
    seq12.prepare([1], 5, 0.0)
    seq12.approved(True, 7, 0.0)
    run_seq(io12, seq12, 30.0, [(4.5, lambda: seq12.stop("STOP by test", io12.t))])
    deck = [w for _t, n, w in io12.sent]
    deck_ok = seq12.state == "STOPPED" and "arm" in deck and deck[-1] == "disarm" and "STOP by test" in seq12.why
    report("UA16 the launch sequence (state machine, scripted magazine): top slot first, strictly in series, the "
           "mission uploaded + read back BEFORE the hatch, the hatch opened before the first ARM and closed after the "
           "last, GUIDED -> ARM -> TAKEOFF to the slot altitude, >= 15 s between ARMs; ANY warning stops it (the boat "
           "leaving STANDBY, a hatch FAULT, the captain's station lost, a slot mismatch, a bad read-back, NO / no "
           "answer) with no further drone and the hatch not switched; armed without a climb = DISARM + the hatch "
           "stays OPEN; no GPS in STANDBY = the ALT_HOLD climb fallback (TAKEOFF param3 1, 15 m) + catch-up, in DP = "
           "stop; a STOP with a drone armed on deck disarms it",
           seq_ok and stop_ok and hf_ok and nc_ok and fb_ok and dp_ok and rb_ok and cap_ok and mm_ok and no_ok and
           deck_ok,
           "normal %s (order %s, gaps %s, relay %s); boat leaves %s (%s); hatch fault %s; no climb %s (%s); fallback "
           "%s (%s); DP %s; read-back %s; captain lost %s; mismatch %s; NO / timeout %s; deck stop %s (%s)" % (
               seq_ok, order, [round(g) for g in gaps], io.relay, stop_ok, seq2.why, hf_ok, nc_ok, nc, fb_ok, fb,
               dp_ok, rb_ok, cap_ok, mm_ok, no_ok, deck_ok, deck))


def ua17_router_flight():
    from uav import mission
    base = free_base()
    fl = simdrone.Fleet(base, 2, ttf=(2.0, 1.0, 0.5), origin=(37.82, 23.60)).start()
    w = Worker()
    try:
        w.send({"op": "config", "slots": [{"slot": i, "usb": "udp:127.0.0.1:%d" % (base + i),
                                           "elrs": "udp:127.0.0.1:%d" % (base + 10 + i), "sysid": 10 + i}
                                          for i in (1, 2)]})
        w.find(lambda e: e.get("ev") == "status" and e["links"].get("1", {}).get("present") and
               e["radio"].get("1", {}).get("present"), 8)
        seq = [0]

        def req(op, **kw):
            seq[0] += 1
            w.send(dict(kw, op=op, id=seq[0]))
            return w.find(lambda e, i=seq[0]: e.get("id") == i and e.get("ev") in ("mission", "cmd"), 15)
        wps = [{"lat": 37.8205, "lon": 23.6005, "alt": 40.0}]
        items = mission.build(wps, (37.82, 23.60), 30.0, 8.0)
        up = req("mission", slot=1, via="usb", items=items)
        on_drone = [int(i["command"]) for i in fl.drones[0].mission]
        bad = [dict(i) for i in items]
        bad[3]["command"] = 183                                       # DO_SET_SERVO: never leaves the router
        up_bad = req("mission", slot=2, via="usb", items=bad)
        for d in fl.drones:
            d.sky = True
        until(lambda: fl.drones[0].fix >= 3, 6)
        mode = req("cmd", slot=1, via="usb", command=176, params=[1, 4])
        arm_closed = req("cmd", slot=1, via="usb", command=400, params=[1, 0])        # no hatch op: refused here
        armed_before = fl.drones[0].armed
        stale = []
        w.send({"op": "hatch", "open": True})
        arm = req("cmd", slot=1, via="usb", command=400, params=[1, 0])
        tko = req("cmd", slot=1, via="usb", command=22, params=[0, 0, 0, 0, 0, 0, 30])
        up_alt = until(lambda: fl.drones[0].alt > 12.0, 12)
        # in the air: the USB cable is gone, the ELRS link carries on; DISARM refused (the drone says "in air")
        st = w.find(lambda e: e.get("ev") == "status" and not e["links"].get("1", {}).get("present") and
                    (e["drones"].get("1") or {}).get("rel_alt", 0) > 10, 10)
        disarm_air = req("cmd", slot=1, via="auto", command=400, params=[0, 0])
        start = req("cmd", slot=1, via="elrs", command=300, params=[1, 0])
        cur = fl.drones[0].cur
        # the hatch op goes stale (the manager gone quiet) = ARM refused again
        time.sleep(1.3)
        arm2 = req("cmd", slot=2, via="usb", command=400, params=[1, 0])
        stale.append(fl.drones[1].armed)
        report("UA17 the router flies a launch (2 simulated drones, USB + ELRS links): mission upload + read-back over "
               "USB (a DO_SET_SERVO item never leaves the router), ARM refused while the router holds no fresh hatch "
               "OPEN, ARM / TAKEOFF with it, the USB link gone in the air and the ELRS link carrying the altitude, "
               "DISARM refused in the air (landed = the drone's own EXTENDED_SYS_STATE / altitude - U2 review LOW-4), "
               "MISSION START from seq 1 over ELRS, a stale hatch state refuses ARM again",
               up and up["ok"] and on_drone == [16, 22, 178, 16, 17] and up_bad and not up_bad["ok"] and
               not fl.drones[1].mission and mode and mode["result"] == 0 and arm_closed and
               arm_closed["result"] is None and not armed_before and arm and arm["result"] == 0 and tko and
               tko["result"] == 0 and up_alt and st is not None and disarm_air and disarm_air["result"] is None and
               start and start["result"] == 0 and cur >= 1 and arm2 and arm2["result"] is None and not stale[0],
               "upload %s (drone %s), bad item %s; mode %s, arm closed %s, arm %s, takeoff %s, climbed %s, ELRS "
               "status %s, disarm in air %s, start %s (seq %s), stale-hatch arm %s" % (
                   up and (up["ok"], up["why"]), on_drone, up_bad and up_bad["why"], mode and mode["result"],
                   arm_closed and arm_closed["why"], arm and arm["result"], tko and tko["result"], bool(up_alt),
                   st is not None, disarm_air and disarm_air["why"], start and start["result"], cur,
                   arm2 and arm2["why"]))
    finally:
        w.close()
        fl.stop()


HATCH_N = 8
UA_RELAYS = [{"n": 6, "name_el": "ΦΩΤΑ", "name_en": "NAV LIGHTS", "type": "onoff", "group": "extra", "pair": None,
              "max_s": 0, "show_hud": False, "power_up": "keep", "role": None},
             {"n": HATCH_N, "name_el": "ΚΑΤΑΠΑΚΤΗ UAV", "name_en": "UAV HATCH", "type": "onoff", "group": "extra",
              "pair": None, "max_s": 0, "show_hud": False, "power_up": "off", "role": None}]


def ua18_gateway_launch():
    """The real gateway + the simulated AURIX (its relay board) + the simulated hatch + the simulated drones."""
    import test_gateway as tg
    import proto
    if not os.path.exists(tg.EXE):
        return report("UA18 the launch through the real gateway", False, "autopilot_sim not built")
    tmp = tempfile.mkdtemp(prefix="uavtest_launch_")
    sim = gw = None
    clients, beat = [], None
    try:
        base = free_base()
        sim = tg.Sim(1.0, "origin=37.82,23.60")
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = tg.gw_cfg_b(sim, root, stations={"driver_timeout_s": 8.0},
                          uav={"enabled": True, "sim": True, "sim_base_port": base,
                               "sim_args": ["--ttf", "2,1,0.5", "--origin", "37.82,23.60"], "assist_s": 30.0,
                               "hatch_relay": HATCH_N, "hatch": {"sim_travel_s": 2.0, "open_s": 8.0, "close_s": 8.0},
                               "launch": {"gap_s": 5.0}})
        gw = tg.GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, _w = sim.station()
        clients.append(link)
        alice = tg.Client(sp, "alice", tg.PW["alice"], name="captain")
        clients.append(alice)
        alice.pump(1.0)
        r_helm = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        beat = tg.DriverBeat(alice)
        bob = tg.Client(sp, "bob", tg.PW["bob"], name="copilot")
        clients.append(bob)
        bob.pump(1.0)
        pay = proto.p_relay_cfg(UA_RELAYS)
        r_cfg = alice.result(alice.cmd(proto.T_RELAY_CFG, pay))
        time.sleep(0.5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.3)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        active = until(lambda: g.telem2.get("relay_table") and g.telem2.get("relay_hash") ==
                       proto.relay_table_hash(UA_RELAYS) and not g.telem2.get("relay_pending"), 10)
        hatch_ok = until(lambda: g.stations.hatch_relay_ok(HATCH_N)[0] and g.uav.hatch.state == "CLOSED", 10)
        tel0 = dict(g.telem or {})
        # a station's relay command for the hatch relay = 125 (copilot and captain), another relay goes as before
        r_h_cap = alice.result(alice.cmd(proto.T_RELAY, proto.p_relay(HATCH_N, proto.RELAY_CMD_ON)))
        r_h_cop = bob.result(bob.cmd(proto.T_RELAY, proto.p_relay(HATCH_N, proto.RELAY_CMD_ON)))
        r_light = bob.result(bob.cmd(proto.T_RELAY, proto.p_relay(6, proto.RELAY_CMD_ON), timeout=4.0))
        up = until(lambda: g.uav.state == "ACTIVE" and all(
            (g.uav.slot(s) or {}).get("usb_present") for s in (1, 2, 3)), 20)
        acks = []

        def uav_acks(c, what):
            out = []
            while True:
                try:
                    k, d = c.l.events.get_nowait()
                except Exception:
                    break
                if k == "uav_ack":
                    acks.append(d)
                elif k == "ack":
                    c.acks.append(d)
            return [a for a in acks if a.get("what") == what]
        for s in (1, 2, 3):
            bob.l.send_msg({"t": "uav_register", "slot": s})
        regs = until(lambda: (lambda l: l if len(l) >= 3 else None)(uav_acks(bob, "register")), 40) or []
        # routes: the next UAV + slot 2 (copilot); one too far (123)
        bob.l.send_msg({"t": "uav_route", "target": "next", "wps": [{"lat": 37.8215, "lon": 23.6010, "alt": 45}]})
        bob.l.send_msg({"t": "uav_route", "target": 2, "wps": [{"lat": 37.8190, "lon": 23.5990}]})
        bob.l.send_msg({"t": "uav_route", "target": 3, "wps": [{"lat": 37.90, "lon": 23.60}]})
        routes = until(lambda: (lambda l: l if len(l) >= 3 else None)(uav_acks(bob, "route")), 10) or []
        by_slot = {a.get("slot"): a for a in routes}
        # the copilot prepares two drones; the captain's YES on screen 1
        lat = []
        bob.l.send_msg({"t": "uav_launch_prepare", "count": 2})
        k, ask = tg.next_event(alice, ("driver_ask",), 8.0)
        if ask:
            alice.l.send_msg({"t": "driver_answer", "id": ask["id"], "yes": True})
        relay_seen = set()
        order, t_arm = [], {}

        def watch():
            if HATCH_N in (g.telem2.get("relay_on") or []):
                relay_seen.add("on")
            elif "on" in relay_seen:
                relay_seen.add("off_after")
            for s in (1, 2):
                if (g.uav.flying.get(s) or {}) and s not in order:
                    order.append(s)
                    t_arm[s] = time.time()
            t0 = time.time()
            gw.call(lambda: None)
            lat.append(time.time() - t0)
            return g.uav.seq.state in ("DONE", "STOPPED")
        end = until(watch, 120, step=0.2)
        seq = g.uav.seq.status(time.time())
        tel1 = dict(g.telem or {})
        drones = [fl for fl in (g.uav.slot(1), g.uav.slot(2))]
        report("UA18a the launch through the real gateway (simulated AURIX relay board, hatch, drones): a station's "
               "T_RELAY for the hatch relay = 125 (captain and copilot; another relay still goes), the copilot's "
               "routes (one too far = 123), PREPARE by the copilot -> the captain's YES on screen 1 -> slot 1 then "
               "slot 2 launched, the hatch relay ON through the AURIX (TELEM2 readback) before the first ARM and OFF "
               "after the last, the hatch CLOSED again, the boat untouched (STANDBY, no new alarm), the event loop "
               "never blocked (max %.0f ms)" % (1000 * max(lat or [0])),
               r_helm == 0 and r_cfg == 0 and active and hatch_ok and r_h_cap == 125 and r_h_cop == 125 and
               r_light == 0 and up and len(regs) >= 3 and all(a["ok"] for a in regs) and
               by_slot.get(1, {}).get("ok") and by_slot.get(2, {}).get("ok") and by_slot.get(3) and
               by_slot[3]["code"] == 123 and ask is not None and "UAV LAUNCH" in ask.get("text", "") and end and
               seq["state"] == "DONE" and seq["launched"] == [1, 2] and order == [1, 2] and
               {"on", "off_after"} <= relay_seen and g.uav.hatch.state == "CLOSED" and
               tel1.get("mode") == tel0.get("mode") == proto.MODE_STANDBY and
               (tel1.get("alarm_bits", 0) & ~tel0.get("alarm_bits", 0)) == 0 and max(lat or [0]) < 0.25,
               "HELM %s, table %s / active %s, hatch %s; hatch relay from the captain %s / copilot %s, lights %s; "
               "regs %s; routes %s; ask %s; sequence %s (%s, launched %s, order %s), relay %s, hatch %s, mode %s -> %s, "
               "alarms %s -> %s; drones %s" % (
                   r_helm, r_cfg, bool(active), g.uav.hatch.status(), r_h_cap, r_h_cop, r_light,
                   [(a["slot"], a["ok"]) for a in regs], {s: (a["ok"], a["code"], a["text"][:50])
                                                          for s, a in by_slot.items()},
                   ask and ask.get("text"), seq["state"], seq["why"], seq["launched"], order, relay_seen,
                   g.uav.hatch.state, tel0.get("mode_name"), tel1.get("mode_name"), tel0.get("alarm_bits"),
                   tel1.get("alarm_bits"), [(d.get("rel_alt"), d.get("mode")) for d in drones]))
        # UA18b no HELM = nothing switches the hatch: HELM released, HATCH CLOSE / a new launch refused
        r_rel = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(1)))
        until(lambda: not g.telem2.get("helm"), 5)
        bob.l.send_msg({"t": "uav_route", "target": "next", "wps": [{"lat": 37.8212, "lon": 23.6012}]})
        alice.l.send_msg({"t": "uav_hatch_close"})
        bob.l.send_msg({"t": "uav_launch_prepare", "count": 1})
        nores = until(lambda: (lambda c, p: (c, p) if c and p else None)(
            [a for a in uav_acks(alice, "hatch_close")], [a for a in uav_acks(bob, "launch") if not a["ok"]]), 12)
        hc, pr = nores or ([], [])
        notice = any("HATCH NOT CLOSED" in n["text"] for n in g.uav.notices)
        report("UA18b with NO station holding HELM nothing switches the hatch (the AURIX would refuse every T_RELAY): "
               "the captain's HATCH CLOSE ends in the notice 'HATCH NOT CLOSED - take HELM and close it' after one "
               "retry, a new launch is refused 125 'no station holds HELM'",
               r_rel == 0 and hc and not hc[-1]["ok"] and "HELM" in hc[-1]["text"] and notice and pr and
               pr[-1]["code"] == 125 and "HELM" in pr[-1]["text"],
               "release %s; close %s; notice %s; prepare %s" % (r_rel, hc and hc[-1]["text"], notice,
                                                               pr and pr[-1]["text"]))
    finally:
        if beat is not None:
            beat.stop()
        for c in clients:
            try:
                c.close() if hasattr(c, "close") else c.stop()
            except Exception:
                pass
        if gw is not None:
            gw.stop()
        if sim is not None:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


def ua19_helm_grant():
    """W55 (docs/uav_design.md 5.6): the hatch relay goes on the HELM station's behalf only while the gateway's OWN
    HELM grant and TELEM2 agree; the grant follows an accepted T_HELM at once and TELEM2 only once it stayed 2 s."""
    from common import proto
    import stations as stmod

    class A:
        connected = True

        def __init__(self):
            self.sent, self.h = [], {}

        def on(self, t, fn):
            self.h[t] = fn

        def send(self, t, p):
            self.sent.append((t, p))

    class F:
        def __init__(self, payload, t=None):
            self.payload, self.type = payload, t

    class G:
        def __init__(self):
            self.aurix, self.telem2, self.storage = A(), {}, None
            self.audits = []

        def audit(self, kind, **kw):
            self.audits.append(kind)

    class W:
        def __init__(self):
            self.out = []

        def write(self, b):
            self.out.append(b)

        def is_closing(self):
            return False
    g = G()
    srv = stmod.StationServer(g, {})
    a = stmod.Station(2, "alice", "HELM", False, "127.0.0.1", "captain", W())
    b = stmod.Station(3, "bob", "HELM", False, "127.0.0.1", "copilot", W())
    srv.stations = {2: a, 3: b}
    srv.gw.uav = type("U", (), {"hatch_relay": HATCH_N})()

    def t2(helm):
        g.telem2 = {"helm": helm}
        srv._on_t2(F(b""))
    t2(None)
    s0 = srv.helm_station()
    # alice takes HELM: the ACK = the grant at once; TELEM2 still says nobody -> no hatch command yet
    srv._forward(a, 1, proto.T_HELM, proto.p_helm(0), 0.0)
    gseq = srv.gw_seq
    srv._on_gw_ack(F(struct.pack("<BHBB", 2, gseq, proto.T_HELM, 0)))
    s1 = srv.helm_station()
    t2(2)
    s2 = srv.helm_station()
    res = []
    srv.uav_relay(HATCH_N, True, lambda r, t: res.append((r, t)))
    sent = [p for t, p in g.aurix.sent if t == proto.T_GW_CMD]
    relay_frame = sent and sent[-1][4] == proto.T_RELAY and sent[-1][5:7] == bytes([HATCH_N, proto.RELAY_CMD_ON]) \
        and sent[-1][0] == 2
    # a role swap: HELM handed to bob (action 2) - TELEM2 lags with alice: they disagree = no hatch command
    srv._forward(a, 2, proto.T_HELM, proto.p_helm_hand(3), 0.0)
    srv._on_gw_ack(F(struct.pack("<BHBB", 2, srv.gw_seq, proto.T_HELM, 0)))
    t2(2)
    s3 = srv.helm_station()
    n_before = len(g.aurix.sent)
    srv.uav_relay(HATCH_N, False, lambda r, t: res.append((r, t)))
    not_sent = len(g.aurix.sent) == n_before and res[-1][0] is None
    t2(3)
    s4 = srv.helm_station()
    # HELM lost on the AURIX (no T_HELM): TELEM2 nobody for 2 s = the grant follows
    t2(0)
    srv.helm_t2_since -= 2.5
    srv.helm_grant_t -= 2.5
    t2(0)
    s5 = srv.helm_station()
    # a station's T_RELAY for the hatch relay = 125
    srv.max_age = 99
    a.last_seq, a.tokens = None, 10.0
    srv.command(a, {"seq": 10, "ts": time.time(), "type": proto.T_RELAY, "sid": a.session,
                    "p": proto.p_relay(HATCH_N, 1).hex()})
    refused = any(b'"result":125' in x for x in a.writer.out)
    report("UA19 W55: the gateway's own HELM grant - a hatch relay command goes only when it agrees with TELEM2 "
           "(an accepted take = the grant at once, TELEM2 lagging = wait; a hand-over = wait until TELEM2 shows the new "
           "holder; HELM lost on the AURIX = nobody after 2 s); it goes as the HELM station's T_RELAY; a station's "
           "own T_RELAY for the hatch relay = refusal 125",
           s0[0] is None and s1[0] is None and s2[0] is a and relay_frame and s3[0] is None and not_sent and
           s4[0] is b and s5[0] is None and refused,
           "nobody %s; after the ACK %s; TELEM2 agrees %s; frame %s; hand-over lagging %s (sent %s); agrees %s; lost "
           "%s; 125 %s" % (s0[1], s1[1], s2[0] and s2[0].id, relay_frame, s3[1], not not_sent, s4[0] and s4[0].id,
                           s5[1], refused))


def ua20_registry_path():
    """U2 review MED-1 / MED-2 / MED-3 / MED-4 / LOW-6 / LOW-9 / LOW-10 / LOW-11 through the manager's real path."""
    from uav import manager as mgr

    class St:
        def __init__(self, role="HELM"):
            self.id, self.user, self.role, self.name, self.sent = 3, "bob", role, "bob", []
            self.crew, self.single_op, self.local = "copilot", False, False

        def send(self, m):
            self.sent.append(m)

    class G:
        def __init__(self, root):
            self.audits, self.telem2 = [], {}
            self.storage = type("S", (), {"usable": True, "folder": lambda _s, n: os.makedirs(
                os.path.join(root, n), exist_ok=True) or os.path.join(root, n)})()
            self.stations = type("X", (), {"stations": {}})()
            self.clock = type("C", (), {"gps_valid": True, "gps_rx": time.time(), "now": lambda _s: time.time()})()

        def audit(self, kind, **kw):
            self.audits.append(kind)

        def telem_fresh(self):
            return None
    tmp = tempfile.mkdtemp(prefix="uavtest_u2r_")
    try:
        extra = os.path.join(tmp, "extra.ubx")
        with open(extra, "wb") as f:
            f.write(gpsassist.ubx(0x13, 0x00, bytes(68)) + gpsassist.ubx(0x06, 0x04, bytes(4)) +   # CFG-RST!
                    gpsassist.ubx(0x13, 0x20, bytes(8)))
        g = G(tmp)
        m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6, "assist_extra": extra}, g)
        sent = []
        m.proc = type("P", (), {"poll": lambda _s: None})()
        m.to_worker = lambda o: sent.append(o) or True
        st = St()
        g.stations.stations[3] = st
        # a registration that succeeds, then a re-registration with no drone: the record is CLEARED
        m.pending[1] = (3, 1, time.time())
        m._registered({"ev": "register", "id": 1, "slot": 1, "reading": good_reading(sysid=11)})
        ok_first = m.reg.recs.get(1, {}).get("ok")
        m.pending[2] = (3, 1, time.time())
        m._registered({"ev": "register", "id": 2, "slot": 1, "reading": {"slot": 1, "why": "no MAVLink"}})
        cleared = 1 not in m.reg.recs
        # a late answer (timed out, not pending) never writes; slot 0 never indexes specs[-1]
        m._registered({"ev": "register", "id": 99, "slot": 2, "reading": good_reading(sysid=12)})
        m.pending[3] = (3, 0, time.time())
        m._registered({"ev": "register", "id": 3, "slot": 0, "reading": good_reading(sysid=13)})
        late_ok = 2 not in m.reg.recs and 0 not in m.reg.recs and "uav_register_late" in g.audits
        # launchable needs the drone ON its cable; a foreign sysid / another UID = unknown drone
        m.pending[4] = (3, 2, time.time())
        m._registered({"ev": "register", "id": 4, "slot": 2, "reading": good_reading(sysid=12)})
        uid2 = m.reg.recs[2].get("uid")
        nothing = m.reg.launchable(2, m.live({}))
        foreign = m.reg.launchable(2, m.live({"present": False, "foreign_sysid": 44}))
        other_uid = m.reg.launchable(2, m.live({"present": True, "sysid": 12, "uid": "beef"}))
        same = m.reg.launchable(2, m.live({"present": True, "sysid": 12, "uid": uid2}))
        # a failed registration gets no GPS warm start (LOW-11)
        m.pending[5] = (3, 3, time.time())
        m._registered({"ev": "register", "id": 5, "slot": 3, "reading": good_reading(sysid=13,
                                                                                    params={"GRIP_ENABLE": 1})})
        m.worker = {"links": {"2": {"present": True}, "3": {"present": True}}}
        m.worker_t = time.time()
        m.next_assist = {}
        sent.clear()
        m.send_assist = lambda s: sent.append({"op": "assist", "slot": s})
        m.u3_tick = lambda now: None
        m.tick(time.time())
        assisted = sorted(x["slot"] for x in sent if x.get("op") == "assist")
        # unregister (a VIEWER refused)
        v = St("VIEWER")
        m.unregister_msg(v, {"slot": 2})
        m.unregister_msg(st, {"slot": 2})
        unreg = 2 not in m.reg.recs and not v.sent[-1]["ok"] and st.sent[-1]["ok"]
        # MED-3: only UBX-MGA from the extra file; MED-4: the time only while the GPS time is fresh
        mga_only = [c for c, _i, _p in gpsassist.parse_ubx(m.extra)] == [0x13, 0x13] and m.extra_dropped == 1
        utc_fresh = m.assist_data()[0] is not None
        g.clock.gps_rx = time.time() - 30
        utc_stale = m.assist_data()[0] is None
        blob, _w = gpsassist.assist_bytes(time.time(), None, gpsassist.ubx(0x06, 0x04, bytes(4)))
        no_cfg = all(c == 0x13 for c, _i, _p in gpsassist.parse_ubx(blob))
        t_acc = struct.unpack_from("<H", gpsassist.parse_ubx(gpsassist.mga_ini_time_utc(time.time()))[0][2], 16)[0]
        # MED-2: wrongly shaped registry files never stop the gateway
        shapes = []
        for content in ("[1, 2]", '{"slots": []}', '{"slots": {"x": 1, "2": [3]}}', '"text"', "{broken"):
            p = os.path.join(tmp, "reg_%d.json" % len(shapes))
            with open(p, "w", encoding="utf-8") as f:
                f.write(content)
            try:
                r = registry.Registry(p)
                shapes.append((r.recs == {}, bool(r.error) or content.startswith('{"slots": {"x"')))
            except Exception as e:
                shapes.append((False, repr(e)))
        # LOW-9 a NaN parameter never raises; LOW-6 sysid 0 / 255, RELAY16, MNT2, BATT2 failsafes
        nan = registry.check(good_reading(params={"FS_GCS_ENABLE": float("nan")}))
        sid0 = registry.check(good_reading(sysid=0))
        sid255 = registry.check(good_reading(sysid=255))
        r16 = registry.check(good_reading(params={"RELAY16_FUNCTION": 1}))
        mnt2 = registry.check(good_reading(params={"MNT2_TYPE": 1}))
        b2 = registry.check(good_reading(params={"BATT2_FS_CRT_ACT": 2}))
        report("UA20 (U2 review) the registry through the manager's real path: a failed / empty registration CLEARS "
               "the old record, a late answer / slot 0 never writes, launchable needs the drone ON its cable, a "
               "foreign sysid or another UID = unknown drone, no warm start to a failed registration, uav_unregister; "
               "only UBX-MGA from assist_extra (a CFG-RST dropped), the time only while the GPS time is fresh "
               "(accuracy 2 s+), wrongly shaped registry files never raise, a NaN parameter never raises, sysid 0 / "
               "255, RELAY16 / MNT2 / BATT2 failsafes",
               ok_first and cleared and late_ok and not nothing[0] and not foreign[0] and "unknown drone" in foreign[1]
               and not other_uid[0] and "UID" in other_uid[1] and same == (True, "") and assisted == [2] and unreg and
               mga_only and utc_fresh and utc_stale and no_cfg and t_acc >= 2 and
               all(a and b for a, b in shapes) and any("not a number" in x for x in nan) and
               any("not allowed" in x for x in sid0) and any("not allowed" in x for x in sid255) and
               any("RELAY16_FUNCTION" in x for x in r16) and any("MNT2_TYPE" in x for x in mnt2) and
               any("BATT2_FS_CRT_ACT" in x for x in b2),
               "registered %s, cleared %s, late %s; nothing %s, foreign %s, other UID %s, same %s; assisted %s; "
               "unregister %s; MGA only %s (dropped %s), time fresh %s / stale %s, CFG kept out %s, t acc %s s; "
               "shapes %s; NaN %s; sysid 0 %s" % (
                   ok_first, cleared, late_ok, nothing, foreign, other_uid, same, assisted, unreg, mga_only,
                   m.extra_dropped, utc_fresh, utc_stale, no_cfg, t_acc, shapes, nan[:1], sid0[-1:]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# frames from the MAVLink spec's rules, computed ONCE with pymavlink (ardupilotmega dialect) - fixed bytes, so
# UA21 checks the codec against something it did not make itself even where pymavlink is not installed
FIXED_FRAMES = [
    ("HEARTBEAT", {"type": 2, "autopilot": 3, "base_mode": 81, "custom_mode": 0, "system_status": 4,
                   "mavlink_version": 3}, 5, 1, 1, "fd09000005010100000000000000020351040395b8"),
    ("COMMAND_LONG", {"target_system": 11, "target_component": 1, "command": 400, "param1": 1.0}, 7, 255, 190,
     "fd20000007ffbe4c00000000803f00000000000000000000000000000000000000000000000090010b017fc1"),
    ("MISSION_ITEM_INT", {"target_system": 11, "target_component": 1, "seq": 3, "frame": 3, "command": 16,
                          "autocontinue": 1, "x": 378210000, "y": 236000000, "z": 50.0}, 8, 255, 190,
     "fd25000008ffbe49000000000000000000000000000000000000d0068b160013110e00004842030010000b010300018a68"),
]


def ua21_fixed():
    bad = []
    for name, fields, seq, sysid, compid, hexs in FIXED_FRAMES:
        mine = mavlink.encode(name, fields, seq, sysid, compid).hex()
        if mine != hexs:
            bad.append("%s %s != %s" % (name, mine, hexs))
        got = mavlink.Parser().feed(bytes.fromhex(hexs))
        if len(got) != 1 or any(got[0].fields[k] != v for k, v in fields.items()):
            bad.append("%s decode" % name)
    # independent MGA fragmentation check: reassemble the GPS_RTCM_DATA fragments by their flags only
    rnd = random.Random(21)
    frag_bad = []
    for ln in (1, 100, 179, 180, 181, 359, 360, 361, 540, 700):
        data = bytes(rnd.randrange(256) for _ in range(ln))
        frs = gpsassist.rtcm_frames(data, 9)
        parts = {}
        for fr in frs:
            assert fr["flags"] >> 3 == 9
            if fr["flags"] & 1:
                parts[(fr["flags"] >> 1) & 3] = bytes(fr["data"][:fr["len"]])
            else:
                parts[0] = bytes(fr["data"][:fr["len"]])
        back = b"".join(parts[k] for k in sorted(parts))
        if back != data or any(fr["len"] > 180 for fr in frs) or len(frs) > 4 or (ln > 180) != bool(frs[0]["flags"] & 1):
            frag_bad.append(ln)
    # UA4 aliasing: the writer check must catch "from .mavlink import encode" / "enc = mavlink.encode"
    alias = re.compile(r"from\s+\.?(?:uav\.)?mavlink\s+import\s+[^\n]*\bencode\b|=\s*mavlink\.encode\b(?!\s*\()")
    found = []
    for p in py_files([HERE, os.path.join(REPO, "groundstation")]):
        rel = os.path.relpath(p, REPO).replace("\\", "/")
        if alias.search(open(p, encoding="utf-8", errors="replace").read()):
            found.append(rel)
    caught = bool(alias.search("from .mavlink import encode\n")) and bool(alias.search("enc = mavlink.encode\n"))
    report("UA21 (U2 review LOW-14) the codec against FIXED frames from the spec's rules (made once by pymavlink), the "
           "GPS_RTCM_DATA fragments reassembled by their flags alone (1..700 bytes), no aliased encode anywhere "
           "(from .mavlink import encode / enc = mavlink.encode)",
           not bad and not frag_bad and not found and caught,
           "frames wrong %s; fragment lengths wrong %s; aliases %s; the alias check works %s" % (
               bad, frag_bad, found, caught))


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
             ua9_simdrone, ua10_router, ua11_gateway, ua12_osdep, ua13_mission, ua14_checks, ua15_hatch,
             ua16_launch, ua17_router_flight,
             ua18_gateway_launch, ua19_helm_grant, ua20_registry_path, ua21_fixed]
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
