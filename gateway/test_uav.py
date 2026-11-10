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
       case, no climb = DISARM + hatch open, the GPS fallback climb (any boat speed, W59), a STOP with a drone on deck
  UA17 the router process flying a launch: mission upload + read-back, ARM only with a fresh hatch OPEN held IN the
       router (U2 review LOW-3), DISARM refused in the air (landed from the drone, LOW-4), ELRS link, MISSION START
  UA18 the real gateway + the simulated AURIX relay board + hatch + drones: 125 for a station's hatch relay, routes,
       the copilot's PREPARE -> the captain's YES -> 2 drones launched, the hatch relay through the AURIX, the boat
       untouched; with no HELM nothing switches the hatch (notice) and a launch is refused
  UA19 W55: the gateway's own HELM grant vs TELEM2 for the hatch relay command; 125 for a station's hatch relay
  --- step U4 (map + telemetry + video + AI + the battery minimum + flight logs) ---
  UA22 the battery minimum = the only landing, inside the router (pure rule + a simulated flight; LAND refused from
       the manager, the router's own LAND, nothing else afterwards); the map telemetry fields
  UA23 the manager's U4 part: map record, notices (battery 60 / 20 s, link, range), uav_telem subscription, the
       per-drone flight log + CSV / GPX export
  UA24 the video worker process: simulated VRX -> ring UAV-n + MediaMTX uav_n, only the slots told to run, stall
  UA25 cameras.py UAV streams: only open ones, data saver / STARLINK none, dropped first, auth
  UA26 the UAV AI: open feeds only, round-robin, the boat's front camera AI first, boxes + notice, never a cap
  UA27 the real gateway with MediaMTX: a station's UAV-1 stream, the ring track in a SAVE, data saver
  --- U3 review fixes (review of 34624bd) ---
  UA28 airspace files never raise (H1), STOP with the ARM on its way (M1), every wait timed (M5), no registration
       while flying (M4), the second alarm word (M2), no FAULT before the first switch reading (L1)
  UA29 router: one job per slot, cancel, MISSION_SET_CURRENT for an in-flight route (M1 / M6)
  --- step U5 (manual control + the pilot lock; owner W61: the captain takes a drone over from a copilot) ---
  UA30 pilot.py: setpoint limits (speed, altitude floor / ceiling, unknown altitude / position, yaw), the air
       geofence / no-fly clip; the lock: first wins 127, the captain only in STANDBY 126, the captain's takeover
       (W61), centred first, old / stale / NaN data ignored, 0.5 s = zero, 3 s / gone = end, STANDBY left = end
       at once, BRAKE -> LOITER, only allow-listed outputs
  UA31 stations.py with the captain's joystick on a UAV: 126 for his driving commands + the copilot's
       ACCEPT-needing ones (never queued), STANDBY / AUTO PAUSE / DP STOP free, no rjoy to the boat
  UA33 U4 review fixes: stale battery %, a command on its way at the battery latch, the allow-list while landing,
       landed(), the deck fallback, MISSION_SET_CURRENT confirmed, the two-stage landing, the latched stage, the UAV
       AI core, the gateway's nav code UAV-free
  UA32 the router's "ctl" op flies a simulated drone (velocity + yaw rate), its own zero after 0.5 s, refused
       while landing for the battery
  UA36 U6 review fixes: MED-3 a STOPPED launch lets an airborne drone go ("hold", the pilot may take it); L1 a thin
       no-fly altitude band inside a climb is seen; L2 an old position moved along the course over ground; L3 one
       lost position packet tolerated (data_stale_s 1.8 s); L4 SYS_STATUS battery_remaining -1 never refreshes the
       battery time (review 2: also a 0.5 m band)
  UA37 (WA L2, review 2) stations.helm_forward (the gateway's newer HELM grant, else TELEM2) + the copilot's 126
       refusal following the captain the command is forwarded for
  UA35 (step WA, owner W59) no boat mode / speed launch condition: boat_state in HDG LOCK / AUTO at 8 kn = no
       reason, high_speed above 5 kn; the manager's NON-blocking HIGH SPEED notice once per crossing while a launch is
       prepared / runs, gw_status high_speed, the captain's question says it, a prepare under way is NOT refused

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
sys.path.insert(1, os.path.join(os.path.dirname(HERE), "tools"))
import strays  # noqa: E402  (CL3: the router / video worker / MediaMTX of a test die with the test)

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
    if not used <= {"boat_state", "boat_mode", "relay_on", "hatch_relay"}:
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
        def _slots():
            with open(path, encoding="utf-8") as f:              # (closed at once: Windows replace / sharing)
                return json.load(f).get("slots", {})
        until(lambda: os.path.exists(path) and _slots().get("3"), 5)
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
        # WA/U6 review 2 LOW: the save retry - a Windows sharing violation 3 times, then written (no loss); every try
        # failing = lost counted + the manager's notice, no wait after the last try
        real_replace, real_sleep, real_wait = registry._replace, registry._sleep, registry.SAVE_WAIT_S
        fails, waits = [3], []

        def flaky(src, dst):
            if fails[0] != 0:
                fails[0] -= 1
                raise PermissionError(13, "sharing violation")
            real_replace(src, dst)
        registry._replace, registry._sleep, registry.SAVE_WAIT_S = flaky, waits.append, 0.0
        try:
            r2.register(5, good_reading(sysid=15), name="FIVE")
            until(lambda: "5" in _slots(), 5)
            retried = "5" in _slots() and r2.error == "" and r2.lost == 0 and len(waits) == 3
            fails[0], waits[:] = -1, []                          # (-1 = fails for ever)
            r2.register(6, good_reading(sysid=16), name="SIX")
            until(lambda: r2.lost == 1, 5)
            lost = r2.lost == 1 and "not saved" in r2.error and "6" not in _slots() and \
                len(waits) == registry.SAVE_TRIES - 1
        finally:
            registry._replace, registry._sleep, registry.SAVE_WAIT_S = real_replace, real_sleep, real_wait
        from uav import manager as mgr
        m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, _G(tmp))
        m.proc = type("P", (), {"poll": lambda _s: None})()
        m.worker_t = time.time()
        m.reg.lost, m.reg.error = 1, "registry not saved: sharing violation"
        m.tick()
        m.tick()
        told = [n for n in m.notices if "registry NOT saved" in n["text"]]
        ok = ok and retried and lost and len(told) == 1
        report("UA7 the registry file (state/uav_registry.json): saved + reloaded; launchable only registered + "
               "checked + the same drone (sysid / UID) in the slot; a duplicate sysid never launchable; a save retried "
               "through sharing violations, all tries failing = counted + ONE notice (no wait after the last)", ok,
               "slot 1 %s, 2 %s, 3 %s, 4 %s, unknown sysid %s, unknown UID %s; 3 failures then saved %s; always "
               "failing: lost + %d waits %s, notice %d" % (l1, l2, l3, l4, l5, l6, retried, len(waits), lost, len(told)))
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
        strays.dies_with_parent(self.p)
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
    # 2) the boat's GPS is lost after the first drone: stop, no further drone, the hatch not switched (W59: the boat's
    #    mode / speed is NOT a launch condition any more - a launch goes when the copilot says so)
    io2 = FakeIo(3)
    seq2 = launch.Launch(io2, {"gap_s": 15.0})
    seq2.prepare([1, 2, 3], 5, 0.0)
    seq2.approved(True, 7, 0.0)

    def leave():
        io2.boat_d["launch_why"] = ["no trusted boat GPS"]
    run_seq(io2, seq2, 200.0, [(10.0, leave)])
    stop_ok = (seq2.state == "STOPPED" and "GPS" in seq2.why and
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
    # 5) no GPS: the climb fallback (ALT_HOLD + TAKEOFF param3 1, 15 m) -> catch-up - in STANDBY AND under way (W59)
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
    fb6 = [w for _t, n, w in io6.sent]
    dp_ok = seq6.state == "DONE" and "mode 2" in fb6 and "takeoff 1/15" in fb6 and "catchup" in fb6
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
           "last, GUIDED -> ARM -> TAKEOFF to the slot altitude, >= 15 s between ARMs; ANY warning stops it (the boat's "
           "GPS lost, a hatch FAULT, the captain's station lost, a slot mismatch, a bad read-back, NO / no "
           "answer) with no further drone and the hatch not switched; armed without a climb = DISARM + the hatch "
           "stays OPEN; no GPS = the ALT_HOLD climb fallback (TAKEOFF param3 1, 15 m) + catch-up, in STANDBY and "
           "under way (W59); a STOP with a drone armed on deck disarms it",
           seq_ok and stop_ok and hf_ok and nc_ok and fb_ok and dp_ok and rb_ok and cap_ok and mm_ok and no_ok and
           deck_ok,
           "normal %s (order %s, gaps %s, relay %s); boat leaves %s (%s); hatch fault %s; no climb %s (%s); fallback "
           "%s (%s); under way %s; read-back %s; captain lost %s; mismatch %s; NO / timeout %s; deck stop %s (%s)" % (
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
        # WA review L4 (W76): the DEMO boat's relay table (demo.py -> the SHORE station's push) is taken after the
        # onboard CONFIRM and makes the hatch relay launch-ready (no relay / hatch reason left for PREPARE LAUNCH)
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "demo"))
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "groundstation"))
        import demo as demo_mod
        import boats as boats_mod
        d_tmp = os.path.join(tmp, "demo_run")
        old_run, demo_mod.RUN = demo_mod.RUN, d_tmp
        try:
            d_path = demo_mod.demo_boat_file()
        finally:
            demo_mod.RUN = old_run
        d_boat = boats_mod.Boat(d_path, json.load(open(d_path, encoding="utf-8")))
        r_demo = alice.result(alice.cmd(proto.T_RELAY_CFG, proto.p_relay_cfg(d_boat.relays)))
        time.sleep(0.5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.3)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        demo_active = until(lambda: g.telem2.get("relay_hash") == proto.relay_table_hash(d_boat.relays) and
                            not g.telem2.get("relay_pending") and g.stations.hatch_relay_ok(HATCH_N)[0], 10)
        demo_why = [w for w in (gw.call(g.uav.launch_conditions, [1]) or []) if "relay" in w or "hatch" in w]
        demo_ok = r_demo == 0 and bool(demo_active) and not demo_why and d_boat.hatch_relay == HATCH_N
        report("UA18d (W76) the demo boat file's relay table: sent by the HELM station, taken after the onboard "
               "CONFIRM, the hatch relay launch-ready (no relay / hatch reason for PREPARE LAUNCH)", demo_ok,
               "result %s, active %s, reasons %s" % (r_demo, bool(demo_active), demo_why))
        bad_t = [dict(r, power_up="on") if r["n"] == HATCH_N else r for r in UA_RELAYS]
        r_bad = alice.result(alice.cmd(proto.T_RELAY_CFG, proto.p_relay_cfg(bad_t)))     # U3 review L5
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
               r_helm == 0 and r_bad == 125 and r_cfg == 0 and active and hatch_ok and r_h_cap == 125 and r_h_cop == 125 and
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


# ------------------------------------------------------------------------------------------------------ U4 ---------

def ua22_battery():
    """The battery minimum (docs 6.2): the pure rule + the router flying a simulated drone: no LAND while the battery
    covers the descent (a NAV_LAND from the manager is refused by the allow-list), LAND by the router's own rule once
    it does not, then every mode / mission command for that drone refused; the telemetry fields of the map."""
    from uav import battery
    j1 = battery.judge(30.0, 50, 3000, 15.0)
    j2 = battery.judge(120.0, 20, 2500, None)
    j3 = battery.judge(None, 50, 3000, 15.0)
    st = [battery.stage(70), battery.stage(55), battery.stage(19), battery.stage(5, True), battery.stage(None)]
    pure = (j1["state"] == "ok" and 200 < j1["eta_s"] < 400 and j2["state"] == "land" and j3["state"] == "unknown"
            and st == [None, 60, 20, "land", None])
    base = free_base()
    fl = simdrone.Fleet(base, 1, ttf=(2.0, 1.0, 0.5), origin=(37.82, 23.60)).start()
    w = Worker()
    try:
        cfg = {"op": "config", "slots": [{"slot": 1, "usb": "udp:127.0.0.1:%d" % (base + 1),
                                          "elrs": "udp:127.0.0.1:%d" % (base + 11), "sysid": 11,
                                          "capacity_mah": 3000}]}
        w.send(cfg)
        w.find(lambda e: e.get("ev") == "status" and e["links"].get("1", {}).get("present") and
               e["radio"].get("1", {}).get("present"), 8)
        seq = [100]

        def req(**kw):
            seq[0] += 1
            w.send(dict(kw, op="cmd", id=seq[0]))
            return w.find(lambda e, i=seq[0]: e.get("id") == i and e.get("ev") == "cmd", 10)
        fl.drones[0].sky = True
        until(lambda: fl.drones[0].fix >= 3, 6)
        req(slot=1, via="usb", command=176, params=[1, 4])
        w.send({"op": "hatch", "open": True})
        arm = req(slot=1, via="usb", command=400, params=[1, 0])
        tko = req(slot=1, via="usb", command=22, params=[0, 0, 0, 0, 0, 0, 30])
        for _ in range(8):                                     # the manager's hatch op stays fresh meanwhile
            w.send({"op": "hatch", "open": True})
            time.sleep(0.5)
        up = until(lambda: fl.drones[0].alt > 15.0, 12)
        land_mgr = req(slot=1, via="elrs", command=21, params=[0] * 7)          # nobody but the rule may land
        time.sleep(2.5)
        no_land = fl.drones[0].mode != 9
        tel = w.find(lambda e: e.get("ev") == "status" and (e["drones"].get("1") or {}).get("judge"), 5)
        d1 = (tel or {}).get("drones", {}).get("1", {})
        lq = ((tel or {}).get("radio", {}).get("1") or {}).get("lq")
        # the battery now too low for the descent (the landing rate made tiny): the router's own rule lands it
        cfg2 = json.loads(json.dumps(cfg))
        cfg2["slots"][0]["capacity_mah"] = 1000
        w.send(dict(cfg2, battery={"uav_land_ms": 0.1}))
        ev = w.find(lambda e: e.get("ev") == "land" and e.get("what") == "start", 6)
        landed_mode = until(lambda: fl.drones[0].mode == 9, 6)
        mode_after = req(slot=1, via="elrs", command=176, params=[1, 3])
        start_after = req(slot=1, via="elrs", command=300, params=[1, 0])
        st2 = w.find(lambda e: e.get("ev") == "status" and (e["drones"].get("1") or {}).get("landing"), 5)
        report("UA22 the battery minimum = the ONLY landing (docs 6.2, inside the router): the pure rule (eta, land, "
               "unknown, the 60 s / 20 s stages); flying: a NAV_LAND from the manager refused (allow-list), no LAND "
               "while the battery covers the descent; once it does not, the router lands it itself (event 'land'), "
               "then a new mode / MISSION START for it refused; the map telemetry (heading, speed, link quality)",
               pure and arm and arm["result"] == 0 and tko and tko["result"] == 0 and up and land_mgr and
               land_mgr["result"] is None and no_land and d1.get("judge", {}).get("state") == "ok" and
               d1.get("hdg") is not None and d1.get("gs") is not None and lq is not None and lq >= 60 and ev and
               landed_mode and mode_after and mode_after["result"] is None and "battery" in (mode_after.get("why") or "")
               and start_after and start_after["result"] is None and st2 is not None,
               "pure %s (%s, %s, %s, %s); arm %s takeoff %s up %s; manager LAND %s; no land before %s; judge %s; hdg %s "
               "gs %s lq %s; land event %s, LAND mode %s; mode after %s, start after %s" % (
                   pure, j1, j2["state"], j3["state"], st, arm and arm["result"], tko and tko["result"], bool(up),
                   land_mgr and land_mgr.get("why"), no_land, d1.get("judge"), d1.get("hdg"), d1.get("gs"), lq,
                   ev and ev.get("why"), bool(landed_mode), mode_after and mode_after.get("why"),
                   start_after and start_after.get("why")))
    finally:
        w.close()
        fl.stop()


class _St:
    def __init__(self, sid=3, role="HELM", crew="copilot"):
        self.id, self.user, self.role, self.name, self.sent = sid, "bob", role, "bob", []
        self.crew, self.single_op, self.local = crew, False, False
        self.session, self.video_key = "sess%d" % sid, "key%d" % sid

    def send(self, m):
        self.sent.append(m)


class _G:
    """A minimal gateway for the manager / cameras / AI unit tests."""

    def __init__(self, root=None):
        self.audits, self.telem2 = [], {}
        self.cfg = {"cameras": {"enabled": True, "rtsp_port": 8554}}
        if root:
            self.storage = type("S", (), {"usable": True, "root": root, "folder": lambda _s, n: os.makedirs(
                os.path.join(root, n), exist_ok=True) or os.path.join(root, n)})()
        else:
            self.storage = None
        self.stations = type("X", (), {"stations": {}})()
        self.clock = type("C", (), {"gps_valid": False, "gps_rx": 0.0, "now": lambda _s: time.time()})()
        self.saver = False
        self.net = type("N", (), {"on_emergency": False})()

    def audit(self, kind, **kw):
        self.audits.append((kind, kw))

    def telem_fresh(self):
        return None

    def data_saver(self):
        return self.saver


def ua23_notices_flightlog():
    """The manager's U4 part with a scripted router: the map record of a flying drone, the notices (battery minimum
    60 s / 20 s, link lost / back, the radio range), the flight log (one file per flight from the ARM, CSV + GPX
    export through uav_logs / uav_log answered from a helper thread), uav_telem only to the stations that asked."""
    from uav import manager as mgr
    tmp = tempfile.mkdtemp(prefix="uavtest_u4m_")
    try:
        g = _G(tmp)
        m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
        m.flog.start()
        m.proc = type("P", (), {"poll": lambda _s: None})()
        m.to_worker = lambda o: True
        m.boat = lambda: {"pos": (37.82, 23.60), "launch_why": []}
        st, st2 = _St(3), _St(4)
        g.stations.stations = {3: st, 4: st2}
        now = [time.time()]

        def drone(eta, lq=100, lat=37.8210, armed=True, alt=40.0):
            m.worker = {"links": {"1": {"present": False}}, "radio": {"1": {"present": lq > 0, "lq": lq,
                                                                              "hb_age": 0.4 if lq else 6.0}},
                        "drones": {"1": {"mode": 3, "armed": armed, "hb_age": 0.3, "rel_alt": alt, "lat": lat,
                                         "lon": 23.6000, "hdg": 45.0, "gs": 8.0, "climb": 0.0, "batt_pct": 40,
                                         "batt_v": 15.1, "batt_a": 15.0, "landed": not armed, "landing": False,
                                         "judge": {"state": "ok", "eta_s": eta} if eta is not None else None}}}
            m.worker_t = now[0]

        def step(dt=1.1):
            now[0] += dt
            m.u4_tick(now[0])
        drone(200)
        step()
        v = m.drones().get("1") or {}
        for eta in (90, 55, 50, 18):
            drone(eta)
            step()
        drone(15, lq=0)
        step()
        drone(15, lq=100)
        step()
        drone(15, lat=37.8350)                                  # ~1550 m from the boat
        step()
        drone(None, armed=False, alt=0.0)
        step()
        texts = [n["text"] for n in m.notices]
        ok_notes = (sum("LANDING IN 60 s" in t for t in texts) == 1 and sum("LANDING IN 20 s" in t for t in texts) == 1
                    and any("LINK LOST" in t for t in texts) and any("link back" in t for t in texts) and
                    any("radio range limit" in t for t in texts))
        # uav_telem only to the station that asked
        m.telem_msg(st, {"t": "uav_telem_on", "on": True})
        drone(200)
        m.next_telem = 0.0
        m.u4_tick(now[0])
        tel = [x for x in st.sent if x.get("t") == "uav_telem"]
        tel2 = [x for x in st2.sent if x.get("t") == "uav_telem"]
        # the flight log: written by the helper thread, exported
        until(lambda: m.flog.lines >= 6, 5)
        m.logs_msg(st, {"t": "uav_logs"})
        lst = until(lambda: [x for x in st.sent if x.get("t") == "uav_logs"], 5)
        logs = lst[-1]["logs"] if lst else []
        out = {}
        if logs:
            for fmt in ("csv", "gpx"):
                m.logs_msg(st, {"t": "uav_log", "name": logs[0]["name"], "file": logs[0]["file"], "fmt": fmt})
                r = until(lambda f=fmt: [x for x in st.sent if x.get("t") == "uav_log" and x.get("fmt") == f], 5)
                out[fmt] = r[-1] if r else {}
        m.logs_msg(st, {"t": "uav_log", "name": "../x", "file": "a.csv", "fmt": "csv"})
        bad = until(lambda: [x for x in st.sent if x.get("t") == "uav_log" and not x.get("ok")], 5)
        csv_ok = out.get("csv", {}).get("ok") and out["csv"]["data"].startswith("utc,slot,sysid,lat,lon,alt_m") and \
            out["csv"]["data"].count("\n") >= 6
        gpx_ok = out.get("gpx", {}).get("ok") and "<trkpt" in out["gpx"]["data"] and "<ele>40.0</ele>" in \
            out["gpx"]["data"]
        report("UA23 the manager's U4 part (scripted router): the map record (position, heading, altitude, battery, "
               "link, distance), notices for the battery minimum 60 s / 20 s (once each), link lost / back, the radio "
               "range limit; uav_telem only to the station that asked; the per-drone flight log (one file per flight, "
               "CSV + GPX export, a bad name refused)",
               v.get("alt") == 40.0 and v.get("hdg") == 45.0 and v.get("lq") == 100 and v.get("dist_m") is not None
               and ok_notes and tel and not tel2 and len(logs) == 1 and csv_ok and gpx_ok and bad,
               "view %s; notices %s; telem %d / other %d; logs %s; csv %s gpx %s bad %s" % (
                   {k: v.get(k) for k in ("alt", "hdg", "lq", "dist_m", "stage")}, texts, len(tel), len(tel2), logs,
                   bool(csv_ok), bool(gpx_ok), bool(bad)))
        m.flog.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def ua24_video_worker():
    """The video worker (its own process): the simulated VRX -> one encode -> the ring files UAV-n (1 file per
    segment, UTC names) AND MediaMTX uav_n (a reader gets a frame); only the slots told to run; a frozen VRX =
    STALLED + restarted; run [] = OFF."""
    import toolpaths
    import cameras
    ff, mtx = toolpaths.ffmpeg(), toolpaths.mediamtx()
    if not ff:
        return report("UA24 the UAV video worker", NA, "no ffmpeg on this PC")
    tmp = tempfile.mkdtemp(prefix="uavtest_video_")
    mp = vp = None
    try:
        port = None
        if mtx:
            s = socket.socket()
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
            s.close()
            api = socket.socket()
            api.bind(("127.0.0.1", 0))
            api_port = api.getsockname()[1]
            api.close()
            cm = cameras.CameraManager({"rtsp_port": port, "api_port": api_port, "list": [],
                                        "webrtc_port": 0, "log_level": "error"})
            cm.uav_slots = [1, 2]
            y = cm.mediamtx_yaml("http://127.0.0.1:1/none").replace("authMethod: http", "authMethod: internal")
            y = "\n".join(l for l in y.splitlines() if not l.startswith(("authHTTP", "  - action", "webrtc")))
            y += "\nwebrtc: no\nauthInternalUsers:\n  - user: any\n    permissions:\n      - action: publish\n" \
                 "      - action: read\n"
            conf = os.path.join(tmp, "mtx.yml")
            with open(conf, "w", encoding="utf-8") as f:
                f.write(y)
            mp = subprocess.Popen([mtx, conf], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            strays.dies_with_parent(mp)
            time.sleep(1.0)
        ring = os.path.join(tmp, "ring")
        vp = subprocess.Popen([sys.executable, os.path.join(UAV_DIR, "video_worker.py"), "--ffmpeg", ff,
                               "--rtsp-port", str(port or 1), "--publish", "1" if port else "0", "--ring", ring,
                               "--seg-s", "3", "--w", "320", "--h", "240", "--fps", "10", "--kbps", "300",
                               "--encoder", "x264", "--stall-s", "3"],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, cwd=UAV_DIR)
        strays.dies_with_parent(vp)
        ev = []
        lock = threading.Lock()

        def rd():
            for line in vp.stdout:
                try:
                    with lock:
                        ev.append(json.loads(line))
                except ValueError:
                    pass
        threading.Thread(target=rd, daemon=True).start()

        def send(o):
            vp.stdin.write((json.dumps(o) + "\n").encode())
            vp.stdin.flush()

        def last(cond):
            with lock:
                for e in reversed(ev):
                    if cond(e):
                        return e
            return None
        send({"op": "config", "devices": {"1": "sim", "2": "sim"}})
        send({"op": "run", "slots": [1]})
        run1 = until(lambda: last(lambda e: e.get("ev") == "status" and e["slots"].get("1", {}).get("state") == "RUN"
                                  and e["slots"]["1"]["frames"] > 20), 20)
        s2 = (run1 or {}).get("slots", {}).get("2", {}).get("state")
        files = until(lambda: [f for f in os.listdir(os.path.join(ring, "UAV-1"))
                               if re.match(r"^\d{8}-\d{6}\.ts$", f)] if os.path.isdir(os.path.join(ring, "UAV-1"))
                      else None, 15)
        got = None
        if port:
            r = subprocess.run([ff, "-hide_banner", "-loglevel", "error", "-rtsp_transport", "tcp", "-i",
                                "rtsp://any:x@127.0.0.1:%d/uav_1" % port, "-frames:v", "3", "-f", "null", "-"],
                               capture_output=True, timeout=25)
            got = r.returncode == 0
        send({"op": "sim", "slot": 1, "freeze": True})
        stalled = until(lambda: last(lambda e: e.get("ev") == "status" and e["slots"].get("1", {}).get("restarts", 0)
                                     >= 1), 15)
        send({"op": "sim", "slot": 1, "freeze": False})
        send({"op": "run", "slots": []})
        off = until(lambda: last(lambda e: e.get("ev") == "status" and e["slots"].get("1", {}).get("state") == "OFF"),
                    10)
        report("UA24 the UAV video worker (own process, simulated VRX): only the slot told to run is encoded, the ring "
               "files UAV-1/YYYYMMDD-HHMMSS.ts, MediaMTX uav_1 readable%s, a frozen VRX = stalled + restarted, run [] = "
               "OFF" % ("" if port else " (N/A: no MediaMTX)"),
               run1 and s2 == "OFF" and files and (got is None or got) and stalled and off,
               "run %s, slot 2 %s, files %s, rtsp frame %s, restarts %s, off %s" % (
                   bool(run1), s2, (files or [])[:3], got, stalled and stalled["slots"]["1"]["restarts"], bool(off)))
    finally:
        for p in (vp, mp):
            if p is not None:
                try:
                    if p is vp:
                        p.stdin.close()
                        p.wait(5)
                except Exception:
                    pass
                if p.poll() is None:
                    p.kill()
                    p.wait(5)
        shutil.rmtree(tmp, ignore_errors=True)


def ua25_uav_streams():
    """cameras.py and the UAV streams: a station gets UAV-n ONLY while open on its screen, nothing on data saver /
    STARLINK (for anyone), dropped first (also the captain's) when the video is reduced for the captain, a UAV the
    gateway has no video for refused, MediaMTX auth only for allowed paths; the captain's UAV kbps is not counted as
    his priority video."""
    import cameras
    g = _G()
    cap, cop = _St(1, crew="captain"), _St(2, crew="copilot")
    g.stations.stations = {1: cap, 2: cop}
    cm = cameras.CameraManager({"list": [], "copilot_budget_kbps": 1000, "view_restore_s": 0.0}, g)
    cm.uav_slots = [1, 2, 3]
    cm.uav_view_request(cop, {"t": "uav_view", "uav": 2, "open": True})
    a1 = cop.sent[-1]
    cm.uav_view_request(cop, {"t": "uav_view", "uav": 6, "open": True})
    bad = cop.sent[-1]
    cm.uav_view_request(cap, {"t": "uav_view", "uav": 1, "open": True})
    auth_ok = cm.auth({"action": "read", "path": "uav_2", "ip": "10.0.0.5", "user": cop.session,
                       "password": cop.video_key, "query": "station=2"})
    auth_no = cm.auth({"action": "read", "path": "uav_1", "ip": "10.0.0.5", "user": cop.session,
                       "password": cop.video_key, "query": "station=2"})
    feeds = cm.uav_views()
    g.saver = True
    sv = cm.streams_msg(cop)
    feeds_saver = cm.uav_views()
    g.saver = False
    # over the budget while the captain receives video: UAV streams dropped first (the captain's too)
    cm.station_kbps = {1: 1300, 2: 400}
    cm.sta_path_kbps = {(1, "cam_front1_hi"): 900, (1, "uav_1"): 400, (2, "uav_2"): 400}
    cm.total_kbps = 1700
    changed = cm.update_priority(now=100.0)
    red_cop, red_cap = cm.streams_msg(cop), cm.streams_msg(cap)
    cm.uav_view_request(cop, {"t": "uav_view", "uav": None})
    closed = cm.uav_open(cop.id)
    report("UA25 UAV streams (cameras.py): UAV-n only while open on a screen, a UAV without video refused, MediaMTX "
           "auth only for an open + allowed UAV path, data saver / STARLINK = no UAV video for anyone (notice), over "
           "the budget the UAV streams go first (also the captain's own), close = no stream",
           a1.get("streams") == {"UAV-2": "uav_2"} and not bad.get("ok", True) and auth_ok and not auth_no and
           feeds == {1, 2} and not any(k.startswith("UAV") for k in sv["streams"]) and "no UAV video" in sv["notice"]
           and feeds_saver == set() and changed and cm.views_reduced and
           not any(k.startswith("UAV") for k in red_cop["streams"]) and
           not any(k.startswith("UAV") for k in red_cap["streams"]) and red_cap["uav_reduced"] and not closed,
           "open %s, refused %s, auth %s / %s, feeds %s, saver %s (%s), reduced %s -> copilot %s captain %s, closed %s" %
           (a1.get("streams"), bad.get("text"), auth_ok, auth_no, feeds, sv["streams"], sv["notice"], changed,
            red_cop["streams"], red_cap["streams"], closed))


def ua26_uav_ai():
    """The UAV AI (uavai.py) with a scripted worker: nothing without an open feed, round-robin over the open feeds
    (one URL per dwell), WAITING (worker paused) while the boat's FRONT camera AI is PAUSED / DEGRADED, boxes + ONE
    notice per feed, <= 0.5 fps; and no UAV module can give the boat a speed cap / navigation request."""
    from uav import uavai
    g = _G()

    class V:
        cfg, state, paused = {"enabled": True}, "ACTIVE", False
    g.vision = V()
    views = [set()]
    g.cams = type("C", (), {"uav_views": lambda _s: views[0]})()
    notes = []
    mgr = type("M", (), {"gw": g, "video_running": lambda _s: [1, 2, 3], "notice": lambda _s, t, l="info":
                         notes.append(t)})()
    ai = uavai.UavAi({"cpu_pct": 1000.0, "dwell_s": 8.0}, mgr)
    sent = []

    class P:
        def poll(self):
            return None
    ai._spawn_bg = lambda: setattr(ai, "proc", P())
    ai.send = lambda o: sent.append(o) or True
    t = 1000.0
    ai.tick(t)
    idle = ai.state
    views[0] = {1, 3}
    ai.tick(t + 1)                                  # starts the worker
    urls = []
    for k in range(2, 30):
        ai.tick(t + k)
    urls = [o["url"] for o in sent if o.get("cmd") == "url"]
    g.vision.state = "PAUSED"
    ai.tick(t + 31)
    waiting, paused = ai.state, any(o.get("cmd") == "pause" for o in sent)
    g.vision.state = "ACTIVE"
    ai.tick(t + 32)
    resumed = any(o.get("cmd") == "resume" for o in sent)
    blob = {"x0": 0.1, "y0": 0.2, "x1": 0.3, "y1": 0.4, "conf": 0.8}
    ai.result(1, [blob], t + 33)
    ai.result(1, [blob], t + 34)
    ai.result(1, [dict(blob, conf=0.2)], t + 35)
    st = ai.status(t + 35.5)
    src = "".join(open(p, encoding="utf-8").read() for p in py_files([UAV_DIR]))
    code = re.sub(r"#.*|\"\"\"[\s\S]*?\"\"\"", "", src)
    nav_free = not re.search(r"\bnav\(|navreq|nav_compose|cap_kn|speed_cap", code)
    report("UA26 the UAV AI (scripted worker): IDLE without an open feed, round-robin over the open feeds (uav_1 / "
           "uav_3, 8 s each), WAITING + worker paused while the boat's front camera AI is PAUSED (front first), boxes "
           "+ ONE notice, max 0.5 fps; no gateway/uav code touches a speed cap / navigation request",
           idle == "IDLE" and urls[:3] == ["rtsp://127.0.0.1:8554/uav_1", "rtsp://127.0.0.1:8554/uav_3",
                                           "rtsp://127.0.0.1:8554/uav_1"] and waiting == "WAITING" and paused and
           resumed and len(notes) == 1 and st["feeds"]["1"]["frames"] == 3 and st["max_fps"] <= 0.5 and nav_free,
           "idle %s, urls %s, waiting %s (paused %s, resumed %s), notices %s, feeds %s, nav-free %s" % (
               idle, urls[:4], waiting, paused, resumed, notes, st["feeds"], nav_free))


def ua27_gateway_video():
    """The real gateway with MediaMTX + the simulated drones + the video worker (slot 1 'always'): a station opens
    UAV 1 -> its stream in cams_ack and readable with the station's own login; the ring track UAV-1 in a SAVE; data
    saver = no UAV stream; the boat untouched."""
    import test_gateway as tg
    from gwclient import GatewayLink
    if not os.path.exists(tg.EXE) or not os.path.exists(tg.MTX_BIN):
        return report("UA27 UAV video through the real gateway", NA, "no simulator / MediaMTX")
    tmp = tempfile.mkdtemp(prefix="uavtest_gwv_")
    rig, links = None, []
    try:
        base = free_base()
        rig = tg.CamRig(tmp, recording={"enabled": True, "segment_s": 4, "pre_s": 30, "post_s": 2, "stall_s": 60},
                        uav={"enabled": True, "sim": True, "sim_base_port": base,
                             "video": {"enabled": True, "always": [1], "encoder": "x264", "w": 320, "h": 240,
                                       "fps": 10}})
        g = rig.g
        sp = rig.cfg["stations"]["port"]
        a = GatewayLink("127.0.0.1:%d" % sp, "alice", password=tg.PW["alice"], station="alice")
        a.start()
        links.append(a)
        seen = []

        def last(kind, cond=lambda d: True):
            while True:
                try:
                    seen.append(a.events.get_nowait())
                except Exception:
                    break
            for k, d in reversed(seen):
                if k == kind and cond(d):
                    return d
            return None
        tel0 = until(lambda: last("telemetry"), 10)
        run = until(lambda: g.uav.video_state(1) == "RUN", 30)
        a.send_msg({"t": "uav_view", "uav": 1, "open": True})
        ack = until(lambda: last("cams_ack", lambda d: "UAV-1" in (d.get("streams") or {})), 10)
        got = None
        if ack:
            url = a.video_url(ack, "UAV-1")
            r = subprocess.run([tg.ffmpeg_bin(), "-hide_banner", "-loglevel", "error", "-rtsp_transport", "tcp", "-i",
                                url, "-frames:v", "3", "-f", "null", "-"], capture_output=True, timeout=25)
            got = r.returncode == 0
        files = until(lambda: len(g.uav.tracks["UAV-1"].segments()) >= 3, 20)
        ok, _msg, eid = g.rec.trigger("button", "UA27", who="test")
        ev_files = until(lambda: (lambda p: [f for f in os.listdir(p) if f.endswith(".ts")] if os.path.isdir(p)
                                  else None)(os.path.join(g.rec.events_dir(), eid or "x", "UAV-1")), 30)
        g.data_saver = lambda: True
        a.send_msg({"t": "uav_view", "uav": 1, "open": True})
        sv = until(lambda: last("cams_ack", lambda d: "streams" in d and "UAV-1" not in d["streams"] and
                                d.get("uav_reduced")), 10)
        st = until(lambda: last("gw_status", lambda d: ((d.get("uav") or {}).get("video") or {}).get("slots")), 5)
        tel1 = last("telemetry")
        report("UA27 UAV video through the real gateway (MediaMTX, simulated VRX on slot 1): the video worker RUNs, "
               "a station's uav_view -> UAV-1 in its streams and readable with its own login, the ring track UAV-1 "
               "linked into a SAVE, data saver = no UAV stream (notice), gw_status uav video, the boat untouched",
               run and ack and got and files and ok and ev_files and sv and st and tel0 and tel1 and
               tel1["mode"] == tel0["mode"],
               "run %s, ack %s, frame %s, ring %s, save %s (%s files), saver %s (%s), status %s" % (
                   bool(run), bool(ack), got, bool(files), eid, len(ev_files or []), bool(sv), sv and sv.get("notice"),
                   bool(st)))
    finally:
        for l in links:
            try:
                l.stop()
            except Exception:
                pass
        if rig is not None:
            try:
                rig.stop()
            except Exception:
                pass
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------------------------------- U3 review fixes -------

def ua28_review_u3():
    """U3 review (34624bd): H1 a malformed airspace file never raises (Airspace + the manager build; every route
    refused); M1 a STOP while the ARM is on its way: the ARM cancelled in the router + DISARM, the result reported
    (never assumed); M5 every wait timed (a command / relay answer that never comes, the router offline = stop, the
    CLOSING not stuck); M4 (un)registration refused while a launch runs / the drone flies; M2 the second alarm word;
    L3 an answer to an older launch question approves nothing; L1 SerialSwitches: no FAULT before the first reading."""
    from uav import hatch, launch, manager as mgr, nofly
    import uavlink
    tmp = tempfile.mkdtemp(prefix="uavtest_u3r_")
    try:
        # H1
        bad = []
        for k, (zf, gf) in enumerate((("[]", None), ("5", None), ("true", None), ('{"zones": 5}', None),
                                      ('{"zones": [7]}', None), (None, "[1, 2]"), (None, "true"), ("{oops", None))):
            d = os.path.join(tmp, "a%d" % k)
            os.makedirs(d)
            for name, txt in (("uav_nofly.json", zf), ("uav_geofence.json", gf)):
                if txt is not None:
                    with open(os.path.join(d, name), "w", encoding="utf-8") as f:
                        f.write(txt)
            try:
                a = nofly.Airspace(d)
                bad.append(bool(a.error) and a.problem([(37.82, 23.6, 0.0), (37.83, 23.6, 50.0)]) is not None)
            except Exception as e:
                bad.append("raised %r" % e)
        g = _G(tmp)
        os.makedirs(os.path.join(tmp, "state"), exist_ok=True)
        with open(os.path.join(tmp, "state", "uav_nofly.json"), "w", encoding="utf-8") as f:
            f.write("[1, 2, 3]")
        try:
            m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
            m_off = mgr.UavManager({"enabled": False, "hatch": {"port": "COM999"}}, g)
            h1_mgr = bool(m.airspace.error) and m_off.switches is None
        except Exception as e:
            h1_mgr = "raised %r" % e
        h1 = all(x is True for x in bad) and h1_mgr is True
        # M1: the ARM answer held back (as on a slow link), STOP meanwhile
        io = FakeIo(1, climb=False)                          # (on deck: the ACK late, the motors spinning)
        held = []
        cancels = []
        real_cmd = io.cmd

        def slow_cmd(n, via, command, params, cb):
            if command == 400 and params[0] == 1:
                io.drones[n]["armed"] = True                  # the drone DID arm, the ACK is late
                io.sent.append((io.t, n, "arm"))
                held.append(cb)
                return
            real_cmd(n, via, command, params, cb)
        io.cmd = slow_cmd
        io.cancel = lambda n, c=None: cancels.append((n, c))
        seq = launch.Launch(io)
        seq.prepare([1], 5, 0.0)
        seq.approved(True, 7, 0.0)
        run_seq(io, seq, 30.0, [(4.0, lambda: seq.stop("STOP by test", io.t))])
        s1 = [w for _t, n, w in io.sent]
        m1 = (seq.state == "STOPPED" and "arm" in s1 and s1[-1] == "disarm" and cancels == [(1, 400)] and
              any("disarmed in the hatch" in t for t in io.notices))
        # M5: a command never answered -> stop after cmd_s; the router offline -> stop; CLOSING relay never answered
        io2 = FakeIo(1)
        io2.cmd = lambda n, via, command, params, cb: io2.sent.append((io2.t, n, "lost %d" % command))
        seq2 = launch.Launch(io2, {"cmd_s": 3.0})
        seq2.prepare([1], 5, 0.0)
        seq2.approved(True, 7, 0.0)
        run_seq(io2, seq2, 40.0)
        io3 = FakeIo(2)
        io3.router_ok = lambda: io3.t < 3.0
        seq3 = launch.Launch(io3)
        seq3.prepare([1, 2], 5, 0.0)
        seq3.approved(True, 7, 0.0)
        run_seq(io3, seq3, 30.0)
        io4 = FakeIo(1)
        relays = []

        def hcmd(on, cb):
            io4.relay.append((io4.t, on))
            io4.hatch_cmd_open = on
            io4.hatch_t = io4.t
            if on:
                io4.q.append(lambda: cb(True, ""))
            else:
                relays.append(cb)                             # the close is never answered
        io4.hatch_cmd = hcmd
        seq4 = launch.Launch(io4, {"relay_s": 4.0})
        seq4.prepare([1], 5, 0.0)
        seq4.approved(True, 7, 0.0)
        run_seq(io4, seq4, 120.0)
        m5 = (seq2.state == "STOPPED" and "no answer" in seq2.why and seq3.state == "STOPPED" and
              "router offline" in seq3.why and seq4.state == "STOPPED" and "HATCH NOT CLOSED" in seq4.why)
        # M4 + L3 through the manager
        m.proc = type("P", (), {"poll": lambda _s: None})()
        m.state = "ACTIVE"
        m.to_worker = lambda o: True
        st = _St(3)
        g.stations.stations = {3: st}
        m.seq.state = "RUN"
        m.register_msg(st, {"slot": 2})
        r_busy = st.sent[-1]
        m.seq.state = "IDLE"
        m.flying[2] = {"phase": "mission"}
        m.worker = {"links": {"2": {"present": False}}, "drones": {"2": {"armed": True, "hb_age": 0.2}}}
        m.worker_t = time.time()
        m.register_msg(st, {"slot": 2})
        r_fly = st.sent[-1]
        m.unregister_msg(st, {"slot": 2})
        u_fly = st.sent[-1]
        m4 = (not r_busy["ok"] and "launch" in r_busy["text"] and not r_fly["ok"] and "flying" in r_fly["text"] and
              not u_fly["ok"] and "flying" in u_fly["text"])
        # M2: a WARNING alarm in the SECOND word (TELEM2 group L) blocks a launch
        class GW2:
            telem2 = {"alarm_bits2": 0x20, "relay_ok": True}          # SB_ALM2_CAN_NODE: not an advisory

            def telem_fresh(self):
                return {"mode": 0, "mode_name": "STANDBY", "pos_source": 1, "est_lat": 37.8, "est_lon": 23.6,
                        "est_sog": 0.0, "alarm_bits": 0, "alarm_bits2": 0}
            stations = type("S", (), {"hatch_relay_ok": lambda _s, n: (True, ""),
                                      "helm_station": lambda _s: (_St(1), ""), "captain": lambda _s: None})()
        b2 = uavlink.boat_state(GW2(), 8)
        m2 = any("WARNING" in w for w in b2["launch_why"])
        # L1: no FAULT latched before the first switch reading
        sw = hatch.SerialSwitches("")
        hh = hatch.Hatch(10.0, 10.0)
        st0 = hh.update(0.0, sw.read())
        st1 = hh.update(6.0, sw.read())
        l1 = st0 == "UNKNOWN" and st1 == "FAULT"
        report("UA28 U3 review fixes: H1 a malformed airspace file (list / number / true / bad zones / broken JSON) "
               "never raises, every route refused, a disabled magazine builds nothing that can fail; M1 a STOP while "
               "the ARM is on its way = the ARM cancelled + DISARM, reported from its result; M5 a command / relay "
               "answer that never comes, the router offline = stop (CLOSING never stuck); M4 no (un)registration "
               "while a launch runs / the drone flies; M2 the second alarm word counts; L1 no FAULT before the first "
               "switch reading",
               h1 and m1 and m5 and m4 and m2 and l1,
               "H1 %s / manager %s; M1 %s (sent %s, cancels %s, notices %s); M5 cmd %s / router %s / closing %s; M4 "
               "%s / %s / %s; M2 %s; L1 %s -> %s" % (
                   bad, h1_mgr, m1, s1, cancels, io.notices[-2:], seq2.why, seq3.why, seq4.why, r_busy.get("text"),
                   r_fly.get("text"), u_fly.get("text"), b2["launch_why"], st0, st1))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def ua29_review_router():
    """U3 review M1 / M6 in the router process + the simulated drone: one command job per slot (an older one is
    answered "replaced"), a "cancel" answers the pending ARM, MISSION_START's first item is IGNORED by a drone already
    in AUTO (like ArduPilot) and MISSION_SET_CURRENT picks it; the manager's in-flight route change uses it."""
    from uav import manager as mgr, mission
    base = free_base()
    fl = simdrone.Fleet(base, 1, ttf=(2.0, 1.0, 0.5), origin=(37.82, 23.60)).start()
    w = Worker()
    try:
        w.send({"op": "config", "slots": [{"slot": 1, "usb": "udp:127.0.0.1:%d" % (base + 1),
                                           "elrs": "udp:127.0.0.1:%d" % (base + 11), "sysid": 11}]})
        w.find(lambda e: e.get("ev") == "status" and e["links"].get("1", {}).get("present"), 8)
        seq = [500]

        def send(**kw):
            seq[0] += 1
            w.send(dict(kw, id=seq[0]))
            return seq[0]

        def answer(i, t=10):
            return w.find(lambda e: e.get("id") == i and e.get("ev") in ("cmd", "mission"), t)
        # two commands for one slot: the first answered "replaced" at once
        d0 = fl.drones[0]
        old_ack = d0.ack
        d0.ack = lambda *a: None                               # the drone answers nothing for a while
        i1 = send(op="cmd", slot=1, via="usb", command=176, params=[1, 4])
        i2 = send(op="cmd", slot=1, via="usb", command=176, params=[1, 5])
        a1 = answer(i1, 4)
        i3 = send(op="cancel", slot=1, command=176)
        a2 = answer(i2, 4)
        d0.ack = old_ack
        # MISSION_START in AUTO ignores param1; MISSION_SET_CURRENT picks the item
        items = mission.build([{"lat": 37.8210, "lon": 23.6, "alt": 30.0}, {"lat": 37.8220, "lon": 23.6, "alt": 30.0},
                               {"lat": 37.8230, "lon": 23.6, "alt": 30.0}], (37.82, 23.60), 30.0, 8.0)
        up = answer(send(op="mission", slot=1, via="usb", items=items), 15)
        d0.sky = True
        until(lambda: d0.fix >= 3, 6)
        with d0.lock:
            d0.armed, d0.mode, d0.cur = True, 3, 5
        answer(send(op="cmd", slot=1, via="elrs", command=300, params=[2, 0]), 5)
        cur_after_start = d0.cur
        sc = answer(send(op="set_current", slot=1, via="elrs", seq=4), 5)
        cur_after_set = until(lambda: d0.cur == 4 and 4, 3)
        with d0.lock:
            d0.armed, d0.mode = False, 5
        # the manager's in-flight change: upload, MISSION_SET_CURRENT, MISSION START
        g = _G()
        m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
        calls = []
        m.proc = type("P", (), {"poll": lambda _s: None})()
        m.worker = {"links": {"1": {"present": False}}, "radio": {"1": {"present": True}},
                    "drones": {"1": {"armed": True, "hb_age": 0.2, "lat": 37.8205, "lon": 23.6, "rel_alt": 40.0,
                                     "batt_pct": 90}}}
        m.worker_t = time.time()
        m.flying[1] = {"phase": "mission", "home": (37.82, 23.60)}
        m.boat = lambda: {"pos": (37.82, 23.60), "launch_why": []}
        m.upload = lambda n, items, via, cb: calls.append("upload") or cb(True, "")
        m.set_current = lambda n, s, via, cb: calls.append("set_current %d" % s) or cb(0, "")
        m.cmd = lambda n, via, c, p, cb: calls.append("cmd %d" % c) or cb(0, "")
        st = _St(3, crew="captain")                     # PF2-u: only the captain's own route applies at once
        m.route_msg(st, {"t": "uav_route", "target": 1, "wps": [{"lat": 37.8215, "lon": 23.6005, "alt": 40}]})
        report("UA29 U3 review M1 / M6 (router + simulated drone): one command job per slot (the older one answered "
               "'replaced'), a cancel answers the pending command, a drone in AUTO ignores MISSION_START's first item "
               "(like ArduPilot) and MISSION_SET_CURRENT picks it; the in-flight route change = upload -> "
               "MISSION_SET_CURRENT -> MISSION START",
               a1 and a1["result"] is None and "replaced" in a1["why"] and a2 and a2["result"] is None and
               "cancelled" in a2["why"] and up and up["ok"] and cur_after_start == 5 and sc and sc["result"] == 0 and
               cur_after_set == 4 and calls == ["upload", "set_current %d" % mission.FLY_FROM, "cmd 300"] and
               st.sent and st.sent[-1].get("ok"),
               "replaced %s, cancelled %s, upload %s, cur after START %s, set_current %s -> %s, manager %s %s" % (
                   a1 and a1.get("why"), a2 and a2.get("why"), up and up.get("ok"), cur_after_start,
                   sc and sc.get("result"), cur_after_set, calls, st.sent[-1:] if st.sent else None))
    finally:
        w.close()
        fl.stop()


def ua37_pf2u_flying_route_needs_yes():
    """PF2-u (owner 2026-11-09): a route for a FLYING drone from a copilot / single operator is PENDING until the
    captain's YES (ask_captain, as the launch approval); the captain's own applies at once; NO / timeout / replaced /
    landed / pilot = nothing changes; a drone in the magazine still STORES a copilot's route; a pilot lock = 127."""
    from uav import manager as mgr, pilot
    res = {}

    def build():
        g = _G()
        asks = []
        g.stations = type("X", (), {"stations": {}, "ask_captain": lambda _s, st, m, what, act: asks.append(
            (st, m, what, act))})()
        mm = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
        calls = []
        mm.proc = type("P", (), {"poll": lambda _s: None})()
        mm.worker = {"links": {"1": {"present": False}}, "radio": {"1": {"present": True}},
                     "drones": {"1": {"armed": True, "hb_age": 0.2, "lat": 37.8205, "lon": 23.6, "rel_alt": 40.0,
                                      "batt_pct": 90}}}
        mm.worker_t = time.time()
        mm.flying[1] = {"phase": "mission", "home": (37.82, 23.60)}
        mm.boat = lambda: {"pos": (37.82, 23.60), "launch_why": []}
        mm.upload = lambda n, items, via, cb: calls.append("upload") or cb(True, "")
        mm.set_current = lambda n, s, via, cb: calls.append("set_current") or cb(0, "")
        mm.cmd = lambda n, via, c, p, cb: calls.append("start") or cb(0, "")
        return mm, asks, calls
    msg = {"t": "uav_route", "target": 1, "wps": [{"lat": 37.8215, "lon": 23.6005, "alt": 40}]}
    # copilot: pending, nothing sent to the drone until the YES
    mm, asks, calls = build()
    cop = _St(3)
    mm.route_msg(cop, dict(msg))
    res["pending"] = (not calls and len(asks) == 1 and cop.sent[-1].get("ok") and cop.sent[-1].get("pending") and
                      "pending the captain" in cop.sent[-1]["text"] and mm.routes.get(1) is None)
    asks[0][3](True, 0)
    res["yes"] = calls == ["upload", "set_current", "start"] and cop.sent[-1]["ok"] and "started" in cop.sent[-1]["text"]
    # NO / timeout (code) = unchanged, the copilot is told
    mm, asks, calls = build()
    cop = _St(3)
    mm.route_msg(cop, dict(msg))
    asks[0][3](False, 112)
    res["no"] = not calls and not cop.sent[-1]["ok"] and "old one goes on" in cop.sent[-1]["text"] and 1 not in mm.routes
    # a newer route replaces the older question: the old answer approves nothing
    mm, asks, calls = build()
    cop = _St(3)
    mm.route_msg(cop, dict(msg))
    mm.route_msg(cop, dict(msg))
    asks[0][3](True, 0)
    res["replaced"] = not calls
    asks[1][3](True, 0)
    res["replaced"] = res["replaced"] and calls == ["upload", "set_current", "start"]
    # landed / pilot took it before the YES = dropped
    mm, asks, calls = build()
    cop = _St(3)
    mm.route_msg(cop, dict(msg))
    mm.flying.pop(1)
    mm.pend_expire_tick()
    asks[0][3](True, 0)
    res["landed"] = not calls and not mm.pend_routes
    mm, asks, calls = build()
    mm.route_msg(cop, dict(msg))
    mm.pilots = type("PL", (), {"locks": {1: pilot.Lock(9, "anna", time.time())}})()
    asks[-1][3](True, 0)
    res["pilot"] = not calls and not cop.sent[-1]["ok"]
    # the captain's own route applies at once, no question
    mm, asks, calls = build()
    cap = _St(4, crew="captain")
    mm.route_msg(cap, dict(msg))
    res["captain"] = calls == ["upload", "set_current", "start"] and not asks
    # a single operator asks his own question (ask_captain, confirm self)
    mm, asks, calls = build()
    so = _St(5, crew="captain")
    so.single_op = True
    mm.route_msg(so, dict(msg, confirm="self"))
    res["single"] = not calls and len(asks) == 1 and asks[0][1].get("confirm") == "self"
    asks[0][3](True, 0)
    res["single"] = res["single"] and calls == ["upload", "set_current", "start"]
    # a pilot-locked drone stays refused 127, before any question
    mm, asks, calls = build()
    mm.pilots = type("PL", (), {"locks": {1: pilot.Lock(9, "anna", time.time())}})()
    cop = _St(3)
    mm.route_msg(cop, dict(msg))
    res["127"] = not asks and not calls and cop.sent[-1]["code"] == pilot.ERR_PILOT
    # a drone in the magazine: a copilot only STORES the route (preparation, no YES)
    mm, asks, calls = build()
    mm.flying.clear()
    mm.worker["links"]["1"] = {"present": True}
    cop = _St(3)
    mm.route_msg(cop, dict(msg))
    res["store"] = not asks and not calls and cop.sent[-1]["ok"] and "stored" in cop.sent[-1]["text"]         or (not asks and not calls and cop.sent[-1].get("ok") is not None and 1 in mm.routes)
    report("UA37 PF2-u: a route for a FLYING UAV from a copilot / single operator is pending the captain's YES "
           "(nothing sent to the drone before it; NO / timeout / replaced / landed / pilot = the old mission goes on), "
           "the captain's own applies at once, a pilot lock stays 127, a magazine drone still stores a copilot's route",
           all(res.values()), str(res))


def ua39_pf4r_pending_races():
    """PF4-r MED-2 (review of 2de2437): (a) the captain's own direct route clears the slot's pending copilot route AND
    closes its question (an older YES can never overwrite the newer mission); (b) a replaced question closes on screen 1
    and its copilot is told; (c) the question SHOWS the route (points, length, last point, altitude + the points for
    screen 1 / the map); (d) while a launch sequence is asked / runs, a copilot's route for ANY slot of it is pending
    the captain's YES (not stored at once), the captain's own is stored; LOW: the copilot gone = dropped + closed."""
    from uav import manager as mgr, launch
    res = {}

    class Sts:
        def __init__(self):
            self.stations, self.asks, self.cancelled, self.n = {}, {}, [], 0

        def ask_captain(self, st, m, what, act, extra=None):
            self.n += 1
            self.asks[self.n] = (st, m, what, act, extra)
            return self.n

        def cancel_ask(self, aid, code=None):
            self.cancelled.append(aid)
            return self.asks.pop(aid, None) is not None

    def build(flying=True):
        g = _G()
        g.stations = Sts()
        mm = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
        calls = []
        mm.proc = type("P", (), {"poll": lambda _s: None})()
        mm.worker = {"links": {"1": {"present": not flying}}, "radio": {"1": {"present": True}},
                     "drones": {"1": {"armed": flying, "hb_age": 0.2, "lat": 37.8205, "lon": 23.6, "rel_alt": 40.0,
                                      "batt_pct": 90}}}
        mm.worker_t = time.time()
        if flying:
            mm.flying[1] = {"phase": "mission", "home": (37.82, 23.60)}
        mm.boat = lambda: {"pos": (37.82, 23.60), "launch_why": []}
        mm.upload = lambda n, items, via, cb: calls.append(("upload", items[-1].get("x") if isinstance(
            items[-1], dict) else None)) or cb(True, "")
        mm.set_current = lambda n, s_, via, cb: calls.append("set_current") or cb(0, "")
        mm.cmd = lambda n, via, c, p, cb: calls.append("start") or cb(0, "")
        return mm, g.stations, calls
    w_a = [{"lat": 37.8215, "lon": 23.6005, "alt": 40}, {"lat": 37.8225, "lon": 23.6015, "alt": 40}]
    w_b = [{"lat": 37.8230, "lon": 23.6020, "alt": 60}]
    # (a) a copilot's pending route, then the captain's own: the question is closed, the old YES approves nothing
    mm, sts, calls = build()
    cop, cap = _St(3), _St(4, crew="captain")
    sts.stations = {3: cop, 4: cap}
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    aid = max(sts.asks)
    old_act = sts.asks[aid][3]
    mm.route_msg(cap, {"t": "uav_route", "target": 1, "wps": w_b})
    n_cap = len(calls)
    old_act(True, 0)                                         # the older question's YES arriving late
    res["a"] = (aid in sts.cancelled and n_cap == 3 and len(calls) == 3 and not mm.pend_routes and
                any(x.get("replaced") and not x["ok"] for x in cop.sent))
    # (b) a newer copilot route replaces the older question: closed on screen 1, the copilot told
    mm, sts, calls = build()
    cop = _St(3)
    sts.stations = {3: cop}
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    a1 = max(sts.asks)
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_b})
    a2 = max(sts.asks)
    res["b"] = a1 in sts.cancelled and a2 != a1 and a2 in sts.asks and any(x.get("replaced") for x in cop.sent) and \
        not calls
    # (c) the question shows the route
    what, extra = sts.asks[a2][2], sts.asks[a2][4]
    res["c"] = ("1 points" in what and "last point 37.82300 23.60200" in what and "60 m" in what and extra and
                extra["uav_route"]["slot"] == 1 and extra["uav_route"]["wps"] == [[37.8230, 23.6020, 60]])
    q2 = mgr.route_question(1, w_a)
    res["c"] = res["c"] and "2 points" in q2 and "0.1 km" in q2 and "40 m" in q2
    # (d) a drone of an asked / running launch sequence (not the current one): a copilot's route is PENDING
    for state in (launch.ASK, launch.RUN):
        mm, sts, calls = build(flying=False)
        cop, cap = _St(3), _St(4, crew="captain")
        sts.stations = {3: cop, 4: cap}
        mm.seq.state, mm.seq.slots, mm.seq.cur, mm.seq.phase = state, [2, 1], 2, "CLIMB"
        mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
        pend = sts.asks and cop.sent[-1].get("pending") and mm.routes.get(1) is None
        sts.asks[max(sts.asks)][3](True, 0)
        stored_after_yes = mm.routes.get(1) == uavcheck_clean(w_a)
        mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_b})
        mm.route_msg(cap, {"t": "uav_route", "target": 1, "wps": w_a})
        cap_ok = cap.sent[-1]["ok"] and "stored" in cap.sent[-1]["text"] and not mm.pend_routes
        res["d_" + state] = bool(pend and stored_after_yes and cap_ok)
    # not in a sequence: a copilot still STORES a magazine route (preparation)
    mm, sts, calls = build(flying=False)
    cop = _St(3)
    sts.stations = {3: cop}
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    res["store"] = not sts.asks and mm.routes.get(1) is not None
    # LOW: the copilot disconnects -> the pending route dropped and its question closed
    mm, sts, calls = build()
    cop = _St(3)
    sts.stations = {3: cop}
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    a3 = max(sts.asks)
    sts.stations = {}
    mm.pend_expire_tick()
    res["gone"] = not mm.pend_routes and a3 in sts.cancelled
    # PF4-r2 MED-1 (review of 64521b1) (e): launch ASK + a pending copilot route A, the launch is cancelled, the copilot
    # stores a NEWER route B directly, the captain answers YES on the OLD question -> B stays, A's question closed
    w_c = [{"lat": 37.8240, "lon": 23.6030, "alt": 50}]
    mm, sts, calls = build(flying=False)
    cop = _St(3)
    sts.stations = {3: cop}
    mm.seq.state, mm.seq.slots, mm.seq.cur, mm.seq.phase = launch.ASK, [1, 2], None, ""
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    ae = max(sts.asks)
    old_e = sts.asks[ae][3]
    mm.seq.state = launch.IDLE                                # the launch was cancelled (the tick has not run yet)
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_b})
    stored_b = mm.routes.get(1) == uavcheck_clean(w_b)
    old_e(True, 0)                                            # the captain's late YES on the old question
    res["e_old_yes"] = (stored_b and mm.routes.get(1) == uavcheck_clean(w_b) and ae in sts.cancelled and
                        not mm.pend_routes and any(x.get("replaced") for x in cop.sent))
    # (f) the sequence ends -> the tick drops its pending route, closes the question, tells the copilot; a late YES = 0
    mm, sts, calls = build(flying=False)
    cop = _St(3)
    sts.stations = {3: cop}
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_c})          # not in a sequence: stored
    mm.seq.state, mm.seq.slots, mm.seq.cur, mm.seq.phase = launch.RUN, [1, 2], 2, "CLIMB"
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    af = max(sts.asks)
    old_f = sts.asks[af][3]
    mm.pend_expire_tick()
    still = bool(mm.pend_routes)                              # still running: kept
    mm.seq.state = launch.STOPPED
    mm.pend_expire_tick()
    old_f(True, 0)
    res["f_seq_over"] = (still and not mm.pend_routes and af in sts.cancelled and mm.routes.get(1) == uavcheck_clean(w_c)
                         and any(x.get("dropped") and not x["ok"] for x in cop.sent))
    # (g) the YES applies only while the stored route is still the one it was asked against (route generation)
    mm, sts, calls = build(flying=False)
    cop = _St(3)
    sts.stations = {3: cop}
    mm.seq.state, mm.seq.slots, mm.seq.cur, mm.seq.phase = launch.ASK, [1], None, ""
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    ag = max(sts.asks)
    mm._set_route(1, uavcheck_clean(w_b))                     # any other store meanwhile
    sts.asks[ag][3](True, 0)
    res["g_gen"] = mm.routes.get(1) == uavcheck_clean(w_b) and "newer route was stored" in cop.sent[-1]["text"] and \
        not cop.sent[-1]["ok"]
    # (h) LOW-5: a pending FLYING route dropped because the drone landed -> the copilot gets an ack (never silent)
    mm, sts, calls = build()
    cop = _St(3)
    sts.stations = {3: cop}
    mm.route_msg(cop, {"t": "uav_route", "target": 1, "wps": w_a})
    mm.flying.pop(1, None)
    mm.pend_expire_tick()
    res["h_ack"] = not mm.pend_routes and bool(cop.sent) and cop.sent[-1].get("dropped") and not cop.sent[-1]["ok"]
    report("UA39 PF4-r MED-2 + PF4-r2 MED-1: the captain's own route clears a pending copilot route + closes its "
           "question (a late YES approves nothing), a replaced question closes + its copilot is told, the question "
           "SHOWS the route, a copilot's route for ANY drone of an asked / running launch sequence is pending the "
           "captain's YES, the requesting station gone = dropped; EVERY direct store closes the pending route (an old "
           "YES after a cancelled launch never overwrites the newer route), a pending route goes when its launch "
           "sequence ends, the YES applies only to the exact route asked (route generation), a drop is acked",
           all(res.values()), str(res))


def uavcheck_clean(wps):
    from uav import uavcheck
    return uavcheck.clean(wps, 50.0)


def ua38_pf4_attitude_and_camera():
    """PF4: the router's drone record carries the ATTITUDE roll / pitch (deg) + its age for marking a point from the
    video, the manager passes the slot's fixed camera mount (gateway.json uav slots[].camera, only numbers), and the
    simulated drone sends a level ATTITUDE."""
    from uav import manager as mgr, router_worker as rw
    d = rw.Drone()
    v0 = d.view(time.time())
    d.roll, d.pitch, d.att_t = 3.0, -2.0, time.time() - 0.4
    v1 = d.view(time.time())
    m = object.__new__(mgr.UavManager)
    m.cfg = {"slots": [{"camera": {"tilt_down_deg": 45, "hfov_deg": 100.5, "vfov_deg": "x", "gimbal": 9}}, {}, {"camera": 7}]}
    ok_cam = m._camera(1) == {"tilt_down_deg": 45.0, "hfov_deg": 100.5} and m._camera(2) is None and         m._camera(3) is None and m._camera(9) is None
    sd = simdrone.SimDrone.__dict__ if hasattr(simdrone, "SimDrone") else {}
    src = open(os.path.join(HERE, "uav", "simdrone.py"), encoding="utf-8").read()
    report("UA38 PF4: the drone record has roll / pitch / att_age (none before the first ATTITUDE), the slot's fixed "
           "camera mount passes only numbers (no gimbal key), the simulated drone sends a level ATTITUDE",
           v0["roll"] is None and v0["att_age"] is None and v1["roll"] == 3.0 and v1["pitch"] == -2.0 and
           0.3 < v1["att_age"] < 0.6 and ok_cam and '"roll": 0.0, "pitch": 0.0' in src,
           "%s %s %s" % (v0.get("roll"), v1.get("roll"), ok_cam))


# ------------------------------------------------------------------------------------------------------ U5 ---------

class _PilotMgr:
    """A scripted UAV manager for pilot.py (UA30 / UA31): drone records, the router ops + commands it was asked."""

    def __init__(self, gw):
        from uav import uavcheck
        self.gw, self.enabled, self.n = gw, True, 6
        self.ccfg = dict(uavcheck.DEFAULTS)
        self.airspace = None
        self.why = ""
        self.ops, self.cmds = [], []
        self.drones = {1: {"hb_age": 0.2, "armed": True, "landed": False, "usb_present": False, "landing": False,
                           "lat": 37.83, "lon": 23.61, "alt": 50.0, "hdg": 90.0, "alt_age": 0.3, "gps_fix": 3},
                       2: {"hb_age": 0.2, "armed": False, "landed": True, "usb_present": True}}

    def slot(self, n):
        return dict(self.drones.get(n) or {})

    def drone_view(self, n):
        return self.slot(n)

    def router_ok(self):
        return True

    def to_worker(self, obj):
        self.ops.append(obj)
        return True

    def cmd(self, n, via, command, params, cb):
        self.cmds.append((n, via, command, list(params)))
        cb(0, "")


class _PGw:
    def __init__(self):
        self.telem2 = {}
        self.mode = 0
        self.audits = []
        self.stations = type("S", (), {"stations": {}, "age": lambda _s, st, ts: time.time() - ts})()

    def telem_fresh(self):
        return None if self.mode is None else {"mode": self.mode}

    def audit(self, kind, **kw):
        self.audits.append((kind, kw))


def ua30_pilot():
    """pilot.py: the setpoint limits + clipping, the lock (first wins, the captain's takeover W61, STANDBY only for the
    captain), centred first, stale / lost, the hover on give-back, only allow-listed outputs."""
    import common  # noqa: F401  (the path to proto, as the gateway)
    from uav import pilot
    cfg = dict(pilot.DEFAULTS, uav_min_alt_m=10.0, uav_max_alt_m=120.0)
    dr = {"lat": 37.83, "lon": 23.61, "alt": 50.0, "hdg": 0.0, "alt_age": 0.1}
    sp = lambda st, d=None, air=None: pilot.setpoint(st, dict(dr, **(d or {})), cfg, air)   # noqa: E731
    fwd = sp({"y": 1.0})[0]
    diag = sp({"x": 1.0, "y": 1.0})[0]
    dead = sp({"x": 0.05, "y": -0.07, "z": 0.06, "r": 0.02})[0]
    floor = sp({"z": -1.0}, {"alt": 10.6})
    at_floor = sp({"z": -1.0}, {"alt": 9.0})[0]
    ceil = sp({"z": 1.0}, {"alt": 119.5})
    no_alt = sp({"z": 1.0}, {"alt": None})
    no_pos = sp({"y": 1.0}, {"lat": None})
    yaw = sp({"r": 1.0})[0]

    class Air:                                      # the fence: north of 37.8303 is outside; a zone 70..200 m
        def problem(self, pts):                     # (U5 review MED-3: the stub has an altitude band)
            if any(p[0] > 37.8303 for p in pts):
                return "outside"
            return "no-fly zone 70..200 m" if any(70.0 <= p[2] <= 200.0 for p in pts) else None
    edge = sp({"y": 1.0}, None, Air())              # heading north, 40 m ahead: outside -> hold
    east = sp({"x": 1.0}, None, Air())              # sideways east: fine
    back_in = sp({"y": -1.0}, {"lat": 37.8304}, Air())   # outside already: flying south (back in) is let through
    band = sp({"z": 1.0}, {"alt": 55.0}, Air())     # MED-3: climbing into the zone's altitude band = no climb
    band_ok = band[0]["vz"] == 0.0 and band[1] and "above / below" in band[1][0] and \
        sp({"z": 1.0}, {"alt": 30.0}, Air())[0]["vz"] < 0
    limits_ok = (abs(fwd["vx"] - 10.0) < 1e-6 and fwd["vy"] == 0 and abs(math.hypot(diag["vx"], diag["vy"]) - 10.0)
                 < 1e-3 and dead == pilot.ZERO and abs(floor[0]["vz"] - 0.6) < 1e-6 and floor[1] and
                 at_floor["vz"] == 0.0 and abs(ceil[0]["vz"] + 0.5) < 1e-6 and ceil[1] and no_alt[0]["vz"] == 0.0 and
                 no_alt[1] and no_pos[0]["vx"] == 0.0 and no_pos[1] and abs(yaw["yaw_rate"] - math.radians(60)) < 1e-3
                 and edge[0]["vx"] == 0.0 and "geofence" in edge[1][0] and east[0]["vy"] > 9 and
                 back_in[0]["vx"] < -9 and band_ok)
    ok_allow = all(allow.check_out("SET_POSITION_TARGET_LOCAL_NED", dict(s, target_system=11, target_component=1,
                                                                         coordinate_frame=pilot.FRAME,
                                                                         type_mask=pilot.MASK))[0]
                   for s in (fwd, diag, floor[0], ceil[0], yaw))
    report("UA30a manual flight setpoints (pilot.setpoint): horizontal <= 10 m/s (also diagonal), the dead band = "
           "zero, the descent slows to 10 m and stops there, the climb to 120 m, an unknown altitude / position = no "
           "vertical / horizontal speed, the yaw rate, the air geofence ahead = hold at the edge (sideways / back "
           "in let through); every setpoint passes the allow-list",
           limits_ok and ok_allow,
           "fwd %s diag %s dead %s floor %s at floor %s ceil %s no alt %s no pos %s yaw %s edge %s east %s back %s "
           "allow %s" % (fwd, diag, dead, floor, at_floor, ceil, no_alt, no_pos, yaw, edge, east, back_in, ok_allow))

    gw = _PGw()
    mgr = _PilotMgr(gw)
    P = pilot.Pilots(mgr)
    A, B, C = _St(3, crew="copilot"), _St(4, crew="copilot"), _St(5, crew="captain")
    A.name, B.name, C.name = "anna", "bob", "cap"
    for s in (A, B, C):
        gw.stations.stations[s.id] = s
    gw.telem2["helm"] = C.id
    gw.mode = 3                                     # the boat in AUTO
    P.pilot_msg(A, {"uav": 1, "take": True})        # a copilot: any time
    a_ok = A.sent[-1]
    guided = mgr.cmds[-1]
    P.pilot_msg(B, {"uav": 1, "take": True})        # the first pilot wins
    b_ref = B.sent[-1]
    P.pilot_msg(C, {"uav": 1, "take": True})        # the captain, boat not in STANDBY
    c_ref = C.sent[-1]
    P.pilot_msg(A, {"uav": 2, "take": True})        # a drone in its slot is not flown
    slot_ref = A.sent[-1]
    gw.mode = 0                                     # STANDBY
    n_ops = len(mgr.ops)
    P.pilot_msg(C, {"uav": 1, "take": True})        # owner W61: the captain takes over at once
    c_ok = C.sent[-1]
    a_end = [m for m in A.sent if m.get("t") == "uav_pilot_end"]
    hold_ho = [o for o in mgr.ops[n_ops:] if o.get("op") == "ctl"]
    P.pilot_msg(A, {"uav": 1, "take": True})        # a copilot never takes it from the captain
    a_ref = A.sent[-1]
    on_uav = P.joystick_on_uav(C)
    a_on = P.joystick_on_uav(A)
    t0 = time.time()

    def ctl(st, seq, ts=None, **stick):
        n0 = len(mgr.ops)
        P.ctl(st, dict({"uav": 1, "seq": seq, "ts": time.time() if ts is None else ts}, **stick))
        return [o for o in mgr.ops[n0:] if o.get("op") == "ctl"]
    held = ctl(C, 1, y=1.0)                         # deflected at the take: zero until seen centred
    cen = ctl(C, 2)
    go = ctl(C, 3, y=1.0, z=0.5)
    old = ctl(C, 3, y=-1.0)                         # repeated seq: ignored
    stale_ts = ctl(C, 4, ts=time.time() - 1.0, y=-1.0)
    nan = ctl(C, 5, y=float("nan"))
    a_ctl = ctl(A, 6, y=1.0)                        # the old pilot's data: ignored
    P.locks[1].last_t = time.time() - 0.7
    n0 = len(mgr.ops)
    P.tick()
    stale_zero = [o for o in mgr.ops[n0:] if o.get("op") == "ctl"]
    stale_flag = P.locks[1].stale
    gw.mode = 3                                     # the boat leaves STANDBY: the captain's control ends at once
    n0c = len(mgr.cmds)
    P.tick()
    c_end = [m for m in C.sent if m.get("t") == "uav_pilot_end"]
    brake = [c for c in mgr.cmds[n0c:] if c[2] == allow.CMD_DO_SET_MODE]
    after = P.joystick_on_uav(C)
    P.loiter[1] = time.time() - 0.1
    P.tick()
    loiter = mgr.cmds[-1]
    # the copilot again (boat in AUTO: fine for him), lost after 3 s, station gone
    P.pilot_msg(A, {"uav": 1, "take": True})
    P.locks[1].last_t = time.time() - 3.5
    P.tick()
    lost = [m for m in A.sent if m.get("t") == "uav_pilot_end"][-1:]
    P.pilot_msg(B, {"uav": 1, "take": True})
    del gw.stations.stations[B.id]
    P.tick()
    gone = 1 not in P.locks
    give_ok = None
    P.pilot_msg(A, {"uav": 1, "take": True})
    P.pilot_msg(A, {"uav": 1, "take": False})
    give_ok = A.sent[-1]
    modes = {int(c[3][1]) for c in mgr.cmds if c[2] == allow.CMD_DO_SET_MODE}
    only_allowed = all(o.get("op") == "ctl" for o in mgr.ops) and modes <= {allow.COPTER_GUIDED, allow.COPTER_BRAKE,
                                                                            allow.COPTER_LOITER} and \
        {c[2] for c in mgr.cmds} == {allow.CMD_DO_SET_MODE}
    lock_ok = (a_ok["ok"] and guided[2:] == (allow.CMD_DO_SET_MODE, [1, allow.COPTER_GUIDED]) and
               b_ref["code"] == 127 and c_ref["code"] == 126 and slot_ref["code"] == 122 and c_ok["ok"] and
               a_end and a_end[0]["why"] == "captain" and hold_ho and all(o["vx"] == 0 == o["vy"] for o in hold_ho)
               and a_ref["code"] == 127 and on_uav == 1 and a_on is None and P.stats["takeover"] == 1)
    ctl_ok = (held and held[0]["vx"] == 0.0 and cen and cen[0]["vx"] == 0.0 and go and go[0]["vx"] > 9 and
              go[0]["vz"] < 0 and not old and not stale_ts and not nan and not a_ctl and stale_zero and
              stale_zero[0]["vx"] == 0.0 and stale_flag)
    end_ok = (c_end and c_end[-1]["why"] == "standby" and brake and brake[0][3][1] == allow.COPTER_BRAKE and
              after is None and loiter[3][1] == allow.COPTER_LOITER and lost and lost[0]["why"] == "lost" and gone and
              give_ok and give_ok["ok"] and not give_ok.get("take"))
    report("UA30b the pilot lock: a copilot flies any time, the FIRST pilot wins (127), the captain only in STANDBY "
           "(126), owner W61 = the captain takes a drone over from a copilot AT ONCE (the copilot told 'captain', the "
           "drone holds), a copilot never takes it from the captain (127), a drone in its slot never (122); the stick "
           "held deflected at the take = zero until centred, old / stale / NaN / another station's data ignored, "
           "0.5 s stale = zero, the boat leaving STANDBY ends the captain's control at once (BRAKE -> LOITER), 3 s "
           "lost / a station gone ends it; only GUIDED / BRAKE / LOITER + velocity setpoints are ever asked for",
           lock_ok and ctl_ok and end_ok and only_allowed,
           "lock %s ctl %s end %s allowed %s (a %s b %s c %s slot %s cap %s a_end %s a_ref %s; held %s cen %s go %s "
           "old %s stale %s nan %s; stale0 %s; c_end %s brake %s loiter %s lost %s gone %s give %s modes %s)" % (
               lock_ok, ctl_ok, end_ok, only_allowed, a_ok.get("ok"), b_ref.get("code"), c_ref.get("code"),
               slot_ref.get("code"), c_ok.get("ok"), a_end[:1], a_ref.get("code"), held[:1], cen[:1], go[:1], old,
               stale_ts, nan, stale_zero[:1], c_end[-1:], brake[:1], loiter, lost, gone, give_ok, modes))
    _ = t0


class _Writer:
    def __init__(self):
        self.lines = []

    def write(self, b):
        self.lines.append(json.loads(b))


class _Aurix:
    connected = True

    def __init__(self):
        self.sent = []

    def on(self, *_a):
        pass

    def send(self, t, p):
        self.sent.append((t, bytes(p)))


def ua31_stations_126():
    """stations.py while the CAPTAIN's joystick flies a UAV: his driving commands + every copilot command needing his
    ACCEPT refused 126 at once (never queued), STANDBY / AUTO PAUSE / DP STOP free, his rjoy never reaches the boat,
    the gateway's own ACCEPT questions refused; all back to normal when the lock ends; a COPILOT flying changes
    nothing for the boat."""
    import common  # noqa: F401
    import proto
    import stations as stm
    from uav import pilot
    gw = _PGw()
    gw.aurix = _Aurix()
    gw.storage = None
    gw.uav = type("U", (), {})()
    ss = stm.StationServer(gw, {})
    gw.stations = ss
    mgr = _PilotMgr(gw)
    P = gw.uav.pilots = pilot.Pilots(mgr)

    def mk(sid, crew, licence=True):
        st = stm.Station(sid, "u%d" % sid, "HELM", False, "1.2.3.%d" % sid, "s%d" % sid, _Writer())
        st.crew, st.licence = crew, licence
        st.tokens, st.refill = 20.0, time.time()
        ss.stations[sid] = st
        return st
    cap, cop = mk(5, "captain"), mk(6, "copilot")
    gw.telem2["helm"] = cap.id
    gw.mode = 0
    seq = [0]

    def cmd(st, ftype, payload=b""):
        seq[0] += 1
        n0 = len(gw.aurix.sent)
        st.writer.lines.clear()
        ss.command(st, {"seq": seq[0], "ts": time.time(), "type": ftype, "p": payload.hex(), "sid": st.session})
        fwd = any(t == proto.T_GW_CMD for t, _p in gw.aurix.sent[n0:])
        acks = [m for m in st.writer.lines if m.get("t") == "ack"]
        return "fwd" if fwd else (acks[-1]["result"] if acks else None)

    def rjoy(st, s):
        n0 = len(gw.aurix.sent)
        ss._rjoy(st, {"seq": s, "ts": time.time(), "x": 0.0, "y": 0.0, "lever": 0.5})
        return any(t == proto.T_GW_RJOY for t, _p in gw.aurix.sent[n0:])
    hdg = bytes([proto.MODE_HDG_LOCK])
    standby = bytes([proto.MODE_STANDBY])
    before = (cmd(cap, proto.T_SET_MODE, hdg), rjoy(cap, 1))
    P.pilot_msg(cop, {"uav": 1, "take": True})      # the COPILOT flies: nothing changes for the captain / boat
    cop_flying = (cmd(cap, proto.T_SET_MODE, hdg), rjoy(cap, 2))
    P.pilot_msg(cap, {"uav": 1, "take": True})      # the captain takes it over (W61), boat in STANDBY
    capfly = {
        "hdg": cmd(cap, proto.T_SET_MODE, hdg),
        "remote": cmd(cap, proto.T_REMOTE, bytes([0])),
        "gen_on": cmd(cap, proto.T_GENERATOR, bytes([1])),
        "standby": cmd(cap, proto.T_SET_MODE, standby),
        "pause": cmd(cap, proto.T_AUTO_PAUSE),
        "dp_stop": cmd(cap, proto.T_DP, bytes([proto.DP_STOP, 0, 0, 0])),
        "cop_gen_on": cmd(cop, proto.T_GENERATOR, bytes([1])),
        "cop_restart": cmd(cop, proto.T_RESTART, bytes([proto.ENG_RESTART])),
        "cop_return": cmd(cop, proto.T_AUTO_RETURN),
        "cop_standby": cmd(cop, proto.T_SET_MODE, standby),
        "rjoy": rjoy(cap, 3),
    }
    asked = []
    ss.ask_local(cop, {}, "vision stage 3", lambda ok, code: asked.append((ok, code)))
    queued = len(ss.asks)
    P.pilot_msg(cap, {"uav": 1, "take": False})
    after = (cmd(cap, proto.T_SET_MODE, hdg), rjoy(cap, 4))
    want = {"hdg": 126, "remote": 126, "gen_on": 126, "standby": "fwd", "pause": "fwd", "dp_stop": "fwd",
            "cop_gen_on": 126, "cop_restart": 126, "cop_return": 126, "cop_standby": "fwd", "rjoy": False}
    report("UA31 stations.py with the captain's joystick on a UAV: his driving commands (HDG LOCK, REMOTE, GENERATOR "
           "ON) and the copilot's ACCEPT-needing ones (GENERATOR ON, RESTART, AUTO RETURN, a gateway setting) = 126 "
           "at once, never queued; STANDBY / AUTO PAUSE / DP STOP forwarded; his rjoy dropped; a COPILOT flying "
           "changes nothing; normal again after RETURN TO BOAT",
           before == ("fwd", True) and cop_flying == ("fwd", True) and capfly == want and asked == [(False, 126)] and
           queued == 0 and after == ("fwd", True) and getattr(cap, "rjoy_uav", 0) == 1,
           "before %s copilot flying %s captain flying %s ask %s queued %d after %s" % (
               before, cop_flying, capfly, asked, queued, after))


def ua32_router_ctl():
    """The router's "ctl" op flies a simulated drone in GUIDED by velocity setpoints (body frame, yaw rate), the
    router's own zero 0.5 s after the last moving setpoint (the drone stops), refused while the drone lands for its
    battery."""
    base = free_base()
    fl = simdrone.Fleet(base, 1, ttf=(1.0, 0.5, 0.3), origin=(37.82, 23.60)).start()
    w = Worker()
    try:
        w.send({"op": "config", "slots": [{"slot": 1, "usb": "udp:127.0.0.1:%d" % (base + 1),
                                           "elrs": "udp:127.0.0.1:%d" % (base + 11), "sysid": 11}]})
        w.find(lambda e: e.get("ev") == "status" and e["links"].get("1", {}).get("present"), 8)
        d = fl.drones[0]
        d.sky = True
        until(lambda: d.fix >= 3, 6)
        seq = [0]

        def req(**kw):
            seq[0] += 1
            w.send(dict(kw, op="cmd", id=seq[0]))
            return w.find(lambda e, i=seq[0]: e.get("id") == i and e.get("ev") == "cmd", 10)
        w.send({"op": "hatch", "open": True})
        req(slot=1, via="usb", command=176, params=[1, 4])
        req(slot=1, via="usb", command=400, params=[1, 0])
        req(slot=1, via="usb", command=22, params=[0, 0, 0, 0, 0, 0, 30])
        until(lambda: d.alt > 25.0, 20)
        req(slot=1, via="elrs", command=176, params=[1, 4])
        n0, e0, y0 = d.n, d.e, d.yaw
        t_end = time.time() + 1.5
        while time.time() < t_end:
            w.send({"op": "ctl", "slot": 1, "vx": 5.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.5})
            time.sleep(0.1)
        moved = math.hypot(d.n - n0, d.e - e0)
        turned = abs(((d.yaw - y0) + math.pi) % (2 * math.pi) - math.pi)
        time.sleep(1.2)                         # no more setpoints: the router's own zero
        n1, e1 = d.n, d.e
        time.sleep(0.6)
        drift = math.hypot(d.n - n1, d.e - e1)
        zeros = w.find(lambda e: e.get("ev") == "status" and e.get("stats", {}).get("ctl_zero", 0) >= 1, 3)
        # the battery rule latched: no setpoint leaves the router any more
        w.send({"op": "hatch", "open": False})
        d.used_mah = d.capacity * 0.97
        landing = w.find(lambda e: e.get("ev") == "status" and (e.get("drones", {}).get("1") or {}).get("landing"), 8)
        w.send({"op": "ctl", "slot": 1, "vx": 5.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0})
        refused = w.find(lambda e: e.get("ev") == "status" and e.get("stats", {}).get("ctl_refused", 0) >= 1, 3)
        report("UA32 the router's manual setpoints: a simulated drone in GUIDED flies the velocity + yaw rate, the "
               "router's own zero setpoint 0.5 s after the last moving one stops it, no setpoint while it lands for "
               "its battery",
               moved > 4.0 and turned > 0.3 and drift < 0.5 and zeros is not None and landing is not None and
               refused is not None,
               "moved %.1f m, turned %.2f rad, drift after the zero %.2f m, zero %s, landing %s, refused %s" % (
                   moved, turned, drift, zeros is not None, landing is not None, refused is not None))
    finally:
        w.close()
        fl.stop()


# ------------------------------------------------------------------------------------------- U4 review fixes -------

class _FakeLink:
    def __init__(self, slot, kind):
        self.slot, self.kind, self.seq, self.up = slot, kind, 0, True
        self.frames = []
        self.hb_t = time.monotonic()

    def present(self, now):
        return True

    def write(self, frame):
        self.frames.append(frame)
        return True


def ua33_review_u4():
    """U4 review fixes inside the router (in this process, a fake link): MED-1 a stale battery % is never judged
    (no LAND on an old value) + one batt_stale event; MED-2 a command already on its way when the battery rule latches
    is dropped (never re-sent) and the allow-list refuses mode / mission / take-off / speed / ARM / setpoints while
    landing (LAND / DISARM still pass); LOW landed() not on the altitude alone, the deck fallback only from a low last
    altitude, MISSION_SET_CURRENT answered only after MISSION_CURRENT shows the seq; the two-stage landing time, the
    battery stage latched per flight, the UAV AI's core never one of the boat vision's; UA26's grep also over the
    gateway's own nav code."""
    from uav import battery, router_worker as rw
    events = []
    old_out = rw.out
    rw.out = lambda obj: events.append(obj)
    try:
        r = rw.Router()
        lk = _FakeLink(1, "elrs")
        r.links[(1, "elrs")] = lk
        r.expect[1] = 11
        r.capacity[1] = 2500
        now = rw.now_s()
        d = r.drone(1)
        d.hb, d.hb_t = {"base_mode": 0x80, "custom_mode": 3}, now
        d.rel_alt, d.vz, d.alt_t = 40.0, 0.0, now
        d.batt_pct, d.batt_t = 2, now - 10.0              # 2 % - but not heard for 10 s: STALE
        r.battery_tick(now)
        stale_ev = [e for e in events if e.get("ev") == "batt_stale"]
        no_land = not r.battery_land.get(1) and (d.judge or {}).get("state") == "unknown"
        d.batt_t = now                                    # fresh again: 2 % = LAND
        job = rw.CmdJob(7, 1, lk, allow.CMD_DO_SET_MODE, [1, allow.COPTER_AUTO, 0, 0, 0, 0, 0], now)
        job.sent = True
        r.cmds[1] = job
        r.battery_tick(now + 1)
        latched = r.battery_land.get(1)
        dropped = [e for e in events if e.get("ev") == "cmd" and e.get("id") == 7]
        n0 = len(lk.frames)
        # a command re-inserted (in flight) after the latch: jobs_tick never re-sends it
        job2 = rw.CmdJob(8, 1, lk, allow.CMD_MISSION_START, [1, 0, 0, 0, 0, 0, 0], now)
        job2.t_sent = now - 5
        r.cmds[1] = job2
        r.jobs_tick(now + 2)
        resent = [f for f in lk.frames[n0:]]
        dropped2 = [e for e in events if e.get("ev") == "cmd" and e.get("id") == 8]
        ctx = {"battery_land": True, "landed": True, "hatch_open": True}
        tgt = {"target_system": 11, "target_component": 1}
        refused = [not allow.check_out("COMMAND_LONG", dict(tgt, command=c, param1=p1, param2=p2, param7=10.0),
                                       ctx)[0]
                   for c, p1, p2 in ((176, 1, 4), (300, 1, 0), (22, 0, 0), (178, 1, 5), (400, 1, 0))]
        sp_ref = not allow.check_out("SET_POSITION_TARGET_LOCAL_NED", dict(tgt, coordinate_frame=9,
                                                                           type_mask=0x5C7), ctx)[0]
        land_ok = allow.check_out("COMMAND_LONG", dict(tgt, command=21), ctx)[0]
        disarm_ok = allow.check_out("COMMAND_LONG", dict(tgt, command=400, param1=0), ctx)[0]
        # LOW: landed() - 0.3 m while descending fast is NOT on the ground
        d2 = rw.Drone()
        d2.rel_alt, d2.vz, d2.alt_t = 0.3, -1.2, now
        not_landed = not d2.landed(now)
        d2.vz = 0.0
        landed = d2.landed(now)
        # LOW: the deck fallback (stale data, the USB cable present) only from a low last altitude
        usb = _FakeLink(2, "usb")
        r.links[(2, "usb")] = usb
        d3 = r.drone(2)
        d3.rel_alt, d3.alt_t = 35.0, now - 10.0
        high_deck = r.on_deck(2, d3, now)
        d3.rel_alt = 0.2
        low_deck = r.on_deck(2, d3, now)
        # LOW: MISSION_SET_CURRENT answered after the drone's MISSION_CURRENT only
        events.clear()
        r.links[(3, "elrs")] = _FakeLink(3, "elrs")
        r.expect[3] = 13
        d4 = r.drone(3)
        r.command({"op": "set_current", "slot": 3, "seq": 2, "id": 21, "via": "elrs"}, now)
        early = [e for e in events if e.get("id") == 21]
        d4.mission_cur, d4.mcur_t = 2, now + 0.2
        r.jobs_tick(now + 0.3)
        confirmed = [e for e in events if e.get("id") == 21]
        r.command({"op": "set_current", "slot": 3, "seq": 4, "id": 22, "via": "elrs"}, now + 1)
        r.jobs_tick(now + 6)
        timed_out = [e for e in events if e.get("id") == 22]
    finally:
        rw.out = old_out
    # the two-stage landing, the latched stage
    t30 = battery.descent_s(30.0, battery.cfg_of())
    t5 = battery.descent_s(5.0, battery.cfg_of())
    seq = []
    prev = None
    for s in (None, 60, None, 20, 60, None, "land"):
        prev = battery.latch(prev, s)
        seq.append(prev)
    # the UAV AI's core: never one of the boat vision's
    from uav import uavai
    g = _G()
    g.vision = type("V", (), {"cfg": {"threads": 2, "cores": None}})()
    ai = uavai.UavAi({}, type("M", (), {"gw": g})())
    n = os.cpu_count() or 4
    core = ai.core_for()
    g.vision.cfg = {"threads": 2, "cores": [0, 1]}
    core2 = ai.core_for()
    # UA26's grep, extended: the gateway's own navigation code never reads the UAV magazine
    gsrc = open(os.path.join(HERE, "gateway.py"), encoding="utf-8").read()
    navs = re.findall(r"\n    def (nav_\w+)\(self[\s\S]*?(?=\n    def |\nclass |\Z)", gsrc)
    bodies = [m.group(0) for m in re.finditer(r"\n    def nav_\w+\(self[\s\S]*?(?=\n    def |\nclass |\Z)", gsrc)]
    nav_clean = bool(bodies) and not any(re.search(r"\buav\b|self\.uav|pilots", b) for b in bodies)
    ok = (stale_ev and stale_ev[0]["stale"] and no_land and latched and dropped and dropped[0]["result"] is None and
          not resent and dropped2 and all(refused) and sp_ref and land_ok and disarm_ok and not_landed and landed and
          not high_deck and low_deck and not early and confirmed and confirmed[0]["result"] == 0 and timed_out and
          timed_out[0]["result"] is None and abs(t30 - 40.0) < 1e-6 and abs(t5 - 10.0) < 1e-6 and
          seq == [None, 60, 60, 20, 20, 20, "land"] and core not in (n - 1, n - 2) and core2 not in (0, 1) and
          nav_clean)
    report("UA33 U4 review fixes: a stale battery % never judged (+ the notice event), a command on its way dropped at "
           "the battery latch and never re-sent, the allow-list refuses mode / mission / take-off / speed / ARM / "
           "setpoints while landing (LAND / DISARM pass), landed() not on the altitude alone, the deck fallback only "
           "from a low altitude, MISSION_SET_CURRENT confirmed by MISSION_CURRENT, the two-stage landing time, the "
           "latched battery stage, the UAV AI core off the boat vision's cores, the gateway's nav code UAV-free",
           ok, "stale %s no land %s latched %s dropped %s resent %d dropped2 %s refused %s sp %s land %s disarm %s "
               "landed %s/%s deck %s/%s setcur %s/%s/%s descent %.0f/%.0f stage %s core %s/%s nav %s (%s)" % (
                   stale_ev, no_land, latched, dropped, len(resent), dropped2, refused, sp_ref, land_ok, disarm_ok,
                   not_landed, landed, high_deck, low_deck, early, confirmed, timed_out, t30, t5, seq, core, core2,
                   nav_clean, navs))


# ------------------------------------------------------------------------------------------- U5 review fixes -------

def ua34_review_u5():
    """U5 review fixes (review of 8f8441e): MED-1 the pilot lock is exclusive (a take refused while the launch / its
    climb fallback owns the drone; uav_route refused 127 for a flown drone; catchup and the launch's CLEAR never
    start a mission under a pilot); MED-2 old drone data = no motion, the position moved on with the ground speed,
    the altitude floor with the climb rate x the age; MED-3 (in UA30a); MED-4 questions open for the captain's ACCEPT
    refused 126 at his take; MED-5 BATTERY_STATUS refreshes the battery time only with a %; LOWs: batt_stale told when
    armed with already-stale data, give-back after the gateway ended the lock = OK, hover without GPS = ALT_HOLD,
    the per-packet STANDBY end of the captain's control."""
    import common  # noqa: F401
    import proto
    import stations as stm
    from uav import launch, manager, pilot, router_worker as rw, uavcheck
    res = {}
    # --- MED-2: setpoint on old / moving data
    cfg = dict(pilot.DEFAULTS, uav_min_alt_m=10.0, uav_max_alt_m=120.0)
    base = {"lat": 37.829896, "lon": 23.61, "alt": 50.0, "hdg": 0.0, "gs": 10.0, "climb": 0.0}

    class Fence:
        def problem(self, pts):
            return "outside" if any(p[0] > 37.8303 for p in pts) else None
    old = pilot.setpoint({"y": 1.0}, dict(base, alt_age=2.0), cfg)        # (U6 review L3: data_stale_s 1.8 s)
    unknown = pilot.setpoint({"y": 1.0}, dict(base, alt_age=None), cfg)
    fresh = pilot.setpoint({"y": 1.0}, dict(base, alt_age=0.0), cfg, Fence())
    moved = pilot.setpoint({"y": 1.0}, dict(base, alt_age=0.8), cfg, Fence())
    fl0 = pilot.setpoint({"z": -1.0}, dict(base, alt=12.0, alt_age=0.0, climb=-2.0), cfg)[0]["vz"]
    fl1 = pilot.setpoint({"z": -1.0}, dict(base, alt=12.0, alt_age=0.8, climb=-2.0), cfg)[0]["vz"]
    res["med2"] = (old[0] == pilot.ZERO and "old" in old[1][0] and unknown[0] == pilot.ZERO and
                   fresh[0]["vx"] > 9 and moved[0]["vx"] == 0.0 and abs(fl0 - 2.0) < 1e-6 and abs(fl1 - 0.4) < 1e-6)
    # --- the Pilots on a scripted manager + a REAL StationServer (MED-1 take refused, MED-2 via drone_view, MED-4,
    #     LOW give-back / hover / per-packet STANDBY end)
    gw = _PGw()
    gw.aurix = _Aurix()
    gw.storage = None
    gw.uav = type("U", (), {})()
    ss = stm.StationServer(gw, {})
    gw.stations = ss
    mgr = _PilotMgr(gw)
    block = {1: None}
    mgr.pilot_block = lambda n: block.get(n)
    P = gw.uav.pilots = pilot.Pilots(mgr)

    def mk(sid, crew):
        st = stm.Station(sid, "u%d" % sid, "HELM", False, "1.2.3.%d" % sid, "s%d" % sid, _Writer())
        st.crew, st.licence = crew, True
        st.tokens, st.refill = 20.0, time.time()
        ss.stations[sid] = st
        return st
    cap, cop = mk(5, "captain"), mk(6, "copilot")
    gw.telem2["helm"] = cap.id
    gw.mode = 0
    cap.driver_open, cap.driver_seen = True, time.time()

    def last(st, what="pilot"):
        a = [m for m in st.writer.lines if m.get("t") == "uav_ack" and m.get("what") == what]
        return a[-1] if a else {}
    block[1] = "UAV 1 is still climbing out (launching)"
    P.pilot_msg(cop, {"uav": 1, "take": True})
    res["med1_take"] = last(cop).get("code") == 122 and "climbing" in last(cop).get("text", "") and 1 not in P.locks
    block[1] = None
    # MED-4: a question open for the captain BEFORE his take
    answers = []
    ss.ask_local(cop, {}, "vision stage 3", lambda ok, code: answers.append((ok, code)))
    seq = [0]

    def cmd(st, ftype, payload=b""):
        seq[0] += 1
        ss.command(st, {"seq": seq[0], "ts": time.time(), "type": ftype, "p": payload.hex(), "sid": st.session})
    cmd(cop, proto.T_GENERATOR, bytes([1]))
    queued = len(ss.asks)
    n_aur = len(gw.aurix.sent)
    P.pilot_msg(cap, {"uav": 1, "take": True})
    gen_ack = [m for m in cop.writer.lines if m.get("t") == "ack" and m.get("type") == proto.T_GENERATOR]
    res["med4"] = (queued == 2 and answers == [(False, 126)] and not ss.asks and gen_ack and
                   gen_ack[-1]["result"] == 126 and not [t for t, _p in gw.aurix.sent[n_aur:] if t == proto.T_GW_CMD])
    # MED-2 through drone_view: old data -> the deflected stick moves nothing
    n0 = len(mgr.ops)
    P.ctl(cap, {"uav": 1, "seq": 1, "ts": time.time()})                       # centred first
    mgr.drones[1]["alt_age"] = 2.5
    P.ctl(cap, {"uav": 1, "seq": 2, "ts": time.time(), "y": 1.0})
    sps = [o for o in mgr.ops[n0:] if o.get("op") == "ctl"]
    mgr.drones[1]["alt_age"] = 0.3
    P.ctl(cap, {"uav": 1, "seq": 3, "ts": time.time(), "y": 1.0})
    sps2 = [o for o in mgr.ops[n0:] if o.get("op") == "ctl"]
    res["med2_view"] = len(sps) == 2 and sps[-1]["vx"] == 0.0 and sps2[-1]["vx"] > 9
    # LOW: the per-packet STANDBY end (no tick in between)
    gw.mode = 3
    n0 = len(mgr.ops)
    P.ctl(cap, {"uav": 1, "seq": 4, "ts": time.time(), "y": 1.0})
    moving = [o for o in mgr.ops[n0:] if o.get("op") == "ctl" and o.get("vx")]
    ends = [m for m in cap.writer.lines if m.get("t") == "uav_pilot_end"]
    res["standby_pkt"] = 1 not in P.locks and not moving and ends and ends[-1]["why"] == "standby"
    # LOW: give back after the gateway ended the lock = OK (no red 127)
    P.pilot_msg(cap, {"uav": 1, "take": False})
    res["give_back"] = last(cap).get("ok") is True
    # LOW: hover without GPS = ALT_HOLD (BRAKE / LOITER need a position)
    gw.mode = 0
    mgr.drones[1]["gps_fix"] = 1
    n0c = len(mgr.cmds)
    P.hover(1)
    modes = [c[3][1] for c in mgr.cmds[n0c:] if c[2] == allow.CMD_DO_SET_MODE]
    res["hover_nogps"] = modes == [allow.COPTER_ALT_HOLD] and 1 not in P.loiter
    # --- MED-1 in the manager: route refused for a flown drone, pilot_block, catchup skipped
    m = object.__new__(manager.UavManager)
    m.gw, m.enabled, m.n, m.ccfg = _G(), True, 6, dict(uavcheck.DEFAULTS)
    m.pilots = type("PL", (), {"locks": {2: pilot.Lock(9, "anna", time.time())}})()
    io_seq = type("Q", (), {"busy": lambda _s: True, "cur": 3, "phase": "CLIMB"})()
    m.seq = io_seq
    m.flying = {2: {"phase": "catchup", "next_try": 0.0}, 4: {"phase": "launching"}, 5: {"phase": "mission"}}
    m.slot = lambda n: {"gps_fix": 3, "ekf_flags": 0x1F}
    sent = []
    m.cmd = lambda n, via, c, p, cb: sent.append((n, c))
    st = _St(6)
    m.route_msg(st, {"target": 2, "wps": [{"lat": 37.83, "lon": 23.61, "alt": 50}]})
    r = st.sent[-1]
    m.catchup_tick(time.time())
    res["med1_mgr"] = (r.get("code") == pilot.ERR_PILOT and "by hand" in r.get("text", "") and not sent and
                       m.pilot_block(3) and m.pilot_block(4) is None and m.pilot_block(2) and m.pilot_block(5) is None)
    # (U6 review MED-3: "launching" blocks only while the sequence owns that drone - slot 4 is not seq.cur)
    # MED-1 in the launch: CLEAR with a pilot on the drone = no MISSION START
    io = FakeIo(1)
    io.pilot_locked = lambda n: True
    seq_ = launch.Launch(io)
    seq_.prepare([1], 7, 0.0)
    seq_.approved(True, 7, 0.0)
    run_seq(io, seq_, 120.0)
    res["med1_clear"] = (1 in seq_.launched and not [x for x in io.sent if x[2] == "start"] and
                         any("by hand" in t for t in io.notices))
    # --- MED-5 + LOW batt_stale in the router (in this process, a fake link)
    events = []
    old_out = rw.out
    rw.out = lambda obj: events.append(obj)
    try:
        r_ = rw.Router()
        lk = _FakeLink(1, "elrs")
        r_.links[(1, "elrs")] = lk
        r_.expect[1] = 11
        now = rw.now_s()
        d = r_.drone(1)
        d.hb, d.hb_t = {"base_mode": 0x80, "custom_mode": 3}, now
        d.rel_alt, d.vz, d.alt_t = 30.0, 0.0, now
        d.batt_pct, d.batt_t = 80, now - 20.0                    # stale BEFORE the arming was seen
        r_.battery_tick(now)
        told = [e for e in events if e.get("ev") == "batt_stale"]
        lk.parser = mavlink.Parser()
        r_.rx(lk, mavlink.encode("BATTERY_STATUS", {"battery_remaining": -1, "current_consumed": 300}, 1, 11, 1), now)
        t_unk = d.batt_t
        r_.rx(lk, mavlink.encode("BATTERY_STATUS", {"battery_remaining": 55, "current_consumed": 310}, 2, 11, 1), now)
        res["med5"] = (len(told) == 1 and told[0]["stale"] is True and t_unk == now - 20.0 and d.batt_pct == 55 and
                       d.batt_t == now)
    finally:
        rw.out = old_out
    report("UA34 U5 review fixes: MED-1 exclusive pilot lock (take refused during a launch / climb-out, uav_route 127 "
           "for a flown drone, no catchup / CLEAR mission start under a pilot), MED-2 old drone data = hold + position "
           "moved on + the floor with the climb rate, MED-4 questions open for the captain refused 126 at his take, "
           "MED-5 BATTERY_STATUS refreshes only with a %, LOWs (stale told when armed, give-back OK, hover without GPS "
           "= ALT_HOLD, the per-packet STANDBY end)", all(res.values()), "%s" % res)


def watchdog():
    def fire():
        print("\nSTALLED: test_uav.py still running after %d s - killed" % LIMIT_S, flush=True)
        os._exit(3)
    t = threading.Timer(LIMIT_S, fire)
    t.daemon = True
    t.start()


# ------------------------------------------------------------------------------------------- step WA (owner W59) ---

def ua35_w59_speed():
    """Owner W59 (2026-11-04): a launch goes WHEN THE COPILOT SAYS SO - no boat mode / speed limit; above 5 kn a
    NON-blocking notice «ΥΨΗΛΗ ΤΑΧΥΤΗΤΑ – κίνδυνος στην απογείωση»; the other conditions stay."""
    import common  # noqa: F401
    import proto
    import uavlink
    from uav import manager as mgr
    res = {}

    class GW:
        def __init__(self, mode, sog, gps=1):
            self.t = {"mode": mode, "mode_name": "X", "pos_source": gps, "est_lat": 37.8, "est_lon": 23.6,
                      "est_sog": sog, "alarm_bits": 0, "alarm_bits2": 0}
            self.telem2 = {"relay_ok": True, "alarm_bits2": 0}
            self.stations = type("S", (), {"hatch_relay_ok": lambda _s, n: (True, ""),
                                           "helm_station": lambda _s: (_St(1), ""), "captain": lambda _s: None})()

        def telem_fresh(self):
            return self.t
    fast = uavlink.boat_state(GW(proto.MODE_HDG_LOCK, 8.0), 8)
    auto = uavlink.boat_state(GW(proto.MODE_AUTO, 4.0), 8)
    nogps = uavlink.boat_state(GW(proto.MODE_AUTO, 4.0, gps=0), 8)
    res["state"] = (fast["launch_why"] == [] and fast["high_speed"] and auto["launch_why"] == [] and
                    not auto["high_speed"] and any("GPS" in w for w in nogps["launch_why"]))
    tmp = tempfile.mkdtemp(prefix="uavtest_w59_")
    try:
        g = _G(tmp)
        m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
        boat = {"pos": (37.82, 23.60), "launch_why": [], "sog_kn": 8.0}
        m.boat = lambda: boat
        notes = lambda: [n["text"] for n in m.notices if "HIGH SPEED" in n["text"]]
        m.high_speed_tick()
        idle_none = notes() == [] and m.status()["high_speed"]          # not busy: the line, no notice
        m.seq.state = "RUN"
        m.high_speed_tick()
        m.high_speed_tick()
        once = len(notes()) == 1
        boat["sog_kn"] = 4.8                                             # inside the 0.5 kn hysteresis: no new one
        m.high_speed_tick()
        boat["sog_kn"] = 6.0
        m.high_speed_tick()
        hyst = len(notes()) == 1
        boat["sog_kn"] = 3.0
        m.high_speed_tick()
        slow = not m.status()["high_speed"]
        boat["sog_kn"] = 7.0
        m.high_speed_tick()
        again = len(notes()) == 2
        res["notice"] = idle_none and once and hyst and slow and again
        # the prepare at 8 kn: NOT refused; the captain's question carries HIGH SPEED
        m.seq.state = "IDLE"
        m.launch_conditions = lambda slots: []
        m.slot_status = lambda: [{"slot": 1, "present": True}]
        m.routes = {1: [{"lat": 37.83, "lon": 23.6, "alt": 50.0}]}
        asked = []
        g.stations.ask_captain = lambda st, msg, what, cb: asked.append(what)
        boat["sog_kn"] = 8.0
        st = _St(3)
        m.prepare_msg(st, {"t": "uav_launch_prepare"})
        res["prepare"] = (bool(asked) and "HIGH SPEED - take-off risk (8.0 kn)" in asked[0] and
                          m.seq.state == "ASK" and st.sent and st.sent[-1].get("ok"))
        try:
            import i18n
            i18n.set_lang("el", save=False)
            tr = i18n.from_en(asked[0] if asked else "")
            res["greek"] = "ΥΨΗΛΗ ΤΑΧΥΤΗΤΑ – κίνδυνος στην απογείωση" in tr and "ΕΚΤΟΞΕΥΣΗ UAV" in tr
            i18n.set_lang("en", save=False)
        except ImportError as e:
            res["greek"] = "i18n not importable: %s" % e and False          # WA review L4: never a silent pass
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    report("UA35 (W59) a launch at any boat mode / speed: no speed / mode reason, high_speed above 5 kn, ONE "
           "non-blocking HIGH SPEED notice per crossing while a launch is prepared / runs (0.5 kn hysteresis), "
           "gw_status high_speed, a prepare at 8 kn is asked (not refused) with HIGH SPEED in the captain's question",
           all(res.values()), "%s" % res)


# ------------------------------------------------------------------------------------------- U6 review fixes -------

def ua36_review_u6():
    """U6 review fixes (review of f7aed5d): MED-3 + L1-L4."""
    from uav import launch, manager as mgr, pilot, router_worker as rw
    res = {}
    # MED-3: a STOP while the drone climbs out -> launch_ended -> phase "hold": the pilot may take it
    tmp = tempfile.mkdtemp(prefix="uavtest_u6_")
    try:
        g = _G(tmp)
        m = mgr.UavManager({"enabled": True, "slots": [{"usb": "udp:127.0.0.1:1"}] * 6}, g)
        m.boat = lambda: {"pos": (37.82, 23.60), "launch_why": []}
        m.flying[1] = {"phase": "launching", "home": None, "t": 0.0}
        m.seq.state, m.seq.cur, m.seq.armed_here = launch.RUN, 1, False
        m.slot = lambda n: {"landed": False, "usb_present": False}
        blocked_run = m.pilot_block(1)
        m.seq.stop("STOP by test", 0.0)
        res["med3"] = (blocked_run is not None and m.flying[1]["phase"] == "hold" and m.pilot_block(1) is None and
                       not m.seq.busy())
        m.flying[2] = {"phase": "launching"}
        res["med3_old"] = m.pilot_block(2) is None                  # no sequence owns it: never blocked for ever
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    cfg = dict(pilot.DEFAULTS, uav_min_alt_m=10.0, uav_max_alt_m=120.0)

    class Band:                                                 # a 1 m thick no-fly band at 51.0..52.0 m
        def problem(self, pts):
            if any(51.0 <= p[2] <= 52.0 for p in pts):
                return "band"
            return None
    base = {"lat": 37.8, "lon": 23.6, "alt": 50.0, "hdg": 0.0, "gs": 0.0, "climb": 0.0, "alt_age": 0.1}
    sp, why = pilot.setpoint({"z": 1.0}, base, cfg, Band())
    res["l1"] = sp["vz"] == 0.0 and any("no-fly" in w for w in why)

    class Thin:                                                 # WA/U6 review 2 LOW: a 0.5 m band (52.6..53.1 m)
        def problem(self, pts):
            if any(52.6 <= p[2] <= 53.1 for p in pts):
                return "thin band"
            return None
    sp2, why2 = pilot.setpoint({"z": 1.0}, base, cfg, Thin())
    res["l1b"] = sp2["vz"] == 0.0 and any("no-fly" in w for w in why2) and pilot.VERT_STEP_M <= 0.5
    # L2: heading north, flying EAST (course 90): the old position moves east, not north
    moved = {}

    class Spy:
        def problem(self, pts):
            moved.setdefault("p", pts[0])
            return None
    pilot.setpoint({"y": 1.0}, dict(base, gs=10.0, cog=90.0, alt_age=1.0), cfg, Spy())
    p0 = moved.get("p")
    res["l2"] = p0 is not None and p0[1] > 23.6 + 5e-5 and abs(p0[0] - 37.8) < 1e-6
    # L3: 1.5 s old = moves (one lost packet); 2.0 s = holds
    a = pilot.setpoint({"y": 1.0}, dict(base, alt_age=1.5), cfg)[0]
    b = pilot.setpoint({"y": 1.0}, dict(base, alt_age=2.0), cfg)[0]
    res["l3"] = a["vx"] > 0 and b == pilot.ZERO
    # L4: battery_remaining -1 never refreshes the battery time
    d = rw.Drone() if hasattr(rw, "Drone") else None
    r = rw.Router()
    lk = type("L", (), {})()
    lk.slot, lk.kind, lk.info, lk.hbs = 1, "usb", {}, []
    from uav import mavlink as mv
    lk.parser = mv.Parser()
    r.expect[1] = 11
    real = rw.now_s
    try:
        r.rx(lk, mv.encode("SYS_STATUS", {"battery_remaining": 55, "voltage_battery": 15000, "current_battery": 1000},
                           0, 11, 1), 100.0)
        t1 = r.drone(1).batt_t
        r.rx(lk, mv.encode("SYS_STATUS", {"battery_remaining": -1, "voltage_battery": 15000, "current_battery": 1000},
                           1, 11, 1), 110.0)
        t2, pct = r.drone(1).batt_t, r.drone(1).batt_pct
    finally:
        rw.now_s = real
    res["l4"] = t1 == 100.0 and t2 == 100.0 and pct == 55
    report("UA36 U6 review fixes: MED-3 a STOPPED launch lets the airborne drone go (hold, a pilot may take it; "
           "'launching' never blocks without a sequence); L1 a 1 m no-fly band inside a climb stops it; L2 an old "
           "position moved along the course over ground; L3 1.5 s old data still moves, 2 s holds; L4 battery_remaining "
           "-1 never refreshes the battery time", all(res.values()), "%s" % res)


def ua37_helm_forward():
    """WA/U6 review 2 LOW (WA L2): stations.helm_forward = the gateway's own HELM grant while it is newer than TELEM2's
    last HELM change and differs from it (the "B3 race"), else TELEM2's holder; uav_joy_refusal's copilot branch asks
    the captain the command is FORWARDED for (a just-granted captain flying a UAV refuses the copilot's command 126
    even while TELEM2 still shows the old holder)."""
    import common  # noqa: F401
    import proto
    import stations as stm
    gw = _PGw()
    gw.aurix = _Aurix()
    gw.storage = None
    gw.uav = type("U", (), {})()
    gw.uav.pilots = object()                                     # (a magazine exists: uav_joy_refusal looks)
    ss = stm.StationServer(gw, {})
    gw.stations = ss

    def mk(sid, crew):
        st = stm.Station(sid, "u%d" % sid, "HELM", False, "1.2.3.%d" % sid, "s%d" % sid, _Writer())
        st.crew, st.licence = crew, True
        ss.stations[sid] = st
        return st
    old, new, cop = mk(5, "captain"), mk(7, "captain"), mk(6, "copilot")
    gw.telem2["helm"] = old.id
    now = time.time()
    res = {}
    res["no_grant"] = ss.helm_forward() == 5
    ss.helm_grant, ss.helm_grant_t, ss.helm_t2_since = 7, now, now - 5.0       # granted, TELEM2 not yet
    res["grant_newer"] = ss.helm_forward() == 7
    ss.helm_grant = 5
    res["grant_same"] = ss.helm_forward() == 5
    ss.helm_grant, ss.helm_t2_since = 7, now + 0.5                             # TELEM2 changed after the grant
    res["t2_newer"] = ss.helm_forward() == 5
    ss.helm_t2_since = now - 5.0
    real = stm._uav_flying_captain
    stm._uav_flying_captain = lambda _s, st: 2 if st is new else None          # the NEW captain flies UAV 2
    try:
        r_new = ss.uav_joy_refusal(cop, proto.T_SET_MODE, bytes([proto.MODE_HDG_LOCK]))
        ss.helm_grant_t = 0.0                                                  # no grant: TELEM2's old holder
        r_old = ss.uav_joy_refusal(cop, proto.T_SET_MODE, bytes([proto.MODE_HDG_LOCK]))
    finally:
        stm._uav_flying_captain = real
    res["refusal_forwarded"] = r_new is not None and "UAV 2" in r_new and r_old is None
    report("UA37 (WA L2, review 2) stations.helm_forward: the gateway's newer HELM grant, else TELEM2's holder; the "
           "copilot's 126 refusal follows the captain the command is forwarded for", all(res.values()),
           "%s (refusal %r / without the grant %r)" % (res, r_new, r_old))


def main():
    watchdog()
    tests = [ua1_codec, ua2_pymavlink, ua3_allow, ua4_writer, ua5_no_aurix, ua6_check, ua7_registry, ua8_assist,
             ua9_simdrone, ua10_router, ua11_gateway, ua12_osdep, ua13_mission, ua14_checks, ua15_hatch,
             ua16_launch, ua17_router_flight,
             ua18_gateway_launch, ua19_helm_grant, ua20_registry_path, ua21_fixed,
             ua22_battery, ua23_notices_flightlog, ua24_video_worker, ua25_uav_streams, ua26_uav_ai,
             ua27_gateway_video, ua28_review_u3, ua29_review_router, ua30_pilot, ua31_stations_126, ua37_helm_forward,
             ua32_router_ctl, ua33_review_u4, ua34_review_u5, ua35_w59_speed, ua36_review_u6, ua37_pf2u_flying_route_needs_yes, ua38_pf4_attitude_and_camera, ua39_pf4r_pending_races]
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
