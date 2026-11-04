"""The MAVLink ALLOW-list = the code-level scope guard of the UAV magazine (CLAUDE.md V, docs/uav_design.md 4.3).

Camera / observation drones ONLY: only the messages and commands listed here pass the router (gateway/uav/
router_worker.py), in either direction; everything else is DROPPED and COUNTED ("denied"). The list holds NO
actuator / servo / relay / gripper / winch / parachute / mount / camera-trigger command, NO PARAM_SET (the gateway
never writes a parameter), NO RC override, NO RTL / SmartRTL. UA3 checks every other message id and command id is
refused; UA4 greps that no other module builds MAVLink frames.

check_out(name, fields, ctx) -> (ok, why) before the router sends a frame to a drone; ctx = what the router knows:
  port      "usb" (the slot's USB cable) / "elrs" (the radio link)
  armed     the drone's last HEARTBEAT said armed
  landed    on the ground / in its slot (not flying)
  hatch_open  the hatch OPEN switch (U3; never ARM without it - storm I81)
  battery_land  the battery rule of docs/uav_design.md 6.2 asks for LAND (the ONLY landing - storm I82)
check_in(name) -> (ok, why) for a frame from a drone.
"""
import math

# --- from the drones ------------------------------------------------------------------------------------------------
FROM_DRONE = frozenset((
    "HEARTBEAT", "SYS_STATUS", "BATTERY_STATUS", "GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD", "GPS_RAW_INT",
    "EKF_STATUS_REPORT", "STATUSTEXT", "HOME_POSITION", "MISSION_CURRENT", "MISSION_ITEM_REACHED",
    "MISSION_REQUEST_INT", "MISSION_ACK", "COMMAND_ACK", "PARAM_VALUE", "AUTOPILOT_VERSION", "TIMESYNC", "SYSTEM_TIME",
    "RADIO_STATUS",
    "MISSION_COUNT", "MISSION_ITEM_INT",          # the mission READ-BACK (U3: count + items compared with the upload)
))

# --- to the drones --------------------------------------------------------------------------------------------------
TO_DRONE = frozenset((
    "HEARTBEAT", "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_CLEAR_ALL", "MISSION_SET_CURRENT",
    "MISSION_REQUEST_LIST", "MISSION_REQUEST_INT", "MISSION_ACK",     # read-back (U1 review LOW)
    "PARAM_REQUEST_READ", "PARAM_REQUEST_LIST",                     # READ ONLY - never PARAM_SET
    "SET_POSITION_TARGET_LOCAL_NED", "TIMESYNC", "SYSTEM_TIME", "GPS_RTCM_DATA", "GPS_INJECT_DATA",
    "COMMAND_LONG", "COMMAND_INT",
))

# MAV_CMD ids
CMD_NAV_WAYPOINT = 16
CMD_NAV_LOITER_UNLIM = 17
CMD_NAV_LAND = 21
CMD_NAV_TAKEOFF = 22
CMD_DO_SET_MODE = 176
CMD_DO_CHANGE_SPEED = 178
CMD_MISSION_START = 300
CMD_COMPONENT_ARM_DISARM = 400
CMD_SET_MESSAGE_INTERVAL = 511
CMD_REQUEST_MESSAGE = 512
COMMANDS = frozenset((CMD_COMPONENT_ARM_DISARM, CMD_DO_SET_MODE, CMD_NAV_TAKEOFF, CMD_MISSION_START,
                      CMD_DO_CHANGE_SPEED, CMD_NAV_LAND, CMD_SET_MESSAGE_INTERVAL, CMD_REQUEST_MESSAGE))
# the ONLY commands a mission item may hold (U1 review MED-1: never DO_SET_SERVO / DO_SET_RELAY / DO_GRIPPER /
# DO_WINCH / DO_PARACHUTE / DO_MOUNT_* / DO_DIGICAM_* / DO_REPEAT_* / scripting ...)
MISSION_COMMANDS = frozenset((CMD_NAV_TAKEOFF, CMD_NAV_WAYPOINT, CMD_DO_CHANGE_SPEED, CMD_NAV_LOITER_UNLIM))
MISSION_FRAMES = frozenset((3, 6))                    # GLOBAL_RELATIVE_ALT / _INT (relative to the launch point)
MISSION_FRAMES_DO = frozenset((2,))                   # MAV_FRAME_MISSION for DO_CHANGE_SPEED
# ArduCopter flight modes (custom_mode) the gateway may choose - never RTL 6 / SMART_RTL 21 / LAND 9 / anything else
COPTER_ALT_HOLD, COPTER_AUTO, COPTER_GUIDED, COPTER_LOITER, COPTER_RTL, COPTER_LAND, COPTER_BRAKE, COPTER_SMART_RTL = (
    2, 3, 4, 5, 6, 9, 17, 21)
MODES = frozenset((COPTER_GUIDED, COPTER_AUTO, COPTER_LOITER, COPTER_ALT_HOLD, COPTER_BRAKE))
FORCE_MAGIC = 21196                                    # ARM_DISARM param2: force (skips the checks) - always refused
MAX_SPEED_MS = 20.0                                    # hard ceiling of DO_CHANGE_SPEED (the manager caps lower)
# SET_POSITION_TARGET_LOCAL_NED: velocity + yaw rate only, body frame
FRAMES_BODY = frozenset((8, 9))                        # MAV_FRAME_BODY_NED / BODY_OFFSET_NED
MASK_MUST_SET = 0x0007 | 0x01C0 | 0x0400               # position, acceleration and yaw ignored
MASK_MUST_CLEAR = 0x0038 | 0x0200                      # velocity used, never "force"
GPS_INJECT = frozenset(("GPS_RTCM_DATA", "GPS_INJECT_DATA"))


def check_in(name):
    if name in FROM_DRONE:
        return True, ""
    return False, "%s from a drone is not on the allow-list" % name


def _finite(*vals):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def _command(cmd, p, ctx):
    if cmd not in COMMANDS:
        return False, "command %d is not on the allow-list" % cmd
    if cmd == CMD_COMPONENT_ARM_DISARM:
        if int(round(p[1])) == FORCE_MAGIC or p[1] != 0:
            return False, "force arm / disarm is never sent"
        if p[0] == 1:
            if not ctx.get("hatch_open"):
                return False, "ARM only with the hatch OPEN switch"
            return True, ""
        if p[0] == 0:
            if not ctx.get("landed"):
                return False, "DISARM only on the ground / in the slot"
            return True, ""
        return False, "bad ARM / DISARM value"
    if cmd == CMD_DO_SET_MODE:
        if int(p[1]) not in MODES or int(p[0]) & 1 != 1:
            return False, "flight mode %s is not allowed (never RTL / SmartRTL / LAND)" % int(p[1])
        return True, ""
    if cmd == CMD_NAV_LAND:
        if not ctx.get("battery_land"):
            return False, "LAND only by the battery rule"
        return True, ""
    if cmd == CMD_DO_CHANGE_SPEED:
        if not (0.0 < p[1] <= MAX_SPEED_MS):
            return False, "speed out of range"
        return True, ""
    if cmd in (CMD_SET_MESSAGE_INTERVAL, CMD_REQUEST_MESSAGE):
        from . import mavlink
        name = mavlink.BY_ID.get(int(p[0]))
        if name not in FROM_DRONE:
            return False, "message %d is not on the allow-list" % int(p[0])
        return True, ""
    return True, ""                                   # TAKEOFF, MISSION_START


def check_out(name, fields, ctx=None):
    """(ok, why) for a frame the router would send to a drone."""
    ctx = ctx or {}
    if name not in TO_DRONE:
        return False, "%s to a drone is not on the allow-list" % name
    f = fields
    if name == "COMMAND_LONG":
        p = [f.get("param%d" % i, 0.0) for i in range(1, 8)]
        if not _finite(*p):
            return False, "a non-finite command parameter"
        return _command(int(f.get("command", 0)), p, ctx)
    if name == "COMMAND_INT":
        p = [f.get("param%d" % i, 0.0) for i in range(1, 5)] + [f.get("x", 0), f.get("y", 0), f.get("z", 0.0)]
        if not _finite(*p):
            return False, "a non-finite command parameter"
        return _command(int(f.get("command", 0)), p, ctx)
    if name == "MISSION_ITEM_INT":
        cmd = int(f.get("command", 0))
        if cmd not in MISSION_COMMANDS:
            return False, "mission command %d is not on the allow-list" % cmd
        frames = MISSION_FRAMES_DO if cmd == CMD_DO_CHANGE_SPEED else MISSION_FRAMES
        if int(f.get("frame", 0)) not in frames:
            return False, "mission frame %d is not allowed" % int(f.get("frame", 0))
        if not _finite(f.get("param1", 0.0), f.get("param2", 0.0), f.get("param3", 0.0), f.get("param4", 0.0),
                       f.get("z", 0.0)):
            return False, "a non-finite mission parameter"
        if cmd == CMD_DO_CHANGE_SPEED and not (0.0 < f.get("param2", 0.0) <= MAX_SPEED_MS):
            return False, "speed out of range"
        return True, ""
    if name == "SET_POSITION_TARGET_LOCAL_NED":
        m = int(f.get("type_mask", 0))
        if int(f.get("coordinate_frame", 0)) not in FRAMES_BODY:
            return False, "setpoints only in the body frame"
        if m & MASK_MUST_SET != MASK_MUST_SET or m & MASK_MUST_CLEAR:
            return False, "setpoints: velocity + yaw rate only"
        if not _finite(f.get("vx", 0.0), f.get("vy", 0.0), f.get("vz", 0.0), f.get("yaw_rate", 0.0)):
            return False, "a non-finite setpoint"
        return True, ""
    if name in GPS_INJECT:
        if ctx.get("port") != "usb":
            return False, "GPS injection only on the slot's USB cable"
        if ctx.get("armed"):
            return False, "GPS injection only before ARM"
        return True, ""
    return True, ""
