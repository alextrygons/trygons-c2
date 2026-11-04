"""Route -> ArduPilot mission (CLAUDE.md V "Routes", docs/uav_design.md 5.1, owner spec H2; step U3).

A station's UAV route = [{"lat", "lon", "alt"}] (alt = metres above the launch point, default uav_wp_alt_m 50, one
per waypoint). The mission (MISSION_ITEM_INT fields, frame 3 = GLOBAL_RELATIVE_ALT: relative to the launch point =
the drone's home = the boat's position at launch):

  seq 0   NAV_WAYPOINT at the launch point, alt 0        = ArduPilot's HOME slot (the drone overwrites it; never
                                                           flown: the mission is always started at seq 1 or later)
  seq 1   NAV_TAKEOFF to the slot's initial altitude       (slot-staggered: 30 + 5 m x (slot - 1), docs 5.3)
  seq 2   DO_CHANGE_SPEED (ground speed) capped at uav_max_speed_ms
  seq 3.. NAV_WAYPOINT per route point with ITS altitude
  last    NAV_LOITER_UNLIM over the last point = hover until a new route or the battery minimum (NEVER a landing:
          the owner's rule "no forced landing except the battery", docs 6.1)

Only the four allow-listed mission commands are ever built (allow.MISSION_COMMANDS: the router refuses anything else
anyway). In flight a new mission is started at FLY_FROM (2 = the speed item: no second take-off).
Pure functions (no I/O): tests UA13.
"""
import math

from . import allow

HOME, TAKEOFF, SPEED = 0, 1, 2
FIRST_WP = 3
START_LAUNCH = 1          # MISSION_START param1 after a launch (the take-off item)
FLY_FROM = SPEED          # ... for a mission changed in flight
MAX_WPS = 20
F_REL = 3                 # MAV_FRAME_GLOBAL_RELATIVE_ALT
F_MISSION = 2             # MAV_FRAME_MISSION (DO_ commands)


def init_alt(slot, base_m=30.0, step_m=5.0):
    """The slot's initial (take-off) altitude: slot-staggered so two drones never climb into each other."""
    return float(base_m) + float(step_m) * (int(slot) - 1)


def _item(seq, cmd, frame, lat=0.0, lon=0.0, alt=0.0, p1=0.0, p2=0.0, p3=0.0, p4=0.0):
    return {"seq": seq, "command": cmd, "frame": frame, "x": int(round(lat * 1e7)), "y": int(round(lon * 1e7)),
            "z": float(alt), "param1": float(p1), "param2": float(p2), "param3": float(p3), "param4": float(p4),
            "current": 0, "autocontinue": 1, "mission_type": 0}


def build(wps, home, take_alt, speed_ms):
    """wps [{"lat", "lon", "alt"}], home (lat, lon) = the boat at launch -> [MISSION_ITEM_INT field dicts]."""
    if not wps:
        raise ValueError("a mission needs at least one waypoint")
    if len(wps) > MAX_WPS:
        raise ValueError("at most %d waypoints" % MAX_WPS)
    hl, ho = float(home[0]), float(home[1])
    items = [_item(HOME, allow.CMD_NAV_WAYPOINT, F_REL, hl, ho, 0.0),
             _item(TAKEOFF, allow.CMD_NAV_TAKEOFF, F_REL, hl, ho, take_alt),
             _item(SPEED, allow.CMD_DO_CHANGE_SPEED, F_MISSION, p1=1.0, p2=speed_ms, p3=-1.0)]
    for w in wps:
        items.append(_item(len(items), allow.CMD_NAV_WAYPOINT, F_REL, w["lat"], w["lon"], w["alt"]))
    last = wps[-1]
    items.append(_item(len(items), allow.CMD_NAV_LOITER_UNLIM, F_REL, last["lat"], last["lon"], last["alt"]))
    return items


def same(sent, got):
    """The read-back (got: items 0..n-1 the drone returned, None = missing) equals the upload? -> (ok, why).
    seq 0 = the drone's own home: not compared (ArduPilot replaces it)."""
    if len(got) != len(sent):
        return False, "read-back: %d items, uploaded %d" % (len(got), len(sent))
    for a, b in zip(sent[1:], got[1:]):
        if b is None:
            return False, "read-back: item %d missing" % a["seq"]
        if int(b.get("command", -1)) != a["command"] or int(b.get("frame", -1)) != a["frame"]:
            return False, "read-back: item %d is command %s frame %s (uploaded %d / %d)" % (
                a["seq"], b.get("command"), b.get("frame"), a["command"], a["frame"])
        if int(b.get("x", 0)) != a["x"] or int(b.get("y", 0)) != a["y"]:
            return False, "read-back: item %d position differs" % a["seq"]
        if not math.isfinite(float(b.get("z", 0.0))) or abs(float(b.get("z", 0.0)) - a["z"]) > 0.01:
            return False, "read-back: item %d altitude differs" % a["seq"]
        if a["command"] == allow.CMD_DO_CHANGE_SPEED and abs(float(b.get("param2", 0.0)) - a["param2"]) > 0.01:
            return False, "read-back: the speed differs"
    return True, ""
