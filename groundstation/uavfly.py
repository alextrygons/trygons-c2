"""Flying a UAV by hand with this station's shore joystick (CLAUDE.md V "Manual control", docs/uav_design.md 6.3,
step U5). REQUESTS ONLY: the gateway (gateway/uav/pilot.py) holds the pilot lock, turns the stick into a clipped
velocity setpoint and checks every rule again.

  «ΧΕΙΡΙΣΜΟΣ UAV n»       take()  -> {"t": "uav_pilot", "uav": n, "take": true}
  «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ»  give()  -> {"t": "uav_pilot", "uav": n, "take": false}: the UAV hovers, the joystick is
                                     back on the BOAT (never "fly back": one-way drones)
  while flying            source() = the 10 Hz {"t": "uav_ctl", "uav", "x", "y", "z", "r"} (UDP, signed like rjoy):
                          stick X / Y = direction + horizontal speed, the TWCS lever = climb / descend (its neutral zone
                          45-55 % = hold the altitude; a station without a lever: the altitude is held), the twist =
                          yaw; let go = zero = the drone holds.
Meanwhile this station sends NO rjoy to the boat (app.rjoy_source) and the 13p YES / NO selector ignores the stick
(questions are answered with the MOUSE only: app.shore_answer_tick). The CAPTAIN (his joystick drives the boat) flies
only with the boat in STANDBY: the boat leaving STANDBY gives the drone back at once (the gateway does it too).
Owner W61 (2026-11-04): the captain may take a drone over from a copilot at once - the copilot's station gets
uav_pilot_end why "captain" = «Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟΝ ΕΛΕΓΧΟ».
"""
import math

import proto

LEVER_NEUTRAL = (0.45, 0.55)        # the TWCS lever's neutral zone = hold the altitude (as the boat's lever)
TWIST = {"dev": 0, "axis": "R", "invert": False}


def _finite(v):
    try:
        return math.isfinite(v)
    except TypeError:
        return False


def lever_climb(lever):
    """The lever 0..1 -> climb -1..1 with the neutral zone (pure: tests UV12)."""
    lo, hi = LEVER_NEUTRAL
    if lever is None or not _finite(lever):
        return 0.0
    lever = max(0.0, min(1.0, float(lever)))
    if lever < lo:
        return -(lo - lever) / lo
    if lever > hi:
        return (lever - hi) / (1.0 - hi)
    return 0.0


def stick(js, cfg):
    """This station's joystick -> {"x", "y", "z", "r"} (-1..1) or None (a device missing / a non-finite value)."""
    s, t = cfg["steer"], cfg["thr"]
    a = js.read(s["dev"])
    if not a.get("ok"):
        return None
    x = a["axes"].get(s["axis"], 0.0) * (-1 if s.get("invert") else 1)
    lever = bool(t.get("lever", True))
    if lever:
        sy = cfg.get("stick_y") or {"dev": s["dev"], "axis": "Y", "invert": True}
        c = a if sy.get("dev") == s["dev"] else js.read(sy["dev"])
        b = js.read(t["dev"])
        if not (c.get("ok") and b.get("ok")):
            return None
        y = c["axes"].get(sy.get("axis", "Y"), 0.0) * (-1 if sy.get("invert", True) else 1)
        v = b["axes"].get(t["axis"], 0.0) * (-1 if t.get("invert") else 1)
        if not _finite(v):
            return None
        z = lever_climb((v + 1.0) / 2.0)
    else:
        b = js.read(t["dev"])
        if not b.get("ok"):
            return None
        y = b["axes"].get(t["axis"], 0.0) * (-1 if t.get("invert") else 1)
        z = 0.0                                     # no lever = the altitude is held
    tw = cfg.get("twist") or TWIST
    d = a if tw.get("dev") == s["dev"] else js.read(tw["dev"])
    r = (d["axes"].get(tw.get("axis", "R"), 0.0) * (-1 if tw.get("invert") else 1)) if d.get("ok") else 0.0
    if not all(_finite(v) for v in (x, y, z, r)):
        return None
    return {"x": max(-1.0, min(1.0, x)), "y": max(-1.0, min(1.0, y)), "z": max(-1.0, min(1.0, z)),
            "r": max(-1.0, min(1.0, r))}


class UavFly:
    """One per station: which UAV this station's joystick flies (None = the boat's), the gateway's answers."""

    def __init__(self):
        self.slot = None            # the UAV flown now (the gateway said yes)
        self.asked = None           # a take on its way
        self.last_end = None        # (uav, why, text) of the last end the gateway sent
        self.captain = False        # the gateway's view: this station is the captain (STANDBY rule)

    def active(self):
        """The joystick is (or is about to be) on a UAV: no rjoy to the boat, the stick answers no question."""
        return self.slot is not None or self.asked is not None

    def take(self, n):
        self.asked = int(n)
        return {"t": "uav_pilot", "uav": int(n), "take": True}

    def give(self):
        n = self.slot if self.slot is not None else self.asked
        self.slot = self.asked = None
        return None if n is None else {"t": "uav_pilot", "uav": int(n), "take": False}

    def on_ack(self, d):
        """uav_ack what "pilot" -> True when it changed the state."""
        if d.get("what") != "pilot":
            return False
        if d.get("ok") and d.get("take"):
            self.slot, self.asked = d.get("uav"), None
            self.captain = bool(d.get("captain"))
        elif d.get("ok"):
            self.slot = self.asked = None
        elif d.get("uav") in (self.asked, self.slot):
            self.slot = self.asked = None
        return True

    def on_end(self, d):
        """uav_pilot_end (the captain took over, the boat left STANDBY, stale data ...)."""
        if d.get("uav") in (self.slot, self.asked):
            self.slot = self.asked = None
        self.last_end = (d.get("uav"), d.get("why"), d.get("text"))

    def must_end(self, mode, captain_like):
        """The station's own check (the gateway does it too): the captain's joystick back to the boat at once when
        the boat is not in STANDBY."""
        return self.active() and captain_like and mode is not None and mode != proto.MODE_STANDBY

    def source(self, js, cfg):
        """The next uav_ctl packet (10 Hz) or None (not flying / the joystick can't be read: the gateway holds)."""
        if self.slot is None or js is None:
            return None
        v = stick(js, cfg)
        if v is None:
            return None
        return dict(v, t="uav_ctl", uav=self.slot)
