"""The academy's UAV magazine SIMULATOR (step U6, CLAUDE.md V "Sim / academy / docs", owner spec H7).

A small simulated boat + magazine (6 slots, the hatch with its OPEN / CLOSED switches, 6 simulated camera drones) that
runs INSIDE the academy process - no network, no process, no gateway, no drone (test_academy AC1-AC3, AC51-AC54). It
drives the gateway's OWN pure rule modules, so a trainee meets exactly the rules of the real magazine:

  gateway/uav/launch.py    the launch state machine (CHECK -> UPLOAD -> HATCH -> GPS -> CMDS -> CLIMB -> CLEAR -> GAP,
                           CLOSING, STOPPED on any warning) - this module is its "io" like the gateway's manager
  gateway/uav/allow.py     the MAVLink ALLOW-list: every simulated command / mission item goes through check_out
                           (ARM only with the hatch OPEN switch, LAND only by the battery rule, never RTL / SmartRTL)
  gateway/uav/uavcheck.py  the route checks (123: altitude / radio range / energy + landing reserve)
  gateway/uav/mission.py   route -> mission (HOME, TAKEOFF slot-staggered, speed cap, waypoints, LOITER_UNLIM)
  gateway/uav/battery.py   the battery minimum = the ONLY landing

The boat's launch conditions are the gateway's (uavlink.boat_state): trusted GPS, no boat warning; else PREPARE
LAUNCH is refused 125 with the reason. Owner W59 (2026-11-04): NO boat mode / speed condition (a launch goes when the
copilot says so); above 5 kn the NON-blocking notice "HIGH SPEED - take-off risk". The captain approves (his YES / NO);
the copilot prepares. Camera / observation drones ONLY: there is no other output in this simulator.

Trainee actions return (ok, code, text); state() = the fields a UAV lesson checks (uavlesson.FIELDS).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GW_DIR = os.path.join(os.path.dirname(HERE), "gateway")
if GW_DIR not in sys.path:
    sys.path.append(GW_DIR)

from uav import allow, battery, launch, mission, uavcheck   # noqa: E402  (pure rule modules: no I/O)

N_SLOTS = 6
SYSID0 = 10                     # simulated sysid = 10 + slot
HATCH_TRAVEL_S = 3.0
GPS_WARM_S = 1.5                # a warm-started GPS under the open hatch (gateway gpsassist, docs 7)
CLIMB_MS = 2.5
CRUISE_MS = 8.0
LAND_MS = 1.0
DRAIN_PCT_S = 0.12              # in flight (accelerated a little for the lesson)
CAPACITY_MAH = 2500.0
ORIGIN = (37.8200, 23.6000)     # the demo's sea area (demo/demo.py origin)
ERR_LAUNCH, ERR_ROUTE, ERR_UAV = 125, uavcheck.ERR_ROUTE, 122
BOAT_MODES = ("STANDBY", "HDG_LOCK", "AUTO", "DP_SPOT")
SCOPE = "camera / observation drones only"


def offset(pos, north_m, east_m):
    """(lat, lon) moved by metres."""
    return (pos[0] + north_m / 111320.0, pos[1] + east_m / (111320.0 * math.cos(math.radians(pos[0]))))


def demo_route(pos=ORIGIN, alt=50.0):
    """The lesson's ready-made observation route: two points 300 / 500 m ahead of the boat at 50 m."""
    a, b = offset(pos, 300.0, 100.0), offset(pos, 500.0, -50.0)
    return [{"lat": a[0], "lon": a[1], "alt": alt}, {"lat": b[0], "lon": b[1], "alt": alt}]


class Drone(object):
    def __init__(self, slot):
        self.slot = slot
        self.sysid = SYSID0 + slot
        self.registered = False
        self.route = None
        self.usb_present = True             # in its slot on the USB cable (the breakaway pulls it at the launch)
        self.armed = False
        self.mode = "STABILIZE"
        self.alt = 0.0
        self.take_alt = None
        self.gps_fix, self.ekf = 1, 0x07
        self.t_sky = None
        self.battery_pct = 95.0
        self.landing = False                # the battery rule asked for LAND (latched)
        self.lost = False                   # landed where it was (over the sea)
        self.pos = ORIGIN
        self.items = None
        self.wp = 0
        self.flown = False

    def view(self):
        return {"usb_present": self.usb_present, "battery_pct": int(self.battery_pct), "gps_fix": self.gps_fix,
                "ekf_flags": self.ekf, "rel_alt": self.alt, "alt_age": 0.1, "landed": self.alt < 0.3,
                "armed": self.armed, "mode": self.mode}


class UavSim(object):
    """The simulated boat + magazine. clock() = seconds; tick() 4 Hz (the gateway's launch tick)."""

    def __init__(self, clock, n=N_SLOTS, cfg=None):
        self.clock = clock
        self.t = clock()
        self.bs = {"mode": "HDG_LOCK", "sog_kn": 5.0, "gps": True, "warning": False, "pos": ORIGIN}
        self.drones = {s: Drone(s) for s in range(1, n + 1)}
        self.hatch = "CLOSED"
        self.hatch_cmd_open = False
        self.hatch_t = self.t
        self.hatch_fault = ""
        self.view_slot = 0
        self.notices = []                   # [(t, text, level, acked)]
        self.denied = []                    # what the allow-list refused (the lesson shows it)
        self.sent = []                      # (t, slot, command, params, ok) every command that went to a drone
        self.relay = []                     # (t, on): the hatch relay = the boat's ONLY UAV action
        self.q = []                         # (due, fn) answers on their way
        self.events = []
        self.launch = launch.Launch(self, dict(cfg or {}))

    # --- the trainee's actions (each = (ok, code, text); code 0 = done) ----------------------------------------------
    def register(self, n):
        d = self.drones.get(n)
        if d is None:
            return False, ERR_UAV, "no such slot"
        if not d.usb_present:
            return False, ERR_UAV, "UAV %d: not on its slot USB cable" % n
        if self.launch.busy():
            return False, ERR_UAV, "not while a launch runs"
        d.registered = True
        return True, 0, "UAV %d registered: the read-only check passed (failsafes, RC ignored, 4 motors only)" % n

    def send_route(self, target, wps=None):
        """target "next" (the top UAV still in the magazine) or a slot; wps None = the lesson's demo route."""
        n = self.next_slot() if target == "next" else target
        d = self.drones.get(n)
        if d is None:
            return False, ERR_UAV, "no UAV in the magazine"
        if not d.registered:
            return False, ERR_UAV, "UAV %d is not registered" % n
        try:
            pts = uavcheck.clean(demo_route(self.bs["pos"]) if wps is None else wps, uavcheck.DEFAULTS["uav_wp_alt_m"])
        except ValueError as e:
            return False, ERR_ROUTE, "UAV route refused: %s" % e
        bad = self._route_check(n, pts)
        if bad:
            return False, bad[0], "UAV route refused: %s" % bad[1]
        d.route = pts
        return True, 0, "route stored for UAV %d (%d points) - uploaded at its launch" % (n, len(pts))

    def set_boat(self, mode, sog_kn=None):
        if mode not in BOAT_MODES:
            return False, ERR_LAUNCH, "unknown mode"
        self.bs["mode"] = mode
        if sog_kn is not None:
            self.bs["sog_kn"] = float(sog_kn)
        elif mode in ("STANDBY", "DP_SPOT"):
            self.bs["target_kn"] = 0.0        # idle + neutral: the boat slows down
        return True, 0, "boat: %s" % mode

    def prepare(self, count=1):
        """PREPARE LAUNCH (the copilot): the drones from the top that have a route. -> the captain's question."""
        if self.launch.busy():
            return False, ERR_LAUNCH, "a launch runs already"
        why = self.launch_why()
        if self.hatch == "FAULT":
            why.append("HATCH FAULT: %s" % self.hatch_fault)
        slots = []
        for s in sorted(self.drones):
            d = self.drones[s]
            if not d.usb_present:
                continue
            if not d.registered or not d.route:
                break
            slots.append(s)
            if len(slots) >= count:
                break
        if not slots:
            why.append("no registered UAV with a route at the top of the magazine")
        if why:
            return False, ERR_LAUNCH, "launch refused: " + "; ".join(why)
        self.launch.prepare(slots, "copilot", self.t)
        if self.high_speed():                       # W59: a notice only, never a refusal
            self.notice("HIGH SPEED - take-off risk (%.1f kn)" % self.bs["sog_kn"], "caution")
        return True, 0, "launch prepared (%s) - the captain answers YES / NO on screen 1" % slots

    def high_speed(self):
        return self.bs["sog_kn"] > launch.DEFAULTS["high_speed_kn"]

    def answer(self, yes):
        """The CAPTAIN's YES / NO (screen 1)."""
        if self.launch.state != launch.ASK:
            return False, ERR_LAUNCH, "no launch question open"
        self.launch.approved(bool(yes), "captain", self.t)
        return True, 0, "the captain said %s" % ("YES" if yes else "NO")

    def stop(self):
        ok = self.launch.stop("STOP pressed", self.t)
        return ok, 0 if ok else ERR_LAUNCH, "launch stopped" if ok else "no launch running"

    def hatch_close(self):
        if self.launch.busy():
            return False, ERR_LAUNCH, "not while a launch runs"
        self.hatch_fault = ""
        self.hatch_cmd(False, lambda ok, why: None)
        return True, 0, "HATCH CLOSE"

    def open_video(self, n):
        d = self.drones.get(n)
        if d is None or d.usb_present or not d.flown:
            return False, ERR_UAV, "UAV %d has no video now (it is not flying)" % n
        self.view_slot = n
        return True, 0, "UAV %d video open" % n

    def close_video(self):
        self.view_slot = 0
        return True, 0, "map"

    def ack_notices(self):
        self.notices = [(t, x, lv, True) for t, x, lv, _a in self.notices]
        return True, 0, "notices acknowledged"

    # --- lesson events ------------------------------------------------------------------------------------------------
    def do(self, action):
        """Setup / at_start events (uavlesson.EVENTS / SETUP)."""
        if action == "boat_moving":
            self.set_boat("HDG_LOCK", 5.0)
        elif action == "magazine_full":
            self.__init__(self.clock, len(self.drones))
        elif action == "battery_low":
            for d in self.drones.values():
                if d.flown and not d.landing and not d.lost:
                    d.battery_pct = min(d.battery_pct, 6.0)
        elif action == "hatch_fault":
            self.hatch, self.hatch_fault = "FAULT", "the OPEN switch did not change in time"
        elif action == "warning":
            self.bs["warning"] = True
        else:
            raise ValueError("unknown UAV lesson action %r" % action)

    # --- the launch sequence's io (gateway/uav/launch.py) -------------------------------------------------------------
    def now(self):
        return self.t

    def launch_why(self):
        b, why = self.bs, []
        if not b["gps"]:
            why.append("no trusted boat GPS")
        if b["warning"]:
            why.append("a boat WARNING alarm")
        return why

    def boat(self):
        return {"launch_why": self.launch_why(), "standby": self.boat_d_mode() == "STANDBY",
                "pos": self.bs["pos"] if self.bs["gps"] else None}

    def boat_d_mode(self):
        return self.bs["mode"]

    def hatch_state(self):
        return self.hatch

    def hatch_why(self):
        return self.hatch_fault

    def hatch_open_s(self):
        return HATCH_TRAVEL_S

    def hatch_commanded_open(self):
        return self.hatch_cmd_open

    def captain_ok(self, sid):
        return True

    def router_ok(self):
        return True

    def slot(self, n):
        return self.drones[n].view()

    def next_slot(self):
        return next((s for s in sorted(self.drones) if self.drones[s].usb_present), None)

    def check_drone(self, n):
        d = self.drones[n]
        if not d.registered:
            return "not registered"
        if not d.usb_present:
            return "not on its slot USB cable"
        if d.armed:
            return "armed in the magazine"
        return None

    def _route_check(self, n, pts):
        d = self.drones[n]
        start = (self.bs["pos"][0], self.bs["pos"][1], 0.0)
        return uavcheck.check(pts, start, self.bs["pos"], {}, {"pct": d.battery_pct, "capacity_mah": CAPACITY_MAH},
                              None, mission.init_alt(n))

    def mission_items(self, n, pos, take_alt):
        d = self.drones[n]
        if not d.route:
            return None, "no route"
        if pos is None:
            return None, "no trusted boat position"
        bad = self._route_check(n, d.route)
        if bad:
            return None, bad[1]
        return mission.build(d.route, pos, take_alt, uavcheck.DEFAULTS["uav_max_speed_ms"]), ""

    def upload(self, n, items, via, cb):
        d = self.drones[n]
        for it in items:
            ok, why = allow.check_out("MISSION_ITEM_INT", dict(it, target_system=d.sysid, target_component=1))
            if not ok:
                self.denied.append((self.t, n, why))
                return self.later(0.2, lambda: cb(False, why))
        d.items = list(items)
        self.later(1.0, lambda: cb(True, ""))

    def mission_loaded(self, n):
        pass

    def ctx(self, d, via):
        return {"port": via, "armed": d.armed, "landed": d.alt < 0.3, "hatch_open": self.hatch == "OPEN",
                "battery_land": d.landing}

    def cmd(self, n, via, command, params, cb):
        """A COMMAND_LONG to drone n - only through the ALLOW-list (allow.check_out), like the gateway's router."""
        d = self.drones[n]
        p = [float(x) for x in (list(params) + [0.0] * 7)[:7]]
        f = dict({"param%d" % (i + 1): v for i, v in enumerate(p)}, command=int(command), target_system=d.sysid,
                 target_component=1)
        ok, why = allow.check_out("COMMAND_LONG", f, self.ctx(d, via))
        self.sent.append((round(self.t, 2), n, int(command), p, ok))
        if not ok:
            self.denied.append((self.t, n, why))
            return self.later(0.2, lambda: cb(4, why))     # MAV_RESULT_DENIED
        self._apply(d, int(command), p)
        self.later(0.3, lambda: cb(0, ""))

    def _apply(self, d, c, p):
        if c == allow.CMD_DO_SET_MODE:
            d.mode = {allow.COPTER_GUIDED: "GUIDED", allow.COPTER_AUTO: "AUTO", allow.COPTER_LOITER: "LOITER",
                      allow.COPTER_ALT_HOLD: "ALT_HOLD", allow.COPTER_BRAKE: "BRAKE"}.get(int(p[1]), d.mode)
        elif c == allow.CMD_COMPONENT_ARM_DISARM:
            d.armed = p[0] == 1
            if not d.armed:
                d.take_alt = None
        elif c == allow.CMD_NAV_TAKEOFF:
            d.take_alt = p[6]
        elif c == allow.CMD_MISSION_START:
            d.mode, d.wp = "AUTO", mission.FIRST_WP
        elif c == allow.CMD_NAV_LAND:
            d.mode = "LAND"

    def armed(self, n):
        self.drones[n].flown = True

    def on_mission(self, n):
        pass

    def hatch_cmd(self, on, cb):
        """The hatch relay (ON = open): the boat's ONLY UAV action, a T_RELAY of the HELM station on the real boat."""
        self.relay.append((round(self.t, 2), bool(on)))
        self.hatch_cmd_open = bool(on)
        self.hatch_t = self.t
        if self.hatch != "FAULT":
            self.hatch = "OPENING" if on else "CLOSING"
        self.later(0.3, lambda: cb(True, ""))

    def notice(self, text, level="info"):
        self.notices.append((round(self.t, 1), text, level, False))
        del self.notices[:-40]

    # --- the world ----------------------------------------------------------------------------------------------------
    def later(self, dt, fn):
        self.q.append((self.t + dt, fn))

    def tick(self):
        now = self.clock()
        dt = max(0.0, min(1.0, now - self.t))
        self.t = now
        due = [x for x in self.q if x[0] <= now]
        self.q = [x for x in self.q if x[0] > now]
        for _t, fn in sorted(due, key=lambda x: x[0]):
            fn()
        self._world(dt)
        self.launch.tick(now)

    def _world(self, dt):
        b = self.bs
        if b["mode"] in ("STANDBY", "DP_SPOT") and b["sog_kn"] > 0.0:
            b["sog_kn"] = max(0.0, b["sog_kn"] - 1.0 * dt)          # idle + neutral: the boat slows
        if self.hatch in ("OPENING", "CLOSING") and self.t - self.hatch_t >= HATCH_TRAVEL_S:
            self.hatch = "OPEN" if self.hatch == "OPENING" else "CLOSED"
        for d in self.drones.values():
            if d.usb_present and self.hatch == "OPEN":
                if d.t_sky is None:
                    d.t_sky = self.t
                if self.t - d.t_sky >= GPS_WARM_S:
                    d.gps_fix, d.ekf = 3, 0x1F
            elif d.usb_present:
                d.t_sky = None
            self._fly(d, dt)

    def _fly(self, d, dt):
        if not d.armed:
            return
        if d.alt > 0.3 and d.usb_present:
            d.usb_present = False                                 # the breakaway USB-C: out of the magazine
        if d.mode == "LAND":
            d.alt = max(0.0, d.alt - LAND_MS * dt)
            if d.alt <= 0.0:
                d.armed, d.lost = False, True
                self.notice("UAV %d landed where it was (battery minimum; over the sea the drone is lost)" % d.slot,
                            "caution")
        elif d.take_alt is not None and d.alt < d.take_alt and d.mode in ("GUIDED", "ALT_HOLD", "AUTO"):
            d.alt = min(d.take_alt, d.alt + CLIMB_MS * dt)
        if d.mode == "AUTO" and d.items and d.wp < len(d.items) and d.alt >= (d.take_alt or 0) - 0.5:
            it = d.items[d.wp]
            tgt = (it["x"] / 1e7, it["y"] / 1e7)
            dist = uavcheck.dist_m(d.pos, tgt)
            step = CRUISE_MS * dt
            if dist <= step or it["command"] == allow.CMD_DO_CHANGE_SPEED:
                if it["command"] != allow.CMD_DO_CHANGE_SPEED:
                    d.pos = tgt
                if it["command"] != allow.CMD_NAV_LOITER_UNLIM:
                    d.wp += 1                                     # LOITER_UNLIM = hover over the last point
            else:
                k = step / dist
                d.pos = (d.pos[0] + (tgt[0] - d.pos[0]) * k, d.pos[1] + (tgt[1] - d.pos[1]) * k)
        if not d.usb_present and d.alt > 0.3:
            d.battery_pct = max(0.0, d.battery_pct - DRAIN_PCT_S * dt)
            j = battery.judge(d.alt, d.battery_pct, CAPACITY_MAH, None)
            if j["state"] == "land" and not d.landing:
                d.landing = True                                  # the ONLY landing (battery.py)
                self.notice("UAV %d LANDING NOW - battery minimum: %s" % (d.slot, j["why"]), "caution")
                self.cmd(d.slot, "elrs", allow.CMD_NAV_LAND, [0] * 7, lambda r, w: None)

    # --- what a lesson checks -----------------------------------------------------------------------------------------
    def state(self):
        ds = self.drones.values()
        ls = self.launch.status(self.t)
        return {
            "boat_mode": self.bs["mode"], "sog_kn": round(self.bs["sog_kn"], 2),
            "registered": sum(1 for d in ds if d.registered),
            "routes": sum(1 for d in ds if d.route and d.usb_present),
            "in_magazine": sum(1 for d in ds if d.usb_present),
            "launch": ls["state"], "phase": ls["phase"] or "", "why": ls["why"] or "",
            "launched": len(ls["launched"]),
            "flying": sum(1 for d in ds if d.armed and d.alt > 0.3),
            "hatch": self.hatch, "view": self.view_slot,
            "landing": sum(1 for d in ds if d.landing),
            "lost": sum(1 for d in ds if d.lost),
            "notices_open": sum(1 for n in self.notices if not n[3]),
            "denied": len(self.denied), "ready": True, "high_speed": self.high_speed(),
        }
