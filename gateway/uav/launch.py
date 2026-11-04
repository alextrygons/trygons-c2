"""The launch sequence of the UAV magazine (CLAUDE.md V, docs/uav_design.md 5.3 / 5.6 / 7, owner spec H3 / H4; U3).

A state machine, one step per tick (4 Hz from the event loop), EVERY wait timed, nothing blocking: requests go to
the router process (mission upload, commands) or through uavlink (the hatch relay) and come back as callbacks.

  IDLE -> ASK (prepared: the captain's YES / NO on screen 1; a single operator answers his own) -> RUN
  RUN, per drone (the TOP slot still in the magazine, strictly in series):
     CHECK   the slot is the top one (next_slot), registered + checked + the same drone, on its USB cable, not armed,
             battery, sensors, its route still passes uavcheck from the boat's position now
     UPLOAD  the mission over the slot's USB + its read-back (router: count + every item compared)
     HATCH   (first drone) relay ON -> the OPEN switch (hatch.py; its travel time = FAULT)
     GPS     a 3D fix + EKF position within gps_wait_s; else ONLY in STANDBY the fallback "climb, then catch up"
             (ALT_HOLD + TAKEOFF param3 = 1 to fallback_alt; the manager switches it to the mission once GPS + EKF
             are good - it hovers until then, never lands); in DP: stop
     CMDS    GUIDED -> ARM (the router's allow-list refuses ARM unless the hatch OPEN switch) -> TAKEOFF to the
             slot's initial altitude (30 + 5 m x (slot - 1)), each with its COMMAND_ACK
     CLIMB   relative altitude >= clear_alt within clear_s; NO climb within no_climb_s = DISARM at once (it is on the
             ground: a motor / prop fault in the hatch), stop, the hatch STAYS OPEN + notice "did not leave the
             hatch - check before closing" (the close is then the captain's own HATCH CLOSE)
     CLEAR   the slot's USB cable gone (the breakaway, W58) within usb_gone_s, then MISSION_START (the fallback
             drone: handed to the manager's catch-up)
     GAP     the next drone not before gap_s after this one's ARM (and only once this one is clear)
  CLOSING   after the last drone: relay OFF -> the CLOSED switch -> DONE
  STOPPED   ANY warning (a check, the hatch FAULT, the boat leaving STANDBY / DP SPOT-JOG or speeding up, stale boat
             telemetry, a boat WARNING alarm, the relay board silent, HELM lost, the captain's station lost, a
             station's STOP): no further drone, no ARM; drones in the air go on; an armed drone still on the ground
             is DISARMED; the hatch is switched by NOTHING (W72 default: the captain's own HATCH CLOSE)
Nothing here sends to the AURIX but the hatch relay through io.hatch_relay (uavlink -> stations.py).
"""
from . import allow, mission

IDLE, ASK, RUN, CLOSING, DONE, STOPPED = "IDLE", "ASK", "RUN", "CLOSING", "DONE", "STOPPED"
EKF_POS = 0x08 | 0x10                     # EKF_STATUS_REPORT: horizontal position relative + absolute

DEFAULTS = {
    "launch_max_kn": 2.0,                 # W59
    "gps_wait_s": 5.0,                    # W59
    "fallback_alt_m": 15.0,               # W59
    "clear_alt_m": 10.0,                  # W59
    "clear_s": 30.0,
    "no_climb_s": 6.0,
    "usb_gone_s": 6.0,
    "gap_s": 15.0,                        # W59
    "min_batt_pct": 60,
    "init_alt_base_m": 30.0,              # W59
    "init_alt_step_m": 5.0,
    "upload_s": 45.0,
    "ask_s": 20.0,                        # the captain's answer (the station's own 10 s question timeout is shorter)
}


class Launch:
    def __init__(self, io, cfg=None):
        self.io = io
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.state, self.why = IDLE, ""
        self.gen = 0
        self.slots = []                   # the slots of this sequence, top first
        self.launched = []
        self.cur = None                   # the slot being launched
        self.phase = ""
        self.t_phase = 0.0
        self.by = None                    # who prepared (station id)
        self.captain = None               # the station that approved (its loss stops the sequence)
        self.wait = None                  # a request out: "upload" / "cmd" / "relay"
        self.cmdq = []
        self.t_arm = None
        self.t_clear = None
        self.fallback = False
        self.armed_here = False           # the current drone was armed by this sequence (DISARM on a stop)
        self.keep_open = False            # a drone did not leave the hatch: the captain closes it himself
        self.history = []

    # --- control --------------------------------------------------------------------------------------------------
    def busy(self):
        return self.state in (ASK, RUN, CLOSING)

    def prepare(self, slots, by, now):
        self.gen += 1
        self.state, self.why, self.slots, self.launched = ASK, "", list(slots), []
        self.by, self.captain, self.cur, self.phase, self.t_phase = by, None, None, "", now
        self.keep_open = False
        self.log(now, "prepared slots %s" % self.slots)

    def approved(self, ok, captain, now, why=""):
        if self.state != ASK:
            return
        if not ok:
            self.state, self.why = STOPPED, why or "the captain said NO"
            self.log(now, self.why)
            return
        self.state, self.captain = RUN, captain
        self.log(now, "approved by station %s" % captain)
        self._next(now)

    def stop(self, why, now):
        """Any warning / a station's STOP: no further drone, no ARM; flying drones go on."""
        if not self.busy():
            return False
        self.gen += 1                      # every answer still on its way is ignored
        was, self.state, self.why = self.state, STOPPED, why
        self.wait, self.cmdq = None, []
        if self.cur is not None and self.armed_here:
            d = self.io.slot(self.cur)
            if d.get("landed") or d.get("usb_present"):
                self.io.cmd(self.cur, "auto", allow.CMD_COMPONENT_ARM_DISARM, [0, 0], lambda r, w: None)
                self.io.notice("UAV %d disarmed in the hatch (the launch stopped)" % self.cur, "caution")
        self.log(now, "STOPPED (%s): %s" % (was, why))
        self.io.notice("UAV launch stopped: %s" % why, "caution")
        return True

    def log(self, now, text):
        self.history.append((round(now, 1), text))
        del self.history[:-30]

    # --- the per-tick step ----------------------------------------------------------------------------------------
    def tick(self, now):
        if self.state == ASK and now - self.t_phase > float(self.cfg["ask_s"]):
            self.state, self.why = STOPPED, "no answer from the captain"
            self.log(now, self.why)
            return
        if self.state not in (RUN, CLOSING):
            return
        boat = self.io.boat()
        hs = self.io.hatch_state()
        if hs == "FAULT":
            if self.state == RUN:
                return self.stop("HATCH FAULT: %s" % self.io.hatch_why(), now)
            self.state, self.why = STOPPED, "HATCH FAULT while closing: %s" % self.io.hatch_why()
            self.io.notice(self.why, "caution")
            return
        if self.state == RUN:
            why = list(boat.get("launch_why") or [])
            if self.captain is not None and not self.io.captain_ok(self.captain):
                why.append("the captain's station is gone")
            if why:
                return self.stop(why[0], now)
            return self._run(now, boat, hs)
        # CLOSING
        if self.wait is None and self.phase == "close":
            if hs == "CLOSED":
                self.state, self.why = DONE, ""
                self.log(now, "hatch closed - done")
                self.io.notice("UAV launch done: %d launched, the hatch closed" % len(self.launched), "info")

    def _phase(self, name, now):
        self.phase, self.t_phase = name, now

    def _next(self, now):
        left = [s for s in self.slots if s not in self.launched]
        if not left:
            return self._close(now)
        self.cur = left[0]
        self.fallback = self.armed_here = False
        self.t_arm = self.t_clear = None
        self._phase("CHECK", now)

    def _close(self, now):
        self.cur, self.state = None, CLOSING
        self._phase("close", now)
        g = self.gen
        self.wait = "relay"

        def done(ok, why):
            if g != self.gen:
                return
            self.wait = None
            if not ok:
                self.state, self.why = STOPPED, "HATCH NOT CLOSED: %s - take HELM and close it" % why
                self.io.notice(self.why, "caution")
        self.io.hatch_cmd(False, done)

    def _fail(self, why, now):
        return self.stop(why, now)

    def _cb(self):
        g = self.gen

        def wrap(fn):
            def inner(*a):
                if g == self.gen:
                    fn(*a)
            return inner
        return wrap

    def _run(self, now, boat, hs):
        c = self.cfg
        n = self.cur
        if self.wait is not None:
            if self.wait == "upload" and now - self.t_phase > float(c["upload_s"]):
                return self._fail("UAV %d: mission upload timed out" % n, now)
            return
        d = self.io.slot(n)
        ph = self.phase
        if ph == "CHECK":
            if self.io.next_slot() != n:
                return self._fail("slot mismatch: the top slot is %s, the sequence wants %d" % (
                    self.io.next_slot(), n), now)
            why = self.io.check_drone(n)
            if why:
                return self._fail("UAV %d: %s" % (n, why), now)
            if (d.get("battery_pct") or 0) < int(c["min_batt_pct"]):
                return self._fail("UAV %d: battery %s %% (at least %d %%)" % (n, d.get("battery_pct"),
                                                                                 int(c["min_batt_pct"])), now)
            items, why = self.io.mission_items(n, boat.get("pos"),
                                               mission.init_alt(n, c["init_alt_base_m"], c["init_alt_step_m"]))
            if items is None:
                return self._fail("UAV %d: %s" % (n, why), now)
            self._phase("UPLOAD", now)
            self.wait = "upload"
            wrap = self._cb()

            def up(ok, why):
                self.wait = None
                if not ok:
                    return self._fail("UAV %d: mission upload: %s" % (n, why), now)
                self.io.mission_loaded(n)
                self._phase("HATCH", self.io.now())
            self.io.upload(n, items, "usb", wrap(up))
            return
        if ph == "HATCH":
            if hs == "OPEN":
                self._phase("GPS", now)
                return
            if not self.io.hatch_commanded_open():
                self.wait = "relay"
                wrap = self._cb()

                def rel(ok, why):
                    self.wait = None
                    if not ok:
                        self._fail("HATCH NOT OPENED: %s" % why, self.io.now())
                self.io.hatch_cmd(True, wrap(rel))
            elif now - self.t_phase > self.io.hatch_open_s() + 5.0:
                return self._fail("the hatch did not open", now)
            return
        if ph == "GPS":
            if hs != "OPEN":
                return self._fail("the hatch is not OPEN (%s)" % hs, now)
            if (d.get("gps_fix") or 0) >= 3 and ((d.get("ekf_flags") or 0) & EKF_POS) == EKF_POS:
                alt = mission.init_alt(n, c["init_alt_base_m"], c["init_alt_step_m"])
                self.cmdq = [("mode GUIDED", allow.CMD_DO_SET_MODE, [1, allow.COPTER_GUIDED]),
                             ("ARM", allow.CMD_COMPONENT_ARM_DISARM, [1, 0]),
                             ("TAKEOFF", allow.CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, alt])]
                self._phase("CMDS", now)
            elif now - self.t_phase > float(c["gps_wait_s"]):
                if not boat.get("standby"):
                    return self._fail("UAV %d: no GPS fix in %.0f s (the climb fallback only in STANDBY)" % (
                        n, float(c["gps_wait_s"])), now)
                self.fallback = True
                self.io.notice("UAV %d: no GPS fix yet - climbs to %.0f m in ALT_HOLD and waits for GPS" % (
                    n, float(c["fallback_alt_m"])), "info")
                self.cmdq = [("mode ALT_HOLD", allow.CMD_DO_SET_MODE, [1, allow.COPTER_ALT_HOLD]),
                             ("ARM", allow.CMD_COMPONENT_ARM_DISARM, [1, 0]),
                             ("TAKEOFF (ALT_HOLD)", allow.CMD_NAV_TAKEOFF,
                              [0, 0, 1, 0, 0, 0, float(c["fallback_alt_m"])])]
                self._phase("CMDS", now)
            return
        if ph == "CMDS":
            if hs != "OPEN":
                return self._fail("the hatch is not OPEN (%s)" % hs, now)
            if not self.cmdq:
                self._phase("CLIMB", now)
                return
            label, cmd, params = self.cmdq[0]
            self.wait = "cmd"
            wrap = self._cb()

            def ack(result, why):
                self.wait = None
                t = self.io.now()
                if result != 0:
                    return self._fail("UAV %d: %s refused (%s)" % (n, label, why or result), t)
                self.cmdq.pop(0)
                if cmd == allow.CMD_COMPONENT_ARM_DISARM:
                    self.armed_here, self.t_arm = True, t
                    self.io.armed(n)
            self.io.cmd(n, "usb", cmd, params, wrap(ack))
            return
        if ph == "CLIMB":
            alt = d.get("rel_alt")
            fresh = d.get("alt_age") is not None and d["alt_age"] < 2.0
            if fresh and alt >= float(c["clear_alt_m"]):
                self.t_clear = now
                self.armed_here = False                    # flying: never disarmed by a stop
                self._phase("CLEAR", now)
            elif now - self.t_arm > float(c["no_climb_s"]) and (not fresh or alt < 1.0):
                self.keep_open = True
                self.io.cmd(n, "auto", allow.CMD_COMPONENT_ARM_DISARM, [0, 0], lambda r, w: None)
                self.armed_here = False
                self.io.notice("UAV %d did not leave the hatch - DISARMED; check before closing (HATCH CLOSE = "
                               "the captain's button)" % n, "caution")
                return self._fail("UAV %d did not leave the hatch" % n, now)
            elif now - self.t_arm > float(c["clear_s"]):
                return self._fail("UAV %d did not reach %.0f m in %.0f s" % (n, float(c["clear_alt_m"]),
                                                                          float(c["clear_s"])), now)
            return
        if ph == "CLEAR":
            if d.get("usb_present"):
                if now - self.t_phase > float(c["usb_gone_s"]):
                    return self._fail("UAV %d is clear but its slot USB cable is still there" % n, now)
                return
            if self.fallback:
                self.io.catchup(n)
                return self._done_drone(now)
            self.wait = "cmd"
            wrap = self._cb()

            def go(result, why):
                self.wait = None
                if result != 0:
                    return self._fail("UAV %d: MISSION START refused (%s)" % (n, why or result), self.io.now())
                self.io.on_mission(n)
                self._done_drone(self.io.now())
            self.io.cmd(n, "elrs", allow.CMD_MISSION_START, [mission.START_LAUNCH, 0], wrap(go))
            return
        if ph == "GAP":
            if now - self.t_phase >= 0:
                self._next(now)

    def _done_drone(self, now):
        self.launched.append(self.cur)
        self.log(now, "UAV %d launched%s" % (self.cur, " (climb fallback)" if self.fallback else ""))
        left = [s for s in self.slots if s not in self.launched]
        if not left:
            return self._close(now)
        wait = max(0.0, (self.t_arm or now) + float(self.cfg["gap_s"]) - now)
        self.phase, self.t_phase = "GAP", now + wait

    def status(self, now):
        return {"state": self.state, "why": self.why, "slots": self.slots, "launched": self.launched,
                "cur": self.cur, "phase": self.phase, "fallback": self.fallback, "keep_open": self.keep_open,
                "by": self.by, "captain": self.captain, "history": [t for _n, t in self.history[-6:]]}
