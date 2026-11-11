"""The launch sequence of the UAV magazine (CLAUDE.md V, docs/uav_design.md 5.3 / 5.6 / 7, owner spec H3 / H4; U3).

A state machine, one step per tick (4 Hz from the event loop), EVERY wait timed, nothing blocking: requests go to
the router process (mission upload, commands) or through uavlink (the hatch relay) and come back as callbacks.

  IDLE -> ASK (prepared: the captain's YES / NO on screen 1; a single operator answers his own) -> RUN
  RUN, per drone (the TOP slot still in the magazine, strictly in series):
     CHECK   the slot is the top one (next_slot), registered + checked + the same drone, on its USB cable, not armed,
             battery, sensors, its route still passes uavcheck from the boat's position now
     UPLOAD  the mission over the slot's USB + its read-back (router: count + every item compared)
     HATCH   (first drone) relay ON -> the OPEN switch (hatch.py; its travel time = FAULT)
     GPS     a 3D fix + EKF position within gps_wait_s (the hatch OPEN: the drone sees the sky); else the sequence
             STOPS with the notice "no GPS fix - not launched": NO launch without GPS (owner W138 2026-11-11, step
             UAV-r1 - the old ALT_HOLD "climb, then catch up" fallback is REMOVED: ArduCopter 4.7 SITL showed the
             ALT_HOLD take-off does not climb with the RC receiver ignored and the drone then stays armed, X2 SI13)
     CMDS    GUIDED -> ARM (the router's allow-list refuses ARM unless the hatch OPEN switch) -> TAKEOFF to the
             slot's initial altitude (30 + 5 m x (slot - 1)), each with its COMMAND_ACK
     CLIMB   relative altitude >= clear_alt within clear_s; NO climb within no_climb_s = DISARM at once (it is on the
             ground: a motor / prop fault in the hatch), stop, the hatch STAYS OPEN + notice "did not leave the
             hatch - check before closing" (the close is then the captain's own HATCH CLOSE); a DISARM the drone
             REFUSES is sent again FORCED (UAV-r1, W138 b) - allow.py lets a force DISARM out ONLY while the router
             sees the drone on the ground in its slot (ctx in_slot), never in flight: no drone stays armed in the hatch
     CLEAR   the slot's USB cable gone (the breakaway, W58) within usb_gone_s, then MISSION_START
     GAP     the next drone not before gap_s after this one's ARM (and only once this one is clear)
  CLOSING   after the last drone: relay OFF -> the CLOSED switch -> DONE
  STOPPED   ANY warning (a check, the hatch FAULT, stale boat telemetry, a boat WARNING alarm, the relay board silent, HELM lost, the captain's station lost, a
             station's STOP; NOT the boat's mode / speed: owner W59 - a launch goes when the copilot says so, above
             5 kn a non-blocking notice): no further drone, no ARM; drones in the air go on; an armed drone still on the ground
             is DISARMED (forced if refused, only in its slot); the hatch is switched by NOTHING (owner W72 2026-11-04: the captain's own HATCH CLOSE)
Nothing here sends to the AURIX but the hatch relay through io.hatch_relay (uavlink -> stations.py).
"""
from . import allow, mission

REFUSED_BY_LIST = "refused by the allow-list"   # the router's answer when allow.check_out refused a command
IDLE, ASK, RUN, CLOSING, DONE, STOPPED = "IDLE", "ASK", "RUN", "CLOSING", "DONE", "STOPPED"
EKF_POS = 0x08 | 0x10                     # EKF_STATUS_REPORT: horizontal position relative + absolute

DEFAULTS = {
    "high_speed_kn": 5.0,                 # W59 (owner 2026-11-04): above it a NON-blocking notice only, no speed limit
    "gps_wait_s": 30.0,                   # UAV-r1 (W138): the hatch open, then no fix in this time = STOP (TO CONFIRM
                                          # W143; was 5 s before the removed fallback climb)
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
    "cmd_s": 10.0,                        # U3 review M5: a command / relay answer not back this long = stop
    "relay_s": 15.0,
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
        self.wait_t = 0.0                 # when it went out (U3 review M5: every wait timed)
        self.cmdq = []
        self.t_arm = None
        self.t_clear = None
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
        # U3 review M1: an ARM still on its way (its answer not back) may have armed the drone: it is cancelled in the
        # router and the drone is DISARMED like one armed here; the result is reported, never assumed
        arm_out = (self.wait == "cmd" and bool(self.cmdq) and self.cmdq[0][1] == allow.CMD_COMPONENT_ARM_DISARM and
                   self.cmdq[0][2][0] == 1)
        was_q = (list(self.cmdq), self.wait, self.phase)
        self.gen += 1                      # every answer still on its way is ignored
        was, self.state, self.why = self.state, STOPPED, why
        self.wait, self.cmdq = None, []
        ended = getattr(self.io, "launch_ended", None)     # U6 review MED-3: the drone in the air is let go (holds)
        if self.cur is not None and ended is not None:
            ended(self.cur)
        if self.cur is not None and (self.armed_here or arm_out):
            n = self.cur
            if arm_out and hasattr(self.io, "cancel"):
                self.io.cancel(n, allow.CMD_COMPONENT_ARM_DISARM)
            d = self.io.slot(n)
            if d.get("landed") or d.get("usb_present"):
                # review HIGH-1: a FORCED retry only while the drone cannot be lifting off - its TAKEOFF not sent yet
                # (an ARM / GUIDED stage) or the sequence's own no-climb judgement; a STOP during a climb-out = the
                # normal DISARM only (the drone refuses it in the air - as it should)
                self._disarm(n, "the launch stopped", force_ok=self._grounded(n, now, was_q))
        self.log(now, "STOPPED (%s): %s" % (was, why))
        self.io.notice("UAV launch stopped: %s" % why, "caution")
        return True

    def _grounded(self, n, now, was):
        """review HIGH-1: True = the sequence knows slot n's drone cannot be lifting off: its TAKEOFF was never sent
        (still queued and not the command on its way), or the CLIMB phase's own no-climb judgement holds (no_climb_s
        after the ARM and a fresh altitude below 1 m)."""
        q, wait, phase = was
        if phase == "CMDS":
            tko = [i for i, c in enumerate(q) if c[1] == allow.CMD_NAV_TAKEOFF]
            return bool(tko) and not (wait == "cmd" and tko[0] == 0)
        if phase == "CLIMB" and self.t_arm is not None and now - self.t_arm > float(self.cfg["no_climb_s"]):
            d = self.io.slot(n)
            return d.get("alt_age") is not None and d["alt_age"] < 2.0 and (d.get("rel_alt") or 0.0) < 1.0
        return False

    def _disarm(self, n, why, force_ok=True):
        """DISARM the drone of slot n in the hatch; a DISARM the drone REFUSES (ArduCopter: "not landed" after an
        accepted take-off that did not climb, X2 SI13) is sent once more FORCED (param2 21196, UAV-r1 / W138 b).
        The router's allow.py lets the forced one out ONLY while the drone is on the ground in its slot (fresh USB
        HEARTBEAT + a fresh altitude < 0.5 m, not moving, not "in air"): never a force DISARM in flight."""
        def forced(result, w):
            if result == 0:
                self.io.notice("UAV %d disarmed in the hatch - FORCED (%s)" % (n, why), "caution")
            else:
                self.io.notice("UAV %d: DISARM not confirmed, also forced (%s) - check the magazine" % (
                    n, w or result), "caution")

        def normal(result, w):
            if result == 0:
                return self.io.notice("UAV %d disarmed in the hatch (%s)" % (n, why), "caution")
            # review M3 / LOW-1: forced only on a REFUSAL - the drone's MAV_RESULT, or allow.py's own (SITL: a stuck
            # take-off reports TAKEOFF, so the router refuses the normal DISARM as "not on the ground") - never on a
            # timeout / "replaced" / "cancelled" (maybe a lost link in the air); only while the sequence knows the
            # drone is not lifting off
            refused = result is not None or (w or "") == REFUSED_BY_LIST     # the drone's / allow.py's refusal
            if refused and force_ok and self.io.slot(n).get("usb_present"):
                return self.io.cmd(n, "usb", allow.CMD_COMPONENT_ARM_DISARM, [0, allow.FORCE_MAGIC], forced)
            self.io.notice("UAV %d: DISARM not confirmed (%s) - check the magazine" % (n, w or result), "caution")
        self.io.cmd(n, "auto", allow.CMD_COMPONENT_ARM_DISARM, [0, 0], normal)

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
            ro = getattr(self.io, "router_ok", None)
            if ro is not None and not ro():
                why.append("UAV router offline")
            if why:
                return self.stop(why[0], now)
            return self._run(now, boat, hs)
        # CLOSING (U3 review M5: never stuck - the relay answer and the travel are timed)
        if self.wait == "relay" and now - self.wait_t > float(self.cfg["relay_s"]):
            self.gen += 1
            self.wait, self.state, self.why = None, STOPPED, "HATCH NOT CLOSED: no answer to the relay command - "                                                               "take HELM and close it"
            self.io.notice(self.why, "caution")
            return
        if self.wait is None and self.phase == "close" and hs != "CLOSED" and                 now - self.t_phase > self.io.hatch_open_s() + float(self.cfg["relay_s"]):
            self.state, self.why = STOPPED, "the hatch did not close"
            self.io.notice("HATCH NOT CLOSED: the hatch did not close - take HELM and close it", "caution")
            return
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
        self.armed_here = False
        self.t_arm = self.t_clear = None
        self._phase("CHECK", now)

    def _close(self, now):
        self.cur, self.state = None, CLOSING
        self._phase("close", now)
        g = self.gen
        self.wait, self.wait_t = "relay", now

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
            if self.wait == "cmd" and now - self.wait_t > float(c["cmd_s"]):
                return self._fail("UAV %d: no answer to a command" % n, now)
            if self.wait == "relay" and now - self.wait_t > float(c["relay_s"]):
                return self._fail("HATCH NOT OPENED: no answer to the relay command", now)
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
                self.wait, self.wait_t = "relay", now
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
                # UAV-r1 (owner W138): NO launch without GPS - never armed, the sequence stops (the hatch: W72)
                return self._fail("UAV %d: no GPS fix in %.0f s with the hatch open - not launched (no launch "
                                  "without GPS)" % (n, float(c["gps_wait_s"])), now)
            return
        if ph == "CMDS":
            if hs != "OPEN":
                return self._fail("the hatch is not OPEN (%s)" % hs, now)
            if not self.cmdq:
                self._phase("CLIMB", now)
                return
            label, cmd, params = self.cmdq[0]
            self.wait, self.wait_t = "cmd", now
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
                self._disarm(n, "it did not climb")
                self.armed_here = False
                self.io.notice("UAV %d did not leave the hatch - check before closing (HATCH CLOSE = the captain's "
                               "button)" % n, "caution")
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
            locked = getattr(self.io, "pilot_locked", None)
            if locked is not None and locked(n):
                # U5 review MED-1: a pilot flies it (a take is refused during a launch - defence in depth): no
                # MISSION START under the pilot's hands; the drone counts as launched
                self.io.notice("UAV %d is flown by hand - its mission was not started" % n, "caution")
                return self._done_drone(now)
            self.wait, self.wait_t = "cmd", now
            wrap = self._cb()

            def go(result, why):
                self.wait = None
                if result != 0:
                    return self._fail("UAV %d: MISSION START refused (%s)" % (n, why or result), self.io.now())
                self.io.on_mission(n)
                self._done_drone(self.io.now())
            # UAV-r1 (SITL, ArduCopter 4.7): MISSION_START with a first / last item (param1 / param2 != 0) is
            # DENIED (MAV_RESULT 2) - [0, 0] starts a freshly uploaded mission at its first item after HOME = seq 1
            # = the take-off item (mission.START_LAUNCH)
            sc = getattr(self.io, "set_current", None)
            if sc is None:
                self.io.cmd(n, "elrs", allow.CMD_MISSION_START, [0, 0], wrap(go))
            else:
                # review MED-1: the first item pinned (a resume index left on the FC from the bench is never used)
                def cur(result, why):
                    if result != 0:
                        return go(result, "MISSION_SET_CURRENT: %s" % (why or result))
                    self.io.cmd(n, "elrs", allow.CMD_MISSION_START, [0, 0], wrap(go))
                sc(n, mission.START_LAUNCH, "elrs", wrap(cur))
            return
        if ph == "GAP":
            if now - self.t_phase >= 0:
                self._next(now)

    def _done_drone(self, now):
        self.launched.append(self.cur)
        self.log(now, "UAV %d launched" % self.cur)
        left = [s for s in self.slots if s not in self.launched]
        if not left:
            return self._close(now)
        wait = max(0.0, (self.t_arm or now) + float(self.cfg["gap_s"]) - now)
        self.phase, self.t_phase = "GAP", now + wait

    def status(self, now):
        return {"state": self.state, "why": self.why, "slots": self.slots, "launched": self.launched,
                "cur": self.cur, "phase": self.phase, "keep_open": self.keep_open,
                "by": self.by, "captain": self.captain, "history": [t for _n, t in self.history[-6:]]}
