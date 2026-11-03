"""YES / NO answers on the shore joystick (step 13p, owner 2026-10-31, docs/owner_findings_2026-10-31.md D).

DISPLAY + REQUESTS ONLY: this module never drives - it only answers this station's own questions (screen 1's
question box: a copilot's action for the captain's ACCEPT, an avoidance proposal, the slamming power restore, a
vision slow-down note; the single operator's own confirmation). Every answer goes the way a mouse click goes
(the gateway / the AURIX check it again).

  TRIGGER        = answer with the SELECTED choice (YES is pre-selected: the trigger alone = a fast ACCEPT); only a
                   FRESH press made once the question has been shown >= GUARD_S 0.5 s (a press held from before
                   never answers).
  thumb button   = a quick REJECT (fresh press, the same 0.5 s guard).
  stick Y        = the selection: forward = YES, back = NO, beyond Y_ON 40 %, a fresh move from the centre - ONLY at a
                   station with a throttle LEVER (the spring-centred stick's Y is free there) and ONLY in STANDBY /
                   AUTO / HDG LOCK / DP. MANEUVER: Y = the gear (13q5); REMOTE MANUAL: Y does nothing; a station
                   driving with the stick Y (no lever): never - the hat instead.
  hat up / down  = YES / NO selection everywhere (a fresh push).

One press never does two things (owner D): a trigger press made while a question is open (or that answered one)
is NEVER sent to the boat (send_trigger() masks it until it is released) - with no question open the trigger goes
to the boat as before (UNCREWED HDG LOCK steered by hand: the new locked heading, step 13q3). The gateway masks it
too (gateway/stations.py, its own questions for this station).
"""
import threading

import proto

Y_ON = 0.40                 # a stick Y deflection beyond this = a selection
Y_CENTRE = 0.15             # back within this = centred again (the next selection needs a fresh move)
GUARD_S = 0.5               # the question must have been shown this long before a press answers it
Y_MODES = (proto.MODE_STANDBY, proto.MODE_AUTO, proto.MODE_HDG_LOCK, proto.MODE_DP)


def y_enabled(mode, lever):
    """The stick Y selects YES / NO here: a lever station (Y is free) in STANDBY / AUTO / HDG LOCK / DP only."""
    return bool(lever) and mode in Y_MODES


class Selector:
    """One per station. question() from the Tk loop (the question now open, None = none), feed() with the stick's
    controls (shorejoy.controls) -> an event, send_trigger() from the 30 Hz joystick sender thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.q = None               # the open question's token
        self.shown = 0.0            # when it appeared
        self.sel = True             # True = YES selected (pre-selected), False = NO
        self.trig_was = False
        self.rej_was = False
        self.hat_was = 0
        self.y_centred = False      # the stick Y seen centred since the last selection (a fresh move)
        self.block = False          # the trigger press now held must not reach the boat
        self.answered = 0           # counters (tests / the Remote page)
        self.guarded = 0            # presses ignored: within GUARD_S or held from before
        self.masked = 0             # trigger packets kept from the boat

    def question(self, token, now):
        """The question open now (any hashable token, None = none); a new token = YES pre-selected again."""
        with self.lock:
            if token != self.q:
                self.q = token
                self.shown = now
                self.sel = True
                self.y_centred = False
                if token is not None and self.trig_was:
                    self.block = True           # a press held from before the question never reaches the boat now

    def feed(self, c, now, mode):
        """c = shorejoy.controls() (None = no joystick). Returns None, ("select", yes) or ("answer", yes)."""
        c = c or {}
        trig, rej, hat = bool(c.get("trigger")), bool(c.get("reject")), int(c.get("hat") or 0)
        y = c.get("y")
        ev = None
        with self.lock:
            if self.q is not None:
                ready = now - self.shown >= GUARD_S
                if y_enabled(mode, c.get("lever")) and y is not None:
                    if abs(y) < Y_CENTRE:
                        self.y_centred = True
                    elif abs(y) > Y_ON and self.y_centred:
                        self.sel = y > 0
                        self.y_centred = False
                        ev = ("select", self.sel)
                if hat and hat != self.hat_was:
                    self.sel = hat > 0
                    ev = ("select", self.sel)
                if rej and not self.rej_was:
                    if ready:
                        ev = ("answer", False)
                    else:
                        self.guarded += 1
                elif trig and not self.trig_was:
                    if ready:
                        ev = ("answer", self.sel)
                    else:
                        self.guarded += 1
                if trig:
                    self.block = True           # this press belongs to the question (never also sent to the boat)
                if ev and ev[0] == "answer":
                    self.answered += 1
            self.trig_was, self.rej_was, self.hat_was = trig, rej, hat
        return ev

    def send_trigger(self, pressed):
        """The trigger bit for the next joystick packet to the boat: never while a question is open, never for a press
        that answered / was held into one (until it is released)."""
        with self.lock:
            if not pressed:
                self.block = False
                return False
            if self.q is not None or self.block:
                self.block = True
                self.masked += 1
                return False
            return True
