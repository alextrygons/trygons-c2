"""Interactive simulator lessons (ACADEMY_SPEC C, step 18): the model - no window, no network.

A lesson (content/lessons/*.json, kind "lesson", runs_on "sim") is a list of steps. Each step says
what to do (do_el / do_en), names the SIMULATOR STATE that proves it was done ("check", read from
the simulator's own telemetry - never from what the trainee clicked), gives a hint after hint_s
seconds and may have a time limit fail_s (a reaction time: too late = the step failed, retry).
A step may start a simulator event first ("at_start": "gps_fail" / "gps_ok"). The lesson's "setup"
puts the simulator in a known state before the first step (STANDBY, start power limit, GPS ok,
alarms acknowledged) and the lesson only starts once the simulated boat is ready (STANDBY, armed).

A check is a list of conditions (all must hold, for hold_s seconds when given):
    {"f": "<field>", "eq" | "ne" | "ge" | "le" | "gt" | "lt": value}
    {"f": "<field>", "bit": mask} / {"f": "<field>", "nobit": mask}      (alarm bits ...)
    {"f": "<field>", "delta": d, "tol": t}   = the field moved by d since the step started
                                               (angles wrap at 360)
Fields = groundstation/proto.py decode_telemetry + decode_telem2 names (FIELDS below).

The frames a lesson sends are only the simulator's inputs and the ground station's own commands
(ALLOWED_FRAMES); the link is academy/simguard.py SimLink (port 5760 of THIS PC only). Lessons
are grouped by level; the ones marked "required" must be passed before that level's exam
(missing_required). Results go to the academy logbook (accounts.Store.log kind "lesson").
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GS_DIR = os.path.join(os.path.dirname(HERE), "groundstation")
if GS_DIR not in sys.path:
    sys.path.append(GS_DIR)

import content                                                          # noqa: E402
import proto                                                            # noqa: E402

LANGS = ("el", "en")
LEVELS = content.LEVELS
OPS = ("eq", "ne", "ge", "le", "gt", "lt", "bit", "nobit", "delta")
ANGLES = ("heading", "target", "gps_cog", "est_cog")
FIELDS = set(proto.TELEM_FIELDS) | {
    "mode_name", "profile_name", "compass_ok", "gps_ok", "alarm", "stopping", "confirm_left_s",
    "power_raised", "speed_limited", "joy_take", "joy_confirm", "alarm_bits2"}
EVENTS = ("gps_fail", "gps_ok")
SETUP = ("standby", "power_lower", "gps_ok", "alarm_ack", "centre")
# what a lesson may send: the simulator's inputs + the station's own commands (never a gateway frame)
ALLOWED_FRAMES = {proto.T_HEARTBEAT, proto.T_SIM_JOYSTICK, proto.T_SIM_BUTTON, proto.T_SIM_GPS,
                  proto.T_HEADING_ADJ, proto.T_POWER_CODE, proto.T_POWER_RAISE, proto.T_POWER_LOWER,
                  proto.T_ALARM_ACK}
ALM_NOT_ARMED = 0x1000
BUTTONS = {"standby": proto.BTN_STANDBY, "hdg_lock": proto.BTN_HDG_LOCK, "minus1": proto.BTN_MINUS_1,
           "plus1": proto.BTN_PLUS_1, "minus10": proto.BTN_MINUS_10, "plus10": proto.BTN_PLUS_10}


def lessons(content_dir=None):
    """Every lesson document, sorted by level then "order"."""
    out = []
    for f in content.files(content_dir):
        d = content.load(f)
        if d.get("kind") == "lesson":
            out.append(d)
    return sorted(out, key=lambda d: (content.LEVELS.index(d.get("level", "DRIVER"))
                                      if d.get("level") in content.LEVELS else 9, d.get("order", 99), d.get("id")))


def by_level(docs=None):
    """{level: [lesson, ...]} in level order (spec C "grouped by level")."""
    docs = lessons() if docs is None else docs
    out = {lv: [] for lv in LEVELS}
    for d in docs:
        out.setdefault(d.get("level", "DRIVER"), []).append(d)
    return out


def required_for(level, docs=None):
    """The ids of the lessons required before the exam of this level (that level and the ones below)."""
    docs = lessons() if docs is None else docs
    rk = LEVELS.index(level) if level in LEVELS else len(LEVELS) - 1
    return [d["id"] for d in docs if d.get("required", True) and d.get("level") in LEVELS
            and LEVELS.index(d["level"]) <= rk]


def done_ids(entries, user):
    """Lesson ids this user passed (logbook entries kind lesson, done)."""
    return {e.get("id") for e in entries if e.get("user") == user and e.get("kind") == "lesson" and e.get("done")}


def missing_required(entries, user, exam=None, level=None, docs=None):
    """Lessons still to pass before an exam: the exam's "required" list, else every required lesson of the level."""
    need = list((exam or {}).get("required") or required_for(level or (exam or {}).get("level", "DRIVER"), docs))
    have = done_ids(entries, user)
    return [i for i in need if i not in have]


def check_doc(doc, name="?"):
    """Lesson-specific checks (content.check does the common ones). [] = OK."""
    p = []
    if doc.get("kind") != "lesson":
        return p
    if not doc.get("id"):
        p.append("%s: no id" % name)
    for s in doc.get("setup", []):
        if s not in SETUP:
            p.append("%s: unknown setup %r" % (name, s))
    steps = doc.get("steps")
    if not isinstance(steps, list) or not steps:
        p.append("%s: no steps" % name)
        return p
    for i, st in enumerate(steps):
        w = "%s step %d" % (name, i + 1)
        for k in ("do_el", "do_en", "hint_el", "hint_en"):
            if not isinstance(st.get(k), str) or not st[k].strip():
                p.append("%s: no %s" % (w, k))
        ev = st.get("at_start")
        if ev is not None and ev not in EVENTS:
            p.append("%s: unknown at_start %r" % (w, ev))
        chk = st.get("check")
        if not isinstance(chk, list) or not chk:
            p.append("%s: no check (the simulator state that proves the step)" % w)
            continue
        for c in chk:
            if not isinstance(c, dict) or c.get("f") not in FIELDS:
                p.append("%s: unknown field %r" % (w, c.get("f") if isinstance(c, dict) else c))
                continue
            ops = [o for o in OPS if o in c]
            if len(ops) != 1:
                p.append("%s: %s needs exactly one of %s" % (w, c["f"], "/".join(OPS)))
        for k in ("hint_s", "fail_s", "hold_s"):
            if k in st and (not isinstance(st[k], (int, float)) or st[k] <= 0):
                p.append("%s: %s must be > 0" % (w, k))
    return p


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def cond_ok(c, state, start):
    """One condition against the simulator state (start = the state when the step started)."""
    f = c["f"]
    if f not in state:
        return False
    v = state[f]
    try:
        if "eq" in c:
            return v == c["eq"]
        if "ne" in c:
            return v != c["ne"]
        if "ge" in c:
            return v >= c["ge"]
        if "le" in c:
            return v <= c["le"]
        if "gt" in c:
            return v > c["gt"]
        if "lt" in c:
            return v < c["lt"]
        if "bit" in c:
            return bool(int(v) & int(c["bit"]))
        if "nobit" in c:
            return not int(v) & int(c["nobit"])
        if "delta" in c:
            if f not in start:
                return False
            d = v - start[f]
            if f in ANGLES:
                d = _wrap(d)
            return abs(d - c["delta"]) <= c.get("tol", 0.5)
    except TypeError:
        return False
    return False


def ready(state):
    """The simulated boat is ready for a lesson: telemetry seen, STANDBY, outputs armed, CREWED."""
    return (state.get("mode_name") == "STANDBY" and "alarm_bits" in state and
            not state["alarm_bits"] & ALM_NOT_ARMED and state.get("profile_name", "CREWED") == "CREWED")


class Run(object):
    """One go at a lesson. start(now) -> setup actions; update(state, now) -> (status, actions):
    status "waiting" (simulator not ready), "step" (doing step index), "done" (passed), "failed"
    (a time limit passed: retry()). Actions = simulator events to send (EVENTS / SETUP names)."""

    def __init__(self, doc, lang="el"):
        self.doc = doc
        self.lang = lang if lang in LANGS else "el"
        self.steps = doc["steps"]
        self.retries = 0
        self.reset()

    def reset(self):
        self.index = -1                     # -1 = waiting for the simulator to be ready
        self.status = "waiting"
        self.t_step = None
        self.t_hold = None
        self.start_state = {}
        self.hints = 0
        self.hint_shown = False
        self.times = []
        self.t0 = None
        self.failed_step = None

    def text(self, key, step=None):
        st = self.steps[self.index if step is None else step]
        return st.get("%s_%s" % (key, self.lang)) or st.get("%s_el" % key, "")

    def title(self):
        return self.doc.get("title_" + self.lang) or self.doc.get("title_el", "")

    def start(self, now):
        self.t0 = now
        return list(self.doc.get("setup", []))

    def retry(self, now):
        self.retries += 1
        self.reset()
        return self.start(now)

    def _begin(self, i, state, now):
        self.index = i
        self.status = "step"
        self.t_step = now
        self.t_hold = None
        self.hint_shown = False
        self.start_state = dict(state)
        ev = self.steps[i].get("at_start")
        return [ev] if ev else []

    def update(self, state, now):
        if self.status in ("done", "failed"):
            return self.status, []
        if self.index < 0:
            if not ready(state):
                return "waiting", []
            return "step", self._begin(0, state, now)
        st = self.steps[self.index]
        if all(cond_ok(c, state, self.start_state) for c in st["check"]):
            if self.t_hold is None:
                self.t_hold = now
            if now - self.t_hold >= st.get("hold_s", 0):
                self.times.append(now - self.t_step)
                if self.index + 1 >= len(self.steps):
                    self.status = "done"
                    return "done", []
                return "step", self._begin(self.index + 1, state, now)
        else:
            self.t_hold = None
        if "fail_s" in st and now - self.t_step > st["fail_s"]:
            self.status = "failed"
            self.failed_step = self.index
            return "failed", []
        if not self.hint_shown and now - self.t_step >= st.get("hint_s", 30):
            self.hint_shown = True
            self.hints += 1
        return "step", []

    def score(self):
        """100 = every step without a hint and without a retry; -10 per hint, -10 per retry (>= 50 when passed)."""
        if self.status != "done":
            return 0
        return max(50, 100 - 10 * self.hints - 10 * self.retries)

    def log_entry(self):
        """The logbook fields (accounts.Store.log(user, "lesson", **these))."""
        return {"id": self.doc["id"], "topic": self.doc["id"], "level": self.doc.get("level"),
                "done": self.status == "done", "score": self.score(), "hints": self.hints, "retries": self.retries,
                "step_s": [round(t, 1) for t in self.times]}


class Sim(object):
    """The simulator's inputs + the state read back, over a byte link (simguard.SimLink in the window;
    any object with send(bytes) / recv() -> bytes). Sends only ALLOWED_FRAMES."""

    def __init__(self, link):
        self.link = link
        self.rx = proto.Receiver()
        self.seq = 0
        self.state = {}
        self.acks = []
        self.joy = {"x": 0.0, "y": 0.0, "take": False, "auto": False, "confirm": False}
        self.sent = []

    def send(self, ftype, payload=b""):
        if ftype not in ALLOWED_FRAMES:
            raise ValueError("a lesson never sends frame 0x%02X" % ftype)
        self.seq = (self.seq + 1) & 0xFF
        self.sent.append(ftype)
        if len(self.sent) > 200:
            del self.sent[:100]
        self.link.send(proto.encode(ftype, self.seq, payload))

    def tick(self):
        """Call ~20 times per second: heartbeat + the simulated joystick, then read what came back."""
        j = self.joy
        self.send(proto.T_HEARTBEAT)
        self.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, j["x"], j["y"], j["take"], j["auto"], j["confirm"]))
        for f in self.rx.frames_from(self.link.recv()):
            try:
                if f.type == proto.T_TELEMETRY:
                    self.state.update(proto.decode_telemetry(f.payload))
                elif f.type == proto.T_TELEM2:
                    self.state.update(proto.decode_telem2(f.payload))
                elif f.type == proto.T_ACK:
                    a = proto.decode_ack(f.payload)
                    self.acks.append(a)
                    del self.acks[:-20]
            except (ValueError, KeyError, IndexError):
                pass
        return self.state

    # --- trainee inputs
    def button(self, name):
        self.send(proto.T_SIM_BUTTON, proto.p_sim_button(BUTTONS[name]))

    def stick(self, x=None, y=None):
        if x is not None:
            self.joy["x"] = max(-1.0, min(1.0, float(x)))
        if y is not None:
            self.joy["y"] = max(-1.0, min(1.0, float(y)))

    def press(self, name, on):
        self.joy[name] = bool(on)

    def power_code(self):
        self.send(proto.T_POWER_CODE, proto.p_power_code(0))

    def power_raise(self, code):
        self.send(proto.T_POWER_RAISE, proto.p_power_raise(0, int(code) & 0xFFFF))

    def power_lower(self):
        self.send(proto.T_POWER_LOWER)

    def alarm_ack(self):
        self.send(proto.T_ALARM_ACK)

    # --- lesson events + setup
    def do(self, action):
        if action == "gps_fail":
            self.send(proto.T_SIM_GPS, proto.p_sim_gps(True))
        elif action == "gps_ok":
            self.send(proto.T_SIM_GPS, proto.p_sim_gps(False))
        elif action == "standby":
            self.button("standby")
        elif action == "power_lower":
            self.power_lower()
        elif action == "alarm_ack":
            self.alarm_ack()
        elif action == "centre":
            self.stick(0.0, 0.0)
            for k in ("take", "auto", "confirm"):
                self.joy[k] = False
        else:
            raise ValueError("unknown lesson action %r" % action)
