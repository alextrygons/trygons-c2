"""The UAV magazine lesson "UAV launch and observation" (step U6, owner spec H7): the model - no window, no network.

A lesson document (content/lessons/u*.json, kind "uav_lesson", runs_on "sim") = the same step format as the boat
lessons (lesson.py: do / hint / check / hint_s / fail_s / hold_s / at_start), but its steps are checked against the
academy's OWN in-process UAV simulator (uavsim.UavSim: the gateway's launch / allow-list / route / battery rule
modules with a simulated boat + magazine) - never a gateway, never a drone, never a boat. Results go to the logbook
like a lesson (accounts.Store.log kind "lesson", the lesson's id).

Camera / observation only (CLAUDE.md V scope): the trainee actions are register, route, boat mode, prepare launch, the
captain's YES / NO, STOP, HATCH CLOSE, open a UAV video, acknowledge notices - there is nothing else.
"""
import content
import lesson
import uavsim

LANGS = lesson.LANGS
FIELDS = {"boat_mode", "sog_kn", "registered", "routes", "in_magazine", "launch", "phase", "why", "launched",
          "flying", "hatch", "view", "landing", "lost", "notices_open", "denied", "ready"}
EVENTS = ("battery_low", "hatch_fault", "warning")
SETUP = ("magazine_full", "boat_moving")
ACTIONS = ("register", "route_next", "standby", "hdg_lock", "prepare", "yes", "no", "stop", "hatch_close",
           "video", "map", "ack")


def lessons(content_dir=None):
    out = [d for d in (content.load(f) for f in content.files(content_dir)) if d.get("kind") == "uav_lesson"]
    return sorted(out, key=lambda d: (d.get("order", 99), d.get("id")))


def check_doc(doc, name="?"):
    """UAV-lesson checks (content.check does the common ones). [] = OK."""
    p = []
    if doc.get("kind") != "uav_lesson":
        return p
    if not doc.get("id"):
        p.append("%s: no id" % name)
    for s in doc.get("setup", []):
        if s not in SETUP:
            p.append("%s: unknown setup %r" % (name, s))
    steps = doc.get("steps")
    if not isinstance(steps, list) or not steps:
        return p + ["%s: no steps" % name]
    for i, st in enumerate(steps):
        w = "%s step %d" % (name, i + 1)
        for k in ("do_el", "do_en", "hint_el", "hint_en"):
            if not isinstance(st.get(k), str) or not st[k].strip():
                p.append("%s: no %s" % (w, k))
        if st.get("at_start") is not None and st["at_start"] not in EVENTS:
            p.append("%s: unknown at_start %r" % (w, st["at_start"]))
        chk = st.get("check")
        if not isinstance(chk, list) or not chk:
            p.append("%s: no check (the simulator state that proves the step)" % w)
            continue
        for c in chk:
            if not isinstance(c, dict) or c.get("f") not in FIELDS:
                p.append("%s: unknown field %r" % (w, c.get("f") if isinstance(c, dict) else c))
            elif len([o for o in lesson.OPS if o in c]) != 1:
                p.append("%s: %s needs exactly one of %s" % (w, c["f"], "/".join(lesson.OPS)))
        for k in ("hint_s", "fail_s", "hold_s"):
            if k in st and (not isinstance(st[k], (int, float)) or st[k] <= 0):
                p.append("%s: %s must be > 0" % (w, k))
    return p


def ready(state):
    """The simulated magazine is ready: its state seen."""
    return bool(state.get("ready"))


def new_run(doc, lang="el"):
    return lesson.Run(doc, lang, ready_fn=ready)


class Trainee(object):
    """The trainee's buttons on a UavSim (the window and the tests use the same ones). act(name) -> (ok, code, text)."""

    def __init__(self, sim):
        self.sim = sim
        self.log = []

    def act(self, name, arg=None):
        s = self.sim
        fn = {"register": lambda: s.register(arg or s.next_slot() or 1),
              "route_next": lambda: s.send_route("next"),
              "standby": lambda: s.set_boat("STANDBY"),
              "hdg_lock": lambda: s.set_boat("HDG_LOCK", 5.0),
              "prepare": lambda: s.prepare(arg or 1),
              "yes": lambda: s.answer(True), "no": lambda: s.answer(False),
              "stop": s.stop, "hatch_close": s.hatch_close,
              "video": lambda: s.open_video(arg or 1), "map": s.close_video, "ack": s.ack_notices}.get(name)
        if fn is None:
            raise ValueError("no UAV lesson action %r" % name)
        r = fn()
        self.log.append((name, r))
        return r


def make_sim(clock):
    return uavsim.UavSim(clock)
