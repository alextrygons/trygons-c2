"""Emergency scenarios (ACADEMY_SPEC E, step 19): the model - no window, no network.

A scenario (content/scenarios/*.json, kind "scenario", runs_on "sim") puts the trainee in a normal
situation on the SIMULATOR (setup + "prep" steps, like a lesson: e.g. HDG LOCK at speed), then at
event.after_s something goes wrong and the trainee's REACTION is timed and scored:

  * a SIMULATOR fault ("fault": compass / gps_spoof / slam / joy_break = PROTO_T_SIM_FAULT, "gps_fail")
    - the simulated AURIX sees it and reacts with its own, unchanged code (its alarms, its mode changes);
  * a HAZARD the academy draws ("hazard": ais / shallow / swimmer) and an alarm the ground station
    would show ("show": gateway_lost / remote_link_lost ...). The academy never plays the gateway
    (D2): these exist only in the academy's picture of the station; the boat's movement is the
    simulator's (the trainee's inputs), so the outcome (did the boat stay clear?) is measured on the
    simulator's own position.

"react" = the condition(s) that prove the right reaction ("any": a list of alternative checks, each a
list of conditions that must all hold - lesson.cond_ok + "delta_ge" / "delta_le" / "adelta_ge" for a
turn since the event); good_s = full marks, max_s = too late (failed). "forbid" = conditions that fail
the scenario at once while reacting / observing (e.g. turning to port in front of a ship crossing from
starboard). "observe_s" = how long the outcome is watched after the reaction (a hazard: "min_m" = the
closest the boat may come; shallow: never inside). "after" = follow-up steps (lesson steps, untimed:
e.g. acknowledge once the cause is gone). The trainee's own stick is the fields in_x / in_y.

Score (passed): 100 when reacted within good_s, down to 50 at max_s, -10 per retry, never below 50.
Every run records a TIMELINE (5 Hz samples + events) = the replay; debrief() = what happened, when,
the score and the explanation (debrief_el / right_el ...). The replay file goes through
simguard.check_recording when it is read back.
"""
import json
import math
import os

import content
import lesson
import proto                                                            # groundstation/ (lesson put it on the path)

LANGS = lesson.LANGS
FAULTS = tuple(proto.SIM_FAULTS)                    # compass, gps_spoof, slam, joy_break
HAZARDS = ("ais", "shallow", "swimmer")
SHOWN = ("gateway_lost", "remote_link_lost", "collision_risk", "shallow_water")   # academy-shown alarm codes
SHOWN_BITS = {"gateway_lost": 0x10000, "remote_link_lost": 0x40000, "collision_risk": 0x800000,
              "shallow_water": 0x2000000}
EXTRA_OPS = ("delta_ge", "delta_le", "adelta_ge")
FIELDS = lesson.FIELDS | {"in_x", "in_y", "gear"}
ALLOWED_FRAMES = set(lesson.ALLOWED_FRAMES) | {proto.T_SIM_FAULT}
SAMPLE_S = 0.2                                      # timeline: 5 Hz
WAIT_FOR_MAX_S = 15.0                               # event.wait_for: the emergency starts at the latest after this
M_PER_DEG = 111320.0
KN = 0.514444


def scenarios(content_dir=None):
    """Every scenario document, sorted by level then "order"."""
    out = [d for d in (content.load(f) for f in content.files(content_dir)) if d.get("kind") == "scenario"]
    return sorted(out, key=lambda d: (content.LEVELS.index(d["level"]) if d.get("level") in content.LEVELS else 9,
                                      d.get("order", 99), d.get("id")))


def _check_conds(conds, where, p):
    if not isinstance(conds, list) or not conds:
        p.append("%s: empty check" % where)
        return
    for c in conds:
        if not isinstance(c, dict) or c.get("f") not in FIELDS:
            p.append("%s: unknown field %r" % (where, c.get("f") if isinstance(c, dict) else c))
            continue
        ops = [o for o in lesson.OPS + EXTRA_OPS if o in c]
        if len(ops) != 1:
            p.append("%s: %s needs exactly one operator" % (where, c["f"]))


def check_doc(doc, name="?"):
    """Scenario-specific checks (content.check does the common ones). [] = OK."""
    p = []
    if doc.get("kind") != "scenario":
        return p
    if not doc.get("id"):
        p.append("%s: no id" % name)
    for s in doc.get("setup", []):
        if s not in lesson.SETUP:
            p.append("%s: unknown setup %r" % (name, s))
    for key in ("prep", "after"):
        for i, st in enumerate(doc.get(key, [])):
            w = "%s %s %d" % (name, key, i + 1)
            for k in ("do_el", "do_en", "hint_el", "hint_en"):
                if not isinstance(st.get(k), str) or not st[k].strip():
                    p.append("%s: no %s" % (w, k))
            _check_conds(st.get("check"), w, p)
    ev = doc.get("event")
    if not isinstance(ev, dict):
        p.append("%s: no event" % name)
        return p
    if not isinstance(ev.get("after_s"), (int, float)) or ev["after_s"] < 0:
        p.append("%s: event.after_s must be >= 0" % name)
    for k in ("say_el", "say_en"):
        if not isinstance(ev.get(k), str) or not ev[k].strip():
            p.append("%s: event.%s missing (what the trainee sees)" % (name, k))
    for f in ev.get("faults", []):
        if f not in FAULTS + ("gps_fail",):
            p.append("%s: unknown fault %r" % (name, f))
    for s in ev.get("show", []):
        if s not in SHOWN:
            p.append("%s: unknown shown alarm %r" % (name, s))
    hz = ev.get("hazard")
    if hz is not None:
        if not isinstance(hz, dict) or hz.get("type") not in HAZARDS:
            p.append("%s: unknown hazard %r" % (name, hz))
        elif not isinstance(doc.get("outcome", {}).get("min_m"), (int, float)):
            p.append("%s: a hazard needs outcome.min_m" % name)
    if not (ev.get("faults") or ev.get("show") or hz):
        p.append("%s: the event does nothing (no fault, no hazard, no shown alarm)" % name)
    if "wait_for" in ev:
        _check_conds(ev["wait_for"], "%s event.wait_for" % name, p)
    if "clear_after_s" in ev and not (isinstance(ev["clear_after_s"], (int, float)) and ev["clear_after_s"] > 0):
        p.append("%s: event.clear_after_s must be > 0" % name)
    r = doc.get("react")
    if not isinstance(r, dict) or not isinstance(r.get("any"), list) or not r["any"]:
        p.append("%s: react.any (the right reactions) missing" % name)
    else:
        for i, alt in enumerate(r["any"]):
            _check_conds(alt, "%s react %d" % (name, i + 1), p)
        g, m = r.get("good_s"), r.get("max_s")
        if not (isinstance(g, (int, float)) and isinstance(m, (int, float)) and 0 < g < m):
            p.append("%s: react needs 0 < good_s < max_s" % name)
    for i, fb in enumerate(doc.get("forbid", [])):
        _check_conds(fb.get("check") if isinstance(fb, dict) else None, "%s forbid %d" % (name, i + 1), p)
        for k in ("why_el", "why_en"):
            if not isinstance(fb, dict) or not isinstance(fb.get(k), str) or not fb[k].strip():
                p.append("%s forbid %d: no %s" % (name, i + 1, k))
    for k in ("debrief_el", "debrief_en", "right_el", "right_en"):
        if not isinstance(doc.get(k), str) or not doc[k].strip():
            p.append("%s: no %s" % (name, k))
    return p


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def cond_ok(c, state, start):
    """lesson.cond_ok + the turn operators (signed / absolute heading change since the event)."""
    for op in EXTRA_OPS:
        if op in c:
            f = c["f"]
            if f not in state or f not in start:
                return False
            d = state[f] - start[f]
            if f in lesson.ANGLES:
                d = _wrap(d)
            if op == "delta_ge":
                return d >= c[op]
            if op == "delta_le":
                return d <= c[op]
            return abs(d) >= c[op]
    return lesson.cond_ok(c, state, start)


def all_ok(conds, state, start):
    return all(cond_ok(c, state, start) for c in conds)


# --- the local picture (metres north / east of the event point) --------------------------------
def local(state, origin):
    lat, lon = state.get("est_lat", 0.0), state.get("est_lon", 0.0)
    return ((lat - origin[0]) * M_PER_DEG, (lon - origin[1]) * M_PER_DEG * math.cos(math.radians(origin[0])))


def _vec(deg, d):
    r = math.radians(deg)
    return d * math.cos(r), d * math.sin(r)


def cpa(p_own, v_own, p_t, v_t):
    """(cpa_m, tcpa_s) of two straight tracks (n, e metres; velocities m/s)."""
    dn, de = p_t[0] - p_own[0], p_t[1] - p_own[1]
    vn, ve = v_t[0] - v_own[0], v_t[1] - v_own[1]
    vv = vn * vn + ve * ve
    t = 0.0 if vv < 1e-9 else -(dn * vn + de * ve) / vv
    return math.hypot(dn + vn * t, de + ve * t), t


class Hazard(object):
    """An academy-drawn hazard, placed from the boat's position / heading / speed at the event."""

    def __init__(self, spec, heading, sog_kn):
        self.spec = spec
        self.type = spec["type"]
        v = max(2.0, sog_kn * KN)
        if self.type == "ais":
            tc = spec.get("collide_s", 45.0)
            vt = spec.get("speed_kn", 8.0) * KN
            self.course = (heading + spec.get("cross_deg", -90.0)) % 360.0
            pn, pe = _vec(heading, v * tc)                      # where the boat will be at tc
            tn, te = _vec(self.course, vt * tc)
            self.p0 = (pn - tn, pe - te)
            self.v = _vec(self.course, vt)
            self.name = spec.get("name", "TRAINING SHIP")
        elif self.type == "shallow":
            self.r = spec.get("radius_m", 40.0)
            self.p0 = _vec(heading, spec.get("edge_m", 80.0) + self.r)
            self.v = (0.0, 0.0)
        else:                                                   # swimmer / kayak: still, no AIS
            n, e = _vec(heading, spec.get("dist_m", 70.0))
            on, oe = _vec(heading + 90.0, spec.get("off_m", 4.0))
            self.p0 = (n + on, e + oe)
            self.v = (0.0, 0.0)

    def pos(self, t):
        return self.p0[0] + self.v[0] * t, self.p0[1] + self.v[1] * t

    def clearance(self, boat, t):
        """The distance that counts for the outcome (shallow: to the edge, < 0 = aground)."""
        p = self.pos(t)
        d = math.hypot(p[0] - boat[0], p[1] - boat[1])
        return d - self.r if self.type == "shallow" else d

    def depth(self, boat, t):
        """shallow: a simple made-up depth (m) under the boat: 1 m inside, deeper away from the edge."""
        if self.type != "shallow":
            return None
        c = self.clearance(boat, t)
        return 1.0 if c <= 0 else min(30.0, 1.0 + 0.15 * c)


class Run(object):
    """One go at a scenario. start(now) -> setup actions; update(state, now) -> (status, actions).
    status: waiting (simulator not ready) / prep (a prep step) / wait_event / pending (faults in, waiting
    for event.wait_for) / react / observe / after / done / failed. Actions = simulator actions (lesson.SETUP / EVENTS names, "fault:<x>", "fault_off:<x>")."""

    def __init__(self, doc, lang="el"):
        self.doc = doc
        self.lang = lang if lang in LANGS else "el"
        self.ev = doc["event"]
        self.react_doc = doc["react"]
        self.retries = 0
        self.reset()

    def reset(self):
        self.status = "waiting"
        self.index = -1                     # prep / after step index
        self.t0 = None
        self.t_step = None
        self.t_hold = None
        self.step_start = {}
        self.t_ready = None
        self.t_event = None
        self.t_inject = None
        self.t_react = None
        self.t_observe = None
        self.ev_state = {}
        self.origin = None
        self.hazard = None
        self.shown = []
        self.cleared = False
        self.min_m = None
        self.why = None                     # failed: "late" / "forbid" / "outcome"
        self.forbid_hit = None
        self.timeline = []
        self.events = []
        self.t_sample = None
        self.cleaned = False

    def text(self, key):
        return self.doc.get("%s_%s" % (key, self.lang)) or self.doc.get("%s_el" % key, "")

    def title(self):
        return self.text("title")

    def step_text(self, key):
        steps = self.doc.get("prep" if self.status in ("prep", "waiting") else "after", [])
        if 0 <= self.index < len(steps):
            st = steps[self.index]
            return st.get("%s_%s" % (key, self.lang)) or st.get("%s_el" % key, "")
        return ""

    def say(self):
        return self.ev.get("say_" + self.lang) or self.ev.get("say_el", "")

    def start(self, now):
        """Every simulator fault off first (a clean simulator, also after an interrupted run), then the setup."""
        self.t0 = now
        return ["fault_off:%s" % f for f in FAULTS] + list(self.doc.get("setup", []))

    def retry(self, now):
        self.retries += 1
        self.reset()
        return self.start(now)

    def _mark(self, now, what, **kw):
        e = {"t": round(now - (self.t0 or now), 2), "what": what}
        e.update(kw)
        self.events.append(e)

    def _steps(self, key, state, now):
        """Lesson-like steps; True when every one passed."""
        steps = self.doc.get(key, [])
        if self.index >= len(steps):
            return True
        st = steps[self.index]
        if all_ok(st["check"], state, self.step_start):
            if self.t_hold is None:
                self.t_hold = now
            if now - self.t_hold >= st.get("hold_s", 0):
                self._mark(now, key, step=self.index + 1)
                self.index += 1
                self.t_step, self.t_hold, self.step_start = now, None, dict(state)
                return self.index >= len(steps)
        else:
            self.t_hold = None
        return False

    def shown_bits(self):
        """The academy-shown alarms as alarm bits (the window draws them like the station would)."""
        b = 0
        for s in self.shown:
            b |= SHOWN_BITS[s]
        return b

    def cleanup(self):
        """Simulator actions that undo every fault of this scenario (once)."""
        if self.cleaned:
            return []
        self.cleaned = True
        acts = ["fault_off:%s" % f for f in self.ev.get("faults", []) if f in FAULTS]
        if "gps_fail" in self.ev.get("faults", []):
            acts.append("gps_ok")
        return acts

    def _sample(self, state, now):
        if self.t_sample is not None and now - self.t_sample < SAMPLE_S:
            return
        self.t_sample = now
        s = {"t": round(now - self.t0, 2), "mode": state.get("mode_name"), "hdg": round(state.get("heading", 0.0), 1),
             "sog": round(state.get("est_sog", 0.0), 2), "thr": round(state.get("throttle", 0.0), 1),
             "alm": int(state.get("alarm_bits", 0)) | self.shown_bits(), "in_x": round(state.get("in_x", 0.0), 2),
             "in_y": round(state.get("in_y", 0.0), 2), "status": self.status}
        if self.origin is not None:
            n, e = local(state, self.origin)
            s["n"], s["e"] = round(n, 1), round(e, 1)
            if self.hazard is not None:
                hn, he = self.hazard.pos(now - self.t_event)
                s["hz_n"], s["hz_e"] = round(hn, 1), round(he, 1)
                s["clear_m"] = round(self.hazard.clearance((n, e), now - self.t_event), 1)
        self.timeline.append(s)

    def picture(self, state, now):
        """What the window draws now: boat + hazard in local metres, CPA / TCPA, depth (None before the event)."""
        if self.origin is None:
            return None
        boat = local(state, self.origin)
        out = {"boat": boat, "heading": state.get("heading", 0.0), "hazard": None}
        if self.hazard is not None:
            t = now - self.t_event
            hz = self.hazard
            out["hazard"] = {"type": hz.type, "pos": hz.pos(t), "clear_m": hz.clearance(boat, t)}
            if hz.type == "ais":
                v_own = _vec(state.get("heading", 0.0), state.get("est_sog", 0.0) * KN)
                c, tc = cpa(boat, v_own, hz.pos(t), hz.v)
                out["hazard"].update({"cpa_m": c, "tcpa_s": tc, "course": hz.course, "name": hz.name,
                                      "kn": self.hazard.spec.get("speed_kn", 8.0)})
            if hz.type == "shallow":
                out["hazard"].update({"r": hz.r, "depth": hz.depth(boat, t)})
        return out

    def update(self, state, now):
        if self.status in ("done", "failed"):
            return self.status, self.cleanup()
        acts = []
        self._last = state
        if self.status == "waiting":
            if not lesson.ready(state):
                return "waiting", []
            self.status, self.index, self.t_step, self.step_start = "prep", 0, now, dict(state)
            self._mark(now, "ready")
        if self.status != "waiting" and self.t0 is not None:
            self._sample(state, now)
        if self.status == "prep":
            if self._steps("prep", state, now):
                self.status, self.t_ready = "wait_event", now
            return self.status, acts
        if self.status == "wait_event":
            if now - self.t_ready < self.ev.get("after_s", 0):
                return self.status, acts
            self.t_inject = now
            for f in self.ev.get("faults", []):
                acts.append("gps_fail" if f == "gps_fail" else "fault:%s" % f)
            self._mark(now, "injected", faults=self.ev.get("faults", []))
            self.status = "pending"
            if not self.ev.get("wait_for"):
                return self._event(state, now, acts)
            return self.status, acts
        if self.status == "pending":
            # the faults are in; the emergency "starts" (say + reaction clock) once the simulator shows it
            if all_ok(self.ev["wait_for"], state, {}) or now - self.t_inject > WAIT_FOR_MAX_S:
                return self._event(state, now, acts)
            return self.status, acts
        return self._running(state, now, acts)

    def _event(self, state, now, acts):
        """The emergency starts now: the reaction clock, the shown alarms, the hazard placed."""
        self.t_event = now
        self.ev_state = dict(state)
        self.origin = (state.get("est_lat", 0.0), state.get("est_lon", 0.0))
        self.shown = list(self.ev.get("show", []))
        if self.ev.get("hazard"):
            self.hazard = Hazard(self.ev["hazard"], state.get("heading", 0.0), state.get("est_sog", 0.0))
        self.status = "react"
        self._mark(now, "event", faults=self.ev.get("faults", []), show=self.shown,
                   hazard=(self.ev.get("hazard") or {}).get("type"))
        self._sample(state, now)
        return self.status, acts

    def _running(self, state, now, acts):
        t = now - self.t_event
        if not self.cleared and "clear_after_s" in self.ev and t >= self.ev["clear_after_s"]:
            self.cleared = True
            acts += self.cleanup()
            self.cleaned = False                # the final cleanup may still be needed for later faults
            self._mark(now, "cleared")
        # the outcome is watched from the event on
        if self.hazard is not None:
            c = self.hazard.clearance(local(state, self.origin), t)
            self.min_m = c if self.min_m is None else min(self.min_m, c)
            if c < self.doc["outcome"]["min_m"]:
                return self._fail(now, "outcome", acts)
        if self.status in ("react", "observe"):
            for fb in self.doc.get("forbid", []):
                if all_ok(fb["check"], state, self.ev_state):
                    self.forbid_hit = fb
                    return self._fail(now, "forbid", acts)
        if self.status == "react":
            if any(all_ok(alt, state, self.ev_state) for alt in self.react_doc["any"]):
                self.t_react = now
                self.status, self.t_observe = "observe", now
                self._mark(now, "reacted", s=round(self.reaction_s(), 2))
            elif t > self.react_doc["max_s"]:
                return self._fail(now, "late", acts)
            return self.status, acts
        if self.status == "observe":
            if now - self.t_observe >= self.doc.get("observe_s", 0) or self._clear(state, t):
                self._mark(now, "observed", min_m=None if self.min_m is None else round(self.min_m, 1))
                if self.doc.get("after"):
                    self.status, self.index, self.t_step, self.t_hold = "after", 0, now, None
                    self.step_start = dict(state)
                    acts += self.cleanup()
                else:
                    return self._done(now, acts)
            return self.status, acts
        if self.status == "after":
            if self._steps("after", state, now):
                return self._done(now, acts)
        return self.status, acts

    def _clear(self, state, t):
        """A hazard that is clearly past (the outcome can no longer get worse): end the observation early."""
        hz = self.hazard
        if hz is None:
            return False
        boat = local(state, self.origin)
        if state.get("mode_name") == "STANDBY" and state.get("est_sog", 9) < 0.6:
            return True
        if hz.type == "ais":
            v_own = _vec(state.get("heading", 0.0), state.get("est_sog", 0.0) * KN)
            _c, tc = cpa(boat, v_own, hz.pos(t), hz.v)
            return tc < -5.0
        p = hz.pos(t)
        brg = math.degrees(math.atan2(p[1] - boat[1], p[0] - boat[0]))
        return abs(_wrap(brg - state.get("heading", 0.0))) > 110.0     # behind the boat

    def _final_sample(self, now):
        """The last state always goes into the replay (the end may come between two samples)."""
        if getattr(self, "_last", None) is not None and self.t0 is not None:
            self.t_sample = None
            self._sample(self._last, now)

    def _done(self, now, acts):
        self.status = "done"
        self._final_sample(now)
        self._mark(now, "done", score=self.score())
        return "done", acts + self.cleanup()

    def _fail(self, now, why, acts):
        self.status, self.why = "failed", why
        self._final_sample(now)
        self._mark(now, "failed", why=why)
        return "failed", acts + self.cleanup()

    def reaction_s(self):
        return None if self.t_react is None else self.t_react - self.t_event

    def score(self):
        """100 within good_s, linear to 50 at max_s; -10 per retry; >= 50 when passed, 0 when failed."""
        if self.status != "done":
            return 0
        g, m = self.react_doc["good_s"], self.react_doc["max_s"]
        r = self.reaction_s() or 0.0
        base = 100.0 if r <= g else 100.0 - 50.0 * min(1.0, (r - g) / (m - g))
        return int(max(50, round(base) - 10 * self.retries))

    def log_entry(self, replay=None):
        """The logbook fields (accounts.Store.log(user, "scenario", **these))."""
        r = self.reaction_s()
        return {"id": self.doc["id"], "topic": self.doc["id"], "level": self.doc.get("level"),
                "done": self.status == "done", "score": self.score(), "retries": self.retries,
                "reaction_s": None if r is None else round(r, 2), "why": self.why,
                "min_m": None if self.min_m is None else round(self.min_m, 1), "replay": replay}

    def debrief(self):
        """The debrief lines (screen language): result, reaction time, closest approach, explanation."""
        from text import T
        L = self.lang
        out = []
        if self.status == "done":
            out.append(T("scen.db_passed", L, score=self.score()))
        else:
            key = "scen.db_failed_" + (self.why or "late")          # late / forbid / outcome
            out.append(T(key, L))
            if self.forbid_hit:
                out.append(self.forbid_hit.get("why_" + L) or self.forbid_hit.get("why_el", ""))
        r = self.reaction_s()
        out.append(T("scen.db_reaction", L, s="%.1f" % r, good=self.react_doc["good_s"], max=self.react_doc["max_s"])
                   if r is not None else T("scen.db_no_reaction", L, max=self.react_doc["max_s"]))
        if self.min_m is not None:
            out.append(T("scen.db_min", L, m="%.0f" % self.min_m, need=self.doc["outcome"]["min_m"]))
        out.append(T("scen.db_right", L) + " " + self.text("right"))
        out.append(self.text("debrief"))
        return out

    def replay_doc(self):
        """Header + timeline for the replay file (English keys; trainee texts by id from the content)."""
        return {"kind": "scenario_replay", "id": self.doc["id"], "status": self.status, "why": self.why,
                "score": self.score(), "reaction_s": self.reaction_s(), "min_m": self.min_m,
                "hazard": (self.ev.get("hazard") or {}).get("type"),
                "hazard_r": getattr(self.hazard, "r", None), "events": self.events}


def save_replay(run, folder, user=None, stamp=None):
    """Write the replay (.jsonl: header line, then one sample per line). Returns the path."""
    import datetime
    os.makedirs(folder, exist_ok=True)
    stamp = stamp or datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    name = "%s_%s_%s.jsonl" % (user or "practice", run.doc["id"], stamp)
    path = os.path.join(folder, "".join(c if c.isalnum() or c in "._-" else "_" for c in name))
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps(run.replay_doc(), ensure_ascii=False) + "\n")
        for s in run.timeline:
            f.write(json.dumps(s) + "\n")
    return path


def load_replay(path):
    """(header, samples) of a replay file - only an ordinary recorded file (simguard.check_recording)."""
    import simguard
    p = simguard.check_recording(path)
    with open(p, encoding="utf-8") as f:
        lines = [ln for ln in f.read().splitlines() if ln.strip()]
    if not lines:
        raise ValueError("empty replay")
    head = json.loads(lines[0])
    if head.get("kind") != "scenario_replay":
        raise ValueError("not a scenario replay")
    samples = []
    for ln in lines[1:]:
        try:
            samples.append(json.loads(ln))
        except ValueError:                      # a cut last line: skipped
            pass
    return head, samples


def replay_dir(env=None):
    """academy/data/replays (gitignored; TRYGONS_ACADEMY_DATA overrides the data folder)."""
    env = os.environ if env is None else env
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(env.get("TRYGONS_ACADEMY_DATA") or os.path.join(here, "data"), "replays")


class Sim(lesson.Sim):
    """lesson.Sim + the simulator faults (PROTO_T_SIM_FAULT) + the trainee's own stick as fields
    in_x / in_y. Sends only ALLOWED_FRAMES (the simulator's inputs + the station's own commands)."""

    def send(self, ftype, payload=b""):
        if ftype not in ALLOWED_FRAMES:
            raise ValueError("a scenario never sends frame 0x%02X" % ftype)
        self.seq = (self.seq + 1) & 0xFF
        self.sent.append(ftype)
        if len(self.sent) > 200:
            del self.sent[:100]
        self.link.send(proto.encode(ftype, self.seq, payload))

    def tick(self):
        """lesson.Sim.tick, keeping every refused command too (self.refused: the window shows why)."""
        j = self.joy
        self.send(proto.T_HEARTBEAT)
        self.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, j["x"], j["y"], j["take"], j["auto"], j["confirm"]))
        if not hasattr(self, "refused"):
            self.refused = []
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
                    if not a.get("ok"):
                        self.refused_n = getattr(self, "refused_n", 0) + 1
                        self.refused.append(a)
                        del self.refused[:-50]
            except (ValueError, KeyError, IndexError):
                pass
        st = self.state
        st["in_x"], st["in_y"] = j["x"], j["y"]
        return st

    def do(self, action):
        if action.startswith("fault:"):
            self.send(proto.T_SIM_FAULT, proto.p_sim_fault(action[6:], True))
        elif action.startswith("fault_off:"):
            self.send(proto.T_SIM_FAULT, proto.p_sim_fault(action[10:], False))
        else:
            lesson.Sim.do(self, action)

    def nacks(self):
        """The refused PC commands seen (the last 50; the window shows the reason)."""
        return list(getattr(self, "refused", []))
