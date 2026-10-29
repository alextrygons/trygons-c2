"""The PRACTICAL TEST on the real boat (ACADEMY_SPEC G + J, step 21): the model - no window, no network.

A Test only OBSERVES (the telemetry the station already receives + the station's audit log, through
linkview.LinkView) and GRADES. It never sends a driving command; its one frame is the final approval's
CONFIRM_ONLY (changes nothing on the boat, answered by the onboard joystick CONFIRM within 10 s). Every safety
rule stays the AURIX's own - the test only ADDS gates (spec "the academy only ADDS gates").

Kinds (content/practical.json "kinds"): upgrade (one level up: DRIVER / CAPTAIN), check_ride (a CAPTAIN every 24
months), refresher (the supervised trip after no logged driving in currency_days).

Before the start (start_problems; shown, "refuses and says why"): the area set (never the placeholder) and the
boat inside it, CREWED profile, not FIRST TEST, the start power limit active (30 / 5 %, not raised), calm water
(sea state + the slamming monitor + the examiner's tick), the examiner on board (his tick; the final CONFIRM is
on the ONBOARD joystick), the wind typed in + confirmed by the examiner and within the limits / not from a
blocked direction, the allowed hours, the examiner = a valid CAPTAIN with his own 2FA login (never the trainee),
the trainee eligible (the exam of the level passed, the required lessons), not an online session, fresh
telemetry.

While it runs (update): each task is SEEN from the telemetry (the evidence), then APPROVED by the examiner
(approve() refuses an unseen task); the examiner may fail a task or abort at any time; leaving the test area =
alert + the test STOPPED (the module sends nothing - never a mode change); the whole ride is recorded
(<data>/practical/<id>.jsonl: samples at record_hz, events, the audit lines) and linked to the logbook.
The end: every task approved -> final_request() -> CONFIRM_ONLY with a random token -> TELEM2 group R shows that
token CONFIRMED -> passed; LATE / CANCELLED / no answer = not passed (ask again). The level / log change itself =
accounts.Store.grant_practical(result, examiner session).
"""
import datetime
import json
import math
import os
import secrets
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ACADEMY = os.path.dirname(HERE)
for _d in (ACADEMY, HERE):
    if _d not in sys.path:
        sys.path.append(_d)

import accounts                                                         # noqa: E402
import content                                                          # noqa: E402
import online                                                           # noqa: E402
import ptarea as areamod                                               # noqa: E402

CONFIG = os.path.join(ACADEMY, "content", "practical.json")
KINDS = ("upgrade", "check_ride", "refresher")
CO_PENDING, CO_CONFIRMED, CO_LATE, CO_CANCELLED = 1, 2, 3, 4          # TELEM2 group R (app/smallboat.h SB_CO_)
PROFILE_CREWED = 1
ALM_SAMPLE = ("mode", "mode_name", "target", "heading", "throttle", "gear", "gps_lat", "gps_lon", "est_lat",
              "est_lon", "gps_sog", "gps_ok", "power_fwd", "power_rev", "power_raised", "alarm_bits", "profile",
              "co_state", "co_token", "slam_state", "sea_state", "confirm_kind")


def config(path=None):
    with open(path or CONFIG, encoding="utf-8") as f:
        return json.load(f)


def check_doc(doc, name="?"):
    p = []
    if doc.get("kind") != "practical":
        return p
    tasks = doc.get("tasks") or {}
    for k, ids in (doc.get("kinds") or {}).items():
        if k.startswith("_"):
            continue
        for i in ids:
            if i not in tasks:
                p.append("%s: kind %s: unknown task %r" % (name, k, i))
    for k in ("upgrade_DRIVER", "upgrade_CAPTAIN", "check_ride", "refresher"):
        if not (doc.get("kinds") or {}).get(k):
            p.append("%s: no task list %s" % (name, k))
    for k in ("leave_m", "return_m", "ramp_m", "react_s", "fresh_s", "confirm_wait_s", "course_change_deg"):
        if not isinstance(doc.get(k), (int, float)) or doc[k] <= 0:
            p.append("%s: %s > 0" % (name, k))
    return p


def angdiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def position(t):
    """The boat's position from the telemetry: the GPS when trusted, None otherwise."""
    if not t or not t.get("gps_ok"):
        return None
    lat, lon = t.get("gps_lat"), t.get("gps_lon")
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)) or (lat == 0.0 and lon == 0.0):
        return None
    return [float(lat), float(lon)]


def target_level(kind, level):
    """The level an upgrade leads to from the trainee's effective level (None = no upgrade from there)."""
    if kind != "upgrade":
        return None
    return {"OBSERVER": "DRIVER", "DRIVER": "CAPTAIN"}.get(level)


def exam_passed(entries, user, level, months=12):
    since = datetime.date.today() - datetime.timedelta(days=int(30.44 * months))
    for e in reversed(entries):
        if e.get("user") == user and e.get("kind") == "exam" and e.get("passed") and e.get("level") == level:
            d = accounts._date(str(e.get("t", ""))[:10])
            if d and d >= since:
                return e
    return None


def required_missing(entries, user, level):
    """The lessons of this level still to pass (lesson.missing_required, without importing the lesson module)."""
    have = {e.get("id") for e in entries if e.get("user") == user and e.get("kind") == "lesson" and e.get("done")}
    rk = content.LEVELS.index(level)
    need = []
    for f in content.files():
        d = content.load(f)
        if d.get("kind") == "lesson" and d.get("required", True) and d.get("level") in content.LEVELS and \
                content.LEVELS.index(d["level"]) <= rk:
            need.append(d["id"])
    return [i for i in need if i not in have]


class Test(object):
    """One practical test / check ride / refresher trip. Times are time.time() seconds (now=...)."""

    def __init__(self, store, area, kind, trainee, session, cfg=None, env=None, today=None):
        if kind not in KINDS:
            raise ValueError("kind %r" % kind)
        self.store, self.area, self.kind, self.trainee, self.session = store, area, kind, trainee, session
        self.cfg = cfg or config()
        self.env = env
        self.today = today
        acc = store.accounts.get(trainee)
        self.from_level = acc.effective_level(today) if acc else "OBSERVER"
        self.target = target_level(kind, self.from_level) if kind == "upgrade" else self.from_level
        key = "upgrade_%s" % self.target if kind == "upgrade" else kind
        ids = (self.cfg.get("kinds") or {}).get(key) or []
        self.tasks = [{"id": i, "state": "todo", "note": ""} for i in ids]
        self.state = "ready"            # ready -> running -> confirming -> passed | stopped | aborted
        self.why = ""
        self.alert = None
        self.id = None
        self.token = None
        self.confirm = {"state": "none", "token": None}
        self.recording = None
        self.started = self.ended = None
        self.wind = None
        self._prev_mode = None
        self._hdg0 = None
        self._took_over = False
        self._maneuver_near = False
        self._alarm_t = None
        self._last_rec = 0.0
        self._confirm_deadline = 0.0
        self.audit_lines = 0

    # ---- helpers
    def task(self, tid):
        for t in self.tasks:
            if t["id"] == tid:
                return t
        return None

    def _seen(self, tid, note=""):
        t = self.task(tid)
        if t and t["state"] == "todo":
            t["state"], t["note"] = "seen", note
            self._event("seen", task=tid, note=note)

    def _dist_ramp(self, pos):
        if pos is None or not self.area.ramp_bottom:
            return None
        return areamod.dist_m(pos, self.area.ramp_bottom)

    # ---- before the start
    def start_problems(self, t, age_s, ticks, wind_kn, wind_from, when=None, now=None):
        """Why the test can't start ([] = it can). ticks = the examiner's {"calm", "on_board", "wind"} ticks."""
        p = []
        st = self.store
        today = self.today
        ticks = ticks or {}
        if online.is_online(self.env):
            p.append("an online session: the practical test runs only on the boat's own station")
        why = st.session_ok(self.session, role="examiner", today=today)
        if why:
            p.append("examiner: " + why)
        elif self.session.user == self.trainee:
            p.append("the examiner can't examine himself")
        acc = st.accounts.get(self.trainee)
        if acc is None:
            p.append("unknown trainee")
        elif self.kind == "upgrade":
            if self.target is None:
                p.append("no upgrade from %s" % self.from_level)
            else:
                if not exam_passed(st.entries(), self.trainee, self.target):
                    p.append("the %s exam is not passed (last 12 months)" % self.target)
                miss = required_missing(st.entries(), self.trainee, self.target)
                if miss:
                    p.append("lessons first: " + ", ".join(miss))
        elif self.kind == "check_ride" and self.from_level != "CAPTAIN":
            p.append("a check ride is for a valid CAPTAIN")
        elif self.kind == "refresher" and accounts.rank(self.from_level) < accounts.rank("DRIVER"):
            p.append("a refresher trip is for a valid DRIVER / CAPTAIN")
        if not self.tasks:
            p.append("no tasks for this test")
        p += self.area.problems()
        if t is None or age_s is None or age_s > float(self.cfg.get("fresh_s", 3)):
            p.append("no fresh telemetry from the boat")
            return p
        if t.get("profile") != PROFILE_CREWED:
            p.append("CREWED profile only (the boat shows %s)" % t.get("profile_name", t.get("profile")))
        if t.get("first_test"):
            p.append("not in a FIRST TEST build")
        if t.get("mode") != 0:
            p.append("start in STANDBY")
        if t.get("power_raised") or float(t.get("power_fwd", 100)) > 30.5 or float(t.get("power_rev", 100)) > 5.5:
            p.append("the start power limit (30 / 5 %%) must be active (now %.0f / %.0f %%)" % (
                float(t.get("power_fwd", 0)), float(t.get("power_rev", 0))))
        pos = position(t)
        if pos is None:
            p.append("no trusted GPS position")
        elif not self.area.placeholder and not self.area.inside(*pos):
            p.append("the boat is outside the test area")
        if int(t.get("sea_state", 9)) > int(self.cfg.get("max_sea_state", 2)) or \
                t.get("slam_state", "CALM") != "CALM" or not ticks.get("calm"):
            p.append("calm water: sea state <= %s, the slamming monitor CALM and the examiner's tick" %
                     self.cfg.get("max_sea_state", 2))
        if not ticks.get("on_board"):
            p.append("the examiner must be on board (tick)")
        if not ticks.get("wind"):
            p.append("the examiner must confirm the typed wind (tick)")
        p += self.area.wind_problems(wind_kn, wind_from)
        h = self.area.hours_problem(when)
        if h:
            p.append(h)
        return p

    def start(self, t, age_s, ticks, wind_kn, wind_from, when=None, now=None):
        probs = self.start_problems(t, age_s, ticks, wind_kn, wind_from, when, now)
        if probs:
            raise accounts.simguard.Refused("; ".join(probs))
        now = time.time() if now is None else now
        self.id = "pt_%s_%s" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(now)), secrets.token_hex(3))
        d = os.path.join(self.store.folder, "practical")
        os.makedirs(d, exist_ok=True)
        self.recording = os.path.join(d, self.id + ".jsonl")
        self.state, self.started = "running", now
        self.wind = {"kn": float(wind_kn), "from": float(wind_from) % 360.0}
        self._prev_mode = t.get("mode")
        self._event("start", kind=self.kind, trainee=self.trainee, examiner=self.session.user, target=self.target,
                    wind=self.wind, tasks=[x["id"] for x in self.tasks], area=self.area.path)
        self.update(t, 0.0, now)
        return self.id

    # ---- the recording
    def _write(self, rec):
        if not self.recording:
            return
        try:
            with open(self.recording, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except OSError:
            pass

    def _event(self, what, **kw):
        self._write(dict(kw, kind="event", what=what, t=time.time()))

    def audit(self, lines):
        """The station's audit / event lines (LinkView.audit) into the recording."""
        for ln in lines or []:
            self.audit_lines += 1
            self._write({"kind": "audit", "t": time.time(), "line": str(ln)[:300]})

    # ---- while it runs
    def update(self, t, age_s, now=None):
        """One telemetry frame. -> the alert text (leaving the area) or None."""
        now = time.time() if now is None else now
        if self.state not in ("running", "confirming") or t is None:
            return self.alert
        hz = float(self.cfg.get("record_hz", 5))
        if now - self._last_rec >= 1.0 / max(0.5, hz):
            self._last_rec = now
            self._write({"kind": "sample", "t": now, "d": {k: t.get(k) for k in ALM_SAMPLE if k in t}})
        pos = position(t)
        if pos is not None and not self.area.inside(*pos):
            return self.stop("left the test area", alert=True)
        if t.get("profile") != PROFILE_CREWED:
            return self.stop("the boat left the CREWED profile", alert=True)
        mode = t.get("mode")
        prev = self._prev_mode
        dr = self._dist_ramp(pos)
        # leave the harbour / return
        if dr is not None and dr > float(self.cfg["leave_m"]):
            self._seen("leave_harbour", "%.0f m from the ramp" % dr)
        lv = self.task("leave_harbour")
        if dr is not None and dr < float(self.cfg["return_m"]) and (lv is None or lv["state"] != "todo"):
            self._seen("return", "%.0f m from the ramp" % dr)
        # HDG LOCK + course change
        if mode == 2:
            if prev != 2 or self._hdg0 is None:
                self._hdg0 = float(t.get("target", 0.0))
            elif angdiff(float(t.get("target", 0.0)), self._hdg0) >= float(self.cfg["course_change_deg"]) - 0.01:
                self._seen("hdg_lock_change", "course %.0f -> %.0f" % (self._hdg0, float(t.get("target", 0.0))))
        else:
            self._hdg0 = None
        # joystick takeover from HDG LOCK / AUTO, then the hand-back to HDG LOCK / AUTO
        if mode == 1 and prev in (2, 3):
            self._took_over = True
            self._event("takeover", from_mode=prev)
        if self._took_over and mode in (2, 3) and prev == 1:
            self._seen("takeover", "MANUAL, then back")
        # the examiner's alarm: STANDBY within react_s
        if self._alarm_t is not None:
            dt = now - self._alarm_t
            if mode == 0:
                self._alarm_t = None
                if dt <= float(self.cfg["react_s"]):
                    self._seen("alarm_reaction", "STANDBY after %.1f s" % dt)
                else:
                    self._note("alarm_reaction", "too late: %.1f s" % dt)
            elif dt > float(self.cfg["react_s"]):
                self._alarm_t = None
                self._note("alarm_reaction", "no STANDBY within %.0f s" % float(self.cfg["react_s"]))
                self._event("alarm_late")
        # MANEUVER at the ramp, then STANDBY almost stopped
        near = dr is not None and dr < float(self.cfg["ramp_m"])
        if mode == 7 and near:
            self._maneuver_near = True
        if self._maneuver_near and mode == 0 and near and float(t.get("gps_sog", 9.0) or 0.0) < 0.5:
            self._seen("trailer_maneuver", "MANEUVER at %.0f m from the ramp, then STANDBY" % dr)
        self._prev_mode = mode
        # the final approval
        if self.state == "confirming":
            co, tok = t.get("co_state"), t.get("co_token")
            if tok == self.token and co == CO_CONFIRMED:
                self.confirm = {"state": "confirmed", "token": self.token}
                self.state, self.ended = "passed", now
                self._event("confirmed", token=self.token)
            elif (tok == self.token and co in (CO_LATE, CO_CANCELLED)) or now > self._confirm_deadline:
                self.confirm = {"state": "late" if co == CO_LATE else "not answered", "token": self.token}
                self.state = "running"
                self.why = "no onboard CONFIRM: ask again"
                self._event("confirm_failed", co_state=co, token=self.token)
        return self.alert

    def _note(self, tid, note):
        t = self.task(tid)
        if t and t["state"] == "todo":
            t["note"] = note

    def trigger_alarm(self, now=None):
        """The examiner's alarm (spec G): shown + sounded ON THIS SCREEN only - nothing is sent to the boat."""
        if self.state != "running" or self.task("alarm_reaction") is None:
            return False
        if self._prev_mode in (None, 0):
            self.why = "the alarm is for a moving boat (not in STANDBY)"
            return False
        self._alarm_t = time.time() if now is None else now
        self._event("alarm")
        return True

    def approve(self, tid):
        """The examiner approves a task - only one SEEN in the telemetry."""
        t = self.task(tid)
        if self.state != "running" or t is None or t["state"] != "seen":
            return False
        t["state"] = "approved"
        self._event("approved", task=tid, by=self.session.user)
        return True

    def fail(self, tid, note=""):
        t = self.task(tid)
        if self.state != "running" or t is None:
            return False
        t["state"], t["note"] = "failed", note or t["note"]
        self._event("failed", task=tid, by=self.session.user, note=note)
        return True

    def stop(self, why, alert=False):
        if self.state in ("passed", "stopped", "aborted"):
            return self.alert
        self.state, self.why, self.ended = "stopped", why, time.time()
        if alert:
            self.alert = why
        self._event("stopped", why=why)
        self._log_fail()
        return self.alert

    def abort(self, why="aborted by the examiner"):
        if self.state in ("passed", "stopped", "aborted"):
            return False
        self.state, self.why, self.ended = "aborted", why, time.time()
        self._event("aborted", why=why)
        self._log_fail()
        return True

    def _log_fail(self):
        if self.id:
            self.store.log(self.trainee, self.kind if self.kind != "upgrade" else "practical", passed=False,
                           id=self.id, why=self.why, by=self.session.user, recording=self.recording,
                           target=self.target, tasks=[dict(x) for x in self.tasks])

    def all_approved(self):
        return bool(self.tasks) and all(x["state"] == "approved" for x in self.tasks)

    def final_request(self, view, now=None):
        """Every task approved -> the CONFIRM_ONLY question (a new random token). -> the token or None."""
        now = time.time() if now is None else now
        if self.state != "running" or not self.all_approved():
            return None
        self.token = secrets.randbelow(65535) + 1
        self.state = "confirming"
        self._confirm_deadline = now + float(self.cfg.get("confirm_wait_s", 12))
        self.confirm = {"state": "asked", "token": self.token}
        self._event("confirm_only", token=self.token)
        view.confirm_only(self.token)
        return self.token

    def result(self):
        return {"id": self.id, "kind": self.kind, "trainee": self.trainee, "examiner": self.session.user,
                "target": self.target, "passed": self.state == "passed", "tasks": [dict(x) for x in self.tasks],
                "token": self.token, "confirm_only": dict(self.confirm), "recording": self.recording,
                "area": self.area.path, "wind_kn": (self.wind or {}).get("kn"), "wind_from": (self.wind or {}).get("from"),
                "score": int(round(100.0 * sum(1 for x in self.tasks if x["state"] == "approved") / max(1, len(self.tasks)))),
                "started": self.started, "ended": self.ended}


def dist_text(m):
    return "-" if m is None or (isinstance(m, float) and math.isnan(m)) else "%.0f m" % m
