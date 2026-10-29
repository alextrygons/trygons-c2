"""The PRACTICAL TEST's own proof (ACADEMY_SPEC G + I, step 21; test_academy AC14 requires this file).

  P1 static: no practical module imports a network / serial / process module or a boat link (link, gwclient,
     app, stations ...), loads code by name, or reaches the simulator / video doors of simguard; NO frame type
     but T_CONFIRM_ONLY appears; the only call that can send is LinkView.confirm_only (-> the station's cmd())
  P2 LinkView: read-only telemetry (a copy) + audit lines, ONE send = T_CONFIRM_ONLY with the token; no other
     public method, no attribute can be added (slots); no link = nothing sent
  P3 the test area: the placeholder refused; save only with a valid CAPTAIN's 2FA login, audit-logged, written
     with set_by / placeholder false / not_for_navigation; inside / outside; wind limit + blocked directions
     (north, the ramp's right side) from strong_wind_kn; hours; .geojson import; a COM port refused; boat GPS
     "add point" only with a trusted fix
  P4 the start refuses (and says why) for every condition: placeholder area, outside the area, UNCREWED, FIRST
     TEST, not STANDBY, a raised power limit, no calm water, examiner not on board, the wind not confirmed / too
     strong / blocked, outside the hours, stale telemetry, no GPS, the examiner = the trainee / not a CAPTAIN /
     expired / without 2FA / not logged in, the exam not passed, lessons missing, an online session
  P5 a whole test on made-up telemetry: tasks SEEN only from the telemetry, approve() refuses an unseen task,
     the examiner's alarm (STANDBY in time / late), the final CONFIRM_ONLY only after every approval and only
     through LinkView, LATE / another token / no answer = not passed, CONFIRMED = passed -> grant_practical
     raises exactly one level, logged; leaving the area = alert + STOPPED with nothing sent; the ride recorded
  P6 grant_practical refuses every forged / incomplete result (no CONFIRM, wrong token, another examiner,
     unapproved task, examiner = trainee, two levels up, check ride for a non-CAPTAIN, online, no 2FA, expired
     examiner); set_level can't raise a level once a CAPTAIN instructor exists ("only via the practical test")
  P7 the REAL simulator: CONFIRM_ONLY through LinkView -> pending -> the onboard CONFIRM -> CONFIRMED with the
     token in TELEM2 group R, and NOTHING changed (mode, power limit, throttle, gear, alarms); refused while a
     power raise waits for its CONFIRM (that one stays); the onboard joystick still takes over in HDG LOCK with
     a question pending (software never blocks the joystick); a press after 10 s = LATE, nothing happens
  P8 the station: menu Academy -> practical test / test area through practicalmenu (LinkView, never the link
     object); every text key used by the practical modules in both languages; the windows show the banner
     PRACTICAL TEST - REAL BOAT and run a test through their buttons on a fake station
Run:  python academy/practical/test_practical.py      (exit 0 = all passed)
"""
import ast
import datetime
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ACADEMY = os.path.dirname(HERE)
REPO = os.path.dirname(ACADEMY)
GS = os.path.join(REPO, "groundstation")
sys.path.insert(0, HERE)
sys.path.insert(1, ACADEMY)
sys.path.append(GS)
os.environ["TRYGONS_LANG"] = "el"
os.environ.setdefault("TRYGONS_ACADEMY_DATA", tempfile.mkdtemp(prefix="trygons_pt_data_"))
for _k in ("TRYGONS_ACADEMY_ONLINE", "SSH_CONNECTION", "SSH_CLIENT"):
    os.environ.pop(_k, None)
if str(os.environ.get("SESSIONNAME", "")).upper().startswith("RDP-"):
    os.environ["SESSIONNAME"] = "Console"

import accounts                                                         # noqa: E402
import auth                                                             # noqa: E402
import linkview                                                         # noqa: E402
import proto                                                            # noqa: E402
import ptarea                                                           # noqa: E402
import ptest                                                            # noqa: E402
import simguard                                                         # noqa: E402

results = []
NET = {"socket", "ssl", "serial", "http", "urllib", "requests", "asyncio", "telnetlib", "ftplib", "smtplib",
       "socketserver", "selectors", "subprocess", "multiprocessing", "pty", "aiohttp", "zmq", "paho", "can", "usb",
       "websocket", "websockets", "xmlrpc", "httpx", "urllib3", "paramiko"}
BOATLINKS = {"link", "gwclient", "app", "stations", "boats", "aurix", "gateway", "shorejoy", "syncrecv"}
NO_DOORS = ("SimLink", "start_simulator", "simguard.connect", "video_copy", "video_frame", "check_target")
CENTRE = (37.7800, 24.0400)


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def modules():
    return [os.path.join(HERE, f) for f in sorted(os.listdir(HERE))
            if f.endswith(".py") and not f.startswith("test_")]


# ---------------------------------------------------------------- helpers: accounts, area, telemetry
def store(tmp=None):
    return accounts.Store(tmp or tempfile.mkdtemp(prefix="trygons_pt_"))


def login(st, user, pw="longpassword1"):
    if not st.auth.has_password(user):
        st.auth.set_password(user, pw)
    if st.needs_2fa(user) and not st.auth.has_totp(user):
        st.auth.enrol_totp(user, pw)
    sec = st.auth.get(user).get("totp")
    return st.login(user, pw, auth.totp(sec) if sec else None)


def people(st):
    """An instructor CAPTAIN (examiner), a second CAPTAIN, a trainee OBSERVER ready for the DRIVER test, a DRIVER."""
    st.add("exa", "Examiner", instructor=True)
    st.add("cap2", "Other captain")
    st.add("ann", "Ann")
    st.add("drv", "Driver")
    st.set_level("drv", "DRIVER")                       # (the first set-up: bootstrap - no CAPTAIN instructor yet)
    st.set_level("cap2", "CAPTAIN")
    st.set_level("exa", "CAPTAIN")                      # from here on: levels go up only via the practical test
    for u, lv in (("ann", "DRIVER"), ("drv", "CAPTAIN")):
        lessons = ["l01_hdg_lock", "l02_takeover", "l03_standby_wins", "l04_alarm_reaction"] + \
                  (["l05_power_raise"] if lv == "CAPTAIN" else [])
        for lid in lessons:
            st.log(u, "lesson", id=lid, done=True)
        st.log(u, "exam", id="%s_exam" % lv.lower(), level=lv, score=90, passed=True)
    return login(st, "exa")


def area_doc(placeholder=False):
    a = ptarea.load(None, ptarea.CONTENT_AREA).doc
    d = json.loads(json.dumps(a))
    la, lo = CENTRE
    d["polygon"] = [[la + 0.01, lo - 0.01], [la + 0.01, lo + 0.01], [la - 0.01, lo + 0.01], [la - 0.01, lo - 0.01]]
    d["ramp_top"] = [la - 0.0062, lo]                   # the ramp runs north: top south, bottom north
    d["ramp_bottom"] = [la - 0.0060, lo]
    d["placeholder"] = placeholder
    d["set_by"] = "exa"
    return d


def real_area():
    return ptarea.Area(area_doc(False), "test")


def tel(**kw):
    la, lo = CENTRE
    t = {"t_ms": 1, "mode": 0, "mode_name": "STANDBY", "profile": 1, "profile_name": "CREWED", "first_test": False,
         "gps_ok": True, "gps_fix": 1, "gps_lat": la - 0.0060, "gps_lon": lo, "gps_sog": 0.0, "target": 90.0,
         "power_fwd": 30.0, "power_rev": 5.0, "power_raised": False, "sea_state": 1, "slam_state": "CALM",
         "co_state": 0, "co_token": 0, "alarm_bits": 0}
    t.update(kw)
    return t


def at_m(north_m, east_m=0.0):
    import math
    la, lo = CENTRE[0] - 0.0060, CENTRE[1]
    return {"gps_lat": la + north_m / 111320.0, "gps_lon": lo + east_m / (111320.0 * math.cos(math.radians(la)))}


TICKS = {"calm": True, "on_board": True, "wind": True}
NOON = datetime.datetime(2026, 10, 29, 12, 0)


class FakeApp(object):
    """The ground station as LinkView sees it: t / t_rx / _evlog / link / cmd()."""

    def __init__(self):
        self.t, self.t_rx, self._evlog, self.link, self.sent = None, 0.0, [], object(), []

    def cmd(self, ftype, payload, what):
        self.sent.append((ftype, bytes(payload), what))

    def event(self, text):
        self._evlog = (self._evlog + [text])[-200:]


# ---------------------------------------------------------------- P1
def imports_of(path):
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    names, dyn = set(), set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            names |= {a.name.split(".")[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.level == 0 and n.module:
            names.add(n.module.split(".")[0])
        elif isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in ("__import__", "exec", "eval",
                                                                                         "compile"):
            dyn.add(n.func.id)
        elif isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == "os" and \
                n.attr in ("system", "popen", "startfile", "execv", "spawnv"):
            dyn.add("os." + n.attr)
    if "importlib" in names:
        dyn.add("importlib")
    return names, dyn, tree


def p1():
    wrong = []
    for p in modules():
        name = os.path.basename(p)
        names, dyn, tree = imports_of(p)
        src = open(p, encoding="utf-8").read()
        if names & NET or dyn:
            wrong.append("%s imports %s" % (name, sorted((names & NET) | dyn)))
        if names & BOATLINKS:
            wrong.append("%s imports a boat link %s" % (name, sorted(names & BOATLINKS)))
        frames = set(re.findall(r"\bT_[A-Z0-9_]+\b", src)) - {"T_CONFIRM_ONLY"}
        if frames:
            wrong.append("%s names frames %s" % (name, sorted(frames)))
        for door in NO_DOORS:
            if door in src:
                wrong.append("%s uses simguard's %s" % (name, door))
        for n in ast.walk(tree):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
                a = n.func.attr
                if a in ("send", "send_msg", "send_rjoy", "_send_cmd", "sendall", "write_frame"):
                    wrong.append("%s calls .%s()" % (name, a))
                if a == "cmd" and name != "linkview.py":
                    wrong.append("%s calls .cmd()" % name)
    lv = open(os.path.join(HERE, "linkview.py"), encoding="utf-8").read()
    if lv.count("self._app.cmd(") != 1 or "FRAME = proto.T_CONFIRM_ONLY" not in lv:
        wrong.append("linkview: not exactly one cmd() with the fixed frame")
    report("P1 static: read-only + CONFIRM_ONLY is the only frame", not wrong, "; ".join(wrong) or
           "%d modules: no network / serial / process / boat-link import, no frame but T_CONFIRM_ONLY, one cmd() in "
           "LinkView" % len(modules()))


# ---------------------------------------------------------------- P2
def p2():
    wrong = []
    app = FakeApp()
    app.t, app.t_rx = tel(), 5.0
    v = linkview.LinkView(app)
    public = sorted(m for m in dir(v) if not m.startswith("_"))
    if public != ["audit", "confirm_only", "connected", "telemetry"]:
        wrong.append("public methods %s" % public)
    try:
        v.send = lambda *a: None
        wrong.append("an attribute could be added")
    except AttributeError:
        pass
    d, rx = v.telemetry()
    d["mode"] = 99
    if app.t["mode"] != 0 or rx != 5.0:
        wrong.append("telemetry is not a copy")
    app.event("a")
    app.event("b")
    a1 = v.audit()
    app.event("c")
    a2 = v.audit()
    if a1 != ["a", "b"] or a2 != ["c"] or v.audit() != []:
        wrong.append("audit %s %s" % (a1, a2))
    v.confirm_only(0x1234)
    if app.sent != [(proto.T_CONFIRM_ONLY, proto.p_confirm_only(0x1234), "confirm-only (practical test)")]:
        wrong.append("sent %s" % app.sent)
    app.link = None
    if v.confirm_only(5) is not None or len(app.sent) != 1:
        wrong.append("sent without a link")
    if proto.T_CONFIRM_ONLY != 0x30 or proto.is_driving_cmd(proto.T_CONFIRM_ONLY, b"\x01\x00"):
        wrong.append("frame number / driving-command flag")
    hdr = open(os.path.join(REPO, "app", "proto.h"), encoding="utf-8").read()
    if not re.search(r"#define PROTO_T_CONFIRM_ONLY\s+0x30u", hdr):
        wrong.append("app/proto.h PROTO_T_CONFIRM_ONLY")
    report("P2 LinkView: read-only + one frame", not wrong, "; ".join(wrong) or
           "telemetry (a copy) + audit lines; confirm_only -> T_CONFIRM_ONLY 0x30 (= proto.h, not a driving "
           "command); no other method, slots; no link = nothing")


# ---------------------------------------------------------------- P3
def p3():
    wrong = []
    st = store()
    exa = people(st)
    ph = ptarea.load(None, ptarea.CONTENT_AREA)
    if not ph.placeholder or not any("PLACEHOLDER" in p for p in ph.problems()):
        wrong.append("the content area is not refused as a placeholder")
    a = real_area()
    if a.problems():
        wrong.append("a real area refused: %s" % a.problems())
    if not a.inside(*CENTRE) or a.inside(CENTRE[0] + 0.02, CENTRE[1]):
        wrong.append("inside / outside")
    # wind: ramp bearing = north (0): its right side = east (45..135)
    cases = [((5, 0), True), ((13, 200), False), ((10, 0), False), ((10, 350), False), ((10, 90), False),
             ((10, 270), True), ((7, 0), True), ((7, 90), True), ((None, 0), False)]
    for (kn, fr), ok in cases:
        if (not a.wind_problems(kn, fr)) != ok:
            wrong.append("wind %s from %s: %s" % (kn, fr, a.wind_problems(kn, fr)))
    if a.hours_problem(NOON) or not a.hours_problem(datetime.datetime(2026, 10, 29, 21, 0)):
        wrong.append("hours")
    # the instructor tool
    dr = ptarea.Draft(None)
    try:
        dr.add_gps(tel(gps_ok=False))
        wrong.append("a corner from an untrusted GPS")
    except ValueError:
        pass
    for p in area_doc()["polygon"][:3]:
        dr.add(*p)
    dr.add_gps(tel(**at_m(-440, -870)))
    dr.set_ramp("top", *area_doc()["ramp_top"])
    dr.set_ramp("bottom", *area_doc()["ramp_bottom"])
    drv = login(st, "ann")                                       # an OBSERVER: no 2FA, not a CAPTAIN
    for sess, what in ((None, "no login"), (drv, "a trainee")):
        try:
            ptarea.save(st, sess, dr)
            wrong.append("saved by %s" % what)
        except simguard.Refused:
            pass
    saved = ptarea.save(st, exa, dr)
    back = ptarea.load(st)
    if back.placeholder or back.doc.get("set_by") != "exa" or back.doc.get("not_for_navigation") is not True or \
            "boat GPS" not in back.doc.get("source", []) or len(back.polygon) != 4 or saved.path != back.path:
        wrong.append("saved area %s" % {k: back.doc.get(k) for k in ("placeholder", "set_by", "source")})
    if not [e for e in st.entries() if e.get("kind") == "area" and e.get("user") == "exa"]:
        wrong.append("no audit entry")
    tmp = tempfile.mkdtemp()
    gj = os.path.join(tmp, "bay.geojson")
    with open(gj, "w", encoding="utf-8") as f:
        json.dump({"type": "FeatureCollection", "features": [{"type": "Feature", "geometry": {
            "type": "Polygon", "coordinates": [[[24.03, 37.79], [24.05, 37.79], [24.05, 37.77], [24.03, 37.77],
                                                [24.03, 37.79]]]}}]}, f)
    d2 = ptarea.Draft(None)
    d2.load_geojson(gj)
    if d2.polygon != [[37.79, 24.03], [37.79, 24.05], [37.77, 24.05], [37.77, 24.03]]:
        wrong.append("geojson %s" % d2.polygon)
    for bad in ("COM4", tmp, os.path.join(tmp, "x.txt")):
        try:
            ptarea.Draft(None).load_geojson(bad)
            wrong.append("geojson from %r" % bad)
        except (ValueError, OSError):
            pass
    report("P3 test area + instructor tool", not wrong, "; ".join(wrong) or
           "placeholder refused; inside / outside; wind limit + north / ramp-right blocked from 8 kn; hours; save "
           "only by a CAPTAIN's 2FA login (audit entry, set_by, not_for_navigation); .geojson import; COM / folder "
           "refused; boat GPS point only with a trusted fix")


# ---------------------------------------------------------------- P4
def p4():
    wrong = []
    st = store()
    exa = people(st)
    good = ptest.Test(st, real_area(), "upgrade", "ann", exa)
    base = good.start_problems(tel(), 0.2, TICKS, 5, 270, NOON)
    if base:
        wrong.append("the good case refused: %s" % base)
    ann = login(st, "ann")
    cap2 = login(st, "cap2")
    nofa = auth.Session("exa", False, st.auth.id)               # a password-only session (no 2FA)
    cases = [
        ("placeholder area", dict(area=ptarea.Area(area_doc(True), "x")), "PLACEHOLDER"),
        ("outside the area", dict(t=tel(**at_m(5000))), "outside the test area"),
        ("UNCREWED", dict(t=tel(profile=2, profile_name="UNCREWED")), "CREWED"),
        ("FIRST TEST", dict(t=tel(first_test=True)), "FIRST TEST"),
        ("not STANDBY", dict(t=tel(mode=2)), "STANDBY"),
        ("power raised", dict(t=tel(power_fwd=50.0, power_raised=True)), "start power limit"),
        ("sea state", dict(t=tel(sea_state=4)), "calm water"),
        ("slamming", dict(t=tel(slam_state="YELLOW")), "calm water"),
        ("no calm tick", dict(ticks=dict(TICKS, calm=False)), "calm water"),
        ("examiner not on board", dict(ticks=dict(TICKS, on_board=False)), "on board"),
        ("wind not confirmed", dict(ticks=dict(TICKS, wind=False)), "typed wind"),
        ("wind too strong", dict(wind=(14, 270)), "limit"),
        ("north wind", dict(wind=(9, 10)), "north"),
        ("ramp right side", dict(wind=(9, 90)), "right side"),
        ("hours", dict(when=datetime.datetime(2026, 10, 29, 6, 30)), "hours"),
        ("stale telemetry", dict(age=9.0), "fresh telemetry"),
        ("no telemetry", dict(t=None), "fresh telemetry"),
        ("no GPS", dict(t=tel(gps_ok=False)), "GPS"),
        ("examiner = trainee", dict(trainee="exa"), "himself"),
        ("examiner not a CAPTAIN", dict(session=ann, trainee="drv"), "not a valid CAPTAIN"),
        ("examiner without 2FA", dict(session=nofa), "2FA"),
        ("not logged in", dict(session=None), "log in"),
        ("exam not passed", dict(trainee="cap2", kind="check_ride", session=exa), None),
        ("online", dict(env={"TRYGONS_ACADEMY_ONLINE": "1"}), "online"),
    ]
    for name, c, want in cases:
        tst = ptest.Test(st, c.get("area", real_area()), c.get("kind", "upgrade"), c.get("trainee", "ann"),
                         c["session"] if "session" in c else exa, env=c.get("env"))
        w = c.get("wind", (5, 270))
        probs = tst.start_problems(c["t"] if "t" in c else tel(), c.get("age", 0.2), c.get("ticks", TICKS), w[0],
                                   w[1], c.get("when", NOON))
        if want is None:                                         # (a valid CAPTAIN on a check ride: allowed)
            if probs:
                wrong.append("%s: %s" % (name, probs))
            continue
        if not any(want in p for p in probs):
            wrong.append("%s not refused (%s)" % (name, probs))
    # the trainee's own gates: exam + lessons
    st.add("new")
    t2 = ptest.Test(st, real_area(), "upgrade", "new", exa)
    pr = t2.start_problems(tel(), 0.2, TICKS, 5, 270, NOON)
    if not any("exam" in p for p in pr) or not any("lessons" in p for p in pr):
        wrong.append("trainee gates %s" % pr)
    try:
        t2.start(tel(), 0.2, TICKS, 5, 270, NOON)
        wrong.append("start() ran despite the problems")
    except simguard.Refused:
        pass
    if t2.state != "ready" or t2.recording:
        wrong.append("a refused test started")
    _ = cap2
    report("P4 every start condition refuses and says why", not wrong, "; ".join(wrong) or
           "%d conditions refused one by one (area, CREWED, FIRST TEST, power limit, calm water, examiner on board, "
           "wind, hours, telemetry, GPS, examiner rules, exam, lessons, online); start() refuses too" % (len(cases) + 1))


# ---------------------------------------------------------------- P5
def run_to_seen(tst, app, now):
    """Feed the telemetry of a good DRIVER practical test (leave, HDG LOCK + 10, takeover, alarm, return)."""
    seq = [tel(**at_m(20)), tel(mode=1, mode_name="MANUAL", **at_m(80)), tel(mode=1, **at_m(200)),
           tel(mode=2, target=90.0, **at_m(260)), tel(mode=2, target=100.0, **at_m(300)),
           tel(mode=1, **at_m(320)), tel(mode=2, target=100.0, **at_m(330))]
    for t in seq:
        now += 0.5
        tst.update(t, 0.1, now)
    return now


def p5():
    wrong = []
    st = store()
    exa = people(st)
    app = FakeApp()
    view = linkview.LinkView(app)
    tst = ptest.Test(st, real_area(), "upgrade", "ann", exa)
    now = 1000.0
    tst.start(tel(), 0.1, TICKS, 5, 270, NOON, now=now)
    if tst.target != "DRIVER" or [x["id"] for x in tst.tasks] != ptest.config()["kinds"]["upgrade_DRIVER"]:
        wrong.append("tasks %s -> %s" % ([x["id"] for x in tst.tasks], tst.target))
    if tst.approve("leave_harbour"):
        wrong.append("an unseen task approved")
    if tst.final_request(view, now) is not None or app.sent:
        wrong.append("final before the approvals")
    now = run_to_seen(tst, app, now)
    seen = {x["id"] for x in tst.tasks if x["state"] == "seen"}
    if seen != {"leave_harbour", "hdg_lock_change", "takeover"}:
        wrong.append("seen %s" % sorted(seen))
    # the alarm: late first, then in time
    if not tst.trigger_alarm(now):
        wrong.append("alarm refused while driving")
    now += 20.0
    tst.update(tel(mode=2, **at_m(330)), 0.1, now)
    if tst.task("alarm_reaction")["state"] != "todo" or "STANDBY" not in tst.task("alarm_reaction")["note"]:
        wrong.append("late alarm %s" % tst.task("alarm_reaction"))
    tst.trigger_alarm(now)
    now += 4.0
    tst.update(tel(mode=0, **at_m(330)), 0.1, now)
    if tst.task("alarm_reaction")["state"] != "seen":
        wrong.append("alarm in time not seen: %s" % tst.task("alarm_reaction"))
    for t in (tel(mode=1, **at_m(100)), tel(mode=1, **at_m(30)), tel(mode=0, **at_m(10))):
        now += 1.0
        tst.update(t, 0.1, now)
    if tst.task("return")["state"] != "seen":
        wrong.append("return not seen")
    tst.audit(["> HDG LOCK", "< ok"])
    for x in tst.tasks:
        if not tst.approve(x["id"]):
            wrong.append("approve %s" % x["id"])
    if any(app.sent):
        wrong.append("something was sent before the final approval: %s" % app.sent)
    # final: LATE -> not passed
    tok = tst.final_request(view, now)
    if not tok or app.sent != [(proto.T_CONFIRM_ONLY, proto.p_confirm_only(tok), "confirm-only (practical test)")]:
        wrong.append("final request %s" % app.sent)
    tst.update(tel(co_state=3, co_token=tok, **at_m(10)), 0.1, now + 11)
    if tst.state != "running" or tst.confirm["state"] == "confirmed":
        wrong.append("LATE passed (%s)" % tst.state)
    tok2 = tst.final_request(view, now + 12)
    tst.update(tel(co_state=2, co_token=(tok2 + 1) & 0xFFFF or 1, **at_m(10)), 0.1, now + 13)
    if tst.state == "passed":
        wrong.append("another token passed")
    tst.update(tel(co_state=2, co_token=tok2, **at_m(10)), 0.1, now + 14)
    if tst.state != "passed":
        wrong.append("CONFIRMED not passed (%s)" % tst.state)
    a = st.grant_practical(tst.result(), exa)
    if a.level != "DRIVER" or a.effective_level() != "DRIVER":
        wrong.append("level %s" % a.level)
    kinds = [e["kind"] for e in st.entries() if e.get("user") == "ann"][-2:]
    if kinds != ["practical", "level"]:
        wrong.append("logbook %s" % kinds)
    rec = [json.loads(ln) for ln in open(tst.recording, encoding="utf-8")]
    kinds = {r["kind"] for r in rec}
    if kinds != {"sample", "event", "audit"} or not any(r.get("what") == "confirmed" for r in rec):
        wrong.append("recording %s" % sorted(kinds))
    # leaving the area: alert + stopped, nothing sent
    app2 = FakeApp()
    t3 = ptest.Test(st, real_area(), "upgrade", "drv", exa)
    t3.start(tel(), 0.1, TICKS, 5, 270, NOON, now=2000.0)
    alert = t3.update(tel(mode=1, **at_m(3000)), 0.1, 2001.0)
    if t3.state != "stopped" or not alert or "left the test area" not in alert or app2.sent:
        wrong.append("leaving the area: %s %s" % (t3.state, alert))
    if t3.approve("leave_harbour") or t3.final_request(linkview.LinkView(app2), 2002.0) is not None or app2.sent:
        wrong.append("a stopped test went on")
    fails = [e for e in st.entries() if e.get("user") == "drv" and e.get("kind") == "practical"]
    if not fails or fails[-1].get("passed") is not False:
        wrong.append("the stop is not logged")
    t4 = ptest.Test(st, real_area(), "upgrade", "drv", exa)
    t4.start(tel(), 0.1, TICKS, 5, 270, NOON, now=3000.0)
    if not t4.abort() or t4.state != "aborted" or t4.trigger_alarm(3001.0):
        wrong.append("abort")
    report("P5 a whole practical test on made-up telemetry", not wrong, "; ".join(wrong) or
           "tasks seen only from the telemetry, no approval before; alarm late / in time; CONFIRM_ONLY only after "
           "every approval, LATE + another token = no pass, CONFIRMED = DRIVER (logged); the ride recorded; "
           "leaving the area = alert + STOPPED with nothing sent; abort")


# ---------------------------------------------------------------- P6
def good_result(st, exa, trainee="ann"):
    tst = ptest.Test(st, real_area(), "upgrade", trainee, exa)
    tst.start(tel(), 0.1, TICKS, 5, 270, NOON, now=100.0)
    for x in tst.tasks:
        x["state"] = "approved"
    tst.token = 777
    tst.confirm = {"state": "confirmed", "token": 777}
    tst.state = "passed"
    return tst.result()


def p6():
    wrong = []
    st = store()
    exa = people(st)
    cap2 = login(st, "cap2")
    ann = login(st, "ann")
    base = good_result(st, exa)
    forged = [
        ("no CONFIRM", dict(confirm_only={"state": "late", "token": 777}), exa, None),
        ("wrong token", dict(confirm_only={"state": "confirmed", "token": 778}), exa, None),
        ("another examiner's result", {}, cap2, None),
        ("a task not approved", dict(tasks=[dict(base["tasks"][0], state="seen")] + base["tasks"][1:]), exa, None),
        ("not passed", dict(passed=False), exa, None),
        ("examiner = trainee", dict(trainee="exa"), exa, None),
        ("two levels up", dict(target="CAPTAIN"), exa, None),
        ("check ride for a DRIVER", dict(kind="check_ride", trainee="drv"), exa, None),
        ("unknown kind", dict(kind="admiral"), exa, None),
        ("online", {}, exa, {"TRYGONS_ACADEMY_ONLINE": "1"}),
        ("no 2FA", {}, auth.Session("exa", False, st.auth.id), None),
        ("a trainee's session", dict(examiner="ann"), ann, None),
        ("a session of another store", {}, auth.Session("exa", True, "other"), None),
    ]
    for name, change, sess, env in forged:
        r = dict(base, **change)
        try:
            st.grant_practical(r, sess, env=env)
            wrong.append("%s accepted" % name)
        except simguard.Refused:
            pass
    if st.accounts["ann"].level != "OBSERVER":
        wrong.append("ann changed to %s" % st.accounts["ann"].level)
    # an expired examiner
    st.accounts["exa"].expiry = datetime.date.today() - datetime.timedelta(days=1)
    try:
        st.grant_practical(base, exa)
        wrong.append("an expired examiner accepted")
    except simguard.Refused:
        pass
    st.accounts["exa"].expiry = datetime.date.today() + datetime.timedelta(days=300)
    # set_level can't raise once a CAPTAIN instructor exists; lowering works
    for u, lv in (("ann", "DRIVER"), ("drv", "CAPTAIN"), ("cap2", "CAPTAIN")):
        try:
            st.set_level(u, lv, datetime.date.today() + datetime.timedelta(days=900))
            wrong.append("set_level raised %s to %s" % (u, lv))
        except simguard.Refused:
            pass
    st.set_level("cap2", "DRIVER")
    if st.accounts["cap2"].level != "DRIVER":
        wrong.append("lowering refused")
    a = st.grant_practical(base, exa)
    if a.level != "DRIVER":
        wrong.append("the good result refused")
    report("P6 grant_practical refuses forged results; set_level never raises", not wrong, "; ".join(wrong) or
           "%d forged / incomplete results + an expired examiner refused; set_level only lowers once a CAPTAIN "
           "instructor exists; the real result -> DRIVER" % len(forged))


# ---------------------------------------------------------------- P7 (the real simulator)
def free_port_pair():
    for _ in range(50):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        t = socket.socket()
        try:
            t.bind(("127.0.0.1", p + 2))
            t.close()
            return p
        except OSError:
            t.close()
    raise RuntimeError("no free port pair")


class SimStation(object):
    """TEST ONLY: plays the ground station (telemetry in, cmd() out) AND the person on board (the simulated
    joystick, the panel) against sim/autopilot_sim server on a free port."""

    def __init__(self, port):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.s.setblocking(False)
        self.rx = proto.Receiver()
        self.seq = 0
        self.t, self.t_rx, self._evlog, self.link = {}, 0.0, [], self
        self.acks, self.sent = [], []
        self.joy = {"x": 0.0, "y": 0.0, "confirm": False}

    def _send(self, ftype, payload=b""):
        self.seq = (self.seq + 1) & 0xFF
        self.s.sendall(proto.encode(ftype, self.seq, payload))
        return self.seq

    def cmd(self, ftype, payload, what):                   # the station's command path (LinkView uses it)
        self.sent.append(ftype)
        self.event("> " + what)
        return self._send(ftype, payload)

    def event(self, text):
        self._evlog = (self._evlog + [text])[-200:]

    def tick(self, n=1):
        for _ in range(n):
            self._send(proto.T_HEARTBEAT)
            j = self.joy
            self._send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, j["x"], j["y"], confirm=j["confirm"]))
            try:
                data = self.s.recv(65536)
            except (BlockingIOError, InterruptedError):
                data = b""
            for f in self.rx.frames_from(data):
                try:
                    if f.type == proto.T_TELEMETRY:
                        self.t.update(proto.decode_telemetry(f.payload))
                        self.t_rx = time.time()
                    elif f.type == proto.T_TELEM2:
                        self.t.update(proto.decode_telem2(f.payload))
                    elif f.type == proto.T_ACK:
                        self.acks.append(proto.decode_ack(f.payload))
                except (ValueError, KeyError, IndexError):
                    pass
            time.sleep(0.05)
        return self.t

    def wait(self, cond, limit=15.0):
        end = time.time() + limit
        while time.time() < end:
            self.tick()
            if cond(self.t):
                return True
        return False

    def last_ack(self, ftype):
        for a in reversed(self.acks):
            if a.get("cmd") == ftype:
                return a
        return None

    def confirm(self):
        self.joy["confirm"] = True
        self.tick(4)
        self.joy["confirm"] = False
        self.tick(4)

    def button(self, b):
        self._send(proto.T_SIM_BUTTON, proto.p_sim_button(b))
        self.tick(3)


def snap(t):
    return {k: t.get(k) for k in ("mode", "power_fwd", "power_rev", "power_raised", "throttle", "gear", "alarm_bits",
                                  "alarm_bits2", "target", "prog_ok")}


def p7():
    exe = os.path.join(REPO, "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.isfile(exe):
        report("P7 the real simulator", False, "sim/autopilot_sim not built")
        return
    wrong = []
    port = free_port_pair()
    d = tempfile.mkdtemp(prefix="trygons_ptsim_")
    p = subprocess.Popen([exe, "server", str(port), "1"], cwd=d, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        end = time.time() + 10
        while True:
            try:
                socket.create_connection(("127.0.0.1", port + 2), timeout=0.5).close()
                break
            except OSError:
                if time.time() > end:
                    raise
                time.sleep(0.2)
        st = SimStation(port)
        view = linkview.LinkView(st)
        ready = st.wait(lambda t: t.get("profile") == 1 and t.get("joy_status") == 2 and t.get("mode") == 0, 20)
        if not ready:
            wrong.append("simulator not ready (profile %s, joystick %s)" % (st.t.get("profile"), st.t.get("joy_status")))
        st.tick(10)
        before = snap(st.t)
        view.confirm_only(0x2A2A)
        pend = st.wait(lambda t: t.get("co_state") == 1 and t.get("co_token") == 0x2A2A, 3)
        ack = st.last_ack(proto.T_CONFIRM_ONLY)
        st.confirm()
        conf = st.wait(lambda t: t.get("co_state") == 2 and t.get("co_token") == 0x2A2A, 3)
        st.tick(10)
        after = snap(st.t)
        if not (pend and ack and ack.get("ok") and conf):
            wrong.append("round trip: pending %s, ack %s, confirmed %s" % (pend, ack, conf))
        if before != after:
            wrong.append("something changed: %s -> %s" % (before, after))
        # a power raise waiting for its CONFIRM: the question is refused and the raise stays pending
        st._send(proto.T_POWER_CODE, proto.p_power_code(0))
        st.wait(lambda t: t.get("power_code", 0) != 0, 3)
        st._send(proto.T_POWER_RAISE, proto.p_power_raise(0, st.t.get("power_code", 0)))
        st.wait(lambda t: t.get("confirm_kind") == 3, 3)
        view.confirm_only(0x0B0B)
        st.tick(6)
        a2 = st.last_ack(proto.T_CONFIRM_ONLY)
        kept = st.t.get("confirm_kind") == 3 and st.t.get("co_token") == 0x2A2A
        if not a2 or a2.get("ok") or not kept:
            wrong.append("during a power raise: ack %s, raise kept %s" % (a2, kept))
        st._send(proto.T_POWER_LOWER)
        st.tick(10)
        # HDG LOCK + a pending question: the onboard joystick still takes over
        st.button(proto.BTN_HDG_LOCK)
        hdg = st.wait(lambda t: t.get("mode") == 2, 4)
        view.confirm_only(0x0C0C)
        st.wait(lambda t: t.get("co_token") == 0x0C0C and t.get("co_state") == 1, 3)
        st.joy["x"] = 0.7
        took = st.wait(lambda t: t.get("mode") == 1, 4)
        st.joy["x"] = 0.0
        st.tick(10)
        unanswered = st.t.get("co_state") in (1, 3, 4)            # (never CONFIRMED by the takeover)
        if not (hdg and took and unanswered):
            wrong.append("takeover with a question pending: HDG LOCK %s, MANUAL %s, state %s" % (
                hdg, took, st.t.get("co_state")))
        st.button(proto.BTN_STANDBY)
        st.wait(lambda t: t.get("mode") == 0, 6)
        st.tick(20)
        # no answer within 10 s = LATE; a press after it does nothing
        view.confirm_only(0x0D0D)
        late = st.wait(lambda t: t.get("co_token") == 0x0D0D and t.get("co_state") == 3, 14)
        b2 = snap(st.t)
        st.confirm()
        st.tick(6)
        if not late or snap(st.t) != b2 or st.t.get("co_state") != 3:
            wrong.append("late: %s, after a press %s" % (late, snap(st.t)))
        if set(st.sent) != {proto.T_CONFIRM_ONLY}:
            wrong.append("LinkView sent %s" % st.sent)
        st.s.close()
    finally:
        p.kill()
        try:
            p.wait(5)
        except Exception:
            pass
    report("P7 the real simulator: CONFIRM_ONLY changes nothing", not wrong, "; ".join(wrong) or
           "pending -> onboard CONFIRM -> CONFIRMED (TELEM2 R, the token), mode / limits / throttle / gear / alarms "
           "unchanged; refused while a power raise waits (kept); the joystick still takes over in HDG LOCK with a "
           "question pending; no answer in 10 s = LATE, a later press does nothing")


# ---------------------------------------------------------------- P8
def p8():
    wrong = []
    import text
    used = set()
    for p in modules() + [os.path.join(ACADEMY, "certificate.py")]:
        used |= set(re.findall(r"\bT\(\s*[\"']([A-Za-z0-9_.\-]+)[\"']", open(p, encoding="utf-8").read()))
    used |= {"pt.kind_%s" % k for k in ptest.KINDS} | {"pt.tick_%s" % k for k in ("calm", "on_board", "wind")}
    used |= {"pt.state_%s" % s for s in ("todo", "seen", "approved", "failed")}
    used |= {"pt.st_%s" % s for s in ("ready", "running", "confirming", "passed", "stopped", "aborted")}
    used |= {"pt.col_%s" % c for c in ("task", "state", "note")} | {"ar.tap_mode_%s" % w for w in ("corner", "top", "bottom")}
    used |= {"ar.ramp_top", "ar.ramp_bottom", "pt.user", "pt.password", "pt.code", "pt.examiner", "ar.who"}
    src = open(os.path.join(HERE, "ptest_app.py"), encoding="utf-8").read()
    used |= set(re.findall(r'\("((?:pt|ar)\.[a-z_0-9]+)"', src))
    el, en = text.load("el"), text.load("en")
    miss = sorted(k for k in used if k not in el or k not in en)
    if miss:
        wrong.append("texts missing %s" % miss)
    if el.get("pt.banner") != "ΠΡΑΚΤΙΚΗ ΕΞΕΤΑΣΗ – ΠΡΑΓΜΑΤΙΚΟ ΣΚΑΦΟΣ" or en.get("pt.banner") != "PRACTICAL TEST – REAL BOAT":
        wrong.append("banner text")
    gsrc = open(os.path.join(GS, "practicalmenu.py"), encoding="utf-8").read()
    if "linkview.LinkView(app)" not in gsrc or "app.link" in gsrc:
        wrong.append("practicalmenu hands over more than LinkView(app)")
    msrc = open(os.path.join(GS, "academymenu.py"), encoding="utf-8").read()
    if "practicalmenu.open_practical(app)" not in msrc or "practicalmenu.open_area(app)" not in msrc:
        wrong.append("menu entries")
    gl = {lc: open(os.path.join(GS, "lang", lc + ".txt"), encoding="utf-8").read() for lc in ("el", "en")}
    for k in ("menu.academy_practical", "menu.academy_area", "acad.practical_opened", "confirm.11"):
        for lc in gl:
            if not re.search(r"^%s = " % re.escape(k), gl[lc], re.M):
                wrong.append("groundstation %s.txt: %s" % (lc, k))
    # the windows on a fake station
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
    except Exception as e:
        report("P8 station menu + windows", not wrong, "; ".join(wrong) or "texts + menu checked; no display (%s)" % e)
        return
    try:
        import ptest_app
        st = store()
        people(st)
        ptarea.save(st, login(st, "exa"), _draft())
        app = FakeApp()
        app.t, app.t_rx = tel(), time.time()
        clock = [time.time()]
        w = ptest_app.PracticalWindow(tk.Toplevel(root), linkview.LinkView(app), st, now=lambda: clock[0],
                                      when=lambda: NOON)
        if w.top.pack_slaves()[0] is not w.banner or w.banner.cget("text") != "ΠΡΑΚΤΙΚΗ ΕΞΕΤΑΣΗ – ΠΡΑΓΜΑΤΙΚΟ ΣΚΑΦΟΣ":
            wrong.append("practical window banner")
        w.v_trainee.set("ann")
        w.start()
        if w.test is not None:
            wrong.append("started without a login")
        w.login.v_user.set("exa")
        w.login.v_pw.set("longpassword1")
        w.login.v_code.set(auth.totp(st.auth.get("exa")["totp"]))
        w.login.login()
        w.v_wind.set("5")
        w.v_from.set("270")
        probs = w.check()
        if not probs or not any("examiner must be on board" in p for p in probs):
            wrong.append("check without ticks: %s" % probs)
        for v in w.ticks.values():
            v.set(True)
        if w.check():
            wrong.append("check with ticks: %s" % w.check())
        w.start()
        if not w.test or w.test.state != "running":
            wrong.append("not started: %s" % w.msg.cget("text"))
        else:
            for t in (tel(**at_m(200)), tel(mode=2, target=90.0, **at_m(260)), tel(mode=2, target=101.0, **at_m(300)),
                      tel(mode=1, **at_m(300)), tel(mode=2, **at_m(300))):
                clock[0] += 0.5
                app.t, app.t_rx = t, clock[0]
                w.step()
            w.alarm()
            clock[0] += 2.0
            app.t, app.t_rx = tel(mode=0, **at_m(300)), clock[0]
            w.step()
            for t in (tel(mode=1, **at_m(50)), tel(mode=0, **at_m(10))):
                clock[0] += 0.5
                app.t, app.t_rx = t, clock[0]
                w.step()
            w.final()
            if app.sent:
                wrong.append("final sent before the approvals")
            for x in w.test.tasks:
                w.tasks.selection_set(x["id"])
                w.approve()
            w.final()
            tok = w.test.token
            clock[0] += 1.0
            app.t, app.t_rx = tel(co_state=2, co_token=tok, **at_m(10)), clock[0]
            w.step()
            if w.test.state != "passed" or not w.granted or st.accounts["ann"].level != "DRIVER":
                wrong.append("window flow: %s, %s" % (w.test.state, w.msg.cget("text")))
            if [s[0] for s in app.sent] != [proto.T_CONFIRM_ONLY]:
                wrong.append("window sent %s" % app.sent)
        w.close()
        aw = ptest_app.AreaWindow(tk.Toplevel(root), linkview.LinkView(app), st)
        if aw.top.pack_slaves()[0] is not aw.banner:
            wrong.append("area window banner")
        if aw.save() is not None:
            wrong.append("area saved without a login")
        n0 = len(aw.draft.polygon)
        app.t = tel(**at_m(400, 300))
        aw.gps_point()

        class Ev(object):
            x, y = 100, 100
        aw.on_tap(Ev())
        if len(aw.draft.polygon) != n0 + 2:
            wrong.append("area tool: GPS point + tap = %d corners" % len(aw.draft.polygon))
        aw.close()
    finally:
        root.destroy()
    report("P8 station menu + windows", not wrong, "; ".join(wrong) or
           "menu entries through practicalmenu (LinkView only); texts in el + en; both windows: banner first; a whole "
           "test through the window's buttons on a fake station -> DRIVER with exactly one CONFIRM_ONLY sent; the area "
           "tool adds a boat-GPS point + a map tap, refuses to save without a login")


def _draft():
    dr = ptarea.Draft(None)
    for pnt in area_doc()["polygon"]:
        dr.add(*pnt)
    dr.set_ramp("top", *area_doc()["ramp_top"])
    dr.set_ramp("bottom", *area_doc()["ramp_bottom"])
    return dr


if __name__ == "__main__":
    for fn in (p1, p2, p3, p4, p5, p6, p7, p8):
        try:
            fn()
        except Exception as e:                                  # a crash = a failure, the others still run
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "crashed: %r" % e)
    print("PRACTICAL TESTS: %d/%d PASS" % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)
