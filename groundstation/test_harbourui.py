"""Harbour autonomy on the ground station (CLAUDE.md Z, step HX6): supervision + teach and repeat. DISPLAY + requests only.

  HU1  symbols: the COLOUR by source (theme.HARB_SRC), the SHAPE by steer_ok (filled diamond / hollow circle), the
       confidence in % - drawn on the map canvas (harbourmap.draw); old data (> 3 s) is not drawn
  HU2  the status model: RUN / SLOW / HOLD / DONE / OFF, the cap, the corridor offset, the planner's reason line in
       the screen language, unsure flags, no picture = said
  HU3  geometry: the corridor edges stay half_m from the route, the planned path = the planner's heading at its cap, no
       path when the planner holds
  HU4  teach: a hand-driven exit -> a route with <= 50 waypoints (also from a 3000-point wiggly track), the ends cut by
       cut_m, legs >= 12 m, leg speeds <= 5 kn, the last waypoint a STOP; too short = refused
  HU5  refusal reasons of the local check (few / many / corridor / waypoint / leg) + the PROG_BEGIN flag byte
  HU6  the quay warning: first / last waypoint < 15 m from the shore, and "unknown" is a warning too
  HU7  route files: save / load round trip, the committed sample (not_for_navigation) listed + flagged, a bad file refused
  HU8  alarm table: SB_ALM2_HARBOUR_STOP has a level (WARNING), a screen text and a voice line in both languages
  HU9  READ-ONLY proof: the display modules import no sender / network module, send nothing at runtime (sockets
       trapped); the route window only calls the app's two request methods
  HU10 the real station: the panel «ΛΙΜΕΝΑΣ» (states, legend, buttons), STOP always sends the STANDBY request, TAKE OVER
       (CREWED = nothing sent, UNCREWED = the REMOTE request), START = the captain's YES then T_HARBOUR START, a copilot's
       START goes straight to the gateway (its ACCEPT), a NO sends nothing, a single operator is asked by cmd()
  HU11 the banner on both screens, never red; the panel fits 1366x768 .. 1920x1080 at 100 / 125 / 150 %, el + en;
       no colour / font literal in the new modules
  HU12 the route window on the real station: MAKE from a saved track, CHECK (gateway ack 129 refuses SEND, OK allows),
       SEND = the normal upload frames with the harbour flag, never a start; a TEST sample is never sent

Run:  python test_harbourui.py      (exit code 0 = all passed)
"""
import math
import os
import re
import socket
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_sar as TS  # noqa: E402  (its environment + Env + fakes: temp settings, sound off, the Rig on the simulator)
import alerts  # noqa: E402
import harbourmap  # noqa: E402
import harbourroute as hr  # noqa: E402
import harbourview as hv  # noqa: E402
import i18n  # noqa: E402
import proto  # noqa: E402
import theme  # noqa: E402
import ynbox  # noqa: E402
from i18n import T  # noqa: E402

results = TS.results
report = TS.report
LAT, LON = 37.82, 23.60
M = 111320.0


def trk(i, src="R", steer=True, conf=0.8, **kw):
    d = {"id": i, "lat": LAT + 0.0003 * i, "lon": LON + 0.0002 * i, "speed": 1.5, "course": 90.0, "cls": "vessel",
         "conf": conf, "src": src, "steer_ok": steer, "static": False, "confirmed": True, "len_m": 8.0, "sd_m": 2.0,
         "mmsi": None}
    d.update(kw)
    return d


def view(tracks=(), state="RUN", **plan):
    p = {"state": 1 if state == "RUN" else 2 if state == "SLOW" else 3 if state == "HOLD" else 4 if state == "DONE" else 0,
         "state_name": state, "why": 0, "kn": 4.0, "corr_m": 10.0, "xte_m": 1.5, "route": True, "entry_ok": True,
         "lease": True, "grace": False, "grace_left_s": 0.0, "max_kn": 5.0, "stat_fresh": True, "stop": False,
         "heading": 90.0, "cap_kn": 4.0, "why_key": "harb.why_clear", "text_el": "ΔΙΑΔΡΟΜΟΣ ΕΛΕΥΘΕΡΟΣ",
         "text_en": "ROUTE CLEAR", "colregs": [], "turn": False, "unsure": [], "hflags": 0}
    p.update(plan)
    return {"t": "harbour_view", "t_s": time.time(), "tracks": list(tracks), "plan": p, "_rx": time.time()}


def gwh(enabled=True, **kw):
    h = {"enabled": enabled, "sim": True, "sensors": {"radar": {"state": "OK"}, "lidar": {"state": "OK"}},
         "tracker": {"unsure": []}, "planner": {"stat_fresh": True}}
    h.update(kw)
    return h


def wiggle(n=600, step_m=1.0, noise=0.4, kn=3.0, seed=1):
    """A hand-driven exit: a gentle S-curve at 1 Hz with GPS noise. -> [(t, lat, lon, kn, hdg)]."""
    import random
    rnd = random.Random(seed)
    pts = []
    for i in range(n):
        x = i * step_m
        y = 25.0 * math.sin(i / 120.0)
        pts.append((1000.0 + i, LAT + (y + rnd.gauss(0, noise)) / M,
                    LON + (x + rnd.gauss(0, noise)) / (M * math.cos(math.radians(LAT))), kn, 90.0))
    return pts


# --- pure tests -----------------------------------------------------------------------------------------------------
def hu1(root):
    import tkinter as tk
    cv = tk.Canvas(root, width=600, height=400)

    class App:
        pass
    a = App()
    a.t = {"est_lat": LAT, "est_lon": LON, "pos_source": 1}
    a.harbour_route = None
    a.harbour_preview = None
    a.harbour_model = {"running": True, "heading": 90.0, "plan_cap_kn": 4.0, "stop": False, "corr_m": 10.0}
    a.harbour_view = view([trk(1, "R", True), trk(2, "L", True), trk(3, "C", False), trk(4, "A", False),
                           trk(5, "R", True, static=True, speed=0.0)])
    P = lambda la, lo: ((lo - LON) * 60000.0 + 300, 200 - (la - LAT) * 90000.0)  # noqa: E731
    th = {"text": "#101010", "route": "#7a00a0"}
    i18n.set_lang("en", save=False)
    harbourmap.draw(cv, P, a, th, ("Arial", 8), 600, 400)
    items = [i for i in cv.find_withtag("harb_track") if cv.type(i) in ("polygon", "oval", "rectangle")]
    diam = [i for i in items if cv.type(i) == "polygon"]
    circ = [i for i in items if cv.type(i) == "oval"]
    fills = {cv.itemcget(i, "outline") for i in items}
    cols = {k: theme.HARB_SRC[k] for k in ("radar", "lidar", "camera", "ais")}
    labels = [cv.itemcget(i, "text") for i in cv.find_withtag("harb_label")]
    distinct = len(set(cols.values())) == 4 and not (set(cols.values()) & {theme.WARN, theme.CAUTION, theme.GOOD})
    ok_shapes = len(diam) == 3 and len(circ) == 2           # 3 range-confirmed (radar, lidar, static radar), 2 slow-only
    ok_cols = all(c in fills for c in cols.values())
    ok_hollow = all(cv.itemcget(i, "fill") == "" for i in circ) and all(cv.itemcget(i, "fill") != "" for i in diam)
    ok_labels = any("80%" in l and "confirmed" in l for l in labels) and any("camera" in l and "slow-only" in l for l in labels)
    a.harbour_view["_rx"] = time.time() - 10.0
    cv.delete("all")
    harbourmap.draw(cv, P, a, th, ("Arial", 8), 600, 400)
    old_drawn = len(cv.find_withtag("harb_track"))
    path = cv.find_withtag("harb_path")
    cv.destroy()
    report("HU1 symbols: colour by source (4 distinct, none a meaning colour), shape by steer_ok (filled diamond / hollow "
           "circle), the confidence in %, a track older than 3 s is not drawn",
           distinct and ok_shapes and ok_cols and ok_hollow and ok_labels and old_drawn == 0 and bool(path),
           "diamonds %d circles %d colours %s labels %s old drawn %d path %d" % (len(diam), len(circ), sorted(fills), labels[:2],
                                                                              old_drawn, len(path)))


def hu2():
    i18n.set_lang("en", save=False)
    out = {}
    for st in ("RUN", "SLOW", "HOLD", "DONE", "OFF"):
        out[st] = hv.model(gwh(), view(state=st), time.time(), "en")
    m = out["RUN"]
    ok1 = m["running"] and m["state"] == "RUN" and m["cap_kn"] == 4.0 and m["corr_m"] == 10.0 and m["xte_m"] == 1.5 and \
        m["why"] == "ROUTE CLEAR" and m["view_ok"] and m["entry_ok"]
    ok2 = out["SLOW"]["running"] and out["HOLD"]["running"] and not out["DONE"]["running"] and out["DONE"]["state"] == "DONE" \
        and out["OFF"]["state"] == "OFF" and not out["OFF"]["running"]
    el = hv.model(gwh(), view(), time.time(), "el")
    old = hv.model(gwh(planner={"stat_fresh": False}), view(), time.time() - 10.0 + 20.0, "en")   # a view from the future = fine
    old2 = hv.model(gwh(planner={"stat_fresh": False}), dict(view(), _rx=time.time() - 9.0), time.time(), "en")
    uns = hv.model(gwh(), view(unsure=["U1 range lost", "U2", "x", "y", "z", "w", "v"], hflags=0x06, grace=True,
                               grace_left_s=42.0), time.time(), "en")
    off = hv.model(None, None, time.time(), "en")
    ok3 = el["why"] == "ΔΙΑΔΡΟΜΟΣ ΕΛΕΥΘΕΡΟΣ" and not old2["view_ok"] and old2["n_tracks"] == 0 and old2["state"] == "OFF" \
        and len(uns["unsure"]) <= 6 and uns["degraded"] and uns["giveway"] and uns["grace"] and uns["grace_left_s"] == 42.0 \
        and off["enabled"] is False and off["state"] == "OFF"
    report("HU2 the status model: RUN / SLOW / HOLD running, DONE / OFF not, cap / corridor / offset / the planner's line in "
           "the screen language, unsure + degraded + give-way + grace, an old or missing picture = OFF / not shown",
           ok1 and ok2 and ok3, "run %s / states %s / el %s / old %s / flags %s" % (ok1, ok2, el["why"], old2["view_ok"], ok3))
    _ = old


def hu3():
    pts = [(LAT, LON), (LAT + 0.0005, LON + 0.0006), (LAT + 0.0010, LON + 0.0007), (LAT + 0.0016, LON + 0.0007)]
    left, right = hv.corridor_lines(pts, 10.0)

    def d_to_route(p):
        best = 1e9
        for a, b in zip(pts, pts[1:]):
            best = min(best, hr._seg_d(p, a, b))
        return best
    dl, dr = [d_to_route(p) for p in left], [d_to_route(p) for p in right]
    ok_edges = len(left) == len(right) == 4 and all(8.0 <= d <= 21.0 for d in dl + dr) and \
        sum(1 for d in dl + dr if abs(d - 10.0) < 0.6) >= 4
    path = hv.plan_path(LAT, LON, 90.0, 4.0)
    dpath = hr._dist(path[0], path[1]) if len(path) == 2 else -1
    ok_path = abs(dpath - 4.0 * 1852.0 / 3600.0 * 40.0) < 0.5 and abs(path[1][0] - LAT) < 1e-7 and path[1][1] > LON
    none = [hv.plan_path(LAT, LON, 90.0, 0.0), hv.plan_path(LAT, LON, None, 4.0), hv.plan_path(LAT, LON, float("nan"), 4.0)]
    one = hv.corridor_lines([(LAT, LON)], 10.0)
    report("HU3 geometry: the corridor edges lie half_m from the route (capped on a sharp corner), the planned path = the "
           "planner's heading at its cap for 40 s, no path for cap 0 / no heading / NaN",
           ok_edges and ok_path and none == [[], [], []] and one == ([], []),
           "edge distances L %s R %s, path %.1f m (want %.1f)" % ([round(d, 1) for d in dl], [round(d, 1) for d in dr], dpath,
                                                                    4.0 * 1852.0 / 3600.0 * 40.0))


def hu4():
    pts = wiggle(600)
    r = hr.from_track(pts, "daskalio", "exit")
    ok = r["route"] is not None
    w = r["route"]["wps"] if ok else []
    legs = [hr._dist((a["lat"], a["lon"]), (b["lat"], b["lon"])) for a, b in zip(w, w[1:])]
    first_d = hr._dist((pts[0][1], pts[0][2]), (w[0]["lat"], w[0]["lon"])) if w else 0
    last_d = hr._dist((pts[-1][1], pts[-1][2]), (w[-1]["lat"], w[-1]["lon"])) if w else 0
    ok1 = ok and 2 <= len(w) <= 50 and min(legs) >= 12.0 - 1e-6 and 11.0 <= first_d <= 18.0 and 11.0 <= last_d <= 18.0 and \
        all(1.0 <= x["kn"] <= 5.0 for x in w) and w[-1]["action"] == 2 and w[0]["action"] == 0 and \
        r["route"]["flags"] == 1 and r["route"]["corr_m"] == 10.0 and not hr.local_check(r["route"])
    big = hr.from_track(wiggle(3000, 1.0, 1.2, 4.0), "h", "big", {"tol_m": 0.5})
    ok2 = big["route"] is not None and len(big["route"]["wps"]) <= 50 and big["tol_m"] >= 0.5 and \
        not [c for c in hr.local_check(big["route"]) if c[0] == "many"]
    short = hr.from_track(wiggle(20), "h", "s")
    ok3 = short["route"] is None and short["why"] == "too_short"
    fast = hr.from_track(wiggle(600, kn=9.0), "h", "f")
    ok4 = all(x["kn"] <= 5.0 for x in fast["route"]["wps"])
    seg = hr.from_track(pts, "h", "seg", i0=100, i1=400)
    ok5 = seg["route"] is not None and hr._dist((pts[100][1], pts[100][2]), (seg["route"]["wps"][0]["lat"],
                                                                              seg["route"]["wps"][0]["lon"])) > 11.0
    report("HU4 teach: a hand-driven exit -> <= 50 waypoints, ends cut by 15 m, legs >= 12 m, speeds <= 5 kn, STOP at the end, "
           "harbour flag + corridor 10 m; 3000 noisy points still <= 50; a too-short part is refused; a chosen segment is used",
           ok1 and ok2 and ok3 and ok4 and ok5,
           "n %d legs min %.1f first %.1f last %.1f / big %d tol %.1f / short %s / fast %s seg %s" % (
               len(w), min(legs) if legs else -1, first_d, last_d, len(big["route"]["wps"]) if big["route"] else -1,
               big["tol_m"], short["why"], ok4, ok5))


def hu5():
    base = hr.from_track(wiggle(600), "h", "x")["route"]
    import copy
    r = copy.deepcopy(base)
    r["wps"] = r["wps"][:1]
    few = hr.local_check(r)
    r = copy.deepcopy(base)
    r["wps"] = (r["wps"] * 8)[:51]
    many = [k for k, _a in hr.local_check(r)]
    r = copy.deepcopy(base)
    r["corr_m"] = 2.0
    corr = [k for k, _a in hr.local_check(r)]
    r = copy.deepcopy(base)
    r["corr_m"] = float("nan")
    nan = [k for k, _a in hr.local_check(r)]
    r = copy.deepcopy(base)
    r["wps"][1]["kn"] = 12.0
    wp = [k for k, _a in hr.local_check(r)]
    r = copy.deepcopy(base)
    r["wps"][1]["lat"], r["wps"][1]["lon"] = r["wps"][0]["lat"], r["wps"][0]["lon"]
    leg = [k for k, _a in hr.local_check(r)]
    ok_keys = few[0][0] == "few" and "many" in many and "corr" in corr and "corr" in nan and "wp" in wp and "leg" in leg
    texts = []
    for lang in ("el", "en"):
        i18n.set_lang(lang, save=False)
        for k, a in (few + [("many", {"n": 51, "max": 50}), ("corr", {"m": 2}), ("wp", {"n": 2}), ("leg", {"n": 1, "m": "3"})]):
            texts.append(T("harb.rt_" + k, **a))
    ok_t = all(t and not t.startswith("harb.") and "{" not in t for t in texts)
    pb = proto.p_prog_begin(5, 0, 1)
    pb0 = proto.p_prog_begin(5, 0)
    import program
    fr = program.upload_frames(hr.to_program(base))
    ok_p = pb == bytes([5, 0, 1]) and pb0 == bytes([5, 0]) and fr[0][1][2] == 1 and fr[0][0] == proto.T_PROG_BEGIN and \
        program.upload_frames({"final": 0, "wps": [dict(base["wps"][0])]})[0][1] == bytes([1, 0])
    report("HU5 refusal reasons (few / many / corridor incl. NaN / waypoint / leg) with texts in both languages; PROG_BEGIN "
           "carries the harbour flag byte only for a harbour route", ok_keys and ok_t and ok_p,
           "keys %s texts %s proto %s" % (ok_keys, ok_t, ok_p))


def hu6():
    i18n.set_lang("en", save=False)
    r = hr.from_track(wiggle(600), "h", "q")["route"]
    w = r["wps"]
    far = hr.quay_warnings(r, lambda la, lo: 40.0)
    near = hr.quay_warnings(r, lambda la, lo: 10.0)
    unk = hr.quay_warnings(r, lambda la, lo: None)
    one = hr.quay_warnings(r, lambda la, lo: 8.0 if abs(la - w[0]["lat"]) < 1e-9 and abs(lo - w[0]["lon"]) < 1e-9 else 40.0)
    boom = hr.quay_warnings(r, lambda la, lo: 1 / 0)
    nofn = hr.quay_warnings(r, None)
    txt = [T("harb.rt_" + k, **a) for k, a in near + unk]
    ok = far == [] and [k for k, _a in near] == ["quay_start", "quay_end"] and \
        [k for k, _a in unk] == ["quay_unknown_start", "quay_unknown_end"] and [k for k, _a in one] == ["quay_start"] and \
        len(boom) == 2 and len(nofn) == 2 and all(t and "{" not in t for t in txt) and "10" in txt[0] and "15" in txt[0]
    report("HU6 the quay warning (M0-8): first / last waypoint under 15 m from the shore warn, a missing / broken chart is a "
           "warning too (never silently fine)", ok, "near %s unk %s one %s" % (near, unk, one))


def hu7():
    tmp = tempfile.mkdtemp(prefix="trygons_hr_")
    r = hr.from_track(wiggle(600), "Daskalio bay", "Exit 1")["route"]
    p = hr.save(r, tmp)
    back = hr.load(p)
    ok1 = back["wps"] == r["wps"] and back["harbour"] == "Daskalio bay" and p.endswith(os.path.join("Daskalio-bay", "Exit-1.json"))
    lst = hr.listing()
    samples = [x for x in lst if x[3]]
    ok2 = any(x[0] == "daskalio" and x[1] == "sample_exit" for x in samples)
    s = hr.load(samples[0][2]) if samples else {}
    ok3 = s.get("not_for_navigation") is True and s.get("placeholder") is True and not hr.local_check(s)
    bad = os.path.join(tmp, "bad.json")
    open(bad, "w").write("{not json")
    empty = os.path.join(tmp, "e.json")
    open(empty, "w").write('{"wps": []}')
    errs = 0
    for b in (bad, empty, os.path.join(tmp, "nope.json")):
        try:
            hr.load(b)
        except ValueError:
            errs += 1
    ig = open(os.path.join(HERE, "..", ".gitignore"), encoding="utf-8").read()
    ok4 = errs == 3 and "groundstation/routes/harbour/*" in ig and "!groundstation/routes/harbour/_samples/" in ig
    report("HU7 route files: save / load round trip per harbour, the committed Daskalio TEST sample (not_for_navigation, "
           "placeholder), bad files refused, user routes gitignored", ok1 and ok2 and ok3 and ok4,
           "%s %s %s %s" % (ok1, ok2, ok3, ok4))


def hu8():
    bit = 0x100 << 32
    code, lvl = alerts.ALARM_LEVELS.get(bit, (None, None))
    names = dict((b << alerts.WORD2, n) for b, n in proto.ALARMS2)
    ok_t = True
    for lang in ("el", "en"):
        for k in ("alarm.harbour_stop", "say.alarm.harbour_stop"):
            ok_t = ok_t and i18n.has(k, lang)
    rows = [b for b in alerts.ALL_ALARMS if b[0] == bit]
    report("HU8 alarm table: HARBOUR STOP (SB_ALM2_HARBOUR_STOP 0x100) is a WARNING (the boat stopped itself: a driver "
           "decision), with a screen text and a voice line in Greek and English",
           code == "harbour_stop" and lvl == alerts.WARNING and ok_t and names.get(bit) == "HARBOUR STOP" and bool(rows),
           "code %s level %s texts %s name %s" % (code, lvl, ok_t, names.get(bit)))


MODS = ("harbourview.py", "harbourmap.py", "harbourroute.py")
BANNED = re.compile(r"^\s*(import|from)\s+(gwclient|link|stations|socket|aurix|app|hud|driver|requests|urllib|http|"
                    r"subprocess|asyncio|serial)\b", re.M)


def hu9():
    import ast
    problems = []
    names_bad = {"send", "send_msg", "cmd", "p_harbour", "T_HARBOUR", "T_SET_MODE", "_send_cmd"}
    for fn in MODS + ("harbourpanel.py",):
        src = open(os.path.join(HERE, fn), encoding="utf-8").read()
        for m in BANNED.finditer(src):
            problems.append("%s: %s" % (fn, m.group(0).strip()))
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    if a.name.split(".")[0] in ("tkinter", "socket", "subprocess") and fn != "harbourpanel.py":
                        problems.append("%s imports %s" % (fn, a.name))
            elif isinstance(node, ast.ImportFrom) and (node.module or "").split(".")[0] in ("tkinter", "socket", "subprocess")                     and fn != "harbourpanel.py":
                problems.append("%s imports %s" % (fn, node.module))
            elif isinstance(node, (ast.Name, ast.Attribute)) and fn != "harbourpanel.py":     # (the window's own button method is called send)
                nm = node.id if isinstance(node, ast.Name) else node.attr
                if nm in names_bad:
                    problems.append("%s uses %s" % (fn, nm))
    pan = open(os.path.join(HERE, "harbourpanel.py"), encoding="utf-8").read()
    app_calls = set(re.findall(r"self\.app\.(\w+)\(", pan))
    allowed = {"harbour_check_request", "harbour_upload", "render"}
    if app_calls - allowed - {"mapwin"}:
        problems.append("the route window calls %s" % sorted(app_calls - allowed))
    if re.search(r"\.send(_msg)?\(", pan):
        problems.append("harbourpanel sends itself")
    # runtime: the modules run with every socket / send trapped
    trapped = []
    orig = socket.socket.connect

    def boom(*a, **k):
        trapped.append("connect")
        raise OSError("trapped")
    socket.socket.connect = boom
    try:
        hv.model(gwh(), view([trk(1)]), time.time(), "en")
        hv.corridor_lines([(LAT, LON), (LAT + 0.001, LON)], 10.0)
        hr.from_track(wiggle(100), "h", "n")
    finally:
        socket.socket.connect = orig
    # the proof is able to fail (negative control): a module that imports a sender is found
    neg = BANNED.search("import gwclient\n") is not None
    report("HU9 READ-ONLY proof: the harbour display / route modules import no sender or network module, contain no send "
           "call, the route window only calls the app's two request methods, nothing connects at runtime; the scan is able "
           "to fail", not problems and not trapped and neg, "problems %s trapped %s negative control %s" % (problems, trapped, neg))


# --- the real station ---------------------------------------------------------------------------------------------------
class Cap:
    """Records what the station sends (frames with payloads + gateway messages) and every question asked."""

    def __init__(self, env):
        self.env = env
        ap = env.ap
        self.frames, self.msgs, self.asked = [], [], []
        env.rig.gateway_like(True)
        seq = [100]

        def send(t, p=b"", **k):
            if t == 1:                                   # the link's own keep-alive, not a request
                return 0
            self.frames.append((t, bytes(p)))
            seq[0] += 1
            return seq[0]
        ap.link.send = send
        ap.link.send_msg = lambda m: self.msgs.append(m)
        self.answer = False
        ynbox.ask = lambda parent, title, text, **k: (self.asked.append((title, text)), self.answer)[1]
        ap.link.welcome = {"id": 3}

    def clear(self):
        self.frames.clear()
        self.msgs.clear()
        self.asked.clear()


def station(env, crew="captain", state="HOLD", entry_ok=True, running=None, single_op=False, licence=True, unc=False,
            tracks=(), **plan):
    ap = env.ap
    ap.single_op = single_op
    run = state in ("RUN", "SLOW", "HOLD") if running is None else running
    ap.gw_status = {"you": 3, "stations": [{"id": 3, "crew": crew, "licence": licence, "role": "HELM", "name": "x",
                                           "user": "x", "single_op": single_op, "local": False, "rtt_ms": 10, "tmode": "full"}],
                    "harbour": gwh(planner={"stat_fresh": True})}
    ap.harbour_view = view(tracks, state=state if run else "OFF", entry_ok=entry_ok, **plan)
    ap.t = dict(ap.t or {}, profile=proto.PROFILE_UNCREWED if unc else proto.PROFILE_CREWED)
    ap.harbour_asked = True
    ap.render(force=True)


def hu10(env):
    ap = env.ap
    cap = Cap(env)
    i18n.set_lang("en", save=False)
    station(env, state="RUN", tracks=[trk(1)])
    o = [x.get("tag") for x in ap.ov.ops]
    ok_panel = all(t in o for t in ("harb_title", "harb_state", "harb_corr", "harb_why", "harb_sensors", "harb_legend",
                                    "harb_stop", "harb_hold", "harb_take")) and "harb_start" not in o
    # STOP = the STANDBY request, whatever the state, no question
    for st in ("RUN", "SLOW", "HOLD", "DONE"):
        station(env, state=st)
        cap.clear()
        ap.harbour_stop()
        stops = [(t, p) for t, p in cap.frames if t == proto.T_SET_MODE]
        if len(stops) != 1 or stops[0][1][:1] != bytes([proto.MODE_STANDBY]) or cap.asked or len(cap.frames) != 1:
            ok_panel = False
    # HOLD state offers RESUME, not PAUSE
    station(env, state="HOLD")
    o = [x.get("tag") for x in ap.ov.ops]
    ok_resume = "harb_resume" in o and "harb_hold" not in o
    # TAKE OVER: CREWED = nothing sent; UNCREWED captain with a licence = the REMOTE request
    station(env, state="RUN")
    cap.clear()
    ap.harbour_take_over()
    crewed_sent = list(cap.frames)
    station(env, state="RUN", unc=True)
    cap.clear()
    ap.harbour_take_over()
    env.rig.pump(0.8)                                   # (the REMOTE request waits 0.4 s for the first stick packets)
    unc_sent = [t for t, _p in cap.frames]
    station(env, state="RUN", unc=True, licence=False)
    cap.clear()
    ap.harbour_take_over()
    nolic = list(cap.frames)
    ok_take = crewed_sent == [] and proto.T_REMOTE in unc_sent and nolic == []
    # START: the captain's own YES first; NO sends nothing; YES sends T_HARBOUR START with the corridor + max speed
    station(env, state="OFF", running=False, entry_ok=True)
    cap.clear()
    cap.answer = False
    ap.harbour_start()
    no_sent = list(cap.frames)
    asked_no = len(cap.asked) == 1
    cap.clear()
    cap.answer = True
    ap.harbour_start()
    st = [p for t, p in cap.frames if t == proto.T_HARBOUR]
    ok_start = no_sent == [] and asked_no and len(st) == 1 and st[0][:1] == bytes([proto.HARB_START]) and len(cap.asked) == 1 \
        and st[0][1] == 100 and st[0][2] == 50
    # not ready (entry_ok False) = refused with the reason, nothing sent, no question
    station(env, state="OFF", running=False, entry_ok=False)
    cap.clear()
    ap.harbour_start()
    ok_nr = cap.frames == [] and cap.asked == []
    o = [x for x in ap.ov.ops if x.get("tag") == "harb_start"]
    ok_nr = ok_nr and bool(o)
    # a copilot: no own YES; the request goes to the gateway (its ACCEPT: proto.is_driving_cmd) and is marked a driving command
    station(env, crew="copilot", state="OFF", running=False, entry_ok=True)
    cap.clear()
    cap.answer = True
    ap.harbour_start()
    cp = [p for t, p in cap.frames if t == proto.T_HARBOUR]
    ok_cp = len(cp) == 1 and cap.asked == [] and proto.is_driving_cmd(proto.T_HARBOUR, cp[0], set())
    hold_free = not proto.is_driving_cmd(proto.T_HARBOUR, proto.p_harbour(proto.HARB_HOLD), set())
    # a single operator: asked by cmd() (captain_confirm) - here: no own ynbox, and the request waits for that question
    station(env, crew="captain", state="OFF", running=False, entry_ok=True, single_op=True)
    cap.clear()
    ap.harbour_start()
    ok_so = cap.asked == [] and cap.frames == [] and ap.cconf is not None
    if ap.cconf is not None:
        ap.cconf_answer(False)
    ap.single_op = False
    # RESUME (from HOLD): the captain's YES too, action RESUME
    station(env, state="HOLD")
    cap.clear()
    cap.answer = True
    ap.harbour_start(resume=True)
    rs = [p for t, p in cap.frames if t == proto.T_HARBOUR]
    ok_rs = len(rs) == 1 and rs[0][:1] == bytes([proto.HARB_RESUME])
    # a viewer / no crew: refused
    station(env, crew="viewer", state="OFF", running=False, entry_ok=True)
    cap.clear()
    ap.harbour_start()
    ok_v = cap.frames == []
    report("HU10 the real station: the panel (states, legend, buttons), STOP always = ONE STANDBY request and no question, "
           "TAKE OVER (CREWED nothing / UNCREWED the REMOTE request / no licence nothing), START = the captain's own YES then "
           "T_HARBOUR START (corridor 10 m, 5 kn), NO sends nothing, not ready = refused, a copilot's START = the "
           "gateway's ACCEPT path, PAUSE is free, a single operator = cmd()'s own question, RESUME, a viewer refused",
           ok_panel and ok_resume and ok_take and ok_start and ok_nr and ok_cp and hold_free and ok_so and ok_rs and ok_v,
           "panel %s resume %s take %s start %s notready %s copilot %s hold_free %s single %s resume %s viewer %s" % (
               ok_panel, ok_resume, ok_take, ok_start, ok_nr, ok_cp, hold_free, ok_so, ok_rs, ok_v))


def hu11(env):
    ap = env.ap
    rig = env.rig
    cap = Cap(env)
    _ = cap
    # the banner on both screens: never red
    ok_b = True
    i18n.set_lang("en", save=False)
    for st in ("RUN", "SLOW", "HOLD"):
        station(env, state=st)
        hb = alerts.harbour_banner(ap.harbour_state())
        ok_b = ok_b and hb is not None and hb[0] == "harbour" and (hb[2] == "caution" if st == "HOLD" else hb[2] == "harbour") \
            and "ROUTE CLEAR" in hb[1]
    station(env, state="DONE")
    ok_b = ok_b and alerts.harbour_banner(ap.harbour_state()) is None
    ok_b = ok_b and theme.BANNERS["harbour"][1] not in (theme.WARN,) and theme.NOTICE["harbour"] != theme.WARN
    texts = {}
    station(env, state="HOLD")
    ap.say("")
    ap.render(force=True)
    banners = [x for x in ap.ov.ops if x.get("tag") == "harbour"]
    ok_b = ok_b and bool(banners)
    # screen 1's notices carry the same line
    notices = []
    drv = env.saved[2]                                   # the station's real screen 1 (Env.reset switched it off)
    station(env, state="HOLD")
    if drv is not None:
        try:
            notices = [n for n in drv.notices(ap.t, ap.gw_status) if n[0] == "harbour"]
        except Exception as e:
            notices = [("error", repr(e), "")]
    ok_b = ok_b and (drv is None or (len(notices) == 1 and notices[0][2] == "caution"))
    # fit: 1366x768 .. 1920x1080 at 100 / 125 / 150 %, Greek + English, the panel full
    import test_screen2 as S2
    station(env, state="RUN", tracks=[trk(i) for i in range(5)], unsure=["U1 range lost", "tracker cycle old"], hflags=0x06,
            grace=True, grace_left_s=30.0)
    cases = []
    for lang in ("el", "en"):
        i18n.set_lang(lang, save=False)
        for W, H in ((1366, 768), (1366, 658), (1920, 1080), (1920, 970)):
            for dpi in (1.0, 1.25, 1.5):
                rig.size(W, H, dpi)
                ap.harbour_view["_rx"] = time.time() + 1e6       # (the picture must not age out while the layouts run)
                ap.render(force=True)
                p = S2.layout_problems(ap)
                have = any(q.key == "harbour" for q in ap.ov.panels)
                cases.append(((lang, W, H, dpi), have, p))
    rig.size(1366, 768, 1.0)
    i18n.set_lang("en", save=False)
    bad = [c for c in cases if not c[1] or c[2]]
    # no colour / font literal in the new modules (S9 style)
    lit = []
    for fn in MODS + ("harbourpanel.py",):
        for n, line in enumerate(open(os.path.join(HERE, fn), encoding="utf-8"), 1):
            if S2.LITERAL.search(line):
                lit.append("%s:%d" % (fn, n))
    report("HU11 the banner on both screens (RUN / SLOW = harbour notice colour, HOLD = caution, never red, none when DONE); the "
           "panel fits 1366x768 .. 1920x1080 at 100 / 125 / 150 %, el + en, nothing clipped, never over the boat; no colour / "
           "font literal in the new modules",
           ok_b and not bad and not lit and not rig.errors,
           "banner %s notices %s / %d layouts, bad %s / literals %s / errors %s" % (ok_b, len(notices), len(cases),
                                                                                    [(c[0], c[2][:2]) for c in bad][:3], lit[:4],
                                                                                    rig.errors[:2]))
    _ = texts


def hu12(env):
    ap = env.ap
    rig = env.rig
    cap = Cap(env)
    i18n.set_lang("en", save=False)
    station(env, state="OFF", running=False, entry_ok=True)
    tmp = tempfile.mkdtemp(prefix="trygons_hu12_")
    # a saved track
    import track as trackmod
    tr = trackmod.Track("TESTBOAT", "k")
    for p in wiggle(600):
        tr.add(p)
    path = os.path.join(tmp, "t.jsonl")
    ap.tracks.load = lambda p, _tr=tr: _tr
    win = ap.harbour_route_window()
    win.set_track(path)
    rig.root.update()
    win.vars["harbour"].set("testharbour")
    win.vars["name"].set("exit")
    win.make()
    r = win.route
    ok_make = r is not None and 2 <= len(r["wps"]) <= 50 and ap.harbour_preview is r
    # SEND before CHECK is refused
    cap.clear()
    win.send()
    ok_first = cap.frames == [] and "CHECK" in win.msg.cget("text")
    # CHECK = a gateway message; a refusal (129) keeps SEND refused; an OK allows it
    cap.clear()
    win.check()
    chk = [m for m in cap.msgs if m.get("t") == "harbour_route" and m.get("op") == "check"]
    ok_chk = len(chk) == 1 and len(chk[0]["wps"]) == len(r["wps"]) and chk[0]["corr_m"] == r["corr_m"] and cap.frames == []
    ap.on_harbour_ack({"t": "harbour_ack", "ok": False, "code": 129, "why": "corridor (left edge, +-10 m) of leg WP1 -> WP2: 1.0 m water"})
    cap.clear()
    win.send()
    ok_ref = cap.frames == [] and not win.checked and "CHECK" in win.msg.cget("text")
    ap.on_harbour_ack({"t": "harbour_ack", "ok": True, "code": 0, "why": "OK"})
    cap.clear()
    win.send()
    types = [t for t, _p in cap.frames]
    begin = [p for t, p in cap.frames if t == proto.T_PROG_BEGIN]
    ok_send = types[0] == proto.T_PROG_BEGIN and types.count(proto.T_PROG_WP) == len(r["wps"]) and types[-1] == proto.T_PROG_COMMIT \
        and len(begin) == 1 and begin[0][2] == 1 and begin[0][0] == len(r["wps"]) and proto.T_HARBOUR not in types \
        and proto.T_AUTO_START not in types and ap.harbour_route is r and ap.harbour_preview is None
    # the normal program is not flagged (the harbour flag never leaks into a plain route)
    plain = proto.p_prog_begin(3, 0)
    ok_plain = len(plain) == 2 and "flags" not in ap.prog
    # a TEST sample loads, is flagged and is never sent
    sample = os.path.join(hr.SAMPLES_DIR, "daskalio", "sample_exit.json")
    win.load(sample)
    win.checked = True
    cap.clear()
    win.send()
    ok_sample = win.sample and cap.frames == [] and "SAMPLE" in win.msg.cget("text")
    # a too short part
    win.vars["from"].set("10")
    win.vars["to"].set("11")
    win.make()
    ok_short = win.route is None and ap.harbour_preview is None
    win.close()
    ok_closed = not win.alive() and ap.harbour_preview is None
    del ap.tracks.load
    report("HU12 the route window: MAKE from a saved track (preview drawn), SEND refused until the gateway's CHECK says OK (129 "
           "keeps it refused), SEND = the normal upload frames with the harbour flag and NEVER a start, a plain route stays "
           "unflagged, a TEST sample is never sent, a too-short part is refused",
           ok_make and ok_first and ok_chk and ok_ref and ok_send and ok_plain and ok_sample and ok_short and ok_closed,
           "make %s first %s chk %s refused %s send %s plain %s sample %s short %s closed %s" % (
               ok_make, ok_first, ok_chk, ok_ref, ok_send, ok_plain, ok_sample, ok_short, ok_closed))


def main():
    tmp = tempfile.mkdtemp(prefix="trygons_harbourui_")
    for fn in (hu2, hu3, hu4, hu5, hu6, hu7, hu8, hu9):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "crashed: %r" % e)
    rig = TS.ts.Rig()
    try:
        hu1(rig.root)
    except Exception as e:
        import traceback
        traceback.print_exc()
        report("HU1", False, "crashed: %r" % e)
    import traceback as _tb
    rig.root.report_callback_exception = lambda *a: (rig.errors.append(repr(a[1])), _tb.print_exception(*a))
    env = None
    try:
        env = TS.Env(rig, tmp)
        keep_t = (rig.ap.t, rig.ap.t_rx)
        for fn in (hu10, hu11, hu12):
            try:
                env.reset()
                fn(env)
            except Exception as e:
                import traceback
                traceback.print_exc()
                report(fn.__name__.upper(), False, "crashed: %r" % e)
        report("HU0 the Tk callbacks raised nothing", not rig.errors, str(rig.errors[:3]))
    finally:
        if env:
            env.restore()
            rig.ap.t, rig.ap.t_rx = keep_t
            rig.ap.harbour_view = rig.ap.harbour_model = rig.ap.harbour_route = rig.ap.harbour_preview = None
        rig.close()
    bad = results.count(False)
    print("HARBOUR STATION TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
