"""Professional look (step 13i, DISPLAY ONLY): start-up screen, app icon, window titles, About, the
academy in the station's theme, Desktop shortcuts, no console windows in the demo.

  LK1 start-up screen: closes by itself when the work succeeds (the progress line shows each step)
  LK2 start-up screen: an error is SHOWN (message + Close) instead of a hang, then it closes (Close /
      Esc / by itself after its hold time); an exception in the work = an error too
  LK3 start-up screen: closes after max_s even when nobody reports "ready"; the work still finishes;
      it is made before the station / academy window is shown and closed before it (never over it)
  LK4 app icon: assets/trygons_c2.ico (16 .. 256 px, the logo mark, no text) on every window + the
      taskbar (Tk default icon); the station and every academy window install it
  LK5 the same window titles everywhere: "Trygons C2 – ..." (el + en): screen 2 (shore / boat station),
      screen 1, tools, dialogs, the academy windows, the practical test, About
  LK6 Help -> About: logo, version, commit (= the git HEAD of this copy), licences / credits (el + en)
  LK7 the academy in the station's theme: dark background, no raw grey Tk button, every button flat
      with the theme's font, the orange banner kept, the accounts table headings never cut (el + en)
  LK8 tools/make_shortcuts.py: two shortcuts with the icon (pythonw = no console) in a TEMPORARY folder;
      no folder = refused (the real Desktop is never written by a test)
  LK9 the demo without console windows: every process demo.py starts is CREATE_NO_WINDOW, START_DEMO.bat
      starts pythonw with the start-up screen, demo.py's messages still go to demo_run/logs/demo.log, the
      stations get --no-splash + --ready-file
13i2 (review 473694b):
  LK10 M1: the demo's start-up screen can't load (no tkinter / Pillow / display) = the traceback in
       demo.log, a message box, the plain start still runs (never silent under pythonw)
  LK11 M2: an error AFTER the 30 s close (and a start still running long after it) is shown (a window /
       message box with the text + the log path) - never hidden; show_error really opens a window
  LK12 M3: START_DEMO.bat / make_shortcuts.py: pythonw ONLY next to the checked Python (tkinter + Pillow),
       else the console Python - never another Python from the PATH
  LK13 M4: the station / academy splash (BackgroundSplash, own thread) keeps its watchdog while the main
       thread is blocked (App() being built): after max_s the small "start is taking long" window, an
       error shown, closed on request; never topmost
  LK14 LOWs: STOP_DEMO during the start stops the starting demo process too (pids.json "demo"), never
       itself; version.commit() "+dirty" on a changed tree; no leftover temp shortcut when PowerShell
       fails; Esc / focus_force on the splash; never topmost

13i3 (review 835187d):
  LK15 M-A: "Open the academy" from a running station starts it with --no-splash (no full-screen splash
       taking the focus over the live station); the academy started alone keeps its splash
  LK16 M-B: version "+dirty" cached per process, gives up after 0.5 s (no mark); no .git, a v4 index,
       CRLF, a copied tree (new mtimes)
  LK17 L1: the splash polling survives a failing message (close() still works, an error still shown)
  LK18 L2: STOP_DEMO kills a pids.json entry only when pid + creation time match (a stale pid is never
       killed)
  LK19 L6: focus_force really called on the splash window; BackgroundSplash next to a busy main Tk

13j2 (review ef51900):
  LK20 LOW-1: STOP_DEMO keeps the refused pids.json entries (and prints their pids) so a second STOP_DEMO
       can try again; the file goes only when nothing was refused
  LK21 LOW-2: a failing _facts() / state copy in BackgroundSplash.poll never raises out of the Tk
       callback (no error every 100 ms) and close() still closes it
  LK22 LOW-3: version.dirty_cached scans ONCE with two threads at the same time (per-repo lock); a changed
       file above HASH_MAX_BYTES is never read (unknown = no mark)

13n1 (owner 2026-10-31 E1):
  LK23 BATTLE SHORT: every input (empty, any text, any length) = INVALID AUTHORISATION CODE, no lockout, nothing
       sent / no setting changed, only the station log line; no code / hash / link in battleshort.py; masked
       field, AUTHORISE above CANCEL, Cancel closes
  LK24 3 text sizes + Aa: every level = one of 3 sizes (widgets, drawn text, the HUD); Aa cycles small / normal /
       large, saved per PC, every window told; widgets rescaled; red = STANDBY only; square frames
  LK25 (13n1 review M1 / L1 / L7) Aa: a widget in the tools window (a Toplevel under the root) is rescaled ONCE by
       the station's text_size_changed (not ratio squared); small / normal / large many times = no size drift; the
       question serial never repeats across two DriverScreens

13n2 (owner 2026-10-31 E2):
  LK26 TOUR.bat next to START_DEMO.bat: academy/tour_app.py --lang el, pythonw ONLY next to the checked Python (as
       START_DEMO.bat), no demo / simulator / gateway started; the tour really opens (a window, no crash in 4 s)

Run:  python test_look.py        (exit code 0 = all passed)
"""
import os
import re
import sys
import tempfile
import threading
import time
import tkinter as tk
import tkinter.font  # noqa: F401  (tk.font, LK24)

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="trygons_look_"), "gs_settings.json")
os.environ.setdefault("TRYGONS_SOUND", "off")
os.environ["TRYGONS_ACADEMY_DATA"] = tempfile.mkdtemp(prefix="trygons_look_acad_")

import i18n      # noqa: E402
import theme     # noqa: E402
import splash    # noqa: E402
import about     # noqa: E402
import version   # noqa: E402

results = []
DASH = "Trygons C2 – "          # EN DASH


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def pump(root, s, until=None):
    end = time.time() + s
    while time.time() < end:
        try:
            root.update()
        except tk.TclError:
            return bool(until and until())
        if until and until():
            return True
        time.sleep(0.02)
    return bool(until and until())


def src(*parts):
    with open(os.path.join(REPO, *parts), encoding="utf-8") as f:
        return f.read()


# --- LK1-LK3: the start-up screen --------------------------------------------------------------------
def lk1():
    seen = {}

    def task(progress):
        for k in ("splash.p_sim", "splash.p_gateway", "splash.p_cameras", "splash.p_stations"):
            progress(k)
            time.sleep(0.15)
        return True, ""

    def watch(sp):
        seen["sp"] = sp
    t0 = time.time()
    ok, _ = splash.run_task(task, ["splash.p_sim", "splash.p_gateway", "splash.p_cameras", "splash.p_stations"],
                            lang="el", fullscreen=False, max_s=20, on_splash=watch)
    dt = time.time() - t0
    sp = seen.get("sp")
    words = [i18n.TL("el", k) for k in ("splash.p_sim", "splash.p_gateway", "splash.p_cameras", "splash.p_stations")]
    line = sp.progress_text() if sp else ""
    report("LK1 start-up screen closes by itself on success, the progress line shows every step",
           ok and sp is not None and sp.state == "closed" and dt < 5 and all(w in line for w in words) and
           line.startswith(i18n.TL("el", "splash.starting")),
           "ok %s, state %s, %.1f s, line %r" % (ok, sp and sp.state, dt, line))


def lk2():
    seen = {}

    def bad(progress):
        progress("splash.p_sim")
        return False, "the simulator did not start (demo_run/logs/simulator.log)"

    def watch(sp):
        seen["sp"] = sp
        sp.w.after(600, lambda: seen.update(err=sp.error_text, st=sp.state, btn=sp.b_close.winfo_ismapped()))
    t0 = time.time()
    ok, msg = splash.run_task(bad, ["splash.p_sim"], lang="en", fullscreen=False, max_s=20, error_hold_s=1.5,
                              on_splash=watch)
    dt = time.time() - t0
    shown = seen.get("st") == "failed" and "simulator did not start" in (seen.get("err") or "") and seen.get("btn")
    closed1 = seen["sp"].state == "closed" and 1.4 < dt < 6
    # an exception in the work = an error on the screen too; Esc closes it at once

    def boom(progress):
        raise RuntimeError("no MediaMTX")

    def esc(sp):
        seen["sp2"] = sp
        sp.w.after(500, lambda: (seen.update(err2=sp.error_text, esc=bool(sp.w.bind("<Escape>"))),
                                 sp.b_close.invoke()))
    t1 = time.time()
    ok2, msg2 = splash.run_task(boom, [], lang="el", fullscreen=False, max_s=20, error_hold_s=60, on_splash=esc)
    dt2 = time.time() - t1
    closed2 = seen["sp2"].state == "closed" and dt2 < 5 and seen.get("esc") and "no MediaMTX" in (seen.get("err2") or "") and \
        i18n.TL("el", "splash.failed") in (seen.get("err2") or "")
    report("LK2 start-up screen: an error is shown (message + Close), never a hang; it closes (Close / Esc / hold time)",
           not ok and shown and closed1 and not ok2 and "no MediaMTX" in msg2 and closed2,
           "error shown %s (%r), closed after %.1f s; exception: %r, Close (Esc bound) closed after %.1f s" % (
               shown, (seen.get("err") or "")[:60], dt, msg2, dt2))


def lk3():
    done = threading.Event()

    def slow(progress):
        progress("splash.p_sim")
        time.sleep(2.5)
        done.set()
        return True, ""
    seen = {}
    t0 = time.time()
    ok, _ = splash.run_task(slow, ["splash.p_sim", "splash.p_gateway"], lang="el", fullscreen=False, max_s=0.8,
                            on_splash=lambda sp: seen.update(sp=sp))
    dt = time.time() - t0
    closed_early = seen["sp"].state == "closed"
    # the default limit: 30 s; the station / academy: made while the window is withdrawn, closed before it shows
    app_src, acad_src = src("groundstation", "app.py"), src("academy", "academy_app.py")
    order = []
    for s in (app_src, acad_src):
        m = s[s.index("root = tk.Tk()"):]
        order.append(m.index("root.withdraw()") < m.index("BackgroundSplash(") < m.index("sp.close()") <
                     m.index("root.deiconify()") < m.index("root.mainloop()"))
    report("LK3 start-up screen: at most max_s (30 s default) without 'ready', the work goes on; made before "
           "and closed before the station / academy window shows (never over it)",
           closed_early and done.is_set() and ok and splash.MAX_S == 30.0 and all(order),
           "closed %s, work finished %s, run_task back after %.1f s; order station / academy %s" % (
               closed_early, done.is_set(), dt, order))


# --- LK4: the app icon -----------------------------------------------------------------------------------
def lk4(root):
    from PIL import Image
    ico = theme.icon_path("ico")
    sizes = set()
    if ico:
        im = Image.open(ico)
        sizes = set(im.info.get("sizes") or [])
    want = {(s, s) for s in (16, 32, 48, 256)}
    got = theme.set_app_icon(root)
    top = tk.Toplevel(root)
    again = theme.set_app_icon(top)            # one icon per application (default for every window)
    top.destroy()
    mark = src("assets", "trygonsdrones_mark.svg")
    no_text = mark.count("<path") == 2 and "st1" not in mark             # st1 = the lettering of the logo
    used = "theme.set_app_icon(root)" in src("groundstation", "app.py") and \
        "theme.set_app_icon(root)" in src("academy", "look.py") and "theme.set_app_id()" in src("groundstation", "app.py")
    acad = [f for f in os.listdir(os.path.join(REPO, "academy")) if f.endswith("_app.py")]
    installs = [f for f in acad if "look.install(" not in src("academy", f) and f != "exam_app.py"]
    report("LK4 app icon: the logo mark (no text) as .ico 16..256 px on every window + the taskbar",
           ico is not None and want <= sizes and got and again is got and no_text and used and not installs,
           "ico %s sizes %s; icon set %s; mark without lettering %s; station + academy %s; academy windows "
           "without look.install %s" % (ico and os.path.basename(ico), sorted(sizes)[:3], bool(got), no_text,
                                        used, installs))


# --- LK5: window titles ----------------------------------------------------------------------------------
def lk5():
    import app as A
    bad = []
    for lc in ("el", "en"):
        i18n.set_lang(lc, save=False)

        class F(object):
            title_extra = ""
        for place, key in ((None, "win.station_shore"), ("shore", "win.station_shore"),
                           ("onboard", "win.station_onboard")):
            f = F()
            f.place = place
            t = A.App.window_title(f)
            if not t.startswith(DASH) or i18n.T(key) not in t or i18n.T("cap.title") not in t:
                bad.append("%s %s: %r" % (lc, place, t))
        for key in ("drv.title", "tools.title", "boat.title", "boat.leave_title", "cap.cc_title", "cap.pw_title"):
            if not i18n.W(key).startswith(DASH):
                bad.append("%s %s" % (lc, key))
    i18n.set_lang("el", save=False)
    # every title in the screen modules goes through i18n.W / window_title / win.title
    rx = re.compile(r"\.title\((?!\)|i18n\.W\(|self\.window_title\(|i18n\.TL\([^,]+,\s*\"win\.title\")")
    for fn in ("app.py", "driver.py", "crewui.py", "splash.py", "about.py"):
        for n, line in enumerate(src("groundstation", fn).splitlines(), 1):
            if rx.search(line) and "def " not in line:
                bad.append("%s:%d %s" % (fn, n, line.strip()[:60]))
    arx = re.compile(r"\.title\((?!\)|T\(\"win\.title\")")
    for fn in sorted(os.listdir(os.path.join(REPO, "academy"))):
        if fn.endswith("_app.py"):
            for n, line in enumerate(src("academy", fn).splitlines(), 1):
                if arx.search(line) and "run.title" not in line and "look.title(" not in line:
                    bad.append("academy/%s:%d" % (fn, n))
    for n, line in enumerate(src("academy", "practical", "ptest_app.py").splitlines(), 1):
        if arx.search(line):
            bad.append("ptest_app.py:%d" % n)
    sys.path.insert(0, os.path.join(REPO, "academy"))
    import text
    for lc in ("el", "en"):
        for key in ("academy.title", "lesson.title", "quiz.title", "pt.title"):
            t = text.T("win.title", lc, name=text.T(key, lc))
            if not t.startswith(DASH):
                bad.append("academy %s %s" % (lc, key))
    report("LK5 the same window titles everywhere: 'Trygons C2 – ...' (station, screen 1, tools, dialogs, "
           "academy, practical test, About; el + en)", not bad, "; ".join(bad[:6]) or
           "screen 2 %r" % A.App.window_title(type("F", (), {"title_extra": "", "place": "shore"})()))


# --- LK6: About --------------------------------------------------------------------------------------------
def lk6(root):
    bad = []
    commit = version.commit()
    if not re.match(r"^[0-9a-f]{7}(\+dirty)?$", commit or ""):
        bad.append("commit %r" % commit)
    for lc in ("el", "en"):
        ab = about.open_about(root, lc)
        pump(root, 0.3)
        names = [n for n, _ in ab.credit_lines]
        if not ab.w.title().startswith(DASH):
            bad.append("title %r" % ab.w.title())
        if version.VERSION not in ab.version_text or commit not in ab.commit_text:
            bad.append("%s version / commit %r %r" % (lc, ab.version_text, ab.commit_text))
        for need in ("libcanard", "eWaSR", "MediaMTX", "FFmpeg", "Piper"):
            if need not in names:
                bad.append("%s credit %s missing" % (lc, need))
        if theme.logo_path() and ab.logo is None:
            bad.append("no logo")
        again = about.open_about(root, lc, ab)
        if again is not ab:
            bad.append("a second window")
        ab.close()
    report("LK6 Help -> About: logo, version, commit (git HEAD), licences / credits (el + en)", not bad,
           "; ".join(bad) or "version %s, commit %s, %d credits" % (version.VERSION, commit, len(about.CREDITS)))


# --- LK7: the academy in the station's theme -----------------------------------------------------------
GREY = {"systembuttonface", "#f0f0f0", "#d9d9d9", "systemwindow", "#ffffff", "white"}


def buttons(w):
    out, stack = [], [w]
    while stack:
        x = stack.pop()
        if x.winfo_class() == "Button":
            out.append(x)
        stack.extend(x.winfo_children())
    return out


def lk7(root):
    import tkinter.font as tkfont
    sys.path.insert(0, os.path.join(REPO, "academy"))
    import text
    import accounts
    import academy_app
    import tour_app
    import quiz_app
    bad = []
    store = accounts.Store(None)
    for u, n in (("maria", "Μαρία Παπαδοπούλου"), ("nikos", "Νίκος Γεωργίου")):
        try:
            store.add(u, n)
        except Exception:
            pass
    heads = []
    for lc in ("el", "en"):
        text.set_lang(lc)
        top = tk.Toplevel(root)
        ap = academy_app.AcademyApp(top, store)
        pump(root, 0.5)
        tops = [(top, "academy"), ]
        t2 = tk.Toplevel(root)
        tour_app.TourWindow(t2, lc, player=tour_app.NullPlayer(), cache=tempfile.mkdtemp())
        t3 = tk.Toplevel(root)
        quiz_app.QuizWindow(t3, lc, player=tour_app.NullPlayer(), cache=tempfile.mkdtemp(), seed=1)
        tops += [(t2, "tour"), (t3, "quiz")]
        pump(root, 0.5)
        for w, name in tops:
            if str(w.cget("bg")).lower() != theme.C["bg"].lower():
                bad.append("%s %s bg %s" % (lc, name, w.cget("bg")))
            for b in buttons(w):
                if str(b.cget("bg")).lower() in GREY or str(b.cget("relief")) != "flat":
                    bad.append("%s %s button %r bg %s relief %s" % (lc, name, b.cget("text"), b.cget("bg"),
                                                                     b.cget("relief")))
                fam = tkfont.Font(root=b, font=b.cget("font")).actual().get("family")
                if fam not in (theme.FAMILY, theme.MONO):
                    bad.append("%s %s button font %s" % (lc, name, fam))
        if ap.banner.cget("bg").lower() != theme.CAUTION.lower() or ap.banner.cget("text") != text.T("academy.banner", lc):
            bad.append("%s banner" % lc)
        hf = tkfont.Font(root=ap.tree, font=theme.font("button"))
        for c in ap.tree["columns"]:
            need = hf.measure(ap.tree.heading(c, "text")) + 8
            have = int(ap.tree.column(c, "width"))
            heads.append(have - need)
            if have < need:
                bad.append("%s column %s cut (%d < %d)" % (lc, ap.tree.heading(c, "text"), have, need))
        for w, _n in tops:
            w.destroy()
    text.set_lang("el")
    report("LK7 the academy in the station's theme: dark, no raw grey Tk button, flat themed buttons, the orange "
           "banner kept, the accounts headings never cut (el + en)", not bad,
           "; ".join(bad[:6]) or "academy + tour + quiz windows; headings with %d px spare at least" % min(heads))


# --- LK8: Desktop shortcuts -----------------------------------------------------------------------------
def lk8():
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import make_shortcuts as ms
    bad = []
    try:
        ms.make(None)
        bad.append("no folder accepted")
    except ValueError:
        pass
    tmp = tempfile.mkdtemp(prefix="trygons_shortcuts_")
    items = ms.make(tmp)
    names = sorted(os.path.basename(it["path"]) for it in items)
    for it in items:
        if not os.path.exists(it["path"]) or os.path.getsize(it["path"]) < 100:
            bad.append("missing %s" % it["path"])
        if not os.path.exists(it["icon"]):
            bad.append("icon %s" % it["icon"])
        if os.name == "nt" and not it["target"].lower().endswith("pythonw.exe"):
            bad.append("not pythonw: %s" % it["target"])
        if not os.path.abspath(it["path"]).startswith(os.path.abspath(tmp)):
            bad.append("outside the folder: %s" % it["path"])
    want = ["Trygons C2 Demo", "Trygons Ακαδημία"]
    if [os.path.splitext(n)[0] for n in names] != want:
        bad.append("names %s" % names)
    if "--splash" not in items[0]["args"] or "demo.py" not in items[0]["args"] or "academy_app.py" not in items[1]["args"]:
        bad.append("arguments %s" % [it["args"] for it in items])
    report("LK8 tools/make_shortcuts.py: 'Trygons C2 Demo' + 'Trygons Ακαδημία' with the icon (pythonw) in a temp "
           "folder; no folder = refused (never the real Desktop in a test)", not bad,
           "; ".join(bad) or "%s in %s" % (names, tmp))


# --- LK9: the demo without console windows ---------------------------------------------------------------
def lk9():
    s = src("demo", "demo.py")
    bad = []
    calls = [m.start() for m in re.finditer(r"subprocess\.(?:Popen|run)\(", s)]
    for i in calls:
        depth, j = 0, s.index("(", i)
        k = j
        while True:
            if s[k] == "(":
                depth += 1
            elif s[k] == ")":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        if "creationflags=NO_WINDOW" not in s[j:k]:
            bad.append("demo.py:%d %s" % (s[:i].count("\n") + 1, s[i:i + 40]))
    bat = src("START_DEMO.bat")
    if "pythonw" not in bat or "--splash" not in bat or 'start ""' not in bat:
        bad.append("START_DEMO.bat does not start pythonw with the start-up screen")
    if "demo.log" not in s or "sys.stdout is not None" not in s:
        bad.append("demo.py messages not logged / not pythonw-safe")
    if s.count("--no-splash") < 2 or s.count("--ready-file") < 2:
        bad.append("stations without --no-splash / --ready-file")
    app_src = src("groundstation", "app.py")
    if "--ready-file" not in app_src or "--no-splash" not in app_src:
        bad.append("app.py options")
    # demo.say works with no console (pythonw: sys.stdout None) and writes the log
    sys.path.insert(0, os.path.join(REPO, "demo"))
    import demo
    old_logs, old_out = demo.LOGS, sys.stdout
    demo.LOGS = tempfile.mkdtemp(prefix="trygons_demolog_")
    try:
        sys.stdout = None
        demo.say("look test line")
    finally:
        sys.stdout = old_out
        logs, demo.LOGS = demo.LOGS, old_logs
    if "look test line" not in open(os.path.join(logs, "demo.log"), encoding="utf-8").read():
        bad.append("demo.log not written")
    report("LK9 the demo without console windows: CREATE_NO_WINDOW for every process, START_DEMO = pythonw + "
           "start-up screen, messages in demo_run/logs/demo.log", not bad,
           "; ".join(bad) or "%d process calls checked" % len(calls))


# --- 13i2: review fixes ---------------------------------------------------------------------------------
def _demo():
    sys.path.insert(0, os.path.join(REPO, "demo"))
    import demo
    return demo


def lk10():
    demo = _demo()
    bad = []
    saved = dict(LOGS=demo.LOGS, start=demo.start, MESSAGE_BOX=demo.MESSAGE_BOX)
    old_mod = sys.modules.get("splash")
    boxes, starts = [], []
    demo.LOGS = tempfile.mkdtemp(prefix="trygons_lk10_")
    demo.MESSAGE_BOX = lambda title, text: boxes.append((title, text))
    demo.start = lambda progress=None: (starts.append(progress), 0)[1]
    try:
        sys.modules["splash"] = None            # "import splash" fails (= no tkinter / Pillow)
        try:
            rc = demo.start_splash()
        except Exception as e:
            rc = "raised %s" % type(e).__name__
        sys.modules["splash"] = old_mod
        logf = os.path.join(demo.LOGS, "demo.log")
        log = open(logf, encoding="utf-8").read() if os.path.exists(logf) else ""
        if rc != 0:
            bad.append("rc %r" % rc)
        if starts != [None]:
            bad.append("the plain start did not run: %r" % starts)
        if "Traceback" not in log or "import of splash" not in log:
            bad.append("no traceback in demo.log")
        if not boxes or i18n.T("splash.no_screen") not in boxes[0][1] or "demo.log" not in boxes[0][1]:
            bad.append("no message box: %r" % boxes)
        # the Tk window itself fails (no display) before the start: the same fallback
        del boxes[:], starts[:]
        real = splash.run_task

        def no_display(*a, **k):
            raise tk.TclError("no display name and no $DISPLAY environment variable")
        splash.run_task = no_display
        try:
            rc2 = demo.start_splash()
        except Exception as e:
            rc2 = "raised %s" % type(e).__name__
        finally:
            splash.run_task = real
        if rc2 != 0 or starts != [None] or not boxes:
            bad.append("no display: rc %r, starts %r, boxes %d" % (rc2, starts, len(boxes)))
        # the plain start fails too: a second box with its error
        del boxes[:], starts[:]
        demo.start = lambda progress=None: (demo.say("the simulator did not build"), 1)[1]
        sys.modules["splash"] = None
        rc3 = demo.start_splash()
        sys.modules["splash"] = old_mod
        if rc3 != 1 or len(boxes) != 2 or "did not build" not in boxes[1][1]:
            bad.append("start error not shown: rc %r boxes %r" % (rc3, boxes))
    finally:
        sys.modules["splash"] = old_mod
        for k, v in saved.items():
            setattr(demo, k, v)
    report("LK10 M1: the demo's start-up screen can't load / show = traceback in demo.log + message box + the "
           "plain start (never silent under pythonw)", not bad, "; ".join(bad) or "3 cases")


def lk11():
    bad = []
    shown = []

    def late(progress):
        progress("splash.p_sim")
        time.sleep(1.2)                         # longer than the splash's cap
        return False, "the gateway did not start\n(demo.log)"
    ok, msg = splash.run_task(late, ["splash.p_sim"], lang="en", fullscreen=False, max_s=0.4,
                              on_error=shown.append)
    if ok or shown != [msg] or "gateway did not start" not in (shown or [""])[0]:
        bad.append("late error not shown: ok %s, shown %r" % (ok, shown))
    # an error shown ON the splash (before the cap) is not shown a second time
    del shown[:]
    splash.run_task(lambda p: (False, "early"), [], lang="en", fullscreen=False, max_s=20, error_hold_s=0.5,
                    on_error=shown.append)
    if shown:
        bad.append("an error shown twice %r" % shown)

    def hang(progress):                         # still running long after the splash: said, + the log path
        time.sleep(2.0)
        return True, ""
    ok2, msg2 = splash.run_task(hang, [], lang="en", fullscreen=False, max_s=0.3, join_s=0.6,
                                log_hint="Messages: X/demo.log", on_error=shown.append)
    if ok2 or not shown or i18n.TL("en", "splash.still_running", s=0)[:20] not in shown[0] or "X/demo.log" not in shown[0]:
        bad.append("still running not said: %r" % shown)
    # show_error opens a real window (Close / Esc / hold time)
    t1 = time.time()
    how = splash.show_error("test error text", lang="en", error_hold_s=0.6)
    dt = time.time() - t1
    if how != "window" or not (0.5 < dt < 6):
        bad.append("show_error %r after %.1f s" % (how, dt))
    report("LK11 M2: an error after the 30 s close / a start still running is shown (window, message box), "
           "never hidden", not bad, "; ".join(bad) or "late error shown once, still-running said, window %.1f s" % dt)


def lk12():
    bad = []
    bat = src("START_DEMO.bat")
    if "where pythonw" in bat:
        bad.append("START_DEMO.bat still takes a pythonw from the PATH")
    if "import sys, tkinter, PIL" not in bat or "%%~dpIpythonw.exe" not in bat:
        bad.append("START_DEMO.bat does not check the Python before its pythonw")
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import make_shortcuts as ms
    ms_src = src("tools", "make_shortcuts.py")
    if 'which("pythonw")' in ms_src or "msys64" in ms_src:
        bad.append("make_shortcuts.py takes another Python")
    if os.name == "nt":
        fake = tempfile.mkdtemp(prefix="trygons_lk12_")            # a "Python" without tkinter / Pillow
        for n in ("python.exe", "pythonw.exe"):
            open(os.path.join(fake, n), "wb").close()
        got = ms.pythonw(os.path.join(fake, "python.exe"))
        if got.lower().endswith("pythonw.exe"):
            bad.append("a pythonw next to an unchecked Python: %s" % got)
        real = ms.pythonw()
        want = os.path.join(os.path.dirname(sys.executable), "pythonw.exe")
        if os.path.exists(want) and os.path.normcase(real) != os.path.normcase(want):
            bad.append("this Python's pythonw not used: %s" % real)
    report("LK12 M3: pythonw only next to the checked Python (tkinter + Pillow), else the console Python",
           not bad, "; ".join(bad) or "bat + make_shortcuts")


def lk13():
    bad = []
    sp = splash.BackgroundSplash(lang="en", what="splash.what_station", steps=["splash.p_station"],
                                 fullscreen=False, max_s=0.8)
    sp.step("splash.p_station")
    first = dict(sp.info)
    time.sleep(2.0)                             # the main thread is blocked (App() being built)
    if sp.state != "slow" or not sp.slow_seen:
        bad.append("no watchdog while the main thread is blocked: state %s" % sp.state)
    if sp.info.get("prog") != i18n.TL("en", "splash.slow"):
        bad.append("no 'taking long' text: %r" % sp.info.get("prog"))
    if sp.info.get("frameless") or sp.info.get("topmost") or first.get("topmost") or not first:
        bad.append("covers: %r / %r" % (first, sp.info))
    w, h = sp.info.get("size", (0, 0))
    if w > splash.SLOW_SIZE[0] + 40 or h > splash.SLOW_SIZE[1] + 80:
        bad.append("still big: %dx%d" % (w, h))
    sp.close()
    if sp.th.is_alive() or sp.state != "closed":
        bad.append("not closed")
    elif sp.freed is None or sp.freed() is not None:
        bad.append("its Tk was not freed in its own thread")
    # an error while the main thread is busy: shown, wait() returns after the hold time
    sp2 = splash.BackgroundSplash(lang="el", what="splash.what_academy", fullscreen=False, max_s=20,
                                  error_hold_s=0.8)
    sp2.fail("RuntimeError: no data")
    t0 = time.time()
    time.sleep(0.5)
    err = sp2.error_text
    sp2.wait(8)
    dt = time.time() - t0
    if "no data" not in err or i18n.TL("el", "splash.failed") not in err or sp2.th.is_alive() or dt > 6:
        bad.append("error %r, closed after %.1f s" % (err, dt))
    # the station / academy use it (a Tk splash on the main thread would never run its timer there)
    for part in (("groundstation", "app.py"), ("academy", "academy_app.py")):
        if "splash.BackgroundSplash(" not in src(*part) or "splash.Splash(root" in src(*part):
            bad.append("%s does not use BackgroundSplash" % part[1])
    report("LK13 M4: the station / academy splash keeps its watchdog while App() blocks the main thread "
           "('taking long' window, small, not topmost), errors shown, closed on request", not bad,
           "; ".join(bad) or "slow after 0.8 s with the main thread asleep; error closed after %.1f s" % dt)


def lk14():
    import json
    import shutil
    import subprocess
    demo = _demo()
    bad = []
    # STOP_DEMO during the start: the starting demo process is in pids.json, stopped first, never itself
    saved = dict(RUN=demo.RUN, PIDS=demo.PIDS, LOGS=demo.LOGS, check_tools=demo.check_tools, kill_tree=demo.kill_tree,
                 proc_ident=demo.proc_ident)
    tmp = tempfile.mkdtemp(prefix="trygons_lk14_")
    seen, killed = {}, []
    demo.RUN, demo.PIDS, demo.LOGS = tmp, os.path.join(tmp, "pids.json"), os.path.join(tmp, "logs")
    demo.kill_tree = killed.append

    def tools(interactive=True):
        with open(demo.PIDS, encoding="utf-8") as f:
            seen.update(json.load(f))
        return False, None
    demo.check_tools = tools
    real_ident = demo.proc_ident
    demo.proc_ident = lambda pid: real_ident(pid) if pid == os.getpid() else "id%s" % pid   # all "running"
    try:
        demo.start()
        if seen.get("demo") != os.getpid():
            bad.append("the starting demo is not in pids.json: %r" % seen)
        if os.path.exists(demo.PIDS):
            bad.append("pids.json left after the start")
        with open(demo.PIDS, "w", encoding="utf-8") as f:
            json.dump({"demo": 22, "simulator": 11, "gateway": 12, "shore": 33,
                       "_id": {"demo": "id22", "simulator": "id11", "gateway": "id12", "shore": "id33"}}, f)
        demo.stop()
        if killed != [22, 33, 12, 11]:
            bad.append("stop order %r" % killed)
        del killed[:]
        with open(demo.PIDS, "w", encoding="utf-8") as f:
            json.dump({"demo": os.getpid(), "simulator": 11, "_id": {"simulator": "id11"}}, f)
        demo.stop()
        if killed != [11]:
            bad.append("stop killed itself / wrong: %r" % killed)
    finally:
        for k, v in saved.items():
            setattr(demo, k, v)
    # version "+dirty" (a temporary git repo)
    git = shutil.which("git")
    if git:
        r = tempfile.mkdtemp(prefix="trygons_lk14_git_")
        os.makedirs(os.path.join(r, "d"))
        for n, t in (("a.txt", "one\n"), (os.path.join("d", "b.txt"), "two\n")):
            with open(os.path.join(r, n), "w", encoding="utf-8") as f:
                f.write(t)
        env = dict(os.environ, GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@t", GIT_COMMITTER_NAME="t",
                   GIT_COMMITTER_EMAIL="t@t")
        for cmd in (["init", "-q"], ["add", "."], ["commit", "-qm", "x"]):
            subprocess.run([git, "-C", r] + cmd, capture_output=True, env=env)
        clean = version.commit(r)
        with open(os.path.join(r, "d", "b.txt"), "w", encoding="utf-8") as f:
            f.write("changed\n")
        version.clear_cache()                   # 13i3: the result is cached per process
        changed = version.commit(r)
        if not re.match(r"^[0-9a-f]{7}$", clean) or changed != clean + "+dirty" or not version.dirty(r):
            bad.append("dirty mark: clean %r changed %r" % (clean, changed))
        if version.commit(r, mark=False) != clean:
            bad.append("mark=False")
    else:
        bad.append("git not found (the +dirty check needs it)")
    # make_shortcuts: PowerShell fails half way = no leftover temp shortcut
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import make_shortcuts as ms
    if os.name == "nt":
        d = tempfile.mkdtemp(prefix="trygons_lk14_lnk_")
        real = ms.subprocess.run

        def half(args, **kw):
            open(os.path.join(d, "_trygons_shortcut_0.lnk"), "wb").close()
            return subprocess.CompletedProcess(args, 1, "", "COM error")
        ms.subprocess.run = half
        try:
            ms.make(d, exe=sys.executable)
            bad.append("no error raised")
        except RuntimeError:
            pass
        finally:
            ms.subprocess.run = real
        if os.listdir(d):
            bad.append("leftover %s" % os.listdir(d))
    # Esc / focus on the frameless splash (Esc closes it), never topmost
    if "focus_force()" not in src("groundstation", "splash.py"):
        bad.append("no focus_force")
    sp = splash.Splash(None, lang="en", fullscreen=False, max_s=20)
    top = bool(sp.w.attributes("-topmost"))
    sp.w.event_generate("<Escape>")
    end = time.time() + 2
    while sp.state != "closed" and time.time() < end:
        sp.update()
        time.sleep(0.02)
    if top or sp.state != "closed":
        bad.append("topmost %s / Esc state %s" % (top, sp.state))
    report("LK14 LOWs: STOP_DEMO during the start stops the starting demo (never itself), '+dirty' commit, "
           "no leftover temp shortcut, Esc / focus, never topmost", not bad, "; ".join(bad) or "all")


# --- 13i3: review 835187d -------------------------------------------------------------------------------
def lk15():
    import academymenu
    sys.path.insert(0, os.path.join(REPO, "academy"))
    import academy_app
    import text as acad_text
    bad = []
    cmd = academymenu.academy_command()
    if "--no-splash" not in cmd:
        bad.append("the station opens the academy WITH its start-up screen: %r" % cmd)

    class Stop(Exception):
        pass
    made = []

    class FakeSplash(object):
        def __init__(self, **k):
            made.append(k)

        def step(self, key):
            pass

        def fail(self, m):
            pass

        def wait(self, timeout=None):
            pass

        def close(self, wait_s=5.0):
            pass

    def fake_app(root, store):
        root.destroy()
        raise Stop()
    saved = (splash.BackgroundSplash, academy_app.AcademyApp)
    splash.BackgroundSplash, academy_app.AcademyApp = FakeSplash, fake_app
    lang0 = acad_text.lang()
    try:
        for args, want in ((cmd[2:], 0), ([], 1)):
            del made[:]
            try:
                academy_app.main(list(args))
                bad.append("main %r did not reach the window" % args)
            except Stop:
                pass
            if len(made) != want:
                bad.append("main %r made %d start-up screens (want %d)" % (args, len(made), want))
    finally:
        splash.BackgroundSplash, academy_app.AcademyApp = saved
        acad_text.set_lang(lang0)
    report("LK15 M-A: 'Open the academy' from a running station = no full-screen start-up screen (--no-splash, "
           "never focus taken over ALARM / STANDBY); the academy started alone keeps it", not bad,
           "; ".join(bad) or " ".join(cmd[2:]))


def _git_repo(files, index_version=None):
    import shutil
    import subprocess
    git = shutil.which("git")
    if not git:
        return None
    r = tempfile.mkdtemp(prefix="trygons_lk16_")
    for n, t in files.items():
        p = os.path.join(r, *n.split("/"))
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "wb") as f:
            f.write(t)
    env = dict(os.environ, GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@t", GIT_COMMITTER_NAME="t",
               GIT_COMMITTER_EMAIL="t@t")
    cmds = [["init", "-q"], ["config", "core.autocrlf", "false"], ["add", "."], ["commit", "-qm", "x"]]
    if index_version:
        cmds.append(["update-index", "--index-version", str(index_version)])
    for c in cmds:
        subprocess.run([git, "-C", r] + c, capture_output=True, env=env)
    return r


def _touch_all(r, dt=500):
    for d, dirs, fs in os.walk(r):
        if ".git" in d.split(os.sep):
            continue
        for n in fs:
            p = os.path.join(d, n)
            st = os.stat(p)
            os.utime(p, (st.st_atime + dt, st.st_mtime + dt))


def lk16():
    import shutil
    bad = []
    # a copied folder without .git: no commit, no mark, no crash
    d = tempfile.mkdtemp(prefix="trygons_lk16_copy_")
    shutil.copytree(os.path.join(REPO, "groundstation", "lang"), os.path.join(d, "lang"))
    version.clear_cache()
    if version.commit(d) != "" or version.dirty(d) is not False:
        bad.append("no .git: %r / %r" % (version.commit(d), version.dirty(d)))
    open(os.path.join(d, ".git"), "w").close()                  # an empty ".git" file: still nothing
    if version.commit(d) != "" or version.dirty(d) is not False:
        bad.append("empty .git file")
    files = {"a.txt": b"one\n", "d/b.txt": b"two\nthree\n", "d/e/c.txt": b"x" * 100}
    r = _git_repo(files, index_version=4)
    if r is None:
        bad.append("git not found")
    else:
        with open(os.path.join(r, ".git", "index"), "rb") as f:
            idx = f.read()
        if idx[4:8] != b"\0\0\0\4":
            bad.append("not a v4 index")
        # a copy: every mtime differs, the content is the same -> hashed -> clean (v4 path compression read)
        _touch_all(r)
        if version.dirty(r):
            bad.append("v4 index, copied (new mtimes): dirty")
        # CRLF in the working tree (core.autocrlf on Windows) = the same content
        with open(os.path.join(r, "d", "b.txt"), "wb") as f:
            f.write(b"two\r\nthree\r\n")
        _touch_all(r, 900)
        if version.dirty(r):
            bad.append("CRLF copy counted as dirty")
        with open(os.path.join(r, "d", "e", "c.txt"), "wb") as f:
            f.write(b"y" * 100)                                 # the same size, a new content
        _touch_all(r, 1300)
        if not version.dirty(r):
            bad.append("v4: a changed file not seen")
        # the time limit: a clock that runs 0.2 s per read -> gives up after ~0.5 s = no mark
        calls = []
        real = version._clock

        def slow_clock():
            calls.append(1)
            return len(calls) * 0.2
        version._clock = slow_clock
        try:
            got = version.dirty(r)
        finally:
            version._clock = real
        if got is not False or len(calls) > 5:
            bad.append("time limit: %r after %d clock reads" % (got, len(calls)))
        if version.dirty(r, limit_s=0.0) is not False:
            bad.append("limit 0 still marks")
        # cached: text() / commit() scan once per repo per process
        n = []
        real_d = version.dirty
        version.dirty = lambda repo=version.REPO, **k: (n.append(repo), True)[1]
        version.clear_cache()
        try:
            texts = [version.text() for _ in range(4)] + [version.commit(r) for _ in range(3)]
        finally:
            version.dirty = real_d
            version.clear_cache()
        if len(n) != 2:
            bad.append("not cached: %d scans for 7 calls" % len(n))
        if not texts[-1].endswith("+dirty"):
            bad.append("cached mark lost %r" % texts[-1])
    report("LK16 M-B: version '+dirty' cached per process, gives up after %.1f s (no mark); no .git / empty .git, "
           "a v4 index, CRLF, a copied tree" % version.DIRTY_LIMIT_S, not bad, "; ".join(bad) or "all")


def lk17():
    bad = []
    real_step, real_fail, real_box = splash.Splash.step, splash.Splash.fail, splash.message_box

    def broken(self, *a):
        raise tk.TclError("bad window path name")
    boxes = []
    splash.Splash.step = broken
    try:
        bs = splash.BackgroundSplash(lang="en", fullscreen=False, max_s=20)
        bs.step("splash.p_station")
        time.sleep(0.4)
        bs.close(wait_s=4)
        if bs.th.is_alive():
            bad.append("BackgroundSplash: close() ignored after a failing message (stray splash)")
        t0 = time.time()
        ok, _ = splash.run_task(lambda p: (p("splash.p_sim"), time.sleep(0.3), (True, ""))[2], ["splash.p_sim"],
                                lang="en", fullscreen=False, max_s=20, on_error=lambda t: None)
        if time.time() - t0 > 6 or not ok:
            bad.append("run_task: the splash stayed %.1f s after a failing step" % (time.time() - t0))
        splash.Splash.step = real_step
        splash.Splash.fail = broken
        splash.message_box = lambda title, text: boxes.append(text)
        bs2 = splash.BackgroundSplash(lang="en", fullscreen=False, max_s=20)
        bs2.fail("RuntimeError: no data")
        bs2.wait(5)
        if bs2.th.is_alive() or not boxes or "no data" not in boxes[0]:
            bad.append("a failing error display: alive %s, boxes %r" % (bs2.th.is_alive(), boxes))
    finally:
        splash.Splash.step, splash.Splash.fail, splash.message_box = real_step, real_fail, real_box
    report("LK17 L1: the splash keeps polling after a failing message (close() still closes it, an error still "
           "shown)", not bad, "; ".join(bad) or "3 cases")


def lk18():
    import json
    import subprocess
    demo = _demo()
    bad = []
    saved = dict(RUN=demo.RUN, PIDS=demo.PIDS, LOGS=demo.LOGS, kill_tree=demo.kill_tree)
    tmp = tempfile.mkdtemp(prefix="trygons_lk18_")
    demo.RUN, demo.PIDS, demo.LOGS = tmp, os.path.join(tmp, "pids.json"), tmp
    killed = []
    demo.kill_tree = killed.append
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], creationflags=demo.NO_WINDOW)
    try:
        ident = demo.proc_ident(other.pid)
        if not ident:
            bad.append("no creation time for a running process")
        # a stale pids.json after a crash: the pid now belongs to "another program" (other creation time)
        for entry in ({"simulator": other.pid, "_id": {"simulator": "1234"}},     # another process now
                      {"gateway": other.pid}):                                    # an old file: pid only
            with open(demo.PIDS, "w", encoding="utf-8") as f:
                json.dump(entry, f)
            demo.stop()
            if killed or other.poll() is not None:
                bad.append("killed by pid alone: %r" % entry)
        with open(demo.PIDS, "w", encoding="utf-8") as f:
            json.dump({"simulator": other.pid, "_id": {"simulator": ident}}, f)
        demo.stop()
        if killed != [other.pid]:
            bad.append("the demo's own process not stopped: %r" % killed)
        # launch() stores the creation time
        pids = {}
        p = demo.launch("probe", [sys.executable, "-c", "import time; time.sleep(30)"], tmp, pids)
        try:
            if not pids.get("_id", {}).get("probe") or pids["_id"]["probe"] != demo.proc_ident(p.pid):
                bad.append("launch did not store the creation time: %r" % pids)
        finally:
            p.kill()
            p.wait()
        if demo.proc_ident(p.pid) is not None:
            bad.append("a finished process still has an identity")
    finally:
        other.kill()
        other.wait()
        for k, v in saved.items():
            setattr(demo, k, v)
    report("LK18 L2: STOP_DEMO kills a pids.json entry only when pid + creation time are the demo's (a stale pid "
           "after a crash is never killed)", not bad, "; ".join(bad) or "stale / legacy refused, own stopped")


def lk19():
    bad = []
    focused = []
    real_ff = tk.Misc.focus_force

    def rec(self):
        focused.append(self)
        return real_ff(self)
    tk.Misc.focus_force = rec
    try:
        sp = splash.Splash(None, lang="en", fullscreen=False, max_s=20)
        if sp.w not in focused:
            bad.append("the splash window never takes the focus (Esc would not work)")
        sp.close()
    finally:
        tk.Misc.focus_force = real_ff
    root = tk.Tk()
    root.withdraw()
    frames = 0
    try:
        bs = splash.BackgroundSplash(lang="en", what="splash.what_station", steps=["splash.p_station"],
                                     fullscreen=False, max_s=0.8)
        bs.step("splash.p_station")
        # an App()-like build on the main Tk, no update: many widgets + canvas items for ~2 s
        t0 = time.time()
        while time.time() - t0 < 2.0:
            f = tk.Frame(root)
            for i in range(40):
                tk.Label(f, text="row %d" % i).grid(row=i, column=0)
            c = tk.Canvas(f, width=400, height=300)
            for i in range(400):
                c.create_line(0, i, 400, 300 - i)
            c.grid(row=0, column=1)
            f.pack()
            frames += 1
        if bs.state != "slow" or not bs.slow_seen:
            bad.append("watchdog did not run next to the busy main Tk: %s" % bs.state)
        bs.close()
        if bs.th.is_alive() or bs.state != "closed":
            bad.append("splash not closed")
        if tk._default_root is not root:
            bad.append("the splash's Tk took the default root")
        root.deiconify()
        root.update()
        lab = tk.Label(root, text="after")
        lab.pack()
        root.update()
        if not lab.winfo_exists() or not root.winfo_viewable():
            bad.append("the main window does not work after the splash")
    finally:
        root.destroy()
    report("LK19 L6: the splash takes the focus (Esc) only for itself; BackgroundSplash runs next to a main Tk "
           "busy building an App-like window, the main window works after it", not bad,
           "; ".join(bad) or "%d frames built while the splash watched" % frames)


def lk20():
    import json
    import subprocess
    demo = _demo()
    bad = []
    saved = dict(RUN=demo.RUN, PIDS=demo.PIDS, LOGS=demo.LOGS, kill_tree=demo.kill_tree, say=demo.say)
    tmp = tempfile.mkdtemp(prefix="trygons_lk20_")
    demo.RUN, demo.PIDS, demo.LOGS = tmp, os.path.join(tmp, "pids.json"), tmp
    killed, said = [], []
    demo.kill_tree = killed.append
    demo.say = said.append
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], creationflags=demo.NO_WINDOW)
    own = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], creationflags=demo.NO_WINDOW)
    try:
        own_id = demo.proc_ident(own.pid)
        with open(demo.PIDS, "w", encoding="utf-8") as f:
            json.dump({"simulator": other.pid, "gateway": own.pid, "station": 999999,
                       "_id": {"simulator": "1234", "gateway": own_id}}, f)
        demo.stop()
        if killed != [own.pid]:
            bad.append("killed %r (want only the matching one)" % killed)
        left = json.load(open(demo.PIDS, encoding="utf-8")) if os.path.exists(demo.PIDS) else None
        if left != {"simulator": other.pid, "_id": {"simulator": "1234"}}:
            bad.append("pids.json after a refusal: %r" % left)
        if not any(str(other.pid) in t and "tries again" in t for t in said):
            bad.append("the refused pid not printed: %r" % said)
        del killed[:]
        demo.stop()                                  # the second STOP_DEMO sees it again (still refused)
        if killed or not os.path.exists(demo.PIDS):
            bad.append("second stop: killed %r, file %s" % (killed, os.path.exists(demo.PIDS)))
        with open(demo.PIDS, "w", encoding="utf-8") as f:
            json.dump({"gateway": own.pid, "_id": {"gateway": own_id}}, f)
        demo.stop()
        if os.path.exists(demo.PIDS):
            bad.append("nothing refused but pids.json kept")
    finally:
        other.kill()
        other.wait()
        own.kill()
        own.wait()
        for k, v in saved.items():
            setattr(demo, k, v)
    report("LK20 LOW-1: STOP_DEMO keeps refused pids.json entries + prints their pids (a second STOP_DEMO "
           "tries again); removed when nothing was refused", not bad, "; ".join(bad) or "3 cases")


def lk21():
    bad = []
    errors = []
    real_facts, real_rce = splash.BackgroundSplash._facts, tk.Tk.report_callback_exception
    calls = []

    def facts(self, sp):
        calls.append(1)
        if len(calls) > 1:                           # the first one (before the poll) works
            raise RuntimeError("facts broken")
        return real_facts(self, sp)

    def rce(self, *a):
        errors.append(a[1] if len(a) > 1 else a)
    splash.BackgroundSplash._facts = facts
    tk.Tk.report_callback_exception = rce
    try:
        bs = splash.BackgroundSplash(lang="en", fullscreen=False, max_s=20)
        bs.step("splash.p_station")
        time.sleep(0.8)
        bs.close(wait_s=4)
        if bs.th.is_alive():
            bad.append("close() did not close it")
        if len(calls) < 3:
            bad.append("poll not running (%d facts calls)" % len(calls))
        if errors:
            bad.append("%d Tk callback errors: %r" % (len(errors), errors[:2]))
    finally:
        splash.BackgroundSplash._facts = real_facts
        tk.Tk.report_callback_exception = real_rce
    report("LK21 LOW-2: a failing _facts() in BackgroundSplash.poll never raises out of the Tk callback, "
           "close() still works", not bad, "; ".join(bad) or "%d polls, no error" % len(calls))


def lk22():
    import threading as th
    bad = []
    n = []
    real_d = version.dirty

    def slow(repo=version.REPO, **k):
        n.append(repo)
        time.sleep(0.3)
        return True
    version.dirty = slow
    version.clear_cache()
    try:
        out = []
        ts = [th.Thread(target=lambda: out.append(version.dirty_cached(REPO))) for _ in range(2)]
        for t in ts:
            t.start()
        for t in ts:
            t.join(5)
        if len(n) != 1 or out != [True, True]:
            bad.append("two threads: %d scans, results %r" % (len(n), out))
    finally:
        version.dirty = real_d
        version.clear_cache()
    r = _git_repo({"small.txt": b"one\n", "big.bin": b"z" * 200})
    if r is None:
        bad.append("git not found")
    else:
        with open(os.path.join(r, "big.bin"), "wb") as f:
            f.write(b"y" * 200)                              # changed, same size, new mtime
        _touch_all(r, 700)
        cap = version.HASH_MAX_BYTES
        if not version.dirty(r):
            bad.append("a changed file below the cap not seen")
        version.HASH_MAX_BYTES = 100
        reads = []
        real_open = open
        import builtins

        def spy(fp, *a, **k):
            if str(fp).endswith("big.bin"):
                reads.append(fp)
            return real_open(fp, *a, **k)
        builtins.open = spy
        try:
            got = version.dirty(r)
        finally:
            builtins.open = real_open
            version.HASH_MAX_BYTES = cap
        if got is not False or reads:
            bad.append("above the cap: %r, read %d times" % (got, len(reads)))
    report("LK22 LOW-3: dirty_cached = ONE scan with two threads (per-repo lock); a changed file above "
           "HASH_MAX_BYTES is never read (unknown, no mark)", not bad, "; ".join(bad) or "both")


def lk23():
    """BATTLE SHORT (13n1): any input refused, nothing sent, nothing changes, no lockout, no code anywhere."""
    import battleshort
    import settings as st
    root = tk.Tk()
    root.withdraw()
    logs = []
    before = dict(st._load() if hasattr(st, "_load") else {})
    w = battleshort.BattleShort(root, logs.append)
    tries = ["", "0000", "1234", "BATTLESHORT", "x" * 300, "' OR 1=1 --", "ΚΩΔΙΚΟΣ", "\x00", "admin", "4711"]
    texts, still = [], True
    for t in tries:
        w.var.set(t)
        w.authorise()
        root.update()
        texts.append(w.msg.cget("text"))
        still = still and str(w.entry.cget("state")) == "normal" and str(w.b_auth.cget("state")) == "normal"
    refused = all(x == i18n.T("bs.invalid") for x in texts)
    always_false = not any(battleshort.attempt(t) for t in tries + [str(i) for i in range(10000)])
    logged = logs == [battleshort.LOG_LINE] * len(tries)
    after = dict(st._load() if hasattr(st, "_load") else {})
    unchanged = before == after
    # nothing in the module can reach a boat / gateway, and no code is stored: no link / socket import, no hash,
    # no comparison of the typed text
    src = open(os.path.join(HERE, "battleshort.py"), encoding="utf-8").read()
    code = "\n".join(ln for ln in src.split("\n") if not ln.strip().startswith("#"))
    code = re.sub(r'"""[\s\S]*?"""', "", code)
    imports = set(re.findall(r"^\s*(?:import|from)\s+([a-zA-Z0-9_.]+)", code, re.M))
    no_path = imports <= {"tkinter", "i18n", "theme"}
    no_code = not re.search(r"hashlib|sha\d|md5|hmac|==\s*[\"']|secret|password|\bsend\b|socket|link", code)
    root.deiconify()
    w.w.deiconify()
    root.update()
    vertical = w.b_auth.winfo_y() < w.b_cancel.winfo_y()
    masked = w.entry.cget("show") != ""
    w.close()
    root.update()
    closed = not w.alive()
    root.destroy()
    report("LK23 BATTLE SHORT (13n1): every input = INVALID AUTHORISATION CODE, no lockout, nothing sent / changed "
           "(settings unchanged), only the log line, no code / hash / link anywhere; masked field, AUTHORISE above "
           "CANCEL, Cancel closes", refused and always_false and still and logged and unchanged and no_path and
           no_code and vertical and masked and closed,
           "refused %s (texts %s), attempt() never True %s, field + button still usable %s, log %s, settings unchanged "
           "%s, imports %s, no code / hash / link %s, vertical %s, masked %s, closed %s" % (
               refused, sorted(set(texts)), always_false, still, logs[:2], unchanged, sorted(imports), no_code,
               vertical, masked, closed))


def lk24():
    """The 3 text sizes + "Aa" (13n1): every level is one of three sizes; Aa = small / normal / large, saved per
    PC, x 0.85 / 1 / 1.2 for widgets, drawn text and the HUD; the colour code: STANDBY the only red button kind."""
    import settings as st
    sizes = {theme.PT[lv] for lv in theme.SIZE_OF}
    pxs = {theme.PX[lv] for lv in theme.SIZE_OF}
    three = len(sizes) == 3 and len(pxs) == 3 and set(theme.SIZE_OF.values()) == set(theme.SIZES)
    hs = theme.hud_sizes(1080)
    hud3 = len(set(hs.values())) == 3
    seen = []
    theme.on_text_size(seen.append)
    theme.set_text_size("normal")
    f_n, p_n, h_n = theme.font("label")[1], theme.px("label", 1920, 1080), theme.hud_sizes(1080)["norm"]
    out = []
    for _ in range(3):
        theme.cycle_text_size()
        out.append((theme.text_size(), theme.font("label")[1], theme.px("label", 1920, 1080),
                    theme.hud_sizes(1080)["norm"], st.get("text_size")))
    cyc = [o[0] for o in out] == ["large", "small", "normal"] and all(o[0] == o[4] for o in out)
    big = out[0][1] > f_n and out[0][2] > p_n and out[0][3] > h_n
    small = out[1][1] < f_n and out[1][2] < p_n and out[1][3] < h_n
    told = len(seen) == 3 and abs(seen[0] - 1.2) < 1e-6
    # rescale_tree really changes a widget's font
    root = tk.Tk()
    root.withdraw()
    lab = tk.Label(root, text="x", font=theme.font("label"))
    s0 = tk.font.Font(root=root, font=lab.cget("font")).actual()["size"]
    theme.rescale_tree(root, 1.5)
    s1 = tk.font.Font(root=root, font=lab.cget("font")).actual()["size"]
    root.destroy()
    rescaled = s1 > s0
    # the colour code: no button kind but STANDBY is red; the info text colours are not the warning red
    red = {k for k in ("normal", "ok", "no", "info", "warn", "sim", "standby")
           if theme.button_opts(k)["bg"].lower() == theme.WARN.lower()}
    reds_ok = red == {"standby"} and all(theme.THEMES[t]["bad_txt"].lower() != theme.WARN.lower() and
                                         theme.THEMES[t]["btn_no"].lower() != theme.WARN.lower() for t in theme.THEMES)
    square = theme.BUTTON["radius"] == 0 and theme.FRAME >= 1 and theme.button_opts()["relief"] == "solid"
    report("LK24 3 text sizes + Aa (13n1): every level = one of 3 sizes (widgets, drawn, HUD), Aa cycles small / "
           "normal / large (saved per PC, every window told), rescale_tree resizes widgets; red = STANDBY only, "
           "square frames", three and hud3 and cyc and big and small and told and rescaled and reds_ok and square,
           "sizes pt %s px %s hud %s; cycle %s; told %s; widget %s -> %s; red kinds %s; square %s" % (
               sorted(sizes), sorted(pxs), sorted(set(hs.values())), out, seen, s0, s1, sorted(red), square))


def lk25():
    """13n1 review: M1 the tools window scaled once; L1 no drift; L7 the question serial is per process."""
    import types
    import app as A
    import driver as D
    root = tk.Tk()
    root.withdraw()
    tools = tk.Toplevel(root)
    tools.withdraw()
    lab_root = tk.Label(root, text="x", font=(theme.FAMILY, 20, "normal"))
    lab_tools = tk.Label(tools, text="x", font=(theme.FAMILY, 20, "normal"))
    size = lambda w: tk.font.Font(root=root, font=w.cget("font")).actual()["size"]
    fake = types.SimpleNamespace(root=root, tools=tools, render=lambda force=False: None)
    A.App.text_size_changed(fake, 1.2)
    once = size(lab_root) == 24 and size(lab_tools) == 24
    got_once = (size(lab_root), size(lab_tools))
    # back and forth 0.85 / 1 / 1.2 many times: back at the start size (no rounding drift)
    lab = tk.Label(root, text="y", font=(theme.FAMILY, 11, "normal"))
    seq = []
    for _ in range(20):
        for r in (1.2, 1 / 1.2 * 0.85, 1 / 0.85):
            theme.rescale_tree(lab, r)
        seq.append(size(lab))
    no_drift = seq == [11] * 20
    # a font the program set meanwhile starts again from its new size
    lab.configure(font=(theme.FAMILY, 30, "normal"))
    theme.rescale_tree(lab, 0.5)
    reset = size(lab) == 15
    root.destroy()
    # L7: two DriverScreens (the method on fakes) never give the same serial
    fakes = [types.SimpleNamespace() for _ in range(2)]
    sers = []
    for f in fakes:
        D._QSERIAL += 1              # what ask() does, without a window
        f.qserial = D._QSERIAL
        sers.append(f.qserial)
    src_ok = "global _QSERIAL" in src("groundstation", "driver.py") and         'getattr(self, "qserial", 0) + 1' not in src("groundstation", "driver.py")
    report("LK25 (13n1 review M1 / L1 / L7) Aa: the tools window rescaled once (not ratio squared); no rounding drift "
           "over 20 small / normal / large rounds; a new font restarts from its size; question serials per process",
           once and no_drift and reset and sers[0] != sers[1] and src_ok,
           "x1.2 of 20 pt: root / tools %s (want 24 / 24); 11 pt after each round %s; 30 pt x0.5 -> 15 %s; serials %s, "
           "module counter %s" % (got_once, seq[:5], reset, sers, src_ok))


def lk26():
    """13n2: TOUR.bat = the guided tour on its own (Greek), no demo needed."""
    import subprocess
    bad = []
    path = os.path.join(REPO, "TOUR.bat")
    bat = src("TOUR.bat") if os.path.exists(path) else ""
    if not bat:
        bad.append("TOUR.bat missing")
    if "academy\\tour_app.py --lang el" not in bat:
        bad.append("TOUR.bat does not start academy\\tour_app.py --lang el")
    if "where pythonw" in bat or "import sys, tkinter, PIL" not in bat or "%%~dpIpythonw.exe" not in bat:
        bad.append("TOUR.bat does not check the Python before its pythonw")
    if any(w in bat.lower() for w in ("demo.py", "autopilot_sim", "gateway.py")):
        bad.append("TOUR.bat starts something besides the tour")
    alive = None
    if not bad:
        env = dict(os.environ, TRYGONS_ACADEMY_DATA=os.environ["TRYGONS_ACADEMY_DATA"])
        pr = subprocess.Popen([sys.executable, os.path.join(REPO, "academy", "tour_app.py"), "--lang", "el"], cwd=REPO,
                              env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(4.0)
        alive = pr.poll() is None
        if alive:
            pr.kill()
        out, err = pr.communicate(timeout=10)
        if not alive:
            bad.append("tour_app.py ended: %s" % err.decode(errors="replace")[-300:])
    report("LK26 (13n2) TOUR.bat: the guided tour on its own in Greek (academy\\tour_app.py --lang el), pythonw only "
           "next to the checked Python, nothing else started; the tour opens without a crash",
           not bad, "; ".join(bad) or "TOUR.bat ok, tour running after 4 s %s" % alive)


def main():
    i18n.set_lang("el", save=False)
    lk1()
    lk2()
    lk3()
    root = tk.Tk()
    root.withdraw()
    lk4(root)
    lk5()
    lk6(root)
    lk7(root)
    root.destroy()
    lk8()
    lk9()
    lk10()
    lk11()
    lk12()
    lk13()
    lk14()
    lk15()
    lk16()
    lk17()
    lk18()
    lk19()
    lk20()
    lk21()
    lk22()
    lk23()
    lk24()
    lk25()
    lk26()
    n = len(results)
    print("LOOK TESTS: %d/%d %s" % (sum(results), n, "PASS" if all(results) else "FAIL"))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
