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

Run:  python test_look.py        (exit code 0 = all passed)
"""
import os
import re
import sys
import tempfile
import threading
import time
import tkinter as tk

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
        order.append(m.index("root.withdraw()") < m.index("Splash(root") < m.index("sp.close()") <
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
        for key in ("drv.title", "tools.title", "boat.title", "boat.confirm_title", "cap.cc_title", "cap.pw_title"):
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
    if not re.match(r"^[0-9a-f]{7}$", commit or ""):
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
    n = len(results)
    print("LOOK TESTS: %d/%d %s" % (sum(results), n, "PASS" if all(results) else "FAIL"))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
