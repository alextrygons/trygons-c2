"""Screenshots of every academy window (ACADEMY_SPEC I, step 22) -> docs/screens/academy/ + README.md.

    python tools/academy_screens.py            (needs a visible desktop: a screen grab, like demo_screens.py)

Every window opens on made-up demo accounts in a TEMPORARY academy data folder (never academy/data). The
lesson / scenario windows run on the real simulator (sim/autopilot_sim server 5760, started through
simguard like the academy does) when the simulator is built and port 5760 is free; otherwise they are
shown without it. The practical test window gets a made-up station (no boat, nothing is sent). Each window
in Greek and English.
"""
import datetime
import json
import os
import socket
import sys
import tempfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACADEMY = os.path.join(REPO, "academy")
OUT = os.path.join(REPO, "docs", "screens", "academy")
DATA = tempfile.mkdtemp(prefix="trygons_screens_")
os.environ["TRYGONS_ACADEMY_DATA"] = DATA
os.environ.setdefault("TRYGONS_SOUND", "off")
for d in (ACADEMY, os.path.join(ACADEMY, "practical"), os.path.join(REPO, "groundstation")):
    sys.path.insert(0, d)

import tkinter as tk                                                    # noqa: E402
from PIL import ImageGrab, Image                                        # noqa: E402

import accounts                                                         # noqa: E402
import text                                                             # noqa: E402

shots = []


def say(s):
    print(s, flush=True)


def pump(root, s):
    end = time.time() + s
    while time.time() < end:
        root.update()
        time.sleep(0.02)


def grab(root, win, name, what):
    win.lift()
    try:
        win.attributes("-topmost", True)
    except tk.TclError:
        pass
    pump(root, 0.8)
    x, y = win.winfo_rootx(), win.winfo_rooty()
    w, h = win.winfo_width(), win.winfo_height()
    im = ImageGrab.grab(bbox=(x, y, x + w, y + h), all_screens=True)
    os.makedirs(OUT, exist_ok=True)
    im.save(os.path.join(OUT, name + ".png"))
    try:
        win.attributes("-topmost", False)
    except tk.TclError:
        pass
    shots.append((name, what))
    say("  %s (%dx%d)" % (name, w, h))
    if win is not root:
        win.withdraw()                             # the next window is grabbed alone


def demo_store():
    st = accounts.Store(os.path.join(DATA, "accounts"), env={"PROGRAMDATA": tempfile.mkdtemp()})
    today = datetime.date.today()
    # the first set-up (bootstrap): the levels before the CAPTAIN instructor exists
    for user, name, level, ins in (("nikos", "Νίκος Κ. (demo)", "DRIVER", False), ("maria", "Μαρία Δ. (demo)", "OBSERVER", False),
                                   ("eleni", "Ελένη Π. (demo)", "CAPTAIN", True)):
        st.add(user, name=name, instructor=ins)
        st.set_level(user, level, expiry=today + datetime.timedelta(days=200 if level != "DRIVER" else 20))
    st.log("nikos", "lesson", id="l01_hdg_lock", done=True, score=90)
    st.log("nikos", "lesson", id="l02_takeover", done=True, score=100)
    st.log("nikos", "quiz", topic="q01_basics", score=62)
    st.log("nikos", "hours", hours=3.5)
    st.log("maria", "lesson", id="l01_hdg_lock", done=True, score=80)
    return st


def port_free(port):
    s = socket.socket()
    try:
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def main():
    import simguard
    proc = None
    if os.path.isfile(simguard.sim_exe()) and port_free(simguard.LESSON_PORT):
        proc = simguard.start_simulator()
        time.sleep(1.5)
        say("simulator started (pid %d)" % proc.pid)
    else:
        say("simulator not started (not built or port 5760 busy): the lesson windows without it")
    st = demo_store()
    import theme
    theme.dpi_aware()                              # window coordinates = screen pixels for the grab
    root = tk.Tk()
    root.withdraw()
    try:
        for lang in ("el", "en"):
            text.set_lang(lang)
            say("== %s" % lang)
            import academy_app
            top = tk.Toplevel(root)
            app = academy_app.AcademyApp(top, st)
            app.tree.selection_set("nikos")
            grab(root, top, "academy_main_%s" % lang, "the academy window: accounts, levels, expiry, weak areas, buttons")
            import tour_app
            t = tour_app.TourWindow(tk.Toplevel(root), lang, player=tour_app.NullPlayer())
            for _ in range(3):
                t.next()
            grab(root, t.root, "tour_%s" % lang, "the guided tour: a stop on the drawing of screen 1")
            import lesson_app
            lw = lesson_app.LessonWindow(tk.Toplevel(root), lang, store=st, user="nikos")
            if proc is not None:
                import lesson
                lw.begin(lesson.lessons()[0])
                for _ in range(80):
                    lw.step()
                    pump(root, 0.05)
            grab(root, lw.root, "lessons_%s" % lang, "simulator lessons by level (HDG LOCK lesson on the simulator)")
            lw.close()
            import quiz_app
            qw = quiz_app.QuizWindow(tk.Toplevel(root), lang, store=st, user="nikos", player=tour_app.NullPlayer(), seed=3)
            qw.start([d["id"] for d in qw.docs])
            grab(root, qw.root, "quizzes_%s" % lang, "a quiz question (choice / picture / sound)")
            import scenario_app
            sw = scenario_app.ScenarioWindow(tk.Toplevel(root), lang, store=st, user="nikos")
            if proc is not None:
                import scenario
                sw.begin([d for d in scenario.scenarios() if d["id"].startswith("s06")][0])
                for _ in range(80):
                    sw.step()
                    pump(root, 0.05)
            grab(root, sw.root, "scenarios_%s" % lang, "emergency scenarios with the academy's picture (AIS collision course)")
            sw.close()
            import videolesson_app
            vw = videolesson_app.VideoLessonWindow(tk.Toplevel(root), lang, store=st, user="nikos")
            grab(root, vw.root, "videolessons_%s" % lang, "real-trip video lessons (the list; no lesson made on this PC)")
            import exam_app
            ew = exam_app.ExamWindow(tk.Toplevel(root), lang, store=st, user="maria", player=tour_app.NullPlayer(), seed=1)
            grab(root, ew.root, "exams_%s" % lang, "the exams per level (refusal shown: required lessons first)")
            import sim3d_app
            w3 = sim3d_app.Sim3DWindow(tk.Toplevel(root), lang, store=st, user="nikos")
            w3.select("m01_leave_bay")
            grab(root, w3.root, "sim3d_%s" % lang, "the 3D simulator interface: backend, missions, drone photos (no 3D world yet)")
            import linkview
            import ptest_app

            class DemoStation(object):                      # a made-up station: nothing is sent anywhere
                t, t_rx, _evlog, link = {"mode_name": "STANDBY", "mode": 0, "power_fwd": 30, "power_rev": 5,
                                         "sea_state": 1, "est_lat": 37.82, "est_lon": 23.6}, time.time(), [], None

                def cmd(self, *a):
                    return None
            pw = ptest_app.PracticalWindow(tk.Toplevel(root), linkview.LinkView(DemoStation()), st)
            grab(root, pw.top, "practical_%s" % lang, "the practical test window (made-up station: start checks refuse - "
                                                     "no boat, placeholder area)")
            for w in list(root.winfo_children()):
                w.destroy()
    finally:
        root.destroy()
        if proc is not None:
            proc.terminate()
    with open(os.path.join(OUT, "README.md"), "w", encoding="utf-8") as f:
        f.write("# Academy screenshots (review)\n\nMade by `python tools/academy_screens.py` on %s with made-up demo "
                "accounts (a temporary data folder) and the simulator on this PC. Every academy window shows the banner "
                "«ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» / TRAINING – SIMULATION; the practical test window shows its own banner "
                "(real boat - it only watches).\n\n| File | What it shows |\n|---|---|\n" % datetime.date.today())
        for name, what in shots:
            f.write("| [%s.png](%s.png) | %s, %s |\n" % (name, name, what, "Greek" if name.endswith("_el") else "English"))
    say("README written, %d screenshots" % len(shots))


if __name__ == "__main__":
    main()
