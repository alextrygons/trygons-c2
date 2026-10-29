"""Menu "Academy" of the ground station (CLAUDE.md section S, ACADEMY_SPEC A, step 16).

DISPLAY + one file hand-over only - nothing here changes the driving:
  * "Open the academy" starts academy/academy_app.py in its OWN process and window with only the
    screen language (--lang). The station's boat link is NEVER handed to the academy (owner W31,
    decision D10): the academy runs only against the simulator / recordings (academy/simguard.py).
  * "Send the academy levels to the gateway" sends the academy's SIGNED levels file
    (academy/data/push/levels.json, made in the academy window) over this station's gateway login
    ({"t": "academy_levels", "doc": ...}). The gateway checks the signature (gateway/trainlevels.py),
    keeps its last copy and uses the levels at the next login only (never mid-trip).
  * "Guided tour" (step 17, ACADEMY_SPEC B) starts academy/tour_app.py in its OWN process (only --lang)
    and prepares the tour's Greek / English voice with THIS station's offline TTS (sound.Library.make,
    in the background): the academy itself runs no process, it only plays the cached .wav files.
    The sentences come from academy/tour.py, loaded by its path (its display tables only, no link).
  * "Open the academy" also writes this station's alarm tones (sound.tone: warning / caution / advisory)
    into its sound cache (<cache>/tones/<name>.wav) for the academy's alarm-sound quiz questions (step 18,
    ACADEMY_SPEC D); the academy only plays them (academy/quiz.py tone_path, simguard.check_recording).
  * "Practical test (this boat)" / "Test area (instructor)" (step 21, ACADEMY_SPEC G + J): practicalmenu.py -
    the one academy part on this station's boat link, read only + the CONFIRM_ONLY question (LinkView).
"""
import importlib.util
import json
import os
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog

import i18n
import practicalmenu
import theme
from i18n import T

HERE = os.path.dirname(os.path.abspath(__file__))
ACADEMY_DIR = os.path.join(os.path.dirname(HERE), "academy")
ACADEMY_APP = os.path.join(ACADEMY_DIR, "academy_app.py")
TOUR_APP = os.path.join(ACADEMY_DIR, "tour_app.py")
TOUR_MODEL = os.path.join(ACADEMY_DIR, "tour.py")
_tour = None
ACADEMY_TONES = ("warning", "caution", "advisory")      # = academy/quiz.py SOUNDS (test_academy AC27)
TONE_DIR = "tones"                                       # = academy/quiz.py TONE_DIR


def push_file(env=None):
    """Where the academy writes its gateway file (academy/accounts.py Store.push_path)."""
    env = os.environ if env is None else env
    data = env.get("TRYGONS_ACADEMY_DATA") or os.path.join(ACADEMY_DIR, "data")
    return os.path.join(data, "push", "levels.json")


def academy_command():
    """The academy's own process: the script + the screen language, NOTHING about the link."""
    return [sys.executable, ACADEMY_APP, "--lang", i18n.lang()]


def tour_command():
    """The tour's own process: the script + the screen language, NOTHING about the link."""
    return [sys.executable, TOUR_APP, "--lang", i18n.lang()]


def tour_model():
    """academy/tour.py, loaded by its path under its own name (no academy folder on sys.path)."""
    global _tour
    if _tour is None:
        spec = importlib.util.spec_from_file_location("academy_tour", TOUR_MODEL)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _tour = mod
    return _tour


def tour_sentences(lang):
    return tour_model().sentences(lang)


def prepare_tour_voice(app, lang=None, run=None):
    """Make the missing tour sentences with this station's TTS, in the background (never blocks the
    screen). No sound system (TRYGONS_SOUND=off, tests) = nothing. Returns the thread (or None)."""
    snd = getattr(app, "sound", None)
    if snd is None or os.environ.get("TRYGONS_SOUND") == "off":
        return None
    lang = lang or i18n.lang()
    try:
        texts = tour_sentences(lang)
    except Exception as e:                                  # a broken tour file: the tour shows text only
        app.say(T("acad.tour_bad", why=str(e)), theme.WARN)
        return None
    th = threading.Thread(target=run or snd.lib.make, args=(lang, texts), daemon=True)
    th.start()
    return th


def open_tour(app, popen=subprocess.Popen):
    prepare_tour_voice(app)
    try:
        p = popen(tour_command(), cwd=ACADEMY_DIR)
        app.say(T("acad.tour_opened"))
        return p
    except OSError as e:
        app.say(T("acad.open_failed", why=str(e)), theme.WARN)
        return None


def add_menu(app, mb):
    m = tk.Menu(mb, tearoff=0)
    m.add_command(label=T("menu.academy_open"), command=lambda: open_academy(app))
    m.add_command(label=T("menu.academy_tour"), command=lambda: open_tour(app))
    m.add_command(label=T("menu.academy_send"), command=lambda: send_levels(app))
    m.add_separator()
    # step 21: the PRACTICAL TEST on THIS boat's link (read only + CONFIRM_ONLY: practicalmenu.py)
    m.add_command(label=T("menu.academy_practical"), command=lambda: practicalmenu.open_practical(app))
    m.add_command(label=T("menu.academy_area"), command=lambda: practicalmenu.open_area(app))
    mb.add_cascade(label=T("menu.academy"), menu=m)
    return m


def tone_file(cache, name):
    return os.path.join(cache, TONE_DIR, name + ".wav")


def prepare_academy_sounds(app):
    """The alarm tones for the academy's sound questions, from THIS station's sound module (made in code,
    no TTS): written once into its sound cache. No sound system (TRYGONS_SOUND=off) = nothing. Returns the
    files written."""
    snd = getattr(app, "sound", None)
    if snd is None or os.environ.get("TRYGONS_SOUND") == "off":
        return []
    import sound
    out = []
    for name in ACADEMY_TONES:
        path = tone_file(snd.lib.cache, name)
        if not os.path.exists(path):
            try:
                sound.write_wav(path, sound.tone(name))
                out.append(path)
            except OSError as e:
                app.say(T("acad.open_failed", why=str(e)), theme.WARN)
                break
    return out


def open_academy(app, popen=subprocess.Popen):
    prepare_academy_sounds(app)
    try:
        p = popen(academy_command(), cwd=ACADEMY_DIR)
        app.say(T("acad.opened"))
        return p
    except OSError as e:
        app.say(T("acad.open_failed", why=str(e)), theme.WARN)
        return None


def send_levels(app, path=None):
    """The signed levels file -> the gateway (only over a gateway login)."""
    from gwclient import GatewayLink
    if not isinstance(getattr(app, "link", None), GatewayLink):
        return app.say(T("acad.only_gw"), theme.WARN)
    if path is None:
        f = push_file()
        path = filedialog.askopenfilename(parent=app.root, title=T("menu.academy_send"),
                                          initialdir=os.path.dirname(f), initialfile=os.path.basename(f),
                                          filetypes=[("JSON", "*.json")])
        if not path:
            return None
    try:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
    except (OSError, ValueError) as e:
        return app.say(T("acad.bad_file", why=str(e)), theme.WARN)
    if not isinstance(doc, dict) or doc.get("kind") != "academy_levels":
        return app.say(T("acad.not_levels"), theme.WARN)
    app.link.send_msg({"t": "academy_levels", "doc": doc})
    app.event(T("acad.sent", n=len(doc.get("records") or [])))
    return doc


def on_ack(app, d):
    txt = i18n.from_en(d.get("text", ""))
    app.say(T("acad.answer", text=txt), None if d.get("ok") else theme.WARN)
    app.event(T("acad.answer", text=txt))
