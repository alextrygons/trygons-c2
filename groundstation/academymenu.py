"""Menu "Academy" of the ground station (CLAUDE.md section S, ACADEMY_SPEC A, step 16).

DISPLAY + one file hand-over only - nothing here changes the driving:
  * "Open the academy" starts academy/academy_app.py in its OWN process and window with only the
    screen language (--lang). The station's boat link is NEVER handed to the academy (owner W31,
    decision D10): the academy runs only against the simulator / recordings (academy/simguard.py).
  * "Send the academy levels to the gateway" sends the academy's SIGNED levels file
    (academy/data/push/levels.json, made in the academy window) over this station's gateway login
    ({"t": "academy_levels", "doc": ...}). The gateway checks the signature (gateway/trainlevels.py),
    keeps its last copy and uses the levels at the next login only (never mid-trip).
"""
import json
import os
import subprocess
import sys
import tkinter as tk
from tkinter import filedialog

import i18n
import theme
from i18n import T

HERE = os.path.dirname(os.path.abspath(__file__))
ACADEMY_DIR = os.path.join(os.path.dirname(HERE), "academy")
ACADEMY_APP = os.path.join(ACADEMY_DIR, "academy_app.py")


def push_file(env=None):
    """Where the academy writes its gateway file (academy/accounts.py Store.push_path)."""
    env = os.environ if env is None else env
    data = env.get("TRYGONS_ACADEMY_DATA") or os.path.join(ACADEMY_DIR, "data")
    return os.path.join(data, "push", "levels.json")


def academy_command():
    """The academy's own process: the script + the screen language, NOTHING about the link."""
    return [sys.executable, ACADEMY_APP, "--lang", i18n.lang()]


def add_menu(app, mb):
    m = tk.Menu(mb, tearoff=0)
    m.add_command(label=T("menu.academy_open"), command=lambda: open_academy(app))
    m.add_command(label=T("menu.academy_send"), command=lambda: send_levels(app))
    mb.add_cascade(label=T("menu.academy"), menu=m)
    return m


def open_academy(app, popen=subprocess.Popen):
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
