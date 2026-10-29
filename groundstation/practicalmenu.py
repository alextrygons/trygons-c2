"""Menu Academy -> "Practical test (this boat)" / "Test area (instructor)" (ACADEMY_SPEC G + J, step 21).

The academy's PRACTICAL TEST is the one academy part on the NORMAL boat link (design D4): it runs in THIS
station's process and sees the boat only through academy/practical/linkview.LinkView(app) - the telemetry this
station already receives, its event / audit log, and ONE frame it may send: PROTO_T_CONFIRM_ONLY (the final
approval: a question for the onboard joystick CONFIRM that changes nothing on the boat). It never gets the link
object itself. The academy's own window (academymenu.open_academy) stays its own process on the simulator only.
"""
import os
import sys
import tkinter as tk

import theme
from i18n import T

HERE = os.path.dirname(os.path.abspath(__file__))
PRACTICAL_DIR = os.path.join(os.path.dirname(HERE), "academy", "practical")


def _modules():
    """academy/practical (its modules put academy/ at the END of sys.path: the station's modules win)."""
    if PRACTICAL_DIR not in sys.path:
        sys.path.append(PRACTICAL_DIR)
    import linkview
    import ptest_app
    import accounts
    import text
    import i18n
    text.set_lang(i18n.lang())
    return linkview, ptest_app, accounts


def _open(app, which):
    try:
        linkview, ptest_app, accounts = _modules()
        store = accounts.Store()
    except Exception as e:                                  # e.g. on a gateway PC, a broken academy folder
        app.say(T("acad.open_failed", why=str(e)), theme.WARN)
        return None
    view = linkview.LinkView(app)
    cls = ptest_app.PracticalWindow if which == "test" else ptest_app.AreaWindow
    w = cls(tk.Toplevel(app.root), view, store)
    app.say(T("acad.practical_opened"))
    return w


def open_practical(app):
    return _open(app, "test")


def open_area(app):
    return _open(app, "area")
