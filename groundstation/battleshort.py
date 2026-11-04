"""BATTLE SHORT (step 13n1, owner 2026-10-31 E1) - DEMO THEATRE ONLY.

The grey button with the black / yellow hazard stripes at the end of screen 2's mode bar opens this window: a
dark window with a red border, the warning text (lang: bs.*), a masked AUTHORISATION CODE field, [AUTHORISE]
on top and [CANCEL] underneath (the ONE vertical layout of step 13p).

There is NO code anywhere: no stored code, no hash, no comparison that could ever pass. EVERY input - empty,
any text, any length - gives INVALID AUTHORISATION CODE (a red flash + a shake of the window). There is no
lockout (the field stays usable), no function behind it and NO path to the boat or the gateway: this module
imports nothing that talks to a link, and the only thing it does besides drawing is the station log line
"battle short attempt" (the `log` function the station gives it). test_look.py LK10 proves all of it.
DISPLAY ONLY.
"""
import tkinter as tk

import i18n
import theme
from i18n import T

LOG_LINE = "battle short attempt"           # the station's event log (English, like every log line)
SHAKE_PX = (14, -14, 10, -10, 6, -6, 2, -2, 0)


def attempt(text):
    """The ONLY judgement of an authorisation code: there is none that works. Always False - no code exists."""
    del text
    return False


class BattleShort:
    def __init__(self, parent, log=None):
        Z = theme.HAZARD
        self.log = log or (lambda s: None)
        self.attempts = 0                   # tests: how many tries (no lockout, nothing else counted)
        w = self.w = tk.Toplevel(parent, bg=Z["red"])
        w.title(i18n.W("bs.title"))
        w.transient(parent)
        w.resizable(False, False)
        inner = tk.Frame(w, bg=Z["bg"])
        inner.pack(padx=6, pady=6, fill="both", expand=True)          # the red border
        self.inner = inner
        tk.Label(inner, text=T("bs.head"), font=theme.font("title"), bg=Z["bg"], fg=Z["red"]).pack(
            padx=24, pady=(18, 8))
        tk.Label(inner, text=T("bs.text"), font=theme.font("label"), bg=Z["bg"], fg=theme.WHITE, wraplength=620,
                 justify="left").pack(padx=24, pady=(0, 12))
        tk.Label(inner, text=T("bs.code"), font=theme.font("value"), bg=Z["bg"], fg=theme.WHITE).pack(
            anchor="w", padx=24)
        self.var = tk.StringVar()
        self.entry = tk.Entry(inner, textvariable=self.var, show="●", font=theme.font("title", mono=True),
                              justify="center", bg=theme.BLACK, fg=theme.WHITE, insertbackground=theme.WHITE,
                              relief="solid", bd=1, highlightthickness=theme.FRAME, highlightbackground=Z["grey"],
                              highlightcolor=Z["red"])
        self.entry.pack(fill="x", padx=24, pady=(4, 6))
        self.msg = tk.Label(inner, text="", font=theme.font("value"), bg=Z["bg"], fg=Z["red"])
        self.msg.pack(pady=(0, 6))
        self.b_auth = tk.Button(inner, text=T("bs.authorise"), command=self.authorise,
                                **dict(theme.button_opts(), bg=Z["grey"], fg=theme.BLACK, font=theme.font("value")))
        self.b_auth.pack(fill="x", padx=24, pady=(2, 4))
        self.b_cancel = tk.Button(inner, text=T("bs.cancel"), command=self.close,
                                  **dict(theme.button_opts(), font=theme.font("value")))
        self.b_cancel.pack(fill="x", padx=24, pady=(4, 18))
        self.entry.bind("<Return>", lambda e: self.authorise())
        w.bind("<Escape>", lambda e: self.close())
        self.entry.focus_set()

    def authorise(self):
        """Any input -> INVALID AUTHORISATION CODE. Nothing is sent, nothing changes, no lockout."""
        self.attempts += 1
        ok = attempt(self.var.get())        # always False
        self.var.set("")
        self.log(LOG_LINE)
        if not ok:
            self.msg.configure(text=T("bs.invalid"))
            self.flash(6)
            self.shake(0)
        self.entry.focus_set()

    def flash(self, n):
        Z = theme.HAZARD
        try:
            self.inner.configure(bg=Z["red"] if n % 2 else Z["bg"])
            self.msg.configure(bg=Z["red"] if n % 2 else Z["bg"], fg=theme.WHITE if n % 2 else Z["red"])
            if n > 0:
                self.w.after(90, lambda: self.flash(n - 1))
        except tk.TclError:
            pass

    def shake(self, i):
        try:
            if i == 0:
                self.x0, self.y0 = self.w.winfo_x(), self.w.winfo_y()
            if i < len(SHAKE_PX):
                self.w.geometry("+%d+%d" % (self.x0 + SHAKE_PX[i], self.y0))
                self.w.after(30, lambda: self.shake(i + 1))
        except tk.TclError:
            pass

    def close(self):
        try:
            self.w.destroy()
        except tk.TclError:
            pass

    def alive(self):
        try:
            return bool(self.w.winfo_exists())
        except tk.TclError:
            return False


def open_window(parent, log=None, old=None):
    """Open (or raise) the BATTLE SHORT window."""
    if old is not None and old.alive():
        old.w.lift()
        return old
    return BattleShort(parent, log)
