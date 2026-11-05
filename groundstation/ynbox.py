"""The settings YES / NO questions in the ONE vertical Y/N layout (owner W53, step WA; DISPLAY ONLY).

Step 13p made every driving question vertical (the question, [YES] on top, [NO] underneath, full width, framed, the
selected one inverted = theme.yn_opts). The settings questions of the station (strict channel, emergency, monitor
trip test, HELM release UNCREWED, single-operator on, crew swap) were still the operating system's own message boxes;
ask() replaces them with the same look:

    ask(parent, title, text, warn=False, default=False) -> True (YES) / False (NO, Esc, closed)

Modal (like messagebox.askyesno), the theme's colours / fonts, keyboard: Up / Down select, Enter / Space answer with
the selection, Esc = NO. NO is pre-selected by default (these questions change a setting; a slip of Enter never says
YES). Nothing here sends anything: the caller does what it did after messagebox.askyesno.

Tests: AUTO (None = ask; True / False = answer at once without a window) and LAST (the last question asked).
"""
import tkinter as tk

import theme
from i18n import T

AUTO = None           # tests: True / False = the answer without a window
LAST = {}             # the last question: {"title", "text", "warn", "default", "answer"}


def build(parent, title, text, warn=False, default=False):
    """The window (not waited for): -> (toplevel, state dict {"answer", "sel", "yes", "no"})."""
    C = theme.C
    w = tk.Toplevel(parent, bg=C["head"])
    w.title(title)
    try:
        w.transient(parent)
    except tk.TclError:
        pass
    st = {"answer": False, "sel": bool(default)}
    tk.Label(w, text=title, font=theme.font("title", bold=True), bg=C["head"],
             fg=theme.CAUTION if warn else C["fg"]).pack(padx=20, pady=(12, 4))
    tk.Label(w, text=text, font=theme.font("value"), bg=C["head"], fg=C["fg"], wraplength=520,
             justify="left").pack(padx=20, pady=(4, 10))
    b = tk.Frame(w, bg=C["head"])
    b.pack(fill="x", padx=20, pady=(0, 16))

    def done(yes):
        st["answer"] = bool(yes)
        try:
            w.grab_release()
        except tk.TclError:
            pass
        w.destroy()

    st["yes"] = tk.Button(b, text=T("q.yes"), command=lambda: done(True), **theme.yn_opts(True, st["sel"], "title"))
    st["yes"].pack(fill="x", pady=(0, 6))
    st["no"] = tk.Button(b, text=T("q.no"), command=lambda: done(False), **theme.yn_opts(False, not st["sel"], "title"))
    st["no"].pack(fill="x")

    def select(yes):
        st["sel"] = bool(yes)
        st["yes"].configure(**{k: v for k, v in theme.yn_opts(True, yes, "title").items() if k != "font"})
        st["no"].configure(**{k: v for k, v in theme.yn_opts(False, not yes, "title").items() if k != "font"})
        try:
            (st["yes"] if yes else st["no"]).focus_set()   # WA review L3: the focus follows the SHOWN selection
        except tk.TclError:
            pass
    st["select"] = select
    st["done"] = done

    def key(fn):
        def h(_e):
            fn()
            return "break"                               # (never also the button's own class binding)
        return h
    # WA review L3: Enter / Space answer the SHOWN selection (also on the buttons: Tk's own Button <space> would
    # invoke the focused one), Up / Down select, Esc = NO
    for wid in (w, st["yes"], st["no"]):
        wid.bind("<Up>", key(lambda: select(True)))
        wid.bind("<Down>", key(lambda: select(False)))
        wid.bind("<Return>", key(lambda: done(st["sel"])))
        wid.bind("<space>", key(lambda: done(st["sel"])))
        wid.bind("<Escape>", key(lambda: done(False)))
    w.protocol("WM_DELETE_WINDOW", lambda: done(False))
    try:
        theme.style_tree(w)
    except Exception:
        pass
    return w, st


def ask(parent, title, text, warn=False, default=False):
    """The vertical YES / NO question, modal. -> True only for YES."""
    LAST.clear()
    LAST.update(title=title, text=text, warn=warn, default=default)
    if AUTO is not None:
        LAST["answer"] = bool(AUTO)
        return bool(AUTO)
    w, st = build(parent, title, text, warn, default)
    try:
        w.wait_visibility()                              # WA review L3: a grab needs the window shown
        w.grab_set()
    except tk.TclError:
        pass
    (st["yes"] if st["sel"] else st["no"]).focus_set()
    parent.wait_window(w)
    LAST["answer"] = st["answer"]
    return st["answer"]
