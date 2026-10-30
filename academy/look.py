"""The academy's look = the ground station's (groundstation/theme.py, step 13i, DISPLAY ONLY).

install(win)        on every academy window: the app icon, the station's dark theme for the ttk widgets
                    (tables, drop-downs) and, through the Tk option database, for every tk widget made
                    after it without its own colours / font (buttons = theme.button_opts: no raw grey Tk
                    buttons), then style_tree() for the ones already made
banner(parent, L)   the orange banner «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» (banner_text, CLAUDE.md S / AC8)
title(win, key, L)  "Trygons C2 – Ακαδημία – <window>" (the same titles everywhere)
fit_columns(tree)   table columns as wide as their heading and their longest value (readable headers)
Never opens anything (no network / process import: test_academy AC1-AC3).
"""
import os
import sys
import tkinter as tk
import tkinter.font as tkfont

HERE = os.path.dirname(os.path.abspath(__file__))
GS = os.path.join(os.path.dirname(HERE), "groundstation")
if GS not in sys.path:
    sys.path.append(GS)

import theme                                                            # noqa: E402
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

C = theme.C


def install(win):
    """The station's look on this window and on every widget made after it (the option database)."""
    root = win.winfo_toplevel()
    theme.set_app_icon(root)
    try:
        win.configure(bg=C["bg"])
    except tk.TclError:
        pass
    theme.apply_ttk(root)
    b = theme.button_opts("normal")
    opts = {"*Button.background": b["bg"], "*Button.foreground": b["fg"],
            "*Button.activeBackground": b["activebackground"], "*Button.activeForeground": b["activeforeground"],
            "*Button.relief": "flat", "*Button.borderWidth": 0, "*Button.highlightThickness": 1,
            "*Button.highlightBackground": C["line"], "*Button.font": theme.font("button"), "*Button.padX": 10,
            "*Button.padY": 4, "*Button.cursor": "hand2", "*Button.disabledForeground": C["faint"],
            "*Label.background": C["bg"], "*Label.foreground": C["fg"], "*Label.font": theme.font("label"),
            "*Frame.background": C["bg"], "*Toplevel.background": C["bg"],
            "*Checkbutton.background": C["bg"], "*Checkbutton.foreground": C["fg"],
            "*Checkbutton.selectColor": C["entry"], "*Checkbutton.activeBackground": C["bg"],
            "*Radiobutton.background": C["bg"], "*Radiobutton.foreground": C["fg"],
            "*Radiobutton.selectColor": C["entry"], "*Radiobutton.activeBackground": C["bg"],
            "*Entry.background": C["entry"], "*Entry.foreground": C["entry_fg"],
            "*Entry.insertBackground": C["entry_fg"], "*Entry.font": theme.font("label"),
            "*Listbox.background": C["entry"], "*Listbox.foreground": C["entry_fg"],
            "*Listbox.font": theme.font("label"), "*Text.background": C["entry"], "*Text.foreground": C["entry_fg"],
            "*Text.insertBackground": C["entry_fg"], "*Scale.background": C["bg"], "*Scale.foreground": C["fg"],
            "*Scale.troughColor": C["entry"], "*Spinbox.background": C["entry"], "*Spinbox.foreground": C["entry_fg"],
            "*Menu.background": C["panel"], "*Menu.foreground": C["fg"]}
    for k, v in opts.items():
        root.option_add(k, v)
    restyle(win)


def restyle(win):
    """Widgets already made with the system look get the theme's (buttons: flat, themed, the button font)."""
    theme.style_tree(win)
    b = theme.button_opts("normal")
    stack = [win]
    while stack:
        w = stack.pop()
        try:
            if w.winfo_class() == "Button":
                o = {"relief": "flat", "bd": 0, "highlightthickness": 1, "highlightbackground": C["line"],
                     "cursor": "hand2", "activeforeground": b["activeforeground"]}
                if str(w.cget("bg")).lower() == C["btn"].lower():
                    o["activebackground"] = b["activebackground"]
                f = tkfont.Font(root=w, font=w.cget("font")).actual()
                if f.get("family") not in (theme.FAMILY, theme.MONO):
                    o["font"] = theme.font("button")
                w.configure(**o)
            stack.extend(w.winfo_children())
        except tk.TclError:
            continue


def banner(parent, lang_code=None):
    """The academy banner (CLAUDE.md S, AC8): top, always visible, the theme's CAUTION colour."""
    return tk.Label(parent, text=banner_text(lang_code), bg=theme.CAUTION, fg=theme.BLACK, font=theme.font("title"),
                    pady=6)


def title(win, key, lang_code=None, main=False):
    """ "Trygons C2 – Ακαδημία – <key's text>" (main=True: "Trygons C2 – Ακαδημία")."""
    name = T("academy.title", lang_code) if main else T("academy.title", lang_code) + " – " + T(key, lang_code)
    win.title(T("win.title", lang_code, name=name))


def fit_columns(tree, columns=None, pad=24, max_w=360):
    """Every column at least as wide as its heading (+ pad) and its longest value (up to max_w)."""
    head = tkfont.Font(root=tree, font=theme.font("button"))
    body = tkfont.Font(root=tree, font=theme.font("label"))
    cols = list(columns or tree["columns"])
    widths = {}
    for i, c in enumerate(cols):
        w = head.measure(tree.heading(c, "text")) + pad
        for iid in tree.get_children():
            vals = tree.item(iid, "values")
            if i < len(vals):
                w = max(w, min(max_w, body.measure(str(vals[i])) + pad))
        tree.column(c, width=w, minwidth=head.measure(tree.heading(c, "text")) + pad, stretch=False)
        widths[c] = w
    return widths
