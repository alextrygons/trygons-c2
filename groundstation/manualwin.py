"""The manual in a read-only window (step AM5, [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ]): MANUAL_EL.txt / MANUAL.txt opened at the lines of
the answer's passage (assistant/show.manual_place), the passage highlighted. DISPLAY ONLY (theme.py look); one window,
reused."""
import tkinter as tk

import i18n
import theme
from assistant import show

_win = None


def open_at(root, lang, a, b):
    """Show the manual of `lang` with lines a..b (1-based, inclusive) highlighted and in view. -> the Toplevel."""
    global _win
    lines = show.manual_lines(lang)
    C = theme.C
    if _win is None or not _win.winfo_exists():
        _win = tk.Toplevel(root)
        _win.configure(bg=C["bg"])
        _win.geometry("980x720")
        fr = tk.Frame(_win, bg=C["bg"])
        fr.pack(fill="both", expand=True, padx=6, pady=6)
        sb = tk.Scrollbar(fr)
        sb.pack(side="right", fill="y")
        txt = tk.Text(fr, wrap="word", font=theme.font("label", mono=True), bg=C["entry"], fg=C["entry_fg"],
                      insertbackground=C["fg"], relief="solid", bd=theme.FRAME, yscrollcommand=sb.set, padx=8, pady=6)
        txt.pack(side="left", fill="both", expand=True)
        sb.configure(command=txt.yview)
        _win.txt = txt
        _win.lang = None
        theme.style_tree(_win)
    w = _win
    w.title(i18n.W("asst.manual_title", lang=show.MANUALS.get(lang, "MANUAL.txt")))
    t = w.txt
    t.configure(state="normal")
    if w.lang != lang:
        t.delete("1.0", "end")
        t.insert("1.0", "\n".join(lines))
        w.lang = lang
    t.tag_remove("hit", "1.0", "end")
    a = max(1, int(a))
    b = max(a, int(b))
    t.tag_add("hit", "%d.0" % a, "%d.0 lineend" % b)
    t.tag_configure("hit", background=C["head"], foreground=C["fg"])
    t.configure(state="disabled")
    t.see("%d.0" % b)
    t.see("%d.0" % a)
    w.deiconify()
    w.lift()
    w.place = (lang, a, b)
    return w


def current():
    return _win if _win is not None and _win.winfo_exists() else None
