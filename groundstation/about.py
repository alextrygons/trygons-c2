"""Help -> About (step 13i, DISPLAY ONLY): the logo, the version, the commit, licences / credits.

open_about(master, lang) opens it (one window: a second call brings the open one to the front). Used by
the ground station (app.py menu Help) and the academy (academy_app.py menu Help). Look from theme.py,
texts from lang/ in the caller's language (i18n.TL).
"""
import tkinter as tk

import i18n
import theme
import version

# (name, licence, what it does here = a lang key) - the licence names as their projects write them
CREDITS = (
    ("libcanard", "MIT", "about.c_libcanard"),
    ("Infineon iLLD", "Infineon", "about.c_illd"),
    ("eWaSR", "Apache-2.0", "about.c_ewasr"),
    ("ONNX Runtime", "MIT", "about.c_onnx"),
    ("MediaMTX", "MIT", "about.c_mediamtx"),
    ("FFmpeg", "LGPL / GPL", "about.c_ffmpeg"),
    ("Piper", "MIT", "about.c_piper"),
    ("Pillow", "MIT-CMU", "about.c_pillow"),
    ("Python, Tcl/Tk", "PSF, Tcl/Tk", "about.c_python"),
    ("EMODnet Bathymetry, GEBCO, OpenSeaMap / OpenStreetMap", "CC BY 4.0, GEBCO, ODbL", "about.c_charts"),
)


class About(object):
    def __init__(self, master, lang=None):
        self.lang = lang if lang in i18n.LANGS else i18n.lang()
        C = theme.C
        L = self.lang
        w = self.w = tk.Toplevel(master, bg=C["bg"])
        theme.set_app_icon(w)
        w.title(i18n.TL(L, "win.title", name=i18n.TL(L, "about.title")))
        w.resizable(False, False)
        self.logo = None
        p = theme.logo_path()
        if p:
            try:
                from PIL import ImageTk
                im = theme.logo_pil(64)
                self.logo = ImageTk.PhotoImage(im, master=w) if im is not None else None
            except Exception:
                try:
                    img = tk.PhotoImage(master=w, file=p)
                    k = max(1, img.height() // 64)
                    self.logo = img.subsample(k, k)
                except Exception:
                    self.logo = None
        if self.logo is not None:
            tk.Label(w, image=self.logo, bg=C["bg"]).pack(padx=24, pady=(18, 6))
        tk.Label(w, text=theme.APP_NAME, bg=C["bg"], fg=C["fg"], font=theme.font("huge")).pack(padx=24)
        tk.Label(w, text=i18n.TL(L, "about.what"), bg=C["bg"], fg=C["dim"], font=theme.font("label")).pack(padx=24)
        c = version.commit()
        self.version_text = i18n.TL(L, "about.version", v=version.VERSION)
        self.commit_text = i18n.TL(L, "about.commit", c=c or i18n.TL(L, "about.no_commit"))
        for t in (self.version_text, self.commit_text, i18n.TL(L, "about.owner")):
            tk.Label(w, text=t, bg=C["bg"], fg=C["fg"], font=theme.font("label")).pack(padx=24)
        tk.Label(w, text=i18n.TL(L, "about.credits"), bg=C["bg"], fg=C["fg"], font=theme.font("value")).pack(
            anchor="w", padx=24, pady=(14, 4))
        box = tk.Frame(w, bg=C["panel"], highlightthickness=1, highlightbackground=C["line"])
        box.pack(fill="x", padx=24)
        self.credit_lines = []
        for i, (name, lic, key) in enumerate(CREDITS):
            tk.Label(box, text=name, bg=C["panel"], fg=C["fg"], font=theme.font("label", bold=True),
                     anchor="w").grid(row=i, column=0, sticky="w", padx=(8, 12), pady=1)
            tk.Label(box, text=lic, bg=C["panel"], fg=C["accent"], font=theme.font("small"),
                     anchor="w").grid(row=i, column=1, sticky="w", padx=(0, 12))
            tk.Label(box, text=i18n.TL(L, key), bg=C["panel"], fg=C["dim"], font=theme.font("small"),
                     anchor="w").grid(row=i, column=2, sticky="w", padx=(0, 8))
            self.credit_lines.append((name, lic))
        tk.Label(w, text=i18n.TL(L, "about.chart_note"), bg=C["bg"], fg=theme.CAUTION, font=theme.font("small"),
                 wraplength=640, justify="left").pack(anchor="w", padx=24, pady=(8, 0))
        tk.Button(w, text=i18n.TL(L, "about.close"), command=self.close, **theme.button_opts("normal")).pack(
            pady=(12, 16))
        w.bind("<Escape>", lambda e: self.close())

    def alive(self):
        try:
            return bool(self.w.winfo_exists())
        except tk.TclError:
            return False

    def close(self):
        try:
            self.w.destroy()
        except tk.TclError:
            pass


def open_about(master, lang=None, current=None):
    """Opens About (or brings the open one forward). Returns the About."""
    if current is not None and current.alive() and current.lang == (lang or i18n.lang()):
        current.w.lift()
        return current
    if current is not None:
        current.close()
    return About(master, lang)
