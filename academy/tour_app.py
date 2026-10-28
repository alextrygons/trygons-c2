"""The guided tour window (ACADEMY_SPEC B, step 17): screen 1, screen 2, tools, modes, alarms, lamps,
joystick and panel buttons - one stop at a time on a DRAWING of the screen, next / back, Greek voice.

    python academy/tour_app.py [--lang el|en]       (or the academy window's "Guided tour" button,
                                                     or the ground station menu Academy -> Guided tour)

It only explains: no connection, no process (test_academy AC1-AC3). The voice = the .wav files the
ground station's offline TTS made for every stop (groundstation/academymenu.py prepare_tour_voice);
this window only PLAYS them, each file checked by simguard.check_recording first. No voice file
(yet) or no sound (TRYGONS_SOUND=off, not Windows) = text only. Every academy screen shows the
banner at the top (AC8).
"""
import os
import sys
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(os.path.dirname(HERE), "groundstation"))

import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
import tour                                                             # noqa: E402
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

import hud                                                              # noqa: E402
import i18n                                                             # noqa: E402

try:
    import theme                                                        # noqa: E402
    CAUTION, WARN, GOOD, ADVISORY = theme.CAUTION, theme.WARN, theme.GOOD, theme.ADVISORY
    MODE_COLOURS = theme.MODE_COLOURS
    COL = theme.C
    FAMILY = theme.FAMILY
except Exception:                                                       # pragma: no cover - academy alone
    theme = None
    CAUTION, WARN, GOOD, ADVISORY = "#ffb000", "#ff3030", "#3cff6a", "#40c0ff"
    MODE_COLOURS = {}
    COL = {"bg": "#0e141b", "fg": "#e6edf3", "panel": "#101a24", "dim": "#8fa3b5"}
    FAMILY = "Segoe UI"

try:
    import winsound                                                     # Windows only: plays a .wav file
except ImportError:                                                     # pragma: no cover
    winsound = None

LEVEL_COLOUR = {"WARNING": WARN, "CAUTION": CAUTION, "ADVISORY": ADVISORY}
HILITE_FILL = "#5a3f00"                                                 # the lit part's fill (dark amber)
BOX = "#5c7080"                                                         # every other part's outline
# the drawings' layout lives in tour.py (shared with the quiz picture questions, step 18)
S2_BOXES = tour.S2_BOXES
PANEL_BTNS = tour.PANEL_BTNS


class NullPlayer(object):
    """No sound here: remembers what would have played (the tests, TRYGONS_SOUND=off, not Windows)."""

    def __init__(self):
        self.played = []

    def play(self, path):
        self.played.append(path)

    def stop(self):
        self.played.append(None)


class WinPlayer(object):
    def play(self, path):
        try:
            winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
        except RuntimeError:
            pass

    def stop(self):
        try:
            winsound.PlaySound(None, 0)
        except RuntimeError:
            pass


def default_player():
    if winsound is None or os.environ.get("TRYGONS_SOUND") == "off":
        return NullPlayer()
    return WinPlayer()


class Drawing(object):
    """The DRAWINGS of the screens on a canvas (self.cv, self.lang): draw_stop(s) with s = a tour stop
    (show, target, title, level, level_text). Used by the tour and by the quiz picture questions
    (target None = nothing lit). self.frame = the 16:9 picture (x0, y0, w, h) on the canvas."""

    frame = (0, 0, 320, 180)

    def draw_stop(self, s):
        cv = self.cv
        cv.delete("all")
        W = max(cv.winfo_width(), 320)
        H = max(cv.winfo_height(), 180)
        if W / H > 16 / 9.0:                                # keep 16:9, centred
            w, h = H * 16 / 9.0, H
        else:
            w, h = W, W * 9 / 16.0
        self.frame = ((W - w) / 2, (H - h) / 2, w, h)
        getattr(self, "draw_" + s["show"])(s)

    def to_frac(self, x, y):
        """Canvas point -> fractions of the picture."""
        x0, y0, w, h = self.frame
        return (x - x0) / float(w), (y - y0) / float(h)

    def R(self, fx, fy, fw, fh):
        x0, y0, w, h = self.frame
        return x0 + fx * w, y0 + fy * h, x0 + (fx + fw) * w, y0 + (fy + fh) * h

    def box(self, r, label, lit, fill="#101a24", fg=None, size=10):
        self.cv.create_rectangle(*r, outline=CAUTION if lit else BOX, width=4 if lit else 1,
                                 fill=HILITE_FILL if lit else fill)
        self.cv.create_text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, text=label, width=max(20, r[2] - r[0] - 6),
                            fill=fg or (CAUTION if lit else COL["fg"]), font=(FAMILY, size, "bold" if lit else "normal"))

    def screen_bg(self, top, bottom):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h * 0.45, fill=top, outline="")
        self.cv.create_rectangle(x0, y0 + h * 0.45, x0 + w, y0 + h, fill=bottom, outline="")

    def draw_none(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        self.cv.create_text(x0 + w / 2, y0 + h * 0.42, text="Trygons C2", fill=COL["fg"], font=(FAMILY, 36, "bold"))
        self.cv.create_text(x0 + w / 2, y0 + h * 0.58, text=T("tour.title", self.lang), fill=CAUTION,
                            font=(FAMILY, 18))

    def draw_s1(self, s):
        self.screen_bg("#4a6a88", "#1d3a52")                 # the front camera: sky + sea
        x0, y0, w, h = self.frame
        self.cv.create_text(x0 + w / 2, y0 + h * 0.48, text=T("tour.camera", self.lang), fill="#c8d8e8",
                            font=(FAMILY, 14))
        for name in hud.OVERLAYS:
            r = self.R(*hud.DEFAULT_LAYOUT[name])
            self.box(r, i18n.TL(self.lang, "hud.ov_" + name), s["target"] == name, size=8)
        self.box(self.R(*hud.LOGO_BOX), T("tour.logo", self.lang), s["target"] == "logo", size=8)
        if s["target"] == "center":                          # an ACCEPT / REJECT question
            r = self.R(0.36, 0.36, 0.28, 0.20)
            self.cv.create_rectangle(*r, outline=CAUTION, width=4, fill=HILITE_FILL)
            self.cv.create_text((r[0] + r[2]) / 2, r[1] + (r[3] - r[1]) * 0.3, text=T("tour.question", self.lang),
                                fill=COL["fg"], font=(FAMILY, 11, "bold"))
            bw = (r[2] - r[0]) / 2 - 12
            for i, (lab, col) in enumerate(((i18n.TL(self.lang, "drv.accept"), GOOD),
                                            (i18n.TL(self.lang, "drv.reject"), WARN))):
                bx = r[0] + 8 + i * (bw + 8)
                self.cv.create_rectangle(bx, r[3] - 36, bx + bw, r[3] - 8, fill=col, outline="")
                self.cv.create_text(bx + bw / 2, r[3] - 22, text=lab, fill="#000000", font=(FAMILY, 10, "bold"))

    def draw_s2(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill="#16324a", outline="")
        for k in range(1, 4):                                # range rings around the boat
            rr = h * 0.09 * k
            self.cv.create_oval(x0 + w / 2 - rr, y0 + h / 2 - rr, x0 + w / 2 + rr, y0 + h / 2 + rr,
                                outline="#2c5676")
        lit = s["target"] == "boat"
        cx, cy, b = x0 + w / 2, y0 + h / 2, h * 0.04
        self.cv.create_polygon(cx, cy - b * 1.4, cx + b, cy + b, cx - b, cy + b,
                               fill=CAUTION if lit else GOOD, outline="#000000")
        if lit:
            rr = min(w, h) * 0.17
            self.cv.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline=CAUTION, width=3, dash=(6, 4))
        for name, frac in S2_BOXES.items():
            label = T("tour.s2_%s" % name, self.lang)
            fill = "#7a1010" if name == "standby" else "#101a24"
            self.box(self.R(*frac), label, s["target"] == name, fill=fill, size=9)

    def draw_tools(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        pages = tour.tools_pages()
        lh = h / (len(pages) + 1)
        self.cv.create_text(x0 + 12, y0 + lh / 2, text=i18n.TL(self.lang, "tools.title"), anchor="w",
                            fill=COL["fg"], font=(FAMILY, 12, "bold"))
        for i, key in enumerate(pages):
            r = (x0 + 8, y0 + lh * (i + 1), x0 + w * 0.28, y0 + lh * (i + 2) - 3)
            self.box(r, i18n.TL(self.lang, key), s["target"] == key, size=10)
        self.cv.create_rectangle(x0 + w * 0.30, y0 + lh, x0 + w - 8, y0 + h - 8, outline=BOX)
        if s["target"]:
            self.cv.create_text(x0 + w * 0.65, y0 + h / 2, text=i18n.TL(self.lang, s["target"]), fill=CAUTION,
                                font=(FAMILY, 22, "bold"))

    def draw_modes(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        modes = list(tour.proto.MODES)
        cols = 3
        bw, bh = w / cols, h / ((len(modes) + cols - 1) // cols)
        for i, m in enumerate(modes):
            r = (x0 + (i % cols) * bw + 8, y0 + (i // cols) * bh + 8, x0 + (i % cols + 1) * bw - 8,
                 y0 + (i // cols + 1) * bh - 8)
            lit = s["target"] == m
            self.cv.create_rectangle(*r, fill=MODE_COLOURS.get(m, "#404040"), outline=CAUTION if lit else "",
                                     width=6 if lit else 0)
            self.cv.create_text((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, text=i18n.TL(self.lang, "mode." + m),
                                fill="#ffffff", font=(FAMILY, 16 if lit else 12, "bold"))

    def lamps(self, warning, caution, r):
        x0, y0, x1, y1 = r
        hh = (y1 - y0 - 8) / 2
        for i, (key, on, col) in enumerate((("hud.master_warning", warning, WARN),
                                            ("hud.master_caution", caution, CAUTION))):
            rr = (x0, y0 + i * (hh + 8), x1, y0 + i * (hh + 8) + hh)
            self.cv.create_rectangle(*rr, fill=col if on else "#202020", outline=col, width=2)
            self.cv.create_text((rr[0] + rr[2]) / 2, (rr[1] + rr[3]) / 2, text=i18n.TL(self.lang, key),
                                fill="#000000" if on else col, font=(FAMILY, 12, "bold"), justify="center")

    def draw_lamps(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        self.lamps(s["target"] == "master_warning", s["target"] == "master_caution",
                   self.R(0.35, 0.15, 0.30, 0.70))

    def draw_alarm(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        lv = s["level"]
        self.lamps(lv == "WARNING", lv == "CAUTION", self.R(0.05, 0.10, 0.20, 0.45))
        r = self.R(0.32, 0.10, 0.63, 0.45)                   # the alarm box with this alarm in it
        self.cv.create_rectangle(*r, fill="#101a24", outline=BOX)
        col = LEVEL_COLOUR.get(lv, COL["fg"])
        row = (r[0] + 8, r[1] + 10, r[2] - 8, r[1] + 10 + (r[3] - r[1]) * 0.35)
        self.cv.create_rectangle(*row, fill=col if lv != "ADVISORY" else "#101a24", outline=col, width=3)
        self.cv.create_text(row[0] + 10, (row[1] + row[3]) / 2, text=s["title"], anchor="w", width=row[2] - row[0] - 16,
                            fill="#000000" if lv != "ADVISORY" else col, font=(FAMILY, 13, "bold"))
        self.cv.create_text(x0 + w / 2, y0 + h * 0.72, text=s["level_text"] or "", fill=col, font=(FAMILY, 20, "bold"))

    def draw_joystick(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        t = s["target"]
        cx, cy, rr = x0 + w * 0.32, y0 + h * 0.55, h * 0.28
        self.cv.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, fill="#202a33", outline=BOX, width=2)
        self.cv.create_oval(cx - rr * 0.3, cy - rr * 0.3, cx + rr * 0.3, cy + rr * 0.3, fill="#3a4652", outline=BOX)
        a = rr * 0.95
        self.cv.create_line(cx - a, cy, cx + a, cy, arrow="both", width=6 if t == "x" else 2,
                            fill=CAUTION if t == "x" else BOX)
        self.cv.create_line(cx, cy - a, cx, cy + a, arrow="both", width=6 if t == "y" else 2,
                            fill=CAUTION if t == "y" else BOX)
        self.cv.create_text(cx + a + 8, cy, text="X", anchor="w", fill=COL["fg"], font=(FAMILY, 12, "bold"))
        self.cv.create_text(cx, cy - a - 6, text="Y", anchor="s", fill=COL["fg"], font=(FAMILY, 12, "bold"))
        for i, (lab, key) in enumerate((("TAKE", "take"), ("AUTO", "auto"), ("CONFIRM", "confirm"))):
            self.box(self.R(0.62, 0.18 + i * 0.16, 0.16, 0.12), lab, t == key, size=11)
        # the USB lever (T.16000M + TWCS): non-returning, neutral zone 45-55 %
        lit = t == "usb"
        r = self.R(0.84, 0.15, 0.08, 0.75)
        self.cv.create_rectangle(*r, outline=CAUTION if lit else BOX, width=4 if lit else 1,
                                 fill=HILITE_FILL if lit else "#101a24")
        nz0 = r[1] + (r[3] - r[1]) * 0.45
        nz1 = r[1] + (r[3] - r[1]) * 0.55
        self.cv.create_rectangle(r[0] + 2, nz0, r[2] - 2, nz1, fill="#303030", outline="")
        self.cv.create_text((r[0] + r[2]) / 2, r[3] + 10, text="USB", fill=COL["fg"], font=(FAMILY, 10, "bold"),
                            anchor="n")

    def draw_panel(self, s):
        x0, y0, w, h = self.frame
        self.cv.create_rectangle(x0, y0, x0 + w, y0 + h, fill=COL["panel"], outline="")
        t = s["target"]
        for lab, key in PANEL_BTNS:
            fill = "#7a1010" if key == "standby" else "#202a33"
            self.box(self.R(*tour.panel_rect(key)), lab, t == key, fill=fill, size=12)
        lit = t == "estop"                                   # the red mushroom button
        cx, cy, rr = x0 + w * tour.ESTOP_C[0], y0 + h * tour.ESTOP_C[1], h * tour.ESTOP_R
        self.cv.create_oval(cx - rr * 1.3, cy - rr * 1.3, cx + rr * 1.3, cy + rr * 1.3, fill="#ffd000", outline="")
        self.cv.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, fill=WARN, outline=CAUTION if lit else "#000000",
                            width=6 if lit else 2)
        self.cv.create_text(cx, cy + rr * 1.3 + 12, text="E-STOP", fill=COL["fg"], font=(FAMILY, 11, "bold"),
                            anchor="n")


class TourWindow(Drawing):
    """The tour on a Tk root or Toplevel. index / show(i) / next() / back() / jump(chapter)."""

    def __init__(self, root, lang=None, player=None, cache=None, doc=None):
        self.root = root
        self.lang = lang if lang in tour.LANGS else text.lang()
        self.player = player or default_player()
        self.cache = cache
        self.stops = tour.stops(self.lang, doc)
        self.index = 0
        self.voice_on = True
        self.voice_state = None             # "played" / "missing" / "off" (the tests read it)
        root.title(T("tour.title", self.lang))
        root.configure(bg=COL["bg"])
        try:
            root.geometry("1180x760")
        except tk.TclError:
            pass
        # the banner: top, always visible, CAUTION colour (CLAUDE.md S, AC8)
        self.banner = tk.Label(root, text=banner_text(self.lang), bg=CAUTION, fg="#000000",
                               font=(FAMILY, 16, "bold"), pady=6)
        self.banner.pack(side="top", fill="x")
        self.head = tk.Label(root, text="", bg=COL["bg"], fg=COL["dim"], anchor="w", font=(FAMILY, 11))
        self.head.pack(side="top", fill="x", padx=10)
        mid = tk.Frame(root, bg=COL["bg"])
        mid.pack(side="top", fill="both", expand=True, padx=10, pady=4)
        self.chapters = []
        for s in self.stops:
            if not self.chapters or self.chapters[-1][0] != s["chapter"]:
                self.chapters.append((s["chapter"], s["chapter_title"]))
        self.chlist = tk.Listbox(mid, width=26, bg=COL["panel"], fg=COL["fg"], selectbackground=CAUTION,
                                 selectforeground="#000000", activestyle="none", font=(FAMILY, 11),
                                 exportselection=False, highlightthickness=0)
        for _cid, title in self.chapters:
            self.chlist.insert("end", title)
        self.chlist.pack(side="left", fill="y")
        self.chlist.bind("<<ListboxSelect>>", self._on_chapter)
        right = tk.Frame(mid, bg=COL["bg"])
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))
        self.cv = tk.Canvas(right, bg="#000000", highlightthickness=0, width=800, height=450)
        self.cv.pack(side="top", fill="both", expand=True)
        self.cv.bind("<Configure>", lambda e: self.draw())
        self.title = tk.Label(right, text="", bg=COL["bg"], fg=COL["fg"], anchor="w", font=(FAMILY, 16, "bold"))
        self.title.pack(side="top", fill="x", pady=(6, 0))
        self.body = tk.Label(right, text="", bg=COL["bg"], fg=COL["fg"], anchor="nw", justify="left",
                             font=(FAMILY, 12), wraplength=820)
        self.body.pack(side="top", fill="x")
        bar = tk.Frame(root, bg=COL["bg"])
        bar.pack(side="top", fill="x", padx=10, pady=6)
        self.b_back = tk.Button(bar, text=T("tour.back", self.lang), width=12, command=self.back)
        self.b_back.pack(side="left")
        self.b_next = tk.Button(bar, text=T("tour.next", self.lang), width=12, command=self.next)
        self.b_next.pack(side="left", padx=6)
        self.b_again = tk.Button(bar, text=T("tour.again", self.lang), command=self.speak)
        self.b_again.pack(side="left", padx=6)
        self.b_voice = tk.Button(bar, text="", command=self.toggle_voice)
        self.b_voice.pack(side="left", padx=6)
        self.msg = tk.Label(bar, text="", bg=COL["bg"], fg=COL["dim"], anchor="w")
        self.msg.pack(side="left", padx=10, fill="x", expand=True)
        for key, fn in (("<Right>", self.next), ("<space>", self.next), ("<Left>", self.back)):
            root.bind(key, lambda e, f=fn: f())
        self._voice_label()
        self.show(0)

    # --- navigation
    def show(self, i):
        self.index = max(0, min(len(self.stops) - 1, i))
        s = self.stops[self.index]
        self.head.configure(text=T("tour.where", self.lang, chapter=s["chapter_title"], n=self.index + 1,
                                   of=len(self.stops)))
        title = s["title"]
        if s["level"]:
            title = "%s  (%s)" % (title, s["level_text"])
        self.title.configure(text=title, fg=LEVEL_COLOUR.get(s["level"], COL["fg"]))
        self.body.configure(text=s["text"])
        self.b_back.configure(state="normal" if self.index > 0 else "disabled")
        self.b_next.configure(text=T("tour.next" if self.index < len(self.stops) - 1 else "tour.done", self.lang))
        ci = [c for c, _ in self.chapters].index(s["chapter"])
        self.chlist.selection_clear(0, "end")
        self.chlist.selection_set(ci)
        self.draw()
        self.speak()

    def next(self):
        if self.index >= len(self.stops) - 1:
            self.player.stop()
            self.msg.configure(text=T("tour.finished", self.lang))
            return
        self.show(self.index + 1)

    def back(self):
        if self.index > 0:
            self.show(self.index - 1)

    def jump(self, chapter):
        for i, s in enumerate(self.stops):
            if s["chapter"] == chapter:
                return self.show(i)

    def _on_chapter(self, _e=None):
        sel = self.chlist.curselection()
        if sel and self.chapters[sel[0]][0] != self.stops[self.index]["chapter"]:
            self.jump(self.chapters[sel[0]][0])

    # --- voice (plays only the ground station's prepared files)
    def _voice_label(self):
        self.b_voice.configure(text=T("tour.voice_on" if self.voice_on else "tour.voice_off", self.lang))

    def toggle_voice(self):
        self.voice_on = not self.voice_on
        if not self.voice_on:
            self.player.stop()
        self._voice_label()
        self.speak()

    def speak(self):
        if not self.voice_on:
            self.voice_state = "off"
            self.msg.configure(text="")
            return
        path = tour.voice_path(self.lang, tour.sentence(self.stops[self.index]), self.cache)
        try:
            ok = simguard.check_recording(path)
        except simguard.Refused:
            self.player.stop()
            self.voice_state = "missing"
            self.msg.configure(text=T("tour.no_voice", self.lang))
            return
        self.player.play(ok)
        self.voice_state = "played"
        self.msg.configure(text="")

    # --- the drawing
    def draw(self):
        self.draw_stop(self.stops[self.index])


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--lang" in argv:
        text.set_lang(argv[argv.index("--lang") + 1])
    if simguard.running_on_gateway():
        print(T("refuse.on_gateway"))
        return 1
    if theme is not None:
        try:
            theme.dpi_aware()
        except Exception:
            pass
    root = tk.Tk()
    TourWindow(root, text.lang())
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
