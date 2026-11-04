"""The UAV magazine lesson window (step U6): "UAV launch and observation".

    python academy/uavlesson_app.py [--lang el|en]      (or the academy window's "UAV lesson" button)

The trainee works on the academy's OWN in-process UAV simulator (uavsim.py: a simulated boat + magazine + hatch +
6 camera drones, driven by the gateway's launch / allow-list / route / battery-rule modules) - no network, no process,
never a gateway, a drone or a boat. The buttons are the station's (REGISTER, SEND ROUTE TO NEXT UAV, PREPARE LAUNCH,
the captain's YES / NO in the vertical layout, STOP, HATCH CLOSE, the UAV video, acknowledge) + the boat's mode.
Every academy screen shows the banner (banner_text, AC8). Camera / observation drones only.
"""
import argparse
import os
import sys
import time
import tkinter as tk
import tkinter.font as tkfont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import lesson                                                           # noqa: E402  (groundstation/ on the path)
import look                                                             # noqa: E402
import text                                                             # noqa: E402
import uavlesson                                                        # noqa: E402
from banner import banner_text                                          # noqa: E402,F401  (AC8: the banner below)
from text import T                                                      # noqa: E402

try:
    import theme                                                        # noqa: E402
    CAUTION, GOOD, COL, FAMILY = theme.CAUTION, theme.GOOD, theme.C, theme.FAMILY
except Exception:                                                       # pragma: no cover - academy alone
    theme = None
    CAUTION, GOOD, FAMILY = "#ffb000", "#3cff6a", "Segoe UI"
    COL = {"bg": "#0e141b", "fg": "#e6edf3", "panel": "#101a24", "dim": "#8fa3b5"}

TICK_MS = 250                    # 4 Hz = the gateway's launch tick
BUTTONS = (("register", "uavl.b_register"), ("route_next", "uavl.b_route"), ("standby", "uavl.b_standby"),
           ("hdg_lock", "uavl.b_hdg"), ("prepare", "uavl.b_prepare"), ("stop", "uavl.b_stop"),
           ("hatch_close", "uavl.b_hatch_close"), ("video", "uavl.b_video"), ("map", "uavl.b_map"),
           ("ack", "uavl.b_ack"))


class UavLessonWindow(object):
    """The UAV lesson on a Tk root / Toplevel. store + user = where the result goes (None = practice)."""

    def __init__(self, root, lang=None, store=None, user=None, clock=time.time, auto_tick=True, docs=None):
        self.root, self.store, self.user, self.clock = root, store, user, clock
        self.lang = lang if lang in uavlesson.LANGS else text.lang()
        self.docs = uavlesson.lessons() if docs is None else docs
        self.sim = self.trainee = self.run = None
        self.status = None
        self.logged = False
        self.alive = True
        L = self.lang
        look.install(root)
        look.title(root, "uavl.title", L)
        root.configure(bg=COL["bg"])
        try:                                # most of the screen (the station's fonts are large on a high-DPI PC)
            root.geometry("%dx%d" % (max(1240, int(root.winfo_screenwidth() * 0.85)),
                                     max(720, int(root.winfo_screenheight() * 0.85))))
        except tk.TclError:
            pass
        self.banner = look.banner(root, L)                 # the banner: top, always visible (CLAUDE.md S, AC8)
        self.banner.pack(side="top", fill="x")
        tk.Label(root, text=T("uavl.sim_only", L), bg=COL["bg"], fg=COL["dim"], anchor="w").pack(
            side="top", fill="x", padx=10)
        body = tk.Frame(root, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=10, pady=6)
        left = tk.Frame(body, bg=COL["bg"])
        left.pack(side="left", fill="both", expand=True)
        self.l_title = tk.Label(left, text="", bg=COL["bg"], fg=COL["fg"], anchor="w", font=(FAMILY, 16, "bold"),
                                justify="left", wraplength=900)
        self.l_title.pack(side="top", fill="x")
        self.l_intro = tk.Label(left, text="", bg=COL["bg"], fg=COL["dim"], anchor="nw", justify="left",
                                wraplength=900)
        self.l_intro.pack(side="top", fill="x", pady=(2, 6))
        self.l_step = tk.Label(left, text="", bg=COL["panel"], fg=COL["fg"], anchor="nw", justify="left",
                               wraplength=900, font=(FAMILY, 14, "bold"), padx=8, pady=8)
        self.l_step.pack(side="top", fill="x")
        self.l_hint = tk.Label(left, text="", bg=COL["bg"], fg=CAUTION, anchor="nw", justify="left", wraplength=900)
        self.l_hint.pack(side="top", fill="x")
        self.l_result = tk.Label(left, text="", bg=COL["bg"], fg=GOOD, anchor="nw", justify="left", wraplength=900,
                                 font=(FAMILY, 13, "bold"))
        self.l_result.pack(side="top", fill="x", pady=4)
        self.cv = tk.Canvas(left, width=560, height=260, bg=COL["panel"], highlightthickness=0)
        self.cv.pack(side="top", fill="x", pady=4)
        self.l_msg = tk.Label(left, text="", bg=COL["bg"], fg=COL["fg"], anchor="nw", justify="left", wraplength=900)
        self.l_msg.pack(side="top", fill="x")
        right = tk.Frame(body, bg=COL["bg"])
        right.pack(side="left", fill="y", padx=(10, 0))
        self.buttons = {}
        b = tk.Button(right, text=T("uavl.start", L), command=self.start)
        b.pack(side="top", fill="x", pady=(0, 8))
        self.buttons["start"] = b
        for key, lk in BUTTONS:
            b = tk.Button(right, text=T(lk, L), command=lambda k=key: self.act(k), wraplength=300, justify="center")
            b.pack(side="top", fill="x", pady=1)
            self.buttons[key] = b
        # the captain's question: ONE vertical layout (13p): the question, [YES] on top, [NO] underneath
        self.q_frame = tk.Frame(right, bg=COL["bg"])
        self.q_frame.pack(side="top", fill="x", pady=(10, 0))
        self.l_q = tk.Label(self.q_frame, text="", bg=COL["bg"], fg=CAUTION, wraplength=300, justify="left")
        self.l_q.pack(side="top", fill="x")
        for key, lk in (("yes", "uavl.b_yes"), ("no", "uavl.b_no")):
            b = tk.Button(self.q_frame, text=T(lk, L), command=lambda k=key: self.act(k), state="disabled")
            b.pack(side="top", fill="x", pady=1)
            self.buttons[key] = b
        self.notes = tk.Listbox(right, width=36, height=9, bg=COL["panel"], fg=COL["fg"], highlightthickness=0)
        self.notes.pack(side="top", fill="both", expand=True, pady=6)
        self.show_doc()
        if auto_tick:
            self.root.after(TICK_MS, self.loop)

    def doc(self):
        return self.docs[0] if self.docs else None

    def show_doc(self):
        d = self.doc()
        if d:
            self.l_title.config(text=d.get("title_" + self.lang, ""))
            self.l_intro.config(text=d.get("intro_" + self.lang, ""))

    def start(self):
        d = self.doc()
        if d is None:
            return
        self.sim = uavlesson.make_sim(self.clock)
        self.trainee = uavlesson.Trainee(self.sim)
        self.run = uavlesson.new_run(d, self.lang)
        self.logged = False
        for a in self.run.start(self.clock()):
            self.sim.do(a)
        self.l_result.config(text="")

    def act(self, key):
        if self.trainee is None:
            return self.l_msg.config(text=T("uavl.press_start", self.lang))
        ok, code, txt = self.trainee.act(key)
        self.l_msg.config(text=(txt if ok else T("uavl.refused", self.lang, code=code, why=txt)))
        return ok

    def loop(self):
        if not self.alive:
            return
        try:
            self.tick()
        finally:
            try:
                self.root.after(TICK_MS, self.loop)
            except tk.TclError:
                self.alive = False

    def tick(self):
        if self.sim is None:
            return
        self.sim.tick()
        st = self.sim.state()
        self.status, acts = self.run.update(st, self.clock())
        for a in acts:
            self.sim.do(a)
        L = self.lang
        if self.status == "step":
            self.l_step.config(text=T("uavl.step_of", L, n=self.run.index + 1, of=len(self.run.steps)) + "\n" +
                               self.run.text("do"))
            self.l_hint.config(text=(T("uavl.hint", L) + " " + self.run.text("hint")) if self.run.hint_shown else "")
        elif self.status == "done":
            self.l_result.config(text=T("uavl.passed", L, score=self.run.score()))
            self.log()
        elif self.status == "failed":
            self.l_result.config(text=T("uavl.failed", L))
        asking = st["launch"] == "ASK"
        self.l_q.config(text=T("uavl.question", L) if asking else "")
        for k in ("yes", "no"):
            self.buttons[k].config(state="normal" if asking else "disabled")
        self.notes.delete(0, "end")
        for t, x, lv, acked in self.sim.notices[-9:]:
            self.notes.insert("end", ("   " if acked else "! ") + x)
        self.draw(st)

    def log(self):
        if self.logged or self.store is None or not self.user:
            return
        self.logged = True
        try:
            self.store.log(self.user, "lesson", **self.run.log_entry())
        except Exception as e:                                  # (a store error never stops the window)
            self.l_msg.config(text=str(e))

    def draw(self, st):
        """The magazine (6 slots stacked, slot 1 on top), the hatch, the drones in the air with their altitude."""
        c = self.cv
        c.delete("all")
        L = self.lang
        fnt = (FAMILY, 11)
        lh = tkfont.Font(root=self.root, font=fnt).metrics("linespace") + 2     # one text line (DPI-proof)
        c.config(height=lh * 13)
        for i, line in enumerate((T("uavl.boat", L, mode=st["boat_mode"], kn="%.1f" % st["sog_kn"]),
                                  T("uavl.hatch", L, state=st["hatch"]),
                                  T("uavl.seq", L, state=st["launch"], phase=st["phase"] or "-"))):
            c.create_text(lh // 2, lh // 2 + i * lh, anchor="nw", fill=COL["fg"], text=line, font=fnt)
        x0, y0, bw = lh, lh * 4, lh * 7
        c.create_rectangle(x0 - 4, y0 - 6, x0 + bw + 4, y0 + 6 * lh + 4, outline=COL["dim"])
        c.create_line(x0 - 4, y0 - 6, x0 + bw + 4, y0 - 6, fill=GOOD if st["hatch"] == "OPEN" else COL["fg"], width=4)
        for s in range(1, 7):
            d = self.sim.drones[s]
            y = y0 + (s - 1) * lh
            label = T("uavl.slot", L, n=s) + (" R" if d.registered else "") + ("" if d.usb_present else " -")
            c.create_rectangle(x0, y, x0 + bw, y + lh - 2, outline=COL["fg"],
                               fill=COL["panel"] if d.usb_present else COL["bg"])
            c.create_text(x0 + 4, y + lh // 2, anchor="w", fill=COL["fg"], text=label, font=fnt)
        for s, d in self.sim.drones.items():
            if not d.usb_present:
                xa = x0 + bw + lh * 2 + lh * 5 * (s - 1)
                ya = lh * 12 - min(lh * 8.0, d.alt * lh / 8.0)
                r = lh // 3
                c.create_oval(xa - r, ya - r, xa + r, ya + r, outline=CAUTION if d.landing else GOOD, width=2)
                c.create_text(xa, ya - lh, fill=COL["fg"], font=fnt,
                              text="UAV %d %.0f m %d %%" % (s, d.alt, d.battery_pct))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lang", choices=uavlesson.LANGS)
    a = ap.parse_args()
    if a.lang:
        text.set_lang(a.lang)
    root = tk.Tk()
    UavLessonWindow(root, text.lang())
    root.mainloop()


if __name__ == "__main__":
    main()
