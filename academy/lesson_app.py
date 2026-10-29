"""The simulator lessons window (ACADEMY_SPEC C, step 18).

    python academy/lesson_app.py [--lang el|en]      (or the academy window's "Lessons" button)

The trainee drives the SIMULATOR on this PC (sim/autopilot_sim server 5760, started with the
button "Start the simulator" = simguard.start_simulator, or already running) with on-screen
simulator inputs: the panel buttons, a self-centring joystick (drag the knob; TAKE / CONFIRM held
while pressed) and the ground station's own PC commands (power code / raise / lower, alarm
acknowledge). Every step is checked from the SIMULATOR's telemetry (lesson.py), with a hint after
a while and retry. The link is simguard.SimLink (port 5760 of this PC only); nothing else is
opened. Results go to the selected account's logbook. Every academy screen shows the banner (AC8).
"""
import os
import sys
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import lesson                                                           # noqa: E402  (puts groundstation/ on the path)
import sim3d                                                            # noqa: E402  (step 22: the 3D view feed)
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

import alerts                                                           # noqa: E402
import i18n                                                             # noqa: E402
import proto                                                            # noqa: E402

try:
    import theme                                                        # noqa: E402
    CAUTION, WARN, GOOD = theme.CAUTION, theme.WARN, theme.GOOD
    COL = theme.C
    FAMILY = theme.FAMILY
except Exception:                                                       # pragma: no cover - academy alone
    theme = None
    CAUTION, WARN, GOOD = "#ffb000", "#ff3030", "#3cff6a"
    COL = {"bg": "#0e141b", "fg": "#e6edf3", "panel": "#101a24", "dim": "#8fa3b5"}
    FAMILY = "Segoe UI"

TICK_MS = 50                                # 20 Hz: heartbeat + simulated joystick (like the station)
PAD = 150                                   # joystick pad size (px)
PANEL = (("standby", "STANDBY"), ("hdg_lock", "HDG LOCK"), ("minus10", "-10"), ("minus1", "-1"), ("plus1", "+1"),
         ("plus10", "+10"))


class LessonWindow(object):
    """Lessons on a Tk root / Toplevel. link_factory() -> a byte link (default simguard.SimLink);
    store + user = where the results go (None = practice, nothing logged)."""

    def __init__(self, root, lang=None, store=None, user=None, link_factory=None, starter=None, clock=time.time,
                 docs=None):
        self.root = root
        self.lang = lang if lang in lesson.LANGS else text.lang()
        self.store, self.user = store, user
        self.link_factory = link_factory or simguard.SimLink
        self.starter = starter or simguard.start_simulator
        self.clock = clock
        self.docs = lesson.lessons() if docs is None else docs
        self.sim = None
        self.run = None
        self.proc = None
        self.status = None
        self.logged = False
        self.alive = True
        L = self.lang
        root.title(T("lesson.title", L))
        root.configure(bg=COL["bg"])
        try:
            root.geometry("1240x720")
        except tk.TclError:
            pass
        # the banner: top, always visible, CAUTION colour (CLAUDE.md S, AC8)
        self.banner = tk.Label(root, text=banner_text(L), bg=CAUTION, fg="#000000", font=(FAMILY, 16, "bold"), pady=6)
        self.banner.pack(side="top", fill="x")
        who = T("lesson.who", L, user=user) if user else T("lesson.practice", L)
        tk.Label(root, text=T("lesson.sim_only", L) + "   " + who, bg=COL["bg"], fg=COL["dim"],
                 anchor="w").pack(side="top", fill="x", padx=10)
        body = tk.Frame(root, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=10, pady=6)
        # left: lessons by level
        left = tk.Frame(body, bg=COL["bg"])
        left.pack(side="left", fill="y")
        self.list = tk.Listbox(left, width=34, bg=COL["panel"], fg=COL["fg"], selectbackground=CAUTION,
                               selectforeground="#000000", activestyle="none", exportselection=False,
                               font=(FAMILY, 11), highlightthickness=0)
        self.list.pack(side="top", fill="y", expand=True)
        self.rows = []                          # list row -> lesson doc (None = a level header)
        self.fill_list()
        lb = tk.Frame(left, bg=COL["bg"])
        lb.pack(side="top", fill="x", pady=4)
        self.b_start = tk.Button(lb, text=T("lesson.start", L), command=self.start_selected)
        self.b_start.pack(side="left")
        self.b_retry = tk.Button(lb, text=T("lesson.retry", L), command=self.retry, state="disabled")
        self.b_retry.pack(side="left", padx=4)
        self.b_sim = tk.Button(left, text=T("lesson.start_sim", L), command=self.start_sim)
        self.b_sim.pack(side="top", fill="x")
        self.conn = tk.Label(left, text=T("lesson.not_connected", L), bg=COL["bg"], fg=COL["dim"], anchor="w",
                             justify="left", wraplength=260)
        self.conn.pack(side="top", fill="x", pady=4)
        # middle: the lesson
        mid = tk.Frame(body, bg=COL["bg"])
        mid.pack(side="left", fill="both", expand=True, padx=10)
        self.l_title = tk.Label(mid, text=T("lesson.pick", L), bg=COL["bg"], fg=COL["fg"], anchor="w",
                                font=(FAMILY, 16, "bold"), justify="left", wraplength=520)
        self.l_title.pack(side="top", fill="x")
        self.l_intro = tk.Label(mid, text="", bg=COL["bg"], fg=COL["dim"], anchor="nw", justify="left",
                                wraplength=520, font=(FAMILY, 11))
        self.l_intro.pack(side="top", fill="x", pady=(2, 8))
        self.l_where = tk.Label(mid, text="", bg=COL["bg"], fg=COL["dim"], anchor="w")
        self.l_where.pack(side="top", fill="x")
        self.l_step = tk.Label(mid, text="", bg=COL["panel"], fg=COL["fg"], anchor="nw", justify="left",
                               wraplength=520, font=(FAMILY, 15, "bold"), padx=8, pady=8)
        self.l_step.pack(side="top", fill="x", pady=4)
        self.l_hint = tk.Label(mid, text="", bg=COL["bg"], fg=CAUTION, anchor="nw", justify="left", wraplength=520,
                               font=(FAMILY, 12))
        self.l_hint.pack(side="top", fill="x")
        self.l_result = tk.Label(mid, text="", bg=COL["bg"], fg=GOOD, anchor="nw", justify="left", wraplength=520,
                                 font=(FAMILY, 13, "bold"))
        self.l_result.pack(side="top", fill="x", pady=6)
        self.l_state = tk.Label(mid, text="", bg=COL["panel"], fg=COL["fg"], anchor="nw", justify="left",
                                font=(FAMILY, 11), padx=8, pady=6, wraplength=520)
        self.l_state.pack(side="top", fill="x", pady=4)
        # right: the simulator's inputs
        right = tk.Frame(body, bg=COL["bg"])
        right.pack(side="left", fill="y")
        tk.Label(right, text=T("lesson.panel", L), bg=COL["bg"], fg=COL["fg"], font=(FAMILY, 11, "bold")).pack(anchor="w")
        pf = tk.Frame(right, bg=COL["bg"])
        pf.pack(side="top", fill="x")
        self.panel_buttons = {}
        for i, (key, lab) in enumerate(PANEL):
            b = tk.Button(pf, text=lab, width=9, command=lambda k=key: self.button(k),
                          bg="#7a1010" if key == "standby" else None, fg="#ffffff" if key == "standby" else None)
            b.grid(row=i // 3, column=i % 3, padx=2, pady=2)
            self.panel_buttons[key] = b
        tk.Label(right, text=T("lesson.joystick", L), bg=COL["bg"], fg=COL["fg"],
                 font=(FAMILY, 11, "bold")).pack(anchor="w", pady=(8, 0))
        self.pad = tk.Canvas(right, width=PAD, height=PAD, bg="#202a33", highlightthickness=0)
        self.pad.pack(side="top", anchor="w")
        self.pad.bind("<ButtonPress-1>", self._pad_drag)
        self.pad.bind("<B1-Motion>", self._pad_drag)
        self.pad.bind("<ButtonRelease-1>", self._pad_release)
        self.hold = tk.BooleanVar(value=False)          # "hold the stick": the knob stays where it was let go
        tk.Checkbutton(right, text=T("lesson.hold_stick", L), variable=self.hold, command=self._hold_changed,
                       bg=COL["bg"], fg=COL["fg"], selectcolor=COL["panel"], activebackground=COL["bg"],
                       activeforeground=COL["fg"]).pack(side="top", anchor="w")
        jb = tk.Frame(right, bg=COL["bg"])
        jb.pack(side="top", fill="x", pady=2)
        self.joy_buttons = {}
        for key in ("take", "confirm"):
            b = tk.Button(jb, text=key.upper(), width=10)
            b.bind("<ButtonPress-1>", lambda e, k=key: self.joy_press(k, True))
            b.bind("<ButtonRelease-1>", lambda e, k=key: self.joy_press(k, False))
            b.pack(side="left", padx=2)
            self.joy_buttons[key] = b
        tk.Label(right, text=T("lesson.pc", L), bg=COL["bg"], fg=COL["fg"],
                 font=(FAMILY, 11, "bold")).pack(anchor="w", pady=(8, 0))
        pc = tk.Frame(right, bg=COL["bg"])
        pc.pack(side="top", fill="x")
        self.pc_buttons = {}
        tk.Button(pc, text=T("lesson.ask_code", L), command=self.ask_code).grid(row=0, column=0, columnspan=2, sticky="we")
        self.code = tk.Entry(pc, width=8)
        self.code.grid(row=1, column=0, sticky="we", pady=2)
        tk.Button(pc, text=T("lesson.raise", L), command=self.raise_power).grid(row=1, column=1, sticky="we")
        tk.Button(pc, text=T("lesson.lower", L), command=self.lower_power).grid(row=2, column=0, columnspan=2, sticky="we")
        tk.Button(pc, text=T("lesson.ack", L), command=self.ack).grid(row=3, column=0, columnspan=2, sticky="we", pady=2)
        self.draw_pad()
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(TICK_MS, self.tick)

    # --- the lesson list
    def fill_list(self):
        self.list.delete(0, "end")
        self.rows = []
        done = set()
        if self.store is not None and self.user:
            try:
                done = lesson.done_ids(self.store.entries(), self.user)
            except Exception:
                done = set()
        for lv, docs in lesson.by_level(self.docs).items():
            if not docs:
                continue
            self.list.insert("end", "— %s —" % T("level.%s" % lv, self.lang))
            self.rows.append(None)
            for d in docs:
                mark = "✓ " if d["id"] in done else "   "
                self.list.insert("end", mark + (d.get("title_" + self.lang) or d.get("title_el")))
                self.rows.append(d)

    def select(self, lesson_id):
        for i, d in enumerate(self.rows):
            if d is not None and d["id"] == lesson_id:
                self.list.selection_clear(0, "end")
                self.list.selection_set(i)
                return True
        return False

    def start_selected(self):
        sel = self.list.curselection()
        if not sel or self.rows[sel[0]] is None:
            self.l_result.configure(text=T("lesson.pick", self.lang), fg=CAUTION)
            return None
        return self.begin(self.rows[sel[0]])

    # --- the simulator
    def start_sim(self):
        try:
            self.proc = self.starter()
            self.conn.configure(text=T("lesson.sim_started", self.lang), fg=COL["dim"])
        except (simguard.Refused, OSError) as e:
            self.conn.configure(text=T("lesson.sim_refused", self.lang, why=str(e)), fg=WARN)

    def connect(self):
        if self.sim is not None and not getattr(self.sim.link, "closed", False):
            return True
        try:
            self.sim = lesson.Sim(self.link_factory())
            self.conn.configure(text=T("lesson.connected", self.lang), fg=GOOD)
            return True
        except simguard.Refused as e:
            self.conn.configure(text=T("lesson.sim_refused", self.lang, why=str(e)), fg=WARN)
        except OSError:
            self.conn.configure(text=T("lesson.no_sim", self.lang), fg=WARN)
        self.sim = None
        return False

    # --- a lesson
    def begin(self, doc):
        if not self.connect():
            return None
        self.run = lesson.Run(doc, self.lang)
        self.logged = False
        for a in self.run.start(self.clock()):
            self.sim.do(a)
        self.code.delete(0, "end")
        self.l_title.configure(text=self.run.title())
        self.l_intro.configure(text=doc.get("intro_" + self.lang) or doc.get("intro_el", ""))
        self.l_result.configure(text="")
        self.b_retry.configure(state="normal")
        self.show()
        return self.run

    def retry(self):
        if self.run is None or not self.connect():
            return
        for a in self.run.retry(self.clock()):
            self.sim.do(a)
        self.logged = False
        self.l_result.configure(text="")
        self.show()

    def tick(self):
        if not self.alive:
            return
        try:
            self.step()
        finally:
            if self.alive:
                self.root.after(TICK_MS, self.tick)

    def step(self):
        """One 50 ms step: talk to the simulator, check the lesson (the tests call it directly)."""
        if self.sim is None:
            return
        if getattr(self.sim.link, "closed", False):
            self.conn.configure(text=T("lesson.no_sim", self.lang), fg=WARN)
            self.sim = None
            return
        state = self.sim.tick()
        self.feed_3d(state)
        if self.run is not None:
            status, acts = self.run.update(state, self.clock())
            for a in acts:
                self.sim.do(a)
            self.status = status
            if status in ("done", "failed") and not self.logged:
                self.finish()
        self.show()

    def feed_3d(self, state):
        """Step 22 (spec H): with the 3D backend in use, the simulated boat goes to live.jsonl for the 3D view
        (files only; the lesson's AURIX stays the simulator on this PC). The builtin backend writes nothing."""
        if self.run is None:
            return None
        if getattr(self, "_feed_run", None) is not self.run:
            self._feed_run = self.run
            try:
                self.feed = sim3d.feed_for(type(self.run).__module__, self.run.doc.get("id"))
                if self.feed is not None:
                    self.feed.t0 = self.clock()             # sample times on the window's clock
            except (simguard.Refused, OSError):
                self.feed = None
        if getattr(self, "feed", None) is not None:
            try:
                self.feed.write(state, self.clock())
            except OSError:
                self.feed = None
        return getattr(self, "feed", None)

    def finish(self):
        self.logged = True
        e = self.run.log_entry()
        if self.run.status == "done":
            self.l_result.configure(text=T("lesson.passed", self.lang, score=e["score"]), fg=GOOD)
        else:
            self.l_result.configure(text=T("lesson.failed", self.lang), fg=WARN)
        if self.store is not None and self.user:
            try:
                self.store.log(self.user, "lesson", **e)
            except Exception as ex:                          # a disk error: shown, the lesson itself is not lost
                self.l_result.configure(text=T("lesson.log_failed", self.lang, why=str(ex)), fg=WARN)
        if self.run.status == "done":
            self.fill_list()

    def show(self):
        L = self.lang
        st = self.sim.state if self.sim else {}
        if self.run is not None:
            r = self.run
            if r.index < 0:
                self.l_where.configure(text="")
                self.l_step.configure(text=T("lesson.waiting", L))
                self.l_hint.configure(text="")
            else:
                self.l_where.configure(text=T("lesson.step_of", L, n=r.index + 1, of=len(r.steps)))
                self.l_step.configure(text=r.text("do"))
                self.l_hint.configure(text=(T("lesson.hint", L) + " " + r.text("hint")) if r.hint_shown else "")
        if st:
            bits = alerts.bits_of(st)
            al = ", ".join(i18n.TL(L, "alarm." + c) for _b, c, _l in alerts.active(bits)) or "-"
            ck = st.get("confirm_kind", 0)
            pend = "-"
            if ck:
                pend = "%s (%.0f s)" % (proto.CONFIRM_KINDS[ck] if ck < len(proto.CONFIRM_KINDS) else ck,
                                        st.get("confirm_left_s", 0))
            self.l_state.configure(text=T(
                "lesson.state", L, mode=i18n.TL(L, "mode." + st.get("mode_name", "STANDBY")),
                heading="%.0f" % st.get("heading", 0), target="%.0f" % st.get("target", 0),
                throttle="%.0f" % st.get("throttle", 0), power="%.0f / %.0f" % (st.get("power_fwd", 0), st.get("power_rev", 0)),
                code=st.get("power_code") or "-", confirm=pend, alarms=al))

    # --- trainee inputs (the simulator's inputs; every one is checked by the simulated AURIX itself)
    def button(self, key):
        if self.sim:
            self.sim.button(key)

    def stick(self, x, y):
        if self.sim:
            self.sim.stick(x, y)
        self.draw_pad(x, y)

    def joy_press(self, key, on):
        if self.sim:
            self.sim.press(key, on)

    def ask_code(self):
        if self.sim:
            self.sim.power_code()

    def raise_power(self):
        if not self.sim:
            return
        try:
            c = int(self.code.get().strip())
        except ValueError:
            self.l_result.configure(text=T("lesson.bad_code", self.lang), fg=WARN)
            return
        self.sim.power_raise(c)

    def lower_power(self):
        if self.sim:
            self.sim.power_lower()

    def ack(self):
        if self.sim:
            self.sim.alarm_ack()

    def _pad_release(self, _e=None):
        if not self.hold.get():                         # a self-centring stick: let go = centre
            self.stick(0.0, 0.0)

    def _hold_changed(self):
        if not self.hold.get():
            self.stick(0.0, 0.0)

    def _pad_drag(self, e):
        x = max(-1.0, min(1.0, (e.x - PAD / 2.0) / (PAD / 2.0)))
        y = max(-1.0, min(1.0, (PAD / 2.0 - e.y) / (PAD / 2.0)))
        self.stick(x, y)

    def draw_pad(self, x=0.0, y=0.0):
        c = self.pad
        c.delete("all")
        m = PAD / 2.0
        c.create_line(m, 6, m, PAD - 6, fill="#5c7080")
        c.create_line(6, m, PAD - 6, m, fill="#5c7080")
        c.create_text(m, 10, text="Y+", fill=COL["dim"])
        c.create_text(PAD - 12, m - 10, text="X+", fill=COL["dim"])
        kx, ky, r = m + x * (m - 14), m - y * (m - 14), 14
        c.create_oval(kx - r, ky - r, kx + r, ky + r, fill=CAUTION if (x or y) else "#3a4652", outline="#000000")

    def close(self):
        self.alive = False
        if self.sim is not None:
            try:
                self.sim.do("centre")
                self.sim.link.close()
            except Exception:
                pass
        try:
            self.root.destroy()
        except tk.TclError:
            pass


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
    LessonWindow(root, text.lang())
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
