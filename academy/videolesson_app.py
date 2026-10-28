"""Real-trip video lessons + the lesson editor (ACADEMY_SPEC F, step 20) - the windows.

    python academy/videolesson_app.py [--lang el|en] [--editor]   (or the academy window's buttons)

VideoLessonWindow = the player: a lesson's camera picture (its OWN small trimmed copy, blurred) with the
station's HUD redrawn over it from the trip's log (groundstation/hud.py, the same painter as screen 1), in
sync; pause points ask a DECISION (choices) or SPOT THE HAZARD (click on the picture); the explanation after
every answer; the score -> the selected account's logbook (kind "videolesson").
EditorWindow = the lesson editor: open a saved event (event.json of the sync folder), preview any camera at
any time, trim, cameras, pause points (decision / hazard questions el + en, hazard areas drawn on the picture),
blur areas (faces, number plates), "Make lesson" = the small trimmed copies + the HUD timeline.
Everything through simguard (recorded files only; ffmpeg only on recorded files). Banner first (AC8).
"""
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import content                                                          # noqa: E402
import simguard                                                         # noqa: E402
import text                                                             # noqa: E402
import videolesson as vl                                                # noqa: E402  (groundstation/ on the path)
from banner import banner_text                                          # noqa: E402
from text import T                                                      # noqa: E402

import alerts                                                           # noqa: E402
import hud                                                              # noqa: E402

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

VW, VH = 640, 360                           # the picture
TICK_MS = 200                               # player clock / picture refresh
HUD_HIDE = ("controls", "drive", "relays", "minimap", "mirror_l", "mirror_r")   # station-only boxes
ANCHORS = {"c": "center"}
DRAW_KEYS = {"hazard": "ve.draw_hazard", "blur": "ve.draw_blur"}
POINT_KEYS = {"decision": "ve.p_decision", "hazard": "ve.p_hazard"}
FIELD_KEYS = {k: "ve.f_" + k for k in ("q_el", "q_en", "choices_el", "choices_en", "answer", "why_el", "why_en")}
TEXT_KEYS = set(DRAW_KEYS.values()) | set(POINT_KEYS.values()) | set(FIELD_KEYS.values()) | {"vl.failed", "vl.passed"}


def _banner(root, lang):
    b = tk.Label(root, text=banner_text(lang), bg=CAUTION, fg="#000000", font=(FAMILY, 16, "bold"), pady=6)
    b.pack(side="top", fill="x")
    return b


def draw_hud(canvas, ops, sx=1.0, sy=1.0, tag="hud"):
    """Paint groundstation/hud.py ops on a Tk canvas (outlines, lines, text; panels as a light stipple)."""
    def col(c):
        return c if isinstance(c, str) and c else ""
    for o in ops:
        k = o.get("k")
        if k in ("panel", "rect", "oval"):
            x0, y0, x1, y1 = o["r"]
            r = (x0 * sx, y0 * sy, x1 * sx, y1 * sy)
            if k == "panel":
                canvas.create_rectangle(*r, fill=col(o.get("fill")) or "#000000", outline="", stipple="gray25",
                                        tags=(tag,))
            elif k == "rect":
                canvas.create_rectangle(*r, fill=col(o.get("fill")), outline=col(o.get("outline")),
                                        width=o.get("w", 1) if o.get("outline") else 0, tags=(tag,))
            else:
                canvas.create_oval(*r, fill=col(o.get("fill")), outline=col(o.get("outline")), width=o.get("w", 1),
                                   tags=(tag,))
        elif k in ("line", "poly"):
            pts = [v * (sx if i % 2 == 0 else sy) for i, v in enumerate(o["pts"])]
            if len(pts) < 4:
                continue
            if k == "line":
                canvas.create_line(*pts, fill=col(o.get("fill")) or "#3cff6a", width=o.get("w", 1), tags=(tag,))
            elif len(pts) >= 6:
                canvas.create_polygon(*pts, fill=col(o.get("fill")), outline=col(o.get("outline")), tags=(tag,))
        elif k == "text":
            x, y = o["xy"]
            canvas.create_text(x * sx, y * sy, text=o.get("s", ""), fill=col(o.get("fill")) or "#3cff6a",
                               anchor=ANCHORS.get(o.get("anchor"), o.get("anchor") or "nw"),
                               font=(FAMILY, -max(7, int(o.get("size", 12) * sy)), "bold" if o.get("bold") else ""),
                               tags=(tag,))


def hud_ops(values, W=VW, H=VH, lamps=None):
    """The station's HUD for logged values (hud.Hud.build), station-only boxes hidden."""
    lay = hud.Layout.__new__(hud.Layout)            # the DEFAULT layout: never this PC's hud_layout.json (read
    lay.path = None                                 # or written back) - the lesson looks the same on every PC
    lay.rects = dict(hud.DEFAULT_LAYOUT)
    lay.hidden = set(HUD_HIDE)
    h = hud.Hud(lay)
    d = vl.hud_state(values)
    lamps = lamps or alerts.MasterLamps()
    lamps.update(alerts.bits_of(d))
    s = hud.Snapshot(t=d, age_s=0.0, lamps=lamps, alarm_rows=lamps.rows(), flash=False, declutter="FULL")
    ops, _hits = h.build(W, H, s)
    return ops


class Grabber(object):
    """One picture at a time in a thread: only the NEWEST wanted picture is fetched (the window never waits).
    The thread never touches Tk: the window polls for the result on its own timer."""

    def __init__(self, root, shown, grab=None):
        self.root, self.shown = root, shown
        self.grab = grab or simguard.video_frame
        self.want = None
        self.busy = False
        self.result = None
        self.polling = False

    def ask(self, path, t, width=VW):
        self.want = (path, round(t, 1), width)
        if not self.busy:
            self.busy = True
            threading.Thread(target=self._run, daemon=True).start()
        if not self.polling:
            self.polling = True
            self.root.after(40, self._poll)

    def _run(self):
        while True:
            w = self.want
            try:
                data = self.grab(w[0], w[1], w[2])
            except Exception:
                data = None
            if self.want == w:
                break
        self.result = (w, data)
        self.busy = False

    def _poll(self):
        r, self.result = self.result, None
        if r is not None:
            try:
                self.shown(*r)
            except tk.TclError:
                self.polling = False
                return
            if r[0] != self.want and not self.busy:     # asked again while the thread was finishing
                self.busy = True
                threading.Thread(target=self._run, daemon=True).start()
        if self.busy or self.result is not None:
            try:
                self.root.after(40, self._poll)
            except tk.TclError:
                self.polling = False
        else:
            self.polling = False


# --- the player ----------------------------------------------------------------------------------
class VideoLessonWindow(object):
    def __init__(self, root, lang=None, store=None, user=None, folder=None, grab=None, clock=time.time):
        self.root, self.lang, self.store, self.user = root, lang or text.lang(), store, user
        self.folder, self.clock = folder, clock
        L = self.lang
        root.title(T("vl.title", L))
        root.configure(bg=COL["bg"])
        try:
            root.geometry("1180x760")
        except tk.TclError:
            pass
        self.banner = _banner(root, L)
        tk.Label(root, text=T("vl.sim_note", L), bg=COL["bg"], fg=COL["dim"]).pack(side="top", anchor="w", padx=10)
        body = tk.Frame(root, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=8, pady=4)
        left = tk.Frame(body, bg=COL["bg"])
        left.pack(side="left", fill="y")
        tk.Label(left, text=T("vl.pick", L), bg=COL["bg"], fg=COL["fg"]).pack(side="top", anchor="w")
        self.list = tk.Listbox(left, width=42, height=24, exportselection=False)
        self.list.pack(side="top", fill="y", expand=True)
        self.b_start = tk.Button(left, text=T("vl.start", L), command=self.start_selected)
        self.b_start.pack(side="top", fill="x", pady=4)
        right = tk.Frame(body, bg=COL["bg"])
        right.pack(side="left", fill="both", expand=True, padx=8)
        self.l_title = tk.Label(right, text="", bg=COL["bg"], fg=COL["fg"], font=(FAMILY, 13, "bold"), anchor="w",
                                justify="left", wraplength=VW)
        self.l_title.pack(side="top", fill="x")
        self.canvas = tk.Canvas(right, width=VW, height=VH, bg="#000000", highlightthickness=0)
        self.canvas.pack(side="top", anchor="w")
        self.canvas.bind("<Button-1>", self.on_click)
        bar = tk.Frame(right, bg=COL["bg"])
        bar.pack(side="top", fill="x", pady=2)
        self.cam_bar = tk.Frame(bar, bg=COL["bg"])
        self.cam_bar.pack(side="left")
        self.b_play = tk.Button(bar, text=T("vl.play", L), command=self.toggle_play, state="disabled")
        self.b_play.pack(side="left", padx=6)
        self.hud_on = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text=T("vl.hud", L), variable=self.hud_on, command=self.redraw, bg=COL["bg"],
                       fg=COL["fg"], selectcolor=COL["panel"]).pack(side="left")
        self.l_time = tk.Label(bar, text="", bg=COL["bg"], fg=COL["fg"])
        self.l_time.pack(side="left", padx=6)
        self.q = tk.Frame(right, bg=COL["panel"])
        self.q.pack(side="top", fill="x", pady=4)
        self.l_q = tk.Label(self.q, text="", bg=COL["panel"], fg=CAUTION, font=(FAMILY, 12, "bold"), anchor="w",
                            justify="left", wraplength=VW)
        self.l_q.pack(side="top", fill="x", padx=6, pady=4)
        self.choices = tk.Frame(self.q, bg=COL["panel"])
        self.choices.pack(side="top", fill="x", padx=6)
        self.l_fb = tk.Label(self.q, text="", bg=COL["panel"], fg=COL["fg"], anchor="w", justify="left",
                             wraplength=VW)
        self.l_fb.pack(side="top", fill="x", padx=6, pady=4)
        self.b_go = tk.Button(self.q, text=T("vl.continue", L), command=self.resume, state="disabled")
        self.b_go.pack(side="top", anchor="w", padx=6, pady=4)
        self.l_result = tk.Label(right, text="", bg=COL["bg"], fg=COL["fg"], anchor="w", justify="left")
        self.l_result.pack(side="top", fill="x")
        self.grabber = Grabber(root, self._shown, grab)
        self.photo = None
        self.run = None
        self.doc = self.dir = None
        self.cam = None
        self.timeline = []
        self.playing = False
        self.logged = False
        self._last = None
        self._job = None
        self.fill_list()

    # --- the list
    def fill_list(self):
        self.list.delete(0, "end")
        self.rows = []
        done = set()
        if self.store is not None and self.user:
            try:
                done = {e.get("id") for e in self.store.entries()
                        if e.get("user") == self.user and e.get("kind") == "videolesson" and e.get("done")}
            except Exception:
                done = set()
        items = vl.load_lessons(self.folder)
        if not items:
            self.list.insert("end", T("vl.none", self.lang))
            self.rows.append(None)
            return
        groups = {}
        for it in items:
            groups.setdefault(it[0].get("level", "DRIVER") if it[0].get("level") in content.LEVELS else "DRIVER",
                              []).append(it)
        for lv in content.LEVELS:
            if not groups.get(lv):
                continue
            self.list.insert("end", "— %s —" % T("level.%s" % lv, self.lang))
            self.rows.append(None)
            for doc, folder, probs in groups[lv]:
                mark = "✓ " if doc.get("id") in done else "   "
                title = doc.get("title_" + self.lang) or doc.get("title_el") or doc.get("id")
                self.list.insert("end", mark + title + (" (!)" if probs else ""))
                self.rows.append((doc, folder, probs))

    def select(self, lesson_id):
        for i, r in enumerate(self.rows):
            if r and r[0].get("id") == lesson_id:
                self.list.selection_clear(0, "end")
                self.list.selection_set(i)
                return True
        return False

    # --- a lesson
    def start_selected(self):
        sel = self.list.curselection()
        r = self.rows[sel[0]] if sel and sel[0] < len(self.rows) else None
        if not r:
            self.l_result.configure(text=T("vl.pick", self.lang), fg=COL["fg"])
            return None
        doc, folder, probs = r
        if probs:
            self.l_result.configure(text=T("vl.bad", self.lang, why="; ".join(probs)[:300]), fg=WARN)
            return None
        self.doc, self.dir = doc, folder
        self.run = vl.Run(doc, self.lang)
        self.timeline = vl.read_timeline(doc, folder)
        self.cam = doc["main_camera"]
        self.logged = False
        for w in self.cam_bar.winfo_children():
            w.destroy()
        for c in doc["cameras"]:
            tk.Button(self.cam_bar, text=c, command=lambda c=c: self.set_camera(c)).pack(side="left", padx=2)
        intro = self.run.tx(doc, "intro")
        self.l_title.configure(text=(self.run.tx(doc, "title") + ("\n" + intro if intro else "")))
        if not self.timeline:
            self.l_result.configure(text=T("vl.no_hud", self.lang), fg=COL["dim"])
        else:
            self.l_result.configure(text="")
        self.b_play.configure(state="normal")
        self.show_question()
        self.playing = False
        self.toggle_play()
        return self.run

    def set_camera(self, cam):
        if self.doc and cam in self.doc["cameras"]:
            self.cam = cam
            self.redraw()

    def toggle_play(self):
        if self.run is None or self.run.finished:
            return
        self.playing = not self.playing and self.run.asking is None and self.run.answered is None
        self.b_play.configure(text=T("vl.pause" if self.playing else "vl.play", self.lang))
        self._last = self.clock()
        if self.playing and self._job is None:
            self._job = self.root.after(TICK_MS, self._tick)

    def _tick(self):
        self._job = None
        if not self.playing or self.run is None:
            return
        self.step()
        if self.playing:
            self._job = self.root.after(TICK_MS, self._tick)

    def step(self, dt=None):
        """Advance the lesson clock (dt = seconds, else the real time since the last step)."""
        now = self.clock()
        if dt is None:
            dt = now - (self._last or now)
        self._last = now
        r = self.run
        hit = r.advance(r.t + dt)
        self.redraw()
        if hit is not None:
            self.playing = False
            self.b_play.configure(text=T("vl.play", self.lang))
            self.show_question()
        elif r.finished:
            self.playing = False
            self.finish()
        return hit

    def redraw(self):
        if self.run is None:
            return
        r = self.run
        self.l_time.configure(text=T("vl.time", self.lang, t="%.1f" % r.t, of="%.0f" % r.length))
        try:
            self.grabber.ask(vl.media_path(self.doc, self.dir, self.cam), r.t, VW)
        except simguard.Refused as e:
            self.l_result.configure(text=T("refuse.recording", self.lang, why=str(e)), fg=WARN)
        self.paint_overlay()

    def _shown(self, want, data):
        if data:
            try:
                self.photo = tk.PhotoImage(data=data, format="ppm")
            except tk.TclError:
                self.photo = None
        self.canvas.delete("frame")
        if self.photo is not None:
            self.canvas.create_image(0, 0, image=self.photo, anchor="nw", tags=("frame",))
            self.canvas.tag_lower("frame")
        else:
            self.canvas.create_text(VW / 2, VH / 2, text=T("vl.no_picture", self.lang), fill="#ffffff",
                                    tags=("frame",))

    def paint_overlay(self):
        self.canvas.delete("hud")
        self.canvas.delete("mark")
        self.canvas.create_text(8, 8, text=self.cam or "", fill="#ffffff", anchor="nw", font=(FAMILY, 11, "bold"),
                                tags=("mark",))
        if self.hud_on.get() and self.timeline:
            vals = vl.at_time(self.timeline, self.run.t)
            if vals is not None:
                draw_hud(self.canvas, hud_ops(vals))
        a = self.run.answered
        if a is not None:                               # after a hazard answer: the right areas are shown
            pt = self.run.points[a[0]]
            if pt.get("type") == "hazard" and pt.get("camera") == self.cam:
                for b in pt["areas"]:
                    self.canvas.create_rectangle(b["x"] * VW, b["y"] * VH, (b["x"] + b["w"]) * VW,
                                                 (b["y"] + b["h"]) * VH, outline=GOOD, width=3, tags=("mark",))

    # --- questions
    def show_question(self):
        for w in self.choices.winfo_children():
            w.destroy()
        r = self.run
        pt = r.point() if r else None
        self.b_go.configure(state="disabled")
        self.l_fb.configure(text="")
        if pt is None:
            self.l_q.configure(text=T("vl.watch", self.lang) if r else "")
            return
        q = r.tx(pt, "q")
        if pt["type"] == "decision":
            self.l_q.configure(text=T("vl.decide", self.lang, q=q))
            self.choice_buttons = []
            for k, c in enumerate(pt["choices"]):
                b = tk.Button(self.choices, text=r.tx(c, "text"), anchor="w", justify="left", wraplength=VW - 40,
                              command=lambda k=k: self.choose(k))
                b.pack(side="top", fill="x", pady=1)
                self.choice_buttons.append(b)
        else:
            if pt.get("camera") != self.cam:
                self.set_camera(pt["camera"])
            self.l_q.configure(text=T("vl.spot", self.lang, q=q, cam=pt["camera"]))

    def _answered(self, right):
        if right is None:
            return
        self.l_fb.configure(text=(T("vl.right", self.lang) if right else T("vl.wrong", self.lang)) + "  " +
                            self.run.explanation(), fg=GOOD if right else WARN)
        for w in self.choices.winfo_children():
            w.configure(state="disabled")
        self.b_go.configure(state="normal")
        self.paint_overlay()

    def choose(self, k):
        if self.run is not None:
            self._answered(self.run.answer_choice(k))

    def on_click(self, e):
        if self.run is None or self.run.point() is None:
            return
        self._answered(self.run.click(self.cam, e.x / float(VW), e.y / float(VH)))

    def resume(self):
        if self.run is None:
            return
        self.run.resume()
        self.show_question()
        if self.run.finished:
            return self.finish()
        self.playing = False
        self.toggle_play()

    def finish(self):
        if self.logged or self.run is None:
            return
        self.logged = True
        e = self.run.log_entry()
        self.l_q.configure(text="")
        self.l_result.configure(text=T("vl.passed" if e["done"] else "vl.failed", self.lang, score=e["score"],
                                       right=e["right"], of=e["of"]), fg=GOOD if e["done"] else WARN)
        self.b_play.configure(state="disabled")
        if self.store is not None and self.user:
            try:
                self.store.log(self.user, "videolesson", **e)
            except Exception as ex:
                self.l_result.configure(text=T("lesson.log_failed", self.lang, why=str(ex)), fg=WARN)
        self.fill_list()

    def close(self):
        self.playing = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass


# --- the editor ----------------------------------------------------------------------------------
class EditorWindow(object):
    """The lesson editor. Times on screen = seconds from the event start."""

    FIELDS = ("q_el", "q_en", "choices_el", "choices_en", "answer", "why_el", "why_en")

    def __init__(self, root, lang=None, grab=None, folder=None):
        self.root, self.lang = root, lang or text.lang()
        self.folder = folder
        L = self.lang
        root.title(T("ve.title", L))
        root.configure(bg=COL["bg"])
        try:
            root.geometry("1280x860")
        except tk.TclError:
            pass
        self.banner = _banner(root, L)
        top = tk.Frame(root, bg=COL["bg"])
        top.pack(side="top", fill="x", padx=8, pady=4)
        tk.Button(top, text=T("ve.open", L), command=self.ask_event).pack(side="left")
        self.l_ev = tk.Label(top, text=T("ve.no_event", L), bg=COL["bg"], fg=COL["fg"], anchor="w", justify="left")
        self.l_ev.pack(side="left", padx=8)
        body = tk.Frame(root, bg=COL["bg"])
        body.pack(side="top", fill="both", expand=True, padx=8)
        left = tk.Frame(body, bg=COL["bg"])
        left.pack(side="left", fill="y")
        self.canvas = tk.Canvas(left, width=VW, height=VH, bg="#000000", highlightthickness=0)
        self.canvas.pack(side="top")
        self.canvas.bind("<ButtonPress-1>", self.drag_start)
        self.canvas.bind("<B1-Motion>", self.drag_move)
        self.canvas.bind("<ButtonRelease-1>", self.drag_end)
        cur = tk.Frame(left, bg=COL["bg"])
        cur.pack(side="top", fill="x")
        self.pcam = tk.StringVar()
        self.cb_cam = ttk.Combobox(cur, textvariable=self.pcam, state="readonly", width=14)
        self.cb_cam.pack(side="left")
        self.cb_cam.bind("<<ComboboxSelected>>", lambda _e: self.preview())
        self.pos = tk.DoubleVar(value=0.0)
        self.scale = tk.Scale(cur, variable=self.pos, from_=0, to=1, resolution=0.5, orient="horizontal", length=440,
                              showvalue=True, command=lambda _v: self.preview(), bg=COL["bg"], fg=COL["fg"],
                              highlightthickness=0)
        self.scale.pack(side="left", padx=4)
        trim = tk.Frame(left, bg=COL["bg"])
        trim.pack(side="top", fill="x", pady=2)
        tk.Button(trim, text=T("ve.start_here", L), command=lambda: self.set_trim("start")).pack(side="left")
        tk.Button(trim, text=T("ve.end_here", L), command=lambda: self.set_trim("end")).pack(side="left", padx=4)
        self.l_trim = tk.Label(trim, text="", bg=COL["bg"], fg=COL["fg"])
        self.l_trim.pack(side="left", padx=6)
        mode = tk.Frame(left, bg=COL["bg"])
        mode.pack(side="top", fill="x", pady=2)
        tk.Label(mode, text=T("ve.draw", L), bg=COL["bg"], fg=COL["fg"]).pack(side="left")
        self.draw_mode = tk.StringVar(value="hazard")
        for v in ("hazard", "blur"):
            tk.Radiobutton(mode, text=T(DRAW_KEYS[v], L), variable=self.draw_mode, value=v, bg=COL["bg"],
                           fg=COL["fg"], selectcolor=COL["panel"]).pack(side="left")
        tk.Label(mode, text=T("ve.blur_s", L), bg=COL["bg"], fg=COL["fg"]).pack(side="left", padx=(10, 2))
        self.blur_s = tk.StringVar(value="10")
        tk.Entry(mode, textvariable=self.blur_s, width=5).pack(side="left")
        tk.Button(mode, text=T("ve.clear_areas", L), command=self.clear_areas).pack(side="left", padx=6)
        right = tk.Frame(body, bg=COL["bg"])
        right.pack(side="left", fill="both", expand=True, padx=8)
        meta = tk.Frame(right, bg=COL["bg"])
        meta.pack(side="top", fill="x")
        self.v_id, self.v_tel, self.v_ten, self.v_level = tk.StringVar(), tk.StringVar(), tk.StringVar(), \
            tk.StringVar(value="DRIVER")
        for row, (key, var) in enumerate((("ve.id", self.v_id), ("ve.title_el", self.v_tel),
                                          ("ve.title_en", self.v_ten))):
            tk.Label(meta, text=T(key, L), bg=COL["bg"], fg=COL["fg"]).grid(row=row, column=0, sticky="w")
            tk.Entry(meta, textvariable=var, width=40).grid(row=row, column=1, sticky="we")
        tk.Label(meta, text=T("ve.level", L), bg=COL["bg"], fg=COL["fg"]).grid(row=3, column=0, sticky="w")
        ttk.Combobox(meta, textvariable=self.v_level, values=content.LEVELS, state="readonly",
                     width=12).grid(row=3, column=1, sticky="w")
        tk.Label(right, text=T("ve.cameras", L), bg=COL["bg"], fg=COL["fg"]).pack(side="top", anchor="w", pady=(6, 0))
        self.cam_frame = tk.Frame(right, bg=COL["bg"])
        self.cam_frame.pack(side="top", fill="x")
        self.cam_vars = {}
        self.main_cam = tk.StringVar()
        q = tk.LabelFrame(right, text=T("ve.question", L), bg=COL["bg"], fg=COL["fg"])
        q.pack(side="top", fill="x", pady=4)
        self.qv = {}
        for row, k in enumerate(self.FIELDS):
            tk.Label(q, text=T(FIELD_KEYS[k], L), bg=COL["bg"], fg=COL["fg"]).grid(row=row, column=0, sticky="w")
            self.qv[k] = tk.StringVar()
            tk.Entry(q, textvariable=self.qv[k], width=46).grid(row=row, column=1, sticky="we")
        qb = tk.Frame(q, bg=COL["bg"])
        qb.grid(row=len(self.FIELDS), column=0, columnspan=2, sticky="w", pady=2)
        tk.Button(qb, text=T("ve.add_decision", L), command=self.add_decision).pack(side="left")
        tk.Button(qb, text=T("ve.add_hazard", L), command=self.add_hazard).pack(side="left", padx=4)
        tk.Label(right, text=T("ve.points", L), bg=COL["bg"], fg=COL["fg"]).pack(side="top", anchor="w")
        self.l_points = tk.Listbox(right, height=7, exportselection=False)
        self.l_points.pack(side="top", fill="x")
        tk.Label(right, text=T("ve.blurs", L), bg=COL["bg"], fg=COL["fg"]).pack(side="top", anchor="w")
        self.l_blur = tk.Listbox(right, height=4, exportselection=False)
        self.l_blur.pack(side="top", fill="x")
        bb = tk.Frame(right, bg=COL["bg"])
        bb.pack(side="top", fill="x", pady=4)
        tk.Button(bb, text=T("ve.delete", L), command=self.delete_selected).pack(side="left")
        self.b_make = tk.Button(bb, text=T("ve.make", L), command=self.make, state="disabled")
        self.b_make.pack(side="left", padx=6)
        self.msg = tk.Label(root, text="", bg=COL["bg"], fg=COL["fg"], anchor="w", justify="left", wraplength=1200)
        self.msg.pack(side="top", fill="x", padx=8, pady=4)
        self.grabber = Grabber(root, self._shown, grab)
        self.photo = None
        self.draft = None
        self.areas = []                     # hazard areas drawn for the next hazard question
        self._drag = None
        self.making = None

    def say(self, s, colour=None):
        self.msg.configure(text=s, fg=colour or COL["fg"])

    # --- the event
    def ask_event(self):
        p = filedialog.askopenfilename(parent=self.root, title=T("ve.open", self.lang),
                                       filetypes=[("saved event", "event.json")])
        if p:
            self.open_event(p)

    def open_event(self, path):
        try:
            ev = vl.Event(path)
        except (simguard.Refused, OSError, ValueError) as e:
            self.say(T("refuse.recording", self.lang, why=str(e)), WARN)
            return None
        if not ev.cameras():
            self.say(T("ve.no_video", self.lang), WARN)
            return None
        self.draft = vl.Draft(ev)
        d = self.draft
        self.v_id.set(d.id)
        self.l_ev.configure(text=T("ve.event", self.lang, id=ev.id, trig=ev.trigger_text or "-",
                                   len="%.0f" % d.length_event, cams=", ".join(ev.cameras()),
                                   log=T("acc.yes" if ev.log_path() else "acc.no", self.lang)))
        for w in self.cam_frame.winfo_children():
            w.destroy()
        self.cam_vars = {}
        self.main_cam.set(d.main)
        for c in ev.cameras():
            v = tk.BooleanVar(value=True)
            self.cam_vars[c] = v
            tk.Checkbutton(self.cam_frame, text=c, variable=v, bg=COL["bg"], fg=COL["fg"],
                           selectcolor=COL["panel"]).pack(side="left")
            tk.Radiobutton(self.cam_frame, text=T("ve.main", self.lang), variable=self.main_cam, value=c,
                           bg=COL["bg"], fg=COL["fg"], selectcolor=COL["panel"]).pack(side="left", padx=(0, 8))
        self.cb_cam.configure(values=ev.cameras())
        self.pcam.set(d.main)
        self.scale.configure(to=max(1.0, d.length_event))
        self.pos.set(round((ev.trigger_utc or ev.start_utc) - ev.start_utc, 1))
        self.areas = []
        self.b_make.configure(state="normal")
        self.refresh()
        self.preview()
        self.say(T("ve.opened", self.lang))
        return d

    def refresh(self):
        d = self.draft
        if d is None:
            return
        self.l_trim.configure(text=T("ve.trim", self.lang, a="%.1f" % d.start, b="%.1f" % d.end,
                                     n="%.1f" % (d.end - d.start)))
        self.l_points.delete(0, "end")
        for p in d.points:
            what = T(POINT_KEYS[p["type"]], self.lang)
            inside = d.start <= p["t"] <= d.end
            self.l_points.insert("end", "%7.1f s  %s  %s%s" % (p["t"], what, p.get("q_" + self.lang) or p["q_el"],
                                                                "" if inside else "  " + T("ve.outside", self.lang)))
        self.l_blur.delete(0, "end")
        for b in d.blur:
            self.l_blur.insert("end", "%s  %.1f-%.1f s  (%.2f, %.2f, %.2f, %.2f)" % (
                b["camera"], b["t0"], b["t1"], b["x"], b["y"], b["w"], b["h"]))
        self.paint_marks()

    def set_trim(self, which):
        d = self.draft
        if d is None:
            return
        t = float(self.pos.get())
        a, b = (t, d.end) if which == "start" else (d.start, t)
        try:
            d.trim(a, b)
            self.say("")
        except ValueError as e:
            self.say(T("ve.bad_trim", self.lang, why=str(e)), WARN)
        self.refresh()

    # --- the preview + drawing boxes
    def preview(self):
        d = self.draft
        if d is None:
            return
        loc = d.ev.locate(self.pcam.get(), d.ev.start_utc + float(self.pos.get()))
        if loc is None:
            self.photo = None
            self._shown(None, None)
            return
        self.grabber.ask(loc[0], loc[1], VW)

    def _shown(self, _want, data):
        self.photo = None
        if data:
            try:
                self.photo = tk.PhotoImage(data=data, format="ppm")
            except tk.TclError:
                self.photo = None
        self.canvas.delete("frame")
        if self.photo is not None:
            self.canvas.create_image(0, 0, image=self.photo, anchor="nw", tags=("frame",))
            self.canvas.tag_lower("frame")
        else:
            self.canvas.create_text(VW / 2, VH / 2, text=T("vl.no_picture", self.lang), fill="#ffffff",
                                    tags=("frame",))
        self.paint_marks()

    def paint_marks(self):
        self.canvas.delete("mark")
        cam, t = self.pcam.get(), float(self.pos.get())
        d = self.draft
        if d is not None:
            for b in d.blur:
                if b["camera"] == cam and b["t0"] <= t <= b["t1"]:
                    self.canvas.create_rectangle(b["x"] * VW, b["y"] * VH, (b["x"] + b["w"]) * VW,
                                                 (b["y"] + b["h"]) * VH, outline="#8080ff", width=2, dash=(4, 2),
                                                 tags=("mark",))
        for a in self.areas:
            self.canvas.create_rectangle(a["x"] * VW, a["y"] * VH, (a["x"] + a["w"]) * VW, (a["y"] + a["h"]) * VH,
                                         outline=CAUTION, width=2, tags=("mark",))

    def drag_start(self, e):
        self._drag = (e.x, e.y)

    def drag_move(self, e):
        if self._drag:
            self.canvas.delete("rubber")
            self.canvas.create_rectangle(self._drag[0], self._drag[1], e.x, e.y, outline="#ffffff", tags=("rubber",))

    def drag_end(self, e):
        if not self._drag:
            return
        x0, y0 = self._drag
        self._drag = None
        self.canvas.delete("rubber")
        self.add_box(x0 / float(VW), y0 / float(VH), e.x / float(VW), e.y / float(VH))

    def add_box(self, xa, ya, xb, yb):
        """A box drawn on the picture (parts 0..1): a hazard area or a blur area (from the cursor for blur_s)."""
        x0, x1 = sorted((min(max(xa, 0.0), 1.0), min(max(xb, 0.0), 1.0)))
        y0, y1 = sorted((min(max(ya, 0.0), 1.0), min(max(yb, 0.0), 1.0)))
        if x1 - x0 < 0.01 or y1 - y0 < 0.01 or self.draft is None:
            return None
        box = {"x": round(x0, 4), "y": round(y0, 4), "w": round(x1 - x0, 4), "h": round(y1 - y0, 4)}
        if self.draw_mode.get() == "blur":
            try:
                n = max(0.5, float(self.blur_s.get()))
            except ValueError:
                n = 10.0
            t = float(self.pos.get())
            self.draft.add_blur(self.pcam.get(), box, t, min(self.draft.length_event, t + n))
        else:
            self.areas.append(box)
        self.refresh()
        return box

    def clear_areas(self):
        self.areas = []
        self.paint_marks()

    # --- questions
    def _texts(self):
        v = {k: self.qv[k].get().strip() for k in self.FIELDS}
        need = ("q_el", "q_en", "why_el", "why_en")
        if not all(v[k] for k in need):
            self.say(T("ve.need_texts", self.lang), WARN)
            return None
        return v

    def add_decision(self):
        v = self._texts()
        if v is None or self.draft is None:
            return None
        ce = [c.strip() for c in v["choices_el"].split(";") if c.strip()]
        cn = [c.strip() for c in v["choices_en"].split(";") if c.strip()]
        try:
            ans = int(v["answer"]) - 1
        except ValueError:
            ans = -1
        if len(ce) < 2 or len(ce) != len(cn) or not 0 <= ans < len(ce):
            self.say(T("ve.need_choices", self.lang), WARN)
            return None
        pt = self.draft.add_decision(float(self.pos.get()), {"el": v["q_el"], "en": v["q_en"]},
                                     [{"el": a, "en": b} for a, b in zip(ce, cn)], ans,
                                     {"el": v["why_el"], "en": v["why_en"]})
        self.refresh()
        self.say(T("ve.added", self.lang))
        return pt

    def add_hazard(self):
        v = self._texts()
        if v is None or self.draft is None:
            return None
        if not self.areas:
            self.say(T("ve.need_areas", self.lang), WARN)
            return None
        pt = self.draft.add_hazard(float(self.pos.get()), self.pcam.get(), {"el": v["q_el"], "en": v["q_en"]},
                                   {"el": v["why_el"], "en": v["why_en"]}, self.areas)
        self.areas = []
        self.refresh()
        self.say(T("ve.added", self.lang))
        return pt

    def delete_selected(self):
        if self.draft is None:
            return
        s = self.l_points.curselection()
        if s:
            self.draft.remove_point(s[0])
        s = self.l_blur.curselection()
        if s:
            self.draft.remove_blur(s[0])
        self.refresh()

    # --- making the lesson
    def apply_form(self):
        d = self.draft
        d.id = self.v_id.get().strip() or d.id
        d.title = {"el": self.v_tel.get().strip(), "en": self.v_ten.get().strip()}
        d.level = self.v_level.get() if self.v_level.get() in content.LEVELS else "DRIVER"
        d.set_cameras([c for c, v in self.cam_vars.items() if v.get()], self.main_cam.get())

    def make(self, wait=False):
        """Makes the lesson in a thread (copies take a while); wait=True (tests) joins it."""
        if self.draft is None or self.making is not None:
            return None
        try:
            self.apply_form()
        except ValueError as e:
            self.say(T("ve.failed", self.lang, why=str(e)), WARN)
            return None
        probs = [p for p in self.draft.problems() if "missing" not in p]
        if probs:
            self.say(T("ve.failed", self.lang, why="; ".join(probs)[:400]), WARN)
            return None
        self.say(T("ve.making", self.lang))
        self.b_make.configure(state="disabled")
        self.result = None

        def work():
            try:
                self.result = ("ok",) + self.draft.make(self.folder)
            except Exception as e:                                  # a refusal / ffmpeg / disk error: said
                self.result = ("failed", str(e))
        self.making = threading.Thread(target=work, daemon=True)
        self.making.start()
        if wait:
            self.making.join()
            self._made()
        else:
            self.root.after(300, self._poll)
        return True

    def _poll(self):
        if self.making is not None and self.making.is_alive():
            self.root.after(300, self._poll)
        else:
            self._made()

    def _made(self):
        self.making = None
        self.b_make.configure(state="normal")
        r = self.result
        if r and r[0] == "ok":
            self.say(T("ve.made", self.lang, path=r[1]) + ("  " + "; ".join(r[3]) if r[3] else ""), GOOD)
        elif r:
            self.say(T("ve.failed", self.lang, why=r[1]), WARN)

    def close(self):
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
    root = tk.Tk()
    if "--editor" in argv:
        EditorWindow(root, text.lang())
    else:
        VideoLessonWindow(root, text.lang())
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
