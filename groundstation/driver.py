"""DRIVER screen of the two-monitor ground station (section I).

The captain's screen is the normal ground-station window (app.py); this is a
second window for the driver, meant for the second monitor:

  +-------------+------------------------------+-------------+
  | REAR-LEFT   |  MODE   alarms               | REAR-RIGHT  |
  | (mirror)    |  channel / latency / REC     | (mirror)    |
  +------+------+------------------------------+------+------+
  | thr. |            FRONT camera               |speed |slam  |
  | bar  |       (FRONT-2 if FRONT-1 is lost)    |      |graph |
  |      |                                       | mini map    |
  +------+---------------------------------------+-------------+
  |          compass tape (heading, target)        |   SAVE    |
  +------------------------------------------------+-----------+

Captain actions that change the driving appear here as a question
(ACCEPT / REJECT); no answer within the timeout = rejected.
The drawing maths are plain functions (tested in test_driver.py).
"""
import math
import time
import tkinter as tk

import proto

BG = "#000000"
FG = "#ffffff"
RED = "#ff3030"
YEL = "#ffd000"
GRN = "#40e040"
F_HUD = ("Segoe UI", 16, "bold")
F_BIG = ("Segoe UI", 34, "bold")
F_MID = ("Segoe UI", 20, "bold")
F_SM = ("Segoe UI", 11)


# --- drawing maths (no tk) ------------------------------------------------------------

def flip_ppm(data):
    """A binary PPM (P6) mirrored left <-> right (the rear cameras as mirrors)."""
    if not data or data[:2] != b"P6":
        return data
    parts, i = [], 2
    while len(parts) < 3:                       # width, height, maxval (header may have comments)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(int(data[i:j]))
        i = j
    i += 1
    w, h = parts[0], parts[1]
    head, px = data[:i], data[i:]
    row = w * 3
    out = bytearray(len(px))
    for y in range(h):
        r = px[y * row:(y + 1) * row]
        rev = bytearray(row)
        rev[0::3], rev[1::3], rev[2::3] = r[-3::-3], r[-2::-3], r[-1::-3]
        out[y * row:(y + 1) * row] = rev
    return head + bytes(out)


def angdiff(a, b):
    """a - b in -180..180."""
    return (a - b + 540.0) % 360.0 - 180.0


def tape_ticks(heading, width, span=90.0):
    """Compass tape: [(x, deg, label or None)] for every 5 deg in view."""
    out = []
    first = math.ceil((heading - span / 2) / 5.0) * 5
    d = first
    while d <= heading + span / 2:
        x = width / 2 + angdiff(d % 360, heading) / span * width
        deg = int(round(d)) % 360
        lab = {0: "N", 90: "E", 180: "S", 270: "W"}.get(deg, "%03d" % deg) if deg % 30 == 0 else None
        out.append((x, deg, lab))
        d += 5
    return out


def tape_x(heading, target, width, span=90.0):
    """x of the target on the tape, clamped to the edge (arrow) if outside."""
    x = width / 2 + angdiff(target, heading) / span * width
    return max(0.0, min(float(width), x))


def slam_geom(peaks, level, limit, h, absolute=False, yellow_frac=0.7):
    """Slamming bar graph. Relative (default): centred on the running
    average (level) - bars go up for harder than usual, down for softer;
    absolute: from 0 g at the bottom. Returns (bars, limit_y, center_y) with
    bars = [(index, y_from, y_to, colour)]."""
    if absolute:
        top = max(limit * 1.2, max(peaks or [0.0]) * 1.05, 1.0)
        Y = lambda v: h - v / top * h
        base = h
    else:
        rng = max(limit - level, level, 0.5) * 1.1
        Y = lambda v: h / 2 - (v - level) / rng * (h / 2)
        base = h / 2
    bars = []
    for i, p in enumerate(peaks):
        col = RED if p >= limit else (YEL if p >= yellow_frac * limit else GRN)
        y = max(0.0, min(float(h), Y(p)))
        bars.append((i, base, y, col))
    return bars, max(0.0, min(float(h), Y(limit))), base


def throttle_geom(out, plim, cap, h):
    """Throttle bar: y of the throttle, the power limit and the slamming cap
    (0 % at the bottom, 100 % at the top)."""
    Y = lambda p: h - max(0.0, min(100.0, p)) / 100.0 * h
    return {"out": Y(out), "plim": Y(plim), "cap": Y(cap) if cap is not None and cap < 100 else None}


# --- the window ---------------------------------------------------------------------------

class DriverScreen:
    def __init__(self, app, geometry=None):
        self.app = app
        self.w = tk.Toplevel(app.root, bg=BG)
        self.w.title("Trygons C2 - DRIVER")
        self.w.geometry(geometry or "1600x900")
        self.w.protocol("WM_DELETE_WINDOW", app.close_driver)
        self.absolute = False
        self.photos = {}
        self.question = None                    # (text, on_yes, on_no, deadline)
        self.answers = []                       # (text, result) for tests
        w = self.w
        for c in range(5):
            w.grid_columnconfigure(c, weight=1 if c == 2 else 0)
        w.grid_rowconfigure(1, weight=1)
        self.mirror_l = tk.Canvas(w, width=400, height=225, bg="#101010", highlightthickness=0)
        self.mirror_l.grid(row=0, column=0, columnspan=2, sticky="nw", padx=4, pady=4)
        self.status = tk.Canvas(w, height=225, bg=BG, highlightthickness=0)
        self.status.grid(row=0, column=2, sticky="nsew", pady=4)
        self.mirror_r = tk.Canvas(w, width=400, height=225, bg="#101010", highlightthickness=0)
        self.mirror_r.grid(row=0, column=3, columnspan=2, sticky="ne", padx=4, pady=4)
        self.thr = tk.Canvas(w, width=110, bg=BG, highlightthickness=0)
        self.thr.grid(row=1, column=0, sticky="ns", padx=4)
        self.front = tk.Canvas(w, bg="#101010", highlightthickness=0)
        self.front.grid(row=1, column=1, columnspan=3, sticky="nsew")
        side = tk.Frame(w, bg=BG)
        side.grid(row=1, column=4, sticky="ns", padx=4)
        self.speed = tk.Label(side, text="--.- kn", font=F_BIG, bg=BG, fg=FG)
        self.speed.pack(pady=(4, 0))
        self.slam = tk.Canvas(side, width=300, height=260, bg="#101010", highlightthickness=0)
        self.slam.pack(pady=4)
        self.slam.bind("<Button-1>", lambda e: self.toggle_absolute())
        self.map = tk.Canvas(side, width=300, height=220, bg="#0a2a40", highlightthickness=0)
        self.map.pack(pady=4)
        self.tape = tk.Canvas(w, height=80, bg="#101010", highlightthickness=0)
        self.tape.grid(row=2, column=0, columnspan=4, sticky="ew", padx=4, pady=4)
        tk.Button(w, text="SAVE", font=F_BIG, bg="#d00000", fg=FG, activebackground="#ff4040",
                  command=lambda: app.video_save("driver")).grid(row=2, column=4, sticky="nsew", padx=4, pady=4)
        self.qbox = tk.Frame(w, bg="#303000", highlightbackground=YEL, highlightthickness=4)
        self.qtext = tk.Label(self.qbox, text="", font=F_MID, bg="#303000", fg=FG, wraplength=700, justify="center")
        self.qtext.pack(padx=20, pady=(16, 6))
        self.qtime = tk.Label(self.qbox, text="", font=F_HUD, bg="#303000", fg=YEL)
        self.qtime.pack()
        qb = tk.Frame(self.qbox, bg="#303000")
        qb.pack(pady=14)
        tk.Button(qb, text="ACCEPT", font=F_MID, bg="#008000", fg=FG, width=9,
                  command=lambda: self.answer(True)).pack(side="left", padx=12)
        tk.Button(qb, text="REJECT", font=F_MID, bg="#a00000", fg=FG, width=9,
                  command=lambda: self.answer(False)).pack(side="left", padx=12)
        for k in ("<F12>", "<Control-Tab>"):
            w.bind(k, lambda e: app.swap_screens())
        w.bind("<Configure>", lambda e: app.layout_changed())
        self.tick()

    def toggle_absolute(self):
        self.absolute = not self.absolute

    def destroy(self):
        if self.question:
            self.answer(False, why="driver screen closed")
        self.w.destroy()

    # --- captain -> driver questions ----------------------------------------------------
    def ask(self, text, on_yes, on_no, timeout_s):
        if self.question:                       # one at a time: the older one is rejected
            self.answer(False, why="replaced by a newer request")
        self.question = (text, on_yes, on_no, time.time() + timeout_s)
        self.qtext.configure(text="CAPTAIN asks:\n" + text)
        self.qbox.place(relx=0.5, rely=0.45, anchor="center")
        self.qbox.lift()
        self.w.bell()

    def answer(self, yes, why=None):
        if not self.question:
            return
        text, on_yes, on_no, _dl = self.question
        self.question = None
        self.qbox.place_forget()
        self.answers.append((text, "accepted" if yes else (why or "rejected")))
        (on_yes if yes else on_no)(why or ("accepted" if yes else "rejected by the driver"))

    # --- drawing --------------------------------------------------------------------------
    def tick(self):
        try:
            self.draw()
        except tk.TclError:
            return
        self.w.after(100, self.tick)

    def draw(self):
        if self.question:
            left = self.question[3] - time.time()
            if left <= 0:
                self.answer(False, why="no answer from the driver (timeout)")
            else:
                self.qtime.configure(text="%.0f s - no answer = REJECTED" % math.ceil(left))
        a = self.app
        d = a.t
        self.draw_video()
        self.draw_status(d)
        if not d:
            return
        self.draw_throttle(d)
        self.speed.configure(text="%.1f kn" % d["est_sog"])
        self.draw_slam(d)
        self.draw_tape(d)
        self.draw_map(d)

    def _video(self, cv, vs, name, w, h, mirror=False):
        cv.delete("all")
        data = vs.ppm() if vs else None
        if data:
            if mirror:
                data = flip_ppm(data)
            self.photos[name] = tk.PhotoImage(data=data, format="ppm")
            cv.create_image(w / 2, h / 2, image=self.photos[name], anchor="center")
        cv.create_text(6, 4, text=name + (" (mirror)" if mirror else ""), anchor="nw", fill=FG, font=F_SM)
        frozen = vs is None or vs.frozen()
        if frozen:
            cv.create_rectangle(w / 2 - 150, h / 2 - 28, w / 2 + 150, h / 2 + 28, fill="#c00000", outline="",
                                tags="frozen")
            cv.create_text(w / 2, h / 2, text="VIDEO FROZEN" if vs else "NO VIDEO", fill=FG, font=F_MID,
                           tags="frozen")
        return frozen

    def draw_video(self):
        a = self.app
        vs = getattr(a, "vstreams", {})
        fw, fh = max(10, self.front.winfo_width()), max(10, self.front.winfo_height())
        f1, f2 = vs.get("FRONT-1"), vs.get("FRONT-2")
        use = a.front_sel.update(f1, f2) if (f1 or f2) else "FRONT-1"
        frozen = self._video(self.front, vs.get(use), use, fw, fh)
        if a.front_sel.notice:
            self.front.create_text(fw / 2, fh - 30, text=a.front_sel.notice, fill=YEL, font=F_HUD)
        d = a.t or {}
        self.front_live = not frozen
        if d.get("mode") == proto.MODE_REMOTE and frozen:
            self.front.create_text(fw / 2, 40, text="NO FRONT CAMERA - DRIVE CAREFULLY / STOP", fill=RED,
                                   font=F_MID, tags="nofront")
        self._video(self.mirror_l, vs.get("REAR-LEFT"), "REAR-LEFT", 400, 225, mirror=True)
        self._video(self.mirror_r, vs.get("REAR-RIGHT"), "REAR-RIGHT", 400, 225, mirror=True)

    def draw_status(self, d):
        c = self.status
        c.delete("all")
        w = max(10, c.winfo_width())
        a = self.app
        if not d:
            c.create_text(w / 2, 40, text="NO DATA FROM THE BOAT", fill=RED, font=F_MID)
            return
        mode = d["mode_name"].replace("_", " ")
        c.create_text(w / 2, 32, text=mode, fill=FG, font=F_BIG, tags="mode")
        al = proto.alarm_texts(d["alarm_bits"])
        y = 72
        if al:
            c.create_text(w / 2, y, text="ALARM: " + ", ".join(al), fill=RED, font=F_HUD, width=w - 20, tags="alarm")
            y += 30
        g = a.gw_status or {}
        n = g.get("net") or {}
        line = []
        if n:
            line.append("STARLINK - EMERGENCY" if n.get("emergency") else (n.get("active") or "NO INTERNET"))
        me = [s for s in g.get("stations", []) if s.get("id") == g.get("you")]
        lat = me[0].get("rtt_ms") if me else None
        if lat is not None:
            line.append("latency %d ms" % lat)
        age = time.time() - a.t_rx if a.t_rx else 99
        line.append("data %.1f s old" % age if age > 0.5 else "data live")
        r = g.get("rec") or {}
        if r.get("enabled"):
            line.append("REC" if any(cm["state"] == "REC" for cm in r.get("cameras", [])) else "NOT RECORDING")
        c.create_text(w / 2, y + 4, text="   ".join(line), fill=YEL if n.get("emergency") else FG, font=F_HUD,
                      tags="line")
        js = []
        if d.get("remote_state"):
            js.append("REMOTE DRIVING %s" % ("LINK LOST" if d["remote_state"] == 2 else "on"))
        js.append("joystick %s" % ("FAULT" if d.get("joy_fault_axis") else ("armed" if d.get("joy_status", 0) & 1 else "-")))
        if d.get("slam_override"):
            js.append("SLAM OVERRIDE")
        if d.get("slam_lost"):
            js.append("slamming sensor lost")
        c.create_text(w / 2, y + 36, text="   ".join(js), fill=FG, font=F_HUD, tags="joy")

    def draw_throttle(self, d):
        c = self.thr
        c.delete("all")
        h = max(50, c.winfo_height()) - 40
        cap = d.get("slam_cap")
        g = throttle_geom(d["throttle"], d["power_fwd"], cap, h)
        x0, x1 = 30, 80
        c.create_rectangle(x0, 20, x1, 20 + h, outline="#606060")
        c.create_rectangle(x0, 20 + g["out"], x1, 20 + h, fill=GRN, outline="", tags="thr")
        if g["cap"] is not None:
            c.create_rectangle(x0, 20 + g["plim"], x1, 20 + g["cap"], fill="#806000", outline="", stipple="gray50",
                               tags="cap")
            c.create_text(55, 20 + g["cap"] + 10, text="SLAM", fill=YEL, font=F_SM)
        c.create_line(x0 - 8, 20 + g["plim"], x1 + 8, 20 + g["plim"], fill=RED, width=3, tags="plim")
        c.create_text(55, 10, text="%.0f %%" % d["throttle"], fill=FG, font=F_SM)
        c.create_text(55, 20 + h + 12, text="limit %.0f" % d["power_fwd"], fill=RED, font=F_SM)

    def draw_slam(self, d):
        c = self.slam
        c.delete("all")
        if "slam_level" not in d:
            c.create_text(150, 130, text="no slamming data", fill="#808080", font=F_SM)
            return
        peaks = [p for _t, p in list(self.app.slam_hist)[-60:]]
        h = 200
        bars, ly, base = slam_geom(peaks, d["slam_level"], d["slam_limit"], h, self.absolute,
                                   d.get("slam_yellow_pc", 70) / 100.0)
        n = 60
        bw = 280.0 / n
        for i, y0, y1, col in bars:
            x = 10 + (n - len(peaks) + i) * bw
            c.create_rectangle(x, 30 + min(y0, y1), x + bw - 1, 30 + max(y0, y1), fill=col, outline="", tags="bar")
        c.create_line(10, 30 + ly, 290, 30 + ly, fill=RED, width=2, dash=(6, 3), tags="limit")
        c.create_line(10, 30 + base, 290, 30 + base, fill="#a0a0a0", tags="base")
        st = d.get("slam_state", "?")
        c.create_text(150, 12, text="SLAMMING %s  avg %.1f g  %d/min" % (st, d["slam_level"], d["slam_rate"]),
                      fill={"RED": RED, "YELLOW": YEL}.get(st, FG), font=F_SM)
        c.create_text(150, 248, text="limit %.1f g - %s (click)" % (
            d["slam_limit"], "absolute" if self.absolute else "around avg"), fill="#c0c0c0", font=F_SM)

    def draw_tape(self, d):
        c = self.tape
        c.delete("all")
        w = max(100, c.winfo_width())
        hd = d["heading"]
        for x, deg, lab in tape_ticks(hd, w):
            c.create_line(x, 50, x, 80 if lab else 70, fill=FG)
            if lab:
                c.create_text(x, 38, text=lab, fill=FG, font=F_HUD)
        c.create_polygon(w / 2 - 12, 0, w / 2 + 12, 0, w / 2, 22, fill=YEL, tags="hdg")
        c.create_text(w / 2 + 60, 12, text="%03.0f°" % hd, fill=YEL, font=F_HUD)
        if d["mode"] in (proto.MODE_HDG_LOCK, proto.MODE_AUTO):
            tx = tape_x(hd, d["target"], w)
            c.create_rectangle(tx - 5, 44, tx + 5, 80, fill="#00c0ff", outline="", tags="target")

    def draw_map(self, d):
        c = self.map
        c.delete("all")
        tr = self.app.track[-600:]
        if not tr:
            return
        la0, lo0 = tr[-1]
        k = math.cos(math.radians(la0)) * 111320.0
        m = 4.0                                         # m per pixel
        P = lambda la, lo: (150 + (lo - lo0) * k / m, 110 - (la - la0) * 111320.0 / m)
        pts = []
        for la, lo in tr[::max(1, len(tr) // 300)]:
            pts += P(la, lo)
        if len(pts) >= 4:
            c.create_line(*pts, fill="#80c0ff", width=2)
        if d.get("wp_set"):
            x, y = P(d["wp_lat"], d["wp_lon"])
            x, y = max(5, min(295, x)), max(5, min(215, y))
            c.create_oval(x - 6, y - 6, x + 6, y + 6, outline=YEL, width=2)
        h = math.radians(d["heading"])
        c.create_polygon(150 + 10 * math.sin(h), 110 - 10 * math.cos(h), 150 + 6 * math.sin(h + 2.4),
                         110 - 6 * math.cos(h + 2.4), 150 + 6 * math.sin(h - 2.4), 110 - 6 * math.cos(h - 2.4),
                         fill=FG)
        c.create_text(6, 210, text="400 m wide", anchor="sw", fill="#a0a0a0", font=F_SM)
