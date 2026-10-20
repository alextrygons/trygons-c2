"""Log viewer for .TLG files (the boat's SD card logs and the ground
station's own copy).

    python logview.py                  choose a file
    python logview.py B00012_001.TLG

  - plots over time: heading vs target, outboard angle, throttle (asked /
    sent / power limit), speed; alarms as red lines, mode changes as grey
  - map: the track, the dead-reckoning uncertainty at the cursor, waypoint
  - events: click one to jump to that moment (cursor in plots and map)
  - click / drag in the plots to move the cursor; the values at the cursor
    are shown at the bottom
"""
import bisect
import math
import os
import sys
import tkinter as tk
from tkinter import filedialog, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tlg  # noqa: E402

BG = "#ffffff"
F_TXT = ("Segoe UI", 11)
F_MED = ("Segoe UI", 12, "bold")

# field names in the boat log, then in the ground-station telemetry copy
ALIASES = {
    "heading": ["heading"], "target": ["target"], "outboard": ["outboard_est", "outboard"],
    "thr_out": ["thr_out", "throttle"], "thr_ask": ["thr_mode", "throttle_demand"],
    "power": ["power_fwd"], "sog": ["gps_sog", "est_sog"], "est_sog": ["est_sog"],
    "lat": ["est_lat"], "lon": ["est_lon"], "unc": ["unc", "unc_m"], "mode": ["mode"],
    "alarm_bits": ["alarm_bits"],
}

PLOTS = [
    ("Heading (black) / target (blue), deg", [("heading", "#000000"), ("target", "#0033cc")], "wrap"),
    ("Outboard angle, deg", [("outboard", "#ff8c00")], None),
    ("Throttle: sent (black) / asked (orange) / power limit (red), %", [("thr_out", "#000000"),
                                                                        ("thr_ask", "#ff8c00"),
                                                                        ("power", "#d00000")], None),
    ("Speed: GPS (black) / estimate (green), kn", [("sog", "#000000"), ("est_sog", "#007a00")], None),
]


class Viewer:
    def __init__(self, root, path=None):
        self.root = root
        self.log = None
        self.cursor = 0
        root.title("Trygons C2 log viewer")
        root.geometry("1400x880")
        root.configure(bg=BG)
        top = tk.Frame(root, bg=BG)
        top.pack(fill="x")
        tk.Button(top, text="Open log...", font=F_MED, command=self.open).pack(side="left", padx=4, pady=4)
        self.title = tk.Label(top, text="", font=F_MED, bg=BG)
        self.title.pack(side="left", padx=8)
        body = tk.PanedWindow(root, orient="horizontal")
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=BG)
        body.add(left, width=430)
        tk.Label(left, text="Events (click = jump there)", font=F_MED, bg=BG).pack(anchor="w")
        self.ev = tk.Listbox(left, font=("Consolas", 10))
        self.ev.pack(fill="both", expand=True)
        self.ev.bind("<<ListboxSelect>>", self.ev_click)
        right = tk.PanedWindow(body, orient="vertical")
        body.add(right)
        self.plot = tk.Canvas(right, bg=BG, highlightthickness=0)
        right.add(self.plot, height=520)
        self.map = tk.Canvas(right, bg="#d8ecff", highlightthickness=0)
        right.add(self.map)
        self.plot.bind("<Button-1>", self.plot_click)
        self.plot.bind("<B1-Motion>", self.plot_click)
        self.plot.bind("<Configure>", lambda e: self.draw())
        self.map.bind("<Configure>", lambda e: self.draw())
        self.info = tk.Label(root, text="", font=("Consolas", 11), bg="#202020", fg="#ffffff", anchor="w",
                             justify="left")
        self.info.pack(fill="x")
        if path:
            self.load(path)

    # --- data ---------------------------------------------------------------------
    def open(self):
        p = filedialog.askopenfilename(filetypes=[("Trygons log", "*.TLG"), ("all", "*.*")])
        if p:
            self.load(p)

    def load(self, path):
        self.log = tlg.read(path)
        s = self.log.samples
        self.t = [x["t_ms"] / 1000.0 for x in s]
        names = set(s[0].keys()) if s else set()
        self.col = {}
        for k, alts in ALIASES.items():
            n = next((a for a in alts if a in names), None)
            self.col[k] = [x[n] for x in s] if n else None
        self.title.configure(text="%s   %d samples, %.0f s, %d events%s" % (
            os.path.basename(path), len(s), (self.t[-1] - self.t[0]) if s else 0, len(self.log.events),
            ", %d CRC errors" % self.log.crc_errors if self.log.crc_errors else ""))
        self.ev.delete(0, "end")
        self.events = sorted(self.log.events + [(t, "PC %s -> %s" % (tlg.proto.TYPE_NAMES.get(ft, hex(ft)),
                                                                     tlg.proto.RESULTS.get(r, r)), -1, [])
                                                for t, ft, _sq, r, _p in self.log.pc_commands],
                             key=lambda e: e[0])
        for t, text, _c, _a in self.events:
            self.ev.insert("end", "%8.1f  %s" % (t / 1000.0, text))
        self.cursor = 0
        self.draw()

    def at(self, key, i):
        c = self.col.get(key)
        return c[i] if c else None

    # --- interaction --------------------------------------------------------------
    def ev_click(self, _e):
        sel = self.ev.curselection()
        if not sel or not self.t:
            return
        t = self.events[sel[0]][0] / 1000.0
        self.cursor = min(max(bisect.bisect_left(self.t, t), 0), len(self.t) - 1)
        self.draw()

    def plot_click(self, e):
        if not self.t:
            return
        w = self.plot.winfo_width()
        x0, x1 = 70, w - 20
        t = self.t[0] + (e.x - x0) / max(1, x1 - x0) * (self.t[-1] - self.t[0])
        self.cursor = min(max(bisect.bisect_left(self.t, t), 0), len(self.t) - 1)
        self.draw()

    # --- drawing ------------------------------------------------------------------
    def draw(self):
        self.draw_plots()
        self.draw_map()
        self.draw_info()

    def draw_plots(self):
        c = self.plot
        c.delete("all")
        if not self.log or not self.t:
            c.create_text(20, 20, text="Open a .TLG log", anchor="nw", font=F_MED)
            return
        w, h = c.winfo_width(), c.winfo_height()
        x0, x1 = 70, w - 20
        t0, t1 = self.t[0], max(self.t[-1], self.t[0] + 1)
        X = lambda t: x0 + (t - t0) / (t1 - t0) * (x1 - x0)
        n = len(PLOTS)
        ph = (h - 10) / n
        step = max(1, len(self.t) // max(1, (x1 - x0)))      # about one point per pixel
        for k, (title, series, kind) in enumerate(PLOTS):
            yt = 5 + k * ph
            c.create_text(x0, yt, text=title, anchor="nw", font=F_TXT)
            vals = [v for key, _col in series if self.col.get(key) for v in self.col[key][::step]]
            if not vals:
                continue
            lo, hi = min(vals), max(vals)
            if kind == "wrap":
                lo, hi = 0.0, 360.0
            if hi - lo < 1e-6:
                hi = lo + 1.0
            Y = lambda v: yt + ph - 8 - (v - lo) / (hi - lo) * (ph - 30)
            c.create_text(x0 - 4, Y(hi), text="%.0f" % hi, anchor="e", font=F_TXT)
            c.create_text(x0 - 4, Y(lo), text="%.0f" % lo, anchor="e", font=F_TXT)
            c.create_line(x0, yt + ph - 4, x1, yt + ph - 4, fill="#d0d0d0")
            for key, col in series:
                data = self.col.get(key)
                if not data:
                    continue
                pts = []
                prev = None
                for i in range(0, len(self.t), step):
                    v = data[i]
                    if kind == "wrap" and prev is not None and abs(v - prev) > 180:
                        if len(pts) >= 4:
                            c.create_line(*pts, fill=col, width=2)
                        pts = []
                    pts += [X(self.t[i]), Y(v)]
                    prev = v
                if len(pts) >= 4:
                    c.create_line(*pts, fill=col, width=2)
        # mode changes (grey) and alarms (red) over all plots
        for t, text, code, _a in self.events:
            x = X(t / 1000.0)
            if text.startswith("ALARM_ON"):
                c.create_line(x, 0, x, h, fill="#ff6060", dash=(3, 3))
            elif text.startswith("MODE") and "refused" not in text:
                c.create_line(x, 0, x, h, fill="#b0b0b0", dash=(2, 4))
        xc = X(self.t[self.cursor])
        c.create_line(xc, 0, xc, h, fill="#0060ff", width=2)
        c.create_text(x1, h - 2, text="time s (%.0f .. %.0f)" % (t0, t1), anchor="se", font=F_TXT)

    def draw_map(self):
        c = self.map
        c.delete("all")
        lat, lon = self.col.get("lat"), self.col.get("lon")
        if not self.t or not lat:
            return
        pts = [(a, b) for a, b in zip(lat, lon) if a or b]
        if not pts:
            return
        w, h = c.winfo_width(), c.winfo_height()
        la0 = sum(p[0] for p in pts) / len(pts)
        k = math.cos(math.radians(la0)) * 111320.0
        ns = [(p[0] - la0) * 111320.0 for p in pts]
        es = [p[1] * k for p in pts]
        e0 = (min(es) + max(es)) / 2
        span = max(max(ns) - min(ns), max(es) - min(es), 50.0) * 1.15
        m = span / max(50, min(w, h) - 40)
        P = lambda la, lo: (w / 2 + (lo * k - e0) / m, h / 2 - ((la - la0) * 111320.0 - (min(ns) + max(ns)) / 2) / m)
        step = max(1, len(pts) // 3000)
        line = []
        for a, b in pts[::step]:
            line += P(a, b)
        if len(line) >= 4:
            c.create_line(*line, fill="#404040", width=2)
        i = self.cursor
        if lat[i] or lon[i]:
            x, y = P(lat[i], lon[i])
            r = (self.at("unc", i) or 0) / m
            c.create_oval(x - r, y - r, x + r, y + r, outline="#d00000", dash=(4, 3), width=2)
            c.create_oval(x - 6, y - 6, x + 6, y + 6, fill="#0060ff", outline="#000000")
        sc = 10 ** math.floor(math.log10(max(1.0, m * 150)))
        c.create_line(20, h - 20, 20 + sc / m, h - 20, width=4)
        c.create_text(20, h - 26, text="%g m" % sc, anchor="sw", font=F_TXT)

    def draw_info(self):
        if not self.t:
            self.info.configure(text="")
            return
        i = self.cursor
        s = self.log.samples[i]
        mode = self.log.mode_name(int(s.get("mode", 0)))
        al = self.log.alarm_text(int(s.get("alarm_bits", 0)))
        keys = [k for k in ("heading", "target", "outboard_est", "outboard", "thr_out", "throttle", "power_fwd",
                            "gps_sog", "est_sog", "unc", "unc_m", "xte", "joy_x", "joy_y") if k in s]
        vals = "  ".join("%s %.2f" % (k, s[k]) for k in keys)
        self.info.configure(text="t %.2f s   mode %s   alarms %s\n%s" % (self.t[i], mode, al, vals))


def main():
    root = tk.Tk()
    Viewer(root, sys.argv[1] if len(sys.argv) > 1 else None)
    root.mainloop()


if __name__ == "__main__":
    main()
