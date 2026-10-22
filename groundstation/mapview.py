"""Chart + AIS layers for the ground-station maps (sections B, D, E2).

ChartView.draw(canvas, lat0, lon0, m_per_px, layers, ...) draws, under the
existing map items (boat, track, program):
  depth shading (paper-chart colours, opacity), depth contours, sea marks,
  AIS targets (red = danger, orange = warn, grey = stale; 6-min course
  line), the avoidance route, the predicted shallow water, and always the
  note "Not for navigation - free data, may be inaccurate near the coast"
  with the data attribution.
Works offline: only the chart files (groundstation/chart.py).
"""
import math
import tkinter as tk

import chart

NOTE = chart.NOTE
COL_AIS = {"danger": "#e00000", "warn": "#ff8c00", None: "#2050c0", "stale": "#909090"}
MARK_COL = {"light": "#d0a000", "buoy_lateral": "#c00000", "beacon_lateral": "#c00000", "harbour": "#004080",
            "buoy_cardinal": "#202020", "beacon_cardinal": "#202020", "buoy_special_purpose": "#c0a000"}


def blend(col, a, bg="#ffffff"):
    """col faded towards bg (a = 1: col, 0: bg)."""
    c1 = [int(col[i:i + 2], 16) for i in (1, 3, 5)]
    c2 = [int(bg[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(int(round(x * a + y * (1 - a))) for x, y in zip(c1, c2))


class ChartView:
    def __init__(self, charts=None):
        self.charts = charts if charts is not None else chart.Charts()
        self.img = None
        self.key = None
        self.img_at = None              # (lat, lon) of the image centre
        self._bboxes = {}

    def projection(self, lat0, lon0, m, w, h):
        k = math.cos(math.radians(lat0)) * 111320.0

        def P(lat, lon):
            return w / 2 + (lon - lon0) * k / m, h / 2 - (lat - lat0) * 111320.0 / m

        def inv(x, y):
            return lat0 + (h / 2 - y) * m / 111320.0, lon0 + (x - w / 2) * m / k
        return P, inv

    # --- depth shading -------------------------------------------------------------------
    def _shade(self, lat0, lon0, m, w, h, opacity, step):
        _, inv = self.projection(lat0, lon0, m, w, h)
        img = tk.PhotoImage(width=(w + step - 1) // step, height=(h + step - 1) // step)
        cache = {}
        rows = []
        for yy in range(0, h, step):
            row = []
            for xx in range(0, w, step):
                la, lo = inv(xx + step / 2, yy + step / 2)
                d = self.charts.depth(la, lo)
                key = None if d is None else int(d * 2) / 2.0
                colr = cache.get(key)
                if colr is None:
                    colr = cache[key] = blend(chart.depth_colour(d), opacity) if d is not None or \
                        self.charts.area_at(la, lo) else blend("#ffffff", opacity)
                row.append(colr)
            rows.append("{" + " ".join(row) + "}")
        img.put(" ".join(rows))
        return img.zoom(step, step) if step > 1 else img

    def draw(self, c, lat0, lon0, m, w, h, layers=None, opacity=0.8, ais=None, threats=None, route=None,
             shallow=None, small=False, vision=None):
        layers = layers or {"depth": True, "contours": True, "marks": True, "ais": True}
        P, _ = self.projection(lat0, lon0, m, w, h)
        if self.charts and layers.get("depth"):
            step = 6 if small else 4
            key = (round(m, 3), w, h, round(opacity, 2), step)
            if self.key != key or self.img_at is None or \
                    abs(P(*self.img_at)[0] - w / 2) > w * 0.2 or abs(P(*self.img_at)[1] - h / 2) > h * 0.2:
                self.img = self._shade(lat0, lon0, m, w, h, opacity, step)
                self.key, self.img_at = key, (lat0, lon0)
            x, y = P(*self.img_at)
            c.create_image(x, y, image=self.img, anchor="center", tags="chart")
        if self.charts and layers.get("contours"):
            self._contours(c, P, m, w, h, small)
        if self.charts and layers.get("marks") and m <= 60:
            for a in self.charts.areas:
                for s in a.seamarks():
                    x, y = P(s["lat"], s["lon"])
                    if 0 <= x <= w and 0 <= y <= h:
                        col = MARK_COL.get(s.get("type"), "#600060")
                        c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=col, outline="", tags="mark")
                        if s.get("name") and m <= 15 and not small:
                            c.create_text(x + 5, y, text=s["name"][:24], anchor="w", fill=col,
                                          font=("Segoe UI", 8), tags="mark")
        if route and len(route) >= 2:
            pts = [v for la, lo in route for v in P(la, lo)]
            c.create_line(*pts, fill="#c000c0", width=3, dash=(8, 4), tags="route")
        if shallow:
            x, y = P(shallow["lat"], shallow["lon"])
            c.create_line(x - 8, y - 8, x + 8, y + 8, fill="#e00000", width=3, tags="shallow")
            c.create_line(x - 8, y + 8, x + 8, y - 8, fill="#e00000", width=3, tags="shallow")
            if not small:
                c.create_text(x + 10, y, anchor="w", fill="#e00000", font=("Segoe UI", 10, "bold"), tags="shallow",
                              text="SHALLOW %s in %.0f s" % ("?" if shallow.get("depth") is None else
                                                              "%.1f m" % shallow["depth"], shallow.get("in_s", 0)))
        if ais and layers.get("ais", True):
            lvl = {t["mmsi"]: t for t in (threats or [])}
            for t in ais:
                if t.get("lat") is None:
                    continue
                x, y = P(t["lat"], t["lon"])
                if not (-50 <= x <= w + 50 and -50 <= y <= h + 50):
                    continue
                th = lvl.get(t["mmsi"], {})
                col = COL_AIS["stale"] if t.get("stale") else COL_AIS.get(th.get("level"))
                cog = math.radians(t.get("cog") or t.get("heading") or 0.0)
                r = 7 if small else 10
                pts = []
                for ang, rr in ((0, r), (140, r * 0.7), (220, r * 0.7)):
                    b = cog + math.radians(ang)
                    pts += [x + rr * math.sin(b), y - rr * math.cos(b)]
                c.create_polygon(*pts, fill=col, outline="#000000", tags="ais")
                if t.get("sog") and t.get("cog") is not None and not t.get("stale"):
                    L = t["sog"] * 1852.0 / 10.0 / m            # 6 minutes ahead
                    c.create_line(x, y, x + L * math.sin(cog), y - L * math.cos(cog), fill=col, dash=(3, 3),
                                  tags="ais")
                if not small or th.get("level"):
                    lab = (t.get("name") or str(t["mmsi"]))[:18]
                    if th.get("level"):
                        lab += "  CPA %.2f nm / %.1f min" % (th["cpa_nm"], th["tcpa_min"])
                    c.create_text(x + r + 3, y - r, text=lab, anchor="w", fill=col,
                                  font=("Segoe UI", 8 if small else 9, "bold" if th.get("level") else "normal"),
                                  tags="ais")
        # camera obstacles (vision): UNCERTAIN - a dashed sector bearing +-4 deg over the rough distance range,
        # drawn from the boat (the map centre); "=AIS name" when it matches an AIS ship
        for o in vision or []:
            if not o.get("confirmed") or o.get("dist") is None:
                continue
            d0 = max(5.0, (o.get("range") or [None])[0] or o["dist"] * 0.7)
            d1 = (o.get("range") or [None, None])[1] or o["dist"] * 2.0
            pts = []
            for dd, bs in ((d0, (-4, 4)), (d1, (4, -4))):
                for db in range(bs[0], bs[1] + (1 if bs[1] > bs[0] else -1), 2 if bs[1] > bs[0] else -2):
                    b = math.radians(o["bearing"] + db)
                    pts += [w / 2 + dd * math.sin(b) / m, h / 2 - dd * math.cos(b) / m]
            col = "#ff6000" if o.get("in_corridor") else "#c08000"
            c.create_polygon(*pts, outline=col, fill=col, stipple="gray25", dash=(4, 3), width=2, tags="vision")
            b = math.radians(o["bearing"])
            x, y = w / 2 + o["dist"] * math.sin(b) / m, h / 2 - o["dist"] * math.cos(b) / m
            lab = "? ~%d m" % o["dist"] + ("  =%s" % o["ais"] if o.get("ais") else "")
            c.create_text(x + 6, y, text=lab, anchor="w", fill=col, font=("Segoe UI", 8 if small else 9, "bold"),
                          tags="vision")
        # the note - always
        c.create_rectangle(0, h - (16 if small else 34), w, h, fill="#fff8d0", outline="", tags="note")
        c.create_text(4, h - (8 if small else 25), anchor="w", text=NOTE, fill="#a00000",
                      font=("Segoe UI", 7 if small else 9, "bold"), tags="note")
        if not small:
            att = " | ".join(a.split(". NOT FOR")[0] for a in self.charts.attribution()) if self.charts else \
                "no chart files (tools/chartprep.py)"
            c.create_text(4, h - 9, anchor="w", text=att[:200], fill="#606060", font=("Segoe UI", 7), tags="note")

    def _contours(self, c, P, m, w, h, small):
        levels = (2, 5, 10, 20, 50, 100) if m < 30 else (5, 20, 50, 100)
        x0, y0 = P(0, 0)
        for a in self.charts.areas:
            cont = a.contours()
            for lv in levels:
                lines = cont.get(str(lv), [])
                key = (a.folder, lv)
                if key not in self._bboxes:
                    self._bboxes[key] = [(min(p[0] for p in ln), max(p[0] for p in ln), min(p[1] for p in ln),
                                          max(p[1] for p in ln)) for ln in lines]
                col = "#3070b0" if lv <= 5 else "#7fa8d0"
                for ln, (la0, la1, lo0, lo1) in zip(lines, self._bboxes[key]):
                    xa, ya = P(la1, lo0)
                    xb, yb = P(la0, lo1)
                    if xb < 0 or xa > w or yb < 0 or ya > h:
                        continue
                    stride = max(1, int(m / 40)) if not small else max(1, int(m / 20))
                    pts = [v for p in ln[::stride] + [ln[-1]] for v in P(p[0], p[1])]
                    if len(pts) >= 4:
                        c.create_line(*pts, fill=col, width=1, tags="contour")
