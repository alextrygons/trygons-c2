"""Chart + AIS layers for the ground-station maps (sections B, D, E2; themes + rotation: HUD A8, map window C).

ChartView.draw(canvas, lat0, lon0, m_per_px, layers, ...) draws, under the
existing map items (boat, track, program):
  depth shading (paper-chart colours, opacity), depth contours, sea marks,
  AIS targets (red = danger, orange = warn, grey = stale; 6-min course
  line), the avoidance route, the predicted shallow water, and always the
  note "Not for navigation - free data, may be inaccurate near the coast"
  with the data attribution.
theme: "day" (paper chart, the captain screen), "dark" (the driver's mini map, the
map window: never bright white) or "night" (dim red). rot: course-up - the map
turned so that this direction (deg) points up (0 = north up).
Works offline: only the chart files (groundstation/chart.py).
"""
import math
import tkinter as tk

import chart
from i18n import T

try:
    from PIL import Image, ImageDraw, ImageTk
except ImportError:                 # without Pillow: shading as tk.PhotoImage, contours as canvas lines
    Image = None

NOTE = chart.NOTE
COL_AIS = {"danger": "#e00000", "warn": "#ff8c00", None: "#2050c0", "stale": "#909090"}
MARK_COL = {"light": "#d0a000", "buoy_lateral": "#c00000", "beacon_lateral": "#c00000", "harbour": "#004080",
            "buoy_cardinal": "#202020", "beacon_cardinal": "#202020", "buoy_special_purpose": "#c0a000"}

THEMES = {
    "day": {"bg": "#d8ecff", "blend": "#ffffff", "bands": chart.BANDS, "nodata": "#a0a0a0", "outside": "#ffffff",
            "note_bg": "#fff8d0", "note_fg": "#a00000", "att": "#606060", "contour": ("#3070b0", "#7fa8d0"),
            "avoid": "#c000c0", "route": "#7a00a0", "track": "#404040", "text": "#000000", "own": "#ffffff",
            "warn": "#e00000", "good": "#008000", "ais": COL_AIS, "mark_dark": "#202020", "vision": ("#ff6000",
                                                                                                  "#c08000")},
    "dark": {"bg": "#05101a", "blend": "#000000",
             "bands": [(0.0, "#3a3222"), (2.0, "#2a6f93"), (3.0, "#235f80"), (5.0, "#1d506d"), (10.0, "#17425b"),
                       (20.0, "#12364b"), (50.0, "#0e2b3c"), (100.0, "#0a2130"), (1e9, "#071824")],
             "nodata": "#262626", "outside": "#0a0f14", "note_bg": "#1c1400", "note_fg": "#ffb000", "att": "#8090a0",
             "contour": ("#6ab0e0", "#2c5272"), "avoid": "#ff50ff", "route": "#ff70ff", "track": "#80c8ff",
             "text": "#c8d8e8", "own": "#3cff6a", "warn": "#ff3030", "good": "#3cff6a",
             "ais": {"danger": "#ff3030", "warn": "#ffb000", None: "#6aa8ff", "stale": "#707070"},
             "mark_dark": "#c0c0c0", "vision": ("#ff7020", "#d09000")},
    "night": {"bg": "#000000", "blend": "#000000",
              "bands": [(0.0, "#2a1408"), (2.0, "#401810"), (3.0, "#36140c"), (5.0, "#2e110a"), (10.0, "#270e08"),
                        (20.0, "#200b06"), (50.0, "#1a0905"), (100.0, "#140704"), (1e9, "#0f0503")],
              "nodata": "#180808", "outside": "#050202", "note_bg": "#140500", "note_fg": "#a04020", "att": "#602818",
              "contour": ("#803020", "#401810"), "avoid": "#a03060", "route": "#a04060", "track": "#a04030",
              "text": "#b04030", "own": "#d84a30", "warn": "#ff2a10", "good": "#a04020",
              "ais": {"danger": "#ff2a10", "warn": "#c08000", None: "#904030", "stale": "#502820"},
              "mark_dark": "#803020", "vision": ("#c04010", "#905000")},
}


def blend(col, a, bg="#ffffff"):
    """col faded towards bg (a = 1: col, 0: bg)."""
    c1 = [int(col[i:i + 2], 16) for i in (1, 3, 5)]
    c2 = [int(bg[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(int(round(x * a + y * (1 - a))) for x, y in zip(c1, c2))


def depth_colour(d, theme="day"):
    th = THEMES.get(theme, THEMES["day"])
    if theme == "day":
        return chart.depth_colour(d)
    if d is None:
        return th["nodata"]
    bands = th["bands"]
    if d <= 0.0:
        return bands[0][1]
    for lim, col in bands[1:]:
        if d < lim:
            return col
    return bands[-1][1]


class ChartView:
    def __init__(self, charts=None):
        self.charts = charts if charts is not None else chart.Charts()
        self.img = None
        self.key = None
        self.img_at = None              # (lat, lon) of the image centre
        self._bboxes = {}

    @staticmethod
    def projection(lat0, lon0, m, w, h, rot=0.0):
        """P(lat, lon) -> (x, y) and back; rot = the direction that points up (course-up)."""
        k = math.cos(math.radians(lat0)) * 111320.0
        cr, sr = math.cos(math.radians(rot)), math.sin(math.radians(rot))

        def P(lat, lon):
            e, n = (lon - lon0) * k / m, (lat - lat0) * 111320.0 / m
            return w / 2 + e * cr - n * sr, h / 2 - (e * sr + n * cr)

        def inv(x, y):
            xr, yr = x - w / 2, h / 2 - y
            e, n = xr * cr + yr * sr, -xr * sr + yr * cr
            return lat0 + n * m / 111320.0, lon0 + e * m / k
        return P, inv

    # --- depth shading -------------------------------------------------------------------
    def _shade(self, lat0, lon0, m, w, h, opacity, step, theme="day", rot=0.0):
        _, inv = self.projection(lat0, lon0, m, w, h, rot)
        th = THEMES.get(theme, THEMES["day"])
        img = tk.PhotoImage(width=(w + step - 1) // step, height=(h + step - 1) // step)
        cache = {}
        rows = []
        for yy in range(0, h, step):
            row = []
            for xx in range(0, w, step):
                la, lo = inv(xx + step / 2, yy + step / 2)
                d = self.charts.depth(la, lo)
                key = None if d is None else int(d * 2) / 2.0
                colr = cache.get(key) if d is not None else None
                if colr is None:
                    if d is not None:
                        colr = cache[key] = blend(depth_colour(d, theme), opacity, th["blend"])
                    elif self.charts.area_at(la, lo):
                        colr = blend(depth_colour(None, theme), opacity, th["blend"])
                    else:
                        colr = blend(th["outside"], opacity, th["blend"])
                row.append(colr)
            rows.append("{" + " ".join(row) + "}")
        img.put(" ".join(rows))
        return img.zoom(step, step) if step > 1 else img

    def _bitmap(self, lat0, lon0, m, w, h, opacity, step, theme, rot, depth_on, contours_on, small):
        """Depth shading + contours as one Pillow picture (-> ImageTk) for this view."""
        P, inv = self.projection(lat0, lon0, m, w, h, rot)
        th = THEMES.get(theme, THEMES["day"])
        cw, ch = (w + step - 1) // step, (h + step - 1) // step
        if depth_on:
            cache, data = {}, []
            rgb = lambda col: (int(col[1:3], 16), int(col[3:5], 16), int(col[5:7], 16), 255)    # noqa: E731
            for yy in range(0, ch * step, step):
                for xx in range(0, cw * step, step):
                    la, lo = inv(xx + step / 2, yy + step / 2)
                    d = self.charts.depth(la, lo)
                    k = None if d is None else int(d * 2) / 2.0
                    colr = cache.get(k) if d is not None else None
                    if colr is None:
                        if d is not None:
                            colr = cache[k] = rgb(blend(depth_colour(d, theme), opacity, th["blend"]))
                        elif self.charts.area_at(la, lo):
                            colr = rgb(blend(depth_colour(None, theme), opacity, th["blend"]))
                        else:
                            colr = rgb(blend(th["outside"], opacity, th["blend"]))
                    data.append(colr)
            im = Image.new("RGBA", (cw, ch))
            im.putdata(data)
            im = im.resize((cw * step, ch * step), Image.NEAREST).crop((0, 0, w, h))
        else:
            im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        if contours_on:
            dr = ImageDraw.Draw(im)
            corners = [inv(x, y) for x, y in ((0, 0), (w, 0), (0, h), (w, h))]
            view = (min(p[0] for p in corners), max(p[0] for p in corners), min(p[1] for p in corners),
                    max(p[1] for p in corners))
            levels = (2, 5, 10, 20, 50, 100) if m < 30 else (5, 20, 50, 100)
            for a in self.charts.areas:
                cont = a.contours()
                for lv in levels:
                    lines = cont.get(str(lv), [])
                    bkey = (a.folder, lv)
                    if bkey not in self._bboxes:
                        self._bboxes[bkey] = [(min(p[0] for p in ln), max(p[0] for p in ln), min(p[1] for p in ln),
                                               max(p[1] for p in ln)) for ln in lines]
                    col = th["contour"][0] if lv <= 5 else th["contour"][1]
                    for ln, (la0, la1, lo0, lo1) in zip(lines, self._bboxes[bkey]):
                        if la1 < view[0] or la0 > view[1] or lo1 < view[2] or lo0 > view[3]:
                            continue
                        stride = max(1, int(m / 40)) if not small else max(1, int(m / 20))
                        pts = [P(p[0], p[1]) for p in ln[::stride] + [ln[-1]]]
                        if len(pts) >= 2:
                            dr.line(pts, fill=col, width=1)
        return ImageTk.PhotoImage(im)

    def draw(self, c, lat0, lon0, m, w, h, layers=None, opacity=0.8, ais=None, threats=None, route=None,
             shallow=None, small=False, vision=None, theme="day", rot=0.0):
        layers = layers or {"depth": True, "contours": True, "marks": True, "ais": True}
        th = THEMES.get(theme, THEMES["day"])
        P, inv = self.projection(lat0, lon0, m, w, h, rot)
        baked = False
        if self.charts and (layers.get("depth") or (layers.get("contours") and Image is not None)):
            # one bitmap per view (depth shading + the contours drawn into it with Pillow): every redraw only
            # places it; it is made again when the scale / size / theme / layers change or the boat moved 20 %
            step = 6 if small else 4
            key = (round(m, 3), w, h, round(opacity, 2), step, theme, round(rot, 1), bool(layers.get("depth")),
                   bool(layers.get("contours")) and Image is not None, small)
            if self.key != key or self.img_at is None or \
                    abs(P(*self.img_at)[0] - w / 2) > w * 0.2 or abs(P(*self.img_at)[1] - h / 2) > h * 0.2:
                if Image is not None:
                    self.img = self._bitmap(lat0, lon0, m, w, h, opacity, step, theme, rot, layers.get("depth"),
                                            layers.get("contours"), small)
                else:
                    self.img = self._shade(lat0, lon0, m, w, h, opacity, step, theme, rot)
                self.key, self.img_at = key, (lat0, lon0)
            x, y = P(*self.img_at)
            baked = bool(layers.get("contours")) and Image is not None
            c.create_image(x, y, image=self.img, anchor="center", tags=("chart", "contour") if baked else "chart")
        if self.charts and layers.get("contours") and not baked:
            corners = [inv(x, y) for x, y in ((0, 0), (w, 0), (0, h), (w, h))]
            view = (min(p[0] for p in corners), max(p[0] for p in corners), min(p[1] for p in corners),
                    max(p[1] for p in corners))
            self._contours(c, P, m, w, h, small, th, view)
        if self.charts and layers.get("marks") and m <= 60:
            for a in self.charts.areas:
                for s in a.seamarks():
                    x, y = P(s["lat"], s["lon"])
                    if 0 <= x <= w and 0 <= y <= h:
                        col = MARK_COL.get(s.get("type"), "#600060")
                        if theme != "day" and col == "#202020":
                            col = th["mark_dark"]
                        c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=col, outline="", tags="mark")
                        if s.get("name") and m <= 15 and not small:
                            c.create_text(x + 5, y, text=s["name"][:24], anchor="w", fill=col,
                                          font=("Segoe UI", 8), tags="mark")
        if route and len(route) >= 2:
            pts = [v for la, lo in route for v in P(la, lo)]
            c.create_line(*pts, fill=th["avoid"], width=3, dash=(8, 4), tags="route")
        if shallow:
            x, y = P(shallow["lat"], shallow["lon"])
            c.create_line(x - 8, y - 8, x + 8, y + 8, fill=th["warn"], width=3, tags="shallow")
            c.create_line(x - 8, y + 8, x + 8, y - 8, fill=th["warn"], width=3, tags="shallow")
            if not small:
                c.create_text(x + 10, y, anchor="w", fill=th["warn"], font=("Segoe UI", 10, "bold"), tags="shallow",
                              text=T("map.shallow_mark", depth="?" if shallow.get("depth") is None else
                                     "%.1f m" % shallow["depth"], s="%.0f" % shallow.get("in_s", 0)))
        if ais and layers.get("ais", True):
            lvl = {t["mmsi"]: t for t in (threats or [])}
            cols = th["ais"]
            for t in ais:
                if t.get("lat") is None:
                    continue
                x, y = P(t["lat"], t["lon"])
                if not (-50 <= x <= w + 50 and -50 <= y <= h + 50):
                    continue
                tt = lvl.get(t["mmsi"], {})
                col = cols["stale"] if t.get("stale") else cols.get(tt.get("level"))
                cog = math.radians((t.get("cog") or t.get("heading") or 0.0) - rot)
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
                if not small or tt.get("level"):
                    lab = (t.get("name") or str(t["mmsi"]))[:18]
                    if tt.get("level"):
                        lab += "  " + T("map.ais_cpa", cpa="%.2f" % tt["cpa_nm"], tcpa="%.1f" % tt["tcpa_min"])
                    c.create_text(x + r + 3, y - r, text=lab, anchor="w", fill=col,
                                  font=("Segoe UI", 8 if small else 9, "bold" if tt.get("level") else "normal"),
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
                    b = math.radians(o["bearing"] + db - rot)
                    pts += [w / 2 + dd * math.sin(b) / m, h / 2 - dd * math.cos(b) / m]
            col = th["vision"][0] if o.get("in_corridor") else th["vision"][1]
            c.create_polygon(*pts, outline=col, fill=col, stipple="gray25", dash=(4, 3), width=2, tags="vision")
            b = math.radians(o["bearing"] - rot)
            x, y = w / 2 + o["dist"] * math.sin(b) / m, h / 2 - o["dist"] * math.cos(b) / m
            lab = "? ~%d m" % o["dist"] + ("  =%s" % o["ais"] if o.get("ais") else "")
            c.create_text(x + 6, y, text=lab, anchor="w", fill=col, font=("Segoe UI", 8 if small else 9, "bold"),
                          tags="vision")
        # the note - always
        c.create_rectangle(0, h - (16 if small else 34), w, h, fill=th["note_bg"], outline="", tags="note")
        c.create_text(4, h - (8 if small else 25), anchor="w", text=T("map.note"), fill=th["note_fg"],
                      font=("Segoe UI", 7 if small else 9, "bold"), tags="note")
        if not small:
            att = " | ".join(a.split(". NOT FOR")[0] for a in self.charts.attribution()) if self.charts else \
                T("map.no_chart_files")
            c.create_text(4, h - 9, anchor="w", text=att[:200], fill=th["att"], font=("Segoe UI", 7), tags="note")

    def _contours(self, c, P, m, w, h, small, th=None, view=None):
        th = th or THEMES["day"]
        levels = (2, 5, 10, 20, 50, 100) if m < 30 else (5, 20, 50, 100)
        for a in self.charts.areas:
            cont = a.contours()
            for lv in levels:
                lines = cont.get(str(lv), [])
                key = (a.folder, lv)
                if key not in self._bboxes:
                    self._bboxes[key] = [(min(p[0] for p in ln), max(p[0] for p in ln), min(p[1] for p in ln),
                                          max(p[1] for p in ln)) for ln in lines]
                col = th["contour"][0] if lv <= 5 else th["contour"][1]
                for ln, (la0, la1, lo0, lo1) in zip(lines, self._bboxes[key]):
                    if view and (la1 < view[0] or la0 > view[1] or lo1 < view[2] or lo0 > view[3]):
                        continue                    # outside the window (lat / lon box: no projection needed)
                    stride = max(1, int(m / 40)) if not small else max(1, int(m / 20))
                    pts = [v for p in ln[::stride] + [ln[-1]] for v in P(p[0], p[1])]
                    if len(pts) >= 4:
                        c.create_line(*pts, fill=col, width=1, tags="contour")
