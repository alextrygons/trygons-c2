"""The driver HUD (section A): layout, colours and every overlay as a DISPLAY LIST.

DISPLAY ONLY: nothing in here sends anything to the boat. The driver window
(driver.py) renders the list over the front camera picture (PIL: real
transparency + outlined text; without PIL: plain canvas items).

    hud = Hud(layout)                  layout = Layout() (hud_layout.json)
    ops, hits = hud.build(W, H, s)     s = Snapshot (all values, already worked out)

ops  = [{"k": "panel" | "rect" | "line" | "poly" | "oval" | "text" | "image", "tag": ..., ...}]
hits = [((x0, y0, x1, y1), action)]  clickable places (lamps, SAVE, toolbar)

Every overlay has a rectangle in fractions of the window (Layout), can be hidden,
moved and resized in the edit-layout mode, and belongs to the declutter levels
FULL / NORMAL / MINIMAL. The drawing maths are plain functions (test_hud.py).
"""
import json
import math
import os

import i18n
import theme
from i18n import T

HERE = os.path.dirname(os.path.abspath(__file__))

# --- overlays -------------------------------------------------------------------------------------
# name: (x, y, w, h) in fractions of the window. Centre of the picture (0.25..0.75 x 0.22..0.75) stays free.
DEFAULT_LAYOUT = {
    "mirror_l": (0.008, 0.010, 0.215, 0.215),
    "mirror_r": (0.777, 0.010, 0.215, 0.215),
    "status": (0.250, 0.010, 0.420, 0.140),
    "master": (0.676, 0.010, 0.095, 0.140),
    "alarms": (0.250, 0.158, 0.520, 0.052),
    "notices": (0.250, 0.214, 0.520, 0.040),
    "throttle": (0.008, 0.235, 0.080, 0.600),
    "speed": (0.876, 0.235, 0.116, 0.175),
    "depth": (0.876, 0.415, 0.116, 0.055),
    "waypoint": (0.876, 0.475, 0.116, 0.120),
    "traffic": (0.876, 0.600, 0.116, 0.095),
    "rot": (0.876, 0.700, 0.056, 0.110),
    "gmeter": (0.936, 0.700, 0.056, 0.110),
    "minimap": (0.655, 0.700, 0.215, 0.240),
    "steer": (0.395, 0.775, 0.210, 0.070),
    "cdi": (0.395, 0.850, 0.210, 0.080),
    "controls": (0.008, 0.888, 0.380, 0.052),
    "drive": (0.095, 0.838, 0.293, 0.045),
    "bottom": (0.008, 0.948, 0.984, 0.045),
}
OVERLAYS = list(DEFAULT_LAYOUT)
# the logo (theme.py): fixed in the free corner above the bottom bar, right of the mini map, below the
# ROT / G meters - never over an instrument, a warning or the picture's centre
LOGO_BOX = (0.876, 0.815, 0.116, 0.125)
MINIMAL = {"status", "master", "alarms", "speed", "throttle", "bottom", "controls", "drive"}   # mode, speed, gear,
NORMAL_HIDES = {"rot", "gmeter", "cdi"}                                                    # heading, alarms
DECLUTTER = ("FULL", "NORMAL", "MINIMAL")
ALWAYS = {"controls", "drive"}     # never hidden by declutter: the way back to FULL; RELEASE of manual driving


def declutter_visible(name, level):
    if level == "MINIMAL":
        return name in MINIMAL
    if level == "NORMAL":
        return name not in NORMAL_HIDES
    return True


class Layout:
    """Overlay rectangles + shown / hidden, saved in hud_layout.json (per PC, not in git)."""

    def __init__(self, path=None):
        self.path = path or os.environ.get("TRYGONS_HUD_LAYOUT") or os.path.join(HERE, "hud_layout.json")
        self.rects = dict(DEFAULT_LAYOUT)
        self.hidden = set()
        try:
            with open(self.path, encoding="utf-8") as f:
                d = json.load(f)
            for k, r in (d.get("rects") or {}).items():
                if k in self.rects and len(r) == 4:
                    self.rects[k] = tuple(float(v) for v in r)
            self.hidden = {k for k in d.get("hidden", []) if k in self.rects}
        except (OSError, ValueError, TypeError):
            pass

    def save(self):
        try:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump({"rects": self.rects, "hidden": sorted(self.hidden)}, f, indent=1)
        except OSError:
            pass

    def reset(self):
        self.rects = dict(DEFAULT_LAYOUT)
        self.hidden = set()
        self.save()

    def px(self, name, W, H):
        x, y, w, h = self.rects[name]
        return (x * W, y * H, (x + w) * W, (y + h) * H)

    def move(self, name, dx, dy):
        x, y, w, h = self.rects[name]
        self.rects[name] = (min(max(0.0, x + dx), 1.0 - w), min(max(0.0, y + dy), 1.0 - h), w, h)

    def resize(self, name, dw, dh):
        x, y, w, h = self.rects[name]
        self.rects[name] = (x, y, min(max(0.03, w + dw), 1.0 - x), min(max(0.025, h + dh), 1.0 - y))

    def toggle(self, name):
        self.hidden ^= {name}


# --- colours ------------------------------------------------------------------------------------------
HUD_COLOURS = theme.HUD_MAIN


def palette(colour="green", night=False):
    """One main HUD colour; red and amber only for warnings / cautions (theme.py: the same meaning colours as
    screen 2). Night: dim red, darker panels. Light theme: light panels on the video (text outlined)."""
    if night:
        return dict(theme.NIGHT, panel=theme.BLACK, alpha=int(255 * theme.ALPHA["hud_panel_night"]),
                    outline=theme.BLACK, video=0.42, map_theme="night")
    main = HUD_COLOURS.get(colour, HUD_COLOURS["green"])
    light = theme.name() == "light"
    return {"main": main, "dim": theme.HUD_LIGHT["dim"] if light else theme.HUD_DIM.get(colour, theme.HUD_DIM["green"]),
            "warn": theme.WARN, "caution": theme.CAUTION, "good": theme.GOOD, "yellow": theme.HUD_YELLOW,
            "panel": theme.HUD_LIGHT["panel"] if light else theme.BLACK, "alpha": int(255 * theme.ALPHA["hud_panel"]),
            "outline": theme.BLACK, "video": 1.0, "grey": theme.HUD_LIGHT["grey"] if light else theme.HUD_GREY,
            "map_theme": theme.map_theme()}


def latency_colour(ms, pal):
    """Green / yellow / red by the station's round trip."""
    if ms is None:
        return pal["grey"]
    return pal["good"] if ms < 250 else pal["yellow"] if ms < 600 else pal["warn"]


# --- maths (no tk, no PIL) -------------------------------------------------------------------------------
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
    """x of the target on the tape, clamped to the edge if outside."""
    x = width / 2 + angdiff(target, heading) / span * width
    return max(0.0, min(float(width), x))


def throttle_geom(out, plim, cap, h):
    """Throttle bar: y of the throttle, the power limit and the slamming cap (0 % bottom, 100 % top)."""
    Y = lambda p: h - max(0.0, min(100.0, p)) / 100.0 * h          # noqa: E731
    return {"out": Y(out), "plim": Y(plim), "cap": Y(cap) if cap is not None and cap < 100 else None}


def speed_tape(sog, h, span_kn=10.0):
    """Speed tape: [(y, kn, label?)] every 1 kn around the speed (centre = sog)."""
    out = []
    k = math.floor(sog - span_kn / 2)
    while k <= sog + span_kn / 2:
        if k >= 0:
            out.append((h / 2 - (k - sog) / span_kn * h, k, k % 5 == 0))
        k += 1
    return out


def trend(samples, window_s=3.0):
    """Slope (units per second) of [(t, v)] over the last window_s seconds (least squares); 0 if too few."""
    if len(samples) < 3:
        return 0.0
    t1 = samples[-1][0]
    pts = [(t, v) for t, v in samples if t >= t1 - window_s]
    if len(pts) < 3:
        return 0.0
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    mv = sum(v for _, v in pts) / n
    den = sum((t - mt) ** 2 for t, _ in pts)
    return 0.0 if den <= 1e-9 else sum((t - mt) * (v - mv) for t, v in pts) / den


def xte(lat, lon, a_lat, a_lon, b_lat, b_lon):
    """Cross-track error of (lat, lon) from the leg A -> B in metres: + = right of the track (steer left)."""
    k = math.cos(math.radians(b_lat)) * 111320.0
    ax, ay = (a_lon - b_lon) * k, (a_lat - b_lat) * 111320.0
    px, py = (lon - b_lon) * k, (lat - b_lat) * 111320.0
    dx, dy = -ax, -ay                                   # A -> B
    L = math.hypot(dx, dy)
    if L < 1.0:
        return 0.0
    return ((px - ax) * dy - (py - ay) * dx) / L        # cross product: + = right of A -> B


def cdi_geom(xte_m, scale_m, w):
    """CDI needle x (centre = w/2), clamped; dots every quarter of the full scale."""
    full = max(1.0, scale_m)
    x = w / 2 - max(-1.0, min(1.0, xte_m / full)) * (w / 2 - 8)       # right of track -> needle left
    return x, [w / 2 + i * (w / 2 - 8) / 4 for i in (-4, -3, -2, -1, 1, 2, 3, 4)]


def dist_bearing(lat1, lon1, lat2, lon2):
    k = math.cos(math.radians(lat1)) * 111320.0
    dx, dy = (lon2 - lon1) * k, (lat2 - lat1) * 111320.0
    return math.hypot(dx, dy), math.degrees(math.atan2(dx, dy)) % 360.0


def clock_position(rel_deg):
    """Relative bearing -> clock position 1..12 (12 = ahead, 3 = starboard beam)."""
    c = int(round((rel_deg % 360.0) / 30.0)) % 12
    return 12 if c == 0 else c


def fmt_dist(m):
    """Distance as on a plotter: metres below 0.25 nm, else nm."""
    if m is None:
        return "-"
    return "%.0f m" % m if m < 463 else "%.2f %s" % (m / 1852.0, T("hud.nm"))


def fmt_s(s):
    if s is None or s < 0 or s > 359999:
        return "--:--"
    s = int(s)
    return "%d:%02d:%02d" % (s // 3600, s // 60 % 60, s % 60) if s >= 3600 else "%02d:%02d" % (s // 60, s % 60)


def dynamic_keys():
    """Language keys this module builds at run time (test_lang.py)."""
    return ["hud.decl_" + d for d in DECLUTTER] + ["hud.ov_" + o for o in OVERLAYS] + \
        ["hud.scale_fill", "hud.scale_fit"]


# --- the snapshot: everything the HUD shows, worked out by the window ------------------------------------
class Snapshot(dict):
    """t (telemetry or None), age_s, gw (gateway status), lamps (alerts.MasterLamps), settings ...
    A dict with attribute access; missing values = None."""

    def __getattr__(self, k):
        return self.get(k)


# --- the painter ---------------------------------------------------------------------------------------
class Hud:
    def __init__(self, layout=None):
        self.layout = layout or Layout()
        self.ops, self.hits = [], []
        self.W = self.H = 1
        self.pal = palette()
        self.edit = False
        self.measure = lambda s, size, bold=False: len(s) * size * 0.58      # the window sets the real one

    # primitive ops
    def panel(self, r, tag, alpha=None):
        self.ops.append({"k": "panel", "r": r, "tag": tag, "alpha": self.pal["alpha"] if alpha is None else alpha,
                         "fill": self.pal["panel"]})

    def rect(self, r, tag, fill=None, outline=None, w=2, alpha=255):
        self.ops.append({"k": "rect", "r": r, "tag": tag, "fill": fill, "outline": outline, "w": w, "alpha": alpha})

    def line(self, pts, tag, fill=None, w=2, alpha=255):
        self.ops.append({"k": "line", "pts": [float(v) for v in pts], "tag": tag, "fill": fill or self.pal["main"],
                         "w": w, "alpha": alpha})

    def poly(self, pts, tag, fill=None, outline=None, alpha=255, w=1):
        self.ops.append({"k": "poly", "pts": [float(v) for v in pts], "tag": tag, "fill": fill, "outline": outline,
                         "alpha": alpha, "w": w})

    def oval(self, r, tag, fill=None, outline=None, w=2, alpha=255):
        self.ops.append({"k": "oval", "r": r, "tag": tag, "fill": fill, "outline": outline, "w": w, "alpha": alpha})

    def text(self, xy, s, tag, size=None, bold=False, fill=None, anchor="nw", max_w=None, box=None):
        """Outlined text. max_w: shrink to fit; box: the overlay rectangle it must stay inside (test)."""
        self.ops.append({"k": "text", "xy": xy, "s": s, "tag": tag, "size": int(size or self.f_small), "bold": bold,
                         "fill": fill or self.pal["main"], "anchor": anchor, "max_w": max_w, "box": box})

    def hit(self, r, action):
        self.hits.append((r, action))

    # helpers
    def R(self, name):
        return self.layout.px(name, self.W, self.H)

    def shown(self, name, s):
        if name in self.layout.hidden:
            return False
        return name in ALWAYS or declutter_visible(name, s.declutter or "FULL")

    def button(self, r, label, action, tag, on=False, colour=None):
        x0, y0, x1, y1 = r
        col = colour or self.pal["main"]
        self.panel(r, tag, alpha=min(230, self.pal["alpha"] + 50))
        self.rect(r, tag, outline=col, w=3 if on else 1, fill=col if on else None, alpha=90 if on else 255)
        self.text(((x0 + x1) / 2, (y0 + y1) / 2), label, tag, size=self.f_small, bold=True, fill=col, anchor="c",
                  max_w=x1 - x0 - 6, box=r)
        self.hit(r, action)

    # --- the whole HUD ---------------------------------------------------------------------------------
    def build(self, W, H, s):
        self.W, self.H = max(200, W), max(150, H)
        self.ops, self.hits = [], []
        self.pal = palette(s.hud_colour or "green", bool(s.night))
        fs = theme.hud_sizes(self.H)                    # the text levels of theme.py
        self.f_big = fs["big"]                          # speed
        self.f_mode = fs["mode"]                        # mode
        self.f_comm = fs["comm"]                        # channel / latency / power
        self.f_norm = fs["norm"]
        self.f_small = fs["small"]
        d = s.t
        for name, fn in (("mirror_l", self.mirror), ("mirror_r", self.mirror), ("throttle", self.throttle),
                         ("speed", self.speed), ("depth", self.depth), ("waypoint", self.waypoint),
                         ("traffic", self.traffic), ("rot", self.rot), ("gmeter", self.gmeter),
                         ("steer", self.steer), ("cdi", self.cdi), ("minimap", self.minimap),
                         ("bottom", self.bottom), ("controls", self.controls), ("drive", self.drive),
                         ("status", self.status),
                         ("master", self.master), ("notices", self.notices), ("alarms", self.alarms)):
            if not self.shown(name, s):
                continue
            if d is None and name not in ("mirror_l", "mirror_r", "status", "master", "alarms", "controls", "bottom",
                                          "minimap", "notices", "drive"):
                continue
            fn(name, s)
        self.logo(s)
        if self.edit:
            self.edit_frames(s)
        return self.ops, self.hits

    def logo_rect(self):
        """The logo's place: LOGO_BOX, aspect kept, right-bottom aligned, scaled with the window."""
        x, y, w, h = LOGO_BOX
        bw, bh = w * self.W, h * self.H
        lh = min(bh, theme.logo_height(self.W, self.H) * 1.2)
        lw, lh = theme.logo_size(int(lh), dark=True)
        if lw > bw:
            lh = lh * bw / lw
            lw = bw
        x1, y1 = (x + w) * self.W, (y + h) * self.H
        return (x1 - lw, y1 - lh, x1, y1)

    def logo(self, s):
        self.ops.append({"k": "image", "src": "logo", "r": self.logo_rect(), "tag": "logo"})

    # --- overlays --------------------------------------------------------------------------------------------
    def mirror(self, name, s):
        r = self.R(name)
        if s.rear_big:                      # MANEUVER / astern: the rear cameras emphasised (CLAUDE.md C)
            x0, y0, x1, y1 = r
            k = 1.7
            w, h = (x1 - x0) * k, (y1 - y0) * k
            if name == "mirror_l":
                r = (x0, y0, min(self.W * 0.49, x0 + w), y0 + h)
            else:
                r = (max(self.W * 0.51, x1 - w), y0, x1, y0 + h)
        m = (s.mirrors or {}).get(name) or {}
        self.ops.append({"k": "image", "src": name, "r": r, "tag": name})          # the picture (renderer)
        self.rect(r, name, outline=self.pal["dim"], w=1)
        label = T("hud.rear_left") if name == "mirror_l" else T("hud.rear_right")
        if s.rear_big:
            label += "  " + T("hud.rear_maneuver")
        self.text((r[0] + 6, r[1] + 4), label, name, size=self.f_small, bold=True, fill=self.pal["main"],
                  max_w=r[2] - r[0] - 12, box=r)
        if not m.get("live"):
            cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
            self.rect((r[0] + 8, cy - self.f_norm, r[2] - 8, cy + self.f_norm), name + "_frozen",
                      fill=self.pal["warn"], alpha=200)
            self.text((cx, cy), T("hud.video_frozen") if m.get("have") else T("hud.no_video"), name + "_frozen",
                      size=self.f_norm, bold=True, fill=theme.WHITE, anchor="c", max_w=r[2] - r[0] - 20, box=r)

    def status(self, name, s):
        r = self.R(name)
        x0, y0, x1, y1 = r
        cx, w = (x0 + x1) / 2, x1 - x0
        self.panel(r, name)
        d = s.t
        if d is None:
            self.text((cx, y0 + (y1 - y0) * 0.35), T("alarm.no_data"), "mode", size=self.f_mode, bold=True,
                      fill=self.pal["warn"], anchor="c", max_w=w - 12, box=r)
            return
        mode = T("mode." + d["mode_name"])
        if d["mode"] in (2, 3):                                     # HDG LOCK / AUTO: + the target heading
            mode += "  %03.0f°" % d["target"]
        col = self.pal["main"]
        if d["mode"] == 5 and d.get("remote_state") == 2:
            col = self.pal["warn"]
        self.text((cx, y0 + 4), mode, "mode", size=self.f_mode, bold=True, fill=col, anchor="n", max_w=w - 12, box=r)
        if s.declutter == "MINIMAL":
            return
        # communication + power: bigger and bold
        yl = y0 + 8 + self.f_mode * 1.25
        n = (s.gw or {}).get("net") or {}
        parts = []
        if n:
            if n.get("emergency"):
                parts.append((T("hud.starlink_emerg"), self.pal["warn"], True))
            else:
                parts.append((n.get("active") or T("hud.no_internet"), self.pal["main"] if n.get("active")
                              else self.pal["warn"], False))
        else:
            parts.append((T("hud.link_direct"), self.pal["main"], False))
        if s.latency_ms is not None:
            parts.append((T("hud.latency", ms=s.latency_ms), latency_colour(s.latency_ms, self.pal), False))
        if s.age_s is not None and s.age_s > 0.6:
            parts.append((T("hud.data_old", s="%.1f" % s.age_s), self.pal["warn"] if s.age_s > 2 else
                          self.pal["caution"], False))
        parts.append((T("hud.power", f="%.0f" % d["power_fwd"]), self.pal["caution"] if d.get("power_raised")
                      else self.pal["main"], False))
        # lay the parts out in one line, centred; shrink together if the line is too long
        size = self.f_comm
        while True:
            gap = size * 0.9
            widths = [self.measure(p[0], size, True) for p in parts]
            total = sum(widths) + gap * (len(parts) - 1)
            if total <= w - 16 or size <= 9:
                break
            size -= 1
        xs = cx - total / 2
        for (txt, col, boxed), tw in zip(parts, widths):
            if boxed:
                self.rect((xs - 4, yl - 2, xs + tw + 4, yl + size * 1.3), "starlink", fill=self.pal["warn"],
                          alpha=110)
            self.text((xs, yl), txt, "comm", size=size, bold=True, fill=col, max_w=x1 - xs - 4, box=r)
            xs += tw + gap

    def master(self, name, s):
        x0, y0, x1, y1 = self.R(name)
        lamps = s.lamps
        hh = (y1 - y0 - 6) / 2
        flash = s.flash
        for i, (lv, key, col) in enumerate(((3, "hud.master_warning", self.pal["warn"]),
                                            (2, "hud.master_caution", self.pal["caution"]))):
            r = (x0, y0 + i * (hh + 6), x1, y0 + i * (hh + 6) + hh)
            on = bool(lamps and lamps.on(lv))
            self.panel(r, "lamp_%d" % lv)
            if on:
                self.rect(r, "lamp_%d" % lv, fill=col, alpha=255 if flash else 170)
            self.rect(r, "lamp_%d" % lv, outline=col, w=3)
            words = T(key).split("\n")
            for j, wd in enumerate(words):
                self.text(((x0 + x1) / 2, r[1] + hh * (j + 0.5) / len(words)), wd, "lamp_%d" % lv,
                          size=self.f_small, bold=True, fill=theme.BLACK if on else col, anchor="c", max_w=x1 - x0 - 6,
                          box=r)
            self.hit(r, "ack")

    def alarms(self, name, s):
        items = s.alarm_items or []                         # [(text, level)] most severe first
        if not items:
            return
        x0, y0, x1, y1 = self.R(name)
        self.panel((x0, y0, x1, y1), name, alpha=min(230, self.pal["alpha"] + 60))
        col = self.pal["warn"] if items[0][1] >= 3 else self.pal["caution"]
        self.rect((x0, y0, x1, y1), name, outline=col, w=3)
        txt = "  •  ".join(t for t, _l in items)
        self.text(((x0 + x1) / 2, (y0 + y1) / 2), txt, "alarm", size=self.f_norm, bold=True, fill=col, anchor="c",
                  max_w=x1 - x0 - 12, box=(x0, y0, x1, y1))

    def notices(self, name, s):
        lines = s.notices or []                              # [(tag, text, colour key)]
        if not lines:
            return
        x0, y0, x1, y1 = self.R(name)
        h = y1 - y0
        for i, (tag, txt, ck) in enumerate(lines[:3]):
            r = (x0, y0 + i * (h + 3), x1, y0 + i * (h + 3) + h)
            self.panel(r, tag, alpha=min(230, self.pal["alpha"] + 60))
            col = self.pal.get(ck, ck)
            self.rect(r, tag, outline=col, w=2)
            self.text(((x0 + x1) / 2, r[1] + h / 2), txt, tag, size=self.f_norm, bold=True, fill=col, anchor="c",
                      max_w=x1 - x0 - 12, box=r)

    def throttle(self, name, s):
        d = s.t
        x0, y0, x1, y1 = self.R(name)
        w = x1 - x0
        gear_h = self.f_big * 1.15
        if s.declutter == "MINIMAL":
            y0 = y1 - gear_h - 6                     # only the gear letter
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        bar = (x0 + w * 0.30, y0 + self.f_small * 1.6, x0 + w * 0.70, y1 - gear_h - self.f_small * 1.4)
        bh = bar[3] - bar[1]
        if s.declutter != "MINIMAL" and bh > 30:
            cap = d.get("slam_cap")
            g = throttle_geom(d["throttle"], d["power_fwd"], cap, bh)
            self.rect(bar, "thr", outline=self.pal["dim"], w=1)
            self.rect((bar[0], bar[1] + g["out"], bar[2], bar[3]), "thr", fill=self.pal["main"], alpha=200)
            if g["cap"] is not None:                                 # slamming reduction: amber band
                self.rect((bar[0], bar[1] + g["plim"], bar[2], bar[1] + g["cap"]), "cap", fill=self.pal["caution"],
                          alpha=150)
                self.text(((bar[0] + bar[2]) / 2, bar[1] + g["cap"] + 2), T("hud.slam_short"), "cap",
                          size=self.f_small, bold=True, fill=self.pal["caution"], anchor="n", max_w=w - 4, box=r)
            yl = bar[1] + g["plim"]
            self.line((bar[0] - w * 0.18, yl, bar[2] + w * 0.18, yl), "plim", fill=self.pal["caution"], w=3)
            self.text(((x0 + x1) / 2, y0 + 3), "%.0f %%" % d["throttle"], "thr_val", size=self.f_small, bold=True,
                      anchor="n", max_w=w - 4, box=r)
            self.text(((x0 + x1) / 2, bar[3] + 2), T("hud.limit", f="%.0f" % d["power_fwd"]), "plim_txt",
                      size=self.f_small, fill=self.pal["caution"], anchor="n", max_w=w - 4, box=r)
            # speed trend arrow (beside the bar, at the top)
            tr = s.sog_trend or 0.0
            if abs(tr) > 0.05:
                ax = bar[2] + w * 0.12
                ay = bar[1] + bh * 0.15
                L = min(bh * 0.25, 12 + abs(tr) * 40)
                sgn = -1 if tr > 0 else 1
                self.line((ax, ay, ax, ay + sgn * L), "trend", w=3)
                self.poly((ax - 6, ay + sgn * L, ax + 6, ay + sgn * L, ax, ay + sgn * (L + 9)), "trend",
                          fill=self.pal["main"])
        gear = d["gear"] if d["gear"] < 3 else None
        gcol = {0: self.pal["main"], 1: self.pal["main"], 2: self.pal["caution"]}.get(gear, self.pal["warn"])
        gr = (x0 + 4, y1 - gear_h, x1 - 4, y1 - 3)
        self.rect(gr, "gear", outline=gcol, w=3)
        self.text(((gr[0] + gr[2]) / 2, (gr[1] + gr[3]) / 2), T("gear.%d" % gear) if gear is not None else "?",
                  "gear", size=self.f_big, bold=True, fill=gcol, anchor="c", max_w=gr[2] - gr[0] - 4, box=r)

    def speed(self, name, s):
        d = s.t
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        w, h = x1 - x0, y1 - y0
        sog = d["est_sog"]
        # tape on the left third
        tx = x0 + w * 0.22
        for y, k, lab in speed_tape(sog, h - 8):
            yy = y0 + 4 + y
            self.line((tx - (10 if lab else 5), yy, tx, yy), "stape", fill=self.pal["dim"], w=2 if lab else 1)
            if lab:
                self.text((tx - 12, yy), "%d" % k, "stape", size=self.f_small, fill=self.pal["dim"], anchor="e",
                          box=r)
        self.line((tx, y0 + 4, tx, y1 - 4), "stape", fill=self.pal["dim"], w=1)
        if d["mode"] == 3 and d.get("speed_set_kn"):                 # AUTO set speed bug
            yb = y0 + 4 + (h - 8) / 2 - (d["speed_set_kn"] - sog) / 10.0 * (h - 8)
            if y0 < yb < y1:
                self.poly((tx, yb, tx + 9, yb - 6, tx + 9, yb + 6), "sbug", fill=theme.ADVISORY)
        self.text((x1 - 6, y0 + h * 0.48), "%.1f" % sog, "speed", size=self.f_big, bold=True, anchor="e",
                  max_w=w * 0.74, box=r)
        self.text((x1 - 6, y1 - 4), T("hud.kn"), "speed_unit", size=self.f_small, bold=True, anchor="se", box=r)
        tr = s.sog_trend or 0.0                                      # trend: where the speed will be in 10 s
        if abs(tr) > 0.03:
            yt = y0 + h / 2 - max(-0.5, min(0.5, tr * 10.0 / 10.0)) * (h - 8)
            self.line((tx + 4, y0 + h / 2, tx + 4, yt), "strend", fill=self.pal["main"], w=4)
        self.text((x1 - 6, y0 + 3), T("hud.sog"), "speed_lbl", size=self.f_small, fill=self.pal["dim"], anchor="ne",
                  max_w=w * 0.7, box=r)

    def depth(self, name, s):
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        dep, lim = s.depth_m, s.shallow_limit or 3.0
        if dep is None:
            txt, col = T("hud.depth_none"), self.pal["caution"] if s.have_charts else self.pal["grey"]
        else:
            txt = T("hud.depth", m="%.1f" % dep if dep < 20 else "%.0f" % dep)
            col = self.pal["warn"] if dep < lim else self.pal["caution"] if dep < 2 * lim else self.pal["main"]
        self.text(((x0 + x1) / 2, (y0 + y1) / 2), txt, "depth", size=self.f_norm, bold=True, fill=col, anchor="c",
                  max_w=x1 - x0 - 8, box=r)

    def waypoint(self, name, s):
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        wp = s.wp
        lh = (y1 - y0 - 6) / 4
        if not wp:
            self.text((x0 + 6, y0 + 4), T("hud.no_wp"), "wp", size=self.f_small, fill=self.pal["grey"],
                      max_w=x1 - x0 - 12, box=r)
            return
        rows = [(T("hud.wp_n", n=wp["n"], of=wp["of"]), True),
                (T("hud.wp_dist", d=fmt_dist(wp["dist_m"]), brg="%03.0f" % wp["brg"]), False),
                (T("hud.wp_eta", eta=wp.get("eta") or "--:--"), False),
                (T("hud.wp_ttg", ttg=fmt_s(wp.get("ttg_s"))), False)]
        for i, (txt, bold) in enumerate(rows):
            self.text((x0 + 6, y0 + 3 + i * lh), txt, "wp", size=self.f_small + (2 if bold else 0), bold=bold,
                      max_w=x1 - x0 - 12, box=r)

    def traffic(self, name, s):
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        th = s.threat
        lh = (y1 - y0 - 6) / 3
        if not th or not th.get("level"):
            self.text((x0 + 6, y0 + 4), T("hud.ais_n", n=s.ais_n or 0), "traffic", size=self.f_small,
                      fill=self.pal["dim"], max_w=x1 - x0 - 12, box=r)
            self.text((x0 + 6, y0 + 4 + lh), T("hud.traffic_none"), "traffic", size=self.f_small, bold=True,
                      fill=self.pal["main"], max_w=x1 - x0 - 12, box=r)
            if th:                                           # the nearest ship, no threat
                self.text((x0 + 6, y0 + 4 + 2 * lh), "%s  %s" % ((th.get("name") or str(th.get("mmsi")))[:14],
                                                                fmt_dist((th.get("dist_nm") or 0) * 1852.0)),
                          "traffic", size=self.f_small, fill=self.pal["dim"], max_w=x1 - x0 - 12, box=r)
            return
        col = self.pal["warn"] if th.get("level") == "danger" else self.pal["caution"] if th.get("level") == "warn" \
            else self.pal["main"]
        self.rect(r, "traffic", outline=col, w=2)
        rows = [(th.get("name") or str(th.get("mmsi")), True),
                (T("hud.cpa", cpa="-" if th.get("cpa_nm") is None else "%.2f" % th["cpa_nm"],
                   tcpa="-" if th.get("tcpa_min") is None or th["tcpa_min"] < 0 else "%.1f" % th["tcpa_min"]), False),
                (T("hud.traffic_at", d=fmt_dist((th.get("dist_nm") or 0) * 1852.0) if th.get("dist_nm") is not None
                   else "-", brg="-" if th.get("bearing") is None else "%03.0f" % th["bearing"]), False)]
        for i, (txt, bold) in enumerate(rows):
            self.text((x0 + 6, y0 + 3 + i * lh), txt, "traffic", size=self.f_small + (1 if bold else 0), bold=bold,
                      fill=col, max_w=x1 - x0 - 12, box=r)

    def rot(self, name, s):
        d = s.t
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        cx, cy = (x0 + x1) / 2, y1 - (y1 - y0) * 0.30
        rad = min((x1 - x0) / 2 - 4, (y1 - y0) * 0.55)
        full = 15.0                                                   # deg/s at the end of the scale
        for a in (-1.0, -0.5, 0.0, 0.5, 1.0):
            ang = math.radians(a * 75.0)
            self.line((cx + (rad - 6) * math.sin(ang), cy - (rad - 6) * math.cos(ang), cx + rad * math.sin(ang),
                       cy - rad * math.cos(ang)), "rot", fill=self.pal["dim"], w=2)
        yr = d["yaw_rate"]
        ang = math.radians(max(-1.0, min(1.0, yr / full)) * 75.0)
        self.line((cx, cy, cx + (rad - 2) * math.sin(ang), cy - (rad - 2) * math.cos(ang)), "rot_needle", w=3)
        self.text((cx, y0 + 2), T("hud.rot"), "rot", size=self.f_small, fill=self.pal["dim"], anchor="n",
                  max_w=x1 - x0 - 4, box=r)
        self.text((cx, y1 - 2), "%+.0f°/s" % yr, "rot", size=self.f_small, bold=True, anchor="s",
                  max_w=x1 - x0 - 4, box=r)

    def gmeter(self, name, s):
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        g = s.gmeter
        self.text(((x0 + x1) / 2, y0 + 2), T("hud.gmeter"), "gmeter", size=self.f_small, fill=self.pal["dim"],
                  anchor="n", max_w=x1 - x0 - 4, box=r)
        if not g:
            self.text(((x0 + x1) / 2, (y0 + y1) / 2), "-", "gmeter", size=self.f_norm, fill=self.pal["grey"],
                      anchor="c", box=r)
            return
        top = max(g["limit"] * 1.3, g["hold"] * 1.05, 1.0)
        b = (x0 + (x1 - x0) * 0.25, y0 + self.f_small * 1.5, x0 + (x1 - x0) * 0.55, y1 - self.f_small * 1.5)
        bh = b[3] - b[1]
        Y = lambda v: b[3] - min(1.0, v / top) * bh                    # noqa: E731
        col = self.pal["warn"] if g["peak"] >= g["limit"] else self.pal["caution"] if g["peak"] >= g["yellow"] \
            else self.pal["main"]
        self.rect(b, "gmeter", outline=self.pal["dim"], w=1)
        self.rect((b[0], Y(g["peak"]), b[2], b[3]), "g_peak", fill=col, alpha=210)
        self.line((b[0] - 3, Y(g["yellow"]), b[2] + 3, Y(g["yellow"])), "g_yellow", fill=self.pal["caution"], w=2)
        self.line((b[0] - 5, Y(g["limit"]), b[2] + 5, Y(g["limit"])), "g_limit", fill=self.pal["warn"], w=3)
        self.poly((b[2] + 2, Y(g["hold"]), b[2] + 10, Y(g["hold"]) - 5, b[2] + 10, Y(g["hold"]) + 5), "g_hold",
                  fill=self.pal["main"])
        self.text(((x0 + x1) / 2, y1 - 2), "%.1f g" % g["hold"], "gmeter", size=self.f_small, bold=True,
                  fill=col, anchor="s", max_w=x1 - x0 - 4, box=r)

    def steer(self, name, s):
        d = s.t
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        w = x1 - x0
        cy = y0 + (y1 - y0) * 0.62
        full = 40.0
        for a in range(-40, 41, 10):
            x = x0 + w / 2 + a / full * (w / 2 - 14)
            self.line((x, cy - (7 if a % 20 == 0 else 4), x, cy + (7 if a % 20 == 0 else 4)), "steer",
                      fill=self.pal["dim"], w=2 if a == 0 else 1)
        self.line((x0 + 14, cy, x1 - 14, cy), "steer", fill=self.pal["dim"], w=1)
        ang = max(-full, min(full, d["outboard"]))
        x = x0 + w / 2 + ang / full * (w / 2 - 14)
        self.poly((x, cy - 9, x - 7, cy + 9, x + 7, cy + 9), "steer_needle", fill=self.pal["main"])
        side = T("hud.stbd") if d["outboard"] > 0.5 else T("hud.port") if d["outboard"] < -0.5 else ""
        self.text((x0 + w / 2, y0 + 2), T("hud.outboard", a="%.0f" % abs(d["outboard"]), side=side), "steer",
                  size=self.f_small, bold=True, anchor="n", max_w=w - 8, box=r)

    def cdi(self, name, s):
        if s.t is not None and s.t["mode"] == 7:                        # MANEUVER: the gear state instead
            return self.maneuver(name, s)
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        w = x1 - x0
        if s.xte_m is None:
            self.text((x0 + w / 2, (y0 + y1) / 2), T("hud.cdi_none"), "cdi", size=self.f_small,
                      fill=self.pal["grey"], anchor="c", max_w=w - 8, box=r)
            return
        cy = y0 + (y1 - y0) * 0.62
        nx, dots = cdi_geom(s.xte_m, s.xte_scale or 50.0, w - 16)
        for dx in dots:
            self.oval((x0 + 8 + dx - 3, cy - 3, x0 + 8 + dx + 3, cy + 3), "cdi", outline=self.pal["dim"], w=1)
        self.line((x0 + w / 2, cy - 10, x0 + w / 2, cy + 10), "cdi", fill=self.pal["dim"], w=2)
        self.line((x0 + 8 + nx, cy - 12, x0 + 8 + nx, cy + 12), "cdi_needle", fill=theme.MAGENTA, w=4)
        side = T("hud.right_of") if s.xte_m > 0 else T("hud.left_of") if s.xte_m < 0 else ""
        self.text((x0 + w / 2, y0 + 2), T("hud.xte", m="%.0f" % abs(s.xte_m), side=side,
                                          scale="%.0f" % (s.xte_scale or 50.0)), "cdi", size=self.f_small,
                  bold=True, anchor="n", max_w=w - 8, box=r)

    def maneuver(self, name, s):
        """MANEUVER (CLAUDE.md C): throttle always idle - only REV / N / FWD. The lit box = the gear engaged
        now (telemetry); the outlined box = what the captain's lever asks for."""
        d = s.t
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, "maneuver", alpha=min(230, self.pal["alpha"] + 50))
        gear = d["gear"] if d["gear"] < 3 else None                  # 0 N, 1 F, 2 R
        ask = s.mv_ask                                               # same codes, None = unknown
        w = (x1 - x0 - 16) / 3.0
        top = y0 + (y1 - y0) * 0.30
        self.text(((x0 + x1) / 2, y0 + 2), T("mv.title"), "maneuver", size=self.f_small, bold=True, anchor="n",
                  max_w=x1 - x0 - 8, box=r)
        for i, (g, key) in enumerate(((2, "mv.rev"), (0, "mv.n"), (1, "mv.fwd"))):
            b = (x0 + 4 + i * (w + 4), top, x0 + 4 + i * (w + 4) + w, y1 - 3)
            on = gear == g
            col = self.pal["caution"] if g == 2 else self.pal["main"]
            if on:
                self.rect(b, "mv_gear", fill=col, alpha=220)
            self.rect(b, "mv_box", outline=col if (on or ask == g) else self.pal["dim"], w=4 if ask == g else 1)
            self.text(((b[0] + b[2]) / 2, (b[1] + b[3]) / 2), T(key), "mv_" + ("on" if on else "off"),
                      size=self.f_norm, bold=True, fill=theme.BLACK if on else col, anchor="c", max_w=w - 6, box=r)

    def drive(self, name, s):
        """Manual driving from this station (CLAUDE.md D): REMOTE MANUAL and MANEUVER for the captain with the
        manual licence (or single-operator + licence); RELEASE while one runs. The gateway (licence, captain)
        and the AURIX (CONFIRM on the joystick, priorities, watchdogs) check every request again."""
        d = s.t
        mode = d["mode"] if d is not None else None
        btns = []
        if s.manual_ok and mode not in (5, 7):
            btns += [(T("hud.manual_req"), "remote_req", False), (T("hud.maneuver_req"), "maneuver_req", False)]
        if mode in (5, 7):
            btns.append((T("hud.release"), "release", True))
        if not btns:
            return
        x0, y0, x1, y1 = self.R(name)
        n = len(btns)
        bw = (x1 - x0 - (n - 1) * 4) / n
        for i, (lab, act, on) in enumerate(btns):
            r = (x0 + i * (bw + 4), y0, x0 + i * (bw + 4) + bw, y1)
            self.button(r, lab, act, "drv_" + act, on=on, colour=self.pal["caution"] if act == "release" else None)

    def minimap(self, name, s):
        r = self.R(name)
        self.ops.append({"k": "minimap", "r": r, "tag": name})          # a child canvas (driver.py)

    def bottom(self, name, s):
        d = s.t
        x0, y0, x1, y1 = self.R(name)
        r = (x0, y0, x1, y1)
        self.panel(r, name)
        h = y1 - y0
        w = x1 - x0
        minimal = s.declutter == "MINIMAL"
        tw = w * (0.98 if minimal else 0.52)                           # heading tape
        tx0 = x0 + (w - tw) / 2 if minimal else x0 + 4
        if d is not None:
            hd = d["heading"]
            for x, deg, lab in tape_ticks(hd, tw, span=60.0 if tw < 600 else 90.0):
                xx = tx0 + x
                self.line((xx, y1 - (h * 0.40 if lab else h * 0.22), xx, y1 - 2), "tape", fill=self.pal["dim"],
                          w=1)
                if lab:
                    self.text((xx, y0 + 1), lab, "tape", size=self.f_small, fill=self.pal["dim"], anchor="n", box=r)
            cx = tx0 + tw / 2
            if d["mode"] in (2, 3):
                txx = tx0 + tape_x(hd, d["target"], tw, span=60.0 if tw < 600 else 90.0)
                self.rect((txx - 5, y0 + h * 0.45, txx + 5, y1 - 1), "target", fill=theme.ADVISORY)
            self.rect((cx - h * 1.05, y0 + 1, cx + h * 1.05, y1 - 1), "hdg", fill=theme.BLACK, outline=self.pal["main"],
                      w=2, alpha=230)
            self.text((cx, (y0 + y1) / 2), "%03.0f°" % hd if d["compass_ok"] else T("hud.no_compass"), "hdg",
                      size=int(h * 0.62), bold=True, fill=self.pal["main"] if d["compass_ok"] else self.pal["warn"],
                      anchor="c", max_w=h * 2.0, box=r)
        if minimal:
            return
        # REC, vision, mission clock, SAVE - in one line to the right of the tape
        x = tx0 + tw + 12
        save_w = max(70, h * 2.6)
        rest = x1 - save_w - 8 - x
        items = []
        rec = s.rec
        if rec is not None:
            items.append((T("hud.rec") if rec else T("hud.not_rec"), self.pal["warn"] if not rec else theme.REC,
                          "rec"))
        if s.vision:
            items.append((s.vision[0], self.pal.get(s.vision[1], s.vision[1]), "vision"))
        items.append((T("hud.clock", clock=s.clock or "--:--", mission=s.mission or "--:--"), self.pal["main"],
                      "clock"))
        if items:
            cw = rest / len(items)
            for i, (txt, col, tag) in enumerate(items):
                xx = x + i * cw
                if tag == "rec" and rec:
                    self.oval((xx, y0 + h * 0.3, xx + h * 0.4, y0 + h * 0.7), "rec", fill=theme.REC)
                    xx += h * 0.5
                self.text((xx, (y0 + y1) / 2), txt, tag, size=self.f_small, bold=True, fill=col, anchor="w",
                          max_w=cw - 8, box=r)
        sr = (x1 - save_w - 4, y0 + 2, x1 - 4, y1 - 2)
        self.rect(sr, "save", outline=self.pal["warn"], w=3)
        self.text(((sr[0] + sr[2]) / 2, (sr[1] + sr[3]) / 2), T("hud.save"), "save", size=self.f_norm, bold=True,
                  fill=self.pal["warn"], anchor="c", max_w=save_w - 8, box=r)
        self.hit(sr, "save")

    def controls(self, name, s):
        x0, y0, x1, y1 = self.R(name)
        btns = [(i18n.NAMES[i18n.other()], "lang", False),
                (T("hud.night"), "night", bool(s.night)),
                (T("hud.decl_" + (s.declutter or "FULL")), "declutter", False),
                (T("hud.scale_" + (s.video_scale or "fill")), "scale", False),
                (T("hud.ar"), "ar_menu", bool(s.ar_on)),
                (T("hud.layout"), "edit", self.edit)]
        if s.vision_cap:
            btns.append((T("hud.clear_slowdown"), "vision_clear", True))
        n = len(btns)
        bw = (x1 - x0 - (n - 1) * 4) / n
        for i, (lab, act, on) in enumerate(btns):
            r = (x0 + i * (bw + 4), y0, x0 + i * (bw + 4) + bw, y1)
            self.button(r, lab, act, "ctl_" + act, on=on,
                        colour=self.pal["caution"] if act == "vision_clear" else None)

    # --- edit-layout mode ---------------------------------------------------------------------------------------
    def edit_frames(self, s):
        for name in OVERLAYS:
            r = self.R(name)
            hidden = name in self.layout.hidden
            col = self.pal["grey"] if hidden else theme.ADVISORY
            self.rect(r, "edit", outline=col, w=2)
            self.text((r[0] + 3, r[1] + 2), T("hud.ov_" + name) + (" (" + T("hud.hidden") + ")" if hidden else ""),
                      "edit", size=self.f_small, bold=True, fill=col, max_w=r[2] - r[0] - 26)
            hs = 14
            self.rect((r[2] - hs, r[3] - hs, r[2], r[3]), "edit_handle", fill=col)
            self.rect((r[2] - 20, r[1], r[2], r[1] + 18), "edit_eye", fill=col, alpha=200)
            self.text((r[2] - 10, r[1] + 9), "×" if not hidden else "+", "edit_eye", size=12, bold=True,
                      fill=theme.BLACK, anchor="c")
        W, H = self.W, self.H
        bw, bh = 240, 44
        for i, (lab, act) in enumerate(((T("hud.layout_reset"), "edit_reset"), (T("hud.layout_done"), "edit"))):
            r = (W / 2 - bw - 10 + i * (bw + 20), H * 0.45, W / 2 - 10 + i * (bw + 20), H * 0.45 + bh)
            self.button(r, lab, act, "edit_btn", on=True, colour=theme.ADVISORY)
        self.text((W / 2, H * 0.45 - 10), T("hud.layout_help"), "edit", size=self.f_norm, bold=True,
                  fill=theme.ADVISORY, anchor="s", max_w=W * 0.6)

    def edit_target(self, x, y):
        """In the edit mode: (overlay, "move" | "resize" | "eye") under (x, y), topmost first."""
        for name in reversed(OVERLAYS):
            x0, y0, x1, y1 = self.R(name)
            if x1 - 20 <= x <= x1 and y0 <= y <= y0 + 18:
                return name, "eye"
            if x1 - 16 <= x <= x1 and y1 - 16 <= y <= y1:
                return name, "resize"
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name, "move"
        return None, None
