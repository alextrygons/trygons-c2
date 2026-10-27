"""DRIVER screen (section A): the FRONT camera fills the whole window, everything
else is an overlay on top of the video (hud.py builds them, this window draws them).

  +----------+     MODE  target      [WARNING]   +-----------+
  | REAR-L   |  channel latency power [CAUTION]  |  REAR-R   |
  | (mirror) |                                   |  (mirror) |
  +----------+                                   +-----------+
  thr                                                  ALARMS (one per line,
  bar           FRONT camera (+ AR, vision)            flashing = not acked)
  gear                                                 notices
                                                       SPEED  depth
                                                       waypoint  traffic
                    outboard angle       mini map      ROT  G
  [toolbar]          CDI
  heading tape ...                REC  vision  clock   [SAVE]

The HUD itself never steers or sets the throttle. What it sends: the captain's
answers to the copilot's questions (ACCEPT / REJECT), SAVE (video), CLEAR
SLOWDOWN (vision) and - for the captain with the manual licence only - the
REQUESTS for REMOTE MANUAL / MANEUVER and their RELEASE (CLAUDE.md D: manual
driving starts on screen 1; the gateway checks licence + captain, the AURIX the
joystick CONFIRM, priorities and watchdogs). MANEUVER shows REV / N / FWD. The
MASTER lamps only silence this PC. Look (fonts, colours, the logo): theme.py,
the same module as screen 2.

Keys: Space / A acknowledge the MASTER lamps, N night, D declutter (FULL /
NORMAL / MINIMAL), V fill / fit, L language, E edit layout, G AR overlays,
O obstacle overlay, F11 full screen, F12 swap with the captain screen,
Ctrl+Shift+1..9 move to monitor n.
Rendering: Pillow (real transparency, outlined text); without Pillow plain
canvas items (stippled panels).
"""
import collections
import io
import math
import os
import sys
import time
import tkinter as tk

import alerts
import hud
import i18n
import mapview
import monitors
import proto
import settings
import theme
import video
from i18n import T

try:
    from PIL import Image, ImageDraw, ImageFont, ImageTk
except ImportError:                    # without Pillow: the canvas renderer
    Image = None

try:
    import hudar                        # AR overlays on the front video (section B)
except ImportError:
    hudar = None

BG = theme.BLACK                # behind the video
FG = theme.WHITE
YEL = theme.YELLOW
F_HUD = theme.font("title")
F_MID = theme.font("huge")

# kept for the old tests / other modules
tape_ticks, tape_x, throttle_geom, angdiff = hud.tape_ticks, hud.tape_x, hud.throttle_geom, hud.angdiff


def flip_ppm(data):
    """A binary PPM (P6) mirrored left <-> right (the rear cameras as mirrors; the canvas renderer)."""
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


def decode_mask(b64, w, h):
    """The obstacle detection mask (2 bits per cell: 0 obstacle, 1 water, 2 sky) -> list of w*h classes."""
    import base64
    raw = base64.b64decode(b64)
    out = []
    for byte in raw:
        out += [byte & 3, (byte >> 2) & 3, (byte >> 4) & 3, (byte >> 6) & 3]
    return out[:w * h]


def scale_rect(iw, ih, W, H, mode="fill"):
    """Where a picture iw x ih goes in a window W x H: (x0, y0, w, h) of the WHOLE scaled picture
    (fill = cover the window, cropped evenly - may stick out; fit = the whole picture, black bars)."""
    k = max(W / float(iw), H / float(ih)) if mode == "fill" else min(W / float(iw), H / float(ih))
    w, h = iw * k, ih * k
    return (W - w) / 2.0, (H - h) / 2.0, w, h


# --- fonts + the Pillow renderer -------------------------------------------------------------------------------
_FONTS = {}
_FONT_FILES = theme.FONT_FILES


def font(size, bold=False):
    key = (int(size), bool(bold))
    f = _FONTS.get(key)
    if f is None:
        for name in _FONT_FILES[bool(bold)]:
            for folder in ("", os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts"),
                           "/usr/share/fonts/truetype/dejavu"):
                try:
                    f = ImageFont.truetype(os.path.join(folder, name) if folder else name, key[0])
                    break
                except (OSError, ValueError):
                    continue
            if f is not None:
                break
        if f is None:
            try:
                f = ImageFont.load_default(key[0])
            except TypeError:
                f = ImageFont.load_default()
        _FONTS[key] = f
    return f


_MEASURED = {}


def measure(text, size, bold=False):
    if Image is None:
        return len(text) * size * 0.58
    key = (text, int(size), bool(bold))
    v = _MEASURED.get(key)
    if v is None:
        if len(_MEASURED) > 4000:
            _MEASURED.clear()
        v = _MEASURED[key] = max(font(size, bold).getlength(line) for line in (text.split("\n") or [""]))
    return v


ANCHORS = {"nw": "la", "n": "ma", "ne": "ra", "w": "lm", "c": "mm", "center": "mm", "e": "rm",
           "sw": "ld", "s": "md", "se": "rd"}


def rgba(col, alpha=255):
    col = col.lstrip("#")
    return int(col[0:2], 16), int(col[2:4], 16), int(col[4:6], 16), int(alpha)


class PilRenderer:
    """Draws a display list on top of a picture: real transparency, outlined text."""

    def __init__(self):
        self.text_boxes = []            # (tag, (x0, y0, x1, y1), box) of every text drawn (tests: fits its overlay)
        self.cache = {}                 # (text, size, bold, colour, anchor) -> (picture, dx, dy): labels repeat

    def render(self, base, ops, images):
        """base: RGB picture (window size); images: {src: RGB picture} for "image" ops. -> RGB."""
        W, H = base.size
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        self.layer = layer
        dr = ImageDraw.Draw(layer)
        self.text_boxes = []
        for op in ops:
            k = op["k"]
            if k == "image" and op["src"] == "logo":
                x0, y0, x1, y1 = [int(round(v)) for v in op["r"]]
                im = theme.logo_pil(max(1, y1 - y0), dark=True)       # light lettering: on the video
                if im is not None:
                    _paste_clip(layer, im, x1 - im.width, y0)
            elif k == "image":
                im = images.get(op["src"])
                if im is not None:
                    x0, y0, x1, y1 = [int(round(v)) for v in op["r"]]
                    if x1 - x0 > 2 and y1 - y0 > 2:
                        layer.paste(fill_crop(im, x1 - x0, y1 - y0).convert("RGBA"), (x0, y0))
            elif k == "mask":
                im = op["img"]
                x0, y0, w, h = op["r"]
                if w > 2 and h > 2:
                    m = im.resize((int(w), int(h)), Image.NEAREST)
                    _paste_clip(layer, m, int(x0), int(y0))
            elif k in ("panel", "rect"):
                fill = rgba(op["fill"], op.get("alpha", 255)) if op.get("fill") else None
                outline = rgba(op["outline"]) if op.get("outline") else None
                x0, y0, x1, y1 = op["r"]
                if x1 > x0 and y1 > y0:
                    if fill and op.get("alpha", 255) < 255:
                        self._tile(layer, op["r"], fill)
                    elif fill:
                        dr.rectangle((x0, y0, x1, y1), fill=fill)
                    if outline:
                        dr.rectangle((x0, y0, x1, y1), outline=outline, width=int(op.get("w", 1)))
            elif k == "line":
                pts = op["pts"]
                dr.line(list(zip(pts[0::2], pts[1::2])), fill=rgba(op["fill"], op.get("alpha", 255)),
                        width=int(op.get("w", 2)))
            elif k == "poly":
                pts = list(zip(op["pts"][0::2], op["pts"][1::2]))
                if len(pts) >= 3:
                    if op.get("alpha", 255) < 255 and op.get("fill"):
                        bx0 = max(0, int(min(p[0] for p in pts)))
                        by0 = max(0, int(min(p[1] for p in pts)))
                        bx1 = min(W, int(max(p[0] for p in pts)) + 2)
                        by1 = min(H, int(max(p[1] for p in pts)) + 2)
                        if bx1 > bx0 and by1 > by0:
                            tmp = Image.new("RGBA", (bx1 - bx0, by1 - by0), (0, 0, 0, 0))
                            ImageDraw.Draw(tmp).polygon([(x - bx0, y - by0) for x, y in pts],
                                                        fill=rgba(op["fill"], op["alpha"]))
                            layer.alpha_composite(tmp, (bx0, by0))
                        if op.get("outline"):
                            dr.line(pts + pts[:1], fill=rgba(op["outline"]), width=int(op.get("w", 1)))
                    else:
                        dr.polygon(pts, fill=rgba(op["fill"]) if op.get("fill") else None,
                                   outline=rgba(op["outline"]) if op.get("outline") else None)
            elif k == "oval":
                dr.ellipse(op["r"], fill=rgba(op["fill"], op.get("alpha", 255)) if op.get("fill") else None,
                           outline=rgba(op["outline"]) if op.get("outline") else None, width=int(op.get("w", 1)))
            elif k == "text":
                self._text(dr, op)
        out = base.convert("RGBA")
        out.alpha_composite(layer)
        return out.convert("RGB")

    @staticmethod
    def _tile(layer, r, colour):
        """A see-through rectangle (clipped to the window)."""
        x0, y0, x1, y1 = [int(round(v)) for v in r]
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(layer.width, x1), min(layer.height, y1)
        if x1 > x0 and y1 > y0:
            layer.alpha_composite(Image.new("RGBA", (x1 - x0, y1 - y0), colour), (x0, y0))

    def _text(self, dr, op):
        s = op["s"]
        if not s:
            return
        size, bold = op["size"], op["bold"]
        if op.get("max_w"):
            while size > 8 and measure(s, size, bold) > op["max_w"]:
                size -= 1
        f = font(size, bold)
        anchor = ANCHORS.get(op["anchor"], "la")
        stroke = max(1, size // 9)
        x, y = op["xy"]
        if "\n" not in s:
            key = (s, size, bold, op["fill"], anchor)
            hit = self.cache.get(key)
            if hit is None:
                if len(self.cache) > 1500:
                    self.cache.clear()
                l_, t_, r_, b_ = f.getbbox(s, anchor=anchor, stroke_width=stroke)
                im = Image.new("RGBA", (max(1, r_ - l_), max(1, b_ - t_)), (0, 0, 0, 0))
                ImageDraw.Draw(im).text((-l_, -t_), s, font=f, fill=rgba(op["fill"]), anchor=anchor,
                                        stroke_width=stroke, stroke_fill=(0, 0, 0, 255))
                hit = self.cache[key] = (im, l_, t_)
            im, l_, t_ = hit
            px, py = int(round(x + l_)), int(round(y + t_))
            _paste_clip(self.layer, im, px, py)
            self.text_boxes.append((op["tag"], (px, py, px + im.width, py + im.height), op.get("box"), s))
            return
        if "\n" in s:
            anchor = anchor[0] + "a"
            dr.multiline_text((x, y), s, font=f, fill=rgba(op["fill"]), anchor=anchor, stroke_width=stroke,
                              stroke_fill=(0, 0, 0, 255), align="center")
            bb = dr.multiline_textbbox((x, y), s, font=f, anchor=anchor, stroke_width=stroke)
        else:
            dr.text((x, y), s, font=f, fill=rgba(op["fill"]), anchor=anchor, stroke_width=stroke,
                    stroke_fill=(0, 0, 0, 255))
            bb = dr.textbbox((x, y), s, font=f, anchor=anchor, stroke_width=stroke)
        self.text_boxes.append((op["tag"], bb, op.get("box"), s))


def _paste_clip(layer, img, x, y):
    """alpha_composite of img at (x, y), clipped to the layer on every side (nothing if it is outside)."""
    cx0, cy0 = max(0, -x), max(0, -y)
    cx1, cy1 = min(img.width, layer.width - x), min(img.height, layer.height - y)
    if cx1 <= cx0 or cy1 <= cy0:
        return
    if (cx0, cy0, cx1, cy1) != (0, 0, img.width, img.height):
        img = img.crop((cx0, cy0, cx1, cy1))
    layer.alpha_composite(img, (max(0, x), max(0, y)))


def fill_crop(img, w, h):
    """img scaled to cover w x h, cropped evenly."""
    k = max(w / float(img.width), h / float(img.height))
    sw, sh = max(1, int(img.width * k + 0.5)), max(1, int(img.height * k + 0.5))
    im = img.resize((sw, sh), Image.BILINEAR)
    x0, y0 = (sw - w) // 2, (sh - h) // 2
    return im.crop((x0, y0, x0 + w, y0 + h))


class CanvasRenderer:
    """Without Pillow: the same display list as canvas items (panels stippled, text with a black outline)."""

    def __init__(self, canvas):
        self.c = canvas
        self.text_boxes = []

    def render(self, ops):
        c = self.c
        c.delete("hud")
        for op in ops:
            k, tag = op["k"], ("hud", op.get("tag") or "x")
            if k in ("panel", "rect"):
                x0, y0, x1, y1 = op["r"]
                if op.get("fill"):
                    c.create_rectangle(x0, y0, x1, y1, fill=op["fill"], outline="", tags=tag,
                                       stipple="gray50" if op.get("alpha", 255) < 200 else "")
                if op.get("outline"):
                    c.create_rectangle(x0, y0, x1, y1, outline=op["outline"], width=op.get("w", 1), tags=tag)
            elif k == "line":
                c.create_line(*op["pts"], fill=op["fill"], width=op.get("w", 2), tags=tag)
            elif k == "poly" and len(op["pts"]) >= 6:
                c.create_polygon(*op["pts"], fill=op.get("fill") or "", outline=op.get("outline") or "", tags=tag,
                                 stipple="gray50" if op.get("alpha", 255) < 200 else "")
            elif k == "oval":
                c.create_oval(*op["r"], fill=op.get("fill") or "", outline=op.get("outline") or "",
                              width=op.get("w", 1), tags=tag)
            elif k == "text" and op["s"]:
                size = op["size"]
                if op.get("max_w"):
                    while size > 8 and measure(op["s"], size, op["bold"]) > op["max_w"]:
                        size -= 1
                fnt = theme.tkfont("label", size, bold=op["bold"])
                anchor = "center" if op["anchor"] == "c" else op["anchor"]
                x, y = op["xy"]
                for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
                    c.create_text(x + dx, y + dy, text=op["s"], fill=theme.BLACK, font=fnt, anchor=anchor, tags=tag)
                c.create_text(x, y, text=op["s"], fill=op["fill"], font=fnt, anchor=anchor, tags=tag)


# --- the window -------------------------------------------------------------------------------------------------
class DriverScreen:
    def __init__(self, app, geometry=None):
        self.app = app
        self.w = tk.Toplevel(app.root, bg=BG)
        self.w.title(T("drv.title"))
        self.w.geometry(geometry or "1600x900")
        self.w.protocol("WM_DELETE_WINDOW", app.close_driver)
        self.photos = {}
        self.question = None                    # (text, on_yes, on_no, deadline)
        self.answers = []                       # (text, result) for tests
        self.layout = hud.Layout()
        self.hud = hud.Hud(self.layout)
        self.cv = tk.Canvas(self.w, bg=BG, highlightthickness=0)
        self.cv.pack(fill="both", expand=True)
        self.status = self.front = self.cv      # old names (tests / gateway tests look for tags here)
        self.map = tk.Canvas(self.w, bg=theme.BLACK, highlightthickness=1, highlightbackground=theme.C["line"])
        self.chartview = mapview.ChartView(app.charts)
        self.renderer = PilRenderer() if Image is not None else CanvasRenderer(self.cv)
        self.ops, self.hits = [], []
        self.pic = None                         # (x0, y0, w, h) of the front picture in the window
        self.sog_hist = collections.deque(maxlen=60)
        self.leg = {}                           # active waypoint -> where the leg started (no program copy)
        self.mission_t0 = None
        self.depth_cache = (0.0, None)
        self.render_ms = 0.0
        self.last_frame = None                  # the last composed picture (screenshots / tests)
        self.drag = None
        self._map_t = 0.0
        self.vis_overlay = bool(settings.get("vision_overlay", True))
        self.vis_opacity = int(settings.get("vision_opacity", 1)) % 3
        # captain -> driver question box
        QB = theme.C["panel"]
        self.qbox = tk.Frame(self.w, bg=QB, highlightbackground=YEL, highlightthickness=4)
        self.qhead = tk.Label(self.qbox, text="", font=F_HUD, bg=QB, fg=YEL)
        self.qhead.pack(padx=20, pady=(12, 0))
        self.qtext = tk.Label(self.qbox, text="", font=F_MID, bg=QB, fg=theme.C["fg"], wraplength=760, justify="center")
        self.qtext.pack(padx=20, pady=(6, 6))
        self.qtime = tk.Label(self.qbox, text="", font=F_HUD, bg=QB, fg=YEL)
        self.qtime.pack()
        qb = tk.Frame(self.qbox, bg=QB)
        qb.pack(pady=14)
        self.q_yes = tk.Button(qb, text=T("drv.accept"), width=10, command=lambda: self.answer(True),
                               **dict(theme.button_opts("ok"), font=F_MID))
        self.q_yes.pack(side="left", padx=12)
        self.q_no = tk.Button(qb, text=T("drv.reject"), width=10, command=lambda: self.answer(False),
                              **dict(theme.button_opts("no"), font=F_MID))
        self.q_no.pack(side="left", padx=12)
        w = self.w
        for k in ("<F12>", "<Control-Tab>"):
            w.bind(k, lambda e: app.swap_screens())
        keys = {"<space>": "ack", "a": "ack", "A": "ack", "n": "night", "N": "night", "d": "declutter",
                "D": "declutter", "v": "scale", "V": "scale", "l": "lang", "L": "lang", "e": "edit", "E": "edit",
                "g": "ar_all", "G": "ar_all", "o": "vis_overlay", "O": "vis_overlay", "<F11>": "fullscreen",
                "<Escape>": "escape"}
        for k, act in keys.items():
            w.bind(k, lambda e, act=act: self.action(act, e))
        self.cv.bind("<ButtonPress-1>", self.on_press)
        self.cv.bind("<B1-Motion>", self.on_motion)
        self.cv.bind("<ButtonRelease-1>", self.on_release)
        self.map.bind("<ButtonPress-1>", lambda e: self.on_press(e, child=self.map))
        self.map.bind("<B1-Motion>", lambda e: self.on_motion(e, child=self.map))
        self.map.bind("<ButtonRelease-1>", lambda e: self.on_release(e, child=self.map))
        w.bind("<Configure>", lambda e: app.layout_changed())
        monitors.bind_keys(w, app)
        i18n.on_change(self.lang_changed)
        theme.on_change(self.theme_changed)
        self.tick()

    # --- settings / actions ---------------------------------------------------------------------------------
    def lang_changed(self):
        try:
            self.w.title(T("drv.title"))
            self.q_yes.configure(text=T("drv.accept"))
            self.q_no.configure(text=T("drv.reject"))
        except tk.TclError:
            raise RuntimeError("driver window gone")             # i18n drops this listener

    def theme_changed(self, old):
        """Dark / light switched on either screen: the question box follows, the HUD redraws."""
        try:
            theme.retint(self.qbox, old)
            self.map.configure(highlightbackground=theme.C["line"])
        except tk.TclError:
            raise RuntimeError("driver window gone")

    def toggle_overlay(self):
        self.vis_overlay = not self.vis_overlay
        settings.put("vision_overlay", self.vis_overlay)

    def next_opacity(self):
        self.vis_opacity = (self.vis_opacity + 1) % 3
        settings.put("vision_opacity", self.vis_opacity)

    def action(self, act, ev=None):
        """A click on the HUD or a key. Never a driving command."""
        a = self.app
        if act == "ack":
            a.master_ack()
        elif act == "save":
            a.video_save("driver")
        elif act == "lang":
            i18n.set_lang(i18n.other())
        elif act == "night":
            settings.put("night", not settings.get("night"))
            if a.sound:
                a.sound.set_quiet(bool(settings.get("night")))
        elif act == "declutter":
            cur = settings.get("declutter") or "FULL"
            settings.put("declutter", hud.DECLUTTER[(hud.DECLUTTER.index(cur) + 1) % 3]
                         if cur in hud.DECLUTTER else "FULL")
        elif act == "scale":
            settings.put("video_scale", "fit" if settings.get("video_scale") == "fill" else "fill")
        elif act == "edit":
            self.hud.edit = not self.hud.edit
            if not self.hud.edit:
                self.layout.save()
        elif act == "edit_reset":
            self.layout.reset()
        elif act == "ar_all":
            ar = dict(settings.get("ar") or {})
            on = not any(ar.values())
            settings.put("ar", {k: on for k in (ar or hudar_keys())})
        elif act == "ar_menu":
            self.ar_menu(ev)
        elif act == "vis_overlay":
            self.toggle_overlay()
        elif act == "vision_clear":
            a.vision_send({"t": "vision_clear"})
        elif act == "remote_req":
            if a.manual_allowed():
                a.remote_request()
        elif act == "maneuver_req":
            if a.manual_allowed():
                a.maneuver_request()
        elif act == "release":
            a.remote_release()
        elif act.startswith("relay:"):
            n = int(act[6:])                    # a relay button (CLAUDE.md O): a request, the AURIX decides
            a.relay_press(n)
            if any(r["n"] == n and r["hold"] for r in a.relay_view()[1]):
                self.relay_hold = n             # momentary: let go at the mouse release
        elif act == "fullscreen":
            self.w.attributes("-fullscreen", not self.w.attributes("-fullscreen"))
        elif act == "escape":
            if self.hud.edit:
                self.action("edit")
            elif self.w.attributes("-fullscreen"):
                self.w.attributes("-fullscreen", False)
        self.draw()

    def ar_menu(self, ev):
        """The AR / overlay switches (each one on / off) + HUD colour + obstacle overlay."""
        m = tk.Menu(self.w, tearoff=0)
        ar = dict(settings.get("ar") or {})
        for k in hudar_keys():
            v = tk.BooleanVar(value=ar.get(k, True))
            m.add_checkbutton(label=T("ar." + k), variable=v,
                              command=lambda k=k, v=v: settings.put("ar", dict(settings.get("ar") or {}, **{k: v.get()})))
        m.add_separator()
        vo = tk.BooleanVar(value=self.vis_overlay)
        m.add_checkbutton(label=T("hud.vis_overlay"), variable=vo, command=self.toggle_overlay)
        m.add_command(label=T("hud.vis_opacity", pc=(25, 50, 75)[(self.vis_opacity + 1) % 3]),
                      command=self.next_opacity)
        m.add_separator()
        for col in hud.HUD_COLOURS:
            m.add_command(label=T("hud.colour_" + col), command=lambda col=col: settings.put("hud_colour", col))
        x = ev.x_root if ev is not None and hasattr(ev, "x_root") else self.w.winfo_rootx() + 100
        y = ev.y_root if ev is not None and hasattr(ev, "y_root") else self.w.winfo_rooty() + 100
        try:
            m.tk_popup(x, y)
        finally:
            m.grab_release()

    # --- mouse: buttons; in the edit mode drag / resize / show-hide -------------------------------------------
    def on_press(self, ev, child=None):
        x, y = ev.x, ev.y
        if child is not None:
            x, y = x + child.winfo_x(), y + child.winfo_y()
        if self.hud.edit:
            for r, act in self.hits:
                if act in ("edit", "edit_reset") and r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                    return self.action(act, ev)
            name, what = self.hud.edit_target(x, y)
            if what == "eye":
                self.layout.toggle(name)
                self.layout.save()
                return self.draw()
            self.drag = (name, what, x, y) if name else None
            return
        for r, act in reversed(self.hits):
            if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                return self.action(act, ev)

    def on_motion(self, ev, child=None):
        if not (self.hud.edit and self.drag):
            return
        x, y = ev.x, ev.y
        if child is not None:
            x, y = x + child.winfo_x(), y + child.winfo_y()
        name, what, x0, y0 = self.drag
        W, H = max(1, self.cv.winfo_width()), max(1, self.cv.winfo_height())
        (self.layout.move if what == "move" else self.layout.resize)(name, (x - x0) / W, (y - y0) / H)
        self.drag = (name, what, x, y)
        self.draw()

    def on_release(self, ev, child=None):
        n, self.relay_hold = getattr(self, "relay_hold", None), None
        if n is not None:
            self.app.relay_release(n)
        if self.drag:
            self.drag = None
            self.layout.save()

    def destroy(self):
        if self.question:
            self.answer(False, why="driver screen closed")
        self.w.destroy()

    # --- captain -> driver questions ----------------------------------------------------
    def ask(self, text, on_yes, on_no, timeout_s, tag=None, head=None):
        if self.question:                       # one at a time: the older one is rejected
            self.answer(False, why="replaced by a newer request")
        self.question = (text, on_yes, on_no, time.time() + timeout_s)
        self.qtag = tag
        self.qhead.configure(text=head or T("drv.head_captain"))
        self.qtext.configure(text=text)
        self.qbox.place(relx=0.5, rely=0.45, anchor="center")
        self.qbox.lift()
        if self.app.sound:
            self.app.sound.feedback("fb.question")
        else:
            self.w.bell()

    def cancel(self, tag):
        """The gateway closed the question (answered elsewhere / timed out there)."""
        if self.question and tag is not None and getattr(self, "qtag", None) == tag:
            self.answer(False, why="done elsewhere")

    def answer(self, yes, why=None):
        if not self.question:
            return
        text, on_yes, on_no, _dl = self.question
        self.question = None
        self.qbox.place_forget()
        self.answers.append((text, "accepted" if yes else (why or "rejected")))
        (on_yes if yes else on_no)(why or ("accepted" if yes else "rejected by the driver"))

    # --- the picture ----------------------------------------------------------------------------------
    def tick(self):
        try:
            t0 = time.time()
            self.draw()
            self.render_ms = 0.8 * self.render_ms + 0.2 * (time.time() - t0) * 1000.0
        except tk.TclError:
            return
        except Exception:
            self.w.after(500, self.tick)            # keep the HUD alive, but report the error
            raise
        self.w.after(200 if self.render_ms > 120 else 100, self.tick)

    def frame_of(self, name):
        """The newest picture of a camera as a Pillow image (None without picture)."""
        vs = getattr(self.app, "vstreams", {}).get(name)
        data = vs.ppm() if vs else None
        if not data or Image is None:
            return None
        try:
            return Image.open(io.BytesIO(data)).convert("RGB")
        except (OSError, ValueError):
            return None

    def snapshot(self):
        """Every value the HUD shows, worked out here (hud.py only draws)."""
        a, now = self.app, time.time()
        d = a.t
        gw = a.gw_status or {}
        s = hud.Snapshot(t=d, gw=gw, lamps=a.lamps, flash=alerts.flash(now),
                         night=bool(settings.get("night")), declutter=settings.get("declutter") or "FULL",
                         hud_colour=settings.get("hud_colour") or "green",
                         video_scale=settings.get("video_scale") or "fill",
                         ar_on=any((settings.get("ar") or {}).values()), have_charts=bool(a.charts))
        s["age_s"] = now - a.t_rx if a.t_rx else None
        me = [st for st in gw.get("stations", []) if st.get("id") == gw.get("you")]
        s["latency_ms"] = me[0].get("rtt_ms") if me else None
        no_data = bool(a.link is not None and a.t_rx and now - a.t_rx > 3.0)       # as the captain screen
        a.lamps.update(alerts.bits_of(d), no_data=no_data)          # both alarm words
        s["alarm_rows"] = a.lamps.rows()                        # [(text, level, steady)] most severe first
        vs = getattr(a, "vstreams", {})
        s["mirrors"] = {k: {"have": vs.get(cam) is not None, "live": vs.get(cam) is not None and not vs[cam].frozen()}
                        for k, cam in (("mirror_l", "REAR-LEFT"), ("mirror_r", "REAR-RIGHT"))}
        # MANEUVER (dock / trailer) or going astern: the rear cameras big (CLAUDE.md C)
        s["rear_big"] = bool(d is not None and (d["mode"] == proto.MODE_MANEUVER or
                                                (d["mode"] in (proto.MODE_REMOTE, proto.MODE_MANUAL) and d["gear"] == 2)))
        s["notices"] = self.notices(d, gw)
        s["manual_ok"] = a.manual_allowed()                     # REMOTE MANUAL / MANEUVER buttons (captain + licence)
        active, rows = a.relay_view()                           # the relays with "show_hud" (CLAUDE.md O)
        hud_rows = [dict(r, click=active and r["group"] != "engine" and r["state"] != "unknown")
                    for r in rows if r.get("show_hud")]
        s["relay_lines"] = [ln for _g, ln in a.relay_lines(hud_rows)]
        s["mv_ask"] = None
        if d is not None and "rjoy_lever1000" in d and d.get("rjoy_age_ms", 99999) < 1000:
            lv = d["rjoy_lever1000"] / 1000.0                   # the captain's lever: REV / N / FWD zone
            s["mv_ask"] = 1 if lv > d.get("nz_hi", 0.55) else (2 if lv < d.get("nz_lo", 0.45) else 0)
        v = gw.get("vision")
        s["vision_cap"] = bool(v and v.get("cap_kn") is not None)
        if v:
            txt = T("hud.vision", state=T("vis." + v["state"]) if i18n.has("vis." + v["state"], "en") else v["state"],
                    fps="%.1f" % v["fps"] if v["state"] in ("ACTIVE", "DEGRADED") else "")
            if v.get("cap_kn") is not None:
                txt += "  " + T("hud.vision_cap", kn="%.0f" % v["cap_kn"]) + (T("hud.vision_held")
                                                                                 if v.get("cap_held") else "")
            s["vision"] = (txt, "good" if v["state"] == "ACTIVE" and v.get("cap_kn") is None else
                           "caution" if v["state"] in ("ACTIVE", "DEGRADED", "PAUSED") else "grey")
        r = gw.get("rec")
        s["rec"] = None if not (r and r.get("enabled")) else any(c["state"] == "REC" for c in r.get("cameras", []))
        s["clock"] = time.strftime("%H:%M:%S")
        if d is not None and d["mode"] != proto.MODE_STANDBY and self.mission_t0 is None:
            self.mission_t0 = now
        s["mission"] = hud.fmt_s(now - self.mission_t0) if self.mission_t0 else None
        if d is None:
            return s
        self.sog_hist.append((now, d["est_sog"]))
        s["sog_trend"] = hud.trend(list(self.sog_hist))
        s["shallow_limit"] = a.shallow_limit()
        if d["pos_source"] and a.charts:
            if now - self.depth_cache[0] > 1.0:
                self.depth_cache = (now, a.charts.depth(d["est_lat"], d["est_lon"]))
            s["depth_m"] = self.depth_cache[1]
        s["wp"], s["xte_m"], s["xte_scale"] = self.waypoint_info(d, now)
        rows = a.ais_rows_now() if hasattr(a, "ais_tree") else []
        s["ais_n"] = len(rows)
        s["threat"] = rows[0] if rows else None
        if "slam_peak" in d:
            hold = max([p for t, p in list(a.slam_hist)[-200:] if d["t_ms"] - t <= 10000] or [d["slam_peak"]])
            s["gmeter"] = {"peak": d["slam_peak"], "hold": hold, "limit": d["slam_limit"],
                           "yellow": d["slam_limit"] * d.get("slam_yellow_pc", 70) / 100.0}
        return s

    def waypoint_info(self, d, now):
        if not d.get("wp_set") or not d.get("pos_source"):
            return None, None, None
        dist, brg = hud.dist_bearing(d["est_lat"], d["est_lon"], d["wp_lat"], d["wp_lon"])
        v = d["est_sog"] * 0.514444
        ttg = dist / v if v > 0.3 else None
        wp = {"n": d["auto_wp"] + 1, "of": d["prog_n"], "dist_m": dist, "brg": brg, "ttg_s": ttg,
              "eta": time.strftime("%H:%M", time.localtime(now + ttg)) if ttg is not None else None}
        if d["mode"] != proto.MODE_AUTO:
            return wp, None, None
        key = (round(d["wp_lat"], 6), round(d["wp_lon"], 6))
        wps = (self.app.prog or {}).get("wps") or []
        if len(wps) == d["prog_n"] and d["auto_wp"] >= 1:
            a_ = wps[d["auto_wp"] - 1]
            start = (a_["lat"], a_["lon"])
        else:
            start = self.leg.setdefault(key, (d["est_lat"], d["est_lon"]))
        radius = wps[d["auto_wp"]]["radius"] if len(wps) == d["prog_n"] and d["auto_wp"] < len(wps) else 25.0
        return wp, hud.xte(d["est_lat"], d["est_lon"], start[0], start[1], d["wp_lat"], d["wp_lon"]), \
            max(50.0, 2.0 * radius)

    def notices(self, d, gw):
        """Banners under the alarm bar: [(tag, text, colour key)]."""
        a = self.app
        out = []
        if a.single_op:
            out.append(("singleop", T("drv.single_op"), theme.NOTICE["single_op"]))
        if getattr(a, "boat", None) is not None and not getattr(a, "boat_ok", True):
            out.append(("boatconfirm", T("boat.banner", name=a.boat.name), "caution"))
        if d is not None:
            ll = d.get("ll_state", 0)
            if ll == 2:
                out.append(("killed", T("cap.ll_killed"), "warn"))
            elif ll == 1:
                out.append(("linklost", T("cap.ll_lost", near=T("cap.ll_near") if d.get("near_shore") else
                                          T("cap.ll_far", m=d.get("coast_dist_m") or "?", s="%.0f" % d.get("ll_left_s", 0))),
                            "warn"))
            if d.get("profile") == proto.PROFILE_UNCREWED:
                out.append(("uncrewed", T("drv.uncrewed"), theme.NOTICE["uncrewed"]))
            if d["mode"] == proto.MODE_MANEUVER:
                out.append(("maneuver", T("drv.maneuver"), "caution"))
            if d.get("confirm_kind"):
                out.append(("confirm", T("cap.confirm_banner", what=T("confirm.%d" % min(
                    d["confirm_kind"], len(proto.CONFIRM_KINDS) - 1)), s="%.0f" % d["confirm_left_s"]), "caution"))
            if d.get("first_test"):
                out.append(("first_test", T("drv.first_test"), theme.NOTICE["first_test"]))
            if d.get("nv_fault"):
                out.append(("nv_fault", T("nv.fault"), "caution"))
            elif d.get("nv_reset"):
                out.append(("nv_reset", T("nv.reset"), "caution"))
            if d.get("batt_no_data"):                   # a notice, never an alarm (CLAUDE.md J)
                out.append(("batt_no_data", T("batt.no_data"), "caution"))
            gs = d.get("gen_state", 0)                  # generator mode (CLAUDE.md K)
            if gs == 2:
                out.append(("gen_starting", T("gen.starting"), "caution"))
            elif gs == 6:
                out.append(("gen_fault", T("gen.fault"), "caution"))
            elif gs == 7:
                out.append(("gen_done", T("gen.done"), "caution"))
            if d["mode"] == proto.MODE_REMOTE and not self.front_live:
                out.append(("nofront", T("drv.no_front"), "warn"))
            if d.get("pickup") and d["mode"] in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK):
                tgt = d["pickup_target"]
                out.append(("pickup", T("drv.pickup", to="%.0f %%" % (100 * tgt) if tgt is not None else
                                        T("cap.neutral")), "caution"))
        av = gw.get("avoid") or {}
        sh = av.get("shallow")
        if sh:
            out.append(("shallow", T("drv.shallow", s="%.0f" % sh["in_s"], depth=T("hud.no_data")
                                      if sh["depth"] is None else "%.1f m" % sh["depth"]), "warn"))
        danger = [t for t in av.get("threats") or [] if t.get("level") == "danger"]
        if danger or av.get("state") == "DRIVER DECISION":
            t = danger[0] if danger else None
            txt = T("drv.collision", name=t["name"], cpa="%.2f" % t["cpa_nm"], tcpa="%.1f" % t["tcpa_min"],
                    brg="%03.0f" % t["bearing"]) if t else ""
            if av.get("state") == "DRIVER DECISION":
                txt = (T("drv.decision") + "   " + txt).strip()
            elif av.get("state") == "AVOID" and av.get("heading") is not None:
                txt += "   " + T("drv.avoiding", hdg="%03.0f" % av["heading"])
            out.append(("collision", txt, "warn"))
        if a.front_sel.notice:
            out.append(("frontsel", video.notice_text(a.front_sel), "caution"))
        return out

    def draw(self):
        if self.question:
            left = self.question[3] - time.time()
            if left <= 0:
                self.answer(False, why="no answer from the driver (timeout)")
            else:
                self.qtime.configure(text=T("drv.q_left", s="%.0f" % math.ceil(left)))
        a = self.app
        W, H = max(200, self.cv.winfo_width()), max(150, self.cv.winfo_height())
        vs = getattr(a, "vstreams", {})
        f1, f2 = vs.get("FRONT-1"), vs.get("FRONT-2")
        use = a.front_sel.update(f1, f2) if (f1 or f2) else "FRONT-1"
        fvs = vs.get(use)
        self.front_live = fvs is not None and not fvs.frozen()
        self.hud.measure = measure
        s = self.snapshot()
        ops, hits = self.hud.build(W, H, s)
        scale = s.video_scale or "fill"
        under = []                                      # AR + vision: under the HUD panels
        pic = None
        img = self.frame_of(use) if Image is not None else None
        if img is not None:
            pic = scale_rect(img.width, img.height, W, H, scale)
        else:
            iw, ih = (fvs.w, fvs.h) if fvs is not None and hasattr(fvs, "w") else (16, 9)
            pic = scale_rect(iw, ih, W, H, scale)
        self.pic = pic
        if self.front_live:
            under += self.vision_ops(pic)
        if hudar is not None and a.t is not None and s.declutter != "MINIMAL":
            under += hudar.build(self, s, pic, W, H)
        if not self.front_live:
            cam = T("hud.video_frozen") if fvs is not None else T("hud.no_video")
            under.append({"k": "rect", "r": (W / 2 - W * 0.16, H * 0.30 - H * 0.035, W / 2 + W * 0.16,
                                              H * 0.30 + H * 0.035), "tag": "frozen", "fill": theme.FROZEN,
                          "outline": None, "w": 1, "alpha": 210})
            under.append({"k": "text", "xy": (W / 2, H * 0.30), "s": cam, "tag": "frozen",
                          "size": max(14, int(H * 0.04)), "bold": True, "fill": theme.WHITE, "anchor": "c",
                          "max_w": W * 0.3, "box": None})
        elif use != "FRONT-1":
            under.append({"k": "text", "xy": (W / 2, H * 0.27), "s": use, "tag": "front_name",
                          "size": max(10, int(H * 0.02)), "bold": True, "fill": theme.YELLOW, "anchor": "c",
                          "max_w": None, "box": None})
        self.ops, self.hits = under + ops, hits
        if Image is not None:
            self.render_pil(W, H, img, scale, s)
        else:
            self.render_canvas(W, H, use, fvs)
        self.place_minimap(s)

    def render_pil(self, W, H, img, scale, s):
        if img is not None:
            if scale == "fill":
                base = fill_crop(img, W, H)
            else:
                base = Image.new("RGB", (W, H), (0, 0, 0))
                x0, y0, w, h = scale_rect(img.width, img.height, W, H, "fit")
                base.paste(img.resize((max(1, int(w)), max(1, int(h))), Image.BILINEAR), (int(x0), int(y0)))
        else:
            base = Image.new("RGB", (W, H), (8, 16, 24))
        k = hud.palette(s.hud_colour or "green", bool(s.night))["video"]
        if k < 1.0:
            base = base.point(lambda v: int(v * k))
        images = {}
        for name, cam in (("mirror_l", "REAR-LEFT"), ("mirror_r", "REAR-RIGHT")):
            im = self.frame_of(cam)
            if im is not None:
                im = im.transpose(Image.FLIP_LEFT_RIGHT)          # a mirror
                images[name] = im.point(lambda v: int(v * k)) if k < 1.0 else im
        out = self.renderer.render(base, self.ops, images)
        self.last_frame = out
        self.photos["hud"] = ImageTk.PhotoImage(out)
        self.cv.delete("all")
        self.cv.create_image(0, 0, image=self.photos["hud"], anchor="nw", tags="hud")
        self.tag_items()

    def render_canvas(self, W, H, use, fvs):
        c = self.cv
        c.delete("all")
        data = fvs.ppm() if fvs else None
        if data:
            self.photos["front"] = tk.PhotoImage(data=data, format="ppm")
            c.create_image(W / 2, H / 2, image=self.photos["front"], anchor="center")
        for name, cam in (("mirror_l", "REAR-LEFT"), ("mirror_r", "REAR-RIGHT")):
            mvs = getattr(self.app, "vstreams", {}).get(cam)
            md = mvs.ppm() if mvs else None
            if md and name not in self.layout.hidden:
                x0, y0, x1, y1 = self.layout.px(name, W, H)
                self.photos[name] = tk.PhotoImage(data=flip_ppm(md), format="ppm")
                c.create_image((x0 + x1) / 2, (y0 + y1) / 2, image=self.photos[name], anchor="center")
        self.renderer.render(self.ops)
        self.tag_items()

    def tag_items(self):
        """Invisible canvas items named like the overlays (tests and the gateway test look for them)."""
        seen = set()
        for op in self.ops:
            t = op.get("tag")
            if t and t not in seen:
                seen.add(t)
                self.cv.create_line(-10, -10, -9, -9, tags=(t, "tagmark"), fill="")

    # --- for the tests -----------------------------------------------------------------------------------
    def items(self, tag):
        return [op for op in self.ops if op.get("tag") == tag]

    def texts(self, tag):
        return [op["s"] for op in self.ops if op.get("tag") == tag and op["k"] == "text"]

    def has(self, tag):
        return any(op.get("tag") == tag for op in self.ops)

    # --- obstacle detection on the front picture -------------------------------------------------------------
    STIPPLE = (64, 128, 190)                     # mask opacity 25 / 50 / 75 %

    def vision_ops(self, pic):
        """The obstacle mask (yellow = obstacle, blue = water; sky not drawn) and boxes with bearing + ROUGH
        distance over the front picture. pic = (x0, y0, w, h) of the picture in the window."""
        v = (self.app.gw_status or {}).get("vision") or {}
        x0, y0, iw, ih = pic
        out = []
        mk = v.get("mask")
        if self.vis_overlay and mk and mk.get("b64") and v.get("state") in ("ACTIVE", "DEGRADED"):
            cells = decode_mask(mk["b64"], mk["w"], mk["h"])
            alpha = self.STIPPLE[self.vis_opacity]
            if settings.get("night"):
                alpha //= 2                             # night: the mask dimmed too
            if Image is not None:
                im = Image.new("RGBA", (mk["w"], mk["h"]), (0, 0, 0, 0))
                px = im.load()
                for i, k in enumerate(cells):
                    if k == 0:
                        px[i % mk["w"], i // mk["w"]] = rgba(theme.VISION["obstacle"], alpha)
                    elif k == 1:
                        px[i % mk["w"], i // mk["w"]] = rgba(theme.VISION["water"], alpha)
                out.append({"k": "mask", "img": im, "r": pic, "tag": "vmask"})
            else:
                cw, ch = iw / mk["w"], ih / mk["h"]
                for r in range(mk["h"]):
                    row = cells[r * mk["w"]:(r + 1) * mk["w"]]
                    c0 = 0
                    while c0 < mk["w"]:
                        k = row[c0]
                        c1 = c0
                        while c1 < mk["w"] and row[c1] == k:
                            c1 += 1
                        if k in (0, 1):
                            out.append({"k": "rect", "r": (x0 + c0 * cw, y0 + r * ch, x0 + c1 * cw, y0 + (r + 1) * ch),
                                        "tag": "vmask", "fill": theme.VISION["obstacle"] if k == 0 else
                                        theme.VISION["water"], "outline": None,
                                        "w": 1, "alpha": alpha})
                        c0 = c1
        for o in v.get("obstacles") or []:
            if not o.get("confirmed") or not o.get("box"):
                continue
            bx = o["box"]
            col = theme.WARN if o.get("in_corridor") else theme.YELLOW
            r = (x0 + bx[0] * iw, y0 + bx[1] * ih, x0 + bx[2] * iw, y0 + bx[3] * ih)
            out.append({"k": "rect", "r": r, "tag": "vbox", "fill": None, "outline": col, "w": 3, "alpha": 255})
            out.append({"k": "text", "xy": (r[0], r[1] - 4), "tag": "vbox", "size": 14, "bold": True, "fill": col,
                        "anchor": "sw", "max_w": None, "box": None,
                        "s": T("drv.obstacle", brg="%03.0f" % o["bearing"],
                               dist="~%d m" % o["dist"] if o.get("dist") else T("hud.far")) +
                        ("  =%s" % o["ais"] if o.get("ais") else "")})
        return out

    def draw_vision(self, cv, w, h, iw, ih):
        """Old entry point (tests): the vision ops for a picture iw x ih centred in w x h."""
        self.ops = self.vision_ops((w / 2 - iw / 2, h / 2 - ih / 2, iw, ih))
        return self.ops

    # --- the mini map ----------------------------------------------------------------------------------------
    def place_minimap(self, s):
        op = next((o for o in self.ops if o["k"] == "minimap"), None)
        if op is None:
            if self.map.winfo_manager():
                self.map.place_forget()
            return
        x0, y0, x1, y1 = op["r"]
        self.map.place(x=int(x0), y=int(y0), width=int(x1 - x0), height=int(y1 - y0))
        now = time.time()
        if now - self._map_t >= 0.45 or self.hud.edit:
            self._map_t = now
            self.draw_map(s, int(x1 - x0), int(y1 - y0))

    def draw_map(self, s=None, w=None, h=None):
        c = self.map
        c.delete("all")
        a = self.app
        w = w or max(50, c.winfo_width())
        h = h or max(50, c.winfo_height())
        d = a.t or {}
        mth = "night" if settings.get("night") else "dark"      # the mini map over the video: never bright
        c.configure(bg=mapview.THEMES[mth]["bg"])
        tr = a.track[-600:]
        if not tr or not d:
            c.create_text(w / 2, h / 2, text=T("map.no_position"), fill=mapview.THEMES[mth]["text"],
                          font=(theme.FAMILY, 10, "bold"))
            return
        la0, lo0 = tr[-1]
        k = math.cos(math.radians(la0)) * 111320.0
        m = 10.0 * 300.0 / max(150.0, w)                # about 3 km wide
        av = (a.gw_status or {}).get("avoid") or {}
        self.chartview.draw(c, la0, lo0, m, w, h, opacity=0.9, ais=a.ais_list(), threats=av.get("threats"),
                            route=av.get("route"), shallow=av.get("shallow"), small=True, theme=mth,
                            vision=((a.gw_status or {}).get("vision") or {}).get("obstacles"))
        P = lambda la, lo: (w / 2 + (lo - lo0) * k / m, h / 2 - (la - la0) * 111320.0 / m)      # noqa: E731
        th = mapview.THEMES[mth]
        wps = (a.prog or {}).get("wps") or []
        if len(wps) >= 2:
            c.create_line(*[v for wp in wps for v in P(wp["lat"], wp["lon"])], fill=th["route"], width=2, dash=(6, 3))
        pts = []
        for la, lo in tr[::max(1, len(tr) // 300)]:
            pts += P(la, lo)
        if len(pts) >= 4:
            c.create_line(*pts, fill=th["track"], width=2)
        if d.get("wp_set"):
            x, y = P(d["wp_lat"], d["wp_lon"])
            x, y = max(5, min(w - 5, x)), max(5, min(h - 5, y))
            c.create_oval(x - 6, y - 6, x + 6, y + 6, outline=th["route"], width=2)
        r = d.get("unc_m", 0) / m
        if r > 2:
            c.create_oval(w / 2 - r, h / 2 - r, w / 2 + r, h / 2 + r, outline=th["warn"] if d.get("pos_source") in
                          (2, 3) else th["good"], dash=(3, 3))
        hd = math.radians(d["heading"])
        c.create_polygon(w / 2 + 10 * math.sin(hd), h / 2 - 10 * math.cos(hd), w / 2 + 6 * math.sin(hd + 2.4),
                         h / 2 - 6 * math.cos(hd + 2.4), w / 2 + 6 * math.sin(hd - 2.4), h / 2 - 6 * math.cos(hd - 2.4),
                         fill=th["own"], outline=theme.BLACK)
        c.create_text(w - 4, 4, text=T("map.km", km="%.0f" % (m * w / 1000.0)), anchor="ne", fill=th["text"],
                      font=(theme.FAMILY, 9, "bold"))


def hudar_keys():
    return hudar.KEYS if hudar is not None else ("horizon", "velocity", "waypoint", "ribbon", "ais", "path", "shallow")


if __name__ == "__main__":
    sys.exit("start the ground station: python app.py")
