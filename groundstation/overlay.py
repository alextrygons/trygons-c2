"""The panels of screen 2 (navigation & control): drawn ON the map canvas, so the map shows
through them (Tk widgets can't be transparent). Like the HUD of screen 1 a display list:

    ov = Overlay(canvas)
    p = ov.panel("status", T("ov.status"), corner="nw")      # a panel in a corner
    p.badge(...); p.text(...); p.br(); p.button(label, action) ...
    ov.render(W, H, ...)                                     # place + draw everything, hits for clicks
    ov.click(x, y) -> True if a panel / button took it (else the map gets it)

Panels: semi-transparent (theme.ALPHA), rounded, a header with the title and a collapse mark
(click: collapsed to the header, remembered); they stay in their corner and never cover the
boat (keep-out circle around the centre). Everything is sized from theme.py (fonts by window
size x Windows scaling); if the panels don't fit, the text is made smaller step by step (fit
factor). The result of the last layout (ops: every drawn item with its rectangle and panel) is
what the layout test checks (test_screen2.py): nothing clipped, STANDBY visible, the boat free,
the logo not under a panel.
DISPLAY ONLY: an action is a function of the app (app.py decides what it sends).
"""
import tkinter as tk
import tkinter.font as tkfont

import theme

try:
    from PIL import Image, ImageDraw, ImageTk
except ImportError:
    Image = None


def rrect_pts(x0, y0, x1, y1, r):
    """A rounded rectangle as a smooth canvas polygon."""
    r = max(0.0, min(r, (x1 - x0) / 2.0, (y1 - y0) / 2.0))
    return [x0 + r, y0, x0 + r, y0, x1 - r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y0 + r, x1, y1 - r, x1, y1 - r,
            x1, y1, x1 - r, y1, x1 - r, y1, x0 + r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y1 - r, x0, y0 + r,
            x0, y0 + r, x0, y0]


def wrap_lines(text, font, maxw):
    """Word wrap: the lines of `text` that fit maxw pixels in `font`."""
    out = []
    for para in str(text).split("\n"):
        line = ""
        for wd in para.split(" "):
            t = (line + " " + wd).strip()
            if font.measure(t) <= maxw or not line:
                line = t
            else:
                out.append(line)
                line = wd
        out.append(line)
    return out


def inside(r, box, tol=0.5):
    return r[0] >= box[0] - tol and r[1] >= box[1] - tol and r[2] <= box[2] + tol and r[3] <= box[3] + tol


def overlaps(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def rect_hits_circle(r, c):
    cx, cy, rad = c
    nx, ny = max(r[0], min(cx, r[2])), max(r[1], min(cy, r[3]))
    return (nx - cx) ** 2 + (ny - cy) ** 2 < rad ** 2


class Item:
    """One thing in a panel line: text / badge / button / table / space."""

    def __init__(self, kind, **kw):
        self.kind = kind
        self.__dict__.update(kw)
        self.w = self.h = 0.0


class Panel:
    def __init__(self, ov, key, title, corner, collapsible=True, min_w=0.0):
        self.ov, self.key, self.title, self.corner = ov, key, title, corner
        self.collapsible = collapsible
        self.min_w = min_w
        self.lines = [[]]
        self.line_opts = [{}]
        self.r = None                    # placed rectangle
        self.collapsed = key in ov.collapsed if collapsible else False

    # --- content (one call per thing, br() = a new line) ----------------------------------------------
    def _add(self, it):
        self.lines[-1].append(it)
        return it

    def br(self, stretch=False, gap=None):
        """A new line. stretch: the buttons of the NEXT line share the panel width."""
        if self.lines[-1]:
            self.lines.append([])
            self.line_opts.append({})
        self.line_opts[-1] = {"stretch": stretch, "gap": gap}
        return self

    def text(self, s, level="label", colour=None, bold=None, tag=None, action=None, wrap=None):
        """wrap: word-wrap to this fraction of the window width (at layout time, with the real font)."""
        return self._add(Item("text", s=str(s), level=level, colour=colour, bold=bold, tag=tag, action=action,
                              wrap=wrap))

    def badge(self, s, fill, fg=theme.WHITE, level="value", tag=None, outline=None, wrap=None):
        """fill "" = see-through (outline only: the dark phase of a flashing alarm); wrap like text()."""
        return self._add(Item("badge", s=str(s), level=level, fill=fill, fg=fg, bold=True, tag=tag,
                              outline=outline, wrap=wrap))

    def button(self, s, action, kind="normal", on=False, enabled=True, level="button", tag=None):
        return self._add(Item("button", s=str(s), level=level, action=action, btn=kind, on=on, enabled=enabled,
                              bold=True, tag=tag))

    def space(self, em=1.0):
        return self._add(Item("space", em=em, level="label"))

    def table(self, heads, rows, aligns=None, tag=None, row_actions=None, level="small"):
        """heads: [str] or None (no heading line); rows: [([str], colour)]; aligns: "l" / "r" per column;
        row_actions: [fn or None]."""
        n = len(heads) if heads else (len(rows[0][0]) if rows else 0)
        return self._add(Item("table", heads=heads, rows=rows, aligns=aligns or ["l"] * n, tag=tag,
                              row_actions=row_actions or [None] * len(rows), level=level))


class Overlay:
    def __init__(self, canvas, collapsed=()):
        self.cv = canvas
        self.panels = []
        self.ops = []                    # the last layout: {"k", "r", "panel", "s", "tag"}
        self.hits = []                   # [(rect, fn)]
        self.collapsed = set(collapsed)
        self.k = 1.0                     # fit factor of the last layout (1 = full size)
        self.dpi = 1.0                   # Windows scaling (theme.dpi_factor)
        self.fits = True
        self.problems = []
        self._fonts = {}
        self._imgs = {}
        self._photos = []
        self.on_collapse = None

    # --- fonts ----------------------------------------------------------------------------------------
    def font(self, size, bold, mono=False):
        key = (int(size), bool(bold), mono)
        f = self._fonts.get(key)
        if f is None:
            f = self._fonts[key] = tkfont.Font(root=self.cv, family=theme.MONO if mono else theme.FAMILY,
                                               size=-int(size), weight="bold" if bold else "normal")
        return f

    def size(self, level):
        return theme.px(level, self.W, self.H, self.k)

    def measure(self, s, level, bold=None):
        b = theme.BOLD.get(level, False) if bold is None else bold
        f = self.font(self.size(level), b)
        return max([f.measure(line) for line in s.split("\n")] or [0]), \
            f.metrics("linespace") * max(1, len(s.split("\n")))

    # --- building -------------------------------------------------------------------------------------
    def begin(self, W, H):
        self.panels = []
        self.W, self.H = W, H

    def panel(self, key, title, corner, collapsible=True, min_w=0.0):
        p = Panel(self, key, title, corner, collapsible, min_w)
        self.panels.append(p)
        return p

    def toggle(self, key):
        self.collapsed ^= {key}
        if self.on_collapse:
            self.on_collapse(sorted(self.collapsed))

    # --- layout ---------------------------------------------------------------------------------------
    def _measure_item(self, it):
        lv = it.level
        if it.kind == "space":
            it.w, it.h = self.size("label") * it.em, 1.0
        elif it.kind == "text":
            if it.wrap:
                b = theme.BOLD.get(lv, False) if it.bold is None else it.bold
                it.shown = "\n".join(wrap_lines(it.s, self.font(self.size(lv), b), it.wrap * self.W))
            else:
                it.shown = it.s
            it.w, it.h = self.measure(it.shown, lv, it.bold)
        elif it.kind == "badge":
            it.shown = "\n".join(wrap_lines(it.s, self.font(self.size(lv), True), it.wrap * self.W)) if it.wrap \
                else it.s
            tw, th = self.measure(it.shown, lv, True)
            it.w, it.h = tw + self.size(lv) * 0.9, th + self.size(lv) * 0.25
        elif it.kind == "button":
            fs = self.size(lv)
            tw, th = self.measure(it.s, lv, True)
            it.w, it.h = tw + 2 * fs * theme.BUTTON["pad_x"], max(fs * theme.BUTTON["height"], th + fs * 0.6)
        elif it.kind == "table":
            gap = self.size(lv) * 1.1
            lines = ([it.heads] if it.heads else []) + [r for r, _c in it.rows]
            cols = list(zip(*lines)) if lines else []
            hb = bool(it.heads)
            it.cw = [max(self.measure(str(v), lv, (hb and j == 0) or i == 0)[0] for j, v in enumerate(col))
                     for i, col in enumerate(cols)]
            it.lh = self.measure("Ag", lv)[1] * 1.12
            it.gap = gap
            it.w = sum(it.cw) + gap * max(0, len(it.cw) - 1)
            it.h = it.lh * len(lines)

    def _measure_panel(self, p):
        pad = self.size("label") * theme.PANEL["pad"]
        gap = self.size("label") * theme.PANEL["gap"]
        tw, th = self.measure(p.title, "label", True)
        p.head_h = th + pad
        p.pad, p.gap = pad, gap
        w = tw + 3.2 * self.size("label")             # title + the collapse mark
        h = p.head_h
        p.lsz = []
        if not p.collapsed:
            h += pad * 0.6
            for line, opt in zip(p.lines, p.line_opts):
                if not line:
                    continue
                for it in line:
                    self._measure_item(it)
                lw = sum(it.w for it in line) + gap * (len(line) - 1)
                lh = max(it.h for it in line)
                p.lsz.append((lw, lh))
                w = max(w, lw)
                h += lh + (opt.get("gap") if opt.get("gap") is not None else gap)
            h += pad * 0.6
        p.w, p.h = max(p.min_w, w + 2 * pad), h

    def layout(self, W, H, keepout, extras):
        """Place every panel. extras: {"standby": (w, h), "logo": (w, h), "bar": (w, h)} sizes at fit 1.
        -> rects dict + the problems (empty = everything fits)."""
        g = 8 * theme.unit(W, H)
        cx, cy, kr = keepout
        maxw = min(W * 0.36, cx - kr - 2 * g)
        for p in self.panels:
            self._measure_panel(p)
        probs = []
        rects = {}
        col = {"nw": [], "ne": [], "sw": [], "se": []}
        for p in self.panels:
            col[p.corner].append(p)
            if p.w > maxw:
                probs.append("panel %s too wide: %.0f > %.0f px" % (p.key, p.w, maxw))
        sb_w, sb_h = extras["standby"]
        lg_w, lg_h = extras["logo"]
        left_w = max([p.w for p in col["nw"] + col["sw"]] + [sb_w])
        right_w = max([p.w for p in col["ne"] + col["se"]] + [lg_w])
        left_w, right_w = min(left_w, maxw), min(right_w, maxw)
        sb = (g, H - g - sb_h, g + left_w, H - g)
        lg = (W - g - lg_w, H - g - lg_h, W - g, H - g)
        rects["standby"], rects["logo"] = sb, lg
        y = g
        for p in col["nw"]:
            p.r = (g, y, g + p.w, y + p.h)
            y += p.h + g
        top_left = y
        y = sb[1] - g
        for p in reversed(col["sw"]):
            p.r = (g, y - p.h, g + p.w, y)
            y -= p.h + g
        if y < top_left - g:
            probs.append("left side: the panels don't fit (%.0f px too high)" % (top_left - g - y))
        y = g
        for p in col["ne"]:
            p.r = (W - g - p.w, y, W - g, y + p.h)
            y += p.h + g
        top_right = y
        y = lg[1] - g
        for p in reversed(col["se"]):
            p.r = (W - g - p.w, y - p.h, W - g, y)
            y -= p.h + g
        if y < top_right - g:
            probs.append("right side: the panels don't fit (%.0f px too high)" % (top_right - g - y))
        for p in self.panels:
            rects[p.key] = p.r
            if rect_hits_circle(p.r, keepout):
                probs.append("panel %s covers the boat" % p.key)
        # the free middle: banners at the top (above the boat), the bar + messages at the bottom
        lx = g + left_w + g
        rx = W - g - right_w - g
        rects["middle"] = (lx, g, rx, H - g)
        if rx - lx < extras["bar"][0]:
            probs.append("the middle is too narrow for the map bar (%.0f < %.0f px)" % (rx - lx, extras["bar"][0]))
        return rects, probs

    # --- drawing --------------------------------------------------------------------------------------
    def _bg(self, w, h, fill, alpha, outline, radius):
        key = (int(w), int(h), fill, round(alpha, 2), outline, int(radius))
        img = self._imgs.get(key)
        if img is None and Image is not None:
            if len(self._imgs) > 80:
                self._imgs.clear()
            im = Image.new("RGBA", (max(1, int(w)), max(1, int(h))), (0, 0, 0, 0))
            dr = ImageDraw.Draw(im)
            rgb = tuple(int(fill[i:i + 2], 16) for i in (1, 3, 5))
            orgb = tuple(int(outline[i:i + 2], 16) for i in (1, 3, 5))
            dr.rounded_rectangle((0, 0, int(w) - 1, int(h) - 1), radius=int(radius), fill=rgb + (int(alpha * 255),),
                                 outline=orgb + (220,), width=theme.PANEL["border"])
            img = self._imgs[key] = ImageTk.PhotoImage(im)
        return img

    def draw_panel(self, p):
        c, C = self.cv, theme.C
        x0, y0, x1, y1 = p.r
        img = self._bg(x1 - x0, y1 - y0, C["panel"], theme.panel_alpha(), C["line"], theme.PANEL["radius"])
        if img is not None:
            c.create_image(x0, y0, image=img, anchor="nw", tags="ov")
        else:
            c.create_rectangle(x0, y0, x1, y1, fill=C["panel"], outline=C["line"], stipple="gray75", tags="ov")
        self.ops.append({"k": "panel", "r": p.r, "panel": p.key, "s": p.title, "tag": p.key})
        fs = self.size("label")
        hy = y0 + p.head_h / 2
        c.create_text(x0 + p.pad, hy, text=p.title, anchor="w", fill=C["dim"],
                      font=self.font(fs, True), tags="ov")
        tw = self.measure(p.title, "label", True)[0]
        self.ops.append({"k": "text", "r": (x0 + p.pad, y0, x0 + p.pad + tw, y0 + p.head_h), "panel": p.key,
                         "s": p.title, "tag": p.key + "_head"})
        if p.collapsible:
            c.create_text(x1 - p.pad, hy, text="▸" if p.collapsed else "▾", anchor="e", fill=C["dim"],
                          font=self.font(fs * 1.2, True), tags="ov")
            self.hits.append(((x0, y0, x1, y0 + p.head_h), lambda k=p.key: self.toggle(k)))
        if p.collapsed:
            return
        c.create_line(x0 + p.pad * 0.5, y0 + p.head_h, x1 - p.pad * 0.5, y0 + p.head_h, fill=C["line"], tags="ov")
        y = y0 + p.head_h + p.pad * 0.6
        li = 0
        for line, opt in zip(p.lines, p.line_opts):
            if not line:
                continue
            lw, lh = p.lsz[li]
            li += 1
            x = x0 + p.pad
            extra = 0.0
            if opt.get("stretch"):
                btns = [it for it in line if it.kind == "button"]
                if btns:
                    extra = (x1 - x0 - 2 * p.pad - lw) / len(btns)
            for it in line:
                w = it.w + (extra if it.kind == "button" else 0.0)
                self.draw_item(it, x, y, w, lh, p)
                x += w + p.gap
            y += lh + (opt.get("gap") if opt.get("gap") is not None else p.gap)

    def draw_item(self, it, x, y, w, lh, p):
        c, C = self.cv, theme.C
        fs = self.size(it.level)
        cy = y + lh / 2
        r = (x, y + (lh - it.h) / 2, x + w, y + (lh + it.h) / 2)
        op = {"k": it.kind, "r": r, "panel": p.key, "s": getattr(it, "s", ""), "tag": getattr(it, "tag", None)}
        if it.kind == "text":
            b = theme.BOLD.get(it.level, False) if it.bold is None else it.bold
            c.create_text(x, cy, text=it.shown, anchor="w", fill=it.colour or C["fg"], font=self.font(fs, b), tags="ov")
            if it.action:
                self.hits.append((r, it.action))
        elif it.kind == "badge":
            c.create_polygon(*rrect_pts(r[0], r[1], r[2], r[3], fs * 0.35), smooth=True, fill=it.fill,
                             outline=it.outline or "", width=2 if it.outline else 1, tags="ov")
            c.create_text((r[0] + r[2]) / 2, cy, text=it.shown, fill=it.fg, font=self.font(fs, True), tags="ov",
                          justify="center")
            op["fill"], op["outline"] = it.fill, it.outline
        elif it.kind == "button":
            kind = it.btn
            fill = {"normal": C["btn"], "ok": C["btn_ok"], "no": C["btn_no"], "info": C["btn_info"],
                    "warn": C["btn_warn"], "sim": C["btn_sim"], "standby": theme.WARN}.get(kind, C["btn"])
            fg = theme.WHITE if kind == "standby" else C["btn_fg"]
            if it.on:
                fill, fg = C["accent"], theme.WHITE
            if not it.enabled:
                fill, fg = C["head"], C["faint"]
            c.create_polygon(*rrect_pts(r[0], r[1], r[2], r[3], fs * theme.BUTTON["radius"] * 2), smooth=True,
                             fill=fill, outline=C["line"], tags="ov")
            c.create_text((r[0] + r[2]) / 2, cy, text=it.s, fill=fg, font=self.font(fs, True), tags="ov")
            op["enabled"] = it.enabled
            if it.enabled and it.action:
                self.hits.append((r, it.action))
        elif it.kind == "table":
            yy = r[1]
            cols = len(it.cw)
            hb = bool(it.heads)
            for ri, cells in enumerate(([it.heads] if hb else []) + [rw for rw, _c in it.rows]):
                k = ri - 1 if hb else ri                      # the row index (-1 = the heading)
                colr = C["dim"] if k < 0 else (it.rows[k][1] or C["fg"])
                xx = r[0]
                for ci in range(cols):
                    v = str(cells[ci]) if ci < len(cells) else ""
                    right = it.aligns[ci] == "r"
                    c.create_text(xx + it.cw[ci] if right else xx, yy + it.lh / 2, text=v, anchor="e" if right else "w",
                                  fill=colr,
                                  font=self.font(self.size(it.level), k < 0 or ci == 0), tags="ov")
                    self.ops.append({"k": "cell", "r": (xx, yy, xx + it.cw[ci], yy + it.lh), "panel": p.key, "s": v,
                                     "tag": it.tag})
                    xx += it.cw[ci] + it.gap
                if k >= 0 and it.row_actions[k]:
                    self.hits.append(((r[0], yy, r[2], yy + it.lh), it.row_actions[k]))
                yy += it.lh
        if it.kind != "space":
            self.ops.append(op)

    def render(self, W, H, keepout, extras_fn, draw_extras):
        """Lay out (largest fitting text size), then draw. extras_fn(k) -> sizes of STANDBY / logo / bar at
        fit factor k; draw_extras(rects) draws them (the app: STANDBY, logo, banners, bar)."""
        self.W, self.H = W, H
        self.ops, self.hits = [], []
        k = self.dpi
        rects, probs = None, None
        while True:
            self.k = k
            rects, probs = self.layout(W, H, keepout, extras_fn())
            if not probs or k <= 0.55:
                break
            k = round(k - 0.05, 2)
        self.fits, self.problems = not probs, probs
        self._photos = []
        self.cv.delete("ov")
        for p in self.panels:
            self.draw_panel(p)
        draw_extras(rects)
        self.cv.tag_raise("ov")
        return rects

    def click(self, x, y):
        for r, fn in reversed(self.hits):
            if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                fn()
                return True
        for p in self.panels:
            if p.r and p.r[0] <= x <= p.r[2] and p.r[1] <= y <= p.r[3]:
                return True                         # on a panel, not on a button: nothing for the map
        return False
