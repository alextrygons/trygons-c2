"""Screen 2's trend panel «ΤΑΣΕΙΣ» (step TR2, CLAUDE.md Y, docs/trends_design.md 3 + 4): the drawing and the click of
the graphs. DISPLAY ONLY and READ-ONLY: it gets decoded values from trends.Trends and draws; it holds NO reference to
the station (the narrow TrendView = settings get / put of the `trends_*` keys only + the night flag), imports no
sender and sends nothing (test_trends TG6 - TG9).

    panel = TrendPanel(model, view)
    panel.fill(ov, p, now, fold)        # the items of the overlay panel p (an overlay.Panel)
    panel.escape()                       # Esc: close the readout
"""
import time

import i18n
import theme
import trends as tr

T = i18n.T
KEY_PREFIX = "trends_"
SHOW_KEY, ZOOM_KEY = "trends_show", "trends_zoom"
LINE_W = 1.25
GLOW_W = 3.0
NIGHT_W = 1.25


class TrendView:
    """What the panel may touch of the station: the settings keys `trends_*` and the night flag. Nothing else."""

    def __init__(self, get, put, night):
        self._get, self._put, self._night = get, put, night

    def get(self, key, default=None):
        if not str(key).startswith(KEY_PREFIX):
            raise KeyError("the trend panel reads only trends_* settings: %r" % key)
        v = self._get(key)
        return default if v is None else v

    def put(self, key, value):
        if not str(key).startswith(KEY_PREFIX):
            raise KeyError("the trend panel writes only trends_* settings: %r" % key)
        self._put(key, value)

    def night(self):
        return bool(self._night())


def fold_limit(n_shown):
    """Fold steps: 0 = every chosen graph, then one graph fewer per step down to 2, and the last step = the header only."""
    return max(1, n_shown - 2 + 1)


class TrendPanel:
    def __init__(self, model, view):
        self.model = model
        self.view = view
        self.readout_t = None           # the clicked time (station epoch seconds) or None
        self.choose_open = False
        self.last = {}                  # the last drawing's record, per graph (what the tests and the click read)
        self._plot = {}                 # graph -> (x0, x1, now, window) of the last drawing: click -> time
        self._cache = {}                # (id, gen, window, cols, now bucket) -> runs

    # --- settings ---------------------------------------------------------------------------------------
    def zoom_min(self):
        z = self.view.get(ZOOM_KEY, tr.DEFAULT_ZOOM)
        return z if z in tr.ZOOMS else tr.DEFAULT_ZOOM

    def shown(self):
        s = self.view.get(SHOW_KEY, list(tr.DEFAULT_SHOW))
        out = []
        if isinstance(s, (list, tuple)):
            for g in s:
                if isinstance(g, int) and not isinstance(g, bool) and g in tr.GRAPHS and g not in out:
                    out.append(g)
        out = out[:tr.MAX_SHOWN]
        return out if out else list(tr.DEFAULT_SHOW)

    def set_zoom(self, minutes):
        if minutes in tr.ZOOMS:
            self.view.put(ZOOM_KEY, minutes)

    def toggle_graph(self, g):
        s = self.shown()
        if g in s:
            if len(s) > 1:
                s.remove(g)
        elif len(s) < tr.MAX_SHOWN:
            s.append(g)
            s.sort()
        self.view.put(SHOW_KEY, s)

    def toggle_choose(self):
        self.choose_open = not self.choose_open

    def escape(self):
        was = self.readout_t is not None
        self.readout_t = None
        return was

    # --- text -------------------------------------------------------------------------------------------
    @staticmethod
    def unit(vid):
        u = tr.INFO[vid]["unit"]
        return T("trend.u_min") if u == "/min" else u

    def fmt(self, vid, x):
        if not tr.finite(x):
            return T("trend.no_data")
        v = x % 360.0 if vid == "hdg" else x
        return "%.*f" % (tr.INFO[vid]["dec"], v)

    # --- colours ----------------------------------------------------------------------------------------
    def colour(self, vid, rank):
        """The line colour: its own (theme.TREND); NIGHT = the dim red - main for the first line of a graph, dim for
        the second; the pattern + the label tell them apart."""
        if self.view.night():
            return theme.NIGHT["main"] if rank == 0 else theme.NIGHT["dim"]
        return theme.TREND[vid]

    # --- the overlay panel ------------------------------------------------------------------------------
    def geometry(self, ov):
        """(graph width, graph height) in px for this window: width max(260 px, 0.22 W), height max(3 info lines,
        48 px) - times the Windows scaling."""
        W = ov.W
        w = max(260.0 * ov.dpi, 0.22 * W)
        w = min(w, 0.34 * W)
        lh = ov.measure("Ag", "small")[1]
        h = max(3.0 * lh, 48.0 * ov.dpi)
        return w, h

    def fill(self, ov, p, now, fold=0):
        """Add the content of the trends panel. p.collapsed = the header only (nothing is added then)."""
        p.bg = (theme.TREND_BG, 0.85, theme.TREND_TEXT)
        if p.collapsed:
            return
        shown = self.shown()
        n = max(2, len(shown) - fold) if fold else len(shown)
        n = min(n, len(shown))
        shown = shown[:n]
        zoom = self.zoom_min()
        for m in tr.ZOOMS:
            p.button(T("trend.zoom_n", n=m), lambda m=m: self.set_zoom(m), "normal", on=(m == zoom), level="small",
                     tag="trend_zoom_%d" % m)
        p.button(T("trend.choose"), self.toggle_choose, "info", on=self.choose_open, level="small",
                 tag="trend_choose")
        p.br()
        if self.choose_open:
            for g in sorted(tr.GRAPHS):
                p.button(T("trend.g%d" % g), lambda g=g: self.toggle_graph(g), "normal", on=g in self.shown(),
                         enabled=(g in self.shown() or len(self.shown()) < tr.MAX_SHOWN), level="small",
                         tag="trend_g%d" % g)
                if g % 4 == 0:
                    p.br()
            p.br()
        gw, gh = self.geometry(ov)
        self.last = {}
        for g in shown:
            p.br(gap=2)
            p.custom(gw, gh, lambda c, r, op, g=g: self.draw_graph(ov, c, r, op, g, now, zoom * 60.0),
                     tag="trend_g%d_plot" % g, click=lambda x, y, g=g: self.click(g, x, y))
        p.br()
        if self.readout_t is not None:
            p.badge(self.readout_text(), "", fg=theme.TREND_TEXT, level="small", tag="trend_readout",
                    outline=theme.TREND_TEXT, wrap=gw / max(1.0, ov.W))
            p.br()

    # --- the drawing of one graph -----------------------------------------------------------------------
    def draw_graph(self, ov, c, r, op, g, now, window):
        x0, y0, x1, y1 = r
        c.create_rectangle(x0, y0, x1, y1, fill=theme.TREND_BG, outline=theme.TREND_GRID, tags="ov")
        ids = tr.GRAPHS[g]
        fs_small = ov.size("small")
        font = ov.font(fs_small, False)
        # the right-end labels: value + unit + name (the name only when it fits)
        labels, lw = [], 0.0
        for vid in ids:
            val = self.model.last(vid)
            vtxt = "%s %s" % (self.fmt(vid, val), self.unit(vid)) if tr.finite(val) else T("trend.no_data")
            full = "%s %s" % (vtxt, T("trend." + vid))
            labels.append((vid, vtxt, full))
            lw = max(lw, font.measure(full))
        pad = fs_small * 0.35
        lw = min(lw + pad, (x1 - x0) * 0.5)
        px0, px1 = x0 + pad, x1 - lw - pad
        plot_w = max(8.0, px1 - px0)
        self._plot[g] = (px0, px1, now, window)
        # grid: 3 horizontal lines, a vertical line per quarter of the window
        for k in (1, 2, 3):
            yy = y0 + (y1 - y0) * k / 4.0
            c.create_line(px0, yy, px1, yy, fill=theme.TREND_GRID, tags="ov")
        for k in (1, 2, 3):
            xx = px0 + plot_w * k / 4.0
            c.create_line(xx, y0 + 1, xx, y1 - 1, fill=theme.TREND_GRID, tags="ov")
        rec = {"graph": g, "rect": r, "plot": (px0, px1), "lines": [], "cursor": None}
        night = self.view.night()
        cols = max(1, int(plot_w / 2.0))        # a min + a max per 2 px column: a spike is kept, half the points
        for rank, (vid, vtxt, full) in enumerate(labels):
            col = self.colour(vid, rank)
            dash = theme.TREND_DASH[vid]
            width = NIGHT_W if night else LINE_W
            runs, lo, hi = self.runs(vid, window, now, cols)
            span = tr.INFO[vid]["span"]
            if lo <= hi:
                mid = (lo + hi) / 2.0
                half = max((hi - lo) / 2.0, span / 2.0)
                lo, hi = mid - half, mid + half
            drawn = []
            top, bot = y0 + 3, y1 - 3
            for run in runs:
                pts = []
                for xf, v in run:
                    pts.append((px0 + xf * plot_w, bot - (v - lo) / (hi - lo) * (bot - top)))
                if len(pts) == 1:                        # a lone sample: a short dash, not a line to anywhere
                    pts = [(pts[0][0] - 1.0, pts[0][1]), (pts[0][0] + 1.0, pts[0][1])]
                flat = [q for pt in pts for q in pt]
                if not night:
                    glow = theme.mix(col, theme.TREND_BG, 0.75)
                    c.create_line(*flat, fill=glow, width=GLOW_W, tags="ov")
                kw = {"dash": dash} if dash else {}
                c.create_line(*flat, fill=col, width=width, tags="ov", **kw)
                drawn.append(pts)
            # the label at the line's right end: its row
            ty = y0 + (y1 - y0) * (rank + 0.5) / len(labels)
            room = x1 - px1 - pad
            txt = full if font.measure(full) <= room else vtxt
            c.create_text(x1 - pad, ty, text=txt, anchor="e", fill=col, font=font, tags="ov")
            rec["lines"].append({"id": vid, "colour": col, "dash": dash, "width": width, "runs": drawn,
                                 "label": txt, "lo": lo, "hi": hi})
        if self.readout_t is not None and now - window <= self.readout_t <= now:
            cx = px0 + (self.readout_t - (now - window)) / window * plot_w
            c.create_line(cx, y0 + 1, cx, y1 - 1, fill=theme.TREND_TEXT, tags="ov")
            rec["cursor"] = cx
        op["trend"] = rec
        self.last[g] = rec

    def runs(self, vid, window, now, cols):
        """Decimated runs, cached per (value, samples, window, columns, half-second): the drawing redraws at most 2 Hz
        of new data (design 9)."""
        key = (vid, self.model.gen, window, cols, int(now * 2))
        hit = self._cache.get(key)
        if hit is None:
            if len(self._cache) > 64:
                self._cache.clear()
            hit = self._cache[key] = self.model.segments(vid, window, now, cols)
        return hit

    # --- the click --------------------------------------------------------------------------------------
    def click(self, g, x, y):
        """A click on a graph: the time + every value at that sample; a second click (or Esc) closes it."""
        if self.readout_t is not None:
            self.readout_t = None
            return
        px0, px1, now, window = self._plot.get(g, (0.0, 1.0, time.time(), 600.0))
        f = (x - px0) / max(1.0, px1 - px0)
        f = min(1.0, max(0.0, f))
        self.readout_t = now - window + f * window

    def readout_text(self):
        t = self.readout_t
        s = self.model.at(t) if t is not None else None
        if s is None:
            return T("trend.readout_none")
        head = T("trend.readout", utc=time.strftime("%H:%M:%S", time.gmtime(s["t"])),
                 local=time.strftime("%H:%M:%S", time.localtime(s["t"])),
                 mode=T("mode." + s["mode"]) if s["mode"] else "—")
        parts = [("%s %s %s" % (T("trend." + v), self.fmt(v, s[v]), self.unit(v) if tr.finite(s[v]) else "")).rstrip()
                 for v in tr.IDS]
        return head + "\n" + " · ".join(parts)
