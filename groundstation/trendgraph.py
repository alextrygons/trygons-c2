"""Shared graph geometry of the trend graphs (step TR4, CLAUDE.md Y, docs/trends_design.md): PURE - no tkinter, no
network, no import of anything that can send. Turns a trends.Trends model into lines that a drawing surface only has to
stroke: screen 1's edge strip (hud.py ops), the debrief (logview.py) and the academy replays (Tk canvas).

READ-ONLY: model in, geometry out (test_trends TG6 / TG7 / TJ1 prove it).

    ls = lines(model, 9, window_s, now, cols, night)    # -> [{"id", "colour", "dash", "width", "runs", "peak_runs", ...}]
    # runs = [[(x 0..1 left to right, y 0..1 bottom to top) ...] ...]   (a new run after every gap)
"""
import trends as tr
import theme

LINE_W = 1.25
NIGHT_W = 1.25


def acc_scale(model, window_s, now, cols):
    """The three acceleration axes (and their peaks) share ONE symmetric scale so they compare: +-max(0.1 g, the
    largest drawn magnitude)."""
    m = 0.0
    for vid in tr.ACC_IDS + tr.PK_IDS:
        _runs, lo, hi = model.segments(vid, window_s, now, cols)
        if lo <= hi:
            m = max(m, abs(lo), abs(hi))
    m = max(m, tr.INFO["acc_lon"]["span"] / 2.0)
    return -m, m


def colour(vid, rank, night):
    """A value's own colour (theme.TREND); NIGHT = the dim red - main for the first line of a graph, dim after it."""
    if night:
        return theme.NIGHT["main"] if rank == 0 else theme.NIGHT["dim"]
    return theme.TREND[vid]


def _norm(runs, lo, hi):
    span = (hi - lo) or 1.0
    return [[(x, (v - lo) / span) for x, v in run] for run in runs]


def lines(model, g, window_s, now, cols, night=False):
    """The lines of graph g: per value its runs normalised to 0..1 (own scale; the acceleration axes share one),
    colour, dash, the newest value and (acceleration) the newest peak. Missing data = no run (a gap)."""
    ids = tr.GRAPHS[g]
    shared = acc_scale(model, window_s, now, cols) if g == tr.ACC_GRAPH else None
    out = []
    for rank, vid in enumerate(ids):
        runs, lo, hi = model.segments(vid, window_s, now, cols)
        peak = []
        if shared is not None:
            lo, hi = shared
            peak = _norm(model.segments(tr.PK_OF[vid], window_s, now, cols)[0], lo, hi)
        elif lo <= hi:
            mid, half = (lo + hi) / 2.0, max((hi - lo) / 2.0, tr.INFO[vid]["span"] / 2.0)
            lo, hi = mid - half, mid + half
        out.append({"id": vid, "colour": colour(vid, rank, night), "dash": theme.TREND_DASH[vid],
                    "width": NIGHT_W if night else LINE_W, "runs": _norm(runs, lo, hi) if lo <= hi else [],
                    "peak_runs": peak, "last": model.last(vid),
                    "last_pk": model.last(tr.PK_OF[vid]) if vid in tr.PK_OF else float("nan"),
                    "lo": lo, "hi": hi})
    return out


def dash_runs(pts, dash):
    """A polyline [(x, y) ...] cut into dash pieces (on, off, ... in px; () = solid) for a drawing surface without a dash
    option (the HUD's Pillow renderer). Returns a list of point lists."""
    if not dash or len(pts) < 2:
        return [list(pts)]
    out, cur, k, left, on = [], [pts[0]], 0, float(dash[0]), True
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        seg = (dx * dx + dy * dy) ** 0.5
        pos = 0.0
        while seg - pos > left:
            pos += left
            f = pos / seg
            p = (x0 + dx * f, y0 + dy * f)
            if on:
                cur.append(p)
                out.append(cur)
                cur = []
            else:
                cur = [p]
            on = not on
            k = (k + 1) % len(dash)
            left = float(dash[k]) or 0.5
        left -= seg - pos
        if on:
            cur.append((x1, y1))
    if on and len(cur) > 1:
        out.append(cur)
    return [c for c in out if len(c) > 1]
