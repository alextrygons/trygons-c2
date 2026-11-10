"""The trend graphs for RECORDED data (step TR4, CLAUDE.md Y, docs/trends_design.md 10 TJ2 / TJ3): the debrief
(logview.py, a .TLG) and the academy replays (scenario replay .jsonl). PURE data + a drawing function for a Tk canvas
handed in; no tkinter import, no network, nothing that can send - files and dicts only (the academy imports this).

    model, t_end = from_log(tlg_log)          # the black box at 20 Hz -> the same 2 Hz rings as live
    model, t_end = from_samples(samples)      # academy replay samples ({"t", "mode", "hdg", "sog", "thr", ...})
    rec = draw(canvas, model, (1, 4, 5, 9), t_end, w, h, cursor_t=None)

What a recording holds (the rest is a GAP, never a fake line): a .TLG has speed, throttle, heading, rate of turn,
cross-track (AUTO), impact peak + rate, battery and - from the ACCEL event, once per second - the three acceleration axes;
roll / pitch / chart depth / the link are NOT logged. A replay file holds speed, throttle, heading and, in newer ones,
rate of turn, impacts and acceleration.
"""
import bisect

import theme
import trendgraph
import trends as tr

ACC_WINDOW_S = 1.0          # an ACCEL event covers the second before it
DEFAULT_GRAPHS = (1, 2, 5, 9)
BG = theme.TREND_BG


def _mode_name(modes, m):
    try:
        return modes[int(m)]
    except (TypeError, ValueError, IndexError):
        return ""


def from_log(log, rows=(), modes=()):
    """A Trends model from a tlg.Log (samples) + its ACCEL rows (tlg.accel_rows(log): the caller reads the file format,
    this module imports no log / protocol code) + the mode names. -> (model, newest time in s since the log's start)."""
    model = tr.Trends()
    rt = [r[0] / 1000.0 for r in rows]
    end = 0.0
    for smp in log.samples:
        t = smp["t_ms"] / 1000.0
        end = t
        mode = _mode_name(modes, smp.get("mode"))
        d = {"est_sog": smp.get("est_sog"), "throttle": smp.get("thr_out"), "heading": smp.get("heading"),
             "yaw_rate": smp.get("yaw_rate"), "slam_peak": smp.get("slam_peak"), "slam_rate": smp.get("slam_rate"),
             "bank_v": smp.get("bank_v") or None, "mode_name": mode}
        if rt:
            i = bisect.bisect_left(rt, t)
            if i < len(rt) and rt[i] - t < ACC_WINDOW_S:
                r = rows[i]
                for k, ax in enumerate(tr.ACC_IDS):
                    d[ax], d[ax + "_pk"] = r[1 + 2 * k], r[2 + 2 * k]
        model.feed(t, d, xte_m=smp.get("xte") if mode == "AUTO" else None)
    model.tick(end + 1.0)
    return model, end


def from_samples(samples):
    """A Trends model from academy replay samples. -> (model, newest time)."""
    model = tr.Trends()
    end = 0.0
    for s in samples:
        t = s.get("t")
        if not tr.finite(t):
            continue
        end = max(end, float(t))
        d = {"est_sog": s.get("sog"), "throttle": s.get("thr"), "heading": s.get("hdg"), "yaw_rate": s.get("rot"),
             "slam_peak": s.get("slam"), "slam_rate": s.get("pm"), "mode_name": s.get("mode") or ""}
        for ax, k in zip(tr.ACC_IDS, ("al", "at", "av")):
            d[ax], d[ax + "_pk"] = s.get(k), s.get(k + "_pk")
        model.feed(float(t), d)
    model.tick(end + 1.0)
    return model, end


def from_hud(timeline):
    """A Trends model from a video lesson's hud.jsonl rows ({"t": seconds into the clip, "d": the HUD's values}); a
    value the row lacks is a gap. -> (model, newest time)."""
    model = tr.Trends()
    end = 0.0
    for row in timeline or ():
        t, v = row.get("t"), row.get("d") or {}
        if not tr.finite(t):
            continue
        end = max(end, float(t))
        model.feed(float(t), {k: v.get(k) for k in ("est_sog", "throttle", "heading", "yaw_rate", "slam_peak",
                                                    "slam_rate", "bank_v")})
    model.tick(end + 1.0)
    return model, end


def available(model, graphs):
    """The graphs of `graphs` that hold at least one value in the model (a graph with nothing is not drawn)."""
    out = []
    for g in graphs:
        if any(any(x == x for x in model._linear(model.vals[v], model.count())) for v in tr.GRAPHS[g]):
            out.append(g)
    return out


def draw(canvas, model, graphs, now, w, h, window_s=None, cursor_t=None, name=lambda v: v, fmt=None, font=None):
    """Draw the graphs stacked on `canvas` (tk.Canvas or anything with create_rectangle / create_line / create_text) over
    the whole recording: a dark panel, each value its OWN colour + line pattern, the newest value in the caption.
    -> a record {"graphs": [{"g", "rect", "lines": [...]}], "plot": (x0, x1), "window": s} (what the tests read)."""
    window_s = window_s or max(10.0, now)
    canvas.create_rectangle(0, 0, w, h, fill=BG, outline="")
    n = max(1, len(graphs))
    ch = h / n
    kw = {"font": font} if font else {}
    rec = {"graphs": [], "plot": (0, w), "window": window_s}
    for k, g in enumerate(graphs):
        y0, y1 = k * ch, (k + 1) * ch
        cap = 16 * len(tr.GRAPHS[g])
        px0, px1, py0, py1 = 70, w - 8, y0 + cap + 2, y1 - 3
        cols = max(8, int((px1 - px0) / 2))
        lines = trendgraph.lines(model, g, window_s, now, cols, False)
        canvas.create_rectangle(px0, py0, px1, py1, outline=theme.TREND_GRID, fill="")
        grec = {"g": g, "rect": (0, y0, w, y1), "lines": lines, "cap": []}
        for i, ln in enumerate(lines):
            vid = ln["id"]
            v = ln["last"]
            txt = name(vid)
            if tr.finite(v):
                txt += "  %s %s" % ("%.*f" % (tr.INFO[vid]["dec"], v), tr.INFO[vid]["unit"])
            canvas.create_text(4, y0 + 2 + i * 16, text=txt, anchor="nw", fill=ln["colour"], **kw)
            grec["cap"].append(txt)
            for runs, wd, fade in ((ln["peak_runs"], 1, True), (ln["runs"], 2, False)):
                for run in runs:
                    pts = [(px0 + x * (px1 - px0), py1 - y * (py1 - py0)) for x, y in run]
                    if len(pts) == 1:
                        pts = [(pts[0][0] - 1.0, pts[0][1]), (pts[0][0] + 1.0, pts[0][1])]
                    col = theme.mix(ln["colour"], BG, 0.45) if fade else ln["colour"]
                    for piece in trendgraph.dash_runs(pts, ln["dash"]):
                        canvas.create_line(*[q for pt in piece for q in pt], fill=col, width=wd)
        if cursor_t is not None and now - window_s <= cursor_t <= now:
            cx = px0 + (cursor_t - (now - window_s)) / window_s * (px1 - px0)
            canvas.create_line(cx, y0 + 1, cx, y1 - 1, fill=theme.TREND_TEXT)
        rec["plot"] = (px0, px1)
        rec["graphs"].append(grec)
    return rec
