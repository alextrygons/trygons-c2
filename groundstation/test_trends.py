"""Tests of the trend panel «ΤΑΣΕΙΣ» (step TR2, CLAUDE.md Y, docs/trends_design.md 10). DISPLAY ONLY, READ-ONLY.

  TG1  ring buffer: size, wrap, NaN gaps, cleared on a boat restart (t_ms back) / boat change
  TG2  sampler: every source field + unit from REAL decoded frames (proto.decode_telemetry / decode_telem2 of packed
       frames); slam peak = the max of a slot; stale / missing / None = a gap; xte only in AUTO / DP TRIP; lost frames
  TG3  drawing: no segment across a gap, a spike survives the decimation, labels at the right end, zoom 2 / 10 / 30 /
       60 min, a click = time + every value, a second click / Esc closes it
  TG4  palette rules (distance, no red / amber, patterns per graph) + the NIGHT colours
  TG5  layout: test_screen2 S29 (the S1 matrix with the panel open at 6 graphs) is part of the gate
  TG6  read-only proof, static: no trend module imports a sender or calls one (AST)
  TG7  read-only proof, transitive import graph of the trend modules
  TG8  read-only proof, runtime: the station with every sender trapped - 10 min of synthetic telemetry, clicks, zoom,
       night, choose row, Esc: 0 sender calls from trend code (stack taint, negative control), alarms / lamps /
       sound unchanged, only trends_* settings written by the panel
  TG9  narrow view: the panel holds no App (gc walk), TrendView refuses other settings keys
  TG10 CPU bounds (tools/trend_load.py, 3x looser than the design's limits)
  TJ1  screen-1 strip, the model side (placement / MINIMAL / layout v4 = test_driver I26)
  TJ2  debrief: a recorded black box -> the same rings; the logview.py pane
  TJ3  academy replays (scenario replay samples, video-lesson hud.jsonl), files only
  TJ4  docs: manuals, lang keys, tour stops, screenshots
  TJ5  acceleration graph: sampler, shared scale, own colour + pattern per axis, peak line, readout

Run:  python test_trends.py        (exit code 0 = all passed)
"""
import ast
import gc
import math
import os
import socket
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import test_screen2 as S        # noqa: E402  (sets the temp settings / test env; the real station rig for TG8)
import i18n  # noqa: E402
import overlay  # noqa: E402
import proto  # noqa: E402
import settings  # noqa: E402
import theme  # noqa: E402
import trendpanel  # noqa: E402
import trends as tr  # noqa: E402
import alerts as alerts_mod  # noqa: E402

results = []
NAN = float("nan")
TREND_FILES = ("trends.py", "trendpanel.py", "trenddetect.py", "trendlog.py", "trendgraph.py", "trendreplay.py")


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


# --- real frames ----------------------------------------------------------------------------------------------
def telem(**kw):
    vals = {n: 0 for n in proto.TELEM_FIELDS}
    vals.update(kw)
    return proto.decode_telemetry(proto._TELEM.pack(*[vals[n] for n in proto.TELEM_FIELDS]))


def telem2(**kw):
    out = b""
    for st, names in proto._T2_GROUPS[:11]:             # up to group L (bank voltage)
        out += st.pack(*[kw.get(n, 0) for n in names])
    return proto.decode_telem2(out)


def frame(t_ms=0, flags=1, t2=None, **kw):
    """TELEMETRY + TELEM2 of one packed frame (compass OK by default) merged like the station's link does."""
    d = telem(t_ms=t_ms, flags=flags, mode=3, **kw)
    d.update(telem2(**(t2 or {})))
    return d


def vals_at(model, k=-1):
    i = (model.n - 1) % tr.N if k == -1 else k
    return {v: float(model.vals[v][i]) for v in tr.IDS}


def feed_run(model, t0, n, step=0.5, fn=None, **kw):
    """n frames from t0 every `step` s (10 Hz frames if step 0.1)."""
    for k in range(n):
        d = frame(t_ms=int((t0 + k * step) * 1000) % 2000000000, **(fn(k) if fn else kw))
        model.feed(t0 + k * step, d, rtt_ms=40.0)


# --- TG1 --------------------------------------------------------------------------------------------------
def tg1():
    m = tr.Trends()
    t0 = 1000.0
    n = tr.N + 500
    for k in range(n):
        d = frame(t_ms=k * 500, est_sog=k * 0.001, t2={"att_age_ms": 5000 if k == 7000 else 0, "att_roll100": 100})
        m.feed(t0 + k * 0.5, d)
    m.tick(t0 + n * 0.5 + 1)
    cnt = m.count()
    times, vals = m.series("sog", 1e9, t0 + n * 0.5 + 10)
    wrapped = len(times) == tr.N and abs(times[0] - (t0 + (n - tr.N) * 0.5)) < 0.51 and times == sorted(times)
    roll_t, roll_v = m.series("roll", 1e9, t0 + n * 0.5 + 10)
    nan_idx = [i for i, x in enumerate(roll_v) if x != x]
    gap_ok = len(nan_idx) == 1                              # the one stale-attitude sample = NaN, nothing else
    # 3 gaps of 5 s: the line is broken (3 + 1 runs), never joined
    g = tr.Trends()
    for blk in range(4):
        feed_run(g, 2000.0 + blk * 20.0, 20, est_sog=5.0)
    g.tick(5000.0)
    runs, lo, hi = g.segments("sog", 100.0, 2080.0, 100)
    # restart: t_ms back clears every ring
    r = tr.Trends()
    feed_run(r, 100.0, 30, est_sog=3.0)
    before = r.count()
    r.feed(130.0, frame(t_ms=10, est_sog=4.0))
    after = r.count()
    # boat change / disconnect = clear()
    r.clear()
    cleared = r.count() == 0 and all(x != x for x in r.vals["sog"])
    report("TG1 ring buffer: 7200 samples (60 min), wraps keeping the newest in time order, a missing value = NaN, a "
           "gap breaks the line, cleared on a boat restart (t_ms back) and on clear() (boat change / disconnect)",
           cnt == tr.N and wrapped and gap_ok and len(runs) == 4 and before > 0 and after == 0 and cleared,
           "count %d (N %d); wrapped %s; NaN samples %d; runs over 3 gaps %d; before restart %d, after %d; cleared %s"
           % (cnt, tr.N, wrapped, len(nan_idx), len(runs), before, after, cleared))


# --- TG2 --------------------------------------------------------------------------------------------------
def fake_app(model, mode, wp=True, pos=True, charts=None):
    leg = {}
    drv = types.SimpleNamespace(leg=leg, waypoint_info=lambda d, now: (None, 12.5, 100.0))
    ev = []
    return types.SimpleNamespace(trends=model, driver=drv, charts=charts, gw_status={"you": 3, "stations": [
        {"id": 3, "rtt_ms": 77}]}, _trend_dep=(0.0, None), event=ev.append, _trend_err=False, ev=ev)


def tg2():
    import app as A
    m = tr.Trends()
    big = dict(est_sog=7.5, throttle=-30.0, heading=123.4, yaw_rate=-2.5,
               t2={"att_roll100": -250, "att_pitch100": 310, "att_age_ms": 10, "slam_peak100": 175, "slam_rate": 4,
                   "bank_v100": 1262})
    m.feed(10.0, frame(t_ms=1000, **big), xte_m=-4.0, depth_m=12.5, rtt_ms=61.0)
    m.feed(10.6, frame(t_ms=1600, **big), xte_m=-4.0, depth_m=12.5, rtt_ms=61.0)
    m.tick(11.5)
    v = vals_at(m)
    want = {"sog": 7.5, "thr": -30.0, "hdg": 123.4, "rot": -2.5, "xte": -4.0, "roll": -2.5, "pitch": 3.1,
            "slam_g": 1.75, "slam_pm": 4.0, "depth": 12.5, "bank_v": 12.62, "link_ms": 61.0}
    bad = {k: (v[k], x) for k, x in want.items() if abs(v[k] - x) > 0.01}
    units = {i: tr.INFO[i]["unit"] for i in tr.IDS}
    units_ok = units["sog"] == "kn" and units["thr"] == "%" and units["hdg"] == "°" and units["rot"] == "°/s" and \
        units["xte"] == "m" and units["slam_g"] == "g" and units["bank_v"] == "V" and units["link_ms"] == "ms"
    # a missing value is a gap, never 0: stale attitude (att_age >= 1 s), no voltage, compass lost
    g = tr.Trends()
    d = frame(t_ms=1, flags=0, est_sog=5.0, t2={"att_age_ms": 4000, "att_roll100": 500, "bank_v100": 0xFFFF})
    g.feed(1.0, d)
    g.feed(1.6, frame(t_ms=600, flags=0, est_sog=5.0, t2={"att_age_ms": 4000, "bank_v100": 0xFFFF}))
    g.tick(3.0)
    gv = vals_at(g)
    gaps = all(gv[k] != gv[k] for k in ("roll", "pitch", "bank_v", "hdg", "xte", "depth", "link_ms")) and gv["sog"] == 5.0
    # slam peak = the MAX of the frames of one slot
    s = tr.Trends()
    for i, pk in enumerate((30, 120, 50)):
        s.feed(1.0 + 0.1 * i, frame(t_ms=100 * i, t2={"slam_peak100": pk}))
    s.feed(1.6, frame(t_ms=600, t2={"slam_peak100": 10}))
    s.tick(3.0)
    slam_max = abs(s.vals["slam_g"][0] - 1.2) < 0.001
    # stale / silent for 5 s = a gap between two samples (two runs), the time after it is never held
    q = tr.Trends()
    feed_run(q, 100.0, 10, est_sog=4.0)
    feed_run(q, 108.0, 10, est_sog=4.0)
    q.tick(120.0)
    runs, _lo, _hi = q.segments("sog", 30.0, 115.0, 60)
    stale_gap = len(runs) == 2
    # the lost frames: every second 100 ms frame missing for 5 s = about 50 %
    L = tr.Trends()
    for k in range(0, 50, 2):
        L.feed(50.0 + k * 0.1, frame(t_ms=k * 100))
    loss = L.loss_pct()
    L2 = tr.Trends()
    for k in range(0, 40):
        L2.feed(50.0 + k * 0.1, frame(t_ms=k * 100))
    loss0 = L2.loss_pct()
    L3 = tr.Trends()
    L3.feed(50.0, frame(t_ms=0))
    L3.feed(50.1, frame(t_ms=100))
    loss_unknown = L3.loss_pct() != L3.loss_pct()       # under 2 s of history = NaN
    # the station-side values through App.trend_feed: xte only in AUTO / a DP TRIP with a waypoint, never otherwise
    out = {}
    for name, mode, extra in (("auto", proto.MODE_AUTO, {}), ("hdg", proto.MODE_HDG_LOCK, {}),
                              ("trip", proto.MODE_DP, {"dp_sub": proto.DP_TRIP}),
                              ("spot", proto.MODE_DP, {"dp_sub": proto.DP_SPOT}),
                              ("auto_nowp", proto.MODE_AUTO, {"wp_lat": 0})):
        mm = tr.Trends()
        fa = fake_app(mm, mode)
        d = frame(t_ms=1000, **{"est_sog": 5.0})
        d.update(mode=mode, pos_source=1, wp_set=extra.get("wp_lat", 1) != 0, wp_lat=37.8, wp_lon=23.6, est_lat=37.7,
                 est_lon=23.5, dp_sub=0, auto_wp=1, prog_n=3)
        d.update({k: x for k, x in extra.items() if k != "wp_lat"})
        A.App.trend_feed(fa, d)
        mm.feed(time.time() + 0.6, dict(d, t_ms=1600), xte_m=mm.cur.get("xte"), rtt_ms=77)
        mm.tick(time.time() + 3.0)
        out[name] = mm.last("xte")
    xte_ok = abs(out["auto"] - 12.5) < 1e-6 and out["hdg"] != out["hdg"] and out["spot"] != out["spot"] and \
        out["auto_nowp"] != out["auto_nowp"] and out["trip"] == out["trip"]
    report("TG2 sampler: every source field + unit from real decoded frames (SOG kn, throttle %, heading, rate of turn, "
           "roll / pitch, slam peak g + per min, bank V, chart depth, round trip, cross-track); a stale / missing / None "
           "value = NaN; slam peak = the max of a slot; a silent stretch = a gap; lost frames in %; xte only in AUTO / "
           "DP TRIP",
           not bad and units_ok and gaps and slam_max and stale_gap and 40 < loss < 60 and loss0 < 1 and loss_unknown
           and xte_ok,
           "wrong values %s; units %s; gaps %s; slam max %s; silent stretch %d runs; loss %.0f %% (full %.0f %%, "
           "<2 s unknown %s); xte by mode %s" % (bad, units_ok, gaps, slam_max, len(runs), loss, loss0, loss_unknown,
                                                  {k: (None if x != x else round(x, 2)) for k, x in out.items()}))


# --- TG3 --------------------------------------------------------------------------------------------------
_root = None


def canvas():
    global _root
    import tkinter as tk
    if _root is None:
        _root = tk.Tk()
        _root.withdraw()
    cv = tk.Canvas(_root, width=1920, height=1080)
    cv.pack()
    return cv


def close_root():
    """The station rig makes its own Tk: ours must be gone first (a PhotoImage belongs to the DEFAULT root)."""
    global _root
    if _root is not None:
        try:
            _root.destroy()
        except Exception:
            pass
        _root = None


def draw(model, store, night=False, W=1920, H=1080, now=None, dpi=1.0, tp=None):
    """Draw the open panel on a canvas: (tp, ov, panel, ops of the graphs)."""
    cv = canvas()
    cv.delete("all")
    view = trendpanel.TrendView(store.get, store.__setitem__, lambda: night)
    tp = tp or trendpanel.TrendPanel(model, view)
    ov = overlay.Overlay(cv)
    ov.dpi = dpi
    ov.begin(W, H)
    ov.k = 1.0
    p = ov.panel("trends", "T", "ne")
    now = now if now is not None else time.time()
    tp.fill(ov, p, now)
    ov._measure_panel(p)
    p.r = (10, 10, 10 + p.w, 10 + p.h)
    ov.ops, ov.hits, ov.holds, ov.heads, ov.hits_xy = [], [], [], [], []
    ov.draw_panel(p)
    plots = {o["trend"]["graph"]: o for o in ov.ops if o.get("trend")}
    return tp, ov, p, plots


def tg3():
    now = 100000.0
    m = tr.Trends()
    # 60 min of data: speed with ONE spike, a 5-minute hole in the pitch, the heading wrapping through 360
    def f(k):
        return {"est_sog": 5.0 + (40.0 if k == 3000 else 0.0), "heading": (350.0 + k * 0.02) % 360.0,
                "yaw_rate": 0.1, "t2": {"att_age_ms": 4000 if 4000 <= k < 4600 else 0, "att_pitch100": 300,
                                        "att_roll100": 100, "bank_v100": 1250}}
    feed_run(m, now - 3600.0, 7200, 0.5, fn=f)
    m.tick(now + 1.0)
    store = {"trends_zoom": 60}
    tp, ov, p, plots = draw(m, store, now=now)
    g1 = plots.get(1)
    g4 = plots.get(4)
    lines1 = {ln["id"]: ln for ln in g1["trend"]["lines"]} if g1 else {}
    sog = lines1.get("sog")
    # the spike survives the decimation: the highest drawn point of the speed line is the topmost y of its range
    top = min((y for run in sog["runs"] for _x, y in run), default=None) if sog else None
    plot_top = g1["r"][1] + 3 if g1 else 0
    spike_ok = top is not None and abs(top - plot_top) < 1.0
    # no segment across a gap: the pitch hole (300 s) splits its line, no two joined points are further apart than
    # the gap allows (more than 4 px per 2 px column pair would mean a bridge)
    pitch = {ln["id"]: ln for ln in g4["trend"]["lines"]}["pitch"]
    w_px = g4["trend"]["plot"][1] - g4["trend"]["plot"][0]
    hole_px = 600 * 0.5 / 3600.0 * w_px
    joined = any(b[0] - a[0] > hole_px * 0.5 for run in pitch["runs"] for a, b in zip(run, run[1:]))
    gap_ok = len(pitch["runs"]) == 2 and not joined
    # a heading through 360 is one continuous line (unwrapped), the label says the true heading
    hdg = None
    plots2 = draw(m, {"trends_zoom": 60, "trends_show": [2]}, now=now)[3]
    hl = {ln["id"]: ln for ln in plots2[2]["trend"]["lines"]}["hdg"]
    jump = max((abs(b[1] - a[1]) for run in hl["runs"] for a, b in zip(run, run[1:])), default=0)
    wrap_ok = len(hl["runs"]) == 1 and jump < (plots2[2]["r"][3] - plots2[2]["r"][1])
    # labels at the right end: value + unit + name, right of the plot
    lab = lines1["sog"]["label"] if "sog" in lines1 else ""
    names = i18n.T("trend.sog") in lab and "kn" in lab
    # zoom: the same data over 2 / 10 / 30 / 60 minutes; the last point is at the right end each time
    zoom_ok, spans = True, {}
    for z in tr.ZOOMS:
        st = {"trends_zoom": z, "trends_show": [1]}
        pl = draw(m, st, now=now)[3][1]
        xs = [x for ln in pl["trend"]["lines"] for run in ln["runs"] for x, _y in run]
        spans[z] = (round(min(xs)), round(max(xs)), round(pl["trend"]["plot"][0]), round(pl["trend"]["plot"][1]))
        zoom_ok = zoom_ok and xs and max(xs) >= pl["trend"]["plot"][1] - 6 and min(xs) >= pl["trend"]["plot"][0] - 1
    # the click: a time + every value; a second click / Esc closes it
    store = {"trends_zoom": 10, "trends_show": [1, 4]}
    tp, ov, p, plots = draw(m, store, now=now)
    r = plots[1]["r"]
    px0, px1 = plots[1]["trend"]["plot"]
    x = px0 + 0.5 * (px1 - px0)
    ov.click(x, (r[1] + r[3]) / 2.0)
    # TR4: the graph is drawn as of the start of the current pixel-column step (qnow, at most one column = 2 px behind
    # the clock): the click maps through the SAME time axis, and the lag is bounded by one column
    qnow = tp._plot[1][2]
    cols_ = max(1, int((px1 - px0 - 2.0) / 2.0))
    want_t = qnow - 600.0 + 0.5 * 600.0
    t_ok = tp.readout_t is not None and abs(tp.readout_t - want_t) < 1.0 and 0.0 <= now - qnow <= 600.0 / cols_ * 1.5 + 0.5
    txt = tp.readout_text()
    smp = m.at(tp.readout_t)
    hhmmss = time.strftime("%H:%M:%S", time.gmtime(smp["t"])) if smp else "?"
    every = all(i18n.T("trend." + v) in txt for v in tr.IDS)
    read_ok = hhmmss in txt and "UTC" in txt and every
    tp2, ov2, p2, pl2 = draw(m, store, now=now, tp=tp)          # drawn again: the cursor line is there
    cursor = pl2[1]["trend"]["cursor"] is not None
    ov2.click(x, (r[1] + r[3]) / 2.0)
    second = tp.readout_t is None
    tp.readout_t = want_t
    esc = tp.escape() and tp.readout_t is None
    report("TG3 drawing: the spike survives the decimation, no segment across a gap, the heading is one line through "
           "360, value + unit + name at the right end, zoom 2 / 10 / 30 / 60 min, a click = time + every value + a "
           "cursor, a second click / Esc closes it",
           spike_ok and gap_ok and wrap_ok and names and zoom_ok and t_ok and read_ok and cursor and second and esc,
           "spike %s (top %.1f / %.1f); pitch runs %d, joined %s; heading runs %d jump %.0f px; label %r; zoom ends "
           "%s; click t %s; readout %s; cursor %s; second click closes %s; Esc %s" % (
               spike_ok, top or -1, plot_top, len(pitch["runs"]), joined, len(hl["runs"]), jump, lab, spans, t_ok,
               read_ok, cursor, second, esc))


# --- TG4 --------------------------------------------------------------------------------------------------
def tg4():
    import colorsys
    import itertools

    def rgb(c):
        return [int(c[i:i + 2], 16) for i in (1, 3, 5)]

    def dist(a, b):
        return math.dist(rgb(a), rgb(b))
    T = theme.TREND
    mean = [theme.WARN, theme.CAUTION, theme.GOOD, theme.ADVISORY, theme.MAGENTA, theme.YELLOW, theme.HUD_YELLOW,
            theme.REC, theme.FROZEN, theme.WARN_DARK] + list(theme.NIGHT.values()) + list(theme.MODE_COLOURS.values())
    pair = min((dist(T[a], T[b]), a, b) for a, b in itertools.combinations(T, 2))
    meaning = min((dist(T[a], m_), a, m_) for a in T for m_ in mean)
    hue_bad = []
    for k, c in T.items():
        h, _l, s = colorsys.rgb_to_hls(*[x / 255.0 for x in rgb(c)])
        if s > 0.25 and (h * 360 >= 340 or h * 360 <= 58):
            hue_bad.append(k)
    same_graph = []
    for g, ids in tr.GRAPHS.items():
        if len(ids) == 2:
            a, b = ids
            la = colorsys.rgb_to_hls(*[x / 255.0 for x in rgb(T[a])])[1]
            lb = colorsys.rgb_to_hls(*[x / 255.0 for x in rgb(T[b])])[1]
            if theme.TREND_DASH[a] == theme.TREND_DASH[b] and abs(la - lb) < 0.1:
                same_graph.append((a, b))
    all_ids = set(T) == set(tr.IDS) == set(theme.TREND_DASH)
    # NIGHT: every drawn line is NIGHT main / dim, the pattern + the label identify it
    now = 5000.0
    m = tr.Trends()
    feed_run(m, now - 700, 1400, 0.5, est_sog=5.0, throttle=40.0, yaw_rate=0.4, heading=100.0,
             t2={"att_roll100": 100, "att_pitch100": 200, "slam_peak100": 20, "slam_rate": 1, "bank_v100": 1250})
    m.tick(now + 1)
    tp, ov, p, plots = draw(m, {"trends_show": [1, 2, 4, 5, 7, 8]}, night=True, now=now)
    cols = {ln["colour"] for pl in plots.values() for ln in pl["trend"]["lines"]}
    night_ok = cols <= {theme.NIGHT["main"], theme.NIGHT["dim"], theme.NIGHT["warn"]} and len(plots) == 6
    pats = all(len({ln["dash"] for ln in pl["trend"]["lines"]}) == len(pl["trend"]["lines"]) or
               pl["graph"] in (7,) for pl in plots.values())
    day = draw(m, {"trends_show": [1, 2, 4, 5, 7, 8]}, night=False, now=now)[3]
    day_cols = {ln["colour"] for pl in day.values() for ln in pl["trend"]["lines"]}
    day_ok = day_cols <= set(T.values())
    report("TG4 palette: every pair of trend colours >= 55 apart, each >= 70 from every meaning colour, no red / amber "
           "hue, the lines of one graph differ in pattern or lightness; NIGHT = only the dim red (main / dim) + the "
           "patterns, the day colours are the table's",
           pair[0] >= 55 and meaning[0] >= 70 and not hue_bad and not same_graph and all_ids and night_ok and pats
           and day_ok,
           "closest pair %.0f (%s / %s); closest meaning %.0f (%s / %s); bad hues %s; same look in a graph %s; night "
           "colours %s; patterns differ %s; day colours ok %s" % (pair + meaning + (hue_bad, same_graph,
                                                                                    sorted(cols), pats, day_ok)))


# --- TG5 --------------------------------------------------------------------------------------------------
def tg5():
    src = open(os.path.join(HERE, "test_screen2.py"), encoding="utf-8").read()
    ok = "def s29_trends(rig):" in src and "s29_trends, s16_silence" in src and "trends" in src
    gate = open(os.path.join(HERE, "..", "tools", "check.sh"), encoding="utf-8").read()
    report("TG5 layout: test_screen2 S29 (the S1 matrix with the panel open at 6 graphs, docked top / bottom, el + en) "
           "is in the screen 2 gate", ok and "test_screen2.py" in gate,
           "s29_trends defined + run by main: %s; test_screen2 in tools/check.sh: %s" % (ok, "test_screen2.py" in gate))


# --- TG6 / TG7 --------------------------------------------------------------------------------------------
BANNED_MODULES = {"socket", "asyncio", "gwclient", "link", "stations", "aurix", "proto", "sound", "alerts", "tts",
                  "asstvoice", "hailer", "subprocess", "threading", "urllib", "http", "ssl", "select", "app", "driver",
                  "hud", "audiodev", "shorejoy", "uavfly", "boats", "syncrecv", "screenrec", "video", "childproc"}
BANNED_CALLS = {"send", "sendall", "sendto", "send_msg", "send_frame", "cmd", "connect", "write_frame", "put_nowait",
                "popen", "Popen", "system", "startfile", "post", "urlopen", "request"}


def trend_modules():
    return [f for f in TREND_FILES if os.path.exists(os.path.join(HERE, f))]


def imports_of(path):
    tree = ast.parse(open(path, encoding="utf-8").read())
    mods = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            mods |= {a.name.split(".")[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            mods.add(n.module.split(".")[0])
    return mods, tree


def tg6():
    bad_imp, bad_calls = [], []
    for f in trend_modules():
        mods, tree = imports_of(os.path.join(HERE, f))
        bad_imp += [(f, m_) for m_ in sorted(mods & BANNED_MODULES)]
        for n in ast.walk(tree):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in BANNED_CALLS:
                bad_calls.append((f, n.func.attr, n.lineno))
            if isinstance(n, ast.Name) and n.id in ("__import__", "eval", "exec"):
                bad_calls.append((f, n.id, n.lineno))
    # negative control: the scan finds a planted sender import / call
    planted = ast.parse("import socket\nself.app.send(1)\n")
    caught = any(isinstance(n, ast.Import) for n in ast.walk(planted)) and \
        any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in BANNED_CALLS
            for n in ast.walk(planted))
    report("TG6 read-only, static: no trend module imports a sender (socket, gwclient, link, stations, proto, sound, "
           "alerts, the App ...) or calls send / cmd / connect / eval (AST scan; the scan catches a planted one)",
           not bad_imp and not bad_calls and caught and len(trend_modules()) >= 2,
           "modules %s; banned imports %s; banned calls %s; negative control caught %s" % (
               trend_modules(), bad_imp, bad_calls, caught))


def graph_of(roots):
    """The local (groundstation/) modules reachable from `roots` through their imports."""
    seen, todo = set(), list(roots)
    while todo:
        m_ = todo.pop()
        if m_ in seen:
            continue
        seen.add(m_)
        path = os.path.join(HERE, m_ + ".py")
        if os.path.exists(path):
            mods, _t = imports_of(path)
            todo += [x for x in mods if os.path.exists(os.path.join(HERE, x + ".py"))]
    return seen


def tg7():
    roots = [f[:-3] for f in trend_modules()]
    reach = graph_of(roots)
    sender_reach = sorted((reach - set(roots)) & BANNED_MODULES)
    # a planted module that imports a sender is found by the same walk
    planted_found = "socket" in imports_of_src("import socket")
    report("TG7 read-only, the transitive import graph of the trend modules reaches no sender module",
           not sender_reach and planted_found,
           "reachable: %s; senders reached: %s" % (sorted(reach), sender_reach))


def imports_of_src(src):
    return {a.name.split(".")[0] for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Import) for a in n.names}


# --- TG8 --------------------------------------------------------------------------------------------------
def tg8():
    close_root()
    rig = S.Rig()
    ap = rig.ap
    taint, total = [], {}
    originals = {}

    def trap(owner, name):
        orig = getattr(owner, name)
        originals[(owner, name)] = orig

        def f(*a, **k):
            total[name] = total.get(name, 0) + 1
            fr = sys._getframe(1)
            while fr is not None:
                if os.path.basename(fr.f_code.co_filename) in TREND_FILES:
                    taint.append((name, fr.f_code.co_name))
                    raise RuntimeError("a sender was called from trend code: " + name)
                fr = fr.f_back
            return orig(*a, **k)
        setattr(owner, name, f)
    try:
        for owner, names in ((ap, ("cmd", "send_msg")), (ap.link, ("send", "send_msg")),
                             (socket.socket, ("send", "sendall", "sendto", "connect"))):
            for n in names:
                if hasattr(owner, n):
                    trap(owner, n)
        ap.trend_feed = lambda d: None          # (the sim's real t_ms would reset the synthetic ring)
        ap.trends.clear()
        rig.size(1920, 1080, 1.0)
        ap.ov.collapsed.discard("trends")
        lamps0 = [tuple(r) for r in ap.lamps.rows()]
        alarm0 = (ap.t or {}).get("alarm_bits")
        sound0 = getattr(ap, "sound", None)
        keys0 = {k: repr(settings.get(k)) for k in list(settings.DEFAULTS) + ["s2_collapsed", "s2_panel_pos"]}
        now = time.time()
        t0 = now - 600.0
        d = dict(ap.t)
        for k in range(1200):                   # 10 min of synthetic telemetry (2 Hz samples)
            tt = t0 + 0.5 * k
            dd = dict(d, t_ms=int(tt * 1000) % 2000000000, est_sog=8.0 + math.sin(k / 30.0), throttle=40.0,
                      heading=(90.0 + k * 0.05) % 360.0, yaw_rate=math.sin(k / 9.0), att_roll=2.0 * math.sin(k / 12.0),
                      att_pitch=3.0, slam_peak=0.2, slam_rate=2, bank_v=12.6, mode_name="AUTO")
            ap.trends.feed(tt, dd, rtt_ms=40.0)
        ap.trends.tick(now + 1.0)
        rig.pump(0.4)
        tp = ap.trendpanel
        r = None
        for _ in range(3):                      # draw, click a graph, zoom, choose row, night, Esc
            ap.render(force=True)
            r = next((o["r"] for o in ap.ov.ops if str(o.get("tag", "")).endswith("g1_plot")), None)
            if r:
                break
        steps = []
        if r:
            ap.cv_press(types.SimpleNamespace(x=(r[0] + r[2]) / 2, y=(r[1] + r[3]) / 2))
            ap.cv_release(types.SimpleNamespace(x=(r[0] + r[2]) / 2, y=(r[1] + r[3]) / 2))
            steps.append(tp.readout_t is not None)
        for tag in ("trend_zoom_2", "trend_zoom_30", "trend_choose", "trend_g3", "trend_choose", "trend_zoom_10"):
            o = next((o for o in ap.ov.ops if o.get("tag") == tag), None)
            if o:
                x, y = (o["r"][0] + o["r"][2]) / 2, (o["r"][1] + o["r"][3]) / 2
                ap.cv_press(types.SimpleNamespace(x=x, y=y))
                ap.cv_release(types.SimpleNamespace(x=x, y=y))
                ap.render(force=True)
                steps.append(True)
            else:
                steps.append(False)
        settings.put("night", True)
        ap.render(force=True)
        settings.put("night", False)
        ap.render(force=True)
        tp.readout_t = tp.readout_t if tp.readout_t is not None else now - 100.0     # (a readout open for Esc)
        ap.root.focus_force()
        rig.pump(0.2)
        ap.root.event_generate("<Escape>")
        rig.pump(0.3)
        steps.append(tp.readout_t is None)
        # the trend code never reached a sender; alarms / lamps / sound / the other settings untouched
        lamps1 = [tuple(r_) for r_ in ap.lamps.rows()]
        changed = [k for k, v in {k: repr(settings.get(k)) for k in keys0}.items()
                   if v != keys0[k] and not k.startswith("trends_") and k not in ("s2_collapsed", "night")]
        # negative control: code that LOOKS like trend code (its file name) calling a sender is caught
        evil = compile("ap.cmd(0, b'', 'x')", "trendpanel.py", "exec")
        caught = False
        try:
            exec(evil, {"ap": ap})
        except RuntimeError:
            caught = True
        ok = (not [t_ for t_ in taint if t_[1] != "<module>"]) and caught and lamps1 == lamps0 and \
            (ap.t or {}).get("alarm_bits") == alarm0 and getattr(ap, "sound", None) is sound0 and not changed and \
            all(steps) and not rig.errors
        report("TG8 read-only, runtime: the station with every sender trapped - 10 min of synthetic telemetry, graph "
               "click, zoom, choose row, a hidden graph, night, Esc: 0 sender calls from trend code (stack taint; the "
               "negative control is caught), lamps / alarm bits / sound unchanged, no setting written but trends_*",
               ok, "tainted %s; control caught %s; steps %s; lamps same %s; other settings changed %s; sender calls "
                   "seen in all (the station's own): %s; errors %s" % (
                       taint[:2], caught, steps, lamps1 == lamps0, changed, total, rig.errors[:2]))
    finally:
        for (owner, name), orig in originals.items():
            try:
                setattr(owner, name, orig)
            except (AttributeError, TypeError):
                pass
        rig.gateway_like(False)
        rig.close()


# --- TG9 --------------------------------------------------------------------------------------------------
def walk_reaches(start, stop_types):
    """Objects reachable from `start` (functions through their closure + defaults only, never their globals; modules
    and classes are not entered): the first object of a type in stop_types, or None."""
    seen, todo, steps = set(), [start], 0
    while todo and steps < 200000:
        o = todo.pop()
        if id(o) in seen:
            continue
        seen.add(id(o))
        steps += 1
        if isinstance(o, stop_types):
            return o
        if isinstance(o, (types.ModuleType, type)):
            continue
        if isinstance(o, types.FunctionType):
            todo += [c.cell_contents for c in (o.__closure__ or ()) if _alive(c)]
            todo += list(o.__defaults__ or ())
            continue
        if isinstance(o, types.MethodType):
            todo += [o.__self__, o.__func__]
            continue
        if isinstance(o, types.FrameType):
            continue
        todo += gc.get_referents(o)
    return None


def _alive(cell):
    try:
        cell.cell_contents
        return True
    except ValueError:
        return False


def tg9():
    import app as A
    m = tr.Trends()
    store = {}
    view = trendpanel.TrendView(store.get, store.__setitem__, lambda: False)
    tp = trendpanel.TrendPanel(m, view)
    clean = walk_reaches(tp, (A.App,)) is None
    # a negative control: a panel that kept an App-like object IS found
    holder = types.SimpleNamespace(view=view)
    fake = A.App.__new__(A.App)
    holder.app = fake
    control = walk_reaches(holder, (A.App,)) is fake
    reads_ok = reads_other = False
    try:
        view.get("night")
    except KeyError:
        reads_other = True
    wrote_other = False
    try:
        view.put("night", True)
    except KeyError:
        wrote_other = True
    view.put("trends_zoom", 30)
    reads_ok = view.get("trends_zoom") == 30
    # the real station's wiring
    close_root()
    rig = S.Rig()
    try:
        real = walk_reaches(rig.ap.trendpanel, (A.App,)) is None and walk_reaches(rig.ap.trends, (A.App,)) is None
    finally:
        rig.gateway_like(False)
        rig.close()
    report("TG9 narrow view: the panel and the model hold no App (gc walk incl. the real station's wiring; the control "
           "that holds one is found); the view reads / writes only trends_* settings",
           clean and control and reads_other and wrote_other and reads_ok and real,
           "clean %s; control found %s; refuses other keys read %s / write %s; trends_* ok %s; real wiring clean %s" % (
               clean, control, reads_other, wrote_other, reads_ok, real))


# --- TG10 -------------------------------------------------------------------------------------------------
def tg10():
    import trend_load
    res = trend_load.measure_all(sizes=("small", "large"), windows=((1920, 1080), (1366, 768)), passes=1)
    lim_s, lim_r = trend_load.LIMITS["sampler_us"] * 3, trend_load.LIMITS["redraw_ms"] * 3
    report("TG10 CPU: the sampler <= %.0f us per frame and the whole panel (6 graphs, a full 60-minute ring, every "
           "zoom, 2 windows, 2 text sizes) <= %.0f ms per redraw (3 x the design limits 50 us / 10 ms; "
           "tools/trend_load.py gives the numbers)" % (lim_s, lim_r),
           res["sampler_us"] <= lim_s and res["worst_redraw_ms"] <= lim_r and
           res["observe_us"] <= trend_load.LIMITS["observe_us"] * 3 and
           res["detect_avg_ms"] <= trend_load.LIMITS["detect_ms"] * 3 and
           res["detect_p99_ms"] <= trend_load.LIMITS["detect_ms"] * 6,
           "sampler %.1f us per frame; worst redraw %.1f ms (data part %.1f ms); silent-highlight detector: filter "
           "%.1f us per frame, judging %.2f ms avg / %.2f ms p99 per 2 Hz sample (limits x3 / x6)" % (
               res["sampler_us"], res["worst_redraw_ms"], res["worst_data_ms"], res["observe_us"],
               res["detect_avg_ms"], res["detect_p99_ms"]))


# --- TR3: the silent highlight «ΤΑΣΗ» (TH1 - TH11) ------------------------------------------------------------
import random  # noqa: E402
import shutil  # noqa: E402
import tempfile  # noqa: E402
import json  # noqa: E402
import trenddetect  # noqa: E402
import trendlog  # noqa: E402


class Sim:
    """Telemetry at 10 Hz through the model + the detector exactly like App.trend_feed / the idle tick."""

    def __init__(self, seed=1, mode="AUTO", rate=10.0):
        self.rng = random.Random(seed)
        self.model = tr.Trends()
        self.events = []
        self.det = trenddetect.Detector()
        self.t = 1000.0
        self.k = 0
        self.mode = mode
        self.rate = rate
        self.base = dict(est_sog=8.0, throttle=40.0, heading=90.0, target=90.0, yaw_rate=0.0, att_roll=0.0,
                         att_pitch=3.0, slam_peak=0.3, slam_rate=2, bank_v=12.6, auto_wp=1, gear=1, att_age_ms=10)
        self.noise = dict(est_sog=0.08, yaw_rate=0.25, att_roll=0.25, att_pitch=0.12, slam_peak=0.03, bank_v=0.02)
        self.rtt = 60.0
        self.link_key = ("lte", False)
        self.drain = None

    def frame(self, over):
        d = {"t_ms": int(self.k * 100) % 2000000000, "mode_name": self.mode, "compass_ok": True}
        for key, v in self.base.items():
            d[key] = v + (self.rng.gauss(0, self.noise[key]) if key in self.noise else 0.0)
        d["slam_peak"] = abs(d["slam_peak"])
        d.update(over)
        return d

    def run(self, secs, over=None, fn=None, observe=True, rtt_noise=2.0):
        for _ in range(int(secs * self.rate)):
            self.t += 1.0 / self.rate
            self.k += 1
            o = dict(over or {})
            if fn:
                o.update(fn(self.t, self.k) or {})
            d = self.frame(o)
            rtt = self.rtt + self.rng.gauss(0, rtt_noise)
            if observe:
                self.det.observe(self.t, d, self.link_key)
            self.model.feed(self.t, d, None, None, rtt)
            self.model.tick(self.t)
            self.det.poll(self.model)
            if self.drain:
                self.drain()                # (the station's own door: log + event line)
            else:
                self.events.extend(self.det.take())

    def ons(self, val=None):
        return [e for e in self.events if e["ev"] == "on" and (val is None or e["val"] == val)]

    def offs(self, val=None):
        return [e for e in self.events if e["ev"] == "off" and (val is None or e["val"] == val)]


STEPS = {   # value -> the step (base override)
    "sog": {"est_sog": 11.0}, "roll": {"att_roll": 6.0}, "pitch": {"att_pitch": 6.5},
    "slam_g": {"slam_peak": 1.4}, "bank_v": {"bank_v": 11.8},
}


def apply_step(sim, vid):
    if vid == "link_ms":
        sim.rtt = 460.0
    else:
        sim.base.update(STEPS[vid])


def th1():
    out, ok = {}, True
    for n, vid in enumerate(list(STEPS) + ["link_ms"]):
        sim = Sim(seed=100 + n)
        sim.run(400.0)
        pre = len(sim.events)
        t_step = sim.t
        apply_step(sim, vid)
        sim.run(120.0)
        on = sim.ons(vid)
        want = "down" if vid == "bank_v" else "up"
        t_on = (on[0]["t"] - t_step) if on else None
        out[vid] = None if t_on is None else round(t_on, 1)
        ok = ok and pre == 0 and len(on) == 1 and len(sim.ons()) == 1 and on[0]["kind"] == "mean" and \
            on[0]["dir"] == want and trenddetect.ON_S <= t_on <= 70.0
    sim = Sim(seed=5)                       # a 6 s blip does not start it
    sim.run(400.0)
    sim.base.update(est_sog=11.0)
    sim.run(6.0)
    sim.base.update(est_sog=8.0)
    sim.run(120.0)
    blip = not sim.ons("sog")
    report("TH1 a clear step of the mean (speed +3 kn, roll +6, pitch +3.5, impact peak +1.1 g, battery -0.8 V, round "
           "trip +400 ms) -> ONE highlight with the right direction, after the 10 s hold and the half-window the "
           "median needs, never before; nothing before the step; a 6 s blip is nothing",
           ok and blip, "seconds from the step to ON %s; blip ignored %s" % (out, blip))


def th2():
    a = Sim(seed=2)                         # spread UP: an oscillation with the same median
    a.run(400.0)
    a.run(120.0, fn=lambda t, k: {"att_roll": 3.0 * math.sin(t / 8.0 * 2 * math.pi)})
    up = a.ons("roll")
    up_ok = len(up) == 1 and up[0]["kind"] == "spread" and up[0]["dir"] == "up"
    b = Sim(seed=3)                         # spread DOWN: a rhythm that stops (s_B >= 2 s_min)
    b.run(600.0, fn=lambda t, k: {"att_roll": 3.0 * math.sin(t / 8.0 * 2 * math.pi)})
    pre = len(b.ons("roll"))
    b.run(120.0)
    dn = b.ons("roll")
    dn_ok = len(dn) == 1 and dn[0]["kind"] == "spread" and dn[0]["dir"] == "down"
    c = Sim(seed=4)                         # rate of turn: spread only, a mean shift is nothing
    c.run(400.0)
    c.run(120.0, over={"yaw_rate": 1.5})
    rot_mean = not c.ons("rot")
    import trendpanel as tp
    arrows = (tp.arrow("spread", "up"), tp.arrow("spread", "down"), tp.arrow("mean", "up"), tp.arrow("mean", "down"))
    report("TH2 a change of the SPREAD (an oscillation added / removed) -> ∿↑ / ∿↓ (spread down only when the old spread "
           "was >= 2 s_min); rate of turn is judged on spread only (a mean shift is nothing)",
           up_ok and dn_ok and pre == 0 and rot_mean and arrows == ("∿↑", "∿↓", "↑", "↓"),
           "up %s; down %s (before the stop: %d); rot mean shift ignored %s; arrows %s" % (
               [(e['kind'], e['dir']) for e in up], [(e['kind'], e['dir']) for e in dn], pre, rot_mean, arrows))


def program(sim, observe=True, fault_at=None):
    """A simulated AUTO program: legs of 300 s, a waypoint turn (heading +60 deg, heel 5 deg, 7.5 deg/s for 8 s) and a
    throttle staircase between the turns (the speed follows with a 10 s lag)."""
    st = {"sog": 8.0, "wp": 1, "tgt": 90.0, "turned": set()}
    stairs = ((450, 55.0), (750, 70.0), (1050, 55.0), (1350, 40.0))
    turns = (300, 600, 900, 1200)
    t0 = sim.t

    def fn(t, k):
        r = t - t0
        thr = 40.0
        for tt, v in stairs:
            if r >= tt:
                thr = v
        for n, tt in enumerate(turns):
            if r >= tt and n not in st["turned"]:
                st["turned"].add(n)
                st["wp"] = n + 2
                st["tgt"] = (st["tgt"] + 60.0) % 360.0
        st["sog"] += (thr * 0.2 - st["sog"]) * (1.0 / sim.rate) / 10.0
        turning = any(0 <= r - tt < 8 for tt in turns)
        o = {"est_sog": st["sog"], "throttle": thr, "auto_wp": st["wp"], "target": st["tgt"],
             "att_roll": 5.0 if turning else 0.0, "yaw_rate": 7.5 if turning else 0.0}
        if fault_at is not None and r >= fault_at:
            o.update(slam_peak=1.6, slam_rate=14)
        return o
    sim.run(1500.0, fn=fn, observe=observe)


def th3():
    cases = []

    def case(name, seed, change, mode=None, link=None, rtt=None):
        sim = Sim(seed=seed)
        sim.run(400.0)
        sim.base.update(change)
        if mode:
            sim.mode = mode
        if link:
            sim.link_key = link
        if rtt:
            sim.rtt = rtt
        sim.run(300.0)
        cases.append((name, len(sim.ons())))
    case("F1 mode", 61, {"est_sog": 11.0}, mode="HDG_LOCK")
    case("F2 throttle", 62, {"est_sog": 11.0, "throttle": 70.0, "slam_peak": 1.4, "att_pitch": 6.5})
    case("F3 waypoint", 63, {"auto_wp": 2, "att_roll": 6.0, "est_sog": 11.0})
    case("F3 target", 64, {"target": 150.0, "att_roll": 6.0})
    case("F4 speed set", 65, {"speed_set_kn": 10.0, "est_sog": 11.0})
    case("F5 gear", 66, {"gear": 2, "est_sog": 11.0, "att_pitch": 6.5})
    case("F6 override", 67, {"ov_hand": 1, "att_roll": 6.0})
    case("F7 generator", 68, {"gen_active": 1, "bank_v": 11.8})
    case("F8 link", 69, {}, link=("sat", True), rtt=460.0)
    bad = [c for c in cases if c[1]]
    g = Sim(seed=9)                         # F9: a gap > 5 s in a value, then another level
    g.run(400.0)
    g.run(8.0, over={"att_roll": None, "att_age_ms": 5000})
    g.base["att_roll"] = 6.0
    g.run(300.0)
    f9 = len(g.ons("roll"))
    p = Sim(seed=11)
    program(p)
    ctl = Sim(seed=11)
    program(ctl, observe=False)
    report("TH3 every commanded change (F1 mode, F2 throttle, F3 waypoint / target, F4 speed set, F5 gear, F6 stick "
           "override, F7 generator, F8 link, F9 a gap) of the size of TH1's step -> NO highlight; a full simulated AUTO "
           "program (4 waypoint turns, a throttle staircase) = 0 highlights, the SAME run without the filter = some",
           not bad and f9 == 0 and len(p.ons()) == 0 and len(ctl.ons()) >= 1,
           "highlights per trigger case %s; F9 %d; program %d (without the filter %d: %s)" % (
               cases, f9, len(p.ons()), len(ctl.ons()), sorted({e['val'] for e in ctl.ons()})))


def th4():
    s = Sim(seed=21)                        # ON at >= 1, kept through 0.6..1, OFF after 20 s below 0.6 (>= 30 s shown)
    s.run(400.0)
    s.base["bank_v"] = 12.6 - 0.4           # ratio 1.33 (threshold 0.3 V)
    s.run(60.0)
    s.base["bank_v"] = 12.6 - 0.22          # ratio 0.73: between 0.6 and 1 -> stays on
    s.run(200.0)
    still = len(s.ons("bank_v")) == 1 and not s.offs("bank_v")
    s.base["bank_v"] = 12.6
    s.run(120.0)
    on, off = s.ons("bank_v"), s.offs("bank_v")
    pair = len(on) == 1 and len(off) == 1 and off[0]["why"] == "calm" and off[0]["dur_s"] >= trenddetect.MIN_SHOWN_S
    h = Sim(seed=22)                        # hovering at the threshold
    h.run(400.0)
    for i in range(40):
        h.base["bank_v"] = 12.6 - (0.32 if i % 2 else 0.28)
        h.run(7.0)
    hover = len(h.ons("bank_v")) <= 1 and len(h.offs("bank_v")) <= 1
    c = Sim(seed=23)                        # the cooldown
    c.run(400.0)
    c.base["bank_v"] = 11.8
    c.run(60.0)
    c.base["bank_v"] = 12.6
    c.run(100.0)
    coff = c.offs("bank_v")
    t_off = coff[0]["t"] if coff else None
    c.base["bank_v"] = 11.8                 # the same again soon after
    c.run(600.0)
    con = c.ons("bank_v")
    cool = bool(coff) and len(con) == 2 and con[1]["t"] - t_off >= trenddetect.COOLDOWN_S
    report("TH4 hysteresis + minimum time + cooldown: ON at >= 1 x the threshold, kept through 0.6..1, OFF only after "
           "20 s below 0.6 and >= 30 s shown; a statistic hovering at the threshold = at most one on / off pair; the "
           "same trend again within 5 min of its end is blocked until the cooldown is over",
           still and pair and hover and cool,
           "kept %s; pair %s (dur %s); hover on/off %d/%d; second on %s s after the first off" % (
               still, pair, off[0]["dur_s"] if off else None, len(h.ons("bank_v")), len(h.offs("bank_v")),
               round(con[1]["t"] - t_off) if len(con) > 1 and t_off else None))


def th5():
    a = Sim(seed=31)                        # too little BASE: 100 s of history, then a step
    a.run(100.0)
    a.base["est_sog"] = 11.0
    a.run(60.0)
    b = Sim(seed=32, rate=1.0)              # too little RECENT: one sample per second (30 < 45 in 30 s)
    b.run(400.0)
    b.base["est_sog"] = 11.0
    b.run(200.0)
    report("TH5 too little BASE (< 2 min) or RECENT (< 45 valid samples in the last 30 s) = nothing is judged",
           not a.ons() and not b.ons(), "base %d / recent %d highlights" % (len(a.ons()), len(b.ons())))


def th6():
    s = Sim(seed=41)                        # missing for 60 s, back at another level: a gap is not a change
    s.run(400.0)
    s.run(60.0, over={"att_roll": None})
    s.base["att_roll"] = 6.0
    s.run(100.0)
    back = not s.ons("roll")
    m = Sim(seed=42)
    m.run(600.0, over={"att_roll": None, "bank_v": None})
    none = not m.ons()
    g = Sim(seed=43)                        # a highlight is ended by a gap > 5 s
    g.run(400.0)
    g.base["att_roll"] = 6.0
    g.run(60.0)
    was = bool(g.det.active().get("roll"))
    g.run(8.0, over={"att_roll": None})
    ended = "roll" not in g.det.active() and bool(g.offs("roll")) and g.offs("roll")[-1]["why"] == "gap"
    report("TH6 missing data: a gap > 5 s ends a highlight (why gap) and restarts the value's BASE; a value that comes "
           "back at another level after a gap is not a trend; an all-missing value never highlights",
           back and none and was and ended, "back %s; all-missing %s; was on %s; ended by the gap %s" % (
               back, none, was, ended))


def th10():
    res = {}
    for mode in ("MANUAL", "REMOTE_MANUAL", "MANEUVER", "TRAINING", "BENCH_TEST", "STANDBY", "DP"):
        s = Sim(seed=51, mode=mode)
        s.run(400.0)
        s.base.update(est_sog=11.0, att_roll=6.0, att_pitch=6.5)
        s.run(120.0)
        res[mode] = sorted({e["val"] for e in s.ons()})
    driver_quiet = all(not set(v) & {"sog", "rot", "xte", "roll", "pitch"} for v in res.values())
    sl = {}
    for mode in ("MANUAL", "REMOTE_MANUAL", "STANDBY", "DP", "BENCH_TEST"):
        s = Sim(seed=52, mode=mode)
        s.run(400.0)
        s.base["slam_peak"] = 1.4
        s.run(100.0)
        sl[mode] = bool(s.ons("slam_g"))
    slam_ok = sl["MANUAL"] and sl["REMOTE_MANUAL"] and not (sl["STANDBY"] or sl["DP"] or sl["BENCH_TEST"])
    allm = True
    for mode in ("MANUAL", "STANDBY"):
        s = Sim(seed=53, mode=mode)
        s.run(400.0)
        apply_step(s, "link_ms")
        s.run(100.0)
        allm = allm and bool(s.ons("link_ms"))
    never = all(v not in trenddetect.CFG for v in trenddetect.NOT_JUDGED)
    f = Sim(seed=54)
    f.run(400.0, over={"first_test": True})
    f.base.update(est_sog=11.0)
    f.run(120.0, over={"first_test": True})
    ft = not f.ons("sog")
    report("TH10 the modes table: no driver-commanded value (speed, rate of turn, cross-track, roll, pitch) is judged "
           "in MANUAL / REMOTE / MANEUVER / TRAINING / BENCH / STANDBY / DP or FIRST TEST; impacts are judged in every "
           "mode but STANDBY / DP / BENCH; battery + link in every mode; thr / hdg / depth are never judged",
           driver_quiet and slam_ok and allm and never and ft,
           "values judged per mode %s; impacts per mode %s; link in every mode %s; first test quiet %s" % (
               res, sl, allm, ft))


def th11():
    """The 'simulator replay': the simulator runs in REAL time only (a 10 min baseline + an AUTO program would take
    longer than the gate), so the replay is the AUTO program of TH3 driven through the same decoded-dict path
    (App.trend_feed's calls) - 0 highlights; the same run with the SIM_FAULT slam values (impact peak + impacts per
    minute up) = a slam_g / slam_pm up. The fault values are the ones the simulator's slam fault produces."""
    p = Sim(seed=71)
    program(p)
    f = Sim(seed=71)
    program(f, fault_at=900.0)
    vals = {(e["val"], e["dir"]) for e in f.ons()}
    only_slam = {e["val"] for e in f.ons()} <= {"slam_g", "slam_pm"}
    report("TH11 replay of an AUTO run through the decoded-telemetry path: 0 highlights; the same run with the slam "
           "fault (impact peak + impacts per minute rise) = slam_g / slam_pm up and nothing else",
           not p.ons() and ("slam_g", "up") in vals and ("slam_pm", "up") in vals and only_slam,
           "clean run %d highlights; slam run %s" % (len(p.ons()), sorted(vals)))


def th8():
    d = tempfile.mkdtemp(prefix="trygons_th8_")
    old = dict(os.environ)
    try:
        os.environ["TRYGONS_TRENDS_LOG"] = d
        os.environ.pop("TRYGONS_TEST", None)
        folder_env = trendlog.log_dir() == d
        os.environ.pop("TRYGONS_TRENDS_LOG")
        os.environ["TRYGONS_TEST"] = "1"
        folder_test = os.path.basename(trendlog.log_dir()) == "trygons_test_trends"
        os.environ["TRYGONS_TRENDS_LOG"] = d
        s = Sim(seed=81)
        s.run(400.0)
        s.base["est_sog"] = 11.0
        s.run(100.0)
        s.base["est_sog"] = 8.0
        s.run(300.0)
        for e in s.events:
            trendlog.append(trendlog.entry(e, "rf49d", 1.7e9 + e["t"] - 1000.0), d)
        files = trendlog.log_files(d)
        lines = [json.loads(x) for fn in os.listdir(d) for x in open(os.path.join(d, fn), encoding="utf-8")]
        keys = {"ts", "boat", "ev", "val", "kind", "dir", "base_m", "base_s", "rec_m", "rec_s", "stat", "thr", "mode",
                "dur_s", "why"}
        fmt = bool(lines) and all(set(x) == keys for x in lines) and lines[0]["ev"] == "on" and \
            lines[0]["val"] == "sog" and lines[0]["dir"] == "up" and lines[0]["boat"] == "rf49d" and \
            lines[0]["ts"].endswith("Z") and lines[0]["rec_m"] > lines[0]["base_m"] + 2 and \
            any(x["ev"] == "off" and x["dur_s"] and x["why"] for x in lines)
        # size limit: a full file continues in a numbered part; 30 days: old files go, at most 5 per write
        d2 = tempfile.mkdtemp(prefix="trygons_th8b_")
        old_max = trendlog.MAX_FILE_BYTES
        trendlog.MAX_FILE_BYTES = 300
        names = {trendlog.append({"ev": "on", "x": "y" * 120}, d2, 1.7e9) for _ in range(6)}
        trendlog.MAX_FILE_BYTES = old_max
        split = len(names) >= 2 and any("_2" in n for n in names)
        d3 = tempfile.mkdtemp(prefix="trygons_th8c_")
        for i in range(8):
            open(os.path.join(d3, "2020-01-%02d.jsonl" % (i + 1)), "w").write("{}\n")
        open(os.path.join(d3, "notes.txt"), "w").write("keep")
        trendlog.append({"ev": "on"}, d3)
        left = sorted(os.listdir(d3))
        prune = len([n for n in left if n.startswith("2020")]) == 3 and "notes.txt" in left
        # a disk error is swallowed
        bad = trendlog.append({"ev": "on"}, os.path.join(d3, "notes.txt", "x"))
        report("TH8 the local log: one JSON line per on / off with the design's fields (UTC time, boat, value, kind, dir, "
               "base / recent mean + spread, statistic, threshold, mode, duration, why) in the log folder (a test run = "
               "a temp folder, an env override); a full 5 MB file continues in a numbered part; files over 30 days go "
               "(at most 5 per write, only its own names); a disk error is swallowed; the modules import no sender "
               "(TG6 / TG7)",
               folder_env and folder_test and fmt and split and prune and bad is None and len(files) >= 1,
               "env %s test-folder %s format %s (%d lines) split %s prune %s (%s) error swallowed %s" % (
                   folder_env, folder_test, fmt, len(lines), split, prune, left, bad is None))
    finally:
        os.environ.clear()
        os.environ.update(old)
        for x in (d,):
            shutil.rmtree(x, ignore_errors=True)


def th_rig_common(rig):
    """The station ready for a highlight run: the live simulator's frames off, the trend rings empty, the panel open,
    taps on the sound system and the local-alarm door."""
    ap = rig.ap
    ap.trend_feed = lambda d: None              # (the live simulator's frames would reset the synthetic ring)
    ap.trends.clear()
    ap.ov.collapsed.discard("trends")
    rig.size(1920, 1080, 1.0)
    taps = {"sound": 0, "local": 0}
    for name in ("request", "say", "alarm", "feedback", "assistant_speak"):
        if ap.sound is not None and hasattr(ap.sound, name):
            orig = getattr(ap.sound, name)

            def f(*a, _o=orig, **k):
                taps["sound"] += 1
                return _o(*a, **k)
            setattr(ap.sound, name, f)
    orig_local = ap.lamps.set_local

    def loc(*a, **k):
        taps["local"] += 1
        return orig_local(*a, **k)
    ap.lamps.set_local = loc
    return taps


def th7():
    close_root()
    d = tempfile.mkdtemp(prefix="trygons_th7_")
    os.environ["TRYGONS_TRENDS_LOG"] = d
    rig = S.Rig()
    ap = rig.ap
    try:
        taps = th_rig_common(rig)
        rig.pump(1.0)
        ap.render(force=True)
        lamps0 = [tuple(r) for r in ap.lamps.rows()]
        bits0 = (ap.t or {}).get("alarm_bits")
        s0 = dict(taps)
        sim = Sim(seed=91)
        sim.model, sim.det = ap.trends, ap.trenddet
        sim.drain = ap.trend_events
        sim.t = time.time() - 560.0
        sim.run(400.0)
        sim.base["est_sog"] = 11.0
        sim.run(140.0)
        on = ap.trenddet.active().get("sog")
        ap.trends.tick(time.time() + 1.0)
        rig.pump(0.5)
        ap.render(force=True)
        recs = [o["trend"] for o in ap.ov.ops if o.get("trend")]
        line = next((li for r in recs for li in r["lines"] if li["id"] == "sog"), None)
        rec1 = next((r for r in recs if r["graph"] == 1), None)
        red = bool(line) and line["hl"] and line["colour"] == theme.WARN and line["tag"] == "TREND: SPEED ↑"
        others = all(li["colour"] != theme.WARN for r in recs for li in r["lines"] if li["id"] != "sog")
        tags = (rec1 or {}).get("tags") or []
        outlined = bool(tags) and tags[0]["colour"] == theme.WARN
        items = [i for i in ap.cv.find_all() if ap.cv.type(i) == "rectangle" and
                 ap.cv.itemcget(i, "outline") == theme.WARN and ap.cv.itemcget(i, "fill") == ""]
        frame_item = len(items) >= 1                   # a 1 px outline, no fill: not a filled alarm box
        events = [e for e in ap.ev_texts() if e.startswith("TREND:")]
        files = sorted(os.listdir(d))
        logged = bool(files) and any('"ev": "on"' in open(os.path.join(d, f), encoding="utf-8").read() for f in files)
        settings.put("night", True)                    # NIGHT: the night palette's warn red, thicker
        ap.render(force=True)
        recs_n = [o["trend"] for o in ap.ov.ops if o.get("trend")]
        ln = next((li for r in recs_n for li in r["lines"] if li["id"] == "sog"), None)
        night_ok = bool(ln) and ln["colour"] == theme.NIGHT["warn"] and ln["width"] == trendpanel.HL_NIGHT_W and \
            all(li["colour"] in (theme.NIGHT["main"], theme.NIGHT["dim"], theme.NIGHT["warn"])
                for r in recs_n for li in r["lines"])
        settings.put("night", False)
        ap.ov.collapsed.add("trends")                  # collapsed: nothing drawn, the header not coloured
        ap.render(force=True)
        pan = next((q for q in ap.ov.panels if q.key == "trends"), None)
        hdr_red = False
        if pan is not None and pan.r:
            x0, y0, x1, y1 = pan.r
            for i in ap.cv.find_overlapping(x0, y0, x1, y1):
                for opt in ("fill", "outline"):
                    try:
                        if ap.cv.itemcget(i, opt) == theme.WARN:
                            hdr_red = True
                    except Exception:
                        pass
        no_ops = not [o for o in ap.ov.ops if o.get("trend")]
        ap.ov.collapsed.discard("trends")
        ap.render(force=True)
        rig.pump(1.0)
        lamps1 = [tuple(r) for r in ap.lamps.rows()]
        silent = lamps1 == lamps0 and (ap.t or {}).get("alarm_bits") == bits0 and taps["sound"] == s0["sound"] and \
            taps["local"] == s0["local"]
        report("TH7 SILENT: a highlight draws the line + label red (NIGHT: the night warn red, thicker) and an OUTLINED "
               "label «TREND: SPEED ↑» (a 1 px frame, no fill) inside the panel only; ONE station event line + the log; "
               "no sound call, no lamp / local-alarm change, alarm bits and rows unchanged; a collapsed panel draws "
               "nothing and its header has no colour",
               bool(on) and red and others and outlined and frame_item and len(events) == 1 and logged and night_ok and
               not hdr_red and no_ops and silent and not rig.errors,
               "on %s; red %s (%s); other lines not red %s; outlined %s; frame item %s; events %s; logged %s; night %s; "
               "collapsed header red %s / no ops %s; silent %s (taps %s); errors %s" % (
                   bool(on), red, (line or {}).get("tag"), others, outlined, frame_item, events[:1], logged, night_ok,
                   hdr_red, no_ops, silent, taps, rig.errors[:2]))
    finally:
        os.environ.pop("TRYGONS_TRENDS_LOG", None)
        settings.put("night", False)
        rig.gateway_like(False)
        rig.close()
        shutil.rmtree(d, ignore_errors=True)


def th9():
    close_root()
    d = tempfile.mkdtemp(prefix="trygons_th9_")
    os.environ["TRYGONS_TRENDS_LOG"] = d
    rig = S.Rig()
    ap = rig.ap
    try:
        th_rig_common(rig)
        beeps = []
        ap.root.bell = lambda *a: beeps.append(0)
        ap._beep = lambda f: beeps.append(f)
        sent = []
        ap.cmd = lambda f, p, w: sent.append(f)

        def alarm_run(with_highlight):
            ap.trends.clear()
            ap.trenddet.reset()
            sim = Sim(seed=92)
            sim.model, sim.det = ap.trends, ap.trenddet
            sim.t = time.time() - 560.0
            sim.run(400.0)
            if with_highlight:
                sim.base["est_sog"] = 11.0
            sim.run(140.0)
            hl = "sog" in ap.trenddet.active()
            del beeps[:]
            rig.extra.update(alarm_bits=0x001, alarm=True)           # COMPASS LOST (a WARNING)
            ap.lamps = alerts_mod.MasterLamps()
            rig.pump(2.6)
            ap.render(force=True)
            res = {"hl": hl, "warn": ap.lamps.on(alerts_mod.WARNING), "beeps": len(beeps) >= 1,
                   "rows": sorted(tuple(r) for r in ap.lamps.rows()),
                   "ack": bool([o for o in ap.ov.ops if o.get("tag") == "ack"]),
                   "alarm_text": sorted(str(o.get("text")) for o in ap.ov.ops if "alarm" in str(o.get("tag", "")))}
            rig.extra.update(alarm_bits=0, alarm=False)
            rig.pump(0.5)
            return res
        a = alarm_run(True)
        b = alarm_run(False)
        same = a["rows"] == b["rows"] and a["alarm_text"] == b["alarm_text"] and a["ack"] and b["ack"]
        report("TH9 a real WARNING at the same time: the MASTER lamp, the sound (beep), the alarm rows and the alarm panel "
               "are exactly as without any highlight (the trend never delays, hides or changes an alarm)",
               a["hl"] and not b["hl"] and a["warn"] and b["warn"] and a["beeps"] and b["beeps"] and same and
               not sent and not rig.errors,
               "highlight on in A %s / B %s; lamps %s / %s; beeps %s / %s; rows equal %s; alarm panel equal %s; sent %s; "
               "errors %s" % (a["hl"], b["hl"], a["warn"], b["warn"], a["beeps"], b["beeps"], a["rows"] == b["rows"],
                              a["alarm_text"] == b["alarm_text"], sent, rig.errors[:2]))
    finally:
        os.environ.pop("TRYGONS_TRENDS_LOG", None)
        rig.extra.update(alarm_bits=0, alarm=False)
        rig.gateway_like(False)
        rig.close()
        shutil.rmtree(d, ignore_errors=True)



# --- TR4: acceleration, the strip, the debrief, the replays ---------------------------------------------------------
def acc_frame(t_ms, lon, lat, vert, pk=None):
    pk = pk or (lon * 3, lat * 3, vert * 3)
    d = frame(t_ms=t_ms, est_sog=5.0)
    d.update({"acc_lon": lon, "acc_lat": lat, "acc_vert": vert, "acc_lon_pk": pk[0], "acc_lat_pk": pk[1],
              "acc_vert_pk": pk[2], "acc_n": 5})
    return d


def tj5():
    now = 9000.0
    m = tr.Trends()
    # two frames per slot: the mean of the averages, the largest MAGNITUDE (with its sign) of the peaks
    m.feed(now, acc_frame(0, 0.10, -0.20, 0.30, pk=(0.5, -0.8, 1.0)))
    m.feed(now + 0.1, acc_frame(100, 0.30, -0.40, 0.50, pk=(-0.9, -0.3, 0.7)))
    m.tick(now + 1.0)
    i = (m.n - 1) % tr.N
    got = (float(m.vals["acc_lon"][i]), float(m.vals["acc_lat"][i]), float(m.vals["acc_vert"][i]),
           float(m.vals["acc_lon_pk"][i]), float(m.vals["acc_lat_pk"][i]), float(m.vals["acc_vert_pk"][i]))
    want = (0.20, -0.30, 0.40, -0.9, -0.8, 1.0)
    sampler_ok = all(abs(a - b) < 1e-5 for a, b in zip(got, want))
    # no ACCEL (None) = a gap; the other values stay
    g = tr.Trends()
    d = frame(t_ms=0, est_sog=5.0)
    d.update({"acc_lon": None, "acc_lat": None, "acc_vert": None})
    g.feed(now, d)
    g.tick(now + 1)
    gap_ok = all(math.isnan(float(g.vals[k][(g.n - 1) % tr.N])) for k in tr.ACC_IDS + tr.PK_IDS) and \
        g.last("sog") == 5.0
    # the palette: three distinct colours, three distinct patterns (TG4 holds the pair / meaning-colour rules for them)
    cols = [theme.TREND[k] for k in tr.ACC_IDS]
    pats = [theme.TREND_DASH[k] for k in tr.ACC_IDS]
    pal_ok = len(set(cols)) == 3 and len(set(pats)) == 3
    # the drawing: 3 lines + 3 faint peak lines, ONE shared scale that contains zero, a zero line
    big = tr.Trends()
    for k in range(240):
        big.feed(now - 120 + k * 0.5, acc_frame(k * 500, 0.1 * math.sin(k / 8.0), -0.05, 0.2,
                                                pk=(0.4, -0.3, 1.5 if k == 100 else 0.6)))
    big.tick(now + 1.0)
    store = {"trends_show": [9, 1], "trends_zoom": 2}
    tp, ov, p, plots = draw(big, store, now=now)
    rec = plots.get(9, {}).get("trend", {})
    lines = rec.get("lines", [])
    shared = bool(lines) and len({(round(ln["lo"], 6), round(ln["hi"], 6)) for ln in lines}) == 1
    zero_inside = shared and lines[0]["lo"] < 0 < lines[0]["hi"]
    peaks = rec.get("peaks", [])
    peak_ok = len(peaks) >= 3 and {pk["id"] for pk in peaks} == set(tr.ACC_IDS)
    spike_ok = bool(lines) and lines[0]["hi"] >= 1.5          # the 1.5 g spike is inside the shared scale
    label_ok = bool(lines) and all("/" in ln["label"] for ln in lines)
    own = [ln["colour"] for ln in lines] == cols and [ln["dash"] for ln in lines] == pats
    nright = draw(big, store, night=True, now=now)[3].get(9, {}).get("trend", {}).get("lines", [])
    night_ok = bool(nright) and {ln["colour"] for ln in nright} <= {theme.NIGHT["main"], theme.NIGHT["dim"]} and \
        len({ln["dash"] for ln in nright}) == 3
    tp.readout_t = now - 60
    txt = tp.readout_text()
    read_ok = "LATERAL" in txt and i18n.T("trend.peak") in txt
    report("TJ5 acceleration graph (G9): the slot mean of the averages and the signed largest peak, no ACCEL = a gap, "
           "three colours and three patterns, 3 lines + 3 faint peak lines on ONE scale containing zero (the spike "
           "stays), average/peak in the labels, NIGHT = the dim red + the patterns, the click readout names the peak",
           sampler_ok and gap_ok and pal_ok and shared and bool(zero_inside) and peak_ok and spike_ok and label_ok
           and own and night_ok and read_ok,
           "slot %s; gap %s; palette %s; shared scale %s (zero inside %s); peak lines %d; spike %s; labels %s; own %s; "
           "night %s; readout %s" % ([round(x, 3) for x in got], gap_ok, pal_ok, shared, bool(zero_inside),
                                     len(peaks), spike_ok, [ln.get("label") for ln in lines], own, night_ok, read_ok))


def tj1():
    import hud
    import trendgraph
    now = 9000.0
    m = tr.Trends()
    for k in range(240):
        m.feed(now - 120 + k * 0.5, acc_frame(k * 500, 0.1, -0.05, 0.2))
    m.tick(now + 1)
    ls = trendgraph.lines(m, 1, 120.0, now, 100, False)
    runs_ok = all(0.0 <= y <= 1.0 and 0.0 <= x <= 1.0 for ls_ in ls for run in ls_["runs"] for x, y in run)
    # dash pieces of a straight line: on/off = 6/3 px over 60 px -> 7 pieces, each at most 6 px long, all on the line
    pieces = trendgraph.dash_runs([(0.0, 0.0), (60.0, 0.0)], (6, 3))
    dash_ok = len(pieces) == 7 and all(abs(pc[-1][0] - pc[0][0]) <= 6.0 + 1e-6 for pc in pieces) and \
        trendgraph.dash_runs([(0, 0), (9, 0)], ()) == [[(0, 0), (9, 0)]]
    h = hud.Hud()
    h.W, h.H = 1920, 1080
    h.pal = hud.palette("green", False)
    fs = theme.hud_sizes(1080)
    h.f_small = fs["small"]
    h.ops, h.hits = [], []
    strip = [{"g": g, "lines": trendgraph.lines(m, g, 120.0, now, 140, False)} for g in (1, 4, 9)]
    h.trends("trends", hud.Snapshot(trend_strip=strip))
    box = h.R("trends")
    lines_ = [o for o in h.ops if o["k"] == "line" and o["tag"] == "trends" and o["fill"] != theme.TREND_GRID]
    inside = bool(lines_) and all(box[0] - 1 <= o["pts"][j] <= box[2] + 1 and box[1] - 1 <= o["pts"][j + 1] <= box[3] + 1
                                  for o in lines_ for j in range(0, len(o["pts"]), 2))
    colours = {o["fill"] for o in lines_}
    own = {theme.TREND["sog"], theme.TREND["thr"], theme.TREND["roll"], theme.TREND["acc_lon"],
           theme.TREND["acc_lat"], theme.TREND["acc_vert"]} <= colours
    caps = [o["s"] for o in h.ops if o["k"] == "text"]
    cap_ok = any("LONGITUDINAL" in c for c in caps) and any("SPEED" in c for c in caps)
    h.ops = []
    h.trends("trends", hud.Snapshot(trend_strip=None))
    off_ok = not h.ops                                   # nothing without a strip (off by default)
    h.ops = []
    h.pal = hud.palette("green", True)
    h.trends("trends", hud.Snapshot(trend_strip=[{"g": g, "lines": trendgraph.lines(m, g, 120.0, now, 140, True)}
                                                 for g in (1, 9)]))
    night = {o["fill"] for o in h.ops if o["k"] == "line" and o["fill"] != theme.TREND_GRID}
    night_ok = bool(night) and night <= {theme.NIGHT["main"], theme.NIGHT["dim"]}
    report("TJ1 screen-1 strip (model side): lines normalised 0..1 per run, dash pieces for the Pillow renderer, all "
           "lines inside the box, each value its own colour, captions with the newest value, nothing without a strip, "
           "NIGHT = the dim red; placement / MINIMAL / layout v4 = test_driver I26",
           runs_ok and dash_ok and inside and own and cap_ok and off_ok and night_ok,
           "runs %s; dash %s (%d pieces); inside %s; own colours %s; captions %s; off %s; night %s"
           % (runs_ok, dash_ok, len(pieces), inside, own, caps[:3], off_ok, night_ok))


class FakeLog:
    """A recorded black box in the shape tlg.read returns: 20 Hz samples + ACCEL events (one per second)."""

    def mode_name(self, m):
        return proto.MODES[m] if 0 <= m < len(proto.MODES) else "?"

    def alarm_text(self, bits):
        return ""

    def __init__(self, seconds=90, with_accel=True):
        self.samples, self.events, self.events_def = [], [], {31: ("ACCEL", [])}
        self.pc_commands, self.crc_errors = [], 0
        for k in range(seconds * 20):
            t = k * 50
            self.samples.append({"t_ms": t, "mode": 3, "est_sog": 6.0 + (3.0 if 40000 <= t < 50000 else 0.0),
                                 "thr_out": 45.0, "heading": 90.0, "yaw_rate": 0.5, "slam_peak": 0.3,
                                 "slam_rate": 2, "bank_v": 12.5, "xte": 4.0})
        if with_accel:
            for sec in range(1, seconds):
                avg = (int(0.1 * 1000) & 0xFFFF) | ((int(0.6 * 1000) & 0xFFFF) << 16)
                lat = (int(-0.05 * 1000) & 0xFFFF) | ((int(-0.2 * 1000) & 0xFFFF) << 16)
                vert = (int(0.2 * 1000) & 0xFFFF) | ((int(1.2 * 1000) & 0xFFFF) << 16)
                self.events.append((sec * 1000, "ACCEL", 31, [avg, lat, vert, 20]))


def tj2():
    import trendreplay
    import tlg
    log = FakeLog()
    rows = tlg.accel_rows(log)
    model, end = trendreplay.from_log(log, rows, proto.MODES)
    i = (model.n - 1) % tr.N
    v = {k: float(model.vals[k][i]) for k in tr.IDS + tr.PK_IDS}
    for k in tr.ACC_IDS + tr.PK_IDS:                 # the last ACCEL event is a second before the last sample: take the newest value
        fin_ = [x for x in model.series(k, 1e9, end + 5)[1] if x == x]
        v[k] = fin_[-1] if fin_ else float("nan")
    ser_t, ser_v = model.series("sog", 1e9, end + 5)
    fin = [x for x in ser_v if x == x]
    step_ok = max(fin) > 8.5 and min(fin) < 6.5          # the +3 kn step is there
    fields_ok = abs(v["thr"] - 45.0) < 0.1 and abs(v["rot"] - 0.5) < 0.01 and abs(v["bank_v"] - 12.5) < 0.01 and \
        abs(v["slam_g"] - 0.3) < 0.01
    acc_ok = abs(v["acc_lon"] - 0.1) < 1e-3 and abs(v["acc_lon_pk"] - 0.6) < 1e-3 and abs(v["acc_vert_pk"] - 1.2) < 1e-3 \
        and abs(v["acc_lat"] + 0.05) < 1e-3
    gaps = all(math.isnan(v[k]) for k in ("roll", "pitch", "depth", "link_ms", "link_loss"))
    xte_ok = abs(v["xte"] - 4.0) < 0.01                  # AUTO only
    thin_ok = model.count() <= 2 * 90 + 2                # the 20 Hz log is thinned to the same 2 Hz rings
    m2, _e = trendreplay.from_log(FakeLog(with_accel=False), [], proto.MODES)
    no_acc = math.isnan(float(m2.vals["acc_lon"][(m2.n - 1) % tr.N])) and m2.last("sog") == 6.0
    avail = trendreplay.available(model, (1, 2, 5, 9, 6))
    avail_ok = avail == [1, 2, 5, 9]                      # depth (G6) has nothing: not drawn
    # the logview pane draws it (a real Viewer on a Tk root, tlg.read replaced by the recording)
    import tkinter as tk
    import logview
    root = tk._default_root or tk.Tk()
    top = tk.Toplevel(root)
    top.geometry("1200x800+0+0")
    orig_read = tlg.read
    tlg.read = lambda path: log
    pane_ok = click_ok = False
    try:
        vw = logview.Viewer(top)
        top.update()
        vw.load("fake.TLG")
        top.update()
        vw.cursor = 700
        vw.draw_trends()
        top.update()
        rec = vw.trec
        pane_ok = rec is not None and [g["g"] for g in rec["graphs"]] == [1, 2, 5, 9] and \
            len(vw.trend.find_all()) > 20
        x0, x1 = rec["plot"] if rec else (0, 1)

        class E:        # a click at the middle of the plot moves the cursor to the middle of the recording
            x = (x0 + x1) / 2
        vw.trend_click(E)
        click_ok = abs(vw.t[vw.cursor] - (vw.t[0] + vw.t[-1]) / 2) < 3.0
    finally:
        tlg.read = orig_read
        top.destroy()
    report("TJ2 debrief: a recorded black box (20 Hz samples + ACCEL events) gives the same 2 Hz rings (speed step, "
           "throttle, rate of turn, impacts, battery, cross-track in AUTO, the three acceleration axes with their "
           "peaks); roll / pitch / depth / link are gaps; no ACCEL events = no acceleration; logview.py draws the pane "
           "(graphs 1 2 5 9) and a click moves the cursor",
           step_ok and fields_ok and acc_ok and gaps and xte_ok and thin_ok and no_acc and avail_ok and pane_ok and click_ok,
           "step %s; fields %s; acc %s; gaps %s; xte %s; thinned %s (%d); no accel %s; available %s; pane %s; click %s"
           % (step_ok, fields_ok, acc_ok, gaps, xte_ok, thin_ok, model.count(), no_acc, avail, pane_ok, click_ok))


def tj3():
    import trendreplay
    # (a) a scenario replay: 5 Hz samples with the keys scenario.py writes
    samples = []
    for k in range(300):
        s_ = {"t": k * 0.2, "mode": "AUTO", "hdg": 90.0, "sog": 5.0 + (2.0 if k > 150 else 0.0), "thr": 40.0,
              "rot": 0.3, "slam": 0.4, "pm": 3, "alm": 0}
        if k % 5 == 0:
            s_.update({"al": 0.1, "al_pk": 0.4, "at": -0.02, "at_pk": -0.1, "av": 0.2, "av_pk": 0.9})
        samples.append(s_)
    m, end = trendreplay.from_samples(samples)
    i = (m.n - 1) % tr.N
    a_ok = abs(float(m.vals["sog"][i]) - 7.0) < 0.01 and abs(float(m.vals["thr"][i]) - 40.0) < 0.01 and \
        abs(float(m.vals["slam_g"][i]) - 0.4) < 0.01
    acc_present = trendreplay.available(m, (9,)) == [9]
    # an OLD replay (only hdg / sog / thr) still works: the rest is a gap
    old = [{"t": k * 0.2, "mode": "AUTO", "hdg": 90.0, "sog": 5.0, "thr": 40.0, "alm": 0} for k in range(100)]
    mo, _e = trendreplay.from_samples(old)
    old_ok = mo.last("sog") == 5.0 and math.isnan(mo.last("slam_g")) and trendreplay.available(mo, (1, 5, 9)) == [1]
    # (b) a video lesson's hud.jsonl rows
    rows = [{"t": k * 0.2, "d": {"est_sog": 4.0, "throttle": 30.0, "heading": 10.0, "yaw_rate": 0.1, "bank_v": 12.4}}
            for k in range(200)]
    mh, _e = trendreplay.from_hud(rows)
    hud_ok = mh.last("sog") == 4.0 and abs(mh.last("bank_v") - 12.4) < 1e-6 and math.isnan(mh.last("slam_g"))
    # (c) the academy records the new keys into its replay (a real timeline sample) + the drawing needs no boat link
    sys.path.insert(0, os.path.join(HERE, "..", "academy"))
    import scenario
    run = scenario.Run.__new__(scenario.Run)
    run.timeline, run.t_sample, run.t0, run.origin, run.hazard, run.status = [], None, 0.0, None, None, "react"
    run.shown_bits = lambda: 0
    run._sample({"mode_name": "AUTO", "heading": 90.0, "est_sog": 5.0, "throttle": 40.0, "yaw_rate": 0.3,
                 "slam_peak": 0.4, "slam_rate": 3, "acc_lon": 0.1, "acc_lon_pk": 0.4, "acc_lat": -0.02,
                 "acc_vert": 0.2, "acc_vert_pk": 0.9}, 1.0)
    smp = run.timeline[0] if run.timeline else {}
    write_ok = smp.get("rot") == 0.3 and smp.get("slam") == 0.4 and smp.get("pm") == 3 and smp.get("al") == 0.1 and \
        smp.get("av_pk") == 0.9 and smp.get("at") == -0.02
    cv = canvas()
    cv.delete("all")
    rec = trendreplay.draw(cv, m, trendreplay.available(m, (1, 2, 5, 9)), end, 360, 200, cursor_t=end / 2,
                           name=lambda v: i18n.T("trend." + v))
    draw_ok = [g["g"] for g in rec["graphs"]] == [1, 2, 5, 9] and len(cv.find_all()) > 30
    report("TJ3 academy replays: a scenario replay (new keys: rate of turn, impacts, acceleration) and an old one "
           "(speed / throttle / heading only = gaps elsewhere) and a video lesson's hud.jsonl go through the same model; "
           "scenario.py records the new keys; the Tk drawing needs no boat link",
           a_ok and acc_present and old_ok and hud_ok and write_ok and draw_ok,
           "replay %s (accel %s); old replay %s; hud.jsonl %s; scenario writes the keys %s %s; drawing %s"
           % (a_ok, acc_present, old_ok, hud_ok, write_ok, smp, draw_ok))


def tj4():
    root_ = os.path.join(HERE, "..")
    en = open(os.path.join(root_, "MANUAL.txt"), encoding="utf-8").read()
    el = open(os.path.join(root_, "MANUAL_EL.txt"), encoding="utf-8").read()
    en_ok = "26. TRENDS" in en and "THE SILENT RED" in en and "ACCELERATION (graph 9)" in en and "NOT an alarm" in en \
        and "NOT engine vibration" in en and "ON SCREEN 1" in en and "not a safety function" in en
    el_ok = "21. ΤΑΣΕΙΣ" in el and "ΣΙΩΠΗΛΗ ΚΟΚΚΙΝΗ" in el and "ΕΠΙΤΑΧΥΝΣΗ (γράφημα 9)" in el and "δεν είναι συναγερμός" in el \
        and "ΔΕΝ είναι κραδασμοί" in el and "ΣΤΗΝ ΟΘΟΝΗ 1" in el and "όχι λειτουργία ασφάλειας" in el
    keys = ["trend.acc_lon", "trend.acc_lat", "trend.acc_vert", "trend.g9", "trend.peak", "trend.hud_toggle",
            "trend.dbg_none", "trend.dbg_note", "hud.ov_trends"]
    lang_ok = all(i18n.has(k, "el") and i18n.has(k, "en") for k in keys)
    greek_ok = not any(c.isascii() and c.isalpha() for c in i18n.TL("el", "trend.acc_lon")) and \
        i18n.TL("el", "trend.g9") != i18n.TL("en", "trend.g9")
    import json
    tour = json.load(open(os.path.join(root_, "academy", "content", "tour.json"), encoding="utf-8"))
    stops = {st["id"]: st for ch in tour["chapters"] for st in ch["stops"]}
    tour_ok = all(stops.get(i, {}).get("voice_pending") for i in ("s2.trends", "s1.trends")) and \
        stops["s1.trends"]["covers"] == ["hud:trends"] and "επιτάχυνση" in stops["s2.trends"]["text_el"] and \
        "acceleration" in stops["s2.trends"]["text_en"]
    shots = os.path.join(root_, "docs", "screens", "trends")
    names = ("screen2_trends_el.png", "screen2_trends_en.png", "screen1_strip_el.png", "screen1_strip_en.png")
    shots_ok = all(os.path.isfile(os.path.join(shots, n_)) and os.path.getsize(os.path.join(shots, n_)) > 5000
                   for n_ in names)
    report("TJ4 docs: MANUAL.txt ch. 26 + MANUAL_EL.txt ch. 21 (what the graphs show, the acceleration axes, the silent red "
           "label is not an alarm, honest limits, the strip, looking back), the lang keys el + en, the tour stops "
           "s2.trends (acceleration) + s1.trends (voice_pending), the screenshots in docs/screens/trends/",
           en_ok and el_ok and lang_ok and greek_ok and tour_ok and shots_ok,
           "manual en %s; manual el %s; lang keys %s; greek %s; tour %s; screenshots %s" % (
               en_ok, el_ok, lang_ok, greek_ok, tour_ok, shots_ok))


def main():
    settings.put("ui_lang", "en")
    i18n.set_lang("en", save=False)
    for fn in (tg1, tg2, tg3, tg4, tg5, tg6, tg7, tg8, tg9, tg10, tj5, tj1, tj2, tj3, tj4, th1, th2, th3, th4, th5, th6, th7, th8, th9,
               th10, th11):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__, False, "crashed: %r" % e)
    bad = results.count(False)
    print("TREND TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
