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

results = []
NAN = float("nan")
TREND_FILES = ("trends.py", "trendpanel.py", "trenddetect.py", "trendlog.py")


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
    want_t = now - 600.0 + 0.5 * 600.0
    t_ok = tp.readout_t is not None and abs(tp.readout_t - want_t) < 1.0
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
    res = trend_load.measure_all(sizes=("small", "large"), windows=((1920, 1080), (1366, 768)))
    lim_s, lim_r = trend_load.LIMITS["sampler_us"] * 3, trend_load.LIMITS["redraw_ms"] * 3
    report("TG10 CPU: the sampler <= %.0f us per frame and the whole panel (6 graphs, a full 60-minute ring, every "
           "zoom, 2 windows, 2 text sizes) <= %.0f ms per redraw (3 x the design limits 50 us / 10 ms; "
           "tools/trend_load.py gives the numbers)" % (lim_s, lim_r),
           res["sampler_us"] <= lim_s and res["worst_redraw_ms"] <= lim_r,
           "sampler %.1f us per frame; worst redraw %.1f ms (data part %.1f ms)" % (
               res["sampler_us"], res["worst_redraw_ms"], res["worst_data_ms"]))


def main():
    settings.put("ui_lang", "en")
    i18n.set_lang("en", save=False)
    for fn in (tg1, tg2, tg3, tg4, tg5, tg6, tg7, tg8, tg9, tg10):
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
