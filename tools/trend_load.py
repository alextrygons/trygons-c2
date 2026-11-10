"""CPU cost of the trend panel «ΤΑΣΕΙΣ» (step TR2, docs/trends_design.md 9): never slows screen 1.

    python tools/trend_load.py                 (a table; exit 0 = every limit kept)
    python tools/trend_load.py --json

What it measures (all on this PC, no boat, no network; the ring is FULL = 60 min, 6 graphs open):
  - sampler: microseconds per telemetry frame at 10 Hz (trends.Trends.feed; limit 50 us average)
  - series + decimation: ms per redraw of the six graphs at every zoom (the cached path = the same data again)
  - panel redraw: ms for the whole panel drawn on a canvas at 1920x1080 and 1366x768, every text size
    (limit 10 ms; the redraw is at most 2 Hz of NEW data and only while the panel is open)
The numbers go into TEST_REPORT.md; test_trends TG10 uses 3x looser bounds (loaded PCs).
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "groundstation"))

LIMITS = {"sampler_us": 50.0, "redraw_ms": 10.0, "detect_ms": 1.0, "observe_us": 50.0, "strip_ms": 2.0}


def frame(k, t_ms):
    return {"t_ms": t_ms, "mode": 3, "mode_name": "AUTO", "est_sog": 8.0 + math.sin(k / 30.0), "throttle": 40.0,
            "heading": (90.0 + k * 0.05) % 360.0, "yaw_rate": math.sin(k / 9.0), "compass_ok": True,
            "att_roll": 2.0 * math.sin(k / 12.0), "att_pitch": 3.0, "slam_peak": 0.2, "slam_rate": 2,
            "bank_v": 12.6, "acc_lon": 0.1 * math.sin(k / 7.0), "acc_lat": -0.03, "acc_vert": 0.2 + 0.1 * math.sin(k / 5.0),
            "acc_lon_pk": 0.4, "acc_lat_pk": -0.1, "acc_vert_pk": 0.9}


def fill_model(tr, minutes=60):
    now = time.time()
    t0 = now - minutes * 60.0
    n = int(minutes * 60 * 10)
    for k in range(n):                      # 10 Hz frames
        tr.feed(t0 + k * 0.1, frame(k, k * 100), 5.0 if k % 50 == 0 else None, 12.0, 40.0)
    tr.tick(now + 1.0)
    return now


def measure_sampler():
    import trends
    tr = trends.Trends()
    n = 20000
    t0 = time.perf_counter()
    base = time.time()
    for k in range(n):
        tr.feed(base + k * 0.1, frame(k, k * 100), None, 12.0, 40.0)
    return (time.perf_counter() - t0) / n * 1e6


def measure_detector(minutes=25):
    """The silent highlight (TR3): per telemetry frame the commanded-change filter (observe, us) and per 2 Hz sample the
    judgement of all 10 values (poll, ms) with the 10-minute buffers full. avg / p99 / max."""
    import trends
    import trenddetect
    tr = trends.Trends()
    det = trenddetect.Detector()
    now = time.time() - minutes * 60.0
    n = int(minutes * 60 * 10)
    obs, polls = [], []
    for k in range(n):
        t = now + k * 0.1
        d = frame(k, k * 100)
        d.update(target=90.0, auto_wp=1, gear=1, power_fwd=100, speed_limited=False)
        t0 = time.perf_counter()
        det.observe(t, d, ("lte", False))
        t1 = time.perf_counter()
        tr.feed(t, d, 5.0 if k % 50 == 0 else None, 12.0, 40.0)
        tr.tick(t)
        t2 = time.perf_counter()
        before = det.done
        det.poll(tr)
        t3 = time.perf_counter()
        if k > n // 2:
            obs.append((t1 - t0) * 1e6)
            if det.done != before:
                polls.append((t3 - t2) * 1000.0)
    polls.sort()
    return {"observe_us": sum(obs) / len(obs), "detect_avg_ms": sum(polls) / max(1, len(polls)),
            "detect_p99_ms": polls[int(len(polls) * 0.99)], "detect_max_ms": polls[-1]}


def measure_redraw(W, H, size, reps=8):
    """Draw the open panel (6 graphs, a full ring) on a canvas: ms per redraw (the median) and the data part alone."""
    import tkinter as tk
    import theme
    import trends
    import trendpanel
    import overlay
    root = tk._default_root
    own = root is None
    if own:
        root = tk.Tk()
        root.withdraw()
    try:
        theme.set_text_size(size)
        cv = tk.Canvas(root, width=W, height=H)
        cv.pack()
        tr = trends.Trends()
        now = fill_model(tr)
        store = {"trends_zoom": 60}
        view = trendpanel.TrendView(store.get, store.__setitem__, lambda: False)
        tp = trendpanel.TrendPanel(tr, view)
        ov = overlay.Overlay(cv)
        ov.dpi = 1.0
        out = {}
        for zoom in trends.ZOOMS:
            store["trends_zoom"] = zoom
            times, data_ms = [], []
            first = None
            for rep in range(reps):
                if rep == 0:                    # the FIRST draw after opening the panel / changing the zoom: empty caches
                    tr._colc.clear()
                    tr._lin_gen = -1
                tp._cache.clear()
                ov.begin(W, H)
                ov.k = 1.0
                p = ov.panel("trends", "T", "ne")
                tp.fill(ov, p, now)
                ov._measure_panel(p)
                p.r = (10, 10, 10 + p.w, 10 + p.h)
                ov.ops, ov.hits, ov.holds, ov.heads, ov.hits_xy = [], [], [], [], []
                cv.delete("all")
                t0 = time.perf_counter()
                ov.draw_panel(p)
                dt = (time.perf_counter() - t0) * 1000.0
                if rep == 0:
                    first = dt
                    continue
                times.append(dt)
                t1 = time.perf_counter()
                for vid in trends.IDS:
                    tr.segments(vid, zoom * 60.0, now, 300)
                data_ms.append((time.perf_counter() - t1) * 1000.0)
            times.sort()
            data_ms.sort()
            out[zoom] = {"redraw_ms": times[len(times) // 2], "data_ms": data_ms[len(data_ms) // 2], "first_ms": first}
        return out
    finally:
        theme.set_text_size("normal")
        if own:
            root.destroy()


def measure_strip(reps=20):
    """TR4: screen 1's edge strip - the model side (trendgraph.lines of 3 graphs at the box's width) + building the HUD
    ops of the box (hud.Hud.trends), ms per frame; the driver caches the lines per half second, so the per-frame cost is
    the ops alone (the 'ops' number) and the 2 Hz cost the whole thing."""
    import hud
    import theme
    import trendgraph
    import trends
    tr = trends.Trends()
    now = fill_model(tr)
    h = hud.Hud()
    h.W, h.H = 1920, 1080
    h.pal = hud.palette("green", False)
    h.f_small = theme.hud_sizes(1080)["small"]
    cols = int(1920 * hud.DEFAULT_LAYOUT["trends"][2] / 3.0)       # driver.trend_strip: one column per 3 px
    both, ops_only, hit_only = [], [], []
    strip = None
    for _ in range(reps):
        t0 = time.perf_counter()
        strip = [{"g": g, "lines": trendgraph.lines(tr, g, 600.0, now, cols, False)} for g in (1, 4, 9)]
        t1 = time.perf_counter()
        h.ops, h.hits = [], []
        h.trends("trends", hud.Snapshot(trend_strip=strip))       # the ops of a new strip (every 0.5 s)
        t2 = time.perf_counter()
        h.ops, h.hits = [], []
        h.trends("trends", hud.Snapshot(trend_strip=strip))       # every other frame: the same strip = reused ops
        t3 = time.perf_counter()
        both.append((t2 - t0) * 1000.0)
        ops_only.append((t2 - t1) * 1000.0)
        hit_only.append((t3 - t2) * 1000.0)
        tr._lin_gen = -1                  # new data every time: a cold cache, as at 2 Hz
    both.sort()
    ops_only.sort()
    hit_only.sort()
    m = len(both) // 2
    return {"strip_2hz_ms": both[m], "strip_ops_ms": ops_only[m], "strip_frame_ms": hit_only[m], "strip_ops": len(h.ops)}


def measure_redraw_best(W, H, size, passes=2):
    """measure_redraw run `passes` times, every cell the LOWER of the passes' medians: a burst of OS noise (the PC's
    other work, a scan, a GC of the whole process) lasts longer than one median of 8 draws and shows up as one slow
    cell (TR4: single 9-11 ms cells next to 5-6 ms neighbours); the real cost does not change between passes, noise does.
    The first-draw figure is kept as the lower of the passes too."""
    best = None
    for _ in range(passes):
        r = measure_redraw(W, H, size)
        if best is None:
            best = r
        else:
            for z, v in r.items():
                for k in ("redraw_ms", "data_ms", "first_ms"):
                    best[z][k] = min(best[z][k], v[k])
    return best


def measure_all(sizes=("small", "normal", "large"), windows=((1920, 1080), (1366, 768)), passes=2):
    res = {"sampler_us": measure_sampler(), "redraw": {}}
    res.update(measure_detector())
    res.update(measure_strip())
    for W, H in windows:
        for sz in sizes:
            res["redraw"]["%dx%d %s" % (W, H, sz)] = measure_redraw_best(W, H, sz, passes)
    res["worst_redraw_ms"] = max(v["redraw_ms"] for r in res["redraw"].values() for v in r.values())
    res["worst_data_ms"] = max(v["data_ms"] for r in res["redraw"].values() for v in r.values())
    res["worst_first_ms"] = max(v["first_ms"] for r in res["redraw"].values() for v in r.values())
    return res


def main():
    res = measure_all()
    if "--json" in sys.argv:
        print(json.dumps(res, indent=1))
    else:
        print("sampler: %.1f us per frame (limit %.0f)" % (res["sampler_us"], LIMITS["sampler_us"]))
        for k, r in res["redraw"].items():
            print("%-18s " % k + "  ".join("%2d min: draw %.1f ms (data %.1f)" % (z, v["redraw_ms"], v["data_ms"])
                                          for z, v in r.items()))
        print("detector: observe %.1f us per frame (limit %.0f), judging all values %.3f ms avg / %.3f p99 / %.3f max per "
              "2 Hz sample (limit %.0f)" % (res["observe_us"], LIMITS["observe_us"], res["detect_avg_ms"],
                                            res["detect_p99_ms"], res["detect_max_ms"], LIMITS["detect_ms"]))
        print("screen-1 strip: %.2f ms per 2 Hz refresh (lines + ops, limit %.0f), %.3f ms per frame while the strip is the "
              "same (ops reused, %d ops; limit %.0f)" % (res["strip_2hz_ms"], LIMITS["redraw_ms"], res["strip_frame_ms"],
                                                         res["strip_ops"], LIMITS["strip_ms"]))
        print("worst redraw %.1f ms (limit %.0f), worst data part %.1f ms; the FIRST draw after opening / a zoom change "
              "(every cache empty) worst %.1f ms" % (res["worst_redraw_ms"], LIMITS["redraw_ms"], res["worst_data_ms"],
                                                     res["worst_first_ms"]))
    ok = res["sampler_us"] <= LIMITS["sampler_us"] and res["worst_redraw_ms"] <= LIMITS["redraw_ms"] and         res["observe_us"] <= LIMITS["observe_us"] and res["detect_avg_ms"] <= LIMITS["detect_ms"] and res["detect_p99_ms"] <= LIMITS["detect_ms"]*2 and res["strip_frame_ms"] <= LIMITS["strip_ms"] and res["strip_2hz_ms"] <= LIMITS["redraw_ms"]
    print("TREND LOAD: %s" % ("OK" if ok else "OVER A LIMIT"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
