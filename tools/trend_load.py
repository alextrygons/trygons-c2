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

LIMITS = {"sampler_us": 50.0, "redraw_ms": 10.0}


def frame(k, t_ms):
    return {"t_ms": t_ms, "mode": 3, "mode_name": "AUTO", "est_sog": 8.0 + math.sin(k / 30.0), "throttle": 40.0,
            "heading": (90.0 + k * 0.05) % 360.0, "yaw_rate": math.sin(k / 9.0), "compass_ok": True,
            "att_roll": 2.0 * math.sin(k / 12.0), "att_pitch": 3.0, "slam_peak": 0.2, "slam_rate": 2,
            "bank_v": 12.6}


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
            for rep in range(reps):
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
                times.append((time.perf_counter() - t0) * 1000.0)
                t1 = time.perf_counter()
                for vid in trends.IDS:
                    tr.segments(vid, zoom * 60.0, now, 300)
                data_ms.append((time.perf_counter() - t1) * 1000.0)
            times.sort()
            data_ms.sort()
            out[zoom] = {"redraw_ms": times[len(times) // 2], "data_ms": data_ms[len(data_ms) // 2]}
        return out
    finally:
        theme.set_text_size("normal")
        if own:
            root.destroy()


def measure_all(sizes=("small", "normal", "large"), windows=((1920, 1080), (1366, 768))):
    res = {"sampler_us": measure_sampler(), "redraw": {}}
    for W, H in windows:
        for sz in sizes:
            res["redraw"]["%dx%d %s" % (W, H, sz)] = measure_redraw(W, H, sz)
    res["worst_redraw_ms"] = max(v["redraw_ms"] for r in res["redraw"].values() for v in r.values())
    res["worst_data_ms"] = max(v["data_ms"] for r in res["redraw"].values() for v in r.values())
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
        print("worst redraw %.1f ms (limit %.0f), worst data part %.1f ms" % (
            res["worst_redraw_ms"], LIMITS["redraw_ms"], res["worst_data_ms"]))
    ok = res["sampler_us"] <= LIMITS["sampler_us"] and res["worst_redraw_ms"] <= LIMITS["redraw_ms"]
    print("TREND LOAD: %s" % ("OK" if ok else "OVER A LIMIT"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
