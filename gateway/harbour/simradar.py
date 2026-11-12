"""The SIMULATED radar (docs/harbour_design.md 5, step HX3): radar plots from simworld.World.

One sweep per 1 / rate_hz (0.8 Hz = 48 rpm); per object in range a plot {range_m, brg_deg (relative to the bow),
speed_mps, course_deg, size_m, size_cls, strength} with noise (range sd 3 m + 0.5 %, bearing sd 1 deg, speed sd
0.3 m/s); a quay = a return every 15 m. Detection probability = the object's rcs, less with range (small objects
fade first), nothing inside the MINIMUM-RANGE blind zone (min_range_m 25 m - a pulse radar is blind close by; the
HALO's true value is NV, bench it) or beyond max_range_m; a dropped sweep with drop_p; sea clutter = clutter_n
random weak plots per sweep within clutter_r_m (no speed). Faults (the worker's "fault" op): degraded (Pd x 0.5,
noise x 3), clutter (x 25 clutter), ghost (a multipath ghost at twice the range of the strongest return, with a
speed), skew (timestamps wrong). "lost" / "crash" / "freeze" are the worker's (worker.py).
"""
import math
import random

from harbour import sensors, simworld


class RadarSim:
    def __init__(self, world, c, seed=1, tag=False):
        self.w, self.c = world, c
        self.rng = random.Random(seed)
        self.tag = tag                  # tests: each plot names the world object it came from ("sim_id")

    def pd(self, rcs, r):
        rmax = float(self.c["max_range_m"])
        fade = max(0.0, 1.0 - (r / (rmax * max(0.05, rcs))) ** 2)     # a small rcs fades at a shorter range
        return min(0.98, rcs + 0.3) * fade

    def sweep(self, t, pose, faults=None):
        f = faults or {}
        c, rng = self.c, self.rng
        if rng.random() < float(c["drop_p"]):
            return None                                             # a missed sweep
        deg = 3.0 if f.get("degraded") else 1.0
        ox, oy = simworld.own_xy(self.w, pose)
        hdg = float(pose["heading"])
        rmin, rmax = float(c["min_range_m"]), float(c["max_range_m"])
        plots = []
        for o in self.w.truth(t):
            pts = simworld.pier_points(o, 15.0) if o["kind"] == "pier" else [(o["x"], o["y"])]
            for (x, y) in pts:
                dx, dy = x - ox, y - oy
                r = math.hypot(dx, dy)
                if r < rmin or r > rmax:
                    continue
                p = self.pd(o["rcs"], r) * (0.5 if f.get("degraded") else 1.0)
                if rng.random() >= p:
                    continue
                rr = max(0.0, r + rng.gauss(0.0, deg * (float(c["range_sd_m"]) + float(c["range_sd_frac"]) * r)))
                b = (math.degrees(math.atan2(dx, dy)) + rng.gauss(0.0, deg * float(c["brg_sd_deg"]))) % 360.0
                size = 15.0 if o["kind"] == "pier" else max(o["len"], o["beam"])
                size = max(0.3, size * (1.0 + rng.gauss(0.0, 0.15)))
                plot = {"range_m": round(rr, 1), "brg_deg": round((b - hdg) % 360.0, 2), "size_m": round(size, 1),
                        "size_cls": sensors.size_cls(size),
                        "strength": round(min(1.0, max(0.05, o["rcs"] * (1.0 - r / (rmax + 1)))), 2),
                        "speed_mps": None, "course_deg": None}
                if o["speed"] > 0.0 or o["kind"] in ("moored", "buoy"):   # the radar's own ARPA: moving + still
                    v = max(0.0, o["speed"] + rng.gauss(0.0, deg * float(c["speed_sd_mps"])))
                    plot["speed_mps"] = round(min(59.0, v), 2)
                    plot["course_deg"] = round((o["hdg"] + rng.gauss(0.0, 5.0 * deg)) % 360.0, 1)
                if self.tag:
                    plot["sim_id"] = o["id"]
                plots.append(plot)
        n = float(c["clutter_n"]) * (25.0 if f.get("clutter") else 1.0)
        for _ in range(self._poisson(n)):
            r = rng.uniform(rmin, max(rmin + 1.0, float(c["clutter_r_m"])))
            plot = {"range_m": round(r, 1), "brg_deg": round(rng.uniform(0.0, 360.0), 2), "size_m": 1.0,
                    "size_cls": "small", "strength": round(rng.uniform(0.05, 0.25), 2), "speed_mps": None,
                    "course_deg": None}
            if self.tag:
                plot["sim_id"] = "clutter"
            plots.append(plot)
        if f.get("ghost") and plots:
            s = max(plots, key=lambda p: p["strength"])
            if 2 * s["range_m"] <= rmax:
                g = dict(s, range_m=round(2 * s["range_m"], 1), strength=round(s["strength"] * 0.5, 2),
                         speed_mps=1.5, course_deg=round(rng.uniform(0, 360), 1))
                if self.tag:
                    g["sim_id"] = "ghost"
                plots.append(g)
        return plots

    def _poisson(self, lam):
        if lam <= 0:
            return 0
        L, k, p = math.exp(-lam), 0, 1.0
        while True:
            p *= self.rng.random()
            if p <= L:
                return k
            k += 1
