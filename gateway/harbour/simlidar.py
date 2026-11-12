"""The SIMULATED LiDAR (docs/harbour_design.md 5, step HX3): LiDAR clusters from simworld.World.

10 Hz scans; per object a cluster {x, y, z (sensor frame: x ahead, y to starboard, z up from the sensor), len_m,
wid_m, hgt_m, n_points}; a quay = a cluster every 5 m. The range depends on the reflectivity (Livox-Mid-360-like:
40 m at 10 %, 70 m at 80 %); the sensor sits mount_h_m (2.5 m) above the water with a vertical field of view
-7..+52 deg, so a LOW object close by is BELOW the field of view (a swimmer closer than ~18 m is not seen - the
minimum-range blind zone of a mast-mounted LiDAR); the number of points falls with the range squared; calm water is
specular (a swimmer is missed in ~40 % of the scans). The raw scan also holds the own hull, a few water-surface
returns and isolated spray points < 3 m: sensors.lidar_filter (in the worker) removes them. Faults: degraded (range
x 0.5, Pd x 0.7), clutter (rain / fog: 20 spurious clusters within 15 m), ghost (a phantom cluster ahead), skew;
"lost" / "crash" / "freeze" are the worker's.
"""
import math
import random

from harbour import simworld

POINTS_K = 4000.0     # points ~ K x visible area / range^2 (~ a 200 k points/s scanner at 10 Hz, 360 x 59 deg)


class LidarSim:
    def __init__(self, world, c, seed=2, tag=False):
        self.w, self.c = world, c
        self.rng = random.Random(seed)
        self.tag = tag
        r10, r80 = float(c["range_10pct_m"]), float(c["range_80pct_m"])
        self.k = math.log(r80 / r10) / math.log(8.0) if r10 > 0 and r80 > r10 else 0.0

    def rmax(self, refl, degraded=False):
        r = float(self.c["range_10pct_m"]) * (max(0.01, refl) / 0.1) ** self.k
        return r * (0.5 if degraded else 1.0)

    def visible_h(self, r, h):
        """The visible height of an object of height h (above the water) at range r, or 0 (outside the FOV)."""
        c = self.c
        mh = float(c["mount_h_m"])
        lo = max(-mh, r * math.tan(math.radians(float(c["vfov_min_deg"]))))
        hi = min(-mh + h, r * math.tan(math.radians(float(c["vfov_max_deg"]))))
        return max(0.0, hi - lo), lo, hi

    def to_sensor(self, pose, x, y):
        ox, oy = simworld.own_xy(self.w, pose)
        h = math.radians(float(pose["heading"]))
        dx, dy = x - ox, y - oy
        ahead = dx * math.sin(h) + dy * math.cos(h)
        stbd = dx * math.cos(h) - dy * math.sin(h)
        return ahead - float(self.c["mount_x_m"]), stbd

    def scan(self, t, pose, faults=None):
        """The RAW scan (before sensors.lidar_filter), or None (a dropped scan)."""
        f = faults or {}
        c, rng = self.c, self.rng
        if rng.random() < float(c["drop_p"]):
            return None
        deg = bool(f.get("degraded"))
        sd0, sdf = float(c["pos_sd_m"]), float(c["pos_sd_frac"])
        mh = float(c["mount_h_m"])
        out = []

        def add(x, y, z, ln, wd, hg, n, sid):
            r = math.hypot(x, y)
            sd = (sd0 + sdf * r) * (3.0 if deg else 1.0)
            o = {"x": round(x + rng.gauss(0, sd), 2), "y": round(y + rng.gauss(0, sd), 2),
                 "z": round(z + rng.gauss(0, sd), 2), "len_m": round(max(0.05, ln * (1 + rng.gauss(0, 0.05))), 2),
                 "wid_m": round(max(0.05, wd * (1 + rng.gauss(0, 0.05))), 2), "hgt_m": round(max(0.02, hg), 2),
                 "n_points": int(n)}
            if self.tag:
                o["sim_id"] = sid
            out.append(o)

        for o in self.w.truth(t):
            if o["kind"] == "pier":
                parts = [(px, py, 5.0, o["w"]) for (px, py) in simworld.pier_points(o, 5.0)]
            else:
                parts = [(o["x"], o["y"], max(o["len"], o["beam"]), min(o["len"], o["beam"]))]
            pdet = 0.6 if o["kind"] == "swimmer" else 0.97
            if deg:
                pdet *= 0.7
            for (px, py, ln, wd) in parts:
                sx, sy = self.to_sensor(pose, px, py)
                r = math.hypot(sx, sy)
                if r < float(c["min_range_m"]) or r > self.rmax(o["refl"], deg):
                    continue
                vh, lo, hi = self.visible_h(r, o["h"])
                if vh <= 0.0 or rng.random() >= pdet:
                    continue
                n = int(POINTS_K * vh * ln / max(1.0, r * r))
                if n < 1:
                    continue
                add(sx, sy, (lo + hi) / 2.0, ln, wd, vh, min(n, 5000), o["id"])
        # own hull (the sensor sees its own boat), a few water-surface returns, isolated spray points < 3 m
        add(-float(c["mount_x_m"]), 0.0, -mh + 0.8, float(c["hull_len_m"]), float(c["hull_wid_m"]), 1.0, 40, "hull")
        for _ in range(rng.randint(0, 2)):
            a, r = rng.uniform(0, 2 * math.pi), rng.uniform(5.0, 20.0)
            add(r * math.cos(a), r * math.sin(a), -mh, 0.5, 0.5, 0.04, 2, "water")
        for _ in range(self._poisson(float(c["spray_n"]))):
            a, r = rng.uniform(0, 2 * math.pi), rng.uniform(0.8, 2.9)
            add(r * math.cos(a), r * math.sin(a), -mh + 0.6, 0.05, 0.05, 0.05, 1, "spray")
        if f.get("clutter"):
            for _ in range(20):
                a, r = rng.uniform(0, 2 * math.pi), rng.uniform(3.0, 15.0)
                add(r * math.cos(a), r * math.sin(a), -mh + 1.0, 0.4, 0.4, 0.5, rng.randint(2, 4), "clutter")
        if f.get("ghost"):
            add(12.0, 3.0, -mh + 1.0, 1.5, 1.0, 1.0, 12, "ghost")
        return out

    def _poisson(self, lam):
        L, k, p = math.exp(-lam), 0, 1.0
        while True:
            p *= self.rng.random()
            if p <= L:
                return k
            k += 1
