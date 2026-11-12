"""The harbour static map (CLAUDE.md Z, docs/harbour_design.md 6.2 + 6a, step HX4).

Two layers in the local flat frame (ENU metres from the harbour origin), 0.5 m cells:
  chart  the chart's shore around the harbour: cells where the offline chart (groundstation/chart.py via gw.charts)
         says shallower than the limit, or NO DATA inside a charted area ("no data = shallow"), + explicit segments
         (piers / breakwaters: OpenSeaMap lines or a harbour file); never decays.
  lidar  long-lived static objects accumulated from LiDAR-confirmed STATIC tracks (moored boats, piers): each
         observation adds weight to the cells of its footprint; weight decays with tau_s (1 h: moored boats move);
         a cell counts once its weight >= occ_w.
No numpy on the gateway: saved as JSON (state/harbour/<harbour>/static.json) - the design's .npz became JSON.
Pure: no I/O except save() / load() with a path; never sends anything.
"""
import json
import math
import os

from harbour import simworld

VERSION = 1


class StaticMap:
    def __init__(self, cell_m=0.5, tau_s=3600.0, occ_w=20.0, origin=None):
        self.cell = float(cell_m)
        self.tau = float(tau_s)
        self.occ_w = float(occ_w)
        self.origin = tuple(origin) if origin else None
        self.chart = set()                  # (i, j) cells of the chart layer
        self.chart_known = False            # a chart covered the harbour (or segments were given)
        self.segments = []                  # [(x1, y1, x2, y2, w, src)]
        self.grid = {}                      # (i, j) -> [weight, t_last]

    def ij(self, x, y):
        return int(math.floor(x / self.cell)), int(math.floor(y / self.cell))

    # --- chart layer --------------------------------------------------------------------------------------------
    def add_segment(self, x1, y1, x2, y2, w, src="chart"):
        self.segments.append((x1, y1, x2, y2, w, src))
        self.chart_known = True
        L = math.hypot(x2 - x1, y2 - y1)
        n = max(1, int(L / (self.cell * 0.5)))
        hw = w / 2.0
        k = int(math.ceil(hw / self.cell))
        nx, ny = ((y2 - y1) / L, -(x2 - x1) / L) if L > 0 else (1.0, 0.0)
        for s in range(n + 1):
            f = s / float(n)
            px, py = x1 + (x2 - x1) * f, y1 + (y2 - y1) * f
            for o in range(-k, k + 1):
                q = o * self.cell
                if abs(q) <= hw + 1e-9:
                    self.chart.add(self.ij(px + nx * q, py + ny * q))

    def add_chart(self, charts, origin, radius_m=300.0, step_m=2.0, limit_m=2.0):
        """Sample the offline chart around the origin: shallower than limit_m, or no data inside a charted area =
        a chart cell. Returns the number of charted samples (0 = the chart does not cover the harbour)."""
        if charts is None or not charts:
            return 0
        self.origin = self.origin or tuple(origin)
        lat0, lon0 = self.origin
        n_cov = 0
        r = float(radius_m)
        k = int(r / step_m)
        for i in range(-k, k + 1):
            for j in range(-k, k + 1):
                x, y = i * step_m, j * step_m
                if x * x + y * y > r * r:
                    continue
                lat, lon = simworld.xy_to_ll(lat0, lon0, x, y)
                try:
                    if charts.area_at(lat, lon) is None:
                        continue
                    d = charts.depth(lat, lon)
                except Exception:
                    continue
                n_cov += 1
                if d is None or d < limit_m:
                    for a in range(int(step_m / self.cell)):
                        for b in range(int(step_m / self.cell)):
                            self.chart.add(self.ij(x + a * self.cell, y + b * self.cell))
        if n_cov:
            self.chart_known = True
        return n_cov

    # --- LiDAR accumulation -------------------------------------------------------------------------------------
    def _w(self, cell, t):
        g = self.grid.get(cell)
        if g is None:
            return 0.0
        return g[0] * math.exp(-max(0.0, t - g[1]) / self.tau)

    def observe(self, x, y, len_m, beam_m, t):
        """One observation of a static object's footprint (a circle of the larger half size, capped 6 m)."""
        r = min(6.0, max(0.25, 0.5 * max(len_m, beam_m)))
        k = int(math.ceil(r / self.cell))
        ci, cj = self.ij(x, y)
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                if (a * self.cell) ** 2 + (b * self.cell) ** 2 > r * r:
                    continue
                cell = (ci + a, cj + b)
                self.grid[cell] = [self._w(cell, t) + 1.0, t]
        self.t_last = t

    def prune(self, t, min_w=0.5):
        for cell in [c for c in self.grid if self._w(c, t) < min_w]:
            del self.grid[cell]

    # --- queries --------------------------------------------------------------------------------------------------
    def in_chart(self, x, y):
        """(x, y) itself is inside a chart cell (a pier / the shore): nothing moving can be there."""
        return self.ij(x, y) in self.chart

    def occupied(self, x, y, r=0.0, t=None):
        """A chart cell or an accumulated LiDAR cell within r of (x, y)."""
        k = int(math.ceil(max(0.0, r) / self.cell))
        ci, cj = self.ij(x, y)
        tt = t if t is not None else getattr(self, "t_last", 0.0)
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                cell = (ci + a, cj + b)
                if cell in self.chart:
                    return True
                if cell in self.grid and self._w(cell, tt) >= self.occ_w:
                    return True
        return False

    def has_data(self):
        return self.chart_known or any(g[0] >= self.occ_w for g in self.grid.values())

    def n_cells(self, t=None):
        tt = t if t is not None else getattr(self, "t_last", 0.0)
        return {"chart": len(self.chart), "lidar": sum(1 for c in self.grid if self._w(c, tt) >= self.occ_w)}

    # --- file ------------------------------------------------------------------------------------------------------
    def to_json(self):
        return {"version": VERSION, "cell_m": self.cell, "tau_s": self.tau, "occ_w": self.occ_w,
                "origin": list(self.origin) if self.origin else None, "chart_known": self.chart_known,
                "chart": sorted(list(c) for c in self.chart), "segments": [list(s) for s in self.segments],
                "grid": sorted([c[0], c[1], round(g[0], 3), round(g[1], 3)] for c, g in self.grid.items())}

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.to_json(), f)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        if d.get("version") != VERSION:
            raise ValueError("static map version %r" % d.get("version"))
        m = cls(d["cell_m"], d["tau_s"], d["occ_w"], d.get("origin"))
        m.chart = {tuple(c) for c in d.get("chart", [])}
        m.chart_known = bool(d.get("chart_known"))
        m.segments = [tuple(s) for s in d.get("segments", [])]
        m.grid = {(g[0], g[1]): [g[2], g[3]] for g in d.get("grid", [])}
        return m
