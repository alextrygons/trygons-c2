"""Offline depth charts (section B) - used by the gateway (shallow-water
checks, avoidance routes) and the ground station (map). Pure Python, no
internet: the files are made once by tools/chartprep.py.

    charts/<area>/area.json       grid size, bounds, sources, attribution
    charts/<area>/depth.bin       int16 LE, row 0 = south edge, west -> east;
                                  depth below LAT in decimetres (+ = water),
                                  land = negative (height), -32768 = no data
    charts/<area>/contours.json   {"2": [[[lat, lon], ...], ...], "5": ...}
    charts/<area>/seamarks.json   [{"lat", "lon", "type", "name", ...}]

NOT FOR NAVIGATION - free data, may be inaccurate near the coast.
Safety side: a point's depth is the SHALLOWEST of the grid cells around it;
no data counts as shallow.
"""
import array
import json
import math
import os

NODATA = -32768
NOTE = "Not for navigation - free data, may be inaccurate near the coast"
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "charts")


class Area:
    def __init__(self, folder):
        self.folder = folder
        with open(os.path.join(folder, "area.json"), encoding="utf-8") as f:
            self.meta = json.load(f)
        m = self.meta
        self.name = m["name"]
        self.lat0, self.lon0 = m["lat0"], m["lon0"]           # centre of cell (0, 0)
        self.dlat, self.dlon = m["dlat"], m["dlon"]
        self.nx, self.ny = m["nx"], m["ny"]
        self.grid = array.array("h")
        with open(os.path.join(folder, "depth.bin"), "rb") as f:
            self.grid.frombytes(f.read())
        if len(self.grid) != self.nx * self.ny:
            raise ValueError("%s: depth.bin has %d cells, expected %d" % (folder, len(self.grid), self.nx * self.ny))
        self._contours = None
        self._seamarks = None

    @property
    def bbox(self):
        return (self.lat0 - self.dlat / 2, self.lat0 + (self.ny - 0.5) * self.dlat,
                self.lon0 - self.dlon / 2, self.lon0 + (self.nx - 0.5) * self.dlon)

    def contains(self, lat, lon):
        a, b, c, d = self.bbox
        return a <= lat <= b and c <= lon <= d

    def cell(self, i, j):
        """depth (m, + water) of cell row i, column j; None = no data."""
        if not (0 <= i < self.ny and 0 <= j < self.nx):
            return None
        v = self.grid[i * self.nx + j]
        return None if v == NODATA else v / 10.0

    def ij(self, lat, lon):
        return (lat - self.lat0) / self.dlat, (lon - self.lon0) / self.dlon

    def depth(self, lat, lon):
        """Shallowest of the 4 cells around the point (None: no data / outside)."""
        fi, fj = self.ij(lat, lon)
        i0, j0 = int(math.floor(fi)), int(math.floor(fj))
        vals = [self.cell(i, j) for i in (i0, i0 + 1) for j in (j0, j0 + 1)]
        if any(v is None for v in vals):
            return None
        return min(vals)

    def min_depth_near(self, lat, lon, radius_m):
        """Shallowest depth within radius_m (None if any cell has no data)."""
        fi, fj = self.ij(lat, lon)
        ri = int(math.ceil(radius_m / (self.dlat * 111320.0))) + 1
        rj = int(math.ceil(radius_m / (self.dlon * 111320.0 * math.cos(math.radians(lat))))) + 1
        best = 1e9
        for i in range(int(fi) - ri, int(fi) + ri + 2):
            for j in range(int(fj) - rj, int(fj) + rj + 2):
                dy = (i - fi) * self.dlat * 111320.0
                dx = (j - fj) * self.dlon * 111320.0 * math.cos(math.radians(lat))
                if dx * dx + dy * dy > (radius_m + 120.0) ** 2:
                    continue
                v = self.cell(i, j)
                if v is None:
                    return None
                best = min(best, v)
        return best

    def nearest_shallow_m(self, lat, lon, limit_m, max_m):
        """Exact distance (m, cell centres) to the nearest cell shallower than limit_m or
        without data, within max_m; None if there is none that close."""
        fi, fj = self.ij(lat, lon)
        my = self.dlat * 111320.0
        mx = self.dlon * 111320.0 * math.cos(math.radians(lat))
        ri, rj = int(math.ceil(max_m / my)) + 1, int(math.ceil(max_m / mx)) + 1
        best = None
        for i in range(int(math.floor(fi)) - ri, int(math.floor(fi)) + ri + 2):
            for j in range(int(math.floor(fj)) - rj, int(math.floor(fj)) + rj + 2):
                d = math.hypot((i - fi) * my, (j - fj) * mx)
                if d > max_m or (best is not None and d >= best):
                    continue
                v = self.cell(i, j)
                if v is None or v < limit_m:
                    best = d
        return best

    def contours(self):
        if self._contours is None:
            p = os.path.join(self.folder, "contours.json")
            self._contours = json.load(open(p, encoding="utf-8")) if os.path.exists(p) else {}
        return self._contours

    def seamarks(self):
        if self._seamarks is None:
            p = os.path.join(self.folder, "seamarks.json")
            self._seamarks = json.load(open(p, encoding="utf-8")) if os.path.exists(p) else []
        return self._seamarks


class Charts:
    """All prepared areas under a folder (several areas can be added later)."""

    def __init__(self, folder=None):
        self.folder = folder or DEFAULT_DIR
        self.areas = []
        if os.path.isdir(self.folder):
            for n in sorted(os.listdir(self.folder)):
                if os.path.exists(os.path.join(self.folder, n, "area.json")):
                    try:
                        self.areas.append(Area(os.path.join(self.folder, n)))
                    except (OSError, ValueError, KeyError):
                        pass

    def __bool__(self):
        return bool(self.areas)

    def area_at(self, lat, lon):
        for a in self.areas:
            if a.contains(lat, lon):
                return a
        return None

    def depth(self, lat, lon):
        a = self.area_at(lat, lon)
        return a.depth(lat, lon) if a else None

    def min_depth_near(self, lat, lon, radius_m):
        a = self.area_at(lat, lon)
        return a.min_depth_near(lat, lon, radius_m) if a else None

    def nearest_shallow_m(self, lat, lon, limit_m, max_m):
        """Distance to the nearest shallow (or no-data) water within max_m: 0.0 outside the
        charts, None if nothing that close (Area.nearest_shallow_m)."""
        a = self.area_at(lat, lon)
        return a.nearest_shallow_m(lat, lon, limit_m, max_m) if a else 0.0

    def profile(self, lat1, lon1, lat2, lon2, step_m=50.0):
        """[(distance m, lat, lon, depth or None)] along a straight leg."""
        d = dist_m(lat1, lon1, lat2, lon2)
        n = max(1, int(math.ceil(d / step_m)))
        out = []
        for k in range(n + 1):
            f = k / n
            la, lo = lat1 + (lat2 - lat1) * f, lon1 + (lon2 - lon1) * f
            out.append((d * f, la, lo, self.depth(la, lo)))
        return out

    def check_leg(self, lat1, lon1, lat2, lon2, limit_m, clearance_m=0.0, step_m=50.0):
        """First problem on a leg: None if safe, else (distance, lat, lon, depth, why)."""
        for dist, la, lo, dep in self.profile(lat1, lon1, lat2, lon2, step_m):
            if dep is None:
                return dist, la, lo, None, "no depth data (outside the charts or at the coast)"
            if dep < limit_m:
                return dist, la, lo, dep, "%.1f m water (limit %.1f m)" % (dep, limit_m)
            if clearance_m > 0:
                near = self.min_depth_near(la, lo, clearance_m)
                if near is None or near < limit_m:
                    return dist, la, lo, near, "shallow water closer than %.0f m (%s)" % (
                        clearance_m, "no data" if near is None else "%.1f m" % near)
        return None

    def attribution(self):
        return sorted({a.meta.get("attribution", "") for a in self.areas if a.meta.get("attribution")})


def dist_m(lat1, lon1, lat2, lon2):
    x = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2))
    y = (lat2 - lat1) * 111320.0
    return math.hypot(x, y)


# --- colours for the map ------------------------------------------------------------------
BANDS = [(0.0, "#c8b48c"), (2.0, "#5fb4dc"), (3.0, "#78c3e6"), (5.0, "#96d2ee"), (10.0, "#b4e0f4"),
         (20.0, "#cae9f7"), (50.0, "#dceffa"), (100.0, "#eaf5fc"), (1e9, "#f4faff")]


def depth_colour(d):
    """Land / shallow (dark blue) ... deep (white) - like a paper chart."""
    if d is None:
        return "#a0a0a0"
    if d <= 0.0:
        return BANDS[0][1]
    for lim, col in BANDS[1:]:
        if d < lim:
            return col
    return BANDS[-1][1]


# --- writing (chartprep + tests) ----------------------------------------------------------

def write_area(folder, name, lat0, lon0, dlat, dlon, rows, sources, attribution, levels=(2, 3, 5, 10, 20, 50, 100),
               seamarks=None):
    """rows: list of rows (south first) of depths in m (+ water, - land, None no data)."""
    os.makedirs(folder, exist_ok=True)
    ny, nx = len(rows), len(rows[0])
    g = array.array("h", [NODATA if v is None else max(-32767, min(32767, int(round(v * 10)))) for r in rows
                          for v in r])
    with open(os.path.join(folder, "depth.bin"), "wb") as f:
        f.write(g.tobytes())
    meta = {"name": name, "lat0": lat0, "lon0": lon0, "dlat": dlat, "dlon": dlon, "nx": nx, "ny": ny,
            "sources": sources, "attribution": attribution, "levels": list(levels), "note": NOTE,
            "units": "decimetres below LAT, + = water"}
    with open(os.path.join(folder, "area.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=1)
    cont = {str(lv): contour_lines(rows, lat0, lon0, dlat, dlon, lv) for lv in levels}
    with open(os.path.join(folder, "contours.json"), "w", encoding="utf-8") as f:
        json.dump(cont, f, separators=(",", ":"))
    with open(os.path.join(folder, "seamarks.json"), "w", encoding="utf-8") as f:
        json.dump(seamarks or [], f, indent=0)
    return meta


def contour_lines(rows, lat0, lon0, dlat, dlon, level):
    """Marching squares: the depth = level lines as polylines [[lat, lon], ...]."""
    ny, nx = len(rows), len(rows[0])
    segs = []

    def v(i, j):
        d = rows[i][j]
        return -1e9 if d is None else d                     # no data: treated as land

    def interp(i1, j1, i2, j2):
        a, b = v(i1, j1), v(i2, j2)
        t = 0.5 if a == b else max(0.0, min(1.0, (level - a) / (b - a)))
        return (round(lat0 + (i1 + (i2 - i1) * t) * dlat, 6), round(lon0 + (j1 + (j2 - j1) * t) * dlon, 6))

    for i in range(ny - 1):
        for j in range(nx - 1):
            c = [v(i, j) >= level, v(i, j + 1) >= level, v(i + 1, j + 1) >= level, v(i + 1, j) >= level]
            k = c[0] | (c[1] << 1) | (c[2] << 2) | (c[3] << 3)
            if k in (0, 15):
                continue
            e = {0: (i, j, i, j + 1), 1: (i, j + 1, i + 1, j + 1), 2: (i + 1, j, i + 1, j + 1), 3: (i, j, i + 1, j)}
            cross = [n for n, (a, b) in enumerate(((0, 1), (1, 2), (3, 2), (0, 3))) if c[a] != c[b]]
            pts = [interp(*e[n]) for n in cross]
            if len(pts) == 2:
                segs.append((pts[0], pts[1]))
            elif len(pts) == 4:
                segs.append((pts[0], pts[1]))
                segs.append((pts[2], pts[3]))
    # join the segments into polylines
    ends = {}
    for s in segs:
        ends.setdefault(s[0], []).append(s)
        ends.setdefault(s[1], []).append(s)
    used = set()
    lines = []
    for s in segs:
        if id(s) in used:
            continue
        used.add(id(s))
        line = [s[0], s[1]]
        for direction in (1, 0):
            while True:
                tip = line[-1] if direction else line[0]
                nxt = next((t for t in ends.get(tip, []) if id(t) not in used), None)
                if nxt is None:
                    break
                used.add(id(nxt))
                other = nxt[1] if nxt[0] == tip else nxt[0]
                if direction:
                    line.append(other)
                else:
                    line.insert(0, other)
        lines.append([list(p) for p in line])
    return lines
