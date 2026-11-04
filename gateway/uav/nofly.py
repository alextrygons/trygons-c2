"""Air no-fly zones + the AIR geofence of the UAVs (CLAUDE.md V "Routes", docs/uav_design.md 5.2; step U3).

Two hand-made files on the gateway's SSD (like the charts: NOT authoritative - the owner makes them from the
official aeronautical information; never a substitute for the authorisation, docs 11):

  state/uav_nofly.json     {"zones": [{"name", "source", "kind": "polygon", "poly": [[lat, lon], ...],
                                       "floor_m": 0, "ceiling_m": 9999},
                                      {"name", "source", "kind": "circle", "center": [lat, lon], "radius_m": r,
                                       "floor_m": ..., "ceiling_m": ...}]}
                           floor / ceiling = metres above the launch point (sea level + deck height)
  state/uav_geofence.json  {"poly": [[lat, lon], ...]}   the UAVs' own fence (3..50 corners, the rules of the
                           boat's geofence.py) - SEPARATE from the boat's geofence

A broken file = refused routes ("no-fly file unreadable": fail safe), never ignored. Files are re-read when they
change (mtime), from the caller's helper thread or at start. Pure checks: route_problem(points, zones, fence).
"""
import json
import math
import os

import geofence

STEP_M = 20.0


def _xy(lat0, lon0, lat, lon):
    kx = math.cos(math.radians(lat0)) * 111320.0
    return (lon - lon0) * kx, (lat - lat0) * 111320.0


def _zone(z, i):
    if not isinstance(z, dict):
        raise ValueError("zone %d: not an object" % (i + 1))
    name = str(z.get("name") or "zone %d" % (i + 1))[:60]
    lo = float(z.get("floor_m", 0.0))
    hi = float(z.get("ceiling_m", 1e9))
    if not (math.isfinite(lo) and math.isfinite(hi) and lo <= hi):
        raise ValueError("%s: floor / ceiling" % name)
    kind = z.get("kind", "polygon")
    if kind == "circle":
        c = z.get("center") or []
        la, lo_ = float(c[0]), float(c[1])
        r = float(z.get("radius_m", 0))
        if not (math.isfinite(la) and math.isfinite(lo_) and -90 <= la <= 90 and -180 <= lo_ <= 180 and 0 < r < 1e6):
            raise ValueError("%s: centre / radius" % name)
        return {"name": name, "kind": "circle", "center": (la, lo_), "radius_m": r, "floor_m": lo, "ceiling_m": hi}
    if kind != "polygon":
        raise ValueError("%s: kind must be polygon / circle" % name)
    ok, why, pts = geofence.valid(z.get("poly"))
    if not ok or not pts:
        raise ValueError("%s: %s" % (name, why or "no corners"))
    return {"name": name, "kind": "polygon", "poly": pts, "floor_m": lo, "ceiling_m": hi}


def parse_zones(d):
    if d is None:
        return []
    if not isinstance(d, dict):                        # U3 review H1: a list / number / true is no zone file
        raise ValueError("not a JSON object")
    zs = d.get("zones", [])
    if not isinstance(zs, list):
        raise ValueError("zones must be a list")
    return [_zone(z, i) for i, z in enumerate(zs)]


def in_zone(z, lat, lon, alt):
    if not (z["floor_m"] <= alt <= z["ceiling_m"]):
        return False
    if z["kind"] == "circle":
        x, y = _xy(z["center"][0], z["center"][1], lat, lon)
        return math.hypot(x, y) <= z["radius_m"]
    return geofence.inside(z["poly"], lat, lon)


def route_problem(points, zones, fence):
    """points [(lat, lon, alt)] from the launch point on; -> None or the reason (English). Every leg is sampled
    every STEP_M with its altitude interpolated (a climb / descent through a zone's floor counts)."""
    if len(points) < 1:
        return None
    for k in range(max(1, len(points) - 1)):
        a = points[k]
        b = points[k + 1] if k + 1 < len(points) else a
        x, y = _xy(a[0], a[1], b[0], b[1])
        n = max(1, int(math.hypot(x, y) / STEP_M))
        for s in range(n + 1):
            f = s / float(n)
            la, lo, al = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f
            for z in zones:
                if in_zone(z, la, lo, al):
                    return "the route enters the no-fly zone '%s' (%.5f %.5f, %.0f m)" % (z["name"], la, lo, al)
            if fence and not geofence.inside(fence, la, lo):
                return "the route leaves the air geofence at %.5f %.5f" % (la, lo)
    return None


class Airspace:
    """The two files, re-read when their mtime changes (reload() from a helper thread / at start)."""

    def __init__(self, folder):
        self.folder = folder
        self.zones, self.fence = [], []
        self.error = ""
        self._mt = (None, None)
        self.reload()

    def _paths(self):
        if not self.folder:
            return None, None
        return os.path.join(self.folder, "uav_nofly.json"), os.path.join(self.folder, "uav_geofence.json")

    def reload(self):
        """Never raises (U3 review H1: it runs in Gateway.__init__ and in a helper thread - a crash there would stop
        the gateway / the checks would fail OPEN): any problem = self.error = every route refused."""
        try:
            self._reload()
        except Exception as e:
            self.zones, self.fence = [], []
            self.error = "airspace files: %s" % (str(e)[:80] or type(e).__name__)

    def _reload(self):
        pz, pf = self._paths()
        if pz is None:
            return
        mt = tuple(os.path.getmtime(p) if os.path.exists(p) else None for p in (pz, pf))
        if mt == self._mt:
            return
        self._mt = mt
        err = []
        zones, fence = [], []
        if mt[0] is not None:
            try:
                with open(pz, encoding="utf-8") as f:
                    zones = parse_zones(json.load(f))
            except Exception as e:                      # U3 review H1: never raise (a broken file = routes refused)
                err.append("uav_nofly.json: %s" % (str(e)[:80] or type(e).__name__))
                zones = []
        if mt[1] is not None:
            try:
                with open(pf, encoding="utf-8") as f:
                    d = json.load(f)
                if not isinstance(d, dict):
                    raise ValueError("not a JSON object")
                ok, why, fence = geofence.valid(d.get("poly"))
                if not ok:
                    raise ValueError(why)
            except Exception as e:                      # U3 review H1: never raise
                err.append("uav_geofence.json: %s" % (str(e)[:80] or type(e).__name__))
                fence = []
        self.zones, self.fence, self.error = zones, fence, "; ".join(err)

    def problem(self, points):
        if self.error:
            return "the UAV airspace file is unreadable (%s)" % self.error
        return route_problem(points, self.zones, self.fence)

    def status(self):
        return {"zones": len(self.zones), "fence": len(self.fence), "error": self.error}
