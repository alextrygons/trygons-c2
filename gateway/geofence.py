"""Geofence (CLAUDE.md D: the copilot edits it on screen 2's map): a polygon (3..50 corners) that
the boat's AUTO routes must stay inside.

Kept by the GATEWAY (state/geofence.json on the SSD, survives a restart) and checked like the
shallow water (section C): an AUTO program upload or an AUTO start / resume whose waypoint or
leg leaves the fence is refused (code 120, with the leg and the position). A station's change
(set / change / clear) goes through stations.ask_local: a copilot's waits for the captain's
ACCEPT on screen 1, the captain's own goes straight through, single-operator = the own
confirmation. Every change is in the audit log.
The fence never steers and never changes the throttle: it only refuses routes.

    valid(poly)                     -> (ok, why, cleaned [(lat, lon)])
    inside(poly, lat, lon)          -> bool
    check_route(poly, wps, start)   -> None or the reason (English: audit / protocol)
    Geofence(path).set(poly, user)  -> status() {"poly", "by", "t"}
"""
import json
import math
import os
import time

MAX_CORNERS = 50
STEP_M = 25.0               # legs are checked every 25 m


def _seg_cross(a, b, c, d):
    """Do the segments a-b and c-d cross (not just touch at a shared corner)?"""
    def orient(p, q, r):
        v = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        return 0 if abs(v) < 1e-18 else (1 if v > 0 else -1)
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return o1 != o2 and o3 != o4 and 0 not in (o1, o2, o3, o4)


def valid(poly):
    """A fence: 3..50 finite corners in range, not crossing itself. [] = no fence (allowed: clear)."""
    try:
        pts = [(float(p[0]), float(p[1])) for p in poly or []]
    except (TypeError, ValueError, IndexError):
        return False, "corners must be [lat, lon] numbers", []
    if not pts:
        return True, "", []
    if len(pts) < 3:
        return False, "a geofence needs at least 3 corners", []
    if len(pts) > MAX_CORNERS:
        return False, "a geofence has at most %d corners" % MAX_CORNERS, []
    for la, lo in pts:
        if not (math.isfinite(la) and math.isfinite(lo) and -90.0 <= la <= 90.0 and -180.0 <= lo <= 180.0):
            return False, "a corner is out of range", []
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue                                # neighbours share a corner
            if _seg_cross(pts[i], pts[(i + 1) % n], pts[j], pts[(j + 1) % n]):
                return False, "the geofence crosses itself (corners %d-%d and %d-%d)" % (
                    i + 1, (i + 1) % n + 1, j + 1, (j + 1) % n + 1), []
    return True, "", pts


def inside(poly, lat, lon):
    """Point in polygon (ray casting on lat / lon: fine for a boat's area)."""
    c = False
    n = len(poly)
    for i in range(n):
        (y1, x1), (y2, x2) = poly[i], poly[(i + 1) % n]
        if (y1 > lat) != (y2 > lat):
            x = x1 + (lat - y1) * (x2 - x1) / (y2 - y1)
            if lon < x:
                c = not c
    return c


def check_route(poly, wps, start=None, step_m=STEP_M):
    """wps: [(lat, lon)]; start: the boat (first leg). -> None or the reason (English)."""
    if not poly or len(poly) < 3:
        return None
    pts = ([start] if start else []) + list(wps)
    for i, (la, lo) in enumerate(wps):
        if not inside(poly, la, lo):
            return "WP%d is outside the geofence (%.5f %.5f)" % (i + 1, la, lo)
    for k in range(len(pts) - 1):
        (a0, o0), (a1, o1) = pts[k], pts[k + 1]
        kx = math.cos(math.radians((a0 + a1) / 2.0)) * 111320.0
        L = math.hypot((o1 - o0) * kx, (a1 - a0) * 111320.0)
        n = max(1, int(L / step_m))
        for s in range(n + 1):
            f = s / float(n)
            la, lo = a0 + (a1 - a0) * f, o0 + (o1 - o0) * f
            if not inside(poly, la, lo):
                if start:
                    leg = "boat -> WP1" if k == 0 else "WP%d -> WP%d" % (k, k + 1)
                else:
                    leg = "WP%d -> WP%d" % (k + 1, k + 2)
                return "leg %s leaves the geofence at %.5f %.5f (%.0f m along the leg)" % (leg, la, lo, f * L)
    return None


class Geofence:
    def __init__(self, path=None):
        self.path = path
        self.poly, self.by, self.t = [], None, None
        if path and os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as f:
                    d = json.load(f)
                ok, _why, pts = valid(d.get("poly"))
                if ok:
                    self.poly, self.by, self.t = pts, d.get("by"), d.get("t")
            except (OSError, ValueError, AttributeError):
                pass

    def set(self, poly, user):
        ok, why, pts = valid(poly)
        if not ok:
            return False, why
        self.poly, self.by, self.t = pts, user, time.time()
        if self.path:
            try:
                tmp = self.path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump({"poly": pts, "by": user, "t": self.t}, f)
                os.replace(tmp, self.path)
            except OSError:
                pass
        return True, ("geofence set: %d corners" % len(pts)) if pts else "geofence cleared"

    def check(self, wps, start=None):
        return check_route(self.poly, wps, start)

    def status(self):
        return {"poly": [[round(a, 7), round(b, 7)] for a, b in self.poly], "by": self.by, "t": self.t}
