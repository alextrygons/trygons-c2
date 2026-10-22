"""Shallow-water checks on the gateway (section C), with the offline charts.

limit = max(shallow_m, draft_m + margin_m)   (default 3.0 m; draft: measure it,
                                              outboard leg DOWN)
  * AUTO program: every leg (and the leg from the boat to the active waypoint
    at AUTO start / resume) must stay over water deeper than the limit and
    not pass closer than clearance_m to shallower water. Refused with the
    leg, the depth and the position. No chart data = shallow (refused).
  * Driving (any mode): the predicted position in the next look_s (at least
    look_min_m ahead) over shallow water -> warning on both screens; in AUTO
    also slow down (navreq speed cap) and the SHALLOW WATER alarm.
  * Every avoidance route (section D) uses the same check (check_leg).
"""
import math

from ais import KN, from_xy

DEFAULTS = {"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0, "look_s": 30.0,
            "look_min_m": 200.0, "slow_kn": 4.0, "require_charts": True}


class Shallow:
    def __init__(self, cfg, charts):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.charts = charts

    def limit(self):
        c = self.cfg
        return max(c["shallow_m"], c["draft_m"] + c["margin_m"])

    def check_leg(self, lat1, lon1, lat2, lon2, clearance=True):
        """None if safe, else (dist, lat, lon, depth, why)."""
        if not self.charts:
            return (0.0, lat1, lon1, None, "no depth charts on the gateway") if self.cfg["require_charts"] else None
        return self.charts.check_leg(lat1, lon1, lat2, lon2, self.limit(),
                                     self.cfg["clearance_m"] if clearance else 0.0, step_m=40.0)

    def check_program(self, wps, start=None):
        """wps: [(lat, lon)]; start = (lat, lon) of the boat for the first leg.
        -> None or text with the leg, the depth and the position."""
        pts = ([start] if start else []) + list(wps)
        for k in range(len(pts) - 1):
            bad = self.check_leg(pts[k][0], pts[k][1], pts[k + 1][0], pts[k + 1][1])
            if bad:
                if start:
                    leg = "boat -> WP%d" % 1 if k == 0 else "WP%d -> WP%d" % (k, k + 1)
                else:
                    leg = "WP%d -> WP%d" % (k + 1, k + 2)
                return "leg %s: %s at %.5f %.5f (%.0f m along the leg)" % (leg, bad[4], bad[1], bad[2], bad[0]), bad
        return None

    def watch(self, own):
        """Predicted shallow water ahead -> {"lat", "lon", "depth", "in_s", "limit"} or None."""
        if own.get("lat") is None or not self.charts:
            return None
        c = self.cfg
        v = max(0.5, (own.get("sog") or 0.0)) * KN
        look_m = max(c["look_min_m"], v * c["look_s"])
        n = max(2, int(look_m / 25.0))
        lim = self.limit()
        cog = math.radians(own.get("cog") or 0.0)
        for k in range(n + 1):
            s = look_m * k / n
            la, lo = from_xy(s * math.sin(cog), s * math.cos(cog), own["lat"], own["lon"])
            dep = self.charts.depth(la, lo)
            if dep is None or dep < lim:
                return {"lat": la, "lon": lo, "depth": dep, "in_s": round(s / v, 1), "limit": lim}
        return None
