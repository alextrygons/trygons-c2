"""The UAV route checks (CLAUDE.md V "Routes", docs/uav_design.md 5.2; step U3) - NOT shallow water.

check(wps, start, boat, cfg, battery, airspace) -> None or (code, reason): 123 = energy / range / altitude,
124 = a no-fly zone / outside the AIR geofence. Every check, refused with the reason (English: protocol / audit):

  - 1..MAX_WPS points, finite, in range; each altitude in [uav_min_alt_m, uav_max_alt_m] (120 m TO CONFIRM, W66);
  - distance <= uav_rf_range_m (1500 m TO CONFIRM: ELRS + EU 25 mW video) from the START (the boat's trusted
    position at launch, or the drone's own position when it flies) AND from the boat now;
  - energy: the legs at the cruise speed + the climb / descents + the LANDING RESERVE (descend from the highest
    point at the landing rate + uav_land_margin_pct) + a wind margin on the cruise part must fit the battery left
    above the drone's own critical backstop (conservative defaults until the drone's measured consumption is known);
  - the air no-fly zones + the air geofence (nofly.Airspace) along every leg, with the altitude.
Pure (no I/O): tests UA14.
"""
import math

from . import mission

DEFAULTS = {
    "uav_min_alt_m": 10.0,          # never below (manual flight too, docs 6.3)
    "uav_max_alt_m": 120.0,         # W66
    "uav_wp_alt_m": 50.0,           # the default waypoint altitude
    "uav_rf_range_m": 1500.0,       # W59
    "uav_max_speed_ms": 10.0,       # the mission speed cap (DO_CHANGE_SPEED)
    "uav_climb_ms": 2.5,
    "uav_land_ms": 1.0,             # the landing descent rate (ArduCopter LAND_SPEED ~0.5-1 m/s)
    "uav_cruise_a": 20.0,           # conservative 7" quad currents until measured (registry, later)
    "uav_hover_a": 18.0,
    "uav_wind_margin_pct": 30.0,
    "uav_land_margin_pct": 30.0,    # W59
    "uav_backstop_pct": 15.0,       # the battery below the drone's own BATT_CRT backstop is never planned
    "uav_capacity_mah": 2500.0,     # when the drone's BATT_CAPACITY is not known
}
ERR_ROUTE, ERR_AIRSPACE = 123, 124


def dist_m(a, b):
    kx = math.cos(math.radians((a[0] + b[0]) / 2.0)) * 111320.0
    return math.hypot((b[1] - a[1]) * kx, (b[0] - a[0]) * 111320.0)


def clean(wps, default_alt):
    """[{"lat", "lon", "alt"?}] -> the same with floats + the default altitude; ValueError when broken."""
    out = []
    if not isinstance(wps, list) or not wps:
        raise ValueError("no waypoints")
    if len(wps) > mission.MAX_WPS:
        raise ValueError("at most %d waypoints" % mission.MAX_WPS)
    for i, w in enumerate(wps):
        try:
            la, lo = float(w["lat"]), float(w["lon"])
            al = float(w["alt"]) if w.get("alt") is not None else float(default_alt)
        except (KeyError, TypeError, ValueError, AttributeError):
            raise ValueError("waypoint %d: lat / lon / alt" % (i + 1))
        if not (math.isfinite(la) and math.isfinite(lo) and math.isfinite(al) and -90 <= la <= 90 and
                -180 <= lo <= 180):
            raise ValueError("waypoint %d out of range" % (i + 1))
        out.append({"lat": la, "lon": lo, "alt": al})
    return out


def energy_mah(start, wps, cfg, take_alt=None):
    """(route mAh incl. the wind margin, landing reserve mAh) from start (lat, lon, alt) through the waypoints."""
    v = max(1.0, float(cfg["uav_max_speed_ms"]))
    pts = [start] + [(w["lat"], w["lon"], w["alt"]) for w in wps]
    horiz = sum(dist_m(pts[k], pts[k + 1]) for k in range(len(pts) - 1))
    up = down = 0.0
    alts = [start[2]] + ([take_alt] if take_alt is not None else []) + [p[2] for p in pts[1:]]
    for k in range(len(alts) - 1):
        d = alts[k + 1] - alts[k]
        up, down = up + max(0.0, d), down + max(0.0, -d)
    t_cruise = horiz / v
    t_vert = up / float(cfg["uav_climb_ms"]) + down / float(cfg["uav_land_ms"])
    route = (float(cfg["uav_cruise_a"]) * t_cruise * (1.0 + float(cfg["uav_wind_margin_pct"]) / 100.0) +
             float(cfg["uav_hover_a"]) * t_vert) / 3.6
    top = max(alts)
    reserve = float(cfg["uav_hover_a"]) * (top / float(cfg["uav_land_ms"])) / 3.6 * (
        1.0 + float(cfg["uav_land_margin_pct"]) / 100.0)
    return route, reserve


def check(wps, start, boat, cfg, battery, airspace=None, take_alt=None):
    """wps: clean() points; start (lat, lon, alt) = where the route begins (the boat at launch / the flying drone);
    boat (lat, lon) now or None; battery {"pct", "capacity_mah"}. -> None or (code, reason)."""
    c = dict(DEFAULTS, **(cfg or {}))
    if start is None or boat is None:
        return ERR_ROUTE, "no trusted boat position (GPS) - a UAV route starts at the boat"
    for i, w in enumerate(wps):
        if not (float(c["uav_min_alt_m"]) <= w["alt"] <= float(c["uav_max_alt_m"])):
            return ERR_ROUTE, "waypoint %d altitude %.0f m (allowed %.0f..%.0f m)" % (
                i + 1, w["alt"], c["uav_min_alt_m"], c["uav_max_alt_m"])
        for what, ref in (("the launch point", start), ("the boat", boat)):
            d = dist_m(ref, (w["lat"], w["lon"]))
            if d > float(c["uav_rf_range_m"]):
                return ERR_ROUTE, "waypoint %d is %.0f m from %s (radio range %.0f m)" % (
                    i + 1, d, what, c["uav_rf_range_m"])
    if take_alt is not None and take_alt > float(c["uav_max_alt_m"]):
        return ERR_ROUTE, "take-off altitude %.0f m above the limit" % take_alt
    pct = battery.get("pct") if battery else None
    if pct is None or not math.isfinite(float(pct)):
        return ERR_ROUTE, "battery state unknown"
    cap = float(battery.get("capacity_mah") or c["uav_capacity_mah"])
    usable = cap * (float(pct) - float(c["uav_backstop_pct"])) / 100.0
    route, reserve = energy_mah(start, wps, c, take_alt)
    if route + reserve > usable:
        return ERR_ROUTE, "not enough battery: route %.0f mAh + landing reserve %.0f mAh > %.0f mAh usable (%d %%)" % (
            route, reserve, max(0.0, usable), int(pct))
    if airspace is not None:
        pts = [start] + ([(start[0], start[1], take_alt)] if take_alt is not None else []) + \
              [(w["lat"], w["lon"], w["alt"]) for w in wps]
        why = airspace.problem(pts)
        if why:
            return ERR_AIRSPACE, why
    return None
