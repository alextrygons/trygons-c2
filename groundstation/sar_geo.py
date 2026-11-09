"""Search and rescue geometry (CLAUDE.md X, docs/sar_design.md 3; step PF3). PURE: no Tk, no network, no boat.

Flat-earth maths around a point (good to a few km: a rescue happens within metres to a few kilometres).
Bearings are TRUE, clockwise from north, degrees.

  distance_bearing(a, b)                    -> (metres, bearing a -> b)
  dest(p, brg, dist_m)                      -> (lat, lon) the point dist_m from p along brg
  standoff_point(boat, marker, current, D)  -> (lat, lon, how) the approach point A (down-current of the person when
                                               the current is known, else on the line to the boat)
  standoff_route(boat, marker, current, cfg)-> {"wps": [...], "final": PROG_END_STOP, ...} or {"why": text key}

The final approach is ALWAYS manual (the propeller): the route ends D metres before the person with the AURIX's own
arrival stop (idle + neutral + STANDBY), never AT the person.
"""
import math

M_PER_DEG = 111320.0
DEFAULTS = {"standoff_m": 30.0, "approach_kn": 5.0, "arrival_radius_m": 8.0, "conf_min": 0.5}
PROG_END_STOP = 0           # program.py: final action 0 = STOP (idle + neutral + STANDBY at the last waypoint)


def _kx(lat):
    return M_PER_DEG * math.cos(math.radians(lat))


def distance_bearing(a, b):
    """a, b = (lat, lon) -> (metres, bearing from a to b)."""
    dn = (b[0] - a[0]) * M_PER_DEG
    de = (b[1] - a[1]) * _kx((a[0] + b[0]) / 2.0)
    return math.hypot(dn, de), math.degrees(math.atan2(de, dn)) % 360.0


def dest(p, brg, dist_m):
    r = math.radians(brg)
    lat = p[0] + dist_m * math.cos(r) / M_PER_DEG
    lon = p[1] + dist_m * math.sin(r) / _kx((p[0] + lat) / 2.0)
    return lat, lon


def standoff_point(boat, marker, current, d_m):
    """The approach point A = M + D x u. u points from M toward the DOWN-current side (bearing = the current's set_deg)
    when the current is known and trusted (conf >= conf_min), else from M back toward the boat. -> (lat, lon, how)."""
    m = (marker[0], marker[1])
    conf = (current or {}).get("conf")
    set_deg = (current or {}).get("set_deg")
    if (set_deg is not None and conf is not None and math.isfinite(set_deg) and math.isfinite(conf)
            and conf >= DEFAULTS["conf_min"] and (current.get("kn") or 0.0) >= 0.1):
        la, lo = dest(m, set_deg % 360.0, d_m)
        return la, lo, "current"
    dist, brg = distance_bearing(m, boat)
    if dist < 1e-3:
        return None
    la, lo = dest(m, brg, d_m)
    return la, lo, "straight"


def standoff_route(boat, marker, current=None, cfg=None):
    """boat, marker = (lat, lon); current = {"set_deg", "kn", "conf"} or None.
    -> {"wps": [(lat, lon), (lat, lon)], "end": (lat, lon), "how", "kn", "final": PROG_END_STOP, "standoff_m"}
    or {"why": "inside"} when the boat is already within the stand-off distance + the arrival radius (no route: the
    approach is by hand)."""
    c = dict(DEFAULTS, **(cfg or {}))
    d_m = float(c["standoff_m"])
    dist, _brg = distance_bearing(boat, marker)
    if dist <= d_m + float(c["arrival_radius_m"]):
        return {"why": "inside", "dist_m": dist, "standoff_m": d_m}
    a = standoff_point(boat, marker, current, d_m)
    if a is None:
        return {"why": "inside", "dist_m": dist, "standoff_m": d_m}
    return {"wps": [(boat[0], boat[1]), (a[0], a[1])], "end": (a[0], a[1]), "how": a[2],
            "kn": float(c["approach_kn"]), "final": PROG_END_STOP, "standoff_m": d_m}


def to_program(route, radius_m=None):
    """A standoff_route() result -> a program dict for program.upload_frames (same shape as the map's route): the
    leg speed <= approach_kn, the LAST waypoint action WP_ACT_SLOW, the final action PROG_END_STOP (never HOLD)."""
    r = float(radius_m if radius_m is not None else DEFAULTS["arrival_radius_m"])
    kn = float(route["kn"])
    wps = []
    for i, (la, lo) in enumerate(route["wps"]):
        wps.append({"lat": la, "lon": lo, "kn": kn, "radius": r, "action": 1 if i == len(route["wps"]) - 1 else 0})
    return {"final": PROG_END_STOP, "wps": wps}
