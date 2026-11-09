"""Search patterns, IAMSAR Vol. II geometry (CLAUDE.md X, docs/sar_design.md 4; step PF2). PURE: no Tk, no network.

Flat-earth maths around the datum through sar_geo.dest / distance_bearing. Bearings are TRUE, clockwise. Turns are to
STARBOARD (IAMSAR standard) unless turn == "port" (the mirror image).

  expanding_square(datum, S, C, legs, turn)   SS  leg k = ceil(k/2) x S, course C + 90 (k-1)
  sector(datum, R, C, turn)                   VS  9 legs of R, 120 degree turns; corners at C, C+60 ... C+300
  parallel_track(datum, S, C, W, Lg, turn)    PS  W across x Lg along C, corner start, S/2 in from the edges
  creeping_line(datum, S, C, W, Lg, turn)     CS  legs ACROSS the long side (legs along C-90, advance along C)
  plan(kind, datum, p)                        -> {"wps", "legs", "total_m", "time_s", "area_m2", "n"} or {"why"}
  boat_check(plan) / cut_at_limit(plan)       the boat route: > 50 waypoints is REFUSED unless the operator cuts at 50
  expanding_total(S, legs)                    the closed formula of the design (checked against the geometry in tests)
"""
import math

import sar_geo

KINDS = ("ss", "vs", "ps", "cs")
MAX_BOAT_WP = 50            # program.MAX_WP / PROG_MAX_WP on the AURIX
M_PER_NM = 1852.0
MAX_LEGS = 200              # a sanity bound


def _sgn(turn):
    return -1.0 if turn == "port" else 1.0


def expanding_square(datum, S, C, legs, turn="starboard"):
    """-> [(lat, lon)] the datum + one end point per leg. Leg k (1..n) = ceil(k/2) x S along C + sgn 90 (k-1)."""
    sg = _sgn(turn)
    pts = [(datum[0], datum[1])]
    for k in range(1, int(legs) + 1):
        pts.append(sar_geo.dest(pts[-1], (C + sg * 90.0 * (k - 1)) % 360.0, math.ceil(k / 2.0) * S))
    return pts


def expanding_total(S, legs):
    """S x sum_{k=1..n} ceil(k/2) = S x (floor(n/2)(floor(n/2)+1) + (n odd ? ceil(n/2) : 0))."""
    n = int(legs)
    h = n // 2
    return S * (h * (h + 1) + ((n + 1) // 2 if n % 2 else 0))


def legs_for_half_side(R, S):
    """A half side R -> n = 2 floor(R/S) + 1 legs."""
    return 2 * int(math.floor(R / S)) + 1


def sector(datum, R, C, turn="starboard"):
    """10 points: datum, R@C, R@(C+60), datum, R@(C+240), R@(C+300), datum, R@(C+120), R@(C+180), datum."""
    sg = _sgn(turn)
    d = (datum[0], datum[1])

    def at(a):
        return sar_geo.dest(d, (C + sg * a) % 360.0, R)
    return [d, at(0), at(60), d, at(240), at(300), d, at(120), at(180), d]


def parallel_track(datum, S, C, W, Lg, turn="starboard"):
    """The datum is the start corner; tracks run along C (length Lg), stepping to the side (C + sgn 90) by S.
    n = ceil(W/S) tracks, the first S/2 inside the edge, search legs Lg - S (S/2 in from each end), cross legs S.
    -> 2n points (needs Lg > S)."""
    sg = _sgn(turn)
    d = (datum[0], datum[1])
    side = (C + sg * 90.0) % 360.0
    n = int(math.ceil(W / S - 1e-9))
    pts = []
    for i in range(n):
        base = sar_geo.dest(d, side, S / 2.0 + i * S)
        a = sar_geo.dest(base, C, S / 2.0)
        b = sar_geo.dest(base, C, Lg - S / 2.0)
        pts += [a, b] if i % 2 == 0 else [b, a]
    return pts


def parallel_total(S, W, Lg):
    n = int(math.ceil(W / S - 1e-9))
    return n * (Lg - S) + (n - 1) * S


def creeping_line(datum, S, C, W, Lg, turn="starboard"):
    """Legs of length Lg - S across the track (along C - sgn 90); the search ADVANCES along C over W."""
    sg = _sgn(turn)
    return parallel_track(datum, S, (C - sg * 90.0) % 360.0, W, Lg, turn)


def _legs(pts):
    out = []
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        d, brg = sar_geo.distance_bearing(a, b)
        out.append({"n": i + 1, "m": d, "brg": brg, "a": a, "b": b})
    return out


def plan(kind, datum, p):
    """p = {"S" m, "C" deg, "turn", "kn", and per kind: "legs" or "R" (ss), "R" (vs), "W" + "Lg" (ps, cs)}.
    -> {"kind", "wps", "legs", "total_m", "time_s", "area_m2", "n", "turn", "S", "C", "kn"} or {"why": key}."""
    try:
        S = float(p.get("S"))
        C = float(p.get("C", 0.0)) % 360.0
        kn = float(p.get("kn", 5.0))
        datum = (float(datum[0]), float(datum[1]))
    except (TypeError, ValueError):
        return {"why": "input"}
    turn = "port" if p.get("turn") == "port" else "starboard"
    if not all(math.isfinite(v) for v in (S, C, kn, datum[0], datum[1])) or not (-90 < datum[0] < 90) \
            or not (-180 <= datum[1] <= 180):
        return {"why": "input"}
    if not (1.0 <= S <= 20000.0) or not (0.5 <= kn <= 60.0):
        return {"why": "input"}
    try:
        if kind == "ss":
            legs = p.get("legs")
            if legs is None:
                R = float(p.get("R", 0.0))
                if not (R >= S):
                    return {"why": "size"}
                legs = legs_for_half_side(R, S)
            legs = int(legs)
            if not (2 <= legs <= MAX_LEGS):
                return {"why": "size"}
            pts = expanding_square(datum, S, C, legs, turn)
        elif kind == "vs":
            R = float(p.get("R", 0.0))
            if not (10.0 <= R <= 50000.0):
                return {"why": "size"}
            pts = sector(datum, R, C, turn)
        elif kind in ("ps", "cs"):
            W, Lg = float(p.get("W", 0.0)), float(p.get("Lg", 0.0))
            if not (W >= S / 2.0 and Lg > S and math.isfinite(W) and math.isfinite(Lg)):
                return {"why": "size"}
            if int(math.ceil(W / S - 1e-9)) > MAX_LEGS:
                return {"why": "size"}
            pts = (parallel_track if kind == "ps" else creeping_line)(datum, S, C, W, Lg, turn)
        else:
            return {"why": "kind"}
    except (TypeError, ValueError):
        return {"why": "size"}
    legs_l = _legs(pts)
    total = sum(l["m"] for l in legs_l)
    return {"kind": kind, "wps": pts, "legs": legs_l, "total_m": total, "time_s": total / (kn * M_PER_NM / 3600.0),
            "area_m2": total * S, "n": len(pts), "turn": turn, "S": S, "C": C, "kn": kn}


def boat_check(pl):
    """None when the plan fits the AURIX's route (<= 50 waypoints), else {"why": "too_many", "n", "cut_leg", "max"}:
    the first 50 waypoints end on leg `cut_leg`. The caller shows it; nothing is ever cut silently."""
    if pl["n"] <= MAX_BOAT_WP:
        return None
    return {"why": "too_many", "n": pl["n"], "cut_leg": MAX_BOAT_WP - 1, "max": MAX_BOAT_WP}


def cut_at_limit(pl):
    """The operator chose "cut at 50": the first 50 waypoints (ending on leg 49). A new plan dict."""
    pts = pl["wps"][:MAX_BOAT_WP]
    legs_l = _legs(pts)
    total = sum(l["m"] for l in legs_l)
    return dict(pl, wps=pts, legs=legs_l, total_m=total, n=len(pts), cut=True,
                time_s=total / (pl["kn"] * M_PER_NM / 3600.0), area_m2=total * pl["S"])


def to_boat_program(pl, radius_m=None, final=1):
    """A plan (<= 50 waypoints) -> the program dict of the map's route (the normal upload + every check). final =
    PROG_END_HOLD (1): the search goes on in heading hold at the approach speed + the arrival alarm."""
    r = float(radius_m) if radius_m is not None else max(5.0, min(25.0, pl["S"] / 4.0))
    return {"final": int(final),
            "wps": [{"lat": la, "lon": lo, "kn": float(pl["kn"]), "radius": r, "action": 0} for la, lo in pl["wps"]]}


def uav_points(pl, alt_m):
    """The UAV mission points {"lat", "lon", "alt"} (altitude per waypoint); the gateway ends it LOITER_UNLIM over the
    last one and uavcheck decides (energy, range, altitude, no-fly)."""
    return [{"lat": la, "lon": lo, "alt": float(alt_m)} for la, lo in pl["wps"]]


def summary(pl):
    """The numbers of the panel line: waypoints, legs, nm, minutes, area nm2."""
    return {"n": pl["n"], "nlegs": len(pl["legs"]), "nm": pl["total_m"] / M_PER_NM, "min": pl["time_s"] / 60.0,
            "area_nm2": pl["area_m2"] / (M_PER_NM ** 2), "m": pl["total_m"]}
