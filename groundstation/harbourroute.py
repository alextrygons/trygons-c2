"""Teach and repeat (step HX6, CLAUDE.md Z; docs/harbour_design.md 8.2; owner M5): a recorded MANUAL harbour exit /
entry (the 13n3 track) -> a CHECKED harbour route. PURE (no tkinter, no network, no sender): the track points in, a
route dict out; the station window (harbourpanel.py) shows it, the gateway checks it (114 shallow / 120 geofence /
129 corridor: gateway.harbour_route_check) and the normal program upload carries it with the PROG_F_HARBOUR flag.

A route = {"format": 1, "harbour", "name", "corr_m", "flags": 1, "final": 0, "wps": [{lat, lon, kn, radius, action}],
"cut_start_m", "cut_end_m", "source", "not_for_navigation"}. It is the station's recording of what a person drove -
never a chart, never a guarantee: the planner + the AURIX keep the boat in the corridor, the first / last metres at the
quay stay manual (M0-8: the route starts / ends >= quay_m from the shore, else a warning).
"""
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES_DIR = os.path.join(HERE, "routes", "harbour")        # <harbour>/<name>.json (user routes gitignored)
SAMPLES_DIR = os.path.join(ROUTES_DIR, "_samples")           # committed TEST samples (not_for_navigation, placeholder)
MAX_WP = 50                                                  # app/program.h PROG_MAX_WP
DEFAULTS = {"corr_m": 10.0,          # W146: +-10 m around the route (the AURIX clamps to harb_corr_m)
            "corr_max_m": 10.0,      # the widest corridor offered (= harb_corr_m default; the gateway checks up to its corr_m)
            "tol_m": 2.0,            # Douglas-Peucker tolerance (design 8.2)
            "cut_m": 15.0,           # M0-8: the first / last 15 m from the quay stay manual
            "quay_m": 15.0,          # warn when the first / last waypoint is nearer than this to the shore
            "max_kn": 5.0,           # harb_max_kn
            "min_kn": 1.0,           # app/program.c leg speed 1..
            "min_leg_m": 12.0,       # app/smallboat.c min_leg_m 10 + margin
            "radius_m": 6.0}         # arrival radius (program.c 5..500, and the leg must be longer than it)
M_DEG = 111320.0
FLAG_HARBOUR = 1


def _dist(a, b):
    k = math.cos(math.radians((a[0] + b[0]) / 2.0))
    return math.hypot((b[1] - a[1]) * k * M_DEG, (b[0] - a[0]) * M_DEG)


def _seg_d(p, a, b):
    k = math.cos(math.radians(a[0]))
    ax, ay = 0.0, 0.0
    bx, by = (b[1] - a[1]) * k * M_DEG, (b[0] - a[0]) * M_DEG
    px, py = (p[1] - a[1]) * k * M_DEG, (p[0] - a[0]) * M_DEG
    n2 = bx * bx + by * by
    if n2 < 1e-9:
        return math.hypot(px, py)
    u = max(0.0, min(1.0, (px * bx + py * by) / n2))
    return math.hypot(px - u * bx, py - u * by)


def douglas_peucker(pts, tol_m):
    """Indices of the points kept (first and last always); iterative (no recursion: CLAUDE.md stack rules are the
    firmware's, but a long track would also hit Python's limit)."""
    n = len(pts)
    if n <= 2:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        best, bi = -1.0, -1
        for m in range(i + 1, j):
            d = _seg_d(pts[m], pts[i], pts[j])
            if d > best:
                best, bi = d, m
        if bi >= 0 and best > tol_m:
            keep[bi] = True
            stack.append((i, bi))
            stack.append((bi, j))
    return [i for i in range(n) if keep[i]]


def cut_ends(pts, cut_start_m, cut_end_m):
    """Drop the first / last metres of the path (measured along it). -> the kept sub-list (the cut points' interior
    is interpolated at the cut so the new ends lie ON the path), [] when nothing is left (>= 2 points needed)."""
    if len(pts) < 2:
        return []
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + _dist(a, b))
    total = cum[-1]
    s0, s1 = max(0.0, cut_start_m), total - max(0.0, cut_end_m)
    if s1 - s0 < 1.0:
        return []

    def at(s):
        for i in range(1, len(pts)):
            if cum[i] >= s:
                seg = cum[i] - cum[i - 1]
                u = 0.0 if seg < 1e-9 else (s - cum[i - 1]) / seg
                return (pts[i - 1][0] + u * (pts[i][0] - pts[i - 1][0]), pts[i - 1][1] + u * (pts[i][1] - pts[i - 1][1]))
        return pts[-1]
    mid = [p for p, c in zip(pts, cum) if s0 < c < s1]
    return [at(s0)] + mid + [at(s1)]


def simplify(pts, tol_m=2.0, max_wp=MAX_WP, min_leg_m=12.0):
    """Douglas-Peucker to <= max_wp points: the tolerance grows (x1.4) until it fits; waypoints closer than
    min_leg_m to the previous one are dropped (the AURIX refuses legs under 10 m), the last one kept.
    -> (points, tolerance used)."""
    if len(pts) < 2:
        return list(pts), tol_m
    tol = float(tol_m)
    for _ in range(60):
        keep = [pts[i] for i in douglas_peucker(pts, tol)]
        out = [keep[0]]
        for p in keep[1:-1]:
            if _dist(out[-1], p) >= min_leg_m:
                out.append(p)
        if keep[-1] != out[-1]:
            if _dist(out[-1], keep[-1]) < min_leg_m and len(out) > 1:
                out.pop()
            out.append(keep[-1])
        if len(out) <= max_wp:
            return out, tol
        tol *= 1.4
    return out[:max_wp], tol


def leg_speeds(track_pts, wps, max_kn, min_kn):
    """Leg speed to each waypoint = the average of the recorded speeds (kn) between the two nearest track points,
    limited to [min_kn, max_kn]; no recorded speed = min(max_kn, 3 kn). track_pts = [(lat, lon, kn | None)]."""
    def nearest(w):
        return min(range(len(track_pts)), key=lambda i: _dist((track_pts[i][0], track_pts[i][1]), w))
    idx = [nearest(w) for w in wps]
    out = []
    for k in range(len(wps)):
        a, b = (idx[k - 1] if k else 0), idx[k]
        sp = [p[2] for p in track_pts[min(a, b):max(a, b) + 1] if p[2] is not None]
        kn = (sum(sp) / len(sp)) if sp else min(max_kn, 3.0)
        out.append(round(max(min_kn, min(max_kn, kn)), 1))
    return out


def from_track(points, harbour, name, opts=None, i0=0, i1=None, source=""):
    """points = a saved track's pts [(t, lat, lon, kn, hdg)] (groundstation/track.py); i0..i1 = the segment (indices,
    inclusive). -> {"route": dict | None, "why": "" | "too short" ..., "tol_m": used, "n_raw": n}."""
    o = dict(DEFAULTS, **(opts or {}))
    seg = [p for p in points[i0:(None if i1 is None else i1 + 1)]]
    pts3 = [(p[1], p[2], p[3]) for p in seg]
    ll = [(p[0], p[1]) for p in pts3]
    cut = cut_ends(ll, float(o["cut_m"]), float(o["cut_m"]))
    if len(cut) < 2:
        return {"route": None, "why": "too_short", "tol_m": o["tol_m"], "n_raw": len(seg)}
    wps, tol = simplify(cut, float(o["tol_m"]), MAX_WP, float(o["min_leg_m"]))
    if len(wps) < 2:
        return {"route": None, "why": "too_short", "tol_m": tol, "n_raw": len(seg)}
    kn = leg_speeds(pts3, wps, float(o["max_kn"]), float(o["min_kn"]))
    route = {"format": 1, "harbour": str(harbour), "name": str(name), "corr_m": float(o["corr_m"]), "flags": FLAG_HARBOUR,
             "final": 0, "cut_start_m": float(o["cut_m"]), "cut_end_m": float(o["cut_m"]), "tol_m": round(tol, 2),
             "source": source, "not_for_navigation": False,
             "wps": [{"lat": round(w[0], 7), "lon": round(w[1], 7), "kn": k, "radius": float(o["radius_m"]),
                      "action": 0 if i < len(wps) - 1 else 2} for i, (w, k) in enumerate(zip(wps, kn))]}
    return {"route": route, "why": "", "tol_m": tol, "n_raw": len(seg)}


def length_m(route):
    w = route.get("wps") or []
    return sum(_dist((a["lat"], a["lon"]), (b["lat"], b["lon"])) for a, b in zip(w, w[1:]))


def local_check(route, opts=None):
    """The station's own quick check (the gateway + the AURIX check again): -> [(key, args dict)] of REFUSALS
    (empty = fine). Keys are lang keys harb.rt_<key>."""
    o = dict(DEFAULTS, **(opts or {}))
    out = []
    w = (route or {}).get("wps") or []
    if len(w) < 2:
        return [("few", {"n": len(w)})]
    if len(w) > MAX_WP:
        out.append(("many", {"n": len(w), "max": MAX_WP}))
    corr = route.get("corr_m")
    if not (isinstance(corr, (int, float)) and math.isfinite(corr) and 4.0 <= corr <= o["corr_max_m"]):
        out.append(("corr", {"m": corr}))
    for i, x in enumerate(w):
        try:
            ok = (math.isfinite(x["lat"]) and math.isfinite(x["lon"]) and abs(x["lat"]) <= 90 and abs(x["lon"]) <= 180 and
                  o["min_kn"] <= x["kn"] <= 8.0 and 5.0 <= x["radius"] <= 500.0)
        except (KeyError, TypeError):
            ok = False
        if not ok:
            out.append(("wp", {"n": i + 1}))
    for i, (a, b) in enumerate(zip(w, w[1:])):
        try:
            d = _dist((a["lat"], a["lon"]), (b["lat"], b["lon"]))
        except (KeyError, TypeError):
            continue
        if d < 10.0 or d <= b.get("radius", 5.0):
            out.append(("leg", {"n": i + 1, "m": "%.0f" % d}))
    return out


def quay_warnings(route, shore_dist_fn, near_m=None):
    """M0-8: berthing and the first metres from the quay stay manual. shore_dist_fn(lat, lon) -> metres to the shore /
    shallow water (None = unknown). A route whose first or last waypoint is nearer than near_m (default 15 m) gets
    a warning each; unknown = a "no data" warning (never silently fine). -> [(key, args)]."""
    near = float(DEFAULTS["quay_m"] if near_m is None else near_m)
    w = (route or {}).get("wps") or []
    out = []
    for key, p in (("start", w[0] if w else None), ("end", w[-1] if w else None)):
        if p is None:
            continue
        try:
            d = shore_dist_fn(p["lat"], p["lon"]) if shore_dist_fn else None
        except Exception:                                    # a chart error is "unknown", never a crash
            d = None
        if d is None:
            out.append(("quay_unknown_" + key, {}))
        elif d < near:
            out.append(("quay_" + key, {"m": "%.0f" % d, "near": "%.0f" % near}))
    return out


# --- files ------------------------------------------------------------------------------------------------------
def safe(s, default="harbour"):
    s = re.sub(r"[^\w.\-]+", "-", str(s or ""), flags=re.UNICODE).strip("-.")
    return s[:50] or default


def route_path(harbour, name, root=None):
    return os.path.join(root or ROUTES_DIR, safe(harbour), safe(name, "route") + ".json")


def save(route, root=None):
    """-> the path. OSError on a disk problem (the caller says so)."""
    p = route_path(route.get("harbour"), route.get("name"), root)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(route, f, indent=1, ensure_ascii=False)
    os.replace(tmp, p)
    return p


def load(path):
    """A route file -> dict; ValueError when it is not a usable route (never a half-read one)."""
    try:
        with open(path, encoding="utf-8") as f:
            r = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        raise ValueError(str(e))
    if not isinstance(r, dict) or not isinstance(r.get("wps"), list) or not r["wps"]:
        raise ValueError("no waypoints")
    r.setdefault("corr_m", DEFAULTS["corr_m"])
    r["flags"] = FLAG_HARBOUR
    r.setdefault("final", 0)
    return r


def listing(root=None):
    """[(harbour, name, path, sample)] of every saved route + the committed samples."""
    out = []
    for base, sample in ((root or ROUTES_DIR, False), (SAMPLES_DIR if root is None else None, True)):
        if not base or not os.path.isdir(base):
            continue
        for h in sorted(os.listdir(base)):
            d = os.path.join(base, h)
            if not os.path.isdir(d) or (not sample and h.startswith("_")):
                continue
            for fn in sorted(os.listdir(d)):
                if fn.endswith(".json"):
                    out.append((h, fn[:-5], os.path.join(d, fn), sample))
    return out


def to_program(route):
    """The route as the station's AUTO program dict (program.py) with the harbour flag: it goes through the normal
    program upload frames (PROG_BEGIN 3rd byte)."""
    return {"final": int(route.get("final", 0)), "flags": FLAG_HARBOUR,
            "wps": [dict(lat=w["lat"], lon=w["lon"], kn=w["kn"], radius=w["radius"], action=w.get("action", 0))
                    for w in route["wps"]]}
