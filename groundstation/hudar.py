"""Augmented-reality overlays on the FRONT video (section B). DISPLAY ONLY.

The camera model is the same as the gateway's obstacle detection
(gateway/vision/geometry.py): camera height above the water, how far it looks
down (tilt), field of view, yaw; the boat's roll / pitch from the IMU (TELEM2
group H) move the horizon. The calibration comes from the gateway's vision
status ("camera", only when it is set in the gateway config) or from this PC's
settings (gs_settings.json "camera"); without one the AR overlays are hidden and
"calibration unknown" is shown.

  B1 horizon line + heel (roll) scale arc        B5 AIS targets boxed at their bearing
  B2 velocity vector (course over ground)        B6 predicted path 30 s / 60 s
  B3 waypoint diamond on the horizon             B7 shallow water ahead tinted red
  B4 route ribbon on the water                   (B8 the vision overlay: driver.py)

build(screen, snapshot, pic, W, H) -> display-list ops (hud.py format), drawn
UNDER the HUD panels. The maths (Cam, path_points, ...) is tested in test_hud_ar.py.
"""
import math
import time

import settings
import theme
from i18n import T

KEYS = ("horizon", "velocity", "waypoint", "ribbon", "ais", "path", "shallow")
LIMITS = {"height_m": (0.2, 20.0), "tilt_down_deg": (-30.0, 60.0), "hfov_deg": (5.0, 170.0),
          "vfov_deg": (5.0, 170.0), "yaw_deg": (-180.0, 180.0)}


def calibration(gw):
    """The camera calibration to use, or None (unknown / not usable)."""
    cam = ((gw or {}).get("vision") or {}).get("camera") or settings.get("camera")
    if not isinstance(cam, dict):
        return None
    out = {}
    for k, (lo, hi) in LIMITS.items():
        v = cam.get(k, 0.0 if k == "yaw_deg" else None)
        try:
            v = float(v)
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(v) and lo <= v <= hi):
            return None
        out[k] = v
    return out


class Cam:
    """Pinhole camera on the bow. Picture coordinates u, v = 0..1 (u right, v down)."""

    def __init__(self, cal, roll=0.0, pitch=0.0):
        self.c = cal
        self.fx = 0.5 / math.tan(math.radians(cal["hfov_deg"]) / 2.0)
        self.fy = 0.5 / math.tan(math.radians(cal["vfov_deg"]) / 2.0)
        self.look_down = math.radians(cal["tilt_down_deg"] - (pitch or 0.0))     # bow up -> looks up
        self.rho = math.radians(roll or 0.0)                                       # + = starboard down
        self.yaw = math.radians(cal.get("yaw_deg", 0.0))

    def _rot(self, X, Y):
        """Heel: the picture turns (starboard down -> the horizon rises to the right)."""
        th = -self.rho
        return X * math.cos(th) - Y * math.sin(th), X * math.sin(th) + Y * math.cos(th)

    def project(self, fwd, right, down=None):
        """A point fwd m ahead, right m to starboard, down m below the camera (default: on the water)
        -> (u, v), or None when it is behind the camera."""
        down = self.c["height_m"] if down is None else down
        f = fwd * math.cos(self.yaw) + right * math.sin(self.yaw)
        r = -fwd * math.sin(self.yaw) + right * math.cos(self.yaw)
        if f <= 0.5:
            return None
        X = r / f                                                   # tan of the azimuth
        dep = math.atan2(down, math.hypot(f, r))                    # angle below the horizontal
        Y = math.tan(dep - self.look_down)
        X, Y = self._rot(X, Y)
        return 0.5 + self.fx * X, 0.5 + self.fy * Y

    def project_bearing(self, rel_deg, dist_m=None):
        """A direction rel_deg off the bow: on the horizon (dist None) or on the water at dist_m.
        -> (u, v, in_view) ; outside the field of view u is clamped to the edge."""
        b = math.radians(rel_deg)
        if dist_m is None:
            p = self.project(math.cos(b) * 1e6, math.sin(b) * 1e6, 0.0)
        else:
            p = self.project(math.cos(b) * dist_m, math.sin(b) * dist_m)
        half = self.c["hfov_deg"] / 2.0
        rel_cam = (rel_deg - self.c.get("yaw_deg", 0.0) + 540.0) % 360.0 - 180.0
        in_view = abs(rel_cam) < half and p is not None and 0.0 <= p[0] <= 1.0
        if p is None:
            u = 0.0 if rel_cam < 0 else 1.0
            p = (u, self.horizon_v(u))
        return p[0], p[1], in_view

    def horizon_v(self, u):
        """v of the horizon at u (on the line through the two horizon points)."""
        (u1, v1), (u2, v2) = self.horizon()
        return v1 + (v2 - v1) * (u - u1) / ((u2 - u1) or 1e-9)

    def horizon(self):
        """Two picture points far left / right on the horizon."""
        Y0 = math.tan(-self.look_down)
        pts = []
        for X in (-3.0, 3.0):
            Xr, Yr = self._rot(X, Y0)
            pts.append((0.5 + self.fx * Xr, 0.5 + self.fy * Yr))
        return pts


def enu(lat0, lon0, lat, lon):
    """(east, north) metres of (lat, lon) from (lat0, lon0)."""
    return (lon - lon0) * math.cos(math.radians(lat0)) * 111320.0, (lat - lat0) * 111320.0


def to_boat(e, n, heading):
    """(east, north) -> (forward, right) of the bow."""
    h = math.radians(heading)
    return n * math.cos(h) + e * math.sin(h), e * math.cos(h) - n * math.sin(h)


def path_points(sog_kn, yaw_dps, drift_deg=0.0, seconds=60.0, step=2.0):
    """Where the boat will be (forward, right in metres of the bow now) at the current speed and
    turn rate, every `step` seconds up to `seconds`. drift_deg = course over ground - heading."""
    v = max(0.0, sog_kn) * 0.514444
    out, f, r, th = [(0.0, 0.0, 0.0)], 0.0, 0.0, math.radians(drift_deg)
    w = math.radians(yaw_dps)
    t = 0.0
    while t < seconds - 1e-9:
        dt = min(step, seconds - t)
        mid = th + w * dt / 2.0
        f += v * math.cos(mid) * dt
        r += v * math.sin(mid) * dt
        th += w * dt
        t += dt
        out.append((t, f, r))
    return out


def route_points(lat0, lon0, legs, step_m=25.0, max_m=2500.0):
    """Points along the route (boat -> wp -> wp ...) every step_m, as [(lat, lon, distance along)]."""
    pts, run = [], 0.0
    prev = (lat0, lon0)
    for la, lo in legs:
        e, n = enu(prev[0], prev[1], la, lo)
        L = math.hypot(e, n)
        k = 0
        while k * step_m <= L and run + k * step_m <= max_m:
            f = k * step_m / L if L > 0 else 1.0
            pts.append((prev[0] + (la - prev[0]) * f, prev[1] + (lo - prev[1]) * f, run + k * step_m))
            k += 1
        run += L
        prev = (la, lo)
        if run > max_m:
            break
    return pts


_SHALLOW = {"key": None, "cells": [], "t": 0.0}


def shallow_cells(charts, lat, lon, heading, limit, half_fov):
    """Cells of water ahead shallower than the limit: [(d0, d1, b0, b1)] (metres, deg off the bow).
    Cached: recomputed when the boat moved ~10 m / turned 2 deg, at most once a second."""
    key = (round(lat, 4), round(lon, 4), int(heading // 2), round(limit, 1), int(half_fov))
    now = time.time()
    if _SHALLOW["key"] == key or now - _SHALLOW["t"] < 1.0:
        return _SHALLOW["cells"]
    dists = (12, 20, 30, 45, 65, 90, 125, 170, 230, 310, 420, 560, 750)
    k = math.cos(math.radians(lat)) * 111320.0
    cells = []
    b = -half_fov
    while b < half_fov:
        b1 = min(half_fov, b + 4.0)
        bm = math.radians(heading + (b + b1) / 2.0)
        for d0, d1 in zip(dists, dists[1:]):
            dm = (d0 + d1) / 2.0
            la, lo = lat + dm * math.cos(bm) / 111320.0, lon + dm * math.sin(bm) / k
            dep = charts.depth(la, lo)
            if (dep is not None and dep < limit) or (dep is None and charts.area_at(la, lo)):
                cells.append((d0, d1, b, b1))
        b = b1
    _SHALLOW.update(key=key, cells=cells, t=now)
    return cells


# --- the overlays -------------------------------------------------------------------------------------------
def build(screen, s, pic, W, H):
    """Display-list ops for the AR overlays (under the HUD panels)."""
    import hud
    a = screen.app
    d = s.t
    ar = settings.get("ar") or {}
    if not any(ar.get(k, True) for k in KEYS):
        return []
    pal = hud.palette(s.hud_colour or "green", bool(s.night))
    main = pal["main"]
    cal = calibration(s.gw)
    ops = []
    f_s = max(10, int(H * 0.018))

    measure = getattr(screen.hud, "measure", lambda t_, s_, b_=False: len(t_) * s_ * 0.6)
    lo_x, hi_x = W * 0.10, W * 0.86          # AR labels / edge arrows stay in the free middle (HUD columns aside)

    def text(xy, txt, tag, col=main, anchor="c", size=f_s, bold=True):
        """A label, moved inside the window when it would stick out at the side."""
        x, y = xy
        w = measure(txt, size, bold)
        left = x - (w / 2 if anchor in ("c", "n", "s") else w if anchor in ("e", "ne", "se") else 0)
        x += max(0.0, lo_x - left) - max(0.0, left + w - hi_x)
        ops.append({"k": "text", "xy": (x, y), "s": txt, "tag": tag, "size": size, "bold": bold, "fill": col,
                    "anchor": anchor, "max_w": None, "box": None})

    if cal is None:
        text((W / 2, H * 0.265), T("ar.no_cal"), "ar_nocal", col=pal["caution"])
        return ops
    roll, pitch = d.get("att_roll"), d.get("att_pitch")
    cam = Cam(cal, roll or 0.0, pitch or 0.0)
    x0, y0, pw, ph = pic
    X = lambda u: x0 + u * pw                       # noqa: E731
    Y = lambda v: y0 + v * ph                       # noqa: E731

    def P(fwd, right, down=None):
        p = cam.project(fwd, right, down)
        return None if p is None else (X(p[0]), Y(p[1]))

    hd = d["heading"]
    lat, lon = d["est_lat"], d["est_lon"]
    have_pos = bool(d.get("pos_source"))

    # B7 shallow tint (first: everything else on top)
    if ar.get("shallow", True) and have_pos and a.charts:
        lim = s.shallow_limit or 3.0
        for d0, d1, b0, b1 in shallow_cells(a.charts, lat, lon, hd, lim, cal["hfov_deg"] / 2.0 + 4):
            corners = []
            for dd, bb in ((d0, b0), (d0, b1), (d1, b1), (d1, b0)):
                p = P(dd * math.cos(math.radians(bb)), dd * math.sin(math.radians(bb)))
                if p is None:
                    break
                corners += p
            if len(corners) == 8:
                ops.append({"k": "poly", "pts": corners, "tag": "ar_shallow", "fill": theme.AR["shallow"], "outline": None,
                            "alpha": 70, "w": 1})
    # B4 route ribbon (fading with the distance)
    if ar.get("ribbon", True) and have_pos and d.get("wp_set"):
        legs = [(d["wp_lat"], d["wp_lon"])]
        wps = (a.prog or {}).get("wps") or []
        if len(wps) == d["prog_n"]:
            legs += [(w["lat"], w["lon"]) for w in wps[d["auto_wp"] + 1:]]
        pts = route_points(lat, lon, legs)
        half = 3.0                                                  # ribbon half width, metres
        prev = None
        for la, lo, run in pts:
            e, n = enu(lat, lon, la, lo)
            f, r = to_boat(e, n, hd)
            q = None
            if f > 15.0:                            # from 15 m ahead (closer: a huge wedge)
                q = (P(f, r - half), P(f, r + half))
                if None in q:
                    q = None
            if q and prev:
                fade = max(0.0, 1.0 - run / 2500.0)
                ops.append({"k": "poly", "pts": [*prev[0], *prev[1], *q[1], *q[0]], "tag": "ar_ribbon",
                            "fill": theme.AR["route"], "outline": None, "alpha": int(40 + 120 * fade), "w": 1})
            prev = q
    # B6 predicted path 30 / 60 s
    if ar.get("path", True) and d["est_sog"] > 0.5:
        drift = ((d["est_cog"] - hd + 540.0) % 360.0 - 180.0) if d["est_sog"] > 1.0 else 0.0
        line = []
        for t, f, r in path_points(d["est_sog"], d["yaw_rate"], drift):
            p = P(f, r) if f > 4.0 else None
            if p:
                line += p
            if t in (30.0, 60.0) and p:
                ops.append({"k": "oval", "r": (p[0] - 6, p[1] - 4, p[0] + 6, p[1] + 4), "tag": "ar_path",
                            "fill": None, "outline": main, "w": 2, "alpha": 255})
                text((p[0] + 10, p[1]), T("ar.t_s", s=int(t)), "ar_path", anchor="w")
        if len(line) >= 4:
            ops.append({"k": "line", "pts": line, "tag": "ar_path", "fill": main, "w": 3, "alpha": 220})
    # B5 AIS targets
    if ar.get("ais", True) and have_pos:
        rows = a.ais_rows_now() if hasattr(a, "ais_tree") else []
        for t in rows[:20]:
            if t.get("lat") is None:
                continue
            e, n = enu(lat, lon, t["lat"], t["lon"])
            dist = math.hypot(e, n)
            if dist > 3 * 1852.0:
                continue
            rel = (math.degrees(math.atan2(e, n)) - hd + 540.0) % 360.0 - 180.0
            col = pal["warn"] if t.get("level") == "danger" else pal["caution"] if t.get("level") == "warn" else main
            u, v, inview = cam.project_bearing(rel, max(dist, 30.0))
            lab = T("ar.ais", name=(t.get("name") or str(t["mmsi"]))[:16], d=hud.fmt_dist(dist),
                    cpa="-" if t.get("cpa_nm") is None else "%.2f" % t["cpa_nm"])
            if inview:
                bh = max(14.0, min(90.0, 2600.0 / max(dist, 30.0)))
                bx, by = X(u), Y(v)
                ops.append({"k": "rect", "r": (bx - bh * 0.8, by - bh, bx + bh * 0.8, by + 2), "tag": "ar_ais",
                            "fill": None, "outline": col, "w": 2, "alpha": 255})
                text((bx, by - bh - 3), lab, "ar_ais", col=col, anchor="s")
            elif t.get("level"):
                ex = max(lo_x + 20, X(0.0) + 30) if rel < 0 else min(hi_x - 20, X(1.0) - 30)
                ey = H * 0.42
                sgn = -1 if rel < 0 else 1
                ops.append({"k": "poly", "pts": [ex + sgn * 18, ey, ex, ey - 12, ex, ey + 12], "tag": "ar_ais_edge",
                            "fill": col, "outline": theme.BLACK, "alpha": 255, "w": 1})
                text((ex - sgn * 6, ey + 22), lab, "ar_ais_edge", col=col, anchor="e" if sgn > 0 else "w")
    # B3 waypoint diamond on the horizon
    if ar.get("waypoint", True) and have_pos and d.get("wp_set"):
        e, n = enu(lat, lon, d["wp_lat"], d["wp_lon"])
        dist = math.hypot(e, n)
        rel = (math.degrees(math.atan2(e, n)) - hd + 540.0) % 360.0 - 180.0
        u, v, inview = cam.project_bearing(rel)
        lab = T("ar.wp", n=d["auto_wp"] + 1, d=hud.fmt_dist(dist))
        if inview:
            cx, cy = X(u), Y(v)
            ops.append({"k": "poly", "pts": [cx, cy - 16, cx + 12, cy, cx, cy + 16, cx - 12, cy], "tag": "ar_wp",
                        "fill": None, "outline": theme.AR["route"], "alpha": 255, "w": 3})
            ops.append({"k": "line", "pts": [cx, cy - 16, cx + 12, cy, cx, cy + 16, cx - 12, cy, cx, cy - 16],
                        "tag": "ar_wp", "fill": theme.AR["route"], "w": 3, "alpha": 255})
            text((cx, cy - 20), lab, "ar_wp", col=theme.AR["route_text"], anchor="s")
        else:
            sgn = -1 if rel < 0 else 1
            ex = max(lo_x + 20, X(0.0) + 40) if sgn < 0 else min(hi_x - 20, X(1.0) - 40)
            ey = H * 0.36
            ops.append({"k": "poly", "pts": [ex + sgn * 20, ey, ex, ey - 13, ex, ey + 13], "tag": "ar_wp_edge",
                        "fill": theme.AR["route"], "outline": theme.BLACK, "alpha": 255, "w": 1})
            text((ex - sgn * 6, ey + 22), lab, "ar_wp_edge", col=theme.AR["route_text"], anchor="e" if sgn > 0 else "w")
    # B1 horizon + heel scale
    if ar.get("horizon", True):
        (u1, v1), (u2, v2) = cam.horizon()
        p1, p2 = (X(u1), Y(v1)), (X(u2), Y(v2))
        mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
        L = math.hypot(p2[0] - p1[0], p2[1] - p1[1]) or 1.0
        ux, uy = (p2[0] - p1[0]) / L, (p2[1] - p1[1]) / L
        gap = W * 0.06
        ops.append({"k": "line", "pts": [p1[0], p1[1], mx - ux * gap, my - uy * gap], "tag": "ar_horizon",
                    "fill": main, "w": 2, "alpha": 200})
        ops.append({"k": "line", "pts": [mx + ux * gap, my + uy * gap, p2[0], p2[1]], "tag": "ar_horizon",
                    "fill": main, "w": 2, "alpha": 200})
        # the heel arc: fixed scale, the pointer shows the roll (starboard down = pointer to the right)
        cx, cy, rad = W / 2, H * 0.47, H * 0.12
        for deg in (-45, -30, -20, -10, 0, 10, 20, 30, 45):
            ang = math.radians(deg)
            ln = 10 if deg % 30 == 0 or deg == 45 or deg == -45 else 6
            ops.append({"k": "line", "pts": [cx + rad * math.sin(ang), cy - rad * math.cos(ang),
                                             cx + (rad + ln) * math.sin(ang), cy - (rad + ln) * math.cos(ang)],
                        "tag": "ar_roll", "fill": main, "w": 2, "alpha": 230})
        if roll is not None:
            ang = math.radians(max(-50.0, min(50.0, roll)))
            px, py = cx + (rad - 3) * math.sin(ang), cy - (rad - 3) * math.cos(ang)
            ops.append({"k": "poly", "pts": [px, py, px - 7 * math.cos(ang) - 10 * math.sin(ang),
                                             py - 7 * math.sin(ang) + 10 * math.cos(ang),
                                             px + 7 * math.cos(ang) - 10 * math.sin(ang),
                                             py + 7 * math.sin(ang) + 10 * math.cos(ang)],
                        "tag": "ar_roll", "fill": main, "outline": theme.BLACK, "alpha": 255, "w": 1})
            text((cx, cy - rad - 16), T("ar.heel", deg="%.0f" % abs(roll), side=T("hud.stbd") if roll > 0.5 else
                                         T("hud.port") if roll < -0.5 else ""), "ar_roll", anchor="s")
        else:
            text((cx, cy - rad - 16), T("ar.no_imu"), "ar_roll", col=pal["caution"], anchor="s")
    # B2 velocity vector (where the boat goes: course over ground)
    if ar.get("velocity", True) and d["est_sog"] > 0.5:
        drift = (d["est_cog"] - hd + 540.0) % 360.0 - 180.0
        u, v, inview = cam.project_bearing(drift, max(60.0, d["est_sog"] * 0.514444 * 30.0))
        if inview:
            cx, cy = X(u), Y(v)
            r = 9
            ops.append({"k": "oval", "r": (cx - r, cy - r, cx + r, cy + r), "tag": "ar_velocity", "fill": None,
                        "outline": main, "w": 3, "alpha": 255})
            ops.append({"k": "line", "pts": [cx - r - 14, cy, cx - r, cy], "tag": "ar_velocity", "fill": main,
                        "w": 3, "alpha": 255})
            ops.append({"k": "line", "pts": [cx + r, cy, cx + r + 14, cy], "tag": "ar_velocity", "fill": main,
                        "w": 3, "alpha": 255})
            ops.append({"k": "line", "pts": [cx, cy - r, cx, cy - r - 9], "tag": "ar_velocity", "fill": main,
                        "w": 3, "alpha": 255})
            if abs(drift) >= 2:
                text((cx, cy + r + 4), T("ar.drift", deg="%.0f" % abs(drift), side=T("hud.stbd") if drift > 0
                                         else T("hud.port")), "ar_velocity", anchor="n", bold=False)
    return ops
