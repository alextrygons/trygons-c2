"""Mark a point from the video (CLAUDE.md X K3, docs/sar_design.md 5; step PF4). PURE model: no Tk, no network.

DISPLAY + REQUESTS ONLY. A click on a camera picture makes an ESTIMATED place on the sea (an uncertainty circle that
grows with the range). Nothing here steers the boat or a UAV: a route / mission from the marker goes through the normal
upload + checks + the captain's YES (app.py), the marker itself is only a place on the map and a row in the log.

The geometry is the inverse of the AR overlay's pinhole model (hudar.Cam.project): the camera looks `tilt_down_deg` below
the horizon, the boat's roll turns the picture, the pitch changes the look-down angle.

    locate(cam, roll, pitch, heading, pos, u, v, res, kind, cfg) -> {"lat", "lon", "r_m", "d_m", "brg", "alpha_deg",
                                                                       "sigma_d", "sigma_lat"}  or  {"why": key}
    locate_boat(cal, telem, u, v, res, age_s, cfg)  the boat's calibrated front camera (gateway vision.camera)
    locate_uav(drone, u, v, res, cfg)                 a UAV's fixed camera (gateway.json uav slots[].camera)
    new_marker(...)                                    the marker record (docs/sar_design.md 1), kind "point"
    still_box(u, v, w, h)                              the crop around the click (160 x 120)

Uncertainty (1 sigma; the circle drawn at 2 sigma):
    sigma_alpha^2 = sigma_att^2 (1 + x^2) + sigma_cal^2 + (sigma_pix / fy_px)^2   (rad; x = the click's sideways
                    offset in the picture, tan(azimuth): the roll error enters by it)
    sigma_d       = (h / sin^2 alpha) x sigma_alpha  (small sigma); exactly: half of h / tan(alpha - s) - h / tan(alpha + s)
    sigma_psi^2   = sigma_hdg^2 + sigma_cal^2 + sigma_att^2 y^2 + (sigma_pix / fx_px)^2   (PF4-r MED-4: y = the
                    click's vertical offset, tan of the elevation in the picture - the roll error turns it sideways)
    sigma_lat     = d x sigma_psi
    r_m           = circle_k x sqrt(sigma_d^2 + sigma_lat^2 + unc_pos^2)   (circle_k 4.0, from 2.45 = the 95 % radius of
                    a round 2-D normal, set so the MODEL holds ~95 % - test_sar MK4 - not measured on a real lens)
Freshness (PF4-r MED-3): the boat's attitude age = the telemetry sample's age at the station + its own att_age_ms;
the UAV's = the gateway's att_age + the age of the record at the station (rx_age): > att_max_age_s 1 s = refused.
The VIDEO's own delay (camera -> gateway -> station, typ. 0.2-0.5 s) is NOT known here: the attitude at the click is
used, not the attitude of the shown frame (stated in the manuals).
"""
import calendar
import math
import random
import time

import sar_geo

DEFAULTS = {
    "sigma_att_boat": 1.0, "sigma_att_uav": 2.0, "sigma_cal": 0.5, "sigma_hdg": 2.0, "sigma_pix": 1.5,
    "min_alpha_deg": 0.3, "max_range_boat": 1000.0, "max_range_uav": 1500.0, "max_rel_sigma": 0.35,
    "att_max_age_s": 1.0, "pos_max_age_s": 5.0, "circle_k": 4.0, "uav_pos_unc_m": 3.0, "uav_min_alt_m": 5.0,
    # W133 (TO CONFIRM): a 7" FPV camera on a fixed mount, 30 deg down, 120 x 90 deg
    "uav_camera": {"tilt_down_deg": 30.0, "hfov_deg": 120.0, "vfov_deg": 90.0},
}
STILL_W, STILL_H = 160, 120
STILL_MAX_BYTES = 40 * 1024        # PF4-r LOW: the same cap as the gateway (one station line is 64 KB)
TILT_MIN_DEG, TILT_MAX_DEG = -30.0, 90.0
MARK_KEY_DEFAULT = "k"
REPEAT_S = 1.0                      # a second mark within this is the same press (double click / key repeat)


def _fin(*vals):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in vals)


def cfg_of(over=None):
    c = dict(DEFAULTS)
    for k, v in (over or {}).items():
        if k in c and k != "uav_camera":
            try:
                f = float(v)
            except (TypeError, ValueError):
                continue
            if math.isfinite(f) and f > 0:
                c[k] = f
    return c


def inverse(cam, roll, pitch, u, v):
    """The picture point (u, v) -> (depression angle below the horizontal rad, azimuth off the camera axis rad), the
    exact inverse of hudar.Cam.project for a point on the water. cam: height_m, tilt_down_deg, hfov_deg, vfov_deg."""
    fx = 0.5 / math.tan(math.radians(cam["hfov_deg"]) / 2.0)
    fy = 0.5 / math.tan(math.radians(cam["vfov_deg"]) / 2.0)
    X, Y = (u - 0.5) / fx, (v - 0.5) / fy
    th = math.radians(roll or 0.0)                    # undo the heel (project rotates by -rho)
    Xr, Yr = X * math.cos(th) - Y * math.sin(th), X * math.sin(th) + Y * math.cos(th)
    look_down = math.radians(cam["tilt_down_deg"] - (pitch or 0.0))
    alpha = math.atan(Yr) + look_down
    return alpha, math.atan(Xr)


def locate(cam, roll, pitch, heading, pos, u, v, res, kind, cfg=None, unc_pos=0.0, yaw_cam=0.0):
    """-> a dict with the place + uncertainty, or {"why": key} (nothing made). kind "boat" | "uav"."""
    c = cfg or DEFAULTS
    if not _fin(u, v, roll, pitch, heading, unc_pos) or not 0.0 <= u <= 1.0 or not 0.0 <= v <= 1.0:
        return {"why": "bad"}
    if pos is None or not _fin(pos[0], pos[1]):
        return {"why": "nopos"}
    h = float(cam["height_m"])
    alpha, az = inverse(cam, roll, pitch, u, v)
    if alpha <= math.radians(c["min_alpha_deg"]):
        return {"why": "horizon"}
    d = h / math.tan(alpha)
    rmax = c["max_range_uav" if kind == "uav" else "max_range_boat"]
    if d > rmax:
        return {"why": "range", "d_m": d}
    w_px, h_px = max(1.0, float(res[0])), max(1.0, float(res[1]))
    fx = 0.5 / math.tan(math.radians(cam["hfov_deg"]) / 2.0)
    fy = 0.5 / math.tan(math.radians(cam["vfov_deg"]) / 2.0)
    s_att = c["sigma_att_uav" if kind == "uav" else "sigma_att_boat"]
    # the pitch error moves alpha 1:1, the roll error moves it by the sideways offset of the click (tan of the azimuth
    # in the picture) x the roll error - big in a wide field of view, so it is part of sigma_alpha
    x_off = abs((u - 0.5) / fx)
    s_alpha = math.sqrt(math.radians(s_att) ** 2 * (1.0 + x_off ** 2) + math.radians(c["sigma_cal"]) ** 2 +
                        (c["sigma_pix"] / (fy * h_px)) ** 2)
    # sigma_d: the half width of the +-1 sigma interval of alpha mapped through d = h / tan(alpha) (= h / sin^2 alpha x
    # sigma_alpha for a small sigma, but honest when sigma_alpha is a large part of alpha: the far side stretches)
    if alpha <= s_alpha:
        return {"why": "uncertain", "d_m": d}
    s_d = 0.5 * (h / math.tan(alpha - s_alpha) - h / math.tan(alpha + s_alpha))
    y_off = abs((v - 0.5) / fy)
    s_psi = math.sqrt(math.radians(c["sigma_hdg"]) ** 2 + math.radians(c["sigma_cal"]) ** 2 +
                      math.radians(s_att) ** 2 * y_off ** 2 + (c["sigma_pix"] / (fx * w_px)) ** 2)
    s_lat = d * s_psi
    if s_d / d > c["max_rel_sigma"]:
        return {"why": "uncertain", "d_m": d}
    r_m = float(c.get("circle_k", 2.45)) * math.sqrt(s_d ** 2 + s_lat ** 2 + float(unc_pos) ** 2)
    brg = (float(heading) + yaw_cam + math.degrees(az)) % 360.0
    la, lo = sar_geo.dest((pos[0], pos[1]), brg, d)
    return {"lat": la, "lon": lo, "r_m": r_m, "d_m": d, "brg": brg, "alpha_deg": math.degrees(alpha),
            "sigma_d": s_d, "sigma_lat": s_lat}


def locate_boat(cal, telem, u, v, res, age_s=0.0, cfg=None):
    """The boat's calibrated front camera. telem = the telemetry sample (est_lat / est_lon / pos_source / unc_m /
    heading / att_roll / att_pitch; the attitude is None when older than 1 s). cal = hudar.calibration()."""
    c = cfg or DEFAULTS
    if cal is None:
        return {"why": "nocal"}
    t = telem or {}
    if not t.get("pos_source") or t.get("est_lat") is None or t.get("est_lon") is None:
        return {"why": "nopos"}
    if age_s is None or age_s > c["pos_max_age_s"]:
        return {"why": "old"}
    if t.get("att_roll") is None or t.get("att_pitch") is None or t.get("heading") is None:
        return {"why": "noatt"}
    # PF4-r MED-3: the attitude's real age = the sample's age here + its own age in the sample
    att_ms = t.get("att_age_ms")
    if not _fin(att_ms) or att_ms < 0:           # PF4-r2 LOW-1: a missing / bad attitude age = refused (never "0 s")
        return {"why": "noatt"}
    att_age = float(att_ms) / 1000.0
    if not _fin(age_s) or float(age_s) + att_age > c["att_max_age_s"]:
        return {"why": "noatt"}
    cam = {"height_m": cal["height_m"], "tilt_down_deg": cal["tilt_down_deg"], "hfov_deg": cal["hfov_deg"],
           "vfov_deg": cal["vfov_deg"]}
    r = locate(cam, t["att_roll"], t["att_pitch"], t["heading"], (t["est_lat"], t["est_lon"]), u, v, res, "boat", c,
               unc_pos=float(t.get("unc_m") or 0.0), yaw_cam=cal.get("yaw_deg", 0.0))
    r["height_m"] = cal["height_m"]
    return r


def locate_uav(drone, u, v, res, cfg=None):
    """A UAV's fixed camera (altitude above the water, its ATTITUDE, its heading, the mount from gateway.json)."""
    c = cfg or DEFAULTS
    d = drone or {}
    if d.get("lat") is None or d.get("lon") is None:
        return {"why": "nopos"}
    alt = d.get("alt")
    if not _fin(alt) or alt < c["uav_min_alt_m"]:
        return {"why": "low"}
    rx = d.get("rx_age", 0.0)                                   # PF4-r MED-3: the record's age at this station
    if not _fin(rx) or rx < 0:
        return {"why": "old"}
    # PF4-r2 LOW-1: a missing altitude age = refused (never skipped: a lost link would keep the old height)
    if not _fin(d.get("alt_age")) or d["alt_age"] < 0 or d["alt_age"] + rx > c["att_max_age_s"]:
        return {"why": "old"}
    age = d.get("att_age")
    if d.get("roll") is None or d.get("pitch") is None or d.get("hdg") is None or not _fin(age) or \
            age + rx > c["att_max_age_s"]:
        return {"why": "noatt"}
    mount = d.get("camera") if isinstance(d.get("camera"), dict) else c["uav_camera"]
    cam = {"height_m": float(alt)}
    for k in ("tilt_down_deg", "hfov_deg", "vfov_deg"):
        try:
            val = float(mount.get(k, c["uav_camera"][k]))
        except (TypeError, ValueError):
            return {"why": "nocal"}
        if not math.isfinite(val) or (k != "tilt_down_deg" and not 5.0 <= val <= 170.0) or \
                (k == "tilt_down_deg" and not TILT_MIN_DEG <= val <= TILT_MAX_DEG):     # PF4-r LOW: range-checked
            return {"why": "nocal"}
        cam[k] = val
    r = locate(cam, d["roll"], d["pitch"], d["hdg"], (d["lat"], d["lon"]), u, v, res, "uav", c,
               unc_pos=c["uav_pos_unc_m"])
    r["height_m"] = float(alt)
    return r


def still_box(u, v, w, h):
    """The crop around the click: (x0, y0, x1, y1) in pixels, STILL_W x STILL_H, kept inside the picture."""
    w, h = max(1, int(w)), max(1, int(h))
    cw, ch = min(STILL_W, w), min(STILL_H, h)
    x0 = int(min(max(0, round(u * w - cw / 2.0)), w - cw))
    y0 = int(min(max(0, round(v * h - ch / 2.0)), h - ch))
    return x0, y0, x0 + cw, y0 + ch


def marker_id(now, n, station=None, rnd=None):
    r = rnd if rnd is not None else "%06x" % random.getrandbits(24)
    st = "".join(ch for ch in str(station if station is not None else "") if ch.isalnum())[:6]
    return "pt_%s_%d_%s%s" % (time.strftime("%Y%m%d-%H%M%S", time.gmtime(now)), n, st, r)


def new_marker(mid, n, loc, camera, u, v, now, who, station=None, name=""):
    return {"id": mid, "kind": "point", "n": n, "name": name or "POINT %d" % n, "lat": loc["lat"], "lon": loc["lon"],
            "t_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "r_m": round(loc["r_m"], 1), "est": True,
            "person": False, "src": {"station": station, "camera": camera, "u": round(u, 4), "v": round(v, 4),
                                     "range_m": round(loc["d_m"], 1), "brg": round(loc["brg"], 1)},
            "still": None, "closed": None, "drift": None, "by": who}


def t_of(marker):
    try:
        return float(calendar.timegm(time.strptime(marker["t_utc"], "%Y-%m-%dT%H:%M:%SZ")))
    except (KeyError, ValueError, TypeError):
        return None
