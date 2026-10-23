"""Camera geometry for the obstacle detection: where an obstacle in the
picture is (bearing) and roughly how far (distance from how far below the
horizon its waterline is). Pure Python, no numpy.

Coordinates: u, v = picture position 0..1 (u right, v down). Camera: height
above the water, how far it looks down (tilt), field of view, yaw (0 =
straight ahead). Horizon: from the IMU (roll / pitch of the boat) when the
AURIX sends it, else from the picture itself (water / sky boundary), else
the camera tilt alone.

DISTANCE IS ROUGH: one pixel or one degree of pitch more or less changes it a
lot, the more the further away. It is always given as a range.
"""
import math

DEFAULT_CAMERA = {"height_m": 1.6, "tilt_down_deg": 5.0, "hfov_deg": 80.0, "vfov_deg": 45.0, "yaw_deg": 0.0}
CAMERA_LIMITS = {"height_m": (0.2, 20.0), "tilt_down_deg": (-30.0, 60.0), "hfov_deg": (5.0, 170.0),
                 "vfov_deg": (5.0, 170.0), "yaw_deg": (-180.0, 180.0)}


def check_camera(cam):
    """A camera config that can't be used (missing, not a number, out of range,
    e.g. a field of view of 0 = division by zero) -> the default value for that
    entry. Returns (camera, [problems])."""
    out, bad = dict(DEFAULT_CAMERA), []
    for k, (lo, hi) in CAMERA_LIMITS.items():
        v = (cam or {}).get(k, DEFAULT_CAMERA[k])
        try:
            v = float(v)
        except (TypeError, ValueError):
            v = float("nan")
        if math.isfinite(v) and lo <= v <= hi:
            out[k] = v
        else:
            bad.append("%s=%r (allowed %g..%g, using %g)" % (k, (cam or {}).get(k), lo, hi, DEFAULT_CAMERA[k]))
    return out, bad


def horizon_from_imu(cam, roll_deg, pitch_deg):
    """The horizon line in the picture: (v at the centre, dv/du) in picture units.
    roll + = starboard side down, pitch + = bow up."""
    fy = 0.5 / math.tan(math.radians(cam["vfov_deg"]) / 2.0)
    fx = 0.5 / math.tan(math.radians(cam["hfov_deg"]) / 2.0)
    look_down = cam["tilt_down_deg"] - (pitch_deg or 0.0)           # bow up -> the camera looks up
    v0 = 0.5 - fy * math.tan(math.radians(look_down))
    # starboard down: the picture turns clockwise -> the horizon rises to the right
    slope = -math.tan(math.radians(roll_deg or 0.0)) * (fy / fx)
    return v0, slope


def horizon_default(cam):
    return horizon_from_imu(cam, 0.0, 0.0)


def locate(cam, u, v, horizon, pitch_unc_deg=1.0, pix_unc=1.5 / 96.0, max_range_m=1000.0):
    """An obstacle's waterline point (u, v) -> (bearing relative to the bow in deg,
    distance m or None = at / above the horizon (far), (d_min, d_max))."""
    fx = 0.5 / math.tan(math.radians(cam["hfov_deg"]) / 2.0)
    fy = 0.5 / math.tan(math.radians(cam["vfov_deg"]) / 2.0)
    rel = cam.get("yaw_deg", 0.0) + math.degrees(math.atan((u - 0.5) / fx))
    v0, slope = horizon
    below = v - (v0 + slope * (u - 0.5))                            # picture units below the horizon line

    def dist(dv, extra_deg=0.0):
        ang = math.atan(dv / fy) + math.radians(extra_deg)          # angle below the horizon
        if ang <= math.radians(0.05):
            return None
        return min(max_range_m, cam["height_m"] / math.tan(ang))
    d = dist(below)
    near = dist(below + pix_unc, pitch_unc_deg)
    far = dist(below - pix_unc, -pitch_unc_deg)
    if d is None:
        return rel, None, (near, None)
    return rel, d, (near if near is not None else d, far)          # far None = could be much further


def in_corridor(rel_deg, dist_m, sog_kn, yaw_rate_dps, cfg):
    """Is the obstacle inside the path ahead (current heading + turn rate)?
    A band of +-half_width_m around the curved path, length max(min_len_m, speed x look_s).
    A distance / bearing that is not a number counts as IN the path (unknown must
    never mean "no obstacle")."""
    if dist_m is None:
        return False
    if not (math.isfinite(dist_m) and math.isfinite(rel_deg)):
        return True
    x = dist_m * math.sin(math.radians(rel_deg))                    # right of the bow
    y = dist_m * math.cos(math.radians(rel_deg))                    # ahead
    v = max(sog_kn or 0.0, 1.0) * 0.514444
    length = max(cfg.get("min_len_m", 150.0), v * cfg.get("look_s", 60.0))
    if not (0.0 < y < length):
        return False
    k = math.radians(yaw_rate_dps or 0.0) / v                       # path curvature 1/m (+ = to starboard)
    xc = 0.5 * k * y * y
    return abs(x - xc) <= cfg.get("half_width_m", 25.0)


def bearing_abs(heading_deg, rel_deg):
    return ((heading_deg or 0.0) + rel_deg) % 360.0


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0
