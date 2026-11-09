"""Man overboard (CLAUDE.md X, docs/sar_design.md 2; step PF3, owner spec K2). PURE model: no Tk, no network.

DISPLAY + REQUESTS ONLY. A MOB press makes a marker, a WARNING on THIS station, a SAVE and a few OFFERS. Nothing here
steers the boat: the return route goes through the normal program upload + the AUTO rules (shallow / geofence checks, the
captain's ACCEPT for a copilot, the joystick CONFIRM at the boat), a UAV over the point through the normal uav_route +
launch rules. The final approach to a person is always by hand (sar_geo.standoff_route ends D metres before).

  capture(have, now)            the position of the PRESS: the newest telemetry sample's, or the last known one + the
                                uncertainty grown by the age x SOG; None = no position ever
  new_marker(...)               the marker record (docs/sar_design.md 1)
  info(marker, boat, now)       distance, bearing, elapsed for the MOB line
  drift_estimate(marker, now)   the ESTIMATE of where the person drifts (only from a trusted learned current)
  offers(marker, boat, uav, search_ok) -> the ordered list of offers: return, uav, search
"""
import calendar
import math
import time

import sar_geo
from i18n import T

MOB_KEY_DEFAULT = "m"
FRESH_S = 2.0                        # a telemetry sample older than this is an estimate (link trouble)
CONF_MIN = 0.5                       # the learned current is trusted from here (current_conf)
SIGMA_MIN_KN = 0.1                   # docs/sar_design.md 2 (6): sigma_v = max(0.1 kn, 0.5 |v_c|)
KN = 0.514444


def capture(have, now):
    """have = {"lat", "lon", "unc_m", "sog", "t"(rx time), "fresh"(the newest sample has a position)} or None.
    -> {"lat", "lon", "r_m", "uncertain", "age_s"} or None."""
    if not have or have.get("lat") is None or have.get("lon") is None:
        return None
    age = max(0.0, now - have["t"])
    unc = float(have.get("unc_m") or 0.0)
    sog = max(0.0, float(have.get("sog") or 0.0))
    if have.get("fresh") and age <= FRESH_S:
        return {"lat": have["lat"], "lon": have["lon"], "r_m": unc, "uncertain": False, "age_s": age}
    return {"lat": have["lat"], "lon": have["lon"], "r_m": unc + age * sog * KN, "uncertain": True, "age_s": age}


def current_of(d):
    """The learned current of a telemetry sample -> {"set_deg", "kn", "conf"} or None (not trusted)."""
    if not d:
        return None
    try:
        conf, kn, st = float(d.get("current_conf")), float(d.get("drift_kn")), float(d.get("set_deg"))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(conf) and math.isfinite(kn) and math.isfinite(st)) or conf < CONF_MIN:
        return None
    return {"set_deg": st % 360.0, "kn": kn, "conf": conf}


def new_marker(mid, n, pos, now, who, drift=None, station=None):
    return {"id": mid, "kind": "mob", "n": n, "name": "MOB %d" % n, "lat": pos["lat"], "lon": pos["lon"],
            "t_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "r_m": round(pos["r_m"], 1),
            "est": bool(pos["uncertain"]), "person": True, "src": {"station": station, "camera": None},
            "still": None, "closed": None, "drift": drift, "by": who}


def marker_id(now, n):
    return "mob_%s_%d" % (time.strftime("%Y%m%d-%H%M%S", time.gmtime(now)), n)


def t_of(marker):
    try:
        return float(calendar.timegm(time.strptime(marker["t_utc"], "%Y-%m-%dT%H:%M:%SZ")))
    except (KeyError, ValueError, TypeError):
        return None


def info(marker, boat, now):
    """-> {"dist_m", "brg", "elapsed_s"} (boat = (lat, lon) or None -> no distance)."""
    t0 = t_of(marker)
    out = {"dist_m": None, "brg": None, "elapsed_s": None if t0 is None else max(0.0, now - t0)}
    if boat is not None:
        out["dist_m"], out["brg"] = sar_geo.distance_bearing(boat, (marker["lat"], marker["lon"]))
    return out


def drift_estimate(marker, now):
    """The person's ESTIMATED place now: P0 + v_c t, uncertainty r = sqrt(r0^2 + (sigma_v t)^2). Only when the learned
    current was trusted at the press (marker["drift"] = {set_deg, kn, conf}); there is NO wind data on the boat, so
    leeway is 0 (said so on the screen). -> {"lat", "lon", "r_m", "set_deg", "kn"} or None."""
    dr = marker.get("drift")
    t0 = t_of(marker)
    if not dr or t0 is None:
        return None
    t = max(0.0, now - t0)
    v = float(dr["kn"]) * KN
    sig = max(SIGMA_MIN_KN, 0.5 * float(dr["kn"])) * KN
    r = math.hypot(float(marker.get("r_m") or 0.0), sig * t)
    la, lo = sar_geo.dest((marker["lat"], marker["lon"]), float(dr["set_deg"]), v * t)
    return {"lat": la, "lon": lo, "r_m": r, "set_deg": float(dr["set_deg"]), "kn": float(dr["kn"])}


def fmt_elapsed(s):
    s = int(max(0, s or 0))
    return "%02d:%02d" % (s // 60, s % 60)


def line(marker, boat, now):
    """The MOB line (screen 1's notices top line, screen 2's banner): «MOB 1 · 85 m · 045° · 02:10»."""
    i = info(marker, boat, now)
    if i["dist_m"] is None:
        base = T("mob.line_nopos", n=marker["n"], t=fmt_elapsed(i["elapsed_s"]))
    else:
        dist = "%d m" % round(i["dist_m"]) if i["dist_m"] < 1000 else "%.2f nm" % (i["dist_m"] / 1852.0)
        base = T("mob.line", n=marker["n"], d=dist, b="%03.0f" % i["brg"], t=fmt_elapsed(i["elapsed_s"]))
    if marker.get("est"):
        base += "  " + T("mob.est_pos")
    dr = marker.get("drift")
    base += "  " + (T("mob.drift", set="%03.0f" % dr["set_deg"], kn="%.1f" % dr["kn"]) if dr else T("mob.no_drift"))
    return base


def offers(marker, boat, uav, search_ok=False):
    """The offers for a MOB, in the order they are asked: ["return", "uav", "search"]. The return route is always
    offered (the station says why when it cannot be made); the UAV only when a magazine with a target exists; the
    search pattern only when PF2's panel exists (search_ok)."""
    out = ["return"]
    if uav and uav.get("enabled") and (uav.get("next") or any(s.get("flying") for s in uav.get("slots") or [])):
        out.append("uav")
    if search_ok:
        out.append("search")
    return out
