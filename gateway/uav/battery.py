"""The battery minimum = the ONLY landing the gateway ever asks for (CLAUDE.md V "NO FORCED LANDING except the
battery", docs/uav_design.md 6.1 / 6.2; step U4). Pure (no I/O): the router (router_worker.py, its OWN process)
calls judge() once a second for every flying drone and sends MAV_CMD_NAV_LAND ONLY when it says "land"; nothing
else in the gateway can ask for a landing (allow.py: NAV_LAND needs ctx battery_land, set by the router's own rule).

  reserve = the energy to descend from the CURRENT altitude at the landing rate (uav_land_ms) with the hover current
            (uav_hover_a, or the drone's own current draw when it reports a higher one) + uav_land_margin_pct
  left    = the pack capacity (BATT_CAPACITY read at the registration, else uav_capacity_mah) x the remaining %
  left <= reserve  -> LAND where it is (over the sea = the drone is lost: accepted by the owner)
  eta_s   = (left - reserve) / the current draw: the station shows a CAUTION 60 s and 20 s before (station-local)

The defaults are uavcheck.DEFAULTS (W59 / W71 / W75: TO CONFIRM with the drones' real numbers). The drone's own
BATT_FS_CRT_ACT Land stays the backstop when the gateway is gone.
"""
import math

from . import uavcheck

WARN_S = (60.0, 20.0)                     # the CAUTION stages on every station (docs 6.2)


def cfg_of(c=None):
    d = dict(uavcheck.DEFAULTS)
    d.update({k: v for k, v in (c or {}).items() if k in d})
    return d


def _num(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def judge(alt_m, pct, capacity_mah, current_a, cfg=None):
    """-> {"state": "unknown" | "ok" | "land", "left_mah", "reserve_mah", "eta_s", "why"}.
    unknown (no altitude / battery %): nothing is decided (the drone's own backstop stays)."""
    c = cfg_of(cfg)
    alt, pct, cur = _num(alt_m), _num(pct), _num(current_a)
    if alt is None or pct is None or pct < 0:
        return {"state": "unknown", "left_mah": None, "reserve_mah": None, "eta_s": None,
                "why": "no altitude / battery state"}
    cap = _num(capacity_mah) or float(c["uav_capacity_mah"])
    hover = float(c["uav_hover_a"])
    draw = max(hover, cur) if cur is not None and cur > 0 else hover
    t_down = max(0.0, alt) / max(0.1, float(c["uav_land_ms"]))
    reserve = draw * t_down / 3.6 * (1.0 + float(c["uav_land_margin_pct"]) / 100.0)
    left = cap * min(100.0, pct) / 100.0
    eta = (left - reserve) / (draw / 3.6)            # mAh / (mA per s = A / 3.6)
    if left <= reserve:
        return {"state": "land", "left_mah": round(left), "reserve_mah": round(reserve), "eta_s": 0.0,
                "why": "battery minimum: %.0f mAh left <= %.0f mAh to descend from %.0f m" % (left, reserve, alt)}
    return {"state": "ok", "left_mah": round(left), "reserve_mah": round(reserve), "eta_s": round(eta, 1),
            "why": ""}


def stage(eta_s, landing=False):
    """The CAUTION stage of a drone: "land" (landing now), 20, 60 or None."""
    if landing:
        return "land"
    if eta_s is None:
        return None
    for s in sorted(WARN_S):
        if eta_s <= s:
            return int(s)
    return None
