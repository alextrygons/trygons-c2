"""The UAV's own green HUD while this station flies it (CLAUDE.md V "Manual control", docs/uav_design.md 6.3, step U5;
DISPLAY ONLY). Screen 1 shows the UAV's video full screen with this HUD instead of the boat's instruments; the boat's
mode, MASTER lamps, alarms, notices and the bottom bar (SAVE ...) STAY (hud.UAV_KEEP).

  banner «JOYSTICK → UAV n» (top centre), altitude + climb (left), ground speed, battery + link (right), the heading,
  the distance + an ARROW to the boat (relative to the UAV's heading), the battery-minimum stage, the clip reasons
  (geofence / altitude: the gateway holds the drone at the edge), «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» (hit "uav_give").
Pure maths (bearing, arrow) for screen 2's camera view too (arrow_char).
"""
import math

import i18n
from i18n import T

ARROWS = "↑↗→↘↓↙←↖"


def bearing(a, b):
    """Bearing a -> b in degrees (lat, lon pairs)."""
    kx = math.cos(math.radians((a[0] + b[0]) / 2.0))
    return math.degrees(math.atan2((b[1] - a[1]) * kx, b[0] - a[0])) % 360.0


def to_boat(drone, boat_pos):
    """(distance m, bearing to the boat RELATIVE to the UAV's heading) or (None, None)."""
    if not boat_pos or boat_pos[0] is None or drone.get("lat") is None or drone.get("lon") is None:
        return None, None
    a, b = (drone["lat"], drone["lon"]), (boat_pos[0], boat_pos[1])
    kx = math.cos(math.radians((a[0] + b[0]) / 2.0)) * 111320.0
    dist = math.hypot((b[1] - a[1]) * kx, (b[0] - a[0]) * 111320.0)
    hdg = drone.get("hdg")
    rel = (bearing(a, b) - (hdg if hdg is not None else 0.0)) % 360.0
    return dist, rel


def arrow_char(rel):
    """An arrow for a relative bearing (0 = straight ahead = up)."""
    if rel is None:
        return "·"
    return ARROWS[int(((rel % 360.0) + 22.5) // 45) % 8]


def build(h, s):
    """Appends the UAV HUD to a hud.Hud (h) for the snapshot s (s.uav_fly = the slot, s.uav = its record,
    s.uav_boat = the boat's position, s.uav_clip = the gateway's clip reasons)."""
    n, u = s.uav_fly, s.uav or {}
    W, H = h.W, h.H
    g = h.pal["main"]
    big, norm, small = h.f_mode, h.f_norm, h.f_small
    # the banner: whose joystick is where
    bw, bh = W * 0.30, H * 0.050
    r = (W / 2 - bw / 2, H * 0.155, W / 2 + bw / 2, H * 0.155 + bh)
    h.panel(r, "uav_banner")
    h.rect(r, "uav_banner", outline=g, w=2)
    h.text((W / 2, (r[1] + r[3]) / 2), T("uavf.banner", n=n), "uav_banner", size=norm, bold=True, fill=g,
           anchor="c", max_w=bw - 8, box=r)
    # left: altitude + climb
    lx, ly = W * 0.10, H * 0.30
    alt = u.get("alt")
    h.text((lx, ly), T("uavh.alt"), "uav_alt", size=small, fill=g)
    h.text((lx, ly + small * 1.4), "%.0f m" % alt if alt is not None else "-- m", "uav_alt", size=big, bold=True,
           fill=g)
    cl = u.get("climb")
    h.text((lx, ly + small * 1.4 + big * 1.3), T("uavh.climb", v="%+.1f" % cl if cl is not None else "--"),
           "uav_alt", size=small, fill=g)
    # right: speed, battery, link
    rx, ry = W * 0.70, H * 0.30
    gs = u.get("gs")
    h.text((rx, ry), T("uavh.gs"), "uav_gs", size=small, fill=g)
    h.text((rx, ry + small * 1.4), "%.1f m/s" % gs if gs is not None else "-- m/s", "uav_gs", size=big, bold=True,
           fill=g)
    low = u.get("batt") is not None and u["batt"] < 30
    h.text((rx, ry + small * 1.4 + big * 1.3), T("uavh.batt", pct=u.get("batt") if u.get("batt") is not None
                                                   else "--"), "uav_batt", size=norm, bold=True,
           fill=h.pal["warn"] if low or u.get("stage") else g)
    h.text((rx, ry + small * 2.6 + big * 1.3 + norm), T("uavh.lq", lq=u.get("lq") if u.get("lq") is not None
                                                          else "--"), "uav_lq", size=small, fill=g)
    # the heading (top of the picture, under the banner)
    hdg = u.get("hdg")
    h.text((W / 2, H * 0.225), "%03.0f°" % hdg if hdg is not None else "---°", "uav_hdg", size=norm, bold=True,
           fill=g, anchor="c")
    # the distance + the arrow to the boat
    dist, rel = to_boat(u, s.uav_boat)
    cx, cy, rad = W / 2, H * 0.79, H * 0.045
    h.oval((cx - rad, cy - rad, cx + rad, cy + rad), "uav_boat", outline=g, w=2)
    if rel is not None:
        a = math.radians(rel)
        tip = (cx + rad * 0.9 * math.sin(a), cy - rad * 0.9 * math.cos(a))
        bl = (cx + rad * 0.5 * math.sin(a + 2.5), cy - rad * 0.5 * math.cos(a + 2.5))
        br = (cx + rad * 0.5 * math.sin(a - 2.5), cy - rad * 0.5 * math.cos(a - 2.5))
        h.poly([tip[0], tip[1], bl[0], bl[1], br[0], br[1]], "uav_boat", fill=g, outline=g)
    h.text((cx + rad * 1.4, cy), T("uavh.boat", m="%.0f" % dist if dist is not None else "--"), "uav_boat",
           size=norm, bold=True, fill=g, anchor="w")
    # the battery minimum + the gateway's clip reasons
    y = H * 0.66
    if u.get("stage"):
        h.text((W / 2, y), T("uav.land_now", n=n) if u["stage"] == "land" else T("uav.land_in", n=n, s=u["stage"]),
               "uav_stage", size=norm, bold=True, fill=h.pal["warn"], anchor="c", max_w=W * 0.5)
        y += norm * 1.5
    for why in (s.uav_clip or [])[:2]:
        h.text((W / 2, y), T("uavh.clip", why=i18n.from_en(why)), "uav_clip", size=small, bold=True, fill=g, anchor="c",
               max_w=W * 0.5)
        y += small * 1.5
    # «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ»: the joystick back to the boat (the UAV hovers)
    rb = h.R("drive")
    h.button(rb, T("uavf.give"), "uav_give", "uav_give")
