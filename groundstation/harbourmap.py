"""Harbour autonomy on screen 2's map (step HX6, CLAUDE.md Z; docs/harbour_design.md 8.1). DISPLAY ONLY - nothing here
sends anything (HU9). draw() paints, on the map canvas c with the map's projection P(lat, lon) -> (x, y):

  the route corridor        two thin lines (theme.HARB_CORRIDOR) at +-corr_m around the harbour route, the route dashed
  the fused tracks          colour by SOURCE (theme.HARB_SRC), shape by steer_ok (filled diamond = range-confirmed, hollow
                            circle = slow-only), the confidence in %, a short velocity line, static ones as a square outline
  the planned path          the planner's heading at its cap for its 40 s horizon (dashed), "STOP" when it holds
  the minimum-distance      the planner's hull-to-hull circles around the boat (8 m static / 15 m moving)
  the route being made      the teach-and-repeat preview (harbourpanel.py), CAUTION colour until it is sent
No red anywhere here (red = alarms only); NIGHT: the dim red of the night map.
"""
import math

import harbourview as hv
import settings
import theme
from i18n import T

KN_MS = 1852.0 / 3600.0


def _c(src, night):
    return theme.NIGHT["main"] if night else theme.HARB_SRC.get(src or "", theme.HARB_SRC["camera"])


def draw(c, P, app, th, font, w, h, k=1.0):
    now = _now()
    night = bool(settings.get("night"))
    corr_col = theme.NIGHT["dim"] if night else theme.HARB_CORRIDOR
    r = getattr(app, "harbour_route", None)
    pv = getattr(app, "harbour_preview", None)
    model = getattr(app, "harbour_model", None) or {}
    # the corridor + the route (the stored checked route, or the one being taught)
    for route, col, tag in ((r, corr_col, "harb_corr"), (pv, theme.NIGHT["main"] if night else theme.CAUTION, "harb_prev")):
        if not route:
            continue
        pts = [(x["lat"], x["lon"]) for x in route.get("wps") or []]
        if len(pts) >= 2:
            a = [v for p in pts for v in P(*p)]
            c.create_line(*a, fill=col, width=1, dash=(6, 4), tags=("harbour", tag))
            left, right = hv.corridor_lines(pts, float(route.get("corr_m") or model.get("corr_m") or 10.0))
            for side in (left, right):
                if len(side) >= 2:
                    c.create_line(*[v for p in side for v in P(*p)], fill=col, width=1, tags=("harbour", tag))
            if route is pv:
                x0, y0 = P(*pts[0])
                c.create_text(x0 + 8, y0 - 8, text=T("harb.preview_note"), anchor="sw", fill=col, font=font,
                              tags=("harbour", tag))
    d = getattr(app, "t", None) or {}
    boat = (d["est_lat"], d["est_lon"]) if d.get("pos_source") else None
    view = getattr(app, "harbour_view", None)
    # the fused tracks
    for t in hv.visible_tracks(view, now):
        s = hv.symbol(t)
        x, y = P(t["lat"], t["lon"])
        col = _c(s["src"], night)
        n = 7 * max(1.0, k)
        if s["shape"] == "diamond":
            c.create_polygon(x, y - n, x + n, y, x, y + n, x - n, y, outline=col, fill=col, width=2,
                             stipple="" if s["conf_pct"] >= 50 else "gray50", tags=("harbour", "harb_track"))
        else:
            c.create_oval(x - n, y - n, x + n, y + n, outline=col, fill="", width=2, tags=("harbour", "harb_track"))
        if s["static"]:
            c.create_rectangle(x - n - 3, y - n - 3, x + n + 3, y + n + 3, outline=col, width=1,
                               tags=("harbour", "harb_track"))
        sp = float(t.get("speed") or 0.0)
        if sp > 0.3 and not s["static"]:                       # a velocity line (10 s ahead)
            hd = math.radians(float(t.get("course") or 0.0))
            la2 = t["lat"] + math.cos(hd) * sp * 10.0 / 111320.0
            lo2 = t["lon"] + math.sin(hd) * sp * 10.0 / (111320.0 * max(0.2, math.cos(math.radians(t["lat"]))))
            x2, y2 = P(la2, lo2)
            c.create_line(x, y, x2, y2, fill=col, width=1, tags=("harbour", "harb_track"))
        c.create_text(x + n + 4, y - n, text=T("harb.trk_label", src=T("harb.src_" + (s["src"] or "none")),
                                              conf="%d" % s["conf_pct"], kind=T("harb.sym_" + ("steer" if s["steer_ok"] else "slow"))),
                      anchor="sw", fill=col, font=font, tags=("harbour", "harb_label"))
    # the planned path + the minimum-distance circles, only while the AURIX runs the harbour mode
    if boat and model.get("running"):
        path = hv.plan_path(boat[0], boat[1], model.get("heading"), model.get("plan_cap_kn"))
        col = theme.NIGHT["main"] if night else th["text"]
        if len(path) >= 2:
            c.create_line(*[v for p in path for v in P(*p)], fill=col, width=2, dash=(8, 4), arrow="last",
                          tags=("harbour", "harb_path"))
        elif model.get("stop"):
            x, y = P(*boat)
            c.create_text(x + 14, y + 14, text=T("harb.path_stop"), anchor="nw", fill=col, font=font,
                          tags=("harbour", "harb_path"))
        x, y = P(*boat)
        for rad, kind in hv.min_circles():
            rpx = abs(P(boat[0] + rad / 111320.0, boat[1])[1] - y)
            if rpx > 4:
                c.create_oval(x - rpx, y - rpx, x + rpx, y + rpx, outline=col, width=1, dash=(2, 4),
                              tags=("harbour", "harb_min"))


def _now():
    import time
    return time.time()
