"""Harbour autonomy - what the stations show (step HX6, CLAUDE.md Z; docs/harbour_design.md 8.1). PURE and READ-ONLY.

Decoded dicts in (the gateway's gw_status "harbour" + the 2.5 Hz "harbour_view" message), drawing data / text keys out.
Nothing here sends anything (no app, no gwclient, no proto writers, no tkinter): the buttons that send live in app.py
and go through the existing request senders (STANDBY / T_HARBOUR / REMOTE) - test_harbourui HU9 proves it.

Symbols (design 8.1): the COLOUR says WHICH sensor saw the object (theme.HARB_SRC: radar / LiDAR / camera / AIS);
the SHAPE says what the planner may do with it: a filled DIAMOND = range-confirmed (steer_ok: a turn may be planned
around it), a hollow CIRCLE = slow-only (camera / AIS only, or not confirmed yet: the boat only slows or stops) - so
the difference is never colour alone. The confidence is written next to it as a percentage.
"""
import math

SRC_LETTERS = (("R", "radar"), ("L", "lidar"), ("C", "camera"), ("A", "ais"))
SRC_KEYS = tuple(k for _l, k in SRC_LETTERS)
STATES = ("OFF", "RUN", "SLOW", "HOLD", "DONE")
RUNNING = ("RUN", "SLOW", "HOLD")
VIEW_MAX_AGE_S = 3.0               # a harbour_view older than this is not drawn (the tracks would be old)
PATH_HORIZON_S = 40.0              # the planner's horizon (design 7)
KN_MS = 1852.0 / 3600.0
M_DEG = 111320.0


def sources(track):
    """The sensors that saw the track recently: ["radar", ...] in a fixed order."""
    s = str((track or {}).get("src") or "")
    return [k for l, k in SRC_LETTERS if l in s]


def primary(track):
    """The source the symbol is coloured by: range sensors first (radar, LiDAR), then camera, then AIS; None = none."""
    s = sources(track)
    return s[0] if s else None


def symbol(track):
    """-> {"src", "shape": "diamond" | "circle", "filled", "steer_ok", "conf_pct", "static", "label"}."""
    t = track or {}
    steer = bool(t.get("steer_ok"))
    try:
        conf = max(0.0, min(1.0, float(t.get("conf") or 0.0)))
    except (TypeError, ValueError):
        conf = 0.0
    return {"src": primary(t), "shape": "diamond" if steer else "circle", "filled": steer, "steer_ok": steer,
            "conf_pct": int(round(conf * 100.0)), "static": bool(t.get("static")), "sources": sources(t),
            "cls": str(t.get("cls") or "unknown")}


def stat_name(plan):
    """The AURIX's harbour state as a name in STATES ("OFF" without a fresh status)."""
    p = plan or {}
    if not p.get("stat_fresh"):
        return "OFF"
    n = p.get("state_name")
    return n if n in STATES else "OFF"


def model(gw_harbour, view, now, lang="el"):
    """The status panel's data. gw_harbour = gw_status["harbour"] (or None), view = the newest harbour_view message
    (or None; received at `now`-relative time view["_rx"]). -> a dict with everything the screens print:
    enabled, state, running, cap_kn, corr_m, xte_m, why (the planner's one line in `lang`), unsure [...], degraded,
    giveway, entry_ok, route, sensors {radar, lidar: state}, grace_left_s, n_tracks, view_ok."""
    g = gw_harbour or {}
    enabled = bool(g.get("enabled"))
    v = view or {}
    fresh = bool(v) and (now - float(v.get("_rx", 0.0))) <= VIEW_MAX_AGE_S
    plan = (v.get("plan") if fresh else None) or {}
    pst = g.get("planner") or {}
    state = stat_name(plan) if plan else ("OFF" if not pst.get("stat_fresh") else
                                          (pst.get("stat") or {}).get("harb_state_name") or "OFF")
    if state not in STATES:
        state = "OFF"
    txt = plan.get("text_" + ("el" if lang == "el" else "en")) or pst.get("text_" + ("el" if lang == "el" else "en")) or ""
    why_key = plan.get("why_key") or pst.get("why")
    hf = int(plan.get("hflags") or 0)
    trk = (g.get("tracker") or {})
    sens = {k: ((g.get("sensors") or {}).get(k) or {}).get("state") for k in ("radar", "lidar")}
    return {"enabled": enabled, "state": state, "running": state in RUNNING, "view_ok": fresh,
            "cap_kn": plan.get("kn") if plan.get("kn") is not None else (pst.get("stat") or {}).get("harb_kn"),
            "corr_m": plan.get("corr_m") if plan.get("corr_m") is not None else (pst.get("stat") or {}).get("harb_corr_m"),
            "xte_m": plan.get("xte_m") if plan.get("xte_m") is not None else (pst.get("stat") or {}).get("harb_xte_m"),
            "why": txt, "why_key": why_key, "stop": bool(plan.get("stop")),
            "unsure": list(plan.get("unsure") or trk.get("unsure") or [])[:6], "degraded": bool(hf & 0x02),
            "giveway": bool(hf & 0x04), "camonly": bool(hf & 0x10), "heading": plan.get("heading"),
            "plan_cap_kn": plan.get("cap_kn"), "entry_ok": bool(plan.get("entry_ok")), "route": bool(plan.get("route")),
            "lease": bool(plan.get("lease")), "grace": bool(plan.get("grace")),
            "grace_left_s": plan.get("grace_left_s") or 0.0, "sensors": sens,
            "n_tracks": len(v.get("tracks") or []) if fresh else 0, "colregs": list(plan.get("colregs") or [])[:2],
            "max_kn": plan.get("max_kn")}


def visible_tracks(view, now):
    if not view or (now - float(view.get("_rx", 0.0))) > VIEW_MAX_AGE_S:
        return []
    return [t for t in (view.get("tracks") or []) if isinstance(t, dict) and t.get("lat") is not None]


# --- geometry ---------------------------------------------------------------------------------------------------
def _xy(lat0, lon0, lat, lon):
    return (lon - lon0) * M_DEG * math.cos(math.radians(lat0)), (lat - lat0) * M_DEG


def _ll(lat0, lon0, x, y):
    return lat0 + y / M_DEG, lon0 + x / (M_DEG * math.cos(math.radians(lat0)))


def corridor_lines(pts, half_m):
    """Both edges of the corridor around the polyline pts [(lat, lon)] at +-half_m: -> (left, right), each a list of
    (lat, lon) with one point per vertex (the bisector of the two legs, the offset scaled so the distance across a
    leg stays half_m, capped at 2 x half_m on a sharp corner). Fewer than 2 points -> ([], [])."""
    if len(pts) < 2 or not (half_m and half_m > 0):
        return [], []
    lat0, lon0 = pts[0]
    xy = [_xy(lat0, lon0, la, lo) for la, lo in pts]
    nrm = []
    for a, b in zip(xy, xy[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        nrm.append((-dy / n, dx / n) if n > 1e-6 else None)
    left, right = [], []
    for i, p in enumerate(xy):
        ns = [n for n in (nrm[i - 1] if i > 0 else None, nrm[i] if i < len(nrm) else None) if n is not None]
        if not ns:
            continue
        sx, sy = sum(n[0] for n in ns), sum(n[1] for n in ns)
        sn = math.hypot(sx, sy)
        if sn < 1e-6:
            sx, sy, sn = ns[0][0], ns[0][1], 1.0
        ux, uy = sx / sn, sy / sn
        k = min(2.0, 1.0 / max(1e-6, ux * ns[0][0] + uy * ns[0][1]))
        d = half_m * k
        left.append(_ll(lat0, lon0, p[0] + ux * d, p[1] + uy * d))
        right.append(_ll(lat0, lon0, p[0] - ux * d, p[1] - uy * d))
    return left, right


def plan_path(lat, lon, heading, cap_kn, horizon_s=PATH_HORIZON_S):
    """The planned path as a line: from the boat along the planner's heading for the horizon at its cap (the
    planner's own 40 s look-ahead; the exact lateral offset line lives in the planner). [] when there is no heading /
    cap / the cap is 0 (STOP: nothing planned)."""
    try:
        h, kn = float(heading), float(cap_kn)
    except (TypeError, ValueError):
        return []
    if not (math.isfinite(h) and math.isfinite(kn)) or kn <= 0.05 or lat is None or lon is None:
        return []
    d = kn * KN_MS * float(horizon_s)
    x, y = d * math.sin(math.radians(h)), d * math.cos(math.radians(h))
    return [(lat, lon), _ll(lat, lon, x, y)]


def min_circles(moving_m=15.0, static_m=8.0):
    """The planner's hull-to-hull minimum distances drawn around the boat (design 7): [(radius m, kind)]."""
    return [(float(static_m), "static"), (float(moving_m), "moving")]


# --- the buttons (the decision; app.py sends) ---------------------------------------------------------------------
def take_over_how(uncrewed):
    """What TAKE OVER means: CREWED = take the onboard joystick (nothing is sent - the stick wins by itself);
    UNCREWED = the existing REMOTE request from the HELM station."""
    return "remote" if uncrewed else "joystick"


def start_path(crew, single_op, running, entry_ok):
    """Who asks what for the START: -> "captain_yes" (the captain's own vertical YES, then the request), "single_op"
    (the existing single-operator question inside cmd()), "copilot" (the request goes to the captain's ACCEPT through
    the gateway), "refuse" (already running / not a station that may ask)."""
    if running:
        return "refuse"
    if single_op:
        return "single_op"
    if crew == "captain":
        return "captain_yes"
    if crew == "copilot":
        return "copilot"
    return "refuse"
