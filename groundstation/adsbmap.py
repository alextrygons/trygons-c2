"""Manned aircraft (ADS-B, receive only; CLAUDE.md X K4, step PF5) on screen 2's map + the receiver's state.
DISPLAY ONLY - nothing here sends anything to the boat, the gateway or a drone.

  notice_text(state, uav_in_air)   «ADS-B: χωρίς δέκτη» / «χωρίς δεδομένα» while a UAV is in the air and the receiver
                                    is not delivering (None when it works or no UAV flies): NEVER an empty, "clear" sky
  label(a)                         "HEL123 · 400 ft · 75 kn"
  draw(...)                        the aircraft (a plane symbol turned with the track, a dashed track, the label; grey
                                    when stale, CAUTION / WARNING colour for the ones that warn) + the notice
"""
import math

import theme
from i18n import T

PLANE = ((0, -10), (2.2, -3), (9, 2), (9, 4), (2, 2), (1.5, 8), (4, 10), (4, 11), (0, 10), (-4, 11), (-4, 10),
         (-1.5, 8), (-2, 2), (-9, 4), (-9, 2), (-2.2, -3))


def notice_text(st, uav_in_air):
    """The text of the 'no receiver / no data' notice, or None."""
    if not uav_in_air:
        return None
    state = (st or {}).get("state", "OFF")
    if state in ("OFF", "NO_RX"):
        return T("adsb.no_rx")
    if state == "NO_DATA":
        return T("adsb.no_data")
    return None


def label(a):
    name = a.get("cs") or a.get("icao") or "?"
    alt = a.get("alt_ft")
    parts = [name, ("%d ft" % round(alt)) if alt is not None else T("adsb.alt_unknown")]
    if a.get("gs_kn") is not None:
        parts.append("%d kn" % round(a["gs_kn"]))
    return " · ".join(parts)


def warn_level(st, icao):
    lv = 0
    for e in (st or {}).get("warn") or []:
        if e.get("icao") == icao:
            lv = max(lv, int(e.get("level") or 0))
    return lv


def draw(canvas, P, app, th, font, w, h, k=1.0):
    c = canvas
    st = getattr(app, "adsb_state", None) or {}
    col = th.get("adsb", th["route"])
    for a in st.get("targets") or []:
        if a.get("lat") is None:
            continue
        lv = warn_level(st, a.get("icao"))
        cc = theme.WARN if lv == 2 else theme.CAUTION if lv == 1 else ("#808080" if a.get("stale") else col)
        tr = a.get("track") or []
        if len(tr) >= 2:
            pts = [v for p in tr for v in P(p[0], p[1])]
            c.create_line(*pts, fill=cc, width=1, dash=(2, 4), tags=("adsb", "adsb_track"))
        x, y = P(a["lat"], a["lon"])
        t = math.radians(a.get("trk") or 0.0)
        sk = 0.8 * k
        pts = []
        for px, py in PLANE:
            pts += [x + sk * (px * math.cos(t) - py * math.sin(t)), y + sk * (px * math.sin(t) + py * math.cos(t))]
        c.create_polygon(*pts, fill=cc, outline="", tags=("adsb", "adsb_sym", "adsb_" + str(a.get("icao"))))
        c.create_text(x + 12 * sk, y - 10 * sk, text=label(a), anchor="sw", font=font, fill=cc, tags=("adsb", "adsb_label"))
    air = any(d.get("armed") and not d.get("landed") for d in (getattr(app.uav_store, "drones", {}) or {}).values())
    txt = notice_text(st, air)
    if txt:
        c.create_text(12, h - 12, text=txt, anchor="sw", font=font, fill=theme.CAUTION, tags=("adsb", "adsb_notice"))
