"""The vehicle choice for a route + the map's right-click menu (CLAUDE.md V "Routes", owner spec H2; step U3).
DISPLAY + requests only: the gateway checks every UAV route again (123 / 124), the boat's route goes through the
normal program upload and every AUTO rule.

  targets(uav)            the choices of the "Send route" target box: ΣΚΑΦΟΣ, ΕΠΟΜΕΝΟ UAV (θέση n), every
                          registered UAV in the magazine or flying (a flying one = its mission changed in flight)
  route_msg(target, pts)  {"t": "uav_route", "target": "next" | slot, "wps": [{"lat", "lon", "alt"}]}
  menu_items(uav, boat)   the right-click menu: «Πήγαινε εδώ» (the boat, a 2-point route from its position) and
                          «Αποστολή UAV εδώ» (the next UAV + every flying one)
  TargetBox               the vertical box with the pictures (the owner's UAV picture; the drawn USV symbol, W65)
"""
import theme
import uavpics
from i18n import T

DEFAULT_ALT_M = 50.0


def magazine_on(uav):
    return bool(uav and uav.get("enabled"))


def targets(uav):
    """[{"key": "boat" | "next" | slot, "kind": "usv" | "uav", "text"}] top first."""
    out = [{"key": "boat", "kind": "usv", "text": T("uavr.boat")}]
    if not magazine_on(uav):
        return out
    nxt = uav.get("next")
    if nxt:
        out.append({"key": "next", "kind": "uav", "text": T("uavr.next", n=nxt)})
    for s in uav.get("slots") or []:
        if not s.get("registered"):
            continue
        if s.get("flying"):
            out.append({"key": s["slot"], "kind": "uav", "text": T("uavr.flying", n=s["slot"],
                                                                   name=s.get("name") or "")})
        elif s.get("present") and s["slot"] != nxt:
            out.append({"key": s["slot"], "kind": "uav", "text": T("uavr.slot", n=s["slot"],
                                                                   name=s.get("name") or "")})
    return out


def route_msg(target, pts, alt=DEFAULT_ALT_M):
    """pts [(lat, lon)] or [{"lat", "lon", "alt"?}] -> the station message."""
    wps = []
    for p in pts:
        if isinstance(p, dict):
            wps.append({"lat": round(float(p["lat"]), 7), "lon": round(float(p["lon"]), 7),
                        "alt": float(p.get("alt") if p.get("alt") is not None else alt)})
        else:
            wps.append({"lat": round(float(p[0]), 7), "lon": round(float(p[1]), 7), "alt": float(alt)})
    return {"t": "uav_route", "target": target, "wps": wps}


def route_question(slot, wps, flying=True):
    """PF4-r2 LOW-6: the SAME route text as the gateway's question to the captain (gateway/uav/manager.route_question:
    points, length, last point, altitudes; English, translated with from_en) for the single operator's own YES."""
    import math
    km = 0.0
    for a, b in zip(wps, wps[1:]):
        dy = (b["lat"] - a["lat"]) * 111320.0
        dx = (b["lon"] - a["lon"]) * 111320.0 * math.cos(math.radians(a["lat"]))
        km += math.hypot(dx, dy) / 1000.0
    alts = [float(w.get("alt") or 0.0) for w in wps]
    last = wps[-1]
    alt = "%d" % round(alts[0]) if min(alts) == max(alts) else "%d-%d" % (round(min(alts)), round(max(alts)))
    tpl = ("UAV ROUTE: change the mission of the flying UAV %s (%d points, %.1f km, last point %.5f %.5f, %s m)" if
           flying else "UAV ROUTE: change the route of UAV %s in the launch sequence (%d points, %.1f km, last point "
           "%.5f %.5f, %s m)")
    return tpl % (slot, len(wps), km, last["lat"], last["lon"], alt)


def menu_items(uav, boat_pos):
    """[(action, target, text, kind)] for the right-click menu at a point."""
    out = []
    if boat_pos is not None:
        out.append(("go_here", "boat", T("uavr.go_here"), "usv"))
    if magazine_on(uav):
        for t in targets(uav)[1:]:
            if t["key"] == "next" or any(s.get("flying") and s["slot"] == t["key"] for s in uav.get("slots") or []):
                out.append(("uav_here", t["key"], T("uavr.uav_here", what=t["text"]), "uav"))
    return out


class TargetBox:
    """A small window: the choices one under the other (picture + text), CANCEL at the bottom."""

    def __init__(self, master, uav, on_choice):
        import tkinter as tk
        C = theme.C
        self.top = tk.Toplevel(master, bg=C["bg"])
        self.top.title(T("uavr.title"))
        try:
            self.top.transient(master)
        except tk.TclError:
            pass
        tk.Label(self.top, text=T("uavr.title"), font=theme.font("value"), bg=C["bg"], fg=C["fg"]).pack(
            padx=12, pady=(10, 6), anchor="w")
        self.pics = []
        self.items = targets(uav)
        for t in self.items:
            row = tk.Frame(self.top, bg=C["bg"])
            row.pack(fill="x", padx=12, pady=3)
            if t["kind"] == "uav":
                im = uavpics.photo(32, master=self.top)
                self.pics.append(im)
                tk.Label(row, image=im, bg=C["bg"]).pack(side="left") if im else None
            else:
                cv = tk.Canvas(row, width=32, height=32, bg=C["bg"], highlightthickness=0)
                uavpics.draw_usv(cv, 16, 16, 30)
                cv.pack(side="left")
            b = tk.Button(row, text=t["text"], anchor="w", command=lambda k=t["key"]: self.choose(k),
                          **theme.button_opts())
            b.pack(side="left", fill="x", expand=True, padx=(8, 0))
        tk.Button(self.top, text=T("uavr.cancel"), command=self.top.destroy, **theme.button_opts()).pack(
            fill="x", padx=12, pady=(8, 10))
        self.on_choice = on_choice

    def choose(self, key):
        try:
            self.top.destroy()
        finally:
            self.on_choice(key)
