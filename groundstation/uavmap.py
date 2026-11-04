"""The UAVs on screen 2's map + their telemetry on this station (CLAUDE.md V "Telemetry / video / AI": "Map: position,
heading, altitude, track, battery, link quality, the picture as symbol"; step U4). DISPLAY ONLY - nothing here sends
anything to the boat or a drone.

  Store       the newest values of every drone (gw_status "uav" "drones" 1 Hz + uav_telem 2 Hz) + a track per drone
              (RAM, a point per >= 1 s once moved 3 m, at most TRACK_MAX points per drone)
  draw()      the owner's UAV picture (uavpics, turned with the drone's heading) at its position, the track in the UAV
              colour (mapview THEMES "uav", dashed - never mixed with the boat's track), a label "UAV 1 · 52 m ·
              78 % · LQ 100 %" (CAUTION colour with a low battery / the battery minimum / a weak link) - only drones
              that have a position and are not in their slot
  local_cautions()  the battery minimum's CAUTION 60 s / 20 s before + the landing (station-local alarms,
              alerts.MasterLamps.set_local: never a boat alarm), raised once per stage
"""
import collections
import math
import time

import theme
import uavpics
from i18n import T

TRACK_MAX = 3000
SYMBOL_PX = 32
LOW_BATT = 30
WEAK_LQ = 50
_rot = {}


def dist_m(a, b):
    kx = math.cos(math.radians((a[0] + b[0]) / 2.0)) * 111320.0
    return math.hypot((b[1] - a[1]) * kx, (b[0] - a[0]) * 111320.0)


class Store:
    def __init__(self):
        self.drones = {}                       # slot -> the newest record
        self.t = {}                            # slot -> when received
        self.tracks = collections.defaultdict(lambda: collections.deque(maxlen=TRACK_MAX))
        self.stage = {}                        # slot -> the last battery stage seen (local cautions)

    def update(self, drones, now=None):
        now = time.time() if now is None else now
        for k, d in (drones or {}).items():
            try:
                s = int(k)
            except (TypeError, ValueError):
                continue
            self.drones[s], self.t[s] = d, now
            if d.get("lat") is not None and d.get("lon") is not None and not d.get("in_slot"):
                tr = self.tracks[s]
                p = (d["lat"], d["lon"], d.get("alt"), now)
                if not tr or (now - tr[-1][3] >= 1.0 and dist_m(tr[-1][:2], p[:2]) >= 3.0):
                    tr.append(p)

    def shown(self, now=None, max_age=10.0):
        """The drones drawn on the map: a position, out of the slot, heard in the last max_age s."""
        now = time.time() if now is None else now
        return [d for s, d in sorted(self.drones.items()) if d.get("lat") is not None and not d.get("in_slot")
                and now - self.t.get(s, 0) <= max_age]

    def clear(self):
        self.drones.clear()
        self.t.clear()
        self.tracks.clear()
        self.stage.clear()

    def local_cautions(self):
        """-> [(code, text or None)] for MasterLamps.set_local: a NEW battery stage of a drone (60 s, 20 s, landing)
        raises its caution once; a drone that is down again clears it."""
        out = []
        for s, d in self.drones.items():
            stg = d.get("stage")
            if stg == self.stage.get(s):
                continue
            self.stage[s] = stg
            code = "uav_land_%d" % s
            if stg == "land":
                out.append((code, T("uav.land_now", n=s)))
            elif stg in (60, 20):
                out.append((code, T("uav.land_in", n=s, s=stg)))
            elif not d.get("armed"):
                out.append((code, None))
        return out


def label(d):
    parts = [T("uav.map_name", n=d["slot"])]
    if d.get("alt") is not None:
        parts.append("%.0f m" % d["alt"])
    if d.get("batt") is not None:
        parts.append("%d %%" % d["batt"])
    lq = d.get("lq")
    parts.append(T("uav.map_lq", lq=lq if lq is not None else "-"))
    if d.get("landing"):
        parts.append(T("uav.map_landing"))
    return " · ".join(parts)


def warn(d):
    """A caution colour: low battery, the battery minimum stage, a weak / lost link."""
    return bool(d.get("landing") or d.get("stage") or (d.get("batt") is not None and d["batt"] < LOW_BATT) or
                (d.get("lq") is not None and d["lq"] < WEAK_LQ))


def symbol(px, hdg, master):
    """The owner's picture turned by the heading (Pillow; cached by 10 deg), a Tk PhotoImage or None."""
    k = (px, int(round((hdg or 0.0) / 10.0)) % 36, id(master))
    if k in _rot:
        return _rot[k]
    im = uavpics.pil(px)
    if im is None:
        return None
    try:
        from PIL import ImageTk
        ph = ImageTk.PhotoImage(im.rotate(-k[1] * 10.0, expand=False), master=master)
    except Exception:
        return None
    _rot[k] = ph
    return ph


def draw(canvas, P, store, th, font, k=1.0, now=None):
    """Every shown drone: track, symbol, label (tag "uav"; the map tags it "map" afterwards). -> how many."""
    c = canvas
    col = th.get("uav", th["route"])
    n = 0
    for d in store.shown(now):
        s = d["slot"]
        tr = store.tracks.get(s)
        if tr and len(tr) >= 2:
            pts = [v for p in list(tr)[-2000:] for v in P(p[0], p[1])]
            c.create_line(*pts, fill=col, width=2, dash=(3, 3), tags=("uav", "uav_track"))
        x, y = P(d["lat"], d["lon"])
        px = max(16, int(SYMBOL_PX * k))
        im = symbol(px, d.get("hdg"), c)
        if im is not None:
            c.create_image(x, y, image=im, anchor="c", tags=("uav", "uav_sym", "uav_%d" % s))
        else:
            c.create_oval(x - 8, y - 8, x + 8, y + 8, outline=col, width=2, tags=("uav", "uav_sym", "uav_%d" % s))
        if d.get("hdg") is not None:
            h = math.radians(d["hdg"])
            L = px * 0.9
            c.create_line(x, y, x + L * math.sin(h), y - L * math.cos(h), fill=col, width=2, arrow="last",
                          tags=("uav",))
        c.create_text(x + px / 2 + 4, y - px / 2, text=label(d), anchor="sw", font=font,
                      fill=theme.CAUTION if warn(d) else col, tags=("uav", "uav_label"))
        n += 1
    return n


def hit(P, store, x, y, r=18, now=None):
    """The slot of the drone symbol at (x, y), or None (a click opens its video)."""
    best = None
    for d in store.shown(now):
        px, py = P(d["lat"], d["lon"])
        dd = math.hypot(px - x, py - y)
        if dd <= r and (best is None or dd < best[1]):
            best = (d["slot"], dd)
    return best[0] if best else None
