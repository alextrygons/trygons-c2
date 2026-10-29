"""The practical test area (ACADEMY_SPEC J, step 21): load, check, the instructor tool's model.

  * the area in git (academy/content/test_area.json) is a PLACEHOLDER (not surveyed): the practical test refuses
    it ("set the real area first"); the real one is written by the instructor tool into <academy data>/test_area.json
    (gitignored) with set_by / set_at / source and placeholder false - map taps, the boat's GPS ("add point" from
    the telemetry), a .geojson import; only a valid CAPTAIN with his 2FA login (accounts.Store.session_ok
    "examiner"); every change audit-logged (logbook kind "area");
  * it is NEVER a chart: no safety check uses it (not_for_navigation; the shallow-water / route / collision checks
    use only charts/ - test_academy AC11);
  * the limits: max_wind_kn, blocked_wind_from ("N" = from 315..45 deg, "ramp_right" = from the right side when
    looking down the ramp to the sea) from strong_wind_kn up, allowed_hours [from, to) local time.
"""
import copy
import datetime
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ACADEMY = os.path.dirname(HERE)
if ACADEMY not in sys.path:
    sys.path.append(ACADEMY)

import accounts                                                         # noqa: E402
import simguard                                                         # noqa: E402

CONTENT_AREA = os.path.join(ACADEMY, "content", "test_area.json")
FILE = "test_area.json"
MAX_CORNERS = 50


def area_path(store):
    return os.path.join(store.folder, FILE)


def load(store=None, path=None):
    """The real area from the academy data (if valid) else the placeholder from content/."""
    for p in ([path] if path else []) + ([area_path(store)] if store is not None else []) + [CONTENT_AREA]:
        try:
            with open(p, encoding="utf-8") as f:
                d = json.load(f)
            if isinstance(d, dict) and d.get("kind") == "test_area":
                return Area(d, p)
        except (OSError, ValueError):
            continue
    raise FileNotFoundError(CONTENT_AREA)


def dist_m(a, b):
    """Distance between [lat, lon] points (equirectangular: a bay)."""
    lat = math.radians(0.5 * (a[0] + b[0]))
    dy = (b[0] - a[0]) * 111320.0
    dx = (b[1] - a[1]) * 111320.0 * math.cos(lat)
    return math.hypot(dx, dy)


def bearing(a, b):
    lat = math.radians(0.5 * (a[0] + b[0]))
    dy = b[0] - a[0]
    dx = (b[1] - a[1]) * math.cos(lat)
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _pt(p):
    try:
        lat, lon = float(p[0]), float(p[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180) or lat != lat or lon != lon:
        return None
    return [lat, lon]


class Area(object):
    def __init__(self, doc, path=None):
        self.doc, self.path = doc, path
        self.polygon = [q for q in (_pt(p) for p in doc.get("polygon") or []) if q]
        self.ramp_top = _pt(doc.get("ramp_top"))
        self.ramp_bottom = _pt(doc.get("ramp_bottom"))
        self.placeholder = doc.get("placeholder") is not False
        self.max_wind_kn = float(doc.get("max_wind_kn", 12))
        self.strong_kn = float(doc.get("strong_wind_kn", 8))
        self.blocked = list(doc.get("blocked_wind_from") or [])
        h = doc.get("allowed_hours") or [8, 19]
        self.hours = (int(h[0]), int(h[1]))

    def problems(self):
        """Why the practical test may not use this area ([] = fine)."""
        p = []
        if self.placeholder:
            p.append("the test area is a PLACEHOLDER (not surveyed): set the real area first with the instructor tool")
        if self.doc.get("not_for_navigation") is not True:
            p.append("the test area must be marked not_for_navigation")
        if not 3 <= len(self.polygon) <= MAX_CORNERS:
            p.append("the area needs 3..%d corners" % MAX_CORNERS)
        if not self.ramp_top or not self.ramp_bottom:
            p.append("the ramp top / bottom points are not set")
        elif not self.inside(*self.ramp_bottom):
            p.append("the ramp's bottom point is outside the area")
        return p

    def inside(self, lat, lon):
        if len(self.polygon) < 3 or lat is None or lon is None:
            return False
        ins = False
        n = len(self.polygon)
        for i in range(n):
            (y1, x1), (y2, x2) = self.polygon[i], self.polygon[(i + 1) % n]
            if (y1 > lat) != (y2 > lat):
                x = x1 + (lat - y1) * (x2 - x1) / (y2 - y1)
                if lon < x:
                    ins = not ins
        return ins

    def ramp_bearing(self):
        """Looking DOWN the ramp to the sea: top -> bottom."""
        if not self.ramp_top or not self.ramp_bottom or self.ramp_top == self.ramp_bottom:
            return None
        return bearing(self.ramp_top, self.ramp_bottom)

    def wind_problems(self, kn, from_deg):
        p = []
        try:
            kn, fr = float(kn), float(from_deg) % 360.0
        except (TypeError, ValueError):
            return ["type the wind (kn + the direction it comes FROM, deg)"]
        if not 0 <= kn < 100:
            return ["wind %s kn?" % kn]
        if kn > self.max_wind_kn:
            p.append("wind %.0f kn > the limit %.0f kn" % (kn, self.max_wind_kn))
        if kn >= self.strong_kn:
            if "N" in self.blocked and (fr >= 315.0 or fr <= 45.0):
                p.append("strong north wind (%.0f kn from %.0f deg) - blocked" % (kn, fr))
            rb = self.ramp_bearing()
            if "ramp_right" in self.blocked and rb is not None and 45.0 <= (fr - rb) % 360.0 <= 135.0:
                p.append("strong wind from the right side at the ramp (%.0f kn from %.0f deg) - blocked" % (kn, fr))
        return p

    def hours_problem(self, when=None):
        when = when or datetime.datetime.now()
        if not self.hours[0] <= when.hour < self.hours[1]:
            return "outside the allowed hours %02d:00-%02d:00 (now %02d:%02d)" % (
                self.hours[0], self.hours[1], when.hour, when.minute)
        return None


# ---- the instructor tool's model -------------------------------------------------------------------------------
def geojson_polygon(path):
    """The first Polygon's outer ring of a .geojson file -> [[lat, lon], ...] (the closing point dropped)."""
    if os.path.splitext(path)[1].lower() not in (".geojson", ".json"):
        raise ValueError("a .geojson file")
    if not os.path.isfile(path):                    # (a device name / folder / URL is not a file)
        raise ValueError("not a file")
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    geoms = []
    if d.get("type") == "FeatureCollection":
        geoms = [ft.get("geometry") or {} for ft in d.get("features") or []]
    elif d.get("type") == "Feature":
        geoms = [d.get("geometry") or {}]
    else:
        geoms = [d]
    for g in geoms:
        if g.get("type") == "Polygon" and g.get("coordinates"):
            ring = g["coordinates"][0]
        elif g.get("type") == "MultiPolygon" and g.get("coordinates"):
            ring = g["coordinates"][0][0]
        else:
            continue
        pts = [_pt([c[1], c[0]]) for c in ring if isinstance(c, (list, tuple)) and len(c) >= 2]
        pts = [p for p in pts if p]
        if len(pts) > 1 and pts[0] == pts[-1]:
            pts = pts[:-1]
        return pts
    raise ValueError("no Polygon in the file")


class Draft(object):
    """The area being edited: corners (map taps / boat GPS / .geojson), the ramp points."""

    def __init__(self, area=None):
        a = area
        self.polygon = [list(p) for p in (a.polygon if a and not a.placeholder else [])]
        self.ramp_top = list(a.ramp_top) if a and not a.placeholder and a.ramp_top else None
        self.ramp_bottom = list(a.ramp_bottom) if a and not a.placeholder and a.ramp_bottom else None
        self.sources = set()

    def add(self, lat, lon, source="map"):
        p = _pt([lat, lon])
        if p is None:
            raise ValueError("not a position")
        if len(self.polygon) >= MAX_CORNERS:
            raise ValueError("at most %d corners" % MAX_CORNERS)
        self.polygon.append(p)
        self.sources.add(source)
        return p

    @staticmethod
    def gps_of(t):
        """The boat's own GPS position from the telemetry, or None (no fix / not trusted)."""
        if not t or not t.get("gps_ok") or not t.get("gps_fix"):
            return None
        return _pt([t.get("gps_lat"), t.get("gps_lon")])

    def add_gps(self, t):
        p = self.gps_of(t)
        if p is None:
            raise ValueError("no trusted GPS fix")
        return self.add(p[0], p[1], "boat GPS")

    def set_ramp(self, which, lat, lon, source="map"):
        p = _pt([lat, lon])
        if p is None or which not in ("top", "bottom"):
            raise ValueError("ramp point")
        setattr(self, "ramp_" + which, p)
        self.sources.add(source)

    def remove(self, i):
        del self.polygon[i]

    def load_geojson(self, path):
        self.polygon = geojson_polygon(path)[:MAX_CORNERS]
        self.sources.add("geojson " + os.path.basename(path))

    def doc(self, base, user, when=None):
        d = copy.deepcopy(base.doc if isinstance(base, Area) else base)
        d.update({"polygon": self.polygon, "ramp_top": self.ramp_top, "ramp_bottom": self.ramp_bottom,
                  "placeholder": False, "not_for_navigation": True, "set_by": user,
                  "set_at": (when or datetime.datetime.now()).strftime("%Y-%m-%dT%H:%M:%S"),
                  "source": sorted(self.sources) or ["map"]})
        d["_help"] = ("The REAL practical test area, set with the instructor tool (step 21) - NOT a chart: no safety "
                      "check uses it. " + str(d.get("_help", "")))[:2000]
        return d


def save(store, session, draft, base=None):
    """Writes <data>/test_area.json; a valid CAPTAIN's 2FA session only; audit-logged. -> the Area."""
    why = store.session_ok(session, role="examiner")
    if why:
        raise simguard.Refused(why)
    base = base or load(None, CONTENT_AREA)
    doc = draft.doc(base, session.user)
    a = Area(doc)
    probs = [p for p in a.problems()]
    if probs:
        raise ValueError("; ".join(probs))
    path = area_path(store)
    accounts._write_json(path, doc)
    store.log(session.user, "area", corners=len(a.polygon), ramp_top=a.ramp_top, ramp_bottom=a.ramp_bottom,
              source=doc["source"])
    return Area(doc, path)
