"""A scripted harbour world for the simulated radar + LiDAR (docs/harbour_design.md 5, step HX3).

Local metres, ENU (x east, y north) from the world origin (lat0, lon0); flat earth (a harbour is < 2 km). Every
object's position is a function of time only (deterministic: the radar, the LiDAR, the tests and HX4's tracker all see
the SAME truth). Kinds:
  pier     a segment (x1, y1) - (x2, y2), width w, height h (a quay / breakwater)
  moored   a box (x, y, len, beam, hdg) - a moored boat (static)
  vessel   a moving boat on a ping-pong path at speed mps; "ais": an MMSI or None (no AIS)
  kayak    a small moving object, no AIS
  swimmer  a person in the water (very small, very low), no AIS
  buoy     a static small object
Each object: rcs (radar strength factor 0..1), refl (LiDAR reflectivity 0..1), h (height above the water, m).

A world file (JSON, demo/demo_world.py writes one) = {"origin": [lat, lon] | null, "t0": epoch | null, "objects":
[...]}; origin null = the own boat's first position (the harbour is placed around wherever the boat starts).
"""
import json
import math

R_EARTH = 6371000.0


def ll_to_xy(lat0, lon0, lat, lon):
    return (math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * R_EARTH)


def xy_to_ll(lat0, lon0, x, y):
    return (lat0 + math.degrees(y / R_EARTH), lon0 + math.degrees(x / (R_EARTH * math.cos(math.radians(lat0)))))


def default_objects():
    """The built-in harbour (a small basin like Daskalio's): two piers, three moored boats, a vessel with AIS, a vessel
    without AIS, a kayak, a swimmer, a buoy. The own boat is expected near the origin."""
    return [
        {"id": "breakwater", "kind": "pier", "x1": -160.0, "y1": 140.0, "x2": 70.0, "y2": 140.0, "w": 8.0, "h": 2.5,
         "rcs": 1.0, "refl": 0.3},
        {"id": "quay", "kind": "pier", "x1": -90.0, "y1": -10.0, "x2": -90.0, "y2": 120.0, "w": 6.0, "h": 1.2,
         "rcs": 0.9, "refl": 0.25},
        {"id": "moored1", "kind": "moored", "x": -82.0, "y": 20.0, "len": 10.0, "beam": 3.5, "hdg": 0.0, "h": 2.0,
         "rcs": 0.7, "refl": 0.5},
        {"id": "moored2", "kind": "moored", "x": -82.0, "y": 45.0, "len": 12.0, "beam": 4.0, "hdg": 0.0, "h": 2.5,
         "rcs": 0.8, "refl": 0.5},
        {"id": "moored3", "kind": "moored", "x": -82.0, "y": 72.0, "len": 8.0, "beam": 3.0, "hdg": 180.0, "h": 1.8,
         "rcs": 0.6, "refl": 0.5},
        {"id": "ferry", "kind": "vessel", "path": [[120.0, -400.0], [120.0, 400.0]], "speed": 3.0, "len": 30.0,
         "beam": 8.0, "h": 6.0, "rcs": 1.0, "refl": 0.4, "ais": 237990001},
        {"id": "fisher", "kind": "vessel", "path": [[-40.0, 100.0], [60.0, -60.0]], "speed": 1.5, "len": 7.0,
         "beam": 2.5, "h": 1.5, "rcs": 0.5, "refl": 0.4, "ais": None},
        {"id": "kayak", "kind": "kayak", "path": [[30.0, 30.0], [-30.0, 80.0]], "speed": 1.0, "len": 4.0,
         "beam": 0.7, "h": 0.4, "rcs": 0.15, "refl": 0.5, "ais": None},
        {"id": "swimmer", "kind": "swimmer", "path": [[15.0, -25.0], [25.0, -15.0]], "speed": 0.4, "len": 0.6,
         "beam": 0.5, "h": 0.3, "rcs": 0.03, "refl": 0.15, "ais": None},
        {"id": "buoy", "kind": "buoy", "x": 40.0, "y": 60.0, "len": 1.0, "beam": 1.0, "hdg": 0.0, "h": 1.2,
         "rcs": 0.4, "refl": 0.6},
    ]


CLS = {"pier": "static", "moored": "static", "vessel": "vessel", "kayak": "small", "swimmer": "small", "buoy": "buoy"}


class World:
    def __init__(self, objects=None, origin=None, t0=None):
        self.objects = [dict(o) for o in (objects if objects is not None else default_objects())]
        self.origin = tuple(origin) if origin else None
        self.t0 = t0

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return cls(d.get("objects"), d.get("origin"), d.get("t0"))

    def to_json(self):
        return {"origin": list(self.origin) if self.origin else None, "t0": self.t0, "objects": self.objects}

    def place(self, lat, lon, t):
        """Fix the origin / time zero on the first pose (when the file left them open)."""
        if self.origin is None:
            self.origin = (lat, lon)
        if self.t0 is None:
            self.t0 = t

    def state(self, o, t):
        """(x, y, heading deg, speed mps) of an object at time t."""
        if "path" not in o:
            if o["kind"] == "pier":
                x = (o["x1"] + o["x2"]) / 2.0
                y = (o["y1"] + o["y2"]) / 2.0
                return x, y, math.degrees(math.atan2(o["x2"] - o["x1"], o["y2"] - o["y1"])) % 360.0, 0.0
            return o["x"], o["y"], float(o.get("hdg", 0.0)), 0.0
        (ax, ay), (bx, by) = o["path"][0], o["path"][1]
        L = math.hypot(bx - ax, by - ay)
        if L <= 0 or o["speed"] <= 0:
            return ax, ay, 0.0, 0.0
        s = (float(o["speed"]) * max(0.0, t - (self.t0 or 0.0)) + float(o.get("phase", 0.0))) % (2 * L)
        if s <= L:
            f, dx, dy = s / L, bx - ax, by - ay
        else:
            f, dx, dy = 2.0 - s / L, ax - bx, ay - by
        return ax + (bx - ax) * f, ay + (by - ay) * f, math.degrees(math.atan2(dx, dy)) % 360.0, float(o["speed"])

    def truth(self, t):
        """Every object at t: [{id, kind, cls, x, y, hdg, speed, len, beam, h, rcs, refl, ais, (pier geometry)}]."""
        out = []
        for o in self.objects:
            x, y, hdg, v = self.state(o, t)
            d = {"id": o["id"], "kind": o["kind"], "cls": CLS.get(o["kind"], "unknown"), "x": x, "y": y, "hdg": hdg,
                 "speed": v, "h": float(o.get("h", 1.0)), "rcs": float(o.get("rcs", 0.5)),
                 "refl": float(o.get("refl", 0.3)), "ais": o.get("ais")}
            if o["kind"] == "pier":
                d.update({"x1": o["x1"], "y1": o["y1"], "x2": o["x2"], "y2": o["y2"], "w": float(o.get("w", 5.0)),
                          "len": math.hypot(o["x2"] - o["x1"], o["y2"] - o["y1"]), "beam": float(o.get("w", 5.0))})
            else:
                d.update({"len": float(o.get("len", 1.0)), "beam": float(o.get("beam", 1.0))})
            out.append(d)
        return out


def pier_points(p, step):
    """Points along a pier every `step` metres (the radar / LiDAR see a quay as many returns)."""
    L = math.hypot(p["x2"] - p["x1"], p["y2"] - p["y1"])
    n = max(1, int(L // step))
    return [(p["x1"] + (p["x2"] - p["x1"]) * (i + 0.5) / n, p["y1"] + (p["y2"] - p["y1"]) * (i + 0.5) / n)
            for i in range(n)]


def own_xy(world, pose):
    return ll_to_xy(world.origin[0], world.origin[1], pose["lat"], pose["lon"])
