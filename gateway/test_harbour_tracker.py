"""Harbour perception (CLAUDE.md Z, docs/harbour_design.md 6 + 6a, step HX4): the fusion tracker + the static map.

    python test_harbour_tracker.py        (exit code 0 = all passed)

Every scenario drives the HX3 SIMULATED radar + LiDAR in-process (the same code the worker processes run, the same
lidar_filter), seeded = deterministic; camera obstacles and AIS targets are made from the world truth like the vision
manager / ais.py would give them.

  HT1  single target: one moving vessel -> one confirmed steer_ok track near the truth (position + velocity), class
  HT2  multiple targets + the default harbour: ferry (AIS-linked, mmsi), fisher, moored boats static, piers static
  HT3  crossing: two vessels crossing paths keep their own track ids (no swap)
  HT4  head-on + overtaking: the own boat moving, closing / slow targets tracked with the right velocity
  HT5  moored boats + piers -> the static map (LiDAR accumulation), the chart layer, save / load, decay
  HT6  kayak + swimmer without AIS, a buoy: small / person / buoy classes, steer_ok from LiDAR / radar alone; a flat
       paddleboard (surface) LiDAR-confirmed = steer_ok person-like; the attitude-compensated water band
  HT7  clutter rejection + ghosts: radar sea clutter never confirms; radar multipath ghost kept + ghost_suspect; LiDAR
       rain clutter -> few tentative tracks, never a deleted real object
  HT8  camera-only never steer_ok (+ camera_without_range conflict ahead); AIS-only never steer_ok (+ ais_without_range)
  HT9  sensor loss / degraded / stale / time skew -> explicit flags (U1 / U2 / U11), steer_ok drops when the range hits
       stop (coasting), the track kept until delete_s
  HT10 conflicting tracks: an AIS position 40 m off its range track near us = ais_range_disagree (U6)
  HT11 track lifetime: tentative -> confirmed -> coasting -> deleted; a tentative never-confirmed track is deleted
  HT12 timing / CPU: the default harbour, both sensors + camera + AIS: cycle p99 < 30 ms budget, overload flag
  HT13 determinism: the same seeds = byte-identical tracks; another seed differs
  HT14 read only: no tracker / staticmap import of aurix / stations / proto / a network module; a run with sockets
       trapped opens none; the tracker has no sender
  HT15 mutation checks: camera hits counted as range (RANGE_SRC + C) -> HT8 check fails; AIS counted -> fails;
       a lowered LiDAR range_ok_n 0 -> the coasting check fails
  HT16 the manager: HarbourSensors feeds the tracker (in-process batches), gw_status "harbour" "tracker" summary,
       flags, the attitude from TELEM2 att_roll / att_pitch, a tracker exception = flag "tracker error", never raised
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from harbour import sensors, simworld, tracker, staticmap  # noqa: E402
from harbour.simradar import RadarSim  # noqa: E402
from harbour.simlidar import LidarSim  # noqa: E402

results = []
LAT0, LON0 = 37.82, 23.60
T0 = 1_800_000_000.0


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def pose_xy(x, y, hdg, t):
    lat, lon = simworld.xy_to_ll(LAT0, LON0, x, y)
    return {"t": t, "lat": lat, "lon": lon, "heading": hdg % 360.0, "sog": 0.0, "cog": hdg % 360.0}


def vessel(oid, a, b, speed, ln=8.0, beam=3.0, h=1.5, rcs=0.7, refl=0.4, ais=None, kind="vessel", phase=0.0):
    return {"id": oid, "kind": kind, "path": [list(a), list(b)], "speed": speed, "len": ln, "beam": beam, "h": h,
            "rcs": rcs, "refl": refl, "ais": ais, "phase": phase}


class Sim:
    """The HX3 simulators in-process + the tracker, ticked at 5 Hz (radar 0.8 Hz, LiDAR 10 Hz)."""

    def __init__(self, objects, own=lambda t: (0.0, 0.0, 0.0), seed=1, faults=None, camera=False, ais_extra=None,
                 cfg=None, static=None, rcfg=None, lcfg=None, attitude=None, ais=True):
        self.w = simworld.World(objects, (LAT0, LON0), T0)
        rc = dict(sensors.DEFAULTS["radar"])
        rc.update(rcfg or {})
        lc = dict(sensors.DEFAULTS["lidar"])
        lc.update(lcfg or {})
        self.rc, self.lc = rc, lc
        self.radar = RadarSim(self.w, rc, seed=seed, tag=True)
        self.lidar = LidarSim(self.w, lc, seed=seed + 100, tag=True)
        self.own = own
        self.faults = faults or {"radar": {}, "lidar": {}}
        self.tr = tracker.Tracker(cfg, static_map=static)
        self.tr.set_origin(LAT0, LON0)
        self.camera, self.ais_on, self.ais_extra = camera, ais, ais_extra or []
        self.attitude = attitude
        self.health = {"radar": "OK", "lidar": "OK"}
        self.on = {"radar": True, "lidar": True}
        self.t = T0
        self.next = {"radar": T0, "lidar": T0}
        self.last_rx = {"radar": None, "lidar": None}
        self.skews = {}
        self.rng_seed = seed

    def pose(self, t):
        x, y, h = self.own(t - T0)
        return pose_xy(x, y, h, t)

    def step(self, dt=0.2):
        t1 = self.t + dt
        for kind, per in (("radar", 1.0 / self.rc["rate_hz"]), ("lidar", 1.0 / self.lc["rate_hz"])):
            while self.next[kind] <= t1:
                tm = self.next[kind]
                self.next[kind] += per
                if not self.on[kind]:
                    continue
                p = self.pose(tm)
                f = self.faults.get(kind, {})
                raw = self.radar.sweep(tm, p, f) if kind == "radar" else self.lidar.scan(tm, p, f)
                if raw is None:
                    continue
                if kind == "lidar":
                    raw = sensors.lidar_filter(raw, self.lc)
                b = {"ev": kind, "t": tm, "pose": p, "items": raw, "bad": 0, "capped": 0, "sim": True}
                t0 = time.perf_counter()
                self.tr.feed(kind, b, now=tm + 0.05)
                self.tr.note_ms((time.perf_counter() - t0) * 1000.0)
                self.last_rx[kind] = tm
        self.t = t1
        t0 = time.perf_counter()
        p = self.pose(t1)
        if self.camera:
            self.tr.camera(self.cam_obs(t1, p), p, t1)
        if self.ais_on:
            self.tr.ais(self.ais_list(t1) + [a(t1) if callable(a) else a for a in self.ais_extra], t1)
        ages = {k: (None if v is None else t1 - v) for k, v in self.last_rx.items()}
        f = self.tr.cycle(health=dict(self.health), ages=ages, skews=dict(self.skews), attitude=self.attitude)
        self.tr.note_ms((time.perf_counter() - t0) * 1000.0)
        return f

    def run(self, secs, dt=0.2):
        f = None
        for _ in range(int(round(secs / dt))):
            f = self.step(dt)
        return f

    def cam_obs(self, t, p):
        ox, oy, h = self.own(t - T0)
        out = []
        for o in self.w.truth(t):
            if o["kind"] == "pier" or (self.camera != "all" and o["id"] not in self.camera):
                continue
            dx, dy = o["x"] - ox, o["y"] - oy
            d = math.hypot(dx, dy)
            rel = (math.degrees(math.atan2(dx, dy)) - h + 540.0) % 360.0 - 180.0
            if d < 3 or d > 200 or abs(rel) > 40:
                continue
            out.append({"rel": rel, "dist": d * 1.1, "range": [d * 0.7, d * 1.5], "conf": 0.9, "confirmed": True})
        return out

    def ais_list(self, t):
        out = []
        for o in self.w.truth(t):
            if not o.get("ais"):
                continue
            lat, lon = simworld.xy_to_ll(LAT0, LON0, o["x"], o["y"])
            out.append({"mmsi": o["ais"], "lat": lat, "lon": lon, "sog": o["speed"] / 0.514444, "cog": o["hdg"],
                        "length": o["len"], "beam": o["beam"], "stale": False})
        return out

    def tracks(self):
        return self.tr.tracks_out()

    def nearest(self, x, y, pred=None):
        best = None
        for tr in self.tracks():
            if pred and not pred(tr):
                continue
            d = math.hypot(tr["x"] - x, tr["y"] - y)
            if best is None or d < best[0]:
                best = (d, tr)
        return best

    def truth(self, oid):
        return [o for o in self.w.truth(self.t) if o["id"] == oid][0]


def ht1():
    s = Sim([vessel("v", (60, -60), (60, 300), 2.0, ln=9.0)])
    s.run(20)
    v = s.truth("v")
    d, tr = s.nearest(v["x"], v["y"], lambda t: t["steer_ok"])
    conf = [t for t in s.tracks() if t["confirmed"]]
    ok = (d < 3.0 and abs(tr["vy"] - 2.0) < 0.5 and abs(tr["vx"]) < 0.5 and len(conf) == 1 and
          tr["cls"] in ("vessel",) and tr["src"] in ("R", "RL", "L") and tr["conf"] > 0.4)
    report("HT1 single target", ok, "err %.2f m, v (%.2f, %.2f), cls %s, src %s, conf %.2f, confirmed %d"
           % (d, tr["vx"], tr["vy"], tr["cls"], tr["src"], tr["conf"], len(conf)))


def ht2():
    sm = staticmap.StaticMap()                       # the chart's piers (as OpenSeaMap lines would give them)
    sm.add_segment(-160.0, 140.0, 70.0, 140.0, 8.0)
    sm.add_segment(-90.0, -10.0, -90.0, 120.0, 6.0)
    s = Sim(None, own=lambda t: (0.0, 0.0, 0.0), camera="all", static=sm)
    s.w = simworld.World(None, (LAT0, LON0), T0)
    s.radar.w = s.lidar.w = s.w
    s.run(40)
    out = []
    fer = s.truth("ferry")
    df, tf = s.nearest(fer["x"], fer["y"])
    out.append(("ferry mmsi", tf["mmsi"] == 237990001 and tf["steer_ok"] and df < 6.0 and tf["cls"] == "vessel"))
    fi = s.truth("fisher")
    dfi, tfi = s.nearest(fi["x"], fi["y"])
    out.append(("fisher", dfi < 4.0 and tfi["steer_ok"] and tfi["mmsi"] is None and tfi["cls"] in ("vessel", "small")))
    mo = [s.nearest(s.truth(m)["x"], s.truth(m)["y"]) for m in ("moored1", "moored2", "moored3")]
    out.append(("moored static", all(d < 4.0 and t["static"] and t["steer_ok"] and t["cls"] == "static"
                                     for d, t in mo)))
    qs = [t for t in s.tracks() if abs(t["x"] + 90.0) < 4.0 and 0 < t["y"] < 120 and t["steer_ok"]]
    out.append(("quay tracks static", len(qs) >= 3 and all(t["static"] and t["in_static_map"] for t in qs)))
    bad = [k for k, v in out if not v]
    report("HT2 multiple targets + the default harbour", not bad,
           "failed %s; ferry %.1f m mmsi %s, fisher %.1f m %s, moored %s, quay tracks %d, total %d"
           % (bad, df, tf["mmsi"], dfi, tfi["cls"], [(round(d, 1), t["cls"]) for d, t in mo], len(qs),
              len(s.tracks())))


def ht3():
    s = Sim([vessel("a", (-40, 20), (40, 20), 2.0), vessel("b", (0, -20), (0, 60), 2.0)],
            own=lambda t: (0.0, -30.0, 0.0))
    s.run(6)
    a0 = s.nearest(s.truth("a")["x"], s.truth("a")["y"], lambda t: t["steer_ok"])[1]["id"]
    b0 = s.nearest(s.truth("b")["x"], s.truth("b")["y"], lambda t: t["steer_ok"])[1]["id"]
    s.run(30)                    # they cross at (0, 20) near t = 20 s
    da, ta = s.nearest(s.truth("a")["x"], s.truth("a")["y"], lambda t: t["steer_ok"])
    db, tb = s.nearest(s.truth("b")["x"], s.truth("b")["y"], lambda t: t["steer_ok"])
    ok = a0 != b0 and ta["id"] == a0 and tb["id"] == b0 and da < 3 and db < 3 and abs(ta["vx"] - 2.0) < 0.6 and \
        abs(tb["vy"] - 2.0) < 0.6
    report("HT3 crossing keeps the ids", ok, "a %s -> %s (%.1f m), b %s -> %s (%.1f m)"
           % (a0, ta["id"], da, b0, tb["id"], db))


def ht4():
    # head-on: own boat north at 2 m/s, a vessel south at 2 m/s on a reciprocal line 15 m to starboard
    s = Sim([vessel("h", (15, 200), (15, -200), 2.0)], own=lambda t: (0.0, 2.0 * t, 0.0))
    s.run(20)
    h = s.truth("h")
    dh, th = s.nearest(h["x"], h["y"], lambda t: t["steer_ok"])
    ok1 = dh < 3 and abs(th["vy"] + 2.0) < 0.5
    # overtaking: own boat at 3 m/s behind a 1 m/s boat on the same line
    s2 = Sim([vessel("o", (0, 40), (0, 400), 1.0)], own=lambda t: (0.0, 3.0 * t, 0.0))
    s2.run(15)
    o = s2.truth("o")
    do, to = s2.nearest(o["x"], o["y"], lambda t: t["steer_ok"])
    ok2 = do < 3 and abs(to["vy"] - 1.0) < 0.5
    report("HT4 head-on + overtaking", ok1 and ok2, "head-on %.1f m vy %.2f; overtaken %.1f m vy %.2f (truth -2 / 1)"
           % (dh, th["vy"], do, to["vy"]))


def ht5():
    sm = staticmap.StaticMap()
    sm.add_segment(-90, -10, -90, 120, 6.0)
    objs = [o for o in simworld.default_objects() if o["kind"] in ("moored", "pier")]
    s = Sim(objs, own=lambda t: (-55.0, 40.0, 0.0), static=sm)
    s.run(40)
    m1 = s.truth("moored2")
    occ = sm.occupied(m1["x"], m1["y"], 1.0, s.t)
    quay_chart = sm.occupied(-90.0, 50.0)
    n = sm.n_cells(s.t)
    p = os.path.join(tempfile.gettempdir(), "ht5_static_%d.json" % os.getpid())
    sm.save(p)
    sm2 = staticmap.StaticMap.load(p)
    os.remove(p)
    same = sm2.occupied(m1["x"], m1["y"], 1.0, s.t) and sm2.n_cells(s.t) == n
    decayed = not sm.occupied(m1["x"], m1["y"], 0.0, s.t + 6 * 3600.0) or sm.occupied(-90.0, 50.0, 0.0,
                                                                                         s.t + 6 * 3600.0)
    decayed = decayed and not any(sm._w(c, s.t + 6 * 3600.0) >= sm.occ_w for c in sm.grid)
    tr_in = [t for t in s.tracks() if t["in_static_map"] and t["static"]]

    class Charts:                                  # a tiny chart: land west of x = -100
        def __bool__(self):
            return True

        def area_at(self, lat, lon):
            return self

        def depth(self, lat, lon):
            x, y = simworld.ll_to_xy(LAT0, LON0, lat, lon)
            return 0.0 if x < -100 else (None if x > 150 else 6.0)
    sm3 = staticmap.StaticMap()
    ncov = sm3.add_chart(Charts(), (LAT0, LON0), radius_m=200.0, step_m=4.0, limit_m=2.0)
    chart_ok = ncov > 0 and sm3.occupied(-150, 0) and not sm3.occupied(0, 0) and sm3.occupied(170, 0) and \
        sm3.has_data() and not staticmap.StaticMap().has_data()
    ok = occ and quay_chart and n["lidar"] > 20 and same and decayed and len(tr_in) >= 3 and chart_ok
    report("HT5 static map", ok, "moored2 occupied %s, quay chart %s, cells %s, save/load %s, decay %s, "
           "tracks in map %d, chart layer %s (samples %d)" % (occ, quay_chart, n, same, decayed, len(tr_in),
                                                             chart_ok, ncov))


def ht6():
    objs = [vessel("kayak", (25, 10), (25, 60), 1.0, ln=4.0, beam=0.7, h=0.4, rcs=0.15, refl=0.5, kind="kayak"),
            vessel("swimmer", (-14, 14), (-14, 18), 0.3, ln=0.6, beam=0.5, h=0.3, rcs=0.03, refl=0.15,
                   kind="swimmer"),
            {"id": "buoy", "kind": "buoy", "x": 0.0, "y": 32.0, "len": 1.0, "beam": 1.0, "hdg": 0.0, "h": 1.2,
             "rcs": 0.4, "refl": 0.6},
            {"id": "board", "kind": "moored", "x": 0.0, "y": -21.5, "len": 1.2, "beam": 0.6, "hdg": 0.0,
             "h": 0.12, "rcs": 0.05, "refl": 0.5}]
    s = Sim(objs, own=lambda t: (0.0, 0.0, 0.0))
    s.run(20)
    frac = {k: 0 for k in ("kayak", "swimmer", "buoy", "board")}
    for _ in range(50):                              # the share of the last 10 s each object is steer_ok
        s.step()
        for oid in frac:
            o = s.truth(oid)
            d, t = s.nearest(o["x"], o["y"])
            frac[oid] += 1 if (d < 2.0 and t["steer_ok"]) else 0
    got = {}
    for oid in ("kayak", "swimmer", "buoy", "board"):
        o = s.truth(oid)
        d, t = s.nearest(o["x"], o["y"])
        got[oid] = (round(d, 1), t["cls"], t["steer_ok"], t["src"], t["surface"])
    ok = (got["kayak"][0] < 2 and got["kayak"][1] == "small" and frac["kayak"] >= 45 and
          got["swimmer"][0] < 2 and got["swimmer"][1] == "person" and frac["swimmer"] >= 35 and
          got["buoy"][0] < 2 and got["buoy"][1] == "buoy" and frac["buoy"] >= 45 and
          got["board"][0] < 2 and got["board"][1] == "person" and frac["board"] >= 45 and got["board"][4])
    # the attitude-compensated water band: bow DOWN 3 deg = the water 20 m ahead appears ~1 m higher in the sensor
    # frame; the worker (no attitude) does not mark it surface, the tracker with the IMU does (a label only)
    tr = tracker.Tracker()
    w_ahead = -2.5 + 20.0 * math.sin(math.radians(3.0))
    o = {"x": 20.0, "y": 0.0, "z": w_ahead, "len_m": 0.5, "wid_m": 0.5, "hgt_m": 0.05, "n_points": 3}
    worker = sensors.lidar_filter([dict(o)], sensors.DEFAULTS["lidar"])[0]["surface"]
    tr.attitude = (0.0, -3.0)
    att = tr.surface_of(dict(o, surface=worker))
    tr.attitude = (0.0, 0.0)
    level = tr.surface_of(dict(o, surface=worker))
    tall = tr.surface_of(dict(o, hgt_m=0.5))
    ok2 = (not worker) and att and not level and not tall
    report("HT6 kayak / swimmer / buoy / flat board + attitude band", ok and ok2, "%s, steer_ok share of 50 ticks %s; band: worker %s, IMU bow down "
           "%s, level %s, tall %s" % (got, frac, worker, att, level, tall))


def ht7():
    # radar sea clutter only (no objects): never a confirmed track
    s = Sim([], rcfg={"clutter_n": 3.0}, seed=5)
    s.run(60)
    conf = [t for t in s.tracks() if t["confirmed"]]
    ok1 = not conf
    # a radar multipath ghost at twice the range of the strongest return (a big static ship 120 m ahead)
    ship = {"id": "ship", "kind": "moored", "x": 0.0, "y": 120.0, "len": 40.0, "beam": 10.0, "hdg": 0.0, "h": 8.0,
            "rcs": 1.0, "refl": 0.4}
    s2 = Sim([ship], faults={"radar": {"ghost": True}, "lidar": {}}, seed=6)
    s2.run(30)
    g = s2.nearest(0.0, 240.0)
    real = s2.nearest(0.0, 120.0)
    ok2 = g is not None and g[0] < 15 and g[1]["ghost_suspect"] and real[0] < 6 and real[1]["steer_ok"] and \
        not real[1]["ghost_suspect"] and g[1]["id"] in s2.tr.flags()["ghost_suspect"]
    # LiDAR rain clutter within 15 m + a real swimmer at 25 m: the swimmer stays steer_ok, the clutter rarely confirms
    sw = vessel("swimmer", (0, 25), (0, 26), 0.05, ln=0.6, beam=0.5, h=0.3, rcs=0.03, refl=0.3, kind="swimmer")
    s3 = Sim([sw], faults={"radar": {}, "lidar": {"clutter": True}}, seed=7)
    fl = s3.run(20)
    d3, t3 = s3.nearest(0.0, 25.0)
    ghosts = [t for t in s3.tracks() if t["confirmed"] and math.hypot(t["x"], t["y"]) < 16]
    ok3 = d3 < 2 and t3["steer_ok"] and fl["clutter"] and any("clutter" in u for u in fl["unsure"])
    report("HT7 clutter + ghosts", ok1 and ok2 and ok3,
           "sea clutter confirmed %d; radar ghost %s (kept %s, suspect %s), real ok %s; LiDAR rain: swimmer %.1f m "
           "steer_ok %s, clutter tracks confirmed %d (tracks %d), %s" % (
               len(conf), None if g is None else round(g[0], 1), g is not None, g and g[1]["ghost_suspect"],
               real[1]["steer_ok"], d3, t3["steer_ok"], len(ghosts), len(s3.tracks()), fl["unsure"]))


def camera_check(mod=tracker):
    """A camera obstacle with NO range return (a swimmer under the LiDAR field of view, inside the radar's blind
    zone): a camera-only track, never steer_ok, a camera_without_range conflict ahead."""
    sw = vessel("swimmer", (0, 12), (0, 13), 0.05, ln=0.6, beam=0.5, h=0.3, rcs=0.03, refl=0.15, kind="swimmer")
    s = Sim([sw], camera=["swimmer"])
    f = s.run(10)
    cam = [t for t in s.tracks() if "C" in t["src"]]
    ok = bool(cam) and not any(t["steer_ok"] for t in cam) and \
        any(c["kind"] == "camera_without_range" for c in f["conflicts"]) and any(u.startswith("U6") for u in f["unsure"])
    return ok, cam, f


def ais_check():
    ghost = {"mmsi": 999000111, "lat": simworld.xy_to_ll(LAT0, LON0, 20.0, 30.0)[0],
             "lon": simworld.xy_to_ll(LAT0, LON0, 20.0, 30.0)[1], "sog": 0.0, "cog": 0.0, "stale": False}
    s = Sim([], ais_extra=[ghost])
    f = s.run(10)
    a = [t for t in s.tracks() if t["mmsi"] == 999000111]
    ok = len(a) == 1 and not a[0]["steer_ok"] and a[0]["src"] == "A" and \
        any(c["kind"] == "ais_without_range" for c in f["conflicts"])
    return ok, a, f


def ht8():
    ok1, cam, f1 = camera_check()
    ok2, a, f2 = ais_check()
    report("HT8 camera-only / AIS-only never steer_ok", ok1 and ok2,
           "camera tracks %s conflicts %s; AIS-only %s conflicts %s" % (
               [(t["id"], t["steer_ok"], t["src"]) for t in cam], [c["kind"] for c in f1["conflicts"]],
               [(t["id"], t["steer_ok"], t["src"]) for t in a], [c["kind"] for c in f2["conflicts"]]))


def ht9():
    s = Sim([vessel("v", (30, 0), (30, 200), 1.0)])
    s.run(10)
    v0 = s.nearest(s.truth("v")["x"], s.truth("v")["y"])[1]
    f0 = s.tr.flags()
    s.on["lidar"] = False
    s.health["lidar"] = "LOST"
    f1 = s.run(1.0)
    s.health["radar"] = "DEGRADED"
    s.skews["radar"] = 0.4
    f2 = s.run(0.4)
    s.on["radar"] = False
    s.health["radar"] = "OK"                         # health not yet LOST: the data age says stale
    s.skews = {}
    f3 = s.run(3.0)
    tr3 = [t for t in s.tracks() if t["id"] == v0["id"]]
    s.health["radar"] = "LOST"
    f4 = s.run(5.0)
    tr4 = [t for t in s.tracks() if t["id"] == v0["id"]]
    ok = (v0["steer_ok"] and not f0["unsure"][:-1] and not f0["range_lost"] and
          f1["degraded"] == ["lidar"] and not f1["range_lost"] and any("U2 lidar" in u for u in f1["unsure"]) and
          "radar" in f2["time_skew"] and "radar" in f2["degraded"] and
          f3["range_lost"] and "radar" in f3["stale"] and tr3 and not tr3[0]["steer_ok"] and tr3[0]["coasting"] and
          f4["range_lost"] and not tr4)
    tq = tracker.Tracker()
    pq = pose_xy(0, 0, 0, T0)
    for k in range(12):
        tq.feed("lidar", {"t": T0 + 0.1 * k, "pose": dict(pq, t=T0 + 0.1 * k), "items": [
            {"x": 20.0, "y": 0.0, "z": -1.5, "len_m": 1.0, "wid_m": 1.0, "hgt_m": 1.0, "n_points": 30}]})
        tq.cycle(health={"lidar": "OK"}, ages={"lidar": 0.0}, now=T0 + 0.1 * k)
    q_ok = bool(tq.tracks) and tq.tracks[0].steer_ok
    tq.cycle(health={"lidar": "OK"}, ages={"lidar": 0.0}, now=T0 + 2.0)        # no data since, only the clock
    q_drop = not tq.tracks[0].steer_ok if tq.tracks else True
    tq.cycle(health={"lidar": "OK"}, ages={"lidar": 0.0}, now=T0 + 10.0)
    q_gone = not tq.tracks
    ok = ok and q_ok and q_drop and q_gone
    report("HT9 sensor loss / degraded / stale / skew flags", ok,
           "ok-state unsure %s; lidar lost %s; skew %s; radar silent: stale %s range_lost %s, track steer_ok %s "
           "coasting %s; after delete_s kept %s; clock only: steer_ok %s -> dropped %s -> deleted %s" % (
               f0["unsure"], f1["unsure"], f2["time_skew"], f3["stale"], f3["range_lost"],
               tr3 and tr3[0]["steer_ok"], tr3 and tr3[0]["coasting"], bool(tr4), q_ok, q_drop, q_gone))


def ht10():
    def off(t):                                      # the boat's AIS reports 40 m east of where it is
        x, y = 15.0 + 40.0, 10.0 + 1.0 * (t - T0)
        lat, lon = simworld.xy_to_ll(LAT0, LON0, x, y)
        return {"mmsi": 237000555, "lat": lat, "lon": lon, "sog": 1.0 / 0.514444, "cog": 0.0, "stale": False}
    # first linked correctly (AIS on the boat), then the AIS position drifts 40 m off
    s = Sim([vessel("v", (15, 10), (15, 400), 1.0, ais=237000555)])
    f0 = s.run(10)
    linked = [t for t in s.tracks() if t["mmsi"] == 237000555 and "R" in t["src"] + "L"]
    s.ais_on = False
    s.ais_extra = [off]
    s.ais_on = True
    s.w.objects[0]["ais"] = None
    f1 = s.run(3)
    kinds = [c["kind"] for c in f1["conflicts"]]
    ok = bool(linked) and not f0["conflicts"] and "ais_range_disagree" in kinds and \
        any(u.startswith("U6") for u in f1["unsure"])
    report("HT10 conflicting tracks", ok, "linked %s, before %s, after %s" % (
        [(t["id"], t["src"]) for t in linked], f0["conflicts"], f1["conflicts"]))


def ht11():
    s = Sim([vessel("v", (30, 0), (30, 300), 1.0)])
    seen = {"tentative": False, "confirmed": False}
    for _ in range(10):
        s.step()
        for t in s.tracks():
            if math.hypot(t["x"] - 30, t["y"] - s.truth("v")["y"]) < 4:
                seen["tentative" if not t["confirmed"] else "confirmed"] = True
    s.run(5)
    tid = s.nearest(s.truth("v")["x"], s.truth("v")["y"])[1]["id"]
    s.on["radar"] = s.on["lidar"] = False
    s.run(1.0)
    coast = [t for t in s.tracks() if t["id"] == tid]
    s.run(5.0)
    gone = not [t for t in s.tracks() if t["id"] == tid]
    # a tentative track that never confirms (one lucky clutter plot) is deleted
    tr = tracker.Tracker()
    p = pose_xy(0, 0, 0, T0)
    tr.feed("radar", {"t": T0, "pose": p, "items": [{"range_m": 80.0, "brg_deg": 45.0, "size_m": 1.0,
                                                     "size_cls": "small", "strength": 0.1}]})
    tr.cycle(health={"radar": "OK"}, ages={"radar": 0.0})
    born = len(tr.tracks) == 1 and not tr.tracks[0].confirmed
    for k in range(1, 10):
        tr.feed("radar", {"t": T0 + k, "pose": p, "items": []})
        tr.cycle(health={"radar": "OK"}, ages={"radar": 0.0})
    ok = seen["tentative"] and seen["confirmed"] and coast and coast[0]["coasting"] and not coast[0]["steer_ok"] \
        and gone and born and not tr.tracks
    report("HT11 track lifetime", ok, "seen %s, coasting %s, deleted %s, tentative born %s and deleted %s" % (
        seen, coast and coast[0]["coasting"], gone, born, not tr.tracks))


def ht12():
    s = Sim(None, camera="all")
    s.w = simworld.World(None, (LAT0, LON0), T0)
    s.radar.w = s.lidar.w = s.w
    s.own = lambda t: (40.0 * math.sin(t / 20.0), 40.0 * math.cos(t / 20.0) - 40.0, (math.degrees(t / 20.0) + 90.0))
    s.tr.timing = []
    t0 = time.process_time()
    s.run(60)
    cpu = time.process_time() - t0
    tm = sorted(s.tr.timing)
    p99 = tm[int(len(tm) * 0.99)]
    summ = s.tr.summary()
    budget = float(tracker.DEFAULTS["budget_ms"])
    # the overload flag: a slow cycle is reported
    s.tr.note_ms(budget * 2)
    f = s.step()
    ov = any("cycle" in w for w in f["overload"]) and any("overload" in u for u in f["unsure"])
    ok = p99 < budget and ov and summ["tracks"] > 5
    report("HT12 timing / CPU", ok, "per feed / cycle p50 %.2f ms p99 %.2f ms max %.2f ms (budget %.0f); CPU %.1f %% "
           "of one core for 60 s of harbour; tracks %d (steer_ok %d); overload flagged %s" % (
               tm[len(tm) // 2], p99, tm[-1], budget, 100.0 * cpu / 60.0, summ["tracks"], summ["steer_ok"], ov))


def ht13():
    def go(seed):
        s = Sim(None, seed=seed, camera="all")
        s.w = simworld.World(None, (LAT0, LON0), T0)
        s.radar.w = s.lidar.w = s.w
        s.radar.rng.seed(seed)
        s.lidar.rng.seed(seed + 100)
        s.run(15)
        return json.dumps(s.tracks(), sort_keys=True)
    a, b, c = go(3), go(3), go(4)
    report("HT13 determinism", a == b and a != c, "same seed identical %s, another seed differs %s (%d bytes)"
           % (a == b, a != c, len(a)))


def ht14():
    pkg = os.path.join(HERE, "harbour")
    banned = re.compile(r"^\s*(import|from)\s+(aurix|stations|proto|socket|asyncio|urllib|http|requests|"
                        r"websocket|uav|gateway|subprocess|threading)\b", re.M)
    bad = []
    for fn in ("tracker.py", "staticmap.py"):
        src = open(os.path.join(pkg, fn), encoding="utf-8").read()
        if banned.search(src) or re.search(r"T_GW_NAVREQ|navreq|\.send\(|\.aurix\b|alarm_bits|audit\(", src):
            bad.append(fn)
    no_send = not [n for n in dir(tracker.Tracker) if re.search(r"send|navreq|alarm|audit", n)]
    probe = os.path.join(tempfile.gettempdir(), "ht14_probe_%d.py" % os.getpid())
    with open(probe, "w", encoding="utf-8") as f:
        f.write("import socket, sys\n"
                "hits = []\n"
                "class T(socket.socket):\n"
                "    def __init__(self, *a, **k):\n"
                "        hits.append(1); raise OSError('blocked')\n"
                "socket.socket = T\n"
                "socket.create_connection = lambda *a, **k: (hits.append(1), (_ for _ in ()).throw(OSError('b')))\n"
                "sys.path.insert(0, %r)\n"
                "import test_harbour_tracker as t\n"
                "s = t.Sim(None, camera='all')\n"
                "s.run(5)\n"
                "bad = [m for m in ('aurix', 'stations', 'proto', 'gwclient') if m in sys.modules]\n"
                "print('HITS %%d BAD %%s TRACKS %%d' %% (len(hits), bad, len(s.tracks())))\n" % HERE)
    try:
        r = subprocess.run([sys.executable, probe], capture_output=True, timeout=120)
        line = r.stdout.decode().strip().splitlines()[-1:] if r.stdout else []
    finally:
        try:
            os.remove(probe)
        except OSError:
            pass
    run_ok = bool(line) and line[0].startswith("HITS 0 BAD []")
    report("HT14 read only", not bad and no_send and run_ok, "banned in %s, sender-like methods %s, trapped run %s"
           % (bad, not no_send, line))


def ht15():
    out = []
    saved = tracker.RANGE_SRC
    try:
        tracker.RANGE_SRC = ("R", "L", "C")
        ok, _, _ = camera_check()
        out.append(("camera counted as range -> the camera check FAILS", not ok))
        tracker.RANGE_SRC = ("R", "L", "A")
        ok, _, _ = ais_check()
        out.append(("AIS counted as range -> the AIS check FAILS", not ok))
    finally:
        tracker.RANGE_SRC = saved
    ok, _, _ = camera_check()
    out.append(("restored -> passes", ok))
    # coasting mutation: a track without range hits still steer_ok (fresh_s huge)
    s = Sim([vessel("v", (30, 0), (30, 300), 1.0)], cfg={"radar": {"fresh_s": 1e6, "window_s": 1e6},
                                                        "lidar": {"fresh_s": 1e6, "window_s": 1e6}})
    s.run(8)
    s.on["radar"] = s.on["lidar"] = False
    s.run(1.0)
    still = [t for t in s.tracks() if t["steer_ok"]]
    out.append(("no fresh-hit rule -> a coasting track stays steer_ok (the HT11 check would fail)", bool(still)))
    bad = [k for k, v in out if not v]
    report("HT15 mutation checks", not bad, "%s" % [(k, v) for k, v in out])


def ht16():
    from harbour.manager import HarbourSensors

    class Vis:
        obstacles = [{"rel": 0.0, "dist": 12.0, "range": [8, 20], "conf": 0.9, "confirmed": True}]

    class Ais:
        def list(self):
            return []

    class Gw:
        telem2 = {"att_roll": 1.5, "att_pitch": -2.0}
        vision = Vis()
        ais = Ais()
        telem_rx = time.time()

        def own(self):
            return {"fresh": True, "lat": LAT0, "lon": LON0, "heading": 0.0, "sog": 0.0, "cog": 0.0}
    gw = Gw()
    m = HarbourSensors({"sim": True}, gw)
    for sp in m.procs.values():                      # no worker processes: batches handed in as the reader would
        sp.c = dict(sp.c, enabled=True)
        sp.spawning = True
        sp.judge.spawned(time.monotonic())
    p = {"t": time.time(), "lat": LAT0, "lon": LON0, "heading": 0.0, "sog": 0.0, "cog": 0.0}
    for k in range(12):
        tt = time.time()
        b = {"ev": "lidar", "seq": k, "t": tt, "t_tx": tt + 0.05, "t_rx_epoch": tt + 0.06, "pose": dict(p, t=tt),
             "items": [{"x": 19.0, "y": 0.0, "z": -1.5, "len_m": 1.0, "wid_m": 1.0, "hgt_m": 1.0, "n_points": 30}],
             "bad": 0, "capped": 0, "sim": True}
        sp = m.procs["lidar"]
        sp.judge.batch(time.monotonic(), b)
        sp.pending.append(b)
        m.tick()
        time.sleep(0.1)
    st = m.status()
    tsum = st.get("tracker") or {}
    att = m.tracker.attitude
    fl = m.tracker_flags()
    trs = m.tracks()
    good = tsum.get("tracks", 0) >= 1 and tsum.get("steer_ok", 0) >= 1 and att == (1.5, -2.0) and \
        any("C" in t["src"] and "L" in t["src"] and t["steer_ok"] for t in trs) and "unsure" in fl
    # a tracker exception never stops the tick: flag "tracker error"
    orig = m.tracker.cycle

    def boom(*a, **k):
        raise RuntimeError("test")
    m.tracker.cycle = boom
    m.tick()
    m.tracker.cycle = orig
    m.tick()
    err = "tracker error" in m.tracker_flags()["unsure"]
    m.tick()
    report("HT16 the manager feeds the tracker", good and err, "summary %s, attitude %s, error flagged %s, unsure %s"
           % ({k: tsum.get(k) for k in ("tracks", "steer_ok", "cycle_ms")}, att, err, fl.get("unsure")))
    m.stop()


def main():
    for fn in (ht1, ht2, ht3, ht4, ht5, ht6, ht7, ht8, ht9, ht10, ht11, ht12, ht13, ht14, ht15, ht16):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "exception %r" % e)
    n = sum(results)
    print("%d / %d passed" % (n, len(results)))
    return 0 if n == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
