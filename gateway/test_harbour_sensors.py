"""Harbour sensors (CLAUDE.md Z, docs/harbour_design.md 5 + 5a, step HX3): interface, simulated radar + LiDAR, health.

    python test_harbour_sensors.py        (exit code 0 = all passed)

  HS1  interface contract: defaults, pose / plot / object validation (NaN, out of range, bad class refused),
       check_config (config.example.json clean, bad values = errors, osdep.check_config carries them)
  HS2  simulated radar vs the world truth: range / bearing (relative to the bow) / speed / size class at known places
  HS3  simulated LiDAR vs the truth: sensor-frame positions, the low-object blind zone, the filter (hull / water / spray)
  HS4  noise + drop-out statistics: radar range / bearing sd, sweep drop rate, minimum / maximum range, swimmer Pd
  HS5  fault injection in the simulators: degraded (noise x3, fewer detections), clutter, ghost
  HS6  HealthJudge (pure): STARTING / OK / LOST / DEGRADED (self, rate, time skew, invalid items) / OFFLINE / OFF,
       back to OK only after 2 s good
  HS7  the worker PROCESSES through HarbourSensors: OK, lost / skew / degraded -> health + notices, clear -> OK,
       crash and hang = a notice + a restart, below-normal priority
  HS8  a real driver name = the skeleton: OFFLINE "not chosen (W148)", no data
  HS9  no AURIX / no network: no harbour module imports aurix / stations / proto / a network module or names a
       navreq; a worker run with sockets trapped opens none
  HS10 the world: deterministic ping-pong paths, file round trip, the origin from the first pose, the demo world file
"""
import json
import math
import os
import re
import statistics
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from harbour import sensors, simworld  # noqa: E402
from harbour.simradar import RadarSim  # noqa: E402
from harbour.simlidar import LidarSim  # noqa: E402
from harbour.manager import HarbourSensors  # noqa: E402

results = []
LAT0, LON0 = 37.82, 23.60


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def pose_at(x, y, hdg, t=1000.0, world=None):
    lat, lon = simworld.xy_to_ll(LAT0, LON0, x, y)
    return {"t": t, "lat": lat, "lon": lon, "heading": hdg, "sog": 0.0, "cog": 0.0}


def one(kind, **o):
    base = {"id": "o", "kind": kind, "x": 0.0, "y": 0.0, "len": 5.0, "beam": 2.0, "hdg": 0.0, "h": 1.5, "rcs": 0.9,
            "refl": 0.5}
    base.update(o)
    return simworld.World([base], (LAT0, LON0), 0.0)


def hs1():
    good_p = {"range_m": 100.0, "brg_deg": 10.0, "size_m": 5.0, "size_cls": "medium", "strength": 0.5,
              "speed_mps": 1.0, "course_deg": 90.0}
    bad = [dict(good_p, range_m=float("nan")), dict(good_p, brg_deg=400.0), dict(good_p, size_cls="huge"),
           dict(good_p, speed_mps=float("inf")), dict(good_p, strength=True), "x"]
    good_o = {"x": 1.0, "y": 2.0, "z": -1.0, "len_m": 1.0, "wid_m": 1.0, "hgt_m": 0.5, "n_points": 5}
    bad_o = [dict(good_o, x=float("nan")), dict(good_o, n_points=0), dict(good_o, n_points=2.5), dict(good_o, z=99)]
    m = sensors.merged({"sim": True})
    with open(os.path.join(HERE, "config.example.json"), encoding="utf-8") as f:
        ex = json.load(f)
    ex_errs = sensors.check_config(ex.get("harbour"))
    import osdep
    full = osdep.check_config({"aurix": "COM5", "baud": 921600, "storage": {"root": "D:\\gw"},
                               "harbour": {"radar": {"driver": "furuno", "rate_hz": -1}}}, "win32")
    checks = {
        "plot valid": sensors.valid_plot(good_p), "bad plots refused": not any(sensors.valid_plot(b) for b in bad),
        "object valid": sensors.valid_object(good_o),
        "bad objects refused": not any(sensors.valid_object(b) for b in bad_o),
        "pose valid": sensors.valid_pose(pose_at(0, 0, 10)),
        "pose NaN refused": not sensors.valid_pose(dict(pose_at(0, 0, 10), lat=float("nan"))),
        "sim = both sensors simulated": m["radar"]["enabled"] and m["lidar"]["enabled"] and
        m["radar"]["driver"] == "sim" and m["lidar"]["driver"] == "sim",
        "example has a harbour section": "harbour" in ex,
        "example clean": not [e for e in ex_errs if e[0] == "error"],
        "bad driver / rate = errors": len([e for e in sensors.check_config(
            {"radar": {"driver": "x", "rate_hz": 0}, "lidar": {"lost_s": float("nan")}}) if e[0] == "error"]) >= 3,
        "osdep carries harbour errors": any("harbour.radar" in t for lvl, t in full if lvl == "error"),
        "size classes": [sensors.size_cls(v) for v in (1, 5, 30)] == ["small", "medium", "large"],
        "sim over a real driver = warning": any("SIMULATOR" in t for lvl, t in sensors.check_config(
            {"sim": True, "radar": {"enabled": True, "driver": "mayara"}}) if lvl == "warning"),
    }
    report("HS1 interface contract", all(checks.values()), ", ".join(k for k, v in checks.items() if not v) or
           "%d checks" % len(checks))


def hs2():
    c = dict(sensors.DEFAULTS["radar"], drop_p=0.0, clutter_n=0.0)
    # a vessel 300 m north-east of the boat, moving east at 2 m/s; the boat heads 090
    w = simworld.World([{"id": "v", "kind": "vessel", "path": [[212.0, 212.0], [2212.0, 212.0]], "speed": 2.0,
                         "len": 20.0, "beam": 6.0, "h": 4.0, "rcs": 1.0, "refl": 0.4}], (LAT0, LON0), 0.0)
    rs = RadarSim(w, c, seed=3, tag=True)
    rng, brg, spd, cls = [], [], [], []
    for _ in range(300):
        for p in rs.sweep(0.0, pose_at(0, 0, 90.0)) or []:
            if p["sim_id"] == "v":
                rng.append(p["range_m"])
                brg.append(p["brg_deg"])
                spd.append(p["speed_mps"])
                cls.append(p["size_cls"])
    exp_r = math.hypot(212, 212)
    exp_b = (45.0 - 90.0) % 360.0
    mb = statistics.mean(brg)
    ok = (len(rng) > 250 and abs(statistics.mean(rng) - exp_r) < 1.0 and abs(mb - exp_b) < 0.3 and
          abs(statistics.mean(spd) - 2.0) < 0.1 and cls.count("large") > 0.6 * len(cls))
    report("HS2 radar vs truth", ok, "n %d range %.1f (%.1f) brg %.2f (%.1f, relative to the bow) speed %.2f" %
           (len(rng), statistics.mean(rng), exp_r, mb, exp_b, statistics.mean(spd)))


def hs3():
    c = dict(sensors.DEFAULTS["lidar"], drop_p=0.0, spray_n=3.0)
    # a kayak 20 m ahead, 5 m to starboard (boat heading 000 at the origin; sensor 1 m ahead of the hull middle)
    w = one("kayak", x=5.0, y=21.0, len=4.0, beam=0.7, h=0.6, refl=0.5)
    ls = LidarSim(w, c, seed=4, tag=True)
    xs, ys, raw_hull, raw_spray, raw_water, kept_bad = [], [], 0, 0, 0, 0
    for _ in range(200):
        raw = ls.scan(0.0, pose_at(0, 0, 0.0))
        raw_hull += sum(1 for o in raw if o["sim_id"] == "hull")
        raw_spray += sum(1 for o in raw if o["sim_id"] == "spray")
        raw_water += sum(1 for o in raw if o["sim_id"] == "water")
        for o in sensors.lidar_filter(raw, c):
            if o["sim_id"] in ("hull", "spray", "water"):
                kept_bad += 1
            if o["sim_id"] == "o":
                xs.append(o["x"])
                ys.append(o["y"])
    # the swimmer blind zone: low (0.3 m) object at 10 m never seen, at 30 m seen (the FOV's lower edge -7 deg)
    sw_near = LidarSim(one("swimmer", x=0.0, y=11.0, h=0.3, len=0.6, beam=0.5, refl=0.15), c, seed=5, tag=True)
    sw_far = LidarSim(one("swimmer", x=0.0, y=26.0, h=0.3, len=0.6, beam=0.5, refl=0.15), c, seed=5, tag=True)
    near = sum(1 for _ in range(200) for o in sw_near.scan(0.0, pose_at(0, 0, 0)) if o["sim_id"] == "o")
    far, far_f = 0, 0
    for _ in range(400):                # HX3 review H1: raw vs AFTER the worker's filter
        raw = sw_far.scan(0.0, pose_at(0, 0, 0))
        far += sum(1 for o in raw if o["sim_id"] == "o")
        far_f += sum(1 for o in sensors.lidar_filter(raw, c) if o["sim_id"] == "o")
    ky = LidarSim(one("kayak", x=0.0, y=21.0, len=4.0, beam=0.7, h=0.4, refl=0.5), c, seed=6, tag=True)
    k_raw = k_f = 0
    for _ in range(300):
        raw = ky.scan(0.0, pose_at(0, 0, 0))
        k_raw += sum(1 for o in raw if o["sim_id"] == "o")
        k_f += sum(1 for o in sensors.lidar_filter(raw, c) if o["sim_id"] == "o")
    ok = (len(xs) > 150 and abs(statistics.mean(xs) - 20.0) < 0.1 and abs(statistics.mean(ys) - 5.0) < 0.1 and
          raw_hull == 200 and raw_spray > 100 and kept_bad == 0 and near == 0 and far > 100 and
          far_f >= 0.95 * far and k_raw > 100 and k_f >= 0.95 * k_raw)
    report("HS3 LiDAR vs truth + filter + blind zone", ok,
           "n %d at x %.2f y %.2f (20, 5); raw hull %d spray %d water %d, kept after the filter %d; swimmer 10 m %d / "
           "25 m %d raw / %d after the filter of 400 scans; kayak 0.4 m %d / %d" % (
               len(xs), statistics.mean(xs) if xs else 0, statistics.mean(ys) if ys else 0, raw_hull, raw_spray,
               raw_water, kept_bad, near, far, far_f, k_raw, k_f))


def hs4():
    c = dict(sensors.DEFAULTS["radar"], clutter_n=0.0)
    w = one("vessel", x=0.0, y=500.0, len=15.0, rcs=1.0)
    rs = RadarSim(w, c, seed=6, tag=True)
    drops, rr, bb = 0, [], []
    for _ in range(3000):
        s = rs.sweep(0.0, pose_at(0, 0, 0))
        if s is None:
            drops += 1
            continue
        for p in s:
            rr.append(p["range_m"])
            bb.append((p["brg_deg"] + 180) % 360 - 180)
    sd_r_exp = c["range_sd_m"] + c["range_sd_frac"] * 500
    blind = RadarSim(one("vessel", x=0.0, y=15.0, rcs=1.0), dict(c, drop_p=0.0), seed=7)
    near = sum(len(blind.sweep(0.0, pose_at(0, 0, 0)) or []) for _ in range(100))
    far = RadarSim(one("vessel", x=0.0, y=3500.0, rcs=1.0), dict(c, drop_p=0.0), seed=7)
    beyond = sum(len(far.sweep(0.0, pose_at(0, 0, 0)) or []) for _ in range(100))
    lc = dict(sensors.DEFAULTS["lidar"], drop_p=0.0)
    sw = LidarSim(one("swimmer", x=0.0, y=26.0, h=0.3, len=0.6, beam=0.5, refl=0.15), lc, seed=8, tag=True)
    pd = sum(1 for _ in range(2000) for o in sensors.lidar_filter(sw.scan(0.0, pose_at(0, 0, 0)), lc)
             if o["sim_id"] == "o") / 2000.0                      # as the worker sends it (after the filter)
    drop_rate = drops / 3000.0
    ok = (abs(drop_rate - c["drop_p"]) < 0.01 and 0.8 < statistics.pstdev(rr) / sd_r_exp < 1.2 and
          0.8 < statistics.pstdev(bb) / c["brg_sd_deg"] < 1.2 and near == 0 and beyond == 0 and 0.45 < pd < 0.7)
    report("HS4 noise / drop-out statistics", ok,
           "drop %.3f (%.2f); range sd %.2f (%.2f); brg sd %.2f (%.1f); blind zone 15 m: %d plots; 3500 m: %d; "
           "swimmer LiDAR Pd %.2f (calm water ~0.6)" % (drop_rate, c["drop_p"], statistics.pstdev(rr), sd_r_exp,
                                                         statistics.pstdev(bb), c["brg_sd_deg"], near, beyond, pd))


def hs5():
    c = dict(sensors.DEFAULTS["radar"], drop_p=0.0, clutter_n=1.0)
    w = one("vessel", x=0.0, y=300.0, len=15.0, rcs=1.0)
    norm, degr = RadarSim(w, c, seed=9, tag=True), RadarSim(w, c, seed=9, tag=True)
    rn = [p["range_m"] for _ in range(500) for p in norm.sweep(0, pose_at(0, 0, 0)) if p["sim_id"] == "o"]
    rd = [p["range_m"] for _ in range(500) for p in degr.sweep(0, pose_at(0, 0, 0), {"degraded": True})
          if p["sim_id"] == "o"]
    cl = RadarSim(w, c, seed=10, tag=True)
    cn = sum(1 for _ in range(200) for p in cl.sweep(0, pose_at(0, 0, 0)) if p["sim_id"] == "clutter") / 200.0
    cf = sum(1 for _ in range(200) for p in cl.sweep(0, pose_at(0, 0, 0), {"clutter": True})
             if p["sim_id"] == "clutter") / 200.0
    gh = [p for p in RadarSim(w, dict(c, clutter_n=0.0), seed=11, tag=True).sweep(0, pose_at(0, 0, 0), {"ghost": True})
          if p["sim_id"] == "ghost"]
    lc = dict(sensors.DEFAULTS["lidar"], drop_p=0.0, spray_n=0.0)
    lw = one("moored", x=0.0, y=30.0, len=8.0, h=2.0)
    lg = [o for o in sensors.lidar_filter(LidarSim(lw, lc, tag=True).scan(0, pose_at(0, 0, 0), {"ghost": True}), lc)
          if o["sim_id"] == "ghost"]
    lcl = [o for o in sensors.lidar_filter(LidarSim(lw, lc, tag=True).scan(0, pose_at(0, 0, 0), {"clutter": True}), lc)
           if o["sim_id"] == "clutter"]
    ratio = statistics.pstdev(rd) / statistics.pstdev(rn)
    ok = (2.5 < ratio < 3.5 and len(rd) < 0.7 * len(rn) and cf > 15 * max(cn, 0.5) and len(gh) == 1 and
          abs(gh[0]["range_m"] - 600) < 30 and len(lg) == 1 and len(lcl) >= 15)
    report("HS5 fault injection (sim)", ok,
           "degraded sd x%.1f, detections %d -> %d; clutter %.1f -> %.1f per sweep; radar ghost %s; LiDAR ghost %d, "
           "rain clutter %d clusters (pass the filter = the tracker's problem)" %
           (ratio, len(rn), len(rd), cn, cf, [g["range_m"] for g in gh], len(lg), len(lcl)))


def hs6():
    c = dict(sensors.DEFAULTS["lidar"], enabled=True)
    j = sensors.HealthJudge("lidar", c)
    seen = {}

    def b(t, skew=0.0, bad=0, n=5, pose_dt=0.0, delay=0.0):
        tm = t - c["latency_s"] - skew
        return {"t": tm, "t_tx": t, "items": [{}] * n, "bad": bad, "pose": {"t": tm - pose_dt},
                "t_rx_epoch": t + delay}
    seen["start"] = j.judge(0.0)[0]
    j.spawned(0.0)
    seen["starting"] = j.judge(1.0)[0]
    seen["no data at start"] = j.judge(9.0)[0]
    j.spawned(10.0)
    t = 10.0
    for _ in range(40):
        t += 0.1
        j.batch(t, b(t))
    seen["ok"] = j.judge(t)[0]
    seen["lost"] = j.judge(t + 1.5)[0]
    for _ in range(40):
        t += 0.1
        j.batch(t, b(t))
    seen["held after lost"] = j.judge(t)[0]
    for _ in range(25):
        t += 0.1
        j.batch(t, b(t))
    seen["ok after 2 s"] = j.judge(t)[0]
    j.batch(t + 0.1, b(t + 0.1, skew=0.3))
    seen["skew"] = j.judge(t + 0.1)
    j.batch(t + 0.2, b(t + 0.2, bad=5, n=5))
    seen["bad items"] = j.judge(t + 0.2)
    t += 0.2
    for _ in range(30):                 # half the rate
        t += 0.3
        j.batch(t, b(t))
    seen["rate"] = j.judge(t)
    t += 0.1
    j.batch(t, b(t, pose_dt=0.8))
    seen["pose old"] = j.judge(t)
    j.batch(t, b(t, delay=0.5))
    seen["delay"] = j.judge(t)
    jf = sensors.HealthJudge("lidar", c)
    jf.spawn_fail = "No such file"
    seen["cannot start"] = jf.judge(0.0)
    j.status_line({"self": "degraded", "why": "sensor says"})
    seen["self"] = j.judge(t)
    j.status_line({"self": "offline", "why": "driver x"})
    seen["offline"] = j.judge(t)[0]
    seen["off"] = sensors.HealthJudge("radar", dict(sensors.DEFAULTS["radar"])).judge(0)[0]
    exp = {"start": "STARTING", "starting": "STARTING", "no data at start": "LOST", "ok": "OK", "lost": "LOST",
           "held after lost": "LOST", "ok after 2 s": "OK", "offline": "OFFLINE", "off": "OFF"}
    ok = all(seen[k] == v for k, v in exp.items())
    ok = ok and seen["skew"][0] == "DEGRADED" and "skew" in seen["skew"][1]
    ok = ok and seen["bad items"][0] == "DEGRADED" and "invalid" in seen["bad items"][1]
    ok = ok and seen["rate"][0] == "DEGRADED" and "rate" in seen["rate"][1]
    ok = ok and seen["self"][0] == "DEGRADED" and "sensor says" in seen["self"][1]
    ok = ok and seen["pose old"][0] == "DEGRADED" and "pose" in seen["pose old"][1]
    ok = ok and seen["delay"][0] == "DEGRADED" and "delivery" in seen["delay"][1]
    ok = ok and seen["cannot start"][0] == "LOST" and "cannot start" in seen["cannot start"][1]
    report("HS6 health judgement", ok, json.dumps({k: v for k, v in seen.items()}, default=str)[:400])


class FakeGw:
    def __init__(self):
        self.fresh = True

    def own(self):
        if not self.fresh:
            return {"lat": None, "fresh": False}
        return {"fresh": True, "lat": LAT0, "lon": LON0, "heading": 30.0, "sog": 1.0, "cog": 30.0}


def run_until(m, cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.tick()
        if cond():
            return time.time() - t0
        time.sleep(0.1)
    return None


def win_priority(pid):
    if os.name != "nt" or not pid:
        return None
    import ctypes
    k = ctypes.WinDLL("kernel32")
    h = k.OpenProcess(0x1000, False, pid)
    if not h:
        return None
    try:
        return k.GetPriorityClass(h)
    finally:
        k.CloseHandle(h)


def hs7():
    gw = FakeGw()
    m = HarbourSensors({"sim": True, "restart_s": 1.0, "silent_s": 3.0,
                        "radar": {"rate_hz": 2.0, "lost_s": 2.0}, "lidar": {"lost_s": 1.0}}, gw)
    got = {}
    try:
        both_ok = lambda: m.health("radar")[0] == "OK" and m.health("lidar")[0] == "OK"   # noqa: E731
        got["ok"] = run_until(m, both_ok, 20)
        time.sleep(0.5)
        got["prio"] = win_priority(m.pids()["radar"])
        m.fault("lidar", "lost")
        got["lost"] = run_until(m, lambda: m.health("lidar")[0] == "LOST", 6)
        m.fault("lidar", "clear")
        got["back"] = run_until(m, lambda: m.health("lidar")[0] == "OK", 8)
        m.fault("radar", "skew")
        got["skew"] = run_until(m, lambda: m.health("radar")[0] == "DEGRADED" and "skew" in m.health("radar")[1], 8)
        m.fault("radar", "clear")
        run_until(m, lambda: m.health("radar")[0] == "OK", 8)
        m.fault("lidar", "degraded")
        got["degraded"] = run_until(m, lambda: m.health("lidar")[0] == "DEGRADED", 6)
        m.fault("lidar", "clear")
        run_until(m, lambda: m.health("lidar")[0] == "OK", 8)
        p0 = m.pids()["radar"]
        m.fault("radar", "crash")
        got["crash"] = run_until(m, lambda: m.procs["radar"].restarts >= 1 and m.health("radar")[0] == "OK", 20)
        got["new pid"] = m.pids()["radar"] not in (None, p0)
        m.fault("lidar", "freeze")
        got["hang"] = run_until(m, lambda: m.procs["lidar"].restarts >= 1 and m.health("lidar")[0] == "OK", 20)
        gw.fresh = False
        got["no pose = lost"] = run_until(m, lambda: m.health("lidar")[0] == "LOST", 8)
        st = m.status()
        texts = [n["text"] for n in st["notices"]] + [n["text"] for n in list(m.notices)]
        got["notices"] = (any("CRASHED" in x for x in texts) and any("HUNG" in x for x in texts) and
                          any(x.startswith("LIDAR LOST") for x in texts))
        got["latency"] = st["sensors"]["lidar"]["latency"]
        lb = m.latest("radar")
        rs = [i["range_m"] for i in (lb or {}).get("items", [])]
        got["batch sim flag + nearest first"] = bool(lb and lb.get("sim") is True and "capped" in lb and
                                                     rs == sorted(rs))
        bad = HarbourSensors({"sim": True, "python": os.path.join(HERE, "no_such_python.exe"),
                              "lidar": {"enabled": False}, "radar": {"start_s": 1.0}}, FakeGw())
        try:
            got["cannot start = LOST"] = run_until(bad, lambda: bad.health("radar")[0] == "LOST" and
                                                   "cannot start" in bad.health("radar")[1], 8)
        finally:
            bad.stop()
        got["fault refused for a real driver"] = not HarbourSensors({"lidar": {"enabled": True, "driver": "livox"}},
                                                                    gw).fault("lidar", "lost")
    finally:
        m.stop()
    ok = (all(got.get(k) is not None for k in ("ok", "lost", "back", "skew", "degraded", "crash", "hang",
                                               "no pose = lost", "cannot start = LOST"))
          and got["new pid"] and got["batch sim flag + nearest first"] and got["notices"] and got["fault refused for a real driver"]
          and (got["prio"] in (None, 0x4000)))
    report("HS7 worker processes: health, notices, crash / hang restart", ok,
           json.dumps(got, default=str)[:500])


def hs8():
    gw = FakeGw()
    m = HarbourSensors({"enabled": True, "radar": {"enabled": True, "driver": "mayara"},
                        "lidar": {"enabled": True, "driver": "livox"}}, gw)
    try:
        t = run_until(m, lambda: m.health("radar")[0] == "OFFLINE" and m.health("lidar")[0] == "OFFLINE", 15)
        time.sleep(1.0)
        m.tick()
        why = m.health("radar")[1] + " | " + m.health("lidar")[1]
        nob = m.procs["radar"].batches == 0 and m.procs["lidar"].batches == 0
    finally:
        m.stop()
    report("HS8 real driver = skeleton (OFFLINE, W148)", t is not None and nob and "W148" in why and
           "untested" in why, why)


def hs9():
    pkg = os.path.join(HERE, "harbour")
    bad = []
    banned = re.compile(r"^\s*(import|from)\s+(aurix|stations|proto|socket|asyncio|urllib|http|requests|"
                        r"websocket|uav|gateway)\b", re.M)
    for fn in sorted(os.listdir(pkg)):
        if fn.endswith(".py"):
            s = open(os.path.join(pkg, fn), encoding="utf-8").read()
            if banned.search(s) or re.search(r"T_GW_NAVREQ|p_gw_navreq|\.aurix\b|alarm_bits", s):
                bad.append(fn)
    probe = os.path.join(tempfile.gettempdir(), "hs9_probe_%d.py" % os.getpid())
    with open(probe, "w", encoding="utf-8") as f:
        f.write("import socket, sys, json, threading\n"
                "hits = []\n"
                "class T(socket.socket):\n"
                "    def __init__(self, *a, **k):\n"
                "        hits.append(1); sys.stderr.write('NET\\n'); raise OSError('blocked')\n"
                "socket.socket = T\n"
                "socket.create_connection = lambda *a, **k: (hits.append(1), (_ for _ in ()).throw(OSError('b')))\n"
                "sys.path.insert(0, %r)\n"
                "from harbour import worker\n"
                "import atexit\n"
                "def done():\n"
                "    bad = [m for m in ('aurix', 'stations', 'proto') if m in sys.modules]\n"
                "    sys.stderr.write('HITS %%d BAD %%s\\n' %% (len(hits), bad))\n"
                "atexit.register(done)\n"
                "worker.run(sys.argv[1], sys.argv[2:])\n" % HERE)
    res = {}
    try:
        for kind in ("radar", "lidar"):
            p = subprocess.Popen([sys.executable, probe, kind, "--cfg", json.dumps({"rate_hz": 5.0})],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            buf = []
            th = threading.Thread(target=lambda: buf.append(p.stdout.read()), daemon=True)
            th.start()                              # read while it runs (a full pipe would block the worker)
            for i in range(12):
                pz = dict(pose_at(10.0, 10.0, 0.0, t=time.time()), op="pose")
                p.stdin.write((json.dumps(pz) + "\n").encode())
                p.stdin.flush()
                time.sleep(0.25)
            p.stdin.close()
            p.wait(10)
            th.join(5)
            err = p.stderr.read()
            out = buf[0] if buf else b""
            lines = [json.loads(x) for x in out.decode().splitlines() if x.strip()]
            res[kind] = (sum(1 for x in lines if x.get("ev") == kind), err.decode().strip().splitlines()[-1:])
    finally:
        try:
            os.remove(probe)
        except OSError:
            pass
    ok = (not bad and all(v[0] > 3 and v[1] == ["HITS 0 BAD []"] for v in res.values()))
    report("HS9 no AURIX / no network", ok, "modules naming a banned import / navreq: %s; runs: %s" % (bad, res))


def hs10():
    w = simworld.World()
    fer = [o for o in w.objects if o["id"] == "ferry"][0]
    L = 800.0
    a = w.state(fer, 0.0)
    w.t0 = 0.0
    p_half = w.state(fer, (L / fer["speed"]) / 2)
    p_end = w.state(fer, L / fer["speed"])
    p_back = w.state(fer, 1.5 * L / fer["speed"])
    det = w.truth(123.4) == simworld.World(w.objects, None, 0.0).truth(123.4)
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        w.place(LAT0, LON0, 5.0)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(w.to_json(), f)
        w2 = simworld.World.load(path)
        rt = w2.origin == (LAT0, LON0) and w2.t0 == 0.0 and len(w2.objects) == len(w.objects)
    finally:
        os.remove(path)
    w3 = simworld.World()
    w3.place(1.0, 2.0, 7.0)
    w3.place(3.0, 4.0, 9.0)
    kinds = sorted({o["kind"] for o in w.objects})
    ais = [o for o in w.objects if o.get("ais")]
    no_ais = [o for o in w.objects if o["kind"] in ("vessel", "kayak", "swimmer") and not o.get("ais")]
    sys.path.insert(0, os.path.join(HERE, "..", "demo"))
    import demo_world
    dw = simworld.World(demo_world.harbour_world()["objects"])
    ok = (abs(p_half[1] - 0.0) < 1e-6 and abs(p_end[1] - 400.0) < 1e-6 and abs(p_back[1] - 0.0) < 1e-6 and
          p_back[2] == 180.0 and det and rt and w3.origin == (1.0, 2.0) and w3.t0 == 7.0 and
          kinds == ["buoy", "kayak", "moored", "pier", "swimmer", "vessel"] and len(ais) >= 1 and len(no_ais) >= 3
          and len(dw.truth(0.0)) >= 6)
    report("HS10 the world", ok, "ferry y at 0 / half / end / back: %.0f / %.0f / %.0f / %.0f (hdg back %.0f); kinds %s; "
           "AIS %d, without AIS %d; demo world %d objects" % (a[1], p_half[1], p_end[1], p_back[1], p_back[2], kinds,
                                                               len(ais), len(no_ais), len(dw.objects)))


def main():
    for fn in (hs1, hs2, hs3, hs4, hs5, hs6, hs7, hs8, hs9, hs10):
        try:
            fn()
        except Exception as e:                  # a crashing check = a FAIL, the others still run
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "crashed: %r" % e)
    bad = results.count(False)
    print("HARBOUR SENSOR TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
