"""Harbour autonomy's local planner (CLAUDE.md Z, docs/harbour_design.md 7, step HX5 - it steers the boat).

    python test_harbour_planner.py        (exit code 0 = all passed)

Pure planner tests use synthetic fused tracks; the closed-loop tests drive the HX3 SIMULATED radar + LiDAR + the HX4
tracker in-process (test_harbour_tracker.Sim, seeded = deterministic) with a kinematic boat that follows the planner's
proposals the way the AURIX validates them (slew 8 deg/s, the 60 deg check, the corridor HOLD, STOP = HOLD).

  HP1  COLREGs classes (head-on, crossing from starboard / port, overtaking, being overtaken, ambiguous, small craft)
  HP2  head-on: never a port turn; a starboard pass when the corridor allows it, else STOP; give-way flag
  HP3  crossing from starboard: give way (slow / STOP), never a port turn, the explanation «ΔΙΝΩ ΔΡΟΜΟ ΣΕ ΣΚΑΦΟΣ ΔΕΞΙΑ»
  HP4  crossing from port (stand-on): never a port turn; overtaking: keeps clear; being overtaken: stand-on
  HP5  moored boats in a narrow channel: the speed down to narrow_kn (or STOP), never a turn into them
  HP6  kayak / swimmer without AIS: LiDAR / radar-confirmed -> avoided (may turn); camera-only / AIS-only -> slow /
       STOP only, never a turn (M0-3)
  HP7  buoy on the line: ±10 m corridor = STOP (8 m cannot be kept), ±20 m = passed
  HP8  every unsure condition U1..U12 + stale tracker / unknown reason / no route / the sim gate -> STOP; U2 / U9 slow
       to degraded_kn; U6 / U8 slow then STOP after their time; U4 late cycle + over budget
  HP9  random scenarios: the plan never leaves the corridor, the speed never above the cap / the route, never raised
  HP10 closed loop (synthetic truth as tracks): the minimum distances to every object kept or STOP early enough
  HP11 the gateway glue: active() only with a fresh HARBOUR_STAT (RUN / SLOW / HOLD, or a checked route waiting),
       never waiting in UNCREWED HDG LOCK / REMOTE / MANEUVER / DP (their kind 1 / 2 lease), the sim gate (sim sensors
       + a real AURIX link = STOP), the fold (caps keep their reasons, a cap < 1.5 kn = STOP),
       the 13-byte kind-4 frame, lease 1000 ms, audit "harbour", the el / en line in the status
  HP12 lease timing: harbour_loop sends a kind-4 request every 0.2 s (5 Hz) while active, none while inactive
  HP13 writer check: only gateway.py builds GW_NAVREQ; nothing in gateway/harbour imports aurix / sends
  HP14 the explanation keys are in groundstation/lang el.txt / en.txt with the same text
  HP15 closed loop with the REAL tracker + simulated sensors: a LiDAR kayak avoided at >= 8 m (world truth), a
       camera-only obstacle never turned for, determinism (same seed = same decisions)
  HP16 mutation checks: the steer_ok gate removed -> HP6 fails; unsure -> stop removed -> HP8 fails; the distance
       check removed -> HP10 fails
"""
import asyncio
import math
import os
import random
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "groundstation"))
from harbour import planner as P, colregs as C, staticmap, simworld  # noqa: E402
import proto  # noqa: E402

KN = 0.514444
results = []


def report(name, ok, detail=""):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def trk(i, x, y, vx=0.0, vy=0.0, ln=8.0, beam=3.0, steer=True, static=False, cls="vessel", src="RL", sd=0.5,
        vsd=0.2):
    sp = math.hypot(vx, vy)
    return {"id": i, "x": x, "y": y, "vx": vx, "vy": vy, "speed": sp,
            "course": math.degrees(math.atan2(vx, vy)) % 360.0,
            "P": [[sd * sd, 0, 0, 0], [0, sd * sd, 0, 0], [0, 0, vsd * vsd, 0], [0, 0, 0, vsd * vsd]], "sd_m": sd,
            "len_m": ln, "beam_m": beam, "cls": cls, "steer_ok": steer, "range_ok": steer, "static": static,
            "src": src}


def inp(tracks, x=0.0, y=0.0, h=0.0, kn=4.0, flags=None, health=None, corr=10.0, route_kn=4.0, smap=None):
    return {"own": {"x": x, "y": y, "heading": h, "sog_kn": kn, "fresh": True, "gps_ok": True, "unc": 2.0},
            "leg": ((0.0, -50.0), (0.0, 600.0)), "corr_m": corr, "route_kn": route_kn, "tracks": tracks,
            "flags": flags if flags is not None else {"unsure": [], "cycle_age_s": 0.1}, "static": smap,
            "health": health or {"radar": "OK", "lidar": "OK"}, "block": []}


def plan1(tracks, **kw):
    pl = P.Planner()
    return pl.plan(10.0, inp(tracks, **kw))


def port_turn(d, leg=0.0, tol=3.0):
    return d["heading"] is not None and P.wrap(d["heading"] - leg) < -tol


def stbd_turn(d, leg=0.0, tol=3.0):
    return d["heading"] is not None and P.wrap(d["heading"] - leg) > tol


# --- HP1 ---------------------------------------------------------------------------------------------------------
def hp1():
    own = {"x": 0.0, "y": 0.0, "hdg": 0.0, "vx": 0.0, "vy": 2.0, "speed": 2.0}
    c = P.DEFAULTS
    cases = [("head_on", trk(1, 1, 120, 0, -2.5), C.HEAD_ON), ("stbd", trk(2, 50, 50, -2.0, 0), C.CROSS_STBD),
             ("port", trk(3, -50, 50, 2.0, 0), C.CROSS_PORT), ("overtaking", trk(4, 0, 30, 0, 0.8), C.OVERTAKING),
             ("overtaken", trk(5, 2, -30, 0, 3.5), C.OVERTAKEN),
             ("ambiguous (aspect unknown)", trk(6, 1, 120, 0, -1.0, vsd=0.9), C.AMBIGUOUS)]
    ok, bad = True, []
    for name, tr, want in cases:
        e = C.classify(own, tr, c)
        if e is None or e["cls"] != want:
            ok = False
            bad.append("%s=%s" % (name, e and e["cls"]))
    small = C.classify(own, trk(7, 1, 60, 0, -1.0, cls="small"), c)
    notok = C.classify(own, trk(8, 1, 120, 0, -2.5, steer=False, src="C"), c)
    far = C.classify(own, trk(9, 200, 0, 0, 2.0), c)
    _l, amb2 = C.encounters(own, [trk(1, 1, 120, 0, -2.5), trk(2, 50, 50, -2.0, 0)], c)
    ok = ok and small is None and notok is None and far is None and amb2
    report("HP1 COLREGs classes", ok, "bad %s; small %s, not steer_ok %s, no CPA %s, two classes ambiguous %s"
           % (bad, small, notok, far, amb2))


# --- HP2 - HP7 ---------------------------------------------------------------------------------------------------
def hp2():
    narrow = plan1([trk(1, 2, 150, 0, -2.5)])
    wide = plan1([trk(1, -12, 160, 0, -2.5)], corr=20.0)
    ok = (narrow["stop"] or not port_turn(narrow)) and narrow["hflags"] & 0x04 and \
        not wide["stop"] and stbd_turn(wide) and wide["why"] == "harb.why_head_on" and not port_turn(wide)
    report("HP2 head-on: starboard only", ok, "narrow stop=%s why=%s; wide heading %s why %s"
           % (narrow["stop"], narrow["why"], wide["heading"], wide["why"]))


def hp3():
    ok, det = True, []
    for x, y, vx in ((60, 60, -2.0), (90, 80, -2.0), (40, 50, -1.5)):
        for corr in (10.0, 20.0):
            d = plan1([trk(2, x, y, vx, 0)], corr=corr)
            enc = [e for e in d["colregs"] if e["cls"] == C.CROSS_STBD]
            g = not enc or (d["why"] == "harb.why_giveway_stbd" and d["hflags"] & 0x04)
            slow_or_stop = not enc or d["stop"] or d["cap_kn"] < 4.0 or stbd_turn(d)
            if port_turn(d) or not g or not slow_or_stop:
                ok = False
            det.append("(%s,%s,%s) stop=%s cap=%s h=%s" % (x, y, corr, d["stop"], d["cap_kn"], d["heading"]))
    el, _en = P.why_text("harb.why_giveway_stbd")
    ok = ok and el == "ΔΙΝΩ ΔΡΟΜΟ ΣΕ ΣΚΑΦΟΣ ΔΕΞΙΑ"
    report("HP3 crossing from starboard: give way", ok, "; ".join(det))


def hp4():
    port = [plan1([trk(3, -x, x, 2.0, 0)], corr=c) for x in (50, 70) for c in (10.0, 20.0)]
    ov = plan1([trk(4, 0, 40, 0, 0.8)], corr=20.0)
    ovk = plan1([trk(5, 2, -40, 0, 4.0)], corr=20.0)
    ok = all(not port_turn(d) for d in port) and all(d["why"] == "harb.why_standon_port" for d in port) and \
        not ov["stop"] and ov["why"] == "harb.why_overtaking" and (ov["turn"] or ov["cap_kn"] < 4.0) and \
        ovk["why"] in ("harb.why_overtaken", "harb.why_stop") and not port_turn(ovk)
    report("HP4 crossing from port / overtaking / overtaken", ok, "port %s; overtaking %s %s %s; overtaken %s %s"
           % ([(d["stop"], d["heading"]) for d in port], ov["heading"], ov["cap_kn"], ov["why"], ovk["why"],
              ovk["heading"]))


def hp5():
    moored = [trk(10 + i, sx * 15.0, 30.0 + 22.0 * i, static=True, ln=10, beam=3.5) for i, sx in
              enumerate((-1, 1, -1, 1))]
    d = plan1(moored, route_kn=5.0, kn=5.0)
    ok = d["stop"] or (d["cap_kn"] <= P.DEFAULTS["narrow_kn"] + 1e-9)
    # a narrow gap that cannot keep 8 m: never squeezes through (closed loop: HOLD before it, 8 m kept)
    r = run_truth([{"id": "a", "kind": "moored", "x": -8.5, "y": 60.0, "len": 10, "beam": 3.5, "hdg": 0, "h": 2},
                   {"id": "b", "kind": "moored", "x": 8.5, "y": 60.0, "len": 10, "beam": 3.5, "hdg": 0, "h": 2}],
                  90.0, 10.0, 4.0)
    tight = {"stop": r["boat"].hold and r["boat"].y < 60.0 and min(g for g, _m in r["min_gap"].values()) >= 7.5,
             "reasons": "y %.0f gaps %s" % (r["boat"].y, r["min_gap"])}
    ok = ok and tight["stop"]
    report("HP5 moored boats / narrow = slow or STOP", ok, "4 moored: stop=%s cap=%s why=%s; tight gap stop=%s %s"
           % (d["stop"], d["cap_kn"], d["why"], tight["stop"], tight["reasons"]))


def hp6(planner_mod=P):
    """-> (range-confirmed objects passed, camera / AIS-only never turned for, details); the mutation check reuses it.
    Closed loop on the world truth: the same kayak / swimmer seen as steer_ok tracks, or as tracks that are NOT."""
    objs = [[{"id": "kayak", "kind": "kayak", "x": 1.0, "y": 80.0, "len": 4, "beam": 0.8, "h": 0.4}],
            [{"id": "swimmer", "kind": "swimmer", "x": -1.0, "y": 70.0, "len": 0.6, "beam": 0.5, "h": 0.3}]]
    res = []
    for o in objs:
        rng = run_truth(o, 90.0, 20.0, 4.0, planner_mod, steer=True)
        cam = run_truth(o, 90.0, 20.0, 4.0, planner_mod, steer=False)
        res.append((rng, cam))
    passed = all(r[0]["boat"].y > 100.0 and not r[0]["boat"].hold and r[0].get("turns") for r in res)
    no_turn = all(not r[1].get("turns") and r[1]["boat"].hold for r in res)
    return passed, no_turn, res


def plan_mod_one(mod, tracks, **kw):
    pl = mod.Planner()
    return pl.plan(10.0, inp(tracks, **kw))


def hp6_report():
    passed, no_turn, res = hp6()
    report("HP6 kayak / swimmer: range-confirmed passed (turns), camera / AIS only never turned for (slow, STOP)",
           passed and no_turn, "range: y %s hold %s turns %s; not confirmed: turns %s hold %s gaps %s" % (
               [round(r[0]["boat"].y) for r in res], [r[0]["boat"].hold for r in res],
               [len(r[0].get("turns") or []) for r in res], [len(r[1].get("turns") or []) for r in res],
               [r[1]["boat"].hold for r in res], [r[1]["min_gap"] for r in res]))


def hp7():
    buoy = [{"id": "buoy", "kind": "buoy", "x": 0.0, "y": 70.0, "len": 1, "beam": 1, "h": 1.2}]
    n = run_truth(buoy, 90.0, 10.0, 4.0)
    w = run_truth(buoy, 90.0, 20.0, 4.0)
    ok = n["boat"].hold and n["boat"].y < 70.0 and n["min_gap"]["buoy"][0] >= 7.5 and         not w["boat"].hold and w["boat"].y > 90.0 and w["min_gap"]["buoy"][0] >= 7.5
    report("HP7 buoy on the line", ok, "±10 m: HOLD %s at y %.0f gap %.1f; ±20 m: passed y %.0f gap %.1f" % (
        n["boat"].hold, n["boat"].y, n["min_gap"]["buoy"][0], w["boat"].y, w["min_gap"]["buoy"][0]))


# --- HP8 the unsure list -------------------------------------------------------------------------------------------
UNSURE = [("U1", ["U1 range sensors lost"]), ("U10", ["U10 no static map"]), ("U11", ["U11 radar time skew"]),
          ("no pose", ["no own pose"]), ("error", ["tracker error"]), ("overload", ["tracker overload: cycle 40 ms"]),
          ("input rejected", ["input rejected (no pose / heading, a time jump or a bad item): 3 in all"]),
          ("not started", ["tracker not started"]), ("unknown", ["something new"])]


def hp8(planner_mod=P):
    out = {}
    for name, u in UNSURE:
        out[name] = plan_mod_one(planner_mod, [], flags={"unsure": u, "cycle_age_s": 0.1})["stop"]
    out["cycle old"] = plan_mod_one(planner_mod, [], flags={"unsure": [], "cycle_age_s": 0.9})["stop"]
    out["flags missing"] = planner_mod.Planner().plan(1.0, dict(inp([]), flags=None))["stop"]
    i = inp([])
    i["own"]["gps_ok"] = False
    out["U3 gps"] = planner_mod.Planner().plan(1.0, i)["stop"]
    i = inp([])
    i["own"]["unc"] = 9.0
    out["U3 unc"] = planner_mod.Planner().plan(1.0, i)["stop"]
    i = inp([])
    i["own"]["heading"] = None
    out["U12"] = planner_mod.Planner().plan(1.0, i)["stop"]
    i = inp([])
    i["own"]["fresh"] = False
    out["U12 stale"] = planner_mod.Planner().plan(1.0, i)["stop"]
    out["no leg"] = planner_mod.Planner().plan(1.0, dict(inp([]), leg=None))["stop"]
    out["block (sim gate)"] = planner_mod.Planner().plan(1.0, dict(inp([]), block=["sim gate"]))["stop"]
    out["U7"] = plan_mod_one(planner_mod, [trk(1, 3, 4, static=True, ln=1, beam=1, cls="buoy")])["stop"]
    out["U5"] = plan_mod_one(planner_mod, [trk(1, 0, 25, static=True, ln=40, beam=30)])["stop"]
    pl = planner_mod.Planner()
    pl.plan(1.0, inp([]))
    out["U4 late"] = pl.plan(1.6, inp([]))["stop"]
    out["U4 budget"] = planner_mod.Planner().plan(1.0, inp([trk(1, 30, 200, -1, 0)]), deadline=0.0)["stop"]
    # slow-only reasons
    d2 = plan_mod_one(planner_mod, [], flags={"unsure": ["U2 lidar LOST"], "cycle_age_s": 0.1})
    d9 = plan_mod_one(planner_mod, [], flags={"unsure": ["U9 clutter (20 new tracks / s within 25 m)"],
                                             "cycle_age_s": 0.1})
    slow_ok = not d2["stop"] and d2["cap_kn"] <= 2.0 and not d9["stop"] and d9["cap_kn"] <= 2.0
    # U6 / U8: slow, then STOP after their time
    pl = planner_mod.Planner()
    f6 = {"unsure": ["U6 conflicting tracks (1)"], "cycle_age_s": 0.1}
    a = pl.plan(1.0, inp([], flags=f6))
    b = pl.plan(1.2, inp([], flags=f6))
    for k in range(15):
        b = pl.plan(1.4 + 0.2 * k, inp([], flags=f6))
    u6 = not a["stop"] and a["cap_kn"] <= 2.0 and b["stop"]
    pl = planner_mod.Planner()
    two = [trk(1, 1, 120, 0, -2.5), trk(2, 60, 60, -2.0, 0)]
    a8 = pl.plan(1.0, inp(two, corr=20.0))
    for k in range(17):
        b8 = pl.plan(1.2 + 0.2 * k, inp(two, corr=20.0))
    u8 = (a8["stop"] or a8["cap_kn"] <= 2.0) and b8["stop"]
    return out, slow_ok, u6, u8


def hp8_report():
    out, slow_ok, u6, u8 = hp8()
    bad = [k for k, v in out.items() if not v]
    report("HP8 every unsure condition = STOP (U2 / U9 slow, U6 / U8 slow then STOP)",
           not bad and slow_ok and u6 and u8, "not stopped: %s; slow %s, U6 %s, U8 %s" % (bad, slow_ok, u6, u8))


# --- HP9 random: corridor / cap / never raised -----------------------------------------------------------------------
def hp9(planner_mod=P, quiet=False):
    rng = random.Random(9)
    bad = []
    n = 0
    for k in range(300):
        tracks = []
        for i in range(rng.randint(0, 5)):
            st = rng.random() < 0.4
            tracks.append(trk(i, rng.uniform(-40, 40), rng.uniform(-20, 200),
                              0 if st else rng.uniform(-3, 3), 0 if st else rng.uniform(-3, 3),
                              ln=rng.choice([1, 4, 8, 15]), beam=rng.choice([1, 3, 5]), static=st,
                              steer=rng.random() < 0.8, cls=rng.choice(["vessel", "small", "buoy", "unknown"])))
        corr = rng.choice([6.0, 10.0, 14.0, 20.0])
        route = rng.choice([2.0, 3.5, 5.0, 7.0])
        e0 = rng.uniform(-corr + 2.5, corr - 2.5)
        flags = {"unsure": rng.choice([[], [], ["U2 radar LOST"], ["U9 crowded (21 moving range tracks)"]]),
                 "cycle_age_s": 0.1}
        d = planner_mod.Planner().plan(10.0, inp(tracks, x=e0, y=rng.uniform(-40, 50), h=rng.uniform(-30, 30), corr=corr,
                                       route_kn=route, kn=rng.uniform(0, 5), flags=flags))
        if d["stop"]:
            continue
        n += 1
        cap = min(route, 5.0) if not flags["unsure"] else min(route, 5.0, 2.0)
        if d["cap_kn"] > cap + 1e-9:
            bad.append("cap %s > %s" % (d["cap_kn"], cap))
        if abs(P.wrap(d["heading"])) > 45.0 + 1e-6:
            bad.append("heading %s off the leg" % d["heading"])
        usable = corr - 1.25 - 1.0 - 2.0
        if d["offset"] is None or abs(d["offset"]) > usable + 1e-6:
            bad.append("offset %s outside the usable corridor %.2f" % (d["offset"], usable))
        # independent: follow the heading 40 s with the AURIX's slew + the own guidance, the line inside corr
        if abs(d["offset"]) > corr:
            bad.append("planned line outside the corridor")
    if quiet:
        return bad
    report("HP9 random: inside the corridor, cap <= min(route, 5 kn, degraded), never above", not bad and n > 50,
           "%d moving decisions, %s" % (n, bad[:4]))


# --- the closed loop (a kinematic boat = the AURIX's validation + the world) -----------------------------------------
class Boat:
    """Follows the planner like the AURIX: the 60 deg check, the heading target slewed 8 deg/s, the boat's turn rate,
    the speed target min(leg, 5 kn, cap) with a 6 s speed constant, STOP / no proposal = HOLD (idle + neutral: the
    speed decays, the heading held), |xte| > corr + 5 = HOLD. Latched HOLD (only the captain resumes)."""

    def __init__(self, x=0.0, y=0.0, hdg=0.0, kn=0.0, corr=10.0, route_kn=4.0, leg=((0.0, -50.0), (0.0, 600.0))):
        self.x, self.y, self.hdg, self.v = x, y, hdg, kn * KN
        self.tgt = hdg
        self.corr, self.route_kn, self.leg = corr, route_kn, leg
        self.hold = False
        self.hold_t = None
        self.t = 0.0

    def xte(self):
        (ax, ay), (bx, by) = self.leg
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        return (self.x - ax) * uy - (self.y - ay) * ux

    def step(self, dec, dt=0.2):
        psi = math.degrees(math.atan2(self.leg[1][0] - self.leg[0][0], self.leg[1][1] - self.leg[0][1]))
        if not self.hold and (dec is None or dec["stop"] or abs(self.xte()) > self.corr + 5.0):
            self.hold, self.hold_t = True, self.t
        if self.hold:
            vt = 0.0
            want = self.tgt
        else:
            vt = min(self.route_kn, 5.0, dec["cap_kn"]) * KN
            want = dec["heading"] if abs(P.wrap(dec["heading"] - psi)) <= 60.0 else psi
        self.tgt = (self.tgt + max(-8.0 * dt, min(8.0 * dt, P.wrap(want - self.tgt)))) % 360.0
        rate = min(8.0, 0.6 * self.v / KN)
        self.hdg = (self.hdg + max(-rate * dt, min(rate * dt, P.wrap(self.tgt - self.hdg)))) % 360.0
        self.v = vt + (self.v - vt) * math.exp(-dt / 6.0)
        self.x += self.v * dt * math.sin(math.radians(self.hdg))
        self.y += self.v * dt * math.cos(math.radians(self.hdg))
        self.t += dt

    def own(self):
        return {"x": self.x, "y": self.y, "heading": self.hdg, "sog_kn": self.v / KN, "fresh": True, "gps_ok": True,
                "unc": 2.0}


def hull_gap(boat, o):
    """The hull-to-hull gap between the own boat (6 m) and a world object (truth)."""
    if o["kind"] == "pier":
        x1, y1, x2, y2 = o["x1"], o["y1"], o["x2"], o["y2"]
        L2 = (x2 - x1) ** 2 + (y2 - y1) ** 2
        f = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((boat.x - x1) * (x2 - x1) + (boat.y - y1) * (y2 - y1)) / L2))
        d = math.hypot(boat.x - (x1 + f * (x2 - x1)), boat.y - (y1 + f * (y2 - y1)))
        return d - 0.5 * o.get("w", 5.0) - 3.0
    return math.hypot(boat.x - o["x"], boat.y - o["y"]) - 0.5 * max(o["len"], o["beam"]) - 3.0


def truth_tracks(world, t, steer=True):
    out = []
    for o in world.truth(t):
        if o["kind"] == "pier":
            continue
        vx = o["speed"] * math.sin(math.radians(o["hdg"]))
        vy = o["speed"] * math.cos(math.radians(o["hdg"]))
        out.append(trk(o["id"], o["x"], o["y"], vx, vy, ln=o["len"], beam=o["beam"], steer=steer,
                       static=o["speed"] <= 0.0, cls=simworld.CLS.get(o["kind"], "unknown")))
    return out


def run_truth(objects, secs=120.0, corr=10.0, route=4.0, planner_mod=P, cfg=None, start_y=0.0, steer=True):
    """The planner fed the WORLD TRUTH as tracks (an ideal tracker) -> the gap / corridor / cap record."""
    w = simworld.World(objects, (37.82, 23.60), 0.0)
    pl = planner_mod.Planner(cfg)
    b = Boat(0.0, start_y, 0.0, route, corr, route)
    rec = {"min_gap": {}, "late_hold": [], "xte_max": 0.0, "cap_bad": [], "decs": 0, "stops": 0}
    t = 0.0
    dec = None
    while t < secs:
        tr = truth_tracks(w, t, steer)
        dec = pl.plan(t, {"own": b.own(), "leg": b.leg, "corr_m": corr, "route_kn": route, "tracks": tr,
                          "flags": {"unsure": [], "cycle_age_s": 0.0}, "static": None,
                          "health": {"radar": "OK", "lidar": "OK"}, "block": []})
        rec["decs"] += 1
        if dec.get("turn") and dec.get("turn_for"):
            rec.setdefault("turns", []).append((round(t, 1), list(dec["turn_for"])))
        if dec["stop"]:
            rec["stops"] += 1
        elif dec["cap_kn"] > min(route, 5.0) + 1e-9:
            rec["cap_bad"].append(dec["cap_kn"])
        was_hold = b.hold
        b.step(dec)
        if b.hold and not was_hold:
            # HOLD began: it must begin with room for the coast-down (gap >= min + v * 6 s), else it came too late
            for o in w.truth(t):
                need = (15.0 if o["speed"] > 0.5 else 8.0)
                if hull_gap(b, o) < need - 0.5 and o["speed"] <= 0.5:
                    rec["late_hold"].append((o["id"], round(hull_gap(b, o), 1)))
        if not b.hold:
            rec["xte_max"] = max(rec["xte_max"], abs(b.xte()))
            for o in w.truth(t):
                g = hull_gap(b, o)
                k = o["id"]
                if g < rec["min_gap"].get(k, (1e9,))[0]:
                    rec["min_gap"][k] = (round(g, 2), o["speed"] > 0.5)
        t += 0.2
    rec["boat"] = b
    return rec


def hp10(planner_mod=P):
    scen = [
        ("buoy wide", [{"id": "buoy", "kind": "buoy", "x": 1.0, "y": 80.0, "len": 1, "beam": 1, "h": 1.2}], 20.0),
        ("kayak crossing", [{"id": "kayak", "kind": "kayak", "path": [[30, 120], [-30, 120]], "speed": 0.8,
                             "len": 4, "beam": 0.7, "h": 0.4, "ais": None}], 20.0),
        ("head-on", [{"id": "v", "kind": "vessel", "path": [[-6, 260], [-6, -400]], "speed": 2.0, "len": 8,
                      "beam": 3, "h": 1.5, "ais": None}], 20.0),
        ("moored both sides", [{"id": "m%d" % i, "kind": "moored", "x": sx * 16.0, "y": 50.0 + 25 * i, "len": 10,
                                "beam": 3.5, "hdg": 0, "h": 2} for i, sx in enumerate((-1, 1, -1))], 10.0),
        ("slow boat ahead", [{"id": "slow", "kind": "vessel", "path": [[0, 60], [0, 900]], "speed": 0.7, "len": 6,
                              "beam": 2.5, "h": 1.2, "ais": None}], 20.0),
        ("buoy narrow", [{"id": "buoy", "kind": "buoy", "x": 0.0, "y": 90.0, "len": 1, "beam": 1, "h": 1.2}], 10.0)]
    bad, det = [], []
    for name, objs, corr in scen:
        r = run_truth(objs, 150.0, corr, 4.0, planner_mod)
        for k, (g, mov) in r["min_gap"].items():
            if g < (15.0 if mov else 8.0) - 0.5:
                bad.append("%s: %s gap %.1f m" % (name, k, g))
        if r["late_hold"]:
            bad.append("%s: HOLD too late %s" % (name, r["late_hold"]))
        if r["xte_max"] > corr:
            bad.append("%s: xte %.1f > corr" % (name, r["xte_max"]))
        if r["cap_bad"]:
            bad.append("%s: cap above the route" % name)
        det.append("%s gap %s xte %.1f stops %d" % (name, {k: v[0] for k, v in r["min_gap"].items()},
                                                    r["xte_max"], r["stops"]))
    return bad, det


def hp10_report():
    bad, det = hp10()
    report("HP10 closed loop: min distances 15 / 8 m kept (or HOLD early), corridor, cap", not bad,
           "%s | %s" % (bad[:4], "; ".join(det)))


# --- HP11 / HP12 the gateway glue --------------------------------------------------------------------------------
class FakeProc:
    def __init__(self):
        self.c = {"enabled": True}


class FakeHarbour:
    def __init__(self, sim=True):
        self.enabled = True
        self.sim = sim
        self.static = staticmap.StaticMap()
        self.static.add_segment(-100, 300, -90, 300, 2)
        from harbour import tracker
        self.tracker = tracker.Tracker()
        self.tracker.set_origin(37.82, 23.60)
        self.procs = {"radar": FakeProc(), "lidar": FakeProc()}
        self.tr = []

    def tracks(self):
        return list(self.tr)

    def tracker_flags(self):
        return {"unsure": [], "cycle_age_s": 0.05}

    def health(self, k):
        return "OK", ""


class FakeGw:
    def __init__(self, aurix="tcp:127.0.0.1:5762", sim=True):
        self.cfg = {"aurix": aurix}
        self.harbour = FakeHarbour(sim)
        lat1, lon1 = simworld.xy_to_ll(37.82, 23.60, 0.0, -50.0)
        lat2, lon2 = simworld.xy_to_ll(37.82, 23.60, 0.0, 600.0)
        self.prog_wps = [(lat1, lon1), (lat2, lon2)]
        self.prog_kn = [4.0, 4.0]
        self.audits = []
        self.lat, self.lon = simworld.xy_to_ll(37.82, 23.60, 0.0, 0.0)
        self.mode = proto.MODE_AUTO
        self.profile = proto.PROFILE_CREWED
        self.telem_rx = time.time()

    def own(self):
        return {"fresh": True, "lat": self.lat, "lon": self.lon, "heading": 0.0, "sog": 4.0, "unc": 2.0,
                "mode": self.mode, "profile": self.profile}

    def telem_fresh(self):
        return {"pos_source": 1, "prog_gw": True, "prog_n": 2, "auto_wp": 1}

    def audit(self, kind, **kw):
        self.audits.append(dict(kw, kind=kind))


def stat(state=1, route=True):
    return {"harb_state": state, "harb_route": route, "harb_corr_m": 10.0, "harb_state_name": "RUN"}


def hp11():
    det = []
    gw = FakeGw()
    hp = P.HarbourPlanner(gw)
    a0 = hp.active()
    hp.on_stat(stat(1))
    a1 = hp.active()
    hp.on_stat(stat(0, route=True))
    a_pre_auto = hp.active()
    gw.mode = proto.MODE_STANDBY
    no_window = hp.active()                            # review MED-2: no START asked = no pre-start kind 4
    hp.note_start()
    a_pre = hp.active() and not no_window
    pre_modes = []
    for m, prof in ((proto.MODE_REMOTE, proto.PROFILE_UNCREWED), (proto.MODE_HDG_LOCK, proto.PROFILE_UNCREWED),
                    (proto.MODE_MANEUVER, proto.PROFILE_UNCREWED), (proto.MODE_DP, proto.PROFILE_CREWED)):
        gw.mode, gw.profile = m, prof
        pre_modes.append(hp.active())
    gw.mode, gw.profile = proto.MODE_HDG_LOCK, proto.PROFILE_CREWED
    a_pre = a_pre and hp.active() and not any(pre_modes)
    gw.mode = proto.MODE_STANDBY
    hp.start_mono -= 31.0
    a_pre = a_pre and not hp.active()                  # the window lapsed (30 s)
    hp.note_start()
    hp.on_stat(stat(0, route=False))
    a_none = hp.active()
    hp.on_stat(stat(4))
    a_done = hp.active()
    hp.on_stat(stat(1))
    hp.stat_rx -= 2.0
    a_stale = hp.active()
    act_ok = (not a0 and a1 and not a_pre_auto and a_pre and not a_none and not a_done and not a_stale)
    det.append("active %s" % [a0, a1, a_pre_auto, a_pre, a_none, a_done, a_stale])
    # the fold + the frame
    gw.mode = proto.MODE_AUTO
    hp = P.HarbourPlanner(gw)
    hp.on_stat(stat(1))
    req = hp.fold({"kind": proto.NAV_NONE, "heading": None, "cap_kn": None, "reason": 0, "alarm": 0}, gw.own())
    clear = req["kind"] == proto.NAV_HARBOUR and not (req["hflags"] & 1) and abs(req["cap_kn"] - 4.0) < 1e-6 and \
        req["lease_ms"] == 1000
    hp.dec = None
    req2 = hp.fold({"kind": proto.NAV_CAP, "heading": None, "cap_kn": 2.5, "reason": 2, "alarm": 4}, gw.own())
    shallow = req2["cap_kn"] == 2.5 and req2["hflags"] & 0x20 and req2["reason"] == 2 and not req2["hflags"] & 1
    hp.dec = None
    req3 = hp.fold({"kind": proto.NAV_CAP, "heading": None, "cap_kn": 1.0, "reason": 8, "alarm": 8}, gw.own())
    lowcap = bool(req3["hflags"] & 1) and req3["cap_kn"] == 0.0 and req3["hflags"] & 0x80
    fr = proto.p_gw_navreq_harbour(7, req["reason"], req["alarm"], req["heading"], req["cap_kn"], req["lease_ms"],
                                   req["hflags"], req["nearest_m"], req["n_tracks"])
    import struct
    seq, kind, _r, _a, hdg, cap, lease = struct.unpack_from("<BBBBHHH", fr)
    frame_ok = len(fr) == 13 and kind == 4 and lease == 1000 and cap == 40 and hdg < 36000
    st = hp.status()
    text_ok = st["text_el"] and st["text_en"] and any(a["kind"] == "harbour" for a in gw.audits)
    det.append("clear %s shallow %s lowcap %s frame %s text %s" % (clear, shallow, lowcap, frame_ok, text_ok))
    # the sim gate: sim sensors + a real AURIX link = STOP + audit refused
    gw2 = FakeGw(aurix="COM7", sim=True)
    hp2 = P.HarbourPlanner(gw2)
    hp2.on_stat(stat(1))
    rq = hp2.fold({"kind": 0, "heading": None, "cap_kn": None, "reason": 0, "alarm": 0}, gw2.own())
    gate = bool(rq["hflags"] & 1) and hp2.gate and any(a.get("what") == "refused" for a in gw2.audits)
    gw3 = FakeGw(aurix="COM7", sim=False)
    hp3 = P.HarbourPlanner(gw3)
    hp3.on_stat(stat(1))
    rq3 = hp3.fold({"kind": 0, "heading": None, "cap_kn": None, "reason": 0, "alarm": 0}, gw3.own())
    gate_real = not (rq3["hflags"] & 1)
    # a route not the gateway's = STOP; inactive = the request untouched
    gw4 = FakeGw()
    gw4.telem_fresh = lambda: {"pos_source": 1, "prog_gw": False, "prog_n": 2, "auto_wp": 1}
    hp4 = P.HarbourPlanner(gw4)
    hp4.on_stat(stat(1))
    rq4 = hp4.fold({"kind": 0, "heading": None, "cap_kn": None, "reason": 0, "alarm": 0}, gw4.own())
    gw6 = FakeGw()
    gw6.telem_rx = time.time() - 1.0                   # U12: the own pose 1 s old (gw.own() still "fresh": 3 s)
    hp6 = P.HarbourPlanner(gw6)
    hp6.on_stat(stat(1))
    rq6 = hp6.fold({"kind": 0, "heading": None, "cap_kn": None, "reason": 0, "alarm": 0}, gw6.own())
    old_pose = bool(rq6["hflags"] & 1)
    hp5 = P.HarbourPlanner(FakeGw())
    base = {"kind": proto.NAV_CAP, "heading": None, "cap_kn": 3.0, "reason": 8, "alarm": 8}
    untouched = hp5.fold(dict(base), FakeGw().own()) == base
    det.append("gate %s real %s route %s untouched %s old pose %s" % (gate, gate_real, bool(rq4["hflags"] & 1),
                                                                      untouched, old_pose))
    report("HP11 the gateway glue (active, fold, frame, sim gate, audit, el / en line)",
           act_ok and clear and shallow and lowcap and frame_ok and text_ok and gate and gate_real and
           rq4["hflags"] & 1 and untouched and old_pose, "; ".join(det))


def hp12():
    import gateway as G

    class Stub:
        def __init__(self, active):
            self.n = 0
            self.harbour = type("H", (), {"tick": lambda s: None})()
            self.hplan = type("HP", (), {"active": lambda s: active})()

        def nav_tick(self):
            self.n += 1

    async def go(stub, secs):
        task = asyncio.create_task(G.Gateway.harbour_loop(stub))
        await asyncio.sleep(secs)
        task.cancel()

    on, off = Stub(True), Stub(False)
    asyncio.run(go(on, 1.05))
    asyncio.run(go(off, 0.5))
    ok = 5 <= on.n <= 7 and off.n == 0
    report("HP12 lease timing: 5 Hz kind-4 requests while active (lease 1000 ms), none inactive",
           ok and P.DEFAULTS["lease_ms"] == 1000, "%d in 1.05 s active, %d inactive" % (on.n, off.n))


def hp13():
    bad = []
    for root, _d, files in os.walk(HERE):
        if "test_" in root or "__pycache__" in root:
            continue
        for fn in files:
            if not fn.endswith(".py") or fn.startswith("test_"):
                continue
            p = os.path.join(root, fn)
            src = open(p, encoding="utf-8").read()
            if "T_GW_NAVREQ" in src and os.path.relpath(p, HERE) != "gateway.py":
                bad.append(p)
    for fn in os.listdir(os.path.join(HERE, "harbour")):
        if fn.endswith(".py"):
            src = open(os.path.join(HERE, "harbour", fn), encoding="utf-8").read()
            if re.search(r"^\s*(import|from)\s+(aurix|stations|socket)\b", src, re.M) or ".send(" in src and \
                    fn in ("planner.py", "colregs.py"):
                bad.append(fn)
    if (P.NAV_HARBOUR, P.MODE_STANDBY, P.MODE_MANUAL, P.MODE_HDG_LOCK, P.PROFILE_CREWED) != (
            proto.NAV_HARBOUR, proto.MODE_STANDBY, proto.MODE_MANUAL, proto.MODE_HDG_LOCK, proto.PROFILE_CREWED):
        bad.append("planner.py's proto constants differ")
    report("HP13 writer check: only gateway.py builds GW_NAVREQ; harbour/ imports no aurix / stations / socket",
           not bad, "bad %s" % bad)


def hp14():
    lang = {}
    for lg in ("el", "en"):
        d = {}
        with open(os.path.join(REPO, "groundstation", "lang", lg + ".txt"), encoding="utf-8") as f:
            for line in f:
                if "=" in line and not line.lstrip().startswith("#"):
                    k, v = line.split("=", 1)
                    d[k.strip()] = v.strip()
        lang[lg] = d
    bad = [k for k, (el, en) in P.WHY.items() if lang["el"].get(k) != el or lang["en"].get(k) != en]
    report("HP14 the explanation keys in lang el / en", not bad, "missing / different %s" % bad)


# --- HP15 the real tracker + simulated sensors ----------------------------------------------------------------------
def closed_loop_real(objects, secs, seed=1, corr=20.0, camera=False, route=4.0, planner_mod=P, health=None,
                     on=None):
    import test_harbour_tracker as TT
    boat = Boat(0.0, 0.0, 0.0, route, corr, route)
    sm = staticmap.StaticMap(origin=(TT.LAT0, TT.LON0))
    sm.add_segment(-200.0, 400.0, -190.0, 400.0, 2.0)          # a chart far away: the map exists (U10 off)
    sim = TT.Sim(objects, own=lambda t: (boat.x, boat.y, boat.hdg), seed=seed, camera=camera, static=sm)
    if health:
        sim.health.update(health)
    if on:
        sim.on.update(on)
    pl = planner_mod.Planner()
    log = []
    rec = {"min_gap": {}, "turns": [], "xte_max": 0.0, "stops": 0, "decs": []}
    for k in range(int(secs / 0.2)):
        f = sim.step(0.2)
        f = dict(f or {}, cycle_age_s=0.0)
        t = sim.t - TT.T0
        dec = pl.plan(t, {"own": boat.own(), "leg": boat.leg, "corr_m": corr, "route_kn": route,
                          "tracks": sim.tracks(), "flags": f, "static": sm, "health": dict(sim.health), "block": []})
        rec["decs"].append((dec["stop"], dec["heading"], dec["cap_kn"]))
        if dec["stop"]:
            rec["stops"] += 1
        if dec.get("turn"):
            rec["turns"].append((round(t, 1), dec["turn_for"], [tr["steer_ok"] for tr in sim.tracks()
                                                                  if str(tr["id"]) in dec["turn_for"]]))
        boat.step(dec)
        if not boat.hold:
            rec["xte_max"] = max(rec["xte_max"], abs(boat.xte()))
            for o in sim.w.truth(sim.t):
                g = hull_gap(boat, o)
                if g < rec["min_gap"].get(o["id"], (1e9,))[0]:
                    rec["min_gap"][o["id"]] = (round(g, 2), o["speed"] > 0.5)
        log.append(dec)
    rec["boat"] = boat
    return rec


def hp15():
    import test_harbour_tracker as TT
    kay = [TT.vessel("kayak", (1.5, 90.0), (1.5, 91.0), 0.0, ln=4.0, beam=0.8, h=0.4, rcs=0.2, refl=0.5,
                     kind="kayak")]
    r = closed_loop_real(kay, 60.0, seed=3)
    g = r["min_gap"].get("kayak", (1e9,))[0]
    turned_ok = all(all(s) for _t, _f, s in r["turns"])
    avoided = g >= 8.0 - 0.5
    # the camera only (radar + LiDAR off - their health LOST): never a turn, STOP
    r2 = closed_loop_real(kay, 30.0, seed=3, camera="all", on={"radar": False, "lidar": False},
                          health={"radar": "LOST", "lidar": "LOST"})
    cam_ok = not r2["turns"] and r2["stops"] > 0
    a = closed_loop_real(kay, 20.0, seed=5)["decs"]
    b = closed_loop_real(kay, 20.0, seed=5)["decs"]
    report("HP15 real tracker + simulated sensors: LiDAR kayak avoided, camera only never turned for, deterministic",
           avoided and turned_ok and cam_ok and a == b and r["xte_max"] <= 20.0,
           "kayak gap %.1f m hold %s turns %d (all steer_ok %s), camera-only turns %d stops %d, same %s"
           % (g, r["boat"].hold, len(r["turns"]), turned_ok, len(r2["turns"]), r2["stops"], a == b))


# --- HP16 mutations ------------------------------------------------------------------------------------------------
def hp16():
    import importlib
    import types
    src = open(os.path.join(HERE, "harbour", "planner.py"), encoding="utf-8").read()

    def mutant(old, new):
        assert old in src, old
        m = types.ModuleType("harbour.planner_mut")
        m.__package__ = "harbour"
        exec(compile(src.replace(old, new), "planner_mut", "exec"), m.__dict__)
        return m
    importlib.import_module("harbour.colregs")
    m1 = mutant('if t is None or not t.get("steer_ok"):', 'if t is None:')
    a1, n1, _ = hp6(m1)
    m2 = mutant('                stop.append(u)                          # U1', '                pass  # U1')
    o2, _s, _u6, _u8 = hp8(m2)
    m3 = mutant("m = math.hypot(px - x, py - y) - r - half - mm", "m = 1e9")
    m3b = mutant("m = math.hypot(px - x, py - y) - r - half - mm", "m = math.hypot(px - x, py - y) - r - half")
    b3, _d = hp10(m3)
    b3b, _d = hp10(m3b)
    m4 = mutant('        cap = min(float(inp.get("route_kn") or 0.0), float(c["max_kn"]))',
                '        cap = float(inp.get("route_kn") or 0.0)')
    m5 = mutant('        usable = corr - 0.5 * float(c["own_beam_m"]) - 1.0 - float(c["edge_margin_m"])',
                '        usable = corr + 6.0')
    b4, b5 = hp9(m4, quiet=True), hp9(m5, quiet=True)
    ok = (not n1) and (not all(o2.values())) and bool(b3) and bool(b3b) and bool(b4) and bool(b5)
    report("HP16 mutations: no steer_ok gate / no unsure STOP / no distance check -> caught", ok,
           "steer_ok gate removed: camera turn caught %s; unsure: %d not stopped; distance removed: %d / %d failures; "
           "cap removed: %d, corridor widened: %d" % (not n1, sum(1 for v in o2.values() if not v), len(b3), len(b3b),
                                                     len(b4), len(b5)))


def main():
    t0 = time.time()
    for fn in (hp1, hp2, hp3, hp4, hp5, hp6_report, hp7, hp8_report, hp9, hp10_report, hp11, hp12, hp13, hp14, hp15,
               hp16):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "exception %r" % e)
    n_ok = sum(results)
    print("\n%d / %d passed (%.1f s)" % (n_ok, len(results), time.time() - t0), flush=True)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
