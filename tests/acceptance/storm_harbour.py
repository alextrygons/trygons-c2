"""The harbour AVOIDANCE STORM (docs/harbour_design.md 9.2, step HX5): randomized harbour scenarios, closed loop, the
REAL gateway modules (not changed): the HX3 simulated radar + LiDAR + the HX4 tracker + the HX5 planner, driving a
kinematic boat that follows the planner the way the AURIX validates it (gateway/test_harbour_planner.Boat: the
60 deg check, the 8 deg/s slew, the corridor HOLD at corr + 5 m, STOP = HOLD latched). The AURIX side of the
invariants (I85, I86, I89, I91, I92: the cap, the corridor clamp, STANDBY / e-stop / joystick win, the lease HOLD) is
the C fault storm's (storm.c, step HX2); this file checks the PLANNER's own invariants, each by its own rule:

  I86h the planner's cap never above min(the leg speed, 5 kn), never above degraded_kn (2) with a sensor degraded
       (recomputed here from the tracker flags), never a proposal that raises the speed above the route's
  I87  a turn AWAY from the line only with a steer_ok (radar / LiDAR confirmed) track within reach (own rule on the
       tracker output: within 200 m of the boat - ahead, crossing or an overtaker astern)
  I88  never closer than 15 m (moving) / 8 m (static) hull to hull to a world object the sensors SAW (a confirmed /
       camera / AIS track within 10 m + its size in the last 5 s), except while a HOLD is in force; a HOLD for a
       static object must begin with room for the coast-down (gap >= min - 0.5 m)
  I89h never further off the line than the corridor half-width while not in HOLD
  I90  every unsure reason that is not U2 / U6 / U9 (judged here from the tracker flags) = STOP in the same cycle;
       radar + LiDAR both lost (health LOST, injected) = STOP in the same cycle

    python tests/acceptance/storm_harbour.py [--episodes N] [--seed S] [--secs 60] [--mutate steer_ok|unsure|distance]
    python tests/acceptance/storm_harbour.py --mutation-check      (each mutation must FAIL the storm)
Every run prints its seed; a failure prints the seed to replay (--seed S --episodes 1)."""
import argparse
import math
import os
import random
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "gateway"))
sys.path.insert(0, os.path.join(REPO, "groundstation"))
from harbour import planner as P, staticmap  # noqa: E402
import test_harbour_tracker as TT  # noqa: E402 - the in-process sensor simulators + tracker (test tooling)
import test_harbour_planner as TP  # noqa: E402 - the Boat (AURIX-like follower) + hull_gap

RES = {}
STATS = {"episodes": 0, "decisions": 0, "stops": 0, "turns": 0, "turns_steer": 0, "camera_eps": 0, "faults": 0,
         "passed_obstacle": 0}
NAMES = {"I86h": "planner cap <= min(leg, 5 kn), <= 2 kn degraded, never raised",
         "I87": "a turn away from the line only with a radar / LiDAR confirmed track ahead",
         "I88": "15 m moving / 8 m static kept to every seen object (or HOLD in time)",
         "I89h": "never outside the corridor while not in HOLD",
         "I90": "every STOP-class unsure reason / both sensors lost = STOP in the same cycle"}


def res(inv, ok, seed, text=""):
    r = RES.setdefault(inv, {"checks": 0, "fails": 0, "first": None})
    r["checks"] += 1
    if not ok:
        r["fails"] += 1
        if r["first"] is None:
            r["first"] = (seed, text)


def world(rng):
    objs = []
    kinds = rng.sample(["headon", "cross_stbd", "cross_port", "overtake", "kayak", "swimmer", "buoy", "moored",
                        "moored2", "fast_overtaker"], rng.randint(1, 4))
    for k in kinds:
        if k == "headon":
            x = rng.uniform(-30.0, 30.0)
            objs.append(TT.vessel("headon", (x, rng.uniform(150, 260)), (x, -500.0), rng.uniform(1.0, 3.0),
                                  ln=rng.uniform(6, 15), beam=3.0, h=2.0, rcs=0.8))
        elif k == "cross_stbd":
            y = rng.uniform(60, 160)
            objs.append(TT.vessel("xs", (rng.uniform(80, 160), y), (-300.0, y), rng.uniform(1.0, 2.5), ln=8.0, rcs=0.7))
        elif k == "cross_port":
            y = rng.uniform(60, 160)
            objs.append(TT.vessel("xp", (-rng.uniform(80, 160), y), (300.0, y), rng.uniform(1.0, 2.5), ln=8.0,
                                  rcs=0.7))
        elif k == "overtake":
            x = rng.uniform(-5.0, 5.0)
            objs.append(TT.vessel("slow", (x, rng.uniform(40, 90)), (x, 900.0), rng.uniform(0.4, 1.0), ln=6.0,
                                  rcs=0.6))
        elif k == "fast_overtaker":
            x = rng.uniform(-20.0, 20.0)
            objs.append(TT.vessel("fast", (x, -rng.uniform(60, 120)), (x, 900.0), rng.uniform(3.0, 4.0), ln=9.0,
                                  rcs=0.8))
        elif k == "kayak":
            y = rng.uniform(50, 140)
            objs.append(TT.vessel("kayak", (rng.uniform(-30, 30), y), (rng.uniform(-30, 30), y + rng.uniform(-20, 20)),
                                  rng.uniform(0.0, 1.0), ln=4.0, beam=0.8, h=0.4, rcs=0.2, refl=0.5, kind="kayak"))
        elif k == "swimmer":
            y = rng.uniform(40, 120)
            objs.append(TT.vessel("swimmer", (rng.uniform(-8, 8), y), (rng.uniform(-8, 8), y + 5.0), 0.3, ln=0.6,
                                  beam=0.5, h=0.3, rcs=0.05, refl=0.2, kind="swimmer"))
        elif k == "buoy":
            objs.append({"id": "buoy", "kind": "buoy", "x": rng.uniform(-12, 12), "y": rng.uniform(50, 150),
                         "len": 1.0, "beam": 1.0, "hdg": 0.0, "h": 1.2, "rcs": 0.4, "refl": 0.6})
        elif k in ("moored", "moored2"):
            sx = rng.choice((-1.0, 1.0))
            objs.append({"id": k, "kind": "moored", "x": sx * rng.uniform(12, 30), "y": rng.uniform(40, 160),
                         "len": 10.0, "beam": 3.5, "hdg": 0.0, "h": 2.0, "rcs": 0.7, "refl": 0.5})
    return objs


def stop_class(flags):
    """Own rule: the reasons that must STOP (everything but U2 / U6 / U9 slow-first)."""
    out = []
    for u in flags.get("unsure") or []:
        if not (str(u).startswith("U2 ") or str(u).startswith("U6 ") or str(u).startswith("U9 ")):
            out.append(u)
    return out


def episode(seed, secs, planner_mod=P, camera_p=0.3):
    rng = random.Random(seed)
    objs = world(rng)
    corr = rng.choice([8.0, 10.0, 10.0, 14.0, 20.0])
    route = rng.choice([2.5, 3.5, 4.0, 5.0, 6.0])        # 6 kn: the planner caps at 5
    camera = rng.random() < camera_p
    boat = TP.Boat(0.0, 0.0, 0.0, min(route, 5.0) * rng.uniform(0.3, 1.0), corr, route)
    sm = staticmap.StaticMap(origin=(TT.LAT0, TT.LON0))
    sm.add_segment(-300.0, 600.0, -290.0, 600.0, 2.0)
    sim = TT.Sim(objs, own=lambda t: (boat.x, boat.y, boat.hdg), seed=seed, camera="all" if camera else False,
                 static=sm)
    pl = planner_mod.Planner()
    # faults: a sensor lost / both lost / degraded / clutter / time skew at a random time
    fault = rng.choice([None, None, "radar", "lidar", "both", "degraded", "clutter", "skew"])
    f_at = rng.uniform(5.0, secs * 0.8)
    STATS["episodes"] += 1
    if camera:
        STATS["camera_eps"] += 1
    if fault:
        STATS["faults"] += 1
    seen = {}
    lost_at = {}
    leg_psi = 0.0
    steps = int(secs / 0.2)
    for k in range(steps):
        t_rel = (k + 1) * 0.2
        for kk, ta in lost_at.items():
            if t_rel >= ta:
                sim.health[kk] = "LOST"
        if fault and abs(t_rel - f_at) < 0.1:
            # review S-MED-3: the sensor goes SILENT; LOST only after the HealthJudge's lost_s (radar 4 s, LiDAR 1 s)
            if fault in ("radar", "both"):
                sim.on["radar"] = False
                lost_at["radar"] = t_rel + 4.0
            if fault in ("lidar", "both"):
                sim.on["lidar"] = False
                lost_at["lidar"] = t_rel + 1.0
            if fault == "degraded":
                sim.faults["radar"] = {"degraded": True}
                sim.health["radar"] = "DEGRADED"
            if fault == "clutter":
                sim.faults["lidar"] = {"clutter": True}
            if fault == "skew":
                sim.skews["lidar"] = 0.25
        f0 = sim.step(0.2)
        res("I90", isinstance(f0, dict) and "unsure" in f0, seed, "t %.1f the tracker gave no flags" % t_rel)
        f = dict(f0 or {}, cycle_age_s=0.0)
        t = sim.t - TT.T0
        tracks = sim.tracks()
        own = boat.own()
        dec = pl.plan(t, {"own": own, "leg": boat.leg, "corr_m": corr, "route_kn": route, "tracks": tracks,
                          "flags": f, "static": sm, "health": dict(sim.health), "block": []})
        inj = rng.random()                              # review S-MED-3: stale tracker / no pose / blocked input,
        if inj < 0.05:                                  # judged by a separate planner (the boat keeps its own)
            fi, oi, bi = dict(f), dict(own), []
            if inj < 0.02:
                fi["cycle_age_s"] = 0.8
            elif inj < 0.04:
                oi["fresh"] = False
            else:
                bi = ["the boat's route is not the one the gateway checked"]
            di = planner_mod.Planner().plan(t, {"own": oi, "leg": boat.leg, "corr_m": corr, "route_kn": route,
                                                "tracks": tracks, "flags": fi, "static": sm,
                                                "health": dict(sim.health), "block": bi})
            res("I90", di["stop"], seed, "t %.1f injected stale / no pose / blocked -> stop %s" % (t, di["stop"]))
        STATS["decisions"] += 1
        # I90
        sc = stop_class(f)
        both = sim.health["radar"] == "LOST" and sim.health["lidar"] == "LOST"
        if sc or both:
            res("I90", dec["stop"], seed, "t %.1f unsure %s both lost %s -> stop %s" % (t, sc, both, dec["stop"]))
        if dec["stop"]:
            STATS["stops"] += 1
        else:
            # I86h
            lim = min(route, 5.0)
            if f.get("degraded") or any(str(u).startswith(("U2 ", "U9 ")) for u in f.get("unsure") or []):
                lim = min(lim, 2.0)
            res("I86h", dec["cap_kn"] <= lim + 1e-9, seed, "t %.1f cap %.2f > %.2f" % (t, dec["cap_kn"], lim))
            # I87: away from the line = the proposal's heading moves |xte| up by more than the turn tolerance
            e = boat.xte()
            off = P.wrap(dec["heading"] - leg_psi)
            away = (off > 3.0 and e >= -0.5) or (off < -3.0 and e <= 0.5)
            if away:
                STATS["turns"] += 1
                ok = False
                for tr in tracks:
                    if not tr.get("steer_ok") or not set(tr.get("src") or "") & {"R", "L"}:
                        continue
                    if math.hypot(tr["x"] - boat.x, tr["y"] - boat.y) <= 200.0:
                        ok = True
                        break
                # review S-MED-1: the CAUSE - every track the planner turned for is steer_ok in the tracker's
                # output with a FRESH range hit (radar <= 2 s, LiDAR <= 0.5 s on the tracker clock)
                trk = {str(x["id"]): x for x in tracks}
                for i in dec.get("turn_for") or []:
                    x = trk.get(str(i))
                    last = (x or {}).get("last") or {}
                    fresh = x is not None and ((last.get("R") is not None and sim.tr.T - last["R"] <= 2.0) or
                                               (last.get("L") is not None and sim.tr.T - last["L"] <= 0.5))
                    if i != "static-map" and not (x is not None and x.get("steer_ok") and fresh):
                        ok = False
                if ok:
                    STATS["turns_steer"] += 1
                res("I87", ok, seed, "t %.1f a turn %.1f deg with no steer_ok track ahead (tracks %s)"
                    % (t, off, [(x["id"], x["src"], x["steer_ok"]) for x in tracks][:6]))
        was_hold = boat.hold
        boat.step(dec)
        # "seen": a confirmed range track or a camera / AIS track near the object
        truth = sim.w.truth(sim.t)
        for o in truth:
            if o["kind"] == "pier":
                continue
            sz = 0.5 * max(o["len"], o["beam"])
            for tr in tracks:
                if (tr.get("confirmed") or not (tr.get("last") or {}).get("R") and not (tr.get("last") or {}).get(
                        "L")) and math.hypot(tr["x"] - o["x"], tr["y"] - o["y"]) <= 10.0 + sz:
                    seen[o["id"]] = t
                    break
        # I89h
        if not boat.hold:
            res("I89h", abs(boat.xte()) <= corr + 1e-6, seed, "t %.1f xte %.1f > corr %.0f" % (t, boat.xte(), corr))
        # I88
        for o in truth:
            if o["kind"] == "pier" or t - seen.get(o["id"], -1e9) > 5.0:
                continue
            moving = o["speed"] > 0.5 or (o["kind"] == "vessel" and o["speed"] > 0.2)
            need = 15.0 if moving else 8.0
            g = TP.hull_gap(boat, o)
            if boat.hold and was_hold:
                if not moving and boat.v > 0.05:
                    # review S-MED-2: during the HOLD's coast-down WE close on a static object: the HOLD came too late
                    res("I88", g >= need - 0.5, seed, "t %.1f coasting in HOLD %.1f m from %s (need %.0f)"
                        % (t, g, o["id"], need))
                continue                                    # a HOLD in force: the closing comes from the object
            if boat.hold and not was_hold:
                if o["speed"] <= 0.5:
                    res("I88", g >= need - 0.5, seed, "t %.1f HOLD began %.1f m from %s (need %.0f)"
                        % (t, g, o["id"], need))
                continue
            res("I88", g >= need - 0.5, seed, "t %.1f %.1f m from %s (need %.0f)" % (t, g, o["id"], need))
    if not boat.hold and boat.y > 40.0:                 # reach: the boat made way (not stopped / held at once)
        STATS["passed_obstacle"] += 1


def mutant(old, new):
    src = open(os.path.join(REPO, "gateway", "harbour", "planner.py"), encoding="utf-8").read()
    assert old in src, old
    m = types.ModuleType("harbour.planner_mut")
    m.__package__ = "harbour"
    exec(compile(src.replace(old, new), "planner_mut", "exec"), m.__dict__)
    return m


MUTANTS = {
    "steer_ok": ('if t is None or not t.get("steer_ok"):', 'if t is None:'),
    "unsure": ('                stop.append(u)                          # U1', '                pass  # U1'),
    "distance": ("m = math.hypot(px - x, py - y) - r - half - mm", "m = math.hypot(px - x, py - y) - r - half"),
}
# (the cap / corridor mutants are caught by test_harbour_planner.py HP16 through HP9's 300 random plans; in the closed
#  loop the AURIX-like boat clamps them again, so the storm cannot see them - review LOW-3)


def run(episodes, seed, secs, planner_mod=P, quiet=False, camera_p=0.3):
    RES.clear()
    for k in STATS:
        STATS[k] = 0
    t0 = time.time()
    for k in range(episodes):
        episode(seed + k, secs, planner_mod, camera_p)
    bad = 0
    if not quiet:
        print("\n=== HARBOUR AVOIDANCE STORM: %d episodes (seeds %d..%d), %.0f s each, %.1f min ===" % (
            episodes, seed, seed + episodes - 1, secs, (time.time() - t0) / 60.0))
    for inv in sorted(RES):
        r = RES[inv]
        bad += r["fails"]
        if not quiet:
            print("%s %-5s %-75s checks %7d  fails %d" % ("FAIL" if r["fails"] else "PASS", inv, NAMES.get(inv, ""),
                                                          r["checks"], r["fails"]))
            if r["first"]:
                print("      first failing seed %s: %s" % r["first"])
    if not quiet:
        print("stats %s" % STATS)
    # reach: the storm must reach turns on confirmed tracks, STOPs, camera episodes and faults (else I87 / I90 could
    # never fail)
    if episodes >= 100:
        for key in ("turns_steer", "stops", "camera_eps", "faults", "passed_obstacle"):
            if STATS[key] == 0:
                bad += 1
                print("FAIL reach: %s never happened in %d episodes" % (key, episodes))
    for inv in NAMES:
        if episodes >= 100 and RES.get(inv, {}).get("checks", 0) == 0:
            bad += 1
            print("FAIL reach: %s never checked" % inv)
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--mutate", choices=list(MUTANTS))
    ap.add_argument("--mutation-check", action="store_true")
    a = ap.parse_args()
    if a.mutation_check:
        ok = True
        for name, (old, new) in MUTANTS.items():
            # the camera on in most episodes: the steer_ok gate matters only with camera-only / AIS-only tracks
            n = run(min(a.episodes, 40), a.seed, a.secs, mutant(old, new), quiet=True, camera_p=0.9)
            print("mutation %-9s -> storm %s (%d failures)" % (name, "FAILS (good)" if n else "PASSES (BAD)", n))
            ok = ok and n > 0
        print("MUTATION CHECK: %s" % ("OK" if ok else "FAILED"))
        return 0 if ok else 1
    mod = mutant(*MUTANTS[a.mutate]) if a.mutate else P
    bad = run(a.episodes, a.seed, a.secs, mod)
    print("HARBOUR STORM: %s" % ("OK" if not bad else "FAILED"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
