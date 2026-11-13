"""HX7: the harbour scenario runner (docs/harbour_design.md 9.1) - END TO END against the REAL AURIX simulator + the real
gateway modules (rig.py). NO app/ hal/ planner / tracker change: it only builds worlds, sends the captain's requests
and watches.

    python tests/harbour/run_scenarios.py list
    python tests/harbour/run_scenarios.py run H4 H11           one or more scenarios (each in its OWN process)
    python tests/harbour/run_scenarios.py all [--jobs 3]       H1-H20
    python tests/harbour/run_scenarios.py storm [--episodes 30] [--seed 1] [--jobs 3]   randomized worlds, the common criteria
    python tests/harbour/run_scenarios.py quick [--jobs 3]     = all + storm 30 (the check.sh set)

Every episode runs in a child process started in a KILL_ON_JOB_CLOSE job (tools/strays.py Job: a killed runner never
leaves a simulator or a sensor worker behind) with a time limit (EPISODE_LIMIT_S); the runner ends with a strays check
of its own. Exit code 0 = every scenario passed. Results (JSON) in test_results/harbour/ (gitignored)."""
import argparse
import json
import os
import random
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "tools"))
EPISODE_LIMIT_S = float(os.environ.get("TRYGONS_HARBOUR_EPISODE_S", "300"))
RUN_LIMIT_S = float(os.environ.get("TRYGONS_HARBOUR_RUN_S", "1500"))        # the whole run (25 min)
RESULTS = os.path.join(REPO, "test_results", "harbour")


def storm_spec(seed):
    """A seeded random harbour world (1-3 objects + maybe a sensor fault): the same seed = the same world."""
    import scenarios as sc
    rng = random.Random(seed)
    objs = []
    for k in rng.sample(["headon", "cross_s", "cross_p", "slow", "fast", "kayak", "buoy", "moored"],
                        rng.randint(1, 3)):
        at = rng.uniform(30.0, 40.0)
        if k == "headon":
            objs.append(sc.vessel("headon", (rng.uniform(55, 85), rng.uniform(-6, 6)), at, 270.0,
                                  rng.uniform(1.0, 3.0), ais=rng.choice([None, 237000100 + seed % 100])))
        elif k == "cross_s":
            objs.append(sc.vessel("cross_s", (rng.uniform(55, 85), 0.0), at, 0.0, rng.uniform(1.0, 2.5)))
        elif k == "cross_p":
            objs.append(sc.vessel("cross_p", (rng.uniform(55, 85), 0.0), at, 180.0, rng.uniform(1.0, 2.5)))
        elif k == "slow":
            objs.append(sc.vessel("slow", (rng.uniform(55, 85), rng.uniform(-3, 3)), at, 90.0, rng.uniform(0.3, 0.9),
                                  ln=6.0))
        elif k == "fast":
            objs.append(sc.vessel("fast", (rng.uniform(40, 80), rng.uniform(-6, 6)), at, 90.0, rng.uniform(3.0, 4.0)))
        elif k == "kayak":
            objs.append(sc.mover("kayak", "kayak", (rng.uniform(55, 85), 0.0), at, rng.choice([0.0, 180.0]),
                                 rng.uniform(0.6, 1.4), 4.0, 0.7, 0.4, 0.15, 0.5))
        elif k == "buoy":
            objs.append(sc.buoy("buoy", rng.uniform(60, 100), rng.uniform(-3, 3)))
        elif k == "moored":
            objs.append(sc.moored("moored", rng.uniform(30, 100), rng.choice([-1, 1]) * rng.uniform(15, 20)))
    faults = []
    ex = {"excused": (), "start_may_be_refused": 1}
    r = rng.random()
    if r < 0.15:
        faults.append({"t": rng.uniform(12, 30), "what": rng.choice(["radar_off", "lidar_off"])})
    elif r < 0.22:
        faults.append({"t": 20.0, "what": "planner_crash"})
    spec = sc.S("S%d" % seed, "storm seed %d: %s" % (seed, ", ".join(o["id"] for o in objs)), objs,
                route=[(0.0, 0.0, 5.0), (70.0, 0.0, 5.0), (130.0, 0.0, 5.0)], secs=75.0, world_delay_s=22.0,
                faults=faults, expect=ex)
    return spec


def run_one(name, out_path):
    """The child: one episode, its checks written to out_path (JSON)."""
    import scenarios as sc
    import rig
    spec = storm_spec(int(name[1:])) if name.startswith("S") and name[1:].isdigit() else sc.by_name()[name]
    import copy
    ep = rig.Episode(copy.deepcopy(spec))
    t_start = time.time()
    res = {"name": name, "title": spec["title"], "checks": [], "ok": False}
    try:
        out = ep.run()
        if out.get("tb") and "no fresh position" in out.get("why", ""):     # a start-up flake: ONE more try, said so
            ep.close()
            res["retried"] = "no fresh position at the start-up"
            ep = rig.Episode(copy.deepcopy(spec))
            out = ep.run()
        if out.get("tb"):
            res["checks"] = [("RIG", False, out["why"])]
            res["tb"] = out["tb"]
        else:
            res["checks"] = sc.evaluate(spec, ep, out)
        res["events"] = ep.events
        res["entry_gate"] = {"entry_before_first_start": getattr(ep, "entry_before", None),
                             "first_start_result": getattr(ep, "first_start", None),
                             "entry_after": getattr(ep, "entry_after", None)}
        res["trace"] = [tuple(round(v, 1) if isinstance(v, float) else v for v in s) for s in ep.samples[::10]]
        res["ok"] = bool(res["checks"]) and all(c[1] for c in res["checks"])
        res["xfail"] = spec.get("xfail")
    finally:
        ep.close()
    res["secs"] = round(time.time() - t_start, 1)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1, default=str)
    return res["ok"]


def run_parallel(names, jobs):
    import strays
    os.makedirs(RESULTS, exist_ok=True)
    job = strays.Job()
    pending = list(names)
    running = {}
    done = {}
    t_all = time.time()
    try:
        while pending or running:
            while pending and len(running) < jobs and time.time() - t_all < RUN_LIMIT_S:
                n = pending.pop(0)
                out = os.path.join(RESULTS, n + ".json")
                if os.path.exists(out):
                    os.remove(out)
                p = job.popen([sys.executable, os.path.abspath(__file__), "_one", n, out],
                              stdout=open(os.path.join(RESULTS, n + ".log"), "w"), stderr=subprocess.STDOUT,
                              stdin=subprocess.DEVNULL)
                running[n] = (p, time.time(), out)
            for n in list(running):
                p, t0, out = running[n]
                if p.poll() is not None:
                    done[n] = load(out, "child exited %s without a result" % p.returncode)
                    del running[n]
                elif time.time() - t0 > EPISODE_LIMIT_S:
                    job.kill(p)             # (kills the whole job: the other running episodes too - they are re-run)
                    done[n] = {"name": n, "ok": False, "checks": [("TIME", False, "episode over %d s - killed" %
                                                                   EPISODE_LIMIT_S)]}
                    del running[n]
                    for m, (pp, tt, oo) in list(running.items()):
                        pending.insert(0, m)
                        del running[m]
            if not running and pending and time.time() - t_all >= RUN_LIMIT_S:
                for n in pending:
                    done[n] = {"name": n, "ok": False, "checks": [("TIME", False, "run limit %d s: not started" %
                                                                   RUN_LIMIT_S)]}
                pending = []
            time.sleep(0.5)
    finally:
        job.kill()
        job.close()
    return done


def load(path, why):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"name": os.path.basename(path), "ok": False, "checks": [("RIG", False, why)]}


def report(done, order):
    fails = 0
    for n in order:
        r = done.get(n) or {"ok": False, "checks": [("RIG", False, "no result")]}
        ok = r.get("ok")
        xf = r.get("xfail")
        if xf and not ok:                       # a documented FINDING (reported to the owner): shown, not counted
            tag = "XFAIL"
        elif xf and ok:
            tag, ok = "FAIL", False             # the finding is fixed: remove the xfail mark (never a silent pass)
        else:
            tag = "PASS" if ok else "FAIL"
        fails += 0 if (ok or tag == "XFAIL") else 1
        print("[%s] %s - %s (%s s)" % (tag, n, r.get("title", ""), r.get("secs", "?")), flush=True)
        if xf:
            print("       finding: %s" % ("XPASS - the finding no longer shows: remove xfail from scenarios.py"
                                          if tag == "FAIL" else xf))
        for c in r.get("checks", []):
            if not c[1] or os.environ.get("TRYGONS_HARBOUR_VERBOSE"):
                print("       %s %s: %s" % ("ok  " if c[1] else "FAIL", c[0], c[2]))
        eg = r.get("entry_gate")
        if eg and n == order[0]:
            print("       note HX7-1: entry_ok before the first START = %s (first START result %s, then %s)" % (
                eg.get("entry_before_first_start"), eg.get("first_start_result"), eg.get("entry_after")))
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["list", "run", "all", "storm", "quick", "_one"])
    ap.add_argument("names", nargs="*")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    import scenarios as sc
    if a.cmd == "list":
        for s in sc.build():
            print("%-4s %s" % (s["name"], s["title"]))
        return 0
    if a.cmd == "_one":
        return 0 if run_one(a.names[0], a.names[1]) else 1
    if a.cmd == "run":
        names = a.names
    elif a.cmd == "all":
        names = [s["name"] for s in sc.build()]
    elif a.cmd == "storm":
        names = ["S%d" % (a.seed + i) for i in range(a.episodes)]
    else:
        names = [s["name"] for s in sc.build()] + ["S%d" % (a.seed + i) for i in range(a.episodes)]
    t = time.time()
    done = run_parallel(names, max(1, a.jobs))
    fails = report(done, names)
    print("harbour scenarios: %d run, %d failed, %.0f s" % (len(names), fails, time.time() - t), flush=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
