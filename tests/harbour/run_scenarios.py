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


# HX7-fix3 (owner answer 2, 2026-11-13): KNOWN LIMITS = an episode that is EXPECTED to FAIL exactly the listed check(s)
# for a stated design reason. It stays RED in substance: printed as [KNOWN LIMIT] with its reason EVERY run, any OTHER
# failed check = FAIL, and an unexpected PASS = FAIL (the limit must be re-judged, never a silent pass) - exactly like
# the XFAILs. The criterion itself is unchanged.
KNOWN_LIMITS = {
    "S4": {"checks": ("C1 never touches while the boat moves",),
           # review M1: the limit is narrow - every contact while moving is with THIS overtaker, in HOLD, after the
           # U13 not-yielding STOP, and the world is still the one judged (a changed storm_spec(4) = FAIL)
           "object": "fast", "fingerprint": "43445bc87cb700e6",
           "reason": "KNOWN LIMIT S4 (owner 2026-11-13, NOT 'unavoidable'): an overtaking vessel that does not give way "
                     "hits the boat ~12 s after the not-yielding STOP while it still coasts at ~0.5 kn in HOLD - a "
                     "stopped boat coasting in HOLD cannot avoid a vessel overtaking from astern; the captain takes over "
                     "(FIRST_TEST.md 7, MANUAL ch. 27). The design answer (COLREGs 13 / 17: keep speed or offset to "
                     "starboard instead of STOP) is the owner's open design question in PROGRESS.md WAITING FOR OWNER"},
}


def known_limit_evidence(name, spec, ep):
    """review M1: is a failure of a KNOWN_LIMITS episode exactly the documented limit? -> (ok, detail)."""
    import hashlib
    kl = KNOWN_LIMITS[name]
    fp = hashlib.sha256(json.dumps(spec, sort_keys=True, default=str).encode()).hexdigest()[:16]   # r2 M-2: the WHOLE spec
    S_ = ep.samples
    u13 = [q[0] for q in S_ if len(q) > 19 and q[19] and "NOT GIVING WAY - STOP" in (q[14] or "")]
    # r2 M-1: contacts on the estimated AND the true position (q[22]), the true position in >= 90 % of the samples
    cont = [(q[0], o, q[5], round(q[4], 2)) for q in S_ if q[4] >= 0.5 for o, g, _m in q[17] if g < 0.5]
    cont += [(q[0], o, q[5], round(q[4], 2)) for q in S_ if q[4] >= 0.5 and len(q) > 22 and q[22]
             for o, g in q[22] if g < 0.5]
    n_t = len([q for q in S_ if len(q) > 22 and q[22]])
    bad = [c for c in cont if c[1] != kl["object"] or c[2] != "HOLD" or not u13 or c[0] < u13[0]]
    ok = fp == kl["fingerprint"] and bool(cont) and not bad and n_t >= 0.9 * len(S_)
    return ok, "world %s (want %s); U13 at %s; contacts while moving %s; NOT the limit %s; true %d / %d" % (
        fp, kl["fingerprint"], u13[0] if u13 else None, cont[:2], bad[:2], n_t, len(S_))


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
                             "presses": len([1 for t, _p in getattr(ep, "presses", []) or []])}
        res["trace"] = [tuple(round(v, 1) if isinstance(v, float) else v for v in s) for s in ep.samples[::10]]
        res["ok"] = bool(res["checks"]) and all(c[1] for c in res["checks"])
        res["xfail"] = spec.get("xfail")
        res["class"] = out.get("class")
        res["stops"] = getattr(ep, "stops", [])[:60]
        if name in KNOWN_LIMITS and not out.get("tb"):
            res["kl_evidence"] = known_limit_evidence(name, spec, ep)
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


def report(done, order, known=None):
    fails = 0
    known = known if known is not None else []
    for n in order:
        r = done.get(n) or {"ok": False, "checks": [("RIG", False, "no result")]}
        ok = r.get("ok")
        xf = r.get("xfail")
        kl = KNOWN_LIMITS.get(n)
        if kl:
            bad = [c[0] for c in r.get("checks", []) if not c[1]]
            if ok:
                tag, ok = "FAIL", False         # XPASS: the known limit no longer shows - re-judge it, never silent
                xf = "XPASS - the KNOWN LIMIT no longer shows (passed by chance? or fixed): re-judge KNOWN_LIMITS"
            elif (bad and all(any(b.startswith(k) for k in kl["checks"]) for b in bad)
                  and (r.get("kl_evidence") or [False])[0]):
                tag, xf = "KNOWN LIMIT", kl["reason"] + " | evidence: " + str(r["kl_evidence"][1])
            else:
                tag = "FAIL"                    # another check failed too: a real failure
                xf = kl["reason"] + " - BUT other checks failed too or the evidence is not the limit: %s" % (
                    (r.get("kl_evidence") or [None, "none"])[1],)
        elif xf and not ok:                     # a documented FINDING (reported to the owner): shown, not counted
            tag = "XFAIL"
        elif xf and ok:
            tag, ok = "FAIL", False             # the finding is fixed: remove the xfail mark (never a silent pass)
        else:
            tag = "PASS" if ok else "FAIL"
        fails += 0 if (ok or tag in ("XFAIL", "KNOWN LIMIT")) else 1
        if tag == "KNOWN LIMIT":
            known.append(n)
        print("[%s] %s - %s (%s s)" % (tag, n, r.get("title", ""), r.get("secs", "?")), flush=True)
        if kl:
            print("       %s" % xf)
        elif xf:
            print("       finding: %s" % ("XPASS - the finding no longer shows: remove xfail from scenarios.py"
                                          if tag == "FAIL" else xf))
        for c in r.get("checks", []):
            if not c[1] or os.environ.get("TRYGONS_HARBOUR_VERBOSE"):
                print("       %s %s: %s" % ("ok  " if c[1] else "FAIL", c[0], c[2]))
        eg = r.get("entry_gate")
        if eg and n == order[0]:
            print("       HX7-1 (fixed): START button enabled before any START = %s, the ONE press -> result %s, "
                  "T_HARBOUR frames sent %s" % (eg.get("entry_before_first_start"), eg.get("first_start_result"),
                                                eg.get("presses")))
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
    known = []
    fails = report(done, names, known)
    cls = {}
    for n in names:
        k = (done.get(n) or {}).get("class") or "-"
        cls[k] = cls.get(k, 0) + 1
    print("classes (HX7-fix2): %s" % ", ".join("%s %d" % kv for kv in sorted(cls.items())), flush=True)
    for n in known:
        print("KNOWN LIMIT %s (not a pass): %s" % (n, KNOWN_LIMITS[n]["reason"]), flush=True)
    print("harbour scenarios: %d run, %d failed, %d known limit(s) %s, %.0f s" % (len(names), fails, len(known), known,
                                                                                 time.time() - t), flush=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
