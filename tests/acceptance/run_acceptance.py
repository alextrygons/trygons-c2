"""The whole acceptance test with one command (Part 4.7).

    python tests/acceptance/run_acceptance.py quick     about 10-15 min
    python tests/acceptance/run_acceptance.py full      several hours
    python tests/acceptance/run_acceptance.py full --short-soak
                                       the same with a SHORTENED soak (4 simulated
                                       hours AURIX, 15 min system) - the full 24 h
                                       soak stays required before an uncrewed trip

quick: fresh clone from GitHub + the main suites (fresh_build.py --quick), the
       storm self-test, 200 fault-storm episodes, the time wrap, the day at sea
       (AURIX part), a short gateway storm.
full:  fresh clone + every suite (tools/check.sh), 2000 storm episodes, the
       24 h AURIX soak with logging, a 60 min system soak, the day at sea
       (both parts), the gateway + station storms, coverage.
ACCEPT_CLONE_URL (environment): where the fresh build clones from (default:
the GitHub remote) - e.g. this folder, to test commits that are not pushed yet.
Every random part prints its seed. Big outputs go to test_results/ (not in
git). Summary: test_results/acceptance_summary.txt (and on the screen).
How to read it: MANUAL.txt chapter "TESTING".
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "test_results")
PY = sys.executable
STORM = os.path.join(OUT, "storm.exe")


def plan(mode, short_soak=False):
    t = os.path.join(HERE, "%s")
    url = ["--url", os.environ["ACCEPT_CLONE_URL"]] if os.environ.get("ACCEPT_CLONE_URL") else []
    ads = ("ADS build excludes every non-firmware folder", [PY, t % "check_ads_excludes.py"], 60)
    ifx = ("ADS builds every iLLD file the firmware calls", [PY, t % "check_ads_ifx.py"], 60)
    q = [ads, ifx,
         ("fresh build from GitHub (quick)", [PY, t % "fresh_build.py", "--quick"] + url, 3600),
         ("fault storm: 200 episodes", [PY, t % "run_storm.py", "--episodes", "200"], 1800),
         ("32-bit time wraparound", [STORM, "--wrap"], 600),
         ("day at sea, AURIX part", [STORM, "--dayatsea"], 900),
         ("gateway storm (I14, I15, I17, I20, I21, I26)", [PY, t % "storm_gateway.py", "--episodes", "10"], 1800)]
    if mode == "quick":
        return q
    return [ads, ifx,
            ("fresh build from GitHub (full)", [PY, t % "fresh_build.py"] + url, 14400),
            ("fault storm: 2000 episodes", [PY, t % "run_storm.py", "--episodes", "2000"], 7200),
            ("32-bit time wraparound", [STORM, "--wrap"], 600),
            ("SHORTENED AURIX soak 4 h with logging" if short_soak else "24 h AURIX soak with logging",
             [STORM, "--soak", "4" if short_soak else "24", "24", "--log"], 7200),
            ("SHORTENED system soak 15 min" if short_soak else "system soak 60 min",
             [PY, t % "soak_system.py", "--minutes", "15" if short_soak else "60"], 7200),
            ("day at sea, AURIX part", [STORM, "--dayatsea"], 900),
            ("day at sea, whole system", [PY, t % "day_at_sea.py"], 5400),
            ("gateway storm (I14, I15, I17, I20, I21, I26)", [PY, t % "storm_gateway.py", "--episodes", "120"], 7200),
            ("station storm (I16)", [PY, t % "stations_storm.py", "--seconds", "300"], 1800),
            ("coverage", [PY, t % "coverage_run.py"], 14400)]


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "quick"
    short_soak = "--short-soak" in sys.argv[2:]
    if mode not in ("quick", "full"):
        print(__doc__)
        return 2
    os.makedirs(OUT, exist_ok=True)
    r = subprocess.run(["cmd", "/c", os.path.join(HERE, "storm", "build_storm.bat")], capture_output=True, text=True)
    if "BUILD OK" not in r.stdout:
        print("storm.exe did not build:\n" + r.stdout[-2000:])
        return 2
    # the system parts (soak, day at sea, station storm) run the simulator: build it here too
    # (a clean checkout has no sim\autopilot_sim.exe - it is not in git)
    r = subprocess.run(["cmd", "/c", os.path.join(REPO, "sim", "build_sim.bat")], capture_output=True, text=True)
    if "BUILD OK" not in r.stdout:
        print("autopilot_sim.exe did not build:\n" + r.stdout[-2000:])
        return 2
    t0 = time.time()
    res = []
    for name, args, tmo in plan(mode, short_soak):
        print("\n######## %s" % name, flush=True)
        log = os.path.join(OUT, "acc_%s.txt" % "".join(c if c.isalnum() else "_" for c in name)[:40])
        t1 = time.time()
        try:
            with open(log, "w", encoding="utf-8") as f:
                p = subprocess.run(args, cwd=OUT if args[0] == STORM else REPO, stdout=f, stderr=subprocess.STDOUT,
                                   timeout=tmo, stdin=subprocess.DEVNULL)
            rc = p.returncode
        except subprocess.TimeoutExpired:
            rc = -1
        dt = time.time() - t1
        tail = open(log, encoding="utf-8", errors="replace").read().strip().splitlines()[-3:]
        res.append((name, rc, dt, log, tail))
        print("%s  %s  (%.1f min)  log %s" % ("PASS" if rc == 0 else ("TIMEOUT" if rc == -1 else "FAIL"), name, dt / 60,
                                             os.path.relpath(log, REPO)), flush=True)
    lines = ["=== ACCEPTANCE TEST (%s%s), %.0f min, %s ===" % (mode, ", SHORTENED SOAK" if short_soak else "",
                                                          (time.time() - t0) / 60,
                                                          time.strftime("%Y-%m-%d %H:%M"))]
    for name, rc, dt, log, tail in res:
        lines.append("%-8s %-45s %6.1f min   %s" % ("PASS" if rc == 0 else ("TIMEOUT" if rc == -1 else "FAIL"), name,
                                                    dt / 60, os.path.relpath(log, REPO)))
        for t in tail:
            lines.append("           " + t[:160])
    lines.append("\nA FAIL is not always a bug: open its log; TEST_REPORT.md explains the known findings.")
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "acceptance_summary.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print("\n" + txt)
    return 0 if all(r[1] == 0 for r in res) else 1


if __name__ == "__main__":
    sys.exit(main())
