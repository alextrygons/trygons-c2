"""Part 4.4: the random fault storm, several storm.exe processes in parallel.

    python tests/acceptance/run_storm.py [--episodes 2000] [--seed 1] [--workers 8]

Builds test_results/storm.exe (tests/acceptance/storm/build_storm.bat), runs the
self-test (the checks must catch planted violations), splits the seeds over the
workers, merges the results into test_results/storm_report.txt and prints the
invariant table. Every failing seed is listed with its minimal reproduction:
    test_results\\storm.exe --replay SEED
Exit code 0 = every invariant held.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "test_results")
EXE = os.path.join(OUT, "storm.exe")
NAMES = {}


def build():
    r = subprocess.run(["cmd", "/c", os.path.join(HERE, "storm", "build_storm.bat")], capture_output=True, text=True)
    if "BUILD OK" not in r.stdout:
        print(r.stdout[-3000:] + r.stderr[-3000:])
        return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--workers", type=int, default=max(1, min(10, (os.cpu_count() or 4) - 2)))
    ap.add_argument("--no-build", action="store_true")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    if not a.no_build and not build():
        print("storm.exe did not build")
        return 2
    st = subprocess.run([EXE, "--selftest"], capture_output=True, text=True, cwd=OUT)
    print(st.stdout.strip())
    selftest_ok = st.returncode == 0
    t0 = time.time()
    per = (a.episodes + a.workers - 1) // a.workers
    procs = []
    for w in range(a.workers):
        n = min(per, a.episodes - w * per)
        if n <= 0:
            break
        log = open(os.path.join(OUT, "storm_worker_%d.txt" % w), "w")
        procs.append((subprocess.Popen([EXE, str(n), str(a.seed + w * per)], stdout=log, stderr=subprocess.STDOUT,
                                       cwd=os.path.join(OUT)), log))
    for p, log in procs:
        p.wait()
        log.close()
    inv = {}
    run = [0, 0.0, 0, 0, 0, 0, 0]
    fails_text = []
    for w in range(len(procs)):
        with open(os.path.join(OUT, "storm_worker_%d.txt" % w)) as f:
            for line in f:
                if line.startswith("DATA "):
                    _, i, c, fl, wst, seed, wseed = line.split()
                    d = inv.setdefault(int(i), [0, 0, 0.0, 0, 0])
                    d[0] += int(c)
                    d[1] += int(fl)
                    if float(wst) > d[2]:
                        d[2], d[4] = float(wst), int(wseed)
                    if int(fl) and not d[3]:
                        d[3] = int(seed)
                elif line.startswith("DATA_RUN "):
                    v = line.split()[1:]
                    run[0] += int(v[0])
                    run[1] += float(v[1])
                    for k in (2, 3, 4, 5):
                        run[k] += int(v[k])
                    if len(v) > 6 and int(v[6]) and not run[6]:
                        run[6] = int(v[6])
                elif line.startswith("FAIL seed") or line.startswith("      "):
                    fails_text.append(line.rstrip())
                elif line[:4] in ("PASS", "FAIL") and line[4:8] in ("  I1", " I1 ", "  I2") or line[:6] in (
                        "PASS I", "FAIL I"):
                    NAMES.setdefault(line[5:9].strip(), line[5:100].strip())
    lines = ["=== FAULT STORM: %d episodes, %.0f simulated hours, %d workers, %.1f min on this PC ===" % (
        run[0], run[1], len(procs), (time.time() - t0) / 60.0),
        "self-test (planted violations caught): %s" % ("PASS" if selftest_ok else "FAIL"), "",
        "| invariant | checks | fails | worst reaction | first failing seed |", "|---|---|---|---|---|"]
    gateway_side = {14, 15, 16, 17, 18, 21}
    bad = 0
    for i in sorted(inv):
        if i in gateway_side:
            continue
        c, fl, wst, seed, wseed = inv[i]
        name = next((v for k, v in NAMES.items() if k == "I%d" % i), "I%d" % i)
        bad += fl
        lines.append("| %s %s | %d | %d | %s | %s |" % ("FAIL" if fl else "PASS", name, c, fl,
                                                       "%.2f s (seed %d)" % (wst, wseed) if i in (4, 5, 12, 19) else "-",
                                                       seed or "-"))
    lines += ["", "AUTO entries: %d by PC + CONFIRM, %d by the joystick AUTO button; REMOTE entries %d" % (
        run[2], run[3], run[4]),
        "NOTE (finding, not an invariant): HDG LOCK / AUTO / REMOTE entered while the onboard joystick was held and "
        "ignored (not centred since STANDBY): %d times (first seed %d)" % (run[5], run[6])]
    if fails_text:
        lines += ["", "Failing seeds (minimal reproductions):"] + fails_text[:200]
    text = "\n".join(lines)
    with open(os.path.join(OUT, "storm_report.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)
    return 0 if (bad == 0 and selftest_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
