"""Part 4.1: fresh build from GitHub, following only MANUAL.txt / the READMEs.

    python tests/acceptance/fresh_build.py [--quick] [--url URL]

Clones the repository into an EMPTY folder (test_results/fresh_clone, not the
working folder) and, as a new user would (MANUAL chapter 3 + 4 + 4A):
builds the simulator, runs every test suite there, the TriCore compile check
(tools/check.sh), START_DEMO / STOP_DEMO; then lists anything that only works
on this PC: hard-coded paths, missing files, undocumented steps.
--quick: sim + bridge suites, charts, planner, 3 gateway sections, the driver
screen; full: tools/check.sh (everything) + all ground-station suites.
-> test_results/fresh_build.txt
"""
import argparse
import os
import re
import shutil
import socket
import stat
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "test_results")
CLONE = os.path.join(OUT, "fresh_clone")
PY = sys.executable
STEPS = []


def step(name, ok, detail, dt):
    STEPS.append((name, ok, detail, dt))
    print("%s %-45s %6.0f s  %s" % ("PASS" if ok else "FAIL", name, dt, detail[:200]), flush=True)


def run(args, cwd, timeout=7200, stdin_null=True):
    t0 = time.time()
    try:
        r = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout,
                           stdin=subprocess.DEVNULL if stdin_null else None)
        return r.returncode, (r.stdout or "") + (r.stderr or ""), time.time() - t0
    except subprocess.TimeoutExpired as e:
        return -1, "TIMEOUT %s" % e, time.time() - t0
    except OSError as e:                                    # e.g. the program was not built
        return -2, "cannot start %s: %s" % (args[0], e), time.time() - t0


def last(text, pat):
    m = [l for l in text.splitlines() if re.search(pat, l)]
    return m[-1].strip() if m else "(no summary line)"


def rm_tree(p):
    def onerr(fn, path, exc):
        os.chmod(path, stat.S_IWRITE)
        fn(path)
    if os.path.isdir(p):
        shutil.rmtree(p, onerror=onerr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--url")
    a = ap.parse_args()
    url = a.url or subprocess.run(["git", "remote", "get-url", "origin"], cwd=REPO, capture_output=True,
                                  text=True).stdout.strip()
    rm_tree(CLONE)
    os.makedirs(OUT, exist_ok=True)
    rc, out, dt = run(["git", "clone", "--depth", "1", url, CLONE], OUT)
    step("git clone %s" % url, rc == 0, last(out, "Cloning|done|fatal") if rc else "cloned", dt)
    if rc:
        return report()
    head = subprocess.run(["git", "log", "--oneline", "-1"], cwd=CLONE, capture_output=True, text=True).stdout.strip()
    step("clone is at", True, head, 0.0)

    # MANUAL chapter 4: build + simulator tests
    rc, out, dt = run(["cmd", "/c", "sim\\build_sim.bat"], CLONE)       # (MSYS2 Python joins with "/")
    step("sim\\build_sim.bat", "BUILD OK" in out, last(out, "BUILD"), dt)
    sim_dir = os.path.join(CLONE, "sim")
    rc, out, dt = run([os.path.join(sim_dir, "autopilot_sim.exe"), "all"], sim_dir)
    step("autopilot_sim all", rc == 0, last(out, "TOTAL"), dt)
    rc, out, dt = run([os.path.join(sim_dir, "autopilot_sim.exe"), "all", "--bridge"], sim_dir)
    step("autopilot_sim all --bridge", rc == 0, last(out, "TOTAL"), dt)

    # MANUAL chapter 3 e: MediaMTX into tools\bin
    rc, out, dt = run([PY, os.path.join("tools", "get_mediamtx.py")], CLONE)
    step("python tools\\get_mediamtx.py", rc == 0, last(out, "MediaMTX|error|Error"), dt)

    gwd, gsd = os.path.join(CLONE, "gateway"), os.path.join(CLONE, "groundstation")
    if a.quick:
        rc, out, dt = run([PY, "test_charts.py"], gsd)
        step("groundstation\\test_charts.py", rc == 0, last(out, "TESTS"), dt)
        rc, out, dt = run([PY, "test_avoid.py"], gwd)
        step("gateway\\test_avoid.py", rc == 0, last(out, "TESTS"), dt)
        rc, out, dt = run([PY, "test_gateway.py", "F1", "SH", "VI"], gwd)
        step("gateway\\test_gateway.py F1 SH VI", rc == 0, last(out, "GATEWAY TESTS"), dt)
    else:
        # the gate: TriCore compile, memory, charts, planner, every gateway section
        rc, out, dt = run(["sh", os.path.join("tools", "check.sh")], CLONE, timeout=10800)
        with open(os.path.join(OUT, "fresh_check.txt"), "w", encoding="utf-8") as f:   # the whole gate output
            f.write(out)
        fails = [l.strip() for l in out.splitlines()
                 if re.search(r"\[FAIL\]|FAIL |FAILED|NOT FOUND|Error|error:", l) and "GATE" not in l]
        step("sh tools/check.sh (gate)", "GATE: OK" in out, "; ".join(fails[:6]) or last(out, "GATE"), dt)
        for t in ("test_e2e.py", "test_logs.py"):
            rc, out, dt = run([PY, t], gsd)
            step("groundstation\\" + t, rc == 0, last(out, "passed|failed"), dt)
    rc, out, dt = run([PY, "test_driver.py"], gsd)
    step("groundstation\\test_driver.py", rc == 0, last(out, "TESTS"), dt)

    # MANUAL chapter 4A: the demo from the clone
    busy = []
    for p in (5760, 5761, 5762, 5790, 8554):
        s = socket.socket()
        s.settimeout(0.3)
        if s.connect_ex(("127.0.0.1", p)) == 0:
            busy.append(p)
        s.close()
    if busy:
        step("START_DEMO.bat", False, "skipped: ports %s in use (another demo running?)" % busy, 0.0)
    else:
        rc, out, dt = run([PY, os.path.join("demo", "demo.py"), "start"], CLONE, timeout=300)
        ok = rc == 0 and "The demo is running" in out
        time.sleep(15)
        up = []
        for p in (5760, 5761, 5762, 5790, 8554):
            s = socket.socket()
            s.settimeout(0.5)
            up.append(s.connect_ex(("127.0.0.1", p)) == 0)
            s.close()
        rc2, out2, dt2 = run([PY, os.path.join("demo", "demo.py"), "stop"], CLONE, timeout=120)
        step("START_DEMO (demo.py start) from the clone", ok and all(up),
             "%s; ports up %s; %s" % (last(out, "charts|vision|started|did not|MISSING"), up, last(out2, "stopped")),
             dt + 15)

    # what only works on this PC
    hits = []
    pat = re.compile(r"C:\\\\?trygons|/c/trygons|alexs|TRYGONS_C2|AppData\\\\Local\\\\Temp|scratchpad", re.I)
    for d, _s, fs in os.walk(CLONE):
        if ".git" in d.split(os.sep) or "test_results" in d:
            continue
        for f in fs:
            if f.endswith((".py", ".c", ".h", ".bat", ".sh", ".json", ".ps1")):
                p = os.path.join(d, f)
                try:
                    for i, line in enumerate(open(p, encoding="utf-8", errors="replace"), 1):
                        if pat.search(line):
                            hits.append("%s:%d: %s" % (os.path.relpath(p, CLONE), i, line.strip()[:120]))
                except OSError:
                    pass
    step("no paths that exist only on this PC", not hits, "; ".join(hits[:8]) or "none found", 0.0)
    missing = []
    for want, why in (("charts/attica/area.json", "real depth charts (tools/chartprep.py, not in git)"),
                      ("gateway/vision/models/ewasr_resnet18.onnx", "vision model (tools/get_vision_model.py)"),
                      ("tools/bin/vision-python/python.exe", "vision Python (tools/get_vision_python.py)")):
        if not os.path.exists(os.path.join(CLONE, want)):
            missing.append("%s - %s" % (want, why))
    step("not in git, downloaded / made by documented tools", True, "; ".join(missing) or "-", 0.0)
    return report()


def report():
    bad = sum(1 for s in STEPS if not s[1])
    lines = ["=== FRESH BUILD FROM GITHUB (%d steps, %d failed) ===" % (len(STEPS), bad), "",
             "| step | result | time | detail |", "|---|---|---|---|"]
    for name, ok, detail, dt in STEPS:
        lines.append("| %s | %s | %.0f s | %s |" % (name, "PASS" if ok else "FAIL", dt, detail.replace("|", "/")[:300]))
    with open(os.path.join(OUT, "fresh_build.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
