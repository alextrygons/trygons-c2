"""Part 4.6: test coverage - which code do the tests actually run?

    python tests/acceptance/coverage_run.py [--quick] [--skip-c] [--skip-py]

C (gcov): app/ + sim/ compiled again with --coverage into test_results/cov/, the
simulator tests (all + all --bridge), a fault storm and the day at sea run on it;
gcov -> lines / branches per file, every function that never ran.
Python (coverage.py, MSYS2: pacman -S mingw-w64-ucrt-x86_64-python-coverage):
the gateway tests (section by section), the planner / gateway storms, the
ground station tests -> lines per file, every function that never ran.
Then the SAFETY-RELEVANT functions that no test executes are listed.
-> test_results/coverage_summary.txt (+ test_results/coverage/html/index.html)
"""
import argparse
import ast
import glob
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "test_results")
COV = os.path.join(OUT, "cov")
PY = sys.executable
GCC = "gcc"

# safety-relevant code (Part 4.6): output writers, mode manager, gear, power limit, joystick,
# CONFIRM, monitor, fault handling, collision avoidance, shallow water, vision speed cap
SAFETY_C = ["servo.c", "modes.c", "gear.c", "powerlim.c", "throttle.c", "joystick.c", "joybridge.c", "monitor.c",
            "smallboat.c", "link.c", "dr.c", "slam.c", "autopilot.c"]
SAFETY_PY = ["gateway/stations.py", "gateway/avoid.py", "gateway/shallow.py", "gateway/vision/manager.py",
             "gateway/channels.py", "gateway/recorder.py", "gateway/gateway.py"]


def sh(args, cwd=REPO, timeout=7200):
    t0 = time.time()
    r = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    return r, time.time() - t0


# --------------------------------------------------------------------------- C --
def c_coverage(quick):
    obj = os.path.join(COV, "obj")
    os.makedirs(obj, exist_ok=True)
    for f in glob.glob(os.path.join(obj, "*.gcda")):
        os.remove(f)
    srcs = sorted(glob.glob(os.path.join(REPO, "app", "*.c"))) + sorted(glob.glob(os.path.join(REPO, "sim", "*.c")))
    srcs.append(os.path.join(REPO, "tests", "acceptance", "storm", "storm.c"))
    # the pure-C board code the sim uses too (data flash log, Modbus master, DroneCAN + libcanard)
    srcs += [os.path.join(REPO, "hal", f) for f in ("tc_nv.c", "tc_modbus.c", "dronecan.c", "can_act.c", "can_safe.c", "can_relay.c", "tc_pulse.c")]
    srcs.append(os.path.join(REPO, "hal", "libcanard", "canard.c"))
    for s in srcs:
        o = os.path.join(obj, os.path.splitext(os.path.basename(s))[0] + ".o")
        r, _ = sh([GCC, "-std=c99", "-O0", "-g", "--coverage", "-DCANARD_64_BIT=1", "-Iapp", "-Ihal", "-Isim", "-c", s,
                   "-o", o])
        if r.returncode:
            return ["C coverage: compile failed: %s\n%s" % (s, r.stderr[-1500:])], {}
    objs = [os.path.join(obj, os.path.splitext(os.path.basename(s))[0] + ".o") for s in srcs]
    sim_objs = [o for o in objs if not o.endswith("storm.o")]
    storm_objs = [o for o in objs if not o.endswith("sim_main.o")]
    exe_sim, exe_storm = os.path.join(COV, "sim_cov.exe"), os.path.join(COV, "storm_cov.exe")
    for exe, ob in ((exe_sim, sim_objs), (exe_storm, storm_objs)):
        r, _ = sh([GCC, "--coverage"] + ob + ["-o", exe, "-lm", "-lws2_32"])
        if r.returncode:
            return ["C coverage: link failed: %s" % r.stderr[-1500:]], {}
    runs = [([exe_sim, "all"], "sim all"), ([exe_sim, "all", "--bridge"], "sim all --bridge"),
            ([exe_storm, "60" if quick else "300", "1"], "fault storm"), ([exe_storm, "--dayatsea"], "day at sea A"),
            ([exe_storm, "--wrap"], "time wrap"), ([exe_storm, "--selftest"], "storm self-test")]
    log = []
    for args, name in runs:
        r, dt = sh(args, cwd=os.path.join(REPO, "sim"))
        log.append("  ran %-18s %5.0f s (exit %d)" % (name, dt, r.returncode))
    files = {}
    for s in sorted(glob.glob(os.path.join(REPO, "app", "*.c"))):
        r, _ = sh(["gcov", "-o", obj, "-b", "-f", "-n", s])
        cur = None
        fn = []
        for line in r.stdout.splitlines():
            m = re.match(r"Function '(.+)'", line)
            if m:
                cur = ("fn", m.group(1))
                continue
            m = re.match(r"File '(.+)'", line)
            if m:
                cur = ("file", m.group(1))
                continue
            m = re.match(r"Lines executed:([\d.]+)% of (\d+)", line)
            if m and cur:
                if cur[0] == "fn":
                    fn.append((cur[1], float(m.group(1)), int(m.group(2))))
                elif os.path.basename(cur[1]) == os.path.basename(s):
                    files.setdefault(os.path.basename(s), {})["lines"] = (float(m.group(1)), int(m.group(2)))
            m = re.match(r"Branches executed:([\d.]+)% of (\d+)", line)
            if m and cur and cur[0] == "file" and os.path.basename(cur[1]) == os.path.basename(s):
                files.setdefault(os.path.basename(s), {})["branches"] = (float(m.group(1)), int(m.group(2)))
            m = re.match(r"Taken at least once:([\d.]+)% of (\d+)", line)
            if m and cur and cur[0] == "file" and os.path.basename(cur[1]) == os.path.basename(s):
                files.setdefault(os.path.basename(s), {})["taken"] = (float(m.group(1)), int(m.group(2)))
        files.setdefault(os.path.basename(s), {})["unrun"] = [f for f, p, n in fn if p == 0.0 and n > 0]
    return log, files


# ---------------------------------------------------------------------- Python --
def py_coverage(quick):
    data = os.path.join(COV, "py")
    os.makedirs(data, exist_ok=True)
    for f in glob.glob(os.path.join(data, ".coverage*")):
        os.remove(f)
    rc = os.path.join(COV, "coveragerc")
    with open(rc, "w") as f:
        f.write("[run]\nparallel = True\ndata_file = %s\nsource =\n    %s\n    %s\nomit =\n    */test_*.py\n"
                "    */tests/*\n" % (os.path.join(data, ".coverage").replace("\\", "/"),
                                    os.path.join(REPO, "gateway").replace("\\", "/"),
                                    os.path.join(REPO, "groundstation").replace("\\", "/")))
    sections = ["F1", "AIS", "SH", "VI", "VR", "B"] if quick else \
        ["F1", "AIS", "SH", "CA", "VI", "VR", "A", "B", "C", "D", "E", "F", "G"]
    runs = [(["-m", "coverage", "run", "--rcfile", rc, "test_gateway.py", s], "gateway", "gateway section %s" % s)
            for s in sections]
    runs += [(["-m", "coverage", "run", "--rcfile", rc, "test_avoid.py"], "gateway", "test_avoid"),
             (["-m", "coverage", "run", "--rcfile", rc, os.path.join(HERE, "storm_gateway.py"), "--episodes", "20"],
              "gateway", "gateway storm"),
             (["-m", "coverage", "run", "--rcfile", rc, os.path.join(HERE, "stations_storm.py"), "--seconds", "60"],
              "gateway", "station storm")]
    for t in ("test_charts.py", "test_logs.py", "test_driver.py", "test_e2e.py"):
        runs.append((["-m", "coverage", "run", "--rcfile", rc, t], "groundstation", t))
    log = []
    for args, cwd, name in runs:
        r, dt = sh([PY] + args, cwd=os.path.join(REPO, cwd))
        log.append("  ran %-24s %5.0f s (exit %d)" % (name, dt, r.returncode))
    sh([PY, "-m", "coverage", "combine", "--rcfile", rc], cwd=data)
    sh([PY, "-m", "coverage", "json", "--rcfile", rc, "-o", os.path.join(COV, "py.json")], cwd=data)
    sh([PY, "-m", "coverage", "html", "--rcfile", rc, "-d", os.path.join(OUT, "coverage", "html")], cwd=data)
    files = {}
    try:
        with open(os.path.join(COV, "py.json")) as f:
            js = json.load(f)
    except (OSError, ValueError):
        return log + ["  (no coverage data)"], files
    for path, d in js["files"].items():
        rel = os.path.relpath(path, REPO).replace("\\", "/")
        executed = set(d["executed_lines"])
        unrun = []
        try:
            tree = ast.parse(open(path, encoding="utf-8").read())
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    body = set(range(node.body[0].lineno, (node.end_lineno or node.lineno) + 1))
                    if not body & executed:
                        unrun.append(node.name)
        except (OSError, SyntaxError):
            pass
        files[rel] = {"lines": (d["summary"]["percent_covered"], d["summary"]["num_statements"]), "unrun": unrun}
    return log, files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--skip-c", action="store_true")
    ap.add_argument("--skip-py", action="store_true")
    a = ap.parse_args()
    os.makedirs(COV, exist_ok=True)
    lines = ["=== COVERAGE (%s) ===" % ("quick" if a.quick else "full")]
    gaps = []
    if not a.skip_c:
        log, cf = c_coverage(a.quick)
        lines += ["", "C (app/, gcov; the simulator tests + fault storm + day at sea):"] + log
        lines += ["", "| file | lines | branches taken | functions never run |", "|---|---|---|---|"]
        for f in sorted(cf):
            d = cf[f]
            ln, br = d.get("lines", (0, 0)), d.get("taken", (0, 0))
            lines.append("| %s | %.0f %% of %d | %.0f %% of %d | %s |" % (f, ln[0], ln[1], br[0], br[1],
                                                                       ", ".join(d.get("unrun", [])) or "-"))
            if f in SAFETY_C and d.get("unrun"):
                gaps.append("app/%s: %s" % (f, ", ".join(d["unrun"])))
    if not a.skip_py:
        log, pf = py_coverage(a.quick)
        lines += ["", "Python (coverage.py; gateway, vision manager, ground station):"] + log
        lines += ["", "| file | lines | functions never run |", "|---|---|---|"]
        for f in sorted(pf):
            d = pf[f]
            lines.append("| %s | %.0f %% of %d | %s |" % (f, d["lines"][0], d["lines"][1],
                                                         ", ".join(d["unrun"][:12]) + (" ..." if len(d["unrun"]) > 12
                                                                                     else "") or "-"))
            if f in SAFETY_PY and d["unrun"]:
                gaps.append("%s: %s" % (f, ", ".join(d["unrun"])))
        lines.append("(the vision worker runs in its own Python - not measured here; its logic is in manager.py)")
    lines += ["", "SAFETY-RELEVANT functions that no test executes:"] + (["  " + g for g in gaps] or ["  none"])
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "coverage_summary.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
