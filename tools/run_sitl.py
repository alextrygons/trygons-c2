"""Run the ArduPilot SITL test suite (gateway/test_sitl.py, step X2) with a log and a time limit.

    python tools/run_sitl.py               run it now (foreground), log in test_results/sitl/, exit 0 / 1
    python tools/run_sitl.py --bg          start it in the background and return at once (prints the log's path);
                                           the result line is the last line of the log ("SITL TESTS: ...")
    python tools/run_sitl.py one six       only these suites (one / ground / six)
    python tools/run_sitl.py --limit-min 45

Without SITL installed (python tools/get_sitl.py) every test is N/A - never a PASS - and the exit code is 0.
The suite and everything it starts (the SITL copters, the router) run in ONE Windows job (KILL_ON_JOB_CLOSE): the
time limit (45 min, PROGRESS.md LOOP RULE 2) kills the whole tree; nothing is left behind (tools/strays.py).
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import strays  # noqa: E402

LOGS = os.path.join(REPO, "test_results", "sitl")


def run(suites, limit_s, log_path, out=print):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    job = strays.Job()
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
    with open(log_path, "wb") as log:
        p = job.popen([sys.executable, os.path.join(REPO, "gateway", "test_sitl.py")] + list(suites),
                      cwd=os.path.join(REPO, "gateway"), stdout=log, stderr=subprocess.STDOUT, env=env,
                      stdin=subprocess.DEVNULL)
        try:
            p.wait(timeout=limit_s)
            code = p.returncode
        except subprocess.TimeoutExpired:
            job.kill(p)
            log.write(("\nSITL TESTS FAILED: KILLED after the %d min time limit (STALLED)\n" % (limit_s // 60))
                      .encode())
            code = 2
        finally:
            job.kill()
            job.close()
    with open(log_path, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    for ln in lines:
        if ln.startswith(("[FAIL]", "[N/A ]", "FINDING:", "SITL TESTS")):
            out(ln)
    out("log: %s" % log_path)
    return code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("suites", nargs="*")
    ap.add_argument("--bg", action="store_true")
    ap.add_argument("--limit-min", type=float, default=45.0)
    ap.add_argument("--log", default="")
    a = ap.parse_args()
    log_path = a.log or os.path.join(LOGS, "sitl_%s.log" % time.strftime("%Y%m%d-%H%M%S", time.gmtime()))
    if a.bg:
        flags = 0x00000008 | 0x00000200 if os.name == "nt" else 0       # DETACHED_PROCESS | NEW_PROCESS_GROUP
        args = [sys.executable, os.path.abspath(__file__), "--log", log_path,
                "--limit-min", str(a.limit_min)] + a.suites
        subprocess.Popen(args, cwd=REPO, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, creationflags=flags, start_new_session=os.name != "nt")
        print("SITL tests started in the background; log: %s" % log_path)
        return 0
    return run(a.suites, int(a.limit_min * 60), log_path)


if __name__ == "__main__":
    sys.exit(main())
