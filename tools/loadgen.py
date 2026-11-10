"""CPU load generator + flaky-test repeat runner (CL2, docs/owner_findings_2026-11-07.md).

    python tools/loadgen.py --procs 4 [--seconds 600]
        N busy processes (a tight loop each, NORMAL priority = the load of other programs on the PC) until
        Ctrl-C / stdin closes / --seconds. They run in a KILL_ON_JOB_CLOSE job of THIS process (tools/strays.py Job):
        whatever ends loadgen ends them too - a busy process can never leak. Their command line names this file, so
        `python tools/strays.py list` shows them as this project's.

    python tools/loadgen.py --procs 4 --repeat 5 [--out NAME] -- <command ...>
        start the load, run <command> --repeat times (one after the other, cwd = the current folder), parse its
        "[PASS] name - detail" / "[FAIL] name - detail" lines and print the failure count PER TEST over the runs
        (+ the first failing detail line of each), the exit codes and the run times. --procs 0 = the same without
        load (the baseline). The full output of every run is kept under test_results/flaky/<out>/run<i>.txt and a
        summary.json next to it. Exit code: 0 when every run passed, 1 otherwise.

    --procs all     = one busy process per logical CPU (the PC saturated - a parallel build / a second gate)
    --priority below  the busy processes at BELOW_NORMAL (only to tell "priority" from "CPU time" apart)
    --test-priority above|high   the measured command at a raised priority class (a candidate harness fix)

Measurements only: nothing here touches the product code or the tests. The failure rates before / after a fix go
into docs/cleanup_report.md (CL2) + TEST_REPORT.md.
"""
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import strays  # noqa: E402

NO_WINDOW = 0x08000000 if os.name == "nt" else 0
PRIO = {"normal": 0x00000020, "below": 0x00004000, "above": 0x00008000, "high": 0x00000080}   # Windows classes
LINE = re.compile(r"^\[(PASS|FAIL)\] (.+?) - (.*)$")


def busy_forever():
    """One busy process: a tight loop (the argument names this file = this project's process for strays.py)."""
    x = 1
    while True:
        x = (x * 1103515245 + 12345) & 0xFFFFFFFF


def start_load(n, priority="normal"):
    """-> (Job, [Popen]) of n busy processes inside a KILL_ON_JOB_CLOSE job of this process."""
    job = strays.Job()
    procs = []
    flags = NO_WINDOW | (PRIO[priority] if os.name == "nt" else 0)
    for _ in range(n):
        p = job.popen([sys.executable, os.path.abspath(__file__), "busy"], cwd=HERE, stdin=subprocess.DEVNULL,
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=flags)
        procs.append(p)
    return job, procs


def stop_load(job, procs):
    for p in procs:
        job.kill(p)
    job.close()


def cpu_count():
    try:
        return len(os.sched_getaffinity(0))
    except AttributeError:
        return os.cpu_count() or 4


def parse(text):
    """{test name: (ok, detail)} of the "[PASS] / [FAIL] name - detail" lines (the LAST line per name counts)."""
    out = {}
    for ln in text.splitlines():
        m = LINE.match(ln.strip())
        if m:
            out[m.group(2)] = (m.group(1) == "PASS", m.group(3))
    return out


PARENT = os.getppid()


def parent_gone():
    """True when the process that started loadgen is gone (a stopped measurement script): the runs end then - a
    repeat runner must never go on alone for an hour with the busy processes (seen in CL2: the shell killed, loadgen
    + 20 busy processes + the test kept running)."""
    try:
        return not strays._alive(PARENT)
    except Exception:
        return False


def run_once(cmd, out_path, test_priority=None, timeout=None):
    flags = NO_WINDOW if os.name == "nt" else 0
    if test_priority and os.name == "nt":
        flags |= PRIO[test_priority]
    t0 = time.time()
    with open(out_path, "w", encoding="utf-8", errors="replace") as f:
        job = strays.Job()
        p = job.popen(cmd, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, creationflags=flags)
        rc = None
        end = time.time() + timeout if timeout else None
        while True:                                     # wait in slices: the time limit AND the parent watched
            try:
                rc = p.wait(timeout=2.0)
                break
            except subprocess.TimeoutExpired:
                if (end and time.time() > end) or parent_gone():
                    break
        job.kill(p)                                     # never a leftover simulator / station / worker
        job.close()
    dt = time.time() - t0
    with open(out_path, encoding="utf-8", errors="replace") as f:
        text = f.read()
    return rc, dt, parse(text)


def repeat(cmd, n, procs, priority, test_priority, out_name, timeout):
    folder = os.path.join(REPO, "test_results", "flaky", out_name)
    os.makedirs(folder, exist_ok=True)
    nload = cpu_count() if procs == "all" else int(procs)
    print("loadgen: %d busy process(es) at %s priority, %d logical CPUs; %d run(s) of: %s"
          % (nload, priority, cpu_count(), n, " ".join(cmd)), flush=True)
    job, load = (None, [])
    if nload:
        job, load = start_load(nload, priority)
        time.sleep(1.0)
    fails, firsts, runs = {}, {}, []
    stopped = None
    try:
        for i in range(1, n + 1):
            if parent_gone():
                stopped = "the parent process is gone: stopped before run %d" % i
                print("loadgen: " + stopped, flush=True)
                break
            rc, dt, res = run_once(cmd, os.path.join(folder, "run%d.txt" % i), test_priority, timeout)
            bad = sorted(k for k, (ok, _d) in res.items() if not ok)
            runs.append({"run": i, "rc": rc, "seconds": round(dt, 1), "tests": len(res), "failed": bad})
            for k in bad:
                fails[k] = fails.get(k, 0) + 1
                firsts.setdefault(k, res[k][1])
            print("run %d/%d: rc %s, %.0f s, %d tests, failed: %s" % (i, n, rc, dt, len(res), ", ".join(bad) or "-"),
                  flush=True)
    finally:
        if job is not None:
            stop_load(job, load)
    print("\nFAILURE RATE per test over %d run(s) under %d busy process(es):" % (n, nload))
    if not fails:
        print("  none failed")
    for k in sorted(fails, key=lambda s: s[:6]):
        print("  %d/%d  %s\n        first failing detail: %s" % (fails[k], n, k[:110], firsts[k][:400]))
    summary = {"cmd": cmd, "runs": runs, "busy_procs": nload, "priority": priority, "test_priority": test_priority,
               "cpus": cpu_count(), "fails": fails, "first_detail": firsts, "when": time.strftime("%Y-%m-%d %H:%M"),
               "stopped": stopped}
    with open(os.path.join(folder, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1)
    print("written: %s" % os.path.join(folder, "summary.json"))
    return 0 if not fails and all(r["rc"] == 0 for r in runs) else 1


def main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if a[:1] == ["busy"]:
        busy_forever()
    cmd = None
    if "--" in a:
        cmd = a[a.index("--") + 1:]
        a = a[:a.index("--")]

    def opt(name, default=None):
        if name in a:
            i = a.index(name)
            v = a[i + 1]
            del a[i:i + 2]
            return v
        return default
    procs = opt("--procs", "4")
    seconds = float(opt("--seconds", "0") or 0)
    priority = opt("--priority", "normal")
    test_priority = opt("--test-priority")
    n = int(opt("--repeat", "1"))
    out_name = opt("--out") or ("run_%s" % time.strftime("%Y%m%d_%H%M%S"))
    timeout = float(opt("--timeout", "0") or 0) or None
    if a or (not cmd and "--help" in sys.argv):
        print(__doc__)
        return 2
    if cmd:
        return repeat(cmd, n, procs, priority, test_priority, out_name, timeout)
    nload = cpu_count() if procs == "all" else int(procs)
    job, load = start_load(nload, priority)
    print("loadgen: %d busy process(es) PIDs %s (%s priority) - Ctrl-C / stdin EOF%s ends them"
          % (nload, [p.pid for p in load], priority, (" / %.0f s" % seconds) if seconds else ""), flush=True)
    end = time.time() + seconds if seconds else None
    try:
        while end is None or time.time() < end:
            if sys.stdin and not sys.stdin.isatty():
                if not sys.stdin.readline():                # the parent closed our stdin
                    break
            else:
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_load(job, load)
    return 0


if __name__ == "__main__":
    sys.exit(main())
