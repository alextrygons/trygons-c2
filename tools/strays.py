"""Stray processes of THIS project (CL3, W10 / W11): list them, stop them, and the test gate's "no leftovers" check.

    python tools/strays.py list              every running process of THIS project (PID, parent, age, command)
    python tools/strays.py stop [--all]      end them (the whole tree); --all = the running demo too
    python tools/strays.py mark --begin      the gate starts: remember what runs now (never blamed on a suite)
    python tools/strays.py check SUITE       after a suite: what it left behind = LEFTOVER lines (name, PID, suite),
                                             stopped (unless --keep), remembered; exit 1 when there was one
    python tools/strays.py report            the gate ends: every leftover of the run; exit 1 when there was one
    python tools/strays.py selftest          the matching rules (a foreign process is NEVER matched) + the job object

A process is "ours" when its program is one the project starts (python / pythonw, autopilot_sim, storm, mediamtx,
ffmpeg / ffprobe, piper) AND its executable or its command line is inside THIS repo folder (both slash styles, the
MSYS form too) or its command line names a test folder of ours (tempfile prefixes "trygons_" / "gwtest_"), OR its
parent is ours (the gateway's workers, a test's simulator ...). Never: another folder (../review-wt, the owner's
other Python programs), this process and its parents (the shell / runner that called us).
The demo (demo_run/pids.json, pid + creation time) is shown as DEMO, stopped only with --all and never a leftover.

Also a library for the test / tool code (no third-party module - psutil is not installed here):
    Job()                a Windows job object with KILL_ON_JOB_CLOSE: job.popen(...) / job.kill(p) ends the tree
    dies_with_parent(p)  the child dies with THIS process whatever ends it (one job per process)
    processes()          every process: {pid: Proc}; ours() -> the project's
"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PROGRAMS = {"python", "pythonw", "autopilot_sim", "storm", "mediamtx", "ffmpeg", "ffprobe", "piper"}
TEMP_MARKERS = ("trygons_", "gwtest_")          # tempfile prefixes of our tests (the owner's programs use neither)
STATE = os.environ.get("TRYGONS_STRAYS_STATE") or os.path.join(tempfile.gettempdir(), "trygons_strays.json")
DEMO_PIDS = os.path.join(REPO, "demo_run", "pids.json")
NO_WINDOW = 0x08000000 if os.name == "nt" else 0


class Proc(object):
    __slots__ = ("pid", "ppid", "name", "exe", "cmd", "created")

    def __init__(self, pid, ppid, name, exe, cmd, created):
        self.pid, self.ppid, self.name, self.exe, self.cmd, self.created = pid, ppid, name, exe, cmd, created

    def key(self):
        return "%d:%s" % (self.pid, self.created)

    def age_s(self, now=None):
        return max(0, int((now or time.time()) - self.created)) if self.created else 0


# --- the process list ---------------------------------------------------------------------------------------------
_PS = ("Get-CimInstance Win32_Process | Select-Object ProcessId,ParentProcessId,Name,ExecutablePath,CommandLine,"
       "@{n='Created';e={ if ($_.CreationDate) { [int64](($_.CreationDate).ToUniversalTime() - "
       "(Get-Date '1970-01-01T00:00:00Z').ToUniversalTime()).TotalSeconds } else { 0 } }} | ConvertTo-Json -Compress")


def processes():
    """{pid: Proc} of every process; {} when the list can't be read."""
    out = {}
    try:
        if os.name == "nt":
            r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", _PS], capture_output=True,
                               text=True, timeout=60, creationflags=NO_WINDOW)
            rows = json.loads(r.stdout or "[]")
            if isinstance(rows, dict):
                rows = [rows]
            for row in rows:
                out[int(row["ProcessId"])] = Proc(int(row["ProcessId"]), int(row.get("ParentProcessId") or 0),
                                                  (row.get("Name") or "").lower(), row.get("ExecutablePath") or "",
                                                  row.get("CommandLine") or "", int(row.get("Created") or 0))
        else:
            r = subprocess.run(["ps", "-eo", "pid=,ppid=,etimes=,comm=,args="], capture_output=True, text=True,
                               timeout=60)
            now = time.time()
            for line in r.stdout.splitlines():
                f = line.split(None, 4)
                if len(f) < 4:
                    continue
                pid, ppid, et, comm = int(f[0]), int(f[1]), int(f[2]), f[3]
                args = f[4] if len(f) > 4 else comm
                exe = ""
                try:
                    exe = os.readlink("/proc/%d/exe" % pid)
                except OSError:
                    pass
                out[pid] = Proc(pid, ppid, os.path.basename(comm).lower(), exe, args, int(now - et))
    except (OSError, ValueError, subprocess.SubprocessError):
        return {}
    return out


def program(name):
    """'python3.12' / 'pythonw.exe' -> 'python' / 'pythonw'; 'autopilot_sim.exe' -> 'autopilot_sim'."""
    n = (name or "").lower()
    if n.endswith(".exe"):
        n = n[:-4]
    if n.startswith("python") and not n.startswith("pythonw"):
        return "python"
    if n.startswith("pythonw"):
        return "pythonw"
    return n


def _norm(s):
    return (s or "").replace("\\", "/").lower()


def repo_forms(repo=None):
    """The repo folder as it appears in command lines: 'c:/trygons/trygons_c2' and the MSYS '/c/trygons/trygons_c2'."""
    r = _norm(os.path.abspath(repo or REPO)).rstrip("/")
    forms = [r]
    if len(r) > 2 and r[1] == ":":
        forms.append("/" + r[0] + r[2:])
    return forms


def in_repo(text, repo=None):
    """True when the text names THIS repo folder (or something inside it) - never a sibling folder."""
    t = _norm(text)
    for f in repo_forms(repo):
        i = t.find(f)
        while i >= 0:
            end = i + len(f)
            if end == len(t) or t[end] in "/\"' ":
                return True
            i = t.find(f, i + 1)
    return False


def marked(text):
    """True when the text names a test folder of ours IN THE TEMP FOLDER (tempfile.mkdtemp prefix 'trygons_' /
    'gwtest_') - the folder itself, never a look-alike name elsewhere (C:/trygons/TRYGONS_C2_other is not ours)."""
    t = _norm(text)
    roots = [_norm(tempfile.gettempdir()).rstrip("/")]
    if os.name != "nt":
        roots.append("/tmp")
    return any((r + "/" + m) in t for r in roots for m in TEMP_MARKERS)


def ours(procs=None, repo=None, exclude=None):
    """The project's processes: [Proc] (parents before children). exclude = pids never matched (default: this
    process and its ancestors)."""
    procs = processes() if procs is None else procs
    if exclude is None:
        exclude = set()
        p = os.getpid()
        while p in procs and p not in exclude:
            exclude.add(p)
            p = procs[p].ppid
    found = {}
    for p in procs.values():
        if p.pid in exclude or program(p.name) not in PROGRAMS:
            continue
        if in_repo(p.exe, repo) or in_repo(p.cmd, repo) or marked(p.cmd):
            found[p.pid] = p
    grew = True
    while grew:                                     # + their children (a gateway worker, a test's simulator)
        grew = False
        for p in procs.values():
            if p.pid not in found and p.pid not in exclude and p.ppid in found and p.ppid != p.pid \
                    and program(p.name) in PROGRAMS:
                found[p.pid] = p
                grew = True
    order, seen = [], set()

    def add(p):
        if p.pid in seen:
            return
        if p.ppid in found and p.ppid != p.pid:
            add(found[p.ppid])
        seen.add(p.pid)
        order.append(p)
    for p in sorted(found.values(), key=lambda x: x.pid):
        add(p)
    return order


def demo_keys():
    """{'pid:created'} of the running demo (demo_run/pids.json entries whose process still exists)."""
    keys = set()
    try:
        with open(DEMO_PIDS, encoding="utf-8") as f:
            pids = json.load(f)
    except (OSError, ValueError):
        return keys
    procs = processes()
    for name, pid in pids.items():
        if name == "_id" or not isinstance(pid, int) or pid not in procs:
            continue
        keys.add(procs[pid].key())
    return keys


# --- stopping -------------------------------------------------------------------------------------------------------
def stop_procs(plist, wait_s=8.0):
    """End these processes (trees, parents first). -> the ones still alive after wait_s (pids)."""
    if not plist:
        return []
    if os.name == "nt":
        for p in plist:
            subprocess.run(["taskkill", "/PID", str(p.pid), "/T", "/F"], capture_output=True, timeout=60,
                           creationflags=NO_WINDOW)
    else:
        import signal
        for p in plist:
            try:
                os.kill(p.pid, signal.SIGKILL)
            except OSError:
                pass
    end = time.time() + wait_s
    want = {p.pid: p.key() for p in plist}
    while True:
        now = processes()
        alive = [pid for pid, k in want.items() if pid in now and now[pid].key() == k]
        if not alive or time.time() > end:
            return alive
        time.sleep(0.5)


# --- the job object (library) --------------------------------------------------------------------------------------
class Job(object):
    """A Windows job object with KILL_ON_JOB_CLOSE: every process started with popen() and everything THEY start
    ends with kill() - or by the OS when this process dies (the handle closes). POSIX: a process group."""

    def __init__(self):
        self.h = None
        if os.name != "nt":
            return
        try:
            import ctypes
            from ctypes import wintypes
            k32 = ctypes.WinDLL("kernel32", use_last_error=True)
            k32.CreateJobObjectW.restype = wintypes.HANDLE
            k32.OpenProcess.restype = wintypes.HANDLE
            h = k32.CreateJobObjectW(None, None)

            class BASIC(ctypes.Structure):
                _fields_ = [("a", ctypes.c_int64), ("b", ctypes.c_int64), ("LimitFlags", wintypes.DWORD),
                            ("MinWS", ctypes.c_size_t), ("MaxWS", ctypes.c_size_t),
                            ("ActiveProcessLimit", wintypes.DWORD), ("Affinity", ctypes.c_size_t),
                            ("PriorityClass", wintypes.DWORD), ("SchedulingClass", wintypes.DWORD)]

            class IOC(ctypes.Structure):
                _fields_ = [("c%d" % i, ctypes.c_uint64) for i in range(6)]

            class EXT(ctypes.Structure):
                _fields_ = [("Basic", BASIC), ("Io", IOC), ("ProcessMemoryLimit", ctypes.c_size_t),
                            ("JobMemoryLimit", ctypes.c_size_t), ("PeakProcessMemoryUsed", ctypes.c_size_t),
                            ("PeakJobMemoryUsed", ctypes.c_size_t)]
            info = EXT()
            info.Basic.LimitFlags = 0x2000              # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if h and k32.SetInformationJobObject(wintypes.HANDLE(h), 9, ctypes.byref(info), ctypes.sizeof(info)):
                self.h, self.k32, self.wt = h, k32, wintypes
        except Exception:
            self.h = None

    def assign(self, p):
        """Put a subprocess.Popen into the job. -> True when the OS will end it with the job."""
        if os.name != "nt" or not self.h:
            return False
        hp = self.k32.OpenProcess(0x0101, False, p.pid)             # PROCESS_TERMINATE | PROCESS_SET_QUOTA
        ok = False
        if hp:
            ok = bool(self.k32.AssignProcessToJobObject(self.wt.HANDLE(self.h), self.wt.HANDLE(hp)))
            self.k32.CloseHandle(self.wt.HANDLE(hp))
        return ok

    def popen(self, args, **kw):
        if os.name != "nt":
            kw.setdefault("start_new_session", True)
            return subprocess.Popen(args, **kw)
        p = subprocess.Popen(args, **kw)
        self.assign(p)
        return p

    def kill(self, p=None):
        """End the whole job (and p, when given) and wait a little."""
        if os.name != "nt":
            if p is not None:
                try:
                    os.killpg(p.pid, 9)
                except OSError:
                    pass
        elif self.h:
            self.k32.TerminateJobObject(self.wt.HANDLE(self.h), 1)
        elif p is not None and p.poll() is None:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True, timeout=60,
                           creationflags=NO_WINDOW)
        if p is not None:
            try:
                p.kill()
            except OSError:
                pass
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass

    def close(self):
        if os.name == "nt" and self.h:
            self.k32.CloseHandle(self.wt.HANDLE(self.h))
            self.h = None


_OWN_JOB = None


def dies_with_parent(p):
    """The child p (a subprocess.Popen) ends when THIS process ends - whatever ends it (a test time limit, Task
    Manager, a crash). Windows: one KILL_ON_JOB_CLOSE job per process (its handle closes with the process). -> True
    when the OS guarantees it; False elsewhere (Ubuntu: systemd / the process group of the caller)."""
    global _OWN_JOB
    if os.name != "nt" or p is None:
        return False
    if _OWN_JOB is None:
        _OWN_JOB = Job()
    return _OWN_JOB.assign(p)


# --- the gate: mark / check / report -------------------------------------------------------------------------------
def _load(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"baseline": {}, "leftovers": [], "preexisting": [], "suites": []}


def _save(path, st):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(st, f, indent=1)


def _row(p, now):
    return {"pid": p.pid, "name": p.name, "age_s": p.age_s(now), "cmd": p.cmd[:300]}


def cmd_mark(path, begin, out=print):
    now = time.time()
    plist = ours()
    st = _load(path) if not begin else {"baseline": {}, "leftovers": [], "preexisting": [], "suites": []}
    st["baseline"] = {p.key(): _row(p, now) for p in plist}
    if begin:
        st["began"] = time.strftime("%Y-%m-%d %H:%M:%S")
        demo = demo_keys()
        st["preexisting"] = [dict(_row(p, now), demo=p.key() in demo) for p in plist]
        for p in plist:
            out("already running before this run (not a leftover of it)%s: %s PID %d age %d s -- %s"
                % (" [DEMO]" if p.key() in demo else "", p.name, p.pid, p.age_s(now), p.cmd[:160]))
    _save(path, st)
    return 0


def cmd_check(path, suite, keep=False, out=print):
    """New project processes since the mark = leftovers of this suite. They are printed, stopped (unless keep),
    remembered in the state file; the baseline moves on. -> 1 when there was a leftover."""
    now = time.time()
    st = _load(path)
    base = set(st.get("baseline", {}))
    demo = demo_keys()
    plist = ours()
    left = [p for p in plist if p.key() not in base and p.key() not in demo]
    for p in left:
        out("LEFTOVER after %s: %s PID %d (parent %d) age %d s -- %s" % (suite, p.name, p.pid, p.ppid, p.age_s(now),
                                                                         p.cmd[:200]))
    alive = []
    if left and not keep:
        alive = stop_procs(left)
        if alive:
            out("  NOT stopped (still alive): %s" % ", ".join(str(a) for a in alive))
        else:
            out("  stopped %d leftover process(es)" % len(left))
    for p in left:
        st.setdefault("leftovers", []).append(dict(_row(p, now), suite=suite, ppid=p.ppid,
                                                   stopped=not keep and p.pid not in alive))
    st.setdefault("suites", []).append(suite)
    # the baseline moves on: what runs now (the stopped leftovers are gone; a kept one is not blamed twice)
    st["baseline"] = {p.key(): _row(p, now) for p in (ours() if left else plist)}
    _save(path, st)
    return 1 if left else 0


def cmd_report(path, out=print):
    st = _load(path)
    left = st.get("leftovers", [])
    n = len(st.get("suites", []))
    if not left:
        out("NO LEFTOVER PROCESSES: %d suite(s) checked" % n)
        return 0
    out("LEFTOVER PROCESSES: %d (in %d suite(s) checked)" % (len(left), n))
    for r in left:
        out("  %-28s %-16s PID %-7d age %5d s  %s -- %s" % (r.get("suite", "?"), r["name"], r["pid"], r["age_s"],
                                                           "stopped" if r.get("stopped") else "STILL RUNNING",
                                                           r["cmd"][:140]))
    return 1


def cmd_list(out=print):
    now = time.time()
    plist = ours()
    demo = demo_keys()
    if not plist:
        out("no process of this project is running")
        return 0
    for p in plist:
        out("%-16s PID %-7d parent %-7d age %6d s%s  %s" % (p.name, p.pid, p.ppid, p.age_s(now),
                                                            "  [DEMO]" if p.key() in demo else "", p.cmd[:160]))
    return 0


def cmd_stop(all_=False, out=print):
    plist = ours()
    demo = demo_keys()
    skip = [p for p in plist if p.key() in demo and not all_]
    todo = [p for p in plist if not (p.key() in demo and not all_)]
    for p in skip:
        out("kept (the demo; STOP_DEMO.bat or --all): %s PID %d" % (p.name, p.pid))
    if not todo:
        out("nothing to stop")
        return 0
    for p in todo:
        out("stopping %s PID %d -- %s" % (p.name, p.pid, p.cmd[:140]))
    alive = stop_procs(todo)
    if alive:
        out("STILL RUNNING: %s" % ", ".join(str(a) for a in alive))
        return 1
    out("stopped %d process(es)" % len(todo))
    return 0


# --- selftest -------------------------------------------------------------------------------------------------------
def selftest(out=print):
    bad = []
    py = sys.executable
    sleeper = "import sys, time; time.sleep(120)"
    kids = []
    foreign_dir = tempfile.mkdtemp(prefix="elsewhere_")          # no project marker in its name
    state = os.path.join(foreign_dir, "state.json")
    try:
        # (1) ours by the command line (the repo folder in an argument), (2) foreign: no marker, parent = this
        # process (excluded) - like the owner's other Python programs; (3) a SIBLING folder (../review-wt, a clone
        # next to the repo) is NOT this project; (4) ours by a test temp folder marker
        mine = subprocess.Popen([py, "-c", sleeper, os.path.join(REPO, "sim")], cwd=foreign_dir, creationflags=NO_WINDOW)
        foreign = subprocess.Popen([py, "-c", sleeper, os.path.join(foreign_dir, "x")], cwd=foreign_dir,
                                   creationflags=NO_WINDOW)
        sibling = subprocess.Popen([py, "-c", sleeper, os.path.join(os.path.dirname(REPO), "review-wt", "sim"),
                                    REPO + "_other"], cwd=foreign_dir, creationflags=NO_WINDOW)
        tmpm = subprocess.Popen([py, "-c", sleeper, os.path.join(tempfile.gettempdir(), "trygons_t1", "x")],
                                cwd=foreign_dir, creationflags=NO_WINDOW)
        kids += [mine, foreign, sibling, tmpm]
        time.sleep(1.0)
        got = {p.pid for p in ours()}
        if mine.pid not in got:
            bad.append("a python with the repo folder in its command line was NOT matched")
        if tmpm.pid not in got:
            bad.append("a python with a trygons_ temp folder in its command line was NOT matched")
        if foreign.pid in got:
            bad.append("NEGATIVE CONTROL FAILED: a foreign python (no marker) was matched")
        if sibling.pid in got:
            bad.append("NEGATIVE CONTROL FAILED: a python of a SIBLING folder (review-wt / _other) was matched")
        if os.getpid() in got:
            bad.append("this process matched itself")
        # the pure functions
        for t, want in ((REPO + "\\sim\\autopilot_sim.exe server 5760", True), (REPO.lower() + "/x", True),
                        (repo_forms()[-1] + "/tools/x.py", True), (REPO + "_other/x", False),
                        (os.path.join(os.path.dirname(REPO), "review-wt", "x"), False), ("", False)):
            if in_repo(t) != want:
                bad.append("in_repo(%r) != %s" % (t, want))
        for n, want in (("python3.12", "python"), ("pythonw.exe", "pythonw"), ("autopilot_sim.exe", "autopilot_sim"),
                        ("FFMPEG.EXE", "ffmpeg")):
            if program(n) != want:
                bad.append("program(%r) = %r" % (n, program(n)))
        # (5) the child rule: a child of OURS with a foreign command line is ours (a gateway worker), a child of the
        # foreign one is not
        cmd_rule = "import subprocess, sys, time; subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)']); time.sleep(120)"
        par = subprocess.Popen([py, "-c", cmd_rule, os.path.join(REPO, "gateway")], cwd=foreign_dir,
                               creationflags=NO_WINDOW)
        fpar = subprocess.Popen([py, "-c", cmd_rule, os.path.join(foreign_dir, "y")], cwd=foreign_dir,
                                creationflags=NO_WINDOW)
        kids += [par, fpar]
        time.sleep(2.0)
        procs = processes()
        plist = ours(procs)
        pids = {p.pid for p in plist}
        kid_of = lambda pp: [p for p in procs.values() if p.ppid == pp.pid and program(p.name) == "python"]
        if not kid_of(par) or any(k.pid not in pids for k in kid_of(par)):
            bad.append("the child of a project process was NOT matched")
        if any(k.pid in pids for k in kid_of(fpar)) or fpar.pid in pids:
            bad.append("NEGATIVE CONTROL FAILED: the child of a foreign process was matched")
        order = [p.pid for p in plist]
        if par.pid in order and any(order.index(k.pid) < order.index(par.pid) for k in kid_of(par) if k.pid in order):
            bad.append("parents are not listed before their children")
        # (6) mark / check / report with the state file: the leftover of "suite X" is named, stopped, reported
        for k in (mine, tmpm, par, fpar, foreign, sibling):
            k.kill()
        for k in kids:
            try:
                k.wait(10)
            except subprocess.TimeoutExpired:
                pass
        kids = []
        lines = []
        cmd_mark(state, True, lines.append)
        late = subprocess.Popen([py, "-c", sleeper, os.path.join(REPO, "sim")], cwd=foreign_dir, creationflags=NO_WINDOW)
        kids.append(late)
        time.sleep(1.0)
        rc = cmd_check(state, "suite X", out=lines.append)
        if rc != 1 or not any(("LEFTOVER after suite X" in l and str(late.pid) in l) for l in lines):
            bad.append("check did not report the leftover: %r" % lines[-3:])
        if late.poll() is None:
            time.sleep(1.0)
        if late.poll() is None:
            bad.append("check did not stop the leftover")
        rc2 = cmd_check(state, "suite Y", out=lines.append)
        if rc2 != 0:
            bad.append("a clean suite reported a leftover")
        rl = []
        rc3 = cmd_report(state, rl.append)
        if rc3 != 1 or not any("suite X" in l for l in rl) or any("suite Y" in l for l in rl):
            bad.append("report wrong: %r" % rl)
        # (7) dies_with_parent: a grandchild in the job dies with its parent; without the job it survives
        gc_cmd = ("import os, subprocess, sys, time; sys.path.insert(0, %r); import strays; "
                  "p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(120)', %r]); "
                  "ok = strays.dies_with_parent(p) if sys.argv[1] == 'job' else False; "
                  "print(p.pid, ok, flush=True); time.sleep(120)" % (HERE, os.path.join(REPO, "x")))
        for mode in ("job", "nojob"):
            par2 = subprocess.Popen([py, "-c", gc_cmd, mode], cwd=foreign_dir, stdout=subprocess.PIPE, text=True,
                                    creationflags=NO_WINDOW)
            kids.append(par2)
            line = par2.stdout.readline().split()
            gpid, ok = int(line[0]), line[1] == "True"
            par2.kill()
            par2.wait(10)
            time.sleep(1.5)
            alive = gpid in processes()
            if mode == "job":
                if os.name == "nt" and (not ok or alive):
                    bad.append("dies_with_parent: grandchild %s (assigned %s)" % ("ALIVE" if alive else "gone", ok))
            else:
                if os.name == "nt" and not alive:
                    bad.append("without the job the grandchild died too (the selftest proves nothing)")
            if alive:
                stop_procs([Proc(gpid, 0, "python", "", "", 0)], 5.0)
    finally:
        for k in kids:
            try:
                k.kill()
            except OSError:
                pass
        for p in ours():                         # whatever the selftest started and still runs
            if "time.sleep(120)" in p.cmd:
                stop_procs([p], 3.0)
        import shutil
        shutil.rmtree(foreign_dir, ignore_errors=True)
    out("STRAYS SELFTEST: %s%s" % ("OK" if not bad else "FAILED", "" if not bad else " - " + "; ".join(bad)))
    return 0 if not bad else 1


def main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if not a or a[0] in ("-h", "--help"):
        print(__doc__)
        return 2
    cmd, rest = a[0], a[1:]
    path = STATE
    if "--state" in rest:
        path = rest[rest.index("--state") + 1]
        del rest[rest.index("--state"):rest.index("--state") + 2]
    if cmd == "list":
        return cmd_list()
    if cmd == "stop":
        return cmd_stop("--all" in rest)
    if cmd == "mark":
        return cmd_mark(path, "--begin" in rest)
    if cmd == "check":
        suite = " ".join(x for x in rest if not x.startswith("--")) or "?"
        return cmd_check(path, suite, "--keep" in rest)
    if cmd == "report":
        return cmd_report(path)
    if cmd == "selftest":
        return selftest()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
