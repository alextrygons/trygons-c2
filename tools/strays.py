"""Stray processes of THIS project (CL3, W10 / W11; CL3-r): list them, stop them, and the test gate's "no leftovers" check.

    python tools/strays.py list              every running process of THIS project (PID, parent, age, command)
    python tools/strays.py stop [--all] [--yes]   end them (the whole tree) after a confirmation; --all = the demo too
    python tools/strays.py mark --begin [--shell WINPID]
                                             the gate starts: remember what runs now (never blamed on a suite) and put
                                             the SHELL that runs the gate (check.sh: `cat /proc/$$/winpid`) into the
                                             gate's job object (what it starts from now on = the suites = provable)
    python tools/strays.py check SUITE       after a suite: what it left behind = LEFTOVER lines (name, PID, suite),
                                             stopped (unless --keep), remembered; exit 1 when there was one, 3 when the
                                             check itself was not possible (process list / state file not readable)
    python tools/strays.py report            the gate ends: every leftover of the run; exit 1 when there was one or a
                                             check was not possible
    python tools/strays.py selftest          the matching rules (a foreign process is NEVER matched), the gate's
                                             proof / protection rules (CL3-r), the job object
    python tools/strays.py tag               the short tag of this repo folder (the gate's state file name)

A process is "ours" when its program is one the project starts (python / pythonw, autopilot_sim, storm, mediamtx,
ffmpeg / ffprobe, piper) AND its executable or its command line is inside THIS repo folder (both slash styles, the
MSYS form too) or its command line names a test folder of ours (tempfile prefixes "trygons_" / "gwtest_"), OR its
parent is ours (the gateway's workers, a test's simulator ...). Never: another folder (../review-wt, the owner's
other Python programs), this process and its parents (the shell / runner that called us).
The demo (demo_run/pids.json, pid + creation time) is shown as DEMO, stopped only with --all and never a leftover.

THE GATE NEVER KILLS WHAT IT CANNOT PROVE (CL3-r, the opus review of c851218, MED-1): a ground station holding HELM
of a real boat, a gateway, the owner's demo may be running while the gate runs 30+ minutes. After a suite, a new
project process is
  - a LEFTOVER (reported, stopped, the gate FAILS) only when it is PROVED to be the suite's: it is in the gate's job
    object (`mark --begin` puts the calling shell into a named job; everything the shell starts afterwards - the
    suites and their children - is in it; `check` asks IsProcessInJob) or in the runner's own Job (run_acceptance.py,
    fresh_build.py), AND it is not protected;
  - PROTECTED (reported, NEVER stopped, never counted): the demo of demo_run/pids.json and every process whose parent
    chain reaches a demo / baseline / protected process (the gateway's workers, MediaMTX, ffmpeg, piper, the
    assistant worker of a station ...), and every ground-station / gateway / demo launcher command line
    (groundstation/app.py, gateway/gateway.py, demo/demo.py) - one of those the suite itself started is reported as a
    leftover the gate did NOT stop (the gate fails; the owner ends it after a look);
  - UNPROVEN (reported only, never stopped, never counted): any other new project process the gate cannot prove
    (started outside the job: a station the owner opened during the gate).
A process list that cannot be read (MED-2) = a LOUD failure (exit 3, the gate fails), nothing is stopped and the
baseline is never replaced by an empty one; the same for a missing / unreadable state file.
`stop` is the OWNER's explicit command (a confirmation, or --yes); the gate never runs it.

Also a library for the test / tool code (no third-party module - psutil is not installed here):
    Job()                a Windows job object with KILL_ON_JOB_CLOSE: job.popen(...) / job.kill(p) ends the tree,
                         job.contains(pid) = the proof for cmd_check(job=...)
    dies_with_parent(p)  the child dies with THIS process whatever ends it (one job per process)
    processes()          every process: {pid: Proc}, or None when the list can't be read; ours() -> the project's
    classify(...)        the LEFTOVER / protected / unproven / demo split of the new processes
"""
import hashlib
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
# command lines the gate NEVER force-kills (a station may hold HELM of a real boat, a gateway talks to the AURIX)
ENTRY_POINTS = (("groundstation/app.py", "a ground station"), ("gateway/gateway.py", "a gateway"),
                ("demo/demo.py", "the demo launcher"))
DEMO_PIDS = os.path.join(REPO, "demo_run", "pids.json")
NO_WINDOW = 0x08000000 if os.name == "nt" else 0
JOB_QUERY = 0x0004                              # JOB_OBJECT_QUERY
HOLDER_MAX_S = 8 * 3600                         # the gate job holder ends by itself after this (a gate never runs longer)
LAST_ERROR = ""                                 # why processes() returned None


def repo_tag(repo=None):
    """A short tag of the repo folder: two clones never share a state file / gate job name (CL3-r LOW)."""
    r = (repo or REPO).replace("\\", "/").lower().rstrip("/")
    return hashlib.sha1(r.encode("utf-8")).hexdigest()[:8]


STATE = os.environ.get("TRYGONS_STRAYS_STATE") or os.path.join(tempfile.gettempdir(),
                                                                 "trygons_strays_%s.json" % repo_tag())


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


def _processes(timeout):
    out = {}
    if os.name == "nt":
        r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", _PS], capture_output=True,
                           text=True, timeout=timeout, creationflags=NO_WINDOW)
        if r.returncode != 0 or not (r.stdout or "").strip():
            raise ValueError("PowerShell exit %d: %s" % (r.returncode, (r.stderr or "")[:200]))
        rows = json.loads(r.stdout)
        if isinstance(rows, dict):
            rows = [rows]
        for row in rows:
            out[int(row["ProcessId"])] = Proc(int(row["ProcessId"]), int(row.get("ParentProcessId") or 0),
                                              (row.get("Name") or "").lower(), row.get("ExecutablePath") or "",
                                              row.get("CommandLine") or "", int(row.get("Created") or 0))
    else:
        r = subprocess.run(["ps", "-eo", "pid=,ppid=,etimes=,comm=,args="], capture_output=True, text=True,
                           timeout=timeout)
        if r.returncode != 0:
            raise ValueError("ps exit %d" % r.returncode)
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
    if os.getpid() not in out:                      # a list without THIS process is no list
        raise ValueError("the process list does not contain this process (%d of %d entries)" % (os.getpid(), len(out)))
    return out


def processes(retry=True):
    """{pid: Proc} of every process, or None when the list can't be read (LAST_ERROR says why). NEVER {} - an empty
    list would wipe a baseline and blame everything on the next suite (CL3-r MED-2). One retry, longer time limit."""
    global LAST_ERROR
    err = None
    for timeout in ((60, 180) if retry else (60,)):
        try:
            out = _processes(timeout)
            LAST_ERROR = ""
            return out
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as e:
            err = e
    LAST_ERROR = "%s: %s" % (type(err).__name__, str(err)[:300])
    return None


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


def entry_point(cmd):
    """'a ground station' / 'a gateway' / 'the demo launcher' when the command line is one of the launchers the gate
    never force-kills, else None."""
    t = _norm(cmd)
    for frag, what in ENTRY_POINTS:
        if frag in t:
            return what
    return None


def is_holder(p):
    """The gate job holder of this tool (strays.py hold ...): never a leftover, ended by `report`."""
    c = _norm(p.cmd)
    return "strays.py" in c and " hold " in c + " "


def _self_and_ancestors(procs):
    exclude = set()
    p = os.getpid()
    while p in procs and p not in exclude:
        exclude.add(p)
        p = procs[p].ppid
    return exclude


def ours(procs=None, repo=None, exclude=None):
    """The project's processes: [Proc] (parents before children), or None when the process list can't be read.
    exclude = pids never matched (default: this process and its ancestors). The gate job holders are never listed."""
    procs = processes() if procs is None else procs
    if procs is None:
        return None
    if exclude is None:
        exclude = _self_and_ancestors(procs)
    found = {}
    for p in procs.values():
        if p.pid in exclude or program(p.name) not in PROGRAMS or is_holder(p):
            continue
        if in_repo(p.exe, repo) or in_repo(p.cmd, repo) or marked(p.cmd):
            found[p.pid] = p
    grew = True
    while grew:                                     # + their children (a gateway worker, a test's simulator)
        grew = False
        for p in procs.values():
            if p.pid not in found and p.pid not in exclude and p.ppid in found and p.ppid != p.pid \
                    and program(p.name) in PROGRAMS and not is_holder(p):
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


def demo_keys(procs=None):
    """{'pid:created'} of the running demo (demo_run/pids.json entries whose process still exists); None when the
    process list can't be read."""
    keys = set()
    try:
        with open(DEMO_PIDS, encoding="utf-8") as f:
            pids = json.load(f)
    except (OSError, ValueError):
        return keys
    procs = processes() if procs is None else procs
    if procs is None:
        return None
    for name, pid in pids.items():
        if name == "_id" or not isinstance(pid, int) or pid not in procs:
            continue
        keys.add(procs[pid].key())
    return keys


# --- the split of the new processes (CL3-r MED-1) -------------------------------------------------------------------
def classify(plist, procs, base, demo, prove):
    """The project processes NOT in the baseline, split (parents before children):
       left      [Proc]          PROVED the suite's (prove(pid) is True) and not protected -> stopped, the gate fails
       keep_left [(Proc, why)]   proved the suite's but a launcher (station / gateway / demo) -> NOT stopped, fails
       protected [(Proc, why)]   the parent chain reaches a demo / baseline / protected process, or a launcher that is
                                 not proved -> never stopped, never counted
       unproven  [(Proc, why)]   no proof (prove None or False) -> reported only, never stopped, never counted
       demo_new  [Proc]          in demo_run/pids.json (a demo (re)started meanwhile) -> never a leftover
    prove = callable(pid) -> True / False / None, or None when no proof is possible at all."""
    left, keep_left, protected, unproven, demo_new = [], [], [], [], []
    shield = set()                                  # pids whose children are protected too

    def chain(p):
        seen, q = set(), p.ppid
        while q and q != p.pid and q not in seen and q in procs:
            seen.add(q)
            par = procs[q]
            if par.key() in demo:
                return "a child of the demo (PID %d)" % q
            if par.key() in base:
                return "a child of a process that ran before (PID %d)" % q
            if q in shield:
                return "a child of a protected process (PID %d)" % q
            q = par.ppid
        return None

    for p in plist:
        if p.key() in base:
            continue
        if p.key() in demo:
            demo_new.append(p)
            shield.add(p.pid)
            continue
        why = chain(p)
        if why:
            protected.append((p, why))
            shield.add(p.pid)
            continue
        pr = prove(p.pid) if prove is not None else None
        what = entry_point(p.cmd)
        if what:
            shield.add(p.pid)
            if pr is True:
                keep_left.append((p, "%s the suite started: NOT stopped by the gate - check it, then end it yourself "
                                     "(tools/strays.py stop)" % what))
            else:
                protected.append((p, "%s not started by the suite" % what))
            continue
        if pr is True:
            left.append(p)
        else:
            unproven.append((p, "not in the gate job" if pr is False else "no proof possible (no gate job)"))
    return left, keep_left, protected, unproven, demo_new


# --- stopping -------------------------------------------------------------------------------------------------------
def stop_procs(plist, wait_s=8.0, tree=False):
    """End these processes (parents first; tree = with everything under them). Right before the kill every pid is
    checked against a FRESH list (pid + creation time: never a reused pid - CL3-r LOW); no fresh list = nothing killed.
    -> the pids still alive after wait_s (or not killed)."""
    if not plist:
        return []
    fresh = processes()
    if fresh is None:
        return [p.pid for p in plist]
    todo = [p for p in plist if p.pid in fresh and fresh[p.pid].key() == p.key()]
    if os.name == "nt":
        for p in todo:
            args = ["taskkill", "/PID", str(p.pid), "/F"] + (["/T"] if tree else [])
            subprocess.run(args, capture_output=True, timeout=60, creationflags=NO_WINDOW)
    else:
        import signal
        for p in todo:
            try:
                if tree:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                else:
                    os.kill(p.pid, signal.SIGKILL)
            except OSError:
                pass
    end = time.time() + wait_s
    want = {p.pid: p.key() for p in todo}
    while True:
        now = processes()
        if now is None:
            return list(want)
        alive = [pid for pid, k in want.items() if pid in now and now[pid].key() == k]
        if not alive or time.time() > end:
            return alive
        time.sleep(0.5)


# --- Windows job objects --------------------------------------------------------------------------------------------
_W = {}


def _k32():
    """kernel32 + wintypes with the argument types of the job calls set once."""
    if not _W:
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.CreateJobObjectW.restype = wintypes.HANDLE
        k32.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
        k32.OpenJobObjectW.restype = wintypes.HANDLE
        k32.OpenJobObjectW.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR]
        k32.OpenProcess.restype = wintypes.HANDLE
        k32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        k32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
        k32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
        k32.CloseHandle.argtypes = [wintypes.HANDLE]
        k32.IsProcessInJob.argtypes = [wintypes.HANDLE, wintypes.HANDLE, ctypes.POINTER(wintypes.BOOL)]
        k32.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
        _W["k32"], _W["wt"], _W["ct"] = k32, wintypes, ctypes
    return _W["k32"], _W["wt"], _W["ct"]


def _in_job(hjob, pid):
    """True / False = the process is / is not in the job; None = unknown (no such process / no access)."""
    k32, wt, ct = _k32()
    hp = k32.OpenProcess(0x1000, False, pid)                     # PROCESS_QUERY_LIMITED_INFORMATION
    if not hp:
        return None
    try:
        b = wt.BOOL()
        if not k32.IsProcessInJob(hp, hjob, ct.byref(b)):
            return None
        return bool(b.value)
    finally:
        k32.CloseHandle(hp)


def _alive(pid):
    if os.name != "nt":
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False
    k32, wt, ct = _k32()
    hp = k32.OpenProcess(0x1000, False, pid)
    if not hp:
        return False
    try:
        code = wt.DWORD()
        return bool(k32.GetExitCodeProcess(hp, ct.byref(code))) and code.value == 259      # STILL_ACTIVE
    finally:
        k32.CloseHandle(hp)


class Job(object):
    """A Windows job object with KILL_ON_JOB_CLOSE: every process started with popen() and everything THEY start
    ends with kill() - or by the OS when this process dies (the handle closes). POSIX: a process group per popen.
    contains(pid) = the PROOF that a process was started under this job (cmd_check(job=...))."""

    def __init__(self):
        self.h = None
        self.unassigned = set()                     # pids popen() could NOT put into the job (kill() takes their tree)
        self.pgids = set()                          # POSIX: the process groups of popen()
        if os.name != "nt":
            return
        try:
            k32, wt, ct = _k32()
            h = k32.CreateJobObjectW(None, None)

            class BASIC(ct.Structure):
                _fields_ = [("a", ct.c_int64), ("b", ct.c_int64), ("LimitFlags", wt.DWORD),
                            ("MinWS", ct.c_size_t), ("MaxWS", ct.c_size_t),
                            ("ActiveProcessLimit", wt.DWORD), ("Affinity", ct.c_size_t),
                            ("PriorityClass", wt.DWORD), ("SchedulingClass", wt.DWORD)]

            class IOC(ct.Structure):
                _fields_ = [("c%d" % i, ct.c_uint64) for i in range(6)]

            class EXT(ct.Structure):
                _fields_ = [("Basic", BASIC), ("Io", IOC), ("ProcessMemoryLimit", ct.c_size_t),
                            ("JobMemoryLimit", ct.c_size_t), ("PeakProcessMemoryUsed", ct.c_size_t),
                            ("PeakJobMemoryUsed", ct.c_size_t)]
            info = EXT()
            info.Basic.LimitFlags = 0x2000              # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if h and k32.SetInformationJobObject(wt.HANDLE(h), 9, ct.byref(info), ct.sizeof(info)):
                self.h, self.k32, self.wt = h, k32, wt
            elif h:
                k32.CloseHandle(h)
        except Exception:
            self.h = None

    def assign(self, p):
        """Put a subprocess.Popen into the job. -> True when the OS will end it with the job."""
        if os.name != "nt" or not self.h:
            return False
        hp = self.k32.OpenProcess(0x0101, False, p.pid)             # PROCESS_TERMINATE | PROCESS_SET_QUOTA
        ok = False
        if hp:
            ok = bool(self.k32.AssignProcessToJobObject(self.h, hp))
            self.k32.CloseHandle(hp)
        return ok

    def popen(self, args, **kw):
        """Popen inside the job. The child runs for a moment before the assignment (CL3-r LOW: a grandchild it starts
        in that gap is outside the job - Popen gives no thread handle to start it suspended); a failed assignment is
        remembered (kill() ends that child's tree itself)."""
        if os.name != "nt":
            kw.setdefault("start_new_session", True)
            p = subprocess.Popen(args, **kw)
            self.pgids.add(p.pid)
            return p
        p = subprocess.Popen(args, **kw)
        if not self.assign(p):
            self.unassigned.add(p.pid)
        return p

    def contains(self, pid):
        """True / False = pid was / was not started under this job; None = unknown (no job / no such process)."""
        if os.name != "nt":
            try:
                return os.getpgid(pid) in self.pgids
            except OSError:
                return None
        if not self.h:
            return None
        return _in_job(self.h, pid)

    def kill(self, p=None):
        """End the whole job (and p with its tree when it is not in the job) and wait a little."""
        if os.name != "nt":
            if p is not None:
                try:
                    os.killpg(p.pid, 9)
                except OSError:
                    pass
        else:
            if self.h:
                self.k32.TerminateJobObject(self.h, 1)
            if p is not None and (not self.h or p.pid in self.unassigned) and p.poll() is None:
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
            self.k32.CloseHandle(self.h)
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


# --- the gate's job: the shell in a named job, a holder keeps its name alive ---------------------------------------
def gate_begin(st, shell_pid, out=print):
    """Windows: create the gate job `Local\\trygons_gate_<tag>_<time>`, put the SHELL that runs the gate into it
    (everything the shell starts from now on - the suites, their children - is in the job: PROVABLE), start a holder
    process that keeps a handle (a named kernel object loses its name with its last handle; the holder is not in the
    job - its parent, the `mark` process, was started before the assignment). st gets job / shell / holder / job_err."""
    st["job"], st["shell"], st["holder"], st["job_err"] = None, shell_pid, None, None
    if os.name != "nt":
        st["job_err"] = "POSIX: no job objects - new processes are reported, never proven"
        out("no gate job on this OS: new processes can only be reported, never proven")
        return False
    name = "Local\\trygons_gate_%s_%d" % (repo_tag(), int(time.time() * 1000) % 10 ** 9)
    h = None
    try:
        k32, wt, ct = _k32()
        h = k32.CreateJobObjectW(None, name)
        if not h:
            raise OSError("CreateJobObjectW: error %d" % ct.get_last_error())
        hp = k32.OpenProcess(0x0101, False, shell_pid)
        if not hp:
            raise OSError("OpenProcess(shell %d): error %d" % (shell_pid, ct.get_last_error()))
        ok = k32.AssignProcessToJobObject(h, hp)
        err = ct.get_last_error()
        k32.CloseHandle(hp)
        if not ok:
            raise OSError("AssignProcessToJobObject(shell %d): error %d" % (shell_pid, err))
        holder = subprocess.Popen([sys.executable, os.path.abspath(__file__), "hold", name, str(shell_pid)],
                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                                  text=True, cwd=HERE, creationflags=NO_WINDOW)
        line = holder.stdout.readline().strip()
        if line != "HELD":
            try:
                holder.kill()
            except OSError:
                pass
            raise OSError("the gate job holder did not start (%r)" % line)
        st["job"], st["holder"] = name, holder.pid
        out("gate job %s: the shell PID %d is in it (what the suites start is provable); holder PID %d"
            % (name, shell_pid, holder.pid))
        return True
    except Exception as e:                           # noqa: BLE001 - everything = no proof, said loudly
        st["job_err"] = str(e)
        out("GATE JOB NOT AVAILABLE (%s): new processes can only be REPORTED, never proven - the report FAILS" % e)
        return False
    finally:
        if h:
            _k32()[0].CloseHandle(h)


FORK_WRAPPER_S = 3                              # an MSYS fork wrapper (same program as its parent) is younger than this


def gate_shell(procs, shell_pid=None):
    """The shell that runs the gate = the process to put into the gate job: shell_pid when given (check.sh passes
    `cat /proc/$$/winpid`), checked to be an ANCESTOR of this process; else the nearest ancestor that is not a
    transient MSYS fork wrapper (under MSYS bash / sh every command's Windows parent is a fresh copy of the shell that
    dies with the command - the script shell is the grandparent). NEVER higher: the owner's interactive shell would
    make everything the owner starts later "provable". -> (pid, None) or (None, why)."""
    chain, p, seen = [], os.getpid(), set()
    while p in procs and p not in seen:
        seen.add(p)
        chain.append(p)
        p = procs[p].ppid
    if shell_pid is not None:
        if shell_pid in chain:                      # an ancestor, or this process itself (a Python runner / the selftest)
            return shell_pid, None
        return None, "the given shell PID %d is not an ancestor of this process (%s)" % (
            shell_pid, " <- ".join("%d %s" % (q, procs[q].name) for q in chain[:4]))
    for q in chain[1:]:
        par = procs[q].ppid
        if par in procs and procs[par].name == procs[q].name and procs[q].age_s() < FORK_WRAPPER_S:
            continue                                # a fork wrapper of the same shell: look one up
        return q, None
    return None, "no ancestor found"


def gate_prover(st):
    """callable(pid) -> True / False / None for the gate job of this state, or None when the job can't be opened."""
    name = st.get("job")
    if not name or os.name != "nt":
        return None
    k32, wt, ct = _k32()
    h = k32.OpenJobObjectW(JOB_QUERY, False, name)
    if not h:
        return None
    return lambda pid: _in_job(h, pid)                # the handle lives with this process (short)


def cmd_hold(name, shell_pid):
    """The gate job holder: opens the job by name, says HELD, and keeps the handle until the shell is gone (or
    `report` ends it, or HOLDER_MAX_S)."""
    if os.name != "nt":
        return 1
    k32, wt, ct = _k32()
    h = k32.OpenJobObjectW(JOB_QUERY, False, name)
    if not h:
        print("NO JOB %d" % ct.get_last_error(), flush=True)
        return 1
    print("HELD", flush=True)
    end = time.time() + HOLDER_MAX_S
    while time.time() < end and _alive(shell_pid):
        time.sleep(2.0)
    k32.CloseHandle(h)
    return 0


def _end_holder(st, procs):
    pid = st.get("holder")
    if pid and procs and pid in procs and is_holder(procs[pid]):
        stop_procs([procs[pid]], 3.0)
    st["holder"] = None


# --- the gate: mark / check / report -------------------------------------------------------------------------------
def _load(path):
    """The state, or None when the file is missing / unreadable (a LOUD failure for check / report: never an empty
    baseline that blames everything on the next suite - CL3-r MED-2)."""
    try:
        with open(path, encoding="utf-8") as f:
            st = json.load(f)
        return st if isinstance(st, dict) else None
    except (OSError, ValueError):
        return None


def _fresh_state():
    return {"baseline": {}, "leftovers": [], "unproven": [], "preexisting": [], "suites": [], "read_failures": [],
            "proof_missing": []}


def _save(path, st):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(st, f, indent=1)


def _row(p, now):
    return {"pid": p.pid, "name": p.name, "age_s": p.age_s(now), "cmd": p.cmd[:300]}


def cmd_mark(path, begin, out=print, gate=False, shell_pid=None):
    """Remember what runs now. begin = a fresh state (the gate starts); gate = also put the shell that runs the gate
    (shell_pid, else gate_shell()'s nearest stable ancestor) into the gate's job object (the CLI `mark --begin`; the
    runners use their own Job)."""
    now = time.time()
    procs = processes()
    if procs is None:
        out("PROCESS LIST NOT READABLE (%s): the mark FAILED - the state is marked broken" % LAST_ERROR)
        if begin:
            _save(path, dict(_fresh_state(), broken="mark --begin: " + LAST_ERROR))
        return 3
    if begin:
        st = _fresh_state()
        st["began"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if gate:
            pid, why = gate_shell(procs, shell_pid)
            if pid is None:
                st["job"], st["shell"], st["holder"], st["job_err"] = None, None, None, why
                out("GATE JOB NOT AVAILABLE (%s): new processes can only be REPORTED, never proven - the report FAILS"
                    % why)
            else:
                gate_begin(st, pid, out)
    else:
        st = _load(path)
        if st is None:
            out("STATE FILE NOT READABLE (%s): the mark FAILED" % path)
            return 3
    plist = ours(procs)
    st["baseline"] = {p.key(): _row(p, now) for p in plist}
    if begin:
        demo = demo_keys(procs)
        st["preexisting"] = [dict(_row(p, now), demo=p.key() in demo) for p in plist]
        for p in plist:
            out("already running before this run (not a leftover of it)%s: %s PID %d age %d s -- %s"
                % (" [DEMO]" if p.key() in demo else "", p.name, p.pid, p.age_s(now), p.cmd[:160]))
    _save(path, st)
    return 0


def cmd_check(path, suite, keep=False, out=print, job=None, prove=None):
    """New project processes since the mark: the PROVED ones (in the gate job / the runner's job) that are not
    protected = leftovers of this suite (printed, stopped unless keep, remembered; -> 1); protected / unproven ones
    are reported only (-> 0). The baseline moves on. -> 3 when the check was not possible (nothing stopped, the
    baseline unchanged). job = the runner's Job (its contains() is the proof); prove = a prover for tests."""
    now = time.time()
    st = _load(path)
    if st is None:
        out("STATE FILE NOT READABLE (%s): the check after %s is NOT possible - nothing stopped" % (path, suite))
        return 3
    if st.get("broken"):
        out("STATE BROKEN (%s): the check after %s is NOT possible - nothing stopped" % (st["broken"], suite))
        return 3
    procs = processes()
    if procs is None:
        out("PROCESS LIST NOT READABLE after %s (%s): nothing stopped, the baseline unchanged - the check FAILS"
            % (suite, LAST_ERROR))
        st.setdefault("read_failures", []).append(suite)
        _save(path, st)
        return 3
    plist = ours(procs)
    demo = demo_keys(procs)
    base = set(st.get("baseline", {}))
    if prove is None:
        prove = job.contains if job is not None else gate_prover(st)
        if os.name == "nt" and (prove is None or (job is not None and not job.h)):
            st.setdefault("proof_missing", []).append(suite)      # said by the report: the gate lost its proof
    left, keep_left, protected, unproven, demo_new = classify(plist, procs, base, demo, prove)
    for p in left:
        out("LEFTOVER after %s: %s PID %d (parent %d) age %d s -- %s" % (suite, p.name, p.pid, p.ppid, p.age_s(now),
                                                                         p.cmd[:200]))
    for p, why in keep_left:
        out("LEFTOVER after %s (%s): %s PID %d (parent %d) age %d s -- %s"
            % (suite, why, p.name, p.pid, p.ppid, p.age_s(now), p.cmd[:200]))
    for p in demo_new:
        out("  demo (demo_run/pids.json), not a leftover: %s PID %d age %d s -- %s" % (p.name, p.pid, p.age_s(now),
                                                                                         p.cmd[:160]))
    for p, why in protected:
        out("  not counted, never stopped (%s): %s PID %d age %d s -- %s" % (why, p.name, p.pid, p.age_s(now),
                                                                               p.cmd[:160]))
    for p, why in unproven:
        out("  NEW project process, %s: reported only, NOT stopped, not counted: %s PID %d (parent %d) age %d s -- %s"
            % (why, p.name, p.pid, p.ppid, p.age_s(now), p.cmd[:160]))
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
    for p, why in keep_left:
        st.setdefault("leftovers", []).append(dict(_row(p, now), suite=suite, ppid=p.ppid, stopped=False, why=why))
    for p, why in unproven + protected:
        st.setdefault("unproven", []).append(dict(_row(p, now), suite=suite, ppid=p.ppid, why=why))
    st.setdefault("suites", []).append(suite)
    # the baseline moves on: what runs now (the stopped leftovers are gone; a kept / unproven one is not blamed twice)
    after = ours() if left else plist
    if after is not None:
        st["baseline"] = {p.key(): _row(p, now) for p in after}
    _save(path, st)
    return 1 if (left or keep_left) else 0


def cmd_report(path, out=print):
    """-> 0 = no leftover and every check was possible; 1 otherwise. Ends the gate job holder."""
    st = _load(path)
    if st is None:
        out("NO-LEFTOVERS CHECK FAILED: the state file %s is missing / unreadable" % path)
        return 1
    rc = 0
    if st.get("broken"):
        out("NO-LEFTOVERS CHECK FAILED: %s" % st["broken"])
        rc = 1
    for s in st.get("read_failures", []):
        out("NO-LEFTOVERS CHECK FAILED after %s: the process list was not readable (nothing was stopped)" % s)
        rc = 1
    if os.name == "nt" and (st.get("job_err") or st.get("proof_missing")):
        out("NO-LEFTOVERS CHECK WITHOUT PROOF: %s%s - new processes were reported only"
            % (st.get("job_err") or "the gate job could not be opened",
               (" (after: %s)" % ", ".join(st["proof_missing"])) if st.get("proof_missing") else ""))
        rc = 1
    left = st.get("leftovers", [])
    n = len(st.get("suites", []))
    unp = st.get("unproven", [])
    if left:
        out("LEFTOVER PROCESSES: %d (in %d suite(s) checked)" % (len(left), n))
        for r in left:
            out("  %-28s %-16s PID %-7d age %5d s  %s -- %s" % (r.get("suite", "?"), r["name"], r["pid"], r["age_s"],
                                                               "stopped" if r.get("stopped") else "STILL RUNNING",
                                                               r["cmd"][:140]))
        rc = 1
    elif rc == 0:
        out("NO LEFTOVER PROCESSES: %d suite(s) checked" % n)
    if unp:
        out("  note: %d new project process(es) not proved to be a suite's (never stopped, not counted):" % len(unp))
        for r in unp[:12]:
            out("    after %-24s %-14s PID %-7d %s -- %s" % (r.get("suite", "?")[:24], r["name"], r["pid"],
                                                            r.get("why", ""), r["cmd"][:100]))
    _end_holder(st, processes())
    _save(path, st)
    return rc


def cmd_list(out=print):
    now = time.time()
    procs = processes()
    if procs is None:
        out("PROCESS LIST NOT READABLE: %s" % LAST_ERROR)
        return 3
    plist = ours(procs)
    demo = demo_keys(procs)
    holders = [p for p in procs.values() if is_holder(p) and in_repo(p.cmd)]
    if not plist and not holders:
        out("no process of this project is running")
        return 0
    for p in plist:
        tag = "  [DEMO]" if p.key() in demo else ("  [%s]" % entry_point(p.cmd) if entry_point(p.cmd) else "")
        out("%-16s PID %-7d parent %-7d age %6d s%s  %s" % (p.name, p.pid, p.ppid, p.age_s(now), tag, p.cmd[:160]))
    for p in holders:
        out("%-16s PID %-7d parent %-7d age %6d s  [gate job holder - a gate is running]" % (p.name, p.pid, p.ppid,
                                                                                               p.age_s(now)))
    return 0


def cmd_stop(all_=False, yes=False, out=print, ask=None):
    """The OWNER's command: end the project's processes (trees) after a confirmation (ask / a TTY question, or
    --yes). The demo and everything under it are kept unless --all."""
    procs = processes()
    if procs is None:
        out("PROCESS LIST NOT READABLE: %s - nothing stopped" % LAST_ERROR)
        return 3
    plist = ours(procs)
    demo = demo_keys(procs)
    if not all_:                                     # the demo tree (pids.json + children) is kept
        _, _, prot, unp, dnew = classify(plist, procs, set(), demo, lambda pid: True)
        kept = {p.pid for p in dnew} | {p.pid for p, why in prot if "demo" in why}
    else:
        kept = set()
    skip = [p for p in plist if p.pid in kept]
    todo = [p for p in plist if p.pid not in kept]
    for p in skip:
        out("kept (the demo; STOP_DEMO.bat or --all): %s PID %d" % (p.name, p.pid))
    if not todo:
        out("nothing to stop")
        return 0
    for p in todo:
        what = entry_point(p.cmd)
        out("to stop: %s PID %d%s -- %s" % (p.name, p.pid, (" [%s]" % what) if what else "", p.cmd[:140]))
    if not yes:
        if ask is None:
            if not sys.stdin.isatty():
                out("not stopped: add --yes (no terminal to ask)")
                return 2
            ask = lambda q: input(q).strip().lower() in ("y", "yes")        # noqa: E731
        if not ask("Stop these %d process(es)? [y/N] " % len(todo)):
            out("not stopped")
            return 2
    alive = stop_procs(todo, tree=True)
    if alive:
        out("STILL RUNNING: %s" % ", ".join(str(a) for a in alive))
        return 1
    out("stopped %d process(es)" % len(todo))
    return 0


# --- selftest -------------------------------------------------------------------------------------------------------
def selftest(out=print):
    global DEMO_PIDS
    bad = []
    py = sys.executable
    sleeper = "import sys, time; time.sleep(120)"
    kids = []
    states = []
    foreign_dir = tempfile.mkdtemp(prefix="elsewhere_")          # no project marker in its name
    state = os.path.join(foreign_dir, "state.json")
    demo_pids_saved = DEMO_PIDS
    gate_st = None

    def start(*args, **kw):
        p = subprocess.Popen([py, "-c", sleeper] + list(args), cwd=foreign_dir, creationflags=NO_WINDOW, **kw)
        kids.append(p)
        return p

    def alive_pid(pid):
        procs = processes()
        return procs is not None and pid in procs

    def pidin(pid, line):
        """The line names THIS pid ('PID 1234 ' - never 1234 as a part of another pid / an age)."""
        return (" PID %d " % pid) in line + " "

    try:
        # (1) ours by the command line (the repo folder in an argument), (2) foreign: no marker, parent = this
        # process (excluded) - like the owner's other Python programs; (3) a SIBLING folder (../review-wt, a clone
        # next to the repo) is NOT this project; (4) ours by a test temp folder marker
        mine = start(os.path.join(REPO, "sim"))
        foreign = start(os.path.join(foreign_dir, "x"))
        sibling = start(os.path.join(os.path.dirname(REPO), "review-wt", "sim"), REPO + "_other")
        tmpm = start(os.path.join(tempfile.gettempdir(), "trygons_t1", "x"))
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
        for c, want in ((REPO + "\\groundstation\\app.py gw:1.2.3.4:5761", "a ground station"),
                        ("python " + REPO + "/gateway/gateway.py --config x", "a gateway"),
                        (REPO + "\\demo\\demo.py start", "the demo launcher"), (REPO + "\\sim\\autopilot_sim.exe server", None),
                        (REPO + "/groundstation/test_screen2.py", None)):
            if entry_point(c) != want:
                bad.append("entry_point(%r) = %r" % (c, entry_point(c)))
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
        kid_of = lambda pp: [p for p in procs.values() if p.ppid == pp.pid and program(p.name) == "python"]  # noqa: E731
        if not kid_of(par) or any(k.pid not in pids for k in kid_of(par)):
            bad.append("the child of a project process was NOT matched")
        if any(k.pid in pids for k in kid_of(fpar)) or fpar.pid in pids:
            bad.append("NEGATIVE CONTROL FAILED: the child of a foreign process was matched")
        order = [p.pid for p in plist]
        if par.pid in order and any(order.index(k.pid) < order.index(par.pid) for k in kid_of(par) if k.pid in order):
            bad.append("parents are not listed before their children")
        for k in (mine, tmpm, par, fpar, foreign, sibling):
            k.kill()
        for k in kids:
            try:
                k.wait(10)
            except subprocess.TimeoutExpired:
                pass
        kids = []
        # (6) the GATE with its job: mark --begin puts THIS process (the "shell") into the named gate job; `before` was
        # started before that = not in the job; everything started after it is
        before = start(os.path.join(REPO, "sim"))
        lines = []
        rc0 = cmd_mark(state, True, lines.append, gate=True, shell_pid=os.getpid())
        gate_st = _load(state)
        if rc0 != 0 or gate_st is None:
            bad.append("mark --begin failed: %r" % lines[-2:])
        elif os.name == "nt" and not gate_st.get("job"):
            bad.append("the gate job was not created: %s" % gate_st.get("job_err"))
        prover = gate_prover(gate_st) if gate_st else None
        late = start(os.path.join(REPO, "sim"))
        time.sleep(1.0)
        if os.name == "nt":
            if prover is None:
                bad.append("the gate job can't be opened by name after the mark process closed its handle (holder?)")
            else:
                if prover(late.pid) is not True:
                    bad.append("a process started after the mark is NOT in the gate job (proof broken)")
                if prover(before.pid) is not False:
                    bad.append("a process started BEFORE the mark is in the gate job (proof too wide)")
                # the Job library: contains() is the runner's proof
                jb = Job()
                jk = jb.popen([py, "-c", sleeper, os.path.join(REPO, "x")], cwd=foreign_dir, creationflags=NO_WINDOW)
                kids.append(jk)
                if jb.contains(jk.pid) is not True or jb.contains(late.pid) is not False:
                    bad.append("Job.contains: own child %s, another process %s" % (jb.contains(jk.pid),
                                                                                  jb.contains(late.pid)))
                jb.kill(jk)
                jb.close()
        # (c) a process started INSIDE the gate job and left behind IS a leftover: named, stopped, exit 1; `before`
        # was in the baseline: untouched
        rc = cmd_check(state, "suite X", out=lines.append)
        if rc != 1 or not any(("LEFTOVER after suite X" in l and pidin(late.pid, l)) for l in lines):
            bad.append("check did not report the leftover: %r" % lines[-3:])
        if late.poll() is None:
            time.sleep(1.0)
        if late.poll() is None:
            bad.append("check did not stop the leftover")
        if before.poll() is not None:
            bad.append("check stopped a process of the baseline")
        rc2 = cmd_check(state, "suite Y", out=lines.append)
        if rc2 != 0:
            bad.append("a clean suite reported a leftover")
        # (a) a DEMO-like tree (re)started after the mark: the parent in a pids.json, its child NOT - both in the gate
        # job (as the tour-pictures step's demo restart) - neither is stopped, the gate does not fail
        DEMO_PIDS = os.path.join(foreign_dir, "pids.json")
        dpar = subprocess.Popen([py, "-c", cmd_rule, os.path.join(REPO, "gateway")], cwd=foreign_dir,
                                creationflags=NO_WINDOW)
        kids.append(dpar)
        with open(DEMO_PIDS, "w", encoding="utf-8") as f:
            json.dump({"gateway": dpar.pid, "_id": {}}, f)
        # (b) a station-like command line (groundstation/app.py) in the job, (b2) a plain project process that is NOT
        # provable (the prover says False: started outside the job - the owner's), (b3) a station the prover says is
        # NOT the suite's
        station = start(os.path.join(REPO, "groundstation", "app.py"), "gw:127.0.0.1:5761")
        plain = start(os.path.join(REPO, "sim"))
        station2 = start(os.path.join(REPO, "groundstation", "app.py"), "tcp:127.0.0.1:5760")
        time.sleep(2.0)
        procs = processes()
        dkid = kid_of(dpar)
        if not dkid:
            bad.append("the demo-like tree did not start")
        real = gate_prover(gate_st) if gate_st and gate_st.get("job") else None
        tell = lambda pid: (False if pid in (plain.pid, station2.pid) else (real(pid) if real else True))  # noqa: E731
        l2 = []
        rc3 = cmd_check(state, "suite Z", out=l2.append, prove=tell)
        txt = "\n".join(l2)
        if rc3 != 1:
            bad.append("suite Z: expected exit 1 (the station the suite started is a leftover), got %d: %r" % (rc3, l2))
        if dpar.poll() is not None or any(k.pid for k in dkid if not alive_pid(k.pid)):
            bad.append("(a) the demo-like tree was STOPPED by the gate")
        if dkid and not any(("not counted" in l and pidin(dkid[0].pid, l) and "demo" in l) for l in l2):
            bad.append("(a) the demo's child was not reported as protected: %r" % l2)
        if any(("LEFTOVER" in l and (pidin(dpar.pid, l) or any(pidin(k.pid, l) for k in dkid))) for l in l2):
            bad.append("(a) a demo process was counted as a leftover")
        if station.poll() is not None:
            bad.append("(b) a ground station the suite started was KILLED by the gate")
        if not any(("LEFTOVER after suite Z" in l and pidin(station.pid, l) and "NOT stopped" in l) for l in l2):
            bad.append("(b) the station the suite started was not reported as a leftover NOT stopped: %r" % l2)
        if plain.poll() is not None:
            bad.append("(b2) an unproven project process was KILLED by the gate")
        if not any(("reported only" in l and pidin(plain.pid, l)) for l in l2):
            bad.append("(b2) the unproven process was not reported: %r" % l2)
        if station2.poll() is not None or any(("LEFTOVER" in l and pidin(station2.pid, l)) for l in l2):
            bad.append("(b3) a station NOT started by the suite was killed / counted")
        if not any(("never stopped" in l and pidin(station2.pid, l)) for l in l2):
            bad.append("(b3) the owner's station was not reported as protected: %r" % l2)
        # the protection does not depend on "unproven": with a prover that says True for EVERYTHING the demo tree and
        # the stations are still never in `left`
        pl = ours(procs)
        st_now = _load(state)
        left_all, keep_all, prot_all, unp_all, dn_all = classify(pl, procs, set(), demo_keys(procs), lambda pid: True)
        lp = {p.pid for p in left_all}
        if dpar.pid in lp or any(k.pid in lp for k in dkid) or station.pid in lp or station2.pid in lp:
            bad.append("classify(prove=True): a demo / station process in `left` (would be killed)")
        if dpar.pid not in {p.pid for p in dn_all} or not any(k.pid in {q.pid for q, w in prot_all} for k in dkid):
            bad.append("classify: the demo tree is not demo_new + protected")
        if plain.pid not in lp:
            bad.append("classify(prove=True): a plain provable process is not in `left`")
        # the suite Z leftovers are remembered unstopped; the next check does not blame them again
        rc4 = cmd_check(state, "suite W", out=l2.append, prove=tell)
        if rc4 != 0:
            bad.append("the kept / unproven processes were blamed twice: %r" % l2[-4:])
        # (d) a process list that can't be read: a LOUD failure (3), nothing stopped, the baseline unchanged
        base_before = json.dumps(_load(state)["baseline"], sort_keys=True)
        late2 = start(os.path.join(REPO, "sim"))
        time.sleep(1.0)
        saved = globals()["processes"]
        globals()["processes"] = lambda retry=True: None
        try:
            l3 = []
            rc5 = cmd_check(state, "suite V", out=l3.append)
        finally:
            globals()["processes"] = saved
        if rc5 != 3 or not any("NOT READABLE" in l for l in l3):
            bad.append("(d) a failed process list read was not a loud failure: %d %r" % (rc5, l3))
        if late2.poll() is not None:
            bad.append("(d) something was stopped after a failed process list read")
        if json.dumps(_load(state)["baseline"], sort_keys=True) != base_before:
            bad.append("(d) the baseline changed after a failed process list read")
        late2.kill()
        late2.wait(10)
        # the real read of a broken list: PowerShell / ps failing = None, never {}
        saved_ps = globals()["_PS"]
        globals()["_PS"] = "throw 'broken'"
        try:
            if os.name == "nt" and processes(retry=False) is not None:
                bad.append("processes() with a failing PowerShell did not return None")
        finally:
            globals()["_PS"] = saved_ps
        # a missing state file = loud failure, never an empty baseline
        l4 = []
        if cmd_check(os.path.join(foreign_dir, "nothere.json"), "suite U", out=l4.append) != 3 or \
                cmd_report(os.path.join(foreign_dir, "nothere.json"), l4.append) != 1:
            bad.append("a missing state file was not a loud failure: %r" % l4)
        # the report: suite X stopped, suite Z's station STILL RUNNING, suite Y / W clean, (d) read failure = FAIL
        rl = []
        rc6 = cmd_report(state, rl.append)
        if rc6 != 1 or not any("suite X" in l and "stopped" in l for l in rl) or any("suite Y" in l for l in rl) \
                or not any("suite Z" in l and "STILL RUNNING" in l for l in rl) \
                or not any("suite V" in l and "not readable" in l for l in rl):
            bad.append("report wrong: %r" % rl)
        holder = gate_st.get("holder") if gate_st else None
        if os.name == "nt" and holder:
            time.sleep(1.0)
            if alive_pid(holder):
                bad.append("report did not end the gate job holder")
        for k in (dpar, station, plain, station2, before):
            k.kill()
        for k in dkid:
            stop_procs([k], 3.0)
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
            alive = alive_pid(gpid)
            if mode == "job":
                if os.name == "nt" and (not ok or alive):
                    bad.append("dies_with_parent: grandchild %s (assigned %s)" % ("ALIVE" if alive else "gone", ok))
            else:
                if os.name == "nt" and not alive:
                    bad.append("without the job the grandchild died too (the selftest proves nothing)")
            if alive:
                stop_procs([Proc(gpid, 0, "python", "", "", 0)], 5.0)
    finally:
        DEMO_PIDS = demo_pids_saved
        for k in kids:
            try:
                k.kill()
            except OSError:
                pass
        procs = processes()
        if procs is not None:
            if gate_st is not None:
                _end_holder(gate_st, procs)
            for p in ours(procs) or []:               # whatever the selftest started and still runs
                if "time.sleep(120)" in p.cmd:
                    stop_procs([p], 3.0, tree=True)
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
        return cmd_stop("--all" in rest, "--yes" in rest)
    if cmd == "mark":
        shell = None
        if "--shell" in rest:
            try:
                shell = int(rest[rest.index("--shell") + 1])
            except (IndexError, ValueError):
                shell = None                         # an empty value (no /proc/$$/winpid) = the fallback
        return cmd_mark(path, "--begin" in rest, gate="--begin" in rest, shell_pid=shell)
    if cmd == "check":
        suite = " ".join(x for x in rest if not x.startswith("--")) or "?"
        return cmd_check(path, suite, "--keep" in rest)
    if cmd == "report":
        return cmd_report(path)
    if cmd == "hold":
        return cmd_hold(rest[0], int(rest[1]))
    if cmd == "tag":
        print(repo_tag())
        return 0
    if cmd == "selftest":
        return selftest()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
