"""The talking manual, step AM2 (CLAUDE.md section W, docs/assistant_design.md): voice in - the worker process,
push-to-talk, its own microphone, term hints, the read-only proof.

  AS1 static imports: no assistant module imports a network / serial / link / gateway / protocol / station-sender
      module (socket, asyncio, serial, urllib, http, requests, ssl, gwclient, link, proto, stations, app, aurix,
      hailer ...); only client.py may import subprocess (the ONE process door); worker / stt / mic / hints /
      paths start no process (no subprocess / multiprocessing / os.system / os.popen)
  AS2 the transitive import graph through groundstation/ (+ gateway/): nothing reached from an assistant module
      is a link / gateway / protocol / station-sender module. LIMIT (AM2 review MED-3): third-party libraries
      (faster-whisper -> huggingface_hub -> httpx ...) are NOT banned - network code by design; a static walk of
      the assistant Python's site-packages LISTS the network modules reached. The no-network proof of the worker
      rests on the runtime blocker (AS3 / AS4), local model folders and the offline environment (AS9)
  AS3 runtime: a whole question through the REAL worker process (stub recogniser, simulated microphone) with the
      station's senders (app.App send* / cmd, gwclient.GatewayLink.send*, link.Link.send) and socket patched to
      fail - in this process AND in the worker (socket blocked): the question still comes back, nothing was sent
  AS4 the REAL models with the network blocked in the worker: a spoken Greek + English question (offline voice)
      is recognised (N/A - never PASS - when the models / the offline voice are not installed)
  AS5 push-to-talk: button + key only (never the joystick), < 0.4 s ignored, 15 s = asked automatically, the key's
      auto-repeat, the key ignored while a text field has focus / another key; the shore joystick modules never
      reach the assistant
  AS6 the loudhailer (AM1 review H1): «ΟΜΙΛΙΑ» pressed = the capture aborted + the assistant's voice stopped at
      once (also through hailer.Hailer.press); while this station holds the horn: no capture, voice stopped
  AS7 its OWN microphone setting (assistant_mic), separate from the loudhailer's devices; mic.py never imports
      audiodev; nothing writes sound to disk (the log folder holds text only)
  AS8 crash / hang: «ΒΟΗΘΟΣ ΕΚΤΟΣ» (a station-local notice on both screens, never an alarm), restart after 5 s,
      at most 3 restarts in 10 minutes then off; a hang (no answer in 30 s) = restarted
  AS9 never on a gateway PC (client + worker refuse); a model NAME instead of a local folder is refused (AM1 review
      M2), the worker's environment is offline (HF_HUB_OFFLINE / TRANSFORMERS_OFFLINE)
  AS10 term hints from lang/el.txt + en.txt (mode bar, Tools pages, alarm names), within the prompt budget
  AS11 the screens: «ΕΡΩΤΗΣΗ» + the "AI" badge on screen 2 bar 1 (also in the camera view) (press / release, inside the bar, the full screen 2
      still fits 1366x768 / 1920x1080 / 1366x658) and in screen 1's ONE bottom bar (press / release, inside the bar,
      not over the alarms / notices boxes); greyed while not ready; the Tools page «Βοηθός» (mic + key settings)
  AS12 log folder: a test run writes to a temp folder (TRYGONS_TEST), the real one is in .gitignore
  AS13 hash-locked downloads (AM2 review MED-4): sha256 of the Python zip + get-pip.py (a commit), pip only from
      tools/assistant_requirements*.txt with every line pinned + hashed and --require-hashes
  AM2-r (review of fb97ac2): AS3 / AS4 FAIL on any network attempt the worker made - recorded by the blocker
      (marker file + stderr, audit hook) even when a library caught the error; a negative control proves it
      (MED-2). AS3b = the AS3 traps through the REAL station wiring (ap.asst, the screen-2 button, ov_tick)
      (LOW-2). AS6: a gc.get_referents walk finds no App behind AsstView (LOW-1), a ready message / sound signal
      stop the assistant (LOW-3). AS8: no pong in 10 s = restarted + counted (LOW-5), stderr closed on a Popen
      failure + rotated (LOW-6). AS5: F10 is no PTT key (LOW-7). AS11 + test_driver I19: «ΕΡΩΤΗΣΗ» is
      >= 2 bar heights from «ΟΜΙΛΙΑ» with other buttons between (MED-1)
  AM2-r2 (review of 25cb44f): AS6 the horn state is a plain bool pushed into the view (Hailer.push_horn,
      App.ov_tick), the view keeps no station object, the gc walk skips no edge (MED-b); AS3b also records
      every sender call in its window and diffs against the same window without the assistant (LOW-4); AS4
      also checks the worker PID's TCP / UDP endpoints (netstat) + reports the OS firewall rule (LOW-1);
      AS13 the install lock marker + --make-requirements needs TRYGONS_ASSISTANT_PYTHON (LOW-2 / LOW-3)

Run:  python test_assistant.py      (exit code 0 = all passed; N/A lines do not fail)
"""
import ast
import array
import collections
import json
import shutil
import subprocess
import math
import os
import re
import sys
import tempfile
import time
import types
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import test_screen2 as ts  # noqa: E402  (temp settings, sound off, TRYGONS_TEST, the Rig on the simulator)
import i18n  # noqa: E402
import overlay  # noqa: E402
import settings  # noqa: E402
from assistant import client, hints, mic, paths, stt  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
results = []
ASST = os.path.join(HERE, "assistant")
BANNED = {"socket", "asyncio", "serial", "urllib", "http", "requests", "ssl", "ftplib", "smtplib", "telnetlib",
          "websocket", "websockets", "gwclient", "link", "proto", "stations", "app", "aurix", "hailer", "shorejoy",
          "yesno", "uavfly", "uavpanel", "gateway", "driver", "hud", "crewui", "syncrecv", "video"}
PROCESS = {"subprocess", "multiprocessing", "concurrent"}


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else ("N/A" if ok is None else "FAIL"), name, detail), flush=True)


def imports_of(path):
    """Top-level module names imported by a file (absolute + the assistant's relative imports)."""
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            out |= {a.name.split(".")[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom):
            if n.level:                                    # from . import x  -> assistant.x
                out |= {"assistant." + a.name for a in n.names} if not n.module else {"assistant." + n.module}
            elif n.module:
                base = n.module.split(".")[0]
                out.add(base)
                if base == "assistant":
                    out |= {"assistant." + a.name for a in n.names}
    return out


def asst_modules():
    return sorted(f[:-3] for f in os.listdir(ASST) if f.endswith(".py"))


# --- AS1 / AS2 ------------------------------------------------------------------------------------------------
def as1():
    bad = []
    for m in asst_modules():
        p = os.path.join(ASST, m + ".py")
        imps = {i for i in imports_of(p) if not i.startswith("assistant.")}
        b = imps & BANNED
        if b:
            bad.append((m, sorted(b)))
        if m != "client" and imps & PROCESS:
            bad.append((m, "starts a process: %s" % sorted(imps & PROCESS)))
        with open(p, encoding="utf-8") as f:
            src = f.read()
        for pat in ("os.system", "os.popen", "os.startfile", "Popen(", "spawn"):
            if m != "client" and pat in src:
                bad.append((m, pat))
    report("AS1 static imports", not bad and len(asst_modules()) >= 7,
           "modules %s; problems %s" % (asst_modules(), bad or "none"))


def resolve(name):
    """A module name -> its file in groundstation/ / groundstation/assistant/ / gateway/ (None = stdlib / site)."""
    if name.startswith("assistant."):
        p = os.path.join(ASST, name.split(".", 1)[1] + ".py")
        return p if os.path.isfile(p) else None
    for d in (HERE, os.path.join(REPO, "gateway")):
        p = os.path.join(d, name + ".py")
        if os.path.isfile(p):
            return p
    return None


def as2():
    seen, bad = {}, []
    todo = [("assistant." + m, None) for m in asst_modules()]
    while todo:
        name, via = todo.pop()
        if name in seen:
            continue
        seen[name] = via
        base = name.split(".")[-1] if name.startswith("assistant.") else name
        if not name.startswith("assistant.") and base in BANNED:
            bad.append((name, "via", via))
            continue
        p = resolve(name)
        if p is None:
            continue
        for i in imports_of(p):
            if not i.startswith("assistant.") and i in BANNED:
                bad.append((i, "via", name))
            if i not in seen:
                todo.append((i, name))
    local = sorted(n for n in seen if resolve(n))
    third = sorted(n for n in seen if not resolve(n) and not n.startswith("assistant.") and
                   n.split(".")[0] not in sys.stdlib_module_names)
    net, nfiles = site_walk(third)
    report("AS2 transitive import graph", not bad,
           "reached %s; banned %s. LIMIT (AM2 review MED-3): the third-party libraries %s are NOT banned - "
           "faster-whisper's model download (huggingface_hub) is network code by design; a static walk of the "
           "assistant Python's site-packages (%s files) reaches the network modules %s. So the no-network proof "
           "for the worker rests on the RUNTIME blocker (AS3 / AS4: every attempt recorded, even a caught one) + "
           "local model folders only + the offline environment (AS9)" % (local, bad or "none", third, nfiles,
                                                                          net or "(not installed)"))


NET_MODULES = {"socket", "ssl", "http", "urllib", "asyncio", "ftplib", "smtplib", "socketserver", "xmlrpc",
               "requests", "urllib3", "httpx", "httpx2", "httpcore", "httpcore2", "aiohttp", "websockets", "hf_xet",
               "h11", "anyio", "truststore"}


def site_walk(roots, limit=8000):
    """A static walk of the assistant Python's site-packages from the worker's third-party imports (every
    import statement, also inside try / functions = an over-approximation) -> ({network module: an importer},
    files read). ({}, 0) when the assistant Python is not installed."""
    py = paths.assistant_python()
    if not py:
        return {}, 0
    site = os.path.join(os.path.dirname(py), "Lib", "site-packages") if os.name == "nt" else None
    if not site or not os.path.isdir(site):
        return {}, 0

    def find(mod):
        b = os.path.join(site, *mod.split("."))
        for p in (b + ".py", os.path.join(b, "__init__.py")):
            if os.path.isfile(p):
                return p
        return None
    seen, todo, net, files = set(), [(r, "worker") for r in roots], {}, 0
    while todo and files < limit:
        mod, via = todo.pop()
        if mod in seen:
            continue
        seen.add(mod)
        top = mod.split(".")[0]
        if top in NET_MODULES:
            net.setdefault(top, via)
        if top in sys.stdlib_module_names:
            continue
        p = find(mod)
        if p is None:
            continue
        files += 1
        pkg = mod if p.endswith("__init__.py") else mod.rpartition(".")[0]
        try:
            with open(p, encoding="utf-8", errors="replace") as f:
                tree = ast.parse(f.read())
        except (SyntaxError, ValueError):
            continue
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                todo += [(a.name, mod) for a in n.names]
            elif isinstance(n, ast.ImportFrom):
                if n.level:
                    parts = pkg.split(".")
                    base = ".".join(parts[:len(parts) - (n.level - 1)]) if n.level > 1 else pkg
                    m = base + ("." + n.module if n.module else "")
                else:
                    m = n.module or ""
                if m:
                    todo.append((m, mod))
                    todo += [(m + "." + a.name, mod) for a in n.names if a.name != "*"]
    return dict(sorted(net.items())), files


# --- a station-side view for the client tests -------------------------------------------------------------------
class View:
    def __init__(self):
        self.horn = False
        self.stops = 0
        self.heard = []

    def lang(self):
        return "el"

    def horn_held(self):
        return self.horn

    def voice_stop(self):
        self.stops += 1

    def on_heard(self, q, info):
        self.heard.append((q, info))


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Sets:
    def __init__(self, **kw):
        self.d = dict(kw)

    def get(self, k, d=None):
        return self.d.get(k, d)


# The worker-side network blocker (AM2 review MED-2): EVERY attempt is RECORDED (a line in the marker file + a
# stderr line) before it is refused - a library that catches the OSError can't hide it: AS3 / AS4 fail when the
# marker file has a line. An audit hook sees the C level too (socket.__new__ / connect / bind / getaddrinfo ...).
BLOCKER = '''import os, socket, sys, runpy
MARK = sys.argv[1]
def _mark(what):
    try:
        with open(MARK, "a", encoding="utf-8") as f:
            f.write(what + "\\n")
        sys.stderr.write("NETWORK ATTEMPT (test_assistant): %s\\n" % what)
    except Exception:
        pass
LOCAL = ("socket.gethostname",)    # a local OS query, no traffic (CTranslate2 / CUDA read it at load)
def _hook(ev, args):
    if ev in LOCAL:
        return
    if ev.startswith("socket.") or ev in ("urllib.Request", "http.client.connect", "ftplib.connect",
                                         "smtplib.connect", "webbrowser.open"):
        _mark(ev)
        raise OSError("network blocked (test_assistant): " + ev)
sys.addaudithook(_hook)
class _Blocked(socket.socket):
    def __init__(self, *a, **k):
        _mark("socket.socket")
        raise OSError("network blocked (test_assistant)")
def _no(*a, **k):
    _mark("socket call")
    raise OSError("network blocked (test_assistant)")
socket.socket = _Blocked
socket.create_connection = _no
socket.getaddrinfo = _no
w = sys.argv[2]
sys.argv = [w] + sys.argv[3:]
runpy.run_path(w, run_name="__main__")
'''
MARKERS = []                                          # every marker file a blocked worker was given


def blocked_cmd(py, script=None):
    d = tempfile.mkdtemp(prefix="trygons_asst_")
    p = os.path.join(d, "blocked_worker.py")
    with open(p, "w", encoding="utf-8") as f:
        f.write(BLOCKER)
    mark = os.path.join(d, "network_attempts.txt")
    MARKERS.append(mark)
    return [py, p, mark, script or os.path.join(ASST, "worker.py")]


def attempts(marks):
    """The network attempts recorded in these marker files (empty = none)."""
    out = []
    for m in marks:
        if os.path.isfile(m):
            with open(m, encoding="utf-8") as f:
                out += [x.strip() for x in f if x.strip()]
    return out


def blocker_selftest(py):
    """The blocker's negative control: a script that tries the network and CATCHES the error - the attempt must
    still be in the marker file (else AS3 / AS4 could not see a hidden attempt)."""
    import subprocess
    d = tempfile.mkdtemp(prefix="trygons_asst_nc_")
    s = os.path.join(d, "tries.py")
    with open(s, "w", encoding="utf-8") as f:
        f.write("import socket\ntry:\n    socket.create_connection(('127.0.0.1', 9))\nexcept OSError:\n    pass\n"
                "try:\n    import _socket\n    _socket.socket()\nexcept OSError:\n    pass\nprint('caught')\n")
    n = len(MARKERS)
    cmd = blocked_cmd(py, s)
    r = subprocess.run(cmd, capture_output=True, timeout=60)
    got = attempts(MARKERS[n:])
    del MARKERS[n:]
    return b"caught" in r.stdout and len(got) >= 2, got


def wait(a, until, s=20.0):
    end = time.time() + s
    while time.time() < end:
        a.tick()
        if until():
            return True
        time.sleep(0.02)
    return False


def tone(sec, f=300):
    n = int(sec * mic.RATE)
    return [int(8000 * math.sin(2 * math.pi * f * i / mic.RATE)) for i in range(n)]


def stub(view=None, clock=None, py=None, **kw):
    a = client.Assistant(view or View(), settings=Sets(**kw), clock=clock or time.monotonic, stt_kind="stub",
                         stub_text="πού κλείνω την καταπακτή", mic_backend="sim")
    if py:
        a.worker_cmd = lambda: blocked_cmd(py)
    return a


def ask_once(a, clk, sec=1.0):
    if not a.press("button"):
        return None
    a.inject(tone(sec))
    clk.t += sec
    a.release()
    n = len(a.view.heard)
    wait(a, lambda: len(a.view.heard) > n or a.state != client.THINKING)
    return a.view.heard[-1][0] if len(a.view.heard) > n else None


# --- AS3 -------------------------------------------------------------------------------------------------------
def as3():
    import socket
    import app as A
    import gwclient
    import link
    sent = []
    saved = []

    def trap(cls, name):
        orig = getattr(cls, name)
        saved.append((cls, name, orig))
        setattr(cls, name, lambda *a, **k: sent.append("%s.%s" % (cls.__name__, name)) or (_ for _ in ()).throw(
            RuntimeError("a sender was called")))
    for cls in (gwclient.GatewayLink, link.Link, A.App):
        for name in list(vars(cls)):
            if name.startswith("send") or name == "cmd":
                trap(cls, name)
    real_sock, real_cc = socket.socket, socket.create_connection

    class NoSock(socket.socket):
        def __init__(self, *a, **k):
            sent.append("socket")
            raise OSError("network blocked")
    socket.socket = NoSock
    socket.create_connection = lambda *a, **k: sent.append("socket") or (_ for _ in ()).throw(OSError("blocked"))
    clk = Clock()
    n = len(MARKERS)
    a = stub(clock=clk, py=sys.executable)
    try:
        a.start()
        ready = wait(a, lambda: a.state == client.READY)
        q = ask_once(a, clk) if ready else None
    finally:
        a.stop()
        socket.socket, socket.create_connection = real_sock, real_cc
        for cls, name, orig in saved:
            setattr(cls, name, orig)
    tried = attempts(MARKERS[n:])
    nc_ok, nc = blocker_selftest(sys.executable)
    found = (a.view.heard[-1][1].get("found") or {}) if a.view.heard else {}   # AM3: the manual search, offline
    top = (found.get("hits") or [{}])[0]
    f_ok = not found.get("cant_find", True) and "ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ" in top.get("text", "")
    report("AS3 runtime: senders + network blocked", ready and q == "πού κλείνω την καταπακτή" and not sent and
           len(MARKERS) > n and not tried and nc_ok and f_ok,
           "ready %s, heard %r, senders / sockets called %s, traps %d; worker network attempts %s; the blocker's "
           "negative control (attempts caught by the script) recorded %s; the manual search in the worker (AM3) "
           "found %s %r" % (ready, q, sent or "none", len(saved), tried or "none", nc, top.get("id"),
                            top.get("text", "")[:60]))


def from_assistant():
    """True when the current call stack runs assistant code (a file in groundstation/assistant/, AsstView, or the
    App's asst_* glue) - the station's own periodic sends are not the assistant's."""
    import inspect
    import app as A
    for fr in inspect.stack()[2:]:
        f = os.path.abspath(fr.filename)
        if f.startswith(ASST + os.sep) or fr.function.startswith("asst_") or \
                isinstance(fr.frame.f_locals.get("self"), A.AsstView):
            return True
    return False


BASE_RUNS = 3


def call_key(cls, name, x, k):
    """AM2-r3 review MED: a sender call as "Class.method:<what>" - the message's "t" (gateway JSON), the frame
    type (USB / gateway frames), so a count per payload kind is compared, not just the method name."""
    arg = x[1] if len(x) > 1 else (k.get("obj") or k.get("ftype"))
    if isinstance(arg, dict):
        what = "t=%s" % arg.get("t")
    elif isinstance(arg, int):
        what = "0x%02X" % arg
    else:
        what = type(arg).__name__
    return "%s.%s:%s" % (cls.__name__, name, what)


def as3_rig(rig, mutate=None, quiet=False):
    """AM2 review LOW-2: the same traps through the REAL station wiring - ap.asst (the App's own Assistant with
    its AsstView), the screen-2 «ΕΡΩΤΗΣΗ» button, App.ov_tick draining the heard queue: no sender / socket is
    called from an assistant code path, no network attempt in the worker, the question reaches the screens.
    AM2-r3 review MED: (1) a callback that assistant code SCHEDULES (tkinter after / after_idle) is tainted - a
    sender reached from it counts as the assistant's, though no assistant frame is on the stack when it runs;
    (2) every sender call is counted per method + payload kind and compared with the MOST of BASE_RUNS windows
    without the assistant (any count above = FAIL; a periodic send may land once more in a window by its phase,
    so (2) alone can't see ONE extra periodic-kind send - (1) does). mutate(ap) = a mutation run (quiet)."""
    import socket
    import tkinter
    import app as A
    import gwclient
    import link
    ap = rig.ap
    a = ap.asst
    sent, saved = [], []
    with_asst, without, win, extra = collections.Counter(), collections.Counter(), 0.0, {}
    every = []                                         # AM2-r2 LOW-4: EVERY sender call in the window (any caller)
    samples = {}                                       # key -> (cls, name, args, kwargs) of one real call
    tainted = [0]                                      # > 0 while a callback scheduled by assistant code runs

    def trap(cls, name):
        orig = getattr(cls, name)
        saved.append((cls, name, orig))

        def t(*x, **k):
            key = call_key(cls, name, x, k)               # AM2-r3 MED: method + WHAT it sends (t / frame type)
            every.append(key)
            samples.setdefault(key, (cls, name, x, k))
            if tainted[0] or from_assistant():
                sent.append(key + (" (scheduled by the assistant)" if tainted[0] else ""))
                raise RuntimeError("a sender was called from the assistant")
            return orig(*x, **k)
        setattr(cls, name, t)
    for cls in (gwclient.GatewayLink, link.Link, A.App):
        for name in list(vars(cls)):
            if name.startswith("send") or name == "cmd":
                trap(cls, name)
    real_after, real_idle = tkinter.Misc.after, tkinter.Misc.after_idle

    def taint(func):
        def run(*x):
            tainted[0] += 1
            try:
                return func(*x)
            finally:
                tainted[0] -= 1
        return run

    def after(self_, ms, func=None, *x):
        if func is not None and (tainted[0] or from_assistant()):
            func = taint(func)
        return real_after(self_, ms, func, *x)

    def after_idle(self_, func, *x):
        if tainted[0] or from_assistant():
            func = taint(func)
        return real_idle(self_, func, *x)
    tkinter.Misc.after, tkinter.Misc.after_idle = after, after_idle
    real_sock, real_cc = socket.socket, socket.create_connection

    class NoSock(real_sock):
        def __init__(self, *x, **k):
            if tainted[0] or from_assistant():
                sent.append("socket")
                raise OSError("network blocked")
            super().__init__(*x, **k)

    def cc(*x, **k):
        if tainted[0] or from_assistant():
            sent.append("socket")
            raise OSError("network blocked")
        return real_cc(*x, **k)
    socket.socket, socket.create_connection = NoSock, cc
    keep = a.stt_kind, a.stub_text, a.mic_backend
    n = len(MARKERS)
    out = {}
    try:
        a.stop()
        # the station's own start-up reads (parameters, joystick map at connect) settle first: until two 1 s
        # slices send the same kinds (at most 20 s)
        last = None
        for _i in range(20):
            every.clear()
            rig.pump(1.0)
            now = set(every)
            if now == last:
                break
            last = now
        a.stt_kind, a.stub_text, a.mic_backend = "stub", "πού κλείνω την καταπακτή", "sim"
        a.worker_cmd = lambda: blocked_cmd(sys.executable)
        ap.asst_heard = None
        t0 = time.monotonic()
        every.clear()
        a.start()
        out["ready"] = rig.pump(20, until=lambda: a.state == client.READY) is not False and a.state == client.READY
        ap.render(force=True)
        ask = ts.op(ap, "ask")
        r = ask["r"]
        ap.ov.click((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)       # the real «ΕΡΩΤΗΣΗ» button on screen 2
        out["listening"] = a.state == client.LISTENING
        a.inject(tone(1.0))
        rig.pump(1.0)
        ap.ov.release()
        rig.pump(20, until=lambda: ap.asst_heard is not None)
        out["heard on the screens"] = bool(ap.asst_heard) and ap.asst_heard[0] == "πού κλείνω την καταπακτή" and \
            "πού κλείνω την καταπακτή" in (ap.msg_text or "")
        if mutate is not None:                          # a mutation run: sends the assistant must never make
            mutate(ap)
            rig.pump(0.5)
        a.stop()
        win = max(2.0, time.monotonic() - t0)
        with_asst = collections.Counter(every)
        # the same window WITHOUT the assistant (stopped, nothing pressed): the station's own periodic sends,
        # compared by COUNT per method + payload kind (the old set comparison hid an extra send of a kind the
        # station also sends on its own); the baseline = the MOST of BASE_RUNS such windows
        for _r in range(BASE_RUNS):
            every.clear()
            rig.pump(win)
            without |= collections.Counter(every)
        extra = {k: (c, without.get(k, 0)) for k, c in with_asst.items() if c > without.get(k, 0)}
        out["no sender call above the station's own count (vs runs without the assistant)"] = not extra
    finally:
        a.stop()
        del a.worker_cmd                              # back to the class's own worker_cmd
        a.stt_kind, a.stub_text, a.mic_backend = keep
        socket.socket, socket.create_connection = real_sock, real_cc
        tkinter.Misc.after, tkinter.Misc.after_idle = real_after, real_idle
        for cls, name, orig in saved:
            setattr(cls, name, orig)
    tried = attempts(MARKERS[n:])
    out["no sender / socket from the assistant"] = not sent
    out["no network attempt in the worker"] = len(MARKERS) > n and not tried
    ok = all(out.values())
    if quiet:
        return ok, extra, list(sent)
    # AM2-r3 review MED: the mutation checks - sends through the method + payload kind the station ALSO sends most
    # often on its own in the baseline: (a) ONE scheduled by assistant code (root.after) = caught by the taint;
    # (b) three more from a path the taint can't see (scheduled by plain code) = caught by the count comparison
    common = max(without, key=without.get) if without else None
    m = {}
    if common is not None:
        cls_, name_, x_, k_ = samples[common]

        def sneak_after(app_):
            def asst_sneak():                          # an asst_* name = assistant glue at SCHEDULING time
                app_.root.after(10, lambda: getattr(cls_, name_)(*x_, **k_))   # runs later: no assistant frame
            asst_sneak()

        def sneak_count(app_):
            for i in range(3):
                app_.root.after(10 + 20 * i, lambda: getattr(cls_, name_)(*x_, **k_))
        m_ok, _e, m_sent = as3_rig(rig, mutate=sneak_after, quiet=True)
        m["(a) scheduled by the assistant"] = (not m_ok and any(common in s and "scheduled" in s for s in m_sent),
                                               m_sent)
        m_ok, m_extra, _s = as3_rig(rig, mutate=sneak_count, quiet=True)
        m["(b) 3 extra of a station kind"] = (not m_ok and common in m_extra, m_extra)
    out["mutation: a scheduled send via an already-used method + payload is caught"] = \
        bool(m) and all(v[0] for v in m.values())
    report("AS3b the real station wiring (ap.asst) with the AS3 traps", all(out.values()),
           "%s; senders %s, worker attempts %s, traps %d; sender calls per method + payload in the window %s vs the "
           "most of %d windows without the assistant %s (%.1f s each); above the baseline %s; mutations through %s: "
           "%s" % (out, sent or "none", tried or "none", len(saved), dict(with_asst), BASE_RUNS, dict(without), win,
                   extra or "none", common, m))


# --- AS4 -------------------------------------------------------------------------------------------------------
def spoken(lang, text):
    """The offline voice (tts.py, Piper) saying text -> 16 kHz samples, or None."""
    try:
        import tts
        d = tempfile.mkdtemp(prefix="trygons_asst_wav_")
        p = os.path.join(d, "q.wav")
        made = False
        for choice in ("windows", None):          # the Windows voice reads Greek better than Piper (AM2 measurement)
            try:
                made = bool(tts.synth(lang, [(text, p)], timeout=120, choice=choice)) and os.path.isfile(p)
            except Exception:
                made = False
            if made:
                break
        if not made:
            return None
        w = wave.open(p)
        rate, n = w.getframerate(), w.getnframes()
        s = array.array("h", w.readframes(n))
        w.close()
        os.remove(p)
        os.rmdir(d)
        if rate != mic.RATE:                          # linear resample
            s = array.array("h", [s[min(len(s) - 1, int(i * rate / mic.RATE))] for i in range(n * mic.RATE // rate)])
        return list(s)
    except Exception:
        return None


def pid_endpoints(pid):
    """AM2-r2 review LOW-1: every TCP / UDP endpoint the OS lists for pid (netstat -ano; no psutil) - sees native
    code too (CTranslate2 C++, hf_xet Rust), which the Python-level blocker cannot. None = netstat failed."""
    import subprocess
    try:
        out = subprocess.run(["netstat", "-ano"] if os.name == "nt" else ["netstat", "-anp"], capture_output=True,
                             text=True, timeout=30, errors="replace").stdout
    except (OSError, subprocess.SubprocessError):
        return None
    rows = []
    for line in out.splitlines():
        f = line.split()
        if f and f[0].upper() in ("TCP", "UDP", "TCP6", "UDP6") and \
                (f[-1] == str(pid) or f[-1].startswith("%d/" % pid)):
            rows.append(" ".join(f))
    return rows


def load_get_whisper():
    import importlib.util
    spec = importlib.util.spec_from_file_location("get_whisper", os.path.join(REPO, "tools", "get_whisper.py"))
    gw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gw)
    return gw


def as4():
    py = paths.assistant_python()
    try:
        as4_run(py)
    finally:
        as4b(py)                                      # AM2-r3 review LOW: its line also without the models


def as4_run(py):
    if os.name == "nt" and not py or not paths.models_installed():
        return report("AS4 real models, network blocked", None, "models not installed (python tools/get_whisper.py)")
    qs = [("el", "Πού κλείνω την καταπακτή;", "πακτ"), ("en", "How do I close the hatch?", "hatch")]
    audio = [(lang, t, must, spoken(lang, t)) for lang, t, must in qs]
    if any(s is None for *_x, s in audio):
        return report("AS4 real models, network blocked", None, "no offline voice (python tools/get_piper.py)")
    clk = Clock()
    a = client.Assistant(View(), settings=Sets(), clock=clk, stt_kind="faster", mic_backend="sim")
    a.worker_cmd = lambda: blocked_cmd(py)
    n0 = len(MARKERS)
    got, snaps = [], []                               # AM2-r3 review LOW: netstat snapshots DURING the run
    import threading
    halt = threading.Event()

    def poll(pid):                                    # every ~1 s while the questions run (native code too)
        while not halt.is_set():
            snaps.append(("during", pid_endpoints(pid)))
            halt.wait(1.0)
    th = None
    try:
        a.start()
        ready = wait(a, lambda: a.state == client.READY, 120)
        if ready and a.proc is not None:
            snaps.append(("ready", pid_endpoints(a.proc.pid)))
            th = threading.Thread(target=poll, args=(a.proc.pid,), daemon=True)
            th.start()
        for lang, t, must, s in audio:
            if not ready:
                break
            a.press("button")
            a.inject(s)
            clk.t += len(s) / mic.RATE
            a.release()
            n = len(a.view.heard)
            wait(a, lambda: len(a.view.heard) > n or a.state == client.READY, 60)
            h = a.view.heard[-1] if len(a.view.heard) > n else ("", {})
            import unicodedata
            plain = "".join(c for c in unicodedata.normalize("NFD", h[0].lower()) if unicodedata.category(c) != "Mn")
            got.append((lang, h[0], h[1].get("ms"), must in plain))
        halt.set()
        if th is not None:
            th.join(60)
        if ready and a.proc is not None and a.proc.poll() is None:
            snaps.append(("after the answers", pid_endpoints(a.proc.pid)))
    finally:
        halt.set()
        a.stop()
    eps = sorted({r for _w, rows in snaps for r in (rows or [])})
    eps_ok = any(w == "ready" for w, _r in snaps) and any(w == "after the answers" for w, _r in snaps) and         all(rows is not None for _w, rows in snaps) and not eps
    tried = attempts(MARKERS[n0:])
    nc_ok, nc = blocker_selftest(py)                  # the negative control in the ASSISTANT Python too
    report("AS4 real models, network blocked", ready and len(got) == 2 and all(g[3] for g in got) and
           len(MARKERS) > n0 and not tried and nc_ok and eps_ok,
           "device %s, %s; worker network attempts %s (a caught attempt counts too); negative control %s; the "
           "worker PID's TCP / UDP endpoints (netstat, OS level - native code too) in %d snapshots (at ready, "
           "every ~1 s while asking, after the answers): %s"
           % (a.device, got, tried or "none", nc, len(snaps), eps or "none"))


def as4b(py):
    """AM2-r2 review LOW-1: the OS-level block (Windows Firewall outbound rule for the assistant Python, added by
    tools/get_whisper.py when an administrator runs it) - absent = N/A with the one command, never PASS. AM2-r3:
    read with Get-NetFirewallRule (Enabled / Outbound / Block for that program), printed with or without models."""
    if os.name != "nt":
        return report("AS4b OS-level network block (firewall rule)", None,
                      "not Windows - the OS firewall is set by hand (docs/assistant_design.md 2; W93)")
    gw = load_get_whisper()
    exe = py or os.path.join(gw.PYDEST, "python.exe")  # not installed yet: the place get_whisper.py puts it
    have = gw.firewall_rule_present(exe)
    if have:
        report("AS4b OS-level network block (firewall rule)", True,
               "an enabled outbound block rule %r for %s" % (gw.FW_RULE, exe))
    else:
        report("AS4b OS-level network block (firewall rule)", None,
               "%s - the owner runs once in an administrator terminal: %s"
               % ("can't read the firewall (PowerShell NetSecurity)" if have is None else
                  "no enabled outbound block rule for this program", gw.firewall_cmd(exe)))


# --- AS5 / AS6 -------------------------------------------------------------------------------------------------
def as5():
    clk = Clock()
    v = View()
    a = stub(view=v, clock=clk, py=sys.executable)
    out = {}
    try:
        a.start()
        out["ready"] = wait(a, lambda: a.state == client.READY)
        out["joystick refused"] = a.press("joystick") is False and a.state == client.READY
        out["shore refused"] = a.press("shore") is False
        a.press("button")
        clk.t += 0.2
        a.release()
        out["tap ignored"] = a.state == client.READY and a.take_note() == ("asst.n_short", {})
        q = ask_once(a, clk, 1.0)
        out["ask"] = q == "πού κλείνω την καταπακτή"
        a.press("key")
        out["repeat"] = a.press("key") is True and a.state == client.LISTENING
        a.inject(tone(1.0))
        clk.t += client.MAX_S + 0.1
        a.tick()
        out["15 s auto"] = a.state == client.THINKING
        wait(a, lambda: a.state == client.READY)
    finally:
        a.stop()
    # the joystick: no shore / onboard joystick module reaches the assistant; app.py presses only button / key
    src = {}
    for m in ("shorejoy", "yesno", "uavfly", "joymap", "practicalmenu"):
        p = os.path.join(HERE, m + ".py")
        if os.path.isfile(p):
            with open(p, encoding="utf-8") as f:
                src[m] = f.read()
    out["no joystick path"] = not any("asst" in s or "assistant" in s for s in src.values())
    import re
    calls = []
    for m in ("app", "driver", "hud"):
        with open(os.path.join(HERE, m + ".py"), encoding="utf-8") as f:
            calls += re.findall(r"asst\.press\(([^)]*)\)", f.read())
    out["only button / key"] = bool(calls) and all(c.strip() in ('"button"', '"key"') for c in calls)
    out["F10 not a PTT key (Windows menu key, LOW-7)"] = "F10" not in client.KEYS and \
        client.Assistant(View(), settings=Sets(assistant_key="F10")).key() == client.DEFAULT_KEY
    report("AS5 push-to-talk", all(out.values()), "%s, app presses %s" % (out, calls))


def as6(rig):
    clk = Clock()
    v = View()
    a = stub(view=v, clock=clk, py=sys.executable)
    out = {}
    try:
        a.start()
        wait(a, lambda: a.state == client.READY)
        a.press("button")
        s0 = v.stops
        a.horn_pressed()
        out["horn aborts"] = a.state == client.READY and v.stops > s0
        v.horn = True
        out["no listen while held"] = a.press("button") is False and a.take_note() == ("asst.n_horn", {})
        s0 = v.stops
        a.tick()
        out["voice stopped while held"] = v.stops > s0
        v.horn = False
        a.press("button")
        v.horn = True
        a.tick()
        out["held mid-question aborts"] = a.state == client.READY
        v.horn = False
    finally:
        a.stop()
    # the real station: hailer.Hailer.press() calls the assistant's horn_pressed() first (even when refused)
    ap = rig.ap
    calls = []
    orig = ap.asst.horn_pressed
    ap.asst.horn_pressed = lambda: calls.append("horn")
    try:
        ap.hail.press()
    finally:
        ap.asst.horn_pressed = orig
    out["Hailer.press -> horn_pressed"] = calls == ["horn"]
    view = ap.asst.view
    # AM2-r2 review MED-b: the horn state is a plain bool PUSHED into the view (press / release / ov_tick)
    ap.hail.pressing = True
    out["horn not pulled (no push yet)"] = view.horn_held() is False
    ap.hail.push_horn()
    out["AsstView.horn_held (pushed)"] = view.horn_held() is True and view.horn is True
    ap.hail.pressing = False
    ap.hail.mine = True
    ap.root.after = lambda *a, **k: None              # one ov_tick only (no second 200 ms loop)
    try:
        ap.ov_tick()                                  # App.ov_tick pushes it too
    finally:
        del ap.root.after
    out["ov_tick pushes the horn"] = view.horn is True
    ap.hail.mine = False
    ap.hail.push_horn()
    out["horn released"] = view.horn_held() is False
    out["AsstView no sender"] = not any(hasattr(view, n) for n in ("app", "send", "cmd", "link", "_hail", "hail"))
    out["AsstView keeps no station object"] = not any(v is ap.hail or v is ap or v is ap.link
                                                      for v in vars(view).values())
    # AM2 review LOW-1 + AM2-r2 MED-b + AM2-r3 LOW: neither the view NOR the Assistant (ap.asst) holds the App - a
    # gc.get_referents walk from both never meets the App, a bound App method, the loudhailer object or the link
    found, seen, stops = asst_gc_walk(ap, [view, ap.asst])
    out["AsstView + ap.asst hold no App (gc walk)"] = not found and len(seen) > 1
    # AM2 review LOW-3: a ready message and a sound signal stop the assistant too (as TALK does)
    for name, call in (("play_message", lambda: ap.hail.play_message(0)), ("signal", lambda: ap.hail.signal("short5"))):
        calls.clear()
        ap.asst.horn_pressed = lambda: calls.append("horn")
        try:
            call()
        except Exception:
            pass
        finally:
            ap.asst.horn_pressed = orig
        out["Hailer.%s -> horn_pressed" % name] = calls[:1] == ["horn"]
    # AM2-r3 review LOW: every place that changes the horn state pushes it into the view (static check of hailer.py)
    with open(os.path.join(HERE, "hailer.py"), encoding="utf-8") as f:
        hl = f.read().splitlines()
    def func_of(i):
        for j in range(i, -1, -1):
            m = re.match(r"\s*def (\w+)", hl[j])
            if m:
                return m.group(1)
        return ""
    unpushed = [i + 1 for i, s in enumerate(hl) if re.search(r"self\.(pressing|mine)\s*=[^=]", s) and
                func_of(i) != "__init__" and not any("push_horn()" in x for x in hl[i:i + 20])]
    out["every pressing / mine change pushes the horn"] = not unpushed
    report("AS6 loudhailer wins", all(out.values()),
           "%s; objects walked %d, App found %s; walk stops: %s; horn changes without push_horn at lines %s"
           % (out, len(seen), found, stops, unpushed or "none"))


def asst_gc_walk(ap, roots, limit=200000):
    """AM2-r3 review LOW: gc.get_referents walk from roots. It goes INTO types (their dicts, methods, closures) and
    into the modules of groundstation/assistant/; it STOPS (counted, reported) at the namespace of every other
    module - a function's __globals__ there is shared code reachable from everything (app.py's globals reach the
    whole station by design), not a reference the assistant keeps. Returns (found, seen, stops)."""
    import app as A
    import gc
    stop_ns = {}
    for m in list(sys.modules.values()):
        f = os.path.abspath(getattr(m, "__file__", "") or "")
        d = getattr(m, "__dict__", None)
        if d is not None and not f.startswith(ASST + os.sep):
            stop_ns[id(d)] = getattr(m, "__name__", "?")
    found, seen, todo = [], set(), list(roots)
    stops = collections.Counter()
    while todo and len(seen) < limit:
        o = todo.pop()
        if id(o) in seen:
            continue
        seen.add(id(o))
        if isinstance(o, A.App) or getattr(o, "__self__", None) is ap or o is ap.link or o is ap.hail or \
                o is ap.root:
            found.append(type(o).__name__)
            continue
        if id(o) in stop_ns:
            stops["module namespace"] += 1
            continue
        if isinstance(o, types.ModuleType):
            f = os.path.abspath(getattr(o, "__file__", "") or "")
            if not f.startswith(ASST + os.sep):
                stops["module"] += 1
                continue
        todo += gc.get_referents(o)
    return found, seen, dict(stops)


# --- AS7 -------------------------------------------------------------------------------------------------------
def as7():
    out = {}
    a = client.Assistant(View(), settings=Sets(assistant_mic=2, in_dev=5, hail_in=7))
    out["own mic setting"] = a.mic_device() == 2
    a = client.Assistant(View(), settings=Sets(in_dev=5))
    out["loudhailer device never used"] = a.mic_device() is None
    out["mic.py no audiodev"] = "audiodev" not in imports_of(os.path.join(ASST, "mic.py"))
    sent = []
    a = stub(py=sys.executable, assistant_mic=3)
    a._send = lambda d: sent.append(d) or True
    a.proc = None
    a.worker_cmd = lambda: [sys.executable, "-c", "import time; time.sleep(5)"]
    a.start()
    out["init carries assistant_mic"] = bool(sent) and sent[0].get("mic") == 3
    a.stop()
    # one real question, then: no sound file anywhere in the log folder / the assistant folder
    clk = Clock()
    b = stub(clock=clk, py=sys.executable)
    try:
        b.start()
        wait(b, lambda: b.state == client.READY)
        ask_once(b, clk)
    finally:
        b.stop()
    exts = (".wav", ".pcm", ".raw", ".ogg", ".mp3", ".flac")
    files = [os.path.join(r, f) for d in (client.log_dir(), ASST) for r, _d, fs in os.walk(d) for f in fs]
    out["no sound on disk"] = not any(f.lower().endswith(exts) for f in files)
    report("AS7 own microphone, nothing on disk", all(out.values()), str(out))


# --- AS8 -------------------------------------------------------------------------------------------------------
def as8(rig):
    out = {}
    clk = Clock()
    v = View()
    a = client.Assistant(v, settings=Sets(), clock=clk, stt_kind="stub", mic_backend="sim")
    a.worker_cmd = lambda: [sys.executable, "-c", "import sys; sys.exit(1)"]
    a.start()
    wait(a, lambda: a.state == client.DOWN, 10)
    out["crash -> DOWN + notice"] = a.state == client.DOWN and a.out_of_service()
    starts = 1
    for _ in range(3):
        clk.t += client.RESTART_DELAY_S + 0.1
        a.tick()
        starts += 1
        wait(a, lambda: a.state in (client.DOWN, client.FAILED), 10)
    out["3 restarts then FAILED"] = a.state == client.FAILED and len(a.restarts) == 3
    clk.t += client.RESTART_DELAY_S + 1
    a.tick()
    out["no 4th restart"] = a.state == client.FAILED and a.proc is None
    # restarts older than 10 minutes do not count
    b = client.Assistant(View(), settings=Sets(), clock=clk, stt_kind="stub", mic_backend="sim")
    b.restarts = [clk.t - client.RESTART_WINDOW_S - 1] * 3
    b._crash("test")
    out["10-min window"] = b.state == client.DOWN
    # a hang: no answer in 30 s
    c = stub(clock=clk, py=sys.executable)
    c.start()
    wait(c, lambda: c.state == client.READY)
    c.press("button")
    clk.t += 1
    c._send({"op": "ping"})                      # (the stub would answer: make it hang instead)
    c.state, c.ask_t = client.THINKING, clk.t
    clk.t += client.ANSWER_TIMEOUT_S + 1
    c.tick()
    out["hang -> restarted"] = c.state == client.DOWN
    c.stop()
    # AM2 review LOW-5: a worker that answers init but never a ping = restarted, and it counts in the 3 / 10 min
    deaf = "import sys, json\nfor l in sys.stdin:\n    m = json.loads(l)\n    if m.get('op') == 'init':\n" \
           "        print(json.dumps({'op': 'ready', 'device': 'deaf'}), flush=True)\n"
    e = client.Assistant(View(), settings=Sets(), clock=clk, stt_kind="stub", mic_backend="sim")
    e.worker_cmd = lambda: [sys.executable, "-c", deaf]
    e.start()
    wait(e, lambda: e.state == client.READY, 10)
    r0 = e.state == client.READY
    clk.t += client.PING_S + 0.1
    e.tick()
    pinged = e.ping_t is not None
    clk.t += client.PONG_TIMEOUT_S - 1
    time.sleep(0.3)
    e.tick()
    still = e.state == client.READY
    clk.t += 2
    e.tick()
    out["no pong -> restarted"] = r0 and pinged and still and e.state == client.DOWN
    clk.t += client.RESTART_DELAY_S + 0.1
    e.tick()
    out["the restart counts"] = len(e.restarts) == 1
    e.stop()
    # a worker that answers its pings stays up (the real worker, stub recogniser)
    f = stub(clock=clk, py=sys.executable)
    f.start()
    wait(f, lambda: f.state == client.READY)
    for _ in range(3):
        clk.t += client.PING_S + 0.1
        f.tick()
        wait(f, lambda: f.ping_t is None, 5)
    out["pong keeps it up"] = f.state == client.READY and not f.restarts
    f.stop()
    # LOW-6: the stderr file is closed when Popen fails (Windows: it can be deleted) and rotated above 1 MB
    errf = os.path.join(client.log_dir(), "worker_err.txt")
    with open(errf, "w", encoding="utf-8") as fh:
        fh.write("x" * (client.ERR_MAX_BYTES + 10))
    g = client.Assistant(View(), settings=Sets(), clock=clk, stt_kind="stub", mic_backend="sim",
                         popen=lambda *a_, **k_: (_ for _ in ()).throw(OSError("no such program")))
    g.start()
    rot = os.path.isfile(os.path.join(client.log_dir(), "worker_err.1.txt")) and os.path.getsize(errf) < 100
    try:
        os.remove(errf)
        closed = True
    except OSError:
        closed = False
    out["Popen fails -> DOWN, stderr file closed"] = g.state == client.DOWN and closed
    out["stderr rotated above 1 MB"] = rot
    # the notice on both screens, never an alarm
    ap = rig.ap
    real = ap.asst
    real._set(client.DOWN, "test")
    try:
        ap.render(force=True)
        b2 = ts.op(ap, "asst_out")
        out["screen 2 notice"] = b2 is not None and b2["s"] == i18n.T("asst.out")
        import driver
        dn = driver.DriverScreen.notices(types.SimpleNamespace(app=ap), ap.t, ap.gw_status or {})
        out["screen 1 notice"] = any(n[0] == "asst_out" for n in dn)
        out["never an alarm"] = not any("asst" in str(k) for k in getattr(ap.lamps, "local", {}) or {})
    finally:
        real._set(client.IDLE)
    report("AS8 crash / hang / restarts", all(out.values()), str(out))


# --- AS9 / AS10 / AS12 ------------------------------------------------------------------------------------------
def as9():
    out = {}
    d = tempfile.mkdtemp(prefix="trygons_gwpc_")
    g = os.path.join(d, "gateway.json")
    with open(g, "w") as f:
        f.write("{}")
    os.environ["TRYGONS_GATEWAY_CONFIG"] = g
    try:
        a = stub(py=sys.executable)
        a.start()
        out["client refuses"] = a.state == client.GATEWAY_PC and a.proc is None
        import subprocess
        r = subprocess.run([sys.executable, os.path.join(ASST, "worker.py")], input=b"", capture_output=True,
                           timeout=30, env=dict(os.environ))
        out["worker refuses"] = r.returncode == 3 and b"refused" in r.stdout
    finally:
        os.environ.pop("TRYGONS_GATEWAY_CONFIG", None)
    for name in ("small", "large-v3", "Systran/faster-whisper-small", "rel/folder", ""):
        try:
            stt.check_model_dir(name)
            out["name %r refused" % name] = False
        except stt.Refused:
            out["name %r refused" % name] = True
    try:
        stt.check_model_dir(d)                      # a real folder without model.bin
        out["folder without model.bin"] = False
    except stt.Refused:
        out["folder without model.bin"] = True
    # the worker answers a model name with an error (never a download)
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(ASST, "worker.py")], timeout=60, capture_output=True,
                       input=(json.dumps({"op": "init", "stt": "faster", "model": "small"}) + "\n").encode())
    lines = [json.loads(x) for x in r.stdout.decode().splitlines() if x.strip()]
    out["worker refuses a name"] = bool(lines) and lines[0]["op"] == "error" and "folder" in lines[0]["why"]
    saved = {k: os.environ.pop(k, None) for k in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE")}
    stt.offline_env()
    out["offline env"] = os.environ.get("HF_HUB_OFFLINE") == "1" and os.environ.get("TRANSFORMERS_OFFLINE") == "1"
    for k, v in saved.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v
    with open(os.path.join(ASST, "client.py"), encoding="utf-8") as f:
        out["client env offline"] = 'HF_HUB_OFFLINE="1"' in f.read()
    report("AS9 gateway PC + offline models", all(out.values()), str(out))


def as10():
    el, en = hints.prompt("el"), hints.prompt("en")
    tel = hints.terms("el")
    low = [t.lower() for t in tel]
    out = {"el budget": 0 < len(el) <= hints.MAX_CHARS + 1, "en budget": 0 < len(en) <= hints.MAX_CHARS + 1,
           "HDG LOCK": "HDG LOCK" in el, "μάγγεμα": "μάγγεμα" in low, "STANDBY": "STANDBY" in el,
           "καταπακτή": "καταπακτή" in el, "alarm word": "ρηχά νερά" in low, "Tools page (lang file)": "Ρυθμίσεις" in tel,
           "en magazine": "UAV magazine" in hints.terms("en"), "no placeholders": "{" not in el + en,
           "no Greek capitals (measured: they hurt)": not any(t.isupper() and any("Ͱ" <= c <= "Ͽ" for c in t)
                                                               for t in tel)}
    report("AS10 term hints", all(out.values()), "%s; el prompt %d chars: %s..." % (out, len(el), el[:120]))


def as12():
    d = client.log_dir()
    with open(os.path.join(REPO, ".gitignore"), encoding="utf-8") as f:
        gi = f.read()
    ok = d.startswith(tempfile.gettempdir()) and "groundstation/assistant/log/" in gi
    report("AS12 log folder", ok, "test log folder %s; .gitignore has groundstation/assistant/log/: %s" %
           (d, "groundstation/assistant/log/" in gi))


# --- AS11 screens -----------------------------------------------------------------------------------------------
def as11(rig):
    ap = rig.ap
    i18n.set_lang("el", save=False)
    out = {}
    a = ap.asst
    calls = []
    orig = a.press, a.release, a.state
    a.press = lambda src: calls.append(("press", src)) or True
    a.release = lambda: calls.append("release")
    try:
        a._set(client.READY)
        rig.size(1366, 768, 1.0)
        ap.render(force=True)
        b2 = ts.op(ap, "bar1")
        ask = ts.op(ap, "ask")
        badge = ts.op(ap, "ai_badge_ask")
        out["s2 label"] = ask is not None and ask["s"] == "ΕΡΩΤΗΣΗ"
        out["s2 in bar 1"] = ask is not None and b2 is not None and overlay.inside(ask["r"], b2["r"], 1.5)
        out["s2 AI badge"] = badge is not None and overlay.inside(badge["r"], ask["r"], 1.0)
        out["s2 not red"] = ask is not None and ask["fill"].lower() != ts.theme.WARN.lower()
        r = ask["r"]
        ap.ov.click((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
        ap.ov.release()
        out["s2 press / release"] = calls[:2] == [("press", "button"), "release"]
        cv0 = ap.camview                              # the camera view (a UAV is flown there): still on bar 1
        ap.camview = "FRONT-1"
        try:
            out["s2 camera view bar 1"] = any(it[0] == "btn" and it[1] == "ask" for it in ap.bar_items(1))
        finally:
            ap.camview = cv0
        out["s2 not on bar 2 (beside TALK: no room)"] = not any(it[0] == "btn" and it[1] == "ask"
                                                                for it in ap.bar_items(2))
        a._set(client.STARTING)
        ap.render(force=True)
        out["s2 greyed"] = ts.op(ap, "ask").get("enabled") is False
        a._set(client.READY)
        fits = []
        for W, H, dpi in ((1366, 768, 1.25), (1920, 1080, 1.0), (1366, 658, 1.5)):
            ts.fill_everything(rig)
            rig.size(W, H, dpi)
            fits.append(((W, H, dpi), ts.layout_problems(ap)[:2]))
        out["s2 fits"] = all(not p for _s, p in fits)
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:                                     # docs/screens/assistant (AM2): listening + a recognised question
            import canvaspic
            for lg, q in (("el", "πού κλείνω την καταπακτή;"), ("en", "how do I close the hatch?")):
                i18n.set_lang(lg, save=False)
                rig.size(1366, 768, 1.0)
                ap.asst_on_heard(q, {})
                a._set(client.LISTENING)
                ap.render(force=True)
                rig.pump(0.3)
                canvaspic.save(ap.cv, os.path.join(shots, "screen2_ask_%s.png" % lg))
                a._set(client.READY)
            i18n.set_lang("el", save=False)
        ts.no_relay_boat(rig)
        # screen 1
        calls.clear()
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.pump(1.5)
        dv.draw()
        rig.pump(0.3)
        hits = [h for h, act in dv.hits if act == "ask"]
        if hits:
            h = hits[0]
            z = dv.zoom
            ev = types.SimpleNamespace(x=(h[0] + h[2]) / 2 * z, y=(h[1] + h[3]) / 2 * z)
            dv.on_press(ev)
            dv.on_release(ev)
        out["s1 press / release"] = calls[:2] == [("press", "button"), "release"]
        bar = dv.hud.R("bottom")
        out["s1 in the bottom bar"] = bool(hits) and overlay.inside(hits[0], bar, 2.0)
        al, no = dv.hud.R("alarms"), dv.hud.R("notices")
        out["s1 not over alarms / notices"] = bool(hits) and not overlay.overlaps(hits[0], al) and \
            not overlay.overlaps(hits[0], no)
        out["s1 AI badge"] = any(o.get("tag") == "ai_badge" and overlay.inside(o["r"], hits[0], 2.0)
                                 for o in dv.hud.ops if o.get("r")) if hits else False
        # AM2 review MED-1: the real screen-1 snapshot with «ΟΜΙΛΙΑ» shown - «ΕΡΩΤΗΣΗ» far from it (other buttons
        # between, >= ASK_PTT_MIN_GAP bar heights) at every size
        import hud
        gaps = []
        for W, H in ((1600, 900), (1366, 768), (1024, 600)):
            s1 = dv.snapshot()
            s1["hail"] = {"label": "ΟΜΙΛΙΑ", "talking": False, "enabled": True, "text": "", "caution": False}
            _o, hh = dv.hud.build(W, H, s1)
            bh = dv.hud.R("bottom")
            g = hud.ask_ptt_gap(hh)
            gaps.append(((W, H), g and (round(g[0]), g[1]), bool(g) and g[0] >= hud.ASK_PTT_MIN_GAP * (bh[3] - bh[1])
                         and g[1] >= 1))
        out["s1 far from TALK"] = all(x[2] for x in gaps)
        dv.draw()
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots and dv.last_frame is not None:
            ap.asst_on_heard("πού κλείνω την καταπακτή;", {})
            dv.draw()
            rig.pump(0.3)
            dv.last_frame.save(os.path.join(shots, "screen1_ask_el.png"))
        ap.close_driver()
        # the Tools page «Βοηθός»
        ap.open_tab("tab.assistant")
        rig.pump(0.5)
        pg = getattr(ap, "asst_page", None)
        out["tools page"] = pg is not None
        if pg is not None:
            pg.set_key("F8")
            out["key saved"] = settings.get("assistant_key") == "F8" and a.key() == "F8"
            pg.set_key("F9")
            pg.refresh()
            # (+ "  ·  device" when a worker ran before: AS3b used the real ap.asst)
            out["page text"] = "F9" in pg.note.cget("text") and                 pg.state.cget("text").startswith(i18n.T("asst.state.ready"))
            # AM3: the last question's place in the manual (the worker's "found" on the "heard" answer)
            from assistant import page as asst_page, worker as asst_worker
            wk = asst_worker.Worker(out=open(os.devnull, "w", encoding="utf-8"))
            last0 = a.last
            for lg, q in (("el", "πού κλείνω την καταπακτή;"), ("en", "how do I close the hatch?")):
                i18n.set_lang(lg, save=False)
                a.last = {"q": q, "lang": lg, "audio_s": 2.1, "ms": 860, "found": wk.found(q, lg)}
                pg.refresh()
                out["found line " + lg] = asst_page.found_line(a.last["found"]) in pg.last.cget("text") and \
                    not a.last["found"]["cant_find"]
                if shots:                             # wide enough for the last question + its place (AM3)
                    geo, wl = ap.tools.geometry(), pg.note.cget("wraplength")
                    ws = [w for w in (pg.note, pg.state, pg.models, pg.last) if "wraplength" in w.keys()]
                    for w in ws:
                        w.configure(wraplength=900)
                    ap.tools.geometry("1500x1600")
                    rig.pump(0.4)
                    ts.shot(ap.tools, os.path.join(shots, "tools_assistant_%s.png" % lg))
                    for w in ws:
                        w.configure(wraplength=wl)
                    ap.tools.geometry(geo)
            a.last = {"q": "πες μου ένα ανέκδοτο", "lang": "el", "audio_s": 1.5, "ms": 700,
                      "found": wk.found("πες μου ένα ανέκδοτο", "el")}
            i18n.set_lang("el", save=False)
            pg.refresh()
            out["can't find line"] = i18n.T("asst.not_found") in pg.last.cget("text")
            a.last = last0
            pg.refresh()
            if shots:
                ap.tools.attributes("-topmost", False)
    finally:
        a.press, a.release = orig[0], orig[1]
        a._set(orig[2])
    report("AS11 screens + Tools page", all(out.values()), "%s; fits %s" % (out, fits if "fits" in dir() else "-"))


def as5_keys(rig):
    """The key on the real station: F9 presses / releases; ignored in a text field and for another key."""
    import tkinter as tk
    ap = rig.ap
    a = ap.asst
    calls = []
    orig = a.press, a.release
    a.press = lambda src: calls.append(("press", src)) or True
    a.release = lambda: calls.append("release")
    out = {}
    try:
        e = tk.Entry(rig.root)
        out["text field"] = (ap.asst_key_down(types.SimpleNamespace(keysym="F9", widget=e)) or True) and not calls
        ap.asst_key_down(types.SimpleNamespace(keysym="F8", widget=ap.cv))
        out["other key"] = not calls
        ap.asst_key_down(types.SimpleNamespace(keysym="F9", widget=ap.cv))
        ap.asst_key_up(types.SimpleNamespace(keysym="F9", widget=ap.cv))
        ap.asst_key_down(types.SimpleNamespace(keysym="F9", widget=ap.cv))     # X11 auto-repeat: release + press
        rig.pump(0.1)
        out["auto-repeat kept"] = "release" not in calls
        ap.asst_key_up(types.SimpleNamespace(keysym="F9", widget=ap.cv))
        rig.pump(0.15)
        out["key press / release"] = calls[0] == ("press", "key") and calls[-1] == "release"
        out["bound on the window"] = bool(rig.root.bind_all("<KeyPress-F9>"))
        out["F9 free"] = not any(k in ("<F9>", "<KeyPress-F9>") for k in rig.root.bind())
        e.destroy()
    finally:
        a.press, a.release = orig
    report("AS5b the key", all(out.values()), str(out))


def as13():
    """AM2 review MED-4: tools/get_whisper.py checks the Python zip + get-pip.py by sha256 (a get-pip COMMIT) and
    installs ONLY from the hash-locked requirements files (--require-hashes, wheels only, every line pinned)."""
    import re
    gw = load_get_whisper()
    out = {"zip sha256": re.fullmatch(r"[0-9a-f]{64}", gw.PY_SHA) is not None,
           "get-pip sha256": re.fullmatch(r"[0-9a-f]{64}", gw.GET_PIP_SHA) is not None,
           "get-pip at a commit": gw.GET_PIP_COMMIT in gw.GET_PIP and "bootstrap.pypa.io" not in gw.GET_PIP}
    with open(os.path.join(REPO, "tools", "get_whisper.py"), encoding="utf-8") as f:
        src = f.read()
    out["zip checked"] = "get(PY_URL.format(v=PY_VERSION), PY_SHA)" in src
    out["get-pip checked"] = "get(GET_PIP, GET_PIP_SHA)" in src
    cmd = gw.install_cmd("py", True)
    out["install = requirements + --require-hashes"] = "--require-hashes" in cmd and ":all:" in cmd and \
        cmd.count("-r") == 2 and not any("==" in c for c in cmd)
    out["no other pip install"] = src.count('"install"') == 1
    pinned = []
    for path in (gw.REQ, gw.REQ_CUDA):
        with open(path, encoding="utf-8") as f:
            body = f.read()
        reqs = re.split(r"\n(?=[A-Za-z])", "\n".join(x for x in body.splitlines() if not x.startswith("#")).strip())
        for r in reqs:
            name = r.split("==")[0]
            pinned.append((name, bool(re.match(r"^[A-Za-z0-9_.\-]+==[^ \\]+ \\", r)) and
                           len(re.findall(r"--hash=sha256:[0-9a-f]{64}", r)) >= 1))
    names = {n.lower() for n, _ok in pinned}
    out["every line pinned + hashed"] = bool(pinned) and all(ok for _n, ok in pinned)
    out["top-level packages in the lock"] = all(p.split("==")[0].lower() in names
                                                for p in gw.PACKAGES + gw.CUDA_PACKAGES)
    # AM2-r2 review LOW-2: the install lock marker (sha256 of the requirements + the bootstrap)
    import shutil
    d = tempfile.mkdtemp(prefix="trygons_asst_lock_")
    keep = gw.GET_PIP_SHA, gw.REQ, gw.REQ_CUDA
    try:
        out["no Python = missing"] = gw.install_state(True, d)[0] == "missing"
        open(os.path.join(d, "python.exe"), "wb").close()
        with open(os.path.join(d, "get-pip.py"), "w") as f:
            f.write("# not the pinned get-pip.py\n")
        out["other get-pip.py = bootstrap"] = gw.install_state(True, d)[0] == "bootstrap"
        gw.GET_PIP_SHA = gw.sha256(os.path.join(d, "get-pip.py"))
        for k in ("REQ", "REQ_CUDA"):
            q = os.path.join(d, os.path.basename(getattr(gw, k)))
            shutil.copy(getattr(gw, k), q)
            setattr(gw, k, q)
        out["no marker = missing"] = gw.install_state(True, d)[0] == "missing"
        mk = os.path.join(d, os.path.basename(gw.LOCK))

        def put(m):
            with open(mk, "w", encoding="utf-8") as f:
                json.dump(m, f)
        put(gw.lock_marker(True))
        out["marker matches = ok"] = gw.install_state(True, d) == ("ok", "")
        out["cuda changed = stale"] = gw.install_state(False, d)[0] == "stale"
        with open(gw.REQ, "a", encoding="utf-8") as f:
            f.write("# changed\n")
        out["requirements changed = stale"] = gw.install_state(True, d)[0] == "stale"
        m = gw.lock_marker(True)
        m["py_zip_sha256"] = "0" * 64
        put(m)
        out["another Python zip = bootstrap"] = gw.install_state(True, d)[0] == "bootstrap"
    finally:
        gw.GET_PIP_SHA, gw.REQ, gw.REQ_CUDA = keep
        shutil.rmtree(d, ignore_errors=True)
    out["pip only when not ok + marker after the check"] = 'if st != "ok":' in src and \
        src.index("json.dump(lock_marker(cuda)") > src.index("import faster_whisper, ctranslate2")
    # AM2-r2 review LOW-3: --make-requirements refuses without TRYGONS_ASSISTANT_PYTHON
    import subprocess
    env = dict(os.environ)
    env.pop("TRYGONS_ASSISTANT_PYTHON", None)
    r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "get_whisper.py"), "--make-requirements"],
                       capture_output=True, text=True, env=env, timeout=60, errors="replace")
    out["--make-requirements refused without TRYGONS_ASSISTANT_PYTHON"] = r.returncode == 2 and "REFUSED" in r.stdout
    # AM2-r2 review LOW-1: the OS-level rule's command
    fc = gw.firewall_cmd("C:/x/python.exe")
    out["firewall rule = outbound block for the assistant Python"] = "dir=out" in fc and "action=block" in fc and \
        "program=" in fc and "firewall(exe)" in src
    # AM2-r3 review LOW: added with an ARGUMENT LIST (no shell), read back with Get-NetFirewallRule /
    # Get-NetFirewallApplicationFilter (untranslated values), Enabled + Outbound + Block for exactly that program
    fa = gw.firewall_args("C:/x/python.exe")
    out["firewall add = argument list, no shell"] = isinstance(fa, list) and fa[:5] == [
        "netsh", "advfirewall", "firewall", "add", "rule"] and "dir=out" in fa and "action=block" in fa and \
        "shell=True" not in src and "subprocess.run(firewall_args(exe)" in src
    out["firewall read = NetSecurity cmdlets"] = "Get-NetFirewallRule" in gw.FW_PS and \
        "Get-NetFirewallApplicationFilter" in gw.FW_PS and "netsh advfirewall firewall show" not in src
    exe = os.path.abspath("C:/x/python.exe")
    row = {"Enabled": "True", "Direction": "Outbound", "Action": "Block", "Program": exe}
    out["rule judged: enabled outbound block for the program"] = gw.rule_blocks([row], exe) and not any(
        gw.rule_blocks([dict(row, **ch)], exe) for ch in ({"Enabled": "False"}, {"Direction": "Inbound"},
                                                         {"Action": "Allow"}, {"Program": "C:/y/python.exe"},
                                                         {"Program": ""})) and not gw.rule_blocks([], exe)
    # AM2-r3 review LOW: stale / missing = installed AFRESH (no leftovers of a dropped package) + the marker only
    # when pip freeze --all equals the lock
    pins = gw.locked_pins(True)
    frz = "".join("%s==%s\n" % (k, v) for k, v in pins.items()) + "pip==26.3\n"
    out["freeze = the lock -> no difference"] = gw.freeze_diff(frz, True) == []
    out["a package the lock dropped is seen"] = any(d.startswith("extra oldpkg") for d in
                                                    gw.freeze_diff(frz + "oldpkg==1.0\n", True))
    gp_src = src[src.index("def get_python"):src.index("def install_cmd")]
    out["not ok = installed afresh (rmtree first)"] = 'if st == "ok":' in gp_src and \
        gp_src.index("shutil.rmtree(PYDEST)") < gp_src.index("install_cmd(exe, cuda)")
    out["marker only after freeze == lock"] = gp_src.index("freeze_diff(") < gp_src.index("json.dump(lock_marker")
    report("AS13 hash-locked downloads", all(out.values()), "%s; %d packages" % (out, len(pinned)))


# --- AS14 - AS18: FIND (step AM3) ------------------------------------------------------------------------------
# AM3-r (review MED-2 / MED-3): measured with the STRICT labels (a specific phrase in the passage TEXT, a phrase in
# >= 6 passages or a heading-only match = no hit) on the TUNING part (the thresholds were re-tuned there only) and
# the frozen HELD-OUT part (tests/assistant/heldout.json, measured once): TEST_REPORT.md "Talking manual - FIND".
# The floors sit just under those honest numbers; a later change may only keep or raise them (lower = FAIL).
# el tuning 56/96 top-1, 75/96 top-3; el held-out 24/38, 29/38; en tuning 15/33, 24/33; en held-out 6/9, 8/9.
FIND_MIN = {"tuning": {"el": {"top1": 0.55, "top3": 0.75}, "en": {"top1": 0.42, "top3": 0.70}},
            "heldout": {"el": {"top1": 0.60, "top3": 0.73}, "en": {"top1": 0.55, "top3": 0.75}}}
FIND_MAX_FALSE_CANT = {"el": 3, "en": 1}         # the whole list
FIND_MIN_CANT_OK = {"el": 8, "en": 2}           # of the "not in the manual" questions (BM25 alone; AM4 adds more)
I2_GROUPS = {"uav", "launch_hatch", "alarms", "modes", "lost_link", "recording", "loudhailer", "roles", "dp",
             "engine", "power_limit"}


def _tools_index():
    import importlib.util
    spec = importlib.util.spec_from_file_location("assistant_index_tool",
                                                  os.path.join(REPO, "tools", "assistant_index.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def as14():
    """The committed index = a fresh build of the manuals + tour (deterministic) and not stale; a changed source,
    a wrong version or a missing file = stale; a CRLF checkout of a source = the same hash."""
    from assistant import index
    out = {}
    with tempfile.TemporaryDirectory() as d:
        for lang in ("el", "en"):
            out["%s not stale" % lang] = index.stale(lang) == []
            index.save(index.build(lang), d)
            with open(index.index_path(lang, d), encoding="utf-8") as f:
                fresh = f.read()
            with open(index.index_path(lang), encoding="utf-8") as f:
                out["%s committed = fresh build" % lang] = f.read() == fresh
            idx = json.loads(fresh)
            idx["sources"]["MANUAL_EL.txt" if lang == "el" else "MANUAL.txt"] = "0" * 64
            index.save(idx, d)
            out["%s changed manual = stale" % lang] = any("changed" in w for w in index.stale(lang, d))
            idx["version"] = index.INDEX_VERSION + 1
            index.save(idx, d)
            out["%s version = stale" % lang] = any("version" in w for w in index.stale(lang, d))
        out["missing = stale"] = bool(index.stale("el", os.path.join(d, "nothing")))
        lf, crlf = os.path.join(d, "lf.txt"), os.path.join(d, "crlf.txt")
        with open(lf, "wb") as f:
            f.write("α\nβ\n".encode("utf-8"))
        with open(crlf, "wb") as f:
            f.write("α\r\nβ\r\n".encode("utf-8"))
        out["CRLF = LF hash"] = index.sha_text(lf) == index.sha_text(crlf)
    srcs = index.sources("el")
    out["sources: manual + tour + parser"] = all(any(s.endswith(x) for s in srcs) for x in (
        "MANUAL_EL.txt", "tour.json", "index.py"))
    out["lang files no source (not indexed, LOW-1)"] = not any(s.endswith(("el.txt", "en.txt")) for s in srcs)
    with open(os.path.join(REPO, "tools", "check.sh"), encoding="utf-8") as f:
        sh = f.read()
    # AM3-r MED-1: the gate CHECKS the committed index (exit 1 = FAIL), it never rebuilds it quietly
    out["check.sh fails on a stale index"] = ("tools/assistant_index.py check" in sh and
                                              "\"$PY\" tools/assistant_index.py build" not in sh)
    # mutation: the sources copied, a manual changed, NO rebuild -> `assistant_index.py check` exits 1 (STALE);
    # the unchanged copy -> 0
    tool = os.path.join(REPO, "tools", "assistant_index.py")
    with tempfile.TemporaryDirectory() as d:
        for rel in set(index.sources("el") + index.sources("en")):
            os.makedirs(os.path.dirname(os.path.join(d, rel)) or d, exist_ok=True)
            shutil.copyfile(os.path.join(REPO, rel), os.path.join(d, rel))
        env = dict(os.environ, TRYGONS_ASSISTANT_SRC=d, PYTHONIOENCODING="utf-8")
        same = subprocess.run([sys.executable, tool, "check"], env=env, capture_output=True, text=True,
                              encoding="utf-8", timeout=120)
        with open(os.path.join(d, "MANUAL_EL.txt"), "a", encoding="utf-8") as f:
            f.write("\nμια νέα γραμμή χωρίς νέο ευρετήριο\n")
        changed = subprocess.run([sys.executable, tool, "check"], env=env, capture_output=True, text=True,
                                 encoding="utf-8", timeout=120)
        out["check: unchanged copy = 0"] = same.returncode == 0
        out["check: manual changed, no rebuild = exit 1 STALE"] = changed.returncode == 1 and             "STALE" in changed.stdout and "MANUAL_EL.txt changed" in changed.stdout
    report("AS14 index build + stale check", all(out.values()), out)


def as15():
    """Normalisation: case, accents, diaeresis, final sigma, Greek look-alikes in a Latin name, stop words in both
    languages, endings, the spoken synonyms (drone = UAV); no network / process module in index.py."""
    from assistant import index
    T = index.tokens
    out = {
        "case + accents + final sigma": T("ΚΑΤΑΠΑΚΤΗΣ", "el") == T("καταπακτής", "el") == T("Καταπακτης", "el"),
        "diaeresis": T("προϊόν", "el") == T("προιον", "el"),
        "noun forms": T("καταπακτή", "el") == T("καταπακτής", "el") and T("ΧΕΙΡΙΣΤΗΡΙΟΥ", "el") == T("χειριστήριο", "el"),
        "verb forms": T("ξεκινάω", "el") == T("ξεκινά", "el") and T("κλείνω", "el") == T("κλείσιμο", "el"),
        "HDG typed with a Greek Η": T("ΗDG", "el") == T("HDG", "el") == ["hdg"],
        "stop words": T("πώς το κάνω;", "el") == [] and T("how do I do it", "en") == [],
        "drone = UAV": T("drone", "el") == T("UAV", "el") == T("drones", "en"),
        "joystick = χειριστήριο": T("joystick", "el") == T("χειριστήριο", "el"),
        "numbers kept": "120" in T("120 m", "el") and "5" in T("5 s", "en"),
        "pairs": index.with_pairs(["a", "b", "c"]) == ["a", "b", "c", "a_b", "b_c"],
    }
    imps = imports_of(os.path.join(ASST, "index.py"))
    out["index.py: no network / process import"] = not (imps & (BANNED | PROCESS))
    report("AS15 normalisation", all(out.values()), out)


def as16():
    """The manual -> passages: every chapter of the contents list, nothing from the contents itself, passage sizes,
    line ranges, the alarm / mode lists split per term, the tour stops as "where is it" passages."""
    from assistant import index
    out = {}
    for lang in ("el", "en"):
        with open(os.path.join(REPO, index.MANUALS[lang]), encoding="utf-8") as f:
            text = f.read()
        lines = text.splitlines()
        first = next(i for i, l in enumerate(lines) if index.CHAPTER_RE.match(l) and index.RULE_RE.match(lines[i - 1]))
        toc = {l.split(".")[0].strip() for l in lines[:first] if l.strip()[:1].isdigit()}
        ps = index.load(lang)["passages"]
        man = [p for p in ps if p["kind"] == "manual"]
        out[lang + " every chapter"] = toc - index.SKIP_CHAPTERS.get(lang, set()) <= {p["ch"] for p in man}
        out[lang + " term table left out"] = not {p["ch"] for p in man} & index.SKIP_CHAPTERS.get(lang, set())
        out[lang + " nothing from the contents"] = all(p["lines"][0] > first for p in man)
        out[lang + " sizes"] = all(1 <= len(p["text"].split()) <= index.MAX_WORDS * 1.5 for p in ps)
        out[lang + " line ranges"] = all(p["lines"][0] <= p["lines"][1] <= len(lines) for p in man)
        # AM3-r LOW-4: every (split) passage's own range - its first word on its first line, its last on its last
        out[lang + " range per split part"] = all(
            p["text"].split()[0] in lines[p["lines"][0] - 1] and p["text"].split()[-1] in lines[p["lines"][1] - 1]
            for p in man)
        # AM3-r LOW-3: the assistant's own chapter is not indexed
        out[lang + " assistant chapter left out"] = ("17" if lang == "el" else "22") in index.SKIP_CHAPTERS[lang] \
            and ("17" if lang == "el" else "22") not in {p["ch"] for p in man}
        terms = {p["term"] for p in man}
        want = ({"ΑΠΩΛΕΙΑ ΠΥΞΙΔΑΣ", "ΧΑΜΗΛΗ ΜΠΑΤΑΡΙΑ", "ΧΤΥΠΗΜΑΤΑ (SLAMMING)", "STANDBY", "ΕΠΙΣΤΡΟΦΗ AUTO"}
                if lang == "el" else {"GATEWAY LOST", "LOW BATTERY", "SLAMMING", "STANDBY", "HDG LOCK"})
        out[lang + " terms " + "/".join(sorted(want))] = want <= terms
        tour = json.load(open(os.path.join(REPO, index.TOUR), encoding="utf-8"))
        n = sum(1 for c in tour["chapters"] for s in c["stops"] if s.get("text_" + lang))
        out[lang + " tour stops"] = sum(1 for p in ps if p["kind"] == "tour") == n
        out[lang + " ids unique"] = len({p["id"] for p in ps}) == len(ps)
    report("AS16 manual -> passages", all(out.values()), out)


def as17():
    """The question list (tests/assistant/questions_*.json, owner spec I2) and the measured retrieval: top-1 /
    top-3 on the TUNING and the HELD-OUT part at least the AM3-r numbers, most not-in-the-manual questions = can't
    find, few false can't-finds, every question has a SPECIFIC expected phrase / real tour stop (a manual edit that
    drops one fails here); the strict hit rule itself (MED-3): a heading-only match and a phrase in >= 6 passages
    are no hit."""
    ai = _tools_index()
    out, info = {}, {}
    ps = [{"id": "el:%d" % i, "kind": "manual", "heading": "ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ", "term": "",
           "text": "κείμενο %d με το κουμπί STANDBY" % i} for i in range(6)]
    ps.append({"id": "el:9", "kind": "manual", "heading": "Χ", "term": "", "text": "πατήστε ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ τώρα"})
    out["strict: heading-only = no hit"] = ai.matches(ps[0], ["ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ"]) == "" and         ai.matches(ps[6], ["κλεισιμο καταπακτης"]) == "manual"
    out["strict: a phrase in >= 6 passages is loose"] = ai.specific(ps, ["κουμπί STANDBY"]) == [] and         ai.specific(ps, ["ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ"]) == ["ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ"]
    out["tour stop hits counted apart"] = ai.matches({"id": "tour:el:uav.stop", "kind": "tour", "text": "x"},
                                                     [], ("uav.stop",)) == "tour"
    for lang in ("el", "en"):
        qs = ai.load_questions(lang)
        rows, t = ai.evaluate(lang, qs=qs)
        n = t["in_manual"]
        info[lang] = "top-1 %d/%d top-3 %d/%d false can't %d, can't find %d/%d" % (
            t["top1"], n, t["top3"], n, t["false_cant_find"], t["cant_find_ok"], t["not_in_manual"])
        for part in ("tuning", "heldout"):
            u = t[part]
            info[lang + " " + part] = "top-1 %d/%d top-3 %d/%d" % (u["top1"], u["in_manual"], u["top3"], u["in_manual"])
            out["%s %s top-1" % (lang, part)] = u["top1"] >= FIND_MIN[part][lang]["top1"] * u["in_manual"]
            out["%s %s top-3" % (lang, part)] = u["top3"] >= FIND_MIN[part][lang]["top3"] * u["in_manual"]
        held = ai.load_heldout(lang)
        ids = [q["id"] for q in qs["questions"]]
        out[lang + " held-out ids real + >= 20 %"] = held <= set(ids) and len(held) >= 0.2 * len(ids)
        out[lang + " no broken expectation"] = not t["broken"]
        out[lang + " false can't find"] = t["false_cant_find"] <= FIND_MAX_FALSE_CANT[lang]
        out[lang + " not in the manual = can't find"] = t["cant_find_ok"] >= FIND_MIN_CANT_OK[lang] and \
            t["not_in_manual"] >= 5
        out[lang + " every question has a must term"] = all(q.get("must") or not q.get("expect")
                                                         for q in qs["questions"])
        if lang == "el":
            out["el 120-150 questions"] = 120 <= t["n"] <= 150
            out["el every I2 group"] = I2_GROUPS <= {q["group"] for q in qs["questions"]}
            out["el several phrasings"] = len({q["must"] for q in qs["questions"] if q["expect"]}) < n
    report("AS17 retrieval on the question list", all(out.values()), "%s; %s" % (info, out))


def as18():
    """The worker's "find" (in-process, no model): a hit with its chapter + full passage text, the can't-find
    reason for a question not in the manual, a missing index = can't find (never a crash), < 50 ms per search."""
    import io
    from assistant import worker
    buf = io.StringIO()
    w = worker.Worker(out=buf)
    w.handle({"op": "find", "q": "πού κλείνω την καταπακτή;", "lang": "el"})
    w.handle({"op": "find", "q": "what will the weather be tomorrow", "lang": "en"})
    w.handle({"op": "find", "q": "πες μου ένα ανέκδοτο", "lang": "el"})
    got = [json.loads(x) for x in buf.getvalue().splitlines()]
    t0 = time.monotonic()
    for _ in range(20):
        w.found("τι σημαίνει χαμηλή μπαταρία", "el")
    ms = (time.monotonic() - t0) * 1000 / 20
    w2 = worker.Worker(out=io.StringIO())
    from assistant import index
    real = index.INDEX_DIR
    try:
        index.INDEX_DIR = os.path.join(tempfile.gettempdir(), "no_such_index_%d" % os.getpid())
        miss = w2.found("καταπακτή", "el")
    finally:
        index.INDEX_DIR = real
    # AM3-r MED-1 / LOW-2: a STALE Greek index (the Greek manual changed, no rebuild) = every Greek question
    # "can't find" + the reason + the station notice (ready "index_err"), and NEVER the English index instead;
    # English (current) still finds
    real_src = index.SRC_ROOT
    with tempfile.TemporaryDirectory() as d:
        for rel in set(index.sources("el") + index.sources("en")):
            os.makedirs(os.path.dirname(os.path.join(d, rel)) or d, exist_ok=True)
            shutil.copyfile(os.path.join(REPO, rel), os.path.join(d, rel))
        with open(os.path.join(d, "MANUAL_EL.txt"), "a", encoding="utf-8") as f:
            f.write("\nμια αλλαγή χωρίς νέο ευρετήριο\n")
        try:
            index.SRC_ROOT = d
            b3 = io.StringIO()
            w3 = worker.Worker(out=b3)
            w3.handle({"op": "init", "stt": "stub", "lang": "el"})
            st_el = w3.found("πού κλείνω την καταπακτή;", "el")
            st_en = w3.found("how do I close the hatch?", "en")
        finally:
            index.SRC_ROOT = real_src
    ready = [json.loads(x) for x in b3.getvalue().splitlines() if '"ready"' in x]
    stale_out = {
        "stale el = can't find": st_el["cant_find"] and "stale" in st_el["why"] and not st_el["hits"],
        "stale el never answered from en": not st_el["hits"],
        "en (current) still finds": not st_en["cant_find"] and bool(st_en["hits"]),
        "ready carries the notice": bool(ready) and "stale" in ready[0].get("index_err", ""),
    }
    # the client turns it into the station notice asst.n_index (amber)
    from assistant import client as aclient
    c = aclient.Assistant.__new__(aclient.Assistant)
    c.state, c.note, c.view, c.device, c.index_err = aclient.STARTING, "", None, "", ""
    c._set = lambda st, *a: setattr(c, "state", st)
    c._on({"op": "ready", "device": "stub", "index_err": "manual index el stale: MANUAL_EL.txt changed"})
    stale_out["client notice asst.n_index"] = c.note == ("asst.n_index", {}) and "asst.n_index" in         aclient.PROBLEM_NOTES and bool(c.index_err)
    h = got[0]["hits"][0] if got and got[0].get("hits") else {}
    out = {"found": got and got[0]["op"] == "found" and not got[0]["cant_find"] and "ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ" in h.get("text", ""),
           "chapter + place": bool(h.get("ch") and h.get("chapter") is not None and "lines" in h),
           "weather = can't find": len(got) > 1 and got[1]["cant_find"] and bool(got[1]["why"]),
           "joke = can't find": len(got) > 2 and got[2]["cant_find"],
           "no index = can't find": miss["cant_find"] and "index" in miss["why"],
           "fast": ms < 50}
    out.update(stale_out)
    report("AS18 worker find", all(out.values()), "%s; %.1f ms per search" % (out, ms))


def main():
    exe = os.path.join(HERE, "..", "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    for fn in (as1, as2, as3, as5, as7, as9, as10, as12, as13, as14, as15, as16, as17, as18) + \
            (() if os.environ.get("AS_SKIP4") else (as4,)):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__, False, "crashed: %r" % e)
    rig = ts.Rig()
    try:
        for fn in (as6, as8, as3_rig, as11, as5_keys):
            try:
                fn(rig)
            except Exception as e:
                import traceback
                traceback.print_exc()
                report(fn.__name__, False, "crashed: %r" % e)
    finally:
        rig.close()
    bad = results.count(False)
    na = results.count(None)
    print("ASSISTANT TESTS: %d passed, %d failed, %d N/A" % (len(results) - bad - na, bad, na))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
