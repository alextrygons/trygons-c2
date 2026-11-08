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
      at most 3 restarts in 10 minutes then off; a hang (no answer in 60 s, AM4) = restarted
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
  AM3-r3 (review of 25906e3): AS3b's +1 phase allowance only for a kind the station sends itself (baseline 0 =
      none; mutation (c): one thread-sent send of a new kind is caught) (MED-1); AS6 asserts the gc walk's own
      "todo ran empty" (LOW-1)
  AM4 (the answer): AS19 the grounded prompt + the POST-CHECK (an invented name, an altered number "5 kn" ->
      "15 kn", an altered unit "5 kn" -> "5 m", a number word with another unit, a bare number, a foreign chapter =
      can't find said + the passage shown) + can't find (search or model); AS20 the GPU memory guard (a GGUF header
      read, fake NVML numbers -> GPU / GPU+CPU / CPU, the margin, a reload on the CPU when the GPU gets short before
      an answer, a model name / URL refused); AS21 retrieval-only (no model, a cut generation, 3 slow answers in a
      row, a model error); AS22 the answer through the real worker (stub model; heard -> THINKING -> answer, the
      release -> answer time, ask_text); AS23 tools/get_llm.py pins + no llama-server / llama-cli / port in any
      assistant module (llama_cpp only in answer.py); AS24 the REAL language model with the network blocked (N/A -
      never PASS - when it is not downloaded)
  AM4-r (review of 3e1b1d3): AS19 the reviewer's bypass probes ("50 feet", "50 nautical miles", "3 milliseconds",
      "3 hrs", "20 per cent", "-3 s", "eighty seconds", «εβδομήντα / τριακόσια δευτερόλεπτα» - MED-1), the
      same-passage-SENTENCE rule for numbers + swapped numbers, a negation either way ("STANDBY never wins", "The PC
      can override STANDBY"), "Reset button" / «κουμπί X» / "PC button" names (MED-2), the wrong language (LOW-1), a
      failed check = can't find SAID + the passage SHOWN with its chapter (W105); AS20 the GPU tried again after a
      CPU reload, rate-limited, never against the user's CPU setting, nothing persisted (LOW-3) + the model file's
      sha256 (paths.llm_verify: unpinned refused unless allowed, a pinned name with other bytes refused) (LOW-2);
      AS21 a generation stopped at max_tokens = cut (finish_reason "length"), the first token timed, a late first
      token = cut (MED-3) + answer mode "extract" (the model only picks numbered passage sentences); AS22 an answer
      whose question was interrupted by «ΟΜΙΛΙΑ» carries speak = False (LOW-3); AS23 paths.LLM_PINNED =
      get_llm.MODELS + the worker verifies before it loads

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
import threading
import time
import types
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import test_screen2 as ts  # noqa: E402  (temp settings, sound off, TRYGONS_TEST, the Rig on the simulator)
import i18n  # noqa: E402
import overlay  # noqa: E402
import proto  # noqa: E402  (AS3b mutation (c): NEW_FTYPE is no frame type of the protocol)
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
ALLOW_PHASE = 1                                       # AM2-r4: +1 per method + kind (phase); AM3-r3: only with a baseline > 0
NEW_FTYPE = 0xFE                                      # AM3-r3 mutation (c): a frame type no protocol defines


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
    sender reached from it counts as the assistant's, though no assistant frame is on the stack when it runs.
    LIMIT (AM2-r4 LOW-1): the taint covers ONLY tkinter after / after_idle - a thread, a socket callback or a
    queue another component drains is not followed (the count comparison (2) is the net for those);
    (2) every sender call is counted per method + payload kind and compared with the MOST of BASE_RUNS windows
    without the assistant + ALLOW_PHASE (AM2-r4 MED-1: a periodic send may land once more in a window by its
    phase; so (2) alone can't see ONE or TWO extra periodic-kind sends - (1) does for scheduled ones; three are
    caught by (2), mutation (b)). AM3-r3 (review of 25906e3 MED-1): the phase allowance exists ONLY for a method +
    kind the station sends on its own (baseline > 0); a kind with a baseline of 0 has no phase, so ONE such send -
    also from a thread the taint can't see - is caught by (2) (mutation (c)). mutate(ap) = a mutation run (quiet)."""
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
        last, settled = None, False
        for _i in range(20):
            every.clear()
            rig.pump(1.0)
            now = set(every)
            if now == last:
                settled = True
                break
            last = now
        # AM2-r4 (review MED-1): never a silent fall-through - a start-up that never settles is reported (FAIL)
        out["start-up sends settled within 20 s"] = settled
        a.stt_kind, a.stub_text, a.mic_backend = "stub", "πού κλείνω την καταπακτή", "sim"
        a.worker_cmd = lambda: blocked_cmd(sys.executable)
        ap.asst_heard = None
        t0 = time.monotonic()
        every.clear()
        a.answer = None
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
        # AM4: the answer follows the question on screen 2's message line (the question stays on screen 1, 8 s)
        rig.pump(20, until=lambda: a.answer is not None)
        out["heard on the screens"] = bool(ap.asst_heard) and ap.asst_heard[0] == "πού κλείνω την καταπακτή"
        out["the answer on screen 2's message line (AM4)"] = a.answer is not None and \
            bool(a.answer.get("a")) and a.answer["a"] in (ap.msg_text or "")
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
        # AM2-r4 (review of 6ec6e0b MED-1): + ALLOW_PHASE per method + kind - a real-time pump may let a periodic
        # send land once more by its phase; ONE send scheduled by assistant code is the taint's job (mutation (a)).
        # AM3-r3 (review of 25906e3 MED-1): the phase allowance ONLY for a kind the station sends on its own
        # (baseline > 0) - a method + kind with a baseline of 0 has no phase: ONE such send = FAIL (mutation (c))
        extra = {k: (c, without.get(k, 0)) for k, c in with_asst.items()
                 if c > without.get(k, 0) + (ALLOW_PHASE if without.get(k, 0) > 0 else 0)}
        out["no sender call above the station's own count (+ %d only for a kind it sends itself; a kind with a "
            "baseline of 0: none)" % ALLOW_PHASE] = not extra
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
    # (b) four more (AM4; three before) from a path the taint can't see (scheduled by plain code) = caught by the count comparison
    common = max(without, key=without.get) if without else None
    m = {}
    if common is not None:
        cls_, name_, x_, k_ = samples[common]

        def sneak_after(app_):
            def asst_sneak():                          # an asst_* name = assistant glue at SCHEDULING time
                app_.root.after(10, lambda: getattr(cls_, name_)(*x_, **k_))   # runs later: no assistant frame
            asst_sneak()

        def sneak_count(app_):
            # AM4: 4 (was 3) - the window is longer now (it waits for the answer), so a periodic kind may land 2
            # fewer times than in the busiest baseline window by its phase; 4 extra still shows above + ALLOW_PHASE
            for i in range(4):
                app_.root.after(10 + 20 * i, lambda: getattr(cls_, name_)(*x_, **k_))
        m_ok, _e, m_sent = as3_rig(rig, mutate=sneak_after, quiet=True)
        m["(a) scheduled by the assistant"] = (not m_ok and any(common in s and "scheduled" in s for s in m_sent),
                                               m_sent)
        m_ok, m_extra, _s = as3_rig(rig, mutate=sneak_count, quiet=True)
        m["(b) 4 extra of a station kind"] = (not m_ok and common in m_extra, m_extra)

        # AM3-r3 (review of 25906e3 MED-1): (c) ONE send of a method + payload kind the station NEVER sends on its
        # own (baseline 0), made from a THREAD (no assistant frame, no tkinter taint): the +1 phase allowance never
        # applies to a kind with a baseline of 0, so this single send shows above the count. The kind: the same
        # method as (a) / (b) with the message "t" = "asst_mutation" or the frame type NEW_FTYPE (unknown to
        # proto.py and app/proto.h - the simulator answers UNKNOWN, nothing changes)
        x2, k2 = list(x_), dict(k_)
        if len(x2) > 1 and isinstance(x2[1], dict):
            x2[1] = dict(x2[1], t="asst_mutation")
        elif len(x2) > 1 and isinstance(x2[1], int):
            x2[1] = NEW_FTYPE
        elif isinstance(k2.get("obj"), dict):
            k2["obj"] = dict(k2["obj"], t="asst_mutation")
        elif "ftype" in k2:
            k2["ftype"] = NEW_FTYPE
        else:
            x2 = None                                   # no payload kind to change: (c) can't be made = FAIL below
        new_key = call_key(cls_, name_, tuple(x2), k2) if x2 is not None else None

        def sneak_thread(app_):
            def go():
                try:
                    getattr(cls_, name_)(*x2, **k2)
                except Exception:
                    pass                                # the trap counted it before the real send (that may fail)
            th = threading.Thread(target=go, daemon=True)
            th.start()
            th.join(5)
        if new_key is not None and new_key not in without:
            m_ok, m_extra, _s = as3_rig(rig, mutate=sneak_thread, quiet=True)
            m["(c) 1 thread-sent send of a NEW kind %s (baseline 0)" % new_key] = (
                not m_ok and m_extra.get(new_key) == (1, 0), m_extra)
        else:
            m["(c) 1 thread-sent send of a NEW kind"] = (False, "no new kind could be made from %s (%r)" % (common, new_key))
    out["mutation: a scheduled send via an already-used method + payload is caught"] = \
        bool(m) and all(v[0] for v in m.values())
    out["NEW_FTYPE unknown to proto.py"] = NEW_FTYPE not in {v for n, v in vars(proto).items() if n.startswith("T_")}
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
    a = client.Assistant(View(), settings=Sets(assistant_llm="off"), clock=clk, stt_kind="faster", mic_backend="sim")
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
    found, seen, stops, complete = asst_gc_walk(ap, [view, ap.asst])
    # AM2-r4 (review LOW-2): the walk must END by itself - hitting the limit would hide a reference past it.
    # AM3-r3 (review of 25906e3 LOW-1): the walk itself says whether its todo list ran EMPTY (len(seen) < limit
    # alone could also mean a walk that stopped for another reason)
    out["gc walk complete (its todo ran empty)"] = complete is True and len(seen) < GC_WALK_LIMIT
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
    # AM2-r3 review LOW: every place that changes the horn state pushes it into the view (static check of hailer.py).
    # LIMIT (AM2-r4 LOW-3): a TEXT check - the push_horn() call must be within the next 20 lines of the change in
    # hailer.py; a change elsewhere (another module, an attribute set through a helper) is not seen
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


GC_WALK_LIMIT = 200000


def asst_gc_walk(ap, roots, limit=GC_WALK_LIMIT):
    """AM2-r3 review LOW: gc.get_referents walk from roots. It goes INTO types (their dicts, methods, closures) and
    into the modules of groundstation/assistant/; it STOPS (counted, reported) at the namespace of every other
    module - a function's __globals__ there is shared code reachable from everything (app.py's globals reach the
    whole station by design), not a reference the assistant keeps. Returns (found, seen, stops, complete):
    complete = True only when the todo list ran EMPTY (the walk ended by itself, not at its limit - AM3-r3 LOW-1)."""
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
    return found, seen, dict(stops), not todo


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
                # AM4: the answer (stub model = the passage's first sentence, post-checked) + where the model runs
                wk.load_llm({"kind": "stub", "reply": "@first"})
                a.answer = dict(wk.ans.answer(q, lg, a.last["found"]), answer_ms=2400)
                a.llm = dict(wk.llm_info, name="Llama 3.2 3B Instruct", mode="gpu")
                pg.refresh()
                out["found line " + lg] = asst_page.found_line(a.last["found"]) in pg.last.cget("text") and \
                    not a.last["found"]["cant_find"]
                out["answer line " + lg] = a.answer["src"] == "llm" and a.answer["a"] in pg.ans.cget("text") and \
                    i18n.T("asst.src.llm") in pg.ans.cget("text") and "Llama 3.2 3B" in pg.models.cget("text")
                if shots:                             # wide enough for the last question + its place (AM3)
                    geo, wl = ap.tools.geometry(), pg.note.cget("wraplength")
                    ws = [w for w in (pg.note, pg.state, pg.models, pg.last, pg.ans) if "wraplength" in w.keys()]
                    for w in ws:
                        w.configure(wraplength=900)
                    ap.tools.geometry("1500x1760")
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
            a.last, a.answer, a.llm = last0, None, {}
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
    # AM2-r4 (review LOW-5): our exe path is never env-expanded (a literal %X% folder stays); the Program read back is
    os.environ["TRYGONS_AM2R4_X"] = "y"
    try:
        out["exe path not expanded"] = "%TRYGONS_AM2R4_X%" in gw.winpath("C:/a%TRYGONS_AM2R4_X%b/python.exe") and             "%TRYGONS_AM2R4_X%" in " ".join(gw.firewall_args("C:/a%TRYGONS_AM2R4_X%b/python.exe"))
        out["read-back Program expanded"] = gw.rule_blocks(
            [dict(row, Program=r"C:\%TRYGONS_AM2R4_X%\python.exe")], os.path.abspath("C:/y/python.exe"))
    finally:
        os.environ.pop("TRYGONS_AM2R4_X", None)
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
# of the "not in the manual" questions (BM25 alone; AM4's post-check is the backstop). AM3-r2 MED-2: en back to 3
# (= what MIN_COVER 0.35 refused) with the per-language MIN_COVER en 0.33 tuned on the TUNING part only
FIND_MIN_CANT_OK = {"el": 8, "en": 3}
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
        # AM3-r2 LOW: a hand-edited passage with the right hashes and version = stale too
        for lang in ("el", "en"):
            idx = index.build(lang)
            idx["passages"][0]["text"] += " ΧΕΡΙ"
            index.save(idx, d)
            out["%s hand-edited passages = stale" % lang] = any("passages differ" in w for w in index.stale(lang, d))
            index.save(index.build(lang), d)
            out["%s fresh build = current" % lang] = index.stale(lang, d) == []
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
    # AM3-r2 MED-1: a phrase found in ANY tour text = a "tour" hit; phrases are counted on manual passages only
    tp = {"id": "tour:el:other", "kind": "tour", "heading": "", "term": "", "text": "γράφει ΒΟΗΘΟΣ ΕΚΤΟΣ εδώ"}
    out["tour text phrase = tour hit"] = ai.matches(tp, ["ΒΟΗΘΟΣ ΕΚΤΟΣ"]) == "tour"
    out["tour text never makes a phrase specific"] = ai.specific(ps + [tp], ["ΒΟΗΘΟΣ ΕΚΤΟΣ"]) == []
    out["tour texts never count toward loose"] = ai.phrase_count(ps + [dict(tp, text="ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ")] * 9,
                                                                 "ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ") == 1
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


# --- AM4: the answer --------------------------------------------------------------------------------------------
HATCH = [{"id": "el:16:1", "ch": "16", "chapter": "Το μαγγέμα UAV", "heading": "Καταπακτή", "term": "",
          "text": "Η καταπακτή κλείνει με το κουμπί «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ» μόνο από τον κυβερνήτη ή την κυβερνήτρια. "
                  "Πάνω από 5 kn κάθε σταθμός δείχνει μια προειδοποίηση. Η κάθοδος γίνεται σε 3 s."},
         {"id": "el:12:4", "ch": "12", "chapter": "Χαμένη σύνδεση", "heading": "", "term": "",
          "text": "Κοντά στην ακτή (κάτω από 50 m) ο χρόνος σύνδεσης είναι πάντα 3 s."}]


def as19():
    """Grounded prompt + post-check (names AND every number + unit) + can't find - pure, with the stub model."""
    from assistant import answer as A
    terms = A.lang_terms()
    found = {"cant_find": False, "why": "", "hits": HATCH}
    out = {}
    msgs = A.prompt("πού κλείνω την καταπακτή;", HATCH, "el")
    body = msgs[1]["content"]
    out["prompt: the instruction + the passages + the question only"] = msgs[0]["role"] == "system" and \
        A.CANT["el"] in msgs[0]["content"] and all(h["text"] in body for h in HATCH) and \
        body.rstrip().endswith("πού κλείνω την καταπακτή;") and len(msgs) == 2
    out["prompt in the question's language"] = "Answer ONLY" in A.prompt("q", HATCH, "en")[0]["content"]

    def run(reply, f=found, q="πού κλείνω την καταπακτή;"):
        llm = A.StubLlm(reply)
        r = A.Answerer(llm, terms).answer(q, "el", f)
        return r, llm
    good = "Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ»: η κάθοδος γίνεται σε 3 s."
    r, _ = run(good)
    out["a grounded answer kept + its chapter cited by the code + the passage shown"] = r["src"] == "llm" and \
        r["a"] == good and r["cite"] == "Κεφάλαιο 16 «Το μαγγέμα UAV»" and not r["problems"] and \
        r["passage"]["text"] == HATCH[0]["text"] and r["passage"]["cite"] == r["cite"]
    # a failed check (AM4-r, W105): «Δεν βρίσκω ...» is SAID, the best passage is SHOWN with its chapter - the
    # passage's first sentence is no longer spoken
    for label, bad in (("an invented button name", "Πατήστε «ΑΝΟΙΓΜΑ ΚΑΤΑΠΑΚΤΗΣ» στην οθόνη 1."),
                       ("an invented name in capitals", "Πατήστε HATCH CLOSE στη σελίδα UAV."),
                       ("an altered number 5 kn -> 15 kn", "Πάνω από 15 kn κάθε σταθμός δείχνει προειδοποίηση."),
                       ("an altered unit 5 kn -> 5 m", "Πάνω από 5 m κάθε σταθμός δείχνει προειδοποίηση."),
                       ("a number word with an altered unit", "Η κάθοδος γίνεται σε τρία λεπτά."),
                       ("a bare number not in the passages", "Το κουμπί είναι το 7ο της μπάρας."),
                       ("a chapter not among the passages", "Δείτε το κεφάλαιο 9 για την καταπακτή."),
                       # AM4-r MED-1: the reviewer's bypass probes (an unknown / other unit, the sign, number words)
                       ("50 feet (an added unit)", "Κοντά στην ακτή κάτω από 50 feet ο χρόνος σύνδεσης είναι 3 s."),
                       ("50 nautical miles", "Κοντά στην ακτή κάτω από 50 ναυτικά μίλια ο χρόνος σύνδεσης είναι 3 s."),
                       ("3 milliseconds", "Η κάθοδος γίνεται σε 3 milliseconds."),
                       ("3 hrs", "Η κάθοδος γίνεται σε 3 hrs."),
                       ("20 per cent", "Πάνω από 20 τοις εκατό κάθε σταθμός δείχνει προειδοποίηση."),
                       ("-3 s (the sign)", "Η κάθοδος γίνεται σε -3 s."),
                       ("eighty seconds (a number word)", "Η κάθοδος γίνεται σε ογδόντα δευτερόλεπτα."),
                       ("εβδομήντα δευτερόλεπτα", "Η κάθοδος γίνεται σε εβδομήντα δευτερόλεπτα."),
                       ("τριακόσια δευτερόλεπτα", "Η κάθοδος γίνεται σε τριακόσια δευτερόλεπτα."),
                       ("a number + a word not in the passages", "Η κάθοδος γίνεται σε 3 βήματα."),
                       # AM4-r MED-2: a number tied to another thing than in the manual (a passage SENTENCE)
                       ("5 kn tied to the link time", "Πάνω από 5 kn ο χρόνος σύνδεσης είναι 3 s."),
                       # AM4-r MED-2: a negation the passage sentence does not have
                       ("a negation not in the passage", "Η καταπακτή δεν κλείνει με «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ»."),
                       ("a Capitalised 'κουμπί X' name", "Πατήστε το κουμπί Reset για την καταπακτή."),
                       ("an ungrounded sentence", "Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ». Μετά περιμένετε."),
                       # AM4-r LOW-1: the wrong language
                       ("an English answer to a Greek question", "The hatch closes with «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ» "
                                                                  "only by the captain.")):
        r, _ = run(bad)
        out["post-check: %s -> can't find said + the passage shown" % label] = \
            r["src"] == "passage" and r["a"] == A.CANT["el"] and r["problems"] and r["model_text"] == bad and \
            r["passage"]["text"] == HATCH[0]["text"] and r["cite"] == "Κεφάλαιο 16 «Το μαγγέμα UAV»"
    # the reviewer's English probes (swapped numbers, a negation either way, "X button" names, PC)
    REM = [{"id": "en:8:1", "ch": "8", "chapter": "Remote", "heading": "", "term": "",
            "text": "The remote cap is 20 % forward. Reverse is capped at 5 %. The STANDBY panel button always wins. "
                    "The PC can never override it."}]
    for label, bad in (("swapped numbers", "Reverse is capped at 20 % and forward at 5 %."),
                       ("swapped numbers (the other way)", "Forward is capped at 20 % and reverse at 5 %."),
                       ("'STANDBY never wins'", "STANDBY never wins."),
                       ("'The PC can override STANDBY'", "The PC can override STANDBY."),
                       ("'Reset button'", "Press the Reset button."),
                       ("'PC button' when PC is not a name in the passages", "Press the PC button to stop.")):
        if "PC button" in label:
            hits = [dict(REM[0], text=REM[0]["text"].replace("The PC can never override it.", ""))]
        else:
            hits = REM
        pr = A.postcheck(bad, hits, terms, "en")
        out["post-check (en probe): %s refused" % label] = bool(pr)
    out["post-check (en): the manual's own statement passes"] = not A.postcheck(
        "The STANDBY panel button always wins.", REM, terms, "en") and not A.postcheck(
        "The remote cap is 20 % forward and 5 % in reverse.", REM, terms, "en")
    out["post-check (en): 'PC' in the passages used as a name passes"] = not A.postcheck(
        "The PC can never override the STANDBY button.", REM, terms, "en")
    # AM4-r2 MED-2: the negation farther than 2 tokens from the shared word ("δεν μπορεί ποτέ να το παρακάμψει",
    # "can never override it" in a sentence that is NOT the best match) - the reviewer's probes
    STB = [{"id": "el:9:1", "ch": "9", "chapter": "Λειτουργίες", "heading": "", "term": "",
            "text": "Το κουμπί STANDBY του πίνακα κερδίζει πάντα. Ο υπολογιστής δεν μπορεί ποτέ να το παρακάμψει."}]
    out["post-check (el probe): «Ο υπολογιστής μπορεί να παρακάμψει το STANDBY.» refused"] = any(
        "negates" in p for p in A.postcheck("Ο υπολογιστής μπορεί να παρακάμψει το STANDBY.", STB, terms, "el"))
    out["post-check (en probe): 'The STANDBY button always wins and the PC can override it.' refused"] = any(
        "negates" in p for p in A.postcheck("The STANDBY button always wins and the PC can override it.", REM,
                                            terms, "en"))
    out["post-check (el): the manual's own negated statement passes"] = not A.postcheck(
        "Ο υπολογιστής δεν μπορεί ποτέ να παρακάμψει το STANDBY.", STB, terms, "el")
    out["negated_shared skips stop words / μπορ- / να / το, not content words"] = \
        A.negated_shared(A.norm("ο υπολογιστής δεν μπορεί ποτέ να το παρακάμψει"), ["παρακαμψει"]) and \
        A.negated_shared(A.norm("the PC can never override it"), ["override"]) and \
        not A.negated_shared(A.norm("never press the hatch button while the boat moves"), ["moves"])
    # AM4-r3 MED-2: the "same thing" rule PER CLAUSE - a long sentence joining a TRUE clause (many content words) to a
    # FALSE one escaped (the 40 % was measured against the whole sentence); and a comma insertion after the negation
    # ("never allowed, in any mode or situation, to override it") escaped the 2-word window
    REM3 = [dict(REM[0], text="The remote cap is 20 % forward. Reverse is capped at 5 %. In AUTO the boat follows the "
                              "waypoint route at the cruise speed of each leg. The STANDBY panel button always wins. "
                              "The PC can never override it.")]
    REM4 = [dict(REM3[0], text=REM3[0]["text"].replace("The PC can never override it.",
                                                        "The PC is never allowed, in any mode or situation, to override it."))]
    STB2 = [dict(STB[0], text="Στο AUTO το σκάφος ακολουθεί τη διαδρομή με την ταχύτητα κάθε σκέλους. " + STB[0]["text"])]
    joined = ("In AUTO the boat follows the waypoint route at the cruise speed of each leg and the PC can override the "
              "STANDBY button.")
    out["post-check (en probe): a true clause 'and' a false one = refused per clause"] = any(
        "negates" in p for p in A.postcheck(joined, REM3, terms, "en"))
    out["post-check (en probe): 'never allowed, in any mode or situation, to override it' negates 'can override'"] = \
        any("negates" in p for p in A.postcheck("The PC can override the STANDBY button.", REM4, terms, "en")) and \
        any("negates" in p for p in A.postcheck(joined, REM4, terms, "en"))
    out["post-check (el probe): «... ακολουθεί τη διαδρομή και ο υπολογιστής μπορεί να παρακάμψει το STANDBY» refused"] = \
        any("negates" in p for p in A.postcheck(
            "Στο AUTO το σκάφος ακολουθεί τη διαδρομή και ο υπολογιστής μπορεί να παρακάμψει το STANDBY.", STB2, terms, "el"))
    out["post-check: a true two-clause sentence passes (en + el)"] = not A.postcheck(
        "The remote cap is 20 % forward and the STANDBY panel button always wins.", REM3, terms, "en") and \
        not A.postcheck("Reverse is capped at 5 %, the remote cap is 20 % forward.", REM3, terms, "en") and \
        not A.postcheck("Το κουμπί STANDBY του πίνακα κερδίζει πάντα και ο υπολογιστής δεν μπορεί ποτέ να το παρακάμψει.",
                        STB2, terms, "el") and not A.postcheck("The PC can never override the STANDBY button.", REM4,
                                                               terms, "en")
    out["clauses(): split at and / και / , ; · - not inside 1,500"] = A.clauses(A.norm(
        "In AUTO the boat follows the route and the PC can override the STANDBY button, within 1,500 m.")) == [
        "in auto the boat follows the route", "the pc can override the standby button", "within 1,500 m."]
    out["negated_shared skips a comma insertion, not a plain list"] = A.negated_shared(
        A.norm("the PC is never allowed, in any mode or situation, to override it"), ["override"]) and \
        not A.negated_shared(A.norm("never press the hatch button while the boat moves"), ["moves"])
    # AM4-r2 LOW-b: thousands separators - "1,500 m" / "1.500 m" are 1500 m, never 1.5 m
    THS = [{"id": "en:5:1", "ch": "5", "chapter": "UAV", "heading": "", "term": "",
            "text": "The radio range is 1,500 m from the boat. The hatch closes in 1.5 s."}]
    out["thousands: 1,500 / 1.500 = 1500, 1.5 = 1.5, 0.125 = 0.125"] = [n[0] for n in A.numbers(
        "1,500 m and 1.500 m and 1.5 s and 0.125 s and 12 %")] == [1500.0, 1500.0, 1.5, 0.125, 12.0]
    out["thousands: '1,500 m' vs the manual's '1,500 m' passes, '1.5 m' / '1500 s' refused"] = \
        not A.postcheck("The radio range is 1,500 m.", THS, terms, "en") and \
        not A.postcheck("The radio range is 1500 m from the boat.", THS, terms, "en") and \
        bool(A.postcheck("The radio range is 1.5 m.", THS, terms, "en")) and \
        bool(A.postcheck("The hatch closes in 1,500 s.", THS, terms, "en"))
    r, _ = run("Η κάθοδος γίνεται σε τρία δευτερόλεπτα, πάνω από πέντε κόμβους μια προειδοποίηση.")
    out["post-check: spoken numbers with the right unit pass"] = r["src"] == "llm"
    r, _ = run("Κοντά στην ακτή κάτω από 50 μέτρα ο χρόνος είναι 3 δευτερόλεπτα.")
    out["post-check: a unit said in words = the same unit (50 m, 3 s)"] = r["src"] == "llm"
    out["number words -> digits (el + en, tens, hundreds, hyphenated, compounds; an article stays)"] = \
        A.norm("εβδομήντα δευτερόλεπτα") == "70 δευτερολεπτα" and A.norm("τριακόσια πενήντα m") == "350 m" and \
        A.norm("twenty-five knots") == "25 knots" and A.norm("one hundred and fifty metres") == "150 metres" and \
        A.norm("εικοσιπέντε kn") == "25 kn" and A.norm("δεκατρία UAV") == "13 uav" and \
        A.norm("ένα κουμπί") == "ενα κουμπι" and A.norm("ένα λεπτό") == "1 λεπτο" and \
        A.norm("eighty seconds") == "80 seconds" and A.norm("δύο χιλιάδες m") == "2000 m"
    out["units folded (feet, miles, hrs, ms, per cent, κόμβους, μίλια)"] = [n[:2] for n in A.numbers(
        "50 feet, 50 miles, 3 hrs, 3 milliseconds, 20 per cent, 5 κόμβους, 2 ναυτικά μίλια, -3 s")] == [
        (50.0, "ft"), (50.0, "mi"), (3.0, "h"), (3.0, "ms"), (20.0, "%"), (5.0, "kn"), (2.0, "nm"), (-3.0, "s")]
    r, _ = run("@cant")
    out["the model's can't-find sentence accepted"] = r["src"] == "cant_find" and r["a"] == A.CANT["el"]
    r, llm = run(good, f={"cant_find": True, "why": "score", "hits": []})
    out["search can't find -> can't find, the model never asked"] = r["src"] == "cant_find" and not llm.calls
    out["cleaned: no markdown / label, at most 2 sentences, a capital first"] = A.clean(
        "**Απάντηση:** πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ». Μετά περιμένετε. Και τρίτη πρόταση. Και τέταρτη.") == \
        "Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ». Μετά περιμένετε."
    r = A.Answerer(A.StubLlm(good), terms).answer("how do I close the hatch?", "en",
                                                   {"cant_find": True, "why": "", "hits": []})
    out["English can't find"] = r["a"] == "I can't find that in the manual."
    # the wrong language the other way (a Greek answer to an English question)
    pr = A.postcheck("Το κουμπί STANDBY κερδίζει πάντα.", REM, terms, "en")
    out["post-check (en): a Greek answer = the wrong language"] = any("language" in p for p in pr)
    report("AS19 grounded prompt + post-check (names, numbers + units, sentences, negation, language) + can't find",
           all(out.values()), {k: v for k, v in out.items() if not v} or "all %d checks" % len(out))


def tiny_gguf(path, arch="llama", layers=32, heads=32, kv=8, embd=4096, pad_mb=4):
    """A GGUF v3 header with the numbers gpu.plan reads (+ an array to skip) and pad_mb of zeros."""
    import struct

    def s(x):
        b = x.encode()
        return struct.pack("<Q", len(b)) + b
    kvs = [(arch + ".block_count", 4, layers), (arch + ".attention.head_count", 4, heads),
           (arch + ".attention.head_count_kv", 4, kv), (arch + ".embedding_length", 4, embd)]
    body = s("general.architecture") + struct.pack("<I", 8) + s(arch)
    body += s("tokenizer.ggml.scores") + struct.pack("<IIQ", 9, 6, 3) + struct.pack("<3f", 0, 1, 2)
    body += s("tokenizer.ggml.tokens") + struct.pack("<IIQ", 9, 8, 2) + s("a") + s("bb")
    for k, t, v in kvs:
        body += s(k) + struct.pack("<I", t) + struct.pack("<I", v)
    with open(path, "wb") as f:
        f.write(b"GGUF" + struct.pack("<IQQ", 3, 0, len(kvs) + 3) + body + b"\0" * (pad_mb << 20))


def as20():
    """The GPU memory guard: fake NVML numbers -> GPU / GPU+CPU / CPU; the reload before an answer; CPU fallback."""
    from assistant import answer as A, gpu, worker as W
    out = {}
    d = tempfile.mkdtemp(prefix="trygons_gguf_")
    p = os.path.join(d, "m.gguf")
    tiny_gguf(p, pad_mb=4)
    meta = gpu.gguf_meta(p)
    out["GGUF header read (layers, heads, kv heads)"] = (meta["arch"], meta["layers"], meta["heads"],
                                                        meta["kv_heads"]) == ("llama", 32, 32, 8)
    meta = dict(meta, file_mb=4900.0)                     # as an 8B Q4_K_M
    kv = gpu.kv_mb(meta, 3072)
    out["KV cache size (8B, 3072 tokens = 384 MB)"] = abs(kv - 384) < 1
    big = gpu.plan(meta, (8000, 8192), 3072)
    mid = gpu.plan(meta, (3700, 6144), 3072)
    low = gpu.plan(meta, (900, 6144), 3072)
    none = gpu.plan(meta, None, 3072)
    cpu = gpu.plan(meta, (8000, 8192), 3072, device="cpu")
    out["plenty free -> GPU (all layers)"] = big["mode"] == "gpu" and big["layers"] == -1
    out["some free -> GPU+CPU (a part of the layers)"] = mid["mode"] == "gpu+cpu" and 1 <= mid["layers"] < 32
    out["too little free -> CPU"] = low["mode"] == "cpu" and low["layers"] == 0
    out["no NVIDIA GPU -> CPU"] = none["mode"] == "cpu"
    out["setting CPU -> CPU"] = cpu["mode"] == "cpu"
    out["the margin is kept free"] = gpu.plan(meta, (int(big["need_mb"] + gpu.MARGIN_MB - 1), 9000), 3072)[
        "mode"] != "gpu" and gpu.plan(meta, (int(big["need_mb"] + gpu.MARGIN_MB + 1), 9000), 3072)["mode"] == "gpu"
    old = os.environ.get("TRYGONS_ASSISTANT_FAKE_GPU")
    try:
        os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "3700,6144"
        out["fake NVML numbers read"] = gpu.free_mb() == (3700, 6144)
        os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "none"
        out["fake: no GPU"] = gpu.free_mb() is None
        # the worker: a model on the GPU, then the free memory drops below LOW_MB before an answer = reloaded
        loads = []

        def fake_llama(self, c, device):
            loads.append(device)
            mode = "cpu" if device == "cpu" else "gpu"
            return A.StubLlm("@first", mode=mode), {"mode": mode, "layers": 0 if mode == "cpu" else -1,
                                                    "why": device, "name": "fake"}
        real = W.Worker._llama
        W.Worker._llama = fake_llama
        try:
            sink = []
            wk = W.Worker(out=types.SimpleNamespace(write=sink.append, flush=lambda: None))
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "4000,6144"
            wk.load_llm({"kind": "llama", "model": p})
            f = wk.found("πού κλείνω την καταπακτή;", "el")
            wk.reply("πού κλείνω την καταπακτή;", "el", f)
            on_gpu = wk.ans.llm.mode == "gpu" and loads == ["auto"]
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "%d,6144" % (gpu.LOW_MB - 50)
            wk.reply("πού κλείνω την καταπακτή;", "el", f)
            last = json.loads(sink[-1])
            out["GPU memory short before an answer -> reloaded on the CPU, the answer still comes"] = on_gpu and \
                loads == ["auto", "cpu"] and wk.ans.llm.mode == "cpu" and last["op"] == "answer" and \
                last.get("llm", {}).get("mode") == "cpu" and last["src"] == "llm"
            wk.reply("πού κλείνω την καταπακτή;", "el", f)
            out["on the CPU: no more reloads while the GPU stays short"] = loads == ["auto", "cpu"]
            # AM4-r LOW-3: the GPU memory is free again -> the GPU is tried again, but rate-limited (GPU_RETRY_S),
            # and only when the plan's need + the margin fit; the user's own "cpu" setting is never overridden
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "5000,6144"
            t = wk.gpu_retry_t
            wk.guard(now=t + W.GPU_RETRY_S - 1)
            out["GPU free again: no retry before GPU_RETRY_S"] = loads == ["auto", "cpu"] and wk.ans.llm.mode == "cpu"
            moved = wk.guard(now=t + W.GPU_RETRY_S + 1)
            out["GPU free again after GPU_RETRY_S: reloaded on the GPU + reported"] = loads == ["auto", "cpu", "auto"] \
                and wk.ans.llm.mode == "gpu" and moved and moved.get("mode") == "gpu" and "free again" in moved["why"] \
                and wk.cpu_forced_t is None
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "%d,6144" % (gpu.LOW_MB - 50)
            wk.reply("πού κλείνω την καταπακτή;", "el", f)
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "%d,6144" % (gpu.MARGIN_MB - 100)
            wk.guard(now=wk.gpu_retry_t + W.GPU_RETRY_S + 1)
            out["GPU still too short for need + margin: stays on the CPU"] = loads == ["auto", "cpu", "auto", "cpu"] \
                and wk.ans.llm.mode == "cpu"
            wk2c = W.Worker(out=types.SimpleNamespace(write=sink.append, flush=lambda: None))
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = "5000,6144"
            wk2c.load_llm({"kind": "llama", "model": p, "device": "cpu"})
            wk2c.guard(now=1e9)
            out["the user's CPU setting: never moved to the GPU"] = loads[-1] == "cpu" and wk2c.ans.llm.mode == "cpu" \
                and wk2c.cpu_forced_t is None
            out["nothing persists: a new worker plans afresh"] = W.Worker().cpu_forced_t is None and \
                W.Worker().gpu_retry_t is None
        finally:
            W.Worker._llama = real
        # a model that fails to load = retrieval only, never a crash; the real loader refuses a name / URL
        wk2 = W.Worker(out=types.SimpleNamespace(write=lambda s: None, flush=lambda: None))
        info = wk2.load_llm({"kind": "llama", "model": "qwen2.5-7b"})
        out["a model NAME (not a local .gguf) = not loaded, retrieval only"] = info["mode"] == "retrieval" and \
            wk2.ans.retrieval_only
        # AM4-r LOW-2: an UNPINNED file (our tiny gguf is not in paths.LLM_PINNED) is refused before any load ...
        info = wk2.load_llm({"kind": "llama", "model": p})
        out["an unpinned model file = refused (retrieval only, the reason named)"] = info["mode"] == "retrieval" and \
            "not pinned" in info["why"] and "assistant_llm_unpinned" in info["why"]
        # ... unless the owner's setting allows it (then the real loader runs: no bindings here = still retrieval,
        # but the refusal is no longer the sha check)
        info = wk2.load_llm({"kind": "llama", "model": p, "allow_unpinned": True})
        out["allow_unpinned: the sha check lets it through"] = "not pinned" not in info["why"]
        # a PINNED name with another content = sha256 mismatch = refused
        pinned_name = sorted(paths.LLM_PINNED)[0]
        fake = os.path.join(d, pinned_name)
        shutil.copyfile(p, fake)
        ok, why = paths.llm_verify(fake)
        out["a pinned file name with other bytes = size / sha256 mismatch = refused"] = not ok and (
            "size" in why or "sha256" in why)
        info = wk2.load_llm({"kind": "llama", "model": fake, "allow_unpinned": True})
        out["allow_unpinned never excuses a pinned file that fails its check"] = info["mode"] == "retrieval" and (
            "size" in info["why"] or "sha256" in info["why"])
        ok, why = paths.llm_verify(p, allow_unpinned=True)
        out["llm_verify: unpinned + allowed = ok, said so"] = ok and "unpinned" in why
        refused = []
        for bad in ("qwen2.5-7b", "https://huggingface.co/x/y.gguf", os.path.join(d, "missing.gguf"), "m.gguf"):
            try:
                A.LlamaLlm(bad)
            except ValueError:
                refused.append(bad)
        out["LlamaLlm refuses a name / URL / missing / relative file (nothing downloaded)"] = len(refused) == 4
    finally:
        if old is None:
            os.environ.pop("TRYGONS_ASSISTANT_FAKE_GPU", None)
        else:
            os.environ["TRYGONS_ASSISTANT_FAKE_GPU"] = old
    shutil.rmtree(d, ignore_errors=True)
    report("AS20 GPU memory guard + CPU fallback", all(out.values()), "%s; plans %s / %s / %s" % (
        out, big, mid, low))


def as21():
    """Retrieval-only mode: no model, a too-slow model (cut), the CPU too slow 3 times in a row."""
    from assistant import answer as A
    out = {}
    found = {"cant_find": False, "why": "", "hits": HATCH}
    short = A.shortest_sentence(HATCH[0]["text"])
    r = A.Answerer(None).answer("πού κλείνω την καταπακτή;", "el", found)
    out["no model -> the best passage's shortest full sentence"] = r["src"] == "retrieval" and r["a"] == short \
        and r["mode"] == "retrieval" and r["cite"].startswith("Κεφάλαιο 16")
    out["shortest = a whole sentence (no fragment)"] = A.shortest_sentence(
        "Η καταπακτή κλείνει μόνο από τον κυβερνήτη. Ποτέ gateway, UAV ή σκάφος). Η κάθοδος γίνεται σε 3 s.") == \
        "Η κάθοδος γίνεται σε 3 s."
    clk = Clock()

    class Slow(A.StubLlm):
        def generate(self, messages, max_tokens=A.MAX_TOKENS, max_s=A.MAX_GEN_S, clock=None):
            clk.t += self.delay_s
            cut = self.delay_s >= max_s
            return {"text": self.reply if cut else "Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ».", "tokens": 9, "cut": cut}
    a = A.Answerer(Slow("Πατήστε το κουμπί «ΚΛΕΙΣΙΜΟ", delay_s=A.MAX_GEN_S), clock=clk)
    r = a.answer("πού κλείνω την καταπακτή;", "el", found)
    out["a generation cut at MAX_GEN_S, no whole sentence -> the passage (retrieval form)"] = \
        r["src"] == "retrieval" and r["a"] == short and not a.retrieval_only
    a = A.Answerer(Slow("Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ». Μετά η κάθοδος γίνεται σε", delay_s=A.MAX_GEN_S), clock=clk)
    r = a.answer("πού κλείνω την καταπακτή;", "el", found)
    out["a cut generation keeps its whole sentences (post-checked)"] = r["src"] == "llm" and \
        r["a"] == "Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ»."
    r = A.decide("1. Κεφάλαιο 10 «ΣΥΝΑΓΕΡΜΟΙ» – ΡΗΧΑ ΝΕΡΑ 2.", HATCH, "el")
    out["an echoed passage label / list number is no answer"] = r["src"] == "retrieval"
    a = A.Answerer(Slow(delay_s=A.SLOW_S + 1), clock=clk)
    srcs = [a.answer("πού κλείνω την καταπακτή;", "el", found)["src"] for _ in range(A.SLOW_N)]
    r = a.answer("πού κλείνω την καταπακτή;", "el", found)
    out["slower than SLOW_S %d times -> RETRIEVAL-ONLY from then on" % A.SLOW_N] = \
        srcs == ["llm"] * A.SLOW_N and a.retrieval_only and r["src"] == "retrieval" and r["mode"] == "retrieval"
    a = A.Answerer(Slow(delay_s=1.0), clock=clk)
    for _ in range(5):
        a.answer("πού κλείνω την καταπακτή;", "el", found)
    out["fast answers stay with the model"] = not a.retrieval_only

    class Boom(A.StubLlm):
        def generate(self, *a, **k):
            raise RuntimeError("CUDA error: out of memory")
    r = A.Answerer(Boom()).answer("πού κλείνω την καταπακτή;", "el", found)
    out["a model error -> the passage sentence, never no answer"] = r["src"] == "retrieval" and r["a"] == short
    # AM4-r MED-3: a generation that stopped at max_tokens (finish_reason "length") is CUT: its unfinished last
    # sentence is dropped, the whole ones are kept (post-checked); the time to the first token is reported
    terms = A.lang_terms()
    r = A.Answerer(A.StubLlm("Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ». Η κάθοδος γίνεται σε 3 s αλλά", finish="length"),
                   terms).answer("πού κλείνω την καταπακτή;", "el", found)
    out["stopped at max_tokens = cut: the unfinished sentence dropped"] = r["cut"] and r["src"] == "llm" and \
        r["a"] == "Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ»." and "first_ms" in r
    r = A.Answerer(A.StubLlm("Η κάθοδος γίνεται σε 3 s αλλά μετά", finish="length"), terms).answer(
        "πού κλείνω την καταπακτή;", "el", found)
    out["stopped at max_tokens with no whole sentence = the retrieval form"] = r["cut"] and r["src"] == "retrieval"

    class Stream:
        """A llama.cpp-like stream for LlamaLlm.generate: pieces, then a finish_reason."""
        def __init__(self, pieces, finish, clk):
            self.pieces, self.finish, self.clk = pieces, finish, clk

        def create_chat_completion(self, **kw):
            for i, p in enumerate(self.pieces):
                self.clk.t += 0.5 if i == 0 else 0.01           # the first token after the prompt pass
                yield {"choices": [{"delta": {"content": p}, "finish_reason": None}]}
            yield {"choices": [{"delta": {}, "finish_reason": self.finish}]}
    ll = A.LlamaLlm.__new__(A.LlamaLlm)
    clk2 = Clock()
    ll.llm = Stream(["Πατήστε ", "«ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ». ", "Η κάθοδος ", "γίνεται"], "length", clk2)
    g = ll.generate([], clock=clk2)
    out["LlamaLlm: finish_reason length -> cut, first_ms = the time to the first token"] = g["cut"] and \
        g["first_ms"] == 500 and g["text"].startswith("Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ».")
    ll.llm = Stream(["Πατήστε ", "«ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ»."], "stop", clk2)
    g = ll.generate([], clock=clk2)
    out["LlamaLlm: a normal stop is not cut"] = not g["cut"] and g["tokens"] == 2

    class Late(Stream):
        def create_chat_completion(self, **kw):
            self.clk.t += A.MAX_GEN_S + 1                           # the prompt pass alone took too long
            yield {"choices": [{"delta": {"content": "Πατήστε"}, "finish_reason": None}]}
            yield {"choices": [{"delta": {"content": " x."}, "finish_reason": "stop"}]}
    ll.llm = Late([], "stop", clk2)
    g = ll.generate([], clock=clk2)
    out["LlamaLlm: a FIRST token later than MAX_GEN_S = cut (timed too)"] = g["cut"] and g["first_ms"] > A.MAX_GEN_S * 1000
    # AM4-r: answer mode "extract" - the model only PICKS numbered passage sentences, said word for word
    msgs = A.prompt_pick("πού κλείνω την καταπακτή;", HATCH, "el")
    body = msgs[1]["content"]
    picks = A.pick_sentences(HATCH)
    out["extract: the prompt numbers every passage sentence (3 + 1) + the question"] = len(picks) == 4 and \
        all(("%d. %s" % (i, s)) in body for i, (s, _h) in enumerate(picks, 1)) and "Ερώτηση" in body and \
        "ΜΟΝΟ τους αριθμούς" in msgs[0]["content"]
    r = A.Answerer(A.StubLlm("1, 3"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: '1, 3' -> those two manual sentences word for word, src extract, chapter cited"] = \
        r["src"] == "extract" and r["a"] == picks[0][0] + " " + picks[2][0] and r["picked"] == [1, 3] and \
        r["cite"] == "Κεφάλαιο 16 «Το μαγγέμα UAV»" and r["answer_mode"] == "extract"
    r = A.Answerer(A.StubLlm("4"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: a sentence of the second passage cites that passage"] = r["src"] == "extract" and \
        r["cite"].startswith("Κεφάλαιο 12")
    r = A.Answerer(A.StubLlm("9"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: a number outside the list = nothing guessed (can't find said, the passage shown)"] = \
        r["src"] == "passage" and r["a"] == A.CANT["el"] and r["passage"]["text"] == HATCH[0]["text"]
    r = A.Answerer(A.StubLlm("Η πρόταση 1 λέει ότι η καταπακτή κλείνει."), terms, mode="extract").answer(
        "πού κλείνω την καταπακτή;", "el", found)
    out["extract: the model wrote its own text = not a pick = can't find said"] = r["src"] == "passage" and \
        r["a"] == A.CANT["el"]
    r = A.Answerer(A.StubLlm("@cant"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: the model's can't find accepted"] = r["src"] == "cant_find"
    r = A.Answerer(A.StubLlm("1, 2, 3, 4"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: at most 2 sentences"] = r["src"] == "extract" and r["picked"] == [1, 2]
    # AM4-r2 LOW-e: garbage outputs = nothing guessed (can't find said, the passage shown), never a crash
    for label, garbage in (("letters", "asdf"), ("a number outside the list", "7."), ("a wrong echo", "1. Something else."),
                           ("words around a number", "I think 1 is right"), ("a number word", "one"),
                           ("only punctuation", "..."), ("a fragment of a pick", picks[0][0][:20]),
                           ("a number + text", "1 Πατήστε το κουμπί")):
        r = A.Answerer(A.StubLlm(garbage), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
        out["extract garbage (%s) -> can't find + the passage" % label] = r["src"] == "passage" and \
            r["a"] == A.CANT["el"] and r["passage"]["text"] == HATCH[0]["text"]
    r = A.Answerer(A.StubLlm(""), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: an empty output = the retrieval form"] = r["src"] == "retrieval"
    # AM4-r2 LOW-c: a CUT extract output "1" is the pick (no whole-sentence filter in extract mode)
    r = A.Answerer(A.StubLlm("1", finish="length"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el",
                                                                                 found)
    out["extract: a cut '1' uses the pick, not the retrieval sentence"] = r["cut"] and r["src"] == "extract" and \
        r["picked"] == [1]
    # AM4-r2 LOW-a: two picks from DIFFERENT passages = both chapters cited, both passages shown
    r = A.Answerer(A.StubLlm("1, 4"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: a mixed pair cites both chapters + shows both passages"] = r["src"] == "extract" and \
        r["cite"] == "Κεφάλαιο 16 «Το μαγγέμα UAV» + Κεφάλαιο 12 «Χαμένη σύνδεση»" and \
        r["passage"]["text"] == HATCH[0]["text"] and r["passage2"]["text"] == HATCH[1]["text"]
    r = A.Answerer(A.StubLlm("1, 3"), terms, mode="extract").answer("πού κλείνω την καταπακτή;", "el", found)
    out["extract: two picks of one passage = one chapter, no passage2"] = r["cite"] == "Κεφάλαιο 16 «Το μαγγέμα UAV»" \
        and r.get("passage2") is None
    from assistant import page
    out["the Tools page prints both passages"] = page.answer_line({"a": "x", "src": "extract", "cite": "c", "ms": 1,
                                                                   "passage": {"text": "P1", "cite": "c1"},
                                                                   "passage2": {"text": "P2", "cite": "c2"}}).count(
        "P1") == 1 and "P2" in page.answer_line({"a": "x", "src": "extract", "passage": {"text": "P1", "cite": "c1"},
                                                "passage2": {"text": "P2", "cite": "c2"}})
    # AM4-r2 MED-1: a ';' / '·' clause stays in its sentence - the pick holds the "nothing but" clause; the Greek
    # question mark ';' still ends a sentence; abbreviations do not
    FRAG = [{"id": "en:9:2", "ch": "9", "chapter": "Modes", "heading": "", "term": "",
             "text": "Extra rules: only the joystick enters MANUAL; TRAINING only from STANDBY by the PC; nothing but "
                     "STANDBY until the outputs are armed (chapter 12). Press e.g. the STANDBY button. Τι κάνει το "
                     "STANDBY; Κερδίζει πάντα· ο υπολογιστής δεν το παρακάμπτει ποτέ. Το ίδιο π.χ. στο HDG LOCK."}]
    ss = A.sentences(FRAG[0]["text"])
    out["sentences(): ';' and '·' clauses joined, the Greek question mark ';' + '.' split, 'e.g.' / 'π.χ.' not"] = \
        ss == ["Extra rules: only the joystick enters MANUAL; TRAINING only from STANDBY by the PC; nothing but STANDBY "
               "until the outputs are armed (chapter 12).", "Press e.g. the STANDBY button.", "Τι κάνει το STANDBY;",
               "Κερδίζει πάντα· ο υπολογιστής δεν το παρακάμπτει ποτέ.", "Το ίδιο π.χ. στο HDG LOCK."]
    fp = [s for s, _h in A.pick_sentences(FRAG)]
    out["picks: the whole ';'-sentence offered, no pick ends in ';' / '·', the question not offered"] = \
        fp[0] == ss[0] and all(not s.endswith((";", "·")) for s in fp) and ss[2] not in fp and ss[3] in fp
    out["ends_sentence: . ! ? + the Greek question mark; not an English ';' or '·'"] = \
        A.ends_sentence("Τι κάνει;") and A.ends_sentence("Done.") and not A.ends_sentence("by the PC;") and \
        not A.ends_sentence("πάντα·")
    # AM4-r2 / W112: the extract score floor - below it nothing is picked, can't find said, the passage still shown
    low = {"cant_find": False, "why": "", "hits": [dict(HATCH[0], score=5.0), dict(HATCH[1], score=4.0)]}
    r = A.Answerer(A.StubLlm("1"), terms, mode="extract", extract_floor=7.2).answer("πού κλείνω την καταπακτή;", "el",
                                                                                    low)
    out["extract floor: best score 5.0 < 7.2 -> can't find + the passage shown, the model not asked"] = \
        r["src"] == "cant_find" and r["a"] == A.CANT["el"] and r["passage"]["text"] == HATCH[0]["text"] and \
        r["floor"] == 7.2 and r["gen_ms"] == 0
    r = A.Answerer(A.StubLlm("1"), terms, mode="extract", extract_floor=7.2).answer(
        "πού κλείνω την καταπακτή;", "el", {"cant_find": False, "why": "", "hits": [dict(HATCH[0], score=7.2)]})
    out["extract floor: a score at the floor picks"] = r["src"] == "extract"
    r = A.Answerer(A.StubLlm("1"), terms, mode="extract", extract_floor=0).answer("πού κλείνω την καταπακτή;", "el", low)
    out["extract floor 0 = no floor"] = r["src"] == "extract"
    out["the default floor is the measured one (client + answer agree)"] = A.EXTRACT_FLOOR == 7.2 and \
        __import__("assistant.client", fromlist=["x"]).DEFAULT_EXTRACT_FLOOR == A.EXTRACT_FLOOR
    r = A.Answerer(A.StubLlm("Πατήστε «ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ»."), terms, extract_floor=7.2).answer(
        "πού κλείνω την καταπακτή;", "el", low)
    out["the floor is extract's only (generate unchanged)"] = r["src"] == "llm"
    # AM4-r3 LOW-b: no setting (None / "") = the DEFAULT floor; only an explicit 0 is off - in answer.py AND client.py
    out["floor None / '' = the default, 0 = off, garbage = the default (answer.Answerer)"] = \
        A.Answerer(A.StubLlm("1"), mode="extract", extract_floor=None).extract_floor == A.EXTRACT_FLOOR and \
        A.Answerer(A.StubLlm("1"), mode="extract", extract_floor="").extract_floor == A.EXTRACT_FLOOR and \
        A.Answerer(A.StubLlm("1"), mode="extract", extract_floor=0).extract_floor == 0.0 and \
        A.Answerer(A.StubLlm("1"), mode="extract", extract_floor="x").extract_floor == A.EXTRACT_FLOOR and \
        A.Answerer(A.StubLlm("1"), mode="extract", extract_floor="5.5").extract_floor == 5.5
    cl = client.Assistant(View(), settings=Sets(assistant_extract_floor=None), stt_kind="stub")
    cl0 = client.Assistant(View(), settings=Sets(assistant_extract_floor=0), stt_kind="stub")
    cl5 = client.Assistant(View(), settings=Sets(assistant_extract_floor=5), stt_kind="stub")
    out["client: a null setting = the default floor, 0 = off, 5 = 5"] = cl._floor() == client.DEFAULT_EXTRACT_FLOOR \
        and cl0._floor() == 0.0 and cl5._floor() == 5.0 and \
        client.Assistant(View(), settings=Sets(), stt_kind="stub")._floor() == client.DEFAULT_EXTRACT_FLOOR
    # AM4-r3 MED-1: a pick followed by a LIMITING sentence about the same thing is offered only together with it
    LIM = [{"id": "el:9:3", "ch": "9", "chapter": "Χάρτες", "heading": "", "term": "",
            "text": "Οι χάρτες είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα. Η καταπακτή κλείνει με το κουμπί. "
                    "Ο καιρός αλλάζει γρήγορα στο Σαρωνικό."}]
    lp = [s for s, _h in A.pick_sentences(LIM)]
    out["pick + its limiting sentence = ONE unit (both, consecutive manual text); the limiting one alone too"] = \
        lp == ["Οι χάρτες είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα.", "ΔΕΝ είναι για ναυσιπλοΐα.",
               "Η καταπακτή κλείνει με το κουμπί.", "Ο καιρός αλλάζει γρήγορα στο Σαρωνικό."]
    r = A.Answerer(A.StubLlm("1"), terms, mode="extract").answer("τι είναι οι χάρτες;", "el",
                                                               {"cant_find": False, "why": "", "hits": LIM})
    out["the joined unit is said word for word (the exception included)"] = r["src"] == "extract" and \
        r["a"] == "Οι χάρτες είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα."
    RET = [{"id": "en:9:4", "ch": "9", "chapter": "Modes", "heading": "", "term": "",
            "text": "AUTO RETURN drives the route back to waypoint 1 under the AUTO START conditions. The way back is "
                    "NEVER started out of REMOTE MANUAL or MANEUVER. The hatch closes in 3 s. The boat files are never "
                    "edited by hand on the gateway PC by anyone. Never edit them."}]
    rp = [s for s, _h in A.pick_sentences(RET)]
    out["'NEVER out of REMOTE MANUAL / MANEUVER' travels with the AUTO RETURN sentence; an unrelated negation does "
        "not; a short 'Never ...' (no subject) travels with the sentence before it"] = \
        rp[0] == "AUTO RETURN drives the route back to waypoint 1 under the AUTO START conditions. The way back is " \
                 "NEVER started out of REMOTE MANUAL or MANEUVER." and "The hatch closes in 3 s." in rp and \
        not any("The hatch closes in 3 s. The boat" in s for s in rp) and \
        "The boat files are never edited by hand on the gateway PC by anyone. Never edit them." in rp
    longs = " ".join(["word"] * (A.PICK_JOIN_WORDS - 5)) + " about the route."
    TOO = [dict(RET[0], text="Routes " + longs + " Never leave the route without a word about it. The end is here.")]
    tp = [s for s, _h in A.pick_sentences(TOO)]
    out["a statement + exception over PICK_JOIN_WORDS = neither the statement nor the pair is offered"] = \
        not any(s.startswith("Routes word") for s in tp) and "Never leave the route without a word about it." in tp
    out["limits(): limiting words + >= 2 shared content words"] = A.limits(A.norm("ΔΕΝ είναι για ναυσιπλοΐα, οι "
                                                                                  "χάρτες."), A.norm("Οι χάρτες είναι βοηθήματα ναυσιπλοΐας.")) \
        and not A.limits(A.norm("The boat files are never edited by hand on the gateway."), A.norm("The hatch closes in 3 s.")) \
        and not A.limits(A.norm("The charts are shown on screen 2."), A.norm("The charts are aids.")) and \
        A.limits(A.norm("Only the captain closes the hatch."), A.norm("The hatch closes with the button, captain.")) \
        and A.limits(A.norm("ΔΕΝ είναι για ναυσιπλοΐα."), A.norm("Οι χάρτες είναι βοηθήματα.")) and \
        not A.limits(A.norm("Never in any of the twelve modes is the hatch relay switched by a station."),
                     A.norm("The charts are aids."))
    # AM4-r3 MED-3 / LOW-a: a glued "* bullet" list longer than LONG_PICK_WORDS is offered item by item
    items = ["* Item %d of the list with a few words in it number %d" % (i, i) for i in range(1, 8)]
    LIST = [dict(RET[0], text="The list: " + " ".join(items) + " The end.")]
    ip = [s for s, _h in A.pick_sentences(LIST)]
    out["a long glued bullet list = its items (each byte-identical manual text), not one 80-word pick"] = \
        "Item 1 of the list with a few words in it number 1" in ip and len([s for s in ip if s.startswith("Item")]) == 7 \
        and not any(len(s.split()) > A.LONG_PICK_WORDS for s in ip) and all(s in LIST[0]["text"] for s in ip)
    short_list = [dict(RET[0], text="The list: * Item one here now * Item two here now and more words to say.")]
    out["a short bullet list stays one sentence (no shorter way needed)"] = [s for s, _h in A.pick_sentences(
        short_list)] == ["The list: * Item one here now * Item two here now and more words to say."]
    # AM4-r2: a verbatim pick is never refused because ANOTHER manual sentence negates a word of it (measured: 6 / 34
    # English + 8 / 97 Greek true manual sentences were), nor by a '/'-joined term ("κίτρινο/πορτοκαλί")
    NEGP = [{"id": "el:16:3", "ch": "16", "chapter": "Το μαγγέμα UAV", "heading": "", "term": "",
             "text": "Κρατήστε το κουμπί ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ. Χωρίς HELM κανείς δεν πατά το κουμπί ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ. "
                     "Χρώματα: ΚΟΚΚΙΝΟ μόνο για συναγερμούς, ΚΙΤΡΙΝΟ/ΠΟΡΤΟΚΑΛΙ για προσοχή."}]
    fnd = {"cant_find": False, "why": "", "hits": [dict(NEGP[0], score=9.0)]}
    out["generate: 'Κρατήστε το κουμπί ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ.' is refused by the negating sentence (the lexical rule)"] = \
        any("negates" in p for p in A.postcheck("Κρατήστε το κουμπί ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ.", NEGP, terms, "el"))
    r = A.Answerer(A.StubLlm("1"), terms, mode="extract").answer("πώς κλείνω την καταπακτή;", "el", fnd)
    out["extract: the same VERBATIM sentence passes (the negation rule is not a backstop for the manual's own text) "
        "- AM4-r3 MED-1: WITH the limiting sentence that follows it"] = r["src"] == "extract" and \
        r["a"] == "Κρατήστε το κουμπί ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ. Χωρίς HELM κανείς δεν πατά το κουμπί ΚΛΕΙΣΙΜΟ ΚΑΤΑΠΑΚΤΗΣ."
    r = A.Answerer(A.StubLlm("3"), terms, mode="extract").answer("ποια χρώματα;", "el", fnd)
    out["extract: a verbatim sentence with a '/'-joined capital term passes"] = r["src"] == "extract"
    out["a bullet item is a pick without its bullet; a short limiting item travels with the one before (its own "
        "bullet kept = consecutive manual text)"] = [s for s, _h in A.pick_sentences(
            [dict(FRAG[0], text="* The ports must be free. - Never start two demos.")])] == [
            "The ports must be free. - Never start two demos.", "Never start two demos."] and \
        [s for s, _h in A.pick_sentences([dict(FRAG[0], text="* The ports must be free. - Start the demo twice.")])] \
        == ["The ports must be free.", "Start the demo twice."]
    report("AS21 retrieval-only mode + cut generations + answer mode extract", all(out.values()),
           {k: v for k, v in out.items() if not v} or "all %d checks" % len(out))


def as22():
    """The answer through the REAL worker process (stub recogniser + stub model, network blocked): heard, then the
    answer; the client stays THINKING until it, measures release -> answer, hands it to the view; ask_text."""
    clk = Clock()
    v = View()
    v.answers = []
    v.on_answer = lambda txt, info: v.answers.append((txt, info))
    n = len(MARKERS)
    os.environ["TRYGONS_ASSISTANT_LLM_DELAY"] = "1.0"       # the answer comes 1 s after "heard" (deterministic)
    a = stub(view=v, clock=clk, py=sys.executable)
    out = {}
    try:
        a.start()
        ready = wait(a, lambda: a.state == client.READY)
        out["ready with the model info"] = ready and a.llm.get("name") == "stub"
        a.press("button")
        a.inject(tone(1.0))
        clk.t += 1.0
        a.release()
        wait(a, lambda: bool(v.heard))
        out["heard -> still THINKING (the answer follows)"] = bool(v.heard) and a.state == client.THINKING
        clk.t += 2.5
        wait(a, lambda: bool(v.answers))
        txt, info = v.answers[-1] if v.answers else ("", {})
        out["the answer: the stub model's grounded sentence, post-checked"] = info.get("src") == "llm" and \
            bool(txt) and info.get("cite", "").startswith(("Κεφάλαιο", "Ξενάγηση"))
        out["release -> answer time measured by the client"] = info.get("answer_ms") == 2500
        out["the answer may be spoken (the horn was not pressed)"] = info.get("speak") is True and \
            "passage" in info and bool(info["passage"]["text"])
        out["READY again"] = a.state == client.READY
        a._send({"op": "ask_text", "q": "πες μου ένα ανέκδοτο", "lang": "el"})
        wait(a, lambda: len(v.answers) > 1)
        out["ask_text: not in the manual -> can't find"] = len(v.answers) > 1 and \
            v.answers[-1][1].get("src") == "cant_find" and v.answers[-1][0] == "Δεν βρίσκω απάντηση στο εγχειρίδιο."
        # AM4-r LOW-3: «ΟΜΙΛΙΑ» pressed while the answer is on its way = the answer is shown with speak = False
        # (AM5 enforces: never spoken); a later question without the horn speaks again
        n2 = len(v.answers)
        a.press("button")
        a.inject(tone(1.0))
        clk.t += 1.0
        a.release()
        wait(a, lambda: len(v.heard) > 1)
        a.horn_pressed()
        clk.t += 2.5
        wait(a, lambda: len(v.answers) > n2)
        info2 = v.answers[-1][1] if len(v.answers) > n2 else {}
        out["horn pressed while answering -> speak False (shown, never spoken)"] = info2.get("speak") is False and \
            info2.get("no_speak_why") == "horn" and info2.get("src") == "llm"
        n3 = len(v.answers)
        a.press("button")
        a.inject(tone(1.0))
        clk.t += 1.0
        a.release()
        clk.t += 2.5
        wait(a, lambda: len(v.answers) > n3)
        out["the next answer without the horn speaks again"] = len(v.answers) > n3 and \
            v.answers[-1][1].get("speak") is True
        # the horn HELD (not a fresh press) while answering = speak False too
        n4 = len(v.answers)
        a.press("button")
        a.inject(tone(1.0))
        clk.t += 1.0
        a.release()
        v.horn = True
        clk.t += 2.5
        wait(a, lambda: len(v.answers) > n4)
        v.horn = False
        out["the horn held while answering -> speak False"] = len(v.answers) > n4 and \
            v.answers[-1][1].get("speak") is False
    finally:
        a.stop()
        os.environ.pop("TRYGONS_ASSISTANT_LLM_DELAY", None)
    tried = attempts(MARKERS[n:])
    out["no network attempt in the worker"] = len(MARKERS) > n and not tried
    report("AS22 worker answer (stub model)", all(out.values()), "%s; attempts %s" % (out, tried or "none"))


def as23():
    """tools/get_llm.py: every download pinned (revision + size + sha256), the bindings wheel pinned, its
    dependencies hash-locked; llama.cpp runs IN the worker (bindings) - never llama-server / llama-cli / a port."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("get_llm", os.path.join(REPO, "tools", "get_llm.py"))
    gl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gl)
    out = {}
    hexre = re.compile(r"^[0-9a-f]+$")
    out["every model: a 40-hex revision, a size, a 64-hex sha256, a .gguf, a licence"] = all(
        len(rev) == 40 and hexre.match(rev) and size > 1e9 and len(sha) == 64 and hexre.match(sha) and
        fn.endswith(".gguf") and lic for repo, rev, fn, size, sha, lic, note in gl.MODELS.values())
    out["the owner's candidates (Qwen2.5-7B, Meltemi, Krikri, Llama-3.1-8B, a 3B)"] = {
        "qwen2.5-7b", "meltemi-7b", "krikri-8b", "llama3.1-8b", "qwen2.5-3b", "llama3.2-3b"} <= set(gl.MODELS)
    out["the chosen model = the station's default"] = gl.CHOSEN == paths.LLM_DEFAULT and gl.CHOSEN in gl.MODELS
    # AM4-r LOW-2: the worker verifies the model file's sha256 against paths.LLM_PINNED = exactly get_llm's table
    out["paths.LLM_PINNED = get_llm.MODELS (file -> size, sha256)"] = paths.LLM_PINNED == {
        fn: (size, sha) for repo, rev, fn, size, sha, lic, note in gl.MODELS.values()}
    with open(os.path.join(ASST, "worker.py"), encoding="utf-8") as f:
        wsrc = f.read()
    out["the worker calls paths.llm_verify before answer.LlamaLlm"] = wsrc.index("paths.llm_verify(") < \
        wsrc.index("answer.LlamaLlm(path")
    # AM4-r2 LOW-d: a file already verified (same path, size, mtime) is NOT hashed again by a guard reload; a changed
    # file (size / mtime / content) is
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        fp = os.path.join(td, "pinned-test.gguf")
        with open(fp, "wb") as f:
            f.write(b"GGUF" + b"\0" * 1020)
        sha = paths.sha256_of(fp)
        saved = dict(paths.LLM_PINNED)
        calls = []
        real = paths.sha256_of
        paths.LLM_PINNED["pinned-test.gguf"] = (1024, sha)
        paths.sha256_of = lambda p, chunk=1 << 20: (calls.append(p), real(p, chunk))[1]
        try:
            cache = {}
            r1 = paths.llm_verify(fp, cache=cache)
            r2 = paths.llm_verify(fp, cache=cache)
            n_after_two = len(calls)
            with open(fp, "r+b") as f:
                f.seek(10)
                f.write(b"\1")
            os.utime(fp, (time.time() + 5, time.time() + 5))
            r3 = paths.llm_verify(fp, cache=cache)
            r4 = paths.llm_verify(fp, cache=None)
        finally:
            paths.sha256_of = real
            paths.LLM_PINNED.clear()
            paths.LLM_PINNED.update(saved)
        out["sha256 cache: verified once, the second call cached (no hash), a changed file hashed + refused, "
            "cache=None hashes always"] = r1 == (True, "sha256 verified") and r2 == (True, "sha256 verified (cached)") \
            and n_after_two == 1 and r3[0] is False and "sha256" in r3[1] and len(calls) == 3 and r4[0] is False
    url, size, sha = gl.WHEEL
    out["the bindings wheel: a release asset, size + sha256"] = url.startswith(
        "https://github.com/abetlen/llama-cpp-python/releases/download/") and size > 1e8 and len(sha) == 64
    with open(gl.REQ, encoding="utf-8") as f:
        req = f.read()
    reqs = [x for x in re.findall(r"^([A-Za-z0-9_.-]+)==(\S+)", req, re.M)]
    out["dependencies: every one pinned + hashed"] = len(reqs) >= 4 and req.count("--hash=sha256:") >= len(reqs)
    with open(os.path.join(REPO, "tools", "get_llm.py"), encoding="utf-8") as f:
        src = f.read()
    out["pip only --require-hashes --no-deps --only-binary into tools/bin"] = "--require-hashes" in src and \
        "--no-deps" in src and '":all:"' in src and 'os.path.join(HERE, "bin", "assistant", "llm")' in src
    out["downloads only into tools/bin (gitignored)"] = subprocess.run(
        ["git", "check-ignore", "-q", "tools/bin/assistant/llm/x.gguf"], cwd=REPO).returncode == 0
    # no assistant module may run llama.cpp as a server / a port / an exe (code strings + imports, not comments)
    bad = []
    for m in asst_modules():
        pth = os.path.join(ASST, m + ".py")
        with open(pth, encoding="utf-8") as f:
            tree = ast.parse(f.read())
        for nd in ast.walk(tree):
            if isinstance(nd, ast.Constant) and isinstance(nd.value, str):
                v = nd.value.lower()
                for w in ("llama-server", "llama_cpp.server", "llama-cli", "--port", "--host", "uvicorn"):
                    if w in v and not (isinstance(nd, ast.Constant) and _is_docstring(tree, nd)):
                        bad.append((m, w))
            if isinstance(nd, (ast.Import, ast.ImportFrom)):
                names = [x.name for x in nd.names] + ([nd.module] if isinstance(nd, ast.ImportFrom) and nd.module
                                                        else [])
                for nm in names:
                    if nm and (nm.startswith("llama_cpp.server") or nm.split(".")[0] in ("uvicorn", "fastapi",
                                                                                         "sse_starlette")):
                        bad.append((m, nm))
                    if nm and nm.split(".")[0] == "llama_cpp" and m != "answer":
                        bad.append((m, "llama_cpp outside answer.py"))
    out["no llama-server / llama-cli / port / server import; llama_cpp only in answer.py"] = not bad
    lp = paths.llm_py_dir()
    net = "-"
    if lp:
        sys.path.insert(0, lp)
        try:
            net, nfiles = site_walk_dir(lp, ["llama_cpp"])
            net = "%s (%d files)" % (net or "none", nfiles)
        finally:
            sys.path.remove(lp)
    report("AS23 get_llm.py pins + llama.cpp in-process only", all(out.values()),
           "%s; problems %s; the bindings' static import walk (information, like AS2) reaches the network "
           "modules %s - the server sub-package needs uvicorn / fastapi, which are NOT installed" % (out, bad or "none",
                                                                                                    net))


def _is_docstring(tree, node):
    for n in ast.walk(tree):
        if isinstance(n, (ast.Module, ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef)) and n.body and \
                isinstance(n.body[0], ast.Expr) and n.body[0].value is node:
            return True
    return False


def site_walk_dir(site, roots, limit=4000):
    """A static import walk inside one folder (the llama.cpp bindings) -> ({network module: importer}, files)."""
    seen, todo, net, files = set(), [(r, "worker") for r in roots], {}, 0
    while todo and files < limit:
        mod, via = todo.pop()
        if mod in seen:
            continue
        seen.add(mod)
        top = mod.split(".")[0]
        if top in NET_MODULES:
            net.setdefault(top, via)
        b = os.path.join(site, *mod.split("."))
        p = next((x for x in (b + ".py", os.path.join(b, "__init__.py")) if os.path.isfile(x)), None)
        if p is None:
            continue
        files += 1
        pkg = mod if p.endswith("__init__.py") else mod.rpartition(".")[0]
        with open(p, encoding="utf-8", errors="replace") as f:
            tree = ast.parse(f.read())
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                todo += [(x.name, mod) for x in n.names]
            elif isinstance(n, ast.ImportFrom):
                if n.level:
                    parts = pkg.split(".")
                    m = ".".join(parts[:len(parts) - (n.level - 1)] if n.level > 1 else parts) + \
                        ("." + n.module if n.module else "")
                else:
                    m = n.module or ""
                if m:
                    todo.append((m, mod))
                    todo += [(m + "." + x.name, mod) for x in n.names if x.name != "*"]
    return dict(sorted(net.items())), files


def as25():
    """AM4-r2 MED-1, over the WHOLE committed index (both languages): no pickable sentence ends in ';' / '·', none
    begins lower-case, and no pick is a CUT clause (its own piece did not end a sentence and a lower-case clause with
    a negation / "only" followed it - what the old splitter did 452 times). The reviewer's example (MANUAL.txt
    "TRAINING only from STANDBY by the PC; nothing but STANDBY ...") is ONE pick. Mutation: the old splitter makes
    the check fail. AM4-r3 MED-1: NO pick is followed (in its passage) by a LIMITING sentence about the same thing
    (answer.limits: a negation / only / except ... sharing >= 2 content words) that is not part of the pick - every
    pick is byte-identical consecutive manual text; the AM4-r2 pick list (no joining) fails it (the reviewer
    counted 105 el / 140 en). MED-3 / LOW-a: no pick over LONG_PICK_WORDS still holds a glued "* bullet" list."""
    from assistant import answer as A, index
    ONLY = re.compile(r"(?<![\w])(?:only|μονο)(?![\w])")

    def old_picks(hits):                   # the AM4-r pick list: whole pieces, a ';' / '·' end not excluded
        return [(s, h) for h in hits for s in A.sentences(h["text"]) if len(s.split()) >= 4 and A._whole(s)]

    def r2_picks(hits):                    # the AM4-r2 pick list: whole sentences, nothing joined
        out = []
        for h in hits:
            for s in A.sentences(h["text"]):
                s = A._BULLET_RE.sub("", s)
                if len(s.split()) >= 4 and A._whole(s) and not s.rstrip().endswith((";", "·")):
                    out.append((s, h))
        return out

    def atoms_of(text):
        """[(offset, atom)]: the passage's sentence pieces (bullets taken off), a long glued bullet list as its
        items, in text order with where each one starts in the text."""
        out, pos = [], 0
        for s in A.sentences(text):
            raw_at = text.find(s, pos)
            pos = raw_at + len(s)
            s = A._BULLET_RE.sub("", s)
            parts = [p.strip() for p in A._ITEM_RE.split(s) if p.strip()] \
                if len(s.split()) > A.LONG_PICK_WORDS and A._ITEM_RE.search(s) else [s]
            at = raw_at
            for part in parts:
                at = text.find(part, at)
                out.append((at, part))
                at += len(part)
        return out

    def scan(pick_fn=A.pick_sentences):
        n = {"picks": 0, "ends": 0, "lower": 0, "cut": 0, "example": 0, "limited": 0, "not_text": 0, "joined": 0,
             "long": 0, "long_list": 0, "example_limit": 0}
        for lang in ("el", "en"):
            for p in index.load(lang)["passages"]:
                text = p["text"]
                pieces = [A._BULLET_RE.sub("", s) for s in A.sentences(text)]
                atoms = atoms_of(text)
                picks = [s for s, _h in pick_fn([p])]
                n["picks"] += len(picks)
                n["ends"] += sum(1 for s in picks if s.rstrip().endswith((";", "·")))
                n["lower"] += sum(1 for s in picks if s.lstrip("«\"'([")[:1].islower())
                n["example"] += sum(1 for s in picks if "TRAINING only from STANDBY by the PC" in s and
                                    "nothing but STANDBY" in s)
                n["example_limit"] += sum(1 for s in picks if "είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα." in s)
                for i, s in enumerate(pieces[:-1]):
                    nxt = pieces[i + 1]
                    if s in picks and not (A.ends_sentence(s) or s.endswith(":")) and \
                            nxt.lstrip("«\"'([")[:1].islower() and (A.has_negation(A.norm(nxt)) or ONLY.search(
                                A.norm(nxt))):
                        n["cut"] += 1
                # AM4-r3 MED-1: every pick = consecutive text of its passage (bullets taken off at its start only);
                # the atom AFTER it must not limit it. MED-3: a long SINGLE-atom pick is never a glued bullet list
                for s in picks:
                    at = text.find(s)
                    if at < 0:
                        n["not_text"] += 1
                        continue
                    end = at + len(s)
                    inside = [i for i, (o, _a) in enumerate(atoms) if at <= o < end]
                    after = [a for o, a in atoms if o >= end]
                    n["joined"] += len(inside) > 1
                    if after and A.limits(A.norm(after[0]), A.norm(s)):
                        n["limited"] += 1
                    if len(inside) <= 1 and len(s.split()) > A.LONG_PICK_WORDS:
                        n["long"] += 1
                        n["long_list"] += bool(A._ITEM_RE.search(s))
        return n
    good = scan()
    r2 = scan(r2_picks)
    old_re, A._SPLIT_RE = A._SPLIT_RE, re.compile(r"(?<=[.!;·?])\s+(?=\S)")
    old_end = A.ends_sentence
    A.ends_sentence = lambda s: (s or "").rstrip().endswith((".", "!", "?"))      # the old splitter's view
    try:
        mutated = scan(old_picks)
    finally:
        A._SPLIT_RE, A.ends_sentence = old_re, old_end
    out = {"%d picks over both indexes" % good["picks"]: good["picks"] > 2500,
           "no pick ends in ';' / '·'": good["ends"] == 0, "no pick begins lower-case": good["lower"] == 0,
           "no pick is a cut clause (a lower-case never / only / not clause followed it)": good["cut"] == 0,
           "the reviewer's example is ONE pick with both clauses": good["example"] == 1,
           "mutation: the old splitter (split on ; ·) fails the check (%d cut, %d ending in ;/·)" % (
               mutated["cut"], mutated["ends"]): mutated["cut"] > 50 and mutated["ends"] > 50,
           # AM4-r3
           "every pick is byte-identical consecutive manual text (%d joined units)" % good["joined"]:
               good["not_text"] == 0 and good["joined"] > 50,
           "no pick is followed by a limiting sentence about the same thing that it leaves out": good["limited"] == 0,
           "the reviewer's example «... είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα.» travels together (%d)" % good[
               "example_limit"]: good["example_limit"] >= 1,
           "mutation: the AM4-r2 pick list (nothing joined) leaves %d exceptions out (reviewer: 105 el + 140 en)" % r2[
               "limited"]: r2["limited"] > 200,
           "no pick over %d words is a glued bullet list (%d long picks stay: real long sentences)" % (
               A.LONG_PICK_WORDS, good["long"]): good["long_list"] == 0 and good["long"] < 120}
    report("AS25 extract picks over the whole index: whole sentences, no clause fragments, exceptions included",
           all(out.values()), {k: v for k, v in out.items() if not v} or "all %d checks; %s" % (len(out), good))


def as29():
    """AM4-r3 LOW-b: the extract score floor's gate (tools/assistant_index.py floor, tools/check.sh): the frozen
    separation (tests/assistant/floor.json: the known off-manual TUNING questions under the floor, the known
    in-manual ones over it, the count over it, the code's floor = the tuned one) holds today; mutations: a frozen
    "below" question that now clears the floor, an "above" one under it, a code floor that differs, a count that
    fell = FAIL with the re-tune message."""
    sys.path.insert(0, os.path.join(HERE, "..", "tools"))
    import assistant_index as ai
    from assistant import answer as A
    out = {}
    ok, lines = ai.floor_check()
    out["the committed floor.json holds (el + en, the code's floor = the tuned one)"] = ok and len(lines) == 2
    with open(ai.FLOOR_FILE, encoding="utf-8") as f:
        frozen = json.load(f)
    out["floor.json = the code's floor, every language has below / above / above_n, nothing held-out in it"] = \
        frozen["floor"] == A.EXTRACT_FLOOR and all(
            frozen[lg]["below"] and frozen[lg]["above"] and frozen[lg]["above_n"] >= len(frozen[lg]["above"])
            for lg in ("el", "en")) and not any(
            i in ai.load_heldout(lg) for lg in ("el", "en") for i in frozen[lg]["below"] + frozen[lg]["above"])
    snap = ai.floor_snapshot(A.EXTRACT_FLOOR)
    out["floor --write would write the same separation (deterministic)"] = all(
        snap[lg] == frozen[lg] for lg in ("el", "en")) and snap["floor"] == frozen["floor"]
    # mutations
    m1 = json.loads(json.dumps(frozen))
    m1["el"]["below"].append(frozen["el"]["above"][0])          # an "off-manual" id that scores over the floor
    ok1, l1 = ai.floor_check(m1)
    m2 = json.loads(json.dumps(frozen))
    m2["en"]["above"].append(frozen["en"]["below"][0])          # an "in-manual" id under the floor
    ok2, l2 = ai.floor_check(m2)
    ok3, l3 = ai.floor_check(frozen, code_floor=A.EXTRACT_FLOOR + 1.0)
    m4 = json.loads(json.dumps(frozen))
    m4["el"]["above_n"] = frozen["el"]["above_n"] + ai.FLOOR_DRIFT + 1
    ok4, l4 = ai.floor_check(m4)
    m5 = json.loads(json.dumps(frozen))
    m5["floor"] = 9.0                                            # a re-tuned file without the code change
    ok5, l5 = ai.floor_check(m5)
    out["mutation: a frozen below-question over the floor = FAIL"] = not ok1 and any("now scores" in x for x in l1)
    out["mutation: a frozen above-question under the floor = FAIL"] = not ok2 and any("now scores" in x for x in l2)
    out["mutation: the code's floor differs from the tuned one = FAIL"] = not ok3 and any("code's floor" in x for x in l3)
    out["mutation: fewer in-manual questions clear the floor = FAIL"] = not ok4 and any("only" in x for x in l4)
    out["mutation: floor.json re-tuned, code not = FAIL"] = not ok5
    out["a failure says how to re-tune (W112 / W117, floor --write)"] = all(
        any("re-tune" in x and "floor --write" in x for x in ls) for ls in (l1, l2, l3, l4, l5))
    report("AS29 extract score floor gate (floor.json holds; mutations fail)", all(out.values()),
           {k: v for k, v in out.items() if not v} or "all %d checks; %s" % (len(out), lines))


def as24():
    """The REAL language model answers with the network blocked (Python blocker + the worker PID's endpoints):
    N/A (never PASS) when the model / bindings are not downloaded."""
    py = paths.assistant_python()
    name = "AS24 real language model, network blocked"
    if not py or not paths.llm_installed() or not paths.models_installed():
        return report(name, None, "language model not installed (python tools/get_llm.py) - N/A, never PASS")
    v = View()
    v.answers = []
    v.on_answer = lambda txt, info: v.answers.append((txt, info))
    a = client.Assistant(v, settings=Sets(), stt_kind="faster", mic_backend="null")
    a.worker_cmd = lambda: blocked_cmd(py)
    n0 = len(MARKERS)
    snaps, out = [], {}
    try:
        a.start()
        ready = wait(a, lambda: a.state == client.READY, 240)
        out["ready, the model loaded"] = ready and a.llm.get("mode") in ("gpu", "gpu+cpu", "cpu")
        if ready:
            snaps.append(pid_endpoints(a.proc.pid))
            from assistant import answer as AN, index as IX
            for lang, q, must in (("el", "πού κλείνω την καταπακτή;", "καταπακτ"),
                                  ("en", "how do I close the hatch?", "hatch")):
                # AM4-r3 LOW-e: this test never depends on the extract floor - the question must score WELL above it
                # (asserted explicitly; a floor "can't find" would otherwise look like a model failure)
                sc = IX.Searcher(IX.load(lang)).search(q)["hits"]
                sc = sc[0]["score"] if sc else 0.0
                out["%s: the question scores %.1f >= the extract floor %.1f + 2 (the test is floor-independent)" % (
                    lang, sc, AN.EXTRACT_FLOOR)] = sc >= AN.EXTRACT_FLOOR + 2.0
                k = len(v.answers)
                a._send({"op": "ask_text", "q": q, "lang": lang})
                a.state, a.ask_t, a.release_t = client.THINKING, time.monotonic(), time.monotonic()
                wait(a, lambda: len(v.answers) > k, 120)
                snaps.append(pid_endpoints(a.proc.pid))
                txt, info = v.answers[-1] if len(v.answers) > k else ("", {})
                import unicodedata
                shown = txt + " " + ((info.get("passage") or {}).get("text") or "")   # AM4-r: a failed check SHOWS
                pl = "".join(c for c in unicodedata.normalize("NFD", shown.lower()) if unicodedata.category(c) != "Mn")
                # AM4-r2 LOW-e: the model's OWN answer must pass (src llm / extract) - a "passage" fallback (a
                # failed post-check / no pick) is no pass
                out["%s answered (%s, %s ms gen, first token %s ms, sha %s ms)" % (
                    lang, info.get("src"), info.get("gen_ms"), info.get("first_ms"), a.llm.get("sha_ms"))] = \
                    info.get("src") in ("llm", "extract") and must in pl
                out["%s: the model file's sha256 was verified before the load" % lang] = \
                    a.llm.get("sha") == "sha256 verified"
    finally:
        a.stop()
    tried = attempts(MARKERS[n0:])
    eps = sorted({r for rows in snaps for r in (rows or [])})
    out["no network attempt, no endpoint"] = len(MARKERS) > n0 and not tried and not eps and \
        all(s is not None for s in snaps)
    report(name, all(out.values()), "%s; model %s; attempts %s; endpoints %s; answers %s" % (
        out, a.llm, tried or "none", eps or "none", [t[:80] for t, _i in v.answers]))


# --- AM5: speak + show ---------------------------------------------------------------------------------------------
def as26():
    """The speech normaliser (assistant/speech.py): capitals -> normal case (the passage's accent), acronyms as the
    "say." lines spell them, numbers with spoken units, ranges / arrows / <= / ±, the chapter intro with every answer
    (two chapters, a tour stop), a long pick = the intro-only sentence, can't find said as it is."""
    from assistant import speech
    out = {}
    v = speech.vocab_from(["Η καταπακτή κλείνει με το κουμπί HATCH CLOSE του κυβερνήτη ή της κυβερνήτριας."])
    t = speech.normalize("Πατήστε STANDBY: το GPS χάθηκε· 5 kn, 50 %, 10..120 m, HDG LOCK, ΚΑΤΑΠΑΚΤΗ -> F9, "
                         "USB / PC, <= 3 s, ± 2 °, 4G, XYZ", "el", v)
    gps = i18n.TL("el", "say.alarm.gps_lost")
    out["GPS as the say. line"] = speech.SAID["GPS"][0] in gps and "τζι πι ες" in t
    out["4G as the say. line"] = speech.SAID["4G"][0] in i18n.TL("el", "say.link.4g") and "τέσσερα τζι" in t
    out["STANDBY as the say. line"] = speech.SAID["STANDBY"][0] in i18n.TL("el", "say.mode.standby") and "στάντμπάι" in t
    out["capitals -> the passage's accented word"] = "καταπακτή" in t and "ΚΑΤΑΠΑΚΤΗ" not in t
    out["units"] = "5 κόμβοι" in t and "50 τοις εκατό" in t and "120 μέτρα" in t and "3 δευτερόλεπτα" in t and \
        "2 μοίρες" in t
    out["range / arrow / <= / ±"] = "10 έως 120" in t and "προς εφ 9" in t and "το πολύ 3" in t and "πλην ή συν 2" in t
    out["HDG LOCK spoken"] = "κλείδωμα πορείας" in t and "HDG" not in t
    out["unknown Latin capitals spelt"] = "εξ γουάι ζετ" in t
    out["/ = a pause"] = "γιου ες μπι, πι σι" in t
    out["no capital word left"] = not re.search(r"(?<![\w])[A-ZΑ-Ω]{2,}(?![\w])", t)
    e = speech.normalize("Press STANDBY: GPS lost; 5 kn, 50 %, 10..120 m, HDG LOCK, the HATCH -> F9, <= 3 s", "en")
    out["English"] = "G P S" in e and "5 knots" in e and "50 per cent" in e and "10 to 120 metres" in e and \
        "heading lock" in e and "hatch" in e and "at most 3 seconds" in e and "standby" in e
    # AM4-r3 LOW-d: a thousands group is read as the number the checker took it for (1500), never "one point five
    # hundred"; a decimal stays a decimal
    th = speech.normalize("The range is 1,500 m or 1.500 m; 0.125 s; 1.5 s; 12,345 kn; 2,5 s", "en")
    out["thousands '1,500 m' / '1.500 m' spoken as 1500 metres, decimals kept"] = th.count("1500 metres") == 2 and \
        "0.125 seconds" in th and "1.5 seconds" in th and "12345 knots" in th and "2,5 seconds" in th
    out["thousands (el)"] = "1500 μέτρα" in speech.normalize("Εμβέλεια 1.500 m από το σκάφος.", "el")
    out["intro one chapter"] = speech.intro("Κεφάλαιο 10 «ΣΥΝΑΓΕΡΜΟΙ»", "el") == "Από το εγχειρίδιο, κεφάλαιο 10."
    out["intro two chapters"] = speech.intro("Κεφάλαιο 10 «Α» + Κεφάλαιο 13A «Β»", "el") == \
        "Από το εγχειρίδιο, κεφάλαια 10 και 13 A."
    out["intro tour"] = speech.intro("Ξενάγηση: Βοηθός", "el") == "Από την ξενάγηση." and \
        speech.intro("Tour: x", "en") == "From the guided tour."
    out["intro en"] = speech.intro("Chapter 7 \"X\"", "en") == "From the manual, chapter 7."
    info = {"a": "Η καταπακτή κλείνει με το κουμπί HATCH CLOSE.", "cite": "Κεφάλαιο 9 «UAV»", "src": "extract",
            "passage": {"text": "Η καταπακτή κλείνει με το κουμπί HATCH CLOSE του κυβερνήτη."}, "lang": "el"}
    txt, kind = speech.spoken(info)
    out["spoken = intro + answer"] = kind == "answer" and txt.startswith("Από το εγχειρίδιο, κεφάλαιο 9. ") and \
        "χατς" in txt.lower() or (kind == "answer" and txt.startswith("Από το εγχειρίδιο, κεφάλαιο 9. "))
    long_a = " ".join(["λέξη"] * (speech.LONG_WORDS + 1)) + "."
    txt, kind = speech.spoken(dict(info, a=long_a))
    out["long pick = intro only"] = kind == "long" and txt == "Βρήκα την απάντηση στο κεφάλαιο 9 – δείτε το κείμενο." \
        and "λέξη" not in txt
    txt, kind = speech.spoken(dict(info, a=" ".join(["λέξη"] * speech.LONG_WORDS) + "."))
    out["50 words still spoken"] = kind == "answer" and "λέξη" in txt
    txt, kind = speech.spoken(dict(info, a=long_a, cite="Ξενάγηση: Βοηθός"))
    out["long tour pick"] = kind == "long" and txt.startswith("Βρήκα την απάντηση στην ξενάγηση")
    txt, kind = speech.spoken({"a": "Δεν βρίσκω απάντηση στο εγχειρίδιο.", "src": "cant_find", "cite": "Κεφάλαιο 9 «Α»",
                               "lang": "el"})
    out["can't find said alone (no chapter)"] = kind == "cant_find" and txt == "Δεν βρίσκω απάντηση στο εγχειρίδιο."
    txt, kind = speech.spoken({"a": "Δεν βρίσκω απάντηση στο εγχειρίδιο.", "src": "passage", "cite": "Κεφάλαιο 9 «Α»",
                               "passage": {"text": "x"}, "lang": "el"})
    out["failed check = can't find said, never the passage"] = kind == "cant_find" and "x" not in txt.split()
    out["nothing"] = speech.spoken({"a": "", "src": "extract", "cite": "Κεφάλαιο 1 «Α»"}) == ("", "")
    out["en two chapters"] = speech.spoken({"a": "Yes.", "cite": "Chapter 2 \"A\" + Chapter 5 \"B\"", "src": "extract",
                                            "lang": "en"})[0].startswith("From the manual, chapters 2 and 5. ")
    report("AS26 speech normaliser", all(out.values()), "%s; el: %s" % (out, t[:120]))


class FakeLamps:
    def __init__(self):
        self.warn = False

    def on(self, lv):
        import alerts
        return self.warn and lv == alerts.WARNING

    def unacked_codes(self, lv):
        return ["compass_lost"] if self.on(lv) else []


def fake_synth_factory(made):
    import sound

    def synth(lang, items, **kw):
        for t, p in items:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            sound.write_wav(p, sound._silence(1000))        # a 1 s "voice"
            made.append((lang, t, p))
        return len(items)
    return synth


def as27():
    """ALARM PRIORITY (CLAUDE.md W "ALARMS WIN", AM1-r M4) through sound.py + asstvoice.py with a recording player:
    the assistant's level is the lowest, never queued; a WARNING / CAUTION sound or a callout stops it at once (the
    rest dropped, [ΞΑΝΑ] replays); while a MASTER WARNING lamp is on / a callout is queued or sounding the answer is
    TEXT ONLY (no voice started, no WAV made), also when the alarm comes between the TTS run and the play; speak =
    False (the horn) / the horn held = never spoken; «ΟΜΙΛΙΑ» stops it at once; a long pick = the intro only; the
    temporary WAV is in the system temp folder and deleted after it played; the local log holds text only."""
    import alerts
    import asstvoice
    import sound
    from assistant import qlog, show
    clk = Clock()
    player = sound.NullPlayer()
    snd = sound.SoundSystem(player=player, clock=clk, library=sound.Library(cache=tempfile.mkdtemp()))
    snd.sched.enabled = True
    view = View()
    app = types.SimpleNamespace(sound=snd, lamps=FakeLamps(), asst=types.SimpleNamespace(view=view, answer=None))
    snd.app = app
    made = []
    vc = asstvoice.AsstVoice(app, synth=fake_synth_factory(made), clock=clk)
    logdir = client.log_dir()
    for fn in os.listdir(logdir):
        if fn.endswith(".jsonl"):
            os.remove(os.path.join(logdir, fn))
    info = show.demo_answer("el") or {"a": "Η καταπακτή κλείνει με το κουμπί HATCH CLOSE.", "cite": "Κεφάλαιο 9 «UAV»",
                                      "src": "extract", "lang": "el", "speak": True, "q": "πού κλείνω την καταπακτή;"}
    info = dict(info, speak=True)
    out = {}
    out["lowest level"] = sound.ASSISTANT < sound.FEEDBACK < sound.ADVISORY < sound.CAUTION < sound.WARNING

    def done(s=5.0):
        end = time.time() + s
        while time.time() < end:
            if vc.pending is not None:
                vc.tick()
                return True
            time.sleep(0.02)
        return False
    # (a) a plain answer is spoken: intro first, the WAV in the temp folder, deleted after its end
    vc.speak(info)
    out["TTS runs off the Tk thread"] = vc.why == "making"
    done()
    path = made[-1][2] if made else ""
    out["spoken"] = vc.why == "spoken" and player.played and player.played[-1] == path and os.path.isfile(path)
    out["the chapter first"] = made and made[-1][1].startswith("Από το εγχειρίδιο, κεφάλαιο ")
    out["WAV in the system temp folder, not the log folder, not sound_cache"] = path.startswith(tempfile.gettempdir()) \
        and not path.startswith(logdir) and "sound_cache" not in path
    clk.t += 2.0
    vc.tick()
    out["WAV deleted after playing"] = not os.path.exists(path) and vc.tmp is None
    # (b) a WARNING sound while speaking stops it AT ONCE: the player stopped, the rest dropped, nothing queued
    vc.speak(info)
    done()
    n = len(player.played)
    out["speaking again"] = vc.why == "spoken" and snd.assistant_playing()
    snd.alarm("compass_lost", alerts.WARNING)
    out["WARNING interrupts: player.stop then the warning"] = player.played[n] is None and \
        player.played[n + 1] is not None and not snd.assistant_playing()
    out["never queued"] = not any(q.code == snd.ASST_CODE for q in snd.sched.queue)
    vc.tick()
    out["panel says interrupted"] = vc.why == "interrupted"
    clk.t += 10.0
    snd.sched.tick(snd.warn_item)
    # (c) a MASTER WARNING lamp on this PC = TEXT ONLY: no voice started, no WAV made
    app.lamps.warn = True
    m0 = len(made)
    vc.speak(info)
    out["lamp WARNING -> text only, no WAV"] = vc.why == "alarm" and len(made) == m0 and vc.pending is None
    app.lamps.warn = False
    # (d) [ΞΑΝΑ] later speaks it
    vc.repeat()
    done()
    out["ΞΑΝΑ speaks it later"] = vc.why == "spoken" and len(made) == m0 + 1
    snd.assistant_stop()
    clk.t += 5.0
    vc.tick()
    # (e) a callout sounding = text only; a callout QUEUED = text only
    snd.say("mode.auto")
    out["callout sounds"] = snd.sched.playing()
    vc.speak(info)
    out["callout sounding -> text only"] = vc.why == "callout" and len(made) == m0 + 1
    snd.say("mode.hdg_lock")                                   # queued behind the first
    clk.t += 20.0
    snd.sched.tick(snd.warn_item)                               # the first ended, the second starts
    clk.t += 20.0
    snd.sched.tick(snd.warn_item)
    out["quiet again"] = not snd.sched.busy()
    # (f) the alarm comes BETWEEN the TTS run and the play: still text only, the WAV removed
    vc.speak(info)
    end = time.time() + 5.0
    while time.time() < end and vc.pending is None:
        time.sleep(0.02)
    p2 = vc.pending[1] if vc.pending else ""
    snd.say("mode.standby")                                    # an advisory starts now
    vc.tick()
    out["alarm between TTS and play -> text only + WAV removed"] = vc.why == "callout" and p2 and \
        not os.path.exists(p2)
    clk.t += 20.0
    snd.sched.tick(snd.warn_item)
    # (g) speak False (the horn was pressed meanwhile) / the horn held = never spoken; «ΟΜΙΛΙΑ» stops at once
    m1 = len(made)
    vc.speak(dict(info, speak=False, no_speak_why="horn"))
    out["speak False -> horn, nothing made"] = vc.why == "horn" and len(made) == m1
    view.horn = True
    vc.speak(info)
    out["horn held -> text only"] = vc.why == "horn" and len(made) == m1
    vc.repeat()
    out["ΞΑΝΑ while the horn is held -> no"] = vc.why == "horn" and len(made) == m1
    view.horn = False
    vc.speak(info)
    done()
    out["spoken once the horn is free"] = vc.why == "spoken" and snd.assistant_playing()
    p3 = vc.tmp
    vc.stop()                                                  # = hailer._asst_horn -> asst_voice.stop()
    out["ΟΜΙΛΙΑ stops at once + the WAV gone"] = vc.why == "stopped" and not snd.assistant_playing() and \
        player.played[-1] is None and not os.path.exists(p3)
    # (h) a long pick: the intro-only sentence is spoken, the text stays on the panel
    long_info = dict(info, a=" ".join(["λέξη"] * 70) + ".")
    vc.speak(long_info)
    done()
    out["long pick -> the intro only"] = vc.why == "long" and made[-1][1].startswith("Βρήκα την απάντηση στο κεφάλαιο") \
        and "λέξη" not in made[-1][1]
    vc.stop()
    # (i) the sound setting off = text only (never a hidden voice)
    snd.sched.enabled = False
    vc.speak(info)
    out["sound off -> text only"] = vc.why == "sound_off"
    snd.sched.enabled = True
    # (j) the local log: THIS PC's temp folder (TRYGONS_TEST), text only, the voice outcome per answer, no audio
    files = [f for f in os.listdir(logdir) if f.endswith(".jsonl")]
    rows = []
    for f in files:
        with open(os.path.join(logdir, f), encoding="utf-8") as fh:
            rows += [json.loads(x) for x in fh if x.strip()]
    whys = [r.get("voice") for r in rows]
    out["log written (text, one line per answer)"] = bool(rows) and all(r.get("q") and r.get("a") and r.get("t")
                                                                          for r in rows)
    out["log holds the voice outcome"] = {"spoken", "alarm", "callout", "horn", "long", "sound_off"} <= set(whys)
    out["log has no audio"] = not any(k in r for r in rows for k in ("samples", "wav", "audio", "pcm")) and \
        not any(fn.endswith((".wav", ".ogg", ".mp3")) for fn in os.listdir(logdir))
    out["log entry keeps passage + cite"] = all("passage_id" in r for r in rows if r.get("src") != "cant_find")
    out["qlog.entry never copies the found texts"] = "text" not in json.dumps(qlog.entry(info))
    report("AS27 alarm priority + voice rules + log", all(out.values()), "%s; made %d; played %s" % (
        out, len(made), [os.path.basename(p) if p else None for p in player.played][-6:]))


def as28(rig):
    """Screen 2's panel "assistant" (AM5; the 13n1 look through overlay.py): the AI badge, the recognised question, the
    answer (cut: a long pick never overflows), how it was spoken, the FULL passage + chapter (fold / open), [ΞΑΝΑ]
    (the voice again), [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] (the manual at the passage's lines, read-only, highlighted); the panel in the
    fit matrix; the message line cut; screen 1's 12 s notice; screenshots docs/screens/assistant."""
    import manualwin
    from assistant import show
    ap = rig.ap
    a = ap.asst
    i18n.set_lang("el", save=False)
    st0, last0, ans0 = a.state, a.last, a.answer
    calls = []
    orig_rep = ap.asst_voice.repeat
    ap.asst_voice.repeat = lambda: calls.append("repeat") or True
    out = {}
    try:
        # a clean screen 2 (the earlier tests left panels, alarms and a real question behind): the content checks run
        # at 1920x1080 where the full passage is always shown; the fit matrix below fills everything again
        ts.no_relay_boat(rig)
        for k in ("alarm_bits", "alarm", "helm", "profile", "ll_state", "ll_flags", "power_raised", "coast_valid",
                  "coast_dist_m", "ll_left_s", "near_shore"):
            rig.extra.pop(k, None)
        ap.gw_status = {}
        ap.decisions.clear()
        a.last, a.answer = None, None
        ap.asst_passage_open, ap.asst_auto_fold, ap.asst_force_open = True, 0, False
        a._set(client.READY)
        rig.size(1920, 1080, 1.0)
        ap.render(force=True)
        out["no panel before a question"] = ts.op(ap, "asst_a") is None and not any(p.key == "assistant"
                                                                                      for p in ap.ov.panels)
        for lg in ("el", "en"):
            i18n.set_lang(lg, save=False)
            info = show.demo_answer(lg)
            out["demo answer %s" % lg] = bool(info) and bool(info.get("passage", {}).get("text"))
            a.last = {"q": info["q"], "lang": lg, "audio_s": 2.0, "ms": 800, "found": info["found"]}
            a.answer = dict(info, speak=True, answer_ms=2100)
            ap.asst_on_heard(info["q"], {})
            ap.asst_on_answer(info["a"], a.answer)
            ap.render(force=True)
            pan = [p for p in ap.ov.panels if p.key == "assistant"]
            out["panel drawn %s" % lg] = bool(pan) and ts.op(ap, "assistant") is not None
            badge = ts.op(ap, "ai_badge_asst")
            out["AI badge in the panel %s" % lg] = badge is not None and pan and overlay.inside(badge["r"], pan[0].r, 1.5)
            q = ts.op(ap, "asst_q")
            out["question shown %s" % lg] = q is not None and info["q"][:20] in q["s"]
            an = ts.op(ap, "asst_a")
            out["answer shown %s" % lg] = an is not None and an["s"].split()[0] in info["a"]
            ps = ts.op(ap, "asst_passage")
            out["full passage + chapter %s" % lg] = ps is not None and info["passage"]["text"][:40] in ps["s"] and \
                info["cite"] in ps["s"]
            for tag in ("asst_again", "asst_manual", "asst_text"):
                out["button %s %s" % (tag, lg)] = ts.op(ap, tag) is not None and ts.op(ap, tag).get("enabled") is True
            out["message line %s" % lg] = info["a"][:30] in ap.msg_text and len(ap.msg_text) < 400
            shots = os.environ.get("TRYGONS_SHOT_DIR")
            if shots:
                import canvaspic
                rig.pump(0.3)
                canvaspic.save(ap.cv, os.path.join(shots, "screen2_answer_%s.png" % lg))
        i18n.set_lang("el", save=False)
        info = show.demo_answer("el")
        a.last = {"q": info["q"], "lang": "el", "audio_s": 2.0, "ms": 800, "found": info["found"]}
        a.answer = dict(info, speak=True, answer_ms=2100)
        ap.asst_on_answer(info["a"], a.answer)
        ap.render(force=True)
        # [ΞΑΝΑ] -> the voice again; [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] -> the manual window at the passage's lines
        r = ts.op(ap, "asst_again")["r"]
        ap.ov.click((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
        ap.ov.release()
        out["ΞΑΝΑ -> repeat"] = calls == ["repeat"]
        r = ts.op(ap, "asst_manual")["r"]
        ap.ov.click((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
        ap.ov.release()
        rig.pump(0.5)
        w = manualwin.current()
        pl = show.manual_place(a.answer)
        out["manual place from the passage's lines"] = pl is not None and pl[0] == "el" and 0 < pl[1] <= pl[2]
        lines = show.manual_lines("el")
        first = (info["passage"]["text"].split() or [""])[0]
        out["ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ -> MANUAL_EL at the passage"] = w is not None and w.place == pl and \
            any(first in ln for ln in lines[pl[1] - 1:pl[2]])
        txt = w.txt.get("%d.0" % pl[1], "%d.0 lineend" % pl[2]) if w is not None else ""
        out["the passage highlighted + read-only"] = w is not None and first in txt and \
            w.txt.cget("state") == "disabled" and bool(w.txt.tag_ranges("hit"))
        if w is not None:
            w.withdraw()
        # a tour hit has no manual place: the button greyed
        a.answer = dict(a.answer, passage={"text": "x y z w", "cite": "Ξενάγηση: Βοηθός", "id": "tour:el:s1.btn_ask"},
                        id="tour:el:s1.btn_ask", found={"hits": [{"id": "tour:el:s1.btn_ask", "ch": "tour",
                                                                  "lines": [0, 0], "text": "x y z w"}]})
        ap.render(force=True)
        out["tour hit: no manual place"] = show.manual_place(a.answer) is None and \
            ts.op(ap, "asst_manual").get("enabled") is False
        # a LONG pick (AM4-r2 review MED-3): the answer cut on the panel + the message line, the full text in the passage
        long_a = " ".join("λέξη%d" % i for i in range(120)) + "."
        a.answer = dict(info, a=long_a, speak=True, passage=dict(info["passage"], text=long_a + " " + info["passage"]["text"]))
        ap.asst_on_answer(long_a, a.answer)
        ap.render(force=True)
        an = ts.op(ap, "asst_a")
        out["long pick cut on the panel"] = an is not None and len(an["s"]) <= show.PANEL_ANSWER_MAX + 1 and \
            an["s"].endswith("…")
        out["long pick cut on the message line"] = len(ap.msg_text) <= show.LINE_MAX + 60 and "…" in ap.msg_text
        out["long pick: the passage still shows its start"] = "λέξη0" in (ts.op(ap, "asst_passage") or {}).get("s", "")
        out["cut at a word"] = show.cut("alpha beta gamma delta", 12) == "alpha beta…" and \
            show.cut("short", 12) == "short"
        # fold / open the passage
        r = ts.op(ap, "asst_text")["r"]
        ap.ov.click((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
        ap.ov.release()
        ap.render(force=True)
        out["ΚΕΙΜΕΝΟ folds the passage"] = ts.op(ap, "asst_passage") is None and ap.asst_passage_open is False
        ap.asst_toggle_passage()
        out["and opens it again"] = ts.op(ap, "asst_passage") is not None and ap.asst_passage_open is True
        # the fit matrix with the panel + the passage (every panel filled, 3 sizes): the FULL passage whenever it fits,
        # folded by the layout where it does not ([ΚΕΙΜΕΝΟ ▸] opens it) - never a panel that does not fit
        a.answer = dict(info, speak=True)
        ap.asst_passage_open = True
        ap.asst_on_answer(info["a"], a.answer)
        fits = []
        for W, H, dpi in ((1366, 768, 1.25), (1920, 1080, 1.0), (1366, 658, 1.5)):
            ts.fill_everything(rig)
            ap.asst_auto_fold = 0
            rig.size(W, H, dpi)
            probs = ts.layout_problems(ap)
            fits.append(((W, H, dpi), probs[:2], any(p.key == "assistant" for p in ap.ov.panels),
                         "open" if ts.op(ap, "asst_passage") else "fold %d" % ap.asst_auto_fold))
        out["fits with every panel"] = all(not p and drawn for _s, p, drawn, _o in fits)
        out["the full passage open at 1920x1080"] = fits[1][3] == "open"
        # 1366x658 at 150 % with every panel: folded by the layout, [ΚΕΙΜΕΝΟ ▸] shown while the panel is not collapsed
        out["folded = ΚΕΙΜΕΝΟ ▸ offered or header only"] = fits[2][3] == "open" or ap.asst_auto_fold >= 3 or \
            i18n.T("asst.text_open") in [o["s"] for o in ap.ov.ops if o.get("tag") == "asst_text"]
        ap.asst_toggle_passage()                     # the operator opens it anyway: the whole passage (the text shrinks)
        out["ΚΕΙΜΕΝΟ ▸ opens the passage (fold reset)"] = ap.asst_auto_fold == 0 and ap.asst_passage_open and \
            ap.asst_force_open and ts.op(ap, "asst_passage") is not None
        ap.asst_on_answer(info["a"], a.answer)       # a new answer: the layout decides again
        out["a new answer resets the forced open"] = ap.asst_force_open is False
        ts.no_relay_boat(rig)
        rig.size(1366, 768, 1.0)
        # screen 1: the answer 12 s on the notices (cut), the question line too
        ap.asst_on_heard(info["q"], {})
        ap.asst_on_answer(info["a"], a.answer)
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.pump(1.5)
        dv.draw()
        rig.pump(0.3)
        tags = [t for t, _x, _c in dv.notices(ap.t, ap.gw_status or {})]
        out["screen 1 notice: question + answer"] = "asst_q" in tags and "asst_a" in tags
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots and dv.last_frame is not None:
            dv.last_frame.save(os.path.join(shots, "screen1_answer_el.png"))
        ap.asst_shown = (ap.asst_shown[0], time.time() - 13.0)
        tags = [t for t, _x, _c in dv.notices(ap.t, ap.gw_status or {})]
        out["gone after 12 s"] = "asst_a" not in tags
        ap.close_driver()
    finally:
        ap.asst_voice.repeat = orig_rep
        a._set(st0)
        a.last, a.answer = last0, ans0
        i18n.set_lang("el", save=False)
        ap.render(force=True)
    report("AS28 panel + manual window + notices", all(out.values()), "%s; fits %s" % (out, fits if "fits" in dir() else "-"))


def main():
    exe = os.path.join(HERE, "..", "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    for fn in (as1, as2, as3, as5, as7, as9, as10, as12, as13, as14, as15, as16, as17, as18, as19, as20, as21,
               as22, as23, as25, as26, as27, as29) + (() if os.environ.get("AS_SKIP4") else (as4, as24)):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__, False, "crashed: %r" % e)
    rig = ts.Rig()
    try:
        for fn in (as6, as8, as3_rig, as11, as5_keys, as28):
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
