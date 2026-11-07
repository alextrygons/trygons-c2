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

Run:  python test_assistant.py      (exit code 0 = all passed; N/A lines do not fail)
"""
import ast
import array
import json
import math
import os
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
    report("AS3 runtime: senders + network blocked", ready and q == "πού κλείνω την καταπακτή" and not sent and
           len(MARKERS) > n and not tried and nc_ok,
           "ready %s, heard %r, senders / sockets called %s, traps %d; worker network attempts %s; the blocker's "
           "negative control (attempts caught by the script) recorded %s" % (ready, q, sent or "none", len(saved),
                                                                             tried or "none", nc))


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


def as3_rig(rig):
    """AM2 review LOW-2: the same traps through the REAL station wiring - ap.asst (the App's own Assistant with
    its AsstView), the screen-2 «ΕΡΩΤΗΣΗ» button, App.ov_tick draining the heard queue: no sender / socket is
    called from an assistant code path, no network attempt in the worker, the question reaches the screens."""
    import socket
    import app as A
    import gwclient
    import link
    ap = rig.ap
    a = ap.asst
    sent, saved = [], []

    def trap(cls, name):
        orig = getattr(cls, name)
        saved.append((cls, name, orig))

        def t(*x, **k):
            if from_assistant():
                sent.append("%s.%s" % (cls.__name__, name))
                raise RuntimeError("a sender was called from the assistant")
            return orig(*x, **k)
        setattr(cls, name, t)
    for cls in (gwclient.GatewayLink, link.Link, A.App):
        for name in list(vars(cls)):
            if name.startswith("send") or name == "cmd":
                trap(cls, name)
    real_sock, real_cc = socket.socket, socket.create_connection

    class NoSock(real_sock):
        def __init__(self, *x, **k):
            if from_assistant():
                sent.append("socket")
                raise OSError("network blocked")
            super().__init__(*x, **k)

    def cc(*x, **k):
        if from_assistant():
            sent.append("socket")
            raise OSError("network blocked")
        return real_cc(*x, **k)
    socket.socket, socket.create_connection = NoSock, cc
    keep = a.stt_kind, a.stub_text, a.mic_backend
    n = len(MARKERS)
    out = {}
    try:
        a.stop()
        a.stt_kind, a.stub_text, a.mic_backend = "stub", "πού κλείνω την καταπακτή", "sim"
        a.worker_cmd = lambda: blocked_cmd(sys.executable)
        ap.asst_heard = None
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
    finally:
        a.stop()
        del a.worker_cmd                              # back to the class's own worker_cmd
        a.stt_kind, a.stub_text, a.mic_backend = keep
        socket.socket, socket.create_connection = real_sock, real_cc
        for cls, name, orig in saved:
            setattr(cls, name, orig)
    tried = attempts(MARKERS[n:])
    out["no sender / socket from the assistant"] = not sent
    out["no network attempt in the worker"] = len(MARKERS) > n and not tried
    report("AS3b the real station wiring (ap.asst) with the AS3 traps", all(out.values()),
           "%s; senders %s, worker attempts %s, traps %d" % (out, sent or "none", tried or "none", len(saved)))


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


def as4():
    py = paths.assistant_python()
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
    got = []
    try:
        a.start()
        ready = wait(a, lambda: a.state == client.READY, 120)
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
    finally:
        a.stop()
    tried = attempts(MARKERS[n0:])
    nc_ok, nc = blocker_selftest(py)                  # the negative control in the ASSISTANT Python too
    report("AS4 real models, network blocked", ready and len(got) == 2 and all(g[3] for g in got) and
           len(MARKERS) > n0 and not tried and nc_ok,
           "device %s, %s; worker network attempts %s (a caught attempt counts too); negative control %s"
           % (a.device, got, tried or "none", nc))


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
    ap.hail.pressing = True
    out["AsstView.horn_held"] = view.horn_held() is True
    ap.hail.pressing = False
    out["AsstView no sender"] = not any(hasattr(view, n) for n in ("app", "send", "cmd", "link"))
    # AM2 review LOW-1: the view holds NO App - a gc.get_referents walk from it (the loudhailer object is the one
    # allowed station object: it is NOT walked into) never meets the App, a bound App method or the link
    import app as A
    import gc
    found, seen, todo = [], set(), [view]
    while todo and len(seen) < 20000:
        o = todo.pop()
        if id(o) in seen or isinstance(o, (type, types.ModuleType)):
            continue
        seen.add(id(o))
        if isinstance(o, A.App) or getattr(o, "__self__", None) is ap or o is ap.link:
            found.append(type(o).__name__)
            continue
        if o is ap.hail:
            continue
        todo += gc.get_referents(o)
    out["AsstView holds no App (gc walk)"] = not found and len(seen) > 1
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
    report("AS6 loudhailer wins", all(out.values()), "%s; objects walked %d, App found %s" % (out, len(seen), found))


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
            if shots:
                for lg in ("el", "en"):
                    i18n.set_lang(lg, save=False)
                    pg.refresh()
                    ts.shot(ap.tools, os.path.join(shots, "tools_assistant_%s.png" % lg))
                i18n.set_lang("el", save=False)
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
    import importlib.util
    import re
    spec = importlib.util.spec_from_file_location("get_whisper", os.path.join(REPO, "tools", "get_whisper.py"))
    gw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gw)
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
    report("AS13 hash-locked downloads", all(out.values()), "%s; %d packages" % (out, len(pinned)))


def main():
    exe = os.path.join(HERE, "..", "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    for fn in (as1, as2, as3, as5, as7, as9, as10, as12, as13) + (() if os.environ.get("AS_SKIP4") else (as4,)):
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
