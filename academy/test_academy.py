"""Academy tests (CLAUDE.md section S, docs/ACADEMY_SPEC.md) - step 15: "no boat link" FIRST.

  AC1  no academy module except simguard.py imports a network / serial / process module or
       loads code by name (__import__, importlib, exec, eval)
  AC2  no academy module imports - directly or through other modules - a ground-station /
       gateway / demo module that leads to a network or serial module (link.py, gwclient.py,
       boats.py's prober, gateway/aurix.py ...): the transitive import graph
  AC3  loading every academy module (and its content) opens no connection (socket patched)
       and pulls in no boat-link module
  AC4  simguard.check_target: only the simulator on this PC (loopback + 5760 / 5761); boats/
       IPs, Tailscale, LAN, host names, other ports (5762 = the sim AURIX's gateway UART) refused;
       connect() refuses BEFORE any socket is made
  AC5  simguard.check_welcome_name: only the demo / sim gateway, never a boats/ boat
  AC6  simguard.check_recording: only ordinary recorded files (no COM / /dev / tcp: / folder)
  AC7  on a gateway PC (its config file exists) the academy refuses everything
  AC8  the banner «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» (EN DASH) in el.txt, "TRAINING – SIMULATION" in
       en.txt; every academy module that uses tkinter shows it (banner.banner_text)
  AC9  lang/el.txt + lang/en.txt: same keys, same {names}; every T("key") used exists in both
  AC10 every content file passes content.check (kind, _help, el + en texts, runs_on sim, quiz
       answers); the checker catches the bad cases
  AC11 photo rule: no safety code (app/, hal/, gateway/, groundstation/chart.py,
       tools/chartprep.py) reads docs/daskalio or the academy; the test area is marked
       not_for_navigation + PLACEHOLDER
  AC12 levels: OBSERVER < DRIVER < CAPTAIN, licence = CAPTAIN only, expired = OBSERVER, an
       unknown level = OBSERVER; the gateway push is not in step 15
  AC13 academy/ is excluded from the ADS build in every configuration (check_ads_excludes.py)
  AC14 the PRACTICAL TEST exception (academy/practical/, step 21) is imported by no other
       academy module and has its own test file when it exists

Run:  python academy/test_academy.py      (exit code 0 = all passed)
"""
import ast
import datetime
import importlib
import os
import re
import socket
import string
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
os.environ["TRYGONS_LANG"] = "el"

results = []

# a module from this list = a way out of the PC (network, serial port, other processes)
NET = {"socket", "ssl", "serial", "http", "urllib", "urllib2", "urllib3", "requests", "httpx",
       "websocket", "websockets", "asyncio", "telnetlib", "ftplib", "smtplib", "poplib",
       "imaplib", "nntplib", "paramiko", "xmlrpc", "socketserver", "selectors", "subprocess",
       "multiprocessing", "pty", "aiohttp", "zmq", "paho", "pymodbus", "can", "usb"}
DYNAMIC = {"__import__", "exec", "eval", "compile"}
GUARD = "simguard"                      # the one sanctioned door (its use is checked in AC4-AC7)
SEARCH = [HERE, os.path.join(REPO, "groundstation"), os.path.join(REPO, "gateway"),
          os.path.join(REPO, "gateway", "vision"), os.path.join(REPO, "demo"), os.path.join(REPO, "tools"),
          os.path.join(REPO, "sim"), os.path.join(REPO, "tests", "acceptance")]


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def academy_modules():
    """(module name, path) of every academy .py except the tests (practical/ separately: AC14)."""
    out = []
    for root, ds, fs in os.walk(HERE):
        ds[:] = [d for d in ds if d not in ("__pycache__", "content", "lang", "practical")]
        for f in sorted(fs):
            if f.endswith(".py") and not f.startswith("test_"):
                rel = os.path.relpath(os.path.join(root, f), HERE)
                out.append((rel[:-3].replace(os.sep, "."), os.path.join(root, f)))
    return out


def imports_of(path):
    """(top-level imported names, dynamic-load calls) of a Python file."""
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    names, dyn = set(), set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            for a in n.names:
                names.add(a.name.split(".")[0])
        elif isinstance(n, ast.ImportFrom):
            if n.level == 0 and n.module:
                names.add(n.module.split(".")[0])
        elif isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in DYNAMIC:
            dyn.add(n.func.id)
        elif isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == "os" \
                and n.attr in ("system", "popen", "spawnl", "spawnv", "execv", "execl", "startfile"):
            dyn.add("os." + n.attr)
    if "importlib" in names:
        dyn.add("importlib")
    return names, dyn


def resolve(name, start_dir):
    """The project file a top-level import name means, or None (= standard library / package)."""
    for d in [start_dir] + SEARCH:
        p = os.path.join(d, name + ".py")
        if os.path.isfile(p):
            return p
        p = os.path.join(d, name, "__init__.py")
        if os.path.isfile(p):
            return p
    return None


def reaches_net(path, seen=None, chain=()):
    """The first import chain from this file to a NET module (through project modules), or None."""
    seen = set() if seen is None else seen
    if path in seen:
        return None
    seen.add(path)
    names, _dyn = imports_of(path)
    for nm in sorted(names):
        if nm in NET:
            return chain + (nm,)
        p = resolve(nm, os.path.dirname(path))
        if p and not (os.path.dirname(p) == HERE and nm == GUARD):
            r = reaches_net(p, seen, chain + (nm,))
            if r:
                return r
    return None


# --- AC1 --------------------------------------------------------------------------------------
def ac1():
    bad = []
    mods = academy_modules()
    for name, path in mods:
        names, dyn = imports_of(path)
        if name == GUARD:
            names -= {"socket"}                 # the guard itself may hold the socket (AC4)
        hit = sorted(names & NET)
        if hit or dyn:
            bad.append("%s: %s" % (name, ", ".join(hit + sorted(dyn))))
    report("AC1 no network / serial / process import outside simguard", not bad and len(mods) >= 5,
           "; ".join(bad) or "%d academy modules checked (%s)" % (len(mods), ", ".join(n for n, _ in mods)))


# --- AC2 --------------------------------------------------------------------------------------
def ac2():
    bad = []
    for name, path in academy_modules():
        names, _dyn = imports_of(path)
        for nm in sorted(names):
            p = resolve(nm, HERE)
            if not p or (nm == GUARD and os.path.dirname(p) == HERE):
                continue
            if os.path.dirname(p) != HERE and nm == "practical":
                continue
            r = reaches_net(p, chain=(nm,))
            if r:
                bad.append("%s -> %s" % (name, " -> ".join(r)))
    # the checker itself must catch the boat links (else AC2 proves nothing)
    probes = {}
    for nm in ("link", "gwclient", "boats", "aurix", "gateway"):
        p = resolve(nm, HERE)
        probes[nm] = bool(p and reaches_net(p, chain=(nm,)))
    blind = [k for k, v in probes.items() if not v]
    report("AC2 no academy import leads to a boat link (transitive)", not bad and not blind,
           "; ".join(bad) or ("checker blind to %s" % blind if blind else
                              "no path; the checker sees link / gwclient / boats / aurix / gateway as boat links"))


# --- AC3 --------------------------------------------------------------------------------------
def ac3():
    attempts = []
    orig = (socket.socket.connect, socket.socket.connect_ex, socket.create_connection)

    def no_connect(self, addr, *a):
        attempts.append(addr)
        raise OSError("academy test: no connection allowed")

    def no_create(addr, *a, **k):
        attempts.append(addr)
        raise OSError("academy test: no connection allowed")
    socket.socket.connect = no_connect
    socket.socket.connect_ex = no_connect
    socket.create_connection = no_create
    before = set(sys.modules)
    try:
        import content
        for name, _p in academy_modules():
            importlib.import_module(name)
        for f in content.files():
            content.check(content.load(f), f)
    finally:
        socket.socket.connect, socket.socket.connect_ex, socket.create_connection = orig
    loaded = set(sys.modules) - before
    boatlinks = sorted(loaded & {"link", "gwclient", "boats", "aurix", "gateway", "serial", "shorejoy",
                                 "syncrecv", "app", "stations"})
    report("AC3 loading the academy opens no connection", not attempts and not boatlinks,
           "connects %s, boat-link modules %s" % (attempts, boatlinks) if attempts or boatlinks else
           "%d modules + content loaded, 0 connection attempts, no boat-link module" % len(academy_modules()))


# --- AC4 --------------------------------------------------------------------------------------
def ac4():
    import simguard
    tmp = tempfile.mkdtemp(prefix="trygons_ac4_")
    with open(os.path.join(tmp, "x.json"), "w", encoding="utf-8") as f:
        f.write('{"id": "x1", "name": "X1", "tailscale_ip": "100.101.102.103", "gateway_port": 5761}')
    env = {"PROGRAMDATA": tmp}
    ok_cases = [("127.0.0.1", 5760), ("localhost", 5760), ("127.0.0.1", 5761), ("::1", 5760)]
    bad_cases = [("100.101.102.103", 5761), ("100.64.0.1", 5760), ("100.127.255.254", 5761),
                 ("192.168.1.10", 5760), ("10.0.0.5", 5761), ("0.0.0.0", 5760), ("8.8.8.8", 5760),
                 ("boat.local", 5761), ("er69d", 5761), ("", 5760), (None, 5760),
                 ("127.0.0.1", 5762), ("127.0.0.1", 8554), ("127.0.0.1", 22), ("127.0.0.1", "5760"),
                 ("PLACEHOLDER-100.x.y.z", 5761)]
    wrong = []
    for h, p in ok_cases:
        try:
            simguard.check_target(h, p, boats_dir=tmp, env=env)
        except simguard.Refused as e:
            wrong.append("refused %s:%s (%s)" % (h, p, e))
    for h, p in bad_cases:
        try:
            simguard.check_target(h, p, boats_dir=tmp, env=env)
            wrong.append("allowed %s:%s" % (h, p))
        except simguard.Refused:
            pass
    # the real boats/ files too: every boat's own address is refused
    _n, ips = simguard.boat_files()
    for ip in ips:
        try:
            simguard.check_target(ip, 5761)
            wrong.append("allowed boats/ address %s" % ip)
        except simguard.Refused:
            pass
    # connect() checks BEFORE a socket exists
    made = []
    orig = socket.create_connection
    socket.create_connection = lambda *a, **k: made.append(a)
    try:
        for h, p in (("100.101.102.103", 5761), ("192.168.1.10", 5760)):
            try:
                simguard.connect(h, p, boats_dir=tmp)
                wrong.append("connect allowed %s" % h)
            except simguard.Refused:
                pass
    finally:
        socket.create_connection = orig
    if made:
        wrong.append("a socket was made for a refused target")
    report("AC4 only the simulator on this PC", not wrong,
           "; ".join(wrong) or "%d allowed, %d refused, %d boats/ addresses refused, connect() refuses first"
           % (len(ok_cases), len(bad_cases), len(ips)))


# --- AC5 --------------------------------------------------------------------------------------
def ac5():
    import simguard
    wrong = []
    try:
        simguard.check_welcome_name("trygons-demo")
    except simguard.Refused as e:
        wrong.append("demo refused: %s" % e)
    names, _ = simguard.boat_files()
    for n in sorted(names) + ["ER69D", "rf49d", "", None, "some-boat"]:
        try:
            simguard.check_welcome_name(n)
            wrong.append("allowed %r" % n)
        except simguard.Refused:
            pass
    demo = open(os.path.join(REPO, "demo", "demo.py"), encoding="utf-8").read()
    if '"boat": "trygons-demo"' not in demo:
        wrong.append("demo/demo.py no longer names its gateway trygons-demo (SIM_BOAT_NAMES)")
    report("AC5 a gateway welcome must be the demo / sim, never a boats/ boat", not wrong,
           "; ".join(wrong) or "trygons-demo allowed; %d boats/ names + unknown refused" % len(names))


# --- AC6 --------------------------------------------------------------------------------------
def ac6():
    import simguard
    tmp = tempfile.mkdtemp(prefix="trygons_ac6_")
    rec = os.path.join(tmp, "trip.tlg")
    open(rec, "wb").close()
    env = {"PROGRAMDATA": tmp}
    wrong = []
    try:
        simguard.check_recording(rec, env=env)
    except simguard.Refused as e:
        wrong.append("refused a .tlg: %s" % e)
    for p in ("COM3", "com12:", "\\\\.\\COM3", "/dev/ttyUSB0", "/dev/serial/by-id/x", "tcp:127.0.0.1:5762",
              "usb:0403:6001", tmp, os.path.join(tmp, "missing.tlg"), os.path.join(tmp, "x.exe"), ""):
        try:
            simguard.check_recording(p, env=env)
            wrong.append("allowed %r" % p)
        except simguard.Refused:
            pass
    report("AC6 only ordinary recorded files", not wrong, "; ".join(wrong) or "a .tlg allowed; devices / addresses / folders refused")


# --- AC7 --------------------------------------------------------------------------------------
def ac7():
    import simguard
    tmp = tempfile.mkdtemp(prefix="trygons_ac7_")
    os.makedirs(os.path.join(tmp, "trygons"))
    open(os.path.join(tmp, "trygons", "gateway.json"), "w").close()
    rec = os.path.join(tmp, "trip.tlg")
    open(rec, "wb").close()
    env = {"PROGRAMDATA": tmp}
    wrong = []
    if not simguard.running_on_gateway(env):
        wrong.append("gateway config not seen")
    for fn, args in ((simguard.check_target, ("127.0.0.1", 5760)), (simguard.check_recording, (rec,))):
        try:
            fn(*args, env=env)
            wrong.append("%s allowed on a gateway PC" % fn.__name__)
        except simguard.Refused:
            pass
    if simguard.running_on_gateway({"PROGRAMDATA": tempfile.mkdtemp()}) and not os.path.isfile("/etc/trygons/gateway.json"):
        wrong.append("false gateway detection")
    report("AC7 never on a gateway PC", not wrong, "; ".join(wrong) or "gateway config seen -> sim and files refused")


# --- AC8 --------------------------------------------------------------------------------------
def ac8():
    import text
    import banner
    wrong = []
    if banner.BANNER_EL != "ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ":
        wrong.append("BANNER_EL changed")
    for lc, want in (("el", banner.BANNER_EL), ("en", banner.BANNER_EN)):
        got = text.load(lc).get(banner.BANNER_KEY)
        if got != want:
            wrong.append("%s.txt banner %r != %r" % (lc, got, want))
        if banner.banner_text(lc) != want:
            wrong.append("banner_text(%s)" % lc)
    if banner.banner_text() != banner.BANNER_EL:
        wrong.append("default language is not Greek")
    for name, path in academy_modules():
        names, _ = imports_of(path)
        if names & {"tkinter", "Tkinter"} and "banner_text" not in open(path, encoding="utf-8").read():
            wrong.append("%s draws a screen without the banner" % name)
    report("AC8 banner on every academy screen", not wrong, "; ".join(wrong) or
           "el %r, en %r; no screen without it" % (banner.BANNER_EL, banner.BANNER_EN))


# --- AC9 --------------------------------------------------------------------------------------
def ac9():
    import text
    el, en = text.load("el"), text.load("en")
    wrong = []
    if set(el) != set(en):
        wrong.append("keys only in el: %s, only in en: %s" % (sorted(set(el) - set(en)), sorted(set(en) - set(el))))
    fmt = string.Formatter()
    for k in set(el) & set(en):
        a = {f for _, f, _, _ in fmt.parse(el[k]) if f}
        b = {f for _, f, _, _ in fmt.parse(en[k]) if f}
        if a != b:
            wrong.append("%s: {names} differ" % k)
    used = set()
    for _n, path in academy_modules():
        used |= set(re.findall(r"\bT\(\s*[\"']([A-Za-z0-9_.\-]+)[\"']", open(path, encoding="utf-8").read()))
    import accounts
    used |= {"level." + lv for lv in accounts.LEVELS}
    missing = sorted(k for k in used if k not in el or k not in en)
    if missing:
        wrong.append("used but missing: %s" % missing)
    report("AC9 el / en language files", not wrong, "; ".join(wrong) or "%d keys, same {names}, %d used keys present" % (len(el), len(used)))


# --- AC10 -------------------------------------------------------------------------------------
def ac10():
    import content
    probs = []
    fs = content.files()
    kinds = set()
    for f in fs:
        d = content.load(f)
        kinds.add(d.get("kind"))
        probs += content.check(d, os.path.relpath(f, HERE))
    need = {"lesson", "quiz", "scenario", "exam", "limits", "test_area"}
    if kinds != need:
        probs.append("example kinds %s, expected %s" % (sorted(k for k in kinds if k), sorted(need)))
    bad_docs = [
        {"kind": "lesson", "_help": "x", "runs_on": "boat", "title_el": "a", "title_en": "b"},
        {"kind": "lesson", "_help": "x", "runs_on": "sim", "title_el": "a"},
        {"kind": "quiz", "_help": "x", "questions": [{"q_el": "a", "q_en": "b", "choices": [{"text_el": "1", "text_en": "1"}], "answer": 3}]},
        {"kind": "nonsense", "_help": "x"},
        {"kind": "exam", "level": "ADMIRAL", "_help": "x"},
        {"kind": "limits"},
        {"kind": "test_area", "_help": "x", "placeholder": True},
    ]
    missed = [i for i, d in enumerate(bad_docs) if not content.check(d, "bad%d" % i)]
    if missed:
        probs.append("checker missed bad case(s) %s" % missed)
    report("AC10 content files", not probs, "; ".join(probs) or "%d files OK (%s); %d bad cases caught" % (len(fs), ", ".join(sorted(kinds)), len(bad_docs)))


# --- AC11 -------------------------------------------------------------------------------------
def ac11():
    import content
    wrong = []
    safety = []
    for sub, ext in (("app", (".c", ".h")), ("hal", (".c", ".h")), ("gateway", (".py",))):
        for root, ds, fs in os.walk(os.path.join(REPO, sub)):
            ds[:] = [d for d in ds if d not in ("__pycache__", "libcanard")]
            safety += [os.path.join(root, f) for f in fs if f.endswith(ext) and not f.startswith("test_")]
    safety += [os.path.join(REPO, "groundstation", "chart.py"), os.path.join(REPO, "tools", "chartprep.py")]
    for p in safety:
        s = open(p, encoding="utf-8", errors="replace").read().lower()
        if "daskalio" in s or re.search(r"^\s*(import|from)\s+academy|academy[/\\]content", s, re.M):
            wrong.append(os.path.relpath(p, REPO))
    area = content.load(os.path.join(content.CONTENT_DIR, "test_area.json"))
    if area.get("not_for_navigation") is not True or area.get("placeholder") is not True:
        wrong.append("test_area.json not marked not_for_navigation + PLACEHOLDER")
    for ph in area.get("photos", []):
        if not os.path.isfile(os.path.join(REPO, ph)) or not ph.startswith("docs/daskalio/"):
            wrong.append("photo %s missing" % ph)
    readme = open(os.path.join(REPO, "docs", "daskalio", "README.txt"), encoding="utf-8").read()
    if "NOT A NAVIGATION CHART" not in readme:
        wrong.append("docs/daskalio/README.txt lost the rule")
    report("AC11 chart photos never in a safety check", not wrong, "; ".join(wrong) or
           "%d safety files never read docs/daskalio or the academy; test area PLACEHOLDER + not_for_navigation" % len(safety))


# --- AC12 -------------------------------------------------------------------------------------
def ac12():
    import accounts
    wrong = []
    if not (accounts.rank("OBSERVER") < accounts.rank("DRIVER") < accounts.rank("CAPTAIN")):
        wrong.append("order")
    if [lv for lv in accounts.LEVELS if accounts.licence(lv)] != ["CAPTAIN"]:
        wrong.append("licence not CAPTAIN only")
    if accounts.rank("ADMIRAL") != 0 or accounts.Account("u", "ADMIRAL").level != "OBSERVER":
        wrong.append("unknown level gives rights")
    d = datetime.date(2026, 10, 28)
    a = accounts.Account("u", "CAPTAIN", expiry=d)
    if a.effective_level(d) != "CAPTAIN" or a.effective_level(d + datetime.timedelta(days=1)) != "OBSERVER":
        wrong.append("expiry")
    if accounts.Account("u", "DRIVER").effective_level(d) != "OBSERVER":
        wrong.append("no expiry = rights")
    r = a.gateway_record(d + datetime.timedelta(days=1))
    if r["licence"] or r["level"] != "OBSERVER":
        wrong.append("expired captain pushed with the licence")
    try:
        accounts.push_to_gateway([a])
        wrong.append("push exists in step 15")
    except NotImplementedError:
        pass
    report("AC12 level model", not wrong, "; ".join(wrong) or "OBSERVER < DRIVER < CAPTAIN, licence = CAPTAIN, expired / unknown = OBSERVER")


# --- AC13 -------------------------------------------------------------------------------------
def ac13():
    import xml.etree.ElementTree as ET
    root = ET.parse(os.path.join(REPO, ".cproject")).getroot()
    n, miss = 0, []
    for cfg in root.iter("configuration"):
        if cfg.get("artifactName") is None:
            continue
        for e in cfg.iter("entry"):
            if e.get("kind") == "sourcePath" and e.get("name", "") == "":
                n += 1
                if "academy" not in e.get("excluding", "").split("|"):
                    miss.append(cfg.get("name"))
    rc = subprocess.run([sys.executable, os.path.join(REPO, "tests", "acceptance", "check_ads_excludes.py")],
                        capture_output=True, text=True).returncode
    report("AC13 academy/ excluded from the ADS build", n >= 1 and not miss and rc == 0,
           "not excluded in %s, check_ads_excludes rc %d" % (miss, rc) if miss or rc else "%d configurations exclude academy; check_ads_excludes OK" % n)


# --- AC14 -------------------------------------------------------------------------------------
def ac14():
    wrong = []
    for name, path in academy_modules():
        names, _ = imports_of(path)
        src = open(path, encoding="utf-8").read()
        if "practical" in names or re.search(r"^\s*from\s+\.?practical|^\s*import\s+practical", src, re.M):
            wrong.append("%s imports the practical test" % name)
    pdir = os.path.join(HERE, "practical")
    exists = os.path.isdir(pdir)
    if exists and not os.path.isfile(os.path.join(pdir, "test_practical.py")):
        wrong.append("academy/practical/ has no test_practical.py (it must prove: read-only, CONFIRM_ONLY only)")
    report("AC14 practical test kept apart", not wrong, "; ".join(wrong) or
           ("academy/practical/ isolated" if exists else "academy/practical/ not created yet (step 21); nothing imports it"))


if __name__ == "__main__":
    for t in (ac1, ac2, ac3, ac4, ac5, ac6, ac7, ac8, ac9, ac10, ac11, ac12, ac13, ac14):
        try:
            t()
        except Exception as e:                                  # a crash = a failure, the others still run
            report(t.__name__.upper(), False, "crashed: %r" % e)
    ok = all(results)
    print("ACADEMY TESTS: %d/%d PASS" % (sum(results), len(results)))
    sys.exit(0 if ok else 1)
