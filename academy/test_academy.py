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
       unknown level = OBSERVER; the push document is signed
  AC13 academy/ is excluded from the ADS build in every configuration (check_ads_excludes.py)
  AC14 the PRACTICAL TEST exception (academy/practical/, step 21) is imported by no other
       academy module and has its own test file when it exists
  AC15 accounts storage (step 16): one file per account, append-only logbook (a cut line is
       skipped), default expiry from limits.json, reload, refused on a gateway PC
  AC16 instructor view: progress, weak areas, hours, currency, expiry reminder
  AC17 the gateway push = a SIGNED file the gateway accepts (gateway/trainlevels.py): no key = no
       file; expired / unknown = OBSERVER without licence; a changed byte = refused
  AC18 backup (W28): a copy to a chosen folder without the key; never inside the data folder, a
       boats/ address, the Tailscale range or on a gateway PC
  AC19 the academy window: the banner is the first thing at the top (exact text, CAUTION colour)
  AC20 the ground-station menu "Academy": its own process with only --lang (never a link / host /
       port), the levels file only over a gateway login, the gateway's answer shown

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
    import academy_app
    used |= {"acc.col_" + c for c in academy_app.COLUMNS}
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
    doc = accounts.push_to_gateway([a], "k" * 32, d)
    if not doc.get("mac") or doc["records"][0] != {"user": "u", "level": "CAPTAIN", "licence": True,
                                                    "expiry": d.isoformat()}:
        wrong.append("push document")
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


def _store(tmp=None):
    import accounts
    return accounts.Store(tmp or tempfile.mkdtemp(prefix="trygons_acst_"), env={"PROGRAMDATA": tempfile.mkdtemp()})


def json_load(p):
    import json
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def gateway_pc():
    """A PROGRAMDATA folder that looks like a gateway PC (its gateway.json exists)."""
    d = tempfile.mkdtemp(prefix="trygons_gwpc_")
    os.makedirs(os.path.join(d, "trygons"))
    open(os.path.join(d, "trygons", "gateway.json"), "w").close()
    return {"PROGRAMDATA": d}


# --- AC15 -------------------------------------------------------------------------------------
def ac15():
    import accounts
    import simguard
    wrong = []
    tmp = tempfile.mkdtemp(prefix="trygons_ac15_")
    st = _store(tmp)
    st.add("anna", "Anna P.")
    st.add("nik", "Nikos")
    for bad in ("anna", "../evil", ""):
        try:
            st.add(bad)
            wrong.append("account %r taken" % bad)
        except ValueError:
            pass
    d = datetime.date(2026, 10, 28)
    a = st.set_level("anna", "CAPTAIN", by="instr", today=d)
    months = accounts.limits().get("validity_months")
    if not months or not (a.expiry - d).days >= 28 * months:
        wrong.append("default expiry %s (validity %s months)" % (a.expiry, months))
    st.log("anna", "hours", h=2.5)
    with open(st.logbook_path, "a", encoding="utf-8") as f:
        f.write('{"user": "anna", "kind": "hou')          # a power loss mid-line
    st2 = _store(tmp)
    if sorted(st2.accounts) != ["anna", "nik"] or st2.accounts["anna"].level != "CAPTAIN" or \
            st2.accounts["anna"].expiry != a.expiry:
        wrong.append("reload %s" % sorted(st2.accounts))
    kinds = [e["kind"] for e in st2.accounts["anna"].logbook]
    if kinds != ["level", "level", "hours"]:
        wrong.append("logbook %s" % kinds)
    if not os.path.isfile(os.path.join(tmp, "accounts", "anna.json")):
        wrong.append("one file per account")
    try:
        st.log("anna", "nonsense")
        wrong.append("unknown logbook kind taken")
    except ValueError:
        pass
    try:
        accounts.Store(tmp, env=gateway_pc())
        wrong.append("store works on a gateway PC")
    except simguard.Refused:
        pass
    if "academy/data/" not in open(os.path.join(REPO, ".gitignore"), encoding="utf-8").read():
        wrong.append("academy/data/ not gitignored")
    report("AC15 accounts storage", not wrong, "; ".join(wrong) or
           "one file per account, append-only logbook (cut line skipped), expiry %s from limits.json, refused on a "
           "gateway PC" % a.expiry)


# --- AC16 -------------------------------------------------------------------------------------
def ac16():
    wrong = []
    st = _store()
    d = datetime.date.today()
    st.add("anna")
    st.add("nik")
    st.set_level("anna", "DRIVER", d + datetime.timedelta(days=10))
    st.log("anna", "lesson", id="L1", done=True)
    st.log("anna", "lesson", id="L2", done=True)
    st.log("anna", "quiz", topic="colregs", score=50)
    st.log("anna", "quiz", topic="colregs", score=60)
    st.log("anna", "quiz", topic="gear", score=95)
    st.log("anna", "exam", id="E1", score=80, passed=True)
    st.log("anna", "attempt", id="E1")
    st.log("anna", "hours", h=1.5)
    st.log("anna", "hours", h=2.0)
    rows = {r["user"]: r for r in st.summary(d)}
    r = rows["anna"]
    got = (r["lessons"], r["exams_passed"], r["attempts"], r["hours"], r["weak"], r["current"], r["days_left"],
           r["reminder"], r["effective"])
    if got != (2, 1, 1, 3.5, ["colregs"], True, 10, True, "DRIVER"):
        wrong.append("anna %s" % (got,))
    n = rows["nik"]
    if (n["effective"], n["current"], n["hours"], n["days_left"]) != ("OBSERVER", False, 0, None):
        wrong.append("nik %s" % n)
    report("AC16 instructor view", not wrong, "; ".join(wrong) or
           "lessons / exams / attempts / hours / weak areas / currency / expiry reminder per trainee")


# --- AC17 -------------------------------------------------------------------------------------
def ac17():
    import accounts
    sys.path.insert(0, os.path.join(REPO, "gateway"))
    import trainlevels
    wrong = []
    st = _store()
    d = datetime.date.today()
    for u in ("anna", "nik", "old", "drv"):
        st.add(u)
    st.set_level("anna", "CAPTAIN", d + datetime.timedelta(days=30))
    st.set_level("old", "CAPTAIN", d + datetime.timedelta(days=30))
    st.accounts["old"].expiry = d - datetime.timedelta(days=1)          # expired since
    st.set_level("drv", "DRIVER", d + datetime.timedelta(days=30))
    try:
        st.make_push()
        wrong.append("a push file without a key")
    except ValueError:
        pass
    key = st.key(create=True)
    if st.key() != key or len(key) < 16:
        wrong.append("key not kept")
    path, _doc = st.make_push()
    on_disk = json_load(path)
    recs, why = trainlevels.verify(on_disk, key, 0)
    if recs is None:
        wrong.append("the gateway refuses the academy's file: %s" % why)
    else:
        want = {"anna": (True, "CAPTAIN"), "nik": (False, "OBSERVER"), "old": (False, "OBSERVER"),
                "drv": (False, "DRIVER")}
        got = {u: (r["licence"], r["level"]) for u, r in recs.items()}
        if got != want:
            wrong.append("records %s" % got)
    bad = dict(on_disk, records=[dict(x, level="CAPTAIN", licence=True,
                                      expiry=(d + datetime.timedelta(days=9)).isoformat())
                                 for x in on_disk["records"]])
    if trainlevels.verify(bad, key, 0)[0] is not None:
        wrong.append("a changed file accepted")
    if trainlevels.verify(on_disk, "x" * 32, 0)[0] is not None:
        wrong.append("another key accepted")
    if accounts.canonical(on_disk) != trainlevels.canonical(on_disk):
        wrong.append("the two sides sign different bytes")
    report("AC17 the gateway push = a signed file the gateway accepts", not wrong, "; ".join(wrong) or
           "no key = no file; the gateway's checker accepts it; expired / unknown = OBSERVER; a changed file refused")


# --- AC18 -------------------------------------------------------------------------------------
def ac18():
    import accounts
    import simguard
    wrong = []
    tmp = tempfile.mkdtemp(prefix="trygons_ac18_")
    st = _store(tmp)
    st.add("anna")
    st.key(create=True)
    st.make_push()
    target = tempfile.mkdtemp(prefix="trygons_ac18t_")
    env = {"PROGRAMDATA": tempfile.mkdtemp()}
    dst = st.backup(target, env=env, stamp="x")
    have = sorted(os.listdir(dst))
    if have != ["accounts", "levels.json", "logbook.jsonl"] or \
            not os.path.isfile(os.path.join(dst, "accounts", "anna.json")):
        wrong.append("backup content %s" % have)
    boats = tempfile.mkdtemp(prefix="trygons_ac18b_")
    with open(os.path.join(boats, "x.json"), "w", encoding="utf-8") as f:
        f.write('{"id": "x1", "name": "X1", "tailscale_ip": "100.101.102.103", "gateway_port": 5761}')
    os.makedirs(os.path.join(tmp, "inside"))
    for t in (os.path.join(tmp, "inside"), tmp, os.path.join(target, "missing"), "",
              r"\\100.101.102.103\share", r"\\100.70.1.2\backup", r"\\x1\share", "//100.101.102.103/share"):
        if accounts.backup_refused(t, tmp, env, boats) is None:
            wrong.append("allowed %r" % t)
    try:
        st.backup(target, env=gateway_pc(), stamp="y")
        wrong.append("backup on a gateway PC")
    except simguard.Refused:
        pass
    report("AC18 backup", not wrong, "; ".join(wrong) or
           "accounts + logbook + push file copied (never the key); data folder, boats / Tailscale address, gateway PC "
           "refused")


# --- AC19 -------------------------------------------------------------------------------------
def ac19():
    import banner
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as e:                                  # no display: the static AC8 check still holds
        report("AC19 academy window banner", True, "no display here (%s): checked statically by AC8" % e)
        return
    wrong = []
    try:
        root.withdraw()
        import academy_app
        app = academy_app.AcademyApp(root, _store())
        first = root.pack_slaves()[0]
        if first is not app.banner or app.banner.cget("text") != banner.banner_text():
            wrong.append("the banner is not the first widget at the top")
        if app.banner.cget("bg").lower() != academy_app.CAUTION.lower():
            wrong.append("banner colour %s" % app.banner.cget("bg"))
        if str(app.banner.pack_info().get("side")) != "top":
            wrong.append("banner not at the top")
    finally:
        root.destroy()
    report("AC19 academy window banner", not wrong, "; ".join(wrong) or
           "first widget at the top, %r, CAUTION colour" % banner.banner_text())


# --- AC20 -------------------------------------------------------------------------------------
def ac20():
    wrong = []
    gs = os.path.join(REPO, "groundstation")
    os.environ.setdefault("TRYGONS_GS_SETTINGS", os.path.join(tempfile.mkdtemp(), "gs_settings.json"))
    os.environ.setdefault("TRYGONS_SOUND", "off")
    sys.path.insert(0, gs)
    import academymenu
    cmd = academymenu.academy_command()
    if cmd[1:] != [os.path.join(REPO, "academy", "academy_app.py"), "--lang", cmd[-1]] or cmd[-1] not in ("el", "en"):
        wrong.append("academy command %s" % cmd)
    names, _ = imports_of(os.path.join(gs, "academymenu.py"))
    risky = names & {"link", "boats", "socket", "serial", "shorejoy", "app"}
    if risky:
        wrong.append("academymenu imports %s" % sorted(risky))
    src = open(os.path.join(gs, "academymenu.py"), encoding="utf-8").read()
    opener = src.split("def open_academy")[1].split("def send_levels")[0]
    if "link" in opener or "academy_command()" not in opener:
        wrong.append("open_academy hands the link over")
    app_src = open(os.path.join(gs, "app.py"), encoding="utf-8").read()
    if "academymenu.add_menu(self, mb)" not in app_src or '"academy_levels_ack"' not in app_src:
        wrong.append("app.py: menu / answer not wired")

    class FakeApp(object):
        def __init__(self, link):
            self.link, self.said, self.events, self.root = link, [], [], None

        def say(self, t, c=None):
            self.said.append((t, c))

        def event(self, t):
            self.events.append(t)
    started = []
    academymenu.open_academy(FakeApp(None), popen=lambda c, cwd=None: started.append(c) or "proc")
    if started != [cmd]:
        wrong.append("open_academy started %s" % started)
    sent = []
    from gwclient import GatewayLink

    class FakeGw(GatewayLink):
        def __init__(self):
            pass

        def send_msg(self, m):
            sent.append(m)
    tmp = tempfile.mkdtemp(prefix="trygons_ac20_")
    good = os.path.join(tmp, "levels.json")
    with open(good, "w", encoding="utf-8") as f:
        f.write('{"kind": "academy_levels", "records": [], "seq": 1, "format": 1, "mac": "x"}')
    other = os.path.join(tmp, "other.json")
    with open(other, "w", encoding="utf-8") as f:
        f.write('{"kind": "lesson"}')
    usb = FakeApp(object())
    academymenu.send_levels(usb, good)                       # USB / no gateway: refused, nothing sent
    gw = FakeApp(FakeGw())
    academymenu.send_levels(gw, other)                       # not a levels file: refused
    academymenu.send_levels(gw, good)
    if not usb.said or sent != [{"t": "academy_levels", "doc": json_load(good)}]:
        wrong.append("send: usb %s, sent %s" % (usb.said, sent))
    import i18n
    i18n.set_lang("el", save=False)
    academymenu.on_ack(gw, {"ok": False, "text": "academy levels refused: wrong signature (not made with this "
                                                 "gateway's academy key)"})
    if not gw.said or "academy" in gw.said[-1][0].lower() or "signature" in gw.said[-1][0]:
        wrong.append("the answer not translated: %s" % (gw.said[-1:],))
    report("AC20 ground-station menu Academy", not wrong, "; ".join(wrong) or
           "own process with only --lang; the levels file only over a gateway login; the answer shown in Greek")


if __name__ == "__main__":
    for t in (ac1, ac2, ac3, ac4, ac5, ac6, ac7, ac8, ac9, ac10, ac11, ac12, ac13, ac14, ac15, ac16, ac17, ac18,
              ac19, ac20):
        try:
            t()
        except Exception as e:                                  # a crash = a failure, the others still run
            report(t.__name__.upper(), False, "crashed: %r" % e)
    ok = all(results)
    print("ACADEMY TESTS: %d/%d PASS" % (sum(results), len(results)))
    sys.exit(0 if ok else 1)
