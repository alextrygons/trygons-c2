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
  AC21 guided tour content (step 17, spec B): content/tour.json passes content.check + tour.check;
       the tour checker catches the bad cases (double id, unknown show / target / item, no text)
  AC22 tour coverage: every HUD box + button, every screen-2 panel, every tools page, every mode,
       every alarm (both words + NO DATA), the MASTER lamps, the joystick and every panel button has
       a stop (the list is read from the ground station's own sources); both languages complete
  AC23 tour voice: the same file names as the ground station's TTS cache (sound.Library); the
       station's menu "Guided tour" = its own process with only --lang + the voice made in the
       background with exactly the tour's sentences (the academy runs no TTS / process itself)
  AC24 the tour window: banner first, next / back / chapter jump through every stop in el + en,
       the voice played only from an existing file through simguard (none = text only, off = quiet,
       on a gateway PC = refused); the academy window's "Guided tour" button opens it
  AC25 lessons (step 18, spec C): every lesson file passes content.check + lesson.check_doc; the
       checker catches the bad cases; the model (ready, checks, delta with wrap, hold, hint, time
       limit, retry, score, logbook entry); grouped by level; required lessons before the exam; a
       lesson sends only the simulator's / station's own frames (never a gateway frame)
  AC26 simguard step 18: SimLink = port 5760 of this PC only (refused before any socket);
       start_simulator = exactly this repo's sim + "server 5760 1" in academy/data/sim, refused on a
       gateway PC / without the built simulator; no other academy module starts a process
  AC27 quizzes (spec D): every pool passes content.check + quiz.check_doc (bad cases caught); choice,
       picture (every show) and sound (every alarm level) questions present; random order + shuffled
       choices; picture clicks inside / outside; scores per pool for the logbook; a sound question
       without its file is left out; the station writes exactly the tones the quiz plays
  AC28 every lesson passes on the REAL simulator by doing what its steps say (spec C: HDG LOCK,
       course +10, takeover, hand-back, power raise with CONFIRM, alarm reaction); wrong actions do
       not pass a step; a missed reaction time fails and the retry passes
  AC29 the lesson + quiz windows: banner first, lessons grouped by level, a lesson run through the
       window on the simulator lands in the logbook (tick in the list), no simulator = said so; the
       quiz walks every question (choice / picture click / sound played via simguard), the score
       lands in the logbook per pool; the academy window's buttons open both
  AC30 emergency scenarios (step 19, spec E): every scenario file passes content.check + scenario.check_doc
       (bad cases caught); every spec E item has a scenario; the model on a made-up clock (reaction time,
       full marks / linear score / too late, forbidden action, outcome distance, retry 90, wait_for,
       hazard geometry); the replay file read back only through simguard; the fault frame
       PROTO_T_SIM_FAULT = the same number in app/proto.h + proto.py, handled only by sim_server.c
  AC31 every scenario passes on the REAL simulator by its right reaction; the simulated AURIX shows its own
       alarms / mode changes for the simulator faults (COMPASS LOST -> STANDBY only at idle, GPS SUSPECT,
       SLAMMING, JOYSTICK FAULT -> STANDBY, GPS LOST); no reaction / a run-over swimmer / a port turn at a
       starboard ship fail; the retry scores 90
  AC32 the power-limit refusal on the real simulator: refused with an alarm, CONFIRM alone raises nothing,
       the proper raise after the fix works (the scenario adds no way around a rule)
  AC33 the scenario window: banner first, by level, no simulator said, the academy's picture (AIS on the map
       + list, swimmer only on the camera, gateway lost = no AIS / camera), a scenario through the window
       on the simulator -> logbook (reaction time, replay file) + debrief; the replay window via simguard

  AC34 real-trip video lessons (step 20, spec F): the lesson checker catches the bad cases (clip length, points
       outside / out of order, bad areas / blur boxes, unknown camera, a media path out of the lesson folder,
       runs_on, a missing language); the player model (pause points, decision + spot-the-hazard answers, one
       answer per point, explanation, score, logbook entry); the HUD timeline synced to UTC from the log's TIME
       events; the editor model (trim limits, cameras, times event -> clip)
  AC35 simguard step 20: ffmpeg only on recorded files ("-protocol_whitelist file" before every input), copies
       only as .mp4 inside the video-lesson folder, URLs / COM / folders / other types refused, nothing on a
       gateway PC, the concat list holds only checked files and is removed; the player opens only the
       lesson's own copies (never the event's originals)
  AC36 the lesson editor on REAL ffmpeg: a saved event (2 cameras, 1-minute .ts files) + its log -> the small
       trimmed copies (length = the clip, smaller than the originals, the blur box blurred, the rest not) + the
       HUD timeline in sync; the lesson passes check_doc with its files
  AC37 the video lesson windows: banner first in the player + the editor; the editor opens an event, trims, adds
       a decision point + a hazard area + a blur box and makes the lesson; the player lists it, plays it with the
       HUD redrawn over the picture, stops at the points, takes the answers (choice / click) and logs
       kind "videolesson"; a COM port as event refused; the academy window's buttons open both
  AC38 exams per level (step 21, spec G): the DRIVER + CAPTAIN exam files pass the checker (bad cases caught); who may
       sit, the required lessons first, 3 attempts then a 24 h cool-down (W27), a pass resets, online only in a
       supervised session; an attempt logged at the start + the result; passing never changes a level; the exam
       window (banner, refusal said, no answers shown during the exam, score logged); the academy window's button
  AC39 logins + 2FA (spec J, W29): RFC 6238 TOTP vectors, PBKDF2 passwords, CAPTAIN + instructor need 2FA, a trainee
       can't examine, lock-out, old / foreign sessions refused, the secrets never in a backup
  AC40 the online academy (spec J): online detection, the Tailscale policy checker (trainee devices reach only the
       academy host, never a boat), nothing online upgrades, supervision only by an instructor's 2FA login
  AC41 renewal (DRIVER refresher, CAPTAIN check ride, expired > 3 months = full test), currency (no driving in 90 days
       = no licence pushed until a refresher trip), set_level only lowers once a CAPTAIN instructor exists
  AC42 the instructor view's certificate PDF (Pillow): an instructor's 2FA session only, the level in force, logged
  AC43 the academy window's step-21 buttons (login, password, 2FA app, renew, supervise, certificate)
Run:  python academy/test_academy.py      (exit code 0 = all passed)
"""
import ast
import datetime
import importlib
import json
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
            names -= {"socket", "subprocess"}   # the guard itself: the socket (AC4) + the simulator start (AC26)
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
    import tour_app
    used |= {"tour.s2_" + n for n in tour_app.S2_BOXES}
    import videolesson_app
    used |= videolesson_app.TEXT_KEYS
    import scenario_app
    used |= {"scen.db_failed_" + w for w in ("late", "forbid", "outcome")} | {"scen.ev_" + e for e in scenario_app.EVENT_KEYS}
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
    need = {"lesson", "quiz", "scenario", "exam", "limits", "test_area", "tour", "practical"}
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


# --- AC21 -------------------------------------------------------------------------------------
def ac21():
    import copy
    import content
    import tour
    doc = tour.load()
    wrong = content.check(doc, "tour.json") + tour.check(doc)
    st0 = doc["chapters"][1]["stops"][1]
    bad = []
    d = copy.deepcopy(doc)
    d["chapters"][1]["stops"].append(copy.deepcopy(st0))                         # double id
    bad.append(d)
    d = copy.deepcopy(doc)
    d["chapters"][1]["stops"][1]["show"] = "boat"                                # unknown show
    bad.append(d)
    d = copy.deepcopy(doc)
    d["chapters"][1]["stops"][1]["target"] = "nonsense"                          # unknown target
    bad.append(d)
    d = copy.deepcopy(doc)
    d["chapters"][1]["stops"][1]["covers"] = ["hud:nonsense"]                    # unknown item
    bad.append(d)
    d = copy.deepcopy(doc)
    d["chapters"][1]["stops"][1]["text_en"] = " "                                # no English text
    bad.append(d)
    d = copy.deepcopy(doc)
    d["kind"] = "lesson"
    bad.append(d)
    missed = [i for i, b in enumerate(bad) if not tour.check(b)]
    if missed:
        wrong.append("tour checker missed bad case(s) %s" % missed)
    n = sum(len(c["stops"]) for c in doc["chapters"])
    report("AC21 guided tour content", not wrong, "; ".join(wrong) or
           "%d chapters, %d stops OK; %d bad cases caught" % (len(doc["chapters"]), n, len(bad)))


# --- AC22 -------------------------------------------------------------------------------------
def ac22():
    import copy
    import alerts
    import hud
    import i18n
    import proto
    import tour
    doc = tour.load()
    wrong = []
    miss = tour.missing(doc)
    if miss:
        wrong.append("not covered: %s" % miss)
    btn, pan, pages = tour.hud_buttons(), tour.s2_panels(), tour.tools_pages()
    if not {"lang", "night", "declutter", "scale", "ar_menu", "edit", "vision_clear", "remote_req",
            "maneuver_req", "release"} <= set(btn):
        wrong.append("the HUD button list reads %s" % btn)
    if not {"status", "alarms", "helm", "link", "control", "traffic", "relays"} <= set(pan):
        wrong.append("the screen-2 panel list reads %s" % pan)
    if len(pages) < 15 or "tab.engine" not in pages:
        wrong.append("the tools page list reads %s" % pages)
    req = tour.required_items()
    want = (len(hud.OVERLAYS) + 1 + len(btn) + len(tour.HUD_BUTTONS_EXTRA) + len(pan) + len(tour.S2_EXTRA) +
            len(pages) + len(proto.MODES) + len(alerts.ALARM_LEVELS) + 1 + len(tour.LAMPS) + len(tour.JOYSTICK) +
            len(tour.PANEL))
    if len(req) != want or len(alerts.ALARM_LEVELS) < 38:
        wrong.append("required %d items, expected %d" % (len(req), want))
    # the checker is not blind: a chapter taken away is reported
    d = copy.deepcopy(doc)
    d["chapters"] = [c for c in d["chapters"] if c["id"] != "alarms"]
    if len(tour.missing(d)) != len(alerts.ALARM_LEVELS) + 1:
        wrong.append("removing the alarm chapter was not seen")
    greek = re.compile(u"[Ͱ-Ͽ]")
    for lc in ("el", "en"):
        for s in tour.stops(lc, doc):
            if not s["title"].strip() or s["title"].startswith("[") or not s["text"].strip():
                wrong.append("%s %s: empty or missing text" % (lc, s["id"]))
            if lc == "el" and not greek.search(s["text"]):
                wrong.append("el %s: not Greek" % s["id"])
            if s["show"] == "alarm" and s["title"] != i18n.TL(lc, "alarm." + s["target"]):
                wrong.append("%s %s: not the screen's alarm text" % (lc, s["id"]))
    report("AC22 tour covers every screen item", not wrong, "; ".join(wrong[:8]) or
           "%d items covered (%d HUD boxes, %d HUD buttons, %d screen-2 parts, %d tools pages, %d modes, %d alarms, "
           "lamps, joystick, panel); el + en complete" % (len(req), len(hud.OVERLAYS) + 1, len(btn) + 2,
                                                          len(pan) + 5, len(pages), len(proto.MODES),
                                                          len(alerts.ALARM_LEVELS) + 1))


# --- AC23 -------------------------------------------------------------------------------------
def ac23():
    wrong = []
    gs = os.path.join(REPO, "groundstation")
    os.environ.setdefault("TRYGONS_GS_SETTINGS", os.path.join(tempfile.mkdtemp(), "gs_settings.json"))
    os.environ.setdefault("TRYGONS_SOUND", "off")
    if gs not in sys.path:
        sys.path.insert(0, gs)
    import tour
    import sound
    import academymenu
    tmp = tempfile.mkdtemp(prefix="trygons_ac23_")
    lib = sound.Library(tmp)
    for lc in ("el", "en"):
        ss = tour.sentences(lc)
        if academymenu.tour_sentences(lc) != ss:
            wrong.append("%s: the station's sentences differ" % lc)
        for t in ss[:5] + ss[-5:]:
            if tour.voice_path(lc, t, tmp) != lib.sentence_path(lc, t):
                wrong.append("%s: voice file name differs" % lc)
                break
        if len(set(ss)) != len(ss):
            wrong.append("%s: two stops say the same sentence" % lc)
    saved = os.environ.pop("TRYGONS_SOUND_CACHE", None)
    if tour.voice_cache() != sound.Library().cache:
        wrong.append("default cache %s != %s" % (tour.voice_cache(), sound.Library().cache))
    if saved is not None:
        os.environ["TRYGONS_SOUND_CACHE"] = saved
    cmd = academymenu.tour_command()
    if cmd[1:] != [os.path.join(REPO, "academy", "tour_app.py"), "--lang", cmd[-1]] or cmd[-1] not in ("el", "en"):
        wrong.append("tour command %s" % cmd)
    src = open(os.path.join(gs, "academymenu.py"), encoding="utf-8").read()
    opener = src.split("def open_tour")[1].split("\ndef ")[0]
    if "link" in opener or "tour_command()" not in opener:
        wrong.append("open_tour hands the link over")
    made = []

    class Lib(object):
        def make(self, lang, texts):
            made.append((lang, list(texts)))

    class Snd(object):
        lib = Lib()

    class FakeApp(object):
        def __init__(self, snd):
            self.sound, self.said = snd, []

        def say(self, t, c=None):
            self.said.append(t)
    started = []
    p = None
    old = os.environ.get("TRYGONS_SOUND")
    try:
        os.environ["TRYGONS_SOUND"] = "on"
        app = FakeApp(Snd())
        p = academymenu.open_tour(app, popen=lambda c, cwd=None: started.append(c) or "proc")
        th = academymenu.prepare_tour_voice(app, "el")
        if th:
            th.join(5)
        if academymenu.prepare_tour_voice(FakeApp(None)) is not None:
            wrong.append("voice without a sound system")
        os.environ["TRYGONS_SOUND"] = "off"
        if academymenu.prepare_tour_voice(FakeApp(Snd())) is not None:
            wrong.append("voice made with TRYGONS_SOUND=off")
    finally:
        if old is None:
            os.environ.pop("TRYGONS_SOUND", None)
        else:
            os.environ["TRYGONS_SOUND"] = old
    if p != "proc" or started != [cmd]:
        wrong.append("open_tour started %s" % started)
    if not made or made[-1] != ("el", tour.sentences("el")):
        wrong.append("the voice was not made with the tour's sentences (%d calls)" % len(made))
    names, _ = imports_of(os.path.join(HERE, "tour_app.py"))
    names2, _ = imports_of(os.path.join(HERE, "tour.py"))
    if (names | names2) & {"sound", "tts", "subprocess"}:
        wrong.append("the academy tour imports the TTS / a process module")
    report("AC23 tour voice", not wrong, "; ".join(wrong) or
           "same cache names as the station's TTS; menu = own process (--lang only) + the voice made in the "
           "background with the %d tour sentences" % len(tour.sentences("el")))


# --- AC24 -------------------------------------------------------------------------------------
def ac24():
    import banner
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as e:
        report("AC24 tour window", True, "no display here (%s): content checked by AC21-AC23" % e)
        return
    import sound
    import tour
    import tour_app
    wrong = []
    tmp = tempfile.mkdtemp(prefix="trygons_ac24_")
    try:
        root.withdraw()
        for lc in ("el", "en"):
            top = tk.Toplevel(root)
            pl = tour_app.NullPlayer()
            w = tour_app.TourWindow(top, lc, player=pl, cache=tmp)
            top.update()
            if top.pack_slaves()[0] is not w.banner or w.banner.cget("text") != banner.banner_text(lc) or \
                    w.banner.cget("bg").lower() != tour_app.CAUTION.lower():
                wrong.append("%s: banner not first / text / colour" % lc)
            n = len(w.stops)
            seen = {w.stops[0]["show"]}
            for _ in range(n + 2):                       # every stop drawn, past the end stays at the end
                w.next()
                seen.add(w.stops[w.index]["show"])
            if w.index != n - 1 or seen != set(tour.SHOWS):
                wrong.append("%s: walk ended at %d of %d, shows %s" % (lc, w.index, n, sorted(seen)))
            w.back()
            if w.index != n - 2:
                wrong.append("%s: back" % lc)
            w.jump("alarms")
            if w.stops[w.index]["show"] != "alarm" or w.stops[w.index - 1]["show"] == "alarm":
                wrong.append("%s: chapter jump" % lc)
            if w.voice_state != "missing":
                wrong.append("%s: no voice file but %s" % (lc, w.voice_state))
            path = tour.voice_path(lc, tour.sentence(w.stops[w.index]), tmp)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            sound.write_wav(path, sound._silence(50))
            w.speak()
            if w.voice_state != "played" or pl.played[-1] != os.path.abspath(path):
                wrong.append("%s: the prepared voice not played (%s)" % (lc, w.voice_state))
            saved = os.environ.get("PROGRAMDATA")
            os.environ["PROGRAMDATA"] = gateway_pc()["PROGRAMDATA"]
            try:
                w.speak()
                if w.voice_state != "missing":
                    wrong.append("%s: played on a gateway PC" % lc)
            finally:
                if saved is None:
                    os.environ.pop("PROGRAMDATA", None)
                else:
                    os.environ["PROGRAMDATA"] = saved
            w.toggle_voice()
            if w.voice_state != "off" or pl.played[-1] is not None:
                wrong.append("%s: voice off" % lc)
            top.destroy()
        import academy_app
        app = academy_app.AcademyApp(tk.Toplevel(root), _store())
        tw = app.open_tour()
        if not isinstance(tw, tour_app.TourWindow):
            wrong.append("academy window: no tour")
    finally:
        root.destroy()
    report("AC24 tour window", not wrong, "; ".join(wrong) or
           "banner first; every stop drawn in el + en; back / jump; voice only from a prepared file via simguard; "
           "academy button opens it")


# --- step 18 helpers --------------------------------------------------------------------------
def _free_port_pair():
    """A port p with p and p + 2 free on 127.0.0.1 (the simulator listens on both)."""
    for _ in range(50):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        if p + 2 > 65535:
            continue
        t = socket.socket()
        try:
            t.bind(("127.0.0.1", p + 2))
            t.close()
            return p
        except OSError:
            t.close()
    raise RuntimeError("no free port pair")


class _TcpLink(object):
    """TEST ONLY: a byte link to a test simulator on a free port (the academy itself only gets
    simguard.SimLink = port 5760; this proves the lessons against the real simulator)."""

    def __init__(self, port):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.s.setblocking(False)
        self.closed = False

    def send(self, d):
        try:
            self.s.sendall(d)
        except OSError:
            self.closed = True

    def recv(self):
        try:
            d = self.s.recv(65536)
            if not d:
                self.closed = True
            return d
        except (BlockingIOError, InterruptedError):
            return b""
        except OSError:
            self.closed = True
            return b""

    def close(self):
        self.closed = True
        try:
            self.s.close()
        except OSError:
            pass


class _TestSim(object):
    """TEST ONLY: sim/autopilot_sim server on a free port in a temp folder."""

    def __init__(self):
        exe = os.path.join(REPO, "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
        self.port = _free_port_pair()
        self.dir = tempfile.mkdtemp(prefix="trygons_acsim_")
        self.p = subprocess.Popen([exe, "server", str(self.port), "1"], cwd=self.dir, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        import time
        end = time.time() + 10
        while True:
            try:
                socket.create_connection(("127.0.0.1", self.port + 2), timeout=0.5).close()
                break
            except OSError:
                if time.time() > end:
                    raise
                time.sleep(0.2)

    def link(self):
        return _TcpLink(self.port)

    def stop(self):
        self.p.kill()
        try:
            self.p.wait(5)
        except Exception:
            pass


# what a trainee does in each step of each lesson (None = only watch; "confirm" = press + release CONFIRM;
# "ack" = acknowledge every 2 s until the step passes). A new lesson without a script fails AC28.
def _code(sim):
    sim.power_raise(sim.state.get("power_code", 0))


LESSON_SCRIPTS = {
    "l01_hdg_lock": [lambda s: s.button("hdg_lock"), lambda s: s.button("plus10"), lambda s: s.button("minus1"),
                     lambda s: s.button("standby")],
    "l02_takeover": [lambda s: s.button("hdg_lock"), lambda s: s.stick(x=0.6), lambda s: s.stick(x=0.0),
                     lambda s: s.button("hdg_lock"), lambda s: s.button("standby")],
    "l03_standby_wins": [lambda s: s.button("hdg_lock"), lambda s: s.stick(y=0.6), lambda s: s.button("standby"),
                         lambda s: s.stick(y=0.0)],
    "l04_alarm_reaction": [lambda s: s.button("hdg_lock"), None, lambda s: s.button("standby"), "ack"],
    "l05_power_raise": [lambda s: s.power_code(), _code, "confirm", lambda s: s.power_lower()],
}
# the spec C list -> the lesson that teaches it
SPEC_C = {"HDG LOCK": "l01_hdg_lock", "course +10": "l01_hdg_lock", "takeover": "l02_takeover",
          "hand-back": "l02_takeover", "power raise with CONFIRM": "l05_power_raise",
          "alarm reaction": "l04_alarm_reaction"}


def _drive(sim, run, script, limit=90.0, skip=(), on_step=None):
    """Run a lesson like a trainee: each step's action 0.6 s after the step shows. skip = step indexes
    where the trainee does nothing. Returns the final status."""
    import time
    for a in run.start(time.time()):
        sim.do(a)
    acted, entered, last_ack = -2, (None, 0.0), 0.0
    end = time.time() + limit
    while time.time() < end:
        now = time.time()
        status, acts = run.update(sim.tick(), now)
        for a in acts:
            sim.do(a)
        if status in ("done", "failed"):
            return status
        i = run.index
        if i >= 0:
            if entered[0] != i:
                entered = (i, now)
            act = script[i]
            if i not in skip and acted != i and now - entered[1] > 0.6:
                acted = i
                if on_step:
                    on_step(i)
                if act == "confirm":
                    sim.press("confirm", True)
                    for _ in range(4):
                        sim.tick()
                        time.sleep(0.05)
                    sim.press("confirm", False)
                elif callable(act):
                    act(sim)
            if act == "ack" and i not in skip and now - last_ack > 2.0:
                sim.alarm_ack()
                last_ack = now
        time.sleep(0.05)
    return "timeout"


# --- AC25 -------------------------------------------------------------------------------------
def ac25():
    import content
    import lesson
    import proto
    wrong = []
    docs = lesson.lessons()
    for d in docs:
        wrong += content.check(d, d["id"]) + lesson.check_doc(d, d["id"])
    if len(docs) < 5:
        wrong.append("only %d lessons" % len(docs))
    base = {"kind": "lesson", "id": "x", "_help": "x", "runs_on": "sim", "title_el": "a", "title_en": "b"}

    def stp(**kw):
        d = {"do_el": "a", "do_en": "b", "hint_el": "c", "hint_en": "d", "check": [{"f": "mode_name", "eq": "HDG_LOCK"}]}
        d.update(kw)
        return d
    bad = [dict(base, steps=[]), dict(base, steps=[stp(check=[])]), dict(base, steps=[stp(check=[{"f": "nonsense", "eq": 1}])]),
           dict(base, steps=[stp(check=[{"f": "mode_name", "eq": "A", "ne": "B"}])]),
           dict(base, steps=[stp(at_start="engine_kill")]), dict(base, setup=["raise_power"], steps=[stp()]),
           dict(base, steps=[stp(hint_el="")]), dict(base, steps=[stp(fail_s=0)])]
    missed = [i for i, d in enumerate(bad) if not lesson.check_doc(d, "bad%d" % i)]
    if missed:
        wrong.append("lesson checker missed %s" % missed)
    # the model on made-up simulator states
    doc = dict(base, setup=["standby"], steps=[
        stp(hint_s=5),
        stp(check=[{"f": "target", "delta": 10, "tol": 0.5}], hold_s=1.0),
        stp(check=[{"f": "mode_name", "eq": "STANDBY"}], fail_s=8)])
    r = lesson.Run(doc, "en")
    if r.start(0.0) != ["standby"]:
        wrong.append("setup actions")
    ready = {"mode_name": "STANDBY", "alarm_bits": 0, "profile_name": "CREWED", "target": 355.0}
    if r.update({"mode_name": "STANDBY", "alarm_bits": 0x1000, "profile_name": "CREWED"}, 1.0)[0] != "waiting":
        wrong.append("not armed = not ready")
    if r.update(dict(ready, profile_name="DETECTING"), 1.1)[0] != "waiting":
        wrong.append("profile not latched = not ready")
    r.update(ready, 2.0)
    if r.index != 0:
        wrong.append("ready did not start step 1")
    r.update(ready, 8.0)
    if not r.hint_shown or r.hints != 1:
        wrong.append("hint after hint_s")
    hl = dict(ready, mode_name="HDG_LOCK")
    r.update(hl, 9.0)
    if r.index != 1:
        wrong.append("step 1 not passed on HDG_LOCK")
    r.update(dict(hl, target=4.0), 9.5)               # 355 -> 4 = +9: not yet
    r.update(dict(hl, target=5.2), 10.0)              # 355 -> 5.2 = +10.2 (wrap) but must hold 1 s
    if r.index != 1:
        wrong.append("hold_s not respected")
    r.update(dict(hl, target=5.2), 11.1)
    if r.index != 2:
        wrong.append("delta with 360 wrap / hold")
    if r.update(dict(hl, target=5.2), 20.0)[0] != "failed" or r.failed_step != 2:
        wrong.append("fail_s")
    r.retry(21.0)
    for t, st in ((22.0, ready), (23.0, hl), (23.5, dict(hl, target=5.0)), (24.6, dict(hl, target=5.0)),
                  (25.0, dict(ready, target=5.0))):
        r.update(st, t)
    e = r.log_entry()
    if r.status != "done" or e["retries"] != 1 or e["score"] != 90 or not e["done"]:
        wrong.append("retry / score %s %s" % (r.status, e))
    # grouping, required lessons, the exam's list
    g = lesson.by_level(docs)
    if list(g) != ["OBSERVER", "DRIVER", "CAPTAIN"] or not g["DRIVER"] or not g["CAPTAIN"]:
        wrong.append("by level %s" % {k: len(v) for k, v in g.items()})
    req_d, req_c = lesson.required_for("DRIVER", docs), lesson.required_for("CAPTAIN", docs)
    if not set(req_d) < set(req_c) or "l05_power_raise" in req_d:
        wrong.append("required by level")
    exam = json_load(os.path.join(HERE, "content", "exams", "driver_exam.json"))
    ids = {d["id"] for d in docs}
    if not set(exam["required"]) <= ids:
        wrong.append("exam requires unknown lessons")
    log = [{"user": "ann", "kind": "lesson", "id": "l01_hdg_lock", "done": True},
           {"user": "ann", "kind": "lesson", "id": "l02_takeover", "done": False},
           {"user": "bob", "kind": "lesson", "id": "l02_takeover", "done": True}]
    miss = lesson.missing_required(log, "ann", exam)
    if "l01_hdg_lock" in miss or "l02_takeover" not in miss:
        wrong.append("missing_required %s" % miss)
    # only simulator inputs + the station's own commands go out
    class Sink(object):
        def __init__(self):
            self.out = []

        def send(self, d):
            self.out.append(d)

        def recv(self):
            return b""
    sim = lesson.Sim(Sink())
    for ft in (proto.T_GW_CMD, proto.T_GW_STATUS, proto.T_HELM, proto.T_REMOTE, proto.T_SET_MODE, proto.T_AUTO_START,
               proto.T_RESTART, proto.T_RELAY, proto.T_PARAM_SET):
        try:
            sim.send(ft)
            wrong.append("frame 0x%02X allowed" % ft)
        except ValueError:
            pass
    if lesson.ALLOWED_FRAMES & set(range(0x30, 0x3F)):
        wrong.append("a gateway frame in ALLOWED_FRAMES")
    sim.tick()
    for a in lesson.SETUP + lesson.EVENTS:
        sim.do(a)
    report("AC25 lessons: content + model", not wrong, "; ".join(wrong) or
           "%d lessons (%s) OK; %d bad cases caught; ready / delta wrap / hold / hint / time limit / retry / score; "
           "required before the exam; only simulator + station frames" % (len(docs), ", ".join(sorted(ids)), len(bad)))


# --- AC26 -------------------------------------------------------------------------------------
def ac26():
    import simguard
    wrong = []
    made = []
    orig = socket.create_connection
    socket.create_connection = lambda *a, **k: made.append(a)
    tmp = tempfile.mkdtemp(prefix="trygons_ac26_")
    with open(os.path.join(tmp, "x.json"), "w", encoding="utf-8") as f:
        f.write('{"id": "x1", "name": "X1", "tailscale_ip": "100.101.102.103", "gateway_port": 5761}')
    try:
        for h, p in (("127.0.0.1", 5761), ("127.0.0.1", 5762), ("192.168.1.10", 5760), ("100.101.102.103", 5760),
                     ("100.64.0.9", 5760), ("boat.local", 5760), ("127.0.0.1", 6000)):
            try:
                simguard.SimLink(h, p, boats_dir=tmp)
                wrong.append("SimLink allowed %s:%s" % (h, p))
            except simguard.Refused:
                pass
    finally:
        socket.create_connection = orig
    if made:
        wrong.append("a socket was made for a refused SimLink")
    calls = []

    def fake_popen(cmd, **kw):
        calls.append((cmd, kw))
        return "proc"
    data = tempfile.mkdtemp(prefix="trygons_ac26d_")
    env = {"PROGRAMDATA": tempfile.mkdtemp(), "TRYGONS_ACADEMY_DATA": data}
    if simguard.start_simulator(popen=fake_popen, env=env) != "proc":
        wrong.append("start_simulator result")
    cmd, kw = calls[-1]
    exe = os.path.join(REPO, "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if cmd != [exe, "server", "5760", "1"]:
        wrong.append("command %s" % cmd)
    if os.path.normcase(kw.get("cwd", "")) != os.path.normcase(os.path.join(data, "sim")) or not os.path.isdir(kw["cwd"]):
        wrong.append("working folder %s" % kw.get("cwd"))
    try:
        simguard.start_simulator(popen=fake_popen, env=gateway_pc())
        wrong.append("started on a gateway PC")
    except simguard.Refused:
        pass
    saved = simguard.sim_exe
    simguard.sim_exe = lambda: os.path.join(tmp, "no_such_sim.exe")
    try:
        simguard.start_simulator(env=env)
        wrong.append("started without the built simulator")
    except simguard.Refused:
        pass
    finally:
        simguard.sim_exe = saved
    # only simguard starts a process (AC1 lets it import subprocess; nobody else may)
    for name, path in academy_modules():
        src = open(path, encoding="utf-8").read()
        if name != GUARD and re.search(r"\b(Popen|subprocess|os\.system|startfile|spawn)\b", src):
            wrong.append("%s starts a process" % name)
    report("AC26 simguard: lessons link + simulator start", not wrong, "; ".join(wrong) or
           "SimLink only 127.0.0.1:5760 (refused before a socket); start = %s in academy/data/sim; refused on a "
           "gateway PC / without the simulator; no other module starts a process" % " ".join(["autopilot_sim"] + list(simguard.SIM_ARGS)))


# --- AC27 -------------------------------------------------------------------------------------
def ac27():
    import content
    import quiz
    import academymenu
    import sound
    wrong = []
    pools = quiz.pools()
    for d in pools:
        wrong += content.check(d, d["id"]) + quiz.check_doc(d, d["id"])
        if len(d.get("questions", [])) < 5:
            wrong.append("%s: fewer than 5 questions" % d["id"])
    qs = [q for d in pools for q in d["questions"]]
    types = {q.get("type", "choice") for q in qs}
    shows = {q["show"] for q in qs if q.get("type") == "picture"}
    sounds = {q["sound"] for q in qs if q.get("type") == "sound"}
    if types != set(quiz.TYPES) or shows != set(quiz.SHOWS) or sounds != set(quiz.SOUNDS):
        wrong.append("coverage types %s shows %s sounds %s" % (sorted(types), sorted(shows), sorted(sounds)))
    if {d.get("level") for d in pools} != {"OBSERVER", "DRIVER", "CAPTAIN"}:
        wrong.append("a level without a pool")
    base = {"kind": "quiz", "id": "x", "_help": "x"}
    q0 = {"q_el": "a", "q_en": "b", "why_el": "c", "why_en": "d", "choices": [{"text_el": "1", "text_en": "1"},
                                                                         {"text_el": "2", "text_en": "2"}], "answer": 0}
    bad = [dict(base, questions=[]), dict(base, questions=[dict(q0, type="essay")]),
           dict(base, questions=[dict(q0, answer=5)]), dict(base, questions=[dict(q0, why_en="")]),
           dict(base, questions=[{"type": "picture", "q_el": "a", "q_en": "b", "why_el": "c", "why_en": "d",
                                  "show": "s1", "area": "nowhere"}]),
           dict(base, questions=[dict(q0, type="sound", sound="cavalry")])]
    missed = [i for i, d in enumerate(bad) if not (quiz.check_doc(d, "b") or content.check(d, "b"))]
    if missed:
        wrong.append("quiz checker missed %s" % missed)
    # the sounds: the station's alarm-level tones, written where the quiz looks
    lv = {v for v in sound.LEVEL_TONE.values() if v}
    if set(quiz.SOUNDS) != lv or tuple(academymenu.ACADEMY_TONES) != tuple(quiz.SOUNDS) or \
            not set(quiz.SOUNDS) <= set(sound.TONE_NAMES) or academymenu.TONE_DIR != quiz.TONE_DIR:
        wrong.append("tone names / folder differ between the station and the quiz")
    cache = tempfile.mkdtemp(prefix="trygons_ac27_")

    class Lib(object):
        pass

    class Snd(object):
        lib = Lib()

    class App(object):
        sound = Snd()

        def say(self, *a):
            pass
    Snd.lib.cache = cache
    saved = os.environ.pop("TRYGONS_SOUND", None)
    try:
        written = academymenu.prepare_academy_sounds(App())
        again = academymenu.prepare_academy_sounds(App())
    finally:
        if saved is not None:
            os.environ["TRYGONS_SOUND"] = saved
    for n in quiz.SOUNDS:
        p = quiz.tone_path(n, cache)
        if p not in written or not os.path.isfile(p) or sound.read_wav(p).tobytes() != sound.tone(n).tobytes():
            wrong.append("tone %s not the station's" % n)
    if again:
        wrong.append("tones rewritten")
    os.environ["TRYGONS_SOUND"] = "off"
    try:
        if academymenu.prepare_academy_sounds(App()) != []:
            wrong.append("tones made with the sound off")
    finally:
        if saved is None:
            os.environ.pop("TRYGONS_SOUND", None)
        else:
            os.environ["TRYGONS_SOUND"] = saved
    # the run: random order, shuffled choices, every type answered
    r1, r2 = quiz.Run(pools, "el", seed=1), quiz.Run(pools, "el", seed=2)
    o1 = [(it["pool"], it["q"]["q_en"]) for it in r1.items]
    o2 = [(it["pool"], it["q"]["q_en"]) for it in r2.items]
    if o1 == o2 or sorted(o1) != sorted(o2) or len(o1) != len(qs):
        wrong.append("random order")
    if all(it["order"] == sorted(it["order"]) for it in r1.items if len(it["order"]) > 2):
        wrong.append("choices never shuffled")
    wrong_first = True
    while not r1.finished():
        q = r1.current()
        it = r1.items[r1.index]
        if q["type"] == "picture":
            x, y, w, h = quiz.area_rect(q["show"], q["area"])
            if wrong_first:
                fx = x + w + 0.15 if x + w + 0.15 <= 1.0 else x - 0.15
                res = r1.click(fx, y + h / 2)
                if res["correct"]:
                    wrong.append("a click outside counted")
                wrong_first = False
            else:
                res = r1.click(x + w / 2, y + h / 2)
                if not res["correct"]:
                    wrong.append("a click inside %s missed" % q["area"])
        else:
            right = it["order"].index(it["q"]["answer"])
            if q["choices"][right] != r1.tx(it["q"]["choices"][it["q"]["answer"]], "text"):
                wrong.append("shuffle lost the answer")
            res = r1.answer(right)
            if not res["correct"] or not res["why"]:
                wrong.append("right answer not right / no explanation")
            if r1.answer((right + 1) % len(q["choices"]))["correct"] is not True:
                wrong.append("answer changed after it was given")
        r1.next()
    ents = r1.log_entries()
    if r1.score() != int(round(100.0 * (len(qs) - 1) / len(qs))) or {e["topic"] for e in ents} != {d["id"] for d in pools}:
        wrong.append("score %s / log %s" % (r1.score(), ents))
    r3 = quiz.Run(pools, "en", seed=3, sound_ok=lambda n: False)
    if r3.left_out != len([q for q in qs if q.get("type") == "sound"]) or any(it["q"].get("type") == "sound" for it in r3.items):
        wrong.append("sound questions without a file not left out")
    report("AC27 quizzes: content + model + sounds", not wrong, "; ".join(wrong) or
           "%d pools, %d questions (choice / picture %s / sound %s); %d bad cases caught; random order + shuffled "
           "choices; clicks in / out; per-pool scores; the station's tones = what the quiz plays"
           % (len(pools), len(qs), "+".join(sorted(shows)), "+".join(sorted(sounds)), len(bad)))


# --- AC28 -------------------------------------------------------------------------------------
def ac28():
    import lesson
    import time
    wrong = []
    docs = lesson.lessons()
    ids = {d["id"] for d in docs}
    for item, lid in SPEC_C.items():
        if lid not in ids:
            wrong.append("spec C %r: lesson %s missing" % (item, lid))
    for d in docs:
        if d["id"] not in LESSON_SCRIPTS or len(LESSON_SCRIPTS[d["id"]]) != len(d["steps"]):
            wrong.append("%s: no trainee script for every step" % d["id"])
    if wrong:
        report("AC28 lessons on the real simulator", False, "; ".join(wrong))
        return
    sim_p = _TestSim()
    times = {}
    try:
        sim = lesson.Sim(sim_p.link())
        for d in docs:
            t0 = time.time()
            r = lesson.Run(d, "el")
            st = _drive(sim, r, LESSON_SCRIPTS[d["id"]])
            times[d["id"]] = round(time.time() - t0, 1)
            if st != "done" or r.score() != 100:
                wrong.append("%s: %s at step %d (score %d)" % (d["id"], st, r.index + 1, r.score()))
        by = {d["id"]: d for d in docs}
        # wrong actions do not pass a step: +10 instead of moving the stick (takeover step stays)
        r = lesson.Run(by["l02_takeover"], "en")
        bad = [LESSON_SCRIPTS["l02_takeover"][0], lambda s: s.button("plus10")] + LESSON_SCRIPTS["l02_takeover"][2:]
        if _drive(sim, r, bad, limit=4.0) != "timeout" or r.index != 1:
            wrong.append("l02: +10 passed the takeover step (step %d)" % (r.index + 1))
        # CONFIRM without a code typed: no raise
        r = lesson.Run(by["l05_power_raise"], "en")
        bad = [LESSON_SCRIPTS["l05_power_raise"][0], "confirm", "confirm", None]
        if _drive(sim, r, bad, limit=5.0) != "timeout" or r.index != 1 or sim.state.get("power_fwd", 0) > 30:
            wrong.append("l05: CONFIRM without the code passed (step %d, limit %s)" % (r.index + 1, sim.state.get("power_fwd")))
        # the reaction time: no STANDBY -> failed; the retry passes (one retry = 90)
        r = lesson.Run(by["l04_alarm_reaction"], "en")
        st = _drive(sim, r, LESSON_SCRIPTS["l04_alarm_reaction"], skip=(2,), limit=60.0)
        if st != "failed" or r.failed_step != 2:
            wrong.append("l04: no STANDBY did not fail (%s, step %s)" % (st, r.failed_step))
        else:
            r.retry(time.time())
            st = _drive_retry(sim, r, LESSON_SCRIPTS["l04_alarm_reaction"])
            if st != "done" or r.log_entry()["retries"] != 1 or r.score() != 90:
                wrong.append("l04 retry: %s %s" % (st, r.log_entry()))
        sim.do("centre")
        sim.do("standby")
        sim.link.close()
    finally:
        sim_p.stop()
    report("AC28 lessons on the real simulator", not wrong, "; ".join(wrong) or
           "every lesson done by its steps (%s s); spec C items covered; +10 is no takeover; CONFIRM without the "
           "code raises nothing; a missed reaction fails, the retry passes (score 90)"
           % ", ".join("%s %s" % (k, v) for k, v in times.items()))


def _drive_retry(sim, run, script):
    """Continue a retried run (start() already called by retry())."""
    saved = run.start
    run.start = lambda now: list(run.doc.get("setup", []))
    try:
        return _drive(sim, run, script)
    finally:
        run.start = saved


# --- AC29 -------------------------------------------------------------------------------------
def ac29():
    import banner
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as e:
        report("AC29 lesson + quiz windows", True, "no display here (%s): models checked by AC25-AC28" % e)
        return
    import time
    import lesson
    import lesson_app
    import quiz
    import quiz_app
    import tour_app
    import sound
    wrong = []
    sim_p = None
    try:
        root.withdraw()
        store = _store()
        store.add("ann", "Ann")
        # no simulator: said so, nothing crashes
        def no_sim():
            raise OSError("refused")
        top = tk.Toplevel(root)
        w = lesson_app.LessonWindow(top, "en", store=store, user="ann", link_factory=no_sim)
        top.update()
        if top.pack_slaves()[0] is not w.banner or w.banner.cget("text") != banner.banner_text("en") or \
                w.banner.cget("bg").lower() != lesson_app.CAUTION.lower():
            wrong.append("lesson window: banner not first")
        heads = [w.list.get(i) for i, d in enumerate(w.rows) if d is None]
        if len(heads) < 2 or w.rows[0] is not None:
            wrong.append("lessons not grouped by level (%s)" % heads)
        w.select("l01_hdg_lock")
        if w.start_selected() is not None or w.conn.cget("text") != lesson_app.T("lesson.no_sim", "en"):
            wrong.append("no simulator not said")
        w.close()
        # a lesson through the window on the simulator -> logbook + tick
        sim_p = _TestSim()
        top = tk.Toplevel(root)
        w = lesson_app.LessonWindow(top, "el", store=store, user="ann", link_factory=sim_p.link)
        w.select("l01_hdg_lock")
        if w.start_selected() is None:
            wrong.append("lesson did not start")
        else:
            acts = {0: lambda: w.button("hdg_lock"), 1: lambda: w.button("plus10"), 2: lambda: w.button("minus1"),
                    3: lambda: w.button("standby")}
            done, entered = set(), (None, 0)
            end = time.time() + 40
            while time.time() < end and w.status != "done":
                w.step()
                top.update()
                i = w.run.index
                if i >= 0:
                    if entered[0] != i:
                        entered = (i, time.time())
                    if i not in done and time.time() - entered[1] > 0.6:
                        done.add(i)
                        acts[i]()
                time.sleep(0.05)
            store.load()
            ok = [e for e in store.entries() if e.get("kind") == "lesson" and e.get("id") == "l01_hdg_lock"
                  and e.get("user") == "ann" and e.get("done")]
            row = [w.list.get(i) for i, d in enumerate(w.rows) if d and d["id"] == "l01_hdg_lock"]
            if w.status != "done" or not ok or not row or not row[0].startswith("✓"):
                wrong.append("window lesson: %s, logged %d, row %s" % (w.status, len(ok), row))
            # the joystick pad + hold: let go = centre; held = stays
            w.stick(0.0, 0.5)
            w._pad_release()
            if w.sim.joy["y"] != 0.0:
                wrong.append("released stick not centred")
            w.hold.set(True)
            w.stick(0.0, 0.5)
            w._pad_release()
            if w.sim.joy["y"] != 0.5:
                wrong.append("held stick not kept")
            w.hold.set(False)
            w._hold_changed()
            if w.sim.joy["y"] != 0.0:
                wrong.append("unticking hold did not centre")
        w.close()
        sim_p.stop()
        sim_p = None
        # the quiz window: every question, sounds through simguard from the station's tones
        cache = tempfile.mkdtemp(prefix="trygons_ac29_")
        for n in quiz.SOUNDS:
            sound.write_wav(quiz.tone_path(n, cache), sound.tone(n))
        for lc in ("el", "en"):
            top = tk.Toplevel(root)
            pl = tour_app.NullPlayer()
            qw = quiz_app.QuizWindow(top, lc, store=store, user="ann", player=pl, cache=cache, seed=7)
            top.update()
            if top.pack_slaves()[0] is not qw.banner or qw.banner.cget("text") != banner.banner_text(lc):
                wrong.append("%s quiz window: banner not first" % lc)
            run = qw.start([d["id"] for d in qw.docs])
            if run.left_out:
                wrong.append("%s: sounds left out with the files there" % lc)
            seen = set()
            while not run.finished():
                q = run.current()
                seen.add(q["type"])
                it = run.items[run.index]
                if q["type"] == "picture":
                    x, y, ww, hh = quiz.area_rect(q["show"], q["area"])
                    top.update()
                    x0, y0, fw, fh = qw.frame
                    res = qw._on_click(type("E", (), {"x": x0 + (x + ww / 2) * fw, "y": y0 + (y + hh / 2) * fh})())
                else:
                    if q["type"] == "sound" and (qw.sound_state != "played" or not pl.played or
                                                 pl.played[-1] != os.path.abspath(quiz.tone_path(q["sound"], cache))):
                        wrong.append("%s: sound %s not played" % (lc, q["sound"]))
                    res = qw.answer(it["order"].index(it["q"]["answer"]))
                if not res or not res["correct"]:
                    wrong.append("%s: %s question not answered right" % (lc, q["type"]))
                    break
                qw.next()
            if seen != set(quiz.TYPES) or run.score() != 100:
                wrong.append("%s: types %s score %d" % (lc, sorted(seen), run.score()))
            top.destroy()
        store.load()
        qe = [e for e in store.entries() if e.get("kind") == "quiz" and e.get("user") == "ann"]
        if len(qe) != 2 * len(quiz.pools()) or any(e.get("score") != 100 for e in qe):
            wrong.append("quiz logbook %d entries" % len(qe))
        # no tone files: the sound questions are left out and the window says so
        top = tk.Toplevel(root)
        qw = quiz_app.QuizWindow(top, "en", player=tour_app.NullPlayer(), cache=tempfile.mkdtemp(), seed=1)
        run = qw.start([d["id"] for d in qw.docs])
        if not run.left_out or not qw.l_note.cget("text"):
            wrong.append("missing tones not said")
        top.destroy()
        import academy_app
        app = academy_app.AcademyApp(tk.Toplevel(root), store)
        lw = app.open_lessons()
        qw = app.open_quizzes()
        if not isinstance(lw, lesson_app.LessonWindow) or not isinstance(qw, quiz_app.QuizWindow) or lw.user is not None:
            wrong.append("academy window buttons")
        lw.close()
    finally:
        if sim_p is not None:
            sim_p.stop()
        root.destroy()
    report("AC29 lesson + quiz windows", not wrong, "; ".join(wrong) or
           "banner first; lessons by level; no simulator said; a lesson through the window on the simulator logged + "
           "ticked; stick hold / let go; quiz el + en every type, sounds played via simguard, logbook per pool; "
           "missing tones left out + said; academy buttons open both")


# --- step 19 helpers: a scripted trainee for the emergency scenarios ---------------------------
def _sb(name):
    return lambda s: s.button(name)


def _sstick(x=None, y=None):
    return lambda s: s.stick(x, y)


def _sconfirm(s):
    import time
    s.press("confirm", True)
    for _ in range(4):
        s.tick()
        time.sleep(0.05)
    s.press("confirm", False)


class _Refused(object):
    """s05: ask for the code (refused: GPS LOST), wait for the GPS, acknowledge, code, raise, CONFIRM."""

    def __init__(self):
        self.t, self.stage = 0.0, 0

    def __call__(self, s):
        import time
        now = time.time()
        if now - self.t < 1.0:
            return
        self.t = now
        st = s.state
        if self.stage == 0:
            s.power_code()
            self.stage = 1
        elif self.stage == 1:
            if not st.get("gps_fix"):
                return
            s.alarm_ack()
            if not st.get("alarm_bits", 0):
                s.power_code()
                self.stage = 2
        elif self.stage == 2 and st.get("power_code"):
            _code(s)
            self.stage = 3
        elif self.stage == 3 and st.get("confirm_kind") == 3:
            _sconfirm(s)
            self.stage = 4


_GO = [_sb("hdg_lock"), _sstick(y=1.0)]
# what a trainee does: prep = one action per prep step, react = the reaction (a list = one after the other),
# after = per follow-up step (None = only wait, "ack" = acknowledge every 2 s). A new scenario without a
# script fails AC31.
SCENARIO_SCRIPTS = {
    "s01_compass_fail": {"prep": _GO, "react": _sstick(y=0.0), "after": [None, "ack"]},
    "s02_gps_spoof": {"prep": _GO, "react": _sstick(y=0.0)},
    "s03_slamming": {"prep": [lambda s: s.power_code(), _code, _sconfirm] + _GO, "react": _sstick(y=0.0)},
    "s04_joystick_fault": {"prep": _GO, "react": _sstick(0.0, 0.0), "after": ["ack"]},
    "s05_power_refusal": {"prep": [_sb("hdg_lock")], "react": "refused"},
    "s06_ais_collision": {"prep": _GO, "react": [_sb("plus10")] * 6},
    "s07_shallow_water": {"prep": _GO, "react": _sb("standby")},
    "s08_swimmer_kayak": {"prep": _GO, "react": _sb("standby")},
    "s09_gateway_lost": {"prep": _GO, "react": _sstick(y=0.0)},
    "s10_remote_link_lost": {"prep": _GO, "react": _sstick(x=0.6)},
}
# the spec E list -> the scenario that trains it
SPEC_E = {"remote link lost": "s10_remote_link_lost", "compass failure": "s01_compass_fail",
          "AIS collision course": "s06_ais_collision", "shallow water": "s07_shallow_water",
          "GPS spoof": "s02_gps_spoof", "slamming": "s03_slamming", "joystick fault": "s04_joystick_fault",
          "power limit refusal": "s05_power_refusal", "gateway lost": "s09_gateway_lost",
          "swimmer / kayak (no AIS) near the beach": "s08_swimmer_kayak"}


def _drive_scen(sim, run, script, react=True, react_delay=1.5, limit=150.0, react_override=None, on_tick=None,
                start=True):
    """Run a scenario like a trainee. Returns the final status (or "timeout")."""
    import time
    if start:
        for a in run.start(time.time()):
            sim.do(a)
    acted, entered, t_ack = set(), {}, 0.0
    refused = _Refused()
    end = time.time() + limit
    while time.time() < end:
        now = time.time()
        status, acts = run.update(sim.tick(), now)
        for a in acts:
            sim.do(a)
        if on_tick:
            on_tick(status)
        if status in ("done", "failed"):
            return status
        key = (status, run.index)
        entered.setdefault(key, now)
        since = now - entered[key]
        if status == "prep" and key not in acted and since > 0.6:
            acted.add(key)
            script["prep"][run.index](sim)
        elif status == "react" and react:
            r = react_override or script["react"]
            if r == "refused":
                refused(sim)
            elif since > react_delay and key not in acted:
                acted.add(key)
                for i, a in enumerate(r if isinstance(r, list) else [r]):
                    if i:
                        for _ in range(4):
                            sim.tick()
                            time.sleep(0.05)
                    a(sim)
        elif status == "after":
            aft = script.get("after", [])
            a = aft[run.index] if run.index < len(aft) else None
            if a == "ack" and now - t_ack > 2.0:
                sim.alarm_ack()
                t_ack = now
        time.sleep(0.05)
    return "timeout"


def _scen_clean(sim):
    import time
    sim.do("centre")
    sim.do("standby")
    for _ in range(10):
        sim.tick()
        time.sleep(0.05)


# --- AC30 -------------------------------------------------------------------------------------
def ac30():
    import content
    import json
    import scenario                                     # puts groundstation/ on the path (proto)
    import proto
    import simguard
    wrong = []
    docs = scenario.scenarios()
    ids = {d["id"] for d in docs}
    for d in docs:
        wrong += content.check(d, d["id"]) + scenario.check_doc(d, d["id"])
    for item, sid in SPEC_E.items():
        if sid not in ids:
            wrong.append("spec E %r: scenario %s missing" % (item, sid))
    if any(n.startswith("example_") for n in os.listdir(os.path.join(HERE, "content", "scenarios"))):
        wrong.append("the step-15 example scenario is still there")
    # the checker catches the bad cases
    good = json.loads(json.dumps(next(d for d in docs if d["id"] == "s08_swimmer_kayak")))
    bads = []
    for mut in (lambda d: d.pop("event"), lambda d: d["event"].update(faults=["engine_fire"]),
                lambda d: d["event"].update(hazard={"type": "whale"}), lambda d: d.pop("outcome"),
                lambda d: d["react"].update(good_s=20), lambda d: d["react"].update(any=[]),
                lambda d: d["react"]["any"].append([{"f": "no_such_field", "eq": 1}]),
                lambda d: d.update(forbid=[{"check": [{"f": "mode_name", "eq": "AUTO"}]}]),
                lambda d: d["event"].update(show=["engine_fire"]), lambda d: d.pop("right_el"),
                lambda d: d["event"].pop("say_en"), lambda d: d["event"].update(hazard=None, show=[], faults=[]),
                lambda d: d.update(setup=["launch"])):
        b = json.loads(json.dumps(good))
        mut(b)
        bads.append(b)
    missed = [i for i, b in enumerate(bads) if not scenario.check_doc(b, "bad")]
    boat = dict(good, runs_on="boat")
    if not content.check(boat, "boat"):
        missed.append("runs_on boat")
    if missed:
        wrong.append("checker missed bad case(s) %s" % missed)
    # the model, on a made-up clock and made-up simulator states
    base = {"mode_name": "STANDBY", "alarm_bits": 0, "profile_name": "CREWED", "heading": 90.0, "target": 90.0,
            "est_lat": 37.8, "est_lon": 23.6, "est_sog": 5.0, "throttle": 30.0, "in_x": 0.0, "in_y": 1.0}
    doc = {"id": "t", "level": "DRIVER", "setup": [], "prep": [],
           "event": {"after_s": 2, "faults": ["compass"], "clear_after_s": 30, "say_el": "a", "say_en": "a"},
           "react": {"any": [[{"f": "mode_name", "eq": "MANUAL"}], [{"f": "target", "delta_ge": 20}]],
                     "good_s": 5, "max_s": 15},
           "forbid": [{"check": [{"f": "heading", "delta_le": -20}], "why_el": "x", "why_en": "x"}], "observe_s": 2,
           "right_el": "r", "right_en": "r", "debrief_el": "d", "debrief_en": "d"}


    def play(react_at=None, react_state=None, tmax=40.0, d=None, state0=None):
        r = scenario.Run(d or doc, "en")
        acts = r.start(0.0)
        t, st = 0.0, dict(state0 or base)
        seen = list(acts)
        while t < tmax:
            if react_at is not None and r.t_event is not None and t - r.t_event >= react_at:
                st.update(react_state)
            status, a = r.update(st, t)
            seen += a
            if status in ("done", "failed"):
                break
            t += 0.1
        return r, seen
    r, seen = play(1.0, {"mode_name": "MANUAL"})
    if r.status != "done" or r.score() != 100 or "fault:compass" not in seen or "fault_off:compass" not in seen:
        wrong.append("fast reaction: %s %s %s" % (r.status, r.score(), seen))
    if seen[:len(scenario.FAULTS)] != ["fault_off:%s" % f for f in scenario.FAULTS]:
        wrong.append("start does not clean every simulator fault first")
    if abs((r.reaction_s() or 0) - 1.0) > 0.15:
        wrong.append("reaction time %s" % r.reaction_s())
    r, _ = play(10.0, {"target": 115.0})                        # half way between good and max = 75
    if r.status != "done" or not 70 <= r.score() <= 80:
        wrong.append("slow reaction score %s (%s)" % (r.score(), r.status))
    r, _ = play(None, None)
    if r.status != "failed" or r.why != "late" or r.score() != 0:
        wrong.append("no reaction: %s %s" % (r.status, r.why))
    r, _ = play(1.0, {"heading": 60.0})                         # turned to port: forbidden
    if r.status != "failed" or r.why != "forbid" or not any("x" == ln for ln in r.debrief()):
        wrong.append("forbidden turn: %s %s" % (r.status, r.why))
    rr = scenario.Run(doc, "en")
    rr.retries = 1
    rr.status, rr.t_event, rr.t_react = "done", 0.0, 1.0
    if rr.score() != 90:
        wrong.append("one retry: score %s, expected 90" % rr.score())
    # wait_for: the emergency starts only once the simulator shows it
    d2 = json.loads(json.dumps(doc))
    d2["event"]["wait_for"] = [{"f": "alarm_bits", "bit": 2}]
    r = scenario.Run(d2, "en")
    r.start(0.0)
    st = dict(base)
    for i in range(60):
        if i == 40:
            st["alarm_bits"] = 2
        r.update(st, i * 0.1)
    if r.status != "react" or abs(r.t_event - 4.0) > 0.11:
        wrong.append("wait_for: %s at %s" % (r.status, r.t_event))
    # a hazard: the AIS target collides when the boat keeps course + speed; the outcome is measured
    hz = scenario.Hazard({"type": "ais", "collide_s": 60, "speed_kn": 8, "cross_deg": -90}, 90.0, 5.0)
    c, tc = scenario.cpa((0.0, 0.0), scenario._vec(90.0, 5.0 * scenario.KN), hz.pos(0), hz.v)
    if c > 1.0 or abs(tc - 60.0) > 0.5 or abs(scenario._wrap(hz.course - 0.0)) > 0.01:
        wrong.append("AIS placement cpa %.1f tcpa %.1f course %.0f" % (c, tc, hz.course))
    sh = scenario.Hazard({"type": "shallow", "edge_m": 80, "radius_m": 40}, 0.0, 5.0)
    if abs(sh.clearance((0.0, 0.0), 0) - 80.0) > 0.01 or sh.depth((120.0, 0.0), 0) != 1.0:
        wrong.append("shallow placement / depth")
    d3 = {"id": "h", "level": "DRIVER", "setup": [], "prep": [], "observe_s": 30, "outcome": {"min_m": 20},
          "event": {"after_s": 0, "hazard": {"type": "swimmer", "dist_m": 70, "off_m": 0}, "say_el": "a", "say_en": "a"},
          "react": {"any": [[{"f": "mode_name", "eq": "STANDBY"}]], "good_s": 4, "max_s": 12},
          "right_el": "r", "right_en": "r", "debrief_el": "d", "debrief_en": "d"}
    r = scenario.Run(d3, "en")
    r.start(0.0)
    st = dict(base, est_lat=37.8, est_lon=23.6, heading=0.0)
    t = 0.0
    while t < 30 and r.status not in ("done", "failed"):
        st["est_lat"] = 37.8 + (3.0 * max(0.0, t - 0.2)) / scenario.M_PER_DEG       # 3 m/s north, never stops
        r.update(st, t)
        t += 0.1
    if r.status != "failed" or r.why not in ("outcome", "late") or (r.min_m is None or r.min_m > 40):
        wrong.append("swimmer run over: %s %s min %s" % (r.status, r.why, r.min_m))
    if not r.timeline or "clear_m" not in r.timeline[-1]:
        wrong.append("timeline without the hazard distance")
    # the replay file: written, read back through simguard only (a device name refused)
    tmp = tempfile.mkdtemp(prefix="trygons_ac30_")
    path = scenario.save_replay(r, tmp, "ann", stamp="20261028-120000")
    head, samples = scenario.load_replay(path)
    if head.get("id") != "h" or head.get("why") != r.why or len(samples) != len(r.timeline):
        wrong.append("replay round trip %s %d/%d" % (head.get("id"), len(samples), len(r.timeline)))
    try:
        scenario.load_replay("COM3")
        wrong.append("a replay from COM3 was read")
    except simguard.Refused:
        pass
    # frames: only the simulator's inputs + the station's own commands (+ the simulator fault)
    class _L(object):
        def __init__(self):
            self.out = []

        def send(self, b):
            self.out.append(b)

        def recv(self):
            return b""
    s = scenario.Sim(_L())
    s.do("fault:compass")
    s.do("fault_off:joy_break")
    try:
        s.send(proto.T_GW_STATUS)
        wrong.append("a scenario sent a gateway frame")
    except ValueError:
        pass
    frames = [f for b in s.link.out for f in proto.Receiver().frames_from(b)]
    if [(f.type, bytes(f.payload)) for f in frames] != [(proto.T_SIM_FAULT, b"\x01\x01"), (proto.T_SIM_FAULT, b"\x04\x00")]:
        wrong.append("fault frames %s" % [(f.type, bytes(f.payload)) for f in frames])
    import lesson
    if scenario.ALLOWED_FRAMES != set(lesson.ALLOWED_FRAMES) | {proto.T_SIM_FAULT}:
        wrong.append("scenario frames are not the lessons' + the simulator fault: %s" % sorted(scenario.ALLOWED_FRAMES))
    # PROTO_T_SIM_FAULT: the same number on both sides, handled only by the simulator
    hdr = open(os.path.join(REPO, "app", "proto.h"), encoding="utf-8").read()
    m = re.search(r"#define PROTO_T_SIM_FAULT\s+0x([0-9A-Fa-f]+)u", hdr)
    if not m or int(m.group(1), 16) != proto.T_SIM_FAULT or not 0x70 <= proto.T_SIM_FAULT <= 0x7F:
        wrong.append("PROTO_T_SIM_FAULT differs between app/proto.h and proto.py")
    srv = open(os.path.join(REPO, "sim", "sim_server.c"), encoding="utf-8").read()
    if "case PROTO_T_SIM_FAULT" not in srv:
        wrong.append("sim_server.c does not handle PROTO_T_SIM_FAULT")
    for root_, _ds, fs in os.walk(os.path.join(REPO, "app")):
        for f in fs:
            if f.endswith(".c") and "PROTO_T_SIM_FAULT" in open(os.path.join(root_, f), encoding="utf-8").read():
                wrong.append("app/%s uses the simulator fault frame" % f)
    report("AC30 scenarios: content + model", not wrong, "; ".join(wrong) or
           "%d scenarios (spec E all 10), %d bad cases caught; reaction time / linear score / late / forbidden / "
           "outcome / retry 90 / wait_for / hazard geometry; replay via simguard; fault frame only in the simulator"
           % (len(docs), len(bads) + 1))


# --- AC31 -------------------------------------------------------------------------------------
def ac31():
    import threading
    import time
    import scenario
    wrong = []
    docs = {d["id"]: d for d in scenario.scenarios()}
    for sid, d in docs.items():
        sc = SCENARIO_SCRIPTS.get(sid)
        if sc is None or len(sc["prep"]) != len(d.get("prep", [])) or len(sc.get("after", [])) < len(d.get("after", [])):
            wrong.append("%s: no trainee script for every step" % sid)
    if wrong:
        report("AC31 scenarios on the real simulator", False, "; ".join(wrong))
        return
    # the real simulator: every scenario with the right reaction, then the wrong ones (two simulators at once)
    good = sorted(docs, key=lambda i: (i == "s02_gps_spoof", i))       # the spoof last: its GPS recovery is slow
    jobs = [[(i, "good") for i in good[0::2]], [(i, "good") for i in good[1::2]]]
    jobs[0] += [("s01_compass_fail", "none"), ("s01_compass_fail", "retry")]
    jobs[1] += [("s08_swimmer_kayak", "none"), ("s06_ais_collision", "port")]
    out, times = {}, {}

    def worker(job):
        sp = _TestSim()
        try:
            sim = scenario.Sim(sp.link())
            for sid, how in job:
                t0 = time.time()
                r = scenario.Run(docs[sid], "en")
                seen = set()
                if how == "good":
                    st = _drive_scen(sim, r, SCENARIO_SCRIPTS[sid], on_tick=lambda s: seen.add(s))
                elif how == "none":
                    st = _drive_scen(sim, r, SCENARIO_SCRIPTS[sid], react=False)
                elif how == "port":
                    st = _drive_scen(sim, r, SCENARIO_SCRIPTS[sid], react_override=[_sb("minus10")] * 3)
                else:                                       # a missed reaction, then the retry passes (90)
                    st = _drive_scen(sim, r, SCENARIO_SCRIPTS[sid], react=False)
                    _scen_clean(sim)
                    for a in r.retry(time.time()):
                        sim.do(a)
                    st = _drive_scen(sim, r, SCENARIO_SCRIPTS[sid], start=False)
                out[(sid, how)] = (st, r)
                times["%s/%s" % (sid, how)] = round(time.time() - t0)
                _scen_clean(sim)
            sim.link.close()
        except Exception as e:                              # reported below
            out[("crash", repr(e))] = (None, None)
        finally:
            sp.stop()
    th = [threading.Thread(target=worker, args=(j,)) for j in jobs]
    for t in th:
        t.start()
    for t in th:
        t.join(600)
    for k, (st, r) in out.items():
        if k[0] == "crash":
            wrong.append("crashed: %s" % k[1])
    for sid in good:
        st, r = out.get((sid, "good"), (None, None))
        if r is None or st != "done" or r.score() != 100 or r.reaction_s() is None:
            wrong.append("%s: %s (%s, score %s)" % (sid, st, r and r.why, r and r.score()))
    # the simulated AURIX's own reactions to the simulator faults (seen in the recorded timeline)
    def alm(sid, bit):
        st, r = out.get((sid, "good"), (None, None))
        return r is not None and any(s.get("alm", 0) & bit for s in r.timeline)

    def modes(sid):
        st, r = out.get((sid, "good"), (None, None))
        return [] if r is None else [s.get("mode") for s in r.timeline]
    for sid, bit, what in (("s01_compass_fail", 0x1, "COMPASS LOST"), ("s02_gps_spoof", 0x4, "GPS SUSPECT"),
                           ("s03_slamming", 0x200000, "SLAMMING"), ("s04_joystick_fault", 0x100, "JOYSTICK FAULT"),
                           ("s05_power_refusal", 0x2, "GPS LOST")):
        if not alm(sid, bit):
            wrong.append("%s: the simulator never showed %s" % (sid, what))
    st, r = out.get(("s01_compass_fail", "good"), (None, None))
    if r is not None:                                       # compass lost in HDG LOCK: STANDBY only once at idle
        first = next((s for s in r.timeline if s.get("mode") == "STANDBY" and s.get("t", 0) > (r.t_event - r.t0)), None)
        if first is None or first.get("thr", 99) > 1.0:
            wrong.append("compass lost: STANDBY not at idle (%s)" % first)
    st, r = out.get(("s04_joystick_fault", "good"), (None, None))
    if r is not None and "STANDBY" not in modes("s04_joystick_fault")[len(modes("s04_joystick_fault")) // 3:]:
        wrong.append("joystick fault did not end in STANDBY")
    for k, why in ((("s01_compass_fail", "none"), ("late",)), (("s08_swimmer_kayak", "none"), ("late", "outcome")),
                   (("s06_ais_collision", "port"), ("forbid",))):
        st, r = out.get(k, (None, None))
        if r is None or st != "failed" or r.why not in why:
            wrong.append("%s/%s should fail (%s): %s %s" % (k[0], k[1], "/".join(why), st, r and r.why))
    st, r = out.get(("s01_compass_fail", "retry"), (None, None))
    if r is None or st != "done" or r.score() != 90 or r.retries != 1:
        wrong.append("compass retry: %s score %s" % (st, r and r.score()))
    report("AC31 scenarios on the real simulator", not wrong, "; ".join(wrong) or
           "every scenario passed by its right reaction (%s s); the simulated AURIX showed COMPASS LOST (STANDBY only at "
           "idle), GPS SUSPECT, SLAMMING, JOYSTICK FAULT (STANDBY), GPS LOST (raise refused); no reaction = late, "
           "run-over swimmer = failed, a port turn at a starboard ship = forbidden; the retry scores 90"
           % ", ".join("%s %s" % kv for kv in sorted(times.items())))


# --- AC32 -------------------------------------------------------------------------------------
def ac32():
    """The power-limit refusal on the REAL simulator: with GPS LOST the code request is refused (the
    station's NACK), the raise works only after the cause is gone + acknowledge + code + CONFIRM - the
    scenario adds no way around any rule (the same frames as the station's own)."""
    import time
    import scenario
    import proto
    wrong = []
    doc = next(d for d in scenario.scenarios() if d["id"] == "s05_power_refusal")
    sp = _TestSim()
    try:
        sim = scenario.Sim(sp.link())
        r = scenario.Run(doc, "en")
        for a in r.start(time.time()):
            sim.do(a)
        asked = False
        end = time.time() + 30
        pressed = False
        while time.time() < end and r.status != "react":
            for a in r.update(sim.tick(), time.time())[1]:
                sim.do(a)
            if r.status == "prep" and not pressed:
                time.sleep(0.5)
                sim.button("hdg_lock")
                pressed = True
            time.sleep(0.05)
        if r.status == "react":
            n0 = len(sim.nacks())
            sim.power_code()
            for _ in range(20):
                for a in r.update(sim.tick(), time.time())[1]:
                    sim.do(a)
                time.sleep(0.05)
            asked = True
            nk = sim.nacks()[n0:]
            if not any(a.get("cmd") == proto.T_POWER_CODE for a in nk) or sim.state.get("power_code"):
                wrong.append("the code was not refused with GPS LOST (%s)" % nk)
            # CONFIRM alone raises nothing
            _sconfirm(sim)
            for _ in range(10):
                for a in r.update(sim.tick(), time.time())[1]:
                    sim.do(a)
                time.sleep(0.05)
            if sim.state.get("power_fwd", 0) > 30:
                wrong.append("CONFIRM alone raised the power")
        if not asked:
            wrong.append("the emergency never started (%s)" % r.status)
        st = _drive_scen(sim, r, SCENARIO_SCRIPTS["s05_power_refusal"], start=False, limit=90)
        if st != "done" or sim.state.get("power_fwd", 0) < 50:
            wrong.append("the proper raise after the fix: %s %s" % (st, sim.state.get("power_fwd")))
        sim.power_lower()
        _scen_clean(sim)
        sim.link.close()
    finally:
        sp.stop()
    report("AC32 power refusal on the real simulator", not wrong, "; ".join(wrong) or
           "HDG LOCK + GPS LOST: the code request refused by the simulated AURIX, CONFIRM alone raises nothing; after the GPS + "
           "acknowledge the code + CONFIRM raise to 50 %")


# --- AC33 -------------------------------------------------------------------------------------
def ac33():
    import banner
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as e:
        report("AC33 scenario window + replay", True, "no display here (%s): models checked by AC30-AC32" % e)
        return
    import time
    import academy_app
    import scenario
    import scenario_app
    wrong = []
    sim_p = None
    try:
        root.withdraw()
        store = _store()
        store.add("ann", "Ann")
        rep = tempfile.mkdtemp(prefix="trygons_ac33_")
        top = tk.Toplevel(root)
        w = scenario_app.ScenarioWindow(top, "en", store=store, user="ann", link_factory=lambda: (_ for _ in ()).throw(OSError()),
                                        replay_folder=rep)
        top.update()
        if top.pack_slaves()[0] is not w.banner or w.banner.cget("text") != banner.banner_text("en"):
            wrong.append("scenario window: banner not first")
        heads = [d for d in w.rows if d is None]
        if len(heads) < 2 or w.rows[0] is not None or len([d for d in w.rows if d]) != len(scenario.scenarios()):
            wrong.append("scenarios not listed by level")
        w.select("s09_gateway_lost")
        if w.start_selected() is not None or w.conn.cget("text") != scenario_app.T("lesson.no_sim", "en"):
            wrong.append("no simulator not said")
        # the picture: an AIS target on the map + its row; a swimmer only on the camera; gateway lost = no camera
        run = scenario.Run(next(d for d in scenario.scenarios() if d["id"] == "s06_ais_collision"), "en")
        run.origin, run.t_event = (37.8, 23.6), 0.0
        run.hazard = scenario.Hazard(run.ev["hazard"], 90.0, 5.0)
        st = {"est_lat": 37.8, "est_lon": 23.6, "heading": 90.0, "est_sog": 5.0}
        w.draw_picture(run.picture(st, 10.0), run)
        if not w.map.find_withtag("ais") or "CPA" not in w.l_ais.cget("text"):
            wrong.append("AIS target not drawn / listed")
        run2 = scenario.Run(next(d for d in scenario.scenarios() if d["id"] == "s08_swimmer_kayak"), "en")
        run2.origin, run2.t_event = (37.8, 23.6), 0.0
        run2.hazard = scenario.Hazard(run2.ev["hazard"], 90.0, 5.0)
        w.draw_picture(run2.picture(st, 1.0), run2)
        if not w.cam.find_withtag("cam_swimmer") or "CPA" in w.l_ais.cget("text"):
            wrong.append("swimmer: not on the camera or on the AIS list")
        run3 = scenario.Run(next(d for d in scenario.scenarios() if d["id"] == "s09_gateway_lost"), "en")
        run3.origin, run3.shown = (37.8, 23.6), ["gateway_lost"]
        w.draw_picture(run3.picture(st, 1.0), run3)
        if scenario_app.T("scen.no_ais", "en") != w.l_ais.cget("text"):
            wrong.append("gateway lost: the AIS list still shown")
        w.close()
        # a scenario through the window on the simulator -> logbook + replay file + debrief + replay window
        sim_p = _TestSim()
        top = tk.Toplevel(root)
        w = scenario_app.ScenarioWindow(top, "el", store=store, user="ann", link_factory=sim_p.link, replay_folder=rep)
        w.select("s09_gateway_lost")
        if w.start_selected() is None:
            wrong.append("scenario did not start")
        else:
            acted = set()
            entered = {}
            end = time.time() + 60
            while time.time() < end and w.status not in ("done", "failed"):
                w.step()
                top.update()
                key = (w.run.status, w.run.index)
                entered.setdefault(key, time.time())
                if key not in acted and time.time() - entered[key] > 0.8:
                    if key == ("prep", 0):
                        acted.add(key)
                        w.button("hdg_lock")
                    elif key == ("prep", 1):
                        acted.add(key)
                        w.hold.set(True)
                        w.stick(0.0, 1.0)
                    elif key[0] == "react":
                        acted.add(key)
                        if "GATEWAY" not in w.l_state.cget("text").upper() and "ΠΥΛΗ" not in w.l_state.cget("text").upper():
                            wrong.append("the shown GATEWAY LOST is not in the alarms (%r)" % w.l_state.cget("text"))
                        w.stick(0.0, 0.0)
                time.sleep(0.05)
            w.hold.set(False)
            w._hold_changed()
            store.load()
            ok = [e for e in store.entries() if e.get("kind") == "scenario" and e.get("id") == "s09_gateway_lost"
                  and e.get("user") == "ann" and e.get("done") and e.get("reaction_s") is not None]
            row = [w.list.get(i) for i, d in enumerate(w.rows) if d and d["id"] == "s09_gateway_lost"]
            if w.status != "done" or not ok or not row or not row[0].startswith("✓"):
                wrong.append("window scenario: %s, logged %d, row %s" % (w.status, len(ok), row))
            files = [f for f in os.listdir(rep) if f.endswith(".jsonl")]
            if not files or not ok or ok[-1].get("replay") not in files:
                wrong.append("replay file %s / logbook %s" % (files, ok and ok[-1].get("replay")))
            if not w.debrief.cget("text").strip() or w.run.doc["right_el"] not in w.debrief.cget("text"):
                wrong.append("no debrief")
            rw = w.open_replay()
            if rw is None:
                wrong.append("replay window did not open")
            else:
                rw.top.update()
                if rw.top.pack_slaves()[0] is not rw.banner or rw.banner.cget("text") != banner.banner_text("el"):
                    wrong.append("replay window: banner not first")
                rw.pos.set(len(rw.samples) - 1)
                rw.show(len(rw.samples) - 1)
                if not rw.canvas.find_withtag("boat") or not rw.canvas.find_withtag("track"):
                    wrong.append("replay: boat / track not drawn")
                rw.play()
                rw.close()
            if w.open_replay("COM4") is not None or not w.l_result.cget("text"):
                wrong.append("a replay from COM4 opened")
        w.close()
        sim_p.stop()
        sim_p = None
        # the academy window's button opens it
        top = tk.Toplevel(root)
        app = academy_app.AcademyApp(top, store)
        sw = app.open_scenarios()
        if not isinstance(sw, scenario_app.ScenarioWindow) or "acc.scenarios" not in app.buttons:
            wrong.append("academy window button")
        sw.close()
        top.destroy()
    finally:
        if sim_p is not None:
            sim_p.stop()
        root.destroy()
    report("AC33 scenario window + replay", not wrong, "; ".join(wrong) or
           "banner first; scenarios by level; no simulator said; AIS on the map + list, swimmer only on the camera, "
           "gateway lost = no AIS / camera; a scenario through the window on the simulator: shown alarm, logbook with "
           "reaction time + replay file, debrief; the replay window (banner, track, slider, play) via simguard, COM4 "
           "refused; the academy window's button opens it")



# --- step 20: real-trip video lessons ------------------------------------------------------------
VL_T0 = 1792914000                      # 2026-10-25 07:40:00 UTC = the first video file of the test event


def _ppm_pixels(data):
    """(w, h, bytes) of a binary PPM."""
    parts = data.split(b"\n", 3)
    w, h = (int(v) for v in parts[1].split())
    return w, h, parts[3]


def _roughness(data, box):
    """Mean absolute difference of neighbouring pixels (all channels) inside box (parts 0..1): blur = low."""
    w, h, px = _ppm_pixels(data)
    x0, y0, x1, y1 = int(box[0] * w), int(box[1] * h), int(box[2] * w), int(box[3] * h)
    tot = n = 0
    for y in range(y0, y1, 2):
        row = y * w * 3
        for x in range(x0, x1 - 1, 3):                                 # odd step: crosses the 4 px cell edges
            for c in range(3):
                tot += abs(px[row + x * 3 + c] - px[row + (x + 1) * 3 + c])
                n += 1
    return tot / float(max(n, 1))


def _make_event(tmp, video=True):
    """A saved event as the gateway writes it + its AURIX log (TIME events: GPS time) under a sync root.
    video = real 1-minute .ts files made by ffmpeg (a game-of-life picture), else empty placeholders."""
    import json
    import struct
    import time as _t
    import simguard
    import videolesson  # noqa: F401  (puts groundstation/ on the path)
    import tlg
    root = os.path.join(tmp, "sync", "boat")
    ev = os.path.join(root, "video", "events", "20261025-074100_alarm")
    ff = simguard.ffmpeg_exe()
    life = ("life=s=160x90:mold=10:rate=5:ratio=0.5:seed=%d:death_color=#203040:life_color=#f0e0a0,"
            "scale=640:360:flags=neighbor")                            # fine detail everywhere: blur is measurable
    for cam, src in (("FRONT-1", life % 7), ("REAR-LEFT", life % 11)):
        os.makedirs(os.path.join(ev, cam))
        for k in range(2):
            n = _t.strftime("%Y%m%d-%H%M%S", _t.gmtime(VL_T0 + 60 * k)) + ".ts"
            out = os.path.join(ev, cam, n)
            if video:
                subprocess.run([ff, "-loglevel", "error", "-y", "-f", "lavfi", "-i", src,
                                "-t", "60", "-c:v", "libx264", "-preset", "ultrafast", "-g", "5", "-f", "mpegts", out],
                               check=True, timeout=120)
            else:
                open(out, "wb").close()
    lp = os.path.join(root, "aurix_logs", "20261025", "P00001_001.TLG")
    os.makedirs(os.path.dirname(lp))
    hdr = ("fields=heading:H:100,target:H:100,mode:B:1,thr_out:H:100,est_sog:H:100,alarm_bits:I:1\n"
           "events=16:TIME:date,tod_ms,t_ms,-\nmodes=STANDBY,MANUAL,HDG LOCK\n")
    boot = 1000000
    with open(lp, "wb") as f:
        f.write(b"TLG1" + tlg.record(1, 0, hdr.encode()))
        f.write(tlg.record(0x20, boot, struct.pack("<Hiiii", 16, 20261025, (7 * 3600 + 40 * 60) * 1000, boot, 0)))
        for i in range(120 * 20):                                       # 20 Hz: heading 90 + 0.2 deg/s
            f.write(tlg.record(0x10, boot + i * 50, struct.pack("<HHBHHI", int(round((90 + i * 0.01) * 100)), 9000,
                                                                  2 if i > 600 else 0, 1500, 500,
                                                                  1 if i > 1300 else 0)))
    with open(os.path.join(ev, "event.json"), "w", encoding="utf-8") as f:
        json.dump({"id": "20261025-074100_alarm", "state": "complete", "start_t": VL_T0, "end_t": VL_T0 + 120,
                   "segment_s": 60, "triggers": [{"t": VL_T0 + 60, "kind": "alarm", "text": "alarm GPS LOST"}],
                   "boat": {"log": {"file": "aurix_logs\\20261025\\P00001_001.TLG"}}}, f)
    return os.path.join(ev, "event.json")


def _vl_doc():
    return {"_help": "test", "kind": "videolesson", "id": "v_test", "level": "DRIVER", "runs_on": "recording",
            "title_el": "Δοκιμή", "title_en": "Test", "intro_el": "εισαγωγή", "intro_en": "intro",
            "clip": {"start_utc": 1000.0, "end_utc": 1030.0}, "cameras": ["FRONT-1", "REAR-LEFT"],
            "main_camera": "FRONT-1", "media": {"FRONT-1": "FRONT-1.mp4", "REAR-LEFT": "REAR-LEFT.mp4"},
            "timeline": "hud.jsonl",
            "blur": [{"camera": "FRONT-1", "x": 0.5, "y": 0.5, "w": 0.2, "h": 0.2, "from_s": 1.0, "to_s": 9.0}],
            "points": [
                {"t_s": 5.0, "type": "decision", "q_el": "τι;", "q_en": "what?", "why_el": "γιατί", "why_en": "why",
                 "choices": [{"text_el": "α", "text_en": "a"}, {"text_el": "β", "text_en": "b"}], "answer": 1},
                {"t_s": 12.0, "type": "hazard", "camera": "REAR-LEFT", "q_el": "πού;", "q_en": "where?",
                 "why_el": "εκεί", "why_en": "there", "areas": [{"x": 0.1, "y": 0.2, "w": 0.3, "h": 0.3}]}]}


# --- AC34 -------------------------------------------------------------------------------------
def ac34():
    import copy
    import videolesson as vl
    wrong = []
    good = _vl_doc()
    if vl.check_doc(good):
        wrong.append("good lesson refused: %s" % vl.check_doc(good))
    bad = []
    for why, fn in (("clip too long", lambda d: d["clip"].update(end_utc=1000.0 + vl.MAX_CLIP_S + 1)),
                    ("clip too short", lambda d: d["clip"].update(end_utc=1001.0)),
                    ("point outside", lambda d: d["points"][1].update(t_s=31.0)),
                    ("points out of order", lambda d: d["points"][0].update(t_s=13.0)),
                    ("bad area", lambda d: d["points"][1]["areas"][0].update(x=0.9, w=0.3)),
                    ("no area", lambda d: d["points"][1].update(areas=[])),
                    ("hazard camera unknown", lambda d: d["points"][1].update(camera="MAST")),
                    ("answer out of range", lambda d: d["points"][0].update(answer=2)),
                    ("bad blur", lambda d: d["blur"][0].update(to_s=40.0)),
                    ("media path out", lambda d: d["media"].update({"FRONT-1": "../x.mp4"})),
                    ("main camera", lambda d: d.update(main_camera="MAST")),
                    ("runs_on", lambda d: d.update(runs_on="boat")),
                    ("no English", lambda d: d["points"][0].pop("q_en")),
                    ("bad id", lambda d: d.update(id="a/b"))):
        d = copy.deepcopy(good)
        fn(d)
        if not vl.check_doc(d):
            bad.append(why)
    if bad:
        wrong.append("checker missed: %s" % bad)
    if not vl.check_doc(good, tempfile.mkdtemp()):
        wrong.append("missing media files not seen")
    # the player model
    r = vl.Run(good, "en")
    if r.advance(3.0) is not None or r.t != 3.0:
        wrong.append("stopped before the first point")
    if r.advance(6.0) != 0 or r.t != 5.0 or r.point()["q_en"] != "what?":
        wrong.append("no stop at the decision point")
    if r.advance(20.0) is not None or r.t != 5.0:
        wrong.append("the clock ran on while asking")
    if r.click("REAR-LEFT", 0.2, 0.3) is not None:
        wrong.append("a click answered a decision")
    if r.answer_choice(1) is not True or r.explanation() != "why" or r.answer_choice(0) is not None:
        wrong.append("decision answer / one answer per point / explanation")
    r.resume()
    if r.advance(12.5) != 1:
        wrong.append("no stop at the hazard point")
    if r.click("FRONT-1", 0.2, 0.3) is not False:
        wrong.append("a click on the wrong camera counted")
    r.resume()
    r.advance(40.0)
    e = r.log_entry()
    if not r.finished or e["score"] != 50 or e["done"] or e["right"] != 1 or e["of"] != 2:
        wrong.append("score / done: %s finished %s" % (e, r.finished))
    r2 = vl.Run(good, "el")
    r2.advance(5.0)
    r2.answer_choice(1)
    r2.resume()
    r2.advance(12.0)
    if r2.click("REAR-LEFT", 0.25, 0.35) is not True or r2.explanation() != "εκεί":
        wrong.append("hazard click inside the area / Greek explanation")
    r2.resume()
    r2.advance(30.0)
    e2 = r2.log_entry()
    if not e2["done"] or e2["score"] != 100 or e2["id"] != "v_test":
        wrong.append("full marks not passed: %s" % e2)
    # HUD timeline from a log (UTC from its TIME events) + the HUD state
    tmp = tempfile.mkdtemp(prefix="trygons_ac34_")
    evp = _make_event(tmp, video=False)
    ev = vl.Event(evp)
    log = vl.read_log(ev.log_path())
    tl = vl.hud_timeline(log, VL_T0 + 50.0, VL_T0 + 60.0)
    if len(tl) != 51 or abs(tl[0]["d"]["heading"] - 100.0) > 0.02 or abs(tl[-1]["d"]["heading"] - 102.0) > 0.02:
        wrong.append("HUD timeline not in sync: %d samples, %s .. %s" % (len(tl), tl and tl[0], tl and tl[-1]))
    st = vl.hud_state(tl[0]["d"]) if tl else {}
    import proto
    if st.get("mode_name") != proto.MODES[2] or st.get("throttle") != 15.0 or "power_fwd" not in st:
        wrong.append("HUD state: %s %s" % (st.get("mode_name"), st.get("throttle")))
    if tl and vl.at_time(tl, 3.33)["heading"] != tl[16]["d"]["heading"]:
        wrong.append("at_time")
    # the editor model
    if ev.cameras() != ["FRONT-1", "REAR-LEFT"] or abs(ev.end_utc - ev.start_utc - 120.0) > 0.01:
        wrong.append("event: %s %s" % (ev.cameras(), ev.end_utc - ev.start_utc))
    d = vl.Draft(ev)
    if (d.start, d.end) != (30.0, 90.0):
        wrong.append("default trim around the trigger: %s" % ((d.start, d.end),))
    for a, b in ((10.0, 11.0), (20.0, 20.5)):
        try:
            d.trim(a, b)
            wrong.append("trim %s..%s allowed" % (a, b))
        except ValueError:
            pass
    d.trim(50.0, 80.0)
    d.add_decision(60.0, {"el": "τι;", "en": "what?"}, [{"el": "α", "en": "a"}, {"el": "β", "en": "b"}], 0,
                   {"el": "γιατί", "en": "why"})
    d.add_hazard(95.0, "FRONT-1", {"el": "πού;", "en": "where?"}, {"el": "εκεί", "en": "there"},
                 [{"x": 0.1, "y": 0.1, "w": 0.2, "h": 0.2}])
    d.add_blur("FRONT-1", {"x": 0.5, "y": 0.5, "w": 0.2, "h": 0.2}, 40.0, 70.0)
    try:
        d.set_cameras([])
        wrong.append("no camera allowed")
    except ValueError:
        pass
    d.set_cameras(["REAR-LEFT", "FRONT-1"], "REAR-LEFT")
    doc = d.to_doc()
    if (doc["clip"]["start_utc"] != VL_T0 + 50.0 or len(doc["points"]) != 1 or doc["points"][0]["t_s"] != 10.0 or
            doc["blur"][0]["from_s"] != 0.0 or doc["blur"][0]["to_s"] != 20.0 or doc["main_camera"] != "REAR-LEFT"):
        wrong.append("draft -> doc times: %s %s %s" % (doc["clip"], doc["points"], doc["blur"]))
    if [p for p in vl.check_doc(doc) if "missing" not in p]:
        wrong.append("the editor's doc fails the checker: %s" % vl.check_doc(doc))
    txt = os.path.join(tmp, "notes.txt")
    open(txt, "w").close()
    for p in ("COM4", txt, "rtsp://10.0.0.5/event.json"):
        try:
            vl.Event(p)
            wrong.append("event %s opened" % p)
        except Exception:
            pass
    report("AC34 video lessons: checker, player, HUD timeline, editor model", not wrong, "; ".join(wrong) or
           "bad lessons caught (14 cases + missing media); pause points stop the clock, one answer per point, "
           "decision + hazard click, explanation el / en, score 50 / 100, logbook entry; HUD timeline in sync from "
           "the log's GPS time; editor trim limits + event -> clip times; COM4 / a text file / rtsp refused as event")


# --- AC35 -------------------------------------------------------------------------------------
def ac35():
    import simguard
    import videolesson as vl
    wrong = []
    tmp = tempfile.mkdtemp(prefix="trygons_ac35_")
    data = os.path.join(tmp, "data")
    env = {"TRYGONS_ACADEMY_DATA": data, "PROGRAMDATA": tempfile.mkdtemp()}
    seg = os.path.join(tmp, "20261025-074000.ts")
    open(seg, "wb").close()
    folder = os.path.join(tmp, "x")
    os.makedirs(folder)
    if simguard.ffmpeg_exe():
        cmd = simguard.frame_command(seg, 2.0, 320, env=env)
        if cmd[cmd.index("-protocol_whitelist") + 1] != "file" or cmd.index("-protocol_whitelist") > cmd.index("-i"):
            wrong.append("frame: no whitelist before the input")
        cc = simguard.copy_command("l.txt", 1.0, 5.0, "o.mp4", 640, [{"x": 0.1, "y": 0.1, "w": 0.2, "h": 0.2}])
        if cc[cc.index("-protocol_whitelist") + 1] != "file" or cc.index("-protocol_whitelist") > cc.index("-i") or \
                "boxblur" not in cc[cc.index("-filter_complex") + 1] or "-an" not in cc:
            wrong.append("copy: whitelist / blur / no sound missing")
    for p in ("rtsp://10.0.0.5/cam", "http://x/y.mp4", "COM3", "tcp:127.0.0.1:5760", folder,
              os.path.join(tmp, "a.exe"), os.path.join(tmp, "nothere.ts")):
        try:
            simguard.check_video(p, env)
            wrong.append("video %s allowed" % p)
        except simguard.Refused:
            pass
    calls = []

    class R(object):
        returncode, stderr = 0, b""

    def fake_run(cmd, **kw):
        calls.append(cmd)
        lst = cmd[cmd.index("-i") + 1]
        if lst.endswith(".txt"):
            with open(lst, encoding="utf-8") as f:
                calls.append(f.read())
        with open(cmd[-1], "wb") as f:
            f.write(b"x")
        return R()
    lessons = simguard.videolessons_dir(env)
    out_ok = os.path.join(lessons, "v1", "FRONT-1.mp4")
    try:
        simguard.video_copy([seg], 1.0, 5.0, out_ok, run=fake_run, env=env)
    except Exception as e:
        wrong.append("a good copy refused: %s" % e)
    if len(calls) < 2 or "file '%s'" % os.path.abspath(seg).replace("\\", "/") not in calls[1] or \
            os.path.exists(out_ok[:-4] + "_list.txt"):
        wrong.append("concat list: %s" % calls[1:2])
    for out in (os.path.join(tmp, "elsewhere.mp4"), os.path.join(lessons, "v1", "x.avi"),
                os.path.join(lessons, "..", "escape.mp4")):
        try:
            simguard.video_copy([seg], 1.0, 5.0, out, run=fake_run, env=env)
            wrong.append("copy to %s allowed" % out)
        except simguard.Refused:
            pass
    for segs in ([], ["rtsp://10.0.0.5/cam"], ["COM3"]):
        try:
            simguard.video_copy(segs, 1.0, 5.0, out_ok, run=fake_run, env=env)
            wrong.append("copy from %s allowed" % segs)
        except simguard.Refused:
            pass
    genv = dict(gateway_pc(), TRYGONS_ACADEMY_DATA=data)
    for fn in (lambda: simguard.video_copy([seg], 1.0, 5.0, out_ok, run=fake_run, env=genv),
               lambda: simguard.video_frame(seg, 1.0, run=fake_run, env=genv)):
        try:
            fn()
            wrong.append("ffmpeg on a gateway PC")
        except simguard.Refused:
            pass
    # the player opens only the lesson's own copies
    doc = _vl_doc()
    own = os.path.join(lessons, "v_test")
    os.makedirs(own, exist_ok=True)
    open(os.path.join(own, "FRONT-1.mp4"), "wb").close()
    try:
        if vl.media_path(doc, own, "FRONT-1", env) != os.path.abspath(os.path.join(own, "FRONT-1.mp4")):
            wrong.append("own copy path")
    except simguard.Refused as e:
        wrong.append("own copy refused: %s" % e)
    for d2, f2, cam in ((dict(doc, media={"FRONT-1": "../../x.mp4"}), own, "FRONT-1"),
                        (doc, os.path.dirname(seg), "FRONT-1"), (doc, own, "MAST")):
        try:
            vl.media_path(d2, f2, cam, env)
            wrong.append("player opened %s / %s" % (f2, cam))
        except simguard.Refused:
            pass
    report("AC35 simguard: ffmpeg only on recorded files", not wrong, "; ".join(wrong) or
           "-protocol_whitelist file before every input; URLs / COM / tcp / folders / other types refused; copies "
           "only .mp4 inside the video-lesson folder; the concat list = checked files only, removed; nothing on a "
           "gateway PC; the player opens only the lesson's own copies")


# --- AC36 -------------------------------------------------------------------------------------
def ac36():
    import simguard
    import videolesson as vl
    if not simguard.ffmpeg_exe():
        report("AC36 editor on real ffmpeg", True, "no ffmpeg on this PC: the model is checked by AC34 / AC35")
        return
    wrong = []
    nums = ""
    tmp = tempfile.mkdtemp(prefix="trygons_ac36_")
    old = os.environ.get("TRYGONS_ACADEMY_DATA")
    os.environ["TRYGONS_ACADEMY_DATA"] = os.path.join(tmp, "data")
    try:
        ev = vl.Event(_make_event(tmp))
        d = vl.Draft(ev, "v_ac36")
        d.trim(50.0, 75.0)                                              # across the two 1-minute files
        d.title = {"el": "Δοκιμή", "en": "Test"}
        d.add_decision(55.0, {"el": "τι;", "en": "what?"}, [{"el": "α", "en": "a"}, {"el": "β", "en": "b"}], 1,
                       {"el": "γιατί", "en": "why"})
        d.add_blur("FRONT-1", {"x": 0.5, "y": 0.5, "w": 0.3, "h": 0.3}, 50.0, 75.0)
        folder, doc, notes = d.make()
        probs = vl.check_doc(doc, folder)
        if probs or notes:
            wrong.append("made lesson: %s %s" % (probs, notes))
        orig = sum(os.path.getsize(p) for p in ev.segments("FRONT-1", ev.start_utc + 50, ev.start_utc + 75)[0])
        small = os.path.getsize(os.path.join(folder, "FRONT-1.mp4"))
        if not 0 < small < orig:
            wrong.append("copy not smaller: %d vs %d" % (small, orig))
        f_end = simguard.video_frame(os.path.join(folder, "FRONT-1.mp4"), 24.0, 640)
        f_after = simguard.video_frame(os.path.join(folder, "FRONT-1.mp4"), 27.0, 640)
        if not f_end or f_after:
            wrong.append("copy length: a frame at 24 s %s, at 27 s %s (clip = 25 s)" % (bool(f_end), bool(f_after)))
        cp = simguard.video_frame(os.path.join(folder, "FRONT-1.mp4"), 5.0, 640)
        src, off = ev.locate("FRONT-1", ev.start_utc + 55.0)
        og = simguard.video_frame(src, off, 640)
        rear = simguard.video_frame(os.path.join(folder, "REAR-LEFT.mp4"), 5.0, 640)
        rsrc, roff = ev.locate("REAR-LEFT", ev.start_utc + 55.0)
        rog = simguard.video_frame(rsrc, roff, 640)
        if not (cp and og and rear and rog):
            wrong.append("frames missing")
        else:
            inner = (0.55, 0.55, 0.75, 0.75)
            outer = (0.05, 0.05, 0.40, 0.40)
            rb, ro = _roughness(cp, inner), _roughness(og, inner)
            ob, oo = _roughness(cp, outer), _roughness(og, outer)
            nums = " (roughness inside %.1f vs %.1f, outside %.1f vs %.1f)" % (rb, ro, ob, oo)
            if not rb < 0.5 * ro or not ob > 0.6 * oo:
                wrong.append("blur: inside %.1f vs original %.1f, outside %.1f vs %.1f" % (rb, ro, ob, oo))
            if _roughness(rear, inner) < 0.6 * _roughness(rog, inner):
                wrong.append("the rear camera (no blur box) blurred too")
        tl = vl.read_timeline(doc, folder)
        if len(tl) != 126 or abs(tl[0]["d"]["heading"] - 100.0) > 0.02 or not doc.get("hud"):
            wrong.append("HUD timeline: %d, %s" % (len(tl), tl and tl[0]))
        if not [x for x in vl.load_lessons() if x[0].get("id") == "v_ac36" and not x[2]]:
            wrong.append("lesson not listed")
    finally:
        if old is None:
            os.environ.pop("TRYGONS_ACADEMY_DATA", None)
        else:
            os.environ["TRYGONS_ACADEMY_DATA"] = old
    report("AC36 editor on real ffmpeg", not wrong, "; ".join(wrong) or
           "event (2 cameras x 2 files) + log -> 25 s copies across the file boundary, smaller than the originals, "
           "the blur box blurred and the rest sharp%s, HUD timeline 126 samples in sync; listed + checked" % nums)


# --- AC37 -------------------------------------------------------------------------------------
def ac37():
    import banner
    import simguard
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as e:
        report("AC37 video lesson windows", True, "no display here (%s): models checked by AC34-AC36" % e)
        return
    if not simguard.ffmpeg_exe():
        root.destroy()
        report("AC37 video lesson windows", True, "no ffmpeg on this PC: models checked by AC34 / AC35")
        return
    import time
    import academy_app
    import videolesson_app as va
    wrong = []
    tmp = tempfile.mkdtemp(prefix="trygons_ac37_")
    old = os.environ.get("TRYGONS_ACADEMY_DATA")
    os.environ["TRYGONS_ACADEMY_DATA"] = os.path.join(tmp, "data")
    try:
        root.withdraw()
        store = _store()
        store.add("ann", "Ann")
        evp = _make_event(tmp)
        top = tk.Toplevel(root)
        ed = va.EditorWindow(top, "en")
        top.update()
        if top.pack_slaves()[0] is not ed.banner or ed.banner.cget("text") != banner.banner_text("en"):
            wrong.append("editor: banner not first")
        if ed.open_event("COM4") is not None:
            wrong.append("COM4 opened as an event")
        if ed.open_event(evp) is None:
            wrong.append("event not opened")
        else:
            ed.pos.set(50.0)
            ed.set_trim("start")
            ed.pos.set(80.0)
            ed.set_trim("end")
            if (ed.draft.start, ed.draft.end) != (50.0, 80.0):
                wrong.append("trim: %s" % ((ed.draft.start, ed.draft.end),))
            for k, v in (("q_el", "Τι κάνετε;"), ("q_en", "What do you do?"), ("choices_el", "Τίποτα;STANDBY"),
                         ("choices_en", "Nothing;STANDBY"), ("answer", "2"), ("why_el", "Το STANDBY κερδίζει."),
                         ("why_en", "STANDBY wins.")):
                ed.qv[k].set(v)
            ed.pos.set(55.0)
            if ed.add_decision() is None:
                wrong.append("decision not added")
            ed.pos.set(62.0)
            ed.pcam.set("REAR-LEFT")
            ed.draw_mode.set("hazard")
            if ed.add_hazard() is not None:
                wrong.append("a hazard question without an area")
            ed.add_box(0.1, 0.2, 0.4, 0.5)
            for k, v in (("q_el", "Πού είναι ο κίνδυνος;"), ("q_en", "Where is the hazard?"),
                         ("why_el", "Το σκάφος πίσω αριστερά."), ("why_en", "The boat astern to port.")):
                ed.qv[k].set(v)
            if ed.add_hazard() is None:
                wrong.append("hazard not added")
            ed.pcam.set("FRONT-1")
            ed.draw_mode.set("blur")
            ed.blur_s.set("8")
            ed.add_box(0.5, 0.5, 0.8, 0.8)
            ed.v_tel.set("Δοκιμαστικό μάθημα")
            ed.v_ten.set("Test lesson")
            ed.v_id.set("v_ac37")
            if len(ed.draft.points) != 2 or len(ed.draft.blur) != 1 or ed.l_points.size() != 2:
                wrong.append("points %d blur %d" % (len(ed.draft.points), len(ed.draft.blur)))
            ed.make(wait=True)
            if "v_ac37" not in ed.msg.cget("text"):
                wrong.append("make: %s" % ed.msg.cget("text"))
        ed.close()
        # the player
        top = tk.Toplevel(root)
        clock = [1000.0]
        pl = va.VideoLessonWindow(top, "el", store=store, user="ann", clock=lambda: clock[0])
        top.update()
        if top.pack_slaves()[0] is not pl.banner or pl.banner.cget("text") != banner.banner_text("el"):
            wrong.append("player: banner not first")
        if not pl.select("v_ac37"):
            wrong.append("lesson not listed")
        elif pl.start_selected() is None:
            wrong.append("lesson did not start: %s" % pl.l_result.cget("text"))
        else:
            pl.playing = False
            hit = None
            for _ in range(40):
                hit = pl.step(0.5)
                if hit is not None:
                    break
            if hit != 0 or not pl.choice_buttons:
                wrong.append("no stop at the decision")
            end = time.time() + 20
            while time.time() < end and pl.photo is None:
                top.update()
                time.sleep(0.05)
            if not pl.canvas.find_withtag("hud") or not pl.canvas.find_withtag("frame") or pl.photo is None:
                wrong.append("picture / HUD not drawn (hud %d, frame %d, photo %s, timeline %d, %s)" % (
                    len(pl.canvas.find_withtag("hud")), len(pl.canvas.find_withtag("frame")), pl.photo is not None,
                    len(pl.timeline), pl.l_result.cget("text")))
            pl.choose(1)
            if "Το STANDBY κερδίζει." not in pl.l_fb.cget("text"):
                wrong.append("no explanation after the answer")
            pl.resume()
            pl.playing = False
            for _ in range(40):
                hit = pl.step(0.5)
                if hit is not None:
                    break
            if hit != 1 or pl.cam != "REAR-LEFT":
                wrong.append("no stop at the hazard / camera not switched")

            class Ev(object):
                x, y = int(0.25 * va.VW), int(0.35 * va.VH)
            pl.on_click(Ev())
            if not pl.run.results or pl.run.results[-1] != (1, True):
                wrong.append("hazard click: %s" % pl.run.results)
            pl.resume()
            pl.playing = False
            for _ in range(80):
                pl.step(0.5)
                if pl.logged:
                    break
            store.load()
            ok = [e for e in store.entries() if e.get("kind") == "videolesson" and e.get("id") == "v_ac37"
                  and e.get("user") == "ann" and e.get("done") and e.get("score") == 100]
            row = [pl.list.get(i) for i, r in enumerate(pl.rows) if r and r[0].get("id") == "v_ac37"]
            if not ok or not row or not row[0].startswith("✓"):
                wrong.append("logbook %d, row %s" % (len(ok), row))
        pl.close()
        top = tk.Toplevel(root)
        app = academy_app.AcademyApp(top, store)
        w1, w2 = app.open_videolessons(), app.open_editor()
        if not isinstance(w1, va.VideoLessonWindow) or not isinstance(w2, va.EditorWindow) or \
                "acc.videolessons" not in app.buttons or "acc.editor" not in app.buttons:
            wrong.append("academy window buttons")
        w1.close()
        w2.close()
        top.destroy()
    finally:
        if old is None:
            os.environ.pop("TRYGONS_ACADEMY_DATA", None)
        else:
            os.environ["TRYGONS_ACADEMY_DATA"] = old
        root.destroy()
    report("AC37 video lesson windows", not wrong, "; ".join(wrong) or
           "banner first in both; the editor: COM4 refused, event opened, trim, decision + hazard area + blur box, "
           "lesson made; the player: listed, picture + HUD drawn, stops at both points, choice + click answers with "
           "the explanation, 100 % in the logbook (kind videolesson, tick); the academy window's buttons open both")


# --- step 21 helpers ----------------------------------------------------------------------------
def _people21(st):
    """An instructor CAPTAIN + trainees (levels set during the first set-up = bootstrap)."""
    for u, ins in (("ins", True), ("ann", False), ("drv", False), ("cap", False)):
        st.add(u, u.upper(), instructor=ins)
    st.set_level("drv", "DRIVER")
    st.set_level("cap", "CAPTAIN")
    st.set_level("ins", "CAPTAIN")
    return _login21(st, "ins")


def _login21(st, user, pw="longpassword1"):
    import auth
    if not st.auth.has_password(user):
        st.auth.set_password(user, pw)
    if st.needs_2fa(user) and not st.auth.has_totp(user):
        st.auth.enrol_totp(user, pw)
    sec = st.auth.get(user).get("totp")
    return st.login(user, pw, auth.totp(sec) if sec else None)


def _stamp(st, user, kind, when, **f):
    """A logbook entry with a chosen time (the tests' clock)."""
    e = st.log(user, kind, **f)
    e["t"] = time_fmt(when)
    lines = open(st.logbook_path, encoding="utf-8").read().splitlines()
    import json as _j
    lines[-1] = _j.dumps(e, ensure_ascii=False)
    with open(st.logbook_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    st.load()
    return e


def time_fmt(epoch):
    import time as _t
    return _t.strftime("%Y-%m-%dT%H:%M:%S", _t.localtime(epoch))


# --- AC38 -------------------------------------------------------------------------------------
def ac38():
    import content
    import exam
    import lesson
    import quiz
    import time as _t
    wrong = []
    pools = {d["id"] for d in quiz.pools()}
    lessons = {d["id"] for d in lesson.lessons()}
    docs = exam.exams()
    if sorted(d["level"] for d in docs) != ["CAPTAIN", "DRIVER"]:
        wrong.append("exams per level %s" % [d.get("level") for d in docs])
    if any(n.startswith("example_") for n in os.listdir(os.path.join(HERE, "content", "exams"))):
        wrong.append("the step-15 example exam is still there")
    for d in docs:
        p = content.check(d, d["id"]) + exam.check_doc(d, d["id"], pools, lessons)
        if p:
            wrong.append("; ".join(p))
    bad = [{"kind": "exam", "id": "x", "level": "OBSERVER", "pass_pct": 80, "questions": 5, "pools": ["q01_safety_basics"]},
           {"kind": "exam", "id": "x", "level": "DRIVER", "pass_pct": 20, "questions": 5, "pools": ["q01_safety_basics"]},
           {"kind": "exam", "id": "x", "level": "DRIVER", "pass_pct": 80, "questions": 0, "pools": ["q01_safety_basics"]},
           {"kind": "exam", "id": "x", "level": "DRIVER", "pass_pct": 80, "questions": 5, "pools": ["nope"]},
           {"kind": "exam", "id": "x", "level": "DRIVER", "pass_pct": 80, "questions": 5, "pools": ["q01_safety_basics"],
            "required": ["l99"]},
           {"kind": "exam", "id": "x", "level": "DRIVER", "pass_pct": 80, "questions": 5, "pools": ["q01_safety_basics"],
            "attempts": 0}]
    missed = [i for i, d in enumerate(bad) if not exam.check_doc(d, "bad", pools, lessons)]
    if missed:
        wrong.append("checker missed %s" % missed)
    st = _store(tempfile.mkdtemp(prefix="trygons_ac38_"))
    _people21(st)
    dd, dc = exam.by_id("driver_exam"), exam.by_id("captain_exam")
    if exam.refused(st, "ann", dc) is None or exam.refused(st, "drv", dd) is not None and "lessons" not in exam.refused(st, "drv", dd):
        wrong.append("who may sit: %s / %s" % (exam.refused(st, "ann", dc), exam.refused(st, "drv", dd)))
    if "lessons" not in (exam.refused(st, "ann", dd) or ""):
        wrong.append("lessons gate: %s" % exam.refused(st, "ann", dd))
    for lid in dd["required"]:
        st.log("ann", "lesson", id=lid, done=True)
    if exam.refused(st, "ann", dd) is not None:
        wrong.append("ready refused: %s" % exam.refused(st, "ann", dd))
    # attempts: 3 failures, then 24 h; a pass resets
    now = _t.time()
    for k in range(3):
        _stamp(st, "ann", "attempt", now - 3600 + k * 60, id="driver_exam")
        _stamp(st, "ann", "exam", now - 3600 + k * 60 + 30, id="driver_exam", level="DRIVER", score=40, passed=False)
    why = exam.refused(st, "ann", dd, now)
    if not why or "cool-down" not in why or exam.attempts_left(st.entries(), "ann", dd, now) != 0:
        wrong.append("no cool-down after 3 failures: %s" % why)
    if exam.refused(st, "ann", dd, now + 23 * 3600 + 1000) is not None:
        wrong.append("still closed after 24 h: %s" % exam.refused(st, "ann", dd, now + 23 * 3600 + 1000))
    if exam.attempts_left(st.entries(), "ann", dd, now + 24 * 3600) != 3:
        wrong.append("attempts after the cool-down %d" % exam.attempts_left(st.entries(), "ann", dd, now + 24 * 3600))
    # online: only supervised
    env = {"TRYGONS_ACADEMY_ONLINE": "1"}
    st2 = _store(tempfile.mkdtemp(prefix="trygons_ac38b_"))
    ins = _people21(st2)
    for lid in dd["required"]:
        st2.log("ann", "lesson", id=lid, done=True)
    if "supervised" not in (exam.refused(st2, "ann", dd, env=env) or ""):
        wrong.append("online without supervision: %s" % exam.refused(st2, "ann", dd, env=env))
    st2.supervise("ann", ins, 60)
    if exam.refused(st2, "ann", dd, env=env) is not None:
        wrong.append("supervised refused: %s" % exam.refused(st2, "ann", dd, env=env))
    # a run: attempt logged at the start, the result at the end, the level never changes
    r = exam.Run(st2, "ann", dd, "el", seed=4, env=env)
    while not r.quiz.finished():
        it = r.quiz.items[r.quiz.index]
        if it["q"].get("type") == "picture":
            x, y, w_, h_ = quiz.area_rect(it["q"]["show"], it["q"]["area"])
            r.quiz.click(x + w_ / 2, y + h_ / 2)
        else:
            r.quiz.answer(it["order"].index(it["q"]["answer"]))
        r.quiz.next()
    res = r.finish()
    ent = [e["kind"] for e in st2.entries() if e.get("user") == "ann"][-2:]
    if not res["passed"] or res["score"] != 100 or ent != ["attempt", "exam"] or not res["online"] or \
            not res["supervised"] or st2.accounts["ann"].level != "OBSERVER":
        wrong.append("run %s, logbook %s, level %s" % ({k: res.get(k) for k in ("score", "passed", "online")}, ent,
                                                       st2.accounts["ann"].level))
    if exam.passed(st2.entries(), "ann", "DRIVER") is None:
        wrong.append("exam.passed")
    # the window
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
    except Exception as e:
        report("AC38 exams", not wrong, "; ".join(wrong) or "model checked; no display (%s)" % e)
        return
    try:
        import banner
        import exam_app
        w = exam_app.ExamWindow(tk.Toplevel(root), "el", store=st2, user="drv", seed=2)
        if w.root.pack_slaves()[0] is not w.banner or w.banner.cget("text") != banner.banner_text("el"):
            wrong.append("exam window banner")
        if w.start_exam("captain_exam") is not None or "Η εξέταση δεν ξεκίνησε" not in w.l_q.cget("text"):
            wrong.append("window refusal %r" % w.l_q.cget("text"))
        w2 = exam_app.ExamWindow(tk.Toplevel(root), "el", store=st, user="ann", seed=3)
        later = _t.time() + 25 * 3600
        st.log("ann", "note", note="(a day later)")
        run = None
        if exam.refused(st, "ann", dd, later) is None:
            # the window uses the real clock: give it a fresh account instead
            pass
        st.add("bob")
        for lid in dd["required"]:
            st.log("bob", "lesson", id=lid, done=True)
        w3 = exam_app.ExamWindow(tk.Toplevel(root), "el", store=st, user="bob", seed=3)
        run = w3.start_exam("driver_exam")
        n = 0
        while run and not run.quiz.finished() and n < 40:
            q = run.quiz.current()
            if q["type"] == "picture":
                w3.click_at(0.01, 0.01)
            else:
                w3.answer(0)
            if "Η απάντηση καταγράφηκε" not in w3.l_res.cget("text"):
                wrong.append("the window showed the answer during the exam: %r" % w3.l_res.cget("text"))
                break
            w3.next()
            n += 1
        logged = [e for e in st.entries() if e.get("user") == "bob" and e.get("kind") == "exam"]
        if not run or not logged or "Βαθμός" not in w3.l_q.cget("text"):
            wrong.append("window run: %s %r" % (len(logged), w3.l_q.cget("text")))
        top = tk.Toplevel(root)
        import academy_app
        app = academy_app.AcademyApp(top, st)
        if "acc.exams" not in app.buttons or not isinstance(app.open_exams(), exam_app.ExamWindow):
            wrong.append("academy window: no Exams button")
        _ = w2
    finally:
        root.destroy()
    report("AC38 exams", not wrong, "; ".join(wrong) or
           "DRIVER + CAPTAIN exams (checker catches %d bad files); who may sit, lessons first, 3 attempts then 24 h, a "
           "pass resets, online only supervised; attempt + result logged, the level never changes; the window: banner, "
           "refusal said, no answers shown during the exam, the score logged; the academy window's button" % len(bad))


# --- AC39 -------------------------------------------------------------------------------------
def ac39():
    import auth
    import simguard
    wrong = []
    # RFC 6238 test vector (SHA-1, secret "12345678901234567890"): T = 59 -> 94287082 (8 digits)
    sec = "GEZDGNBVGY3TQOJQGEZDGNBVGY3TQOJQ"
    if auth.totp(sec, 59, digits=8) != "94287082" or auth.totp(sec, 1111111109, digits=8) != "07081804":
        wrong.append("TOTP vectors %s %s" % (auth.totp(sec, 59, digits=8), auth.totp(sec, 1111111109, digits=8)))
    t0 = 1700000000
    if not auth.totp_ok(sec, auth.totp(sec, t0 - 30), t0) or auth.totp_ok(sec, auth.totp(sec, t0 - 90), t0) or \
            auth.totp_ok(sec, "abc", t0) or auth.totp_ok("", "123456", t0):
        wrong.append("TOTP window")
    tmp = tempfile.mkdtemp(prefix="trygons_ac39_")
    st = _store(tmp)
    ins = _people21(st)
    if not ins.twofa or st.session_ok(ins) is not None:
        wrong.append("instructor session %s" % st.session_ok(ins))
    rec = json_load(os.path.join(tmp, "auth", "ins.json"))
    if "longpassword1" in json.dumps(rec) or rec["password"]["rounds"] < 100000:
        wrong.append("password stored in clear / weak")
    st.auth.set_password("ann", "anotherpass")
    try:
        st.auth.set_password("ann", "x" * 9, "wrong-old")
        wrong.append("password changed without the old one")
    except auth.AuthError:
        pass
    try:
        st.auth.set_password("drv", "short")
        wrong.append("a short password")
    except auth.AuthError:
        pass
    a = st.login("ann", "anotherpass")                          # an OBSERVER: no 2FA needed
    if a.twofa or st.session_ok(a, role="examiner") is None:
        wrong.append("a trainee session may examine")
    st.auth.set_password("cap", "captainpass")
    try:
        st.login("cap", "captainpass")
        wrong.append("a CAPTAIN without 2FA enrolled logged in")
    except auth.AuthError:
        pass
    s2 = st.auth.enrol_totp("cap", "captainpass")
    try:
        st.login("cap", "captainpass", "000000" if auth.totp(s2) != "000000" else "111111")
        wrong.append("a wrong 2FA code accepted")
    except auth.AuthError:
        pass
    try:
        st.auth.enrol_totp("cap", "captainpass")
        wrong.append("2FA replaced without the current code")
    except auth.AuthError:
        pass
    st.auth.set_password("drv", "driverpass")
    for _ in range(auth.LOCK_AFTER):
        try:
            st.login("drv", "bad")
        except auth.AuthError:
            pass
    try:
        st.login("drv", "driverpass")
        wrong.append("not locked after %d wrong logins" % auth.LOCK_AFTER)
    except auth.AuthError as e:
        if "locked" not in str(e):
            wrong.append("lock message %s" % e)
    old = auth.Session("ins", True, st.auth.id, t=0)
    other = auth.Session("ins", True, "another-store")
    if st.session_ok(old) is None or st.session_ok(other) is None or st.session_ok(None) is None:
        wrong.append("an old / foreign / missing session accepted")
    target = tempfile.mkdtemp(prefix="trygons_ac39t_")
    env = {"PROGRAMDATA": tempfile.mkdtemp()}
    dst = st.backup(target, env=env, stamp="z")
    if os.path.exists(os.path.join(dst, "auth")):
        wrong.append("the secrets went into the backup")
    if "otpauth://totp/" not in auth.otpauth_uri("ins", s2) or "secret=" + s2 not in auth.otpauth_uri("ins", s2):
        wrong.append("otpauth link")
    _ = simguard
    report("AC39 logins + 2FA", not wrong, "; ".join(wrong) or
           "RFC 6238 vectors, +-30 s window; PBKDF2 passwords (never in clear), change needs the old one; CAPTAIN + "
           "instructor need 2FA (enrolled with the password, replaced only with the current code), a trainee doesn't "
           "and may not examine; locked after 5 wrong logins; old / foreign sessions refused; secrets never in a backup")


# --- AC40 -------------------------------------------------------------------------------------
def ac40():
    import online
    import simguard
    wrong = []
    for env, want in (({}, False), ({"TRYGONS_ACADEMY_ONLINE": "1"}, True), ({"SESSIONNAME": "RDP-Tcp#3"}, True),
                      ({"SESSIONNAME": "Console"}, False), ({"SSH_CONNECTION": "1 2 3 4"}, True)):
        if online.is_online(env) != want:
            wrong.append("is_online %s" % env)
    if online.check_acl(online.EXAMPLE_ACL):
        wrong.append("the example policy: %s" % online.check_acl(online.EXAMPLE_ACL))
    boats = tempfile.mkdtemp(prefix="trygons_ac40b_")
    with open(os.path.join(boats, "x.json"), "w", encoding="utf-8") as f:
        f.write('{"id": "x1", "name": "X1", "tailscale_ip": "100.101.102.103", "gateway_port": 5761}')
    bad = [
        {"acls": [{"action": "accept", "src": ["tag:trainee"], "dst": ["*:*"]}]},
        {"acls": [{"action": "accept", "src": ["tag:trainee"], "dst": ["tag:academy:3389", "tag:boat:5761"]}]},
        {"acls": [{"action": "accept", "src": ["tag:trainee"], "dst": ["100.101.102.103:5761"]}]},
        {"acls": [{"action": "accept", "src": ["tag:trainee"], "dst": ["100.64.0.0/10:*"]}]},
        {"acls": [{"action": "accept", "src": ["*"], "dst": ["tag:boat:*"]},
                  {"action": "accept", "src": ["tag:trainee"], "dst": ["tag:academy:3389"]}]},
        {"acls": [{"action": "accept", "src": ["tag:station"], "dst": ["tag:boat:*"]}]},
    ]
    missed = [i for i, p in enumerate(bad) if not online.check_acl(p, boats)]
    if missed:
        wrong.append("policy checker missed %s" % missed)
    st = _store(tempfile.mkdtemp(prefix="trygons_ac40_"))
    ins = _people21(st)
    env = {"TRYGONS_ACADEMY_ONLINE": "1"}
    st.accounts["drv"].expiry = datetime.date.today() + datetime.timedelta(days=5)
    for u, lv in (("ann", "DRIVER"), ("drv", "DRIVER")):
        try:
            st.set_level(u, lv, datetime.date.today() + datetime.timedelta(days=400), env=env)
            wrong.append("online set_level raised %s" % u)
        except simguard.Refused as e:
            if "online" not in str(e):
                wrong.append("online refusal says %s" % e)
    try:
        st.grant_practical({"trainee": "ann", "kind": "upgrade"}, ins, env=env)
        wrong.append("online grant")
    except simguard.Refused as e:
        if "online" not in str(e):
            wrong.append("grant refusal says %s" % e)
    ann = _login21(st, "ann")
    for sess in (None, ann):
        try:
            st.supervise("drv", sess, 60)
            wrong.append("supervised by %s" % (sess and sess.user))
        except simguard.Refused:
            pass
    import time as _t
    st.supervise("drv", ins, 30)
    if not st.supervised("drv") or st.supervised("drv", _t.time() + 31 * 60) or st.supervised("ann"):
        wrong.append("supervised window")
    src = open(os.path.join(HERE, "online.py"), encoding="utf-8").read()
    names, _ = imports_of(os.path.join(HERE, "online.py"))
    if names & NET:
        wrong.append("online.py imports %s" % (names & NET))
    _ = src
    report("AC40 the online academy", not wrong, "; ".join(wrong) or
           "online = RDP / SSH / TRYGONS_ACADEMY_ONLINE; the example Tailscale policy passes, %d bad policies caught "
           "(trainee -> *, a boat tag, a boats/ IP, the Tailscale range, everyone -> boat, no trainee rule); nothing "
           "online upgrades; supervision only by an instructor's 2FA login, for its minutes" % len(bad))


# --- AC41 -------------------------------------------------------------------------------------
def ac41():
    import accounts
    import simguard
    wrong = []
    st = _store(tempfile.mkdtemp(prefix="trygons_ac41_"))
    ins = _people21(st)
    today = datetime.date.today()
    st.accounts["drv"].expiry = today + datetime.timedelta(days=20)
    need = st.renewal_status("drv", today)["needs"]
    if len(need) != 3:
        wrong.append("DRIVER renewal needs %s" % need)
    try:
        st.renew("drv", ins, today)
        wrong.append("renewed without the refresher")
    except simguard.Refused:
        pass
    st.log("drv", "lesson", id="l01_hdg_lock", done=True)
    st.log("drv", "quiz", id="q03_alarms", topic="q03_alarms", score=80)
    st.log("drv", "scenario", id="s01_compass_fail", done=True, score=90)
    if st.renewal_status("drv", today)["needs"]:
        wrong.append("still needs %s" % st.renewal_status("drv", today)["needs"])
    ann = _login21(st, "ann")
    try:
        st.renew("drv", ann, today)
        wrong.append("renewed by a trainee")
    except simguard.Refused:
        pass
    a = st.renew("drv", ins, today)
    if a.level != "DRIVER" or (a.expiry - today).days < 360:
        wrong.append("renewal expiry %s" % a.expiry)
    for k in ("lesson", "quiz", "scenario"):
        st.log("cap", k, id="x", topic="x", done=True, score=90)
    cneed = st.renewal_status("cap", today)["needs"]
    if len(cneed) != 1 or "check ride" not in cneed[0]:
        wrong.append("CAPTAIN renewal needs %s" % cneed)
    st.log("cap", "check_ride", passed=True, by="ins")
    if st.renewal_status("cap", today)["needs"]:
        wrong.append("CAPTAIN with a check ride still needs %s" % st.renewal_status("cap", today)["needs"])
    st.accounts["cap"].expiry = today - datetime.timedelta(days=120)
    s = st.renewal_status("cap", today)
    if not s["full_test"]:
        wrong.append("expired 4 months: no full test %s" % s)
    # currency: no logged driving in 90 days = no licence in the gateway file (the level stays)
    later = today + datetime.timedelta(days=100)
    st.accounts["cap"].expiry = later + datetime.timedelta(days=100)
    if st.current("cap", later):
        wrong.append("current after 100 days without driving")
    key = st.key(create=True)
    _path, doc = st.make_push(today=later, key=key)
    rec = {r["user"]: r for r in doc["records"]}["cap"]
    if rec["licence"] or rec["level"] != "CAPTAIN":
        wrong.append("non-current CAPTAIN pushed %s" % rec)
    _path, doc = st.make_push(today=today, key=key)
    if not {r["user"]: r for r in doc["records"]}["cap"]["licence"]:
        wrong.append("current CAPTAIN pushed without licence")
    st.log("cap", "refresher", passed=True, by="ins")
    if not st.current("cap", today + datetime.timedelta(days=60)):
        wrong.append("a refresher trip does not restore the currency")
    rows = {r["user"]: r for r in st.summary(today)}
    if not rows["cap"]["current"]:
        wrong.append("instructor view currency")
    # set_level: after the set-up it only lowers
    try:
        st.set_level("ann", "DRIVER")
        wrong.append("set_level raised after the set-up")
    except simguard.Refused as e:
        if "practical test" not in str(e):
            wrong.append("refusal says %s" % e)
    st.set_level("drv", "OBSERVER")
    if st.accounts["drv"].effective_level() != "OBSERVER" or st.bootstrap():
        wrong.append("lowering / bootstrap")
    if accounts.LOG_KINDS.count("practical") != 1:
        wrong.append("log kinds")
    report("AC41 renewal + currency + no upgrade outside the practical test", not wrong, "; ".join(wrong) or
           "DRIVER renewal = refresher lesson + quiz + scenario (instructor's 2FA), CAPTAIN + a check ride, expired > "
           "3 months = full test; no driving in 90 days = no licence in the gateway file until a refresher trip; "
           "set_level only lowers once a CAPTAIN instructor exists")


# --- AC42 -------------------------------------------------------------------------------------
def ac42():
    import certificate
    import simguard
    import text
    wrong = []
    try:
        import PIL                                              # noqa: F401
    except ImportError:
        report("AC42 certificate PDF", False, "Pillow is not installed (the ground station needs it too)")
        return
    st = _store(tempfile.mkdtemp(prefix="trygons_ac42_"))
    ins = _people21(st)
    st.log("cap", "practical", passed=True, by="ins", target="CAPTAIN", test_kind="upgrade")
    st.log("cap", "exam", id="captain_exam", level="CAPTAIN", score=93, passed=True)
    out = os.path.join(tempfile.mkdtemp(prefix="trygons_ac42o_"), "cap.pdf")
    for sess in (None, _login21(st, "ann")):
        try:
            certificate.make(st, "cap", out, sess, text.T)
            wrong.append("made without an instructor's login")
        except simguard.Refused:
            pass
    p = certificate.make(st, "cap", out, ins, text.T)
    data = open(p, "rb").read()
    if not data.startswith(b"%PDF") or len(data) < 5000:
        wrong.append("not a PDF (%d bytes)" % len(data))
    f = certificate.facts(st, "cap")
    if f["effective"] != "CAPTAIN" or not f["practical"] or not f["exams"]:
        wrong.append("facts %s" % f)
    st.accounts["drv"].expiry = datetime.date.today() - datetime.timedelta(days=3)
    if certificate.facts(st, "drv")["expiry"] is not None or certificate.facts(st, "drv")["effective"] != "OBSERVER":
        wrong.append("an expired level printed as in force")
    if not [e for e in st.entries() if e.get("kind") == "certificate" and e.get("user") == "cap"]:
        wrong.append("not logged")
    try:
        certificate.make(st, "cap", out[:-4] + ".txt", ins, text.T)
        wrong.append("a .txt certificate")
    except ValueError:
        pass
    report("AC42 certificate PDF", not wrong, "; ".join(wrong) or
           "an instructor's 2FA session only; a real PDF (%d bytes) with the level in force, the practical test and the "
           "exams; an expired level never printed as in force; logged" % len(data))


# --- AC43 -------------------------------------------------------------------------------------
def ac43():
    import auth
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
    except Exception as e:
        report("AC43 academy window step 21", True, "no display (%s)" % e)
        return
    wrong = []
    try:
        import academy_app
        st = _store(tempfile.mkdtemp(prefix="trygons_ac43_"))
        st.add("ins", "Instr", instructor=True)
        st.add("ann")
        st.set_level("ins", "CAPTAIN")
        app = academy_app.AcademyApp(tk.Toplevel(root), st)
        for k in academy_app.AUTH_KEYS + ("acc.exams",):
            if k not in app.buttons:
                wrong.append("button %s" % k)
        app.tree.selection_set("ins")
        app.set_password(new="instructor-pw")
        sec = app.enrol(password="instructor-pw")
        if not sec or not st.auth.has_totp("ins"):
            wrong.append("enrol")
        if app.login("ins", "instructor-pw", "000000" if auth.totp(sec) != "000000" else "111111") is not None:
            wrong.append("a wrong code logged in")
        if not app.login("ins", "instructor-pw", auth.totp(sec)) or app.session.user != "ins":
            wrong.append("login: %s" % app.msg.cget("text"))
        app.tree.selection_set("ann")
        if not app.supervise(60) or not st.supervised("ann"):
            wrong.append("supervise")
        app.root.after_idle(lambda: None)
        out = os.path.join(tempfile.mkdtemp(), "ann.pdf")
        try:
            import PIL                                          # noqa: F401
            if app.certificate(out) != out:
                wrong.append("certificate: %s" % app.msg.cget("text"))
        except ImportError:
            pass
        app.session = None
        app.tree.selection_set("ann")
        if app.renew() is not None:
            wrong.append("renew without a login")
    finally:
        root.destroy()
    report("AC43 academy window step 21", not wrong, "; ".join(wrong) or
           "Exams / login / password / 2FA app / renew / supervise / certificate buttons; password + 2FA enrolment, a "
           "wrong code refused, the instructor's login; supervise + certificate through the window; renew needs the "
           "login")

if __name__ == "__main__":
    for t in (ac1, ac2, ac3, ac4, ac5, ac6, ac7, ac8, ac9, ac10, ac11, ac12, ac13, ac14, ac15, ac16, ac17, ac18,
              ac19, ac20, ac21, ac22, ac23, ac24, ac25, ac26, ac27, ac28, ac29,
              ac30, ac31, ac32, ac33, ac34, ac35, ac36, ac37, ac38, ac39, ac40, ac41, ac42, ac43):
        try:
            t()
        except Exception as e:                                  # a crash = a failure, the others still run
            report(t.__name__.upper(), False, "crashed: %r" % e)
    ok = all(results)
    print("ACADEMY TESTS: %d/%d PASS" % (sum(results), len(results)))
    sys.exit(0 if ok else 1)
