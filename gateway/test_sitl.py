"""SITL tests of the UAV magazine (CLAUDE.md V, step X2): REAL ArduPilot ArduCopter SITL copters behind the REAL router
process (gateway/uav/router_worker.py + allow.py) and the REAL UavManager (registry, launch sequence, pilot lock),
with the simulated hatch. Run them with tools/run_sitl.py (background, log, 45 min limit); tools/check.sh runs them
only when SITL is installed (python tools/get_sitl.py). Without SITL every test is N/A - NEVER a PASS.

What is real here and what is simulated:
  real        ArduCopter (the flight code, its failsafes, modes, mission, EKF, arming checks) in SITL; the router
              process; allow.py; the manager's registry / launch / route / pilot / battery code
  simulated   the boat (fresh STANDBY telemetry at a fixed position, the relay board + the hatch relay: the
              captain's YES is given by the test), the hatch (simhatch.py), the USB cable / ELRS radio = a TCP <-> UDP
              bridge per link (the test cuts it: the breakaway USB-C at the launch, a lost radio link)
  the bench   the test's own connection to each SITL copter's SERIAL2 (sysid 250) = the physics / world side only:
              SIM_* parameters (GPS off / on) and reading the true state. It is NOT the gateway: the gateway never
              writes a parameter (SI12 proves the drone's parameters are unchanged after every flight).

The drones' parameters = docs/uav_params.param (+ the SITL calibration / battery lines below = the bench's job),
loaded with SITL's --defaults like the owner loads them with Mission Planner.

  SI1   registration: a copter with the file's parameters is registered; one with a wrong failsafe parameter
        (FS_GCS_ENABLE 1) and a wrong file-only SAFETY parameter (GUID_TIMEOUT 10) is refused, the notice names both
  SI2   the GPS warm start (GPS_RTCM_DATA with UBX-MGA time + position) reaches the drone: accepted, no error
  SI3   a C2 route -> the mission over USB + read-back, the launch sequence with the simulated hatch: ARM only with the
        hatch OPEN switch, climb, the USB breakaway, MISSION START, the hatch closed after the last drone
  SI4   the mission is flown to the last waypoint and the drone loiters there (LOITER_UNLIM, never lands)
  SI5   an in-flight mission change (the captain's route for the flying drone) is flown
  SI6   radio link lost (USB gone, ELRS cut 20 s) = the drone HOLDS (no RTL / no LAND, altitude kept); back = seen again
  SI7   manual assisted flying: GUIDED velocity setpoints move it, centred = it stops and holds its altitude,
        «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» (give back) = BRAKE then LOITER
  SI8   GPS lost in the air (W68): the EKF failsafe = ALT_HOLD, the altitude held with the RC receiver ignored, no land
  SI9   the allow-list with a real ArduPilot: servo / relay / gripper / winch / mount / parachute / RTL / LAND / force
        ARM / RTL mode never leave the router (result None), the drone unchanged; ArduPilot's messages that are not on
        the list are dropped (denied_in), none of them is in the .tlog
  SI10  the battery minimum = the ONLY landing: a manager NAV_LAND refused; the router's rule lands it (LAND, then on
        the ground + disarmed); ArduPilot never landed by itself before
  SI11  the SITL-made .tlog read by pymavlink: every packet, timestamps in order, no denied command in it
  SI12  the gateway never wrote a parameter: every SAFETY parameter of the drone is the same after the flights
  SI13  the no-GPS climb fallback (W59 / W68) measured: ALT_HOLD + TAKEOFF param3 = 1 with the RC receiver ignored
  SI14  six copters: 5 good + 1 with a wrong FENCE_ACTION; registration (5 ok, 1 refused naming it); the launch of 5
        in series (gap >= gap_s between the ARMs, slot order); all 5 hold when every radio link is cut
"""
import json
import math
import os
import queue
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")
os.environ.setdefault("TRYGONS_TEST", "1")
REPO = os.path.dirname(HERE)
sys.path.insert(1, os.path.join(REPO, "tools"))
import common  # noqa: E402,F401  (the ground station's modules on the path: proto / alerts for uavlink)
import proto  # noqa: E402
import strays  # noqa: E402
import toolpaths  # noqa: E402
from uav import allow, mavlink, registry, tlog  # noqa: E402
from uav import manager as mgr_mod  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

NA = "N/A"
results = []
HOME = (37.82, 23.60)
INST0 = 40                       # SITL instance = INST0 + slot: SERIAL0 5760 + 10 x instance (never the boat sim's 5760)
MODE = {"STABILIZE": 0, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9, "BRAKE": 17,
        "SMART_RTL": 21}
NEVER = (MODE["RTL"], MODE["SMART_RTL"])
SITL_EXTRA = [                   # the bench's part (calibration + a 4S pack), never a C2 rule
    ("INS_ACCOFFS_X", 0.001), ("INS_ACCOFFS_Y", 0.001), ("INS_ACCOFFS_Z", 0.001),
    ("INS_ACCSCAL_X", 1.001), ("INS_ACCSCAL_Y", 1.001), ("INS_ACCSCAL_Z", 1.001),
    ("INS_ACC2OFFS_X", 0.001), ("INS_ACC2OFFS_Y", 0.001), ("INS_ACC2OFFS_Z", 0.001),
    ("INS_ACC2SCAL_X", 1.001), ("INS_ACC2SCAL_Y", 1.001), ("INS_ACC2SCAL_Z", 1.001),
    ("SIM_BATT_VOLTAGE", 16.8), ("SIM_BATT_CAP_AH", 3.0),
]
FINDINGS = []                    # measured answers to the "TO CONFIRM in SITL" items (printed + TEST_REPORT.md)


def report(name, ok, detail):
    results.append(None if ok is NA else bool(ok))
    print("[%s] %s - %s" % ("N/A " if ok is NA else ("PASS" if ok else "FAIL"), name, detail), flush=True)


def finding(text):
    FINDINGS.append(text)
    print("FINDING: %s" % text, flush=True)


def until(cond, timeout, step=0.1, pump=None):
    end = time.time() + timeout
    while time.time() < end:
        if pump is not None:
            pump()
        v = cond()
        if v:
            return v
        time.sleep(step)
    return None


# ------------------------------------------------------------------------------------------- the cable / radio ---

class Bridge:
    """A drone link as the router sees it: "udp:127.0.0.1:<port>" here, TCP to the SITL copter's serial port there.
    cut = True: nothing passes either way (the USB breakaway / a lost radio link)."""

    def __init__(self, tcp_port, name):
        self.tcp_port, self.name = tcp_port, name
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.settimeout(0.2)
        self.port = self.udp.getsockname()[1]
        self.peer = None
        self.tcp = None
        self.cut = False
        self.stop = False
        self.n_up = self.n_down = 0
        threading.Thread(target=self._tcp_loop, daemon=True).start()
        threading.Thread(target=self._udp_loop, daemon=True).start()

    def spec(self):
        return "udp:127.0.0.1:%d" % self.port

    def _tcp_loop(self):
        while not self.stop:
            if self.tcp is None:
                try:
                    s = socket.create_connection(("127.0.0.1", self.tcp_port), timeout=1.0)
                    s.settimeout(0.2)
                    self.tcp = s
                except OSError:
                    time.sleep(0.2)
                    continue
            try:
                data = self.tcp.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                data = b""
            if not data:
                try:
                    self.tcp.close()
                except OSError:
                    pass
                self.tcp = None
                time.sleep(0.2)
                continue
            if self.peer is not None and not self.cut:
                try:
                    self.udp.sendto(data, self.peer)
                    self.n_down += len(data)
                except OSError:
                    pass

    def _udp_loop(self):
        while not self.stop:
            try:
                data, addr = self.udp.recvfrom(65536)
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                break
            self.peer = addr
            if self.cut or self.tcp is None:
                continue
            try:
                self.tcp.sendall(data)
                self.n_up += len(data)
            except OSError:
                pass

    def close(self):
        self.stop = True
        for s in (self.udp, self.tcp):
            try:
                if s is not None:
                    s.close()
            except OSError:
                pass


# ----------------------------------------------------------------------------------------------- a SITL copter ---

class Bench:
    """The test's own connection to a SITL copter (SERIAL2): the world side (SIM_* parameters) + the true state."""

    def __init__(self, port):
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection("tcp:127.0.0.1:%d" % port, source_system=250, source_component=1,
                                            autoreconnect=True)
        self.lock = threading.Lock()
        self.hb = None
        self.pos = None
        self.servo = None
        self.sys = None
        self.texts = []
        self.modes = []                     # (t, custom_mode, armed) on every change
        self.params = {}
        self.stop = False
        self.t_hb = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        asked = 0.0
        while not self.stop:
            try:
                x = self.m.recv_match(blocking=True, timeout=0.5)
            except Exception:
                time.sleep(0.2)
                continue
            now = time.time()
            if now - asked > 5.0 and self.hb is not None:
                asked = now
                try:
                    self.m.mav.request_data_stream_send(self.m.target_system or 1, 1, 0, 4, 1)
                except Exception:
                    pass
            if x is None:
                continue
            t = x.get_type()
            with self.lock:
                if t == "HEARTBEAT" and x.autopilot != 8:
                    if self.hb is None or self.hb.custom_mode != x.custom_mode or \
                            (self.hb.base_mode & 0x80) != (x.base_mode & 0x80):
                        self.modes.append((now, x.custom_mode, bool(x.base_mode & 0x80)))
                    self.hb, self.t_hb = x, now
                elif t == "GLOBAL_POSITION_INT":
                    self.pos = x
                elif t == "SERVO_OUTPUT_RAW":
                    self.servo = x
                elif t == "SYS_STATUS":
                    self.sys = x
                elif t == "STATUSTEXT":
                    self.texts.append((now, x.text))
                    del self.texts[:-200]
                elif t == "PARAM_VALUE":
                    self.params[x.param_id] = x.param_value

    def mode(self):
        return self.hb.custom_mode if self.hb is not None else None

    def armed(self):
        return bool(self.hb is not None and self.hb.base_mode & 0x80)

    def alt(self):
        return self.pos.relative_alt / 1000.0 if self.pos is not None else None

    def latlon(self):
        return (self.pos.lat / 1e7, self.pos.lon / 1e7) if self.pos is not None else None

    def gs(self):
        return math.hypot(self.pos.vx, self.pos.vy) / 100.0 if self.pos is not None else None

    def ready(self):
        return bool(self.sys is not None and self.sys.onboard_control_sensors_health & 0x10000000)

    def set_param(self, name, value, timeout=5.0):
        """The world side only (SIM_*): never a C2 parameter."""
        assert name.startswith("SIM_"), name
        self.params.pop(name, None)
        end = time.time() + timeout
        while time.time() < end:
            self.m.mav.param_set_send(self.m.target_system or 1, 1, name.encode(), float(value), 9)
            if until(lambda: name in self.params, 1.0):
                return abs(self.params[name] - value) < 1e-3
        return False

    def read_params(self, names, timeout=20.0):
        for n in names:
            self.params.pop(n, None)
        end = time.time() + timeout
        while time.time() < end:
            left = [n for n in names if n not in self.params]
            if not left:
                break
            for n in left[:20]:
                self.m.mav.param_request_read_send(self.m.target_system or 1, 1, n.encode(), -1)
            time.sleep(0.5)
        return {n: self.params.get(n) for n in names}

    def close(self):
        self.stop = True
        try:
            self.m.close()
        except Exception:
            pass


class Copter:
    def __init__(self, exe, root, slot, extra=()):
        self.slot, self.sysid, self.inst = slot, 10 + slot, INST0 + slot
        self.wd = os.path.join(root, "sitl_%d" % slot)
        os.makedirs(self.wd, exist_ok=True)
        own = os.path.join(self.wd, "bench.parm")
        with open(own, "w") as f:
            for n, v in SITL_EXTRA + [("MAV_SYSID", self.sysid)] + list(extra):
                f.write("%s,%s\n" % (n, v))
        defs = ",".join([registry.PARAM_FILE, own])
        base = 5760 + 10 * self.inst
        self.usb = Bridge(base, "usb%d" % slot)               # SERIAL0 (the SITL waits for it before it starts)
        self.elrs = Bridge(base + 2, "elrs%d" % slot)        # SERIAL1
        self.log = open(os.path.join(self.wd, "sitl.txt"), "wb")
        self.p = subprocess.Popen([exe, "-M", "quad", "--home", "%f,%f,0,0" % HOME, "-I%d" % self.inst,
                                   "--defaults", defs], cwd=self.wd, stdout=self.log, stderr=subprocess.STDOUT,
                                  stdin=subprocess.DEVNULL)
        strays.dies_with_parent(self.p)
        time.sleep(1.5)
        self.bench = Bench(base + 3)                          # SERIAL2

    def close(self):
        for x in (self.bench, self.usb, self.elrs):
            try:
                x.close()
            except Exception:
                pass
        try:
            self.p.kill()
            self.p.wait(10)
        except Exception:
            pass
        try:
            self.log.close()
        except OSError:
            pass


# --------------------------------------------------------------------------------- the boat + gateway around it ---

class St:
    def __init__(self, sid, crew):
        self.id, self.user, self.role, self.name = sid, "u%d" % sid, "HELM", "u%d" % sid
        self.crew, self.single_op, self.local, self.closed = crew, False, False, False
        self.session, self.video_key = "s%d" % sid, "k%d" % sid
        self.sent = []

    def send(self, m):
        self.sent.append(m)


class Stations:
    def __init__(self, gw):
        self.gw = gw
        self.captain_st, self.copilot_st = St(1, "captain"), St(2, "copilot")
        self.stations = {1: self.captain_st, 2: self.copilot_st}
        self.asked = []
        self.answer = True
        self.n_ask = 0

    def helm_station(self):
        return self.captain_st, ""

    def captain(self):
        return self.captain_st

    def hatch_relay_ok(self, n):
        return True, ""

    def uav_relay(self, n, on, done):
        self.gw.relay[n] = bool(on)
        self.gw.loop.call_soon_threadsafe(done, 0, "")

    def ask_captain(self, st, msg, what, cb, *a, **kw):
        """The captain's YES / NO on screen 1: the test is the captain (answer = True = YES)."""
        self.n_ask += 1
        self.asked.append(what)
        self.gw.loop.call_soon_threadsafe(cb, self.answer, 0 if self.answer else 125)
        return self.n_ask

    def cancel_ask(self, *a, **kw):
        return None

    def refuse_asks_for(self, *a, **kw):
        return None

    def age(self, st, ts):
        return max(0.0, time.time() - float(ts))


class Loop:
    """The gateway's event loop as the manager uses it: callbacks from threads, run by pump() on the test thread."""

    def __init__(self):
        self.q = queue.Queue()

    def call_soon_threadsafe(self, fn, *a):
        self.q.put((fn, a))

    def run(self):
        while True:
            try:
                fn, a = self.q.get_nowait()
            except queue.Empty:
                return
            fn(*a)


class Gw:
    def __init__(self, root):
        self.root = root
        self.loop = Loop()
        self.relay = {}
        self.audits = []
        self.cfg = {"cameras": {"enabled": False, "rtsp_port": 8554}}
        self.storage = type("S", (), {"usable": True, "root": root, "folder": lambda _s, n: os.makedirs(
            os.path.join(root, n), exist_ok=True) or os.path.join(root, n)})()
        self.stations = Stations(self)
        self.clock = type("C", (), {"gps_valid": True, "gps_rx": 0.0, "now": lambda _s: time.time()})()
        self.saver = False
        self.net = type("N", (), {"on_emergency": False})()

    @property
    def telem2(self):
        return {"relay_ok": True, "relay_on": [n for n, on in self.relay.items() if on], "relay_unknown": [],
                "helm": 1, "alarm_bits2": 0}

    def telem_fresh(self):
        return {"mode": proto.MODE_STANDBY, "mode_name": "STANDBY", "pos_source": 1, "est_lat": HOME[0],
                "est_lon": HOME[1], "est_sog": 0.0, "unc_m": 5.0, "alarm_bits": 0, "alarm_bits2": 0}

    def audit(self, kind, **kw):
        self.audits.append((round(time.time(), 1), kind, kw))

    def data_saver(self):
        return False


HATCH_N = 8


class World:
    """n SITL copters (slot -> extra bad parameters) + the real manager (+ its real router process)."""

    def __init__(self, exe, n, bad=None, launch=None, prefix="sitl_"):
        self.root = tempfile.mkdtemp(prefix="trygons_" + prefix)
        self.copters = {}
        self.events = []                      # (t, text) of what the world did (breakaway, cut, ...)
        self.arm_hatch = {}                   # slot -> the hatch state the moment the bench saw the drone ARMED
        self.max_alt = {}                     # slot -> the highest relative altitude the bench saw
        bad = bad or {}
        for s in range(1, n + 1):
            self.copters[s] = Copter(exe, self.root, s, bad.get(s, ()))
        self.gw = Gw(self.root)
        cfg = {"enabled": True, "n_slots": 6, "hatch_relay": HATCH_N,
               "hatch": {"sim": True, "sim_travel_s": 3.0, "open_s": 20.0, "close_s": 20.0},
               "video": {"enabled": False}, "ai": {"enabled": False}, "launch": dict(launch or {}),
               "slots": [{"usb": self.copters[s].usb.spec(), "radio": self.copters[s].elrs.spec(),
                          "name": "SITL %d" % s} if s in self.copters else {"usb": ""} for s in range(1, 7)]}
        self.m = mgr_mod.UavManager(cfg, self.gw)
        self.m.start(self.gw.loop)
        self.next_tick = 0.0
        self.breakaway = True

    # the world + the gateway's 4 Hz tick
    def pump(self):
        self.gw.loop.run()
        p = self.m.proc
        if p is not None and not getattr(p, "_trygons_job", False):
            strays.dies_with_parent(p)
            p._trygons_job = True
        now = time.time()
        if now >= self.next_tick:
            self.next_tick = now + 0.25
            self.m.tick(now)
        for s, c in self.copters.items():
            b = c.bench
            if b.armed() and s not in self.arm_hatch:
                self.arm_hatch[s] = self.m.hatch.state
            if b.alt() is not None and b.alt() > self.max_alt.get(s, -99.0):
                self.max_alt[s] = b.alt()
            if self.breakaway and not c.usb.cut and b.armed() and (b.alt() or 0.0) > 2.0:
                c.usb.cut = True                      # the breakaway USB-C leaves the slot with the drone
                self.events.append((now, "slot %d USB breakaway at %.1f m" % (s, b.alt())))

    def run(self, seconds, cond=None):
        end = time.time() + seconds
        while time.time() < end:
            self.pump()
            if cond is not None:
                v = cond()
                if v:
                    return v
            time.sleep(0.05)
        return None

    def ready(self, timeout=150.0):
        return self.run(timeout, lambda: all(c.bench.ready() for c in self.copters.values()) and
                        self.m.state == mgr_mod.ACTIVE)

    def register(self, slots, timeout=40.0):
        st = self.gw.stations.copilot_st
        for s in slots:
            self.m.register_msg(st, {"t": "uav_register", "slot": s})
        self.run(timeout, lambda: all(any(x.get("what") == "register" and x.get("slot") == s for x in st.sent)
                                       for s in slots))
        return {s: next((x for x in reversed(st.sent) if x.get("what") == "register" and x.get("slot") == s), None)
                for s in slots}

    def drone(self, s):
        return self.m.slot(s)

    def close(self):
        try:
            self.m.stop(wait=True)
        except Exception:
            pass
        for c in self.copters.values():
            c.close()
        shutil.rmtree(self.root, ignore_errors=True)


def wps_out(dn, de, alt=40.0):
    k = 111320.0
    return {"lat": HOME[0] + dn / k, "lon": HOME[1] + de / (k * math.cos(math.radians(HOME[0]))), "alt": alt}


def dist(a, b):
    k = 111320.0
    return math.hypot((a[0] - b[0]) * k, (a[1] - b[1]) * k * math.cos(math.radians(a[0])))


def ack(st, what, since=0):
    return next((x for x in reversed(st.sent[since:]) if x.get("what") == what), None)


def modes_seen(b, since=0.0):
    return [m for t, m, _a in b.modes if t >= since]


def land_after_rule(w, s):
    """Every LAND of slot s came after the router's own battery 'land' event."""
    t_rule = next((t for t, sl, what, _w in w.m.land_events if sl == s and what == "start"), None)
    lands = [t for t, m, _a in w.copters[s].bench.modes if m == MODE["LAND"]]
    return all(t_rule is not None and t >= t_rule - 0.5 for t in lands), t_rule, lands


def cmd(w, s, via, command, params, timeout=8.0):
    box = []
    w.m.cmd(s, via, command, list(params) + [0] * (7 - len(params)), lambda r, why: box.append((r, why)))
    w.run(timeout, lambda: box)
    return box[0] if box else ("no answer", "")


UPLOAD_WHY = ("the mission upload needs ArduPilot's MISSION_REQUEST (id 40): ArduCopter answers MISSION_COUNT with "
              "MISSION_REQUEST, never MISSION_REQUEST_INT (measured in SITL) - it is not on allow.py's FROM_DRONE list "
              "nor in the router's codec: an allow.py change = a SERIAL safety review (W110), NOT done in X2")


def upload_blocked(w):
    """The launch failed at the mission upload (the MISSION_REQUEST finding)."""
    return "mission upload" in (w.m.seq.why or "")


def fly_cmds(w, s, alt, hatch=True):
    """Fly slot s up WITHOUT a mission (GUIDED, ARM, TAKEOFF through the real router + allow.py; the hatch OPEN switch
    from the simulated hatch) - so the in-flight tests still run while a mission cannot be uploaded."""
    b = w.copters[s].bench
    if hatch and w.m.hatch.state != "OPEN":
        box = []
        w.m.hatch_cmd(True, lambda ok, why: box.append(ok))
        w.run(20, lambda: w.m.hatch.state == "OPEN")
    r1 = cmd(w, s, "usb", allow.CMD_DO_SET_MODE, [1, allow.COPTER_GUIDED])
    r2 = cmd(w, s, "usb", allow.CMD_COMPONENT_ARM_DISARM, [1, 0])
    r3 = cmd(w, s, "usb", allow.CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, alt])
    up = w.run(60, lambda: (b.alt() or 0) > alt - 2.0)
    w.run(5)
    return bool(up), (r1, r2, r3)


SAFETY_NAMES = None


def safety_names():
    rules, _e = registry.load_param_file()
    return [r["name"] for r in rules if r["kind"] == "SAFETY"]


# -------------------------------------------------------------------------------------------- the one-copter run --

def suite_one(exe):
    t_start = time.time()
    # slot 1 = the file's parameters; slot 2 = a wrong failsafe + a wrong file-only SAFETY parameter
    w = World(exe, 2, bad={2: [("FS_GCS_ENABLE", 1), ("GUID_TIMEOUT", 10)]}, prefix="sitl1_")
    try:
        ok = w.ready()
        if not ok:
            for t in ("SI1", "SI2", "SI3", "SI4", "SI5", "SI6", "SI7", "SI8", "SI9", "SI10", "SI11", "SI12"):
                report("%s SITL copters ready" % t, False, "the SITL copters / the router did not get ready in "
                       "150 s (manager %s %s; ready %s)" % (w.m.state, w.m.why,
                                                            [c.bench.ready() for c in w.copters.values()]))
            return
        b1 = w.copters[1].bench
        p_before = b1.read_params(safety_names())
        # ---- SI1 registration -----------------------------------------------------------------------------------
        regs = w.register([1, 2])
        r1, r2 = regs.get(1) or {}, regs.get(2) or {}
        probs2 = r2.get("problems") or []
        notes = [n["text"] for n in w.m.notices]
        named = [n for n in ("FS_GCS_ENABLE", "GUID_TIMEOUT") if any(n in p for p in probs2) and
                 any(n in t for t in notes)]
        rec1 = w.m.reg.recs.get(1) or {}
        report("SI1 registration with real ArduPilot (READ ONLY): the copter with docs/uav_params.param registers "
               "(sysid %s, firmware %s); the copter with FS_GCS_ENABLE 1 + GUID_TIMEOUT 10 is refused and the notice "
               "names BOTH parameters" % (rec1.get("sysid"), rec1.get("fw")),
               r1.get("ok") is True and rec1.get("sysid") == 11 and r2.get("ok") is False and len(named) == 2,
               "slot 1 %s %s; slot 2 problems %s; notices %s; advice slot 1 %s" % (
                   r1.get("ok"), r1.get("problems"), probs2, notes[-2:], rec1.get("advice")))
        # ---- SI2 the GPS warm start --------------------------------------------------------------------------------
        a = w.run(15, lambda: w.m.assist.get(1) if (w.m.assist.get(1) or {}).get("what") else None) or {}
        report("SI2 the GPS warm start (UBX-MGA-INI time + position in GPS_RTCM_DATA, allow-listed) is sent to the "
               "registered SITL copter in its slot and the router reports it delivered",
               a.get("ok") is True and "pos" in (a.get("what") or ""), "assist %s" % a)
        finding("W56 / H4 warm start (SITL): the router's GPS_RTCM_DATA frames reach ArduPilot (accepted, no "
                "error); whether ArduPilot passes UBX-MGA bytes to a REAL u-blox untouched and how much faster the fix "
                "is = still bench only (SITL's simulated GPS has no almanac / time-to-fix model to inject into)")
        # ---- SI3 route + launch -----------------------------------------------------------------------------------
        cap, cop = w.gw.stations.captain_st, w.gw.stations.copilot_st
        far = wps_out(150.0, 60.0, 40.0)
        n0 = len(cop.sent)
        w.m.route_msg(cop, {"t": "uav_route", "target": 1, "wps": [wps_out(80.0, 0.0, 40.0), far]})
        rt = ack(cop, "route", n0)
        n0 = len(cop.sent)
        w.m.prepare_msg(cop, {"t": "uav_launch_prepare", "count": 1})
        t_launch = time.time()
        done = w.run(150, lambda: w.m.seq.state in ("DONE", "STOPPED"))
        hist = [h for h in w.m.seq.history][-14:] if hasattr(w.m.seq, "history") else []
        armed_at_open = w.arm_hatch.get(1)
        in_auto = until(lambda: b1.mode() == MODE["AUTO"], 20, pump=w.pump)
        report("SI3 a C2 route -> mission over USB + read-back -> the launch sequence with the simulated hatch "
               "(copilot prepares, captain's YES): ARM only with the hatch OPEN, climb, USB breakaway, MISSION START, "
               "the hatch CLOSED after the last drone",
               rt is not None and rt.get("ok") and done == "DONE" and w.m.seq.launched == [1] and
               armed_at_open == "OPEN" and in_auto and w.m.hatch.state == "CLOSED" and w.copters[1].usb.cut,
               "route %s; seq %s %s; launched %s; hatch at ARM %s; AUTO %s; hatch now %s; world %s; history %s" % (
                   rt and rt.get("ok"), w.m.seq.state, w.m.seq.why, w.m.seq.launched, armed_at_open, bool(in_auto),
                   w.m.hatch.state, w.events[-2:], hist))
        blocked = upload_blocked(w)
        if blocked:
            finding("MISSION UPLOAD (SITL, ArduCopter %s): %s" % (rec1.get("fw"), UPLOAD_WHY))
            flown, rr = fly_cmds(w, 1, 30.0)
            w.m.hatch_cmd(False, lambda ok, why: None)
            print("  (SI4..: slot 1 flown up by GUIDED / ARM / TAKEOFF through the router instead: %s %s)" % (
                flown, rr), flush=True)
        # ---- SI4 the mission is flown -----------------------------------------------------------------------------
        reached = None if blocked else w.run(90, lambda: b1.latlon() and dist(b1.latlon(), (far["lat"], far["lon"])) < 12.0)
        w.run(5)
        loiter = b1.mode() == MODE["AUTO"] and b1.armed() and (b1.alt() or 0) > 30.0
        report("SI4 the mission is flown to the last waypoint (40 m) and the drone loiters there (LOITER_UNLIM: AUTO, "
               "armed, never a landing / RTL)",
               reached and loiter and not set(modes_seen(b1, t_launch)) & set(NEVER + (MODE["LAND"],)),
               "reached %s, mode %s alt %s, modes %s%s" % (bool(reached), b1.mode(), b1.alt(),
                                                          modes_seen(b1, t_launch),
                                                          ("; NOT FLOWN: " + UPLOAD_WHY) if blocked else ""))
        # ---- SI5 in-flight mission change ------------------------------------------------------------------------
        new = wps_out(40.0, -120.0, 50.0)
        n0 = len(cap.sent)
        w.m.route_msg(cap, {"t": "uav_route", "target": 1, "wps": [new]})
        got = w.run(60, lambda: next((x for x in cap.sent[n0:] if x.get("what") == "route" and
                                      not x.get("pending")), None))
        there = None if blocked else w.run(90, lambda: b1.latlon() and dist(b1.latlon(), (new["lat"], new["lon"])) < 12.0 and
                      abs((b1.alt() or 0) - 50.0) < 4.0)
        report("SI5 an in-flight mission change (the captain's route for the flying UAV) over ELRS: upload + read-back, "
               "MISSION_SET_CURRENT + MISSION START, the new point flown at its altitude (50 m)",
               got is not None and got.get("ok") and there,
               "ack %s; at the new point %s (alt %s)%s" % (got and (got.get("ok"), got.get("text")), bool(there),
                                                           b1.alt(), ("; " + UPLOAD_WHY) if blocked else ""))
        # ---- SI6 link loss = hold ----------------------------------------------------------------------------------
        w.run(3)
        m0, a0, p0, t0 = b1.mode(), b1.alt(), b1.latlon(), time.time()
        w.copters[1].elrs.cut = True
        w.events.append((t0, "slot 1 ELRS cut"))
        lost = w.run(12, lambda: not (w.m.worker.get("radio") or {}).get("1", {}).get("present"))
        w.run(20 - (time.time() - t0))
        alts = b1.alt()
        moved = dist(b1.latlon(), p0) if p0 and b1.latlon() else None
        held = b1.mode() == m0 and b1.armed() and alts is not None and abs(alts - a0) < 2.0 and \
            moved is not None and moved < 10.0
        w.copters[1].elrs.cut = False
        back = w.run(15, lambda: (w.m.worker.get("radio") or {}).get("1", {}).get("present"))
        notes = [n["text"] for n in w.m.notices]
        report("SI6 the radio link lost for 20 s (USB already gone) = the drone HOLDS: same mode, altitude kept, no RTL / "
               "no LAND (FS_GCS_ENABLE 0, FS_THR_ENABLE 0, RC ignored); the router sees the link go and come back",
               lost and held and back and not set(modes_seen(b1, t0)) & set(NEVER + (MODE["LAND"],)) and
               any("LINK LOST" in t for t in notes),
               "mode %s -> %s, alt %.1f -> %s, moved %s m, lost seen %s, back %s, modes %s, notices %s" % (
                   m0, b1.mode(), a0 or -1, alts, moved and round(moved, 1), bool(lost), bool(back),
                   modes_seen(b1, t0), [t for t in notes if "LINK" in t][-2:]))
        finding("W60 / docs 6.1 'lost link = hold' (SITL ArduCopter): with FS_GCS_ENABLE 0 + FS_THR_ENABLE 0 + "
                "RC_OPTIONS bit 0 a 20 s radio loss leaves the drone in its mode at its altitude (measured: %s)" % (
                    "held" if held else "NOT held"))
        # ---- SI7 manual assisted flying ---------------------------------------------------------------------------
        n0 = len(cap.sent)
        w.m.pilots.pilot_msg(cap, {"t": "uav_pilot", "uav": 1, "take": True})
        took = ack(cap, "pilot", n0)
        guided = w.run(8, lambda: b1.mode() == MODE["GUIDED"])
        seq = [0]

        def stick(x, y=0.0, z=0.0, r=0.0, secs=1.0):
            end = time.time() + secs
            while time.time() < end:
                seq[0] += 1
                w.m.pilots.ctl(cap, {"t": "uav_ctl", "uav": 1, "seq": seq[0], "ts": time.time(), "x": x, "y": y,
                                     "z": z, "r": r})
                w.run(0.1)
        stick(0.0, secs=1.0)                                   # centred first (the pilot lock's rule)
        a_m = b1.alt()
        stick(0.0, 0.8, secs=6.0)                              # stick forward
        moving = b1.gs() or 0.0
        stick(0.0, secs=6.0)                                   # let go = centred
        stopped = b1.gs() or 0.0
        alt_held = b1.alt() is not None and a_m is not None and abs(b1.alt() - a_m) < 2.5
        n0 = len(cap.sent)
        w.m.pilots.pilot_msg(cap, {"t": "uav_pilot", "uav": 1, "take": False})      # «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ»
        gave = ack(cap, "pilot", n0)
        t_gb = time.time()
        loiter = w.run(10, lambda: b1.mode() == MODE["LOITER"])
        seen = modes_seen(b1, t_gb)
        report("SI7 manual assisted flying (captain, boat in STANDBY): GUIDED; the stick forward = velocity setpoints "
               "move it, centred = it stops and holds its altitude; «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» (give back) = BRAKE then "
               "LOITER (it hovers where it is)",
               took and took.get("ok") and guided and moving > 2.0 and stopped < 0.8 and alt_held and gave and
               gave.get("ok") and loiter and MODE["BRAKE"] in seen,
               "take %s, GUIDED %s, speed forward %.1f m/s, after centring %.1f m/s, alt %s -> %s, give back %s, "
               "modes after %s" % (took and took.get("ok"), bool(guided), moving, stopped, a_m, b1.alt(),
                                   gave and gave.get("ok"), seen))
        # ---- SI8 GPS lost in the air ------------------------------------------------------------------------------
        w.run(3)
        a0, t0 = b1.alt(), time.time()
        gps_off = b1.set_param("SIM_GPS1_ENABLE", 0)
        alth = w.run(40, lambda: b1.mode() == MODE["ALT_HOLD"])
        w.run(20)
        a1 = b1.alt()
        ok8 = gps_off and alth and b1.armed() and a1 is not None and abs(a1 - a0) < 3.0 and \
            MODE["LAND"] not in modes_seen(b1, t0)
        b1.set_param("SIM_GPS1_ENABLE", 1)
        report("SI8 GPS lost in the air: ArduPilot's EKF failsafe (FS_EKF_ACTION 2) = ALT_HOLD; with the RC receiver "
               "IGNORED (RC_OPTIONS bit 0) ALT_HOLD holds the altitude (20 s), never lands",
               ok8, "gps off %s, ALT_HOLD after %s s, alt %s -> %s, modes %s" % (
                   gps_off, alth and round(time.time() - t0, 1), a0, a1, modes_seen(b1, t0)))
        finding("W68 (in the air): ALT_HOLD with the RC receiver ignored holds the altitude in ArduCopter SITL "
                "(measured %s -> %s m over 20 s; it drifts with the wind, no position hold)" % (
                    a0 and round(a0, 1), a1 and round(a1, 1)))
        until(lambda: (w.drone(1).get("gps_fix") or 0) >= 3, 60, pump=w.pump)
        r_l = cmd(w, 1, "elrs", allow.CMD_DO_SET_MODE, [1, allow.COPTER_LOITER])
        w.run(8)
        # ---- SI9 the allow-list against a real ArduPilot ----------------------------------------------------------
        before_out = dict((w.m.worker.get("stats") or {}))
        sv0 = [getattr(b1.servo, "servo%d_raw" % i, 0) for i in range(5, 17)] if b1.servo else None
        m9, t9 = b1.mode(), time.time()
        bad = [("DO_SET_SERVO", 183, [9, 1900]), ("DO_REPEAT_SERVO", 184, [9, 1900, 3, 1]),
               ("DO_SET_RELAY", 181, [0, 1]), ("DO_REPEAT_RELAY", 182, [0, 3, 1]), ("DO_GRIPPER", 211, [1, 0]),
               ("DO_WINCH", 42600, [1, 1, 5]), ("DO_MOUNT_CONTROL", 205, [10, 0, 0]),
               ("DO_MOUNT_CONFIGURE", 204, [2]), ("DO_PARACHUTE", 208, [2]),
               ("NAV_RETURN_TO_LAUNCH", 20, []), ("NAV_LAND (not the battery rule)", 21, []),
               ("force DISARM", 400, [0, 21196]), ("DO_SET_MODE RTL", 176, [1, 6]),
               ("DO_SET_MODE SMART_RTL", 176, [1, 21]), ("DO_SET_MODE LAND", 176, [1, 9]),
               ("PREFLIGHT_STORAGE (params)", 245, [1]), ("DO_FLIGHTTERMINATION", 185, [1])]
        res = {name: cmd(w, 1, "elrs", c, p, 6.0) for name, c, p in bad}
        refused = [n for n, (r, _w) in res.items() if r is None]
        w.run(3)
        st9 = w.m.worker.get("stats") or {}
        sv1 = [getattr(b1.servo, "servo%d_raw" % i, 0) for i in range(5, 17)] if b1.servo else None
        den = w.m.worker.get("denied") or {}
        den_in = sorted(k[3:] for k in den if k.startswith("in "))
        unchanged = b1.mode() == m9 and b1.armed() and set(modes_seen(b1, t9)) <= {m9}
        report("SI9 the allow-list with a real ArduPilot: servo / relay / gripper / winch / mount / parachute / RTL / "
               "LAND / force DISARM / RTL-SmartRTL-LAND modes / parameter storage / flight termination never leave the "
               "router (result None, denied_out counted), the drone unchanged (mode, armed, outputs 5-16); ArduPilot's "
               "own messages that are not allow-listed are dropped (never decoded / denied_in)",
               len(refused) == len(bad) and unchanged and sv0 == sv1 and
               st9.get("denied_out", 0) - before_out.get("denied_out", 0) >= len(bad) and
               st9.get("unknown", 0) - before_out.get("unknown", 0) > 100,
               "refused %d/%d %s; mode %s->%s; outputs 5-16 %s -> %s; denied_out +%s; dropped in: not in the codec "
               "+%s frames (SERVO_OUTPUT_RAW, RC_CHANNELS, ... never decoded), denied by the list %s" % (
                   len(refused), len(bad), [n for n in res if n not in refused], m9, b1.mode(), sv0, sv1,
                   st9.get("denied_out", 0) - before_out.get("denied_out", 0),
                   st9.get("unknown", 0) - before_out.get("unknown", 0), den_in[:10]))
        # ---- SI10 the battery minimum -----------------------------------------------------------------------------
        t10 = time.time()
        r_land = cmd(w, 1, "elrs", allow.CMD_NAV_LAND, [])
        w.run(4)
        no_land = b1.mode() != MODE["LAND"]
        w.m.ccfg["uav_land_ms"] = 0.05                     # the descent energy now larger than what is left
        w.m.to_worker(w.m._config())
        ev = w.run(60, lambda: next((e for e in w.m.land_events if e[1] == 1 and e[2] == "start"), None))
        landing = w.run(10, lambda: b1.mode() == MODE["LAND"])
        down = w.run(120, lambda: not b1.armed())
        only, t_rule, lands = land_after_rule(w, 1)
        r_after = cmd(w, 1, "elrs", allow.CMD_DO_SET_MODE, [1, allow.COPTER_LOITER]) if not down else ("down", "")
        report("SI10 the battery minimum = the ONLY landing: a manager NAV_LAND refused (allow-list), no ArduPilot "
               "landing before; the router's own rule lands the drone where it is (LAND), it touches down and "
               "disarms; every LAND came after the rule (BATT_FS_LOW_ACT 0 = ArduPilot only warns)",
               r_land[0] is None and no_land and ev and landing and down and only and lands,
               "manager LAND %s; no land before %s; rule %s; LAND %s; down %s; LANDs %s after rule %s; after %s" % (
                   r_land, no_land, ev and ev[3], bool(landing), bool(down), [round(t - t10, 1) for t in lands],
                   t_rule and round(t_rule - t10, 1), r_after))
        # ---- SI11 the .tlog ----------------------------------------------------------------------------------------
        w.run(3)
        tl_dir = os.path.join(w.root, "uav_logs")
        files = []
        for dp, _dn, fn in os.walk(tl_dir):
            files += [os.path.join(dp, f) for f in fn if f.endswith(".tlog")]
        okt, detail = check_tlogs(files, [c for _n, c, _p in bad])
        report("SI11 the SITL-made .tlog (one per flight) read by pymavlink: every packet, timestamps in order, the "
               "router's commands in it, no denied command / PARAM_SET in it", okt, detail)
        # ---- SI12 never a parameter write -------------------------------------------------------------------------
        p_after = b1.read_params(list(p_before))
        diff = {n: (p_before[n], p_after.get(n)) for n in p_before if p_before[n] != p_after.get(n)}
        missing = [n for n, v in p_before.items() if v is None and n in [r["name"] for r in
                                                                          registry.load_param_file()[0]
                                                                          if not r["present_only"]]]
        report("SI12 the gateway never wrote a parameter: every SAFETY parameter of the flown SITL copter is the same "
               "after all the flights (read by the bench)", not diff and not missing and len(p_before) > 50,
               "%d read, changed %s, missing %s" % (len(p_before), diff, missing))
        print("suite one: %.0f s" % (time.time() - t_start), flush=True)
    finally:
        w.close()


def check_tlogs(files, denied_cmds):
    try:
        from pymavlink import mavutil
    except ImportError:
        return NA, "pymavlink not installed"
    if not files:
        return False, "no .tlog written"
    n, bad_order, types, denied_seen, param_set = 0, 0, set(), [], 0
    for f in files:
        m = mavutil.mavlink_connection(f)
        last = 0.0
        while True:
            x = m.recv_match(blocking=False)
            if x is None:
                break
            t = x.get_type()
            if t == "BAD_DATA":
                continue
            n += 1
            types.add(t)
            if x._timestamp < last:
                bad_order += 1
            last = x._timestamp
            if t == "COMMAND_LONG" and int(x.command) in denied_cmds and int(x.command) not in (21, 176, 400):
                denied_seen.append(int(x.command))
            if t == "COMMAND_LONG" and int(x.command) == 400 and abs(x.param2 - 21196) < 1:
                denied_seen.append(400)
            if t == "COMMAND_LONG" and int(x.command) == 176 and int(x.param2) in (6, 21):
                denied_seen.append(176)
            if t == "PARAM_SET":
                param_set += 1
    ok = n > 100 and bad_order == 0 and "COMMAND_LONG" in types and "HEARTBEAT" in types and \
        "GLOBAL_POSITION_INT" in types and not denied_seen and not param_set
    return ok, "%d files, %d packets, out of order %d, types %d (%s ...), denied in it %s, PARAM_SET %d" % (
        len(files), n, bad_order, len(types), sorted(types)[:8], denied_seen, param_set)


# ------------------------------------------------------------------------------------- the no-GPS fallback run ---

def suite_fallback(exe):
    w = World(exe, 1, launch={"gps_wait_s": 5.0}, prefix="sitlf_")
    try:
        ok = w.ready()
        b = w.copters[1].bench
        regs = w.register([1])
        # "no sky in the hatch": the simulated GPS off after the registration (a real GPS in a closed magazine);
        # the fallback's ARM then needs no position (FENCE_ENABLE 0 in docs/uav_params.param)
        gps_off = b.set_param("SIM_GPS1_ENABLE", 0)
        nofix = w.run(30, lambda: (w.drone(1).get("gps_fix") or 0) < 3)
        w.run(10)
        cop = w.gw.stations.copilot_st
        w.m.route_msg(cop, {"t": "uav_route", "target": 1, "wps": [wps_out(60.0, 0.0, 40.0)]})
        w.m.prepare_msg(cop, {"t": "uav_launch_prepare", "count": 1})
        t0 = time.time()
        end = w.run(90, lambda: w.m.seq.state in ("DONE", "STOPPED"))
        how = "the launch sequence"
        acks = None
        if upload_blocked(w):
            # the launch stops at the upload (the MISSION_REQUEST finding): the fallback's own commands through the
            # real router + allow.py, exactly as launch.py sends them (ALT_HOLD, ARM with the hatch OPEN, TAKEOFF
            # param3 = 1 to fallback_alt 15 m)
            how = "the fallback's commands through the router (the launch stops at the mission upload)"
            w.m.hatch_cmd(True, lambda ok, why: None)
            w.run(20, lambda: w.m.hatch.state == "OPEN")
            t0 = time.time()
            acks = (cmd(w, 1, "usb", allow.CMD_DO_SET_MODE, [1, allow.COPTER_ALT_HOLD]),
                    cmd(w, 1, "usb", allow.CMD_COMPONENT_ARM_DISARM, [1, 0]),
                    cmd(w, 1, "usb", allow.CMD_NAV_TAKEOFF, [0, 0, 1, 0, 0, 0, 15.0]))
            w.run(10)
            motors = [getattr(b.servo, "servo%d_raw" % i, 0) for i in range(1, 5)] if b.servo else None
            # launch.py's no-climb rule: DISARM on deck (never forced) - tried 4 times over ~15 s
            for _k in range(4):
                if not b.armed() or w.max_alt.get(1, 0.0) >= 2.0:
                    break
                acks = acks + (cmd(w, 1, "auto", allow.CMD_COMPONENT_ARM_DISARM, [0, 0]),)
                w.run(3)
            t_obs = time.time()
            w.run(30, lambda: not b.armed())
            disarm_t = next((round(t - t_obs, 1) for t, _m, a in b.modes if t >= t_obs and not a), None)
            acks = acks + (("motors after 10 s", motors), ("disarmed by itself after the DISARMs (s)", disarm_t))
        modes = modes_seen(b, t0)
        armed_ever = any(a for _t, _m, a in b.modes if _t >= t0)
        climbed = w.max_alt.get(1, 0.0) > 5.0
        safe = not b.armed() and not climbed
        report("SI13 the no-GPS climb fallback (W59 / W68) MEASURED with ArduCopter SITL (%s): no fix -> ALT_HOLD, ARM, "
               "TAKEOFF param3 = 1 to 15 m with the RC receiver ignored; the drone ends safe (a drone that does not "
               "climb is disarmed on deck; one that climbs hovers until GPS)" % how,
               bool(ok and gps_off and nofix and (regs.get(1) or {}).get("ok") and armed_ever and
                    (safe or climbed)),
               "ready %s, gps off %s, no fix %s; registered %s; sequence %s (%s); acks %s; armed %s; climbed %s (max "
               "alt %s); modes %s; notices %s" % (
                   bool(ok), gps_off, bool(nofix), (regs.get(1) or {}).get("ok"), end, w.m.seq.why, acks,
                   armed_ever, climbed, w.max_alt.get(1), modes, [n["text"] for n in w.m.notices][-3:]))
        finding("W68 / W59 (no GPS at the launch, ArduCopter %s SITL): %s" % (
            (w.m.reg.recs.get(1) or {}).get("fw"),
            ("ALT_HOLD + MAV_CMD_NAV_TAKEOFF param3 = 1: acks %s - the copter does NOT climb with the RC receiver "
             "ignored (ArduCopter's ALT_HOLD take-off needs pilot throttle); the launch's no-climb DISARM (never forced) "
             "-> %s; armed at the end: %s - the climb fallback does not work: owner decision W138" % (
                 [a[0] for a in acks] if acks else "-", [a[1] for a in acks[3:]] if acks else "-", b.armed())) if
            armed_ever and not climbed else
            ("the ALT_HOLD take-off climbs with the RC receiver ignored (max %.1f m)" % w.max_alt.get(1, 0.0)
             if climbed else "the copter did not even ARM in ALT_HOLD without GPS (acks %s)" % (
                 [a[0] for a in acks] if acks else "-"))))
        finding("FENCE (SITL): an enabled fence refuses ARM without a GPS position (also FENCE_TYPE 1) - "
                "docs/uav_params.param has FENCE_ENABLE 0 (the fence only reports with FENCE_ACTION 0)")
    finally:
        w.close()


# ------------------------------------------------------------------------------------------------ six copters ---

def suite_six(exe):
    t_start = time.time()
    w = World(exe, 6, bad={6: [("FENCE_ACTION", 1)]}, prefix="sitl6_")
    try:
        ok = w.ready(200)
        if not ok:
            report("SI14 six SITL copters ready", False, "not ready in 200 s: %s %s" % (
                w.m.state, [c.bench.ready() for c in w.copters.values()]))
            return
        regs = w.register(list(range(1, 7)), 60)
        good = [s for s in range(1, 6) if (regs.get(s) or {}).get("ok")]
        r6 = regs.get(6) or {}
        cop = w.gw.stations.copilot_st
        for s in range(1, 6):
            w.m.route_msg(cop, {"t": "uav_route", "target": s,
                                "wps": [wps_out(60.0 + 25.0 * s, -50.0 + 25.0 * s, 40.0 + 5.0 * s)]})
        w.m.prepare_msg(cop, {"t": "uav_launch_prepare", "count": 5})
        t0 = time.time()
        end = w.run(420, lambda: w.m.seq.state in ("DONE", "STOPPED"))
        arms = {}
        for s in range(1, 6):
            arms[s] = next((t for t, _m, a in w.copters[s].bench.modes if a and t >= t0), None)
        order = [s for s, _t in sorted(((s, t) for s, t in arms.items() if t is not None), key=lambda x: x[1])]
        gaps = [round(arms[b] - arms[a], 1) for a, b in zip(order, order[1:])]
        gap_s = float(w.m.seq.cfg["gap_s"])
        blocked = upload_blocked(w)
        report("SI14a six SITL copters, registration behind the real router: 5 registered, slot 6 (FENCE_ACTION 1) "
               "refused naming the parameter", good == [1, 2, 3, 4, 5] and r6.get("ok") is False and
               any("FENCE_ACTION" in p for p in r6.get("problems") or []),
               "registered %s, slot 6 %s" % (good, r6.get("problems")))
        report("SI14b the launch of 5 strictly in series (slot order, >= gap_s %.0f s between the ARMs), the hatch "
               "closed after the last" % gap_s,
               end == "DONE" and order == [1, 2, 3, 4, 5] and all(g >= gap_s - 0.5 for g in gaps) and
               w.m.hatch.state == "CLOSED",
               "sequence %s %s; ARM order %s gaps %s; hatch %s%s" % (end, w.m.seq.why, order, gaps, w.m.hatch.state,
                                                                  ("; " + UPLOAD_WHY) if blocked else ""))
        if blocked:
            for s in range(1, 6):
                fly_cmds(w, s, 30.0 + 5.0 * (s - 1))
            w.m.hatch_cmd(False, lambda ok, why: None)
        w.run(20)
        up = [s for s in range(1, 6) if w.copters[s].bench.armed() and (w.copters[s].bench.alt() or 0) > 20]
        # every radio link cut 15 s: all hold
        alts0 = {s: w.copters[s].bench.alt() for s in up}
        m0 = {s: w.copters[s].bench.mode() for s in up}
        tc = time.time()
        for s in up:
            w.copters[s].elrs.cut = True
        w.run(15)
        held = [s for s in up if w.copters[s].bench.mode() == m0[s] and w.copters[s].bench.armed() and
                abs((w.copters[s].bench.alt() or 0) - (alts0[s] or 0)) < 3.0]
        for s in up:
            w.copters[s].elrs.cut = False
        never = [s for s in range(1, 7) if set(modes_seen(w.copters[s].bench, t0)) & set(NEVER + (MODE["LAND"],))]
        report("SI14c all 5 flying copters HOLD when every radio link is cut 15 s (mode, altitude); no RTL / LAND "
               "anywhere%s" % (" (flown by GUIDED / ARM / TAKEOFF: no mission upload)" if blocked else ""),
               len(up) == 5 and held == up and not never,
               "up %s; held %s; never-modes %s; %.0f s" % (up, held, never, time.time() - t_start))
        _ = tc
    finally:
        w.close()


# ---------------------------------------------------------------------------------------------------------------

def na_all(why):
    for t in ("SI1 registration", "SI2 GPS warm start", "SI3 route + launch sequence", "SI4 mission flown",
              "SI5 in-flight mission change", "SI6 link loss = hold", "SI7 manual assisted flying + give back",
              "SI8 GPS lost in the air = ALT_HOLD", "SI9 allow-list vs a real ArduPilot", "SI10 battery minimum",
              "SI11 SITL .tlog via pymavlink", "SI12 never a parameter write", "SI13 no-GPS climb fallback",
              "SI14a six copters: registration", "SI14b six copters: launch in series", "SI14c six copters: hold"):
        report(t, NA, why)


def main():
    exe = None if os.environ.get("TRYGONS_SITL_NA") else toolpaths.sitl()     # (the N/A path test: UA41)
    only = [a for a in sys.argv[1:] if not a.startswith("-")]
    if exe is None:
        na_all("ArduPilot SITL not installed / not verified (python tools/get_sitl.py) - N/A, never a PASS")
        print("SITL TESTS: N/A (not installed)")
        return 0
    try:
        import pymavlink  # noqa: F401
    except ImportError:
        na_all("pymavlink missing (the bench side needs it: pip install pymavlink)")
        print("SITL TESTS: N/A (no pymavlink)")
        return 0
    print("SITL: %s" % exe, flush=True)
    for name, fn in (("one", suite_one), ("fallback", suite_fallback), ("six", suite_six)):
        if only and name not in only:
            continue
        t0 = time.time()
        try:
            fn(exe)
        except Exception as e:
            import traceback
            traceback.print_exc()
            report("suite %s crashed" % name, False, "%s: %s" % (type(e).__name__, e))
        print("suite %s: %.0f s" % (name, time.time() - t0), flush=True)
    print("\nFINDINGS (measured answers to the 'TO CONFIRM in SITL' items):")
    for f in FINDINGS:
        print("  - %s" % f)
    fails = sum(1 for r in results if r is False)
    passed = sum(1 for r in results if r)
    nas = sum(1 for r in results if r is None)
    print("\nSITL TESTS: %d passed, %d failed, %d N/A" % (passed, fails, nas))
    if fails:
        print("SITL TESTS FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
