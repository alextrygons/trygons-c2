"""Gateway tests on the PC: the simulator (sim/autopilot_sim server) plays the
AURIX, the real gateway code runs here like on the Pi.

    python test_gateway.py            all sections
    python test_gateway.py A C        only some sections

Exit code 0 = all passed.
"""
import glob
import math
import re
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import asyncio

HERE = os.path.dirname(os.path.abspath(__file__))
# the ground station in English with a throw-away settings file (never the user's language / HUD settings)
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")                    # silent: no sound, no voice making
os.environ.setdefault("TRYGONS_GS_SETTINGS", os.path.join(__import__("tempfile").gettempdir(), "trygons_gs_test.json"))
sys.path.insert(0, HERE)
from common import load_config, proto, setup_logging  # noqa: E402
import gateway as gwmod  # noqa: E402

sys.path.insert(0, os.path.join(HERE, "..", "groundstation"))
from link import Link, Waiter  # noqa: E402

SIM_DIR = os.path.join(HERE, "..", "sim")
# AUTOPILOT_SIM: another simulator build (e.g. while the demo runs from sim/)
EXE = os.environ.get("AUTOPILOT_SIM") or os.path.join(SIM_DIR, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
results = []


def report(name, ok, detail):
    results.append((name, ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def free_port():
    while True:
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        try:                                    # the gateway port (p + 2) must be free too
            t = socket.socket()
            t.bind(("127.0.0.1", p + 2))
            t.close()
            return p
        except OSError:
            continue


class Sim:
    """autopilot_sim server: ground station on port, gateway UART on port + 2."""

    def __init__(self, speed=10.0, *args):
        self.port = free_port()
        self.speed = speed
        self.out = open(os.path.join(tempfile.gettempdir(), "gwtest_sim_%d.log" % self.port), "w")
        self.p = subprocess.Popen([EXE, "server", str(self.port), str(speed)] + list(args), cwd=SIM_DIR,
                                  stdout=self.out, stderr=subprocess.STDOUT)   # never a full pipe
        for _ in range(100):
            try:
                socket.create_connection(("127.0.0.1", self.port + 2), timeout=0.2).close()
                break
            except OSError:
                time.sleep(0.05)
        time.sleep(0.3)

    @property
    def gw_addr(self):
        return "tcp:127.0.0.1:%d" % (self.port + 2)

    def station(self):
        link = Link("tcp:127.0.0.1:%d" % self.port, log=False, heartbeat_s=0.5 / self.speed)
        link.start()
        return link, Waiter(link)

    def stop(self):
        self.p.terminate()
        try:
            self.p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.p.kill()
        self.out.close()


class GwRunner:
    """The gateway in a thread with its own asyncio loop (like the service)."""

    def __init__(self, cfg):
        self.gw = gwmod.Gateway(cfg)
        self.loop = asyncio.new_event_loop()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.gw.run())

    def call(self, fn, *a):
        """Run fn in the gateway's loop and return its result."""
        fut = asyncio.run_coroutine_threadsafe(self._wrap(fn, *a), self.loop)
        return fut.result(10)

    @staticmethod
    async def _wrap(fn, *a):
        r = fn(*a)
        if asyncio.iscoroutine(r):
            r = await r
        return r

    def stop(self):
        self.loop.call_soon_threadsafe(self.gw.stop)
        self.th.join(10)


def gw_cfg(sim, root, **extra):
    base = {"aurix": sim.gw_addr, "set_clock": False, "status_s": 1.0 / sim.speed,   # 1 Hz boat time
            "channels": {"driver": "sim", "starlink_status": False},
            "cameras": {"enabled": False}, "ais": {"enabled": False}, "vision": {"enabled": False},
            "storage": {"root": root, "require_mount": False, "check_s": 1, "low_free_gb": 0.0,
                        "full_free_gb": 0.0}}
    from common import deep_merge
    return load_config(None, deep_merge(base, extra))


def records(path):
    """All (type, t, bytes) records of a TLG file."""
    out = []
    with open(path, "rb") as f:
        b = f.read()
    i = 4
    while i + 10 <= len(b):
        if b[i] != 0xA5:
            i += 1
            continue
        ln = b[i + 2] | (b[i + 3] << 8)
        if i + 10 + ln > len(b):
            break
        out.append((b[i + 1], struct.unpack_from("<I", b, i + 4)[0], bytes(b[i:i + 10 + ln])))
        i += 10 + ln
    return out


def sd_files(boot):
    return sorted(glob.glob(os.path.join(SIM_DIR, "sdcard", "B%05d_*.TLG" % boot)))


def boot_of(path):
    for t, _, r in records(path):
        if t == 1:
            import re
            m = re.search(rb"boot=(\d+)", r)
            return int(m.group(1)) if m else None
    return None


def until(cond, timeout, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        v = cond()
        if v:
            return v
        time.sleep(step)
    return None


# ------------------------------------------------------------------- section A --

def section_a():
    print("\n=== Section A: gateway architecture ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_")
    try:
        # A1/A2 full log copy, compared byte for byte with the AURIX SD card
        sim = Sim(10.0)
        root = os.path.join(tmp, "ssd1")
        os.makedirs(root)
        gw = GwRunner(gw_cfg(sim, root))
        link, w = sim.station()
        w.until(lambda d: d["mode"] == 0, 5)
        time.sleep(25)                                  # 250 s of boat time
        st = gw.gw.status()
        t_sim = gw.gw.telem["t_ms"] if gw.gw.telem else 0
        gw.stop()
        link.stop()
        time.sleep(0.5)
        pi = sorted(glob.glob(os.path.join(root, "aurix_logs", "*", "P*.TLG")))
        pi_data = [r for p in pi for (t, _, r) in records(p) if t == 0x10]
        boot = boot_of(pi[0]) if pi else None
        sd = sd_files(boot) if boot is not None else []
        sd_data = [r for p in sd for (t, _, r) in records(p) if t == 0x10]
        # the same sequence (the SD side may hold a few more at the end / start)
        common = 0
        if pi_data and sd_data and pi_data[0] in sd_data:
            k = sd_data.index(pi_data[0])
            for a, b in zip(pi_data, sd_data[k:]):
                if a != b:
                    break
                common += 1
        same = pi_data and common == len(pi_data)
        report("A1 the Pi holds a complete copy of the AURIX log (byte for byte = SD card)",
               bool(same) and len(pi_data) > 4000 and st["logs"]["gaps"] == 0 and st["logs"]["broken"] == 0,
               "%d Pi files, %d data records, %d identical to the SD card (%d there), gaps %d, broken %d, "
               "boat time %.0f s, %.2f MB" % (len(pi), len(pi_data), common, len(sd_data), st["logs"]["gaps"],
                                             st["logs"]["broken"], t_sim / 1000, st["logs"]["mb"]))
        # A7 time
        c = st["clock"]
        report("A7 the gateway takes its time from the AURIX GPS",
               c["gps_valid"] and abs(c["offset_s"]) > 1000,
               "GPS valid %s, clock %s (offset to this PC %.0f s; on the Pi the clock itself is set)" % (
                   c["gps_valid"], c["utc"], c["offset_s"]))
        sim.stop()

        # A2 slow UART: the log copy loses records (noted), the SD card and control traffic don't
        sim = Sim(5.0, "gwbaud=38400")
        root = os.path.join(tmp, "ssd2")
        os.makedirs(root)
        gw = GwRunner(gw_cfg(sim, root))
        link, w = sim.station()
        n0 = [0]
        w.on_telemetry = lambda d: n0.__setitem__(0, n0[0] + 1)
        w.until(lambda d: False, 12)                    # 60 s of boat time
        st = gw.gw.status()
        gw.stop()
        sd = sd_files(boot_of(sorted(glob.glob(os.path.join(root, "aurix_logs", "*", "P*.TLG")))[0]))
        ts = [t for p in sd for (k, t, _) in records(p) if k == 0x10]
        holes = sum(1 for a, b in zip(ts, ts[1:]) if b - a > 60)
        gaps_csv = glob.glob(os.path.join(root, "aurix_logs", "*", "gaps.csv"))
        rate = n0[0] / 12.0 / sim.speed * 10                    # telemetry frames per boat second x10
        report("A2 slow UART: records for the Pi dropped first and noted; SD card and control traffic complete",
               st["logs"]["gaps"] > 0 and gaps_csv and holes == 0 and n0[0] >= 0.9 * 12 * 10 * sim.speed,
               "38400 baud: Pi copy %d records, %d gaps noted in gaps.csv (%d missing); SD card %d data records, "
               "holes %d; telemetry %d frames in 60 s boat time" % (
                   st["logs"]["records"], st["logs"]["gaps"], st["logs"]["missing"], len(ts), holes, n0[0]))
        link.stop()
        sim.stop()

        # A6 gateway lost / A5 storage
        sim = Sim(5.0)
        root = os.path.join(tmp, "ssd3")
        os.makedirs(root)
        gw = GwRunner(gw_cfg(sim, root))
        link, w = sim.station()
        ok_state = w.until(lambda d: d.get("gw_state") == 1, 5)
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        hdg = w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 3)
        gw.gw.storage.fail_inject = True
        gw.gw.storage.check(force=True)
        stor = w.until(lambda d: d["alarm_bits"] & 0x20000, 3)
        report("A5 SSD failing -> alarm GATEWAY STORAGE on the boat, everything else keeps running",
               stor is not None and stor["mode"] == proto.MODE_HDG_LOCK and gw.gw.aurix.connected,
               "alarm %s, mode %s, gateway still connected %s, storage: %s" % (
                   stor is not None, stor and stor["mode_name"], gw.gw.aurix.connected, gw.gw.storage.reason))
        gw.stop()                                       # Pi crash / unplugged
        t0 = time.time()
        lost = w.until(lambda d: d["alarm_bits"] & 0x10000, 3)
        dt = (time.time() - t0) * sim.speed
        cmd = w.command(proto.T_SEA_STATE, proto.p_sea_state(3))
        report("A6 gateway lost -> alarm GATEWAY LOST, no mode change, local USB link still works",
               ok_state is not None and hdg is not None and lost is not None and lost["mode"] == proto.MODE_HDG_LOCK
               and lost.get("gw_state") == 2 and cmd is not None and cmd["ok"],
               "gateway OK before %s; alarm after ~%.1f s boat time, mode %s, gateway state %s; USB command %s" % (
                   ok_state is not None, dt, lost and lost["mode_name"], lost and lost.get("gw_state_name"),
                   cmd and cmd["text"]))
        link.stop()
        sim.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # GA5 (R20): "AURIX alive" only from a frame that passed the CRC, never from line noise
    import aurix
    a = aurix.AurixLink("tcp:127.0.0.1:1")
    a._feed(os.urandom(4000))
    noise = a.last_rx
    a._feed(proto.encode(proto.T_HEARTBEAT, 1, b""))
    report("GA5 AURIX alive only on a valid frame (line noise doesn't count)", noise == 0.0 and a.last_rx > 0.0,
           "4000 noise bytes: alive time %s; one good frame: alive %s" % (noise, a.last_rx > 0.0))


# ------------------------------------------------------------------- section B --

from users import hash_pw, hash_key  # noqa: E402
from gwclient import GatewayLink  # noqa: E402
from netsim import NetSim, PROFILES  # noqa: E402
import json as _json  # noqa: E402
import urllib.request  # noqa: E402
import base64 as _b64  # noqa: E402

PW = {"alice": "alice-password-1", "bob": "bob-password-22", "vic": "viewer-password"}
KIOSK_KEY = "kiosk-key-0123456789abcdefghij"


def users_cfg():
    return {"alice": {"role": "HELM", "pw": hash_pw(PW["alice"]), "licence": True},
            "bob": {"role": "HELM", "pw": hash_pw(PW["bob"]), "licence": True},
            "vic": {"role": "VIEWER", "pw": hash_pw(PW["vic"])},
            "kiosk": {"role": "HELM", "keys": [hash_key(KIOSK_KEY)]}}


def gw_cfg_b(sim, root, **extra):
    st_port, web_port = free_port(), free_port()
    from common import deep_merge
    base = {"stations": {"listen": "127.0.0.1", "port": st_port, "local_nets": ["127.0.0.2/32"]},
            "web_port": web_port, "users": users_cfg(), "audit_fallback": os.path.join(root, "audit_fb")}
    return gw_cfg(sim, root, **deep_merge(base, extra))


class Client:
    """A ground station on the gateway (gwclient.GatewayLink) + its answers."""

    def __init__(self, port, user, pw=None, key=None, local=False, clock_offset=0.0, name=None, crew=None, boat=None):
        self.l = GatewayLink("127.0.0.1:%d" % port, user, password=pw, key=key, station=name or user,
                             local_addr="127.0.0.2" if local else "127.0.0.1", clock_offset=clock_offset,
                             crew=crew, boat=boat)
        self.welcome = self.l.start()
        self.acks = []
        self.telem = None
        self.n_tel = 0
        self.lite = 0
        self.status = None

    def pump(self, t=0.05):
        end = time.time() + t
        while True:
            try:
                kind, d = self.l.events.get(timeout=max(0.0, end - time.time()))
            except Exception:
                return
            if kind == "telemetry":
                self.telem = d
                self.n_tel += 1
                self.lite += 1 if d.get("lite") else 0
            elif kind == "ack":
                self.acks.append(d)
            elif kind == "gw_status":
                self.status = d

    def wait_ack(self, seq, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            for a in self.acks:
                if a["seq"] == seq:
                    self.acks.remove(a)
                    return a
            self.pump(0.05)
        return None

    def cmd(self, ftype, payload=b"", timeout=3.0):
        return self.wait_ack(self.l.send(ftype, payload), timeout)

    def raw_cmd(self, seq, ts, ftype, payload=b"", sid=None, timeout=3.0):
        self.l._send_obj({"t": "cmd", "sid": sid or self.l.welcome["sid"], "seq": seq, "ts": ts, "type": ftype,
                          "p": payload.hex()})
        return self.wait_ack(seq, timeout)

    def result(self, a):
        return None if a is None else a["result"]

    def close(self):
        self.l.stop()


def http(port, path, user, pw, body=None):
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), data=body,
                                 headers={"Authorization": "Basic " + _b64.b64encode(("%s:%s" % (user, pw)).encode()).decode(),
                                          "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


SEA = proto.p_sea_state(3)


def section_b():
    print("\n=== Section B: users, roles, security ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_b_")
    try:
        sim = Sim(2.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root)
        gw = GwRunner(cfg)
        sp, wp = cfg["stations"]["port"], cfg["web_port"]
        time.sleep(1.0)

        # B1 login
        errs = []
        for user, pw in (("alice", "wrong"), ("nobody", "x")):
            try:
                Client(sp, user, pw).close()
                errs.append("ACCEPTED")
            except ConnectionError as e:
                errs.append(str(e))
        for _ in range(4):
            try:
                Client(sp, "bob", "wrong-again")
            except ConnectionError as e:
                last = str(e)
        try:
            Client(sp, "bob", PW["bob"])
            locked = "NOT locked"
        except ConnectionError as e:
            locked = str(e)
        gw.gw.users.fails.clear()                               # the lock-out is per address
        kiosk = Client(sp, "kiosk", key=KIOSK_KEY)
        report("B1 login: wrong password / unknown user refused, repeated failures lock out, key login works",
               errs[0].startswith("wrong") and errs[1].startswith("wrong") and "too many" in locked and
               kiosk.welcome["role"] == "HELM",
               "wrong password: %s; unknown user: %s; 6th try with the right password: %s; key login: %s" % (
                   errs[0], errs[1], locked, kiosk.welcome["role"]))
        kiosk.close()
        gw.gw.users.fails.clear()

        # B2 viewer + B3 HELM rules through the gateway
        vic = Client(sp, "vic", PW["vic"])
        alice = Client(sp, "alice", PW["alice"])                    # remote
        bob = Client(sp, "bob", PW["bob"])                          # remote
        bob_local = Client(sp, "bob", PW["bob"], local=True, name="bob on board")
        alice.pump(1.0)
        vic.pump(0.5)
        r_vcmd = vic.result(vic.cmd(proto.T_ALARM_ACK, b""))
        r_vhelm = vic.result(vic.cmd(proto.T_HELM, proto.p_helm(0)))
        r_vread = vic.result(vic.cmd(proto.T_JOYMAP_GET))
        report("B2 VIEWER sees everything but can't command or take HELM",
               r_vcmd == 100 and r_vhelm == 100 and r_vread == 0 and vic.n_tel > 3,
               "command: %s, take HELM: %s, read: %s, telemetry frames %d" % (r_vcmd, r_vhelm, r_vread, vic.n_tel))
        link, w = sim.station()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 3)
        r1 = alice.result(alice.cmd(proto.T_ALARM_ACK, b""))
        r2 = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        r3 = alice.result(alice.cmd(proto.T_ALARM_ACK, b""))
        r4 = bob.result(bob.cmd(proto.T_HELM, proto.p_helm(0)))
        time.sleep(1.2)
        vic.pump(0.3)
        seen = vic.status["helm"]["name"] if vic.status else "?"
        r5 = bob_local.result(bob_local.cmd(proto.T_HELM, proto.p_helm(0)))
        r6 = alice.result(alice.cmd(proto.T_ALARM_ACK, b""))
        r7 = bob_local.result(bob_local.cmd(proto.T_ALARM_ACK, b""))
        time.sleep(1.2)
        alice.pump(0.3)
        seen2 = alice.status["helm"]["name"] if alice.status else "?"
        report("B3 only the HELM station commands; one HELM; a local station takes it from a remote one; "
               "shown on every station",
               (r1, r2, r3, r4, r5, r6, r7) == (21, 0, 0, 117, 0, 0, 0) and "alice" in seen and "bob on board" in seen2,
               "alice (captain) without HELM %s, takes %s, command %s; bob (remote copilot) takes %s (117: only the "
               "captain); viewer sees HELM = %s; bob on board takes %s (becomes the captain); alice now copilot: her "
               "command runs on the captain's behalf %s; bob's command %s; alice sees HELM = %s" % (
                   r1, r2, r3, r4, seen, r5, r6, r7, seen2))
        bob_local.cmd(proto.T_HELM, proto.p_helm(1))

        # B2b replay / duplicate / out of order / stale / future / rate / wrong session
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        s0 = alice.l.seq + 100
        alice.l.seq = s0 + 1000
        now = alice.l.now()
        ok1 = alice.result(alice.raw_cmd(s0, now, proto.T_ALARM_ACK, b""))
        dup = alice.result(alice.raw_cmd(s0, now, proto.T_ALARM_ACK, b""))
        old = alice.result(alice.raw_cmd(s0 - 5, now, proto.T_ALARM_ACK, b""))
        stale = alice.result(alice.raw_cmd(s0 + 1, now - 3.0, proto.T_ALARM_ACK, b""))
        fut = alice.result(alice.raw_cmd(s0 + 2, now + 5.0, proto.T_ALARM_ACK, b""))
        sess = alice.result(alice.raw_cmd(s0 + 3, now, proto.T_ALARM_ACK, b"", sid="0123456789abcdef"))
        for i in range(30):                                     # 30 at once, no waiting
            alice.l._send_obj({"t": "cmd", "sid": alice.l.welcome["sid"], "seq": s0 + 10 + i, "ts": alice.l.now(),
                               "type": proto.T_ALARM_ACK, "p": ""})
        rates = [alice.result(alice.wait_ack(s0 + 10 + i, 2.0)) for i in range(30)]
        n_rate = rates.count(105)
        # a PC with a clock 1 hour wrong: its commands are NOT stale (offset measured)
        skew = Client(sp, "kiosk", key=KIOSK_KEY, clock_offset=3600.0)
        skew.pump(2.5)
        r_skew = skew.result(skew.cmd(proto.T_JOYMAP_GET))
        skew.close()
        alice.l.seq = s0 + 2000
        alice.cmd(proto.T_HELM, proto.p_helm(1))
        report("B2 duplicate, out-of-order, stale, future, wrong-session and too-fast commands refused",
               ok1 == 0 and dup == 101 and old == 102 and stale == 103 and fut == 104 and sess == 107 and
               n_rate >= 5 and r_skew == 0,
               "first %s, same seq again %s, older seq %s, 3 s old %s, 5 s in the future %s, old session %s; "
               "30 at once: %d rate limited; PC clock 1 h wrong: command %s (clock offset measured)" % (
                   ok1, dup, old, stale, fut, sess, n_rate, r_skew))
        time.sleep(2.5)

        # B5 remote link lost (silent: the link just stops) while holding HELM
        proxy_port = free_port()
        net = NetSim(proxy_port, sp, PROFILES["4g"]).start()
        carol = Client(proxy_port, "alice", PW["alice"], name="alice via 4G")
        carol.pump(1.5)
        r_take = carol.result(carol.cmd(proto.T_HELM, proto.p_helm(0)))
        net.forced_down = True                                  # the 4G link dies
        lost = w.until(lambda d: d["alarm_bits"] & 0x40000, 10)
        report("B5 remote HELM station's link lost -> REMOTE LINK LOST, HELM to nobody, no mode change",
               r_take == 0 and lost is not None and lost.get("helm") == 0 and lost["mode"] == proto.MODE_HDG_LOCK,
               "took HELM %s; after the link died: alarm %s, HELM %s, mode %s" % (
                   r_take, lost is not None, lost and lost.get("helm"), lost and lost["mode_name"]))
        net.forced_down = False
        net.stop()

        # B6 web page
        code_page, page = http(wp, "/", "vic", PW["vic"])
        code_st, st_body = http(wp, "/api/status", "vic", PW["vic"])
        code_bad, _ = http(wp, "/api/status", "vic", "wrong")
        code_cmd, cmd_body = http(wp, "/api/cmd", "alice", PW["alice"], _json.dumps({"cmd": "standby"}).encode())
        st_json = _json.loads(st_body) if code_st == 200 else {}
        report("B6 iPhone web page: view only by default, login needed",
               code_page == 200 and b"Trygons" in page and code_st == 200 and st_json.get("mode") and
               code_bad == 401 and code_cmd == 403 and not st_json.get("web_helm"),
               "page %d, status %d (mode %s, HELM %s), wrong password %d, HELM user command %d (%s)" % (
                   code_page, code_st, st_json.get("mode"), st_json.get("helm"), code_bad, code_cmd,
                   _json.loads(cmd_body).get("text")))
        gw.gw.web.helm_allowed = True                           # switched on in gateway.json
        c1, b1 = http(wp, "/api/cmd", "alice", PW["alice"], _json.dumps({"cmd": "helm_take"}).encode())
        c2, b2 = http(wp, "/api/cmd", "alice", PW["alice"], _json.dumps({"cmd": "standby"}).encode())
        mode_after = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 3)
        c3, b3 = http(wp, "/api/cmd", "vic", PW["vic"], _json.dumps({"cmd": "standby"}).encode())
        report("B6b web HELM only when switched on, only for HELM users, through the same checks",
               c1 == 200 and _json.loads(b1)["ok"] and _json.loads(b2)["ok"] and mode_after is not None and c3 == 403,
               "take HELM: %s, STANDBY: %s -> mode %s; viewer: %d" % (
                   _json.loads(b1)["text"], _json.loads(b2)["text"], mode_after and mode_after["mode_name"], c3))
        http(wp, "/api/cmd", "alice", PW["alice"], _json.dumps({"cmd": "helm_release"}).encode())

        # B4 adaptive telemetry + latency per station
        pslow = free_port()
        slow_net = NetSim(pslow, sp, PROFILES["slow"]).start()
        slow = Client(pslow, "vic", PW["vic"], name="vic far away")
        fast = Client(sp, "vic", PW["vic"], name="vic near")
        slow.pump(6.0)
        fast.pump(0.1)
        slow.n_tel = slow.lite = fast.n_tel = 0
        t0 = time.time()
        while time.time() - t0 < 5.0:
            slow.pump(0.1)
            fast.pump(0.1)
        fast.pump(0.5)
        rtts = {s["name"]: s["rtt_ms"] for s in (fast.status or {}).get("stations", [])}
        rate_slow, rate_fast = slow.n_tel / 5.0, fast.n_tel / 5.0 / sim.speed
        report("B4 adaptive telemetry: full rate on a good link, 2 Hz essentials on a slow one; latency per station",
               slow.lite == slow.n_tel and 1.5 <= rate_slow <= 2.6 and fast.lite == 0 and rate_fast >= 9.0 and
               rtts.get("vic far away", 0) > 400 and rtts.get("vic near", 999) < 100,
               "slow link (600 ms): %.1f frames/s, all reduced %s; good link: %.1f frames/s per boat second; "
               "latency shown: %s" % (rate_slow, slow.lite == slow.n_tel, rate_fast, rtts))
        slow.close()
        fast.close()
        slow_net.stop()

        # links: 4G, RF, Starlink - no stale command ever executed
        for prof in ("4g", "rf", "starlink"):
            pp = free_port()
            net = NetSim(pp, sp, PROFILES[prof], seed=7).start()
            c = Client(pp, "alice", PW["alice"], name="alice via " + prof)
            c.pump(2.0)
            c.cmd(proto.T_HELM, proto.p_helm(0), timeout=8)
            sent = 0
            seqs = []
            t0 = time.time()
            while time.time() - t0 < 25.0:
                seqs.append(c.l.send(proto.T_ALARM_ACK, b""))
                sent += 1
                c.pump(0.2)
            c.pump(8.0)
            got = {a["seq"]: a["result"] for a in c.acks}
            c.acks.clear()
            executed = sum(1 for s in seqs if got.get(s) == 0)
            stale_n = sum(1 for s in seqs if got.get(s) == 103)
            ages = [e.get("age_ms", 0) for e in gw.gw.audit.recent
                    if e["kind"] == "cmd" and e.get("result") == 0 and e.get("user") == "alice" and "age_ms" in e]
            max_age = max(ages[-executed:]) if executed else 0
            c.cmd(proto.T_HELM, proto.p_helm(1), timeout=8)
            c.close()
            net.stop()
            report("Links %s: no command older than 2 s executed; telemetry keeps coming" % prof.upper(),
                   executed > 0 and max_age <= 2000 and (prof != "starlink" or stale_n > 0) and c.n_tel > 20,
                   "%d commands at 5 Hz: %d executed (oldest %d ms), %d refused as stale, %d unanswered; "
                   "%d telemetry frames (%d reduced)" % (sent, executed, max_age, stale_n,
                                                        sent - len([s for s in seqs if s in got]), c.n_tel, c.lite))

        # B7 audit log
        files = glob.glob(os.path.join(root, "audit", "*.jsonl"))
        kinds = {}
        for fpath in files:
            for line in open(fpath, encoding="utf-8"):
                k = _json.loads(line)["kind"]
                kinds[k] = kinds.get(k, 0) + 1
        need = ("connect", "disconnect", "login_failed", "cmd", "helm")
        report("B7 audit log: connections, failed logins, HELM changes, every command with its result",
               files and all(kinds.get(k) for k in need),
               "%s: %s" % (", ".join(os.path.basename(f) for f in files), kinds))
        for c in (vic, alice, bob, bob_local):
            c.close()
        link.stop()
        gw.stop()
        sim.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------- section C --

class ChannelNet:
    """One network-simulator proxy per internet channel; only the channel
    the gateway made active carries traffic (like the real boat)."""

    def __init__(self, gw, station_port):
        self.gw = gw
        self.drv = gw.net.driver
        self.proxies = {}
        prof = {"4G": "4g", "4G2": "4g", "RF": "rf", "STARLINK": "starlink"}
        for i, n in enumerate(gw.net.order):
            p = PROFILES[prof.get(n, "4g")].copy()
            p.pop("outage_every_s", None)                      # outages come from the test here
            self.proxies[n] = (free_port(), NetSim(0, station_port, p, seed=i))
        for n, (port, ns) in self.proxies.items():
            ns.listen_port = port
            ns.start()
        self.run = True
        threading.Thread(target=self._follow, daemon=True).start()

    def _follow(self):
        while self.run:
            for n, (_, ns) in self.proxies.items():
                ns.forced_down = (self.drv.active != n) or not self.drv.state[n]["internet"]
            time.sleep(0.05)

    def port(self, n=None):
        return self.proxies[n or self.drv.active][0]

    def stop(self):
        self.run = False
        for _, ns in self.proxies.values():
            ns.stop()


def wait_for(cond, t):
    end = time.time() + t
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def section_c():
    print("\n=== Section C: internet channels (4G, RF, STARLINK = emergency only) ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_c_")
    try:
        sim = Sim(2.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root, channels={"driver": "sim", "check_s": 0.2, "stable_s": 3.0, "fail_s": 3.0,
                                            "trial_s": 6.0, "starlink_status": False})
        gw = GwRunner(cfg)
        g = gw.gw
        net = g.net
        drv = net.driver
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        st = drv.state
        st["4G2"]["internet"] = False                          # only one SIM card in the router
        time.sleep(1.0)
        cn = ChannelNet(g, sp)

        def used(n):
            return any(a == n for _, a in drv.switches)

        # C1 AUTO: 4G -> RF when 4G fails; never STARLINK while a normal channel works
        a0 = net.active
        st["4G"]["internet"] = False
        to_rf = wait_for(lambda: net.active == "RF", 3)
        report("C1 AUTO: 4G down -> RF; STARLINK never used while 4G or RF works",
               a0 == "4G" and to_rf and not used("STARLINK"),
               "start %s; 4G down -> %s; STARLINK used: %s" % (a0, net.active, used("STARLINK")))

        # C2 every normal channel down -> STARLINK + banner flag + data saver + no sync
        st["RF"]["internet"] = False
        on_sl = wait_for(lambda: net.active == "STARLINK", 3)
        tel = w.until(lambda d: d.get("gw_flags", 0) & proto.GWF_STARLINK, 5)
        viewer = Client(cn.port(), "vic", PW["vic"], name="vic over starlink")
        viewer.pump(3.0)
        lite_ok = viewer.l.tmode == "lite"
        report("C2 all normal channels down -> STARLINK, emergency banner flag, data saver on, sync stopped",
               on_sl and tel is not None and tel["gw_flags"] & proto.GWF_SAVER and lite_ok and not net.sync_allowed(),
               "active %s; boat telemetry flags STARLINK %s SAVER %s; station telemetry %s; sync allowed %s" % (
                   net.active, bool(tel and tel["gw_flags"] & proto.GWF_STARLINK),
                   bool(tel and tel["gw_flags"] & proto.GWF_SAVER), viewer.l.tmode, net.sync_allowed()))
        viewer.close()

        # C3 4G back -> stays on STARLINK until 4G is stable (3 s here, 60 s default), then back
        t0 = time.time()
        st["4G"]["internet"] = True
        time.sleep(1.0)
        still = net.active == "STARLINK"
        back = wait_for(lambda: net.active == "4G", 6)
        report("C3 4G back -> automatic return once it has been stable",
               still and back and 2.5 <= time.time() - t0 <= 5.0,
               "1 s after 4G came back: still %s; back on %s after %.1f s (stable time 3 s)" % (
                   "STARLINK" if still else "?", net.active, time.time() - t0))
        st["RF"]["internet"] = True

        # C4 VIEWER can't change; a HELM user without HELM (remote) can't; STRICT only on board
        vic = Client(cn.port(), "vic", PW["vic"])
        alice = Client(cn.port(), "alice", PW["alice"])
        boat = Client(sp, "bob", PW["bob"], local=True, name="bob on board")

        def chan(c, mode, ch=None, confirm=False, keep=False):
            c.acks.clear()
            c.l.send_msg({"t": "channel_keep"} if keep else {"t": "channel", "mode": mode, "channel": ch,
                                                             "confirm": confirm})
            end = time.time() + 3
            while time.time() < end:
                try:
                    kind, d = c.l.events.get(timeout=0.1)
                except Exception:
                    continue
                if kind == "channel_ack":
                    return d
            return None

        r_v = chan(vic, "MANUAL", "RF")
        r_a = chan(alice, "MANUAL", "RF")
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        r_strict = chan(alice, "MANUAL_STRICT", "RF")
        r_sl = chan(alice, "MANUAL", "STARLINK")
        report("C4 VIEWER and non-HELM remote can't change the channel; STRICT only on board; STARLINK needs confirm",
               r_v and not r_v["ok"] and r_a and not r_a["ok"] and r_strict and not r_strict["ok"] and
               r_sl and not r_sl["ok"] and r_sl["needs_confirm"],
               "viewer: %s | remote without HELM: %s | STRICT from remote: %s | STARLINK without confirm: %s" % (
                   r_v and r_v["text"], r_a and r_a["text"], r_strict and r_strict["text"], r_sl and r_sl["text"]))

        # C5 remote manual switch to a WORKING channel: reconnect over it, Keep -> kept
        r_rf = chan(alice, "MANUAL", "RF")
        switched = wait_for(lambda: net.active == "RF", 2)
        alice.close()                                           # the old link (4G) is gone for her
        alice = Client(cn.port("RF"), "alice", PW["alice"], name="alice over RF")
        r_keep = chan(alice, None, keep=True)
        time.sleep(7.0)                                         # longer than the 6 s trial
        report("C5 remote switch to a working channel: reconnect over it, KEEP -> kept",
               r_rf and r_rf["ok"] and switched and r_keep and r_keep["ok"] and net.active == "RF" and net.mode == "MANUAL",
               "ask MANUAL RF: %s; switched %s; reconnected over RF and kept: %s; after the trial time: %s %s" % (
                   r_rf and r_rf["ok"], switched, r_keep and r_keep["text"], net.mode, net.active))

        # C6 remote manual switch to a DEAD channel: can't reconnect, no Keep -> revert, logged
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        st["4G2"]["internet"] = False
        st["4G2"]["up"] = True
        r_dead = chan(alice, "MANUAL", "4G2")
        t0 = time.time()
        reverted = wait_for(lambda: net.active == "RF" and net.mode == "MANUAL" and net.manual == "RF" and
                            time.time() - t0 > 2, 12)
        dt = time.time() - t0
        rev = [e for e in g.audit.recent if e["kind"] == "channel_trial_reverted"]
        report("C6 remote switch to a dead channel: no KEEP possible -> back to the last working channel",
               r_dead and r_dead["ok"] and reverted and 5.0 <= dt <= 9.0 and rev,
               "ask MANUAL 4G SIM 2 (dead): %s; back to %s %s after %.1f s (trial 6 s); audit: %s" % (
                   r_dead and r_dead["ok"], net.mode, net.active, dt, rev[-1] if rev else None))
        alice.close()

        # C7 MANUAL STRICT from on board: allowed with a warning; never switches
        r_st = chan(boat, "MANUAL_STRICT", "RF", confirm=True)
        chan(boat, None, keep=True)
        st["RF"]["internet"] = False
        time.sleep(5.0)
        strict_stays = net.active == "RF"
        st["RF"]["internet"] = True
        report("C7 MANUAL STRICT (on board only, with warning) never switches, even when the channel fails",
               r_st and r_st["ok"] and "WARNING" in r_st["text"] and strict_stays,
               "%s | RF failed 5 s: still on %s" % (r_st and r_st["text"][:80], net.active))

        # C8 MANUAL channel fails -> fallback + CHANNEL FALLBACK alarm on the boat; back when stable
        r_m = chan(boat, "MANUAL", "RF")
        chan(boat, None, keep=True)
        st["RF"]["internet"] = False
        fb = wait_for(lambda: net.fallback and net.active == "4G", 6)
        al = w.until(lambda d: d["alarm_bits"] & 0x80000, 5)
        st["RF"]["internet"] = True
        back = wait_for(lambda: net.active == "RF" and not net.fallback, 8)
        report("C8 MANUAL channel fails -> next working channel + alarm CHANNEL FALLBACK; back when it is stable",
               r_m and r_m["ok"] and fb and al is not None and al["mode"] == 0 and back,
               "RF failed -> %s (fallback %s); boat alarm %s, mode %s; RF back -> %s" % (
                   "4G" if fb else net.active, fb, al is not None, al and al["mode_name"], net.active))

        # C9 data counters, STARLINK session / month and the monthly warning
        chan(boat, "AUTO")
        chan(boat, None, keep=True)
        net.channels["STARLINK"]["monthly_warn_gb"] = 0.001             # 1 MB for the test
        for n in ("4G", "4G2", "RF"):
            st[n]["internet"] = False
        wait_for(lambda: net.active == "STARLINK", 3)
        for _ in range(10):
            drv.bytes["STARLINK"] += 300000
            drv.bytes["4G"] += 100000
            time.sleep(0.25)
        info = net.info()
        sl = info["starlink"]
        c4 = [c for c in info["channels"] if c["name"] == "STARLINK"][0]
        report("C9 data used today / this month per channel; STARLINK session counter and monthly warning",
               c4["today_mb"] >= 2.0 and sl["session_mb"] >= 2.0 and sl["warning"],
               "STARLINK today %.1f MB, month %.1f MB, this session %.1f MB, warning (limit %s GB) %s" % (
                   c4["today_mb"], c4["month_mb"], sl["session_mb"], sl["warn_gb"], sl["warning"]))
        for n in ("4G", "RF"):
            st[n]["internet"] = True
        # C10 the RUT901 driver: mwan3 metrics it sets, router answers it reads
        from channels import RutosDriver, DEFAULT_CHANNELS
        sent = []
        mwan_out = (" interface mob1s1a1 is online and tracking is active\n"
                    " interface mob1s2a1 is offline and tracking is down\n"
                    " interface wan is offline and tracking is down\n---\n-71\nLTE\nVodafone GR\n")
        rd = RutosDriver({}, {c["name"]: c for c in DEFAULT_CHANNELS})

        def fake_ssh(cmd, timeout=8):
            sent.append(cmd)
            if cmd.startswith("mwan3"):
                return mwan_out
            if cmd.startswith("ping"):
                return ("3 packets transmitted, 3 packets received, 0% packet loss\n"
                        "round-trip min/avg/max = 40.1/52.3/70.2 ms")
            return ""
        rd._ssh = fake_ssh
        rd.apply("STARLINK", ["4G", "4G2", "RF", "STARLINK"])
        s4 = rd.status()
        set_cmd = sent[0]
        report("C10 RUT901 driver: sets mwan3 metrics (wanted = 1), reads online state, latency, 4G signal",
               "mwan3.member_wan.metric=1" in set_cmd and "mwan3.member_mob1s1a1.metric=10" in set_cmd and
               "mwan3 reload" in set_cmd and s4["4G"]["internet"] and s4["4G"]["latency_ms"] == 52.3 and
               s4["4G"]["signal"]["rssi_dbm"] == "-71" and not s4["STARLINK"]["internet"] and "standby" in s4["4G2"]["note"],
               "commands: %s | 4G: online %s, %.1f ms, %s dBm %s %s | SIM 2: %s" % (
                   set_cmd[:110] + "...", s4["4G"]["internet"], s4["4G"]["latency_ms"], s4["4G"]["signal"]["rssi_dbm"],
                   s4["4G"]["signal"]["type"], s4["4G"]["signal"]["operator"], s4["4G2"].get("note")))
        for c in (vic, boat):
            c.close()
        cn.stop()
        link.stop()
        gw.stop()
        sim.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    section_c11()


def section_c11():
    """T1: STARLINK never chosen automatically while a normal channel works -
    the two cases the gateway storm found (trial revert, MANUAL fallback)."""
    from channels import ChannelManager, SimDriver, DEFAULT_CHANNELS
    clk = [1.8e9]

    def mk():
        drv = SimDriver([c["name"] for c in DEFAULT_CHANNELS])
        net = ChannelManager({"stable_s": 60.0, "fail_s": 30.0, "trial_s": 60.0, "starlink_status": False},
                             driver=drv, clock=lambda: clk[0])
        return drv, net

    def run(net, s):
        for _ in range(int(s)):
            clk[0] += 1.0
            net.tick()

    def link(drv, n, ok):
        drv.state[n]["up"] = drv.state[n]["internet"] = ok

    # a) AUTO on STARLINK (every normal channel down); RF comes back; a user tries a
    #    dead channel and doesn't keep it -> the revert must not go back to STARLINK
    drv, net = mk()
    for n in ("4G", "4G2", "RF"):
        link(drv, n, False)
    run(net, 40)
    on_star = net.active
    net.request("MANUAL", "4G2", user="u1", may=True)
    run(net, 40)
    link(drv, "RF", True)                           # back 20 s before the trial runs out (not stable yet)
    run(net, 25)
    back_a = net.active
    # b) MANUAL RF fails -> fallback 4G; RF works again (not stable yet); then 4G dies
    #    -> RF, not STARLINK
    drv, net = mk()
    net.request("MANUAL", "RF", user="u1", may=True)
    net.keep("u1")
    run(net, 5)
    link(drv, "RF", False)
    run(net, 35)
    fb = net.active
    link(drv, "RF", True)
    run(net, 5)
    link(drv, "4G", False)
    link(drv, "4G2", False)
    run(net, 3)
    back_b = net.active
    report("C11 STARLINK never chosen automatically while a normal channel works (trial revert, MANUAL fallback)",
           on_star == "STARLINK" and back_a == "RF" and fb == "4G" and back_b == "RF",
           "a) all normal down: %s; RF back, trial of a dead channel not kept -> %s (want RF); "
           "b) MANUAL RF fails -> %s; RF works again, then the fallback dies -> %s (want RF)" % (
               on_star, back_a, fb, back_b))


# ------------------------------------------------------------------- section D --

from shorejoy import RemoteSender  # noqa: E402

LATENCY = {}                        # for the final report


def section_d():
    print("\n=== Section D: remote joystick driving (shore PC) ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_d_")
    try:
        sim = Sim(1.0)                                  # real time: latencies are real
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root)
        gw = GwRunner(cfg)
        sp = cfg["stations"]["port"]
        link, w = sim.station()                          # the laptop on board (USB): CONFIRM, observing
        pp = free_port()
        net = NetSim(pp, sp, PROFILES["4g"], seed=3).start()
        alice = Client(pp, "alice", PW["alice"], name="alice shore PC (4G)")
        alice.pump(2.0)
        stick = {"x": 0.0, "lever": 0.5}
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        r_helm = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        r_ask = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(0)))
        time.sleep(1.5)
        w.pump(0.1)
        not_yet = w.telem and w.telem["mode"] == proto.MODE_STANDBY and w.telem["confirm_kind"] == 5
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))   # the person on board
        time.sleep(0.2)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        on = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 3)
        report("D1 remote driving from the shore PC: HELM + request + joystick CONFIRM on board",
               r_helm == 0 and r_ask == 0 and not_yet and on is not None,
               "HELM %s, request %s; waiting for CONFIRM (banner REMOTE DRIVING) %s; after CONFIRM on board: %s" % (
                   r_helm, r_ask, not_yet, on and on["mode_name"]))

        # control latency: stick moves on shore -> seen in the boat's telemetry
        one_way, round_trip = [], []
        for k in range(12):
            target = 0.8 if k % 2 == 0 else -0.8
            t0 = time.time()
            stick["x"] = target
            got = w.until(lambda d: abs(d.get("rjoy_x1000", 0) / 1000.0 - target) < 0.05, 3)
            if got:
                one_way.append((time.time() - t0) * 1000)
            got2 = None
            end = time.time() + 3
            while time.time() < end:
                alice.pump(0.01)
                if alice.telem and abs(alice.telem.get("rjoy_x1000", 0) / 1000.0 - target) < 0.05:
                    got2 = True
                    break
            if got2:
                round_trip.append((time.time() - t0) * 1000)
            time.sleep(0.3)
        stick["x"] = 0.0
        LATENCY["control_one_way"] = one_way
        LATENCY["control_round_trip"] = round_trip
        ow = sorted(one_way)
        report("D2 control latency shore joystick -> boat (4G simulated: ~80 ms round trip, 1 % loss)",
               len(one_way) >= 10 and ow[len(ow) // 2] < 250 and max(one_way) < 400,
               "stick move -> boat telemetry: median %.0f ms, max %.0f ms; -> back on the shore screen: median %.0f ms "
               "(includes the 30 Hz send period, the 20 Hz boat loop and the 10 Hz telemetry)" % (
                   ow[len(ow) // 2] if ow else -1, max(one_way) if one_way else -1,
                   sorted(round_trip)[len(round_trip) // 2] if round_trip else -1))

        # the cap, through the whole chain
        for lv in (0.6, 0.8, 1.0):
            stick["lever"] = lv
            time.sleep(0.5)
        top = 0.0
        t0 = time.time()
        while time.time() - t0 < 8:
            w.pump(0.1)
            if w.telem:
                top = max(top, w.telem["throttle"])
        report("D3 remote cap through the gateway: full lever -> at most 20 %",
               15.0 <= top <= 20.01, "full remote lever: throttle up to %.1f %% (cap 20 %%, power limit %.0f %%)" % (
                   top, w.telem["power_fwd"] if w.telem else -1))

        # a station without HELM sends joystick packets: dropped by the gateway
        bob = Client(sp, "bob", PW["bob"], name="bob")
        bsnd = RemoteSender(bob.l, lambda: {"x": -1.0, "lever": 0.0})
        bsnd.start()
        time.sleep(1.5)
        bsnd.stop()
        w.pump(0.2)
        bob_st = [s for s in gw.gw.stations.stations.values() if s.name == "bob"]
        dropped = getattr(bob_st[0], "rjoy_dropped", 0) if bob_st else 0
        report("D4 joystick packets from a station without HELM are ignored",
               dropped > 20 and w.telem["mode"] == proto.MODE_REMOTE and w.telem.get("rjoy_x1000", 0) > -500,
               "bob sent %d packets: %d dropped by the gateway; boat still driven by alice (x %.2f)" % (
                   bsnd.sent, dropped, w.telem.get("rjoy_x1000", 0) / 1000.0))
        bob.close()

        # D6 a long drive over 4G with 1 % packet loss: no spurious LINK LOST
        n_lost = 0
        prev = None
        t0 = time.time()
        k = 0
        while time.time() - t0 < 60.0:
            stick["x"] = 0.5 * math.sin(k / 15.0)
            k += 1
            w.pump(0.033)
            st_now = w.telem.get("remote_state") if w.telem else None
            if st_now == 2 and prev != 2:
                n_lost += 1
            prev = st_now
        stick["x"] = 0.0
        report("D6 60 s of remote driving over 4G with 1 % packet loss: no spurious LINK LOST (UDP joystick)",
               n_lost == 0 and w.telem["mode"] == proto.MODE_REMOTE,
               "LINK LOST events %d; UDP packets passed %d, lost on the way %d; still %s" % (
                   n_lost, net.udp_passed, net.udp_dropped, w.telem["mode_name"]))

        # D7 forged UDP packets (wrong key) and replayed old packets are ignored
        import socket as _so
        import hmac as _hm
        import hashlib as _hl
        st_obj = [x for x in gw.gw.stations.stations.values() if x.user == "alice"][0]
        bad0 = getattr(st_obj, "udp_bad", 0)
        u = _so.socket(_so.AF_INET, _so.SOCK_DGRAM)
        for i in range(20):                                 # forged: right session id, wrong key
            body = _json.dumps({"t": "rjoy", "sid": alice.l.welcome["sid"], "seq": snd.seq + 1000 + i,
                                "ts": alice.l.now(), "x": -1.0, "lever": 1.0}).encode()
            u.sendto(body + b"|" + _hm.new(b"wrong key 123456", body, _hl.sha256).hexdigest()[:32].encode(),
                     ("127.0.0.1", sp))
        old = dict(snd.last or {})                          # a recorded genuine packet, sent again later
        time.sleep(0.5)
        for i in range(10):
            body = _json.dumps(dict(old, sid=alice.l.welcome["sid"], x=-1.0), separators=(",", ":")).encode()
            u.sendto(body + b"|" + _hm.new(bytes.fromhex(alice.l.welcome["udp_key"]), body, _hl.sha256)
                     .hexdigest()[:32].encode(), ("127.0.0.1", sp))
        time.sleep(0.5)
        w.pump(0.3)
        report("D7 forged (wrong key) and replayed UDP joystick packets are ignored",
               getattr(st_obj, "udp_bad", 0) - bad0 >= 20 and w.telem.get("rjoy_x1000", 0) > -500,
               "forged packets refused %d; replayed old packets with x = -1: boat x stays %.2f" % (
                   getattr(st_obj, "udp_bad", 0) - bad0, w.telem.get("rjoy_x1000", 0) / 1000.0))

        # the 4G link breaks for 1 s: LINK LOST at once, STANDBY after 5 s, never resumes
        t0 = time.time()
        net.forced_down = True
        lost = w.until(lambda d: d.get("remote_state") == 2, 2)
        t_lost = (time.time() - t0) * 1000
        time.sleep(max(0.0, 1.0 - (time.time() - t0)))
        net.forced_down = False
        t_back = time.time()
        sb = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 7)
        t_sb = time.time() - t0
        resumed = False
        end = time.time() + 2
        while time.time() < end:
            w.pump(0.1)
            if w.telem and w.telem["mode"] == proto.MODE_REMOTE:
                resumed = True
        report("D5 4G link breaks during remote driving: LINK LOST (idle + neutral + gyro hold), STANDBY after 5 s, "
               "no automatic resume",
               lost is not None and t_lost < 800 and lost["alarm_bits"] & 0x100000 and sb is not None and
               4.5 <= t_sb <= 7.0 and not resumed,
               "LINK LOST after %.0f ms (%s), throttle %.1f %%; link back after 1 s; STANDBY after %.1f s; "
               "driving again by itself: %s" % (
                   t_lost, lost and proto.REMOTE_LOST_WHY[min(lost.get("remote_lost_why", 0), 3)],
                   lost["throttle"] if lost else -1, t_sb, resumed))
        _ = t_back
        snd.stop()
        alice.close()
        net.stop()
        link.stop()
        gw.stop()
        sim.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------- section E --

from simcam import SimCamera, decode_ts, latency_ms  # noqa: E402
from video import VideoStream, FrontSelector  # noqa: E402
from simcam import ffmpeg_bin  # noqa: E402
from recorder import seg_time  # noqa: E402
import tlg  # noqa: E402

import toolpaths  # noqa: E402
MTX_BIN = toolpaths.mediamtx() or "mediamtx (missing: python tools/get_mediamtx.py)"
CAM_NAMES = ["FRONT-1", "FRONT-2", "REAR-LEFT", "REAR-RIGHT"]
CAM_ROLES = ["front_main", "front_backup", "rear_left", "rear_right"]


class CamRig:
    """Simulator + gateway with MediaMTX + 4 simulated cameras."""

    def __init__(self, tmp, speed=1.0, **extra):
        extra.setdefault("recording", {"enabled": False})       # only section F records
        self.sim = Sim(speed)
        self.rtsp, api, web, wu = free_port(), free_port(), free_port(), free_port()
        cams = [{"name": n, "role": r, "url_main": "rtsp://127.0.0.1:%d/simcam_%d" % (self.rtsp, i),
                 "url_sub": "rtsp://127.0.0.1:%d/simcam_%d" % (self.rtsp, i)}
                for i, (n, r) in enumerate(zip(CAM_NAMES, CAM_ROLES))]
        from common import deep_merge
        ccfg = {"enabled": True, "list": cams, "mediamtx_bin": MTX_BIN, "mediamtx_conf": os.path.join(tmp, "mtx.yml"),
                "rtsp_port": self.rtsp, "api_port": api, "webrtc_port": web, "webrtc_udp_port": wu,
                "extra_paths": ["simcam_%d" % i for i in range(4)], "mediamtx_log": os.path.join(tmp, "mtx.log"),
                "log_level": "info"}
        root = os.path.join(tmp, "ssd")
        os.makedirs(root, exist_ok=True)
        self.cfg = gw_cfg_b(self.sim, root, **deep_merge({"cameras": ccfg, "channels": {"driver": "sim", "check_s": 0.2,
                                                                                         "stable_s": 1.0,
                                                                                         "starlink_status": False}},
                                                         extra))
        self.gw = GwRunner(self.cfg)
        time.sleep(1.0)
        self.cams = [SimCamera("rtsp://127.0.0.1:%d/simcam_%d" % (self.rtsp, i)).start() for i in range(4)]
        self.g = self.gw.gw
        wait_for(lambda: all(c.state == "OK" for c in self.g.cams.cams.values()), 20)

    def ask(self, client, **msg):
        """Camera request -> the cams_ack answer (older, pushed answers are dropped first)."""
        while True:
            try:
                client.l.events.get_nowait()
            except Exception:
                break
        client.l.send_msg(dict({"t": "cams"}, **msg))
        return self.wait_ack(client)

    @staticmethod
    def wait_ack(client, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            try:
                k, d = client.l.events.get(timeout=0.1)
            except Exception:
                continue
            if k == "cams_ack":
                return d
        return None

    def url(self, ack, name, port=None, path=None):
        return "rtsp://%s:%s@127.0.0.1:%d/%s?%s" % (ack["user"], ack["password"], port or self.rtsp,
                                                    path or ack["streams"][name], ack["query"])

    def stop(self):
        for c in self.cams:
            c.stop()
        self.gw.stop()
        self.sim.stop()


def section_e():
    print("\n=== Section E: cameras (4 x Tapo C310, simulated) ===", flush=True)
    if not os.path.exists(MTX_BIN):
        report("E0 MediaMTX available for the camera tests", False, "not found: %s (set MEDIAMTX_BIN)" % MTX_BIN)
        return
    tmp = tempfile.mkdtemp(prefix="gwtest_e_")
    rig = None
    try:
        rig = CamRig(tmp)
        g = rig.g
        states = {c.name: c.state for c in g.cams.cams.values()}
        alice = Client(rig.cfg["stations"]["port"], "alice", PW["alice"])

        # E1 presets -> the streams a station gets; anything else is refused by the relay
        got = {}
        for pr in ("Full", "Front only", "Mirrors only", "All off"):
            a = rig.ask(alice, preset=pr)
            got[pr] = a["streams"] if a else None
        a = rig.ask(alice, preset="Front only")
        ok_vs = VideoStream(rig.url(a, "FRONT-1"), 320, 240).start()
        bad_vs = VideoStream(rig.url(a, "REAR-LEFT", path="cam_rearleft_hi"), 320, 240).start()
        time.sleep(4)
        report("E1 camera presets: each station gets only its streams; other streams are refused by the relay",
               all(s == "OK" for s in states.values()) and got["Full"] == {
                   "FRONT-1": "cam_front1_hi", "FRONT-2": "cam_front2_hi", "REAR-LEFT": "cam_rearleft_lo",
                   "REAR-RIGHT": "cam_rearright_lo"} and set(got["Front only"]) == {"FRONT-1", "FRONT-2"} and
               set(got["Mirrors only"]) == {"REAR-LEFT", "REAR-RIGHT"} and got["All off"] == {} and
               ok_vs.frames >= 10 and bad_vs.frames == 0,
               "cameras %s; Full %s; Front only %s; Mirrors only %s; All off %s; allowed stream %d frames, "
               "not allowed: %d frames (%s)" % (states, got["Full"], got["Front only"], got["Mirrors only"],
                                                 got["All off"], ok_vs.frames, bad_vs.frames, bad_vs.error[:40]))
        bad_vs.stop()

        # E2 OFF streams use no uplink; per-stream and total bitrate shown
        time.sleep(2)
        st = g.cams.status()
        out = {c["name"]: c["kbps_out"] for c in st["cameras"]}
        report("E2 streams that are OFF use no uplink; per-stream and total video bitrate shown",
               out["FRONT-1"]["LOW"] > 10 and out["REAR-LEFT"] == {"HIGH": 0, "LOW": 0} and
               out["REAR-RIGHT"] == {"HIGH": 0, "LOW": 0} and st["total_kbps"] == out["FRONT-1"]["LOW"],
               "kbit/s to the stations per camera %s, total %d; station %s" % (out, st["total_kbps"],
                                                                                st["station_kbps"]))
        ok_vs.stop()

        # E3 FRONT-1 frozen -> FRONT-2 within 1 s, VIDEO FROZEN, back when FRONT-1 is good; camera failure: no mode change
        a = rig.ask(alice, preset="Front only")
        f1 = VideoStream(rig.url(a, "FRONT-1"), 320, 240).start()
        f2 = VideoStream(rig.url(a, "FRONT-2"), 320, 240).start()
        sel = FrontSelector()
        link, w = rig.sim.station()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 3)
        wait_for(lambda: f1.frames > 10 and f2.frames > 10, 8)
        sel.update(f1, f2)
        t0 = time.time()
        rig.cams[0].freeze(True)
        switched = frozen_seen = None
        while time.time() - t0 < 4:
            if sel.update(f1, f2) == "FRONT-2" and switched is None:
                switched = time.time() - t0
            if f1.frozen() and frozen_seen is None:
                frozen_seen = time.time() - t0
            time.sleep(0.02)
        gw_state = g.cams.cams["FRONT-1"].state
        rig.cams[0].freeze(False)
        t1 = time.time()
        back = None
        while time.time() - t1 < 8:
            if sel.update(f1, f2) == "FRONT-1":
                back = time.time() - t1
                break
            time.sleep(0.05)
        # unplugged
        rig.cams[0].unplug()
        t2 = time.time()
        sw2 = None
        while time.time() - t2 < 4:
            if sel.update(f1, f2) == "FRONT-2":
                sw2 = time.time() - t2
                break
            time.sleep(0.02)
        mode_ok = w.until(lambda d: True, 1) and w.telem["mode"] == proto.MODE_HDG_LOCK
        report("E3 FRONT-1 frozen / unplugged -> FRONT-2 shown within 1 s, VIDEO FROZEN, back to FRONT-1; "
               "camera failure never changes the mode",
               switched is not None and switched <= 1.2 and frozen_seen is not None and frozen_seen <= 1.2 and
               gw_state == "FROZEN" and back is not None and sw2 is not None and sw2 <= 1.2 and mode_ok,
               "frozen: VIDEO FROZEN after %.2f s, FRONT-2 after %.2f s (gateway: %s); unfrozen: FRONT-1 again after "
               "%.1f s; unplugged: FRONT-2 after %.2f s; boat mode still %s" % (
                   frozen_seen or -1, switched or -1, gw_state, back or -1, sw2 or -1, w.telem["mode_name"]))
        f1.stop()
        f2.stop()
        rig.cams[0] = SimCamera("rtsp://127.0.0.1:%d/simcam_0" % rig.rtsp).start()

        # E4 data saver / STARLINK: front only LOW, no override on STARLINK; HIGH readers closed
        a = rig.ask(alice, preset="Full")
        hi = VideoStream(rig.url(a, "FRONT-1"), 320, 240).start()
        wait_for(lambda: hi.frames > 10, 8)
        drv = g.net.driver
        for n in ("4G", "4G2", "RF"):
            drv.state[n]["internet"] = False
        push = rig.wait_ack(alice, 5)                       # the gateway sends the new list by itself
        n0 = hi.frames
        time.sleep(3)
        hi_stopped = hi.frames - n0 < 5
        ov = rig.ask(alice, preset="Full", override=True)
        for n in ("4G", "RF"):
            drv.state[n]["internet"] = True
        wait_for(lambda: not g.net.on_emergency, 8)
        g.net.saver_manual = True                           # data saver by hand (not STARLINK)
        ov2 = rig.ask(alice, preset="Full", override=True)
        g.net.saver_manual = False
        report("E4 STARLINK / data saver: front camera LOW only (override refused on STARLINK); HIGH streams closed",
               push is not None and push["streams"] == {"FRONT-1": "cam_front1_lo", "FRONT-2": "cam_front2_lo"} and
               "DATA SAVER" in push["notice"] and hi_stopped and ov["streams"] == push["streams"] and
               ov2["streams"].get("FRONT-1") == "cam_front1_hi",
               "on STARLINK: %s (%s); FRONT-1 HIGH reader closed %s; override on STARLINK: %s; "
               "override with data saver on 4G: %s" % (push and push["streams"], push and push["notice"], hi_stopped,
                                                       ov["streams"], ov2["streams"]))
        hi.stop()

        # E5 camera internet: blocked by default; only from a station on board; closes by itself
        r_remote = rig.wait_ack(alice) if alice.l.send_msg({"t": "cam_internet", "on": True}) is None else None
        boat = Client(rig.cfg["stations"]["port"], "bob", PW["bob"], local=True, name="bob on board")
        blocked = not g.cams.internet_allowed()
        boat.l.send_msg({"t": "cam_internet", "on": True, "minutes": 0.05})
        r_boat = rig.wait_ack(boat)
        on = g.cams.internet_allowed()
        time.sleep(3.5)
        off = not g.cams.internet_allowed()
        report("E5 cameras have no internet; a station on board can open it briefly (firmware update), it closes "
               "by itself", blocked and r_remote and not r_remote["ok"] and r_boat and r_boat["ok"] and on and off,
               "default blocked %s; remote asks: %s; on board asks 3 s: %s -> open %s, after 3.5 s closed %s" % (
                   blocked, r_remote and r_remote["text"], r_boat and r_boat["text"], on, off))
        boat.close()

        # E6 video latency: LAN and over the 4G link simulation
        a = rig.ask(alice, preset="Front only")
        res = {}
        for label, prof in (("LAN", None), ("4G", "4g")):
            port = None
            net = None
            if prof:
                port = free_port()
                net = NetSim(port, rig.rtsp, PROFILES[prof], seed=5).start()
            lats = []
            vs = VideoStream(rig.url(a, "FRONT-1", port=port), 320, 240)
            vs.on_frame = lambda f, lats=lats: lats.append(latency_ms(decode_ts(f, 320, 240)))
            vs.start()
            time.sleep(10)
            vs.stop()
            if net:
                net.stop()
            good = sorted(x for x in lats if x is not None)
            res[label] = (good[len(good) // 2], good[int(len(good) * 0.95)], len(good)) if good else (None, None, 0)
            if not good:
                print("E6 %s: no frames, decoder error: %s; url %s" % (label, vs.error, vs.url.split("@")[-1]))
                print(open(os.path.join(tmp, "mtx.log")).read()[-1500:])
        LATENCY["video"] = res
        report("E6 video latency (simulated camera -> gateway relay -> station decoder)",
               res["LAN"][0] is not None and res["4G"][0] is not None and res["LAN"][0] < 400 and res["4G"][0] < 800,
               "LAN: median %s ms, 95 %% %s ms (%d frames); 4G simulated: median %s ms, 95 %% %s ms (%d frames). "
               "Add the Tapo's own encoder delay (not simulated)" % (res["LAN"] + res["4G"]))
        alice.close()
        link.stop()
    finally:
        if rig:
            rig.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------- section F --

def frame_code(path, w=320, h=240):
    """Time code of the first picture in a recorded file (simulated camera)."""
    r = subprocess.run([ffmpeg_bin(), "-hide_banner", "-loglevel", "error", "-i", path, "-frames:v", "1",
                        "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True, timeout=20)
    return decode_ts(r.stdout, w, h) if len(r.stdout) >= w * h * 3 else None


def code_offset_ms(code, t):
    """Picture time code - expected code for Unix time t, in ms (wrap-safe)."""
    m = 1 << 26
    d = (code - (int(t * 1000) & (m - 1))) % m
    return d - m if d > m // 2 else d


def save_ack(client, timeout=4.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            k, d = client.l.events.get(timeout=0.1)
        except Exception:
            continue
        if k == "save_ack":
            return d
    return None


def send_wait(client, **msg):
    while True:
        try:
            client.l.events.get_nowait()
        except Exception:
            break
    client.l.send_msg(msg)
    return save_ack(client)


def event_json(rec, eid):
    try:
        with open(os.path.join(rec.events_dir(), eid, "event.json"), encoding="utf-8") as f:
            return _json.load(f)
    except (OSError, ValueError):
        return None


def section_f():
    print("\n=== Section F: video recording (ring buffer, SAVE, storage) ===", flush=True)
    if not os.path.exists(MTX_BIN):
        report("F0 MediaMTX available for the recording tests", False, "not found: %s" % MTX_BIN)
        return
    tmp = tempfile.mkdtemp(prefix="gwtest_f_")
    rig = None
    SEG, POST = 2, 4
    try:
        rig = CamRig(tmp, recording={"enabled": True, "segment_s": SEG, "ring_choices_min": [0.1, 0.2, 5, 10],
                                     "ring_min": 0.2, "post_s": POST, "house_s": 1.0, "stall_s": 4.0})
        g, rec = rig.g, rig.g.rec
        port = rig.cfg["stations"]["port"]
        alice = Client(port, "alice", PW["alice"])
        bob = Client(port, "bob", PW["bob"])
        vic = Client(port, "vic", PW["vic"])
        onboard = Client(port, "kiosk", key=KIOSK_KEY, local=True, name="captain")

        # F1 all 4 cameras recorded in segments; the file name time matches the picture
        ok_seg = wait_for(lambda: all(len(c.segments()) >= 4 for c in rec.cams.values()), 40)
        states = {n: c.state for n, c in rec.cams.items()}
        segs = rec.cams["FRONT-1"].segments()
        steps = [b[0] - a[0] for a, b in zip(segs, segs[1:])]
        offs = []
        for t, p in segs[1:-1]:
            c = frame_code(p)
            if c is not None:
                offs.append(code_offset_ms(c, t))
        playable = all(frame_code(c.segments()[-2][1]) is not None for c in rec.cams.values())
        report("F1 ring recording: 4 cameras, %d s files (1 min on the boat), playable, file time = picture time" % SEG,
               ok_seg and all(s == "REC" for s in states.values()) and playable and len(steps) >= 2 and
               all(s == SEG for s in steps[1:]) and offs and all(-300 <= o <= 1500 for o in offs),
               "states %s; FRONT-1 files every %s s (the first one after the start is shorter); picture time - file name time: %s ms (keyframe after the "
               "boundary + relay); every camera's file plays: %s" % (states, sorted(set(steps)), offs,
                                                                    "yes" if playable else "NO"))

        # F2 ring length: set by the captain (on board) or HELM; old files deleted; VIEWER refused
        r_vic = send_wait(vic, t="rec", ring_min=0.1)
        r_bad = send_wait(onboard, t="rec", ring_min=7)
        r_ok = send_wait(onboard, t="rec", ring_min=0.1)
        time.sleep(12)
        now = time.time()
        oldest = {n: (now - c.segments()[0][0]) if c.segments() else None for n, c in rec.cams.items()}
        counts = {n: len(c.segments()) for n, c in rec.cams.items()}
        report("F2 ring length set on board (5/10/20/30/60 min on the boat); older files deleted; VIEWER refused",
               r_vic and not r_vic["ok"] and r_bad and not r_bad["ok"] and r_ok and r_ok["ok"] and rec.ring_min == 0.1 and
               all(o is not None and o <= 6 + 2 * SEG + 2.5 for o in oldest.values()) and
               all(n <= 7 for n in counts.values()),
               "viewer: %s; 7 min: %s; 0.1 min (6 s): %s; oldest file per camera %s s, files %s" % (
                   r_vic and r_vic["text"], r_bad and r_bad["text"], r_ok and r_ok["text"],
                   {k: round(v, 1) for k, v in oldest.items()}, counts))
        send_wait(onboard, t="rec", ring_min=0.2)
        time.sleep(14)                                      # the ring fills to 12 s again

        # F3 SAVE from a station: ring window before + post-time after, metadata, hard links
        r_v = send_wait(vic, t="save")
        t0 = time.time()
        a = send_wait(alice, t="save", source="captain")
        eid = a and a.get("id")
        wait_for(lambda: (event_json(rec, eid) or {}).get("state") in ("complete", "incomplete"), POST + 3 * SEG + 10)
        ev = event_json(rec, eid) or {}
        cov = {}
        for n, files in ev.get("cameras", {}).items():
            ts = [seg_time(f) for f in files]
            cov[n] = (round(min(ts) - t0, 1), round(max(ts) + SEG - t0, 1), len(ts)) if ts else None
        b = ev.get("boat", {})
        f1 = os.path.join(rec.events_dir(), eid or "x", "FRONT-1", (ev.get("cameras", {}).get("FRONT-1") or ["x"])[-1])
        links = os.stat(f1).st_nlink if os.path.exists(f1) else 0
        report("F3 SAVE: the ring window before + post-time after, all 4 cameras, metadata, log link; VIEWER refused",
               r_v and not r_v["ok"] and a and a["ok"] and ev.get("state") == "complete" and len(cov) == 4 and
               all(c and c[0] <= -12 + SEG + 0.5 and c[1] >= POST - 0.5 for c in cov.values()) and
               "lat" in b and b.get("mode") == "STANDBY" and b.get("log", {}).get("file") and
               ev["triggers"][0]["kind"] == "button" and ev["triggers"][0].get("who") == "alice",
               "viewer: %s; %s -> %s; covered per camera (from s, to s, files): %s; position %s %s, mode %s, "
               "alarms %s, log %s record %s; trigger %s; hard links on a file: %d" % (
                   r_v and r_v["text"], a and a["text"], ev.get("state"), cov, b.get("lat"), b.get("lon"), b.get("mode"),
                   b.get("alarms"), b.get("log", {}).get("file"), b.get("log", {}).get("aurix_record"),
                   ev.get("triggers", [{}])[0].get("text"), links))

        # F4 triggers merge: a second SAVE while the first is open extends it
        a1 = send_wait(alice, t="save", source="driver")
        time.sleep(2)
        a2 = send_wait(bob, t="save", source="shore joystick")
        e1 = a1 and a1.get("id")
        wait_for(lambda: (event_json(rec, e1) or {}).get("state") == "complete", POST + 3 * SEG + 12)
        ev = event_json(rec, e1) or {}
        span = ev.get("end_t", 0) - ev.get("triggers", [{"t": 0}])[0]["t"]
        report("F4 triggers merge: a SAVE during the post-time joins the open event and extends it",
               a1 and a2 and a1["ok"] and a2["ok"] and a2.get("id") == e1 and len(ev.get("triggers", [])) == 2 and
               POST + 1.5 <= span <= POST + 3.0 and ev.get("state") == "complete",
               "first %s; second: %s; triggers %s; end - first trigger %.1f s (post %d s + 2 s)" % (
                   a1 and a1["text"], a2 and a2["text"], [t_["text"] for t_ in ev.get("triggers", [])], span, POST))

        # F5 automatic SAVE from the AURIX: joystick takeover of HDG LOCK (TELEM2 SAVE request)
        link, w = rig.sim.station()
        n0 = rec.saves
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        hdg = w.until(lambda d: d["mode"] == 2, 5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.6, 0.0))
        man = w.until(lambda d: d["mode"] == 1, 5)
        wait_for(lambda: rec.saves > n0, 5)
        ae = [e for _t, _f, e in rec.list_events() if e["triggers"][0].get("who") == "AURIX"]
        txt = ae[-1]["triggers"][0]["text"] if ae else None
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        report("F5 automatic SAVE from the boat: joystick takeover (TELEM2 SAVE request) -> event",
               hdg is not None and man is not None and ae and ae[-1]["triggers"][0]["kind"] == "takeover" and
               "HDG_LOCK" in (txt or ""),
               "HDG LOCK %s, joystick -> MANUAL %s; event %s: \"%s\"" % (
                   "yes" if hdg else "NO", "yes" if man else "NO", ae[-1]["id"] if ae else None, txt))
        link.stop()
        wait_for(lambda: not rec.events, POST + 3 * SEG + 12)

        # F6 storage policy: oldest ring first, then synced events; never un-synced events; STORAGE FULL
        rec.cfg["house_s"] = 1e9                            # this test runs the housekeeping itself
        st = g.storage
        made = []
        for i in range(5):
            eid_ = "20200101-00000%d_test" % i
            p = os.path.join(rec.events_dir(), eid_)
            os.makedirs(os.path.join(p, "FRONT-1"), exist_ok=True)
            with open(os.path.join(p, "FRONT-1", "x.ts"), "wb") as f:
                f.write(os.urandom(4_000_000))
            with open(os.path.join(p, "event.json"), "w") as f:
                _json.dump({"id": eid_, "start_t": 1577836800 + i, "triggers": [{"text": "t"}]}, f)
            if i < 3:                               # confirmed by the shore: <file>.synced for every file
                for fn in (os.path.join(p, "FRONT-1", "x.ts"), os.path.join(p, "event.json")):
                    open(fn + ".synced", "w").close()
            made.append(eid_)
        used = st.used_now()
        ring_del = sum(os.path.getsize(p) for c in rec.cams.values() for _t, p in c.segments()[:-1])
        st.quota_b = used + 1_000_000
        st.low_b = 1_000_000 + ring_del + 6_000_000
        st.full_b = 0
        rec.housekeeping()
        left = [e for e in made if os.path.exists(os.path.join(rec.events_dir(), e))]
        ring_left = {n: len(c.segments()) for n, c in rec.cams.items()}
        step1 = left == made[2:] and all(v <= 2 for v in ring_left.values())
        # nothing synced left to delete -> STORAGE FULL: SAVE refused, GATEWAY STORAGE alarm on the boat
        st.low_b = st.full_b = st.free_now() + 50_000_000
        rec.housekeeping()
        left2 = [e for e in made if os.path.exists(os.path.join(rec.events_dir(), e))]
        r_full = send_wait(alice, t="save")
        alice.pump(0.2)
        alarm = wait_for(lambda: (alice.pump(0.1) or True) and alice.telem and alice.telem["alarm_bits"] & 0x20000, 8)
        st.quota_b, st.low_b, st.full_b = None, 0, 0
        st.check(force=True)
        rec.housekeeping()
        r_ok2 = send_wait(alice, t="save")
        report("F6 storage: oldest ring files first, then SYNCED events (oldest first), never un-synced; STORAGE FULL",
               step1 and left2 == made[3:] and rec.deleted["events"] >= 3 and r_full and not r_full["ok"] and
               "STORAGE FULL" in r_full["text"] and alarm and r_ok2 and r_ok2["ok"],
               "after the first pass: events left %s (2 oldest synced deleted), ring files left %s; nothing synced left: "
               "events left %s (only un-synced), SAVE -> \"%s\", GATEWAY STORAGE alarm on the boat %s; space back -> "
               "SAVE %s" % ([e[-6:] for e in left], ring_left, [e[-6:] for e in left2], r_full and r_full["text"],
                            "yes" if alarm else "NO", "ok" if r_ok2 and r_ok2["ok"] else "NO"))
        rec.cfg["house_s"] = 1.0

        # F7 a camera unplugged: its recorder notices and restarts, the others keep recording
        n_before = {n: len(os.listdir(rec.ring_dir(n))) for n in rec.cams}
        last_before = {n: c.segments()[-1][0] for n, c in rec.cams.items()}
        rig.cams[2].unplug()
        down = wait_for(lambda: rec.cams["REAR-LEFT"].state in ("DOWN", "STALLED"), 20)
        time.sleep(3 * SEG)
        others = all(rec.cams[n].segments()[-1][0] > last_before[n] and rec.cams[n].state == "REC"
                     for n in ("FRONT-1", "FRONT-2", "REAR-RIGHT"))
        rig.cams[2] = SimCamera("rtsp://127.0.0.1:%d/simcam_2" % rig.rtsp).start()
        back = wait_for(lambda: rec.cams["REAR-LEFT"].state == "REC" and
                        rec.cams["REAR-LEFT"].segments()[-1][0] > time.time() - 3 * SEG, 40)
        report("F7 camera unplugged: its recording restarts by itself when it is back; the others never stop",
               down and others and back,
               "REAR-LEFT %s; other cameras kept recording %s; REAR-LEFT recording again %s (restarts %d)" % (
                   "DOWN/STALLED" if down else "NOT noticed", "yes" if others else "NO", "yes" if back else "NO",
                   rec.cams["REAR-LEFT"].restarts))
        # F8 log viewer: the saved event's video at a log moment (boat time -> GPS time -> file + offset)
        import videolog
        vi = videolog.VideoIndex(os.path.join(rec.events_dir(), eid))
        lg = tlg.read(g.logs.path) if g.logs.path else None
        to_utc = videolog.log_utc(lg) if lg else None
        errs, found = [], 0
        for dt in (-9.0, -5.0, -1.0, 2.0):
            u = t0 + dt
            fr, loc = vi.frame("FRONT-1", u, videolog.frame_rgb)
            fr = fr or b""
            if loc:
                found += 1
                c = decode_ts(fr, 320, 240)
                if c is not None:
                    errs.append(code_offset_ms(c, u))
                else:
                    errs.append("no picture in %s at +%.1f s (%d bytes)" % (os.path.basename(loc[0]), loc[1], len(fr)))
        utc_err = None
        if to_utc and lg.samples:
            last = lg.samples[-1]["t_ms"]
            utc_err = to_utc(last) - (g.clock.now() - (g.telem["t_ms"] - last) / 1000.0) if g.telem else None
        report("F8 log viewer: video of a saved event at a moment of the log, within 1 s; boat time -> GPS time",
               vi.names() == CAM_NAMES and found == 4 and len(errs) == 4 and all(isinstance(e, int) and abs(e) <= 1000 for e in errs) and
               utc_err is not None and abs(utc_err) < 1.5,
               "cameras in the event %s; picture time - wanted time at -9/-5/-1/+2 s: %s ms; boat log time -> UTC "
               "(TIME events) vs the gateway's GPS clock: %s s" % (
                   vi.names(), errs, None if utc_err is None else round(utc_err, 2)))
        s = rec.status()
        print("    recording status: ring %s MB over %s min (%s GB/h for the simple simulated pictures)" % (
            s["ring_mb"], s["ring_have_min"], s["gb_per_hour"]), flush=True)
        for c in (alice, bob, vic, onboard):
            c.close()
    finally:
        if rig:
            rig.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------- section G --

from syncrecv import SyncReceiver, sha256_file  # noqa: E402
from sync import marker  # noqa: E402
from netsim import Uplink  # noqa: E402

SYNC_KEY = "test-sync-key-0123456789"


def tlg_file(path, incident, pad=20000):
    """A small AURIX-style log file; with an ALARM_ON event if incident."""
    def rec(rtype, t, payload):
        body = bytes([rtype]) + struct.pack("<HI", len(payload), t) + payload
        c = proto.crc16(body)
        return bytes([0xA5]) + body + bytes([c & 0xFF, c >> 8])
    out = b"TLG1" + b"".join(rec(0x10, 1, os.urandom(min(60000, pad - k))) for k in range(0, pad, 60000))
    if incident:
        out += rec(0x20, 2, struct.pack("<Hiiii", 3, 1, 1, 0, 0))
    out += rec(0x10, 3, os.urandom(64))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(out)


def old(path, age=3600):
    t = time.time() - age
    os.utime(path, (t, t))


def put(path, size=None, data=None, age=3600):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data if data is not None else os.urandom(size))
    old(path, age)


def section_g():
    print("\n=== Section G: sync to the shore PC ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_g_")
    sim = gw = recv = net_s = net_st = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        shore = os.path.join(tmp, "shore")
        os.makedirs(root)
        recv = SyncReceiver(shore, {"trygons-c2": SYNC_KEY}, port=free_port(), listen="127.0.0.1").start()
        up = Uplink(2000)                                   # the boat's upload: 2 Mbit/s, shared
        sp = free_port()
        net_s = NetSim(sp, recv.port, PROFILES["lan"], seed=5, uplink=up, up="c2s").start()
        cfg = gw_cfg_b(sim, root, sync={"shore": "127.0.0.1:%d" % sp, "settle_s": 0.5, "scan_s": 0.5,
                                        "max_kbps": 8000},
                       channels={"driver": "sim", "check_s": 0.2, "stable_s": 1.0, "starlink_status": False})
        cfg["secrets"] = {"sync_key": SYNC_KEY}
        cfg["sync"]["enabled"] = False                     # start with sync off: the files first
        gw = GwRunner(cfg)
        g = gw.gw
        sc = g.sync
        stp = cfg["stations"]["port"]
        time.sleep(2.0)

        # G1 priority: black box / alarms -> event logs -> telemetry logs -> saved events; no ring video
        F = {
            "ev_json": ("video/events/20261020-100000_alarm/event.json", 1),
            "tlg_alarm": ("aurix_logs/20261020/P00001_001.TLG", 1),
            "audit": ("audit/20261019.jsonl", 2),
            "gaps": ("aurix_logs/20261020/gaps.csv", 2),
            "tlg_plain": ("aurix_logs/20261020/P00001_002.TLG", 3),
            "ev_video": ("video/events/20261020-100000_alarm/FRONT-1/20261020-095000.ts", 4),
            "ring": ("video/ring/FRONT-1/20261020-101000.ts", 5),
        }
        P = lambda k: os.path.join(root, *F[k][0].split("/"))
        put(P("ev_json"), data=_json.dumps({"id": "20261020-100000_alarm", "state": "complete"}).encode())
        tlg_file(P("tlg_alarm"), True, 200000); old(P("tlg_alarm"))
        put(P("audit"), data=b'{"kind":"test"}\n' * 2000)
        put(P("gaps"), data=b"utc,boot,first_record,last_record,count\n")
        tlg_file(P("tlg_plain"), False, 300000); old(P("tlg_plain"))
        put(P("ev_video"), 1_500_000)
        put(P("ring"), 500_000)
        files_before = sum(len(f) for _d, _s, f in os.walk(root))
        sc.enabled = True
        mine = {F[k][0]: F[k][1] for k in F if k != "ring"}
        ok1 = wait_for(lambda: all(r in sc.done_list for r in mine), 60)
        order = [r for r in sc.done_list if r in mine]
        classes = [mine[r] for r in order]
        same = all(os.path.exists(os.path.join(shore, "trygons-c2", *r.split("/"))) and
                   sha256_file(os.path.join(shore, "trygons-c2", *r.split("/"))) == sha256_file(
                       os.path.join(root, *r.split("/"))) for r in mine)
        ring_sent = os.path.exists(os.path.join(shore, "trygons-c2", *F["ring"][0].split("/")))
        report("G1 sync order: black box / alarms, event logs, telemetry logs, saved events; ring video not sent (off)",
               ok1 and classes == sorted(classes) and same and not ring_sent,
               "confirmed in this order (class): %s; identical on shore (SHA-256) %s; ring video sent: %s" % (
                   ["%s(%d)" % (r.split("/")[-1], mine[r]) for r in order], "yes" if same else "NO",
                   "YES" if ring_sent else "no"))

        # G2 resumable: the connection breaks in the middle -> continues where the shore copy ends
        big = "video/events/20261020-110000_button/FRONT-1/20261020-105000.ts"
        put(os.path.join(root, *big.split("/")), 3_000_000)
        put(os.path.join(root, "video/events/20261020-110000_button/event.json"),
            data=_json.dumps({"id": "x", "state": "complete"}).encode())
        b0 = sc.sent_b
        sc.cut_after = b0 + 1_200_000
        ok2 = wait_for(lambda: big in sc.done_list, 60)
        sent = sc.sent_b - b0
        report("G2 resumable: connection cut after 1.2 of 3 MB -> continues from the shore copy, SHA-256 checked",
               ok2 and sc.resumed >= 1 and sent < 3_000_000 * 1.2 and
               sha256_file(os.path.join(shore, "trygons-c2", *big.split("/"))) ==
               sha256_file(os.path.join(root, *big.split("/"))),
               "confirmed %s; resumed %d time(s); %.2f MB sent for a 3 MB file (+ event.json)" % (
                   "yes" if ok2 else "NO", sc.resumed, sent / 1e6))

        # G3 a damaged shore copy is found (checksum) and sent again; "synced" only after the shore's OK
        big2 = "video/events/20261020-120000_button/FRONT-1/20261020-115000.ts"
        lp = os.path.join(root, *big2.split("/"))
        put(lp, 2_000_000)
        put(os.path.join(root, "video/events/20261020-120000_button/event.json"),
            data=_json.dumps({"id": "y", "state": "complete"}).encode())
        r0 = sc.restarted
        sc.hold_after_cut = True
        sc.cut_after = sc.sent_b + 800_000
        held = wait_for(lambda: sc.hold, 30)
        part = os.path.join(shore, "trygons-c2", *big2.split("/")) + ".part"
        wait_for(lambda: os.path.exists(part), 5)
        with open(part, "r+b") as f:                        # a bit flips on the shore disk
            f.seek(1000)
            b = f.read(1)
            f.seek(1000)
            f.write(bytes([b[0] ^ 0xFF]))
        no_marker = not os.path.exists(lp + ".synced")
        recv.fail_done = True                               # and the first "done" is answered "bad"
        sc.hold_after_cut = False
        sc.hold = False
        ok3 = wait_for(lambda: big2 in sc.done_list, 60)
        m = marker(lp) or {}
        files_after = sum(1 for _d, _s, f in os.walk(root) for n in f if not n.endswith(".synced"))
        report("G3 damaged shore copy detected (SHA-256) and sent again; synced only after the shore's OK; nothing deleted",
               held and no_marker and ok3 and sc.restarted >= r0 + 1 and m.get("sha") == sha256_file(lp) and
               files_after >= files_before + 4,
               "no 'synced' mark while the shore had only part: %s; restarts %d; final mark %s (matches %s); files on "
               "the boat before %d, after %d (only new ones added: sync deletes nothing)" % (
                   no_marker, sc.restarted - r0, (m.get("sha") or "-")[:12],
                   "yes" if m.get("sha") == sha256_file(lp) else "NO", files_before, files_after))

        # G4 channel: STARLINK (emergency, metered) -> no sync; back on 4G -> continues
        gw.call(g.net.request, "MANUAL", "STARLINK", "test", True, True, True, True)
        gw.call(g.net.tick)
        paused = wait_for(lambda: sc.state == "PAUSED", 5)
        put(os.path.join(root, "audit/20261018.jsonl"), data=b'{"kind":"x"}\n' * 500)
        b1 = sc.sent_b
        time.sleep(3)
        none_sent = sc.sent_b == b1 and "audit/20261018.jsonl" not in sc.done_list
        why = sc.reason
        gw.call(g.net.request, "AUTO", None, "test", True, True, False, True)
        gw.call(g.net.tick)
        ok4 = wait_for(lambda: "audit/20261018.jsonl" in sc.done_list, 20)
        report("G4 STARLINK (emergency link): sync paused, nothing sent; back on 4G: sync continues",
               paused and none_sent and ok4,
               "on STARLINK: %s (\"%s\"), bytes sent in 3 s: %d; back on %s: synced %s" % (
                   sc.state if not ok4 else "PAUSED", why, sc.sent_b - b1, g.net.active, "yes" if ok4 else "NO"))

        # G5 spare bandwidth only: a shore station's commands over the SAME 2 Mbit/s uplink
        net_st = NetSim(free_port(), stp, PROFILES["4g"], seed=6, uplink=up, up="s2c").start()
        alice = Client(net_st.listen_port, "alice", PW["alice"])
        alice.pump(1.0)

        def rtts(seconds):
            out = []
            end = time.time() + seconds
            while time.time() < end:
                t0 = time.time()
                if alice.cmd(proto.T_PARAM_GET, bytes([0]), timeout=5) is not None:
                    out.append((time.time() - t0) * 1000)
                time.sleep(0.2)
            return sorted(out)

        quiet = rtts(4)
        for k in range(3):
            put(os.path.join(root, "video/events/20261020-13000%d_button/FRONT-1/20261020-125000.ts" % k), 4_000_000)
            put(os.path.join(root, "video/events/20261020-13000%d_button/event.json" % k),
                data=_json.dumps({"id": "z", "state": "complete"}).encode())
        wait_for(lambda: sc.state == "SYNCING", 10)
        b2, t2 = sc.sent_b, time.time()
        busy = rtts(10)
        thr = (sc.sent_b - b2) * 8 / (time.time() - t2) / 1000
        med = lambda v: v[len(v) // 2] if v else -1
        # for comparison: a sender that ignores the delay (fixed 8 Mbit/s)
        sc.target, sc.min_rate, sc.rate = 100.0, 8000 * 125.0, 8000 * 125.0
        naive = rtts(5)
        sc.target, sc.min_rate, sc.rate = 0.1, 32 * 125.0, 256 * 125.0
        report("G5 sync uses spare bandwidth only: station command round trip stays low on a shared 2 Mbit/s uplink",
               quiet and busy and med(busy) < med(quiet) + 250 and thr > 600,
               "station round trip (4G): without sync median %d ms; while syncing median %d ms, 95 %% %d ms, sync "
               "%.0f kbit/s of 2000; a sender ignoring the delay would give median %d ms" % (
                   med(quiet), med(busy), busy[int(len(busy) * 0.95) - 1] if busy else -1, thr, med(naive)))
        LATENCY["sync_station_rtt"] = {"without": med(quiet), "with": med(busy), "naive": med(naive), "kbps": thr}

        # G6 REMOTE MANUAL: sync pauses (the link belongs to the driver); afterwards it continues
        link, w = sim.station()
        stick = {"x": 0.0, "lever": 0.5}
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        alice.cmd(proto.T_REMOTE, proto.p_remote(0))
        time.sleep(1.0)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.2)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        on = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 3)
        p6 = wait_for(lambda: sc.state == "PAUSED" and "REMOTE" in sc.reason, 5)
        why6 = sc.reason
        b3 = sc.sent_b
        time.sleep(2)
        quiet6 = sc.sent_b - b3 < 64 * 1024
        alice.cmd(proto.T_REMOTE, proto.p_remote(1))
        snd.stop()
        off = w.until(lambda d: d["mode"] != proto.MODE_REMOTE, 3)
        again = wait_for(lambda: sc.state in ("SYNCING", "IDLE", "CONNECTING"), 8)
        report("G6 REMOTE MANUAL driving: sync pauses; when it ends sync continues",
               on is not None and p6 and quiet6 and off is not None and again,
               "remote driving %s -> sync %s (\"%s\"), bytes in 2 s %d; driving ended -> sync %s" % (
                   "on" if on else "NOT on", "PAUSED" if p6 else sc.state, why6, sc.sent_b - b3,
                   sc.state))
        link.stop()
        alice.close()
        s = sc.status()
        print("    sync status: %s; %d files confirmed, %.1f MB sent, %d resumed, %d restarted" % (
            s["state"], s["confirmed"], s["sent_mb"], s["resumed"], s["restarted"]), flush=True)
    finally:
        for x in (net_st, net_s, recv):
            if x:
                x.stop()
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section F1 --

class DriverBeat:
    """A station that shows a DRIVER screen: heartbeat every 0.5 s (stop = crash)."""

    def __init__(self, client):
        self.c = client
        self.on = True
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        while self.on:
            try:
                self.c.l.send_msg({"t": "driver", "open": True})
            except Exception:
                return
            time.sleep(0.5)

    def stop(self):
        self.on = False


def next_event(client, kinds, timeout=5.0):
    """The next gateway message of one of these kinds (others are kept as usual)."""
    end = time.time() + timeout
    while time.time() < end:
        try:
            k, d = client.l.events.get(timeout=0.1)
        except Exception:
            continue
        if k in kinds:
            return k, d
        if k == "ack":
            client.acks.append(d)
        elif k == "telemetry":
            client.telem = d
        elif k == "gw_status":
            client.status = d
    return None, None


def section_f1():
    print("\n=== F1 / section D: copilot actions need the captain's ACCEPT (screen 1), enforced in the gateway ===",
          flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_f1_")
    sim = gw = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root, stations={"driver_timeout_s": 2.0, "driver_beat_s": 1.5})
        gw = GwRunner(cfg)
        sp = cfg["stations"]["port"]
        time.sleep(1.0)
        g = gw.gw
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        bob = Client(sp, "bob", PW["bob"], name="copilot")             # licence too, but the second: copilot
        bob.pump(1.0)
        SEA = lambda n: proto.p_sea_state(n)

        def sea():
            alice.pump(0.6)
            return alice.telem["sea_state"] if alice.telem else None

        # F1a the captain's screen 1 not open: the copilot's driving actions refused; STANDBY, hold, alarm ack
        # and non-driving commands still work; the captain's own action goes straight through
        r_sea = bob.result(bob.cmd(proto.T_SEA_STATE, SEA(3)))
        r_par = bob.result(bob.cmd(proto.T_PARAM_SET, proto.p_param_set(0, 1.0)))
        r_sb = bob.result(bob.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY)))
        r_hold = bob.result(bob.cmd(proto.T_AUTO_PAUSE))
        r_ack = bob.result(bob.cmd(proto.T_ALARM_ACK))
        r_helm = bob.result(bob.cmd(proto.T_HELM, proto.p_helm(0)))
        r_cap = alice.result(alice.cmd(proto.T_SEA_STATE, SEA(2)))
        crews = {s.name: s.crew for s in g.stations.stations.values()}
        report("F1a copilot without the captain's screen 1: driving actions REFUSED; STANDBY / hold / ack direct; "
               "the captain's own action direct; a copilot can't take HELM",
               r_sea == 109 and r_par == 109 and r_sb == 0 and r_hold not in (None, 109) and r_ack == 0 and
               r_helm == 117 and r_cap == 0 and crews == {"captain": "captain", "copilot": "copilot"},
               "crew %s; copilot: sea state %s, parameter %s (109 = screen not open); STANDBY %s, AUTO pause %s "
               "(sent: the boat answers 7 = not in AUTO), alarm ack %s; HELM take %s (117); captain's own sea state %s" % (
                   crews, r_sea, r_par, r_sb, r_hold, r_ack, r_helm, r_cap))

        # F1b the captain's screen 1 open: copilot action -> ACCEPT -> sent; REJECT -> refused; no answer -> refused
        beat = DriverBeat(alice)
        time.sleep(1.0)
        s0 = sea()
        new = 3 if s0 != 3 else 4
        seq = bob.l.send(proto.T_SEA_STATE, SEA(new), what="sea state %d" % new)
        k, ask = next_event(alice, ("driver_ask",))
        alice.l.send_msg({"t": "driver_answer", "id": ask["id"], "yes": True})
        a_yes = bob.result(bob.wait_ack(seq))
        s1 = sea()
        seq = bob.l.send(proto.T_SEA_STATE, SEA(s0), what="sea state back")
        k, ask2 = next_event(alice, ("driver_ask",))
        bob.l.send_msg({"t": "driver_answer", "id": ask2["id"], "yes": True})   # the copilot can't answer it
        alice.l.send_msg({"t": "driver_answer", "id": ask2["id"], "yes": False})
        a_no = bob.result(bob.wait_ack(seq))
        t0 = time.time()
        seq = bob.l.send(proto.T_SEA_STATE, SEA(s0))
        a_to = bob.result(bob.wait_ack(seq, timeout=5))
        dt = time.time() - t0
        s2 = sea()
        report("F1b captain's ACCEPT on screen 1 -> sent; REJECT -> refused; no answer within the timeout -> refused",
               ask and ask["text"] == "sea state %d" % new and ask["from"] == "copilot" and a_yes == 0 and s1 == new and
               a_no == 110 and a_to == 111 and 1.5 < dt < 4.0 and s2 == new,
               "question on the captain's screen 1: \"%s\" from %s; ACCEPT -> %s, boat sea state %s -> %s; "
               "REJECT (the copilot's own answer ignored) -> %s; no answer -> %s after %.1f s; sea state still %s" % (
                   ask and ask["text"], ask and ask["from"], a_yes, s0, s1, a_no, a_to, dt, s2))

        # F1c the captain's screen 1 crashes / is closed while a question is open -> rejected
        seq = bob.l.send(proto.T_SEA_STATE, SEA(s0))
        k, ask3 = next_event(alice, ("driver_ask",))
        beat.stop()                                         # crash: the heartbeat stops
        t0 = time.time()
        a_crash = bob.result(bob.wait_ack(seq, timeout=5))
        t_crash = time.time() - t0
        beat = DriverBeat(alice)
        time.sleep(0.8)
        seq = bob.l.send(proto.T_SEA_STATE, SEA(s0))
        k, ask4 = next_event(alice, ("driver_ask",))
        beat.stop()
        alice.l.send_msg({"t": "driver", "open": False})    # screen 1 closed
        a_gone = bob.result(bob.wait_ack(seq, timeout=5))
        r_after = bob.result(bob.cmd(proto.T_SEA_STATE, SEA(s0)))
        report("F1c captain's screen 1 crashes / is closed during a question -> the copilot's action is rejected",
               ask3 is not None and a_crash == 112 and t_crash < 3.5 and ask4 is not None and a_gone == 112 and
               r_after == 109 and sea() == new,
               "heartbeat stopped -> %s after %.1f s (112 = screen lost); closed -> %s; next command -> %s; "
               "nothing changed on the boat" % (a_crash, t_crash, a_gone, r_after))
        bob.close()

        # F1d SINGLE-OPERATOR mode: own confirmation; shown; logged; off when the station disconnects
        alice.l.send_msg({"t": "single_op", "on": True})
        k, so = next_event(alice, ("single_op_ack",))
        r_nc = alice.result(alice.cmd(proto.T_SEA_STATE, SEA(s0)))
        seq = alice.l.send(proto.T_SEA_STATE, SEA(s0), what="sea state", confirm="self")
        r_c = alice.result(alice.wait_ack(seq))
        s3 = sea()
        alice.pump(1.2)
        shown = any(s["single_op"] for s in (alice.status or {}).get("stations", []) if s["id"] == alice.status["you"])
        alice.close()
        wait_for(lambda: any(e["kind"] == "single_operator" and not e["on"] for e in g.audit.recent), 8)
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.5)
        off = not any(s["single_op"] for s in alice.status.get("stations", []))
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        r_again = alice.result(alice.cmd(proto.T_SEA_STATE, SEA(new)))
        logged = [e for e in g.audit.recent if e["kind"] == "single_operator"]
        report("F1d SINGLE-OPERATOR: driving action only with the operator's own confirmation; shown in the status; "
               "logged; OFF after a disconnect",
               so and so["on"] and r_nc == 113 and r_c == 0 and s3 == s0 and shown and off and r_again == 0 and
               len(logged) >= 2 and logged[0]["on"] and not logged[-1]["on"],
               "switched on %s; without own confirmation %s (113); with it %s, sea state %s; status shows it %s; "
               "after reconnect off %s, the captain's command %s (direct again); audit %s" % (
                   so and so["on"], r_nc, r_c, s3, shown, off, r_again,
                   [(e["on"], e.get("why", "")) for e in logged]))

        # F1e the real ground station through the gateway: driver window answers the gateway's question,
        # closing it mid-question rejects; single-operator banners on both screens + own confirmation
        import tkinter as tk
        import app as A
        lay = None
        if os.path.exists(A.LAYOUT):
            with open(A.LAYOUT, "rb") as f:
                lay = f.read()
            os.remove(A.LAYOUT)
        alice.close()
        troot = tk.Tk()
        ap = A.App(troot, "gw:127.0.0.1:%d" % sp, login=("alice", PW["alice"]))

        def pump(s, until=None):
            end = time.time() + s
            while time.time() < end:
                troot.update()
                if until and until():
                    return True
                time.sleep(0.02)
            return bool(until and until())
        try:
            pump(8, lambda: ap.t is not None)
            ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
            pump(1.0)
            ap.open_driver()
            pump(2.5)
            cop = Client(sp, "bob", PW["bob"], name="copilot")
            seq = cop.l.send(proto.T_SEA_STATE, proto.p_sea_state(s0), what="sea state %d" % s0)
            asked = pump(4, lambda: ap.driver.question is not None)
            ap.driver.answer(True)
            sent = pump(5, lambda: ap.t["sea_state"] == s0)
            seq = cop.l.send(proto.T_SEA_STATE, proto.p_sea_state(new), what="sea state %d" % new)
            asked2 = pump(4, lambda: ap.driver.question is not None)
            ap.close_driver()                                   # the driver window is closed mid-question
            end = time.time() + 6
            a_lost = None
            while time.time() < end and a_lost is None:
                pump(0.1)
                a_lost = cop.wait_ack(seq, timeout=0.05)
            lost = a_lost is not None and a_lost["result"] in (109, 112)
            cop.close()
            ap.single_op_toggle(confirmed=True)
            so_on = pump(4, lambda: ap.single_op)
            ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
            ap.open_driver()
            pump(1.5)
            banner_c = bool(ap.so_banner.winfo_manager())
            banner_d = bool(ap.driver.status.find_withtag("singleop"))
            ap.cmd(proto.T_SEA_STATE, proto.p_sea_state(new), "sea state %d" % new)
            pump(0.5)
            own = ap.cconf is not None and ap.driver.question is None
            ap.cconf_send()
            sent2 = pump(5, lambda: ap.t["sea_state"] == new)
            report("F1e ground station: the captain's driver window answers the copilot's question; closing it "
                   "rejects; single-operator banner on both screens + own confirmation",
                   asked and sent and asked2 and lost and so_on and banner_c and banner_d and own and sent2,
                   "question shown on the driver window %s, ACCEPT -> boat %s; closed mid-question -> rejected %s; "
                   "single-operator on %s, banner captain %s / driver %s; own confirmation window %s -> sent %s %s" % (
                       asked, sent, lost, so_on, banner_c, banner_d, own, sent2, "" if sent2 else ap.ev_texts()[-6:]))
        finally:
            ap.quit()
            if lay is not None:
                with open(A.LAYOUT, "wb") as f:
                    f.write(lay)
            elif os.path.exists(A.LAYOUT):
                os.remove(A.LAYOUT)
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ----------------------------------------------------------------- section AIS --

import ais as aismod  # noqa: E402
from simais import AisSim, Ship  # noqa: E402
import http.server as httpsrv  # noqa: E402  (not "http": section B has an http() helper)

# published test sentences (gpsd / pyais test data) and their published decode
VEC_A = "!AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C"           # type 1: MMSI 477553000, 47.58283 N 122.34583 W
VEC_5 = ["!AIVDM,2,1,1,A,55?MbV02;H;s<HtKR20EHE:0@T4@Dn2222222216L961O5Gf0NSQEp6ClRp8,0*1C",
         "!AIVDM,2,2,1,A,88888888880,2*25"]                           # type 5: EVER DIADEM, 3FOF8, NEW YORK
VEC_18 = "!AIVDM,1,1,,B,B5NJ;PP005l4ot5Isbl03wsUkP06,0*75"         # type 18: MMSI 367430530, 37.785035 N 122.26732 W


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def section_ais():
    print("\n=== Section A: AIS receiver (receive only) ===", flush=True)

    # AIS1 decoding: standard test sentences (Class A, Class B, static), 2-part messages, bad checksum
    T = aismod.AisTargets({})
    a = T.feed_line(VEC_A)
    p1 = T.feed_line(VEC_5[0])
    s5 = T.feed_line(VEC_5[1])
    b = T.feed_line(VEC_18)
    bad = T.feed_line(VEC_A.replace("*5C", "*5D"))
    m19 = {"type": 24, "mmsi": 239555000, "part": 0, "name": "KAYAK TOUR"}
    r24a = T.feed_line(aismod.sentences(m19)[0])
    r24b = T.feed_line(aismod.sentences({"type": 24, "mmsi": 239555000, "part": 1, "ship_type": 37, "callsign": "SV1AB",
                                         "to_bow": 5, "to_stern": 5, "to_port": 1, "to_starboard": 1})[0])
    ok_a = a and a["mmsi"] == 477553000 and abs(a["lat"] - 47.582833) < 1e-5 and abs(a["lon"] + 122.345833) < 1e-5 \
        and a["nav_status"] == 5 and a["cog"] == 51.0 and a["heading"] == 181
    ok_5 = p1 is None and s5 and s5["name"] == "EVER DIADEM" and s5["callsign"] == "3FOF8" and \
        s5["destination"] == "NEW YORK" and s5["draught"] == 12.2 and s5["to_bow"] + s5["to_stern"] == 295
    ok_18 = b and b["mmsi"] == 367430530 and abs(b["lat"] - 37.785035) < 1e-5 and abs(b["lon"] + 122.26732) < 1e-5
    t24 = T.targets[239555000]
    report("AIS1 decoding: standard Class A / Class B / static (2-part) sentences, Class B static, bad checksum",
           ok_a and ok_5 and ok_18 and bad is None and T.bad_crc == 1 and t24.name == "KAYAK TOUR" and
           t24.length == 10 and t24.ship_type == 37,
           "type 1: %s %.5f %.5f status %s; type 5 (2 parts): %s / %s / %s / draught %s; type 18: %s %.5f %.5f; "
           "bad checksum refused (%d); type 24 A+B: %s, %s m, type %s" % (
               a["mmsi"], a["lat"], a["lon"], a["nav_status"], s5["name"], s5["callsign"], s5["destination"],
               s5["draught"], b["mmsi"], b["lat"], b["lon"], T.bad_crc, t24.name, t24.length, t24.ship_type))

    # AIS2 targets: prediction, stale (A 3 min, B 6 min), removal, invalid data, messages / minute
    clk = FakeClock()
    T = aismod.AisTargets({"stale_a_s": 180, "stale_b_s": 360, "remove_s": 600}, clock=clk)
    for line in aismod.sentences({"type": 1, "mmsi": 239100001, "lat": 37.8, "lon": 23.7, "sog": 10.0, "cog": 90.0}):
        T.feed_line(line)
    for line in aismod.sentences({"type": 18, "mmsi": 239100002, "lat": 37.81, "lon": 23.71, "sog": 5.0, "cog": 0.0}):
        T.feed_line(line)
    clk.t += 60
    lat_p, lon_p = T.targets[239100001].predict(clk.t)
    moved = aismod.dist_m(37.8, 23.7, lat_p, lon_p)
    clk.t += 130                                          # 190 s after the reports
    st1 = {m: T.is_stale(t) for m, t in T.targets.items()}
    clk.t += 200                                          # 390 s
    st2 = {m: T.is_stale(t) for m, t in T.targets.items()}
    n_before = T.invalid
    T.feed_line(aismod.sentences({"type": 1, "mmsi": 12345, "lat": 37.8, "lon": 23.7, "sog": 1, "cog": 0})[0])
    T.feed_line(aismod.sentences({"type": 1, "mmsi": 239100001, "lat": 38.5, "lon": 23.7, "sog": 10, "cog": 90})[0])
    T.feed_line(aismod.sentences({"type": 1, "mmsi": 239100003, "lat": None, "lon": None, "sog": 3, "cog": 0})[0])
    jumped = T.targets[239100001].lat
    clk.t += 700
    T.tick()
    gone = 239100001 not in T.targets and 239100002 not in T.targets
    report("AIS2 targets: predicted between reports, stale after 3 min (A) / 6 min (B), removed later, bad data ignored",
           abs(moved - 10 * aismod.KN * 60) < 1.0 and st1 == {239100001: True, 239100002: False} and
           st2 == {239100001: True, 239100002: True} and T.invalid - n_before == 2 and jumped == 37.8 and gone,
           "predicted %.0f m after 60 s at 10 kn (expected %.0f); stale at 190 s: %s; at 390 s: %s; invalid MMSI + "
           "70 km jump refused (%d), no-position report kept out; removed after 15 min: %s" % (
               moved, 10 * aismod.KN * 60, st1, st2, T.invalid - n_before, gone))

    # AIS3 the gateway with simulated traffic: list, names, rate; RECEIVER SILENT alarm on the boat
    tmp = tempfile.mkdtemp(prefix="gwtest_ais_")
    sim = gw = asim = http_srv = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        aport, hport = free_port(), free_port()
        asim = AisSim(aport, seed=3, noise_m=8.0, drop=0.1, bad=0.03, start_offset=0.5).start()
        for i, (cls, name) in enumerate((("A", "BLUE STAR DELOS"), ("A", "AEGEAN SPIRIT"), ("B", "SAILING ELENI"))):
            asim.add(Ship(239200001 + i, 37.83 + 0.01 * i, 23.62 + 0.01 * i, 12.0 - 3 * i, 45.0 * i, cls=cls,
                          name=name))
        asim.add(Ship(239200009, 37.90, 23.55, 26.0, 270.0, cls="A", name="FLYING CAT"))   # 26 kn: every 2 s

        class Hub(httpsrv.BaseHTTPRequestHandler):
            hits = 0

            def do_GET(self):
                Hub.hits += 1
                body = _json.dumps([{"ERROR": False}, [
                    {"MMSI": 239300001, "TIME_UNIX": int(time.time()) - 20, "LATITUDE": 37.70, "LONGITUDE": 23.80,
                     "COG": 200.0, "SOG": 15.0, "HEADING": 199, "NAME": "ONLINE ONLY", "TYPE": 60, "A": 100, "B": 30},
                    {"MMSI": 239200001, "TIME_UNIX": int(time.time()) - 600, "LATITUDE": 37.50, "LONGITUDE": 23.50,
                     "COG": 0.0, "SOG": 5.0, "HEADING": 0, "NAME": "OLD ONLINE COPY", "TYPE": 60}]]).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *a):
                pass
        http_srv = httpsrv.HTTPServer(("127.0.0.1", hport), Hub)
        threading.Thread(target=http_srv.serve_forever, daemon=True).start()
        cfg = gw_cfg_b(sim, root, ais={"enabled": True, "source": "tcp:127.0.0.1:%d" % aport, "silent_s": 8.0,
                                       "tick_s": 0.25, "second_gps": True,
                                       "online": {"enabled": True, "period_s": 1.0, "radius_deg": 0.5,
                                                  "url": "http://127.0.0.1:%d/ws?lat0={lat0}&lat1={lat1}" % hport}},
                       channels={"driver": "sim", "check_s": 0.2, "stable_s": 1.0, "starlink_status": False})
        gw = GwRunner(cfg)
        g = gw.gw
        link, w = sim.station()
        time.sleep(1.0)
        got = wait_for(lambda: len(g.ais.targets) >= 5 and all(
            g.ais.targets.get(239200001 + i) and g.ais.targets[239200001 + i].name for i in range(3)), 25)
        names = sorted(t.name for t in g.ais.targets.values())
        st = g.ais_status()
        report("AIS3 gateway: simulated traffic (Class A + B, noise, 10 % lost, 3 % broken) -> target list with names",
               got and st["msgs_per_min"] > 3 and st["bad_crc"] >= 0 and g.ais_reader.connected,
               "targets %s; %d messages / min, %d broken lines refused, %d sent by the simulator (%d lost)" % (
                   names, st["msgs_per_min"], st["bad_crc"], asim.sent, asim.dropped))

        # AIS4 online feed: merged by MMSI, local wins when newer; never on STARLINK
        on_only = g.ais.targets.get(239300001)
        loc = g.ais.targets.get(239200001)
        polls0 = g.ais_online.polls
        gw.call(g.net.request, "MANUAL", "STARLINK", "test", True, True, True, True)
        gw.call(g.net.tick)
        time.sleep(3.0)
        polls_sl = g.ais_online.polls - polls0
        state_sl = g.ais_online.state
        gw.call(g.net.request, "AUTO", None, "test", True, True, False, True)
        gw.call(g.net.tick)
        back = wait_for(lambda: g.ais_online.polls > polls0 + polls_sl, 6)
        report("AIS4 online AIS feed: merged by MMSI (local data wins when newer); never used on STARLINK",
               on_only is not None and on_only.src == "online" and loc is not None and loc.src == "local" and
               abs(loc.lat - 37.83) < 0.01 and polls0 > 0 and polls_sl == 0 and "STARLINK" in state_sl and back,
               "online-only ship: %s (%s); our own receiver's copy of 239200001 kept (%s, %.4f N) against an older "
               "online one; polls on 4G %d, on STARLINK %d (%s); back on 4G: polling again %s" % (
                   on_only and on_only.name, on_only and on_only.src, loc and loc.src, loc.lat if loc else 0,
                   polls0, polls_sl, state_sl, back))

        # AIS5 second GPS from the AIS device (AIVDO / RMC) -> forwarded to the AURIX only when switched on
        s0 = g.gps2_sent
        for k in range(3):
            asim.send_raw("$GPRMC,120000.00,A,3750.000,N,02337.000,E,6.2,045.0,211026,,,A*%s" % aismod.nmea_checksum(
                "GPRMC,120000.00,A,3750.000,N,02337.000,E,6.2,045.0,211026,,,A"))
            time.sleep(0.4)
        sent_on = g.gps2_sent - s0
        g2 = g.ais.gps2
        g.cfg["ais"]["second_gps"] = False
        s1 = g.gps2_sent
        asim.send_raw("$GPRMC,120001.00,A,3750.000,N,02337.000,E,6.2,045.0,211026,,,A*%s" % aismod.nmea_checksum(
            "GPRMC,120001.00,A,3750.000,N,02337.000,E,6.2,045.0,211026,,,A"))
        time.sleep(1.5)
        report("AIS5 GNSS of the AIS device = second GPS for the AURIX check (only when switched on)",
               sent_on >= 2 and g2 and abs(g2["lat"] - (37 + 50 / 60.0)) < 1e-6 and g.gps2_sent == s1,
               "RMC from the AIS device: %.5f N %.5f E -> %d second-GPS frames to the AURIX; switched off -> %d" % (
                   g2["lat"] if g2 else 0, g2["lon"] if g2 else 0, sent_on, g.gps2_sent - s1))

        # AIS6 receiver silent while at sea -> AIS RECEIVER SILENT on the boat (alarm only)
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 4)
        p0 = w.telem["power_fwd"] if w.telem else None
        asim.silent = True
        t0 = time.time()
        al = w.until(lambda d: d["alarm_bits"] & 0x400000, 15)
        dt = time.time() - t0
        flag_on = g.ais_silent
        asim.silent = False
        cleared = wait_for(lambda: not g.ais_silent, 15)
        report("AIS6 AIS RECEIVER SILENT at sea: alarm on the boat after the set time, no mode / power change; clears",
               al is not None and 7.0 <= dt <= 14.0 and al["mode"] == proto.MODE_HDG_LOCK and
               al["power_fwd"] == p0 and flag_on and cleared,
               "silent for %.1f s (limit 8 s) -> alarm %s, mode %s, power limit %s -> %s; messages again -> cleared %s" % (
                   dt, "yes" if al else "NO", al and al["mode_name"], p0, al and al["power_fwd"], cleared))
        link.stop()
    finally:
        for x in (asim,):
            if x:
                x.stop()
        if http_srv:
            http_srv.shutdown()
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section SH --

import chart as chartmod  # noqa: E402
import program as progmod  # noqa: E402

SIM_LAT, SIM_LON = 60.0, 25.0                   # the simulator's origin (sim/sim_world.c)
SHOAL = (60.0, 25.0 + 2000.0 / (111320.0 * math.cos(math.radians(60.0))))   # 2 km east of the start


def sim_chart(folder, shoal_m=1.5, radius_m=250.0, n_lat=160, n_lon=220):
    """20 m water around the simulator origin, a round shoal 2 km east of the boat."""
    dlat, dlon = 0.0005, 0.001
    lat0, lon0 = 59.97, 24.94
    rows = []
    for i in range(n_lat):
        r = []
        for j in range(n_lon):
            la, lo = lat0 + i * dlat, lon0 + j * dlon
            d = chartmod.dist_m(la, lo, SHOAL[0], SHOAL[1])
            r.append(shoal_m if d < radius_m else 20.0)
        rows.append(r)
    chartmod.write_area(os.path.join(folder, "simarea"), "simarea", lat0, lon0, dlat, dlon, rows, ["synthetic"],
                        "Synthetic test chart. NOT FOR NAVIGATION")


def east_of(m, north_m=0.0):
    return (SIM_LAT + north_m / 111320.0, SIM_LON + m / (111320.0 * math.cos(math.radians(SIM_LAT))))


def upload(client, wps, kn=10.0):
    prog = {"final": 0, "wps": [{"lat": a, "lon": b, "kn": kn, "radius": 30, "action": 0} for a, b in wps]}
    frames = progmod.upload_frames(prog)
    for t, p in frames[:-1]:
        client.l.send(t, p)
    client.pump(0.3)
    a = client.cmd(frames[-1][0], frames[-1][1])
    return a


def section_sh():
    print("\n=== Section C: shallow-water checks ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_sh_")
    sim = gw = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts)
        cfg = gw_cfg_b(sim, root, charts_dir=charts, stations={"driver_timeout_s": 5.0},
                       shallow={"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        bob = Client(sp, "bob", PW["bob"], name="driver screen")
        beat = DriverBeat(bob)

        # SH4 MANUAL heading for the shoal: warning on both screens only - no alarm, no speed cap
        g.shallow.cfg["look_s"] = 900.0              # look far ahead (the boat is slow at the start)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.4))
        w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 4)
        warn4 = wait_for(lambda: g.nav_state.get("shallow") is not None, 30)
        w.pump(2.0)
        d4 = w.telem
        sh4 = g.nav_state.get("shallow")
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 6)
        g.shallow.cfg["look_s"] = 30.0
        report("SH4 MANUAL heading for shallow water: warning on both screens only (no alarm, no speed change)",
               warn4 and d4["mode"] == proto.MODE_MANUAL and not d4.get("nav_applied") and
               not (d4["alarm_bits"] & 0x2000000),
               "warning %s (%s m water %.0f s ahead); mode %s, speed cap applied %s, SHALLOW alarm %s" % (
                   warn4, sh4 and sh4["depth"], sh4["in_s"] if sh4 else -1, d4["mode_name"], d4.get("nav_applied"),
                   bool(d4["alarm_bits"] & 0x2000000)))

        # SH1 program across a 1.5 m shoal refused (leg + depth + position); around it accepted
        a_bad = upload(alice, [east_of(500), east_of(4000)])
        a_near = upload(alice, [east_of(500, 0), east_of(4000, 290)])           # passes 40 m from the shoal edge
        detour = [east_of(1000, 700), east_of(3000, 700)]
        a_ok = upload(alice, detour)
        report("SH1 AUTO program across water shallower than the limit (or closer than 50 m) refused with the position",
               a_bad and a_bad["result"] == 114 and "1.5 m" in a_bad["text"] and "WP1 -> WP2" in a_bad["text"] and
               a_near and a_near["result"] == 114 and "closer" in a_near["text"] and a_ok and a_ok["result"] == 0,
               "through the shoal: %s \"%s\"; 40 m from it: %s \"%s\"; around it: %s" % (
                   a_bad and a_bad["result"], a_bad and a_bad["text"][:110], a_near and a_near["result"],
                   a_near and a_near["text"][27:95], a_ok and a_ok["result"]))

        # SH2 AUTO start: the leg from the boat to the waypoint is checked too
        a_one = upload(alice, [east_of(3500)])                                  # one waypoint behind the shoal
        seq = alice.l.send(proto.T_AUTO_START, b"", what="AUTO start")
        k_, ask = next_event(bob, ("driver_ask",), 2.0)
        r_start = alice.result(alice.wait_ack(seq))
        a_ok2 = upload(alice, detour)
        seq = alice.l.send(proto.T_AUTO_START, b"", what="AUTO start")
        k_, ask2 = next_event(bob, ("driver_ask",), 4.0)
        if ask2:
            bob.l.send_msg({"t": "driver_answer", "id": ask2["id"], "yes": True})
        r_start2 = alice.result(alice.wait_ack(seq))
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.2)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        auto = w.until(lambda d: d["mode"] == proto.MODE_AUTO, 4)
        report("SH2 AUTO start refused when the leg from the boat to the waypoint crosses shallow water",
               a_one and a_one["result"] == 0 and r_start == 114 and ask is None and a_ok2["result"] == 0 and
               r_start2 == 0 and auto is not None,
               "one waypoint behind the shoal: upload %s, AUTO start %s (refused before the driver is even asked: %s); "
               "detour: AUTO start %s after the driver's ACCEPT, AUTO %s" % (
                   a_one and a_one["result"], r_start, "yes" if ask is None else "NO", r_start2,
                   "running" if auto else "NOT running"))

        # SH3 AUTO: shallow water predicted ahead -> warning, slow down, SHALLOW alarm (power limit kept)
        w.until(lambda d: d.get("est_sog", 0) > 6.0, 40)
        p0 = w.telem["power_fwd"]
        g.shallow.cfg["shallow_m"] = 25.0           # e.g. a deeper draft entered: everything ahead is "too shallow"
        warn = wait_for(lambda: g.nav_state.get("shallow") is not None, 5)
        al = w.until(lambda d: d["alarm_bits"] & 0x2000000 and d.get("nav_applied"), 6)
        slow = w.until(lambda d: d.get("est_sog", 99) < 5.0, 45)
        g.shallow.cfg["shallow_m"] = 3.0
        gone = w.until(lambda d: not d.get("nav_applied"), 6)
        report("SH3 AUTO: shallow water predicted ahead -> warning on the screens, slows down, SHALLOW WATER alarm",
               warn and al is not None and slow is not None and slow["power_fwd"] == p0 and gone is not None and
               al["mode"] == proto.MODE_AUTO,
               "warning %s; alarm + speed cap %s kn applied %s; speed %.1f kn after slowing; power limit %s -> %s; "
               "clear again -> request removed %s" % (
                   warn, al and al.get("nav_cap_kn"), "yes" if al else "NO", slow["est_sog"] if slow else -1, p0,
                   slow and slow["power_fwd"], "yes" if gone else "NO"))

        beat.stop()
        for c in (alice, bob):
            c.close()
        link.stop()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section CA --

# smaller distances / times than at sea, so that an encounter plays out in a few minutes
CA_AVOID = {"warn_nm": 0.3, "warn_min": 4.0, "danger_nm": 0.2, "danger_min": 3.0, "recheck_s": 6.0, "safe_nm": 0.3,
            "clear_nm": 0.5, "horizon_min": 6.0, "data_max_age_s": 40.0, "min_kn": 3.0, "repropose_s": 10.0}
COLL_BIT, DECIS_BIT = 0x800000, 0x1000000


def crossing_ship(d, mmsi, kn, t_s, name, from_side=90.0):
    """A ship from starboard (from_side 90) that reaches the boat's position in t_s seconds
    (the boat keeping its course and speed): CPA ~ 0."""
    lat, lon, cog, sog = d["est_lat"], d["est_lon"], d["est_cog"], d["est_sog"]
    x_lat, x_lon = aismod.move(lat, lon, cog, sog, t_s)
    s_cog = (cog - from_side) % 360.0
    s_lat, s_lon = aismod.move(x_lat, x_lon, (s_cog + 180.0) % 360.0, kn, t_s)
    return Ship(mmsi, s_lat, s_lon, kn, s_cog, cls="A", name=name)


def ship_dist_nm(d, ship):
    return aismod.dist_m(d["est_lat"], d["est_lon"], ship.lat, ship.lon) / 1852.0


def avoid_log(g, since=0):
    return [e.get("text", "") for e in list(g.audit.recent)[since:] if e["kind"] == "avoid"]


def section_ca():
    print("\n=== Section D: collision avoidance (gateway + boat) ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_ca_")
    sim = gw = asim = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts, n_lat=400, n_lon=400)
        aport = free_port()
        asim = AisSim(aport, seed=5, noise_m=5.0, start_offset=0.2).start()
        cfg = gw_cfg_b(sim, root, charts_dir=charts, avoid=CA_AVOID, stations={"driver_timeout_s": 5.0},
                       ais={"enabled": True, "source": "tcp:127.0.0.1:%d" % aport, "silent_s": 600.0, "tick_s": 0.25})
        gw = GwRunner(cfg)
        g = gw.gw

        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        bob = Client(sp, "bob", PW["bob"], name="copilot")
        beat = DriverBeat(alice)                         # the captain's screen 1 (CLAUDE.md D)

        def joy(x=0.0, y=0.0, **kw):
            link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, x, y, **kw))

        def answer(client, yes):
            p = g.nav_state.get("proposal")
            client.l.send_msg({"t": "avoid_answer", "id": p and p["id"], "yes": yes})
            return p, next_event(client, ("avoid_ack",), 3.0)[1]

        # CA-M MANUAL: a ship on a collision course -> COLLISION alarm only (no request applied, nothing proposed)
        w.until(lambda d: d.get("pos_source", 0) != 0, 10)
        joy(0.5, 0.0)
        w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 4)
        joy(0.0, 0.0)
        d = w.telem
        lat, lon = aismod.move(d["est_lat"], d["est_lon"], 90.0, 0.0, 0)
        s_lat, s_lon = aismod.move(lat, lon, 90.0, 15.0, 100.0)           # 0.4 nm east, coming at us at 15 kn
        asim.add(Ship(239400001, s_lat, s_lon, 15.0, 270.0, name="FERRY WEST"))
        al = w.until(lambda d: d["alarm_bits"] & COLL_BIT, 25)
        st_m, prop_m = g.nav_state.get("state"), g.nav_state.get("proposal")
        time.sleep(1.0)
        w.pump(0.3)
        d_m = w.telem
        asim.remove(239400001)
        g.ais.targets.pop(239400001, None)
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 6)
        report("CA-M MANUAL: ship on a collision course -> COLLISION RISK alarm only (no steering, no speed request)",
               al is not None and st_m == "ALARM ONLY" and prop_m is None and d_m["mode"] == proto.MODE_MANUAL and
               not d_m.get("nav_applied"),
               "alarm %s; planner %s, proposal %s; mode %s, request applied by the boat %s" % (
                   "yes" if al else "NO", st_m, prop_m, d_m["mode_name"], d_m.get("nav_applied")))

        # CA-A AUTO: crossing ship from starboard -> slow down, route astern (starboard), alarm + SAVE, rejoin
        wp = east_of(0, 9000)                                             # due north, deep water all the way
        upload(alice, [wp], kn=10.0)
        seq = alice.l.send(proto.T_AUTO_START, b"", what="AUTO start")
        alice.wait_ack(seq)
        joy(confirm=True)
        time.sleep(0.2)
        joy()
        auto = w.until(lambda d: d["mode"] == proto.MODE_AUTO, 4)
        # (at the 30 % power limit the boat makes about 7 kn)
        steady = w.until(lambda d: d.get("est_sog", 0) > 5.5 and abs(((d["est_cog"] + 180) % 360) - 180) < 10, 120)
        d0 = w.telem
        leg = d0["est_cog"]
        n_log = len(g.audit.recent)
        save0 = d0.get("save_seq")
        ship = asim.add(crossing_ship(d0, 239400002, 12.0, 180.0, "BLUE STAR CROSSING"))
        t0 = time.time()
        slow = w.until(lambda d: d.get("nav_applied") and d.get("nav_cap_kn") is not None and
                       d.get("nav_kind") == proto.NAV_CAP, 30)
        avoid = w.until(lambda d: d.get("nav_applied") and d.get("nav_kind") == proto.NAV_HEADING, 40)
        hdg_rel = ((avoid["nav_heading"] - leg + 180) % 360 - 180) if avoid else 0
        route_st = None
        for _ in range(30):
            alice.pump(0.2)
            route_st = (alice.status or {}).get("avoid", {}).get("route")
            if route_st:
                break
        coll = w.until(lambda d: d["alarm_bits"] & COLL_BIT, 60)
        saved = w.until(lambda d: d.get("save_seq") != save0 and d.get("save_trig") == 2, 10)
        dmin, rejoin = 99.0, None
        end = time.time() + 300
        while time.time() < end:
            w.pump(0.5)
            dd = w.telem
            dmin = min(dmin, ship_dist_nm(dd, ship))
            if not dd.get("nav_applied") and g.nav_state.get("state") == "CLEAR" and time.time() - t0 > 60:
                rejoin = dd
                break
        back = None
        if rejoin:
            def xt(d):                                    # metres from the leg line (due north)
                return abs(d["est_lon"] - wp[1]) * 111320.0 * math.cos(math.radians(d["est_lat"]))
            xt0 = xt(w.telem)
            back = w.until(lambda d: xt(d) < max(60.0, 0.5 * xt0) and d["est_sog"] > 5.0, 240)
        texts = avoid_log(g, n_log)
        report("CA-A AUTO crossing ship: slow down, re-check, route astern (starboard), alarm + video SAVE, rejoin",
               auto is not None and steady is not None and slow is not None and avoid is not None and
               0 < hdg_rel <= 120 and route_st and coll is not None and saved is not None and
               saved["save_arg"] in (23, 24) and dmin >= 0.15 and rejoin is not None and back is not None and
               any("step 1" in t for t in texts) and any("step 2" in t for t in texts) and
               any("back to the track" in t or "clear" in t for t in texts),
               "step 1 speed cap %s kn after %.0f s; step 2 heading %03.0f (%+.0f deg from the leg, starboard), route "
               "on the screens %s points; COLLISION alarm %s, SAVE request %s (%s); closest %.2f nm; request ended "
               "%s, back on the leg to the waypoint %s (%.0f m off it -> %.0f m); %d decisions logged, e.g. \"%s\"" % (
                   slow and slow.get("nav_cap_kn"), (time.time() - t0) if not slow else 0,
                   avoid["nav_heading"] if avoid else -1, hdg_rel, len(route_st or []), "yes" if coll else "NO",
                   "yes" if saved else "NO", saved and proto.save_trigger_text(saved["save_trig"], saved["save_arg"]),
                   dmin, "yes" if rejoin else "NO", "yes" if back else "NO", xt0 if rejoin else -1,
                   xt(back) if back else -1, len(texts),
                   texts[1][:120] if len(texts) > 1 else texts))
        asim.remove(239400002)
        g.ais.targets.pop(239400002, None)

        # CA-L gateway lost while avoiding in AUTO is the last test (it stops the gateway); first HDG LOCK:
        # CA-H HEADING LOCK: proposal on the driver screen; REJECT -> unchanged; ACCEPT -> new heading; return
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 4)
        joy(0.0, 0.5)                                                     # the throttle stays with the driver
        w.until(lambda d: d.get("est_sog", 0) > 5.0, 60)
        d1 = w.telem
        h0 = d1["target"]
        ship = asim.add(crossing_ship(d1, 239400003, 12.0, 200.0, "FLYING CAT"))
        prop = wait_for(lambda: (g.nav_state.get("proposal") or {}).get("kind") == "avoid", 60)
        why_h = "%s: %s" % (g.avoider.state, g.avoider.why)
        p_cap, a_cap = answer(bob, True)                                  # the copilot can't answer it
        p1, a_rej = answer(alice, False)
        # (Waiter.pump returns at the first frame: wait on the clock / on the gateway's state)
        gone = wait_for(lambda: g.nav_state.get("proposal") is None, 3)
        time.sleep(5.0)                                   # inside the 10 s hold-off
        w.pump(0.3)
        h_after_rej = w.telem["target"]
        quiet = gone and g.nav_state.get("proposal") is None
        again = wait_for(lambda: (g.nav_state.get("proposal") or {}).get("kind") == "avoid" and
                         g.nav_state["proposal"]["id"] != (p1 or {}).get("id"), 25)
        p2, a_acc = answer(alice, True)
        turned = w.until(lambda d: p2 and abs((d["target"] - p2["heading"] + 180) % 360 - 180) < 1.5, 6)
        report("CA-H HEADING LOCK: proposal on the captain's screen 1; REJECT keeps the heading; ACCEPT changes it",
               prop and a_cap and not a_cap["ok"] and a_rej and a_rej["ok"] and
               abs((h_after_rej - h0 + 180) % 360 - 180) < 1.5 and quiet and again and a_acc and a_acc["ok"] and
               turned is not None and turned["mode"] == proto.MODE_HDG_LOCK,
               "planner %s; proposal \"%s\"; from the copilot: %s; REJECT -> heading %03.0f -> %03.0f, no new proposal "
               "for 10 s %s; proposed again %s; ACCEPT -> locked heading %s (proposal %s)" % (
                   why_h[:80], p1 and p1["text"], a_cap and a_cap["text"], h0, h_after_rej, quiet, again,
                   "%03.0f" % turned["target"] if turned else "NOT changed", p2 and p2["heading"]))
        ret = wait_for(lambda: (g.nav_state.get("proposal") or {}).get("kind") == "return", 240)
        p3, a_ret = answer(alice, True)
        home = w.until(lambda d: abs((d["target"] - h0 + 180) % 360 - 180) < 1.5, 6)
        report("CA-H2 HEADING LOCK: when clear, the way back is proposed too (ACCEPT -> the old heading)",
               ret and p3 and p3["heading"] == round(h0) and a_ret and a_ret["ok"] and home is not None,
               "return proposal \"%s\"; ACCEPT -> heading %s" % (
                   p3 and p3["text"], "%03.0f" % home["target"] if home else "NOT back"))
        asim.remove(239400003)
        g.ais.targets.pop(239400003, None)

        # CA-L AUTO, avoiding, gateway lost -> the boat holds its heading, slows to nav_lost_kn, DRIVER DECISION
        joy()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 6)
        seq = alice.l.send(proto.T_AUTO_START, b"", what="AUTO start")
        alice.wait_ack(seq)
        joy(confirm=True)
        time.sleep(0.2)
        joy()
        w.until(lambda d: d["mode"] == proto.MODE_AUTO, 4)
        brg_l = aismod.bearing(w.telem["est_lat"], w.telem["est_lon"], wp[0], wp[1])
        w.until(lambda d: d.get("est_sog", 0) > 5.5 and abs((d["est_cog"] - brg_l + 180) % 360 - 180) < 10, 120)
        asim.add(crossing_ship(w.telem, 239400004, 16.0, 120.0, "LATE FERRY"))
        steer = w.until(lambda d: d.get("nav_applied") and d.get("nav_kind") in (proto.NAV_CAP, proto.NAV_HEADING),
                        40)
        h_l = w.telem["target"]
        t_l = time.time()
        gw.stop()
        gw = None
        lost = w.until(lambda d: d.get("nav_lost") and d["alarm_bits"] & DECIS_BIT, 8)
        dt_l = time.time() - t_l
        slow_l = w.until(lambda d: d.get("est_sog", 99) < 5.0, 60)
        report("CA-L gateway lost during an avoidance in AUTO -> the boat holds its heading, slows, DRIVER DECISION",
               steer is not None and lost is not None and dt_l < 5.0 and lost["mode"] == proto.MODE_AUTO and
               slow_l is not None and abs((slow_l["target"] - lost["target"] + 180) % 360 - 180) < 2.0,
               "request applied %s; gateway stopped -> lease lost + alarm after %.1f s (lease 3 s), mode %s; "
               "speed %.1f kn, held heading %03.0f (at the loss %03.0f; requested before %03.0f)" % (
                   "yes" if steer else "NO", dt_l, lost and lost["mode_name"],
                   slow_l["est_sog"] if slow_l else -1, slow_l["target"] if slow_l else -1,
                   lost["target"] if lost else -1, h_l))

        beat.stop()
        for c in (alice, bob):
            c.close()
        link.stop()
    finally:
        if asim:
            asim.stop()
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)

# ------------------------------------------------------------------ section VI --

OBST_BIT = 0x4000000                            # OBSTACLE AHEAD (app/smallboat.h SB_ALM_OBSTACLE)


def section_vi():
    print("\n=== Part 2: camera obstacle detection (vision stages, gateway + boat) ===", flush=True)
    # VI0 (R21): an unusable camera config is replaced per entry; an unknown distance counts as IN the path
    from vision import geometry as geo
    cam, bad = geo.check_camera({"hfov_deg": 0, "vfov_deg": "x", "height_m": 1.2})
    nan_in = geo.in_corridor(0.0, float("nan"), 5.0, 0.0, {})
    loc = geo.locate(cam, 0.5, 0.8, geo.horizon_default(cam))
    report("VI0 camera config checked (fov 0 / not a number -> default); NaN distance = in the path, not ignored",
           cam["hfov_deg"] == 80.0 and cam["vfov_deg"] == 45.0 and cam["height_m"] == 1.2 and len(bad) == 2 and
           nan_in and math.isfinite(loc[1]),
           "problems %s; NaN distance in the corridor %s; distance with the fixed config %.1f m" % (bad, nan_in, loc[1]))
    # VI0b the driver HUD's AR overlays get the calibration only when it is really set (and usable)
    from vision.manager import VisionManager
    good = {"height_m": 2.0, "tilt_down_deg": 2.0, "hfov_deg": 75.0, "vfov_deg": 44.0}
    c_none = VisionManager({"enabled": False}, None).status()["camera"]
    c_set = VisionManager({"enabled": False, "camera": good}, None).status()["camera"]
    c_bad = VisionManager({"enabled": False, "camera": dict(good, hfov_deg=0)}, None).status()["camera"]
    report("VI0b vision status: the camera calibration for the HUD's AR overlays only when configured and usable",
           c_none is None and c_set == dict(good, yaw_deg=0.0) and c_bad is None,
           "not configured -> %s; configured -> %s; field of view 0 -> %s" % (c_none, c_set, c_bad))
    tmp = tempfile.mkdtemp(prefix="gwtest_vi_")
    sim = gw = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts, n_lat=400, n_lon=400)
        cfg = gw_cfg_b(sim, root, charts_dir=charts, stations={"driver_timeout_s": 5.0},
                       vision={"enabled": True, "source": "fake", "allow_inject": True, "max_fps": 4.0,
                               "stage1": True, "stage2": False, "stage3": False, "clear_s": 4.0})
        gw = GwRunner(cfg)
        g = gw.gw
        v = g.vision
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        bob = Client(sp, "bob", PW["bob"], name="driver screen")
        beat = DriverBeat(bob)

        def joy(x=0.0, y=0.0, **kw):
            link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, x, y, **kw))

        def navs_since(n0):
            return [e for e in list(g.audit.recent)[n0:] if e["kind"] == "navreq"]

        def stage(client, name, on, confirm=None):
            msg = {"t": "vision_stage", "stage": name, "on": on}
            if confirm:
                msg["confirm"] = confirm
            client.l.send_msg(msg)

        def ack(client, timeout=8.0):
            return next_event(client, ("vision_ack",), timeout)[1]

        # MANUAL, moving slowly ahead (the corridor needs a direction)
        w.until(lambda d: d.get("pos_source", 0) != 0, 10)
        joy(0.0, 0.3)
        w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 4)
        w.until(lambda d: d.get("est_sog", 0) > 2.0, 30)

        # VI1 STAGE 1 only: overlay + log, nothing reaches the boat
        n0 = len(g.audit.recent)
        thr0 = w.telem["throttle"]
        v.inject(0.0, 90.0, seconds=4.0)
        conf1 = wait_for(lambda: any(o["confirmed"] and o["in_corridor"] for o in v.obstacles), 4)
        time.sleep(1.0)
        st1 = g.status()["vision"]
        d1 = w.telem
        alarm1 = any(e.get("alarm", 0) & 8 for e in navs_since(n0))
        dets = glob.glob(os.path.join(root, "vision", "detections-*.jsonl"))
        report("VI1 STAGE 1 (overlay + logging only): obstacle shown and logged, no alarm, no effect on the boat",
               conf1 and st1["obstacles"] and not alarm1 and not (d1["alarm_bits"] & OBST_BIT) and
               abs(d1["throttle"] - thr0) < 2.0 and d1["mode"] == proto.MODE_MANUAL and bool(dets),
               "confirmed in the corridor %s (%s m, bearing %s); request alarm %s; boat alarm %s; throttle %.1f -> %.1f %%;"
               " detection log %s" % (conf1, st1["obstacles"][0]["dist"] if st1["obstacles"] else "-",
                                     st1["obstacles"][0]["bearing"] if st1["obstacles"] else "-", alarm1,
                                     bool(d1["alarm_bits"] & OBST_BIT), thr0, d1["throttle"],
                                     os.path.basename(dets[0]) if dets else "MISSING"))
        wait_for(lambda: not v.obstacles, 6)

        # VI2 persistence: a one-frame blip (wake, foam, glare) never counts
        c0 = v.counts["confirmed"]
        stage(alice, "stage2", True)
        a2 = ack(alice)
        n0 = len(g.audit.recent)
        for _ in range(3):
            v.inject(-10.0, 60.0, seconds=5.0, frames=1)        # seen in ONE frame only
            time.sleep(1.5)
        time.sleep(1.0)
        report("VI2 persistence filter: single-frame blips never become obstacles (no alarm)",
               a2 and a2["ok"] and v.counts["confirmed"] == c0 and
               not any(e.get("alarm", 0) & 8 for e in navs_since(n0)) and not (w.telem["alarm_bits"] & OBST_BIT),
               "3 blips -> confirmed %d (before %d), candidates seen %d, alarm %s" % (
                   v.counts["confirmed"], c0, v.counts["candidates"], bool(w.telem["alarm_bits"] & OBST_BIT)))

        # VI3 STAGE 2: OBSTACLE AHEAD on the boat (+ the AURIX's SAVE request); throttle / mode unchanged
        save0 = w.telem.get("save_seq")
        thr0 = w.telem["throttle"]
        v.inject(5.0, 70.0, seconds=6.0)
        al = w.until(lambda d: d["alarm_bits"] & OBST_BIT, 8)
        saved = w.until(lambda d: d.get("save_seq") != save0 and d.get("save_trig") == 2, 6)
        d3 = w.telem
        report("VI3 STAGE 2: alarm OBSTACLE AHEAD on the boat + automatic video SAVE; no effect on the driving",
               al is not None and saved is not None and saved["save_arg"] == 26 and
               d3["mode"] == proto.MODE_MANUAL and abs(d3["throttle"] - thr0) < 2.0 and not d3.get("nav_applied"),
               "alarm %s; SAVE request %s (%s); mode %s, throttle %.1f -> %.1f %%, request applied %s" % (
                   "yes" if al else "NO", "yes" if saved else "NO",
                   saved and proto.save_trigger_text(saved["save_trig"], saved["save_arg"]), d3["mode_name"], thr0,
                   d3["throttle"], d3.get("nav_applied")))
        wait_for(lambda: not v.threats, 10)
        alice.cmd(proto.T_ALARM_ACK)

        # VI4 STAGE 3 ON (affects the driving): the captain's own decision; the copilot's needs the captain's
        # ACCEPT on screen 1 (no screen 1 / REJECT -> refused) - CLAUDE.md D
        beat.stop()
        time.sleep(3.0)                                    # no screen 1 anywhere (no heartbeat)
        stage(bob, "stage3", True)                          # bob = the copilot
        a_none = ack(bob)
        beat = DriverBeat(alice)                            # the captain's screen 1
        time.sleep(1.0)
        stage(bob, "stage3", True)
        k_, q = next_event(alice, ("driver_ask",), 4.0)
        if q:
            alice.l.send_msg({"t": "driver_answer", "id": q["id"], "yes": False})
        a_rej = ack(bob)
        on_after_rej = v.cfg["stage3"]
        stage(bob, "stage3", True)
        k_, q2 = next_event(alice, ("driver_ask",), 4.0)
        if q2:
            alice.l.send_msg({"t": "driver_answer", "id": q2["id"], "yes": True})
        a_acc = ack(bob)
        on_cop = v.cfg["stage3"]
        stage(alice, "stage3", False)
        ack(alice)
        stage(alice, "stage3", True)                        # the captain: straight through, no question
        a_cap = ack(alice)
        logged = [e for e in g.audit.recent if e["kind"] == "vision_stage" and e.get("stage") == "stage3"]
        report("VI4 STAGE 3 (affects the driving): the copilot only with the captain's ACCEPT (screen 1), the captain "
               "directly; logged",
               a_none and not a_none["ok"] and "driver screen not open" in a_none["text"] and q and
               a_rej and not a_rej["ok"] and not on_after_rej and q2 and a_acc and a_acc["ok"] and on_cop and
               a_cap and a_cap["ok"] and v.cfg["stage3"] and logged,
               "copilot, no screen 1: \"%s\"; REJECT: \"%s\" (stage 3 %s); ACCEPT: \"%s\"; the captain itself: "
               "\"%s\"; audit %d entries" % (
                   a_none and a_none["text"], a_rej and a_rej["text"], on_after_rej, a_acc and a_acc["text"],
                   a_cap and a_cap["text"], len(logged)))

        # VI5 MANUAL with STAGE 3: alarm only - throttle unchanged, no cap applied
        thr0 = w.telem["throttle"]
        n0 = len(g.audit.recent)
        v.inject(0.0, 30.0, seconds=4.0)
        al5 = w.until(lambda d: d["alarm_bits"] & OBST_BIT, 8)
        time.sleep(1.0)
        d5 = w.telem
        report("VI5 STAGE 3 in MANUAL: alarm only (no speed cap applied, throttle unchanged)",
               al5 is not None and not d5.get("nav_applied") and abs(d5["throttle"] - thr0) < 2.0 and
               d5["mode"] == proto.MODE_MANUAL,
               "alarm %s; cap applied by the boat %s; throttle %.1f -> %.1f %%; mode %s" % (
                   "yes" if al5 else "NO", d5.get("nav_applied"), thr0, d5["throttle"], d5["mode_name"]))
        wait_for(lambda: not v.threats, 10)
        joy()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 8)
        alice.cmd(proto.T_ALARM_ACK)

        # VI6 HEADING LOCK with STAGE 3: a slow-down PROPOSAL for the driver; the throttle stays manual
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 4)
        joy(0.0, 0.4)
        w.until(lambda d: d.get("est_sog", 0) > 3.0, 40)
        thr0 = w.telem["throttle"]
        v.inject(0.0, 60.0, seconds=5.0)
        prop = wait_for(lambda: v.proposal is not None, 6)
        p6 = v.proposal
        alice.l.send_msg({"t": "vision_ack", "id": p6 and p6["id"]})            # the captain's screen 1
        time.sleep(1.0)
        d6 = w.telem
        report("VI6 STAGE 3 in HEADING LOCK: slow-down proposal on the driver screen, throttle unchanged",
               prop and p6 and "slow down" in p6["text"] and (v.proposal or {}).get("acked") and
               not d6.get("nav_applied") and abs(d6["throttle"] - thr0) < 2.0 and d6["mode"] == proto.MODE_HDG_LOCK,
               "proposal \"%s\", acknowledged %s; cap applied %s; throttle %.1f -> %.1f %%" % (
                   p6 and p6["text"], (v.proposal or {}).get("acked"), d6.get("nav_applied"), thr0, d6["throttle"]))
        wait_for(lambda: not v.threats, 10)
        joy()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 8)
        alice.cmd(proto.T_ALARM_ACK)

        # VI7 AUTO with STAGE 3: a confirmed obstacle 30 m ahead -> cap 0 = idle; only a cap, never a heading,
        # never more throttle; then vision CRASHES: no mode change, the cap is HELD until the driver clears it
        wp = east_of(0, 9000)
        upload(alice, [wp], kn=10.0)
        seq = alice.l.send(proto.T_AUTO_START, b"", what="AUTO start")      # the captain's own: direct
        alice.wait_ack(seq)
        joy(confirm=True)
        time.sleep(0.2)
        joy()
        w.until(lambda d: d["mode"] == proto.MODE_AUTO, 4)
        w.until(lambda d: d.get("est_sog", 0) > 5.5, 90)
        n0 = len(g.audit.recent)
        thr0 = w.telem["throttle"]
        v.inject(0.0, 30.0, seconds=8.0)
        cap = w.until(lambda d: d.get("nav_applied") and d.get("nav_cap_kn") == 0.0, 8)
        thr_max, t_end = 0.0, time.time() + 6.0
        while time.time() < t_end:
            w.pump(0.2)
            thr_max = max(thr_max, w.telem["throttle"])
        v.fail("crash: model process died (test)")
        time.sleep(1.5)
        held = v.cap_held and v.cap_kn == 0.0
        time.sleep(8.0)                                   # the obstacle is gone, vision is down: still held
        d7 = w.telem
        still = d7.get("nav_applied") and d7.get("nav_cap_kn") == 0.0 and d7["mode"] == proto.MODE_AUTO
        st7 = g.status()["vision"]
        bob.l.send_msg({"t": "vision_clear"})                # the copilot: refused
        a_cap = ack(bob, 3.0)
        alice.l.send_msg({"t": "vision_clear"})              # the captain at screen 1: cleared
        a_drv = ack(alice, 3.0)
        released = w.until(lambda d: not d.get("nav_applied"), 6)
        kinds = {e.get("nav_kind") for e in navs_since(n0)}
        v.fail(None)
        report("VI7 STAGE 3 in AUTO: slows to idle for a confirmed obstacle (cap only, never steering / more throttle);"
               " vision crash -> OFFLINE, no mode change, the slowdown HELD until the driver clears it",
               cap is not None and thr_max <= thr0 + 0.5 and kinds <= {proto.NAV_NONE, proto.NAV_CAP} and held and
               still and st7["state"] == "OFFLINE" and a_cap and not a_cap["ok"] and a_drv and a_drv["ok"] and
               released is not None,
               "cap 0 kn applied %s; throttle %.1f %% -> max %.1f %% after; request kinds %s (1 = cap only); crash -> "
               "%s, cap held %s, still applied after 8 s %s (mode %s); copilot clear: \"%s\"; captain (screen 1) clear: \"%s\" -> "
               "released %s" % ("yes" if cap else "NO", thr0, thr_max, sorted(kinds), st7["state"], held, still,
                                d7["mode_name"], a_cap and a_cap["text"], a_drv and a_drv["text"],
                                "yes" if released else "NO"))

        # VI8 the joystick and STANDBY still win during a vision slowdown
        v.inject(0.0, 30.0, seconds=30.0)
        w.until(lambda d: d.get("nav_applied"), 8)
        t0 = time.time()
        joy(-0.6, 0.0)
        man = w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 2)
        dt = time.time() - t0
        joy()
        time.sleep(0.5)
        t1 = time.time()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        sb = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 3)
        report("VI8 during a vision slowdown the joystick takes over at once; STANDBY wins",
               man is not None and dt < 1.0 and sb is not None,
               "joystick -> %s after %.2f s; STANDBY -> %s after %.2f s" % (
                   man and man["mode_name"], dt, sb and sb["mode_name"], time.time() - t1))

        beat.stop()
        for c in (alice, bob):
            c.close()
        link.stop()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


def section_vr():
    """The real worker (model + ffmpeg): low priority, thread / core limit, newest frame only, fps,
    crash -> restart; the gateway's main timing with and without it (THIS PC; the Pi must be measured)."""
    print("\n=== Part 2: the vision worker process (real model, CPU limits) ===", flush=True)
    import toolpaths
    model = os.path.join(HERE, "vision", "models", "ewasr_resnet18.onnx")
    py = toolpaths.vision_python()
    ok_py = py and subprocess.run([py, "-c", "import onnxruntime, numpy"], capture_output=True).returncode == 0
    if not (os.path.exists(model) and ok_py):
        print("[SKIP] VR: no model (tools/get_vision_model.py) or no onnxruntime Python (tools/get_vision_python.py)")
        return
    tmp = tempfile.mkdtemp(prefix="gwtest_vr_")
    sim = gw = None
    try:
        video = os.path.join(tmp, "sea.mp4")
        subprocess.run([toolpaths.ffmpeg(), "-loglevel", "error", "-f", "lavfi", "-i",
                        "testsrc2=size=640x360:rate=15", "-t", "20", "-pix_fmt", "yuv420p", video], check=True)
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root, vision={"enabled": False})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        alice = Client(sp, "alice", PW["alice"], name="captain")

        def timing(seconds):
            """telemetry gaps at the station + how late the gateway's loop runs."""
            alice.pump(0.5)
            n0, gaps, last = alice.n_tel, [], None
            end = time.time() + seconds
            while time.time() < end:
                try:
                    k, d = alice.l.events.get(timeout=0.2)
                except Exception:
                    continue
                if k == "telemetry":
                    now = time.time()
                    if last:
                        gaps.append(now - last)
                    last = now
                    alice.n_tel += 1
            gaps.sort()
            return {"frames": alice.n_tel - n0, "gap_max_ms": round(gaps[-1] * 1000) if gaps else None,
                    "gap_p99_ms": round(gaps[int(0.99 * (len(gaps) - 1))] * 1000) if gaps else None}
        base = timing(20)
        # (stage 2 off: the test picture can look like obstacles - here only the worker mechanics count)
        g.vision.cfg.update({"enabled": True, "source": "camera", "threads": 2, "max_fps": 2.0, "min_fps": 1.0,
                             "stage2": False,
                             "warmup_s": 5.0})
        g.vision._front_url = lambda: video               # the test video instead of the front camera
        up = wait_for(lambda: g.vision.hello.get("pid") and g.vision.frames >= 3, 40)
        pid = g.vision.hello.get("pid")
        prio, aff = None, None
        if os.name == "nt" and pid:
            import ctypes
            k = ctypes.windll.kernel32
            h = k.OpenProcess(0x1000, False, pid)            # PROCESS_QUERY_LIMITED_INFORMATION
            prio = k.GetPriorityClass(h)
            pm, sm = ctypes.c_size_t(), ctypes.c_size_t()
            k.GetProcessAffinityMask(h, ctypes.byref(pm), ctypes.byref(sm))
            aff = bin(pm.value).count("1")
            k.CloseHandle(h)
        elif pid:
            prio = os.getpriority(os.PRIO_PROCESS, pid)
            aff = len(os.sched_getaffinity(pid))
        busy = timing(20)
        st = g.vision.status()
        report("VR1 the vision worker runs in its own process at the lowest priority on 2 cores / 2 threads",
               up and pid and pid != os.getpid() and (prio in (0x40,) if os.name == "nt" else prio >= 19) and
               aff == 2 and st["threads"] == 2,
               "worker pid %s (gateway %d), priority %s (%s), cores %s, threads %d, %s fps, inference %s ms, "
               "state %s" % (pid, os.getpid(), hex(prio) if os.name == "nt" and prio else prio,
                             "IDLE" if prio == 0x40 else "?", aff, st["threads"], st["fps"], st["infer_ms"],
                             st["state"]))
        report("VR2 the gateway's main job timing is unchanged with vision running (measured on THIS PC; "
               "the Pi must be measured on the Pi)",
               busy["frames"] >= 0.9 * base["frames"] and (busy["gap_max_ms"] or 0) <= max(250, 1.5 * (
                   base["gap_max_ms"] or 0)),
               "without vision: %s; with vision: %s" % (base, busy))
        n_fr = g.vision.frames
        time.sleep(3.0)
        rate = (g.vision.frames - n_fr) / 3.0
        report("VR3 newest frame only: the worker never falls behind the camera (frame age small, fps <= max)",
               rate <= 2.6 and st["state"] in ("ACTIVE", "DEGRADED"),
               "%.1f results / s (max 2), the camera sends 15 fps (the rest is skipped, never queued)" % rate)
        # VR5 samples (frame + mask) for a later own dataset: saved on request, inside the storage quota
        g.vision.cfg["samples"].update({"enabled": True, "max_mb": 1.0})
        folder = os.path.join(root, "vision", "samples")
        for _ in range(4):
            g.vision._sample("random")
            time.sleep(1.2)
        files = sorted(os.listdir(folder)) if os.path.isdir(folder) else []
        size = sum(os.path.getsize(os.path.join(folder, f)) for f in files)
        g.vision._sample("random")                           # the quota check runs before every new sample
        time.sleep(1.2)
        size2 = sum(os.path.getsize(os.path.join(folder, f)) for f in os.listdir(folder))
        report("VR5 samples (frame + mask) saved for an own dataset, the oldest deleted above the quota",
               any(f.endswith(".ppm") for f in files) and any(f.endswith("_mask.pgm") for f in files) and
               size2 <= 1.0e6 + 600e3,
               "%d files (%.0f kB) after 4 samples; quota 1 MB -> %.0f kB after the next one" % (
                   len(files), size / 1e3, size2 / 1e3))
        g.vision.kill_worker()
        off = wait_for(lambda: g.vision.state == "OFFLINE", 5)
        back = wait_for(lambda: g.vision.state in ("ACTIVE", "DEGRADED") and g.vision.restarts >= 2, 40)
        report("VR4 worker crash -> VISION OFFLINE notice (no alarm, no mode change), restarted by itself",
               off and back and g.telem["mode"] == proto.MODE_STANDBY and not (g.telem["alarm_bits"] & 0x4000000),
               "OFFLINE %s, back %s (restarts %d), mode %s, notices %s" % (
                   off, back, g.vision.restarts, g.telem["mode_name"], [n["text"] for n in g.vision.notices]))
        alice.close()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


def section_cr():
    """Crew (CLAUDE.md sections A-D): manual licence, MANEUVER, UNCREWED shore CONFIRM - end to end."""
    print("\n=== Section CR: crew - licence, MANEUVER, UNCREWED profile ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_cr_")
    try:
        # --- CREWED boat: licence refusals, MANEUVER with the onboard CONFIRM ---
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        users = users_cfg()
        users["carl"] = {"role": "HELM", "pw": hash_pw("carl-password-33")}           # no manual licence
        cfg = gw_cfg_b(sim, root, users=users)
        gw = GwRunner(cfg)
        sp = cfg["stations"]["port"]
        link, w = sim.station()                          # the laptop on board (USB): CONFIRM, observing
        carl = Client(sp, "carl", "carl-password-33", name="carl")
        carl.pump(1.5)
        r_helm_c = carl.result(carl.cmd(proto.T_HELM, proto.p_helm(0)))
        r_rem_c = carl.result(carl.cmd(proto.T_REMOTE, proto.p_remote(0)))
        r_man_c = carl.result(carl.cmd(proto.T_MANEUVER, proto.p_remote(0)))
        r_rel_c = carl.result(carl.cmd(proto.T_MANEUVER, proto.p_remote(1)))       # a release is always allowed
        carl.result(carl.cmd(proto.T_HELM, proto.p_helm(1)))
        carl.close()
        report("CR1 (I26) REMOTE MANUAL / MANEUVER refused for a user without a manual licence",
               r_helm_c == 117 and r_rem_c == 116 and r_man_c == 116 and r_rel_c in (0, 119),
               "carl (HELM user, no licence = copilot): HELM %s (117), REMOTE %s, MANEUVER %s (116 = no manual "
               "licence), release %s (never refused for the licence)" % (
                   r_helm_c, r_rem_c, r_man_c, r_rel_c))

        alice = Client(sp, "alice", PW["alice"], name="alice")
        alice.pump(1.0)
        stick = {"x": 0.0, "lever": 0.5}
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        r_helm = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        r_ask = alice.result(alice.cmd(proto.T_MANEUVER, proto.p_remote(0)))
        time.sleep(0.5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))   # the person on board
        time.sleep(0.2)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        on = w.until(lambda d: d["mode"] == proto.MODE_MANEUVER, 3)
        stick["lever"] = 1.0
        top, gear_f = 0.0, False
        t0 = time.time()
        while time.time() - t0 < 4:
            w.pump(0.1)
            if w.telem:
                top = max(top, w.telem["throttle"])
                gear_f = gear_f or w.telem["gear"] == 1
        prof = w.telem.get("profile_name") if w.telem else None
        report("CR2 MANEUVER through the gateway: licensed captain asks, CONFIRM on board; full lever = forward gear, "
               "throttle idle",
               r_helm == 0 and r_ask == 0 and on is not None and gear_f and top <= 0.01 and prof == "CREWED",
               "alice: HELM %s, MANEUVER %s, after CONFIRM: %s; full lever: gear F %s, highest throttle %.2f %%; "
               "profile %s" % (r_helm, r_ask, on and on["mode_name"], gear_f, top, prof))
        snd.stop()
        alice.close()
        link.stop()
        gw.stop()
        sim.stop()

        # --- UNCREWED boat (no onboard joystick at power-up): CONFIRM from the captain's joystick ---
        sim = Sim(1.0, "nojoy")
        link, w = sim.station()
        root2 = os.path.join(tmp, "ssd2")
        os.makedirs(root2)
        cfg = gw_cfg_b(sim, root2)
        gw = GwRunner(cfg)
        sp = cfg["stations"]["port"]
        unc = w.until(lambda d: d.get("profile") == proto.PROFILE_UNCREWED, 12)
        alice = Client(sp, "alice", PW["alice"], name="alice")
        alice.pump(1.0)
        stick = {"x": 0.0, "lever": 0.5, "confirm": False}
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        time.sleep(0.5)
        alice.result(alice.cmd(proto.T_ALARM_ACK))
        r_ask = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(0)))
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))   # onboard: ignored
        time.sleep(0.3)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(False, 0.0, 0.0))
        w.pump(0.5)
        ignored = w.telem and w.telem["mode"] == proto.MODE_STANDBY
        stick["confirm"] = True                          # the captain's CONFIRM button
        time.sleep(0.3)
        stick["confirm"] = False
        on = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 3)
        report("CR3 UNCREWED: the onboard joystick is ignored, the shore captain's CONFIRM (through the gateway) "
               "starts REMOTE",
               unc is not None and r_ask == 0 and ignored and on is not None,
               "profile %s; REMOTE asked %s; onboard CONFIRM: still STANDBY %s; shore CONFIRM -> %s" % (
                   unc and unc.get("profile_name"), r_ask, ignored, on and on["mode_name"]))
        # CR5 the same boat: the captain's station disappears near the shore (no charts on this gateway =
        # no valid shore distance = near) -> STANDBY, idle + neutral, ENGINE KILLED, latched
        snd.stop()
        alice.close()                                   # TCP closed: the gateway drops it at once
        lost = w.until(lambda d: d.get("ll_state") in (1, 2) and d["mode"] == proto.MODE_STANDBY, 8)
        killed = w.until(lambda d: d.get("ll_state") == 2, 10)
        alice = Client(sp, "alice", PW["alice"], name="alice")
        alice.pump(1.0)
        alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        w.pump(3.0)
        still = w.telem and w.telem.get("ll_state") == 2 and w.telem["throttle"] <= 0.6
        alarms = proto.alarm_texts(w.telem["alarm_bits"]) if w.telem else []
        report("CR5 UNCREWED captain's station lost near the shore (no shore info): STANDBY, ENGINE KILLED, latched",
               lost is not None and killed is not None and still and "ENGINE KILLED" in alarms,
               "LINK LOST + STANDBY %s; ENGINE KILLED %s; captain back with HELM: still killed %s; alarms %s" % (
                   lost is not None, killed is not None, still, ", ".join(alarms)))
        alice.close()
        link.stop()
        gw.stop()
        sim.stop()

        # --- crew roles on an UNCREWED boat (no onboard joystick): captain / copilot, swap, boat name ---
        sim = Sim(1.0, "nojoy")
        link, w = sim.station()
        root4 = os.path.join(tmp, "ssd4")
        os.makedirs(root4)
        users = users_cfg()
        users["carl"] = {"role": "HELM", "pw": hash_pw("carl-password-33")}           # no manual licence
        cfg = gw_cfg_b(sim, root4, users=users)
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        w.until(lambda d: d.get("profile") == proto.PROFILE_UNCREWED, 12)
        alice = Client(sp, "alice", PW["alice"], name="alice")
        bob = Client(sp, "bob", PW["bob"], name="bob", crew="captain")             # asks, but alice is the captain
        carl = Client(sp, "carl", "carl-password-33", name="carl", crew="captain")  # asks, without a licence
        vic = Client(sp, "vic", PW["vic"], name="vic")
        crews = {s.name: s.crew for s in g.stations.stations.values()}
        notes = (bob.welcome.get("crew_note"), carl.welcome.get("crew_note"))
        alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        r_bob_rem = bob.result(bob.cmd(proto.T_REMOTE, proto.p_remote(0)))
        r_carl_helm = carl.result(carl.cmd(proto.T_HELM, proto.p_helm(0)))
        report("CR6 crew at login: the first licensed user = captain; a second captain, or one without a licence -> "
               "copilot; a copilot can't take HELM or ask for manual driving",
               crews == {"alice": "captain", "bob": "copilot", "carl": "copilot", "vic": None} and all(notes) and
               r_bob_rem == 118 and r_carl_helm == 117,
               "crews %s; notes %s; copilot bob asks REMOTE %s (118 = captain only); copilot carl HELM %s (117)" % (
                   crews, notes, r_bob_rem, r_carl_helm))

        # CR7 role swap (both licensed): HELM moves to the new captain in one step - no LINK LOST (UNCREWED);
        # with an unlicensed partner refused
        alice.pump(1.0)
        bob.l.send_msg({"t": "crew_swap"})
        k, ask = next_event(alice, ("crew_swap_ask",))
        lost_seen = []
        alice.l.send_msg({"t": "crew_swap_answer", "yes": True})
        k, done = next_event(bob, ("crew_swap_done",))
        t0 = time.time()
        while time.time() - t0 < 3:
            w.pump(0.1)
            if w.telem and w.telem.get("ll_state"):
                lost_seen.append(w.telem["ll_state"])
        helm = w.telem.get("helm") if w.telem else None
        crews2 = {s.name: s.crew for s in g.stations.stations.values()}
        bob_id = bob.welcome["id"]
        bob.l.send_msg({"t": "crew_swap", "with": carl.welcome["id"]})
        k, refused = next_event(bob, ("crew_swap_done",))
        r_alice_rem = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(0)))
        report("CR7 role swap captain <-> copilot (both licensed): HELM handed over in one step, no LINK LOST; "
               "refused when one has no licence",
               ask is not None and done and done.get("ok") and helm == bob_id and not lost_seen and
               crews2.get("bob") == "captain" and crews2.get("alice") == "copilot" and refused and
               not refused.get("ok") and r_alice_rem == 118,
               "bob asks, alice accepts: %s; AURIX HELM -> %s (bob %s); lost-link states seen %s; crews %s; swap with "
               "carl (no licence): %s; alice (now copilot) asks REMOTE %s" % (
                   done, helm, bob_id, lost_seen or "none", crews2, refused and refused.get("text"), r_alice_rem))
        for c in (alice, bob, carl, vic):
            c.close()

        # CR8 the boat name: a station that expects another boat is refused (gateway and station both check)
        boat = g.cfg.get("boat")
        err = None
        try:
            Client(sp, "alice", PW["alice"], name="alice", boat="er69d-not-this-one")
        except ConnectionError as e:
            err = str(e)
        good = Client(sp, "alice", PW["alice"], name="alice", boat=boat)
        ok_boat = good.welcome is not None and good.welcome.get("boat") == boat
        good.close()
        report("CR8 boat name check: the gateway of another boat refuses the login; the right one connects",
               err is not None and "wrong boat" in err and ok_boat,
               "this gateway is %r; asked for another boat -> %r; the right name -> connected %s" % (boat, err, ok_boat))

        # CR9 a station on the boat takes HELM from a remote captain: it becomes the captain (on board wins)
        cap = Client(sp, "alice", PW["alice"], name="shore")
        cap.result(cap.cmd(proto.T_HELM, proto.p_helm(0)))
        onb = Client(sp, "bob", PW["bob"], name="on board", local=True)
        r_take = onb.result(onb.cmd(proto.T_HELM, proto.p_helm(0)))
        onb.pump(1.0)
        crews3 = {s.name: s.crew for s in g.stations.stations.values()}
        report("CR9 a station on the boat can always take HELM and then is the captain; the shore one -> copilot",
               r_take == 0 and crews3.get("on board") == "captain" and crews3.get("shore") == "copilot",
               "on board takes HELM %s; crews %s" % (r_take, crews3))
        cap.close()
        onb.close()
        link.stop()
        gw.stop()
        sim.stop()

        # CR4 the shore distance from the charts: gateway -> GW_SHORE -> AURIX telemetry
        import shallow as shmod
        charts = os.path.join(tmp, "charts")
        sim_chart(charts)
        ch = chartmod.Charts(charts)
        sh = shmod.Shallow({}, ch)
        f_boat, d_boat = sh.shore({"lat": SIM_LAT, "lon": SIM_LON})
        la, lo = east_of(2000.0 - 250.0 - 60.0)                   # 60 m from the edge of the shoal
        f_near, d_near = sh.shore({"lat": la, "lon": lo})
        f_none, d_none = shmod.Shallow({}, chartmod.Charts(os.path.join(tmp, "nothing"))).shore(
            {"lat": SIM_LAT, "lon": SIM_LON})
        sim = Sim(1.0)
        link, w = sim.station()
        root3 = os.path.join(tmp, "ssd3")
        os.makedirs(root3)
        cfg = gw_cfg_b(sim, root3, charts_dir=charts)
        gw = GwRunner(cfg)
        got = w.until(lambda d: d.get("coast_valid") and d.get("coast_dist_m") is not None, 10)
        report("CR4 shore distance from the charts (same data as the route check) reaches the AURIX once a second",
               f_boat == 2 and 1500 <= d_boat <= 1900 and f_near == 3 and 20 <= d_near <= 100 and
               (f_none, d_none) == (0, 0xFFFF) and got is not None and abs(got["coast_dist_m"] - d_boat) <= 60 and
               not got["near_shore"],
               "boat: flags %d, %d m from the shoal; 60 m from its edge: flags %d (near), %d m; no charts: flags %d, "
               "%d (= not valid -> near on the AURIX); AURIX telemetry: %s m, near %s" % (
                   f_boat, d_boat, f_near, d_near, f_none, d_none, got and got["coast_dist_m"],
                   got and got["near_shore"]))
        link.stop()
        gw.stop()
        sim.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


SECTIONS = {"F1": section_f1, "CR": section_cr, "AIS": section_ais, "SH": section_sh, "CA": section_ca, "VI": section_vi, "VR": section_vr, "A": section_a, "B": section_b, "C": section_c, "D": section_d, "E": section_e, "F": section_f,
            "G": section_g}


def main():
    setup_logging(30)                       # warnings only
    if not os.path.exists(EXE):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    want = [a.upper() for a in sys.argv[1:]] or list(SECTIONS)
    if len(want) == 1:
        SECTIONS[want[0]]()
    else:
        # every section in its own process: timing tests must not feel the
        # threads / proxies of the sections before them
        for k in want:
            r = subprocess.run([sys.executable, os.path.abspath(__file__), k], capture_output=True, text=True)
            for line in r.stdout.splitlines():
                if line.startswith("[PASS] ") or line.startswith("[FAIL] "):
                    print(line, flush=True)
                    results.append((line[7:], line.startswith("[PASS]")))
                elif line.startswith("==="):
                    print("\n" + line, flush=True)
            # a Python error ends a section with exit code 1 too (like a failed test):
            # look for its traceback (not the harmless "Exception ignored" noise at exit)
            # (only a traceback through the script's top level: background tasks log harmless ones)
            real_tb = re.search(r'File ".*test_gateway\.py", line \d+, in <module>', r.stderr) is not None
            if r.returncode not in (0, 1) or real_tb:
                print("section %s CRASHED (the tests after the error did not run):\n%s" % (k, r.stderr[-1500:]))
                results.append(("section %s ran to the end" % k, False))
    bad = [n for n, ok in results if not ok]
    print("\nGATEWAY TESTS: %d passed, %d failed" % (len(results) - len(bad), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
