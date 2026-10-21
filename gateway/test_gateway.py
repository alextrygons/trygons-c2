"""Gateway tests on the PC: the simulator (sim/autopilot_sim server) plays the
AURIX, the real gateway code runs here like on the Pi.

    python test_gateway.py            all sections
    python test_gateway.py A C        only some sections

Exit code 0 = all passed.
"""
import glob
import math
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
sys.path.insert(0, HERE)
from common import load_config, proto, setup_logging  # noqa: E402
import gateway as gwmod  # noqa: E402

sys.path.insert(0, os.path.join(HERE, "..", "groundstation"))
from link import Link, Waiter  # noqa: E402

SIM_DIR = os.path.join(HERE, "..", "sim")
EXE = os.path.join(SIM_DIR, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
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
            "cameras": {"enabled": False},
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
    return {"alice": {"role": "HELM", "pw": hash_pw(PW["alice"])},
            "bob": {"role": "HELM", "pw": hash_pw(PW["bob"])},
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

    def __init__(self, port, user, pw=None, key=None, local=False, clock_offset=0.0, name=None):
        self.l = GatewayLink("127.0.0.1:%d" % port, user, password=pw, key=key, station=name or user,
                             local_addr="127.0.0.2" if local else "127.0.0.1", clock_offset=clock_offset)
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
               (r1, r2, r3, r4, r5, r6, r7) == (21, 0, 0, 22, 0, 21, 0) and "alice" in seen and "bob on board" in seen2,
               "alice without HELM %s, takes %s, command %s; bob (remote) takes %s; viewer sees HELM = %s; "
               "bob on board takes %s; alice now %s; bob's command %s; alice sees HELM = %s" % (
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

MTX_BIN = os.environ.get("MEDIAMTX_BIN") or shutil.which("mediamtx") or os.path.join(
    tempfile.gettempdir(), "claude", "c--trygons-TRYGONS-C2", "800448bc-0615-4bb2-9260-80ce21ee6535",
    "scratchpad", "mediamtx", "mediamtx.exe")
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
                _json.dump({"id": eid_, "start_t": 1577836800 + i, "synced": i < 3, "triggers": [{"text": "t"}]}, f)
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
            loc = vi.locate("FRONT-1", u)
            if loc:
                found += 1
                fr = videolog.frame_rgb(*loc)
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
    print("\n=== Fix F1: driver confirmation enforced in the gateway ===", flush=True)
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
        SEA = lambda n: proto.p_sea_state(n)

        def sea():
            alice.pump(0.6)
            return alice.telem["sea_state"] if alice.telem else None

        # F1a no driver screen: driving actions refused; STANDBY and non-driving commands still work
        r_sea = alice.result(alice.cmd(proto.T_SEA_STATE, SEA(3)))
        r_par = alice.result(alice.cmd(proto.T_PARAM_SET, proto.p_param_set(0, 1.0)))
        r_sb = alice.result(alice.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY)))
        r_ack = alice.result(alice.cmd(proto.T_ALARM_ACK))
        report("F1a no driver screen open: driving actions REFUSED (\"driver screen not open\"); STANDBY still works",
               r_sea == 109 and r_par == 109 and r_sb == 0 and r_ack == 0,
               "sea state %s, parameter %s (109 = driver screen not open); STANDBY %s, alarm ack %s (0 = OK)" % (
                   r_sea, r_par, r_sb, r_ack))

        # F1b driver screen open: ACCEPT -> sent; REJECT -> refused; no answer -> refused
        bob = Client(sp, "bob", PW["bob"], name="driver screen")
        beat = DriverBeat(bob)
        time.sleep(1.0)
        s0 = sea()
        new = 3 if s0 != 3 else 2
        seq = alice.l.send(proto.T_SEA_STATE, SEA(new), what="sea state %d" % new)
        k, ask = next_event(bob, ("driver_ask",))
        bob.l.send_msg({"t": "driver_answer", "id": ask["id"], "yes": True})
        a_yes = alice.result(alice.wait_ack(seq))
        s1 = sea()
        seq = alice.l.send(proto.T_SEA_STATE, SEA(s0), what="sea state back")
        k, ask2 = next_event(bob, ("driver_ask",))
        bob.l.send_msg({"t": "driver_answer", "id": ask2["id"], "yes": False})
        a_no = alice.result(alice.wait_ack(seq))
        t0 = time.time()
        seq = alice.l.send(proto.T_SEA_STATE, SEA(s0))
        a_to = alice.result(alice.wait_ack(seq, timeout=5))
        dt = time.time() - t0
        s2 = sea()
        report("F1b driver ACCEPT -> sent; REJECT -> refused; no answer within the timeout -> refused",
               ask and ask["text"] == "sea state %d" % new and a_yes == 0 and s1 == new and a_no == 110 and
               a_to == 111 and 1.5 < dt < 4.0 and s2 == new,
               "question on the driver screen: \"%s\" from %s; ACCEPT -> %s, boat sea state %s -> %s; REJECT -> %s; "
               "no answer -> %s after %.1f s; sea state still %s" % (
                   ask and ask["text"], ask and ask["from"], a_yes, s0, s1, a_no, a_to, dt, s2))

        # F1c the driver screen crashes / disconnects while a question is open -> rejected
        seq = alice.l.send(proto.T_SEA_STATE, SEA(s0))
        k, ask3 = next_event(bob, ("driver_ask",))
        beat.stop()                                         # crash: the heartbeat stops
        t0 = time.time()
        a_crash = alice.result(alice.wait_ack(seq, timeout=5))
        t_crash = time.time() - t0
        beat = DriverBeat(bob)
        time.sleep(0.8)
        seq = alice.l.send(proto.T_SEA_STATE, SEA(s0))
        k, ask4 = next_event(bob, ("driver_ask",))
        beat.stop()
        bob.close()                                         # the driver PC goes away
        a_gone = alice.result(alice.wait_ack(seq, timeout=5))
        r_after = alice.result(alice.cmd(proto.T_SEA_STATE, SEA(s0)))
        report("F1c driver screen crashes / disconnects during a question -> the pending action is rejected",
               ask3 is not None and a_crash == 112 and t_crash < 3.5 and ask4 is not None and a_gone == 112 and
               r_after == 109 and sea() == new,
               "heartbeat stopped -> %s after %.1f s (112 = driver screen lost); disconnected -> %s; next command -> %s; "
               "nothing changed on the boat" % (a_crash, t_crash, a_gone, r_after))

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
        report("F1d SINGLE-OPERATOR: driving action only with the captain's own confirmation; shown in the status; "
               "logged; OFF after a disconnect",
               so and so["on"] and r_nc == 113 and r_c == 0 and s3 == s0 and shown and off and r_again == 109 and
               len(logged) >= 2 and logged[0]["on"] and not logged[-1]["on"],
               "switched on %s; without own confirmation %s (113); with it %s, sea state %s; status shows it %s; "
               "after reconnect off %s, command %s (109 again); audit %s" % (
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
            ap.cmd(proto.T_SEA_STATE, proto.p_sea_state(s0), "sea state %d" % s0)
            asked = pump(4, lambda: ap.driver.question is not None)
            ap.driver.answer(True)
            sent = pump(5, lambda: ap.t["sea_state"] == s0)
            ap.cmd(proto.T_SEA_STATE, proto.p_sea_state(new), "sea state %d" % new)
            asked2 = pump(4, lambda: ap.driver.question is not None)
            ap.close_driver()                                   # the driver window is closed mid-question
            lost = pump(6, lambda: any("driver screen lost" in str(v) or "not open" in str(v)
                                       for v in ap.ev_texts()))
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
            report("F1e ground station: the driver window answers the gateway; closing it rejects; single-operator "
                   "banner on both screens + captain's own confirmation",
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


SECTIONS = {"F1": section_f1, "A": section_a, "B": section_b, "C": section_c, "D": section_d, "E": section_e, "F": section_f,
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
            if r.returncode not in (0, 1):
                print("section %s crashed:\n%s" % (k, r.stderr[-2000:]))
                results.append(("section %s ran" % k, False))
    bad = [n for n, ok in results if not ok]
    print("\nGATEWAY TESTS: %d passed, %d failed" % (len(results) - len(bad), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
