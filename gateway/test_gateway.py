"""Gateway tests on the PC: the simulator (sim/autopilot_sim server) plays the
AURIX, the real gateway code runs here like on the Pi.

    python test_gateway.py            all sections
    python test_gateway.py A C        only some sections

Exit code 0 = all passed.
"""
import glob
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
        r_vcmd = vic.result(vic.cmd(proto.T_SEA_STATE, SEA))
        r_vhelm = vic.result(vic.cmd(proto.T_HELM, proto.p_helm(0)))
        r_vread = vic.result(vic.cmd(proto.T_JOYMAP_GET))
        report("B2 VIEWER sees everything but can't command or take HELM",
               r_vcmd == 100 and r_vhelm == 100 and r_vread == 0 and vic.n_tel > 3,
               "command: %s, take HELM: %s, read: %s, telemetry frames %d" % (r_vcmd, r_vhelm, r_vread, vic.n_tel))
        link, w = sim.station()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 3)
        r1 = alice.result(alice.cmd(proto.T_SEA_STATE, SEA))
        r2 = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        r3 = alice.result(alice.cmd(proto.T_SEA_STATE, SEA))
        r4 = bob.result(bob.cmd(proto.T_HELM, proto.p_helm(0)))
        time.sleep(1.2)
        vic.pump(0.3)
        seen = vic.status["helm"]["name"] if vic.status else "?"
        r5 = bob_local.result(bob_local.cmd(proto.T_HELM, proto.p_helm(0)))
        r6 = alice.result(alice.cmd(proto.T_SEA_STATE, SEA))
        r7 = bob_local.result(bob_local.cmd(proto.T_HEADING_ADJ, proto.p_heading(1)))
        time.sleep(1.2)
        alice.pump(0.3)
        seen2 = alice.status["helm"]["name"] if alice.status else "?"
        report("B3 only the HELM station commands; one HELM; a local station takes it from a remote one; "
               "shown on every station",
               (r1, r2, r3, r4, r5, r6, r7) == (21, 0, 0, 22, 0, 21, 0) and "alice" in seen and "bob on board" in seen2,
               "alice without HELM %s, takes %s, command %s; bob (remote) takes %s; viewer sees HELM = %s; "
               "bob on board takes %s; alice now %s; bob's +1 in HDG LOCK %s; alice sees HELM = %s" % (
                   r1, r2, r3, r4, seen, r5, r6, r7, seen2))
        bob_local.cmd(proto.T_HELM, proto.p_helm(1))

        # B2b replay / duplicate / out of order / stale / future / rate / wrong session
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        s0 = alice.l.seq + 100
        alice.l.seq = s0 + 1000
        now = alice.l.now()
        ok1 = alice.result(alice.raw_cmd(s0, now, proto.T_SEA_STATE, SEA))
        dup = alice.result(alice.raw_cmd(s0, now, proto.T_SEA_STATE, SEA))
        old = alice.result(alice.raw_cmd(s0 - 5, now, proto.T_SEA_STATE, SEA))
        stale = alice.result(alice.raw_cmd(s0 + 1, now - 3.0, proto.T_SEA_STATE, SEA))
        fut = alice.result(alice.raw_cmd(s0 + 2, now + 5.0, proto.T_SEA_STATE, SEA))
        sess = alice.result(alice.raw_cmd(s0 + 3, now, proto.T_SEA_STATE, SEA, sid="0123456789abcdef"))
        for i in range(30):                                     # 30 at once, no waiting
            alice.l._send_obj({"t": "cmd", "sid": alice.l.welcome["sid"], "seq": s0 + 10 + i, "ts": alice.l.now(),
                               "type": proto.T_SEA_STATE, "p": SEA.hex()})
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
                seqs.append(c.l.send(proto.T_SEA_STATE, SEA))
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


SECTIONS = {"A": section_a, "B": section_b, "C": section_c}


def main():
    setup_logging(30)                       # warnings only
    if not os.path.exists(EXE):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    want = [a.upper() for a in sys.argv[1:]] or list(SECTIONS)
    for k in want:
        SECTIONS[k]()
    bad = [n for n, ok in results if not ok]
    print("\nGATEWAY TESTS: %d passed, %d failed" % (len(results) - len(bad), len(bad)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
