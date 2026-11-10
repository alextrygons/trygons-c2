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
os.environ.setdefault("TRYGONS_TEST", "1")      # temp track folder (review 13n3 L7)
sys.path.insert(0, HERE)
from common import load_config, proto, setup_logging  # noqa: E402
import gateway as gwmod  # noqa: E402

sys.path.insert(0, os.path.join(HERE, "..", "groundstation"))
from link import Link, Waiter  # noqa: E402

SIM_DIR = os.path.join(HERE, "..", "sim")
# AUTOPILOT_SIM: another simulator build (e.g. while the demo runs from sim/)
EXE = os.environ.get("AUTOPILOT_SIM") or os.path.join(SIM_DIR, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
results = []
# W50s: every simulator process and gateway runner of this process, so that (a) a section that stalls / crashes
# never leaves an orphan autopilot_sim.exe behind (one kept writing into sim/sdcard and holding ports in 13q5c) and
# (b) no GwRunner's event loop is ever garbage-collected: BaseEventLoop.__del__ -> close() of an unclosed Windows
# proactor loop can wait for ever in IocpProactor.close -> _poll, in WHATEVER thread the collector runs (mid-section)
_SIMS = []
_RUNNERS = []
SECTION_LIMIT_S = float(os.environ.get("TRYGONS_GW_SECTION_S", "900"))   # the parent's limit for one section
STALL_EXIT = 3                                  # a section's exit code: its watchdog found it stalled
CRASH_EXIT = 2                                  # a section's exit code: a Python error ended it


def report(name, ok, detail):
    results.append((name, ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def free_port():
    while True:
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        p = s.getsockname()[1]
        s.close()
        if p + 2 > 65535:                       # (U4 gate: an ephemeral port at the top of the range crashed CA)
            continue
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
                                  stdout=self.out, stderr=subprocess.STDOUT,   # never a full pipe
                                  stdin=subprocess.DEVNULL)
        _SIMS.append(self)
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
        try:
            self.p.terminate()
            self.p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.p.kill()
            try:
                self.p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print("W50s: autopilot_sim pid %d did not end after kill" % self.p.pid, file=sys.stderr, flush=True)
        except OSError:
            pass
        try:
            self.out.close()
        except OSError:
            pass
        if self in _SIMS:
            _SIMS.remove(self)


def kill_leftovers():
    """W50s: end every simulator this process started and did not stop (a stalled / crashed section)."""
    for s in list(_SIMS):
        try:
            s.p.kill()
            s.p.wait(timeout=3)
        except Exception:
            pass
        if s in _SIMS:
            _SIMS.remove(s)


class GwRunner:
    """The gateway in a thread with its own asyncio loop (like the service)."""

    def __init__(self, cfg):
        self.gw = gwmod.Gateway(cfg)
        self.loop = asyncio.new_event_loop()
        self.closed = False
        _RUNNERS.append(self)                   # W50s: kept for the whole process - never collected (see _RUNNERS)
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.gw.run())
        finally:
            self._close_loop()

    def _close_loop(self):
        """W50s: close the loop EXPLICITLY in its own (daemon) thread, every step with a time limit: cancel what is
        left, async generators, the default executor, then close(). If the proactor's close still waits for an
        overlapped operation that never completes, only this daemon thread waits - never the section or the exit."""
        loop = self.loop
        try:
            left = [t for t in asyncio.all_tasks(loop) if not t.done()]
            for t in left:
                t.cancel()
            if left:
                loop.run_until_complete(asyncio.wait(left, timeout=3.0))
            loop.run_until_complete(asyncio.wait_for(loop.shutdown_asyncgens(), 3.0))
            loop.run_until_complete(loop.shutdown_default_executor(timeout=3.0))
        except Exception as e:                  # a test result never depends on the clean-up
            print("W50s: gateway loop clean-up: %r" % (e,), file=sys.stderr, flush=True)
        try:
            loop.close()
            self.closed = True
        except Exception as e:
            print("W50s: gateway loop close: %r" % (e,), file=sys.stderr, flush=True)

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
        try:
            self.loop.call_soon_threadsafe(self.gw.stop)
        except RuntimeError:                    # the loop is closed already
            pass
        self.th.join(20)                        # run() ends + the clean-up (<= ~9 s) + close
        if self.th.is_alive():
            print("W50s: a gateway loop thread is still busy 20 s after stop (closed %s) - left to the daemon thread"
                  % self.closed, file=sys.stderr, flush=True)


def gw_cfg(sim, root, **extra):
    base = {"aurix": sim.gw_addr, "set_clock": False, "status_s": 1.0 / sim.speed,   # 1 Hz boat time
            "channels": {"driver": "sim", "starlink_status": False},
            "cameras": {"enabled": False}, "ais": {"enabled": False}, "vision": {"enabled": False},
            "loudhailer": {"enabled": False},           # 13r: off here (never this PC's sound card); tests in test_loudhailer.py
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
        # 57600 (like sim GA2): at 38400 the control traffic alone (telemetry + TELEM2 at 10 Hz, ~3.7 kB/s since
        # TELEM2 group O, step 7) fills the line, so the log copy gets no room at all - nothing left to note
        sim = Sim(5.0, "gwbaud=57600")
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
               "57600 baud: Pi copy %d records, %d gaps noted in gaps.csv (%d missing); SD card %d data records, "
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
        h_before = gw.gw.telem2.get("helm")
        r5 = bob_local.result(bob_local.cmd(proto.T_HELM, proto.p_helm(0)))
        # W55: the gateway's own grant is used at once (helm_forward) - and B3 also waits for TELEM2 to agree
        fwd_now = gw.gw.stations.helm_forward()
        import types as _ty
        _f = lambda g, gt, t2s: type(gw.gw.stations).helm_forward(_ty.SimpleNamespace(
            gw=_ty.SimpleNamespace(telem2={"helm": 1}), helm_grant=g, helm_grant_t=gt, helm_t2_since=t2s))
        unit_w55 = (_f(2, 10.0, 5.0), _f(2, 10.0, 11.0), _f(None, 10.0, 5.0), _f(1, 10.0, 5.0)) == (2, 1, None, 1)
        t_w = time.time() + 3.0
        while time.time() < t_w and gw.gw.telem2.get("helm") == h_before:
            time.sleep(0.05)
        r6 = alice.result(alice.cmd(proto.T_ALARM_ACK, b""))
        r7 = bob_local.result(bob_local.cmd(proto.T_ALARM_ACK, b""))
        time.sleep(1.2)
        alice.pump(0.3)
        seen2 = alice.status["helm"]["name"] if alice.status else "?"
        report("B3 only the HELM station commands; one HELM; a local station takes it from a remote one; "
               "shown on every station",
               (r1, r2, r3, r4, r5, r6, r7) == (21, 0, 0, 117, 0, 0, 0) and "alice" in seen and "bob on board" in seen2
               and fwd_now != h_before and fwd_now == gw.gw.telem2.get("helm") and unit_w55,
               "alice (captain) without HELM %s, takes %s, command %s; bob (remote copilot) takes %s (117: only the "
               "captain); viewer sees HELM = %s; bob on board takes %s (becomes the captain); alice now copilot: her "
               "command runs on the captain's behalf %s; bob's command %s; alice sees HELM = %s; W55 forward-for %s at the "
               "take ACK (TELEM2 had %s)" % (r1, r2, r3, r4, seen, r5, r6, r7, seen2, fwd_now, h_before))
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

        # E7 (13n2) the copilot's camera view: the camera added (LOW = the lighter stream) only while it is open;
        # closing it kicks the reader (no uplink any more)
        a = rig.ask(alice, preset="Front only")
        while True:
            try:
                alice.l.events.get_nowait()
            except Exception:
                break
        alice.l.send_msg({"t": "cam_view", "cam": "REAR-RIGHT"})
        av = rig.wait_ack(alice)
        vvs = VideoStream(rig.url(av, "REAR-RIGHT"), 320, 240).start() if av and av.get("streams", {}).get(
            "REAR-RIGHT") else None
        got = wait_for(lambda: vvs is not None and vvs.frames >= 10, 10)
        alice.l.send_msg({"t": "cam_view", "cam": None})
        ac = rig.wait_ack(alice)
        time.sleep(2.0)
        n0 = vvs.frames if vvs else 0
        time.sleep(2.0)
        n1 = vvs.frames if vvs else 0
        if vvs:
            vvs.stop()
        report("E7 (13n2) camera view: REAR-RIGHT added as LOW only while open (lighter = not the captain's: CV1), "
               "frames arrive; closed = "
               "the stream gone from the list and the reader kicked",
               bool(av) and av["streams"].get("REAR-RIGHT") == "cam_rearright_lo" and av.get("view") == "REAR-RIGHT"
               and av.get("view_light") == (not g.cams.is_priority(alice.welcome["id"])) and got and ac is not None and "REAR-RIGHT" not in ac["streams"] and
               ac.get("view") is None and n1 - n0 <= 1,
               "open: %s view %s light %s, frames %s; closed: %s; frames after close %d -> %d" % (
                   (av or {}).get("streams"), (av or {}).get("view"), (av or {}).get("view_light"), got,
                   (ac or {}).get("streams"), n0, n1))
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
        # 13p review L5: count the AURIX events before - a stale one from an earlier test never passes F5
        a0 = len([1 for _t, _f, e in rec.list_events() if e["triggers"][0].get("who") == "AURIX"])
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        hdg = w.until(lambda d: d["mode"] == 2, 5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.6, 0.0))
        man = w.until(lambda d: d["mode"] == 1, 5)
        # step 13p: the SAVE request reaches the recorder through the GATEWAY's own TELEM2 (its UART decode), which
        # can lag the direct sim link `w` under load (it failed twice in a full gate, before and with 13p, and passed
        # alone) - wait for the event itself, with a time limit
        wait_for(lambda: rec.saves > n0 and len([1 for _t, _f, e in rec.list_events()
                                                 if e["triggers"][0].get("who") == "AURIX"]) > a0, 20)
        ae = [e for _t, _f, e in rec.list_events() if e["triggers"][0].get("who") == "AURIX"]
        ae = ae if len(ae) > a0 else []
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
        # F9 KEEP 5' / 1h from a station (step 13n4): keep_ack with the station's id + how much was there; VIEWER no
        def keep_wait(client, **msg):
            client.l.send_msg(msg)
            end = time.time() + 4.0
            while time.time() < end:
                try:
                    k, d = client.l.events.get(timeout=0.1)
                except Exception:
                    continue
                if k == "keep_ack":
                    return d
            return None
        kid = time.strftime("%Y%m%d-%H%M%S", time.gmtime()) + "_keep5"
        k_v = keep_wait(vic, t="save_keep", min=5, id=kid)
        k_a = keep_wait(alice, t="save_keep", min=5, id=kid)
        wait_for(lambda: (event_json(rec, kid) or {}).get("state") in ("complete", "incomplete"), 3 * SEG + 10)
        kev = event_json(rec, kid) or {}
        report("F9 KEEP 5' from a station: its id, the cameras' files, keep_ack says how much the ring had; VIEWER refused",
               k_v and not k_v["ok"] and k_a and k_a["ok"] and k_a["id"] == kid and k_a["ask"] == kid and
               k_a["have_min"] is not None and k_a["have_min"] < 5 and "only" in k_a["text"] and kev.get("keep") and
               len(kev.get("cameras", {})) == 4 and all(kev["cameras"].values()),
               "viewer: %s; alice: %s (have %s min); event %s, files per camera %s" % (
                   k_v and k_v["text"], k_a and k_a["text"], k_a and k_a["have_min"], kev.get("state"),
                   {n: len(v) for n, v in kev.get("cameras", {}).items()}))
        s = rec.status()
        print("    recording status: ring %s MB over %s min (%s GB/h for the simple simulated pictures)" % (
            s["ring_mb"], s["ring_have_min"], s["gb_per_hour"]), flush=True)
        for c in (alice, bob, vic, onboard):
            c.close()
    finally:
        if rig:
            rig.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section KS --
# KEEP 5' / 1h (step 13n4, owner 2026-10-31 E4): the recorder alone (no MediaMTX, no simulator)

class _KsCam:
    def __init__(self, folder):
        self.folder, self.proc = folder, None
        self.state, self.since, self.restarts = "REC", time.time(), 0

    def tick(self, now):
        pass

    def segments(self):
        out = []
        for n in sorted(os.listdir(self.folder)) if os.path.isdir(self.folder) else []:
            t = seg_time(n)
            if t is not None:
                out.append((t, os.path.join(self.folder, n)))
        return out


class _KsGw:
    def __init__(self, root):
        from storage import Storage
        self.storage = Storage({"root": root, "require_mount": False, "low_free_gb": 0.0, "full_free_gb": 0.0})
        self.storage.check(force=True)
        self.telem, self.telem2 = {}, {}
        self.audits = []

        class _Clock:
            @staticmethod
            def stamp(t=None):
                return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() if t is None else t))
        self.clock = _Clock()

        class _Logs:
            path, last_seq, boot = None, None, None
        self.logs = _Logs()

    def audit(self, kind, **kw):
        self.audits.append((kind, kw))


def section_ks():
    print("\n=== Section KS: KEEP 5' / 1h (the last 5 / 60 min of every camera, permanent, not uploaded) ===",
          flush=True)
    from recorder import Recorder, is_keep
    from sync import SyncClient
    tmp = tempfile.mkdtemp(prefix="gwtest_ks_")
    try:
        g = _KsGw(tmp)
        rec = Recorder({"segment_s": 60}, g)
        now = time.time()
        for cam in CAM_NAMES:
            d = rec.ring_dir(cam)
            os.makedirs(d, exist_ok=True)
            for i in range(21):                     # 20 min of finished files + the one being written
                t = now - (20 - i) * 60
                with open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S.ts", time.gmtime(t))), "wb") as f:
                    f.write(b"x" * 1000)
            rec.cams[cam] = _KsCam(d)
        # KS1 the defaults: ring 60 min, a normal SAVE still 10 min before
        st = rec.status()
        report("KS1 defaults: ring 60 min (KEEP 1h possible), a normal SAVE keeps 10 min before + 2 min after",
               rec.ring_min == 60 and st["pre_s"] == 600 and st["post_s"] == 120 and
               load_config(None)["recording"]["ring_min"] == 60,
               "ring %s min, SAVE pre %s s / post %s s" % (rec.ring_min, st["pre_s"], st["post_s"]))
        # KS2 KEEP 5: the station's id, pre 5 min, post 0, keep flag, never merged with a SAVE
        sid = time.strftime("%Y%m%d-%H%M%S", time.gmtime(now)) + "_keep5"
        ok, text, eid, have = rec.keep(5, who="alice", eid=sid, now=now)
        rec.tick(now + 0.5)                         # the WORKER links the files (review 13n4 M1)
        ev = _json.load(open(os.path.join(rec.events_dir(), eid, "event.json"), encoding="utf-8"))
        files = ev["cameras"]["FRONT-1"]
        ok_s, _t, eid_s = rec.trigger("button", "SAVE (bob)", who="bob", now=now + 1)
        report("KS2 KEEP 5': the station's id, 5 min before, nothing after, permanent; a SAVE right after is its own event",
               ok and eid == sid and ev["keep"] and ev["pre_s"] == 300 and ev["post_s"] == 0 and 5 <= len(files) <= 6 and
               ok_s and eid_s != eid and ("save_keep", {"id": eid, "minutes": 5, "have_min": have, "user": "alice"})
               in g.audits,
               "%s -> %s, %d files per camera, SAVE after it -> %s" % (sid, text, len(files), eid_s))
        # KS3 KEEP 60 with 20 min in the ring: what exists + "only 20 min"
        ok6, text6, eid6, have6 = rec.keep(60, who="alice", eid="bad id/../x", now=now + 2)
        rec.tick(now + 2.5)
        ev6 = _json.load(open(os.path.join(rec.events_dir(), eid6, "event.json"), encoding="utf-8"))
        report("KS3 KEEP 1h with a 20 min ring: what exists is kept, the answer says how much; a bad id = the gateway's own",
               ok6 and 19.5 <= have6 <= 21.5 and "cameras: only" in text6 and len(ev6["cameras"]["REAR-LEFT"]) >= 20
               and recorder_id_ok(eid6) and ".." not in eid6,
               "%s (have %s min, %d files)" % (text6, have6, len(ev6["cameras"]["REAR-LEFT"])))
        # KS4 refused: not 5 / 60, STORAGE FULL (nothing kept is touched)
        bad = rec.keep(7, who="x")
        g.storage.state = "FULL"
        full = rec.keep(5, who="x")
        g.storage.check(force=True)
        still = os.path.isdir(os.path.join(rec.events_dir(), eid)) and os.path.isdir(os.path.join(rec.events_dir(), eid6))
        report("KS4 refused: only 5 / 60 min; STORAGE FULL = a message, nothing kept is touched",
               not bad[0] and "5 or 60" in bad[1] and not full[0] and "STORAGE FULL" in full[1] and still,
               "7 min: %s; full: %s" % (bad[1], full[1]))
        # KS5 never deleted automatically (even when synced, even when the ring deletes its own files), never uploaded
        for _c, folder, e in rec.list_events():
            for dp, _ds, fs in os.walk(folder):
                for fn in fs:
                    open(os.path.join(dp, fn) + ".synced", "w").close()
        rec.tick(now + 1e5)                         # every event finished
        rec.storage.low_b = 1e18                    # the disk "low": the policy deletes what it may
        rec.housekeeping(now + 1e5)
        kept_left = os.path.isdir(os.path.join(rec.events_dir(), eid)) and is_keep(os.path.join(rec.events_dir(), eid))
        ring_left = sum(len(c.segments()) for c in rec.cams.values())
        save_gone = not os.path.isdir(os.path.join(rec.events_dir(), eid_s))
        files_left = os.listdir(os.path.join(rec.events_dir(), eid, "FRONT-1"))
        rec.storage.low_b = 0
        sc = SyncClient.__new__(SyncClient)
        sc.root, sc.gw, sc._inc = tmp, g, {}
        g.rec = rec
        ts_rel = "video/events/%s/FRONT-1/%s" % (eid, [f for f in files_left if f.endswith(".ts")][0])
        js_rel = "video/events/%s/event.json" % eid
        ts_s = "video/events/%s/FRONT-1/x.ts" % eid_s
        os.makedirs(os.path.join(tmp, "video", "events", eid_s, "FRONT-1"), exist_ok=True)
        _json.dump({"state": "complete"}, open(os.path.join(tmp, "video", "events", eid_s, "event.json"), "w"))
        cls = (sc.classify(ts_rel, None, None), sc.classify(js_rel, None, None), sc.classify(ts_s, None, None))
        report("KS5 KEEP is permanent: the space policy deletes synced SAVEs and the ring, never a KEEP; its video is "
               "not uploaded (only event.json)",
               kept_left and save_gone and ring_left <= len(CAM_NAMES) and len(files_left) >= 6 and cls == (None, 1, 4),
               "KEEP left %s (%d files), the synced SAVE deleted %s, ring files left %d; sync class of KEEP .ts / "
               "event.json / a SAVE .ts: %s" % (kept_left, len(files_left), save_gone, ring_left, cls))
        ks_worker(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def ks_worker(tmp):
    """KS6 / KS7 (review 13n4 M1): the file work of a SAVE / KEEP is the worker's, never the event loop's."""
    from recorder import Recorder
    import threading
    root = os.path.join(tmp, "ks6")
    os.makedirs(root)
    g = _KsGw(root)
    rec = Recorder({"segment_s": 60, "house_s": 1e12}, g)
    now = time.time()
    for cam in CAM_NAMES:
        d = rec.ring_dir(cam)
        os.makedirs(d, exist_ok=True)
        for i in range(61):                         # 60 min of finished files + the one being written
            t = now - (60 - i) * 60
            with open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S.ts", time.gmtime(t))), "wb") as f:
                f.write(b"x" * 1000)
        rec.cams[cam] = _KsCam(d)
    gate, order = threading.Event(), []

    def slow_link(src, dst):                        # a drive that takes forever for every link
        order.append(os.path.basename(src))
        gate.wait(10)
        os.link(src, dst)
    rec.link_file = slow_link
    t0 = time.time()
    ok, _txt, eid, _have = rec.keep(60, who="alice", now=now)
    t_keep = time.time() - t0
    links_in_loop = len(order)
    worker = threading.Thread(target=rec.tick, args=(now + 0.5,), daemon=True)
    worker.start()
    time.sleep(0.3)                                 # the worker is now stuck in its first link
    t0 = time.time()
    ok_s, _t, eid_s = rec.trigger("button", "SAVE (bob)", who="bob", now=now + 1)
    ok2, _t2, eid2, _h2 = rec.keep(5, who="bob", now=now + 1.5)
    st = rec.status()
    t_loop = time.time() - t0
    gate.set()
    worker.join(30)
    stuck = worker.is_alive()
    first = list(order)                             # the worker's first step (the KEEP 1h)
    rec.tick(now + 1e5)                             # the rest + every event finished
    files = len(_json.load(open(os.path.join(rec.events_dir(), eid, "event.json"), encoding="utf-8"))["cameras"]
                ["FRONT-1"])
    oldest_first = bool(first) and first[:len(CAM_NAMES)] == [min(first)] * len(CAM_NAMES) and first == sorted(first)
    report("KS6 a KEEP 1h / SAVE never does file work on the event loop: no link inside keep() / trigger(); with the "
           "worker stuck in a link the loop's SAVE + KEEP + status answer at once; the worker links the oldest "
           "files first",
           ok and ok_s and ok2 and links_in_loop == 0 and t_keep < 0.5 and t_loop < 0.5 and files >= 59 and
           oldest_first and not stuck,
           "keep() %.3f s with %d links, SAVE + KEEP + status while the worker is blocked %.3f s; KEEP 1h files %d; "
           "first links %s" % (t_keep, links_in_loop, t_loop, files, order[:3]))
    # KS7 no hard links on the drive: a copy only while it leaves the free-space margin
    root7 = os.path.join(tmp, "ks7")
    os.makedirs(root7)
    g7 = _KsGw(root7)
    rec7 = Recorder({"segment_s": 60, "house_s": 1e12}, g7)
    for cam in CAM_NAMES[:1]:
        d = rec7.ring_dir(cam)
        os.makedirs(d, exist_ok=True)
        for i in range(6):
            t = now - (5 - i) * 60
            with open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S.ts", time.gmtime(t))), "wb") as f:
                f.write(b"x" * 1000)
        rec7.cams[cam] = _KsCam(d)

    def no_link(src, dst):
        raise OSError("no hard links here")
    rec7.link_file = no_link
    g7.storage.low_b = g7.storage.free_now() + 1e9           # "no room": every copy refused
    okf, _tf, eidf, _hf = rec7.keep(5, who="x", now=now)
    rec7.tick(now + 0.5)
    evf = _json.load(open(os.path.join(rec7.events_dir(), eidf, "event.json"), encoding="utf-8"))
    refused = rec7.copy_refused
    g7.storage.low_b = 0                                     # room: copied
    okc, _tc, eidc, _hc = rec7.keep(5, who="x", now=now + 1)
    rec7.tick(now + 1.5)
    evc = _json.load(open(os.path.join(rec7.events_dir(), eidc, "event.json"), encoding="utf-8"))
    copied = os.listdir(os.path.join(rec7.events_dir(), eidc, CAM_NAMES[0]))
    report("KS7 no hard links on the drive: a copy only while it leaves the free-space margin (else nothing copied, "
           "counted); with room the files are copied",
           okf and evf["cameras"][CAM_NAMES[0]] == [] and refused >= 5 and okc and
           len(evc["cameras"][CAM_NAMES[0]]) >= 5 and len(copied) >= 5,
           "no room: %d listed, %d refused; room: %d copied" % (len(evf["cameras"][CAM_NAMES[0]]), refused,
                                                                 len(copied)))
    # KS8 (review 13n4b L9) a refused copy is counted ONCE per file (not every worker second), in event.json + status
    g7.storage.low_b = g7.storage.free_now() + 1e9
    n0 = rec7.copy_refused
    okr, _tr, eidr, _hr = rec7.keep(5, who="x", now=now + 2)
    for k in range(4):                                       # the worker retries every second while it is open
        rec7.tick(now + 2.5 + k)
    evr = _json.load(open(os.path.join(rec7.events_dir(), eidr, "event.json"), encoding="utf-8"))
    per_file = rec7.copy_refused - n0
    g7.storage.low_b = 0
    report("KS8 (review 13n4b L9) a file not kept (no hard link, no room) is counted and logged ONCE, not every worker "
           "second; event.json copy_refused + the status show it",
           okr and 5 <= per_file <= 6 and evr.get("copy_refused") == per_file and
           rec7.status()["copy_refused"] == rec7.copy_refused,
           "4 worker steps: %d counted, event.json %s, status %s" % (per_file, evr.get("copy_refused"),
                                                                    rec7.status()["copy_refused"]))
    # KS9 (review 13n4b L8) the ring's housekeeping never deletes a file inside an open event's window: a KEEP 1h made
    # with a 10 min ring, housekeeping BEFORE the worker linked it -> its old files are still there for the link
    root9 = os.path.join(tmp, "ks9")
    os.makedirs(root9)
    g9 = _KsGw(root9)
    rec9 = Recorder({"segment_s": 60, "house_s": 1e12, "ring_min": 10}, g9)
    rec9.ring_min = 10
    d = rec9.ring_dir(CAM_NAMES[0])
    os.makedirs(d, exist_ok=True)
    for i in range(31):
        t = now - (30 - i) * 60
        with open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S.ts", time.gmtime(t))), "wb") as f:
            f.write(b"x" * 1000)
    rec9.cams[CAM_NAMES[0]] = _KsCam(d)
    ok9, _t9, eid9, _h9 = rec9.keep(60, who="x", now=now)
    rec9.housekeeping(now)                                  # the race: housekeeping first
    left = len(rec9.cams[CAM_NAMES[0]].segments())
    rec9.tick(now + 0.5)
    ev9 = _json.load(open(os.path.join(rec9.events_dir(), eid9, "event.json"), encoding="utf-8"))
    kept = len(ev9["cameras"][CAM_NAMES[0]])
    rec9.tick(now + 1e5)                                    # the KEEP finished: the ring is trimmed again
    rec9.housekeeping(now + 1)
    after = len(rec9.cams[CAM_NAMES[0]].segments())
    report("KS9 (review 13n4b L8) housekeeping skips ring files inside an open SAVE / KEEP window (10 min ring, KEEP 1h "
           "made just before): all 30 files still linked; once the event is done the ring is trimmed to its length",
           ok9 and left == 31 and kept >= 30 and after <= 13,
           "files left after housekeeping %d, linked %d, ring after the event %d" % (left, kept, after))
    # KS10 (review 13n4b L10) "merged" only for a real merge into an open SAVE - a taken id ("-2" suffix) is not one
    sid = time.strftime("%Y%m%d-%H%M%S", time.gmtime(now + 2e5)) + "_save"
    o1 = rec9.trigger("button", "SAVE 1", who="a", now=now + 2e5, eid=sid)
    m1 = rec9.last_merged
    o2 = rec9.trigger("button", "SAVE 2", who="b", now=now + 2e5 + 5, eid=sid + "x")
    m2 = rec9.last_merged
    rec9.tick(now + 3e5)                                    # both finished
    o3 = rec9.trigger("button", "SAVE 3", who="c", now=now + 3e5, eid=sid)      # the id is taken: "-2"
    m3 = rec9.last_merged
    report("KS10 (review 13n4b L10) save_ack \"merged\" only for a real merge into an open camera event; an id that "
           "was taken (\"-2\" suffix) is a new event, not a merge",
           o1[0] and not m1 and o2[0] and m2 and o2[2] == o1[2] and o3[0] and not m3 and o3[2] == sid + "-2",
           "first %s merged %s; second -> %s merged %s; taken id -> %s merged %s" % (o1[2], m1, o2[2], m2, o3[2], m3))


def recorder_id_ok(eid):
    from recorder import EVENT_ID_RE
    return bool(EVENT_ID_RE.match(eid))


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
        # step 13q2b (LOW-4): there is no release any more (refused while driving); REMOTE ends here by the
        # freshness rule - the shore joystick data stops: LOST (idle + neutral, gyro hold) at once, STANDBY after 5 s
        r_rel = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(1)))
        still = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 1)
        snd.stop()
        lost6 = w.until(lambda d: d.get("remote_state") == 2, 2)
        off = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 8)
        again = wait_for(lambda: sc.state in ("SYNCING", "IDLE", "CONNECTING"), 8)
        report("G6 REMOTE MANUAL driving: sync pauses; the release is refused; the data stops -> LOST, STANDBY, sync "
               "continues",
               on is not None and p6 and quiet6 and r_rel == 7 and still is not None and
               lost6 is not None and off is not None and again,
               "remote driving %s -> sync %s (\"%s\"), bytes in 2 s %d; release -> %s (7), still REMOTE %s; data "
               "stopped -> LOST %s, STANDBY %s -> sync %s" % (
                   "on" if on else "NOT on", "PAUSED" if p6 else sc.state, why6, sc.sent_b - b3, r_rel,
                   still is not None, lost6 is not None, off is not None, sc.state))
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
        f1e_err = []
        troot.report_callback_exception = lambda *a: f1e_err.append(repr(a[1]))
        ap = A.App(troot, "gw:127.0.0.1:%d" % sp, login=("alice", PW["alice"]))
        # a watchdog: where is the station's main thread when its Tk loop stalls for more than 2 s?
        import traceback as _tb2
        f1e_beat = [time.time()]
        f1e_stalls = []
        f1e_main = threading.get_ident()

        def _watch():
            while f1e_beat[0] > 0:
                time.sleep(0.5)
                if f1e_beat[0] > 0 and time.time() - f1e_beat[0] > 2.0:
                    fr = sys._current_frames().get(f1e_main)
                    f1e_stalls.append((round(time.time() - f1e_beat[0], 1),
                                       "".join(_tb2.format_stack(fr, limit=8)) if fr else "?"))
                    time.sleep(3.0)
        threading.Thread(target=_watch, daemon=True).start()

        def pump(s, until=None):
            end = time.time() + s
            while time.time() < end:
                f1e_beat[0] = time.time()
                troot.update()
                f1e_beat[0] = time.time()
                if until and until():
                    return True
                time.sleep(0.02)
            return bool(until and until())
        try:
            pump(8, lambda: ap.t is not None)
            f1e_id0 = ap.link.welcome.get("id")
            f1e_seen = []                                   # every gateway message kind the station received
            _put = ap.link.events.put
            ap.link.events.put = lambda e: (f1e_seen.append(e[0]), _put(e))[1]
            _ask = ap.on_driver_ask
            ap.on_driver_ask = lambda d: (f1e_seen.append("ASK:%s" % d.get("text")), _ask(d))[1]
            ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
            pump(1.0)
            g.stations.driver_timeout = 10.0                    # the real answer time (F1b tested the short one)
            ap.open_driver()
            # the gateway must have seen the captain's screen 1 open (1 s heartbeat; a busy PC is slower)
            pump(8, lambda: any(s.driver_open for s in g.stations.drivers() if s.user == "alice"))
            pump(0.5)
            cop = Client(sp, "bob", PW["bob"], name="copilot")
            seq = cop.l.send(proto.T_SEA_STATE, proto.p_sea_state(s0), what="sea state %d" % s0)
            asked = pump(8, lambda: ap.driver.question is not None)
            f1e_diag = "" if asked else "; copilot answer %s; gateway stations %s; station got %s; answers %s" % (
                cop.result(cop.wait_ack(seq, 0.5)) if not asked else None,
                [(s.user, s.crew, s.driver_open, round(time.time() - s.driver_seen, 1))
                 for s in g.stations.stations.values()],
                [k for k in f1e_seen if k != "telemetry"][-12:] + ["telemetry x%d" % f1e_seen.count("telemetry")],
                (ap.driver.answers[-3:], A.i18n.lang(), round(ap.driver.render_ms)) if ap.driver else "no driver")
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
            f1e_lost = (a_lost and a_lost["result"], len(g.stations.asks),
                        [(s_.user, s_.id, s_.driver_open, round(time.time() - s_.driver_seen, 1))
                         for s_ in g.stations.stations.values()], ap.link.welcome.get("id"),
                        [k for k in f1e_seen if k not in ("telemetry", "gw_status", "ack")][-8:])
            cop.close()
            ap.single_op_toggle(confirmed=True)
            so_on = pump(4, lambda: ap.single_op)
            ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
            ap.open_driver()
            pump(1.5)
            ap.render(force=True)
            banner_c = ap.banner_on("single_op")
            banner_d = bool(ap.driver.status.find_withtag("singleop"))
            f1e_so = (ap.single_op, ap.link.welcome.get("id"), ap.link.connected)
            f1e_stack = []
            _say = ap.say
            import traceback as _tb
            ap.say = lambda text, color=None: (f1e_stack.append("".join(_tb.format_stack(limit=6))) if "NOT SENT"
                                               in text or "ΔΕΝ ΣΤΑΛΘΗΚΕ" in text else None, _say(text, color))[1]
            ap.cmd(proto.T_SEA_STATE, proto.p_sea_state(new), "sea state %d" % new)
            pump(0.5)
            own = ap.cconf is not None and ap.driver.question is None
            f1e_own = (ap.cconf is not None, ap.driver.question and ap.driver.question[0], ap.driver.answers[-3:],
                       [k for k in f1e_seen if k not in ("telemetry", "gw_status", "ack")][-6:],
                       (ap.gw_status or {}).get("avoid"), ((ap.gw_status or {}).get("vision") or {}).get("proposal"))
            ap.cconf_send()
            sent2 = pump(5, lambda: ap.t["sea_state"] == new)
            report("F1e ground station: the captain's driver window answers the copilot's question; closing it "
                   "rejects; single-operator banner on both screens + own confirmation",
                   asked and sent and asked2 and lost and so_on and banner_c and banner_d and own and sent2,
                   "question shown on the driver window %s, ACCEPT -> boat %s; closed mid-question -> rejected %s; "
                   "single-operator on %s, banner captain %s / driver %s; own confirmation window %s -> sent %s %s" % (
                       asked, sent, lost, so_on, banner_c, banner_d, own, sent2, "" if sent2 else ap.ev_texts()[-6:])
                   + ("" if asked else "; screen-2 errors %s; events %s; crew %s%s" % (
                       f1e_err[:3], ap.ev_texts()[-6:], A.crewui.crew_of(ap), f1e_diag))
                   + ("" if lost and own else "; lost-check %s; single-op at the command %s; errors %s; station id "
                      "at the start %s; own-check %s; not-sent stack %s; STALLS %s" % (
                          f1e_lost, f1e_so, f1e_err[:3], f1e_id0, f1e_own, f1e_stack[:1], f1e_stalls[:3])))
        finally:
            f1e_beat[0] = 0
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
        stick = {"x": 0.0, "lever": 0.5, "y": 0.0}     # step 13q5: the stick Y beside the lever (MANEUVER gear)
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        r_helm = alice.result(alice.cmd(proto.T_HELM, proto.p_helm(0)))
        r_ask = alice.result(alice.cmd(proto.T_MANEUVER, proto.p_remote(0)))
        time.sleep(0.5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))   # the person on board
        time.sleep(0.2)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        on = w.until(lambda d: d["mode"] == proto.MODE_MANEUVER, 3)
        stick["lever"] = 1.0                             # step 13q5: the lever is not used in MANEUVER ...
        # step 13q5b (review LOW-1): a window that can fail - the boat's own gear ask (TELEM2 D mv_stick_ask) at once,
        # and the engaged gear over 4.5 s (an N -> F shift takes >= 2.8 s)
        lever_f = False
        t0 = time.time()
        while time.time() - t0 < 4.5:
            w.pump(0.1)
            if w.telem:
                lever_f = lever_f or w.telem["gear"] == 1 or w.telem.get("mv_stick_ask", 0) != 0
        stick["lever"] = 0.5
        stick["y"] = 1.0                                 # ... the stick full forward, held = FORWARD
        top, gear_f = 0.0, False
        t0 = time.time()
        while time.time() - t0 < 6:
            w.pump(0.1)
            if w.telem:
                top = max(top, w.telem["throttle"])
                gear_f = gear_f or w.telem["gear"] == 1
        prof = w.telem.get("profile_name") if w.telem else None
        report("CR2 MANEUVER through the gateway: licensed captain asks, CONFIRM on board; the lever does nothing, the "
               "stick held forward = forward gear (13q5), throttle idle",
               r_helm == 0 and r_ask == 0 and on is not None and not lever_f and gear_f and top <= 0.01 and
               prof == "CREWED",
               "alice: HELM %s, MANEUVER %s, after CONFIRM: %s; full lever: gear F %s (none); stick forward: gear F %s, "
               "highest throttle %.2f %%; profile %s" % (r_helm, r_ask, on and on["mode_name"], lever_f, gear_f, top,
                                                         prof))
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
        # step 13q2 (owner 2026-10-31): UNCREWED = one action, REMOTE at once (shore stick centred + lever neutral),
        # no CONFIRM; an off-centre stick is refused (NACK 8); an old station's "release" never ends it
        stick["x"] = 0.6
        time.sleep(0.4)
        r_off = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(0)))
        w.pump(0.4)
        ignored = w.telem and w.telem["mode"] == proto.MODE_STANDBY
        stick["x"] = 0.0
        time.sleep(0.4)
        r_ask = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(0)))
        on = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 3)
        stick["release"] = True                          # an old station's release button: dropped
        time.sleep(0.6)
        stick["release"] = False
        w.pump(0.4)
        kept = w.telem and w.telem["mode"] == proto.MODE_REMOTE
        report("CR3 UNCREWED: REMOTE at once on the captain's request with the shore stick centred (off centre: NACK "
               "8, nothing starts); an old \"release\" never ends it (step 13q2)",
               unc is not None and r_off == 8 and ignored and r_ask == 0 and on is not None and kept,
               "profile %s; stick off centre %s (8), still STANDBY %s; centred %s -> %s; old release -> still REMOTE "
               "%s" % (unc and unc.get("profile_name"), r_off, ignored, r_ask, on and on["mode_name"], kept))
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
        # (CR10, section N) the captain's RESTART through the gateway clears the ENGINE KILL - the boat checks
        # idle + neutral + e-stop released; nothing starts by itself
        r_rs = alice.result(alice.cmd(proto.T_RESTART, proto.p_restart(proto.ENG_RESTART)))
        freed = w.until(lambda d: d.get("ll_state") == 0 and not d.get("eng_killed"), 5)
        w.pump(2.0)
        calm = bool(w.telem) and w.telem["mode"] == proto.MODE_STANDBY and w.telem["throttle"] <= 0.6
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
        # CR10 engine controls (section N): the captain's RESTART cleared the kill above; a copilot can't hold
        # START (118) and its RESTART goes to the captain's screen 1 for the ACCEPT (none open here: 109)
        r_bob_crank = bob.result(bob.cmd(proto.T_RESTART, proto.p_restart(proto.ENG_CRANK)))
        r_bob_rs = bob.result(bob.cmd(proto.T_RESTART, proto.p_restart(proto.ENG_RESTART)))
        r_bob_rel = bob.result(bob.cmd(proto.T_RESTART, proto.p_restart(proto.ENG_RELEASE)))
        report("CR10 engine RESTART through the gateway: the captain's clears the ENGINE KILL (boat checks idle + "
               "neutral), nothing starts; a copilot can't hold START, its RESTART needs the captain's ACCEPT",
               r_rs == 0 and freed is not None and calm and r_bob_crank == 118 and r_bob_rs == 109 and r_bob_rel == 0,
               "captain RESTART %s -> kill cleared %s, STANDBY idle %s; copilot START %s (118), RESTART %s (109 = no "
               "captain screen to ACCEPT), RELEASE %s" % (r_rs, freed is not None, calm, r_bob_crank, r_bob_rs, r_bob_rel))

        # LT1 LINK TIME (step 13q1): the HELM captain only; the AURIX keeps it (TELEM2 group S); the gateway keeps the
        # HELM station LINK TIME + 2 s before it drops it and reports its age in GW_STATUS
        r_bob_lt = bob.result(bob.cmd(proto.T_LINK_TIME, proto.p_link_time(30)))
        r_al_bad = alice.result(alice.cmd(proto.T_LINK_TIME, proto.p_link_time(61)))
        r_al_lt = alice.result(alice.cmd(proto.T_LINK_TIME, proto.p_link_time(30)))
        lt_got = w.until(lambda d: d.get("lt_set") == 30 and d.get("lt_override"), 3)
        # step 13p (review of b50413b): lost_for() reads the GATEWAY's own TELEM2 (its UART decode, it can lag the
        # direct sim link `w` under load) - wait for that copy too, with a time limit
        t_lt = time.time() + 5.0
        while (g.telem2 or {}).get("lt_set") != 30 and time.time() < t_lt:
            time.sleep(0.05)
        g.stations.tick()
        st_al = g.stations.stations.get(alice.welcome["id"])
        st_bob = g.stations.stations.get(bob.welcome["id"])
        keep_al = g.stations.lost_for(st_al) if st_al else None
        keep_bob = g.stations.lost_for(st_bob) if st_bob else None
        hid, hage = g.stations.helm_age()
        pl = proto.p_gw_status(0, 0, 0, 0, 0, hid, hage)
        pl_gone = proto.p_gw_status(0, 0, 0, 0, 0, hid, None)
        pl_none = proto.p_gw_status(0, 0, 0, 0, 0, 1, 0.5)
        r_al_lt3 = alice.result(alice.cmd(proto.T_LINK_TIME, proto.p_link_time(3)))
        report("LT1 LINK TIME through the gateway: the HELM captain only (copilot 118), 3..60 s on the AURIX (TELEM2 S), "
               "the HELM station kept LINK TIME + 2 s, its age in GW_STATUS",
               r_bob_lt == 118 and r_al_bad == 3 and r_al_lt == 0 and lt_got is not None and keep_al == 32.0 and
               keep_bob == g.stations.lost_s and hid == alice.welcome["id"] and len(pl) == 12 and pl[10] == hid and
               pl[11] <= 8 and pl_gone[11] == 255 and pl_none[10:] == bytes(2) and r_al_lt3 == 0,
               "copilot %s (118), captain 61 s %s (3), 30 s %s -> TELEM2 %s; kept %s s (bob %s s); GW_STATUS station "
               "%s age %s (x 0.25 s), gone %s, USB helm %r; back to 3 s %s" % (
                   r_bob_lt, r_al_bad, r_al_lt, lt_got and lt_got.get("lt_set"), keep_al, keep_bob, pl[10], pl[11],
                   pl_gone[11], pl_none[10:], r_al_lt3))

        # SR1 slamming power restore (step 13q4, owner 2026-10-31): the captain's ACCEPT only - a copilot's is
        # refused by the gateway (118, never forwarded for the captain's ACCEPT); the captain's goes to the AURIX,
        # which refuses it with nothing reduced (6 = PRECOND); TELEM2 group T decoded
        r_bob_sr = bob.result(bob.cmd(proto.T_SLAM_RESTORE, proto.p_slam_restore(True)))
        r_bob_rj = bob.result(bob.cmd(proto.T_SLAM_RESTORE, proto.p_slam_restore(False)))
        r_al_sr = alice.result(alice.cmd(proto.T_SLAM_RESTORE, proto.p_slam_restore(True)))
        t_sr = w.until(lambda d: "slr_flags" in d, 3)
        report("SR1 slamming power restore through the gateway: the captain only (copilot 118, ACCEPT and REJECT), "
               "the AURIX decides (nothing reduced = 6); TELEM2 group T",
               r_bob_sr == 118 and r_bob_rj == 118 and r_al_sr == 6 and t_sr is not None and
               not t_sr.get("slam_ask") and not t_sr.get("slam_restore_ok"),
               "copilot ACCEPT %s / REJECT %s (118), captain ACCEPT %s (6 = nothing to restore); TELEM2 T %s" % (
                   r_bob_sr, r_bob_rj, r_al_sr, t_sr and (t_sr.get("slr_flags"), t_sr.get("slr_before"))))

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
        la, lo = east_of(2000.0 - 250.0 - 5.0)                    # 5 m from the edge of the shoal (~36 m from the cell: near 50 m)
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
               f_boat == 2 and 1500 <= d_boat <= 1900 and f_near == 3 and 20 <= d_near <= 50 and
               (f_none, d_none) == (0, 0xFFFF) and got is not None and abs(got["coast_dist_m"] - d_boat) <= 60 and
               not got["near_shore"],
               "boat: flags %d, %d m from the shoal; 5 m from its edge: flags %d (near), %d m; no charts: flags %d, "
               "%d (= not valid -> near on the AURIX); AURIX telemetry: %s m, near %s" % (
                   f_boat, d_boat, f_near, d_near, f_none, d_none, got and got["coast_dist_m"],
                   got and got["near_shore"]))
        link.stop()
        gw.stop()
        sim.stop()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section GF --

def fence_around(half_m=1000.0):
    return [east_of(-half_m, -half_m), east_of(-half_m, half_m), east_of(half_m, half_m), east_of(half_m, -half_m)]


def section_gf():
    """Geofence + the copilot's work on the map (CLAUDE.md D): routes must stay inside the fence (code 120); a
    copilot's route (PROG_COMMIT) and fence change wait for the captain's ACCEPT on screen 1."""
    print("\n=== Section GF: geofence, the copilot's route / fence -> the captain's ACCEPT ===", flush=True)
    import geofence as GF
    sq = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)]
    u = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.7), (0.3, 0.7), (0.3, 0.3), (1.0, 0.3), (1.0, 0.0)]   # a "C"
    ins = GF.inside(sq, 0.5, 0.5) and not GF.inside(sq, 1.5, 0.5) and not GF.inside(sq, -0.1, 0.5)
    v_cross = GF.valid([(0, 0), (1, 1), (0, 1), (1, 0)])
    v_few, v_range, v_clear = GF.valid([(0, 0), (1, 1)]), GF.valid([(0, 0), (95, 1), (1, 1)]), GF.valid([])
    v_nan = GF.valid([(0, 0), (float("nan"), 1), (1, 1)])
    wp_out = GF.check_route(sq, [(0.2, 0.2), (0.2, 1.5)])
    leg_out = GF.check_route(u, [(0.8, 0.2), (0.8, 0.8)], step_m=200.0)           # both inside, the leg cuts the gap
    fine = GF.check_route(u, [(0.1, 0.2), (0.1, 0.8)], step_m=200.0)
    report("GF1 geofence maths: inside / outside, crossing / too few / out of range / NaN corners refused, a waypoint "
           "outside and a leg through a gap of a C-shaped fence found",
           ins and not v_cross[0] and "crosses" in v_cross[1] and not v_few[0] and not v_range[0] and not v_nan[0] and
           v_clear == (True, "", []) and wp_out and wp_out.startswith("WP2 is outside") and leg_out and
           leg_out.startswith("leg WP1 -> WP2 leaves") and fine is None,
           "inside %s; crossing %s; 2 corners %s; out of range %s; NaN %s; empty = clear %s; %r; %r; inside route %s" % (
               ins, v_cross[1], v_few[1], v_range[1], v_nan[1], v_clear[0], wp_out, leg_out and leg_out[:40], fine))

    tmp = tempfile.mkdtemp(prefix="gwtest_gf_")
    sim = gw = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts)                                   # deep water around the simulator (a shoal 2 km east)
        cfg = gw_cfg_b(sim, root, charts_dir=charts, stations={"driver_timeout_s": 6.0},
                       shallow={"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        beat = DriverBeat(alice)                            # the captain's screen 1 is open
        bob = Client(sp, "bob", PW["bob"], name="copilot")
        bob.pump(1.0)
        crews = (g.stations.stations[alice.welcome["id"]].crew, g.stations.stations[bob.welcome["id"]].crew)

        # GF2 the captain sets the fence (no question); routes out of it refused (120), inside accepted
        fence = fence_around(1000.0)
        alice.l.send_msg({"t": "geofence", "poly": [list(p_) for p_ in fence], "what": "geofence: 4 corners"})
        k, ack1 = next_event(alice, ("geofence_ack",), 4.0)
        alice.pump(1.5)
        st = (alice.status or {}).get("geofence") or {}
        a_out = upload(alice, [east_of(300), east_of(-1800, 300)])           # deep water, but out of the fence
        a_in = upload(alice, [east_of(300), east_of(600, 300)])
        report("GF2 the captain's geofence is set at once (in the status, audited); a route leaving it refused (120, the "
               "leg + position), a route inside accepted",
               crews == ("captain", "copilot") and ack1 and ack1["ok"] and len(st.get("poly") or []) == 4 and
               st.get("by") == "alice" and a_out and a_out["result"] == 120 and "geofence" in a_out["text"] and
               a_in and a_in["result"] == 0 and any(e["kind"] == "geofence" for e in g.audit.recent),
               "crews %s; ack %s; status %d corners by %s; out %s \"%s\"; in %s" % (
                   crews, ack1 and ack1.get("text"), len(st.get("poly") or []), st.get("by"),
                   a_out and a_out["result"], a_out and a_out["text"][:90], a_in and a_in["result"]))

        # GF3 the copilot's route: the COMMIT waits for the captain's ACCEPT (yes -> on the boat, no -> 110)
        prog = {"final": 0, "wps": [{"lat": a, "lon": b, "kn": 8.0, "radius": 30, "action": 0}
                                    for a, b in (east_of(200), east_of(500, 200))]}
        frames = progmod.upload_frames(prog)
        for t, p_ in frames[:-1]:
            bob.l.send(t, p_)
        bob.pump(0.3)
        seq = bob.l.send(frames[-1][0], frames[-1][1], what="program upload (2 waypoints)")
        k, ask = next_event(alice, ("driver_ask",), 4.0)
        if ask:
            alice.l.send_msg({"t": "driver_answer", "id": ask["id"], "yes": True})
        r_yes = bob.result(bob.wait_ack(seq, 6.0))
        prog_n = w.until(lambda d: d["prog_n"] == 2, 3)
        for t, p_ in frames[:-1]:
            bob.l.send(t, p_)
        bob.pump(0.3)
        seq = bob.l.send(frames[-1][0], frames[-1][1], what="program upload (2 waypoints)")
        k, ask2 = next_event(alice, ("driver_ask",), 4.0)
        if ask2:
            alice.l.send_msg({"t": "driver_answer", "id": ask2["id"], "yes": False})
        r_no = bob.result(bob.wait_ack(seq, 6.0))
        report("GF3 the copilot's new AUTO route (PROG_COMMIT) waits for the captain's ACCEPT on screen 1: accepted -> "
               "on the boat, rejected -> refused (110)",
               ask is not None and "program" in ask["text"] and r_yes == 0 and prog_n is not None and ask2 is not None
               and r_no == 110,
               "question %s; accepted -> %s, boat program %s; rejected -> %s" % (
                   ask and ask["text"], r_yes, prog_n and prog_n["prog_n"], r_no))

        # GF4 the copilot changes / clears the fence: the captain's ACCEPT decides; persisted on the SSD
        big = fence_around(1500.0)
        bob.l.send_msg({"t": "geofence", "poly": [list(p_) for p_ in big], "what": "geofence: 4 corners"})
        k, ask3 = next_event(alice, ("driver_ask",), 4.0)
        if ask3:
            alice.l.send_msg({"t": "driver_answer", "id": ask3["id"], "yes": False})
        k, ack3 = next_event(bob, ("geofence_ack",), 4.0)
        same = g.geofence.poly == [tuple(p_) for p_ in fence] or g.geofence.poly == fence
        bob.l.send_msg({"t": "geofence", "poly": [list(p_) for p_ in big], "what": "geofence: 4 corners"})
        k, ask4 = next_event(alice, ("driver_ask",), 4.0)
        if ask4:
            alice.l.send_msg({"t": "driver_answer", "id": ask4["id"], "yes": True})
        k, ack4 = next_event(bob, ("geofence_ack",), 4.0)
        changed = abs(g.geofence.poly[0][0] - big[0][0]) < 1e-9 and g.geofence.by == "bob"
        bad = []
        bob.l.send_msg({"t": "geofence", "poly": [[0, 0], [1, 1]]})
        k, ack5 = next_event(bob, ("geofence_ack",), 4.0)
        reread = GF.Geofence(g.geofence.path) if g.geofence.path else None
        kept = reread is not None and len(reread.poly) == 4 and reread.by == "bob"
        report("GF4 the copilot's fence change waits for the captain's ACCEPT: rejected -> unchanged (110), accepted -> "
               "set; a bad fence refused at once; the fence is kept on the SSD (restart)",
               ask3 is not None and ack3 and not ack3["ok"] and ack3.get("code") == 110 and same and ask4 is not None
               and ack4 and ack4["ok"] and changed and ack5 and not ack5["ok"] and "3 corners" in ack5["text"] and
               kept and not bad,
               "question %s; rejected %s (code %s), unchanged %s; accepted %s, changed %s; bad fence %s; after a "
               "restart %s" % (ask3 and ask3["text"], ack3 and ack3.get("text"), ack3 and ack3.get("code"), same,
                               ack4 and ack4.get("text"), changed, ack5 and ack5.get("text"),
                               reread and (len(reread.poly), reread.by)))

        # GF5 the copilot's ground station shows WAITING, then the captain's answer (route + fence)
        import tkinter as tk
        import app as A
        bob.close()
        troot = tk.Tk()
        errs = []
        troot.report_callback_exception = lambda *a: errs.append(repr(a[1]))
        ap = A.App(troot, "gw:127.0.0.1:%d" % sp, login=("bob", PW["bob"]),
                   layout=os.path.join(tmp, "layout_gf.json"))

        def pump(t_, until=None):
            end = time.time() + t_
            while time.time() < end:
                troot.update()
                if until and until():
                    return True
                time.sleep(0.02)
            return bool(until and until())
        try:
            pump(8, lambda: ap.t is not None and ap.gw_status is not None)
            ap.fence_draft = [tuple(p_) for p_ in fence_around(800.0)]
            ap.fence_dirty = True
            ap.fence_send()
            waiting = bool(ap.decisions) and ap.decisions[-1]["state"] == "wait"
            ap.render(force=True)
            shown_wait = any(o.get("tag") == "decision" and A.T("ov.dec_wait", s="").split("(")[0].strip() in o["s"]
                             for o in ap.ov.ops)
            k, ask6 = next_event(alice, ("driver_ask",), 4.0)
            if ask6:
                alice.l.send_msg({"t": "driver_answer", "id": ask6["id"], "yes": True})
            ok_state = pump(5, lambda: ap.decisions[-1]["state"] == "ok")
            ap.prog["wps"] = [{"lat": a, "lon": b, "kn": 8.0, "radius": 30, "action": 0}
                              for a, b in (east_of(200), east_of(400, 100))]
            ap.charts = A.chart.Charts(os.path.join(tmp, "no_charts"))
            ap.route_dirty = True
            ap.route_send()
            k, ask7 = next_event(alice, ("driver_ask",), 4.0)
            if ask7:
                alice.l.send_msg({"t": "driver_answer", "id": ask7["id"], "yes": False})
            no_state = pump(6, lambda: ap.decisions[-1]["state"] == "no")
            ap.render(force=True)
            shown_no = any(o.get("tag") == "decision" and A.T("ov.dec_no") in o["s"] for o in ap.ov.ops)
            report("GF5 the copilot's screen 2 shows its route / fence WAITING for the captain, then ACCEPTED / REJECTED",
                   waiting and shown_wait and ask6 is not None and ok_state and ask7 is not None and no_state and
                   shown_no and ap.route_dirty and not errs,
                   "fence: waiting %s (shown %s), captain asked %s, accepted %s; route: asked %s, rejected %s (shown %s), "
                   "still 'not sent' %s; errors %s" % (waiting, shown_wait, ask6 is not None, ok_state, ask7 is not None,
                                                       no_state, shown_no, ap.route_dirty, errs[:2]))
        finally:
            ap.quit()
        beat.stop()
        alice.close()
        link.stop()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section SA --

def sa8_sar_queue_drops_counted():
    """SA8 (PF4-r2 LOW-8): Recorder.note_sar's bounded queue never drops a note silently: the oldest goes, counted +
    logged; the newest are kept."""
    import recorder as R
    fr = type("FR", (), {})()
    fr.gw = type("GW", (), {"clock": type("C", (), {"stamp": lambda _s: "2026-11-10T10:30:00Z"})()})()
    for i in range(R.SAR_Q_MAX + 5):
        R.Recorder.note_sar(fr, {"op": "add", "id": "p%d" % i})
    q = fr.sar_q
    report("SA8 PF4-r2 LOW-8: a full SAVE marker-note queue drops its oldest note COUNTED + logged (never silently), "
           "the newest kept", getattr(fr, "sar_dropped", 0) == 5 and len(q) == R.SAR_Q_MAX and
           q[0][0]["id"] == "p5" and q[-1][0]["id"] == "p%d" % (R.SAR_Q_MAX + 4),
           "dropped %s, queued %d, oldest %s" % (getattr(fr, "sar_dropped", 0), len(q), q[0][0]["id"]))


def sa7_still_worker_bounded():
    """SA7 (PF4-r2 LOW-9, review of 64521b1): sar_still uses ONE helper thread with a bounded queue (never a thread per
    message): with the writer blocked, SAR_STILL_QUEUE stills wait, the next ones are REFUSED with an ack + counted;
    the files are written one after another through a temp file + rename."""
    import base64
    import gateway as G
    tmp = tempfile.mkdtemp(prefix="gwtest_sa7_")
    gate = threading.Event()
    mid = "pt_20261110-103000_9_ab12cd"

    class St:
        id, user, role, name = 7, "bob", "HELM", "bob"

        def __init__(self):
            self.sent = []

        def send(self, o):
            self.sent.append(o)

    class Store:
        usable = True

        def folder(self, n):
            gate.wait(5.0)
            return tmp

    class Fake:
        storage = Store()
        sar = type("M", (), {"list": lambda _s: [{"id": mid}], "set_still": lambda _s, i, n: None})()
        recorder = type("Rc", (), {"note_sar": lambda _s, *a: None})()
        stations = type("X", (), {"stations": {}})()

        def audit(self, *a, **kw):
            pass

        def _sar_list_msg(self):
            return {"t": "sar_markers", "list": []}
    fk, st = Fake(), St()
    jpg = b"\xff\xd8\xff\xe0" + b"y" * 300 + b"\xff\xd9"
    before = sum(1 for t in threading.enumerate() if t.name == "sar-still")
    n_send = G.SAR_STILL_QUEUE + 4
    for _ in range(n_send):
        G.Gateway._sar_still(fk, st, {"id": mid, "jpg_b64": base64.b64encode(jpg).decode()})
        time.sleep(0.01)
    threads = sum(1 for t in threading.enumerate() if t.name == "sar-still") - before
    refused = [x for x in st.sent if not x.get("ok") and "busy" in x.get("text", "")]
    gate.set()
    t0 = time.time()
    while len(st.sent) < n_send and time.time() - t0 < 5.0:
        time.sleep(0.02)
    stored = [x for x in st.sent if x.get("ok")]
    files = sorted(os.listdir(tmp))
    report("SA7 PF4-r2 LOW-9: sar_still = ONE helper thread + a bounded queue (%d): a flood of stills starts no thread "
           "per message, the ones over the limit are refused with an ack + counted, the rest are written (temp + "
           "rename, no .tmp left)" % G.SAR_STILL_QUEUE,
           threads == 1 and 1 <= len(refused) <= 4 and getattr(fk, "sar_still_dropped", 0) == len(refused) and
           len(stored) == n_send - len(refused) and files == [mid + ".jpg"],
           "threads %d, refused %d (counted %s), stored %d, files %s" % (
               threads, len(refused), getattr(fk, "sar_still_dropped", 0), len(stored), files))


def sa5_no_file_work_on_loop():
    """SA5 (PF4-r MED-5, the 13n4b rule): the gateway's sar_still writes its file in a HELPER THREAD (never on the
    calling = asyncio loop thread) and Recorder.note_sar only QUEUES (returns at once even while Recorder.lock is held
    by the worker); the worker's drain adds the note to the open event and writes the copy WITHOUT holding the lock."""
    import base64
    import builtins
    import gateway as G
    import recorder as R
    tmp = tempfile.mkdtemp(prefix="gwtest_sa5_")
    seen = []
    real_open = builtins.open
    lock = threading.Lock()

    def spy(path, *a, **kw):
        if str(path).endswith((".jpg", ".jpg.tmp")):        # PF4-r2: the still goes through a temp file
            seen.append((threading.current_thread().name, lock.locked()))
        return real_open(path, *a, **kw)
    try:
        mid = "pt_20261109-103000_9_ab12cd"

        class St:
            id, user, role, name = 7, "bob", "HELM", "bob"
            sent = []

            def send(self, o):
                self.sent.append(o)

        class Fake:
            storage = type("S", (), {"usable": True, "folder": lambda _s, n: tmp})()
            sar = type("M", (), {"list": lambda _s: [{"id": mid}], "set_still": lambda _s, i, n: None})()
            recorder = type("Rc", (), {"note_sar": lambda _s, *a: None})()
            stations = type("X", (), {"stations": {}})()

            def audit(self, *a, **kw):
                pass

            def _sar_list_msg(self):
                return {"t": "sar_markers", "list": []}
        st = St()
        jpg = b"\xff\xd8\xff\xe0" + b"x" * 500 + b"\xff\xd9"
        builtins.open = spy
        try:
            G.Gateway._sar_still(Fake(), st, {"id": mid, "jpg_b64": base64.b64encode(jpg).decode()})
            t0 = time.time()
            while not st.sent and time.time() - t0 < 3.0:
                time.sleep(0.02)
            gw_threads = [n for n, _l in seen]
            # Recorder.note_sar while the worker holds the lock: it must return at once (a queue, no file work)
            seen.clear()
            fr = type("FR", (), {})()
            fr.lock = lock
            fr.gw = type("GW", (), {"clock": type("C", (), {"stamp": lambda _s: "2026-11-09T10:30:00Z"})()})()
            os.makedirs(os.path.join(tmp, "ev"), exist_ok=True)
            ev = {"id": "e1", "path": os.path.join(tmp, "ev"), "state": "open"}
            fr.events = [ev]
            lock.acquire()
            th = threading.Thread(target=R.Recorder.note_sar, args=(fr, {"op": "still", "id": mid}, jpg, mid + ".jpg"))
            t1 = time.time()
            th.start()
            th.join(1.0)
            quick = not th.is_alive() and time.time() - t1 < 0.5 and not seen
            lock.release()
            th.join(2.0)
            R.Recorder._drain_sar(fr)
            drained = (len(ev.get("sar") or []) == 1 and os.path.exists(os.path.join(ev["path"], "sar_" + mid + ".jpg"))
                       and seen and not any(locked for _n, locked in seen))
        finally:
            builtins.open = real_open
        report("SA5 PF4-r MED-5: sar_still writes its file in a helper thread (never on the loop's thread), the ack "
               "follows; Recorder.note_sar only queues (returns at once with Recorder.lock held), the worker's drain "
               "adds it to the open SAVE event and writes the copy without the lock",
               st.sent and st.sent[-1].get("ok") and gw_threads and threading.current_thread().name not in gw_threads
               and "sar-still" in gw_threads and quick and drained,
               "ack %s; file written by %s; note_sar quick %s; drained %s" % (
                   st.sent[-1:] if st.sent else None, gw_threads, quick, ev.get("sar")))
        # PF4-r LOW: a delete is said explicitly (the stations never take a missing point for a deleted one)
        import sarstore as SAR
        ss = SAR.SarStore(os.path.join(tmp, "state", "sar.json"))
        ss.apply("add", {"id": "pt_x_1", "kind": "point", "lat": 60.0, "lon": 25.0}, "a")
        ss.apply("delete", {"id": "pt_x_1"}, "a")
        ss.flush()
        ss2 = SAR.SarStore(os.path.join(tmp, "state", "sar.json"))
        report("SA6 PF4-r LOW: a deleted marker is listed in the store's \"deleted\" list (kept in the state file) - "
               "sar_markers says it, a station deletes ONLY those", ss.deleted == ["pt_x_1"] and ss2.deleted == ["pt_x_1"]
               and "deleted" in G.Gateway._sar_list_msg(type("F", (), {"sar": ss})()), "deleted %s / reloaded %s" % (
                   ss.deleted, ss2.deleted))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def section_sa():
    """Search and rescue markers + the MOB return route (CLAUDE.md X, step PF3): the gateway's marker store (add /
    move / close / delete, idempotent, kept on the SSD), a VIEWER refused (128), every station gets the list, and the
    stand-off route to a person goes through the SAME checks as a hand route (shallow 114, a copilot's ACCEPT)."""
    print("\n=== Section SA: SAR markers, the MOB stand-off route through the normal checks ===", flush=True)
    import sarstore as SAR
    import sar_geo
    tmp = tempfile.mkdtemp(prefix="gwtest_sa_")
    sim = gw = None
    clients = []
    try:
        path = os.path.join(tmp, "state", "sar_markers.json")
        st = SAR.SarStore(path)
        mk = {"id": "mob_20261109-100000_1", "kind": "mob", "lat": 60.0, "lon": 25.0, "t_utc": "2026-11-09T10:00:00Z",
              "r_m": 5.0}
        a1 = st.apply("add", mk, "alice")
        a2 = st.apply("add", mk, "alice")                      # the same id again = idempotent
        rev1 = st.rev
        a3 = st.apply("add", dict(mk, id="mob_b", n=None), "bob")
        bad = [st.apply("add", dict(mk, id="x y"), "a")[0], st.apply("add", dict(mk, id="c1", lat=95), "a")[0],
               st.apply("add", dict(mk, id="c2", lon=float("nan")), "a")[0],
               st.apply("add", dict(mk, id="c3", kind="bomb"), "a")[0], st.apply("move", {"id": "nope"}, "a")[0]]
        mv = st.apply("move", {"id": mk["id"], "lat": 60.001, "lon": 25.001}, "alice")
        cl = st.apply("close", {"id": mk["id"], "why": "found", "t_utc": "2026-11-09T10:05:00Z"}, "alice")
        cl2 = st.apply("close", {"id": mk["id"]}, "alice")
        st.flush()
        again = SAR.SarStore(path)
        kept = [m for m in again.list() if m["id"] == mk["id"]]
        n_b = st.markers["mob_b"]["n"]
        dl = st.apply("delete", {"id": "mob_b"}, "bob")
        report("SA1 marker store: add (idempotent: the same id changes nothing), numbered per kind, bad id / position / "
               "kind refused, move, close (time + who kept, closing twice harmless), delete; a restart keeps the file",
               a1 == (True, mk["id"], "added") and a2[2] == "same" and st.rev >= rev1 and a3[0] and
               n_b == 2 and not any(bad) and mv[0] and cl[2] == "closed" and cl2[2] == "same" and
               len(kept) == 1 and abs(kept[0]["lat"] - 60.001) < 1e-9 and kept[0]["closed"]["why"] == "found" and
               kept[0]["closed"]["by"] == "alice" and dl[2] == "deleted",
               "add %s; again %s; bad %s; close %s; kept %s; rev %s/%s" % (a1, a2[2], bad, cl, bool(kept), again.rev,
                                                                           st.rev))

        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts)
        cfg = gw_cfg_b(sim, root, charts_dir=charts, stations={"driver_timeout_s": 6.0},
                       shallow={"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        clients.append(alice)
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        beat = DriverBeat(alice)
        bob = Client(sp, "bob", PW["bob"], name="copilot")
        vic = Client(sp, "vic", PW["vic"], name="viewer")
        clients += [bob, vic]
        bob.pump(1.0)
        vic.pump(0.5)

        # SA2 a crew station adds a marker: acknowledged, every station gets the list, audited, in the status; a
        # VIEWER is refused (128); sar_get gives the list again; no ACCEPT is asked (it changes no driving)
        m1 = {"id": "mob_20261109-101500_1", "kind": "mob", "lat": SIM_LAT + 0.0004, "lon": SIM_LON, "r_m": 4.0,
              "t_utc": "2026-11-09T10:15:00Z", "person": True}
        bob.l.send_msg({"t": "sar_mark", "op": "add", "m": m1})
        k1, ack = next_event(bob, ("sar_ack",), 4.0)
        k2, lst_a = next_event(alice, ("sar_markers",), 4.0)
        k3, lst_b = next_event(bob, ("sar_markers",), 4.0)
        vic.l.send_msg({"t": "sar_mark", "op": "add", "m": dict(m1, id="mob_viewer")})
        k4, nack = next_event(vic, ("sar_ack",), 4.0)
        alice.l.send_msg({"t": "sar_get"})
        k5, lst_c = next_event(alice, ("sar_markers",), 4.0)
        alice.pump(1.5)
        asked = next_event(alice, ("driver_ask",), 0.3)[1]
        sar_st = (alice.status or {}).get("sar") or {}
        aud = [e for e in g.audit.recent if e["kind"] == "sar"]
        report("SA2 a marker from a copilot: acknowledged, every station gets the list, audited (kind sar), in the "
               "status; a VIEWER is refused (128) and changes nothing; sar_get; no captain's ACCEPT is asked",
               ack and ack["ok"] and lst_a and lst_b and lst_a["list"][0]["id"] == m1["id"] and
               lst_b["rev"] == lst_a["rev"] and nack and not nack["ok"] and nack["code"] == 128 and
               "mob_viewer" not in g.sar.markers and lst_c and len(lst_c["list"]) == 1 and asked is None and
               sar_st.get("n_open") == 1 and aud and aud[-1]["user"] == "bob" and aud[-1]["op"] == "add",
               "ack %s; alice %s; bob %s; viewer %s; sar_get %s; asked %s; status %s; audit %s" % (
                   ack and ack["text"], lst_a and len(lst_a["list"]), lst_b and len(lst_b["list"]),
                   nack and nack["code"], lst_c and len(lst_c["list"]), asked, sar_st, len(aud)))

        # SA3 the stand-off route to the person = a hand route to the checks: deep water accepted (last point D before
        # the person, PROG_END_STOP), toward the shoal refused (114), a copilot's waits for the captain's ACCEPT (110)
        def route_frames(marker_m, north=0.0):
            boat = (SIM_LAT, SIM_LON)
            mark = east_of(marker_m, north)
            r = sar_geo.standoff_route(boat, mark, None, {"standoff_m": 30.0, "approach_kn": 5.0})
            return r, mark, progmod.upload_frames(sar_geo.to_program(r))
        r_ok, mark_ok, fr_ok = route_frames(600.0)
        for t_, p_ in fr_ok[:-1]:
            alice.l.send(t_, p_)
        alice.pump(0.3)
        a_ok = alice.cmd(fr_ok[-1][0], fr_ok[-1][1])
        d_end = sar_geo.distance_bearing(r_ok["end"], mark_ok)[0]
        r_sh, mark_sh, fr_sh = route_frames(2000.0)
        for t_, p_ in fr_sh[:-1]:
            alice.l.send(t_, p_)
        alice.pump(0.3)
        a_sh = alice.cmd(fr_sh[-1][0], fr_sh[-1][1])
        r_cp, mark_cp, fr_cp = route_frames(500.0, 100.0)
        for t_, p_ in fr_cp[:-1]:
            bob.l.send(t_, p_)
        bob.pump(0.3)
        seq = bob.l.send(fr_cp[-1][0], fr_cp[-1][1], what="program upload (2 waypoints)")
        k6, ask = next_event(alice, ("driver_ask",), 4.0)
        if ask:
            alice.l.send_msg({"t": "driver_answer", "id": ask["id"], "yes": False})
        r_no = bob.result(bob.wait_ack(seq, 6.0))
        inside = sar_geo.standoff_route((SIM_LAT, SIM_LON), east_of(25.0), None, {"standoff_m": 30.0})
        res_ok, res_sh = alice.result(a_ok), alice.result(a_sh)
        report("SA3 the MOB stand-off route goes through the NORMAL checks: deep water accepted (the last point 30 m "
               "before the person, final action STOP), a person on the shoal refused (114), a copilot's waits for the "
               "captain's ACCEPT (rejected -> 110); inside the stand-off distance = no route at all",
               res_ok == 0 and abs(d_end - 30.0) < 0.5 and r_ok["final"] == 0 and res_sh == 114 and ask is not None
               and r_no == 110 and inside.get("why") == "inside" and "wps" not in inside,
               "deep %s (end %.1f m before); shoal %s; copilot asked %s -> %s; inside %s" % (
                   res_ok, d_end, res_sh, bool(ask), r_no, inside.get("why")))
        # SA4 (PF4) a point marked from the video: the marker (kind point, ESTIMATE, uncertainty, source camera) is shared
        # to every station, its small still image (<= 40 KB JPEG) is stored on the SSD and linked to the marker, a
        # VIEWER / a bad image / an unknown marker / an oversize image is refused, and it asks nobody
        import base64
        pt = {"id": "pt_20261109-103000_1_ab12cd", "kind": "point", "n": 1, "lat": SIM_LAT + 0.0003, "lon": SIM_LON,
              "r_m": 22.5, "est": True, "t_utc": "2026-11-09T10:30:00Z", "person": False,
              "src": {"station": 2, "camera": "FRONT-1", "u": 0.5, "v": 0.6, "range_m": 120.0, "brg": 10.0}}
        bob.l.send_msg({"t": "sar_mark", "op": "add", "m": pt})
        k7, ack_pt = next_event(bob, ("sar_ack",), 4.0)
        jpg = b"\xff\xd8\xff\xe0" + b"x" * 3000 + b"\xff\xd9"
        bob.l.send_msg({"t": "sar_still", "id": pt["id"], "jpg_b64": base64.b64encode(jpg).decode()})
        k8, ack_st = next_event(bob, ("sar_ack",), 4.0)
        k9, lst_s = next_event(alice, ("sar_markers",), 4.0)
        while lst_s and not any(x.get("still") for x in lst_s["list"]):
            k9, lst_s = next_event(alice, ("sar_markers",), 2.0)
        vic.l.send_msg({"t": "sar_still", "id": pt["id"], "jpg_b64": base64.b64encode(jpg).decode()})
        k10, v_st = next_event(vic, ("sar_ack",), 4.0)
        bob.l.send_msg({"t": "sar_still", "id": pt["id"], "jpg_b64": base64.b64encode(b"GIF89a" + b"x" * 50).decode()})
        k11, bad_st = next_event(bob, ("sar_ack",), 4.0)
        bob.l.send_msg({"t": "sar_still", "id": "pt_unknown", "jpg_b64": base64.b64encode(jpg).decode()})
        k12, unk_st = next_event(bob, ("sar_ack",), 4.0)
        big = b"\xff\xd8" + b"x" * (40 * 1024 + 5)
        bob.l.send_msg({"t": "sar_still", "id": pt["id"], "jpg_b64": base64.b64encode(big).decode()})
        k13, big_st = next_event(bob, ("sar_ack",), 6.0)
        stored = os.path.join(root, "sar", pt["id"] + ".jpg")
        got = open(stored, "rb").read() if os.path.exists(stored) else b""
        mk_pt = [x for x in g.sar.list() if x["id"] == pt["id"]]
        asked2 = next_event(alice, ("driver_ask",), 0.3)[1]
        report("SA4 a point marked from the video: the marker (point, ESTIMATE, r_m, source camera) is shared, its small "
               "still image is stored on the SSD and linked to the marker; a VIEWER (128), a non-JPEG, an unknown "
               "marker and an oversize image are refused; nobody is asked",
               ack_pt and ack_pt["ok"] and ack_st and ack_st["ok"] and got == jpg and mk_pt and
               mk_pt[0]["still"] == pt["id"] + ".jpg" and mk_pt[0]["est"] is True and mk_pt[0]["r_m"] == 22.5 and
               mk_pt[0]["src"]["camera"] == "FRONT-1" and lst_s and v_st and v_st["code"] == 128 and bad_st and
               not bad_st["ok"] and unk_st and not unk_st["ok"] and big_st and not big_st["ok"] and asked2 is None,
               "add %s; still %s; stored %s B; marker %s; viewer %s; gif %s; unknown %s; big %s" % (
                   ack_pt and ack_pt["ok"], ack_st and ack_st["text"], len(got), mk_pt and mk_pt[0]["still"],
                   v_st and v_st["code"], bad_st and bad_st["text"], unk_st and unk_st["text"],
                   big_st and big_st["text"]))
        sa5_no_file_work_on_loop()
        sa7_still_worker_bounded()
        sa8_sar_queue_drops_counted()
        beat.on = False
    finally:
        for c_ in clients:
            try:
                c_.close()
            except Exception:
                pass
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section RL --
RL_TABLE = [
    {"n": 1, "name_el": "ΑΝΑΦΛΕΞΗ", "name_en": "IGNITION", "type": "onoff", "group": "engine", "pair": None,
     "max_s": 0, "show_hud": True, "power_up": "off", "role": "ignition"},
    {"n": 4, "name_el": "ΤΡΙΜ ΠΑΝΩ", "name_en": "TRIM UP", "type": "momentary", "group": "trim", "pair": 5,
     "max_s": 3.0, "show_hud": True, "power_up": "off", "role": None},
    {"n": 5, "name_el": "ΤΡΙΜ ΚΑΤΩ", "name_en": "TRIM DOWN", "type": "momentary", "group": "trim", "pair": 4,
     "max_s": 3.0, "show_hud": True, "power_up": "off", "role": None},
    {"n": 6, "name_el": "ΦΩΤΑ", "name_en": "NAV LIGHTS", "type": "onoff", "group": "extra", "pair": None,
     "max_s": 0, "show_hud": False, "power_up": "keep", "role": None}]


def section_rl():
    """Relay buttons (CLAUDE.md O): a copilot's ENGINE-group relay waits for the captain's ACCEPT, a trim / extra relay
    goes straight - the group from the relay table the AURIX reports ACTIVE (TELEM2 group M hash), learned by the
    gateway from the table a station sent (kept on the SSD); unknown table = every relay = ACCEPT (fail safe)."""
    print("\n=== Section RL: relay buttons - engine relays need the captain's ACCEPT, trim / extra go straight ===",
          flush=True)
    import relaytab
    pay = proto.p_relay_cfg(RL_TABLE)
    h = proto.relay_table_hash(RL_TABLE)
    grp = proto.relay_groups(pay)
    unit = grp == {1: "engine", 4: "trim", 5: "trim", 6: "extra"} and \
        proto.is_driving_cmd(proto.T_RELAY, proto.p_relay(1, 1), grp) and \
        not proto.is_driving_cmd(proto.T_RELAY, proto.p_relay(6, 1), grp) and \
        not proto.is_driving_cmd(proto.T_RELAY, proto.p_relay(4, 2, 3000), grp) and \
        proto.is_driving_cmd(proto.T_RELAY, proto.p_relay(6, 1), None) and \
        proto.is_driving_cmd(proto.T_RELAY, proto.p_relay(9, 1), grp) and \
        proto.is_driving_cmd(proto.T_RELAY_CFG, pay, grp) and proto.relay_groups(pay[:-1]) is None
    # a relay with an engine role but a non-engine group byte still counts as engine
    odd = bytearray(pay)
    odd[1 + 8 * 3 + 2] = 2
    odd[1 + 8 * 3 + 7] = 2
    unit = unit and proto.relay_groups(bytes(odd))[6] == "engine"

    tmp = tempfile.mkdtemp(prefix="gwtest_rl_")
    sim = gw = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root, stations={"driver_timeout_s": 6.0})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        beat = DriverBeat(alice)
        bob = Client(sp, "bob", PW["bob"], name="copilot")
        bob.pump(1.0)
        # the captain's relay table: own action (no question); a new table waits for the joystick CONFIRM on board
        known0 = g.stations.relay_status()["known"]
        r_cfg = alice.result(alice.cmd(proto.T_RELAY_CFG, pay))
        time.sleep(0.5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.3)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        active = until(lambda: g.telem2.get("relay_table") and g.telem2.get("relay_hash") == h and
                       not g.telem2.get("relay_pending"), 8)
        known1 = g.stations.relay_status()
        # RL1 the copilot: extra (NAV LIGHTS ON) and trim (TRIM UP pulse) straight; IGNITION waits for the ACCEPT
        n_ask0 = len(g.stations.asks)
        a_extra = bob.cmd(proto.T_RELAY, proto.p_relay(6, proto.RELAY_CMD_ON), timeout=4.0)
        a_trim = bob.cmd(proto.T_RELAY, proto.p_relay(4, proto.RELAY_CMD_PULSE, 1000), timeout=4.0)
        lit = until(lambda: 6 in (g.telem2.get("relay_on") or []), 4)
        no_ask = len(g.stations.asks) == n_ask0
        seq = bob.l.send(proto.T_RELAY, proto.p_relay(1, proto.RELAY_CMD_ON), what="relay IGNITION ON")
        k, ask = next_event(alice, ("driver_ask",), 4.0)
        if ask:
            alice.l.send_msg({"t": "driver_answer", "id": ask["id"], "yes": False})
        r_eng = bob.result(bob.wait_ack(seq, 6.0))
        audit = [e for e in g.audit.recent if e.get("kind") == "copilot_cmd" and e.get("type") == proto.T_RELAY]
        report("RL1 relay switches through the gateway: the copilot's NAV LIGHTS (extra) and TRIM UP (trim) go straight "
               "to the AURIX on the captain's behalf (audited, the relay on in the readback), its IGNITION (engine) waits "
               "for the captain's ACCEPT (rejected -> 110); the table itself = the captain's own (no question)",
               unit and r_cfg == 0 and active and not known0 and known1["known"] and known1["hash"] == h and
               bob.result(a_extra) == 0 and bob.result(a_trim) == 0 and lit and no_ask and ask is not None and
               "IGNITION" in ask["text"] and r_eng == 110 and len(audit) >= 2,
               "proto groups %s / rules %s; table -> %s, active %s, gateway knows it before %s / after %s; extra %s, "
               "trim %s, lit %s, no question %s; engine: asked %s, rejected -> %s; audited %d" % (
                   grp, unit, r_cfg, bool(active), known0, known1, bob.result(a_extra), bob.result(a_trim),
                   bool(lit), no_ask, ask and ask["text"], r_eng, len(audit)))

        # RL2 the gateway's memory: a restart reads the table back from the SSD; a table it doesn't know (a new
        # SSD) = every relay is held for the captain's ACCEPT, and the status says so
        path = g.stations.relay_tables.path
        reread = relaytab.RelayTables(path).active(g.telem2) if path else None
        saved = g.stations.relay_tables.tables
        g.stations.relay_tables.tables = {}
        status_unknown = g.stations.relay_status()
        seq = bob.l.send(proto.T_RELAY, proto.p_relay(6, proto.RELAY_CMD_OFF), what="relay NAV LIGHTS OFF")
        k, ask2 = next_event(alice, ("driver_ask",), 4.0)
        if ask2:
            alice.l.send_msg({"t": "driver_answer", "id": ask2["id"], "yes": True})
        r_unknown = bob.result(bob.wait_ack(seq, 6.0))
        off = until(lambda: 6 not in (g.telem2.get("relay_on") or []), 4)
        g.stations.relay_tables.tables = saved
        broken = relaytab.RelayTables(None)
        bad = broken.learn(b"\x02\x01") is None and broken.active(g.telem2) is None
        report("RL2 the gateway keeps the relay groups on the SSD (a restart knows the active table); a table it doesn't "
               "know = every relay waits for the captain's ACCEPT (here accepted -> NAV LIGHTS off), status known "
               "false; a broken table frame is never learned",
               reread == grp and not status_unknown["known"] and ask2 is not None and r_unknown == 0 and off and bad,
               "after a restart %s; unknown: status %s, asked %s, accepted -> %s, off %s; broken frame ignored %s" % (
                   reread, status_unknown, ask2 and ask2["text"], r_unknown, bool(off), bad))
        beat.stop()
        alice.close()
        bob.close()
        link.stop()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------ section DP --

DP_THRUSTERS = {"bow": {"max_pct": 100, "run_s": 1800, "rest_s": 600},
                "stern": {"max_pct": 100, "run_s": 1800, "rest_s": 600}, "jog_m": 1.5, "spot_alarm_m": 15}


def section_dp():
    """CLAUDE.md P, step 11 (+ 11b DP4): a DP TRIP is checked by the gateway like AUTO - shallow water (114), the geofence
    (120), collision avoidance (the navreq cap / heading applied in TRIP); in DP SPOT / JOG / CRUISE a collision
    danger = the DRIVER DECISION alarm only."""
    print("\n=== Section DP: DP TRIP checked like AUTO (shallow 114, geofence 120, collision avoidance) ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_dp_")
    sim = gw = asim = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts, n_lat=400, n_lon=400)             # deep water, a 1.5 m shoal 2 km east of the boat
        aport = free_port()
        asim = AisSim(aport, seed=7, noise_m=5.0, start_offset=0.2).start()
        cfg = gw_cfg_b(sim, root, charts_dir=charts, avoid=CA_AVOID, stations={"driver_timeout_s": 5.0},
                       shallow={"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0},
                       ais={"enabled": True, "source": "tcp:127.0.0.1:%d" % aport, "silent_s": 600.0, "tick_s": 0.25})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        beat = DriverBeat(alice)                            # the captain's screen 1 is open

        def joy(x=0.0, y=0.0, **kw):
            link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, x, y, **kw))

        def dp(action, arg=0, value=0):
            return alice.result(alice.cmd(proto.T_DP, proto.p_dp(action, arg, value)))

        def confirm():
            joy(confirm=True)
            time.sleep(0.2)
            joy()

        w.until(lambda d: d.get("pos_source", 0) != 0, 10)
        r_thr = alice.result(alice.cmd(proto.T_THRUSTER_CFG, proto.p_thruster_cfg(DP_THRUSTERS)))
        thr = w.until(lambda d: d.get("thr_cfg") and d.get("thr_fitted_bow") and d.get("thr_fitted_stern"), 4)

        # DP1 a DP TRIP whose way from the boat crosses the shoal: refused (114) before the boat is even asked;
        # "continue" and the same program's AUTO start too; a DP SPOT start is not a route (passed on: it waits)
        a_one = upload(alice, [east_of(3500)], kn=3.0)                          # one waypoint behind the shoal
        n0 = len(g.audit.recent)
        r_trip = dp(proto.DP_START, proto.DP_TRIP, 300)
        r_cont = dp(proto.DP_START, proto.DP_TRIP | proto.DP_TRIP_CONTINUE, 300)
        w.pump(0.5)
        pend1 = w.telem.get("dp_pending")
        r_auto = alice.result(alice.cmd(proto.T_AUTO_START))
        aud1 = [e for e in list(g.audit.recent)[n0:] if e["kind"] == "dp_trip_refused"]
        r_spot = dp(proto.DP_START, proto.DP_SPOT, 0)
        pend_spot = w.until(lambda d: d.get("dp_pending"), 2)
        r_stop = dp(proto.DP_STOP)                                              # cancels the waiting request
        unpend = w.until(lambda d: not d.get("dp_pending"), 2)
        report("DP1 a DP TRIP across shallow water refused by the gateway (114, audited, nothing waits on the boat), "
               "\"continue\" too; a DP SPOT start (no route) is passed on",
               r_thr == 0 and thr is not None and a_one and a_one["result"] == 0 and r_trip == 114 and r_cont == 114
               and not pend1 and r_auto == 114 and len(aud1) == 2 and r_spot == 0 and pend_spot is not None and
               r_stop == 0 and unpend is not None,
               "thruster settings %s (taken %s); program %s; TRIP %s, continue %s, waiting on the boat %s; AUTO start "
               "%s; audited %d; SPOT %s (waits for the CONFIRM %s), STOP %s (cancelled %s)" % (
                   r_thr, thr is not None, a_one and a_one["result"], r_trip, r_cont, pend1, r_auto, len(aud1),
                   r_spot, pend_spot is not None, r_stop, unpend is not None))

        # DP2 the geofence: a program in deep water, then a fence that leaves its waypoint outside -> a DP TRIP
        # start refused (120); the fence cleared -> passed on (the CONFIRM then waits)
        a_two = upload(alice, [east_of(1500)], kn=3.0)
        alice.l.send_msg({"t": "geofence", "poly": [list(p_) for p_ in fence_around(1000.0)],
                          "what": "geofence: 4 corners"})
        k, ack_f = next_event(alice, ("geofence_ack",), 4.0)
        r_fence = dp(proto.DP_START, proto.DP_TRIP, 300)
        alice.l.send_msg({"t": "geofence", "poly": [], "what": "geofence: clear"})
        k, ack_c = next_event(alice, ("geofence_ack",), 4.0)
        r_free = dp(proto.DP_START, proto.DP_TRIP, 300)
        pend2 = w.until(lambda d: d.get("dp_pending"), 2)
        dp(proto.DP_STOP)
        w.until(lambda d: not d.get("dp_pending"), 2)
        report("DP2 a DP TRIP that leaves the geofence refused (120); without the fence the same TRIP start goes "
               "to the boat (waits for the CONFIRM)",
               a_two and a_two["result"] == 0 and ack_f and ack_f["ok"] and r_fence == 120 and ack_c and ack_c["ok"]
               and r_free == 0 and pend2 is not None,
               "program %s; fence %s; TRIP %s; fence cleared %s; TRIP %s, waiting %s" % (
                   a_two and a_two["result"], ack_f and ack_f.get("text"), r_fence, ack_c and ack_c.get("text"),
                   r_free, pend2 is not None))

        # DP3 collision avoidance: in a DP TRIP like AUTO (slow down = the navreq cap, applied by the boat); in
        # DP SPOT the same danger = the DRIVER DECISION alarm only (no cap, no heading, DP keeps its spot)
        wp = east_of(0, 3000)                                                   # due north, deep water
        a_n = upload(alice, [wp], kn=3.0)
        r_go = dp(proto.DP_START, proto.DP_TRIP, 300)
        alice.pump(0.3)
        confirm()
        trip = w.until(lambda d: d["mode"] == proto.MODE_DP and d.get("dp_sub") == proto.DP_TRIP, 4)
        moving = w.until(lambda d: d.get("est_sog", 0) > 1.2, 90)              # (DP TRIP: <= 3 kn)
        d0 = w.telem
        n_log = len(g.audit.recent)
        asim.add(crossing_ship(d0, 239400011, 8.0, 150.0, "DP CROSSER"))
        slow = w.until(lambda d: d.get("nav_applied") and d.get("nav_kind") in (proto.NAV_CAP, proto.NAV_HEADING)
                       and d.get("nav_cap_kn") is not None, 40)
        st_trip = g.nav_state.get("state")
        texts = avoid_log(g, n_log)
        asim.remove(239400011)
        g.ais.targets.pop(239400011, None)
        r_hold = dp(proto.DP_START, proto.DP_SPOT, 0)                           # in DP: SPOT here at once
        spot = w.until(lambda d: d["mode"] == proto.MODE_DP and d.get("dp_sub") == proto.DP_SPOT, 4)
        clear = wait_for(lambda: g.nav_state.get("state") == "CLEAR", 30)
        w.until(lambda d: d.get("est_sog", 9) < 0.8, 60)
        d1 = w.telem
        s_lat, s_lon = aismod.move(d1["est_lat"], d1["est_lon"], 90.0, 10.0, 90.0)   # 0.25 nm east, at us at 10 kn
        asim.add(Ship(239400012, s_lat, s_lon, 10.0, 270.0, name="DP SPOT VISITOR"))
        dec = w.until(lambda d: d["alarm_bits"] & DECIS_BIT and d["alarm_bits"] & COLL_BIT, 30)
        st_spot = g.nav_state.get("state")
        w.pump(1.0)
        d2 = w.telem
        asim.remove(239400012)
        g.ais.targets.pop(239400012, None)
        r_end = dp(proto.DP_STOP)
        stby = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 6)
        report("DP3 collision avoidance in a DP TRIP like AUTO (the slow-down cap applied by the boat, audited); in DP "
               "SPOT a danger = COLLISION + DRIVER DECISION alarm only (nothing applied, still SPOT); DP STOP -> STANDBY",
               a_n and a_n["result"] == 0 and r_go == 0 and trip is not None and moving is not None and
               slow is not None and slow["mode"] == proto.MODE_DP and slow.get("dp_sub") == proto.DP_TRIP and
               slow["nav_cap_kn"] <= 3.0 and st_trip in ("SLOW", "AVOID", "DRIVER DECISION") and texts and
               r_hold == 0 and spot is not None and dec is not None and st_spot == "DRIVER DECISION" and
               d2["mode"] == proto.MODE_DP and d2.get("dp_sub") == proto.DP_SPOT and not d2.get("nav_applied") and
               d2.get("nav_kind") == proto.NAV_NONE and r_end == 0 and stby is not None,
               "TRIP %s (start %s), moving %s (%.1f kn, %03.0f); crossing ship -> request applied %s (kind %s, cap %s kn, planner %s, "
               "%d decisions logged); SPOT %s (cleared %s); ship at the spot -> alarms %s, planner %s, mode %s/%s, "
               "applied %s, kind %s; STOP %s -> STANDBY %s" % (
                   trip is not None, r_go, moving is not None, d0.get("est_sog", 0), d0.get("est_cog", 0),
                   slow is not None, slow and slow.get("nav_kind"),
                   slow and slow.get("nav_cap_kn"), st_trip, len(texts), spot is not None, clear,
                   "yes" if dec else "NO", st_spot, d2["mode_name"], d2.get("dp_sub_name"), d2.get("nav_applied"),
                   d2.get("nav_kind"), r_end, stby is not None))

        # DP4 (step 11b) a program the gateway did not check can't start a DP TRIP: loaded over USB (also over
        # a program the gateway checked before: the copy is stale), or the gateway's copy lost (restart) -> 121;
        # uploaded through the gateway -> checked and passed on
        def usb_upload(wps, kn=3.0):
            # the laptop on the USB cable (on board) takes HELM, uploads, gives HELM back; the captain takes it again
            prog = {"final": 0, "wps": [{"lat": a, "lon": b, "kn": kn, "radius": 30, "action": 0} for a, b in wps]}
            link.send(proto.T_HELM, proto.p_helm(0))
            time.sleep(0.2)
            for t_, p_ in progmod.upload_frames(prog):
                link.send(t_, p_)
                time.sleep(0.05)
            time.sleep(0.2)
            link.send(proto.T_HELM, proto.p_helm(1))
            time.sleep(0.2)
            alice.cmd(proto.T_HELM, proto.p_helm(0))
            alice.pump(0.3)
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY and d.get("est_sog", 9) < 1.0, 30)
        usb_upload([east_of(1200)])
        usb = w.until(lambda d: d["prog_n"] == 1 and not d.get("prog_gw"), 4)
        n4 = len(g.audit.recent)
        r_usb = dp(proto.DP_START, proto.DP_TRIP, 300)
        r_usb_c = dp(proto.DP_START, proto.DP_TRIP | proto.DP_TRIP_CONTINUE, 300)
        aud4 = [e for e in list(g.audit.recent)[n4:] if e["kind"] == "dp_trip_refused"]
        a_gw = upload(alice, [east_of(1200)], kn=3.0)
        viagw = w.until(lambda d: d["prog_n"] == 1 and d.get("prog_gw"), 4)
        r_gw = dp(proto.DP_START, proto.DP_TRIP, 300)
        pend4 = w.until(lambda d: d.get("dp_pending"), 2)
        dp(proto.DP_STOP)
        w.until(lambda d: not d.get("dp_pending"), 2)
        usb_upload([east_of(1300)])                                              # same count, other route
        stale = w.until(lambda d: d["prog_n"] == 1 and not d.get("prog_gw"), 4)
        r_stale = dp(proto.DP_START, proto.DP_TRIP, 300)
        a_gw2 = upload(alice, [east_of(1200)], kn=3.0)
        w.until(lambda d: d.get("prog_gw"), 4)
        saved, g.prog_wps = g.prog_wps, None                                    # the gateway restarted: no copy
        r_lost = dp(proto.DP_START, proto.DP_TRIP, 300)
        g.prog_wps = saved
        report("DP4 a DP TRIP with a program the gateway did not check is refused (121): loaded over USB, a stale "
               "copy after a USB upload, the copy lost; uploaded through the gateway -> checked and passed on",
               usb is not None and r_usb == 121 and r_usb_c == 121 and len(aud4) == 2 and a_gw and a_gw["result"] == 0
               and viagw is not None and r_gw == 0 and pend4 is not None and stale is not None and r_stale == 121 and
               a_gw2 and a_gw2["result"] == 0 and r_lost == 121,
               "USB program %s: TRIP %s, continue %s (audited %d); through the gateway %s (flag %s): TRIP %s (waits %s); "
               "USB again %s: TRIP %s; gateway copy lost: TRIP %s" % (
                   usb is not None, r_usb, r_usb_c, len(aud4), a_gw and a_gw["result"], viagw is not None, r_gw,
                   pend4 is not None, stale is not None, r_stale, r_lost))

        beat.stop()
        alice.close()
        link.stop()
    finally:
        if asim:
            asim.stop()
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)



def section_ar():
    """CLAUDE.md R / D, step 13: AUTO RETURN through the gateway - the way back (the program's waypoints before the
    active one, backwards, from the boat) checked like a DP TRIP: shallow water (114), the geofence (120), only a
    program the gateway checked (121); a copilot's AUTO return goes straight (no captain's ACCEPT) - the boat still
    wants the joystick CONFIRM."""
    print("\n=== Section AR: AUTO RETURN through the gateway (checked way back, copilot without ACCEPT) ===", flush=True)
    tmp = tempfile.mkdtemp(prefix="gwtest_ar_")
    sim = gw = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        sim_chart(charts, n_lat=400, n_lon=400)             # deep water, a 1.5 m shoal 2 km east of the boat
        cfg = gw_cfg_b(sim, root, charts_dir=charts, stations={"driver_timeout_s": 3.0},
                       shallow={"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0})
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        link, w = sim.station()
        alice = Client(sp, "alice", PW["alice"], name="captain")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        bob = Client(sp, "bob", PW["bob"], name="copilot")
        bob.pump(1.0)
        w.until(lambda d: d.get("pos_source", 0) != 0, 10)

        # AR1 deep water: the captain's AUTO return is checked and passed on (the boat waits for the CONFIRM); the
        # copilot's goes straight too (no captain's screen open = no question: COPILOT_FREE, like AUTO pause)
        a_deep = upload(alice, [east_of(0, 800), east_of(0, 1600)], kn=8.0)
        viagw = w.until(lambda d: d["prog_n"] == 2 and d.get("prog_gw"), 4)
        r_cap = alice.result(alice.cmd(proto.T_AUTO_RETURN))
        pend = w.until(lambda d: d["confirm_kind"] == 10, 3)
        alice.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))           # (cancels the request)
        w.until(lambda d: d["confirm_kind"] == 0, 3)
        r_cop = bob.result(bob.cmd(proto.T_AUTO_RETURN))
        pend_c = w.until(lambda d: d["confirm_kind"] == 10, 3)
        alice.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
        w.until(lambda d: d["confirm_kind"] == 0, 3)
        r_cop_sea = bob.result(bob.cmd(proto.T_SEA_STATE, proto.p_sea_state(3)))   # a normal driving action: 109
        report("AR1 AUTO return through the gateway: checked + passed on (the boat waits for the CONFIRM); a copilot's "
               "goes straight without the captain's ACCEPT (another copilot action still waits for screen 1)",
               a_deep and a_deep["result"] == 0 and viagw is not None and r_cap == 0 and pend is not None and
               r_cop == 0 and pend_c is not None and r_cop_sea == 109,
               "program %s (through the gateway %s); captain -> %s, waiting for the CONFIRM %s; copilot -> %s, waiting "
               "%s; copilot sea state -> %s (109 = the captain's screen 1 not open)" % (
                   a_deep and a_deep["result"], viagw is not None, r_cap, pend is not None, r_cop,
                   pend_c is not None, r_cop_sea))

        # AR2 the way back crosses the shoal: refused (114, audited, nothing waits on the boat); the geofence (120)
        a_sh = upload(alice, [east_of(3500)], kn=8.0)                       # waypoint 1 behind the shoal
        w.until(lambda d: d["prog_n"] == 1 and d.get("prog_gw"), 4)
        n0 = len(g.audit.recent)
        r_sh = alice.result(alice.cmd(proto.T_AUTO_RETURN))
        w.pump(0.5)
        pend_sh = w.telem.get("confirm_kind")
        aud = [e for e in list(g.audit.recent)[n0:] if e["kind"] == "auto_return_refused"]
        a_two = upload(alice, [east_of(0, 1500)], kn=8.0)
        w.until(lambda d: d["prog_n"] == 1 and d.get("prog_gw"), 4)
        alice.l.send_msg({"t": "geofence", "poly": [list(p_) for p_ in fence_around(1000.0)],
                          "what": "geofence: 4 corners"})
        k, ack_f = next_event(alice, ("geofence_ack",), 4.0)
        r_fence = alice.result(alice.cmd(proto.T_AUTO_RETURN))
        alice.l.send_msg({"t": "geofence", "poly": [], "what": "geofence: clear"})
        next_event(alice, ("geofence_ack",), 4.0)
        report("AR2 AUTO return whose way back crosses shallow water refused (114, audited, nothing waits on the "
               "boat); leaving the geofence refused (120)",
               a_sh and a_sh["result"] == 0 and r_sh == 114 and pend_sh == 0 and len(aud) == 1 and
               a_two and a_two["result"] == 0 and ack_f and ack_f["ok"] and r_fence == 120,
               "program behind the shoal %s: return %s, waiting %s, audited %d; fence %s: return %s" % (
                   a_sh and a_sh["result"], r_sh, pend_sh, len(aud), ack_f and ack_f.get("text"), r_fence))

        # AR3 the way back the gateway checks = the waypoints before the active one, backwards, from the boat;
        # without a checked copy (USB / the boat's memory / a gateway restart) -> 121
        seen = []

        class Fake:
            pass
        f = Fake()
        f.prog_wps = [east_of(0, 500), east_of(0, 1000), east_of(0, 1500), east_of(0, 2000)]
        f.prog_ack_new = False
        f.telem = {"auto_wp": 2, "prog_gw": True, "prog_n": 4, "prog_returning": False}
        f.own = lambda: {"lat": SIM_LAT, "lon": SIM_LON + 0.01}

        class Rec:
            def check_program(self, wps, start=None):
                seen.append((list(wps), start))
                return None

            def check(self, wps, start=None):
                return None
        f.shallow = Rec()
        f.geofence = Rec()
        f.audit = lambda *a, **kw: None
        st = Fake()
        st.id, st.user = 1, "alice"
        r_mid = g.__class__.check_prog(f, st, proto.T_AUTO_RETURN, b"")
        f.telem = dict(f.telem, auto_wp=0)
        r_first = g.__class__.check_prog(f, st, proto.T_AUTO_RETURN, b"")
        f.telem = dict(f.telem, prog_gw=False)
        r_usb = g.__class__.check_prog(f, st, proto.T_AUTO_RETURN, b"")
        f.telem = dict(f.telem, prog_gw=True, prog_n=3)
        r_stale = g.__class__.check_prog(f, st, proto.T_AUTO_RETURN, b"")
        want_mid = [f.prog_wps[1], f.prog_wps[0]]
        report("AR3 the way back the gateway checks: waypoints before the active one backwards from the boat (on the "
               "first leg: waypoint 1); a program the gateway did not check (USB / stale) -> 121",
               r_mid is None and r_first is None and len(seen) == 2 and seen[0][0] == want_mid and
               seen[0][1] == (SIM_LAT, SIM_LON + 0.01) and seen[1][0] == [f.prog_wps[0]] and
               isinstance(r_usb, tuple) and r_usb[0] == 121 and isinstance(r_stale, tuple) and r_stale[0] == 121,
               "active waypoint 3 -> checked %s; first leg -> %s; USB -> %s; stale -> %s" % (
                   seen[0][0] if seen else None, seen[1][0] if len(seen) > 1 else None,
                   r_usb and r_usb[0], r_stale and r_stale[0]))

        # AR4 (step 13q2b MED-1) while the captain drives by hand (REMOTE / MANEUVER) a copilot's AUTO return is a
        # mode change out of the captain's hands: it needs the captain's ACCEPT (here no screen 1 open = 109); the
        # captain's own is passed on and the AURIX refuses it there (7 WRONG_MODE: never AUTO RETURN out of REMOTE /
        # MANEUVER); MANEUVER / no telemetry = not free either, STANDBY / AUTO = free
        stick = {"x": 0.0, "lever": 0.5}
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        alice.cmd(proto.T_REMOTE, proto.p_remote(0))
        time.sleep(1.0)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.2)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        on = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 3)
        w.until(lambda d: g.telem and g.telem.get("mode") == proto.MODE_REMOTE, 2)
        r_cop_rem = bob.result(bob.cmd(proto.T_AUTO_RETURN))
        r_cap_rem = alice.result(alice.cmd(proto.T_AUTO_RETURN))
        w.pump(0.5)
        kept = w.telem.get("mode") == proto.MODE_REMOTE and w.telem.get("confirm_kind") == 0
        alice.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
        snd.stop()
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 3)

        class StubGw:
            telem = None
            telem_rx = 0.0
            TELEM_MAX_AGE_S = 3.0
            aurix = Fake()

            def telem_fresh(self):
                return gwmod.Gateway.telem_fresh(self)
        stub = Fake()
        stub.gw = StubGw()
        stub.gw.aurix.connected = True
        free = {}
        for name, mode in (("none", None), ("maneuver", proto.MODE_MANEUVER), ("remote", proto.MODE_REMOTE),
                           ("standby", proto.MODE_STANDBY), ("auto", proto.MODE_AUTO)):
            stub.gw.telem = None if mode is None else {"mode": mode}
            stub.gw.telem_rx = time.time()
            free[name] = g.stations.__class__.copilot_free(stub, proto.T_AUTO_RETURN)
        pause_free = g.stations.__class__.copilot_free(stub, proto.T_AUTO_PAUSE)
        report("AR4 in REMOTE / MANEUVER a copilot's AUTO return needs the captain's ACCEPT; the boat refuses an "
               "AUTO return out of REMOTE (7); free only in the other modes",
               on is not None and r_cop_rem == 109 and r_cap_rem == 7 and kept and
               free == {"none": False, "maneuver": False, "remote": False, "standby": True, "auto": True} and
               pause_free,
               "REMOTE %s: copilot's AUTO return -> %s (109 = waits for the captain's screen 1), captain's -> %s (7), "
               "still REMOTE + nothing pending %s; free by mode %s, AUTO pause free %s" % (
                   on is not None, r_cop_rem, r_cap_rem, kept, free, pause_free))

        alice.close()
        bob.close()
        link.stop()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)


AL_KEY = "academy-key-0123456789abcdefghij"


def section_al():
    """Academy levels (CLAUDE.md S, ACADEMY_SPEC A, step 16): the shore PC's academy signs a levels file, a station
    hands it in, the gateway checks it (trainlevels.py), keeps its last copy on the SSD and uses it at the NEXT login
    only (never mid-trip); the user list stays the authority (unknown users ignored, no new user)."""
    print("\n=== Section AL: academy levels - signed push, last copy, next login only ===", flush=True)
    import datetime as dt
    import trainlevels as TL
    sys.path.insert(0, os.path.join(HERE, "..", "academy"))
    import accounts as ACC
    today = dt.date.today()
    later = today + dt.timedelta(days=200)
    accs = [ACC.Account("alice", "CAPTAIN", later), ACC.Account("bob", "OBSERVER"), ACC.Account("zed", "CAPTAIN", later)]
    doc = ACC.make_push_doc(accs, AL_KEY, today, seq=1000)
    # AL1 the rules on their own: signature, replay, licence = CAPTAIN only, expiry needed, expired -> OBSERVER
    v_ok, _ = TL.verify(doc, AL_KEY, 0)
    bad_key = TL.verify(doc, "another-key-0123456789", 0)[1]
    no_key = TL.verify(doc, None, 0)[1]
    replay = TL.verify(doc, AL_KEY, 1000)[1]
    t = dict(doc, records=[dict(doc["records"][1], level="CAPTAIN", licence=True, expiry=later.isoformat())])
    tampered = TL.verify(t, AL_KEY, 0)[1]
    drv = ACC.make_push_doc([ACC.Account("bob", "DRIVER", later)], AL_KEY, today, seq=5)
    drv_lic = TL.verify(drv, AL_KEY, 0)[0]["bob"]["licence"]
    odd = {k: v for k, v in drv.items() if k != "mac"}
    odd["records"] = [{"user": "bob", "level": "DRIVER", "licence": True, "expiry": later.isoformat()}]
    odd["mac"] = TL.sign(odd, AL_KEY)
    odd_why = TL.verify(odd, AL_KEY, 0)[1]
    nexp = {k: v for k, v in drv.items() if k != "mac"}
    nexp["records"] = [{"user": "bob", "level": "CAPTAIN", "licence": True, "expiry": None}]
    nexp["mac"] = TL.sign(nexp, AL_KEY)
    nexp_why = TL.verify(nexp, AL_KEY, 0)[1]
    old = ACC.Account("carl", "CAPTAIN", today - dt.timedelta(days=1)).gateway_record(today)
    lv = TL.AcademyLevels(None, AL_KEY, today=lambda: today)
    lv.records = {"a": {"level": "CAPTAIN", "licence": True, "expiry": today.isoformat()},
                  "b": {"level": "CAPTAIN", "licence": True, "expiry": (today - dt.timedelta(days=1)).isoformat()},
                  "c": {"level": "DRIVER", "licence": False, "expiry": later.isoformat()}}
    lic = (lv.licence("a", False), lv.licence("b", True), lv.licence("c", True), lv.licence("n", True), lv.licence("n", False))
    report("AL1 levels file: signed with the academy key (wrong / no key / tampered refused), no replay, licence = "
           "CAPTAIN only with an expiry, an expired captain pushed as OBSERVER, the expiry checked on the gateway's "
           "clock, no record = the secrets.json flag",
           v_ok is not None and set(v_ok) == {"alice", "bob", "zed"} and "signature" in bad_key and "no academy key" in no_key
           and "older" in replay and "signature" in tampered and drv_lic is False and "licence" in odd_why and
           "expiry" in nexp_why and old == {"user": "carl", "level": "OBSERVER", "licence": False, "expiry": None} and
           lic == (True, False, False, True, False),
           "wrong key %r; no key %r; replay %r; tampered %r; DRIVER licence %s; %r; %r; expired %s; licence a/b/c/n/n %s" % (
               bad_key, no_key, replay, tampered, drv_lic, odd_why, nexp_why, old, lic))

    tmp = tempfile.mkdtemp(prefix="gwtest_al_")
    sim = gw = gw2 = None
    try:
        sim = Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = gw_cfg_b(sim, root, academy_key=AL_KEY)
        gw = GwRunner(cfg)
        g = gw.gw
        sp = cfg["stations"]["port"]
        alice = Client(sp, "alice", PW["alice"], name="captain")
        bob = Client(sp, "bob", PW["bob"], name="copilot")
        vic = Client(sp, "vic", PW["vic"], name="viewer")
        alice.pump(1.0)
        before = (alice.welcome.get("licence"), bob.welcome.get("licence"))
        # AL2 a viewer can't hand it in; the captain's station can: known users taken, the unknown one ignored
        vic.l.send_msg({"t": "academy_levels", "doc": doc})
        k, a_vic = next_event(vic, ("academy_levels_ack",), 4.0)
        alice.l.send_msg({"t": "academy_levels", "doc": doc})
        k, a_ok = next_event(alice, ("academy_levels_ack",), 4.0)
        alice.pump(1.5)
        stat = (alice.status or {}).get("academy") or {}
        audited = [e for e in g.audit.recent if e["kind"] == "academy_levels"]
        saved = os.path.isfile(os.path.join(g.storage.folder("state"), "academy_levels.json"))
        report("AL2 a viewer's push refused; the captain's station hands in the signed file: 2 known users taken, the "
               "unknown one ignored (no new user), audited, kept on the SSD, shown in the gateway status",
               a_vic is not None and not a_vic["ok"] and a_ok is not None and a_ok["ok"] and a_ok["n"] == 2 and
               a_ok["ignored"] == 1 and "zed" not in g.users.users and stat.get("n") == 2 and stat.get("seq") == 1000
               and len(audited) == 2 and saved,
               "viewer %s; captain %s; status %s; audit %d; saved %s" % (
                   a_vic and a_vic["text"], a_ok and a_ok["text"], stat, len(audited), saved))
        # AL3 never mid-trip: the logged-in stations keep their licence; the next login uses the levels
        st_bob = g.stations.stations[bob.welcome["id"]]
        mid = st_bob.licence
        bob2 = Client(sp, "bob", PW["bob"], name="copilot2")
        alice2 = Client(sp, "alice", PW["alice"], name="captain2")
        nxt = (alice2.welcome.get("licence"), bob2.welcome.get("licence"))
        alice.l.send_msg({"t": "academy_levels", "doc": doc})
        k, a_rep = next_event(alice, ("academy_levels_ack",), 4.0)
        report("AL3 never revoked mid-trip: bob's logged-in station keeps the licence; at the NEXT login alice = CAPTAIN "
               "(licence), bob = OBSERVER (no licence); the same file again = refused (replay)",
               before == (True, True) and mid is True and nxt == (True, False) and a_rep is not None and not a_rep["ok"],
               "before %s; bob logged in %s; next login %s; replay %s" % (before, mid, nxt, a_rep and a_rep["text"]))
        for c in (alice, bob, vic, bob2, alice2):
            c.close()
        gw.stop()
        gw = None
        # AL4 the last copy: a gateway restart without the academy keeps the levels
        cfg2 = gw_cfg_b(sim, root, academy_key=AL_KEY)
        gw2 = GwRunner(cfg2)
        bob3 = Client(cfg2["stations"]["port"], "bob", PW["bob"], name="copilot3")
        alice3 = Client(cfg2["stations"]["port"], "alice", PW["alice"], name="captain3")
        kept = (gw2.gw.academy.seq, alice3.welcome.get("licence"), bob3.welcome.get("licence"))
        bob3.close()
        alice3.close()
        report("AL4 the gateway's last copy: after a restart (academy not reachable) the levels still apply at login",
               kept == (1000, True, False), "seq, alice, bob = %s" % (kept,))
    finally:
        for r in (gw, gw2):
            if r:
                r.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)

# ------------------------------------------------------------------ section UH --

def section_uh():
    """Step 13q2c (owner 2026-11-01): UNCREWED HDG LOCK on the gateway - the navigation request acts like AUTO but
    only slows / stops (never a heading), a geofence breach = alarm bit 16 (the AURIX goes to STANDBY); CREWED HDG LOCK
    unchanged; a copilot's HDG LOCK = the captain's ACCEPT (a driving command); review LOW-1: an old telemetry copy =
    unknown mode. Unit level: the gateway's own nav_compose on a stub (no simulator)."""
    print("\n=== Section UH: UNCREWED HDG LOCK (step 13q2c) ===", flush=True)
    import types
    from avoid import Avoider
    from shallow import Shallow
    from vision.manager import VisionManager
    G = gwmod.Gateway
    lat0, lon0 = 37.80, 23.60

    class Charts:
        """Deep water, but shallower than the limit north of lat0 + shoal_m."""
        def __init__(self, shoal_m):
            self.shoal_lat = lat0 + shoal_m / 111120.0

        def depth(self, la, lo):
            return 1.0 if la >= self.shoal_lat else 30.0

        def nearest_shallow_m(self, la, lo, limit, max_m):
            d = (self.shoal_lat - la) * 111120.0
            return max(0.0, d) if d <= max_m else None

    class Fence:
        def __init__(self, poly):
            self.poly = poly

    def stub(mode, profile, shoal_m=None, fence=None, sog=6.0, pos=1, unc=5.0, age=0.0, link=True, cog=0.0, ov=False):
        s = types.SimpleNamespace()
        s.telem = {"pos_source": pos, "est_lat": lat0, "est_lon": lon0, "est_sog": sog, "est_cog": cog,
                   "heading": cog, "mode": mode, "unc_m": unc}
        s.telem2 = {"profile": profile, "ov_hand": ov}
        s.TELEM_MAX_AGE_S = G.TELEM_MAX_AGE_S
        s.telem_rx = time.time() - age
        s.aurix = types.SimpleNamespace(connected=link)
        s.telem_fresh = types.MethodType(G.telem_fresh, s)
        s.uh_check = types.MethodType(G.uh_check, s)
        s.fence_out = types.MethodType(G.fence_out, s)
        s.auto_uncrewed = G.auto_uncrewed
        s.check_hdg_shore = types.MethodType(G.check_hdg_shore, s)
        s.nav_compose = types.MethodType(G.nav_compose, s)
        s.audits = []
        s.audit = lambda kind, **kw: s.audits.append((kind, kw))
        s.sent = []
        s.nav_send = lambda req: s.sent.append(req)
        s.nav_skipped = 0
        s.nav_errors = 0
        s.avoider = Avoider()
        s.ais = types.SimpleNamespace(list=lambda: [])
        s.hdg_set_until = 0.0
        s.shallow = Shallow({}, Charts(shoal_m) if shoal_m is not None else None)
        s.vision = types.SimpleNamespace(nav=lambda mode, hdg_uncrewed=False: (None, 0, 0))
        s.geofence = Fence(fence or [])
        s.own = types.MethodType(G.own, s)
        s.nav_auto = types.MethodType(G.nav_auto, s)
        s.hdg_uncrewed = G.hdg_uncrewed
        s.uh_strict = G.uh_strict
        s.nav_state = {}
        return s

    U, C = proto.PROFILE_UNCREWED, proto.PROFILE_CREWED
    H = proto.MODE_HDG_LOCK
    # UH1 shallow water / the shore ahead: UNCREWED HDG LOCK slows (slow_kn), closer than stop_m = STOP (cap 0);
    # CREWED HDG LOCK: warning only (no cap); REMOTE: no cap
    far = G.nav_compose(stub(H, U, shoal_m=180.0))
    near = G.nav_compose(stub(H, U, shoal_m=60.0))
    crew = G.nav_compose(stub(H, C, shoal_m=60.0))
    rem = G.nav_compose(stub(proto.MODE_REMOTE, U, shoal_m=60.0))
    report("UH1 UNCREWED HDG LOCK: shallow / shore ahead -> slow (slow_kn), close ahead -> STOP (cap 0); CREWED HDG "
           "LOCK / REMOTE: no cap",
           far["kind"] == proto.NAV_CAP and far["cap_kn"] == 4.0 and far["alarm"] & 4 and
           near["kind"] == proto.NAV_CAP and near["cap_kn"] == 0.0 and crew["cap_kn"] is None and
           crew["kind"] == proto.NAV_NONE and rem["cap_kn"] is None,
           "180 m ahead: kind %s cap %s alarm %s; 60 m ahead: kind %s cap %s; CREWED HDG LOCK: kind %s cap %s; REMOTE: "
           "cap %s" % (far["kind"], far["cap_kn"], far["alarm"], near["kind"], near["cap_kn"], crew["kind"],
                       crew["cap_kn"], rem["cap_kn"]))

    # UH2 the geofence: UNCREWED HDG LOCK outside -> alarm bit 16; inside -> none; CREWED HDG LOCK / REMOTE outside ->
    # none (the AURIX would ignore it there anyway); no fence -> none
    box_in = [(lat0 - 0.01, lon0 - 0.01), (lat0 - 0.01, lon0 + 0.01), (lat0 + 0.01, lon0 + 0.01), (lat0 + 0.01, lon0 - 0.01)]
    box_out = [(lat0 + 0.02, lon0 - 0.01), (lat0 + 0.02, lon0 + 0.01), (lat0 + 0.04, lon0 + 0.01), (lat0 + 0.04, lon0 - 0.01)]
    o_out = G.nav_compose(stub(H, U, fence=box_out))
    o_in = G.nav_compose(stub(H, U, fence=box_in))
    c_out = G.nav_compose(stub(H, C, fence=box_out))
    r_out = G.nav_compose(stub(proto.MODE_REMOTE, U, fence=box_out))
    none = G.nav_compose(stub(H, U))
    report("UH2 UNCREWED HDG LOCK outside the geofence -> navreq alarm 16 (the AURIX: STANDBY); inside / CREWED / "
           "REMOTE / no fence -> nothing",
           o_out["alarm"] & 16 and not o_in["alarm"] & 16 and not c_out["alarm"] & 16 and not r_out["alarm"] & 16 and
           not none["alarm"] & 16,
           "outside %s, inside %s, CREWED outside %s, REMOTE outside %s, no fence %s" % (
               o_out["alarm"], o_in["alarm"], c_out["alarm"], r_out["alarm"], none["alarm"]))

    # UH3 the camera obstacle cap (stage 3) applies in AUTO and in UNCREWED HDG LOCK, never in CREWED HDG LOCK
    vm = types.SimpleNamespace(cfg={"stage2": True, "stage3": True}, threats=[1], cap_kn=1.5, cap_held=False)
    v_auto = VisionManager.nav(vm, proto.MODE_AUTO)
    v_uh = VisionManager.nav(vm, H, hdg_uncrewed=True)
    v_hdg = VisionManager.nav(vm, H)
    report("UH3 vision speed cap in AUTO and UNCREWED HDG LOCK (never a heading); CREWED HDG LOCK alarm only",
           v_auto[0] == 1.5 and v_uh[0] == 1.5 and v_hdg[0] is None and v_hdg[1] == 8,
           "AUTO %s, UNCREWED HDG LOCK %s, CREWED HDG LOCK %s" % (v_auto, v_uh, v_hdg))

    # UH4 a copilot's HDG LOCK (SET_MODE) is a driving command = the captain's ACCEPT; STANDBY stays free
    hl = proto.is_driving_cmd(proto.T_SET_MODE, proto.p_set_mode(H))
    sb = proto.is_driving_cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
    report("UH4 a copilot's HDG LOCK = the captain's ACCEPT (driving command); STANDBY free",
           hl and not sb, "HDG LOCK driving %s, STANDBY driving %s" % (hl, sb))

    # UH5 (review LOW-1) the gateway's telemetry copy is aged: older than 3 s or the AURIX link down = unknown mode ->
    # a copilot's AUTO return is not free any more (the captain's ACCEPT)
    import stations as stmod

    class StubGw:
        TELEM_MAX_AGE_S = 3.0

        def __init__(self):
            self.telem = {"mode": proto.MODE_STANDBY}
            self.telem_rx = time.time()
            self.aurix = types.SimpleNamespace(connected=True)

        def telem_fresh(self):
            return G.telem_fresh(self)
    st = types.SimpleNamespace(gw=StubGw())
    fresh = stmod.StationServer.copilot_free(st, proto.T_AUTO_RETURN)
    st.gw.telem_rx = time.time() - 5.0
    old = stmod.StationServer.copilot_free(st, proto.T_AUTO_RETURN)
    st.gw.telem_rx = time.time()
    st.gw.aurix.connected = False
    down = stmod.StationServer.copilot_free(st, proto.T_AUTO_RETURN)
    pause = stmod.StationServer.copilot_free(st, proto.T_AUTO_PAUSE)
    report("UH5 (LOW-1) an old telemetry copy (> 3 s / AURIX link down) = unknown mode: a copilot's AUTO return needs "
           "the captain's ACCEPT; AUTO pause stays free",
           fresh and not old and not down and pause,
           "fresh STANDBY free %s; 5 s old %s; AURIX down %s; AUTO pause %s" % (fresh, old, down, pause))

    # UH6 (step 13q2d, review MED-1 / MED-2) "no data = shallow" in UNCREWED HDG LOCK: no position (pos_source 0), no
    # charts, a position uncertainty above unc_max_m, or shallow water within the uncertainty = cannot evaluate = STOP
    # (cap 0, SHALLOW alarm) - never a 'no danger' lease; the look-ahead stop distance grows by the uncertainty.
    # CREWED HDG LOCK / AUTO / REMOTE without a position or charts: unchanged (no cap)
    def sh(r):
        return (r["kind"], r["cap_kn"], r["alarm"] & 4)
    nopos = G.nav_compose(stub(H, U, shoal_m=5000.0, pos=0))
    nochart = G.nav_compose(stub(H, U))
    bigunc = G.nav_compose(stub(H, U, shoal_m=5000.0, unc=150.0))
    within = G.nav_compose(stub(H, U, shoal_m=40.0, unc=60.0, cog=180.0))   # (near_m: shallow 40 m astern)
    behind = G.nav_compose(stub(H, U, shoal_m=40.0, unc=5.0, cog=180.0))    # (astern, unc 5 m: not ahead = none)
    ahead = G.nav_compose(stub(H, U, shoal_m=130.0, unc=60.0))             # ~130 m <= stop_m 100 + 60
    ahead_small = G.nav_compose(stub(H, U, shoal_m=130.0, unc=5.0))        # ~130 m > 105: slow only
    clear = G.nav_compose(stub(H, U, shoal_m=5000.0, unc=5.0))
    c_nopos = G.nav_compose(stub(H, C, pos=0))
    c_nochart = G.nav_compose(stub(H, C))
    a_nochart = G.nav_compose(stub(proto.MODE_AUTO, U))
    r_nopos = G.nav_compose(stub(proto.MODE_REMOTE, U, pos=0))
    stop = (proto.NAV_CAP, 0.0, 4)
    report("UH6 (13q2d) UNCREWED HDG LOCK: no position / no charts / uncertainty > unc_max_m / shallow within the "
           "uncertainty = STOP (cap 0); the stop distance grows by the uncertainty; CREWED / AUTO / REMOTE unchanged",
           sh(nopos) == stop and sh(nochart) == stop and sh(bigunc) == stop and sh(within) == stop and
           behind["kind"] == proto.NAV_NONE and sh(ahead) == stop and ahead_small["cap_kn"] == 4.0 and clear["kind"] == proto.NAV_NONE and
           c_nopos["kind"] == proto.NAV_NONE and c_nochart["kind"] == proto.NAV_NONE and
           a_nochart["kind"] == proto.NAV_NONE and r_nopos["kind"] == proto.NAV_NONE,
           "no position %s, no charts %s, unc 150 m %s, shoal 40 m astern with unc 60 m %s / unc 5 m %s, 130 m ahead unc 60 %s / unc 5 "
           "%s, clear %s; CREWED no position %s / no charts %s; AUTO no charts %s; REMOTE no position %s" % (
               sh(nopos), sh(nochart), sh(bigunc), sh(within), sh(behind), sh(ahead), sh(ahead_small), sh(clear), sh(c_nopos),
               sh(c_nochart), sh(a_nochart), sh(r_nopos)))

    # UH7 (step 13q2d, review LOW-1) a frozen telemetry copy (older than 3 s / the AURIX link down) is NOT used: own()
    # knows nothing, and the navigation request is NOT sent (no 'no danger' lease renewed on old data); fresh = sent
    st_fresh, st_old, st_down = stub(H, U, shoal_m=5000.0), stub(H, U, shoal_m=5000.0, age=5.0), \
        stub(H, U, shoal_m=5000.0, link=False)
    o_old = G.own(st_old)
    sent_fresh, sent_old, sent_down = G.nav_tick(st_fresh), G.nav_tick(st_old), G.nav_tick(st_down)
    report("UH7 (13q2d LOW-1) a frozen telemetry copy: own() knows nothing, no navreq lease is sent; fresh = sent",
           o_old.get("lat") is None and not o_old.get("fresh") and o_old.get("mode") is None and sent_fresh and
           len(st_fresh.sent) == 1 and not sent_old and not st_old.sent and st_old.nav_skipped == 1 and
           not sent_down and not st_down.sent,
           "old own() %s; sent fresh %s (%d), 5 s old %s (%d, skipped %d), AURIX down %s (%d)" % (
               o_old, sent_fresh, len(st_fresh.sent), sent_old, len(st_old.sent), st_old.nav_skipped, sent_down,
               len(st_down.sent)))

    # UH8 (step 13q2d, review MED-1) the UNCREWED SET_MODE HDG LOCK from REMOTE / MANEUVER is refused like an AUTO
    # start (114) when the gateway can't evaluate (no fresh telemetry / position / charts, uncertainty too big) or
    # shallow water / the shore is closer than stop_m ahead; 120 outside the geofence; fine = forwarded; CREWED /
    # STANDBY / another mode: the AURIX decides
    hl, sbm = proto.p_set_mode(H), proto.p_set_mode(proto.MODE_STANDBY)
    R, M = proto.MODE_REMOTE, proto.MODE_MANEUVER
    who = types.SimpleNamespace(id=2, user="alice")

    def chk(s_, pl=hl):
        return G.check_prog(s_, who, proto.T_SET_MODE, pl)
    cases = {"ok": chk(stub(R, U, shoal_m=5000.0)), "maneuver_ok": chk(stub(M, U, shoal_m=5000.0)),
             "no_pos": chk(stub(R, U, shoal_m=5000.0, pos=0)), "no_charts": chk(stub(R, U)),
             "unc": chk(stub(R, U, shoal_m=5000.0, unc=150.0)), "shoal": chk(stub(R, U, shoal_m=60.0)),
             "stale": chk(stub(R, U, shoal_m=5000.0, age=5.0)),
             "fence": chk(stub(R, U, shoal_m=5000.0, fence=box_out)),
             "crewed": chk(stub(R, C)), "standby_cmd": chk(stub(R, U), sbm), "from_auto": chk(stub(proto.MODE_AUTO, U))}
    st_a = stub(R, U, pos=0)
    chk(st_a)
    txt = lambda v: isinstance(v, str)
    report("UH8 (13q2d MED-1) UNCREWED HDG LOCK entry refused without a position / charts / fresh telemetry, too "
           "uncertain or shallow close ahead (114), outside the geofence (120); CREWED / STANDBY / AUTO -> the AURIX",
           cases["ok"] is None and cases["maneuver_ok"] is None and txt(cases["no_pos"]) and txt(cases["no_charts"]) and
           txt(cases["unc"]) and txt(cases["shoal"]) and txt(cases["stale"]) and
           isinstance(cases["fence"], tuple) and cases["fence"][0] == 120 and cases["crewed"] is None and
           cases["standby_cmd"] is None and cases["from_auto"] is None and
           [k for k, _ in st_a.audits] == ["hdg_lock_refused"] and proto.T_SET_MODE in stmod.PROG_CHECK_TYPES,
           "; ".join("%s %s" % (k, v) for k, v in cases.items()) + "; audit %s" % st_a.audits)

    # UH9 (step 13q2d, review MED-2) the geofence with the position uncertainty: inside but nearer the edge than the
    # uncertainty = a breach (alarm 16); far enough inside = none
    box_tight = [(lat0 - 0.01, lon0 - 0.01), (lat0 - 0.01, lon0 + 0.01), (lat0 + 50.0 / 111320.0, lon0 + 0.01),
                 (lat0 + 50.0 / 111320.0, lon0 - 0.01)]                  # the north edge 50 m from the boat
    e_big = G.nav_compose(stub(H, U, shoal_m=5000.0, fence=box_tight, unc=60.0, sog=0.0))
    e_small = G.nav_compose(stub(H, U, shoal_m=5000.0, fence=box_tight, unc=10.0, sog=0.0))
    import geofence as gfmod
    em = gfmod.edge_m(box_tight, lat0, lon0)
    report("UH9 (13q2d MED-2) geofence: nearer the edge than the position uncertainty = breach (alarm 16)",
           e_big["alarm"] & 16 and not e_small["alarm"] & 16 and 45.0 < em < 55.0,
           "edge %.1f m; unc 60 m alarm %s; unc 10 m alarm %s" % (em, e_big["alarm"], e_small["alarm"]))

    # UH10 (step 13q2e, review) nav_compose fails (a bug / bad chart data): logged + counted; in UNCREWED HDG LOCK an
    # explicit STOP request is sent (cap 0, SHALLOW + DRIVER DECISION = cannot evaluate, audit navreq_error) - never a
    # silently un-renewed lease; AUTO / CREWED HDG LOCK: nothing sent (the exception goes on to nav_loop's log = the
    # AURIX's lost-lease rule); without fresh telemetry: nothing sent either
    def boom():
        raise RuntimeError("test: nav_compose broken")

    def tick(s_):
        s_.nav_compose = boom
        try:
            return G.nav_tick(s_), None
        except RuntimeError as e:
            return None, e
    import logging as _lg
    _lg.disable(_lg.CRITICAL)
    try:
        st_u, st_a2, st_c, st_old2 = stub(H, U, shoal_m=5000.0), stub(proto.MODE_AUTO, U, shoal_m=5000.0),             stub(H, C, shoal_m=5000.0), stub(H, U, shoal_m=5000.0, age=5.0)
        res = {k: tick(v) for k, v in (("uh", st_u), ("auto", st_a2), ("crewed", st_c), ("old", st_old2))}
    finally:
        _lg.disable(_lg.NOTSET)
    rq = st_u.sent[0] if st_u.sent else {}
    report("UH10 (13q2e) nav_compose failing in UNCREWED HDG LOCK = an explicit STOP request (cap 0, alarm 4 + 2); "
           "AUTO / CREWED / old telemetry: nothing sent, the error goes on",
           res["uh"][0] is True and rq.get("kind") == proto.NAV_CAP and rq.get("cap_kn") == 0.0 and
           rq.get("alarm", 0) & 6 == 6 and st_u.nav_errors == 1 and [k for k, _ in st_u.audits] == ["navreq_error"] and
           res["auto"][1] is not None and not st_a2.sent and res["crewed"][1] is not None and not st_c.sent and
           res["old"][1] is not None and not st_old2.sent,
           "UNCREWED HDG LOCK %s sent %s errors %d audits %s; AUTO %s sent %d; CREWED %s sent %d; old telemetry %s sent %d" % (
               res["uh"], rq, st_u.nav_errors, st_u.audits, res["auto"], len(st_a2.sent), res["crewed"],
               len(st_c.sent), res["old"], len(st_old2.sent)))


    # UH11 (step 13q3, owner 2026-10-31 C3) UNCREWED AUTO: the shore captain may steer by hand inside AUTO - the fence
    # bit (16) is sent outside the geofence also in UNCREWED AUTO (the AURIX: STANDBY only while steering by hand);
    # CREWED AUTO: none; the stick's TRIGGER is forwarded as GW_RJOY flags b3 (the AURIX: the new HDG LOCK heading)
    a = proto.MODE_AUTO
    au_out = G.nav_compose(stub(a, U, fence=box_out))
    au_in = G.nav_compose(stub(a, U, fence=box_in))
    ac_out = G.nav_compose(stub(a, C, fence=box_out))
    import struct as _st
    import stations as stmod
    sent = []
    fake = types.SimpleNamespace(gw=types.SimpleNamespace(telem2={"helm": 5}, aurix=types.SimpleNamespace(
        send=lambda t, b: sent.append((t, b)))), age=lambda st_, ts: 0.05,
        question_open=lambda st_: False)           # (step 13p: no question open on the station here)
    st5 = types.SimpleNamespace(id=5, role="PILOT", rtt_ms=40)
    stmod.StationServer._rjoy(fake, st5, {"seq": 1, "ts": 0.0, "x": 0.4, "lever": 0.6, "trigger": True})
    stmod.StationServer._rjoy(fake, st5, {"seq": 2, "ts": 0.0, "x": 0.4, "lever": 0.6})
    fl = [_st.unpack("<BHHHhhHBB", b)[-1] for t, b in sent if t == proto.T_GW_RJOY]
    report("UH11 (13q3) UNCREWED AUTO outside the geofence -> alarm 16 (STANDBY only while steering by hand); "
           "CREWED AUTO none; the TRIGGER -> GW_RJOY flags b3",
           au_out["alarm"] & 16 and not au_in["alarm"] & 16 and not ac_out["alarm"] & 16 and fl == [1 | 8, 1],
           "UNCREWED AUTO outside %s inside %s, CREWED AUTO outside %s; flags %s" % (
               au_out["alarm"], au_in["alarm"], ac_out["alarm"], fl))

    # UH12 (step 13q3b, review MED-1) UNCREWED AUTO steered BY HAND with the shore stick (TELEM2 D b3 = ov_hand) gets
    # the UNCREWED HDG LOCK protections: no position / no charts / too uncertain = STOP (cap 0), shallow / the shore
    # within stop_m ahead = STOP, farther = slow_kn; a broken nav_compose = the explicit STOP. Not steering by hand
    # (AUTO on its checked route) / CREWED: unchanged (no charts = nothing, 60 m = slow_kn only)
    h_nochart = G.nav_compose(stub(a, U, ov=True))
    h_nopos = G.nav_compose(stub(a, U, shoal_m=5000.0, pos=0, ov=True))
    h_unc = G.nav_compose(stub(a, U, shoal_m=5000.0, unc=150.0, ov=True))
    h_near = G.nav_compose(stub(a, U, shoal_m=60.0, ov=True))
    h_far = G.nav_compose(stub(a, U, shoal_m=180.0, ov=True))
    h_clear = G.nav_compose(stub(a, U, shoal_m=5000.0, ov=True))
    n_nochart = G.nav_compose(stub(a, U))
    n_near = G.nav_compose(stub(a, U, shoal_m=60.0))
    c_near = G.nav_compose(stub(a, C, shoal_m=60.0, ov=True))
    o_hand = G.own(stub(a, U, shoal_m=5000.0, ov=True))
    _lg.disable(_lg.CRITICAL)
    try:
        st_h = stub(a, U, shoal_m=5000.0, ov=True)
        res_h = tick(st_h)
    finally:
        _lg.disable(_lg.NOTSET)
    rq_h = st_h.sent[0] if st_h.sent else {}
    report("UH12 (13q3b MED-1) UNCREWED AUTO steered by hand: no data / too uncertain / shallow within stop_m = STOP "
           "(cap 0), farther = slow; a broken nav_compose = STOP; not by hand / CREWED: unchanged",
           sh(h_nochart) == stop and sh(h_nopos) == stop and sh(h_unc) == stop and sh(h_near) == stop and
           h_far["cap_kn"] == 4.0 and h_clear["kind"] == proto.NAV_NONE and n_nochart["kind"] == proto.NAV_NONE and
           n_near["cap_kn"] == 4.0 and c_near["cap_kn"] == 4.0 and o_hand.get("ov_hand") is True and
           res_h[0] is True and rq_h.get("cap_kn") == 0.0,
           "by hand: no charts %s, no position %s, unc 150 %s, 60 m %s, 180 m %s, clear %s; not by hand: no charts %s, "
           "60 m %s; CREWED 60 m %s; own() ov_hand %s; nav_compose broken %s -> %s" % (
               sh(h_nochart), sh(h_nopos), sh(h_unc), sh(h_near), h_far["cap_kn"], h_clear["kind"],
               n_nochart["kind"], n_near["cap_kn"], c_near["cap_kn"], o_hand.get("ov_hand"), res_h, rq_h))

    # UH13 (step 13q3b, review LOW-2) one press never does two things: a trigger mapped to the same button as the
    # CONFIRM (or SAVE) is never sent as the trigger (shorejoy.trigger_conflict); different buttons: both sent
    import shorejoy as sj

    class FakeJs:
        def __init__(self, held):
            self.held = held

        def read(self, dev):
            b = 0
            for d_, n in self.held:
                if d_ == dev:
                    b |= 1 << (n - 1)
            return {"ok": True, "axes": {"X": 0.0, "Z": 0.0}, "buttons": b}
    cfg_ok = dict(sj.DEFAULT)
    cfg_same = dict(sj.DEFAULT, trigger_button=dict(sj.DEFAULT["confirm_button"]))
    cfg_save = dict(sj.DEFAULT, trigger_button=dict(sj.DEFAULT["save_button"]))
    cb, tb = sj.DEFAULT["confirm_button"], sj.DEFAULT["trigger_button"]
    both = sj.to_remote(FakeJs([(cb["dev"], cb["button"]), (tb["dev"], tb["button"])]), cfg_ok)
    same = sj.to_remote(FakeJs([(cb["dev"], cb["button"])]), cfg_same)
    save = sj.to_remote(FakeJs([(sj.DEFAULT["save_button"]["dev"], sj.DEFAULT["save_button"]["button"])]), cfg_save)
    report("UH13 (13q3b LOW-2) a trigger on the CONFIRM / SAVE button is never sent as the trigger; separate buttons: both",
           both.get("confirm") and both.get("trigger") and same.get("confirm") and not same.get("trigger") and
           not save.get("trigger") and sj.trigger_conflict(cfg_same) and sj.trigger_conflict(cfg_save) and
           not sj.trigger_conflict(cfg_ok),
           "separate %s; trigger = CONFIRM %s; trigger = SAVE %s" % (both, same, save))

    # UH14 (step 13q5, owner 2026-10-31 C5) the station sends the spring-centred stick's Y BESIDE the lever (the
    # MANEUVER gear: the AURIX decides); pushed forward = + (the default: the steer device's Y, inverted); the
    # gateway forwards it in GW_RJOY (lever flag b0 still set)
    class AxisJs:
        def __init__(self, axes, ok_devs=(0, 1)):
            self.axes, self.ok_devs = axes, ok_devs

        def read(self, dev):
            return {"ok": dev in self.ok_devs, "axes": dict(self.axes.get(dev, {})), "buttons": 0}
    fwd = sj.to_remote(AxisJs({0: {"X": 0.0, "Y": -1.0}, 1: {"Z": 0.0}}), dict(sj.DEFAULT))
    back = sj.to_remote(AxisJs({0: {"X": 0.0, "Y": 0.9}, 1: {"Z": 0.0}}), dict(sj.DEFAULT))
    own = sj.to_remote(AxisJs({0: {"X": 0.0}, 2: {"V": 0.5}}, ok_devs=(0, 1, 2)),
                       dict(sj.DEFAULT, stick_y={"dev": 2, "axis": "V", "invert": False}))
    gone = sj.to_remote(AxisJs({0: {"X": 0.0}}, ok_devs=(0, 1)), dict(sj.DEFAULT, stick_y={"dev": 2, "axis": "Y"}))
    nolever = sj.to_remote(AxisJs({0: {"X": 0.0, "Y": 0.3}, 1: {"Z": 0.4}}),
                           dict(sj.DEFAULT, thr={"dev": 1, "axis": "Z", "invert": False, "lever": False}))
    report("UH14 (13q5) the stick Y is sent beside the lever (forward +, own axis, missing device = no packet); no "
           "lever: y = the throttle axis",
           fwd is not None and fwd.get("y") == 1.0 and "lever" in fwd and back.get("y") == -0.9 and
           own.get("y") == 0.5 and gone is None and nolever.get("y") == 0.4 and "lever" not in nolever,
           "forward %s; back %s; own axis %s; missing %s; no lever %s" % (fwd, back, own, gone, nolever))

    # UH15 (step 13q5b, review LOW-3) a NaN / infinite stick or lever never becomes a full deflection (Python's min /
    # max would pass NaN as the limit = full forward gear / full lever): the station sends no packet, the gateway
    # drops such a packet (counted); a normal packet still goes through
    nan, inf = float("nan"), float("inf")
    s_x = sj.to_remote(AxisJs({0: {"X": nan, "Y": 0.0}, 1: {"Z": 0.0}}), dict(sj.DEFAULT))
    s_y = sj.to_remote(AxisJs({0: {"X": 0.0, "Y": nan}, 1: {"Z": 0.0}}), dict(sj.DEFAULT))
    s_l = sj.to_remote(AxisJs({0: {"X": 0.0, "Y": 0.0}, 1: {"Z": inf}}), dict(sj.DEFAULT))
    s_n = sj.to_remote(AxisJs({0: {"X": 0.0, "Y": nan}, 1: {"Z": 0.0}}),
                       dict(sj.DEFAULT, thr={"dev": 0, "axis": "Y", "invert": False, "lever": False}))
    s_ok = sj.to_remote(AxisJs({0: {"X": 0.1, "Y": 0.0}, 1: {"Z": 0.0}}), dict(sj.DEFAULT))
    sent15 = []
    fake15 = types.SimpleNamespace(gw=types.SimpleNamespace(telem2={"helm": 5}, aurix=types.SimpleNamespace(
        send=lambda t, b: sent15.append((t, b)))), age=lambda st_, ts: 0.05 + (ts - ts))
    st15 = types.SimpleNamespace(id=5, role="PILOT", rtt_ms=40)
    bad = [{"x": nan}, {"y": nan}, {"y": -inf}, {"lever": nan}, {"lever": inf}, {"ts": nan}]
    for i, b in enumerate(bad):
        stmod.StationServer._rjoy(fake15, st15, dict({"seq": i + 1, "ts": 0.0, "x": 0.0, "y": 0.0, "lever": 0.5}, **b))
    n_bad = len(sent15)
    stmod.StationServer._rjoy(fake15, st15, {"seq": 20, "ts": 0.0, "x": 0.2, "y": 0.0, "lever": 0.5})
    good = [_st.unpack("<BHHHhhHBB", b_) for t, b_ in sent15 if t == proto.T_GW_RJOY]
    report("UH15 (13q5b LOW-3) NaN / infinite stick or lever: the station sends no packet, the gateway drops it "
           "(counted); a normal packet still goes through",
           s_x is None and s_y is None and s_l is None and s_n is None and s_ok is not None and n_bad == 0 and
           getattr(st15, "rjoy_bad", 0) == len(bad) and len(good) == 1 and good[0][4] == 200,
           "station: x NaN %s, y NaN %s, lever inf %s, no-lever y NaN %s, normal %s; gateway: %d bad forwarded %d "
           "(counted %d), normal %s" % (s_x, s_y, s_l, s_n, s_ok, len(bad), n_bad, getattr(st15, "rjoy_bad", 0), good))


# ------------------------------------------------------------------- section CV --

def section_cv():
    """13n2 (owner 2026-10-31 E2): the camera view of screen 2 (gateway/cameras.py) - no MediaMTX needed."""
    print("\n=== Section CV: camera view, the captain's video first (13n2) ===", flush=True)
    import types
    import stations as stmod
    from cameras import CameraManager
    saver = [False]
    audits = []
    gw = types.SimpleNamespace(cfg={}, telem2={}, net=types.SimpleNamespace(on_emergency=False),
                               data_saver=lambda: saver[0], audit=lambda kind, **kw: audits.append((kind, kw)),
                               stations=types.SimpleNamespace(stations={}))
    cm = CameraManager({"copilot_budget_kbps": 6000, "view_restore_s": 10.0}, gw)

    def station(sid, crew, single_op=False):
        st = types.SimpleNamespace(id=sid, user="u%d" % sid, crew=crew, single_op=single_op, session="s%d" % sid,
                                   video_key="k%d" % sid, sent=[])
        st.send = st.sent.append
        gw.stations.stations[sid] = st
        return st
    cap, cop = station(2, "captain"), station(3, "copilot")
    last = lambda st: st.sent[-1] if st.sent else {}
    # CV1 lighter stream for the copilot, only while open; the captain's own view keeps its quality
    cm.request(cop, {"preset": "Full"})
    full = dict(last(cop)["streams"])
    cm.view_request(cop, {"cam": "FRONT-1"})
    v_front = dict(last(cop)["streams"]), last(cop)["view_light"]
    cm.view_request(cop, {"cam": "REAR-RIGHT"})
    cm.request(cop, {"preset": "All off"})                  # a preset change keeps the open view
    v_rear = dict(last(cop)["streams"]), last(cop)["view"]
    cm.view_request(cop, {"cam": "NOSE"})
    unknown = last(cop).get("ok") is False
    cm.view_request(cop, {"cam": None})
    closed = dict(last(cop)["streams"]), last(cop)["view"]
    cm.request(cap, {"preset": "Full"})
    cm.view_request(cap, {"cam": "FRONT-1"})
    cap_v = dict(last(cap)["streams"]), last(cap)["view_light"]
    cm.view_request(cap, {"cam": None})
    report("CV1 camera view: the copilot gets the LOW (lighter) stream of the chosen camera, only while it is open "
           "(a preset change keeps it, closing removes it), an unknown camera refused; the captain's own view keeps "
           "HIGH",
           full["FRONT-1"] == "cam_front1_hi" and v_front[0]["FRONT-1"] == "cam_front1_lo" and v_front[1] and
           v_rear == ({"REAR-RIGHT": "cam_rearright_lo"}, "REAR-RIGHT") and unknown and closed == ({}, None) and
           cap_v[0]["FRONT-1"] == "cam_front1_hi" and cap_v[1] is False and
           ("camera_view", {"station": 3, "user": "u3", "cam": "FRONT-1"}) in audits,
           "Full %s; view FRONT-1 %s; view REAR-RIGHT (preset All off) %s; unknown refused %s; closed %s; captain "
           "view %s" % (full, v_front, v_rear, unknown, closed, cap_v))
    # CV2 the captain's video first: over the budget -> the copilot's view dropped (never the captain's), back with
    # hysteresis only when the captain's video + the dropped view fit 80 % for 10 s
    cm.request(cop, {"preset": "All off"})
    cm.view_request(cop, {"cam": "REAR-LEFT"})
    cm.request(cap, {"preset": "Full"})
    cm.station_kbps, cm.total_kbps = {2: 3000, 3: 3500}, 6500
    t0 = 1000.0
    ch1 = cm.update_priority(t0)
    red = cm.allowed_paths(3), cm.streams_msg(cop), cm.allowed_paths(2)
    cm.station_kbps, cm.total_kbps = {2: 3000}, 3000           # the copilot's view stopped: 3000 + 3500 > 4800
    still = not cm.update_priority(t0 + 30) and cm.views_reduced
    cm.station_kbps, cm.total_kbps = {2: 1000}, 1000           # the captain needs less now: 1000 + 3500 < 4800
    early = cm.update_priority(t0 + 31) or cm.update_priority(t0 + 40)
    back = cm.update_priority(t0 + 41.5) and not cm.views_reduced and "cam_rearleft_lo" in cm.allowed_paths(3)
    no_cap = not cm.update_priority(t0 + 50)                    # nothing to do without the captain's video
    cm.station_kbps, cm.total_kbps = {3: 9000}, 9000
    no_cap_red = not cm.update_priority(t0 + 51) and not cm.views_reduced
    report("CV2 the captain's video has priority: over copilot_budget_kbps the copilot's view drops FIRST (\"VIDEO "
           "REDUCED FOR THE CAPTAIN\"), the captain keeps every stream; back only when it fits 80 % for 10 s; no "
           "captain watching = no reduction",
           ch1 and "cam_rearleft_lo" not in red[0] and red[1]["view_reduced"] and
           red[1]["notice"].startswith("VIDEO REDUCED FOR THE CAPTAIN") and "cam_front1_hi" in red[2] and still and
           not early and back and no_cap and no_cap_red and any(k == "camera_view_reduced" for k, _ in audits) and
           any(k == "camera_view_restored" for k, _ in audits),
           "reduced %s (copilot paths %s, notice %r, captain paths %s); still reduced while it does not fit %s; "
           "not back before 10 s %s; back %s; no captain %s / %s" % (
               ch1, sorted(red[0]), red[1]["notice"], sorted(red[2]), still, not early, back, no_cap, no_cap_red))
    # CV3 data saver: no extra view for anybody; HELM holder = the captain's video
    saver[0] = True
    sv = cm.streams_msg(cop)
    saver[0] = False
    helper = station(4, "copilot")
    cm.view_request(helper, {"cam": "FRONT-2"})
    gw.telem2["helm"] = 4
    helm_prio = cm.is_priority(4) and cm.streams_msg(helper)["view_light"] is False
    gw.telem2["helm"] = 2
    report("CV3 data saver / STARLINK: no camera view (the front-LOW rule stays); the HELM holder counts as the "
           "captain's video",
           "REAR-LEFT" not in sv["streams"] and sv["view_reduced"] and sv["notice"].startswith("DATA SAVER") and
           helm_prio, "data saver streams %s, notice %r; HELM holder priority %s" % (sv["streams"], sv["notice"],
                                                                                    helm_prio))
    # CV4 the view (and the streams) end with the station: a disconnect forgets it
    srv = types.SimpleNamespace(stations={3: cop}, gw=types.SimpleNamespace(
        cams=cm, aurix=types.SimpleNamespace(send=lambda t, b: None), audit=lambda *a, **k: None),
        _check_asks=lambda: None)
    cop.closed, cop.local, cop.writer = False, False, types.SimpleNamespace(close=lambda: None)
    stmod.StationServer._drop(srv, cop, "disconnected")
    report("CV4 the camera view ends with the station (disconnect: nothing kept for it)",
           3 not in cm.station_cams and "REAR-LEFT" not in cm.status()["views"].values(),
           "station_cams %s, views %s" % (sorted(cm.station_cams), cm.status()["views"]))
    # CV5 (review 13n2 MED-2) a station that never asked for cameras (a copilot's screen 2 alone) gets the VIEW ONLY:
    # no default "Front only" streams that would keep running after MAP / Esc; data saver = nothing
    lone = station(5, "copilot")
    cm.view_request(lone, {"cam": "REAR-LEFT"})
    only = dict(last(lone)["streams"])
    cm.view_request(lone, {"cam": None})
    after = dict(last(lone)["streams"])
    cm.view_request(lone, {"cam": "FRONT-2"})
    saver[0] = True
    lone_saver = dict(cm.streams_msg(lone)["streams"])
    saver[0] = False
    cm.request(lone, {"preset": "Front only"})              # asking for cameras later = the normal preset again
    preset_back = "FRONT-1" in last(lone)["streams"] and last(lone)["view"] == "FRONT-2"
    cm.view_request(lone, {"cam": None})
    report("CV5 (review 13n2 MED-2) a copilot's view without its own camera request = that camera only, nothing after "
           "MAP / Esc, nothing with data saver; a later camera request = its preset again",
           only == {"REAR-LEFT": "cam_rearleft_lo"} and after == {} and lone_saver == {} and preset_back,
           "view only %s; after closing %s; data saver %s; preset back %s" % (only, after, lone_saver, preset_back))
    # CV6 (review 13n2 MED-3) a hung MediaMTX API never blocks the gateway's loop: closing readers runs in a worker
    import threading as _th
    posted, release = [], _th.Event()

    def hung_post(path):
        posted.append(path)
        release.wait(3.0)
        return 200
    cm._post = hung_post
    cm.request(cop, {"preset": "All off"})
    cm.view_request(cop, {"cam": "FRONT-1"})
    with cm.lock:
        cm.sessions = {"r1": ("cam_rearright_hi", 3, 0, time.time()), "r2": ("cam_front1_lo", 3, 0, time.time())}
    t = time.perf_counter()
    cm.view_request(cop, {"cam": None})                     # FRONT-1 not allowed any more -> kick r2
    fut = cm.kick_disallowed()                              # (and again from the camera loop)
    cm.update_priority()
    quick = (time.perf_counter() - t) < 0.2
    _th.Timer(0.3, release.set).start()
    done = fut is not None and fut.result(timeout=10) is None
    kicked = "/v3/rtspsessions/kick/r2" in posted and "/v3/rtspsessions/kick/r1" in posted
    report("CV6 (review 13n2 MED-3) a hung MediaMTX API: view_request / kick_disallowed / update_priority return at "
           "once (the posts run in the worker thread, in order), the readers are still closed",
           quick and done and kicked, "returned in %.0f ms; posts %s" % ((time.perf_counter() - t) * 1000, posted))


SECTIONS = {"F1": section_f1, "CR": section_cr, "AIS": section_ais, "SH": section_sh, "CA": section_ca, "VI": section_vi, "VR": section_vr, "A": section_a, "B": section_b, "C": section_c, "D": section_d, "E": section_e, "F": section_f,
            "G": section_g, "GF": section_gf, "RL": section_rl, "DP": section_dp, "AR": section_ar,
            "AL": section_al, "UH": section_uh, "CV": section_cv, "KS": section_ks, "SA": section_sa}


def section_stall_selftest():
    """W50s self-test of the harness (only with TRYGONS_GW_STALL_SELFTEST=1): a section that starts a simulator,
    passes one test and then hangs - the full run must report it STALLED (re-run once), never a pass, and leave no
    autopilot_sim.exe behind. TRYGONS_GW_STALL_SELFTEST=hold: it hangs without letting Python threads run (the
    parent's own limit + its job-object kill)."""
    print("\n=== Section ZZ: harness stall self-test ===", flush=True)
    Sim(1.0)
    report("ZZ1 the self-test section started a simulator", True, "then it hangs")
    if os.environ.get("TRYGONS_GW_STALL_SELFTEST") == "hold":
        import faulthandler
        faulthandler.cancel_dump_traceback_later()
        import ctypes                               # a PyDLL call keeps the GIL: no watchdog thread can run any more
        if os.name == "nt":
            ctypes.PyDLL("kernel32").Sleep(0xFFFFFFFF)
        else:
            ctypes.PyDLL(None).pause()
    while True:
        time.sleep(1)


if os.environ.get("TRYGONS_GW_STALL_SELFTEST"):
    SECTIONS["ZZ"] = section_stall_selftest


def _watchdog(name, limit_s):
    """W50s: the section's own time limit (below the parent's): dump every thread's stack (where it hangs), end
    the simulators it started and leave with STALL_EXIT - a stall is never reported as a pass. faulthandler's own
    C-level timer is the backstop even if no Python thread can run any more."""
    import faulthandler
    faulthandler.dump_traceback_later(limit_s + 20.0, exit=True)

    def fire():
        print("\nSTALLED: section %s still running after %.0f s (its own time limit) - stacks on stderr" % (
            name, limit_s), flush=True)
        try:
            faulthandler.dump_traceback(all_threads=True)
        except Exception:
            pass
        kill_leftovers()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(STALL_EXIT)
    t = threading.Timer(limit_s, fire)
    t.daemon = True
    t.start()


def run_one(name):
    """One section in THIS process (the child of the full run, or `python test_gateway.py CR`). Never returns."""
    _watchdog(name, max(30.0, SECTION_LIMIT_S - 60.0))
    code = None
    try:
        SECTIONS[name]()
    except BaseException:
        import traceback
        traceback.print_exc()
        print("SECTION CRASHED: %s (the tests after the error did not run)" % name, flush=True)
        code = CRASH_EXIT
    # W50 (found in step 13r with faulthandler): after every test of a section has passed, the interpreter's exit
    # could hang for ever in BaseEventLoop.__del__ -> ProactorEventLoop.close -> _poll (an unclosed Windows
    # proactor loop of a GwRunner waiting for an overlapped operation that never completes). The results are
    # printed by now: end what is left (W50s) and leave at once with their exit code
    kill_leftovers()
    for r in list(_RUNNERS):                    # a runner a crashed section never stopped: a short stop
        if r.th.is_alive():
            try:
                r.loop.call_soon_threadsafe(r.gw.stop)
            except RuntimeError:
                pass
            r.th.join(5)
    bad = [n for n, ok in results if not ok]
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code if code is not None else (1 if bad else 0))


class _Job:
    """W50s: a Windows job object that holds a section process and everything it starts (the simulators, ffmpeg,
    the vision worker ...): kill() ends the whole tree, also processes whose parent has already gone. POSIX: a
    process group."""

    def __init__(self):
        self.h = None
        if os.name != "nt":
            return
        try:
            import ctypes
            from ctypes import wintypes
            k32 = ctypes.WinDLL("kernel32", use_last_error=True)
            k32.CreateJobObjectW.restype = wintypes.HANDLE
            k32.OpenProcess.restype = wintypes.HANDLE
            h = k32.CreateJobObjectW(None, None)

            class BASIC(ctypes.Structure):
                _fields_ = [("a", ctypes.c_int64), ("b", ctypes.c_int64), ("LimitFlags", wintypes.DWORD),
                            ("MinWS", ctypes.c_size_t), ("MaxWS", ctypes.c_size_t),
                            ("ActiveProcessLimit", wintypes.DWORD), ("Affinity", ctypes.c_size_t),
                            ("PriorityClass", wintypes.DWORD), ("SchedulingClass", wintypes.DWORD)]

            class IOC(ctypes.Structure):
                _fields_ = [("c%d" % i, ctypes.c_uint64) for i in range(6)]

            class EXT(ctypes.Structure):
                _fields_ = [("Basic", BASIC), ("Io", IOC), ("ProcessMemoryLimit", ctypes.c_size_t),
                            ("JobMemoryLimit", ctypes.c_size_t), ("PeakProcessMemoryUsed", ctypes.c_size_t),
                            ("PeakJobMemoryUsed", ctypes.c_size_t)]
            info = EXT()
            info.Basic.LimitFlags = 0x2000              # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if h and k32.SetInformationJobObject(wintypes.HANDLE(h), 9, ctypes.byref(info), ctypes.sizeof(info)):
                self.h, self.k32, self.wt = h, k32, wintypes
            else:
                print("W50s: no job object (error %d): taskkill /T is used" % ctypes.get_last_error(), flush=True)
        except Exception as e:
            print("W50s: no job object (%r): taskkill /T is used" % (e,), flush=True)

    def popen(self, args, out, err):
        if os.name != "nt":
            return subprocess.Popen(args, stdout=out, stderr=err, stdin=subprocess.DEVNULL, start_new_session=True)
        p = subprocess.Popen(args, stdout=out, stderr=err, stdin=subprocess.DEVNULL)
        if self.h:
            hp = self.k32.OpenProcess(0x0101, False, p.pid)       # PROCESS_TERMINATE | PROCESS_SET_QUOTA
            ok = False
            if hp:
                ok = bool(self.k32.AssignProcessToJobObject(self.wt.HANDLE(self.h), self.wt.HANDLE(hp)))
                self.k32.CloseHandle(self.wt.HANDLE(hp))
            if not ok:
                print("W50s: section process not in the job: taskkill /T is used", flush=True)
                self.close()
        return p

    def kill(self, p):
        """End the section process and every process it started (always called: nothing is left behind)."""
        if os.name != "nt":
            try:
                os.killpg(p.pid, 9)
            except OSError:
                pass
        elif self.h:
            self.k32.TerminateJobObject(self.wt.HANDLE(self.h), 1)
        elif p.poll() is None:
            try:
                subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True, timeout=30)
            except Exception:
                pass
        try:
            p.kill()
        except OSError:
            pass
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass

    def close(self):
        if self.h:
            self.k32.CloseHandle(self.wt.HANDLE(self.h))
            self.h = None


def _win_processes():
    """{pid: (parent pid, exe name)} of every process (Windows ToolHelp snapshot); {} if it fails."""
    out = {}
    try:
        import ctypes
        from ctypes import wintypes

        class PE(ctypes.Structure):
            _fields_ = [("dwSize", wintypes.DWORD), ("cntUsage", wintypes.DWORD), ("pid", wintypes.DWORD),
                        ("heap", ctypes.c_size_t), ("mod", wintypes.DWORD), ("threads", wintypes.DWORD),
                        ("ppid", wintypes.DWORD), ("prio", ctypes.c_long), ("flags", wintypes.DWORD),
                        ("exe", ctypes.c_wchar * 260)]
        k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
        snap = k32.CreateToolhelp32Snapshot(2, 0)              # TH32CS_SNAPPROCESS
        if not snap or snap == wintypes.HANDLE(-1).value:
            return out
        try:
            e = PE()
            e.dwSize = ctypes.sizeof(PE)
            ok = k32.Process32FirstW(wintypes.HANDLE(snap), ctypes.byref(e))
            while ok:
                out[e.pid] = (e.ppid, e.exe)
                ok = k32.Process32NextW(wintypes.HANDLE(snap), ctypes.byref(e))
        finally:
            k32.CloseHandle(wintypes.HANDLE(snap))
    except Exception:
        return {}
    return out


class _Leftovers:
    """Review of W50s LOW-2: without a job object (Windows) the processes a section started are remembered while it
    runs (a snapshot every 2 s) and the ones still alive are killed after the section ends - also after a NORMAL
    exit (taskkill /T can't reach an orphan whose parent is gone)."""

    def __init__(self, root_pid):
        self.root, self.seen, self.stop = root_pid, {}, threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _scan(self):
        procs = _win_processes()
        tree = {self.root}
        grew = True
        while grew:
            grew = False
            for pid, (ppid, exe) in procs.items():
                if ppid in tree and pid not in tree and pid != ppid:
                    tree.add(pid)
                    self.seen[pid] = exe
                    grew = True

    def _run(self):
        while not self.stop.wait(2.0):
            self._scan()

    def kill(self):
        self.stop.set()
        self.th.join(5)
        now = _win_processes()
        for pid, exe in self.seen.items():
            if pid in now and now[pid][1] == exe:            # still alive (the same program: not a reused pid)
                try:
                    subprocess.run(["taskkill", "/F", "/PID", str(pid)], capture_output=True, timeout=30)
                except Exception:
                    pass


def run_section_process(k):
    """W50s: one section in its own process with the parent's time limit. Output goes to files (never a pipe a
    grandchild could hold open). Returns (status, output lines, stderr tail, seconds); status "ok" / "fail" /
    "crashed" / "stalled"."""
    tmpd = tempfile.mkdtemp(prefix="gwtest_sec_%s_" % k)
    po, pe = os.path.join(tmpd, "out.txt"), os.path.join(tmpd, "err.txt")
    t0 = time.time()
    job = _Job()
    with open(po, "w", encoding="utf-8", errors="replace") as fo, \
            open(pe, "w", encoding="utf-8", errors="replace") as fe:
        p = job.popen([sys.executable, os.path.abspath(__file__), k], fo, fe)
        left = _Leftovers(p.pid) if os.name == "nt" and not job.h else None
        try:
            rc = p.wait(timeout=SECTION_LIMIT_S)
            parent_stall = False
        except subprocess.TimeoutExpired:
            rc, parent_stall = None, True
        if left is not None:
            left._scan()                        # the last children before the tree is ended
        job.kill(p)                             # always: no orphan simulator / ffmpeg after a section
        job.close()
        if left is not None:
            left.kill()                         # review of W50s LOW-2: also after a normal exit
    dt = time.time() - t0
    with open(po, encoding="utf-8", errors="replace") as f:
        out = f.read()
    with open(pe, encoding="utf-8", errors="replace") as f:
        err = f.read()
    shutil.rmtree(tmpd, ignore_errors=True)
    lines = [ln for ln in out.splitlines()
             if ln.startswith(("[PASS] ", "[FAIL] ", "===", "STALLED", "SECTION CRASHED"))]
    # a Python error through the script's top level (not the harmless "Exception ignored" noise of background tasks)
    real_tb = re.search(r'File ".*test_gateway\.py", line \d+, in (<module>|run_one)', err) is not None
    # STALL_EXIT = the section's own watchdog; "Timeout (0:" = faulthandler's C-level backstop
    stalled = parent_stall or rc == STALL_EXIT or "Timeout (0:" in err
    if stalled:
        status = "stalled"
    elif rc == CRASH_EXIT or rc not in (0, 1) or real_tb:
        status = "crashed"
    else:
        status = "ok" if rc == 0 else "fail"
    return status, lines, err[-3000:], dt


def main():
    setup_logging(30)                       # warnings only
    if not os.path.exists(EXE):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    want = [a.upper() for a in sys.argv[1:]] or list(SECTIONS)
    if len(want) == 1:
        run_one(want[0])                        # never returns (os._exit)
    # every section in its own process: timing tests must not feel the threads / proxies of the sections before them.
    # W50s + LOOP RULE 3: a section that STALLS (its time limit hit) is killed with everything it started and reported
    # STALLED - never a pass. Stalled without a failed test = re-run ONCE alone; a pass then is noted as "stalled
    # once, re-run alone: PASS" in the summary. A FAILED test is never retried.
    stalls = []
    for k in want:
        for attempt in (1, 2):
            status, lines, err, dt = run_section_process(k)
            got = [(ln[7:], ln.startswith("[PASS]")) for ln in lines if ln.startswith(("[PASS] ", "[FAIL] "))]
            failed = any(not ok for _, ok in got)
            if status == "stalled" and attempt == 1 and not failed:
                print("\nsection %s STALLED after %.0f s (%d tests had passed, none failed) - killed, re-run once "
                      "alone; where it hung:\n%s" % (k, dt, len(got), err[-2500:]), flush=True)
                stalls.append(k)
                continue
            for line in lines:
                print(("\n" + line) if line.startswith("===") else line, flush=True)
            results.extend(got)
            if attempt == 2:
                stalls[-1] = "section %s stalled once, re-run alone: %s" % (k, "PASS" if status == "ok"
                                                                         else status.upper())
            print("section %s: %s in %.0f s" % (k, status.upper(), dt), flush=True)
            if status == "stalled":
                print("section %s STALLED (killed after %.0f s; the tests after it did not run); where it hung:\n%s"
                      % (k, dt, err[-2500:]), flush=True)
                results.append(("section %s STALLED" % k, False))
            elif status == "crashed":
                print("section %s CRASHED (the tests after the error did not run):\n%s" % (k, err[-1500:]), flush=True)
                results.append(("section %s ran to the end" % k, False))
            elif status == "fail" and not failed:
                # review of W50s LOW-1: exit code 1 with no "[FAIL] " line parsed = still a failure, never a pass
                print("section %s FAILED (exit code 1) without a [FAIL] line" % k, flush=True)
                results.append(("section %s exit code 1 without a [FAIL] line" % k, False))
            break
    bad = [n for n, ok in results if not ok]
    note = ("; " + "; ".join(stalls)) if stalls else ""
    print("\nGATEWAY TESTS: %d passed, %d failed%s" % (len(results) - len(bad), len(bad), note))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
