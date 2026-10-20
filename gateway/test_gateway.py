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


SECTIONS = {"A": section_a}


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
