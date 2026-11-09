"""The ADS-B receiver reader = its OWN low-priority process (CLAUDE.md X K4, docs/sar_design.md 6, step PF5).

Reads readsb / dump1090 (SBS, TCP port 30003, or Beast, port 30005), keeps the aircraft (adsb.AdsbTargets) and writes
ONE JSON line per second to stdout:
    {"ev": "state", "state": "RX" | "NO_RX" | "NO_DATA", "frames": n, "bad": n, "err": "...", "targets": [...]}
stdin: JSON lines  {"op": "ref", "lat", "lon"} (the boat position: only the simulator uses it) / {"op": "stop"};
the process ends when stdin closes (the gateway's end) and takes the decoder it started with it.

  NO_RX    the receiver's port is not there (connection refused / decoder not running): "no receiver"
  NO_DATA  connected, but no frame for no_data_s: "no data" - NEVER shown as an empty, clear sky
  RX       frames are arriving (an aircraft-free sky with a working receiver still gives frames only when aircraft
           are heard, so a long quiet spell also reads NO_DATA: the safe wording)

It sends NOTHING anywhere but its stdout; it never touches the AURIX, a UAV or the boat.
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import adsb  # noqa: E402

lock = threading.Lock()
out_lock = threading.Lock()


def emit(obj):
    try:
        with out_lock:
            sys.stdout.write(json.dumps(obj) + "\n")
            sys.stdout.flush()
    except (OSError, ValueError):
        os._exit(0)


def low_priority():
    try:
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            k = ctypes.WinDLL("kernel32", use_last_error=True)
            k.GetCurrentProcess.restype = wintypes.HANDLE
            k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            return bool(k.SetPriorityClass(k.GetCurrentProcess(), 0x00004000))     # BELOW_NORMAL
        os.nice(10)
        return True
    except (OSError, AttributeError, ValueError):
        return False


class Reader(threading.Thread):
    def __init__(self, host, port, proto, targets):
        threading.Thread.__init__(self, daemon=True)
        self.host, self.port, self.proto, self.targets = host, port, proto, targets
        self.connected = False
        self.err = ""
        self.t_frame = None
        self.bad = 0
        self.stop = False

    def run(self):
        while not self.stop:
            try:
                s = socket.create_connection((self.host, self.port), timeout=3.0)
            except OSError as e:
                self.connected, self.err = False, "%s:%d %s" % (self.host, self.port, e.__class__.__name__)
                time.sleep(2.0)
                continue
            self.connected, self.err = True, ""
            s.settimeout(1.0)
            beast, buf = adsb.BeastParser(), b""
            try:
                while not self.stop:
                    try:
                        data = s.recv(8192)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    now = time.time()
                    if self.proto == "beast":
                        ds = beast.feed(data, now)
                        self.bad = beast.bad
                    else:
                        buf += data
                        ds = []
                        while b"\n" in buf:
                            ln, buf = buf.split(b"\n", 1)
                            if not ln.strip():
                                continue
                            d = adsb.parse_sbs(ln)
                            if d is None:
                                self.bad += 1
                            else:
                                ds.append(d)
                        if len(buf) > 4096:                  # no line end for ages = garbage
                            buf = b""
                            self.bad += 1
                    if ds:
                        with lock:
                            for d in ds:
                                self.targets.update(d, now)
                        self.t_frame = now
            except OSError:
                pass
            finally:
                self.connected = False
                try:
                    s.close()
                except OSError:
                    pass
            time.sleep(1.0)


def parse_source(src):
    """"tcp:host:port" or "host:port" -> (host, port)."""
    s = src[4:] if src.startswith("tcp:") else src
    host, _c, port = s.rpartition(":")
    return host or "127.0.0.1", int(port)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="tcp:127.0.0.1:30003")
    ap.add_argument("--proto", default="sbs", choices=("sbs", "beast"))
    ap.add_argument("--decoder", default="", help="readsb / dump1090 to start (it is stopped with this process)")
    ap.add_argument("--decoder-args", default="")
    ap.add_argument("--sim", action="store_true", help="a simulated receiver inside this process (the demo)")
    ap.add_argument("--sim-start-m", type=float, default=4000.0)
    ap.add_argument("--cfg", default="{}", help="adsb.DEFAULTS overrides as JSON")
    a = ap.parse_args()
    cfg = adsb.merged(json.loads(a.cfg))
    prio = low_priority()
    targets = adsb.AdsbTargets(cfg)
    dec = None
    simd = None
    if a.sim:
        import simadsb
        simd = simadsb.AdsbSim(0, None, start_m=a.sim_start_m).start()
        host, port, proto = "127.0.0.1", simd.port, "sbs"
    else:
        host, port = parse_source(a.source)
        proto = a.proto
        if a.decoder:
            try:
                dec = subprocess.Popen([a.decoder] + a.decoder_args.split(), stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
            except OSError as e:
                emit({"ev": "hello", "decoder_error": str(e)})
    rd = Reader(host, port, proto, targets)
    rd.start()
    emit({"ev": "hello", "source": "sim" if a.sim else "%s:%d" % (host, port), "proto": proto, "low_priority": prio,
          "decoder": bool(dec)})

    def stdin_loop():
        try:
            for line in sys.stdin:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                if m.get("op") == "stop":
                    break
                if m.get("op") == "ref" and simd is not None:
                    try:
                        simd.set_origin(float(m["lat"]), float(m["lon"]))
                    except (KeyError, TypeError, ValueError):
                        pass
        except (OSError, ValueError):
            pass
        if dec is not None and dec.poll() is None:       # never leave the decoder behind
            try:
                dec.kill()
            except OSError:
                pass
        os._exit(0)
    threading.Thread(target=stdin_loop, daemon=True).start()
    t0 = time.time()
    try:
        while True:
            time.sleep(1.0)
            now = time.time()
            with lock:
                targets.tick(now)
                snap = targets.snapshot(now)
                frames = targets.frames
            if not rd.connected:
                st = "NO_RX"
            elif rd.t_frame is None or rd.t_frame < now - cfg["no_data_s"]:
                st = "NO_DATA"
            else:
                st = "RX"
            if dec is not None and dec.poll() is not None:
                st, rd.err = "NO_RX", "the decoder ended (code %s)" % dec.returncode
            emit({"ev": "state", "state": st, "frames": frames, "bad": rd.bad, "err": rd.err, "targets": snap})
    finally:
        rd.stop = True
        for p in (dec,):
            if p is not None and p.poll() is None:
                p.kill()
        if simd is not None:
            simd.stop()


if __name__ == "__main__":
    main()
