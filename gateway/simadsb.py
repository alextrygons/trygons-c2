"""Simulated ADS-B traffic for the tests and the demo (CLAUDE.md X K4, step PF5) - like simais.py.

    sim = AdsbSim(port=0, origin=(lat, lon)).start()   an SBS (port 30003 style) TCP server on 127.0.0.1;
                                                         the worker connects: "tcp:127.0.0.1:<sim.port>"
    sim.add(Aircraft("4B1815", "HEL123", lat, lon, 400.0, 75.0, 270.0))
    sim.silent = True                                    the receiver sends nothing (a quiet sky / a dead antenna)
    sim.set_origin(lat, lon)                             where the scripted helicopter crosses (once)

The scripted helicopter (HELICOPTER, default): 400 ft, 75 kn, crosses the origin (= where the UAVs fly) from the east
to the west, 4 km each side, then starts again from the east after a pause. Stand-alone for `autopilot_sim server` use:
    python simadsb.py [--port 30003] [--lat 37.82 --lon 23.60] [--no-heli]
"""
import math
import socket
import threading
import time

KN = 1852.0 / 3600.0


def move(lat, lon, trk, gs_kn, dt):
    d = gs_kn * KN * dt
    t = math.radians(trk)
    return (lat + d * math.cos(t) / 111320.0,
            lon + d * math.sin(t) / (111320.0 * max(0.05, math.cos(math.radians(lat)))))


class Aircraft:
    def __init__(self, icao, cs, lat, lon, alt_ft, gs_kn, trk, vs_fpm=0.0):
        self.icao, self.cs, self.lat, self.lon = icao, cs, lat, lon
        self.alt_ft, self.gs_kn, self.trk, self.vs_fpm = alt_ft, gs_kn, trk, vs_fpm
        self.sent = 0

    def step(self, dt):
        self.lat, self.lon = move(self.lat, self.lon, self.trk, self.gs_kn, dt)
        self.alt_ft += self.vs_fpm * dt / 60.0

    def lines(self, now=None):
        t = time.gmtime(now or time.time())
        d, tm = time.strftime("%Y/%m/%d", t), time.strftime("%H:%M:%S.000", t)
        head = "MSG,%d,1,1,%s,1,%s,%s,%s,%s"
        out = [head % (1, self.icao, d, tm, d, tm) + ",%s,,,,,,,,,,,," % self.cs,
               head % (3, self.icao, d, tm, d, tm) + ",,%d,,,%.5f,%.5f,,,0,0,0,0" % (self.alt_ft, self.lat, self.lon),
               head % (4, self.icao, d, tm, d, tm) + ",,,%.0f,%.0f,,,%d,,0,0,0,0" % (self.gs_kn, self.trk, self.vs_fpm)]
        return out


class AdsbSim:
    def __init__(self, port=0, origin=None, heli=True, rate_s=1.0, bad=0.0, start_m=4000.0):
        self.port = port
        self.origin = origin
        self.start_m = float(start_m)       # the helicopter starts this far east of the origin
        self.heli_on = heli
        self.rate_s = rate_s
        self.silent = False
        self.bad = bad                       # share of the lines with a broken field count
        self.ac = {}
        self.clients = []
        self.lock = threading.Lock()
        self.stop_ev = threading.Event()
        self.srv = None
        self.heli = None
        self.pause = 0.0
        self.lines_sent = 0

    def add(self, a):
        with self.lock:
            self.ac[a.icao] = a
        return a

    def set_origin(self, lat, lon):
        if self.origin is None:
            self.origin = (lat, lon)

    def _start_heli(self):
        lat, lon = self.origin
        la, lo = move(lat, lon, 90.0, 75.0, self.start_m / (75.0 * KN))   # start_m east of the origin
        self.heli = Aircraft("4B1815", "HEL123", la, lo, 400.0, 75.0, 270.0)
        self.ac[self.heli.icao] = self.heli       # (tick() holds the lock)

    def start(self):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def _accept(self):
        self.srv.settimeout(0.5)
        while not self.stop_ev.is_set():
            try:
                c, _a = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self.lock:
                self.clients.append(c)

    def tick(self, dt):
        """One simulation step (the thread calls it; tests may call it directly)."""
        with self.lock:
            if self.heli_on and self.origin is not None:
                if self.heli is None and self.pause <= 0:
                    self._start_heli()
                elif self.heli is not None and dist(self.heli, self.origin) > max(4200.0, self.start_m + 200.0) and self.heli.lon < self.origin[1]:
                    self.ac.pop(self.heli.icao, None)
                    self.heli, self.pause = None, 30.0
                elif self.heli is None:
                    self.pause -= dt
            for a in self.ac.values():
                a.step(dt)

    def _run(self):
        import random
        rnd = random.Random(7)
        last = time.time()
        while not self.stop_ev.wait(self.rate_s):
            now = time.time()
            self.tick(now - last)
            last = now
            if self.silent:
                continue
            with self.lock:
                lines = [ln for a in self.ac.values() for ln in a.lines(now)]
                cl = list(self.clients)
            if self.bad:
                lines = [(ln.replace("MSG,", "MSG,9,", 1) if rnd.random() < self.bad else ln) for ln in lines]
            data = ("\r\n".join(lines) + "\r\n").encode("ascii") if lines else b""
            for c in cl:
                try:
                    c.sendall(data)
                except OSError:
                    with self.lock:
                        if c in self.clients:
                            self.clients.remove(c)
            self.lines_sent += len(lines)

    def stop(self):
        self.stop_ev.set()
        try:
            self.srv.close()
        except OSError:
            pass
        with self.lock:
            for c in self.clients:
                try:
                    c.close()
                except OSError:
                    pass
            self.clients = []


def dist(a, origin):
    kx = math.cos(math.radians(origin[0])) * 111320.0
    return math.hypot((a.lon - origin[1]) * kx, (a.lat - origin[0]) * 111320.0)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="simulated ADS-B (SBS) receiver for the demo")
    ap.add_argument("--port", type=int, default=30003)
    ap.add_argument("--lat", type=float, default=37.82)
    ap.add_argument("--lon", type=float, default=23.60)
    ap.add_argument("--no-heli", action="store_true")
    ap.add_argument("--stdin-eof-exit", action="store_true")
    a = ap.parse_args()
    sim = AdsbSim(a.port, (a.lat, a.lon), heli=not a.no_heli).start()
    print("simulated ADS-B (SBS) on 127.0.0.1:%d" % sim.port, flush=True)
    try:
        if a.stdin_eof_exit:
            import sys
            sys.stdin.read()
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    sim.stop()


if __name__ == "__main__":
    main()
