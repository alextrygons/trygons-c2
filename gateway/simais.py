"""Simulated AIS traffic for the tests and the simulator (section E1).

    sim = AisSim(port).start()          a TCP server: the gateway's AIS reader
                                         (ais.source = "tcp:127.0.0.1:<port>") connects
    sim.add(Ship(mmsi, lat, lon, sog, cog, cls="A", name="..."))
    sim.ship(mmsi).cog = 120            change course / speed at any time
    sim.silent = True                    the receiver stops (antenna cable)
    for sh in traffic(lat, lon, 12): sim.add(sh)     a realistic mix around a point

Stand-alone (traffic for "autopilot_sim server" + the gateway, ais.source =
"tcp:127.0.0.1:5790"):
    python simais.py [--port 5790] [--lat 60.0 --lon 25.0] [--ships 12]
                     [--seed 1] [--drop 0.05] [--bad 0.01]

Report intervals like ITU-R M.1371: Class A every 10 s (0-14 kn), 6 s
(14-23 kn), 2 s (above), 3 min at anchor (< 0.5 kn); Class B every 30 s
(> 2 kn) or 3 min; static data (type 5 / 24) every 6 min. Noise: position
+-noise_m, speed +-0.1 kn, course +-1 deg; drop = part of the messages lost;
bad = part of the lines with a broken checksum.
"""
import math
import random
import socket
import threading
import time

from ais import move, sentences

# (kind, class, speed range kn, length m, AIS ship type, share of the traffic)
KINDS = (("FERRY", "A", (14.0, 19.0), 120, 60, 0.20), ("FAST FERRY", "A", (24.0, 32.0), 60, 40, 0.08),
         ("CARGO", "A", (10.0, 14.0), 150, 70, 0.14), ("TANKER", "A", (9.0, 13.0), 180, 80, 0.08),
         ("YACHT", "B", (4.0, 7.5), 13, 36, 0.22), ("MOTORBOAT", "B", (12.0, 25.0), 9, 37, 0.13),
         ("FISHING", "B", (2.0, 8.0), 15, 30, 0.10), ("AT ANCHOR", "A", (0.0, 0.2), 90, 70, 0.05))
NAMES = ("AEGEAN", "SARONIC", "POSEIDON", "ELENI", "KALLISTI", "MELTEMI", "ARTEMIS", "NIKOS", "THALASSA",
         "DELOS", "HYDRA", "AEGINA", "SOUNION", "KEA", "MAKRONISOS", "PORTO", "IRIS", "ANNA")


class Ship:
    def __init__(self, mmsi, lat, lon, sog, cog, cls="A", name="", length=60, beam=12, ship_type=70, turn_dpm=0.0):
        self.mmsi, self.lat, self.lon, self.sog, self.cog = mmsi, lat, lon, sog, cog
        self.cls, self.name, self.length, self.beam, self.ship_type = cls, name, length, beam, ship_type
        self.turn_dpm = turn_dpm                # deg per minute (+ = starboard)
        self.next_pos = 0.0
        self.next_static = 0.0
        self.sent = 0

    def interval(self):
        if self.cls == "A":
            if self.sog < 0.5:
                return 180.0
            return 10.0 if self.sog <= 14 else 6.0 if self.sog <= 23 else 2.0
        return 30.0 if self.sog > 2 else 180.0

    def step(self, dt):
        self.cog = (self.cog + self.turn_dpm * dt / 60.0) % 360.0
        self.lat, self.lon = move(self.lat, self.lon, self.cog, self.sog, dt)


def traffic(lat, lon, n, seed=1, radius_nm=6.0):
    """n ships within radius_nm of (lat, lon): ferries / cargo / tankers (Class A), yachts / motorboats /
    fishing boats (Class B), a few at anchor; random positions and courses, speeds of their kind,
    fishing boats turning slowly."""
    rng = random.Random(seed)
    ships = []
    for i in range(n):
        r = rng.random()
        for kind, cls, (v0, v1), length, stype, share in KINDS:
            r -= share
            if r <= 0:
                break
        d = radius_nm * 1852.0 * math.sqrt(rng.random())
        la, lo = move(lat, lon, rng.uniform(0, 360), d / 0.514444, 1.0)
        name = "%s %s" % (rng.choice(NAMES), kind if kind != "AT ANCHOR" else "STAR")
        mmsi = (237000000 if cls == "A" else 239000000) + rng.randrange(100000, 999999)
        turn = rng.uniform(-6, 6) if kind == "FISHING" else 0.0
        ships.append(Ship(mmsi, la, lo, round(rng.uniform(v0, v1), 1), rng.uniform(0, 360), cls=cls, name=name,
                          length=length, beam=max(3, length // 6), ship_type=stype, turn_dpm=turn))
    return ships


class AisSim:
    def __init__(self, port, seed=1, noise_m=8.0, drop=0.0, bad=0.0, speed=1.0, start_offset=None):
        self.port = port
        self.rng = random.Random(seed)
        self.noise_m, self.drop, self.bad, self.speed = noise_m, drop, bad, speed
        self.ships = {}
        self.clients = []
        self.lock = threading.Lock()
        self.running = False
        self.silent = False
        self.extra = []                         # lines to send once (tests: AIVDO, RMC, raw)
        self.sent = 0
        self.dropped = 0
        self.t = time.time()
        self.seq = 0
        self.start_offset = start_offset        # spread the first reports (s); None = random

    def add(self, ship):
        with self.lock:
            ship.next_pos = time.time() + (self.rng.uniform(0, ship.interval()) if self.start_offset is None
                                           else self.start_offset)
            ship.next_static = ship.next_pos + 1.0
            self.ships[ship.mmsi] = ship
        return ship

    def ship(self, mmsi):
        return self.ships[mmsi]

    def remove(self, mmsi):
        with self.lock:
            self.ships.pop(mmsi, None)

    def send_raw(self, line):
        with self.lock:
            self.extra.append(line)

    def start(self):
        self.running = True
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(4)
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def stop(self):
        self.running = False
        try:
            self.srv.close()
        except OSError:
            pass
        for c in self.clients:
            try:
                c.close()
            except OSError:
                pass

    def _accept(self):
        while self.running:
            try:
                c, _ = self.srv.accept()
                self.clients.append(c)
            except OSError:
                return

    def _emit(self, lines):
        data = "".join(l + "\r\n" for l in lines).encode()
        for c in list(self.clients):
            try:
                c.sendall(data)
            except OSError:
                self.clients.remove(c)

    def _noisy(self, s):
        dlat = self.rng.gauss(0, self.noise_m) / 111320.0
        dlon = self.rng.gauss(0, self.noise_m) / (111320.0 * math.cos(math.radians(s.lat)))
        return {"lat": s.lat + dlat, "lon": s.lon + dlon, "sog": max(0.0, s.sog + self.rng.gauss(0, 0.1)),
                "cog": (s.cog + self.rng.gauss(0, 1.0)) % 360.0, "heading": int(round(s.cog)) % 360}

    def _corrupt(self, line):
        body, cs = line.rsplit("*", 1)
        return body + "*%02X" % ((int(cs, 16) + 1) & 0xFF)

    def _run(self):
        last = time.time()
        while self.running:
            now = time.time()
            dt = (now - last) * self.speed
            last = now
            out = []
            with self.lock:
                out += self.extra
                self.extra = []
                for s in self.ships.values():
                    s.step(dt)
                    if self.silent:
                        continue
                    if now >= s.next_pos:
                        s.next_pos = now + s.interval() / self.speed
                        m = dict(self._noisy(s), mmsi=s.mmsi, type=1 if s.cls == "A" else 18)
                        out += self._maybe(sentences(m, channel="AB"[s.sent % 2]))
                        s.sent += 1
                    if now >= s.next_static:
                        s.next_static = now + 360.0 / self.speed
                        self.seq += 1
                        if s.cls == "A":
                            m = {"type": 5, "mmsi": s.mmsi, "name": s.name, "callsign": "SIM%d" % (s.mmsi % 1000),
                                 "ship_type": s.ship_type, "to_bow": s.length * 2 // 3, "to_stern": s.length // 3,
                                 "to_port": s.beam // 2, "to_starboard": s.beam - s.beam // 2, "draught": 5.0,
                                 "destination": "PIRAEUS"}
                            out += self._maybe(sentences(m, seq_id=self.seq))
                        else:
                            out += self._maybe(sentences({"type": 24, "mmsi": s.mmsi, "part": 0, "name": s.name}))
                            out += self._maybe(sentences({"type": 24, "mmsi": s.mmsi, "part": 1, "ship_type": 37,
                                                          "to_bow": s.length // 2, "to_stern": s.length // 2,
                                                          "to_port": 2, "to_starboard": 2}))
            if out:
                self.sent += len(out)
                self._emit(out)
            time.sleep(0.05)

    def _maybe(self, lines):
        if self.drop and self.rng.random() < self.drop:
            self.dropped += 1
            return []
        if self.bad and self.rng.random() < self.bad:
            return [self._corrupt(lines[0])] + lines[1:]
        return lines



def main():
    import argparse
    ap = argparse.ArgumentParser(description="simulated AIS traffic on a TCP port (NMEA AIVDM)")
    ap.add_argument("--port", type=int, default=5790)
    ap.add_argument("--lat", type=float, default=60.0)          # the simulator's start (sim/sim_world.c)
    ap.add_argument("--lon", type=float, default=25.0)
    ap.add_argument("--ships", type=int, default=12)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--drop", type=float, default=0.05)
    ap.add_argument("--bad", type=float, default=0.01)
    a = ap.parse_args()
    sim = AisSim(a.port, seed=a.seed, drop=a.drop, bad=a.bad).start()
    for sh in traffic(a.lat, a.lon, a.ships, seed=a.seed):
        sim.add(sh)
        print("%9d  %-22s class %s  %4.1f kn  %3.0f deg" % (sh.mmsi, sh.name, sh.cls, sh.sog, sh.cog))
    print("AIS traffic on tcp port %d (gateway: \"ais\": {\"enabled\": true, \"source\": \"tcp:127.0.0.1:%d\"}),"
          " Ctrl+C stops" % (a.port, a.port))
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        sim.stop()


if __name__ == "__main__":
    main()
