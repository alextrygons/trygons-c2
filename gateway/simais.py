"""Simulated AIS traffic for the tests and the simulator (section E1).

    sim = AisSim(port).start()          a TCP server: the gateway's AIS reader
                                         (ais.source = "tcp:127.0.0.1:<port>") connects
    sim.add(Ship(mmsi, lat, lon, sog, cog, cls="A", name="..."))
    sim.ship(mmsi).cog = 120            change course / speed at any time
    sim.silent = True                    the receiver stops (antenna cable)

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
