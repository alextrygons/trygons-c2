"""Network simulator for tests: a TCP proxy that behaves like a 4G, RF or
Starlink link between a station and the gateway.

TCP never loses data, it delays it: a lost packet is sent again after a
retransmission timeout, and everything behind it waits (in order). An
outage holds all data until the link is back. That is what the gateway and
the stations really see, so that is what this proxy does.

    p = NetSim(listen_port, target_port, PROFILES["starlink"]); p.start()
"""
import asyncio
import random
import threading
import time

PROFILES = {
    "lan":      {"delay_ms": 1, "jitter_ms": 0, "loss": 0.0},
    "4g":       {"delay_ms": 40, "jitter_ms": 15, "loss": 0.01},                 # ~80 ms round trip, 1 % loss
    "rf":       {"delay_ms": 25, "jitter_ms": 10, "loss": 0.005,
                 "outage_every_s": (6, 12), "outage_s": (0.5, 2.5)},            # dropouts
    "starlink": {"delay_ms": 25, "jitter_ms": 10, "loss": 0.002,
                 "outage_every_s": (8, 15), "outage_s": (2.0, 5.0)},            # ~50 ms, 2-5 s outages
    "slow":     {"delay_ms": 300, "jitter_ms": 50, "loss": 0.0},                # ~600 ms round trip
}


class NetSim:
    def __init__(self, listen_port, target_port, profile, seed=1, host="127.0.0.1"):
        self.listen_port = listen_port
        self.target_port = target_port
        self.host = host
        self.p = dict(profile)
        self.rng = random.Random(seed)
        self.loop = None
        self.server = None
        self.outages = []                # (start, end) wall times
        self.down_until = 0.0
        self.bytes = 0
        self.forced_down = False         # tests: cut the link completely

    # --- link state -------------------------------------------------------------------
    def _plan_outages(self, t0, span=3600.0):
        if "outage_every_s" not in self.p:
            return
        t = t0 + self.rng.uniform(*self.p["outage_every_s"])
        while t < t0 + span:
            d = self.rng.uniform(*self.p["outage_s"])
            self.outages.append((t, t + d))
            t += d + self.rng.uniform(*self.p["outage_every_s"])

    def link_back_at(self, now):
        for a, b in self.outages:
            if a <= now < b:
                return b
            if a > now:
                break
        return now

    def deliver_time(self, now):
        d = self.p["delay_ms"] + self.rng.uniform(-1, 1) * self.p.get("jitter_ms", 0)
        if self.rng.random() < self.p.get("loss", 0.0):
            d += self.rng.uniform(200, 600)                 # TCP retransmission
        return self.link_back_at(now) + max(0.0, d) / 1000.0

    # --- proxy ----------------------------------------------------------------------------
    async def _pipe(self, reader, writer):
        q = asyncio.Queue()
        last = [0.0]

        async def rx():
            while True:
                data = await reader.read(4096)
                if not data:
                    await q.put(None)
                    return
                now = time.time()
                at = max(last[0], self.deliver_time(now))
                last[0] = at
                await q.put((at, data))

        async def tx():
            while True:
                item = await q.get()
                if item is None:
                    break
                at, data = item
                while True:
                    wait = at - time.time()
                    if self.forced_down:
                        await asyncio.sleep(0.05)
                        continue
                    if wait <= 0:
                        break
                    await asyncio.sleep(min(wait, 0.05))
                writer.write(data)
                self.bytes += len(data)
                await writer.drain()
            writer.close()

        await asyncio.gather(rx(), tx(), return_exceptions=True)

    async def _client(self, creader, cwriter):
        try:
            sreader, swriter = await asyncio.open_connection(self.host, self.target_port)
        except OSError:
            cwriter.close()
            return
        await asyncio.gather(self._pipe(creader, swriter), self._pipe(sreader, cwriter), return_exceptions=True)

    async def _serve(self):
        self._plan_outages(time.time())
        self.server = await asyncio.start_server(self._client, self.host, self.listen_port)
        async with self.server:
            await self.server.serve_forever()

    def start(self):
        self.loop = asyncio.new_event_loop()
        def run():
            try:
                self.loop.run_until_complete(self._serve())
            except (asyncio.CancelledError, RuntimeError):
                pass                                    # stopped
        th = threading.Thread(target=run, daemon=True)
        th.start()
        time.sleep(0.2)
        return self

    def stop(self):
        if self.loop and self.server:
            self.loop.call_soon_threadsafe(self.server.close)
