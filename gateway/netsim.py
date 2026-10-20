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


class Uplink:
    """A shared, limited upload (the boat's 4G uplink): every connection that
    uses it queues behind the others (first in, first out)."""

    def __init__(self, kbps):
        self.rate = kbps * 125.0                # bytes / s
        self.free_at = 0.0
        self.lock = threading.Lock()

    def reserve(self, n, now):
        with self.lock:
            start = max(now, self.free_at)
            self.free_at = start + n / self.rate
            return self.free_at


class NetSim:
    def __init__(self, listen_port, target_port, profile, seed=1, host="127.0.0.1", uplink=None, up="s2c"):
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
        self.udp_dropped = 0
        self.udp_passed = 0
        self.uplink = uplink             # shared Uplink: the boat -> shore direction queues on it
        self.up = up                     # which direction is boat -> shore: "s2c" (station) / "c2s" (sync)

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

    def deliver_time(self, now, n=0, up=False):
        d = self.p["delay_ms"] + self.rng.uniform(-1, 1) * self.p.get("jitter_ms", 0)
        if self.rng.random() < self.p.get("loss", 0.0):
            d += self.rng.uniform(200, 600)                 # TCP retransmission
        if up and self.uplink:
            now = self.uplink.reserve(n, now)               # wait for the shared upload
        return self.link_back_at(now) + max(0.0, d) / 1000.0

    # --- proxy ----------------------------------------------------------------------------
    async def _pipe(self, reader, writer, up=False):
        q = asyncio.Queue()
        last = [0.0]

        async def rx():
            while True:
                data = await reader.read(4096)
                if not data:
                    await q.put(None)
                    return
                now = time.time()
                at = max(last[0], self.deliver_time(now, len(data), up))
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
        await asyncio.gather(self._pipe(creader, swriter, self.up == "c2s"),
                             self._pipe(sreader, cwriter, self.up == "s2c"), return_exceptions=True)

    async def _serve(self):
        self._plan_outages(time.time())
        loop = asyncio.get_running_loop()
        sim = self

        class Udp(asyncio.DatagramProtocol):
            def connection_made(self, tr):
                self.tr = tr

            def datagram_received(self, data, addr):
                now = time.time()
                if sim.forced_down or sim.link_back_at(now) > now or sim.rng.random() < sim.p.get("loss", 0.0):
                    sim.udp_dropped += 1
                    return                              # UDP: lost is lost
                d = sim.p["delay_ms"] + sim.rng.uniform(-1, 1) * sim.p.get("jitter_ms", 0)
                loop.call_later(max(0.0, d) / 1000.0, self.tr.sendto, data, (sim.host, sim.target_port))
                sim.udp_passed += 1

        self.udp_tr, _ = await loop.create_datagram_endpoint(Udp, local_addr=(self.host, self.listen_port))
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
