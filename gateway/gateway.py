"""Trygons C2 gateway (Raspberry Pi 5) - the boat's door to the internet.

    python3 gateway.py --config /etc/trygons/gateway.json
    python3 gateway.py --sim                  (PC: talk to autopilot_sim server)

The AURIX is never on the network: it talks only to this Pi, over a UART,
with the same framed protocol (CRC) as the USB laptop link. This program:
  A  keeps the full copy of the AURIX log on the USB SSD, sets the clock from
     GPS, watches the SSD, sends a 1 Hz status/heartbeat to the AURIX
  (later sections add: stations + roles, internet channels, remote driving,
   cameras, recording, sync)
Runs as a systemd service (see README.md). Every part keeps running when
another one fails; a problem is reported, never hidden.
"""
import argparse
import asyncio
import json
import os
import signal
import sys
import time

from common import Clock, load_config, log, proto, setup_logging
from audit import Audit
from aurix import AurixLink
from logstore import LogStore
from stations import StationServer
from storage import Storage
from timesync import TimeSync
from users import Users
from web import WebServer


class Gateway:
    def __init__(self, cfg):
        self.cfg = cfg
        self.clock = Clock()
        self.storage = Storage(cfg["storage"])
        self.storage.check(force=True)
        self.aurix = AurixLink(cfg["aurix"], cfg.get("baud", 921600))
        self.time = TimeSync(self.clock, cfg.get("set_clock", True))
        self.logs = LogStore(self.storage, self.clock)
        self.aurix.on(proto.T_LOG_CHUNK, lambda f: self.logs.on_chunk(f.payload))
        self.aurix.on(proto.T_TIME, self.time.on_time)
        self.aurix.on(proto.T_TELEMETRY, self._on_telem)
        self.aurix.on(proto.T_TELEM2, self._on_telem2)
        self.telem = None               # latest AURIX telemetry (dict)
        self.telem2 = {}
        self.extra_flags = 0            # set by later sections (channels, recording)
        self.channel_id = 0
        self.station_count = 0
        self.started = time.time()
        self._stop = None
        sec = cfg.get("secrets", {})
        self.users = Users(sec.get("users") or cfg.get("users", {}))
        self.audit = Audit(self.storage, self.clock, cfg.get("audit_fallback"))
        self.stations = StationServer(self, cfg["stations"])
        self.web = WebServer(self, cfg)

    def _on_telem(self, f):
        try:
            d = proto.decode_telemetry(f.payload)
        except ValueError:
            return
        d.update(self.telem2)
        self.telem = d

    def _on_telem2(self, f):
        self.telem2 = proto.decode_telem2(f.payload)

    # --- the 1 Hz status to the AURIX (also its "gateway alive" heartbeat) ---------
    def status_flags(self):
        st = self.storage.state
        flags = self.extra_flags
        if self.storage.alarm:
            flags |= proto.GWF_STORAGE
        if st == "FULL":
            flags |= proto.GWF_STORAGE_FULL
        return flags

    async def status_loop(self):
        while True:
            self.storage.check()
            self.logs.flush()
            self.station_count = len(self.stations.stations)
            self.aurix.send(proto.T_GW_STATUS, proto.p_gw_status(
                self.status_flags(), self.channel_id, self.station_count,
                self.logs.last_seq or 0, self.logs.gaps))
            self.stations.tick()
            await asyncio.sleep(self.cfg.get("status_s", 1.0))

    def status(self):
        """Everything the ground stations show about the gateway."""
        return {"uptime_s": int(time.time() - self.started), "aurix": self.aurix.connected,
                "storage": self.storage.status(), "logs": self.logs.status(),
                "clock": {"gps_valid": self.clock.gps_valid, "offset_s": round(self.clock.offset, 3),
                          "utc": self.clock.stamp()}}

    async def run(self):
        self._stop = asyncio.Event()
        await self.stations.start()
        await self.web.start()
        self.audit("gateway_start", boat=self.cfg.get("boat"))
        tasks = [asyncio.create_task(self.aurix.run()), asyncio.create_task(self.status_loop())]
        tasks += self.extra_tasks()
        try:
            await self._stop.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await self.stations.stop()
            await self.web.stop()
            self.logs.close()

    def extra_tasks(self):
        return []

    def stop(self):
        if self._stop:
            self._stop.set()


def main():
    ap = argparse.ArgumentParser(description="Trygons C2 gateway")
    ap.add_argument("--config", help="gateway.json")
    ap.add_argument("--sim", action="store_true",
                    help="PC test: AURIX = autopilot_sim server on tcp:127.0.0.1:5762, data in ./gw_data")
    ap.add_argument("--status", action="store_true", help="print the status once a minute")
    a = ap.parse_args()
    setup_logging()
    extra = {}
    if a.sim:
        extra = {"aurix": "tcp:127.0.0.1:5762", "set_clock": False,
                 "stations": {"listen": "127.0.0.1", "local_nets": ["127.0.0.1/32"]},
                 "audit_fallback": os.path.abspath("gw_data/audit_fallback"),
                 "storage": {"root": os.path.abspath("gw_data"), "require_mount": False}}
        os.makedirs("gw_data", exist_ok=True)
    cfg = load_config(a.config, extra)
    gw = Gateway(cfg)
    loop = asyncio.new_event_loop()
    if sys.platform != "win32":
        for s in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(s, gw.stop)
    try:
        loop.run_until_complete(gw.run())
    except KeyboardInterrupt:
        pass
    finally:
        print(json.dumps(gw.status(), indent=1))


if __name__ == "__main__":
    main()
