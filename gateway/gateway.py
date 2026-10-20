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
from cameras import CameraManager
from channels import ChannelManager, PolicyDriver, RutosDriver, SimDriver, DEFAULT_CHANNELS
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
        cc = cfg["channels"]
        lst = cc.get("list") or DEFAULT_CHANNELS
        chans = {c["name"]: c for c in lst}
        drv = {"rutos": lambda: RutosDriver(cc.get("router", {}), chans),
               "policy": lambda: PolicyDriver(cc.get("router", {}), chans),
               "sim": lambda: SimDriver(list(chans))}[cc.get("driver", "rutos")]()
        state = None
        if self.storage.usable:
            state = os.path.join(self.storage.folder("state"), "channels.json")
        self.net = ChannelManager(cc, drv, audit=self.audit, state_path=state)
        self.next_net = 0.0
        self.cams = CameraManager(cfg["cameras"], self)
        self.stations.ext["cams"] = self.cams.request
        self.stations.ext["cam_internet"] = self._cam_internet
        self._saver_was = False
        self.stations.ext["channel"] = self._channel_msg
        self.stations.ext["channel_keep"] = self._channel_keep

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
    # --- internet channels (section C) ---------------------------------------------
    def may_change(self, st):
        """The HELM station or a station on the boat; never a VIEWER."""
        if st.role == "VIEWER":
            return False
        return st.local or self.telem2.get("helm") == st.id

    def _channel_msg(self, st, m):
        ok, text, conf = self.net.request(m.get("mode"), m.get("channel"), user=st.user, local=st.local,
                                          may=self.may_change(st), confirm_emergency=bool(m.get("confirm")),
                                          switch_now=False)
        st.send({"t": "channel_ack", "ok": ok, "text": text, "needs_confirm": conf})
        if ok:      # answer first (the station's link may change), switch 1 s later
            self.next_net = time.time() + 1.0

    def _channel_keep(self, st, m):
        ok, text = self.net.keep(st.user)
        st.send({"t": "channel_ack", "ok": ok, "text": text, "needs_confirm": False})

    def data_saver(self):
        return self.net.data_saver()

    def _cam_internet(self, st, m):
        """Camera internet for a firmware update: only from a station on the boat."""
        if not st.local or st.role == "VIEWER":
            return st.send({"t": "cams_ack", "ok": False, "text": "camera internet only from a station on the boat"})
        minutes = max(0.0, min(120.0, float(m.get("minutes", 30))))
        self.cams.internet(bool(m.get("on")), minutes)
        st.send({"t": "cams_ack", "ok": True, "text": "camera internet %s" % (
            "ON for %g min (then it closes by itself)" % minutes if m.get("on") else "OFF")})

    async def camera_loop(self):
        loop = asyncio.get_running_loop()
        while True:
            await loop.run_in_executor(None, self.cams.poll)
            saver = self.data_saver()
            if saver != self._saver_was:            # data saver on / off: new stream lists
                self._saver_was = saver
                self.cams.kick_disallowed()
                for st in list(self.stations.stations.values()):
                    if st.id in self.cams.station_cams:
                        st.send(self.cams.streams_msg(st))
            await asyncio.sleep(0.25)

    def channel_text(self):
        n = self.net
        if not n.active:
            return "no internet"
        t = n.channels[n.active].get("label", n.active)
        return ("STARLINK - EMERGENCY LINK" if n.on_emergency else t) + " (%s)" % n.mode

    def status_flags(self):
        st = self.storage.state
        flags = self.extra_flags
        if self.net.fallback:
            flags |= proto.GWF_FALLBACK
        if self.net.on_emergency:
            flags |= proto.GWF_STARLINK
        if self.net.data_saver():
            flags |= proto.GWF_SAVER
        if self.storage.alarm:
            flags |= proto.GWF_STORAGE
        if st == "FULL":
            flags |= proto.GWF_STORAGE_FULL
        return flags

    async def status_loop(self):
        while True:
            if time.time() >= self.next_net:
                self.next_net = time.time() + self.cfg["channels"].get("check_s", 10.0)
                if isinstance(self.net.driver, SimDriver):
                    self.net.tick()
                else:                                   # SSH / ping take seconds: not in the event loop
                    await asyncio.get_running_loop().run_in_executor(None, self.net.tick)
                self.channel_id = self.net.channel_id()
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
                          "utc": self.clock.stamp()},
                "net": self.net.info(), "cameras": self.cams.status()}

    async def run(self):
        self._stop = asyncio.Event()
        await self.stations.start()
        await self.web.start()
        if self.cfg["cameras"].get("enabled", True):
            self.cams.start("http://127.0.0.1:%d/mtxauth" % self.cfg.get("web_port", 8080))
        self.audit("gateway_start", boat=self.cfg.get("boat"))
        tasks = [asyncio.create_task(self.aurix.run()), asyncio.create_task(self.status_loop())]
        if self.cfg["cameras"].get("enabled", True):
            tasks.append(asyncio.create_task(self.camera_loop()))
        tasks += self.extra_tasks()
        try:
            await self._stop.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await self.stations.stop()
            await self.web.stop()
            self.cams.stop()
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
                 "channels": {"driver": "sim"}, "cameras": {"enabled": False},
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
