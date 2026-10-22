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
import threading
import time

from common import Clock, load_config, log, proto, setup_logging
from audit import Audit
from ais import AisTargets, OnlineFeed
from shallow import Shallow
from avoid import Avoider
from vision.manager import VisionManager
import chart
from cameras import CameraManager, path_name
from channels import ChannelManager, PolicyDriver, RutosDriver, SimDriver, DEFAULT_CHANNELS
from aurix import AurixLink
from logstore import LogStore
from recorder import Recorder
from sync import SyncClient
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
        self.rec = Recorder(cfg["recording"], self)
        self.stations.ext["save"] = self._save_msg
        self.stations.ext["rec"] = self._rec_msg
        self.sync = SyncClient(cfg["sync"], self)
        # AIS (section A): receive only; the AURIX never parses AIS
        ac = cfg["ais"]
        self.ais = AisTargets(ac)
        self.ais_reader = AisReader(ac["source"], ac.get("baud", 38400), self.ais) if ac.get("enabled") else None
        self.ais_online = OnlineFeed(ac.get("online", {}), self.ais, self._ais_online_allowed)
        self.ais_started = time.time()
        self.ais_silent = False
        self.at_sea_until = 0.0
        self.gps2_sent = 0
        # depth charts + shallow-water checks (section C), navigation requests to the AURIX
        self.charts = chart.Charts(cfg.get("charts_dir"))
        self.shallow = Shallow(cfg.get("shallow"), self.charts)
        self.prog_wps = None            # the last AUTO program accepted through the gateway: [(lat, lon)]
        self.nav_seq = 0
        self.nav_last = None
        self.nav_sent = 0
        self.nav_state = {"shallow": None, "threats": [], "route": [], "state": "CLEAR", "why": ""}
        self.stations.prog_check = self.check_prog
        # collision avoidance (section D): every decision into the audit log
        self.avoider = Avoider(cfg.get("avoid"), self.shallow,
                               log=lambda text, **kw: self.audit("avoid", text=text, **kw))
        self.hdg_set_until = 0.0
        self.stations.ext["avoid_answer"] = self._avoid_answer
        # camera obstacle detection (Part 2): its own process, never steers, only a speed cap
        self.vision = VisionManager(cfg.get("vision"), self)
        for k, fn in (("vision_stage", self._vision_stage), ("vision_clear", self._vision_clear),
                      ("vision_ack", self._vision_ack), ("vision_inject", self._vision_inject)):
            self.stations.ext[k] = fn

    def _on_telem(self, f):
        try:
            d = proto.decode_telemetry(f.payload)
        except ValueError:
            return
        d.update(self.telem2)
        self.telem = d

    # --- AIS (section A) ---------------------------------------------------------------
    def _ais_online_allowed(self):
        if not self.net.sync_allowed():
            return False, "channel %s (metered / emergency)" % (self.net.active or "none")
        return True, ""

    def at_sea(self):
        """Moving or in a driving mode now, or within the last 5 minutes."""
        d = self.telem or {}
        now = time.time()
        if d and ((d.get("est_sog") or 0.0) > self.cfg["ais"].get("at_sea_kn", 2.0) or
                  d.get("mode") in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK, proto.MODE_AUTO, proto.MODE_REMOTE)):
            self.at_sea_until = now + 300.0
        return now < self.at_sea_until

    def ais_tick(self):
        ac = self.cfg["ais"]
        self.ais.tick()
        silent = bool(self.ais_reader) and self.at_sea() and \
            self.ais.silent_for(self.ais_started) > ac.get("silent_s", 300.0)
        if silent != self.ais_silent:
            self.ais_silent = silent
            self.audit("ais_silent", on=silent, silent_s=round(self.ais.silent_for(self.ais_started)))
            log.warning("AIS receiver %s", "SILENT" if silent else "receiving again")
        g = self.ais.gps2
        if ac.get("second_gps") and g and time.time() - g["t"] < 2.0:
            self.aurix.send(proto.T_GW_GPS2, proto.p_gw_gps2(g["lat"], g["lon"], (time.time() - g["t"]) * 1000))
            self.gps2_sent += 1

    async def ais_loop(self):
        loop = asyncio.get_running_loop()
        while True:
            self.ais_tick()
            d = self.telem or {}
            if self.ais_online.cfg.get("enabled"):
                await loop.run_in_executor(None, self.ais_online.tick, d.get("est_lat"), d.get("est_lon"))
            await asyncio.sleep(self.cfg["ais"].get("tick_s", 1.0))

    def ais_status(self, saver=False):
        """For the stations: receiver state + the nearest targets (compact)."""
        st = self.ais.status()
        st["receiver"] = {"enabled": bool(self.ais_reader), "connected": bool(self.ais_reader and self.ais_reader.connected),
                          "source": self.cfg["ais"]["source"], "error": self.ais_reader.error if self.ais_reader else "",
                          "silent": self.ais_silent}
        st["online"] = self.ais_online.status()
        d = self.telem or {}
        lst = self.ais.list()
        if d.get("est_lat") is not None:
            from ais import dist_m
            for t in lst:
                t["dist_nm"] = round(dist_m(d["est_lat"], d["est_lon"], t["lat"], t["lon"]) / 1852.0, 2)
            lst.sort(key=lambda t: t["dist_nm"])
        st["list"] = lst[:15 if saver else self.cfg["ais"].get("max_listed", 40)]
        return st

    # --- shallow water (section C) + navigation requests ------------------------------
    def own(self):
        d = self.telem or {}
        if not d or d.get("pos_source", 0) == 0:
            return {"lat": None}
        return {"lat": d.get("est_lat"), "lon": d.get("est_lon"), "sog": d.get("est_sog"), "cog": d.get("est_cog"),
                "heading": d.get("heading"), "mode": d.get("mode")}

    def check_prog(self, st, ftype, payload):
        """Program upload / AUTO start through the gateway: refused across shallow water.
        -> None (fine) or the refusal text."""
        import struct as _st
        if ftype == proto.T_PROG_BEGIN and len(payload) >= 2:
            st.prog_up = {"n": payload[0], "wps": {}}
        elif ftype == proto.T_PROG_WP and len(payload) >= 18 and getattr(st, "prog_up", None):
            i, la, lo = _st.unpack_from("<Bii", payload)
            st.prog_up["wps"][i] = (la / 1e7, lo / 1e7)
        elif ftype == proto.T_PROG_COMMIT and getattr(st, "prog_up", None):
            up = st.prog_up
            wps = [up["wps"][k] for k in sorted(up["wps"])]
            bad = self.shallow.check_program(wps)
            if bad:
                self.audit("program_refused", station=st.id, user=st.user, why=bad[0])
                return bad[0]
            self.prog_wps = wps
        elif ftype in (proto.T_AUTO_START, proto.T_AUTO_RESUME):
            o, d = self.own(), self.telem or {}
            if o["lat"] is None:
                return None                                 # the AURIX refuses without a position anyway
            wps = self.prog_wps or []
            k = d.get("auto_wp", 0) or 0
            rest = wps[k:] if wps and k < len(wps) else ([(d["wp_lat"], d["wp_lon"])] if d.get("wp_set") else [])
            bad = self.shallow.check_program(rest, start=(o["lat"], o["lon"])) if rest else None
            if bad:
                self.audit("auto_refused", station=st.id, user=st.user, why=bad[0])
                return bad[0]
        return None

    def nav_compose(self):
        """The navigation request for the AURIX: collision avoidance (section D) + shallow water
        (section C). Heading from the avoidance, the LOWER speed cap, all alarms."""
        o = self.own()
        now = time.time()
        if o.get("mode") != proto.MODE_HDG_LOCK and self.avoider.hdg_set is not None:
            self.avoider.hdg_set_sent()                     # left HDG LOCK: an accepted heading is dropped
            self.hdg_set_until = 0.0
        req = self.avoider.update(now, o, self.ais.list(), o.get("mode"))
        if req["kind"] == proto.NAV_HDG_SET:
            if not self.hdg_set_until:
                self.hdg_set_until = now + 3.0              # repeated for 3 s (same number: applied once)
            elif now > self.hdg_set_until:
                self.avoider.hdg_set_sent()
                self.hdg_set_until = 0.0
        sh = self.shallow.watch(o) if o["lat"] is not None else None
        if sh and o.get("mode") == proto.MODE_AUTO:
            req["cap_kn"] = min(req["cap_kn"] if req["cap_kn"] is not None else 1e9, self.shallow.cfg["slow_kn"])
            if req["kind"] == proto.NAV_NONE:
                req["kind"] = proto.NAV_CAP
            req["reason"] |= 2
            req["alarm"] |= 4
        # camera obstacle detection: only a speed cap (the lower one wins) + the OBSTACLE AHEAD alarm
        v_cap, v_alarm, v_why = self.vision.nav(o.get("mode"))
        if v_cap is not None:
            req["cap_kn"] = min(req["cap_kn"] if req["cap_kn"] is not None else 1e9, v_cap)
            if req["kind"] == proto.NAV_NONE:
                req["kind"] = proto.NAV_CAP
        req["alarm"] |= v_alarm
        req["reason"] |= v_why
        self.nav_state = dict(self.avoider.status(), shallow=sh)
        return req

    # --- camera obstacle detection (Part 2) -------------------------------------------
    async def vision_loop(self):
        last = time.time()
        while True:
            await asyncio.sleep(0.25)
            now = time.time()
            lag = max(0.0, (now - last - 0.25) * 1000.0)   # how late the event loop is (the main jobs)
            last = now
            try:
                self.vision.tick(now, lag)
            except Exception:
                log.exception("vision")

    def _vision_driver(self, st):
        return st in self.stations.drivers() or st.single_op

    def _vision_stage(self, st, m):
        """Stage on / off from a station. STAGE 3 ON (it slows AUTO) needs the driver's ACCEPT."""
        name, on = m.get("stage"), bool(m.get("on"))
        if st.role == "VIEWER":
            return st.send({"t": "vision_ack", "ok": False, "text": "VIEWER: no settings"})

        def done(ok, code):
            if ok:
                ok2, text = self.vision.set_stage(name, on, st.user)
            else:
                from stations import GW_RESULTS
                ok2, text = False, "refused: %s" % GW_RESULTS.get(code, proto.RESULTS.get(code, code))
            st.send({"t": "vision_ack", "ok": ok2, "text": text})
        if name == "stage3" and on:
            return self.stations.ask_local(st, m, "vision STAGE 3 ON (obstacles slow AUTO down)", done)
        done(True, 0)

    def _vision_clear(self, st, m):
        if not self._vision_driver(st):
            return st.send({"t": "vision_ack", "ok": False, "text": "only the driver screen clears a slowdown"})
        ok, text = self.vision.clear(st.user)
        st.send({"t": "vision_ack", "ok": ok, "text": text})

    def _vision_ack(self, st, m):
        if self._vision_driver(st):
            self.vision.ack_proposal(m.get("id"), st.user)

    def _vision_inject(self, st, m):
        ok = self.vision.inject(m.get("rel", 0.0), m.get("dist", 100.0), m.get("seconds", 10.0),
                                m.get("conf", 0.9), m.get("frames"))
        st.send({"t": "vision_ack", "ok": ok, "text": "injected" if ok else "injection not allowed (vision.allow_inject)"})

    def _avoid_answer(self, st, m):
        """The driver's ACCEPT / REJECT of a HEADING LOCK proposal: only from a station that
        shows the DRIVER screen, or the captain in SINGLE-OPERATOR mode."""
        if st not in self.stations.drivers() and not st.single_op:
            return st.send({"t": "avoid_ack", "ok": False, "text": "only the driver screen answers proposals"})
        pid = m.get("id")
        if m.get("yes"):
            # the way back = the heading the driver had LOCKED (not the momentary compass heading)
            d = self.telem or {}
            back = d.get("target") if d.get("mode") == proto.MODE_HDG_LOCK and d.get("target") is not None \
                else d.get("heading", 0.0)
            ok, text = self.avoider.accept(pid, back)
        else:
            ok, text = self.avoider.reject(pid, time.time()), "rejected - heading unchanged"
        self.audit("avoid_answer", station=st.id, user=st.user, id=pid, yes=bool(m.get("yes")), ok=ok, text=text)
        st.send({"t": "avoid_ack", "ok": ok, "text": text})

    def nav_send(self, req):
        key = (req["kind"], req["heading"] and round(req["heading"], 1), req["cap_kn"], req["reason"], req["alarm"])
        if key != self.nav_last:
            self.nav_seq = (self.nav_seq + 1) & 0xFF
            self.nav_last = key
            self.audit("navreq", seq=self.nav_seq, nav_kind=req["kind"], heading=req["heading"], cap_kn=req["cap_kn"],
                       reason=req["reason"], alarm=req["alarm"])
        self.aurix.send(proto.T_GW_NAVREQ, proto.p_gw_navreq(self.nav_seq, req["kind"], req["reason"], req["alarm"],
                                                             req["heading"], req["cap_kn"], 3000))
        self.nav_sent += 1

    async def nav_loop(self):
        while True:
            try:
                self.nav_send(self.nav_compose())
            except Exception:
                log.exception("navigation request")
            await asyncio.sleep(self.cfg.get("nav_s", 0.5))

    def _on_telem2(self, f):
        self.telem2 = proto.decode_telem2(f.payload)
        self.rec.on_telem2(self.telem2)

    # --- recording (section F) ------------------------------------------------------
    def _save_msg(self, st, m):
        """SAVE button of a station (captain / driver / shore joystick): never a VIEWER."""
        if st.role == "VIEWER":
            return st.send({"t": "save_ack", "ok": False, "text": "VIEWER: no SAVE"})
        src = str(m.get("source", "button"))[:20]
        ok, text, eid = self.rec.trigger("button", "SAVE %s (%s)" % (src, st.user), who=st.user)
        st.send({"t": "save_ack", "ok": ok, "text": text, "id": eid})

    def _rec_msg(self, st, m):
        if not self.may_change(st):
            return st.send({"t": "save_ack", "ok": False, "text": "ring length: HELM or a station on the boat"})
        try:
            minutes = float(m.get("ring_min"))
            minutes = int(minutes) if minutes == int(minutes) else minutes
        except (TypeError, ValueError):
            minutes = None
        ok, text = self.rec.set_ring(minutes, st.user)
        st.send({"t": "save_ack", "ok": ok, "text": text, "id": None})

    async def recorder_loop(self):
        loop = asyncio.get_running_loop()
        while True:
            try:
                await loop.run_in_executor(None, self.rec.tick)
            except Exception as e:                          # never let recording stop the gateway
                log.warning("recorder: %s", e)
            await asyncio.sleep(1.0)

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
        if self.ais_silent:
            flags |= proto.GWF_AIS_SILENT
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
                "net": self.net.info(), "cameras": self.cams.status(), "rec": self.rec.status(),
                "sync": self.sync.status(), "ais": self.ais_status(self.net.data_saver()),
                "avoid": dict(self.nav_state, shallow_limit_m=self.shallow.limit(), charts=bool(self.charts)),
                "vision": self.vision.status(self.net.data_saver())}

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
            if self.rec.enabled:
                self.rec.start(self.cfg["cameras"]["rtsp_port"],
                               {n: path_name(n, "HIGH") for n in self.cams.cams})
                tasks.append(asyncio.create_task(self.recorder_loop()))
        tasks.append(asyncio.create_task(self.sync.run()))
        tasks.append(asyncio.create_task(self.ais_loop()))
        tasks.append(asyncio.create_task(self.nav_loop()))
        tasks.append(asyncio.create_task(self.vision_loop()))
        if self.ais_reader:
            tasks.append(asyncio.create_task(self.ais_reader.run()))
        tasks += self.extra_tasks()
        try:
            await self._stop.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            self.vision.stop()
            await self.stations.stop()
            await self.web.stop()
            self.sync._close()
            self.rec.stop()
            self.cams.stop()
            self.logs.close()

    def extra_tasks(self):
        return []

    def stop(self):
        if self._stop:
            self._stop.set()


# --- the AIS receiver (section A) ------------------------------------------------------------

class AisReader:
    """NMEA lines from the AIS receiver: "/dev/serial0" (dAISy HAT, 38400),
    "/dev/ttyUSB0" (USB receiver), or "tcp:host:port" (network receiver,
    tests). Reconnects by itself; every line goes to AisTargets.feed_line."""

    def __init__(self, source, baud, targets):
        self.source = source
        self.baud = baud
        self.targets = targets
        self.connected = False
        self.lines = 0
        self.error = ""

    async def run(self):
        while True:
            try:
                if self.source.startswith("tcp:"):
                    _, host, port = self.source.split(":")
                    reader, writer = await asyncio.open_connection(host, int(port))
                    self.connected, self.error = True, ""
                    try:
                        while True:
                            line = await reader.readline()
                            if not line:
                                raise ConnectionError("AIS source closed")
                            self.lines += 1
                            self.targets.feed_line(line.decode("ascii", "replace"))
                    finally:
                        writer.close()
                else:
                    await self._serial()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.error = str(e)[:80]
            self.connected = False
            await asyncio.sleep(2.0)

    async def _serial(self):
        import serial                                   # pyserial
        loop = asyncio.get_running_loop()
        port = serial.Serial(self.source, self.baud, timeout=0.5)
        self.connected, self.error = True, ""
        dead = asyncio.Event()

        def reader():
            try:
                while True:
                    line = port.readline()
                    if line:
                        self.lines += 1
                        loop.call_soon_threadsafe(self.targets.feed_line, line.decode("ascii", "replace"))
            except Exception as e:
                self.error = str(e)[:80]
            loop.call_soon_threadsafe(dead.set)
        threading.Thread(target=reader, daemon=True).start()
        try:
            await dead.wait()
        finally:
            port.close()


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
                 "channels": {"driver": "sim"}, "cameras": {"enabled": False}, "ais": {"enabled": False},
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
