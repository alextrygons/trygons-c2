"""Trygons C2 gateway (the gateway PC: Ubuntu 24.04 LTS or Windows 11) - the
boat's door to the internet.

    python3 gateway.py --config /etc/trygons/gateway.json     (Ubuntu)
    python gateway.py --config %PROGRAMDATA%/trygons/gateway.json   (Windows)
    python3 gateway.py --sim                  (PC: talk to autopilot_sim server)
    python3 gateway.py --list-serial          (the stable names of the USB adapters)
    python3 gateway.py --config ... --check-config   (exit code 1 = an error in it)

The AURIX is never on the network: it talks only to this PC, over a UART
(a 3.3 V TTL USB-UART adapter - NEVER RS232),
with the same framed protocol (CRC) as the USB laptop link. This program:
  A  keeps the full copy of the AURIX log on the USB SSD, sets the clock from
     GPS, watches the SSD, sends a 1 Hz status/heartbeat to the AURIX
  (later sections add: stations + roles, internet channels, remote driving,
   cameras, recording, sync)
Runs as a systemd service (Ubuntu) or a Task Scheduler task (Windows, gateway/windows/;
see README.md). Every part keeps running when
another one fails; a problem is reported, never hidden.
"""
import argparse
import asyncio
import json
import os
import sys
import threading
import time

from common import Clock, load_config, log, proto, setup_logging
import osdep
from audit import Audit
from adsbmgr import AdsbManager
from harbour.manager import HarbourSensors
from harbour.planner import HarbourPlanner
from ais import AisTargets, OnlineFeed
from shallow import Shallow
from geofence import Geofence
from sarstore import SarStore
from trainlevels import AcademyLevels
from avoid import Avoider
ALM_FENCE = 16                      # navreq alarm bit: outside the geofence (app/smallboat.h SB_NAV_ALM_FENCE)
from vision.manager import VisionManager
from loudhailer import Loudhailer
from uav.manager import UavManager
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


SAR_STILL_QUEUE = 8          # PF4-r2 LOW-9: stills waiting for the one sar-still helper thread


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
        self.aurix.on(proto.T_HARBOUR_STAT, self._on_harbour_stat)
        self.telem = None               # latest AURIX telemetry (dict)
        self.telem_rx = 0.0             # when it arrived (time.time(); 13q2c LOW-1: telem_fresh)
        self.telem2 = {}
        self.extra_flags = 0            # set by later sections (channels, recording)
        self.channel_id = 0
        self.station_count = 0
        self.started = time.time()
        self._stop = None
        sec = cfg.get("secrets", {})
        self.users = Users(sec.get("users") or cfg.get("users", {}))
        # academy levels (CLAUDE.md S, step 16): a signed push from the shore PC's academy; the user list
        # above stays the authority; the last copy on the SSD; read at login only (never mid-trip)
        self.academy = AcademyLevels(os.path.join(self.storage.folder("state"), "academy_levels.json")
                                     if self.storage.usable else None,
                                     sec.get("academy_key") or cfg.get("academy_key"))
        self.users.academy = self.academy
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
        self.stations.ext["cam_view"] = self.cams.view_request     # the copilot's camera view (13n2)
        self.stations.ext["cam_internet"] = self._cam_internet
        self._saver_was = False
        self.stations.ext["channel"] = self._channel_msg
        self.stations.ext["channel_keep"] = self._channel_keep
        self.rec = Recorder(cfg["recording"], self)
        self.stations.ext["save"] = self._save_msg
        self.stations.ext["rec"] = self._rec_msg
        self.stations.ext["save_keep"] = self._keep_msg
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
        self.prog_kn = None             # ... its leg speeds (kn), same order (step HX5: the harbour route speed)
        self.prog_pending_kn = None
        self.prog_pending = None        # ... checked here, waiting for the AURIX's ACK of the PROG_COMMIT
        self.prog_ack_new = False       # a commit ACK came after the latest telemetry (prog_gw not shown yet)
        self.nav_seq = 0
        self.shore_now = (0, 0xFFFF)    # last GW_SHORE: (flags, distance m)
        self.nav_last = None
        self.nav_sent = 0
        self.nav_skipped = 0                # navreqs not sent: no fresh telemetry (step 13q2d)
        self.nav_errors = 0                 # nav_compose exceptions (step 13q2e)
        self.nav_state = {"shallow": None, "threats": [], "route": [], "state": "CLEAR", "why": ""}
        self.stations.prog_check = self.check_prog
        # geofence (CLAUDE.md D): routes must stay inside; a copilot's change waits for the captain's ACCEPT
        self.geofence = Geofence(os.path.join(self.storage.folder("state"), "geofence.json")
                                 if self.storage.usable else None)
        self.stations.ext["geofence"] = self._geofence_msg
        self.stations.ext["academy_levels"] = self._academy_msg
        # search and rescue markers (CLAUDE.md X, step PF3): bookkeeping only, never the boat
        self.sar = SarStore(os.path.join(self.storage.folder("state"), "sar_markers.json")
                            if self.storage.usable else None)
        self.stations.ext["sar_mark"] = self._sar_mark
        self.stations.ext["sar_get"] = self._sar_get
        self.stations.ext["sar_still"] = self._sar_still
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
        # loudhailer + microphone (step 13r, owner 2026-11-02): its own process, a notice only, never the AURIX
        self.hail = Loudhailer(cfg.get("loudhailer"), self)
        for k, fn in (("ptt", self.hail.ptt), ("hail_signal", self.hail.signal), ("hail_volume", self.hail.set_volume)):
            self.stations.ext[k] = fn
        if self.hail.track is not None and self.rec.enabled:
            self.rec.cams["AUDIO"] = self.hail.track    # both directions into the ring + every SAVE / KEEP event
        # UAV magazine (CLAUDE.md V, step U2): registry + slot USB checks + GPS warm start; its router = its own
        # process; a notice only, never the AURIX
        try:
            self.uav = UavManager(cfg.get("uav"), self)
        except Exception:                               # U3 review H1: the UAV part never stops the gateway
            log.exception("uav: the magazine could not be set up - switched off")
            self.uav = UavManager({"enabled": False}, self)
        self.stations.ext["uav_register"] = self.uav.register_msg
        # ADS-B (CLAUDE.md X K4, step PF5): receive only, its own process; judged against the flying UAVs
        try:
            self.adsb = AdsbManager(cfg.get("adsb"), self)
        except Exception:                               # the ADS-B part never stops the gateway
            log.exception("adsb: could not be set up - switched off")
            self.adsb = AdsbManager({"enabled": False}, self)
        # harbour sensors (CLAUDE.md Z, step HX3): radar + LiDAR workers (own processes; simulated until W148);
        # health = notices in gw_status "harbour" only - nothing to the AURIX, no tracker / planner yet
        try:
            self.harbour = HarbourSensors(cfg.get("harbour"), self)
        except Exception:                               # the harbour sensors never stop the gateway
            log.exception("harbour: the sensors could not be set up - switched off")
            self.harbour = HarbourSensors({"enabled": False}, self)
        # harbour planner (step HX5): the kind-4 navreq through nav_compose ONLY while the AURIX's HARBOUR_STAT says
        # harbour autonomy runs (or a checked harbour route waits for the captain's START)
        self.hplan = HarbourPlanner(self, cfg.get("harbour") if isinstance(cfg.get("harbour"), dict) else None)
        # U3: routes -> missions, the launch sequence (the captain's YES / NO), STOP, the captain's HATCH CLOSE
        for k, fn in (("uav_route", self.uav.route_msg), ("uav_launch_prepare", self.uav.prepare_msg),
                      ("uav_launch_stop", self.uav.stop_msg), ("uav_hatch_close", self.uav.hatch_close_msg),
                      ("uav_unregister", self.uav.unregister_msg)):
            self.stations.ext[k] = fn
        # U4: telemetry to the stations, the flight logs, the UAV video (MediaMTX uav_1..6 + the ring tracks UAV-n in
        # every SAVE / KEEP; a station pulls only the UAV streams open on its screens)
        self.hview_subs = set()                              # step HX6: stations that asked for harbour_view
        self.stations.ext["harbour_route"] = self.harbour_route_msg
        self.stations.ext["harbour_view_on"] = self.harbour_view_msg
        for k, fn in (("uav_telem_on", self.uav.telem_msg), ("uav_logs", self.uav.logs_msg),
                      ("uav_log", self.uav.logs_msg), ("uav_view", self.cams.uav_view_request)):
            self.stations.ext[k] = fn
        # U5: manual control + the pilot lock (uav_ctl also arrives over UDP, signed like rjoy: stations.udp_packet)
        for k, fn in (("uav_pilot", self.uav.pilots.pilot_msg), ("uav_ctl", self.uav.pilots.ctl),
                      ("uav_hold", self.uav.pilots.hold_msg)):
            self.stations.ext[k] = fn
        if self.uav.video_on:
            self.cams.uav_slots = sorted(int(s) for s in self.uav.vcfg["devices"])
        if self.rec.enabled:
            for name, track in self.uav.tracks.items():
                self.rec.cams[name] = track

    def _on_telem(self, f):
        try:
            d = proto.decode_telemetry(f.payload)
        except ValueError:
            return
        d.update(self.telem2)
        self.telem = d
        self.telem_rx = time.time()
        self.prog_ack_new = False           # (this telemetry was sent after the last PROG_COMMIT ACK: it knows)

    TELEM_MAX_AGE_S = 3.0

    def telem_fresh(self):
        """The AURIX telemetry if it is fresh (younger than TELEM_MAX_AGE_S and the AURIX link up), else None:
        an old copy is an UNKNOWN mode (step 13q2c, review LOW-1) - never a reason to skip the captain's ACCEPT."""
        if not self.telem or time.time() - self.telem_rx > self.TELEM_MAX_AGE_S:
            return None
        if not getattr(self.aurix, "connected", False):
            return None
        return self.telem

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
                  d.get("mode") in (proto.MODE_MANUAL, proto.MODE_HDG_LOCK, proto.MODE_AUTO, proto.MODE_REMOTE,
                                    proto.MODE_DP)):
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
        """The boat from the FRESH telemetry (telem_fresh, step 13q2d review LOW-1: a frozen copy never renews a
        'no danger' lease). No fresh telemetry = nothing known (no mode either); no position (pos_source 0) =
        lat None but the mode / profile still known (MED-1: UNCREWED HDG LOCK then = cannot evaluate = STOP)."""
        d = self.telem_fresh() or {}
        if not d:
            return {"lat": None, "fresh": False}
        o = {"lat": None, "fresh": True, "mode": d.get("mode"),
             "dp_sub": d.get("dp_sub") if d.get("mode") == proto.MODE_DP else None,
             "profile": (self.telem2 or {}).get("profile", d.get("profile")),
             "ov_hand": bool((self.telem2 or {}).get("ov_hand"))}   # 13q3b: the shore stick steers UNCREWED AUTO
        if d.get("pos_source", 0) == 0:
            return o
        o.update({"lat": d.get("est_lat"), "lon": d.get("est_lon"), "sog": d.get("est_sog"), "cog": d.get("est_cog"),
                  "heading": d.get("heading"), "unc": d.get("unc_m")})
        return o

    @staticmethod
    def hdg_uncrewed(o):
        """UNCREWED HDG LOCK (step 13q2c, owner 2026-11-01): the navigation request acts like AUTO but only slows /
        stops (never a heading); a geofence breach -> the AURIX goes to STANDBY."""
        return o.get("mode") == proto.MODE_HDG_LOCK and o.get("profile") == proto.PROFILE_UNCREWED

    @staticmethod
    def auto_uncrewed(o):
        """UNCREWED AUTO (step 13q3, owner 2026-10-31 C3): the captain may steer by hand with the shore stick; steering
        out of the geofence by hand -> the AURIX goes to STANDBY (the fence bit; ignored while AUTO steers itself)."""
        return o.get("mode") == proto.MODE_AUTO and o.get("profile") == proto.PROFILE_UNCREWED

    @classmethod
    def uh_strict(cls, o):
        """The UNCREWED HDG LOCK protections apply (step 13q3b, review MED-1): UNCREWED HDG LOCK, or UNCREWED AUTO
        while the captain steers it by hand with the shore stick (TELEM2 D b3) - a hand-steered course was never
        checked like AUTO's route: no data / too uncertain = STOP, shallow / the shore within stop_m ahead = STOP."""
        return cls.hdg_uncrewed(o) or (cls.auto_uncrewed(o) and bool(o.get("ov_hand")))

    def fence_out(self, o):
        """Outside the geofence, or closer to its edge than the position uncertainty (None = no fence / no position)."""
        if not self.geofence.poly or o.get("lat") is None:
            return None
        from geofence import inside, edge_m
        unc = o.get("unc") or 0.0
        return (not inside(self.geofence.poly, o["lat"], o["lon"]) or
                edge_m(self.geofence.poly, o["lat"], o["lon"]) <= unc)

    def uh_check(self, o):
        """UNCREWED HDG LOCK (step 13q2d, review MED-1 / MED-2): -> (why, fence_out). why = the reason to STOP
        (English) when the gateway cannot evaluate (no position, no charts, the position too uncertain) or shallow
        water / water without chart data lies within the position uncertainty; fence_out = outside the geofence
        or closer to its edge than the uncertainty (None = no fence / no position)."""
        unc = o.get("unc") or 0.0
        fence = self.fence_out(o)
        why = self.shallow.uh_unknown(o)
        if why:
            return why, fence
        if unc > 0.0 and self.shallow.near_m(o, unc) is not None:
            return "shallow water within the position uncertainty (%.0f m)" % unc, fence
        return None, fence

    def nav_auto(self, o):
        """The navigation request applies like in AUTO: AUTO, or a DP TRIP (CLAUDE.md P; the AURIX's nav_applies)."""
        return o.get("mode") == proto.MODE_AUTO or (o.get("mode") == proto.MODE_DP and o.get("dp_sub") == proto.DP_TRIP)

    def harbour_route_check(self, wps, corr_m):
        """A harbour route (CLAUDE.md Z, step HX6): the centre line AND both corridor edges (+-corr_m, default the
        gateway's harbour "corr_m" 10 m) must be deep enough on the charts - NO 50 m clearance (a harbour route
        runs close to the quay by nature; the planner / the AURIX keep the boat in the corridor) - and inside the
        geofence. No charts = refused (no data = shallow). -> None or (code, English text): 114 shallow, 120 fence,
        129 corridor."""
        import math as _m
        hc = self.cfg.get("harbour") if isinstance(self.cfg.get("harbour"), dict) else {}
        cmax = float(hc.get("corr_m", 10.0))                 # the widest corridor the boat may use (W146: +-10 m; the AURIX clamps to harb_corr_m)
        corr = float(corr_m if corr_m is not None else cmax)
        if not (_m.isfinite(corr) and 4.0 <= corr <= cmax):
            return 129, "corridor half-width %s m outside 4..%.0f m" % (corr_m, cmax)
        ch = self.shallow.charts
        if not ch:
            return 114, "no depth charts on the gateway (no data = shallow)"
        lim = self.shallow.limit()
        for k in range(len(wps) - 1):
            a, b = wps[k], wps[k + 1]
            bad = ch.check_leg(a[0], a[1], b[0], b[1], lim, 0.0, step_m=10.0)
            if bad:
                return 114, "leg WP%d -> WP%d: %s at %.5f %.5f (%.0f m along the leg)" % (k + 1, k + 2, bad[4], bad[1],
                                                                                          bad[2], bad[0])
            ky = 111320.0
            kx = ky * _m.cos(_m.radians((a[0] + b[0]) / 2.0))
            dx, dy = (b[1] - a[1]) * kx, (b[0] - a[0]) * ky
            n = _m.hypot(dx, dy)
            if n < 1e-6:
                continue
            ox, oy = -dy / n * corr, dx / n * corr         # the left normal x corr (metres)
            # the corridor as lines 0.5 x corr apart (review HX6 M1: a shoal between the centre and an edge)
            for f, side in ((1.0, "left edge"), (-1.0, "right edge"), (0.5, "left half"), (-0.5, "right half")):
                la1, lo1 = a[0] + f * oy / ky, a[1] + f * ox / kx
                la2, lo2 = b[0] + f * oy / ky, b[1] + f * ox / kx
                bad = ch.check_leg(la1, lo1, la2, lo2, lim, 0.0, step_m=5.0)
                if bad:
                    return 129, "corridor (%s, +-%.0f m) of leg WP%d -> WP%d: %s at %.5f %.5f" % (
                        side, corr, k + 1, k + 2, bad[4], bad[1], bad[2])
        # the wedge outside every bend: spokes from each waypoint to a ring of corr metres
        for k, w in enumerate(wps):
            for i in range(8):
                ang = _m.radians(45.0 * i)
                la2 = w[0] + corr * _m.cos(ang) / ky
                lo2 = w[1] + corr * _m.sin(ang) / (ky * max(0.2, _m.cos(_m.radians(w[0]))))
                bad = ch.check_leg(w[0], w[1], la2, lo2, lim, 0.0, step_m=5.0)
                if bad:
                    return 129, "corridor around WP%d (+-%.0f m): %s at %.5f %.5f" % (k + 1, corr, bad[4], bad[1], bad[2])
        why = self.geofence.check(wps)
        if why:
            return 120, why
        return None

    def harbour_route_msg(self, st, m):
        """{"t": "harbour_route", "op": "check", "wps": [{lat, lon, kn}], "corr_m"} -> {"t": "harbour_ack", "ok", "code",
        "why"} (HX6). READ ONLY: it stores nothing and sends nothing to the AURIX."""
        import math as _m
        ok, code, why = False, 129, "bad request"
        try:
            wl = m.get("wps")
            pts = [(float(w["lat"]), float(w["lon"])) for w in wl] if isinstance(wl, list) else []
            if not 2 <= len(pts) <= 50:
                why = "2..50 waypoints needed (have %d)" % len(pts)
            elif not all(_m.isfinite(la) and _m.isfinite(lo) and abs(la) <= 90 and abs(lo) <= 180 for la, lo in pts):
                why = "bad coordinates"
            else:
                r = self.harbour_route_check(pts, m.get("corr_m"))
                ok, code, why = (True, 0, "OK") if r is None else (False, r[0], r[1])
        except (TypeError, ValueError, KeyError):
            pass
        st.send({"t": "harbour_ack", "ok": ok, "code": code, "why": why, "op": m.get("op"), "seq": m.get("seq")})

    def harbour_view_msg(self, st, m):
        (self.hview_subs.add if m.get("on", True) else self.hview_subs.discard)(st.id)

    def harbour_view_push(self):
        """harbour_view (2.5 Hz) to the stations that asked: the fused tracks (compact) + the planner's decision. A
        read-only copy of what the gateway already holds; nothing in it is ever sent to the AURIX."""
        if not self.hview_subs or not getattr(self.harbour, "enabled", False):
            return
        msg = {"t": "harbour_view", "t_s": round(time.time(), 2), "tracks": self.harbour.view_tracks(),
               "plan": self.hplan.view()}
        for sid in list(self.hview_subs):
            st = self.stations.stations.get(sid)
            if st is None:
                self.hview_subs.discard(sid)
                continue
            st.send(msg)

    def check_prog(self, st, ftype, payload):
        """Program upload / AUTO start / AUTO return / DP TRIP start through the gateway: refused across shallow
        water (114) or out of the geofence (120). -> None (fine), the refusal text (114) or (code, text)."""
        import struct as _st
        from math import isfinite as _st_isfinite
        if ftype == proto.T_SET_MODE:
            return self.check_hdg_shore(st, payload)
        if ftype == proto.T_HARBOUR:
            if payload[:1] in (bytes([proto.HARB_START]), bytes([proto.HARB_RESUME])):
                self.hplan.note_start()                     # the planner's kind-4 pre-start window (HX5 review MED-2)
            return None
        if ftype == proto.T_PROG_BEGIN and len(payload) >= 2:
            st.prog_up = {"n": payload[0], "wps": {},
                          "harbour": len(payload) >= 3 and bool(payload[2] & 1)}     # HX2 flag byte (HX6: a harbour route)
        elif ftype == proto.T_PROG_WP and len(payload) >= 18 and getattr(st, "prog_up", None):
            i, la, lo, kn = _st.unpack_from("<Biif", payload)
            st.prog_up["wps"][i] = (la / 1e7, lo / 1e7)
            st.prog_up.setdefault("kn", {})[i] = kn
        elif ftype == proto.T_PROG_COMMIT and getattr(st, "prog_up", None):
            up = st.prog_up
            wps = [up["wps"][k] for k in sorted(up["wps"])]
            if up.get("harbour"):                           # step HX6: a harbour route = the corridor check, no 50 m clearance
                kns = [k for k in (up.get("kn") or {}).values() if k is not None]
                hb = None
                if not 2 <= len(wps) <= 50 or any(not (_st_isfinite(k) and 0.5 <= k <= 8.0) for k in kns):
                    hb = (129, "a harbour route needs 2..50 waypoints and leg speeds 0.5..8 kn")      # review HX6 M2
                if hb is None:
                    hb = self.harbour_route_check(wps, None)
                if hb:
                    self.audit("program_refused", station=st.id, user=st.user, why=hb[1], harbour=True)
                    return hb
                bad = None
            else:
                bad = self.shallow.check_program(wps)
            if bad:
                self.audit("program_refused", station=st.id, user=st.user, why=bad[0])
                return bad[0]
            why = self.geofence.check(wps)
            if why:
                self.audit("program_refused", station=st.id, user=st.user, why=why)
                return 120, why
            self.prog_pending = wps                         # the AURIX's ACK makes it the program (prog_committed)
            self.prog_pending_kn = [up.get("kn", {}).get(k) for k in sorted(up["wps"])]
        elif ftype in (proto.T_AUTO_START, proto.T_AUTO_RESUME, proto.T_AUTO_RETURN) or (
                ftype == proto.T_DP and len(payload) >= 2 and payload[0] == proto.DP_START and
                payload[1] & 0x7F == proto.DP_TRIP):
            o, d = self.own(), self.telem or {}
            if o["lat"] is None:
                return None                                 # the AURIX refuses without a position anyway
            wps = self.prog_wps or []
            dp = ftype == proto.T_DP
            ret = ftype == proto.T_AUTO_RETURN
            if ret:
                # AUTO RETURN (CLAUDE.md R, step 13): the way back = the program's waypoints before the active one,
                # backwards (on the first leg: waypoint 1), the boat -> first of them included. Checked like a DP
                # TRIP: only a program the gateway itself checked and the AURIX still holds (121 otherwise)
                fresh = self.prog_ack_new
                if not wps or not (fresh or (d.get("prog_gw") and len(wps) == d.get("prog_n"))):
                    why = "upload the route again through the gateway (loaded over USB, from the boat's memory or " \
                          "before a gateway restart)"
                    self.audit("auto_return_refused", station=st.id, user=st.user, why=why)
                    return 121, why
                if d.get("prog_returning"):
                    return None                             # already on the way back: the AURIX refuses it
                k = min(d.get("auto_wp", 0) or 0, len(wps) - 1)
                rest = list(reversed(wps[:k])) if k > 0 else wps[:1]
            elif dp:
                # a DP TRIP (CLAUDE.md P) is checked like AUTO: from waypoint 1, or "continue" from the DP's waypoint;
                # the boat -> first waypoint leg included (start=). Only a program the gateway itself checked and
                # the AURIX still holds (loaded over the gateway link, same count) can be checked - a USB-loaded one,
                # one from the boat's memory or from before a gateway restart can't: the TRIP is refused (121)
                fresh = self.prog_ack_new                   # accepted just now: the telemetry is older
                if not wps or not (fresh or (d.get("prog_gw") and len(wps) == d.get("prog_n"))):
                    why = "upload the route again through the gateway (loaded over USB, from the boat's memory or "                           "before a gateway restart)"
                    self.audit("dp_trip_refused", station=st.id, user=st.user, why=why)
                    return 121, why
                k = (d.get("dp_wp", 0) or 0) if payload[1] & proto.DP_TRIP_CONTINUE else 0
                rest = wps[k:] if k < len(wps) else []
                if not rest:
                    why = "no waypoint left to continue from"
                    self.audit("dp_trip_refused", station=st.id, user=st.user, why=why)
                    return 121, why
            else:
                k = d.get("auto_wp", 0) or 0
                rest = wps[k:] if wps and k < len(wps) else ([(d["wp_lat"], d["wp_lon"])] if d.get("wp_set") else [])
            kind = "dp_trip_refused" if dp else "auto_return_refused" if ret else "auto_refused"
            bad = self.shallow.check_program(rest, start=(o["lat"], o["lon"])) if rest else None
            if bad:
                self.audit(kind, station=st.id, user=st.user, why=bad[0])
                return bad[0]
            why = self.geofence.check(rest, start=(o["lat"], o["lon"])) if rest else None
            if why:
                self.audit(kind, station=st.id, user=st.user, why=why)
                return 120, why
        return None

    def check_hdg_shore(self, st, payload):
        """SET_MODE HDG LOCK (step 13q2d, review MED-1): the UNCREWED entry from REMOTE / MANEUVER is refused like an
        AUTO start (114) when the gateway can't evaluate the water (no fresh telemetry / position, no charts, the
        position too uncertain) or shallow water / the shore is closer than stop_m ahead; 120 outside (or at the
        edge of) the geofence. CREWED (fresh telemetry) or another mode: the AURIX decides (it refuses those)."""
        if payload[:1] != bytes([proto.MODE_HDG_LOCK]):
            return None
        o = self.own()
        if o.get("fresh") and (o.get("profile") == proto.PROFILE_CREWED or
                               o.get("mode") not in (proto.MODE_REMOTE, proto.MODE_MANEUVER)):
            return None
        why, fence = (("no fresh telemetry from the boat", None) if not o.get("fresh") else self.uh_check(o))
        if not why and o["lat"] is not None:
            sh = self.shallow.watch(o)
            if sh and sh.get("dist_m", 1e9) <= self.shallow.cfg["stop_m"] + (o.get("unc") or 0.0):
                why = "shallow water / the shore %.0f m ahead (depth %s)" % (sh["dist_m"], sh.get("depth"))
        if why:
            self.audit("hdg_lock_refused", station=st.id, user=st.user, why=why)
            return why
        if fence:
            why = "outside / at the edge of the geofence"
            self.audit("hdg_lock_refused", station=st.id, user=st.user, why=why)
            return 120, why
        return None

    def prog_committed(self, result):
        """The AURIX's answer to a PROG_COMMIT sent through the gateway: accepted = the checked waypoints are
        the boat's program now (the DP TRIP / AUTO checks use them)."""
        if result == 0 and self.prog_pending is not None:
            self.prog_wps = self.prog_pending
            self.prog_kn = self.prog_pending_kn
            self.prog_ack_new = True        # the telemetry still in hand is older than this program
        self.prog_pending = None

    def _geofence_msg(self, st, m):
        """Set / change / clear the geofence from a station's map (CLAUDE.md D). A copilot's change waits
        for the captain's ACCEPT on screen 1; the captain's own goes straight through."""
        from geofence import valid
        ok, why, pts = valid(m.get("poly"))
        if not ok:
            return st.send({"t": "geofence_ack", "ok": False, "code": 0, "text": "geofence refused: " + why})
        what = str(m.get("what") or ("geofence: %d corners" % len(pts) if pts else "geofence: clear"))[:80]

        def done(yes, code):
            if yes:
                ok2, text = self.geofence.set(pts, st.user)
                self.audit("geofence", station=st.id, user=st.user, corners=len(pts), ok=ok2)
            else:
                from stations import GW_RESULTS
                ok2, text = False, "refused: %s" % GW_RESULTS.get(code, proto.RESULTS.get(code, code))
            st.send({"t": "geofence_ack", "ok": ok2, "code": code, "text": text})
        self.stations.ask_local(st, m, what, done)

    def _sar_list_msg(self):
        return {"t": "sar_markers", "rev": self.sar.rev, "list": self.sar.list(),
                "deleted": list(getattr(self.sar, "deleted", []))}

    def _sar_get(self, st, m):
        st.send(self._sar_list_msg())

    def _sar_mark(self, st, m):
        """A station's marker change (PF3). Any crew role (captain, copilot), never a VIEWER (128). It changes no
        driving, so no ACCEPT. The answer goes to the sender, the list to every station."""
        mk = m.get("m") if isinstance(m.get("m"), dict) else {}
        if st.role == "VIEWER":
            return st.send({"t": "sar_ack", "ok": False, "id": mk.get("id"), "code": 128, "text": "SAR: not allowed"})
        op = str(m.get("op") or "")
        ok, mid, text = self.sar.apply(op, mk, st.user)
        self.audit("sar", station=st.id, user=st.user, op=op, id=mid, ok=ok, text=text,
                   lat=mk.get("lat"), lon=mk.get("lon"))
        st.send({"t": "sar_ack", "ok": ok, "id": mid, "code": 0 if ok else 108, "text": text})
        if ok and text != "same":
            try:
                self.recorder.note_sar({"op": op, "id": mid, "lat": mk.get("lat"), "lon": mk.get("lon"),
                                        "kind": mk.get("kind"), "by": st.user})
            except Exception:
                pass
            msg = self._sar_list_msg()
            for s_ in list(self.stations.stations.values()):
                s_.send(msg)

    def _sar_still(self, st, m):
        """PF4: the small still image (<= 40 KB JPEG: one station line is 64 KB) of a point marked from the video -> <data>/sar/<id>.jpg, linked to
        the marker and to every open SAVE event. Any crew role, never a VIEWER (128). Bookkeeping only."""
        import base64
        mid = str(m.get("id") or "")
        res = {"t": "sar_ack", "what": "still", "id": mid}
        if st.role == "VIEWER":
            return st.send(dict(res, ok=False, code=128, text="SAR: not allowed"))
        try:
            jpg = base64.b64decode(str(m.get("jpg_b64") or ""), validate=True)
        except (ValueError, TypeError):
            jpg = b""
        if not jpg or len(jpg) > 40 * 1024 or jpg[:2] != b"\xff\xd8":
            return st.send(dict(res, ok=False, code=108, text="bad still image"))
        if not any(mk["id"] == mid for mk in self.sar.list()):
            return st.send(dict(res, ok=False, code=108, text="no such marker"))
        name = mid + ".jpg"
        # PF4-r MED-5 (the 13n4b rule): the file is written by a HELPER THREAD, never on the asyncio loop; the marker /
        # audit / answers follow on the loop once it is written
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None

        def done(err):
            if err:
                return st.send(dict(res, ok=False, code=108, text="still not stored: %s" % err))
            self.sar.set_still(mid, name)
            self.audit("sar", station=st.id, user=st.user, op="still", id=mid, ok=True, bytes=len(jpg))
            try:
                self.recorder.note_sar({"op": "still", "id": mid, "file": "sar_" + name, "by": st.user}, jpg, name)
            except Exception:
                pass
            st.send(dict(res, ok=True, code=0, text="still stored"))
            msg = self._sar_list_msg()
            for s_ in list(self.stations.stations.values()):
                s_.send(msg)

        def write():
            err = None
            try:
                folder = self.storage.folder("sar") if self.storage is not None and self.storage.usable else None
                if folder:
                    path = os.path.join(folder, name)
                    tmp = path + ".tmp"
                    with open(tmp, "wb") as f:              # PF4-r2 LOW-9: a temp file + rename (never half a file)
                        f.write(jpg)
                    os.replace(tmp, path)
            except OSError as e:
                err = str(e) or "OSError"
            if loop is not None:
                loop.call_soon_threadsafe(done, err)
            else:
                done(err)
        # PF4-r2 LOW-9 (review of 64521b1): ONE helper thread with a bounded queue (never a thread per message; the
        # stills are written one after another = never two writers of one file); a full queue = refused + counted
        q = self.__dict__.get("sar_still_q")
        if q is None:
            import queue as _queue
            q = self.__dict__.setdefault("sar_still_q", _queue.Queue(maxsize=SAR_STILL_QUEUE))

            def worker():
                while True:
                    job = q.get()
                    try:
                        job()
                    except Exception as e:                  # a bookkeeping error never stops the worker
                        log.warning("sar_still: %s", e)
            threading.Thread(target=worker, name="sar-still", daemon=True).start()
        try:
            q.put_nowait(write)
        except Exception:                                   # queue.Full
            self.sar_still_dropped = getattr(self, "sar_still_dropped", 0) + 1
            log.warning("sar_still %s refused: %d stills waiting (dropped %d)", mid, SAR_STILL_QUEUE,
                        self.sar_still_dropped)
            return st.send(dict(res, ok=False, code=108, text="still not stored: busy (too many stills waiting)"))
        self.sar_still_writes = getattr(self, "sar_still_writes", 0) + 1

    def _academy_msg(self, st, m):
        """Academy levels from a station (CLAUDE.md S, step 16): the file is signed by the shore PC's academy
        (trainlevels.py checks it); only a HELM-role user's station may hand it in; known users only; it
        changes nothing for a station already logged in (next login)."""
        if st.role != "HELM":
            ok, text, n, ign = False, "academy levels refused: HELM users only", 0, 0
        else:
            ok, text, n, ign = self.academy.apply(m.get("doc"), set(self.users.users), by=st.user)
        self.audit("academy_levels", station=st.id, user=st.user, ok=ok, users=n, ignored=ign, text=text)
        st.send({"t": "academy_levels_ack", "ok": ok, "text": text, "n": n, "ignored": ign})

    def nav_compose(self):
        """The navigation request for the AURIX: collision avoidance (section D) + shallow water
        (section C). Heading from the avoidance, the LOWER speed cap, all alarms."""
        o = self.own()
        now = time.time()
        if o.get("mode") != proto.MODE_HDG_LOCK and self.avoider.hdg_set is not None:
            self.avoider.hdg_set_sent()                     # left HDG LOCK: an accepted heading is dropped
            self.hdg_set_until = 0.0
        uh = self.hdg_uncrewed(o)
        strict = self.uh_strict(o)                         # 13q3b MED-1: also UNCREWED AUTO steered by hand
        req = self.avoider.update(now, o, self.ais.list(), o.get("mode"), o.get("dp_sub"), uncrewed=uh)
        if req["kind"] == proto.NAV_HDG_SET:
            if not self.hdg_set_until:
                self.hdg_set_until = now + 3.0              # repeated for 3 s (same number: applied once)
            elif now > self.hdg_set_until:
                self.avoider.hdg_set_sent()
                self.hdg_set_until = 0.0
        sh = self.shallow.watch(o) if o["lat"] is not None else None
        uh_why, fence = self.uh_check(o) if strict else (None, self.fence_out(o) if self.auto_uncrewed(o) else None)
        if uh_why:
            # UNCREWED HDG LOCK (or UNCREWED AUTO steered by hand, 13q3b) and the gateway can't evaluate (no position /
            # charts / too uncertain) or shallow water within the uncertainty: no data = shallow = STOP (never a 'no
            # danger' lease built on missing data)
            sh = dict(sh or {}, why=uh_why, dist_m=0.0)
        if sh and (self.nav_auto(o) or uh):                # AUTO / a DP TRIP / UNCREWED HDG LOCK: slow down (SHALLOW)
            slow = self.shallow.cfg["slow_kn"]
            if strict and (uh_why or sh.get("dist_m", 1e9) <= self.shallow.cfg["stop_m"] + (o.get("unc") or 0.0)):
                slow = 0.0                                 # shallow / shore close ahead = STOP (UNCREWED, by hand too)
            req["cap_kn"] = min(req["cap_kn"] if req["cap_kn"] is not None else 1e9, slow)
            if req["kind"] == proto.NAV_NONE:
                req["kind"] = proto.NAV_CAP
            req["reason"] |= 2
            req["alarm"] |= 4
        # camera obstacle detection: only a speed cap (the lower one wins) + the OBSTACLE AHEAD alarm
        v_cap, v_alarm, v_why = self.vision.nav(o.get("mode"), hdg_uncrewed=uh)
        if v_cap is not None:
            req["cap_kn"] = min(req["cap_kn"] if req["cap_kn"] is not None else 1e9, v_cap)
            if req["kind"] == proto.NAV_NONE:
                req["kind"] = proto.NAV_CAP
        req["alarm"] |= v_alarm
        req["reason"] |= v_why
        # UNCREWED HDG LOCK outside the geofence: the AURIX goes to STANDBY (alarm bit 16); step 13q3: also UNCREWED AUTO
        # (STANDBY only while the captain steers by hand, ignored while AUTO steers itself); ignored in other modes
        if fence:                                          # (uh_check: outside, or nearer the edge than the uncertainty)
            req["alarm"] |= ALM_FENCE
        self.nav_state = dict(self.avoider.status(), shallow=sh, fence_out=fence)
        # harbour autonomy (step HX5): every cap above folds into the ONE kind-4 request while the AURIX runs it
        hp = getattr(self, "hplan", None)                  # (a test stub has no harbour planner)
        return hp.fold(req, o) if hp is not None else req

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
        """The captain at an open screen 1 (CLAUDE.md D), or a single operator."""
        return (st in self.stations.drivers() and st.crew == "captain") or st.single_op

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
            return st.send({"t": "vision_ack", "ok": False, "text": "only the captain's screen 1 clears a slowdown"})
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
        """The captain's ACCEPT / REJECT of a HEADING LOCK proposal: only from the captain's
        screen 1 (CLAUDE.md D), or a station in SINGLE-OPERATOR mode."""
        if not ((st in self.stations.drivers() and st.crew == "captain") or st.single_op):
            return st.send({"t": "avoid_ack", "ok": False, "text": "only the captain's screen 1 answers proposals"})
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
        key = (req["kind"], req["heading"] and round(req["heading"], 1), req["cap_kn"], req["reason"], req["alarm"],
               req.get("hflags"))
        if key != self.nav_last:
            self.nav_seq = (self.nav_seq + 1) & 0xFF
            self.nav_last = key
            self.audit("navreq", seq=self.nav_seq, nav_kind=req["kind"], heading=req["heading"], cap_kn=req["cap_kn"],
                       reason=req["reason"], alarm=req["alarm"])
        if req["kind"] == proto.NAV_HARBOUR:
            self.aurix.send(proto.T_GW_NAVREQ, proto.p_gw_navreq_harbour(
                self.nav_seq, req["reason"], req["alarm"], req["heading"], req["cap_kn"], req["lease_ms"],
                req["hflags"], req["nearest_m"], req["n_tracks"]))
            self.nav_sent += 1
            return
        self.aurix.send(proto.T_GW_NAVREQ, proto.p_gw_navreq(self.nav_seq, req["kind"], req["reason"], req["alarm"],
                                                             req["heading"], req["cap_kn"], 3000))
        self.nav_sent += 1

    def nav_tick(self):
        """One navigation request. No fresh AURIX telemetry = nothing is known, so no lease is renewed (step 13q2d,
        review LOW-1): the AURIX's lease runs out (a slowdown held, DRIVER DECISION; in UNCREWED HDG LOCK every lapsed
        lease) instead of a 'no danger' built on old data. -> True when sent.
        Step 13q2e (review): nav_compose failing (a bug, bad chart data ...) is logged + counted; in UNCREWED HDG
        LOCK the gateway then sends an explicit STOP (cap 0, SHALLOW + DRIVER DECISION = "cannot evaluate") instead
        of leaving the lease un-renewed; elsewhere (AUTO / DP TRIP) nothing is sent = the AURIX's lost-lease rule."""
        try:
            req = self.nav_compose()
        except Exception:
            self.nav_errors += 1
            log.exception("navigation request: nav_compose failed")
            o = self.own()
            if not (o.get("fresh") and self.uh_strict(o)):     # (13q3b: UNCREWED AUTO steered by hand too)
                raise
            req = {"kind": proto.NAV_CAP, "heading": None, "cap_kn": 0.0, "reason": 2, "alarm": 4 | 2}
            self.audit("navreq_error", text="nav_compose failed: cannot evaluate = STOP (UNCREWED HDG LOCK / AUTO steered by hand)")
        if self.telem_fresh() is None:
            self.nav_skipped += 1
            return False
        self.nav_send(req)
        return True

    async def nav_loop(self):
        while True:
            try:
                self.nav_tick()
            except Exception:
                log.exception("navigation request")
            await asyncio.sleep(self.cfg.get("nav_s", 0.5))

    def _on_harbour_stat(self, f):
        try:
            self.hplan.on_stat(proto.decode_harbour_stat(f.payload))
        except ValueError:
            return

    def _on_telem2(self, f):
        self.telem2 = proto.decode_telem2(f.payload)
        self.rec.on_telem2(self.telem2)

    # --- recording (section F) ------------------------------------------------------
    def _save_msg(self, st, m):
        """SAVE button of a station (captain / driver / shore joystick): never a VIEWER."""
        if st.role == "VIEWER":
            return st.send({"t": "save_ack", "ok": False, "text": "VIEWER: no SAVE"})
        src = str(m.get("source", "button"))[:20]
        ok, text, eid = self.rec.trigger("mob" if src == "mob" else "button", "SAVE %s (%s)" % (src, st.user),
                                         who=st.user, eid=m.get("id"))
        # "merged" (review 13n4b L10): the station's id joined an OPEN camera event - not just a "-2" name suffix
        st.send({"t": "save_ack", "ok": ok, "text": text, "id": eid, "ask": m.get("id"),
                 "merged": bool(ok and self.rec.last_merged)})

    def _keep_msg(self, st, m):
        """KEEP 5' / 1h of a station (step 13n4): the last 5 / 60 min of every camera, permanent, not uploaded.
        Never a VIEWER; nothing here touches the boat."""
        ask = m.get("id") if isinstance(m.get("id"), str) else None
        if st.role == "VIEWER":
            return st.send({"t": "keep_ack", "ok": False, "text": "VIEWER: no SAVE", "id": None, "ask": ask})
        try:
            minutes = int(m.get("min"))
        except (TypeError, ValueError):
            minutes = None
        ok, text, eid, have = self.rec.keep(minutes, who=st.user, eid=ask)
        st.send({"t": "keep_ack", "ok": ok, "text": text, "id": eid, "ask": ask, "min": minutes, "have_min": have})

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

    async def hail_loop(self):
        while True:
            try:
                self.hail.tick()
            except Exception:                               # the loudhailer never stops the gateway
                log.exception("loudhailer")
            await asyncio.sleep(0.25)

    async def adsb_loop(self):
        while True:
            try:
                self.adsb.tick()
            except Exception:                               # ADS-B never stops the gateway
                log.exception("adsb")
            await asyncio.sleep(1.0)

    async def harbour_loop(self):
        while True:
            try:
                self.harbour.tick()
            except Exception:                               # the harbour sensors never stop the gateway
                log.exception("harbour")
            try:
                if self.hplan.active():
                    self.nav_tick()                         # step HX5: the kind-4 request at 5 Hz (lease 1 s)
            except Exception:
                log.exception("harbour planner")
            try:
                n = getattr(self, "_hview_n", 0) + 1
                self._hview_n = n
                if n % 2 == 0:
                    self.harbour_view_push()                # step HX6: 2.5 Hz display copy (read only)
            except Exception:
                log.exception("harbour view")
            await asyncio.sleep(0.2)

    async def uav_loop(self):
        while True:
            try:
                self.uav.tick()
            except Exception:                               # the UAV magazine never stops the gateway
                log.exception("uav")
            await asyncio.sleep(0.25)

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
            prio = self.cams.update_priority()      # the captain's video first: other views dropped first (13n2)
            if saver != self._saver_was or prio:    # data saver on / off, views reduced / back: new stream lists
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
            helm_id, helm_age = self.stations.helm_age()     # the LINK TIME rule on the AURIX (step 13q1)
            self.aurix.send(proto.T_GW_STATUS, proto.p_gw_status(
                self.status_flags(), self.channel_id, self.station_count,
                self.logs.last_seq or 0, self.logs.gaps, helm_id, helm_age))
            self.shore_send()
            self.stations.tick()
            await asyncio.sleep(self.cfg.get("status_s", 1.0))

    def shore_send(self):
        """GW_SHORE once a second: the distance to the shore / shallow water (UNCREWED lost-link
        rule, CLAUDE.md B). Missing on the AURIX = near shore (fail safe)."""
        try:
            self.shore_now = self.shallow.shore(self.own())
        except Exception:
            log.exception("shore distance")
            self.shore_now = (0, 0xFFFF)
        self.aurix.send(proto.T_GW_SHORE, proto.p_gw_shore(*self.shore_now))

    def status(self):
        """Everything the ground stations show about the gateway."""
        return {"uptime_s": int(time.time() - self.started), "aurix": self.aurix.connected,
                "storage": self.storage.status(), "logs": self.logs.status(),
                "clock": {"gps_valid": self.clock.gps_valid, "offset_s": round(self.clock.offset, 3),
                          "utc": self.clock.stamp()},
                "net": self.net.info(), "cameras": self.cams.status(), "rec": self.rec.status(),
                "sync": self.sync.status(), "ais": self.ais_status(self.net.data_saver()),
                "avoid": dict(self.nav_state, shallow_limit_m=self.shallow.limit(), charts=bool(self.charts)),
                "shore": {"flags": self.shore_now[0], "dist_m": self.shore_now[1]},
                "geofence": self.geofence.status(),
                "academy": self.academy.status(),
                "relays": self.stations.relay_status(),
                "vision": self.vision.status(self.net.data_saver()),
                "hail": self.hail.status(),
                "uav": self.uav.status(), "sar": self.sar.status(),
                "adsb": self.adsb.status(self.net.data_saver()),
                "harbour": dict(self.harbour.status(), planner=self.hplan.status())}

    async def run(self):
        self._stop = asyncio.Event()
        await self.stations.start()
        await self.web.start()
        if self.cfg["cameras"].get("enabled", True):
            self.cams.start("http://127.0.0.1:%d/mtxauth" % self.cfg.get("web_port", 8080))
        self.audit("gateway_start", boat=self.cfg.get("boat"))
        tasks = [asyncio.create_task(self.aurix.run()), asyncio.create_task(self.status_loop())]
        rec_loop = False
        if self.cfg["cameras"].get("enabled", True):
            tasks.append(asyncio.create_task(self.camera_loop()))
            if self.rec.enabled:
                self.rec.start(self.cfg["cameras"]["rtsp_port"],
                               {n: path_name(n, "HIGH") for n in self.cams.cams})
                tasks.append(asyncio.create_task(self.recorder_loop()))
                rec_loop = True
        self.hail.start(asyncio.get_running_loop())
        tasks.append(asyncio.create_task(self.hail_loop()))
        self.uav.start(asyncio.get_running_loop())
        tasks.append(asyncio.create_task(self.uav_loop()))
        tasks.append(asyncio.create_task(self.adsb_loop()))
        if self.harbour.enabled:
            tasks.append(asyncio.create_task(self.harbour_loop()))
        if not rec_loop and ("AUDIO" in self.rec.cams or self.uav.tracks):
            tasks.append(asyncio.create_task(self.recorder_loop()))     # the audio / UAV ring without cameras
        tasks.append(asyncio.create_task(self.sync.run()))
        tasks.append(asyncio.create_task(self.ais_loop()))
        tasks.append(asyncio.create_task(self.nav_loop()))
        tasks.append(asyncio.create_task(self.vision_loop()))
        if self.ais_reader:
            tasks.append(asyncio.create_task(self.ais_reader.run()))
        if self.cfg.get("print_status_s"):
            tasks.append(asyncio.create_task(self.print_status_loop(self.cfg["print_status_s"])))
        tasks += self.extra_tasks()
        try:
            await self._stop.wait()
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            self.vision.stop()
            self.hail.stop()
            self.uav.stop()
            self.adsb.stop()
            self.harbour.stop()
            await self.stations.stop()
            await self.web.stop()
            self.sync._close()
            self.rec.stop()
            self.cams.stop()
            self.logs.close()

    def extra_tasks(self):
        return []

    async def print_status_loop(self, every_s):
        """--status: one summary line in the log every every_s seconds (also used by
        tools/gateway_load.py to read the vision frame rate while it measures CPU / RAM)."""
        while True:
            await asyncio.sleep(every_s)
            try:
                v = self.vision.status(True)
                cams = self.cams.status() if self.cfg["cameras"].get("enabled", True) else {}
                log.info("STATUS aurix %s, storage %s, cameras %s, vision %s fps %.2f infer_ms %s lag_ms %s",
                         "connected" if self.aurix.connected else "NOT connected (%s)" % (self.aurix.error or "-"),
                         self.storage.state, "%d/%d OK" % (sum(1 for c in cams.get("cameras", []) if c["state"] == "OK"),
                                                           len(cams.get("cameras", []))),
                         v.get("state"), v.get("fps") or 0.0, v.get("infer_ms"),
                         int(getattr(self.vision, "lag_ms", 0) or 0))
                u = self.uav
                if u.enabled:                       # (U4: tools/gateway_load.py --uav sorts the processes by these)
                    ai = u.ai.status()
                    log.info("UAVSTATUS pids router=%s sim=%s video=%s ai=%s; video %s encoder %s; uav_ai %s ai_fps %.2f "
                             "ai_ms %s frames %d",
                             *[getattr(p, "pid", 0) or 0 for p in (u.proc, u.sim_proc, u.vproc, u.ai.proc)],
                             ",".join("%s:%s" % (s, x.get("state")) for s, x in sorted(u.vstatus.items())) or "-",
                             u.v_hello.get("encoder"), ai.get("state"), ai.get("fps") or 0.0, ai.get("infer_ms"),
                             ai.get("frames") or 0)
            except Exception:
                log.exception("status line")

    def stop(self):
        if self._stop:
            self._stop.set()


# --- the AIS receiver (section A) ------------------------------------------------------------

class AisReader:
    """NMEA lines from the USB AIS receiver (a stable name like the AURIX's:
    "usb:VID:PID:SERIAL", "/dev/serial/by-id/...", "COMn"; 38400 baud), or
    "tcp:host:port" (network receiver, tests). Reconnects by itself; every line goes to AisTargets.feed_line."""

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
        port = serial.Serial(osdep.resolve_serial(self.source), self.baud, timeout=0.5)
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
    ap.add_argument("--list-serial", action="store_true",
                    help="list the serial ports with the stable names for gateway.json, then exit")
    ap.add_argument("--check-config", action="store_true",
                    help="check gateway.json (exit code 1 = an error), then exit")
    a = ap.parse_args()
    setup_logging()
    if a.list_serial:
        try:
            for line in osdep.list_serial():
                print(line)
        except ImportError:
            print("pyserial is missing (Ubuntu: sudo apt install python3-serial; Windows: pip install pyserial)")
            sys.exit(1)
        return
    extra = {}
    if a.sim:
        extra = {"aurix": "tcp:127.0.0.1:5762", "set_clock": False,
                 "stations": {"listen": "127.0.0.1", "local_nets": ["127.0.0.1/32"]},
                 "channels": {"driver": "sim"}, "cameras": {"enabled": False}, "ais": {"enabled": False},
                 "loudhailer": {"backend": "sim"},
                 "audit_fallback": os.path.abspath("gw_data/audit_fallback"),
                 "storage": {"root": os.path.abspath("gw_data"), "require_mount": False}}
        os.makedirs("gw_data", exist_ok=True)
    if a.status:
        extra = dict(extra, print_status_s=60.0)
    cfg = load_config(a.config, extra)
    problems = osdep.check_config(cfg)
    for level, text in problems:
        (log.error if level == "error" else log.warning)("config: %s", text)
    if a.check_config:
        print("config %s: %d error(s), %d warning(s)" % (a.config or "(defaults)",
              sum(1 for p in problems if p[0] == "error"), sum(1 for p in problems if p[0] != "error")))
        sys.exit(1 if any(p[0] == "error" for p in problems) else 0)
    gw = Gateway(cfg)
    loop = asyncio.new_event_loop()
    osdep.install_stop(loop, gw.stop)
    try:
        loop.run_until_complete(gw.run())
    except KeyboardInterrupt:
        pass
    finally:
        print(json.dumps(gw.status(), indent=1))


if __name__ == "__main__":
    main()
