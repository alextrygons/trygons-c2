"""Ground stations on the gateway (section B): login, roles, commands,
telemetry - over the VPN (remote) or the boat LAN (local).

Protocol: TCP port 5761, one JSON object per line (UTF-8).

  station -> gateway
    {"t":"login", "user":"alex", "password":"..." | "key":"...", "station":"shore PC", "ts":<unix s>}
    {"t":"cmd", "sid":"<session>", "seq":17, "ts":<unix s>, "type":16, "p":"<hex payload>"}
         any command of app/proto.h; HELM take/release = type 0x33
    {"t":"gpong", "id":5, "ts":<unix s>}          answer to the gateway's gping
    {"t":"tmode", "mode":"auto"|"full"|"lite"}    telemetry wish
    (sections C-I add their own: channel, cams, save, rjoy, ...)

  gateway -> station
    {"t":"welcome", "id":2, "role":"HELM", "local":false, "sid":"...", "max_age_s":2.0}
    {"t":"tm", "b":"<hex TELEMETRY>"}  {"t":"t2", "b":"<hex TELEM2>"}      (full rate, 10 Hz)
    {"t":"lite", "d":{...essential fields...}}                             (reduced, 2 Hz)
    {"t":"frame", "type":130, "b":"<hex>"}       other AURIX answers (parameter, program, ...)
    {"t":"ack", "seq":17, "type":16, "result":0, "text":"OK"}
    {"t":"gping", "id":5, "gw":<unix s>}         latency measurement, every second
    {"t":"status", ...}                          gateway status + stations + HELM, every second
    {"t":"tmode", "mode":"lite", "why":"latency 650 ms"}
    {"t":"error", "text":"..."}

Crew (CLAUDE.md section D):
  captain  holds a manual licence (users.py "licence"), sits at screen 1, holds HELM.
           Only the captain asks for REMOTE MANUAL / MANEUVER. Its own commands go
           straight to the AURIX.
  copilot  everything except manual driving. Its commands are sent to the AURIX on
           the captain's behalf (the HELM station's id); the ones that affect the
           driving (proto.DRIVING_CMDS) only after the captain's ACCEPT on screen 1
           ("driver_ask" to the captain's station). STANDBY and AUTO PAUSE (hold)
           always go straight through. A relay switch (CLAUDE.md O): engine group =
           the captain's ACCEPT, trim / extra straight - the group from the table the
           AURIX reports active (relaytab.py; unknown = engine).
  login {"crew": "captain" | "copilot"} asks for a role: captain needs the licence
  and is refused while another station is captain (-> copilot, welcome "crew_note").
  A station on the boat that takes HELM becomes the captain (on board wins); so does a
  licensed user who takes a FREE HELM (nobody holds it).
  Role swap {"t":"crew_swap"} -> the other side {"t":"crew_swap_ask"} -> its
  {"t":"crew_swap_answer","yes":true}: only if BOTH users hold the licence; HELM is
  handed to the new captain in ONE step (AURIX HELM action 2: no lost link).
  login {"boat": "<name>"}: refused if this gateway is another boat ("wrong boat").

Command checks (B2), BEFORE anything goes to the AURIX:
  - role VIEWER: refused (watch only)
  - session id must be this connection's (no replay from an old session)
  - seq must grow: equal = duplicate, smaller = out of order -> refused
  - ts: older than max_age_s (2 s) on the gateway's clock -> stale, refused.
    The station's clock offset is measured with every gping/gpong, so a
    wrong PC clock doesn't matter.
  - rate limit per station (10 / s, bursts of 20)
The AURIX then enforces HELM (B3): only the HELM station's commands run.
Answers to commands that didn't come from 0..100 are AURIX results
(proto.RESULTS); the gateway's own refusals are 100..: see GW_RESULTS.
"""
import asyncio
import collections
import hashlib
import hmac
import ipaddress
import json
import math
import os
import secrets
import statistics
import struct
import time

from common import log, proto
from relaytab import RelayTables

GW_RESULTS = {100: "VIEWER: watch only", 101: "duplicate command", 102: "command out of order",
              103: "stale command", 104: "timestamp in the future", 105: "rate limited",
              106: "AURIX not connected", 107: "wrong session", 108: "bad command",
              109: "driver screen not open", 110: "rejected by the driver", 111: "no answer from the driver (timeout)",
              112: "driver screen lost", 113: "single-operator mode: confirm on the captain screen first",
              114: "shallow water on the route",
              120: "the route leaves the geofence",
              121: "route not checked by the gateway",
              116: "no manual licence: REMOTE MANUAL / MANEUVER only for a licensed captain",
              117: "only the captain (or a station on the boat) holds HELM",
              118: "captain only: REMOTE MANUAL / MANEUVER / engine START / LINK TIME / power restore",
              119: "no captain holds HELM",
              # UAV magazine (CLAUDE.md V; 122 U2, 123-125 U3, 126 / 127 U5)
              122: "UAV not registered / its check failed",
              123: "UAV route refused (energy / range / altitude)",
              124: "UAV route in a no-fly zone / outside the air geofence",
              125: "UAV launch conditions not met",
              126: "the joystick is on a UAV",
              127: "another pilot flies this UAV"}
# copilot commands that never wait for the captain's ACCEPT (CLAUDE.md D: STANDBY, hold, AUTO return - the AURIX
# still wants the joystick CONFIRM for the return; DP STOP: proto.is_driving_cmd)
COPILOT_FREE = (proto.T_AUTO_PAUSE, proto.T_AUTO_RETURN)
READ_ONLY = (proto.T_PARAM_GET, proto.T_PROG_GET, proto.T_JOYMAP_GET, proto.T_JOY_RAW_GET)
LITE_FIELDS = ("t_ms", "mode", "mode_name", "heading", "target", "throttle", "gear", "gps_fix", "compass_ok",
               "est_lat", "est_lon", "est_sog", "est_cog", "unc_m", "pos_source", "alarm_bits", "power_fwd",
               "power_rev", "joy_status", "confirm_kind", "confirm_left_s", "helm", "helm_flags", "gw_state")


class Station:
    def __init__(self, sid, user, role, local, ip, name, writer):
        self.id = sid
        self.user = user
        self.role = role
        self.local = local
        self.ip = ip
        self.name = name or user
        self.writer = writer
        self.session = secrets.token_hex(8)
        self.udp_key = secrets.token_bytes(16)     # authenticates its UDP joystick packets
        self.video_key = secrets.token_hex(12)     # its password for the video relay (MediaMTX)
        self.last_seq = None
        self.rtt_ms = None
        self.offsets = collections.deque(maxlen=15)
        self.pings = {}                     # id -> sent time
        self.last_pong = time.time()
        self.last_rx = self.last_pong       # any message from it (step 13q1: the HELM station's age for the AURIX)
        self.tokens = 0.0
        self.refill = time.time()
        self.rtokens = 60.0                 # read-only requests (parameter / program reads): their own allowance
        self.rrefill = time.time()
        self.tmode_pref = "auto"
        self.tmode = "full"
        self.tmode_why = ""
        self.next_lite = 0.0
        self.bytes_out = 0
        self.cmds = 0
        self.refused = 0
        self.joined = time.time()
        self.closed = False
        self.driver_open = False            # this station shows the DRIVER screen (heartbeat, section F1)
        self.driver_seen = 0.0
        self.single_op = False              # SINGLE-OPERATOR mode (off again when it disconnects)
        self.licence = False                # the user holds a manual licence (users.py)
        self.crew = None                    # "captain" / "copilot" (None: VIEWER)
        self.swap_from = None               # a role swap asked of this station by station id

    def send(self, obj):
        if self.closed:
            return
        line = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        self.bytes_out += len(line)
        try:
            self.writer.write(line)
        except Exception:
            self.closed = True

    def backlog(self):
        try:
            return self.writer.transport.get_write_buffer_size()
        except Exception:
            return 0

    def offset(self):
        """station clock - gateway clock (s); median of the recent samples."""
        return statistics.median(self.offsets) if self.offsets else 0.0

    def info(self):
        return {"id": self.id, "user": self.user, "name": self.name, "role": self.role, "local": self.local,
                "ip": self.ip, "rtt_ms": None if self.rtt_ms is None else round(self.rtt_ms),
                "tmode": self.tmode, "connected_s": int(time.time() - self.joined), "cmds": self.cmds,
                "refused": self.refused, "kb_out": round(self.bytes_out / 1024),
                "driver": self.driver_open, "single_op": self.single_op, "licence": self.licence,
                "crew": self.crew}


# the commands the gateway checks itself before forwarding (Gateway.check_prog: shallow water 114, geofence 120,
# route not checked 121); step 13q2d: SET_MODE (HDG LOCK = the UNCREWED entry from REMOTE / MANEUVER)
PROG_CHECK_TYPES = (proto.T_PROG_BEGIN, proto.T_PROG_WP, proto.T_PROG_COMMIT, proto.T_AUTO_START, proto.T_AUTO_RESUME,
                    proto.T_AUTO_RETURN, proto.T_DP, proto.T_SET_MODE)


def _uav_flying_captain(srv, st):
    """Step U5: the UAV slot the captain's joystick flies now (None = none); safe on a test's fake server."""
    pil = getattr(getattr(getattr(srv, "gw", None), "uav", None), "pilots", None)
    return pil.joystick_on_uav(st) if pil is not None and st is not None else None


class StationServer:
    def __init__(self, gw, cfg):
        self.gw = gw
        self.cfg = cfg
        self.local_nets = [ipaddress.ip_network(n) for n in cfg.get("local_nets", ["10.42.0.0/24"])]
        self.max_age = cfg.get("max_age_s", 2.0)
        self.rate = cfg.get("rate_per_s", 10.0)
        self.burst = cfg.get("rate_burst", 20.0)
        self.lite_rtt = cfg.get("lite_rtt_ms", 400.0)
        self.lite_hz = cfg.get("lite_hz", 2.0)
        self.lost_s = cfg.get("lost_s", 5.0)
        self.stations = {}
        self.pending = {}                   # gateway seq -> (station id, station seq, type, time)
        self.gw_seq = 0
        self.ping_id = 0
        self.ext = {}                       # message type -> handler(station, msg) (later sections)
        self.helm_seen = None
        # the gateway's OWN HELM grant (W55 / docs/uav_design.md 5.6): the holder of the last T_HELM the AURIX
        # accepted, else TELEM2's holder once it has stayed for HELM_STABLE_S. A command the GATEWAY itself sends on
        # the HELM station's behalf (the UAV hatch relay) goes only while TELEM2 agrees with it.
        self.helm_grant = None
        self.helm_t2, self.helm_t2_since, self.helm_grant_t = None, 0.0, 0.0
        self.helm_reqs = {}                 # gateway seq -> T_HELM payload (to learn the grant from its ACK)
        self.server = None
        self.ext["rjoy"] = self._rjoy
        self.ext["driver"] = self._driver_beat
        self.ext["driver_answer"] = self._driver_answer
        self.ext["single_op"] = self._single_op
        self.ext["crew_swap"] = self._crew_swap
        self.ext["crew_swap_answer"] = self._crew_swap_answer
        self.asks = {}                      # id -> waiting captain command (driver ACCEPT, section F1)
        self.prog_check = None              # (station, type, payload) -> refusal text (section C)
        self.ask_id = 0
        self.driver_timeout = cfg.get("driver_timeout_s", 10.0)
        self.driver_beat_s = cfg.get("driver_beat_s", 2.5)
        st = getattr(gw, "storage", None)
        self.relay_tables = RelayTables(os.path.join(st.folder("state"), "relay_tables.json")
                                        if st is not None and st.usable else None)
        a = gw.aurix
        a.on(proto.T_GW_ACK, self._on_gw_ack)
        a.on(proto.T_TELEMETRY, self._on_telem)
        a.on(proto.T_TELEM2, self._on_t2)
        for t in (proto.T_PARAM_VALUE, proto.T_PROG_WP_VALUE, proto.T_PROG_ERROR, proto.T_JOYMAP_VALUE,
                  proto.T_JOY_RAW):
            a.on(t, self._on_reply)

    # --- server ----------------------------------------------------------------------
    async def start(self):
        self.server = await asyncio.start_server(self._client, self.cfg.get("listen", "0.0.0.0"),
                                                 self.cfg.get("port", 5761), limit=65536)
        # the remote joystick also on UDP (same port number): a lost packet is
        # just gone, the next one comes 33 ms later - TCP would stall everything
        # behind a lost packet for 200-600 ms (retransmission)
        loop = asyncio.get_running_loop()
        self.udp, _ = await loop.create_datagram_endpoint(
            lambda: _Udp(self), local_addr=(self.cfg.get("listen", "0.0.0.0"), self.cfg.get("port", 5761)))
        log.info("stations: listening on %s:%d", self.cfg.get("listen", "0.0.0.0"), self.cfg.get("port", 5761))

    async def stop(self):
        if self.server:
            self.server.close()
        if getattr(self, "udp", None):
            self.udp.close()
        for st in list(self.stations.values()):
            st.writer.close()

    def is_local(self, ip):
        try:
            a = ipaddress.ip_address(ip)
        except ValueError:
            return False
        return any(a in n for n in self.local_nets)

    def _new_id(self):
        for i in range(2, 255):
            if i not in self.stations:
                return i
        return None

    async def _client(self, reader, writer):
        peer = writer.get_extra_info("peername") or ("?", 0)
        ip = peer[0]
        st = None
        try:
            line = await asyncio.wait_for(reader.readline(), 5.0)
            m = json.loads(line or b"{}")
            if m.get("t") != "login":
                raise ValueError("login first")
            role, why = self.gw.users.login(ip, str(m.get("user", "")), m.get("password"), m.get("key"))
            if role is None:
                self.gw.audit("login_failed", user=m.get("user"), ip=ip, why=why)
                writer.write((json.dumps({"t": "error", "text": why}) + "\n").encode())
                await writer.drain()
                return
            boat = self.gw.cfg.get("boat")
            if m.get("boat") is not None and str(m.get("boat")) != str(boat):
                why = "wrong boat: this gateway is %s" % boat
                self.gw.audit("login_failed", user=m.get("user"), ip=ip, why=why)
                writer.write((json.dumps({"t": "error", "text": why, "boat": boat}) + "\n").encode())
                await writer.drain()
                return
            sid = self._new_id()
            if sid is None:
                raise ValueError("too many stations")
            st = Station(sid, m["user"], role, self.is_local(ip), ip, m.get("station"), writer)
            st.licence = self.gw.users.licence(m["user"])
            note = self._crew_login(st, m.get("crew"))
            if "ts" in m:
                st.offsets.append(float(m["ts"]) - time.time())
            self.stations[sid] = st
            st.tokens = self.burst
            st.send({"t": "welcome", "id": sid, "role": role, "local": st.local, "sid": st.session,
                     "max_age_s": self.max_age, "boat": boat, "udp_key": st.udp_key.hex(),
                     "crew": st.crew, "licence": st.licence, "crew_note": note})
            self.gw.aurix.send(proto.T_GW_STATION, struct.pack("<BBB", sid, 0, 1 if st.local else 0))
            self.gw.audit("connect", station=sid, user=st.user, name=st.name, role=role, local=st.local, ip=ip,
                          crew=st.crew, licence=st.licence)
            log.info("station %d (%s, %s, %s) connected from %s", sid, st.user, role,
                     "local" if st.local else "remote", ip)
            while True:
                line = await reader.readline()
                if not line:
                    break
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                self._handle(st, m)
        except (asyncio.TimeoutError, ValueError, ConnectionError, OSError) as e:
            if st is None:
                try:
                    writer.write((json.dumps({"t": "error", "text": str(e)}) + "\n").encode())
                except Exception:
                    pass
        finally:
            if st is not None:
                self._drop(st, "disconnected")
            try:
                writer.close()
            except Exception:
                pass

    def _drop(self, st, why):
        if st.closed and st.id not in self.stations:
            return
        st.closed = True
        if self.stations.get(st.id) is st:
            del self.stations[st.id]
            # the AURIX: HELM back to nobody if it held it (REMOTE LINK LOST if remote)
            self.gw.aurix.send(proto.T_GW_STATION, struct.pack("<BBB", st.id, 1, 1 if st.local else 0))
            self.gw.audit("disconnect", station=st.id, user=st.user, why=why)
            if st.single_op:
                self.gw.audit("single_operator", station=st.id, user=st.user, on=False, why="disconnected")
            self._check_asks()
            cams = getattr(self.gw, "cams", None)
            if cams is not None and hasattr(cams, "station_cams"):
                cams.station_cams.pop(st.id, None)      # its streams + camera view end with it (13n2)
            pil = getattr(getattr(self.gw, "uav", None), "pilots", None)
            if pil is not None:
                pil.station_gone(st)                    # its UAV hovers, the lock ends (step U5)
            hail = getattr(self.gw, "hail", None)
            if hail is not None and hasattr(hail, "station_gone"):
                hail.station_gone(st)                   # its loudhailer floor ends with it (13r)
            log.info("station %d (%s) %s", st.id, st.user, why)
            try:
                st.writer.close()
            except Exception:
                pass

    # --- messages ----------------------------------------------------------------------
    def _handle(self, st, m):
        st.last_rx = time.time()
        t = m.get("t")
        if t == "gpong":
            sent = st.pings.pop(m.get("id"), None)
            if sent is not None:
                now = time.time()
                rtt = (now - sent) * 1000.0
                st.rtt_ms = rtt if st.rtt_ms is None else 0.7 * st.rtt_ms + 0.3 * rtt
                st.last_pong = now
                if "ts" in m:
                    st.offsets.append(float(m["ts"]) - (sent + now) / 2.0)
        elif t == "cmd":
            self.command(st, m)
        elif t == "tmode" and m.get("mode") in ("auto", "full", "lite"):
            st.tmode_pref = m["mode"]
        elif t in self.ext:
            try:
                self.ext[t](st, m)
            except Exception:
                log.exception("station message %s", t)

    def _rjoy(self, st, m):
        """Remote joystick (section D), 30 Hz from the HELM station: forwarded with
        the data age and one-way latency measured here; the AURIX decides
        (older than 300 ms / latency over 400 ms = link lost)."""
        if st.role == "VIEWER" or self.gw.telem2.get("helm") != st.id:
            st.rjoy_dropped = getattr(st, "rjoy_dropped", 0) + 1
            return
        pil = getattr(getattr(self.gw, "uav", None), "pilots", None)
        if pil is not None and pil.slot_of(st.id) is not None:
            # step U5: this station's joystick flies a UAV - nothing of it reaches the boat (storm I83 reserved)
            st.rjoy_uav = getattr(st, "rjoy_uav", 0) + 1
            return
        try:
            seq = int(m["seq"]) & 0xFFFF
            age = self.age(st, float(m["ts"]))
            x, y = float(m.get("x", 0.0)), float(m.get("y", 0.0))
            lever = m.get("lever")
            if lever is not None:
                lever = float(lever)
        except (KeyError, ValueError, TypeError):
            return
        # step 13q5b (review LOW-3): a NaN / infinite number would pass min / max as FULL deflection (forward gear /
        # full lever) - such a packet is invalid: dropped and counted (no packets = the AURIX's 0.3 s rule: neutral)
        if not all(math.isfinite(v) for v in (x, y, age) + (() if lever is None else (lever,))):
            st.rjoy_bad = getattr(st, "rjoy_bad", 0) + 1
            return
        last = getattr(st, "rjoy_seq", None)
        if last is not None and ((seq - last) & 0xFFFF) >= 0x8000 or seq == last:
            return                                          # old / repeated packet
        st.rjoy_seq = seq
        age_ms = int(min(65535, max(0, age * 1000)))
        lat_ms = int(min(65535, (st.rtt_ms or 0) / 2))
        # b2: the captain's CONFIRM button (counts on the AURIX only in UNCREWED); b1 (the old "release") is never
        # set - step 13q2 (owner 2026-10-31): there is no release, an old station's "release" is dropped here
        # b3: the TRIGGER (step 13q3: in UNCREWED HDG LOCK while steering by hand = the current heading is locked)
        # step 13p (owner 2026-10-31 D, "one press never does two things"): a trigger press made while this station
        # has a question open (the captain's ACCEPT of a copilot action, an avoidance / vision proposal, the slamming
        # restore) answers it on the station - it is never also forwarded, until it is released
        trig = bool(m.get("trigger"))
        if trig and (getattr(st, "trig_block", False) or self.question_open(st)):
            st.trig_block = True
            st.rjoy_trig_masked = getattr(st, "rjoy_trig_masked", 0) + 1
            trig = False
        elif not trig:
            st.trig_block = False
        flags = (1 if lever is not None else 0) | (4 if m.get("confirm") else 0) | (8 if trig else 0)
        lv = int(min(1000, max(0, (lever if lever is not None else 0.5) * 1000)))
        self.gw.aurix.send(proto.T_GW_RJOY, struct.pack(
            "<BHHHhhHBB", st.id, seq, age_ms, lat_ms, int(max(-1000, min(1000, x * 1000))),
            int(max(-1000, min(1000, y * 1000))), lv, 0, flags))
        st.rjoy_fwd = getattr(st, "rjoy_fwd", 0) + 1

    def question_open(self, st):
        """A YES / NO question is open on this station's screen 1 (step 13p): a copilot action waiting for its ACCEPT,
        an avoidance proposal, a vision slow-down note (the captain / single operator), the slamming restore."""
        if any(a.get("target") == st.id for a in self.asks.values()):
            return True
        if st.crew == "captain" or st.single_op:
            if getattr(getattr(self.gw, "avoider", None), "proposal", None):
                return True
            vp = getattr(getattr(self.gw, "vision", None), "proposal", None)
            if vp and not vp.get("acked"):
                return True
        return bool((self.gw.telem2 or {}).get("slam_ask"))

    def udp_packet(self, data):
        """body|mac: body = JSON {"sid", "t":"rjoy", ...}, mac = HMAC-SHA256(udp_key, body)[:16] hex."""
        try:
            body, _, mac = data.rpartition(b"|")
            m = json.loads(body)
        except ValueError:
            return
        st = next((s for s in self.stations.values() if s.session == m.get("sid")), None)
        if st is None:
            return
        want = hmac.new(st.udp_key, body, hashlib.sha256).hexdigest()[:32].encode()
        if not hmac.compare_digest(want, mac):
            st.udp_bad = getattr(st, "udp_bad", 0) + 1
            return
        st.udp_ok = getattr(st, "udp_ok", 0) + 1
        if m.get("t") == "rjoy":
            self._rjoy(st, m)
        elif m.get("t") == "uav_ctl" and "uav_ctl" in self.ext:
            try:
                self.ext["uav_ctl"](st, m)          # step U5: manual UAV flight, 10 Hz, signed like rjoy
            except Exception:
                log.exception("uav_ctl")

    def age(self, st, ts):
        """How old a station time stamp is on the gateway's clock (s)."""
        return time.time() - (ts - st.offset())

    def refuse(self, st, seq, ftype, code, extra=""):
        st.refused += 1
        text = GW_RESULTS[code] + extra
        st.send({"t": "ack", "seq": seq, "type": ftype, "result": code, "text": text})
        self.gw.audit("cmd", station=st.id, user=st.user, type=ftype, seq=seq, result=code, text=text)

    def command(self, st, m):
        try:
            seq, ts, ftype = int(m["seq"]), float(m["ts"]), int(m["type"])
            payload = bytes.fromhex(m.get("p", ""))
        except (KeyError, ValueError, TypeError):
            return self.refuse(st, m.get("seq"), m.get("type"), 108)
        if m.get("sid") != st.session:
            return self.refuse(st, seq, ftype, 107)
        if st.role == "VIEWER" and ftype not in READ_ONLY:
            return self.refuse(st, seq, ftype, 100)
        if st.last_seq is not None and seq == st.last_seq:
            return self.refuse(st, seq, ftype, 101)
        if st.last_seq is not None and seq < st.last_seq:
            return self.refuse(st, seq, ftype, 102)
        age = self.age(st, ts)
        if age > self.max_age:
            st.last_seq = seq
            return self.refuse(st, seq, ftype, 103, " (%.1f s old, limit %.1f s)" % (age, self.max_age))
        if age < -1.0:
            return self.refuse(st, seq, ftype, 104)
        now = time.time()
        if ftype in READ_ONLY:                      # reads change nothing: 30 / s, bursts of 60
            st.rtokens = min(60.0, st.rtokens + (now - st.rrefill) * 30.0)
            st.rrefill = now
            if st.rtokens < 1.0:
                return self.refuse(st, seq, ftype, 105)
            st.rtokens -= 1.0
        else:
            st.tokens = min(self.burst, st.tokens + (now - st.refill) * self.rate)
            st.refill = now
            if st.tokens < 1.0:
                return self.refuse(st, seq, ftype, 105)
            st.tokens -= 1.0
        st.last_seq = seq
        if not self.gw.aurix.connected or len(payload) > 194:
            return self.refuse(st, seq, ftype, 106 if not self.gw.aurix.connected else 108)
        manual_ask = ftype in (proto.T_REMOTE, proto.T_MANEUVER) and payload[:1] == bytes([0])
        if manual_ask and not st.licence:
            return self.refuse(st, seq, ftype, 116)          # manual driving: a licensed user only
        if manual_ask and st.crew != "captain" and not st.single_op:
            return self.refuse(st, seq, ftype, 118)          # ... and only the captain
        if ftype == proto.T_RESTART and payload[:1] == bytes([proto.ENG_CRANK]) and st.crew != "captain" \
                and not st.single_op:
            return self.refuse(st, seq, ftype, 118)          # hold-to-crank (engine START): the captain only (section N)
        if ftype == proto.T_LINK_TIME and st.crew != "captain" and not st.single_op:
            return self.refuse(st, seq, ftype, 118)          # LINK TIME: the HELM captain only (step 13q1)
        if ftype == proto.T_LINK_TIME:
            self.gw.audit("link_time", station=st.id, user=st.user, seconds=payload[0] if payload else None)
        if ftype == proto.T_SLAM_RESTORE and st.crew != "captain" and not st.single_op:
            # the slamming power restore (step 13q4, owner 2026-10-31): the captain's ACCEPT on screen 1 only - a
            # copilot can't restore power, not even through the captain's ACCEPT (never forwarded)
            return self.refuse(st, seq, ftype, 118)
        if ftype == proto.T_SLAM_RESTORE:
            self.gw.audit("slam_restore", station=st.id, user=st.user,
                          answer="accept" if payload[:1] == bytes([proto.SLAM_ACCEPT]) else "reject")
        if ftype == proto.T_HELM and payload[:1] == bytes([0]) and st.crew != "captain" and not st.local \
                and not st.single_op and not (st.licence and not self.gw.telem2.get("helm")):
            # a copilot never takes HELM from the captain (that is the role swap); a licensed user may take a
            # FREE HELM and so becomes the captain
            return self.refuse(st, seq, ftype, 117)
        hatch = getattr(getattr(self.gw, "uav", None), "hatch_relay", None)
        if ftype == proto.T_RELAY and hatch and payload[:1] == bytes([hatch]):
            # the UAV hatch relay (CLAUDE.md V): switched ONLY by the gateway's launch sequence / the captain's
            # HATCH CLOSE (uav_hatch_close) - never a station's relay command
            return self.refuse(st, seq, ftype, 125, ": the UAV hatch relay is switched by the launch sequence only")
        if ftype == proto.T_RELAY_CFG and hatch:
            # U3 review L5: a table that would switch the hatch relay ON at the AURIX's power-up (= open the hatch by
            # itself) or makes it anything but an extra on/off relay without a pair is refused
            import relaytab
            d = relaytab.details_of(bytes(payload)).get(hatch)
            if d is not None and (d["power_up"] != "off" or d["group"] != "extra" or d["type"] != "onoff" or d["pair"]):
                return self.refuse(st, seq, ftype, 125, ": the UAV hatch relay must be an extra on/off relay, "
                                                        "power_up off, no pair")
        why126 = StationServer.uav_joy_refusal(self, st, ftype, payload)
        if why126:
            return self.refuse(st, seq, ftype, 126, why126)
        if self.prog_check and ftype in PROG_CHECK_TYPES:
            why = self.prog_check(st, ftype, payload)
            if why:
                code, why = why if isinstance(why, tuple) else (114, why)
                return self.refuse(st, seq, ftype, code, ": " + why)
        what = str(m.get("what") or proto.TYPE_NAMES.get(ftype, "command %d" % ftype))[:80]
        if ftype == proto.T_RELAY_CFG:
            self.relay_tables.learn(payload)                 # which relay is in which group (CLAUDE.md O)
        driving = proto.is_driving_cmd(ftype, payload, self.relay_groups())
        if st.single_op:
            if driving and m.get("confirm") != "self":
                return self.refuse(st, seq, ftype, 113)
            if driving:
                self.gw.audit("single_op_cmd", station=st.id, user=st.user, type=ftype, what=what)
            return self._forward(st, seq, ftype, payload, age)
        helm = self.helm_forward()
        if st.crew == "copilot" and helm != st.id and ftype not in READ_ONLY and ftype != proto.T_HELM:
            # the copilot: on the captain's behalf; driving actions after the captain's ACCEPT
            cap = self.stations.get(helm)
            if cap is None:
                return self.refuse(st, seq, ftype, 119)
            if driving and not self.copilot_free(ftype):
                return self._ask_driver(st, seq, ftype, payload, age, what, target=cap)
            self.gw.audit("copilot_cmd", station=st.id, user=st.user, type=ftype, what=what, captain=cap.id)
            return self._forward(st, seq, ftype, payload, age, env=cap)
        self._forward(st, seq, ftype, payload, age)

    # --- step U5: the captain's joystick on a UAV (CLAUDE.md V, docs/uav_design.md 6.3) ------------------------------
    def uav_pilots(self):
        return getattr(getattr(self.gw, "uav", None), "pilots", None)

    def uav_flying_captain(self, st):
        """The UAV slot the captain's (HELM / single operator) joystick flies now, else None."""
        pil = getattr(getattr(self.gw, "uav", None), "pilots", None)
        return pil.joystick_on_uav(st) if pil is not None and st is not None else None

    def uav_joy_refusal(self, st, ftype, payload):
        """126 (text) while the CAPTAIN's joystick is on a UAV: his own driving commands AND every copilot command
        that would need his ACCEPT are refused at once (never queued: he is flying, not watching the boat); STANDBY,
        AUTO PAUSE and DP STOP stay free (MED-3 of the U1 review). None = not refused."""
        if getattr(getattr(self.gw, "uav", None), "pilots", None) is None or ftype == proto.T_AUTO_PAUSE:
            return None                     # (no UAV magazine / a test's fake server: nothing to refuse)
        manual = ftype in (proto.T_REMOTE, proto.T_MANEUVER) and payload[:1] == bytes([0])   # (a REMOTE / MANEUVER ask)
        if not (manual or proto.is_driving_cmd(ftype, payload, StationServer.relay_groups(self))):
            return None
        n = _uav_flying_captain(self, st)
        if n is not None:
            return ": the captain's joystick flies UAV %d (STANDBY / AUTO PAUSE / DP STOP only)" % n
        if st.crew == "copilot" and not st.single_op:
            cap = self.stations.get(self.gw.telem2.get("helm"))
            n = _uav_flying_captain(self, cap) if cap is not None and cap is not st else None
            if n is not None:
                return ": the captain flies UAV %d - no ACCEPT can be asked now" % n
        return None

    def copilot_free(self, ftype):
        """A copilot command forwarded without the captain's ACCEPT (COPILOT_FREE). Step 13q2b (MED-1): AUTO RETURN
        only while the captain is NOT driving by hand - in REMOTE / MANEUVER (or with no telemetry yet) it is a mode
        change out of the captain's hands = the captain's ACCEPT (the AURIX refuses it there anyway: WRONG_MODE)."""
        if ftype not in COPILOT_FREE:
            return False
        if ftype == proto.T_AUTO_RETURN:
            # (13q2c, review LOW-1: an OLD copy of the telemetry - older than 3 s or the AURIX link down - is an
            # unknown mode = the captain's ACCEPT)
            t = self.gw.telem_fresh() if hasattr(self.gw, "telem_fresh") else None
            mode = (t or {}).get("mode")
            return mode is not None and mode not in (proto.MODE_REMOTE, proto.MODE_MANEUVER)
        return True

    # --- the gateway's own commands on the HELM station's behalf (the UAV hatch, CLAUDE.md V / U3) ------------------
    HELM_STABLE_S = 2.0

    def helm_forward(self):
        """The HELM holder a copilot's command is forwarded for (owner W55, step WA): the gateway's OWN grant while it
        is newer than TELEM2's last HELM change (an accepted T_HELM take / hand-over the AURIX already ACKed, TELEM2
        not yet showing it: up to one TELEM2 period - the old "B3 race", where a just-demoted captain's command went
        under his own id and the AURIX refused it NOT HELM); else TELEM2's holder. The AURIX checks HELM again."""
        h = (self.gw.telem2 or {}).get("helm")
        if self.helm_grant_t and self.helm_grant_t > self.helm_t2_since and self.helm_grant != h:
            return self.helm_grant
        return h

    def helm_station(self):
        """(station, why): the station holding HELM - only when the gateway's own grant and TELEM2 agree (W55)."""
        h = (self.gw.telem2 or {}).get("helm")
        if not h:
            return None, "no station holds HELM"
        st = self.stations.get(h)
        if st is None or st.role == "VIEWER":
            return None, "HELM is not held by a gateway station (%s)" % self.helm_name(h)
        if self.helm_grant != h:
            return None, "HELM is changing (TELEM2 %s, the gateway's grant %s)" % (h, self.helm_grant)
        return st, ""

    def uav_relay(self, n, on, done):
        """The UAV hatch relay n ON (open) / OFF (close) as a T_RELAY of the HELM station (the AURIX checks HELM,
        the table, FIRST TEST as always). done(result, text): result None = not sent (the reason)."""
        st, why = self.helm_station()
        if st is None:
            return done(None, why)
        aur = getattr(self.gw, "aurix", None)
        if aur is not None and not getattr(aur, "connected", True):     # U3 review M5
            return done(None, "no link to the AURIX")
        self.gw.audit("uav_hatch_relay", relay=n, on=bool(on), helm=st.id)
        self._forward(st, None, proto.T_RELAY, proto.p_relay(n, proto.RELAY_CMD_ON if on else proto.RELAY_CMD_OFF),
                      0.0, env=st, done=lambda r: done(r, "no answer from the AURIX" if r is None else
                                                      proto.RESULTS.get(r, "error %d" % r)))
        if self.gw_seq in self.pending:                 # timed: tick() answers it after UAV_RELAY_S
            self.pending[self.gw_seq] = self.pending[self.gw_seq] + ("uav",)

    def hatch_relay_ok(self, n):
        """(ok, why): relay n is in the table ACTIVE on the AURIX as an extra onoff relay, power_up off, no pair."""
        d = self.relay_tables.detail(self.gw.telem2, n) if n else None
        if not n:
            return False, "no hatch relay configured (gateway.json uav hatch_relay)"
        if d is None:
            return False, "the hatch relay %d is not in the active relay table (or the gateway does not know it)" % n
        if d["group"] != "extra" or d["type"] != "onoff" or d["pair"] or d["power_up"] != "off":
            return False, "relay %d is not an extra on/off relay with power_up off and no pair" % n
        return True, ""

    def ask_captain(self, st, m, what, action):
        """A YES / NO for the CAPTAIN's screen 1 (the UAV launch approval, CLAUDE.md V): always asked there - also
        when the captain prepared it himself; a single operator answers his own (m["confirm"] == "self")."""
        if st.role == "VIEWER":
            return action(False, 100)
        if st.single_op:
            if m.get("confirm") != "self":
                return action(False, 113)
            self.gw.audit("single_op_cmd", station=st.id, user=st.user, what=what)
            return action(True, 0)
        cap = self.captain()
        if cap is None:
            return action(False, 119)
        self._ask_driver(st, None, None, b"", 0.0, what, action=action, target=cap)

    def relay_groups(self):
        """{n: group} of the relay table active on the AURIX, None if the gateway doesn't know it."""
        return self.relay_tables.active(self.gw.telem2)

    def relay_status(self):
        """gw_status "relays": does the gateway know the active table's groups (hash = TELEM2 group M)."""
        t2 = self.gw.telem2 or {}
        return {"hash": t2.get("relay_hash"), "known": self.relay_groups() is not None}

    # --- the captain's ACCEPT (screen 1) for copilot actions (section D, was F1) ------
    def drivers(self):
        now = time.time()
        return [s for s in self.stations.values() if s.driver_open and now - s.driver_seen <= self.driver_beat_s]

    def captain(self):
        return next((s for s in self.stations.values() if s.crew == "captain"), None)

    def _screens(self, target):
        """The screen 1 that answers: the target station's (the captain), if it is open."""
        return [s for s in self.drivers() if target is not None and s is target]

    def _ask_driver(self, st, seq, ftype, payload, age, what, action=None, target=None):
        target = target if target is not None else self.captain()
        if _uav_flying_captain(self, target) is not None:
            # step U5: the captain flies a UAV - no question is queued for his ACCEPT (126 at once)
            if action:
                return action(False, 126)
            return self.refuse(st, seq, ftype, 126, ": the captain flies a UAV - no ACCEPT can be asked now")
        drv = self._screens(target)
        if not drv:
            if action:
                return action(False, 109)
            return self.refuse(st, seq, ftype, 109)
        self.ask_id += 1
        a = {"id": self.ask_id, "st": st.id, "seq": seq, "type": ftype, "payload": payload, "age": age,
             "what": what, "deadline": time.time() + self.driver_timeout, "action": action, "target": target.id}
        self.asks[a["id"]] = a
        self.gw.audit("driver_ask", station=st.id, user=st.user, what=what, id=a["id"], captain=target.id)
        for d in drv:
            d.send({"t": "driver_ask", "id": a["id"], "text": what, "from": st.name, "timeout_s": self.driver_timeout})

    def ask_local(self, st, m, what, action):
        """A GATEWAY setting that affects the driving (e.g. vision STAGE 3 on): the same rule as the
        captain's driving commands - the driver screen's ACCEPT, or single-operator with its own
        confirmation (m["confirm"] == "self"). action(ok, code) runs with the answer
        (codes as for commands: 109 no driver screen, 110 rejected, 111 timeout, 112 lost, 113)."""
        if st.role == "VIEWER":
            return action(False, 21)
        if st.single_op:
            if m.get("confirm") != "self":
                return action(False, 113)
            self.gw.audit("single_op_cmd", station=st.id, user=st.user, what=what)
            return action(True, 0)
        if st.crew == "captain":                            # the captain's own decision
            self.gw.audit("captain_cmd", station=st.id, user=st.user, what=what)
            return action(True, 0)
        self._ask_driver(st, None, None, b"", 0.0, what, action=action)

    def _driver_beat(self, st, m):
        was = st.driver_open
        st.driver_open = bool(m.get("open")) and st.role != "VIEWER"
        st.driver_seen = time.time()
        if st.driver_open != was:
            self.gw.audit("driver_screen", station=st.id, user=st.user, open=st.driver_open)
            self._check_asks()

    def _driver_answer(self, st, m):
        a = self.asks.get(m.get("id"))
        if a is None or st not in self.drivers() or st.id != a.get("target"):
            return                                          # only the captain's screen 1 answers
        del self.asks[a["id"]]
        cap = self.stations.get(a["st"])
        for d in self.drivers():
            d.send({"t": "driver_ask_done", "id": a["id"]})
        self.gw.audit("driver_answer", station=st.id, user=st.user, id=a["id"], yes=bool(m.get("yes")),
                      what=a["what"])
        if a.get("action"):
            return a["action"](bool(m.get("yes")), 0 if m.get("yes") else 110)
        if cap is None:
            return
        if m.get("yes"):
            self._forward(cap, a["seq"], a["type"], a["payload"], a["age"], env=st)
        else:
            self.refuse(cap, a["seq"], a["type"], 110)

    def _check_asks(self):
        """Waiting copilot actions: timed out, or the captain's screen 1 is gone -> rejected."""
        now = time.time()
        for a in list(self.asks.values()):
            have = bool(self._screens(self.stations.get(a.get("target"))))
            code = 112 if not have else (111 if now > a["deadline"] else None)
            if code is None:
                continue
            del self.asks[a["id"]]
            for d in self.drivers():
                d.send({"t": "driver_ask_done", "id": a["id"]})
            if a.get("action"):
                a["action"](False, code)
                continue
            cap = self.stations.get(a["st"])
            if cap:
                self.refuse(cap, a["seq"], a["type"], code)

    def refuse_asks_for(self, target, code=126):
        """U5 review MED-4: the captain's joystick went onto a UAV - every question already open for HIS ACCEPT is
        refused now (code 126), never answered later by a captain who flies a drone. -> how many."""
        n = 0
        for a in list(self.asks.values()):
            if target is None or a.get("target") != target.id:
                continue
            del self.asks[a["id"]]
            n += 1
            for d in self.drivers():
                d.send({"t": "driver_ask_done", "id": a["id"]})
            self.gw.audit("driver_ask_refused", id=a["id"], what=a["what"], code=code)
            if a.get("action"):
                a["action"](False, code)
                continue
            cap = self.stations.get(a["st"])
            if cap:
                self.refuse(cap, a["seq"], a["type"], code, ": the captain flies a UAV")
        return n

    def _single_op(self, st, m):
        on = bool(m.get("on")) and st.role != "VIEWER"
        if on != st.single_op:
            st.single_op = on
            self.gw.audit("single_operator", station=st.id, user=st.user, on=on)
            log.warning("station %d (%s): SINGLE-OPERATOR mode %s", st.id, st.user, "ON" if on else "off")
        st.send({"t": "single_op_ack", "on": st.single_op})

    def _forward(self, st, seq, ftype, payload, age, env=None, done=None):
        """To the AURIX in a GW_CMD envelope of station env (default: st itself; a copilot's
        command goes in the captain's); the answer goes back to st (or to done(result))."""
        env = env or st
        now = time.time()
        self.gw_seq = (self.gw_seq + 1) & 0xFFFF
        self.pending[self.gw_seq] = (st.id, seq, ftype, now, age, env.id, done)
        if ftype == proto.T_HELM:
            self.helm_reqs[self.gw_seq] = (env.id, bytes(payload))
            while len(self.helm_reqs) > 64:
                self.helm_reqs.pop(next(iter(self.helm_reqs)))
        st.cmds += 1
        env = struct.pack("<BBHB", env.id, 1 if env.local else 0, self.gw_seq, ftype) + payload
        self.gw.aurix.send(proto.T_GW_CMD, env)
        # forget answers that never came
        if len(self.pending) > 512:
            for k in sorted(self.pending, key=lambda k: self.pending[k][3])[:256]:
                del self.pending[k]

    # --- crew: captain / copilot (section D) -----------------------------------------
    def _crew_login(self, st, want):
        """The role at login -> a note for the welcome (None if it got what it asked)."""
        if st.role == "VIEWER":
            st.crew = None
            return None
        cap = self.captain()
        if want == "copilot":
            st.crew = "copilot"
            return None
        if cap is None and st.licence:
            st.crew = "captain"
            return None
        st.crew = "copilot"
        if want == "captain":
            return "captain refused: no manual licence" if not st.licence else \
                "captain refused: %s is the captain" % cap.name
        return None

    def _set_captain(self, st, why):
        old = self.captain()
        if old is st:
            return
        if old is not None:
            old.crew = "copilot"
        st.crew = "captain"
        self.gw.audit("crew", captain=st.id, user=st.user, before=old.id if old else None, why=why)
        for s in list(self.stations.values()):
            s.send({"t": "crew", "captain": st.id, "why": why})

    def _crew_swap(self, st, m):
        """Captain <-> copilot, asked by either side; the other one must accept."""
        cap = self.captain()
        if st.role == "VIEWER" or cap is None:
            return st.send({"t": "crew_swap_done", "ok": False, "text": "no captain to swap with"})
        other = cap if st is not cap else self.stations.get(m.get("with"))
        if other is None or other is st or other.role == "VIEWER":
            return st.send({"t": "crew_swap_done", "ok": False, "text": "no such station"})
        if not (st.licence and other.licence):
            self.gw.audit("crew_swap", station=st.id, other=other.id, ok=False, why="licence")
            return st.send({"t": "crew_swap_done", "ok": False,
                            "text": "role swap refused: both users need a manual licence"})
        other.swap_from = st.id
        other.send({"t": "crew_swap_ask", "from": st.id, "name": st.name})
        self.gw.audit("crew_swap", station=st.id, other=other.id, asked=True)

    def _crew_swap_answer(self, st, m):
        asker = self.stations.get(st.swap_from) if st.swap_from else None
        st.swap_from = None
        if asker is None:
            return
        cap = self.captain()
        new = st if asker is cap else asker
        if not m.get("yes") or cap is None or new is cap:
            asker.send({"t": "crew_swap_done", "ok": False, "text": "role swap rejected"})
            return
        helm = self.gw.telem2.get("helm")

        def done(result):
            ok = result == 0
            if ok:
                self._set_captain(new, "role swap")
            self.gw.audit("crew_swap", captain=new.id, ok=ok, result=result)
            for s in (asker, st):
                s.send({"t": "crew_swap_done", "ok": ok, "captain": new.id,
                        "text": "OK" if ok else proto.RESULTS.get(result, "error %d" % result)})
        if helm == cap.id:
            # HELM to the new captain in ONE step (no gap: no lost link when UNCREWED)
            self._forward(cap, None, proto.T_HELM, proto.p_helm_hand(new.id, new.local), 0.0, env=cap, done=done)
        else:
            done(0)

    # --- from the AURIX --------------------------------------------------------------
    def _on_gw_ack(self, f):
        if len(f.payload) < 5:
            return
        sid, gseq, ftype, result = struct.unpack_from("<BHBB", f.payload)
        p = self.pending.get(gseq)
        if p is None or p[5] != sid:
            return
        del self.pending[gseq]
        hr = self.helm_reqs.pop(gseq, None)
        if hr is not None and ftype == proto.T_HELM and result == 0:
            holder, pay = hr
            act = pay[0] if pay else 0
            self.helm_grant = holder if act == 0 else (pay[1] if act == 2 and len(pay) > 1 else None)
            self.helm_grant_t = time.time()
        if ftype == proto.T_PROG_COMMIT and hasattr(self.gw, "prog_committed"):
            self.gw.prog_committed(result)                   # the gateway's copy of the route (DP TRIP check)
        if p[6]:
            return p[6](result)
        st = self.stations.get(p[0])
        if st is None:
            return
        text = proto.RESULTS.get(result, "error %d" % result)
        st.send({"t": "ack", "seq": p[1], "type": ftype, "result": result, "text": text})
        self.gw.audit("cmd", station=st.id, user=st.user, type=ftype, seq=p[1], result=result, text=text,
                      age_ms=round(p[4] * 1000), **({"as_captain": sid} if sid != st.id else {}))

    def _on_telem(self, f):
        b = f.payload.hex()
        now = time.time()
        for st in list(self.stations.values()):
            if st.tmode == "full":
                st.send({"t": "tm", "b": b})
            elif now >= st.next_lite and self.gw.telem:
                st.next_lite = now + 1.0 / self.lite_hz
                d = {k: self.gw.telem.get(k) for k in LITE_FIELDS if k in self.gw.telem}
                st.send({"t": "lite", "d": d})

    def _on_t2(self, f):
        b = f.payload.hex()
        for st in list(self.stations.values()):
            if st.tmode == "full":
                st.send({"t": "t2", "b": b})
        helm = self.gw.telem2.get("helm")
        now = time.time()
        if helm != self.helm_t2:
            self.helm_t2, self.helm_t2_since = helm, now
        elif helm != self.helm_grant and now - max(self.helm_t2_since, self.helm_grant_t) >= self.HELM_STABLE_S:
            self.helm_grant = helm or None          # the AURIX's view, once it stayed (lost / a local take / start)
        if helm is not None and helm != self.helm_seen:
            if self.helm_seen is not None:
                self.gw.audit("helm", holder=helm, holder_name=self.helm_name(helm))
            self.helm_seen = helm
            new = self.stations.get(helm)
            if new is not None and new.crew != "captain" and new.role != "VIEWER":
                self._set_captain(new, "took HELM")         # e.g. a station on the boat took it

    def _on_reply(self, f):
        msg = {"t": "frame", "type": f.type, "b": f.payload.hex()}
        for st in list(self.stations.values()):
            st.send(msg)

    def lost_for(self, st):
        """Seconds without an answer before a station is dropped: lost_s; the HELM station is kept at least 2 s
        longer than the AURIX's LINK TIME (step 13q1: the boat decides the lost link, the captain keeps HELM
        through a short outage within it). UNCREWED only: CREWED keeps its REMOTE LINK LOST after lost_s."""
        t2 = self.gw.telem2 or {}
        if t2.get("helm") == st.id and t2.get("lt_set") and t2.get("profile") == proto.PROFILE_UNCREWED:
            return max(self.lost_s, float(t2["lt_set"]) + 2.0)
        return self.lost_s

    def helm_age(self):
        """(HELM station id, seconds since this gateway last heard it) for GW_STATUS (step 13q1); (0, 0) when the
        HELM is not a gateway station; age None = the HELM station is not connected here."""
        h = (self.gw.telem2 or {}).get("helm")
        if not h or h < 2:
            return 0, 0
        st = self.stations.get(h)
        return h, (None if st is None or st.closed else
                   max(0.0, time.time() - max(st.last_pong, getattr(st, "last_rx", st.last_pong))))

    def helm_name(self, helm):
        if helm in (None, 0):
            return "nobody"
        if helm == 1:
            return "USB laptop (local)"
        st = self.stations.get(helm)
        return "%s (%s, %s)" % (st.name, st.user, "local" if st.local else "remote") if st else "station %d" % helm

    # --- every second ------------------------------------------------------------------
    UAV_RELAY_S = 5.0

    def tick(self):
        now = time.time()
        self._check_asks()
        # U3 review M5: a hatch relay command the AURIX never answered = answered "no answer" (never a wait for ever).
        # (U4 review LOW, documented: an AURIX answer arriving AFTER this timeout finds no pending entry and is ignored;
        # the relay may still have switched - the hatch's own limit switches show the truth: Hatch.command was set
        # before the T_RELAY went out, so a hatch that moves anyway is the commanded move, not a FAULT, and one that
        # does not move is a FAULT after open_s / close_s; the launch already stopped on the "no answer")
        for k, p in list(self.pending.items()):
            if len(p) > 7 and p[7] == "uav" and now - p[3] > self.UAV_RELAY_S:
                del self.pending[k]
                try:
                    p[6](None)
                except Exception:
                    log.exception("uav relay timeout")
        saver = self.gw.data_saver() if hasattr(self.gw, "data_saver") else False
        self.ping_id += 1
        for st in list(self.stations.values()):
            lost_s = self.lost_for(st)
            if now - st.last_pong > lost_s:
                self._drop(st, "lost (no answer for %.0f s)" % lost_s)
                continue
            st.pings[self.ping_id] = now
            if len(st.pings) > 20:
                st.pings.pop(min(st.pings))
            st.send({"t": "gping", "id": self.ping_id, "gw": now})
            # adaptive telemetry (B4)
            old = st.tmode
            if st.tmode_pref in ("full", "lite"):
                st.tmode, st.tmode_why = st.tmode_pref, "chosen by the station"
            elif saver:
                st.tmode, st.tmode_why = "lite", "data saver"
            elif st.rtt_ms is not None and st.rtt_ms > self.lite_rtt:
                st.tmode, st.tmode_why = "lite", "latency %.0f ms" % st.rtt_ms
            elif st.backlog() > 65536:
                st.tmode, st.tmode_why = "lite", "slow link (backlog)"
            elif st.tmode == "lite" and st.rtt_ms is not None and st.rtt_ms > 0.6 * self.lite_rtt:
                pass                                        # hysteresis: stay lite
            else:
                st.tmode, st.tmode_why = "full", "good link"
            if st.tmode != old:
                st.send({"t": "tmode", "mode": st.tmode, "why": st.tmode_why})
                log.info("station %d telemetry %s (%s)", st.id, st.tmode, st.tmode_why)
        status = self.gw.status()
        status["t"] = "status"
        status["stations"] = [s.info() for s in self.stations.values()]
        h = self.gw.telem2.get("helm")
        status["helm"] = {"id": h, "name": self.helm_name(h)}
        for st in list(self.stations.values()):
            status["you"] = st.id
            st.send(status)


class _Udp(asyncio.DatagramProtocol):
    def __init__(self, server):
        self.server = server

    def datagram_received(self, data, addr):
        if data[:2] == b"TV":                               # loudhailer voice (step 13r, voicecodec)
            hail = getattr(self.server.gw, "hail", None)
            if hail is not None:
                hail.udp(data, addr)
            return
        self.server.udp_packet(data)
