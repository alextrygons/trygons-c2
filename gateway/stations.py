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
import secrets
import statistics
import struct
import time

from common import log, proto

GW_RESULTS = {100: "VIEWER: watch only", 101: "duplicate command", 102: "command out of order",
              103: "stale command", 104: "timestamp in the future", 105: "rate limited",
              106: "AURIX not connected", 107: "wrong session", 108: "bad command"}
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
                "refused": self.refused, "kb_out": round(self.bytes_out / 1024)}


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
        self.server = None
        self.ext["rjoy"] = self._rjoy
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
            sid = self._new_id()
            if sid is None:
                raise ValueError("too many stations")
            st = Station(sid, m["user"], role, self.is_local(ip), ip, m.get("station"), writer)
            if "ts" in m:
                st.offsets.append(float(m["ts"]) - time.time())
            self.stations[sid] = st
            st.tokens = self.burst
            st.send({"t": "welcome", "id": sid, "role": role, "local": st.local, "sid": st.session,
                     "max_age_s": self.max_age, "boat": self.gw.cfg.get("boat"), "udp_key": st.udp_key.hex()})
            self.gw.aurix.send(proto.T_GW_STATION, struct.pack("<BBB", sid, 0, 1 if st.local else 0))
            self.gw.audit("connect", station=sid, user=st.user, name=st.name, role=role, local=st.local, ip=ip)
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
            log.info("station %d (%s) %s", st.id, st.user, why)
            try:
                st.writer.close()
            except Exception:
                pass

    # --- messages ----------------------------------------------------------------------
    def _handle(self, st, m):
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
        try:
            seq = int(m["seq"]) & 0xFFFF
            age = self.age(st, float(m["ts"]))
            x, y = float(m.get("x", 0.0)), float(m.get("y", 0.0))
            lever = m.get("lever")
        except (KeyError, ValueError, TypeError):
            return
        last = getattr(st, "rjoy_seq", None)
        if last is not None and ((seq - last) & 0xFFFF) >= 0x8000 or seq == last:
            return                                          # old / repeated packet
        st.rjoy_seq = seq
        age_ms = int(min(65535, max(0, age * 1000)))
        lat_ms = int(min(65535, (st.rtt_ms or 0) / 2))
        flags = (1 if lever is not None else 0) | (2 if m.get("release") else 0)
        lv = int(min(1000, max(0, float(lever if lever is not None else 0.5) * 1000)))
        self.gw.aurix.send(proto.T_GW_RJOY, struct.pack(
            "<BHHHhhHBB", st.id, seq, age_ms, lat_ms, int(max(-1000, min(1000, x * 1000))),
            int(max(-1000, min(1000, y * 1000))), lv, 0, flags))
        st.rjoy_fwd = getattr(st, "rjoy_fwd", 0) + 1

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
        self.gw_seq = (self.gw_seq + 1) & 0xFFFF
        self.pending[self.gw_seq] = (st.id, seq, ftype, now, age)
        st.cmds += 1
        env = struct.pack("<BBHB", st.id, 1 if st.local else 0, self.gw_seq, ftype) + payload
        self.gw.aurix.send(proto.T_GW_CMD, env)
        # forget answers that never came
        if len(self.pending) > 512:
            for k in sorted(self.pending, key=lambda k: self.pending[k][3])[:256]:
                del self.pending[k]

    # --- from the AURIX --------------------------------------------------------------
    def _on_gw_ack(self, f):
        if len(f.payload) < 5:
            return
        sid, gseq, ftype, result = struct.unpack_from("<BHBB", f.payload)
        p = self.pending.pop(gseq, None)
        st = self.stations.get(sid)
        if p is None or st is None or p[0] != sid:
            return
        text = proto.RESULTS.get(result, "error %d" % result)
        st.send({"t": "ack", "seq": p[1], "type": ftype, "result": result, "text": text})
        self.gw.audit("cmd", station=sid, user=st.user, type=ftype, seq=p[1], result=result, text=text,
                      age_ms=round(p[4] * 1000))

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
        if helm is not None and helm != self.helm_seen:
            if self.helm_seen is not None:
                self.gw.audit("helm", holder=helm, holder_name=self.helm_name(helm))
            self.helm_seen = helm

    def _on_reply(self, f):
        msg = {"t": "frame", "type": f.type, "b": f.payload.hex()}
        for st in list(self.stations.values()):
            st.send(msg)

    def helm_name(self, helm):
        if helm in (None, 0):
            return "nobody"
        if helm == 1:
            return "USB laptop (local)"
        st = self.stations.get(helm)
        return "%s (%s, %s)" % (st.name, st.user, "local" if st.local else "remote") if st else "station %d" % helm

    # --- every second ------------------------------------------------------------------
    def tick(self):
        now = time.time()
        saver = self.gw.data_saver() if hasattr(self.gw, "data_saver") else False
        self.ping_id += 1
        for st in list(self.stations.values()):
            if now - st.last_pong > self.lost_s:
                self._drop(st, "lost (no answer for %.0f s)" % self.lost_s)
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
        self.server.udp_packet(data)
