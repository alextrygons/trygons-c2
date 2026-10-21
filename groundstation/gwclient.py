"""Ground station <-> gateway (Raspberry Pi) over the VPN or the boat LAN.

GatewayLink has the same interface as link.Link (start, stop, send, events),
so the ground station works the same through the gateway as on the USB
cable. Protocol: gateway/stations.py (JSON lines on TCP 5761).

    gw = GatewayLink("trygons-gw:5761", user="alex", password="...")
    gw.start()                       # logs in; gw.welcome / gw.error tell how it went
    seq = gw.send(proto.T_HELM, proto.p_helm(0))
"""
import hashlib
import hmac
import json
import queue
import socket
import threading
import time

import proto
from link import Logger


def full_from_lite(lite, last_full=None):
    """A complete telemetry dict from a reduced (lite) one: the last full
    values where the lite frame has nothing (marked d["lite"] = True)."""
    if last_full is None:
        last_full = proto.decode_telemetry(bytes(proto.TELEM_SIZE))
    d = dict(last_full)
    d.update(lite)
    d["lite"] = True
    d["mode_name"] = proto.MODES[d["mode"]] if d.get("mode", 0) < len(proto.MODES) else "?"
    return d


class GatewayLink:
    def __init__(self, address, user, password=None, key=None, station="ground station", log=False,
                 log_folder="logs", local_addr=None, clock_offset=0.0):
        host, port = address.rsplit(":", 1)
        self.host, self.port = host, int(port)
        self.user, self.password, self.key, self.station = user, password, key, station
        self.local_addr = local_addr            # tests: connect from a given address
        self.clock_offset = clock_offset        # tests: a PC clock that is wrong
        self.events = queue.Queue()
        self.connected = False
        self.last_rx = 0.0
        self.welcome = None
        self.error = None
        self.gw_status = None
        self.t2 = {}
        self.last_full = None
        self.tmode = "full"
        self.seq = 0
        self.sock = None
        self.running = False
        self.send_lock = threading.Lock()
        self.logger = Logger(log_folder) if log else None
        self.bytes_in = 0
        self._th = None
        self.udp = None                         # remote joystick packets (UDP, same host / port)
        self.use_udp = True

    def now(self):
        return time.time() + self.clock_offset

    # --- connection -----------------------------------------------------------------
    def start(self, timeout=5.0):
        s = socket.create_connection((self.host, self.port), timeout=timeout,
                                     source_address=(self.local_addr, 0) if self.local_addr else None)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(None)
        self.sock = s
        self.running = True
        login = {"t": "login", "user": self.user, "station": self.station, "ts": self.now()}
        if self.key:
            login["key"] = self.key
        else:
            login["password"] = self.password
        self._send_obj(login)
        self._th = threading.Thread(target=self._rx_loop, daemon=True)
        self._th.start()
        end = time.time() + timeout
        while time.time() < end and self.welcome is None and self.error is None and self.running:
            time.sleep(0.02)
        if self.welcome is None:
            self.stop()
            raise ConnectionError(self.error or "no answer from the gateway")
        return self.welcome

    def stop(self):
        self.running = False
        self.connected = False
        try:
            if self.sock:
                # shutdown first: close() alone leaves the connection open while the
                # reader thread still uses it (the gateway would keep the station - and
                # its HELM - and the reader would even answer its pings)
                self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        if self.logger:
            self.logger.close()

    def _send_obj(self, obj):
        data = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        with self.send_lock:
            try:
                self.sock.sendall(data)
            except OSError as e:
                self._lost(e)

    def _lost(self, err):
        if self.connected or self.running:
            self.events.put(("status", "GATEWAY DISCONNECTED: %s" % err))
        self.connected = False
        self.running = False

    # --- sending (same as Link.send) ----------------------------------------------------
    def send(self, ftype, payload=b"", what=None, confirm=None):
        """what: text for the driver's ACCEPT question; confirm="self" in
        SINGLE-OPERATOR mode after the captain's own confirmation."""
        if not self.welcome:
            return None
        self.seq += 1
        obj = {"t": "cmd", "sid": self.welcome["sid"], "seq": self.seq, "ts": self.now(),
               "type": ftype, "p": bytes(payload).hex()}
        if what:
            obj["what"] = what
        if confirm:
            obj["confirm"] = confirm
        self._send_obj(obj)
        if self.logger and ftype != proto.T_HEARTBEAT:
            self.logger.write("tx", proto.Frame(ftype, self.seq, bytes(payload)))
        return self.seq

    def video_url(self, ack, cam):
        """RTSP address of a camera stream from a cams_ack answer (None if OFF)."""
        path = (ack or {}).get("streams", {}).get(cam)
        if not path:
            return None
        return "rtsp://%s:%s@%s:%d/%s?%s" % (ack["user"], ack["password"], self.host, ack["rtsp_port"], path,
                                             ack["query"])

    def send_msg(self, obj):
        """Gateway-only messages (channel, cameras, save, ...)."""
        self._send_obj(obj)

    def send_rjoy(self, obj):
        """The remote joystick packet: UDP (no stalls on a lossy link), signed
        with this session's key; TCP if UDP is switched off."""
        if not (self.use_udp and self.welcome and self.welcome.get("udp_key")):
            return self._send_obj(obj)
        if self.udp is None:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self.local_addr:
                self.udp.bind((self.local_addr, 0))
        body = json.dumps(dict(obj, sid=self.welcome["sid"]), separators=(",", ":")).encode()
        mac = hmac.new(bytes.fromhex(self.welcome["udp_key"]), body, hashlib.sha256).hexdigest()[:32].encode()
        try:
            self.udp.sendto(body + b"|" + mac, (self.host, self.port))
        except OSError:
            pass

    # --- receiving ------------------------------------------------------------------------
    def _rx_loop(self):
        f = self.sock.makefile("rb")
        try:
            for line in f:
                self.bytes_in += len(line)
                self.last_rx = time.time()
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                self._dispatch(m)
        except OSError as e:
            self._lost(e)
            return
        self._lost("closed by the gateway")

    def _dispatch(self, m):
        try:
            self._dispatch2(m)
        except (ValueError, KeyError, TypeError) as e:
            if self.running:                    # (a closed log file after stop() is fine)
                self.events.put(("status", "bad message from the gateway: %s" % e))

    def _dispatch2(self, m):
        t = m.get("t")
        if t == "gping":
            self._send_obj({"t": "gpong", "id": m["id"], "ts": self.now()})
        elif t == "tm":
            b = bytes.fromhex(m["b"])
            d = proto.decode_telemetry(b)
            d.update(self.t2)
            self.last_full = d
            if self.logger:
                self.logger.write("rx", proto.Frame(proto.T_TELEMETRY, 0, b), {"telemetry": d})
            self.events.put(("telemetry", d))
        elif t == "t2":
            self.t2 = proto.decode_telem2(bytes.fromhex(m["b"]))
        elif t == "lite":
            self.events.put(("telemetry", full_from_lite(m["d"], self.last_full)))
        elif t == "ack":
            r = m.get("result")
            a = {"cmd": m.get("type"), "seq": m.get("seq"), "result": r, "ok": r == 0,
                 "text": m.get("text") or proto.RESULTS.get(r, "error %s" % r),
                 "cmd_name": proto.TYPE_NAMES.get(m.get("type"), str(m.get("type")))}
            if self.logger and isinstance(a["seq"], int) and isinstance(r, int) and r < 256:
                self.logger.write("rx", proto.Frame(proto.T_ACK, a["seq"] & 0xFF,
                                                    bytes([a["cmd"] & 0xFF, a["seq"] & 0xFF, r])), {"ack": a})
            self.events.put(("ack", a))
        elif t == "frame":
            b = bytes.fromhex(m["b"])
            ft = m["type"]
            kinds = {proto.T_PARAM_VALUE: ("param", proto.decode_param),
                     proto.T_PROG_WP_VALUE: ("prog_wp", proto.decode_prog_wp),
                     proto.T_PROG_ERROR: ("prog_error", proto.decode_prog_error),
                     proto.T_JOYMAP_VALUE: ("joymap", proto.decode_joymap),
                     proto.T_JOY_RAW: ("joy_raw", proto.decode_joy_raw)}
            if ft in kinds:
                self.events.put((kinds[ft][0], kinds[ft][1](b)))
        elif t == "status":
            self.gw_status = m
            self.events.put(("gw_status", m))
        elif t == "tmode":
            self.tmode = m.get("mode")
            self.events.put(("status", "telemetry %s (%s)" % (m.get("mode"), m.get("why"))))
        elif t == "welcome":
            self.welcome = m
            self.connected = True
        elif t == "error":
            self.error = m.get("text")
            self.events.put(("status", "GATEWAY: %s" % self.error))
        else:
            self.events.put((t, m))                     # later sections (channel, camera, ...)
