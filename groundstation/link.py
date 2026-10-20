"""Connection to the boat (USB virtual COM) or the simulator (TCP), with
heartbeat, frame decoding and a log of every frame.

    link = Link("COM5")              # real board, 921600 baud
    link = Link("tcp:127.0.0.1:5760")  # simulator (autopilot_sim server)
    link.start()
    ...  link.events.get() -> ("telemetry", dict) / ("ack", dict) /
                              ("param", dict) / ("prog_wp", dict) /
                              ("prog_error", dict) / ("status", text)
    link.send(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
"""
import json
import os
import queue
import socket
import threading
import time

import proto
import tlg

BAUD = 921600
HEARTBEAT_S = 0.5    # boat alarms after 3 s without a frame
STALE_S = 0.1          # incomplete frame older than this: rescan (like link.c)


def list_ports():
    """Available COM ports, plus the simulator address."""
    ports = []
    try:
        from serial.tools import list_ports as lp
        ports = [p.device for p in lp.comports()]
    except Exception:
        pass
    return ["tcp:127.0.0.1:5760"] + ports


class Logger:
    """One JSON object per line: every frame in both directions.
    {"t": wall time, "dir": "rx"|"tx", "type": n, "seq": n, "hex": "...",
     "telemetry": {...} (rx telemetry only)}"""

    def __init__(self, folder="logs"):
        os.makedirs(folder, exist_ok=True)
        self.path = os.path.join(folder, time.strftime("log_%Y%m%d_%H%M%S.jsonl"))
        self.f = open(self.path, "w", encoding="utf-8")
        self.lock = threading.Lock()
        self.count = 0
        # the same telemetry in the boat's log format (.TLG): the log
        # viewer / CSV tools read both
        self.tlg_path = self.path[:-6] + ".TLG"
        self.tlg = tlg.Writer(self.tlg_path)
        self.t_boat = 0
        self.sent = {}                  # seq -> (type, payload) for the TLG command records

    def write(self, direction, frame, extra=None):
        rec = {"t": round(time.time(), 3), "dir": direction, "type": frame.type,
               "seq": frame.seq, "hex": frame.payload.hex()}
        if extra:
            rec.update(extra)
        with self.lock:
            self.f.write(json.dumps(rec) + "\n")
            self.count += 1
            if direction == "tx":
                self.sent[frame.seq] = (frame.type, frame.payload)
            elif extra and "telemetry" in extra:
                self.t_boat = extra["telemetry"]["t_ms"]
                self.tlg.telemetry(frame.payload, extra["telemetry"])
            elif extra and "ack" in extra:
                a = extra["ack"]
                _t, p = self.sent.pop(a["seq"], (a["cmd"], b""))
                self.tlg.pc_command(self.t_boat, a["cmd"], a["seq"], a["result"], p)
            if self.count % 50 == 0:
                self.f.flush()
                self.tlg.flush()

    def close(self):
        with self.lock:
            self.f.close()
            self.tlg.close()


class Link:
    def __init__(self, address, log=True, log_folder="logs", heartbeat_s=HEARTBEAT_S):
        self.address = address
        self.events = queue.Queue()
        self.rx = proto.Receiver()
        self.seq = 0
        self.conn = None
        self.running = False
        self.connected = False
        self.last_rx = 0.0
        self.logger = Logger(log_folder) if log else None
        self.send_lock = threading.Lock()
        self.threads = []
        self.heartbeat_s = heartbeat_s
        self.t2 = {}                # latest TELEM2 (merged into every telemetry dict)

    # --- connection --------------------------------------------------------
    def _open(self):
        if self.address.startswith("tcp:"):
            _, host, port = self.address.split(":")
            s = socket.create_connection((host, int(port)), timeout=3)
            s.settimeout(0.05)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return ("tcp", s)
        import serial
        return ("serial", serial.Serial(self.address, BAUD, timeout=0.05))

    def _read(self):
        kind, c = self.conn
        if kind == "tcp":
            try:
                data = c.recv(4096)
            except socket.timeout:
                return b""
            if not data:
                raise ConnectionError("simulator closed the connection")
            return data
        return c.read(4096)

    def _write(self, data):
        kind, c = self.conn
        if kind == "tcp":
            c.sendall(data)
        else:
            c.write(data)

    def start(self):
        self.conn = self._open()
        self.running = True
        self.connected = True
        self.events.put(("status", "connected to " + self.address))
        for fn in (self._rx_loop, self._hb_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self.threads.append(t)

    def stop(self):
        self.running = False
        for t in self.threads:
            t.join(timeout=1)
        try:
            self.conn[1].close()
        except Exception:
            pass
        if self.logger:
            self.logger.close()
        self.connected = False

    # --- sending -----------------------------------------------------------
    def send(self, ftype, payload=b""):
        """Send one command. Returns its seq (the ACK echoes it)."""
        with self.send_lock:
            self.seq = (self.seq + 1) & 0xFF
            frame = proto.Frame(ftype, self.seq, payload)
            if self.logger and ftype != proto.T_HEARTBEAT:
                self.logger.write("tx", frame)
            try:
                self._write(frame.raw())
            except Exception as e:
                self._lost(e)
            return self.seq

    def _lost(self, err):
        if self.connected:
            self.connected = False
            self.events.put(("status", "DISCONNECTED: %s" % err))

    # --- threads -----------------------------------------------------------
    def _hb_loop(self):
        while self.running:
            if self.connected:
                self.send(proto.T_HEARTBEAT)
            time.sleep(self.heartbeat_s)

    def _rx_loop(self):
        last_data = time.time()
        while self.running:
            try:
                data = self._read()
            except Exception as e:
                self._lost(e)
                time.sleep(0.2)
                continue
            now = time.time()
            if data:
                last_data = now
                self.rx.push(data)
            elif self.rx.pending() and now - last_data > STALE_S:
                self.rx.stale()
                last_data = now
            while True:
                f = self.rx.next()
                if f is None:
                    break
                self.last_rx = now
                self._dispatch(f)

    def _dispatch(self, f):
        try:
            if f.type == proto.T_TELEM2:
                self.t2 = proto.decode_telem2(f.payload)
                return
            if f.type == proto.T_TELEMETRY:
                d = proto.decode_telemetry(f.payload)
                d.update(self.t2)
                if self.logger:
                    self.logger.write("rx", f, {"telemetry": d})
                self.events.put(("telemetry", d))
            elif f.type == proto.T_ACK:
                d = proto.decode_ack(f.payload)
                if self.logger:
                    self.logger.write("rx", f, {"ack": d})
                self.events.put(("ack", d))
            elif f.type == proto.T_PARAM_VALUE:
                d = proto.decode_param(f.payload)
                if self.logger:
                    self.logger.write("rx", f, {"param": d})
                self.events.put(("param", d))
            elif f.type == proto.T_PROG_WP_VALUE:
                d = proto.decode_prog_wp(f.payload)
                if self.logger:
                    self.logger.write("rx", f, {"prog_wp": d})
                self.events.put(("prog_wp", d))
            elif f.type == proto.T_JOYMAP_VALUE:
                d = proto.decode_joymap(f.payload)
                if self.logger:
                    self.logger.write("rx", f, {"joymap": d})
                self.events.put(("joymap", d))
            elif f.type == proto.T_JOY_RAW:
                d = proto.decode_joy_raw(f.payload)       # polled often: not logged
                self.events.put(("joy_raw", d))
            elif f.type == proto.T_PROG_ERROR:
                d = proto.decode_prog_error(f.payload)
                if self.logger:
                    self.logger.write("rx", f, {"prog_error": d})
                self.events.put(("prog_error", d))
            elif self.logger:
                self.logger.write("rx", f)
        except (ValueError, IndexError) as e:
            self.events.put(("status", "bad frame 0x%02x: %s" % (f.type, e)))


class Waiter:
    """Helper for scripts / tests: wait for an ACK or a telemetry condition."""

    def __init__(self, link):
        self.link = link
        self.telem = None
        self.acks = {}
        self.params = {}
        self.prog = {}
        self.prog_error = None
        self.joymap = None
        self.joy_raw = None

    def pump(self, timeout=0.05):
        try:
            item = self.link.events.get(timeout=timeout)
        except queue.Empty:
            return
        while True:
            kind, d = item
            if kind == "telemetry":
                self.telem = d
                self.on_telemetry(d)
            elif kind == "ack":
                self.acks[d["seq"]] = d
            elif kind == "param":
                self.params[d["id"]] = d
            elif kind == "prog_wp":
                self.prog[d["index"]] = d
            elif kind == "prog_error":
                self.prog_error = d
            elif kind == "joymap":
                self.joymap = d
            elif kind == "joy_raw":
                self.joy_raw = d
            try:
                item = self.link.events.get_nowait()
            except queue.Empty:
                return

    def on_telemetry(self, d):
        """Override to see every frame (not only the latest)."""

    def command(self, ftype, payload=b"", timeout=3.0):
        seq = self.link.send(ftype, payload)
        end = time.time() + timeout
        while time.time() < end:
            self.pump()
            if seq in self.acks:
                return self.acks.pop(seq)
        return None

    def until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.pump()
            if self.telem is not None and cond(self.telem):
                return self.telem
        return None
