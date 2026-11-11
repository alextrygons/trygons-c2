"""The per-flight MAVLink telemetry log (.tlog; step X1, owner 2026-11-08 J).

Standard format (Mission Planner "Telemetry Logs", MAVExplorer, UAV Log Viewer, pymavlink mavutil): per packet an
8-byte BIG-endian timestamp (microseconds since the Unix epoch) followed by the raw MAVLink 2 packet.

  <data>/uav_logs/<drone name>/<YYYYMMDD-HHMMSS>.tlog      same folder + naming as the CSV flight log (flightlog.py);
  a flight that grows past MAX_FILE_BYTES goes on in <stamp>-2.tlog, -3 ... (the CSV name stays the base name).

Written by the ROUTER (router_worker.py) for the packets it RECEIVES from the drone (after the CRC, allow.check_in and
the sysid check) and for the commands it SENT (after allow.check_out): a message the allow-list refused is NOT written.
Only while the drone is armed (one file per flight). Writing is done by a helper thread (a queue; full = dropped and
counted; the router's loop never waits for a disk). Size-capped: MAX_FILE_BYTES per file, MAX_DIR_BYTES per drone
(the oldest files go first). NEVER uploaded automatically: the only way out is the station's export (flightlog.read
fmt "tlog") or copying the file from the gateway PC.
Pure Python, no network, no MAVLink writer (it only stores bytes that tx() / rx() already handled).
"""
import os
import queue
import re
import struct
import threading
import time

from . import mavlink

MAX_FILE_BYTES = 40 * 1024 * 1024
MAX_DIR_BYTES = 400 * 1024 * 1024
FILE_RE = re.compile(r"^(\d{8}-\d{6})(?:-(\d+))?\.tlog$")
QUEUE_MAX = 5000


def stamp(t):
    return time.strftime("%Y%m%d-%H%M%S", time.gmtime(t))


def pack(t_us, raw):
    return struct.pack(">Q", int(t_us)) + bytes(raw)


class TlogWriter:
    def __init__(self, folder):
        self.folder = folder                       # None = no storage: nothing written
        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self.open = {}                             # slot -> {"name", "base", "part", "size"} (router thread only)
        self.last_us = {}                          # slot -> the last timestamp (never goes back)
        self.dropped = 0
        self.packets = 0
        self.errors = 0
        self.th = None
        if self.folder:
            self.th = threading.Thread(target=self._run, daemon=True, name="uav-tlog")
            self.th.start()

    def begin(self, slot, name, now=None):
        """A flight starts (the ARM): the next packets go into a new file."""
        if not self.folder:
            return
        self.end(slot)
        now = time.time() if now is None else now
        self.open[slot] = {"name": name, "base": stamp(now), "part": 1, "size": 0}

    def end(self, slot):
        self.open.pop(slot, None)

    def is_open(self, slot):
        return slot in self.open

    def write(self, slot, raw, now=None):
        """One packet of slot's flight (router thread). now = wall-clock seconds (default time.time())."""
        o = self.open.get(slot)
        if o is None or not raw:
            return False
        now = time.time() if now is None else now
        t_us = max(int(now * 1e6), self.last_us.get(slot, 0))        # timestamps never go back
        self.last_us[slot] = t_us
        rec = pack(t_us, raw)
        if o["size"] + len(rec) > MAX_FILE_BYTES:
            o["part"] += 1
            o["size"] = 0
        o["size"] += len(rec)
        fn = o["base"] + ("" if o["part"] == 1 else "-%d" % o["part"]) + ".tlog"
        try:
            self.q.put_nowait((o["name"], fn, rec))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def close(self):
        if self.th is not None:
            try:
                self.q.put_nowait(None)
            except queue.Full:
                pass

    def flush(self, timeout=5.0):
        """Wait until the queue is written (tests)."""
        end = time.time() + timeout
        while time.time() < end and self.q.unfinished_tasks:
            time.sleep(0.01)

    def _run(self):
        handles = {}                               # (name, file) -> open file object
        try:
            while True:
                item = self.q.get()
                if item is None:
                    self.q.task_done()
                    return
                nm, fn, rec = item
                try:
                    f = handles.get((nm, fn))
                    if f is None:
                        d = os.path.join(self.folder, nm)
                        os.makedirs(d, exist_ok=True)
                        for k in [k for k in handles if k[0] == nm]:
                            handles.pop(k).close()
                        f = handles[(nm, fn)] = open(os.path.join(d, fn), "ab")
                        self._prune(d, keep=fn)
                    f.write(rec)
                    f.flush()
                    self.packets += 1
                except OSError:
                    self.errors += 1
                self.q.task_done()
        finally:
            for f in handles.values():
                try:
                    f.close()
                except OSError:
                    pass

    @staticmethod
    def _prune(d, keep):
        """The oldest .tlog files of a drone go first when its folder holds more than MAX_DIR_BYTES."""
        try:
            files = []
            for fn in os.listdir(d):
                if FILE_RE.match(fn):
                    p = os.path.join(d, fn)
                    files.append((os.path.getmtime(p), fn, p, os.path.getsize(p)))
            total = sum(x[3] for x in files)
            for _m, fn, p, size in sorted(files):
                if total <= MAX_DIR_BYTES:
                    break
                if fn == keep:
                    continue
                os.remove(p)
                total -= size
        except OSError:
            pass


def parts_of(folder, name, base):
    """The .tlog files of one flight (base = '<stamp>'), in order: [path, ...]."""
    d = os.path.join(folder, name)
    out = []
    try:
        for fn in os.listdir(d):
            m = FILE_RE.match(fn)
            if m and m.group(1) == base:
                out.append((int(m.group(2) or 1), os.path.join(d, fn)))
    except OSError:
        return []
    return [p for _n, p in sorted(out)]


def parse(data):
    """-> [(t_us, packet bytes)] of a .tlog's bytes (MAVLink 2 packets). Raises
    ValueError on a broken file. Used by the tests as the fallback parser when pymavlink is not installed."""
    out, i = [], 0
    while i < len(data):
        if len(data) - i < 8 + 12:
            raise ValueError("truncated record at %d" % i)
        t_us = struct.unpack_from(">Q", data, i)[0]
        if data[i + 8] != mavlink.STX:
            raise ValueError("no MAVLink 2 start byte at %d" % (i + 8))
        ln, inc = data[i + 9], data[i + 10]
        total = 10 + ln + 2 + (13 if inc & 1 else 0)
        if len(data) - i - 8 < total:
            raise ValueError("truncated packet at %d" % i)
        out.append((t_us, bytes(data[i + 8:i + 8 + total])))
        i += 8 + total
    return out
