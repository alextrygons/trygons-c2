"""Sync to the shore PC (section G): the boat's SSD -> the shore PC, over the VPN.

The gateway connects OUT to the shore receiver (groundstation/syncrecv.py,
TCP 5770 on the shore PC's VPN address) - nothing listens on the boat.

What, in this order (a higher class always goes first):
  1  black box / alarms   AURIX log files with an alarm, fault or SAVE in
                          them; every SAVE's event.json
  2  event logs           audit log, log gap list
  3  telemetry logs       every other AURIX log file
  4  saved events         the video of every SAVE
  5  ring video           only if recording.ring_sync is on (default OFF)
Files are sent once they have not changed for settle_s. Files that grow
(the audit log of today) are continued from where the shore copy ends.

Rules
  * only on a channel with sync_allowed (never STARLINK); paused while
    REMOTE MANUAL is driving; never anything the cameras / stations need:
    the send rate follows the link's delay (spare bandwidth only, like
    LEDBAT): our own ping to the shore rises -> slower at once
  * resumable: the shore keeps <file>.part; on reconnect its size and
    SHA-256 are checked against the start of our file and it continues
  * "synced" only when the shore has checked the SHA-256 of the whole file:
    then <file>.synced is written here (size + SHA-256)
  * sync never deletes anything (the storage policy in recorder.py deletes
    only synced data, and only when the SSD runs low)
"""
import asyncio
import hashlib
import hmac
import json
import os
import time

from common import log

SYNC, EVENT = 0xA5, 0x20
INCIDENT_EVENTS = {3, 11, 14, 18}          # ALARM_ON, FAULT, MONITOR, SAVE
CLASS_NAMES = {1: "black box / alarms", 2: "event logs", 3: "telemetry logs", 4: "saved events", 5: "ring video"}


def sha256_file(path, limit=None):
    h = hashlib.sha256()
    left = limit
    with open(path, "rb") as f:
        while left is None or left > 0:
            b = f.read(1 << 20 if left is None else min(1 << 20, left))
            if not b:
                break
            h.update(b)
            if left is not None:
                left -= len(b)
    return h.hexdigest()


def marker(path):
    try:
        with open(path + ".synced", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def tlg_has_incident(path):
    """An AURIX log with an alarm / fault / monitor trip / SAVE request in it."""
    try:
        with open(path, "rb") as f:
            buf = f.read()
    except OSError:
        return False
    i, n = 4, len(buf)
    while i + 10 <= n:
        if buf[i] != SYNC:
            i += 1
            continue
        rtype, ln = buf[i + 1], buf[i + 2] | (buf[i + 3] << 8)
        if rtype == EVENT and ln >= 2 and i + 10 + ln <= n:
            if (buf[i + 8] | (buf[i + 9] << 8)) in INCIDENT_EVENTS:
                return True
        i += 10 + ln
    return False


class SyncClient:
    def __init__(self, cfg, gw):
        self.cfg = cfg
        self.gw = gw
        self.root = gw.storage.root
        sec = gw.cfg.get("secrets", {})
        self.key = (sec.get("sync_key") or cfg.get("key") or "").encode()
        self.shore = cfg.get("shore")                   # "host:port" (the shore PC's VPN name)
        self.enabled = bool(cfg.get("enabled", True) and self.shore and self.key)
        self.chunk = cfg.get("chunk", 16384)
        self.cap = cfg.get("max_kbps", 8000) * 125.0    # bytes/s, never more than this
        self.rate = cfg.get("start_kbps", 256) * 125.0
        self.min_rate = cfg.get("min_kbps", 32) * 125.0
        self.target = cfg.get("target_delay_ms", 100) / 1000.0
        self.settle = cfg.get("settle_s", 60.0)
        self.state, self.reason = "OFF", "not started"
        self.queue = []
        self.current = None
        self.rtt = None
        self.base = None
        self.rtts = []                                  # (t, rtt) last 60 s
        self.hold_until = 0.0
        self.sent_b = 0
        self.confirmed = 0
        self.last_ok = None
        self.errors = 0
        self.last_error = ""
        self.resumed = 0
        self.restarted = 0
        self._inc = {}                                  # path -> (size, mtime, incident)
        self._next_scan = 0.0
        self.reader = self.writer = None
        self.waiters = {}
        self.cut_after = None                           # tests: drop the connection after N bytes
        self.hold_after_cut = False                     # tests: ... and wait (hold) until released
        self.hold = False
        self.done_list = []                             # confirmed files, in order (last 2000)

    # --- what to send --------------------------------------------------------------
    def classify(self, rel, path, st):
        p = rel.replace("\\", "/")
        if p.startswith("aurix_logs/"):
            if p.endswith(".TLG"):
                k = (st.st_size, st.st_mtime)
                c = self._inc.get(path)
                if not c or c[0] != k:
                    c = (k, tlg_has_incident(path))
                    self._inc[path] = c
                return 1 if c[1] else 3
            return 2 if p.endswith(".csv") else None
        if p.startswith("audit/") and p.endswith(".jsonl"):
            return 2
        if p.startswith("video/events/"):
            parts = p.split("/")
            if len(parts) < 4:
                return None
            ev = self._event_state(os.path.join(self.root, "video", "events", parts[2]))
            if ev in (None, "recording"):
                return None                             # still being written
            return 1 if parts[-1] == "event.json" else (4 if p.endswith(".ts") else None)
        if p.startswith("video/ring/") and p.endswith(".ts"):
            return 5 if self.gw.rec.ring_sync else None
        return None

    @staticmethod
    def _event_state(folder):
        try:
            with open(os.path.join(folder, "event.json"), encoding="utf-8") as f:
                return json.load(f).get("state")
        except (OSError, ValueError):
            return None

    def scan(self):
        """Unsynced, settled files, by class then oldest first."""
        out = []
        now = time.time()
        for dirpath, _dirs, files in os.walk(self.root):
            for n in files:
                if n.endswith((".synced", ".part", ".tmp")) or n.startswith("."):
                    continue
                path = os.path.join(dirpath, n)
                try:
                    st = os.stat(path)
                except OSError:
                    continue
                if now - st.st_mtime < self.settle:
                    continue
                rel = os.path.relpath(path, self.root).replace("\\", "/")
                cls = self.classify(rel, path, st)
                if cls is None:
                    continue
                m = marker(path)
                if m and m.get("size") == st.st_size:
                    continue
                out.append((cls, st.st_mtime, rel, path, st.st_size))
        out.sort()
        self.queue = out
        return out

    # --- allowed? ---------------------------------------------------------------------
    def allowed(self):
        if not self.enabled:
            return False, "not set up (sync.shore + sync_key)"
        if self.hold:
            return False, "held"
        if not self.gw.storage.usable:
            return False, "SSD %s" % self.gw.storage.state
        if not self.gw.net.sync_allowed():
            ch = self.gw.net.active or "none"
            return False, "channel %s: no sync (metered / emergency)" % ch
        if self.gw.telem2.get("remote_state") in (1, 2) or (self.gw.telem or {}).get("mode") == 5:
            return False, "REMOTE MANUAL driving: link kept free"
        return True, ""

    # --- connection -------------------------------------------------------------------
    async def _connect(self):
        host, port = self.shore.rsplit(":", 1)
        self.reader, self.writer = await asyncio.wait_for(asyncio.open_connection(host, int(port)), 10)
        self.writer.get_extra_info("socket").setsockopt(6, 1, 1)        # TCP_NODELAY
        ch = json.loads(await asyncio.wait_for(self.reader.readline(), 10))
        mac = hmac.new(self.key, ch["nonce"].encode(), hashlib.sha256).hexdigest()
        self._send({"t": "auth", "boat": self.gw.cfg.get("boat"), "mac": mac})
        w = json.loads(await asyncio.wait_for(self.reader.readline(), 10))
        if w.get("t") != "welcome":
            raise ConnectionError("shore refused: %s" % w.get("why"))
        self.waiters = {}
        self.rx_task = asyncio.create_task(self._rx())
        self.ping_task = asyncio.create_task(self._pinger())

    def _send(self, obj, data=b""):
        self.writer.write(json.dumps(obj).encode() + b"\n" + data)

    async def _rx(self):
        try:
            while True:
                line = await self.reader.readline()
                if not line:
                    raise ConnectionError("shore closed")
                m = json.loads(line)
                if m.get("t") == "pong":
                    self._on_rtt(time.time() - m["ts"])
                    continue
                f = self.waiters.pop(m.get("path"), None)
                if f and not f.done():
                    f.set_result(m)
        except (OSError, ValueError, ConnectionError, asyncio.IncompleteReadError) as e:
            for f in self.waiters.values():
                if not f.done():
                    f.set_exception(ConnectionError(str(e)))
            self.waiters = {}

    async def _pinger(self):
        while True:
            self._send({"t": "ping", "ts": time.time()})
            await asyncio.sleep(0.25)

    def _on_rtt(self, rtt):
        now = time.time()
        self.rtt = rtt
        self.rtts = [(t, r) for t, r in self.rtts if now - t < 60.0] + [(now, rtt)]
        self.base = min(r for _t, r in self.rtts)
        if rtt > self.base + self.target:                # our data is queueing: back off at once,
            if now >= self.hold_until:                  # once per round trip
                self.rate = max(self.min_rate, self.rate * 0.6)
                self.hold_until = now + rtt
        elif rtt < self.base + self.target / 2 and now >= self.hold_until:
            self.rate = min(self.cap, self.rate + max(self.rate * 0.03, 2000.0))   # slowly up

    def spare_cap(self):
        """Never more than what the cameras / stations leave (budget of the channel)."""
        budget = self.gw.net.channels.get(self.gw.net.active or "", {}).get("sync_budget_kbps")
        if not budget:
            return self.cap
        used = self.gw.cams.status().get("total_kbps", 0) if self.gw.cams else 0
        return max(self.min_rate, (budget - used) * 125.0)

    async def _ask(self, obj, data=b"", timeout=60):
        fut = asyncio.get_running_loop().create_future()
        self.waiters[obj["path"]] = fut
        self._send(obj, data)
        await self.writer.drain()
        return await asyncio.wait_for(fut, timeout)

    def _close(self):
        for t in ("rx_task", "ping_task"):
            task = getattr(self, t, None)
            if task:
                task.cancel()
        if self.writer:
            try:
                self.writer.close()
            except OSError:
                pass
        self.reader = self.writer = None

    # --- one file -----------------------------------------------------------------------
    async def send_file(self, rel, path, size):
        loop = asyncio.get_running_loop()
        have = await self._ask({"t": "stat", "path": rel})
        off = 0
        if have.get("size", 0) > 0:
            hs = have["size"]
            if hs <= size and await loop.run_in_executor(None, sha256_file, path, hs) == have["sha"]:
                off = hs
                if not have.get("complete") or hs < size:
                    self.resumed += 1 if off else 0
            else:
                self.restarted += 1
                off = 0
        self.current = {"path": rel, "size": size, "done": off}
        with open(path, "rb") as f:
            f.seek(off)
            t_last = time.time()
            while off < size:
                n = min(self.chunk, size - off)
                data = f.read(n)
                if not data:
                    break
                self._send({"t": "data", "path": rel, "off": off, "len": len(data), "new": off == 0}, data)
                await self.writer.drain()
                off += len(data)
                self.sent_b += len(data)
                self.current["done"] = off
                if self.cut_after is not None and self.sent_b >= self.cut_after:
                    self.cut_after = None
                    self.hold = self.hold_after_cut
                    raise ConnectionError("test: connection cut")
                # pace: never faster than the rate (and what the link has spare)
                rate = min(self.rate, self.spare_cap())
                t_last += len(data) / max(rate, 1.0)
                wait = t_last - time.time()
                if wait > 0:
                    await asyncio.sleep(wait)
                else:
                    t_last = time.time()
                if not self.allowed()[0]:
                    raise PauseSync()
        sha = await loop.run_in_executor(None, sha256_file, path, off)
        r = await self._ask({"t": "done", "path": rel, "size": off, "sha": sha})
        if r.get("t") == "ok" and r.get("sha") == sha:
            with open(path + ".synced", "w", encoding="utf-8") as f:
                json.dump({"size": off, "sha": sha, "utc": self.gw.clock.stamp()}, f)
            self.confirmed += 1
            self.last_ok = rel
            self.done_list = (self.done_list + [rel])[-2000:]
            return True
        self.restarted += 1
        raise ConnectionError("shore: %s %s" % (r.get("t"), r.get("why", "")))

    # --- main loop ------------------------------------------------------------------------
    async def run(self):
        loop = asyncio.get_running_loop()
        backoff = 1.0
        while True:
            ok, why = self.allowed()
            if not ok:
                if self.writer:
                    self._close()
                self.state, self.reason = ("OFF" if not self.enabled else "PAUSED"), why
                await asyncio.sleep(0.5)
                continue
            try:
                if time.time() >= self._next_scan or not self.queue:
                    await loop.run_in_executor(None, self.scan)
                    self._next_scan = time.time() + self.cfg.get("scan_s", 10.0)
                if not self.queue:
                    self.state, self.reason, self.current = "IDLE", "everything synced", None
                    await asyncio.sleep(1.0)
                    continue
                if not self.writer:
                    self.state, self.reason = "CONNECTING", self.shore
                    await self._connect()
                    backoff = 1.0
                cls, _mt, rel, path, size = self.queue.pop(0)
                self.state, self.reason = "SYNCING", CLASS_NAMES[cls]
                await self.send_file(rel, path, size)
            except PauseSync:
                self.queue = []
            except (OSError, ValueError, KeyError, ConnectionError, asyncio.TimeoutError) as e:
                self.errors += 1
                self.last_error = str(e) or type(e).__name__
                self.state, self.reason = "RETRY", self.last_error
                self._close()
                self.queue = []
                await asyncio.sleep(backoff)
                backoff = min(30.0, backoff * 2)

    def status(self):
        q = {}
        for cls, _mt, _rel, _p, size in self.queue:
            e = q.setdefault(CLASS_NAMES[cls], {"files": 0, "mb": 0.0})
            e["files"] += 1
            e["mb"] = round(e["mb"] + size / 1e6, 1)
        return {"state": self.state, "reason": self.reason, "rate_kbps": round(self.rate / 125.0),
                "rtt_ms": None if self.rtt is None else round(self.rtt * 1000),
                "base_ms": None if self.base is None else round(self.base * 1000),
                "queue": q, "current": self.current, "sent_mb": round(self.sent_b / 1e6, 1),
                "confirmed": self.confirmed, "last": self.last_ok, "errors": self.errors,
                "last_error": self.last_error, "resumed": self.resumed, "restarted": self.restarted}


class PauseSync(Exception):
    pass
