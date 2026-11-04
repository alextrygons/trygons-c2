"""AI on the UAV video (CLAUDE.md V "Telemetry / video / AI", docs/uav_design.md 8; step U4): the SAME eWaSR worker
(gateway/vision/worker.py, the "AI" badge) as a SECOND process, ONE thread, low priority, on the UAV feeds.

Rules (owner H5 + U1):
  - the boat's FRONT camera AI ALWAYS first: nothing runs while the boat's vision is PAUSED / DEGRADED (it needs the
    CPU) or the PC is busy (cpu_pct) - the UAV AI waits, the boat's never does;
  - only the UAV feeds OPEN ON A SCREEN of some station (cameras.uav_views) whose video runs, ROUND-ROBIN: one feed
    for dwell_s, then the next; the worker runs at max_fps (0.5) in all = <= 0.5 fps per feed;
  - output = boxes (+ a NOTICE when a feed's first object appears) - NEVER a speed cap, never a navigation request,
    never an alarm, never anything to the boat (nothing here is read by Gateway.nav_compose; the boat's stage-3 cap
    stays from the boat's own cameras only, docs 6.4);
  - eWaSR was trained on views from a boat at water level: a top-down aerial view is outside its training, its
    results there may be poor (W62: measured on the simulated VRX picture only - no real aerial video yet).
A worker crash / no model = state OFFLINE + why (a notice), restarted later. The worker process is started / stopped
from a helper thread (never on the event loop).
"""
import collections
import json
import os
import subprocess
import threading
import time

ACTIVE, WAITING, IDLE, OFFLINE = "ACTIVE", "WAITING", "IDLE", "OFFLINE"
DEFAULTS = {
    "enabled": True,
    "max_fps": 0.5,             # the whole UAV AI (all feeds together)
    "dwell_s": 8.0,             # one feed this long, then the next (the decoder needs ~2 s to start)
    "settle_s": 1.5,            # results of the first frames after a switch are not trusted to the new feed
    "idle_stop_s": 15.0,        # no feed open this long = the worker ends
    "cpu_pct": 85.0,            # the PC busier than this = wait
    "min_conf": 0.5,
    "keep_s": 12.0,             # boxes older than this are not shown
    "notice_s": 60.0,           # one "objects seen" notice per feed per minute at most
    "restart_s": 30.0,
}


class UavAi:
    def __init__(self, cfg, mgr):
        from common import deep_merge
        self.cfg = deep_merge(DEFAULTS, cfg or {})
        self.mgr = mgr
        self.gw = mgr.gw
        self.state, self.why = IDLE, "no UAV video open on a screen"
        self.proc = None
        self.inbox = collections.deque(maxlen=100)
        self.spawning = False
        self.next_start = 0.0
        self.feed, self.feed_t = None, 0.0
        self.order = []
        self.feeds = {}                        # slot -> {"boxes", "t", "frames", "n_last", "noticed"}
        self.last_open = 0.0
        self.paused = False
        self.fps, self.infer_ms = 0.0, None
        self.frames = 0
        self.starts = 0
        self.load = None
        try:
            from vision.manager import SysLoad
            self.load = SysLoad()
        except Exception:                                       # (osdep missing on an odd install)
            self.load = None
        self.cpu = None

    # --- the worker -----------------------------------------------------------------------------------------------
    def _paths(self):
        v = getattr(self.gw, "vision", None)
        py = v._python() if v is not None else None
        model = v._model() if v is not None else None
        return py, model

    def _spawn_bg(self):
        if self.spawning:
            return
        self.spawning = True
        self.next_start = time.time() + float(self.cfg["restart_s"])

        def run():
            try:
                self._spawn()
            finally:
                self.spawning = False
        threading.Thread(target=run, daemon=True).start()

    def _spawn(self):
        py, model = self._paths()
        if not py or not model or not os.path.exists(model):
            self.state, self.why = OFFLINE, "no vision Python / model (tools/get_vision_python.py, get_vision_model.py)"
            return
        import toolpaths
        here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vision")
        n = os.cpu_count() or 4
        core = max(0, n - 3)                                    # not the boat vision's cores (the last ones)
        args = [py, os.path.join(here, "worker.py"), "--model", model, "--threads", "1", "--cores", str(core),
                "--max-fps", str(float(self.cfg["max_fps"])), "--ffmpeg", toolpaths.ffmpeg() or "ffmpeg"]
        try:
            p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 text=True, bufsize=1, cwd=here)
        except OSError as e:
            self.state, self.why = OFFLINE, "the UAV AI worker did not start: %s" % e
            return
        self.proc = p
        self.starts += 1
        self.feed = None
        threading.Thread(target=self._read, args=(p,), daemon=True).start()

    def _read(self, p):
        try:
            for line in p.stdout:
                try:
                    self.inbox.append(json.loads(line))
                except ValueError:
                    pass
        except (OSError, ValueError):
            pass
        self.inbox.append({"t": "exit"})

    def send(self, obj):
        p = self.proc
        try:
            if p is not None and p.poll() is None:
                p.stdin.write(json.dumps(obj) + "\n")
                p.stdin.flush()
                return True
        except (OSError, ValueError):
            pass
        return False

    def stop(self):
        p, self.proc = self.proc, None
        if p is None:
            return
        try:
            p.stdin.write('{"cmd": "quit"}\n')
            p.stdin.flush()
        except (OSError, ValueError):
            pass

        def wait():
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
            except OSError:
                pass
        threading.Thread(target=wait, daemon=True).start()

    # --- which feed -----------------------------------------------------------------------------------------------
    def open_feeds(self):
        cams = getattr(self.gw, "cams", None)
        if cams is None or not hasattr(cams, "uav_views"):
            return []
        running = set(self.mgr.video_running())
        # "measure_feeds": ONLY for tools/gateway_load.py (the load with UAV AI and no station watching)
        opened = set(cams.uav_views()) | {int(s) for s in self.cfg.get("measure_feeds") or []}
        return sorted(s for s in opened if s in running)

    def blocked(self):
        """Why the UAV AI must wait now (the boat's FRONT camera AI first), or None."""
        v = getattr(self.gw, "vision", None)
        if v is not None and v.cfg.get("enabled", True) and (v.state in ("PAUSED", "DEGRADED") or v.paused):
            return "the boat's front camera AI needs the CPU (%s)" % v.state
        if self.load is not None:
            c = self.load.cpu_pct()
            self.cpu = c if c is not None else self.cpu
        if self.cpu is not None and self.cpu > float(self.cfg["cpu_pct"]):
            return "PC busy (CPU %d %%)" % self.cpu
        return None

    def url(self, slot):
        port = (self.gw.cfg.get("cameras") or {}).get("rtsp_port", 8554)
        return "rtsp://127.0.0.1:%d/uav_%d" % (port, slot)

    def tick(self, now=None):
        """1 x a second from the manager's tick (the event loop): cheap; process work in helper threads."""
        now = time.time() if now is None else now
        c = self.cfg
        while self.inbox:
            self._handle(self.inbox.popleft(), now)
        if not c["enabled"]:
            self.state, self.why = IDLE, "switched off"
            return self.stop()
        feeds = self.open_feeds()
        if feeds:
            self.last_open = now
        elif self.proc is not None and now - self.last_open > float(c["idle_stop_s"]):
            self.stop()
        if not feeds:
            self.feed = None
            if self.state != OFFLINE:
                self.state, self.why = IDLE, "no UAV video open on a screen"
            return
        why = self.blocked()
        if why:
            if not self.paused:
                self.paused = True
                self.send({"cmd": "pause"})
            self.state, self.why = WAITING, why
            return
        if self.paused:
            self.paused = False
            self.send({"cmd": "resume"})
        if self.proc is None or self.proc.poll() is not None:
            if self.proc is not None:
                self.proc = None
                self.state, self.why = OFFLINE, "the UAV AI worker stopped (restart in %.0f s)" % c["restart_s"]
                self.next_start = now + float(c["restart_s"])
            if now >= self.next_start:
                self._spawn_bg()
            return
        # round robin over the open feeds
        if self.feed not in feeds or now - self.feed_t >= float(c["dwell_s"]):
            nxt = feeds[0] if self.feed not in feeds else feeds[(feeds.index(self.feed) + 1) % len(feeds)]
            if nxt != self.feed or self.feed_t == 0.0:
                self.feed, self.feed_t = nxt, now
                self.send({"cmd": "url", "url": self.url(nxt)})
            else:
                self.feed_t = now
        self.state, self.why = ACTIVE, "UAV %d (%d open)" % (self.feed, len(feeds))

    def _handle(self, m, now):
        t = m.get("t")
        if t == "result":
            self.frames += 1
            self.fps, self.infer_ms = m.get("fps", 0.0), m.get("infer_ms")
            ft = m.get("frame_t") or now
            if self.feed is None or ft < self.feed_t + float(self.cfg["settle_s"]):
                return                                          # a frame of the feed before the switch
            self.result(self.feed, m.get("blobs") or [], now)
        elif t == "error":
            self.state, self.why = OFFLINE, m.get("text", "UAV AI worker error")
        elif t == "exit":
            pass

    def result(self, slot, blobs, now):
        """Boxes of one analysed frame of UAV slot's video (tests call it directly)."""
        c = self.cfg
        boxes = [[round(b["x0"], 3), round(b["y0"], 3), round(b["x1"], 3), round(b["y1"], 3), round(b["conf"], 2)]
                 for b in blobs if b.get("conf", 0.0) >= float(c["min_conf"])][:12]
        f = self.feeds.setdefault(slot, {"boxes": [], "t": 0.0, "frames": 0, "n_last": 0, "noticed": 0.0})
        f.update(boxes=boxes, t=now)
        f["frames"] += 1
        if boxes and not f["n_last"] and now - f["noticed"] >= float(c["notice_s"]):
            f["noticed"] = now
            self.mgr.notice("UAV %d AI: %d object(s) seen in its video (boxes only - the boat is not affected)" % (
                slot, len(boxes)))
        f["n_last"] = len(boxes)

    def status(self, now=None):
        now = time.time() if now is None else now
        keep = float(self.cfg["keep_s"])
        return {"state": self.state, "why": self.why, "feed": self.feed, "fps": round(self.fps or 0.0, 2),
                "infer_ms": self.infer_ms, "frames": self.frames, "starts": self.starts,
                "max_fps": float(self.cfg["max_fps"]), "open": self.open_feeds(),
                "feeds": {str(s): {"boxes": f["boxes"] if now - f["t"] <= keep else [], "age_s": round(now - f["t"], 1),
                                   "frames": f["frames"]} for s, f in self.feeds.items()},
                "note": "eWaSR is trained on water-level views - aerial results may be poor (W62)"}
