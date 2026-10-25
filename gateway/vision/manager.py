"""Obstacle detection on the gateway: the worker process, its health, the
decisions (Part 2). The worker (worker.py) finds obstacle blobs in the front
camera; here they get a bearing / rough distance (geometry.py), must be seen
in several frames in a row (persistence), and act by STAGE:

  STAGE 1 (default ON)   overlay on the screens + logging only
  Threats = confirmed obstacles in the path corridor, only in a driving mode
  (MANUAL / HDG LOCK / AUTO / REMOTE): moored in STANDBY nothing alarms.
  STAGE 2 (default ON)   a confirmed obstacle in the path corridor -> alarm
                         OBSTACLE AHEAD (an advisory on the AURIX: no effect
                         on the boat) -> the AURIX asks for a video SAVE
  STAGE 3 (default OFF)  AUTO: speed cap (slow_kn, 0 = idle when nearer than
                         stop_m) through the navigation request; HEADING LOCK:
                         a slow-down PROPOSAL on the driver screen; MANUAL /
                         REMOTE: alarm only. Switching it ON needs the
                         driver screen's ACCEPT (stations.ask_local).
Vision NEVER steers and NEVER asks for more throttle: it only ever adds a
speed cap (the AURIX applies a cap as a reduction only, in AUTO only).

Health (shown, logged, a notice - never an alarm, never a mode change):
  ACTIVE     results at >= min_fps
  PAUSED     the gateway's main jobs need the CPU / the Pi is hot / throttling
  DEGRADED   below min_fps, or the camera gives no new frames
  OFFLINE    switched off, no model / camera, the worker crashed or is silent
An active STAGE 3 cap is HELD while vision is not ACTIVE, until the driver
clears it ("vision_clear") - a slowdown is never released silently.
"""
import collections
import json
import logging
import os
import random
import subprocess
import threading
import time

from . import geometry as geo

log = logging.getLogger("gateway")
HERE = os.path.dirname(os.path.abspath(__file__))
ACTIVE, PAUSED, DEGRADED, OFFLINE = "ACTIVE", "PAUSED", "DEGRADED", "OFFLINE"
MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_REMOTE = 1, 2, 3, 5
NAV_CAP = 1
ALM_OBSTACLE, WHY_VISION = 8, 8

DEFAULTS = {
    "enabled": True,
    "source": "camera",             # "camera" (the worker on the front camera) or "fake" (tests / demo: no model)
    "python": None,                 # None = gateway/toolpaths.vision_python()
    "model": None,                  # None = gateway/vision/models/ewasr_resnet18.onnx
    "threads": 2, "cores": None,    # None = the last <threads> cores
    "max_fps": 2.0, "min_fps": 1.0, "warmup_s": 20.0, "offline_s": 10.0,
    "stage1": True, "stage2": True, "stage3": False,
    "persist_frames": 3,            # seen in this many processed frames in a row -> confirmed
    "drop_frames": 2,               # missed this many -> forgotten
    "match_deg": 6.0,               # same obstacle: bearing within this
    "bottom_ignore": 0.15, "min_blob_px": 6, "min_conf": 0.5, "max_width": 0.6,
    "camera": dict(geo.DEFAULT_CAMERA),
    "corridor": {"half_width_m": 25.0, "look_s": 60.0, "min_len_m": 150.0},
    "max_range_m": 600.0,
    "slow_kn": 3.0, "stop_m": 40.0, "clear_s": 10.0,
    "pause": {"loop_lag_ms": 150.0, "cpu_pct": 90.0, "temp_c": 78.0, "resume_s": 10.0},
    "samples": {"enabled": False, "on_detection": True, "random_per_hour": 4.0, "max_mb": 500.0},
    "log_detections": True,
    "allow_inject": False,          # synthetic detections from the sim / demo / tests
}


def deep(d, k, default=None):
    return d.get(k, default) if isinstance(d, dict) else default


class SysLoad:
    """CPU busy % (whole machine), the Pi's temperature and throttling."""

    def __init__(self):
        self.prev = None
        self.next_thr = 0.0
        self.throttled = 0

    def cpu_pct(self):
        try:
            if os.name == "nt":
                import ctypes
                idle, kern, user = (ctypes.c_ulonglong(), ctypes.c_ulonglong(), ctypes.c_ulonglong())
                ctypes.windll.kernel32.GetSystemTimes(ctypes.byref(idle), ctypes.byref(kern), ctypes.byref(user))
                t = (idle.value, kern.value + user.value)
            else:
                with open("/proc/stat") as f:
                    v = [int(x) for x in f.readline().split()[1:]]
                t = (v[3] + v[4], sum(v))
        except (OSError, ValueError, AttributeError):
            return None
        p, self.prev = self.prev, t
        if not p or t[1] == p[1]:
            return None
        return 100.0 * (1.0 - (t[0] - p[0]) / float(t[1] - p[1]))

    def temp_c(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                return int(f.read()) / 1000.0
        except (OSError, ValueError):
            return None

    def throttle_bits(self):
        """vcgencmd get_throttled (Pi): b0 under-voltage, b1 freq capped, b2 throttled, b3 soft temp limit."""
        if time.time() >= self.next_thr:
            self.next_thr = time.time() + 10.0
            try:
                out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
                self.throttled = int(out.strip().split("=")[1], 16) & 0xF
            except (OSError, ValueError, IndexError, subprocess.SubprocessError):
                self.throttled = 0
        return self.throttled


class Track:
    def __init__(self, tid, ob, now):
        self.id, self.t0 = tid, now
        self.seen, self.missed, self.confirmed = 1, 0, False
        self.update(ob, now)

    def update(self, ob, now):
        self.rel, self.dist, self.rng = ob["rel"], ob["dist"], ob["rng"]
        self.conf, self.box, self.last = ob["conf"], ob["box"], now
        self.synthetic = ob.get("synthetic", False)


class VisionManager:
    def __init__(self, cfg, gw):
        from common import deep_merge
        self.cfg = deep_merge(DEFAULTS, cfg or {})
        self.cfg["camera"], bad = geo.check_camera(self.cfg.get("camera"))
        for b in bad:
            log.warning("vision camera config not usable: %s", b)
        # the ground stations' AR overlays (driver HUD) use the calibration only when it is really set here
        self.cam_calibrated = bool((cfg or {}).get("camera")) and not bad
        self.gw = gw
        self.state, self.reason = OFFLINE, "starting"
        self.proc, self.reader = None, None
        self.inbox = collections.deque(maxlen=200)
        self.hello = {}
        self.fps, self.infer_ms, self.frames, self.last_result = 0.0, None, 0, 0.0
        self.started = 0.0
        self.restarts, self.next_start, self.backoff = 0, 0.0, 5.0
        self.url = None
        self.paused, self.pause_why, self.ok_since = False, "", 0.0
        self.load = SysLoad()
        self.lag_ms, self.last_tick = 0.0, None
        self.tracks, self.next_tid = [], 1
        self.obstacles, self.threats = [], []
        self.mask, self.horizon_src = None, "none"
        self.cap_kn, self.cap_why, self.cap_held, self.clear_since = None, "", False, None
        self.proposal, self.prop_id = None, 0
        self.inject_list = []
        self.fake_next = 0.0
        self.notices = collections.deque(maxlen=10)
        self.counts = collections.Counter()
        self.next_random_sample = time.time() + self._sample_gap()
        self.forced = None                  # tests: a simulated failure (model crash / overload)

    # --- settings -----------------------------------------------------------------------
    def stages(self):
        c = self.cfg
        return {"stage1": bool(c["stage1"]), "stage2": bool(c["stage2"]), "stage3": bool(c["stage3"])}

    def set_stage(self, name, on, who):
        if name not in ("stage1", "stage2", "stage3"):
            return False, "unknown stage"
        self.cfg[name] = bool(on)
        self.gw.audit("vision_stage", stage=name, on=bool(on), by=who)
        log.warning("vision %s %s (%s)", name.upper(), "ON" if on else "off", who)
        if name == "stage3" and not on and self.cap_kn is not None:
            self._release("STAGE 3 switched off by %s" % who)
        return True, "%s %s" % (name.upper(), "ON" if on else "off")

    # --- the worker process ------------------------------------------------------------
    def _python(self):
        if self.cfg.get("python"):
            return self.cfg["python"]
        import toolpaths
        return toolpaths.vision_python()

    def _model(self):
        return self.cfg.get("model") or os.path.join(HERE, "models", "ewasr_resnet18.onnx")

    def _front_url(self):
        cams = getattr(self.gw, "cams", None)
        if not cams or not self.gw.cfg["cameras"].get("enabled", True):
            return None
        from cameras import path_name
        pick = None
        for role in ("front_main", "front_backup"):
            for c in cams.cams.values():
                if c.role == role and c.state == "OK":
                    pick = pick or c
        if not pick:
            return None
        return "rtsp://127.0.0.1:%d/%s" % (self.gw.cfg["cameras"].get("rtsp_port", 8554), path_name(pick.name, "LOW"))

    def _start(self, now):
        c = self.cfg
        py, model = self._python(), self._model()
        if not py:
            return self._set(OFFLINE, "no Python with onnxruntime (tools/get_vision_python.py / pip)")
        if not os.path.exists(model):
            return self._set(OFFLINE, "no model file (tools/get_vision_model.py)")
        import toolpaths
        n = os.cpu_count() or 4
        cores = c["cores"] if c["cores"] is not None else list(range(max(0, n - c["threads"]), n))
        args = [py, os.path.join(HERE, "worker.py"), "--model", model, "--threads", str(c["threads"]),
                "--cores", ",".join(str(x) for x in cores), "--max-fps", str(c["max_fps"]),
                "--ffmpeg", toolpaths.ffmpeg() or "ffmpeg", "--bottom-ignore", str(c["bottom_ignore"]),
                "--min-blob-px", str(c["min_blob_px"])]
        if self.url:
            args += ["--url", self.url]
        try:
            errlog = open(os.path.join(self._dir(), "worker.log"), "w") if self.gw.storage.usable else \
                subprocess.DEVNULL
            self.proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=errlog,
                                         text=True, bufsize=1, cwd=HERE)
        except OSError as e:
            return self._set(OFFLINE, "worker did not start: %s" % e)
        self.started, self.last_result, self.fps = now, now, 0.0
        self.reader = threading.Thread(target=self._read, args=(self.proc,), daemon=True)
        self.reader.start()
        self.restarts += 1
        log.info("vision worker started (pid %d, %d threads on cores %s)", self.proc.pid, c["threads"], cores)

    def _read(self, proc):
        for line in proc.stdout:
            try:
                self.inbox.append(json.loads(line))
            except ValueError:
                pass
        self.inbox.append({"t": "exit"})

    def _send(self, obj):
        try:
            if self.proc and self.proc.poll() is None:
                self.proc.stdin.write(json.dumps(obj) + "\n")
                self.proc.stdin.flush()
        except (OSError, ValueError):
            pass

    def stop(self):
        if self.proc:
            self._send({"cmd": "quit"})
            try:
                self.proc.wait(2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None

    def fail(self, why):
        """Tests: simulate a failure ("crash: ..." -> OFFLINE, anything else -> DEGRADED); None = recovered."""
        self.forced = why

    def kill_worker(self):
        """Tests: the worker crashes."""
        if self.proc:
            self.proc.kill()

    # --- state / notices ---------------------------------------------------------------------
    def _set(self, state, reason):
        if state != self.state or reason != self.reason:
            if state != self.state:
                self.gw.audit("vision_state", state=state, reason=reason)
                (log.warning if state in (OFFLINE, DEGRADED) else log.info)("vision %s (%s)", state, reason)
                self.notices.append({"utc": time.time(), "text": "VISION %s: %s" % (state, reason)})
            self.state, self.reason = state, reason

    def _dir(self, *sub):
        try:
            return self.gw.storage.folder("vision", *sub)
        except OSError:
            return os.path.join(HERE, "..", "gw_data")

    # --- resource guard ---------------------------------------------------------------------
    def _guard(self, now):
        p = self.cfg["pause"]
        why = []
        if self.lag_ms > p["loop_lag_ms"]:
            why.append("gateway busy (loop %d ms late)" % self.lag_ms)
        cpu = self.load.cpu_pct()
        if cpu is not None and cpu > p["cpu_pct"]:
            why.append("CPU %d %%" % cpu)
        t = self.load.temp_c()
        if t is not None and t > p["temp_c"]:
            why.append("Pi %.0f C" % t)
        if os.name != "nt" and self.load.throttle_bits() & 0xE:
            why.append("Pi throttling")
        if why:
            self.ok_since = 0.0
            if not self.paused:
                self.paused, self.pause_why = True, ", ".join(why)
                self._send({"cmd": "pause"})
            self.pause_why = ", ".join(why)
        elif self.paused:
            self.ok_since = self.ok_since or now
            if now - self.ok_since >= p["resume_s"]:
                self.paused = False
                self._send({"cmd": "resume"})

    # --- the tick (event loop, 4 Hz) -----------------------------------------------------
    def tick(self, now=None, lag_ms=0.0):
        now = now or time.time()
        self.lag_ms = lag_ms
        c = self.cfg
        if not c["enabled"]:
            self.stop()
            self._set(OFFLINE, "switched off")
            self._decide(now)
            return
        if self.forced:
            self._set(OFFLINE if self.forced.startswith("crash") else DEGRADED, self.forced)
            self.fps = 0.0
        elif c["source"] == "fake":
            self._fake_frame(now)
        else:
            url = self._front_url()
            if url != self.url:
                self.url = url
                self._send({"cmd": "url", "url": url})
            alive = self.proc is not None and self.proc.poll() is None
            if not alive:
                if self.proc is not None:                       # it was running: crashed
                    self.proc = None
                    self.counts["crashes"] += 1
                    self._set(OFFLINE, "worker stopped (restart in %.0f s)" % self.backoff)
                    self.next_start = now + self.backoff
                    self.backoff = min(60.0, self.backoff * 2.0)
                elif now >= self.next_start:
                    self._start(now)
                    if self.proc is None:
                        self.next_start = now + 30.0
            self._guard(now)
            while self.inbox:
                self._handle(self.inbox.popleft(), now)
            if self.proc is not None:
                self._health(now)
        self._decide(now)

    def _health(self, now):
        c = self.cfg
        if self.url is None:
            return self._set(OFFLINE, "no front camera")
        if self.paused:
            return self._set(PAUSED, self.pause_why)
        if now - self.last_result > c["offline_s"] and now - self.started > c["offline_s"]:
            return self._set(OFFLINE if not self.frames_seen() else DEGRADED,
                             "no results for %.0f s (camera frozen / no frames?)" % (now - self.last_result))
        if now - self.started < c["warmup_s"]:
            return self._set(ACTIVE if self.fps >= c["min_fps"] else self.state, "starting")
        if self.fps < c["min_fps"]:
            return self._set(DEGRADED, "VISION DEGRADED: %.1f fps (minimum %.1f)" % (self.fps, c["min_fps"]))
        self._set(ACTIVE, "%.1f fps" % self.fps)

    def frames_seen(self):
        return self.frames > 0

    def _handle(self, m, now):
        t = m.get("t")
        if t == "hello":
            self.hello = m
            self.backoff = 5.0
        elif t == "result":
            self.frames += 1
            self.last_result = now
            self.fps, self.infer_ms = m.get("fps", 0.0), m.get("infer_ms")
            self.mask = {"w": m.get("mask_w"), "h": m.get("mask_h"), "b64": m.get("mask")}
            self._frame(now, m.get("blobs") or [], m.get("horizon"))
        elif t == "status":
            if m.get("state") == "NO_FRAMES" and not self.paused:
                self._set(DEGRADED, "front camera gives no frames (frozen?)")
                self.fps = 0.0
        elif t == "error":
            self._set(OFFLINE, m.get("text", "worker error"))
            self.counts["errors"] += 1
        elif t == "exit":
            pass

    # --- frames -> obstacles -> tracks ---------------------------------------------------------
    def _horizon(self, img_horizon):
        cam = self.cfg["camera"]
        t2 = getattr(self.gw, "telem2", {}) or {}
        if t2.get("att_roll") is not None and t2.get("att_pitch") is not None:
            self.horizon_src = "IMU"
            return geo.horizon_from_imu(cam, t2["att_roll"], t2["att_pitch"])
        if img_horizon:
            self.horizon_src = "picture"
            return tuple(img_horizon)
        self.horizon_src = "camera tilt only"
        return geo.horizon_default(cam)

    def _frame(self, now, blobs, img_horizon, synthetic=()):
        c = self.cfg
        hz = self._horizon(img_horizon)
        obs = []
        for b in blobs:
            if b.get("conf", 0.0) < c["min_conf"] or b.get("width", 0.0) > c["max_width"]:
                continue                                    # weak, or the shore across the picture
            rel, d, rng = geo.locate(c["camera"], b["u"], b["v"], hz, max_range_m=c["max_range_m"])
            obs.append({"rel": rel, "dist": d, "rng": rng, "conf": b["conf"],
                        "box": [b["x0"], b["y0"], b["x1"], b["y1"]]})
        obs += list(synthetic)
        self._track(obs, now)

    def _track(self, obs, now):
        c = self.cfg
        used = set()
        for tr in self.tracks:
            best, bd = None, c["match_deg"]
            for i, ob in enumerate(obs):
                if i in used:
                    continue
                dd = abs(geo.angle_diff(ob["rel"], tr.rel))
                # the same obstacle: similar bearing and (when both have one) a similar rough distance
                ratio_ok = ob["dist"] is None or tr.dist is None or 0.5 <= ob["dist"] / max(tr.dist, 1.0) <= 2.0
                if dd <= bd and ratio_ok:
                    best, bd = i, dd
            if best is None:
                tr.missed += 1
                if not tr.confirmed:
                    tr.seen = 0
            else:
                used.add(best)
                tr.update(obs[best], now)
                tr.seen += 1
                tr.missed = 0
                if not tr.confirmed and tr.seen >= c["persist_frames"]:
                    tr.confirmed = True
                    self.counts["confirmed"] += 1
                    self._log_det(tr, "confirmed", now)
                    self._sample("det_%d" % tr.id)
        before = len(self.tracks)
        self.tracks = [tr for tr in self.tracks if tr.missed <= c["drop_frames"] and
                       (tr.confirmed or tr.seen > 0)]
        self.counts["dropped"] += before - len(self.tracks)
        for i, ob in enumerate(obs):
            if i not in used:
                self.tracks.append(Track(self.next_tid, ob, now))
                self.next_tid += 1
                self.counts["candidates"] += 1
        for tr in self.tracks:
            if tr.confirmed and now - getattr(tr, "logged", 0.0) >= 5.0:
                self._log_det(tr, "seen", now)
        if self.cfg["samples"]["enabled"] and now >= self.next_random_sample:
            self.next_random_sample = now + self._sample_gap()
            self._sample("random")

    # --- injection (sim / demo / tests): obstacles in the world, not in the picture ----------------
    def inject(self, rel_deg, dist_m, seconds=10.0, conf=0.9, frames=None):
        if not self.cfg["allow_inject"]:
            return False
        self.inject_list.append({"rel": float(rel_deg), "dist": float(dist_m), "conf": float(conf),
                                 "until": time.time() + seconds, "frames": frames})
        self.gw.audit("vision_inject", rel=rel_deg, dist=dist_m, seconds=seconds, frames=frames)
        return True

    def _synthetic(self, now):
        out = []
        for s in list(self.inject_list):
            if s["frames"] is not None:
                s["frames"] -= 1
            if now > s["until"] or (s["frames"] is not None and s["frames"] < 0):
                self.inject_list.remove(s)
                continue
            d = s["dist"]
            out.append({"rel": s["rel"], "dist": d, "rng": (d * 0.7, d * 1.5), "conf": s["conf"], "box": None,
                        "synthetic": True})
        return out

    def _fake_frame(self, now):
        """source "fake": frames at max_fps without a camera / model, only injected obstacles."""
        if now < self.fake_next:
            return
        self.fake_next = now + 1.0 / max(0.2, self.cfg["max_fps"])
        if self.paused:
            return self._set(PAUSED, self.pause_why)
        self.frames += 1
        self.last_result, self.fps = now, self.cfg["max_fps"]
        self._frame(now, [], None, synthetic=self._synthetic(now))
        self._set(ACTIVE, "fake source (synthetic detections only)")

    # --- decisions ---------------------------------------------------------------------------------
    def _own(self):
        d = getattr(self.gw, "telem", None) or {}
        return d, d.get("mode")

    def _decide(self, now):
        d, mode = self._own()
        c = self.cfg
        if c["source"] != "fake" and self.inject_list and now >= self.fake_next and self.state != OFFLINE:
            self.fake_next = now + 1.0 / max(0.2, c["max_fps"])
            self._track(self._synthetic(now), now)          # the sim's kayak, on top of the camera
        self.obstacles = []
        driving = mode in (MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_REMOTE)    # moored in STANDBY: no threats
        for tr in self.tracks:
            ob = {"id": tr.id, "rel": round(tr.rel, 1), "bearing": round(geo.bearing_abs(d.get("heading"), tr.rel), 1),
                  "dist": None if tr.dist is None else round(tr.dist), "range": [None if x is None else round(x)
                                                                            for x in tr.rng],
                  "conf": tr.conf, "confirmed": tr.confirmed, "box": tr.box, "synthetic": tr.synthetic,
                  "in_corridor": driving and tr.confirmed and geo.in_corridor(tr.rel, tr.dist, d.get("est_sog"),
                                                                  d.get("yaw_rate"), c["corridor"])}
            ob["ais"] = self._ais_link(ob, d)
            self.obstacles.append(ob)
        self.threats = [o for o in self.obstacles if o["in_corridor"]]
        # STAGE 3 cap: set by a threat in AUTO; held while vision is not healthy; released when clear
        if c["stage3"] and mode == MODE_AUTO and self.threats and self.state in (ACTIVE, PAUSED, DEGRADED):
            near = min((o["dist"] for o in self.threats if o["dist"] is not None), default=None)
            cap = 0.0 if near is not None and near < c["stop_m"] else c["slow_kn"]
            if self.cap_kn is None or cap < self.cap_kn:
                self.gw.audit("vision_cap", cap_kn=cap, dist=near, why="obstacle in the path corridor")
                log.warning("vision: speed cap %.1f kn (obstacle %s m ahead)", cap, near)
            self.cap_kn = cap if self.cap_kn is None else min(self.cap_kn, cap)
            self.cap_why, self.cap_held, self.clear_since = "obstacle ahead", False, None
        elif self.cap_kn is not None:
            if mode != MODE_AUTO:
                self._release("left AUTO (the driver took over)")
            elif self.state != ACTIVE:
                if not self.cap_held:
                    self.cap_held = True
                    self.gw.audit("vision_cap_held", state=self.state, reason=self.reason)
                    log.warning("vision %s: speed cap HELD until the driver clears it", self.state)
            elif not self.cap_held:
                self.clear_since = self.clear_since or now
                if now - self.clear_since >= c["clear_s"]:
                    self._release("clear for %.0f s" % c["clear_s"])
        # HEADING LOCK + STAGE 3: a slow-down proposal for the driver (the throttle stays manual)
        if c["stage3"] and mode == MODE_HDG_LOCK and self.threats:
            if self.proposal is None:
                o = self.threats[0]
                self.prop_id += 1
                self.proposal = {"id": self.prop_id, "text": "OBSTACLE AHEAD %s (rough), bearing %03.0f: "
                                 "slow down" % ("~%d m" % o["dist"] if o["dist"] else "far", o["bearing"])}
                self.gw.audit("vision_proposal", text=self.proposal["text"])
        elif self.proposal is not None and not self.threats:
            self.proposal = None

    def _release(self, why):
        self.gw.audit("vision_cap_released", cap_kn=self.cap_kn, why=why)
        log.warning("vision: speed cap released (%s)", why)
        self.cap_kn, self.cap_held, self.clear_since = None, False, None

    def clear(self, who):
        """The driver clears a held slowdown (driver screen / single-operator only)."""
        if self.cap_kn is None:
            return False, "no vision slowdown active"
        self._release("cleared by %s" % who)
        return True, "vision slowdown cleared"

    def ack_proposal(self, pid, who):
        if self.proposal and self.proposal["id"] == pid:
            self.gw.audit("vision_proposal_ack", id=pid, by=who)
            self.proposal = dict(self.proposal, acked=True)
            return True
        return False

    def nav(self, mode):
        """This module's part of the navigation request: (cap_kn or None, alarm bits, reason bits).
        Never a heading. Never more speed (a cap is only ever a reduction on the AURIX)."""
        c = self.cfg
        alarm = ALM_OBSTACLE if self.threats and (c["stage2"] or c["stage3"]) else 0
        cap = self.cap_kn if (c["stage3"] and mode == MODE_AUTO) else None
        if cap is not None and self.cap_held:
            alarm |= ALM_OBSTACLE
        return cap, alarm, (WHY_VISION if (cap is not None or alarm) else 0)

    def _ais_link(self, ob, d):
        """A vision obstacle that matches an AIS target (bearing within 8 deg, distance within x2)."""
        if ob["dist"] is None:
            return None
        av = getattr(self.gw, "nav_state", {}) or {}
        for t in av.get("threats") or []:
            if abs(geo.angle_diff(t.get("bearing", 0.0), ob["bearing"])) <= 8.0:
                dm = (t.get("dist_nm") or 0.0) * 1852.0
                if dm and 0.5 <= ob["dist"] / dm <= 2.0:
                    return t.get("name")
        return None

    # --- logging + samples ---------------------------------------------------------------------
    def _log_det(self, tr, what, now):
        tr.logged = now
        if not self.cfg["log_detections"] or not self.gw.storage.usable:
            return
        d, mode = self._own()
        c = self.cfg
        action = "stage1 log"
        if tr.confirmed and geo.in_corridor(tr.rel, tr.dist, d.get("est_sog"), d.get("yaw_rate"), c["corridor"]):
            action = "stage3 cap" if (c["stage3"] and mode == MODE_AUTO) else \
                "stage3 proposal" if (c["stage3"] and mode == MODE_HDG_LOCK) else \
                "stage2 alarm" if (c["stage2"] or c["stage3"]) else "stage1 log"
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "what": what, "id": tr.id,
               "bearing": round(geo.bearing_abs(d.get("heading"), tr.rel), 1), "rel": round(tr.rel, 1),
               "dist_m": None if tr.dist is None else round(tr.dist), "range_m": tr.rng, "conf": tr.conf,
               "synthetic": tr.synthetic, "action": action, "mode": mode, "state": self.state}
        try:
            fn = os.path.join(self._dir(), "detections-%s.jsonl" % time.strftime("%Y%m%d", time.gmtime(now)))
            with open(fn, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError:
            pass

    def _sample_gap(self):
        per_h = max(0.1, self.cfg["samples"]["random_per_hour"])
        return random.uniform(0.5, 1.5) * 3600.0 / per_h

    def _sample(self, kind):
        s = self.cfg["samples"]
        if not s["enabled"] or self.proc is None or self.gw.storage.state not in ("ok",):
            return                                          # storage low / full: no samples
        if kind != "random" and not s["on_detection"]:
            return
        folder = self._dir("samples")
        self._quota(folder, s["max_mb"] * 1e6)
        self._send({"cmd": "sample", "path": os.path.join(folder, "%s_%s" % (
            time.strftime("%Y%m%d-%H%M%S", time.gmtime()), kind))})
        self.counts["samples"] += 1

    @staticmethod
    def _quota(folder, max_b):
        files = sorted((os.path.getmtime(os.path.join(folder, f)), os.path.join(folder, f)) for f in os.listdir(folder))
        total = sum(os.path.getsize(p) for _t, p in files)
        for _t, p in files:
            if total <= max_b:
                break
            total -= os.path.getsize(p)
            os.remove(p)

    # --- status for the screens -----------------------------------------------------------------
    def status(self, saver=False):
        st = {"state": self.state, "reason": self.reason, "fps": round(self.fps, 2), "infer_ms": self.infer_ms,
              "stages": self.stages(), "horizon": self.horizon_src, "obstacles": self.obstacles[:12],
              "threats": len(self.threats), "cap_kn": self.cap_kn, "cap_held": self.cap_held,
              "proposal": self.proposal, "notices": list(self.notices)[-3:], "source": self.cfg["source"],
              "restarts": self.restarts, "counts": dict(self.counts), "paused": self.paused,
              "model": self.hello.get("model"), "threads": self.cfg["threads"],
              "camera": dict(self.cfg["camera"]) if self.cam_calibrated else None}
        if self.mask and not saver:
            st["mask"] = self.mask
        return st
