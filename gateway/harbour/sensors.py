"""The harbour sensor interface (CLAUDE.md Z, docs/harbour_design.md 5 + 5a, step HX3).

ONE interface for every radar / LiDAR, so the tracker (HX4) never knows the brand or whether it is simulated.

Frames (every one documented; the tracker reads only these):
  pose   {t, lat, lon, heading (deg true), sog (kn), cog (deg)}       the own boat at t (from the gateway's fresh
                                                                       AURIX telemetry; a sensor batch carries the
                                                                       pose the worker held at the measurement)
  RadarPlot   {range_m, brg_deg (relative to the bow, clockwise, 0..360), speed_mps | None, course_deg | None
               (true, a radar's own ARPA / MARPA if it gives one), size_m, size_cls "small" | "medium" | "large",
               strength 0..1}
  LidarObject {x, y, z (m, SENSOR frame: x ahead, y to STARBOARD, z up from the sensor), len_m, wid_m, hgt_m,
               n_points}  - after the worker's filter (own hull box, the water band, isolated spray < 3 m)
  batch  {ev "radar" | "lidar", seq, t (sensor time = the measurement, epoch s, gateway clock after the sync),
          t_tx (the worker's clock when it sent it), pose, items [...] (nearest first), bad (items refused by
          validation), capped (items over max_items: the farthest), sim (true = SIMULATED data)}
  status {ev "status", kind, driver, self "ok" | "degraded" | "offline", why, rate_hz, batches, t}

Health (the gateway's judgement, HealthJudge - never the worker's word alone):
  OFF       not configured
  OFFLINE   the driver cannot run (a real driver not chosen / not installed: W148)
  STARTING  the worker runs, no batch yet (start_s)
  OK
  DEGRADED  the worker says so, or the batch rate < half the nominal rate, or the time skew > skew_max_s (0.1 s),
            or the pose > pose_dt_max_s (0.5 s) from the measurement, or the delivery delay > delay_max_s (0.2 s),
            or more than bad_frac_max of the items refused
  (the manager judges every age on time.monotonic(): a GPS clock step never fakes an OK - HX3 review M1)
  LOST      no batch for lost_s, or the worker process gone / hung (restarted)
A change = a NOTICE (manager.py), never an alarm, never a mode change, never anything sent to the AURIX. The planner
(HX5) treats LOST / DEGRADED by its unsure list (design 7.4 U1 / U2).
"""
import math

OFF, OFFLINE, STARTING, OK, DEGRADED, LOST = "OFF", "OFFLINE", "STARTING", "OK", "DEGRADED", "LOST"
STATES = (OFF, OFFLINE, STARTING, OK, DEGRADED, LOST)
KINDS = ("radar", "lidar")
SIZE_CLS = ("small", "medium", "large")

# the drivers: "sim" = simulated from simworld.py; the real ones are SKELETONS ONLY (W148: the owner has not chosen;
# UNTESTED, never on hardware) - they report OFFLINE "not chosen" and produce nothing
DRIVERS = {"radar": ("sim", "mayara", "none"), "lidar": ("sim", "livox", "ouster", "none")}

DEFAULTS = {
    "radar": {"enabled": False, "driver": "sim", "rate_hz": 0.8, "lost_s": 4.0, "start_s": 8.0, "latency_s": 0.3,
              "skew_max_s": 0.1, "bad_frac_max": 0.2, "pose_dt_max_s": 0.5, "delay_max_s": 0.2, "min_range_m": 25.0, "max_range_m": 3000.0,
              "range_sd_m": 3.0, "range_sd_frac": 0.005, "brg_sd_deg": 1.0, "speed_sd_mps": 0.3,
              "drop_p": 0.02, "clutter_n": 1.0, "clutter_r_m": 250.0, "url": ""},
    "lidar": {"enabled": False, "driver": "sim", "rate_hz": 10.0, "lost_s": 1.0, "start_s": 8.0, "latency_s": 0.05,
              "skew_max_s": 0.1, "bad_frac_max": 0.2, "pose_dt_max_s": 0.5, "delay_max_s": 0.2, "range_10pct_m": 40.0, "range_80pct_m": 70.0,
              "min_range_m": 0.1, "mount_h_m": 2.5, "vfov_min_deg": -7.0, "vfov_max_deg": 52.0, "pos_sd_m": 0.05,
              "pos_sd_frac": 0.002, "drop_p": 0.01, "spray_n": 2.0, "hull_len_m": 7.0, "hull_wid_m": 2.6,
              "mount_x_m": 1.0, "water_band_m": 0.3, "min_points": 2, "cluster_m": 0.5, "addr": ""},
}
COMMON = {"enabled": False, "sim": False, "world": "", "python": "", "restart_s": 5.0, "silent_s": 6.0,
          "max_items": 400}


def merged(cfg):
    """The harbour config with every default filled in (cfg = gateway.json "harbour")."""
    cfg = cfg or {}
    out = dict(COMMON)
    for k, v in cfg.items():
        if k not in KINDS:
            out[k] = v
    for kind in KINDS:
        d = dict(DEFAULTS[kind])
        d.update(cfg.get(kind) or {})
        if cfg.get("sim") and "enabled" not in (cfg.get(kind) or {}):
            d["enabled"] = True                 # "sim": true alone = both simulated sensors
        if cfg.get("sim"):
            d["driver"] = "sim"
        out[kind] = d
    return out


def check_config(cfg):
    """[(level, text)] for gateway.py --check-config (osdep.check_config calls it). An error = that sensor can't work
    (the rest of the gateway runs)."""
    out = []
    if cfg is None:
        return out
    if not isinstance(cfg, dict):
        return [("error", "harbour: must be an object")]
    for kind in KINDS:
        c = cfg.get(kind)
        if c is None:
            continue
        if not isinstance(c, dict):
            out.append(("error", "harbour.%s: must be an object" % kind))
            continue
        drv = c.get("driver", "sim")
        if drv not in DRIVERS[kind]:
            out.append(("error", "harbour.%s.driver %r: one of %s" % (kind, drv, ", ".join(DRIVERS[kind]))))
        elif cfg.get("sim") and drv not in ("sim", "none"):
            out.append(("warning", "harbour.sim is true: the %s driver %s is REPLACED by the SIMULATOR - never on "
                                   "the boat" % (kind, drv)))
        elif c.get("enabled") and drv not in ("sim", "none"):
            out.append(("warning", "harbour.%s.driver %s: a SKELETON only - the sensor is not chosen yet (W148), "
                                   "it stays OFFLINE" % (kind, drv)))
        for k, v in c.items():
            if k.startswith("_") or k in ("enabled", "driver", "url", "addr"):
                continue
            if k not in DEFAULTS[kind]:
                out.append(("warning", "harbour.%s.%s: unknown key" % (kind, k)))
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
                out.append(("error", "harbour.%s.%s: %r must be a number >= 0" % (kind, k, v)))
        for k in ("rate_hz", "lost_s"):
            v = c.get(k, 1.0)
            if isinstance(v, (int, float)) and not isinstance(v, bool) and v <= 0:
                out.append(("error", "harbour.%s.%s must be > 0" % (kind, k)))
    w = cfg.get("world", "")
    if not isinstance(w, str):
        out.append(("error", "harbour.world: a file path or \"\" (the built-in harbour)"))
    return out


# --- validation (a NaN / out-of-range item is refused, never used) ---------------------------------------------------
def _num(v, lo, hi):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and lo <= v <= hi


def valid_pose(p):
    return (isinstance(p, dict) and _num(p.get("t"), 0, 1e11) and _num(p.get("lat"), -90, 90) and
            _num(p.get("lon"), -180, 180) and _num(p.get("heading"), 0, 360))


def valid_plot(p, max_range=20000.0):
    if not isinstance(p, dict):
        return False
    if not (_num(p.get("range_m"), 0, max_range) and _num(p.get("brg_deg"), 0, 360) and _num(p.get("size_m"), 0, 500)
            and _num(p.get("strength"), 0, 1) and p.get("size_cls") in SIZE_CLS):
        return False
    for k, hi in (("speed_mps", 60.0), ("course_deg", 360.0)):
        if p.get(k) is not None and not _num(p.get(k), 0, hi):
            return False
    return True


def valid_object(o, max_range=500.0):
    if not isinstance(o, dict):
        return False
    return (_num(o.get("x"), -max_range, max_range) and _num(o.get("y"), -max_range, max_range) and
            _num(o.get("z"), -50, 50) and _num(o.get("len_m"), 0, 500) and _num(o.get("wid_m"), 0, 500) and
            _num(o.get("hgt_m"), 0, 100) and isinstance(o.get("n_points"), int) and o["n_points"] >= 1)


def size_cls(m):
    return "small" if m < 3.0 else ("medium" if m < 12.0 else "large")


FLAT_M = 0.15            # a water-surface return is flatter than this


def lidar_filter(objs, c):
    """The worker's LiDAR clean-up (design 5): drop the own hull box, the water band (a FLAT object - height < 0.15 m
    - whose top is below water + water_band_m) and isolated spray (fewer than min_points points closer than 3 m). Sensor frame."""
    hl, hw, mx = float(c["hull_len_m"]), float(c["hull_wid_m"]), float(c["mount_x_m"])
    water = -float(c["mount_h_m"])
    band = float(c["water_band_m"])
    out = []
    for o in objs:
        bx = o["x"] + mx                               # boat frame: x from the hull's middle
        if abs(bx) <= hl / 2.0 + 0.2 and abs(o["y"]) <= hw / 2.0 + 0.2:
            continue                                   # own hull
        if o["z"] + o["hgt_m"] / 2.0 < water + band and o["hgt_m"] < FLAT_M:
            continue                                   # a FLAT return in the water band (+- 0.3 m): the surface
        # (HX3 review H1: a low object that is not flat - a swimmer's head 0.3 m - is KEPT; judging the top alone
        # dropped half of the swimmer detections at the noise edge)
        if o["n_points"] < int(c["min_points"]) and math.hypot(o["x"], o["y"]) < 3.0:
            continue                                   # spray
        out.append(o)
    return out


# --- the gateway's health judgement -----------------------------------------------------------------------------------
class HealthJudge:
    """One sensor's health from what the gateway RECEIVES (pure: every time is passed in)."""

    def __init__(self, kind, c, ok_hold_s=2.0):
        self.kind, self.c = kind, c
        self.ok_hold_s = ok_hold_s
        self.state, self.why = (OFF, "not configured") if not c.get("enabled") else (STARTING, "starting")
        self.spawn_t = None
        self.ever = False               # a worker ran once (before that: STARTING, never LOST)
        self.rx = []                    # receive times of the last batches (rate)
        self.last_rx = None
        self.skew = None
        self.pose_dt = None
        self.delay = None
        self.spawn_fail = ""             # HX3 review M2: why the worker cannot start
        self.bad_frac = 0.0
        self.self_state, self.self_why = "ok", ""
        self.bad_since = None
        self.good_since = None

    def spawned(self, now):
        self.spawn_t, self.ever = now, True
        self.state, self.why, self.good_since = STARTING, "starting", None
        self.rx, self.last_rx, self.skew = [], None, None
        self.self_state, self.self_why = "ok", ""

    def gone(self, now):
        self.spawn_t = None

    def batch(self, now, b):
        self.last_rx = now
        self.rx.append(now)
        self.rx = [t for t in self.rx if now - t <= 5.0]
        try:
            self.skew = (float(b["t_tx"]) - float(b["t"])) - float(self.c["latency_s"])
        except (KeyError, TypeError, ValueError):
            self.skew = None
        try:                            # HX3 review M3: the pose must belong to the measurement
            self.pose_dt = abs(float(b["t"]) - float(b["pose"]["t"]))
        except (KeyError, TypeError, ValueError):
            self.pose_dt = None
        try:                            # HX3 review L5: the delivery delay worker -> gateway (epoch clocks)
            self.delay = float(b["t_rx_epoch"]) - float(b["t_tx"])
        except (KeyError, TypeError, ValueError):
            self.delay = None
        n = len(b.get("items") or []) + int(b.get("bad") or 0)
        self.bad_frac = (int(b.get("bad") or 0) / float(n)) if n else 0.0

    def status_line(self, s):
        self.self_state, self.self_why = s.get("self", "ok"), s.get("why", "")

    def judge(self, now):
        c = self.c
        if not c.get("enabled"):
            return self._set(OFF, "not configured", now)
        if self.self_state == "offline":
            return self._set(OFFLINE, self.self_why or "driver not available", now)
        if self.spawn_t is None:
            if self.spawn_fail:
                return self._set(LOST, "worker cannot start: %s" % self.spawn_fail, now)
            if not self.ever:
                return self._set(STARTING, "starting", now)
            return self._set(LOST, "worker not running", now)
        if self.last_rx is None:
            if now - self.spawn_t <= float(c["start_s"]):
                return self._set(STARTING, "starting", now)
            return self._set(LOST, "no data since the start (%.0f s)" % (now - self.spawn_t), now)
        age = now - self.last_rx
        if age > float(c["lost_s"]):
            return self._set(LOST, "no data for %.1f s" % age, now)
        why = []
        if self.self_state == "degraded":
            why.append(self.self_why or "the sensor reports degraded")
        span = now - max(self.spawn_t, self.rx[0] if self.rx else now)
        if span >= 3.0 / float(c["rate_hz"]) and span >= 2.0:
            rate = (len(self.rx) - 1) / max(1e-3, self.rx[-1] - self.rx[0]) if len(self.rx) > 1 else 0.0
            if rate < 0.5 * float(c["rate_hz"]):
                why.append("rate %.2f Hz (nominal %.2f)" % (rate, float(c["rate_hz"])))
        if self.skew is not None and abs(self.skew) > float(c["skew_max_s"]):
            why.append("time skew %.0f ms" % (self.skew * 1000.0))
        if self.pose_dt is not None and self.pose_dt > float(c["pose_dt_max_s"]):
            why.append("pose %.0f ms from the measurement" % (self.pose_dt * 1000.0))
        if self.delay is not None and self.delay > float(c["delay_max_s"]):
            why.append("delivery delay %.0f ms" % (self.delay * 1000.0))
        if self.bad_frac > float(c["bad_frac_max"]):
            why.append("%.0f %% invalid items" % (self.bad_frac * 100.0))
        if why:
            self.good_since = None
            return self._set(DEGRADED, "; ".join(why), now)
        if self.state in (DEGRADED, LOST):
            if self.good_since is None:
                self.good_since = now
            if now - self.good_since < self.ok_hold_s:
                return self.state, self.why            # hysteresis: back to OK only after 2 s good
        return self._set(OK, "", now)

    def _set(self, st, why, now):
        self.state, self.why = st, why
        if st != OK and st != DEGRADED:
            self.good_since = None
        return st, why


# --- the real drivers: SKELETONS (W148 not chosen - UNTESTED, never run on hardware) ---------------------------------
class RealDriverSkeleton:
    """A real sensor driver's place. The owner has NOT chosen the radar / LiDAR (PROGRESS.md W148), so no driver is
    written: open() reports OFFLINE with the reason; read() yields nothing. When the owner chooses, the driver fills
    read() with the vendor's data in the frames above (design 5a: Mayara / Signal K Radar API spokes or ARPA plots for
    the Navico HALO20+; Livox-SDK2 (MIT) / ouster-sdk (BSD-3) point clouds -> clusters -> lidar_filter)."""
    NOTE = {"mayara": "radar Navico HALO20+ via Mayara / Signal K Radar API (recommended, NOT chosen: W148)",
            "livox": "LiDAR Livox Mid-360 via Livox-SDK2 (recommended, NOT chosen: W148)",
            "ouster": "LiDAR Ouster OS1 via ouster-sdk (alternative, NOT chosen: W148)",
            "none": "no sensor configured"}

    def __init__(self, kind, driver, c):
        self.kind, self.driver, self.c = kind, driver, c

    def open(self):
        return False, "driver %s: skeleton only, untested - %s" % (self.driver, self.NOTE.get(self.driver, "?"))

    def read(self, now, pose):
        return None
