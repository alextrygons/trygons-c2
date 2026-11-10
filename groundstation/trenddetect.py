"""The silent highlight «ΤΑΣΗ» of the trend panel (step TR3, CLAUDE.md Y, docs/trends_design.md 5 + 6 + 7): PURE - no
tkinter, no network, no import of anything that can send or sound. Every judged value is compared with its own recent
normal (the last 10 min) and the last 30 s: a clear change of the median (mean) or of the spread (rhythm /
oscillation) is a TREND. COMMANDED changes (a mode / waypoint / throttle / speed-cap / gear / link change ...) REBASE
the values they explain instead of highlighting.

SILENT and READ-ONLY: the detector only calls a list of dicts per highlight on / off (take(); the station
logs them); it never raises an alarm, plays a sound, lights a lamp or sends anything (test_trends TH7 / TG6 - TG8).

    det = Detector()
    det.observe(now, d, link_key)      # every telemetry frame: the commanded-change filter (cheap)
    det.poll(model)                    # after feed / tick: judges the new 2 Hz samples of trends.Trends
    det.active()                       # {value id: {"kind": "mean" | "spread", "dir": "up" | "down", ...}}

Not judged at all (display only): thr (commanded), hdg (commanded / steered), depth (position-driven).
"""
import math
from collections import deque

BASE_S = 630.0              # BASE = [t - 630 s, t - 30 s]
RECENT_S = 30.0
BASE_MIN = 240              # valid samples (2 min at 2 Hz)
RECENT_MIN = 45
K_MEAN = 4.0
SPREAD_UP = 2.5
SPREAD_DOWN = 0.4
SPREAD_DOWN_BASE = 2.0      # the spread-down test only when s_B >= 2 x s_min
ON_S = 10.0                 # the condition must hold this long
OFF_RATIO = 0.6             # hysteresis: off when the statistic stays below 0.6 x its threshold ...
OFF_S = 20.0                # ... for this long
MIN_SHOWN_S = 30.0
COOLDOWN_S = 300.0
BASE_EVERY_S = 5.0
GAP_S = 5.0                 # F9
MAD_K = 1.4826
MAX_BASE = 1300
STAGGER_S = 0.5

ALL_BUT = ("STANDBY", "DP", "BENCH_TEST")
DRIVEN = ("AUTO", "HDG_LOCK")
# id: (s_min, abs_min, mean directions, spread directions, judged modes (None = every mode but ALL_BUT for slamming))
CFG = {
    "sog": (0.2, 1.0, "ud", "ud", DRIVEN),
    "rot": (0.5, None, "", "ud", DRIVEN),
    "xte": (2.0, 10.0, "ud", "u", ("AUTO", "DP")),
    "roll": (0.5, 3.0, "ud", "ud", DRIVEN),
    "pitch": (0.3, 1.5, "ud", "ud", DRIVEN),
    "slam_g": (0.1, 0.3, "ud", "", ALL_BUT),
    "slam_pm": (1.0, 3.0, "ud", "", ALL_BUT),
    "bank_v": (0.05, 0.3, "ud", "", None),
    "link_ms": (20.0, 150.0, "ud", "u", None),
    "link_loss": (1.0, 5.0, "u", "", None),
}
NOT_JUDGED = ("thr", "hdg", "depth")
FIRST_TEST_OFF = ("sog", "rot", "xte", "roll", "pitch")
DP_TRIP = 4

# the commanded-change filter (design 6): trigger -> (values rebased, settle seconds)
F1 = (("sog", "rot", "xte", "roll", "pitch", "slam_g", "slam_pm"), 20.0)
F2 = (("sog", "pitch", "slam_g", "slam_pm"), 20.0)
F3 = (("rot", "xte", "roll", "sog"), 30.0)
F4 = (("sog", "pitch", "slam_g", "slam_pm"), 20.0)
F5 = (("sog", "rot", "roll", "pitch", "slam_g", "slam_pm"), 20.0)
F6 = (("rot", "xte", "roll"), 30.0)
F7 = (("bank_v",), 60.0)
F8 = (("link_ms", "link_loss"), 30.0)
THR_STEP = 5.0              # % from the mean of the previous 10 s
THR_RATE = 2.0              # %/s
TARGET_STEP = 5.0           # deg within 1 s


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def median(a):
    n = len(a)
    if not n:
        return float("nan")
    s = sorted(a)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def robust(a):
    """(median, 1.4826 x MAD)."""
    m = median(a)
    return m, MAD_K * median([abs(x - m) for x in a])


def angdiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


class Detector:
    def __init__(self):
        self.out = []                               # the highlight on / off events not yet taken by the station
        self.reset()

    def take(self):
        """The events since the last call (the station logs them): the detector calls NOTHING, it holds no callback."""
        out, self.out = self.out, []
        return out

    def reset(self):
        """F10: a boat restart / boat change / disconnect (the rings were cleared)."""
        self.order = {v: i for i, v in enumerate(CFG)}
        self.buf = {v: deque() for v in CFG}
        self.until = {v: 0.0 for v in CFG}              # samples before this time are not used (settle)
        self.last_valid = {v: None for v in CFG}
        self.base = {v: None for v in CFG}              # (computed at, m, s, n)
        self.pend = {}                                  # value -> (kind, dir, first t)
        self.hl = {}                                    # value -> the active highlight
        self.cool = {}                                  # (value, kind, dir) -> time the cooldown ends
        self.mode, self.dp_sub, self.first_test = "", None, False
        self.prev = None                                # the last frame's trigger keys
        self.thr_hist, self.dem_hist, self.tgt_hist = deque(), deque(), deque()
        self.done = 0
        self.epoch = None
        self.n_events = 0

    def active(self):
        return self.hl

    # --- the commanded-change filter ---------------------------------------------------------------------
    def rebase(self, ids, settle, now, why="rebase"):
        for v in ids:
            if v in self.hl:
                self._end(v, now, why)
            self.buf[v].clear()
            self.base[v] = None
            self.pend.pop(v, None)
            self.until[v] = max(self.until[v], now + settle)

    def observe(self, now, d, link_key=None):
        """The trigger filter, once per telemetry frame (field compares only)."""
        self.mode = d.get("mode_name") or ""
        self.dp_sub = d.get("dp_sub")
        self.first_test = bool(d.get("first_test"))
        keys = {
            "f1": (self.mode, self.dp_sub, bool(d.get("stopping"))),
            "f3": (d.get("auto_wp"), bool(d.get("prog_returning")), d.get("nav_kind"), bool(d.get("nav_applied"))),
            "f4": (d.get("speed_set_kn"), d.get("wp_kn"), d.get("power_fwd"), d.get("nav_cap10"), d.get("slam_cap"),
                   d.get("speed_limited")),
            "f5": d.get("gear"),
            "f6": bool(d.get("ov_hand")),
            "f7": bool(d.get("gen_active")),
            "f8": (link_key, d.get("gw_channel"), d.get("helm")),
        }
        p, self.prev = self.prev, keys
        trig = []
        if p is not None:
            for name, spec in (("f1", F1), ("f3", F3), ("f4", F4), ("f5", F5), ("f6", F6), ("f7", F7), ("f8", F8)):
                if keys[name] != p[name]:
                    trig.append(spec)
        if self._thr_step(self.thr_hist, now, d.get("throttle")) | \
                self._thr_step(self.dem_hist, now, d.get("throttle_demand")):
            trig.append(F2)
        if self._tgt_step(now, d.get("target")):
            trig.append(F3)
        if d.get("nav_applied") and finite(d.get("nav_heading")) and finite(d.get("heading")) and \
                angdiff(d["nav_heading"], d["heading"]) > TARGET_STEP:
            trig.append(F3)                         # an avoidance / navigation heading is steering the boat
        for ids, settle in trig:
            self.rebase(ids, settle, now)

    @staticmethod
    def _thr_step(hist, now, x):
        """A throttle step: > 5 % from the mean of the previous 10 s, or a rate over 2 %/s."""
        if not finite(x):
            return False
        hit = False
        if len(hist) >= 3:
            m = sum(h[1] for h in hist) / len(hist)
            hit = abs(x - m) > THR_STEP
            for t0, x0 in hist:                     # the oldest entry that is at least 0.8 s old
                if now - t0 >= 0.8:
                    if now - t0 <= 2.0 and abs(x - x0) / (now - t0) > THR_RATE:
                        hit = True
                    break
        hist.append((now, float(x)))
        while hist and now - hist[0][0] > 10.0:
            hist.popleft()
        return hit

    def _tgt_step(self, now, tgt):
        if not finite(tgt):
            return False
        h, hit = self.tgt_hist, False
        for t0, x0 in h:
            if now - t0 <= 1.0:
                hit = angdiff(tgt, x0) > TARGET_STEP
                break
        h.append((now, float(tgt)))
        while h and now - h[0][0] > 2.0:
            h.popleft()
        return hit

    # --- the judgement -----------------------------------------------------------------------------------
    def poll(self, model):
        """Judge the samples the model committed since the last call."""
        if self.epoch != model.epoch:
            self.reset()
            self.epoch = model.epoch
        n, size = model.n, model.N
        if n - self.done > size:
            self.done = n - size
        while self.done < n:
            i = self.done % size
            self._sample(float(model.times[i]), model.modes[i], {v: float(model.vals[v][i]) for v in CFG})
            self.done += 1

    def _modes_ok(self, v, mode):
        modes = CFG[v][4]
        if modes is None:
            return True
        if modes is ALL_BUT:
            return mode not in ALL_BUT
        if mode not in modes:
            return False
        return not (v == "xte" and mode == "DP" and self.dp_sub != DP_TRIP)

    def _sample(self, t, mode, vals):
        for v, cfg in CFG.items():
            x = vals[v]
            lv = self.last_valid[v]
            if lv is not None and t - lv > GAP_S:               # F9: a gap > 5 s ends everything of this value
                self._gap(v, t)
            if not finite(x):
                continue
            self.last_valid[v] = t
            if not self._modes_ok(v, mode) or (self.first_test and v in FIRST_TEST_OFF):
                if v in self.hl:
                    self._end(v, t, "rebase")
                self.pend.pop(v, None)
                self.buf[v].clear()
                self.base[v] = None
                continue
            if t < self.until[v]:
                continue
            b = self.buf[v]
            b.append((t, x))
            while b and b[0][0] < t - BASE_S:
                b.popleft()
            self._judge(v, cfg, t, mode)

    def _gap(self, v, t):
        if v in self.hl:
            self._end(v, t, "gap")
        self.buf[v].clear()
        self.base[v] = None
        self.pend.pop(v, None)
        self.last_valid[v] = None

    def _stats(self, v, t):
        """((m_B, s_B, n_B), (m_R, s_R, n_R)) or None when there is too little of either."""
        b = self.buf[v]
        c = self.base[v]
        if c is None or t - c[0] >= BASE_EVERY_S:
            lim = t - RECENT_S
            old = [x for tt, x in b if tt < lim]
            if len(old) > MAX_BASE:
                old = old[-MAX_BASE:]
            if len(old) >= BASE_MIN:
                m, s = robust(old)
                # (the next recompute is staggered per value: at most about one value sorts per 2 Hz sample)
                c = self.base[v] = (t - STAGGER_S * self.order[v], m, s, len(old))
            else:
                c = self.base[v] = (t, float("nan"), float("nan"), len(old))
        if c[3] < BASE_MIN:
            return None
        rec = [x for tt, x in reversed(b) if tt >= t - RECENT_S]
        if len(rec) < RECENT_MIN:
            return None
        mr, sr = robust(rec)
        return (c[1], c[2], c[3]), (mr, sr, len(rec))

    @staticmethod
    def _ratios(cfg, st):
        """{(kind, dir): (ratio, stat, thr)} of every test that applies; ratio >= 1 = beyond its threshold."""
        smin, amin, mdirs, sdirs, _m = cfg
        (mb, sb, _nb), (mr, sr, _nr) = st
        seff = max(sb, smin)
        out = {}
        dm = mr - mb
        z = abs(dm) / seff
        r = z / K_MEAN
        if amin:
            r = min(r, abs(dm) / amin)
        d = "u" if dm > 0 else "d"
        if d in mdirs:
            out[("mean", "up" if d == "u" else "down")] = (r, z, K_MEAN)
        q = max(sr, 0.0) / seff
        if "u" in sdirs:
            out[("spread", "up")] = (q / SPREAD_UP, q, SPREAD_UP)
        if "d" in sdirs and sb >= SPREAD_DOWN_BASE * smin:
            qd = sr / sb
            out[("spread", "down")] = ((SPREAD_DOWN / qd) if qd > 0 else 99.0, qd, SPREAD_DOWN)
        return out

    def _judge(self, v, cfg, t, mode):
        st = self._stats(v, t)
        h = self.hl.get(v)
        if st is None:
            if h:
                self._end(v, t, "gap")
            self.pend.pop(v, None)
            return
        rs = self._ratios(cfg, st)
        if h:
            r = rs.get((h["kind"], h["dir"]))
            if r is not None and r[0] >= OFF_RATIO:
                h["low"] = None
                h["ratio"] = r[0]
            else:
                if h["low"] is None:
                    h["low"] = t
                if t - h["low"] >= OFF_S and t - h["t"] >= MIN_SHOWN_S:
                    self._end(v, t, "calm")
            return
        cand = None
        for kind in ("mean", "spread"):                          # mean wins over spread
            for dr in ("up", "down"):
                r = rs.get((kind, dr))
                if r is not None and r[0] >= 1.0 and t >= self.cool.get((v, kind, dr), 0.0) and \
                        (cand is None or r[0] > cand[2][0]):
                    cand = (kind, dr, r)
            if cand is not None:
                break
        if cand is None:
            self.pend.pop(v, None)
            return
        p = self.pend.get(v)
        if p is None or (p[0], p[1]) != (cand[0], cand[1]):
            self.pend[v] = (cand[0], cand[1], t)
            return
        if t - p[2] >= ON_S:
            (mb, sb, _nb), (mr, sr, _nr) = st
            self.pend.pop(v, None)
            self.hl[v] = {"kind": cand[0], "dir": cand[1], "t": t, "low": None, "ratio": cand[2][0],
                          "base_m": mb, "rec_m": mr}
            self._emit({"ev": "on", "val": v, "kind": cand[0], "dir": cand[1], "base_m": mb, "base_s": sb,
                        "rec_m": mr, "rec_s": sr, "stat": cand[2][1], "thr": cand[2][2], "mode": mode,
                        "dur_s": None, "why": None, "t": t})

    def _end(self, v, t, why):
        h = self.hl.pop(v, None)
        if h is None:
            return
        self.cool[(v, h["kind"], h["dir"])] = t + COOLDOWN_S
        self._emit({"ev": "off", "val": v, "kind": h["kind"], "dir": h["dir"], "base_m": h["base_m"], "base_s": None,
                    "rec_m": h["rec_m"], "rec_s": None, "stat": h["ratio"], "thr": None, "mode": self.mode,
                    "dur_s": t - h["t"], "why": why, "t": t})

    def end_all(self, now, why="disconnect"):
        for v in list(self.hl):
            self._end(v, now, why)

    def _emit(self, ev):
        self.n_events += 1
        self.out.append(ev)
        if len(self.out) > 200:
            del self.out[:100]
