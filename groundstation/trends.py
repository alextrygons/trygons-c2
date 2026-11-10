"""The data of the trend panel «ΤΑΣΕΙΣ» (step TR2, CLAUDE.md Y, docs/trends_design.md 1 + 2): PURE - no tkinter, no
network, no import of anything that can send. A sampler fed with DECODED telemetry dicts (the existing TELEMETRY /
TELEM2 fields only) and a ring buffer per value.

READ-ONLY: this module cannot reach a boat, a gateway or a sound (test_trends TG6 / TG7 / TG8 prove it).

    tr = Trends()
    tr.feed(now, d, xte_m=None, depth_m=None, rtt_ms=None)    # every telemetry frame (cheap: field copies)
    tr.series("sog", 600, now)                                  # -> (times, values) of the last 10 min
    tr.segments("sog", 600, now, 260)                           # -> runs of (x 0..1, value), min / max per column
"""
import math
from array import array
from bisect import bisect_left

N = 7200                    # samples per ring: 60 min at 2 Hz (the largest zoom)
SLOT = 0.5                  # one sample every 0.5 s (2 Hz) on the station clock
BREAK_S = 1.0               # two samples more than this apart: the line is broken (a gap, never a fake line)
LOSS_S = 10.0               # the lost-frames window
FRAME_MS = 100              # the AURIX's telemetry period (link.c TELEM_PERIOD_MS)
ZOOMS = (2, 10, 30, 60)     # minutes
DEFAULT_ZOOM = 10
NAN = float("nan")

# value table (design 1): id, graph, unit, decimals, minimum span of the y scale (so noise is not amplified), unwrap
VALUES = (
    ("sog", 1, "kn", 1, 2.0, False),
    ("thr", 1, "%", 0, 20.0, False),
    ("hdg", 2, "°", 0, 20.0, True),
    ("rot", 2, "°/s", 1, 4.0, False),
    ("xte", 3, "m", 0, 10.0, False),
    ("roll", 4, "°", 1, 4.0, False),
    ("pitch", 4, "°", 1, 4.0, False),
    ("slam_g", 5, "g", 2, 1.0, False),
    ("slam_pm", 5, "/min", 0, 5.0, False),
    ("depth", 6, "m", 1, 5.0, False),
    ("bank_v", 7, "V", 1, 1.0, False),
    ("link_ms", 8, "ms", 0, 100.0, False),
    ("link_loss", 8, "%", 0, 5.0, False),
)
IDS = tuple(v[0] for v in VALUES)
INFO = {v[0]: {"graph": v[1], "unit": v[2], "dec": v[3], "span": v[4], "unwrap": v[5]} for v in VALUES}
GRAPHS = {g: tuple(v[0] for v in VALUES if v[1] == g) for g in range(1, 9)}
DEFAULT_SHOW = (1, 2, 4, 5, 7, 8)       # G1 G2 G4 G5 G7 G8 (design 4)
MAX_SHOWN = 6


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def num(d, key):
    """d[key] as a float, or NaN when missing / None / not a finite number (= a gap)."""
    v = d.get(key)
    return float(v) if finite(v) else NAN


class Trends:
    """Sampler + the rings. RAM only, never on disk (the debrief uses the logs)."""

    def __init__(self):
        self.vals = {i: array("f", [NAN]) * N for i in IDS}
        self.times = array("d", [0.0]) * N
        self.modes = [""] * N
        self.N = N
        self.epoch = 0                      # +1 per clear: the highlight detector restarts with it (TR3)
        self.n = 0                          # samples ever written since the last clear
        self.gen = 0                        # +1 per sample / clear: a cache key for the drawing
        self._clear_slot()
        self.frames = []                    # (t_ms) of the last LOSS_S seconds
        self.last_t_ms = None
        self.last_frame_t = None
        self._lin_gen = -1
        self._lin_times, self._lin_vals, self._bounds = None, {}, {}

    # --- clearing ---------------------------------------------------------------------------------------
    def _clear_slot(self):
        self.slot_t0 = None
        self.cur = {}
        self.cur_mode = ""

    def clear(self):
        """A boat change / disconnect / boat restart (t_ms back): every ring empty (design 2)."""
        for r in self.vals.values():
            for k in range(N):
                r[k] = NAN
        self.n = 0
        self.epoch += 1
        self.gen += 1
        self.frames = []
        self.last_t_ms = None
        self.last_frame_t = None
        self._clear_slot()

    # --- feeding ----------------------------------------------------------------------------------------
    def feed(self, now, d, xte_m=None, depth_m=None, rtt_ms=None):
        """One decoded telemetry dict (and the three station-side values). Cheap: field copies; a sample is
        committed when a frame arrives 0.5 s after the slot began."""
        t_ms = d.get("t_ms")
        if finite(t_ms):
            if self.last_t_ms is not None and t_ms < self.last_t_ms:
                self.clear()                # the boat restarted
            self.last_t_ms = t_ms
            self.frames.append((now, t_ms))
            while self.frames and now - self.frames[0][0] > LOSS_S:
                self.frames.pop(0)
        if self.slot_t0 is not None and now - self.slot_t0 >= SLOT:
            self._commit()
        if self.slot_t0 is None:
            self.slot_t0 = now
        c = self.cur
        c["sog"] = num(d, "est_sog")
        c["thr"] = num(d, "throttle")
        c["hdg"] = num(d, "heading") if d.get("compass_ok", True) else NAN
        c["rot"] = num(d, "yaw_rate")
        c["xte"] = float(xte_m) if finite(xte_m) else NAN
        c["roll"] = num(d, "att_roll")
        c["pitch"] = num(d, "att_pitch")
        g = num(d, "slam_peak")
        if finite(g):
            old = c.get("slam_g")
            g = abs(g)
            c["slam_g"] = g if old is None or not finite(old) else max(old, g)     # the MAX of the frames in a slot
        elif "slam_g" not in c:
            c["slam_g"] = NAN
        c["slam_pm"] = num(d, "slam_rate")
        c["depth"] = float(depth_m) if finite(depth_m) else NAN
        c["bank_v"] = num(d, "bank_v")
        c["link_ms"] = float(rtt_ms) if finite(rtt_ms) else NAN
        c["link_loss"] = self.loss_pct()
        self.cur_mode = d.get("mode_name") or ""
        self.last_frame_t = now

    def loss_pct(self):
        """Telemetry frames missed over the last LOSS_S seconds from the t_ms steps (expected FRAME_MS apart), in %;
        NaN until there are 2 s of history (design 1)."""
        f = self.frames
        if len(f) < 2 or f[-1][1] - f[0][1] < 2000:
            return NAN
        expected = (f[-1][1] - f[0][1]) / float(FRAME_MS) + 1.0
        return max(0.0, min(100.0, 100.0 * (expected - len(f)) / expected))

    def _commit(self):
        """Write the slot as one sample (stamped with its newest frame's time)."""
        lf, cur, mode = self.last_frame_t, self.cur, self.cur_mode
        self._clear_slot()
        if lf is None or not cur:
            return
        if self.n and self.times[(self.n - 1) % N] >= lf:
            return
        i = self.n % N
        for k in IDS:
            self.vals[k][i] = cur.get(k, NAN)
        self.times[i] = lf
        self.modes[i] = mode
        self.n += 1
        self.gen += 1

    def tick(self, now):
        """Called from the station's idle tick: a slot whose frames stopped is committed once it is older than SLOT
        (so the last sample before a gap is not held back until the next frame). Its time stays the newest frame's:
        the missing time after it is a gap (BREAK_S), never a held value."""
        if self.slot_t0 is not None and now - self.slot_t0 >= SLOT:
            self._commit()

    # --- reading ----------------------------------------------------------------------------------------
    def count(self):
        return min(self.n, N)

    def _linear(self, arr, cnt):
        """The newest cnt entries of a ring in time order."""
        start = (self.n - cnt) % N
        if start + cnt <= N:
            return arr[start:start + cnt]
        return arr[start:] + arr[:start + cnt - N]

    def series(self, vid, window_s, now):
        """(times, values) of the last window_s seconds (times as station epoch seconds; NaN = a gap). The heading is
        unwrapped (no jump at 0 / 360)."""
        cnt = self.count()
        if not cnt:
            return [], []
        ts = self._linear(self.times, cnt)
        lo = now - window_s
        a = 0
        b = cnt
        while a < b:                        # first index with ts >= lo (binary search)
            m = (a + b) // 2
            if ts[m] < lo:
                a = m + 1
            else:
                b = m
        vs = self._linear(self.vals[vid], cnt)
        t, v = list(ts[a:]), list(vs[a:])
        if INFO[vid]["unwrap"]:
            off, prev = 0.0, None
            for k, x in enumerate(v):
                if finite(x):
                    if prev is not None:
                        if x - prev > 180.0:
                            off -= 360.0
                        elif prev - x > 180.0:
                            off += 360.0
                    prev = x
                    v[k] = x + off
        return t, v

    def last(self, vid):
        """The newest committed value (NaN when none)."""
        if not self.n:
            return NAN
        return float(self.vals[vid][(self.n - 1) % N])

    def at(self, t):
        """The sample nearest to station time t within 1 s -> {"t", "mode", id: value ...} or None."""
        cnt = self.count()
        if not cnt:
            return None
        ts = self._linear(self.times, cnt)
        a, b = 0, cnt
        while a < b:
            m = (a + b) // 2
            if ts[m] < t:
                a = m + 1
            else:
                b = m
        best = None
        for k in (a - 1, a):
            if 0 <= k < cnt and (best is None or abs(ts[k] - t) < abs(ts[best] - t)):
                best = k
        if best is None or abs(ts[best] - t) > BREAK_S:
            return None
        start = (self.n - cnt) % N
        i = (start + best) % N
        out = {"t": float(self.times[i]), "mode": self.modes[i]}
        for k in IDS:
            out[k] = float(self.vals[k][i])
        return out

    # --- the drawing data: per pixel column --------------------------------------------------------------
    def _lin_t(self):
        if self._lin_gen != self.gen:
            self._lin_gen = self.gen
            self._lin_times = self._linear(self.times, self.count())
            self._lin_vals = {}
            self._bounds = {}
        return self._lin_times

    def _lin_v(self, vid):
        self._lin_t()
        v = self._lin_vals.get(vid)
        if v is None:
            v = self._lin_vals[vid] = self._linear(self.vals[vid], self.count())
        return v

    def _columns(self, window_s, now, cols):
        """Index boundaries of the pixel columns of [now - window_s, now] in the time-ordered samples (shared by every
        value of one redraw)."""
        ts = self._lin_t()
        key = (window_s, now, cols)
        b = self._bounds.get(key)
        if b is None:
            if len(self._bounds) > 24:
                self._bounds.clear()
            t0, w = now - window_s, window_s / float(cols)
            b = self._bounds[key] = [bisect_left(ts, t0 + c * w) for c in range(cols + 1)]
        return b

    def segments(self, vid, window_s, now, cols):
        """The line as runs [(x, value) ...], x in 0..1 over [now - window_s, now]: a run ends where a gap of more than
        BREAK_S (a NaN stretch included) separates two columns; at most one MIN and one MAX point per pixel column in
        the order they happened (a spike is never lost); the heading is unwrapped (no jump at 0 / 360).
        Returns (runs, lo, hi) of the finite values drawn (lo > hi = nothing to draw)."""
        cols = max(1, int(cols))
        runs, lo, hi = [], float("inf"), float("-inf")
        if not self.n:
            return runs, lo, hi
        ts, vs = self._lin_t(), self._lin_v(vid)
        bounds = self._columns(window_s, now, cols)
        t0 = now - window_s
        unwrap = INFO[vid]["unwrap"]
        run, last_t, prev_raw, off = [], None, None, 0.0
        for c in range(cols):
            a, b = bounds[c], bounds[c + 1]
            if a >= b:
                continue
            seg = vs[a:b]
            s = sum(seg)
            if s == s and not (unwrap and max(seg) - min(seg) > 180.0):      # no NaN (the usual case): C-speed
                mn, mx = min(seg), max(seg)
                imn, imx = a + seg.index(mn), a + seg.index(mx)
                first_raw, last_raw = seg[0], seg[-1]
                t_first, t_last = ts[a], ts[b - 1]
            else:                                                            # NaNs / a wrap inside the column
                fin = [(i, x) for i, x in zip(range(a, b), seg) if x == x]
                if not fin:
                    continue
                first_raw, last_raw = fin[0][1], fin[-1][1]
                t_first, t_last = ts[fin[0][0]], ts[fin[-1][0]]
                if unwrap and max(x for _i, x in fin) - min(x for _i, x in fin) > 180.0:
                    fin = [(i, x + 360.0 if x < 180.0 else x) for i, x in fin]
                imn, mn = min(fin, key=lambda p: p[1])
                imx, mx = max(fin, key=lambda p: p[1])
            if last_t is not None and t_first - last_t > BREAK_S:
                runs.append(run)
                run = []
            if unwrap and prev_raw is not None:
                if first_raw - prev_raw > 180.0:
                    off -= 360.0
                elif prev_raw - first_raw > 180.0:
                    off += 360.0
            prev_raw, last_t = last_raw, t_last
            pts = [(imn, mn)] if imn == imx else sorted(((imn, mn), (imx, mx)))
            for i, x in pts:
                run.append(((ts[i] - t0) / window_s, x + off))
            lo, hi = min(lo, mn + off), max(hi, mx + off)
        if run:
            runs.append(run)
        return runs, lo, hi
