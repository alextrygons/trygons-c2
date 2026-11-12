"""The harbour fusion tracker (CLAUDE.md Z, docs/harbour_design.md 6 + 6a, step HX4).

Radar plots + LiDAR clusters + camera obstacles (eWaSR boxes already turned into bearing + rough range by
gateway/vision) + AIS targets -> FUSED TRACKS in the local flat frame (ENU metres from the harbour origin).

PURE + READ ONLY: no process, no thread, no I/O, no network; it never imports aurix / stations / proto and never sends
anything (gateway/test_harbour_tracker.py HT14 + test_harbour_sensors.py HS9). The planner (HX5) reads tracks() and
flags(); nothing in here decides a heading or a speed.

Track (as dicts from tracks()):
  id, t, x, y, lat, lon, vx, vy, speed, course, P (4x4 covariance), sd_m (1-sigma position), len_m, beam_m, hgt_m,
  cls "vessel" | "static" | "small" (kayak / dinghy) | "person" (person-like: small + low / flat) | "buoy" | "unknown",
  conf 0..1, src "RLCA" letters (radar, LiDAR, camera, AIS seen in the last 5 s), last {R, L, C, A}, hits_R / hits_L
  (in their window), confirmed, steer_ok (= the design's range_ok), coasting, static, in_static_map, surface (a flat
  LiDAR return in the water band), ghost_suspect, mmsi, age_s, born.

M0-3 - steer_ok: ONLY radar or LiDAR hits can set it (RANGE_SRC); a camera-only or AIS-only track is never steer_ok
(it may only feed slow / stop); steer_ok needs a confirmed track AND (radar: >= 3 hits in 4 s, the last < 2 s) or
(LiDAR: >= 3 hits in 1 s, the last < 0.5 s). The radar window is longer than the design's 1 s because a 0.8 Hz radar
can never give 3 hits in 1 s (as built 6a). A track that stops getting range hits loses steer_ok at once (coasting);
the TRACK itself is kept (an obstacle never silently disappears) until delete_s.

CONSERVATIVE: every doubt makes an obstacle, never removes one - a radar multipath ghost is KEPT (ghost_suspect,
still steer_ok when radar-confirmed), a LiDAR phantom is an obstacle, a flat water-band return is tracked like any
other cluster (surface = a label, never a deletion).

flags() = the planner's "unsure = stop" inputs (design 7.4), always explicit:
  range_lost (U1), degraded [sensors] (U2), stale {sensor: age} , time_skew [sensors] (U11), conflicts [...] (U6),
  crowded (U9), no_static_map (U10), no_pose, overload [...], error, ghost_suspect [ids], attitude "imu" | "none";
  unsure = the list of every active reason code.
"""
import math

from harbour import simworld

RANGE_SRC = ("R", "L")          # ONLY these sources confirm a track for steering (M0-3); HT mutation checks it
SRC_ALL = ("R", "L", "C", "A")
BIG = 1e9
FLAT_M = 0.15

DEFAULTS = {
    "gate_chi2": 13.8,          # 2 dof, 99.9 % (a confirmed track)
    "gate_chi2_tent": 9.21,     # 2 dof, 99 % (a tentative track: fewer clutter captures)
    "birth_sd_v": 1.5,          # m/s: the velocity sd of a new track without an ARPA velocity
    "clutter_births_s": 15.0,   # range-sensor births per second within clutter_r_m = a clutter flag (U9)
    "clutter_r_m": 25.0,
    "pre_gate_m": 30.0,         # a cheap Euclidean pre-gate before the Mahalanobis test
    "q_acc": 0.3,               # process noise: acceleration sd m/s^2 (harbour craft)
    "radar": {"window_s": 4.0, "fresh_s": 2.0, "confirm_n": 3, "range_ok_n": 3, "delete_s": 5.0, "stale_s": 2.5,
              "range_sd_m": 3.0, "range_sd_frac": 0.005, "brg_sd_deg": 1.0, "arpa_sd_mps": 0.5},
    "lidar": {"window_s": 1.0, "fresh_s": 0.5, "confirm_n": 5, "range_ok_n": 3, "delete_s": 3.0, "stale_s": 0.5,
              "pos_sd_m": 0.25, "mount_x_m": 1.0, "mount_h_m": 2.5, "water_band_m": 0.3},
    # window_s / fresh_s / confirm_n / range_ok_n of camera + AIS are used ONLY if C / A were a range source - they
    # never are (RANGE_SRC); the HT15 mutation adds them to prove the camera / AIS checks catch it
    "camera": {"window_s": 10.0, "fresh_s": 2.0, "confirm_n": 3, "range_ok_n": 3, "brg_gate_deg": 4.0, "brg_sd_deg": 2.0, "range_sd_min_m": 8.0, "delete_s": 5.0, "max_range_m": 400.0},
    "ais": {"window_s": 10.0, "fresh_s": 2.0, "confirm_n": 3, "range_ok_n": 3, "assoc_m": 30.0, "assoc_v_mps": 2.0, "sd_m": 10.0, "delete_s": 5.0, "unlink_m": 90.0},
    "conflict": {"near_m": 50.0, "disagree_m": 20.0, "camera_near_m": 30.0, "camera_ahead_deg": 30.0},
    "tentative_s": 3.0,         # a tentative track not confirmed within this (+ its sensor window) is deleted
    "static_v": 0.25, "static_s": 20.0, "static_tau_s": 5.0, "static_off_v": 0.5,
    "crowded_n": 20, "crowded_r": 100.0,
    "max_tracks": 300, "budget_ms": 30.0,
    "time_jump_s": 10.0,        # a batch / camera / AIS time this far from the tracker clock is refused (review H3)
    "err_hold_s": 2.0,          # rejected input / capped items stay an unsure reason this long
    "merge_m": 1.5, "merge_v": 1.0,
    "ghost_brg_deg": 2.0, "ghost_frac": 0.06,
}


def merged(cfg):
    out = {}
    cfg = cfg or {}
    for k, v in DEFAULTS.items():
        if isinstance(v, dict):
            d = dict(v)
            d.update(cfg.get(k) or {})
            out[k] = d
        else:
            out[k] = cfg.get(k, v)
    return out


# --- small linear algebra (4x4 / 2x2, pure Python) ------------------------------------------------------------------
def _inv2(a, b, c, d):
    det = a * d - b * c
    if abs(det) < 1e-12:
        det = 1e-12
    return d / det, -b / det, -c / det, a / det


def polar_cov(rng, brg_rad, sd_r, sd_b):
    """Covariance (xx, xy, yy) in ENU of a range / true-bearing measurement (bearing clockwise from north)."""
    s, c = math.sin(brg_rad), math.cos(brg_rad)
    sr2, st2 = sd_r * sd_r, (rng * sd_b) ** 2
    # unit vectors: along = (s, c), across = (c, -s)
    xx = sr2 * s * s + st2 * c * c
    yy = sr2 * c * c + st2 * s * s
    xy = (sr2 - st2) * s * c
    return xx, xy, yy


def hungarian(cost):
    """Minimum-cost assignment rows -> columns (len(rows) <= len(cols)); returns col index per row (-1 = none)."""
    n, m = len(cost), len(cost[0]) if cost else 0
    if n == 0 or m == 0:
        return [-1] * n
    INF = float("inf")
    u, v, p, way = [0.0] * (n + 1), [0.0] * (m + 1), [0] * (m + 1), [0] * (m + 1)
    for i in range(1, n + 1):
        p[0], j0 = i, 0
        minv, used = [INF] * (m + 1), [False] * (m + 1)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], INF, 0
            row = cost[i0 - 1]
            for j in range(1, m + 1):
                if not used[j]:
                    cur = row[j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    ans = [-1] * n
    for j in range(1, m + 1):
        if p[j]:
            ans[p[j] - 1] = j - 1
    return ans


def assign(nt, nm, pairs):
    """Global nearest neighbour over gated pairs {(ti, mi): cost}: Hungarian per connected component.
    Returns {ti: mi}."""
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for (ti, mi) in pairs:
        ra, rb = find(("t", ti)), find(("m", mi))
        if ra != rb:
            parent[ra] = rb
    comps = {}
    for (ti, mi) in pairs:
        comps.setdefault(find(("t", ti)), []).append((ti, mi))
    out = {}
    for plist in comps.values():
        ts = sorted({p[0] for p in plist})
        ms = sorted({p[1] for p in plist})
        if len(ts) == 1 and len(ms) == 1:
            out[ts[0]] = ms[0]
            continue
        tr = len(ts) > len(ms)
        rows, cols = (ms, ts) if tr else (ts, ms)
        c = [[BIG] * len(cols) for _ in rows]
        ri, ci = {r: i for i, r in enumerate(rows)}, {x: i for i, x in enumerate(cols)}
        for (ti, mi), cost in ((p, pairs[p]) for p in plist):
            if tr:
                c[ri[mi]][ci[ti]] = cost
            else:
                c[ri[ti]][ci[mi]] = cost
        res = hungarian(c)
        for i, j in enumerate(res):
            if j < 0 or c[i][j] >= BIG:
                continue
            if tr:
                out[cols[j]] = rows[i]
            else:
                out[rows[i]] = cols[j]
    return out


def _interval(ob, d):
    """The vision obstacle's range interval (near, far); far None = up to 2 x the estimate."""
    rng = ob.get("range") or ob.get("rng")
    lo, hi = 0.5 * d, 2.0 * d
    if isinstance(rng, (list, tuple)) and len(rng) == 2:
        try:
            if rng[0] is not None:
                lo = float(rng[0])
            if rng[1] is not None:
                hi = float(rng[1])
        except (TypeError, ValueError):
            pass
    return min(lo, d), max(hi, d)


class Track:
    __slots__ = ("id", "t", "s", "P", "len_m", "beam_m", "hgt_m", "hits", "last", "born", "confirmed", "mmsi",
                 "slow_since", "surface_n", "lidar_n", "ais_xy", "arpa", "ghost", "in_static", "cam_only_born",
                 "coasting", "steer_ok", "static", "cls", "conf", "disagree", "v_ema", "v_t")

    def __init__(self, tid, t, x, y, vx=0.0, vy=0.0, sd_p=2.0, sd_v=3.0):
        self.id, self.t = tid, t
        self.s = [x, y, vx, vy]
        self.P = [[sd_p * sd_p, 0, 0, 0], [0, sd_p * sd_p, 0, 0], [0, 0, sd_v * sd_v, 0], [0, 0, 0, sd_v * sd_v]]
        self.len_m = self.beam_m = 0.0
        self.hgt_m = None
        self.hits = {k: [] for k in SRC_ALL}
        self.last = {k: None for k in SRC_ALL}
        self.born, self.confirmed, self.mmsi = t, False, None
        self.slow_since = None
        self.surface_n = self.lidar_n = 0
        self.ais_xy = None
        self.arpa = None
        self.ghost = self.in_static = self.coasting = self.steer_ok = self.static = False
        self.cam_only_born = False
        self.cls, self.conf = "unknown", 0.0
        self.disagree = None
        self.v_ema, self.v_t = None, None      # the speed averaged over ~static_tau_s (the static judgement)

    # constant-velocity Kalman filter
    def predict(self, t, q):
        dt = t - self.t
        if dt == 0.0:
            return
        a = abs(dt)
        s, P = self.s, self.P
        s[0] += s[2] * dt
        s[1] += s[3] * dt
        # P = F P F' + Q
        for i in (0, 1):
            j = i + 2
            # rows
            for k in range(4):
                P[i][k] += dt * P[j][k]
            for k in range(4):
                P[k][i] += dt * P[k][j]
        q2 = q * q
        d4, d3, d2 = a ** 4 / 4.0 * q2, a ** 3 / 2.0 * q2, a * a * q2
        P[0][0] += d4
        P[1][1] += d4
        P[0][2] += d3
        P[2][0] += d3
        P[1][3] += d3
        P[3][1] += d3
        P[2][2] += d2
        P[3][3] += d2
        self.t = t                  # an out-of-order (older) batch moves the state back; Q grows with |dt| both ways

    def maha(self, x, y, R):
        P = self.P
        a, b, c, d = P[0][0] + R[0], P[0][1] + R[1], P[1][0] + R[1], P[1][1] + R[2]
        ia, ib, ic, idd = _inv2(a, b, c, d)
        ex, ey = x - self.s[0], y - self.s[1]
        return ex * (ia * ex + ib * ey) + ey * (ic * ex + idd * ey)

    def update(self, x, y, R, k=0):
        """A Kalman update of the state pair (k, k + 1): k = 0 position (x, y), k = 2 velocity (the radar's ARPA)."""
        P = self.P
        m = k + 1
        a, b, c, d = P[k][k] + R[0], P[k][m] + R[1], P[m][k] + R[1], P[m][m] + R[2]
        ia, ib, ic, idd = _inv2(a, b, c, d)
        K = [[P[i][k] * ia + P[i][m] * ic, P[i][k] * ib + P[i][m] * idd] for i in range(4)]
        ex, ey = x - self.s[k], y - self.s[m]
        for i in range(4):
            self.s[i] += K[i][0] * ex + K[i][1] * ey
        r0, r1 = list(P[k]), list(P[m])
        for i in range(4):
            for j in range(4):
                P[i][j] -= K[i][0] * r0[j] + K[i][1] * r1[j]
        for i in range(4):                      # keep it symmetric
            for j in range(i + 1, 4):
                m = 0.5 * (P[i][j] + P[j][i])
                P[i][j] = P[j][i] = m

    def hit(self, src, t, window):
        h = self.hits[src]
        h.append(t)
        while h and t - h[0] > window:
            h.pop(0)
        self.last[src] = t if self.last[src] is None else max(self.last[src], t)

    def n_hits(self, src, T, window):
        return sum(1 for x in self.hits[src] if T - x <= window)

    def has_range(self):
        """Any hit from a range source (RANGE_SRC) - even a single, maybe spurious one."""
        return any(self.last[k] is not None for k in RANGE_SRC)

    def ranged(self):
        """Confirmed by a range source: the only kind of track AIS / camera associate to."""
        return self.confirmed and self.has_range()


class Tracker:
    def __init__(self, cfg=None, static_map=None):
        self.c = merged(cfg)
        self.tracks = []
        self.next_id = 1
        self.origin = None
        self.T = 0.0                        # the tracker clock = the newest measurement time (never back)
        self.pose = None                    # the newest batch pose
        self.own = None                     # (x, y, heading)
        self.static = static_map
        self.attitude = None                # (roll, pitch) deg from the BNO085 telemetry, or None
        self.timing = []                    # ms per feed / cycle (newest 2000)
        self.latency = []                   # s: gateway time - sensor time when a batch is fused
        self.counts = {"batches": 0, "meas": 0, "births": 0, "deleted": 0, "merged": 0, "dropped_cap": 0,
                       "camera": 0, "ais": 0, "errors": 0, "rejected": 0,
                       "time_jump": 0}
        self.err_seen = 0                    # errors counted at the last cycle
        self.err_until = None                # input errors are reported (unsure) until this tracker time
        self.cap_seen = 0
        self.cap_until = None
        self._flags = {"range_lost": True, "unsure": ["tracker not started"], "degraded": ["radar", "lidar"],
                       "conflicts": [], "error": False, "no_pose": True}      # review M3: fail closed
        self.health = {}
        self.cycles = 0
        self.near_births = []                # birth times close by (the clutter flag)

    # --- frames ------------------------------------------------------------------------------------------------------
    def set_origin(self, lat, lon):
        if self.origin is None:
            self.origin = (float(lat), float(lon))

    def xy(self, lat, lon):
        return simworld.ll_to_xy(self.origin[0], self.origin[1], lat, lon)

    def _own_from(self, pose):
        if not isinstance(pose, dict) or pose.get("lat") is None or pose.get("heading") is None:
            return None
        self.set_origin(pose["lat"], pose["lon"])
        x, y = self.xy(pose["lat"], pose["lon"])
        return x, y, float(pose["heading"]) % 360.0

    def water_z(self, x, y):
        """The expected water height in the LiDAR SENSOR frame at (x ahead, y starboard), with the boat's attitude
        (roll + = starboard down, pitch + = bow up) when the telemetry has it."""
        mh = float(self.c["lidar"]["mount_h_m"])
        if self.attitude is None:
            return -mh
        r, p = math.radians(self.attitude[0]), math.radians(self.attitude[1])
        return -mh * math.cos(r) * math.cos(p) - x * math.sin(p) + y * math.sin(r)

    def surface_of(self, o):
        """The water-band label (HX3 round 2 MED-1: a label only, never a deletion), attitude-compensated."""
        if self.attitude is None:
            return bool(o.get("surface"))
        flat = float(o.get("hgt_m", 1.0)) < FLAT_M
        top = float(o["z"]) + float(o.get("hgt_m", 0.0)) / 2.0
        return flat and top < self.water_z(float(o["x"]), float(o["y"])) + float(self.c["lidar"]["water_band_m"])

    def _time_ok(self, t):
        if not math.isfinite(t) or (self.T > 0 and abs(t - self.T) > float(self.c["time_jump_s"])):
            self.counts["errors"] += 1
            self.counts["rejected"] += 1
            self.counts["time_jump"] += 1
            return False
        return True

    # --- measurements ------------------------------------------------------------------------------------------------
    def _meas_radar(self, items, own):
        c = self.c["radar"]
        ox, oy, hdg = own
        out = []
        for p in items:
            r = float(p["range_m"])
            b = math.radians((hdg + float(p["brg_deg"])) % 360.0)
            size = float(p.get("size_m") or 0.0)
            sd_r = float(c["range_sd_m"]) + float(c["range_sd_frac"]) * r + 0.25 * size
            sd_b = math.radians(float(c["brg_sd_deg"])) + (0.25 * size / max(r, 1.0))
            arpa = None
            if p.get("speed_mps") is not None and p.get("course_deg") is not None:
                v, cr = float(p["speed_mps"]), math.radians(float(p["course_deg"]))
                arpa = (v * math.sin(cr), v * math.cos(cr))
            out.append({"x": ox + r * math.sin(b), "y": oy + r * math.cos(b), "R": polar_cov(r, b, sd_r, sd_b),
                        "len": size, "beam": size, "hgt": None, "arpa": arpa, "surface": False,
                        "strength": float(p.get("strength") or 0.0), "range": r, "brg": math.degrees(b) % 360.0})
        return out

    def _meas_lidar(self, items, own):
        c = self.c["lidar"]
        ox, oy, hdg = own
        h = math.radians(hdg)
        mx = float(c["mount_x_m"])
        out = []
        for o in items:
            ahead, stbd = float(o["x"]) + mx, float(o["y"])        # boat frame from the hull middle
            x = ox + ahead * math.sin(h) + stbd * math.cos(h)
            y = oy + ahead * math.cos(h) - stbd * math.sin(h)
            size = max(float(o.get("len_m", 0.0)), float(o.get("wid_m", 0.0)))
            sd = float(c["pos_sd_m"]) + 0.25 * size + 0.002 * math.hypot(ahead, stbd)
            out.append({"x": x, "y": y, "R": (sd * sd, 0.0, sd * sd), "len": float(o.get("len_m", 0.0)),
                        "beam": float(o.get("wid_m", 0.0)), "hgt": float(o.get("hgt_m", 0.0)), "arpa": None,
                        "surface": self.surface_of(o), "strength": 1.0,
                        "range": math.hypot(ahead, stbd), "brg": None})
        return out

    def feed(self, kind, batch, now=None):
        """One radar / LiDAR batch (sensors.py frame). `now` = the gateway's epoch clock (latency only)."""
        if kind not in ("radar", "lidar") or not isinstance(batch, dict):
            return
        own = self._own_from(batch.get("pose"))
        if own is None:                     # review H2: no pose / heading = the batch is NOT fused - said so
            self.counts["errors"] += 1
            self.counts["rejected"] += 1
            self.own = None
            return
        try:
            t = float(batch["t"])
        except (KeyError, TypeError, ValueError):
            t = float("nan")
        if not math.isfinite(t) or (self.T > 0 and abs(t - self.T) > float(self.c["time_jump_s"])):
            self.counts["errors"] += 1              # review H3: a time far from the tracker clock = refused
            self.counts["rejected"] += 1
            self.counts["time_jump"] += 1
            return
        self.T = max(self.T, t)
        self.own, self.pose = own, batch.get("pose")
        items = batch.get("items") or []
        if batch.get("capped"):
            self.counts["dropped_cap"] += int(batch["capped"])
        meas = self._meas_radar(items, own) if kind == "radar" else self._meas_lidar(items, own)
        src = "R" if kind == "radar" else "L"
        self._associate(meas, t, src)
        self.counts["batches"] += 1
        self.counts["meas"] += len(meas)
        if now is not None:
            self.latency.append(max(0.0, float(now) - t))
            del self.latency[:-2000]

    def _associate(self, meas, t, src):
        c = self.c
        q = float(c["q_acc"])
        gate, pre = float(c["gate_chi2"]), float(c["pre_gate_m"])
        gate_t = float(c["gate_chi2_tent"])
        sc = c["radar"] if src == "R" else c["lidar"]
        for tr in self.tracks:
            tr.predict(t, q)
        # a coarse spatial hash of the tracks for the pre-gate
        cell = max(pre, 5.0)
        grid = {}
        for i, tr in enumerate(self.tracks):
            grid.setdefault((int(math.floor(tr.s[0] / cell)), int(math.floor(tr.s[1] / cell))), []).append(i)
        pairs = {}
        for mi, m in enumerate(meas):
            gx, gy = int(math.floor(m["x"] / cell)), int(math.floor(m["y"] / cell))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for ti in grid.get((gx + dx, gy + dy), ()):
                        tr = self.tracks[ti]
                        if tr.mmsi is not None and not tr.has_range():
                            continue                # an AIS-only track: range data make their own track
                        if math.hypot(tr.s[0] - m["x"], tr.s[1] - m["y"]) > pre + 0.5 * max(tr.len_m, m["len"]):
                            continue
                        d2 = tr.maha(m["x"], m["y"], m["R"])
                        if d2 <= (gate if tr.confirmed else gate_t):
                            pairs[(ti, mi)] = d2
        res = assign(len(self.tracks), len(meas), pairs)
        used = set()
        for ti, mi in res.items():
            tr, m = self.tracks[ti], meas[mi]
            used.add(mi)
            tr.update(m["x"], m["y"], m["R"])
            if m["arpa"] is not None:               # the radar's own target velocity (ARPA): a velocity update
                sv = float(sc.get("arpa_sd_mps", 0.5))
                tr.update(m["arpa"][0], m["arpa"][1], (sv * sv, 0.0, sv * sv), k=2)
            self._absorb(tr, m, t, src, sc)
        for mi, m in enumerate(meas):
            if mi in used:
                continue
            vx = vy = 0.0
            sd_v = float(c["birth_sd_v"])
            if m["arpa"] is not None:
                vx, vy = m["arpa"]
                sd_v = 1.0
            sd_p = math.sqrt(max(m["R"][0], m["R"][2]))
            tr = Track(self.next_id, t, m["x"], m["y"], vx, vy, sd_p=sd_p, sd_v=sd_v)
            tr.P[0][0], tr.P[0][1], tr.P[1][0], tr.P[1][1] = m["R"][0], m["R"][1], m["R"][1], m["R"][2]
            self.next_id += 1
            self.counts["births"] += 1
            self._absorb(tr, m, t, src, sc)
            self.tracks.append(tr)
            if self.own is not None and math.hypot(m["x"] - self.own[0], m["y"] - self.own[1]) <= \
                    float(c["clutter_r_m"]):
                self.near_births.append(t)
        while self.near_births and t - self.near_births[0] > 2.0:
            self.near_births.pop(0)

    def _absorb(self, tr, m, t, src, sc):
        tr.hit(src, t, float(sc["window_s"]) * 2.0)
        a = 0.3
        tr.len_m = m["len"] if tr.len_m == 0.0 else max(m["len"], (1 - a) * tr.len_m + a * m["len"])
        tr.beam_m = m["beam"] if tr.beam_m == 0.0 else (1 - a) * tr.beam_m + a * m["beam"]
        if m["hgt"] is not None:
            tr.hgt_m = m["hgt"] if tr.hgt_m is None else max(m["hgt"], (1 - a) * tr.hgt_m + a * m["hgt"])
            tr.lidar_n += 1
            if m["surface"]:
                tr.surface_n += 1
        if m["arpa"] is not None:
            tr.arpa = m["arpa"]

    # --- camera + AIS --------------------------------------------------------------------------------------------
    def camera(self, obstacles, pose, t):
        """Vision obstacles {rel (deg from the bow), dist m | None, range [near, far], conf, confirmed} at time t (the
        gateway clock). Associated by BEARING to an existing range track (adds C: confidence / class only, the
        position stays the range sensors'), else a camera-only track (never steer_ok)."""
        own = self._own_from(pose)
        if own is None:
            return
        c = self.c["camera"]
        ox, oy, hdg = own
        t = float(t)
        if not self._time_ok(t):
            return
        self.T = max(self.T, t)
        for ob in obstacles or ():
            try:
                if ob.get("confirmed") is False:
                    continue
                d = ob.get("dist")
                rel = float(ob["rel"])
                if d is None or not math.isfinite(rel) or not math.isfinite(float(d)) or float(d) <= 0:
                    continue                # beyond the horizon: no position to fuse (shown by vision itself)
                d = min(float(d), float(c["max_range_m"]))
                rng = ob.get("range") or ob.get("rng") or (None, None)
            except (KeyError, TypeError, ValueError):
                self.counts["errors"] += 1
                continue
            self.counts["camera"] += 1
            brg = (hdg + rel) % 360.0
            best, bd = None, None
            for tr in self.tracks:
                if not tr.ranged():
                    continue
                dx, dy = tr.s[0] - ox, tr.s[1] - oy
                r_t = math.hypot(dx, dy)
                if r_t < 1.0:
                    continue
                db = abs((math.degrees(math.atan2(dx, dy)) - brg + 540.0) % 360.0 - 180.0)
                tol = float(c["brg_gate_deg"]) + math.degrees(math.atan2(0.5 * max(tr.len_m, 1.0), r_t))
                # review M2: the range track must lie INSIDE the vision range interval (+ its own half size + 10 %):
                # a swimmer in front of a moored boat on the same bearing is never absorbed by the boat's track
                lo, hi = _interval(ob, d)
                half = 0.5 * max(tr.len_m, tr.beam_m)
                # and its NEAR side (the centroid - half its size: the camera sees the near waterline) within
                # max(3 m, 30 %) of the camera's own estimate - a nearer camera object is a separate one
                near_side = max(0.0, r_t - half)
                if db <= tol and lo * 0.9 - half <= r_t <= hi * 1.1 + half and                         abs(near_side - d) <= max(3.0, 0.3 * d) and (bd is None or db < bd):
                    best, bd = tr, db
            if best is not None:
                best.hit("C", t, 10.0)
                continue
            near, far = (rng[0], rng[1]) if isinstance(rng, (list, tuple)) and len(rng) == 2 else (None, None)
            sd_r = max(float(c["range_sd_min_m"]), (float(far) - float(near)) / 2.0 if near is not None and
                       far is not None else d * 0.5)
            b = math.radians(brg)
            R = polar_cov(d, b, sd_r, math.radians(float(c["brg_sd_deg"])))
            x, y = ox + d * math.sin(b), oy + d * math.cos(b)
            cand = None
            for tr in self.tracks:
                if tr.ranged() or tr.mmsi is not None:
                    continue
                tr.predict(t, float(self.c["q_acc"]))
                d2 = tr.maha(x, y, R)
                if d2 <= float(self.c["gate_chi2"]) and (cand is None or d2 < cand[1]):
                    cand = (tr, d2)
            if cand is not None:
                cand[0].update(x, y, R)
                cand[0].hit("C", t, 10.0)
            else:
                tr = Track(self.next_id, t, x, y, sd_p=sd_r)
                tr.P[0][0], tr.P[0][1], tr.P[1][0], tr.P[1][1] = R[0], R[1], R[1], R[2]
                tr.cam_only_born = True
                tr.hit("C", t, 10.0)
                self.next_id += 1
                self.counts["births"] += 1
                self.tracks.append(tr)

    def ais(self, targets, t):
        """ais.AisTargets.list() entries (lat / lon predicted to t, sog kn, cog, length, beam, stale, mmsi). A target
        is linked to a range track within assoc_m with a similar velocity (adds A + the MMSI + class), else it is an
        AIS-only track (never steer_ok). A linked AIS position > disagree_m from its range track = a conflict (U6)."""
        if self.origin is None:
            return
        c = self.c["ais"]
        t = float(t)
        if not self._time_ok(t):
            return
        self.T = max(self.T, t)
        for tg in targets or ():
            try:
                if tg.get("stale") or tg.get("lat") is None or tg.get("lon") is None:
                    continue
                mmsi = tg["mmsi"]
                x, y = self.xy(float(tg["lat"]), float(tg["lon"]))
                sog = float(tg.get("sog") or 0.0) * 0.514444
                cog = math.radians(float(tg.get("cog") or 0.0))
                vx, vy = sog * math.sin(cog), sog * math.cos(cog)
            except (KeyError, TypeError, ValueError):
                self.counts["errors"] += 1
                continue
            if not all(math.isfinite(v) for v in (x, y, vx, vy)):
                self.counts["errors"] += 1
                continue
            self.counts["ais"] += 1
            linked = [tr for tr in self.tracks if tr.mmsi == mmsi]
            rt = next((tr for tr in linked if tr.ranged()), None)
            for tr in linked:                     # review H1: a range-history track is never AIS's to remove /
                if tr is not rt and tr.has_range():   # overwrite: a tentative range track only loses the MMSI
                    tr.mmsi, tr.disagree = None, None
            linked = [tr for tr in linked if not tr.has_range()]     # the AIS-only twins (removable)
            if rt is not None and not rt.steer_ok:   # a coasting linked track: hand the MMSI to a steer_ok track
                alt = self._ais_best(x, y, vx, vy, mmsi, c, steer_only=True)  # right at the report, if one is
                if alt is not None and alt[0] is not rt:
                    rt.mmsi, rt.disagree = None, None
                    rt = None
            if rt is not None:
                dd = math.hypot(rt.s[0] - x, rt.s[1] - y)
                if dd > float(c["unlink_m"]):
                    rt.mmsi, rt.disagree = None, None      # not the same object any more (the track stays)
                    rt = None
                else:
                    rt.disagree = dd
                    rt.ais_xy = (x, y)
                    rt.hit("A", t, 10.0)
                    for tr in linked:                         # an AIS-only twin is no longer needed
                        self.tracks.remove(tr)
                        self.counts["merged"] += 1
                    continue
            bb = self._ais_best(x, y, vx, vy, mmsi, c, steer_only=False)
            best, bd = bb if bb is not None else (None, None)
            if best is not None:
                best.mmsi, best.ais_xy, best.disagree = mmsi, (x, y), bd
                best.hit("A", t, 10.0)
                for tr in linked:
                    self.tracks.remove(tr)
                    self.counts["merged"] += 1
                continue
            sd = float(c["sd_m"])
            tr = linked[0] if linked else None
            if tr is None:
                tr = Track(self.next_id, t, x, y, vx, vy, sd_p=sd, sd_v=1.0)
                tr.mmsi = mmsi
                self.next_id += 1
                self.counts["births"] += 1
                self.tracks.append(tr)
            else:
                tr.s = [x, y, vx, vy]
                tr.t = t
            tr.ais_xy = (x, y)
            ln, bm = tg.get("length"), tg.get("beam")
            if isinstance(ln, (int, float)) and ln > 0:
                tr.len_m = float(ln)
            if isinstance(bm, (int, float)) and bm > 0:
                tr.beam_m = float(bm)
            tr.hit("A", t, 10.0)

    def _ais_best(self, x, y, vx, vy, mmsi, c, steer_only):
        best = None
        for tr in self.tracks:
            if not tr.ranged() or (tr.mmsi is not None and tr.mmsi != mmsi) or (steer_only and not tr.steer_ok):
                continue
            dd = math.hypot(tr.s[0] - x, tr.s[1] - y)
            dv = math.hypot(tr.s[2] - vx, tr.s[3] - vy)
            if dd <= float(c["assoc_m"]) and dv <= float(c["assoc_v_mps"]) and (best is None or dd < best[1]):
                best = (tr, dd)
        return best

    # --- the cycle -------------------------------------------------------------------------------------------------
    def cycle(self, health=None, ages=None, skews=None, attitude=None, no_pose=False, error=False, now=None):
        """Maintenance + flags after the batches of a gateway tick. health {radar: state, lidar: state} (the
        HealthJudge's), ages {kind: s since the last batch (monotonic)}, skews {kind: s}, attitude (roll, pitch) or
        None, now = the gateway's epoch clock (the tracker clock never goes back: a stopped data flow never leaves a
        track steer_ok)."""
        c = self.c
        self.attitude = attitude
        if now is not None:                  # the gateway clock: with NO data at all the tracks still age out
            # (the manager passes its MONOTONIC clock and maps every measurement onto it - review H3)
            self.T = max(self.T, float(now))
        self.cycles += 1
        T = self.T
        rc, lc = c["radar"], c["lidar"]
        keep = []
        for tr in self.tracks:
            dl = []
            if tr.last["R"] is not None:
                dl.append(tr.last["R"] + float(rc["delete_s"]))
            if tr.last["L"] is not None:
                dl.append(tr.last["L"] + float(lc["delete_s"]))
            if dl:
                dead = T > max(dl)
                for k in ("C", "A"):             # a camera / AIS source keeps it as long as it reports
                    if tr.last[k] is not None and T - tr.last[k] <= float(c["camera" if k == "C" else "ais"]
                                                                         ["delete_s"]):
                        dead = False
                if not tr.confirmed:
                    win = max(float(rc["window_s"]) if tr.last["R"] is not None else 0.0,
                              float(lc["window_s"]) if tr.last["L"] is not None else 0.0)
                    if T - tr.born > float(c["tentative_s"]) + win:
                        dead = True
            else:
                lastc = max([x for x in (tr.last["C"], tr.last["A"]) if x is not None] or [tr.born])
                dead = T - lastc > max(float(c["camera"]["delete_s"]), float(c["ais"]["delete_s"]))
            if dead:
                self.counts["deleted"] += 1
            else:
                keep.append(tr)
        self.tracks = keep
        self._merge()
        own = self.own
        for tr in self.tracks:
            if not tr.confirmed:
                for k in RANGE_SRC:
                    sc = self._src_cfg(k)
                    if tr.n_hits(k, T, float(sc["window_s"])) >= int(sc["confirm_n"]):
                        tr.confirmed = True
            r_ok = tr.confirmed and tr.last["R"] is not None and T - tr.last["R"] <= float(rc["fresh_s"]) and \
                tr.n_hits("R", T, float(rc["window_s"])) >= int(rc["range_ok_n"])
            l_ok = tr.confirmed and tr.last["L"] is not None and T - tr.last["L"] <= float(lc["fresh_s"]) and \
                tr.n_hits("L", T, float(lc["window_s"])) >= int(lc["range_ok_n"])
            tr.steer_ok = steer_ok_from(tr, T, c)
            tr.coasting = tr.confirmed and tr.has_range() and not (r_ok or l_ok)
            sp = math.hypot(tr.s[2], tr.s[3])
            if tr.v_ema is None:
                tr.v_ema, tr.v_t = sp, T
            elif T > tr.v_t:
                a = min(1.0, (T - tr.v_t) / float(c["static_tau_s"]))
                tr.v_ema, tr.v_t = tr.v_ema + a * (sp - tr.v_ema), T
            lim = float(c["static_off_v"]) if tr.slow_since is not None else float(c["static_v"])
            if tr.v_ema < lim and tr.has_range():        # hysteresis: static until the average passes 0.5 m/s
                tr.slow_since = T if tr.slow_since is None else tr.slow_since
            else:
                tr.slow_since = None
            tr.in_static = bool(self.static is not None and self.static.occupied(tr.s[0], tr.s[1], 0.5 * tr.len_m))
            # static: the centroid inside a CHART pier / shore cell (nothing moving can be there - a long quay's
            # returns slide along it), or an accumulated LiDAR cell while slow, or slow for static_s
            chart_in = bool(self.static is not None and self.static.in_chart(tr.s[0], tr.s[1]))
            tr.static = (chart_in and sp < 1.5) or (tr.in_static and sp < 0.5) or (tr.slow_since is not None and
                                                                   T - tr.slow_since >= float(c["static_s"]))
            tr.cls = classify(tr)
            tr.conf = confidence(tr, T)
            if self.static is not None and tr.static and tr.steer_ok and tr.last["L"] is not None and \
                    T - tr.last["L"] < 0.5:
                self.static.observe(tr.s[0], tr.s[1], max(tr.len_m, 0.5), max(tr.beam_m, 0.5), T)
        self._ghosts()
        self._flags = self._make_flags(health or {}, ages or {}, skews or {}, no_pose, error)
        return self._flags

    def _src_cfg(self, k):
        return self.c[{"R": "radar", "L": "lidar", "C": "camera", "A": "ais"}[k]]

    def _merge(self):
        c = self.c
        mm, mv = float(c["merge_m"]), float(c["merge_v"])
        n = len(self.tracks)
        if n < 2:
            return
        drop = set()
        trs = sorted(self.tracks, key=lambda t: (t.born, t.id))
        cell = max(mm * 4, 5.0)
        grid = {}
        for i, tr in enumerate(trs):
            grid.setdefault((int(tr.s[0] // cell), int(tr.s[1] // cell)), []).append(i)
        for i, a in enumerate(trs):
            if i in drop or not a.has_range():
                continue
            gx, gy = int(a.s[0] // cell), int(a.s[1] // cell)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in grid.get((gx + dx, gy + dy), ()):
                        if j <= i or j in drop:
                            continue
                        b = trs[j]
                        if not b.has_range():
                            continue
                        if math.hypot(a.s[0] - b.s[0], a.s[1] - b.s[1]) > max(mm, 0.25 * min(a.len_m, b.len_m) + mm):
                            continue
                        if math.hypot(a.s[2] - b.s[2], a.s[3] - b.s[3]) > mv:
                            continue
                        # review M4: both updated by the SAME batch = two objects seen at once (rafted boats)
                        if set(a.hits["R"]) & set(b.hits["R"]) or set(a.hits["L"]) & set(b.hits["L"]):
                            continue
                        # b joins a (the older): every hit kept
                        for k in SRC_ALL:
                            a.hits[k] = sorted(a.hits[k] + b.hits[k])
                            if b.last[k] is not None:
                                a.last[k] = b.last[k] if a.last[k] is None else max(a.last[k], b.last[k])
                        a.confirmed = a.confirmed or b.confirmed
                        a.len_m, a.beam_m = max(a.len_m, b.len_m), max(a.beam_m, b.beam_m)
                        a.lidar_n += b.lidar_n
                        a.surface_n += b.surface_n
                        if a.mmsi is None:
                            a.mmsi = b.mmsi
                        drop.add(j)
        if drop:
            self.counts["merged"] += len(drop)
            self.tracks = [t for i, t in enumerate(trs) if i not in drop]

    def _ghosts(self):
        """A radar multipath ghost sits on the bearing of a stronger nearer track at about twice its range. KEPT and
        still steer_ok when radar-confirmed (conservative: never a removed obstacle); only marked."""
        c = self.c
        if self.own is None:
            return
        ox, oy, _ = self.own
        pol = []
        for tr in self.tracks:
            dx, dy = tr.s[0] - ox, tr.s[1] - oy
            pol.append((tr, math.hypot(dx, dy), math.degrees(math.atan2(dx, dy)) % 360.0))
        for tr, r, b in pol:
            tr.ghost = False
            if tr.last["L"] is not None or tr.last["A"] is not None or tr.last["R"] is None or r < 10.0:
                continue
            for o, r2, b2 in pol:
                if o is tr or r2 < 5.0:
                    continue
                if abs((b - b2 + 540.0) % 360.0 - 180.0) <= float(c["ghost_brg_deg"]) and \
                        abs(r - 2.0 * r2) <= float(c["ghost_frac"]) * r:
                    tr.ghost = True
                    break

    # --- flags (the planner's unsure inputs) --------------------------------------------------------------------
    def _make_flags(self, health, ages, skews, no_pose, error):
        c = self.c
        f = {"radar": health.get("radar", "OFF"), "lidar": health.get("lidar", "OFF")}
        stale = {}
        usable = {}
        for k in ("radar", "lidar"):
            st = health.get(k, "OFF")
            age = ages.get(k)
            lim = float(c[k]["stale_s"])
            if st in ("OK", "DEGRADED") and (age is None or age > lim):
                stale[k] = None if age is None else round(age, 2)
            usable[k] = st in ("OK", "DEGRADED") and k not in stale
        f["range_lost"] = not usable["radar"] and not usable["lidar"]
        f["degraded"] = sorted(k for k in ("radar", "lidar") if health.get(k, "OFF") != "OK" or k in stale)
        f["stale"] = stale
        f["time_skew"] = sorted(k for k, v in skews.items() if v is not None and abs(v) > 0.1)
        f["conflicts"] = self.conflicts()
        own = self.own
        nmov = 0
        if own is not None:
            for tr in self.tracks:
                if tr.steer_ok and not tr.static and math.hypot(tr.s[0] - own[0], tr.s[1] - own[1]) <= \
                        float(c["crowded_r"]):
                    nmov += 1
        f["crowded"] = nmov > int(c["crowded_n"])
        nb = sum(1 for x in self.near_births if self.T - x <= 2.0) / 2.0
        f["clutter"] = nb > float(c["clutter_births_s"])
        f["births_near_s"] = nb
        f["n_near_moving"] = nmov
        f["no_static_map"] = self.static is None or not self.static.has_data()
        f["no_pose"] = bool(no_pose) or self.own is None
        ov = []
        if len(self.tracks) > int(c["max_tracks"]):
            ov.append("tracks %d > %d" % (len(self.tracks), int(c["max_tracks"])))
        hold = float(c["err_hold_s"])
        if self.counts["dropped_cap"] != self.cap_seen:
            self.cap_seen, self.cap_until = self.counts["dropped_cap"], self.T + hold
        if self.cap_until is not None and self.T <= self.cap_until:
            ov.append("sensor items over the cap")
        if self.counts["errors"] != self.err_seen:
            self.err_seen, self.err_until = self.counts["errors"], self.T + hold
        f["input_errors"] = self.err_until is not None and self.T <= self.err_until
        last = self.timing[-20:]
        if last and max(last) > float(c["budget_ms"]):
            ov.append("cycle %.0f ms > %.0f ms" % (max(last), float(c["budget_ms"])))
        f["overload"] = ov
        f["error"] = bool(error)
        f["ghost_suspect"] = sorted(tr.id for tr in self.tracks if tr.ghost)
        f["attitude"] = "imu" if self.attitude is not None else "none"
        u = []
        if f["range_lost"]:
            u.append("U1 range sensors lost")
        for k in f["degraded"]:
            u.append("U2 %s %s" % (k, "stale" if k in stale else health.get(k, "OFF")))
        if f["conflicts"]:
            u.append("U6 conflicting tracks (%d)" % len(f["conflicts"]))
        if f["clutter"]:
            u.append("U9 clutter (%.0f new tracks / s within %.0f m)" % (nb, float(c["clutter_r_m"])))
        if f["crowded"]:
            u.append("U9 crowded (%d moving range tracks within %.0f m)" % (nmov, float(c["crowded_r"])))
        if f["no_static_map"]:
            u.append("U10 no static map")
        for k in f["time_skew"]:
            u.append("U11 %s time skew" % k)
        if f["no_pose"]:
            u.append("no own pose")
        if f["input_errors"]:
            u.append("input rejected (no pose / heading, a time jump or a bad item): %d in all"
                     % self.counts["rejected"])
        for w in ov:
            u.append("tracker overload: " + w)
        if error:
            u.append("tracker error")
        f["unsure"] = u
        return f

    def conflicts(self):
        """U6: a linked AIS position > disagree_m from its range track (object < near_m from us); an AIS-only track
        < near_m with no range track near it; a camera-only track ahead < camera_near_m with no range return."""
        c = self.c["conflict"]
        out = []
        if self.own is None:
            return out
        ox, oy, hdg = self.own
        near, dis = float(c["near_m"]), float(c["disagree_m"])
        for tr in self.tracks:
            r = math.hypot(tr.s[0] - ox, tr.s[1] - oy)
            if tr.steer_ok and tr.disagree is not None and tr.disagree > dis and r < near and \
                    tr.last["A"] is not None and self.T - tr.last["A"] < 10.0:
                out.append({"kind": "ais_range_disagree", "ids": [tr.id], "m": round(tr.disagree, 1)})
            elif not tr.steer_ok and tr.last["A"] is not None and self.T - tr.last["A"] < 10.0 and r < near:
                out.append({"kind": "ais_without_range", "ids": [tr.id], "m": round(r, 1)})
            elif not tr.steer_ok and tr.last["C"] is not None and self.T - tr.last["C"] < 5.0:
                rel = abs(((math.degrees(math.atan2(tr.s[0] - ox, tr.s[1] - oy)) - hdg) + 540.0) % 360.0 - 180.0)
                if r < float(c["camera_near_m"]) and rel <= float(c["camera_ahead_deg"]):
                    out.append({"kind": "camera_without_range", "ids": [tr.id], "m": round(r, 1)})
        return out

    # --- output ------------------------------------------------------------------------------------------------------
    def flags(self):
        return self._flags

    def tracks_out(self):
        T = self.T
        out = []
        for tr in sorted(self.tracks, key=lambda t: t.id):
            lat = lon = None
            if self.origin is not None:
                lat, lon = simworld.xy_to_ll(self.origin[0], self.origin[1], tr.s[0], tr.s[1])
            sp = math.hypot(tr.s[2], tr.s[3])
            out.append({"id": tr.id, "t": round(tr.t, 3), "x": round(tr.s[0], 2), "y": round(tr.s[1], 2),
                        "lat": lat, "lon": lon, "vx": round(tr.s[2], 3), "vy": round(tr.s[3], 3),
                        "speed": round(sp, 3), "course": round(math.degrees(math.atan2(tr.s[2], tr.s[3])) % 360.0, 1),
                        "P": [[round(v, 4) for v in row] for row in tr.P],
                        "sd_m": round(math.sqrt(max(0.0, 0.5 * (tr.P[0][0] + tr.P[1][1]))), 2),
                        "len_m": round(tr.len_m, 2), "beam_m": round(tr.beam_m, 2),
                        "hgt_m": None if tr.hgt_m is None else round(tr.hgt_m, 2), "cls": tr.cls,
                        "conf": round(tr.conf, 2),
                        "src": "".join(k for k in SRC_ALL if tr.last[k] is not None and T - tr.last[k] <= 5.0),
                        "last": {k: (None if v is None else round(v, 3)) for k, v in tr.last.items()},
                        "hits_R": tr.n_hits("R", T, float(self.c["radar"]["window_s"])),
                        "hits_L": tr.n_hits("L", T, float(self.c["lidar"]["window_s"])),
                        "confirmed": tr.confirmed, "steer_ok": tr.steer_ok, "range_ok": tr.steer_ok,
                        "coasting": tr.coasting, "static": tr.static, "in_static_map": tr.in_static,
                        "surface": tr.lidar_n > 0 and tr.surface_n * 2 >= tr.lidar_n, "ghost_suspect": tr.ghost,
                        "mmsi": tr.mmsi, "age_s": round(T - tr.born, 2), "born": round(tr.born, 3)})
        return out

    def summary(self):
        tm = sorted(self.timing)
        lat = sorted(self.latency)

        def pct(v, p):
            return None if not v else round(v[min(len(v) - 1, int(len(v) * p))], 2)
        f = self._flags or {}
        return {"tracks": len(self.tracks), "steer_ok": sum(1 for t in self.tracks if t.steer_ok),
                "confirmed": sum(1 for t in self.tracks if t.confirmed),
                "cycle_ms": {"p50": pct(tm, 0.5), "p99": pct(tm, 0.99), "max": tm[-1] if tm else None},
                "latency_ms": {"p50": None if not lat else round(pct(lat, 0.5) * 1000.0, 1),
                               "p99": None if not lat else round(pct(lat, 0.99) * 1000.0, 1)},
                "unsure": list(f.get("unsure") or []), "counts": dict(self.counts)}

    def note_ms(self, ms):
        self.timing.append(ms)
        del self.timing[:-2000]


# --- pure rules (each one used by the tracker; the tests check them directly + by mutation) -------------------------
def steer_ok_from(tr, T, c):
    """M0-3: the ONLY place steer_ok is decided - a confirmed track with fresh hits from a RANGE sensor (RANGE_SRC);
    camera / AIS hits never count."""
    if not tr.confirmed:
        return False
    for k in RANGE_SRC:
        sc = c[{"R": "radar", "L": "lidar", "C": "camera", "A": "ais"}[k]]
        win = float(sc.get("window_s", 1.0))
        fresh = float(sc.get("fresh_s", 0.5))
        need = int(sc.get("range_ok_n", 3))
        if tr.last[k] is not None and T - tr.last[k] <= fresh and tr.n_hits(k, T, win) >= need:
            return True
    return False


def classify(tr):
    size = max(tr.len_m, tr.beam_m)
    sp = math.hypot(tr.s[2], tr.s[3])
    low = tr.hgt_m is not None and tr.hgt_m <= 0.6
    flat = tr.lidar_n > 0 and tr.surface_n * 2 >= tr.lidar_n
    if tr.mmsi is not None:
        return "vessel"
    if not tr.ranged():
        return "unknown"
    if size < 1.5 and tr.hgt_m is not None:
        if low or flat:
            return "person"
        if tr.static:
            return "buoy"
        return "unknown"
    if size < 1.5:
        return "buoy" if tr.static else "unknown"
    if size < 6.0 and (sp >= 0.3 or low or flat):
        return "small"
    if tr.static:
        return "static"
    if sp >= 0.5:
        return "vessel" if size >= 6.0 else "small"
    return "unknown"


def confidence(tr, T):
    w = {"R": 0.5, "L": 0.6, "C": 0.25, "A": 0.35}
    miss = 1.0
    for k, v in w.items():
        if tr.last[k] is not None and T - tr.last[k] <= 5.0:
            miss *= 1.0 - v
    c = 1.0 - miss
    if not tr.confirmed:
        c *= 0.5
    return max(0.0, min(1.0, c))
