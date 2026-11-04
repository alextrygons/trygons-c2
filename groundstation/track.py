"""The boat's track on THIS PC (step 13n3, owner 2026-10-31 E3) - DISPLAY ONLY: nothing here sends anything to
the boat or the gateway, nothing here changes how the boat is driven.

  * ONE live track PER BOAT (the key = the boat's name; a typed address without a name = the address): switching
    boats shows that boat's own track, never a mix
  * no point limit: every point is kept (time, position, speed, heading); the drawing uses a simplified copy only
    (simplified(), runs())
  * written to disk continuously (crash-safe): one JSON line per point, flushed at once, fsync every FSYNC_S; at the
    next start the file is read back (a cut last line is dropped), the track goes on
  * NEW TRACK: the live track is saved as <date>_<HHMM>_<boat>.jsonl (the time of its first point) and a new one
    starts; saved tracks are listed (date, boat, duration, nm), loaded for the map, exported / imported as GPX 1.1
  * two stations of one PC on the same boat: the first one writes the file (a lock), the other keeps its track in
    RAM only (Live.shared) - never two writers of one file

Folder: TRYGONS_TRACKS=<folder>; else, while TRYGONS_TEST is set (the tests), a temporary folder of this process;
else groundstation/tracks/ (not in git). Inside: live/<key>.jsonl (+ .lock), saved/*.jsonl.
"""
import atexit
import datetime
import json
import math
import os
import re
import shutil
import tempfile
import time
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
FORMAT = 1
MIN_S = 1.0                 # at most one point per second ...
MIN_M = 2.0                 # ... once the boat moved this far from the last point
SLOW_S = 10.0               # (slow drift: a point every SLOW_S once it moved SLOW_M)
SLOW_M = 0.3
FSYNC_S = 5.0               # the file is flushed at every point and fsync'ed this often
GPX_MAX_BYTES = 64 * 1024 * 1024
GPX_MAX_POINTS = 200000      # review 13n3 MED-B: an import never makes a track the map can't draw quickly
T_MAX = 4102444800.0        # 2100-01-01: later times are refused (Windows fromtimestamp, review 13n3 L5)
CHUNK = 256                 # points per bounding box (the map skips whole chunks outside the window)
MAX_DRAW = 20000            # at most this many points drawn per track (more = a coarser line)
LIVE_RAM_MAX = 400000       # the live track keeps at most this many points in RAM (the FILE keeps every one)
GPX_NS = "urn:trygons:track:1"
M_PER_DEG = 111320.0
_tmp_root = None


def root_dir():
    global _tmp_root
    env = os.environ.get("TRYGONS_TRACKS")
    if env:
        return env
    if os.environ.get("TRYGONS_TEST"):                   # a test run: never this PC's real tracks (review 13n3 L7:
        #                                                   a user's own TRYGONS_GS_SETTINGS never moves the tracks)
        if _tmp_root is None:
            _tmp_root = tempfile.mkdtemp(prefix="trygons_tracks_")
            atexit.register(shutil.rmtree, _tmp_root, True)
        return _tmp_root
    return os.path.join(HERE, "tracks")


def safe_name(s, default="BOAT"):
    """A file-name part: letters (any language), digits, '-', '_' and '.'; the rest -> '-'."""
    s = re.sub(r"[^\w.\-]+", "-", str(s or ""), flags=re.UNICODE).strip("-.")
    return s[:60] or default


def _num(v, lo, hi):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) and lo <= v <= hi else None


def valid_point(p):
    """(t, lat, lon, kn, hdg): lat / lon in range and finite; t finite or None (an imported GPX without times);
    kn / hdg None or in range. -> the cleaned tuple or None."""
    try:
        t, la, lo, kn, hd = p
    except (TypeError, ValueError):
        return None
    la, lo = _num(la, -90.0, 90.0), _num(lo, -180.0, 180.0)
    if la is None or lo is None:
        return None
    if t is not None:
        t = _num(t, 0.0, T_MAX)
        if t is None:
            return None
    return (t, la, lo, _num(kn, 0.0, 100.0) if kn is not None else None,
            _num(hd, 0.0, 360.0) if hd is not None else None)


def dist_m(la1, lo1, la2, lo2):
    k = math.cos(math.radians((la1 + la2) / 2.0))
    return math.hypot((lo2 - lo1) * k * M_PER_DEG, (la2 - la1) * M_PER_DEG)


class Track:
    """The points of one track: pts = [(t, lat, lon, kn, hdg)], latlon = [(lat, lon)] (the map's view)."""

    def __init__(self, boat="", key="", path=None, imported=False):
        self.boat, self.key, self.path, self.imported = boat, key, path, imported
        self.pts = []
        self.latlon = []
        self._simp = {}
        self._nm = (0, 0.0)
        self._box = []                  # per CHUNK points: [la_min, la_max, lo_min, lo_max]
        self.dropped = 0                # points no longer in RAM (the oldest; the file keeps them)

    def add(self, p):
        i = len(self.pts)
        self.pts.append(p)
        self.latlon.append((p[1], p[2]))
        if i % CHUNK == 0:
            self._box.append([p[1], p[1], p[2], p[2]])
        else:
            b = self._box[-1]
            b[0], b[1], b[2], b[3] = min(b[0], p[1]), max(b[1], p[1]), min(b[2], p[2]), max(b[3], p[2])

    def set_points(self, pts):
        self.clear()
        for p in pts:
            self.add(p)

    def trim(self, keep):
        """Only the newest `keep` points stay in RAM (whole chunks dropped; the caller's file keeps them all)."""
        if len(self.pts) <= keep:
            return 0
        n = ((len(self.pts) - keep) // CHUNK + 1) * CHUNK
        rest = self.pts[n:]
        dropped = self.dropped + n
        self.set_points(rest)
        self.dropped = dropped
        return n

    def clear(self):
        self.pts, self.latlon, self._simp, self._nm, self._box = [], [], {}, (0, 0.0), []
        self.dropped = 0

    def __len__(self):
        return len(self.pts)

    def nm(self):
        n0, m = self._nm
        for i in range(max(1, n0), len(self.pts)):
            a, b = self.pts[i - 1], self.pts[i]
            m += dist_m(a[1], a[2], b[1], b[2])
        self._nm = (len(self.pts), m)
        return m / 1852.0

    def times(self):
        ts = [p[0] for p in self.pts if p[0] is not None]
        return (ts[0], ts[-1]) if ts else (None, None)

    def stats(self):
        t0, t1 = self.times()
        return {"n": len(self.pts), "start": t0, "end": t1,
                "duration_s": (t1 - t0) if t0 is not None else None, "nm": self.nm()}

    def simplified(self, tol_m):
        """Indices of the points kept for drawing at this tolerance (metres): a point is kept once it is tol_m from
        the last kept one (incremental: the new points only), the last point always. Tolerances are rounded to a
        power of two (a cache per level)."""
        lv = 2.0 ** round(math.log2(max(0.25, tol_m)))
        c = self._simp.get(lv)
        if c is None:
            c = self._simp[lv] = [0, []]
        n0, kept = c
        pts = self.pts
        for i in range(n0, len(pts)):
            if not kept:
                kept.append(i)
                continue
            q = pts[kept[-1]]
            if dist_m(q[1], q[2], pts[i][1], pts[i][2]) >= lv:
                kept.append(i)
        c[0] = len(pts)
        if kept and kept[-1] != len(pts) - 1:
            return kept + [len(pts) - 1]
        return kept

    def visible(self, P, w, h, m_per_px, tol_px=1.5):
        """(simplified indices of the chunks that touch the window + one window of margin, as runs of consecutive
        indices) - only these points are projected: a long track far outside the window costs one box test per
        CHUNK points (review 13n3 MED-B). At most MAX_DRAW points (more = every k-th, the last one of each run kept)."""
        import bisect
        idx = self.simplified(m_per_px * tol_px)
        if not idx:
            return []
        x0, y0, x1, y1 = -w, -h, 2 * w, 2 * h
        runs, cur_lo = [], None
        nb = len(self._box)
        for k in range(nb + 1):
            on = False
            if k < nb:
                la0, la1, lo0, lo1 = self._box[k]
                xs, ys = zip(*(P(la, lo) for la in (la0, la1) for lo in (lo0, lo1)))
                on = not (max(xs) < x0 or min(xs) > x1 or max(ys) < y0 or min(ys) > y1)
            if on and cur_lo is None:
                cur_lo = k
            elif not on and cur_lo is not None:
                a = max(0, cur_lo * CHUNK - 1)                       # the segment into the chunk too
                b = min(len(self.pts) - 1, k * CHUNK)                # ... and out of it
                i0, i1 = bisect.bisect_left(idx, a), bisect.bisect_right(idx, b)
                sel = idx[max(0, i0 - 1):min(len(idx), i1 + 1)]
                if sel:
                    runs.append(sel)
                cur_lo = None
        total = sum(len(r) for r in runs)
        if total > MAX_DRAW:
            step = total // MAX_DRAW + 1
            runs = [r[::step] + ([r[-1]] if (len(r) - 1) % step else []) for r in runs]
        return runs

    def runs(self, P, w, h, m_per_px, tol_px=1.5):
        """The polylines to draw [[x0, y0, x1, y1, ...]]: the simplified points of the visible chunks only, only the
        segments that touch the window (+ one window of margin) - however long the track, the drawing stays small."""
        x0, y0, x1, y1 = -w, -h, 2 * w, 2 * h
        out = []
        for sel in self.visible(P, w, h, m_per_px, tol_px):
            out += self._cut([P(*self.latlon[i]) for i in sel], x0, y0, x1, y1)
        return out

    @staticmethod
    def _cut(xy, x0, y0, x1, y1):
        out, cur = [], []
        for i in range(len(xy) - 1):
            (ax, ay), (bx, by) = xy[i], xy[i + 1]
            if max(ax, bx) < x0 or min(ax, bx) > x1 or max(ay, by) < y0 or min(ay, by) > y1:
                if len(cur) >= 4:
                    out.append(cur)
                cur = []
                continue
            if not cur:
                cur = [ax, ay]
            cur += [bx, by]
        if len(cur) >= 4:
            out.append(cur)
        return out

    def nearest(self, P, x, y, max_px, m_per_px):
        """The place on the line nearest to the pixel (x, y), within max_px: {"lat", "lon", "t", "kn", "hdg", "px"}
        (time + speed interpolated along the segment) or None. Only the chunks near the click are looked at (review
        13n3 MED-B)."""
        if len(self.pts) < 1:
            return None
        w = h = max(1.0, 4.0 * max_px)
        PP = lambda la, lo: (lambda q: (q[0] - x + w / 2, q[1] - y + h / 2))(P(la, lo))     # noqa: E731
        runs = self.visible(PP, w, h, m_per_px)
        if not runs:
            return None
        best = None                                     # (d, run, j) on the simplified line
        for r in runs:
            if len(r) == 1:
                d = math.hypot(*(a - b for a, b in zip(P(*self.latlon[r[0]]), (x, y))))
                if best is None or d < best[0]:
                    best = (d, r, None)
                continue
            xy = [P(*self.latlon[i]) for i in r]
            for jj in range(len(r) - 1):
                d = _seg_d(xy[jj], xy[jj + 1], x, y)[0]
                if d <= max_px * 3 and (best is None or d < best[0]):
                    best = (d, r, jj)
        if best is None:
            return None
        d0, r, jj = best
        if jj is None:
            if d0 > max_px:
                return None
            p = self.pts[r[0]]
            return {"lat": p[1], "lon": p[2], "t": p[0], "kn": p[3], "hdg": p[4], "px": d0}
        lo_i, hi_i = r[max(0, jj - 1)], r[min(len(r) - 1, jj + 2)]
        found = None
        prev = P(*self.latlon[lo_i])
        for i in range(lo_i + 1, hi_i + 1):
            cur = P(*self.latlon[i])
            d, u = _seg_d(prev, cur, x, y)
            if found is None or d < found[0]:
                found = (d, i - 1, u)
            prev = cur
        if found is None:
            return None
        d, i, u = found
        if d > max_px:
            return None
        a, b = self.pts[i], self.pts[i + 1]
        lerp = lambda va, vb: None if va is None or vb is None else va + u * (vb - va)      # noqa: E731
        return {"lat": a[1] + u * (b[1] - a[1]), "lon": a[2] + u * (b[2] - a[2]), "t": lerp(a[0], b[0]),
                "kn": lerp(a[3], b[3]) if a[3] is not None and b[3] is not None else (a[3] if u < 0.5 else b[3]),
                "hdg": a[4] if u < 0.5 else b[4], "px": d}


def _seg_d(a, b, x, y):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    u = 0.0 if L <= 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L))
    return math.hypot(ax + u * dx - x, ay + u * dy - y), u


# --- files -----------------------------------------------------------------------------------------------------
def _line(p):
    t, la, lo, kn, hd = p
    return json.dumps([None if t is None else round(t, 1), round(la, 7), round(lo, 7),
                       None if kn is None else round(kn, 2), None if hd is None else round(hd, 1)]) + "\n"


def _header(boat, key, imported=False):
    return json.dumps({"trygons_track": FORMAT, "boat": boat, "key": key, "imported": bool(imported)},
                      ensure_ascii=False) + "\n"


def read_file(path):
    """A track file -> Track (bad lines skipped, a cut last line dropped). OSError if it can't be read."""
    tr = Track(path=path)
    with open(path, "rb") as f:
        data = f.read()
    for raw in data.split(b"\n"):
        if not raw.strip():
            continue
        try:
            v = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            continue
        if isinstance(v, dict):
            if "trygons_track" in v:
                tr.boat, tr.key = str(v.get("boat") or ""), str(v.get("key") or "")
                tr.imported = bool(v.get("imported"))
            continue
        p = valid_point(v) if isinstance(v, list) else None
        if p is not None:
            tr.add(p)
    return tr


def read_header(path):
    """The header dict of a track file (its first line), {} without one."""
    try:
        with open(path, "rb") as f:
            v = json.loads(f.readline().decode("utf-8"))
        return v if isinstance(v, dict) and "trygons_track" in v else {}
    except (OSError, ValueError, UnicodeDecodeError):
        return {}


def _lock(path):
    """An exclusive lock (held while this process runs; the OS frees it when the process dies). -> file or None."""
    try:
        f = open(path, "a+b")
    except OSError:
        return None
    try:
        if os.name == "nt":
            import msvcrt
            f.seek(0)
            msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return f
    except OSError:
        f.close()
        return None


def _unlock(f):
    if f is None:
        return
    try:
        if os.name == "nt":
            import msvcrt
            f.seek(0)
            msvcrt.locking(f.fileno(), msvcrt.LK_UNLCK, 1)
    except OSError:
        pass
    f.close()


class Live:
    """The live track of ONE boat: restored from its file, every new point appended at once."""

    def __init__(self, root, key, boat):
        self.key, self.boat = key, boat
        d = os.path.join(root, "live")
        self.path = os.path.join(d, safe_name(key, "track") + ".jsonl")
        hk = read_header(self.path).get("key") if os.path.exists(self.path) else None
        if hk is not None and hk != key:        # another key with the same file name (review 13n3 L1): its own file
            import hashlib
            self.path = os.path.join(d, "%s_%s.jsonl" % (safe_name(key, "track"),
                                                         hashlib.sha1(key.encode("utf-8")).hexdigest()[:8]))
        self.error = None
        self.f = None
        self.lockf = None
        self.shared = False
        self._sync = 0.0
        self.track = Track(boat=boat, key=key, path=self.path)
        try:
            os.makedirs(d, exist_ok=True)
        except OSError as e:
            self.error = str(e)
            return
        self.lockf = _lock(self.path[:-6] + ".lock")
        if self.lockf is None:
            self.shared = True                  # another station of this PC writes it: RAM only here
        if os.path.exists(self.path):
            try:
                old = read_file(self.path)
                self.track.set_points(old.pts)
                self.track.trim(LIVE_RAM_MAX)
            except OSError as e:
                self.error = str(e)
        if not self.shared:
            self._open()

    def _open(self):
        try:
            if os.path.exists(self.path):
                with open(self.path, "rb+") as f:          # a cut last line (power loss) would glue onto the next
                    data = f.read()
                    if data and not data.endswith(b"\n"):
                        f.truncate(data.rfind(b"\n") + 1)
            new = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
            self.f = open(self.path, "a", encoding="utf-8")
            if new:
                self.f.write(_header(self.boat, self.key))
                self.f.flush()
        except OSError as e:
            self.error = str(e)
            self.f = None

    def last(self):
        return self.track.pts[-1] if self.track.pts else None

    def feed(self, t, lat, lon, kn=None, hdg=None):
        """A position (wall-clock t): recorded when it moved MIN_M (>= MIN_S since the last one) or SLOW_M after
        SLOW_S. -> True if a point was recorded."""
        p = valid_point((t, lat, lon, kn, hdg))
        if p is None or p[0] is None:
            return False
        q = self.last()
        if q is not None:
            dt = p[0] - (q[0] or 0.0)
            dd = dist_m(q[1], q[2], p[1], p[2])
            if dt < 0:                          # the PC clock stepped back (review 13n3 L4): the distance rule only
                if dd < MIN_M:
                    return False
            else:
                if dt < MIN_S:
                    return False
                if dd < MIN_M and not (dt >= SLOW_S and dd >= SLOW_M):
                    return False
        self.track.add(p)
        self._write(p)
        if len(self.track.pts) > LIVE_RAM_MAX + CHUNK:
            self.track.trim(LIVE_RAM_MAX)       # RAM bounded; the file keeps every point (review 13n3 MED-B)
        return True

    def _write(self, p):
        if self.f is None:
            return
        try:
            self.f.write(_line(p))
            self.f.flush()
            if time.time() - self._sync >= FSYNC_S:
                os.fsync(self.f.fileno())
                self._sync = time.time()
        except (OSError, ValueError) as e:
            self.error = str(e)             # the disk is full / gone: the track goes on in RAM, the station says so
            try:
                self.f.close()
            except OSError:
                pass
            self.f = None

    def close(self):
        if self.f is not None:
            try:
                self.f.flush()
                os.fsync(self.f.fileno())
                self.f.close()
            except (OSError, ValueError):
                pass
            self.f = None
        _unlock(self.lockf)
        self.lockf = None

    def save_as(self, saved_dir, boat_name):
        """NEW TRACK: this track -> saved_dir/<date>_<HHMM>_<boat>.jsonl (the time of its first point), then an
        empty live track. -> the saved path, or None (nothing to save). OSError if it can't be saved (then the
        live track is kept as it was)."""
        if not self.track.pts:
            return None
        os.makedirs(saved_dir, exist_ok=True)
        t0 = self.track.times()[0] or time.time()
        base = "%s_%s" % (datetime.datetime.fromtimestamp(t0).strftime("%Y-%m-%d_%H%M"), safe_name(boat_name))
        dst = os.path.join(saved_dir, base + ".jsonl")
        n = 2
        while os.path.exists(dst):
            dst = os.path.join(saved_dir, "%s_%d.jsonl" % (base, n))
            n += 1
        tmp = dst + ".part"
        try:
            with open(tmp, "w", encoding="utf-8") as f:        # written in full first, then renamed: never half
                f.write(_header(boat_name, self.key))
                if self.track.dropped and not self.shared and os.path.exists(self.path):
                    if self.f is not None:
                        self.f.flush()
                    old = read_file(self.path)                 # the RAM holds only the newest: every point from the file
                    for p in old.pts:
                        f.write(_line(p))
                else:
                    for p in self.track.pts:
                        f.write(_line(p))
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, dst)
        except OSError:
            _rm(tmp)
            raise
        if self.f is not None:
            try:
                self.f.close()
            except OSError:
                pass
            self.f = None
        if not self.shared:
            try:
                with open(self.path, "w", encoding="utf-8") as f:
                    f.write(_header(self.boat, self.key))
            except OSError as e:
                self.error = str(e)
        self.track.clear()
        if not self.shared:
            self._open()
        return dst


class TrackStore:
    """This PC's tracks: the live one of the boat shown now + the saved ones."""

    def __init__(self, root=None):
        self.root = root or root_dir()
        self.live = None
        self._saved = {}                    # path -> (mtime, size, Track)

    @property
    def saved_dir(self):
        return os.path.join(self.root, "saved")

    def select(self, key, boat):
        """The boat whose live track is shown / recorded (a change closes the other one's file)."""
        if self.live is not None and self.live.key == key:
            return self.live
        self.close()
        self.live = Live(self.root, key, boat)
        return self.live

    def close(self):
        if self.live is not None:
            self.live.close()
            self.live = None

    def latlon(self):
        return self.live.track.latlon if self.live is not None else []

    def feed_telem(self, d, now=None):
        """From one telemetry frame (pos_source 0 = no position: nothing)."""
        if self.live is None or not d or not d.get("pos_source"):
            return False
        kn = d.get("est_sog") if d.get("est_sog") is not None else d.get("gps_sog")
        return self.live.feed(time.time() if now is None else now, d.get("est_lat"), d.get("est_lon"), kn,
                              d.get("heading"))

    def new_track(self, boat_name):
        if self.live is None:
            return None
        return self.live.save_as(self.saved_dir, boat_name)

    def load(self, path):
        """A saved track (cached until the file changes)."""
        st = os.stat(path)
        c = self._saved.get(path)
        if c is not None and c[0] == st.st_mtime and c[1] == st.st_size:
            return c[2]
        tr = read_file(path)
        self._saved[path] = (st.st_mtime, st.st_size, tr)
        return tr

    def saved_list(self):
        """[{"path", "file", "boat", "start", "end", "duration_s", "nm", "n", "imported"}], newest first."""
        out = []
        try:
            names = os.listdir(self.saved_dir)
        except OSError:
            return out
        for fn in names:
            if not fn.endswith(".jsonl"):
                continue
            path = os.path.join(self.saved_dir, fn)
            try:
                tr = self.load(path)
            except OSError:
                continue
            s = tr.stats()
            s.update({"path": path, "file": fn[:-6], "boat": tr.boat or "?", "imported": tr.imported})
            out.append(s)
        out.sort(key=lambda s: (s["start"] or 0.0, s["file"]), reverse=True)
        return out

    def import_gpx(self, src):
        """A GPX file -> a saved track. -> the new path. ValueError (not a usable GPX) / OSError."""
        name, pts = gpx_read(src)
        os.makedirs(self.saved_dir, exist_ok=True)
        t0 = next((p[0] for p in pts if p[0] is not None), None)
        stamp = datetime.datetime.fromtimestamp(t0 if t0 is not None else time.time()).strftime("%Y-%m-%d_%H%M")
        base = "%s_%s" % (stamp, safe_name(name, "GPX"))
        dst = os.path.join(self.saved_dir, base + ".jsonl")
        n = 2
        while os.path.exists(dst):
            dst = os.path.join(self.saved_dir, "%s_%d.jsonl" % (base, n))
            n += 1
        tmp = dst + ".part"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(_header(name, "", imported=True))
                for p in pts:
                    f.write(_line(p))
            os.replace(tmp, dst)
        except OSError:
            _rm(tmp)
            raise
        return dst


# --- GPX 1.1 ---------------------------------------------------------------------------------------------------
def _iso(t):
    return datetime.datetime.fromtimestamp(min(max(t, 0.0), T_MAX), datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")


def gpx_write(track, dst, name=None):
    """GPX 1.1 (Google Earth, OpenCPN, plotters): one trk / trkseg, every point with its time; speed (kn) and
    heading in our own extension (other programs ignore it)."""
    name = name or track.boat or "track"
    tmp = dst + ".part"
    try:
        _gpx_body(track, tmp, name)
        os.replace(tmp, dst)
    except OSError:
        _rm(tmp)                                # never a leftover .part (review 13n3 L6)
        raise
    return dst


def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass


def _gpx_body(track, tmp, name):
    with open(tmp, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n'
                '<gpx version="1.1" creator="Trygons C2" xmlns="http://www.topografix.com/GPX/1/1" '
                'xmlns:trygons="%s">\n <trk>\n  <name>%s</name>\n  <trkseg>\n' % (GPX_NS, escape(name)))
        for t, la, lo, kn, hd in track.pts:
            f.write('   <trkpt lat="%.7f" lon="%.7f">' % (la, lo))
            if t is not None:
                f.write("<time>%s</time>" % _iso(t))
            if kn is not None or hd is not None:
                f.write("<extensions>")
                if kn is not None:
                    f.write("<trygons:sog_kn>%.2f</trygons:sog_kn>" % kn)
                if hd is not None:
                    f.write("<trygons:hdg>%.1f</trygons:hdg>" % hd)
                f.write("</extensions>")
            f.write("</trkpt>\n")
        f.write("  </trkseg>\n </trk>\n</gpx>\n")


def _parse_time(s):
    s = (s or "").strip()
    if not s:
        return None
    try:
        if s.endswith("Z"):
            s = s[:-1] + "+00:00"
        dt = datetime.datetime.fromisoformat(s)
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=datetime.timezone.utc)
        return dt.timestamp()
    except ValueError:
        return None


def _local(tag):
    return tag.rsplit("}", 1)[-1]


def gpx_read(src):
    """A GPX file -> (name, [(t, lat, lon, kn, hdg)]): every trkpt of every trk / trkseg (bad points skipped).
    Speed: our extension, a GPX 1.0 <speed> (m/s) or from the times; heading: our extension / <course>.
    ValueError(code): "too_big", "dtd" (a DTD / entity: refused), "not_xml", "too_many", "no_points"."""
    if os.path.getsize(src) > GPX_MAX_BYTES:
        raise ValueError("too_big")
    from xml.parsers import expat
    st = {"name": None, "cur": None, "text": [], "depth_name": False}
    pts = []

    def refuse(*_a):
        raise ValueError("dtd")         # a DTD / an entity anywhere (any encoding, any offset): refused by the parser

    def start(tag, attrs):
        tag = _local(tag)
        st["text"] = []
        if tag == "trkpt":
            st["cur"] = {"lat": attrs.get("lat"), "lon": attrs.get("lon")}

    def chars(data):
        if len(st["text"]) < 64:
            st["text"].append(data)

    def end(tag):
        tag = _local(tag)
        text = "".join(st["text"]).strip()[:200]
        st["text"] = []
        cur = st["cur"]
        if cur is not None and tag in ("time", "sog_kn", "speed", "hdg", "course"):
            cur[tag] = text
        elif tag == "name" and st["name"] is None and cur is None:
            st["name"] = text or None
        elif tag == "trkpt" and cur is not None:
            kn = _num(cur.get("sog_kn"), 0.0, 100.0)
            if kn is None and cur.get("speed"):
                v = _num(cur.get("speed"), 0.0, 60.0)
                kn = None if v is None else v * 3600.0 / 1852.0
            hd = _num(cur.get("hdg"), 0.0, 360.0)
            if hd is None:
                hd = _num(cur.get("course"), 0.0, 360.0)
            p = valid_point((_parse_time(cur.get("time")), cur.get("lat"), cur.get("lon"), kn, hd))
            if p is not None:
                pts.append(p)
                if len(pts) > GPX_MAX_POINTS:
                    raise ValueError("too_many")
            st["cur"] = None

    ps = expat.ParserCreate(namespace_separator="}")
    ps.StartDoctypeDeclHandler = refuse
    ps.EntityDeclHandler = refuse
    ps.UnparsedEntityDeclHandler = refuse
    ps.ExternalEntityRefHandler = refuse
    ps.SetParamEntityParsing(expat.XML_PARAM_ENTITY_PARSING_NEVER)
    ps.StartElementHandler, ps.EndElementHandler, ps.CharacterDataHandler = start, end, chars
    ps.buffer_text = True
    try:
        with open(src, "rb") as f:
            while True:
                chunk = f.read(1 << 16)
                ps.Parse(chunk, not chunk)
                if not chunk:
                    break
    except expat.ExpatError as e:
        raise ValueError("not_xml") from e
    name = st["name"]
    if not pts:
        raise ValueError("no_points")
    # speeds from the times where the file has none
    out = []
    for i, p in enumerate(pts):
        t, la, lo, kn, hd = p
        if kn is None and i > 0 and t is not None and pts[i - 1][0] is not None and t > pts[i - 1][0]:
            q = pts[i - 1]
            kn = _num(dist_m(q[1], q[2], la, lo) / (t - q[0]) * 3600.0 / 1852.0, 0.0, 100.0)
        out.append((t, la, lo, kn, hd))
    return name or os.path.splitext(os.path.basename(src))[0], out
