"""Recorded camera video next to a log (section F).

The gateway records 1-minute files named by their UTC start time:
    .../<CAMERA>/YYYYMMDD-HHMMSS.ts     (ring buffer or a saved event)
The boat log counts milliseconds since power-up; its TIME events (GPS date +
time of day, once a minute) turn that into UTC. Accuracy: about 1 s (a file
starts with the first key frame after the minute).

    idx = VideoIndex(folder)                 every .ts below the folder, by camera
    utc = log_utc(log)                       function boat t_ms -> UTC (or None)
    idx.locate("FRONT-1", utc(t_ms))         -> (file, seconds into it) or None
    frame_ppm(file, seconds, width)          -> PPM bytes (tk.PhotoImage) or None
"""
import bisect
import calendar
import os
import re
import shutil
import subprocess

SEG_RE = re.compile(r"^(\d{8})-(\d{6})\.ts$")
EV_TIME = 16


def seg_time(name):
    m = SEG_RE.match(name)
    if not m:
        return None
    d, t = m.group(1), m.group(2)
    return calendar.timegm((int(d[:4]), int(d[4:6]), int(d[6:]), int(t[:2]), int(t[2:4]), int(t[4:]), 0, 0, 0))


def gps_epoch(date, tod_ms):
    y, m, d = date // 10000, (date // 100) % 100, date % 100
    return calendar.timegm((y, m, d, 0, 0, 0, 0, 0, 0)) + tod_ms / 1000.0


def log_utc(log):
    """boat t_ms -> UTC seconds, from the log's TIME events; None without GPS time."""
    pts = sorted((t, gps_epoch(a[0], a[1]) - a[2] / 1000.0) for t, _txt, code, a in log.events
                 if code == EV_TIME and a[0] > 19700101)        # (when, UTC - boat time)
    if not pts:
        return None
    ts = [p[0] for p in pts]

    def f(t_ms):
        i = min(max(bisect.bisect_right(ts, t_ms) - 1, 0), len(pts) - 1)
        return pts[i][1] + t_ms / 1000.0             # the nearest earlier TIME event's offset
    return f


class VideoIndex:
    def __init__(self, folder, seg_s=60.0):
        self.folder = folder
        self.seg_s = seg_s
        self.cams = {}                                  # name -> [(start, path)] sorted
        for dirpath, _dirs, files in os.walk(folder):
            cam = os.path.basename(dirpath)
            for n in files:
                t = seg_time(n)
                if t is not None:
                    self.cams.setdefault(cam, []).append((t, os.path.join(dirpath, n)))
        for cam, lst in self.cams.items():
            seen, out = set(), []
            for t, p in sorted(lst):                    # the same minute in the ring and an event: once
                if t not in seen:
                    seen.add(t)
                    out.append((t, p))
            self.cams[cam] = out

    def names(self):
        order = ["FRONT-1", "FRONT-2", "REAR-LEFT", "REAR-RIGHT"]
        return sorted(self.cams, key=lambda n: (order.index(n) if n in order else 9, n))

    def span(self):
        allt = [t for lst in self.cams.values() for t, _p in lst]
        return (min(allt), max(allt) + self.seg_s) if allt else None

    def locate(self, cam, utc):
        lst = self.cams.get(cam) or []
        if utc is None or not lst:
            return None
        i = bisect.bisect_right([t for t, _p in lst], utc) - 1
        if i < 0:
            return None
        t, p = lst[i]
        end = lst[i + 1][0] if i + 1 < len(lst) else t + self.seg_s
        if utc >= min(end, t + self.seg_s + 1.0):
            return None                                 # a gap: no video at that moment
        return p, utc - t

    def frame(self, cam, utc, fn):
        """fn(path, offset) at the moment utc. A file starts with the first key
        frame after its name time (up to ~1 s later), so a moment near the end of
        one file can be in the next one: then that one is used."""
        loc = self.locate(cam, utc)
        if loc is None:
            return None, None
        data = fn(*loc)
        if data:
            return data, loc
        lst = self.cams.get(cam) or []
        starts = [t for t, _p in lst]
        i = bisect.bisect_right(starts, utc) - 1
        if 0 <= i < len(lst) - 1:
            nxt = (lst[i + 1][1], max(0.0, utc - lst[i + 1][0]))
            data = fn(*nxt)
            if data:
                return data, nxt
        return None, loc


def ffmpeg_bin():
    return shutil.which("ffmpeg") or r"C:\msys64\ucrt64\bin\ffmpeg.exe"


def _grab(path, offset_s, out):
    """ffmpeg picture at offset_s. MPEG-TS seeking is byte-based and can miss
    near the end of a file: jump to 2.5 s before, decode the rest exactly;
    if that gives nothing, decode from the start (slower, always works)."""
    off = max(0.0, offset_s)
    pre = max(0.0, off - 2.5)
    for args in (["-ss", "%.3f" % pre, "-i", path, "-ss", "%.3f" % (off - pre)],
                 ["-i", path, "-ss", "%.3f" % off]):
        try:
            r = subprocess.run([ffmpeg_bin(), "-hide_banner", "-loglevel", "quiet"] + args +
                               ["-frames:v", "1"] + out + ["-"], capture_output=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if r.stdout:
            return r.stdout
    return b""


def frame_ppm(path, offset_s, width=480):
    """One picture at offset_s into the file, as PPM (None if it can't be read)."""
    data = _grab(path, offset_s, ["-vf", "scale=%d:-2" % width, "-f", "image2pipe", "-vcodec", "ppm"])
    return data if data[:2] == b"P6" else None


def frame_rgb(path, offset_s):
    """One picture at full size as raw RGB (tests: the time code)."""
    return _grab(path, offset_s, ["-f", "rawvideo", "-pix_fmt", "rgb24"])
