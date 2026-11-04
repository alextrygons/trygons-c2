"""Screen recording of THIS PC (step 13n4, owner 2026-10-31 E4) - DISPLAY ONLY: nothing here sends anything to
the boat or the gateway, nothing here changes how the boat is driven.

REC SCREEN (the "SCR" button on both screens)
  * both screens of this station (screen 1 = the driver window, screen 2 = the map window) are recorded by one
    ffmpeg each, in its OWN process at a low priority (BELOW NORMAL on Windows, nice 10 elsewhere): the window
    itself (Windows gdigrab hwnd=, X11 x11grab -window_id), at most WIDTH (1280) pixels wide, FPS (5) pictures a
    second, H.264 with NVENC when this PC has it, else x264 "veryfast" - about 0.15 GB per hour per screen
  * a RING of 1-minute MPEG-TS files (a file cut by a crash still plays), the last RING_MIN (60) minutes:
        <root>/ring/SCREEN-1/YYYYMMDD-HHMMSS.ts      (UTC start time, like the gateway's camera ring)
  * "●SCR" (white) while it records; a screen whose window is closed / not there just has no file
  * free disk below MIN_FREE_GB: it does not start / stops by itself with a message (low_disk); never deletes
    anything already kept
SAVE (the normal SAVE button) keeps the screen part too: the same event id, pre + post like the cameras
  (the gateway's pre_s / post_s), <root>/events/<id>/SCREEN-n/*.ts + event.json.
KEEP 5' / 1h: the last 5 / 60 min of both screens, permanent (post 0); the boat's cameras are the gateway's part.
Kept files are HARD LINKS to the ring files: the ring deletes its own name, the kept copy stays; the station never
deletes a kept event (the owner removes the folder by hand).

Folder: TRYGONS_SCREENREC=<folder> (the tests: a temporary one); else groundstation/screenrec/ (not in git).
REC SCREEN is off at every start unless this PC's setting "screen_rec" says it was on when the station closed.
"""
import calendar
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCREENS = ("SCREEN-1", "SCREEN-2")
WIDTH = 1280
FPS = 5
SEG_S = 60
RING_MIN = 60
MIN_FREE_GB = 2.0
RESTART_S = 5.0
HOUSE_S = 5.0
KEEP_CHOICES_MIN = (5, 60)
SEG_RE = re.compile(r"^(\d{8})-(\d{6})\.ts$")
EVENT_ID_RE = re.compile(r"^\d{8}-\d{6}_[A-Za-z0-9-]{1,40}$")


def root_dir():
    """TRYGONS_SCREENREC (the tests set a temporary folder), else groundstation/screenrec/ (not in git)."""
    return os.environ.get("TRYGONS_SCREENREC") or os.path.join(HERE, "screenrec")


def ffmpeg_bin():
    """gateway/toolpaths.py (loaded by its path: the gateway folder is never put on sys.path here)."""
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("trygons_toolpaths",
                                                      os.path.join(HERE, "..", "gateway", "toolpaths.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.ffmpeg()
    except (OSError, ImportError, AttributeError):
        return shutil.which("ffmpeg")


def seg_time(name):
    m = SEG_RE.match(name)
    if not m:
        return None
    d, t = m.group(1), m.group(2)
    return calendar.timegm((int(d[:4]), int(d[4:6]), int(d[6:]), int(t[:2]), int(t[2:4]), int(t[4:]), 0, 0, 0))


def event_id(now, kind):
    """The id of a SAVE / KEEP made on this PC (the gateway uses the same one for the cameras)."""
    kind = re.sub(r"[^A-Za-z0-9]+", "-", str(kind)).strip("-")[:30] or "save"
    return time.strftime("%Y%m%d-%H%M%S", time.gmtime(now)) + "_" + kind


def encoder_args(enc):
    """H.264 settings: ~300 kbit/s per screen (a mostly still map / HUD), a keyframe every 10 s."""
    if enc == "h264_nvenc":
        return ["-c:v", "h264_nvenc", "-preset", "p1", "-rc", "vbr", "-b:v", "250k", "-maxrate", "400k",
                "-bufsize", "800k", "-g", str(FPS * 10)]
    return ["-c:v", "libx264", "-preset", "veryfast", "-crf", "32", "-maxrate", "400k", "-bufsize", "800k",
            "-g", str(FPS * 10), "-tune", "stillimage"]


def grab_cmd(ffmpeg, handle, folder, enc, fps=FPS, width=WIDTH, seg_s=SEG_S, platform=None):
    """The ffmpeg command line of one screen (handle = the window's native id)."""
    platform = platform or sys.platform
    if platform.startswith("win"):
        src = ["-f", "gdigrab", "-framerate", str(fps), "-draw_mouse", "1", "-i", "hwnd=%d" % handle]
    else:
        src = ["-f", "x11grab", "-framerate", str(fps), "-window_id", "0x%x" % handle, "-i",
               os.environ.get("DISPLAY", ":0")]
    scale = "scale='trunc(min(%d,iw)/2)*2':-2,format=yuv420p" % width
    return ([ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin"] + src + ["-vf", scale] + encoder_args(enc) +
            ["-an", "-f", "segment", "-segment_time", str(seg_s), "-segment_atclocktime", "1", "-strftime", "1",
             "-reset_timestamps", "1", "-segment_format", "mpegts", "-segment_format_options", "flush_packets=1",
             os.path.join(folder, "%Y%m%d-%H%M%S.ts")])


def low_priority():
    """Popen keywords: the recorder never competes with the station (or anything else on this PC)."""
    if os.name == "nt":
        return {"creationflags": 0x00004000 | 0x08000000}     # BELOW_NORMAL_PRIORITY_CLASS | CREATE_NO_WINDOW
    return {"preexec_fn": lambda: os.nice(10)}


def probe_encoder(ffmpeg, timeout=8.0):
    """h264_nvenc when it really encodes on this PC (a GPU + driver), else libx264."""
    try:
        r = subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
                            "color=c=black:s=256x144:r=5", "-frames:v", "3", "-c:v", "h264_nvenc", "-f", "null", "-"],
                           stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=timeout, **low_priority())
        if r.returncode == 0:
            return "h264_nvenc"
    except (OSError, subprocess.SubprocessError):
        pass
    return "libx264"


class Grab:
    """One screen's ring recording (one ffmpeg process)."""

    def __init__(self, name, folder):
        self.name, self.folder = name, folder
        self.proc = None
        self.state = "OFF"                  # REC / NO WINDOW / DOWN / OFF
        self.restarts = 0
        self.next_start = 0.0

    def segments(self):
        out = []
        try:
            for n in os.listdir(self.folder):
                t = seg_time(n)
                if t is not None:
                    out.append((t, os.path.join(self.folder, n)))
        except OSError:
            pass
        out.sort()
        return out

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.running():
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
        self.state = "OFF"


class ScreenRec:
    """REC SCREEN + the screen part of SAVE / KEEP. sources: {SCREEN-n: callable -> native window id or None}."""

    def __init__(self, sources, root=None, ffmpeg=None, encoder=None, ring_min=RING_MIN, seg_s=SEG_S,
                 min_free_gb=MIN_FREE_GB, popen=None, free=None, clock=None):
        self.sources = dict(sources)
        self.root = root or root_dir()
        self.ffmpeg = ffmpeg if ffmpeg is not None else ffmpeg_bin()
        self.encoder = encoder                          # None = probed in a thread at the first start
        self.ring_min, self.seg_s = ring_min, seg_s
        self.min_free_b = min_free_gb * 1e9
        self.popen = popen or subprocess.Popen
        self.free = free or (lambda p: shutil.disk_usage(p).free)
        self.clock = clock or time.time
        self.grabs = {n: Grab(n, os.path.join(self.root, "ring", n)) for n in self.sources}
        self.on = False
        self.events = []                                # open SAVE events (post-time running / last file open)
        self.notes = []                                 # (lang key, kwargs) for the station's message line
        self.last_house = 0.0
        self.probing = False
        self.lock = threading.RLock()

    # --- folders --------------------------------------------------------------------------------
    def events_dir(self):
        return os.path.join(self.root, "events")

    def _disk_free(self):
        p = self.root
        while p and not os.path.isdir(p):
            parent = os.path.dirname(p)
            if parent == p:
                break
            p = parent
        try:
            return self.free(p or ".")
        except OSError:
            return 0

    def note(self, key, **kw):
        self.notes.append((key, kw))

    def pop_notes(self):
        out, self.notes = self.notes, []
        return out

    # --- REC SCREEN -----------------------------------------------------------------------------
    def recording(self):
        return self.on

    def start(self):
        """-> (ok, lang key). Never raises."""
        if self.on:
            return True, "scr.on"
        if not self.ffmpeg or not os.path.isfile(self.ffmpeg):
            return False, "scr.no_ffmpeg"
        try:
            os.makedirs(self.root, exist_ok=True)
        except OSError:
            return False, "scr.no_folder"
        if self._disk_free() < self.min_free_b:
            return False, "scr.low_disk"
        self.on = True
        if self.encoder is None and not self.probing:
            self.probing = True
            threading.Thread(target=self._probe, daemon=True).start()
        self.tick(force=True)
        return True, "scr.on"

    def _probe(self):
        enc = probe_encoder(self.ffmpeg)
        with self.lock:
            self.encoder, self.probing = enc, False

    def stop(self, why=None):
        self.on = False
        for g in self.grabs.values():
            g.stop()
        if why:
            self.note(why)

    def tick(self, now=None, force=False):
        """Once a second from the station: keep the ffmpegs running, the ring length, the disk, open SAVEs.
        Cheap (a few small folders); never blocks."""
        now = self.clock() if now is None else now
        with self.lock:
            if self.on:
                if self._disk_free() < self.min_free_b:
                    self.stop("scr.low_disk_stop")
                else:
                    for g in self.grabs.values():
                        self._keep_running(g, now, force)
            if force or now - self.last_house >= HOUSE_S:
                self.last_house = now
                self.housekeeping(now)
            self._tick_events(now)

    def _keep_running(self, g, now, force):
        if g.running():
            g.state = "REC"
            return
        if g.proc is not None:                          # ffmpeg ended (window closed, error): try again later
            g.proc = None
            g.restarts += 1
            g.next_start = now + RESTART_S
        if self.encoder is None or (now < g.next_start and not force):
            g.state = "DOWN" if g.state != "NO WINDOW" else g.state
            return
        try:
            h = self.sources[g.name]()
        except Exception:
            h = None
        if not h:
            g.state = "NO WINDOW"
            g.next_start = now + RESTART_S
            return
        try:
            os.makedirs(g.folder, exist_ok=True)
            env = dict(os.environ, TZ="UTC0")           # file names in UTC (strftime)
            g.proc = self.popen(grab_cmd(self.ffmpeg, h, g.folder, self.encoder, seg_s=self.seg_s),
                                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                env=env, **low_priority())
            g.state = "REC"
        except (OSError, ValueError):
            g.proc = None
            g.state = "DOWN"
        g.next_start = now + RESTART_S

    def housekeeping(self, now):
        """The ring: files older than RING_MIN + one file are deleted (never the newest = the one being written).
        Kept events are hard links: deleting the ring's name never touches them."""
        keep_from = now - self.ring_min * 60 - self.seg_s
        for g in self.grabs.values():
            for t, p in g.segments()[:-1]:
                if t < keep_from:
                    try:
                        os.remove(p)
                    except OSError:
                        pass

    def have_s(self, now=None):
        """How far back the screen ring reaches now (the longer screen), s; 0 = nothing recorded."""
        now = self.clock() if now is None else now
        have = 0.0
        for g in self.grabs.values():
            segs = g.segments()
            if segs:
                have = max(have, now - segs[0][0])
        return min(have, self.ring_min * 60 + self.seg_s)

    def status(self):
        return {"on": self.on, "encoder": self.encoder, "screens": {n: g.state for n, g in self.grabs.items()},
                "have_min": round(self.have_s() / 60.0, 1), "open": len(self.events)}

    # --- SAVE / KEEP ------------------------------------------------------------------------------
    def save(self, eid, pre_s, post_s, kind="save", keep=False, now=None):
        """The screen part of a SAVE (pre + post) or a KEEP (keep=True, post 0).
        -> (ok, lang key, have_min). Nothing recorded = (False, "scr.none", 0)."""
        now = self.clock() if now is None else now
        with self.lock:
            have = self.have_s(now)
            if have <= 0 and not self.on:
                return False, "scr.none", 0.0
            if self._disk_free() < self.min_free_b:
                return False, "scr.low_disk", 0.0          # a message; nothing already kept is touched
            if not (isinstance(eid, str) and EVENT_ID_RE.match(eid)):
                eid = event_id(now, kind)
            path = os.path.join(self.events_dir(), eid)
            k = 1
            while os.path.exists(path):
                k += 1
                path = os.path.join(self.events_dir(), "%s-%d" % (eid, k))
            try:
                os.makedirs(path, exist_ok=True)
            except OSError:
                return False, "scr.no_folder", 0.0
            ev = {"id": os.path.basename(path), "path": path, "kind": kind, "keep": bool(keep), "state": "recording",
                  "start_t": round(now - pre_s, 3), "end_t": round(now + post_s, 3), "pre_s": pre_s, "post_s": post_s,
                  "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "have_min": round(have / 60.0, 1),
                  "screens": {n: [] for n in self.grabs}, "bytes": 0}
            self.events.append(ev)
            self._link(ev)
            self._write(ev)
            return True, "scr.saved", ev["have_min"]

    def _link(self, ev):
        """Hard-link (else copy, when there is room) every finished ring file inside the window."""
        done = True
        for name, g in self.grabs.items():
            segs = g.segments()
            have = set(ev["screens"].setdefault(name, []))
            for i, (t, p) in enumerate(segs):
                if t + self.seg_s < ev["start_t"] or t > ev["end_t"]:
                    continue
                if i == len(segs) - 1 and g.running():
                    done = False                        # still being written: taken when it is finished
                    continue
                fn = os.path.basename(p)
                if fn in have:
                    continue
                dst = os.path.join(ev["path"], name)
                try:
                    os.makedirs(dst, exist_ok=True)
                    try:
                        os.link(p, os.path.join(dst, fn))
                    except OSError:
                        if self._disk_free() - os.path.getsize(p) < self.min_free_b:
                            self.note("scr.low_disk")
                            continue
                        shutil.copy2(p, os.path.join(dst, fn))
                    ev["screens"][name].append(fn)
                    ev["bytes"] += os.path.getsize(p)
                except OSError:
                    continue
            ev["screens"][name].sort()
        return done

    def _write(self, ev):
        out = {k: v for k, v in ev.items() if k != "path"}
        tmp = os.path.join(ev["path"], "event.json.tmp")
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=1)
            os.replace(tmp, os.path.join(ev["path"], "event.json"))
        except OSError:
            pass

    def _tick_events(self, now):
        for ev in list(self.events):
            done = self._link(ev)
            if now > ev["end_t"] and (done or not self.on or now > ev["end_t"] + self.seg_s + 30):
                ev["state"] = "complete" if done else "incomplete"
                self.events.remove(ev)
            self._write(ev)

    def close(self):
        """The station closes: the ffmpegs stop, the open events take what exists."""
        self.stop()
        with self.lock:
            for ev in list(self.events):
                self._link(ev)
                ev["state"] = "incomplete" if ev["end_t"] > self.clock() else "complete"
                self._write(ev)
            self.events = []
