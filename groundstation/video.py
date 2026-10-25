"""Live camera video for the ground station (section E).

VideoStream: one RTSP stream from the gateway's relay (MediaMTX), decoded by
ffmpeg into raw RGB frames (low-latency flags, no buffering). The newest
frame is kept; age() tells how long ago it came - more than 1 s = VIDEO
FROZEN (the driver screen shows the overlay).

FrontSelector: the driver screen shows FRONT-1; if it has no new frame for
1 s it switches to FRONT-2 (with a notice) and back to FRONT-1 once that has
delivered frames again for 2 s. A manual choice overrides it.

Needs ffmpeg on the PATH (Windows: MSYS2 "pacman -S mingw-w64-ucrt-x86_64-ffmpeg"
or ffmpeg.org).
"""
import shutil
import subprocess
import threading
import time

FROZEN_S = 1.0


def ffmpeg_bin():
    return shutil.which("ffmpeg") or r"C:\msys64\ucrt64\bin\ffmpeg.exe"


class VideoStream:
    def __init__(self, url, width=640, height=360, name=""):
        self.url = url
        self.w, self.h = width, height
        self.name = name
        self.frame = None               # newest frame, raw RGB (w*h*3 bytes)
        self.t_frame = 0.0              # when it arrived
        self.frames = 0
        self.proc = None
        self.running = False
        self.error = ""
        self.started = 0.0
        self.on_frame = None            # optional callback(frame bytes) (tests)

    def start(self):
        self.running = True
        self.started = time.time()
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def stop(self):
        self.running = False
        if self.proc:
            try:
                self.proc.kill()
            except OSError:
                pass

    def _cmd(self):
        return [ffmpeg_bin(), "-hide_banner", "-loglevel", "error", "-rtsp_transport", "tcp",
                "-fflags", "nobuffer", "-flags", "low_delay", "-probesize", "32768", "-analyzeduration", "0",
                "-i", self.url, "-an", "-vf", "scale=%d:%d" % (self.w, self.h),
                "-pix_fmt", "rgb24", "-f", "rawvideo", "-"]

    def _run(self):
        size = self.w * self.h * 3
        while self.running:
            try:
                self.proc = subprocess.Popen(self._cmd(), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                             bufsize=size)
            except OSError as e:
                self.error = "ffmpeg not found: %s" % e
                return
            out = self.proc.stdout
            while self.running:
                buf = out.read(size)
                if not buf or len(buf) < size:
                    break
                self.frame = buf
                self.t_frame = time.time()
                self.frames += 1
                if self.on_frame:
                    self.on_frame(buf)
            if self.proc.poll() is None:
                self.proc.kill()
            err = self.proc.stderr.read().decode(errors="replace").strip() if self.proc.stderr else ""
            if err:
                self.error = err.splitlines()[-1][:120]
            if self.running:
                time.sleep(1.0)                 # camera / link gone: try again

    def age(self):
        return time.time() - (self.t_frame or self.started)

    def frozen(self):
        return self.age() > FROZEN_S

    def ppm(self):
        """The newest frame as PPM (tkinter PhotoImage(data=..., format="ppm"))."""
        if self.frame is None:
            return None
        return b"P6 %d %d 255\n" % (self.w, self.h) + self.frame


def notice_text(sel):
    """The FrontSelector notice (English, as logged) in the screen language."""
    import i18n
    return i18n.from_en(sel.notice)


class FrontSelector:
    """FRONT-1 big; FRONT-2 when FRONT-1 is frozen / gone; back when FRONT-1 is good again."""

    def __init__(self, back_after_s=2.0):
        self.use = "FRONT-1"
        self.manual = None              # None = automatic, else "FRONT-1" / "FRONT-2"
        self.back_after_s = back_after_s
        self.good_since = None
        self.notice = ""
        self.switches = []

    def update(self, f1, f2, now=None):
        """f1 / f2: VideoStream or None. Returns the name to show."""
        now = now or time.time()
        if self.manual:
            self.use, self.notice = self.manual, "%s chosen by hand" % self.manual
            return self.use
        f1_ok = f1 is not None and not f1.frozen()
        f2_ok = f2 is not None and not f2.frozen()
        if f1_ok:
            self.good_since = self.good_since or now
        else:
            self.good_since = None
        old = self.use
        if self.use == "FRONT-1" and not f1_ok and f2_ok:
            self.use, self.notice = "FRONT-2", "FRONT-1 LOST - showing FRONT-2 (backup)"
        elif self.use == "FRONT-2" and f1_ok and now - self.good_since >= self.back_after_s:
            self.use, self.notice = "FRONT-1", ""
        elif self.use == "FRONT-2" and not f2_ok and f1_ok:
            self.use, self.notice = "FRONT-1", ""
        if self.use != old:
            self.switches.append((now, old, self.use))
        return self.use
