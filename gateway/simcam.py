"""Simulated cameras for the tests (no Tapo needed): ffmpeg encodes frames
made here (H.264, like the camera) and sends them by RTSP to MediaMTX.

  freeze(True)   the camera stays connected but sends no new frames
  unplug()       the camera is gone
The picture carries a time code (26 black / white squares = the send time in
ms), so a viewer can measure the real video latency: decode_ts(frame).
"""
import subprocess
import threading
import time

try:
    import childproc                    # groundstation/childproc.py (on the path through common.py)
except ImportError:
    childproc = None

BITS, SQ, X0, Y0 = 26, 12, 4, 8


def ffmpeg_bin():
    import toolpaths
    return toolpaths.ffmpeg() or "ffmpeg"


def now_code():
    return int(time.time() * 1000) & ((1 << BITS) - 1)


class SimCamera:
    def __init__(self, url, width=320, height=240, fps=15, kbps=600):
        self.url, self.w, self.h, self.fps, self.kbps = url, width, height, fps, kbps
        self.frozen = False
        self.running = False
        self.proc = None
        self.sent = 0

    def start(self):
        self.running = True
        self.proc = subprocess.Popen(
            [ffmpeg_bin(), "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "gray",
             "-s", "%dx%d" % (self.w, self.h), "-r", str(self.fps), "-i", "-", "-c:v", "libx264",
             "-preset", "ultrafast", "-tune", "zerolatency", "-g", str(self.fps), "-b:v", "%dk" % self.kbps,
             "-pix_fmt", "yuv420p", "-f", "rtsp", "-rtsp_transport", "tcp", self.url],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if childproc:
            childproc.dies_with_parent(self.proc)              # no orphaned encoder when a test is killed
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def _frame(self, k):
        w, h = self.w, self.h
        img = bytearray([96]) * (w * h)
        x = (k * 4) % w                                        # a moving bar: "live" picture
        for y in range(h // 2, h):
            img[y * w + x:y * w + min(w, x + 8)] = b"\xc8" * (min(w, x + 8) - x)
        code = now_code()
        for i in range(BITS):
            v = 255 if (code >> (BITS - 1 - i)) & 1 else 0
            x0 = X0 + i * SQ
            for y in range(Y0, Y0 + SQ):
                img[y * w + x0:y * w + x0 + SQ] = bytes([v]) * SQ
        return bytes(img)

    def _run(self):
        k = 0
        period = 1.0 / self.fps
        nxt = time.time()
        while self.running:
            if not self.frozen:
                try:
                    self.proc.stdin.write(self._frame(k))
                    self.proc.stdin.flush()
                    self.sent += 1
                except (OSError, ValueError):
                    return
            k += 1
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))

    def freeze(self, on=True):
        self.frozen = on

    def unplug(self):
        self.running = False
        if self.proc:
            self.proc.kill()

    stop = unplug


def aerial_frame(w, h, k, slot=1):
    """A simulated VRX picture (step U4: the UAV video in the demo / tests, no real VRX needed), grey w x h: the sea
    seen from above (bands drifting with k), a light hull crossing the picture, the slot number as small bars on the
    right and the time code of decode_ts top left (so a viewer can measure the latency like a boat camera)."""
    rows = []
    hull_y0, hull_y1 = h // 2 - h // 12, h // 2 + h // 12
    hx = (k * 3) % (w + w // 4) - w // 8
    hw = max(8, w // 6)
    for y in range(h):
        v = 60 + (18 if ((y + k) // 12) % 2 else 0)
        row = bytearray([v]) * w
        if hull_y0 <= y < hull_y1:
            x0, x1 = max(0, hx), min(w, hx + hw)
            if x1 > x0:
                row[x0:x1] = b"\xd2" * (x1 - x0)
        rows.append(row)
    for i in range(max(1, int(slot))):                        # the slot: n bars on the right
        x0 = w - 10 - i * 8
        for y in range(Y0, Y0 + SQ * 2):
            if 0 <= x0 < w - 4:
                rows[y][x0:x0 + 4] = b"\xff" * 4
    code = now_code()
    for i in range(BITS):
        val = 255 if (code >> (BITS - 1 - i)) & 1 else 0
        x0 = X0 + i * SQ
        if x0 + SQ > w:
            break
        for y in range(Y0, Y0 + SQ):
            rows[y][x0:x0 + SQ] = bytes([val]) * SQ
    return b"".join(bytes(r) for r in rows)


def decode_ts(frame, w, h):
    """The time code from an RGB frame (w x h x 3), or None."""
    if frame is None or len(frame) < w * h * 3:
        return None                                            # no picture at all
    code = 0
    yc = Y0 + SQ // 2
    for i in range(BITS):
        xc = X0 + i * SQ + SQ // 2
        p = (yc * w + xc) * 3
        lum = frame[p] if p < len(frame) else 0
        if 40 < lum < 215:
            return None                                        # not clear enough
        code = (code << 1) | (1 if lum >= 128 else 0)
    return code


def latency_ms(code):
    if code is None:
        return None
    d = (now_code() - code) & ((1 << BITS) - 1)
    return d if d < 60000 else None
