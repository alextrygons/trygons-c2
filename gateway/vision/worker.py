"""eWaSR obstacle detection - the worker process (section 2, Part 2).

Runs in its OWN process, at low priority, on a limited number of CPU cores /
threads, started and watched by gateway/vision/manager.py. Needs numpy and
onnxruntime (on a Windows PC: tools/get_vision_python.py; on Ubuntu: pip).

    python worker.py --url rtsp://127.0.0.1:8554/cam_front1_lo --model models/ewasr_resnet18.onnx

Frames: ffmpeg decodes the camera sub-stream (stream2) and scales it to the
model input (512 x 384). A reader thread keeps ONLY the newest frame: older
ones are dropped, never queued.
Model: eWaSR (Apache-2.0, github.com/tersekmatija/eWaSR), ewasr_resnet18,
input "image" 1x3x384x512 RGB normalised like ImageNet, output "prediction"
1x3x96x128 class scores: 0 obstacle, 1 water, 2 sky.
Post-processing here: obstacle blobs that sit on / in the water (their lower
edge touches water), the bottom part of the picture ignored (own bow / wake),
a confidence per blob, the water / sky horizon. Geometry (bearing, distance,
corridor) and the decisions are done by the gateway (manager.py).

Talks JSON lines: results / status on stdout; commands on stdin:
  {"cmd": "pause"} {"cmd": "resume"} {"cmd": "url", "url": ...}
  {"cmd": "sample", "path": "<file name without extension>"}   frame (.ppm) + mask (.pgm)
  {"cmd": "quit"}
"""
import argparse
import base64
import json
import os
import queue
import subprocess
import sys
import threading
import time

import warnings

import numpy as np

warnings.filterwarnings("ignore", message=".*smallest subnormal.*")   # numpy notices the flush-to-zero setting

W, H = 512, 384                     # model input (fixed in the published ONNX file)
OBST, WATER, SKY = 0, 1, 2
MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)
PALETTE = np.array([[247, 195, 37], [41, 167, 224], [90, 75, 164]], np.uint8)   # obstacle, water, sky (eWaSR)


def emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


# --- resources -------------------------------------------------------------------------
def low_priority(cores):
    """Lowest CPU priority, only on the given cores (the gateway's main jobs come first)."""
    try:
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            k = ctypes.WinDLL("kernel32", use_last_error=True)
            k.GetCurrentProcess.restype = wintypes.HANDLE       # a 64-bit handle (not a 32-bit int)
            k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            k.SetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_size_t]
            h = k.GetCurrentProcess()
            ok = bool(k.SetPriorityClass(h, 0x00000040))       # IDLE_PRIORITY_CLASS (ffmpeg inherits it)
            if cores:
                ok = bool(k.SetProcessAffinityMask(h, sum(1 << c for c in cores))) and ok
            return ok
        else:
            os.nice(19)
            if cores:
                os.sched_setaffinity(0, set(cores))
        return True
    except (OSError, AttributeError, ValueError):
        return False


# --- frames ------------------------------------------------------------------------------
class FrameReader(threading.Thread):
    """ffmpeg -> raw RGB frames; keeps only the newest one."""

    def __init__(self, ffmpeg, url, max_fps):
        super().__init__(daemon=True)
        self.ffmpeg, self.url, self.max_fps = ffmpeg, url, max_fps
        self.lock = threading.Lock()
        self.latest = None                  # (seq, t, bytes)
        self.seq = 0
        self.proc = None
        self.running = True
        self.restart = threading.Event()

    def set_url(self, url):
        self.url = url
        self.restart.set()
        if self.proc:
            self.proc.kill()

    def get(self):
        with self.lock:
            return self.latest

    def run(self):
        n = W * H * 3
        while self.running:
            self.restart.clear()
            if not self.url:
                time.sleep(0.5)
                continue
            src = ["-rtsp_transport", "tcp", "-i", self.url] if self.url.startswith("rtsp") else \
                ["-re", "-stream_loop", "-1", "-i", self.url]
            self.proc = subprocess.Popen(
                [self.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-threads", "1"] + src +
                ["-an", "-vf", "fps=%g,scale=%d:%d" % (self.max_fps * 2.0, W, H), "-f", "rawvideo",
                 "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            while self.running and not self.restart.is_set():
                buf = self.proc.stdout.read(n)
                if not buf or len(buf) < n:
                    break
                with self.lock:
                    self.seq += 1
                    self.latest = (self.seq, time.time(), buf)
            try:
                self.proc.kill()
            except OSError:
                pass
            if self.running and not self.restart.is_set():
                time.sleep(2.0)                                 # the stream ended / failed: try again


# --- model -------------------------------------------------------------------------------
class Model:
    def __init__(self, path, threads):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = max(1, threads)
        so.inter_op_num_threads = 1
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        # tiny weights ("denormal" floats) make the model ~13x slower on x86 without this
        so.add_session_config_entry("session.set_denormal_as_zero", "1")
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")    # no busy-waiting threads
        self.sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        self.input = self.sess.get_inputs()[0].name

    def run(self, rgb):
        """rgb: H x W x 3 uint8 -> (class map 96x128 uint8, obstacle probability 96x128 float)."""
        x = ((rgb.transpose(2, 0, 1).astype(np.float32) / 255.0 - MEAN) / STD)[None]
        s = self.sess.run(["prediction"], {self.input: x})[0][0]
        e = np.exp(s - s.max(axis=0, keepdims=True))
        p = e / e.sum(axis=0, keepdims=True)
        return p.argmax(axis=0).astype(np.uint8), p[OBST]


class FakeModel:
    """For tests without the model file: sky on top, water below, nothing else."""

    def run(self, rgb):
        cls = np.full((96, 128), WATER, np.uint8)
        cls[:40] = SKY
        return cls, np.zeros((96, 128), np.float32)


# --- post-processing -----------------------------------------------------------------------
def label_runs(mask):
    """Connected areas (4-neighbour) of a boolean picture by row runs + union-find:
    -> label picture (0 = none) and the number of labels. Much faster than pixel by pixel."""
    h, w = mask.shape
    parent = [0]
    lab = np.zeros((h, w), np.int32)
    prev = []                                                   # (x0, x1, label) of the row above

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for y in range(h):
        row = mask[y].astype(np.int8)
        d = np.diff(np.concatenate(([0], row, [0])))
        starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]
        cur = []
        for x0, x1 in zip(starts.tolist(), ends.tolist()):
            my = None
            for px0, px1, pl in prev:
                if px0 < x1 and x0 < px1:                       # overlaps the run above
                    if my is None:
                        my = find(pl)
                    else:
                        a, b = find(pl), my
                        if a != b:
                            parent[max(a, b)] = min(a, b)
                            my = min(a, b)
            if my is None:
                parent.append(len(parent))
                my = len(parent) - 1
            lab[y, x0:x1] = my
            cur.append((x0, x1, my))
        prev = cur
    roots = np.array([find(i) for i in range(len(parent))], np.int32)
    return roots[lab], len(parent) - 1


def blobs(cls, p_obst, bottom_ignore=0.15, min_px=6):
    """Obstacle blobs sitting on / in the water: connected obstacle pixels whose lower
    edge (or side) touches water. Bottom part of the picture ignored (own bow, own wake)."""
    h, w = cls.shape
    limit = int(round(h * (1.0 - bottom_ignore)))
    obst = (cls == OBST)
    obst[limit:] = False
    water = (cls == WATER)
    touch = np.zeros_like(obst)                                 # obstacle pixel with water below / beside
    touch[:-1] |= water[1:]
    touch[:, :-1] |= water[:, 1:]
    touch[:, 1:] |= water[:, :-1]
    touch &= obst
    lab, n = label_runs(obst)
    out = []
    if n == 0:
        return out
    for k in np.unique(lab[obst]).tolist():
        sel = lab == k
        npx = int(sel.sum())
        if npx < min_px or not touch[sel].any():
            continue
        by, bx = np.nonzero(sel)
        bottom = int(by.max())
        bot_x = bx[by == bottom]
        out.append({"u": float((bot_x.mean() + 0.5) / w), "v": float((bottom + 1.0) / h),
                    "x0": float(bx.min() / w), "y0": float(by.min() / h),
                    "x1": float((bx.max() + 1) / w), "y1": float((bottom + 1) / h),
                    "px": npx, "width": float((bx.max() - bx.min() + 1) / w),
                    "conf": round(float(p_obst[sel].mean()), 3)})
    out.sort(key=lambda b: -b["v"])                              # nearest (lowest in the picture) first
    return out[:20]


def horizon_from_mask(cls):
    """The water / sky boundary as a line: (v at the centre, dv/du) in picture units, or None."""
    h, w = cls.shape
    us, vs = [], []
    for x in range(0, w, 4):
        col = cls[:, x]
        sky = np.nonzero(col == SKY)[0]
        wat = np.nonzero(col == WATER)[0]
        if len(sky) and len(wat) and wat.max() > sky.max():
            us.append((x + 0.5) / w)
            vs.append((sky.max() + 1.0) / h)
    if len(us) < 0.3 * (w / 4):
        return None
    a, b = np.polyfit(np.array(us), np.array(vs), 1)
    return [float(a * 0.5 + b), float(a)]


def encode_mask(cls):
    """64 x 48, 2 bits per pixel, base64 (~1 kB) - for the overlay on the screens."""
    small = cls[::2, ::2].astype(np.uint8).reshape(-1)
    packed = (small[0::4] | (small[1::4] << 2) | (small[2::4] << 4) | (small[3::4] << 6)).astype(np.uint8)
    return base64.b64encode(packed.tobytes()).decode()


def save_sample(path, rgb, cls):
    with open(path + ".ppm", "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (rgb.shape[1], rgb.shape[0]) + rgb.tobytes())
    with open(path + "_mask.pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (cls.shape[1], cls.shape[0]) + (cls * 100).astype(np.uint8).tobytes())


def analyse(model, rgb, cfg):
    t0 = time.time()
    cls, p = model.run(rgb)
    infer = time.time() - t0
    return cls, {"blobs": blobs(cls, p, cfg["bottom_ignore"], cfg["min_blob_px"]), "horizon": horizon_from_mask(cls),
                 "infer_ms": round(infer * 1000.0, 1)}


# --- main ------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url")
    ap.add_argument("--model")
    ap.add_argument("--fake", action="store_true", help="no model (tests)")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--cores", default="", help="e.g. 2,3")
    ap.add_argument("--max-fps", type=float, default=2.0)
    ap.add_argument("--ffmpeg", default="ffmpeg")
    ap.add_argument("--bottom-ignore", type=float, default=0.15)
    ap.add_argument("--min-blob-px", type=int, default=6)
    a = ap.parse_args()
    cores = [int(c) for c in a.cores.split(",") if c.strip()]
    os.environ.setdefault("OMP_NUM_THREADS", str(a.threads))
    prio = low_priority(cores)
    cfg = {"bottom_ignore": a.bottom_ignore, "min_blob_px": a.min_blob_px}
    try:
        model = FakeModel() if a.fake else Model(a.model, a.threads)
    except Exception as e:                                     # noqa: BLE001 - reported to the gateway
        emit({"t": "error", "text": "model: %s" % e})
        return 2
    emit({"t": "hello", "pid": os.getpid(), "model": "fake" if a.fake else os.path.basename(a.model or ""),
          "threads": a.threads, "cores": cores, "low_priority": prio, "input": [W, H]})
    reader = FrameReader(a.ffmpeg, a.url, a.max_fps)
    reader.start()
    cmds = queue.Queue()

    def stdin_reader():
        for line in sys.stdin:
            try:
                cmds.put(json.loads(line))
            except ValueError:
                pass
        cmds.put({"cmd": "quit"})                              # the gateway is gone
    threading.Thread(target=stdin_reader, daemon=True).start()

    paused, last_seq, sample = False, 0, None
    period = 1.0 / max(0.1, a.max_fps)
    t_last, fps, last_status, last_frame_t = 0.0, 0.0, 0.0, time.time()
    while True:
        try:
            while True:
                c = cmds.get_nowait()
                k = c.get("cmd")
                if k == "quit":
                    reader.running = False
                    if reader.proc:
                        reader.proc.kill()
                    return 0
                if k == "pause":
                    paused = True
                elif k == "resume":
                    paused = False
                elif k == "url":
                    reader.set_url(c.get("url"))
                elif k == "sample":
                    sample = c.get("path")
        except queue.Empty:
            pass
        now = time.time()
        fr = reader.get()
        if fr and fr[0] != last_seq:
            last_frame_t = fr[1]
        if now - last_status > 2.0:
            state = "PAUSED" if paused else ("NO_FRAMES" if now - last_frame_t > 3.0 else "RUNNING")
            emit({"t": "status", "state": state, "fps": round(fps, 2), "frames_in": reader.seq})
            last_status = now
        if paused or fr is None or fr[0] == last_seq or now - t_last < period:
            time.sleep(0.02)
            continue
        last_seq = fr[0]
        rgb = np.frombuffer(fr[2], np.uint8).reshape(H, W, 3)
        dt = now - t_last if t_last else period                # the rate = from start to start
        t_last = now
        cls, res = analyse(model, rgb, cfg)
        fps = 1.0 / dt if fps == 0.0 else 0.8 * fps + 0.2 / max(dt, 1e-3)
        res.update({"t": "result", "frame_t": fr[1], "age_ms": round((time.time() - fr[1]) * 1000.0),
                    "fps": round(fps, 2), "mask": encode_mask(cls), "mask_w": 64, "mask_h": 48})
        emit(res)
        if sample:
            try:
                save_sample(sample, rgb, cls)
                emit({"t": "sampled", "path": sample})
            except OSError as e:
                emit({"t": "error", "text": "sample: %s" % e})
            sample = None


if __name__ == "__main__":
    sys.exit(main())
