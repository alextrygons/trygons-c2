"""The UAV video = its OWN low-priority process (CLAUDE.md V "Telemetry / video / AI", docs/uav_design.md 8; step U4).

    started by gateway/uav/manager.py: python video_worker.py --ffmpeg ... --rtsp-port 8554 --ring <ssd>/video/ring
    (JSON lines: commands on stdin, events on stdout; it ends by itself when the gateway closes its stdin)

Each drone's analog VTX is received by a VRX on a fixed channel and a USB capture device on the gateway PC. For every
slot that the manager says RUNS (only drones that fly - a drone in its slot has its VTX in pit mode - plus the
config's "always" slots for a bench test) ONE ffmpeg captures, encodes ONCE (a light stream: 640 x 480, 15 fps,
~400 kbit/s TO MEASURE; Intel Quick Sync = h264_qsv on Windows / h264_vaapi on Ubuntu when the probe works, else
libx264 ultrafast) and writes BOTH:
  - the MediaMTX path uav_<n> (RTSP publish; the stations pull it only while it is open on a screen: cameras.py)
  - the gateway ring <ring>/UAV-<n>/YYYYMMDD-HHMMSS.ts (1-minute MPEG-TS files, UTC) = the recorder's "UAV-n" track
    in every SAVE / KEEP (recorder.py links them like a camera's files)
Devices (gateway.json "uav" "video" "devices" {slot: spec}):
  "dshow:video=<name>"      Windows DirectShow capture (ffmpeg -list_devices true -f dshow -i dummy)
  "v4l2:/dev/videoN"        Ubuntu (a /dev/v4l/by-path name keeps the USB port = the slot), MJPEG input preferred
  "file:<path>"             a video file, looped (bench)
  "sim"                     the simulated VRX: simcam.aerial_frame test pictures (demo / tests)
A capture that stops giving frames for stall_s is restarted (state STALLED / DOWN = the manager's notice
"UAV n VIDEO LOST"); nothing else changes - video is never safety-critical and never touches the boat.

Commands: {"op": "config", "devices": {slot: spec}}  {"op": "run", "slots": [n, ...]}
          {"op": "sim", "slot": n, "freeze": bool}   (tests: a simulated VRX stops sending frames)
          {"op": "stop"}
Events:   {"ev": "hello", "encoder", "probe"}  {"ev": "status", "slots": {n: {...}}} once a second
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import simcam  # noqa: E402  (the simulated VRX pictures)

PROBE_S = 8.0


def out(obj):
    try:
        sys.stdout.write(json.dumps(obj) + "\n")
        sys.stdout.flush()
    except (OSError, ValueError):
        os._exit(0)


def low_priority():
    """Below the gateway's main jobs (ffmpeg inherits it). Not IDLE: the recording must keep up."""
    try:
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            k = ctypes.WinDLL("kernel32", use_last_error=True)
            k.GetCurrentProcess.restype = wintypes.HANDLE
            k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            return bool(k.SetPriorityClass(k.GetCurrentProcess(), 0x00004000))     # BELOW_NORMAL_PRIORITY_CLASS
        os.nice(10)
        return True
    except (OSError, AttributeError, ValueError):
        return False


def enc_args(encoder, fps, kbps):
    g = str(max(1, int(fps * 2)))
    b = ["-b:v", "%dk" % kbps, "-maxrate", "%dk" % kbps, "-bufsize", "%dk" % (2 * kbps), "-g", g]
    if encoder == "qsv":
        return ["-vf_post", "format=nv12", "-c:v", "h264_qsv", "-preset", "veryfast"] + b
    if encoder == "vaapi":
        return ["-vf_post", "format=nv12,hwupload", "-c:v", "h264_vaapi"] + b
    return ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p"] + b


def split_vf(args, base_vf):
    """enc_args may ask for a filter after the base one ("-vf_post"): -> (encoder args, the -vf string)."""
    vf, outl, i = base_vf, [], 0
    while i < len(args):
        if args[i] == "-vf_post":
            vf = vf + "," + args[i + 1]
            i += 2
            continue
        outl.append(args[i])
        i += 1
    return outl, vf


def probe(ffmpeg, want):
    """The first hardware encoder that really encodes here (else x264). In this process, never in the gateway."""
    order = {"auto": (["qsv"] if os.name == "nt" else ["vaapi", "qsv"]), "qsv": ["qsv"], "vaapi": ["vaapi"]}.get(
        want, [])
    tried = {}
    for enc in order:
        ea, vf = split_vf(enc_args(enc, 5, 300), "scale=320:240")
        pre = ["-vaapi_device", "/dev/dri/renderD128"] if enc == "vaapi" else []
        cmd = [ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin"] + pre + [
            "-f", "lavfi", "-i", "testsrc2=size=320x240:rate=5", "-frames:v", "5", "-vf", vf] + ea + ["-f", "null", "-"]
        try:
            r = subprocess.run(cmd, capture_output=True, timeout=PROBE_S)
            tried[enc] = r.returncode == 0
            if r.returncode == 0:
                return enc, tried
        except (OSError, subprocess.SubprocessError):
            tried[enc] = False
    return "x264", tried


class Capture:
    """One slot: ffmpeg capture -> encode -> MediaMTX + the ring; restarted when it stalls."""

    def __init__(self, slot, spec, a, encoder):
        self.slot, self.spec, self.a, self.encoder = slot, spec, a, encoder
        self.proc = None
        self.frames, self.last_frames, self.last_frame_t = 0, 0, 0.0
        self.state, self.since, self.restarts, self.error = "OFF", time.time(), 0, ""
        self.next_start = 0.0
        self.freeze = False
        self.started = 0.0
        self.fps = 0.0
        self.gen = 0

    def cmd(self):
        a = self.a
        w, h, fps = a.w, a.h, a.fps
        sp = self.spec or ""
        if sp == "sim":
            inp = ["-f", "rawvideo", "-pix_fmt", "gray", "-s", "%dx%d" % (w, h), "-r", str(fps), "-i", "-"]
        elif sp.startswith("dshow:"):
            inp = ["-nostdin", "-f", "dshow", "-rtbufsize", "32M", "-video_size", "%dx%d" % (w, h), "-framerate",
                   str(fps), "-i", sp[len("dshow:"):]]
        elif sp.startswith("v4l2:"):
            inp = ["-nostdin", "-f", "v4l2", "-input_format", "mjpeg", "-video_size", "%dx%d" % (w, h),
                   "-framerate", str(fps), "-i", sp[len("v4l2:"):]]
        elif sp.startswith("file:"):
            inp = ["-nostdin", "-re", "-stream_loop", "-1", "-i", sp[len("file:"):]]
        else:
            raise ValueError("unknown VRX device %r" % sp)
        pre = ["-vaapi_device", "/dev/dri/renderD128"] if self.encoder == "vaapi" else []
        ea, vf = split_vf(enc_args(self.encoder, fps, a.kbps), "scale=%d:%d,fps=%g" % (w, h, fps))
        outs = []
        if a.publish:
            outs.append("[f=rtsp:rtsp_transport=tcp:onfail=ignore]rtsp://127.0.0.1:%d/uav_%d" % (a.rtsp_port, self.slot))
        if a.ring:
            folder = os.path.join(a.ring, "UAV-%d" % self.slot)
            os.makedirs(folder, exist_ok=True)
            pat = os.path.join(folder, "%Y%m%d-%H%M%S.ts").replace("\\", "/")
            outs.append("[f=segment:segment_time=%d:segment_atclocktime=1:strftime=1:reset_timestamps=1:"
                        "segment_format=mpegts:onfail=ignore]%s" % (a.seg_s, pat))
        if not outs:
            outs.append("[f=null]-")
        return ([a.ffmpeg, "-hide_banner", "-loglevel", "error"] + pre + inp +
                ["-an", "-map", "0:v", "-vf", vf] + ea + ["-flags", "+global_header"] +
                ["-progress", "pipe:1", "-nostats", "-f", "tee", "|".join(outs)])

    def start(self, now):
        self.gen += 1
        gen = self.gen
        try:
            cmd = self.cmd()
            env = dict(os.environ, TZ="UTC0")                 # the ring file names in UTC (strftime)
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE if self.spec == "sim" else subprocess.DEVNULL,
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        except (OSError, ValueError) as e:
            self.proc, self.error, self.state = None, str(e)[:120], "DOWN"
            return
        self.started, self.last_frame_t, self.error = now, now, ""
        self.state = "STARTING"
        threading.Thread(target=self._progress, args=(self.proc, gen), daemon=True).start()
        threading.Thread(target=self._stderr, args=(self.proc,), daemon=True).start()
        if self.spec == "sim":
            threading.Thread(target=self._feed, args=(self.proc, gen), daemon=True).start()

    def _progress(self, p, gen):
        try:
            for line in p.stdout:
                if gen != self.gen:
                    break
                if line.startswith(b"frame="):
                    try:
                        self.frames = int(line.split(b"=", 1)[1])
                    except ValueError:
                        pass
        except (OSError, ValueError):
            pass

    def _stderr(self, p):
        try:
            for line in p.stderr:
                self.error = line.decode("utf-8", "replace").strip()[:160]
        except (OSError, ValueError):
            pass

    def _feed(self, p, gen):
        """The simulated VRX: aerial test pictures at the frame rate (frozen = nothing)."""
        k, period = 0, 1.0 / max(1, self.a.fps)
        nxt = time.time()
        while gen == self.gen and p.poll() is None:
            if not self.freeze:
                try:
                    p.stdin.write(simcam.aerial_frame(self.a.w, self.a.h, k, self.slot))
                    p.stdin.flush()
                except (OSError, ValueError):
                    return
            k += 1
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))

    def stop(self, why="OFF"):
        self.gen += 1
        p, self.proc = self.proc, None
        if p is not None:
            try:
                if p.stdin:
                    p.stdin.close()
            except OSError:
                pass
            try:
                p.terminate()
                p.wait(2)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    p.kill()
                except OSError:
                    pass
        self.state, self.since = why, time.time()

    def tick(self, now, run):
        old = self.state
        if not run:
            if self.proc is not None or self.state != "OFF":
                self.stop("OFF")
            return
        if self.proc is None or self.proc.poll() is not None:
            if self.proc is not None:
                self.restarts += 1
                self.proc = None
                self.state = "DOWN"
            if now >= self.next_start:
                self.next_start = now + 3.0
                self.start(now)
        else:
            if self.frames != self.last_frames:
                dt = max(0.5, now - self.last_frame_t) if self.last_frame_t else 1.0
                self.fps = 0.7 * self.fps + 0.3 * (self.frames - self.last_frames) / dt
                self.last_frames, self.last_frame_t = self.frames, now
                self.state = "RUN"
            elif now - self.last_frame_t > self.a.stall_s:
                self.restarts += 1
                self.stop("STALLED")
                self.next_start = now + 1.0
                self.fps = 0.0
        if self.state != old:
            self.since = now

    def view(self, now):
        return {"spec": self.spec, "state": self.state, "for_s": round(now - self.since, 1), "frames": self.frames,
                "fps": round(self.fps, 1), "restarts": self.restarts, "error": self.error}


def main():
    ap = argparse.ArgumentParser(description="UAV video: VRX capture -> MediaMTX uav_<n> + the gateway ring")
    ap.add_argument("--ffmpeg", default="ffmpeg")
    ap.add_argument("--rtsp-port", type=int, default=8554)
    ap.add_argument("--publish", type=int, default=1, help="1 = publish to MediaMTX")
    ap.add_argument("--ring", default="", help="the ring folder (<ssd>/video/ring); empty = no recording")
    ap.add_argument("--seg-s", type=int, default=60)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--kbps", type=int, default=400)
    ap.add_argument("--encoder", default="auto", help="auto / x264 / qsv / vaapi")
    ap.add_argument("--stall-s", type=float, default=5.0)
    a = ap.parse_args()
    prio = low_priority()
    enc, tried = probe(a.ffmpeg, a.encoder) if a.encoder != "x264" else ("x264", {})
    out({"ev": "hello", "pid": os.getpid(), "encoder": enc, "probe": tried, "low_priority": prio})
    cq = queue.Queue()

    def reader():
        for line in sys.stdin:
            try:
                cq.put(json.loads(line))
            except ValueError:
                continue
        cq.put({"op": "stop"})
    threading.Thread(target=reader, daemon=True).start()
    caps, run = {}, set()
    next_status = 0.0
    try:
        while True:
            now = time.time()
            while True:
                try:
                    c = cq.get_nowait()
                except queue.Empty:
                    break
                op = c.get("op")
                if op == "stop":
                    raise SystemExit(0)
                if op == "config":
                    want = {int(k): v for k, v in (c.get("devices") or {}).items() if v}
                    for s in list(caps):
                        if s not in want or want[s] != caps[s].spec:
                            caps.pop(s).stop()
                    for s, spec in want.items():
                        if s not in caps:
                            caps[s] = Capture(s, spec, a, enc)
                elif op == "run":
                    run = {int(s) for s in c.get("slots") or []}
                elif op == "sim" and int(c.get("slot", 0)) in caps:
                    caps[int(c["slot"])].freeze = bool(c.get("freeze"))
            for s, cap in caps.items():
                cap.tick(now, s in run)
            if now >= next_status:
                next_status = now + 1.0
                out({"ev": "status", "encoder": enc, "run": sorted(run),
                     "slots": {str(s): cap.view(now) for s, cap in caps.items()}})
            time.sleep(0.2)
    except SystemExit:
        pass
    for cap in caps.values():
        cap.stop()
    os._exit(0)


if __name__ == "__main__":
    main()
