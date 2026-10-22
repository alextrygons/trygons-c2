"""The demo's simulated world outside the AURIX simulator: 4 camera test
streams (published to MediaMTX) and AIS traffic around the start position.
Started and stopped by demo.py (START_DEMO.bat / STOP_DEMO.bat).

A command file lets demo.py add things while it runs:
  {"ship": {"lat":..., "lon":..., "sog":..., "cog":..., "name":...}}   one more AIS ship
The front camera can show a video file instead of the test picture
(--front-video, looped).
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gateway"))
from simais import AisSim, Ship, traffic  # noqa: E402
from simcam import SimCamera, ffmpeg_bin  # noqa: E402
from ais import dist_m  # noqa: E402


def video_camera(url, path):
    """A video file, looped, as a live RTSP camera (H.264, low latency)."""
    return subprocess.Popen([ffmpeg_bin(), "-hide_banner", "-loglevel", "error", "-re", "-stream_loop", "-1",
                             "-i", path, "-an", "-vf", "scale=640:-2", "-c:v", "libx264", "-preset", "ultrafast",
                             "-tune", "zerolatency", "-g", "15", "-b:v", "800k", "-pix_fmt", "yuv420p",
                             "-f", "rtsp", "-rtsp_transport", "tcp", url],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtsp", type=int, default=8554)
    ap.add_argument("--ais-port", type=int, default=5790)
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--ships", type=int, default=10)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--cmd", required=True, help="command file (JSON)")
    ap.add_argument("--front-video", help="video file for FRONT-1 (looped)")
    a = ap.parse_args()

    url = "rtsp://127.0.0.1:%d/simcam_%%d" % a.rtsp
    cams, procs = [], []
    for i in range(4):
        if i == 0 and a.front_video and os.path.exists(a.front_video):
            procs.append(video_camera(url % i, a.front_video))
        else:
            cams.append(SimCamera(url % i, width=320, height=240).start())

    ais = AisSim(a.ais_port, seed=a.seed, noise_m=6.0, drop=0.05, bad=0.01).start()
    n = 0
    for sh in traffic(a.lat, a.lon, a.ships * 2, seed=a.seed, radius_nm=6.0):
        if dist_m(a.lat, a.lon, sh.lat, sh.lon) > 1852.0 and n < a.ships:      # none on top of us
            ais.add(sh)
            n += 1
    print("demo world: %d cameras, %d AIS ships around %.4f %.4f" % (4, n, a.lat, a.lon), flush=True)
    mmsi = 237999000
    try:
        while True:
            time.sleep(0.5)
            if os.path.exists(a.cmd):
                try:
                    with open(a.cmd, encoding="utf-8") as f:
                        c = json.load(f)
                    os.remove(a.cmd)
                except (OSError, ValueError):
                    continue
                s = c.get("ship")
                if s:
                    mmsi += 1
                    ais.add(Ship(mmsi, s["lat"], s["lon"], s["sog"], s["cog"], cls="A",
                                 name=s.get("name", "DEMO FERRY")))
                    print("added ship %s %.1f kn %03.0f" % (s.get("name"), s["sog"], s["cog"]), flush=True)
            for i, c in enumerate(cams):                 # a camera encoder died: start it again
                if c.proc and c.proc.poll() is not None:
                    c.stop()
                    cams[i] = SimCamera(c.url, width=320, height=240).start()
                    print("camera %s restarted" % c.url, flush=True)
            for p in procs:
                if p.poll() is not None:                 # the video player ended: start it again
                    procs.remove(p)
                    procs.append(video_camera(url % 0, a.front_video))
                    break
    except KeyboardInterrupt:
        pass
    finally:
        for c in cams:
            c.stop()
        for p in procs:
            p.kill()
        ais.stop()


if __name__ == "__main__":
    main()
