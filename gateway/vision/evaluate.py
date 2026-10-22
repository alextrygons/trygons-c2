"""Evaluate the obstacle detection on a recorded video or a folder of pictures.

    <vision python> gateway/vision/evaluate.py VIDEO_OR_FOLDER [--masks FOLDER] [--out DIR] [--max N]

Writes DIR/report.md (default test_results/vision_eval/): frames per second on
THIS machine (model alone and the whole step), detections per minute (video),
example pictures with the overlay (yellow = obstacle, blue = water, purple =
sky, red boxes = obstacles on the water), and - with --masks (MaSTr1325 style:
0 obstacle, 1 water, 2 sky, 4 unknown) - how often the model is right.

Your own recordings: the gateway records the front camera's main stream
(video/ring, SAVE events in video/events); any .ts / .mp4 file works.
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import worker as wk  # noqa: E402
import toolpaths  # noqa: E402

FF = toolpaths.ffmpeg() or "ffmpeg"


def decode(path, w=wk.W, h=wk.H, gray=False, nearest=False):
    r = subprocess.run([FF, "-loglevel", "error", "-i", path, "-vf", "scale=%d:%d%s" % (
        w, h, ":flags=neighbor" if nearest else ""), "-frames:v", "1", "-f", "rawvideo", "-pix_fmt",
        "gray" if gray else "rgb24", "-"], capture_output=True)
    n = w * h * (1 if gray else 3)
    if len(r.stdout) < n:
        return None
    a = np.frombuffer(r.stdout[:n], np.uint8)
    return a.reshape(h, w) if gray else a.reshape(h, w, 3)


def video_frames(path, fps):
    p = subprocess.Popen([FF, "-loglevel", "quiet", "-i", path, "-vf", "fps=%g,scale=%d:%d" % (fps, wk.W, wk.H),
                          "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)
    n = wk.W * wk.H * 3
    try:
        while True:
            b = p.stdout.read(n)
            if len(b) < n:
                break
            yield np.frombuffer(b, np.uint8).reshape(wk.H, wk.W, 3)
    finally:
        p.kill()                                              # stopped early (--max): no "broken pipe" noise


def overlay(rgb, cls, blobs, path):
    up = np.repeat(np.repeat(cls, 4, axis=0), 4, axis=1)               # 96x128 -> 384x512
    img = (rgb.astype(np.float32) * 0.65 + wk.PALETTE[up].astype(np.float32) * 0.35).astype(np.uint8)
    for b in blobs:
        x0, y0 = int(b["x0"] * wk.W), int(b["y0"] * wk.H)
        x1, y1 = min(wk.W - 1, int(b["x1"] * wk.W)), min(wk.H - 1, int(b["y1"] * wk.H))
        img[y0:y1 + 1, [x0, x1]] = (255, 0, 0)
        img[[y0, y1], x0:x1 + 1] = (255, 0, 0)
    subprocess.run([FF, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "%dx%d" % (
        wk.W, wk.H), "-i", "-", path], input=img.tobytes(), check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--masks", help="folder of ground-truth masks (<name>m.png)")
    ap.add_argument("--model", default=os.path.join(HERE, "models", "ewasr_resnet18.onnx"))
    ap.add_argument("--out", default=os.path.join(HERE, "..", "..", "test_results", "vision_eval"))
    ap.add_argument("--max", type=int, default=200)
    ap.add_argument("--fps", type=float, default=2.0, help="video: pictures per second to look at")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--examples", type=int, default=6)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    model = wk.Model(a.model, a.threads)
    cfg = {"bottom_ignore": 0.15, "min_blob_px": 6}
    is_video = os.path.isfile(a.input)
    if is_video:
        frames = ((None, f) for f in video_frames(a.input, a.fps))
    else:
        names = sorted(f for f in os.listdir(a.input) if f.lower().endswith((".jpg", ".png", ".ppm")))[:a.max]
        frames = ((n, decode(os.path.join(a.input, n))) for n in names)
    t_inf, t_all, n, dets, ex = [], [], 0, 0, []
    acc = {"pix": 0, "ok": 0, "obst_gt": 0, "obst_hit": 0, "water_gt": 0, "water_as_obst": 0}
    for name, rgb in frames:
        if rgb is None:
            continue
        t0 = time.time()
        cls, res = wk.analyse(model, rgb, cfg)
        t_all.append(time.time() - t0)
        t_inf.append(res["infer_ms"] / 1000.0)
        good = [b for b in res["blobs"] if b["conf"] >= 0.5 and b["width"] <= 0.6]
        dets += len(good)
        n += 1
        if a.masks and name:
            mp = os.path.join(a.masks, os.path.splitext(name)[0] + "m.png")
            gt = decode(mp, 128, 96, gray=True, nearest=True) if os.path.exists(mp) else None
            if gt is not None:
                valid = gt != 4
                acc["pix"] += int(valid.sum())
                acc["ok"] += int(((cls == gt) & valid).sum())
                acc["obst_gt"] += int((gt == 0).sum())
                acc["obst_hit"] += int(((gt == 0) & (cls == 0)).sum())
                acc["water_gt"] += int((gt == 1).sum())
                acc["water_as_obst"] += int(((gt == 1) & (cls == 0)).sum())
        if len(ex) < a.examples and (good or n % 25 == 1):
            fn = "example_%02d.png" % (len(ex) + 1)
            overlay(rgb, cls, good, os.path.join(a.out, fn))
            ex.append((fn, name or "frame %d" % n, len(good)))
        if n >= a.max:
            break
    if not n:
        print("no pictures read from %s" % a.input)
        return 1
    med = lambda v: sorted(v)[len(v) // 2]                       # noqa: E731
    lines = ["# Obstacle detection evaluation", "",
             "- input: `%s` (%s), %d pictures%s" % (a.input, "video" if is_video else "folder", n,
                                                    " at %g / s" % a.fps if is_video else ""),
             "- model: `%s`, %d threads, machine: %s" % (os.path.basename(a.model), a.threads,
                                                       os.environ.get("COMPUTERNAME") or os.uname().nodename),
             "- speed: model %.0f ms (median) = %.1f fps; whole step (model + post-processing) %.0f ms = %.1f fps"
             % (med(t_inf) * 1000, 1.0 / med(t_inf), med(t_all) * 1000, 1.0 / med(t_all)),
             "- obstacles on the water found: %d in %d pictures (%.2f per picture)%s" % (
                 dets, n, dets / float(n), "; %.1f per minute of video" % (dets / (n / a.fps / 60.0))
                 if is_video else "")]
    if acc["pix"]:
        lines += ["", "## Against the hand-made masks", "",
                  "| measure | value |", "|---|---|",
                  "| pixels right (obstacle / water / sky) | %.1f %% |" % (100.0 * acc["ok"] / acc["pix"]),
                  "| obstacle pixels found (recall) | %.1f %% |" % (100.0 * acc["obst_hit"] / max(1, acc["obst_gt"])),
                  "| water wrongly called obstacle | %.2f %% |" % (100.0 * acc["water_as_obst"] / max(1, acc["water_gt"])),
                  "", "MaSTr1325 was used to TRAIN eWaSR: these numbers are better than at sea. "
                  "Record your own Saronic videos for a fair test."]
    lines += ["", "## Examples", ""] + ["- %s (%d obstacles): ![](%s)" % (nm, k, fn) for fn, nm, k in ex]
    with open(os.path.join(a.out, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines[:6]))
    if acc["pix"]:
        print("\n".join(lines[8:13]))
    print("report: %s" % os.path.abspath(os.path.join(a.out, "report.md")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
