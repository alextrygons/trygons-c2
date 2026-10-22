"""Sample sea pictures for the obstacle detection: evaluation + the demo's front camera.

    python tools/get_vision_samples.py

Downloads the MaSTr1325 dataset (1325 pictures from a small boat in the Gulf of
Koper, 512 x 384, with hand-made masks: obstacle / water / sky) into
gateway/vision/data/mastr1325/ (not in git) and makes demo/media/sea.mp4 (a
slideshow of 150 of them, looped by the demo as the FRONT-1 camera).

Licence: MaSTr1325 is CC BY 4.0 - use allowed (also commercial) WITH this
attribution: B. Bovcon, J. Muhovic, J. Pers, M. Kristan, "The MaSTr1325
dataset for training deep USV obstacle detection models", IROS 2019.
https://box.vicos.si/borja/viamaro/index.html
NOTE: eWaSR was TRAINED on MaSTr1325 - results on it look better than at sea.
Record your own Saronic videos for a fair test (MANUAL chapter 9 part N).
"""
import io
import os
import subprocess
import sys
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(REPO, "gateway"))
import toolpaths  # noqa: E402

BASE = "https://data.vicos.si/borja/mastr1325_dataset/"
FILES = {"images": "MaSTr1325_images_512x384.zip", "masks": "MaSTr1325_masks_512x384.zip"}
DEST = os.path.join(REPO, "gateway", "vision", "data", "mastr1325")
VIDEO = os.path.join(REPO, "demo", "media", "sea.mp4")
ATTRIBUTION = ("MaSTr1325 (CC BY 4.0): B. Bovcon, J. Muhovic, J. Pers, M. Kristan, \"The MaSTr1325 dataset for "
               "training deep USV obstacle detection models\", IROS 2019. https://box.vicos.si/borja/viamaro/\n")


def main():
    for sub, fn in FILES.items():
        folder = os.path.join(DEST, sub)
        if os.path.isdir(folder) and len(os.listdir(folder)) > 1000:
            continue
        print("downloading %s ..." % (BASE + fn), flush=True)
        with urllib.request.urlopen(BASE + fn, timeout=120) as r:
            data = r.read()
        os.makedirs(folder, exist_ok=True)
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            for n in z.namelist():
                if not n.endswith("/"):
                    with open(os.path.join(folder, os.path.basename(n)), "wb") as f:
                        f.write(z.read(n))
    with open(os.path.join(DEST, "ATTRIBUTION.txt"), "w", encoding="utf-8") as f:
        f.write(ATTRIBUTION)
    os.makedirs(os.path.dirname(VIDEO), exist_ok=True)
    with open(os.path.join(os.path.dirname(VIDEO), "ATTRIBUTION.txt"), "w", encoding="utf-8") as f:
        f.write("sea.mp4 = a slideshow of MaSTr1325 pictures 0001-0150.\n" + ATTRIBUTION)
    ff = toolpaths.ffmpeg()
    if not ff:
        print("ffmpeg missing: no demo video")
        return 1
    # 150 pictures (the first ones by number; there are gaps), 1.5 s each, as a 15 fps H.264 video
    pics = sorted(f for f in os.listdir(os.path.join(DEST, "images")) if f[:4].isdigit())[:150]
    lst = os.path.join(DEST, "slideshow.txt")
    with open(lst, "w", encoding="utf-8") as f:
        for p in pics + pics[-1:]:                      # (the last one twice: the concat format needs it)
            f.write("file '%s'\nduration 1.5\n" % os.path.join(DEST, "images", p).replace("\\", "/"))
    subprocess.run([ff, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst,
                    "-vf", "fps=15,format=yuv420p", "-c:v", "libx264", "-preset", "veryfast", "-g", "30", VIDEO],
                   check=True)
    print("MaSTr1325 in %s\ndemo video %s" % (DEST, VIDEO))
    return 0


if __name__ == "__main__":
    sys.exit(main())
