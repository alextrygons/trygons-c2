"""The app icon from the logo MARK (no lettering) -> assets/ (step 13i).

    python tools/make_icon.py

assets/trygonsdrones_mark.svg (the two mark paths of trygonsdrones_logo.svg) is drawn with rsvg-convert
(MSYS2: pacman -S mingw-w64-ucrt-x86_64-librsvg) on a light rounded square (readable on a dark and a light
taskbar) and written as
  assets/trygons_c2.ico          every window + the taskbar on Windows, the Desktop shortcuts (16 .. 256 px)
  assets/trygonsdrones_mark.png  256 px: the window icon on other systems (Tk iconphoto)
The generated files are in git; run this only after the logo changed.
"""
import io
import os
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(REPO, "assets")
SIZES = (16, 20, 24, 32, 40, 48, 64, 128, 256)
TILE = "#f2f4f7"          # = groundstation/theme.py THEMES["light"]["bg"]


def main():
    from PIL import Image, ImageDraw
    rsvg = shutil.which("rsvg-convert")
    if not rsvg:
        print("rsvg-convert not found (MSYS2: pacman -S mingw-w64-ucrt-x86_64-librsvg)")
        return 1
    png = subprocess.run([rsvg, "-w", "1024", "-h", "1024", os.path.join(ASSETS, "trygonsdrones_mark.svg")],
                         capture_output=True, check=True).stdout
    mark = Image.open(io.BytesIO(png)).convert("RGBA")
    mark = mark.crop(mark.getbbox())
    big = 1024
    tile = Image.new("RGBA", (big, big), (0, 0, 0, 0))
    ImageDraw.Draw(tile).rounded_rectangle((0, 0, big - 1, big - 1), radius=int(big * 0.18), fill=TILE)
    w = int(big * 0.88)
    h = int(mark.height * w / float(mark.width))
    m = mark.resize((w, h), Image.LANCZOS)
    tile.alpha_composite(m, ((big - w) // 2, (big - h) // 2))
    tile.resize((256, 256), Image.LANCZOS).save(os.path.join(ASSETS, "trygonsdrones_mark.png"))
    tile.save(os.path.join(ASSETS, "trygons_c2.ico"), sizes=[(s, s) for s in SIZES])
    print("written assets/trygons_c2.ico (%s px) + assets/trygonsdrones_mark.png" % ", ".join(map(str, SIZES)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
