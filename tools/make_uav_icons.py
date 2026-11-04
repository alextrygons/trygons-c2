"""The small sizes of the owner's UAV picture (CLAUDE.md V "Pictures", docs/uav_design.md 3.3).

    python tools/make_uav_icons.py

assets/uav.png (the owner's picture, 1024 x 1024, transparent) -> assets/uav_16.png, uav_24, uav_32, uav_48, uav_64
(square, transparent, the drone centred; used wherever a vehicle is chosen or shown: the magazine panel, the UAV
list, the "Send route" target box, the map menu and symbol). The generated files are in git; run this only after the
owner's picture changed. Needs Pillow.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(REPO, "assets")
SIZES = (16, 24, 32, 48, 64)


def main():
    from PIL import Image
    src = os.path.join(ASSETS, "uav.png")
    if not os.path.exists(src):
        print("assets/uav.png missing (the owner's UAV picture)")
        return 1
    im = Image.open(src).convert("RGBA")
    box = im.getbbox() or (0, 0, im.width, im.height)
    im = im.crop(box)
    side = max(im.width, im.height)
    sq = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    sq.alpha_composite(im, ((side - im.width) // 2, (side - im.height) // 2))
    for s in SIZES:
        out = os.path.join(ASSETS, "uav_%d.png" % s)
        sq.resize((s, s), Image.LANCZOS).save(out, optimize=True)
        print("wrote", os.path.relpath(out, REPO))
    return 0


if __name__ == "__main__":
    sys.exit(main())
