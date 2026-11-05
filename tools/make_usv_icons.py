"""The drawn USV symbol (owner W65, 2026-11-04: "a drawn USV symbol, simple boat outline, same style as uav.png",
no owner picture; CLAUDE.md V "Pictures"; DISPLAY ONLY).

    python tools/make_usv_icons.py

Draws assets/usv.png (1024 x 1024, transparent: the boat seen from above, bow up - a dark grey hull with a lighter
deck, the console, a red outboard at the transom, like the owner's UAV picture: dark grey body, red motors, a dark
outline) and its small sizes assets/usv_16.png, usv_24, usv_32, usv_48, usv_64 (used wherever the USV is chosen or
shown: the "Send route" target box ΣΚΑΦΟΣ, groundstation/uavpics.py). The generated files are in git; run this only
after a change here. Needs Pillow.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(REPO, "assets")
SIZES = (16, 24, 32, 48, 64)
OUTLINE = (24, 24, 26, 255)
HULL = (72, 74, 78, 255)
DECK = (118, 121, 126, 255)
CONSOLE = (46, 47, 50, 255)
GLASS = (160, 172, 184, 255)
MOTOR = (196, 24, 28, 255)
MOTOR_DARK = (120, 12, 14, 255)


def hull_points(cx, top, bottom, half_w, n=40):
    """A planing hull from above: a pointed bow curving out to a straight, wide transom."""
    import math
    bow_len = (bottom - top) * 0.42
    right = []
    for i in range(n + 1):
        t = i / n                                         # 0 = the bow tip, 1 = the end of the bow curve
        y = top + bow_len * t
        x = half_w * math.sin(t * math.pi / 2) ** 0.8
        right.append((cx + x, y))
    right.append((cx + half_w * 0.98, bottom - (bottom - top) * 0.02))
    right.append((cx + half_w * 0.94, bottom))
    left = [(2 * cx - x, y) for x, y in reversed(right)]
    return right + left


def draw(size=1024):
    from PIL import Image, ImageDraw
    ss = 2                                                # supersampled, then reduced: smooth edges
    S = size * ss
    im = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    cx, top, bottom, hw = S / 2, S * 0.07, S * 0.86, S * 0.20
    hull = hull_points(cx, top, bottom, hw)
    d.polygon(hull, fill=HULL)
    d.line(hull + [hull[0]], fill=OUTLINE, width=int(S * 0.012), joint="curve")
    deck = hull_points(cx, top + S * 0.07, bottom - S * 0.035, hw * 0.80)
    d.polygon(deck, fill=DECK)
    d.line(deck + [deck[0]], fill=OUTLINE, width=int(S * 0.005), joint="curve")
    # the console + its windscreen
    cw, ct, cb = hw * 0.62, S * 0.42, S * 0.58
    d.rounded_rectangle([cx - cw, ct, cx + cw, cb], radius=int(S * 0.025), fill=CONSOLE, outline=OUTLINE,
                        width=int(S * 0.006))
    d.polygon([(cx - cw * 0.92, ct + S * 0.012), (cx + cw * 0.92, ct + S * 0.012), (cx + cw * 0.75, ct - S * 0.04),
               (cx - cw * 0.75, ct - S * 0.04)], fill=GLASS, outline=OUTLINE)
    # a hatch outline (the UAV magazine) forward of the console
    d.rounded_rectangle([cx - hw * 0.45, S * 0.25, cx + hw * 0.45, S * 0.36], radius=int(S * 0.012),
                        outline=OUTLINE, width=int(S * 0.006))
    # the outboard at the transom: red cowling like the UAV's motors
    ow, ot, ob = hw * 0.30, bottom - S * 0.015, bottom + S * 0.085
    d.rounded_rectangle([cx - ow, ot, cx + ow, ob], radius=int(S * 0.02), fill=MOTOR, outline=OUTLINE,
                        width=int(S * 0.006))
    d.rectangle([cx - ow * 0.55, ob - S * 0.02, cx + ow * 0.55, ob - S * 0.008], fill=MOTOR_DARK)
    return im.resize((size, size), Image.LANCZOS)


def main():
    try:
        from PIL import Image
    except ImportError:
        print("Pillow is needed (pip install pillow)")
        return 1
    big = draw(1024)
    out = os.path.join(ASSETS, "usv.png")
    big.save(out, optimize=True)
    print("wrote", os.path.relpath(out, REPO))
    box = big.getbbox() or (0, 0, big.width, big.height)
    im = big.crop(box)
    side = max(im.width, im.height)
    sq = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    sq.alpha_composite(im, ((side - im.width) // 2, (side - im.height) // 2))
    for s in SIZES:
        p = os.path.join(ASSETS, "usv_%d.png" % s)
        sq.resize((s, s), Image.LANCZOS).save(p, optimize=True)
        print("wrote", os.path.relpath(p, REPO))
    return 0


if __name__ == "__main__":
    sys.exit(main())
