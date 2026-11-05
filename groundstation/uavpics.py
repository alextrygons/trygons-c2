"""The vehicle pictures (CLAUDE.md V "Pictures", docs/uav_design.md 3.3; DISPLAY ONLY).

UAV = the OWNER'S picture assets/uav.png and its small sizes assets/uav_<px>.png (tools/make_uav_icons.py); used
wherever a vehicle is chosen or shown (the magazine panel now; the UAV list, the "Send route" target box, the map
menu + symbol in U3 / U4). USV = the DRAWN symbol assets/usv.png + usv_<px>.png (owner W65 2026-11-04: a simple boat
outline in the style of uav.png, no owner picture; tools/make_usv_icons.py); a line drawing only if it is missing.
"""
import os

import theme

SIZES = (16, 24, 32, 48, 64)
_cache = {}


def path(px=None, kind="uav"):
    """assets/<kind>_<px>.png (the nearest made size at or above px), or assets/<kind>.png (kind uav / usv)."""
    if px is None:
        return os.path.join(theme.ASSETS, "%s.png" % kind)
    size = next((s for s in SIZES if s >= px), SIZES[-1])
    return os.path.join(theme.ASSETS, "%s_%d.png" % (kind, size))


def photo(px, master=None, kind="uav"):
    """A Tk PhotoImage of the UAV (or kind "usv": the boat) picture (cached per size + Tk interpreter); None if the
    file is missing."""
    import tkinter as tk
    key = (kind, px, id(master) if master is not None else 0)
    im = _cache.get(key)
    if im is not None:
        return im
    p = path(px, kind)
    if not os.path.exists(p):
        return None
    try:
        im = tk.PhotoImage(master=master, file=p)
        size = next((s for s in SIZES if s >= px), SIZES[-1])
        if size > px and size % px == 0:
            im = im.subsample(size // px)
    except tk.TclError:
        return None
    _cache[key] = im
    return im


def pil(px):
    """The UAV picture as a Pillow RGBA image of px x px (map symbols; None without Pillow / the file)."""
    try:
        from PIL import Image
    except ImportError:
        return None
    try:
        return Image.open(path(px)).convert("RGBA").resize((px, px))
    except OSError:
        return None


def draw_usv(canvas, x, y, px, colour=None, tag="usv"):
    """The USV symbol (owner W65): the drawn picture assets/usv_<px>.png centred at x, y (bow up); a simple hull line
    drawing only if the picture is missing / can't be read."""
    im = photo(px, master=canvas, kind="usv")
    if im is not None:
        canvas.create_image(x, y, image=im, tags=tag)
        refs = getattr(canvas, "_usv_pics", None)
        if refs is None:
            refs = canvas._usv_pics = {}
        refs[("usv", px)] = im                           # (Tk drops an image nobody refers to; ONE per kind + size)
        return tag
    c = colour or theme.C["fg"]
    w, h = px * 0.42, px * 0.92
    pts = [x, y - h / 2, x + w / 2, y - h / 6, x + w / 2, y + h / 2, x - w / 2, y + h / 2, x - w / 2, y - h / 6]
    canvas.create_polygon(pts, fill="", outline=c, width=max(1, px // 16), tags=tag)
    canvas.create_line(x - w / 4, y + h / 4, x + w / 4, y + h / 4, fill=c, width=max(1, px // 20), tags=tag)
    return tag
