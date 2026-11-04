"""A Tk canvas drawn again with Pillow - a picture of screen 2 for docs/screens WITHOUT the desktop (step 13n1).

    canvaspic.save(canvas, "docs/screens/x.png")

Screen grabs (ImageGrab) show whatever is on the monitor: on a locked / disconnected desktop that is the wallpaper,
so the test pictures were wrong. This walks the canvas items (rectangles, polygons, lines, ovals, text, images)
in stacking order and paints them into a Pillow image of the same size. Tests / documentation only; DISPLAY ONLY.
"""
import os
import tkinter.font as tkfont

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:              # no Pillow: no picture
    Image = None

_FONTS = {}


def _rgb(cv, colour):
    if not colour:
        return None
    try:
        r, g, b = cv.winfo_rgb(colour)
        return (r // 257, g // 257, b // 257)
    except Exception:
        return None


def _font(cv, spec):
    a = tkfont.Font(root=cv, font=spec).actual()
    size = abs(int(a["size"])) if a["size"] < 0 else int(round(a["size"] * float(cv.tk.call("tk", "scaling"))))
    bold = a["weight"] == "bold"
    key = (size, bold, a["family"])
    if key not in _FONTS:
        names = (("segoeuib.ttf", "DejaVuSans-Bold.ttf", "arialbd.ttf") if bold else
                 ("segoeui.ttf", "DejaVuSans.ttf", "arial.ttf"))
        if "Consolas" in a["family"]:
            names = ("consolab.ttf",) + names if bold else ("consola.ttf",) + names
        f = None
        for n in names:
            for d in ("", os.path.join(os.environ.get("WINDIR", "C:\\Windows"), "Fonts")):
                try:
                    f = ImageFont.truetype(os.path.join(d, n) if d else n, max(6, size))
                    break
                except OSError:
                    continue
            if f:
                break
        _FONTS[key] = f or ImageFont.load_default()
    return _FONTS[key]


def _photo(cv, name):
    """A canvas image (a Tk photo) as a Pillow RGBA image, or None."""
    try:
        w, h = int(cv.tk.call("image", "width", name)), int(cv.tk.call("image", "height", name))
        data = cv.tk.call(name, "data", "-format", "png")
        import base64
        import io
        raw = base64.b64decode(data) if isinstance(data, str) else bytes(data)
        im = Image.open(io.BytesIO(raw)).convert("RGBA")
        return im if im.size == (w, h) else im.resize((w, h))
    except Exception:
        try:                                    # older Tk: read the pixels row by row (slow, small images only)
            w, h = int(cv.tk.call("image", "width", name)), int(cv.tk.call("image", "height", name))
            if w * h > 400000:
                return None
            im = Image.new("RGBA", (w, h))
            px = im.load()
            for y in range(h):
                for x in range(w):
                    r, g, b = (int(v) for v in str(cv.tk.call(name, "get", x, y)).split()[:3])
                    px[x, y] = (r, g, b, 255)
            return im
        except Exception:
            return None


def render(cv, bg=None):
    """The canvas as a Pillow RGB image."""
    if Image is None:
        return None
    W, H = cv.winfo_width(), cv.winfo_height()
    img = Image.new("RGBA", (W, H), _rgb(cv, bg or cv.cget("bg")) + (255,))
    for i in cv.find_all():
        if cv.itemcget(i, "state") == "hidden":
            continue
        t = cv.type(i)
        co = [float(v) for v in cv.coords(i)]
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        try:
            if t == "image":
                name = cv.itemcget(i, "image")
                im = _photo(cv, name) if name else None
                if im is not None:
                    anchor = cv.itemcget(i, "anchor")
                    x, y = co[0], co[1]
                    if anchor == "center":
                        x, y = x - im.width / 2, y - im.height / 2
                    elif anchor == "nw":
                        pass
                    img.paste(im, (int(x), int(y)), im)
                continue
            fill = _rgb(cv, cv.itemcget(i, "fill"))
            if t in ("rectangle", "oval", "polygon", "arc"):
                ol = _rgb(cv, cv.itemcget(i, "outline"))
                w = max(1, int(float(cv.itemcget(i, "width") or 1)))
                if t == "rectangle":
                    d.rectangle(co, fill=fill + (255,) if fill else None, outline=ol + (255,) if ol else None, width=w)
                elif t == "oval":
                    d.ellipse(co, fill=fill + (255,) if fill else None, outline=ol + (255,) if ol else None, width=w)
                elif t == "polygon" and len(co) >= 6:
                    d.polygon(co, fill=fill + (255,) if fill else None, outline=ol + (255,) if ol else None)
            elif t == "line" and fill and len(co) >= 4:
                d.line(co, fill=fill + (255,), width=max(1, int(float(cv.itemcget(i, "width") or 1))))
            elif t == "text" and fill:
                s = cv.itemcget(i, "text")
                f = _font(cv, cv.itemcget(i, "font"))
                anchor = cv.itemcget(i, "anchor")
                pa = {"center": "mm", "w": "lm", "e": "rm", "n": "mt", "s": "mb", "nw": "lt", "ne": "rt", "sw": "lb",
                      "se": "rb"}.get(anchor, "mm")
                lines = s.split("\n")
                bb = f.getbbox("ÁgΣ")
                lh = (bb[3] - bb[1]) * 1.25
                total = lh * len(lines)
                y0 = co[1] if pa[1] == "t" else co[1] - total if pa[1] == "b" else co[1] - total / 2
                for k, ln in enumerate(lines):
                    d.text((co[0], y0 + k * lh), ln, font=f, fill=fill + (255,), anchor=pa[0] + "t")
        except Exception:
            continue
        img.alpha_composite(layer)
    return img.convert("RGB")


def save(cv, path):
    im = render(cv)
    if im is None:
        print("(no Pillow: %s not made)" % path)
        return False
    im.save(path)
    return True
