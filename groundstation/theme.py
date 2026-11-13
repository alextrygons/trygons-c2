"""ONE look for both screens of a station (CLAUDE.md section E): screen 1 (driving: the
camera HUD, driver.py + hud.py) and screen 2 (navigation & control: the map with the
corner panels, app.py + overlay.py + mapwin.py) and their dialogs.

Everything visual comes from here - no screen module has its own fonts or colours:
  font family + the size levels   title / value / label / button / small
                                  font(level) for tk widgets (points: follow Windows scaling),
                                  px(level, W, H) for drawn text (canvas / HUD, pixels)
  colours                         C[...] = the current theme (dark / light), the meaning colours
                                  (WARN, CAUTION, GOOD, ADVISORY, MODE_COLOURS) are the SAME in both
                                  themes and on both screens
  buttons + panels                BUTTON (shape, size), PANEL (corner radius, border), ALPHA
                                  (how much of the map / video shows through a panel)
  dark / light                    set_theme("dark" | "light") switches BOTH screens at once
                                  (listeners: on_change); night mode (settings "night") stays the
                                  dim red of the HUD and the map
  the logo                        logo_path(dark) / logo_image(height, dark): assets/
  Windows scaling                 dpi_aware() (sharp text), dpi_factor(root) = 1.0 / 1.25 / 1.5 ...
  the app icon + splash (13i)     set_app_icon(window): assets/trygons_c2.ico (the logo MARK, no text) on
                                  every window + the taskbar; SPLASH = the start-up screen's colours
DISPLAY ONLY: nothing in here talks to the boat.
"""
import os
import sys

import settings

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(os.path.dirname(HERE), "assets")

# --- fonts -----------------------------------------------------------------------------------------
FAMILY = "Segoe UI"
MONO = "Consolas"
FONT_FILES = {False: ("segoeui.ttf", "DejaVuSans.ttf", "arial.ttf"),        # the Pillow renderer (HUD)
              True: ("segoeuib.ttf", "DejaVuSans-Bold.ttf", "arialbd.ttf")}
LEVELS = ("title", "value", "label", "button", "small")
# ONLY 3 text sizes everywhere (step 13n1, owner 2026-10-31 E1): very important / heading / info. The old level
# names stay as the callers' words; each one is one of the three sizes.
SIZES = ("important", "heading", "info")
SIZE_OF = {"title": "important", "huge": "important", "code": "important", "value": "heading", "label": "info",
           "button": "info", "small": "info"}
PT_SIZE = {"important": 16, "heading": 13, "info": 10}
PX_SIZE = {"important": 24, "heading": 18, "info": 14}           # drawn text at a 1920 x 1080 window
MIN_PX_SIZE = {"important": 15, "heading": 12, "info": 10}
PT = {lv: PT_SIZE[SIZE_OF[lv]] for lv in SIZE_OF}
BOLD = {"title": True, "value": True, "label": False, "button": True, "small": False, "huge": True, "code": True}
PX = {lv: PX_SIZE[SIZE_OF[lv]] for lv in SIZE_OF}
MIN_PX = {lv: MIN_PX_SIZE[SIZE_OF[lv]] for lv in SIZE_OF}
# the text-size setting of every window ("Aa": small / normal / large, saved per PC = settings "text_size")
TEXT_SIZES = ("small", "normal", "large")
TEXT_SCALE = {"small": 0.85, "normal": 1.0, "large": 1.2}


def text_size():
    t = settings.get("text_size") or "normal"
    return t if t in TEXT_SCALE else "normal"


def text_scale():
    return TEXT_SCALE[text_size()]


_size_listeners = []


def on_text_size(fn):
    """fn(ratio) after an "Aa" change (ratio = new / old scale): every window resizes its text."""
    _size_listeners.append(fn)


def set_text_size(t):
    if t not in TEXT_SCALE or t == text_size():
        return
    ratio = TEXT_SCALE[t] / text_scale()
    settings.put("text_size", t)
    for fn in list(_size_listeners):
        try:
            fn(ratio)
        except Exception:               # a closed window: forget it
            _size_listeners.remove(fn)


def cycle_text_size():
    """The "Aa" button: small -> normal -> large -> small."""
    set_text_size(TEXT_SIZES[(TEXT_SIZES.index(text_size()) + 1) % len(TEXT_SIZES)])


def font(level="label", bold=None, mono=False):
    """A tk font for widgets: (family, points, weight). Points follow the Windows scaling + the "Aa" setting."""
    b = BOLD.get(level, False) if bold is None else bold
    return (MONO if mono else FAMILY, int(round(PT.get(level, PT["label"]) * text_scale())), "bold" if b else "normal")


def unit(W, H):
    """The size factor of a window against 1920 x 1080 (0.62 .. 1.6)."""
    return max(0.62, min(1.6, min(W / 1920.0, H / 1080.0)))


def px(level, W, H, k=1.0):
    """A drawn font size in pixels for a W x H window; k = Windows scaling (dpi_factor) x fit factor; x "Aa"."""
    return int(round(max(MIN_PX.get(level, 10), PX.get(level, 15) * unit(W, H) * k * text_scale())))


def rescale_tree(widget, ratio):
    """After an "Aa" change: every tk widget's font in this window x ratio (the window keeps working). The exact
    (unrounded) size is kept per widget, so small / normal / large back and forth never drifts (13n1 review L1);
    a font changed by the program meanwhile starts again from its new size."""
    import tkinter.font as tkf
    stack = [widget]
    while stack:
        w = stack.pop()
        try:
            f = w.cget("font")
            if f:
                a = tkf.Font(root=w, font=f).actual()
                size = a["size"]
                base = getattr(w, "_aa_size", None)
                exact = base[0] if base is not None and base[1] == size else float(size)
                exact *= ratio
                new = int(round(exact))
                if new == 0:
                    new = 1 if exact > 0 else -1
                if new != size:
                    w.configure(font=(a["family"], new, a["weight"]))
                w._aa_size = (exact, new)
        except Exception:
            pass
        try:
            stack.extend(w.winfo_children())
        except Exception:
            pass


def tkfont(level, size_px, bold=None, mono=False):
    """A canvas font in PIXELS (negative size): (family, -px, weight)."""
    b = BOLD.get(level, False) if bold is None else bold
    return (MONO if mono else FAMILY, -int(size_px), "bold" if b else "normal")


def hud_sizes(H):
    """The HUD's text sizes (pixels) for a window H high - the SAME 3 sizes (13n1): very important (speed, mode),
    heading (channel / latency / power, normal lines), info (small labels); x the "Aa" setting."""
    k = text_scale()
    imp, head, info = max(16, int(H * 0.060 * k)), max(11, int(H * 0.022 * k)), max(9, int(H * 0.0175 * k))
    return {"big": imp, "mode": imp, "comm": head, "norm": head, "small": info}


# --- colours: meaning (identical everywhere) ----------------------------------------------------------
WARN = "#ff3030"            # WARNING / alarm / danger (red)
CAUTION = "#ffb000"         # CAUTION (amber)
GOOD = "#3cff6a"            # all good / active (green)
ADVISORY = "#40c0ff"        # target heading, set speed, edit frames (blue)
MAGENTA = "#ff40ff"         # FIRST TEST, the CDI needle
WARN_DARK = "#800000"       # the alarm banner's flash phase
YELLOW = "#ffd000"          # pick-up / confirm marks
WHITE = "#ffffff"           # text on a coloured (dark) fill, the video picture's labels
BLACK = "#000000"           # text on a coloured (light) fill, text outlines, the video background
NIGHT = {"main": "#d84a30", "dim": "#8a3020", "warn": "#ff2a10", "caution": "#c08000", "good": "#a04020",
         "yellow": "#b07000", "grey": "#704030"}
MODE_COLOURS = {"STANDBY": "#606060", "MANUAL": "#ff8c00", "HDG_LOCK": "#007a00", "AUTO": "#7a00a0",
                "TRAINING": "#a06000", "REMOTE_MANUAL": "#0033cc", "BENCH_TEST": "#008080", "MANEUVER": "#b03060",
                "DP": "#006a8a"}
BADGE_FG = {"dark": "#ffffff", "light": "#ffffff"}
# banner colours (both screens): text colour + fill
BANNERS = {"alarm": ("#ffffff", WARN), "joystick": ("#000000", "#ff8c00"), "confirm": ("#000000", "#ffd000"),
           "remote": ("#ffffff", "#0033cc"), "pickup": ("#000000", "#ffd000"), "starlink": ("#ffffff", "#b00060"),
           "single_op": ("#ffffff", "#7000a0"), "first_test": ("#ffffff", "#c000c0"),
           "uncrewed": ("#ffffff", "#004080"), "lost": ("#ffffff", WARN), "boat": ("#000000", "#ffe080"),
           "maneuver": ("#ffffff", MODE_COLOURS["MANEUVER"]), "harbour": ("#000000", "#30d0b0")}
HUD_MAIN = {"green": GOOD, "white": "#ffffff"}             # the HUD's main colour (setting hud_colour)
HUD_DIM = {"green": "#b8c8b8", "white": "#c8c8c8"}
HUD_GREY = "#a0a0a0"
HUD_YELLOW = "#ffe000"
HUD_LIGHT = {"panel": "#e8eef4", "dim": "#2c3e50", "grey": "#5a6b7b"}     # light theme: light panels on the video
NOTICE = {"single_op": "#e080ff", "uncrewed": "#60a0ff", "first_test": MAGENTA, "harbour": "#30d0b0"}   # screen 1 notice lines
AR = {"route": "#ff50ff", "route_text": "#ff70ff", "shallow": "#ff2020"}         # AR overlays on the video
VISION = {"obstacle": "#f7c325", "water": "#29a7e0"}        # the obstacle-detection mask (eWaSR classes)
REC = "#ff2020"             # the recording dot
SCR_ON = "#ffffff"          # "●SCR": this PC records its screens (step 13n4, owner 2026-10-31 E4: white)
FROZEN = "#c00000"          # VIDEO FROZEN box
# harbour autonomy (step HX6): the fused tracks by the sensor that saw them (never red / amber: those are the meaning
# colours; the SHAPE says steer_ok), the corridor + the planned path
HARB_SRC = {"radar": "#c8a0ff", "lidar": "#00e0c0", "camera": "#d8b080", "ais": "#9aa8ff"}
HARB_CORRIDOR = ADVISORY    # blue: the corridor edges (a planning frame, like the edit frames)

# --- colours: the two themes (every value distinct inside a theme: retint() maps them 1:1) --------------
THEMES = {
    "dark": {"bg": "#0e141b", "panel": "#101a24", "head": "#182634", "line": "#2c3e50", "fg": "#e6edf3",
             "dim": "#8fa3b5", "faint": "#5b6d7e", "accent": "#4aa8ff", "btn": "#22313f", "btn_fg": "#e6edf4",
             "btn_active": "#2f4457", "btn_ok": "#1f5a32", "btn_no": "#3a4048", "btn_info": "#1f3f66",
             "btn_warn": "#5a4a18", "btn_sim": "#4a4020", "entry": "#0a1016", "entry_fg": "#f0f4f8",
             "sel": "#2a6aa0", "row_danger": "#5a3a00", "row_warn": "#504000", "row_stale": "#707070",
             "ok_txt": "#5be07a", "bad_txt": "#ffa040", "note_txt": "#ffcc55", "lit": "#2d8a4a",
             "plot_grid": "#34495e", "plot_line": "#e6edf2", "plot_target": "#4aa8fe", "plot_outboard": "#ff9a30",
             "prof_ok": "#2a5f80", "prof_bad": "#8a6020", "prof_bg": "#0b1219", "map": "dark"},
    "light": {"bg": "#f2f4f7", "panel": "#ffffff", "head": "#e4e9ef", "line": "#b8c2cc", "fg": "#101820",
              "dim": "#5a6b7b", "faint": "#93a1ae", "accent": "#0050c0", "btn": "#e2e7ec", "btn_fg": "#101821",
              "btn_active": "#cfd8e0", "btn_ok": "#b8f0c4", "btn_no": "#d4d8de", "btn_info": "#c8dcff",
              "btn_warn": "#ffe6a0", "btn_sim": "#fff4c0", "entry": "#fdfdfd", "entry_fg": "#000000",
              "sel": "#9cc4ff", "row_danger": "#ffd8a0", "row_warn": "#fff0a0", "row_stale": "#888888",
              "ok_txt": "#007a00", "bad_txt": "#b05000", "note_txt": "#a06000", "lit": "#7de07d",
              "plot_grid": "#d0d0d0", "plot_line": "#000001", "plot_target": "#0033cc", "plot_outboard": "#ff8c00",
              "prof_ok": "#b0d8f0", "prof_bad": "#ffe0b0", "prof_bg": "#fafafa", "map": "day"},
}
# how much of the map / video shows through (0 = solid): the screen 2 panels and the HUD panels
ALPHA = {"panel": 0.80, "panel_light": 0.84, "hud_panel": 0.55, "hud_panel_night": 0.73, "banner": 0.92}
# A SQUARE frame around everything that can be pressed or typed into (13n1, owner 2026-10-31 E1): radius 0
BUTTON = {"radius": 0.0, "pad_x": 0.70, "height": 1.85, "border": 2}       # x the button font size
FRAME = 2                           # px: the square frame of a button / entry field
# the colour code (13n1): red = alarms (WARNING) only, amber = CAUTION, red buttons = STANDBY and SAVE only (owner 2026-11-04); on the
# camera video (screen 1 HUD) every information GREEN; elsewhere information black or white by the background;
# the "connected" lamp white; NIGHT = the dim red (NIGHT below)
CONNECTED = ("#ffffff", "#000000")  # the "connected" lamp: fill, its text
# the "AI" badge (13n1): a small framed white "AI" ONLY where a neural network works (vision eWaSR, the Piper
# voice) - never on collision avoidance / the autopilot / the shallow-water checks
AI_BADGE = {"fill": "#ffffff", "fg": "#000000", "text": "AI"}
# the trend panel «ΤΑΣΕΙΣ» (step TR2, docs/trends_design.md 3): a dark "matrix" panel in BOTH themes, every value its
# OWN colour + line pattern (never red / amber - those are the meaning colours; NIGHT: NIGHT main / dim + the pattern)
TREND_BG = "#05080a"
TREND_GRID = "#14202a"
TREND_TEXT = "#c8d4dc"
TREND = {"sog": "#00e0c0", "thr": "#d0b0ff", "hdg": "#ff80c0", "rot": "#e8e8a0", "xte": "#8090ff",
         "roll": "#00a0ff", "pitch": "#a0ff40", "slam_g": "#f4f4f4", "slam_pm": "#a0a0a0", "depth": "#4060ff",
         "bank_v": "#80ffff", "link_ms": "#ffc0e0", "link_loss": "#90c060",
         # TR4: the three acceleration axes (BNO085 linear acceleration; display only), distinct from every other
         "acc_lon": "#c060ff", "acc_lat": "#b0ff90", "acc_vert": "#6080c0"}
# Tk dash tuples (on, off, ...) in px; () = solid
TREND_DASH = {"sog": (), "thr": (6, 3), "hdg": (), "rot": (2, 3), "xte": (), "roll": (6, 3), "pitch": (),
              "slam_g": (), "slam_pm": (2, 3), "depth": (), "bank_v": (), "link_ms": (), "link_loss": (6, 3, 2, 3),
              "acc_lon": (), "acc_lat": (6, 3), "acc_vert": (2, 3)}


def mix(c1, c2, f):
    """The colour f of the way from c1 to c2 ('#rrggbb'): a Tk canvas has no alpha, the glow of a trend line is the
    line colour mixed 25 % into the panel colour."""
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(int(round(x + (y - x) * f)) for x, y in zip(a, b))


# BATTLE SHORT (13n1, display only): the grey bold button with black / yellow hazard stripes
HAZARD = {"grey": "#8a8a8a", "black": "#000000", "yellow": "#ffd000", "red": "#d00000", "bg": "#120000"}
PANEL = {"radius": 8, "border": 1, "pad": 0.55, "gap": 0.35}                # pad / gap x the label font size
LOGO_H = (0.045, 26, 64)            # logo height: fraction of the window height, min px, max px

_listeners = []
C = dict(THEMES["dark"])


def name():
    t = settings.get("theme") or "dark"
    return t if t in THEMES else "dark"


def _load():
    C.clear()
    C.update(THEMES[name()])


def is_dark():
    return name() == "dark" or bool(settings.get("night"))


def map_theme():
    """The chart theme of both maps (mapview.THEMES): night > dark / light."""
    return "night" if settings.get("night") else THEMES[name()]["map"]


def panel_alpha():
    return ALPHA["panel"] if name() == "dark" else ALPHA["panel_light"]


def on_change(fn):
    """fn() after a theme switch (both screens redraw / retint)."""
    _listeners.append(fn)


def set_theme(t):
    """Dark / light for BOTH screens (saved per PC)."""
    if t not in THEMES or t == name():
        return
    old = dict(C)
    settings.put("theme", t)
    _load()
    for fn in list(_listeners):
        try:
            fn(old)
        except Exception:               # a closed window: forget it
            _listeners.remove(fn)


def toggle():
    set_theme("light" if name() == "dark" else "dark")


# --- tk widgets (dialogs, the tools window) ---------------------------------------------------------------
_OPTS = ("bg", "fg", "activebackground", "activeforeground", "highlightbackground", "highlightcolor",
         "selectcolor", "insertbackground", "disabledforeground", "selectbackground", "troughcolor")


def retint(widget, old, new=None):
    """After a theme switch: every widget colour that was a value of the OLD theme gets the new one."""
    new = new or C
    m = {v.lower(): new[k] for k, v in old.items() if isinstance(v, str) and v.startswith("#") and k in new}
    stack = [widget]
    while stack:
        w = stack.pop()
        try:
            for o in _OPTS:
                try:
                    v = str(w.cget(o)).lower()
                except Exception:
                    continue
                if v in m:
                    w.configure(**{o: m[v]})
            stack.extend(w.winfo_children())
        except Exception:
            continue


_SYS = ("systembuttontext", "systemwindowtext", "black", "#000000", "systembuttonface", "systemwindow", "white",
        "#ffffff", "#f0f0f0", "#d9d9d9", "systemhighlight", "systemmenu", "systemmenutext", "")


def style_tree(widget):
    """Widgets made without the theme's colours (the system defaults) get them: labels, check / radio
    buttons, entries, lists, scales, frames, canvases. Run after building a window / page / dialog."""
    stack = [widget]
    while stack:
        w = stack.pop()
        try:
            cls = w.winfo_class()
            opts = {}

            def sys_(o):
                try:
                    return str(w.cget(o)).lower() in _SYS
                except Exception:
                    return False
            if cls in ("Frame", "Toplevel", "Canvas", "Labelframe") and sys_("bg"):
                opts["bg"] = C["bg"]
            elif cls in ("Label", "Checkbutton", "Radiobutton", "Message"):
                if sys_("bg"):
                    opts["bg"] = C["bg"]
                if sys_("fg"):
                    opts["fg"] = C["fg"]
                if cls != "Label":
                    opts["selectcolor"] = C["entry"]
                    if sys_("activebackground"):
                        opts["activebackground"] = C["bg"]
                    opts["activeforeground"] = C["fg"]
            elif cls in ("Entry", "Spinbox", "Listbox", "Text"):
                if cls in ("Entry", "Spinbox"):         # a square frame around a field to type into (13n1)
                    opts["relief"] = "solid"
                    opts["bd"] = 1
                if sys_("bg"):
                    opts["bg"] = C["entry"]
                if sys_("fg"):
                    opts["fg"] = C["entry_fg"]
                if cls != "Listbox":
                    opts["insertbackground"] = C["entry_fg"]
            elif cls == "Button":
                if sys_("bg"):
                    opts["bg"] = C["btn"]
                if sys_("fg"):
                    opts["fg"] = C["btn_fg"]
            elif cls == "Scale":
                if sys_("bg"):
                    opts["bg"] = C["bg"]
                if sys_("fg"):
                    opts["fg"] = C["fg"]
                opts["troughcolor"] = C["entry"]
                opts["highlightthickness"] = 0
            if opts:
                w.configure(**opts)
            stack.extend(w.winfo_children())
        except Exception:
            continue


def apply_ttk(root):
    """The ttk widgets (tables, tabs, drop-downs) in the theme."""
    from tkinter import ttk
    st = ttk.Style(root)
    try:
        st.theme_use("clam")
    except Exception:
        pass
    st.configure(".", background=C["bg"], foreground=C["fg"], fieldbackground=C["entry"], font=font("label"))
    st.configure("Treeview", background=C["panel"], fieldbackground=C["panel"], foreground=C["fg"],
                 font=font("label"), rowheight=int(PT["label"] * 2.4))
    st.configure("Treeview.Heading", background=C["head"], foreground=C["fg"], font=font("button"))
    st.map("Treeview", background=[("selected", C["sel"])])
    st.configure("TNotebook", background=C["bg"])
    st.configure("TNotebook.Tab", background=C["btn"], foreground=C["btn_fg"], font=font("button"), padding=(8, 5))
    st.map("TNotebook.Tab", background=[("selected", C["panel"])])
    st.configure("TCombobox", fieldbackground=C["entry"], foreground=C["entry_fg"], background=C["btn"])
    root.option_add("*TCombobox*Listbox.background", C["entry"])
    root.option_add("*TCombobox*Listbox.foreground", C["entry_fg"])


def button_opts(kind="normal", level="button"):
    """Options of a tk.Button of this kind: normal / ok / no / info / warn / standby / sim."""
    bg = {"normal": C["btn"], "ok": C["btn_ok"], "no": C["btn_no"], "info": C["btn_info"], "warn": C["btn_warn"],
          "sim": C["btn_sim"], "standby": WARN}.get(kind, C["btn"])
    return {"bg": bg, "fg": "#ffffff" if kind == "standby" else C["btn_fg"], "activebackground": C["btn_active"],
            "activeforeground": C["btn_fg"], "font": font(level), "relief": "solid", "bd": 1,
            "highlightthickness": BUTTON["border"] - 1, "highlightbackground": C["fg"], "padx": 10, "pady": 4,
            "cursor": "hand2"}


def yn_opts(yes, selected, level="button"):
    """A YES / NO answer button of the ONE vertical Y/N layout (step 13p, owner 2026-10-31 D: question, [YES] on top,
    [NO] underneath, full width, framed; the selected one inverted = filled in its meaning colour)."""
    col = C["btn_ok"] if yes else C["btn_no"]
    o = button_opts("ok" if yes else "no", level)
    if selected:
        o.update({"bg": col, "fg": C["btn_fg"], "highlightthickness": 4, "highlightbackground": C["fg"],
                  "highlightcolor": C["fg"], "relief": "flat", "bd": 3})
    else:                                   # (the frame drawn by the border: a Tk highlight ring alone may not show)
        o.update({"bg": C["panel"], "fg": C["fg"], "highlightthickness": 3, "highlightbackground": col,
                  "highlightcolor": col, "relief": "solid", "bd": 3})
    return o


def ai_badge(parent, bg=None):
    """The small framed white "AI" badge as a tk Label (13n1): ONLY where a neural network works."""
    import tkinter as tk
    return tk.Label(parent, text=AI_BADGE["text"], font=font("small", bold=True), bg=AI_BADGE["fill"],
                    fg=AI_BADGE["fg"], relief="solid", bd=1, padx=3, pady=0)


def label_opts(level="label", dim=False):
    return {"bg": C["bg"], "fg": C["dim"] if dim else C["fg"], "font": font(level)}


# --- Windows scaling ---------------------------------------------------------------------------------------
def dpi_aware():
    """Sharp text at 125 / 150 % Windows scaling (before the first window). Harmless elsewhere."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        try:
            import ctypes
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


def dpi_factor(root):
    """The Windows display scaling of this window: 1.0 (100 %), 1.25, 1.5 ..."""
    try:
        return max(1.0, min(2.0, float(root.tk.call("tk", "scaling")) * 72.0 / 96.0))
    except Exception:
        return 1.0


# --- the logo ----------------------------------------------------------------------------------------------
LOGO_FILES = ("trygonsdrones_logo.png", "trygonsdrones_logo.svg")
LOGO_TEXT = "TRYGONS DRONES"         # the neutral stand-in when no logo file / no Pillow


def logo_path(dark=None):
    """The logo for a dark (light lettering) or a light background; None if there is none."""
    dark = is_dark() if dark is None else dark
    p = os.path.join(ASSETS, "trygonsdrones_logo_dark.png" if dark else "trygonsdrones_logo.png")
    if os.path.exists(p):
        return p
    p = os.path.join(ASSETS, "trygonsdrones_logo.png")
    return p if os.path.exists(p) else None


def logo_height(W, H):
    f, lo, hi = LOGO_H
    return int(max(lo, min(hi, H * f)))


_logo_cache = {}


def logo_pil(height, dark=None):
    """The logo as a Pillow RGBA image `height` px high (aspect kept), or None (no Pillow / no file)."""
    p = logo_path(dark)
    if p is None:
        return None
    key = (p, int(height))
    if key not in _logo_cache:
        try:
            from PIL import Image
            im = Image.open(p).convert("RGBA")
            w = max(1, int(round(im.width * height / float(im.height))))
            _logo_cache[key] = im.resize((w, int(height)), Image.LANCZOS)
        except Exception:
            _logo_cache[key] = None
    return _logo_cache[key]


def logo_size(height, dark=None):
    im = logo_pil(height, dark)
    return (im.width, im.height) if im is not None else (int(height * 3.85), int(height))


# --- the app icon + the start-up screen (step 13i) ------------------------------------------------------
APP_NAME = "Trygons C2"             # the product name (never translated): window titles, splash, About
APP_ID = "TrygonsDrones.TrygonsC2"  # Windows: one taskbar group with our icon (not python.exe's)
ICON_ICO = "trygons_c2.ico"         # tools/make_icon.py from assets/trygonsdrones_mark.svg
ICON_PNG = "trygonsdrones_mark.png"
SPLASH = {"bg": "#0b1a2e", "fg": "#e6edf3", "dim": "#8fa3b5", "accent": "#4aa8ff", "bar": "#1c3350",
          "error": "#ff6b6b"}


def icon_path(kind="ico"):
    """assets/trygons_c2.ico ("ico") or the 256 px PNG ("png"); None when the file is missing."""
    p = os.path.join(ASSETS, ICON_ICO if kind == "ico" else ICON_PNG)
    return p if os.path.exists(p) else None


def set_app_id():
    """Windows: the taskbar shows OUR icon and groups our windows (before the first window). Harmless elsewhere."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
    except Exception:
        pass


def set_app_icon(win):
    """The app icon on this window AND every window made after it (Tk default): the .ico on Windows
    (title bar + taskbar), the PNG elsewhere. A missing file / no display: nothing (never an error)."""
    try:
        root = win._root()              # the Tk of this application: one icon for all its windows
        if getattr(root, "_trygons_icon", None) is not None:
            return root._trygons_icon
        ico, png = icon_path("ico"), icon_path("png")
        img = None
        if png:
            import tkinter as tk
            img = tk.PhotoImage(master=root, file=png)
            root.iconphoto(True, img)
        if ico and sys.platform == "win32":
            root.iconbitmap(default=ico)
        root._trygons_icon = img or ico or False
        return root._trygons_icon
    except Exception:
        return None


_load()
