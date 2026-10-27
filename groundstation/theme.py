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
PT = {"title": 16, "value": 14, "label": 11, "button": 11, "small": 9, "huge": 24, "code": 36}
BOLD = {"title": True, "value": True, "label": False, "button": True, "small": False, "huge": True, "code": True}
# drawn text in pixels at a 1920 x 1080 window (scaled with the window, never below MIN_PX)
PX = {"title": 24, "value": 19, "label": 15, "button": 15, "small": 13, "huge": 30}
MIN_PX = {"title": 15, "value": 13, "label": 11, "button": 11, "small": 10, "huge": 17}


def font(level="label", bold=None, mono=False):
    """A tk font for widgets: (family, points, weight). Points follow the Windows scaling."""
    b = BOLD.get(level, False) if bold is None else bold
    return (MONO if mono else FAMILY, PT.get(level, PT["label"]), "bold" if b else "normal")


def unit(W, H):
    """The size factor of a window against 1920 x 1080 (0.62 .. 1.6)."""
    return max(0.62, min(1.6, min(W / 1920.0, H / 1080.0)))


def px(level, W, H, k=1.0):
    """A drawn font size in pixels for a W x H window; k = Windows scaling (dpi_factor) x fit factor."""
    return int(round(max(MIN_PX.get(level, 10), PX.get(level, 15) * unit(W, H) * k)))


def tkfont(level, size_px, bold=None, mono=False):
    """A canvas font in PIXELS (negative size): (family, -px, weight)."""
    b = BOLD.get(level, False) if bold is None else bold
    return (MONO if mono else FAMILY, -int(size_px), "bold" if b else "normal")


def hud_sizes(H):
    """The HUD's text sizes (pixels) for a window H high: big (speed), mode, comm, norm, small."""
    return {"big": max(18, int(H * 0.075)), "mode": max(16, int(H * 0.050)), "comm": max(12, int(H * 0.030)),
            "norm": max(11, int(H * 0.022)), "small": max(9, int(H * 0.0175))}


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
           "maneuver": ("#ffffff", MODE_COLOURS["MANEUVER"])}
HUD_MAIN = {"green": GOOD, "white": "#ffffff"}             # the HUD's main colour (setting hud_colour)
HUD_DIM = {"green": "#b8c8b8", "white": "#c8c8c8"}
HUD_GREY = "#a0a0a0"
HUD_YELLOW = "#ffe000"
HUD_LIGHT = {"panel": "#e8eef4", "dim": "#2c3e50", "grey": "#5a6b7b"}     # light theme: light panels on the video
NOTICE = {"single_op": "#e080ff", "uncrewed": "#60a0ff", "first_test": MAGENTA}   # screen 1 notice lines
AR = {"route": "#ff50ff", "route_text": "#ff70ff", "shallow": "#ff2020"}         # AR overlays on the video
VISION = {"obstacle": "#f7c325", "water": "#29a7e0"}        # the obstacle-detection mask (eWaSR classes)
REC = "#ff2020"             # the recording dot
FROZEN = "#c00000"          # VIDEO FROZEN box

# --- colours: the two themes (every value distinct inside a theme: retint() maps them 1:1) --------------
THEMES = {
    "dark": {"bg": "#0e141b", "panel": "#101a24", "head": "#182634", "line": "#2c3e50", "fg": "#e6edf3",
             "dim": "#8fa3b5", "faint": "#5b6d7e", "accent": "#4aa8ff", "btn": "#22313f", "btn_fg": "#e6edf4",
             "btn_active": "#2f4457", "btn_ok": "#1f5a32", "btn_no": "#6a2020", "btn_info": "#1f3f66",
             "btn_warn": "#6a5010", "btn_sim": "#4a4020", "entry": "#0a1016", "entry_fg": "#f0f4f8",
             "sel": "#2a6aa0", "row_danger": "#601010", "row_warn": "#504000", "row_stale": "#707070",
             "ok_txt": "#5be07a", "bad_txt": "#ff6b6b", "note_txt": "#ffcc55", "lit": "#2d8a4a",
             "plot_grid": "#34495e", "plot_line": "#e6edf2", "plot_target": "#4aa8fe", "plot_outboard": "#ff9a30",
             "prof_ok": "#2a5f80", "prof_bad": "#a04040", "prof_bg": "#0b1219", "map": "dark"},
    "light": {"bg": "#f2f4f7", "panel": "#ffffff", "head": "#e4e9ef", "line": "#b8c2cc", "fg": "#101820",
              "dim": "#5a6b7b", "faint": "#93a1ae", "accent": "#0050c0", "btn": "#e2e7ec", "btn_fg": "#101821",
              "btn_active": "#cfd8e0", "btn_ok": "#b8f0c4", "btn_no": "#ffc4c4", "btn_info": "#c8dcff",
              "btn_warn": "#ffe6a0", "btn_sim": "#fff4c0", "entry": "#fdfdfd", "entry_fg": "#000000",
              "sel": "#9cc4ff", "row_danger": "#ffb0b0", "row_warn": "#fff0a0", "row_stale": "#888888",
              "ok_txt": "#007a00", "bad_txt": "#d00000", "note_txt": "#a06000", "lit": "#7de07d",
              "plot_grid": "#d0d0d0", "plot_line": "#000001", "plot_target": "#0033cc", "plot_outboard": "#ff8c00",
              "prof_ok": "#b0d8f0", "prof_bad": "#ffb0b0", "prof_bg": "#fafafa", "map": "day"},
}
# how much of the map / video shows through (0 = solid): the screen 2 panels and the HUD panels
ALPHA = {"panel": 0.80, "panel_light": 0.84, "hud_panel": 0.55, "hud_panel_night": 0.73, "banner": 0.92}
BUTTON = {"radius": 0.22, "pad_x": 0.70, "height": 1.85, "border": 1}      # x the button font size
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
            "activeforeground": C["btn_fg"], "font": font(level), "relief": "flat", "bd": 0,
            "highlightthickness": BUTTON["border"], "highlightbackground": C["line"], "padx": 10, "pady": 4,
            "cursor": "hand2"}


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


_load()
