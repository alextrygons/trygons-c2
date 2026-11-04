"""Per-PC settings of the ground station (gs_settings.json next to app.py, not in git).

Screen language, voice language, HUD colour / night / declutter / video
scaling, the AR overlay switches and the sound settings. DISPLAY ONLY: nothing
in here changes how the boat is driven.

    import settings
    settings.get("ui_lang")          -> "el"
    settings.put("night", True)      (written to the file at once)

TRYGONS_GS_SETTINGS=<file> uses another file (the tests use a temporary one).
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULTS = {
    "ui_lang": "el",            # screen language: el (default) / en
    "theme": "dark",            # dark / light: BOTH screens (theme.py)
    "voice_lang": "el",         # voice callout language (may differ from the screen)
    "hud_colour": "green",      # main HUD colour: green (13n1, owner 2026-10-31: all info on the video GREEN)
    "text_size": "normal",      # "Aa" in every window: small / normal / large (theme.text_scale, 13n1)
    "s2_panel_pos": {},         # screen 2 panels dragged by the user: key -> [x, y] fractions (13n1, RESET TO DEFAULT)
    "s2_bars": {"hidden": [], "dock": "bottom"},   # screen 2 bottom bars: hidden ones, dock top / bottom (13n1)
    "night": False,             # night mode (dim red / dark)
    "declutter": "FULL",        # FULL / NORMAL / MINIMAL
    "video_scale": "fill",      # fill (cover, crop evenly) / fit (whole picture, black bars)
    "ar": {"horizon": True, "velocity": True, "waypoint": True, "ribbon": True, "ais": True, "path": True,
           "shallow": True},
    "render_max_px": 1920,      # screen 1 is composed at most this wide / high, shown scaled (high-DPI, step 13l)
    "vision_overlay": True,     # obstacle mask on the front video
    "vision_opacity": 1,        # 0 / 1 / 2 = 25 / 50 / 75 %
    "volume": 80,               # master volume 0..100 (warnings never below WARN_MIN_VOLUME)
    "mute_advisory": False,
    "mute_feedback": False,
    "quiet": False,             # night / quiet sound mode: softer, fewer advisories
    "repeat_s": 20.0,           # the same callout is not repeated within this time
    "sound": True,              # sound + voice on this PC
    "voice_engine": "piper",    # piper (offline AI voice, default) / windows (SAPI voice, e.g. Greek Stefanos) (13o)
    "map": {"theme": "dark", "course_up": False, "m_per_px": 10.0},
}
_data = None


def path():
    return os.environ.get("TRYGONS_GS_SETTINGS") or os.path.join(HERE, "gs_settings.json")


def _load():
    global _data
    _data = json.loads(json.dumps(DEFAULTS))
    try:
        with open(path(), encoding="utf-8") as f:
            saved = json.load(f)
        for k, v in saved.items():
            if isinstance(v, dict) and isinstance(_data.get(k), dict):
                _data[k].update(v)
            else:
                _data[k] = v
    except (OSError, ValueError):
        pass
    return _data


def get(key, default=None):
    d = _data if _data is not None else _load()
    return d.get(key, default)


def put(key, value):
    d = _data if _data is not None else _load()
    d[key] = value
    try:
        with open(path(), "w", encoding="utf-8") as f:
            json.dump(d, f, indent=1, ensure_ascii=False)
    except OSError:
        pass


def reload():
    """Read the file again (tests)."""
    return _load()
