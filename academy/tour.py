"""Guided tour model (ACADEMY_SPEC B, step 17): the stops of content/tour.json, their spoken
sentences, the voice files and the coverage check.

Pure model - no window, no network, no process (test_academy AC1-AC3). The window is tour_app.py.
The ground station (groundstation/academymenu.py) loads THIS file by its path to prepare the voice
with its own offline TTS (Piper, sound.Library.make); the academy then only PLAYS the cached .wav
files (tour_app.py through simguard.check_recording) - it never runs the TTS itself (no process).

Imports only the ground station's display tables (i18n, alerts, proto, hud): no link module.

    python academy/tour.py            -> coverage report (exit 1 when an item is not covered)
"""
import hashlib
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GS_DIR = os.path.join(REPO, "groundstation")
if GS_DIR not in sys.path:
    sys.path.append(GS_DIR)

import alerts                                                           # noqa: E402
import hud                                                              # noqa: E402
import i18n                                                             # noqa: E402
import proto                                                            # noqa: E402

TOUR_FILE = os.path.join(HERE, "content", "tour.json")
LANGS = ("el", "en")
SHOWS = ("none", "s1", "s2", "tools", "modes", "alarm", "lamps", "joystick", "panel")
# screen 2 parts that are not an ov.panel(...) in app.py (STANDBY corner, banners, map bar, the boat, editing)
S2_EXTRA = ("standby", "banners", "mapbar", "boat", "edit")
HUD_EXTRA = ("logo", "center")              # LOGO_BOX + the centre (ACCEPT / REJECT questions)
HUD_BUTTONS_EXTRA = ("save", "accept")      # SAVE (bottom line) + the ACCEPT / REJECT answer (driver.py)
LAMPS = ("master_warning", "master_caution")
JOYSTICK = ("x", "y", "take", "auto", "confirm", "usb")
PANEL = ("standby", "hdg_lock", "minus1", "plus1", "minus10", "plus10", "estop")
# the DRAWINGS' layout (fractions of a 16:9 picture) - shared by the tour window and the quiz picture
# questions (step 18: quiz.area_rect); the real places are decided by app.py / overlay.py
S2_BOXES = {
    "status": (0.010, 0.010, 0.250, 0.300),
    "banners": (0.300, 0.010, 0.400, 0.060),
    "alarms": (0.740, 0.010, 0.250, 0.140),
    "helm": (0.740, 0.160, 0.250, 0.120),
    "link": (0.740, 0.290, 0.250, 0.140),
    "relays": (0.010, 0.330, 0.200, 0.200),
    "control": (0.010, 0.550, 0.300, 0.340),
    "standby": (0.010, 0.905, 0.150, 0.085),
    "traffic": (0.740, 0.600, 0.250, 0.260),
    "logo": (0.840, 0.875, 0.150, 0.110),
    "mapbar": (0.330, 0.905, 0.380, 0.085),
}
# the panel buttons drawing (label, target), left to right, then the e-stop
PANEL_BTNS = (("STANDBY", "standby"), ("HDG LOCK", "hdg_lock"), ("-10", "minus10"), ("-1", "minus1"),
              ("+1", "plus1"), ("+10", "plus10"))
ESTOP_C, ESTOP_R = (0.86, 0.48), 0.17      # centre (fractions), radius (fraction of the height)


def panel_rect(key):
    """(x, y, w, h) fractions of a panel button (or the e-stop) in the drawing."""
    if key == "estop":
        rx, ry = ESTOP_R * 1.3 * 9 / 16.0, ESTOP_R * 1.3
        return (ESTOP_C[0] - rx, ESTOP_C[1] - ry, 2 * rx, 2 * ry)
    bw = 0.70 / len(PANEL_BTNS)
    for i, (_lab, k) in enumerate(PANEL_BTNS):
        if k == key:
            return (0.04 + i * bw, 0.35, bw - 0.015, 0.25)
    raise KeyError(key)


def load(path=None):
    with open(path or TOUR_FILE, encoding="utf-8") as f:
        return json.load(f)


# --- what must be covered (from the ground station's own sources: nothing is forgotten) ----------
def _src(name):
    with open(os.path.join(GS_DIR, name), encoding="utf-8") as f:
        return f.read()


def hud_buttons():
    """The action names of the HUD's screen buttons (hud.Hud.controls) and drive buttons (Hud.drive)."""
    src = _src("hud.py")
    acts = set()
    for fn in ("controls", "drive"):
        body = src.split("    def %s(self" % fn, 1)[1].split("\n    def ", 1)[0]
        acts |= set(re.findall(r"[\)\]], \"([a-z_]+)\", (?:False|True|bool|self)", body))
    return sorted(acts)


def s2_panels():
    """The corner panels of screen 2 (app.py: self.ov.panel("<name>", ...))."""
    return sorted(set(re.findall(r"self\.ov\.panel\(\"([a-z_]+)\"", _src("app.py"))))


def tools_pages():
    """The pages of the tools window (app.py _build_tools: tab("tab.<x>", ...))."""
    return re.findall(r"\btab\(\"(tab\.[a-z_]+)\"", _src("app.py"))


def required_items():
    """Every item the tour must explain (ACADEMY_SPEC B: every HUD item, button, lamp, alarm, mode,
    joystick, panel button; screen 2 + the tools window)."""
    req = ["hud:" + n for n in hud.OVERLAYS] + ["hud:logo"]
    req += ["hudbtn:" + a for a in hud_buttons()] + ["hudbtn:" + a for a in HUD_BUTTONS_EXTRA]
    req += ["s2:" + p for p in s2_panels()] + ["s2:" + p for p in S2_EXTRA]
    req += ["tools:" + t for t in tools_pages()]
    req += ["mode:" + m for m in proto.MODES]
    req += ["alarm:" + code for code, _lv in alerts.ALARM_LEVELS.values()] + ["alarm:" + alerts.NO_DATA]
    req += ["lamp:" + x for x in LAMPS] + ["joy:" + x for x in JOYSTICK] + ["panel:" + x for x in PANEL]
    out = []
    for r in req:
        if r not in out:
            out.append(r)
    return out


def alarm_level(code):
    if code == alerts.NO_DATA:
        return "WARNING"
    for c, lv in alerts.ALARM_LEVELS.values():
        if c == code:
            return alerts.LEVEL_NAMES[lv]
    return None


def valid_targets(show):
    if show == "s1":
        return set(hud.OVERLAYS) | set(HUD_EXTRA) | {""}
    if show == "s2":
        return set(s2_panels()) | set(S2_EXTRA)
    if show == "tools":
        return set(tools_pages())
    if show == "modes":
        return set(proto.MODES)
    if show == "alarm":
        return {c for c, _ in alerts.ALARM_LEVELS.values()} | {alerts.NO_DATA}
    if show == "lamps":
        return set(LAMPS)
    if show == "joystick":
        return set(JOYSTICK)
    if show == "panel":
        return set(PANEL)
    return {""}


def check(doc):
    """Problems of a tour document (empty = OK): kind, unique ids, known show / target / covers."""
    p = []
    if not isinstance(doc, dict) or doc.get("kind") != "tour":
        return ["not a tour document"]
    chapters = doc.get("chapters")
    if not isinstance(chapters, list) or not chapters:
        return ["no chapters"]
    seen = set()
    known = set(required_items())
    for ch in chapters:
        for st in ch.get("stops") or []:
            sid = st.get("id")
            if not sid or sid in seen:
                p.append("stop id %r missing or not unique" % sid)
            seen.add(sid)
            show = st.get("show")
            if show not in SHOWS:
                p.append("%s: unknown show %r" % (sid, show))
                continue
            if st.get("target", "") not in valid_targets(show):
                p.append("%s: unknown target %r for %s" % (sid, st.get("target"), show))
            for c in st.get("covers") or []:
                if c not in known:
                    p.append("%s: covers unknown item %r" % (sid, c))
            for lc in LANGS:
                if show != "alarm" and not (st.get("title_" + lc) or "").strip():
                    p.append("%s: no title_%s" % (sid, lc))
                if not (st.get("text_" + lc) or "").strip():
                    p.append("%s: no text_%s" % (sid, lc))
    for lv in ("WARNING", "CAUTION", "ADVISORY"):
        for lc in LANGS:
            if not ((doc.get("levels") or {}).get(lv) or {}).get("text_" + lc):
                p.append("levels.%s: no text_%s" % (lv, lc))
    return p


def missing(doc):
    """Required items no stop covers."""
    have = set()
    for ch in doc.get("chapters") or []:
        for st in ch.get("stops") or []:
            have |= set(st.get("covers") or [])
    return [r for r in required_items() if r not in have]


# --- the stops in one language -------------------------------------------------------------------
def stops(lang, doc=None):
    """Every stop in order, in the language: dict id / chapter / chapter_title / show / target /
    title / text / level (alarm stops: the WARNING / CAUTION / ADVISORY name, else None)."""
    doc = doc or load()
    lang = lang if lang in LANGS else "el"
    out = []
    for ch in doc["chapters"]:
        for st in ch["stops"]:
            lv = alarm_level(st["target"]) if st["show"] == "alarm" else None
            title = st.get("title_" + lang)
            if st["show"] == "alarm":                     # the screen's own words for the alarm
                title = i18n.TL(lang, "alarm." + st["target"])
            out.append({"id": st["id"], "chapter": ch["id"], "chapter_title": ch["title_" + lang],
                        "show": st["show"], "target": st.get("target", ""), "title": title,
                        "text": st["text_" + lang], "level": lv,
                        "level_text": doc["levels"][lv]["text_" + lang] if lv else None})
    return out


def sentence(stop):
    """What the voice says for a stop: title. [level.] text"""
    parts = [stop["title"].strip().rstrip(".")]
    if stop.get("level_text"):
        parts.append(stop["level_text"].strip().rstrip("."))
    parts.append(stop["text"].strip())
    s = ". ".join(p for p in parts if p)
    return s if s.endswith((".", "!", "?")) else s + "."


def sentences(lang, doc=None):
    return [sentence(s) for s in stops(lang, doc)]


def voice_cache(env=None):
    """The ground station's sound cache (sound.Library: TRYGONS_SOUND_CACHE or groundstation/sound_cache)."""
    env = os.environ if env is None else env
    return env.get("TRYGONS_SOUND_CACHE") or os.path.join(GS_DIR, "sound_cache")


def voice_path(lang, text, cache=None):
    """The .wav the ground station's TTS makes for this sentence (= sound.Library.sentence_path)."""
    h = hashlib.sha1(text.encode("utf-8")).hexdigest()[:20]
    return os.path.join(cache or voice_cache(), lang, "p_%s.wav" % h)


def main():
    doc = load()
    probs, miss = check(doc), missing(doc)
    for x in probs:
        print("PROBLEM", x)
    for x in miss:
        print("NOT COVERED", x)
    n = sum(len(c["stops"]) for c in doc["chapters"])
    print("%d stops, %d required items, %d problems, %d not covered" % (n, len(required_items()), len(probs),
                                                                          len(miss)))
    return 1 if probs or miss else 0


if __name__ == "__main__":
    sys.exit(main())
