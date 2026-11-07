"""Term hints for the speech recognition (step AM2): the C2's own words - mode names, the mode bar, the bars'
buttons, panel titles, Tools page names, alarm titles - read from groundstation/lang/el.txt + en.txt, so a
spoken «ΜΑΓΓΕΜΑ», «ΚΑΤΑΠΑΚΤΗ», "HDG LOCK" or "DP" is recognised as written on the screen.

Pure: reads the two text files only (no station module, no network). The result is Whisper's initial
prompt (worker.py); its length is kept small (Whisper uses at most ~220 tokens of it).
"""
import os
import re

from . import paths

# which lang keys give useful words, most useful first (normal-case texts only: the screens' capitals have no
# accents - a prompt in capitals makes Whisper write and mis-hear in capitals, AM2 measurement)
PREFIXES = ("tab.", "ov.b_", "hud.", "ov.", "uav.", "hail.", "track.", "cv.")
EXTRA = ("HDG LOCK", "AUTO", "STANDBY", "DP", "MANEUVER", "REMOTE", "UAV", "AIS", "GPS", "HELM", "SAVE", "KEEP",
         "CPA", "TCPA", "AURIX", "gateway", "joystick", "TWCS")
# the C2 words an operator says most, in normal case with accents (the screens show them in capitals)
EXTRA_LANG = {
    "el": ("καταπακτή", "μάγγεμα", "κλείδωμα πορείας", "τιμόνι", "σχέση", "μεγάφωνο", "ομιλία", "συναγερμός",
           "επιβεβαίωση", "όριο ισχύος", "κυβερνήτης", "συγκυβερνήτης", "τηλεχειρισμός", "μανούβρα", "χειριστήριο",
           "κάμερες", "διαδρομή", "γεωφράχτης", "ρηχά νερά", "κίνδυνος σύγκρουσης", "απώλεια σύνδεσης",
           "μηχανή", "μίζα", "ρελέ", "γεννήτρια", "μπαταρία", "επαναφορά ισχύος", "πυξίδα", "εγχειρίδιο"),
    "en": ("hatch", "magazine", "heading lock", "helm", "gear", "loudhailer", "talk", "alarm", "confirm",
           "power limit", "captain", "copilot", "remote", "maneuver", "joystick", "cameras", "route", "geofence",
           "shallow water", "collision risk", "link lost", "engine", "starter", "relay", "generator", "battery",
           "restore power", "compass", "manual")}
MAX_WORDS = 4                       # a hint is a short name, not a sentence
MAX_CHARS = 700                     # per language (the prompt budget)


def read_lang(lang, base=None):
    """{key: text} of lang/<lang>.txt (the i18n file format: key = text, # comments)."""
    p = os.path.join(base or os.path.join(paths.GS, "lang"), lang + ".txt")
    out = {}
    try:
        with open(p, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    except OSError:
        pass
    return out


def _clean(v):
    if "{" in v or "\\n" in v or len(v) < 2:
        return None
    v = re.sub(r"\s*\(.*?\)", "", v).strip(" :·-–.,")      # "(συστοιχία 12 V)" etc. off
    v = v.split(":")[0].strip()
    if not v or len(v.split()) > MAX_WORDS or not re.search(r"\w", v):
        return None
    return v


def terms(lang, base=None):
    """The hint terms of one language, most useful first, no duplicates."""
    d = read_lang(lang, base)
    out, seen = [], set()

    def add(t):
        k = t.lower()
        if t and k not in seen:
            seen.add(k)
            out.append(t)
    for t in EXTRA + EXTRA_LANG.get(lang, ()):
        add(t)
    for pre in PREFIXES:
        for k, v in d.items():
            if k.startswith(pre):
                c = _clean(v)
                if c and not (c.isupper() and lang == "el" and re.search("[Ͱ-Ͽ]", c)):
                    add(c)
    return out


def natural(t):
    """«ΚΑΤΑΠΑΚΤΗ» -> «καταπακτή»-like normal case for the prompt: a prompt in capitals makes Whisper write (and
    mis-hear) in capitals (AM2 measurement). Latin acronyms (HDG LOCK, DP, UAV) stay as written."""
    out = []
    for w in t.split():
        if any("Ͱ" <= c <= "Ͽ" or "ἀ" <= c <= "῿" for c in w) and w.isupper():
            w = w.lower().replace("σ ", "ς ")
            if w.endswith("σ"):
                w = w[:-1] + "ς"
        out.append(w)
    return " ".join(out)


def prompt(lang, base=None, max_chars=MAX_CHARS):
    """Whisper's initial prompt: the terms (normal case) joined, cut at max_chars (whole terms only)."""
    s = ""
    for t in (natural(x) for x in terms(lang, base)):
        nxt = (s + ", " + t) if s else t
        if len(nxt) > max_chars:
            break
        s = nxt
    return s + "." if s else ""
