"""Screen language of the ground station (section E): Greek (default) and English.

Every user-visible text comes from the language files lang/el.txt and
lang/en.txt (plain text, one entry per line, comments with #):

    # the big mode name on the driver screen
    mode.STANDBY = STANDBY

T("mode.STANDBY") gives the text in the current language; {name} parts are
filled from the arguments: T("cap.log", path="x.log"). A key missing in the
current file falls back to English, then to "[key]" (and is remembered in
MISSING - the tests check that no key is missing).

Live switching: set_lang("en") changes the language at once; widgets made
with bind() are updated, other code (canvas drawings) calls T() on every
redraw anyway, and listeners (on_change) are told.

Logs, the protocol, recordings and test reports stay in English + event
codes (E3): only the screens use this module.
"""
import os
import re
import weakref

import settings

HERE = os.path.dirname(os.path.abspath(__file__))
LANG_DIR = os.path.join(HERE, "lang")
LANGS = ("el", "en")
NAMES = {"el": "ΕΛ", "en": "EN"}

_tables = {}
_lang = None
_bound = []                 # (weakref widget, key, option, kwargs)
_listeners = []
MISSING = set()
_LINE = re.compile(r"^([A-Za-z0-9_.\-]+)\s*=\s?(.*)$")


def parse(text):
    """The contents of a language file -> {key: text}. "\\n" in a text = a new line."""
    out = {}
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = _LINE.match(line)
        if not m:
            raise ValueError("line %d: not 'key = text': %r" % (n, raw))
        out[m.group(1)] = m.group(2).replace("\\n", "\n")
    return out


def load(lang):
    if lang not in _tables:
        try:
            with open(os.path.join(LANG_DIR, lang + ".txt"), encoding="utf-8") as f:
                _tables[lang] = parse(f.read())
        except OSError:
            _tables[lang] = {}
    return _tables[lang]


def lang():
    global _lang
    if _lang is None:
        env = os.environ.get("TRYGONS_LANG")            # tests / screenshots: a fixed language
        _lang = env if env in LANGS else settings.get("ui_lang") if settings.get("ui_lang") in LANGS else "el"
    return _lang


def T(key, *args, **kw):
    s = load(lang()).get(key)
    if s is None:
        MISSING.add((lang(), key))
        s = load("en").get(key)
        if s is None:
            return "[%s]" % key
    if args or kw:
        try:
            return s.format(*args, **kw)
        except (KeyError, IndexError, ValueError):
            return s
    return s


def TE(key, *args, **kw):
    """The ENGLISH text of a key: for what goes into the protocol / audit / logs (E3)."""
    s = load("en").get(key, "[%s]" % key)
    if args or kw:
        try:
            return s.format(*args, **kw)
        except (KeyError, IndexError, ValueError):
            return s
    return s


_rx = {}


def from_en(text, prefixes=("what.", "gw.")):
    """An English text made from one of the templates "what.*" / "gw.*" (a command
    description, a gateway answer) -> the same text in the screen language. The
    English text is what travels in the protocol / audit log; only the screen
    shows the translation. Unknown text -> unchanged."""
    if not text or lang() == "en":
        return text
    en = load("en")
    for key, tpl in en.items():
        if not key.startswith(prefixes):
            continue
        rx = _rx.get(key)
        if rx is None:
            parts = re.split(r"\{(\w+)(?:![rs])?(?::[^}]*)?\}", tpl)
            pat = "".join(re.escape(p) if i % 2 == 0 else "(?P<%s>.+?)" % p for i, p in enumerate(parts))
            rx = _rx[key] = re.compile("^" + pat + "$", re.S)
        m = rx.match(text)
        if m:
            return T(key, **m.groupdict())
    return text


def has(key, lang_=None):
    return key in load(lang_ or lang())


def set_lang(new, save=True):
    """Switch the screen language now (no restart)."""
    global _lang
    if new not in LANGS:
        return
    _lang = new
    if save:
        settings.put("ui_lang", new)
    refresh()


def other():
    return "en" if lang() == "el" else "el"


def bind(widget, key, option="text", **kw):
    """Set widget[option] = T(key, **kw) now and again on every language switch."""
    try:
        widget.configure({option: T(key, **kw)})
    except Exception:
        return widget
    _bound.append((weakref.ref(widget), key, option, kw))
    return widget


def on_change(fn):
    _listeners.append(fn)


def refresh():
    alive = []
    for ref, key, option, kw in _bound:
        w = ref()
        if w is None:
            continue
        try:
            w.configure({option: T(key, **kw)})
            alive.append((ref, key, option, kw))
        except Exception:
            pass                            # destroyed widget
    _bound[:] = alive
    for fn in list(_listeners):
        try:
            fn()
        except Exception:
            _listeners.remove(fn)


def reload_files():
    """Read the language files again (after editing them by hand)."""
    _tables.clear()
    _rx.clear()
    refresh()
