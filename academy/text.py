"""Academy screen texts: lang/el.txt (Greek, the default) and lang/en.txt.

Same file format as groundstation/lang (one "key = text" per line, # comments, {name} parts,
\\n = new line). The language: TRYGONS_LANG (tests, screenshots), else Greek. When the academy
is started from the ground station (step 16+) it follows the ground station's language.
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
LANG_DIR = os.path.join(HERE, "lang")
LANGS = ("el", "en")
_LINE = re.compile(r"^([A-Za-z0-9_.\-]+)\s*=\s?(.*)$")
_tables = {}
_lang = None
MISSING = set()


def parse(text):
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
        with open(os.path.join(LANG_DIR, lang + ".txt"), encoding="utf-8") as f:
            _tables[lang] = parse(f.read())
    return _tables[lang]


def lang():
    global _lang
    if _lang is None:
        env = os.environ.get("TRYGONS_LANG")
        _lang = env if env in LANGS else "el"
    return _lang


def set_lang(code):
    global _lang
    if code in LANGS:
        _lang = code


def T(key, lang_code=None, **kw):
    lc = lang_code or lang()
    s = load(lc).get(key)
    if s is None:
        MISSING.add((lc, key))
        s = load("en").get(key, "[%s]" % key)
    return s.format(**kw) if kw else s
