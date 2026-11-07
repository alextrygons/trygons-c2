"""What the screens SHOW of an answer (step AM5): pure helpers for the slim panel of screen 2, the message line,
screen 1's notice and the [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] place. No tkinter, no sound, no file written.

  cut(text, n)            a text cut at a word with "…" (long picks: AM4-r2 review MED-3 - never an overflowing line)
  manual_place(info)      (lang, first line, last line) of the answer's passage in MANUAL_EL.txt / MANUAL.txt, or None
  demo_answer(lang)       a canned answer from the committed index (the tour picture / screenshots: no model runs)
"""
import os
import re

from . import index, paths

LINE_MAX = 160            # screen 2's message line
NOTICE_MAX = 120          # screen 1's notice line
PANEL_ANSWER_MAX = 320    # the answer on the panel
PANEL_PASSAGE_MAX = 700   # the passage on the panel (a passage is <= 120 words ~ 800 chars: nearly always whole)
PANEL_Q_MAX = 140
MANUALS = {"el": "MANUAL_EL.txt", "en": "MANUAL.txt"}


def cut(text, n):
    """At most n characters, cut at a word boundary with an ellipsis."""
    t = " ".join((text or "").split())
    if len(t) <= n:
        return t
    c = t[:n - 1]
    if " " in c[n // 2:]:
        c = c[:c.rfind(" ")]
    return c.rstrip(" ,;:·-–") + "…"


def manual_file(lang):
    return os.path.join(paths.REPO, MANUALS.get(lang, MANUALS["en"]))


def manual_place(info):
    """The lines of the answer's passage: (lang, a, b) 1-based inclusive, from the passage id matched against the
    search hits (index.py gives every manual passage its line range; a tour stop has none -> None)."""
    if not info:
        return None
    p = info.get("passage") or {}
    pid = p.get("id") or info.get("id") or ""
    hits = ((info.get("found") or {}).get("hits")) or []
    hit = next((h for h in hits if h.get("id") == pid), None) or (hits[0] if hits and not pid else None)
    if hit is None or hit.get("ch") == "tour":
        return None
    ln = hit.get("lines") or [0, 0]
    if not ln or not ln[0]:
        return None
    lang = (pid.split(":")[0] if ":" in pid else None) or info.get("lang") or "el"
    return lang if lang in MANUALS else "en", int(ln[0]), int(max(ln[0], ln[1] or ln[0]))


def manual_lines(lang):
    """The manual's lines (for the read-only window); [] when the file is missing."""
    try:
        with open(manual_file(lang), encoding="utf-8") as f:
            return f.read().splitlines()
    except OSError:
        return []


def demo_answer(lang="el", q=None):
    """A canned answer made from the committed index without any model: the first whole sentence of the best
    passage for a typical question (the tour picture, docs/screens, tests). {} when the index is missing."""
    from . import answer
    q = q or ("πού κλείνω την καταπακτή;" if lang == "el" else "how do I close the hatch?")
    try:
        s = index.Searcher(index.load(lang))
    except (OSError, ValueError, KeyError):
        return {}
    res = s.search(q)
    found = index.hit_summary(res)
    hits = found.get("hits") or []
    if not hits:
        return {}
    h = next((x for x in hits if x.get("ch") != "tour"), hits[0])    # a MANUAL passage (it has a line range)
    sents = [x for x in answer.sentences(h.get("text") or "") if len(x.split()) >= 4] or [h.get("text") or ""]
    a = sents[0]
    return {"op": "answer", "q": q, "lang": lang, "a": a, "src": "extract", "cite": answer.cite(h, lang),
            "id": h.get("id", ""), "passage": answer.passage_of(h, lang), "found": found, "ms": 820,
            "answer_ms": 2100, "speak": True, "mode": "gpu"}


def status_key(voice_why):
    """The panel's voice line: the lang key for how the last answer was (not) spoken."""
    return "asst.v_" + (voice_why or "none")


_WS = re.compile(r"\s+")


def one_line(text):
    return _WS.sub(" ", text or "").strip()
