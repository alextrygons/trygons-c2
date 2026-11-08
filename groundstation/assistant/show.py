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


MIN_STMT = 40             # the least a shortened STATEMENT keeps on a line (below it the short "has a restriction" line)


def cut_answer(text, n, stmts=(), fallback=""):
    """AM4-r6 MED-1 (review of a8bb45f): an answer that is a JOINED unit - a manual statement + the sentence(s) that
    LIMIT it («... είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα.») - is never END-cut on a line: the plain cut() dropped
    exactly the limiting sentence (196 of 330 joined units on screen 1's notice, 121 on screen 2's message line).
    `stmts` = the statement piece(s) (answer "stmts"): the text around them (the limits) is kept WHOLE and only the
    statement(s) are shortened in the middle of the line; when less than MIN_STMT characters would remain for a
    statement, `fallback` (the short «has a restriction - see the panel» line) is returned instead of a cut that
    could lose a limit. Without `stmts` (a single sentence, a can't-find) = cut()."""
    t = " ".join((text or "").split())
    if len(t) <= n:
        return t
    sts = [" ".join(s.split()) for s in (stmts or ()) if s and " ".join(s.split()) in t]
    if not sts:
        return cut(t, n)
    # the line = fixed parts (the limits, kept whole) + the statements sharing what is left
    parts, rest = [], t
    for s in sts:                                    # (in the order they stand in the text)
        i = rest.find(s)
        parts.append(("fixed", rest[:i]))
        parts.append(("stmt", s))
        rest = rest[i + len(s):]
    parts.append(("fixed", rest))
    fixed = sum(len(p) for k, p in parts if k == "fixed")
    budget = (n - fixed) // len(sts)
    if budget < MIN_STMT:
        return fallback or cut(t, n)
    out = "".join(p if k == "fixed" else cut(p, budget) for k, p in parts)
    return " ".join(out.split())


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
