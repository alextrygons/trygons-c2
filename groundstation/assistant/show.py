"""What the screens SHOW of an answer (step AM5): pure helpers for the slim panel of screen 2, the message line,
screen 1's notice and the [ΣΤΟ ΕΓΧΕΙΡΙΔΙΟ] place. No tkinter, no sound, no file written.

  cut(text, n)            a text cut at a word with "…" (long picks: AM4-r2 review MED-3 - never an overflowing line)
  cut_answer(...)         the character cut that keeps a joined unit's LIMIT whole (AM4-r6)
  fit_line(...)           the PIXEL rule of the two one-line places (AM4-r7): the whole line or the fallback
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


def pixel_cut(text, room, measure):
    """An END-cut by PIXELS (AM4-r7): whole words + "…" that measure() says fit in `room`, then characters."""
    t = " ".join((text or "").split())
    if not t or measure(t) <= room:
        return t
    words = t.split(" ")
    while len(words) > 1:
        words.pop()
        c = " ".join(words).rstrip(" ,;:·-–") + "…"
        if measure(c) <= room:
            return c
    c = t
    while len(c) > 1 and measure(c + "…") > room:
        c = c[:-1]
    return (c.rstrip(" ,;:·-–") + "…") if c else ""


def has_limit(text, stmts=()):
    """AM4-r7 (review of b1a0138, MED): an answer whose one-line display must never be END-cut - a JOINED unit
    (stmts) OR any answer with a limiting word in it (answer.LIMIT_RE: never / not / only / except / unless /
    exception ...: «... είναι βοηθήματα. ΔΕΝ είναι για ναυσιπλοΐα.», "... refused toward an end stop whose switch is
    open"). A plain answer may keep the ellipsis."""
    from . import answer                              # (lazy: show.py stays light for the panel helpers)
    return bool(stmts) or bool(answer.LIMIT_RE.search(answer.plain(text or "")))


def fit_line(full, room, measure, limited, fallbacks=()):
    """AM4-r7 (review of b1a0138, MED): the two ONE-LINE places (screen 1's notice, screen 2's message line) cut
    by PIXELS, but cut_answer() counted CHARACTERS - a 111-character joined answer passed it unchanged and the
    renderer's pixel cut (f.measure to the bar's free width; the HUD's font shrink to 8 px then the window-edge
    crop) still dropped the limiting sentence. Rule: the WHOLE line (`full` = prefix + answer + cite, as it is
    drawn) is shown only when measure(full) <= room; a `limited` answer (has_limit) that does not fit shows the
    first of `fallbacks` that fits (the short «has a restriction - see the panel» line, with / without its
    prefix; the last one pixel-cut: it holds no claim), never an end-cut; a plain answer keeps the pixel end-cut
    with the ellipsis. `measure` = the renderer's own pixel measure of the font it draws with."""
    t = " ".join((full or "").split())
    if not t or measure(t) <= room:
        return t
    if limited:
        fbs = [" ".join(f.split()) for f in (fallbacks or ()) if f and f.strip()]
        for fb in fbs:
            if measure(fb) <= room:
                return fb
        return pixel_cut(fbs[-1] if fbs else t, room, measure)
    return pixel_cut(t, room, measure)


MIN_ANSWER_ROOM = 60      # AM4-r8: below this many pixels beside a prefix no answer line is drawn (the prefix alone)


def fit_message(prefix, full, room, measure, limited, fallbacks=()):
    """AM4-r8 (the opus review of 37d85dd, safety MED): the message line of screen 2 can carry a PREFIX before the
    assistant's answer - the W79 «Κεντράρετε το χειριστήριο για να επιβεβαιώσετε» hint or the loudhailer's «ΜΙΛΑ:
    ...» - joined with "  ·  " (app.bar_items); AM4-r7's fit_line was applied only when the drawn text WAS the answer
    line, so with a prefix the renderer fell back to the old END-cut and the limit was lost again. Rule: the prefix
    (with its separator) is kept WHOLE at the start (as the old cut did), the answer line goes through fit_line in the
    room that is left: whole / the fallback (limited) / the pixel end-cut (plain). Less than MIN_ANSWER_ROOM beside
    the prefix = the prefix alone (pixel-cut): the answer is on the panel and on screen 1 anyway, and an unfinished
    answer line beside a hint is worse than none. No prefix = fit_line."""
    p = (prefix or "").replace("\n", " ")
    if not p.strip():
        return fit_line(full, room, measure, limited, fallbacks)
    pw = measure(p)
    if room - pw < MIN_ANSWER_ROOM:
        return pixel_cut(p.strip(), room, measure)
    return p + fit_line(full, room - pw, measure, limited, fallbacks)


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
