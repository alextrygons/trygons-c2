"""Quizzes (ACADEMY_SPEC D, step 18): the model - no window, no network, no process.

A quiz pool = content/quizzes/*.json (kind "quiz"). Three question types:
  "choice"  (default) multiple choice: choices [{text_el, text_en}], answer = the right index;
  "picture" a DRAWING of a screen (show "s1" = screen 1 HUD, "s2" = screen 2, "panel" = the panel
            buttons) - the trainee clicks the asked part (area = a hud.DEFAULT_LAYOUT box / LOGO
            "logo", a tour.S2_BOXES part, a tour.PANEL_BTNS button or "estop"); nothing is lit;
  "sound"   an alarm sound (sound = "warning" / "caution" / "advisory", the ground station's own
            tones) + multiple choice: which level is it?
Every question has its explanation (why_el / why_en), shown after the answer. A run takes the
questions of one or more pools in RANDOM order with the choices shuffled; a sound question is left
out when its sound file is not there (the ground station makes the tones: academymenu.py), the
window says so. Scores go to the academy logbook (kind "quiz", topic = the pool id: weak areas).
"""
import os
import random

import content
import tour                                    # first: it puts groundstation/ on sys.path
import hud                                      # noqa: E402

LANGS = ("el", "en")
TYPES = ("choice", "picture", "sound")
SHOWS = ("s1", "s2", "panel")
# the alarm-level tones of the ground station (groundstation/sound.py LEVEL_TONE: AC27 checks it)
SOUNDS = ("warning", "caution", "advisory")
TONE_DIR = "tones"                      # <the station's sound cache>/tones/<name>.wav (academymenu.py)
CLICK_MARGIN = 0.01                     # a click this close to the asked part still counts


def pools(content_dir=None):
    out = []
    for f in content.files(content_dir):
        d = content.load(f)
        if d.get("kind") == "quiz":
            out.append(d)
    return sorted(out, key=lambda d: (content.LEVELS.index(d["level"]) if d.get("level") in content.LEVELS else 9,
                                      d.get("id")))


def tone_path(name, cache=None):
    return os.path.join(cache or tour.voice_cache(), TONE_DIR, name + ".wav")


def area_rect(show, area):
    """(x, y, w, h) fractions of a part in the drawing of that screen (KeyError = unknown part)."""
    if show == "s1":
        if area == "logo":
            return tuple(hud.LOGO_BOX)
        if area not in hud.OVERLAYS:
            raise KeyError(area)
        return tuple(hud.DEFAULT_LAYOUT[area])
    if show == "s2":
        return tuple(tour.S2_BOXES[area])
    if show == "panel":
        return tour.panel_rect(area)
    raise KeyError(show)


def inside(fx, fy, rect, margin=CLICK_MARGIN):
    x, y, w, h = rect
    return x - margin <= fx <= x + w + margin and y - margin <= fy <= y + h + margin


def check_doc(doc, name="?"):
    """Quiz-specific checks (content.check does the common ones). [] = OK."""
    p = []
    if doc.get("kind") != "quiz":
        return p
    if not doc.get("id"):
        p.append("%s: no id" % name)
    qs = doc.get("questions")
    if not isinstance(qs, list) or not qs:
        return p + ["%s: no questions" % name]
    for i, q in enumerate(qs):
        w = "%s question %d" % (name, i + 1)
        t = q.get("type", "choice")
        if t not in TYPES:
            p.append("%s: unknown type %r" % (w, t))
            continue
        for k in ("q_el", "q_en", "why_el", "why_en"):
            if not isinstance(q.get(k), str) or not q[k].strip():
                p.append("%s: no %s" % (w, k))
        if t == "picture":
            try:
                area_rect(q.get("show"), q.get("area"))
            except (KeyError, TypeError):
                p.append("%s: unknown picture part %r / %r" % (w, q.get("show"), q.get("area")))
        else:
            n = len(q.get("choices", []))
            if n < 2 or not isinstance(q.get("answer"), int) or not 0 <= q["answer"] < n:
                p.append("%s: answer must index its %d choices" % (w, n))
        if t == "sound" and q.get("sound") not in SOUNDS:
            p.append("%s: sound must be one of %s" % (w, "/".join(SOUNDS)))
    return p


class Run(object):
    """One quiz: the questions of the pools in random order. current() -> the question shown;
    answer(i) for choice / sound, click(fx, fy) for picture -> the result dict; next()."""

    def __init__(self, docs, lang="el", n=None, seed=None, sound_ok=None):
        self.lang = lang if lang in LANGS else "el"
        self.rng = random.Random(seed)
        self.left_out = 0
        qs = []
        for d in docs:
            for q in d.get("questions", []):
                if q.get("type") == "sound" and sound_ok is not None and not sound_ok(q.get("sound")):
                    self.left_out += 1
                    continue
                qs.append((d["id"], q))
        self.rng.shuffle(qs)
        if n:
            qs = qs[:n]
        self.items = []
        for pool, q in qs:
            order = list(range(len(q.get("choices", []))))
            self.rng.shuffle(order)
            self.items.append({"pool": pool, "q": q, "order": order, "result": None})
        self.index = 0

    def tx(self, obj, key):
        return obj.get("%s_%s" % (key, self.lang)) or obj.get("%s_el" % key, "")

    def finished(self):
        return self.index >= len(self.items)

    def current(self):
        """The question as shown: type, text, choices (shuffled), show / area / sound, answered result."""
        if self.finished():
            return None
        it = self.items[self.index]
        q = it["q"]
        return {"type": q.get("type", "choice"), "pool": it["pool"], "text": self.tx(q, "q"),
                "choices": [self.tx(q["choices"][i], "text") for i in it["order"]],
                "show": q.get("show"), "area": q.get("area"), "sound": q.get("sound"),
                "n": self.index + 1, "of": len(self.items), "result": it["result"]}

    def _result(self, ok, right):
        it = self.items[self.index]
        it["result"] = {"correct": bool(ok), "right": right, "why": self.tx(it["q"], "why")}
        return it["result"]

    def answer(self, shown_index):
        """A choice / sound question: shown_index = the index of the choice as SHOWN (shuffled)."""
        it = self.items[self.index]
        if it["result"] is not None or it["q"].get("type") == "picture":
            return it["result"]
        right = it["order"].index(it["q"]["answer"])
        return self._result(shown_index == right, right)

    def click(self, fx, fy):
        """A picture question: the click in fractions of the drawing."""
        it = self.items[self.index]
        q = it["q"]
        if it["result"] is not None or q.get("type") != "picture":
            return it["result"]
        rect = area_rect(q["show"], q["area"])
        return self._result(inside(fx, fy, rect), rect)

    def next(self):
        if not self.finished() and self.items[self.index]["result"] is not None:
            self.index += 1
        return self.current()

    def score(self):
        done = [it for it in self.items if it["result"] is not None]
        if not self.items:
            return 0
        return int(round(100.0 * sum(1 for it in done if it["result"]["correct"]) / len(self.items)))

    def log_entries(self):
        """One logbook entry per pool (accounts.Store.log(user, "quiz", **entry)): topic = the pool id."""
        out = []
        for pool in sorted({it["pool"] for it in self.items}):
            its = [it for it in self.items if it["pool"] == pool]
            right = sum(1 for it in its if it["result"] and it["result"]["correct"])
            out.append({"id": pool, "topic": pool, "score": int(round(100.0 * right / len(its))),
                        "right": right, "n": len(its)})
        return out
