"""Academy content files (CLAUDE.md section S: "commented content files under academy/content/").

The format = JSON with "_help" / "_note..." keys as comments (the same convention as boats/*.json):
every file has "kind" and "_help"; every text a trainee sees is given in BOTH languages as
"<name>_el" + "<name>_en" (Greek first). Screen chrome (buttons, titles) stays in lang/*.txt.

  content/lessons/*.json    kind "lesson"   simulator lessons (spec C, step 18)
  content/quizzes/*.json    kind "quiz"     question pools (spec D, step 18)
  content/scenarios/*.json  kind "scenario" emergency scenarios (spec E, step 19)
  content/exams/*.json      kind "exam"     exams per level (spec G, step 21)
  content/limits.json       kind "limits"   validity, reminder, currency, attempts (spec J)
  content/test_area.json    kind "test_area" practical test area (spec J) - PLACEHOLDER

check(doc) returns a list of problems (empty = OK); test_academy.py AC10 checks every file.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CONTENT_DIR = os.path.join(HERE, "content")
KINDS = {"lesson": "lessons", "quiz": "quizzes", "scenario": "scenarios", "exam": "exams",
         "limits": None, "test_area": None}
LEVELS = ("OBSERVER", "DRIVER", "CAPTAIN")


def files(content_dir=None):
    d = content_dir or CONTENT_DIR
    out = []
    for root, _ds, fs in os.walk(d):
        for f in sorted(fs):
            if f.endswith(".json"):
                out.append(os.path.join(root, f))
    return sorted(out)


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _texts(obj, where, problems):
    """Every "<x>_el" needs its "<x>_en" and the other way round, both non-empty strings."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k.endswith("_el") or k.endswith("_en"):
                other = k[:-2] + ("en" if k.endswith("_el") else "el")
                if other not in obj:
                    problems.append("%s: %s without %s" % (where, k, other))
                if not isinstance(v, str) or not v.strip():
                    problems.append("%s: %s is empty" % (where, k))
            elif not k.startswith("_"):
                _texts(v, "%s.%s" % (where, k), problems)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            _texts(v, "%s[%d]" % (where, i), problems)


def check(doc, name="?"):
    p = []
    if not isinstance(doc, dict):
        return ["%s: not a JSON object" % name]
    kind = doc.get("kind")
    if kind not in KINDS:
        p.append("%s: unknown kind %r" % (name, kind))
    if not isinstance(doc.get("_help"), str) or not doc["_help"].strip():
        p.append("%s: no _help comment" % name)
    if "level" in doc and doc["level"] not in LEVELS:
        p.append("%s: unknown level %r" % (name, doc["level"]))
    _texts(doc, name, p)
    if kind == "quiz":
        for i, q in enumerate(doc.get("questions", [])):
            n = len(q.get("choices", []))
            if n < 2 or not isinstance(q.get("answer"), int) or not 0 <= q["answer"] < n:
                p.append("%s: question %d: answer must index its %d choices" % (name, i, n))
    if kind in ("lesson", "scenario") and doc.get("runs_on") != "sim":
        p.append("%s: runs_on must be \"sim\" (the academy never drives a real boat)" % name)
    if kind == "test_area":
        if doc.get("not_for_navigation") is not True:
            p.append("%s: not_for_navigation must be true" % name)
        if doc.get("placeholder") is not True and not doc.get("set_by"):
            p.append("%s: a real area needs set_by (instructor tool, step 21)" % name)
    return p
