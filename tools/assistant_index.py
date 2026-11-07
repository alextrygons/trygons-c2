"""The talking manual's index (step AM3, CLAUDE.md W, docs/assistant_design.md section 4).

    python tools/assistant_index.py build [--if-stale]   rebuild groundstation/assistant/index/manual_*.json
                                                         (then commit it; tools/check.sh only CHECKS, AM3-r)
    python tools/assistant_index.py check                exit 1 when a committed index is stale (tools/check.sh)
    python tools/assistant_index.py ask "πού κλείνω την καταπακτή;" [--lang el]   the top 3 passages
    python tools/assistant_index.py eval [--report FILE] [--gaps FILE] [--tuning] [-v]   the question list:
                                                         top-1 / top-3, can't find, the manual gaps - all, the
                                                         in-sample (tuning) part and the HELD-OUT part
                                                         (tests/assistant/heldout.json) separately

Text only: reads the manuals, the tour and the lang files; writes only the index folder (build) and the
files named on the command line (eval). No network, no process.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "groundstation"))

from assistant import index  # noqa: E402

QDIR = os.path.join(REPO, "tests", "assistant")

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def cmd_build(a):
    for lang in ("el", "en"):
        why = index.stale(lang)
        if a.if_stale and not why:
            print("assistant index %s: current" % lang)
            continue
        p = index.save(index.build(lang))
        print("assistant index %s: built %s (%s)" % (lang, os.path.relpath(p, REPO), "; ".join(why) or "forced"))
    return 0


def cmd_check(a):
    bad = 0
    for lang in ("el", "en"):
        why = index.stale(lang)
        print("assistant index %s: %s" % (lang, "STALE - " + "; ".join(why) if why else "current"))
        bad += bool(why)
    return 1 if bad else 0


def cmd_ask(a):
    s = index.Searcher(index.load(a.lang))
    r = s.search(a.question)
    print("terms: %s" % " ".join(r["terms"]))
    print("CAN'T FIND (%s)" % r["why"] if r["cant_find"] else "found")
    for n, h in enumerate(r["hits"], 1):
        p = h["passage"]
        print("\n%d. score %.2f cover %.0f %%  [%s] %s / %s  lines %s" % (
            n, h["score"], h["cover"] * 100, p["ch"], p["chapter"], p["heading"], p["lines"]))
        print("   " + p["text"][:400])
    return 0


def load_questions(lang):
    with open(os.path.join(QDIR, "questions_%s.json" % lang), encoding="utf-8") as f:
        return json.load(f)


def load_heldout(lang):
    """The frozen held-out ids (tests/assistant/heldout.json, AM3-r MED-2): never used for tuning."""
    with open(os.path.join(QDIR, "heldout.json"), encoding="utf-8") as f:
        return set(json.load(f)[lang])


# AM3-r MED-3: an expected phrase counts only when it is SPECIFIC: found in the TEXT of a passage (a heading /
# term alone is no hit - every part of a long section shares its heading) and in fewer than LOOSE_N passages
LOOSE_N = 6


def _norm(t):
    return " ".join(index.fold(t).split())


_NORMED = {}


def _texts(passages):
    """The normalised passage texts (cached per passage list)."""
    k = id(passages)
    if k not in _NORMED or _NORMED[k][0] is not passages:
        _NORMED[k] = (passages, [_norm(p["text"]) for p in passages])
    return _NORMED[k][1]


def phrase_count(passages, phrase):
    e = _norm(phrase)
    return sum(1 for t in _texts(passages) if e in t)


def specific(passages, expect):
    """The expected phrases that are specific enough to judge a hit (1 .. LOOSE_N - 1 passages hold them)."""
    return [e for e in expect if 0 < phrase_count(passages, e) < LOOSE_N]


def matches(passage, expect, tour=()):
    """A hit: the passage TEXT holds one of the (specific) expected phrases, or it is one of the tour stops named
    for it. Returns "manual" / "tour" / ""."""
    if passage.get("kind") == "tour" and passage["id"].split(":", 2)[-1] in tour:
        return "tour"
    t = _norm(passage["text"])
    return "manual" if any(_norm(e) in t for e in expect) else ""


def evaluate(lang, searcher=None, qs=None, heldout=None):
    """The question list of one language -> per-question results + totals for ALL / the TUNING part / the
    HELD-OUT part (tot["all"], tot["tuning"], tot["heldout"]; tot itself = all)."""
    searcher = searcher or index.Searcher(index.load(lang))
    qs = qs or load_questions(lang)
    held = load_heldout(lang) if heldout is None else set(heldout)
    stops = {p["id"].split(":", 2)[-1] for p in searcher.passages if p["kind"] == "tour"}
    rows = []
    for q in qs["questions"]:
        r = searcher.search(q["q"])
        exp = q.get("expect") or []
        tour = q.get("tour") or ()
        row = {"id": q["id"], "q": q["q"], "group": q.get("group", ""), "in_manual": bool(exp or tour),
               "cant_find": r["cant_find"], "why": r["why"], "gap": q.get("gap", ""), "heldout": q["id"] in held,
               "top": [(h["passage"]["id"], h["passage"]["ch"], h["score"], h["cover"]) for h in r["hits"]]}
        if row["in_manual"]:
            spec = specific(searcher.passages, exp)
            row["loose"] = [e for e in exp if e not in spec]
            kinds = [matches(h["passage"], spec, tour) for h in r["hits"]]
            ranks = [i for i, k in enumerate(kinds) if k]
            row["rank"] = (ranks[0] + 1) if ranks else 0
            row["via"] = kinds[ranks[0]] if ranks else ""
            # a usable label: a specific phrase (or no phrase, only tour stops) and every named tour stop real
            row["exists"] = bool(spec or (tour and not exp)) and all(t in stops for t in tour)
            row["top1"] = row["rank"] == 1 and not r["cant_find"]
            row["top3"] = row["rank"] >= 1 and not r["cant_find"]
        else:
            row["ok"] = r["cant_find"]
        rows.append(row)

    def totals(rs):
        inm = [r for r in rs if r["in_manual"]]
        out = [r for r in rs if not r["in_manual"]]
        return {"lang": lang, "n": len(rs), "in_manual": len(inm), "not_in_manual": len(out),
                "top1": sum(r["top1"] for r in inm), "top3": sum(r["top3"] for r in inm),
                "top1_tour": sum(r["top1"] and r["via"] == "tour" for r in inm),
                "false_cant_find": sum(r["cant_find"] for r in inm),
                "cant_find_ok": sum(r["ok"] for r in out),
                "broken": [r["id"] for r in inm if not r["exists"]],
                "gaps": [r for r in rs if r["gap"]]}
    tot = totals(rows)
    tot["all"] = dict(tot)
    tot["tuning"] = totals([r for r in rows if not r["heldout"]])
    tot["heldout"] = totals([r for r in rows if r["heldout"]])
    return rows, tot


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def line_of(name, t):
    return ("%s %s: %d questions (%d in the manual, %d not): top-1 %d/%d = %.0f %% (%d via a tour stop), "
            "top-3 %d/%d = %.0f %%, can't find wrongly %d, can't find right %d/%d"
            % (t["lang"], name, t["n"], t["in_manual"], t["not_in_manual"], t["top1"], t["in_manual"],
               pct(t["top1"], t["in_manual"]), t["top1_tour"], t["top3"], t["in_manual"],
               pct(t["top3"], t["in_manual"]), t["false_cant_find"], t["cant_find_ok"], t["not_in_manual"]))


NAMES = {"all": "all", "tuning": "in-sample (tuning)", "heldout": "HELD-OUT"}


def cmd_eval(a):
    lines, gaps, bad = [], [], 0
    parts = ("tuning",) if a.tuning else ("all", "tuning", "heldout")
    for lang in ("el", "en"):
        rows, t = evaluate(lang)
        for name in parts:
            lines.append(line_of(NAMES[name], t[name]))
        if t["broken"]:
            lines.append("  BROKEN (no specific expected phrase in any passage text, or a missing tour stop): %s"
                         % ", ".join(t["broken"]))
            bad += 1
        if a.verbose:
            for r in rows:
                if a.tuning and r["heldout"]:
                    continue                            # tuning never looks at the held-out questions
                if r["in_manual"] and r["loose"]:
                    lines.append("  loose %s: %s" % (r["id"], r["loose"]))
                if r["in_manual"] and not r["top1"]:
                    lines.append("  miss %s rank %s %s: %s  top %s" % (
                        r["id"], r["rank"], "CANT" if r["cant_find"] else "", r["q"], r["top"][:3]))
                if not r["in_manual"] and not r["ok"]:
                    lines.append("  found? %s: %s  top %s" % (r["id"], r["q"], r["top"][:1]))
        for g in t["gaps"]:
            gaps.append("- %s %s: \"%s\" - %s" % (lang, g["id"], g["q"], g["gap"]))
    print("\n".join(lines))
    print("manual gaps: %d" % len(gaps))
    if a.verbose:
        print("\n".join(gaps))
    if a.report:
        with open(a.report, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
    if a.gaps:
        with open(a.gaps, "w", encoding="utf-8") as f:
            f.write("\n".join(gaps) + "\n")
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--if-stale", action="store_true")
    sub.add_parser("check")
    k = sub.add_parser("ask")
    k.add_argument("question")
    k.add_argument("--lang", default="el", choices=("el", "en"))
    e = sub.add_parser("eval")
    e.add_argument("--report")
    e.add_argument("--gaps")
    e.add_argument("-v", "--verbose", action="store_true")
    e.add_argument("--tuning", action="store_true", help="only the tuning part (never shows the held-out questions)")
    a = ap.parse_args(argv)
    return {"build": cmd_build, "check": cmd_check, "ask": cmd_ask, "eval": cmd_eval}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
