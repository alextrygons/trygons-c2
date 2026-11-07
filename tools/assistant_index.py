"""The talking manual's index (step AM3, CLAUDE.md W, docs/assistant_design.md section 4).

    python tools/assistant_index.py build [--if-stale]   rebuild groundstation/assistant/index/manual_*.json
    python tools/assistant_index.py check                exit 1 when a committed index is stale
    python tools/assistant_index.py ask "πού κλείνω την καταπακτή;" [--lang el]   the top 3 passages
    python tools/assistant_index.py eval [--report FILE] [--gaps FILE]   the question list: top-1 / top-3,
                                                         can't find, the manual gaps

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


def matches(passage, expect, tour=()):
    """A hit: the passage holds one of the expected manual phrases, or it is one of the tour stops named for it."""
    if passage.get("kind") == "tour" and passage["id"].split(":", 2)[-1] in tour:
        return True
    t = index.fold(" ".join((passage["text"], passage.get("heading") or "", passage.get("term") or "")))
    t = " ".join(t.split())
    return any(" ".join(index.fold(e).split()) in t for e in expect)


def evaluate(lang, searcher=None, qs=None):
    """The question list of one language -> per-question results + totals."""
    searcher = searcher or index.Searcher(index.load(lang))
    qs = qs or load_questions(lang)
    rows = []
    for q in qs["questions"]:
        r = searcher.search(q["q"])
        exp = q.get("expect") or []
        row = {"id": q["id"], "q": q["q"], "group": q.get("group", ""), "in_manual": bool(exp),
               "cant_find": r["cant_find"], "why": r["why"], "gap": q.get("gap", ""),
               "top": [(h["passage"]["id"], h["passage"]["ch"], h["score"], h["cover"]) for h in r["hits"]]}
        if exp:
            tour = q.get("tour") or ()
            ranks = [i for i, h in enumerate(r["hits"]) if matches(h["passage"], exp, tour)]
            row["rank"] = (ranks[0] + 1) if ranks else 0
            stops = {p["id"].split(":", 2)[-1] for p in searcher.passages if p["kind"] == "tour"}
            row["exists"] = any(matches(p, exp) for p in searcher.passages) and all(t in stops for t in tour)
            row["top1"] = row["rank"] == 1 and not r["cant_find"]
            row["top3"] = row["rank"] >= 1 and not r["cant_find"]
        else:
            row["ok"] = r["cant_find"]
        rows.append(row)
    inm = [r for r in rows if r["in_manual"]]
    out = [r for r in rows if not r["in_manual"]]
    tot = {"lang": lang, "n": len(rows), "in_manual": len(inm), "not_in_manual": len(out),
           "top1": sum(r["top1"] for r in inm), "top3": sum(r["top3"] for r in inm),
           "false_cant_find": sum(r["cant_find"] for r in inm),
           "cant_find_ok": sum(r["ok"] for r in out),
           "broken": [r["id"] for r in inm if not r["exists"]],
           "gaps": [r for r in rows if r["gap"]]}
    return rows, tot


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def cmd_eval(a):
    lines, gaps, bad = [], [], 0
    for lang in ("el", "en"):
        rows, t = evaluate(lang)
        lines.append("%s: %d questions (%d in the manual, %d not): top-1 %d/%d = %.0f %%, top-3 %d/%d = %.0f %%, "
                     "can't find wrongly %d, can't find right %d/%d"
                     % (lang, t["n"], t["in_manual"], t["not_in_manual"], t["top1"], t["in_manual"],
                        pct(t["top1"], t["in_manual"]), t["top3"], t["in_manual"], pct(t["top3"], t["in_manual"]),
                        t["false_cant_find"], t["cant_find_ok"], t["not_in_manual"]))
        if t["broken"]:
            lines.append("  BROKEN (expected text not in any passage): %s" % ", ".join(t["broken"]))
            bad += 1
        if a.verbose:
            for r in rows:
                if r["in_manual"] and not r["top1"]:
                    lines.append("  miss %s rank %s %s: %s  top %s" % (r["id"], r["rank"], "CANT" if r["cant_find"] else "",
                                                                     r["q"], r["top"][:3]))
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
    a = ap.parse_args(argv)
    return {"build": cmd_build, "check": cmd_check, "ask": cmd_ask, "eval": cmd_eval}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
