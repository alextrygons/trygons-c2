"""The talking manual's index (step AM3, CLAUDE.md W, docs/assistant_design.md section 4).

    python tools/assistant_index.py build [--if-stale]   rebuild groundstation/assistant/index/manual_*.json
                                                         (then commit it; tools/check.sh only CHECKS, AM3-r)
    python tools/assistant_index.py check                exit 1 when a committed index is stale (tools/check.sh)
    python tools/assistant_index.py ask "πού κλείνω την καταπακτή;" [--lang el]   the top 3 passages
    python tools/assistant_index.py eval [--report FILE] [--gaps FILE] [--tuning] [-v]   the question list:
                                                         top-1 / top-3, can't find, the manual gaps - all, the
                                                         in-sample (tuning) part and the HELD-OUT part
                                                         (tests/assistant/heldout.json) separately
    python tools/assistant_index.py floor [--write]      AM4-r3 LOW-b: the extract SCORE FLOOR (answer.EXTRACT_FLOOR,
                                                         W112) still separates what it was tuned on - the known
                                                         off-manual TUNING questions score under it, the known
                                                         in-manual ones over it, the code's floor = the tuned one
                                                         (tests/assistant/floor.json); exit 1 = the BM25 scores
                                                         moved (a manual / parser change): re-tune the floor on the
                                                         TUNING part (tools/assistant_llm_eval.py, docs 5e), then
                                                         `floor --write` freezes the new separation (tools/check.sh)

Text only: reads the manuals, the tour and the index parser (index.sources(); the lang files are NOT read,
AM3-r LOW-1); `check` also compares the committed passages with a fresh build (AM3-r2 LOW); writes only the index folder (build) and the
files named on the command line (eval), `floor --write` tests/assistant/floor.json. No network, no process.
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


def load_questions(lang, qset="main"):
    """The question list of a language: "main" = tests/assistant/questions_<lang>.json (the AM3 list: tuning +
    held-out parts); another name = tests/assistant/<name>_<lang>.json - "fresh" = the AM6 set written AFTER every
    tuning choice, measured ONCE and never tuned on (every id in it is its own part)."""
    fn = "questions_%s.json" % lang if qset == "main" else "%s_%s.json" % (qset, lang)
    with open(os.path.join(QDIR, fn), encoding="utf-8") as f:
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


def manual_only(passages):
    """The MANUAL passages (AM3-r2 MED-1: phrases are counted on these only - never on a tour text)."""
    k = ("manual", id(passages))
    if k not in _NORMED or _NORMED[k][0] is not passages:
        _NORMED[k] = (passages, [p for p in passages if p.get("kind") != "tour"])
    return _NORMED[k][1]


def phrase_count(passages, phrase):
    """In how many MANUAL passage texts the phrase is (tour stops never count, AM3-r2 MED-1)."""
    e = _norm(phrase)
    return sum(1 for t in _texts(manual_only(passages)) if e in t)


def specific(passages, expect):
    """The expected phrases that are specific enough to judge a hit (1 .. LOOSE_N - 1 MANUAL passages hold them;
    a phrase that is only in a tour text is not specific: that question is a tour-only one, AM3-r2 MED-1)."""
    return [e for e in expect if 0 < phrase_count(passages, e) < LOOSE_N]


def matches(passage, expect, tour=()):
    """A hit: the passage TEXT holds one of the (specific) expected phrases, or it is one of the tour stops named
    for it. Returns "manual" / "tour" / "". A tour passage is ALWAYS "tour" (AM3-r2 MED-1: a phrase found in a
    tour text is a tour hit, reported apart from the manual hits)."""
    is_tour = passage.get("kind") == "tour"
    if is_tour and passage["id"].split(":", 2)[-1] in tour:
        return "tour"
    t = _norm(passage["text"])
    if not any(_norm(e) in t for e in expect):
        return ""
    return "tour" if is_tour else "manual"


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
    return ("%s %s: %d questions (%d in the manual, %d not): top-1 %d/%d = %.0f %% (%d from the manual, %d via a "
            "tour stop - reported apart), top-3 %d/%d = %.0f %%, can't find wrongly %d, can't find right %d/%d"
            % (t["lang"], name, t["n"], t["in_manual"], t["not_in_manual"], t["top1"], t["in_manual"],
               pct(t["top1"], t["in_manual"]), t["top1"] - t["top1_tour"], t["top1_tour"], t["top3"], t["in_manual"],
               pct(t["top3"], t["in_manual"]), t["false_cant_find"], t["cant_find_ok"], t["not_in_manual"]))


NAMES = {"all": "all", "tuning": "in-sample (tuning)", "heldout": "HELD-OUT"}


def cmd_eval(a):
    lines, gaps, bad = [], [], 0
    qset = getattr(a, "set", "main") or "main"
    parts = ("tuning",) if a.tuning else ("all", "tuning", "heldout")
    if qset != "main":                      # AM6: a separate set (fresh) is ONE part - no tuning / held-out split
        parts = ("all",)
    for lang in ("el", "en"):
        if qset == "main":
            rows, t = evaluate(lang)
        else:
            rows, t = evaluate(lang, qs=load_questions(lang, qset), heldout=())
        for name in parts:
            lines.append(line_of(NAMES[name] if qset == "main" else qset.upper(), t[name]))
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


# --- the extract score floor gate (AM4-r3 LOW-b) ------------------------------------------------------------------
FLOOR_FILE = os.path.join(QDIR, "floor.json")
FLOOR_MARGIN = 0.5          # an in-manual question is frozen as "above" only when it clears the floor by this much
FLOOR_DRIFT = 2             # the number of in-manual tuning questions over the floor may fall by at most this


def floor_snapshot(floor, margin=FLOOR_MARGIN):
    """What the floor separates TODAY on the TUNING part (never the held-out one): per language the off-manual
    questions under it, the in-manual ones at least `margin` over it, and how many in-manual ones are over it."""
    snap = {"_help": "AM4-r3 LOW-b (W112 / W117): the extract score floor's frozen separation on the TUNING part. "
                     "`python tools/assistant_index.py floor` fails when the BM25 scores moved so that a 'below' "
                     "question reaches the floor, an 'above' one falls under it, fewer than above_n - %d in-manual "
                     "tuning questions clear it, or the code's floor (answer.EXTRACT_FLOOR = client."
                     "DEFAULT_EXTRACT_FLOOR) differs from 'floor'. Then re-tune the floor on the tuning part "
                     "(tools/assistant_llm_eval.py run --answer extract, docs/assistant_design.md 5e) and run "
                     "`floor --write`. Never tune on the held-out questions." % FLOOR_DRIFT,
            "floor": floor, "margin": margin,
            # AM7: the floor is in the units of these reference corpora (index.SCALE_REF_N / SCALE_DF)
            "scale_ref_n": dict(index.SCALE_REF_N), "scale_df": index.SCALE_DF}
    for lang in ("el", "en"):
        s = index.Searcher(index.load(lang))
        eff = floor * s.scale
        rows, _t = evaluate(lang, searcher=s)
        tun = [r for r in rows if not r["heldout"]]
        best = {r["id"]: (r["top"][0][2] if r["top"] else 0.0) for r in tun}
        snap[lang] = {"below": [r["id"] for r in tun if not r["in_manual"] and best[r["id"]] < eff],
                      "above": [r["id"] for r in tun if r["in_manual"] and best[r["id"]] >= eff + margin],
                      "above_n": sum(1 for r in tun if r["in_manual"] and best[r["id"]] >= eff), "n": s.n}
    return snap


def floor_check(frozen=None, code_floor=None):
    """(ok, lines): the frozen separation (tests/assistant/floor.json) still holds with the current index + code."""
    from assistant import answer, client
    if frozen is None:
        with open(FLOOR_FILE, encoding="utf-8") as f:
            frozen = json.load(f)
    floor = float(frozen["floor"])
    lines, bad = [], []
    code = answer.EXTRACT_FLOOR if code_floor is None else code_floor
    if abs(code - floor) > 1e-9 or abs(client.DEFAULT_EXTRACT_FLOOR - code) > 1e-9:
        bad.append("the code's floor (answer.EXTRACT_FLOOR %s, client.DEFAULT_EXTRACT_FLOOR %s) != the tuned one %s "
                   "in tests/assistant/floor.json" % (code, client.DEFAULT_EXTRACT_FLOOR, floor))
    # AM7: the reference corpora the floor is expressed in - changing them silently would re-tune the floor
    if frozen.get("scale_ref_n") != dict(index.SCALE_REF_N) or frozen.get("scale_df") != index.SCALE_DF:
        bad.append("the corpus-scale reference (index.SCALE_REF_N %s, SCALE_DF %s) != tests/assistant/floor.json (%s, "
                   "%s): a reference change is a RE-TUNE of the floor" % (index.SCALE_REF_N, index.SCALE_DF,
                                                                         frozen.get("scale_ref_n"),
                                                                         frozen.get("scale_df")))
    for lang in ("el", "en"):
        fr = frozen.get(lang) or {}
        s = index.Searcher(index.load(lang))
        eff = floor * s.scale                          # AM7: the floor in today's corpus units
        rows, _t = evaluate(lang, searcher=s)
        tun = {r["id"]: r for r in rows if not r["heldout"]}
        best = {i: (r["top"][0][2] if r["top"] else 0.0) for i, r in tun.items()}
        for i in fr.get("below", ()):
            if i in best and best[i] >= eff:
                bad.append("%s %s (off-manual) now scores %.2f >= the scaled floor %.2f (= %.2f x corpus scale %.4f): "
                           "it rose FASTER than the corpus scale" % (lang, i, best[i], eff, floor, s.scale))
        for i in fr.get("above", ()):
            if i in best and best[i] < eff:
                bad.append("%s %s (in-manual) now scores %.2f < the scaled floor %.2f" % (lang, i, best[i], eff))
        n = sum(1 for i, r in tun.items() if r["in_manual"] and best[i] >= eff)
        if n < int(fr.get("above_n", 0)) - FLOOR_DRIFT:
            bad.append("%s: only %d in-manual tuning questions clear the floor (frozen %d)" % (lang, n, fr["above_n"]))
        lines.append("extract floor %.2f x corpus scale %.4f (N %d, reference %d) = %.2f %s: %d off-manual tuning "
                     "questions under it, %d in-manual over it (frozen %d / %d)" % (
                         floor, s.scale, s.n, index.SCALE_REF_N.get(lang, 0), eff, lang,
                         sum(1 for i, r in tun.items() if not r["in_manual"] and best[i] < eff), n,
                         len(fr.get("below", ())), fr.get("above_n", 0)))
    if bad:
        lines += ["FLOOR CHECK FAILED - the search scores moved MORE than the corpus scale explains (a manual / parser "
                  "change that brought an off-manual question closer to the manual text, or a new reference): the "
                  "floor is NOT lowered and NOT raised by itself - re-tune it on the TUNING part (W112 / W117: tools/"
                  "assistant_llm_eval.py run --answer extract --floor F, docs/assistant_design.md 5e / 5n), set it in "
                  "answer.EXTRACT_FLOOR + client.DEFAULT_EXTRACT_FLOOR (index.SCALE_REF_N = the new reference), then "
                  "python tools/assistant_index.py floor --write"] + ["  " + b for b in bad]
    return not bad, lines


def floor_blockers(floor):
    """AM4-r4 LOW-1: the off-manual TUNING questions that score >= the floor TODAY, per language - a floor that does
    not separate them must not be frozen by `floor --write` (it would be frozen as "fine" with them silently left
    out of 'below')."""
    out = {}
    for lang in ("el", "en"):
        s = index.Searcher(index.load(lang))
        rows, _t = evaluate(lang, searcher=s)
        out[lang] = [(r["id"], r["top"][0][2]) for r in rows if not r["heldout"] and not r["in_manual"] and r["top"]
                     and r["top"][0][2] >= floor * s.scale]
    return out


def cmd_floor(a):
    if a.write:
        from assistant import answer
        blockers = floor_blockers(answer.EXTRACT_FLOOR)
        over = ["%s %s scores %.2f >= the floor %.2f" % (lg, i, s, answer.EXTRACT_FLOOR)
                for lg in ("el", "en") for i, s in blockers[lg]]
        if over and not a.force:
            print("FLOOR --write REFUSED (AM4-r4 LOW-1): off-manual TUNING questions reach the floor %.2f - raise the "
                  "floor (answer.EXTRACT_FLOOR + client.DEFAULT_EXTRACT_FLOOR) or, knowingly, --force (they are then "
                  "listed in floor.json 'off_manual_over'):" % answer.EXTRACT_FLOOR)
            print("\n".join("  " + x for x in over))
            return 1
        snap = floor_snapshot(answer.EXTRACT_FLOOR)
        snap["off_manual_over"] = over                 # [] normally; --force leaves the accepted ones on record
        with open(FLOOR_FILE, "w", encoding="utf-8", newline="\n") as f:
            json.dump(snap, f, ensure_ascii=False, indent=1)
            f.write("\n")
        print("wrote %s: floor %s, el %d below / %d above (%d over), en %d / %d (%d)" % (
            os.path.relpath(FLOOR_FILE, REPO), snap["floor"], len(snap["el"]["below"]), len(snap["el"]["above"]),
            snap["el"]["above_n"], len(snap["en"]["below"]), len(snap["en"]["above"]), snap["en"]["above_n"]))
    ok, lines = floor_check()
    print("\n".join(lines))
    return 0 if ok else 1


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
    e.add_argument("--set", default="main", help="the question set: main (questions_<lang>.json, the gate) or the "
                   "name of tests/assistant/<name>_<lang>.json - 'fresh' = the AM6 set, measured ONCE, never tuned on")
    fl = sub.add_parser("floor")
    fl.add_argument("--write", action="store_true", help="freeze the current separation (after a re-tune); REFUSED "
                    "while an off-manual tuning question scores >= the floor (AM4-r4 LOW-1)")
    fl.add_argument("--force", action="store_true", help="--write anyway, the off-manual questions over the floor "
                    "listed in floor.json 'off_manual_over'")
    a = ap.parse_args(argv)
    return {"build": cmd_build, "check": cmd_check, "ask": cmd_ask, "eval": cmd_eval, "floor": cmd_floor}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
