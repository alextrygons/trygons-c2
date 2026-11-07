"""The talking manual's model comparison (step AM4, docs/assistant_design.md 5b): every installed language model
(tools/get_llm.py) answers the question list through the REAL worker process, measured.

    python tools/assistant_llm_eval.py run  [--models a,b] [--part tuning|heldout|all] [--lang el,en] [--limit N]
    python tools/assistant_llm_eval.py e2e  [--model NAME] [--n 10]     release -> answer text with the real Whisper +
                                                                         model (spoken questions: the offline voice)
    python tools/assistant_llm_eval.py report                            the table of the last runs
    python tools/assistant_llm_eval.py blind [--n 20]                    the owner's blind Greek review sheet

The worker is started exactly as the station does (the assistant Python, groundstation/assistant/worker.py, the
real Whisper loaded first, the network blocked by the HF offline env); "run" sends typed questions ("ask_text"),
"e2e" speaks them into the simulated microphone. "none" = no model (retrieval only) = the baseline.

Measured per question: correct, the model's own answer kept (post-check passed), the post-check's rejections,
can't find right / false, Greek quality proxy (Greek script, a complete sentence, <= 40 words), the time from the
question to the answer text, the generation time + tokens; per model: load time, where it runs (gpu / gpu+cpu /
cpu), the GPU memory it takes (nvidia-smi memory.used before / after its load). Results: test_results/
assistant_llm/ (gitignored). The held-out part (tests/assistant/heldout.json) is NEVER used for a choice: choose
on --part tuning, then measure --part heldout ONCE.
Correct = the final text (whatever is shown / spoken) holds the question's must-mention term (folded) and is not
"can't find" (no must term: it comes from a passage that matches the expected place); a not-in-the-manual
question is correct only as "can't find".
"""
import argparse
import json
import os
import re
import statistics
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GS = os.path.join(REPO, "groundstation")
sys.path.insert(0, GS)
sys.path.insert(0, HERE)

from assistant import answer, index, paths  # noqa: E402
import assistant_index as ai  # noqa: E402

OUT = os.path.join(REPO, "test_results", "assistant_llm")
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def gpu_used_mb():
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=20)
        u, t = (int(x) for x in r.stdout.strip().splitlines()[0].split(","))
        return u, t
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def installed_models():
    out = []
    root = paths.llm_root()
    for n in sorted(os.listdir(root)) if os.path.isdir(root) else []:
        if n not in ("py", "dl") and paths.llm_model(n):
            out.append(n)
    return out


class Worker:
    """The real worker process, spoken to like client.py does (JSON lines)."""

    def __init__(self, llm, stt_kind="faster", mic="null"):
        py = paths.assistant_python()
        env = dict(os.environ, HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_HUB_DISABLE_TELEMETRY="1",
                   PYTHONIOENCODING="utf-8", TRYGONS_TEST="1")
        self.p = subprocess.Popen([py, os.path.join(GS, "assistant", "worker.py")], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=open(os.path.join(OUT, "worker_err.txt"), "a"),
                                  env=env, cwd=GS, creationflags=0x00004000 if os.name == "nt" else 0)
        self.lines = []
        self.cv = threading.Condition()
        threading.Thread(target=self._read, daemon=True).start()
        gpu_m, cpu_m = paths.whisper_model(paths.MODEL_GPU), paths.whisper_model(paths.MODEL_CPU)
        self.send({"op": "init", "stt": stt_kind, "model": gpu_m or cpu_m, "cpu_model": cpu_m, "device": "auto",
                   "mic_backend": mic, "mic": None, "lang": "el", "hints": {}, "llm": llm})
        self.ready = self.wait("ready", 300)

    def _read(self):
        for line in self.p.stdout:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            with self.cv:
                self.lines.append((time.monotonic(), m))
                self.cv.notify_all()

    def send(self, d):
        self.p.stdin.write((json.dumps(d) + "\n").encode("ascii"))
        self.p.stdin.flush()

    def wait(self, op, timeout):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                for i, (t, m) in enumerate(self.lines):
                    if m.get("op") in (op, "error", "refused"):
                        del self.lines[:i + 1]
                        return (t, m)
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self.cv.wait(left)

    def ask(self, q, lang):
        t0 = time.monotonic()
        self.send({"op": "ask_text", "q": q, "lang": lang})
        r = self.wait("answer", 120)
        if r is None:
            return None, None
        return r[1], int((r[0] - t0) * 1000)

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(20)
        except Exception:
            self.p.kill()


def llm_cfg(name, answer_mode="generate"):
    """name "none" = no model; "<model>+extract" = that model in answer mode "extract" (AM4-r)."""
    if name == "none":
        return {"kind": "none"}
    if "+" in name:
        name, answer_mode = name.split("+", 1)
    return {"kind": "llama", "model": paths.llm_model(name), "py": paths.llm_py_dir(), "device": "auto",
            "measure": True, "answer_mode": answer_mode}


GREEK = re.compile(r"[α-ωά-ώϊϋΐΰ]", re.I)
LATIN_WORD = re.compile(r"[A-Za-z][A-Za-z0-9-]*")


def quality(a, passages_text, lang):
    """The automatic Greek / English quality proxy of one model answer: the right script (a Latin word in a Greek
    answer must be a name from the passages), a complete sentence, short enough to speak (<= 40 words)."""
    words = a.split()
    plain_src = answer.plain(passages_text)
    if lang == "el":
        foreign = [w for w in LATIN_WORD.findall(a) if answer.plain(w) not in plain_src]
        script = not foreign and len(GREEK.findall(a)) >= 0.5 * sum(c.isalpha() for c in a)
    else:
        script = not GREEK.findall(a) or all(answer.plain(w) in plain_src for w in a.split() if GREEK.search(w))
    complete = a.rstrip().endswith((".", "!", ";", "·", "?"))
    return {"script": bool(script), "complete": complete, "short": len(words) <= 40,
            "ok": bool(script) and complete and len(words) <= 40}


def judge(q, r, searcher):
    """correct / model_kept / ... for one question (module docstring)."""
    must = answer.plain(q.get("must") or "")
    in_manual = bool(q.get("expect") or q.get("tour"))
    a = r.get("a", "")
    src = r.get("src")
    if not in_manual:
        return {"in_manual": False, "correct": src in ("cant_find", "passage"), "model_kept": False,
                "answered": src not in ("cant_find", "passage")}
    ok = src not in ("cant_find", "passage")        # AM4-r: a failed post-check SAYS "can't find" = not an answer
    if ok and must:
        ok = must in answer.plain(a)
    elif ok:
        spec = ai.specific(searcher.passages, q.get("expect") or [])
        p = next((x for x in searcher.passages if x["id"] == r.get("id")), None)
        ok = bool(p) and bool(ai.matches(p, spec, q.get("tour") or ()))
    return {"in_manual": True, "correct": ok, "model_kept": ok and src in ("llm", "extract"),
            "answered": src not in ("cant_find", "passage")}


def cmd_run(a):
    os.makedirs(OUT, exist_ok=True)
    models = a.models.split(",") if a.models else ["none"] + installed_models()
    if a.answer == "extract":
        models = [m if m == "none" or "+" in m else m + "+extract" for m in models]
    langs = a.lang.split(",")
    held = {lang: ai.load_heldout(lang) for lang in langs}
    for name in models:
        before = gpu_used_mb()
        t0 = time.monotonic()
        w = Worker(llm_cfg(name))
        if w.ready is None or w.ready[1].get("op") != "ready":
            print("%s: the worker did not get ready: %s" % (name, w.ready))
            w.close()
            continue
        if w.ready[1].get("index_err"):           # a stale index = every answer "can't find": never measured
            print("%s: the manual index is not usable (%s) - run tools/assistant_index.py build" % (
                name, w.ready[1]["index_err"]))
            w.close()
            return 2
        load_s = time.monotonic() - t0
        after = gpu_used_mb()
        info = w.ready[1].get("llm") or {}
        print("== %s: %s, %s layers, load %.1f s, GPU used %s -> %s MB (%s)" % (
            name, info.get("mode"), info.get("layers"), load_s, before and before[0], after and after[0],
            info.get("why")), flush=True)
        for lang in langs:
            searcher = index.Searcher(index.load(lang))
            qs = ai.load_questions(lang)["questions"]
            if a.part == "tuning":
                qs = [q for q in qs if q["id"] not in held[lang]]
            elif a.part == "heldout":
                qs = [q for q in qs if q["id"] in held[lang]]
            if a.limit:
                qs = qs[:a.limit]
            path = os.path.join(OUT, "%s_%s_%s.jsonl" % (name, lang, a.part))
            with open(path, "w", encoding="utf-8") as f:
                f.write(json.dumps({"model": name, "lang": lang, "part": a.part, "llm": info, "load_s": load_s,
                                    "gpu_before": before, "gpu_after": after, "worker_device": w.ready[1].get("device"),
                                    "t": time.strftime("%Y-%m-%d %H:%M")}, ensure_ascii=False) + "\n")
                for i, q in enumerate(qs):
                    r, ms = w.ask(q["q"], lang)
                    if r is None:
                        print("  %s: no answer" % q["id"])
                        continue
                    hits = (r.get("found") or {}).get("hits") or []
                    j = judge(q, r, searcher)
                    row = {"id": q["id"], "q": q["q"], "group": q.get("group"), "must": q.get("must", ""),
                           "a": r.get("a"), "src": r.get("src"), "model_text": r.get("model_text", ""),
                           "problems": r.get("problems", []), "cite": r.get("cite"), "pid": r.get("id"),
                           "ms": ms, "gen_ms": r.get("gen_ms"), "first_ms": r.get("first_ms"),
                           "tokens": r.get("tokens"), "mode": r.get("mode"), "answer_mode": r.get("answer_mode"),
                           "passage": r.get("passage"),
                           "heldout": q["id"] in held[lang], "raw": r.get("raw", ""), "cut": r.get("cut", False),
                           "hits": [{k: h.get(k) for k in ("id", "ch", "chapter", "heading", "term", "text")}
                                    for h in hits[:answer.N_PASSAGES]], "search_cant": (r.get("found") or {}).get(
                               "cant_find", False), **j}
                    if r.get("src") in ("llm", "extract"):
                        row["quality"] = quality(r["a"], " ".join(h.get("text", "") for h in hits[:3]), lang)
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
                    f.flush()
                    if a.verbose:
                        print("  %s %-9s %5d ms %s | %s" % (q["id"], r.get("src"), ms,
                                                           "OK " if j["correct"] else "-- ", r.get("a", "")[:110]))
            print("  %s %s: %s" % (lang, a.part, summary_line(load_rows(path))), flush=True)
        w.close()
        time.sleep(2)
    return 0


def load_rows(path):
    with open(path, encoding="utf-8") as f:
        rows = [json.loads(x) for x in f if x.strip()]
    return rows[0], rows[1:]


def pct(a, b):
    return "%d/%d = %.0f %%" % (a, b, 100.0 * a / b) if b else "-"


def summarise(head, rows):
    inm = [r for r in rows if r["in_manual"]]
    out = [r for r in rows if not r["in_manual"]]
    gen = [r for r in rows if r["src"] in ("llm", "extract", "passage")]
    ms = sorted(r["ms"] for r in rows if r["src"] != "cant_find" or r.get("gen_ms"))
    # AM4-r MED-4: the time over the answers the model ACTUALLY GENERATED (its own sentence / pick kept or
    # rejected) - a "can't find" ends after a few tokens and pulls the median down
    ms_gen = sorted(r["ms"] for r in gen)
    q = [r["quality"]["ok"] for r in rows if r.get("quality")]
    toks = [(r["tokens"], r["gen_ms"]) for r in rows if r.get("gen_ms")]
    tps = (sum(t for t, _g in toks) / (sum(g for _t, g in toks) / 1000.0)) if toks and sum(g for _t, g in toks) else 0
    answered = [r for r in inm if r.get("answered", r["src"] not in ("cant_find", "passage"))]
    return {"n": len(rows), "correct": sum(r["correct"] for r in inm), "in_manual": len(inm),
            "model_kept": sum(r["model_kept"] for r in inm),
            "answered": len(answered), "precision": sum(r["correct"] for r in answered),
            "rejected": sum(1 for r in gen if r["src"] == "passage"), "generated": len(gen),
            "cant_right": sum(r["correct"] for r in out), "not_in_manual": len(out),
            "false_cant": sum(1 for r in inm if r["src"] in ("cant_find", "passage")),
            "false_cant_model": sum(1 for r in inm if r["src"] == "cant_find"),
            "quality": sum(q), "quality_n": len(q),
            "ms_med": int(statistics.median(ms)) if ms else 0,
            "ms_p90": ms[min(len(ms) - 1, int(0.9 * len(ms)))] if ms else 0,
            "ms_gen_med": int(statistics.median(ms_gen)) if ms_gen else 0, "tok_s": round(tps, 1),
            "mode": (head.get("llm") or {}).get("mode"), "layers": (head.get("llm") or {}).get("layers"),
            "gpu_mb": (head["gpu_after"][0] - head["gpu_before"][0]) if head.get("gpu_after") and
            head.get("gpu_before") and head["gpu_before"][0] > 0 else None,
            "free_mb": (head.get("llm") or {}).get("free_mb"), "load_s": round(head.get("load_s", 0), 1)}


def summary_line(hr):
    s = summarise(*hr)
    return ("correct %s (model's own answer %s), answered %s, precision %s, post-check rejected %s, can't find "
            "right %s, false can't find %d (the model's own %d), quality %s, %d ms median / %d ms p90 (generated "
            "answers only: %d ms median), %.1f tok/s, %s, GPU +%s MB" % (
                pct(s["correct"], s["in_manual"]), pct(s["model_kept"], s["in_manual"]),
                pct(s["answered"], s["in_manual"]), pct(s["precision"], s["answered"]),
                pct(s["rejected"], s["generated"]), pct(s["cant_right"], s["not_in_manual"]), s["false_cant"],
                s["false_cant_model"], pct(s["quality"], s["quality_n"]), s["ms_med"], s["ms_p90"],
                s["ms_gen_med"], s["tok_s"], s["mode"], s["gpu_mb"]))


def cmd_rescore(a):
    """Re-judge stored runs with the CURRENT clean / post-check (answer.decide) - no model needed. Only for choices
    on the TUNING part; a held-out file is re-scored only with --heldout (after the choices are frozen)."""
    terms = answer.lang_terms()
    for fn in sorted(os.listdir(OUT)):
        if not fn.endswith(".jsonl") or ("_heldout" in fn and not a.heldout):
            continue
        path = os.path.join(OUT, fn)
        head, rows = load_rows(path)
        if head["model"] == "none":
            continue
        searcher = index.Searcher(index.load(head["lang"]))
        qs = {q["id"]: q for q in ai.load_questions(head["lang"])["questions"]}
        new = []
        for r in rows:
            if r.get("search_cant") or not r.get("hits") or (r.get("src") == "retrieval" and not r.get("raw")
                                                              and not r.get("cut")):
                new.append(r)
                continue
            d = answer.decide(r.get("raw", ""), r["hits"], head["lang"], terms, cut=r.get("cut"),
                              mode=r.get("answer_mode") or ("extract" if "+extract" in head["model"] else "generate"))
            r = dict(r, a=d["a"], src=d["src"], problems=d.get("problems", []), pid=d.get("id"),
                     model_text=d.get("model_text", ""), passage=d.get("passage"))
            r.update(judge(qs[r["id"]], {"a": r["a"], "src": r["src"], "id": r["pid"]}, searcher))
            r.pop("quality", None)
            if r["src"] in ("llm", "extract"):
                r["quality"] = quality(r["a"], " ".join(h.get("text", "") for h in r["hits"]), head["lang"])
            new.append(r)
        with open(path, "w", encoding="utf-8") as f:
            for x in [head] + new:
                f.write(json.dumps(x, ensure_ascii=False) + "\n")
        print("%s: %s" % (fn, summary_line((head, new))))
    return 0


def cmd_report(a):
    if not os.path.isdir(OUT):
        print("no runs yet")
        return 1
    print("| model | lang | part | correct | model's own | answered | precision | post-check rejected | can't find "
          "right | false can't find (model's own) | quality | median / p90 ms (generated only) | tok/s | runs on "
          "(GPU layers) | worker GPU MB | free MB at load | load s |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for fn in sorted(os.listdir(OUT)):
        if not fn.endswith(".jsonl"):
            continue
        try:
            head, rows = load_rows(os.path.join(OUT, fn))
        except IndexError:                          # a run still writing its file
            continue
        if a.first:                                 # the same subset for every model (runs made with --limit N)
            rows = rows[:a.first]
        s = summarise(head, rows)
        print("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %d (%d) | %s | %d / %d (%d) | %s | %s (%s) | %s | %s "
              "| %s |" % (
                  head["model"], head["lang"], head["part"], pct(s["correct"], s["in_manual"]),
                  pct(s["model_kept"], s["in_manual"]), pct(s["answered"], s["in_manual"]),
                  pct(s["precision"], s["answered"]), pct(s["rejected"], s["generated"]),
                  pct(s["cant_right"], s["not_in_manual"]), s["false_cant"], s["false_cant_model"],
                  pct(s["quality"], s["quality_n"]), s["ms_med"], s["ms_p90"], s["ms_gen_med"], s["tok_s"],
                  s["mode"], s["layers"], s["gpu_mb"] if s["gpu_mb"] is not None else "?",
                  s["free_mb"] if s["free_mb"] is not None else "-", s["load_s"]))
    return 0


def cmd_blind(a):
    """20 Greek questions x every model's answer, shuffled under letters (the key in a separate file)."""
    import random
    rnd = random.Random(7)
    runs = {}
    for fn in sorted(os.listdir(OUT)):
        if fn.endswith("_el_tuning.jsonl"):
            head, rows = load_rows(os.path.join(OUT, fn))
            if head["model"] != "none":
                runs[head["model"]] = {r["id"]: r for r in rows}
    if not runs:
        print("no Greek tuning runs")
        return 1
    common = sorted(set.intersection(*(set(v) for v in runs.values())))
    pick = [i for i in common if next(iter(runs.values()))[i]["in_manual"]]
    rnd.shuffle(pick)
    pick = sorted(pick[:a.n])
    names = sorted(runs)
    letters = "ABCDEFGH"
    key = {}
    lines = ["# Talking manual - blind review of the Greek answers (step AM4)", "",
             "For each question: rate every answer 1-5 for Greek (natural, correct, easy to hear) and mark x if it "
             "says something WRONG. The letters change per question; the key is in docs/assistant_llm_review_key.md "
             "(do not open it before rating). The text is what the model wrote, before the post-check.", ""]
    for qid in pick:
        order = names[:]
        rnd.shuffle(order)
        key[qid] = {letters[i]: n for i, n in enumerate(order)}
        r0 = runs[order[0]][qid]
        lines += ["## %s  %s" % (qid, r0["q"]), ""]
        for i, n in enumerate(order):
            r = runs[n][qid]
            t = r.get("model_text") or r.get("a") if r["src"] in ("llm", "passage", "extract") else \
                "(" + r["src"] + ") " + r.get("a", "")
            lines.append("- **%s** %s   rating: __  wrong: __" % (letters[i], t))
        lines.append("")
    with open(os.path.join(REPO, "docs", "assistant_llm_review.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    with open(os.path.join(REPO, "docs", "assistant_llm_review_key.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write("# Key of docs/assistant_llm_review.md\n\n" + "\n".join(
            "- %s: %s" % (q, ", ".join("%s = %s" % kv for kv in sorted(key[q].items()))) for q in pick) + "\n")
    print("wrote docs/assistant_llm_review.md + _key.md (%d questions x %d models)" % (len(pick), len(names)))
    return 0


def cmd_e2e(a):
    """Spoken questions through the simulated microphone, the real Whisper + model: release -> answer text."""
    import array
    import wave
    sys.path.insert(0, GS)
    import tts
    os.makedirs(OUT, exist_ok=True)
    name = a.model or paths.LLM_DEFAULT
    qs = [q for q in ai.load_questions("el")["questions"] if q["id"] not in ai.load_heldout("el") and
          (q.get("expect") or q.get("tour"))][:a.n]
    clips = []
    tmp = os.path.join(OUT, "e2e_wav")
    os.makedirs(tmp, exist_ok=True)
    for q in qs:
        p = os.path.join(tmp, q["id"] + ".wav")
        if not os.path.isfile(p):
            tts.synth("el", [(q["q"], p)], timeout=120, choice="windows")
        w = wave.open(p)
        rate, n = w.getframerate(), w.getnframes()
        s = array.array("h", w.readframes(n))
        w.close()
        s = [s[min(len(s) - 1, int(i * rate / 16000))] for i in range(n * 16000 // rate)]
        clips.append((q, s))
    before = gpu_used_mb()
    w = Worker(llm_cfg(name), mic="sim")
    after = gpu_used_mb()
    info = (w.ready or (0, {}))[1]
    print("e2e %s: %s; GPU used %s -> %s MB" % (name, info.get("llm"), before, after), flush=True)
    rows = []
    for q, s in clips:
        w.send({"op": "listen"})
        for i in range(0, len(s), 16000):
            w.send({"op": "inject", "samples": s[i:i + 16000]})
        time.sleep(0.2)
        t0 = time.monotonic()                              # = the release
        w.send({"op": "ask", "lang": "el"})
        h = w.wait("heard", 60)
        r = w.wait("answer", 120) if h and h[1].get("q") else None
        if not r:
            rows.append({"id": q["id"], "heard": h and h[1].get("q"), "ms": None})
            continue
        rows.append({"id": q["id"], "q": q["q"], "heard": h[1]["q"], "stt_ms": h[1].get("ms"),
                     "heard_at_ms": int((h[0] - t0) * 1000), "answer_ms": int((r[0] - t0) * 1000),
                     "src": r[1].get("src"), "a": r[1].get("a"), "gen_ms": r[1].get("gen_ms")})
        print("  %s %5d ms (heard %d ms) %-8s %s -> %s" % (q["id"], rows[-1]["answer_ms"], rows[-1]["heard_at_ms"],
                                                          rows[-1]["src"], h[1]["q"], r[1].get("a", "")[:80]),
              flush=True)
    w.close()
    ok = [r["answer_ms"] for r in rows if r.get("answer_ms")]
    gen = [r["answer_ms"] for r in rows if r.get("answer_ms") and r.get("src") in ("llm", "extract", "passage")]
    res = {"model": name, "llm": info.get("llm"), "device": info.get("device"), "gpu_before": before,
           "gpu_after": after, "rows": rows, "median_ms": int(statistics.median(ok)) if ok else None,
           "max_ms": max(ok) if ok else None, "n": len(ok),
           "generated_median_ms": int(statistics.median(gen)) if gen else None, "generated_n": len(gen),
           "generated_max_ms": max(gen) if gen else None}
    with open(os.path.join(OUT, "e2e_%s.json" % name.replace("+", "_")), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print("e2e %s: release -> answer text median %s ms, max %s ms over %d spoken questions; over the %d answers the "
          "model actually generated: median %s ms, max %s ms (AM4-r MED-4)" % (
              name, res["median_ms"], res["max_ms"], len(ok), len(gen), res["generated_median_ms"],
              res["generated_max_ms"]))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--models", default="")
    r.add_argument("--part", default="tuning", choices=("tuning", "heldout", "all"))
    r.add_argument("--lang", default="el,en")
    r.add_argument("--limit", type=int, default=0)
    r.add_argument("--answer", default="generate", choices=("generate", "extract"),
                   help="the answer mode (AM4-r): extract = the model only picks passage sentences")
    r.add_argument("-v", "--verbose", action="store_true")
    rp = sub.add_parser("report")
    rp.add_argument("--first", type=int, default=0, help="only the first N questions of every run (a common subset)")
    rs = sub.add_parser("rescore")
    rs.add_argument("--heldout", action="store_true")
    b = sub.add_parser("blind")
    b.add_argument("--n", type=int, default=20)
    e = sub.add_parser("e2e")
    e.add_argument("--model", default="", help="a model name, or <model>+extract for the extract answer mode")
    e.add_argument("--n", type=int, default=10)
    a = ap.parse_args(argv)
    return {"run": cmd_run, "report": cmd_report, "rescore": cmd_rescore, "blind": cmd_blind, "e2e": cmd_e2e}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
