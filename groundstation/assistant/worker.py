"""The talking manual's worker process (step AM2): its OWN below-normal-priority process on the station PC
(started ONLY by client.py). JSON lines on stdin / stdout; it ends when stdin closes.

    -> {"op": "init", "stt": "faster" | "stub", "model": <local folder>, "device": "auto" | "cpu" | "cuda",
        "mic_backend": "auto" | "winmm" | "sim" | "null", "mic": <device index> | null, "hints": {"el", "en"},
        "stub_text": "...", "lang": "el"}
    <- {"op": "ready", "device", "ms"} | {"op": "error", "why"}
    -> {"op": "listen"}             the microphone opens, capture into RAM
    -> {"op": "ask", "lang"}        capture stops -> speech recognition
    <- {"op": "heard", "q", "lang", "ms", "audio_s", "db", "device"}   (q "" = nothing understood)
    -> {"op": "abort"}              capture dropped (the loudhailer was pressed, the station closes ...)
    -> {"op": "ping"}  <- {"op": "pong"}
    -> {"op": "inject", "samples"}  tests with mic_backend "sim" only
    -> {"op": "find", "q", "lang"}  the manual search alone (step AM3) <- {"op": "found", "q", "lang", "ms",
                                    "cant_find", "why", "hits": [{id, ch, chapter, heading, term, score, cover, text}]}
    The "heard" answer carries "found" too (the same search on the recognised question, AM3).
    AM4: init also takes "llm": {"kind": "llama" | "stub" | "none", "model": <local .gguf>, "py": <bindings folder>,
        "device": "auto" | "cpu" | "cuda", "n_ctx", "reply" / "delay_s" (stub), "answer_mode": "generate" |
        "extract" (AM4-r), "allow_unpinned": bool (AM4-r LOW-2: a model file not in paths.LLM_PINNED),
        "extract_floor": number (AM4-r2 / W112: the best passage's search score the extract mode needs)}; ready
        carries "llm" (name, mode gpu / gpu+cpu / cpu / retrieval, layers, why, free / need MB, load ms, sha_ms).
        After every "heard" with a question:
    <- {"op": "answer", "q", "lang", "a", "src" (llm | extract | passage | retrieval | cant_find), "mode", "cite",
        "id", "passage" {text, cite, id} (what the panel shows), "problems", "ms", "gen_ms", "first_ms", "tokens",
        "found"}
    -> {"op": "ask_text", "q", "lang"}  a typed question: the same search + answer (tests, tools/assistant_llm_eval.py)

READ-ONLY + OFFLINE (CLAUDE.md W): it starts no process, opens no socket, writes no file (audio in RAM only,
dropped after each question); models are local folder paths; it refuses to run on a gateway PC.
AM3 added "find" (the manual index, read once at init: groundstation/assistant/index/); AM4 the answer: llama.cpp
runs IN this process through its Python bindings (answer.LlamaLlm) - no exe, no llama-cli / llama-server, no port.
"""
import json
import os
import sys

if __package__ in (None, ""):                     # run as a script by client.py: import as the package
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "assistant"

from assistant import answer, gpu, index, mic, paths, stt   # noqa: E402

GPU_RETRY_S = 120.0       # after a CPU reload by the guard: the GPU is tried again at most this often (AM4-r LOW-3)


def below_normal():
    """Below normal priority (not IDLE: an answer has a latency target; the station's windows stay normal)."""
    try:
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            k = ctypes.WinDLL("kernel32", use_last_error=True)
            k.GetCurrentProcess.restype = wintypes.HANDLE
            k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            return bool(k.SetPriorityClass(k.GetCurrentProcess(), 0x00004000))   # BELOW_NORMAL_PRIORITY_CLASS
        os.nice(10)
        return True
    except (OSError, AttributeError, ValueError):
        return False


class Worker:
    def __init__(self, out=None):
        self.out = out or sys.stdout
        self.stt = None
        self.cfg = {}
        self.inp = None                           # the microphone while listening
        self.search = {}                          # lang -> index.Searcher (the manual index, read once)
        self.search_err = {}                      # lang -> why that index is not used (missing / broken / stale)
        self.ans = answer.Answerer(None)          # AM4: no model = retrieval only (until init loads one)
        self.llm_cfg = {}
        self.llm_info = {"mode": "retrieval", "why": "no language model"}
        self.cpu_forced_t = None                  # the guard moved the model to the CPU at this time (RAM only:
        #                                           a worker restart plans afresh - nothing persists, AM4-r LOW-3)
        self.gpu_retry_t = None                   # the last time the guard tried the GPU again

    def send(self, d):
        self.out.write(json.dumps(d) + "\n")      # ASCII escapes: the pipe's code page never matters
        self.out.flush()

    def handle(self, m):
        op = m.get("op")
        if op == "init":
            return self.init(m)
        if op == "ping":
            return self.send({"op": "pong"})
        if op == "listen":
            return self.listen()
        if op == "inject":
            if isinstance(self.inp, mic.SimIn):
                self.inp.inject(m.get("samples") or [])
            return None
        if op == "ask":
            return self.ask(m.get("lang") or self.cfg.get("lang") or "el")
        if op == "abort":
            return self.close_mic()
        if op == "find":
            return self.find(m.get("q") or "", m.get("lang") or self.cfg.get("lang") or "el")
        if op == "ask_text":
            q, lang = m.get("q") or "", m.get("lang") or self.cfg.get("lang") or "el"
            return self.reply(q, lang, self.found(q, lang))
        return self.send({"op": "error", "why": "unknown op %r" % op})

    def init(self, m):
        import time
        t0 = time.monotonic()
        self.cfg = m
        try:
            self.stt = stt.load(m.get("stt", "faster"), m.get("model"), m.get("device", "auto"),
                                text=m.get("stub_text", ""), lang=m.get("lang", "el"),
                                cpu_model_dir=m.get("cpu_model"))
        except stt.Refused as e:
            return self.send({"op": "error", "why": str(e)})
        except Exception as e:                    # a broken install: never a crash loop of tracebacks
            return self.send({"op": "error", "why": "speech recognition failed to load: %s" % str(e)[:120]})
        self.load_index()
        self.load_llm(m.get("llm") or {})
        dev = self.stt.device + ("/" + self.stt.model_name if getattr(self.stt, "model_name", "") else "")
        self.send({"op": "ready", "device": dev, "ms": int((time.monotonic() - t0) * 1000),
                   "index_err": self.index_problem(), "llm": self.llm_info})

    # --- the language model (AM4) ------------------------------------------------------------------------------
    def load_llm(self, c, device=None):
        """The answer step's model: a stub (tests), llama.cpp through its bindings (a LOCAL .gguf), or none (=
        retrieval only). Where it runs = gpu.plan (the GPU guard); a failed GPU load is tried once on the CPU."""
        import time
        t0 = time.monotonic()
        self.llm_cfg = dict(c)
        old = getattr(self.ans, "llm", None)
        if old is not None and hasattr(old, "close"):
            old.close()
        kind = c.get("kind") or "none"
        terms = answer.lang_terms()
        llm, info = None, {"mode": "retrieval", "why": "no language model", "name": ""}
        if kind == "stub":
            llm = answer.StubLlm(c.get("reply", "@first"), float(c.get("delay_s") or 0), mode=c.get("mode", "gpu"))
            info = {"mode": llm.mode, "why": "stub", "name": "stub", "layers": -1}
        elif kind == "llama":
            try:
                llm, info = self._llama(c, device or c.get("device") or "auto")
            except Exception as e:                # missing bindings / file / out of memory: retrieval only
                llm, info = None, {"mode": "retrieval", "why": "language model not loaded: %s" % str(e)[:160],
                                   "name": os.path.basename(str(c.get("model") or ""))}
        self.ans = answer.Answerer(llm, terms, mode=c.get("answer_mode") or "generate",
                                   extract_floor=c.get("extract_floor", answer.EXTRACT_FLOOR))
        self.ans.stick = not c.get("measure")    # tools/assistant_llm_eval.py: never stuck in retrieval-only
        if llm is None:
            self.ans.why_ro = info["why"]
        info["load_ms"] = int((time.monotonic() - t0) * 1000)
        info["answer_mode"] = self.ans.mode
        self.llm_info = info
        return info

    def _llama(self, c, device):
        import time
        path = c.get("model")
        if not isinstance(path, str) or not os.path.isabs(path) or not os.path.isfile(path):
            raise ValueError("the language model must be a local .gguf file: %r" % (path,))
        t0 = time.monotonic()                     # AM4-r LOW-2: the file is loaded only after its sha256 matches
        ok, why_sha = paths.llm_verify(path, allow_unpinned=bool(c.get("allow_unpinned")))
        if not ok:
            raise ValueError(why_sha)
        sha_ms = int((time.monotonic() - t0) * 1000)
        answer.bindings_env(c.get("py"))
        meta = gpu.gguf_meta(path)
        free = gpu.free_mb()
        n_ctx = int(c.get("n_ctx") or answer.N_CTX)
        p = gpu.plan(meta, free, n_ctx, device=device)
        info = {"name": meta.get("name") or os.path.basename(path), "file": os.path.basename(path),
                "mode": p["mode"], "layers": p["layers"], "why": p["why"], "need_mb": p["need_mb"],
                "free_mb": free[0] if free else None, "n_ctx": n_ctx, "sha_ms": sha_ms, "sha": why_sha}
        try:
            return answer.LlamaLlm(path, layers=p["layers"], n_ctx=n_ctx, mode=p["mode"]), info
        except (RuntimeError, ValueError, MemoryError, OSError) as e:
            if p["mode"] == "cpu" or not os.path.isfile(str(path)):
                raise
            info.update(mode="cpu", layers=0, why="the GPU load failed (%s) - the CPU" % str(e)[:80])
            return answer.LlamaLlm(path, layers=0, n_ctx=n_ctx, mode="cpu"), info

    def guard(self, now=None):
        """Before every answer: the model on the GPU and less than gpu.LOW_MB free = the other programs on this PC
        are short of GPU memory = the model is reloaded with a new plan (usually the CPU). AM4-r LOW-3: once the
        guard moved it to the CPU, the GPU is tried again when the memory is free again (the plan's need + the
        margin), at most every GPU_RETRY_S; the user's own "cpu" setting is never overridden. All of it in RAM:
        a worker restart plans afresh from the real free memory."""
        import time
        now = time.monotonic() if now is None else now
        llm = getattr(self.ans, "llm", None)
        if llm is None or self.llm_cfg.get("kind") != "llama":
            return None
        free = gpu.free_mb()
        if getattr(llm, "mode", "cpu") != "cpu":
            if free is not None and free[0] >= gpu.LOW_MB:
                return None
            info = self._reload("cpu")
            info["why"] = "GPU memory short before an answer (%s MB free) - reloaded on the CPU" % (
                free[0] if free else "?")
            self.cpu_forced_t, self.gpu_retry_t = now, now
            return info
        if self.cpu_forced_t is None or free is None:
            return None                           # the CPU by the plan / the setting / no GPU: nothing to retry
        if now - self.gpu_retry_t < GPU_RETRY_S:
            return None
        self.gpu_retry_t = now
        need = (self.llm_info or {}).get("need_mb") or 0
        if free[0] < need + gpu.MARGIN_MB:
            return None
        info = self._reload(self.llm_cfg.get("device") or "auto")
        if info.get("mode") == "cpu":
            return None                           # the plan still says CPU: nothing changed, no message
        info["why"] = "GPU memory free again (%s MB) - %s" % (free[0], info.get("why") or "reloaded on the GPU")
        self.cpu_forced_t = None
        return info

    def _reload(self, device):
        """The model loaded again with another plan; the answerer's slow-mode state kept."""
        ro, why_ro, slow = self.ans.retrieval_only, self.ans.why_ro, self.ans.slow
        info = self.load_llm(self.llm_cfg, device=device)
        if ro:
            self.ans.retrieval_only, self.ans.why_ro, self.ans.slow = ro, why_ro, slow
        return info

    def reply(self, q, lang, found):
        """The answer to one question (after "heard" / for "ask_text")."""
        moved = self.guard()
        r = self.ans.answer(q, lang, found)
        r.update(op="answer", found=found)
        if self.ans.retrieval_only and r["mode"] == "retrieval":
            r["why_mode"] = self.ans.why_ro
        if moved:
            r["llm"] = moved
        self.send(r)

    def load_index(self):
        """Both manual indexes, read once. A missing / broken index OR a STALE one (AM3-r MED-1: its manual / tour /
        parser changed since it was built - it could point at text that is no longer there) = every search in
        that language "can't find" + the reason; never the other language's index (LOW-2)."""
        self.search, self.search_err = {}, {}
        for lang in ("el", "en"):
            try:
                why = index.stale(lang)
                if why:
                    self.search_err[lang] = "manual index %s stale: %s" % (lang, "; ".join(why)[:100])
                    continue
                self.search[lang] = index.Searcher(index.load(lang))
            except (OSError, ValueError, KeyError) as e:
                self.search_err[lang] = "manual index %s: %s" % (lang, str(e)[:100])

    def index_problem(self):
        """The text of the station notice for a stale / missing index ("" = both indexes usable)."""
        return "; ".join(self.search_err[k] for k in sorted(self.search_err))

    def found(self, q, lang):
        import time
        t0 = time.monotonic()
        if not self.search and not self.search_err:
            self.load_index()
        s = self.search.get(lang)                 # only the question's language (AM3-r LOW-2)
        if s is None:
            r = {"cant_find": True, "why": self.search_err.get(lang) or "no manual index for %r" % lang, "hits": []}
        else:
            r = index.hit_summary(s.search(q))
        r["ms"] = int((time.monotonic() - t0) * 1000)
        return r

    def find(self, q, lang):
        r = self.found(q, lang)
        r.update(op="found", q=q, lang=lang)
        self.send(r)

    def listen(self):
        self.close_mic()
        self.inp, err = mic.open_input(self.cfg.get("mic_backend", "auto"), self.cfg.get("mic"))
        if err:
            self.send({"op": "mic_error", "why": err})

    def close_mic(self):
        samples = None
        if self.inp is not None:
            samples = self.inp.read_all()
            self.inp.close()
            self.inp = None
        return samples

    def ask(self, lang):
        samples = self.close_mic()
        if samples is None:
            return self.send({"op": "heard", "q": "", "lang": lang, "ms": 0, "audio_s": 0.0, "db": -90.0,
                              "device": getattr(self.stt, "device", "")})
        audio_s = len(samples) / float(mic.RATE)
        db = mic.level_db(samples)
        if self.stt is None:
            return self.send({"op": "error", "why": "not initialised"})
        hints = (self.cfg.get("hints") or {}).get(lang, "")
        try:
            r = self.stt.transcribe(samples, lang=lang, hints=hints)
        except Exception as e:
            return self.send({"op": "error", "why": "speech recognition failed: %s" % str(e)[:120]})
        finally:
            del samples                           # RAM only, dropped after the question
        out = {"op": "heard", "q": r["text"], "lang": r["lang"], "ms": r["ms"], "audio_s": round(audio_s, 2),
               "db": round(db, 1), "device": r["device"]}
        if r["text"]:
            out["found"] = self.found(r["text"], r["lang"])
        self.send(out)
        if r["text"]:
            self.reply(r["text"], r["lang"], out["found"])


def main():
    if paths.on_gateway_pc():
        sys.stdout.write(json.dumps({"op": "refused", "why": "the assistant never runs on a gateway PC"}) + "\n")
        sys.stdout.flush()
        return 3
    below_normal()
    w = Worker()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            m = json.loads(line)
        except ValueError:
            w.send({"op": "error", "why": "bad line"})
            continue
        if m.get("op") == "quit":
            break
        w.handle(m)
    w.close_mic()
    return 0


if __name__ == "__main__":
    sys.exit(main())
