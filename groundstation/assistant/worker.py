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

READ-ONLY + OFFLINE (CLAUDE.md W): it starts no process, opens no socket, writes no file (audio in RAM only,
dropped after each question); models are local folder paths; it refuses to run on a gateway PC.
AM3 added "find" (the manual index, read once at init: groundstation/assistant/index/); AM4 adds "answer".
"""
import json
import os
import sys

if __package__ in (None, ""):                     # run as a script by client.py: import as the package
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "assistant"

from assistant import index, mic, paths, stt   # noqa: E402


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
        dev = self.stt.device + ("/" + self.stt.model_name if getattr(self.stt, "model_name", "") else "")
        self.send({"op": "ready", "device": dev, "ms": int((time.monotonic() - t0) * 1000),
                   "index_err": self.index_problem()})

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
