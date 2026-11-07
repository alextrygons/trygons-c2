"""The talking manual on the station side (step AM2): starts / restarts the worker, the push-to-talk state machine,
the «ΒΟΗΘΟΣ ΕΚΤΟΣ» notice. The ONLY assistant module app.py imports, and the ONLY process door (subprocess):
the worker itself starts nothing.

READ-ONLY (CLAUDE.md W): this module gets a narrow `view` from app.py (language, horn state, voice stop, a
message line) - never `app` - and imports no link / gateway / proto / station module, no socket.

Push-to-talk (W83): ONLY the framed «ΕΡΩΤΗΣΗ» button (screen 1 bottom bar, screen 2 bar 1) and the key
(gs_settings "assistant_key", default F9; a USB foot pedal sends that key). press(source) accepts the sources
"button" / "key" only: never the joystick. Release = ask; < MIN_S ignored, MAX_S = asked automatically.
The loudhailer «ΟΜΙΛΙΑ» (AM1 review H1): its press aborts a capture AND stops the assistant's voice at once
(horn_pressed()); while this station holds the horn nothing is captured and nothing is spoken (text only).

The answer (step AM4): the worker runs llama.cpp IN its own process through the Python bindings (answer.py) - this
module starts the worker and nothing else: no llama-cli, never llama-server, never a TCP port (CLAUDE.md W, AM1-r
L4). After "heard" the worker sends "answer" (src llm / extract / passage / retrieval / cant_find); the time from
the release to the answer text is measured here (answer_ms). The answer is SHOWN (Tools page, screen 2's message
line); it is spoken from step AM5 - never when its "speak" flag is False (the horn was pressed meanwhile, AM4-r).
"""
import json
import os
import queue
import subprocess
import tempfile
import threading
import time

from . import hints, paths

MIN_S = 0.4                     # a shorter press = ignored (a tap)
MAX_S = 15.0                    # a longer one is asked automatically
INIT_TIMEOUT_S = 180.0          # loading the models (Whisper ~1.6 GB + the language model ~2-5 GB, AM4: 90 -> 180 s)
ANSWER_TIMEOUT_S = 60.0         # no answer = a hang: the worker is restarted (AM4: 30 -> 60 s - a CPU-only model reads
                                # the passages for up to ~20 s before its first word; retrieval-only follows if slow)
RESTART_DELAY_S = 5.0
RESTARTS_MAX = 3                # at most 3 restarts in RESTART_WINDOW_S, then off until the station restarts
RESTART_WINDOW_S = 600.0
PING_S = 10.0                   # a ping every 10 s while READY / LISTENING (AM2 review LOW-5) ...
PONG_TIMEOUT_S = 10.0           # ... no pong (no line at all) within 10 s = a hang: restarted (counts in the 3 / 10 min)
ERR_MAX_BYTES = 1 << 20         # worker_err.txt is rotated to worker_err.1.txt above 1 MB (AM2 review LOW-6)
SOURCES = ("button", "key")     # never "joystick" (CLAUDE.md W)
KEYS = ("F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9")     # F10 = the Windows menu key, F11 / F12 = the screens
DEFAULT_KEY = "F9"
PROBLEM_NOTES = ("asst.n_mic", "asst.n_error", "asst.n_index")    # amber on the message line; the other notes = plain info (LOW-4)
# how the model answers (AM4-r, docs/assistant_design.md 5d; gs_settings "assistant_answer_mode" / the environment
# TRYGONS_ASSISTANT_ANSWER): "extract" = the model only PICKS manual sentences by number, said word for word;
# "generate" = it writes 1-2 sentences from the passages (post-checked). The default follows the AM4-r measurement.
DEFAULT_ANSWER_MODE = "extract"
# AM4-r2 (W112): in "extract" the best passage's search score must reach this before the model may pick, else "can't
# find" + the passage shown; 0 = no floor. 7.2 = just above the 3 off-manual questions extract still answered on the
# tuning part (6.07 / 6.79 / 7.10; costs 3 of 41 correct Greek answers, no English one - docs/assistant_design.md
# 5e); gs_settings "assistant_extract_floor" (a number)
DEFAULT_EXTRACT_FLOOR = 7.2

# states
OFF, IDLE, NO_MODELS, GATEWAY_PC, STARTING, READY, LISTENING, THINKING, DOWN, FAILED = (
    "off", "idle", "no_models", "gateway_pc", "starting", "ready", "listening", "thinking", "down", "failed")


def log_dir():
    """Where the assistant's local files go (THIS PC only, gitignored); a test run = a temp folder."""
    d = os.environ.get("TRYGONS_ASSISTANT_LOG")
    if not d:
        d = os.path.join(tempfile.gettempdir(), "trygons_test_assistant") if os.environ.get("TRYGONS_TEST") \
            else os.path.join(paths.HERE, "log")
    os.makedirs(d, exist_ok=True)
    return d


class Assistant:
    def __init__(self, view, settings=None, clock=time.monotonic, popen=None, stt_kind=None, stub_text="",
                 mic_backend=None):
        self.view = view
        self.settings = settings                 # get(key, default) of the station's gs_settings (or None)
        self.clock = clock
        self.popen = popen or subprocess.Popen
        self.stt_kind = stt_kind or os.environ.get("TRYGONS_ASSISTANT_STT") or "faster"
        self.stub_text = stub_text
        self.mic_backend = mic_backend or os.environ.get("TRYGONS_ASSISTANT_MIC") or "auto"
        self.proc = None
        self.q = queue.Queue()
        self.state = IDLE if self.enabled() else OFF     # IDLE = on, not started yet
        self.why = ""
        self.device = ""
        self.index_err = ""                      # the worker's manual index is stale / missing (AM3-r MED-1)
        self.llm = {}                            # the worker's language model: name, mode (gpu / gpu+cpu / cpu / retrieval) ...
        self.answer = None                       # the last "answer" (+ "answer_ms" = from the release to the text)
        self.release_t = None
        self.pressed_t = None
        self.ask_t = None
        self.started_t = None
        self.down_t = None
        self.restarts = []                       # times of the restarts (the 3-in-10-min limit)
        self.last = None                         # the last "heard" result
        self.note = ""                           # the last refusal / problem (screen language key + args)
        self.gen = 0                             # the worker generation (an old worker's lines are dropped)
        self.heard_t = None                      # the last line from the worker (alive)
        self.ping_t = None                       # an unanswered ping was sent at this time
        self.horn_since_ask = False              # «ΟΜΙΛΙΑ» went down while a question was being answered (AM4-r
        #                                          LOW-3: the answer then carries speak = False - AM5 enforces it)

    # --- settings ------------------------------------------------------------------------------------------------
    def _get(self, k, dflt):
        try:
            return self.settings.get(k, dflt) if self.settings is not None else dflt
        except Exception:
            return dflt

    def enabled(self):
        return bool(self._get("assistant", True)) and os.environ.get("TRYGONS_ASSISTANT") != "off"

    def key(self):
        k = self._get("assistant_key", DEFAULT_KEY)
        return k if k in KEYS else DEFAULT_KEY

    def mic_device(self):
        m = self._get("assistant_mic", None)
        return m if isinstance(m, int) and not isinstance(m, bool) and m >= 0 else None

    # --- the worker -------------------------------------------------------------------------------------------------
    def worker_cmd(self):
        py = paths.assistant_python() if self.stt_kind == "faster" else (paths.assistant_python() or
                                                                           _this_python())
        return [py, os.path.join(paths.HERE, "worker.py")] if py else None

    def llm_cfg(self):
        """What the worker loads as the language model (AM4). $TRYGONS_ASSISTANT_LLM = "stub" / "none" (tests) or a
        model folder name / .gguf path; gs_settings "assistant_llm" = "off" = retrieval only on this PC."""
        env = os.environ.get("TRYGONS_ASSISTANT_LLM", "")
        if env == "stub" or (self.stt_kind == "stub" and not env):
            return {"kind": "stub", "reply": os.environ.get("TRYGONS_ASSISTANT_LLM_REPLY", "@first"),
                    "delay_s": float(os.environ.get("TRYGONS_ASSISTANT_LLM_DELAY") or 0),
                    "answer_mode": os.environ.get("TRYGONS_ASSISTANT_ANSWER") or "generate"}
        if env == "none" or self._get("assistant_llm", "on") == "off":
            return {"kind": "none"}
        m = paths.llm_model()
        if not m or not paths.llm_py_dir():
            return {"kind": "none"}
        mode = os.environ.get("TRYGONS_ASSISTANT_ANSWER") or self._get("assistant_answer_mode", DEFAULT_ANSWER_MODE)
        return {"kind": "llama", "model": m, "py": paths.llm_py_dir(), "device": self._get("assistant_device", "auto"),
                "answer_mode": mode if mode in ("generate", "extract") else DEFAULT_ANSWER_MODE,
                "allow_unpinned": bool(self._get("assistant_llm_unpinned", False)),
                # AM4-r2 / W112: the search-score floor before "extract" may pick (0 = none, the measured default)
                "extract_floor": self._get("assistant_extract_floor", DEFAULT_EXTRACT_FLOOR)}

    def models_ok(self):
        if self.stt_kind == "stub":
            return True
        return paths.models_installed()

    def start(self):
        """Start the worker (setting on, models installed, not a gateway PC). Never raises."""
        if not self.enabled():
            return self._set(OFF)
        if paths.on_gateway_pc():
            return self._set(GATEWAY_PC, "the assistant never runs on a gateway PC")
        if not self.models_ok():
            return self._set(NO_MODELS, "models not installed (python tools/get_whisper.py)")
        cmd = self.worker_cmd()
        if not cmd:
            return self._set(NO_MODELS, "no assistant Python (python tools/get_whisper.py)")
        env = dict(os.environ, HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", HF_HUB_DISABLE_TELEMETRY="1",
                   PYTHONIOENCODING="utf-8")
        kw = {}
        if os.name == "nt":
            kw["creationflags"] = 0x08000000 | 0x00004000          # CREATE_NO_WINDOW | BELOW_NORMAL_PRIORITY_CLASS
        err = None
        try:
            err = open(_err_file(), "a", encoding="utf-8")
            self.proc = self.popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, env=env,
                                   cwd=paths.GS, **kw)
        except OSError as e:
            return self._crash("worker did not start: %s" % str(e)[:80])
        finally:
            if err is not None:                  # the child keeps its own handle; ours is closed (also on a failure)
                err.close()
        self.gen += 1
        threading.Thread(target=self._reader, args=(self.proc, self.gen), daemon=True).start()
        self.started_t = self.clock()
        self.heard_t, self.ping_t = self.started_t, None
        self._set(STARTING)
        gpu, cpu = paths.whisper_model(paths.MODEL_GPU), paths.whisper_model(paths.MODEL_CPU)
        self._send({"op": "init", "stt": self.stt_kind, "model": gpu or cpu, "cpu_model": cpu,
                    "device": self._get("assistant_device", "auto"), "mic_backend": self.mic_backend,
                    "mic": self.mic_device(), "lang": self.view.lang(), "stub_text": self.stub_text,
                    "hints": {"el": hints.prompt("el"), "en": hints.prompt("en")}, "llm": self.llm_cfg()})

    def _reader(self, proc, gen):
        try:
            for line in proc.stdout:
                try:
                    self.q.put((gen, json.loads(line)))
                except ValueError:
                    pass
        except (OSError, ValueError):
            pass
        self.q.put((gen, {"op": "_eof"}))

    def _send(self, d):
        try:
            self.proc.stdin.write((json.dumps(d) + "\n").encode("ascii"))
            self.proc.stdin.flush()
            return True
        except (OSError, AttributeError, ValueError):
            self._crash("worker pipe closed")
            return False

    def stop(self):
        p, self.proc = self.proc, None
        self.gen += 1
        self.pressed_t = self.ask_t = self.heard_t = self.ping_t = None
        self._set(IDLE if self.enabled() else OFF)
        if p is not None:
            try:
                p.stdin.close()
            except (OSError, AttributeError):
                pass
            try:
                p.wait(2.0)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass

    def _set(self, st, why=""):
        self.state, self.why = st, why

    def _crash(self, why):
        """A crash / hang: notice «ΒΟΗΘΟΣ ΕΚΤΟΣ», restart after RESTART_DELAY_S (3 in 10 min, then off)."""
        p, self.proc = self.proc, None
        self.gen += 1
        if p is not None:
            try:
                p.kill()
            except Exception:
                pass
        now = self.clock()
        self.restarts = [t for t in self.restarts if now - t < RESTART_WINDOW_S]
        self.pressed_t = self.ask_t = None
        self.view.voice_stop()
        if len(self.restarts) >= RESTARTS_MAX:
            return self._set(FAILED, why)
        self.down_t = now
        self._set(DOWN, why)

    # --- push to talk -----------------------------------------------------------------------------------------------
    def press(self, source):
        """«ΕΡΩΤΗΣΗ» / the key went down. -> True when listening."""
        if source not in SOURCES:
            return False                          # never the joystick
        if self.state == LISTENING:
            return True                           # the key's auto-repeat
        if self.view.horn_held():
            self.note = ("asst.n_horn", {})
            return False
        if self.state != READY:
            self.note = ("asst.n_not_ready", {})
            return False
        self.view.voice_stop()                    # a new question silences the old answer
        if not self._send({"op": "listen"}):
            return False
        self.pressed_t = self.clock()
        self.note = ""
        self._set(LISTENING)
        return True

    def release(self):
        if self.state != LISTENING:
            return
        held = self.clock() - self.pressed_t
        self.pressed_t = None
        if held < MIN_S:
            self._send({"op": "abort"})
            self.note = ("asst.n_short", {})
            return self._set(READY)
        self._ask()

    def _ask(self):
        self.pressed_t = None
        if self._send({"op": "ask", "lang": self.view.lang()}):
            self.ask_t = self.release_t = self.clock()
            self.horn_since_ask = False
            self._set(THINKING)

    def abort(self):
        if self.state == LISTENING:
            self._send({"op": "abort"})
            self.pressed_t = None
            self._set(READY)

    def horn_pressed(self):
        """The loudhailer «ΟΜΙΛΙΑ» went down (any source): capture dropped + the assistant's voice stopped AT ONCE;
        an answer still on its way will not be spoken (speak = False)."""
        self.abort()
        self.horn_since_ask = True
        self.view.voice_stop()

    # --- the Tk loop (every 100 ms) -----------------------------------------------------------------------------------
    def tick(self):
        now = self.clock()
        if self.view.horn_held():                 # this station holds the horn: never capture, never speak
            self.abort()
            self.horn_since_ask = True
            self.view.voice_stop()
        while True:
            try:
                gen, m = self.q.get_nowait()
            except queue.Empty:
                break
            if gen != self.gen:
                continue                          # a line of an old worker
            if m.get("op") != "_eof":
                self.heard_t, self.ping_t = now, None     # any line = alive (a pong, or an answer after a busy ask)
            self._on(m)
        if self.state == LISTENING and now - self.pressed_t >= MAX_S:
            self._ask()
        elif self.state == THINKING and now - self.ask_t > ANSWER_TIMEOUT_S:
            self._crash("no answer in %.0f s" % ANSWER_TIMEOUT_S)
        elif self.state == STARTING and now - self.started_t > INIT_TIMEOUT_S:
            self._crash("worker did not get ready in %.0f s" % INIT_TIMEOUT_S)
        elif self.state == DOWN and now - self.down_t >= RESTART_DELAY_S:
            self.restarts.append(now)
            self.start()
        elif self.state in (READY, LISTENING) and self.proc is not None:   # alive check (LOW-5); THINKING: timeout
            if self.ping_t is not None and now - self.ping_t > PONG_TIMEOUT_S:
                self._crash("no pong in %.0f s" % PONG_TIMEOUT_S)
            elif self.ping_t is None and now - (self.heard_t or now) >= PING_S:
                if self._send({"op": "ping"}):
                    self.ping_t = now
        if self.proc is not None and self.state in (READY, LISTENING, THINKING, STARTING):
            try:
                rc = self.proc.poll()
            except Exception:
                rc = None
            if rc is not None:
                self._crash("worker exited (%s)" % rc)

    def _on(self, m):
        op = m.get("op")
        if op == "ready":
            self.device = m.get("device", "")
            self.index_err = m.get("index_err") or ""
            self.llm = m.get("llm") or {}
            if self.index_err:                   # every search = can't find; the station says why once
                self.note = ("asst.n_index", {})
            self._set(READY)
        elif op == "heard":
            self.last = dict(m, t=time.time())
            if m.get("q"):                       # the answer follows (AM4): still THINKING, the timeout runs on
                self.view.on_heard(m["q"], m)
            else:
                self.ask_t = None
                if self.state == THINKING:
                    self._set(READY)
                self.note = ("asst.n_nothing", {})
        elif op == "answer":
            self.ask_t = None
            a = dict(m, t=time.time())
            if self.release_t is not None:
                a["answer_ms"] = int((self.clock() - self.release_t) * 1000)
            self.release_t = None
            # AM4-r LOW-3: «ΟΜΙΛΙΑ» pressed / held since the question = this answer is SHOWN but must NOT be spoken
            a["speak"] = not (self.horn_since_ask or self.view.horn_held())
            if not a["speak"]:
                a["no_speak_why"] = "horn"
            self.answer = a
            if m.get("llm"):                     # the GPU guard moved the model (AM4)
                self.llm = m["llm"]
            if self.state == THINKING:
                self._set(READY)
            on = getattr(self.view, "on_answer", None)
            if on is not None:
                on(a.get("a", ""), a)
        elif op == "mic_error":
            self.note = ("asst.n_mic", {"err": m.get("why", "")})
        elif op == "refused":
            self.stop()
            self._set(GATEWAY_PC, m.get("why", ""))
        elif op == "error":
            if self.state == STARTING:
                self._crash(m.get("why", "error"))
            else:
                self.note = ("asst.n_error", {"err": m.get("why", "")})
                if self.state == THINKING:
                    self._set(READY)
        elif op == "_eof":
            if self.state not in (OFF, IDLE, NO_MODELS, GATEWAY_PC, FAILED, DOWN):
                self._crash("worker exited")

    # --- what the screens show ------------------------------------------------------------------------------------------
    def out_of_service(self):
        """The station-local notice «ΒΟΗΘΟΣ ΕΚΤΟΣ» (never an alarm, no sound): a crash / hang / restarts used up."""
        return self.state in (DOWN, FAILED)

    def view_bar(self):
        """For the bars: label key, enabled, on (listening). None = not shown (the setting is off)."""
        if self.state == OFF or not self.enabled():
            return None
        return {"on": self.state == LISTENING, "enabled": self.state in (READY, LISTENING),
                "busy": self.state == THINKING}

    def take_note(self):
        """The last refusal / problem as (lang key, args) once, for the station's message line; else None."""
        n, self.note = self.note, ""
        return n or None

    def inject(self, samples):
        """Tests (mic backend "sim"): samples as if spoken into the microphone."""
        return self._send({"op": "inject", "samples": list(samples)})


def _err_file():
    """The worker's stderr file in log_dir(), rotated once above ERR_MAX_BYTES (one old copy kept)."""
    p = os.path.join(log_dir(), "worker_err.txt")
    try:
        if os.path.getsize(p) > ERR_MAX_BYTES:
            os.replace(p, os.path.join(log_dir(), "worker_err.1.txt"))
    except OSError:
        pass
    return p


def _this_python():
    import sys
    return sys.executable
