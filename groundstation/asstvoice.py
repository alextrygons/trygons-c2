"""The talking manual's VOICE on the station (step AM5, CLAUDE.md W): the answer the worker found is made into a
one-off WAV by the station's own tts.py (the sound setting voice_engine: Piper or the Windows voice) through the
speech normaliser (assistant/speech.py) and played by sound.py at the LOWEST level (sound.ASSISTANT).

Rules (owner I0 "ALARMS WIN", AM1-r M4, AM4-r LOW-3, AM4-r2 review MED-3):
  * a WARNING / CAUTION sound or a voice callout stops the assistant's voice AT ONCE (the scheduler interrupts it,
    the rest is dropped - [ΞΑΝΑ] says it again); it never delays, queues ahead of or mutes an alarm;
  * while lamps.on(WARNING) on this PC, a repeating warning is not acknowledged, or a callout plays / waits, an
    answer is shown as TEXT ONLY and NOT spoken (no voice started at all; checked again right before the play, after
    the TTS run); [ΞΑΝΑ] speaks it later;
  * never spoken when the answer's speak flag is False («ΟΜΙΛΙΑ» was pressed meanwhile) or while this station holds
    the horn; pressing «ΟΜΙΛΙΑ» stops the voice at once (hailer.Hailer._asst_horn -> stop()); the assistant's voice
    plays only on this PC's sound card through sound.py - never into the loudhailer (no shared channel);
  * a long pick (> speech.LONG_WORDS words) = the chapter intro only is spoken, the text is on the panel;
  * the temporary WAV lives in the system temp folder (never the log folder, never sound_cache/) and is deleted after
    it played (or at once when it is not played); the local log (assistant/qlog.py) gets how it was spoken.
AM5-r (review of 5f15787): MED-1 EVERY WAV this station asked for is in `files` from the moment its name is chosen
until it is really gone: stop() deletes a result the TTS thread finished meanwhile (a new question or «ΟΜΙΛΙΑ» in
the gap before the tick), a thread that finishes after a stop / close deletes its own file, a delete that failed
(the player still has the file open) is tried again at every tick, close() deletes every owed file and every
asst_<this pid>_*.wav left in the folder; a file of a crashed station older than STALE_S is removed at the next
start. MED-3 this module gets a NARROW view (VoicePort: the sound system through a WEAK reference, the horn state
and the language from the assistant's own App-free AsstView) - never the App (test AS6 walks it). LOW-3 the
reason of a failed voice is logged (qlog "detail"). LOW-4 the WAV's gain / resampling run in the TTS thread in
small chunks with a sleep(0) between them (sound.read_wav / gain `breathe`), so the Tk thread is not starved.
W121: a voice not made within TTS_MAX_S is dropped - the answer stays text only ("slow"), the late file deleted.
AM5-r2 (review of 9516ecb): LOW-1 the TTS_MAX_S rule also when a RESULT is taken (the Tk thread stalled: a clip
finished late is never played, "slow"); LOW-2 ONE TTS thread at a time (a new question while one runs waits in
`queued` - the newest request only, an older waiting one is replaced, never a pile-up) and the TTS subprocess is
bounded (tts.synth timeout TTS_TIMEOUT_S = TTS_MAX_S + 2 s); LOW-3 `making` is cleared ONLY by the TTS thread itself
when it is done (a stop / a slow drop never deletes a file under its writer), close() / _clean_stale also remove
'*.wav.tmp' (sound.write_wav's half-written file); LOW-4 the pending result, the generation and `making` change
under ONE lock (the thread's "is this still wanted + hand it over" and the Tk thread's "take it / drop it" are atomic).
AM5-r3 (review of 19db2ff, MED-1 - the SWEEP RACE): tick() takes the pending result FIRST and sweeps LAST; a TTS thread
that finished in between (pending set, `making` cleared) left its clip in `files` with nothing protecting it - the
sweep deleted the clip the NEXT tick would play (the answer was silently lost, "sound_off"). _live() now reads
`pending` and `making` under the lock and protects the pending clip too, with every file's SIBLINGS (`<path>.tmp` of
sound.write_wav and `<path>.lead.wav` of tts.lead_in); a swept / dropped clip takes its siblings along; close() never
deletes under the writer (the thread removes its own files when it finds the generation moved). Test AS27 (v) replays
the race through the real tick() and proves the AM5-r2 _live() loses the clip (negative control).
DISPLAY + sound only: nothing here sends anything.
"""
import glob
import os
import tempfile
import threading
import time
import weakref

import sound
import tts
from assistant import qlog, speech

TMP_SUB = "trygons_asst_voice"
STALE_S = 300.0           # a WAV of a crashed station older than this is removed at the next start (a clip lives
#                           at most TTS_MAX_S + its length + 0.5 s; two stations on one PC never share a name: the pid)
TTS_MAX_S = 8.0           # W121: the TTS run must deliver within this, else the answer stays text only ("slow")
TTS_TIMEOUT_S = TTS_MAX_S + 2.0   # AM5-r2 LOW-2: the TTS subprocess itself is killed by tts.synth after this
TMP_SUFFIXES = (".wav", ".wav.tmp")   # sound.write_wav writes <path>.tmp first (AM5-r2 LOW-3: cleaned too)
SIBLINGS = (".tmp", ".lead.wav")      # AM5-r3: <path>.tmp (sound.write_wav), <path>.lead.wav (tts.lead_in) - live and
#                                       swept together with their clip
TTS_THREAD_NAME = "asst-tts"          # the ONE TTS thread (AS27 counts the alive threads of this name)
WHYS = ("spoken", "long", "alarm", "callout", "horn", "no_voice", "sound_off", "interrupted", "stopped", "failed",
        "slow", "making", "nothing")


def _breathe():
    time.sleep(0)


class VoicePort:
    """The NARROW view AsstVoice gets (AM5-r MED-3, like the App's AsstView for the assistant): `horn_held()` and `lang()`
    are the assistant's own AsstView's (it holds no station object: AS6), `sound()` is the station's SoundSystem
    through a WEAK reference (the SoundSystem itself holds the App for its lamps / telemetry; a weakref is no
    reference the gc walk can follow - the voice can only call the sound system's assistant_* methods). None = no
    sound yet (text only)."""

    def __init__(self, horn_held, lang, sound_system=None):
        self.horn_held = horn_held
        self.lang = lang
        self._snd = None
        self.set_sound(sound_system)

    def set_sound(self, snd):
        self._snd = weakref.ref(snd) if snd is not None else None

    def sound(self):
        return self._snd() if self._snd is not None else None


class AsstVoice:
    def __init__(self, port, synth=None, clock=time.monotonic):
        self.port = port
        self.synth = synth or tts.synth
        self.clock = clock
        self.last = None             # the last answer (info) - [ΞΑΝΑ] says it again
        self.why = ""                # how the last answer was (not) spoken: one of WHYS
        self.detail = ""             # why it failed (logged, AM5-r LOW-3)
        self.kind = ""               # speech.spoken's kind of the last text ("answer" / "long" / "cant_find")
        self.text = ""               # the last spoken text (tests)
        self.pending = None          # (gen, path, secs, detail) made by the TTS thread, played on the Tk thread
        self.gen = 0                 # a stop / a new answer drops the older TTS result
        self._lock = threading.Lock()   # AM5-r2 LOW-4: pending / gen / making change only under it
        self.files = set()           # every WAV owed a delete (AM5-r MED-1): named -> made -> played -> removed
        self.making = None           # the WAV the TTS thread writes now (never deleted under it; AM5-r2 LOW-3: cleared
        #                              ONLY by that thread when it is done - a stop / slow drop leaves it)
        self.make_t0 = 0.0           # when the voice was asked for (speak): TTS_MAX_S counts from here (LOW-1 too)
        self.thread = None           # the ONE TTS thread (AM5-r2 LOW-2: never two at once)
        self.queued = None           # a (gen, lang, text, path, vol) job waiting for that thread to end - the newest only
        self.tmp = None              # the WAV playing now (deleted after its end)
        self.tmp_until = 0.0
        self.hist_n = 0              # sound history index when the clip started (interrupt detection)
        self.folder = os.path.join(tempfile.gettempdir(), TMP_SUB)
        self._clean_stale()

    # --- what the panel says about the voice -----------------------------------------------------------------------
    def status(self):
        return self.why

    # --- speaking --------------------------------------------------------------------------------------------------
    def speak(self, info, user=False):
        """An answer arrived (user=False) or [ΞΑΝΑ] was pressed (user=True). -> True when a voice is being made."""
        self.stop(keep_why=False)
        self.last = info
        snd = self.port.sound()
        lang = info.get("lang") or self.port.lang()
        if not user and info.get("speak") is False:
            return self._why("horn", info)
        if self.port.horn_held():
            return self._why("horn", info)
        if snd is None or not snd.sched.enabled:
            return self._why("sound_off", info)
        blocked = snd.assistant_blocked()
        if blocked:
            return self._why(blocked, info)
        text, kind = speech.spoken(info, lang)
        if not text:
            return self._why("nothing", info)
        if tts.engine(lang) is None:
            return self._why("no_voice", info)
        self.kind, self.text = kind, text
        with self._lock:
            self.gen += 1
            gen = self.gen
        vol = snd.sched.vol(sound.ASSISTANT)
        path = os.path.join(self.folder, "asst_%d_%d.wav" % (os.getpid(), gen))
        self.files.add(path)                     # owed from now on, whatever happens to the TTS run
        self.make_t0 = self.clock()
        self.why, self.detail = "making", ""
        job = (gen, lang, text, path, vol)
        if self._busy():
            self.queued = job                    # AM5-r2 LOW-2: one TTS thread at a time - the running one ends first
        else:                                    # (its result is dropped by the gen), this is the ONLY waiting job
            self._start(job)
        return True

    def _busy(self):
        return self.thread is not None and self.thread.is_alive()

    def _start(self, job):
        self.queued = None
        with self._lock:
            self.making = job[3]
        self.thread = threading.Thread(target=self._make, args=job, daemon=True, name=TTS_THREAD_NAME)
        self.thread.start()

    def _make(self, gen, lang, text, path, vol):
        """The TTS run (a subprocess inside tts.py: Piper or PowerShell, bounded by TTS_TIMEOUT_S) off the Tk thread;
        the gain / resampling in chunks (sound `breathe` = sleep(0) between them). The hand-over of the result and the
        "done" of `making` happen under the lock (AM5-r2 LOW-3 / LOW-4)."""
        try:
            os.makedirs(self.folder, exist_ok=True)
            n = self.synth(lang, [(text, path)], timeout=TTS_TIMEOUT_S)
            if not n or not os.path.exists(path):
                res = (gen, None, 0.0, "tts made no file (engine %s, lang %s)" % (tts.engine(lang), lang))
            else:
                samples = sound.read_wav(path, breathe=_breathe)
                sound.write_wav(path, sound.gain(samples, vol, breathe=_breathe))
                res = (gen, path, len(samples) / float(sound.RATE), "")
        except Exception as e:
            res = (gen, None, 0.0, "%s: %s" % (type(e).__name__, str(e)[:160]))
        with self._lock:
            mine = gen == self.gen
            if mine:
                self.pending = res
            if self.making == path:              # the writer itself says it is done (nothing else clears it)
                self.making = None
        if not mine:                             # stopped / dropped / closed meanwhile: nothing to play, the file goes
            self._remove_all(path)               # (it stays in `files` until a tick / close() sees it gone)

    def tick(self):
        """From App.ov_tick (every 200 ms): play a finished WAV (after the checks again; a result taken later than
        TTS_MAX_S after the request is dropped too - AM5-r2 LOW-1), drop a TTS run that takes too long, start the
        waiting job once the one TTS thread ended, notice an interruption, delete every owed file that is neither
        playing nor being made."""
        with self._lock:
            p, self.pending = self.pending, None
            gen_now = self.gen
        if p is not None:
            gen, path, secs, detail = p
            if gen != gen_now:
                pass                             # dropped meanwhile: the sweep removes the file
            elif path is None:
                self._why("failed", self.last, detail)
            elif self.clock() - self.make_t0 > TTS_MAX_S:
                self._why("slow", self.last, "tts result after %.0f s (not played)" % TTS_MAX_S)
            else:
                self._play(path, secs)
        elif self.why == "making" and self.clock() - self.make_t0 > TTS_MAX_S:
            with self._lock:
                self.gen += 1                    # the late result is dropped by the thread itself (its file too)
            self.queued = None                   # a job still waiting is dropped as well (never started late)
            self._why("slow", self.last, "tts run longer than %.0f s" % TTS_MAX_S)
        if self.queued is not None and not self._busy():
            if self.queued[0] == self.gen:
                self._start(self.queued)
            else:
                self.queued = None
        snd = self.port.sound()
        if self.tmp is not None:
            if self.why == "spoken" and snd is not None and snd.assistant_interrupted_since(self.hist_n):
                self._why("interrupted", self.last)
            if self.clock() >= self.tmp_until and (snd is None or not snd.assistant_playing()):
                self.tmp = None
        self._sweep()

    def _play(self, path, secs):
        snd = self.port.sound()
        if self.port.horn_held():
            return self._why("horn", self.last)
        blocked = snd.assistant_blocked() if snd is not None else "sound_off"
        if blocked:
            return self._why(blocked, self.last)
        self.hist_n = len(snd.sched.history)
        r = snd.assistant_speak(path, secs)
        if r != "played":
            return self._why("callout" if r == "busy" else "sound_off", self.last)
        self.tmp, self.tmp_until = path, self.clock() + secs + 0.5
        return self._why("spoken" if self.kind != "long" else "long", self.last)

    def repeat(self):
        """[ΞΑΝΑ]: the last answer again (also one that was text only - unless the horn is held now)."""
        if self.last:
            return self.speak(self.last, user=True)
        return False

    def stop(self, keep_why=True):
        """At once: the voice stops, a TTS result on its way is dropped (a new question, «ΟΜΙΛΙΑ», the station closes);
        a result the thread already finished is deleted here, not left behind (AM5-r MED-1)."""
        with self._lock:
            self.gen += 1
            self.pending = None                  # (AM5-r2 LOW-4: atomic with the thread's hand-over)
        self.queued = None
        # `making` is NOT cleared here (AM5-r2 LOW-3): the thread still writes that file - it sees the new gen when it
        # is done and removes its own file; until then _live() keeps the sweep off it
        snd = self.port.sound()
        stopped = bool(snd is not None and snd.assistant_stop())
        if stopped and keep_why:
            self.why = "stopped"
        self.tmp = None
        self._sweep()
        return stopped

    def close(self):
        """The station closes: the voice stops and EVERY file this process made is deleted (also one a late TTS
        thread wrote after stop(), and a half-written '.wav.tmp') - except the file the TTS thread writes NOW and its
        siblings (AM5-r3: never under the writer; that thread removes them itself when it finds the generation moved;
        a thread killed with the process leaves them to the next start's _clean_stale)."""
        self.stop()
        self._sweep()
        with self._lock:
            making = self.making
        writer = {making + s for s in ("",) + SIBLINGS} if making else set()   # AM5-r3: never under the writer -
        try:                                                                   # the thread deletes its own (gen moved)
            for suf in TMP_SUFFIXES:
                for p in glob.glob(os.path.join(glob.escape(self.folder), "asst_%d_*%s" % (os.getpid(), suf))):
                    if p not in writer:
                        self._remove(p)
        except OSError:
            pass

    # --- helpers ---------------------------------------------------------------------------------------------------
    def _why(self, why, info, detail=""):
        self.why, self.detail = why, detail or ""
        if info is not None and why != "making":
            qlog.write(info, voice=why, detail=detail or None)
        return False

    def _live(self):
        """The files that must NOT be deleted now: the clip playing, the one the TTS thread writes AND a finished result
        not yet taken by tick() (AM5-r3 MED-1: the thread may finish between tick()'s take and its sweep) - each with
        its siblings (<path>.tmp, <path>.lead.wav)."""
        snd = self.port.sound()
        live = set()
        if self.tmp is not None and (self.clock() < self.tmp_until or (snd is not None and snd.assistant_playing())):
            live.add(self.tmp)
        with self._lock:
            making = self.making
            pending = self.pending[1] if self.pending is not None else None
        for base in (making, pending):
            if base:
                live.add(base)
                live.update(base + s for s in SIBLINGS)
        return live

    def _sweep(self):
        live = self._live()
        for p in list(self.files):
            if p in live:
                continue
            if self._remove_all(p):
                self.files.discard(p)

    def _remove_all(self, path):
        """A clip and its siblings (AM5-r3) -> True when the clip itself is gone (a sibling that stays is caught by
        close() / _clean_stale: it never holds the clip on the list)."""
        for s in SIBLINGS:
            self._remove(path + s)
        return self._remove(path)

    def owed(self):
        """The WAV paths not yet deleted (tests)."""
        return sorted(self.files)

    @staticmethod
    def _remove(path):
        if not path:
            return True
        try:
            os.remove(path)
            return True
        except FileNotFoundError:
            return True
        except OSError:
            return False                   # still open by the player: tried again at the next tick

    def _clean_stale(self):
        try:
            now = time.time()
            for fn in os.listdir(self.folder):
                p = os.path.join(self.folder, fn)
                if fn.endswith(TMP_SUFFIXES) and now - os.path.getmtime(p) > STALE_S:
                    self._remove(p)
        except OSError:
            pass
