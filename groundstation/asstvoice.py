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
DISPLAY + sound only: nothing here sends anything; it gets the App only for app.sound / app.asst / app.root.
"""
import os
import tempfile
import threading
import time

import sound
import tts
from assistant import qlog, speech

TMP_SUB = "trygons_asst_voice"
STALE_S = 3600.0          # a WAV of a crashed station older than this is removed at the next start
WHYS = ("spoken", "long", "alarm", "callout", "horn", "no_voice", "sound_off", "interrupted", "stopped", "failed",
        "making", "nothing")


class AsstVoice:
    def __init__(self, app, synth=None, clock=time.monotonic):
        self.app = app
        self.synth = synth or tts.synth
        self.clock = clock
        self.last = None             # the last answer (info) - [ΞΑΝΑ] says it again
        self.why = ""                # how the last answer was (not) spoken: one of WHYS
        self.kind = ""               # speech.spoken's kind of the last text ("answer" / "long" / "cant_find")
        self.text = ""               # the last spoken text (tests)
        self.pending = None          # (gen, path, secs) made by the TTS thread, played on the Tk thread (tick)
        self.gen = 0                 # a stop / a new answer drops the older TTS result
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
        snd = getattr(self.app, "sound", None)
        asst = getattr(self.app, "asst", None)
        lang = info.get("lang") or (asst.view.lang() if asst is not None else "el")
        if not user and info.get("speak") is False:
            return self._why("horn", info)
        if asst is not None and asst.view.horn_held():
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
        self.gen += 1
        gen = self.gen
        vol = snd.sched.vol(sound.ASSISTANT)
        path = os.path.join(self.folder, "asst_%d_%d.wav" % (os.getpid(), gen))
        self.why = "making"
        th = threading.Thread(target=self._make, args=(gen, lang, text, path, vol), daemon=True)
        th.start()
        return True

    def _make(self, gen, lang, text, path, vol):
        """The TTS run (a subprocess inside tts.py: Piper or PowerShell) off the Tk thread."""
        try:
            os.makedirs(self.folder, exist_ok=True)
            n = self.synth(lang, [(text, path)])
            if not n or not os.path.exists(path):
                self.pending = (gen, None, 0.0)
                return
            samples = sound.read_wav(path)
            sound.write_wav(path, sound.gain(samples, vol))
            secs = len(samples) / float(sound.RATE)
            self.pending = (gen, path, secs)
        except Exception:
            self.pending = (gen, None, 0.0)

    def tick(self):
        """From App.ov_tick (every 200 ms): play a finished WAV (after the checks again), notice an interruption,
        delete the file after its end."""
        p, self.pending = self.pending, None
        if p is not None:
            gen, path, secs = p
            if gen != self.gen or path is None:
                self._remove(path)
                if gen == self.gen and path is None:
                    self._why("failed", self.last)
            else:
                self._play(path, secs)
        snd = getattr(self.app, "sound", None)
        if self.tmp is not None:
            if self.why == "spoken" and snd is not None and snd.assistant_interrupted_since(self.hist_n):
                self._why("interrupted", self.last)
            if self.clock() >= self.tmp_until and (snd is None or not snd.assistant_playing()):
                if self._remove(self.tmp):
                    self.tmp = None

    def _play(self, path, secs):
        snd = getattr(self.app, "sound", None)
        asst = getattr(self.app, "asst", None)
        if asst is not None and asst.view.horn_held():
            self._remove(path)
            return self._why("horn", self.last)
        blocked = snd.assistant_blocked() if snd is not None else "sound_off"
        if blocked:
            self._remove(path)
            return self._why(blocked, self.last)
        self.hist_n = len(snd.sched.history)
        r = snd.assistant_speak(path, secs)
        if r != "played":
            self._remove(path)
            return self._why("callout" if r == "busy" else "sound_off", self.last)
        self.tmp, self.tmp_until = path, self.clock() + secs + 0.5
        return self._why("spoken" if self.kind != "long" else "long", self.last)

    def repeat(self):
        """[ΞΑΝΑ]: the last answer again (also one that was text only - unless the horn is held now)."""
        if self.last:
            return self.speak(self.last, user=True)
        return False

    def stop(self, keep_why=True):
        """At once: the voice stops, a TTS result on its way is dropped (a new question, «ΟΜΙΛΙΑ», the station closes)."""
        self.gen += 1
        self.pending = None
        snd = getattr(self.app, "sound", None)
        stopped = bool(snd is not None and snd.assistant_stop())
        if stopped and keep_why:
            self.why = "stopped"
        if self.tmp is not None and not (snd is not None and snd.assistant_playing()):
            if self._remove(self.tmp):
                self.tmp = None
        return stopped

    def close(self):
        self.stop()
        if self.tmp is not None:
            self._remove(self.tmp)
            self.tmp = None

    # --- helpers ---------------------------------------------------------------------------------------------------
    def _why(self, why, info):
        self.why = why
        if info is not None and why != "making":
            qlog.write(info, voice=why)
        return False

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
                if fn.endswith(".wav") and now - os.path.getmtime(p) > STALE_S:
                    self._remove(p)
        except OSError:
            pass
