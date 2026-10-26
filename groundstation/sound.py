"""Cockpit-style sounds and voice callouts (section D). DISPLAY ONLY: nothing here
sends anything to the boat; the boat's own buzzer keeps sounding its alarms
through the existing alarm path (D5).

Levels (alerts.py):
  WARNING   repeating urgent tone + voice until acknowledged (MASTER lamp / Space)
  CAUTION   single chime + voice once
  ADVISORY  soft click or a short voice
  FEEDBACK  small sounds for own actions (SAVE shutter, takeover chirp, confirm / refuse)
Rules (Scheduler): a higher level interrupts a lower one; voices never overlap
(one clip at a time, the rest waits); the same callout is not repeated within
repeat_s (settings); advisories and feedback can be muted, WARNINGS never (and
never below WARN_MIN_VOLUME); quiet / night mode = softer, fewer advisories.

Voice: every phrase is a "say.*" entry of the language files (voice language may
differ from the screen language); made once by the offline TTS (tts.py) and kept
in sound_cache/ so a callout starts at once. A recorded file
sounds/<lang>/<key>.wav replaces the TTS for that phrase (later: real voices).
"""
import array
import hashlib
import math
import os
import random
import threading
import time
import wave

import alerts
import i18n
import settings
import tts

HERE = os.path.dirname(os.path.abspath(__file__))
RATE = 22050
WARN_MIN_VOLUME = 60                    # % - warnings are never quieter than this
WARNING, CAUTION, ADVISORY, FEEDBACK = alerts.WARNING, alerts.CAUTION, alerts.ADVISORY, alerts.FEEDBACK
QUEUE_MAX_AGE = {ADVISORY: 6.0, CAUTION: 20.0, FEEDBACK: 1.5, WARNING: 60.0}
WARN_GAP_S = 3.0                        # pause between two repeats of a warning

try:
    import winsound
except ImportError:
    winsound = None


# --- tones (made here, no sound files needed) ----------------------------------------------------------
def _env(i, n, a=0.01, r=0.04):
    t = i / RATE
    d = n / RATE
    return min(1.0, t / a if a else 1.0, (d - t) / r if r else 1.0)


def _note(freq, ms, amp=0.6, shape="sine", f2=None, a=0.008, r=0.03):
    n = int(RATE * ms / 1000.0)
    out = array.array("h", [0]) * n
    ph = 0.0
    for i in range(n):
        f = freq if f2 is None else freq + (f2 - freq) * i / max(1, n - 1)
        ph += 2 * math.pi * f / RATE
        s = math.sin(ph)
        if shape == "square":
            s = 0.6 if s > 0 else -0.6
        elif shape == "bell":
            s = (math.sin(ph) + 0.35 * math.sin(2 * ph) + 0.15 * math.sin(3 * ph)) * math.exp(-3.0 * i / n)
        out[i] = int(32767 * amp * s * max(0.0, _env(i, n, a, r)))
    return out


def _silence(ms):
    return array.array("h", [0]) * int(RATE * ms / 1000.0)


def _noise(ms, amp=0.5):
    n = int(RATE * ms / 1000.0)
    rnd = random.Random(7)
    return array.array("h", (int(32767 * amp * (rnd.random() * 2 - 1) * max(0.0, _env(i, n, 0.002, 0.01)))
                             for i in range(n)))


def concat(parts, gap_ms=0):
    out = array.array("h")
    for k, p in enumerate(parts):
        if k and gap_ms:
            out += _silence(gap_ms)
        out += p
    return out


TONE_NAMES = ("warning", "caution", "advisory", "cavalry", "shutter", "chirp2", "confirm", "refuse", "question",
              "rising")


def tone(name, latency_ms=300):
    """The samples of a tone. warning = urgent two-tone 'whoop' x3, caution = chime, advisory = soft click,
    cavalry = the autopilot-disconnect charge, rising = remote-driving latency (higher = later)."""
    if name == "warning":
        return concat([concat([_note(760, 150, 0.7, "square"), _note(1180, 150, 0.7, "square")]) for _ in range(3)])
    if name == "caution":
        return concat([_note(1047, 300, 0.7, "bell", r=0.12), _note(784, 380, 0.6, "bell", r=0.2)])
    if name == "advisory":
        return _note(1500, 45, 0.35, a=0.003, r=0.02)
    if name == "cavalry":                    # G4 C5 E5 G5 - E5 G5: the famous charge
        seq = [(392, 110), (523, 110), (659, 110), (784, 220), (659, 110), (784, 380)]
        return concat([_note(f, ms, 0.6, "square", a=0.004, r=0.02) for f, ms in seq], gap_ms=25)
    if name == "shutter":
        return concat([_noise(18, 0.6), _silence(45), _noise(26, 0.45)])
    if name == "chirp2":
        return concat([_note(1100, 70, 0.5, f2=2300), _note(1100, 70, 0.5, f2=2300)], gap_ms=60)
    if name == "confirm":
        return concat([_note(880, 70, 0.4), _note(1320, 110, 0.4)], gap_ms=20)
    if name == "refuse":
        return _note(220, 260, 0.5, "square")
    if name == "question":
        return concat([_note(660, 90, 0.45, "bell"), _note(880, 140, 0.45, "bell")], gap_ms=30)
    if name == "rising":
        f0 = 600
        return _note(f0, 260, 0.45, f2=f0 + min(1400, latency_ms * 2))
    return _silence(10)


# --- wav files --------------------------------------------------------------------------------------------
def write_wav(path, samples):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with wave.open(tmp, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(samples.tobytes())
    os.replace(tmp, path)


def read_wav(path):
    """Any 8 / 16-bit mono / stereo WAV -> 16-bit mono samples at RATE."""
    with wave.open(path, "rb") as w:
        ch, sw, fr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if sw == 2:
        a = array.array("h")
        a.frombytes(raw)
    elif sw == 1:
        a = array.array("h", ((b - 128) << 8 for b in raw))
    else:
        raise ValueError("%d-byte samples" % sw)
    if ch > 1:
        a = array.array("h", (sum(a[i:i + ch]) // ch for i in range(0, len(a), ch)))
    if fr != RATE and len(a) > 1:                   # linear resampling
        m = int(len(a) * RATE / fr)
        k = (len(a) - 1) / max(1, m - 1)
        out = array.array("h", [0]) * m
        for i in range(m):
            x = i * k
            j = int(x)
            f = x - j
            out[i] = int(a[j] * (1 - f) + a[min(j + 1, len(a) - 1)] * f)
        a = out
    return a


def gain(samples, vol):
    if abs(vol - 1.0) < 1e-3:
        return samples
    return array.array("h", (max(-32767, min(32767, int(s * vol))) for s in samples))


# --- the phrase / clip library ----------------------------------------------------------------------------
class Library:
    """Tones, cached voice phrases and the clips played (tone + voice joined, at the right volume)."""

    def __init__(self, cache=None, overrides=None):
        self.cache = cache or os.environ.get("TRYGONS_SOUND_CACHE") or os.path.join(HERE, "sound_cache")
        self.overrides = overrides or os.path.join(HERE, "sounds")
        self.lock = threading.Lock()
        self.busy = {}                          # lang -> "making the voice cache" message
        self.made = {}                          # lang -> phrases in the cache

    # A callout is spoken as ONE sentence ("Traffic, two o'clock, eight hundred metres."): neural voices
    # sound right on sentences, some garble single short words (the Greek one does).
    FRAGMENTS = ("say.clock_", "say.dist_", "say.digit_")
    ONLY_IN_SENTENCES = ("say.traffic", "say.nav.turning")

    def own_file(self, lang, key):
        p = os.path.join(self.overrides, lang, key + ".wav")
        return p if os.path.exists(p) else None

    @staticmethod
    def text_of(lang, keys):
        parts = [i18n.TL(lang, k).strip() for k in keys]
        if not parts or any(not t or t.startswith("[") for t in parts):
            return None
        return ", ".join(t.rstrip(".") for t in parts) + "."

    def sentence_path(self, lang, text):
        h = hashlib.sha1(text.encode("utf-8")).hexdigest()[:20]
        return os.path.join(self.cache, lang, "p_%s.wav" % h)

    @staticmethod
    def suspect(text, path):
        """A sentence far longer than its text: the neural voice garbled it (happens on short phrases)."""
        try:
            with wave.open(path, "rb") as w:
                secs = w.getnframes() / float(w.getframerate())
        except (OSError, EOFError, wave.Error):
            return True
        return secs > 0.9 + 0.11 * len(text)

    def make(self, lang, texts, retries=3):
        """Make the missing sentences (one TTS run); a garbled one is made again (the voice is a little
        different every time) and the shortest take is kept."""
        need = [(t, self.sentence_path(lang, t)) for t in texts if t]
        need = [(t, p) for t, p in need if not os.path.exists(p)]
        if not need:
            return
        with self.lock:
            need = [(t, p) for t, p in need if not os.path.exists(p)]
            if not need:
                return
            tts.synth(lang, need)
            for _ in range(retries):
                again = [(t, p) for t, p in need if os.path.exists(p) and self.suspect(t, p)]
                if not again:
                    break
                tmp = [(t, p + ".retry.wav") for t, p in again]
                tts.synth(lang, tmp)
                for (t, p), (_t, q) in zip(again, tmp):
                    try:
                        with wave.open(p, "rb") as a, wave.open(q, "rb") as b:
                            better = b.getnframes() / float(b.getframerate()) < a.getnframes() / float(a.getframerate())
                        if better:
                            os.replace(q, p)
                    except (OSError, EOFError, wave.Error):
                        pass
                    if os.path.exists(q):
                        os.remove(q)

    def voice(self, lang, keys, block=True):
        """The spoken part as samples (None = no voice): the recorded files when every phrase has one,
        else the sentence from the TTS cache. block=False never waits for the TTS: a missing sentence is
        made in the background and this call gives None (the callout plays its tone only, the screen is
        never held up)."""
        if not keys:
            return None
        own = [self.own_file(lang, k) for k in keys]
        try:
            if all(own):
                return concat([read_wav(p) for p in own], gap_ms=70)
            text = self.text_of(lang, keys)
            if not text:
                return None
            p = self.sentence_path(lang, text)
            if not os.path.exists(p):
                if not block:
                    threading.Thread(target=self.make, args=(lang, [text]), daemon=True).start()
                    return None
                self.make(lang, [text])
            return read_wav(p) if os.path.exists(p) else None
        except (OSError, ValueError, EOFError, wave.Error):
            return None

    def all_sentences(self, lang):
        """Every sentence the callouts can say: the fixed ones, traffic (clock x distance), "turning to" every 5 deg."""
        keys = [k for k in i18n.load(lang) if k.startswith("say.") and not k.startswith(self.FRAGMENTS)
                and k not in self.ONLY_IN_SENTENCES]
        lists = [[k] for k in keys]
        dists = ["say.dist_m_%d" % m for m in DIST_M] + ["say.dist_nm_0%d" % t for t in DIST_NM10] + \
            ["say.dist_nm_%d" % n for n in DIST_NM] + ["say.dist_far"]
        lists += [["say.traffic", "say.clock_%d" % c, dk] for c in CLOCKS for dk in dists]
        lists += [["say.nav.turning"] + heading_keys(h) for h in range(0, 360, 5)]
        return [t for t in (self.text_of(lang, ks) for ks in lists) if t]

    def pregenerate(self, lang, done=None):
        """Every voice sentence of the language, in the background (once; then cached on disk)."""
        def run():
            self.busy[lang] = True
            try:
                texts = self.all_sentences(lang)
                self.make(lang, texts)
                self.made[lang] = sum(1 for t in texts if os.path.exists(self.sentence_path(lang, t)))
            finally:
                self.busy[lang] = False
                if done:
                    done(lang)
        threading.Thread(target=run, daemon=True).start()

    def clip(self, tone_name, keys, lang, vol, latency_ms=300, block=True):
        """tone + the voice sentence -> (wav path, seconds). No voice (yet) = tone only."""
        text = self.text_of(lang, keys) or ""
        tag = hashlib.sha1(("%s|%s|%s|%.2f|%d|%s" % (tone_name, ",".join(keys), lang, vol, latency_ms, text)
                            ).encode("utf-8")).hexdigest()[:20]
        path = os.path.join(self.cache, "clips", tag + ".wav")
        if not os.path.exists(path):
            parts = []
            if tone_name:
                parts.append(tone(tone_name, latency_ms))
            v = self.voice(lang, keys, block)
            if v is not None:
                if parts:
                    parts.append(_silence(120))
                parts.append(v)
            if v is None and keys:                  # not cached yet: tone only now, the voiced clip next time
                path = os.path.join(self.cache, "clips", "novoice_%s.wav" % tag)
            write_wav(path, gain(concat(parts) if parts else _silence(50), vol))
        with wave.open(path, "rb") as w:
            secs = w.getnframes() / float(w.getframerate())
        return path, secs


# --- players ------------------------------------------------------------------------------------------------
class WinPlayer:
    """One clip at a time (Windows winsound, asynchronous): a new clip stops the old one."""

    def play(self, path):
        try:
            winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
        except RuntimeError:
            pass

    def stop(self):
        try:
            winsound.PlaySound(None, 0)
        except RuntimeError:
            pass


class NullPlayer:
    """No sound device / not Windows / the tests: only remembers what would have played."""

    def __init__(self):
        self.played = []

    def play(self, path):
        self.played.append(path)

    def stop(self):
        self.played.append(None)


# --- the rules ------------------------------------------------------------------------------------------------
class Item:
    def __init__(self, code, level, tone_name, keys, keep_quiet=False, latency_ms=300, t=0.0):
        self.code, self.level, self.tone, self.keys = code, level, tone_name, list(keys)
        self.keep_quiet, self.latency_ms, self.t = keep_quiet, latency_ms, t


class Scheduler:
    """Who plays when. build(item, volume) -> (path, seconds); clock() -> now. Pure logic (test_sound.py)."""

    def __init__(self, build, player, clock=time.time):
        self.build, self.player, self.clock = build, player, clock
        self.current = None                     # (Item, end time)
        self.queue = []
        self.last = {}                          # code -> when it last started
        self.active = {}                        # warning code -> next time due
        self.acked = set()
        self.history = []                       # (t, code, level, what) for the tests
        self.repeat_s = 20.0
        self.volume = 80
        self.mute_advisory = self.mute_feedback = self.quiet = False
        self.enabled = True

    def vol(self, level):
        v = self.volume * (0.5 if self.quiet else 1.0)
        if level == WARNING:
            v = max(v, WARN_MIN_VOLUME)          # warnings: never muted, never too quiet
        return max(0.0, min(100.0, v)) / 100.0

    def request(self, item):
        """A callout / sound wants to play -> 'played' / 'queued' / 'muted' / 'quiet' / 'repeat' / 'off'."""
        now = self.clock()
        item.t = now
        lv = item.level
        if not self.enabled and lv != WARNING:
            return self._note(item, "off")
        if lv == FEEDBACK and self.mute_feedback or lv == ADVISORY and self.mute_advisory:
            return self._note(item, "muted")
        if self.quiet and lv == ADVISORY and not item.keep_quiet:
            return self._note(item, "quiet")
        if lv not in (FEEDBACK, WARNING) and now - self.last.get(item.code, -1e9) < self.repeat_s:
            return self._note(item, "repeat")
        if self.current and self.current[1] > now:
            if self.current[0].level >= lv:
                if not any(q.code == item.code for q in self.queue):
                    self.queue.append(item)
                return self._note(item, "queued")
        self._start(item, now, interrupt=bool(self.current and self.current[1] > now))
        return "played"

    def _note(self, item, what):
        self.history.append((self.clock(), item.code, item.level, what))
        return what

    def _start(self, item, now, interrupt=False):
        if interrupt:
            self.history.append((now, self.current[0].code, self.current[0].level, "interrupted"))
            self.player.stop()
        path, secs = self.build(item, self.vol(item.level))
        self.player.play(path)
        self.current = (item, now + secs)
        self.last[item.code] = now
        self.history.append((now, item.code, item.level, "played"))

    def set_warnings(self, codes):
        """The warnings that are active and not acknowledged now: each repeats until acknowledged / gone."""
        now = self.clock()
        codes = set(codes)
        for c in list(self.active):
            if c not in codes:
                del self.active[c]
        self.acked &= codes                       # a warning that went away may come back later
        for c in codes:
            if c not in self.active and c not in self.acked:
                self.active[c] = now

    def ack(self):
        """MASTER lamp acknowledged: the repeating warnings stop (until a new one)."""
        self.acked |= set(self.active)
        self.active.clear()
        if self.current and self.current[0].level == WARNING and self.current[1] > self.clock():
            self.player.stop()
            self.history.append((self.clock(), self.current[0].code, WARNING, "silenced"))
            self.current = None

    def tick(self, warn_item):
        """Every ~100 ms. warn_item(code) -> the Item for a repeating warning."""
        now = self.clock()
        if self.current and now >= self.current[1]:
            self.current = None
        busy_lv = self.current[0].level if self.current else -1
        due = sorted((t, c) for c, t in self.active.items() if t <= now)
        if due and busy_lv < WARNING:
            code = due[0][1]
            it = warn_item(code)
            self._start(it, now, interrupt=self.current is not None)
            self.active[code] = self.current[1] + WARN_GAP_S + 0.5 * len(self.active)
            return
        if self.current is None and self.queue:
            self.queue = [q for q in self.queue if now - q.t <= QUEUE_MAX_AGE.get(q.level, 10.0)]
            if self.queue:
                self.queue.sort(key=lambda q: (-q.level, q.t))
                self._start(self.queue.pop(0), now)


# --- the callouts ---------------------------------------------------------------------------------------------
# code: (level, tone, voice keys, keep in quiet mode)
CALLOUTS = {
    "mode.hdg_lock": (ADVISORY, "advisory", ["say.mode.hdg_lock"], True),
    "mode.auto": (ADVISORY, "advisory", ["say.mode.auto"], True),
    "mode.standby": (ADVISORY, "advisory", ["say.mode.standby"], True),
    "mode.manual": (ADVISORY, "chirp2", ["say.mode.manual"], True),
    "mode.remote_on": (CAUTION, "caution", ["say.mode.remote_on"], True),
    "mode.remote_off": (ADVISORY, "advisory", ["say.mode.remote_off"], True),
    "ap.disconnect": (CAUTION, "cavalry", ["say.ap.disconnect"], True),
    "traffic.slowing": (CAUTION, "caution", ["say.traffic.slowing"], True),
    "traffic.avoid_stbd": (CAUTION, "caution", ["say.traffic.avoid_stbd"], True),
    "traffic.avoid_port": (CAUTION, "caution", ["say.traffic.avoid_port"], True),
    "depth.10": (CAUTION, "caution", ["say.depth.10"], True),
    "depth.5": (CAUTION, "caution", ["say.depth.5"], True),
    "depth.3": (CAUTION, "caution", ["say.depth.3"], True),
    "nav.wp30": (ADVISORY, "advisory", ["say.nav.wp30"], False),
    "link.lost": (CAUTION, "caution", ["say.link.lost"], True),
    "link.restored": (ADVISORY, "advisory", ["say.link.restored"], True),
    "link.starlink": (CAUTION, "caution", ["say.link.starlink"], True),
    "link.4g": (ADVISORY, "advisory", ["say.link.4g"], False),
    "link.rf": (ADVISORY, "advisory", ["say.link.rf"], False),
    "link.channel": (ADVISORY, "advisory", ["say.link.channel"], False),
    "link.latency": (CAUTION, "rising", [], True),
    "power.limit": (ADVISORY, "advisory", ["say.power.limit"], False),
    "sea.rough": (CAUTION, "caution", ["say.sea.rough"], True),
    "fb.shutter": (FEEDBACK, "shutter", [], True),
    "fb.chirp2": (FEEDBACK, "chirp2", [], True),
    "fb.confirm": (FEEDBACK, "confirm", [], True),
    "fb.refuse": (FEEDBACK, "refuse", [], True),
    "fb.question": (FEEDBACK, "question", [], True),
}
LEVEL_TONE = {WARNING: "warning", CAUTION: "caution", ADVISORY: "advisory", FEEDBACK: None}
POWER_STEPS = {"fwd": (30, 50, 75, 100), "rev": (5, 10, 15)}
CLOCKS = range(1, 13)
DIST_M = (100, 200, 300, 400, 500, 600, 700, 800, 900)
DIST_NM10 = (5, 6, 7, 8, 9)
DIST_NM = (1, 2, 3, 4, 5)


def dist_key(m):
    """A distance -> its voice phrase key (metres to 900 m, then tenths, then whole miles)."""
    if m < 950:
        return "say.dist_m_%d" % max(100, min(900, int(round(m / 100.0)) * 100))
    nm = m / 1852.0
    if nm < 0.95:
        return "say.dist_nm_0%d" % max(5, min(9, int(round(nm * 10))))
    if nm < 5.5:
        return "say.dist_nm_%d" % int(round(nm))
    return "say.dist_far"


def heading_keys(hdg):
    """245 -> the digits 'two four five' (as on the radio)."""
    return ["say.digit_%s" % c for c in "%03d" % (int(round(hdg)) % 360)]


def dynamic_keys():
    """Voice keys built at run time (test_lang.py)."""
    k = ["say.alarm." + c for c, _l in alerts.ALARM_LEVELS.values()] + ["say.alarm." + alerts.NO_DATA]
    k += ["say.clock_%d" % c for c in CLOCKS] + ["say.dist_m_%d" % m for m in DIST_M]
    k += ["say.dist_nm_0%d" % t for t in DIST_NM10] + ["say.dist_nm_%d" % n for n in DIST_NM] + ["say.dist_far"]
    k += ["say.digit_%d" % i for i in range(10)]
    k += ["say.power_%s_%d" % (d, n) for d, steps in POWER_STEPS.items() for n in steps]
    k += ["snd.level_%d" % lv for lv in (WARNING, CAUTION, ADVISORY, FEEDBACK)] + ["snd.tone_" + t for t in TONE_NAMES]
    return k


class SoundSystem:
    """The ground station's sound: settings + library + scheduler + player."""

    def __init__(self, player=None, clock=time.time, library=None):
        self.lib = library or Library()
        silent = os.environ.get("TRYGONS_SOUND") == "off"              # the tests: no sound card, no TTS runs
        self.player = player or (WinPlayer() if winsound and not silent else NullPlayer())
        self.sched = Scheduler(self.build, self.player, clock)
        self.voice_lang = settings.get("voice_lang") if settings.get("voice_lang") in i18n.LANGS else "el"
        self.apply_settings()

    @classmethod
    def for_app(cls, app):
        s = cls()
        s.app = app
        if os.environ.get("TRYGONS_SOUND") != "off":
            s.lib.pregenerate(s.voice_lang)
        app.root.after(100, s._tick)
        return s

    app = None

    def apply_settings(self):
        sc = self.sched
        sc.volume = float(settings.get("volume", 80))
        sc.mute_advisory = bool(settings.get("mute_advisory"))
        sc.mute_feedback = bool(settings.get("mute_feedback"))
        sc.quiet = bool(settings.get("quiet")) or bool(settings.get("night"))
        sc.repeat_s = float(settings.get("repeat_s", 20.0))
        sc.enabled = bool(settings.get("sound", True))

    def enabled(self):
        return self.sched.enabled and not isinstance(self.player, NullPlayer)

    def set_quiet(self, on):
        self.sched.quiet = bool(on) or bool(settings.get("quiet"))

    def set_voice_lang(self, lang):
        if lang in i18n.LANGS:
            self.voice_lang = lang
            settings.put("voice_lang", lang)
            self.lib.pregenerate(lang)

    def build(self, item, vol):
        return self.lib.clip(item.tone, item.keys, self.voice_lang, vol, item.latency_ms, block=False)

    def say(self, code, keys=None, level=None, tone_name=None, keep_quiet=None, latency_ms=300):
        """A callout by its code (CALLOUTS) or an alarm code ("alarm.<code>")."""
        if code in CALLOUTS:
            lv, tn, ks, kq = CALLOUTS[code]
        else:
            lv, tn, ks, kq = (level if level is not None else CAUTION), None, [], True
        lv = lv if level is None else level
        tn = tone_name or tn or LEVEL_TONE.get(lv)
        return self.sched.request(Item(code, lv, tn, keys if keys is not None else ks,
                                       kq if keep_quiet is None else keep_quiet, latency_ms))

    def feedback(self, code):
        return self.say(code)

    def alarm(self, code, level):
        return self.say("alarm." + code, keys=["say.alarm." + code], level=level, tone_name=LEVEL_TONE.get(level))

    def warn_item(self, code):
        return Item("alarm." + code, WARNING, "warning", ["say.alarm." + code], True)

    def ack(self):
        self.sched.ack()

    def _tick(self):
        try:
            if self.app is not None:
                self.sched.set_warnings(self.app.lamps.unacked_codes(WARNING) if self.app.t is not None or
                                        self.app.lamps.no_data else [])
            self.sched.tick(self.warn_item)
        finally:
            if self.app is not None:
                try:
                    self.app.root.after(100, self._tick)
                except Exception:
                    pass


class Callouts:
    """Works out WHEN to speak from the telemetry and the gateway status (only on changes)."""

    def __init__(self, sound, app, clock=time.time):
        self.s, self.app, self.clock = sound, app, clock
        self.depth_said = None
        self.depth_t = 0.0
        self.wp30_done = set()
        self.last_wp = None
        self.threat_lv = {}
        self.channel = None
        self.avoid_state = None
        self.prop_id = None
        self.push_since = None
        self.lat_t = 0.0
        self.alarm_t = {}                        # alarm bit -> when it appeared (autopilot disconnect)
        self.no_data = False

    def telem(self, prev, d):
        s, now = self.s, self.clock()
        if prev is None:
            return
        # alarms (new bits). WARNINGS repeat through the MASTER lamps (SoundSystem._tick)
        new = alerts.bits_of(d) & ~alerts.bits_of(prev)          # both alarm words
        for bit, code, lv in alerts.active(new):
            self.alarm_t[bit] = now
            if lv < WARNING and code != "joystick_took_control":
                s.alarm(code, lv)
        # modes
        pm, m = prev["mode"], d["mode"]
        if pm != m:
            if m == 2:
                s.say("mode.hdg_lock")
            elif m == 3:
                s.say("mode.auto")
            elif m == 1:
                if pm in (2, 3, 5):
                    s.feedback("fb.chirp2")              # the joystick took over
                s.say("mode.manual")
            elif m == 5:
                s.say("mode.remote_on")
            elif m == 0:
                fault = any(now - t <= 2.0 and alerts.level(b) >= CAUTION for b, t in self.alarm_t.items())
                if pm in (2, 3, 5) and fault:
                    s.say("ap.disconnect")              # dropped out by a fault: the cavalry charge
                else:
                    s.say("mode.standby")
            if pm == 5 and m != 5:
                s.say("mode.remote_off")
        # power limit raised (confirmed on the boat)
        for kind, field in (("fwd", "power_fwd"), ("rev", "power_rev")):
            if d[field] > prev[field] + 0.5:
                n = int(round(d[field]))
                key = "say.power_%s_%d" % (kind, n)
                s.say("power.%s.%d" % (kind, n), keys=[key] if key in i18n.load(s.voice_lang) else [], level=ADVISORY,
                      tone_name="advisory", keep_quiet=True)
        # pushing against the power limit (throttle at the limit, more asked) for 1.5 s
        pushing = m in (1, 2, 5) and d["power_fwd"] < 99.5 and d["throttle"] >= d["power_fwd"] - 0.6 and \
            d.get("throttle_demand", 0) > d["throttle"] + 2.0
        if pushing:
            self.push_since = self.push_since or now
            if now - self.push_since >= 1.5:
                s.say("power.limit")
        else:
            self.push_since = None
        # rough water: slamming red / a new slamming throttle step
        if d.get("slam_state") == "RED" and prev.get("slam_state") != "RED" or \
                (d.get("slam_cap", 100) < 100 and d.get("slam_cap", 100) < prev.get("slam_cap", 100)):
            s.say("sea.rough")
        # navigation (AUTO): new leg -> "turning to ...", 30 s to the waypoint, arrived (alarm bit, above)
        if m == 3 and d.get("wp_set") and d.get("pos_source"):
            wpkey = (d["auto_wp"], round(d["wp_lat"], 5), round(d["wp_lon"], 5))
            import hud
            dist, brg = hud.dist_bearing(d["est_lat"], d["est_lon"], d["wp_lat"], d["wp_lon"])
            if self.last_wp is not None and wpkey != self.last_wp:
                # spoken to the nearest 5 deg: every such sentence is made in advance (starts at once)
                s.say("nav.turning.%d" % d["auto_wp"], keys=["say.nav.turning"] + heading_keys(round(brg / 5.0) * 5),
                      level=ADVISORY, tone_name="advisory", keep_quiet=False)
            self.last_wp = wpkey
            v = d["est_sog"] * 0.514444
            if v > 0.5 and dist / v <= 30.0 and wpkey not in self.wp30_done:
                self.wp30_done.add(wpkey)
                s.say("nav.wp30")
        # depth (chart) getting shallower: 10, 5, 3 m
        if d.get("pos_source") and self.app.charts and now - self.depth_t >= 1.0:
            self.depth_t = now
            dep = self.app.charts.depth(d["est_lat"], d["est_lon"])
            if dep is not None:
                for lim in (3, 5, 10):
                    if dep < lim and (self.depth_said is None or self.depth_said > lim):
                        self.depth_said = lim
                        s.say("depth.%d" % lim)
                        break
                if self.depth_said is not None and dep > self.depth_said * 1.3:
                    self.depth_said = None if dep > 12 else min(x for x in (3, 5, 10) if x * 1.3 >= dep)
        # remote driving: a rising tone when the latency gets high (every 2 s)
        if m == 5 and d.get("rjoy_lat_ms", 0) > 250 and now - self.lat_t >= 2.0:
            self.lat_t = now
            s.say("link.latency.%d" % (now // 2), level=CAUTION, tone_name="rising", keys=[],
                  latency_ms=d["rjoy_lat_ms"])

    def link(self, connected_with_data):
        """Data from the boat lost / back (the WARNING itself is the MASTER lamp's)."""
        if not connected_with_data and not self.no_data:
            self.no_data = True
        elif connected_with_data and self.no_data:
            self.no_data = False
            self.s.say("link.restored")

    def gw(self, g):
        s = self.s
        n = (g or {}).get("net") or {}
        ch = ("EMERGENCY" if n.get("emergency") else n.get("active")) if n else None
        if n and ch != self.channel:
            if self.channel is not None:
                if ch == "EMERGENCY":
                    s.say("link.starlink")
                elif ch is None:
                    s.say("link.lost")
                elif ch.upper().startswith("4G"):
                    s.say("link.4g")
                elif ch.upper().startswith("RF"):
                    s.say("link.rf")
                else:
                    s.say("link.channel")
            self.channel = ch
        av = (g or {}).get("avoid") or {}
        d = self.app.t or {}
        rank = {"warn": 1, "danger": 2}
        seen = {}
        for t in av.get("threats") or []:
            lv = rank.get(t.get("level"), 0)
            seen[t["mmsi"]] = lv
            if lv > self.threat_lv.get(t["mmsi"], 0) and t.get("bearing") is not None:
                rel = (t["bearing"] - d.get("heading", 0.0)) % 360.0
                import hud
                keys = ["say.traffic", "say.clock_%d" % hud.clock_position(rel),
                        dist_key((t.get("dist_nm") or 0.0) * 1852.0)]
                s.say("traffic.%s.%d" % (t["mmsi"], lv), keys=keys, level=CAUTION, tone_name="caution",
                      keep_quiet=True)
        self.threat_lv = seen
        st = av.get("state")
        if st != self.avoid_state:
            if st in ("SLOW", "AVOID") and d.get("mode") == 3:
                s.say("traffic.slowing")
            self.avoid_state = st
        p = av.get("proposal")
        if p and p.get("id") != self.prop_id and p.get("heading") is not None:
            turn = (p["heading"] - d.get("heading", 0.0) + 540.0) % 360.0 - 180.0
            s.say("traffic.avoid_stbd" if turn >= 0 else "traffic.avoid_port")
        self.prop_id = p.get("id") if p else None
