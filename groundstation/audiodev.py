"""Sound card input / output for the loudhailer (step 13r): the ground station's headset / speakers + microphone
and the gateway's USB sound card (horn amplifier + boat microphone). 8 kHz mono int16, 40 ms frames
(voicecodec.FRAME_N). No library is required:

    "winmm"        Windows: waveIn / waveOut through ctypes
    "alsa"         Linux (the gateway on Ubuntu): arecord / aplay pipes (alsa-utils)
    "sounddevice"  when the Python package is installed (either OS)
    "sim"          simulated speaker + microphone (demo, academy, tests): sea noise + an echo of the speaker
    "null"         nothing (TRYGONS_SOUND=off, tests): output dropped, input silent

Every backend is non-blocking for its caller: the device work runs in its own thread, frames go through small
bounded queues (a full queue drops the oldest), a device that fails reports .error and stops - the caller
shows a notice. Never in a control path.
"""
import array
import collections
import os
import shutil
import subprocess
import threading
import time

import voicecodec as vc

BACKENDS = ("auto", "winmm", "alsa", "sounddevice", "sim", "null")


def auto_backend():
    if os.environ.get("TRYGONS_SOUND") == "off":
        return "null"
    try:
        import sounddevice  # noqa: F401
        return "sounddevice"
    except Exception:
        pass
    if os.name == "nt":
        return "winmm"
    if shutil.which("arecord") and shutil.which("aplay"):
        return "alsa"
    return "null"


class _Base:
    def __init__(self, depth=25):
        self.q = collections.deque(maxlen=depth)
        self.error = ""
        self.ok = True
        self.dropped = 0
        self.frames = 0

    def close(self):
        pass


# --- null + sim -------------------------------------------------------------------------------------------------
class NullOut(_Base):
    def write(self, pcm):
        self.frames += 1

    def pending(self):
        return 0


class NullIn(_Base):
    def read(self):
        return None


class SimSpeaker(_Base):
    """The simulated horn speaker: remembers its level + what it played (tests), feeds the simulated mic's echo."""

    def __init__(self):
        super().__init__()
        self.last_db = -90.0
        self.played = 0
        self.echo = collections.deque(maxlen=50)

    def write(self, pcm):
        self.frames += 1
        self.played += len(pcm)
        self.last_db = vc.level_db(pcm)
        self.echo.append(pcm)

    def pending(self):
        return 0


class SimMic(_Base):
    """The simulated boat microphone: sea / wind noise, the speaker's echo (half duplex must mute it), and now and
    then a 'contact' answering (a voice-like warble) - one frame per read()."""

    def __init__(self, speaker=None, echo=0.5, reply_every_s=0.0, clock=time.time):
        super().__init__()
        self.speaker = speaker
        self.echo_gain = echo
        self.reply_every = reply_every_s
        self.clock = clock
        self.next_reply = clock() + reply_every_s if reply_every_s else None
        self.reply = None
        self.injected = collections.deque(maxlen=500)

    def inject(self, pcm):
        """Tests: the next frames the microphone hears."""
        for f in vc.frames(pcm):
            self.injected.append(f)

    def read(self):
        f = vc.sea_noise()
        if self.injected:
            f = array.array("h", (max(-32767, min(32767, a + b)) for a, b in zip(f, self.injected.popleft())))
        elif self.next_reply is not None and self.clock() >= self.next_reply:
            self.reply = vc.frames(vc.tone(300, 1200, vol=0.25) + vc.tone(380, 800, vol=0.25))
            self.next_reply = self.clock() + self.reply_every
        if self.reply:
            f = array.array("h", (max(-32767, min(32767, a + b)) for a, b in zip(f, self.reply.pop(0))))
        if self.speaker is not None and self.speaker.echo:
            e = self.speaker.echo.popleft()
            f = array.array("h", (max(-32767, min(32767, a + int(b * self.echo_gain))) for a, b in zip(f, e)))
        self.frames += 1
        return f


# --- ALSA (arecord / aplay) -------------------------------------------------------------------------------------
class AlsaOut(_Base):
    def __init__(self, device="default"):
        super().__init__()
        cmd = ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(vc.RATE), "-c", "1", "-B", "200000"]
        if device and device != "default":
            cmd += ["-D", device]
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.cv = threading.Condition()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def write(self, pcm):
        with self.cv:
            if len(self.q) == self.q.maxlen:
                self.dropped += 1
            self.q.append(bytes(pcm))
            self.cv.notify()

    def pending(self):
        return len(self.q)

    def _run(self):
        try:
            while self.p.poll() is None:
                with self.cv:
                    while not self.q:
                        self.cv.wait(0.5)
                        if self.p.poll() is not None:
                            raise OSError("aplay ended")
                    b = self.q.popleft()
                self.p.stdin.write(b)
                self.p.stdin.flush()
                self.frames += 1
            raise OSError("aplay ended (code %s)" % self.p.returncode)
        except (OSError, ValueError) as e:
            self.ok, self.error = False, str(e)[:80]

    def close(self):
        try:
            self.p.kill()
        except OSError:
            pass


class AlsaIn(_Base):
    def __init__(self, device="default"):
        super().__init__()
        cmd = ["arecord", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(vc.RATE), "-c", "1"]
        if device and device != "default":
            cmd += ["-D", device]
        self.p = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        n = vc.FRAME_N * 2
        try:
            while True:
                b = self.p.stdout.read(n)
                if not b or len(b) < n:
                    raise OSError("arecord ended")
                if len(self.q) == self.q.maxlen:
                    self.dropped += 1
                self.q.append(array.array("h", b))
                self.frames += 1
        except (OSError, ValueError) as e:
            self.ok, self.error = False, str(e)[:80]

    def read(self):
        return self.q.popleft() if self.q else None

    def close(self):
        try:
            self.p.kill()
        except OSError:
            pass


# --- sounddevice --------------------------------------------------------------------------------------------------
class SdOut(_Base):
    def __init__(self, device=None):
        super().__init__()
        import sounddevice as sd
        self.s = sd.RawOutputStream(samplerate=vc.RATE, channels=1, dtype="int16", device=device or None,
                                    blocksize=vc.FRAME_N, callback=self._cb)
        self.s.start()

    def _cb(self, out, n, t, status):
        b = self.q.popleft() if self.q else bytes(2 * n)
        out[:] = (b + bytes(2 * n))[:2 * n]

    def write(self, pcm):
        if len(self.q) == self.q.maxlen:
            self.dropped += 1
        self.q.append(bytes(pcm))
        self.frames += 1

    def pending(self):
        return len(self.q)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


class SdIn(_Base):
    def __init__(self, device=None):
        super().__init__()
        import sounddevice as sd
        self.s = sd.RawInputStream(samplerate=vc.RATE, channels=1, dtype="int16", device=device or None,
                                   blocksize=vc.FRAME_N, callback=self._cb)
        self.s.start()

    def _cb(self, data, n, t, status):
        if len(self.q) == self.q.maxlen:
            self.dropped += 1
        self.q.append(array.array("h", bytes(data)))
        self.frames += 1

    def read(self):
        return self.q.popleft() if self.q else None

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


# --- Windows waveIn / waveOut (ctypes) -----------------------------------------------------------------------------
if os.name == "nt":
    import ctypes
    from ctypes import wintypes

    class WAVEFORMATEX(ctypes.Structure):
        _fields_ = [("wFormatTag", wintypes.WORD), ("nChannels", wintypes.WORD), ("nSamplesPerSec", wintypes.DWORD),
                    ("nAvgBytesPerSec", wintypes.DWORD), ("nBlockAlign", wintypes.WORD),
                    ("wBitsPerSample", wintypes.WORD), ("cbSize", wintypes.WORD)]

    class WAVEHDR(ctypes.Structure):
        pass
    WAVEHDR._fields_ = [("lpData", ctypes.c_void_p), ("dwBufferLength", wintypes.DWORD),
                        ("dwBytesRecorded", wintypes.DWORD), ("dwUser", ctypes.c_size_t),
                        ("dwFlags", wintypes.DWORD), ("dwLoops", wintypes.DWORD),
                        ("lpNext", ctypes.POINTER(WAVEHDR)), ("reserved", ctypes.c_size_t)]
    WHDR_DONE = 1
    WAVE_MAPPER = 0xFFFFFFFF

    def _fmt():
        return WAVEFORMATEX(1, 1, vc.RATE, vc.RATE * 2, 2, 16, 0)


class _WinBufs:
    def __init__(self, n):
        self.bufs = [ctypes.create_string_buffer(vc.FRAME_N * 2) for _ in range(n)]
        self.hdrs = [WAVEHDR() for _ in range(n)]
        for b, h in zip(self.bufs, self.hdrs):
            h.lpData = ctypes.cast(b, ctypes.c_void_p)
            h.dwBufferLength = vc.FRAME_N * 2


class WinOut(_Base):
    """waveOut with 6 rotating 40 ms buffers, fed by its own thread (never blocks write())."""

    def __init__(self, device=None):
        super().__init__()
        self.mm = ctypes.windll.winmm
        self.h = wintypes.HANDLE()
        r = self.mm.waveOutOpen(ctypes.byref(self.h), WAVE_MAPPER if device is None else int(device),
                                ctypes.byref(_fmt()), 0, 0, 0)
        if r != 0:
            raise OSError("waveOutOpen failed (%d)" % r)
        self.b = _WinBufs(6)
        self.busy = [False] * 6
        self.cv = threading.Condition()
        self.run = True
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def write(self, pcm):
        with self.cv:
            if len(self.q) == self.q.maxlen:
                self.dropped += 1
            self.q.append(bytes(pcm))
            self.cv.notify()

    def pending(self):
        return len(self.q) + sum(self.busy)

    def _run(self):
        sz = ctypes.sizeof(WAVEHDR)
        try:
            while self.run:
                for i, h in enumerate(self.b.hdrs):
                    if self.busy[i] and h.dwFlags & WHDR_DONE:
                        self.mm.waveOutUnprepareHeader(self.h, ctypes.byref(h), sz)
                        self.busy[i] = False
                free = [i for i in range(6) if not self.busy[i]]
                with self.cv:
                    if not self.q or not free:
                        self.cv.wait(0.01)
                        continue
                    data = self.q.popleft()
                i = free[0]
                h = self.b.hdrs[i]
                ctypes.memmove(self.b.bufs[i], data, min(len(data), vc.FRAME_N * 2))
                h.dwFlags = 0
                h.dwBufferLength = min(len(data), vc.FRAME_N * 2)
                if self.mm.waveOutPrepareHeader(self.h, ctypes.byref(h), sz) or \
                        self.mm.waveOutWrite(self.h, ctypes.byref(h), sz):
                    raise OSError("waveOutWrite failed")
                self.busy[i] = True
                self.frames += 1
        except OSError as e:
            self.ok, self.error = False, str(e)[:80]

    def close(self):
        self.run = False
        try:
            self.th.join(1.0)
            self.mm.waveOutReset(self.h)
            for i, h in enumerate(self.b.hdrs):
                if self.busy[i]:
                    self.mm.waveOutUnprepareHeader(self.h, ctypes.byref(h), ctypes.sizeof(WAVEHDR))
            self.mm.waveOutClose(self.h)
        except Exception:
            pass


class WinIn(_Base):
    """waveIn with 8 rotating 40 ms buffers, polled by its own thread."""

    def __init__(self, device=None):
        super().__init__()
        self.mm = ctypes.windll.winmm
        self.h = wintypes.HANDLE()
        r = self.mm.waveInOpen(ctypes.byref(self.h), WAVE_MAPPER if device is None else int(device),
                               ctypes.byref(_fmt()), 0, 0, 0)
        if r != 0:
            raise OSError("waveInOpen failed (%d): no microphone?" % r)
        self.b = _WinBufs(8)
        sz = ctypes.sizeof(WAVEHDR)
        for h in self.b.hdrs:
            self.mm.waveInPrepareHeader(self.h, ctypes.byref(h), sz)
            self.mm.waveInAddBuffer(self.h, ctypes.byref(h), sz)
        self.mm.waveInStart(self.h)
        self.run = True
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        sz = ctypes.sizeof(WAVEHDR)
        try:
            while self.run:
                got = False
                for i, h in enumerate(self.b.hdrs):
                    if h.dwFlags & WHDR_DONE:
                        n = h.dwBytesRecorded
                        if n:
                            if len(self.q) == self.q.maxlen:
                                self.dropped += 1
                            self.q.append(array.array("h", self.b.bufs[i].raw[:n - n % 2]))
                            self.frames += 1
                        h.dwFlags &= ~WHDR_DONE
                        self.mm.waveInUnprepareHeader(self.h, ctypes.byref(h), sz)
                        self.mm.waveInPrepareHeader(self.h, ctypes.byref(h), sz)
                        if self.mm.waveInAddBuffer(self.h, ctypes.byref(h), sz):
                            raise OSError("waveInAddBuffer failed")
                        got = True
                if not got:
                    time.sleep(0.01)
        except OSError as e:
            self.ok, self.error = False, str(e)[:80]

    def read(self):
        return self.q.popleft() if self.q else None

    def close(self):
        self.run = False
        try:
            self.th.join(1.0)
            self.mm.waveInStop(self.h)
            self.mm.waveInReset(self.h)
            for h in self.b.hdrs:
                self.mm.waveInUnprepareHeader(self.h, ctypes.byref(h), ctypes.sizeof(WAVEHDR))
            self.mm.waveInClose(self.h)
        except Exception:
            pass


# --- open --------------------------------------------------------------------------------------------------------
def open_output(backend="auto", device=None):
    """-> (output, error text). A failing device = a null output + the reason (shown as a notice)."""
    b = auto_backend() if backend in (None, "auto") else backend
    try:
        if b == "winmm":
            return WinOut(device), ""
        if b == "alsa":
            return AlsaOut(device or "default"), ""
        if b == "sounddevice":
            return SdOut(device), ""
        if b == "sim":
            return SimSpeaker(), ""
        if b == "null":
            return NullOut(), ""
        return NullOut(), "unknown sound backend %r" % b
    except Exception as e:
        return NullOut(), "no speaker (%s): %s" % (b, str(e)[:80])


def open_input(backend="auto", device=None, speaker=None, reply_every_s=0.0):
    b = auto_backend() if backend in (None, "auto") else backend
    try:
        if b == "winmm":
            return WinIn(device), ""
        if b == "alsa":
            return AlsaIn(device or "default"), ""
        if b == "sounddevice":
            return SdIn(device), ""
        if b == "sim":
            return SimMic(speaker if isinstance(speaker, SimSpeaker) else None, reply_every_s=reply_every_s), ""
        if b == "null":
            return NullIn(), ""
        return NullIn(), "unknown sound backend %r" % b
    except Exception as e:
        return NullIn(), "no microphone (%s): %s" % (b, str(e)[:80])
