"""The talking manual's OWN microphone (step AM2): 16 kHz mono int16 for Whisper, separate from the loudhailer's
devices (audiodev.py, 8 kHz). Audio stays in RAM: nothing here writes a file (CLAUDE.md W).

    open_input(backend, device)   backend "winmm" (Windows, ctypes), "sounddevice" (if installed), "sim" (tests:
                                  inject() samples), "null" (no microphone); device None = the default input
    list_inputs()                 [(index, name)] of this PC's microphones (Windows waveIn)

Every input runs its own thread and keeps at most MAX_S seconds; read_all() returns what was captured.
"""
import array
import os
import threading
import time

RATE = 16000
FRAME_MS = 50
FRAME_N = RATE * FRAME_MS // 1000
MAX_S = 16.0                                    # a question is <= 15 s (client.py); one second of slack


class _Base:
    def __init__(self):
        self.buf = array.array("h")
        self.lock = threading.Lock()
        self.error = ""
        self.ok = True

    def _add(self, samples):
        with self.lock:
            room = int(MAX_S * RATE) - len(self.buf)
            if room > 0:
                self.buf.extend(samples[:room])

    def read_all(self):
        with self.lock:
            out, self.buf = self.buf, array.array("h")
        return out

    def close(self):
        pass


class NullIn(_Base):
    pass


class SimIn(_Base):
    """Tests: inject() samples as if spoken."""

    def inject(self, samples):
        self._add(array.array("h", samples))


class SdIn(_Base):
    def __init__(self, device=None):
        super().__init__()
        import sounddevice as sd
        self.s = sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=FRAME_N, device=device,
                                callback=lambda data, n, t, st: self._add(array.array("h", bytes(data))))
        self.s.start()

    def close(self):
        try:
            self.s.stop()
            self.s.close()
        except Exception:
            pass


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

    class WAVEINCAPSW(ctypes.Structure):
        _fields_ = [("wMid", wintypes.WORD), ("wPid", wintypes.WORD), ("vDriverVersion", wintypes.UINT),
                    ("szPname", ctypes.c_wchar * 32), ("dwFormats", wintypes.DWORD), ("wChannels", wintypes.WORD),
                    ("wReserved1", wintypes.WORD)]
    WHDR_DONE = 1
    WAVE_MAPPER = 0xFFFFFFFF


class WinIn(_Base):
    """waveIn at 16 kHz with 8 rotating 50 ms buffers, polled by its own thread."""

    def __init__(self, device=None):
        super().__init__()
        self.mm = ctypes.windll.winmm
        self.h = wintypes.HANDLE()
        fmt = WAVEFORMATEX(1, 1, RATE, RATE * 2, 2, 16, 0)
        r = self.mm.waveInOpen(ctypes.byref(self.h), WAVE_MAPPER if device is None else int(device),
                               ctypes.byref(fmt), 0, 0, 0)
        if r != 0:
            raise OSError("waveInOpen failed (%d): no microphone?" % r)
        self.bufs = [ctypes.create_string_buffer(FRAME_N * 2) for _ in range(8)]
        self.hdrs = [WAVEHDR() for _ in range(8)]
        sz = ctypes.sizeof(WAVEHDR)
        for b, h in zip(self.bufs, self.hdrs):
            h.lpData = ctypes.cast(b, ctypes.c_void_p)
            h.dwBufferLength = FRAME_N * 2
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
                for i, h in enumerate(self.hdrs):
                    if h.dwFlags & WHDR_DONE:
                        n = h.dwBytesRecorded
                        if n:
                            self._add(array.array("h", self.bufs[i].raw[:n - n % 2]))
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

    def close(self):
        self.run = False
        try:
            self.th.join(1.0)
            self.mm.waveInStop(self.h)
            self.mm.waveInReset(self.h)
            for h in self.hdrs:
                self.mm.waveInUnprepareHeader(self.h, ctypes.byref(h), ctypes.sizeof(WAVEHDR))
            self.mm.waveInClose(self.h)
        except Exception:
            pass


def auto_backend():
    if os.environ.get("TRYGONS_SOUND") == "off":
        return "null"
    if os.name == "nt":
        return "winmm"
    try:
        import sounddevice  # noqa: F401
        return "sounddevice"
    except Exception:
        return "null"


def open_input(backend="auto", device=None):
    """-> (input, error text). A failing device = a null input + the reason."""
    b = auto_backend() if backend in (None, "auto") else backend
    try:
        if b == "winmm":
            return WinIn(device), ""
        if b == "sounddevice":
            return SdIn(device), ""
        if b == "sim":
            return SimIn(), ""
        if b == "null":
            return NullIn(), ""
        return NullIn(), "unknown microphone backend %r" % b
    except Exception as e:
        return NullIn(), "no microphone (%s): %s" % (b, str(e)[:80])


def list_inputs():
    """[(index, name)] of the PC's microphones; [] when none / not Windows."""
    if os.name != "nt" or os.environ.get("TRYGONS_SOUND") == "off":
        return []
    try:
        mm = ctypes.windll.winmm
        out = []
        for i in range(mm.waveInGetNumDevs()):
            c = WAVEINCAPSW()
            if mm.waveInGetDevCapsW(i, ctypes.byref(c), ctypes.sizeof(c)) == 0:
                out.append((i, c.szPname))
        return out
    except Exception:
        return []


def level_db(samples):
    """The loudness of a capture in dBFS (-90 = silence)."""
    if not samples:
        return -90.0
    import math
    s = sum(x * x for x in samples[::4]) / max(1, len(samples[::4]))
    return max(-90.0, 10 * math.log10(s / 32768.0 ** 2)) if s > 0 else -90.0
