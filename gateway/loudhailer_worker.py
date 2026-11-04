"""The loudhailer's OWN process on the gateway PC (step 13r, owner 2026-11-02): the USB sound card = the horn
amplifier (speaker) + the boat microphone, the half-duplex rule and the audio ring recording.

    python loudhailer_worker.py --backend sim|auto|alsa|winmm|sounddevice|null [--out-dev D] [--in-dev D]
                                [--ring DIR] [--seg-s 60] [--volume-max 80] [--sim-reply-s 0]

It never talks to the AURIX and nothing in it is in a control path: it only plays what the gateway hands it and
hands back what the microphone hears. The gateway (loudhailer.py) restarts it when it dies (a notice only).

stdin / stdout = frames: 1 byte kind + u32 length + data
  gateway -> worker   J {"talk": bool} | {"volume": pct} | {"signal": "danger" | "long"} | {"stop": true}
                      A one ADPCM voice frame to play (voicecodec)
  worker -> gateway   J status {"t": "status", ...} once a second, {"t": "signal_done"}
                      M one ADPCM frame of the microphone - ONLY while nothing is spoken / played (half duplex:
                        the boat mic is muted while someone talks, no echo / howl to the shore)

Half duplex: from the gateway's talk ON (or a sound signal) until HANG_S after the last frame played the
microphone is muted (not sent). The ring records BOTH directions in one track: what the horn played while
talking, else what the microphone heard - 1-minute WAV files (8 kHz, UTC start time in the name, like the
camera ring) next to the camera folders; the gateway's recorder links them into SAVE / KEEP events.
"""
import argparse
import array
import calendar
import json
import os
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "groundstation"))
import audiodev  # noqa: E402
import voicecodec as vc  # noqa: E402

HANG_S = 0.3                  # the mic stays muted this long after the last frame played (the horn's tail)
PREBUFFER = 2                 # talk frames buffered before the first one plays (jitter)
MAX_QUEUE = 25                # 1 s of talk at most waiting (older ones dropped: latency stays low)


def put(out, lock, kind, data):
    with lock:
        out.write(kind + struct.pack("<I", len(data)) + data)
        out.flush()


class WavRing:
    """1-minute WAV files YYYYMMDD-HHMMSS.wav (UTC), cut at the clock minute; the header is rewritten every
    second, so a file cut by a power loss still plays up to its last second."""

    def __init__(self, folder, seg_s=60):
        self.folder, self.seg_s = folder, seg_s
        self.f = None
        self.start = None
        self.n = 0
        self.last_hdr = 0.0
        self.filled = 0                 # samples of silence written to stay on the clock (tests / status)
        self.dropped = 0                # frames not written: the file was ahead of the clock (a catch-up / clock back)
        self.recovered = 0              # catch-up frames written into the silence of their own gap (review LOW-2)
        self.hole = None                # [next, end) samples of silence a catch-up frame may still take
        self.reopened = 0               # existing files continued (never truncated)
        self.error = ""
        os.makedirs(folder, exist_ok=True)

    def _open(self, now):
        """Review of 13r-r MED-B: an EXISTING file of this minute (a worker restart mid-minute, the clock set back)
        is NEVER truncated - it may be hard-linked into a SAVE / KEEP event (permanent evidence): it is opened for
        writing at its end and the clock rule of write() goes on from its last sample (silence up to now, or frames
        dropped while the file is ahead of the clock)."""
        start = int(now // self.seg_s * self.seg_s)
        name = time.strftime("%Y%m%d-%H%M%S", time.gmtime(start)) + ".wav"
        self.close()
        path = os.path.join(self.folder, name)
        n = 0
        if os.path.exists(path):
            f = open(path, "r+b")
            size = os.path.getsize(path)
            n = max(0, (size - 44) // 2)                # a file shorter than its header holds no audio
            f.seek(44 + 2 * n)
            f.truncate()                                # only an odd last byte / nothing: never a sample
            self.reopened += 1
        else:
            f = open(path, "wb")
        self.f, self.start, self.n = f, start, n
        self.hole = None
        self._header()

    def _header(self):
        data = self.n * 2
        self.f.seek(0)
        self.f.write(b"RIFF" + struct.pack("<I", 36 + data) + b"WAVEfmt " +
                     struct.pack("<IHHIIHH", 16, 1, 1, vc.RATE, vc.RATE * 2, 2, 16) + b"data" + struct.pack("<I", data))
        self.f.seek(0, 2)

    def write(self, pcm, now):
        """One frame at the time now. Review 13r MED-3: the file stays on the clock - a gap (the worker's loop was
        late / caught up, the file opened in the middle of its minute) is filled with silence up to now, so the
        AUDIO track never drifts against the camera timeline (its name = the time of its first sample).
        Review of 13r-r MED-A: the other way too - a frame while the file is already half a frame or more AHEAD of
        the clock (the loop catching up missed steps back to back after a 80-500 ms stall, the clock set back) is
        dropped, so a stall is never counted twice (silence + the catch-up frames). LOW-5: the clock set back before
        this file's minute rolls to that minute's file; a file never grows past its minute (+ one frame).
        Review of 38add4b LOW-2: the catch-up frames are REAL audio of the stall - the frame that finds the gap is
        written at the START of the gap (its own time), the silence after it, and the catch-up frames that follow
        are written into that silence in order (overwriting it) instead of being dropped; only what never came
        stays silent. LOW-1: the clock set back by the PC (NTP / the GPS time) leaves the track silent / short for
        that much time - the files keep the clock, never the samples (documented in gateway/README.md 23)."""
        try:
            if self.f is None or now >= self.start + self.seg_s or now < self.start:
                self._open(now)
            gap = int((now - self.start) * vc.RATE) - self.n       # samples missing before this frame
            nsamp = len(pcm)                                       # samples (an array of int16)
            if -gap >= vc.FRAME_N // 2 or self.n >= int(self.seg_s * vc.RATE):
                h = self.hole
                if h is not None and h[1] - h[0] >= nsamp:           # a catch-up frame: into its gap's silence
                    self.f.seek(44 + 2 * h[0])
                    self.f.write(bytes(pcm))
                    self.f.seek(0, 2)
                    h[0] += nsamp
                    self.filled -= nsamp
                    self.recovered += 1
                else:
                    self.dropped += 1
                return
            if gap >= 2 * vc.FRAME_N:
                gap = min(gap, int(self.seg_s * vc.RATE) - self.n)
                self.f.write(bytes(pcm))                             # this frame first: its own time
                self.n += nsamp
                if gap > 0:
                    self.hole = [self.n, self.n + gap]
                    self.f.write(bytes(2 * gap))
                    self.n += gap
                    self.filled += gap
            else:
                self.hole = None                                     # on the clock again
                self.f.write(bytes(pcm))
                self.n += nsamp
            if now - self.last_hdr >= 1.0:
                self.last_hdr = now
                self._header()
                self.f.flush()
            self.error = ""
        except OSError as e:
            self.error = str(e)[:80]
            self.f = None

    def close(self):
        if self.f is not None:
            try:
                self._header()
                self.f.close()
            except OSError:
                pass
            self.f = None


DEFAULT_NAMES = ("", "default", "sysdefault", "pulse")     # review of 38add4b LOW-3: these open the PC's own device


def named(dev):
    """A sound device NAMED in gateway.json (never None, "" or "default" / "sysdefault" / "pulse" = the PC's own
    default device)."""
    return dev is not None and str(dev).strip().lower() not in DEFAULT_NAMES


def seg_time(name):
    if len(name) != 19 or not name.endswith(".wav"):
        return None
    try:
        return calendar.timegm(time.strptime(name[:15], "%Y%m%d-%H%M%S"))
    except ValueError:
        return None


class Worker:
    def __init__(self, a, out, clock=time.time):
        self.a = a
        self.out = out
        self.lock = threading.Lock()
        self.clock = clock
        real = a.backend not in ("sim", "null")
        # review 13r LOW-1: a real backend needs the sound card NAMED (gateway.json loudhailer out_dev / in_dev):
        # never the PC's default device (the laptop's own speaker / microphone) silently
        # (review of 13r-r LOW-3: "" / "default" are no name either - they open the PC's default device)
        if real and not named(a.out_dev):
            self.spk, e1 = audiodev.NullOut(), "speaker not configured (loudhailer out_dev)"
        else:
            self.spk, e1 = audiodev.open_output(a.backend, a.out_dev)
        if real and not named(a.in_dev):
            self.mic, e2 = audiodev.NullIn(), "microphone not configured (loudhailer in_dev)"
        else:
            self.mic, e2 = audiodev.open_input(a.backend, a.in_dev, speaker=self.spk, reply_every_s=a.sim_reply_s)
        self.err = "; ".join(e for e in (e1, e2) if e)
        self.ring = WavRing(a.ring, a.seg_s) if a.ring else None
        self.talk = False
        self.volume = min(a.volume_max, a.volume)
        self.queue = []
        self.started_play = False
        self.mute_until = 0.0
        self.signal = []                # frames of a sound signal still to play
        self.signal_kind = None
        self.n_mic = self.n_play = self.n_drop = 0
        self.mic_db = self.out_db = -90.0
        self.running = True

    # --- input from the gateway -------------------------------------------------------------------------------
    def reader(self, inp):
        try:
            while self.running:
                h = inp.read(5)
                if len(h) < 5:
                    break
                kind, n = h[:1], struct.unpack("<I", h[1:])[0]
                data = inp.read(n) if n else b""
                if len(data) < n:
                    break
                if kind == b"A":
                    with self.lock:
                        if len(self.queue) >= MAX_QUEUE:
                            self.queue.pop(0)
                            self.n_drop += 1
                        self.queue.append(data)
                elif kind == b"J":
                    self.control(json.loads(data.decode("utf-8")))
        except (OSError, ValueError):
            pass
        self.running = False            # the gateway is gone: stop

    def control(self, m):
        if "talk" in m:
            self.talk = bool(m["talk"])
            if not self.talk:
                self.started_play = False
        if "volume" in m:
            try:
                self.volume = max(0, min(self.a.volume_max, int(m["volume"])))
            except (TypeError, ValueError):
                pass
        if m.get("signal"):
            pcm = vc.signal(m["signal"])
            if pcm is not None:
                self.signal, self.signal_kind = vc.frames(pcm), m["signal"]
        if m.get("stop"):
            self.running = False

    # --- one 40 ms step ---------------------------------------------------------------------------------------
    def step(self, now):
        g = self.volume / 100.0
        played = None
        with self.lock:
            q = self.queue
            if self.signal:
                played = self.signal.pop(0)
            elif q and (self.started_play or len(q) >= PREBUFFER or not self.talk):
                self.started_play = True
                try:
                    played = vc.decode(q.pop(0))
                except ValueError:
                    self.n_drop += 1
        if played is not None:
            out = vc.gain(played, g)
            self.spk.write(out)
            self.n_play += 1
            self.out_db = vc.level_db(out)
            self.mute_until = now + HANG_S
        if self.signal_kind and not self.signal:
            self.emit({"t": "signal_done", "kind": self.signal_kind})
            self.signal_kind = None
        mic = self.mic.read()
        if mic is not None:
            self.mic_db = vc.level_db(mic)
        if self.ring:
            # review 13r MED-3: ONE frame every 40 ms step, whatever happens: the shore's voice (before the horn's
            # gain) when a frame played, else the microphone (also while it is muted for the shore: prebuffer,
            # underrun, hang time), else silence - the track keeps the camera timeline
            self.ring.write(played if played is not None else (mic if mic is not None else vc.silence()), now)
        if mic is None:
            return
        muted = self.talk or played is not None or now < self.mute_until
        if muted:
            return                                 # half duplex: never back to the shore while the horn speaks
        self.n_mic += 1
        put(self.out, self.lock_out, b"M", vc.encode(mic))

    def emit(self, obj):
        put(self.out, self.lock_out, b"J", json.dumps(obj).encode("utf-8"))

    def status(self):
        errs = [e for e in (self.err, self.spk.error, self.mic.error, self.ring.error if self.ring else "") if e]
        return {"t": "status", "backend": self.a.backend if self.a.backend != "auto" else audiodev.auto_backend(),
                "out_ok": bool(self.spk.ok and not isinstance(self.spk, audiodev.NullOut)),
                "in_ok": bool(self.mic.ok and not isinstance(self.mic, audiodev.NullIn)),
                "error": "; ".join(errs)[:160], "talk": self.talk, "volume": self.volume,
                "volume_max": self.a.volume_max, "mic_db": round(self.mic_db, 1), "out_db": round(self.out_db, 1),
                "mic_frames": self.n_mic, "played": self.n_play, "dropped": self.n_drop,
                "playing": bool(self.queue or self.signal), "signal": self.signal_kind,
                "ring": bool(self.ring), "ring_filled_s": round(self.ring.filled / vc.RATE, 2) if self.ring else 0,
                "ring_dropped": self.ring.dropped if self.ring else 0, "ring_file": os.path.basename(self.ring.f.name) if self.ring and self.ring.f
                else None,
                "sim_speaker_frames": getattr(self.spk, "frames", 0)}

    def run(self, inp):
        self.lock_out = threading.Lock()
        threading.Thread(target=self.reader, args=(inp,), daemon=True).start()
        nxt = time.monotonic()
        last_status = 0.0
        while self.running:
            now = self.clock()
            try:
                self.step(now)
            except Exception as e:                  # a device problem: reported, the loop keeps going
                self.err = "step: %s" % str(e)[:80]
            if now - last_status >= 1.0:
                last_status = now
                try:
                    self.emit(self.status())
                except OSError:
                    break
            nxt += vc.FRAME_MS / 1000.0
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            elif d < -0.5:
                nxt = time.monotonic()              # far behind (the PC was busy): catch up, never a burst
        if self.ring:
            self.ring.close()
        self.spk.close()
        self.mic.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="auto", choices=audiodev.BACKENDS)
    ap.add_argument("--out-dev", default=None)
    ap.add_argument("--in-dev", default=None)
    ap.add_argument("--ring", default=None)
    ap.add_argument("--seg-s", type=int, default=60)
    ap.add_argument("--volume-max", type=int, default=80)
    ap.add_argument("--volume", type=int, default=70)
    ap.add_argument("--sim-reply-s", type=float, default=0.0)
    a = ap.parse_args()
    try:
        os.nice(5)                                  # (Ubuntu) below the gateway's main jobs, above vision (19)
    except (AttributeError, OSError):
        pass
    Worker(a, sys.stdout.buffer).run(sys.stdin.buffer)


if __name__ == "__main__":
    main()
