"""Loudhailer voice (step 13r, owner 2026-11-02): the codec and the signed UDP packets, shared by the ground
station (groundstation/hailer.py) and the gateway (gateway/loudhailer.py + loudhailer_worker.py).

Voice = 8 kHz mono, 40 ms frames (320 samples), IMA ADPCM 4 bit = 32 kbit/s (~45 kbit/s on the wire with the
headers). Pure Python (no codec library on the PCs): Opus 16-24 kbit/s is a later option once a library is
fitted (PARTS.md / gateway README). Every packet carries its own ADPCM start state: a lost packet is only
40 ms of silence, never a garbled rest of the sentence.

Packet (UDP, the station's session port = the gateway's station port, like the remote joystick):
    b"TV" | ver 1 | kind | session 8 bytes | seq u16 | ts f64 | payload | HMAC-SHA256(udp_key, all before)[:16]
kind  T = talk (station -> gateway, only from the station that holds the floor)
      L = listen (gateway -> station: the boat microphone)
      H = hello (station -> gateway, 1 Hz: "I listen" + where to send; payload = flags b0 listen)
The key is the station's own session key (welcome udp_key): a packet of another session is dropped.
Nothing here can reach the AURIX.
"""
import array
import hashlib
import hmac
import math
import random
import struct

RATE = 8000
FRAME_MS = 40
FRAME_N = RATE * FRAME_MS // 1000             # 320 samples
FRAME_BYTES = 3 + FRAME_N // 2                # ADPCM state (3) + 4 bit per sample
KBPS = FRAME_BYTES * 8 / float(FRAME_MS)      # 32.6 kbit/s of voice
MAGIC = b"TV"
VERSION = 1
HEAD = struct.Struct("<2sBc8sHd")             # magic, version, kind, session, seq, ts
MAC_LEN = 16
KINDS = (b"T", b"L", b"H")

_STEPS = (7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88,
          97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
          724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660,
          4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818,
          18500, 20350, 22385, 24623, 27086, 29794, 32767)
_INDEX = (-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8)


# --- IMA ADPCM --------------------------------------------------------------------------------------------------
def encode(pcm, state=None):
    """pcm: a sequence of int16 (an even count). -> bytes = the start state (predictor i16, index u8) + nibbles.
    state: (predictor, index) to start from (None = from the first sample: every packet decodes alone)."""
    if state is None:
        pred, idx = (int(pcm[0]) if len(pcm) else 0), 0
    else:
        pred, idx = state
    out = bytearray(struct.pack("<hB", max(-32768, min(32767, pred)), idx))
    steps, itab = _STEPS, _INDEX
    nib_lo = None
    for s in pcm:
        step = steps[idx]
        diff = int(s) - pred
        code = 0
        if diff < 0:
            code = 8
            diff = -diff
        delta = step >> 3
        if diff >= step:
            code |= 4
            diff -= step
            delta += step
        step >>= 1
        if diff >= step:
            code |= 2
            diff -= step
            delta += step
        step >>= 1
        if diff >= step:
            code |= 1
            delta += step
        pred = pred - delta if code & 8 else pred + delta
        if pred > 32767:
            pred = 32767
        elif pred < -32768:
            pred = -32768
        idx += itab[code]
        if idx < 0:
            idx = 0
        elif idx > 88:
            idx = 88
        if nib_lo is None:
            nib_lo = code
        else:
            out.append(nib_lo | (code << 4))
            nib_lo = None
    if nib_lo is not None:
        out.append(nib_lo)
    return bytes(out)


def decode(data):
    """bytes of encode() -> array('h') (2 samples per data byte after the 3-byte state). Bad data -> ValueError."""
    if len(data) < 3:
        raise ValueError("ADPCM frame too short")
    pred, idx = struct.unpack_from("<hB", data)
    if idx > 88:
        raise ValueError("ADPCM index out of range")
    out = array.array("h")
    steps, itab = _STEPS, _INDEX
    for b in data[3:]:
        for code in (b & 15, b >> 4):
            step = steps[idx]
            delta = step >> 3
            if code & 4:
                delta += step
            if code & 2:
                delta += step >> 1
            if code & 1:
                delta += step >> 2
            pred = pred - delta if code & 8 else pred + delta
            if pred > 32767:
                pred = 32767
            elif pred < -32768:
                pred = -32768
            idx += itab[code]
            if idx < 0:
                idx = 0
            elif idx > 88:
                idx = 88
            out.append(pred)
    return out


# --- sample helpers ---------------------------------------------------------------------------------------------
def level_db(pcm):
    """RMS level in dBFS (-90 = silence)."""
    if not pcm:
        return -90.0
    acc = 0
    for s in pcm:
        acc += s * s
    rms = math.sqrt(acc / float(len(pcm)))
    return max(-90.0, 20.0 * math.log10(rms / 32768.0)) if rms > 0 else -90.0


def meter(db):
    """dBFS -> 0..1 for a level meter (-60 dB = empty)."""
    return max(0.0, min(1.0, (db + 60.0) / 60.0))


def gain(pcm, g):
    if abs(g - 1.0) < 1e-3:
        return array.array("h", pcm)
    return array.array("h", (max(-32767, min(32767, int(s * g))) for s in pcm))


def silence(n=FRAME_N):
    return array.array("h", bytes(2 * n))


def resample(pcm, src_rate, dst_rate=RATE):
    """Linear resampling (a ready message from the TTS at 16 / 22.05 kHz -> 8 kHz)."""
    if src_rate == dst_rate or not pcm:
        return array.array("h", pcm)
    n = int(len(pcm) * dst_rate / float(src_rate))
    out = array.array("h", bytes(2 * n))
    r = src_rate / float(dst_rate)
    last = len(pcm) - 1
    for i in range(n):
        x = i * r
        j = int(x)
        f = x - j
        a = pcm[j]
        b = pcm[j + 1] if j < last else a
        out[i] = int(a + (b - a) * f)
    return out


def frames(pcm, n=FRAME_N):
    """Cut samples into whole frames (the last one padded with silence)."""
    out = []
    for i in range(0, len(pcm), n):
        f = array.array("h", pcm[i:i + n])
        if len(f) < n:
            f.extend(array.array("h", bytes(2 * (n - len(f)))))
        out.append(f)
    return out


def tone(freq, ms, vol=0.6, rate=RATE, attack_ms=20):
    """A horn-like tone (fundamental + 2nd / 3rd harmonic), soft start / end."""
    n = int(rate * ms / 1000)
    a = max(1, int(rate * attack_ms / 1000))
    out = array.array("h", bytes(2 * n))
    w = 2 * math.pi * freq / rate
    for i in range(n):
        env = min(1.0, i / a, (n - 1 - i) / a)
        v = math.sin(w * i) + 0.35 * math.sin(2 * w * i) + 0.2 * math.sin(3 * w * i)
        out[i] = int(32767 * vol * env * v / 1.55)
    return out


# the sound signals (clearly labelled on the screens): COLREG Rule 34 lengths - short ~1 s, prolonged 4-6 s
SIGNALS = {"danger": [(1000, 700)] * 5,       # 5 short blasts = danger / doubt ("keep clear")
           "long": [(5000, 0)]}               # 1 prolonged blast
SIGNAL_HZ = 420


def signal(kind, rate=RATE):
    """The samples of a sound signal (None = unknown kind)."""
    pat = SIGNALS.get(kind)
    if pat is None:
        return None
    out = array.array("h")
    for on_ms, off_ms in pat:
        out.extend(tone(SIGNAL_HZ, on_ms, rate=rate))
        out.extend(array.array("h", bytes(2 * int(rate * off_ms / 1000))))
    return out


def signal_s(kind):
    pat = SIGNALS.get(kind) or []
    return sum(a + b for a, b in pat) / 1000.0


def sea_noise(n=FRAME_N, level=600, rng=random):
    """The simulated microphone: wind / sea noise (a low-passed random walk)."""
    out = array.array("h", bytes(2 * n))
    v = 0.0
    for i in range(n):
        v = 0.85 * v + 0.15 * rng.uniform(-1.0, 1.0)
        out[i] = int(level * 3 * v)
    return out


# --- packets ----------------------------------------------------------------------------------------------------
def session_bytes(session):
    """The welcome "sid" (16 hex) -> 8 bytes."""
    b = bytes.fromhex(session)
    return (b + bytes(8))[:8]


def pack(kind, session, seq, ts, payload, key):
    """-> the signed datagram. kind b"T" / b"L" / b"H"; session = 8 bytes; key = the station's udp_key bytes."""
    body = HEAD.pack(MAGIC, VERSION, kind, session, seq & 0xFFFF, float(ts)) + bytes(payload)
    return body + hmac.new(key, body, hashlib.sha256).digest()[:MAC_LEN]


def is_voice(data):
    return len(data) >= HEAD.size + MAC_LEN and data[:2] == MAGIC


def peek(data):
    """-> (kind, session, seq, ts, payload) WITHOUT checking the signature (find the key first), or None."""
    if not is_voice(data):
        return None
    magic, ver, kind, sess, seq, ts = HEAD.unpack_from(data)
    if ver != VERSION or kind not in KINDS or not math.isfinite(ts):
        return None
    return kind, sess, seq, ts, data[HEAD.size:-MAC_LEN]


def verify(data, key):
    want = hmac.new(key, data[:-MAC_LEN], hashlib.sha256).digest()[:MAC_LEN]
    return hmac.compare_digest(want, data[-MAC_LEN:])


def unpack(data, key):
    """-> (kind, session, seq, ts, payload) when the signature is right, else None."""
    p = peek(data)
    if p is None or not verify(data, key):
        return None
    return p


def seq_newer(seq, last):
    """seq after last (16-bit wrap)."""
    return last is None or 0 < ((seq - last) & 0xFFFF) < 0x8000
