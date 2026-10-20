"""The shore PC's USB joystick for REMOTE MANUAL driving (section D).

Any standard USB (HID) joystick works, e.g. the Thrustmaster T.16000M stick
with the TWCS throttle (two separate devices). Windows: the built-in
joystick API (winmm, nothing to install). Linux: /dev/input/js*.

shore_joystick.json (next to app.py, written by the ground station):
  {"steer": {"dev": 0, "axis": "X", "invert": false},
   "thr":   {"dev": 1, "axis": "Z", "invert": true, "lever": true},
   "release_button": {"dev": 0, "button": 1}}
"lever": true = non-returning throttle lever (TWCS, 0 back .. 1 forward);
false = spring-centred stick Y (-1 .. +1).

RemoteSender sends {"t":"rjoy", "seq", "ts", "x", "lever" | "y"} at 30 Hz
through the gateway. The AURIX only uses it while REMOTE MANUAL is on and
this station holds HELM.
"""
import ctypes
import json
import os
import sys
import threading
import time

AXES = ["X", "Y", "Z", "R", "U", "V"]
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shore_joystick.json")
DEFAULT = {"steer": {"dev": 0, "axis": "X", "invert": False},
           "thr": {"dev": 1, "axis": "Z", "invert": True, "lever": True},
           "release_button": {"dev": 0, "button": 1}}


# --- reading ---------------------------------------------------------------------------

class _JOYINFOEX(ctypes.Structure):
    _fields_ = [(n, ctypes.c_uint32) for n in ("dwSize", "dwFlags", "dwXpos", "dwYpos", "dwZpos", "dwRpos",
                                               "dwUpos", "dwVpos", "dwButtons", "dwButtonNumber", "dwPOV",
                                               "dwReserved1", "dwReserved2")]


class _JOYCAPSW(ctypes.Structure):
    _fields_ = [("wMid", ctypes.c_uint16), ("wPid", ctypes.c_uint16), ("szPname", ctypes.c_wchar * 32)] + \
               [(n, ctypes.c_uint32) for n in ("wXmin", "wXmax", "wYmin", "wYmax", "wZmin", "wZmax",
                                               "wNumButtons", "wPeriodMin", "wPeriodMax", "wRmin", "wRmax",
                                               "wUmin", "wUmax", "wVmin", "wVmax", "wCaps", "wMaxAxes",
                                               "wNumAxes", "wMaxButtons")] + \
               [("szRegKey", ctypes.c_wchar * 32), ("szOEMVxD", ctypes.c_wchar * 260)]


class Joysticks:
    """All joysticks of this PC: list() and read(dev) -> {"axes": {X: -1..1},
    "buttons": int, "ok": bool}."""

    def __init__(self):
        self.win = sys.platform == "win32"
        self.caps = {}
        if self.win:
            self.mm = ctypes.WinDLL("winmm")

    def list(self):
        out = []
        if self.win:
            n = self.mm.joyGetNumDevs()
            for i in range(min(n, 16)):
                c = _JOYCAPSW()
                if self.mm.joyGetDevCapsW(i, ctypes.byref(c), ctypes.sizeof(c)) == 0 and self._read_win(i)["ok"]:
                    self.caps[i] = c
                    out.append({"dev": i, "name": c.szPname, "axes": c.wNumAxes, "buttons": c.wNumButtons})
        else:
            for i in range(16):
                p = "/dev/input/js%d" % i
                if os.path.exists(p):
                    out.append({"dev": i, "name": p, "axes": 6, "buttons": 32})
        return out

    def _read_win(self, dev):
        ji = _JOYINFOEX()
        ji.dwSize = ctypes.sizeof(ji)
        ji.dwFlags = 0xFF                                   # JOY_RETURNALL
        if self.mm.joyGetPosEx(dev, ctypes.byref(ji)) != 0:
            return {"ok": False, "axes": {}, "buttons": 0}
        c = self.caps.get(dev)
        raw = {"X": ji.dwXpos, "Y": ji.dwYpos, "Z": ji.dwZpos, "R": ji.dwRpos, "U": ji.dwUpos, "V": ji.dwVpos}
        rng = {"X": (c.wXmin, c.wXmax), "Y": (c.wYmin, c.wYmax), "Z": (c.wZmin, c.wZmax), "R": (c.wRmin, c.wRmax),
               "U": (c.wUmin, c.wUmax), "V": (c.wVmin, c.wVmax)} if c else {k: (0, 65535) for k in raw}
        axes = {}
        for k, v in raw.items():
            lo, hi = rng[k]
            axes[k] = (2.0 * (v - lo) / (hi - lo) - 1.0) if hi > lo else 0.0
        return {"ok": True, "axes": axes, "buttons": ji.dwButtons}

    def read(self, dev):
        if self.win:
            return self._read_win(dev)
        js = self.caps.get(dev)
        if js is None:
            js = self.caps[dev] = LinuxJs("/dev/input/js%d" % dev)
        return js.read()


class LinuxJs:
    """/dev/input/jsN: a thread keeps the latest axis / button values
    (struct js_event: u32 time, s16 value, u8 type, u8 number)."""

    def __init__(self, path):
        import struct as _st
        self.st = _st
        self.axes = {}
        self.buttons = 0
        self.ok = False
        try:
            self.f = open(path, "rb")
            self.ok = True
            threading.Thread(target=self._loop, daemon=True).start()
        except OSError:
            self.f = None

    def _loop(self):
        try:
            while True:
                ev = self.f.read(8)
                if len(ev) < 8:
                    break
                _t, value, typ, num = self.st.unpack("<IhBB", ev)
                typ &= 0x7F                                   # without the "initial state" bit
                if typ == 2 and num < len(AXES):
                    self.axes[AXES[num]] = value / 32767.0
                elif typ == 1 and num < 32:
                    self.buttons = (self.buttons | (1 << num)) if value else (self.buttons & ~(1 << num))
        except OSError:
            pass
        self.ok = False

    def read(self):
        return {"ok": self.ok, "axes": dict(self.axes), "buttons": self.buttons}


def load_config():
    try:
        with open(CONFIG, encoding="utf-8") as f:
            c = json.load(f)
        return {**DEFAULT, **c}
    except (OSError, ValueError):
        return dict(DEFAULT)


def save_config(c):
    with open(CONFIG, "w", encoding="utf-8") as f:
        json.dump(c, f, indent=1)


def to_remote(js, cfg):
    """Joystick readings -> {"x", "lever" or "y", "release"} or None if a device is missing."""
    s, t = cfg["steer"], cfg["thr"]
    a, b = js.read(s["dev"]), js.read(t["dev"])
    if not a["ok"] or not b["ok"]:
        return None
    x = a["axes"].get(s["axis"], 0.0) * (-1 if s.get("invert") else 1)
    v = b["axes"].get(t["axis"], 0.0) * (-1 if t.get("invert") else 1)
    out = {"x": max(-1.0, min(1.0, x))}
    if t.get("lever", True):
        out["lever"] = max(0.0, min(1.0, (v + 1.0) / 2.0))
    else:
        out["y"] = max(-1.0, min(1.0, v))
    rb = cfg.get("release_button")
    if rb:
        r = js.read(rb["dev"])
        out["release"] = bool(r["ok"] and r["buttons"] & (1 << (rb["button"] - 1)))
    return out


# --- sending -----------------------------------------------------------------------------

class RemoteSender:
    """30 Hz rjoy packets through a GatewayLink. source() returns the
    {"x", "lever"/"y"} dict (the real joystick, or a test)."""

    def __init__(self, gwlink, source, hz=30.0):
        self.gw = gwlink
        self.source = source
        self.period = 1.0 / hz
        self.seq = 0
        self.running = False
        self.sent = 0
        self.last = None
        self.missing = False                # the joystick can't be read

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False

    def _loop(self):
        nxt = time.time()
        while self.running:
            v = self.source()
            self.missing = v is None
            if v is not None and self.gw.connected:
                self.seq = (self.seq + 1) & 0xFFFF
                msg = {"t": "rjoy", "seq": self.seq, "ts": self.gw.now()}
                msg.update(v)
                self.last = msg
                self.gw.send_rjoy(msg)
                self.sent += 1
            nxt += self.period
            time.sleep(max(0.0, nxt - time.time()))
