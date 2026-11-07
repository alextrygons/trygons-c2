"""The ground station's side of the loudhailer (step 13r, owner 2026-11-02): talk to any contact near the USV
(swimmers, fishing boats, coast guard, divers) through the horn speaker on the boat, and listen to the boat
microphone. NO VHF radio. Gateway side: gateway/loudhailer.py (+ its own audio process).

  PUSH-TO-TALK = a framed SCREEN button «ΟΜΙΛΙΑ», press and hold: on the bottom bar of screen 1, on bar 2 of
  screen 2 (the copilot's station too) and on the Tools page "Loudhailer". NOT on the joystick (owner choice).
  Half duplex, one speaker at a time: the gateway gives the floor to the FIRST station (the captain's press takes it
  at once from anyone: owner W54 - the displaced station sees «Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟ ΜΕΓΑΦΩΝΟ»); the others see
  «ΜΙΛΑ: ΚΥΒΕΡΝΗΤΗΣ / ΣΥΓΚΥΒΕΡΝΗΤΗΣ» and a greyed button.
  LISTEN: the boat microphone plays continuously on this PC (mute + volume + a white level meter); it is muted
  on the boat while anyone talks.
  Ready messages (one click, Greek then English, an editable list kept on this PC) are spoken with this PC's
  offline voice and sent like talk; sound signals (5 short blasts = danger, 1 prolonged) are made on the boat.

Display + requests only: nothing here sends an AURIX command, nothing changes the driving. Voice goes over UDP
(voicecodec, signed with this session's key); the gateway drops it first on a weak link (a note says so).
"""
import array
import os
import socket
import threading
import time
import wave

import audiodev
import i18n
import settings
import theme
import voicecodec as vc
from i18n import T

HELLO_S = 1.0
JITTER_FRAMES = 3                 # listen: 120 ms buffered before playing (a late packet is not a click)
LISTEN_STALE_S = 0.5              # no boat audio this long: the meter falls, "no audio"
DEFAULT_MESSAGES = [
    {"el": "Μη επανδρωμένο σκάφος – κρατηθείτε μακριά.", "en": "Unmanned vessel - keep clear."},
    {"el": "Προσοχή – το σκάφος κινείται, απομακρυνθείτε.", "en": "Caution - this vessel is moving, move away."},
    {"el": "Μείνετε εκεί που είστε – το σκάφος σταματά.", "en": "Stay where you are - the vessel is stopping."},
    {"el": "Δύτες, κολυμβητές: σας έχουμε δει, κρατάμε απόσταση.",
     "en": "Divers, swimmers: we have seen you, we keep our distance."},
    {"el": "Χρειάζεστε βοήθεια; Σηκώστε το χέρι σας.", "en": "Do you need help? Raise your arm."},
    {"el": "Λιμενικό: εδώ το μη επανδρωμένο σκάφος, σας ακούμε.",
     "en": "Coast guard: this is the unmanned vessel, we are listening."},
]
CREW_KEYS = {"captain": "hail.crew_captain", "copilot": "hail.crew_copilot", "single": "hail.crew_single",
             "station": "hail.crew_station"}


def messages():
    """The ready messages of this PC (gs_settings "hail_messages"; the defaults when none / broken)."""
    m = settings.get("hail_messages")
    if isinstance(m, list) and m and all(isinstance(x, dict) and (x.get("el") or x.get("en")) for x in m):
        return [{"el": str(x.get("el") or ""), "en": str(x.get("en") or "")} for x in m][:30]
    return [dict(x) for x in DEFAULT_MESSAGES]


def save_messages(lst):
    clean = [{"el": str(x.get("el") or "").strip()[:200], "en": str(x.get("en") or "").strip()[:200]} for x in lst]
    clean = [x for x in clean if x["el"] or x["en"]][:30]
    settings.put("hail_messages", clean)
    return clean


def reset_messages():
    settings.put("hail_messages", None)
    return messages()


def read_wav(path):
    """A TTS sentence -> 8 kHz int16 samples."""
    with wave.open(path, "rb") as w:
        rate, ch, sw = w.getframerate(), w.getnchannels(), w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if sw != 2:
        raise ValueError("not 16-bit")
    a = array.array("h", raw)
    if ch > 1:
        a = array.array("h", a[::ch])
    return vc.resample(a, rate)


class Hailer:
    """One per station (app.hail). tick() from the Tk loop every 100 ms; the audio runs in its own threads."""

    def __init__(self, app, backend=None, clock=time.time):
        self.app = app
        self.backend = backend or os.environ.get("TRYGONS_HAIL_BACKEND") or "auto"
        self.clock = clock
        self.link = None                 # the GatewayLink the socket belongs to
        self.sock = None
        self.key = self.sess = None
        self.rx_th = None
        self.out = None                  # the listen device (opened while listening)
        self.out_err = ""
        self.in_err = ""
        self.speaker = None              # the gateway's floor: {"id", "user", "crew", "what", "for_s"} or None
        self.mine = False                # the gateway gave THIS station the floor
        self.pressing = False
        self.talk_stop = threading.Event()
        self.talk_th = None
        self.next_hello = 0.0
        self.seq = 0
        self.rx_seq = None
        self.jbuf = []
        self.rx_t = 0.0
        self.rx_frames = 0
        self.rx_db = -90.0
        self.tx_db = -90.0
        self.tx_frames = 0
        self.info = {}                   # the gateway's per-station "hail" note (voice_ok, why, up_ms)
        self.note = ""                   # the last refusal / problem (screen language)
        self.synth = None                # tests: fn(texts) -> 8 kHz samples (else the PC's offline voice)
        self.msg_busy = False
        self.page = None
        self.lock = threading.Lock()

    # --- settings ------------------------------------------------------------------------------------------------
    listen_local = None              # tests: several stations in one process (else the per-PC setting)

    @property
    def listen(self):
        if self.listen_local is not None:
            return self.listen_local
        return bool(settings.get("hail_listen", True))

    def set_listen(self, on):
        if self.listen_local is not None:
            self.listen_local = bool(on)
        else:
            settings.put("hail_listen", bool(on))
        self.next_hello = 0.0
        if not on:
            self._close_out()

    @property
    def volume(self):
        try:
            return max(0, min(100, int(settings.get("hail_volume", 80))))
        except (TypeError, ValueError):
            return 80

    def set_volume(self, pct):
        settings.put("hail_volume", max(0, min(100, int(pct))))

    # --- the gateway --------------------------------------------------------------------------------------------
    def gw(self):
        """The gateway's loudhailer status (None = no gateway / no loudhailer)."""
        lk = self.app.link
        if lk is None or not getattr(lk, "welcome", None):
            return None
        h = (getattr(self.app, "gw_status", None) or {}).get("hail")
        return h if h and h.get("enabled") else None

    def available(self):
        h = self.gw()
        return h is not None and h.get("state") != "OFFLINE" and bool(h.get("out_ok"))

    def _bind(self):
        """Open the voice socket for the current gateway link (a new link = a new socket)."""
        lk = self.app.link
        ok = lk is not None and getattr(lk, "welcome", None) and lk.welcome.get("udp_key") and \
            hasattr(lk, "host")
        if not ok or self.gw() is None:
            if self.link is not None:
                self._unbind()
            return False
        if lk is self.link and self.sock is not None and self.sess == vc.session_bytes(lk.welcome["sid"]):
            return True                  # (a new login on the same link object = a new session: bound again)
        self._unbind()
        self.link = lk
        self.key = bytes.fromhex(lk.welcome["udp_key"])
        self.sess = vc.session_bytes(lk.welcome["sid"])
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if getattr(lk, "local_addr", None):
            s.bind((lk.local_addr, 0))
        else:
            s.bind(("0.0.0.0", 0))
        s.settimeout(0.5)
        with self.lock:                  # review 13r MED-2: a new link = the gateway's LISTEN seq starts again
            self.rx_seq = None
            self.jbuf = []
        self.sock = s
        self.rx_th = threading.Thread(target=self._rx, args=(s,), daemon=True)
        self.rx_th.start()
        self.next_hello = 0.0
        return True

    def _unbind(self):
        self.talk_stop.set()
        self.mine = self.pressing = False
        self.push_horn()
        s, self.sock, self.link = self.sock, None, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass
        self._close_out()                # (clears the jitter buffer)
        with self.lock:
            self.rx_seq = None           # review 13r MED-2: never compare a new link's seq with the old one
        self.speaker = None
        self.info = {}

    def _send(self, kind, payload):
        s = self.sock
        if s is None:
            return False
        self.seq = (self.seq + 1) & 0xFFFF
        ts = self.link.now() if hasattr(self.link, "now") else time.time()
        try:
            s.sendto(vc.pack(kind, self.sess, self.seq, ts, payload, self.key), (self.link.host, self.link.port))
            return True
        except OSError:
            return False

    def on_msg(self, kind, d):
        if kind == "hail_floor":
            self.speaker = d.get("speaker")
            self.mine = bool(self.speaker and self.link and self.speaker.get("id") == self.link.welcome.get("id"))
            if not self.mine:
                self.talk_stop.set()
        elif kind == "ptt_ack":
            self.speaker = d.get("speaker")
            if d.get("on") and d.get("ok"):
                self.mine = True
                if self.pressing and not self.msg_busy:
                    self._start_talk(None)
            elif d.get("on"):
                self.mine = False
                self.pressing = False
                self.talk_stop.set()
                # owner W54: "taken" = the captain took the horn over from this station
                self.note = T("hail.taken") if d.get("taken") else T("hail.refused", why=i18n.from_en(d.get("text", "")))
                self.app.say(self.note, theme.CAUTION)
        elif kind == "hail":
            self.info = d
        if kind in ("hail_floor", "ptt_ack"):
            self.push_horn()
        elif kind == "hail_signal_ack" and not d.get("ok"):
            self.note = T("hail.refused", why=i18n.from_en(d.get("text", "")))
            self.app.say(self.note, theme.CAUTION)
        elif kind == "hail_volume_ack":
            self.app.say(T("hail.vol_set", pct=d.get("pct"), limit=(self.gw() or {}).get("volume_max", "-"))
                         if d.get("ok") else T("hail.refused", why=i18n.from_en(d.get("text", ""))),
                         None if d.get("ok") else theme.CAUTION)

    # --- PUSH-TO-TALK ------------------------------------------------------------------------------------------
    def captain_here(self):
        """Owner W54: the captain's press takes the horn at once from anyone (the gateway decides; a sound signal that
        plays is never cut). Here only: do not refuse it locally."""
        f = getattr(self.app, "hail_is_captain", None)
        try:
            return bool(f()) if f is not None else False
        except Exception:
            return False

    def other_speaks(self):
        return bool(self.speaker) and not self.mine

    def push_horn(self):
        """AM2-r2 review MED-b: the horn state as a PLAIN bool into the assistant's view (view.horn); the view holds
        no reference to this object. Called on press / release / floor changes and by App.ov_tick."""
        a = getattr(self.app, "asst", None)
        v = getattr(a, "view", None)
        if v is not None:
            try:
                v.horn = bool(self.pressing or self.mine)
            except Exception:
                pass

    def _asst_horn(self):
        """The talking manual (AM1 review H1, AM2 review LOW-3): anything that may put sound into the horn - TALK,
        a ready message, a sound signal - first drops the assistant's capture and stops its voice AT ONCE."""
        a = getattr(self.app, "asst", None)
        if a is not None:
            try:
                a.horn_pressed()
            except Exception:
                pass

    def press(self, what="talk"):
        """The «ΟΜΙΛΙΑ» button went down: ask the gateway for the floor (the first wins)."""
        self._asst_horn()                        # a question never goes out of the horn
        if not self._bind():
            self.note = T("hail.no_gateway")
            return self.app.say(self.note, theme.CAUTION)
        if not self.available():
            self.note = T("hail.offline", why=i18n.from_en((self.gw() or {}).get("why", "")))
            return self.app.say(self.note, theme.CAUTION)
        if self.other_speaks() and not self.captain_here():
            self.note = T("hail.speaks", who=self.crew_text(self.speaker))
            return self.app.say(self.note, theme.CAUTION)
        self.pressing = True
        self.push_horn()
        self.app.link.send_msg({"t": "ptt", "on": True, "what": what})
        return True

    def release(self):
        """The button went up: stop talking, give the floor back."""
        if not self.pressing and not self.mine:
            return
        self.pressing = False
        self.talk_stop.set()
        if self.app.link is not None and getattr(self.app.link, "welcome", None):
            self.app.link.send_msg({"t": "ptt", "on": False})
        self.mine = False
        self.push_horn()

    def _start_talk(self, pcm):
        """pcm None = the microphone of this PC; else these samples (a ready message) at the real-time pace."""
        if self.talk_th is not None and self.talk_th.is_alive():
            return
        self.talk_stop = threading.Event()
        self.talk_th = threading.Thread(target=self._talk, args=(pcm, self.talk_stop), daemon=True)
        self.talk_th.start()

    def _talk(self, pcm, stop):
        mic = None
        if pcm is None:
            mic, err = audiodev.open_input(self.backend)
            self.in_err = err
            if err:
                self.note = T("hail.no_mic", err=err)
        frames = vc.frames(pcm) if pcm is not None else None
        paced = frames is not None or isinstance(mic, audiodev.NullIn)   # a message / no mic: 40 ms by the clock
        nxt = time.monotonic()
        try:
            while not stop.is_set():
                if frames is not None:
                    if not frames:
                        break
                    out = [frames.pop(0)]
                elif paced:
                    out = [vc.silence()]                 # no microphone here: silence holds the floor (note shown)
                else:
                    out = []
                    f = mic.read()
                    while f is not None:                 # everything the card captured (it sets the pace)
                        out.append(f)
                        f = mic.read()
                for f in [g for x in out for g in vc.frames(x)]:   # whole 40 ms frames only
                    self.tx_db = vc.level_db(f)
                    if self._send(b"T", vc.encode(f)):
                        self.tx_frames += 1
                if not paced:
                    stop.wait(0.01)
                    continue
                nxt += vc.FRAME_MS / 1000.0
                d = nxt - time.monotonic()
                if d > 0:
                    stop.wait(d)
                elif d < -0.5:
                    nxt = time.monotonic()
        finally:
            if mic is not None:
                mic.close()
            self.tx_db = -90.0
        if frames is not None and not stop.is_set():
            self.msg_busy = False
            self.release()                       # a ready message: the floor back when it ends
        self.msg_busy = False

    # --- ready messages + sound signals -----------------------------------------------------------------------
    def message_pcm(self, i):
        """The samples of ready message i: Greek then English (this PC's offline voice). None = no voice here."""
        lst = messages()
        if not 0 <= i < len(lst):
            return None
        texts = [(lg, lst[i][lg]) for lg in ("el", "en") if lst[i].get(lg)]
        if self.synth is not None:
            return self.synth(texts)
        snd = getattr(self.app, "sound", None)
        if snd is None or os.environ.get("TRYGONS_SOUND") == "off":
            return None
        out = array.array("h")
        for lg, text in texts:
            try:
                snd.lib.make(lg, [text])
                out.extend(read_wav(snd.lib.sentence_path(lg, text)))
                out.extend(vc.silence(vc.RATE // 3))
            except Exception:
                return None
        return out if out else None

    def play_message(self, i):
        """One click: the floor is asked, the message is spoken into it (made in a worker thread)."""
        self._asst_horn()
        if self.msg_busy or self.mine or self.pressing:
            return False
        if not self._bind() or not self.available():
            self.note = T("hail.offline", why=i18n.from_en((self.gw() or {}).get("why", "")))
            self.app.say(self.note, theme.CAUTION)
            return False
        if self.other_speaks():
            self.app.say(T("hail.speaks", who=self.crew_text(self.speaker)), theme.CAUTION)
            return False
        self.msg_busy = True

        def run():
            pcm = self.message_pcm(i)
            if pcm is None:
                self.msg_busy = False
                self.note = T("hail.no_voice")
                return
            self.pressing = True
            self.push_horn()
            self.app.link.send_msg({"t": "ptt", "on": True, "what": "message"})
            end = time.time() + 3.0
            while time.time() < end and not self.mine and self.pressing:
                time.sleep(0.02)
            if not self.mine:
                self.msg_busy = False
                self.pressing = False
                return
            self._start_talk(pcm)
        threading.Thread(target=run, daemon=True).start()
        return True

    def signal(self, kind):
        self._asst_horn()
        if not self._bind() or not self.available():
            self.app.say(T("hail.offline", why=i18n.from_en((self.gw() or {}).get("why", ""))), theme.CAUTION)
            return False
        self.app.link.send_msg({"t": "hail_signal", "kind": kind})
        return True

    def gw_volume(self, pct):
        if self.app.link is not None and getattr(self.app.link, "welcome", None):
            self.app.link.send_msg({"t": "hail_volume", "pct": int(pct)})

    # --- LISTEN ------------------------------------------------------------------------------------------------
    def _close_out(self):
        with self.lock:
            o, self.out = self.out, None
            self.jbuf = []
        if o is not None:
            o.close()

    def _rx(self, s):
        while self.sock is s:
            try:
                data, _addr = s.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            p = vc.unpack(data, self.key) if self.key else None
            if p is None or p[0] != b"L" or p[1] != self.sess:
                continue
            _k, _s, seq, _ts, payload = p
            if not vc.seq_newer(seq, self.rx_seq):
                continue
            self.rx_seq = seq
            try:
                pcm = vc.decode(payload)
            except ValueError:
                continue
            self.rx_t = time.time()
            self.rx_frames += 1
            self.rx_db = vc.level_db(pcm)
            if not self.listen:
                continue
            with self.lock:
                if self.out is None:
                    self.out, self.out_err = audiodev.open_output(self.backend)
                self.jbuf.append(vc.gain(pcm, self.volume / 100.0))
                if len(self.jbuf) >= JITTER_FRAMES or (self.out.pending() if hasattr(self.out, "pending") else 0):
                    for f in self.jbuf:
                        self.out.write(f)
                    self.jbuf = []

    # --- the Tk loop ----------------------------------------------------------------------------------------
    def tick(self):
        now = time.time()
        if not self._bind():
            return
        if now >= self.next_hello:
            self.next_hello = now + HELLO_S
            self._send(b"H", bytes([1 if self.listen else 0]))
        if self.mine and not self.pressing and not self.msg_busy and (self.speaker or {}).get("what") != "signal":
            self.release()                       # (the button went up while the grant was on its way)
        if self.page is not None:
            try:
                self.page.refresh()
            except Exception:
                self.page = None

    # --- what the screens show -----------------------------------------------------------------------------
    @staticmethod
    def crew_text(sp):
        return T(CREW_KEYS.get((sp or {}).get("crew"), "hail.crew_station"))

    def listening_now(self):
        return self.listen and time.time() - self.rx_t < LISTEN_STALE_S

    def view(self):
        """For the bars: None = no loudhailer on this link; else {"label", "talking", "enabled", "state", "text",
        "level", "listen"}. Colours: white / black info, amber CAUTION for a note - never red (13n1)."""
        h = self.gw()
        if h is None:
            return None
        talking = self.mine and (self.pressing or self.msg_busy)
        if self.speaker and not self.mine:
            text = T("hail.speaks", who=self.crew_text(self.speaker))
        elif talking:
            text = T("hail.you_talk")
        elif h.get("state") == "OFFLINE" or not h.get("out_ok"):
            text = T("hail.st_offline")
        elif self.info and not self.info.get("voice_ok", True):
            text = T("hail.weak")
        elif self.listening_now():
            text = T("hail.listen_level", bar=self.bar(self.rx_db))
        elif not self.listen:
            text = T("hail.muted")
        else:
            text = T("hail.no_audio")
        other = self.other_speaks()
        lvl = self.bar(self.rx_db, 3) if self.listening_now() else ""
        return {"label": T("hail.ptt"), "talking": talking,
                "other": other,
                "bar_label": T("hail.ptt"),      # screen 2: greyed while another speaks; who = the message line
                "listen_label": (T("hail.listen_btn") + " " + lvl).strip(), "enabled": self.available() and (not self.other_speaks() or self.captain_here()),
                "state": h.get("state"), "text": text, "level": vc.meter(self.rx_db), "listen": self.listen,
                "caution": bool(self.other_speaks() or h.get("state") != "ACTIVE" or
                                (self.info and not self.info.get("voice_ok", True)))}

    @staticmethod
    def bar(db, n=6):
        k = int(round(vc.meter(db) * n))
        return "▮" * k + "▯" * (n - k)

    def latency_ms(self):
        """Talk: the gateway's measured age of our frames; listen: half the round trip + the jitter buffer."""
        rtt = None
        st = getattr(self.app, "gw_status", None) or {}
        me = [s for s in st.get("stations", []) if s.get("id") == st.get("you")]
        if me:
            rtt = me[0].get("rtt_ms")
        up = (self.info or {}).get("up_ms")
        down = None if rtt is None else int(rtt / 2 + JITTER_FRAMES * vc.FRAME_MS)
        return up, down

    def close(self):
        self._unbind()


# --- the Tools page "Loudhailer" --------------------------------------------------------------------------------
class Page:
    def __init__(self, fr, app):
        import tkinter as tk
        self.tk = tk
        self.app, self.h = app, app.hail
        C = theme.C
        BG = C["bg"]
        F_TXT, F_MED = theme.font("label"), theme.font("button")
        self.F_TXT = F_TXT
        self.fr = fr
        i18n.bind(tk.Label(fr, font=theme.font("value"), bg=BG, fg=C["fg"]), "hail.title").pack(anchor="w", padx=6,
                                                                                               pady=(6, 2))
        i18n.bind(tk.Label(fr, font=F_TXT, bg=BG, fg=C["fg"], wraplength=640, justify="left"), "hail.note").pack(
            anchor="w", padx=6)
        self.state = tk.Label(fr, font=F_MED, bg=BG, fg=C["fg"], anchor="w", justify="left", wraplength=640)
        self.state.pack(fill="x", padx=6, pady=4)
        row = tk.Frame(fr, bg=BG)
        row.pack(anchor="w", padx=6, pady=4)
        self.ptt = tk.Button(row, width=9, **theme.button_opts())
        self.ptt.configure(font=theme.font("title"))
        i18n.bind(self.ptt, "hail.ptt")
        self.ptt.bind("<ButtonPress-1>", lambda _e: self.h.press())
        self.ptt.bind("<ButtonRelease-1>", lambda _e: self.h.release())
        self.ptt.pack(side="left", padx=(0, 12))
        self.lis = tk.BooleanVar(value=self.h.listen)
        i18n.bind(tk.Checkbutton(row, variable=self.lis, font=F_MED, bg=BG, fg=C["fg"], selectcolor=C["entry"],
                                 command=lambda: self.h.set_listen(self.lis.get())), "hail.listen").pack(side="left")
        i18n.bind(tk.Label(row, font=F_TXT, bg=BG, fg=C["fg"]), "hail.volume").pack(side="left", padx=(14, 2))
        vol = tk.Scale(row, from_=0, to=100, orient="horizontal", length=140, bg=BG, fg=C["fg"], highlightthickness=0,
                       command=lambda v: self.h.set_volume(int(float(v))))
        vol.set(self.h.volume)
        vol.pack(side="left")
        self.meter = tk.Canvas(row, width=180, height=18, bg=C["head"], highlightthickness=1,
                               highlightbackground=C["fg"])
        self.meter.pack(side="left", padx=10)
        self.lat = tk.Label(row, font=F_TXT, bg=BG, fg=C["fg"])
        self.lat.pack(side="left", padx=6)
        # sound signals (clearly labelled)
        row = tk.Frame(fr, bg=BG)
        row.pack(anchor="w", padx=6, pady=(8, 2))
        i18n.bind(tk.Label(row, font=F_MED, bg=BG, fg=C["fg"]), "hail.signals").pack(side="left", padx=(0, 8))
        for kind in ("danger", "long"):
            b = tk.Button(row, command=lambda k=kind: self.h.signal(k), **theme.button_opts())
            i18n.bind(b, "hail.sig_" + kind)
            b.pack(side="left", padx=4)
        # ready messages
        i18n.bind(tk.Label(fr, font=F_MED, bg=BG, fg=C["fg"]), "hail.messages").pack(anchor="w", padx=6, pady=(8, 2))
        self.mbox = tk.Frame(fr, bg=BG)
        self.mbox.pack(fill="x", padx=6)
        ed = tk.Frame(fr, bg=BG)
        ed.pack(fill="x", padx=6, pady=(6, 2))
        self.sel = tk.IntVar(value=-1)
        self.e_el, self.e_en = tk.StringVar(), tk.StringVar()
        for key, var in (("hail.msg_el", self.e_el), ("hail.msg_en", self.e_en)):
            r = tk.Frame(ed, bg=BG)
            r.pack(fill="x")
            i18n.bind(tk.Label(r, font=F_TXT, bg=BG, fg=C["fg"], width=10, anchor="w"), key).pack(side="left")
            tk.Entry(r, textvariable=var, font=F_TXT, width=46, bg=C["entry"], fg=C["fg"]).pack(side="left", fill="x")
        r = tk.Frame(ed, bg=BG)
        r.pack(anchor="w", pady=4)
        for key, fn in (("hail.msg_add", self.add), ("hail.msg_save", self.save), ("hail.msg_del", self.delete),
                        ("hail.msg_reset", self.reset)):
            b = tk.Button(r, command=fn, **theme.button_opts())
            i18n.bind(b, key)
            b.pack(side="left", padx=3)
        # the gateway's horn volume (HELM / on board), never above its limit
        row = tk.Frame(fr, bg=BG)
        row.pack(anchor="w", padx=6, pady=(10, 4))
        i18n.bind(tk.Label(row, font=F_TXT, bg=BG, fg=C["fg"]), "hail.horn_volume").pack(side="left")
        self.hv = tk.Scale(row, from_=0, to=100, orient="horizontal", length=160, bg=BG, fg=C["fg"],
                           highlightthickness=0)
        self.hv.pack(side="left")
        b = tk.Button(row, command=lambda: self.h.gw_volume(self.hv.get()), **theme.button_opts())
        i18n.bind(b, "hail.horn_set")
        b.pack(side="left", padx=4)
        self.limit = tk.Label(row, font=F_TXT, bg=BG, fg=C["fg"])
        self.limit.pack(side="left", padx=6)
        self.msgs = None
        self.fill()
        self.h.page = self
        i18n.on_change(self.lang_changed)

    def lang_changed(self):
        try:
            self.fill()                          # the ready messages in the new screen language
        except self.tk.TclError:
            raise RuntimeError("loudhailer page gone")     # (i18n drops this listener)

    def fill(self):
        tk = self.tk
        for w in self.mbox.winfo_children():
            w.destroy()
        lst = messages()
        self.msgs = lst
        for i, m in enumerate(lst):
            r = tk.Frame(self.mbox, bg=theme.C["bg"])
            r.pack(fill="x", pady=1)
            tk.Button(r, text=T("hail.msg_play"), command=lambda i=i: self.h.play_message(i),
                      **theme.button_opts()).pack(side="left")
            tk.Radiobutton(r, variable=self.sel, value=i, command=self.pick, bg=theme.C["bg"], fg=theme.C["fg"],
                           selectcolor=theme.C["entry"], font=self.F_TXT,
                           text=m.get(i18n.lang()) or m.get("el") or m.get("en")).pack(side="left", padx=4)
            # (the screen language only; both texts are in the edit fields below and both are spoken)

    def pick(self):
        i = self.sel.get()
        if 0 <= i < len(self.msgs):
            self.e_el.set(self.msgs[i]["el"])
            self.e_en.set(self.msgs[i]["en"])

    def add(self):
        lst = messages() + [{"el": self.e_el.get(), "en": self.e_en.get()}]
        save_messages(lst)
        self.fill()

    def save(self):
        i, lst = self.sel.get(), messages()
        if 0 <= i < len(lst):
            lst[i] = {"el": self.e_el.get(), "en": self.e_en.get()}
            save_messages(lst)
            self.fill()

    def delete(self):
        i, lst = self.sel.get(), messages()
        if 0 <= i < len(lst):
            del lst[i]
            save_messages(lst)
            self.sel.set(-1)
            self.fill()

    def reset(self):
        reset_messages()
        self.sel.set(-1)
        self.fill()

    def refresh(self):
        h = self.h
        v = h.view()
        g = h.gw() or {}
        if v is None:
            txt = T("hail.no_gateway")
        else:
            txt = T("hail.state", st=T("hail.st_" + str(g.get("state", "OFFLINE")).lower()),
                    why=i18n.from_en(g.get("why") or "") or "-", text=v["text"])
            if h.note:
                txt += "\n" + h.note
        self.state.configure(text=txt, fg=theme.CAUTION if v and v["caution"] else theme.C["fg"])
        talking = bool(v and v["talking"])
        self.ptt.configure(state="normal" if v and (v["enabled"] or talking) else "disabled",
                           bg=theme.WHITE if talking else theme.C["btn"], fg=theme.BLACK if talking else theme.C["btn_fg"])
        self.meter.delete("all")
        w = int(180 * vc.meter(h.rx_db if h.listening_now() else (h.tx_db if talking else -90.0)))
        self.meter.create_rectangle(0, 0, w, 18, fill=theme.WHITE, outline="")      # white (13n1 colour code)
        up, down = h.latency_ms()
        self.lat.configure(text=T("hail.latency", up="-" if up is None else up, down="-" if down is None else down))
        self.limit.configure(text=T("hail.horn_limit", pct=g.get("volume", "-"), limit=g.get("volume_max", "-")))


def build_page(fr, app):
    return Page(fr, app)
