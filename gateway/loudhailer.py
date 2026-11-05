"""Loudhailer + microphone on the boat (step 13r, owner 2026-11-02): the shore crew talks with any contact near
the USV (swimmers, fishing boats, coast guard, divers). NO VHF radio.

Hardware (PARTS.md / WIRING.md): a waterproof horn speaker 25-40 W IP67 + a 12 V class-D amplifier and an
outdoor microphone with a wind shield, on a USB sound card of the GATEWAY PC. Nothing on the AURIX: nothing here
sends a frame to it, nothing here is in a control path (tests LH9 / LH10).

  station --(UDP, signed, voicecodec "T")--> gateway --(pipe)--> loudhailer_worker.py --> amplifier + horn
  station <--(UDP, signed, "L")-- gateway <--(pipe)-- worker <-- boat microphone        (LISTEN)

PUSH-TO-TALK, half duplex: {"t": "ptt", "on": true} asks for the floor - the FIRST station wins (this class
decides) - EXCEPT the captain (owner W54, 2026-11-04): the captain's press (crew captain / single operator) takes the
horn AT ONCE from anyone talking (the displaced station gets "ptt_ack" ok false, "taken" true, "the captain took the
horn"; a sound signal already playing still plays to its end - a few seconds, a cut signal means nothing); the others
get "ptt_ack" ok false + who speaks, every station gets "hail_floor" (the current speaker
or nobody). While someone talks (and HANG_S after) the worker mutes the boat microphone. The floor ends with
{"on": false}, the station leaving, no voice frame for frame_gap_s, or talk_max_s.
Sound signals {"t": "hail_signal", "kind": "danger" (5 short) | "long" (1 prolonged)} take the floor for their
time. Ready messages are spoken by the station (its offline voice) and sent like talk (what = "message").
Volume {"t": "hail_volume", "pct"}: the HELM station or one on the boat; never above volume_max_pct (the
gateway's limit).

Link priority (owner: above the copilot's video, never above the captain's driving data / video): voice to /
from a station stops first when its link is weak (round trip > drop_rtt_ms, a TCP backlog > drop_backlog_kb,
the reduced telemetry because of the link) or with the data saver for everyone but the captain - with a note
("hail" message, why); the camera manager counts the voice as the captain's priority load (cameras.
update_priority), so the copilot's camera view is dropped before the voice.

Recording: the worker writes both directions into 1-minute WAV files <ssd>/video/ring/AUDIO/ (one track: the
shore's voice while talking, else the microphone); this class hands that folder to the recorder as the
"AUDIO" track, so every SAVE / KEEP 5' / 1h event holds it next to the cameras (same window, same timeline).

Health = a NOTICE only (never an alarm, never a mode change, never in GW_STATUS): ACTIVE, DEGRADED (only the
speaker or only the microphone works), OFFLINE (switched off, no sound card, the worker crashed / silent - it
is restarted every restart_s).
"""
import collections
import json
import os
import queue
import struct
import subprocess
import sys
import threading
import time

from common import log
import voicecodec as vc

HERE = os.path.dirname(os.path.abspath(__file__))
FIRST_FRAME_SLACK_S = 1.5       # review 13r LOW-3: the first voice frame after the grant may take this much longer
ACTIVE, DEGRADED, OFFLINE = "ACTIVE", "DEGRADED", "OFFLINE"

DEFAULTS = {
    "enabled": True,
    "backend": "auto",              # auto / alsa / winmm / sounddevice / sim (demo, tests) / null
    "out_dev": None, "in_dev": None,  # the USB sound card (ALSA "plughw:CARD=Device", a Windows device number)
    "python": None,                 # None = this Python
    "volume_max_pct": 80,           # the gateway's VOLUME LIMIT (the horn is loud: hearing, neighbours); a LINEAR
                                    # gain of the samples - the real loudness is set by the amplifier's trimmer
    "volume_pct": 70,
    "talk_max_s": 120.0,            # one press at most this long (a stuck button / mouse)
    "frame_gap_s": 1.0,             # no voice frame this long while holding the floor -> released
    "listen_s": 3.0,                # a station listens while its hello is younger than this
    "max_age_s": 1.0,               # older talk frames are dropped (stale)
    "record": True,                 # both directions into the ring (video/ring/AUDIO) + SAVE / KEEP events
    "drop_rtt_ms": 400.0,           # weak link -> voice off for that station (the driving data first)
    "drop_backlog_kb": 64,
    "restart_s": 5.0,
    "silent_s": 5.0,                # no status from the worker this long -> OFFLINE + restart
    "sim_reply_s": 0.0,             # sim: a 'contact' answers every N s (demo)
}


class AudioTrack:
    """The ring's AUDIO folder for gateway/recorder.py (duck-typed like CamRecorder): its 1-minute WAV files are
    linked into the SAVE / KEEP events like a camera's .ts files."""

    def __init__(self, folder, hail):
        self.name, self.folder, self.hail = "AUDIO", folder, hail
        self.state, self.restarts, self.since = "OFF", 0, time.time()

    @property
    def proc(self):
        return self.hail.proc                       # the newest file is still being written while it runs

    def segments(self):
        from loudhailer_worker import seg_time
        out = []
        try:
            for n in os.listdir(self.folder):
                t = seg_time(n)
                if t is not None:
                    out.append((t, os.path.join(self.folder, n)))
        except OSError:
            pass
        out.sort()
        return out

    def tick(self, now):
        st = "REC" if self.hail.state != OFFLINE and self.hail.worker.get("ring") else "DOWN"
        if st != self.state:
            self.state, self.since = st, now

    def stop(self):
        pass


class Loudhailer:
    def __init__(self, cfg, gw):
        from common import deep_merge
        self.cfg = deep_merge(DEFAULTS, cfg or {})
        try:                                        # review 13r LOW-8: the limit is a percentage, 0..100
            self.cfg["volume_max_pct"] = max(0, min(100, int(self.cfg["volume_max_pct"])))
        except (TypeError, ValueError):
            self.cfg["volume_max_pct"] = DEFAULTS["volume_max_pct"]
        self.gw = gw
        self.enabled = bool(self.cfg["enabled"])
        self.state, self.why = OFFLINE, "switched off" if not self.enabled else "starting"
        self.proc = None
        self.wq = queue.Queue(maxsize=60)           # to the worker (a full queue drops: never blocks the loop)
        self.loop = None
        self.worker = {}                            # the worker's last status
        self.worker_t = 0.0
        self.next_start = 0.0
        self.spawning = False
        self.starts = 0
        self.floor = None                           # {"id", "user", "crew", "what", "t0", "last"}
        self.peers = {}                             # station id -> {"addr", "until", "seq", "up_ms", "why", ...}
        try:                                        # review W50s/13r LOW-4: 0..limit (a negative value made the worker's
            vol = int(self.cfg["volume_pct"])       # clamped status differ for ever = endless resends)
        except (TypeError, ValueError):
            vol = DEFAULTS["volume_pct"]
        self.volume = max(0, min(vol, int(self.cfg["volume_max_pct"])))
        self.n = collections.Counter()              # talk_in, talk_other, talk_stale, listen_out, listen_drop, ...
        self.bytes = collections.deque()            # (t, bytes) of voice to / from stations (kbps)
        self.next_info = 0.0
        self.history = collections.deque(maxlen=50)   # (time, state, why): every health change (status / tests)
        self.track = None
        st = getattr(gw, "storage", None)
        if self.enabled and self.cfg["record"] and st is not None and st.usable:
            self.track = AudioTrack(os.path.join(st.root, "video", "ring", "AUDIO"), self)

    # --- the worker process --------------------------------------------------------------------------------------
    def start(self, loop):
        self.loop = loop
        if self.enabled:
            self._spawn_bg()

    def _spawn_bg(self):
        """Start the audio process in a helper thread: a process start can take tens of ms on Windows - never on
        the event loop (the captain's joystick packets go through it)."""
        if self.spawning:
            return
        self.spawning = True
        self.next_start = time.time() + float(self.cfg["restart_s"])

        def run():
            try:
                self._spawn()
            finally:
                self.spawning = False
        threading.Thread(target=run, daemon=True).start()

    def _spawn(self):
        self.next_start = time.time() + float(self.cfg["restart_s"])
        c = self.cfg
        cmd = [c["python"] or sys.executable, os.path.join(HERE, "loudhailer_worker.py"), "--backend", c["backend"],
               "--volume-max", str(int(c["volume_max_pct"])), "--volume", str(self.volume),
               "--sim-reply-s", str(float(c["sim_reply_s"]))]
        if c["out_dev"] is not None:
            cmd += ["--out-dev", str(c["out_dev"])]
        if c["in_dev"] is not None:
            cmd += ["--in-dev", str(c["in_dev"])]
        if self.track is not None:
            cmd += ["--ring", self.track.folder, "--seg-s", str(int(self.gw.rec.seg_s if getattr(self.gw, "rec", None)
                                                                     else 60))]
        err = subprocess.DEVNULL
        st = getattr(self.gw, "storage", None)
        try:
            if st is not None and st.usable:
                err = open(os.path.join(st.folder("logs"), "loudhailer_worker.log"), "ab")
        except OSError:
            err = subprocess.DEVNULL
        try:
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, bufsize=0, cwd=HERE)
        except OSError as e:
            if self.loop is not None:
                self.loop.call_soon_threadsafe(self._health, OFFLINE, "the audio process did not start: %s" % e)
            return
        finally:
            if err is not subprocess.DEVNULL:
                err.close()                                 # (the child has its own handle)
        q = queue.Queue(maxsize=60)
        self.wq = q
        self.worker_t = time.time()
        self.starts += 1
        self.proc = p
        threading.Thread(target=self._reader, args=(p,), daemon=True).start()
        threading.Thread(target=self._writer, args=(p, q), daemon=True).start()
        log.info("loudhailer: audio process started (%s)", c["backend"])

    def _reader(self, p):
        """Worker stdout -> the event loop (call_soon_threadsafe): never reads on the loop."""
        f = p.stdout
        try:
            while True:
                h = f.read(5)
                if len(h) < 5:
                    break
                kind, n = h[:1], struct.unpack("<I", h[1:])[0]
                data = f.read(n) if n else b""
                if len(data) < n:
                    break
                if self.loop is not None:
                    self.loop.call_soon_threadsafe(self._from_worker, p, kind, data)
        except (OSError, ValueError, RuntimeError):
            pass

    def _writer(self, p, q):
        try:
            while True:
                item = q.get()
                if item is None:
                    break
                p.stdin.write(item)
        except (OSError, ValueError):
            pass

    def to_worker(self, kind, data):
        if self.proc is None:
            return False
        try:
            self.wq.put_nowait(kind + struct.pack("<I", len(data)) + data)
            return True
        except queue.Full:
            self.n["to_worker_drop"] += 1
            return False

    def control(self, obj):
        return self.to_worker(b"J", json.dumps(obj).encode("utf-8"))

    def _from_worker(self, p, kind, data):
        if p is not self.proc:
            return
        if kind == b"M":
            self._mic(data)
        elif kind == b"J":
            try:
                m = json.loads(data.decode("utf-8"))
            except ValueError:
                return
            if m.get("t") == "status":
                self.worker, self.worker_t = m, time.time()
                # review 13r LOW-2: a control frame lost (full queue) or a restarted worker: send the state again
                if bool(m.get("talk")) != bool(self.floor):
                    self.n["talk_resent"] += 1
                    self.control({"talk": bool(self.floor)})
                if m.get("volume") is not None and m.get("volume") != self.volume:
                    self.control({"volume": self.volume})
            elif m.get("t") == "signal_done" and self.floor and self.floor.get("what") == "signal":
                self._release("signal played")

    def stop(self, wait=True):
        """wait=False (from the event loop): never waits - a hung process is killed."""
        p, self.proc = self.proc, None
        self.worker = {}                            # review 13r LOW-4: never a dead worker's last status
        if p is None:
            return
        try:
            self.wq.put_nowait(b"J" + struct.pack("<I", 14) + b'{"stop": true}')
            self.wq.put_nowait(None)
        except queue.Full:
            pass
        try:
            if wait:
                p.wait(1.5)
            elif p.poll() is None:
                p.kill()
        except subprocess.TimeoutExpired:
            p.kill()
        except OSError:
            pass

    # --- health (a notice only) --------------------------------------------------------------------------------
    def _health(self, state, why):
        if (state, why) != (self.state, self.why):
            old = self.state
            self.state, self.why = state, why
            (log.warning if state != ACTIVE else log.info)("loudhailer %s (%s)", state, why)
            if state != old:
                self.history.append((time.time(), state, why))
                self.gw.audit("hail_state", state=state, why=why)
            if state == OFFLINE and self.floor:
                self._release("loudhailer offline")

    def tick(self, now=None):
        """4 Hz from the event loop: worker alive, health, floor timeouts, the per-station voice notes."""
        now = time.time() if now is None else now
        if not self.enabled:
            return
        p = self.proc
        if p is not None and (p.poll() is not None or now - self.worker_t > float(self.cfg["silent_s"])):
            dead = p.poll() is not None
            self.stop(wait=False)
            self._health(OFFLINE, "the audio process %s - restarting" % ("ended" if dead else "is silent"))
        if self.proc is None and now >= self.next_start:
            self._spawn_bg()
        w = self.worker if self.proc is not None else {}
        if self.proc is not None and w:
            if w.get("out_ok") and w.get("in_ok"):
                self._health(ACTIVE, w.get("error") or "")
            elif w.get("out_ok") or w.get("in_ok"):
                self._health(DEGRADED, ("no microphone" if w.get("out_ok") else "no speaker") +
                             (": " + w["error"] if w.get("error") else ""))
            else:
                self._health(OFFLINE, "no sound card" + (": " + w["error"] if w.get("error") else ""))
        f = self.floor
        if f:
            if f["id"] not in self.gw.stations.stations:
                self._release("the station left")
            elif f["what"] == "signal":
                if now - f["t0"] > f.get("max_s", 10.0):
                    self._release("signal time over")
            elif now - f["last"] > float(self.cfg["frame_gap_s"]) + (0.0 if f.get("got") else FIRST_FRAME_SLACK_S):
                self._release("no voice for %.1f s" % float(self.cfg["frame_gap_s"]))   # (LOW-3: + slack before the first)
            elif now - f["t0"] > float(self.cfg["talk_max_s"]):
                self._release("talk time limit %.0f s" % float(self.cfg["talk_max_s"]))
        for sid in [s for s in self.peers if s not in self.gw.stations.stations]:
            del self.peers[sid]
        if now >= self.next_info:
            self.next_info = now + 1.0
            for st in list(self.gw.stations.stations.values()):
                ok, why = self.voice_ok(st)
                pe = self.peers.get(st.id, {})
                st.send({"t": "hail", "voice_ok": ok, "why": why, "up_ms": pe.get("up_ms"),
                         "listening": now < pe.get("until", 0.0), "state": self.state})

    # --- link priority -----------------------------------------------------------------------------------------
    def is_captain(self, st):
        cams = getattr(self.gw, "cams", None)
        if cams is not None and hasattr(cams, "is_priority"):
            return cams.is_priority(st.id)
        return st.crew == "captain" or st.single_op

    def voice_ok(self, st):
        """Voice to / from this station now? -> (ok, why in English). The driving data + the captain's video first."""
        c = self.cfg
        if st.rtt_ms is not None and st.rtt_ms > float(c["drop_rtt_ms"]):
            return False, "weak link (%.0f ms): voice off - the driving data first" % st.rtt_ms
        if st.backlog() > int(c["drop_backlog_kb"]) * 1024:
            return False, "link busy: voice off - the driving data first"
        if st.tmode == "lite" and st.tmode_why not in ("chosen by the station", "data saver"):
            return False, "reduced telemetry (%s): voice off" % st.tmode_why
        saver = self.gw.data_saver() if hasattr(self.gw, "data_saver") else False
        if saver and not self.is_captain(st):
            return False, "data saver: voice for the captain only"
        return True, ""

    def kbps(self, now=None):
        now = time.time() if now is None else now
        while self.bytes and now - self.bytes[0][0] > 2.0:
            self.bytes.popleft()
        return sum(b for _t, b in self.bytes) * 8 / 2000.0

    # --- the floor (push-to-talk) -------------------------------------------------------------------------------
    @staticmethod
    def crew_of(st):
        return st.crew or ("single" if st.single_op else "station")

    def speaker(self):
        f = self.floor
        return None if not f else {"id": f["id"], "user": f["user"], "crew": f["crew"], "what": f["what"],
                                   "for_s": round(time.time() - f["t0"], 1)}

    def _broadcast(self):
        msg = {"t": "hail_floor", "speaker": self.speaker()}
        for st in list(self.gw.stations.stations.values()):
            st.send(msg)

    def _grant(self, st, what, max_s=None):
        now = time.time()
        self.floor = {"id": st.id, "user": st.user, "crew": self.crew_of(st), "what": what, "t0": now, "last": now}
        if max_s:
            self.floor["max_s"] = max_s
        self.control({"talk": True})
        self.gw.audit("hail_talk", on=True, station=st.id, user=st.user, crew=self.floor["crew"], what=what)
        self._broadcast()

    def _release(self, why):
        f, self.floor = self.floor, None
        if not f:
            return
        self.control({"talk": False})
        self.gw.audit("hail_talk", on=False, station=f["id"], user=f["user"], what=f["what"], why=why,
                      seconds=round(time.time() - f["t0"], 1))
        self._broadcast()

    def _may_talk(self, st):
        if st.role == "VIEWER":
            return "VIEWER: watch only"
        if not self.enabled:
            return "no loudhailer on this boat"
        if self.state == OFFLINE or not self.worker.get("out_ok"):
            return "loudhailer offline (%s)" % (self.why or "-")
        ok, why = self.voice_ok(st)
        if not ok:
            return why
        return None

    def ptt(self, st, m):
        """{"t": "ptt", "on": bool, "what": "talk" | "message"}: the first station wins the floor."""
        on = bool(m.get("on"))
        what = "message" if m.get("what") == "message" else "talk"
        if not on:
            if self.floor and self.floor["id"] == st.id and self.floor["what"] != "signal":   # a signal plays to its end
                self._release("released")
            return st.send({"t": "ptt_ack", "ok": True, "on": False, "speaker": self.speaker()})
        why = self._may_talk(st)
        take = (why is None and self.floor is not None and self.floor["id"] != st.id and
                self.floor["what"] != "signal" and self.floor_captain(st))
        if why is None and self.floor and self.floor["id"] != st.id and not take:
            why = "%s speaks" % self.floor["crew"]
        if why:
            self.n["ptt_refused"] += 1
            self.gw.audit("hail_refused", station=st.id, user=st.user, why=why)
            return st.send({"t": "ptt_ack", "ok": False, "on": True, "text": why, "speaker": self.speaker()})
        if take:                                         # owner W54: the captain takes the horn at once
            self._take_over(st)
        if not self.floor:
            self._grant(st, what)
        st.send({"t": "ptt_ack", "ok": True, "on": True, "speaker": self.speaker()})

    @staticmethod
    def floor_captain(st):
        """Owner W54 (2026-11-04): the captain (crew captain / a single operator) takes the horn over from anyone."""
        return st.role != "VIEWER" and (st.crew == "captain" or bool(st.single_op))

    def _take_over(self, st):
        f = self.floor
        old = self.gw.stations.stations.get(f["id"])
        self.n["ptt_taken"] += 1
        self.gw.audit("hail_taken", station=st.id, user=st.user, from_station=f["id"], from_user=f["user"])
        self._release("the captain took the horn")
        if old is not None:
            old.send({"t": "ptt_ack", "ok": False, "on": True, "taken": True, "text": "the captain took the horn",
                      "speaker": None})

    def signal(self, st, m):
        """{"t": "hail_signal", "kind": "danger" | "long"}: a sound signal through the horn (takes the floor)."""
        kind = m.get("kind")
        why = None if kind in vc.SIGNALS else "unknown sound signal"
        why = why or self._may_talk(st)
        if why is None and self.floor and self.floor["id"] != st.id:
            why = "%s speaks" % self.floor["crew"]
        if why:
            self.n["ptt_refused"] += 1
            return st.send({"t": "hail_signal_ack", "ok": False, "text": why, "kind": kind})
        if self.floor:
            self._release("a sound signal")
        self._grant(st, "signal", max_s=vc.signal_s(kind) + 2.0)
        self.control({"signal": kind})
        self.gw.audit("hail_signal", station=st.id, user=st.user, signal=kind)
        st.send({"t": "hail_signal_ack", "ok": True, "kind": kind})

    def set_volume(self, st, m):
        """{"t": "hail_volume", "pct"}: the HELM station or a station on the boat; clamped to the volume limit."""
        if st.role == "VIEWER" or not (st.local or (self.gw.telem2 or {}).get("helm") == st.id):
            return st.send({"t": "hail_volume_ack", "ok": False, "text": "volume: HELM or a station on the boat",
                            "pct": self.volume})
        try:
            pct = int(m.get("pct"))
        except (TypeError, ValueError):
            return st.send({"t": "hail_volume_ack", "ok": False, "text": "bad volume", "pct": self.volume})
        lim = int(self.cfg["volume_max_pct"])
        self.volume = max(0, min(lim, pct))
        self.control({"volume": self.volume})
        self.gw.audit("hail_volume", station=st.id, user=st.user, asked=pct, pct=self.volume)
        st.send({"t": "hail_volume_ack", "ok": True, "pct": self.volume, "limit": lim,
                 "text": "volume %d %%" % self.volume + (" (the limit is %d %%)" % lim if pct > lim else "")})

    def station_gone(self, st):
        self.peers.pop(st.id, None)
        if self.floor and self.floor["id"] == st.id:
            self._release("the station left")

    # --- voice packets (UDP, on the loop: light work only - the codec runs in the worker) ----------------------
    def udp(self, data, addr):
        p = vc.peek(data)
        if p is None:
            self.n["bad"] += 1
            return
        kind, sess, seq, ts, payload = p
        st = next((s for s in self.gw.stations.stations.values() if vc.session_bytes(s.session) == sess), None)
        if st is None or not vc.verify(data, st.udp_key):
            self.n["bad"] += 1
            return
        now = time.time()
        pe = self.peers.setdefault(st.id, {"until": 0.0, "seq_in": None, "seq_out": 0})
        if kind == b"H":
            pe["addr"] = addr
            pe["until"] = now + float(self.cfg["listen_s"]) if payload[:1] and payload[0] & 1 else 0.0
            return
        if kind != b"T":
            return
        f = self.floor
        if not f or f["id"] != st.id or f["what"] == "signal":
            self.n["talk_other"] += 1               # not the speaker: dropped (first wins)
            return
        if not vc.seq_newer(seq, pe["seq_in"]):
            return
        pe["seq_in"] = seq
        age = self.gw.stations.age(st, ts)
        pe["up_ms"] = int(max(0.0, age) * 1000)
        if age > float(self.cfg["max_age_s"]):
            self.n["talk_stale"] += 1
            return
        if len(payload) != vc.FRAME_BYTES:
            self.n["bad"] += 1
            return
        ok, _why = self.voice_ok(st)
        if not ok:
            self.n["talk_weak"] += 1
            return
        f["last"] = now
        f["got"] = True
        self.bytes.append((now, len(data) + 28))
        if self.to_worker(b"A", payload):
            self.n["talk_in"] += 1

    def _mic(self, frame):
        """One microphone frame (ADPCM) from the worker -> every listening station whose link allows it."""
        if self.floor:
            return                                   # (the worker mutes it already: twice safe)
        udp = getattr(self.gw.stations, "udp", None)
        if udp is None:
            return
        now = time.time()
        for sid, pe in list(self.peers.items()):
            st = self.gw.stations.stations.get(sid)
            if st is None or now >= pe.get("until", 0.0) or "addr" not in pe:
                continue
            ok, why = self.voice_ok(st)
            if not ok:
                self.n["listen_drop"] += 1
                pe["why"] = why
                continue
            pe["seq_out"] = (pe["seq_out"] + 1) & 0xFFFF
            pkt = vc.pack(b"L", vc.session_bytes(st.session), pe["seq_out"], now, frame, st.udp_key)
            try:
                udp.sendto(pkt, pe["addr"])
                self.n["listen_out"] += 1
                self.bytes.append((now, len(pkt) + 28))
            except OSError:
                self.n["listen_drop"] += 1

    # --- status -------------------------------------------------------------------------------------------------
    def status(self):
        w = self.worker if self.proc is not None else {}
        now = time.time()
        return {"enabled": self.enabled, "state": self.state, "why": self.why, "backend": self.cfg["backend"],
                "speaker": self.speaker(), "volume": self.volume, "volume_max": int(self.cfg["volume_max_pct"]),
                "mic_db": w.get("mic_db"), "out_db": w.get("out_db"), "out_ok": bool(w.get("out_ok")),
                "in_ok": bool(w.get("in_ok")), "recording": bool(self.track and w.get("ring")),
                "listeners": sum(1 for pe in self.peers.values() if now < pe.get("until", 0.0)),
                "kbps": round(self.kbps(now), 1), "codec": "ADPCM %.0f kbit/s" % vc.KBPS,
                "talk_max_s": self.cfg["talk_max_s"], "counts": dict(self.n), "starts": self.starts}
