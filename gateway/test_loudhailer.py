"""Loudhailer + boat microphone (step 13r, owner 2026-11-02): the real gateway (loudhailer.py + its own audio
process loudhailer_worker.py with the SIMULATED speaker / microphone) and real ground-station voice clients
(groundstation/hailer.py) on the simulated AURIX.

  LH1  PUSH-TO-TALK one at a time: the first station wins, the other is refused + sees who speaks (every station
       gets the floor), a VIEWER never talks, the floor back on release
  LH2  half duplex: the boat microphone is muted while someone talks (no frame to the listeners, no echo of the
       horn), back after the release; LISTEN delivers the mic to a listening station only
  LH3  recording: both directions in the ring (video/ring/AUDIO, 1-minute WAV) and inside a SAVE event + a KEEP 5'
       (the shore's voice + the microphone on the camera timeline)
  LH4  link priority: a weak link (round trip) / data saver -> voice off for that station with a note (talk refused,
       no listen frames), the captain keeps it with the data saver; the voice counts with the captain's video so the
       copilot's camera view drops first (cameras.update_priority)
  LH5  no effect on driving: the same AURIX frame types with and without voice, the mode / alarms unchanged, no
       loudhailer module sends to the AURIX, the event loop stays fast while voice flows
  LH6  the audio process killed = OFFLINE notice only (no alarm, no GW_STATUS flag, the floor released), restarted
  LH7  volume limit: never above volume_max_pct, only HELM / a station on the boat may set it
  LH8  the floor ends by itself: no voice frames (frame gap), the talk time limit, the speaker leaving
  LH9  sound signals (5 short / 1 prolonged) take the floor for their time; another station is refused meanwhile
  LH10 bad packets: another session's key, an old seq, a stale time, a wrong size, frames of a non-speaker = dropped
  LH11 ready message: spoken by the station (its voice; here injected samples) into the floor, then released
  LH12 (review 13r MED-1) a SAVE's AUDIO .wav is uploaded like its .ts (sync class 4), a KEEP's is not
  LH13 (review 13r MED-3) the AUDIO track keeps the clock (a frame every step, gaps filled with silence)
  LH14 (review 13r LOW-1 / LOW-8 / LOW-4) no out_dev / in_dev = not configured; volume limit 0..100
  LH15 (review 13r LOW-2 / LOW-3) the talk state resent to the worker; the first voice frame gets a slack
  LH14b (review of 13r-r LOW-3 / LOW-4) '' / 'default' devices = not configured; a negative volume_pct = 0
  LH16 (review of 13r-r MED-A) 80-500 ms stalls caught up back to back: the AUDIO track stays on the clock
  LH17 (review of 13r-r MED-B / LOW-5) a restart mid-minute never truncates the file; the clock set back rolls
  LH18 (review of 38add4b LOW-2) the catch-up frames after a stall keep their real audio inside the stall
  LH19 (U2 review LOW-15) a file opened mid-minute: silence first, the first frame at its own time

Run:  python test_loudhailer.py      (exit code 0 = all passed)
"""
import ast
import io
import os
import shutil
import sys
import tempfile
import threading
import time
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="trygons_lh_set_"), "gs_settings.json")
os.environ.setdefault("TRYGONS_TEST", "1")
import test_gateway as tg  # noqa: E402  (Sim, GwRunner, gw_cfg_b, users)
from common import proto, setup_logging  # noqa: E402
sys.path.insert(0, os.path.join(HERE, "..", "groundstation"))
from gwclient import GatewayLink  # noqa: E402
import hailer  # noqa: E402
import voicecodec as vc  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
results = []


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def until(cond, timeout, step=0.05, pump=()):
    end = time.time() + timeout
    while time.time() < end:
        for p in pump:
            p.pump()
        v = cond()
        if v:
            return v
        time.sleep(step)
    return None


class Station:
    """A ground station: its gateway link + its loudhailer client (null sound devices: nothing on this PC's card)."""

    def __init__(self, port, user, crew=None, local=False):
        self.link = GatewayLink("127.0.0.1:%d" % port, user, password=tg.PW[user], station=user,
                                local_addr="127.0.0.2" if local else "127.0.0.1", crew=crew)
        self.welcome = self.link.start()
        self.gw_status = None
        self.said = []
        self.msgs = []
        self.hail = hailer.Hailer(self, backend="null")
        self.hail.listen_local = True                  # (each station its own LISTEN switch in this one process)

    def say(self, text, colour=None):
        self.said.append(text)

    def pump(self):
        while True:
            try:
                kind, d = self.link.events.get_nowait()
            except Exception:
                break
            if kind == "gw_status":
                self.gw_status = d
            elif kind in ("hail_floor", "ptt_ack", "hail", "hail_signal_ack", "hail_volume_ack"):
                self.msgs.append((kind, d))
                self.hail.on_msg(kind, d)
        self.hail.tick()

    def last(self, kind):
        for k, d in reversed(self.msgs):
            if k == kind:
                return d
        return None

    def close(self):
        self.hail.close()
        self.link.stop()


def wait(s, pumps):
    """Sleep while every station keeps its link alive (hello / messages)."""
    end = time.time() + s
    while time.time() < end:
        for p in pumps:
            p.pump()
        time.sleep(0.05)


def wav_levels(path):
    with wave.open(path, "rb") as w:
        import array
        a = array.array("h", w.readframes(w.getnframes()))
    return [vc.level_db(a[i:i + vc.FRAME_N]) for i in range(0, len(a) - vc.FRAME_N, vc.FRAME_N)]


def lh12_15():
    """Unit checks of the review fixes of 13r (no simulator)."""
    import argparse
    import types
    import json as _json
    import sync as syncmod
    import recorder as recmod
    import loudhailer as lhmod
    import loudhailer_worker as lw
    import array as array_mod
    tmp = tempfile.mkdtemp(prefix="gwtest_lh12_")
    try:
        # LH12 the sync classes of a SAVE's / a KEEP's files
        for eid, keep in (("20261104-100000_save", False), ("20261104-100500_keep5", True)):
            os.makedirs(os.path.join(tmp, "video", "events", eid, "AUDIO"))
            with open(os.path.join(tmp, "video", "events", eid, "event.json"), "w") as f:
                _json.dump({"state": "complete", "keep": keep}, f)
        sc = object.__new__(syncmod.SyncClient)
        sc.root, sc._inc = tmp, {}
        files = {"save_wav": "20261104-100000_save/AUDIO/20261104-100000.wav",
                 "save_ts": "20261104-100000_save/FRONT-1/20261104-100000.ts",
                 "keep_wav": "20261104-100500_keep5/AUDIO/20261104-100500.wav",
                 "keep_json": "20261104-100500_keep5/event.json"}
        cls = {k: sc.classify("video/events/" + v, None, None) for k, v in files.items()}
        sv = os.path.join(tmp, "video", "events", "20261104-100000_save")
        open(os.path.join(sv, "AUDIO", "20261104-100000.wav"), "wb").close()
        before = recmod.event_synced(sv)
        for rel in ("event.json", os.path.join("AUDIO", "20261104-100000.wav")):
            open(os.path.join(sv, rel) + ".synced", "w").close()      # what the shore PC's confirmation writes
        freed = recmod.event_synced(sv)
        report("LH12 (review 13r MED-1) a SAVE's AUDIO .wav is uploaded like its video (class 4), a KEEP's stays on the "
               "boat; once uploaded the SAVE counts as synced (the housekeeping may free it)",
               cls == {"save_wav": 4, "save_ts": 4, "keep_wav": None, "keep_json": 1} and not before and freed,
               "classes %s; synced before %s, after the upload %s" % (cls, before, freed))

        # LH13 the AUDIO ring on the clock
        clk = [1793000020.0]                               # 40 s into its minute (:59:40)
        a = argparse.Namespace(backend="sim", out_dev=None, in_dev=None, ring=os.path.join(tmp, "ring"), seg_s=60,
                               volume_max=80, volume=70, sim_reply_s=0.0)
        w = lw.Worker(a, io.BytesIO(), clock=lambda: clk[0])
        w.lock_out = threading.Lock()
        w.control({"talk": True})                          # the floor is given: nothing arrived yet (prebuffer)
        for _k in range(25):                               # 1 s with talk on and no frame played
            w.step(clk[0])
            clk[0] += 0.04
        w.queue.append(vc.encode(vc.tone(500, 40)))        # one frame: below the prebuffer, still waiting
        for _k in range(25):
            w.step(clk[0])
            clk[0] += 0.04
        clk[0] += 2.0                                      # the loop was 2 s late (a busy PC)
        for _k in range(25):
            w.step(clk[0])
            clk[0] += 0.04
        w.ring.close()
        fn_ = os.listdir(a.ring)[0]
        with wave.open(os.path.join(a.ring, fn_), "rb") as wf:
            n = wf.getnframes()
        want = int((clk[0] - lw.seg_time(fn_)) * vc.RATE)
        report("LH13 (review 13r MED-3) the AUDIO track keeps the clock: a frame every 40 ms step also while the voice "
               "waits (prebuffer / underrun), a 2 s late loop filled with silence, a file opened 40 s into its minute "
               "starts with 40 s of silence (name = its first sample's time)",
               abs(n - want) <= vc.FRAME_N and w.ring.filled >= int(41.9 * vc.RATE) and
               lw.seg_time(fn_) == 1793000020.0 // 60 * 60,
               "file %s: %d samples, the clock says %d; silence filled %.2f s" % (fn_, n, want, w.ring.filled / vc.RATE))

        # LH16 (review of 13r-r MED-A) 80-500 ms stalls: run() catches the missed steps up back to back (it resets
        # its schedule only > 0.5 s behind) - the gap is filled once, the catch-up frames must not be written again
        clk = [1793000100.0]                               # a minute start
        a16 = argparse.Namespace(backend="sim", out_dev=None, in_dev=None, ring=os.path.join(tmp, "ring16"), seg_s=60,
                                 volume_max=80, volume=70, sim_reply_s=0.0)
        w16 = lw.Worker(a16, io.BytesIO(), clock=lambda: clk[0])
        w16.lock_out = threading.Lock()
        for _k in range(25):
            w16.step(clk[0])
            clk[0] += 0.04
        for stall in (0.08, 0.12, 0.2, 0.3, 0.45, 0.1, 0.25, 0.5, 0.15, 0.4):
            clk[0] += stall                                # the loop was late by this much
            for _k in range(int(stall / 0.04) + 1):        # the missed steps run at once (a few ms apart)
                w16.step(clk[0])
                clk[0] += 0.002
            for _k in range(10):                           # back on the 40 ms schedule
                w16.step(clk[0])
                clk[0] += 0.04
        w16.ring.close()
        f16 = os.path.join(a16.ring, os.listdir(a16.ring)[0])
        with wave.open(f16, "rb") as wf:
            n16 = wf.getnframes()
        want16 = int((clk[0] - lw.seg_time(os.path.basename(f16))) * vc.RATE)
        rec16, drop16 = getattr(w16.ring, 'recovered', 0), getattr(w16.ring, 'dropped', 0)
        report("LH16 (review of 13r-r MED-A; 38add4b LOW-2) ten 80-500 ms stalls caught up back to back: the AUDIO "
               "track stays on the clock (the stall is counted once: the catch-up frames go INTO the stall's silence, "
               "never after it)",
               abs(n16 - want16) <= vc.FRAME_N and rec16 > 0,
               "%d samples, the clock says %d (%.3f s apart); %d catch-up frames into their gap, %d dropped" % (
                   n16, want16, (n16 - want16) / float(vc.RATE), rec16, drop16))

        # LH18 (review of 38add4b LOW-2) the catch-up frames are REAL audio: kept, in order, inside their own gap
        r18 = lw.WavRing(os.path.join(tmp, "ring18"), 60)
        t = 1793000400.0                                   # a minute start
        quiet = vc.silence()
        for _k in range(25):                               # 1 s of quiet on the clock
            r18.write(quiet, t)
            t += 0.04
        tone = vc.frames(vc.tone(800, 400))                # 0.4 s of tone = 10 frames, all late (a 0.4 s stall)
        t += 0.4
        for fr in tone:                                    # caught up back to back (a few ms apart)
            r18.write(fr, t)
            t += 0.002
        t = 1793000400.0 + 1.4
        for _k in range(25):                               # back on the clock
            r18.write(quiet, t)
            t += 0.04
        r18.close()
        with wave.open(os.path.join(tmp, "ring18", os.listdir(os.path.join(tmp, "ring18"))[0]), "rb") as wf:
            n18 = wf.getnframes()
            import array as _arr
            a18 = _arr.array("h", wf.readframes(n18))
        seg = a18[int(1.0 * vc.RATE):int(1.4 * vc.RATE)]  # where the stall was: the tone must be there
        loud = sum(1 for k in range(0, len(seg), vc.FRAME_N) if max(abs(x) for x in seg[k:k + vc.FRAME_N]) > 1000)
        after = max(abs(x) for x in a18[int(1.45 * vc.RATE):]) if n18 > int(1.45 * vc.RATE) else 0
        report("LH18 (review of 38add4b LOW-2) a 0.4 s stall caught up: the catch-up frames' REAL audio is kept, in "
               "order inside the stall (not silence + dropped), the file still on the clock",
               loud >= 9 and after < 1000 and abs(n18 - int(2.4 * vc.RATE)) <= vc.FRAME_N and r18.dropped <= 1,   # (the first on-clock frame may still be ahead)
               "%d of 10 tone frames heard in the stall, after it %d, %d samples (clock %d), dropped %d, kept %d" % (
                   loud, after, n18, int(2.4 * vc.RATE), r18.dropped, r18.recovered))

        # LH19 (U2 review LOW-15) a file opened in the middle of its minute: silence first, the first frame at ITS time
        r19 = lw.WavRing(os.path.join(tmp, "ring19"), 60)
        t19 = 1793000460.0 + 30.0                          # 30 s into the minute
        tone19 = vc.frames(vc.tone(900, 200))
        for k, fr in enumerate(tone19):
            r19.write(fr, t19 + 0.04 * k)
        r19.close()
        with wave.open(os.path.join(tmp, "ring19", os.listdir(os.path.join(tmp, "ring19"))[0]), "rb") as wf:
            n19 = wf.getnframes()
            a19 = array_mod.array("h", wf.readframes(n19))
        early = max(abs(x) for x in a19[:int(29.5 * vc.RATE)])
        at = max(abs(x) for x in a19[int(30.0 * vc.RATE):int(30.2 * vc.RATE)])
        report("LH19 (U2 review LOW-15) a file opened / continued in the middle of its minute: the silence comes first "
               "and the first frame lands at its own time (never up to 60 s early in the evidence audio)",
               early < 100 and at > 1000 and abs(n19 - int(30.2 * vc.RATE)) <= vc.FRAME_N,
               "before 29.5 s max %d, at 30.0-30.2 s max %d, %d samples (clock %d)" % (early, at, n19,
                                                                                   int(30.2 * vc.RATE)))

        # LH17 (review of 13r-r MED-B / LOW-5) a restart mid-minute never truncates; the clock set back rolls / stops
        rdir = os.path.join(tmp, "ring17")
        r1 = lw.WavRing(rdir, 60)
        t = 1793000160.0                                   # a minute start
        tone = vc.frames(vc.tone(700, 2000))               # 2 s of a tone, then quiet frames
        for k in range(250):                               # 10 s
            r1.write(tone[k] if k < len(tone) else vc.silence(), t)
            t += 0.04
        r1._header()
        r1.f.flush()
        name17 = os.path.basename(r1.f.name)
        n_before = r1.n
        r1.f.close()                                       # the worker killed (no close(), the header of 1 s ago)
        r1.f = None
        t += 5.0                                           # the new worker 5 s later, same minute
        r2 = lw.WavRing(rdir, 60)
        for _k in range(25):
            r2.write(vc.silence(), t)
            t += 0.04
        r2.close()
        with wave.open(os.path.join(rdir, name17), "rb") as wf:
            n17 = wf.getnframes()
            head = array_mod.array("h", wf.readframes(vc.FRAME_N * 10))
        tone_kept = max(abs(x) for x in head) > 1000
        want17 = int((t - lw.seg_time(name17)) * vc.RATE)
        # the clock set back: 20 s inside the same minute (no growth), then 90 s back (a new file of that minute)
        r3 = lw.WavRing(os.path.join(tmp, "ring17b"), 60)
        t3 = 1793000270.0                                  # 50 s into its minute
        for _k in range(100):
            r3.write(vc.silence(), t3)
            t3 += 0.04
        t3 -= 20.0
        for _k in range(100):
            r3.write(vc.silence(), t3)
            t3 += 0.04
        first = os.path.basename(r3.f.name)
        n_first = r3.n
        t3 -= 90.0
        for _k in range(50):
            r3.write(vc.silence(), t3)
            t3 += 0.04
        second = os.path.basename(r3.f.name)
        r3.close()
        sizes = []
        for fn3 in os.listdir(os.path.join(tmp, "ring17b")):
            with wave.open(os.path.join(tmp, "ring17b", fn3), "rb") as wf:
                sizes.append(wf.getnframes())
        report("LH17 (review of 13r-r MED-B / LOW-5) a worker restart mid-minute continues the minute's file (never "
               "truncated: hard-linked into SAVE / KEEP events) on the clock; the clock set back never grows a file past "
               "its minute and rolls to the earlier minute's file",
               getattr(r2, 'reopened', 0) == 1 and n17 >= n_before and tone_kept and abs(n17 - want17) <= vc.FRAME_N and
               n_first <= int(60 * vc.RATE) + vc.FRAME_N and first != second and
               max(sizes) <= int(60 * vc.RATE) + vc.FRAME_N,
               "reopened %d; %s: %d samples before, %d after (the clock says %d), tone kept %s; clock back: %s %d "
               "samples, then %s; file sizes %s" % (getattr(r2, 'reopened', 0), name17, n_before, n17, want17, tone_kept, first,
                                                    n_first, second, sizes))

        # LH14 not configured / the volume limit / stop
        b = argparse.Namespace(backend="winmm", out_dev=None, in_dev=None, ring=None, seg_s=60, volume_max=80,
                               volume=70, sim_reply_s=0.0)
        wr = lw.Worker(b, io.BytesIO())
        st_ = wr.status()
        h = lhmod.Loudhailer({"enabled": False, "volume_max_pct": 150}, types.SimpleNamespace(storage=None))
        h2 = lhmod.Loudhailer({"enabled": False, "volume_max_pct": -5}, types.SimpleNamespace(storage=None))
        # review of 13r-r LOW-3 / LOW-4: "" / "default" = not named either; a negative volume_pct = 0
        nd = []
        for od, idv in (("", "default"), ("default", " "), ("sysdefault", "pulse"), ("PULSE", "SysDefault")):
            bx = argparse.Namespace(backend="winmm", out_dev=od, in_dev=idv, ring=None, seg_s=60, volume_max=80,
                                    volume=70, sim_reply_s=0.0)
            wx = lw.Worker(bx, io.BytesIO())
            nd.append(isinstance(wx.spk, lw.audiodev.NullOut) and isinstance(wx.mic, lw.audiodev.NullIn) and
                      "not configured" in wx.status()["error"])
        h3 = lhmod.Loudhailer({"enabled": False, "volume_pct": -20}, types.SimpleNamespace(storage=None))
        report("LH14b (review of 13r-r LOW-3 / LOW-4; 38add4b LOW-3) out_dev / in_dev '' / 'default' / 'sysdefault' / "
               "'pulse' with a real backend = not "
               "configured (never the PC's default device); a negative volume_pct starts at 0 (no endless resends)",
               all(nd) and h3.volume == 0, "not configured %s; volume %s" % (nd, h3.volume))
        h.worker = {"out_ok": True, "in_ok": True}
        h.proc = types.SimpleNamespace(wait=lambda t: 0, poll=lambda: 0, kill=lambda: None)
        h.stop()
        report("LH14 (review 13r LOW-1 / LOW-8 / LOW-4) a real backend without out_dev / in_dev opens NO device (never "
               "the PC's default speaker / mic): 'not configured', no speaker, no microphone; volume_max_pct clamped "
               "0..100; stop() forgets the old worker's status",
               isinstance(wr.spk, lw.audiodev.NullOut) and isinstance(wr.mic, lw.audiodev.NullIn) and
               not st_["out_ok"] and not st_["in_ok"] and "speaker not configured" in st_["error"] and
               "microphone not configured" in st_["error"] and h.cfg["volume_max_pct"] == 100 and
               h2.cfg["volume_max_pct"] == 0 and h.worker == {},
               "status %r; limits %s / %s; worker after stop %s" % (
                   st_["error"], h.cfg["volume_max_pct"], h2.cfg["volume_max_pct"], h.worker))

        # LH15 the control resend + the first-frame slack
        sent = []
        stn = types.SimpleNamespace(id=7, send=lambda m: sent.append(m))
        fake_gw = types.SimpleNamespace(storage=None, stations=types.SimpleNamespace(stations={7: stn}),
                                        audit=lambda *a_, **k_: None)
        h = lhmod.Loudhailer({"enabled": True, "frame_gap_s": 1.0}, fake_gw)
        p = object()
        h.proc = p
        h.next_start = h.next_info = 1e12
        h.worker_t = time.time()
        h.floor = {"id": 7, "user": "alice", "crew": "captain", "what": "talk", "t0": time.time(), "last": time.time()}
        h._from_worker(p, b"J", _json.dumps({"t": "status", "talk": False, "volume": h.volume}).encode())
        q = []
        while not h.wq.empty():
            q.append(h.wq.get_nowait())
        resent = h.n["talk_resent"] == 1 and any(b'"talk": true' in x for x in q if x)
        h._from_worker(p, b"J", _json.dumps({"t": "status", "talk": True, "volume": h.volume}).encode())
        no_more = h.n["talk_resent"] == 1
        h.proc = None                                      # (tick: no worker checks below)
        now = time.time()
        h.floor["last"] = now - 2.0                        # 2 s since the grant, no frame yet: within 1 + 1.5 s
        h.tick(now)
        kept = h.floor is not None
        if h.floor:
            h.floor["last"] = now - 2.6
        h.tick(now)
        gone_first = h.floor is None
        h.floor = {"id": 7, "user": "alice", "crew": "captain", "what": "talk", "t0": now, "last": now - 1.2,
                   "got": True}
        h.tick(now)
        gone_after = h.floor is None
        report("LH15 (review 13r LOW-2 / LOW-3) the worker's talk state differing from the floor = sent again (once per "
               "status); the first voice frame after the grant may take frame_gap_s + %.1f s, later frames "
               "frame_gap_s" % lhmod.FIRST_FRAME_SLACK_S,
               resent and no_more and kept and gone_first and gone_after,
               "resent %s (once %s); 2.0 s before the first frame: kept %s, 2.6 s: released %s; 1.2 s after frames: "
               "released %s" % (resent, no_more, kept, gone_first, gone_after))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    setup_logging(30)
    if not os.path.exists(tg.EXE):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    print("\n=== Section LH: loudhailer + boat microphone (step 13r) ===", flush=True)
    try:
        lh12_15()
    except Exception as e:
        import traceback
        traceback.print_exc()
        report("LH12-LH15 unit checks", False, "crashed: %r" % e)
    tmp = tempfile.mkdtemp(prefix="gwtest_lh_")
    sim = gwr = None
    st = {}
    try:
        sim = tg.Sim(2.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = tg.gw_cfg_b(sim, root, loudhailer={"enabled": True, "backend": "sim", "frame_gap_s": 0.8,
                                                 "talk_max_s": 6.0, "volume_max_pct": 80, "restart_s": 1.0},
                          recording={"segment_s": 4, "post_s": 2, "pre_s": 30})
        gwr = tg.GwRunner(cfg)
        g = gwr.gw
        port = cfg["stations"]["port"]
        time.sleep(0.8)
        cap = st["cap"] = Station(port, "alice", crew="captain")
        cop = st["cop"] = Station(port, "bob", crew="copilot", local=True)
        vic = st["vic"] = Station(port, "vic")
        pumps = list(st.values())
        ready = until(lambda: all(s.gw_status and (s.gw_status.get("hail") or {}).get("state") == "ACTIVE"
                                  for s in pumps), 15, pump=pumps)
        if not ready:
            report("LH0 the loudhailer comes up ACTIVE (simulated card)", False,
                   "hail status %s" % ((cap.gw_status or {}).get("hail")))
            return 1

        # LH1 one at a time, the first wins
        cap.hail.press()
        got = until(lambda: cap.hail.mine and cop.hail.speaker, 3, pump=pumps)
        local_ref = cop.hail.press() is None and "SPEAKING: CAPTAIN" in cop.hail.note   # refused here already
        cop.link.send_msg({"t": "ptt", "on": True})        # ... and by the gateway (the first wins)
        refused = until(lambda: cop.last("ptt_ack") and cop.last("ptt_ack").get("ok") is False, 3, pump=pumps)
        cop_sees = cop.hail.view()
        ref_ack = dict(cop.last("ptt_ack") or {})
        vic.link.send_msg({"t": "ptt", "on": True})
        vic_ref = until(lambda: vic.last("ptt_ack") and vic.last("ptt_ack").get("ok") is False, 3, pump=pumps)
        floor1 = dict(g.hail.speaker() or {})
        cap.hail.release()
        free = until(lambda: g.hail.floor is None and cop.hail.speaker is None, 3, pump=pumps)
        cop.hail.press()
        cop_got = until(lambda: cop.hail.mine, 3, pump=pumps)
        cap_sees = cap.hail.view()
        cop.hail.release()
        until(lambda: g.hail.floor is None, 3, pump=pumps)
        report("LH1 PUSH-TO-TALK one at a time: the captain first wins, the copilot is refused and sees SPEAKING: "
               "CAPTAIN (greyed), a VIEWER never; released = free; then the copilot speaks and the captain sees it",
               got and local_ref and refused and "captain speaks" in ref_ack.get("text", "") and cop_sees["enabled"] is False and
               "CAPTAIN" in cop_sees["text"] and vic_ref and "VIEWER" in (vic.last("ptt_ack") or {}).get("text", "") and
               floor1.get("crew") == "captain" and free and cop_got and "COPILOT" in cap_sees["text"] and
               not cap_sees["enabled"],
               "captain granted %s; copilot refused %r, view %s; viewer %r; floor %s; free %s; copilot granted %s, "
               "captain view %s" % (bool(got), ref_ack.get("text"), cop_sees,
                                    (vic.last("ptt_ack") or {}).get("text"), floor1, bool(free), bool(cop_got),
                                    cap_sees))

        # LH2 half duplex: the mic is muted while talking; listen only to a listening station
        cap.hail.set_listen(True)
        vic.hail.set_listen(False)
        wait(1.5, pumps)
        for s in pumps:
            s.pump()
        n0, v0 = cap.hail.rx_frames, vic.hail.rx_frames
        wait(1.0, pumps)
        n1 = cap.hail.rx_frames
        cop.hail.press()
        until(lambda: cop.hail.mine, 3, pump=pumps)
        wait(0.6, pumps)                                    # (the hang time + frames in flight)
        n2, w2 = cap.hail.rx_frames, dict(g.hail.worker)
        wait(1.5, pumps)
        for s in pumps:
            s.pump()
        n3, w3 = cap.hail.rx_frames, dict(g.hail.worker)
        cop.hail.release()
        until(lambda: g.hail.floor is None, 3, pump=pumps)
        wait(1.5, pumps)
        n4 = cap.hail.rx_frames
        report("LH2 half duplex: the boat microphone reaches a LISTENING station (~25 frames/s), NOTHING while the "
               "copilot talks (the worker mutes the mic, the horn's echo never goes back), back after the release; a "
               "station with LISTEN off gets nothing",
               n1 - n0 >= 15 and n3 - n2 <= 1 and w3.get("mic_frames", 0) - w2.get("mic_frames", 0) <= 1 and
               w3.get("played", 0) > w2.get("played", 0) and n4 - n3 >= 15 and vic.hail.rx_frames == v0,
               "listening 1 s: %d frames; while talking 1.5 s: %d (worker mic frames +%d, played +%d); after: %d; "
               "LISTEN off station: +%d" % (n1 - n0, n3 - n2, w3.get("mic_frames", 0) - w2.get("mic_frames", 0),
                                             w3.get("played", 0) - w2.get("played", 0), n4 - n3,
                                             vic.hail.rx_frames - v0))

        # LH3 recording: the AUDIO track in the ring, a SAVE and a KEEP 5' hold it
        ring = os.path.join(root, "video", "ring", "AUDIO")
        loud = vc.tone(600, 1500, vol=0.7)
        cap.hail.synth = lambda texts: loud
        cap.hail.play_message(0)
        until(lambda: g.hail.floor and g.hail.floor["what"] == "message", 3, pump=pumps)
        until(lambda: g.hail.floor is None, 6, pump=pumps)
        wait(4.5, pumps)                                     # the segment with the voice is finished
        ok, text, eid = gwr.call(g.rec.trigger, "button", "SAVE test (alice)", "alice")
        kok, ktext, keid, _have = gwr.call(g.rec.keep, 5, "alice")
        ev = until(lambda: tg.event_json(g.rec, eid) if eid and (tg.event_json(g.rec, eid) or {}).get(
            "state") == "complete" else None, 20)
        kev = until(lambda: tg.event_json(g.rec, keid) if keid and (tg.event_json(g.rec, keid) or {}).get(
            "state") == "complete" else None, 20)
        files = sorted(os.listdir(ring)) if os.path.isdir(ring) else []
        aud = (ev or {}).get("cameras", {}).get("AUDIO", [])
        kaud = (kev or {}).get("cameras", {}).get("AUDIO", [])
        levels = []
        for fn in aud:
            p = os.path.join(g.rec.events_dir(), eid, "AUDIO", fn)
            if os.path.exists(p):
                levels += wav_levels(p)
        loud_n = sum(1 for x in levels if x > -12.0)        # the message (shore -> boat)
        mic_n = sum(1 for x in levels if -60.0 < x < -25.0)  # the microphone's sea noise (boat -> shore)
        report("LH3 recording: both directions in one track (1-minute WAV in video/ring/AUDIO): a SAVE event and a "
               "KEEP 5' hold the AUDIO files next to the cameras, the shore's message (loud) and the boat microphone "
               "(sea noise) are in them",
               ok and kok and files and aud and kaud and loud_n >= 20 and mic_n >= 20 and
               (cap.gw_status.get("hail") or {}).get("recording"),
               "ring %s; SAVE %s AUDIO %s (%d loud frames, %d mic frames); KEEP %s AUDIO %s" % (
                   files[-3:], eid, aud, loud_n, mic_n, keid, kaud))

        # LH4 link priority
        cop_st = gwr.call(lambda: g.stations.stations[cop.welcome["id"]])
        import types
        rtt_rule = g.hail.voice_ok(types.SimpleNamespace(rtt_ms=900.0, backlog=lambda: 0, tmode="full", tmode_why="",
                                                         crew="copilot", single_op=False, id=99))
        cop_st.backlog = lambda: 10 ** 6                    # its TCP link is full (the driving data waits)
        cop.hail.info = {}
        until(lambda: cop.hail.info.get("voice_ok") is False, 3, pump=pumps)
        weak_view = cop.hail.view()
        cop.hail.press()
        weak_ref = until(lambda: "link busy" in (cop.last("ptt_ack") or {}).get("text", ""), 3, pump=pumps)
        cop.hail.set_listen(True)
        wait(1.2, pumps)
        c0 = cop.hail.rx_frames
        wait(1.0, pumps)
        weak_rx = cop.hail.rx_frames - c0
        del cop_st.backlog
        saver = {"on": True}
        real_saver = g.data_saver
        g.data_saver = lambda: saver["on"]
        wait(1.2, pumps)
        a0, c0 = cap.hail.rx_frames, cop.hail.rx_frames
        wait(1.0, pumps)
        saver_cap, saver_cop = cap.hail.rx_frames - a0, cop.hail.rx_frames - c0
        g.data_saver = real_saver
        import types
        from cameras import CameraManager
        gwx = types.SimpleNamespace(cfg={}, telem2={}, net=types.SimpleNamespace(on_emergency=False),
                                    data_saver=lambda: False, audit=lambda *a, **k: None,
                                    stations=types.SimpleNamespace(stations={}),
                                    hail=types.SimpleNamespace(kbps=lambda: 0.0))
        cm = CameraManager({"copilot_budget_kbps": 6000}, gwx)
        for sid, crew in ((2, "captain"), (3, "copilot")):
            s_ = types.SimpleNamespace(id=sid, user="u", crew=crew, single_op=False, session="s", video_key="k",
                                       sent=[])
            s_.send = s_.sent.append
            gwx.stations.stations[sid] = s_
        cm.request(gwx.stations.stations[3], {"preset": "All off"})
        cm.view_request(gwx.stations.stations[3], {"cam": "REAR-LEFT"})
        cm.station_kbps, cm.total_kbps = {2: 3000, 3: 2970}, 5970
        no_voice = cm.update_priority(1000.0)
        gwx.hail.kbps = lambda: 45.0
        with_voice = cm.update_priority(1001.0) and cm.views_reduced and "cam_rearleft_lo" not in cm.allowed_paths(3)
        report("LH4 link priority: a weak link (round trip 900 ms; a full TCP link) = VOICE OFF for that station with a note (TALK "
               "refused, no listen frames); the data saver keeps the voice for the captain only; the voice counts with "
               "the captain's video: 5970 kbit/s video fits the 6000 budget, + 45 kbit/s voice -> the copilot's camera "
               "view drops first",
               rtt_rule[0] is False and "weak link" in rtt_rule[1] and
               weak_view and "VOICE OFF" in weak_view["text"] and weak_ref and weak_rx == 0 and saver_cap >= 15 and
               saver_cop == 0 and not no_voice and with_voice,
               "weak: view %r, refused %s, listen frames %d; data saver: captain %d / copilot %d frames; camera view "
               "reduced without voice %s, with voice %s" % (weak_view and weak_view["text"], bool(weak_ref), weak_rx,
                                                            saver_cap, saver_cop, no_voice, with_voice))

        # LH5 no effect on driving
        sent = []
        real_send = g.aurix.send

        def spy(ftype, payload=b""):
            sent.append((ftype, threading.current_thread().name))
            return real_send(ftype, payload)
        g.aurix.send = spy
        t0 = cap.link.last_full or {}
        time.sleep(3.0)
        idle_types = {t for t, _ in sent}
        n_idle = len(sent)
        del sent[:]
        lags = []
        cap.hail.press()
        until(lambda: cap.hail.mine, 3, pump=pumps)
        for _ in range(15):
            a = time.perf_counter()
            gwr.call(lambda: None)
            lags.append((time.perf_counter() - a) * 1000)
            time.sleep(0.2)
            cap.pump()
        cap.hail.release()
        talk_types = {t for t, _ in sent}
        n_talk = len(sent)
        g.aurix.send = real_send
        for s in pumps:
            s.pump()
        t1 = cap.link.last_full or {}
        bad_src = []
        for fn in ("loudhailer.py", "loudhailer_worker.py", os.path.join("..", "groundstation", "voicecodec.py"),
                   os.path.join("..", "groundstation", "audiodev.py"), os.path.join("..", "groundstation", "hailer.py")):
            src = open(os.path.join(HERE, fn), encoding="utf-8").read()
            tree = ast.parse(src)
            for node in ast.walk(tree):
                if isinstance(node, ast.Attribute) and node.attr in ("aurix", "T_GW_NAVREQ", "T_GW_RJOY") or \
                        isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and \
                        node.func.attr == "send" and os.path.basename(fn) == "hailer.py":
                    bad_src.append("%s:%d %s" % (fn, node.lineno, getattr(node, "attr", "send(")))
        report("LH5 no effect on driving: the gateway sends the AURIX the same frame types at the same rate with and "
               "without voice, the mode and the alarm bits stay, no loudhailer / voice module touches the AURIX (no "
               "aurix / navreq / rjoy, the station never send()s a command), the event loop answers fast while voice "
               "flows",
               idle_types == talk_types and abs(n_talk - n_idle) <= max(4, n_idle // 3) and
               t0.get("mode") == t1.get("mode") and t0.get("alarm_bits") == t1.get("alarm_bits") and not bad_src and
               max(lags) < 150,
               "idle %d frames %s, talking %d frames %s; mode %s -> %s, alarms %s -> %s; source hits %s; loop answer "
               "max %.0f ms" % (n_idle, sorted(idle_types), n_talk, sorted(talk_types), t0.get("mode"), t1.get("mode"),
                                t0.get("alarm_bits"), t1.get("alarm_bits"), bad_src, max(lags)))

        # LH6 the audio process killed: a notice only, restarted
        until(lambda: g.hail.floor is None, 3, pump=pumps)
        cap.hail.press()
        until(lambda: cap.hail.mine, 3, pump=pumps)
        flags0 = g.status_flags()
        alarms0 = (cap.link.last_full or {}).get("alarm_bits")
        g.hail.proc.kill()
        t_kill = time.time()
        off = until(lambda: any(t >= t_kill and stt == "OFFLINE" for t, stt, _w in list(g.hail.history)), 5, pump=pumps)
        floor_gone = until(lambda: g.hail.floor is None, 3, pump=pumps)
        flags1 = g.status_flags()
        cap.gw_status = dict(cap.gw_status, hail=dict(cap.gw_status["hail"], state="OFFLINE", out_ok=False))
        view_off = cap.hail.view()                          # what the station shows with that status
        back = until(lambda: (cap.gw_status.get("hail") or {}).get("state") == "ACTIVE", 10, pump=pumps)
        cap.hail.release()
        alarms1 = (cap.link.last_full or {}).get("alarm_bits")
        audit = open(g.audit.path, encoding="utf-8").read() if getattr(g.audit, "path", None) else ""
        report("LH6 the audio process killed = OFFLINE NOTICE only: the floor released, GW_STATUS flags and the boat's "
               "alarm bits unchanged (never an alarm / mode change), LOUDHAILER OFF on the station, restarted -> ACTIVE",
               off and floor_gone and flags0 == flags1 and alarms0 == alarms1 and view_off and
               "LOUDHAILER OFF" in view_off["text"] and back and g.hail.starts >= 2,
               "offline %s, floor released %s, flags %s -> %s, alarms %s -> %s, view %r, back %s (starts %d)%s" % (
                   bool(off), bool(floor_gone), flags0, flags1, alarms0, alarms1, view_off and view_off["text"],
                   bool(back), g.hail.starts, ", audit hail_state" if "hail_state" in audit else ""))

        # LH7 volume limit
        cop.link.send_msg({"t": "hail_volume", "pct": 150})
        v_cop = until(lambda: cop.last("hail_volume_ack"), 3, pump=pumps)
        cap.link.send_msg({"t": "hail_volume", "pct": 95})
        v_cap = until(lambda: cap.last("hail_volume_ack"), 3, pump=pumps)
        vic.link.send_msg({"t": "hail_volume", "pct": 10})
        v_vic = until(lambda: vic.last("hail_volume_ack"), 3, pump=pumps)
        wv = until(lambda: g.hail.worker.get("volume") == 80 and g.hail.worker, 4)
        report("LH7 volume limit: a station on the boat asking 150 % gets the gateway's limit 80 %, the worker plays at "
               "80 %; a shore station without HELM and a VIEWER are refused",
               v_cop and v_cop["ok"] and v_cop["pct"] == 80 and v_cap and not v_cap["ok"] and v_vic and
               not v_vic["ok"] and wv and g.hail.volume == 80,
               "on board %s; captain (no HELM) %s; viewer %s; worker volume %s" % (
                   v_cop, v_cap and v_cap["text"], v_vic and v_vic["text"], (wv or {}).get("volume")))

        # LH8 the floor ends by itself
        sent_frames = []
        cap.hail._start_talk = lambda pcm: sent_frames.append("blocked")     # granted, but no voice frame is sent
        cap.hail.press()
        until(lambda: g.hail.floor is not None, 3, pump=pumps)
        t_gap = until(lambda: g.hail.floor is None, 4, pump=[])
        del cap.hail._start_talk
        wait(0.5, pumps)                                    # (the release reached the station)
        cap.hail.pressing = cap.hail.mine = False
        cap.hail.press()
        until(lambda: g.hail.floor is not None, 3, pump=pumps)
        t_a = time.time()
        t_max = until(lambda: g.hail.floor is None, 10, pump=pumps)
        held_s = time.time() - t_a
        cap.hail.release()
        cop.hail.press()
        until(lambda: g.hail.floor is not None and g.hail.floor["id"] == cop.welcome["id"], 3, pump=pumps)
        st.pop("cop").close()
        pumps.remove(cop)
        t_left = until(lambda: g.hail.floor is None, 4, pump=pumps)
        report("LH8 the floor ends by itself: no voice frame for frame_gap_s (0.8 s) = released, the talk time limit "
               "(6 s) even while frames flow, the speaker's station leaving",
               t_gap and t_max and 5.0 <= held_s <= 8.0 and t_left,
               "gap release %s; limit release after %.1f s; left %s" % (bool(t_gap), held_s, bool(t_left)))

        # LH9 sound signal
        cap.hail.signal("long")
        sig = until(lambda: g.hail.floor and g.hail.floor["what"] == "signal", 3, pump=pumps)
        vic2_ok = cap.last("hail_signal_ack")
        ps = st["cop2"] = Station(port, "bob", crew="copilot", local=True)
        pumps.append(ps)
        until(lambda: ps.gw_status, 4, pump=pumps)
        ps.link.send_msg({"t": "ptt", "on": True})
        ps_ref = until(lambda: ps.last("ptt_ack") and not ps.last("ptt_ack")["ok"], 3, pump=pumps)
        t_s = time.time()
        sig_end = until(lambda: g.hail.floor is None, 10, pump=pumps)
        sig_s = time.time() - t_s
        report("LH9 sound signal 1 prolonged blast: takes the floor (what = signal), another station is refused "
               "meanwhile, released when it has played (~5 s)",
               sig and vic2_ok and vic2_ok["ok"] and ps_ref and sig_end and sig_s < 8.0,
               "signal floor %s, ack %s, other refused %r, released after %.1f s" % (
                   bool(sig), vic2_ok, (ps.last("ptt_ack") or {}).get("text"), sig_s))

        # LH10 bad packets
        c0 = dict(g.hail.n)
        cap_st = gwr.call(lambda: g.stations.stations[cap.welcome["id"]])
        cap.hail.press()
        until(lambda: cap.hail.mine, 3, pump=pumps)
        sock, addr = cap.hail.sock, (cap.link.host, cap.link.port)
        sess = vc.session_bytes(cap.welcome["sid"])
        frame = vc.encode(vc.silence())
        good_key = bytes.fromhex(cap.welcome["udp_key"])
        q = cap.hail.seq + 200                              # (newer than the station's own frames)
        sock.sendto(vc.pack(b"T", sess, q, cap.link.now(), frame, b"x" * 16), addr)               # wrong key
        sock.sendto(vc.pack(b"T", sess, q + 1, cap.link.now() - 5.0, frame, good_key), addr)      # stale
        sock.sendto(vc.pack(b"T", sess, q + 2, cap.link.now(), frame[:20], good_key), addr)       # wrong size
        sock.sendto(vc.pack(b"T", sess, 1, cap.link.now(), frame, good_key), addr)                # old seq: ignored
        psess = vc.session_bytes(ps.welcome["sid"])
        sock.sendto(vc.pack(b"T", psess, 5, ps.link.now(), frame, bytes.fromhex(ps.welcome["udp_key"])), addr)
        time.sleep(0.5)
        c1 = dict(g.hail.n)
        cap.hail.seq = (q + 10) & 0xFFFF                    # (the station's own counter past the test packets)
        cap.hail.release()
        d = {k: c1.get(k, 0) - c0.get(k, 0) for k in ("bad", "talk_stale", "talk_other")}
        report("LH10 bad voice packets are dropped: a wrong key, a wrong size (bad), a stale time stamp, frames of a "
               "station that does not hold the floor",
               d["bad"] >= 2 and d["talk_stale"] >= 1 and d["talk_other"] >= 1 and cap_st is not None,
               "counts %s" % d)

        # LH11 ready message (the station's voice; injected samples here)
        until(lambda: g.hail.floor is None and cap.hail.speaker is None and not cap.hail.mine, 3, pump=pumps)
        p0 = g.hail.worker.get("played", 0)
        cap.hail.synth = lambda texts: vc.tone(500, 1200) if [lg for lg, _t in texts] == ["el", "en"] else None
        started = cap.hail.play_message(1)
        as_msg = until(lambda: g.hail.floor and g.hail.floor["what"] == "message", 3, pump=pumps)
        done = until(lambda: g.hail.floor is None and not cap.hail.msg_busy, 6, pump=pumps)
        played = until(lambda: g.hail.worker.get("played", 0) - p0 >= 20 and g.hail.worker, 3)
        until(lambda: cap.hail.speaker is None and not cap.hail.mine, 3, pump=pumps)
        cap.hail.synth = lambda texts: None
        no_voice = cap.hail.play_message(1)
        until(lambda: not cap.hail.msg_busy, 3, pump=pumps)
        report("LH11 ready message (Greek then English): one click asks the floor as a message, the spoken samples "
               "play on the boat (~30 frames), the floor is given back at the end; no voice on this PC = a note, "
               "nothing sent",
               started and as_msg and done and played and no_voice and g.hail.floor is None and
               "No offline voice" in cap.hail.note,
               "started %s, floor as message %s, released %s, played +%s, no-voice note %r" % (
                   started, bool(as_msg), bool(done), (played or {}).get("played", 0) - p0, cap.hail.note) +
               " history %s starts %d busy %s" % (list(g.hail.history)[-3:], g.hail.starts, cap.hail.msg_busy))
    finally:
        for s in st.values():
            try:
                s.close()
            except Exception:
                pass
        if gwr:
            gwr.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)
    bad = results.count(False)
    print("\nLOUDHAILER TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
