"""Sound + voice tests (section D / G2). No sound card needed (a recording player).

  S1 tones are made here (every tone, WAV round trip, resampling 16 -> 22.05 kHz)
  S2 a higher level interrupts a lower one; a lower one waits (queued), then plays
  S3 voices never overlap: many callouts at once play one after another
  S4 the same callout is not repeated within repeat_s; after it, it is
  S5 advisories / feedback can be muted, WARNINGS never (not even sound off /
     volume 0: they play at >= WARN_MIN_VOLUME)
  S6 a WARNING repeats until acknowledged; a new / returning one sounds again
  S7 quiet (night) mode: softer, advisories dropped (mode changes kept)
  S8 the callouts: modes, autopilot disconnect only on a fault, takeover chirp,
     traffic only when the situation changes (clock position + distance),
     avoidance proposal starboard / port, power raised, channel change,
     depth 10 / 5 / 3 m, waypoint in 30 s, turning to <heading digits>
  S9 the real offline voice: a Greek and an English phrase (Piper / Windows)
  S10 sound never changes control: the sound modules never send to the boat
     (no link / protocol use), a whole callout session sends nothing
  S11 step 13o: the voice engine is a sound setting (Piper / the Windows voice), fallback when one is missing
  S12 a new voice / new settings remake every cached sentence (+ clips); same voice = kept; no voice = kept
  S13 lead-in silence + fade-in in front of every sentence; Piper with the calmer 13o settings
  S14 the voice samples: 5 sentences per option + README (the real Piper when installed)
  S15 review 13o M1: the sound page never waits for the voice maker (play while a slow synth holds the lock)
  S16 review 13o L1-L4: the Windows voice lookup off the Tk thread, published whole, a failure = unknown + cache kept,
      a quote in the temp path escaped

Run:  python test_sound.py        (exit code 0 = all passed)
"""
import os
import re
import sys
import tempfile
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
TMP = tempfile.mkdtemp(prefix="trygons_snd_")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(TMP, "gs.json")
os.environ.setdefault("TRYGONS_TEST", "1")      # temp track folder (review 13n3 L7)
os.environ["TRYGONS_SOUND_CACHE"] = os.path.join(TMP, "cache")
os.environ["TRYGONS_LANG"] = "en"
import alerts  # noqa: E402
import i18n  # noqa: E402
import sound  # noqa: E402
import tts  # noqa: E402

results = []
W, C, A, F = sound.WARNING, sound.CAUTION, sound.ADVISORY, sound.FEEDBACK


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Player:
    """Records (time, path or None = stop)."""

    def __init__(self, clock):
        self.clock, self.log = clock, []

    def play(self, path):
        self.log.append((self.clock(), path))

    def stop(self):
        self.log.append((self.clock(), None))


def make(secs=2.0):
    clk = Clock()
    pl = Player(clk)
    built = []

    def build(item, vol):
        built.append((item.code, item.level, round(vol, 2)))
        return item.code, secs
    sc = sound.Scheduler(build, pl, clk)
    sc.repeat_s = 20.0
    return sc, clk, pl, built


def it(code, lv, keep=False):
    return sound.Item(code, lv, None, [], keep)


def run(sc, clk, seconds, step=0.1, warn=lambda c: sound.Item("alarm." + c, W, "warning", [], True)):
    end = clk.t + seconds
    while clk.t < end:
        clk.t += step
        sc.tick(warn)


def overlaps(pl, secs):
    """Did a clip start while another one still played, without a stop first?"""
    bad, busy_until = 0, -1e9
    for t, p in pl.log:
        if p is None:
            busy_until = -1e9
            continue
        if t < busy_until - 1e-6:
            bad += 1
        busy_until = t + secs
    return bad


def main():
    # S1 tones + wav
    lens = {n: len(sound.tone(n)) for n in sound.TONE_NAMES}
    p = os.path.join(TMP, "t.wav")
    sound.write_wav(p, sound.tone("caution"))
    back = sound.read_wav(p)
    with wave.open(p, "wb") as w:                       # a 16 kHz file (like the Greek Piper voice)
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(b"\x00\x10" * 16000)
    rs = sound.read_wav(p)
    report("S1 every tone made here (no sound files needed); WAV round trip; 16 kHz voice resampled to 22.05 kHz",
           all(n > 100 for n in lens.values()) and len(back) == lens["caution"] and abs(len(rs) - 22050) < 2,
           "tone lengths (samples) %s; caution back %d; 1 s at 16 kHz -> %d samples" % (lens, len(back), len(rs)))

    # S2 priority
    sc, clk, pl, built = make()
    r1 = sc.request(it("adv1", A))
    clk.t += 0.5
    r2 = sc.request(it("cau1", C))                      # higher: interrupts
    clk.t += 0.5
    r3 = sc.request(it("adv2", A))                      # lower: waits
    run(sc, clk, 4.0)
    order = [h[1] for h in sc.history if h[3] == "played"]
    interrupted = any(h[1] == "adv1" and h[3] == "interrupted" for h in sc.history)
    report("S2 a higher level interrupts a lower one; a lower one waits and plays afterwards",
           r1 == "played" and r2 == "played" and r3 == "queued" and interrupted and order == ["adv1", "cau1", "adv2"],
           "advisory %s, caution %s (advisory interrupted %s), next advisory %s; order %s" % (r1, r2, interrupted, r3,
                                                                                            order))

    # S3 no overlap
    sc, clk, pl, built = make(1.5)
    res = [sc.request(it("c%d" % i, C)) for i in range(5)]
    run(sc, clk, 10.0)
    played = [h[1] for h in sc.history if h[3] == "played"]
    report("S3 voices never overlap: 5 callouts at once play one after the other",
           res.count("played") == 1 and res.count("queued") == 4 and played == ["c%d" % i for i in range(5)] and
           overlaps(pl, 1.5) == 0,
           "requests %s; played %s; overlapping starts %d" % (res, played, overlaps(pl, 1.5)))

    # S4 no repeat
    sc, clk, pl, built = make(1.0)
    a = sc.request(it("traffic.5.1", C))
    clk.t += 5
    b = sc.request(it("traffic.5.1", C))
    clk.t += 16
    c = sc.request(it("traffic.5.1", C))
    sc.repeat_s = 60.0
    clk.t += 30
    d = sc.request(it("traffic.5.1", C))
    report("S4 the same callout not again within repeat_s (configurable); afterwards it is",
           (a, b, c, d) == ("played", "repeat", "played", "repeat"),
           "t=0 %s, t=5 s %s, t=21 s %s; repeat_s 60 -> t=51 s %s" % (a, b, c, d))

    # S5 mutes; warnings never muted
    sc, clk, pl, built = make(1.0)
    sc.mute_advisory = sc.mute_feedback = True
    ra, rf = sc.request(it("adv", A)), sc.request(it("fb.confirm", F))
    clk.t += 5
    rc = sc.request(it("cau", C))
    clk.t += 5
    sc.enabled, sc.volume = False, 0
    rw = sc.request(it("alarm.compass_lost", W))
    vol_w = [b_[2] for b_ in built if b_[0] == "alarm.compass_lost"]
    clk.t += 5
    sc.set_warnings(["gear_fault"])
    run(sc, clk, 0.5)
    rep_w = [h for h in sc.history if h[1] == "alarm.gear_fault" and h[3] == "played"]
    report("S5 advisories and small sounds can be muted; WARNINGS never (sound off + volume 0 -> still played at "
           ">= %d %%)" % sound.WARN_MIN_VOLUME,
           ra == "muted" and rf == "muted" and rc == "played" and rw == "played" and vol_w and
           vol_w[0] >= sound.WARN_MIN_VOLUME / 100.0 and rep_w,
           "advisory %s, feedback %s, caution %s; warning with sound off / volume 0: %s at volume %s; repeating "
           "warning played %d" % (ra, rf, rc, rw, vol_w, len(rep_w)))

    # S6 repeat until acknowledged
    sc, clk, pl, built = make(1.0)
    sc.set_warnings(["compass_lost"])
    run(sc, clk, 12.0)
    n1 = len([h for h in sc.history if h[1] == "alarm.compass_lost" and h[3] == "played"])
    sc.ack()
    sc.set_warnings(["compass_lost"])
    run(sc, clk, 12.0)
    n2 = len([h for h in sc.history if h[1] == "alarm.compass_lost" and h[3] == "played"])
    sc.set_warnings(["compass_lost", "joystick_fault"])
    run(sc, clk, 2.0)
    new = any(h[1] == "alarm.joystick_fault" and h[3] == "played" for h in sc.history)
    sc.ack()
    sc.set_warnings([])
    sc.set_warnings(["compass_lost"])
    run(sc, clk, 2.0)
    n3 = len([h for h in sc.history if h[1] == "alarm.compass_lost" and h[3] == "played"])
    report("S6 a WARNING repeats until acknowledged; acknowledged = silent; a new one sounds; one that went away "
           "and came back sounds again",
           n1 >= 3 and n2 == n1 and new and n3 == n2 + 1,
           "in 12 s: %d times; after ACK +%d in 12 s; a new warning played %s; gone + back -> played again %s" % (
               n1, n2 - n1, new, n3 == n2 + 1))

    # S6b step 13m: the station disconnected = reset: a sounding warning stops, nothing repeats or waits, and the
    # next boat's same warning sounds again (its acknowledge is not carried over)
    sc, clk, pl, built = make(3.0)
    sc.set_warnings(["compass_lost"])
    run(sc, clk, 0.5)
    sc.request(it("cau_q", C))                          # waits behind the warning
    ss = sound.SoundSystem.__new__(sound.SoundSystem)
    ss.sched = sc
    ss.reset()
    stopped = pl.log[-1][1] is None and sc.current is None and not sc.queue and not sc.active and not sc.acked
    m1 = len([h for h in sc.history if h[3] == "played"])
    sc.set_warnings([])
    run(sc, clk, 6.0)
    silent = len([h for h in sc.history if h[3] == "played"]) == m1
    sc.set_warnings(["compass_lost"])
    run(sc, clk, 0.5)
    fresh = len([h for h in sc.history if h[1] == "alarm.compass_lost" and h[3] == "played"]) == 2
    report("S6b disconnect (13m): reset stops the sounding warning, drops the queue, nothing repeats; the next boat's "
           "warning sounds again", stopped and silent and fresh, "stopped %s silent %s fresh %s" % (stopped, silent,
                                                                                                   fresh))

    # S7 quiet
    sc, clk, pl, built = make(1.0)
    sc.quiet = True
    q1 = sc.request(it("nav.wp30", A))
    clk.t += 3
    q2 = sc.request(it("mode.auto", A, keep=True))
    clk.t += 3
    q3 = sc.request(it("cau", C))
    vols = dict((b_[0], b_[2]) for b_ in built)
    clk.t += 3
    sc.request(it("alarm.x", W))
    vols = dict((b_[0], b_[2]) for b_ in built)
    report("S7 quiet / night mode: softer, fewer advisories (mode changes kept), warnings stay loud enough",
           q1 == "quiet" and q2 == "played" and q3 == "played" and vols["cau"] == 0.4 and vols["alarm.x"] >= 0.6,
           "waypoint advisory %s, mode change %s, caution %s at %.2f (80 %% -> half), warning at %.2f" % (
               q1, q2, q3, vols.get("cau", -1), vols.get("alarm.x", -1)))

    # S8 callouts
    said = []

    class FakeSound:
        voice_lang = "en"

        def say(self, code, keys=None, **kw):
            said.append((code, keys))

        def feedback(self, code):
            said.append((code, None))

        def alarm(self, code, lv):
            said.append(("alarm." + code, lv))

    class FakeCharts:
        dep = 30.0

        def __bool__(self):
            return True

        def depth(self, la, lo):
            return self.dep

    class FakeApp:
        t = {"heading": 90.0, "mode": 3}
        charts = FakeCharts()
    clk = Clock()
    co = sound.Callouts(FakeSound(), FakeApp(), clock=clk)
    base = {"mode": 0, "alarm_bits": 0, "power_fwd": 30.0, "power_rev": 5.0, "throttle": 0.0, "throttle_demand": 0.0,
            "pos_source": 1, "est_lat": 37.8, "est_lon": 23.6, "est_sog": 6.0, "wp_set": False, "heading": 90.0}

    def step(**kw):
        nonlocal base
        new = dict(base, **kw)
        clk.t += 1.1
        co.telem(base, new)
        base = new
    step(mode=2)
    step(mode=0)                                        # STANDBY by the button: no fault
    step(mode=3)
    step(mode=1)                                        # the joystick took over from AUTO
    step(mode=2)
    step(alarm_bits=0x001)                              # compass lost ...
    step(mode=0, alarm_bits=0x001)                      # ... and the autopilot dropped out
    step(alarm_bits=0, power_fwd=50.0)                  # power raised (confirmed)
    for dep in (12, 9, 7, 4.5, 4.0, 2.5):
        FakeCharts.dep = dep
        clk.t += 1.1
        co.telem(base, base)
    lat0 = 37.8
    step(mode=3, wp_set=True, wp_lat=lat0 + 0.01, wp_lon=23.6, auto_wp=0)
    step(est_lat=lat0 + 0.01 - 85 / 111320.0)          # 85 m at 6 kn = 27 s
    step(est_lat=lat0 + 0.01 - 80 / 111320.0)          # still: said once
    step(auto_wp=1, wp_lat=lat0 + 0.01, wp_lon=23.61)  # next leg: due east -> "zero nine zero"
    gw1 = {"net": {"active": "4G"}, "avoid": {"threats": [{"mmsi": 7, "level": "warn", "bearing": 150.0,
                                                           "dist_nm": 0.45}]}}
    co.gw(gw1)
    co.gw(gw1)                                          # nothing changed: silent
    gw2 = {"net": {"active": "4G", "emergency": True}, "avoid": {
        "threats": [{"mmsi": 7, "level": "danger", "bearing": 150.0, "dist_nm": 0.3}], "state": "PROPOSE",
        "proposal": {"id": 3, "heading": 160}}}
    co.gw(gw2)
    co.gw(dict(gw2, avoid=dict(gw2["avoid"], proposal={"id": 4, "heading": 30})))
    codes = [c for c, _k in said]
    traffic = [(c, k) for c, k in said if c.startswith("traffic.7")]
    turning = [k for c, k in said if c.startswith("nav.turning")]
    exp = ["mode.hdg_lock", "mode.standby", "mode.auto", "fb.chirp2", "mode.manual", "mode.hdg_lock",
           "ap.disconnect", "power.fwd.50", "depth.10", "depth.5", "depth.3", "mode.auto", "nav.wp30"]
    seq_ok = [c for c in codes if not c.startswith(("alarm.", "traffic", "link", "nav.turning"))] == exp
    report("S8 callouts: modes, takeover chirp, autopilot disconnect only on a fault, power, depth 10/5/3, waypoint "
           "in 30 s, turning to <digits>, traffic only on a change (clock + distance), avoid starboard / port, "
           "STARLINK",
           seq_ok and len(traffic) == 2 and traffic[0][1] == ["say.traffic", "say.clock_2", "say.dist_m_800"] and
           traffic[1][1][2] == "say.dist_m_600" and turning and turning[0][1:] == ["say.digit_0", "say.digit_8",
                                                                                   "say.digit_5"]
           and "traffic.avoid_stbd" in codes and "traffic.avoid_port" in codes and "link.starlink" in codes and
           ("alarm.compass_lost", None) not in said,
           "said %s; traffic %s; turning %s" % (codes, traffic, turning[:1]))

    # S9 the real voice
    lib = sound.Library(cache=os.path.join(TMP, "cache"), overrides=os.path.join(TMP, "own"))
    got = {}
    for lg in ("el", "en"):
        if not tts.engine(lg):
            got[lg] = None
            continue
        path, secs = lib.clip("caution", ["say.traffic", "say.clock_2", "say.dist_m_800"], lg, 0.8)
        got[lg] = secs
    own = os.path.join(TMP, "own", "en")
    os.makedirs(own)
    sound.write_wav(os.path.join(own, "say.mode.auto.wav"), sound._note(440, 2000))
    p_own, secs_own = lib.clip(None, ["say.mode.auto"], "en", 1.0)
    if any(v is None for v in got.values()):
        print("[N/A ] S9 the offline voice - %s (install: python tools/get_piper.py)" % {
            k: tts.describe(k) for k in got})
        override_ok = abs(secs_own - 2.0) < 0.05
        report("S9b a recorded file sounds/<lang>/<key>.wav replaces the voice", override_ok, "%.2f s" % secs_own)
    else:
        report("S9 the offline voice speaks Greek and English (tone + phrase joined, cached); a recorded file "
               "sounds/<lang>/<key>.wav replaces a phrase",
               got["el"] > 1.2 and got["en"] > 1.2 and abs(secs_own - 2.0) < 0.05,
               "Greek %s: %.1f s, English %s: %.1f s; own recording %.2f s" % (
                   tts.describe("el"), got["el"], tts.describe("en"), got["en"], secs_own))

    # S10 never to the boat
    bad = []
    for fn in ("sound.py", "soundtest.py", "tts.py"):
        src = open(os.path.join(HERE, fn), encoding="utf-8").read()
        for pat in (r"\.send\(", r"send_msg", r"proto\.T_", r"app\.link\b"):
            if re.search(pat, src):
                bad.append("%s: %s" % (fn, pat))

    class NoLink:
        def __getattr__(self, k):
            raise AssertionError("the sound touched the link: %s" % k)

    class App2:
        t = {"heading": 0.0, "mode": 0}
        charts = None
        link = NoLink()
        lamps = alerts.MasterLamps()
    snd2 = sound.SoundSystem(player=sound.NullPlayer(), library=sound.Library(cache=os.path.join(TMP, "c2")))
    co2 = sound.Callouts(snd2, App2(), clock=Clock())
    try:
        prev = dict(base, mode=0)
        for m in (2, 3, 1, 0, 5, 0):
            d = dict(prev, mode=m, alarm_bits=0x400001)
            co2.telem(prev, d)
            prev = d
        co2.gw(gw2)
        snd2.sched.set_warnings(["compass_lost"])
        snd2.sched.tick(snd2.warn_item)
        snd2.ack()
        touched = None
    except AssertionError as e:
        touched = str(e)
    report("S10 sound never changes control: no link / protocol use in the sound modules; a whole callout session "
           "never touches the link", not bad and touched is None,
           "found %s; session %s" % (bad, touched or "never touched the link"))
    voice_tests()
    review_13o_tests()
    i18n.set_lang("en", save=False)
    bad_n = results.count(False)
    print("SOUND TESTS: %d passed, %d failed" % (len(results) - bad_n, bad_n))
    return 1 if bad_n else 0


def wav_secs(p):
    with wave.open(p, "rb") as w:
        return w.getnframes() / float(w.getframerate())


def voice_tests():
    """S11-S14: step 13o (owner 2026-10-31 F) - the voice engine setting, a new voice remakes the cache,
    the lead-in silence, the tuned Piper settings, the voice samples."""
    import settings
    saved = (tts._win, tts._prefer, tts.voice_id, tts.synth, tts.subprocess.run, tts.piper_bin, tts.piper_voice)
    env_sound = os.environ.get("TRYGONS_SOUND")            # (review 13o L7: restored afterwards)
    try:
        # S11 the engine follows the setting; a missing Windows voice falls back to Piper (and back)
        tts.piper_bin = lambda: "piper.exe"
        tts.piper_voice = lambda lang: "el_GR-rapunzelina-low.onnx"
        tts._win = {"el": ("rt", "Microsoft Stefanos")}
        a = (tts.engine("el", "piper"), tts.engine("el", "windows"))
        ida, idb = tts.voice_id("el", "piper"), tts.voice_id("el", "windows")
        tts._win = {}
        b = tts.engine("el", "windows")
        tts.piper_voice = lambda lang: None
        tts._win = {"el": ("rt", "Microsoft Stefanos")}
        c = tts.engine("el", "piper")
        tts.piper_voice = lambda lang: None
        tts._win = {}
        d = (tts.engine("el", "piper"), tts.voice_id("el"))
        snd = sound.SoundSystem(player=sound.NullPlayer(), library=sound.Library(cache=os.path.join(TMP, "c11")))
        os.environ["TRYGONS_SOUND"] = "off"
        snd.set_engine("windows")
        p1 = (settings.get("voice_engine"), tts.prefer())
        snd.set_engine("nonsense")
        snd.set_engine("piper")
        p2 = (settings.get("voice_engine"), tts.prefer())
        tts.set_prefer("bad")
        report("S11 the voice engine is a sound setting (piper / windows): followed when it can speak the language, "
               "else the other one; a different engine = a different voice id; saved per PC",
               a == ("piper", "windows") and ida != idb and "Stefanos" in idb and b == "piper" and c == "windows" and
               d == (None, None) and p1 == ("windows", "windows") and p2 == ("piper", "piper") and
               tts.prefer() == "piper",
               "with both %s, ids differ %s; no Windows voice -> %s; no Piper -> %s; none -> %s; setting %s / %s" % (
                   a, ida != idb, b, c, d, p1, p2))

        # S12 a new voice (engine, model, settings) deletes the old sentences + clips: all made again
        cache = os.path.join(TMP, "c12")
        lib = sound.Library(cache=cache)
        os.makedirs(os.path.join(cache, "el"))
        os.makedirs(os.path.join(cache, "clips"))
        old = os.path.join(cache, "el", "p_old.wav")
        clip = os.path.join(cache, "clips", "x.wav")
        own = os.path.join(cache, "el", "keep.txt")
        for p in (old, clip, own):
            open(p, "w").write("x")
        vid = {"v": "A"}
        tts.voice_id = lambda lang, choice=None: vid["v"]
        g1 = lib.fresh("el")                                    # a cache from before 13o (no stamp): remade
        gone1 = not os.path.exists(old) and not os.path.exists(clip) and os.path.exists(own)
        open(old, "w").write("x")
        lib2 = sound.Library(cache=cache)                       # a restart with the same voice: kept
        g2 = lib2.fresh("el")
        kept = os.path.exists(old)
        vid["v"] = "B"
        g3 = lib2.fresh("el")
        gone3 = not os.path.exists(old)
        open(old, "w").write("x")
        vid["v"] = None                                          # no voice on this PC: keep what is there
        g4 = sound.Library(cache=cache).fresh("el")
        made = []
        vid["v"] = "C"
        tts.synth = lambda lang, items, **k: made.extend(items) or 0
        lib3 = sound.Library(cache=cache)
        lib3.make("el", ["Μία πρόταση."])
        report("S12 the voice cache belongs to one voice: a cache from before 13o or another voice / setting = every "
               "sentence + clip deleted and made again; the same voice after a restart = kept; no voice = kept; "
               "own files untouched", g1 == 1 and gone1 and g2 == 0 and kept and g3 == 1 and gone3 and g4 == 0 and
               os.path.exists(old) is False and len(made) == 1 and
               open(os.path.join(cache, "el", "voice.txt"), encoding="utf-8").read().strip() == "C",
               "first %d (gone %s), restart %d (kept %s), new voice %d (gone %s), no voice %d, remade %d" % (
                   g1, gone1, g2, kept, g3, gone3, g4, len(made)))
        tts.voice_id, tts.synth = saved[2], saved[3]

        # S13 lead-in silence + fade-in; Piper runs with the tuned settings (not the model's own)
        p = os.path.join(TMP, "lead.wav")
        sound.write_wav(p, sound._note(440, 1000, 0.8))
        s0 = wav_secs(p)
        tts.lead_in(p)
        s1 = wav_secs(p)
        smp = sound.read_wav(p)
        rate = 22050
        with wave.open(p, "rb") as w:
            rate = w.getframerate()
        lead = int(rate * tts.LEAD_IN_MS / 1000.0)
        quiet = max(abs(x) for x in smp[:lead]) == 0
        fade_n = int(rate * tts.FADE_MS / 1000.0)
        ramp = max(abs(x) for x in smp[lead:lead + fade_n // 4]) < 0.3 * max(abs(x) for x in smp[lead + fade_n:])
        calls = []

        class R:
            returncode = 0
        tts.piper_bin = lambda: os.path.join(TMP, "piper.exe")
        tts.piper_voice = lambda lang: "el.onnx"
        tts._win = {}
        tts.subprocess.run = lambda args, **k: calls.append(args) or R()
        tts.synth("el", [("Δοκιμή.", os.path.join(TMP, "x", "t.wav"))])
        args = calls[0] if calls else []
        tuned = all(k in args and abs(float(args[args.index(k) + 1]) - v) < 1e-6 for k, v in (
            ("--noise_scale", tts.PIPER_TUNE["noise_scale"]), ("--noise_w", tts.PIPER_TUNE["noise_w"]),
            ("--length_scale", tts.PIPER_TUNE["length_scale"])))
        report("S13 every sentence starts with %d ms of silence + a %d ms fade-in; Piper runs with the calmer 13o "
               "settings %s (model defaults %s)" % (tts.LEAD_IN_MS, tts.FADE_MS, tts.PIPER_TUNE, tts.PIPER_DEFAULT),
               abs(s1 - s0 - tts.LEAD_IN_MS / 1000.0) < 0.01 and quiet and ramp and tuned and
               tts.PIPER_TUNE["noise_scale"] < tts.PIPER_DEFAULT["noise_scale"] and
               tts.PIPER_TUNE["noise_w"] < tts.PIPER_DEFAULT["noise_w"],
               "%.2f -> %.2f s, silent lead %s, fade %s, piper args %s" % (s0, s1, quiet, ramp, args[3:]))
        tts.subprocess.run, tts.piper_bin, tts.piper_voice, tts._win = saved[4], saved[5], saved[6], saved[0]

        # S14 the voice samples: 5 sentences per option + README (a missing Windows voice is said)
        folder = os.path.join(TMP, "samples")
        real = all(tts.engine(lg, "piper") == "piper" for lg in ("el",))
        if not real:
            def fake(lang, items, **k):
                for _t, q in items:
                    os.makedirs(os.path.dirname(q), exist_ok=True)
                    sound.write_wav(q, sound._note(300, 500))
                return len(items)
            tts.synth = fake
            tts.piper_bin = lambda: "piper.exe"
            tts.piper_voice = lambda lang: "el.onnx"
        tts._win = {}                                       # no Windows Greek voice here
        got = sound.make_samples(folder, "el")
        files = {o: sorted(os.listdir(os.path.join(folder, o))) if os.path.isdir(os.path.join(folder, o)) else []
                 for o, _e, _p in sound.SAMPLE_OPTIONS}
        readme = open(os.path.join(folder, "README.txt"), encoding="utf-8").read()
        lead_new = lead_old = None
        if real:
            lead_new = max(abs(x) for x in sound.read_wav(os.path.join(folder, "1_piper_new", "sample_1.wav"))[:3000])
            lead_old = max(abs(x) for x in sound.read_wav(os.path.join(folder, "2_piper_before", "sample_1.wav"))[:3000])
        texts = [i18n.TL("el", k) for k in sound.SAMPLE_KEYS]
        report("S14 voice samples: 5 sentences per option (Piper 13o, Piper before, the Windows voice) in a folder + "
               "README; no Windows Greek voice = said, nothing made; %s Piper" % ("the REAL" if real else "a fake"),
               got["1_piper_new"] == 5 and got["2_piper_before"] == 5 and got["3_windows"] == 0 and
               len(files["1_piper_new"]) == 5 and "no windows voice" in readme and all(t in readme for t in texts)
               and "σχέση" in texts[0] and (not real or (lead_new == 0 and lead_old > 0)),
               "made %s; README names the missing voice %s; real lead-in silent %s / before %s" % (
                   got, "no windows voice" in readme, lead_new, lead_old))
    finally:
        (tts._win, tts._prefer, tts.voice_id, tts.synth, tts.subprocess.run, tts.piper_bin,
         tts.piper_voice) = saved
        if env_sound is None:
            os.environ.pop("TRYGONS_SOUND", None)
        else:
            os.environ["TRYGONS_SOUND"] = env_sound


def review_13o_tests():
    """S15 / S16 (review of 13o c704a7f): M1 the sound page never waits for the voice maker on the Tk thread;
    L1-L4 the Windows voice lookup."""
    import ast
    import threading
    import time as _t
    saved = (tts._win, tts._win_retry_t, tts._ps, tts._prefer, tts.piper_bin, tts.piper_voice)
    try:
        # S15 (M1) a slow synth batch holds the library lock (a new voice remakes hundreds of sentences): playing a
        # sound from the page returns at once (tone now, the voice made in the background)
        cache = os.path.join(TMP, "c15")
        lib = sound.Library(cache=cache)
        lib.checked["en"] = "some voice"                    # (the cache was checked already)
        hold, held = threading.Event(), threading.Event()

        def busy():
            with lib.lock:
                held.set()
                hold.wait(10)
        threading.Thread(target=busy, daemon=True).start()
        held.wait(5)
        lib.make = lambda lang, texts, retries=3: (lib.lock.acquire(), lib.lock.release())   # waits like make()
        t0 = _t.perf_counter()
        path, secs = lib.clip("warning", ["say.alarm.compass_lost"], "en", 0.8, block=False)
        dt = _t.perf_counter() - t0
        hold.set()
        src = open(os.path.join(HERE, "soundtest.py"), encoding="utf-8").read()
        calls = [n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call) and
                 isinstance(n.func, ast.Attribute) and n.func.attr == "clip"]
        nonblock = bool(calls) and all(any(k.arg == "block" and getattr(k.value, "value", None) is False
                                           for k in c.keywords) for c in calls)
        report("S15 (review 13o M1) the sound page never waits for the voice maker: with a slow synth batch holding the "
               "library lock a played sound returns at once (the tone now, the voice next time); the page plays with "
               "block=False",
               dt < 0.3 and os.path.exists(path) and secs > 0 and nonblock,
               "clip returned in %.0f ms (%s), soundtest clip calls non-blocking %s" % (dt * 1000,
                                                                                      os.path.basename(path), nonblock))
        # S16 (L1-L4) the Windows voice lookup
        calls_ps, gate = [], threading.Event()
        main_th = threading.current_thread()

        def slow_ps(script, timeout):
            calls_ps.append(threading.current_thread() is main_th)
            gate.wait(5)

            class R:
                returncode = 0
                stdout = "rt|el-GR|Microsoft Stefanos\n".encode()
            return R()
        tts._ps = slow_ps
        tts._win, tts._win_retry_t = None, 0.0
        tts.piper_bin = lambda: "piper.exe"
        tts.piper_voice = lambda lang: "x.onnx"
        tts.set_prefer("piper")
        t0 = _t.perf_counter()
        av = tts.available("el")                            # the page asks every 2 s
        quick = _t.perf_counter() - t0 < 0.2 and av == ["piper"]
        _t.sleep(0.2)
        known_during = tts.windows_known()                  # L2: never "known" while the lookup runs
        gate.set()
        tts._win_bg.join(5)
        after = tts.windows_known() and tts.available("el") == ["piper", "windows"]
        not_tk = calls_ps and not any(calls_ps)             # L1: never on the calling (Tk) thread
        # L3 a failed lookup = unknown (not "no voices"), tried later; the cache is NOT deleted for it
        tts._win, tts._win_retry_t = None, 0.0

        def fail_ps(script, timeout):
            raise OSError("PowerShell missing")
        tts._ps = fail_ps
        got = tts.windows_voices()
        unknown = got == {} and not tts.windows_known() and tts._win_retry_t > _t.time()
        tts.set_prefer("windows")
        cache = os.path.join(TMP, "c16")
        os.makedirs(os.path.join(cache, "el"))
        with open(os.path.join(cache, "el", "voice.txt"), "w", encoding="utf-8") as f:
            f.write("v2|windows|rt|Microsoft Stefanos|lead250\n")
        keep = os.path.join(cache, "el", "p_keep.wav")
        open(keep, "wb").close()
        gone = sound.Library(cache=cache).fresh("el")
        kept = os.path.exists(keep) and gone == 0
        # L4 a ' in the temp list path is doubled inside the PowerShell script
        scripts = []
        tts._win = {"el": ("rt", "Microsoft Stefanos")}
        tts._ps = lambda script, timeout: scripts.append(script)
        real_mk = tts.tempfile.mkstemp
        qdir = os.path.join(TMP, "o'brien")
        os.makedirs(qdir, exist_ok=True)
        tts.tempfile.mkstemp = lambda suffix="": real_mk(suffix=suffix, dir=qdir)
        try:
            tts.synth("el", [("Δοκιμή.", os.path.join(TMP, "x.wav"))], choice="windows")
        finally:
            tts.tempfile.mkstemp = real_mk
        quoted = bool(scripts) and "o''brien" in scripts[0]
        report("S16 (review 13o L1-L4) Windows voices: the page's check never runs PowerShell on its thread (a "
               "background lookup), not 'known' while it runs, then known; a failed lookup = unknown + tried later, "
               "the voice cache is NOT deleted for it; a ' in the temp path is escaped in the script",
               quick and not known_during and after and not_tk and unknown and kept and quoted,
               "quick %s, known during %s, after %s, PowerShell on the caller %s; failed -> %s known %s, cache kept %s; "
               "quoted %s" % (quick, known_during, after, calls_ps, got, tts.windows_known(), kept, quoted))
    finally:
        (tts._win, tts._win_retry_t, tts._ps, tts._prefer, tts.piper_bin, tts.piper_voice) = saved


if __name__ == "__main__":
    sys.exit(main())
