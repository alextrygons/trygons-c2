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
    i18n.set_lang("en", save=False)
    bad_n = results.count(False)
    print("SOUND TESTS: %d passed, %d failed" % (len(results) - bad_n, bad_n))
    return 1 if bad_n else 0


if __name__ == "__main__":
    sys.exit(main())
