"""Search and rescue tools on the ground station (CLAUDE.md X, docs/sar_design.md; step PF3 = MOB).

  MB1  the marker = the boat position of the PRESS sample (a later sample never moves it); an old sample = an ESTIMATE
       whose uncertainty grows with age x SOG; the learned current (only when trusted) is stored for the drift estimate
  MB2  MOB: a WARNING on this station + the voice callout + the automatic SAVE (source "mob") + the marker to the
       gateway, the event log; nothing else is sent (no route, no UAV, no mode change) before an answer
  MB3  no gateway / a dead link: the marker, the alarm and the SAVE of this PC still happen; the marker is handed to
       the gateway once it is back (the same id is idempotent), not again after its acknowledgement
  MB4  the offers: one vertical YES / NO at a time on screen 1 (the captain's), in order; nothing is sent before a YES;
       no answer = nothing; another question open = waits; a UAV offer only with a magazine target
  MB5  the return route: ends 30 m BEFORE the person (never at it), PROG_END_STOP, the last waypoint slow, <= 5 kn,
       through the normal program upload, AUTO START only after the route is stored; inside the stand-off distance =
       no route; single operator / copilot = the same path (the gateway's ACCEPT decides, test_gateway SA3)
  MB6  the key M: not in a text field, not with Ctrl, only the chosen key (setting mob_key)
  MB7  another station's MOB: the same WARNING here once, cleared when it is closed; «ΒΡΕΘΗΚΕ» closes it everywhere
  MB8  the pure geometry: the stand-off point down-current when the current is known, else on the line to the boat;
       the drift estimate P(t) = P0 + v t with r(t) = sqrt(r0^2 + (sigma t)^2); no estimate without a trusted current
  MB9  the screens: the MOB button on screen 2's bar 1 and screen 1's bottom bar, the MOB line at the top of the
       notices and as a banner; Greek and English texts; red only as the WARNING alarm

  PS1  PF2 geometry against the IAMSAR formulas: expanding square (leg lengths ceil(k/2) S, 90 degree turns, total
       length), sector (9 legs of R, 120 degree turns, corners on the circle), parallel track (2n points, spacing S,
       total n (Lg - S) + (n - 1) S), creeping line (legs across, advance along the course), port = mirror image
  PS2  the 50-waypoint rule: a boat pattern above 50 is REFUSED with the count and the leg (nothing sent, nothing cut
       silently); CUT AT 50 is the operator's explicit choice; a UAV mission is not limited to 50
  PS3  the boat route is the SAME kind of program as a hand-made one: program.check passes / fails alike, the normal
       prog_upload is used (final HOLD), the speed / radius limits stay
  PS4  nothing is sent without the YES (screen 1's vertical box or the station's own); a UAV mission goes through
       uav_route with an altitude per waypoint; NO = nothing; without a gateway a UAV target is refused
  PS5  the MOB offer «ΜΟΤΙΒΟ ΕΡΕΥΝΑΣ»: the MOB is the datum, the preview is drawn, nothing is sent
  MK1  mark from the video, the boat's calibrated camera: a known point (the AR overlay's own forward model) is marked back
       within centimetres, with roll / pitch; the circle grows with the range
  MK2  a UAV's fixed camera: altitude + attitude + heading + mount, round trip, farther than the boat
  MK3  refusals with their reason: horizon, too uncertain, no calibration, no / old attitude, no / old position, a UAV
       too low, beyond the range, NaN / outside the picture, a bad camera mount
  MK4  accuracy with the stated noise (12000 trials): error and circle per range band, inside the circle >= 78 %
  MK5  the station: a click = ONE marker (point, ESTIMATE, r_m, camera) to the gateway, nothing else sent; refusals
  MK6  the small still image (160 x 120), sent once after the marker's acknowledgement
  MK7  a UAV video click; old attitude / no gateway refused
  MK8  the actions need the YES (route, stand-off route for a person, UAV, delete); pattern datum; rename / person shared
  MK9  screen 2's camera view click + the key K, screen 1's click, the map drawing, a click on a marker
  MK10 texts: Greek and English keys, the ESTIMATE wording, the manuals

  PS6  the panel: opens, switches the pattern, takes typed values, shows the plan numbers and the >50 note, closes
       and clears the preview; Greek and English texts exist

Run:  python test_sar.py      (exit code 0 = all passed)
"""
import math
import os
import re
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_screen2 as ts  # noqa: E402  (its environment: temp settings, sound off, the Rig on the simulator)
import alerts  # noqa: E402
import gwclient  # noqa: E402
import i18n  # noqa: E402
import mob  # noqa: E402
import proto  # noqa: E402
import sar as sarmod  # noqa: E402
import sar_geo  # noqa: E402
import sar_mark  # noqa: E402
import sar_pattern as sp  # noqa: E402
import sarpanel  # noqa: E402
import program  # noqa: E402
import settings  # noqa: E402
import theme  # noqa: E402
import ynbox  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
results = []
LAT, LON = 60.0, 25.0


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


class FakeSound:
    def enabled(self):
        return False

    def __init__(self):
        self.said = []

    def say(self, code, *a, **k):
        self.said.append(code)

    def feedback(self, code):
        self.said.append(code)

    def ack(self):
        pass


class FakeDriver:
    def __init__(self):
        self.question = None
        self.asks = []
        self.w = None

    def ask(self, text, on_yes, on_no, timeout_s, tag=None, head=None):
        self.asks.append({"text": text, "yes": on_yes, "no": on_no, "tag": tag, "head": head, "timeout": timeout_s})


class Env:
    """The real station with its MOB state isolated: own marker folder, recorded sends, a fake sound."""

    def __init__(self, rig, tmp):
        self.rig, self.ap = rig, rig.ap
        ap = self.ap
        self.saved = (ap.sar, ap.sound, ap.driver, ap.prog_upload, ap.auto_start, ap.single_op, ap.gw_status,
                      ap.mob_have, ap.mob_alarmed, ap.mob_queue)
        self.tmp = tmp
        self.asked = []
        self.frames = []
        self.old_ynbox = ynbox.ask
        self.reset()

    def reset(self):
        ap = self.ap
        self.n = 0
        ap.sar = sarmod.LocalSar(os.path.join(self.tmp, "sar%d" % time.time_ns()))
        ap.sound = FakeSound()
        self.sent = []
        self.rig.gateway_like(True)                  # the real tcp link looks like a gateway link; nothing leaves it
        ap.link.send_msg = lambda m: self.sent.append(m)
        self.seq = 0

        def _send(t, *a, **k):
            self.frames.append(t)
            self.seq += 1
            return self.seq
        ap.link.send = _send
        self.frames.clear()
        self.asked.clear()
        ynbox.ask = lambda parent, title, text, **k: (self.asked.append(text), False)[1]    # NO unless a test says
        ap.driver = None
        self.uploads, self.starts = [], []
        def _upload(final=None):
            self.uploads.append((final, [dict(w) for w in ap.prog["wps"]]))
            ap._send_cmd(proto.T_PROG_COMMIT, b"", "program upload")      # the real commit goes through _send_cmd
        ap.prog_upload = _upload
        ap.auto_start = lambda: self.starts.append(time.time())
        ap.single_op = False
        ap.gw_status = {}
        ap.mob_alarmed = set()
        ap.mob_queue = []
        ap.mob_auto_until = 0.0
        ap.mob_auto_arm, ap.mob_auto_seq = False, None
        ap.mob_last_press = 0.0
        ap.mob_keys_down = {}
        ap.mob_refused = {}
        ap.mob_sar_rev = None
        ap.mob_sync_t = 0.0
        ap.mob_search_ok = False                     # PF3's tests are about return / UAV; PF2's tests switch it on
        ap.sar_ctl = sarpanel.PatternCtl(ap)
        ap.sar_preview = None
        for k in list(ap.lamps.local):
            ap.lamps.set_local(k, None)

    def position(self, lat=LAT, lon=LON, **extra):
        t0 = time.time()
        e = {"est_lat": lat, "est_lon": lon, "pos_source": 1, "unc_m": 3.0, "est_sog": 0.0}
        e.update(extra)
        self.rig.extra.clear()
        self.rig.extra.update(e)
        self.rig.pump(1.5, lambda: self.ap.mob_have is not None and abs(self.ap.mob_have["lat"] - lat) < 1e-9 and
                      self.ap.mob_have["t"] > t0)

    def restore(self):
        ap = self.ap
        (ap.sar, ap.sound, ap.driver, ap.prog_upload, ap.auto_start, ap.single_op, ap.gw_status,
         ap.mob_have, ap.mob_alarmed, ap.mob_queue) = self.saved
        ynbox.ask = self.old_ynbox
        self.rig.gateway_like(False)
        for k in list(ap.lamps.local):
            ap.lamps.set_local(k, None)
        self.rig.extra.clear()


FORBIDDEN = (proto.T_PROG_BEGIN, proto.T_PROG_WP, proto.T_PROG_COMMIT, proto.T_AUTO_START, proto.T_SET_MODE, proto.T_DP)


def kinds(sent):
    """The station messages (the driver heartbeat is not one of the MOB's)."""
    return [m.get("t") for m in sent if isinstance(m, dict) and m.get("t") != "driver"]


def east(lat, m):
    return m / (111320.0 * math.cos(math.radians(lat)))


def mb1(env):
    ap = env.ap
    env.position(LAT, LON)
    m1 = ap.mob_press("button")
    env.position(LAT + 0.01, LON + 0.01)                      # the boat moves on: the marker must not
    still = (m1["lat"], m1["lon"]) == (LAT, LON)
    stored = ap.sar.get(m1["id"])
    # an old sample (the telemetry is late / the link is bad) = an ESTIMATE, uncertainty grown by age x SOG
    now = time.time()
    old = mob.capture({"lat": LAT, "lon": LON, "unc_m": 5.0, "sog": 10.0, "t": now - 20.0, "fresh": True}, now)
    new = mob.capture({"lat": LAT, "lon": LON, "unc_m": 5.0, "sog": 10.0, "t": now - 0.5, "fresh": True}, now)
    none = mob.capture(None, now)
    cur = mob.current_of({"current_conf": 0.8, "drift_kn": 1.2, "set_deg": 190.0})
    weak = mob.current_of({"current_conf": 0.2, "drift_kn": 1.2, "set_deg": 190.0})
    nan = mob.current_of({"current_conf": float("nan"), "drift_kn": 1.2, "set_deg": 190.0})
    report("MB1 the marker is the boat position of the PRESS sample (a later sample never moves it); an old sample is "
           "an ESTIMATE with the uncertainty grown by age x SOG (5 m + 20 s x 10 kn); no position ever = none; the "
           "learned current is kept only when trusted",
           m1 and still and stored["lat"] == LAT and not m1["est"] and abs(old["r_m"] - (5 + 20 * 10 * 0.514444)) < 0.01
           and old["uncertain"] and not new["uncertain"] and none is None and cur and cur["set_deg"] == 190.0 and
           weak is None and nan is None,
           "marker %.5f,%.5f (boat now %.5f); old r %.1f m est %s; fresh %s; current %s / weak %s" % (
               m1["lat"], m1["lon"], ap.mob_have["lat"], old["r_m"], old["uncertain"], new["uncertain"], cur, weak))
    env.reset()


def mb2(env):
    ap = env.ap
    i18n.set_lang("el", save=False)
    env.position()
    env.sent.clear()
    m = ap.mob_press("button")
    code = "mob_" + m["id"]
    lamp = ap.lamps.local.get(code)
    save = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "save"]
    add = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "sar_mark"]
    others = [k for k in kinds(env.sent) if k not in ("save", "sar_mark")] + \
        [f for f in env.frames if f in FORBIDDEN]
    report("MB2 MOB: a WARNING on this station (Greek text) + the voice callout + the automatic SAVE (source mob) + "
           "the marker to the gateway + the event log; before an answer NOTHING else is sent (no route, no UAV, no "
           "mode, no command)",
           lamp and lamp[1] == alerts.WARNING and "ΑΝΘΡΩΠΟΣ ΣΤΗ ΘΑΛΑΣΣΑ" in lamp[0] and ap.lamps.on(alerts.WARNING) and
           "sar.mob" in ap.sound.said and len(save) == 1 and save[0]["source"] == "mob" and len(add) == 1 and
           add[0]["op"] == "add" and add[0]["m"]["id"] == m["id"] and add[0]["m"]["person"] is True and not others
           and not env.uploads and not env.starts and any("MOB" in e for e in ap._evlog),
           "lamp %s; sound %s; save %s; marker msgs %s; other sends %s; uploads %s" % (
               lamp, ap.sound.said, [s_["source"] for s_ in save], len(add), others, env.uploads))
    ap.master_ack()
    still = ap.sar.get(m["id"]) is not None and len(ap.sar.open_mobs()) == 1 and ap.mob_lines()
    silent = not ap.lamps.on(alerts.WARNING)
    env.reset()
    report("MB2b ACK silences this PC's lamps and sound; the marker and its MOB line stay until it is closed",
           still and silent, "marker + line kept after ACK: %s; lamp off %s" % (bool(still), silent))


def mb3(env):
    ap = env.ap
    env.position()
    env.rig.gateway_like(False)                      # not a gateway link (USB / the gateway is gone): nothing can be sent
    env.sent.clear()
    m = ap.mob_press("button")
    local = ap.sar.get(m["id"]) is not None and m["id"] in {x["id"] for x in ap.sar.unsent()}
    alarm = ("mob_" + m["id"]) in ap.lamps.local and "sar.mob" in ap.sound.said
    no_send = not env.sent
    env.rig.gateway_like(True)                       # the gateway is back
    ap.link.send_msg = lambda m: env.sent.append(m)
    ap.mob_sync_t = 0.0
    ap.mob_sync({"rev": 0, "n_open": 0})
    adds = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "add"]
    gets = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "sar_get"]
    ap.on_sar_ack({"ok": True, "id": m["id"], "code": 0})
    env.sent.clear()
    ap.mob_sync_t = 0.0
    ap.mob_sync({"rev": 0, "n_open": 0})
    again = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "add"]
    # no position at all: the alarm + SAVE still, no marker
    ap.mob_have = None
    ap.mob_last_press = 0.0                           # (PF3-r: a second MOB within 2 s would be the same press)
    ap.t_rx = ap.t_rx
    ap.sar = sarmod.LocalSar(os.path.join(env.tmp, "nopos%d" % time.time_ns()))
    ap.lamps.local.clear()
    env.sent.clear()
    r = ap.mob_press("key")
    nopos_alarm = any(k.startswith("mob_nopos") for k in ap.lamps.local) and not ap.sar.view() and \
        any(isinstance(x, dict) and x.get("t") == "save" for x in env.sent)
    report("MB3 no gateway / a dead link: the marker, the WARNING and the sound still happen; the marker goes to the "
           "gateway once it is back (a list request + ONE add), not again after its acknowledgement; no position at "
           "all = the alarm + SAVE without a marker",
           local and alarm and no_send and len(adds) == 1 and len(gets) == 1 and not again and r is None and
           nopos_alarm,
           "local %s; alarm %s; sent while away %d; on return adds %d gets %d; after ack %d; no-position alarm %s" % (
               local, alarm, 0 if no_send else 1, len(adds), len(gets), len(again), nopos_alarm))
    env.reset()


def mb4(env):
    ap = env.ap
    i18n.set_lang("el", save=False)
    env.position()
    dr = FakeDriver()
    dr.w = env.rig.root
    ap.driver = dr
    ap.gw_status = {"uav": {"enabled": True, "next": 2, "slots": []}}
    env.sent.clear()
    m = ap.mob_press("button")
    env.rig.pump(0.8, lambda: dr.asks)
    first = dr.asks[0] if dr.asks else None
    before = [k for k in kinds(env.sent) if k not in ("save", "sar_mark")] + [f for f in env.frames if f in FORBIDDEN]
    no_upload = not env.uploads
    # another question open on screen 1: the offer waits (never replaces a copilot's ACCEPT)
    dr.question = ("x",)
    n_before = len(dr.asks)
    first["no"]()
    env.rig.pump(1.2)
    waited = len(dr.asks) == n_before
    dr.question = None
    env.rig.pump(2.6, lambda: len(dr.asks) > n_before)
    second = dr.asks[1] if len(dr.asks) > 1 else None
    env.sent.clear()
    if second:
        second["yes"]()
    uav = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "uav_route"]
    prep = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "uav_launch_prepare"]
    report("MB4 the offers: the return offer first (the vertical YES / NO on screen 1, 60 s), nothing sent before a "
           "YES; another open question makes it wait; then the UAV offer; its YES = uav_route (next UAV, 30 m, over the "
           "MOB) + the normal launch preparation (the captain's approval stays)",
           first and first["tag"] == ("mob", "return") and first["timeout"] == 60.0 and "MOB 1" in first["text"] and
           not before and no_upload and waited and second and second["tag"] == ("mob", "uav") and len(uav) == 1 and
           uav[0]["target"] == "next" and uav[0]["wps"][0]["alt"] == 30.0 and
           abs(uav[0]["wps"][0]["lat"] - m["lat"]) < 1e-6 and len(prep) == 1 and prep[0].get("count") == 1,
           "first %s; sent before %s; waited %s; second %s; uav %s; prepare %s" % (
               first and first["tag"], before, waited, second and second["tag"], uav, prep))
    # no UAV in the magazine = no UAV offer; the search offer is only there when PF2's panel exists
    env.reset()
    ap.gw_status = {}
    mm = {"id": "mob_x", "n": 1, "lat": LAT, "lon": LON}
    o1 = mob.offers(mm, None, None)
    o2 = mob.offers(mm, None, {"enabled": True, "next": None, "slots": [{"slot": 3, "flying": True}]}, True)
    o3 = mob.offers(mm, None, {"enabled": True, "next": None, "slots": []})
    report("MB4b offers by situation: return only (no magazine), + UAV with a next / a flying UAV, + the search "
           "pattern only when it exists (the PF2 hook)",
           o1 == ["return"] and o2 == ["return", "uav", "search"] and o3 == ["return"], "%s %s %s" % (o1, o2, o3))
    env.reset()


def mb5(env):
    ap = env.ap
    i18n.set_lang("en", save=False)
    env.position()
    boat = (LAT, LON)
    mark = (LAT + 300.0 / 111320.0, LON)                                # 300 m north
    m = mob.new_marker("mob_t5", 1, {"lat": mark[0], "lon": mark[1], "r_m": 3.0, "uncertain": False}, time.time(), "a")
    ap.sar.add(m)
    ap.mob_return(m)
    f, wps = env.uploads[-1] if env.uploads else (None, [])
    end = (wps[-1]["lat"], wps[-1]["lon"]) if wps else (0, 0)
    d_end = sar_geo.distance_bearing(end, mark)[0] if wps else -1
    armed = ap.mob_auto_until > time.time()
    s0 = len(env.starts)
    seq = ap.mob_auto_seq
    ap.mob_route_stored(seq + 5)                                        # PF3-r: an unrelated upload's ACK starts nothing
    ap.mob_route_stored()
    unrelated = len(env.starts) - s0
    ap.mob_route_stored(seq)
    ap.mob_route_stored(seq)                                            # a second ACK does not start twice
    started = len(env.starts) - s0
    # inside the stand-off distance: no route at all, and the propeller warning
    n_up = len(env.uploads)
    near = mob.new_marker("mob_t5b", 2, {"lat": LAT + 20.0 / 111320.0, "lon": LON, "r_m": 3.0, "uncertain": False},
                          time.time(), "a")
    ap.sar.add(near)
    ap.mob_return(near)
    no_route = len(env.uploads) == n_up and "PROPELLER" in ap.msg_text
    # the route is also refused when the boat has no position
    env.rig.extra.clear()
    ap.t = dict(ap.t, pos_source=0)
    n_up = len(env.uploads)
    ap.mob_return(m)
    nopos = len(env.uploads) == n_up
    env.position()
    # a current: the approach point lies on the down-current side
    cur_t = {"current_conf": 0.9, "drift_kn": 1.0, "set_deg": 90.0}      # flows toward the east
    env.rig.extra.update(cur_t)
    env.position(LAT, LON, **cur_t)
    ap.mob_return(m)
    f2, wps2 = env.uploads[-1]
    east_of_marker = wps2[-1]["lon"] > mark[1] + 1e-6 and abs(wps2[-1]["lat"] - mark[0]) < 1e-6
    report("MB5 the return route goes through the normal upload: ends 30 m BEFORE the person (never at it), final "
           "action STOP, the last waypoint slow, <= 5 kn, AUTO START only once the route is stored (one start per "
           "route); inside the stand-off distance = no route + the propeller warning; no position = nothing; a trusted "
           "current moves the approach to the down-current side",
           f == sar_geo.PROG_END_STOP and len(wps) == 2 and abs(d_end - 30.0) < 0.5 and wps[-1]["action"] == 1 and
           all(w["kn"] <= 5.0 for w in wps) and armed and seq is not None and unrelated == 0 and started == 1 and no_route and nopos and east_of_marker and
           abs(wps[0]["lat"] - boat[0]) < 1e-9,
           "final %s; end %.1f m before; last action %s; armed %s; starts %d; inside no route %s; no position %s; "
           "current -> east side %s" % (f, d_end, wps and wps[-1]["action"], armed, started, no_route, nopos,
                                        east_of_marker))
    env.reset()
    # single operator / no driver screen: the settings-style box (ynbox), NO = nothing is sent
    ap.single_op = True
    asked = []
    old = ynbox.ask
    ynbox.ask = lambda parent, title, text, **k: (asked.append(text), False)[1]
    try:
        env.position()
        ap.gw_status = {}
        ap.mob_press("button")
        env.rig.pump(1.6, lambda: len(asked) >= 1)
        no_ups = not env.uploads
        env.rig.pump(1.2)
    finally:
        ynbox.ask = old
    report("MB5b single operator / no screen 1: the same offer as a vertical YES / NO box of this station; NO = no "
           "route, nothing sent",
           len(asked) == 1 and no_ups and not env.uploads and "MOB 1" in asked[0], "asked %s; uploads %s" % (
               [a[:30] for a in asked], env.uploads))
    env.reset()


def mb6(env):
    ap = env.ap
    env.position()
    calls = []
    orig = ap.mob_press
    ap.mob_press = lambda source="button", now=None: calls.append(source)
    import tkinter as tk
    ent = tk.Entry(env.rig.root)

    def ev(keysym="m", widget=None, state=0):
        return types.SimpleNamespace(keysym=keysym, widget=widget, state=state)
    try:
        ap.mob_key_press(ev("m"), now=100.0)
        for i in range(1, 25):                              # PF3-r: auto-repeat = KeyPress again and again, no release
            ap.mob_key_press(ev("m"), now=100.0 + 0.03 * i)
        n_repeat = len(calls)
        ap.mob_key_release(ev("m"))
        ap.mob_key_press(ev("M"), now=101.0)                # a real second press after the release
        n_after_release = len(calls)
        ap.mob_key_press(ev("m"), now=103.5)                # held, but no repeat for > KEY_STALE_S (a lost release)
        n_stale = len(calls)
        ap.mob_key_press(ev("m", ent), now=200.0)           # typing into a field
        ap.mob_key_press(ev("m", None, 0x4), now=200.0)     # Ctrl+M
        ap.mob_key_press(ev("k"), now=200.0)
        n_default = len(calls)
        settings.put("mob_key", "k")
        ap.mob_key_press(ev("k"), now=300.0)
        ap.mob_key_press(ev("m"), now=300.0)
        n_custom = len(calls) - n_default
        settings.put("mob_key", None)
    finally:
        ap.mob_press = orig
        ent.destroy()
    src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    report("MB6 the MOB key (default M, setting mob_key): works from the station window (a held key = ONE press, a release or 1 s without a repeat re-arms it), not in a text field, not with "
           "Ctrl, only the chosen key; it is a keyboard key of the station, never a joystick button",
           n_repeat == 1 and n_after_release == 2 and n_stale == 3 and n_default == 3 and n_custom == 1 and
           calls == ["key"] * 4 and
           "root.bind_all(\"<KeyPress>\", self.mob_key_press" in src and "mob" not in open(
               os.path.join(HERE, "shorejoy.py"), encoding="utf-8").read().lower().replace("mobile", ""),
           "after 25 repeats %d, after release %d, stale %d; default presses %d, custom %d, calls %s" % (
               n_repeat, n_after_release, n_stale, n_default, n_custom, calls))
    env.reset()


def mb7(env):
    ap = env.ap
    i18n.set_lang("en", save=False)
    rm = {"id": "mob_20261109-120000_7", "kind": "mob", "n": 7, "lat": LAT, "lon": LON, "t_utc": "2026-11-09T12:00:00Z",
          "r_m": 3.0, "est": False, "person": True, "closed": None, "drift": None}
    ap.on_sar_markers({"rev": 4, "list": [rm]})
    lamp = ap.lamps.local.get("mob_" + rm["id"])
    said1 = ap.sound.said.count("sar.mob")
    ap.on_sar_markers({"rev": 5, "list": [rm]})                       # the same list again: no second alarm
    said2 = ap.sound.said.count("sar.mob")
    closed = dict(rm, closed={"t_utc": "2026-11-09T12:10:00Z", "by": "bob", "why": "found"})
    ap.on_sar_markers({"rev": 6, "list": [closed]})
    gone = "mob_" + rm["id"] not in ap.lamps.local
    # «ΒΡΕΘΗΚΕ» here
    env.position()
    m = ap.mob_press("button")
    env.sent.clear()
    ap.mob_found(m["id"])
    cl = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "close"]
    report("MB7 another station's MOB raises the same WARNING here ONCE; its closing clears it; FOUND closes the "
           "marker here (time kept), clears the alarm and tells the gateway (close, found)",
           lamp and lamp[1] == alerts.WARNING and said1 == 1 and said2 == 1 and gone and
           ap.sar.get(m["id"])["closed"]["why"] == "found" and ("mob_" + m["id"]) not in ap.lamps.local and
           len(cl) == 1 and cl[0]["m"]["why"] == "found" and not ap.sar.open_mobs(),
           "lamp %s; said %d/%d; cleared %s; close msgs %d" % (lamp, said1, said2, gone, len(cl)))
    env.reset()


def sarmod_id_ok(i):
    return bool(re.match(r"^[A-Za-z0-9_.\-]{1,48}$", i))


def mb10(env):
    """PF3-r: MED-1 FOUND (button + key, YES / NO, log time, nothing re-opened), MED-2 double click / auto-repeat,
    LOW-5 refused markers not resent, LOW-6 unique ids across stations."""
    ap = env.ap
    i18n.set_lang("en", save=False)
    env.position()
    t0 = time.time()
    m1 = ap.mob_press("button", now=t0)
    again = [ap.mob_press("button", now=t0 + 0.2 * i) for i in range(1, 8)]
    later = ap.mob_press("button", now=t0 + 2.5)
    one = m1 is not None and all(x is None for x in again) and later is not None and len(ap.sar.open_mobs()) == 2
    ids = {mob.marker_id(t0, 1, st) for st in ("s1", "s2")}
    same_station = mob.marker_id(t0, 1, "s1") != mob.marker_id(t0, 1, "s1")
    uniq = len(ids) == 2 and same_station and all(sarmod_id_ok(i) for i in ids)
    items = [it for it in ap.mode_bar_items() if it["tag"] == "mob_found"]
    ynbox.ask = lambda parent, title, text, **k: (env.asked.append(text), False)[1]
    if items:
        items[0]["action"]()
    no_close = len(ap.sar.open_mobs()) == 2
    _yes(env)
    env.sent.clear()
    if items:
        items[0]["action"]()
    one_closed = len(ap.sar.open_mobs()) == 1 and ap.sar.get(later["id"])["closed"]["why"] == "found"
    cl = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "close"]
    asked_yn = bool(env.asked) and "MOB 2" in env.asked[-1]
    lamp_clear = ("mob_" + later["id"]) not in ap.lamps.local and ("mob_" + m1["id"]) in ap.lamps.local
    reload = sarmod.LocalSar(ap.sar.folder)
    stays = reload.get(later["id"])["closed"]["t_utc"].endswith("Z") and later["id"] not in [
        x["id"] for x in reload.open_mobs()]
    stale = dict(later, closed=None)
    env.sent.clear()
    ap.on_sar_markers({"rev": 9, "list": [stale]})
    kept = ap.sar.get(later["id"])["closed"] is not None and ("mob_" + later["id"]) not in ap.lamps.local
    resent = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "close" and x["m"]["id"] == later["id"]]
    cm = {"id": "mob_other_9", "kind": "mob", "n": 9, "lat": LAT, "lon": LON, "t_utc": "2026-11-09T12:00:00Z",
          "r_m": 1.0, "est": False, "person": True, "drift": None,
          "closed": {"t_utc": "2026-11-09T12:05:00Z", "by": "x", "why": "found"}}
    said = ap.sound.said.count("sar.mob")
    ap.on_sar_markers({"rev": 10, "list": [cm]})
    no_realarm = ap.sound.said.count("sar.mob") == said and ("mob_" + cm["id"]) not in ap.lamps.local
    env.asked.clear()
    ap.mob_key_press(types.SimpleNamespace(keysym="f", widget=None, state=0), now=500.0)
    ap.mob_key_press(types.SimpleNamespace(keysym="f", widget=None, state=0), now=500.05)
    key_q = sum(1 for t in env.asked if t.startswith("MOB 1"))
    env.reset()
    env.position()
    mm = ap.mob_press("button", now=time.time())
    env.sent.clear()
    ap.on_sar_ack({"ok": False, "id": mm["id"], "code": 128})
    msg1 = ap.msg_text
    ap.msg_text = ""
    for _ in range(3):
        ap.mob_sync_t = 0.0
        ap.mob_sync({"rev": 5, "n_open": 0})
    adds = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "add"]
    ap.on_sar_ack({"ok": False, "id": mm["id"], "code": 128})
    once = ap.msg_text == "" and not adds and msg1 != ""
    report("MB10 PF3-r: a double click / burst of presses = ONE MOB; marker ids differ across stations; FOUND (mode "
           "bar button + key F) asks a YES / NO, NO closes nothing, YES closes the newest (time in the log, not "
           "reopened by a reload or the gateway's stale list, the close is said again, no new WARNING for a closed "
           "one); a refused marker is reported once and not resent",
           one and uniq and bool(items) and no_close and one_closed and len(cl) == 1 and asked_yn and lamp_clear and
           stays and kept and len(resent) == 1 and no_realarm and key_q == 1 and once,
           "one %s uniq %s button %s no_close %s closed %s close msgs %d asked %s lamp %s reload %s kept %s resent %d "
           "no_realarm %s key questions %d once %s" % (one, uniq, bool(items), no_close, one_closed, len(cl), asked_yn,
                                                       lamp_clear, stays, kept, len(resent), no_realarm, key_q, once))
    env.reset()


def mb8():
    boat = (LAT, LON)
    mark = (LAT + 0.005, LON + 0.002)
    a = sar_geo.standoff_point(boat, mark, {"set_deg": 90.0, "kn": 1.0, "conf": 0.9}, 30.0)
    b = sar_geo.standoff_point(boat, mark, None, 30.0)
    c = sar_geo.standoff_point(boat, mark, {"set_deg": 90.0, "kn": 1.0, "conf": 0.1}, 30.0)
    da, ba = sar_geo.distance_bearing(mark, a[:2])
    db, bb = sar_geo.distance_bearing(mark, b[:2])
    _d, brg_to_boat = sar_geo.distance_bearing(mark, boat)
    t0 = time.time() - 600.0
    m = mob.new_marker("m", 1, {"lat": LAT, "lon": LON, "r_m": 5.0, "uncertain": False}, t0, "a",
                       drift={"set_deg": 90.0, "kn": 1.0, "conf": 0.9})
    est = mob.drift_estimate(m, t0 + 600.0)
    d_est, b_est = sar_geo.distance_bearing((LAT, LON), (est["lat"], est["lon"]))
    sig = max(0.1, 0.5) * 0.514444
    want_r = math.hypot(5.0, sig * 600.0)
    nodrift = mob.drift_estimate(dict(m, drift=None), t0 + 600.0)
    line = mob.line(m, (LAT, LON + east(LAT, 300.0)), t0 + 600.0)
    inside = sar_geo.standoff_route(boat, (LAT + 25.0 / 111320.0, LON), None, {"standoff_m": 30.0})
    ok_route = sar_geo.standoff_route(boat, mark, None, {"standoff_m": 30.0, "approach_kn": 5.0})
    prog = sar_geo.to_program(ok_route)
    report("MB8 pure geometry: the approach point 30 m from the person on the DOWN-current side when the current is "
           "trusted (else on the line to the boat); the drift ESTIMATE moves 1 kn x 600 s east with r = sqrt(5^2 + "
           "(sigma t)^2), none without a trusted current; the MOB line shows distance, bearing and elapsed time; "
           "inside the stand-off distance = no route; the program ends with STOP and a slow last waypoint",
           abs(da - 30.0) < 0.2 and abs(ba - 90.0) < 0.5 and a[2] == "current" and abs(db - 30.0) < 0.2 and
           abs(bb - brg_to_boat) < 0.5 and b[2] == "straight" and c[2] == "straight" and
           abs(d_est - 0.514444 * 600.0) < 1.0 and abs(b_est - 90.0) < 0.5 and abs(est["r_m"] - want_r) < 1.0 and
           nodrift is None and "10:00" in line and "300 m" in line and "270" in line and
           inside.get("why") == "inside" and prog["final"] == 0 and prog["wps"][-1]["action"] == 1,
           "down-current %.1f m @ %.0f; straight %.1f m @ %.0f (boat at %.0f); drift %.0f m @ %.0f r %.0f (want %.0f); "
           "line '%s'" % (da, ba, db, bb, brg_to_boat, d_est, b_est, est["r_m"], want_r, line))


def mb9(env):
    ap = env.ap
    import hud
    import driver
    from unittest import mock
    i18n.set_lang("el", save=False)
    env.position()
    m = ap.mob_press("button")
    ap.render(force=True)
    mob_btn = [(o["k"], o["tag"], o["s"], None, None, None, o["fill"] == theme.WARN) for o in ap.ov.ops
               if o.get("tag") == "mob" and o["k"] == "button"]
    banners = [b for b in ap.banner_list() if b[0] == "mob"]
    notices = driver.DriverScreen.notices(types.SimpleNamespace(app=ap, front_live=True), ap.t, {})
    top = notices[0] if notices else None
    s = mock.MagicMock()
    s.mob, s.asst, s.hail, s.vision, s.vision_cap, s.slam_restore, s.rec, s.scr, s.t = True, None, None, False, False, \
        False, None, False, None
    s.night, s.declutter, s.video_scale, s.ar_on, s.clock, s.mission = False, "FULL", "fill", False, "10:00", "00:00"
    pal = {k: "#000000" for k in ("warn", "grey", "caution", "main", "dim")}
    groups = hud.Hud.bar_groups(types.SimpleNamespace(pal=pal, edit=False), s)
    hud_btn = [it for g in groups for it in g if it[3] == "ctl_mob"]
    s.mob = False
    hud_off = [it for g in hud.Hud.bar_groups(types.SimpleNamespace(pal=pal, edit=False), s) for it in g if it[3] == "ctl_mob"]
    el = i18n.T("mob.alarm", n=1)
    i18n.set_lang("en", save=False)
    en = i18n.T("mob.alarm", n=1)
    i18n.set_lang("el", save=False)
    red_btn = hud_btn and hud_btn[0][5] == pal["warn"] and hud_off and hud_off[0][5] is None
    report("MB9 the screens: a MOB button on screen 2's bar 1 (inverted while a MOB is open) and in screen 1's bottom "
           "bar (red only while a MOB alarm is open), the MOB line at the TOP of screen 1's notices and as a banner on "
           "screen 2, Greek and English alarm texts",
           len(mob_btn) == 1 and mob_btn[0][2] == "MOB" and mob_btn[0][6] is True and len(banners) == 1 and
           "MOB 1" in banners[0][1] and top and top[0] == "mob" and top[2] == "warn" and "MOB 1" in top[1] and
           len(hud_btn) == 1 and red_btn and "ΑΝΘΡΩΠΟΣ" in el and "MAN OVERBOARD" in en,
           "bar1 %d; banner %s; top notice %s; hud button %s / %s" % (
               len(mob_btn), [b[1][:40] for b in banners], top and top[:2], hud_btn and hud_btn[0][:2],
               hud_off and hud_off[0][5]))
    env.reset()


def _angle_diff(a, b):
    return (b - a + 180.0) % 360.0 - 180.0


def ps1():
    d = (60.0, 25.0)
    ok = True
    notes = []
    # expanding square: leg k = ceil(k/2) S, right turns of 90 degrees, total = the closed formula
    for S, C, n in ((50.0, 0.0, 9), (100.0, 37.0, 16), (200.0, 270.0, 25)):
        pl = sp.plan("ss", d, {"S": S, "C": C, "legs": n, "kn": 5})
        lens = [l["m"] for l in pl["legs"]]
        brgs = [l["brg"] for l in pl["legs"]]
        want = [math.ceil((k + 1) / 2.0) * S for k in range(n)]
        turns = [_angle_diff(brgs[i], brgs[i + 1]) for i in range(n - 1)]
        good = (all(abs(a - b) < 0.5 for a, b in zip(lens, want)) and all(abs(t - 90.0) < 0.3 for t in turns) and
                abs(brgs[0] - C) < 0.3 and abs(pl["total_m"] - sp.expanding_total(S, n)) < 1.0 and pl["n"] == n + 1)
        ok &= good
        notes.append("SS S%g n%d %s" % (S, n, good))
    pl = sp.plan("ss", d, {"S": 100.0, "C": 0.0, "legs": 9, "kn": 5, "turn": "port"})
    brgs = [l["brg"] for l in pl["legs"]]
    port = all(abs(_angle_diff(brgs[i], brgs[i + 1]) + 90.0) < 0.3 for i in range(8))
    h = sp.plan("ss", d, {"S": 100.0, "C": 0.0, "R": 450.0, "kn": 5})
    half = len(h["legs"]) == 2 * 4 + 1
    ok &= port and half
    notes.append("SS port %s half-side %s" % (port, half))
    # sector: 9 legs of R, 120 degree turns, the corner points on the circle at C, C+60 ... C+300, 10 waypoints
    for R, C in ((300.0, 0.0), (1000.0, 45.0)):
        pl = sp.plan("vs", d, {"S": 100.0, "C": C, "R": R, "kn": 5})
        lens = [l["m"] for l in pl["legs"]]
        brgs = [l["brg"] for l in pl["legs"]]
        turns = [_angle_diff(brgs[i], brgs[i + 1]) for i in range(8)]
        w = pl["wps"]
        corners = [w[1], w[2], w[4], w[5], w[7], w[8]]
        angs = [(C + a) % 360.0 for a in (0, 60, 240, 300, 120, 180)]
        on_circle = all(abs(sar_geo.distance_bearing(d, c)[0] - R) < 0.5 and
                        abs(_angle_diff(sar_geo.distance_bearing(d, c)[1], a)) < 0.3 for c, a in zip(corners, angs))
        good = (pl["n"] == 10 and len(lens) == 9 and all(abs(x - R) < 0.5 for x in lens) and on_circle and
                abs(pl["total_m"] - 9 * R) < 1.0 and abs(turns[0] - 120.0) < 0.3 and abs(turns[1] - 120.0) < 0.3 and
                abs(w[3][0] - d[0]) < 1e-9 and abs(w[9][1] - d[1]) < 1e-9)
        ok &= good
        notes.append("VS R%g %s" % (R, good))
    # parallel track: 2n points, search legs Lg - S, cross legs S, spacing S, total n (Lg - S) + (n - 1) S
    for S, W, Lg, C in ((100.0, 500.0, 1000.0, 45.0), (80.0, 330.0, 900.0, 200.0)):
        pl = sp.plan("ps", d, {"S": S, "C": C, "W": W, "Lg": Lg, "kn": 5})
        n = int(math.ceil(W / S))
        lens = [l["m"] for l in pl["legs"]]
        search, cross = lens[0::2], lens[1::2]
        brgs = [l["brg"] for l in pl["legs"]]
        along = all(abs(_angle_diff(brgs[i], (C + (0 if (i // 2) % 2 == 0 else 180)) % 360.0)) < 0.3
                    for i in range(0, len(brgs), 2))
        good = (pl["n"] == 2 * n and all(abs(x - (Lg - S)) < 0.5 for x in search) and
                all(abs(x - S) < 0.5 for x in cross) and abs(pl["total_m"] - sp.parallel_total(S, W, Lg)) < 1.0 and
                abs(pl["total_m"] - (n * (Lg - S) + (n - 1) * S)) < 1.0 and along and
                all(abs(_angle_diff(b, (C + 90.0) % 360.0)) < 0.3 for b in brgs[1::2]))
        ok &= good
        notes.append("PS S%g %s" % (S, good))
    # creeping line: the legs run across (C - 90 / C + 90), the search advances along C
    pl = sp.plan("cs", d, {"S": 100.0, "C": 45.0, "W": 500.0, "Lg": 1000.0, "kn": 5})
    brgs = [l["brg"] for l in pl["legs"]]
    good = (all(min(abs(_angle_diff(b, 315.0)), abs(_angle_diff(b, 135.0))) < 0.3 for b in brgs[0::2]) and
            all(abs(_angle_diff(b, 45.0)) < 0.3 for b in brgs[1::2]) and abs(pl["total_m"] - 4900.0) < 1.0)
    ok &= good
    notes.append("CS %s" % good)
    pl = sp.plan("ss", d, {"S": 100.0, "C": 0.0, "legs": 9, "kn": 6})
    tm = abs(pl["time_s"] - pl["total_m"] / (6 * 1852.0 / 3600.0)) < 0.5 and abs(pl["area_m2"] - pl["total_m"] * 100.0) < 1
    bad = [sp.plan("ss", d, {"S": 0.0, "legs": 9}), sp.plan("ss", d, {"S": float("nan"), "legs": 9}),
           sp.plan("ps", d, {"S": 100.0, "W": 500.0, "Lg": 50.0}), sp.plan("vs", d, {"S": 100.0, "R": 2.0}),
           sp.plan("xx", d, {"S": 100.0}), sp.plan("ss", (95.0, 0.0), {"S": 100.0, "legs": 9})]
    nobad = all(b.get("why") and "wps" not in b for b in bad)
    ok &= tm and nobad
    report("PS1 geometry against the IAMSAR formulas: expanding square (leg lengths, 90 degree turns, total, port "
           "mirror, half-side rule), sector (9 legs of R, 120 degree turns, corners on the circle, total 9R), parallel "
           "track (2n points, spacing S, total n(Lg-S)+(n-1)S), creeping line, time / area; bad input = no plan",
           ok, "; ".join(notes) + "; time/area %s; bad input %s" % (tm, nobad))


def _pat_ap(env, kind="ss", **p):
    ap = env.ap
    ctl = ap.sar_ctl
    ctl.kind = kind
    for k, v in p.items():
        ctl.p[k] = v
    ctl.set_datum(LAT, LON, "typed")
    return ctl


def _yes(env):
    ynbox.ask = lambda parent, title, text, **k: (env.asked.append(text), True)[1]


def ps2(env):
    ap = env.ap
    i18n.set_lang("en", save=False)
    ctl = _pat_ap(env, "ss", S=50.0, R=1500.0)                      # 61 legs = 62 waypoints
    pl = ctl.plan
    chk = sp.boat_check(pl)
    n_big = pl["n"]
    ctl.choose("boat")
    refused = not env.uploads and not env.asked and "62" in ap.msg_text and "50" in ap.msg_text
    full = len(pl["wps"]) == n_big                                    # nothing was cut by the refusal
    ctl.cut = True
    _yes(env)
    ctl.choose("boat")
    up = env.uploads[-1] if env.uploads else (None, [])
    cut_ok = len(up[1]) == 50 and abs(up[1][49]["lat"] - pl["wps"][49][0]) < 1e-9
    env.reset()
    ap.gw_status = {"uav": {"enabled": True, "next": 1, "slots": []}}
    ctl = _pat_ap(env, "ss", S=50.0, R=1500.0)
    _yes(env)
    env.asked.clear()
    asked62 = ctl.choose("next")
    ur = [m for m in env.sent if isinstance(m, dict) and m.get("t") == "uav_route"]
    uav_refused = asked62 is False and not ur and not env.asked and "20" in ap.msg_text      # PF2-r MED-2
    ctl = _pat_ap(env, "ss", S=50.0, R=500.0)                         # 21 legs = 22 > 20 ... use 19 legs = 20 points
    ctl.p["legs"] = 19
    ctl.set_datum(LAT, LON, "typed")
    ctl.choose("next")
    ur = [m for m in env.sent if isinstance(m, dict) and m.get("t") == "uav_route"]
    uav_ok = uav_refused and ctl.plan and ctl.plan["n"] == 20 and len(ur) == 1 and len(ur[0]["wps"]) == 20
    report("PS2 the 50-waypoint rule: 62 waypoints for the boat = REFUSED with the count (nothing sent, nothing cut "
           "silently), CUT AT 50 is the operator's choice (ends on leg 49, 50 waypoints uploaded), a UAV mission "
           "over the gateway's 20-waypoint limit is refused before the YES, 20 points go (the %d-point plan stays a boat / split matter)" % n_big,
           chk and chk["n"] == 62 and chk["cut_leg"] == 49 and refused and full and cut_ok and uav_ok,
           "check %s; refused %s; uncut %s; cut %s; uav %s" % (chk, refused, full, cut_ok, uav_ok))
    env.reset()


def ps3(env):
    ap = env.ap
    ctl = _pat_ap(env, "ps", S=100.0, W=500.0, Lg=1000.0, C=45.0)
    pl = ctl.plan
    prog = sp.to_boat_program(pl)
    problems = program.check(prog)
    hand = {"final": 1, "wps": [{"lat": LAT, "lon": LON, "kn": 5.0, "radius": 25.0, "action": 0},
                                {"lat": LAT + 0.01, "lon": LON, "kn": 5.0, "radius": 25.0, "action": 0}]}
    same_ok = not problems and not program.check(hand)
    fast = sp.plan("ps", (LAT, LON), {"S": 100.0, "W": 500.0, "Lg": 1000.0, "C": 45.0, "kn": 40.0})
    fast_prog = sp.to_boat_program(fast)
    hand_fast = dict(hand, wps=[dict(w, kn=40.0) for w in hand["wps"]])
    fast_same = bool(program.check(fast_prog)) and bool(program.check(hand_fast))
    tiny = sp.to_boat_program(sp.plan("ps", (LAT, LON), {"S": 8.0, "W": 40.0, "Lg": 100.0, "C": 0.0, "kn": 5.0}))
    radius_ok = all(5.0 <= w["radius"] <= 500.0 for w in tiny["wps"]) and not program.check(tiny)
    _yes(env)
    ctl.choose("boat")
    f, wps = env.uploads[-1] if env.uploads else (None, [])
    used = (f == sar_geo.PROG_END_STOP and prog["final"] == 0 and len(wps) == pl["n"] and abs(wps[0]["lat"] - pl["wps"][0][0]) < 1e-9 and
            ap.route_dirty and ap.prog["wps"][0]["kn"] == 5.0)
    report("PS3 the boat route is the same kind of program as a hand-made one: program.check passes / fails alike "
           "(a 40 kn pattern fails like a 40 kn hand route), small spacing keeps the radius legal, the normal "
           "prog_upload is used with the final action STOP (PF2-r: arrive and stop, never HOLD) and the pattern's first point first",
           same_ok and fast_same and radius_ok and used,
           "problems %s; same %s; fast %s; radius %s; used %s (final %s, %d wps)" % (
               problems, same_ok, fast_same, radius_ok, used, f, len(wps)))
    env.reset()


def ps4(env):
    ap = env.ap
    i18n.set_lang("en", save=False)
    dr = FakeDriver()
    dr.w = env.rig.root
    ap.driver = dr
    ap.gw_status = {"uav": {"enabled": True, "next": 2, "slots": [{"slot": 2, "registered": True, "present": True}]}}
    ctl = _pat_ap(env, "vs", R=400.0)
    boxes = []
    old_box = sarpanel.uavroute.TargetBox
    sarpanel.uavroute.TargetBox = lambda master, uav, cb: boxes.append(cb)
    try:
        ctl.send()
    finally:
        sarpanel.uavroute.TargetBox = old_box
    target_box = len(boxes) == 1 and not env.sent and not env.uploads
    boxes[0]("boat")
    ask = dr.asks[-1] if dr.asks else None
    before = (not env.uploads) and not [k for k in kinds(env.sent) if k]
    ask_ok = ask is not None and ask["tag"] == ("pat", "send") and ask["timeout"] == 60.0 and "Sector" in ask["text"]
    ask["no"]()
    after_no = not env.uploads and not [k for k in kinds(env.sent) if k]
    boxes[0]("2")
    ask2 = dr.asks[-1]
    before2 = not [k for k in kinds(env.sent) if k]
    ask2["yes"]()
    ur = [m for m in env.sent if isinstance(m, dict) and m.get("t") == "uav_route"]
    uav_ok = (len(ur) == 1 and ur[0]["target"] == "2" and len(ur[0]["wps"]) == 10 and
              all(w["alt"] == 50.0 for w in ur[0]["wps"]) and not env.uploads)
    n_asks = len(dr.asks)
    boxes[0]("boat")
    dr.asks[-1]["yes"]()
    boat_ok = len(env.uploads) == 1 and len(dr.asks) == n_asks + 1
    env.reset()
    ap.gw_status = {"uav": {"enabled": True, "next": 1, "slots": []}}
    ctl = _pat_ap(env, "vs", R=400.0)
    ctl.set_alt(80)
    _yes(env)
    ctl.choose("next")
    ur2 = [m for m in env.sent if isinstance(m, dict) and m.get("t") == "uav_route"]
    alt_ok = len(ur2) == 1 and all(w["alt"] == 80.0 for w in ur2[0]["wps"])
    env.reset()
    ctl = _pat_ap(env, "vs", R=400.0)
    _yes(env)
    env.rig.gateway_like(False)
    ctl.choose("next")
    nogw = not env.asked and not env.sent and not env.uploads
    env.rig.gateway_like(True)
    report("PS4 nothing is sent without the YES: SEND opens the target box (nothing sent), the choice asks the "
           "vertical YES / NO on screen 1 (60 s, tag pat); NO = nothing; a UAV YES = uav_route with an altitude per "
           "waypoint (typed altitude kept); a boat YES = the normal upload; a UAV target without a gateway is refused",
           target_box and before and ask_ok and after_no and before2 and uav_ok and boat_ok and alt_ok and nogw,
           "box %s; before %s; ask %s; no %s; before2 %s; uav %s; boat %s; alt %s; nogw %s" % (
               target_box, before, ask_ok, after_no, before2, uav_ok, boat_ok, alt_ok, nogw))
    env.reset()


def ps5(env):
    ap = env.ap
    i18n.set_lang("en", save=False)
    env.position()
    ap.mob_search_ok = True
    m = mob.new_marker("mob_t6", 1, {"lat": LAT + 0.002, "lon": LON, "r_m": 3.0, "uncertain": False}, time.time(), "a")
    ap.sar.add(m)
    offers = mob.offers(m, None, None, ap.mob_search_ok)
    env.sent.clear()
    ap.mob_do("search", m)
    ctl = ap.sar_ctl
    datum_ok = ctl.datum == (m["lat"], m["lon"]) and ctl.datum_src == "mob"
    preview = ap.sar_preview is not None and ap.sar_preview["wps"][0] == ctl.datum
    quiet = not env.uploads and not [k for k in kinds(env.sent) if k not in ("save", "sar_mark")] and not env.asked
    panel = ap.sar_panel is not None
    if ap.sar_panel is not None:
        ap.sar_panel.close()
    cleared = ap.sar_preview is None and ap.sar_panel is None
    report("PS5 the MOB offer «ΜΟΤΙΒΟ ΕΡΕΥΝΑΣ»: the MOB is the datum, the plan is drawn on the map as a preview, the "
           "panel opens, nothing is sent; closing the panel clears the preview",
           "search" in offers and datum_ok and preview and quiet and panel and cleared,
           "offers %s; datum %s; preview %s; quiet %s; panel %s; cleared %s" % (
               offers, datum_ok, preview, quiet, panel, cleared))
    env.reset()


def ps6(env):
    import re
    ap = env.ap
    i18n.set_lang("en", save=False)
    ctl = ap.sar_ctl
    sarpanel.open_panel(ap, (LAT, LON), "map")
    pn = ap.sar_panel
    env.rig.pump(0.2)
    ok_open = pn is not None and ctl.plan is not None and ctl.kind == "ss"
    pn.kind_var.set("ps")
    pn.on_kind()
    kind_ok = ctl.kind == "ps" and "W" in pn.fields and "Lg" in pn.fields and "R" not in pn.fields
    for k, v in (("S", "100"), ("W", "500"), ("Lg", "1000")):
        pn.fields[k].set(v)
        pn.on_field(k)
    shown = "length" in pn.sum.cget("text") and ctl.plan is not None and ctl.plan["n"] == 10
    pn.kind_var.set("ss")
    pn.on_kind()
    for k, v in (("S", "50"), ("R", "1500")):
        pn.fields[k].set(v)
        pn.on_field(k)
    txt = pn.sum.cget("text")
    big_note = ("50" in txt and str(ctl.plan["n"]) in txt and str(pn.cut_btn.cget("state")) == "normal")
    pn.fields["S"].set("abc")
    pn.on_field("S")
    bad_input = ctl.plan is None and ctl.why == "input"
    pn.close()
    cleared = ap.sar_preview is None and ap.sar_panel is None
    ks = set(k for k in re.findall(r"T\(\"(pat\.[a-z_A-Z0-9]+)\"", open(os.path.join(HERE, "sarpanel.py"),
                                                                       encoding="utf-8").read()) if not k.endswith("_"))
    ks |= {"pat.title", "pat.menu_here", "pat.draw_note", "pat.why_datum", "pat.why_input", "pat.why_size",
           "pat.why_kind", "pat.summary", "pat.boat_over", "pat.cut_note", "pat.ask_boat", "pat.ask_uav"}
    ks |= {"pat.n_" + k for k in sp.KINDS} | {"pat.k_" + k for k in sp.KINDS}
    ks |= {"pat.src_" + k for k in ("map", "mob", "boat", "typed")}
    missing = []
    for lg in ("el", "en"):
        t = open(os.path.join(HERE, "lang", lg + ".txt"), encoding="utf-8").read()
        for k in sorted(ks):
            if not re.search(r"^" + re.escape(k) + r" =", t, re.M):
                missing.append((lg, k))
    report("PS6 the panel: opens with the datum, switches the pattern and its fields, takes typed values, shows the "
           "plan numbers, the >50 note and the CUT AT 50 button, refuses bad input, closes and clears the preview; "
           "every text in Greek and English",
           ok_open and kind_ok and shown and big_note and bad_input and cleared and not missing,
           "open %s; kind %s; shown %s; note %s; bad %s; cleared %s; missing %s" % (
               ok_open, kind_ok, shown, big_note, bad_input, cleared, missing[:4]))
    env.reset()


def ps7(env):
    """PF2-r: MED-1 STOP final (also cut at 50), MED-2 the UAV waypoint limit before the YES, LOW-1 non-finite input,
    LOW-4 radius vs leg length + short legs, the design text; PF3 with mob_search_ok True (the production default)."""
    ap = env.ap
    i18n.set_lang("en", save=False)
    env.position()
    # MED-1: the program's final action is STOP, also for a route cut at 50, and the YES text says it
    big = sp.plan("ss", (LAT, LON), {"S": 50.0, "legs": 61, "C": 0.0, "kn": 5.0})
    cut = sp.cut_at_limit(big)
    prog_cut = sp.to_boat_program(cut)
    ctl = _pat_ap(env, "ps", S=100.0, W=500.0, Lg=1000.0, C=45.0)
    _yes(env)
    env.asked.clear()
    ctl.choose("boat")
    final_ok = (prog_cut["final"] == sar_geo.PROG_END_STOP == 0 and env.uploads and env.uploads[-1][0] == 0 and
                ap.prog["final"] == 0 and bool(env.asked) and "STOPS" in env.asked[-1])
    el_text = open(os.path.join(HERE, "lang", "el.txt"), encoding="utf-8").read()
    says_stop = "σταματά" in [l for l in el_text.splitlines() if l.startswith("pat.ask_boat")][0]
    # MED-2: 30 waypoints go to the boat but a UAV is told the limit BEFORE the YES
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "gateway"))
    from uav import mission as uav_mission
    ctl2 = _pat_ap(env, "ss", S=100.0, legs=29, C=0.0)
    n_wps = ctl2.plan["n"]
    env.asked.clear()
    ap.gw_status = {"uav": {"enabled": True, "next": 1, "slots": []}}
    sent_before = len(env.sent)
    asked = ctl2.choose(3)
    line = ctl2.lines()
    uav_ok = (sp.UAV_MAX_WPS == uav_mission.MAX_WPS and n_wps == 30 and asked is False and not env.asked and
              "20" in line and "30" in line and not [m for m in env.sent[sent_before:] if m.get("t") == "uav_route"])
    small = _pat_ap(env, "ss", S=100.0, legs=10, C=0.0)
    small_ok = sp.uav_check(small.plan) is None
    # LOW-1: non-finite input is rejected, never raises
    bad = []
    for kind, p in (("ss", {"S": 100.0, "R": float("inf")}), ("ss", {"S": 100.0, "legs": float("inf")}),
                    ("vs", {"S": 100.0, "R": float("nan")}), ("ps", {"S": 100.0, "W": float("inf"), "Lg": 500.0}),
                    ("cs", {"S": 100.0, "W": 500.0, "Lg": float("inf")}), ("ss", {"S": float("inf"), "R": 500.0}),
                    ("ss", {"S": 100.0, "R": 10 ** 400})):
        try:
            r = sp.plan(kind, (LAT, LON), dict(p, C=0.0, kn=5.0))
            bad.append("why" in r)
        except Exception as e:
            bad.append(repr(e))
    finite_ok = all(b is True for b in bad)
    # LOW-4: the radius never exceeds the shortest leg; legs under the AURIX's 10 m are said before the YES
    tiny = sp.plan("ps", (LAT, LON), {"S": 8.0, "W": 40.0, "Lg": 100.0, "C": 0.0, "kn": 5.0})
    chk = sp.boat_check(tiny)
    mid = sp.plan("ps", (LAT, LON), {"S": 30.0, "W": 120.0, "Lg": 300.0, "C": 0.0, "kn": 5.0})
    pm = sp.to_boat_program(mid)
    short_leg = min(l["m"] for l in mid["legs"])
    radius_ok = all(w["radius"] <= short_leg for w in pm["wps"]) and not program.check(pm) and sp.boat_check(mid) is None
    ctl3 = _pat_ap(env, "ps", S=8.0, W=40.0, Lg=100.0, C=0.0)
    env.asked.clear()
    n_up = len(env.uploads)
    asked3 = ctl3.choose("boat")
    leg_ok = chk and chk["why"] == "short_leg" and asked3 is False and len(env.uploads) == n_up and not env.asked
    # the design text: STOP, and the creeping line's real direction
    dsg = open(os.path.join(os.path.dirname(HERE), "docs", "sar_design.md"), encoding="utf-8").read()
    design_ok = "final_action PROG_END_STOP" in dsg and "PROG_END_HOLD (search" not in dsg and "legs along C - 90" in dsg
    # a creeping line really runs its legs along C - 90 and advances along C
    cl = sp.plan("cs", (LAT, LON), {"S": 100.0, "C": 45.0, "W": 500.0, "Lg": 1000.0, "kn": 5.0})
    brg = cl["legs"][0]["brg"]
    cs_ok = min(abs(_angle_diff(brg, 45.0 - 90.0)), abs(_angle_diff(brg, 45.0 + 90.0))) < 0.5
    # PF3 with the production default mob_search_ok True: all three offers, in order, the search one opens the panel
    env.reset()
    ap.mob_search_ok = True
    env.position()
    ap.gw_status = {"uav": {"enabled": True, "next": 2, "slots": []}}
    dr = FakeDriver()
    dr.w = env.rig.root
    ap.driver = dr
    ap.mob_press("button")
    env.rig.pump(0.8, lambda: dr.asks)
    tags = []
    for _ in range(3):
        if not dr.asks:
            break
        a = dr.asks.pop(0)
        tags.append(a["tag"])
        env.sent.clear()
        if a["tag"] == ("mob", "search"):
            a["yes"]()
            break
        a["no"]()
        env.rig.pump(2.8, lambda: dr.asks)
    panel = ap.sar_panel is not None and ap.sar_ctl.datum_src == "mob"
    quiet = not env.uploads and not [k for k in kinds(env.sent) if k not in ("save", "sar_mark")]
    if ap.sar_panel is not None:
        ap.sar_panel.close()
    ap.driver = None
    report("PS7 PF2-r: the pattern route ends with STOP (also cut at 50; the YES text says so), a UAV pattern over "
           "20 waypoints is refused before the YES with the count and the limit (= gateway MAX_WPS), inf / NaN / huge "
           "input is a clear 'no plan', the radius never exceeds the shortest leg and legs under 10 m are refused "
           "before the YES, the design text matches; PF3 with search ON offers return, UAV, search in order",
           final_ok and says_stop and uav_ok and small_ok and finite_ok and radius_ok and leg_ok and design_ok and
           cs_ok and tags == [("mob", "return"), ("mob", "uav"), ("mob", "search")] and panel and quiet,
           "final %s stop-text %s uav %s/%s finite %s radius %s leg %s design %s cs %s offers %s panel %s quiet %s" % (
               final_ok, says_stop, uav_ok, small_ok, bad, radius_ok, leg_ok, design_ok, cs_ok, tags, panel, quiet))
    env.reset()


# --------------------------------------------------------------------------------------------- PF4: mark a point ----

CAL = {"height_m": 3.0, "tilt_down_deg": 5.0, "hfov_deg": 80.0, "vfov_deg": 45.0, "yaw_deg": 0.0}
UCAM = {"tilt_down_deg": 30.0, "hfov_deg": 120.0, "vfov_deg": 90.0}
RES = (640, 360)


def _hudar():
    import hudar
    return hudar


def _project_boat(cal, roll, pitch, heading, boat, true_pt):
    """The forward model (hudar.Cam, the AR overlay's own): a known point -> its place in the picture."""
    hudar = _hudar()
    e, n = hudar.enu(boat[0], boat[1], true_pt[0], true_pt[1])
    fwd, right = hudar.to_boat(e, n, heading)
    return hudar.Cam(cal, roll, pitch).project(fwd, right)


def _project_uav(alt, roll, pitch, heading, pos, true_pt, mount=UCAM):
    hudar = _hudar()
    cal = dict(mount, height_m=alt, yaw_deg=0.0)
    e, n = hudar.enu(pos[0], pos[1], true_pt[0], true_pt[1])
    fwd, right = hudar.to_boat(e, n, heading)
    return hudar.Cam(cal, roll, pitch).project(fwd, right)


def _telem(boat, heading, roll, pitch, unc=2.0):
    return {"est_lat": boat[0], "est_lon": boat[1], "pos_source": 1, "unc_m": unc, "heading": heading,
            "att_roll": roll, "att_pitch": pitch}


def mk1():
    """The boat's calibrated camera: a known point (forward model of the AR overlay) is marked back at its place within
    centimetres without noise; the uncertainty circle grows with the range; a heeling, pitching boat is handled."""
    import sar_mark
    boat, heading = (LAT, LON), 40.0
    worst, rs = 0.0, []
    for roll, pitch in ((0.0, 0.0), (4.0, 0.0), (-6.0, 2.0), (3.0, -3.0)):
        rs_this = []
        for d, b in ((12, -25), (20, 0), (30, 15), (45, 30), (60, -10)):
            tp = sar_geo.dest(boat, (heading + b) % 360.0, d)
            uv = _project_boat(CAL, roll, pitch, heading, boat, tp)
            if uv is None or not (0 <= uv[0] <= 1 and 0 <= uv[1] <= 1):
                continue
            r = sar_mark.locate_boat(CAL, _telem(boat, heading, roll, pitch), uv[0], uv[1], RES, 0.2)
            if r.get("why"):
                rs_this.append(("refused", d, r["why"]))
                continue
            err = sar_geo.distance_bearing((r["lat"], r["lon"]), tp)[0]
            worst = max(worst, err)
            if roll == 0.0 and pitch == 0.0:
                rs.append((d, r["r_m"]))
    grows = all(b[1] > a[1] for a, b in zip(rs, rs[1:])) and len(rs) >= 3
    report("MK1 the boat's calibrated camera: a known point marked back within %.2f m (no noise), also with roll / "
           "pitch; the uncertainty circle grows with the range %s" % (worst, [(d, round(r)) for d, r in rs]),
           worst < 0.5 and grows, "worst error %.3f m; r_m by range %s" % (worst, [(d, round(r, 1)) for d, r in rs]))


def mk2():
    """A UAV's fixed camera: altitude + attitude + heading + mount; the forward model round trip, farther than the boat."""
    import sar_mark
    pos, heading, alt = (LAT, LON), 200.0, 50.0
    worst, rs = 0.0, []
    for roll, pitch in ((0.0, 0.0), (5.0, -3.0), (-4.0, 4.0)):
        for d, b in ((40, 0), (100, -20), (250, 25), (500, 5), (900, -10)):
            tp = sar_geo.dest(pos, (heading + b) % 360.0, d)
            uv = _project_uav(alt, roll, pitch, heading, pos, tp)
            if uv is None or not (0 <= uv[0] <= 1 and 0 <= uv[1] <= 1):
                continue
            drone = {"lat": pos[0], "lon": pos[1], "alt": alt, "hdg": heading, "roll": roll, "pitch": pitch,
                     "att_age": 0.3, "alt_age": 0.4, "camera": dict(UCAM)}
            r = sar_mark.locate_uav(drone, uv[0], uv[1], RES)
            if r.get("why"):
                continue
            err = sar_geo.distance_bearing((r["lat"], r["lon"]), tp)[0]
            worst = max(worst, err)
            if roll == 0.0 and pitch == 0.0:
                rs.append((d, r["r_m"]))
    grows = len(rs) >= 3 and all(b[1] > a[1] for a, b in zip(rs, rs[1:]))
    report("MK2 a UAV's fixed camera (50 m, 30 deg down, 120 x 90): a known point marked back within %.2f m, to 900 m "
           "(the boat's camera gives up at about 60 m: sigma_d/d > 0.5); the circle grows with the range" % worst,
           worst < 1.0 and grows, "worst %.3f m; r_m by range %s" % (worst, [(d, round(r)) for d, r in rs]))


def mk3():
    """Refusals: at / above the horizon, beyond the range, too uncertain, no calibration, old / missing attitude, no
    position, a UAV too low, NaN / out of picture - each with its reason, nothing made."""
    import sar_mark
    boat = (LAT, LON)
    t = _telem(boat, 40.0, 0.0, 0.0)
    hz = sar_mark.locate_boat(CAL, t, 0.5, 0.30, RES, 0.2)             # above the horizon line
    far = sar_mark.locate_boat(CAL, t, 0.5, 0.41, RES, 0.2)            # a few pixels below it: 2 degrees too uncertain
    nocal = sar_mark.locate_boat(None, t, 0.5, 0.8, RES, 0.2)
    noatt = sar_mark.locate_boat(CAL, dict(t, att_roll=None), 0.5, 0.8, RES, 0.2)
    nopos = sar_mark.locate_boat(CAL, dict(t, pos_source=0), 0.5, 0.8, RES, 0.2)
    old = sar_mark.locate_boat(CAL, t, 0.5, 0.8, RES, 9.0)
    nan = sar_mark.locate_boat(CAL, t, float("nan"), 0.8, RES, 0.2)
    off = sar_mark.locate_boat(CAL, t, 1.2, 0.8, RES, 0.2)
    ok = sar_mark.locate_boat(CAL, t, 0.5, 0.8, RES, 0.2)
    dr = {"lat": LAT, "lon": LON, "alt": 50.0, "hdg": 10.0, "roll": 0.0, "pitch": 0.0, "att_age": 0.3, "alt_age": 0.3}
    u_hz = sar_mark.locate_uav(dr, 0.5, 0.05, RES)                     # 30 deg down, 45 deg half FOV: the top is 15 deg
    u_ok = sar_mark.locate_uav(dr, 0.5, 0.6, RES)
    u_low = sar_mark.locate_uav(dict(dr, alt=2.0), 0.5, 0.6, RES)
    u_old = sar_mark.locate_uav(dict(dr, att_age=3.0), 0.5, 0.6, RES)
    u_none = sar_mark.locate_uav(dict(dr, roll=None), 0.5, 0.6, RES)
    u_pos = sar_mark.locate_uav(dict(dr, lat=None), 0.5, 0.6, RES)
    u_range = sar_mark.locate_uav(dict(dr, alt=120.0), 0.5, 0.505, RES, dict(sar_mark.DEFAULTS, max_range_uav=200.0))
    u_cam = sar_mark.locate_uav(dict(dr, camera={"tilt_down_deg": 30, "hfov_deg": 0, "vfov_deg": 90}), 0.5, 0.6, RES)
    report("MK3 refusals with their reason (nothing made): horizon, too uncertain, no calibration, no / old attitude, "
           "no position, old position, NaN / outside the picture; UAV: horizon, too low, old attitude, no position, "
           "beyond the range, bad camera mount",
           hz.get("why") == "horizon" and far.get("why") == "uncertain" and nocal.get("why") == "nocal" and
           noatt.get("why") == "noatt" and nopos.get("why") == "nopos" and old.get("why") == "old" and
           nan.get("why") == "bad" and off.get("why") == "bad" and not ok.get("why") and u_hz.get("why") == "horizon"
           and not u_ok.get("why") and u_low.get("why") == "low" and u_old.get("why") == "noatt" and
           u_none.get("why") == "noatt" and u_pos.get("why") == "nopos" and u_range.get("why") == "range" and
           u_cam.get("why") == "nocal",
           "%s" % [x.get("why") for x in (hz, far, nocal, noatt, nopos, old, nan, off, ok, u_hz, u_ok, u_low, u_old,
                                          u_none, u_pos, u_range, u_cam)])


def mk4():
    """Accuracy with known points and the stated noise (attitude 1 / 2 deg, calibration 0.5 deg, heading 2 deg,
    1.5 pixels, position): how far off the estimate is and how often the true place is inside the circle. These numbers
    are in the manual (chapter 23 / 18)."""
    import random
    import sar_mark
    rnd = random.Random(20261109)
    cfg = sar_mark.DEFAULTS
    rows = []
    # the bands are the ESTIMATED range (what the operator sees); the true point is drawn from the whole range
    trials = {}
    for kind, span in (("boat", (6.0, 80.0)), ("uav", (30.0, 1100.0))):
        for _ in range(6000):
            half = (CAL if kind == "boat" else UCAM)["hfov_deg"] / 2.0     # PF4-r MED-4: the WHOLE field of view
            d, b = rnd.uniform(*span), rnd.uniform(-half, half)
            heading = rnd.uniform(0, 360)
            roll, pitch = rnd.uniform(-5, 5), rnd.uniform(-3, 3)
            base = (LAT + rnd.uniform(-0.01, 0.01), LON + rnd.uniform(-0.01, 0.01))
            tp = sar_geo.dest(base, (heading + b) % 360.0, d)
            s_att = cfg["sigma_att_boat" if kind == "boat" else "sigma_att_uav"]
            n_roll, n_pitch = roll + rnd.gauss(0, s_att), pitch + rnd.gauss(0, s_att)
            n_hdg = heading + rnd.gauss(0, cfg["sigma_hdg"])
            unc = 2.0 if kind == "boat" else cfg["uav_pos_unc_m"]
            dn, de = rnd.gauss(0, unc / 1.414), rnd.gauss(0, unc / 1.414)
            n_pos = (base[0] + dn / 111320.0, base[1] + de / (111320.0 * math.cos(math.radians(base[0]))))
            if kind == "boat":
                uv = _project_boat(CAL, roll, pitch, heading, base, tp)
            else:
                uv = _project_uav(50.0, roll, pitch, heading, base, tp)
            if uv is None or not (0.02 <= uv[0] <= 0.98 and 0.02 <= uv[1] <= 0.98):
                continue
            u = uv[0] + rnd.gauss(0, cfg["sigma_pix"]) / RES[0]
            v = uv[1] + rnd.gauss(0, cfg["sigma_pix"]) / RES[1]
            cal_err = rnd.gauss(0, cfg["sigma_cal"])
            if kind == "boat":
                cal = dict(CAL, tilt_down_deg=CAL["tilt_down_deg"] + cal_err)
                r = sar_mark.locate_boat(cal, _telem(n_pos, n_hdg, n_roll, n_pitch, unc), u, v, RES, 0.2)
            else:
                drone = {"lat": n_pos[0], "lon": n_pos[1], "alt": 50.0, "hdg": n_hdg, "roll": n_roll,
                         "pitch": n_pitch, "att_age": 0.3, "alt_age": 0.3,
                         "camera": dict(UCAM, tilt_down_deg=UCAM["tilt_down_deg"] + cal_err)}
                r = sar_mark.locate_uav(drone, u, v, RES)
            if r.get("why"):
                continue
            err = sar_geo.distance_bearing((r["lat"], r["lon"]), tp)[0]
            trials.setdefault(kind, []).append((r["d_m"], err, r["r_m"]))
    rows = []
    for kind, bands in (("boat", ((10, 20), (20, 35), (35, 50))), ("uav", ((50, 150), (150, 300), (300, 900)))):
        for lo, hi in bands:
            sel = [t for t in trials.get(kind, []) if lo <= t[0] < hi]
            n = len(sel)
            if n >= 30:
                errs = sorted(t[1] for t in sel)
                rr = sorted(t[2] for t in sel)
                rows.append((kind, lo, hi, n, errs[n // 2], errs[int(n * 0.9)], rr[n // 2],
                             100.0 * sum(1 for t in sel if t[1] <= t[2]) / n))
    ok = bool(rows) and all(r[7] >= 92.0 for r in rows) and len(rows) == 6       # PF4-r MED-4: ~95 % in the MODEL
    for r in rows:
        print("    MK4 %s %4d-%4d m: n %3d  median error %6.1f m  90 %% %6.1f m  median circle %6.1f m  inside %5.1f %%" % r)
    report("MK4 accuracy with known points + the stated noise over the WHOLE field of view (12000 trials, PF4-r MED-4): "
           "the true place is inside the circle in at least 92 %% of the cases in every band by the ESTIMATED range "
           "(93-100 %% measured; circle_k 4.0 x the 1 sigma; this is the MODEL, not a real lens)",
           ok, "; ".join("%s %d-%d: err %.1f / circle %.1f / %.0f %%" % (r[0], r[1], r[2], r[4], r[6], r[7])
                         for r in rows))


def mk11():
    """PF4-r MED-3: the attitude's REAL age counts - the boat: the telemetry sample's age at the station + its own
    att_age_ms > 1 s = refused; the UAV: the gateway's att_age / alt_age + the record's age at the station (rx_age)
    > 1 s = refused (a missing / non-finite rx_age = old). LOW: a UAV mount's tilt_down_deg out of -30..90 = refused.
    LOW: the still image cap is 40 KB on the station too (= the gateway's)."""
    import sar_mark
    boat = (LAT, LON)
    tp = sar_geo.dest(boat, 40.0, 25.0)
    uv = _project_boat(CAL, 0.0, 0.0, 40.0, boat, tp)
    t = dict(_telem(boat, 40.0, 0.0, 0.0), att_age_ms=600)
    ok_fresh = not sar_mark.locate_boat(CAL, t, uv[0], uv[1], RES, 0.3).get("why")
    old_sum = sar_mark.locate_boat(CAL, t, uv[0], uv[1], RES, 0.5).get("why")          # 0.5 + 0.6 > 1 s
    old_sample = sar_mark.locate_boat(CAL, _telem(boat, 40.0, 0.0, 0.0), uv[0], uv[1], RES, 1.2).get("why")
    pos = (LAT + 0.002, LON + 0.002)
    tpu = sar_geo.dest(pos, 130.0, 180.0)
    uvu = _project_uav(60.0, 0.0, 0.0, 100.0, pos, tpu)
    dr = {"lat": pos[0], "lon": pos[1], "alt": 60.0, "hdg": 100.0, "roll": 0.0, "pitch": 0.0, "att_age": 0.4,
          "alt_age": 0.4, "camera": dict(UCAM)}
    u_ok = not sar_mark.locate_uav(dict(dr, rx_age=0.3), uvu[0], uvu[1], RES).get("why")
    u_old = sar_mark.locate_uav(dict(dr, rx_age=0.7), uvu[0], uvu[1], RES).get("why")
    u_inf = sar_mark.locate_uav(dict(dr, rx_age=float("inf")), uvu[0], uvu[1], RES).get("why")
    tilt = [sar_mark.locate_uav(dict(dr, camera=dict(UCAM, tilt_down_deg=x)), uvu[0], uvu[1], RES).get("why")
            for x in (-45.0, 120.0)]
    ok = (ok_fresh and old_sum == "noatt" and old_sample == "noatt" and u_ok and u_old in ("old", "noatt") and
          u_inf == "old" and tilt == ["nocal", "nocal"] and sar_mark.STILL_MAX_BYTES == 40 * 1024)
    report("MK11 PF4-r: the attitude's real age (sample age + att_age_ms for the boat, att_age + the age at the station "
           "for a UAV) over 1 s = refused; a UAV mount tilt out of -30..90 deg = refused; the still cap is 40 KB here "
           "too", ok, "boat fresh %s / sum %s / sample %s; uav %s / %s / inf %s; tilt %s; still %d" % (
               ok_fresh, old_sum, old_sample, u_ok, u_old, u_inf, tilt, sar_mark.STILL_MAX_BYTES))


def _setup_boat(env, **kw):
    ap = env.ap
    boat = (LAT, LON)
    env.position(LAT, LON)                              # a real telemetry sample (the screens render from it)
    t = dict(ap.t)
    t.update({"est_lat": boat[0], "est_lon": boat[1], "pos_source": 1, "unc_m": 2.0, "heading": 40.0,
              "att_roll": 0.0, "att_pitch": 0.0})
    t.update(kw)
    ap.t, ap.t_rx = t, time.time()
    ap.gw_status = {"vision": {"camera": dict(CAL)}}
    ap.mark_last = 0.0
    return boat


def _img(w=640, h=360):
    from PIL import Image
    return Image.new("RGB", (w, h), (30, 90, 160))


def mk5(env):
    """The station: a click on the video makes ONE marker (kind point, ESTIMATE, r_m, source camera, time) shared through
    the gateway; the event log + message say it; a refusal makes nothing and says why; nothing else is sent."""
    ap = env.ap
    i18n.set_lang("en", save=False)
    boat = _setup_boat(env)
    tp = sar_geo.dest(boat, 40.0 + 10.0, 30.0)
    uv = _project_boat(CAL, 0.0, 0.0, 40.0, boat, tp)
    env.sent.clear()
    m = ap.mark_from_video("FRONT-1", uv[0], uv[1], _img())
    add = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "sar_mark"]
    err = sar_geo.distance_bearing((m["lat"], m["lon"]), tp)[0] if m else 99.0
    again = ap.mark_from_video("FRONT-1", uv[0], uv[1], _img())          # a double click within 1 s = the same press
    others = [k for k in kinds(env.sent) if k != "sar_mark"] + [f for f in env.frames if f in FORBIDDEN]
    ap.mark_last = 0.0
    n_before = len(env.sent)
    hz = ap.mark_from_video("FRONT-1", 0.5, 0.30, _img())                # above the horizon
    rear = ap.mark_from_video("REAR-LEFT", 0.5, 0.8, _img())             # an uncalibrated camera
    sent_after = len(env.sent) - n_before
    ap.mark_last = 0.0
    ap.gw_status = {}
    nocal = ap.mark_from_video("FRONT-1", 0.5, 0.8, _img())
    msg = ap._evlog[-1] if ap._evlog else ""
    ap.mark_last = 0.0
    _setup_boat(env, att_roll=None)
    noatt = ap.mark_from_video("FRONT-1", uv[0], uv[1], _img())
    report("MK5 a click on the video = ONE marker (point, ESTIMATE, r_m, camera, time) to the gateway, nothing else sent; "
           "a double click is the same press; above the horizon / an uncalibrated camera / no calibration / no attitude "
           "= refused with the reason, no marker, nothing sent",
           m and err < 0.5 and m["kind"] == "point" and m["est"] is True and m["r_m"] > 0 and m["person"] is False and
           m["src"]["camera"] == "FRONT-1" and m["name"] == "POINT 1" and len(add) == 1 and add[0]["op"] == "add" and
           add[0]["m"]["id"] == m["id"] and again is None and not others and hz is None and rear is None and
           sent_after == 0 and nocal is None and noatt is None and len(ap.mark_points()) == 1 and
           "calibrat" in msg.lower(),
           "marker %s err %.2f m; sends %s; refusals %s; last event %r" % (
               m and (m["name"], m["r_m"]), err, kinds(add), [hz, rear, nocal, noatt], msg[:70]))
    env.reset()


def mk6(env):
    """The small still image: 160 x 120 around the click, kept on this PC, sent to the gateway only after the marker was
    acknowledged (once); a refused still never refuses the marker."""
    import base64
    ap = env.ap
    boat = _setup_boat(env)
    tp = sar_geo.dest(boat, 45.0, 25.0)
    uv = _project_boat(CAL, 0.0, 0.0, 40.0, boat, tp)
    env.sent.clear()
    m = ap.mark_from_video("FRONT-1", uv[0], uv[1], _img())
    held = m["id"] in ap.mark_stills and 0 < len(ap.mark_stills[m["id"]]) <= 40 * 1024
    from PIL import Image
    import io
    im = Image.open(io.BytesIO(ap.mark_stills[m["id"]]))
    size_ok = im.size == (160, 120)
    box_ok = sar_mark_box_ok()
    before = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "sar_still"]
    ap.on_sar_ack({"ok": True, "id": m["id"], "code": 0, "text": "added"})
    st = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "sar_still"]
    ap.on_sar_ack({"ok": True, "id": m["id"], "code": 0, "text": "added"})
    st2 = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "sar_still"]
    jpg_ok = st and base64.b64decode(st[0]["jpg_b64"])[:2] == b"\xff\xd8" and st[0]["id"] == m["id"]
    ap.on_sar_ack({"ok": False, "what": "still", "id": m["id"], "code": 108, "text": "bad still image"})
    kept = ap.sar.get(m["id"]) is not None and m["id"] not in ap.sar.refused
    on_disk = os.path.exists(os.path.join(ap.sar.folder, "stills", m["id"] + ".jpg"))
    report("MK6 the small still image: 160 x 120 around the click (inside the picture at the edges), kept on this PC, sent "
           "to the gateway ONCE and only after the marker was acknowledged; a refused still never refuses the marker",
           held and size_ok and box_ok and not before and len(st) == 1 and len(st2) == 1 and jpg_ok and kept and on_disk,
           "held %s size %s box %s before %d sent %d/%d jpg %s kept %s disk %s" % (
               held, im.size, box_ok, len(before), len(st), len(st2), bool(jpg_ok), kept, on_disk))
    env.reset()


def sar_mark_box_ok():
    import sar_mark
    a = sar_mark.still_box(0.0, 0.0, 640, 360)
    b = sar_mark.still_box(1.0, 1.0, 640, 360)
    c = sar_mark.still_box(0.5, 0.5, 640, 360)
    d = sar_mark.still_box(0.5, 0.5, 100, 80)
    return a == (0, 0, 160, 120) and b == (480, 240, 640, 360) and c == (240, 120, 400, 240) and d == (0, 0, 100, 80)


def mk7(env):
    """A UAV's video: the drone's altitude + attitude + heading + mount -> the marker; a UAV picture needs the gateway;
    no fresh attitude = refused; the UAV code never reads anything of the boat's attitude."""
    ap = env.ap
    i18n.set_lang("en", save=False)
    _setup_boat(env, att_roll=None, att_pitch=None)                     # the boat's attitude is NOT used for a UAV
    pos = (LAT + 0.002, LON + 0.002)
    tp = sar_geo.dest(pos, 130.0, 180.0)
    uv = _project_uav(60.0, 3.0, -2.0, 100.0, pos, tp)
    ap.uav_store.drones[2] = {"slot": 2, "lat": pos[0], "lon": pos[1], "alt": 60.0, "hdg": 100.0, "roll": 3.0,
                              "pitch": -2.0, "att_age": 0.4, "alt_age": 0.5, "camera": dict(UCAM), "in_slot": False}
    ap.uav_store.t[2] = time.time()                                     # received just now (as uav_store.update does)
    ap.mark_last = 0.0
    env.sent.clear()
    m = ap.mark_from_video("UAV-2", uv[0], uv[1], _img())
    err = sar_geo.distance_bearing((m["lat"], m["lon"]), tp)[0] if m else 99.0
    ap.mark_last = 0.0
    ap.uav_store.drones[2]["att_age"] = 5.0
    old = ap.mark_from_video("UAV-2", uv[0], uv[1], _img())
    ap.uav_store.drones[2]["att_age"] = 0.4
    ap.mark_last = 0.0
    env.rig.gateway_like(False)
    nogw = ap.mark_from_video("UAV-2", uv[0], uv[1], _img())
    env.rig.gateway_like(True)
    ap.link.send_msg = lambda mm: env.sent.append(mm)
    report("MK7 a click on a UAV video: marked from the drone's altitude + attitude + heading + camera mount (error %.2f "
           "m), source UAV-2; the boat's own attitude is not needed; an old drone attitude and a missing gateway are "
           "refused" % err,
           m and err < 1.0 and m["src"]["camera"] == "UAV-2" and m["est"] and old is None and nogw is None,
           "marker %s; old %s; no gateway %s" % (m and m["r_m"], old, nogw))
    env.reset()


def mk8(env):
    """The actions of a marker: nothing moves before the YES; route there (a plain point = the normal route, a person =
    the stand-off route ending 30 m before it with STOP), UAV there (uav_route + the launch preparation), search pattern
    here (the datum, nothing sent), person / rename / delete shared."""
    ap = env.ap
    i18n.set_lang("en", save=False)
    boat = _setup_boat(env)
    ap.t["est_lat"], ap.t["est_lon"] = boat
    dr = FakeDriver()
    dr.w = env.rig.root
    ap.driver = dr
    ap.gw_status = {"uav": {"enabled": True, "next": 1, "slots": []}}
    ap.mob_have = {"lat": LAT, "lon": LON, "unc_m": 2.0, "sog": 0.0, "t": time.time(), "fresh": True}
    tp = sar_geo.dest(boat, 60.0, 400.0)
    m = {"id": "pt_t_1", "kind": "point", "n": 1, "name": "POINT 1", "lat": tp[0], "lon": tp[1], "r_m": 12.0,
         "est": True, "person": False, "t_utc": "2026-11-09T12:00:00Z", "src": {"camera": "FRONT-1"}, "closed": None}
    ap.sar.add(m)
    env.sent.clear()
    env.uploads.clear()
    ap.mark_ask("route", m)
    quiet = not env.uploads and len(dr.asks) == 1
    q1 = dr.asks[0]
    dr.asks.clear()
    q1["no"]()
    no_ok = not env.uploads
    ap.mark_ask("route", m)
    dr.asks.pop(0)["yes"]()
    plain = len(env.uploads) == 1 and len(env.uploads[0][1]) == 2
    abs_end = (env.uploads[0][1][-1]["lat"], env.uploads[0][1][-1]["lon"]) if env.uploads else None
    plain_end_ok = abs_end and sar_geo.distance_bearing(abs_end, tp)[0] < 0.5
    env.uploads.clear()
    ap.mark_person(m["id"], True)
    person_msg = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "person"]
    ap.mark_ask("route", ap.sar.get(m["id"]))
    q2 = dr.asks.pop(0)
    txt2 = q2["text"]
    q2["yes"]()
    stand = len(env.uploads) == 1 and env.uploads[0][0] == sar_geo.PROG_END_STOP
    d_end = sar_geo.distance_bearing((env.uploads[0][1][-1]["lat"], env.uploads[0][1][-1]["lon"]), tp)[0] \
        if env.uploads else 0
    env.sent.clear()
    ap.mark_ask("uav", ap.sar.get(m["id"]))
    quiet_uav = not [x for x in env.sent if isinstance(x, dict) and x.get("t") in ("uav_route", "uav_launch_prepare")]
    dr.asks.pop(0)["yes"]()
    ur = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "uav_route"]
    up = [x for x in env.sent if isinstance(x, dict) and x.get("t") == "uav_launch_prepare"]
    env.sent.clear()
    ap.mark_ask("delete", ap.sar.get(m["id"]))
    still_there = ap.sar.get(m["id"]) is not None
    dr.asks.pop(0)["yes"]()
    dele = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "delete"]
    ap.sar.add(dict(m, id="pt_t_2", n=2))
    ap.mark_rename("pt_t_2", "Life ring")
    ren = [x for x in env.sent if isinstance(x, dict) and x.get("op") == "rename"]
    sarpanel_ok = True
    env.uploads.clear()
    env.sent.clear()
    sarpanel.open_panel(ap, (m["lat"], m["lon"]), "mark")
    sarpanel_ok = ap.sar_ctl.datum_src == "mark" and not env.uploads and not env.sent
    if ap.sar_panel is not None:
        ap.sar_panel.close()
    # a point deleted on another station goes here too (the gateway's list no longer holds it); a MOB never does
    ap.sar.mark_sent("pt_t_2")
    ap.sar.add({"id": "mob_keep", "kind": "mob", "n": 1, "lat": LAT, "lon": LON, "t_utc": "2026-11-09T12:00:00Z",
                "closed": None})
    ap.sar.mark_sent("mob_keep")
    # PF4-r LOW: an EMPTY / restarted gateway list deletes nothing - the sent point is kept and handed in again
    ap.on_sar_markers({"rev": 9, "list": []})
    kept = ap.sar.get("pt_t_2") is not None and any(x["id"] == "pt_t_2" for x in ap.sar.unsent())
    ap.sar.mark_sent("pt_t_2")
    ap.on_sar_markers({"rev": 10, "list": [], "deleted": ["pt_t_2", "mob_keep"]})
    merged = kept and ap.sar.get("pt_t_2") is None and ap.sar.get("mob_keep") is not None
    report("MK8 the actions: NOTHING moves before the YES (NO = nothing); route there = a 2-point route to a plain point; "
           "a person = the stand-off route ending %.0f m before it with STOP; UAV there = uav_route + the launch "
           "preparation; search pattern = the datum only; person / rename / delete are shared; a point deleted elsewhere "
           "(the gateway's \"deleted\" list) disappears here, a MOB never does, an empty list deletes nothing" % d_end,
           quiet and no_ok and plain and plain_end_ok and person_msg and "DANGER FROM THE PROPELLER" in txt2 and stand
           and abs(d_end - 30.0) < 0.6 and quiet_uav and len(ur) == 1 and len(up) == 1 and ur[0]["target"] == "next" and
           still_there and len(dele) == 1 and ren and ren[0]["m"]["name"] == "Life ring" and sarpanel_ok and merged,
           "quiet %s no %s plain %s/%s person %s stand %s %.1f uav %s %d %d delete %s/%s rename %s panel %s merged %s" % (
               quiet, no_ok, plain, plain_end_ok, bool(person_msg), stand, d_end, quiet_uav, len(ur), len(up),
               still_there, len(dele), bool(ren), sarpanel_ok, merged))
    ap.driver = None
    env.reset()


def mk9(env):
    """The screens: screen 2's camera view click and the key K (not in a text field, not with Ctrl, only the chosen key),
    screen 1's click on the free video, the map drawing (ESTIMATE text, circle) and a click on the marker finds it."""
    ap = env.ap
    i18n.set_lang("en", save=False)
    boat = _setup_boat(env)
    tp = sar_geo.dest(boat, 40.0, 30.0)
    uv = _project_boat(CAL, 0.0, 0.0, 40.0, boat, tp)
    # --- screen 2's camera view
    ap.camview = "FRONT-1"
    ap.mark_rect = (100.0, 50.0, 640.0, 360.0)
    ap.camview_frame = lambda: _img()
    x, y = 100.0 + uv[0] * 640.0, 50.0 + uv[1] * 360.0
    outside = ap.camview_mark(10.0, 10.0)
    m = ap.camview_mark(x, y)
    ap.mark_last = 0.0
    # --- the key K
    ap.mark_cursor = (time.time(), None, x, y)
    ev = types.SimpleNamespace(keysym="k", state=0, widget=None)
    ap.mark_key_press(ev)
    n_key = len(ap.mark_points())
    ap.mark_last = 0.0
    ap.mark_key_press(types.SimpleNamespace(keysym="k", state=0x4, widget=None))          # Ctrl+K
    ap.mark_key_press(types.SimpleNamespace(keysym="m", state=0, widget=None))            # another key
    entry = types.SimpleNamespace(winfo_class=lambda: "Entry")
    ap.mark_key_press(types.SimpleNamespace(keysym="k", state=0, widget=entry))           # typing into a field
    n_same = len(ap.mark_points())
    ap.mark_last = 0.0
    ap.mark_cursor = None
    ap.mark_key_press(ev)                                                                  # no video under the mouse
    hint = len(ap.mark_points()) == n_same
    ap.camview = None
    # --- screen 1's click on the free video (the driver's own mark_at, a stand-in window)
    import driver as drv
    calls = []
    stub = types.SimpleNamespace(pic=(10.0, 20.0, 800.0, 450.0), front_use="FRONT-2",
                                 app=types.SimpleNamespace(mark_from_video=lambda cam, u, v, img: calls.append((cam, u, v))
                                                           or "ok"),
                                 frame_of=lambda cam: None)
    r1 = drv.DriverScreen.mark_at(stub, 10.0 + 400.0, 20.0 + 225.0)
    r2 = drv.DriverScreen.mark_at(stub, 5.0, 5.0)
    click_ok = r1 == "ok" and calls and calls[0][0] == "FRONT-2" and abs(calls[0][1] - 0.5) < 1e-9 and \
        abs(calls[0][2] - 0.5) < 1e-9 and r2 is None
    # --- the map
    class Cv:
        def __init__(self):
            self.items = []

        def create_oval(self, *a, **k):
            self.items.append(("oval", k.get("tags")))

        def create_polygon(self, *a, **k):
            self.items.append(("poly", k.get("tags")))

        def create_text(self, *a, **k):
            self.items.append(("text", k.get("text")))
    cv = Cv()
    ap.mapwin.draw_points(cv, lambda la, lo: (400 + (lo - LON) * 1e5, 300 - (la - LAT) * 1e5), {}, None)
    texts = [i[1] for i in cv.items if i[0] == "text"]
    drawn = any(i[0] == "oval" for i in cv.items) and any(i[0] == "poly" for i in cv.items) and \
        any("ESTIMATE" in t for t in texts) and any("POINT 1" in t for t in texts)
    ap.mapwin.view = (LAT, LON, 800, 600)
    P = lambda la, lo: (400 + (lo - LON) * 1e5, 300 - (la - LAT) * 1e5)   # noqa: E731
    ap.mapwin.P = lambda: (P, None)
    mk_ = ap.mark_points()[0]
    px, py = P(mk_["lat"], mk_["lon"])
    menus = []
    ap.mark_menu = lambda x, y, mm: menus.append(mm["id"])
    hit = ap.sar_map_hit(px + 3, py - 2)
    miss = ap.sar_map_hit(px + 80, py + 80)
    del ap.mapwin.P
    del ap.mark_menu
    report("MK9 screen 2: a click on the camera view marks (outside the picture nothing), the key K marks under the mouse "
           "and is ignored with Ctrl / in a text field / for another key / without a video under the mouse; screen 1's "
           "click on the free video asks for the point under it; the map draws diamond + circle + ESTIMATE text and a "
           "click on a marker finds it",
           outside is None and m and n_key == 2 and n_same == n_key and hint and click_ok and drawn and hit and
           menus == [mk_["id"]] and not miss,
           "outside %s marker %s keys %d/%d click %s drawn %s hit %s miss %s" % (
               outside, bool(m), n_key, n_same, click_ok, drawn, hit, miss))
    env.reset()


def mk10():
    """Texts: every mark.* key in Greek AND English, the Greek ESTIMATE wording, the same placeholders; the design / the
    manuals state the numbers; no app / hal / allow.py change."""
    import re
    root = os.path.dirname(HERE)

    def keys(lang):
        out = {}
        with open(os.path.join(HERE, "lang", lang + ".txt"), encoding="utf-8") as f:
            for line in f:
                if line.startswith("mark."):
                    k, _, v = line.rstrip("\r\n").partition(" = ")
                    out[k] = v
        return out
    en, el = keys("en"), keys("el")
    same = set(en) == set(el) and len(en) >= 35
    ph = all(sorted(re.findall(r"\{(\w+)\}", en[k])) == sorted(re.findall(r"\{(\w+)\}", el[k])) for k in en if k in el)
    est = el.get("mark.estimate") == "ΕΚΤΙΜΗΣΗ – όχι για ναυσιπλοΐα" and en.get("mark.estimate") == \
        "ESTIMATE - not for navigation"
    man = ""
    for f in ("MANUAL.txt", "MANUAL_EL.txt"):
        with open(os.path.join(root, f), encoding="utf-8") as fh:
            man += fh.read()
    manual_ok = "MARK A POINT" in man and "ΣΗΜΑΝΣΗ ΣΗΜΕΙΟΥ" in man
    report("MK10 the texts: every mark.* key in Greek and English with the same placeholders, the ESTIMATE wording, both "
           "manuals have the chapter", same and ph and est and manual_ok,
           "keys %d/%d placeholders %s estimate %s manuals %s" % (len(en), len(el), ph, est, manual_ok))


def main():
    import tempfile
    tmp = tempfile.mkdtemp(prefix="trygons_sar_")
    mb8()
    ps1()
    mk1()
    mk2()
    mk3()
    mk4()
    mk11()
    mk10()
    rig = ts.Rig()
    env = None
    try:
        env = Env(rig, tmp)
        keep_t = (rig.ap.t, rig.ap.t_rx)
        for fn in (mb1, mb2, mb3, mb4, mb5, mb6, mb7, mb9, mb10, ps2, ps3, ps4, ps5, ps6, ps7, mk5, mk6, mk7, mk8, mk9):
            try:
                fn(env)
            except Exception as e:
                import traceback
                traceback.print_exc()
                report(fn.__name__, False, "crashed: %r" % e)
                try:
                    env.reset()
                except Exception:
                    pass
        report("MB0 the Tk callbacks raised nothing", not rig.errors, str(rig.errors[:3]))
    finally:
        if env:
            env.restore()
            rig.ap.t, rig.ap.t_rx = keep_t
        rig.close()
    bad = results.count(False)
    print("SAR STATION TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
