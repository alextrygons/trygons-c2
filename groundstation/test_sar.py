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

Run:  python test_sar.py      (exit code 0 = all passed)
"""
import math
import os
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
        ap.link.send = lambda t, *a, **k: (self.frames.append(t), 1)[1]
        self.frames.clear()
        self.asked.clear()
        ynbox.ask = lambda parent, title, text, **k: (self.asked.append(text), False)[1]    # NO unless a test says
        ap.driver = None
        self.uploads, self.starts = [], []
        ap.prog_upload = lambda final=None: self.uploads.append((final, [dict(w) for w in ap.prog["wps"]]))
        ap.auto_start = lambda: self.starts.append(time.time())
        ap.single_op = False
        ap.gw_status = {}
        ap.mob_alarmed = set()
        ap.mob_queue = []
        ap.mob_auto_until = 0.0
        ap.mob_sar_rev = None
        ap.mob_sync_t = 0.0
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
    ap.mob_route_stored()
    ap.mob_route_stored()                                               # a second ACK does not start twice
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
           all(w["kn"] <= 5.0 for w in wps) and armed and started == 1 and no_route and nopos and east_of_marker and
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
        ap.mob_key_press(ev("m"))
        ap.mob_key_press(ev("M"))
        ap.mob_key_press(ev("m", ent))                      # typing into a field
        ap.mob_key_press(ev("m", None, 0x4))                # Ctrl+M
        ap.mob_key_press(ev("k"))
        n_default = len(calls)
        settings.put("mob_key", "k")
        ap.mob_key_press(ev("k"))
        ap.mob_key_press(ev("m"))
        n_custom = len(calls) - n_default
        settings.put("mob_key", None)
    finally:
        ap.mob_press = orig
        ent.destroy()
    src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    report("MB6 the MOB key (default M, setting mob_key): works from the station window, not in a text field, not with "
           "Ctrl, only the chosen key; it is a keyboard key of the station, never a joystick button",
           n_default == 2 and n_custom == 1 and calls == ["key"] * 3 and
           "root.bind_all(\"<KeyPress>\", self.mob_key_press" in src and "mob" not in open(
               os.path.join(HERE, "shorejoy.py"), encoding="utf-8").read().lower().replace("mobile", ""),
           "default presses %d, custom %d, calls %s" % (n_default, n_custom, calls))
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


def main():
    import tempfile
    tmp = tempfile.mkdtemp(prefix="trygons_sar_")
    mb8()
    rig = ts.Rig()
    env = None
    try:
        env = Env(rig, tmp)
        for fn in (mb1, mb2, mb3, mb4, mb5, mb6, mb7, mb9):
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
        rig.close()
    bad = results.count(False)
    print("SAR STATION TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
