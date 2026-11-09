"""ADS-B on the ground station (CLAUDE.md X K4, step PF5): DISPLAY + station-local notices + a PROPOSAL.

  AV1  the aircraft on the map: symbol turned with the track, label (callsign, altitude, speed), track, grey when stale,
       CAUTION / WARNING colour for the ones that warn
  AV2  no receiver / no data = «ADS-B: χωρίς δέκτη» / «χωρίς δεδομένα» while a UAV is in the air (also when ADS-B is not
       configured); none while it works or no UAV flies; drawn on the map, once in the event log - never a clear sky
  AV3  warnings: CAUTION then WARNING = station-local lamps (never a boat alarm, nothing sent), the voice once per rise
       «Αεροσκάφος κοντά στο UAV n», cleared when the aircraft is gone
  AV4  the proposal: asked once, only to the captain / that UAV's pilot, one vertical YES / NO; NO or no answer = nothing
       sent; YES HOLD = uav_hold; YES DESCEND = the normal uav_route (30 m over its position); a proposal that no
       longer stands sends nothing
  AV5  texts (Greek + English, every voice line), the callouts exist for six slots
  AV6  the station's ADS-B code sends only what the answer to a proposal sends

Run:  python test_adsbui.py      (exit code 0 = all passed)
"""
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_sar as TS  # noqa: E402  (its environment + Env + fakes)
import adsbmap  # noqa: E402
import alerts  # noqa: E402
import i18n  # noqa: E402
import sound  # noqa: E402
import theme  # noqa: E402
from i18n import T  # noqa: E402

results = TS.results
report = TS.report
LAT, LON = 60.0, 25.0


def target(icao="4B1815", cs="HEL123", dx=0.0, **kw):
    d = {"icao": icao, "cs": cs, "lat": LAT, "lon": LON + dx, "alt_ft": 400.0, "gs_kn": 75.0, "trk": 270.0,
         "vs_fpm": 0, "gnd": False, "age": 1.0, "stale": False, "track": [[LAT, LON + dx + 0.01], [LAT, LON + dx]]}
    d.update(kw)
    return d


def state(st="RX", targets=(), warn=(), props=(), flying=(3,)):
    return {"enabled": True, "state": st, "targets": list(targets), "warn": list(warn), "proposals": list(props),
            "flying": list(flying)}


def warn(level, slot=3, icao="4B1815", cs="HEL123"):
    return {"icao": icao, "cs": cs, "slot": slot, "level": level, "t": 20.0, "d_h": 400, "d_v": 60, "kind": "hold"}


def prop(pid="adsb1", slot=3, kind="hold", level=1):
    return {"id": pid, "slot": slot, "icao": "4B1815", "cs": "HEL123", "level": level, "kind": kind,
            "alt_safe_m": 30.0, "d_h": 400, "d_v": 60, "t": 20.0}


def fly(env, slot=3, armed=True):
    env.ap.uav_store.drones.clear()
    env.ap.uav_store.drones[slot] = {"slot": slot, "lat": LAT, "lon": LON, "alt": 80.0, "armed": armed, "landed": not armed,
                                    "in_slot": False}


def captain(env, me=7, pilots=None, crew="captain"):
    env.ap.gw_status = {"you": me, "stations": [{"id": me, "crew": crew, "role": "HELM", "name": "x", "user": "x", "local": False, "rtt_ms": 10, "tmode": "full"}],
                        "uav": {"enabled": True, "pilots": pilots or {}}}


def av1(env):
    import tkinter as tk
    ap = env.ap
    cv = tk.Canvas(env.rig.root, width=400, height=300)
    P = lambda la, lo: ((lo - LON) * 20000.0 + 200, 150 - (la - LAT) * 20000.0)
    th = theme.map_theme_colours() if hasattr(theme, "map_theme_colours") else {"adsb": "#a04000", "route": "#7a00a0"}
    ap.adsb_state = state(targets=[target(), target("A1B2C3", "OLD1", dx=0.003, stale=True, gs_kn=None),
                                   target("D4E5F6", None, dx=0.002, alt_ft=None)], warn=[warn(2)])
    ap.uav_store.drones.clear()
    adsbmap.draw(cv, P, ap, th, ("Arial", 8), 400, 300)
    syms = cv.find_withtag("adsb_sym")
    labels = [cv.itemcget(i, "text") for i in cv.find_withtag("adsb_label")]
    cols = [cv.itemcget(i, "fill") for i in syms]
    tracks = cv.find_withtag("adsb_track")
    ok = (len(syms) == 3 and len(tracks) == 3 and "HEL123 · 400 ft · 75 kn" in labels and
          any(l.startswith("OLD1 · 400 ft") and "kn" not in l for l in labels) and
          any(l.startswith("D4E5F6") and T("adsb.alt_unknown") in l for l in labels) and
          cols[0] == theme.WARN and cols[1] == "#808080" and adsbmap.label({"icao": "X"}).startswith("X"))
    cv.destroy()
    report("AV1 aircraft on the map: symbol, label (callsign, altitude, speed), track, stale grey, warning colour", ok,
           "symbols %d, tracks %d, labels %s, colours %s" % (len(syms), len(tracks), labels, cols))


def av2(env):
    import tkinter as tk
    ap = env.ap
    i18n.set_lang("el", save=False)
    ap.adsb_state = None
    fly(env)
    out = {}
    for name, st in (("no_rx", state("NO_RX")), ("off", {"enabled": False, "state": "OFF"}), ("old", None),
                     ("no_data", state("NO_DATA")), ("rx", state("RX"))):
        ap.adsb_sync(st)
        out[name] = ap.adsb_notice
    fly(env, armed=False)
    ap.adsb_sync(state("NO_RX"))
    out["on_ground"] = ap.adsb_notice
    fly(env)
    ap.adsb_sync(state("NO_RX"))
    ev = [e for e in ap.events.get(0, "end") if "ADS-B" in e]
    ap.adsb_sync(state("NO_RX"))
    ev2 = [e for e in ap.events.get(0, "end") if "ADS-B" in e]
    cv = tk.Canvas(env.rig.root, width=400, height=300)
    adsbmap.draw(cv, lambda a, b: (0, 0), ap, {"adsb": "#a04000", "route": "#000"}, ("Arial", 8), 400, 300)
    shown = [cv.itemcget(i, "text") for i in cv.find_withtag("adsb_notice")]
    cv.destroy()
    ok = (out["no_rx"] == "ADS-B: χωρίς δέκτη" and out["off"] == "ADS-B: χωρίς δέκτη" and
          out["old"] == "ADS-B: χωρίς δέκτη" and out["no_data"] == "ADS-B: χωρίς δεδομένα" and out["rx"] is None and
          out["on_ground"] is None and shown == ["ADS-B: χωρίς δέκτη"] and len(ev2) == len(ev) and ev)
    report("AV2 no receiver / no data is SAID while a UAV is in the air (also when ADS-B is off); never a clear sky", ok,
           "%s, on the map %s, event lines %d -> %d" % (out, shown, len(ev), len(ev2)))


def av3(env):
    ap = env.ap
    i18n.set_lang("el", save=False)
    fly(env)
    captain(env)
    env.sent.clear()
    env.frames.clear()
    ap.sound.said.clear()
    ap.adsb_sync(state(warn=[warn(1)]))
    code = "adsb_4B1815_3"
    l1 = ap.lamps.local.get(code)
    ap.adsb_sync(state(warn=[warn(1)]))
    ap.adsb_sync(state(warn=[warn(2)]))
    l2 = ap.lamps.local.get(code)
    ap.adsb_sync(state(warn=[warn(2)]))
    said = list(ap.sound.said)
    ap.adsb_sync(state(warn=[]))
    gone = code not in ap.lamps.local
    ap.sound.said.clear()
    ap.adsb_sync(state(warn=[warn(2, slot=2, icao="AAAAAA")]))
    ap.adsb_sync(state(warn=[]))
    ap.adsb_sync(state(warn=[warn(2, slot=2, icao="AAAAAA")]))
    again = ap.sound.said.count("adsb.w2")
    nothing_sent = not [m for m in env.sent if m.get("t") not in ("driver",)] and not env.frames
    ok = bool(l1 and l1[1] == alerts.CAUTION and "HEL123" in l1[0] and "UAV 3" in l1[0] and l2 and
              l2[1] == alerts.WARNING and said == ["adsb.c3", "adsb.w3"] and gone and again == 2 and nothing_sent)
    report("AV3 CAUTION then WARNING: station-local lamps, the voice once per rise, cleared when gone, nothing sent", ok,
           "caution %s, warning %s, said %s, gone %s, again %d, sent %s" % (l1, l2, said, gone, again,
                                                                          [m.get("t") for m in env.sent]))
    for k in list(ap.lamps.local):
        ap.lamps.set_local(k, None)


def av4(env):
    ap = env.ap
    i18n.set_lang("en", save=False)
    dr = TS.FakeDriver()
    dr.w = env.rig.root
    ap.driver = dr
    fly(env)
    captain(env)
    env.sent.clear()
    p = prop("adsb1", 3, "hold")
    ap.adsb_sync(state(props=[p], warn=[warn(1)]))
    env.rig.pump(1.0, lambda: dr.asks)
    ask = dr.asks[0] if dr.asks else None
    before = [m for m in env.sent if m.get("t") in ("uav_hold", "uav_route")]
    ap.adsb_sync(state(props=[p], warn=[warn(1)]))                 # the same proposal again: not asked twice
    env.rig.pump(1.0)
    twice = len(dr.asks)
    ask["no"]()                                                     # NO = nothing
    no_sent = [m for m in env.sent if m.get("t") in ("uav_hold", "uav_route")]
    # a new proposal, YES = uav_hold for THAT UAV
    p2 = prop("adsb2", 3, "hold")
    ap.adsb_sync(state(props=[p2], warn=[warn(1)]))
    env.rig.pump(1.0, lambda: len(dr.asks) > 1)
    dr.asks[1]["yes"]()
    holds = [m for m in env.sent if m.get("t") == "uav_hold"]
    # DESCEND = the normal uav_route at the UAV's own position, 30 m
    env.sent.clear()
    p3 = prop("adsb3", 3, "descend")
    ap.adsb_sync(state(props=[p3], warn=[warn(1)]))
    env.rig.pump(1.0, lambda: len(dr.asks) > 2)
    dr.asks[2]["yes"]()
    routes = [m for m in env.sent if m.get("t") == "uav_route"]
    # a proposal that no longer stands: YES sends nothing
    env.sent.clear()
    p4 = prop("adsb4", 3, "hold")
    ap.adsb_sync(state(props=[p4], warn=[warn(1)]))
    env.rig.pump(1.0, lambda: len(dr.asks) > 3)
    ap.adsb_sync(state(props=[], warn=[]))
    dr.asks[3]["yes"]()
    stale_sent = [m for m in env.sent if m.get("t") in ("uav_hold", "uav_route")]
    # who is asked: a copilot is not; that UAV's pilot is; a single operator is
    n0 = len(dr.asks)
    captain(env, crew="copilot")
    ap.adsb_sync(state(props=[prop("adsb5", 3)], warn=[warn(1)]))
    env.rig.pump(1.0)
    cop = len(dr.asks) - n0
    captain(env, crew="copilot", pilots={"3": {"station": 7}})
    ap.adsb_sync(state(props=[prop("adsb6", 3)], warn=[warn(1)]))
    env.rig.pump(1.0, lambda: len(dr.asks) > n0)
    pil = len(dr.asks) - n0
    captain(env, crew="copilot", pilots={"2": {"station": 7}})
    ap.adsb_sync(state(props=[prop("adsb7", 3)], warn=[warn(1)]))
    env.rig.pump(1.0)
    other_uav = len(dr.asks) - n0 - pil
    ok = (ask and ask["tag"] == ("adsb", "adsb1") and ask["head"] == T("adsb.head") and "HEL123" in ask["text"] and
          "UAV 3" in ask["text"] and "nothing happens without your YES" in ask["text"] and not before and twice == 1 and
          not no_sent and holds == [{"t": "uav_hold", "uav": 3}] and len(routes) == 1 and routes[0]["target"] == 3 and
          routes[0]["wps"][0]["alt"] == 30.0 and abs(routes[0]["wps"][0]["lat"] - LAT) < 1e-6 and not stale_sent and
          cop == 0 and pil == 1 and other_uav == 0)
    report("AV4 the proposal: asked once to the captain / that UAV's pilot only; NO or a gone proposal sends nothing; "
           "YES = uav_hold / the normal uav_route", ok,
           "ask %s, before %s, twice %d, holds %s, routes %s, stale %s, copilot %d, pilot %d, other UAV's pilot %d" %
           (ask and ask["tag"], before, twice, holds, routes, stale_sent, cop, pil, other_uav))
    ap.driver = None


def av5(env):
    ok_lang = True
    miss = []
    for lang in ("el", "en"):
        d = i18n.load(lang)
        for k in ("adsb.no_rx", "adsb.no_data", "adsb.head", "adsb.caution", "adsb.warning", "adsb.offer_hold",
                  "adsb.offer_descend", "adsb.sent_hold", "adsb.sent_descend", "adsb.gone", "adsb.alt_unknown") + \
                tuple("say.adsb.uav%d" % i for i in range(1, 7)):
            if not d.get(k):
                ok_lang = False
                miss.append((lang, k))
    el = i18n.load("el")
    call = all(("adsb.c%d" % i) in sound.CALLOUTS and ("adsb.w%d" % i) in sound.CALLOUTS and
               sound.CALLOUTS["adsb.w%d" % i][0] == sound.WARNING and sound.CALLOUTS["adsb.c%d" % i][0] == sound.CAUTION
               for i in range(1, 7))
    ok = ok_lang and call and el["say.adsb.uav3"] == "Αεροσκάφος κοντά στο UAV 3" and \
        el["adsb.no_rx"] == "ADS-B: χωρίς δέκτη"
    report("AV5 texts in Greek + English, the six voice lines, the callouts (CAUTION / WARNING)", ok, "missing %s" % miss)


def av6(env):
    src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    i = src.index("# --- ADS-B (step PF5")
    j = src.index("    def map_menu", i)
    block = src[i:j]
    sends = re.findall(r'send_msg\(\{"t": "(\w+)"', block) + re.findall(r"self\.(uav_route)\(", block)
    cmds = re.findall(r"self\.cmd\(|_send_cmd\(|link\.send\(|prog_upload|auto_start|set_mode", block)
    mapsrc = open(os.path.join(HERE, "adsbmap.py"), encoding="utf-8").read()
    ok = sorted(set(sends)) == ["uav_hold", "uav_route"] and not cmds and "send" not in re.sub(r'""".*?"""', "", mapsrc,
                                                                                               flags=re.S)
    report("AV6 the station's ADS-B code only sends uav_hold / uav_route (the answer to a proposal), nothing to the boat",
           ok, "sends %s, other commands %s" % (sorted(set(sends)), cmds))


def main():
    import tempfile
    tmp = tempfile.mkdtemp(prefix="trygons_adsb_")
    rig = TS.ts.Rig()
    import traceback as _tb
    rig.root.report_callback_exception = lambda *a: (rig.errors.append(repr(a[1])), _tb.print_exception(*a))
    env = None
    try:
        env = TS.Env(rig, tmp)
        keep_t = (rig.ap.t, rig.ap.t_rx)
        for fn in (av1, av2, av3, av4, av5, av6):
            try:
                env.reset()
                fn(env)
            except Exception as e:
                import traceback
                traceback.print_exc()
                report(fn.__name__.upper(), False, "crashed: %r" % e)
        report("AV0 the Tk callbacks raised nothing", not rig.errors, str(rig.errors[:3]))
    finally:
        if env:
            env.restore()
            rig.ap.t, rig.ap.t_rx = keep_t
            rig.ap.adsb_state = None
        rig.close()
    bad = results.count(False)
    print("ADS-B STATION TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
