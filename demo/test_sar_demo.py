"""Tests of the SAR demo guide (step PF6): SD1 the progress model, SD2 the guide sends nothing, SD3 the ADS-B cue,
SD4 the simulated person in UAV 1's picture, SD5 the launcher. Run: python demo/test_sar_demo.py"""
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "gateway"))
import sar_demo  # noqa: E402

FAILS = []


def report(name, ok, detail=""):
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILS.append(name)


def sd1():
    m = sar_demo.SarDemo()
    ok = m.step == 0
    m.feed("telemetry", {"mode_name": "AUTO"})
    ok &= m.step == 1                                                        # AUTO seen -> step 2 (MOB)
    m.feed("sar_markers", {"list": [{"id": "a", "kind": "mob"}]})
    ok &= m.step == 2                                                        # MOB seen -> return offer: manual
    m.feed("gw_status", {"uav": {"flying": {"1": {"phase": "climb"}}}})
    ok &= m.step == 2                                                        # a UAV flying does NOT skip a manual step
    m.done_pressed()
    ok &= sar_demo.STEPS[m.step][0] == "pattern" or sar_demo.STEPS[m.step][0] == "uav"
    # the UAV is flying already -> step uav auto-ticks, the pattern step waits for the presenter
    ok &= sar_demo.STEPS[m.step][0] == "pattern"
    m.done_pressed()
    ok &= sar_demo.STEPS[m.step][0] == "mark"
    m.feed("sar_markers", {"list": [{"id": "a", "kind": "mob"}, {"id": "b", "kind": "point"}]})
    ok &= sar_demo.STEPS[m.step][0] == "heli"
    m.feed("gw_status", {"adsb": {"warn": [{"level": 2}]}})
    ok &= sar_demo.STEPS[m.step][0] == "heli"                                # not sent by the guide yet: no tick
    m.heli_sent = True
    m.feed("gw_status", {"adsb": {"warn": [{"level": 2}]}})
    ok &= sar_demo.STEPS[m.step][0] == "hold"
    m.done_pressed()
    ok &= sar_demo.STEPS[m.step][0] == "end"
    m.done_pressed()
    ok &= sar_demo.STEPS[m.step][0] == "end"
    for s in sar_demo.STEPS:                                                 # every prompt in both languages
        ok &= bool(s[2]) and bool(s[3])
    report("SD1 guide: steps tick from what the demo shows; manual steps wait for the presenter", ok)


def sd2():
    src = open(os.path.join(HERE, "sar_demo.py"), encoding="utf-8").read()
    code = "\n".join(l for l in src.split("\n") if not l.strip().startswith("#"))
    code = re.sub(r'""".*?"""', "", code, flags=re.S)
    bad = [w for w in ("send_msg", ".send(", "sendall", "_send_obj", "T_SET_MODE", "answer(", "driver_answer",
                       "subprocess", "os.system") if w in code]
    ok = not bad
    report("SD2 guide sends nothing (receive-only link, no command, no process)", ok, "found: %s" % bad if bad else "")


def sd3():
    import simadsb
    tmp = tempfile.mkdtemp(prefix="trygons_sd_")
    cue = os.path.join(tmp, "cue")
    old = os.environ.get("TRYGONS_SIM_ADSB_CUE")
    try:
        os.environ["TRYGONS_SIM_ADSB_CUE"] = cue
        s = simadsb.AdsbSim(0, (37.82, 23.6))
        s.tick(1.0)
        ok = s.heli is None                                                  # no cue: no helicopter
        open(cue, "w").write("go")
        s.tick(1.0)
        ok &= s.heli is not None and not os.path.exists(cue)                 # cue: it starts, the cue is used up
        del os.environ["TRYGONS_SIM_ADSB_CUE"]
        s2 = simadsb.AdsbSim(0, (37.82, 23.6))
        s2.tick(1.0)
        ok &= s2.heli is not None                                            # no variable: as before
    finally:
        if old is None:
            os.environ.pop("TRYGONS_SIM_ADSB_CUE", None)
        else:
            os.environ["TRYGONS_SIM_ADSB_CUE"] = old
        try:
            os.rmdir(tmp)
        except OSError:
            pass
    report("SD3 ADS-B: the helicopter starts on the cue only (demo --sar); plain demo unchanged", ok)


def sd4():
    import simcam
    w, h = 320, 240
    px, py = int(w * simcam.PERSON_AT[0]), int(h * simcam.PERSON_AT[1])
    old = os.environ.pop("TRYGONS_SIM_PERSON", None)
    try:
        a = simcam.aerial_frame(w, h, 3)
        os.environ["TRYGONS_SIM_PERSON"] = "1"
        b = simcam.aerial_frame(w, h, 3)
        c = simcam.aerial_frame(w, h, 3, slot=2)
    finally:
        os.environ.pop("TRYGONS_SIM_PERSON", None)
        if old is not None:
            os.environ["TRYGONS_SIM_PERSON"] = old
    ok = a[py * w + px] != 255 and b[py * w + px] == 255 and c[py * w + px] != 255 and len(a) == len(b) == w * h
    report("SD4 the simulated person is in UAV 1's picture only with TRYGONS_SIM_PERSON", ok)


def sd5():
    bat = open(os.path.join(REPO, "DEMO_SAR.bat"), encoding="utf-8").read()
    d = open(os.path.join(HERE, "demo.py"), encoding="utf-8").read()
    ok = bat.count("start --sar --splash") == 2 and '"--sar"' in d and "sar_demo.py" in d
    report("SD5 DEMO_SAR.bat starts demo.py start --sar; demo.py launches the guide", ok)


if __name__ == "__main__":
    for f in (sd1, sd2, sd3, sd4, sd5):
        f()
    print("SAR DEMO TESTS %s" % ("FAILED: " + ", ".join(FAILS) if FAILS else "OK"))
    sys.exit(1 if FAILS else 0)
