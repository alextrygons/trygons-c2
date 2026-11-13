"""Tests of the harbour demo guide (step HX7): HD1 the progress model, HD2 the guide sends nothing, HD3 the launcher.
Run: python demo/test_harbour_demo.py"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import harbour_demo  # noqa: E402

FAILS = []


def report(name, ok, detail=""):
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILS.append(name)


def gws(state=None, sensors="OK", active=True):
    return {"harbour": {"sensors": {"radar": {"state": sensors}, "lidar": {"state": sensors}},
                        "planner": {"active": active, "stat": {"harb_state_name": state}}}}


def hd1():
    m = harbour_demo.HarbourDemo()
    ok = m.step == 0
    m.feed("gw_status", gws("OFF", sensors="STARTING", active=False))
    ok &= m.step == 0                                                    # sensors not OK: no tick
    m.feed("gw_status", gws("OFF"))
    ok &= harbour_demo.STEPS[m.step][0] == "start" or harbour_demo.STEPS[m.step][0] == "route"
    m.feed("gw_status", gws("OFF"))
    ok &= harbour_demo.STEPS[m.step][0] == "start"                       # route seen (planner active), START waits
    m.feed("telemetry", {"mode_name": "AUTO"})
    m.feed("gw_status", gws("RUN"))
    ok &= harbour_demo.STEPS[m.step][0] == "traffic"                     # RUN seen -> run ticked
    m.feed("gw_status", gws("SLOW"))
    ok &= harbour_demo.STEPS[m.step][0] == "decide"                      # SLOW after RUN = traffic seen; the decision is manual
    m.feed("telemetry", {"mode_name": "STANDBY"})
    ok &= harbour_demo.STEPS[m.step][0] == "decide"                      # STANDBY does not skip the manual step
    m.done_pressed()
    ok &= harbour_demo.STEPS[m.step][0] == "end"
    for s in harbour_demo.STEPS:
        ok &= bool(s[2]) and bool(s[3])
    report("HD1 guide: steps tick from what the demo shows; the decision step waits for the presenter", ok)


def hd2():
    src = open(os.path.join(HERE, "harbour_demo.py"), encoding="utf-8").read()
    code = "\n".join(l for l in src.split("\n") if not l.strip().startswith("#"))
    code = re.sub(r'""".*?"""', "", code, flags=re.S)
    bad = [w for w in ("send_msg", ".send(", "sendall", "_send_obj", "T_SET_MODE", "T_HARBOUR", "answer(",
                       "driver_answer", "subprocess", "os.system", "cmd(") if w in code]
    report("HD2 guide sends nothing (receive-only link, no command, no process)", not bad,
           "found: %s" % bad if bad else "")


def hd3():
    bat = open(os.path.join(REPO, "DEMO_HARBOUR.bat"), encoding="utf-8").read()
    d = open(os.path.join(HERE, "demo.py"), encoding="utf-8").read()
    ok = bat.count("start --harbour --splash") == 2 and '"--harbour"' in d and "harbour_demo.py" in d
    report("HD3 DEMO_HARBOUR.bat starts demo.py start --harbour; demo.py launches the guide", ok)


if __name__ == "__main__":
    for f in (hd1, hd2, hd3):
        f()
    print("HARBOUR DEMO TESTS %s" % ("FAILED: %s" % FAILS if FAILS else "OK"))
    sys.exit(1 if FAILS else 0)
