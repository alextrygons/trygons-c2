"""HX7-fix4: a self-test of the storm CLASS logic in scenarios.evaluate on SYNTHETIC episodes (no simulator): every
class passes its good case and FAILS each mutation of its evidence (a class that cannot fail proves nothing).
    python tests/harbour/selftest_classes.py          exit 0 = every case as expected"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "tools"))
import scenarios as sc  # noqa: E402

U13 = "VESSEL NOT GIVING WAY - STOP - TAKE OVER"


class Ep:
    def standby_check(self):
        return True


def samp(t, x, sog, state, why=0, txt="", alarm=0, ny=None, tx=None, gaps=None, mode="AUTO"):
    g = gaps or [("fast", 60.0)]
    return (t, x, 0.0, 90.0, sog, state, 0.0, mode, 60.0, 0.0, True, g[0][0], 0.0, why, txt, 0, 0.0,
            [(o, gg, True) for o, gg in g], alarm, ny or [], False, (tx if tx is not None else x, 0.0, 90.0, sog),
            [(o, gg) for o, gg in g])


def s13(mut=None):
    ep = Ep()
    ep.events = []
    S_ = [samp(i * 0.2, 1.0, 0.0, "HOLD", 3, U13, 0x1000000, ["67"], tx=(5.0 if mut == "moved" and i > 50 else 0.0),
               gaps=[("fast", 60 - i * 0.2)]) for i in range(400)]
    ep.samples = S_
    m = "kayak" if mut == "wrongobj" else "fast"
    spd = 0.0 if mut == "moored" else 3.5
    ep.stops = [{"t": 0.0, "reason": "U13 a vessel not giving way (67)", "true_pos": True, "blockers": [],
                 "not_yielding": [{"id": "67", "track": True, "match": "fast", "match_d": 10.6, "gap": 62,
                                   "obj_speed": spd}]},
                {"t": 19.1, "reason": "U7 an object inside 8 m", "true_pos": True, "blockers": [],
                 "not_yielding": [{"id": "67", "track": True, "match": m, "match_d": 9.0 if mut == "far" else 0.4,
                                   "gap": 4, "obj_speed": spd}]}]
    if mut == "notfirst":
        ep.stops.insert(0, {"t": 0.0, "reason": "U5 x", "true_pos": True, "blockers": [], "not_yielding": []})
    spec = {"name": "S13", "expect": {"start_may_be_refused": 1}, "objects": [{"id": "fast", "kind": "vessel"}],
            "route": [(0, 0, 5), (130, 0, 5)]}
    return spec, ep, {"upload": 0, "start": 0, "min_gap": {"fast": (2.0, True, "HOLD")}}


def s21(mut=None):
    ep = Ep()
    ep.events = [(20.0, "fault radar_off" if mut == "other" else "fault planner_crash")]
    S_ = []
    for i in range(200):
        t = i * 0.2
        x = min(t, 24.0)
        st, why, mode = ("RUN", 0, "AUTO") if t < 21 else ("HOLD", 2, "AUTO")
        if mut == "why" and t >= 21:
            why = 3
        if mut == "early":
            x = min(t, 15.0)
            st, why = ("HOLD", 2) if t > 15 else ("RUN", 0)
        if mut == "slowpre" and 19.0 <= t < 19.9:
            st = "HOLD"
        if mut == "redrive" and t > 30:
            st = "RUN"
        if mut == "late" and 21 <= t < 26:
            st, why = "SLOW", 0
        if mut == "off_auto" and t > 30:
            st = "OFF"
        S_.append(samp(t, x, 0.0 if st == "HOLD" else 3.0, st, why, gaps=[("buoy", 50.0)], mode=mode))
    ep.samples = S_
    if mut == "resume":
        ep.events.append((22.0, "RESUME after a host-load HOLD (why 2)"))
    ep.stops = []
    if mut == "pstop":
        ep.stops = [{"t": 20.5, "reason": "U5 x", "blockers": []}]
    if mut == "pstop_before":
        ep.stops = [{"t": 19.5, "reason": "U5 x", "blockers": []}]
    spec = {"name": "S21", "expect": {"start_may_be_refused": 1}, "objects": [{"id": "buoy", "kind": "buoy"}],
            "faults": [{"what": "radar_off" if mut == "other" else "planner_crash"}], "route": [(0, 0, 5), (130, 0, 5)]}
    return spec, ep, {"upload": 0, "start": 0, "min_gap": {"buoy": (50.0, False, "HOLD")}}


def s14(mut=None):
    ep = Ep()
    ep.events = []
    S_ = [samp(i * 0.2, min(i * 0.2, 21.0), 0.0 if i * 0.2 > 22 else 2.0, "HOLD" if i * 0.2 > 22 else "RUN",
               3 if i * 0.2 > 22 else 0, gaps=[("slow", 30.0)]) for i in range(300)]
    ep.samples = S_
    cells = [{"x": 40.0, "y": 1.0, "layer": "lidar", "match": "slow", "match_d": 29.0 if mut == "phantom" else 1.0}]
    if mut == "nocells":
        cells = []
    ep.stops = [{"t": 21.0, "reason": "U5 no free path inside the corridor", "true_pos": True, "static_blocker": True,
                 "blockers": [{"id": "1", "track": True, "match": "slow", "match_d": 0.1, "gap": 30.0},
                              {"id": "static-map", "track": False}],
                 "static": {"chart": 0, "lidar": 1, "cells": cells}, "not_yielding": []}]
    spec = {"name": "S14", "expect": {"start_may_be_refused": 1}, "objects": [{"id": "slow", "kind": "vessel"}],
            "route": [(0, 0, 5), (130, 0, 5)]}
    return spec, ep, {"upload": 0, "start": 0, "min_gap": {"slow": (30.0, True, "HOLD")}}


CASES = [(s13, None, "start_stop", True), (s13, "moved", "start_stop", False), (s13, "wrongobj", "start_stop", False),
         (s13, "far", "start_stop", False), (s13, "notfirst", "start_stop", False), (s13, "moored", "start_stop", False),
         (s21, None, "fault_stop", True), (s21, "why", "fault_stop", False), (s21, "pstop", "fault_stop", False),
         (s21, "pstop_before", "fault_stop", False), (s21, "slowpre", "fault_stop", False),
         (s21, "redrive", "fault_stop", False), (s21, "early", "normal", False), (s21, "other", "normal", False),
         (s21, "late", "fault_stop", False), (s21, "off_auto", "fault_stop", False), (s21, "resume", "fault_stop", False),
         (s14, None, "early_stop", True), (s14, "phantom", "early_stop", False), (s14, "nocells", "early_stop", False)]


def main():
    bad = 0
    for f, mut, want_cls, want_ok in CASES:
        spec, ep, out = f(mut)
        r = sc.evaluate(spec, ep, out)
        ok = all(c[1] for c in r)
        good = out.get("class") == want_cls and ok == want_ok
        bad += 0 if good else 1
        print("[%s] %s %s: class %s (want %s), passes %s (want %s)%s" % (
            "PASS" if good else "FAIL", f.__name__, mut or "good", out.get("class"), want_cls, ok, want_ok,
            "" if ok else " - failed: " + ", ".join(c[0][:24] for c in r if not c[1])), flush=True)
    print("harbour class self-test: %d / %d as expected" % (len(CASES) - bad, len(CASES)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
