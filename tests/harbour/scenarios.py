"""HX7: the SCENARIO LIBRARY (docs/harbour_design.md 9.1, H1-H20). Every scenario is a seeded, deterministic spec for
rig.Episode (the real autopilot_sim + the real gateway modules, end to end) plus its PASS CRITERIA.

World frame: the own boat starts at (0, 0) heading EAST (the simulator's start), the harbour route runs along +x, the
corridor is |y| <= corr_m (10 m). y > 0 = to PORT, y < 0 = to STARBOARD. The world's t0 is the moment the captain's
START is sent (+- 3 s): a moving object is built by mover(meet=(x, y), at=T_s, ...) = it is at `meet` at T seconds
after t0 (the own boat is about x = 2.5 T - 14 m at 5 kn after the ramp-up).

COMMON criteria (every scenario, checks() below):
  C1 never touches: hull-to-hull gap (own boat = circle r 3.5 m, objects = their sizes) >= 0.5 m all the time
  C2 keeps its distance: while RUN / SLOW the gap to every object that is NOT excused is >= 15 m (moving) / 8 m
     (static) - 0.5 m (design I88); a HOLD must begin in time (the gap at the standstill >= 2 m)
  C3 stays inside the corridor while RUN / SLOW: |xte| <= corr + 1 m
  C4 never above the speed cap: SOG <= 5 + 0.5 kn (leg speed 5), <= 2.5 kn 10 s after a sensor was lost
  C5 STANDBY always wins: a STANDBY request at the end of the run gives MODE STANDBY within 4 s
  C6 ends in a planned state: DONE / HOLD / RUN / SLOW (still under the planner at the time limit) / STANDBY
plus the scenario's own `expect` (see each)."""
import math

from rig import ll  # noqa: F401  (kept for the rehearsal / demo users)

# HX7-2 (FIXED in HX7-fix, owner option (a): steer clear first, stop second): a range-confirmed vessel that does not
# give way and meets the boat even if it stops. WITH ROOM (a path keeps 15 m inside the corridor): the boat steers clear
# and keeps the minimum distances (H4r / H8r / H19r: the common criteria unchanged + "passed without a HOLD"). WITHOUT
# room (+-10 m corridor, the vessel ON the line: no path keeps 15 m): the boat STOPS at once and raises the loud state
# (the planner's "VESSEL NOT GIVING WAY - STOP - TAKE OVER" + the AURIX's COLLISION RISK - DRIVER DECISION) and a
# contact by THAT vessel with the STANDING boat is "UNAVOIDABLE BY DESIGN" (harbour autonomy cannot make a stopped boat
# safe against a vessel that ignores it: the captain takes over) - judged by the scenario's own criteria U1-U4 below,
# only for the listed vessel; every other check stays as it is.
UNAVOIDABLE = "unavoidable by design: no path keeps 15 m in the +-10 m corridor; the boat stops + alarms, the captain takes over"
FINDING_3 = ("HX7-3 (open): WITH room the steer-clear path is found, but with the real simulated sensors the decision "
             "flickers (track sigma / AIS twins): ONE U13 STOP cycle latches the AURIX HOLD (the captain RESUMEs) - "
             "robust escape hysteresis = a later step")
ALM_COLL = 0x800000 | 0x1000000                     # proto ALARMS: COLLISION RISK / COLLISION RISK - DRIVER DECISION
ARRIVED = 15            # app/harbour.h HARB_WHY_ARRIVED (a planned stop: the route's last waypoint)
ROUTE = [(0.0, 0.0, 5.0), (80.0, 0.0, 5.0), (150.0, 0.0, 5.0)]


def mover(oid, kind, meet, at, hdg, speed, ln, beam, h, rcs, refl, ais=None, span=400.0):
    """A straight mover: at `meet` T seconds after t0, heading hdg (deg true: 0 = north, 90 = east)."""
    dx, dy = math.sin(math.radians(hdg)), math.cos(math.radians(hdg))
    a = (meet[0] - dx * speed * at, meet[1] - dy * speed * at)
    b = (a[0] + dx * span * 2.0, a[1] + dy * span * 2.0)
    return {"id": oid, "kind": kind, "path": [[a[0], a[1]], [b[0], b[1]]], "speed": speed, "len": ln, "beam": beam,
            "h": h, "rcs": rcs, "refl": refl, "ais": ais}


def vessel(oid, meet, at, hdg, speed, ln=9.0, beam=3.0, ais=None):
    return mover(oid, "vessel", meet, at, hdg, speed, ln, beam, 2.0, 0.8, 0.4, ais)


def moored(oid, x, y, ln=9.0, beam=3.0):
    return {"id": oid, "kind": "moored", "x": x, "y": y, "len": ln, "beam": beam, "hdg": 90.0, "h": 2.0, "rcs": 0.7,
            "refl": 0.5}


def pier(oid, x1, y1, x2, y2, w=6.0):
    return {"id": oid, "kind": "pier", "x1": x1, "y1": y1, "x2": x2, "y2": y2, "w": w, "h": 2.0, "rcs": 1.0,
            "refl": 0.3}


def buoy(oid, x, y):
    return {"id": oid, "kind": "buoy", "x": x, "y": y, "len": 1.0, "beam": 1.0, "hdg": 0.0, "h": 1.2, "rcs": 0.4,
            "refl": 0.6}


def S(name, title, objects, **kw):
    d = {"name": name, "title": title, "objects": objects, "route": ROUTE, "max_kn": 5.0, "corr_m": 10.0,
         "secs": 100.0, "world_delay_s": 24.0, "expect": {}}
    d.update(kw)
    return d


def build():
    sc = []
    sc.append(S("H1", "empty harbour exit", [], expect={"end": "DONE"}))
    sc.append(S("H2", "arrival: the quay ends the route (final STOP, 18 m from the pier head)",
                [pier("quay", 175.0, -40.0, 175.0, 40.0)], expect={"end": "DONE"}))
    sc.append(S("H3", "moored boats on both sides of a wide channel (+-24 m)",
                [moored("m1", 40.0, 24.0), moored("m2", 65.0, 24.0), moored("m3", 50.0, -24.0),
                 moored("m4", 80.0, -24.0)], expect={"end": "DONE"}))
    sc.append(S("H3n", "moored boats on both sides of a NARROW channel (+-17 m): it may stop for them (HX7 finding: it "
                "holds ~30 m before them, 11-14 m hull gap, never touches)",
                [moored("m1", 40.0, 17.0), moored("m2", 65.0, 17.0), moored("m3", 50.0, -17.0),
                 moored("m4", 80.0, -17.0)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H4", "head-on vessel (AIS) on the line",
                [vessel("headon", (70.0, 1.0), 38.0, 270.0, 2.0, ais=237000004)], expect={"safe_pass_or_hold": 1, "unavoidable": ("headon",)}))
    sc.append(S("H4r", "head-on vessel that does NOT give way, passing 24 m to port of the line (room: steer clear, keep 15 m)",
                [vessel("headon", (70.0, 24.0), 38.0, 270.0, 2.0, ln=6.0, beam=2.5, ais=237000014)],
                xfail=FINDING_3, expect={"safe_pass_or_hold": 1, "passed_no_hold": 1}))
    sc.append(S("H5", "crossing from starboard (no AIS)",
                [vessel("cross_s", (70.0, 0.0), 38.0, 0.0, 2.0)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H6", "crossing from port (AIS)",
                [vessel("cross_p", (70.0, 0.0), 38.0, 180.0, 2.0, ais=237000006)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H7", "overtaking a slow boat on the line",
                [vessel("slow", (70.0, 1.0), 38.0, 90.0, 0.5, ln=6.0, beam=2.5)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H8", "being overtaken by a fast vessel from astern",
                [vessel("fast", (55.0, 4.0), 32.0, 90.0, 3.5, ln=9.0)], expect={"safe_pass_or_hold": 1, "unavoidable": ("fast",)}))
    sc.append(S("H8r", "overtaken by a fast vessel that does NOT give way, 24 m to port (room: keep 15 m)",
                [vessel("fast", (55.0, 24.0), 32.0, 90.0, 3.5, ln=6.0, beam=2.5)],
                xfail=FINDING_3, expect={"safe_pass_or_hold": 1, "passed_no_hold": 1}))
    sc.append(S("H9", "a kayak (no AIS) crossing ahead",
                [mover("kayak", "kayak", (70.0, 0.0), 36.0, 0.0, 1.0, 4.0, 0.7, 0.4, 0.15, 0.5)],
                expect={"safe_pass_or_hold": 1}))
    sc.append(S("H10", "a swimmer (no AIS) near the line",
                [mover("swimmer", "swimmer", (72.0, 3.0), 36.0, 180.0, 0.3, 0.6, 0.5, 0.3, 0.03, 0.15)],
                expect={"safe_pass_or_hold": 1, "known_limit": "a swimmer < ~18 m is inside the LiDAR blind zone"}))
    sc.append(S("H11", "a buoy on the line", [buoy("buoy", 75.0, 0.5)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H12", "radar lost at 20 s", [], faults=[{"t": 20.0, "what": "radar_off"}],
                expect={"degraded_after": 20.0}))
    sc.append(S("H13", "LiDAR lost at 20 s", [], faults=[{"t": 20.0, "what": "lidar_off"}],
                expect={"degraded_after": 20.0}))
    sc.append(S("H14", "radar AND LiDAR lost at 20 s", [], faults=[{"t": 20.0, "what": "radar_off"},
                                                                   {"t": 20.0, "what": "lidar_off"}],
                expect={"hold_within": (20.0, 6.0)}))
    sc.append(S("H15", "the shore link lost at 20 s (harbour lost-link: STANDBY first, then the kill; harb_ll_grace_s = 0 today)", [],
                faults=[{"t": 20.0, "what": "link_lost"}], secs=125.0, stop_on_standby=False, stop_on_done=False,
                expect={"link_lost": 20.0}))
    sc.append(S("H16", "GPS spoofed (position jump) at 25 s", [], faults=[{"t": 25.0, "what": "gps_spoof"}],
                expect={"hold_within": (25.0, 8.0)}))
    sc.append(S("H17", "a camera-only false obstacle ahead (no range return): slow, never a turn",
                [], faults=[{"t": 8.0, "what": "camera_ghost"}], expect={"camera_only": 8.0}))
    sc.append(S("H18", "an AIS ghost ahead (no range return): slow, never a turn",
                [], faults=[{"t": 8.0, "what": "ais_ghost"}], expect={"ais_only": 8.0}))
    sc.append(S("H19", "a vessel that does not give way (fast, head-on, on the line)",
                [vessel("rogue", (70.0, 0.0), 38.0, 270.0, 3.5, ais=237000019)],
                expect={"safe_pass_or_hold": 1, "unavoidable": ("rogue",)}))
    sc.append(S("H19r", "a fast vessel that does NOT give way, head-on 24 m to port (room: steer clear, keep 15 m)",
                [vessel("rogue", (70.0, 24.0), 38.0, 270.0, 3.5, ln=6.0, beam=2.5, ais=237000029)],
                xfail=FINDING_3, expect={"safe_pass_or_hold": 1, "passed_no_hold": 1}))
    sc.append(S("H20", "the planner crashes at 20 s (the AURIX lease lapses -> HOLD)", [],
                faults=[{"t": 20.0, "what": "planner_crash"}], expect={"hold_within": (20.0, 5.0)}))
    return sc


def by_name():
    return {s["name"]: s for s in build()}


# ------------------------------------------------------------------ evaluation
def evaluate(spec, ep, out):
    """[(check name, ok, detail)] for one finished episode."""
    r = []
    ex = spec.get("expect", {})
    corr = float(spec.get("corr_m", 10.0))
    S_ = ep.samples
    if not S_:
        return [("RIG", False, "no samples: %s" % (out.get("why") or "no pose"))]
    if out.get("upload") == 0 and out.get("start") == 6 and ex.get("start_may_be_refused"):
        # a random world with a vessel already in sight at the start: the planner's STOP flag keeps the AURIX from
        # entering (NO_LEASE / planner STOP): correct and safe - the boat never moved
        moved = max(s[4] for s in S_) if S_ else 0.0
        return [("E START refused (NACK 6) while the planner says STOP at the start, the boat never moves",
                 moved < 0.5, "max SOG %.2f kn" % moved)]
    if out.get("upload") != 0 or out.get("start") != 0:
        r.append(("RIG-START", False, "route upload %r, START %r (0 = ok)" % (out.get("upload"), out.get("start"))))
    # PRECONDITIONS (review HIGH): a boat that never left, a fault that never fired or an object that never came near
    # must not pass anything
    xmax = max(q[1] for q in S_)
    ran = any(q[5] in ("RUN", "SLOW") for q in S_)
    # (an "unavoidable" scenario may stop at once for the vessel that does not give way - by design, far away; H8's
    # overtaker is a threat from the start: there P1 = RUN / SLOW seen + P3 the encounter; every other scenario >= 25 m)
    need_x = 0.0 if ex.get("unavoidable") else 25.0    # (H8: the overtaker is already a threat at the start)
    r.append(("P1 the boat drove: RUN / SLOW seen and >= %.0f m covered" % need_x, ran and xmax >= need_x,
              "RUN / SLOW %s, max x %.0f m" % (ran, xmax)))
    fired = {e[1] for e in ep.events}
    miss = [f["what"] for f in spec.get("faults", []) if "fault " + f["what"] not in fired]
    if spec.get("faults"):
        r.append(("P2 every injected fault fired", not miss, "not fired: %s" % miss if miss else "all fired"))
    objs = [o["id"] for o in spec.get("objects", []) if o["kind"] != "pier"]
    if ex.get("safe_pass_or_hold") and objs:
        mg0 = out.get("min_gap", {})
        close = min((v[0] for k, v in mg0.items() if k in objs), default=999.0)
        r.append(("P3 the encounter happened (the object came within 40 m hull to hull)", close < 40.0,
                  "closest approach to %s: %.1f m" % (objs, close)))
    # C1 / C2 from the per-object minima + the samples
    mg = out.get("min_gap", {})
    worst = min(((v[0], k) for k, v in mg.items()), default=(999.0, None))
    storm = str(spec.get("name", "")).startswith("S") and str(spec.get("name", ""))[1:].isdigit()
    if storm:
        # a random world: a vessel that ignores the boat may run into a boat that is STANDING (finding HX7-2, counted,
        # not failed here: the library's H4 / H8 / H19 show it); a contact while the boat itself MOVES fails
        mov = min((s[8] for s in S_ if s[4] > 0.5), default=999.0)
        r.append(("C1 never touches while the boat moves (gap >= 0.5 m)", mov >= 0.5,
                  "min hull gap while moving %.1f m; overall %.1f m to %s (struck at rest: %s)" % (
                      mov, worst[0], worst[1], worst[0] < 0.5 and mov >= 0.5)))
    elif ex.get("unavoidable"):
        unv = set(ex["unavoidable"])
        oth = min(((v[0], k) for k, v in mg.items() if k not in unv), default=(999.0, None))
        r.append(("C1 never touches any OTHER object (gap >= 0.5 m)", oth[0] >= 0.5, "min hull gap %.1f m to %s" % oth))
        # U1-U4 (HX7-fix, the scenario's own criteria for the vessel that does not give way, see UNAVOIDABLE)
        cont = [s for s in S_ if any(o == k and g < 0.5 for o, g, _m in s[17] for k in unv)]
        t_c = cont[0][0] if cont else None
        ny = [s[0] for s in S_ if s[19] and "NOT GIVING WAY - STOP" in (s[14] or "")]
        r.append(("U1 the not-yielding STOP + TAKE OVER state raised (before any contact)",
                  bool(ny) and (t_c is None or ny[0] < t_c), "first at %s s, contact at %s s" % (ny[0] if ny else None, t_c)))
        al = [s[0] for s in S_ if s[18] & 0x1000000]          # DRIVER DECISION (COLLISION alone = steering clear)
        r.append(("U2 the AURIX's COLLISION RISK / DRIVER DECISION alarm on (before any contact)",
                  bool(al) and (t_c is None or al[0] < t_c), "first at %s s" % (al[0] if al else None)))
        st = [s[0] for s in S_ if s[5] == "HOLD" and s[4] < 0.5]
        r.append(("U3 the boat STOPPED (HOLD, SOG < 0.5 kn) before any contact",
                  bool(st) and (t_c is None or st[0] < t_c), "standing at %s s" % (st[0] if st else None)))
        moving = [(s[0], round(s[4], 1)) for s in cont if s[4] >= 0.5]
        r.append(("U4 a contact only with the STANDING boat (%s)" % UNAVOIDABLE, not moving,
                  "contact while moving %s; contact %s" % (moving[:2], "none" if t_c is None else "at %.1f s (by design)" % t_c)))
    else:
        r.append(("C1 never touches (gap >= 0.5 m)", worst[0] >= 0.5, "min hull gap %.1f m to %s" % worst))
    viol = []
    for t, x, y, hdg, sog, state, xte, mode, gap, cap, moving, oid, kcap, why, txt, ll_s, thr, allg, *_r in S_:
        if state in ("RUN", "SLOW"):
            for oid2, g2, mv2 in allg:                  # EVERY object against its own limit (review)
                lim = (15.0 if mv2 else 8.0) - 0.5
                if g2 < lim and oid2 not in ex.get("excused", ()):
                    viol.append((round(t, 1), oid2, round(g2, 1), state))
    r.append(("C2 distance kept while RUN / SLOW (15 m moving / 8 m static)", not viol,
              "first violations %s" % (viol[:3],) if viol else "none"))
    unv0 = set(ex.get("unavoidable") or ())
    hold_gaps = [(round(s[0], 1), o, round(g, 1)) for s in S_ if s[5] == "HOLD" and s[4] < 0.3
                 and not (storm and s[10]) for o, g, _m in s[17] if o not in unv0]
    hold_gaps = [h for h in hold_gaps if h[2] < 999] or [(0, None, 999.0)]
    stand = min((g[2] for g in hold_gaps), default=999.0)
    r.append(("C2b a HOLD stands with room (>= 2 m at the standstill)", stand >= 2.0,
              "least gap at standstill %.1f m" % stand))
    offc = [(round(s[0], 1), round(s[6], 1)) for s in S_ if s[5] in ("RUN", "SLOW") and s[6] is not None
            and abs(s[6]) > corr + 1.0]
    r.append(("C3 inside the corridor while RUN / SLOW", not offc, "outside: %s" % (offc[:3],) if offc else "never"))
    smax = max(s[4] for s in S_)
    r.append(("C4 speed <= 5.5 kn", smax <= 5.5, "max SOG %.2f kn" % smax))
    # end state
    last = S_[-1]
    fin_ok = last[5] in ("DONE", "HOLD", "RUN", "SLOW", "OFF") or last[7] == "STANDBY"
    r.append(("C6 ends in a planned state (DONE / HOLD / RUN / SLOW / STANDBY: not still driving off the plan)", fin_ok, "end state %s, mode %s, SOG %.1f" % (last[5], last[7],
                                                                                                    last[4])))
    sb = ep.standby_check() if not ex.get("link_lost") else None
    if sb is not None:
        r.append(("C5 STANDBY wins (mode STANDBY within 4 s)", sb, "STANDBY request -> %s" % sb))
    # own expectations
    if ex.get("end") == "DONE":
        reached = any(s[5] == "DONE" or (s[5] == "HOLD" and s[13] == ARRIVED) for s in S_) or last[7] == "STANDBY"
        reached = reached and xmax >= spec["route"][-1][0] - 25.0           # (and it really got to the end)
        r.append(("E arrives (DONE, final STOP) at the route's end", reached, "states %s" % sorted({s[5] for s in S_})))
    if ex.get("safe_pass_or_hold"):
        seen_hold = any(s[5] == "HOLD" for s in S_)
        done = any(s[5] == "DONE" or (s[5] == "HOLD" and s[13] == ARRIVED) for s in S_) or last[7] == "STANDBY"
        passed = last[1] > 120.0
        r.append(("E passed it or stopped for it (no contact either way)", seen_hold or done or passed,
                  "HOLD %s, DONE %s, last x %.0f" % (seen_hold, done, last[1])))
    if ex.get("passed_no_hold"):
        held = [s[0] for s in S_ if s[5] == "HOLD" and s[13] != ARRIVED]
        passed = last[1] > 120.0 or any(s[5] == "DONE" for s in S_)
        r.append(("E steered clear of the vessel that does not give way: passed, no HOLD", passed and not held,
                  "last x %.0f, HOLD at %s" % (last[1], held[:1])))
    if "degraded_after" in ex:
        t_f = ex["degraded_after"]
        late = [s for s in S_ if s[0] > t_f + 12.0 and s[7] == "AUTO" and s[4] > 2.5]
        stopped = any(s[5] == "HOLD" for s in S_ if s[0] > t_f)
        r.append(("E one sensor lost: <= 2.5 kn after 12 s (or HOLD)", not late or stopped,
                  "fast after: %s" % (late[:2],) if late else "slow / stopped"))
    if "hold_within" in ex:
        t_f, win = ex["hold_within"]
        h = [s[0] for s in S_ if s[0] >= t_f and s[5] in ("HOLD", "OFF") or (s[0] >= t_f and s[7] == "STANDBY")]
        pre = any(q[5] in ("RUN", "SLOW") for q in S_ if q[0] < t_f)         # it was under way when the fault hit
        ok = bool(h) and (h[0] - t_f) <= win and pre
        slow_after = [s for s in S_ if s[0] > t_f + win + 12.0 and s[4] > 0.8]
        r.append(("E STOP within %.0f s of the fault, then standing" % win, ok and not slow_after,
                  "HOLD / STANDBY after %s s; still moving later %s" % (None if not h else round(h[0] - t_f, 1),
                                                                         slow_after[:1])))
    if "link_lost" in ex:
        t_f = ex["link_lost"]
        sb_t = [s[0] for s in S_ if s[0] >= t_f and s[7] == "STANDBY"]
        r.append(("E link lost: STANDBY (idle + neutral first) within 10 s", bool(sb_t) and sb_t[0] - t_f <= 10.0,
                  "STANDBY after %s s" % (None if not sb_t else round(sb_t[0] - t_f, 1))))
        moving_late = [s for s in S_ if sb_t and s[0] > sb_t[0] + 15.0 and s[4] > 0.8]
        r.append(("E link lost: the boat is stopped 15 s after STANDBY", not moving_late and bool(sb_t),
                  "moving later %s" % (moving_late[:1],)))
        kills = [q[0] for q in S_ if q[15] == 2]
        first_kill = kills[0] if kills else None
        # harb_ll_grace_s is 0 in today's firmware (W147: the owner's grace is the HX8 firmware step): a lost link in
        # harbour = the usual near-shore procedure: detection 3 s, then idle + neutral, then the kill
        r.append(("E link lost: no ENGINE KILL before the detection time (>= 3 s after the loss)",
                  first_kill is None or first_kill - t_f >= 3.0,
                  "first kill %s s after the loss" % (None if first_kill is None else round(first_kill - t_f, 1))))
        r.append(("E link lost: ... and the engine IS stopped by the end of the run (UNCREWED)",
                  first_kill is not None, "ll_state 2 seen: %s" % (first_kill is not None)))
        idle = [q for q in S_ if sb_t and q[0] >= sb_t[0] + 8.0 and (q[16] is None or q[16] > 1.0)]
        r.append(("E link lost: idle (throttle <= 1 %) 8 s after STANDBY", bool(sb_t) and not idle,
                  "throttle above 1 %%: %s" % ([(q[0], q[16]) for q in idle[:1]],)))
    if "camera_only" in ex or "ais_only" in ex:
        t_f = ex.get("camera_only", ex.get("ais_only"))
        turned = [s for s in S_ if s[0] > t_f + 3.0 and abs(s[2]) > 2.5 and s[5] in ("RUN", "SLOW")]
        slowed = any(s[5] in ("SLOW", "HOLD") and s[0] > t_f for s in S_)
        r.append(("E never a turn for a camera / AIS-only object (lateral offset < 2.5 m)", not turned,
                  "turned %s" % (turned[:1],) if turned else "straight"))
        r.append(("E ... but it slows / stops", slowed, "slow or hold seen: %s" % slowed))
    return r
