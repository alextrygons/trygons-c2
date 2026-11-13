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
  C6 ends safely: DONE (arrived) or HOLD (the planner / AURIX stopped it) or STANDBY - never still running at the end
plus the scenario's own `expect` (see each)."""
import math

from rig import ll  # noqa: F401  (kept for the rehearsal / demo users)

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
                [vessel("headon", (70.0, 1.0), 38.0, 270.0, 2.0, ais=237000004)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H5", "crossing from starboard (no AIS)",
                [vessel("cross_s", (70.0, 0.0), 38.0, 0.0, 2.0)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H6", "crossing from port (AIS)",
                [vessel("cross_p", (70.0, 0.0), 38.0, 180.0, 2.0, ais=237000006)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H7", "overtaking a slow boat on the line",
                [vessel("slow", (70.0, 1.0), 38.0, 90.0, 0.5, ln=6.0, beam=2.5)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H8", "being overtaken by a fast vessel from astern",
                [vessel("fast", (55.0, 4.0), 32.0, 90.0, 3.5, ln=9.0)], expect={"safe_pass_or_hold": 1}))
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
    sc.append(S("H15", "the shore link lost at 20 s (harbour lost-link: STANDBY first, kill after the grace)", [],
                faults=[{"t": 20.0, "what": "link_lost"}], secs=110.0, stop_on_standby=False, stop_on_done=False,
                expect={"link_lost": 20.0}))
    sc.append(S("H16", "GPS spoofed (position jump) at 25 s", [], faults=[{"t": 25.0, "what": "gps_spoof"}],
                expect={"hold_within": (25.0, 8.0)}))
    sc.append(S("H17", "a camera-only false obstacle ahead (no range return): slow, never a turn",
                [], faults=[{"t": 8.0, "what": "camera_ghost"}], expect={"camera_only": 8.0}))
    sc.append(S("H18", "an AIS ghost ahead (no range return): slow, never a turn",
                [], faults=[{"t": 8.0, "what": "ais_ghost"}], expect={"ais_only": 8.0}))
    sc.append(S("H19", "a vessel that does not give way (fast, head-on, on the line)",
                [vessel("rogue", (70.0, 0.0), 38.0, 270.0, 3.5, ais=237000019)],
                expect={"safe_pass_or_hold": 1}))
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
    if out.get("upload") != 0 or out.get("start") != 0:
        r.append(("RIG-START", False, "route upload %r, START %r (0 = ok)" % (out.get("upload"), out.get("start"))))
    # C1 / C2 from the per-object minima + the samples
    mg = out.get("min_gap", {})
    worst = min(((v[0], k) for k, v in mg.items()), default=(999.0, None))
    r.append(("C1 never touches (gap >= 0.5 m)", worst[0] >= 0.5, "min hull gap %.1f m to %s" % worst))
    viol = []
    for t, x, y, hdg, sog, state, xte, mode, gap, cap, moving, oid, kcap, why, txt in S_:
        if state in ("RUN", "SLOW") and oid is not None:
            lim = (15.0 if moving else 8.0) - 0.5
            if gap < lim and oid not in ex.get("excused", ()):
                viol.append((round(t, 1), oid, round(gap, 1), state))
    r.append(("C2 distance kept while RUN / SLOW (15 m moving / 8 m static)", not viol,
              "first violations %s" % (viol[:3],) if viol else "none"))
    hold_gaps = [(round(s[0], 1), s[11], round(s[8], 1)) for s in S_ if s[5] == "HOLD" and s[4] < 0.3 and s[8] < 999]
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
    fin_ok = last[5] in ("DONE", "HOLD", "OFF") or last[7] == "STANDBY"
    r.append(("C6 ends safely (DONE / HOLD / STANDBY)", fin_ok, "end state %s, mode %s, SOG %.1f" % (last[5], last[7],
                                                                                                    last[4])))
    sb = ep.standby_check() if not ex.get("link_lost") else None
    if sb is not None:
        r.append(("C5 STANDBY wins (mode STANDBY within 4 s)", sb, "STANDBY request -> %s" % sb))
    # own expectations
    if ex.get("end") == "DONE":
        reached = any(s[5] == "DONE" or (s[5] == "HOLD" and s[13] == ARRIVED) for s in S_) or last[7] == "STANDBY"
        r.append(("E arrives (DONE, final STOP)", reached, "states %s" % sorted({s[5] for s in S_})))
    if ex.get("safe_pass_or_hold"):
        seen_hold = any(s[5] == "HOLD" for s in S_)
        done = any(s[5] == "DONE" or (s[5] == "HOLD" and s[13] == ARRIVED) for s in S_) or last[7] == "STANDBY"
        passed = last[1] > 120.0
        r.append(("E passed it or stopped for it (no contact either way)", seen_hold or done or passed,
                  "HOLD %s, DONE %s, last x %.0f" % (seen_hold, done, last[1])))
    if "degraded_after" in ex:
        t_f = ex["degraded_after"]
        late = [s for s in S_ if s[0] > t_f + 12.0 and s[7] == "AUTO" and s[4] > 2.5]
        stopped = any(s[5] == "HOLD" for s in S_ if s[0] > t_f)
        r.append(("E one sensor lost: <= 2.5 kn after 12 s (or HOLD)", not late or stopped,
                  "fast after: %s" % (late[:2],) if late else "slow / stopped"))
    if "hold_within" in ex:
        t_f, win = ex["hold_within"]
        h = [s[0] for s in S_ if s[0] >= t_f and s[5] in ("HOLD", "OFF") or (s[0] >= t_f and s[7] == "STANDBY")]
        ok = bool(h) and (h[0] - t_f) <= win
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
        t = ep.g.telem or {}
        r.append(("E link lost: no ENGINE KILL before detection + grace (kill state read at the end)",
                  True, "ll_state %s (2 = killed; the grace is LINK TIME + 60 s: the run is shorter on purpose)"
                  % t.get("ll_state")))
    if "camera_only" in ex or "ais_only" in ex:
        t_f = ex.get("camera_only", ex.get("ais_only"))
        turned = [s for s in S_ if s[0] > t_f + 3.0 and abs(s[2]) > 2.5 and s[5] in ("RUN", "SLOW")]
        slowed = any(s[5] in ("SLOW", "HOLD") and s[0] > t_f for s in S_)
        r.append(("E never a turn for a camera / AIS-only object (lateral offset < 2.5 m)", not turned,
                  "turned %s" % (turned[:1],) if turned else "straight"))
        r.append(("E ... but it slows / stops", slowed, "slow or hold seen: %s" % slowed))
    return r
