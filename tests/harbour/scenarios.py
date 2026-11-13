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
# only for the listed vessel; every other check stays as it is. H8 / H8r: the overtaker is already a threat when the
# captain would start - the READY lease is then a STOP (review M2) and the START is refused (NACK 6): the boat never
# moves under harbour autonomy (judged by evaluate()'s "start refused" path, as for the random worlds).
UNAVOIDABLE = "unavoidable by design: no path keeps 15 m in the +-10 m corridor; the boat stops + alarms, the captain takes over"
FINDING_3 = ("HX7-3 (HX7-fix2: persistence built, these still fail): in a +-10 m corridor (usable +-5.75 m) a vessel 24 m "
             "off the line needs 15 m + both hull halves (6 m) + 2 sigma of its predicted position (radar sigma grows to "
             "~3-4 m over 10 s) = 27-29 m of 29.75 m: the escape is feasible only in some cycles for SECONDS at a time "
             "(rig events: U13 'no free path', blockers = the vessel) - a 0.8 s persistence cannot bridge it; H8r is "
             "refused at the START (the overtaker is already a threat)")
# HX7-fix3 (owner answer 4, 2026-11-13): H4r / H8r / H19r are EXPECTED SAFE STOP tests (no longer XFAIL): the boat
# must stop and hold and never come closer to the vessel than the minimum distance (15 m to a moving vessel, hull to
# hull, on the estimated AND the true position). A wider corridor is a per-route owner decision; the default stays
# +-10 m (docs/harbour_design.md open notes).
SAFE_STOP_WHY = ("expected safe STOP: in a +-10 m corridor (usable +-5.75 m) a vessel 24 m off the line needs 15 m + "
                 "both hull halves (6 m) + 2 sigma of its predicted position (~3-4 m) = 27-29 m of separation, only "
                 "29.75 m is available - no path keeps 15 m reliably, so the planner STOPs and holds; the minimum "
                 "distance must still hold (owner answer 4)")
SS0_NEAR_M = 100.0      # SS0 (review r1 M3): a U5 STOP counts for the vessel only while it is the closest object this near
SAFE_STOP_MIN_M = 15.0  # the minimum distance to a moving vessel (design I88), never less
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
                [vessel("headon", (70.0, 1.0), 38.0, 270.0, 2.0, ais=237000004)], expect={"safe_pass_or_hold": 1, "unavoidable": ("headon",), "min_x": 10.0}))
    sc.append(S("H4r", "head-on vessel that does NOT give way, passing 24 m to port of the line (room: steer clear, keep 15 m)",
                [vessel("headon", (70.0, 24.0), 38.0, 270.0, 2.0, ln=6.0, beam=2.5, ais=237000014)],
                why=SAFE_STOP_WHY, expect={"safe_pass_or_hold": 1, "safe_stop": ("headon",)}))
    sc.append(S("H5", "crossing from starboard (no AIS)",
                [vessel("cross_s", (70.0, 0.0), 38.0, 0.0, 2.0)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H6", "crossing from port (AIS)",
                [vessel("cross_p", (70.0, 0.0), 38.0, 180.0, 2.0, ais=237000006)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H7", "overtaking a slow boat on the line",
                [vessel("slow", (70.0, 1.0), 38.0, 90.0, 0.5, ln=6.0, beam=2.5)], expect={"safe_pass_or_hold": 1}))
    sc.append(S("H8", "being overtaken by a fast vessel from astern",
                [vessel("fast", (55.0, 4.0), 32.0, 90.0, 3.5, ln=9.0)], expect={"safe_pass_or_hold": 1, "unavoidable": ("fast",), "min_x": 0.0,
                                                                     "start_may_be_refused": 1, "refused_for": "not_yielding"}))
    sc.append(S("H8c", "being overtaken by a fast vessel that does NOT give way, the start CLEAR (no threat at "
                "the start: it is 275 m astern at t0, on the line, 5.5 m/s): START passes, it closes in, the not-yielding STOP",
                # HX7-fix3: the HX7-fix2 path [(-40, -60), (-40, 4), (600, 4)] never reached the line - simworld moves an
                # object along path[0] -> path[1] only (back and forth), so the vessel stayed 40 m astern (closest
                # 42.2 m). A first fix-3 try (160 m astern, 5 m/s) was refused at the START: 32 s to reach the standing
                # boat < the planner's horizon_s 40 = a threat at the start. Now: 275 m astern at 5.5 m/s = 50 s to the
                # standing boat (clear at the start); with the boat under way (~2 m/s) it catches up around T ~ 75 s,
                # x ~ 140 m - so this scenario has its own longer route (250 m) and run time. The 40 m encounter (P3)
                # and every other criterion unchanged
                [vessel("fast", (-275.0, 1.0), 0.0, 90.0, 5.5)],
                route=[(0.0, 0.0, 5.0), (120.0, 0.0, 5.0), (250.0, 0.0, 5.0)], secs=130.0,
                expect={"safe_pass_or_hold": 1, "unavoidable": ("fast",), "min_x": 10.0}))
    sc.append(S("H8r", "overtaken by a fast vessel that does NOT give way, 24 m to port (room: keep 15 m)",
                [vessel("fast", (55.0, 24.0), 32.0, 90.0, 3.5, ln=6.0, beam=2.5)],
                why=SAFE_STOP_WHY, expect={"safe_pass_or_hold": 1, "safe_stop": ("fast",), "start_may_be_refused": 1,
                                           "refused_for": "not_yielding"}))
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
                expect={"safe_pass_or_hold": 1, "unavoidable": ("rogue",), "min_x": 10.0}))
    sc.append(S("H19r", "a fast vessel that does NOT give way, head-on 24 m to port (room: steer clear, keep 15 m)",
                [vessel("rogue", (70.0, 24.0), 38.0, 270.0, 3.5, ln=6.0, beam=2.5, ais=237000029)],
                why=SAFE_STOP_WHY, expect={"safe_pass_or_hold": 1, "safe_stop": ("rogue",), "min_x": 10.0}))
    # (H19r min_x 10 m as H19: the fast head-on vessel is judged not giving way ~150 m away and the expected safe STOP
    # comes at ~22 m - the boat left (RUN / SLOW + 10 m), SS1 / SS2 / P3 judge the encounter itself; owner answer 4)
    sc.append(S("H20", "the planner crashes at 20 s (the AURIX lease lapses -> HOLD)", [],
                faults=[{"t": 20.0, "what": "planner_crash"}], expect={"hold_within": (20.0, 5.0)}))
    return sc


def by_name():
    return {s["name"]: s for s in build()}


# ------------------------------------------------------------------ evaluation
def _min_gaps(S_, oid):
    """(least hull gap to oid on the estimated position, on the TRUE position, true samples / all samples)."""
    est = min((g for s in S_ for o, g, _m in s[17] if o == oid), default=999.0)
    tru = [g for s in S_ if len(s) > 22 and s[22] for o, g in s[22] if o == oid]
    return est, min(tru, default=999.0), len(tru), len(S_)


def _safe_stop_checks(S_, ex, ep_stops=None):
    """HX7-fix3 (owner answer 4): an EXPECTED SAFE STOP - stopped (HOLD, SOG < 0.5 kn) before the vessel's closest
    approach, held still until then, and never closer than SAFE_STOP_MIN_M on the estimated AND the true position."""
    r = []
    for oid in ex["safe_stop"]:
        est, tru, n_t, n = _min_gaps(S_, oid)
        r.append(("SS1 never closer than %.0f m to %s (hull to hull, estimated AND true position)" % (SAFE_STOP_MIN_M, oid),
                  est >= SAFE_STOP_MIN_M and tru >= SAFE_STOP_MIN_M and n_t >= 0.9 * n,
                  "least gap %.1f m estimated, %.1f m true (%d / %d samples with the true position)" % (est, tru, n_t, n)))
        def gap_at(s):
            g = [g for o, g, _m in s[17] if o == oid] + [g for o, g in (s[22] if len(s) > 22 and s[22] else []) if o == oid]
            return min(g, default=999.0)
        tc = min(S_, key=gap_at)[0]                     # the closest approach (estimated or true, review M3)
        # the standstill AFTER the boat got under way (a HOLD at the start, before RUN, is no stop for the vessel)
        go = [s[0] for s in S_ if s[5] in ("RUN", "SLOW") and s[4] >= 1.0]
        st = [s[0] for s in S_ if go and s[0] > go[0] and s[5] == "HOLD" and s[4] < 0.5]
        # SS0 (review M3): the HOLD is the planner's U13 not-yielding STOP for THIS vessel (its track matched to it)
        # (the planner names the vessel in a U13; a U5 "no free path" names no blocker - then THIS vessel must be the
        # closest object on the TRUE position, within SS0_NEAR_M at the STOP: never a STOP far from the encounter)
        def tied(e):
            if "U13" in e["reason"]:
                return any(b.get("match") == oid and b.get("match_d", 999.0) <= EARLY_MATCH_M
                           for b in e.get("not_yielding", []))
            if "U5" in e["reason"]:
                q = min(S_, key=lambda s: abs(s[0] - e["t"]))
                g = dict(q[22]) if len(q) > 22 and q[22] else {}
                return oid in g and g[oid] <= SS0_NEAR_M and g[oid] == min(g.values())
            return False
        ny = [e for e in (ep_stops or []) if st and e["t"] <= st[0] and tied(e)]
        r.append(("SS0 the STOP before the standstill is the planner's U13 (naming %s) or U5 with %s the closest "
                  "object within %.0f m (true position)" % (oid, oid, SS0_NEAR_M), bool(ny) and bool(st),
                  "tied STOP %s, standing at %s s" % ((ny[0]["t"], ny[0]["reason"][:40]) if ny else None,
                                                      st[0] if st else None)))
        moved = st and [(s[0], round(s[4], 2)) for s in S_ if st[0] <= s[0] <= tc and s[4] >= 0.5]
        r.append(("SS2 STOPPED (HOLD, SOG < 0.5 kn) before the closest approach (%.1f s) and held still until it" % tc,
                  bool(st) and st[0] <= tc and not moved,
                  "standing at %s s; moving after it %s" % (st[0] if st else None, (moved or [])[:2])))
    return r


def _halting_stops(S_, ep):
    """The STOP(s) that ended the boat's progress (review r2 H-1): the last logged STOP at or before the moment the
    boat first reached its furthest point AND every STOP logged after it up to 5 s later (the reason text may change
    while it comes to rest) - every one of them must carry the evidence."""
    xmax = max(q[1] for q in S_)
    t_x = min(q[0] for q in S_ if q[1] >= xmax - 0.5)
    stops = getattr(ep, "stops", None) or []
    before = [e for e in stops if e["t"] <= t_x]
    t_a = before[-1]["t"] if before else t_x
    return [e for e in stops if t_a <= e["t"] <= t_x + 5.0], t_x


def _early_stop_class(S_, ep, out):
    """HX7-fix3 (owner answer 3; review r1 H2 / M4, r2 H-1 / L-1): an EVIDENCED early STOP in a random world (the boat
    stopped before 25 m): PASS only if ALL hold - EVERY STOP that halted the boat (_halting_stops) names a TRACK, every
    track it names matches a REAL world object (track position within EARLY_MATCH_M of the object) and one of them is
    within EARLY_RANGE_M (hull gap at the STOP, from the TRUE position), the boat ends in HOLD (standing) or STANDBY,
    and there is no contact (every object's hull gap >= 0.5 m all the time, on the estimated AND the true position,
    the true position in >= 90 % of the samples). Each condition is its own check."""
    r = []
    hs, t_x = _halting_stops(S_, ep)

    def named(e):
        return [b for b in e.get("blockers", []) if b.get("track")]

    def real_ok(e):
        tr = named(e)
        real = [b for b in tr if b.get("match") is not None and b["match_d"] <= EARLY_MATCH_M]
        near = [b for b in real if b.get("gap") is not None and b["gap"] <= EARLY_RANGE_M]
        return bool(tr) and len(real) == len(tr) and bool(near) and bool(e.get("true_pos"))
    r.append(("P1e-a every STOP that halted the boat names a TRACK", bool(hs) and all(named(e) for e in hs),
              "halting STOPs (furthest point at %.1f s): %s" % (t_x, [(e["t"], e["reason"][:50]) for e in hs][:4])))
    r.append(("P1e-b ... EVERY track they name = a REAL object (<= %.0f m), one within %.0f m (true position)" % (
              EARLY_MATCH_M, EARLY_RANGE_M), bool(hs) and all(real_ok(e) for e in hs),
              "tracks %s" % ([(e["t"], e.get("true_pos"), [(b["id"], b.get("match"), b.get("match_d"), b.get("gap"))
                                                             for b in named(e)]) for e in hs][:4],)))
    # HX7-fix4 (S14, stricter): a 'static-map' blocker beside the named track(s) was never checked - the planner may
    # have stopped for the static map, not for the track. Every occupied static-map cell the rig found in the band ahead
    # at such a STOP must be a CHART cell (the real shore / a pier) or lie within EARLY_MATCH_M of a REAL object now
    # (the gateway's own StaticMap read by the rig, rig._static_evidence); none recorded = not evidenced.
    def static_ok(e):
        if not e.get("static_blocker") and "static-map" not in [str(b.get("id")) for b in e.get("blockers", [])]:
            return True
        st = e.get("static") or {}
        cells = st.get("cells") or []
        return bool(cells) and all(c["layer"] == "chart" or (c.get("match") is not None and c["match_d"] <= EARLY_MATCH_M)
                                   for c in cells)

    def static_d(e):
        st = e.get("static")
        if not st:
            return None
        far = [c for c in st.get("cells") or [] if c["layer"] != "chart" and not (c.get("match") is not None and
                                                                                 c["match_d"] <= EARLY_MATCH_M)]
        return {"chart": st.get("chart"), "lidar": st.get("lidar"), "not on a real object": len(far),
                "e.g.": [(c["x"], c["y"], c.get("match"), c["match_d"]) for c in far[:2]]}
    r.append(("P1e-e ... and a 'static-map' blocker of those STOPs = cells on the chart or on a REAL object now (<= %.0f m)"
              % EARLY_MATCH_M, bool(hs) and all(static_ok(e) for e in hs),
              "static-map blockers %s" % ([(e["t"], static_d(e)) for e in hs if static_d(e) is not None][:3],)))
    last = S_[-1]
    r.append(("P1e-c the boat ends in HOLD (standing) or STANDBY", (last[5] == "HOLD" and last[4] < 0.5)
              or last[7] == "STANDBY", "end state %s, mode %s, SOG %.2f" % (last[5], last[7], last[4])))
    mg = out.get("min_gap", {})
    worst = min(((v[0], k) for k, v in mg.items()), default=(999.0, None))
    tsm = [s for s in S_ if len(s) > 22 and s[22]]
    tru = min((g for s in tsm for _o, g in s[22]), default=999.0)
    r.append(("P1e-d no contact (every object >= 0.5 m, estimated AND true position, true in >= 90 % of samples)",
              worst[0] >= 0.5 and tru >= 0.5 and len(tsm) >= 0.9 * len(S_),
              "least gap %.1f m to %s (true %.1f m, %d / %d samples)" % (worst[0], worst[1], tru, len(tsm), len(S_))))
    return r


EARLY_MATCH_M = 6.0     # a STOP's track -> the real object it stands for (centre to centre)
EARLY_RANGE_M = 40.0    # ... within the encounter distance of P3 (hull to hull)
START_STOP_X_M = 3.0    # HX7-fix4: "never got under way" = the TRUE position never 3 m from where it started
FAULT_HOLD_S = 5.0      # HX7-fix4: the injected fault's HOLD within this time (= H20's own window, not a new value)
# HX7-fix4 (owner decision 2026-11-13): the AURIX HOLD cause each INJECTED fault must produce to count as the STOP's
# cause. Only faults whose designed reaction IS a stop are listed: planner_crash -> the kind-4 lease lapses ->
# HARB_WHY_LEASE (app/harbour.h 2). radar_off / lidar_off alone = M0-4 U2 'one sensor lost' = SLOW, never a STOP:
# a STOP after one of them is NOT this class (it must be evidenced by a track, else FAIL + investigate).
FAULT_STOP_WHY = {"planner_crash": (2,)}


def _fault_halted(S_, fired):
    """A listed injected fault fired while the boat was still making progress (its furthest point reached at / after
    the fault): the candidate for the fault-STOP class (the class then asserts every condition)."""
    xmax = max(q[1] for q in S_)
    t_x = min(q[0] for q in S_ if q[1] >= xmax - 0.5)
    return any(f in FAULT_STOP_WHY and t_f <= t_x + 0.5 for f, t_f in fired.items())


def _fault_stop_class(S_, ep, out, fired):
    """HX7-fix4 (owner decision 2026-11-13, S21): an EVIDENCED FAULT STOP in a random world (the boat stopped before
    25 m). PASS only if ALL hold, each its own check: (F-a) the logged STOP cause is a fault the scenario INJECTED -
    it fired (rig event), the boat was RUN / SLOW at the fault, the AURIX's HOLD reason is the one that fault produces
    (FAULT_STOP_WHY) within FAULT_HOLD_S of it, no planner STOP was logged between the fault and that HOLD, and the
    boat's furthest point is reached AFTER the fault (it was still making progress when the fault hit, never stopped
    before it); (F-b) the boat ends in HOLD (standing) or STANDBY;
    (F-c) no contact (every object >= 0.5 m, estimated AND true position, true in >= 90 % of the samples)."""
    r = []
    ev = []
    for f, t_f in sorted(fired.items(), key=lambda a: a[1]):
        why = FAULT_STOP_WHY.get(f)
        if not why:
            continue
        # (review r1 H1) under way for the WHOLE second before the fault, not one sample
        pre = [q for q in S_ if t_f - 1.0 <= q[0] <= t_f]
        moving = bool(pre) and all(q[5] in ("RUN", "SLOW") for q in pre)
        h = [q[0] for q in S_ if t_f <= q[0] <= t_f + FAULT_HOLD_S and q[5] == "HOLD" and q[13] in why]
        # (r1 H1 / H2) NO planner STOP logged anywhere in the run - before the fault (an unexplained STOP must not
        # hide behind the fault) nor after it (nothing else may be what halted the boat)
        pstop = [(e["t"], e["reason"][:40]) for e in (getattr(ep, "stops", None) or [])]
        # (r1 H2, r2 MED-1) from that HOLD to the end the boat stays held: every sample HOLD, or mode STANDBY (OFF
        # only in STANDBY), and it is never driven on: no sample after h + 1 s faster than at the HOLD + 0.3 kn
        v_h = max((q[4] for q in S_ if h and abs(q[0] - h[0]) <= 0.3), default=0.0)
        redrive = [(q[0], q[5], q[7], round(q[4], 2)) for q in S_ if h and q[0] >= h[0] and (
            not (q[5] == "HOLD" or q[7] == "STANDBY") or (q[0] >= h[0] + 1.0 and q[4] > v_h + 0.3))]
        # (r2 MED-2) the rig never RESUMEd after the fault
        res = [e[0] for e in ep.events if e[0] >= t_f and str(e[1]).startswith("RESUME")]
        ev.append((f, t_f, moving, h[0] if h else None, pstop[:3], redrive[:2] + [("RESUME", t) for t in res[:2]]))
    xmax = max(q[1] for q in S_)
    t_x = min(q[0] for q in S_ if q[1] >= xmax - 0.5)
    ok_a = bool(ev) and all(m and h is not None and not ps and not rd for _f, _t, m, h, ps, rd in ev) and \
        all(t_x >= t_f for _f, t_f, _m, _h, _p, _r in ev)
    other = [f for f in fired if f not in FAULT_STOP_WHY]
    r.append(("F-a the STOP that halted the boat is an INJECTED fault's designed reaction (%s), under way the whole "
              "second before the fault, HOLD within %.0f s and held to the end, NO planner STOP in the run" % (", ".join(
                  "%s -> why %s" % (k, v) for k, v in FAULT_STOP_WHY.items()), FAULT_HOLD_S),
              ok_a, "faults (name, t, under way 1 s before, HOLD at, planner STOPs, re-driven) %s; furthest point at %.1f s; other "
                    "faults %s" % (ev, t_x, other)))
    last = S_[-1]
    r.append(("F-b the boat ends in HOLD (standing) or STANDBY", (last[5] == "HOLD" and last[4] < 0.5)
              or last[7] == "STANDBY", "end state %s, mode %s, SOG %.2f" % (last[5], last[7], last[4])))
    r.append(_no_contact(S_, out, "F-c"))
    return r


def _no_contact(S_, out, tag):
    mg = out.get("min_gap", {})
    worst = min(((v[0], k) for k, v in mg.items()), default=(999.0, None))
    tsm = [s for s in S_ if len(s) > 22 and s[22]]
    tru = min((g for s in tsm for _o, g in s[22]), default=999.0)
    return ("%s no contact (every object >= 0.5 m, estimated AND true position, true in >= 90 %% of samples)" % tag,
            worst[0] >= 0.5 and tru >= 0.5 and len(tsm) >= 0.9 * len(S_),
            "least gap %.1f m to %s (true %.1f m, %d / %d samples)" % (worst[0], worst[1], tru, len(tsm), len(S_)))


def _start_stop_class(S_, ep, out, u13, dd, hold):
    """HX7-fix4 (S13): the U13 not-yielding STOP before the boat got under way. PASS only if ALL hold, each its own
    check: P1u's three (U13 STOP, DRIVER DECISION, standstill in HOLD) - unchanged; (S-a) the U13 is the FIRST logged
    STOP and the boat was never RUN / SLOW before it; (S-b) the vessel the U13 names is a REAL object: the SAME track id
    is matched to a real moving object within EARLY_MATCH_M in a logged STOP (centre to centre, the true position), and
    every match of that id names that one object; (S-c) that object really came within 40 m (P3, estimated or true).
    (The never-moved rule = P1s in evaluate; C1 / U4 standing contacts are judged as for every storm world.)"""
    r = [("P1u class UNAVOIDABLE (start): U13 STOP + DRIVER DECISION + standstill", bool(u13) and bool(dd) and bool(hold),
          "U13 at %s s, DRIVER DECISION at %s s, standing at %s s" % (u13[0] if u13 else None, dd[0] if dd else None,
                                                                     hold[0] if hold else None))]
    stops = getattr(ep, "stops", None) or []
    first = stops[0] if stops else None
    ran_before = [q[0] for q in S_ if u13 and q[0] <= u13[0] and q[5] in ("RUN", "SLOW")]
    r.append(("S-a the U13 is the FIRST logged STOP, the boat never RUN / SLOW before it",
              bool(first) and "U13" in first["reason"] and not ran_before,
              "first STOP %s; RUN / SLOW before it %s" % ((first["t"], first["reason"][:40]) if first else None,
                                                         ran_before[:2])))
    ids = {str(b["id"]) for b in (first or {}).get("not_yielding", [])}
    seen = {}
    for e in stops:
        for b in e.get("not_yielding", []) + e.get("blockers", []):
            if str(b.get("id")) in ids and b.get("track") and b.get("match") is not None and e.get("true_pos"):
                seen.setdefault(str(b["id"]), []).append((e["t"], b["match"], b["match_d"], b.get("obj_speed")))
    # (review r1 M3) a REAL MOVING object: its scenario speed > 0.2 m/s (a moored boat / buoy never counts)
    real = {i: [m for m in v if m[2] <= EARLY_MATCH_M and (m[3] or 0.0) > 0.2] for i, v in seen.items()}
    objs = {m[1] for v in seen.values() for m in v}
    ok_b = bool(ids) and all(real.get(i) for i in ids) and len(objs) == 1
    r.append(("S-b the vessel the U13 names is REAL + MOVING: its track id matched to ONE real moving object within %.0f m (true "
              "position) in a logged STOP" % EARLY_MATCH_M, ok_b,
              "named %s; matches (t, object, m) %s" % (sorted(ids), {i: v[:3] for i, v in seen.items()})))
    obj = next(iter(objs)) if len(objs) == 1 else None
    est, tru, _n_t, _n = _min_gaps(S_, obj) if obj else (999.0, 999.0, 0, 0)
    r.append(("S-c ... and that object came within 40 m (estimated or true)", obj is not None and min(est, tru) < 40.0,
              "%s closest %.1f m estimated, %.1f m true" % (obj, est, tru)))
    return r


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
        rf = out.get("refusal") or {}
        out["class"] = "start_refused"
        r0 = [("E START refused (NACK 6) while the planner says STOP at the start, the boat never moves",
               moved < 0.5, "max SOG %.2f kn; the planner's reason %s" % (moved, rf.get("reasons")))]
        if ex.get("refused_for") == "not_yielding":
            # HX7-fix2 (3): the refusal is acceptable ONLY because of the vessel that does not give way
            r0.append(("E ... and the reason is the vessel not giving way (U13, the vessel named)",
                       bool(rf.get("stop")) and rf.get("why") == "harb.why_not_yielding" and bool(rf.get("not_yielding")),
                       "refusal %s" % rf))
        if ex.get("safe_stop"):
            # HX7-fix3 (owner answer 4): the boat never moved = stopped; the minimum distance must still hold
            for oid in ex["safe_stop"]:
                est, tru, n_t, n = _min_gaps(S_, oid)
                r0.append(("SS1 never closer than %.0f m to %s (estimated AND true position)" % (SAFE_STOP_MIN_M, oid),
                           est >= SAFE_STOP_MIN_M and tru >= SAFE_STOP_MIN_M and n_t >= 0.9 * n,
                           "least gap %.1f m estimated, %.1f m true (%d / %d samples)" % (est, tru, n_t, n)))
                close = min(est, tru)
                r0.append(("P3 the encounter happened (the vessel came within 40 m hull to hull)", close < 40.0,
                           "closest %.1f m" % close))
        return r0
    if out.get("upload") != 0 or out.get("start") != 0:
        r.append(("RIG-START", False, "route upload %r, START %r (0 = ok)" % (out.get("upload"), out.get("start"))))
    # PRECONDITIONS (review HIGH): a boat that never left, a fault that never fired or an object that never came near
    # must not pass anything
    xmax = max(q[1] for q in S_)
    ran = any(q[5] in ("RUN", "SLOW") for q in S_)
    # (an "unavoidable" scenario may stop at once for the vessel that does not give way - by design, far away; H8's
    # overtaker is a threat from the start: there P1 = RUN / SLOW seen + P3 the encounter; every other scenario >= 25 m)
    storm0 = str(spec.get("name", "")).startswith("S") and str(spec.get("name", ""))[1:].isdigit()
    if storm0:
        # HX7-fix2 (4): a random world whose boat stopped for a vessel that does NOT give way is the explicit class
        # "unavoidable": evidenced by the planner's U13 STOP + the AURIX's DRIVER DECISION alarm + a standstill in HOLD;
        # its precondition is RUN / SLOW seen + that evidence (never silently dropped: the class is reported)
        u13 = [q[0] for q in S_ if len(q) > 19 and q[19] and "NOT GIVING WAY - STOP" in (q[14] or "")]
        dd = [q[0] for q in S_ if len(q) > 18 and q[18] & 0x1000000]
        hold = [q[0] for q in S_ if q[5] == "HOLD" and q[4] < 0.5]
        fired0 = {}
        for e in ep.events:                             # (r2 LOW-3) the FIRST firing of each fault
            if str(e[1]).startswith("fault ") and e[1][len("fault "):] not in fired0:
                fired0[e[1][len("fault "):]] = e[0]
        if u13 and xmax < 25.0 and not ran:
            # HX7-fix4 (S13): the U13 not-yielding STOP came BEFORE the boat ever got under way (START accepted, the
            # vessel judged a threat at once): the start_refused rule (never moved) + P1u's evidence + the vessel
            # named by the U13 is a REAL object (its track id matched within EARLY_MATCH_M) that came within 40 m
            out["class"] = "start_stop"
            r.extend(_start_stop_class(S_, ep, out, u13, dd, hold))
        elif u13 and xmax < 25.0:
            out["class"] = "unavoidable"
            r.append(("P1u class UNAVOIDABLE: U13 STOP + DRIVER DECISION + standstill (the boat stopped for a vessel "
                      "not giving way before 25 m)", ran and bool(dd) and bool(hold),
                      "U13 at %s s, DRIVER DECISION at %s s, standing at %s s" % (u13[0], dd[0] if dd else None,
                                                                                 hold[0] if hold else None)))
        elif xmax < 25.0 and fired0 and _fault_halted(S_, fired0):
            # HX7-fix4 (owner decision 2026-11-13, S21): the EVIDENCED FAULT-STOP class - the boat was stopped by a
            # fault the scenario INJECTED (all conditions asserted in _fault_stop_class; the 25 m precondition stays
            # for every other world)
            out["class"] = "fault_stop"
            r.extend(_fault_stop_class(S_, ep, out, fired0))
        elif xmax < 25.0 and getattr(ep, "stops", None):
            # HX7-fix3 (owner answer 3): the EVIDENCED early STOP class (all four conditions asserted; the 25 m
            # precondition stays for every other world)
            out["class"] = "early_stop"
            r.extend(_early_stop_class(S_, ep, out))
        else:
            out["class"] = "normal"
    need_x = 0.0 if (storm0 and out.get("class") in ("unavoidable", "early_stop", "fault_stop")) else float(ex.get("min_x", 25.0))                 # (H8: min_x 0 - the overtaker is a threat from the start)
    if storm0 and out.get("class") == "start_stop":
        # HX7-fix4 (S13): a boat that was stopped BEFORE it got under way cannot "drive": P1 here = the start_refused
        # rule (the boat never moved: SOG < 0.5 kn and |x| < START_STOP_X_M all the time) - asserted, never skipped
        smax0 = max(q[4] for q in S_)
        tp = [q[21] for q in S_ if len(q) > 21 and q[21]]
        disp = max((math.hypot(p[0] - tp[0][0], p[1] - tp[0][1]) for p in tp), default=999.0)
        r.append(("P1s the boat never got under way (no RUN / SLOW, SOG < 0.5 kn, TRUE position within %.0f m of the "
                  "start, true in >= 90 %% of samples)" % START_STOP_X_M,
                  not ran and smax0 < 0.5 and disp < START_STOP_X_M and len(tp) >= 0.9 * len(S_),
                  "RUN / SLOW %s, max SOG %.2f kn, true displacement %.1f m (%d / %d samples)" % (
                      ran, smax0, disp, len(tp), len(S_))))
    else:
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
    if ex.get("safe_stop"):
        r.extend(_safe_stop_checks(S_, ex, getattr(ep, "stops", None)))
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
        # HX7-fix3 (owner answer 1; REPLACES HX7-fix2's 3 s mean / 2.5 m, which was not signed off): the ORIGINAL
        # instantaneous check |lateral offset| > 2.5 m, now on the simulator's TRUE boat position (sample [21], the
        # world truth - never the 1 Hz GPS estimate, whose fix noise is 1.5 m per axis), OR the planner's own turn
        # decision (sample [20]). A sample without a fresh true position counts as a TURN (never a silent pass).
        win = [s for s in S_ if s[0] > t_f + 3.0 and s[5] in ("RUN", "SLOW")]
        turned = [s[:8] for s in win if (len(s) > 20 and s[20]) or len(s) < 22 or s[21] is None or abs(s[21][1]) > 2.5]
        r.append(("E never a turn for a camera / AIS-only object (no planner turn, |TRUE lateral offset| <= 2.5 m at "
                  "every sample)", not turned,
                  ("turned / no true position %s" % (turned[:1],)) if turned else
                  ("straight: max |true y| %.2f m over %d samples" % (max(abs(s[21][1]) for s in win), len(win))
                   if win else "no RUN / SLOW samples after the fault")))
        slowed = any(s[5] in ("SLOW", "HOLD") and s[0] > t_f for s in S_)
        r.append(("E ... but it slows / stops", slowed, "slow or hold seen: %s" % slowed))
    return r
