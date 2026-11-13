"""Harbour autonomy's local planner (CLAUDE.md Z, docs/harbour_design.md 7; step HX5 - it STEERS the boat: SERIAL review).

Two parts:
  Planner        PURE (no I/O, no clock of its own: `now` is given): one decision per call from the fused tracks (HX4),
                 the tracker flags, the static map, the own pose, the route leg + corridor.
  HarbourPlanner the gateway glue (runs in the gateway thread: tracks() / flags have no lock - HX4 note): the AURIX's
                 HARBOUR_STAT, the route the gateway checked, the sim gate, the kind-4 navreq fold, audit + status.

The method (design 7.1, AS BUILT - the candidates are LATERAL OFFSET LINES inside the corridor x speeds; the heading
proposal = the leg track + atan((offset - xte) / lookahead), so every candidate path stays inside the corridor by
construction and the heading offsets of the design are sampled through the offsets):
  - candidates: offsets -usable..+usable (usable = corr - beam / 2 - 1 m - edge margin; HX2 note: never lead the boat
    to the edge) every offset_step_m + the current xte (= the STRAIGHT candidate: heading = the leg track), x speeds
    cap x {1, 0.75, 0.5, 0.35}; a speed below min_move_kn is never offered (HX2 note: a cap below the in-gear idle
    speed only brings the throttle to idle - the planner sends STOP instead);
  - each simulated horizon_s with the AURIX's slew (rot_dps) and the boat's speed-dependent turn rate, the speed
    coming down with speed_tau_s (a coast-down: W147 note), against every track's constant-velocity prediction
    (radius = 2 sigma + half size, the sigma growth capped at unc_t_s) and the static map / static tracks;
  - HARD: the path inside the usable corridor, >= min_moving_m from a moving track, >= min_static_m from a static
    object (W145), speed <= the cap; COLREGs: no port turn in a head-on / crossing / being-overtaken encounter;
  - M0-3 (7.2): a TURN (a candidate whose heading leaves the leg track by > turn_deg) only when EVERY track blocking
    the straight candidate (at any speed) is steer_ok (radar / LiDAR confirmed) and, with a sensor degraded (U2), seen
    by a sensor that is still OK; otherwise only the straight candidate at a lower speed, else STOP;
  - COST: deviation from the line, lost time, turning (x3 in a narrow corridor: slowing preferred), a change penalty
    (no flip-flop);
  - nothing feasible = STOP (U5). Every item of the unsure list (7.4) = STOP, except U2 / U9 (slow to degraded_kn)
    and U6 / U8 (slow, then STOP after their time); an unknown unsure reason = STOP (fail closed).
The speed cap is never above the route leg's speed nor the harbour cap (W144): the planner only lowers."""
import math
import time

from . import colregs

KN = 0.514444
# groundstation/proto.py values (this package imports no proto / aurix: test_harbour_sensors HS9; HP13 checks them)
NAV_HARBOUR = 4
MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK = 0, 1, 2
PROFILE_CREWED = 1

DEFAULTS = {
    "min_moving_m": 15.0, "min_static_m": 8.0,          # W145
    "max_kn": 5.0,                                       # W144 (the AURIX clamps again: harb_max_kn)
    "horizon_s": 40.0, "dt_s": 1.0,
    "degraded_kn": 2.0, "narrow_kn": 3.0, "min_move_kn": 1.5,
    "max_dev_deg": 45.0,                                 # below the AURIX's harb_max_dev_deg 60
    "rot_dps": 8.0, "turn_dps_per_kn": 0.6, "speed_tau_s": 6.0,
    "own_beam_m": 2.5, "own_len_m": 6.0, "edge_margin_m": 2.0, "offset_step_m": 1.5,
    "speeds": [1.0, 0.75, 0.5, 0.35],
    "unc_t_s": 10.0, "lookahead_m": 15.0,
    "late_s": 0.4, "late_hold_s": 1.0, "budget_ms": 150.0,
    "conflict_stop_s": 3.0, "ambig_stop_s": 3.0,
    "unc_m": 5.0, "corr_m": 10.0, "lease_ms": 1000,
    "moving_mps": 0.5, "turn_deg": 3.0, "narrow_ahead_m": 40.0,
    "stat_max_age_s": 1.0,
    "slow_horizon_s": 15.0,
    "own_max_age_s": 0.5,
    "start_window_s": 30.0,                              # review MED-2: the pre-start kind-4 only this long after a
                                                         # station's START / RESUME (ACCEPT + the CONFIRM window)
    "tentative_ahead_m": 30.0,                           # review MED-1: a tentative range track this close ahead
    "tentative_hits": 2,                                 # ... with at least this many range hits (its window)
    "vessel_moving_mps": 0.2,                            # a vessel-class track faster than this = MOVING (15 m)
    "obstacle_moving_mps": 0.25,                         # HX5-r2 storm: any other track faster than this = MOVING
                                                         # (a kayak at 0.5-1 m/s estimated slower kept only 8 m)
    # HX7-fix (owner option (a), finding HX7-2): a range-confirmed MOVING track that does NOT give way = observed for
    # >= ny_min_obs_s, its speed not falling (>= ny_slow_frac x its highest speed over ny_window_s), its course
    # steady (<= ny_turn_deg), and its constant-velocity path meets the boat EVEN IF THE BOAT STOPS (stopping is no
    # refuge): then the planner steers clear inside the corridor if a path keeps the minimum distances, else STOP +
    # the "vessel not giving way - TAKE OVER" alarm (never a creep towards it)
    "ny_window_s": 4.0, "ny_min_obs_s": 2.0, "ny_slow_frac": 0.85, "ny_turn_deg": 15.0,
    "ny_hold_s": 10.0,                                   # review M3: the not-yielding state LATCHED this long after
                                                         # its last detection (a re-ID / U7 never clears it)
    # HX7-fix (finding HX7-1): the READY lease before the captain's START (STANDBY only): heading = the leg track,
    # cap = the most restrictive of the planner's cap, the route leg and ready_kn
    "ready_kn": 5.0,
}

# The explanation line (design 7.5): key -> (el, en); the same keys in groundstation/lang el.txt / en.txt (HP test).
WHY = {
    "harb.why_clear": ("ΕΛΕΥΘΕΡΗ ΔΙΑΔΡΟΜΗ", "ROUTE CLEAR"),
    "harb.why_avoid": ("ΑΠΟΦΥΓΗ ΕΜΠΟΔΙΟΥ", "AVOIDING AN OBSTACLE"),
    "harb.why_slow_obstacle": ("ΕΜΠΟΔΙΟ ΜΠΡΟΣΤΑ – επιβράδυνση", "OBSTACLE AHEAD - slowing"),
    "harb.why_camera_only": ("ΕΜΠΟΔΙΟ ΜΠΡΟΣΤΑ (κάμερα / AIS μόνο) – επιβράδυνση", "OBSTACLE AHEAD (camera / AIS only) - slowing"),
    "harb.why_narrow": ("ΣΤΕΝΟ ΠΕΡΑΣΜΑ – επιβράδυνση", "NARROW PASSAGE - slowing"),
    "harb.why_degraded": ("ΑΙΣΘΗΤΗΡΑΣ ΣΕ ΥΠΟΒΑΘΜΙΣΗ – επιβράδυνση", "SENSOR DEGRADED - slowing"),
    "harb.why_head_on": ("ΑΝΤΙΘΕΤΗ ΠΟΡΕΙΑ – ΣΤΡΟΦΗ ΔΕΞΙΑ", "HEAD-ON - TURNING TO STARBOARD"),
    "harb.why_giveway_stbd": ("ΔΙΝΩ ΔΡΟΜΟ ΣΕ ΣΚΑΦΟΣ ΔΕΞΙΑ", "GIVING WAY TO A VESSEL ON STARBOARD"),
    "harb.why_standon_port": ("ΣΚΑΦΟΣ ΑΡΙΣΤΕΡΑ – ΔΙΑΤΗΡΩ ΠΟΡΕΙΑ", "VESSEL ON PORT - STAND-ON"),
    "harb.why_overtaking": ("ΠΡΟΣΠΕΡΝΩ ΣΚΑΦΟΣ – κρατώ απόσταση", "OVERTAKING A VESSEL - keeping clear"),
    "harb.why_overtaken": ("ΜΕ ΠΡΟΣΠΕΡΝΑ ΣΚΑΦΟΣ – διατηρώ πορεία", "BEING OVERTAKEN - stand-on"),
    "harb.why_stop": ("ΑΒΕΒΑΙΟΤΗΤΑ – ΣΤΑΣΗ", "UNSURE - STOP"),
    "harb.why_ny_avoid": ("ΣΚΑΦΟΣ ΔΕΝ ΔΙΝΕΙ ΔΡΟΜΟ – ΑΠΟΜΑΚΡΥΝΣΗ ΣΤΟΝ ΔΙΑΔΡΟΜΟ",
                          "VESSEL NOT GIVING WAY - STEERING CLEAR IN THE CORRIDOR"),
    "harb.why_not_yielding": ("ΣΚΑΦΟΣ ΔΕΝ ΔΙΝΕΙ ΔΡΟΜΟ – ΣΤΑΣΗ – ΠΑΡΤΕ ΤΟΝ ΕΛΕΓΧΟ",
                              "VESSEL NOT GIVING WAY - STOP - TAKE OVER"),
}
# navreq alarm bits (app/link.c gw_navreq: 1 collision, 2 driver decision) - the EXISTING advisory path
ALM_COLLISION, ALM_DECISION = 1, 2


def why_text(key, detail=""):
    el, en = WHY[key]
    if detail:
        return "%s: %s" % (el, detail), "%s: %s" % (en, detail)
    return el, en


def wrap(a):
    return (a + 540.0) % 360.0 - 180.0


def _tentative(t):
    last = t.get("last") if isinstance(t.get("last"), dict) else {}
    return t.get("confirmed") is False and any(last.get(k) is not None for k in ("R", "L"))


def _vsd(tr):
    return colregs.vel_sd(tr) if not tr.get("static") else 0.0


class Planner:
    def __init__(self, cfg=None):
        c = dict(DEFAULTS)
        c.update(cfg or {})
        self.c = c
        self.prev = None                # (offset, speed kn) of the last decision (change penalty)
        self.last_t = None              # the last plan() time (U4: a late cycle)
        self.u4_until = 0.0
        self.since = {}                 # slow-then-stop timers: "U6" / "U8" -> since when
        self.cycles = 0
        self.hist = {}                  # HX7-fix: track id -> [(t, speed m/s, course deg)] (the not-yielding judgement)
        self.ny_t = None                # review M3: the last time a vessel was judged not yielding
        self.ny_ids = []

    def reset(self):
        self.prev = None
        self.last_t = None
        self.since = {}
        self.hist = {}
        self.ny_t = None
        self.ny_ids = []

    # --- HX7-fix: the not-yielding judgement ------------------------------------------------------------------
    def _observe(self, now, tracks):
        """Keep each track's recent (time, speed, course) - plan() calls it every cycle with `now`."""
        w = float(self.c["ny_window_s"])
        seen = set()
        for t in tracks:
            i = t.get("id")
            if i is None:
                continue
            seen.add(i)
            h = self.hist.setdefault(i, [])
            if h and now < h[-1][0]:
                h.clear()                                   # a clock step back: start again
            h.append((now, float(t.get("speed") or 0.0), float(t.get("course") or 0.0)))
            while h and now - h[0][0] > w:
                h.pop(0)
        for i in [k for k in self.hist if k not in seen and now - self.hist[k][-1][0] > w]:
            del self.hist[i]                                # (a track missed for a frame keeps its history)

    def _not_slowing(self, t, now):
        """True = observed long enough, the speed not falling, the course steady (it keeps coming as it is)."""
        c = self.c
        h = self.hist.get(t.get("id")) or []
        if len(h) < 3 or now - h[0][0] < float(c["ny_min_obs_s"]) - 1e-6:
            return False
        sp_now, cs_now = h[-1][1], h[-1][2]
        if sp_now < float(c["ny_slow_frac"]) * max(q[1] for q in h):
            return False                                    # it slows down: it may be giving way
        return all(abs(wrap(q[2] - cs_now)) <= float(c["ny_turn_deg"]) for q in h)

    # --- the unsure list (design 7.4) --------------------------------------------------------------------------
    def _unsure(self, now, inp):
        c = self.c
        stop, slow = [], []
        for b in inp.get("block") or []:
            stop.append(b)
        own = inp.get("own")
        if not own or not own.get("fresh") or own.get("heading") is None or own.get("x") is None:
            stop.append("U12 own position / heading not fresh")
        elif not own.get("gps_ok") or own.get("unc") is None or float(own["unc"]) > float(c["unc_m"]):
            stop.append("U3 GPS not trusted / uncertainty > %.0f m" % float(c["unc_m"]))
        if inp.get("leg") is None:
            stop.append("no checked harbour route leg")
        # round 2 MED-2: the planner judges the sensor health ITSELF too (defence in depth beside the tracker's U1 /
        # U2): neither radar nor LiDAR OK = STOP, one not OK = slow to degraded_kn
        h = inp.get("health") if isinstance(inp.get("health"), dict) else {}
        bad = [k for k in ("radar", "lidar") if h.get(k) != "OK"]
        if len(bad) == 2:
            stop.append("U1 radar and LiDAR not OK (%s / %s)" % (h.get("radar"), h.get("lidar")))
        elif bad:
            slow.append("U2 %s %s" % (bad[0], h.get(bad[0])))
        f = inp.get("flags")
        if not isinstance(f, dict):
            stop.append("tracker flags missing")
            f = {}
        conflict = False
        for u in f.get("unsure") or []:
            u = str(u)
            if u.startswith("U2 ") or u.startswith("U9 "):
                slow.append(u)
            elif u.startswith("U6 "):
                conflict = True
            else:
                stop.append(u)                          # U1 / U10 / U11 / stale / no pose / error / overload ... / unknown
        ca = f.get("cycle_age_s")
        if ca is None or ca > 0.5:
            stop.append("tracker cycle old")
        if self.last_t is not None and now - self.last_t > float(c["late_s"]):
            self.u4_until = now + float(c["late_hold_s"])
        if now < self.u4_until:
            stop.append("U4 planner cycle late")
        if conflict:
            t0 = self.since.setdefault("U6", now)
            (stop if now - t0 >= float(c["conflict_stop_s"]) else slow).append("U6 conflicting tracks")
        else:
            self.since.pop("U6", None)
        return stop, slow

    # --- one decision -------------------------------------------------------------------------------------------
    def plan(self, now, inp, deadline=None):
        """inp = {own {x, y, heading, sog_kn, fresh, gps_ok, unc}, leg ((ax, ay), (bx, by)) | None, corr_m, route_kn,
        tracks [...], flags {...}, static StaticMap | None, health {radar, lidar}, block [reasons]} -> decision dict."""
        return self._latch(now, self._plan_once(now, inp, deadline))

    def _latch(self, now, dec):
        """Review M3: the not-yielding state stays raised for ny_hold_s after its last detection, whatever the cycle's
        own reason (U7 inside 8 m, a re-ID, the history rebuilding): on a STOP the loud state (why, alarm COLLISION |
        DECISION, the vessel named) stays; while steering clear COLLISION stays. Only ever ADDS a state / an alarm."""
        if dec.get("not_yielding"):
            self.ny_t, self.ny_ids = now, list(dec["not_yielding"])
        elif self.ny_t is not None and 0.0 <= now - self.ny_t <= float(self.c["ny_hold_s"]):
            dec["not_yielding"] = list(self.ny_ids)
            if dec.get("stop"):
                dec["alarm"] = int(dec.get("alarm") or 0) | ALM_COLLISION | ALM_DECISION
                if dec.get("why") != "harb.why_not_yielding":
                    dec["detail"] = "U13 (held) %s" % (dec.get("detail") or "")
                    dec["why"] = "harb.why_not_yielding"
            else:
                dec["alarm"] = int(dec.get("alarm") or 0) | ALM_COLLISION
        elif self.ny_t is not None:
            self.ny_t, self.ny_ids = None, []
        return dec

    def _plan_once(self, now, inp, deadline=None):
        t_start = time.perf_counter()
        c = self.c
        self.cycles += 1
        stop, slow = self._unsure(now, inp)
        self.last_t = now
        own = inp.get("own") or {}
        dec = {"t": now, "stop": True, "heading": None, "cap_kn": 0.0, "hflags": 0, "nearest_m": 255, "n_tracks": 0,
               "reasons": [], "slow": [], "colregs": [], "turn": False, "turn_for": [], "blockers": [], "offset": None,
               "why": "harb.why_stop", "detail": "", "alarm": 0, "not_yielding": []}
        tracks = [t for t in (inp.get("tracks") or []) if isinstance(t, dict) and t.get("x") is not None]
        self._observe(now, tracks)
        # a TENTATIVE range track (radar / LiDAR hits, not confirmed yet: <= 1 s LiDAR, <= 4 s radar; spray, sea
        # clutter, a new object) is not planned around until it is confirmed - else every spray return would latch a
        # HOLD. Camera-only / AIS-only tracks (never "confirmed": no range hits) ARE obstacles (slow / STOP only).
        tent = [t for t in tracks if _tentative(t)]
        dec["tentative"] = len(tent)
        tracks = [t for t in tracks if not _tentative(t)]
        # review MED-1: a tentative range return is never ignored entirely - one within tentative_ahead_m ahead in the
        # corridor band (from the bow on; a repeated return: tentative_hits) SLOWS the boat to degraded_kn (never a turn,
        # never a STOP by itself: a spray / clutter return would latch a HOLD); confirmed, it is planned around
        own0 = inp.get("own") or {}
        leg0 = inp.get("leg")
        if tent and own0.get("x") is not None and leg0 is not None:
            (lax, lay), (lbx, lby) = leg0
            ll = math.hypot(lbx - lax, lby - lay) or 1.0
            ux0, uy0 = (lbx - lax) / ll, (lby - lay) / ll
            cw = float(inp.get("corr_m") or c["corr_m"])
            for t in tent:
                if int(t.get("hits_R") or 0) + int(t.get("hits_L") or 0) < int(c["tentative_hits"]):
                    continue                        # a single return (spray / sea clutter) until it repeats
                dx, dy = t["x"] - float(own0["x"]), t["y"] - float(own0["y"])
                along, lat = dx * ux0 + dy * uy0, abs(dx * uy0 - dy * ux0)
                if 0.5 * float(c["own_len_m"]) <= along <= float(c["tentative_ahead_m"]) and lat <= cw:
                    slow.append("tentative range return %.0f m ahead - slowing" % max(0.0, along))
                    break
        degraded = [k for k in ("radar", "lidar") if (inp.get("health") or {}).get(k) != "OK"]
        if degraded:
            dec["hflags"] |= 0x02
        if not stop:
            ox, oy = float(own["x"]), float(own["y"])
            near = None
            n100 = 0
            for t in tracks:
                d = math.hypot(t["x"] - ox, t["y"] - oy) - 0.5 * max(t.get("len_m") or 0.0, t.get("beam_m") or 0.0)
                if near is None or d < near:
                    near = d
                if d <= 100.0:
                    n100 += 1
            dec["nearest_m"] = 255 if near is None else int(max(0, min(250, round(near))))
            dec["n_tracks"] = min(255, n100)
            if near is not None and near < float(c["min_static_m"]):
                stop.append("U7 an object inside %.0f m (%.1f m)" % (float(c["min_static_m"]), near))
        if not stop:
            try:
                return self._plan(now, inp, dec, tracks, slow, degraded, deadline, t_start)
            except _Late:
                stop.append("U4 planner over its time budget")
            except _Stop as e:
                dec.update({"stop": True, "heading": None, "cap_kn": 0.0, "turn": False, "turn_for": []})
                dec["reasons"] = [e.why]
                dec["slow"] = slow
                dec["why"] = e.key
                dec["detail"] = e.why
                dec["alarm"] = e.alarm
                dec["not_yielding"] = list(e.ny)
                self.prev = None
                return dec
        dec["reasons"] = stop
        dec["slow"] = slow
        dec["detail"] = stop[0] if stop else ""
        self.prev = None
        return dec

    def _plan(self, now, inp, dec, tracks, slow, degraded, deadline, t_start):
        c = self.c
        own = inp["own"]
        (ax, ay), (bx, by) = inp["leg"]
        L = math.hypot(bx - ax, by - ay)
        psi = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
        ux, uy = math.sin(math.radians(psi)), math.cos(math.radians(psi))
        nx, ny = uy, -ux                                 # starboard normal: xte + = starboard of the line
        ox, oy = float(own["x"]), float(own["y"])
        s0 = (ox - ax) * ux + (oy - ay) * uy
        e0 = (ox - ax) * nx + (oy - ay) * ny
        corr = float(inp.get("corr_m") or c["corr_m"])
        usable = corr - 0.5 * float(c["own_beam_m"]) - 1.0 - float(c["edge_margin_m"])
        if usable < 1.0:
            raise _Stop("corridor too narrow for the boat")
        cap = min(float(inp.get("route_kn") or 0.0), float(c["max_kn"]))
        why, detail = "harb.why_clear", ""
        if slow:
            cap = min(cap, float(c["degraded_kn"]))
            why = "harb.why_degraded"
            detail = slow[0]
        hdg = float(own["heading"])
        v0 = max(0.0, float(own.get("sog_kn") or 0.0)) * KN
        own_c = {"x": ox, "y": oy, "hdg": hdg, "vx": v0 * math.sin(math.radians(hdg)),
                 "vy": v0 * math.cos(math.radians(hdg)), "speed": v0}
        encs, amb = colregs.encounters(own_c, tracks, c)
        dec["colregs"] = encs
        if amb:
            t0 = self.since.setdefault("U8", now)
            if now - t0 >= float(c["ambig_stop_s"]):
                raise _Stop("U8 ambiguous COLREGs encounter")
            cap = min(cap, float(c["degraded_kn"]))
            dec["slow"].append("U8 ambiguous COLREGs encounter")
            why = "harb.why_slow_obstacle"
        else:
            self.since.pop("U8", None)
        kinds = {e["cls"] for e in encs}
        no_port = bool(kinds & set(colregs.NO_PORT))
        if kinds & set(colregs.GIVE_WAY):
            dec["hflags"] |= 0x04
        # the static points near the corridor ahead (static map cells + static tracks), in leg coordinates (s, e)
        reach = float(c["horizon_s"]) * max(cap * KN, v0) + 20.0
        # round 2 MED-1: the FULL horizon, also past the leg's end (the line extended); past the end + corr while the
        # AURIX has not switched to the next leg = STOP (never "clear" on a horizon cut to nothing)
        if s0 > L + corr:
            raise _Stop("past the leg's end (%.0f m) - the next leg not active" % (s0 - L))
        s_end = s0 + reach
        stat = self._static_points(inp.get("static"), ax, ay, ux, uy, nx, ny, s0 - 10.0, s_end,
                                   corr + float(c["min_static_m"]) + 2.0, now)
        movers = []
        for t in tracks:
            mv = float(c["vessel_moving_mps"] if t.get("cls") == "vessel" else c["obstacle_moving_mps"])
            if t.get("static") or float(t.get("speed") or 0.0) <= mv:
                hs = 0.5 * max(t.get("len_m") or 0.0, t.get("beam_m") or 0.0)
                stat.append(((t["x"] - ax) * ux + (t["y"] - ay) * uy, (t["x"] - ax) * nx + (t["y"] - ay) * ny,
                             hs + 2.0 * float(t.get("sd_m") or 0.0), t))
            else:
                movers.append(t)
        sidx = {"big": []}
        for p in stat:
            if p[2] > 6.0:
                sidx["big"].append(p)                   # a large static track: checked at every step
            else:
                sidx.setdefault(int(p[0] // 4.0), []).append(p)
        if not any(not t.get("static") and float(t.get("speed") or 0.0) > float(c["moving_mps"]) for t in tracks):
            dec["hflags"] |= 0x08
        # narrow: a static object within the corridor + min_static of the line just ahead = slower (W144)
        narrow = any(0.0 <= s - s0 <= float(c["narrow_ahead_m"]) and abs(e) - r <= corr + float(c["min_static_m"])
                     for s, e, r, _t in stat)
        if narrow and cap > float(c["narrow_kn"]):
            cap = float(c["narrow_kn"])
            if why == "harb.why_clear":
                why = "harb.why_narrow"
        if cap < float(c["min_move_kn"]):
            raise _Stop("speed cap %.1f kn below the in-gear idle speed" % cap)
        speeds = [cap * f for f in c["speeds"] if cap * f >= float(c["min_move_kn"]) - 1e-9]
        e_straight = max(-usable, min(usable, e0))
        step = float(c["offset_step_m"])
        offs = sorted({round(-usable + i * step, 2) for i in range(int(2 * usable / step) + 1)} | {round(usable, 2)} |
                      {round(e_straight, 2)} | {0.0})
        look = float(c["lookahead_m"])
        tol = math.tan(math.radians(float(c["turn_deg"]))) * look

        def is_turn(o):
            return abs(o - e0) > tol

        # HX7-fix (finding HX7-2): a range-confirmed moving track that does not give way and whose path meets the
        # boat even if the boat STOPS - standing still is no refuge from it; with one, never a speed increase (review
        # M5: no candidate faster than the boat goes now / the last command)
        threats = self._threats(now, movers, s0, e0, psi, usable, hdg, v0, ax, ay, ux, uy, nx, ny, s_end)
        v_lim = max(v0 / KN, self.prev[1] if self.prev is not None else 0.0)
        cand = []
        straight_block = set()
        for v_kn in speeds:
            for o in offs:
                if deadline is not None and time.perf_counter() > deadline:
                    raise _Late()
                if no_port and o < e0 - tol:
                    continue                            # COLREGs: never a port turn in this encounter
                ok, clear, blk = self._sim(o, v_kn, s0, e0, psi, usable, hdg, v0, movers, sidx, ax, ay, ux, uy, nx,
                                           ny, s_end)
                if o == round(e_straight, 2):
                    straight_block |= blk
                cand.append((o, v_kn, ok, clear, blk))
        trk = {t.get("id"): t for t in tracks}
        bad_turn = []
        for i in straight_block:
            if i == "static-map":
                continue        # chart cells + LiDAR cells laid only by steer_ok LiDAR tracks = range / chart confirmed
            t = trk.get(i)
            if t is None or not t.get("steer_ok"):
                bad_turn.append(i)
            elif degraded and not self._seen_by_ok(t, degraded):
                bad_turn.append(i)
        turns_ok = not bad_turn
        best = None
        narrow_w = 2.0 * corr < 3.0 * float(c["min_static_m"])
        for o, v_kn, ok, clear, blk in cand:
            if not ok:
                continue
            if threats and v_kn > v_lim + 1e-6:
                continue                                # (review M5) never faster with a vessel not giving way
            if is_turn(o) and not turns_ok:
                continue
            cost = 10.0 * abs(o) / usable + 8.0 * (1.0 - v_kn / cap)
            cost += 6.0 * abs(o - e0) / usable * (3.0 if narrow_w else 1.0)
            if self.prev is not None:
                cost += 0.5 * abs(o - self.prev[0]) / step + (0.5 if abs(v_kn - self.prev[1]) > 1e-6 else 0.0)
            cost -= 0.05 * min(clear, 10.0)
            if best is None or cost < best[0]:
                best = (cost, o, v_kn, blk)
        dec["blockers"] = sorted(str(x) for x in straight_block)
        dec["not_yielding"] = sorted(str(t.get("id")) for t in threats)
        if threats:
            dec["alarm"] |= ALM_COLLISION
        esc = None
        if best is None and threats and not amb and turns_ok:
            # steer clear FIRST: a path inside the corridor that keeps the minimum distances from every moving track
            # until the threat has passed (and from every static object over the full horizon), starboard first (a
            # port turn only where COLREGs allows one: no_port), at the highest speed that keeps it feasible (never
            # above the cap: the speeds are the cap's fractions)
            esc = self._escape(threats, speeds, offs, no_port, e0, tol, s0, psi, usable, hdg, v0, movers, sidx, own_c,
                               ax, ay, ux, uy, nx, ny, s_end)
            if esc is not None:
                best = (0.0, esc[0], esc[1], set())
        if best is None and not amb and not threats:
            # slow, then STOP (design 7.4): nothing is feasible over the full horizon - the slowest STRAIGHT candidate
            # (no turn: M0-3 holds for any obstacle) judged over slow_horizon_s keeps the boat creeping on while the
            # danger is still far; once that fails too = STOP (the HOLD is latched on the AURIX: only when needed)
            v_min = min(speeds)
            ok, clear, blk = self._sim(round(e_straight, 2), v_min, s0, e0, psi, usable, hdg, v0, movers, sidx, ax,
                                       ay, ux, uy, nx, ny, s_end, H=float(c["slow_horizon_s"]))
            if ok and not is_turn(round(e_straight, 2)):
                best = (0.0, round(e_straight, 2), v_min, blk)
                dec["slow"].append("nothing free over %.0f s: creeping straight" % float(c["horizon_s"]))
        if best is None and threats:
            # ... STOP second: no path keeps the minimum distances - STOP at once (never a creep towards it) and the
            # loud state: the existing navreq COLLISION RISK + DRIVER DECISION advisories + the station's WARNING
            # «ΣΚΑΦΟΣ ΔΕΝ ΔΙΝΕΙ ΔΡΟΜΟ – ΠΑΡΤΕ ΤΟΝ ΕΛΕΓΧΟ» (the captain takes over: a stopped boat is NOT safe from it)
            raise _Stop("U13 a vessel not giving way (%s) - no free path inside the corridor - STOP, TAKE OVER"
                        % ", ".join(dec["not_yielding"]), "harb.why_not_yielding", ALM_COLLISION | ALM_DECISION,
                        dec["not_yielding"])
        if best is None:
            for k, w in ((colregs.HEAD_ON, "harb.why_head_on"), (colregs.CROSS_STBD, "harb.why_giveway_stbd"),
                         (colregs.OVERTAKING, "harb.why_overtaking"), (colregs.CROSS_PORT, "harb.why_standon_port")):
                if k in kinds and not amb:
                    raise _Stop("U5 no free path inside the corridor - STOP", w)
            if bad_turn:
                raise _Stop("U5 no free path (an obstacle seen by camera / AIS only blocks the line)",
                            "harb.why_camera_only")
            raise _Stop("U5 no free path inside the corridor")
        _cost, o, v_kn, _blk = best
        off = math.degrees(math.atan((o - e0) / look))
        off = max(-float(c["max_dev_deg"]), min(float(c["max_dev_deg"]), off))
        dec.update({"stop": False, "heading": round((psi + off) % 360.0, 2), "cap_kn": round(v_kn, 2),
                    "offset": o, "xte": round(e0, 2), "leg_track": round(psi, 2)})
        dec["turn"] = is_turn(o)
        dec["turn_for"] = dec["blockers"] if dec["turn"] else []
        if bad_turn and v_kn < cap - 1e-6:
            dec["hflags"] |= 0x10
            why = "harb.why_camera_only"
        elif dec["turn"] and straight_block:
            why = "harb.why_avoid"
        elif straight_block and v_kn < cap - 1e-6:
            why = "harb.why_slow_obstacle"
        for k, w in ((colregs.HEAD_ON, "harb.why_head_on"), (colregs.CROSS_STBD, "harb.why_giveway_stbd"),
                     (colregs.OVERTAKING, "harb.why_overtaking"), (colregs.CROSS_PORT, "harb.why_standon_port"),
                     (colregs.OVERTAKEN, "harb.why_overtaken")):
            if k in kinds and not amb:
                why = w
                break
        if threats and (esc is not None or dec["turn"]):
            why = "harb.why_ny_avoid"                       # steering clear of a vessel that does not give way
        dec["why"] = why
        dec["detail"] = detail or ("%.1f kn" % v_kn)
        dec["ms"] = round((time.perf_counter() - t_start) * 1000.0, 1)
        self.prev = (o, v_kn)
        return dec

    def _threats(self, now, movers, s0, e0, psi, usable, hdg, v0, ax, ay, ux, uy, nx, ny, s_end):
        """HX7-fix: the movers that do NOT give way: range-confirmed (steer_ok), not slowing / turning (_not_slowing),
        and their constant-velocity path comes inside the minimum moving distance of the boat even if the boat STOPS
        here and now (the boat simulated STANDING where it is: only the other vessel's own motion closes the gap - a
        slow boat ahead that we would coast up to is not a vessel that fails to give way; the planner's normal
        candidates keep its distance)."""
        out = []
        for t in movers:
            if not t.get("steer_ok") or not self._not_slowing(t, now):
                continue
            ok, _cl, _b = self._sim(round(e0, 2), 0.0, s0, e0, psi, usable + 1e3, hdg, 0.0, [t], {"big": []}, ax, ay,
                                    ux, uy, nx, ny, s_end)
            if not ok:
                out.append(t)
        return out

    def _escape(self, threats, speeds, offs, no_port, e0, tol, s0, psi, usable, hdg, v0, movers, sidx, own_c, ax, ay,
                ux, uy, nx, ny, s_end):
        """HX7-fix: the escape path -> (offset, speed kn) or None. Judged against EVERY moving track at the full
        minimum distance until the threats have passed (their TCPA + 5 s, at least slow_horizon_s, at most the
        horizon) and against every static object over the full horizon; inside the usable corridor; starboard (or
        straight) first, a port offset only when COLREGs allows one; then the highest speed."""
        c = self.c
        tc = 0.0
        for t in threats:
            _d, tt = colregs.cpa(float(t["x"]) - own_c["x"], float(t["y"]) - own_c["y"],
                                 float(t["vx"]) - own_c["vx"], float(t["vy"]) - own_c["vy"])
            tc = max(tc, tt)
        # review M4: every mover over the FULL horizon (a slower escape meets the threat later than the TCPA from the
        # current velocity); review M5: never faster than the boat goes now / the last command (never a speed increase)
        H = float(c["horizon_s"])
        v_lim = max(own_c["speed"] / KN, self.prev[1] if self.prev is not None else 0.0)
        found = []
        for v_kn in speeds:
            if v_kn > v_lim + 1e-6:
                continue
            for o in offs:
                port = o < e0 - tol
                if port and no_port:
                    continue
                ok_m, cl, _b = self._sim(o, v_kn, s0, e0, psi, usable, hdg, v0, movers, {"big": []}, ax, ay, ux, uy,
                                         nx, ny, s_end, H=H)
                if not ok_m:
                    continue
                ok_s, _c2, _b2 = self._sim(o, v_kn, s0, e0, psi, usable, hdg, v0, [], sidx, ax, ay, ux, uy, nx, ny,
                                           s_end)
                if ok_s:
                    found.append((1 if port else 0, -v_kn, -cl, abs(o - e0), o, v_kn))
        if not found:
            return None
        found.sort()
        return found[0][4], found[0][5]

    @staticmethod
    def _seen_by_ok(t, degraded):
        """U2: a turn only on a track seen by a sensor that is still OK."""
        ok_src = {"radar": "R", "lidar": "L"}
        for k, s in ok_src.items():
            if k not in degraded and s in (t.get("src") or ""):
                return True
        return False

    def _static_points(self, smap, ax, ay, ux, uy, nx, ny, s_a, s_b, emax, now):
        """Occupied static-map cells (chart + LiDAR) in the band ahead, sampled every 1 m, as (s, e, r, None)."""
        out = []
        if smap is None:
            return out
        s = s_a
        while s <= s_b:
            e = -emax
            while e <= emax:
                x = ax + ux * s + nx * e
                y = ay + uy * s + ny * e
                try:
                    hit = self._occ(smap, x, y)
                except Exception:
                    hit = True                              # an unreadable map = occupied (fail closed)
                if hit:
                    out.append((s, e, 0.75, None))
                e += 1.0
            s += 1.0
        return out

    @staticmethod
    def _occ(smap, x, y):
        ci, cj = smap.ij(x, y)
        tt = getattr(smap, "t_last", 0.0)
        for a in (0, 1, -1):
            for b in (0, 1, -1):
                k = (ci + a, cj + b)
                if k in smap.chart:
                    return True
                if k in smap.grid and smap._w(k, tt) >= smap.occ_w:
                    return True
        return False

    def _sim(self, o, v_kn, s0, e0, psi, usable, hdg, v0, movers, sidx, ax, ay, ux, uy, nx, ny, s_end, H=None):
        """Simulate one candidate -> (feasible, min clearance margin, blocking track ids)."""
        c = self.c
        dt = float(c["dt_s"])
        H = float(c["horizon_s"]) if H is None else float(H)
        look = float(c["lookahead_m"])
        tau = float(c["speed_tau_s"])
        vt = v_kn * KN
        x = ax + ux * s0 + nx * e0
        y = ay + uy * s0 + ny * e0
        h = hdg
        v = v0
        half = 0.5 * float(c["own_len_m"])
        mm, ms = float(c["min_moving_m"]), float(c["min_static_m"])
        ok = True
        blk = set()
        clear = 1e9
        t = 0.0
        e_prev = e0
        while t < H - 1e-9:
            t += dt
            v = vt + (v - vt) * math.exp(-dt / tau)
            s = (x - ax) * ux + (y - ay) * uy
            e = (x - ax) * nx + (y - ay) * ny
            want = (psi + math.degrees(math.atan((o - e) / look))) % 360.0
            rate = min(float(c["rot_dps"]), float(c["turn_dps_per_kn"]) * v / KN)
            dh = wrap(want - h)
            h = (h + max(-rate * dt, min(rate * dt, dh))) % 360.0
            x += v * dt * math.sin(math.radians(h))
            y += v * dt * math.cos(math.radians(h))
            s = (x - ax) * ux + (y - ay) * uy
            e = (x - ax) * nx + (y - ay) * ny
            if abs(e) > usable + 0.5 and abs(e) > abs(e_prev) - 1e-6:
                return False, -1.0, blk                 # leaves the usable corridor
            e_prev = e
            for tr in movers:
                tu = min(t, float(c["unc_t_s"]))
                sig = math.sqrt(float(tr.get("sd_m") or 0.0) ** 2 + (_vsd(tr) * tu) ** 2)
                r = 0.5 * max(tr.get("len_m") or 0.0, tr.get("beam_m") or 0.0) + 2.0 * sig
                px = float(tr["x"]) + float(tr["vx"]) * t
                py = float(tr["y"]) + float(tr["vy"]) * t
                m = math.hypot(px - x, py - y) - r - half - mm
                if m < clear:
                    clear = m
                if m < 0.0:
                    ok = False
                    blk.add(tr.get("id"))
            b0 = int(s // 4.0)
            near = list(sidx["big"])
            for bk in range(b0 - 6, b0 + 7):
                near.extend(sidx.get(bk, ()))
            for ss, ee, r, tr in near:
                if abs(ss - s) > ms + r + half + 1.0:
                    continue
                m = math.hypot(ss - s, ee - e) - r - half - ms
                if m < clear:
                    clear = m
                if m < 0.0:
                    ok = False
                    if tr is not None:
                        blk.add(tr.get("id"))
                    else:
                        blk.add("static-map")
        return ok, clear, blk


class _Late(Exception):
    pass


class _Stop(Exception):
    def __init__(self, why, key="harb.why_stop", alarm=0, ny=()):
        Exception.__init__(self, why)
        self.why = why
        self.key = key
        self.alarm = alarm
        self.ny = tuple(ny)


# --- the gateway glue ------------------------------------------------------------------------------------------------
RUNNING = (1, 2, 3)            # HARBOUR_STAT RUN / SLOW / HOLD


class HarbourPlanner:
    """In the gateway (one object): HARBOUR_STAT in, the kind-4 navreq out through Gateway.nav_compose (the ONE
    output - nothing here sends). Runs only while the AURIX's own fresh HARBOUR_STAT says RUN / SLOW / HOLD (the
    captain started it: START + CONFIRM) or, before the start, a gateway-checked harbour route is stored (the entry
    needs a fresh kind-4 request without STOP)."""

    def __init__(self, gw, cfg=None):
        self.gw = gw
        c = dict(cfg or {})
        self.cfg = c
        self.planner = Planner(c.get("planner") if isinstance(c.get("planner"), dict) else None)
        self.stat = None
        self.stat_rx = 0.0              # monotonic
        self.dec = None
        self.dec_mono = 0.0
        self.last_key = None
        self.errors = 0
        self.gate = None                # the sim gate: simulated sensors + a real AURIX link
        self.was_active = False
        self.decisions = 0
        self.stops = 0
        self.start_mono = None          # review MED-2: the last station START / RESUME (the pre-start window)

    def note_start(self):
        self.start_mono = time.monotonic()

    # --- inputs -----------------------------------------------------------------------------------------------
    def on_stat(self, d):
        self.stat = d
        self.stat_rx = time.monotonic()

    def stat_fresh(self):
        return self.stat is not None and time.monotonic() - self.stat_rx <= float(
            self.planner.c["stat_max_age_s"])

    def active(self, o=None):
        if not getattr(self.gw.harbour, "enabled", False) or not self.stat_fresh():
            return False
        st = self.stat.get("harb_state")
        if st in RUNNING:
            return True
        o = o if o is not None else self.gw.own()
        # waiting for the captain's START: only in the modes whose navreq does not matter (STANDBY, MANUAL, CREWED
        # HDG LOCK) - never in UNCREWED HDG LOCK / REMOTE / MANEUVER / AUTO / DP, where the kind 1 / 2 lease is
        # needed (a kind-4 request is inert there on the AURIX = their lease would lapse)
        # round 2 MED-3 / LOW-5: only in STANDBY (no kind 1 / 2 cap acts there) and only with FRESH telemetry
        # HX7-fix (finding HX7-1): no START needed first any more - the READY lease (ready_frame: the leg track, the
        # most restrictive cap, inert on the AURIX outside harbour autonomy) is sent while a checked harbour route is
        # loaded, the boat is in STANDBY (not stopping) with fresh telemetry and a gateway station holds HELM - so
        # entry_ok can become true and the captain's first START passes. Any of them missing = no kind 4 at once.
        return bool(self.stat.get("harb_route")) and st == 0 and self.ready_ok(o)

    def ready_ok(self, o):
        """The READY lease's conditions besides the AURIX's HARBOUR_STAT (route loaded, state OFF)."""
        if not (bool(o.get("fresh")) and o.get("mode") == MODE_STANDBY):
            return False
        try:
            d = self.gw.telem_fresh() or {}
        except Exception:
            return False
        if not d or d.get("stopping") is not False:
            return False                                # not yet idle + neutral after the STANDBY / not reported (M1)
        sts = getattr(self.gw, "stations", None)
        try:
            helm = sts.helm_station()[0] if sts is not None and hasattr(sts, "helm_station") else None
        except Exception:
            helm = None
        return helm is not None                         # nobody holds HELM = nobody can start: no lease

    def real_link(self):
        a = str((self.gw.cfg or {}).get("aurix", ""))
        return not (a.startswith("tcp:127.0.0.1") or a.startswith("tcp:localhost") or a.startswith("tcp:[::1]"))

    def _inputs(self, o):
        from . import simworld
        gw = self.gw
        block = []
        hs = gw.harbour
        if getattr(hs, "sim", False) and self.real_link():
            block.append("simulated harbour sensors with a real AURIX link - the planner is refused")
            if self.gate is None:
                self.gate = block[-1]
                gw.audit("harbour", what="refused", why=self.gate)
        d = gw.telem_fresh() or {}
        tr = hs.tracker
        # U12: gw.own() is "fresh" up to 3 s (TELEM_MAX_AGE_S) - far too old at 5 kn in a harbour: own_max_age_s
        age = time.time() - float(getattr(gw, "telem_rx", 0.0) or 0.0)
        own = {"fresh": bool(o.get("fresh")) and o.get("lat") is not None and
               0.0 <= age <= float(self.planner.c["own_max_age_s"]), "heading": o.get("heading"),
               "sog_kn": o.get("sog"), "unc": o.get("unc"), "gps_ok": d.get("pos_source") == 1}
        if own["fresh"] and tr.origin is not None:
            own["x"], own["y"] = simworld.ll_to_xy(tr.origin[0], tr.origin[1], o["lat"], o["lon"])
        else:
            own["x"] = own["y"] = None
        leg, route_kn = None, None
        wps = getattr(gw, "prog_wps", None) or []
        kns = getattr(gw, "prog_kn", None) or []
        st = self.stat or {}
        if not st.get("harb_route"):
            block.append("no gateway-checked harbour route on the boat")
        elif not wps or len(wps) < 2 or not (d.get("prog_gw") and d.get("prog_n") == len(wps)) or \
                len(kns) != len(wps):
            block.append("the boat's route is not the one the gateway checked")
        elif tr.origin is None:
            block.append("no tracker frame yet")
        else:
            k = d.get("auto_wp") if st.get("harb_state") in RUNNING else 1
            k = 1 if not isinstance(k, int) or k < 1 else min(k, len(wps) - 1)
            a = simworld.ll_to_xy(tr.origin[0], tr.origin[1], wps[k - 1][0], wps[k - 1][1])
            b = simworld.ll_to_xy(tr.origin[0], tr.origin[1], wps[k][0], wps[k][1])
            leg = (a, b)
            route_kn = float(kns[k])
            if not (route_kn > 0.0) or math.isnan(route_kn):
                block.append("the leg has no speed")
                leg = None
        corr = st.get("harb_corr_m") if st.get("harb_state") in RUNNING and st.get("harb_corr_m") else None
        return {"own": own, "leg": leg, "corr_m": corr, "route_kn": route_kn, "tracks": hs.tracks(),
                "flags": hs.tracker_flags(), "static": hs.static,
                "health": {k: hs.health(k)[0] if hs.procs[k].c.get("enabled") else "OFF" for k in ("radar", "lidar")},
                "block": block}

    def decide(self, o=None):
        """One planner decision (at most one per 0.15 s; the gateway thread)."""
        o = o if o is not None else self.gw.own()
        now = time.monotonic()
        if self.dec is not None and now - self.dec_mono < 0.15:
            return self.dec
        if not self.was_active:
            self.planner.reset()
        self.was_active = True
        try:
            inp = self._inputs(o)
            dl = time.perf_counter() + float(self.planner.c["budget_ms"]) / 1000.0
            dec = self.planner.plan(now, inp, deadline=dl)
        except Exception:
            self.errors += 1
            import logging
            logging.getLogger("gateway").exception("harbour planner")
            dec = {"t": now, "stop": True, "heading": None, "cap_kn": 0.0, "hflags": 0, "nearest_m": 255,
                   "n_tracks": 0, "reasons": ["U4 planner error"], "slow": [], "colregs": [], "turn": False,
                   "turn_for": [], "blockers": [], "why": "harb.why_stop", "detail": "U4 planner error"}
        el, en = why_text(dec["why"], dec.get("detail", ""))
        dec["text_el"], dec["text_en"] = el, en
        self.dec, self.dec_mono = dec, now
        self.decisions += 1
        if dec["stop"]:
            self.stops += 1
        key = (dec["stop"], dec["why"], tuple(dec.get("reasons") or ()), dec["cap_kn"],
               None if dec["heading"] is None else round(dec["heading"] / 5.0))
        if key != self.last_key:
            self.last_key = key
            self.gw.audit("harbour", what="stop" if dec["stop"] else "decision", why=dec["why"], text=en,
                          reasons=dec.get("reasons"), slow=dec.get("slow"), cap_kn=dec["cap_kn"],
                          heading=dec["heading"], turn=dec.get("turn"), turn_for=dec.get("turn_for"),
                          colregs=[e["cls"] for e in dec.get("colregs") or []])
        return dec

    def inactive(self):
        self.was_active = False
        self.dec = None

    def fold(self, req, o):
        """Gateway.nav_compose: while active, EVERY cap of `req` (shallow / collision / vision) folds into ONE kind-4
        request (the lowest cap keeps its reason; hflags b5 shallow / b6 collision / b7 vision). A folded cap below
        the in-gear idle speed = STOP (HX2 note). The collision avoider's heading is NOT used (the planner steers)."""
        if not self.active(o):
            self.inactive()
            return req
        dec = self.decide(o)
        hf = int(dec["hflags"])
        reason = int(req.get("reason") or 0)
        if reason & 2:
            hf |= 0x20
        if reason & 1:
            hf |= 0x40
        if reason & 8:
            hf |= 0x80
        cap = dec["cap_kn"]
        other = req.get("cap_kn")
        stop = bool(dec["stop"])
        if other is not None:
            cap = min(cap, float(other))
            if float(other) < float(self.planner.c["min_move_kn"]):
                stop = True
        o_hdg = o.get("heading")
        hdg = dec["heading"] if dec["heading"] is not None else (o_hdg if o_hdg is not None else 0.0)
        alarm = int(req.get("alarm") or 0)
        ready = (self.stat or {}).get("harb_state") not in RUNNING
        if ready:
            # HX7-fix: the READY lease (before the START): NO heading offset (the leg track the planner judged; else
            # the bow), the most restrictive cap (the planner's, the route leg's, ready_kn), the same lease, no alarm of
            # its own - harmless: inert on the AURIX until the captain's START + CONFIRM, and only ever lower
            lt = dec.get("leg_track")
            hdg = lt if lt is not None else (o_hdg if o_hdg is not None else 0.0)
            cap = min(cap, float(self.planner.c["ready_kn"]))
            # review M2: the cap was judged for the planner's OWN path - only a STRAIGHT solution (its heading = the
            # leg track) may be offered; any offset / turn = STOP (entry refused) until the boat is lined up
            ph = dec.get("heading")
            if lt is None or ph is None or abs(wrap(float(ph) - float(lt))) > 0.5:
                stop = True
        else:
            alarm |= int(dec.get("alarm") or 0)         # HX7-fix: not yielding = COLLISION (+ DECISION when stopped)
        if stop:
            hf |= 0x01
            cap = 0.0
        return {"kind": NAV_HARBOUR, "heading": hdg, "cap_kn": cap, "reason": reason,
                "alarm": alarm, "hflags": hf, "nearest_m": dec["nearest_m"],
                "n_tracks": dec["n_tracks"], "lease_ms": int(self.planner.c["lease_ms"]), "ready": ready}

    def view(self):
        """The decision as the stations draw it (HX6, read only): the AURIX state, the planner's heading / cap /
        explanation keys (harb.why_*), the unsure list - nothing the planner decides changes."""
        d = self.dec or {}
        st = self.stat or {}
        return {"state": st.get("harb_state"), "state_name": st.get("harb_state_name"), "why": st.get("harb_why"),
                "why_text": st.get("harb_why_text"), "kn": st.get("harb_kn"), "corr_m": st.get("harb_corr_m"),
                "xte_m": st.get("harb_xte_m"), "route": st.get("harb_route"), "entry_ok": st.get("harb_entry_ok"),
                "lease": st.get("harb_lease"), "grace": st.get("harb_grace"),
                "grace_left_s": st.get("harb_grace_left_s"), "max_kn": st.get("harb_max_kn"),
                "stat_fresh": self.stat_fresh(), "stop": d.get("stop"), "heading": d.get("heading"),
                "cap_kn": d.get("cap_kn"), "why_key": d.get("why"), "text_el": d.get("text_el"),
                "text_en": d.get("text_en"), "colregs": d.get("colregs"), "turn": d.get("turn"),
                "unsure": list(d.get("reasons") or [])[:6], "hflags": d.get("hflags"),
                "not_yielding": list(d.get("not_yielding") or [])[:6]}

    def status(self):
        d = self.dec or {}
        return {"active": self.dec is not None, "stat": {k: (self.stat or {}).get(k) for k in
                                                          ("harb_state_name", "harb_why_text", "harb_kn",
                                                           "harb_corr_m", "harb_xte_m")},
                "stat_fresh": self.stat_fresh(), "gate": self.gate,
                "stop": d.get("stop"), "why": d.get("why"), "text_el": d.get("text_el"), "text_en": d.get("text_en"),
                "cap_kn": d.get("cap_kn"), "heading": d.get("heading"), "reasons": d.get("reasons"),
                "colregs": d.get("colregs"), "turn": d.get("turn"), "not_yielding": d.get("not_yielding"),
                "decisions": self.decisions,
                "stops": self.stops, "errors": self.errors, "ms": d.get("ms")}
