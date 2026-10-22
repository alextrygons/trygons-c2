"""Collision avoidance on the gateway (section D). The gateway DECIDES and
PROPOSES; the AURIX only applies a validated request (navreq: heading and / or
speed cap + reason + alarm) in AUTO, or after the driver's ACCEPT in HEADING
LOCK, under all its own safety rules (power limit, joystick wins, STANDBY
wins, lease: no refresh -> slow + alarm). AIS does NOT show small boats, jet
skis, swimmers or kayaks: the lookout on board and the cameras stay essential.

Per target: CPA (closest point of approach) and TCPA (time to it) from our
position, speed and course (flat earth around us, straight courses).
  WARN    CPA < warn_nm  (0.5 nm) and 0 <= TCPA < warn_min  (10 min)
  DANGER  CPA < danger_nm (0.25 nm) and 0 <= TCPA < danger_min (6 min)

AUTO (automatic):
  1. a DANGER target -> slow down (slow_factor x speed, at least min_kn), re-check
  2. still DANGER after recheck_s -> a route ASTERN of the target: try headings
     to starboard first (+10 .. +90 deg), then to port (-10 .. -60); a route
     must keep every target at least safe_nm away, never cross ahead of any
     target's bow, and pass the depth check (shallow.py, same as the AUTO
     programs)
  3. clear (all targets opening and further than clear_nm) -> request ends, the
     AURIX follows its leg again (rejoins the track)
  Not safely solvable (two or more DANGER targets, a DANGER target turning,
  stale / inconsistent data, no safe route) -> DRIVER DECISION: slow to
  min_kn, keep the course, alarm. Never an unchecked route.
HEADING LOCK: the same route as a PROPOSAL (heading + suggested speed) for
  the driver; ACCEPT -> the locked heading changes; when clear: a return
  proposal (also ACCEPT). The throttle stays with the driver.
MANUAL / REMOTE MANUAL: alarm only.
"""
import math

from ais import KN, NM, bearing, from_xy, to_xy

MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING, MODE_REMOTE = range(6)

# navreq kinds / reasons / alarm bits (app/smallboat.h SB_NAV_*)
KIND_NONE, KIND_CAP, KIND_HEADING, KIND_HDG_SET = 0, 1, 2, 3
WHY_COLLISION, WHY_SHALLOW, WHY_DECISION = 1, 2, 4
ALM_COLLISION, ALM_DECISION, ALM_SHALLOW = 1, 2, 4

DEFAULTS = {
    "warn_nm": 0.5, "warn_min": 10.0, "danger_nm": 0.25, "danger_min": 6.0,
    "slow_factor": 0.5, "min_kn": 4.0, "recheck_s": 20.0, "safe_nm": 0.5, "clear_nm": 1.0,
    "horizon_min": 15.0, "data_max_age_s": 90.0, "turn_limit_dpm": 10.0,
    "starboard": [10, 20, 30, 40, 50, 60, 70, 80, 90], "port": [-10, -20, -30, -40, -50, -60],
    "side_m": 100.0,                    # "crossing ahead": from one side of its heading line to the other (> this)
    "repropose_s": 60.0,                # HDG LOCK: after a REJECT no new proposal for this long (the alarm stays)
}


def cpa_tcpa(own, tgt):
    """own / tgt: (x, y, vx, vy) in m and m/s -> (cpa m, tcpa s, distance now m)."""
    rx, ry = tgt[0] - own[0], tgt[1] - own[1]
    vx, vy = tgt[2] - own[2], tgt[3] - own[3]
    v2 = vx * vx + vy * vy
    d0 = math.hypot(rx, ry)
    if v2 < 1e-6:
        return d0, 0.0, d0
    t = -(rx * vx + ry * vy) / v2
    if t < 0:
        return d0, t, d0                                    # opening: closest is now
    return math.hypot(rx + vx * t, ry + vy * t), t, d0


def vel(sog_kn, cog):
    s = (sog_kn or 0.0) * KN
    c = math.radians(cog or 0.0)
    return s * math.sin(c), s * math.cos(c)


def crosses_ahead(own_pts, tgt_pts, tgt_cog, ahead_m=4.0 * NM, side_m=100.0):
    """True if our track crosses the target's heading line AHEAD of its bow
    (within ahead_m) - forbidden. own_pts / tgt_pts: same-time samples (x, y).
    A side counts once we are more than side_m off the line (starting ON the
    line - head-on, overtaking - is not a crossing)."""
    ux, uy = math.sin(math.radians(tgt_cog)), math.cos(math.radians(tgt_cog))
    side0 = None
    for (ox, oy), (tx, ty) in zip(own_pts, tgt_pts):
        rx, ry = ox - tx, oy - ty
        along = rx * ux + ry * uy
        side = rx * uy - ry * ux
        if abs(side) < side_m:
            continue
        s = side > 0
        if side0 is None:
            side0 = s
        elif s != side0:
            if 0 < along < ahead_m:
                return True
            side0 = s
    return False


class Avoider:
    def __init__(self, cfg=None, shallow=None, log=None):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.shallow = shallow              # shallow.Shallow: the same depth check as the AUTO programs
        self.log = log or (lambda text, **kw: None)
        self.state = "CLEAR"
        self.since = 0.0
        self.slow_kn = None
        self.set_kn = 0.0                   # our speed before slowing down
        self.track_cog = 0.0                # our course before the avoidance
        self.track_start = None
        self.heading = None                 # active avoidance heading (AUTO) / accepted (HDG LOCK)
        self.orig_heading = None            # HDG LOCK: the heading before the accepted proposal
        self.proposal = None                # HDG LOCK: {"id", "heading", "speed_kn", "text", "kind"}
        self.prop_id = 0
        self.route = []
        self.why = ""
        self.threats = []
        self.decisions = 0
        self.hdg_set = None                 # pending one-shot HDG LOCK heading (after ACCEPT)
        self.hdg_set_seq = 0
        self.rejected_until = 0.0
        self.best_effort = self.avoid_best = False
        self.last_dmin = 99.0

    # --- threats -----------------------------------------------------------------------
    def evaluate(self, own, targets, ref):
        """CPA / TCPA of every target (stale ones too: flagged). ref = (lat, lon) of the local frame."""
        c = self.cfg
        lat0, lon0 = ref
        ox, oy = to_xy(own["lat"], own["lon"], lat0, lon0)
        ovx, ovy = vel(own["sog"], own["cog"])
        out = []
        for t in targets:
            if t.get("lat") is None:
                continue
            tx, ty = to_xy(t["lat"], t["lon"], lat0, lon0)
            tvx, tvy = vel(t.get("sog"), t.get("cog"))
            cpa, tcpa, d0 = cpa_tcpa((ox, oy, ovx, ovy), (tx, ty, tvx, tvy))
            level = None
            if cpa < c["danger_nm"] * NM and 0 <= tcpa < c["danger_min"] * 60:
                level = "danger"
            elif cpa < c["warn_nm"] * NM and 0 <= tcpa < c["warn_min"] * 60:
                level = "warn"
            out.append({"mmsi": t["mmsi"], "name": t.get("name") or str(t["mmsi"]), "cpa_nm": round(cpa / NM, 3),
                        "tcpa_min": round(tcpa / 60.0, 2), "dist_nm": round(d0 / NM, 3),
                        "bearing": round(bearing(own["lat"], own["lon"], t["lat"], t["lon"]), 1), "level": level,
                        "stale": bool(t.get("stale")), "age_s": t.get("age_s"), "turn_dpm": t.get("turn_dpm", 0.0),
                        "cls": t.get("cls", "A"),
                        "sog": t.get("sog"), "cog": t.get("cog"), "lat": t["lat"], "lon": t["lon"]})
        out.sort(key=lambda r: (r["level"] != "danger", r["level"] != "warn", r["tcpa_min"] if r["tcpa_min"] >= 0
                                else 1e9))
        return out

    def unsafe_reason(self, danger):
        """Why the situation can't be solved automatically (None = it can try)."""
        c = self.cfg
        if len(danger) >= 2:
            return "%d ships at the same time" % len(danger)
        for d in danger:
            limit = c["data_max_age_s"] * (2.0 if d.get("cls") == "B" else 1.0)    # Class B reports less often
            if d["stale"] or (d["age_s"] is not None and d["age_s"] > limit):
                return "AIS data of %s is old (%s s)" % (d["name"], d["age_s"])
            if abs(d.get("turn_dpm") or 0.0) > c["turn_limit_dpm"]:
                return "%s is turning (%.0f deg/min)" % (d["name"], d["turn_dpm"])
            if d["sog"] is None or d["cog"] is None:
                return "%s sends no speed / course" % d["name"]
        return None

    # --- route search --------------------------------------------------------------------
    def check_route(self, own, heading, speed_kn, targets, ref, horizon_s, safe_nm=None, ahead_nm=4.0,
                    back=False):
        """A straight route on `heading` at speed_kn (back=True: the way back onto the track,
        steered like the AURIX) -> (ok, why, points lat/lon).
        self.last_dmin = the closest distance to any ship on it (nm)."""
        c = self.cfg
        self.last_dmin = 99.0
        lat0, lon0 = ref
        ox, oy = to_xy(own["lat"], own["lon"], lat0, lon0)
        steps = max(2, int(horizon_s / 10.0))
        if back:
            own_pts = self._back_pts(own, speed_kn, steps, ref)
        else:
            vx, vy = vel(speed_kn, heading)
            own_pts = [(ox + vx * k * 10.0, oy + vy * k * 10.0) for k in range(steps + 1)]
        for t in targets:
            if t.get("lat") is None or t.get("stale"):
                continue
            tx, ty = to_xy(t["lat"], t["lon"], lat0, lon0)
            tvx, tvy = vel(t.get("sog"), t.get("cog"))
            tpts = [(tx + tvx * k * 10.0, ty + tvy * k * 10.0) for k in range(steps + 1)]
            dist = [math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(own_pts, tpts)]
            safe = (safe_nm or c["safe_nm"]) * NM
            self.last_dmin = min(self.last_dmin, min(dist) / NM)
            if dist[0] < safe:
                # already closer than safe: the route must never come closer than now and open beyond safe
                if min(dist) < dist[0] - 20.0 or dist[-1] < safe:
                    return False, "does not open from %s (%.2f nm)" % (t.get("name") or t["mmsi"], dist[0] / NM), []
            elif min(dist) < safe:
                return False, "passes %s at %.2f nm" % (t.get("name") or t["mmsi"], min(dist) / NM), []
            if (t.get("sog") or 0) > 1.0 and crosses_ahead(own_pts, tpts, t["cog"], ahead_m=ahead_nm * NM,
                                                            side_m=c["side_m"]):
                return False, "would cross ahead of %s's bow" % (t.get("name") or t["mmsi"]), []
        pts = [from_xy(x, y, lat0, lon0) for x, y in own_pts]
        if self.shallow is not None:
            for (la1, lo1), (la2, lo2) in zip(pts, pts[1:]):
                bad = self.shallow.check_leg(la1, lo1, la2, lo2)
                if bad:
                    return False, "depth: %s at %.5f %.5f" % (bad[4], bad[1], bad[2]), []
        return True, "", pts

    def _back_pts(self, own, speed_kn, steps, ref):
        """The way back onto the track line in 10-s steps (cross-track steering, up to 45 deg)."""
        lat0, lon0 = self.track_start or (own["lat"], own["lon"])
        sx, sy = to_xy(lat0, lon0, ref[0], ref[1])
        x, y = to_xy(own["lat"], own["lon"], ref[0], ref[1])
        c = math.radians(self.track_cog)
        v = (speed_kn or 0.0) * KN
        pts = [(x, y)]
        for _k in range(steps):
            xte = (x - sx) * math.cos(c) - (y - sy) * math.sin(c)          # + = right of the track
            h = math.radians(self.track_cog - math.copysign(min(45.0, abs(xte) / 8.0), xte))
            x += v * math.sin(h) * 10.0
            y += v * math.cos(h) * 10.0
            pts.append((x, y))
        return pts

    def plan(self, own, targets, ref, speed_kn, horizon_s, alt_kn=None):
        """First safe heading: starboard first, then port; at the slow speed first,
        else at alt_kn (being overtaken: slowing down makes it worse).
        -> (heading, speed, points, tried)"""
        tried = []
        self.best_effort = False
        speeds = [speed_kn] + ([alt_kn] if alt_kn and alt_kn > speed_kn + 0.5 else [])
        # overtaking a slower ship (ahead, about the same course): a speed that doesn't pass it
        # clearly never ends the situation - only speeds at least 2 kn faster than it
        ox, oy = to_xy(own["lat"], own["lon"], ref[0], ref[1])
        overtaking = False
        for t in targets:
            if t.get("lat") is None or t.get("stale") or t.get("sog") is None or t.get("cog") is None:
                continue
            tx, ty = to_xy(t["lat"], t["lon"], ref[0], ref[1])
            ahead = (tx - ox) * math.sin(math.radians(own["cog"])) + (ty - oy) * math.cos(math.radians(own["cog"])) > 0
            same = abs((t["cog"] - own["cog"] + 540.0) % 360.0 - 180.0) < 60.0
            if ahead and same and math.hypot(tx - ox, ty - oy) < 3 * NM:
                speeds = sorted([kn for kn in speeds if kn >= t["sog"] + 2.0] or speeds, reverse=True)
                overtaking = True
        angles = list(self.cfg["starboard"]) + list(self.cfg["port"])
        if overtaking:          # pass it quickly: every angle at the faster speed first
            cands = [((own["cog"] + d) % 360.0, d, kn) for kn in speeds for d in angles]
        else:                   # the smallest turn first, the slow speed first
            cands = [((own["cog"] + d) % 360.0, d, kn) for d in angles for kn in speeds]
        for h, d, kn in cands:
            ok, why, pts = self.check_route(own, h, kn, targets, ref, horizon_s)
            tried.append("%+d @%.0f kn: %s" % (d, kn, "OK" if ok else why))
            if ok:
                return h, kn, pts, tried
        # nothing reaches safe_nm: the checked route that keeps the most room, if it is at
        # least out of the danger zone (always better than holding a collision course)
        best = None
        for h, d, kn in cands:
            ok, why, pts = self.check_route(own, h, kn, targets, ref, horizon_s, safe_nm=self.cfg["danger_nm"])
            if ok and (best is None or self.last_dmin > best[0]):
                best = (self.last_dmin, h, kn, pts, d)
        if best:
            self.best_effort = True
            tried.append("best available: %+d @%.0f kn keeps %.2f nm (below the safe %.2f nm)" % (
                best[4], best[2], best[0], self.cfg["safe_nm"]))
            return best[1], best[2], best[3], tried
        return None, None, [], tried

    # --- the decision ---------------------------------------------------------------------
    def _set(self, state, why, now):
        if state != self.state or why != self.why:
            self.decisions += 1
            self.log("avoid: %s -> %s (%s)" % (self.state, state, why), state=state, why=why)
        if state != self.state:
            self.since = now
        self.state, self.why = state, why

    def update(self, now, own, targets, mode):
        """own: {"lat", "lon", "sog", "cog", "heading"}; returns the navreq part for the AURIX:
        {"kind", "heading", "cap_kn", "reason", "alarm"} and fills threats / proposal / route."""
        c = self.cfg
        req = {"kind": KIND_NONE, "heading": None, "cap_kn": None, "reason": 0, "alarm": 0}
        if own.get("lat") is None:
            self.threats = []
            self._set("CLEAR", "no own position", now)
            return req
        ref = (own["lat"], own["lon"])
        self.threats = self.evaluate(own, targets, ref)
        danger = [t for t in self.threats if t["level"] == "danger"]
        warn = [t for t in self.threats if t["level"] == "warn"]
        horizon = c["horizon_min"] * 60.0
        if danger:
            horizon = min(horizon, max(t["tcpa_min"] for t in danger) * 60.0 + 300.0)
        clear = all((t["tcpa_min"] < 0 or t["cpa_nm"] >= c["safe_nm"]) and
                    (t["dist_nm"] >= c["clear_nm"] or t["tcpa_min"] < 0) for t in self.threats
                    if not t["stale"])
        if mode == MODE_AUTO:
            self._auto(now, own, targets, ref, danger, warn, clear, horizon, req)
        elif mode == MODE_HDG_LOCK:
            self._hdg_lock(now, own, targets, ref, danger, warn, clear, horizon, req)
        else:
            self.route, self.proposal, self.heading = [], None, None
            self.slow_kn = None
            if mode in (MODE_MANUAL, MODE_REMOTE):
                self._set("ALARM ONLY" if danger else "CLEAR", "manual: the driver decides", now)
                if danger:
                    req["alarm"] |= ALM_COLLISION
            else:
                self._set("CLEAR", "no driving mode", now)
        if self.hdg_set is not None and mode == MODE_HDG_LOCK:
            req["kind"] = KIND_HDG_SET
            req["heading"] = self.hdg_set
            req["reason"] |= WHY_COLLISION
        return req

    def _auto(self, now, own, targets, ref, danger, warn, clear, horizon, req):
        """Acts early: at the WARN level (CPA < warn_nm within warn_min); the alarm
        sounds for DANGER targets."""
        c = self.cfg
        self.proposal = None
        st = self.state
        threats = danger + warn
        quiet = ("CLEAR", "WATCH", "ALARM ONLY", "PROPOSE", "RETURN", "AVOIDING")
        if st in quiet and not threats:
            self.route, self.heading, self.slow_kn = [], None, None
            return self._set("CLEAR", "", now)
        unsafe = self.unsafe_reason(threats)
        if unsafe and st != "DRIVER DECISION":
            self._set("DRIVER DECISION", unsafe, now)
        if self.state == "DRIVER DECISION":
            if clear and not threats:
                self.route, self.heading, self.slow_kn = [], None, None
                return self._set("CLEAR", "clear again", now)
            req.update(kind=KIND_CAP, cap_kn=c["min_kn"], reason=WHY_COLLISION | WHY_DECISION,
                       alarm=ALM_COLLISION | ALM_DECISION)
            self.route = []
            return
        if self.state in quiet:
            self.set_kn = own["sog"] or 0.0
            self.track_cog = own["cog"] or 0.0          # the course to go back to
            self.track_start = (own["lat"], own["lon"])  # a point on the track line
            self.slow_kn = max(c["min_kn"], self.set_kn * c["slow_factor"])
            t0 = threats[0]
            self._set("SLOW", "%s: %s CPA %.2f nm in %.1f min - step 1: slow down to %.1f kn and re-check" % (
                t0["level"], t0["name"], t0["cpa_nm"], t0["tcpa_min"], self.slow_kn), now)
        alarm = ALM_COLLISION if danger else 0
        # still passing too close (any time ahead): step 2 is needed
        close = [t for t in self.threats if not t["stale"] and t["tcpa_min"] >= 0 and t["cpa_nm"] < c["safe_nm"]]
        if self.state == "SLOW":
            req.update(kind=KIND_CAP, cap_kn=self.slow_kn, reason=WHY_COLLISION, alarm=alarm)
            if not close:
                # leave the slow-down only when going on at the old speed is safe too (no flip-flop)
                back, _w, _p = self.check_route(own, None, self.set_kn or own["sog"], targets, ref, horizon,
                                                ahead_nm=c["clear_nm"], back=True)
                if clear and back:
                    self.slow_kn = None
                    self._set("CLEAR", "clear after slowing down", now)
                    req.update(kind=KIND_NONE, cap_kn=None, reason=0, alarm=0)
                return
            if now - self.since < c["recheck_s"]:
                return
            h, kn, pts, tried = self.plan(own, targets, ref, self.slow_kn, horizon, alt_kn=self.set_kn)
            if h is None:
                self._set("DRIVER DECISION", "no safe route (%s)" % "; ".join(tried[-3:]), now)
                req.update(kind=KIND_CAP, cap_kn=c["min_kn"], reason=WHY_COLLISION | WHY_DECISION,
                           alarm=ALM_COLLISION | ALM_DECISION)
                return
            self.heading, self.route, self.slow_kn = h, pts, kn
            self.avoid_best = self.best_effort
            self._set("AVOID", "step 2: %s %s - heading %03.0f at %.1f kn (%s)" % (
                "best available route around" if self.best_effort else "pass astern of", close[0]["name"], h, kn,
                tried[-1]), now)
        if self.state == "AVOID":
            # clear = going BACK (the old course, the old speed) is safe now: no ship closer than
            # safe_nm on the way, no bow crossed within clear_nm
            back, why_b, _p = self.check_route(own, None, self.set_kn or own["sog"], targets, ref, horizon,
                                               ahead_nm=c["clear_nm"], back=True)
            if back:
                self.heading, self.route, self.slow_kn = None, [], None
                self._set("CLEAR", "clear: back to the track", now)
                return
            ok, why, pts = self.check_route(own, self.heading, self.slow_kn, targets, ref, horizon)
            if not ok:
                h, kn, pts, tried = self.plan(own, targets, ref, self.slow_kn, horizon, alt_kn=self.set_kn)
                if h is None:
                    self._set("DRIVER DECISION", "avoidance route no longer safe (%s)" % why, now)
                    req.update(kind=KIND_CAP, cap_kn=c["min_kn"], reason=WHY_COLLISION | WHY_DECISION,
                               alarm=ALM_COLLISION | ALM_DECISION)
                    return
                self.heading, self.slow_kn = h, kn
                self.avoid_best = self.best_effort
                self._set("AVOID", "new avoidance heading %03.0f (%s)" % (h, why), now)
            self.route = pts
            # a best-available route (below the safe distance): the driver must watch -> DRIVER DECISION alarm too
            req.update(kind=KIND_HEADING, heading=self.heading, cap_kn=self.slow_kn,
                       reason=WHY_COLLISION | (WHY_DECISION if self.avoid_best else 0),
                       alarm=(ALM_COLLISION if danger else 0) | (ALM_DECISION if self.avoid_best else 0))

    def _hdg_lock(self, now, own, targets, ref, danger, warn, clear, horizon, req):
        """Proposes early, at the WARN level like AUTO (at DANGER a slow boat often can't get
        astern any more); the alarm sounds for DANGER targets."""
        c = self.cfg
        self.slow_kn = None
        threats = danger + warn
        if danger:
            req["alarm"] |= ALM_COLLISION
        if threats:
            unsafe = self.unsafe_reason(threats)
            if unsafe:
                self.proposal, self.route = None, []
                req["alarm"] |= ALM_DECISION
                return self._set("DRIVER DECISION", unsafe, now)
            if self.state != "PROPOSE" or self.proposal is None:
                if self.state == "AVOIDING" or now < self.rejected_until:
                    return                              # steering the accepted heading / the driver said no
                speed = max(c["min_kn"], (own["sog"] or 0.0) * c["slow_factor"])
                h, speed, pts, tried = self.plan(own, targets, ref, speed, horizon, alt_kn=own["sog"])
                if h is None:
                    self.proposal, self.route = None, []
                    req["alarm"] |= ALM_DECISION
                    return self._set("DRIVER DECISION", "no safe route", now)
                self.prop_id += 1
                self.proposal = {"id": self.prop_id, "kind": "avoid", "heading": round(h), "speed_kn": round(speed, 1),
                                 "text": "%s: CPA %.2f nm in %.1f min. Steer %03.0f and slow to %.0f kn?" % (
                                     threats[0]["name"], threats[0]["cpa_nm"], threats[0]["tcpa_min"], h, speed)}
                self.route = pts
                self._set("PROPOSE", "proposal: heading %03.0f, %.1f kn" % (h, speed), now)
            return
        if self.orig_heading is not None and clear:
            if (self.state != "RETURN" or self.proposal is None) and now >= self.rejected_until:
                self.prop_id += 1
                self.proposal = {"id": self.prop_id, "kind": "return", "heading": round(self.orig_heading),
                                 "speed_kn": None, "text": "Clear. Back to heading %03.0f?" % self.orig_heading}
                self.route = []
                self._set("RETURN", "proposal: back to %03.0f" % self.orig_heading, now)
            return
        if self.state in ("PROPOSE", "DRIVER DECISION") or (self.state == "RETURN" and self.orig_heading is None):
            self.proposal, self.route = None, []
            self._set("CLEAR", "no danger", now)

    def accept(self, prop_id, own_heading):
        """Driver ACCEPT of a HEADING LOCK proposal -> one-shot heading for the AURIX."""
        p = self.proposal
        if not p or p["id"] != prop_id:
            return False, "proposal no longer valid"
        if p["kind"] == "avoid":
            if self.orig_heading is None:
                self.orig_heading = own_heading
            self.heading = p["heading"]
        else:
            self.orig_heading = None
            self.heading = None
        self.hdg_set = p["heading"]
        self.hdg_set_seq += 1
        self.log("avoid: driver ACCEPTED heading %03d (%s)" % (p["heading"], p["kind"]), accepted=p["heading"])
        self.proposal = None
        self._set("CLEAR" if p["kind"] == "return" else "AVOIDING", "accepted %s" % p["kind"], 0.0)
        return True, "heading %03d" % p["heading"]

    def reject(self, prop_id, now):
        p = self.proposal
        if p and p["id"] == prop_id:
            self.log("avoid: driver REJECTED the proposal (%s)" % p["kind"], rejected=p["heading"])
            self.proposal = None
            self.rejected_until = now + self.cfg["repropose_s"]
            if p["kind"] == "return":
                self.orig_heading = None
            return True
        return False

    def hdg_set_sent(self):
        self.hdg_set = None

    def status(self):
        return {"state": self.state, "why": self.why, "threats": self.threats[:20], "proposal": self.proposal,
                "route": [[round(a, 6), round(b, 6)] for a, b in self.route[::3]], "slow_kn": self.slow_kn,
                "heading": self.heading, "decisions": self.decisions}
