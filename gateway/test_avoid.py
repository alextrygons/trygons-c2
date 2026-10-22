"""Collision avoidance (section D) - the gateway's planner against scenario
ships, with a simple boat that follows the navigation requests the way the
AURIX applies them (AUTO: avoidance heading + speed cap; leg to its waypoint
otherwise). No simulator needed, fast and repeatable.

    python test_avoid.py        (exit code 0 = all passed)

  CA1 CPA / TCPA against known cases
  CA2..CA6 AUTO: crossing from starboard, crossing from port, head-on, being
     overtaken, overtaking a slower ship -> slow down FIRST, then a route that
     passes astern (starboard preferred), back to the track when clear, never
     across a ship's bow, every ship kept at a safe distance
  CA7 the avoidance route blocked by shallow water -> another safe route or
     slow + DRIVER DECISION, never an unchecked route
  CA8 two threats at once / a ship turning towards us / stale AIS data ->
     slow down + DRIVER DECISION, keep the course
  CA9 HEADING LOCK: proposal, ACCEPT changes the locked heading, REJECT keeps
     it (no new proposal for a while), return-to-course proposal after clear
  CA10 MANUAL / REMOTE MANUAL: alarm only (no heading, no speed request)
"""
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "groundstation"))
import avoid as A  # noqa: E402
import chart  # noqa: E402
from ais import KN, NM, from_xy, move, to_xy  # noqa: E402
from shallow import Shallow  # noqa: E402

results = []
LAT0, LON0 = 37.70, 23.90


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def xy2ll(x, y):
    return from_xy(x, y, LAT0, LON0)


class Ship:
    def __init__(self, mmsi, x, y, sog, cog, name=None, turn=0.0, stale=False, age=0.0):
        self.mmsi, self.sog, self.cog, self.turn = mmsi, sog, cog, turn
        self.lat, self.lon = xy2ll(x, y)
        self.name = name or "SHIP %d" % mmsi
        self.stale, self.age = stale, age
        self.track = []

    def step(self, dt):
        self.cog = (self.cog + self.turn * dt / 60.0) % 360.0
        self.lat, self.lon = move(self.lat, self.lon, self.cog, self.sog, dt)
        self.track.append(to_xy(self.lat, self.lon, LAT0, LON0))

    def info(self):
        return {"mmsi": self.mmsi, "name": self.name, "lat": self.lat, "lon": self.lon, "sog": self.sog,
                "cog": self.cog, "stale": self.stale, "age_s": self.age, "turn_dpm": self.turn}


class Boat:
    """Follows the AUTO leg to wp (course to it), the avoidance heading / speed cap when asked."""

    def __init__(self, x, y, cog, sog, wp, mode=A.MODE_AUTO):
        self.lat, self.lon = xy2ll(x, y)
        self.cog, self.sog, self.set_kn = cog, sog, sog
        self.wp = xy2ll(*wp)
        self.mode = mode
        self.track = []
        self.req = {"kind": 0}

    def own(self):
        return {"lat": self.lat, "lon": self.lon, "sog": self.sog, "cog": self.cog, "heading": self.cog}

    def step(self, dt):
        r = self.req
        target, kn = self.cog, self.set_kn
        if self.mode == A.MODE_AUTO:
            # the leg is the line x = 0 northwards; like the AURIX: back onto the track (cross-track)
            x, _y = to_xy(self.lat, self.lon, LAT0, LON0)
            target = (-max(-45.0, min(45.0, x / 8.0))) % 360.0
            if r["kind"] == A.KIND_HEADING:
                target = r["heading"]
            if r["kind"] in (A.KIND_CAP, A.KIND_HEADING) and r.get("cap_kn") is not None:
                kn = min(kn, r["cap_kn"])
        d = (target - self.cog + 540.0) % 360.0 - 180.0
        self.cog = (self.cog + max(-5.0 * dt, min(5.0 * dt, d))) % 360.0     # 5 deg/s turn rate
        self.sog += (kn - self.sog) * min(1.0, dt / 8.0)                      # speed: 8 s time constant
        self.lat, self.lon = move(self.lat, self.lon, self.cog, self.sog, dt)
        self.track.append(to_xy(self.lat, self.lon, LAT0, LON0))


def run(boat, ships, seconds=2500, av=None, mode=None):
    av = av or A.Avoider({"recheck_s": 20.0})
    kinds, states, headings = [], [], []
    first_cap_t = first_hdg_t = None
    t = 0.0
    while t < seconds:
        req = av.update(t, boat.own(), [s.info() for s in ships], mode if mode is not None else boat.mode)
        boat.req = req
        kinds.append(req["kind"])
        states.append(av.state)
        if req["kind"] == A.KIND_CAP and first_cap_t is None:
            first_cap_t = t
        if req["kind"] == A.KIND_HEADING:
            if first_hdg_t is None:
                first_hdg_t = t
            headings.append(req["heading"])
        boat.step(1.0)
        for s in ships:
            s.step(1.0)
        t += 1.0
    dmin = {s.mmsi: min(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(boat.track, s.track)) / NM
            for s in ships}
    ahead = any(A.crosses_ahead(boat.track, s.track, s.cog, ahead_m=0.5 * NM) for s in ships if s.sog > 1)
    return {"kinds": kinds, "states": states, "headings": headings, "first_cap": first_cap_t,
            "first_hdg": first_hdg_t, "dmin": dmin, "ahead": ahead, "end_kind": kinds[-1], "av": av}


def collision_course(t_cpa, own_sog, tgt_sog, tgt_cog, offset_m=0.0):
    """A ship that would hit us (own heading north from (0, 0)) after t_cpa seconds."""
    px, py = offset_m, own_sog * KN * t_cpa
    vx, vy = tgt_sog * KN * math.sin(math.radians(tgt_cog)), tgt_sog * KN * math.cos(math.radians(tgt_cog))
    return px - vx * t_cpa, py - vy * t_cpa


def scenario(name, tgt_sog, tgt_cog, t_cpa=420, offset=0.0, own_sog=14.0):
    x, y = collision_course(t_cpa, own_sog, tgt_sog, tgt_cog, offset)
    ship = Ship(239000001, x, y, tgt_sog, tgt_cog, name=name)
    boat = Boat(0, 0, 0.0, own_sog, (0, 12000))
    r = run(boat, [ship])
    start_hdg = 0.0
    first = r["headings"][0] if r["headings"] else None
    delta = None if first is None else (first - start_hdg + 540.0) % 360.0 - 180.0
    xte = abs(boat.track[-1][0])                     # back on the track (x = 0 line)
    return r, delta, xte


def main():
    # CA1 CPA / TCPA known cases
    head_on = A.cpa_tcpa((0, 0, 0, 10 * KN), (0, 2 * NM, 0, -10 * KN))
    beam = A.cpa_tcpa((0, 0, 0, 10 * KN), (1 * NM, 1 * NM, -10 * KN, 0))
    miss = A.cpa_tcpa((0, 0, 0, 10 * KN), (0.5 * NM, 2 * NM, 0, -10 * KN))
    open_ = A.cpa_tcpa((0, 0, 0, 10 * KN), (0, -1 * NM, 0, -5 * KN))
    still = A.cpa_tcpa((0, 0, 0, 0), (3 * NM, 4 * NM, 0, 0))
    ok = (abs(head_on[0]) < 1 and abs(head_on[1] - 360) < 1 and abs(beam[0]) < 1 and abs(beam[1] - 360) < 1 and
          abs(miss[0] - 0.5 * NM) < 1 and abs(miss[1] - 360) < 1 and open_[1] < 0 and abs(open_[0] - NM) < 1 and
          abs(still[0] - 5 * NM) < 1)
    report("CA1 CPA / TCPA known cases (head-on, crossing at 90 deg, passing 0.5 nm, opening, both stopped)", ok,
           "head-on 2 nm at 10+10 kn: CPA %.0f m in %.0f s (360); crossing: %.0f m in %.0f s (360); passing: CPA "
           "%.2f nm in %.0f s; opening: TCPA %.0f s (< 0), CPA %.2f nm = now; stopped: %.2f nm" % (
               head_on[0], head_on[1], beam[0], beam[1], miss[0] / NM, miss[1], open_[1], open_[0] / NM, still[0] / NM))

    # CA2..CA6 the classic encounters in AUTO (+ "b": the ship timed to meet us AFTER we slowed down,
    # so slowing alone is not enough and step 2 must find a route)
    cases = [("CA2 crossing from STARBOARD (ship heading west)", 12.0, 270.0, 420, 0.0, 14.0),
             ("CA2b slow ship (4 kn) crossing from starboard: slowing is not enough", 4.0, 270.0, 420, 0.0, 14.0),
             ("CA3 crossing from PORT (ship heading east)", 12.0, 90.0, 420, 0.0, 14.0),
             ("CA3b slow ship (4 kn) crossing from port: slowing is not enough", 4.0, 90.0, 420, 0.0, 14.0),
             ("CA4 head-on", 12.0, 180.0, 300, 40.0, 14.0),
             ("CA5 being overtaken (25 kn ship from astern)", 25.0, 0.0, 300, -30.0, 14.0),
             ("CA6 overtaking a slower ship (5 kn ahead)", 5.0, 0.0, 420, 30.0, 14.0)]
    for name, sog, cog, tc, off, plan_kn in cases:
        x, y = collision_course(tc, plan_kn, sog, cog, off)
        ship = Ship(239000001, x, y, sog, cog, name=name.split(" ", 1)[1])
        boat = Boat(0, 0, 0.0, 14.0, (0, 12000))
        r = run(boat, [ship])
        first = r["headings"][0] if r["headings"] else None
        delta = None if first is None else (first + 540.0) % 360.0 - 180.0
        xte = abs(boat.track[-1][0])
        dmin = list(r["dmin"].values())[0]
        slow_first = r["first_cap"] is not None and (r["first_hdg"] is None or r["first_cap"] < r["first_hdg"])
        by_slowing = r["first_hdg"] is None and dmin >= 0.45
        need_route = name.split(" ")[0].endswith("b") or "overtak" in name
        good = slow_first and (by_slowing or r["first_hdg"] is not None) and not r["ahead"] and dmin >= 0.25 and             r["end_kind"] == A.KIND_NONE and xte < 150.0
        if need_route:
            good = good and r["first_hdg"] is not None
        if r["first_hdg"] is not None and "PORT" not in name and "from port" not in name:
            good = good and delta > 0                                       # starboard preferred
        report("%s: slow down first; then %s; never across a bow; back to the track" % (
            name, "a route (step 2)" if need_route else "slowing alone or a route"), good,
            "slowed at %s s; %s; closest %.2f nm; crossed a bow %s; end: request %s, %.0f m off the track; %s" % (
                r["first_cap"], "solved by slowing down" if by_slowing else "avoidance heading at %s s (%s deg %s)" % (
                    r["first_hdg"], None if delta is None else "%+.0f" % delta,
                    "starboard" if delta and delta > 0 else "port" if delta else "-"), dmin,
                "YES" if r["ahead"] else "no", ["none", "cap", "heading", "hdg set"][r["end_kind"]], xte,
                r["av"].why[:90]))

    # CA7 avoidance route blocked by shallow water (shoal to starboard of us)
    tmp = tempfile.mkdtemp(prefix="avoid_test_")
    try:
        dlat, dlon = 0.0005, 0.0006
        lat0, lon0 = LAT0 - 0.02, LON0 - 0.03
        rows = []
        for i in range(320):
            rw = []
            for j in range(200):
                la, lo = lat0 + i * dlat, lon0 + j * dlon
                x, y = to_xy(la, lo, LAT0, LON0)
                rw.append(1.0 if 300 < x < 3000 and -500 < y < 9000 else 25.0)     # shallow strip to starboard
            rows.append(rw)
        chart.write_area(os.path.join(tmp, "a"), "a", lat0, lon0, dlat, dlon, rows, ["synthetic"], "test")
        sh = Shallow({"clearance_m": 50.0}, chart.Charts(tmp))
        x, y = collision_course(420, 14.0, 4.0, 270.0)                  # slow crosser: a route is needed
        ship = Ship(239000002, x, y, 4.0, 270.0, name="CROSSER")
        boat = Boat(0, 0, 0.0, 14.0, (0, 12000))
        av = A.Avoider({"recheck_s": 20.0}, shallow=sh)
        r = run(boat, [ship], av=av)
        deep_ok = all(sh.charts.depth(*xy2ll(px, py)) is not None and sh.charts.depth(*xy2ll(px, py)) >= sh.limit()
                      for px, py in boat.track)
        dd = "DRIVER DECISION" in r["states"]
        route_ok = r["first_hdg"] is not None
        first = r["headings"][0] if r["headings"] else None
        report("CA7 avoidance route blocked by shallow water: another safe route (depth-checked) or slow + DRIVER "
               "DECISION",
               deep_ok and (route_ok or dd) and not r["ahead"] and min(r["dmin"].values()) >= 0.2,
               "%s; the boat never over water shallower than %.1f m: %s; closest %.2f nm; crossed a bow %s" % (
                   "avoidance heading %03.0f (starboard blocked by the shoal)" % first if route_ok else
                   "no safe route -> DRIVER DECISION, slowed", sh.limit(), deep_ok, min(r["dmin"].values()),
                   r["ahead"]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # CA8 two threats / a turning ship / stale data -> slow + DRIVER DECISION, keep the course
    out = []
    ok8 = True
    for label, ships in (
            ("two ships at once", [Ship(239000003, *collision_course(400, 14, 12, 270), 12, 270, name="A"),
                                   Ship(239000004, *collision_course(380, 14, 12, 90), 12, 90, name="B")]),
            ("ship turning towards us", [Ship(239000005, *collision_course(400, 14, 12, 270), 12, 270, name="TURNER",
                                              turn=-20.0)]),
            ("stale AIS data", [Ship(239000006, *collision_course(400, 14, 12, 270), 12, 270, name="OLD",
                                     age=200.0)])):
        boat = Boat(0, 0, 0.0, 14.0, (0, 12000))
        av = A.Avoider({"recheck_s": 20.0})
        t, dd_speed, kinds, states = 0.0, [], [], []
        while t < 300:
            req = av.update(t, boat.own(), [s.info() for s in ships], A.MODE_AUTO)
            boat.req = req
            kinds.append(req["kind"])
            states.append(av.state)
            if av.state == "DRIVER DECISION":
                dd_speed.append(req["cap_kn"])
            boat.step(1.0)
            for s in ships:
                s.step(1.0)
            t += 1.0
        dd = "DRIVER DECISION" in states
        no_hdg = A.KIND_HEADING not in kinds
        slowed = bool(dd_speed) and max(dd_speed) <= 4.0
        ok8 = ok8 and dd and no_hdg and slowed
        out.append("%s: DRIVER DECISION %s, heading request %s, speed cap %.1f kn" % (
            label, "yes" if dd else "NO", "none" if no_hdg else "SENT", min(dd_speed) if dd_speed else -1))
    report("CA8 two threats / a ship turning towards us / stale data -> slow down + DRIVER DECISION, keep the course",
           ok8, "; ".join(out))

    # CA9 HEADING LOCK: propose, ACCEPT, REJECT, return proposal
    def hdg_run(accept):
        x, y = collision_course(420, 10.0, 12.0, 270.0)
        ship = Ship(239000007, x, y, 12.0, 270.0, name="CROSSER")
        boat = Boat(0, 0, 0.0, 10.0, (0, 12000), mode=A.MODE_HDG_LOCK)
        av = A.Avoider()
        locked = 0.0
        prop = ret = None
        accepted = []
        t = 0.0
        while t < 1500:
            req = av.update(t, boat.own(), [ship.info()], A.MODE_HDG_LOCK)
            if req["kind"] == A.KIND_HDG_SET:
                locked = req["heading"]
                accepted.append(locked)
                av.hdg_set_sent()
            p = av.proposal
            if p and p["kind"] == "avoid" and prop is None:
                prop = p
                if accept:
                    av.accept(p["id"], boat.cog)
                else:
                    av.reject(p["id"], t)
            if p and p["kind"] == "return" and ret is None:
                ret = p
                av.accept(p["id"], boat.cog)
            d = (locked - boat.cog + 540.0) % 360.0 - 180.0         # HDG LOCK: steer the locked heading
            boat.cog = (boat.cog + max(-5.0, min(5.0, d))) % 360.0
            boat.lat, boat.lon = move(boat.lat, boat.lon, boat.cog, boat.sog, 1.0)
            boat.track.append(to_xy(boat.lat, boat.lon, LAT0, LON0))
            ship.step(1.0)
            t += 1.0
        return prop, ret, accepted, av
    p1, r1, acc1, av1 = hdg_run(True)
    p2, r2, acc2, av2 = hdg_run(False)
    report("CA9 HEADING LOCK: proposal; ACCEPT changes the locked heading; REJECT keeps it; return proposal when clear",
           p1 is not None and p1["heading"] != 0 and r1 is not None and len(acc1) == 2 and acc1[0] == p1["heading"] and
           acc1[1] == 0 and p2 is not None and acc2 == [] and r2 is None,
           "proposal \"%s\"; ACCEPT -> locked %s, later \"%s\" -> %s; REJECT -> locked heading unchanged (%s), "
           "no return proposal" % (p1 and p1["text"], acc1[:1], r1 and r1["text"], acc1[1:], acc2))

    # CA10 MANUAL / REMOTE MANUAL: alarm only
    rr = []
    for mode in (A.MODE_MANUAL, A.MODE_REMOTE):
        x, y = collision_course(300, 14.0, 12.0, 270.0)
        ship = Ship(239000008, x, y, 12.0, 270.0)
        boat = Boat(0, 0, 0.0, 14.0, (0, 12000), mode=mode)
        r = run(boat, [ship], seconds=200, mode=mode)
        alarm = r["av"].state == "ALARM ONLY" or "ALARM ONLY" in r["states"]
        rr.append((alarm, set(r["kinds"]) == {A.KIND_NONE}))
    report("CA10 MANUAL / REMOTE MANUAL: collision alarm only - no heading or speed request",
           all(a and n for a, n in rr), "MANUAL: alarm %s, requests only 'none' %s; REMOTE: alarm %s, %s" % (
               rr[0][0], rr[0][1], rr[1][0], rr[1][1]))

    bad = results.count(False)
    print("AVOIDANCE TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
