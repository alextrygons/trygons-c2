"""Part 4.4, gateway side: random storms against the REAL gateway modules (not
changed), each checked by its own independent rule. Every run prints its seed.

    python tests/acceptance/storm_gateway.py [--episodes N] [--seed S] [--part avoid|channels|events|vision|licence]

  I14 avoidance routes / accepted AUTO programs never over water shallower than
      the limit (checked independently: the chart every 5 m), avoidance never
      crosses ahead of a ship's bow (own geometry)              -> gateway/avoid.py, shallow.py
  I15 STARLINK never used for sync; never switched to automatically while a
      normal channel works                                     -> gateway/channels.py
  I17 a saved event that is not fully synced is never deleted by the storage
      policy (also: never an open one)                          -> gateway/recorder.py
  I20 vision never steers (only a speed cap), never asks for more speed;
      STAGE 3 off -> no cap at all
  I21 a vision crash / overload never releases an active cap by itself (only
      the driver's CLEAR, STAGE 3 off, or leaving AUTO)          -> gateway/vision/manager.py
  I26 REMOTE MANUAL / MANEUVER are never asked of the AURIX for a user without a
      manual licence                                            -> gateway/stations.py, users.py
  (I16, driver ACCEPT, needs the real gateway + simulator: stations_storm.py)
"""
import argparse
import json
import math
import os
import random
import shutil
import sys
import tempfile
import time
import time as _time_mod

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "gateway"))
sys.path.insert(0, os.path.join(REPO, "groundstation"))
import avoid as A  # noqa: E402
import chart  # noqa: E402
import test_avoid as TA  # noqa: E402 - its Boat / Ship models (test tooling)
from ais import NM, from_xy, to_xy  # noqa: E402
from shallow import Shallow  # noqa: E402
import simais  # noqa: E402
import channels as CH  # noqa: E402
import recorder as RC  # noqa: E402

RES = {}
CAUSES = {}


def res(inv, ok, seed=None, text=""):
    r = RES.setdefault(inv, {"checks": 0, "fails": 0, "first": None})
    r["checks"] += 1
    if not ok:
        r["fails"] += 1
        if r["first"] is None:
            r["first"] = (seed, text)


# ------------------------------------------------------------------------- I14 --
def synthetic_chart(folder, rng):
    """20-60 m sea with 4-8 shoals (1-2.5 m) of 40-400 m; 25 m cells (finer than the real chart)."""
    lat0, lon0 = TA.LAT0 - 0.06, TA.LON0 - 0.08
    dlat, dlon = 0.000225, 0.000285
    shoals = []
    for _ in range(rng.randint(4, 8)):
        x, y = rng.uniform(-4000, 4000), rng.uniform(-1000, 6000)
        if abs(x) < 300:
            x += 600 if x >= 0 else -600                       # not on the AUTO leg itself (x = 0)
        shoals.append((x, y, rng.uniform(40, 400), rng.uniform(1.0, 2.5)))
    rows = []
    for i in range(540):
        r = []
        for j in range(560):
            la, lo = lat0 + i * dlat, lon0 + j * dlon
            x, y = to_xy(la, lo, TA.LAT0, TA.LON0)
            d = 20.0 + 40.0 * (j / 560.0)
            for sx, sy, rad, dep in shoals:
                if math.hypot(x - sx, y - sy) < rad:
                    d = min(d, dep)
            r.append(d)
        rows.append(r)
    chart.write_area(os.path.join(folder, "s"), "s", lat0, lon0, dlat, dlon, rows, ["synthetic"], "acceptance storm")
    return shoals


def dense_ok(charts, la1, lo1, la2, lo2, limit, step=5.0):
    """Independent: the chart every 5 m along the segment (no clearance) -> (ok, worst depth)."""
    d = chart.dist_m(la1, lo1, la2, lo2)
    n = max(1, int(d / step))
    worst = 1e9
    for k in range(n + 1):
        la, lo = la1 + (la2 - la1) * k / n, lo1 + (lo2 - lo1) * k / n
        dep = charts.depth(la, lo)
        if dep is None or dep < limit:
            return False, dep
        worst = min(worst, dep)
    return True, worst


def crosses_ahead_indep(route, start_t, ship, side_m=100.0, ahead_m=2.0 * NM):
    """Own geometry: the planned route (a point every 10 s) passes from one side of the ship's
    course line to the other while ahead of the ship (and within 2 nm)."""
    vx, vy = math.sin(math.radians(ship.cog)), math.cos(math.radians(ship.cog))
    v = ship.sog * 0.514444
    sx, sy = to_xy(ship.lat, ship.lon, TA.LAT0, TA.LON0)
    side = 0
    for k, (la, lo) in enumerate(route):
        x, y = to_xy(la, lo, TA.LAT0, TA.LON0)
        qx, qy = sx + vx * v * k * 10.0, sy + vy * v * k * 10.0
        dx, dy = x - qx, y - qy
        ahead = dx * vx + dy * vy
        lat = dx * vy - dy * vx                                      # + = right of the ship's course
        s = 1 if lat > side_m else (-1 if lat < -side_m else 0)
        if s and side and s != side and 0 < ahead < ahead_m:
            return True
        if s:
            side = s
    return False


def storm_avoid(seed):
    rng = random.Random(seed)
    tmp = tempfile.mkdtemp(prefix="acc_avoid_")
    try:
        synthetic_chart(tmp, rng)
        charts = chart.Charts(tmp)
        sh = Shallow({"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0, "look_s": 30.0,
                      "look_min_m": 200.0, "slow_kn": 4.0, "require_charts": True}, charts)
        lim = sh.limit()
        # AUTO programs: accepted ones must be deep everywhere (independent dense check)
        for _ in range(6):
            wps = [from_xy(rng.uniform(-3500, 3500), rng.uniform(-500, 5500), TA.LAT0, TA.LON0)
                   for _ in range(rng.randint(1, 5))]
            if sh.check_program(wps) is None:
                good = all(dense_ok(charts, a[0], a[1], b[0], b[1], lim)[0] for a, b in zip(wps, wps[1:]))
                res("I14", good, seed, "program accepted but a leg crosses water < %.1f m: %s" % (lim, wps))
        # traffic around a boat on its AUTO leg (north along x = 0)
        av = A.Avoider({"recheck_s": 20.0}, sh)
        boat = TA.Boat(0, 0, 0.0, rng.uniform(6, 16), (0, 12000))
        ships = []
        for x in simais.traffic(TA.LAT0, TA.LON0, rng.randint(6, 14), seed=seed, radius_nm=3.0):
            px, py = to_xy(x.lat, x.lon, TA.LAT0, TA.LON0)
            if math.hypot(px, py) < 700:
                continue
            ships.append(TA.Ship(x.mmsi, px, py, x.sog, x.cog, name=x.name, turn=x.turn_dpm))
        if rng.random() < 0.5:                                       # a sure crossing from starboard / port
            cx, cy = TA.collision_course(rng.uniform(300, 500), boat.sog, rng.uniform(8, 18),
                                         rng.choice((270.0, 90.0)), rng.uniform(-200, 200))
            ships.append(TA.Ship(239999999, cx, cy, rng.uniform(8, 18), rng.choice((270.0, 90.0)), name="CROSSER"))
        last_route = None
        t = 0.0
        while t < 1200:
            req = av.update(t, boat.own(), [s.info() for s in ships], A.MODE_AUTO)
            if req["kind"] == A.KIND_HEADING and av.route and av.route != last_route:
                last_route = list(av.route)
                deep = all(dense_ok(charts, a[0], a[1], b[0], b[1], lim)[0] for a, b in zip(last_route, last_route[1:]))
                res("I14", deep, seed, "avoidance route at t=%.0f s over water < %.1f m" % (t, lim))
                bow = any(crosses_ahead_indep(last_route, t, s) for s in ships if s.sog > 1.0 and not s.stale)
                res("I14", not bow, seed, "avoidance route at t=%.0f s crosses ahead of a ship's bow" % t)
            boat.req = req
            boat.step(1.0)
            for s in ships:
                s.step(1.0)
            t += 1.0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------------- I15 --
class Clock:
    def __init__(self):
        self.t = 1.8e9

    def __call__(self):
        return self.t


def storm_channels(seed):
    rng = random.Random(seed)
    clk = Clock()
    drv = CH.SimDriver([c["name"] for c in CH.DEFAULT_CHANNELS])
    why = []
    net = CH.ChannelManager({"stable_s": 60.0, "fail_s": 30.0, "trial_s": 60.0, "starlink_status": False},
                            driver=drv, clock=clk, audit=lambda kind, **kw: why.append((kind, kw)))
    normal = [c["name"] for c in CH.DEFAULT_CHANNELS if not c.get("emergency_only")]
    for _ in range(7200):                                            # 2 hours, one tick a second
        clk.t += 1.0
        p = rng.random()
        if p < 0.02:                                                 # a link goes down / comes back
            n = rng.choice(list(drv.state))
            drv.state[n]["internet"] = rng.random() < 0.5
            drv.state[n]["up"] = drv.state[n]["internet"] or rng.random() < 0.5
        elif p < 0.025:                                              # a station asks for something
            mode = rng.choice(("AUTO", "MANUAL", "MANUAL_STRICT"))
            net.request(mode, rng.choice(list(drv.state)), user="u%d" % rng.randint(1, 3),
                        local=rng.random() < 0.5, may=rng.random() < 0.9, confirm_emergency=rng.random() < 0.3)
        elif p < 0.027:
            net.keep("u%d" % rng.randint(1, 3))
        before = net.active
        manual_star = net.mode in ("MANUAL", "MANUAL_STRICT") and net.manual in net.emergency
        # a trial running out goes back to the previous setting: an explicit (confirmed) STARLINK choice
        # of a user is not "automatic"; going back to STARLINK that AUTO had picked is
        tr = net.trial
        if tr and clk.t + 1.0 >= tr["deadline"] and tr["prev"][0] in ("MANUAL", "MANUAL_STRICT") and                 tr["prev"][1] in net.emergency:
            manual_star = True
        net.tick()
        normal_ok = [n for n in normal if net.healthy(n)]
        if net.active != before and net.active in net.emergency and not manual_star:
            last = [w for w in why if w[0] in ("channel", "channel_trial_reverted")][-2:]
            CAUSES[(net.mode, last[-1][1].get("why") if last else "?")] = CAUSES.get(
                (net.mode, last[-1][1].get("why") if last else "?"), 0) + (0 if not normal_ok else 1)
            res("I15", not normal_ok, seed, "switched to %s automatically although %s worked (mode %s; %s)" % (
                net.active, normal_ok, net.mode, last))
        if net.active in net.emergency:
            res("I15", not net.sync_allowed(), seed, "sync allowed on %s" % net.active)
        else:
            res("I15", True)


# ------------------------------------------------------------------------- I17 --
class FakeStorage:
    def __init__(self, root, capacity):
        self.root, self.capacity = root, capacity
        self.low_b, self.full_b = capacity * 0.3, capacity * 0.05
        self.state = "ok"

    @property
    def usable(self):
        return True

    def used(self):
        return sum(os.path.getsize(os.path.join(d, f)) for d, _s, fs in os.walk(self.root) for f in fs)

    def free_now(self):
        return self.capacity - self.used()

    def folder(self, *parts):
        p = os.path.join(self.root, *parts)
        os.makedirs(p, exist_ok=True)
        return p

    def check(self, force=False):
        f = self.free_now()
        self.state = "FULL" if f < self.full_b else ("low" if f < self.low_b else "ok")

    def status(self):
        return {"state": self.state}


class FakeGw:
    def __init__(self, storage):
        self.storage = storage
        self.log = []

    def audit(self, kind, **kw):
        self.log.append(dict(kw, kind=kind))


def storm_events(seed):
    rng = random.Random(seed)
    tmp = tempfile.mkdtemp(prefix="acc_events_")
    try:
        sto = FakeStorage(tmp, 4_000_000)
        gw = FakeGw(sto)
        rec = RC.Recorder({"enabled": True, "segment_s": 60}, gw)
        evdir = rec.events_dir()
        for k in range(rng.randint(5, 20)):
            eid = "ev%03d" % k
            folder = os.path.join(evdir, eid)
            os.makedirs(folder)
            with open(os.path.join(folder, "event.json"), "w") as f:
                json.dump({"id": eid, "start_t": 1000.0 + k, "synced": False}, f)
            for j in range(rng.randint(0, 4)):
                p = os.path.join(folder, "cam%d.ts" % j)
                with open(p, "wb") as f:
                    f.write(b"\0" * rng.randint(10_000, 400_000))
            for fn in os.listdir(folder):                        # some files confirmed by the shore
                if rng.random() < 0.6:
                    open(os.path.join(folder, fn + ".synced"), "w").close()
        if rng.random() < 0.5:                                   # one is still open (being recorded)
            rec.events.append({"id": "ev%03d" % rng.randint(0, 4)})
        snap = {}
        for n in os.listdir(evdir):
            folder = os.path.join(evdir, n)
            files = [f for f in os.listdir(folder) if not f.endswith(".synced")]
            snap[n] = all(os.path.exists(os.path.join(folder, f + ".synced")) for f in files) and bool(files)
        open_ids = {e["id"] for e in rec.events}
        sto.capacity = sto.used() + int(sto.low_b * rng.uniform(0.2, 0.9))   # space is short
        rec.housekeeping(now=5000.0)
        for n, all_synced in snap.items():
            gone = not os.path.isdir(os.path.join(evdir, n))
            res("I17", not gone or (all_synced and n not in open_ids), seed,
                "event %s deleted although %s" % (n, "open" if n in open_ids else "not fully synced"))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------------------------------------- I20 / I21 --
class VClock:
    def __init__(self):
        self.t = 1.8e9

    def time(self):
        return self.t

    gmtime = staticmethod(_time_mod.gmtime)
    strftime = staticmethod(_time_mod.strftime)


class VGw:
    def __init__(self, root):
        self.cfg = {"cameras": {"enabled": False}}
        self.storage = FakeStorage(root, 1e9)
        self.telem = {"mode": 0, "heading": 0.0, "est_sog": 6.0, "yaw_rate": 0.0}
        self.telem2 = {}
        self.nav_state = {}
        self.cams = None
        self.log = []

    def audit(self, kind, **kw):
        self.log.append(kind)


def storm_vision(seed):
    import vision.manager as VM
    rng = random.Random(seed)
    clk = VClock()
    real_time = VM.time
    VM.time = clk
    tmp = tempfile.mkdtemp(prefix="acc_vision_")
    try:
        gw = VGw(tmp)
        v = VM.VisionManager({"source": "fake", "allow_inject": True, "max_fps": 4.0, "stage3": rng.random() < 0.7,
                              "clear_s": 5.0}, gw)
        prev_cap, prev_state = None, None
        for _ in range(int(30 * 60 / 0.25)):                          # 30 minutes, 4 ticks a second
            clk.t += 0.25
            op = None
            p = rng.random()
            if p < 0.02:
                v.inject(rng.uniform(-30, 30), rng.uniform(15, 300), seconds=rng.uniform(2, 30),
                         frames=rng.choice((None, None, 1, 2)))
            elif p < 0.024:
                gw.telem["mode"] = rng.choice((0, 1, 2, 3, 3, 3, 5))
                op = "mode"
            elif p < 0.027:
                v.fail(rng.choice(("crash: worker died", "overload: 0.3 fps", None)))
            elif p < 0.029:
                v.clear("driver")
                op = "clear"
            elif p < 0.030:
                v.set_stage("stage3", rng.random() < 0.5, "captain")
                op = "stage"
            elif p < 0.032:
                v.set_stage(rng.choice(("stage1", "stage2")), rng.random() < 0.5, "captain")
            v.tick(clk.t, lag_ms=rng.choice((0.0, 0.0, 0.0, 300.0)))
            cap, alarm, why = v.nav(gw.telem["mode"])
            # I20: a cap only (never a heading: nav() has no heading at all), never more speed
            res("I20", alarm & ~8 == 0 and (cap is None or cap >= 0.0), seed, "nav() gave alarm %s cap %s" % (alarm, cap))
            if not v.cfg["stage3"]:
                res("I20", cap is None, seed, "STAGE 3 off but a cap %s" % cap)
            if prev_cap is not None and cap is not None and op is None:   # (after a CLEAR a new cap is a new one)
                res("I20", cap <= prev_cap + 1e-9, seed, "cap went UP %.1f -> %.1f kn" % (prev_cap, cap))
            # I21: vision not working -> an active cap is never released by itself
            if prev_cap is not None and cap is None and v.state != "ACTIVE" and op is None:
                res("I21", False, seed, "cap %.1f kn released while vision %s (%s)" % (prev_cap, v.state, v.reason))
            else:
                res("I21", True)
            prev_cap, prev_state = cap, v.state
    finally:
        VM.time = real_time
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------------- I26 --
class _Writer:
    def write(self, b):
        pass

    def close(self):
        pass


class _Aurix:
    connected = True

    def __init__(self):
        self.sent = []

    def on(self, t, f):
        pass

    def send(self, t, p):
        self.sent.append((t, p))


class _Gw:
    def __init__(self, users):
        import users as U
        self.aurix = _Aurix()
        self.users = U.Users(users)
        self.cfg = {}
        self.telem2 = {}
        self.telem = {}

    def audit(self, *a, **k):
        pass


def storm_licence(seed):
    """Random users (licensed or not, HELM / VIEWER, local / remote) send random commands
    straight into the real StationServer.command(); every frame that reaches the AURIX
    is checked: a REMOTE / MANEUVER request (payload 0) only from a licensed user."""
    import stations as STN
    import proto as P
    rng = random.Random(seed)
    users = {}
    for i in range(6):
        u = {"role": rng.choice(["HELM", "HELM", "VIEWER"]), "pw": "x"}
        if rng.random() < 0.5:
            u["licence"] = rng.choice([True, True, "yes", 1, False])     # only the JSON true counts
        users["u%d" % i] = u
    gw = _Gw(users)
    srv = STN.StationServer(gw, {"rate_per_s": 1e6, "rate_burst": 1e6})
    sts = []
    for i, name in enumerate(users):
        st = STN.Station(2 + i, name, users[name]["role"], rng.random() < 0.3, "10.0.0.%d" % i, name, _Writer())
        st.licence = gw.users.licence(name)
        st.tokens = 1e6
        srv.stations[st.id] = st
        sts.append(st)
    types = [P.T_REMOTE, P.T_MANEUVER, P.T_REMOTE, P.T_MANEUVER, P.T_HELM, P.T_ALARM_ACK, P.T_POWER_LOWER]
    for k in range(300):
        st = rng.choice(sts)
        ft = rng.choice(types)
        pl = bytes([rng.choice([0, 0, 1, 2])]) if ft in (P.T_REMOTE, P.T_MANEUVER, P.T_HELM) else b""
        st.last_seq = None if rng.random() < 0.5 else st.last_seq
        n0 = len(gw.aurix.sent)
        srv.command(st, {"sid": st.session, "seq": k + 1, "ts": time.time(), "type": ft, "p": pl.hex()})
        for t, env in gw.aurix.sent[n0:]:
            if t != P.T_GW_CMD:
                continue
            sid, ftype, body = env[0], env[4], env[5:]
            if ftype in (P.T_REMOTE, P.T_MANEUVER) and body[:1] == bytes([0]):
                who = srv.stations.get(sid)
                ok = who is not None and users[who.user].get("licence") is True
                res("I26", ok, seed, "%s asked by %s (licence %r)" % (P.TYPE_NAMES.get(ftype), who and who.user,
                                                                      who and users[who.user].get("licence")))
        if ft in (P.T_REMOTE, P.T_MANEUVER) and pl == bytes([0]) and users[st.user].get("licence") is not True:
            res("I26", len([1 for t, e in gw.aurix.sent[n0:] if t == P.T_GW_CMD]) == 0, seed,
                "unlicensed %s: forwarded" % st.user)


PARTS = {"avoid": storm_avoid, "channels": storm_channels, "events": storm_events, "vision": storm_vision,
         "licence": storm_licence}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--part", choices=list(PARTS), action="append")
    a = ap.parse_args()
    t0 = time.time()
    for name in a.part or list(PARTS):
        n = a.episodes if name != "avoid" else max(1, a.episodes // 2)
        for k in range(n):
            PARTS[name](a.seed + k)
        print("%-9s %d episodes (seeds %d..%d)" % (name, n, a.seed, a.seed + n - 1), flush=True)
    names = {"I14": "avoidance / programs never over shallow water, never across a bow",
             "I15": "STARLINK never for sync, never switched to while a normal channel works",
             "I17": "a not fully synced (or open) saved event is never deleted",
             "I20": "vision: a speed cap only, never more speed, none with STAGE 3 off",
             "I21": "vision crash / overload never releases an active cap by itself",
             "I26": "REMOTE MANUAL / MANEUVER never asked for a user without a manual licence"}
    bad = 0
    print("\n=== GATEWAY STORM (%.1f min) ===" % ((time.time() - t0) / 60.0))
    for inv in sorted(RES):
        r = RES[inv]
        bad += r["fails"]
        print("%s %-4s %-75s checks %7d  fails %d" % ("FAIL" if r["fails"] else "PASS", inv, names.get(inv, ""),
                                                     r["checks"], r["fails"]))
        if r["first"]:
            print("     first failing seed %s: %s" % r["first"])
    if CAUSES:
        print("I15 automatic switches to STARLINK while a normal channel worked, by (mode, reason): %s" % CAUSES)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
