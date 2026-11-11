"""Part 4.4, gateway side: random storms against the REAL gateway modules (not
changed), each checked by its own independent rule. Every run prints its seed.

    python tests/acceptance/storm_gateway.py [--episodes N] [--seed S] [--part avoid|channels|events|vision|licence|uav]

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
  I80 no UAV module sends anything to the AURIX but the hatch relay (the HELM
      station's T_RELAY through stations.py)                    -> gateway/uav/, uavlink.py, stations.py
  I81 never ARM without the hatch OPEN switch (the launch sequence AND the
      router's allow-list)                                      -> gateway/uav/launch.py, router_worker.py
  I82 never a LAND command except the battery rule, never RTL / SmartRTL /
      a LAND mode                                               -> gateway/uav/router_worker.py, battery.py
  I83 the captain's joystick flies a UAV only with the boat in STANDBY; no
      GW_RJOY to the boat meanwhile                            -> gateway/uav/pilot.py, stations.py
  I84 only allow-listed MAVLink leaves the router (checked by this file's own
      rules on the parsed bytes)                                -> gateway/uav/router_worker.py, allow.py
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


# --------------------------------------------------------------------- I80 - I84 (UAV magazine, step U6) --
# Independent rules (written here, NOT allow.py's): what may ever leave the router towards a drone.
UAV_OUT_NAMES = {"HEARTBEAT", "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_CLEAR_ALL", "MISSION_SET_CURRENT",
                 "MISSION_REQUEST_LIST", "MISSION_REQUEST_INT", "MISSION_ACK", "PARAM_REQUEST_READ",
                 "PARAM_REQUEST_LIST", "SET_POSITION_TARGET_LOCAL_NED", "TIMESYNC", "SYSTEM_TIME", "GPS_RTCM_DATA",
                 "GPS_INJECT_DATA", "COMMAND_LONG", "COMMAND_INT"}
UAV_CMDS = {400: "ARM/DISARM", 176: "SET_MODE", 22: "TAKEOFF", 300: "MISSION_START", 511: "MSG_INTERVAL",
            512: "REQUEST_MESSAGE", 21: "LAND", 178: "CHANGE_SPEED"}
UAV_MODES_OK = {2, 3, 4, 5, 17}                    # ALT_HOLD AUTO GUIDED LOITER BRAKE (never RTL 6 / LAND 9 / SRTL 21)
UAV_MISSION_CMDS = {16, 17, 22, 178}               # WAYPOINT LOITER_UNLIM TAKEOFF CHANGE_SPEED
# frames a storm drone / station may try (anything a gateway bug could produce)
UAV_TRY_CMDS = [400, 400, 400, 176, 176, 22, 300, 21, 21, 20, 183, 181, 211, 42600, 84, 94, 178, 511, 512, 246, 0]
UAV_TRY_MODES = [0, 2, 3, 4, 5, 6, 9, 17, 21, 1, 16]


def _uav_world():
    from uav import router_worker as rw
    return rw


class _StormLink(object):
    """A router link that is always up; frames written are kept (base attributes as router_worker.Link)."""

    def __new__(cls, rw, slot, kind):
        lk = rw.Link(slot, kind, "storm", rw.queue.Queue())
        lk.up = True
        lk.frames = []
        lk.write = lambda frame, lk=lk: lk.frames.append(frame) or True
        lk.open = lambda now: None
        lk.close = lambda why: None
        return lk


def storm_uav_router(seed):
    """I84 / I81 / I82 at the ROUTER (gateway/uav/router_worker.Router, in this process, fake links, fake drones):
    random gateway ops (commands incl. forbidden ones, modes, force ARM, missions with forbidden items, setpoints
    incl. huge / NaN, hatch switch reports, battery levels) - every byte that leaves towards a drone is parsed and
    checked by THIS file's own rules: only allow-listed messages / commands / modes / mission items (I84), an ARM only
    within 1 s of an OPEN switch report (I81), a LAND only for a drone the battery rule judged (I82), never RTL."""
    from uav import allow, mavlink
    rw = _uav_world()
    rng = random.Random(seed)
    clock = [1000.0]
    real_now, real_out = rw.now_s, rw.out
    rw.now_s = lambda: clock[0]
    events = []
    rw.out = lambda obj: events.append(obj)
    try:
        r = rw.Router()
        n_slots = rng.randint(1, 6)
        hatch = [None]                                 # when the router got the last OPEN report
        land_ok = set()                                # U6 review MED-2: slots OUR OWN battery formula says may land
        recent = {}                                    # slot -> [(t, alt, pct, amps)] what the drone told the router
        last_a = {}                                    # slot -> the last current the drone reported (A; as the router)
        drone = {}
        for s in range(1, n_slots + 1):
            for kind in ("usb", "elrs"):
                r.links[(s, kind)] = _StormLink(rw, s, kind)
            r.expect[s] = 10 + s
            r.capacity[s] = rng.choice([1500, 2500, None])
            drone[s] = {"armed": False, "alt": 0.0, "pct": rng.randint(5, 100), "mode": 0, "seq": 0}
        parsers = {k: mavlink.Parser() for k in r.links}
        # UAV-r1 (W138 b): OUR OWN record of "on the ground in its slot" (never the router's in_slot): the last
        # HEARTBEAT of the right sysid on the slot's USB link, the last altitude / vertical speed / landed state
        usb_hb, alt_rec, land_rec = {}, {}, {}

        def own_in_slot(s, now):
            a, ls = alt_rec.get(s), land_rec.get(s)
            return (now - usb_hb.get(s, -1e9) < 3.0 and a is not None and now - a[0] < 2.0 and abs(a[1]) < 0.5 and
                    abs(a[2]) < 0.3 and not (ls is not None and ls[1] in (2, 4) and now - ls[0] < 2.0))

        def drain(now):
            """Check every frame that went out since the last call (independent rules), at once: the hatch state of
            THAT moment."""
            for key, lk in r.links.items():
                frames, lk.frames = lk.frames, []
                for m in parsers[key].feed(b"".join(frames)):
                    f = m.fields
                    ok = m.name in UAV_OUT_NAMES
                    why = m.name
                    if ok and "target_system" in f and m.name not in ("HEARTBEAT", "SYSTEM_TIME", "TIMESYNC"):
                        ok, why = f["target_system"] != 0, "%s broadcast" % m.name
                    if ok and "mission_type" in f:
                        ok, why = f["mission_type"] == 0, "%s mission_type %s" % (m.name, f["mission_type"])
                    if ok and m.name in ("COMMAND_LONG", "COMMAND_INT"):
                        c = int(f.get("command", -1))
                        ok, why = c in UAV_CMDS, "command %d" % c
                        if ok and c == 176:
                            ok, why = int(f.get("param2", -1)) in UAV_MODES_OK, "mode %s" % f.get("param2")
                            res("I82", ok, seed, "slot %d: flight mode %s sent" % (key[0], f.get("param2")))
                        if ok and c == 400 and int(f.get("param1", 0)) == 1:
                            arm_ok = hatch[0] is not None and now - hatch[0] <= 1.0 + 1e-6 and \
                                f.get("param2", 0) == 0
                            res("I81", arm_ok, seed, "slot %d: ARM without a fresh OPEN switch report" % key[0])
                        if ok and c == 21:
                            res("I82", key[0] in land_ok, seed, "slot %d: LAND not by the battery rule" % key[0])
                        if ok and c == 400:
                            p2, p1 = f.get("param2", 0), int(f.get("param1", 0))
                            # UAV-r1: a force DISARM only on the ground in its slot (our own record); force ARM never
                            ok = p2 == 0 or (p2 == allow.FORCE_MAGIC and p1 == 0 and own_in_slot(key[0], now))
                            why = "force ARM / DISARM (in slot by our record: %s)" % own_in_slot(key[0], now)
                    if ok and m.name == "MISSION_ITEM_INT":
                        c = int(f.get("command", -1))
                        if c == 178:                       # DO_CHANGE_SPEED: frame MISSION, a sane speed
                            ok = int(f.get("frame", -1)) == 2 and 0.0 < f.get("param2", 0.0) <= 20.0
                        else:                              # a position item: relative frame, 0..120 m
                            ok = (c in UAV_MISSION_CMDS and int(f.get("frame", -1)) in (3, 6) and
                                  math.isfinite(f.get("z", 0.0)) and 0.0 <= f.get("z", 0.0) <= 120.0)
                        why = "mission item %d frame %s z %s" % (c, f.get("frame"), f.get("z"))
                    if ok and m.name == "SET_POSITION_TARGET_LOCAL_NED":
                        v = (f.get("vx", 0.0), f.get("vy", 0.0), f.get("vz", 0.0), f.get("yaw_rate", 0.0))
                        ok = all(math.isfinite(x) for x in v) and math.hypot(v[0], v[1]) <= 15.0 + 1e-6 and \
                            abs(v[2]) <= 3.0 + 1e-6 and abs(v[3]) <= 2.0 + 1e-6
                        why = "setpoint %s" % (v,)
                    res("I84", ok, seed, "slot %d %s: not allow-listed (%s)" % (key[0], key[1], why))

        for _step in range(rng.randint(150, 400)):
            clock[0] += rng.choice([0.05, 0.05, 0.25, 0.5, 1.2])
            now = clock[0]
            # the fake drones talk to the router (HEARTBEAT / position / battery / landed state / mission asks)
            for s, d in drone.items():
                if rng.random() < 0.15:
                    d["armed"] = not d["armed"] if rng.random() < 0.3 else d["armed"]
                    d["alt"] = max(0.0, d["alt"] + rng.uniform(-10, 15)) if d["armed"] else 0.0
                    d["pct"] = max(0, min(100, d["pct"] + rng.randint(-15, 3)))
                for kind in ("usb", "elrs"):
                    if rng.random() < 0.6:
                        lk = r.links[(s, kind)]
                        sysid = 10 + s if rng.random() < 0.9 else rng.randint(1, 250)
                        cb = rng.choice([-1, 1800, 3000])      # (cA; -1 = not reported: the last value stays)
                        msgs = [("HEARTBEAT", {"type": 2, "autopilot": 3, "base_mode": 0x80 if d["armed"] else 0,
                                               "custom_mode": d["mode"], "system_status": 4, "mavlink_version": 3}),
                                ("GLOBAL_POSITION_INT", {"relative_alt": int(d["alt"] * 1000), "lat": 378200000,
                                                         "lon": 236000000, "vz": rng.randint(-50, 50)}),
                                ("SYS_STATUS", {"battery_remaining": d["pct"], "voltage_battery": 15000,
                                                "current_battery": cb})]
                        if rng.random() < 0.3:
                            msgs.append(("EXTENDED_SYS_STATE", {"landed_state": 1 if d["alt"] < 0.3 else 2}))
                        if rng.random() < 0.4:
                            msgs.append(("MISSION_REQUEST_INT", {"seq": rng.randint(0, 6), "target_system": 255,
                                                                 "mission_type": 0}))
                        if rng.random() < 0.2:
                            msgs.append(("COMMAND_ACK", {"command": rng.choice(UAV_TRY_CMDS), "result": 0}))
                        if rng.random() < 0.1:
                            msgs.append(("MISSION_ACK", {"type": 0, "mission_type": 0}))
                        if rng.random() < 0.05:
                            msgs.append(("PARAM_SET", {"param_value": 1.0}))       # never allow-listed in
                        data = b"".join(mavlink.encode(n, f, d["seq"] + i, sysid, 1) for i, (n, f) in enumerate(msgs))
                        if sysid == 10 + s:
                            if kind == "usb":
                                usb_hb[s] = now
                            gpi = msgs[1][1]
                            alt_rec[s] = (now, gpi["relative_alt"] / 1000.0, -gpi["vz"] / 100.0)
                            for nm, ff in msgs:
                                if nm == "EXTENDED_SYS_STATE":
                                    land_rec[s] = (now, ff["landed_state"])
                            if cb >= 0:
                                last_a[s] = cb / 100.0
                            recent.setdefault(s, []).append((now, d["alt"], d["pct"], last_a.get(s)))
                        d["seq"] = (d["seq"] + len(msgs)) & 0xFF
                        r.rx(lk, data, now)
                        drain(now)
            # random gateway ops
            for _k in range(rng.randint(0, 3)):
                s = rng.randint(1, n_slots)
                via = rng.choice(["usb", "elrs", "auto"])
                op = rng.random()
                if op < 0.4:
                    c = rng.choice(UAV_TRY_CMDS)
                    p = [rng.choice([0, 1, 1, 2]), rng.choice([0, 0, 0, allow.FORCE_MAGIC, 1, 21196.0]),
                         rng.choice([0, 1]), 0, 0, 0, rng.choice([10.0, 50.0, 200.0, float("nan"), 0.0])]
                    if c == 176:
                        p[0], p[1] = rng.choice([1, 1, 0]), rng.choice(UAV_TRY_MODES)
                    r.command({"op": "cmd", "slot": s, "id": _step, "via": via, "command": c, "params": p}, now)
                elif op < 0.55:
                    items = []
                    for q in range(rng.randint(1, 6)):
                        items.append({"seq": q, "command": rng.choice([16, 16, 17, 22, 178, 183, 211, 94, 21, 20]),
                                      "frame": rng.choice([3, 6, 2, 0, 1]), "x": 378200000, "y": 236000000,
                                      "z": rng.choice([50.0, 30.0, 500.0, float("inf")]), "param1": 0.0,
                                      "param2": rng.choice([5.0, 50.0]), "param3": 0.0, "param4": 0.0,
                                      "current": 0, "autocontinue": 1, "mission_type": rng.choice([0, 0, 0, 1])})
                    r.command({"op": "mission", "slot": s, "id": _step, "via": via, "items": items}, now)
                elif op < 0.75:
                    r.command({"op": "ctl", "slot": s, "vx": rng.choice([0.0, 3.0, 40.0, float("nan")]),
                               "vy": rng.uniform(-20, 20), "vz": rng.choice([0.0, 1.0, -9.0]),
                               "yaw_rate": rng.uniform(-3, 3)}, now)
                elif op < 0.9:
                    is_open = rng.random() < 0.5
                    r.command({"op": "hatch", "open": is_open}, now)
                    hatch[0] = now if is_open else None
                elif op < 0.95:
                    r.command({"op": "set_current", "slot": s, "id": _step, "via": via, "seq": rng.randint(0, 5)}, now)
                else:
                    r.command({"op": "cancel", "slot": s}, now)
                drain(now)
            # our own record of what the battery rule may land: battery_tick latches battery_land itself
            r.jobs_tick(now)
            r.ctl_tick(now)
            r.battery_tick(now)
            for s in drone:
                recent[s] = [x for x in recent.get(s, []) if now - x[0] <= 6.0]
                if any(_own_land(alt, pct, r.capacity.get(s), amps) for _t, alt, pct, amps in recent[s]):
                    land_ok.add(s)                     # (latched: a landing goes on, LAND repeated every 2 s)
            drain(now)
    finally:
        rw.now_s, rw.out = real_now, real_out


def _own_land(alt, pct, capacity_mah, amps=None):
    """U6 review MED-2 / WA+U6 review 2 MED: the storm's OWN battery-minimum formula (never battery.judge): the energy
    to descend from alt in the two stages (the last uav_land_final_m at uav_land_final_ms, the rest at uav_land_ms)
    with the current the drone itself reported in that frame (the last one it sent, as the router keeps it; at least
    the hover current) + the margin, vs the pack left, with a 1.2 x tolerance (rounding / the router holding a value
    from a neighbouring frame of the recorded window). A router that lands 2 x early or more fails I82.
    True = a LAND is allowed for this drone."""
    from uav import uavcheck
    c = uavcheck.DEFAULTS
    cap = capacity_mah or float(c["uav_capacity_mah"])
    hover = float(c["uav_hover_a"])
    draw = max(hover, amps) if amps is not None and amps > 0 else hover
    a = max(0.0, alt)
    low = min(a, float(c.get("uav_land_final_m", 0.0)))
    t_down = (a - low) / max(0.1, float(c["uav_land_ms"])) + low / max(0.1, float(c.get("uav_land_final_ms") or
                                                                                   c["uav_land_ms"]))
    reserve = draw * t_down / 3.6 * (1.0 + float(c["uav_land_margin_pct"]) / 100.0)
    return cap * max(0, min(100, pct)) / 100.0 <= 1.2 * reserve


class _UStation(object):
    def __init__(self, sid, crew, single=False):
        self.id, self.name, self.crew, self.single_op = sid, "s%d" % sid, crew, single
        self.role = "HELM"
        self.closed = False
        self.sent = []

    def send(self, m):
        self.sent.append(m)


def _cap_like(st, gw):
    """U6 review L5: the storm's OWN "this joystick drives the boat" (never pilot.captain_like): the crew captain, a
    single operator or the HELM holder in TELEM2."""
    return st is not None and (st.crew == "captain" or bool(st.single_op) or (gw.telem2 or {}).get("helm") == st.id)


def storm_uav_pilot(seed):
    """I83 + I80 with the REAL pilot lock (gateway/uav/pilot.Pilots) and the REAL StationServer._rjoy: random boat
    modes (fresh / stale telemetry), HELM moving, captain / copilot / single operator takes, gives, uav_ctl and rjoy
    packets. Checked: after every 4 Hz tick no captain-like station flies a UAV unless the boat is in STANDBY (I83),
    no moving setpoint of a captain-like pilot goes out while the boat is not in STANDBY (I83), a station flying a
    UAV never gets a GW_RJOY to the boat (I83), and nothing reaches the AURIX but the rjoy of a station that does NOT
    fly (I80: the UAV code sends nothing to the boat)."""
    import stations as STN
    import proto as P
    from uav import pilot
    rng = random.Random(seed)
    clock = [1.7e9]
    real_time = time.time
    time.time = lambda: clock[0]
    try:
        gw = type("G", (), {})()
        gw.telem2, gw.mode = {}, P.MODE_STANDBY
        gw.aurix = _Aurix()
        gw.cfg, gw.telem = {}, {}
        gw.audit = lambda *a, **k: None
        gw.telem_fresh = lambda: None if gw.mode is None else {"mode": gw.mode}
        gw.storage = None
        gw.uav = type("U", (), {})()
        srv = STN.StationServer(gw, {"rate_per_s": 1e6, "rate_burst": 1e6})
        gw.stations = srv
        ops = []

        class Mgr(object):
            enabled, n, why, airspace = True, 6, "", None

            def __init__(self):
                self.gw = gw
                self.ccfg = {"uav_min_alt_m": 10.0, "uav_max_alt_m": 120.0}

            def slot(self, k):
                return {"hb_age": 0.2, "armed": True, "landed": False, "usb_present": False, "landing": False,
                        "lat": 37.83, "lon": 23.61, "alt": 50.0, "hdg": 90.0, "alt_age": 0.2, "gps_fix": 3}

            drone_view = slot

            def router_ok(self):
                return True

            def to_worker(self, obj):
                ops.append(obj)
                return True

            def cmd(self, k, via, command, params, cb):
                ops.append({"op": "cmd", "slot": k, "command": command})
                cb(0, "")
        P_ = gw.uav.pilots = pilot.Pilots(Mgr())
        sts = []
        for i, (crew, single) in enumerate((("captain", False), ("copilot", False), ("copilot", False),
                                            ("copilot", rng.random() < 0.3))):
            st = STN.Station(2 + i, "u%d" % i, "HELM", False, "10.0.0.%d" % i, "s%d" % i, _Writer())
            st.crew, st.single_op = crew, single
            st.tokens = 1e6
            srv.stations[st.id] = st
            sts.append(st)
        gw.telem2["helm"] = sts[0].id
        seqs = {st.id: 0 for st in sts}
        for _k in range(rng.randint(200, 600)):
            clock[0] += rng.choice([0.05, 0.1, 0.25])
            r = rng.random()
            if r < 0.06:
                gw.mode = rng.choice([P.MODE_STANDBY, P.MODE_STANDBY, P.MODE_HDG_LOCK, P.MODE_AUTO, P.MODE_REMOTE,
                                      P.MODE_MANEUVER, P.MODE_DP, None])
            elif r < 0.08:
                gw.telem2["helm"] = rng.choice([s.id for s in sts] + [None])
            st = rng.choice(sts)
            n = rng.randint(1, 3)
            a = rng.random()
            n0 = len(gw.aurix.sent)
            flying_before = P_.slot_of(st.id) is not None
            if a < 0.08:
                P_.pilot_msg(st, {"uav": n, "take": rng.random() < 0.8})
            elif a < 0.6:
                seqs[st.id] += 1
                P_.ctl(st, {"uav": n, "seq": seqs[st.id], "ts": clock[0] - rng.choice([0.0, 0.0, 0.2, 1.0]),
                            "x": rng.choice([0.0, 0.0, 0.8, -1.0]), "y": rng.uniform(-1, 1), "z": rng.uniform(-1, 1),
                            "r": 0.0})
            else:
                seqs[st.id] += 1
                srv._rjoy(st, {"seq": seqs[st.id], "ts": clock[0], "x": rng.uniform(-1, 1), "y": 0.0, "lever": 0.5})
                got = [t for t, _p in gw.aurix.sent[n0:] if t == P.T_GW_RJOY]
                if flying_before:
                    res("I83", not got, seed, "station %d flies UAV %s but its rjoy reached the boat" % (
                        st.id, P_.slot_of(st.id)))
            for t, _p in gw.aurix.sent[n0:]:
                res("I80", t == P.T_GW_RJOY and not flying_before, seed,
                    "frame 0x%02X to the AURIX from a UAV path" % t)
            if rng.random() < 0.3:
                P_.tick(clock[0])
                standby = gw.mode == P.MODE_STANDBY
                for slot, lk in P_.locks.items():
                    holder = srv.stations.get(lk.sid)
                    res("I83", standby or not _cap_like(holder, gw), seed,
                        "captain-like station %s flies UAV %d with the boat in mode %s after a tick" % (
                            lk.sid, slot, gw.mode))
            for o in ops:
                if o.get("op") == "ctl" and any(o.get(k, 0.0) for k in ("vx", "vy", "vz", "yaw_rate")):
                    lk = P_.locks.get(o["slot"])
                    holder = srv.stations.get(lk.sid) if lk else None
                    if holder is not None and _cap_like(holder, gw):
                        res("I83", gw.mode == P.MODE_STANDBY, seed,
                            "a moving setpoint of the captain's joystick with the boat in mode %s" % gw.mode)
            del ops[:]
    finally:
        time.time = real_time


class _LaunchIo(object):
    """launch.Launch's io (like the gateway's manager) with a random magazine; the hatch relay goes through the REAL
    uavlink.hatch_relay -> StationServer.uav_relay (a fake AURIX link records every frame)."""

    def __init__(self, rng, srv, gw, relay_n):
        self.rng, self.srv, self.gw, self.relay_n = rng, srv, gw, relay_n
        self.t = 0.0
        n = rng.randint(1, 6)
        self.drones = {s: {"usb_present": True, "battery_pct": rng.choice([95, 90, 40]), "gps_fix": 1,
                           "ekf_flags": 0x07, "rel_alt": 0.0, "alt_age": 0.1, "landed": True, "armed": False}
                       for s in range(1, n + 1)}
        self.boat_d = {"launch_why": [], "standby": True, "pos": (37.82, 23.60)}
        self.hatch_st, self.cmd_open, self.hatch_t = "CLOSED", False, 0.0
        self.fault = rng.random() < 0.3
        self.q, self.sent = [], []

    def now(self):
        return self.t

    def boat(self):
        return self.boat_d

    def hatch_state(self):
        return self.hatch_st

    def hatch_why(self):
        return "storm"

    def hatch_open_s(self):
        return 3.0

    def hatch_commanded_open(self):
        return self.cmd_open

    def captain_ok(self, sid):
        return self.rng.random() > 0.002

    def router_ok(self):
        return self.rng.random() > 0.002

    def slot(self, n):
        return dict(self.drones[n])

    def next_slot(self):
        return next((s for s, d in sorted(self.drones.items()) if d["usb_present"]), None)

    def check_drone(self, n):
        return None if self.rng.random() > 0.01 else "storm: a check failed"

    def mission_items(self, n, pos, take):
        return ([{"seq": 0}], "") if self.rng.random() > 0.02 else (None, "storm: route refused")

    def upload(self, n, items, via, cb):
        ok = self.rng.random() > 0.05
        self.q.append((self.t + self.rng.uniform(0.2, 3.0), lambda: cb(ok, "" if ok else "storm")))

    def mission_loaded(self, n):
        pass

    def cmd(self, n, via, command, params, cb):
        p = (list(params) + [0] * 7)[:7]
        self.sent.append((self.t, n, command, p, self.hatch_st))
        d = self.drones[n]
        if command == 400 and p[0] == 1:
            d["armed"] = True
        elif command == 400:
            d["armed"] = False
        res_ = 0 if self.rng.random() > 0.05 else 4
        if self.rng.random() > 0.02:
            self.q.append((self.t + self.rng.uniform(0.1, 2.0), lambda: cb(res_, "")))

    def cancel(self, n, command):
        pass

    def armed(self, n):
        pass

    def on_mission(self, n):
        pass

    def catchup(self, n):
        pass

    def hatch_cmd(self, on, cb):
        import uavlink
        self.cmd_open, self.hatch_t = on, self.t
        n0 = len(self.gw.aurix.sent)
        answered = []
        uavlink.hatch_relay(self.gw, self.relay_n, on, lambda r, w: answered.append((r, w)))
        sent = self.gw.aurix.sent[n0:]
        if answered and answered[0][0] is None:
            return cb(False, answered[0][1])
        if self.rng.random() > 0.03:
            self.q.append((self.t + 0.3, lambda: cb(bool(sent), "")))

    def notice(self, text, level="info"):
        pass

    def world(self, dt):
        if self.fault and self.rng.random() < 0.01:
            self.hatch_st = "FAULT"
        elif self.hatch_st != "FAULT":
            if self.cmd_open and self.hatch_st in ("CLOSED", "CLOSING") and self.t - self.hatch_t >= 2.0:
                self.hatch_st = "OPEN"
            elif not self.cmd_open and self.hatch_st == "OPEN" and self.t - self.hatch_t >= 2.0:
                self.hatch_st = "CLOSED"
        if self.rng.random() < 0.004:
            self.boat_d["launch_why"] = ["storm: the boat left STANDBY"]
            self.boat_d["standby"] = False
        for s, d in self.drones.items():
            if self.hatch_st == "OPEN" and d["usb_present"] and self.rng.random() < 0.3:
                d["gps_fix"], d["ekf_flags"] = 3, 0x1F
            if d["armed"] and self.rng.random() < 0.9:
                d["rel_alt"] += 2.5 * dt
                d["landed"] = d["rel_alt"] < 0.3
                if d["rel_alt"] > 0.3:
                    d["usb_present"] = False


def storm_uav_launch(seed):
    """I81 / I82 / I80 at the LAUNCH SEQUENCE (the real gateway/uav/launch.py): random magazines, switch faults,
    lost answers, refused commands, the boat leaving STANDBY, stops at random moments. Checked: every ARM the
    sequence asks for happens with the hatch's OPEN switch (I81); it never asks for LAND / RTL / SmartRTL / a LAND
    mode (I82); the only frames to the AURIX are the HELM station's T_RELAY of the hatch relay (I80)."""
    import stations as STN
    import proto as P
    from uav import allow, launch
    rng = random.Random(seed)
    gw = type("G", (), {})()
    gw.telem2, gw.telem, gw.cfg = {}, {}, {}
    gw.aurix = _Aurix()
    gw.audit = lambda *a, **k: None
    srv = STN.StationServer(gw, {"rate_per_s": 1e6, "rate_burst": 1e6})
    gw.stations = srv
    st = STN.Station(2, "cap", "HELM", False, "10.0.0.2", "cap", _Writer())
    st.crew = "captain"
    srv.stations[st.id] = st
    gw.telem2["helm"] = st.id
    srv.helm_grant = st.id
    relay_n = rng.randint(1, 16)
    io = _LaunchIo(rng, srv, gw, relay_n)
    seq = launch.Launch(io)
    seq.prepare(sorted(io.drones), st.id, io.t)
    seq.approved(rng.random() > 0.05, st.id, io.t)
    stop_at = rng.uniform(0, 200) if rng.random() < 0.3 else None
    t_stopped = None
    while io.t < 400:
        if t_stopped is None and seq.state == launch.STOPPED:
            t_stopped = io.t
        if stop_at is not None and io.t >= stop_at:
            seq.stop("storm STOP", io.t)
            stop_at = None
        seq.tick(io.t)
        due = [x for x in io.q if x[0] <= io.t]
        io.q = [x for x in io.q if x[0] > io.t]
        for _t, fn in due:
            fn()
        io.world(0.25)
        io.t += 0.25
        if seq.state in (launch.DONE, launch.STOPPED) and not io.q:
            break
    armed_order = []
    for t, n, c, p, hs in io.sent:
        if c == allow.CMD_COMPONENT_ARM_DISARM and p[0] == 1:
            res("I81", hs == "OPEN" and p[1] == 0, seed, "UAV %d ARM asked at t=%.1f with the hatch %s" % (n, t, hs))
            # U6 review L6: strictly in series from the top, each drone once, never after the sequence STOPPED
            res("I81", n not in armed_order and (not armed_order or n > armed_order[-1]), seed,
                "UAV %d ARM out of order / twice (before: %s)" % (n, armed_order))
            res("I81", t_stopped is None or t < t_stopped, seed, "UAV %d ARM at t=%.1f after the STOP" % (n, t))
            armed_order.append(n)
        res("I82", c != allow.CMD_NAV_LAND and not (c == allow.CMD_DO_SET_MODE and int(p[1]) in (6, 9, 21)),
            seed, "UAV %d: the launch asked for command %d %s" % (n, c, p[:2]))
    for t, env in gw.aurix.sent:
        ok = t == P.T_GW_CMD and env[4] == P.T_RELAY and env[5] == relay_n and env[0] == st.id
        res("I80", ok, seed, "frame 0x%02X %s to the AURIX from the launch (hatch relay %d)" % (t, bytes(env[:7]).hex(),
                                                                                         relay_n))
    if not gw.aurix.sent:
        res("I80", True)


def storm_uav_static(seed):
    """I80 (static, once): no gateway/uav module imports aurix / proto / stations or names the AURIX link; I84
    (static): mavlink.encode is called only by the router's tx() (after allow.check_out)."""
    import ast
    folder = os.path.join(REPO, "gateway", "uav")
    for f in sorted(os.listdir(folder)):
        if not f.endswith(".py"):
            continue
        src = open(os.path.join(folder, f), encoding="utf-8").read()
        tree = ast.parse(src)
        names = set()
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                names |= {a.name.split(".")[0] for a in n.names}
            elif isinstance(n, ast.ImportFrom):
                # U6 review L6: relative imports (from .. import aurix / from ..stations import x) count too
                if n.module:
                    names |= set(n.module.split("."))
                if n.level:
                    names |= {a.name.split(".")[0] for a in n.names}
            elif isinstance(n, ast.Attribute) and n.attr in ("aurix", "send_frame"):
                names.add(".aurix")
            elif isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value == "aurix":
                names.add(".aurix")                    # getattr(x, "aurix") / importlib of a boat module
        res("I80", not names & {"aurix", "proto", "stations", ".aurix"}, seed, "gateway/uav/%s uses %s" % (
            f, sorted(names & {"aurix", "proto", "stations", ".aurix"})))
        enc = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and
               n.func.attr == "encode" and isinstance(n.func.value, ast.Name) and n.func.value.id == "mavlink"]
        res("I84", f in ("router_worker.py", "simdrone.py") or not enc, seed,
            "gateway/uav/%s builds MAVLink frames itself" % f)
        if f == "router_worker.py":
            res("I84", len(enc) == 1 and "allow.check_out" in src.split("def tx(")[1].split("\n    def ")[0], seed,
                "router_worker: mavlink.encode outside tx() / without allow.check_out")


def storm_uav(seed):
    if seed == 1 or not RES.get("I80"):
        storm_uav_static(seed)
    storm_uav_router(seed)
    storm_uav_pilot(seed)
    storm_uav_launch(seed)


PARTS = {"avoid": storm_avoid, "channels": storm_channels, "events": storm_events, "vision": storm_vision,
         "licence": storm_licence, "uav": storm_uav}


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
             "I26": "REMOTE MANUAL / MANEUVER never asked for a user without a manual licence",
             "I80": "UAV: nothing to the AURIX but the hatch relay (HELM station T_RELAY)",
             "I81": "UAV: never ARM without the hatch OPEN switch",
             "I82": "UAV: never LAND except the battery rule, never RTL / SmartRTL",
             "I83": "UAV: captain flies only in STANDBY, no GW_RJOY to the boat meanwhile",
             "I84": "UAV: only allow-listed MAVLink leaves the router"}
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
