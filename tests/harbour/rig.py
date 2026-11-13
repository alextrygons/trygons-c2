"""HX7: the END-TO-END rig for the harbour scenario library (docs/harbour_design.md 9.1).

ONE episode = the REAL autopilot_sim (UNCREWED boat, `server <port> 1 nojoy`: real time, the same code as the AURIX) +
the REAL gateway in this process (harbour sensors = the HX3 simulated radar + LiDAR in their own worker processes, the
HX4 tracker, the HX5 planner, the harbour manager, the HX6 route check) + the captain's station (alice, HELM, the shore
stick sender for the CONFIRM). The simulated world (gateway/harbour/simworld.py objects) is placed around the
simulator's start position (60.0, 25.0) with an ABSOLUTE t0, so the objects move the same way every run (deterministic
world; the sensors' noise has a fixed seed per worker).

Nothing here changes app/ hal/ or the planner / tracker: the rig only builds the world, sends the captain's requests
(the station's own frames) and WATCHES (telemetry, the planner's view, the world truth).

Run only through run_scenarios.py (bounded time, kill-on-close job, strays check)."""
import json
import math
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
GW = os.path.join(REPO, "gateway")
sys.path.insert(0, GW)
sys.path.insert(0, os.path.join(REPO, "groundstation"))
sys.path.insert(0, os.path.join(REPO, "tools"))
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")
os.environ.setdefault("TRYGONS_TEST", "1")

import test_gateway as TG  # noqa: E402 - the sim / gateway runners of the gateway tests (test tooling, not changed)
from test_gateway import Sim, GwRunner, Client, gw_cfg_b, sim_chart, PW, SIM_LAT, SIM_LON  # noqa: E402
from common import proto  # noqa: E402
import program as progmod  # noqa: E402
from shorejoy import RemoteSender  # noqa: E402
from harbour import simworld  # noqa: E402

KN = 0.514444
M_PER_DEG = 111320.0


def ll(x, y):
    return simworld.xy_to_ll(SIM_LAT, SIM_LON, x, y)


def xy(lat, lon):
    return simworld.ll_to_xy(SIM_LAT, SIM_LON, lat, lon)


def seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


OWN_R = 3.5           # the own hull as a circle (length 7 m): gaps are centre distance - radii
STATIC_AHEAD_M = 130.0 # HX7-fix4: the band ahead searched for the static-map cells of a STOP (>= the planner horizon)


def hull_gap(ox, oy, o):
    """Hull-to-hull gap (m) between the own boat (circle OWN_R) and one truth object (simworld.World.truth item)."""
    if o["kind"] == "pier":
        return seg_dist(ox, oy, o["x1"], o["y1"], o["x2"], o["y2"]) - o["w"] / 2.0 - OWN_R
    return math.hypot(ox - o["x"], oy - o["y"]) - o["len"] / 2.0 - OWN_R        # (review: the conservative circle)


_PORTS = [0]


def free_port():
    """Ports of THIS process come from its own block (parallel episodes never meet on a port): the gateway test's
    free_port() takes any free one - two children could pick p and p + 2 of each other."""
    base = 20000 + (os.getpid() % 400) * 100
    while True:
        _PORTS[0] += 4
        p = base + _PORTS[0] % 96
        ok = True
        for q in (p, p + 1, p + 2):
            sk = TG.socket.socket()
            try:
                sk.bind(("127.0.0.1", q))
            except OSError:
                ok = False
            finally:
                sk.close()
        if ok:
            return p


TG.free_port = free_port


class Truth:
    """HX7-fix3 (owner answer 1): the simulator's TRUE boat position (sim_server.c TRYGONS_SIM_TRUTH, a line every
    100 ms of simulated time: t_ms lat lon heading kn). latest() = (x, y, hdg, kn) or None when no line arrived in the
    last TRUTH_STALE_S of wall time (never an old value as the truth)."""
    TRUTH_STALE_S = 1.0

    def __init__(self, path):
        self.path = path
        self.f = None
        self.buf = ""
        self.last = None
        self.t_rx = 0.0

    def latest(self):
        if self.f is None:
            try:
                self.f = open(self.path, "r", encoding="ascii")
            except OSError:
                return None
        chunk = self.f.read()
        if chunk:
            self.buf += chunk
            lines = self.buf.split("\n")
            self.buf = lines.pop()
            for ln in reversed(lines):
                p = ln.split()
                if len(p) == 5:
                    try:
                        x, y = xy(float(p[1]), float(p[2]))
                        self.last = (x, y, float(p[3]), float(p[4]))
                        self.t_rx = time.time()
                        break
                    except ValueError:
                        continue
        if self.last is None or time.time() - self.t_rx > self.TRUTH_STALE_S:
            return None
        return self.last

    def close(self):
        if self.f is not None:
            self.f.close()
            self.f = None


class RigSim(Sim):
    """autopilot_sim in its OWN working folder (the nv / sd files of parallel episodes never meet)."""

    def __init__(self, cwd, speed=10.0, *args):
        self.port = free_port()
        self.speed = speed
        self.out = open(os.path.join(cwd, "sim.log"), "w")
        # HX7-fix3 (owner answer 1): the simulator writes its world's TRUE boat position (100 ms) to this file
        self.truth_path = os.path.join(cwd, "truth.txt")
        env = dict(os.environ, TRYGONS_SIM_TRUTH=self.truth_path)
        self.p = TG.subprocess.Popen([TG.EXE, "server", str(self.port), str(speed)] + list(args), cwd=cwd,
                                     stdout=self.out, stderr=TG.subprocess.STDOUT, stdin=TG.subprocess.DEVNULL,
                                     env=env)
        TG._SIMS.append(self)
        for _ in range(100):
            try:
                TG.socket.create_connection(("127.0.0.1", self.port + 2), timeout=0.2).close()
                break
            except OSError:
                time.sleep(0.05)
        time.sleep(0.3)


class _Station:
    """The minimum of the ground station that App.harbour_start / App.harbour_state use: the REAL GatewayLink (alice),
    the gateway's gw_status + harbour_view messages as the station receives them, cmd() = the station's send. The
    handler code itself is the station's (groundstation/app.py), unchanged."""

    def __init__(self, ep):
        self.ep = ep
        self.cl = ep.alice
        self.link = ep.alice.l
        self.gw_status = None
        self.harbour_view = None
        self.harbour_model = None
        self.harbour_route = None
        self.single_op = False
        self.root = None
        self.sent = []
        self.seqs = []

    def pump(self, t):
        end = time.time() + t
        while time.time() < end:
            try:
                kind, d = self.link.events.get(timeout=max(0.0, end - time.time()))
            except Exception:
                break
            if kind == "gw_status":
                self.gw_status = d
            elif kind == "harbour_view":
                d["_rx"] = time.time()
                self.harbour_view = d
            elif kind == "ack":
                self.cl.acks.append(d)
            elif kind == "telemetry":
                self.cl.telem = d

    def harbour_state(self):
        import app as stapp
        return stapp.App.harbour_state(self)

    def cmd(self, ftype, payload=b"", what=""):
        self.sent.append((ftype, bytes(payload)))
        self.seqs.append(self.link.send(ftype, payload))

    def wait_start(self, timeout):
        if not self.seqs:
            return None
        return self.cl.wait_ack(self.seqs[-1], timeout)

    def say(self, *a, **k):
        pass

    def event(self, *a, **k):
        pass


class Episode:
    """One end-to-end episode. spec = {objects, route [(x, y, kn)], max_kn, corr_m, secs, faults [(t_s, what)], ...}."""

    def __init__(self, spec, log=print):
        self.spec = spec
        self.log = log
        self.tmp = tempfile.mkdtemp(prefix="trygons_hx7_")
        self.sim = self.gw = self.alice = self.snd = self.link = self.w = None
        self.samples = []               # (t_rel, x, y, hdg, sog, state, xte, mode, min_gap, cap)
        self.events = []
        self.t0 = None
        self.stick = {"x": 0.0, "lever": 0.5, "confirm": False}
        self.resumes = 0
        self.last_resume = 0.0
        self.truth = None               # HX7-fix3: the simulator's true position (Truth)
        self.stops = []                 # HX7-fix3: every planner STOP with its blockers matched to the world truth

    # -------------------------------------------------------------- set-up
    def setup(self):
        s = self.spec
        self.t_world = time.time() + s.get("world_delay_s", 55.0)
        wf = os.path.join(self.tmp, "world.json")
        with open(wf, "w", encoding="utf-8") as f:
            json.dump({"origin": [SIM_LAT, SIM_LON], "t0": self.t_world, "objects": s["objects"]}, f)
        self.world = simworld.World.load(wf)
        os.makedirs(os.path.join(self.tmp, "simcwd"))
        self.sim = RigSim(os.path.join(self.tmp, "simcwd"), 1.0, "nojoy")
        self.truth = Truth(self.sim.truth_path)
        self.link, self.w = self.sim.station()
        root = os.path.join(self.tmp, "ssd")
        os.makedirs(root)
        charts = os.path.join(self.tmp, "charts")
        sim_chart(charts)
        sens = {"enabled": True, "driver": "sim"}
        hc = {"enabled": True, "sim": True, "world": wf, "corr_m": s.get("corr_m", 10.0),
              "radar": dict(sens, **s.get("radar", {})), "lidar": dict(sens, **s.get("lidar", {}))}
        if s.get("harbour_extra"):
            hc.update(s["harbour_extra"])
        cfg = gw_cfg_b(self.sim, root, charts_dir=charts, harbour=hc,
                       shallow={"draft_m": 0.8, "margin_m": 1.0, "shallow_m": 3.0, "clearance_m": 50.0})
        self.gw = GwRunner(cfg)
        self.g = self.gw.gw
        sp = cfg["stations"]["port"]
        self.w.until(lambda d: d.get("profile") == proto.PROFILE_UNCREWED, 15)
        if not self.w.until(lambda d: d.get("pos_source") == 1 or d.get("gps_fix"), 25):
            raise RuntimeError("no fresh position from the simulated AURIX in 25 s")
        self.alice = Client(sp, "alice", PW["alice"], name="alice")
        self.alice.pump(1.0)
        self.snd = RemoteSender(self.alice.l, lambda: dict(self.stick))
        self.snd.start()
        self.alice.cmd(proto.T_HELM, proto.p_helm(0))
        time.sleep(0.5)
        self.alice.cmd(proto.T_ALARM_ACK)

    def upload_route(self):
        s = self.spec
        wps = []
        for (x, y, kn) in s["route"]:
            la, lo = ll(x, y)
            wps.append({"lat": la, "lon": lo, "kn": kn, "radius": 8, "action": 0})
        prog = {"final": 1, "flags": 1, "wps": wps}        # final 1 = STOP at the end, flags b0 = harbour route
        frames = progmod.upload_frames(prog)
        for t, p in frames[:-1]:
            self.alice.l.send(t, p)
        self.alice.pump(0.3)
        a = None
        for _ in range(3):
            a = self.alice.cmd(frames[-1][0], frames[-1][1], 12.0)
            if a is not None:
                break
        return a

    def start_harbour(self):
        """START through the STATION'S OWN BUTTON PATH (HX7-fix, finding HX7-1 fixed): the station's model of the
        gateway's real harbour_view (groundstation/harbourview.model) must enable the START button (`entry_ok` = the
        planner's READY lease reached the AURIX: no earlier START needed), then the button's handler App.harbour_start
        runs on a thin station object (the real GatewayLink of alice, the gateway's own gw_status; the captain's
        vertical YES answered YES) and sends the ONE T_HARBOUR START; then the shore stick's CONFIRM. ONE press."""
        import app as stapp
        import ynbox
        self.alice.l.send_msg({"t": "harbour_view_on", "on": True})
        st = _Station(self)
        end = time.time() + 15.0
        m = {}
        while time.time() < end:
            st.pump(0.1)
            m = st.harbour_state() or {}
            if bool(st.link) and m.get("entry_ok"):          # = the panel's START enable (app.panel_harbour)
                break
        self.entry_before = bool(m.get("entry_ok"))
        self.first_start = None
        if not self.entry_before:
            dec = self.g.hplan.dec or {}
            self.refusal = {"stop": bool(dec.get("stop")), "reasons": list(dec.get("reasons") or [])[:3],
                            "why": dec.get("why"), "not_yielding": list(dec.get("not_yielding") or [])}
            return {"result": 6 if dec.get("stop") else -1, "why": "the START button never became enabled"}
        old = ynbox.ask
        ynbox.ask = lambda *a, **k: True                     # the captain's own YES
        try:
            stapp.App.harbour_start(st)
        finally:
            ynbox.ask = old
        a = st.wait_start(12.0)
        self.first_start = None if a is None else a.get("result")
        self.presses = st.sent
        # the AURIX asks the shore joystick's CONFIRM (UNCREWED): stick centred + the CONFIRM button
        self.stick["confirm"] = True
        time.sleep(0.6)
        self.stick["confirm"] = False
        return a

    def stat(self):
        return getattr(self.g.hplan, "stat", None) or {}

    def pose(self):
        o = self.g.own()
        if not o or o.get("lat") is None:
            return None
        x, y = xy(o["lat"], o["lon"])
        return x, y, float(o.get("heading") or 0.0), float(o.get("sog") or 0.0)

    # -------------------------------------------------------------- the run
    def run(self):
        s = self.spec
        out = {"name": s.get("name"), "ok": False, "why": ""}
        try:
            self.setup()
            ra = self.upload_route()
            out["upload"] = None if ra is None else ra.get("result")
            # wait for the world's t0 (objects start moving at t0) - the boat is ready long before
            wait = self.t_world - time.time() - s.get("lead_s", 4.0)
            if wait > 0:
                end = time.time() + wait
                while time.time() < end:
                    self.alice.pump(0.2)
            # the kind-4 planner window + START
            ts = self.start_harbour()
            out["start"] = None if ts is None else ts.get("result")
            out["refusal"] = getattr(self, "refusal", None)
            self.t0 = time.time()
            self.watch(s.get("secs", 90.0), out)
            out["ok"] = True
        except Exception as e:                      # a rig error is NOT a pass
            import traceback
            out["why"] = "rig error: %r" % (e,)
            out["tb"] = traceback.format_exc()
        return out

    def faults_due(self, t):
        s = self.spec
        for f in s.get("faults", []):
            if not f.get("done") and t >= f["t"]:
                f["done"] = True
                self.events.append((round(t, 1), "fault " + f["what"]))
                getattr(self, "fault_" + f["what"])(f)

    # fault injectors (all through the real components' own interfaces)
    def fault_radar_off(self, f):
        self.g.harbour.procs["radar"].proc.kill()
        self.g.harbour.procs["radar"].next_start = time.monotonic() + 1e9     # no restart: stays LOST

    def fault_lidar_off(self, f):
        self.g.harbour.procs["lidar"].proc.kill()
        self.g.harbour.procs["lidar"].next_start = time.monotonic() + 1e9

    def fault_link_lost(self, f):
        self.snd.stop()
        self.alice.close()
        self.alice = None

    def fault_standby(self, f):
        self.alice.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))

    def fault_gps_spoof(self, f):
        self.w.command(proto.T_SIM_FAULT, proto.p_sim_fault("gps_spoof", True), 3.0)

    def fault_camera_ghost(self, f):
        """A camera obstacle 70 m dead ahead that no radar / LiDAR confirms (injected into the gateway's vision list;
        the tracker takes it as camera-only: never steer_ok)."""
        self.g.vision.obstacles = [{"rel": 0.0, "dist": 70.0, "range": [50.0, 90.0], "conf": 0.8, "confirmed": True}]

    def fault_ais_ghost(self, f):
        """An AIS target 90 m ahead of the start that no radar / LiDAR sees (a ghost / a spoofed AIS)."""
        la, lo = ll(120.0, 0.0)
        tg = {"mmsi": 999000001, "lat": la, "lon": lo, "sog": 0.0, "cog": 0.0, "length": 10.0, "beam": 3.0,
              "stale": False}
        # HX7-fix2: only the target LIST is replaced - the earlier SimpleNamespace had no tick() / silent_for() /
        # status(): the gateway's status loop died on it, GW_STATUS stopped and the AURIX did its lost-link STANDBY
        # (H18 "left harbour mode at 11 s" = this rig defect, not the planner)
        real = self.g.ais.list
        self.g.ais.list = lambda: [dict(t) for t in real()] + [dict(tg)]

    def fault_planner_crash(self, f):
        self.g.hplan.planner = None                  # the next decide() raises: the lease lapses

    def watch(self, secs, out):
        s = self.spec
        end = time.time() + secs
        min_gap = {}
        t_prev = 0.0
        while time.time() < end:
            t = time.time() - self.t0
            if self.alice is not None:
                self.alice.pump(0.05)
            else:
                time.sleep(0.05)
            self.faults_due(t)
            p = self.pose()
            if p is None:
                continue
            x, y, hdg, sog = p
            st = self.stat()
            truth = self.world.truth(time.time())
            gaps = [(hull_gap(x, y, o), o) for o in truth]
            g, go = min(gaps, key=lambda a: a[0]) if gaps else (999.0, None)
            moving = bool(go and go["speed"] > 0.2)
            for gg, o in gaps:
                k = o["id"]
                if k not in min_gap or gg < min_gap[k][0]:
                    min_gap[k] = (gg, o["speed"] > 0.2, st.get("harb_state_name"))
            mode = (self.g.telem or {}).get("mode_name")
            tr = self.truth.latest() if self.truth is not None else None      # HX7-fix3: the TRUE position
            allg = [(o["id"], round(hull_gap(x, y, o), 1), o["speed"] > 0.2) for o in truth]
            tel = self.g.telem or {}
            self.samples.append((round(t, 2), x, y, hdg, sog, st.get("harb_state_name"), st.get("harb_xte_m"), mode,
                                 g, (self.g.hplan.dec or {}).get("cap_kn"), moving, go["id"] if go else None,
                                 st.get("harb_kn"), st.get("harb_why"), ((self.g.hplan.dec or {}).get("text_en") or "")[:70],
                                 tel.get("ll_state"), tel.get("throttle"), allg, int(tel.get("alarm_bits") or 0),
                                 list((self.g.hplan.dec or {}).get("not_yielding") or []),
                                 bool((self.g.hplan.dec or {}).get("turn")),
                                 tr,                                                       # [21] TRUE (x, y, hdg, kn)
                                 [(o["id"], round(hull_gap(tr[0], tr[1], o), 1)) for o in truth] if tr else None))  # [22]
            # a HOLD in the first 25 m from a HOST-LOAD transient ("planner cycle late" U4 / the tracker overload / the lease
            # lapsed while episodes share the PC) is the captain's RESUME once the planner says clear (what a person does);
            # counted + reported as an event, only before 25 m of progress, at most 6 times
            dd = self.g.hplan.dec or {}
            rs = (dd.get("reasons") or [""])[0] if dd.get("stop") else ""
            if rs and rs != getattr(self, "_last_stop", None) and len(self.events) < 200:
                trk = {str(q.get("id")): q for q in (self.g.harbour.tracks() or [])}
                bl = [(b, (trk.get(str(b)) or {}).get("src"), (trk.get(str(b)) or {}).get("steer_ok"),
                       round((trk.get(str(b)) or {}).get("speed") or 0.0, 2)) for b in dd.get("blockers") or []]
                self.events.append((round(t, 1), "planner STOP: %s | blockers %s" % (rs[:90], bl[:4])))
                # HX7-fix3 (owner answer 3): the STOP's evidence - every blocker TRACK matched against the world truth
                # (the nearest real object to the track's position, centre to centre, and that object's hull gap)
                tr0 = tr
                bx, by = (tr0[0], tr0[1]) if tr0 else (x, y)
                ev = {"t": round(t, 2), "reason": rs[:120], "x": round(bx, 1), "true_pos": tr0 is not None,
                      "blockers": [], "not_yielding": []}

                def _match(b):
                    q = trk.get(str(b))
                    if not q or q.get("lat") is None:
                        return {"id": str(b), "track": False}
                    qx, qy = xy(q["lat"], q["lon"])
                    near = min(((math.hypot(qx - o.get("x", 0.0), qy - o.get("y", 0.0)), o) for o in truth
                                if o["kind"] != "pier"), default=(999.0, None), key=lambda a: a[0])
                    return {"id": str(b), "track": True, "src": q.get("src"), "steer_ok": bool(q.get("steer_ok")),
                            "tx": round(qx, 1), "ty": round(qy, 1), "match": near[1]["id"] if near[1] else None,
                            "match_d": round(near[0], 1),
                            "gap": round(hull_gap(bx, by, near[1]), 1) if near[1] else None}
                ev["blockers"] = [_match(b) for b in dd.get("blockers") or []]
                ev["not_yielding"] = [_match(b) for b in dd.get("not_yielding") or []]     # (U13 names them here)
                if "static-map" in [str(b) for b in dd.get("blockers") or []]:
                    ev["static"] = self._static_evidence(x, y, truth, tr0)
                self.stops.append(ev)
            self._last_stop = rs
            if (self.g.hplan.dec or {}).get("not_yielding") and "NOT GIVING WAY - STOP" in (
                    (self.g.hplan.dec or {}).get("text_en") or ""):
                self.ny_stop_seen = True                 # HX7-fix: a real stop for a vessel not giving way: never resumed
            if (st.get("harb_state_name") == "HOLD" and x < 25.0 and self.resumes < 6 and t < 40.0
                    and not getattr(self, "ny_stop_seen", False)
                    and not any(e[1] == "fault planner_crash" for e in self.events)   # HX7-fix4: never RESUME
                    # the HOLD an injected planner crash causes (it IS the fault's designed reaction, not host load)
                    and st.get("harb_why") in (2, 3) and not (self.g.hplan.dec or {}).get("stop")
                    and "overload" not in (((self.g.hplan.dec or {}).get("text_en")) or "")
                    and time.time() - self.last_resume > 4.0 and self.alice is not None):
                self.resumes += 1
                self.last_resume = time.time()
                self.events.append((round(t, 1), "RESUME after a host-load HOLD (why %s)" % st.get("harb_why")))
                self.alice.cmd(proto.T_HARBOUR, proto.p_harbour(proto.HARB_RESUME, self.spec.get("corr_m", 10.0),
                                                               self.spec.get("max_kn", 5.0)), 6.0)
                self.stick["confirm"] = True
                time.sleep(0.6)
                self.stick["confirm"] = False
            if st.get("harb_state_name") == "DONE" and s.get("stop_on_done", True):
                break
            if mode == "STANDBY" and t > 15 and s.get("stop_on_standby", True):
                break
            if s.get("stop_pred") and s["stop_pred"](self, t):
                break
        out["min_gap"] = min_gap
        out["secs_run"] = round(time.time() - self.t0, 1)

    def _static_evidence(self, x, y, truth, tr0):
        """HX7-fix4 (S14): a STOP whose blocker is 'static-map' names no track - record WHAT occupied static-map cells
        lie in the band the planner judges (ahead of the boat, inside the corridor + 2 m, up to STATIC_AHEAD_M) and the
        nearest REAL world object to each cluster (centre to centre, its hull gap from the TRUE position). Read only:
        the gateway's own StaticMap (chart cells + accumulated LiDAR cells at or above occ_w)."""
        sm = getattr(self.g.harbour, "static", None)
        out = {"chart": 0, "lidar": 0, "cells": []}
        if sm is None or not sm.origin:
            out["why"] = "no static map"
            return out
        corr = float(self.spec.get("corr_m", 10.0))
        tt = getattr(sm, "t_last", 0.0)
        lat0, lon0 = sm.origin
        pts = []
        for layer, keys in (("chart", list(sm.chart)), ("lidar", [k for k in list(sm.grid) if sm._w(k, tt) >= sm.occ_w])):
            for (i, j) in keys:
                la, lo = simworld.xy_to_ll(lat0, lon0, (i + 0.5) * sm.cell, (j + 0.5) * sm.cell)
                cx, cy = xy(la, lo)
                if x - 2.0 <= cx <= x + STATIC_AHEAD_M and abs(cy) <= corr + 2.0:
                    out[layer] += 1
                    pts.append((cx, cy, layer))
        objs = [o for o in truth if o["kind"] != "pier"]
        bx, by = (tr0[0], tr0[1]) if tr0 else (x, y)
        for cx, cy, layer in pts:
            near = min(((math.hypot(cx - o["x"], cy - o["y"]), o) for o in objs), default=(999.0, None),
                       key=lambda a: a[0])
            out["cells"].append({"x": round(cx, 1), "y": round(cy, 1), "layer": layer,
                                 "match": near[1]["id"] if near[1] else None, "match_d": round(near[0], 1),
                                 "gap": round(hull_gap(bx, by, near[1]), 1) if near[1] else None,
                                 "obj_len": near[1]["len"] if near[1] else None})
        return out

    # ------------------------------------------------------------- finish
    def standby_check(self):
        """STANDBY always wins: send it, the mode must be STANDBY within 3 s (idle + neutral first)."""
        if self.alice is None:
            return None
        self.alice.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
        end = time.time() + 4.0
        while time.time() < end:
            self.alice.pump(0.1)
            if (self.g.telem or {}).get("mode") == proto.MODE_STANDBY:
                return True
        return False

    def close(self):
        for fn in (lambda: self.snd and self.snd.stop(),
                   lambda: self.alice and self.alice.close(),
                   lambda: self.link and self.link.stop(),
                   lambda: self.gw and self.gw.stop(),
                   lambda: self.sim and self.sim.stop(),
                   lambda: self.truth and self.truth.close()):
            try:
                fn()
            except Exception:
                pass
        shutil.rmtree(self.tmp, ignore_errors=True)
