"""The harbour sensors on the gateway (CLAUDE.md Z, docs/harbour_design.md 5 + 5a, step HX3).

HarbourSensors starts one worker process per configured sensor (radar_worker.py / lidar_worker.py: OWN processes,
below normal), feeds them the own boat pose from the FRESH AURIX telemetry (gw.own(), 5 Hz), keeps their newest batch,
judges each sensor's health (sensors.HealthJudge: OFF / OFFLINE / STARTING / OK / DEGRADED / LOST) and writes
gw_status "harbour" {"sensors": {radar: {...}, lidar: {...}}, "notices": [...], "sim"}.

RULES (gateway/test_harbour_sensors.py checks them):
  * READ ONLY towards the boat: it never imports aurix.py, never sends a frame, never raises an alarm, never changes a
    mode - a sensor change = a NOTICE in gw_status (the stations show it from HX6; the planner (HX5) reads health()
    for its unsure list U1 / U2);
  * a worker crash / hang = a notice + a restart after restart_s (5 s); the gateway never stops for a sensor;
  * the simulated world gets its origin from the own boat's FIRST fresh position (both sensors see the same world).
HX4: the fusion tracker (tracker.py) + the static map (staticmap.py) run HERE, in the gateway thread's tick (pure,
no own process: design 6a - measured < 30 ms p99): every received batch is fused, then the camera obstacles
(gw.vision.obstacles), the AIS targets (gw.ais.list()) and the attitude (TELEM2 att_roll / att_pitch); tracks() /
tracker_flags() are the planner's (HX5) inputs; gw_status "harbour" "tracker" = the summary + the flags. Still READ
ONLY: nothing goes to the AURIX.
"""
import collections
import json
import logging
import os
import queue
import subprocess
import sys
import threading
import time

from harbour import sensors, simworld, staticmap, tracker

mono = time.monotonic          # HX3 review M1: every age / timer (a GPS clock step never fakes an OK)

log = logging.getLogger("harbour")
HERE = os.path.dirname(os.path.abspath(__file__))
KN = 0.514444


class SensorProc:
    """One worker process + its health."""

    def __init__(self, kind, c, mgr):
        self.kind, self.c, self.mgr = kind, c, mgr
        self.judge = sensors.HealthJudge(kind, c)
        self.proc = None
        self.q = queue.Queue(maxsize=50)
        self.lock = threading.Lock()
        self.last_line = 0.0
        self.status = {}
        self.latest = None
        self.lat = collections.deque(maxlen=500)     # receive time - sensor time (s): the latency
        self.starts = 0
        self.restarts = 0
        self.next_start = 0.0
        self.spawning = False
        self.batches = 0
        self.pending = []                            # batches for the gateway thread (tick)

    def cmd(self, world_json):
        c = self.mgr.cfg
        cmd = [c.get("python") or sys.executable, os.path.join(HERE, "%s_worker.py" % self.kind),
               "--cfg", json.dumps(self.c)]
        if self.c.get("driver") == "sim" and world_json:
            cmd += ["--wait-world"]                     # the world goes on stdin (review M2: never argv)
        return cmd

    def spawn(self, world_json):
        try:
            err = None
            st = getattr(self.mgr.gw, "storage", None)
            try:
                if st is not None and getattr(st, "usable", False):
                    err = open(os.path.join(st.folder("logs"), "harbour_%s_worker.log" % self.kind), "ab")
            except OSError:
                err = None
            try:
                p = subprocess.Popen(self.cmd(world_json), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=err or subprocess.DEVNULL, bufsize=0, cwd=HERE)
            finally:
                if err is not None:
                    err.close()
        except (OSError, ValueError) as e:
            log.warning("harbour %s: the worker did not start: %s", self.kind, e)
            with self.lock:
                self.judge.spawn_fail = str(e)[:120] or e.__class__.__name__    # review M2: LOST, said why
            self.spawning = False
            return
        self.q = queue.Queue(maxsize=50)
        with self.lock:
            self.proc = p
            self.status, self.last_line = {}, mono()
            self.judge.spawn_fail = ""
            self.judge.spawned(mono())
        if world_json:
            self.send({"op": "world", "world": json.loads(world_json)})
        self.starts += 1
        self.spawning = False
        threading.Thread(target=self._reader, args=(p,), daemon=True).start()
        threading.Thread(target=self._writer, args=(p, self.q), daemon=True).start()

    def _reader(self, p):
        try:
            for line in p.stdout:
                try:
                    m = json.loads(line.decode("utf-8"))
                except ValueError:
                    continue
                now = mono()
                with self.lock:
                    if p is not self.proc:
                        return
                    self.last_line = now
                    ev = m.get("ev")
                    if ev == "status":
                        self.status = m
                        self.judge.status_line(m)
                    elif ev == self.kind:
                        m["t_rx_epoch"] = time.time()
                        m["t_rx_mono"] = now
                        self.judge.batch(now, m)
                        try:
                            self.lat.append(m["t_rx_epoch"] - float(m["t"]))
                        except (KeyError, TypeError, ValueError):
                            pass
                        self.latest = m
                        self.batches += 1
                        if len(self.pending) < 50:
                            self.pending.append(m)
        except (OSError, ValueError):
            pass

    @staticmethod
    def _writer(p, q):
        try:
            while True:
                b = q.get()
                if b is None:
                    return
                p.stdin.write(b)
                p.stdin.flush()
        except (OSError, ValueError):
            pass

    def send(self, obj):
        try:
            self.q.put_nowait((json.dumps(obj) + "\n").encode("utf-8"))
        except queue.Full:
            pass

    def stop(self, wait=True):
        with self.lock:
            p, self.proc = self.proc, None
            self.judge.gone(mono())
        if p is None:
            return
        if not wait:                                    # review L2: never block the gateway loop on a kill
            threading.Thread(target=self._end, args=(p, False, self.q), daemon=True).start()
            return
        self._end(p, True, self.q)

    @staticmethod
    def _end(p, wait, q):
        try:
            q.put_nowait(b'{"op": "stop"}\n')
            q.put_nowait(None)
        except queue.Full:
            pass
        try:
            if wait:
                p.wait(1.5)
        except (subprocess.TimeoutExpired, OSError):
            pass
        try:
            if p.poll() is None:
                p.kill()
                p.wait(2)
        except (OSError, subprocess.TimeoutExpired):
            pass
        for s in (p.stdin, p.stdout):
            try:
                s.close()
            except (OSError, AttributeError):
                pass

    def pid(self):
        p = self.proc
        return p.pid if p is not None else None

    def latency(self):
        v = sorted(self.lat)
        if not v:
            return None
        return {"n": len(v), "p50_ms": round(v[len(v) // 2] * 1000.0, 1),
                "p99_ms": round(v[min(len(v) - 1, int(len(v) * 0.99))] * 1000.0, 1), "max_ms": round(v[-1] * 1000, 1)}


class HarbourSensors:
    def __init__(self, cfg, gw):
        self.raw_cfg = cfg or {}
        self.cfg = sensors.merged(cfg)
        self.gw = gw
        self.enabled = bool(self.cfg.get("enabled") or self.cfg.get("sim")) and any(
            self.cfg[k]["enabled"] for k in sensors.KINDS)
        self.sim = bool(self.cfg.get("sim"))
        self.procs = {k: SensorProc(k, self.cfg[k], self) for k in sensors.KINDS}
        self.world = None
        self.world_json = ""
        self.notices = collections.deque(maxlen=30)
        self.last_state = {k: None for k in sensors.KINDS}
        self.no_rx_logged = False
        self.on_batch = []                  # HX4: the tracker's callback(kind, batch) - called in tick()
        self.errors = 0
        tc = self.cfg.get("tracker") if isinstance(self.cfg.get("tracker"), dict) else {}
        self.static = staticmap.StaticMap()
        self.static_file = self.cfg.get("static_file") or ""
        if self.static_file and os.path.exists(self.static_file):
            try:
                self.static = staticmap.StaticMap.load(self.static_file)
            except (OSError, ValueError, KeyError, TypeError) as e:
                log.warning("harbour: static map %s not usable (%s) - a new one", self.static_file, e)
        self.static_saved = mono()
        self.chart_done = False
        self.tracker = tracker.Tracker(tc, static_map=self.static)
        self.tracker_err = 0
        self.tracker_err_until = 0.0
        self.cycle_mono = None
        if self.enabled and self.sim:
            w = self.cfg.get("world") or ""
            try:
                self.world = simworld.World.load(w) if w else simworld.World()
            except (OSError, ValueError, KeyError) as e:
                log.warning("harbour: world file %s not usable (%s) - the built-in harbour", w, e)
                self.world = simworld.World()

    def start(self, loop=None):
        pass                                # spawned by tick()

    def own_pose(self):
        try:
            o = self.gw.own()
        except Exception:
            self.errors += 1
            return None
        if not o or not o.get("fresh") or o.get("lat") is None or o.get("heading") is None:
            return None
        # review M3 + round 2 MED-2: the pose time = when the gateway RECEIVED that telemetry frame (gw.telem_rx,
        # epoch, set with gw.telem in gateway.py on every TELEMETRY frame = the one carrying lat / lon / heading);
        # NO fallback - an unknown receive time = no pose (the sensors go DEGRADED "no own pose", then LOST)
        t_rx = getattr(self.gw, "telem_rx", None)
        if isinstance(t_rx, bool) or not isinstance(t_rx, (int, float)) or t_rx <= 0:
            if not self.no_rx_logged:
                self.no_rx_logged = True
                log.warning("harbour: the telemetry receive time is unknown - no pose for the sensors")
            return None
        p = {"t": float(t_rx), "lat": o["lat"], "lon": o["lon"], "heading": float(o["heading"]) % 360.0,
             "sog": o.get("sog"), "cog": o.get("cog")}
        return p if sensors.valid_pose(p) else None

    def notice(self, text, level="info"):
        self.notices.append({"t": round(time.time(), 1), "text": text, "level": level})   # epoch: display only
        log.warning("harbour: %s", text)

    def tick(self, now=None):
        """5 Hz from the gateway: the own pose to the workers, the workers alive, the health, the notices."""
        now = mono() if now is None else now
        if not self.enabled:
            return
        pose = self.own_pose()
        if self.world is not None and not self.world_json and pose is not None:
            self.world.place(pose["lat"], pose["lon"], time.time())      # the harbour around the boat's first position
            self.world_json = json.dumps(self.world.to_json())
        if self.tracker.origin is None and self.world is not None and self.world.origin:
            self.tracker.set_origin(*self.world.origin)       # before the first batch is fused
        for kind, sp in self.procs.items():
            if not sp.c.get("enabled"):
                continue
            p = sp.proc
            if p is not None:
                dead = p.poll() is not None
                hung = now - sp.last_line > float(self.cfg["silent_s"])
                if dead or hung:
                    self.notice("%s WORKER %s - restarting" % (kind.upper(), "CRASHED" if dead else "HUNG"),
                                "caution")
                    sp.stop(wait=False)
                    sp.restarts += 1
                    sp.next_start = now + float(self.cfg["restart_s"])
            wait_world = sp.c.get("driver") == "sim" and self.world is not None and not self.world_json
            if sp.proc is None and not sp.spawning and now >= sp.next_start and not wait_world:
                sp.spawning = True
                sp.next_start = now + float(self.cfg["restart_s"])
                threading.Thread(target=sp.spawn, args=(self.world_json,), daemon=True).start()
            if pose is not None and sp.proc is not None:
                sp.send(dict(pose, op="pose"))
            with sp.lock:
                st, why = sp.judge.judge(now)
                if wait_world:
                    st, why = sensors.STARTING, "waiting for the own boat position"
                pend, sp.pending = sp.pending, []
            if st != self.last_state[kind]:
                if self.last_state[kind] is not None or st not in (sensors.STARTING, sensors.OFF):
                    lvl = "caution" if st in (sensors.LOST, sensors.DEGRADED, sensors.OFFLINE) else "info"
                    self.notice("%s %s%s" % (kind.upper(), st, (" - " + why) if why else ""), lvl)
                self.last_state[kind] = st
            for b in pend:
                t0 = time.perf_counter()
                try:
                    self.tracker.feed(kind, self._to_mono(b), now=mono())
                except Exception:
                    self._tracker_failed(now)
                self.tracker.note_ms((time.perf_counter() - t0) * 1000.0)
                for fn in list(self.on_batch):
                    try:
                        fn(kind, b)
                    except Exception:
                        self.errors += 1
                        log.exception("harbour: on_batch")

        self._track(now, pose)

    def _to_mono(self, b):
        """Review H3: the tracker runs on the MONOTONIC clock. A batch's measurement time = its monotonic receive
        time minus its age (the epoch receive time - the epoch measurement time, both from nearby clocks); an age
        outside -0.5 .. 10 s (a clock step between the worker and here) = the batch time made invalid -> the tracker
        refuses it and says so (input rejected)."""
        b = dict(b)
        try:
            age = float(b["t_rx_epoch"]) - float(b["t"])
            rx = float(b["t_rx_mono"])
        except (KeyError, TypeError, ValueError):
            b["t"] = float("nan")
            return b
        b["t"] = rx - age if -0.5 <= age <= 10.0 else float("nan")
        return b

    def _tracker_failed(self, now):
        self.tracker_err += 1
        self.tracker_err_until = now + 5.0          # the flag is held 5 s (the planner must see it)
        log.exception("harbour: tracker")

    def _track(self, now, pose):
        """The camera + AIS + attitude into the tracker, then its cycle (maintenance + flags). Never raises."""
        tr = self.tracker
        t0 = time.perf_counter()
        try:
            if tr.origin is None:
                if self.world is not None and self.world.origin:
                    tr.set_origin(*self.world.origin)
                elif self.static.origin:             # a stored harbour map fixes the harbour frame
                    tr.set_origin(*self.static.origin)
                elif pose is not None:
                    tr.set_origin(pose["lat"], pose["lon"])
            if tr.origin is not None:               # review M1: the static map lives in the tracker's frame
                so = self.static.origin
                if so is None:
                    self.static.origin = tuple(tr.origin)
                elif abs(so[0] - tr.origin[0]) > 1e-5 or abs(so[1] - tr.origin[1]) > 1e-5:
                    self.notice("STATIC MAP REFUSED - stored for another harbour origin; a new one", "caution")
                    self.static = staticmap.StaticMap(origin=tr.origin)
                    tr.static = self.static
                    self.chart_done = False
            if tr.origin is not None and not self.chart_done:
                self.chart_done = True
                charts = getattr(self.gw, "charts", None)
                if charts:
                    lim = 2.0
                    try:
                        lim = float(self.gw.shallow.limit())
                    except Exception:
                        pass
                    self.static.add_chart(charts, tr.origin, float(self.cfg.get("chart_radius_m", 300.0)), 2.0, lim)
            t_ep = now                              # review H3: the tracker clock = monotonic
            vis = getattr(self.gw, "vision", None)
            if pose is not None and vis is not None:
                tr.camera(list(getattr(vis, "obstacles", None) or []), pose, t_ep)
            ais = getattr(self.gw, "ais", None)
            if ais is not None and hasattr(ais, "list"):
                tr.ais(ais.list(), t_ep)
            t2 = getattr(self.gw, "telem2", None) or {}
            att = None
            if isinstance(t2.get("att_roll"), (int, float)) and isinstance(t2.get("att_pitch"), (int, float)):
                att = (float(t2["att_roll"]), float(t2["att_pitch"]))
            health, ages, skews = {}, {}, {}
            for kind, sp in self.procs.items():
                with sp.lock:
                    health[kind] = sp.judge.state if sp.c.get("enabled") else sensors.OFF
                    ages[kind] = None if sp.judge.last_rx is None else now - sp.judge.last_rx
                    skews[kind] = sp.judge.skew
            tr.cycle(health=health, ages=ages, skews=skews, attitude=att, no_pose=pose is None,
                     error=now < self.tracker_err_until, now=t_ep)
        except Exception:
            self._tracker_failed(now)
            try:
                tr._flags = dict(tr._flags or {})
                u = [x for x in (tr._flags.get("unsure") or []) if x != "tracker error"] + ["tracker error"]
                tr._flags.update({"error": True, "unsure": u})
            except Exception:
                pass
        tr.note_ms((time.perf_counter() - t0) * 1000.0)
        self.cycle_mono = now
        if self.static_file and now - self.static_saved > 300.0:
            self.static_saved = now
            try:
                js = json.dumps(self.static.to_json())        # serialised here (no dict change under the writer)
            except (TypeError, ValueError):
                js = None
            if js:
                threading.Thread(target=self._save_static, args=(js,), daemon=True).start()

    def _save_static(self, js):
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.static_file)), exist_ok=True)
            tmp = self.static_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(js)
            os.replace(tmp, self.static_file)
        except (OSError, ValueError) as e:
            log.warning("harbour: static map not saved: %s", e)

    def tracks(self):
        """The fused tracks (tracker.py) - the planner's (HX5) input."""
        return self.tracker.tracks_out()

    def tracker_flags(self):
        """The planner's unsure inputs (design 7.4): explicit flags, never hidden."""
        f = dict(self.tracker.flags() or {})
        # review M3: the age of the last cycle (a hung tick = old flags: the planner treats > 0.5 s as unsure)
        f["cycle_age_s"] = None if self.cycle_mono is None else round(mono() - self.cycle_mono, 3)
        if self.cycle_mono is None or mono() - self.cycle_mono > 0.5:
            f["unsure"] = list(f.get("unsure") or []) + ["tracker cycle old / not run"]
        if mono() < self.tracker_err_until and "tracker error" not in (f.get("unsure") or []):
            f["error"] = True
            f["unsure"] = list(f.get("unsure") or []) + ["tracker error"]
        return f

    def fault(self, kind, name, on=True):
        """SIMULATOR only: inject a fault into a simulated sensor (tests, demo); refused for a real driver."""
        sp = self.procs.get(kind)
        if sp is None or sp.c.get("driver") != "sim":
            return False
        sp.send({"op": "fault", "name": name, "on": bool(on)})
        return True

    def health(self, kind):
        sp = self.procs[kind]
        with sp.lock:
            return sp.judge.state, sp.judge.why

    def latest(self, kind):
        sp = self.procs[kind]
        with sp.lock:
            return sp.latest

    def pids(self):
        return {k: sp.pid() for k, sp in self.procs.items()}

    def stop(self):
        for sp in self.procs.values():
            sp.stop()

    def status(self):
        out = {"enabled": self.enabled, "sim": self.sim, "sensors": {}, "notices": list(self.notices)[-10:]}
        if self.enabled:
            ts = self.tracker.summary()
            ts["flags"] = {k: v for k, v in self.tracker_flags().items() if k != "unsure"}
            ts["unsure"] = self.tracker_flags().get("unsure", [])
            ts["static_cells"] = self.static.n_cells()
            out["tracker"] = ts
        if self.world is not None and self.world.origin:
            out["origin"] = list(self.world.origin)
        for k, sp in self.procs.items():
            with sp.lock:
                st, why = sp.judge.state, sp.judge.why
                b = sp.latest
                s = dict(sp.status)
            out["sensors"][k] = {"state": st if self.enabled and sp.c.get("enabled") else sensors.OFF, "why": why,
                                 "driver": sp.c.get("driver"), "rate_hz": s.get("rate_hz"),
                                 "items": len(b["items"]) if b else 0, "batches": sp.batches,
                                 "starts": sp.starts, "restarts": sp.restarts, "latency": sp.latency(),
                                 "skew_ms": None if sp.judge.skew is None else round(sp.judge.skew * 1000.0)}
        return out
