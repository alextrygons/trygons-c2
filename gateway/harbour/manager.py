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
No tracker here (HX4 reads latest() / on_batch).
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

from harbour import sensors, simworld

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
        self.on_batch = []                  # HX4: the tracker's callback(kind, batch) - called in tick()
        self.errors = 0
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
        # review M3: the pose time = when the gateway RECEIVED that telemetry (gw.telem_rx), not this read
        t_rx = getattr(self.gw, "telem_rx", None)
        t = float(t_rx) if isinstance(t_rx, (int, float)) and t_rx > 0 else time.time()
        p = {"t": t, "lat": o["lat"], "lon": o["lon"], "heading": float(o["heading"]) % 360.0,
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
                for fn in list(self.on_batch):
                    try:
                        fn(kind, b)
                    except Exception:
                        self.errors += 1
                        log.exception("harbour: on_batch")

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
