"""The ADS-B receiver on the gateway (CLAUDE.md X K4, docs/sar_design.md 6, step PF5): RECEIVE ONLY, like AIS.

AdsbManager starts adsb_worker.py (its OWN low-priority process: reads readsb / dump1090 SBS or Beast), keeps its newest
state, judges the aircraft against every UAV that is IN THE AIR (adsb.evaluate: horizontal + vertical CPA) and writes
gw_status "adsb":

  {"enabled", "state": OFF | NO_RX | NO_DATA | RX, "sim", "frames", "bad", "err", "targets": [...],
   "warn": [{icao, cs, slot, level 1 CAUTION | 2 WARNING, t, d_h, d_v, ...}],
   "proposals": [{id, slot, icao, cs, level, kind "hold" | "descend", alt_safe_m, text}], "flying": [slots]}

RULES (nothing here may break them; adsb tests AB* grep for them):
  * it only READS: no AURIX, no boat alarm, no UAV command, no MAVLink - a proposal is DATA for the stations; the station
    (the UAV's pilot or the captain) answers it with the NORMAL uav_hold / uav_route, which go through the unchanged
    UAV rules (pilot lock, the captain's YES for a flying route, the allow-list);
  * no receiver / no data is said («ADS-B: χωρίς δέκτη») - never an empty "clear sky";
  * a worker crash = a notice + restart; the drones never need it.
"""
import json
import logging
import os
import queue
import subprocess
import sys
import threading
import time

import adsb

log = logging.getLogger("adsb")
HERE = os.path.dirname(os.path.abspath(__file__))

OFF, NO_RX, NO_DATA, RX = "OFF", "NO_RX", "NO_DATA", "RX"
DEFAULTS = {"enabled": False, "sim": False, "source": "tcp:127.0.0.1:30003", "proto": "sbs", "decoder": "",
            "decoder_args": "", "python": "", "sim_start_m": 4000.0, "restart_s": 5.0, "silent_s": 6.0, "hold_s": 10.0, "max_listed": 40}


class AdsbManager:
    def __init__(self, cfg, gw):
        c = dict(DEFAULTS)
        for k, v in (cfg or {}).items():
            c[k] = v
        self.cfg = c
        self.gw = gw
        self.enabled = bool(c["enabled"] or c["sim"])
        self.lim = adsb.merged(c)
        self.proc = None
        self.worker = {}                    # the newest "state" line
        self.worker_t = 0.0
        self.starts = 0
        self.next_start = 0.0
        self.spawning = False
        self.lock = threading.Lock()
        self.events = []                    # adsb.evaluate result (newest)
        self.warn = {}                      # (icao, slot) -> held event
        self.props = {}                     # slot -> proposal
        self.flying = []
        self.seq = 0
        self.ref_t = 0.0
        self.q = queue.Queue(maxsize=20)
        self.errors = 0

    # --- the worker process -------------------------------------------------------------------------------------
    def start(self, loop=None):
        pass                                # spawned by tick() (restart rules in one place)

    def _cmd(self):
        c = self.cfg
        cmd = [c["python"] or sys.executable, os.path.join(HERE, "adsb_worker.py"), "--cfg",
               json.dumps({k: c[k] for k in adsb.DEFAULTS if k in c})]
        if c["sim"]:
            return cmd + ["--sim", "--sim-start-m", str(float(c["sim_start_m"]))]
        cmd += ["--source", str(c["source"]), "--proto", str(c["proto"])]
        dec = c.get("decoder")
        if dec == "auto":
            import toolpaths
            dec = toolpaths.readsb() or ""
        if dec:
            cmd += ["--decoder", dec, "--decoder-args", str(c.get("decoder_args") or "")]
        return cmd

    def _spawn(self):
        try:
            err = None
            st = getattr(self.gw, "storage", None)
            try:
                if st is not None and st.usable:
                    err = open(os.path.join(st.folder("logs"), "adsb_worker.log"), "ab")
            except OSError:
                err = None
            try:
                p = subprocess.Popen(self._cmd(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=err or subprocess.DEVNULL, bufsize=0, cwd=HERE)
            finally:
                if err is not None:
                    err.close()
        except OSError as e:
            log.warning("adsb: the worker did not start: %s", e)
            return
        finally:
            self.spawning = False
        self.proc = p
        self.worker, self.worker_t = {}, time.time()
        self.starts += 1
        self.q = queue.Queue(maxsize=20)
        threading.Thread(target=self._reader, args=(p,), daemon=True).start()
        threading.Thread(target=self._writer, args=(p, self.q), daemon=True).start()

    def _reader(self, p):
        try:
            for line in p.stdout:
                try:
                    m = json.loads(line.decode("utf-8"))
                except ValueError:
                    continue
                if p is self.proc and m.get("ev") == "state":
                    with self.lock:
                        self.worker, self.worker_t = m, time.time()
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

    def to_worker(self, obj):
        try:
            self.q.put_nowait((json.dumps(obj) + "\n").encode("utf-8"))
        except queue.Full:
            pass

    def stop(self, wait=True):
        p, self.proc = self.proc, None
        if p is None:
            return
        try:
            self.q.put_nowait(b'{"op": "stop"}\n')
            self.q.put_nowait(None)
        except queue.Full:
            pass
        try:
            if wait:
                p.wait(1.5)
            elif p.poll() is None:
                p.kill()
        except (subprocess.TimeoutExpired, OSError):
            pass
        try:
            if p.poll() is None:
                p.kill()
                p.wait(2)
        except (OSError, subprocess.TimeoutExpired):
            pass

    # --- the judgement ------------------------------------------------------------------------------------------
    def boat_pos(self):
        try:
            return self.gw.uav.boat().get("pos")
        except Exception:
            return None

    def drones(self):
        try:
            return self.gw.uav.drones() if self.gw.uav.enabled else {}
        except Exception:
            self.errors += 1
            return {}

    def state(self, now=None):
        now = time.time() if now is None else now
        if not self.enabled:
            return OFF
        with self.lock:
            w, wt = self.worker, self.worker_t
        if self.proc is None or not w or now - wt > float(self.cfg["silent_s"]):
            return NO_RX
        return w.get("state") or NO_RX

    def tick(self, now=None):
        """1 Hz: the worker alive, the judgement, the proposals."""
        now = time.time() if now is None else now
        if not self.enabled:
            return
        p = self.proc
        if p is not None and (p.poll() is not None or (self.worker_t and now - self.worker_t > 3 * float(
                self.cfg["silent_s"]))):
            self.stop(wait=False)
            self.next_start = now + float(self.cfg["restart_s"])
        if self.proc is None and now >= self.next_start and not self.spawning:
            self.spawning = True
            self.next_start = now + float(self.cfg["restart_s"])
            threading.Thread(target=self._spawn, daemon=True).start()
        pos = self.boat_pos()
        if self.cfg["sim"] and self.proc is not None and now - self.ref_t >= 5.0:
            ref = pos
            if ref is None:
                for d in self.drones().values():
                    if d.get("lat") is not None and not d.get("in_slot"):
                        ref = (d["lat"], d["lon"])
                        break
            if ref is not None:
                self.ref_t = now
                self.to_worker({"op": "ref", "lat": ref[0], "lon": ref[1]})
        self.judge(now)

    def aircraft(self):
        with self.lock:
            return list(self.worker.get("targets") or [])

    def judge(self, now, aircraft=None, drones=None, state=None):
        """The CPA events + the held warnings + the proposals. aircraft / drones / state are injectable (tests)."""
        st = self.state(now) if state is None else state
        ac = self.aircraft() if aircraft is None else aircraft
        dr = self.drones() if drones is None else drones
        if st != RX:
            ac = []              # a receiver that is not receiving gives no judgement (and the station says so)
        unknown, nopos = [], []
        events, flying = adsb.evaluate(ac, dr, self.lim, unknown_out=unknown, nopos_out=nopos)
        self.events, self.flying = events, flying
        self.uav_unknown = unknown              # PF5-r MED-2: notice «ADS-B: UAV n unknown altitude» on the stations
        self.uav_nopos = sorted(set(nopos))     # PF4-r2 LOW-3: «ADS-B: UAV n position unknown – no judgement»
        hold = float(self.cfg["hold_s"])
        for e in events:
            k = (e["icao"], e["slot"])
            old = self.warn.get(k)
            e = dict(e, t_seen=now, level_max=max(e["level"], old["level_max"] if old else 0))
            self.warn[k] = e
        for k in [k for k, e in self.warn.items() if now - e["t_seen"] > hold]:
            del self.warn[k]
        # one proposal per UAV: its worst held warning; the id is stable while that aircraft stays the cause
        by_slot = {}
        for e in sorted(self.warn.values(), key=lambda e: (-e["level"], e["t"])):
            by_slot.setdefault(e["slot"], e)
        for s in [s for s in self.props if s not in by_slot]:
            del self.props[s]
        for s, e in by_slot.items():
            p = self.props.get(s)
            if p is None or p["icao"] != e["icao"]:
                self.seq += 1
                p = self.props[s] = {"id": "adsb%d" % self.seq, "slot": s, "icao": e["icao"]}
            p.update({"cs": e["cs"], "level": e["level"], "kind": e["kind"], "alt_safe_m": self.lim["alt_safe_m"],
                      "d_h": e["d_h"], "d_v": e["d_v"], "t": e["t"]})
        return events

    def status(self, saver=False):
        st = self.state()
        if not self.enabled:
            return {"enabled": False, "state": OFF, "targets": [], "warn": [], "proposals": [], "flying": []}
        with self.lock:
            w = dict(self.worker)
        refs = []
        pos = self.boat_pos()
        if pos:
            refs.append(tuple(pos))
        for d in self.drones().values():
            if d.get("lat") is not None and not d.get("in_slot") and d.get("armed") and not d.get("landed"):
                refs.append((d["lat"], d["lon"]))
        tg = [a for a in (w.get("targets") or []) if adsb.keep(a, refs, self.lim)] if st == RX else []
        for a in tg:
            a["stale"] = (a.get("age") or 0.0) > self.lim["stale_s"]
        tg = [a for a in tg if (a.get("age") or 0.0) <= self.lim["remove_s"]]
        tg = tg[:15 if saver else int(self.cfg["max_listed"])]
        warn = sorted(self.warn.values(), key=lambda e: (-e["level"], e["t"]))
        return {"enabled": True, "state": st, "sim": bool(self.cfg["sim"]), "frames": w.get("frames", 0),
                "bad": w.get("bad", 0), "err": w.get("err", ""), "starts": self.starts, "targets": tg,
                "warn": [{k: e[k] for k in ("icao", "cs", "slot", "level", "t", "d_h", "d_v", "kind")}
                         for e in warn],
                "proposals": [dict(p) for p in self.props.values()], "flying": list(self.flying),
                "uav_unknown": list(getattr(self, "uav_unknown", [])),
                "uav_nopos": list(getattr(self, "uav_nopos", [])),
                "limits": {"max_alt_ft": self.lim["max_alt_ft"], "max_km": self.lim["max_km"]}}
