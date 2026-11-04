"""The UAV magazine on the gateway (CLAUDE.md V, docs/uav_design.md): step U2 = the registry, the slot USB checks,
the GPS warm start and the simulated drones.

  station --uav_register {slot}--> UavManager --(pipe, JSON lines)--> router_worker.py (OWN process) --USB--> drone
  station <--uav_ack {ok, code, slot, text, problems}-- ...        gw_status "uav" (1 Hz): slots, router, notices

Never blocks the event loop: the router is its own process (started from a helper thread), its pipe is read / written
by threads, the registry file is written by a helper thread. A failure = a NOTICE only ("uav" state OFFLINE + why;
never an alarm, never a mode change, never a frame to the AURIX - nothing here imports aurix.py: UA5). The worker is
restarted after restart_s.

Who may register: any station but a VIEWER (copilot, captain, a single operator, a station on the boat), only while
no launch sequence runs (U3). Refusal 122 = not registered / the check failed (the reasons in "problems").
GPS warm start: a registered drone present on its slot cable and not armed gets time (+ the boat's trusted position)
at once after its registration and then every assist_s (gpsassist.py).
"""
import json
import os
import queue
import subprocess
import sys
import threading
import time

from common import deep_merge, log
from . import gpsassist
from .registry import Registry
from .usbslots import slot_specs

HERE = os.path.dirname(os.path.abspath(__file__))
ACTIVE, OFFLINE = "ACTIVE", "OFFLINE"
ERR_UAV = 122                                  # stations.GW_RESULTS 122: UAV not registered / check failed

DEFAULTS = {
    "enabled": False,                          # a boat without the magazine: nothing starts
    "n_slots": 6,
    "slots": [],                               # [{"usb": "usbloc:1-2.1", "vtx": "R1", "elrs": "tx1", "name": ...}]
    "sim": False,                              # start the simulated drones (gateway/uav/simdrone.py) - demo / tests
    "sim_base_port": 14600,
    "sim_args": [],                            # e.g. ["--fault", "2:no_fix"]
    "python": None,                            # None = this Python
    "restart_s": 5.0,
    "silent_s": 5.0,                           # no status from the router this long -> OFFLINE + restart
    "register_s": 20.0,                        # a registration not answered this long -> refused
    "assist_s": 60.0,                          # the GPS warm start again every assist_s while in the slot
    "assist_extra": "",                        # W56 (b): a file of UBX-MGA bytes from an assist u-blox (ephemeris)
}


class UavManager:
    def __init__(self, cfg, gw):
        self.cfg = deep_merge(DEFAULTS, cfg or {})
        self.gw = gw
        self.enabled = bool(self.cfg["enabled"])
        self.n = int(self.cfg["n_slots"])
        if self.cfg["sim"] and not self.cfg["slots"]:
            b = int(self.cfg["sim_base_port"])
            self.cfg["slots"] = [{"usb": "udp:127.0.0.1:%d" % (b + i), "vtx": "R%d" % min(8, i), "elrs": "tx%d" % i}
                                 for i in range(1, self.n + 1)]
        self.specs = slot_specs(self.cfg, self.n)
        st = getattr(gw, "storage", None)
        path = os.path.join(st.folder("state"), "uav_registry.json") if st is not None and st.usable else None
        self.reg = Registry(path, self.n)
        self.state, self.why = OFFLINE, "switched off" if not self.enabled else "starting"
        self.proc = self.sim_proc = None
        self.wq = queue.Queue(maxsize=200)
        self.loop = None
        self.worker = {}
        self.worker_t = 0.0
        self.next_start = 0.0
        self.spawning = False
        self.starts = 0
        self.pending = {}                       # request id -> (station id, slot, t0)
        self.req_id = 0
        self.assist = {}                        # slot -> {"t", "what", "ok", "why"}
        self.next_assist = {}
        self.history = []                       # (t, state, why)
        self.log = []                           # router log lines (last 20)

    # --- the processes --------------------------------------------------------------------------------------------
    def start(self, loop):
        self.loop = loop
        if self.enabled:
            self._spawn_bg()

    def _spawn_bg(self):
        if self.spawning:
            return
        self.spawning = True
        self.next_start = time.time() + float(self.cfg["restart_s"])

        def run():
            try:
                self._spawn()
            finally:
                self.spawning = False
        threading.Thread(target=run, daemon=True).start()

    def _errlog(self, name):
        st = getattr(self.gw, "storage", None)
        try:
            if st is not None and st.usable:
                return open(os.path.join(st.folder("logs"), name), "ab")
        except OSError:
            pass
        return subprocess.DEVNULL

    def _spawn(self):
        py = self.cfg["python"] or sys.executable
        if self.cfg["sim"] and (self.sim_proc is None or self.sim_proc.poll() is not None):
            err = self._errlog("uav_simdrone.log")
            try:
                self.sim_proc = subprocess.Popen(
                    [py, os.path.join(HERE, "simdrone.py"), "--base-port", str(int(self.cfg["sim_base_port"])),
                     "--n", str(self.n), "--stdin-eof-exit"] + [str(a) for a in self.cfg["sim_args"]],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=err, cwd=HERE)
            except OSError as e:
                log.warning("uav: simulated drones did not start: %s", e)
            finally:
                if err is not subprocess.DEVNULL:
                    err.close()
        err = self._errlog("uav_router.log")
        try:
            p = subprocess.Popen([py, os.path.join(HERE, "router_worker.py")], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, stderr=err, bufsize=0, cwd=HERE)
        except OSError as e:
            if self.loop is not None:
                self.loop.call_soon_threadsafe(self._health, OFFLINE, "the router process did not start: %s" % e)
            return
        finally:
            if err is not subprocess.DEVNULL:
                err.close()
        q = queue.Queue(maxsize=200)
        self.wq = q
        self.worker_t = time.time()
        self.starts += 1
        self.proc = p
        threading.Thread(target=self._reader, args=(p,), daemon=True).start()
        threading.Thread(target=self._writer, args=(p, q), daemon=True).start()
        self.to_worker(self._config())
        log.info("uav: router process started (%d slots%s)", self.n, ", simulated drones" if self.cfg["sim"] else "")

    def _config(self):
        return {"op": "config", "slots": [{"slot": s, "usb": usb, "sysid": (self.reg.recs.get(s) or {}).get("sysid")
                                           if (self.reg.recs.get(s) or {}).get("ok") else None}
                                          for s, usb, _v, _e in self.specs]}

    def _reader(self, p):
        try:
            for line in p.stdout:
                try:
                    m = json.loads(line.decode("utf-8"))
                except ValueError:
                    continue
                if self.loop is not None:
                    self.loop.call_soon_threadsafe(self._from_worker, p, m)
        except (OSError, ValueError, RuntimeError):
            pass

    def _writer(self, p, q):
        try:
            while True:
                item = q.get()
                if item is None:
                    break
                p.stdin.write(item)
        except (OSError, ValueError):
            pass

    def to_worker(self, obj):
        if self.proc is None:
            return False
        try:
            self.wq.put_nowait((json.dumps(obj) + "\n").encode("utf-8"))
            return True
        except queue.Full:
            return False

    def stop(self, wait=True):
        """wait=True: the gateway stops - the router AND the simulated drones end (never a stray process).
        wait=False (from the event loop, a router restart): only the router is killed, never waited for."""
        p, self.proc = self.proc, None
        self.worker = {}
        if p is not None:
            try:
                self.wq.put_nowait(b'{"op": "stop"}\n')
                self.wq.put_nowait(None)
            except queue.Full:
                pass
            try:
                if wait:
                    p.wait(1.5)
                elif p.poll() is None:
                    p.kill()
            except subprocess.TimeoutExpired:
                p.kill()
            except OSError:
                pass
        if wait:
            for proc in (p, self.sim_proc):
                if proc is None:
                    continue
                try:
                    if proc is self.sim_proc and proc.stdin:
                        proc.stdin.close()             # its --stdin-eof-exit
                    proc.wait(1.0)
                except (OSError, subprocess.TimeoutExpired, ValueError):
                    pass
                try:
                    if proc.poll() is None:
                        proc.kill()
                        proc.wait(2)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            self.sim_proc = None

    def _from_worker(self, p, m):
        if p is not self.proc:
            return
        ev = m.get("ev")
        if ev == "status":
            self.worker, self.worker_t = m, time.time()
            self._health(ACTIVE, "")
        elif ev == "register":
            self._registered(m)
        elif ev == "assist":
            self.assist[int(m["slot"])] = {"t": time.time(), "what": m.get("what"), "ok": bool(m.get("ok")),
                                           "why": m.get("why", "")}
        elif ev == "log":
            self.log.append(m.get("text", ""))
            del self.log[:-20]

    def _health(self, state, why):
        if (state, why) != (self.state, self.why):
            old = self.state
            self.state, self.why = state, why
            (log.warning if state != ACTIVE else log.info)("uav router %s (%s)", state, why)
            if state != old:
                self.history.append((time.time(), state, why))
                del self.history[:-50]
                self.gw.audit("uav_state", state=state, why=why)

    def tick(self, now=None):
        """4 Hz from the event loop: the router alive, registration time-outs, GPS warm starts."""
        now = time.time() if now is None else now
        if not self.enabled:
            return
        p = self.proc
        if p is not None and (p.poll() is not None or now - self.worker_t > float(self.cfg["silent_s"])):
            dead = p.poll() is not None
            self.stop(wait=False)
            # restart after restart_s, never at once: no crash loop, and the OFFLINE notice reaches every station
            # (gw_status) before the router is back (gate U2: a restart within one tick was never shown)
            self.next_start = now + float(self.cfg["restart_s"])
            self._health(OFFLINE, "the router process %s - restarting" % ("ended" if dead else "is silent"))
        if self.proc is None and now >= self.next_start:
            self._spawn_bg()
        for rid, (sid, slot, t0) in list(self.pending.items()):
            if now - t0 > float(self.cfg["register_s"]):
                del self.pending[rid]
                self._answer(sid, slot, False, "registration of slot %d: no answer from the router" % slot)
        links = self.worker.get("links", {}) if self.proc is not None else {}
        for s, _usb, _v, _e in self.specs:
            lk = links.get(str(s)) or {}
            if not (lk.get("present") and not lk.get("armed") and s in self.reg.recs):
                continue
            if now >= self.next_assist.get(s, 0.0):
                self.next_assist[s] = now + float(self.cfg["assist_s"])
                self.send_assist(s)

    # --- GPS warm start (H4) --------------------------------------------------------------------------------------
    def assist_data(self):
        clk = getattr(self.gw, "clock", None)
        utc = clk.now() if clk is not None and clk.gps_valid else None
        fresh = self.gw.telem_fresh() if hasattr(self.gw, "telem_fresh") else None
        pos = gpsassist.boat_fix(fresh, fresh is not None)
        extra = b""
        if self.cfg["assist_extra"]:
            try:
                with open(self.cfg["assist_extra"], "rb") as f:
                    extra = f.read(720 * 8)
            except OSError:
                extra = b""
        return utc, pos, extra

    def send_assist(self, slot):
        utc, pos, extra = self.assist_data()
        if utc is None and pos is None and not extra:
            self.assist[slot] = {"t": time.time(), "what": "", "ok": False,
                                 "why": "no GPS time / trusted boat position on the gateway yet"}
            return False
        return self.to_worker({"op": "assist", "slot": slot, "utc": utc, "pos": list(pos) if pos else None,
                               "extra": extra.hex()})

    # --- station messages -----------------------------------------------------------------------------------------
    def _answer(self, sid, slot, ok, text, problems=(), rec=None):
        st = self.gw.stations.stations.get(sid) if sid is not None else None
        if st is not None:
            st.send({"t": "uav_ack", "what": "register", "slot": slot, "ok": ok, "code": 0 if ok else ERR_UAV,
                     "text": text, "problems": list(problems), "rec": rec})

    def may_register(self, st):
        if st.role == "VIEWER":
            return "VIEWER: watch only"
        if not self.enabled:
            return "no UAV magazine on this boat"
        if self.state != ACTIVE or self.proc is None:
            return "UAV router offline (%s)" % (self.why or "-")
        return None

    def register_msg(self, st, m):
        """{"t": "uav_register", "slot": n}: read + check the drone on slot n's USB cable (read only)."""
        try:
            slot = int(m.get("slot"))
        except (TypeError, ValueError):
            slot = 0
        why = None if 1 <= slot <= self.n else "no such slot"
        why = why or self.may_register(st)
        if why is None and any(s == slot for _sid, s, _t in self.pending.values()):
            why = "slot %d is being registered" % slot
        if why:
            self.gw.audit("uav_register_refused", station=st.id, user=st.user, slot=slot, why=why)
            return st.send({"t": "uav_ack", "what": "register", "slot": slot, "ok": False, "code": ERR_UAV,
                            "text": "UAV refused: %s" % why, "problems": []})
        self.req_id += 1
        self.pending[self.req_id] = (st.id, slot, time.time())
        self.to_worker({"op": "register", "slot": slot, "id": self.req_id})
        self.gw.audit("uav_register", station=st.id, user=st.user, slot=slot)

    def _registered(self, m):
        rid = m.get("id")
        sid, slot, _t0 = self.pending.pop(rid, (None, int(m.get("slot", 0)), 0.0))
        reading = m.get("reading") or {}
        if not reading.get("heartbeat"):
            text = "slot %d: %s" % (slot, reading.get("why") or "no drone")
            self.gw.audit("uav_register_result", slot=slot, ok=False, why=text)
            return self._answer(sid, slot, False, text, [reading.get("why") or "no drone"])
        _s, _usb, vtx, elrs = self.specs[slot - 1]
        name = ((self.cfg["slots"][slot - 1] if slot - 1 < len(self.cfg["slots"]) else {}) or {}).get("name")
        rec = self.reg.register(slot, reading, name=name, vtx=vtx, elrs=elrs)
        self.gw.audit("uav_register_result", slot=slot, ok=rec["ok"], sysid=rec.get("sysid"), uid=rec.get("uid"),
                      fw=rec.get("fw"), problems=rec["problems"])
        self.to_worker(self._config())              # the router now drops other sysids on this slot
        if rec["ok"]:
            self.next_assist[slot] = 0.0            # the warm start at once
        text = ("slot %d registered: %s" % (slot, rec["name"]) if rec["ok"] else
                "slot %d not launchable: %s" % (slot, "; ".join(rec["problems"][:3])))
        self._answer(sid, slot, rec["ok"], text, rec["problems"], rec)

    # --- status ---------------------------------------------------------------------------------------------------
    def slot_status(self, now=None):
        now = time.time() if now is None else now
        links = self.worker.get("links", {}) if self.proc is not None else {}
        out = []
        for s, usb, vtx, elrs in self.specs:
            lk = links.get(str(s)) or {}
            r = self.reg.recs.get(s)
            live = {"sysid": lk.get("sysid")} if lk.get("present") else None
            ok, why = self.reg.launchable(s, live)
            a = self.assist.get(s)
            out.append({"slot": s, "usb": bool(usb), "present": bool(lk.get("present")), "link_error": lk.get("error"),
                        "registered": r is not None, "name": (r or {}).get("name"), "sysid": (r or {}).get("sysid"),
                        "uid": (r or {}).get("uid"), "fw": (r or {}).get("fw"),
                        "battery_pct": lk.get("batt_pct", (r or {}).get("battery_pct")),
                        "battery_v": lk.get("batt_v", (r or {}).get("battery_v")),
                        "gps_fix": lk.get("gps_fix", (r or {}).get("gps_fix")), "sats": lk.get("sats"),
                        "armed": bool(lk.get("armed")), "vtx": (r or {}).get("vtx") or vtx,
                        "elrs": (r or {}).get("elrs") or elrs, "mission": False,
                        "checked": (r or {}).get("checked"), "problems": (r or {}).get("problems", []),
                        "launchable": ok, "why": why,
                        "registering": any(ps == s for _sid, ps, _t in self.pending.values()),
                        "assist": None if not a else {"age_s": int(now - a["t"]), "what": a["what"], "ok": a["ok"],
                                                      "why": a["why"]}})
        return out

    def status(self):
        w = self.worker if self.proc is not None else {}
        return {"enabled": self.enabled, "state": self.state, "why": self.why, "sim": bool(self.cfg["sim"]),
                "slots": self.slot_status() if self.enabled else [], "next": self.next_slot(),
                "router": {"stats": w.get("stats", {}), "denied": w.get("denied", {}), "starts": self.starts},
                "registry_error": self.reg.error}

    def next_slot(self):
        """The next drone out = the lowest-numbered slot still present (slot 1 = top)."""
        if not self.enabled:
            return None
        for s in self.slot_status():
            if s["present"]:
                return s["slot"]
        return None
