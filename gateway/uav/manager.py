"""The UAV magazine on the gateway (CLAUDE.md V, docs/uav_design.md): step U2 = the registry, the slot USB checks,
the GPS warm start and the simulated drones; step U3 = routes -> missions (uav_route), the launch sequence
(uav_launch_prepare -> the captain's YES / NO -> launch.py; uav_launch_stop), the hatch (hatch.py: the limit
switches, simhatch.py in the demo / tests; the relay through uavlink.py only) and uav_hatch_close.

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

import socket

from common import deep_merge, log
import uavlink
from . import allow, gpsassist, hatch, launch, mission, nofly, simhatch, uavcheck
from .registry import Registry
from .usbslots import slot_specs

HERE = os.path.dirname(os.path.abspath(__file__))
ACTIVE, OFFLINE = "ACTIVE", "OFFLINE"
ERR_UAV = 122                                  # stations.GW_RESULTS 122: UAV not registered / check failed
ERR_LAUNCH = 125                               # 125: launch conditions not met (also the hatch relay from a station)

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
    # U3: the hatch relay = n of the EXISTING boat relay table (an extra onoff relay, W57 b: >= 8 channels);
    # 0 / None = no hatch relay = no launch
    "hatch_relay": None,
    "hatch": {"port": "", "sim": None, "open_s": 20.0, "close_s": 20.0, "sim_travel_s": 6.0},
    "launch": {},                              # launch.DEFAULTS
    "check": {},                               # uavcheck.DEFAULTS
}


class UavManager:
    def __init__(self, cfg, gw):
        self.cfg = deep_merge(DEFAULTS, cfg or {})
        self.gw = gw
        self.enabled = bool(self.cfg["enabled"])
        self.n = int(self.cfg["n_slots"])
        if self.cfg["sim"] and not self.cfg["slots"]:
            b = int(self.cfg["sim_base_port"])
            self.cfg["slots"] = [{"usb": "udp:127.0.0.1:%d" % (b + i), "radio": "udp:127.0.0.1:%d" % (b + 10 + i),
                                  "vtx": "R%d" % min(8, i), "elrs": "tx%d" % i} for i in range(1, self.n + 1)]
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
        # --- U3 -------------------------------------------------------------------------------------------------
        try:
            self.hatch_relay = int(self.cfg.get("hatch_relay") or 0) or None
        except (TypeError, ValueError):
            self.hatch_relay = None
        hc = self.cfg["hatch"]
        self.hatch = hatch.Hatch(hc.get("open_s", 20.0), hc.get("close_s", 20.0))
        use_sim = hc.get("sim") if hc.get("sim") is not None else bool(self.cfg["sim"])
        self.simhatch = simhatch.SimHatch(hc.get("sim_travel_s", 6.0)) if self.enabled and use_sim else None
        self.switches = None if self.simhatch is not None or not hc.get("port") else \
            hatch.SerialSwitches(hc["port"])
        self.sky = None                          # (sim) what the simulated drones were told
        self.sim_ctl = None
        self.ccfg = dict(uavcheck.DEFAULTS, **(self.cfg["check"] or {}))
        self.airspace = nofly.Airspace(st.folder("state") if st is not None and st.usable else None)
        self.next_air = 0.0
        self.seq = launch.Launch(self, dict(self.cfg["launch"] or {}))
        self.routes = {}                         # slot -> [{"lat", "lon", "alt"}] (checked)
        self.loaded = {}                         # slot -> True: the mission is on the drone (read back)
        self.flying = {}                         # slot -> {"phase": launching / catchup / mission, "home", "t"}
        self.cbs = {}                            # router request id -> callback
        self.notices = []                        # {"t", "text", "level"} (last 12)
        self.relay_retry = None                  # [on, cb, until]: one retry of a hatch relay command (W55)
        self.relay_busy = False

    # --- the processes --------------------------------------------------------------------------------------------
    def start(self, loop):
        self.loop = loop
        if self.enabled:
            self._spawn_bg()
            if self.switches is not None:
                self.switches.start()

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
        lst = self.cfg["slots"] or []
        return {"op": "config", "slots": [{"slot": s, "usb": usb, "sysid": (self.reg.recs.get(s) or {}).get("sysid")
                                           if (self.reg.recs.get(s) or {}).get("ok") else None,
                                           "elrs": ((lst[s - 1] if s - 1 < len(lst) else {}) or {}).get("radio") or ""}
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
        if wait and self.switches is not None:
            self.switches.stop()
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
        elif ev in ("mission", "cmd"):
            cb = self.cbs.pop(m.get("id"), None)
            if cb is not None:
                if ev == "mission":
                    cb(bool(m.get("ok")), m.get("why", ""))
                else:
                    cb(m.get("result"), m.get("why", ""))

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
        self.u3_tick(now)

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

    # --- U3: notices, the hatch, the launch io ---------------------------------------------------------------------
    def notice(self, text, level="info"):
        """A NOTICE only (gw_status "uav" notices + the audit log): never an alarm, never a mode change."""
        self.notices.append({"t": round(time.time(), 1), "text": text, "level": level})
        del self.notices[:-12]
        self.gw.audit("uav_notice", text=text, level=level)
        log.info("uav: %s", text)

    def now(self):
        return time.time()

    def boat(self):
        return uavlink.boat_state(self.gw, self.hatch_relay, float(self.seq.cfg["launch_max_kn"]))

    def hatch_state(self):
        return self.hatch.state

    def hatch_why(self):
        return self.hatch.why

    def hatch_open_s(self):
        return self.hatch.open_s

    def hatch_commanded_open(self):
        return self.hatch.cmd == "open"

    def captain_ok(self, sid):
        st = self.gw.stations.stations.get(sid)
        return st is not None and (st.crew == "captain" or st.single_op)

    def slot(self, n):
        """What the router knows of slot n's drone now (USB link + radio link + the drone's own state)."""
        if self.proc is None:
            return {}
        age = time.time() - self.worker_t
        lk = (self.worker.get("links") or {}).get(str(n)) or {}
        rd = (self.worker.get("radio") or {}).get(str(n)) or {}
        dr = dict((self.worker.get("drones") or {}).get(str(n)) or {})
        if dr.get("alt_age") is not None:
            dr["alt_age"] += age
        dr["usb_present"] = bool(lk.get("present"))
        dr["radio_present"] = bool(rd.get("present"))
        dr["present"] = dr["usb_present"]
        if dr.get("battery_pct") is None:
            dr["battery_pct"] = dr.get("batt_pct")
        return dr

    def check_drone(self, n):
        """None = slot n may be launched now, else why."""
        d = self.slot(n)
        if not d.get("usb_present"):
            return "not on its slot USB cable"
        ok, why = self.reg.launchable(n, {"sysid": ((self.worker.get("links") or {}).get(str(n)) or {}).get("sysid")})
        if not ok:
            return why
        if d.get("armed"):
            return "armed in the slot"
        return None

    def mission_items(self, n, pos, take_alt):
        wps = self.routes.get(n)
        if not wps:
            return None, "no route for this UAV"
        if not pos or pos[0] is None:
            return None, "no trusted boat position"
        rec = self.reg.recs.get(n) or {}
        start = (pos[0], pos[1], 0.0)
        bad = uavcheck.check(wps, start, pos, self.ccfg, {"pct": self.slot(n).get("battery_pct"),
                                                          "capacity_mah": rec.get("capacity_mah")},
                             self.airspace, take_alt)
        if bad:
            return None, bad[1]
        return mission.build(wps, pos, take_alt, min(float(self.ccfg["uav_max_speed_ms"]), allow.MAX_SPEED_MS)), ""

    def _rid(self, cb):
        self.req_id += 1
        self.cbs[self.req_id] = cb
        return self.req_id

    def upload(self, n, items, via, cb):
        if not self.to_worker({"op": "mission", "slot": n, "id": self._rid(cb), "via": via, "items": items}):
            cb(False, "the router is offline")

    def cmd(self, n, via, command, params, cb):
        if not self.to_worker({"op": "cmd", "slot": n, "id": self._rid(cb), "via": via, "command": int(command),
                               "params": [float(p) for p in params]}):
            cb(None, "the router is offline")

    def mission_loaded(self, n):
        self.loaded[n] = True

    def armed(self, n):
        b = self.boat()
        self.flying[n] = {"phase": "launching", "home": b.get("pos"), "t": time.time()}

    def on_mission(self, n):
        if n in self.flying:
            self.flying[n]["phase"] = "mission"

    def catchup(self, n):
        if n in self.flying:
            self.flying[n].update(phase="catchup", next_try=0.0)

    def hatch_cmd(self, on, cb):
        """The hatch relay through uavlink -> stations.py (the HELM station's T_RELAY). cb(ok, why). A refusal NOT
        HELM (21) or HELM not agreeing yet (W55) = ONE retry once the gateway's grant and TELEM2 agree (<= 5 s),
        else cb(False) + the notice "HATCH NOT OPENED / CLOSED - take HELM ..."."""
        self.hatch.command(on, time.time())
        tries = [0]

        def done(result, text):
            if result == 0:
                self.relay_retry = None
                return cb(True, "")
            if (result is None or result == 21) and tries[0] == 0:
                tries[0] += 1
                self.relay_retry = {"on": on, "send": send, "until": time.time() + 5.0, "text": text, "cb": cb}
                return
            self.relay_retry = None
            verb = ("OPENED", "open") if on else ("CLOSED", "close")
            self.notice("HATCH NOT %s - take HELM and %s it (%s)" % (verb[0], verb[1], text), "caution")
            cb(False, text or "relay command refused (%s)" % result)

        def send():
            uavlink.hatch_relay(self.gw, self.hatch_relay, on, done)
        send()

    def u3_tick(self, now):
        # the hatch switches (the simulated hatch follows the AURIX's relay readback)
        if self.simhatch is not None:
            self.simhatch.step(now, uavlink.relay_on(self.gw, self.hatch_relay))
            sw = self.simhatch.read(now)
            sky = self.simhatch.sky()
            if sky != self.sky and self.cfg["sim"]:
                self.sky = sky
                self._sim_sky(sky)
        elif self.switches is not None:
            sw = self.switches.read(now)
        else:
            sw = (False, False, False, "no hatch switch port configured (gateway.json uav hatch port)")
        was = self.hatch.state
        st = self.hatch.update(now, sw)
        if st != was:
            self.gw.audit("uav_hatch", state=st, why=self.hatch.why)
            if st == hatch.FAULT:
                self.notice("HATCH FAULT: %s" % self.hatch.why, "caution")
        op = st == hatch.OPEN and bool(sw[0])
        self.to_worker({"op": "hatch", "open": op})
        # the one retry of a hatch relay command, once the HELM grant and TELEM2 agree
        rr = self.relay_retry
        if rr is not None:
            if self.gw.stations.helm_station()[0] is not None:
                self.relay_retry = None
                rr["send"]()
            elif now > rr["until"]:
                self.relay_retry = None
                verb = ("OPENED", "open") if rr["on"] else ("CLOSED", "close")
                self.notice("HATCH NOT %s - take HELM and %s it (%s)" % (verb[0], verb[1], rr["text"]), "caution")
                rr["cb"](False, rr["text"] or "HELM")
        if now >= self.next_air:
            self.next_air = now + 10.0
            threading.Thread(target=self.airspace.reload, daemon=True).start()
        self.seq.tick(now)
        self.catchup_tick(now)

    def _sim_sky(self, sky):
        try:
            if self.sim_ctl is None:
                self.sim_ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sim_ctl.setblocking(False)
            for s in range(1, self.n + 1):
                self.sim_ctl.sendto(json.dumps({"slot": s, "sky": bool(sky)}).encode(),
                                    ("127.0.0.1", int(self.cfg["sim_base_port"]) + 30))
        except OSError:
            pass

    def catchup_tick(self, now):
        """A drone of the climb fallback hovers in ALT_HOLD until GPS + EKF are good, then its mission starts."""
        for n, f in list(self.flying.items()):
            if f.get("phase") != "catchup" or f.get("busy") or now < f.get("next_try", 0.0):
                continue
            d = self.slot(n)
            if (d.get("gps_fix") or 0) >= 3 and ((d.get("ekf_flags") or 0) & launch.EKF_POS) == launch.EKF_POS:
                f["busy"] = True

                def go(result, why, n=n, f=f):
                    f["busy"] = False
                    if result == 0:
                        f["phase"] = "mission"
                        self.notice("UAV %d has GPS now - its mission started" % n)
                    else:
                        f["next_try"] = time.time() + 5.0
                self.cmd(n, "elrs", allow.CMD_MISSION_START, [mission.START_LAUNCH, 0], go)

    # --- U3: station messages --------------------------------------------------------------------------------------
    def _ack(self, st, what, ok, code, text, **kw):
        st.send(dict({"t": "uav_ack", "what": what, "ok": ok, "code": 0 if ok else code, "text": text}, **kw))
        self.gw.audit("uav_" + what, station=st.id, user=st.user, ok=ok, code=0 if ok else code, text=text)

    def route_msg(self, st, m):
        """{"t": "uav_route", "target": "next" | slot, "wps": [{"lat", "lon", "alt"}]}: a route for the next UAV /
        a chosen one; a FLYING one gets the new mission in flight (old one kept when refused)."""
        if st.role == "VIEWER":
            return self._ack(st, "route", False, ERR_UAV, "VIEWER: watch only")
        if not self.enabled:
            return self._ack(st, "route", False, ERR_UAV, "no UAV magazine on this boat")
        tgt = m.get("target", "next")
        slot = self.next_slot() if tgt == "next" else (int(tgt) if str(tgt).isdigit() else None)
        if slot is None or not 1 <= slot <= self.n:
            return self._ack(st, "route", False, ERR_UAV, "no UAV for this route (%s)" % tgt)
        try:
            wps = uavcheck.clean(m.get("wps"), self.ccfg["uav_wp_alt_m"])
        except ValueError as e:
            return self._ack(st, "route", False, uavcheck.ERR_ROUTE, "UAV route refused: %s" % e, slot=slot)
        boat = self.boat()
        rec = self.reg.recs.get(slot) or {}
        d = self.slot(slot)
        f = self.flying.get(slot)
        if f is not None and not d.get("usb_present"):
            if d.get("lat") is None or d.get("rel_alt") is None:
                return self._ack(st, "route", False, uavcheck.ERR_ROUTE, "UAV %d: its position is not known" % slot,
                                 slot=slot)
            start = (d["lat"], d["lon"], d["rel_alt"])
            bad = uavcheck.check(wps, start, boat.get("pos"), self.ccfg, {"pct": d.get("battery_pct"),
                                 "capacity_mah": rec.get("capacity_mah")}, self.airspace)
            if bad:
                return self._ack(st, "route", False, bad[0], "UAV %d route refused: %s" % (slot, bad[1]), slot=slot)
            home = f.get("home") or boat.get("pos") or (d["lat"], d["lon"])
            items = mission.build(wps, home, mission.init_alt(slot, self.seq.cfg["init_alt_base_m"],
                                                               self.seq.cfg["init_alt_step_m"]),
                                  min(float(self.ccfg["uav_max_speed_ms"]), allow.MAX_SPEED_MS))

            def up(ok, why):
                if not ok:
                    return self._ack(st, "route", False, uavcheck.ERR_ROUTE,
                                     "UAV %d: the new mission was not taken (%s) - the old one goes on" % (slot, why),
                                     slot=slot)

                def go(result, w):
                    ok2 = result == 0
                    if ok2:
                        self.routes[slot] = wps
                        f["phase"] = "mission"
                    self._ack(st, "route", ok2, uavcheck.ERR_ROUTE, "UAV %d: new mission %s" % (
                        slot, "started" if ok2 else "loaded but not started (%s)" % w), slot=slot)
                self.cmd(slot, "elrs", allow.CMD_MISSION_START, [mission.FLY_FROM, 0], go)
            return self.upload(slot, items, "elrs", up)
        if self.seq.busy() and self.seq.cur == slot and self.seq.phase not in ("CHECK", ""):
            return self._ack(st, "route", False, ERR_LAUNCH, "UAV %d is being launched now" % slot, slot=slot)
        pos = boat.get("pos")
        take = mission.init_alt(slot, self.seq.cfg["init_alt_base_m"], self.seq.cfg["init_alt_step_m"])
        bad = uavcheck.check(wps, (pos[0], pos[1], 0.0) if pos else None, pos, self.ccfg,
                             {"pct": d.get("battery_pct", rec.get("battery_pct")),
                              "capacity_mah": rec.get("capacity_mah")}, self.airspace, take)
        if bad:
            return self._ack(st, "route", False, bad[0], "UAV %d route refused: %s" % (slot, bad[1]), slot=slot)
        self.routes[slot], self.loaded[slot] = wps, False
        self._ack(st, "route", True, 0, "UAV %d: route of %d points stored (uploaded + read back at the launch)" % (
            slot, len(wps)), slot=slot)

    def launch_conditions(self, slots):
        """[] = a launch of these slots may start now, else the reasons (125 / 122)."""
        why = list(self.boat().get("launch_why") or [])
        if self.hatch.state not in (hatch.CLOSED, hatch.OPEN):
            why.append("hatch %s%s" % (self.hatch.state, (": " + self.hatch.why) if self.hatch.why else ""))
        if self.state != ACTIVE or self.proc is None:
            why.append("UAV router offline")
        for n in slots:
            w = self.check_drone(n)
            if w:
                why.append("UAV %d: %s" % (n, w))
            if not self.routes.get(n):
                why.append("UAV %d: no route" % n)
        return why

    def prepare_msg(self, st, m):
        """{"t": "uav_launch_prepare", "count": k}: the top k drones (default: every drone from the top that has a
        route). The captain approves it on screen 1 (YES / NO), a single operator answers his own."""
        if st.role == "VIEWER":
            return self._ack(st, "launch", False, ERR_LAUNCH, "VIEWER: watch only")
        if not self.enabled:
            return self._ack(st, "launch", False, ERR_LAUNCH, "no UAV magazine on this boat")
        if self.seq.busy():
            return self._ack(st, "launch", False, ERR_LAUNCH, "a launch sequence runs already")
        present = [s["slot"] for s in self.slot_status() if s["present"]]
        try:
            k = int(m.get("count") or 0)
        except (TypeError, ValueError):
            k = 0
        if k <= 0:
            for s in present:
                if not self.routes.get(s):
                    break
                k += 1
        slots = present[:max(0, k)]
        if not slots:
            return self._ack(st, "launch", False, ERR_LAUNCH, "UAV launch refused: no drone with a route at the top "
                             "of the magazine")
        why = self.launch_conditions(slots)
        if why:
            return self._ack(st, "launch", False, ERR_LAUNCH, "UAV launch refused: %s" % "; ".join(why[:4]),
                             problems=why)
        now = time.time()
        self.seq.prepare(slots, st.id, now)
        what = "UAV LAUNCH: %s (slot %s first)" % (", ".join("UAV %d" % s for s in slots), slots[0])

        def answer(ok, code):
            cap = st if st.single_op else self.gw.stations.captain()
            self.seq.approved(ok, cap.id if cap is not None and ok else None, time.time(),
                              "" if ok else "the captain did not approve (%s)" % code)
            self._ack(st, "launch", ok, code or ERR_LAUNCH, "UAV launch %s" % ("started" if ok else
                                                                              "not approved (%s)" % code))
        self._ack(st, "launch", True, 0, "UAV launch prepared: waiting for the captain's YES on screen 1",
                  pending=True)
        self.gw.stations.ask_captain(st, m, what, answer)

    def stop_msg(self, st, m):
        if st.role == "VIEWER":
            return self._ack(st, "launch_stop", False, ERR_LAUNCH, "VIEWER: watch only")
        ok = self.seq.stop("STOP by %s" % st.name, time.time())
        self._ack(st, "launch_stop", True, 0, "UAV launch stopped" if ok else "no launch runs")

    def hatch_close_msg(self, st, m):
        """The captain's own HATCH CLOSE after a stop (docs 5.3 / 5.6; W72 default: never by itself)."""
        if st.crew != "captain" and not st.single_op and not st.local:
            return self._ack(st, "hatch_close", False, 118, "HATCH CLOSE: the captain only")
        if self.seq.state in (launch.ASK, launch.RUN):
            return self._ack(st, "hatch_close", False, ERR_LAUNCH, "a launch sequence runs - STOP it first")
        if not self.hatch_relay:
            return self._ack(st, "hatch_close", False, ERR_LAUNCH, "no hatch relay configured")

        def done(ok, why):
            self._ack(st, "hatch_close", ok, ERR_LAUNCH, "hatch closing" if ok else "HATCH NOT CLOSED: %s - take "
                      "HELM and close it" % why)
        self.seq.keep_open = False
        self.hatch_cmd(False, done)

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
                        "elrs": (r or {}).get("elrs") or elrs,
                        "route": len(self.routes.get(s) or []), "mission": bool(self.loaded.get(s)),
                        "flying": (self.flying.get(s) or {}).get("phase"),
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
                "registry_error": self.reg.error,
                "hatch": dict(self.hatch.status(), relay=self.hatch_relay, sim=self.simhatch is not None),
                "launch": self.seq.status(time.time()), "notices": list(self.notices[-6:]),
                "airspace": self.airspace.status(),
                "flying": {str(k): {"phase": v.get("phase")} for k, v in self.flying.items()}}

    def next_slot(self):
        """The next drone out = the lowest-numbered slot still present (slot 1 = top)."""
        if not self.enabled:
            return None
        for s in self.slot_status():
            if s["present"]:
                return s["slot"]
        return None
