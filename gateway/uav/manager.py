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
from . import allow, battery, flightlog, gpsassist, hatch, launch, mission, nofly, pilot, simhatch, uavai, uavcheck
from .registry import Registry
from .usbslots import slot_specs

COPTER_MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND",
                16: "POSHOLD", 17: "BRAKE", 21: "SMART_RTL"}
VIDEO_DEFAULTS = {
    "enabled": False,                          # the six VRX + USB captures (U4); sim: the simulated VRX pictures
    "devices": {},                             # {"1": "dshow:video=USB Video" | "v4l2:/dev/v4l/by-path/..." | "sim"}
    "w": 640, "h": 480, "fps": 15, "kbps": 400,
    "encoder": "auto",                         # auto = Quick Sync when the probe works, else x264 ultrafast
    "always": [],                              # slots encoded also while the drone sits in its slot (bench)
    "stall_s": 5.0,
    "lost_s": 3.0,                             # a running video DOWN / STALLED this long = notice UAV n VIDEO LOST
}


class UavVideoTrack:
    """The ring's UAV-n folder for gateway/recorder.py (duck-typed like CamRecorder / the loudhailer's AudioTrack):
    its 1-minute .ts files go into every SAVE / KEEP like a camera's. The video worker writes them."""

    class _Proc:
        def __init__(self, track):
            self.track = track

        def poll(self):                         # "still being written" while that slot's capture runs
            return None if self.track.mgr.video_state(self.track.slot) in ("RUN", "STARTING") else 0

    def __init__(self, folder, mgr, slot):
        self.name, self.folder, self.mgr, self.slot = "UAV-%d" % slot, folder, mgr, slot
        self.state, self.restarts, self.since = "OFF", 0, time.time()
        self.proc = self._Proc(self)

    def segments(self):
        from recorder import seg_time
        out = []
        try:
            for n in os.listdir(self.folder):
                t = seg_time(n)
                if t is not None:
                    out.append((t, os.path.join(self.folder, n)))
        except OSError:
            pass
        out.sort()
        return out

    def tick(self, now):
        st = {"RUN": "REC", "STARTING": "STARTING"}.get(self.mgr.video_state(self.slot), "OFF")
        if st != self.state:
            self.state, self.since = st, now

    def stop(self):
        pass

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
    "check": {},                               # uavcheck.DEFAULTS (also the battery rule's numbers, battery.py)
    # --- U4 ---
    "video": {},                               # VIDEO_DEFAULTS
    "ai": {},                                  # uavai.DEFAULTS
    "telem_s": 0.5,                            # uav_telem to the stations that asked for it (2 Hz)
    # --- U5: manual control (pilot.DEFAULTS: speed / climb / yaw limits, stale / lost times, the edge look-ahead) ---
    "pilot": {},
}


def route_question(slot, wps, flying=True):
    """PF4-r MED-2c: the captain's question SHOWS the route: points, length, altitudes, the last point (English; the
    station translates it with the lang templates what.uav_route_fly / what.uav_route_seq)."""
    import math
    km = 0.0
    for a, b in zip(wps, wps[1:]):
        dy = (b["lat"] - a["lat"]) * 111320.0
        dx = (b["lon"] - a["lon"]) * 111320.0 * math.cos(math.radians(a["lat"]))
        km += math.hypot(dx, dy) / 1000.0
    alts = [float(w.get("alt") or 0.0) for w in wps]
    last = wps[-1]
    alt = "%d" % round(alts[0]) if min(alts) == max(alts) else "%d-%d" % (round(min(alts)), round(max(alts)))
    tpl = ("UAV ROUTE: change the mission of the flying UAV %d (%d points, %.1f km, last point %.5f %.5f, %s m)" if
           flying else "UAV ROUTE: change the route of UAV %d in the launch sequence (%d points, %.1f km, last point "
           "%.5f %.5f, %s m)")
    return tpl % (slot, len(wps), km, last["lat"], last["lon"], alt)


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
        self.switches = None if (not self.enabled or self.simhatch is not None or not hc.get("port")) else \
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
        self.hs_on = False                       # W59: the HIGH SPEED notice was given for this crossing
        self.hs_now = False                      # W59: the boat is faster than high_speed_kn now
        self.relay_retry = None                  # [on, cb, until]: one retry of a hatch relay command (W55)
        self.old_keys = [k for k in ("launch_max_kn",) if k in (self.cfg["launch"] or {})]
        if self.old_keys:
            # WA review M1: owner W59 removed the launch speed limit - an old gateway.json key is IGNORED, said once
            log.warning("uav: gateway.json uav launch %s is IGNORED (owner W59: no launch speed limit; above "
                        "high_speed_kn a notice only)", ", ".join(self.old_keys))
            try:
                self.notice("gateway.json: %s is ignored (no launch speed limit, W59)" % ", ".join(self.old_keys))
            except Exception:
                pass
        self.relay_busy = False
        self._load_extra()
        # --- U4: telemetry to the stations, the video worker, the UAV AI, the flight logs, the U4 notices ----------
        self.vcfg = deep_merge(VIDEO_DEFAULTS, self.cfg["video"] or {})
        if self.cfg["sim"] and not self.vcfg["devices"]:
            self.vcfg["devices"] = {str(s): "sim" for s in range(1, self.n + 1)}
        self.video_on = self.enabled and bool(self.vcfg["enabled"])
        self.vproc = None
        self.vq = queue.Queue(maxsize=50)
        self.vstatus, self.v_t, self.v_hello = {}, 0.0, {}
        self.v_next_start, self.v_spawning, self.v_starts = 0.0, False, 0
        self.v_run_sent = None
        self.tracks = {}
        if self.video_on and st is not None and st.usable:
            for s in range(1, self.n + 1):
                self.tracks["UAV-%d" % s] = UavVideoTrack(os.path.join(st.root, "video", "ring", "UAV-%d" % s), self, s)
        self.ai = uavai.UavAi(self.cfg["ai"], self)
        self.flog = flightlog.FlightLogs(st.folder("uav_logs") if (self.enabled and st is not None and st.usable)
                                         else None)
        self.telem_subs = set()                  # station ids that asked for uav_telem
        self.next_telem = 0.0
        self.next_u4 = 0.0
        self.mem = {}                            # slot -> what the U4 notices remember (link, range, stage, video)
        self.land_events = []                    # (t, slot, what, why) from the router's battery rule
        # --- U5: manual control + the pilot lock (pilot.py) -------------------------------------------------------
        self.pilots = pilot.Pilots(self, self.cfg["pilot"] or {})

    # --- the processes --------------------------------------------------------------------------------------------
    def start(self, loop):
        self.loop = loop
        if self.enabled:
            self._spawn_bg()
            if self.switches is not None:
                self.switches.start()
            self.flog.start()
            if self.video_on:
                self._vspawn_bg()

    # --- U4: the video worker (its OWN low-priority process) -------------------------------------------------------
    def _vspawn_bg(self):
        if self.v_spawning:
            return
        self.v_spawning = True
        self.v_next_start = time.time() + float(self.cfg["restart_s"])

        def run():
            try:
                self._vspawn()
            finally:
                self.v_spawning = False
        threading.Thread(target=run, daemon=True).start()

    def _vspawn(self):
        import toolpaths
        v = self.vcfg
        st = getattr(self.gw, "storage", None)
        rec = getattr(self.gw, "rec", None)
        ring = os.path.join(st.root, "video", "ring") if (st is not None and st.usable and rec is not None and
                                                          rec.enabled) else ""
        cams_on = bool((self.gw.cfg.get("cameras") or {}).get("enabled", True)) if hasattr(self.gw, "cfg") else False
        cmd = [self.cfg["python"] or sys.executable, os.path.join(HERE, "video_worker.py"),
               "--ffmpeg", toolpaths.ffmpeg() or "ffmpeg",
               "--rtsp-port", str(int(((self.gw.cfg.get("cameras") or {}) if hasattr(self.gw, "cfg") else {}).get(
                   "rtsp_port", 8554))),
               "--publish", "1" if cams_on else "0", "--ring", ring,
               "--seg-s", str(int(getattr(rec, "seg_s", 60) or 60)), "--w", str(int(v["w"])), "--h", str(int(v["h"])),
               "--fps", str(int(v["fps"])), "--kbps", str(int(v["kbps"])), "--encoder", str(v["encoder"]),
               "--stall-s", str(float(v["stall_s"]))]
        err = self._errlog("uav_video.log")
        try:
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, bufsize=0, cwd=HERE)
        except OSError as e:
            log.warning("uav: the video worker did not start: %s", e)
            return
        finally:
            if err is not subprocess.DEVNULL:
                err.close()
        q = queue.Queue(maxsize=50)
        self.vq = q
        self.v_t = time.time()
        self.v_starts += 1
        self.vproc = p
        self.v_run_sent = None
        threading.Thread(target=self._vreader, args=(p,), daemon=True).start()
        threading.Thread(target=self._writer, args=(p, q), daemon=True).start()
        self.to_video({"op": "config", "devices": dict(self.vcfg["devices"])})

    def _vreader(self, p):
        try:
            for line in p.stdout:
                try:
                    m = json.loads(line.decode("utf-8"))
                except ValueError:
                    continue
                if self.loop is not None:
                    self.loop.call_soon_threadsafe(self._from_video, p, m)
        except (OSError, ValueError, RuntimeError):
            pass

    def _from_video(self, p, m):
        if p is not self.vproc:
            return
        if m.get("ev") == "status":
            self.vstatus, self.v_t = m.get("slots") or {}, time.time()
        elif m.get("ev") == "hello":
            self.v_hello, self.v_t = m, time.time()

    def to_video(self, obj):
        if self.vproc is None:
            return False
        try:
            self.vq.put_nowait((json.dumps(obj) + "\n").encode("utf-8"))
            return True
        except queue.Full:
            return False

    def video_state(self, slot):
        if self.vproc is None:
            return "OFF"
        return (self.vstatus.get(str(slot)) or {}).get("state", "OFF")

    def video_running(self):
        """The slots whose video is encoded now (MediaMTX uav_<n> published)."""
        return [int(s) for s, v in self.vstatus.items() if v.get("state") in ("RUN", "STARTING")] \
            if self.vproc is not None else []

    def video_want(self):
        """Only drones that FLY are encoded (a drone in its slot = VTX pit mode) + the config's 'always' slots."""
        out = {int(s) for s in self.vcfg.get("always") or []}
        for s in range(1, self.n + 1):
            d = self.slot(s)
            if s in self.flying or (d.get("armed") and not d.get("usb_present")):
                out.add(s)
        return sorted(s for s in out if str(s) in {str(k) for k in self.vcfg["devices"]})

    def _video_tick(self, now):
        p = self.vproc
        # U4 review LOW: the worker probes the encoders first (up to 2 x PROBE_S) - more time until its final hello
        silent_s = 15.0 if self.v_hello.get("encoder") else 30.0
        if p is not None and (p.poll() is not None or now - self.v_t > silent_s):
            self._stop_video(wait=False)
            self.v_next_start = now + float(self.cfg["restart_s"])
            self.notice("UAV VIDEO process %s - restarting" % ("ended" if p.poll() is not None else "silent"), "caution")
        if self.vproc is None and now >= self.v_next_start:
            self._vspawn_bg()
        want = self.video_want()
        if want != self.v_run_sent and self.to_video({"op": "run", "slots": want}):
            self.v_run_sent = want

    def _stop_video(self, wait=True):
        p, self.vproc = self.vproc, None
        self.vstatus = {}
        if p is None:
            return
        try:
            self.vq.put_nowait(b'{"op": "stop"}\n')
            self.vq.put_nowait(None)
        except queue.Full:
            pass
        try:
            if wait:
                p.wait(4)
            elif p.poll() is None:
                p.kill()
        except subprocess.TimeoutExpired:
            p.kill()
        except OSError:
            pass

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
        return {"op": "config", "battery": dict(self.ccfg), "tlog_dir": self.flog.folder,
                "slots": [{"slot": s, "usb": usb, "sysid": (self.reg.recs.get(s) or {}).get("sysid")
                           if (self.reg.recs.get(s) or {}).get("ok") else None,
                           "capacity_mah": (self.reg.recs.get(s) or {}).get("capacity_mah"),
                           "name": (self.reg.recs.get(s) or {}).get("name") or "UAV %d" % s,
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
        if wait:
            self._stop_video(wait=True)
            self.ai.stop()
            self.flog.stop()
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
        elif ev == "land":
            # the router's battery rule (U4, docs 6.2): the ONLY landing
            s = int(m.get("slot") or 0)
            self.land_events.append((time.time(), s, m.get("what"), m.get("why", "")))
            del self.land_events[:-20]
            if m.get("what") == "start":
                self.notice("UAV %d LANDING NOW - battery minimum (where it is): %s" % (s, m.get("why", "")),
                            "caution")
            self.gw.audit("uav_land", slot=s, what=m.get("what"), why=m.get("why", ""))
        elif ev == "batt_stale":
            # U4 review MED-1: the battery % of a flying drone not heard: the gateway's battery rule cannot judge it
            s = int(m.get("slot") or 0)
            self.notice("UAV %d BATTERY DATA STALE - only the drone's own battery backstop" % s if m.get("stale") else
                        "UAV %d battery data back" % s, "caution" if m.get("stale") else "info")
        elif ev in ("mission", "cmd"):
            kc = self.cbs.pop(m.get("id"), None)
            if kc is not None:
                cb = kc[1]
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
            self._flush_cbs("the UAV router restarted")
            # restart after restart_s, never at once: no crash loop, and the OFFLINE notice reaches every station
            # (gw_status) before the router is back (gate U2: a restart within one tick was never shown)
            self.next_start = now + float(self.cfg["restart_s"])
            self._health(OFFLINE, "the router process %s - restarting" % ("ended" if dead else "is silent"))
        if self.proc is None and now >= self.next_start:
            self._spawn_bg()
        lost = getattr(self.reg, "lost", 0)
        if lost != getattr(self, "_reg_lost_told", 0):
            # WA/U6 review 2 LOW: a registry save that failed every try = said (a notice), never silent
            self._reg_lost_told = lost
            self.notice("UAV registry NOT saved (%s) - register again or check the gateway disk" %
                        (self.reg.error or "write failed"), "caution")
        for rid, (sid, slot, t0) in list(self.pending.items()):
            if now - t0 > float(self.cfg["register_s"]):
                del self.pending[rid]
                self._answer(sid, slot, False, "registration of slot %d: no answer from the router" % slot)
        links = self.worker.get("links", {}) if self.proc is not None else {}
        for s, _usb, _v, _e in self.specs:
            lk = links.get(str(s)) or {}
            if not (lk.get("present") and not lk.get("armed") and (self.reg.recs.get(s) or {}).get("ok")):
                continue
            if now >= self.next_assist.get(s, 0.0):
                self.next_assist[s] = now + float(self.cfg["assist_s"])
                self.send_assist(s)
        self.u3_tick(now)
        self.u4_tick(now)
        self.pilots.tick(now)

    # --- GPS warm start (H4) --------------------------------------------------------------------------------------
    def _load_extra(self):
        """The assist_extra file, read ONCE (U2 review LOW-1: from the start, never on the event loop) and filtered
        to UBX-MGA messages only (MED-3: a CFG-* / any other class could reconfigure or reset the drones' GPS -
        dropped + counted)."""
        self.extra, self.extra_dropped = b"", 0
        if not self.cfg["assist_extra"]:
            return
        try:
            with open(self.cfg["assist_extra"], "rb") as f:
                raw = f.read(720 * 8)
        except OSError:
            return
        self.extra, self.extra_dropped = gpsassist.only_mga(raw)

    def assist_data(self):
        clk = getattr(self.gw, "clock", None)
        # U2 review MED-4 / LOW-2: the time only while the boat's GPS time is fresh (TIME frames at 1 Hz)
        gt = getattr(clk, "gps_rx", 0.0) or 0.0
        utc = clk.now() if clk is not None and clk.gps_valid and (not gt or time.time() - gt < 5.0) else None
        fresh = self.gw.telem_fresh() if hasattr(self.gw, "telem_fresh") else None
        pos = gpsassist.boat_fix(fresh, fresh is not None)
        return utc, pos, self.extra

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

    def slot_busy(self, slot):
        """U3 review M4: no (un)registration while a launch runs or while that slot's drone flies (clearing / changing
        its record would make the router drop its ELRS frames: no telemetry, no battery rule, no route)."""
        if self.seq.busy():
            return "a launch sequence runs"
        d = self.slot(slot) if 1 <= slot <= self.n else {}
        if d.get("armed") or (slot in self.flying and not d.get("usb_present")):
            return "UAV %d is flying" % slot
        return None

    def register_msg(self, st, m):
        """{"t": "uav_register", "slot": n}: read + check the drone on slot n's USB cable (read only)."""
        try:
            slot = int(m.get("slot"))
        except (TypeError, ValueError):
            slot = 0
        why = None if 1 <= slot <= self.n else "no such slot"
        why = why or self.may_register(st) or self.slot_busy(slot)
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
        if rid not in self.pending:
            # U2 review LOW-10: an answer after its time-out (already refused to the station) never writes the registry
            self.gw.audit("uav_register_late", slot=m.get("slot"), id=rid)
            return
        sid, slot, _t0 = self.pending.pop(rid)
        if not 1 <= int(slot or 0) <= self.n:
            return
        reading = m.get("reading") or {}
        if not reading.get("heartbeat"):
            # U2 review MED-1: a failed / empty registration CLEARS the old record (never launched on an old check)
            if slot in self.reg.recs:
                self.reg.unregister(slot)
                self.to_worker(self._config())
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
        if not rec["ok"]:
            # step X2: a refused drone = a notice on every station naming the parameter(s) (gw_status uav notices)
            self.notice(text, "caution")
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
        return uavlink.boat_state(self.gw, self.hatch_relay, float(self.seq.cfg["high_speed_kn"]))

    def high_speed_tick(self, boat=None):
        """Owner W59 (2026-11-04): a launch goes at ANY boat speed; above high_speed_kn (5 kn) the NON-blocking notice
        HIGH SPEED - take-off risk, once when a launch is prepared / runs and the boat crosses it (0.5 kn hysteresis).
        A notice only - it never refuses, stops or slows anything."""
        b = boat if boat is not None else self.boat()
        kn, lim = b.get("sog_kn"), float(self.seq.cfg["high_speed_kn"])
        fast = kn is not None and (kn > lim or (self.hs_on and kn > lim - 0.5))
        if fast and not self.hs_on and self.seq.busy():
            self.notice("HIGH SPEED - take-off risk (%.1f kn)" % kn, "caution")
        self.hs_on = bool(fast) and (self.hs_on or self.seq.busy())
        self.hs_now = kn is not None and kn > lim
        return self.hs_now

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

    @staticmethod
    def live(lk):
        """What is on a slot's cable now for Registry.launchable: another drone's HEARTBEAT (the router drops its
        frames and reports its sysid - U2 review MED-1), the registered one (+ its live UID), or None."""
        if lk.get("foreign_sysid") is not None:
            return {"sysid": lk["foreign_sysid"], "uid": None}
        if lk.get("present"):
            return {"sysid": lk.get("sysid"), "uid": lk.get("uid")}
        return None

    def check_drone(self, n):
        """None = slot n may be launched now, else why."""
        d = self.slot(n)
        if not d.get("usb_present"):
            return "not on its slot USB cable"
        ok, why = self.reg.launchable(n, self.live((self.worker.get("links") or {}).get(str(n)) or {}))
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

    def _rid(self, cb, kind="cmd"):
        self.req_id += 1
        self.cbs[self.req_id] = (kind, cb)
        return self.req_id

    def _flush_cbs(self, why):
        """U3 review M5: the router restarted / went silent - every answer still owed is answered now (never a wait
        for ever)."""
        cbs, self.cbs = self.cbs, {}
        for kind, cb in cbs.values():
            try:
                cb(False, why) if kind == "mission" else cb(None, why)
            except Exception:
                log.exception("uav: a callback after the router restart")

    def router_ok(self):
        return self.state == ACTIVE and self.proc is not None

    def upload(self, n, items, via, cb):
        if not self.to_worker({"op": "mission", "slot": n, "id": self._rid(cb, "mission"), "via": via, "items": items}):
            self.cbs.pop(self.req_id, None)
            cb(False, "the router is offline")

    def cmd(self, n, via, command, params, cb):
        if not self.to_worker({"op": "cmd", "slot": n, "id": self._rid(cb), "via": via, "command": int(command),
                               "params": [float(p) for p in params]}):
            self.cbs.pop(self.req_id, None)
            cb(None, "the router is offline")

    def cancel(self, n, command=None):
        """U3 review M1: a command on its way to slot n's drone is not re-sent any more (answered "cancelled")."""
        self.to_worker({"op": "cancel", "slot": n, "command": command})

    def set_current(self, n, seq, via, cb):
        if not self.to_worker({"op": "set_current", "slot": n, "seq": int(seq), "id": self._rid(cb), "via": via}):
            self.cbs.pop(self.req_id, None)
            cb(None, "the router is offline")

    def mission_loaded(self, n):
        self.loaded[n] = True

    def armed(self, n):
        b = self.boat()
        self.flying[n] = {"phase": "launching", "home": b.get("pos"), "t": time.time()}

    def on_mission(self, n):
        if n in self.flying:
            self.flying[n]["phase"] = "mission"

    def launch_ended(self, n):
        """U6 review MED-3: the launch sequence let go of slot n's drone without a MISSION START (a STOP while it
        climbed out): it holds where it is (phase "hold") - never "launching" for ever (that blocked the pilot)."""
        f = self.flying.get(n)
        if f is not None and f.get("phase") == "launching":
            f["phase"] = "hold"

    def catchup(self, n):
        if n in self.flying:
            self.flying[n].update(phase="catchup", next_try=0.0)

    # --- U5 review MED-1: the pilot lock is exclusive ----------------------------------------------------------------
    def pilot_locked(self, n):
        """A pilot (the joystick) flies slot n's drone now: no mission / route / MISSION START for it."""
        return n in self.pilots.locks

    def pilot_block(self, n):
        """Why the joystick may NOT take slot n's drone now (None = it may): the launch sequence or its climb
        fallback still owns it (a take's GUIDED + zero setpoint could stop the climb / the mission start)."""
        if self.seq.busy() and self.seq.cur == n:
            return "UAV %d is being launched (%s) - wait until it flies its mission" % (n, self.seq.phase or "-")
        ph = (self.flying.get(n) or {}).get("phase")
        # U6 review MED-3: "launching" blocks only while the sequence still owns this drone (above); the climb
        # fallback's catch-up keeps it until GPS + EKF are good
        if ph == "catchup":
            return "UAV %d is still climbing out (%s) - wait until it flies its mission" % (n, ph)
        return None

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
        self.high_speed_tick()
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
        self.pend_expire_tick()
        for n, f in list(self.flying.items()):
            if f.get("phase") != "catchup" or f.get("busy") or now < f.get("next_try", 0.0):
                continue
            if self.pilot_locked(n):                     # U5 review MED-1 (a take is refused in catchup anyway)
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

    # --- U4: the map / telemetry / notices / flight log --------------------------------------------------------------
    def drone_view(self, n, boat_pos=None):
        """The map + HUD values of slot n's drone (stations: gw_status "uav" "drones" + uav_telem), or None."""
        d = self.slot(n)
        if not d or d.get("hb_age") is None:
            return None
        rd = (self.worker.get("radio") or {}).get(str(n)) or {}
        lk = (self.worker.get("links") or {}).get(str(n)) or {}
        j = d.get("judge") or {}
        rec = self.reg.recs.get(n) or {}
        out = {"slot": n, "name": rec.get("name") or "UAV %d" % n, "sysid": rec.get("sysid"),
               "lat": d.get("lat"), "lon": d.get("lon"), "alt": d.get("rel_alt"), "hdg": d.get("hdg"),
               "gs": d.get("gs"), "cog": d.get("cog"), "climb": d.get("climb"), "batt": d.get("battery_pct"), "batt_v": d.get("batt_v"),
               "batt_a": d.get("batt_a"), "mode": COPTER_MODES.get(d.get("mode"), str(d.get("mode"))),
               "armed": bool(d.get("armed")), "landed": bool(d.get("landed")), "in_slot": bool(d.get("usb_present")),
               "flying": (self.flying.get(n) or {}).get("phase"),
               "lq": rd.get("lq") if rd.get("present") else (0 if rd else None), "rssi": rd.get("rssi"),
               "link_age": rd.get("hb_age"), "usb_lq": lk.get("lq"),
               "land_eta": j.get("eta_s"), "landing": bool(d.get("landing")),
               "stage": self._stage(n, battery.stage(j.get("eta_s"), bool(d.get("landing"))),
                                    bool(d.get("armed") and not d.get("landed"))),
               "gps_fix": d.get("gps_fix"), "text": d.get("text"), "video": self.video_state(n),
               "alt_age": d.get("alt_age"),           # U5 review MED-2: pilot.setpoint holds on old data
               "roll": d.get("roll"), "pitch": d.get("pitch"), "att_age": d.get("att_age"),   # PF4: mark from video
               "camera": self._camera(n)}
        if d.get("lat") is not None and (d["lat"], d.get("lon")) == (0.0, 0.0):
            out["lat"] = out["lon"] = None                              # no position yet
        if boat_pos and out["lat"] is not None:
            out["dist_m"] = round(uavcheck.dist_m(boat_pos, (out["lat"], out["lon"])))
        return out

    def _camera(self, n):
        """PF4: slot n's FIXED camera mount from gateway.json uav slots[].camera {tilt_down_deg, hfov_deg, vfov_deg}
        (W133: no gimbal command anywhere), or None = the station's default."""
        try:
            cam = (self.cfg["slots"][n - 1] or {}).get("camera")
        except (IndexError, KeyError, TypeError):
            return None
        if not isinstance(cam, dict):
            return None
        out = {}
        for k in ("tilt_down_deg", "hfov_deg", "vfov_deg"):
            v = cam.get(k)
            if isinstance(v, (int, float)) and not isinstance(v, bool) and v == v and abs(v) < 1e4:
                out[k] = float(v)
        return out or None

    def _stage(self, n, raw, air):
        """U4 review LOW: the battery stage latched per flight (the most severe reached), cleared on the ground."""
        hold = getattr(self, "stage_hold", None)
        if hold is None:
            hold = self.stage_hold = {}
        if not air:
            hold.pop(n, None)
            return None
        hold[n] = battery.latch(hold.get(n), raw)
        return hold[n]

    def drones(self):
        pos = self.boat().get("pos") if self.enabled else None
        out = {}
        for s in range(1, self.n + 1):
            v = self.drone_view(s, pos)
            if v is not None:
                out[str(s)] = v
        return out

    def u4_tick(self, now):
        if self.video_on:
            self._video_tick(now)
        if now >= self.next_telem:
            self.next_telem = now + float(self.cfg["telem_s"])
            self.push_telem()
        if now < self.next_u4:
            return
        self.next_u4 = now + 1.0
        self.ai.tick(now)
        rng = float(self.ccfg["uav_rf_range_m"])
        pos = self.boat().get("pos")
        for s in range(1, self.n + 1):
            v = self.drone_view(s, pos)
            m = self.mem.setdefault(s, {"armed": False, "link": True, "range": 0.0, "stage": None, "video": None,
                                        "v_bad": None})
            if v is None:
                continue
            air = v["armed"] and not v["landed"]
            # the flight log: one file per flight (from the ARM), a line a second while armed
            if v["armed"]:
                self.flog.record(s, v["name"], {"sysid": v["sysid"], "lat": v["lat"], "lon": v["lon"],
                                                "alt_m": v["alt"], "hdg": v["hdg"], "gs_ms": v["gs"],
                                                "climb_ms": v["climb"], "batt_pct": v["batt"], "batt_v": v["batt_v"],
                                                "batt_a": v["batt_a"], "used_mah": self.slot(s).get("used_mah"),
                                                "mode": v["mode"], "armed": int(v["armed"]), "lq_pct": v["lq"],
                                                "land_eta_s": v["land_eta"], "landing": int(v["landing"]),
                                                "phase": v["flying"]}, new_flight=not m["armed"], now=now)
            elif m["armed"]:
                self.flog.end(s)
            m["armed"] = v["armed"]
            if not air:
                m["stage"] = None
                continue
            # ELRS link lost / back (the drone holds or flies its mission - never lands for it)
            ok = bool(v["lq"]) and (v["link_age"] is not None and v["link_age"] < 3.0)
            if ok != m["link"] and not v["in_slot"]:
                m["link"] = ok
                self.notice("UAV %d LINK LOST - it holds / flies its mission (never lands for it)" % s if not ok else
                            "UAV %d link back" % s, "caution" if not ok else "info")
            # the radio range
            dist = v.get("dist_m")
            if dist is not None and dist > 0.9 * rng and now - m["range"] > 60.0:
                m["range"] = now
                self.notice("UAV %d towards the radio range limit (%d m of %d m) - the crew decides" % (
                    s, dist, rng), "caution")
            # the battery minimum: CAUTION 60 s / 20 s before (the stations show it as a local caution)
            stg = v["stage"]
            if stg != m["stage"]:
                if stg in (60, 20):
                    self.notice("UAV %d LANDING IN %d s - battery minimum" % (s, stg), "caution")
                m["stage"] = stg
            # the video
            vs = v["video"]
            if s in (self.v_run_sent or []):
                bad = vs not in ("RUN", "STARTING")
                if bad and m["v_bad"] is None:
                    m["v_bad"] = now
                if not bad:
                    if m["video"] == "lost":
                        self.notice("UAV %d video back" % s)
                    m["v_bad"], m["video"] = None, "ok"
                elif now - m["v_bad"] >= float(self.vcfg["lost_s"]) and m["video"] != "lost":
                    m["video"] = "lost"
                    self.notice("UAV %d VIDEO LOST (%s) - nothing else changes" % (s, vs), "caution")

    def push_telem(self):
        """uav_telem (2 Hz) to the stations that asked for it ({"t": "uav_telem_on", "on": true}); never on the data
        saver (gw_status "uav" "drones" at 1 Hz is enough there)."""
        if not self.telem_subs:
            return
        saver = bool(getattr(self.gw, "data_saver", lambda: False)())
        if saver:
            return
        sts = self.gw.stations.stations
        msg = None
        for sid in list(self.telem_subs):
            st = sts.get(sid)
            if st is None:
                self.telem_subs.discard(sid)
                continue
            if msg is None:
                msg = {"t": "uav_telem", "t_s": round(time.time(), 2), "drones": self.drones()}
            st.send(msg)

    def telem_msg(self, st, m):
        (self.telem_subs.add if m.get("on", True) else self.telem_subs.discard)(st.id)

    def logs_msg(self, st, m):
        """{"t": "uav_logs"} -> the list; {"t": "uav_log", "name", "file", "fmt": "csv" | "gpx" | "tlog"} -> the text (tlog: base64). Read in
        a helper thread (never file work on the event loop)."""
        loop = self.loop

        def reply(obj):
            if loop is not None:
                loop.call_soon_threadsafe(st.send, obj)
            else:
                st.send(obj)

        def run():
            if m.get("t") == "uav_logs":
                reply({"t": "uav_logs", "logs": self.flog.list()})
            else:
                fmt = m.get("fmt") if m.get("fmt") in ("csv", "gpx", "tlog") else "csv"
                ok, text = self.flog.read(m.get("name"), m.get("file"), fmt)
                reply({"t": "uav_log", "ok": ok, "name": m.get("name"), "file": m.get("file"), "fmt": fmt,
                       "data": text if ok else "", "text": "" if ok else text})
        if st.role == "VIEWER" and m.get("t") == "uav_log":
            return st.send({"t": "uav_log", "ok": False, "text": "VIEWER: watch only"})
        threading.Thread(target=run, daemon=True).start()

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
        if self.pilot_locked(slot):
            # U5 review MED-1: the joystick flies it - no new mission / MISSION START under the pilot's hands
            lk = self.pilots.locks.get(slot)
            return self._ack(st, "route", False, pilot.ERR_PILOT, "UAV %d is flown by hand by %s - give it back "
                             "first" % (slot, getattr(lk, "name", "?")), slot=slot)
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
                        self._set_route(slot, wps, loaded=None)     # in flight: the mission flag unchanged
                        f["phase"] = "mission"
                    self._ack(st, "route", ok2, uavcheck.ERR_ROUTE, "UAV %d: new mission %s" % (
                        slot, "started" if ok2 else "loaded but not started (%s)" % w), slot=slot)
                # U3 review M6: ArduPilot ignores MISSION_START's first item while AUTO already runs - the new
                # mission's first waypoint is chosen with MISSION_SET_CURRENT, then MISSION START (when not in AUTO)
                def cur(result, w):
                    if result != 0:
                        return go(result, w)
                    self.cmd(slot, "elrs", allow.CMD_MISSION_START, [mission.FLY_FROM, 0], go)
                self.set_current(slot, mission.FLY_FROM, "elrs", cur)

            def apply_now():
                self.upload(slot, items, "elrs", up)
            # PF2-u (owner 2026-11-09): applying a mission to a FLYING drone needs the captain's YES - the captain's own
            # route applies directly; a copilot's / a single operator's is PENDING until the YES on screen 1 (a copilot
            # may still STORE a route for a drone in the magazine: preparation, no YES)
            if st.crew == "captain" and not st.single_op:
                self._pend_cancel(slot, "replaced by the captain's own route")   # PF4-r MED-2a: never an older YES later
                return apply_now()
            return self._route_pending(st, m, slot, f, wps, apply_now)
        if self.seq.busy() and self.seq.cur == slot and self.seq.phase not in ("CHECK", ""):
            return self._ack(st, "route", False, ERR_LAUNCH, "UAV %d is being launched now" % slot, slot=slot)
        pos = boat.get("pos")
        take = mission.init_alt(slot, self.seq.cfg["init_alt_base_m"], self.seq.cfg["init_alt_step_m"])
        bad = uavcheck.check(wps, (pos[0], pos[1], 0.0) if pos else None, pos, self.ccfg,
                             {"pct": d.get("battery_pct", rec.get("battery_pct")),
                              "capacity_mah": rec.get("capacity_mah")}, self.airspace, take)
        if bad:
            return self._ack(st, "route", False, bad[0], "UAV %d route refused: %s" % (slot, bad[1]), slot=slot)

        def store():
            if self.flying.get(slot) is not None or (self.seq.busy() and self.seq.cur == slot and
                                                     self.seq.phase not in ("CHECK", "")):
                return self._ack(st, "route", False, ERR_LAUNCH, "UAV %d is being launched now - the route was "
                                 "dropped" % slot, slot=slot)
            self._set_route(slot, wps)
            self._ack(st, "route", True, 0, "UAV %d: route of %d points stored (uploaded + read back at the launch)" % (
                slot, len(wps)), slot=slot)
        # PF4-r MED-2d: a drone of an asked / running launch sequence (ANY slot of seq.slots, the ASK phase too) is
        # not re-routed by a copilot / single operator without the captain's YES - the pending rule as in flight
        if self.seq.busy() and slot in (self.seq.slots or []) and not (st.crew == "captain" and not st.single_op):
            return self._route_pending(st, m, slot, None, wps, store)
        # PF4-r2 MED-1 (review of 64521b1): EVERY direct store - whoever sends it - first closes the slot's pending
        # route + its question (its station told), so an older captain's YES can never put the old route back
        self._pend_cancel(slot, "replaced by the captain's own route" if st.crew == "captain" and not st.single_op
                          else "replaced by a newer route")
        store()

    def _set_route(self, slot, wps, loaded=False):
        """The ONLY writer of a slot's stored route (PF4-r2 MED-1): every change counts the slot's route generation,
        so a pending route's YES applies only while the route it was asked against is still the stored one."""
        self.routes[slot] = wps
        if loaded is not None:
            self.loaded[slot] = loaded
        rg = self.__dict__.setdefault("route_gen", {})
        rg[slot] = rg.get(slot, 0) + 1

    def _route_pending(self, st, m, slot, f, wps, apply_now):
        """PF2-u: a route for a FLYING drone (or, PF4-r MED-2d, a drone of a launch sequence that is asked / running)
        from a copilot / single operator waits for the captain's YES (the launch approval's ask_captain). NO / timeout
        / a newer route / the captain's own route / the requesting station gone / the drone landed or a pilot took it =
        nothing changes, the old mission keeps flying. The question SHOWS the route (text: points, length, last point,
        altitudes + the points themselves for screen 1 / the map: PF4-r MED-2c)."""
        pend = self.__dict__.setdefault("pend_routes", {})
        self._pend_cancel(slot, "replaced by a newer route")          # PF4-r MED-2b: its question closes, its station told
        self.pend_n = getattr(self, "pend_n", 0) + 1
        rgen = (self.__dict__.get("route_gen") or {}).get(slot, 0)
        gen = pend[slot] = {"station": st.id, "user": st.user, "wps": wps, "f": f, "t": time.time(), "ask": None,
                            "st": st, "n": self.pend_n, "rgen": rgen}
        what = route_question(slot, wps, f is not None)
        my_n = gen["n"]

        def answer(ok, code):
            # PF4-r2 MED-1: the YES applies only to the EXACT pending route that was asked (its number) and only while
            # the slot's stored route is still the one it was asked against (no other store meanwhile)
            cur = pend.get(slot)
            if cur is not gen or cur.get("n") != my_n:   # a newer route replaced this question: it approves nothing
                return
            pend.pop(slot, None)
            if ok and (self.__dict__.get("route_gen") or {}).get(slot, 0) != rgen:
                self.gw.audit("uav_route_pending_stale", slot=slot, station=st.id, n=my_n)
                return self._ack(st, "route", False, ERR_UAV, "UAV %d: a newer route was stored meanwhile - this "
                                 "older route was not applied" % slot, slot=slot)
            if not ok:
                return self._ack(st, "route", False, code or ERR_UAV, "UAV %d: the new mission was not approved (%s) "
                                 "- the old one goes on" % (slot, code), slot=slot)
            if self.flying.get(slot) is not f or self.pilot_locked(slot):
                return self._ack(st, "route", False, uavcheck.ERR_ROUTE, "UAV %d: no longer flying its mission "
                                 "(landed / flown by hand) - the route was dropped" % slot, slot=slot)
            apply_now()
        self._ack(st, "route", True, 0, "UAV %d: route of %d points pending the captain's YES (the old mission "
                  "goes on)" % (slot, len(wps)), slot=slot, pending=True)
        extra = {"uav_route": {"slot": slot, "wps": [[w["lat"], w["lon"], w.get("alt")] for w in wps]}}
        try:
            aid = self.gw.stations.ask_captain(st, m, what, answer, extra=extra)
        except TypeError:                                            # a test double without "extra"
            aid = self.gw.stations.ask_captain(st, m, what, answer)
        if pend.get(slot) is gen and isinstance(aid, int) and not isinstance(aid, bool):
            gen["ask"] = aid

    def _pend_cancel(self, slot, why):
        """PF4-r MED-2a/b: drop the pending route of this slot: its open question closes on screen 1 (never answered
        later) and the station that asked is told."""
        pend = self.__dict__.get("pend_routes") or {}
        g = pend.pop(slot, None)
        if g is None:
            return
        self.gw.audit("uav_route_pending_replaced", slot=slot, station=g["station"], why=why)
        cancel = getattr(self.gw.stations, "cancel_ask", None)
        if g.get("ask") is not None and cancel is not None:
            cancel(g["ask"])
        st = g.get("st")
        if st is not None and (self.gw.stations.stations or {}).get(g["station"]) is st:
            self._ack(st, "route", False, ERR_UAV, "UAV %d: your pending route was %s - it will not be applied" % (
                slot, why), slot=slot, replaced=True)

    def pend_expire_tick(self):
        """PF2-u: a pending route is dropped when its drone is not flying its mission any more or a pilot took it;
        PF4-r LOW: also when the station that asked for it disconnected (its question closes on screen 1)."""
        pend = self.__dict__.get("pend_routes")
        for slot, g in list((pend or {}).items()):
            sts = self.gw.stations.stations or {}
            gone = g.get("st") is not None and sts.get(g["station"]) is not g["st"]
            # PF4-r2 MED-1: a pending route of a launch sequence (f None) goes when that sequence is no longer asked /
            # running or the slot is not in it any more (a later YES would store a route nobody asked for now)
            seq_over = g["f"] is None and (not self.seq.busy() or slot not in (self.seq.slots or []))
            if self.flying.get(slot) is not g["f"] or self.pilot_locked(slot) or gone or seq_over:
                pend.pop(slot, None)
                why = ("station gone" if gone else "the launch sequence ended" if seq_over else
                       "not flying its mission / flown by hand")
                self.gw.audit("uav_route_pending_dropped", slot=slot, station=g["station"], why=why)
                cancel = getattr(self.gw.stations, "cancel_ask", None)
                if g.get("ask") is not None and cancel is not None:
                    cancel(g["ask"])
                st = g.get("st")
                if not gone and st is not None:          # PF4-r2 LOW-5: never a silent drop - the station is told
                    self._ack(st, "route", False, ERR_UAV, "UAV %d: your pending route was dropped (%s) - it will "
                              "not be applied" % (slot, why), slot=slot, dropped=True)

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
        what = "%s (slot %s first)" % (", ".join("UAV %d" % s for s in slots), slots[0])
        self.hs_on = False
        if self.high_speed_tick():                     # W59: the notice + the captain's question says it (never refused)
            what = "HIGH SPEED - take-off risk (%.1f kn): %s" % (self.boat().get("sog_kn"), what)
        what = "UAV LAUNCH: " + what

        gen = self.seq.gen

        def answer(ok, code):
            if gen != self.seq.gen:                    # U3 review L3: an answer to an OLDER question approves nothing
                return
            cap = st if st.single_op else self.gw.stations.captain()
            self.seq.approved(ok, cap.id if cap is not None and ok else None, time.time(),
                              "" if ok else "the captain did not approve (%s)" % code)
            self._ack(st, "launch", ok, code or ERR_LAUNCH, "UAV launch %s" % ("started" if ok else
                                                                              "not approved (%s)" % code))
        self._ack(st, "launch", True, 0, "UAV launch prepared: waiting for the captain's YES on screen 1",
                  pending=True)
        self.gw.stations.ask_captain(st, m, what, answer)

    def unregister_msg(self, st, m):
        """{"t": "uav_unregister", "slot": n} (U2 review MED-1): forget a slot's record (never while a launch runs)."""
        try:
            slot = int(m.get("slot"))
        except (TypeError, ValueError):
            slot = 0
        why = "VIEWER: watch only" if st.role == "VIEWER" else ("no such slot" if not 1 <= slot <= self.n else
                                                                 self.slot_busy(slot))
        if why:
            return self._ack(st, "unregister", False, ERR_UAV, "UAV refused: %s" % why, slot=slot)
        self.reg.unregister(slot)
        self._pend_cancel(slot, "dropped: the slot was unregistered")
        self.routes.pop(slot, None)
        self.loaded.pop(slot, None)
        rg = self.__dict__.setdefault("route_gen", {})
        rg[slot] = rg.get(slot, 0) + 1
        self.to_worker(self._config())
        self._ack(st, "unregister", True, 0, "slot %d unregistered" % slot, slot=slot)

    def stop_msg(self, st, m):
        if st.role == "VIEWER":
            return self._ack(st, "launch_stop", False, ERR_LAUNCH, "VIEWER: watch only")
        ok = self.seq.stop("STOP by %s" % st.name, time.time())
        self._ack(st, "launch_stop", True, 0, "UAV launch stopped" if ok else "no launch runs")

    def hatch_close_msg(self, st, m):
        """The captain's own HATCH CLOSE after a stop (docs 5.3 / 5.6; owner W72 2026-11-04: never by itself)."""
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
            live = self.live(lk)
            ok, why = self.reg.launchable(s, live)
            a = self.assist.get(s)
            out.append({"slot": s, "usb": bool(usb), "present": live is not None, "link_error": lk.get("error"),
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
                "high_speed": bool(self.hs_now),          # W59: the station's non-blocking HIGH SPEED line
                "airspace": self.airspace.status(),
                "flying": {str(k): {"phase": v.get("phase")} for k, v in self.flying.items()},
                # U4: the map / HUD (also via uav_telem 2 Hz), the video worker, the UAV AI, the flight logs
                "drones": self.drones() if self.enabled else {},
                "video": {"enabled": self.video_on, "running": self.vproc is not None,
                          "encoder": self.v_hello.get("encoder"), "starts": self.v_starts,
                          "slots": dict(self.vstatus), "run": self.v_run_sent or []},
                "ai": self.ai.status() if self.enabled else {},
                # U5: who flies which UAV by hand (pilot.py; the station shows «JOYSTICK → UAV n»)
                "pilots": self.pilots.status() if self.enabled else {},
                "flightlog": {"lines": self.flog.lines, "dropped": self.flog.dropped, "errors": self.flog.errors}}

    def next_slot(self):
        """The next drone out = the lowest-numbered slot still present (slot 1 = top)."""
        if not self.enabled:
            return None
        for s in self.slot_status():
            if s["present"]:
                return s["slot"]
        return None
