"""The UAV MAVLink router = its OWN process (CLAUDE.md V "Gateway architecture", docs/uav_design.md 4.2).

    started by gateway/uav/manager.py: python router_worker.py      (JSON lines: commands on stdin, events on stdout)

It owns every drone link - the slot's USB cable ("usb") and the drone's ELRS radio link ("elrs", step U3: the
launch needs the drone's altitude once its USB cable is gone) - so no serial read / write ever runs in the gateway's
event loop. Each link has a reader thread; this process's main loop (20 Hz) decodes, checks and answers. It ends by
itself when the gateway closes its stdin (never a stray process).

  in   (MAVLink)  allow.check_in: not allow-listed = dropped + counted ("denied_in"); a frame from another sysid than
                  the slot's registered one = dropped + counted ("wrong_sysid"); bad CRC / unknown id = counted
  out  (MAVLink)  EVERY frame through tx() -> allow.check_out(name, fields, ctx) first: refused = not sent + counted
                  ("denied_out"); tx() is the ONLY place that calls mavlink.encode (UA4). ctx is built HERE from
                  the state the router holds (U2 review LOW-3): armed / landed from the drone itself (HEARTBEAT,
                  EXTENDED_SYS_STATE, the relative altitude - LOW-4), the hatch OPEN switch from the last "hatch"
                  op (stale after HATCH_FRESH_S = not open), battery_land: set ONLY by the router's own battery rule
                  (step U4, battery.judge once a second per flying drone: left <= the energy to descend from here)
                  = the only way a LAND ever leaves the router (storm I82 reserved); while a drone lands for its
                  battery the router refuses every mode / mission / take-off / ARM / speed command for it

Telemetry (step U4, docs/uav_design.md 4.1 / 8 "Map"): per drone position, heading, altitude, ground speed, climb,
battery (%, V, A, mAh used), mode, the battery rule's eta, and the ELRS link quality (HEARTBEATs heard in the last
5 s of 5 + RADIO_STATUS rssi / remrssi when the radio sends it) - in every status event.

Commands: {"op": "config", "slots": [{"slot", "usb", "elrs", "sysid", "capacity_mah"}], "gcs": [sysid, compid],
           "battery": {uavcheck.DEFAULTS keys}}
          {"op": "register", "slot", "id"}       -> {"ev": "register", "id", "slot", "reading"}
          {"op": "assist", "slot", "utc", "pos": [lat, lon, acc] | null, "extra": hex}  -> {"ev": "assist", ...}
          {"op": "hatch", "open": bool}          the hatch OPEN switch (manager, 4 Hz)
          {"op": "mission", "slot", "id", "via": "usb" | "elrs" | "auto", "items": [...]}
                                                 -> upload + read-back -> {"ev": "mission", "id", "slot", "ok", "why"}
          {"op": "cmd", "slot", "id", "via", "command", "params": [7]}
                                                 -> COMMAND_LONG -> {"ev": "cmd", "id", "slot", "command", "result",
                                                    "why"} (result None = refused here / no answer)
          {"op": "stop"}
Events:   {"ev": "status", "links": {slot: usb link}, "radio": {slot: elrs link}, "drones": {slot: state},
          "stats": {...}} once a second (twice while a job runs), {"ev": "log", "text"}
"""
import collections
import json
import math
import os
import queue
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from uav import allow, battery, gpsassist, mavlink, mission, usbslots  # noqa: E402

TICK_S = 0.05
RECONNECT_S = 2.0
HB_LOST_S = 3.0
HATCH_FRESH_S = 1.0          # the manager sends the hatch switch 4 x a second; older = NOT open
ALT_FRESH_S = 2.0
ON_GROUND = 1                # MAV_LANDED_STATE_ON_GROUND
JOB_STEP_S = 1.5             # a mission step not answered -> asked again
JOB_TRIES = 4
JOB_TOTAL_S = 40.0
CMD_RESEND_S = 1.5
CMD_TOTAL_S = 4.0
LQ_WINDOW_S = 5.0            # the link quality: HEARTBEATs heard (1 Hz) in the last 5 s
LAND_RESEND_S = 2.0          # the battery LAND again while the drone is not in LAND yet
COPTER_LAND = 9
# refused for a drone that lands for its battery (a new mode / mission / take-off would end the landing)
NOT_WHILE_LANDING = (allow.CMD_DO_SET_MODE, allow.CMD_MISSION_START, allow.CMD_NAV_TAKEOFF,
                     allow.CMD_DO_CHANGE_SPEED)


def out(obj):
    try:
        sys.stdout.write(json.dumps(obj) + "\n")
        sys.stdout.flush()
    except (OSError, ValueError):
        os._exit(0)                                # the gateway is gone


class Link:
    """One drone cable / radio: "udp:host:port" (the simulator) or a serial name (osdep.resolve_serial: usbloc:...,
    /dev/serial/by-path/..., COMn). Received bytes go into the router's queue from a reader thread."""

    def __init__(self, slot, kind, spec, rxq):
        self.slot, self.kind, self.spec, self.rxq = slot, kind, spec, rxq
        self.up = False
        self.error = "not opened"
        self.next_open = 0.0
        self.sock = self.ser = None
        self.addr = None
        self.parser = mavlink.Parser()
        self.seq = 0
        self.gen = 0
        self.hb_t = 0.0
        self.sysid = None
        self.hb = {}
        self.info = {}
        self.foreign = None                                # (sysid, when) of a HEARTBEAT of another drone
        self.hbs = collections.deque(maxlen=20)            # HEARTBEAT times (the link quality)
        self.radio = {}                                    # RADIO_STATUS (rssi / remrssi / rxerrors) + its time

    def lq(self, now):
        """Link quality %: HEARTBEATs (1 Hz) heard in the last LQ_WINDOW_S; None = never heard."""
        if not self.hbs:
            return None
        n = sum(1 for t in self.hbs if now - t <= LQ_WINDOW_S)
        return int(min(100, round(100.0 * n / LQ_WINDOW_S)))

    def open(self, now):
        if self.up or now < self.next_open or not self.spec:
            return
        self.next_open = now + RECONNECT_S
        self.gen += 1
        gen = self.gen
        try:
            if self.spec.startswith("udp:"):
                _u, host, port = self.spec.split(":")
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(("127.0.0.1", 0))
                s.settimeout(0.2)
                self.sock, self.addr = s, (host, int(port))
                threading.Thread(target=self._udp_reader, args=(s, gen), daemon=True).start()
            else:
                import serial                              # pyserial
                import osdep
                dev = osdep.resolve_serial(self.spec)
                self.ser = serial.Serial(dev, 115200, timeout=0.2, write_timeout=0.2)
                threading.Thread(target=self._ser_reader, args=(self.ser, gen), daemon=True).start()
            self.up, self.error = True, ""
        except Exception as e:                             # not plugged in / no such port: retried
            self.up, self.error = False, str(e)[:80]

    def _udp_reader(self, s, gen):
        while gen == self.gen:
            try:
                data, _a = s.recvfrom(4096)
            except (socket.timeout, ConnectionResetError):
                continue                           # (Windows: an ICMP "port unreachable" of an earlier send)
            except OSError:
                break
            self.rxq.put((self, data))

    def _ser_reader(self, ser, gen):
        while gen == self.gen:
            try:
                data = ser.read(512)
            except Exception as e:
                self.rxq.put((self, e))
                break
            if data:
                self.rxq.put((self, data))

    def present(self, now):
        return self.up and self.hb_t > 0 and now - self.hb_t < HB_LOST_S

    def write(self, frame):
        try:
            if self.sock is not None:
                self.sock.sendto(frame, self.addr)
            elif self.ser is not None:
                self.ser.write(frame)
            return True
        except Exception as e:
            self.close(str(e)[:80])
            return False

    def close(self, why):
        self.gen += 1
        self.up, self.error = False, why
        for h in (self.sock, self.ser):
            try:
                if h is not None:
                    h.close()
            except Exception:
                pass
        self.sock = self.ser = None


class Drone:
    """What the router knows of the drone of one slot, from BOTH its links (the newest of each)."""

    def __init__(self):
        self.hb, self.hb_t = {}, 0.0
        self.compid = 1
        self.rel_alt = self.vz = self.lat = self.lon = None
        self.alt_t = 0.0
        self.landed_state, self.ext_t = None, 0.0
        self.fix = self.sats = None
        self.ekf_flags = None
        self.batt_pct = self.batt_v = None
        # step U4: the map / HUD values + the battery rule
        self.hdg = self.gs = self.climb = None
        self.batt_a = self.used_mah = None
        self.judge = None                                  # battery.judge() of the last second
        self.statustext = None                             # (severity, text) of the drone's newest STATUSTEXT

    def armed(self):
        return bool(self.hb.get("base_mode", 0) & 0x80)

    def landed(self, now):
        """On the ground (U2 review LOW-4): the drone says so (EXTENDED_SYS_STATE), else a fresh relative altitude
        below 0.5 m; unknown = NOT landed (a DISARM is then refused: never a DISARM of a flying drone)."""
        if self.landed_state is not None and now - self.ext_t < ALT_FRESH_S:
            return self.landed_state == ON_GROUND
        if self.rel_alt is not None and now - self.alt_t < ALT_FRESH_S:
            return self.rel_alt < 0.5
        return False

    def view(self, now):
        return {"mode": self.hb.get("custom_mode"), "armed": self.armed(), "hb_age": round(now - self.hb_t, 1)
                if self.hb_t else None, "rel_alt": None if self.rel_alt is None else round(self.rel_alt, 1),
                "vz": None if self.vz is None else round(self.vz, 2), "lat": self.lat, "lon": self.lon,
                "alt_age": round(now - self.alt_t, 1) if self.alt_t else None, "landed": self.landed(now),
                "gps_fix": self.fix, "sats": self.sats, "ekf_flags": self.ekf_flags, "batt_pct": self.batt_pct,
                "batt_v": self.batt_v, "hdg": None if self.hdg is None else round(self.hdg, 1),
                "gs": None if self.gs is None else round(self.gs, 1),
                "climb": None if self.climb is None else round(self.climb, 1),
                "batt_a": None if self.batt_a is None else round(self.batt_a, 1), "used_mah": self.used_mah,
                "judge": self.judge, "text": self.statustext}


class MissionJob:
    """Mission upload + read-back on one link (MAVLink mission protocol): COUNT -> the drone asks every item ->
    ACK -> REQUEST_LIST -> COUNT -> every item asked + compared (mission.same) -> ACK. Every step timed."""

    def __init__(self, rid, slot, link, items, now):
        self.id, self.slot, self.link, self.items = rid, slot, link, items
        self.phase = "upload"
        self.t0 = self.t_step = now
        self.tries = 0
        self.got = []
        self.next_rb = 0
        self.done = False
        self.ok, self.why = False, ""


class CmdJob:
    def __init__(self, rid, slot, link, command, params, now):
        self.id, self.slot, self.link, self.command, self.params = rid, slot, link, command, params
        self.t0 = self.t_sent = now
        self.sent = False


class Router:
    def __init__(self):
        self.rxq = queue.Queue()
        self.links = {}                                    # (slot, kind) -> Link
        self.drones = {}                                   # slot -> Drone
        self.expect = {}                                   # slot -> the registered sysid
        self.gcs = (usbslots.GCS_SYSID, usbslots.GCS_COMPID)
        self.probes = {}                                   # slot -> (SlotProbe, request id)
        self.jobs = {}                                     # slot -> MissionJob
        self.cmds = {}                                     # slot -> CmdJob
        self.hatch_open, self.hatch_t = False, 0.0
        self.battery_land = {}                             # slot -> True: the battery rule asked for LAND (U4)
        self.land_sent = {}                                # slot -> the last time the LAND went out
        self.capacity = {}                                 # slot -> BATT_CAPACITY mAh (the registry)
        self.bcfg = battery.cfg_of()                       # the battery rule's numbers (uavcheck.DEFAULTS)
        self.next_batt = 0.0
        self.stats = {"rx": 0, "tx": 0, "denied_in": 0, "denied_out": 0, "wrong_sysid": 0, "crc": 0, "unknown": 0}
        self.denied = {}                                   # name -> count (what was refused, both directions)
        self.next_status = 0.0
        self.next_hb = 0.0

    def link(self, slot, kind):
        return self.links.get((slot, kind))

    def drone(self, slot):
        d = self.drones.get(slot)
        if d is None:
            d = self.drones[slot] = Drone()
        return d

    # --- the ONLY sender ------------------------------------------------------------------------------------------
    def ctx(self, link, now=None):
        """What allow.check_out may know - from the router's OWN state only (U2 review LOW-3 / LOW-4)."""
        now = now_s() if now is None else now
        d = self.drone(link.slot)
        return {"port": link.kind, "armed": d.armed(), "landed": d.landed(now),
                "hatch_open": bool(self.hatch_open and now - self.hatch_t < HATCH_FRESH_S),
                "battery_land": bool(self.battery_land.get(link.slot))}

    def tx(self, link, name, fields):
        ok, why = allow.check_out(name, fields, self.ctx(link))
        if not ok:
            self.stats["denied_out"] += 1
            self.denied["out " + name] = self.denied.get("out " + name, 0) + 1
            out({"ev": "log", "text": "slot %d: not sent (%s)" % (link.slot, why)})
            return False
        link.seq = (link.seq + 1) & 0xFF
        frame = mavlink.encode(name, fields, link.seq, self.gcs[0], self.gcs[1])
        if link.write(frame):
            self.stats["tx"] += 1
            return True
        return False

    def tgt_of(self, slot):
        exp = self.expect.get(slot)
        lk = self.link(slot, "usb")
        sysid = exp if exp is not None else (lk.sysid if lk is not None else None)
        return {"target_system": sysid or 0, "target_component": self.drone(slot).compid or 1}

    # --- in -------------------------------------------------------------------------------------------------------
    def rx(self, link, data, now):
        if isinstance(data, Exception):
            link.close(str(data)[:80])
            return
        n0 = dict(link.parser.n)
        for m in link.parser.feed(data):
            self.stats["rx"] += 1
            ok, _why = allow.check_in(m.name)
            if not ok:
                self.stats["denied_in"] += 1
                self.denied["in " + m.name] = self.denied.get("in " + m.name, 0) + 1
                continue
            exp = self.expect.get(link.slot)
            pr = self.probes.get(link.slot) if link.kind == "usb" else None
            if exp is not None and m.sysid != exp and pr is None:
                self.stats["wrong_sysid"] += 1
                if m.name == "HEARTBEAT" and m.fields.get("autopilot") != 8:
                    link.foreign = (m.sysid, now)          # U2 review MED-1: another drone on this slot's cable
                continue
            if exp is None and link.kind == "elrs":
                continue                                   # a radio link only for a registered drone
            d = self.drone(link.slot)
            f = m.fields
            if m.name == "HEARTBEAT" and f.get("autopilot") != 8:
                link.hb, link.hb_t, link.sysid = f, now, m.sysid
                link.hbs.append(now)
                d.hb, d.hb_t, d.compid = f, now, m.compid
            elif m.name == "SYS_STATUS":
                link.info["batt_pct"] = f.get("battery_remaining")
                link.info["batt_v"] = round(f.get("voltage_battery", 0) / 1000.0, 2)
                d.batt_pct, d.batt_v = link.info["batt_pct"], link.info["batt_v"]
                if f.get("battery_remaining", -1) < 0:
                    d.batt_pct = link.info["batt_pct"] = None          # -1 = the drone does not know
                cb = f.get("current_battery", -1)
                if cb is not None and cb >= 0:
                    d.batt_a = cb / 100.0                               # cA
            elif m.name == "BATTERY_STATUS":
                if f.get("current_consumed", -1) >= 0:
                    d.used_mah = int(f["current_consumed"])
                if f.get("current_battery", -1) >= 0:
                    d.batt_a = f["current_battery"] / 100.0
            elif m.name == "GPS_RAW_INT":
                link.info["gps_fix"] = f.get("fix_type")
                link.info["sats"] = f.get("satellites_visible")
                d.fix, d.sats = f.get("fix_type"), f.get("satellites_visible")
            elif m.name == "GLOBAL_POSITION_INT":
                d.rel_alt, d.vz, d.alt_t = f.get("relative_alt", 0) / 1000.0, -f.get("vz", 0) / 100.0, now
                d.lat, d.lon = f.get("lat", 0) / 1e7, f.get("lon", 0) / 1e7
                d.gs = math.hypot(f.get("vx", 0), f.get("vy", 0)) / 100.0
                if f.get("hdg", 65535) != 65535:
                    d.hdg = f["hdg"] / 100.0
            elif m.name == "VFR_HUD":
                d.climb = f.get("climb")
                if d.hdg is None and f.get("heading") is not None:
                    d.hdg = float(f["heading"]) % 360.0
            elif m.name == "ATTITUDE":
                if f.get("yaw") is not None and math.isfinite(f["yaw"]):
                    d.hdg = math.degrees(f["yaw"]) % 360.0
            elif m.name == "RADIO_STATUS":
                link.radio = {"rssi": f.get("rssi"), "remrssi": f.get("remrssi"), "rxerrors": f.get("rxerrors"),
                              "t": now}
            elif m.name == "STATUSTEXT":
                d.statustext = (f.get("severity"), (f.get("text") or "")[:50])
            elif m.name == "EXTENDED_SYS_STATE":
                d.landed_state, d.ext_t = f.get("landed_state"), now
            elif m.name == "EKF_STATUS_REPORT":
                d.ekf_flags = f.get("flags")
            elif m.name == "AUTOPILOT_VERSION" and link.kind == "usb":
                uid = ("%016x" % f["uid"]) if f.get("uid") else (bytes(f.get("uid2") or []).hex() or None)
                link.info["uid"] = uid                     # the live UID (U2 review MED-1)
            if pr is not None:
                pr[0].feed(m)
            self.job_rx(link, m, now)
        for k in ("crc", "unknown"):
            self.stats[k] += link.parser.n[k] - n0.get(k, 0)

    def job_rx(self, link, m, now):
        j = self.jobs.get(link.slot)
        if j is not None and j.link is link and not j.done:
            f = m.fields
            if j.phase == "upload" and m.name == "MISSION_REQUEST_INT":
                s = int(f.get("seq", -1))
                if 0 <= s < len(j.items):
                    if not self.tx(link, "MISSION_ITEM_INT", dict(j.items[s], **self.tgt_of(link.slot))):
                        return self.job_end(j, False, "mission item %d refused by the allow-list" % s)
                    j.t_step, j.tries = now, 0
            elif j.phase == "upload" and m.name == "MISSION_ACK":
                if int(f.get("type", 1)) != 0:
                    self.job_end(j, False, "the drone refused the mission (MISSION_ACK %d)" % f.get("type", 1))
                else:
                    j.phase, j.t_step, j.tries = "count", now, 0
                    self.tx(link, "MISSION_REQUEST_LIST", dict(self.tgt_of(link.slot), mission_type=0))
            elif j.phase == "count" and m.name == "MISSION_COUNT":
                j.got = [None] * int(f.get("count", 0))
                j.phase, j.next_rb, j.t_step, j.tries = "items", 0, now, 0
                self.rb_next(j)
            elif j.phase == "items" and m.name == "MISSION_ITEM_INT":
                s = int(f.get("seq", -1))
                if 0 <= s < len(j.got):
                    j.got[s] = dict(f)
                    j.next_rb = s + 1
                    j.t_step, j.tries = now, 0
                    self.rb_next(j)
        c = self.cmds.get(link.slot)
        if c is not None and m.name == "COMMAND_ACK" and int(m.fields.get("command", -1)) == c.command:
            del self.cmds[link.slot]
            res = int(m.fields.get("result", 255))
            out({"ev": "cmd", "id": c.id, "slot": c.slot, "command": c.command, "result": res,
                 "why": "" if res == 0 else "MAV_RESULT %d" % res})

    def rb_next(self, j):
        if j.next_rb >= len(j.got):
            self.tx(j.link, "MISSION_ACK", dict(self.tgt_of(j.slot), type=0, mission_type=0))
            ok, why = mission.same(j.items, j.got)
            self.job_end(j, ok, why)
        else:
            self.tx(j.link, "MISSION_REQUEST_INT", dict(self.tgt_of(j.slot), seq=j.next_rb, mission_type=0))

    def job_end(self, j, ok, why):
        j.done, j.ok, j.why = True, ok, why
        self.jobs.pop(j.slot, None)
        out({"ev": "mission", "id": j.id, "slot": j.slot, "ok": ok, "why": why, "via": j.link.kind})

    # --- commands -------------------------------------------------------------------------------------------------
    def pick(self, slot, via, now):
        if via in ("usb", "elrs"):
            lk = self.link(slot, via)
            return lk if lk is not None and lk.present(now) else None
        for kind in ("usb", "elrs"):
            lk = self.link(slot, kind)
            if lk is not None and lk.present(now):
                return lk
        return None

    def command(self, c, now):
        op = c.get("op")
        if op == "config":
            self.gcs = tuple(c.get("gcs") or self.gcs)
            if c.get("battery") is not None:
                self.bcfg = battery.cfg_of(c.get("battery"))
            want = {}
            for s in c.get("slots", []):
                for kind in ("usb", "elrs"):
                    if s.get(kind):
                        want[(int(s["slot"]), kind)] = s[kind]
                self.expect[int(s["slot"])] = s.get("sysid")
                self.capacity[int(s["slot"])] = s.get("capacity_mah")
            for key in list(self.links):
                if key not in want or want[key] != self.links[key].spec:
                    self.links.pop(key).close("removed")
            for key, spec in want.items():
                if key not in self.links:
                    self.links[key] = Link(key[0], key[1], spec, self.rxq)
        elif op == "register":
            slot = int(c["slot"])
            if self.link(slot, "usb") is None:
                out({"ev": "register", "id": c.get("id"), "slot": slot,
                     "reading": {"slot": slot, "why": "no USB cable configured for this slot"}})
                return
            self.probes[slot] = (usbslots.SlotProbe(slot, now), c.get("id"))
        elif op == "assist":
            self.assist(c)
        elif op == "hatch":
            self.hatch_open, self.hatch_t = bool(c.get("open")), now
        elif op == "mission":
            slot = int(c["slot"])
            lk = self.pick(slot, c.get("via", "auto"), now)
            if lk is None or slot in self.jobs or self.battery_land.get(slot):
                return out({"ev": "mission", "id": c.get("id"), "slot": slot, "ok": False,
                            "why": "no link to the drone" if lk is None else (
                                "battery minimum: the drone is landing" if self.battery_land.get(slot) else
                                "an upload runs already")})
            j = self.jobs[slot] = MissionJob(c.get("id"), slot, lk, list(c["items"]), now)
            if not self.tx(lk, "MISSION_COUNT", dict(self.tgt_of(slot), count=len(j.items), mission_type=0)):
                self.job_end(j, False, "refused by the allow-list")
        elif op == "cmd":
            slot = int(c["slot"])
            lk = self.pick(slot, c.get("via", "auto"), now)
            res = {"ev": "cmd", "id": c.get("id"), "slot": slot, "command": int(c["command"]), "result": None}
            if lk is None:
                return out(dict(res, why="no link to the drone"))
            p = [float(x) for x in (list(c.get("params") or []) + [0.0] * 7)[:7]]
            if self.battery_land.get(slot) and (int(c["command"]) in NOT_WHILE_LANDING or (
                    int(c["command"]) == allow.CMD_COMPONENT_ARM_DISARM and p[0] == 1)):
                return out(dict(res, why="battery minimum: the drone is landing"))
            job = CmdJob(c.get("id"), slot, lk, int(c["command"]), p, now)
            if not self.send_cmd(job):
                return out(dict(res, why="refused by the allow-list"))
            self.cmds[slot] = job
        elif op == "stop":
            raise SystemExit(0)

    def send_cmd(self, j):
        f = dict(self.tgt_of(j.slot), command=j.command, confirmation=1 if j.sent else 0,
                 **{"param%d" % (i + 1): v for i, v in enumerate(j.params)})
        j.sent = True
        return self.tx(j.link, "COMMAND_LONG", f)

    def assist(self, c):
        slot = int(c["slot"])
        link = self.link(slot, "usb")
        res = {"ev": "assist", "slot": slot, "ok": False, "what": "", "why": ""}
        if link is None or not link.up or link.sysid is None or now_s() - link.hb_t > HB_LOST_S:
            res["why"] = "no drone on the slot's USB cable"
            return out(res)
        pos = tuple(c["pos"]) if c.get("pos") else None
        blob, what = gpsassist.assist_bytes(c.get("utc"), pos, bytes.fromhex(c.get("extra") or ""))
        if not blob:
            res["why"] = "nothing to inject (no GPS time on the gateway)"
            return out(res)
        sent = 0
        for k, msg in enumerate(gpsassist.split_messages(blob)):
            for fr in gpsassist.rtcm_frames(msg, (link.seq + k) & 0x1F):
                sent += 1 if self.tx(link, "GPS_RTCM_DATA", fr) else 0
        res.update(ok=sent > 0, what=what, frames=sent, why="" if sent else "refused (armed / not USB)")
        out(res)

    # --- the loop -------------------------------------------------------------------------------------------------
    def jobs_tick(self, now):
        for j in list(self.jobs.values()):
            if now - j.t0 > JOB_TOTAL_S:
                self.job_end(j, False, "mission upload timed out")
            elif now - j.t_step > JOB_STEP_S:
                j.tries += 1
                j.t_step = now
                if j.tries > JOB_TRIES or not j.link.up:
                    self.job_end(j, False, "no answer from the drone (%s)" % j.phase)
                elif j.phase == "upload":
                    self.tx(j.link, "MISSION_COUNT", dict(self.tgt_of(j.slot), count=len(j.items), mission_type=0))
                elif j.phase == "count":
                    self.tx(j.link, "MISSION_REQUEST_LIST", dict(self.tgt_of(j.slot), mission_type=0))
                else:
                    self.rb_next(j)
        for slot, c in list(self.cmds.items()):
            if now - c.t0 > CMD_TOTAL_S:
                del self.cmds[slot]
                out({"ev": "cmd", "id": c.id, "slot": slot, "command": c.command, "result": None,
                     "why": "no COMMAND_ACK from the drone"})
            elif now - c.t_sent > CMD_RESEND_S:
                c.t_sent = now
                self.send_cmd(c)

    def step(self, now):
        for link in self.links.values():
            link.open(now)
        while True:
            try:
                link, data = self.rxq.get_nowait()
            except queue.Empty:
                break
            self.rx(link, data, now)
        for slot, (pr, rid) in list(self.probes.items()):
            link = self.link(slot, "usb")
            for name, fields in pr.tick(now):
                if link is not None and link.up:
                    self.tx(link, name, fields)
            if pr.done:
                del self.probes[slot]
                out({"ev": "register", "id": rid, "slot": slot, "reading": pr.result()})
        self.jobs_tick(now)
        if now >= self.next_batt:
            self.next_batt = now + 1.0
            self.battery_tick(now)
        if now >= self.next_hb:
            self.next_hb = now + 1.0
            self.n_hb = getattr(self, "n_hb", 0) + 1
            for link in self.links.values():
                if link.up:
                    self.tx(link, "HEARTBEAT", {"type": 6, "autopilot": 8, "base_mode": 0, "custom_mode": 0,
                                                "system_status": 4, "mavlink_version": 3})
                if link.kind != "usb":
                    continue
                if not link.present(now):
                    link.info.pop("uid", None)             # the live UID is the drone ON the cable now
                elif self.n_hb % 10 == 0 and link.slot not in self.probes:
                    # U2 review MED-1: the live UID every 10 s (an unknown drone with the registered sysid)
                    self.tx(link, "COMMAND_LONG", dict(self.tgt_of(link.slot), command=allow.CMD_REQUEST_MESSAGE,
                                                       param1=float(mavlink.MESSAGES["AUTOPILOT_VERSION"][0])))
        if now >= self.next_status:
            self.next_status = now + 0.5
            self.status(now)

    # --- the battery minimum (step U4, docs/uav_design.md 6.2): the ONLY landing --------------------------------
    def battery_tick(self, now):
        """Once a second: every flying drone (armed, not on the ground, a fresh altitude) judged by battery.judge;
        "land" = battery_land latched + MAV_CMD_NAV_LAND (lat / lon 0 = where it is) through tx() - again every
        LAND_RESEND_S until the drone's HEARTBEAT says LAND; the latch ends when the drone is on the ground and
        disarmed. Nothing else sets battery_land."""
        for slot, d in self.drones.items():
            if not d.hb_t or now - d.hb_t > HB_LOST_S:
                continue
            flying = d.armed() and not d.landed(now) and d.rel_alt is not None and now - d.alt_t < ALT_FRESH_S
            if not flying:
                d.judge = None
                if self.battery_land.get(slot) and not d.armed():
                    self.battery_land.pop(slot, None)
                    self.land_sent.pop(slot, None)
                    out({"ev": "land", "slot": slot, "what": "down", "why": "on the ground, disarmed"})
                continue
            j = battery.judge(d.rel_alt, d.batt_pct, self.capacity.get(slot), d.batt_a, self.bcfg)
            d.judge = j
            if j["state"] == "land" and not self.battery_land.get(slot):
                self.battery_land[slot] = True
                out({"ev": "land", "slot": slot, "what": "start", "why": j["why"]})
            if self.battery_land.get(slot) and d.hb.get("custom_mode") != COPTER_LAND and \
                    now - self.land_sent.get(slot, 0.0) >= LAND_RESEND_S:
                lk = self.pick(slot, "auto", now)
                if lk is not None:
                    self.land_sent[slot] = now
                    self.tx(lk, "COMMAND_LONG", dict(self.tgt_of(slot), command=allow.CMD_NAV_LAND, confirmation=0,
                                                     param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0,
                                                     param6=0.0, param7=0.0))

    def status(self, now):
        def lk(l):
            return dict({"up": l.up, "error": l.error, "present": l.present(now), "sysid": l.sysid,
                         "armed": bool(l.hb.get("base_mode", 0) & 0x80), "mode": l.hb.get("custom_mode"),
                         "hb_age": round(now - l.hb_t, 1) if l.hb_t else None, "lq": l.lq(now),
                         "rssi": l.radio.get("rssi") if l.radio and now - l.radio["t"] < 5.0 else None,
                         "remrssi": l.radio.get("remrssi") if l.radio and now - l.radio["t"] < 5.0 else None,
                         "foreign_sysid": l.foreign[0] if l.foreign and now - l.foreign[1] < HB_LOST_S else None},
                        **l.info)
        out({"ev": "status", "stats": self.stats, "denied": self.denied, "probing": sorted(self.probes),
             "jobs": sorted(self.jobs), "hatch_open": self.ctx_hatch(now),
             "links": {str(s): lk(l) for (s, k), l in self.links.items() if k == "usb"},
             "radio": {str(s): lk(l) for (s, k), l in self.links.items() if k == "elrs"},
             "drones": {str(s): dict(d.view(now), landing=bool(self.battery_land.get(s)))
                        for s, d in self.drones.items() if d.hb_t}})

    def ctx_hatch(self, now):
        return bool(self.hatch_open and now - self.hatch_t < HATCH_FRESH_S)


def now_s():
    return time.monotonic()


def main():
    r = Router()
    cq = queue.Queue()

    def reader():
        for line in sys.stdin:
            try:
                cq.put(json.loads(line))
            except ValueError:
                continue
        cq.put({"op": "stop"})                             # stdin closed = the gateway is gone
    threading.Thread(target=reader, daemon=True).start()
    out({"ev": "log", "text": "router started"})
    try:
        while True:
            now = now_s()
            while True:
                try:
                    c = cq.get_nowait()
                except queue.Empty:
                    break
                try:
                    r.command(c, now)
                except (KeyError, TypeError, ValueError) as e:
                    out({"ev": "log", "text": "bad command %s: %s" % (c.get("op"), e)})
            r.step(now)
            time.sleep(TICK_S)
    except SystemExit:
        pass
    for link in r.links.values():
        link.close("stopped")
    os._exit(0)


if __name__ == "__main__":
    main()
