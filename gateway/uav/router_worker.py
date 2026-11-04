"""The UAV MAVLink router = its OWN process (CLAUDE.md V "Gateway architecture", docs/uav_design.md 4.2).

    started by gateway/uav/manager.py: python router_worker.py      (JSON lines: commands on stdin, events on stdout)

It owns every drone link (the 6 slot USB cables now; the 6 ELRS TX modules in U4) so no serial read / write ever
runs in the gateway's event loop. Each link has a reader thread; this process's main loop (20 Hz) decodes, checks
and answers. It ends by itself when the gateway closes its stdin (never a stray process).

  in   (MAVLink)  allow.check_in: not allow-listed = dropped + counted ("denied_in"); a frame from another sysid than
                  the slot's registered one = dropped + counted ("wrong_sysid"); bad CRC / unknown id = counted
  out  (MAVLink)  EVERY frame through tx() -> allow.check_out(name, fields, ctx) first: refused = not sent + counted
                  ("denied_out"); tx() is the ONLY place that calls mavlink.encode (UA4)

Commands: {"op": "config", "slots": [{"slot", "usb", "sysid"}], "gcs": [sysid, compid]}
          {"op": "register", "slot", "id"}       -> {"ev": "register", "id", "slot", "reading"}
          {"op": "assist", "slot", "utc", "pos": [lat, lon, acc] | null, "extra": hex}  -> {"ev": "assist", ...}
          {"op": "stop"}
Events:   {"ev": "status", "links": {slot: {...}}, "stats": {...}} once a second, {"ev": "log", "text"}
"""
import json
import os
import queue
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from uav import allow, gpsassist, mavlink, usbslots  # noqa: E402

TICK_S = 0.05
RECONNECT_S = 2.0
HB_LOST_S = 3.0


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
                self.ser = serial.Serial(dev, 115200, timeout=0.2)
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


class Router:
    def __init__(self):
        self.rxq = queue.Queue()
        self.links = {}                                    # slot -> Link (USB)
        self.expect = {}                                   # slot -> the registered sysid
        self.gcs = (usbslots.GCS_SYSID, usbslots.GCS_COMPID)
        self.probes = {}                                   # slot -> (SlotProbe, request id)
        self.stats = {"rx": 0, "tx": 0, "denied_in": 0, "denied_out": 0, "wrong_sysid": 0, "crc": 0, "unknown": 0}
        self.denied = {}                                   # name -> count (what was refused, both directions)
        self.next_status = 0.0
        self.next_hb = 0.0

    # --- the ONLY sender ------------------------------------------------------------------------------------------
    def ctx(self, link):
        armed = bool(link.hb.get("base_mode", 0) & 0x80)
        return {"port": link.kind, "armed": armed, "landed": link.kind == "usb" or not armed}

    def tx(self, link, name, fields, ctx=None):
        ok, why = allow.check_out(name, fields, self.ctx(link) if ctx is None else ctx)
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
            pr = self.probes.get(link.slot)
            if exp is not None and m.sysid != exp and pr is None:
                self.stats["wrong_sysid"] += 1
                continue
            if m.name == "HEARTBEAT" and m.fields.get("autopilot") != 8:
                link.hb, link.hb_t, link.sysid = m.fields, now, m.sysid
            elif m.name == "SYS_STATUS":
                link.info["batt_pct"] = m.fields.get("battery_remaining")
                link.info["batt_v"] = round(m.fields.get("voltage_battery", 0) / 1000.0, 2)
            elif m.name == "GPS_RAW_INT":
                link.info["gps_fix"] = m.fields.get("fix_type")
                link.info["sats"] = m.fields.get("satellites_visible")
            if pr is not None:
                pr[0].feed(m)
        for k in ("crc", "unknown"):
            self.stats[k] += link.parser.n[k] - n0.get(k, 0)

    # --- commands -------------------------------------------------------------------------------------------------
    def command(self, c, now):
        op = c.get("op")
        if op == "config":
            self.gcs = tuple(c.get("gcs") or self.gcs)
            want = {int(s["slot"]): s for s in c.get("slots", [])}
            for slot in list(self.links):
                if slot not in want or want[slot].get("usb") != self.links[slot].spec:
                    self.links.pop(slot).close("removed")
            for slot, s in want.items():
                if slot not in self.links and s.get("usb"):
                    self.links[slot] = Link(slot, "usb", s["usb"], self.rxq)
                self.expect[slot] = s.get("sysid")
        elif op == "register":
            slot = int(c["slot"])
            link = self.links.get(slot)
            if link is None:
                out({"ev": "register", "id": c.get("id"), "slot": slot,
                     "reading": {"slot": slot, "why": "no USB cable configured for this slot"}})
                return
            self.probes[slot] = (usbslots.SlotProbe(slot, now), c.get("id"))
        elif op == "assist":
            self.assist(c)
        elif op == "stop":
            raise SystemExit(0)

    def assist(self, c):
        slot = int(c["slot"])
        link = self.links.get(slot)
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
            link = self.links.get(slot)
            for name, fields in pr.tick(now):
                if link is not None and link.up:
                    self.tx(link, name, fields)
            if pr.done:
                del self.probes[slot]
                out({"ev": "register", "id": rid, "slot": slot, "reading": pr.result()})
        if now >= self.next_hb:
            self.next_hb = now + 1.0
            for link in self.links.values():
                if link.up:
                    self.tx(link, "HEARTBEAT", {"type": 6, "autopilot": 8, "base_mode": 0, "custom_mode": 0,
                                                "system_status": 4, "mavlink_version": 3})
        if now >= self.next_status:
            self.next_status = now + 1.0
            out({"ev": "status", "stats": self.stats, "denied": self.denied, "probing": sorted(self.probes),
                 "links": {str(s): {"up": l.up, "error": l.error, "present": l.up and now - l.hb_t < HB_LOST_S,
                                    "sysid": l.sysid, "armed": bool(l.hb.get("base_mode", 0) & 0x80),
                                    "mode": l.hb.get("custom_mode"), "hb_age": round(now - l.hb_t, 1) if l.hb_t
                                    else None, **l.info} for s, l in self.links.items()}})


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
