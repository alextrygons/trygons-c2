"""The 6 slots of the magazine and their USB cables (CLAUDE.md V "Setup", docs/uav_design.md 3.1).

A slot = a FIXED USB port path on the powered hub, never taken from the drone (all flight controllers show the same
USB id; only the hub port tells them apart). gateway.json "uav": {"slots": [{"usb": ..., "vtx": ..., "elrs": ...}]}:
  "usbloc:1-2.3"                      the USB port location (pyserial "location"; python gateway.py --list-serial
                                      shows it) - Ubuntu and Windows (osdep.resolve_serial)
  "/dev/serial/by-path/..."           Ubuntu
  "udp:127.0.0.1:14601"               the simulated drone of that slot (gateway/uav/simdrone.py)

SlotProbe = the READ-ONLY registration sequence on one slot's cable (the router worker runs it, its own process):
HEARTBEAT (sysid / component) -> REQUEST_MESSAGE AUTOPILOT_VERSION / SYS_STATUS / BATTERY_STATUS / GPS_RAW_INT /
EKF_STATUS_REPORT -> PARAM_REQUEST_READ of every parameter of registry.wanted_params() (missing ones asked again,
at most `rounds` times; an optional one that never answers = not on this firmware) -> result(). Every wait is
timed (hb_s, total_s). It sends ONLY allow-listed, read-only messages; the router checks each with allow.py anyway.
"""
from . import allow
from . import mavlink
from . import registry

GCS_SYSID, GCS_COMPID = 255, 190          # the gateway = a GCS (MAV_COMP_ID_MISSIONPLANNER)
WANT_MSGS = ("AUTOPILOT_VERSION", "SYS_STATUS", "BATTERY_STATUS", "GPS_RAW_INT", "EKF_STATUS_REPORT")


def slot_specs(cfg, n=6):
    """[(slot, usb spec, vtx, elrs)] for slots 1..n from gateway.json "uav" "slots"."""
    lst = (cfg or {}).get("slots") or []
    out = []
    for i in range(1, n + 1):
        s = lst[i - 1] if i - 1 < len(lst) and isinstance(lst[i - 1], dict) else {}
        out.append((i, s.get("usb") or "", s.get("vtx"), s.get("elrs")))
    return out


class SlotProbe:
    def __init__(self, slot, now, hb_s=3.0, total_s=15.0, rounds=3, batch=25):
        self.slot = slot
        self.t0 = now
        self.hb_s, self.total_s, self.rounds, self.batch = hb_s, total_s, rounds, batch
        self.sysid = self.compid = None
        self.hb = None
        self.msgs = {}
        self.params = {}
        self.prearm = []
        self.req, self.opt = registry.wanted_params()
        self.round = 0
        self.next_ask = now
        self.done = False
        self.why = ""

    def feed(self, m):
        if m.name == "HEARTBEAT":
            if m.fields.get("autopilot") == 8:          # MAV_AUTOPILOT_INVALID: another component (camera, gimbal)
                return
            if self.sysid is None:
                self.sysid, self.compid = m.sysid, m.compid
            if m.sysid == self.sysid and m.compid == self.compid:
                self.hb = m.fields
            return
        if self.sysid is None or m.sysid != self.sysid:
            return
        if m.name == "PARAM_VALUE":
            self.params[m.fields["param_id"]] = m.fields["param_value"]
        elif m.name == "STATUSTEXT":
            t = m.fields.get("text", "")
            if "prearm" in t.lower().replace("-", ""):
                self.prearm.append(t)
                del self.prearm[:-5]
        elif m.name in ("AUTOPILOT_VERSION", "SYS_STATUS", "BATTERY_STATUS", "GPS_RAW_INT", "EKF_STATUS_REPORT"):
            self.msgs[m.name] = m.fields

    def missing(self):
        return [n for n in self.req + self.opt if n not in self.params]

    def tick(self, now):
        """-> [(name, fields)] to send now. Sets done when finished or timed out."""
        if self.done:
            return []
        if self.sysid is None:
            if now - self.t0 > self.hb_s:
                self.done, self.why = True, "no MAVLink from the slot's USB cable"
            return []
        if now - self.t0 > self.total_s:
            self.done, self.why = True, "timed out"
            return []
        have_msgs = all(n in self.msgs for n in ("AUTOPILOT_VERSION", "SYS_STATUS", "GPS_RAW_INT"))
        req_left = [n for n in self.req if n not in self.params]
        if have_msgs and not req_left and (self.round >= self.rounds or not self.missing()):
            self.done = True
            return []
        if now < self.next_ask:
            return []
        if self.round >= self.rounds and req_left:
            self.round = self.rounds - 1               # required ones: keep asking until total_s
        self.round += 1
        self.next_ask = now + 1.0
        out = []
        tgt = {"target_system": self.sysid, "target_component": self.compid}
        for n in WANT_MSGS:
            if n not in self.msgs:
                out.append(("COMMAND_LONG", dict(tgt, command=allow.CMD_REQUEST_MESSAGE,
                                                 param1=float(mavlink.MESSAGES[n][0]))))
        for n in self.missing()[:self.batch * 4]:
            out.append(("PARAM_REQUEST_READ", dict(tgt, param_id=n, param_index=-1)))
        return out

    def result(self):
        return {"slot": self.slot, "sysid": self.sysid, "compid": self.compid, "heartbeat": self.hb,
                "version": self.msgs.get("AUTOPILOT_VERSION"), "sys_status": self.msgs.get("SYS_STATUS"),
                "battery": self.msgs.get("BATTERY_STATUS"), "gps": self.msgs.get("GPS_RAW_INT"),
                "ekf": self.msgs.get("EKF_STATUS_REPORT"), "params": dict(self.params), "required": list(self.req),
                "prearm": list(self.prearm), "why": self.why}
