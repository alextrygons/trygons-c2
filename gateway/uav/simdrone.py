"""Simulated drones of the UAV magazine (CLAUDE.md V "Sim", docs/uav_design.md 9 "Simulation", W67).

    python simdrone.py --base-port 14600 --n 6 [--fault 2:no_fix] [--origin 37.82,23.60]

ArduPilot SITL copters are not practical on Windows (WSL / Cygwin builds; SITL only as an optional cross-check on
Ubuntu, W67), and pymavlink's "mavlink drone simulators" need SITL too - so this is an OWN small simulator: six
ArduCopter-like quads speaking the SAME MAVLink 2 subset as the real ones (gateway/uav/mavlink.py; pymavlink is
installable on Windows by pip, but LGPL-3.0 and not needed - UA2 uses it only as a cross-check of the codec).

Each drone = two UDP links (the real ones are serial ports): its slot's USB cable on base + slot and its ELRS radio
link on base + 10 + slot (it answers to whoever sent to it last). A control port base + 30 takes JSON lines
{"slot": n, "fault": name, "on": true} / {"slot": n, "sky": true} / {"slot": n, "out": true} (the hatch is open /
it left its slot).

Behaviour (what the gateway depends on): HEARTBEAT 1 Hz, SYS_STATUS / GLOBAL_POSITION_INT / ATTITUDE / VFR_HUD 2 Hz,
GPS_RAW_INT / BATTERY_STATUS / EKF_STATUS_REPORT 1 Hz; parameters (read: PARAM_REQUEST_READ / _LIST; PARAM_SET is
ignored like an unknown message - the gateway never sends it); REQUEST_MESSAGE / SET_MESSAGE_INTERVAL; ARM (refused
without a 3D fix in a GPS mode, never with motor_fail climbing), DISARM on the ground, modes GUIDED / AUTO / LOITER /
ALT_HOLD / BRAKE / LAND / RTL (RTL only so a test can show the gateway never asks for it); TAKEOFF; the mission
protocol (upload + read-back, items TAKEOFF / WAYPOINT / CHANGE_SPEED / LOITER_UNLIM); GUIDED velocity setpoints
(stop after 3 s without a new one, like ArduPilot); EXTENDED_SYS_STATE landed_state 1 Hz; MISSION_START from param1 (seq 0 =
HOME is never flown); a take-off in ALT_HOLD only with param3 = 1 (ArduCopter's user take-off without GPS); battery (capacity, hover current, critical -> LAND = the
BATT_FS_CRT_ACT backstop); GPS: no sky in the closed magazine, then a fix after ttf_cold / ttf_warm (MGA-INI time +
position injected by GPS_RTCM_DATA) / ttf_hot (+ ephemeris); EKF failsafe without GPS in flight = ALT_HOLD (drifts).
Faults: no_fix, motor_fail, link_loss (ELRS silent), silent (nothing at all), battery_low, bad_param (FS_GCS_ENABLE 1),
extra_output (SERVO9_FUNCTION 28 = gripper), gpio_output (SERVO10_FUNCTION -1), rc_not_ignored (RC_OPTIONS 0),
release_relay (RELAY1_FUNCTION 1), armed (armed in its slot), wrong_type (a plane), spam_denied (it also sends
SERVO_OUTPUT_RAW / MOUNT_CONTROL - not on the allow-list: the router drops + counts them).
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from uav import gpsassist, mavlink  # noqa: E402

FW = (4 << 24) | (5 << 16) | (7 << 8) | 255            # 4.5.7 official
SENS = 0x1 | 0x2 | 0x4 | 0x8 | 0x20                    # gyro, accel, mag, baro, GPS
PREARM = 0x10000000
M_ALT_HOLD, M_AUTO, M_GUIDED, M_LOITER, M_RTL, M_LAND, M_BRAKE = 2, 3, 4, 5, 6, 9, 17
GPS_MODES = (M_AUTO, M_GUIDED, M_LOITER, M_RTL, M_BRAKE)
R_EARTH = 6378137.0


def good_params():
    p = {"FS_THR_ENABLE": 0, "FS_GCS_ENABLE": 0, "FS_EKF_ACTION": 2, "FS_DR_ENABLE": 0, "FENCE_ACTION": 0,
         "BATT_FS_LOW_ACT": 0, "BATT_FS_CRT_ACT": 1, "BATT_CRT_VOLT": 13.6, "BATT_CRT_MAH": 0, "RC_OPTIONS": 1,
         "FRAME_CLASS": 1, "GRIP_ENABLE": 0, "CHUTE_ENABLED": 0, "MNT1_TYPE": 0, "SCR_ENABLE": 0, "WINCH_TYPE": 0,
         "WPNAV_SPEED": 1000, "BATT_CAPACITY": 3000}
    for i in range(1, 17):
        p["SERVO%d_FUNCTION" % i] = 32 + i if i <= 4 else 0
        p["RC%d_OPTION" % i] = 0
    for i in range(1, 7):
        p["RELAY%d_FUNCTION" % i] = 0
        p["RELAY%d_PIN" % i] = -1
    return p


class Link:
    def __init__(self, port, host="127.0.0.1"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.02)
        self.peer = None
        self.parser = mavlink.Parser()
        self.seq = 0


class Drone:
    def __init__(self, slot, sysid, usb_port, elrs_port, origin=(37.82, 23.60), clock=time.time,
                 ttf=(30.0, 12.0, 3.0)):
        self.slot, self.sysid, self.compid = slot, sysid, 1
        self.clock = clock
        self.usb, self.elrs = Link(usb_port), Link(elrs_port)
        self.params = good_params()
        self.uid = 0x3A000000 + slot * 0x1111
        self.faults = set()
        self.ttf_cold, self.ttf_warm, self.ttf_hot = ttf
        self.lock = threading.Lock()
        self.reset(origin)

    def reset(self, origin):
        self.lat0, self.lon0 = origin
        self.n = self.e = 0.0
        self.alt = 0.0                 # above the launch point (m)
        self.vn = self.ve = self.vz = 0.0
        self.yaw = 0.0
        self.armed = False
        self.mode = M_LOITER
        self.in_slot, self.sky, self.sky_t = True, False, None
        self.landed = True
        self.used_mah, self.capacity = 0.0, 3000.0
        self.assist = {}               # "time" / "pos" / "eph" -> when injected
        self.fix = 0
        self.mission, self.up_count, self.up_items, self.cur = [], 0, {}, 0
        self.speed = 10.0
        self.target_alt = None
        self.guided_v, self.guided_t = (0.0, 0.0, 0.0, 0.0), 0.0
        self.rtcm = {}
        self.got = []                  # names of the messages received (tests: the router's output)
        self.last = {}
        self.t = self.clock()
        self.events = []

    # --- faults -------------------------------------------------------------------------------------------------
    def fault(self, name, on=True):
        with self.lock:
            (self.faults.add if on else self.faults.discard)(name)
            p = self.params
            if name == "bad_param":
                p["FS_GCS_ENABLE"] = 1 if on else 0
            elif name == "extra_output":
                p["SERVO9_FUNCTION"] = 28 if on else 0
            elif name == "gpio_output":
                p["SERVO10_FUNCTION"] = -1 if on else 0
            elif name == "rc_not_ignored":
                p["RC_OPTIONS"] = 0 if on else 1
            elif name == "release_relay":
                p["RELAY1_FUNCTION"] = 1 if on else 0
            elif name == "armed":
                self.armed = on
            elif name == "battery_low":
                self.used_mah = self.capacity * (0.9 if on else 0.0)

    # --- radio --------------------------------------------------------------------------------------------------
    def send(self, link, name, fields):
        if "silent" in self.faults or (link is self.elrs and "link_loss" in self.faults):
            return
        if link is self.usb and not self.in_slot:
            return
        if link.peer is None:
            return
        link.seq = (link.seq + 1) & 0xFF
        try:
            link.sock.sendto(mavlink.encode(name, fields, link.seq, self.sysid, self.compid), link.peer)
        except OSError:
            pass

    def both(self, name, fields):
        self.send(self.usb, name, fields)
        self.send(self.elrs, name, fields)

    def poll(self):
        for link in (self.usb, self.elrs):
            while True:
                try:
                    data, addr = link.sock.recvfrom(4096)
                except (socket.timeout, BlockingIOError):
                    break
                except ConnectionResetError:
                    continue                                 # (Windows: ICMP "port unreachable" of an earlier send)
                except OSError:
                    break
                if link is self.usb and not self.in_slot:
                    continue
                if link is self.elrs and "link_loss" in self.faults:
                    continue
                link.peer = addr
                for m in link.parser.feed(data):
                    if m.fields.get("target_system", self.sysid) not in (self.sysid, 0):
                        continue
                    with self.lock:
                        self.got.append(m.name)
                        del self.got[:-400]
                        self.handle(link, m)

    # --- messages in --------------------------------------------------------------------------------------------
    def ack(self, link, cmd, result):
        self.send(link, "COMMAND_ACK", {"command": cmd, "result": result, "target_system": 255,
                                        "target_component": 190})

    def handle(self, link, m):
        f = m.fields
        if m.name == "PARAM_REQUEST_READ":
            n = f["param_id"]
            if n in self.params:
                self.send_param(link, n)
        elif m.name == "PARAM_REQUEST_LIST":
            for n in sorted(self.params):
                self.send_param(link, n)
        elif m.name in ("COMMAND_LONG", "COMMAND_INT"):
            self.command(link, int(f["command"]), [f.get("param%d" % i, 0.0) for i in range(1, 8)])
        elif m.name == "MISSION_COUNT":
            self.up_count, self.up_items = int(f["count"]), {}
            if self.up_count:
                self.send(link, "MISSION_REQUEST_INT", {"seq": 0, "target_system": 255, "target_component": 190})
            else:
                self.mission = []
                self.send(link, "MISSION_ACK", {"type": 0, "target_system": 255, "target_component": 190})
        elif m.name == "MISSION_ITEM_INT":
            if self.up_count:
                self.up_items[int(f["seq"])] = dict(f)
                nxt = len(self.up_items)
                if nxt < self.up_count:
                    self.send(link, "MISSION_REQUEST_INT", {"seq": nxt, "target_system": 255, "target_component": 190})
                else:
                    self.mission = [self.up_items[i] for i in range(self.up_count)]
                    self.up_count = 0
                    self.send(link, "MISSION_ACK", {"type": 0, "target_system": 255, "target_component": 190})
        elif m.name == "MISSION_REQUEST_LIST":
            self.send(link, "MISSION_COUNT", {"count": len(self.mission), "target_system": 255,
                                              "target_component": 190})
        elif m.name == "MISSION_REQUEST_INT":
            i = int(f["seq"])
            if i < len(self.mission):
                self.send(link, "MISSION_ITEM_INT", dict(self.mission[i], target_system=255, target_component=190))
        elif m.name == "MISSION_CLEAR_ALL":
            self.mission = []
            self.send(link, "MISSION_ACK", {"type": 0, "target_system": 255, "target_component": 190})
        elif m.name == "MISSION_SET_CURRENT":
            self.cur = int(f["seq"])
        elif m.name == "SET_POSITION_TARGET_LOCAL_NED":
            if self.mode == M_GUIDED:
                self.guided_v, self.guided_t = (f["vx"], f["vy"], f["vz"], f["yaw_rate"]), self.clock()
        elif m.name in ("GPS_RTCM_DATA", "GPS_INJECT_DATA"):
            self.inject(f)

    def inject(self, f):
        data = bytes(f["data"][:f["len"]])
        if "flags" in f and f["flags"] & 1:                  # fragmented: collect by sequence
            seq, frag = f["flags"] >> 3, (f["flags"] >> 1) & 3
            parts = self.rtcm.setdefault(seq, {})
            parts[frag] = data
            if len(data) == gpsassist.RTCM_MAX and frag < 3:
                return
            data = b"".join(parts[k] for k in sorted(parts))
            self.rtcm.pop(seq, None)
        now = self.clock()
        for cls, mid, p in gpsassist.parse_ubx(data):
            if cls == 0x13 and mid == 0x40 and p[:1] == b"\x10":
                self.assist["time"] = now
            elif cls == 0x13 and mid == 0x40 and p[:1] == b"\x01":
                self.assist["pos"] = now
            elif cls == 0x13:
                self.assist["eph"] = now

    def send_param(self, link, n):
        self.send(link, "PARAM_VALUE", {"param_id": n, "param_value": float(self.params[n]), "param_type": 9,
                                        "param_count": len(self.params), "param_index": sorted(self.params).index(n)})

    def command(self, link, cmd, p):
        if cmd == 512:                                       # REQUEST_MESSAGE
            name = mavlink.BY_ID.get(int(p[0]))
            if name in ("AUTOPILOT_VERSION", "SYS_STATUS", "BATTERY_STATUS", "GPS_RAW_INT", "EKF_STATUS_REPORT",
                        "HOME_POSITION"):
                self.ack(link, cmd, 0)
                self.send(link, name, self.msg(name))
            else:
                self.ack(link, cmd, 3)
        elif cmd == 511:
            self.ack(link, cmd, 0)
        elif cmd == 400:                                     # ARM / DISARM
            if p[0] == 1:
                ok = self.fix >= 3 or self.mode not in GPS_MODES
                if ok and not self.armed:
                    self.armed, self.events = True, self.events + ["arm"]
                self.ack(link, cmd, 0 if ok else 4)
            else:
                ok = self.landed or p[1] == 21196
                if ok:
                    self.armed = False
                    self.events.append("disarm")
                self.ack(link, cmd, 0 if ok else 4)
        elif cmd == 176:                                     # DO_SET_MODE
            mode = int(p[1])
            if mode in (M_ALT_HOLD, M_AUTO, M_GUIDED, M_LOITER, M_RTL, M_LAND, M_BRAKE, 0):
                if mode in GPS_MODES and self.fix < 3:
                    return self.ack(link, cmd, 4)
                self.set_mode(mode)
                self.ack(link, cmd, 0)
            else:
                self.ack(link, cmd, 3)
        elif cmd == 22:                                      # TAKEOFF
            # ArduCopter: a user take-off in ALT_HOLD only with param3 = 1 ("horizontal navigation by the pilot
            # acceptable", docs/uav_design.md 7: the GPS fallback climb)
            if self.armed and self.landed and (self.mode == M_GUIDED or (self.mode == M_ALT_HOLD and p[2] == 1)):
                self.target_alt = float(p[6])
                self.ack(link, cmd, 0)
            else:
                self.ack(link, cmd, 4)
        elif cmd == 300:                                     # MISSION_START
            if self.mission and self.armed and self.fix >= 3:
                self.set_mode(M_AUTO)
                self.cur = max(1, int(p[0]))                 # seq 0 = HOME: never flown (ArduPilot)
                self.ack(link, cmd, 0)
            else:
                self.ack(link, cmd, 4)
        elif cmd == 178:
            self.speed = float(p[1])
            self.ack(link, cmd, 0)
        elif cmd == 21:                                      # NAV_LAND
            self.set_mode(M_LAND)
            self.ack(link, cmd, 0)
        else:
            self.ack(link, cmd, 3)                           # unsupported

    def set_mode(self, mode):
        if mode != self.mode:
            self.events.append("mode %d" % mode)
        self.mode = mode
        if mode == M_GUIDED:
            self.guided_v, self.guided_t = (0.0, 0.0, 0.0, 0.0), self.clock()

    # --- the world ----------------------------------------------------------------------------------------------
    def latlon(self):
        lat = self.lat0 + math.degrees(self.n / R_EARTH)
        lon = self.lon0 + math.degrees(self.e / (R_EARTH * math.cos(math.radians(self.lat0))))
        return lat, lon

    def volts(self):
        left = max(0.0, 1.0 - self.used_mah / self.capacity)
        return 13.2 + 3.6 * left

    def step(self, dt):
        now = self.clock()
        sky = (self.sky or not self.in_slot) and "no_fix" not in self.faults
        if sky and self.sky_t is None:
            self.sky_t = now
        if not sky:
            self.sky_t, self.fix = None, 1          # the receiver is there, no fix (no sky)
        else:
            fresh = lambda k: k in self.assist and now - self.assist[k] < 4 * 3600   # noqa: E731
            ttf = self.ttf_hot if fresh("eph") and fresh("time") else (
                self.ttf_warm if fresh("time") and fresh("pos") else self.ttf_cold)
            self.fix = 3 if now - self.sky_t >= ttf else 1
        cur = 0.5
        if self.armed:
            cur = 15.0
            climb_ok = "motor_fail" not in self.faults
            vz = 0.0
            if self.mode == M_LAND:
                vz = -1.0
            elif self.target_alt is not None and self.mode in (M_GUIDED, M_ALT_HOLD):
                vz = 2.5 if self.alt < self.target_alt - 0.2 else 0.0
                if vz == 0.0:
                    self.target_alt = None
            vn = ve = 0.0
            if self.mode == M_GUIDED and now - self.guided_t < 3.0:
                gvx, gvy, gvz, _r = self.guided_v
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                vn, ve = gvx * c - gvy * s, gvx * s + gvy * c
                vz = vz or -gvz
            elif self.mode == M_AUTO:
                vn, ve, vz = self.auto_step(vz)
            elif self.mode == M_ALT_HOLD:
                vn, ve = 0.5, 0.3                             # drifts with the wind (no position hold)
            if self.fix < 3 and not self.landed and self.mode in GPS_MODES:
                self.set_mode(M_ALT_HOLD)                    # EKF failsafe = AltHold (FS_EKF_ACTION 2)
            if not climb_ok:
                vz = min(vz, 0.0)
            self.alt = max(0.0, self.alt + vz * dt)
            self.n += vn * dt
            self.e += ve * dt
            self.vn, self.ve, self.vz = vn, ve, vz
            if self.alt > 0.3:
                if self.landed:
                    self.landed = False
                    self.in_slot = False                     # the breakaway cable: the USB link is gone
            elif not self.landed and vz <= 0:
                self.landed = True
                if self.mode == M_LAND:
                    self.armed = False
                    self.events.append("landed")
            if self.volts() <= float(self.params.get("BATT_CRT_VOLT", 0)) and self.mode != M_LAND and not self.landed:
                self.set_mode(M_LAND)                        # BATT_FS_CRT_ACT 1 = the backstop
        self.used_mah += cur * dt / 3.6

    def auto_step(self, vz0):
        if self.cur >= len(self.mission):
            return 0.0, 0.0, 0.0
        it = self.mission[self.cur]
        c = int(it["command"])
        if c == 22:
            alt = float(it["z"])
            if self.alt >= alt - 0.5:
                self.cur += 1
            return 0.0, 0.0, 2.5
        if c == 178:
            self.speed = float(it["param2"]) or self.speed
            self.cur += 1
            return 0.0, 0.0, 0.0
        tlat, tlon = it["x"] / 1e7, it["y"] / 1e7
        dn = math.radians(tlat - self.lat0) * R_EARTH - self.n
        de = math.radians(tlon - self.lon0) * R_EARTH * math.cos(math.radians(self.lat0)) - self.e
        dz = float(it["z"]) - self.alt
        d = math.hypot(dn, de)
        vz = max(-2.0, min(2.5, dz))
        if c == 17:                                          # LOITER_UNLIM: hover over the point
            if d < 1.0:
                return 0.0, 0.0, vz
        elif d < 2.0 and abs(dz) < 1.0:
            self.send(self.elrs, "MISSION_ITEM_REACHED", {"seq": self.cur})
            self.cur += 1
            return 0.0, 0.0, 0.0
        v = min(self.speed, d)
        self.yaw = math.atan2(de, dn)
        return v * dn / max(d, 1e-6), v * de / max(d, 1e-6), vz

    # --- messages out -------------------------------------------------------------------------------------------
    def msg(self, name):
        lat, lon = self.latlon()
        left = max(0.0, 1.0 - self.used_mah / self.capacity)
        if name == "HEARTBEAT":
            typ = 1 if "wrong_type" in self.faults else 2
            return {"type": typ, "autopilot": 3, "base_mode": (0x80 if self.armed else 0) | 1, "custom_mode": self.mode,
                    "system_status": 4 if self.armed else 3, "mavlink_version": 3}
        if name == "SYS_STATUS":
            health = SENS if self.fix else SENS & ~0x20
            if self.fix >= 3:
                health |= PREARM
            return {"onboard_control_sensors_present": SENS, "onboard_control_sensors_enabled": SENS,
                    "onboard_control_sensors_health": health, "voltage_battery": int(self.volts() * 1000),
                    "current_battery": 1500 if self.armed else 50, "battery_remaining": int(left * 100)}
        if name == "BATTERY_STATUS":
            return {"voltages": [int(self.volts() * 250)] * 4 + [0xFFFF] * 6, "current_battery": 1500 if self.armed
                    else 50, "current_consumed": int(self.used_mah), "battery_remaining": int(left * 100),
                    "temperature": 2500}
        if name == "GPS_RAW_INT":
            return {"time_usec": int(self.clock() * 1e6), "fix_type": self.fix, "lat": int(lat * 1e7),
                    "lon": int(lon * 1e7), "alt": int(self.alt * 1000), "eph": 120 if self.fix >= 3 else 9999,
                    "epv": 200, "vel": int(math.hypot(self.vn, self.ve) * 100), "cog": 0,
                    "satellites_visible": 14 if self.fix >= 3 else 0}
        if name == "EKF_STATUS_REPORT":
            return {"flags": 0x1F if self.fix >= 3 else 0x07, "velocity_variance": 0.1, "pos_horiz_variance": 0.1,
                    "pos_vert_variance": 0.1, "compass_variance": 0.1}
        if name == "GLOBAL_POSITION_INT":
            return {"time_boot_ms": int(self.clock() * 1000) & 0xFFFFFFFF, "lat": int(lat * 1e7), "lon": int(lon * 1e7),
                    "alt": int(self.alt * 1000), "relative_alt": int(self.alt * 1000), "vx": int(self.vn * 100),
                    "vy": int(self.ve * 100), "vz": int(-self.vz * 100), "hdg": int(math.degrees(self.yaw) % 360 * 100)}
        if name == "ATTITUDE":
            return {"time_boot_ms": int(self.clock() * 1000) & 0xFFFFFFFF, "yaw": self.yaw}
        if name == "VFR_HUD":
            return {"groundspeed": math.hypot(self.vn, self.ve), "alt": self.alt, "climb": self.vz,
                    "heading": int(math.degrees(self.yaw) % 360), "throttle": 50 if self.armed else 0}
        if name == "AUTOPILOT_VERSION":
            return {"capabilities": 0xFFFF, "flight_sw_version": FW, "uid": self.uid, "board_version": 1063,
                    "vendor_id": 0x1209, "product_id": 0x5741}
        if name == "EXTENDED_SYS_STATE":
            return {"vtol_state": 0, "landed_state": 1 if self.landed else 2}
        if name == "HOME_POSITION":
            return {"latitude": int(self.lat0 * 1e7), "longitude": int(self.lon0 * 1e7), "q": [1.0, 0, 0, 0]}
        return {}

    def tick(self):
        now = self.clock()
        dt = max(0.0, min(0.5, now - self.t))
        self.t = now
        with self.lock:
            self.step(dt)
            for name, period in (("HEARTBEAT", 1.0), ("EXTENDED_SYS_STATE", 1.0), ("SYS_STATUS", 0.5), ("GLOBAL_POSITION_INT", 0.5),
                                 ("ATTITUDE", 0.5), ("VFR_HUD", 0.5), ("GPS_RAW_INT", 1.0), ("BATTERY_STATUS", 1.0),
                                 ("EKF_STATUS_REPORT", 1.0)):
                if now - self.last.get(name, 0.0) >= period:
                    self.last[name] = now
                    self.both(name, self.msg(name))
            if "spam_denied" in self.faults and now - self.last.get("spam", 0.0) >= 0.5:
                self.last["spam"] = now                       # not on the allow-list: the router must drop them
                self.both("SERVO_OUTPUT_RAW", {"servo1_raw": 1500})
                self.both("MOUNT_CONTROL", {"input_a": 1})
            if self.armed and "prearm" not in self.last:
                self.last["prearm"] = now
            elif not self.armed and self.fix < 3 and now - self.last.get("prearm_t", 0.0) > 5.0:
                self.last["prearm_t"] = now
                self.both("STATUSTEXT", {"severity": 3, "text": "PreArm: GPS: Bad fix"})

    def close(self):
        for link in (self.usb, self.elrs):
            try:
                link.sock.close()
            except OSError:
                pass


class Fleet:
    """n simulated drones in one thread (tests: in this process; the demo: simdrone.py as its own process)."""

    def __init__(self, base_port, n=6, origin=(37.82, 23.60), ttf=(30.0, 12.0, 3.0), sysid0=10, faults=()):
        self.drones = [Drone(i, sysid0 + i, base_port + i, base_port + 10 + i, origin, ttf=ttf) for i in range(1, n + 1)]
        for slot, name in faults:
            self.drones[slot - 1].fault(name)
        self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctl.bind(("127.0.0.1", base_port + 30))
        self.ctl.settimeout(0.0)
        self.stop_ev = threading.Event()
        self.th = None

    def control(self, d):
        dr = self.drones[int(d["slot"]) - 1]
        if "fault" in d:
            dr.fault(d["fault"], bool(d.get("on", True)))
        if "sky" in d:
            dr.sky = bool(d["sky"])
        if d.get("out"):
            dr.in_slot = False
        if "sysid" in d:
            dr.sysid = int(d["sysid"])

    def run(self):
        while not self.stop_ev.is_set():
            try:
                data, _a = self.ctl.recvfrom(2048)
                self.control(json.loads(data.decode("utf-8")))
            except (BlockingIOError, socket.timeout, OSError, ValueError, KeyError, IndexError):
                pass
            for d in self.drones:
                d.poll()
                d.tick()
            time.sleep(0.01)
        for d in self.drones:
            d.close()
        try:
            self.ctl.close()
        except OSError:
            pass

    def start(self):
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()
        return self

    def stop(self):
        self.stop_ev.set()
        if self.th:
            self.th.join(3)


def main():
    ap = argparse.ArgumentParser(description="simulated UAV magazine drones (MAVLink 2 over UDP)")
    ap.add_argument("--base-port", type=int, default=14600)
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--origin", default="37.82,23.60")
    ap.add_argument("--fault", action="append", default=[], help="slot:name, e.g. 2:no_fix")
    ap.add_argument("--ttf", default="30,12,3", help="GPS time to fix after the sky: cold,warm,hot (s)")
    ap.add_argument("--stdin-eof-exit", action="store_true",
                    help="end when stdin closes (started by the gateway: never left running after it)")
    a = ap.parse_args()
    lat, lon = (float(x) for x in a.origin.split(","))
    faults = [(int(f.split(":")[0]), f.split(":")[1]) for f in a.fault]
    fl = Fleet(a.base_port, a.n, (lat, lon), ttf=tuple(float(x) for x in a.ttf.split(",")), faults=faults)
    print("simdrone ready: %d drones, USB udp %d.., ELRS udp %d.., control %d" % (
        a.n, a.base_port + 1, a.base_port + 11, a.base_port + 30), flush=True)
    if a.stdin_eof_exit:
        def watch():
            try:
                while sys.stdin.read(1024):
                    pass
            except (OSError, ValueError):
                pass
            fl.stop_ev.set()
        threading.Thread(target=watch, daemon=True).start()
    try:
        fl.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
