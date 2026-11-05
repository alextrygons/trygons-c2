"""Manual UAV control from a shore joystick + the pilot lock (CLAUDE.md V "Manual control", docs/uav_design.md 6.3,
step U5; owner decision W61 2026-11-04).

  station --uav_pilot {uav, take | give}-->  Pilots.pilot_msg      the lock: one pilot per drone
  station --uav_ctl {uav, seq, ts, x, y, z, r} (10 Hz, UDP signed like rjoy)-->  Pilots.ctl
                                            -> the router op "ctl" = a GUIDED body-frame VELOCITY setpoint
  station --uav_hold {uav}-->               the drone hovers (BRAKE -> LOITER), no pilot needed
  station <--uav_ack {what: "pilot", ok, code, uav, text}--   <--uav_pilot_end {uav, why, text}-- (the lock ended)

ASSISTED FLYING ONLY: the stick never reaches the drone as RC (the drones ignore their RC receiver, W68); the gateway
turns the normalised stick (-1..1: x = right, y = forward, z = climb, r = yaw right) into a velocity setpoint
(<= uav_man_speed_ms horizontal, <= uav_man_climb_ms vertical, <= uav_man_yaw_dps) and CLIPS it:
  - altitude: never below uav_min_alt_m above the launch level, never above uav_max_alt_m (the descent / climb slows
    down to the limit; an unknown altitude = no vertical speed);
  - the air geofence + the no-fly zones (nofly.Airspace): the point the drone would reach in edge_lookahead_s (+
    edge_margin_m) must be allowed, else NO horizontal speed = it holds at the edge (an unknown position / heading =
    no horizontal speed; already outside: only a move that brings it back in is let through).
Let go (centred) = zero = the drone holds its position.

The lock (one pilot per drone):
  - the FIRST pilot wins; another station gets 127 - EXCEPT (owner W61, 2026-11-04) the CAPTAIN, who takes a drone
    over from a copilot AT ONCE: the copilot's station gets uav_pilot_end why "captain" (screen «Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟΝ
    ΕΛΕΓΧΟ»), the drone holds (zero setpoint) during the hand-over and until the captain's stick has been seen
    CENTRED (every new pilot: no jump when the stick is held deflected at the take). A copilot never takes a drone
    from the captain (127).
  - the CAPTAIN (the crew captain, a single operator, or the HELM holder: his joystick drives the boat) flies ONLY
    with the boat in STANDBY (fresh telemetry): else 126; the boat leaving STANDBY ends it AT ONCE (the drone holds,
    the joystick goes back to the boat). While the captain's joystick is on a UAV the gateway refuses the captain's
    driving commands + every copilot command that would need his ACCEPT (126, stations.py), and drops his rjoy.
  - stale pilot data: no fresh uav_ctl for pilot_stale_s 0.5 s = zero setpoint (hold); for pilot_lost_s 3 s, the
    station gone / logged out = the lock ends (hold). The router also sends a zero setpoint by itself 0.5 s after the
    last moving one (a gateway hang), and ArduPilot's GUIDED timeout stops a velocity setpoint after ~3 s.
  - giving back («ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» = the JOYSTICK goes back to the boat, never "fly back"): zero setpoint, BRAKE,
    then LOITER (the drone hovers where it is; one-way drones never land on the boat).
Nothing here talks to the AURIX (UA5): the boat's mode comes from uavlink.boat_mode.
"""
import math
import time

import uavlink
from . import allow

ERR_UAV, ERR_JOY, ERR_PILOT = 122, 126, 127
FRAME = 9                                          # MAV_FRAME_BODY_OFFSET_NED: velocity relative to the heading
MASK = 0x0007 | 0x01C0 | 0x0400                    # position, acceleration and yaw ignored: velocity + yaw rate
DEFAULTS = {
    "uav_man_speed_ms": 10.0,       # TO CONFIRM (docs 6.3)
    "uav_man_climb_ms": 2.0,
    "uav_man_yaw_dps": 60.0,
    "deadband": 0.08,               # a stick within this = centred (zero)
    "pilot_stale_s": 0.5,           # no fresh uav_ctl this long = hold (zero setpoint)
    "pilot_lost_s": 3.0,            # ... this long = the lock ends
    "edge_lookahead_s": 3.0,        # the geofence / no-fly clip looks this far ahead
    "edge_margin_m": 10.0,
    "loiter_after_s": 1.0,          # give back: BRAKE, LOITER this long after
    "data_stale_s": 1.8,            # U5 review MED-2: the drone's data older than this = no motion (it holds); U6
                                    # review L3: one lost GLOBAL_POSITION_INT (normal age up to ~1 s + a packet) is
                                    # tolerated, two are not
}
R_EARTH = 6371000.0


def _finite(*v):
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v)


def shape(v, db):
    """A stick value -1..1 with the dead band: within db = 0, beyond it rescaled to 0..1 (no jump at the edge)."""
    v = max(-1.0, min(1.0, float(v)))
    if abs(v) <= db:
        return 0.0
    return math.copysign((abs(v) - db) / (1.0 - db), v)


def ahead(lat, lon, dist_m, brg_deg):
    b = math.radians(brg_deg)
    dn, de = dist_m * math.cos(b), dist_m * math.sin(b)
    return (lat + math.degrees(dn / R_EARTH),
            lon + math.degrees(de / (R_EARTH * math.cos(math.radians(lat)))))


def setpoint(stick, drone, cfg, airspace=None):
    """stick {x, y, z, r} (-1..1) + the drone (map record: lat, lon, alt, hdg, gs, climb, alt_age) -> (fields, clipped
    reasons). fields = {vx, vy, vz (NED: + down), yaw_rate} in the body frame. Pure (tests UA30 / UA34).
    U5 review MED-2: the drone's data age (alt_age: the router's age + the status' age) older than data_stale_s = NO
    motion (hold); within it the position is moved on with the drone's ground speed and the altitude limits use the
    altitude it may have reached meanwhile (the climb rate x the age). MED-3: a climb / descent is checked against the
    no-fly zones (with their altitude bands) like a horizontal move."""
    db = float(cfg["deadband"])
    age = drone.get("alt_age")
    if not _finite(age) or age > float(cfg.get("data_stale_s", 1.8)):
        return dict(ZERO), ["drone data %s old - it holds" % ("unknown" if not _finite(age) else "%.1f s" % age)]
    age = max(0.0, age)
    x, y, z, r = (shape(stick.get(k, 0.0) or 0.0, db) for k in ("x", "y", "z", "r"))
    spd = min(float(cfg["uav_man_speed_ms"]), allow.MAX_VXY_MS)
    clb = min(float(cfg["uav_man_climb_ms"]), allow.MAX_VZ_MS)
    yaw = min(math.radians(float(cfg["uav_man_yaw_dps"])), allow.MAX_YAW_RATE)
    vx, vy, up, yr = y * spd, x * spd, z * clb, r * yaw
    h = math.hypot(vx, vy)
    if h > spd:
        vx, vy = vx * spd / h, vy * spd / h
    why = []
    alt = drone.get("alt")
    lo, hi = float(cfg["uav_min_alt_m"]), float(cfg["uav_max_alt_m"])
    clb_now = drone.get("climb") if _finite(drone.get("climb")) else 0.0
    if up:
        if not _finite(alt):
            up = 0.0
            why.append("altitude unknown")
        elif up < 0:
            alt_lo = alt - abs(clb_now) * age       # MED-2: where it may be by now (old data, either way)
            room = max(0.0, alt_lo - lo)            # slows down to the floor (1 s ahead)
            if -up > room:
                up = -room
                why.append("minimum altitude %.0f m" % lo)
        else:
            room = max(0.0, hi - (alt + abs(clb_now) * age))
            if up > room:
                up = room
                why.append("maximum altitude %.0f m" % hi)
    lat, lon, hdg = drone.get("lat"), drone.get("lon"), drone.get("hdg")
    if _finite(lat, lon) and _finite(drone.get("gs")) and drone["gs"] > 0 and age > 0:
        # MED-2: moved on with its ground speed - U6 review L2: along its real COURSE over ground (GLOBAL_POSITION_INT
        # vx / vy), the heading only when no course is known
        cog = drone.get("cog") if _finite(drone.get("cog")) else hdg
        if _finite(cog):
            lat, lon = ahead(lat, lon, drone["gs"] * age, cog)
    look = float(cfg["edge_lookahead_s"])
    a = alt if _finite(alt) else 0.0
    if up and airspace is not None and _finite(lat, lon, alt):
        # MED-3: a climb / descent into a zone's altitude band (the margin in the direction of the move)
        a2 = a + up * look + math.copysign(float(cfg["edge_margin_m"]), up)
        here_bad = airspace.problem([(lat, lon, a)])
        # U6 review L1: a vertical leg has no horizontal length - sampled every VERT_STEP_M (0.5 m; WA/U6 review 2
        # LOW: samples at most 0.5 m apart, so an altitude band at least 0.5 m thick between the two ends is always
        # seen - a thinner band could fall between two samples)
        k = max(1, int(math.ceil(abs(a2 - a) / VERT_STEP_M)))
        ups = [(lat, lon, a + (a2 - a) * i / k) for i in range(1, k + 1)]
        bad = next((b for b in (airspace.problem([p]) for p in ups) if b), None)
        if not bad and not here_bad:
            bad = airspace.problem([(lat, lon, a), (lat, lon, a2)])
        if bad:
            up = 0.0
            why.append("air geofence / no-fly zone above / below: %s" % bad)
    if vx or vy:
        if not _finite(lat, lon, hdg):
            vx = vy = 0.0
            why.append("position / heading unknown")
        elif airspace is not None:
            v = math.hypot(vx, vy)
            brg = (hdg + math.degrees(math.atan2(vy, vx))) % 360.0
            a2 = a + up * look
            p = ahead(lat, lon, v * look + float(cfg["edge_margin_m"]), brg)
            here_bad = airspace.problem([(lat, lon, a)])
            bad = airspace.problem([(p[0], p[1], a2)]) if here_bad else \
                airspace.problem([(lat, lon, a), (p[0], p[1], a2)])
            if bad:
                vx = vy = 0.0
                why.append("air geofence / no-fly zone: %s" % bad)
    return {"vx": round(vx, 3), "vy": round(vy, 3), "vz": round(-up, 3), "yaw_rate": round(yr, 4)}, why


ZERO = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0}
VERT_STEP_M = 0.5                       # U6 review L1 (+ review 2): the vertical leg's sample step


class Lock:
    def __init__(self, sid, name, now):
        self.sid, self.name, self.since = sid, name, now
        self.last_t = now                       # the last fresh uav_ctl (the take counts as one)
        self.seq = None
        self.centred = False                    # the pilot's stick seen centred since the take (no jump)
        self.stale = False
        self.sp = dict(ZERO)
        self.why = []
        self.guided = None                      # None = asked, True = the drone took GUIDED, False = refused
        self.n_ctl = self.n_old = 0


class Pilots:
    def __init__(self, mgr, cfg=None):
        self.mgr = mgr
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.locks = {}                         # slot -> Lock
        self.loiter = {}                        # slot -> time: LOITER after the BRAKE of a give-back
        self.events = []                        # (t, slot, what, station, why) last 20 (status / tests)
        self.stats = {"ctl": 0, "stale": 0, "old": 0, "bad": 0, "refused": 0, "clipped": 0, "takeover": 0}

    # --- helpers --------------------------------------------------------------------------------------------------
    def ccfg(self):
        return dict(self.cfg, **{k: self.mgr.ccfg[k] for k in ("uav_min_alt_m", "uav_max_alt_m")})

    def stations(self):
        return self.mgr.gw.stations.stations

    def captain_like(self, st):
        """This station's joystick drives the BOAT: the crew captain, a single operator, the HELM holder."""
        if st is None:
            return False
        helm = (getattr(self.mgr.gw, "telem2", None) or {}).get("helm")
        return st.crew == "captain" or bool(st.single_op) or (helm is not None and helm == st.id)

    def slot_of(self, sid):
        return next((s for s, lk in self.locks.items() if lk.sid == sid), None)

    def joystick_on_uav(self, st):
        """The slot the CAPTAIN's joystick flies now (stations.py: 126), else None. A copilot's never counts (his
        joystick does not drive the boat)."""
        s = self.slot_of(st.id) if st is not None else None
        return s if s is not None and self.captain_like(st) else None

    def flying(self, n):
        """None = slot n's drone is in the air and may be flown by hand, else why not."""
        d = self.mgr.slot(n)
        if not d or d.get("hb_age") is None:
            return "UAV %d: no data from it" % n
        if not d.get("armed") or d.get("landed") or d.get("usb_present"):
            return "UAV %d is not flying" % n
        if d.get("landing"):
            return "UAV %d lands for its battery" % n
        return None

    def note(self, slot, what, sid, why=""):
        self.events.append((round(time.time(), 1), slot, what, sid, why))
        del self.events[:-20]
        self.mgr.gw.audit("uav_pilot", slot=slot, what=what, station=sid, why=why)

    def ack(self, st, ok, code, text, n=None, **kw):
        st.send(dict({"t": "uav_ack", "what": "pilot", "ok": ok, "code": 0 if ok else code, "text": text, "uav": n},
                     **kw))
        if not ok:
            self.stats["refused"] += 1

    def send_sp(self, n, sp):
        self.mgr.to_worker(dict({"op": "ctl", "slot": n}, **sp))

    def hover(self, n):
        """Zero setpoint, BRAKE now, LOITER after loiter_after_s: the drone hovers where it is. U5 review LOW: BRAKE /
        LOITER need a GPS position - without one (no 3D fix / no position) the drone gets ALT_HOLD (holds its
        altitude, drifts with the wind = the documented "no GPS = hold" of docs 6.1) and no LOITER later."""
        self.send_sp(n, ZERO)
        d = self.mgr.slot(n) or {}
        if (d.get("gps_fix") or 0) >= 3 and d.get("lat") is not None:
            self.mgr.cmd(n, "elrs", allow.CMD_DO_SET_MODE, [1, allow.COPTER_BRAKE], lambda _r, _w: None)
            self.loiter[n] = time.time() + float(self.cfg["loiter_after_s"])
        else:
            self.mgr.cmd(n, "elrs", allow.CMD_DO_SET_MODE, [1, allow.COPTER_ALT_HOLD], lambda _r, _w: None)
            self.loiter.pop(n, None)

    def end(self, n, why, text, tell=True):
        lk = self.locks.pop(n, None)
        if lk is None:
            return
        self.hover(n)
        self.note(n, "end", lk.sid, why)
        st = self.stations().get(lk.sid)
        if tell and st is not None:
            st.send({"t": "uav_pilot_end", "uav": n, "why": why, "text": text})

    # --- station messages -----------------------------------------------------------------------------------------
    def pilot_msg(self, st, m):
        """{"t": "uav_pilot", "uav": n, "take": true | false}. Any crew station but a VIEWER may fly (U5 review LOW:
        by design - the copilot flies from his own station; the captain only in STANDBY)."""
        try:
            n = int(m.get("uav"))
        except (TypeError, ValueError):
            n = 0
        take = bool(m.get("take", True))
        if not take:
            lk = self.locks.get(n)
            if lk is None:
                # U5 review LOW: the gateway ended the lock already (STANDBY left, lost, landed): nothing to refuse
                return self.ack(st, True, 0, "UAV %d: already given back - the joystick is on the boat" % n, n,
                                take=False)
            if lk.sid != st.id:
                return self.ack(st, False, ERR_PILOT, "UAV %d: you do not fly it" % n, n)
            self.end(n, "given", "UAV %d given back: it hovers, the joystick is back on the boat" % n, tell=False)
            return self.ack(st, True, 0, "UAV %d given back: it hovers, the joystick is back on the boat" % n, n,
                            take=False)
        mgr = self.mgr
        why = None
        if st.role == "VIEWER":
            why = (ERR_UAV, "VIEWER: watch only")
        elif not mgr.enabled:
            why = (ERR_UAV, "no UAV magazine on this boat")
        elif not 1 <= n <= mgr.n:
            why = (ERR_UAV, "no such slot")
        elif not mgr.router_ok():
            why = (ERR_UAV, "UAV router offline (%s)" % (mgr.why or "-"))
        else:
            w = self.flying(n)
            if w:
                why = (ERR_UAV, w)
            else:
                # U5 review MED-1: never while the launch / its climb fallback still owns the drone (a take's GUIDED +
                # zero setpoint could stop the climb or the mission start)
                blk = getattr(mgr, "pilot_block", None)
                w = blk(n) if blk is not None else None
                if w:
                    why = (ERR_UAV, w)
        cap = self.captain_like(st)
        if why is None and cap and not uavlink.boat_mode(mgr.gw)["standby"]:
            why = (ERR_JOY, "the captain flies a UAV only with the boat in STANDBY")
        old = self.locks.get(n)
        if why is None and old is not None and old.sid != st.id:
            holder = self.stations().get(old.sid)
            if not (cap and not self.captain_like(holder)):
                why = (ERR_PILOT, "UAV %d is flown by %s" % (n, old.name))
        if why is not None:
            self.note(n, "refused", st.id, why[1])
            return self.ack(st, False, why[0], "UAV pilot refused: %s" % why[1], n)
        if old is not None and old.sid != st.id:
            # owner W61 (2026-11-04): the captain takes the drone over from the copilot at once; it holds meanwhile
            self.stats["takeover"] += 1
            self.locks.pop(n, None)
            self.send_sp(n, ZERO)
            self.note(n, "takeover", st.id, "from station %s" % old.sid)
            ost = self.stations().get(old.sid)
            if ost is not None:
                ost.send({"t": "uav_pilot_end", "uav": n, "why": "captain",
                          "text": "the captain took control of UAV %d" % n})
        mine = self.slot_of(st.id)
        if mine is not None and mine != n:
            self.end(mine, "switched", "UAV %d given back: it hovers" % mine, tell=False)
        lk = self.locks[n] = Lock(st.id, st.name, time.time())
        self.send_sp(n, ZERO)

        def guided(result, w, lk=lk):
            if self.locks.get(n) is not lk:
                return
            lk.guided = result == 0
            if result != 0:
                self.locks.pop(n, None)
                self.note(n, "refused", lk.sid, "GUIDED: %s" % (w or result))
                s2 = self.stations().get(lk.sid)
                if s2 is not None:
                    self.ack(s2, False, ERR_UAV, "UAV pilot refused: UAV %d did not take GUIDED (%s)" % (
                        n, w or result), n)
        mgr.cmd(n, "elrs", allow.CMD_DO_SET_MODE, [1, allow.COPTER_GUIDED], guided)
        self.note(n, "take", st.id, "captain" if cap else "copilot")
        if cap:
            # U5 review MED-4: questions already open for the captain's ACCEPT are refused now (126)
            ref = getattr(getattr(mgr.gw, "stations", None), "refuse_asks_for", None)
            if ref is not None:
                ref(st, ERR_JOY)
        self.ack(st, True, 0, "UAV %d: the joystick flies it now - centre the stick first" % n, n, take=True,
                 captain=cap)

    def ctl(self, st, m):
        """{"t": "uav_ctl", "uav", "seq", "ts", "x", "y", "z", "r"} 10 Hz from the pilot (UDP, signed)."""
        try:
            n = int(m.get("uav"))
            seq = int(m["seq"]) & 0xFFFF
            ts = float(m["ts"])
            stick = {k: float(m.get(k, 0.0) or 0.0) for k in ("x", "y", "z", "r")}
        except (KeyError, TypeError, ValueError):
            self.stats["bad"] += 1
            return
        lk = self.locks.get(n)
        if lk is None or lk.sid != st.id:
            return                                      # not this station's drone (a takeover: the old pilot's)
        if not _finite(ts, *stick.values()):
            self.stats["bad"] += 1                      # a NaN / infinite stick is never a full deflection
            return
        if lk.seq is not None and (((seq - lk.seq) & 0xFFFF) >= 0x8000 or seq == lk.seq):
            self.stats["old"] += 1
            return
        age = self.mgr.gw.stations.age(st, ts) if hasattr(self.mgr.gw.stations, "age") else 0.0
        if age > float(self.cfg["pilot_stale_s"]):
            lk.n_old += 1
            self.stats["stale"] += 1
            return                                      # too old to fly with (tick() holds the drone)
        lk.seq = seq
        if self.captain_like(st) and not uavlink.boat_mode(self.mgr.gw)["standby"]:
            return self.end(n, "standby", "the boat left STANDBY: UAV %d hovers, the joystick is back on the boat" % n)
        lk.last_t, lk.stale = time.time(), False
        lk.n_ctl += 1
        self.stats["ctl"] += 1
        db = float(self.cfg["deadband"])
        if not lk.centred:
            if all(abs(v) <= db for v in stick.values()):
                lk.centred = True
            else:
                lk.sp, lk.why = dict(ZERO), ["centre the stick first"]
                return self.send_sp(n, ZERO)
        drone = self.mgr.drone_view(n) or {}
        sp, why = setpoint(stick, drone, self.ccfg(), self.mgr.airspace)
        if why:
            self.stats["clipped"] += 1
        lk.sp, lk.why = sp, why
        self.send_sp(n, sp)

    def hold_msg(self, st, m):
        """{"t": "uav_hold", "uav": n}: the drone hovers now (any crew station; a drone someone flies: its pilot)."""
        try:
            n = int(m.get("uav"))
        except (TypeError, ValueError):
            n = 0
        if st.role == "VIEWER":
            return self.ack(st, False, ERR_UAV, "VIEWER: watch only", n, what="hold")
        lk = self.locks.get(n)
        if lk is not None and lk.sid != st.id:
            return st.send({"t": "uav_ack", "what": "hold", "ok": False, "code": ERR_PILOT, "uav": n,
                            "text": "UAV pilot refused: UAV %d is flown by %s" % (n, lk.name)})
        w = self.flying(n) if 1 <= n <= self.mgr.n else "no such slot"
        if w:
            return st.send({"t": "uav_ack", "what": "hold", "ok": False, "code": ERR_UAV, "uav": n,
                            "text": "UAV pilot refused: %s" % w})
        if lk is not None:
            self.end(n, "hold", "UAV %d hovers" % n, tell=False)
        else:
            self.hover(n)
        self.note(n, "hold", st.id)
        st.send({"t": "uav_ack", "what": "hold", "ok": True, "code": 0, "uav": n, "text": "UAV %d hovers" % n})

    def station_gone(self, st):
        n = self.slot_of(st.id)
        if n is not None:
            self.end(n, "gone", "", tell=False)

    # --- 4 Hz ------------------------------------------------------------------------------------------------------
    def tick(self, now=None):
        now = time.time() if now is None else now
        for n, t in list(self.loiter.items()):
            if now >= t:
                del self.loiter[n]
                if n not in self.locks:
                    self.mgr.cmd(n, "elrs", allow.CMD_DO_SET_MODE, [1, allow.COPTER_LOITER], lambda _r, _w: None)
        if not self.locks:
            return
        standby = uavlink.boat_mode(self.mgr.gw)["standby"]
        for n, lk in list(self.locks.items()):
            st = self.stations().get(lk.sid)
            if st is None or getattr(st, "closed", False):
                self.end(n, "gone", "", tell=False)
            elif self.captain_like(st) and not standby:
                self.end(n, "standby", "the boat left STANDBY: UAV %d hovers, the joystick is back on the boat" % n)
            elif self.flying(n):
                self.end(n, "landed", "UAV %d: %s - manual control ended" % (n, self.flying(n)))
            elif now - lk.last_t > float(self.cfg["pilot_lost_s"]):
                self.end(n, "lost", "UAV %d: no joystick data from this station - it hovers" % n)
            elif now - lk.last_t > float(self.cfg["pilot_stale_s"]):
                if not lk.stale:
                    lk.stale = True
                    self.stats["stale"] += 1
                    self.note(n, "stale", lk.sid)
                lk.sp = dict(ZERO)
                self.send_sp(n, ZERO)

    def status(self):
        return {str(n): {"station": lk.sid, "name": lk.name, "since": round(lk.since, 1), "centred": lk.centred,
                         "stale": lk.stale, "sp": lk.sp, "clip": lk.why[:2], "guided": lk.guided,
                         "captain": self.captain_like(self.stations().get(lk.sid))}
                for n, lk in self.locks.items()}
