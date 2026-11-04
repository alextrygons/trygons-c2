"""The magazine hatch: its two limit switches + the state (CLAUDE.md V, docs/uav_design.md 5.4 / 5.5 / 5.6; U3).

The relay (ON = open) is a relay of the EXISTING boat relay table (an "extra" onoff relay, gateway.json "uav"
"hatch_relay"); it is switched ONLY through gateway/uavlink.py -> stations.py on the HELM station's behalf (the
AURIX checks HELM / the table / FIRST TEST as always). This module never sends anything: it READS the switches and
judges the hatch.

Switches = read by the GATEWAY through the modem-status inputs of a 3.3 V FT232R breakout (W57 / W57a): CTS = the
OPEN switch, DSR = the CLOSED switch, each switch closes to GND at its end (external pull-ups): pyserial getCTS() /
getDSR() are True while the line is LOW = "at that end"; a broken wire = high = "not there" (fail safe: "fully
open" can only be read with an intact wire). SerialSwitches polls them at 20 Hz in its OWN thread (never on the event
loop), 50 ms debounce; the port gone = not ok. The simulated hatch (simhatch.py) gives the same read().

Hatch.update(now, (open_sw, closed_sw, ok, why)) -> the state:
  CLOSED   CLOSED switch only          OPEN     OPEN switch only
  OPENING  commanded open, on the way  CLOSING  commanded closed, on the way
  UNKNOWN  no switch yet at start (FAULT once longer than the travel time)
  FAULT    latched until the next command: the switch cable / port gone, BOTH switches, NONE for longer than the
           travel time, the commanded end not reached within its travel time, a switch changing WITHOUT a command
           (e.g. a relay-board reboot switched the relay OFF = the hatch closes by itself, docs 5.5)
Never ARM unless OPEN (the router's ctx: allow.py refuses ARM without it).
"""
import threading
import time

OPEN, CLOSED, OPENING, CLOSING, UNKNOWN, FAULT = "OPEN", "CLOSED", "OPENING", "CLOSING", "UNKNOWN", "FAULT"
DEBOUNCE_S = 0.05


class SerialSwitches:
    """CTS = OPEN switch, DSR = CLOSED switch of a USB-UART (osdep.resolve_serial name), its own thread."""

    def __init__(self, spec, poll_s=0.05):
        self.spec, self.poll_s = spec, poll_s
        self.lock = threading.Lock()
        self.val = (False, False, False, "not opened")
        self.raw, self.raw_t = None, 0.0
        self.stop_ev = threading.Event()
        self.th = None

    def start(self):
        if self.spec and self.th is None:
            self.th = threading.Thread(target=self._run, daemon=True)
            self.th.start()
        return self

    def _run(self):
        ser = None
        while not self.stop_ev.is_set():
            if ser is None:
                try:
                    import serial
                    import osdep
                    ser = serial.Serial(osdep.resolve_serial(self.spec), 9600, timeout=0.1, write_timeout=0.1)
                except Exception as e:                      # not plugged in: "not ok" = FAULT, retried
                    with self.lock:
                        self.val = (False, False, False, "hatch switch cable: %s" % str(e)[:60])
                    self.stop_ev.wait(1.0)
                    continue
            try:
                raw = (bool(ser.getCTS()), bool(ser.getDSR()))
            except Exception as e:
                with self.lock:
                    self.val = (False, False, False, "hatch switch cable: %s" % str(e)[:60])
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                continue
            now = time.monotonic()
            if raw != self.raw:
                self.raw, self.raw_t = raw, now
            with self.lock:
                if now - self.raw_t >= DEBOUNCE_S:
                    self.val = (raw[0], raw[1], True, "")
                elif not self.val[2]:
                    self.val = (False, False, True, "")
            self.stop_ev.wait(self.poll_s)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def read(self, now=None):
        with self.lock:
            return self.val

    def stop(self):
        self.stop_ev.set()


class Hatch:
    def __init__(self, open_s=20.0, close_s=20.0):
        self.open_s, self.close_s = float(open_s), float(close_s)
        self.state, self.why = UNKNOWN, ""
        self.cmd, self.cmd_t = None, 0.0          # "open" / "close" / None, when
        self.reached = False                     # the commanded end was reached (then any change = a fault)
        self.t0 = None
        self.sw = (False, False, False, "")
        self.faults = 0

    def command(self, want_open, now):
        """The gateway sent the relay command (open = ON): a new travel starts, a latched FAULT is cleared."""
        self.cmd, self.cmd_t, self.reached = ("open" if want_open else "close"), now, False
        if self.state == FAULT:
            self.state, self.why = UNKNOWN, ""

    def fault(self, why):
        if self.state != FAULT:
            self.faults += 1
        self.state, self.why = FAULT, why

    def update(self, now, sw):
        if self.t0 is None:
            self.t0 = now
        self.sw = sw
        op, cl, ok, why = sw
        if self.state == FAULT:
            return self.state
        if not ok:
            self.fault(why or "hatch switch cable unplugged")
        elif op and cl:
            self.fault("both hatch switches closed")
        elif self.cmd == "open":
            if self.reached and not op:
                self.fault("the hatch moved without a command")
            elif op:
                self.state, self.why, self.reached = OPEN, "", True
            elif now - self.cmd_t > self.open_s:
                self.fault("the hatch did not open within %.0f s" % self.open_s)
            elif cl and now - self.cmd_t > max(2.0, self.open_s / 2.0):
                self.fault("the hatch did not leave CLOSED")
            else:
                self.state = OPENING
        elif self.cmd == "close":
            if self.reached and not cl:
                self.fault("the hatch moved without a command")
            elif cl:
                self.state, self.why, self.reached = CLOSED, "", True
            elif now - self.cmd_t > self.close_s:
                self.fault("the hatch did not close within %.0f s" % self.close_s)
            else:
                self.state = CLOSING
        else:                                     # no command since the gateway started: the switches tell
            if self.state in (OPEN, CLOSED) and ((self.state == OPEN) != op or (self.state == CLOSED) != cl):
                self.fault("the hatch moved without a command")
            elif op:
                self.state = OPEN
            elif cl:
                self.state = CLOSED
            elif now - self.t0 > max(self.open_s, self.close_s):
                self.fault("no hatch switch closed for longer than the travel time")
            else:
                self.state = UNKNOWN
        return self.state

    def status(self):
        op, cl, ok, _w = self.sw
        return {"state": self.state, "why": self.why, "open_sw": op, "closed_sw": cl, "switch_ok": ok,
                "cmd": self.cmd, "faults": self.faults}
