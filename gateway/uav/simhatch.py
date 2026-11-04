"""The simulated magazine hatch (CLAUDE.md V "Sim", docs/uav_design.md 5.4; step U3) - demo + tests.

The hatch motor follows the REAL relay readback of the (simulated) AURIX: TELEM2 group M "relay_on" holds the
hatch relay = it opens, else it closes (so a relay-board reboot / a relay switched OFF elsewhere closes it, like the
real one, docs 5.5). Position 0 (closed) .. 1 (open), travel_s for the full way. read() = the same tuple as
hatch.SerialSwitches: (OPEN switch, CLOSED switch, cable ok, why).

Faults (fault(name, on)): stuck (the motor does not move), slow (3 x the travel time), open_sw_broken / closed_sw_broken
(that switch never reads "there" = a broken wire), both (both switches read closed), unplugged (the switch cable
gone), close_now (the hatch closes by itself, the relay notwithstanding = a relay-board reboot).
"""
FAULTS = ("stuck", "slow", "open_sw_broken", "closed_sw_broken", "both", "unplugged", "close_now")


class SimHatch:
    def __init__(self, travel_s=6.0, pos=0.0):
        self.travel_s = float(travel_s)
        self.pos = float(pos)
        self.faults = set()
        self.t = None
        self.drive = False                          # the relay's state as last seen

    def fault(self, name, on=True):
        if name not in FAULTS:
            raise ValueError(name)
        (self.faults.add if on else self.faults.discard)(name)

    def step(self, now, relay_on):
        dt = 0.0 if self.t is None else max(0.0, min(1.0, now - self.t))
        self.t = now
        self.drive = bool(relay_on) and "close_now" not in self.faults
        if "stuck" in self.faults:
            return
        v = dt / (self.travel_s * (3.0 if "slow" in self.faults else 1.0))
        self.pos = min(1.0, self.pos + v) if self.drive else max(0.0, self.pos - v)

    def read(self, now=None):
        if "unplugged" in self.faults:
            return False, False, False, "hatch switch cable unplugged (simulated)"
        op = self.pos >= 0.98 and "open_sw_broken" not in self.faults
        cl = self.pos <= 0.02 and "closed_sw_broken" not in self.faults
        if "both" in self.faults:
            op = cl = True
        return op, cl, True, ""

    def sky(self):
        """The drones in the magazine see the sky (their GPS can get a fix) once the hatch is half open."""
        return self.pos >= 0.5
