"""Time for the whole boat (section A7): the AURIX sends GPS time once a
second. When it is valid the Pi sets its own clock from it (if allowed and
more than max_err_s off), so the gateway logs, the AURIX log copy and the
camera recordings all use the same, GPS-correct time.

Without permission to set the clock (tests, PC) the difference is kept as
an offset in Clock instead - everything time-stamped by the gateway uses
Clock.now(), so the result is the same.
"""
import subprocess
import sys
import time

from common import log, proto


class TimeSync:
    def __init__(self, clock, set_clock=True, max_err_s=1.0):
        self.clock = clock
        self.set_clock = set_clock and sys.platform.startswith("linux")
        self.max_err_s = max_err_s
        self.gps_time = None
        self.sets = 0

    def on_time(self, frame):
        d = proto.decode_time(frame.payload)
        if not d["valid"] or d["date"] == 0:
            self.clock.gps_valid = False
            return
        gps = proto.gps_epoch(d["date"], d["tod_ms"])
        self.gps_time = gps
        self.clock.gps_valid = True
        err = gps - self.clock.now()
        if abs(err) <= self.max_err_s:
            return
        if self.set_clock:
            try:
                subprocess.run(["date", "-u", "-s", "@%.3f" % gps], check=True, capture_output=True)
                self.clock.offset = 0.0
                self.sets += 1
                log.info("clock set from GPS (was %.1f s off)", err)
                return
            except (OSError, subprocess.CalledProcessError) as e:
                log.warning("cannot set the clock (%s): keeping an offset instead", e)
                self.set_clock = False
        first = self.sets == 0
        self.clock.offset = gps - time.time()
        self.clock.last_sync = time.time()
        self.sets += 1
        (log.info if first else log.debug)("GPS time offset %.3f s", self.clock.offset)
