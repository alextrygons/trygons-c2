"""Shared helpers for the Trygons gateway (Raspberry Pi 5).

The frame format is the same as the ground station's (groundstation/proto.py,
which must match app/proto.h), so that file is imported from the repository
instead of being copied.
"""
import copy
import json
import logging
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "groundstation"))

import proto  # noqa: E402  (groundstation/proto.py)

log = logging.getLogger("gateway")

# --- configuration ---------------------------------------------------------------

DEFAULTS = {
    "boat": "trygons-c2",
    # the AURIX: the Pi's UART (GPIO14/15) or, for the simulator, a TCP port
    "aurix": "/dev/ttyAMA0",
    "baud": 921600,
    # set the Pi's clock from the AURIX GPS time (needs CAP_SYS_TIME, see README)
    "set_clock": True,
    # GW_STATUS to the AURIX (its "gateway alive" heartbeat; the AURIX alarms after 3 s)
    "status_s": 1.0,
    "storage": {
        "root": "/mnt/ssd/trygons",     # the USB SSD; the SD card holds only the OS
        "require_mount": True,          # root must be a mounted drive (not the SD card)
        "check_s": 30,                  # free space / write test interval
        "low_free_gb": 20.0,            # below this: delete old synced data (section F)
        "full_free_gb": 2.0,            # below this: STORAGE FULL, only the ring buffer runs
    },
}


def deep_merge(base, extra):
    out = copy.deepcopy(base)
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(path=None, extra=None):
    """gateway.json (+ secrets.json next to it, never in git) over the defaults."""
    cfg = copy.deepcopy(DEFAULTS)
    if path:
        with open(path, encoding="utf-8") as f:
            cfg = deep_merge(cfg, json.load(f))
        sec = os.path.join(os.path.dirname(os.path.abspath(path)), "secrets.json")
        if os.path.exists(sec):
            with open(sec, encoding="utf-8") as f:
                cfg["secrets"] = json.load(f)
    return deep_merge(cfg, extra or {})


# --- time ---------------------------------------------------------------------------

class Clock:
    """GPS-corrected time. On the Pi the system clock itself is set from the
    AURIX GPS time (offset then ~0); in tests / without permission the
    offset is kept here instead, so logs and video still agree."""

    def __init__(self):
        self.offset = 0.0
        self.gps_valid = False
        self.last_sync = 0.0

    def now(self):
        return time.time() + self.offset

    def utc(self, t=None):
        return time.gmtime(self.now() if t is None else t)

    def stamp(self, t=None):
        """2026-10-20T10:00:05.123Z"""
        t = self.now() if t is None else t
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + ".%03dZ" % int((t % 1) * 1000)


def setup_logging(level=logging.INFO):
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
