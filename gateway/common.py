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
    "stations": {
        "listen": "0.0.0.0",            # the firewall lets in only the VPN and the boat LAN
        "port": 5761,
        "local_nets": ["10.42.0.0/24"], # the boat LAN: stations here are LOCAL (can always take HELM)
        "max_age_s": 2.0,               # older commands are refused (stale)
        "rate_per_s": 10.0,
        "rate_burst": 20.0,
        "lite_rtt_ms": 400.0,           # slower link -> reduced telemetry (2 Hz, essentials)
        "lite_hz": 2.0,
        "lost_s": 5.0,                  # no answer this long -> station lost
    },
    "channels": {
        "driver": "rutos",              # "rutos" (Teltonika RUT901), "policy" (the Pi routes), "sim"
        "router": {"host": "10.42.0.1", "ssh_user": "root", "ssh_key": "/home/trygons/.ssh/id_ed25519",
                   "ping_host": "1.1.1.1"},
        "check_s": 10.0,                # health / status of every channel
        "stable_s": 60.0,               # a channel back this long -> used again (AUTO)
        "fail_s": 30.0,                 # MANUAL: no internet this long -> fall back
        "trial_s": 60.0,                # a switch must be confirmed (Keep) within this
        "list": None,                   # None = channels.DEFAULT_CHANNELS (4G, 4G SIM 2, RF, STARLINK)
    },
    "cameras": {
        "enabled": True,
        "list": None,                   # None = cameras.DEFAULT_CAMERAS (FRONT-1, FRONT-2, REAR-LEFT, REAR-RIGHT)
        "mediamtx_bin": None,           # None = "mediamtx" on the PATH or /usr/local/bin/mediamtx
        "mediamtx_conf": "/var/lib/trygons/mediamtx.yml",
        "api_port": 9997, "rtsp_port": 8554, "webrtc_port": 8889, "webrtc_udp_port": 8189,
        "frozen_s": 1.0,                # no new frames this long = FROZEN
        "internet_cmd": None,           # e.g. ["sudo", ".../gateway/tools/camera_internet.sh"]
    },
    "recording": {
        "enabled": True,                # ring buffer of all cameras (main streams) on the SSD
        "segment_s": 60,                # 1-minute files
        "ring_min": 10,                 # ring length (the captain picks one of ring_choices_min)
        "ring_choices_min": [5, 10, 20, 30, 60],
        "post_s": 120,                  # SAVE keeps this long after the (last) trigger
        "pre_s": None,                  # SAVE keeps this long before it; None = the whole ring
        "ring_sync": False,             # section G: ring video to the shore PC too (off: data!)
        "ffmpeg_bin": None,             # None = "ffmpeg" on the PATH
        "stall_s": 10.0,                # no new video this long -> restart that camera's recorder
        "house_s": 5.0,                 # ring length + storage policy interval
    },
    "sync": {
        "enabled": True,
        "shore": None,                  # "shore-pc:5770" (the shore PC's Tailscale name); key: secrets.json sync_key
        "max_kbps": 8000,               # never faster than this
        "start_kbps": 256, "min_kbps": 32,
        "target_delay_ms": 100,         # our ping to the shore rises more than this -> slower (spare bandwidth only)
        "settle_s": 60.0,               # a file is sent when it has not changed this long
        "scan_s": 10.0,
        "chunk": 16384,
    },
    "ais": {
        "enabled": True,                # RECEIVE ONLY (no transponder): dAISy HAT / USB receiver
        "source": "/dev/serial0",       # dAISy HAT; USB receiver: "/dev/ttyACM0"; network: "tcp:host:port"
        "baud": 38400,
        "stale_a_s": 180.0,             # Class A target stale after 3 min without a position
        "stale_b_s": 360.0,             # Class B after 6 min
        "remove_s": 900.0,              # removed after 15 min
        "silent_s": 300.0,              # no AIS message this long while at sea -> AIS RECEIVER SILENT
        "at_sea_kn": 2.0,               # "at sea" = faster than this or in a driving mode (last 5 min)
        "second_gps": False,            # AIVDO / GNSS of the AIS device -> second GPS for the AURIX check
        "max_listed": 40,               # targets sent to the stations (15 with data saver)
        "online": {                     # wider area from the internet: OFF; only on sync channels
            "enabled": False,
            "url": None,                # e.g. AISHub: "http://data.aishub.net/ws.php?username=YOU&format=1&output=json&compress=0&latmin={lat0}&latmax={lat1}&lonmin={lon0}&lonmax={lon1}"
            "period_s": 60.0, "radius_deg": 0.5,
        },
    },
    "charts_dir": None,                 # None = <repo>/charts (tools/chartprep.py)
    "shallow": {                        # section C
        "draft_m": 0.8,                 # MEASURE IT: outboard leg DOWN, lowest point below the water line
        "margin_m": 1.0,                # safety margin under the leg
        "shallow_m": 3.0,               # limit = max(shallow_m, draft_m + margin_m)
        "clearance_m": 50.0,            # AUTO legs: not closer than this to shallower water
        "look_s": 30.0, "look_min_m": 200.0,   # predicted position: this far ahead
        "slow_kn": 4.0,                 # AUTO slows to this towards shallow water
        "require_charts": True,         # no charts = AUTO programs refused
    },
    "nav_s": 0.5,                       # navigation request to the AURIX every 0.5 s (lease 3 s)
    "audit_fallback": "/var/lib/trygons/audit",   # when the SSD is missing
    "web_port": 8080,                   # the iPhone page (VPN only)
    "web_helm": False,                  # HELM from the web page: OFF unless you switch it on
    "storage": {
        "root": "/mnt/ssd/trygons",     # the USB SSD; the SD card holds only the OS
        "require_mount": True,          # root must be a mounted drive (not the SD card)
        "check_s": 30,                  # free space / write test interval
        "low_free_gb": 20.0,            # below this: delete old synced data (section F)
        "full_free_gb": 2.0,            # below this: STORAGE FULL, only the ring buffer runs
        "quota_gb": None,               # optional: use at most this much of the drive
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
