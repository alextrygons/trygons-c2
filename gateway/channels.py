"""Internet channels (section C): 4G (SIM 1), 4G SIM 2, RF, STARLINK.

STARLINK is the EMERGENCY link: metered, never used for sync, and in AUTO
only when every other channel is down.

Modes
  AUTO          failover between the normal channels by priority (4G, 4G SIM 2,
                RF); STARLINK only when all of them are down. A normal channel
                that is back and stable for stable_s (60 s) is taken again.
  MANUAL        the user's channel. It fails (no internet for fail_s = 30 s) ->
                the next working normal channel by priority (STARLINK last) +
                alarm CHANNEL FALLBACK; back to the chosen one when it is
                stable again. Choosing STARLINK needs an extra confirmation.
                STARLINK is never chosen automatically while a normal channel
                works - the chosen one (not yet stable) is preferred to it.
  MANUAL_STRICT only the chosen channel, never switch. Only from a station on
                the boat (a remote user could lock himself out).

Every switch a user asks for is a TRIAL (trial_s = 60 s): it is kept only if
that user, over the new channel, confirms "Keep". Otherwise the gateway goes
back to the previous mode / channel (to STARLINK only if the user had chosen
it - one that AUTO / a fallback had picked is decided again). A remote user can never lock himself
out.

Who may change it: the HELM station or a station on the boat. VIEWERs only
see the status.

Drivers (how a channel is really switched / measured):
  "rutos"   Teltonika RUT901 (RutOS 07.06+): the gateway PC logs in to the router by SSH
            (key) and uses mwan3 (RutOS failover = OpenWrt mwan3): WAN metrics,
            interface state, ping through each WAN, gsmctl for the 4G signal.
  "policy"  the gateway PC routes by itself: each provider on its own port / VLAN,
            default route + health pings per interface (ip route, ping -I).
  "sim"     tests / PC.
"""
import json
import os
import re
import shlex
import subprocess
import time

from common import log

AUTO, MANUAL, STRICT = "AUTO", "MANUAL", "MANUAL_STRICT"

DEFAULT_CHANNELS = [
    # name, label, router interface (RutOS) / gateway PC interface (policy), priority, metered, sync, emergency
    # sync_budget_kbps: upload the sync may share with the video (section G; cameras first)
    {"name": "4G", "label": "4G (SIM 1)", "wan": "mob1s1a1", "priority": 1, "metered": False,
     "sync_allowed": True, "emergency_only": False, "sync_budget_kbps": 4000},
    {"name": "4G2", "label": "4G SIM 2", "wan": "mob1s2a1", "priority": 2, "metered": False,
     "sync_allowed": True, "emergency_only": False, "sync_budget_kbps": 4000},
    {"name": "RF", "label": "RF link", "wan": "rf", "priority": 3, "metered": False,
     "sync_allowed": True, "emergency_only": False, "sync_budget_kbps": 1500},
    {"name": "STARLINK", "label": "STARLINK (emergency)", "wan": "wan", "priority": 9, "metered": True,
     "sync_allowed": False, "emergency_only": True, "monthly_warn_gb": 20.0},
]


# --- drivers -------------------------------------------------------------------------

class SimDriver:
    """Tests / PC: link states set by the test; the "active" channel is what
    the driver was told."""

    def __init__(self, names):
        self.state = {n: {"up": True, "internet": True, "latency_ms": 60.0, "loss_pct": 0.0, "signal": {}}
                      for n in names}
        self.active = None
        self.switches = []
        self.bytes = {n: 0 for n in names}

    def apply(self, active, order):
        if active != self.active:
            self.switches.append((time.time(), active))
        self.active = active

    def status(self):
        out = {}
        for n, s in self.state.items():
            d = dict(s)
            d["bytes"] = self.bytes[n]
            out[n] = d
        return out


class RutosDriver:
    """Teltonika RUT901 over SSH (RutOS = OpenWrt; failover = mwan3).

    Needs: on the router, System -> Administration -> Access control: SSH on
    LAN only, and the gateway PC's public key in /etc/dropbear/authorized_keys.
    Active channel = the mwan3 member with the lowest metric; we give the
    wanted one metric 1 and the others 10 + priority, then reload mwan3.
    """

    def __init__(self, cfg, channels):
        self.host = cfg.get("host", "10.42.0.1")
        self.user = cfg.get("ssh_user", "root")
        self.key = cfg.get("ssh_key", "/home/trygons/.ssh/id_ed25519")
        self.ping_host = cfg.get("ping_host", "1.1.1.1")
        self.channels = channels
        self.active = None
        self.last_err = None

    def _ssh(self, cmd, timeout=8):
        args = ["ssh", "-i", self.key, "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
                "-o", "StrictHostKeyChecking=accept-new", "%s@%s" % (self.user, self.host), cmd]
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        if r.returncode != 0:
            raise OSError(r.stderr.strip() or "ssh exit %d" % r.returncode)
        return r.stdout

    def apply(self, active, order):
        """active = the channel to use now (None = let mwan3 decide by `order`)."""
        cmds = []
        for i, name in enumerate(order):
            ch = self.channels[name]
            metric = 1 if name == active else 10 + i
            member = ch.get("mwan3_member", "member_%s" % ch["wan"])     # check: uci show mwan3 | grep =member
            cmds.append("uci -q set mwan3.%s.metric=%d" % (shlex.quote(member), metric))
        cmds += ["uci commit mwan3", "/etc/init.d/mwan3 reload >/dev/null 2>&1"]
        try:
            self._ssh(" ; ".join(cmds))
            self.active = active
            self.last_err = None
        except (OSError, subprocess.TimeoutExpired) as e:
            self.last_err = str(e)
            log.warning("router: switching to %s failed: %s", active, e)

    def status(self):
        out = {}
        try:
            mw = self._ssh("mwan3 interfaces 2>/dev/null; echo ---; gsmctl -q -t -o 2>/dev/null")
        except (OSError, subprocess.TimeoutExpired) as e:
            self.last_err = str(e)
            return {n: {"up": False, "internet": False, "error": "router not reachable"} for n in self.channels}
        mwan, _, gsm = mw.partition("---")
        for n, ch in self.channels.items():
            m = re.search(r"interface %s is (\w+)" % re.escape(ch["wan"]), mwan)
            online = bool(m and m.group(1) == "online")
            d = {"up": bool(m and m.group(1) in ("online", "offline")), "internet": online}
            if online:
                try:
                    ping = self._ssh("ping -I $(ubus call network.interface.%s status | jsonfilter -e '@.l3_device')"
                                     " -c 3 -W 2 %s" % (ch["wan"], self.ping_host), timeout=12)
                    rx = re.search(r"(\d+) packets received", ping)
                    avg = re.search(r"= [\d.]+/([\d.]+)/", ping)
                    d["loss_pct"] = 100.0 - 100.0 * int(rx.group(1)) / 3 if rx else None
                    d["latency_ms"] = float(avg.group(1)) if avg else None
                except (OSError, subprocess.TimeoutExpired):
                    d["latency_ms"] = None
            if ch["wan"].startswith("mob"):
                lines = gsm.strip().splitlines()
                d["signal"] = {"rssi_dbm": lines[0] if lines else None,
                               "type": lines[1] if len(lines) > 1 else None,
                               "operator": lines[2] if len(lines) > 2 else None}
                if ch["wan"].startswith("mob1s2") and not online:
                    d["note"] = "SIM 2 standby (one modem: only the active SIM can be measured)"
            out[n] = d
        return out


class PolicyDriver:
    """The gateway PC routes by itself: each provider on its own interface (a port,
    USB Ethernet adapter or VLAN), "wan" = that interface's name, "via" =
    the provider's gateway address."""

    def __init__(self, cfg, channels):
        self.channels = channels
        self.ping_host = cfg.get("ping_host", "1.1.1.1")

    def apply(self, active, order):
        ch = self.channels.get(active)
        if not ch:
            return
        subprocess.run(["ip", "route", "replace", "default", "via", ch["via"], "dev", ch["wan"]], check=False)

    def status(self):
        out = {}
        for n, ch in self.channels.items():
            up = os.path.exists("/sys/class/net/%s" % ch["wan"])
            d = {"up": up, "internet": False}
            if up:
                r = subprocess.run(["ping", "-I", ch["wan"], "-c", "3", "-W", "2", self.ping_host],
                                   capture_output=True, text=True)
                rx = re.search(r"(\d+) received", r.stdout)
                avg = re.search(r"= [\d.]+/([\d.]+)/", r.stdout)
                d["internet"] = bool(rx and int(rx.group(1)) > 0)
                d["loss_pct"] = 100.0 - 100.0 * int(rx.group(1)) / 3 if rx else None
                d["latency_ms"] = float(avg.group(1)) if avg else None
                try:
                    with open("/sys/class/net/%s/statistics/rx_bytes" % ch["wan"]) as f:
                        rxb = int(f.read())
                    with open("/sys/class/net/%s/statistics/tx_bytes" % ch["wan"]) as f:
                        d["bytes"] = rxb + int(f.read())
                except OSError:
                    pass
            out[n] = d
        return out


class StarlinkStatus:
    """The dish's local status interface (gRPC, 192.168.100.1:9200), if
    reachable and if the starlink-grpc-tools package is installed
    (pip install starlink-grpc-tools). Optional: without it the channel
    still works, only obstruction / outages / signal are not shown."""

    def __init__(self, target="192.168.100.1:9200"):
        self.target = target
        try:
            import starlink_grpc            # noqa: F401
            self.ok = True
        except ImportError:
            self.ok = False

    def read(self):
        if not self.ok:
            return {"note": "dish status not available (install starlink-grpc-tools)"}
        try:
            import starlink_grpc
            ctx = starlink_grpc.ChannelContext(target=self.target)
            st = starlink_grpc.status_data(ctx)[0]
            return {"state": st.get("state"), "obstructed_pct": round(100 * (st.get("fraction_obstructed") or 0), 1),
                    "outage": st.get("currently_obstructed"), "snr_ok": st.get("is_snr_above_noise_floor"),
                    "pop_ping_ms": st.get("pop_ping_latency_ms"), "down_mbps": round((st.get("downlink_throughput_bps") or 0) / 1e6, 1),
                    "up_mbps": round((st.get("uplink_throughput_bps") or 0) / 1e6, 1)}
        except Exception as e:
            return {"note": "dish not reachable: %s" % e}


# --- the logic ------------------------------------------------------------------------

class ChannelManager:
    def __init__(self, cfg, driver=None, clock=time.time, audit=None, state_path=None):
        lst = cfg.get("list") or DEFAULT_CHANNELS
        self.channels = {c["name"]: dict(c) for c in lst}
        self.order = [c["name"] for c in sorted(lst, key=lambda c: c["priority"])]
        self.normal = [n for n in self.order if not self.channels[n].get("emergency_only")]
        self.emergency = [n for n in self.order if self.channels[n].get("emergency_only")]
        self.stable_s = cfg.get("stable_s", 60.0)
        self.fail_s = cfg.get("fail_s", 30.0)
        self.trial_s = cfg.get("trial_s", 60.0)
        self.driver = driver or SimDriver(list(self.channels))
        self.now = clock
        self.audit = audit or (lambda *a, **k: None)
        self.state_path = state_path
        self.mode = AUTO
        self.manual = None
        self.active = None
        self.fallback = False               # MANUAL: the chosen channel failed
        self.trial = None                   # {"to", "user", "deadline", "prev": (mode, manual, active)}
        self.status = {}
        self.good_since = {}                # channel -> time it has had internet continuously
        self.bad_since = {}                 # channel -> time it lost internet
        self.saver_manual = False
        self.usage = {"day": "", "month": "", "today": {}, "month_b": {}, "last": {}}
        self.starlink_session_b = 0
        self.starlink = StarlinkStatus(cfg.get("starlink_target", "192.168.100.1:9200")) if cfg.get(
            "starlink_status", True) else None
        self.starlink_info = {}
        self._load_usage()

    # --- health ---------------------------------------------------------------------
    def healthy(self, n):
        s = self.status.get(n, {})
        return bool(s.get("up") and s.get("internet"))

    def stable(self, n, now):
        return self.healthy(n) and now - self.good_since.get(n, now) >= self.stable_s

    def failed(self, n, now):
        """No internet for fail_s."""
        return not self.healthy(n) and now - self.bad_since.get(n, now) >= self.fail_s

    def first_working(self, now, exclude=(), normal_only=False):
        for n in self.normal:
            if n not in exclude and self.healthy(n):
                return n
        if normal_only:
            return None
        for n in self.emergency:                        # the last resort
            if n not in exclude and self.healthy(n):
                return n
        return None

    # --- decisions -------------------------------------------------------------------
    def decide(self, now, fresh=False):
        """The channel to use now. fresh=True: ignore the current one (after a
        trial that was not kept: its STARLINK is no reason to stay there)."""
        if self.mode == STRICT:
            return self.manual
        cur = None if fresh else self.active
        if self.mode == MANUAL:
            want = self.manual
            if self.healthy(want) and (not self.fallback or self.stable(want, now)):
                self.fallback = False
                return want
            if self.fallback or self.failed(want, now):
                if not self.fallback:
                    self.fallback = True
                    self.audit("channel_fallback", chosen=want)
                    log.warning("channel %s failed: falling back", want)
                if cur and cur != want and self.healthy(cur):
                    return cur                           # (STARLINK: until the chosen one is stable)
                # STARLINK only if no normal channel works - the chosen one either
                return (self.first_working(now, exclude=(want,), normal_only=True) or
                        (want if self.healthy(want) else None) or
                        self.first_working(now, exclude=(want,)) or want)
            return want                                  # not failed long enough yet
        # AUTO
        best = None
        for n in self.normal:                            # best normal channel that is up
            if self.healthy(n):
                best = n
                break
        if best is None:
            for n in self.emergency:
                if self.healthy(n):
                    return n                             # every normal channel is down
            return cur
        if cur in self.normal and self.healthy(cur):
            # go back up only to a better channel that has been stable for a while
            if self.normal.index(best) < self.normal.index(cur) and self.stable(best, now):
                return best
            return cur
        if cur in self.emergency and self.healthy(cur) and not self.stable(best, now):
            return cur                                   # stay on Starlink until 4G / RF is stable
        return best

    def tick(self):
        now = self.now()
        try:
            self.status = self.driver.status()
        except Exception as e:
            log.warning("channel status: %s", e)
        for n in self.channels:
            if self.healthy(n):
                self.good_since.setdefault(n, now)
                self.bad_since.pop(n, None)
            else:
                self.bad_since.setdefault(n, now)
                self.good_since.pop(n, None)
        # trial ran out without "Keep": back to what worked before
        if self.trial and now >= self.trial["deadline"]:
            prev = self.trial["prev"]
            self.audit("channel_trial_reverted", to=self.trial["to"], user=self.trial["user"], back_to=prev[2])
            log.warning("channel trial %s not confirmed: back to %s", self.trial["to"], prev[2])
            self.mode, self.manual = prev[0], prev[1]
            self.trial = None
            # back to the channel that worked before - but STARLINK only if the
            # user had chosen it (MANUAL + confirmed); one that AUTO / a fallback
            # had picked is decided again (a normal channel may work by now)
            back = prev[2]
            chosen_emergency = prev[0] in (MANUAL, STRICT) and prev[1] in self.emergency
            if back in self.emergency and not chosen_emergency:
                back = None
            self._switch(back if back and self.healthy(back) else self.decide(now, fresh=True),
                         "trial not confirmed")
        want = self.decide(now)
        if want != self.active:
            self._switch(want, "%s mode" % self.mode)
        self._count(now)
        if self.starlink and self.active in self.emergency:
            self.starlink_info = self.starlink.read()

    def _switch(self, to, why):
        old = self.active
        self.active = to
        self.driver.apply(to, self.order)
        if to in self.emergency and old not in self.emergency:
            self.starlink_session_b = 0
        self.audit("channel", frm=old, to=to, why=why, mode=self.mode)
        log.info("internet channel %s -> %s (%s)", old, to, why)

    # --- requests from stations ------------------------------------------------------
    def request(self, mode, channel=None, user="?", local=False, may=True, confirm_emergency=False,
                switch_now=True):
        """-> (ok, text, needs_confirm). switch_now=False: the next tick() switches
        (the gateway first answers the station - its link may go away)."""
        if not may:
            return False, "only the HELM station or a station on the boat may change the channel", False
        if mode not in (AUTO, MANUAL, STRICT):
            return False, "unknown mode", False
        if mode != AUTO and channel not in self.channels:
            return False, "unknown channel", False
        if mode == STRICT and not local:
            return False, "MANUAL STRICT only from a station on the boat", False
        if mode != AUTO and channel in self.emergency and not confirm_emergency:
            return False, "EMERGENCY LINK - metered data. Confirm to use STARLINK.", True
        now = self.now()
        prev = (self.mode, self.manual, self.active)
        self.mode, self.manual = mode, (channel if mode != AUTO else None)
        self.fallback = False
        self.trial = {"to": channel or "AUTO", "user": user, "deadline": now + self.trial_s, "prev": prev}
        self.audit("channel_request", mode=mode, channel=channel, user=user, local=local)
        want = self.decide(now)
        if want != self.active and switch_now:
            self._switch(want, "asked by %s" % user)
        text = "%s%s: trial - press KEEP within %.0f s over the new link, otherwise it goes back" % (
            mode, " " + channel if channel else "", self.trial_s)
        if mode == STRICT:
            text = "WARNING: MANUAL STRICT never switches, even if %s fails. " % channel + text
        return True, text, False

    def keep(self, user):
        if not self.trial:
            return False, "nothing to keep"
        if user != self.trial["user"]:
            return False, "only %s can keep this change" % self.trial["user"]
        self.audit("channel_kept", to=self.trial["to"], user=user)
        self.trial = None
        return True, "kept"

    # --- data counters -----------------------------------------------------------------
    def _load_usage(self):
        if self.state_path and os.path.exists(self.state_path):
            try:
                with open(self.state_path) as f:
                    self.usage.update(json.load(f))
            except (OSError, ValueError):
                pass

    def _count(self, now):
        day = time.strftime("%Y%m%d", time.gmtime(now))
        mon = day[:6]
        u = self.usage
        if u["day"] != day:
            u["day"], u["today"] = day, {}
        if u["month"] != mon:
            u["month"], u["month_b"] = mon, {}
        for n, s in self.status.items():
            b = s.get("bytes")
            if b is None:
                continue
            last = u["last"].get(n)
            u["last"][n] = b
            if last is None or b < last:            # first reading / counter reset
                continue
            d = b - last
            u["today"][n] = u["today"].get(n, 0) + d
            u["month_b"][n] = u["month_b"].get(n, 0) + d
            if n in self.emergency and n == self.active:
                self.starlink_session_b += d
        if self.state_path:
            try:
                with open(self.state_path + ".tmp", "w") as f:
                    json.dump(u, f)
                os.replace(self.state_path + ".tmp", self.state_path)
            except OSError:
                pass

    # --- for the gateway -------------------------------------------------------------------
    @property
    def on_emergency(self):
        return self.active in self.emergency

    def sync_allowed(self):
        ch = self.channels.get(self.active)
        return bool(ch and ch.get("sync_allowed") and not ch.get("emergency_only"))

    def data_saver(self):
        return self.on_emergency or self.saver_manual

    def channel_id(self):
        return self.order.index(self.active) + 1 if self.active in self.order else 0

    def info(self):
        now = self.now()
        out = []
        for n in self.order:
            ch, s = self.channels[n], self.status.get(n, {})
            out.append({"name": n, "label": ch.get("label", n), "active": n == self.active,
                        "up": s.get("up"), "internet": s.get("internet"), "latency_ms": s.get("latency_ms"),
                        "loss_pct": s.get("loss_pct"), "signal": s.get("signal"), "note": s.get("note"),
                        "metered": ch.get("metered"), "sync_allowed": ch.get("sync_allowed"),
                        "emergency_only": ch.get("emergency_only"),
                        "today_mb": round(self.usage["today"].get(n, 0) / 1e6, 1),
                        "month_mb": round(self.usage["month_b"].get(n, 0) / 1e6, 1)})
        st = None
        if self.emergency:
            e = self.emergency[0]
            warn_gb = self.channels[e].get("monthly_warn_gb")
            month_gb = self.usage["month_b"].get(e, 0) / 1e9
            st = {"session_mb": round(self.starlink_session_b / 1e6, 1), "month_gb": round(month_gb, 2),
                  "warn_gb": warn_gb, "warning": bool(warn_gb and month_gb >= warn_gb), "dish": self.starlink_info}
        return {"mode": self.mode, "manual": self.manual, "active": self.active, "fallback": self.fallback,
                "emergency": self.on_emergency, "saver": self.data_saver(), "sync_allowed": self.sync_allowed(),
                "trial": None if not self.trial else {"to": self.trial["to"], "user": self.trial["user"],
                                                      "left_s": max(0, round(self.trial["deadline"] - now))},
                "channels": out, "starlink": st}
