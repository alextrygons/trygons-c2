"""The ONLY bridge between the UAV magazine (gateway/uav/, CLAUDE.md V) and the boat side of the gateway (U3).

Nothing in gateway/uav/ imports aurix / proto / stations (test_uav UA5): what the UAV code needs of the boat comes
from here as plain values, and its one boat action - the hatch relay - goes from here through stations.py on the
HELM station's behalf (the AURIX checks HELM / the relay table / FIRST TEST as always; R7).

  boat_state(gw, hatch_relay) -> dict   the launch conditions (docs/uav_design.md 5.3) as values + "why" texts
  hatch_relay(gw, n, on, done)          the T_RELAY (stations.uav_relay)
"""
import time

import proto

SLOW_KN = 2.0


def warning_alarms(t):
    """The boat's WARNING-level alarms now (the ground station's levels, alerts.py) - names, [] = none."""
    try:
        import alerts
    except Exception:                                   # (no i18n / lang files: every alarm counts)
        return ["alarm bits %x" % (t.get("alarm_bits", 0) | t.get("alarm_bits2", 0) << 32)] if (
            t.get("alarm_bits") or t.get("alarm_bits2")) else []
    return [c for _b, c, lv in alerts.active(alerts.bits_of(t)) if lv == alerts.WARNING]


def boat_state(gw, hatch_relay, max_kn=SLOW_KN):
    """Everything the launch sequence asks of the boat. "launch_why" = [] when a launch may go on."""
    t = gw.telem_fresh() if hasattr(gw, "telem_fresh") else None
    t2 = getattr(gw, "telem2", None) or {}
    why = []
    out = {"fresh": t is not None, "mode": None, "standby": False, "slow_mode": False, "sog_kn": None,
           "gps": False, "pos": None, "helm": None, "captain": None, "launch_why": why, "relay_ok": False}
    if t is None:
        why.append("no fresh boat telemetry")
        return out
    mode = t.get("mode")
    out["mode"] = proto.MODE_NAMES[mode] if hasattr(proto, "MODE_NAMES") and isinstance(mode, int) and \
        mode < len(proto.MODE_NAMES) else t.get("mode_name")
    out["standby"] = mode == proto.MODE_STANDBY
    dp_hold = mode == proto.MODE_DP and t2.get("dp_sub") in (proto.DP_SPOT, proto.DP_JOG)
    out["slow_mode"] = out["standby"] or dp_hold
    if not out["slow_mode"]:
        why.append("the boat is not in STANDBY or DP SPOT / JOG (%s)" % (t.get("mode_name") or mode))
    gps = t.get("pos_source") == 1
    out["gps"] = gps
    if gps:
        out["pos"] = (t.get("est_lat"), t.get("est_lon"))
    else:
        why.append("no trusted boat GPS")
    sog = t.get("est_sog")
    out["sog_kn"] = sog
    if sog is None or sog >= max_kn:
        why.append("the boat is not slower than %.1f kn (%s)" % (max_kn, "-" if sog is None else "%.1f" % sog))
    # U3 review M2: BOTH alarm words - the second one (TELEM2 group L) from the newest TELEM2, not the copy merged into
    # the telemetry when it arrived
    tt = dict(t)
    if "alarm_bits2" in t2:
        tt["alarm_bits2"] = t2.get("alarm_bits2") or 0
    warn = warning_alarms(tt)
    if warn:
        why.append("boat WARNING alarm: %s" % ", ".join(warn[:3]))
    out["relay_ok"] = bool(t2.get("relay_ok"))
    if not t2.get("relay_ok"):
        why.append("the relay board does not answer")
    ok, w = gw.stations.hatch_relay_ok(hatch_relay)
    if not ok:
        why.append(w)
    st, w = gw.stations.helm_station()
    if st is None:
        why.append(w)
    else:
        out["helm"] = st.id
    cap = gw.stations.captain()
    out["captain"] = cap.id if cap is not None else None
    out["relay_on"] = list(t2.get("relay_on") or [])
    out["t"] = time.time()
    return out


def relay_on(gw, n):
    """The hatch relay's readback on the AURIX: True / False, None = unknown (TELEM2 group M)."""
    t2 = getattr(gw, "telem2", None) or {}
    if not n or not t2.get("relay_ok") or n in (t2.get("relay_unknown") or []):
        return None
    return n in (t2.get("relay_on") or [])


def hatch_relay(gw, n, on, done):
    gw.stations.uav_relay(n, on, done)
