"""End-to-end test: ground-station link <-> simulator over TCP.

Starts  sim/autopilot_sim server <port> 10  (10x real time), then:
  1. NAV mission to a waypoint 500 m east -> arrival -> STANDBY
  2. second NAV leg, and while it runs a simulated joystick takes over
  3. the PC is refused during the override, hand-back only with the stick centred
  4. every frame was logged

Run:  python test_e2e.py        (exit code 0 = all passed)
"""
import glob
import json
import math
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import proto  # noqa: E402
from link import Link, Waiter  # noqa: E402

SPEED = 10.0
results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def offset(lat, lon, north_m, east_m):
    return (lat + north_m / 111320.0,
            lon + east_m / (111320.0 * math.cos(math.radians(lat))))


def dist_m(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * 111320.0
    de = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def main():
    sim_dir = os.path.join(HERE, "..", "sim")
    exe = os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    port = free_port()
    sim = subprocess.Popen([exe, "server", str(port), str(SPEED)], cwd=sim_dir,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    log_dir = os.path.join(HERE, "logs", "e2e")
    link = None
    try:
        for _ in range(50):                        # wait until it listens
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
                break
            except OSError:
                time.sleep(0.1)
        time.sleep(0.2)                            # let it notice the probe went away
        link = Link("tcp:127.0.0.1:%d" % port, log_folder=log_dir, heartbeat_s=0.5 / SPEED)
        link.start()
        w = Waiter(link)
        n_telem = [0]
        w.on_telemetry = lambda d: n_telem.__setitem__(0, n_telem[0] + 1)

        t = w.until(lambda d: d["gps_ok"] and d["pos_source"] == 1, 10)
        report("Telemetry arrives, GPS fix", t is not None,
               "mode %s, position %.6f %.6f" % (t["mode_name"], t["est_lat"], t["est_lon"]) if t else "no telemetry")
        if t is None:
            return 1

        # 1. NAV mission
        wp = offset(t["est_lat"], t["est_lon"], 0.0, 500.0)
        a1 = w.command(proto.T_WAYPOINT, proto.p_waypoint(wp[0], wp[1], 8.0))
        a2 = w.command(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_NAV))
        nav = w.until(lambda d: d["mode"] == proto.MODE_NAV, 3)
        t0 = time.time()
        arr = w.until(lambda d: d["mode"] == proto.MODE_STANDBY and d["alarm_bits"] & 0x20, 60)
        dt = (time.time() - t0) * SPEED
        miss = dist_m(arr["est_lat"], arr["est_lon"], wp[0], wp[1]) if arr else -1
        report("NAV mission from the ground station: 500 m leg, arrival -> STANDBY",
               bool(a1 and a1["ok"] and a2 and a2["ok"] and nav and arr and miss < 30),
               "waypoint %s, NAV %s, arrived after %.0f s sim, %.1f m from the waypoint, alarms %s"
               % (a1 and a1["text"], a2 and a2["text"], dt, miss, arr and proto.alarm_texts(arr["alarm_bits"])))

        # 2. second leg, joystick takes over half way
        w.command(proto.T_ALARM_ACK)
        t = w.telem
        wp2 = offset(t["est_lat"], t["est_lon"], 600.0, 0.0)
        w.command(proto.T_WAYPOINT, proto.p_waypoint(wp2[0], wp2[1], 10.0))
        a3 = w.command(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_NAV))
        moving = w.until(lambda d: d["mode"] == proto.MODE_NAV and d["est_sog"] > 6.0, 10)
        before = w.telem
        a4 = w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.5, 0.0))
        joy = w.until(lambda d: d["mode"] == proto.MODE_JOYSTICK, 3)
        sw = (joy["t_ms"] - before["t_ms"]) / 1000.0 if joy and before else -1
        report("Simulated joystick takes over during NAV",
               bool(a3 and a3["ok"] and moving and a4 and a4["ok"] and joy and sw <= 0.4),
               "NAV at %.1f kn, JOYSTICK %.2f s (sim) after the deflection, banner data: status %s, x %.2f"
               % (moving["est_sog"] if moving else -1, sw, joy and proto.JOY_STATUS[joy["joy_status"]],
                  joy["joy_x"] if joy else 0))

        # 3. PC refused during the override; hand-back rules
        r1 = w.command(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_NAV))
        r2 = w.command(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
        r3 = w.command(proto.T_JOY_HANDBACK)
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, -1.0))      # full astern
        rev = w.until(lambda d: d["gear"] == 2 and d["throttle"] > 20.0, 5)
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))       # centre
        idle = w.until(lambda d: d["gear"] == 0 and d["throttle"] < 1.0, 5)
        time.sleep(0.1)
        r4 = w.command(proto.T_JOY_HANDBACK)
        sb = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 3)
        report("PC refused while the joystick is in control, hand-back only centred",
               bool(r1 and r1["result"] == 4 and r2 and r2["result"] == 4 and r3 and r3["result"] == 8
                    and rev and rev["throttle"] <= 30.01 and idle and r4 and r4["ok"] and sb),
               "PC NAV: %s; PC STANDBY: %s; hand-back deflected: %s; astern throttle %.0f %%; "
               "centred hand-back: %s -> %s"
               % (r1 and r1["text"], r2 and r2["text"], r3 and r3["text"], rev["throttle"] if rev else -1,
                  r4 and r4["text"], sb and sb["mode_name"]))

        # 4. log
        link.stop()
        while not link.events.empty():             # count what was still queued
            w.pump(0)
        path = link.logger.path
        link = None
        lines = [json.loads(s) for s in open(path, encoding="utf-8")]
        tel = sum(1 for r in lines if "telemetry" in r)
        tx = sum(1 for r in lines if r["dir"] == "tx")
        report("Every telemetry frame logged", tel == n_telem[0] and tel > 100 and tx >= 10,
               "%s: %d telemetry frames logged (%d received), %d commands" %
               (os.path.relpath(path, HERE), tel, n_telem[0], tx))
    finally:
        if link:
            link.stop()
        sim.terminate()
        try:
            sim.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sim.kill()
        for old in sorted(glob.glob(os.path.join(log_dir, "*.jsonl")))[:-3]:
            os.remove(old)                         # keep the last 3 e2e logs
    print("E2E: %d passed, %d failed" % (sum(results), len(results) - sum(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
