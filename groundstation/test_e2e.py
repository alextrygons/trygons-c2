"""End-to-end test: ground-station link <-> simulator over TCP.

Starts  sim/autopilot_sim server <port> 10  (10x real time), then:
  1. AUTO program upload (a bad one is refused), start + joystick CONFIRM, arrival
  2. AUTO again, and while it runs a simulated joystick takes over
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
import program  # noqa: E402
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

        def confirm():
            w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
            time.sleep(0.15 / SPEED * 10)
            w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))

        # 0. power limit: 30 % at start; wrong code refused; code + CONFIRM -> 50 %
        p0 = w.telem["power_fwd"]
        w.command(proto.T_POWER_CODE, proto.p_power_code(0))
        tc = w.until(lambda d: d["power_code"] != 0, 3)
        wrong = w.command(proto.T_POWER_RAISE, proto.p_power_raise(0, (tc["power_code"] % 9000) + 1000
                                                                    if tc else 1234))
        w.command(proto.T_POWER_CODE, proto.p_power_code(0))
        tc = w.until(lambda d: d["power_code"] != 0, 3)
        good = w.command(proto.T_POWER_RAISE, proto.p_power_raise(0, tc["power_code"])) if tc else None
        asking = w.until(lambda d: d["confirm_kind"] == 3, 3)
        confirm()
        up = w.until(lambda d: d["power_fwd"] == 50.0, 3)
        report("Power limit from the ground station: code + joystick CONFIRM, one step",
               bool(p0 == 30.0 and wrong and wrong["result"] == 15 and good and good["ok"] and asking and up),
               "start %.0f %%; wrong code: %s; right code: %s, CONFIRM asked %s; now %s"
               % (p0, wrong and wrong["text"], good and good["text"], "yes" if asking else "NO",
                  "%.0f %%" % up["power_fwd"] if up else "unchanged"))

        # 1. AUTO program: a bad upload is refused, a good one runs to the end

        lat0, lon0 = t["est_lat"], t["est_lon"]
        wp1 = offset(lat0, lon0, 0.0, 300.0)
        wp2 = offset(lat0, lon0, 0.0, 600.0)
        bad = {"final": 0, "wps": [{"lat": wp1[0], "lon": wp1[1], "kn": 40.0, "radius": 25.0, "action": 0}]}
        ok_bad, txt_bad = program.upload(w, bad)
        prog = {"final": 0, "wps": [{"lat": wp1[0], "lon": wp1[1], "kn": 8.0, "radius": 25.0, "action": 0},
                                    {"lat": wp2[0], "lon": wp2[1], "kn": 6.0, "radius": 25.0, "action": 0}]}
        ok_up, txt_up = program.upload(w, prog)
        a2 = w.command(proto.T_AUTO_START)
        waiting = w.until(lambda d: d["confirm_kind"] == 1, 3)
        confirm()
        aut = w.until(lambda d: d["mode"] == proto.MODE_AUTO, 3)
        t0 = time.time()
        arr = w.until(lambda d: d["mode"] == proto.MODE_STANDBY and d["alarm_bits"] & 0x20, 60)
        dt = (time.time() - t0) * SPEED
        miss = dist_m(arr["est_lat"], arr["est_lon"], wp2[0], wp2[1]) if arr else -1
        report("AUTO program from the ground station: bad one refused, 2 legs, CONFIRM, arrival -> STANDBY",
               bool(not ok_bad and ok_up and a2 and a2["ok"] and waiting and aut and arr and miss < 30),
               "bad upload: %s; good: %s; start %s, CONFIRM asked %s, AUTO %s, arrived after %.0f s sim, "
               "%.1f m from the last waypoint, alarms %s"
               % (txt_bad, txt_up, a2 and a2["text"], "yes" if waiting else "NO", "yes" if aut else "NO", dt,
                  miss, arr and proto.alarm_texts(arr["alarm_bits"])))

        # 2. again, and while it runs a simulated joystick takes over
        w.command(proto.T_ALARM_ACK)
        t = w.telem
        wp3 = offset(t["est_lat"], t["est_lon"], 600.0, 0.0)
        program.upload(w, {"final": 0, "wps": [{"lat": wp3[0], "lon": wp3[1], "kn": 10.0, "radius": 25.0,
                                               "action": 0}]})
        a3 = w.command(proto.T_AUTO_START)
        confirm()
        moving = w.until(lambda d: d["mode"] == proto.MODE_AUTO and d["est_sog"] > 6.0, 10)
        before = w.telem
        a4 = w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.5, 0.0))
        joy = w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 3)
        sw = (joy["t_ms"] - before["t_ms"]) / 1000.0 if joy and before else -1
        report("Simulated joystick takes over during AUTO",
               bool(a3 and a3["ok"] and moving and a4 and a4["ok"] and joy and sw <= 0.4 and joy["prog_suspended"]),
               "AUTO at %.1f kn, MANUAL %.2f s (sim) after the deflection, program suspended %s, x %.2f"
               % (moving["est_sog"] if moving else -1, sw, joy and joy["prog_suspended"], joy["joy_x"] if joy else 0))

        # 3. PC refused during the override; hand-back rules (joystick AUTO button)
        r1 = w.command(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))
        r2 = w.command(proto.T_AUTO_PAUSE)
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.5, 0.0, autob=True))    # AUTO, not centred
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.5, 0.0))
        still = w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 1)
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, -0.5))
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, -1.0))      # full astern
        rev = w.until(lambda d: d["gear"] == 2 and d["throttle"] > 3.0, 5)      # reverse limit 5 % after a takeover
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, -0.5))
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))       # centre
        idle = w.until(lambda d: d["gear"] == 0 and d["throttle"] < 1.0, 5)
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, autob=True))    # AUTO, centred
        w.command(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        sb = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 3)
        report("PC refused while the joystick is in control, hand-back only centred",
               bool(r1 and r1["result"] == 4 and r2 and r2["result"] == 4 and still
                    and rev and rev["throttle"] <= rev["power_rev"] + 0.01 and idle and sb),
               "PC STANDBY: %s; PC pause: %s; joystick AUTO deflected: %s; astern throttle %.0f %%; "
               "centred AUTO -> %s"
               % (r1 and r1["text"], r2 and r2["text"], "stays MANUAL" if still else "LEFT MANUAL",
                  rev["throttle"] if rev else -1, sb and sb["mode_name"]))

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
