"""Replay a .TLG log through the simulator to reproduce a drive.

    python tlgreplay.py LOG.TLG [--speed 10]

What is replayed, at the same boat time as recorded:
  - the joystick: stick X / Y and TAKE / AUTO / CONFIRM from the 20 Hz data
    (for a ground-station log: the recorded simulator-joystick commands)
  - the panel buttons (STANDBY, HDG LOCK, -1 / +1 / -10 / +10)
  - every PC command with its full payload (a power-limit raise uses the
    new code the simulated boat shows, as the user would type it)
The sea, the wind and the sensors are the simulator's, not the real ones,
so a real drive is reproduced in what the operator did, not exactly in
where the boat went. At the end the mode changes of the recording and of
the replay are printed side by side.
"""
import argparse
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import proto  # noqa: E402
import tlg  # noqa: E402
from link import Link, Waiter  # noqa: E402


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def mode_changes(samples, t0):
    out, last = [], None
    for s in samples:
        m = int(s["mode"])
        if m != last:
            out.append(((s["t_ms"] - t0) / 1000.0, proto.MODES[m] if m < len(proto.MODES) else str(m)))
            last = m
    return out


def actions(log):
    """(boat time ms, kind, data) to replay, in time order."""
    acts = []
    sim_joy = any(ft == proto.T_SIM_JOYSTICK for _t, ft, _s, _r, _p in log.pc_commands)
    for t, ft, _seq, _res, p in log.pc_commands:
        if ft != proto.T_HEARTBEAT:
            acts.append((t, "pc", (ft, p)))
    if not sim_joy and log.samples and "joy_x" in log.samples[0]:
        last = None
        for s in log.samples:
            j = (round(s["joy_x"], 3), round(s["joy_y"], 3), int(s.get("joy_buttons", 0)))
            if j != last:
                acts.append((s["t_ms"], "joy", j))
                last = j
    for s in log.samples:
        b = int(s.get("panel_buttons", 0))
        if b:
            acts.append((s["t_ms"], "buttons", b))
    acts.sort(key=lambda a: a[0])
    return acts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--speed", type=float, default=10.0)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    log = tlg.read(a.log)
    if not log.samples:
        print("no data in this log")
        return 1
    t_rec0 = log.samples[0]["t_ms"]
    acts = actions(log)
    sim_dir = os.path.join(HERE, "..", "sim")
    exe = os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    port = free_port()
    sim = subprocess.Popen([exe, "server", str(port), str(a.speed)], cwd=sim_dir, stdout=subprocess.DEVNULL)
    replayed = []
    try:
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
                break
            except OSError:
                time.sleep(0.1)
        time.sleep(0.2)
        link = Link("tcp:127.0.0.1:%d" % port, log_folder=os.path.join(HERE, "logs", "replay"),
                    heartbeat_s=0.5 / a.speed)
        link.start()
        w = Waiter(link)
        w.on_telemetry = replayed.append
        if w.until(lambda d: d["gps_ok"], 10) is None:
            print("no telemetry from the simulator")
            return 1
        t_sim0 = replayed[-1]["t_ms"]

        def wait_boat(rel_ms):
            while replayed[-1]["t_ms"] - t_sim0 < rel_ms:
                w.pump(0.005)

        for t, kind, data in acts:
            rel = t - t_rec0
            if rel < 0:
                continue
            wait_boat(rel)
            if kind == "pc":
                ft, p = data
                if ft == proto.T_POWER_RAISE and len(p) == 3:
                    code = w.telem["power_code"] if w.telem else 0      # what the user would read now
                    p = proto.p_power_raise(p[0], code)
                link.send(ft, bytes(p))
            elif kind == "joy":
                x, y, b = data
                link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, x, y, bool(b & 1), bool(b & 2),
                                                                      bool(b & 4)))
            elif kind == "buttons":
                for bit in range(6):
                    if data & (1 << bit):
                        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(bit))
            if not a.quiet:
                print("%8.1f s  %s %s" % (rel / 1000.0, kind, data if kind != "pc" else
                                          proto.TYPE_NAMES.get(data[0], hex(data[0]))))
        wait_boat(log.samples[-1]["t_ms"] - t_rec0)
        link.stop()
    finally:
        sim.terminate()
    rec = mode_changes(log.samples, t_rec0)
    rep = mode_changes(replayed, replayed[0]["t_ms"]) if replayed else []
    print("\nmode changes   recorded                 replayed")
    for i in range(max(len(rec), len(rep))):
        ra = "%7.1f s %-10s" % rec[i] if i < len(rec) else ""
        rb = "%7.1f s %-10s" % rep[i] if i < len(rep) else ""
        print("  %2d          %-24s %s" % (i + 1, ra, rb))
    same = [m for _, m in rec] == [m for _, m in rep]
    print("\nSAME MODE SEQUENCE" if same else "\nMODE SEQUENCE DIFFERS")
    return 0 if same else 2


if __name__ == "__main__":
    sys.exit(main())
