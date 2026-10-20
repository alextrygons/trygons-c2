"""Replay a ground-station log (logs/log_*.jsonl).

  python replay.py LOG --sim  [--speed 10]
      Starts the simulator and re-sends every recorded PC command at the
      same boat time as in the recording (simulation runs speed x faster). Prints the mode changes of the
      recording next to those of the replay, so you can check that the
      same commands give the same behaviour after a software change.
      Note: joystick moves and GPS failures from the simulator controls
      are commands too and are replayed; the real boat's sea, wind and
      sensors are not, so a real-boat log will not match exactly.

  python replay.py LOG --view [--port 5761] [--speed 1]
      Pretends to be the boat: serves the recorded telemetry on a TCP port.
      Connect the ground station to  tcp:127.0.0.1:5761  to watch it again.
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import proto  # noqa: E402
from link import Link, Waiter  # noqa: E402


def load(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def mode_changes(telemetry):
    out, last = [], None
    t0 = telemetry[0]["t_ms"] if telemetry else 0
    for d in telemetry:
        if d["mode"] != last:
            out.append(((d["t_ms"] - t0) / 1000.0, proto.MODES[d["mode"]]))
            last = d["mode"]
    return out


def replay_sim(recs, speed):
    # schedule every command by the boat's clock (t_ms of the last telemetry
    # before it), so the replay matches whatever speed the log was made at
    rec_tel, cmds, boat_t = [], [], None
    for r in recs:
        if "telemetry" in r:
            rec_tel.append(r["telemetry"])
            boat_t = r["telemetry"]["t_ms"]
        elif r["dir"] == "tx" and boat_t is not None:
            cmds.append((boat_t - rec_tel[0]["t_ms"], r))
    if not cmds:
        print("no commands (after the first telemetry) in this log")
        return 1
    sim_dir = os.path.join(HERE, "..", "sim")
    exe = os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    port = 5790
    sim = subprocess.Popen([exe, "server", str(port), str(speed)], cwd=sim_dir, stdout=subprocess.DEVNULL)
    try:
        time.sleep(0.8)
        link = Link("tcp:127.0.0.1:%d" % port, log_folder=os.path.join(HERE, "logs", "replay"),
                    heartbeat_s=0.5 / speed)
        link.start()
        w = Waiter(link)
        new_tel = []
        w.on_telemetry = new_tel.append
        if w.until(lambda d: True, 5) is None:
            print("no telemetry from the simulator")
            return 1
        t0 = new_tel[0]["t_ms"]

        def wait_boat(rel_ms):
            while new_tel[-1]["t_ms"] - t0 < rel_ms:
                w.pump(0.01)

        for rel, c in cmds:
            wait_boat(rel)
            a = w.command(c["type"], bytes.fromhex(c["hex"]), timeout=2.0)
            print("%7.1f s  %-22s -> %s" % (rel / 1000.0, proto.TYPE_NAMES.get(c["type"], hex(c["type"])),
                                            a["text"] if a else "no answer"))
        wait_boat(rec_tel[-1]["t_ms"] - rec_tel[0]["t_ms"])
        link.stop()
    finally:
        sim.terminate()
    a, b = mode_changes(rec_tel), mode_changes(new_tel)
    print("\nmode changes   recorded                 replayed")
    for i in range(max(len(a), len(b))):
        ra = "%6.1f s %-10s" % a[i] if i < len(a) else ""
        rb = "%6.1f s %-10s" % b[i] if i < len(b) else ""
        print("  %2d          %-24s %s" % (i + 1, ra, rb))
    same = [m for _, m in a] == [m for _, m in b]
    print("\nSAME MODE SEQUENCE" if same else "\nMODE SEQUENCE DIFFERS")
    return 0 if same else 1


def replay_view(recs, port, speed):
    tel = [r for r in recs if r["dir"] == "rx" and r["type"] == proto.T_TELEMETRY]
    if not tel:
        print("no telemetry in this log")
        return 1
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print("replaying %d telemetry frames: connect the ground station to tcp:127.0.0.1:%d" % (len(tel), port))
    while True:
        conn, _ = srv.accept()
        conn.settimeout(0.001)
        print("ground station connected, playing...")
        t0, w0 = tel[0]["t"], time.time()
        try:
            for i, r in enumerate(tel):
                while time.time() < w0 + (r["t"] - t0) / speed:
                    try:
                        conn.recv(4096)        # ignore its heartbeats / commands
                    except socket.timeout:
                        pass
                conn.sendall(proto.encode(r["type"], i & 0xFF, bytes.fromhex(r["hex"])))
            print("end of log")
        except OSError:
            print("ground station disconnected")
        conn.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", action="store_true", help="re-send the commands to the simulator")
    g.add_argument("--view", action="store_true", help="serve the telemetry to the ground station")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--port", type=int, default=5761)
    a = ap.parse_args()
    recs = load(a.log)
    if a.sim:
        return replay_sim(recs, a.speed or 10.0)
    return replay_view(recs, a.port, a.speed or 1.0)


if __name__ == "__main__":
    sys.exit(main())
