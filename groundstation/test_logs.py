"""Tests of the log tools (run after building the simulator):

  1. round trip: the simulator writes a log and its own exact values
     (autopilot_sim logtest) -> log2csv -> CSV values match the simulator
     to the stored resolution
  2. the ground station's own .TLG copy (from the last test_e2e run) has
     every telemetry frame and the commands
  3. replay: that log through the simulator -> same mode sequence

    python test_logs.py
"""
import csv
import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tlg  # noqa: E402

PY = sys.executable
results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def main():
    sim_dir = os.path.join(HERE, "..", "sim")
    exe = os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")

    # 1. round trip
    out_dir = "sd_pyrt"
    subprocess.run([exe, "logtest", out_dir], cwd=sim_dir, check=True, stdout=subprocess.DEVNULL)
    d = os.path.join(sim_dir, out_dir)
    log_path = sorted(glob.glob(os.path.join(d, "B*.TLG")))[0]
    csv_path = os.path.join(d, "out.csv")
    subprocess.run([PY, os.path.join(HERE, "log2csv.py"), log_path, "-o", csv_path], check=True,
                   stdout=subprocess.DEVNULL)
    rows = {int(r["t_ms"]): r for r in csv.DictReader(open(csv_path, encoding="utf-8"))}
    res = {"mode": 1, "heading": 0.01, "thr_out": 0.01, "est_lat": 1e-7, "est_lon": 1e-7, "unc": 0.1,
           "speed_target": 0.01, "gps_sog": 0.01, "alarm_bits": 1}
    worst, worst_f, found, n = 0.0, "", 0, 0
    for r in csv.DictReader(open(os.path.join(d, "ref.csv"), encoding="utf-8")):
        n += 1
        row = rows.get(int(r["t_ms"]))
        if row is None:
            continue
        found += 1
        for f, step in res.items():
            e = abs(float(row[f]) - float(r[f])) / step
            if e > worst:
                worst, worst_f = e, f
    report("Round trip: simulator -> .TLG -> CSV, values match the simulator",
           found == n and n >= 290 and worst <= 0.5 + 1e-6,
           "%d/%d reference samples found in the CSV, worst difference %.3f x the stored resolution (%s)"
           % (found, n, worst, worst_f))

    # 2. the ground station's own copy
    logs = sorted(glob.glob(os.path.join(HERE, "logs", "e2e", "*.TLG")))
    if not logs:
        subprocess.run([PY, os.path.join(HERE, "test_e2e.py")], check=True, stdout=subprocess.DEVNULL)
        logs = sorted(glob.glob(os.path.join(HERE, "logs", "e2e", "*.TLG")))
    gs = tlg.read(logs[-1])
    js = [json.loads(line) for line in open(logs[-1][:-4] + ".jsonl", encoding="utf-8")]
    n_tel = sum(1 for x in js if "telemetry" in x)
    n_cmd = sum(1 for x in js if "ack" in x)
    report("Ground station saves its telemetry in the same .TLG format",
           len(gs.samples) == n_tel and len(gs.pc_commands) == n_cmd and gs.crc_errors == 0 and
           "heading" in gs.samples[0],
           "%s: %d telemetry samples (jsonl %d), %d commands with answers (jsonl %d), %d events, CRC errors %d"
           % (os.path.basename(logs[-1]), len(gs.samples), n_tel, len(gs.pc_commands), n_cmd, len(gs.events),
              gs.crc_errors))

    # 3. replay
    r = subprocess.run([PY, os.path.join(HERE, "tlgreplay.py"), logs[-1], "--quiet"], capture_output=True, text=True)
    tail = r.stdout.strip().splitlines()
    report("Replay of a log through the simulator: same mode sequence", r.returncode == 0,
           " | ".join(l.strip() for l in tail[-8:]))
    print("LOG TESTS: %d passed, %d failed" % (sum(results), len(results) - sum(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
