"""Step FW-TR: the acceleration frame (BNO085 3 axes, DISPLAY / LOGGING ONLY) on the PC side.

  AP1 groundstation/proto.py = app/proto.h: T_ACCEL 0x8F, the 28-byte layout (PROTO_ACCEL_LEN)
  AP2 decode_accel: average + peak per axis in g, None where the boat had no sample / no fresh second; a short frame
      is refused; encode_accel is its inverse (milli-g resolution)
  AP3 the pass-through: the gateway sends {"t": "acc"} to the full-rate stations only (stations._on_acc); the station
      (gwclient) keeps it and merges it into the next telemetry dict like TELEM2; the USB link (link.py) the same
  AP4 the log: the simulator's .TLG (autopilot_sim logtest) has event 31 ACCEL once a second - tlg.accel_rows and
      log2csv --accel read it (values in g, ~100 samples a second); the shortened HELM header entry still reads with
      its action names

    python test_accel.py
"""
import csv
import glob
import os
import queue
import re
import struct
import subprocess
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
os.environ.setdefault("TRYGONS_TEST", "1")
import proto  # noqa: E402
import tlg  # noqa: E402

PY = sys.executable
results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def main():
    # AP1
    h = open(os.path.join(ROOT, "app", "proto.h"), encoding="utf-8").read()
    t = re.search(r"#define PROTO_T_ACCEL\s+0x([0-9A-Fa-f]+)u", h)
    n = re.search(r"#define PROTO_ACCEL_LEN\s+(\d+)u", h)
    ok = (t is not None and n is not None and int(t.group(1), 16) == proto.T_ACCEL == 0x8F and
          int(n.group(1)) == proto.ACCEL_LEN == struct.calcsize("<BB6h6hH") and
          proto.TYPE_NAMES.get(proto.T_ACCEL) == "acceleration")
    report("AP1 proto.py = app/proto.h (T_ACCEL, 28 bytes)", ok,
           "proto.h 0x%s / %s bytes, proto.py 0x%02X / %d" % (t and t.group(1), n and n.group(1), proto.T_ACCEL,
                                                             proto.ACCEL_LEN))

    # AP2
    per = ((0.052, -0.31), (0.004, 0.12), (-0.02, 2.48))
    sec = ((0.031, -0.4), (-0.006, 0.2), (0.011, -3.1))
    d = proto.decode_accel(proto.encode_accel(per, sec, n=10, n1=100))
    near = all(abs(d["acc_" + a] - per[k][0]) < 6e-4 and abs(d["acc_" + a + "_pk"] - per[k][1]) < 6e-4 and
               abs(d["acc1_" + a] - sec[k][0]) < 6e-4 and abs(d["acc1_" + a + "_pk"] - sec[k][1]) < 6e-4
               for k, a in enumerate(proto.ACCEL_AXES))
    e = proto.decode_accel(proto.encode_accel(None, None))
    empty = all(e[k] is None for k in e if k.startswith("acc") and not k.endswith("_n")) and e["acc1_n"] == 0
    p_only = proto.decode_accel(proto.encode_accel(per, None, n=9, n1=100))
    try:
        proto.decode_accel(b"\x01" * 27)
        short = False
    except ValueError:
        short = True
    report("AP2 decode_accel: avg + peak per axis (g), None without samples, a short frame refused",
           near and empty and p_only["acc1_lon"] is None and p_only["acc_lat_pk"] is not None and short and
           d["acc_n"] == 10 and d["acc1_n"] == 100,
           "decoded %s; empty frame all None %s; short frame refused %s" % (
               {k: d[k] for k in ("acc_lon", "acc_vert_pk", "acc1_vert_pk", "acc1_n")}, empty, short))

    # AP3 the gateway's pass-through + the station's merge
    sys.path.insert(0, os.path.join(ROOT, "gateway"))
    import stations as S  # noqa: E402
    sent = {"full": [], "lite": []}
    sts = {2: types.SimpleNamespace(tmode="full", send=lambda m: sent["full"].append(m)),
           3: types.SimpleNamespace(tmode="lite", send=lambda m: sent["lite"].append(m))}
    payload = proto.encode_accel(per, sec, n=10, n1=100)
    S.StationServer._on_acc(types.SimpleNamespace(stations=sts), proto.Frame(proto.T_ACCEL, 0, payload))
    gw_ok = sent["full"] == [{"t": "acc", "b": payload.hex()}] and sent["lite"] == []
    import gwclient  # noqa: E402
    g = gwclient.GatewayLink.__new__(gwclient.GatewayLink)
    g.events, g.t2, g.acc, g.logger, g.last_full = queue.Queue(), {"helm": 2}, {}, None, None
    g._dispatch2({"t": "acc", "b": payload.hex()})
    st_ok = abs(g.acc.get("acc_vert_pk", 0) - 2.48) < 6e-4 and g.events.empty()
    src_gw = open(os.path.join(HERE, "gwclient.py"), encoding="utf-8").read()
    src_usb = open(os.path.join(HERE, "link.py"), encoding="utf-8").read()
    merged = "d.update(self.acc)" in src_gw and "d.update(self.acc)" in src_usb and "proto.T_ACCEL" in src_usb
    report("AP3 the gateway passes ACCEL to full-rate stations only; the station merges it into the telemetry",
           gw_ok and st_ok and merged,
           "gateway -> full %d message(s), lite %d; station kept vert peak %s; merged (gateway / USB link) %s" % (
               len(sent["full"]), len(sent["lite"]), g.acc.get("acc_vert_pk"), merged))

    # AP4 the log
    sim_dir = os.path.join(ROOT, "sim")
    exe = os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    out_dir = "sd_axpy"
    subprocess.run([exe, "logtest", out_dir], cwd=sim_dir, check=True, stdout=subprocess.DEVNULL)
    log_path = sorted(glob.glob(os.path.join(sim_dir, out_dir, "B*.TLG")))[0]
    log = tlg.read(log_path)
    rows = tlg.accel_rows(log)
    tmp = tempfile.mkdtemp(prefix="trygons_ax_")
    csv_path = os.path.join(tmp, "out.csv")
    subprocess.run([PY, os.path.join(HERE, "log2csv.py"), log_path, "-o", csv_path, "--accel"], check=True,
                   stdout=subprocess.DEVNULL)
    crow = list(csv.DictReader(open(os.path.join(tmp, "out_accel.csv"), encoding="utf-8")))
    texts = [x[1] for x in log.events if x[1].startswith("ACCEL")]
    helm = log.event_text(17, [3, 5, 0, 3])
    listed = log.events_def.get(31, ("",))[0] == "ACCEL" and log.events_def.get(17, ("",))[0] == "HELM"
    sane = bool(rows) and all(90 <= r[7] <= 110 for r in rows[1:]) and all(abs(v) < 20.0 for r in rows for v in r[1:7])
    moving = max(r[1] for r in rows) > 0.02 if rows else False          # the AUTO start: longitudinal + average
    report("AP4 log event 31 ACCEL once a second: tlg.accel_rows + log2csv --accel; HELM still named",
           listed and len(rows) >= 250 and len(crow) == len(rows) and sane and moving and texts and
           "handed over" in helm and set(crow[0]) == set(["t_ms"] + tlg.ACCEL_COLS),
           "%d ACCEL events (%d CSV rows), e.g. %r; HELM event text %r" % (len(rows), len(crow),
                                                                         texts[len(texts) // 2] if texts else None,
                                                                         helm))

    print("\n%d / %d passed" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
