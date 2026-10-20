"""Convert a .TLG log (boat SD card or ground-station copy) to CSV.

    python log2csv.py B00012_001.TLG                       -> B00012_001.csv (all fields)
    python log2csv.py B00012_001.TLG -f heading,target,thr_out
    python log2csv.py B00012_001.TLG --events              -> B00012_001_events.csv as well
    python log2csv.py B00012_001.TLG -o out.csv

Opens in Excel / LibreOffice. Values are in real units (degrees, %, kn, m),
the first column is the boat time in ms. Field names: see the file header
or run with --list.
"""
import argparse
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tlg  # noqa: E402


def fmt(v):
    if isinstance(v, float):
        return ("%.7f" % v).rstrip("0").rstrip(".") if v != int(v) else str(int(v))
    return str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("-f", "--fields", help="comma-separated field names (default: all)")
    ap.add_argument("-o", "--out", help="output CSV (default: next to the log)")
    ap.add_argument("--events", action="store_true", help="also write the events to <name>_events.csv")
    ap.add_argument("--list", action="store_true", help="list the field names and stop")
    a = ap.parse_args()

    log = tlg.read(a.log)
    names = [f.name for f in log.fields]
    if a.list:
        for f in log.fields:
            print("%-16s %s  x%g" % (f.name, f.type, f.scale))
        return 0
    cols = a.fields.split(",") if a.fields else names
    bad = [c for c in cols if c not in names]
    if bad:
        print("unknown field(s): %s (use --list)" % ", ".join(bad))
        return 1
    out = a.out or os.path.splitext(a.log)[0] + ".csv"
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t_ms"] + cols)
        for s in log.samples:
            w.writerow([s["t_ms"]] + [fmt(s[c]) for c in cols])
    print("%s: %d samples x %d fields%s" % (out, len(log.samples), len(cols),
                                             ", %d CRC errors skipped" % log.crc_errors if log.crc_errors else ""))
    if a.events:
        eout = os.path.splitext(out)[0] + "_events.csv"
        with open(eout, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_ms", "event"])
            for t, text, _c, _a in log.events:
                w.writerow([t, text])
            for t, ftype, seq, res, p in log.pc_commands:
                w.writerow([t, "PC %s seq %d -> %s payload %s" % (
                    tlg.proto.TYPE_NAMES.get(ftype, hex(ftype)), seq, tlg.proto.RESULTS.get(res, res), p.hex())])
        print("%s: %d events" % (eout, len(log.events) + len(log.pc_commands)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
