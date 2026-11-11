"""The per-drone flight log (CLAUDE.md V "Sim / academy / docs": "Per-drone flight log (exportable)"; step U4).

  <data>/uav_logs/<drone name>/<YYYYMMDD-HHMMSS>.csv      one file per flight (from the ARM), UTC, 1 line a second:
      utc, slot, sysid, lat, lon, alt_m, hdg, gs_ms, climb_ms, batt_pct, batt_v, batt_a, used_mah, mode, armed, lq_pct,
      land_eta_s, landing, phase
Written by a helper thread (a queue; never file work on the gateway's event loop); a full queue drops + counts.
Export: CSV as it is, or GPX 1.1 (a track with the altitude + time) - station messages uav_logs (the list) and
uav_log {name, file, fmt} (the text, at most MAX_BYTES), answered from a helper thread too.
"""
import base64
import csv
import io
import os
import queue
import re
import threading
import time
from xml.sax.saxutils import escape

from . import tlog

FIELDS = ("utc", "slot", "sysid", "lat", "lon", "alt_m", "hdg", "gs_ms", "climb_ms", "batt_pct", "batt_v", "batt_a",
          "used_mah", "mode", "armed", "lq_pct", "land_eta_s", "landing", "phase")
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
FILE_RE = re.compile(r"^\d{8}-\d{6}(-\d+)?\.csv$")
MAX_BYTES = 4 * 1024 * 1024
MAX_TLOG_BYTES = 6 * 1024 * 1024               # a .tlog sent to a station (base64 in JSON); bigger: copy it by hand


def safe_name(text, slot):
    s = re.sub(r"[^A-Za-z0-9_-]+", "-", str(text or "")).strip("-")[:40]
    return s or "UAV%d" % int(slot)


def utc(t):
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))


class FlightLogs:
    def __init__(self, folder):
        self.folder = folder                       # None = no storage: nothing written
        self.q = queue.Queue(maxsize=500)
        self.files = {}                            # slot -> (name, file name) of the flight now
        self.dropped = 0
        self.lines = 0
        self.errors = 0
        self.th = None

    def start(self):
        if self.folder and self.th is None:
            self.th = threading.Thread(target=self._run, daemon=True, name="uav-flightlog")
            self.th.start()

    def stop(self):
        if self.th is not None:
            try:
                self.q.put_nowait(None)
            except queue.Full:
                pass

    def record(self, slot, name, row, new_flight=False, now=None):
        """One line for slot's drone (called 1 x a second while it flies). new_flight = a new file (at the ARM)."""
        if not self.folder:
            return
        now = time.time() if now is None else now
        nm = safe_name(name, slot)
        if new_flight or slot not in self.files or self.files[slot][0] != nm:
            self.files[slot] = (nm, time.strftime("%Y%m%d-%H%M%S", time.gmtime(now)) + ".csv")
        r = dict(row, utc=utc(now), slot=slot)
        try:
            self.q.put_nowait((self.files[slot][0], self.files[slot][1], r))
        except queue.Full:
            self.dropped += 1

    def end(self, slot):
        """The flight of slot ended (disarmed): the next record starts a new file."""
        self.files.pop(slot, None)

    def _run(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            nm, fn, r = item
            try:
                d = os.path.join(self.folder, nm)
                os.makedirs(d, exist_ok=True)
                p = os.path.join(d, fn)
                new = not os.path.exists(p)
                with open(p, "a", newline="", encoding="utf-8") as f:
                    w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
                    if new:
                        w.writeheader()
                    w.writerow({k: ("" if r.get(k) is None else r.get(k)) for k in FIELDS})
                self.lines += 1
            except OSError:
                self.errors += 1

    # --- export ---------------------------------------------------------------------------------------------------
    def list(self):
        out = []
        if not self.folder or not os.path.isdir(self.folder):
            return out
        for nm in sorted(os.listdir(self.folder)):
            d = os.path.join(self.folder, nm)
            if not (NAME_RE.match(nm) and os.path.isdir(d)):
                continue
            for fn in sorted(os.listdir(d)):
                if FILE_RE.match(fn):
                    p = os.path.join(d, fn)
                    try:
                        out.append({"name": nm, "file": fn, "bytes": os.path.getsize(p), "t": os.path.getmtime(p),
                                    "tlog": sum(os.path.getsize(q) for q in tlog.parts_of(self.folder, nm, fn[:15]))})
                    except OSError:
                        pass
        return out

    def path(self, name, file):
        if not self.folder or not NAME_RE.match(str(name or "")) or not FILE_RE.match(str(file or "")):
            return None
        return os.path.join(self.folder, name, file)

    def read(self, name, file, fmt="csv"):
        """-> (ok, text or why)."""
        p = self.path(name, file)
        if p is None:
            return False, "no such flight log"
        if fmt == "tlog":
            parts = tlog.parts_of(self.folder, name, file[:15])
            if not parts:
                return False, "no .tlog for this flight"
            try:
                if sum(os.path.getsize(q) for q in parts) > MAX_TLOG_BYTES:
                    return False, "telemetry log too big to send: copy it from the gateway PC"
                data = b"".join(open(q, "rb").read() for q in parts)
            except OSError as e:
                return False, "telemetry log not readable: %s" % e
            return True, base64.b64encode(data).decode("ascii")
        try:
            if os.path.getsize(p) > MAX_BYTES:
                return False, "flight log too big to send (%d bytes): copy it from the gateway PC" % os.path.getsize(p)
            with open(p, encoding="utf-8", newline="") as f:
                text = f.read()
        except OSError as e:
            return False, "flight log not readable: %s" % e
        if fmt == "gpx":
            return True, to_gpx(text, "%s %s" % (name, file[:-4]))
        if fmt != "csv":
            return False, "format csv, gpx or tlog"
        return True, text


def to_gpx(csv_text, title):
    """A GPX 1.1 track (lat / lon / ele / time) of a flight log's CSV text."""
    pts = []
    for r in csv.DictReader(io.StringIO(csv_text)):
        try:
            la, lo = float(r["lat"]), float(r["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (-90 <= la <= 90 and -180 <= lo <= 180) or (la == 0 and lo == 0):
            continue
        ele = r.get("alt_m") or ""
        pts.append('   <trkpt lat="%.7f" lon="%.7f">%s<time>%s</time></trkpt>' % (
            la, lo, "<ele>%s</ele>" % escape(ele) if ele else "", escape(r.get("utc") or "")))
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<gpx version="1.1" creator="Trygons C2 gateway (UAV flight log)" xmlns="http://www.topografix.com/GPX/1/1">\n'
            ' <trk><name>%s</name><trkseg>\n%s\n </trkseg></trk>\n</gpx>\n' % (escape(title), "\n".join(pts)))
