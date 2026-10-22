"""Depth charts, sea marks and the map layers (section B) - no internet needed.

    python test_charts.py        (exit code 0 = all passed)

  B1 chart files: write / read, depth = shallowest cell around a point, no data = None
  B2 depth contours where they belong (a round shoal: the 2 m line on its 2 m edge)
  B3 chartprep: EMODnet CSV + GEBCO gap fill + OpenSeaMap marks (local fake servers)
  B4 map layers: shading, contours, sea marks, AIS, avoidance route, the "Not for
     navigation" note and the attribution - drawn with the network blocked
  B5 the real Attica chart (if prepared with tools/chartprep.py): known depths
"""
import http.server
import json
import math
import os
import shutil
import socket
import sys
import tempfile
import threading
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import chart  # noqa: E402
import mapview  # noqa: E402

results = []
LAT0, LON0, D = 37.65, 23.85, 0.001          # synthetic area: 0.001 deg grid (~111 m N-S)


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def shoal_depth(lat, lon):
    """30 m sea, a round shoal (radius 1 km, 1 m at the centre), an island (land) to the east."""
    r = chart.dist_m(lat, lon, LAT0 + 0.02, LON0 + 0.02)
    if chart.dist_m(lat, lon, LAT0 + 0.02, LON0 + 0.045) < 400:
        return -5.0                                     # land
    return min(30.0, 1.0 + 29.0 * (r / 1000.0)) if r < 1000 else 30.0


def make_area(folder):
    rows = [[shoal_depth(LAT0 + i * D, LON0 + j * D) for j in range(60)] for i in range(40)]
    rows[39][59] = None                                 # one cell without data
    marks = [{"lat": LAT0 + 0.02, "lon": LON0 + 0.02, "type": "buoy_cardinal", "name": "SHOAL N", "colour": "black"}]
    chart.write_area(folder, "test", LAT0, LON0, D, D, rows, ["synthetic"], "Synthetic test data. NOT FOR NAVIGATION",
                     seamarks=marks)


def main():
    tmp = tempfile.mkdtemp(prefix="charts_test_")
    try:
        make_area(os.path.join(tmp, "test"))
        C = chart.Charts(tmp)

        # B1 files, depth query (shallowest of the 4 cells), no data, outside
        a = C.areas[0]
        p_deep = C.depth(LAT0 + 0.005, LON0 + 0.005)
        p_shoal = C.depth(LAT0 + 0.02, LON0 + 0.02)
        p_land = C.depth(LAT0 + 0.02, LON0 + 0.045)
        p_nodata = C.depth(LAT0 + 39 * D - 0.0003, LON0 + 59 * D - 0.0003)
        p_out = C.depth(LAT0 - 1, LON0)
        # between a 30 m cell and a shallower one: the shallower wins
        i, j = 20, 29
        la, lo = LAT0 + (i + 0.5) * D, LON0 + (j + 0.5) * D
        cells = [a.cell(i + di, j + dj) for di in (0, 1) for dj in (0, 1)]
        report("B1 chart files: depth = the SHALLOWEST cell around a point; land / no data / outside = None",
               p_deep == 30.0 and p_shoal is not None and p_shoal < 2.0 and p_land is not None and p_land < 0 and
               p_nodata is None and p_out is None and C.depth(la, lo) == min(cells) and a.nx == 60 and a.ny == 40,
               "deep %.1f m, shoal centre %.1f m, island %.1f m (land < 0), no-data corner %s, outside %s, "
               "between cells %.1f (cells %s)" % (p_deep, p_shoal, p_land, p_nodata, p_out, C.depth(la, lo),
                                                   [round(v, 1) for v in cells]))

        # B2 the 2 m and 10 m contours of the round shoal at the right distance from its centre
        errs = {}
        for lv in (2, 10):
            want = (lv - 1.0) / 29.0 * 1000.0
            dists = [chart.dist_m(p[0], p[1], LAT0 + 0.02, LON0 + 0.02) for ln in a.contours()[str(lv)] for p in ln
                     if chart.dist_m(p[0], p[1], LAT0 + 0.02, LON0 + 0.02) < 1500]
            errs[lv] = (round(want), round(min(dists)), round(max(dists))) if dists else None
        report("B2 depth contours on the right depth (round shoal: 2 m line ~34 m, 10 m line ~310 m from its centre)",
               all(e and abs(e[1] - e[0]) < 90 and abs(e[2] - e[0]) < 90 for e in errs.values()),
               "contour: (expected m, nearest m, farthest m) %s; within one grid cell (111 m)" % errs)

        # B3 chartprep against fake EMODnet / Overpass servers + a GEBCO grid for the gaps
        import chartprep
        grid_lat = [LAT0 + i * D for i in range(10)]
        grid_lon = [LON0 + j * D for j in range(12)]

        class Fake(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                if "erddap" in self.path:
                    lines = ["latitude,longitude,elevation", "degrees_north,degrees_east,m"]
                    for la in grid_lat:
                        for lo in grid_lon:
                            gap = abs(la - grid_lat[3]) < 1e-9 and abs(lo - grid_lon[4]) < 1e-9
                            lines.append("%.7f,%.7f,%s" % (la, lo, "NaN" if gap else "%.2f" % -shoal_depth(la, lo)))
                    body = "\n".join(lines).encode()
                else:
                    body = json.dumps({"elements": [{"type": "node", "lat": LAT0 + 0.003, "lon": LON0 + 0.004,
                                                     "tags": {"seamark:type": "light", "seamark:name": "TEST LIGHT",
                                                              "seamark:light:character": "Fl"}}]}).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *a):
                pass
        srv = http.server.HTTPServer(("127.0.0.1", 0), Fake)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        port = srv.server_address[1]
        chartprep.ERDDAP = ("http://127.0.0.1:%d/erddap?{lat0},{lat1},{lon0},{lon1},{s}" % port)
        chartprep.OVERPASS = ["http://127.0.0.1:%d/overpass" % port]
        asc = os.path.join(tmp, "gebco.asc")
        with open(asc, "w") as f:                       # GEBCO: 12.3 m everywhere (fills only the gap)
            f.write("ncols 20\nnrows 20\nxllcorner %f\nyllcorner %f\ncellsize 0.001\nNODATA_value -9999\n" % (
                LON0 - 0.004, LAT0 - 0.004))
            for _ in range(20):
                f.write(" ".join(["-12.3"] * 20) + "\n")
        out = os.path.join(tmp, "prep")
        chartprep.build("fake", {"lat": [LAT0, LAT0 + 0.009], "lon": [LON0, LON0 + 0.011], "stride": 1}, out,
                        gebco=asc)
        srv.shutdown()
        P = chart.Charts(out)
        pa = P.areas[0] if P.areas else None
        gap_v = pa.cell(3, 4) if pa else None
        report("B3 chartprep: EMODnet depth grid + GEBCO for its gaps + OpenSeaMap marks -> offline files",
               pa is not None and pa.nx == 12 and pa.ny == 10 and gap_v == 12.3 and
               abs(pa.cell(0, 0) - shoal_depth(LAT0, LON0)) < 0.11 and pa.seamarks()[0]["name"] == "TEST LIGHT" and
               "EMODnet" in pa.meta["attribution"] and "GEBCO" in pa.meta["attribution"],
               "grid %sx%s, the EMODnet gap filled from GEBCO: %s m; sea mark %s; attribution: %s" % (
                   pa and pa.nx, pa and pa.ny, gap_v, pa and pa.seamarks()[0]["name"],
                   pa and pa.meta["attribution"][:90]))

        # B4 the map layers with the network BLOCKED (offline)
        real_connect = socket.socket.connect
        tried = []

        def blocked(self, addr):
            tried.append(addr)
            raise OSError("network blocked by the test")
        socket.socket.connect = blocked
        try:
            root = tk.Tk()
            cv = tk.Canvas(root, width=600, height=400, bg="#ffffff")
            cv.pack()
            root.update()
            v = mapview.ChartView(chart.Charts(tmp))
            ais = [{"mmsi": 239000001, "name": "DANGER SHIP", "lat": LAT0 + 0.018, "lon": LON0 + 0.015, "sog": 12,
                    "cog": 90, "stale": False},
                   {"mmsi": 239000002, "name": "OLD", "lat": LAT0 + 0.025, "lon": LON0 + 0.025, "sog": 5, "cog": 0,
                    "stale": True}]
            thr = [{"mmsi": 239000001, "level": "danger", "cpa_nm": 0.1, "tcpa_min": 3.2}]
            v.draw(cv, LAT0 + 0.02, LON0 + 0.02, 8.0, 600, 400, opacity=0.7, ais=ais, threats=thr,
                   route=[(LAT0 + 0.02, LON0 + 0.02), (LAT0 + 0.025, LON0 + 0.03)],
                   shallow={"lat": LAT0 + 0.021, "lon": LON0 + 0.02, "depth": 1.4, "in_s": 20})
            root.update()
            tags = {t: len(cv.find_withtag(t)) for t in ("chart", "contour", "mark", "ais", "route", "shallow", "note")}
            note = [cv.itemcget(i, "text") for i in cv.find_withtag("note") if cv.type(i) == "text"]
            fills = [cv.itemcget(i, "fill") for i in cv.find_withtag("ais") if cv.type(i) == "polygon"]
            root.destroy()
        finally:
            socket.socket.connect = real_connect
        report("B4 map: depth shading, contours, sea marks, AIS (danger red / stale grey), route, shallow, "
               "note + attribution - with the network blocked",
               all(tags[t] > 0 for t in tags) and chart.NOTE in note and any("Synthetic" in n for n in note) and
               "#e00000" in fills and "#909090" in fills and not tried,
               "items %s; note \"%s\"; AIS colours %s; network attempts %d" % (tags, note[0] if note else None, fills,
                                                                              len(tried)))

        # B5 the real Attica chart, if it has been prepared on this PC
        real = chart.Charts()
        if real.area_at(37.82, 23.50):
            known = {"Saronic Gulf (Aegina - Salamina)": (37.82, 23.50, 30, 150), "Kea channel": (37.60, 24.20, 150, 600),
                     "2 km south of Sounion": (37.63, 24.025, 40, 200)}
            got = {k: real.depth(la, lo) for k, (la, lo, _a, _b) in known.items()}
            land = real.depth(37.975, 23.735)
            report("B5 the prepared Attica chart: depths where they should be, Athens is land / no data",
                   all(got[k] is not None and a_ <= got[k] <= b_ for k, (_l, _o, a_, b_) in known.items()) and
                   land is None,
                   "%s; Athens centre: %s" % ({k: v for k, v in got.items()}, land))
        else:
            print("[N/A ] B5 the real Attica chart - not prepared on this PC (python tools/chartprep.py)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    bad = results.count(False)
    print("CHART TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
