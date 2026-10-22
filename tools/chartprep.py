"""Make the offline depth charts once (section B), on a PC with internet.

    python tools/chartprep.py                    the areas in charts/areas.json
    python tools/chartprep.py --area attica      only one area
    python tools/chartprep.py --gebco gebco.asc  fill gaps from a GEBCO grid
    python tools/chartprep.py --seamarks marks.json   OpenSeaMap data saved before

Sources (free; their licences require the attribution shown on the map):
  * depth: EMODnet Bathymetry DTM 2024 (~115 m grid, depth below LAT), read
    as CSV from the EMODnet ERDDAP server (no GIS software needed).
    https://emodnet.ec.europa.eu/en/bathymetry - CC BY 4.0
  * gaps (no EMODnet value): GEBCO grid, if you give one: on
    https://download.gebco.net choose the area, format "Esri ASCII", and
    pass the .asc file with --gebco. GEBCO Compilation Group, public domain
    with attribution.
  * sea marks (buoys, beacons, lights, harbours): OpenSeaMap = OpenStreetMap
    seamark tags, via the Overpass API. (c) OpenStreetMap contributors, ODbL.
Result: charts/<area>/ (see groundstation/chart.py). Copy that folder to the
shore PC's and the Pi's charts/ folder (gateway README section 17). The
map then works with NO internet.

NOT FOR NAVIGATION - free data, may be inaccurate near the coast.
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "groundstation"))
import chart  # noqa: E402

UA = "trygons-chartprep/1.0 (small boat project; one-time download)"
ERDDAP = "https://erddap.emodnet.eu/erddap/griddap/bathymetry_dtm_2024.csv?elevation%5B({lat0}):{s}:({lat1})%5D%5B({lon0}):{s}:({lon1})%5D"
OVERPASS = ["https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter",
            "https://maps.mail.ru/osm/tools/overpass/api/interpreter"]
ATTRIBUTION = ("Depth: EMODnet Bathymetry Consortium (2024) EMODnet Digital Bathymetry (DTM), CC BY 4.0%s. "
               "Sea marks: (c) OpenStreetMap contributors / OpenSeaMap, ODbL. "
               "NOT FOR NAVIGATION - free data, may be inaccurate near the coast.")
DEFAULT_AREAS = {
    # Saronic Gulf, Attica coast around Sounion, South Euboean Gulf, Kea / Makronisos, with a margin
    "attica": {"lat": [37.30, 38.35], "lon": [22.95, 24.65], "stride": 1},
}


def get(url, tries=4, timeout=120):
    wait = 5.0
    for k in range(tries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=timeout) as f:
                return f.read().decode("utf-8", "replace")
        except Exception as e:
            if k == tries - 1:
                raise
            print("   retry in %.0f s (%s)" % (wait, str(e)[:80]))
            time.sleep(wait)
            wait *= 2


def emodnet(lat0, lat1, lon0, lon1, stride=1, tile=0.25):
    """{(lat, lon): depth m (+ water)} from the EMODnet ERDDAP server, in tiles."""
    out = {}
    lats = []
    la = lat0
    while la < lat1 - 1e-9:
        lats.append((la, min(lat1, la + tile)))
        la += tile
    los = []
    lo = lon0
    while lo < lon1 - 1e-9:
        los.append((lo, min(lon1, lo + tile)))
        lo += tile
    n = len(lats) * len(los)
    k = 0
    for a, b in lats:
        for c, d in los:
            k += 1
            print("  EMODnet tile %d/%d  %.2f..%.2f N  %.2f..%.2f E" % (k, n, a, b, c, d))
            txt = get(ERDDAP.format(lat0=a, lat1=b, lon0=c, lon1=d, s=stride))
            for line in txt.splitlines()[2:]:
                p = line.split(",")
                if len(p) != 3:
                    continue
                try:
                    lat, lon = round(float(p[0]), 7), round(float(p[1]), 7)
                except ValueError:
                    continue
                out[(lat, lon)] = None if p[2] in ("NaN", "") else -float(p[2])    # elevation -> depth
    return out


def read_asc(path):
    """Esri ASCII grid (GEBCO download) -> function (lat, lon) -> depth or None."""
    hdr = {}
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            p = line.split()
            if len(p) == 2 and p[0].lower() in ("ncols", "nrows", "xllcorner", "yllcorner", "xllcenter",
                                                 "yllcenter", "cellsize", "nodata_value"):
                hdr[p[0].lower()] = float(p[1])
            elif p:
                rows.append([float(v) for v in p])
    nc, nr, cs = int(hdr["ncols"]), int(hdr["nrows"]), hdr["cellsize"]
    x0 = hdr.get("xllcorner", hdr.get("xllcenter", 0) - cs / 2)
    y0 = hdr.get("yllcorner", hdr.get("yllcenter", 0) - cs / 2)
    nod = hdr.get("nodata_value", -9999)

    def at(lat, lon):
        j = int((lon - x0) / cs)
        i = nr - 1 - int((lat - y0) / cs)              # first row = north
        if not (0 <= i < nr and 0 <= j < nc) or rows[i][j] == nod:
            return None
        return -rows[i][j]
    return at


def seamarks(lat0, lat1, lon0, lon1, saved=None):
    if saved:
        data = json.load(open(saved, encoding="utf-8"))
    else:
        q = '[out:json][timeout:120];(node["seamark:type"](%f,%f,%f,%f););out body;' % (lat0, lon0, lat1, lon1)
        data = None
        for url in OVERPASS:
            try:
                print("  OpenSeaMap sea marks from %s" % url)
                data = json.loads(get(url + "?" + urllib.parse.urlencode({"data": q}), tries=3))
                break
            except Exception as e:
                print("   failed: %s" % str(e)[:80])
        if data is None:
            print("  NO sea marks (Overpass busy) - run again later, or save the Overpass answer and use --seamarks")
            return []
    out = []
    for el in data.get("elements", []):
        t = el.get("tags", {})
        if "lat" not in el:
            continue
        out.append({"lat": el["lat"], "lon": el["lon"], "type": t.get("seamark:type", ""),
                    "name": t.get("seamark:name") or t.get("name", ""),
                    "colour": t.get("seamark:light:colour") or t.get("seamark:buoy_lateral:colour", ""),
                    "light": t.get("seamark:light:character", ""), "category": next(
                        (v for k, v in t.items() if k.endswith(":category")), "")})
    return out


def build(name, cfg, out_dir, gebco=None, saved_marks=None):
    lat0, lat1 = cfg["lat"]
    lon0, lon1 = cfg["lon"]
    print("Area %s: %.2f..%.2f N, %.2f..%.2f E" % (name, lat0, lat1, lon0, lon1))
    pts = emodnet(lat0, lat1, lon0, lon1, cfg.get("stride", 1))
    lats = sorted({k[0] for k in pts})
    lons = sorted({k[1] for k in pts})
    dlat = (lats[-1] - lats[0]) / max(1, len(lats) - 1)
    dlon = (lons[-1] - lons[0]) / max(1, len(lons) - 1)
    fill = read_asc(gebco) if gebco else None
    rows, filled = [], 0
    for la in lats:
        r = []
        for lo in lons:
            v = pts.get((la, lo))
            if v is None and fill:
                v = fill(la, lo)
                filled += v is not None
            r.append(v)
        rows.append(r)
    marks = seamarks(lat0, lat1, lon0, lon1, saved_marks)
    srcs = ["EMODnet Bathymetry DTM 2024 (erddap.emodnet.eu bathymetry_dtm_2024)"]
    if gebco:
        srcs.append("GEBCO grid %s (%d gap cells filled)" % (os.path.basename(gebco), filled))
    srcs.append("OpenSeaMap / OpenStreetMap seamark:* (%d marks)" % len(marks))
    meta = chart.write_area(os.path.join(out_dir, name), name, lats[0], lons[0], dlat, dlon, rows, srcs,
                            ATTRIBUTION % ("; GEBCO Compilation Group" if gebco else ""), seamarks=marks)
    print("  -> %s: %d x %d cells (%.0f m), %d sea marks" % (os.path.join(out_dir, name), meta["nx"], meta["ny"],
                                                             dlat * 111320, len(marks)))


def main():
    ap = argparse.ArgumentParser(description="Make the offline depth charts")
    ap.add_argument("--area")
    ap.add_argument("--out", default=chart.DEFAULT_DIR)
    ap.add_argument("--gebco", help="GEBCO Esri ASCII grid to fill gaps")
    ap.add_argument("--seamarks", help="saved Overpass JSON answer")
    a = ap.parse_args()
    areas_file = os.path.join(a.out, "areas.json")
    areas = DEFAULT_AREAS
    if os.path.exists(areas_file):
        areas = json.load(open(areas_file, encoding="utf-8"))
    else:
        os.makedirs(a.out, exist_ok=True)
        json.dump(DEFAULT_AREAS, open(areas_file, "w", encoding="utf-8"), indent=1)
        print("wrote %s (add more areas there)" % areas_file)
    for name, cfg in areas.items():
        if a.area and name != a.area:
            continue
        build(name, cfg, a.out, a.gebco, a.seamarks)
    return 0


if __name__ == "__main__":
    sys.exit(main())
