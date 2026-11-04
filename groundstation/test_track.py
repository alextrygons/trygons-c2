"""The track on this PC (step 13n3, owner 2026-10-31 E3) - groundstation/track.py, no window needed.

  TR1 recording: no point limit (20 000 points kept), at most one point per second, only after MIN_M moved (a slow
      drift after SLOW_S), bad numbers (NaN, out of range, no position) never recorded
  TR2 crash-safe: every point is on the disk at once; a restart restores the track; a cut last line (power loss) is
      dropped and the next point is not glued onto it; a corrupt line is skipped
  TR3 one live track PER BOAT: switching boats shows that boat's own track (never mixed), back again = the old one
  TR4 two stations of one PC on the same boat: one writer (the lock), the other keeps its track in RAM only
  TR5 NEW TRACK: saved as <date>_<HHMM>_<boat> (the first point's time), a new empty line starts (also after a
      restart), the same minute twice = _2; nothing to save = no file; a failing save keeps the live track
  TR6 the list: date, boat, duration, nm, newest first; a file that is not a track is skipped
  TR7 drawing: simplified (a 20 000-point track draws as ~1 000 points), cut to the window, the last point
      always drawn; a click on the line = the interpolated time + speed there, a click away = nothing
  TR8 GPX 1.1 export + import round trip (times, speeds, heading); import of other programs' GPX (GPX 1.0 speed in
      m/s, no times, several segments); refused: not XML, a DTD / entity, no points, too big; never a crash
  TR9 a disk that fails while recording: the track goes on in RAM, the error is reported (once), nothing raises
  TR10 (review 13n3 MED-B) 200 000 points at 0.5 m/px drawn / clicked within 50 ms, a draw budget, the live RAM
       bounded while the file + NEW TRACK keep every point
  TR11 (review 13n3 L1 / L4 / L5) never two boats in one file, a clock stepped back, times after 2100
  TR12 (review 13n3 L7) the temporary folder only with TRYGONS_TEST

Run:  python test_track.py        (exit code 0 = all passed)
"""
import math
import os
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["TRYGONS_TRACKS"] = tempfile.mkdtemp(prefix="trygons_tr_")
import track  # noqa: E402

results = []
T0 = 1793000000.0          # 2026-10-26 (a fixed time: file names are reproducible)


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def fresh():
    d = tempfile.mkdtemp(prefix="trygons_tr_")
    return d, track.TrackStore(d)


def walk(lv, n, t0=T0, lat=37.8, lon=23.6, step_m=5.0, dt=1.0, kn=6.0):
    for i in range(n):
        lv.feed(t0 + i * dt, lat + i * step_m / 111320.0, lon, kn, 0.0)


def tr1():
    d, st = fresh()
    lv = st.select("boat_A", "A")
    walk(lv, 20000)
    many = len(lv.track) == 20000
    n0 = len(lv.track)
    lv.feed(T0 + 20000 - 0.5, 37.9, 23.6, 6, 0)            # < 1 s after the last one
    lv.feed(T0 + 20001, lv.last()[1] + 0.5 / 111320.0, 23.6, 6, 0)   # 1 s later but 0.5 m only
    lv.feed(T0 + 20012, lv.last()[1] + 0.5 / 111320.0, 23.6, 0.1, 0)   # 12 s later, 0.5 m: a slow drift point
    rate = len(lv.track) == n0 + 1
    bad = 0
    for p in ((T0 + 30000, float("nan"), 23.6, 1, 0), (T0 + 30001, 95.0, 23.6, 1, 0), (T0 + 30002, 37.0, 200.0, 1, 0),
              (T0 + 30003, 37.0, float("inf"), 1, 0), (None, 37.0, 23.0, 1, 0)):
        bad += bool(lv.feed(*p))
    nopos = not st.feed_telem({"pos_source": 0, "est_lat": 38.0, "est_lon": 24.0, "heading": 0}, T0 + 40000)
    badkn = lv.feed(T0 + 40001, 38.0, 24.0, float("nan"), 400.0) and lv.last()[3] is None and lv.last()[4] is None
    lv.close()
    shutil.rmtree(d, True)
    report("TR1 recording: no point limit, <= 1 point/s, only after it moved (slow drift after 10 s), bad numbers "
           "never recorded", many and rate and bad == 0 and nopos and badkn,
           "20000 kept %s; rate %s; bad recorded %d; no position %s; bad speed/heading dropped %s" % (
               many, rate, bad, nopos, badkn))


def tr2():
    d, st = fresh()
    lv = st.select("boat_A", "A")
    walk(lv, 50)
    on_disk = len(track.read_file(lv.path)) == 50              # written at once (flushed), before any close
    path = lv.path
    st.close()
    with open(path, "ab") as f:                                 # a power loss in the middle of a line
        f.write(b'[1793000100.0, 37.80')
    with open(path, "rb") as f:
        data = f.read()
    lines = data.split(b"\n")
    lines.insert(10, b"{not json")                              # a corrupt line in the middle
    with open(path, "wb") as f:
        f.write(b"\n".join(lines))
    st2 = track.TrackStore(d)
    lv2 = st2.select("boat_A", "A")
    restored = len(lv2.track) == 50
    lv2.feed(T0 + 200, 37.9, 23.6, 5, 10)
    st2.close()
    back = track.read_file(path)
    not_glued = len(back) == 51 and abs(back.pts[-1][1] - 37.9) < 1e-9
    shutil.rmtree(d, True)
    report("TR2 crash-safe: on the disk at once, restored at the next start, a cut last line dropped (the next point "
           "not glued onto it), a corrupt line skipped", on_disk and restored and not_glued,
           "on disk %s; restored %s; next point clean %s (%d)" % (on_disk, restored, not_glued, len(back)))


def tr3():
    d, st = fresh()
    a = st.select("boat_A", "A")
    walk(a, 30)
    b = st.select("boat_B", "B")
    other = len(st.latlon()) == 0
    walk(b, 10, lat=36.0)
    only_b = len(st.latlon()) == 10 and all(abs(la - 36.0) < 0.01 for la, _ in st.latlon())
    st.select("boat_A", "A")
    back_a = len(st.latlon()) == 30 and all(abs(la - 37.8) < 0.01 for la, _ in st.latlon())
    st.close()
    files = sorted(os.listdir(os.path.join(d, "live")))
    shutil.rmtree(d, True)
    report("TR3 one live track per boat: another boat = its own track (never mixed), back = the old one",
           other and only_b and back_a, "B empty %s; only B %s; A back %s; files %s" % (other, only_b, back_a, files))


def tr4():
    d = tempfile.mkdtemp(prefix="trygons_tr_")
    s1, s2 = track.TrackStore(d), track.TrackStore(d)
    a = s1.select("boat_A", "A")
    b = s2.select("boat_A", "A")
    one_writer = not a.shared and b.shared
    walk(a, 5)
    walk(b, 5, lat=10.0)
    on_disk = track.read_file(a.path)
    clean = len(on_disk) == 5 and all(abs(p[1] - 37.8) < 0.01 for p in on_disk.pts)
    ram = len(b.track) == 5
    s1.close()
    s3 = track.TrackStore(d)
    c = s3.select("boat_A", "A")                    # the writer closed: the next one takes over
    takes = not c.shared
    s2.close()
    s3.close()
    shutil.rmtree(d, True)
    report("TR4 two stations of one PC, same boat: one writer (lock), the other RAM only; the lock is freed on close",
           one_writer and clean and ram and takes, "one writer %s; file clean %s; RAM %s; next takes over %s" % (
               one_writer, clean, ram, takes))


def tr5():
    d, st = fresh()
    none = st.select("boat_A", "TRYGON-1") and st.new_track("TRYGON-1") is None
    walk(st.live, 40)
    p1 = st.new_track("TRYGON-1")
    name_ok = p1 and os.path.basename(p1) == time.strftime("%Y-%m-%d_%H%M", time.localtime(T0)) + "_TRYGON-1.jsonl"
    saved = p1 and len(track.read_file(p1)) == 40 and track.read_file(p1).boat == "TRYGON-1"
    new_empty = len(st.live.track) == 0
    walk(st.live, 5)
    p2 = st.new_track("TRYGON-1")
    second = p2 and p2.endswith("_TRYGON-1_2.jsonl")
    walk(st.live, 3, t0=T0 + 3600)
    st.close()
    st2 = track.TrackStore(d)
    lv = st2.select("boat_A", "TRYGON-1")
    after_restart = len(lv.track) == 3
    # a save that can't be written keeps the live track as it was
    bad = os.path.join(d, "saved")
    real = track.TrackStore.saved_dir
    track.TrackStore.saved_dir = property(lambda self: os.path.join(bad, "x\0y"))
    try:
        try:
            st2.new_track("TRYGON-1")
            raised = False
        except (OSError, ValueError):
            raised = True
    finally:
        track.TrackStore.saved_dir = real
    kept = len(lv.track) == 3
    st2.close()
    shutil.rmtree(d, True)
    report("TR5 NEW TRACK: <date>_<HHMM>_<boat>, a new empty line, _2 for the same minute, nothing to save = no file, "
           "a failed save keeps the live track", none and name_ok and saved and new_empty and second and
           after_restart and raised and kept,
           "nothing %s; name %s (%s); saved %s; new empty %s; _2 %s; after restart %s; failed save raised %s kept %s" % (
               none, name_ok, p1 and os.path.basename(p1), saved, new_empty, second, after_restart, raised, kept))


def tr6():
    d, st = fresh()
    st.select("boat_A", "ALPHA")
    walk(st.live, 61, dt=60.0, step_m=1852.0 / 60)         # 1 h, 1 nm
    st.new_track("ALPHA")
    st.select("boat_B", "BETA")
    walk(st.live, 10, t0=T0 + 86400)
    st.new_track("BETA")
    with open(os.path.join(st.saved_dir, "junk.jsonl"), "w") as f:
        f.write("hello\n")
    rows = st.saved_list()
    real = [r for r in rows if r["n"]]
    order = [r["boat"] for r in real] == ["BETA", "ALPHA"]
    a = [r for r in real if r["boat"] == "ALPHA"][0]
    stats = abs(a["duration_s"] - 3600) < 1 and abs(a["nm"] - 1.0) < 0.01
    junk = all(r["n"] == 0 for r in rows if r["file"] == "junk")
    st.close()
    shutil.rmtree(d, True)
    report("TR6 the list: date, boat, duration, nm, newest first; a non-track file shows no points",
           order and stats and junk, "order %s; ALPHA %.0f s %.3f nm; junk %s" % (
               [r["boat"] for r in real], a["duration_s"], a["nm"], junk))


def tr7():
    tr = track.Track()
    for i in range(20000):                                  # a 20 000-point zig-zag ~ 100 km
        tr.add((T0 + i, 37.8 + i * 5 / 111320.0, 23.6 + (0.00002 if i % 2 else 0.0), 4.0 + i / 10000.0, 0.0))
    m = 50.0                                                # 50 m per pixel
    lat0, lon0 = tr.latlon[-1]
    k = math.cos(math.radians(lat0)) * 111320.0
    w, h = 1600, 900
    P = lambda la, lo: (w / 2 + (lo - lon0) * k / m, h / 2 - (la - lat0) * 111320.0 / m)   # noqa: E731
    t = time.perf_counter()
    runs = tr.runs(P, w, h, m)
    t_ms = (time.perf_counter() - t) * 1000.0
    n_draw = sum(len(r) for r in runs) // 2
    small = 0 < n_draw < 2500                              # (vs 20 000 points)
    last = runs and abs(runs[-1][-2] - w / 2) < 1 and abs(runs[-1][-1] - h / 2) < 1
    t = time.perf_counter()
    tr.runs(P, w, h, m)
    again_ms = (time.perf_counter() - t) * 1000.0
    # the click: the point 10000 (half way) is ~ 50 km south: zoom so it is on screen
    m2 = 5.0
    la_c, lo_c = tr.latlon[10000]
    P2 = lambda la, lo: (w / 2 + (lo - lo_c) * k / m2, h / 2 - (la - la_c) * 111320.0 / m2)   # noqa: E731
    x, y = P2(37.8 + 10000.5 * 5 / 111320.0, 23.6)
    hit = tr.nearest(P2, x + 3, y, 14, m2)
    hit_ok = hit is not None and abs(hit["t"] - (T0 + 10000.5)) < 1.5 and abs(hit["kn"] - 5.0) < 0.01
    miss = tr.nearest(P2, x + 200, y, 14, m2) is None
    report("TR7 drawing simplified + cut to the window (20 000 points), the last point drawn; a click = the time + "
           "speed there, away = nothing", small and last and hit_ok and miss and t_ms < 1500,
           "drawn %d points in %d runs, %.0f ms (cached %.0f ms); last %s; hit %s; miss %s" % (
               n_draw, len(runs), t_ms, again_ms, bool(last), hit and (round(hit["t"] - T0, 1), round(hit["kn"], 2)),
               miss))


def tr8():
    d, st = fresh()
    st.select("boat_A", "TRYGON-1")
    walk(st.live, 20, kn=7.5)
    gpx = os.path.join(d, "out.gpx")
    track.gpx_write(st.live.track, gpx, name="TRYGON-1 <test>")
    p = st.import_gpx(gpx)
    back = st.load(p)
    a, b = st.live.track.pts, back.pts
    trip = len(a) == len(b) and all(abs(x[0] - y[0]) < 1 and abs(x[1] - y[1]) < 1e-6 and abs(x[3] - y[3]) < 0.01 and
                                    abs(x[4] - y[4]) < 0.1 for x, y in zip(a, b)) and back.boat == "TRYGON-1 <test>" \
        and back.imported
    other = os.path.join(d, "other.gpx")
    with open(other, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0"?><gpx version="1.0" xmlns="http://www.topografix.com/GPX/1/0"><trk><name>Plotter'
                '</name><trkseg><trkpt lat="37.1" lon="23.1"><speed>2.0</speed><course>90</course></trkpt>'
                '<trkpt lat="37.2" lon="23.2"></trkpt></trkseg><trkseg><trkpt lat="37.3" lon="23.3"/>'
                '<trkpt lat="999" lon="23.3"/></trkseg></trk></gpx>')
    o = st.load(st.import_gpx(other))
    other_ok = len(o) == 3 and abs(o.pts[0][3] - 2.0 * 3600 / 1852) < 0.01 and o.pts[0][4] == 90.0 and \
        o.pts[1][0] is None and o.boat == "Plotter"
    refused = []
    for name, body in (("bad.gpx", "not xml at all <<<"),
                       ("dtd.gpx", '<?xml version="1.0"?><!DOCTYPE gpx [<!ENTITY a "aaaa">]><gpx>&a;</gpx>'),
                       ("empty.gpx", '<?xml version="1.0"?><gpx version="1.1"><trk><trkseg/></trk></gpx>')):
        fp = os.path.join(d, name)
        with open(fp, "w", encoding="utf-8") as f:
            f.write(body)
        try:
            st.import_gpx(fp)
            refused.append((name, None))
        except ValueError as e:
            refused.append((name, str(e)))
    # review 13n3 MED-A: the DTD refusal is in the parser - a UTF-16 file or > 64 KB of comments / blanks before
    # the DOCTYPE (a "billion laughs") is refused too, never expanded
    lol = ('<!DOCTYPE gpx [<!ENTITY a "aaaaaaaaaa"><!ENTITY b "&a;&a;&a;&a;&a;&a;&a;&a;&a;&a;">'
           '<!ENTITY c "&b;&b;&b;&b;&b;&b;&b;&b;&b;&b;">]>')
    trk = '<gpx version="1.1"><trk><name>&c;</name><trkseg><trkpt lat="37.1" lon="23.1"/></trkseg></trk></gpx>'
    bypass = []
    for name, data in (("utf16.gpx", ('<?xml version="1.0" encoding="UTF-16"?>' + lol + trk).encode("utf-16")),
                       ("padded.gpx", ('<?xml version="1.0"?>' + "<!-- " + "x" * 70000 + " -->" + "\n" * 2000 +
                                       lol + trk).encode("utf-8")),
                       ("ext.gpx", ('<?xml version="1.0"?><!DOCTYPE gpx SYSTEM "http://example.invalid/x.dtd">' +
                                    trk.replace("&c;", "x")).encode("utf-8"))):
        fp = os.path.join(d, name)
        with open(fp, "wb") as f:
            f.write(data)
        try:
            st.import_gpx(fp)
            bypass.append((name, None))
        except ValueError as e:
            bypass.append((name, str(e)))
    no_part = not [f for f in os.listdir(st.saved_dir) if f.endswith(".part")]
    big = os.path.join(d, "big.gpx")
    with open(big, "wb") as f:
        f.truncate(track.GPX_MAX_BYTES + 1)
    try:
        st.import_gpx(big)
        refused.append(("big", None))
    except ValueError as e:
        refused.append(("big", str(e)))
    codes = [c for _n, c in refused] == ["not_xml", "dtd", "no_points", "too_big"] and \
        [c for _n, c in bypass] == ["dtd", "dtd", "dtd"] and no_part
    st.close()
    shutil.rmtree(d, True)
    report("TR8 GPX 1.1 export + import round trip (times, speeds, heading); other programs' GPX (1.0 speed m/s, no "
           "times, segments, a bad point skipped); refused: not XML, DTD / entity, no points, too big",
           trip and other_ok and codes, "round trip %s; other %s (%s); refused %s; UTF-16 / 70 KB padding / "
           "external DTD %s; no .part left %s" % (
               trip, other_ok, [(round(p[1], 2), p[3] and round(p[3], 2), p[4], p[0]) for p in o.pts], refused,
               bypass, no_part))


def tr9():
    d, st = fresh()
    lv = st.select("boat_A", "A")
    walk(lv, 3)

    class Broken:
        def write(self, s):
            raise OSError(28, "No space left on device")

        def close(self):
            pass
    lv.f = Broken()
    walk(lv, 10, t0=T0 + 100)
    ram = len(lv.track) == 13
    err = lv.error is not None and "space" in lv.error
    st.close()
    on_disk = len(track.read_file(lv.path))
    shutil.rmtree(d, True)
    report("TR9 a disk that fails while recording: the track goes on in RAM, the error is kept for the station, "
           "nothing raises", ram and err and on_disk == 3, "RAM %s; error %r; on disk %d" % (ram, lv.error, on_disk))


def tr10():
    """Review 13n3 MED-B: a 200 000-point track drawn at 0.5 m/px within a time budget (only the visible chunks are
    projected), a click on it answers quickly; the live track's RAM is bounded but the FILE keeps every point and
    NEW TRACK saves all of them; GPX import limited to GPX_MAX_POINTS."""
    tr = track.Track()
    la0, lo0 = 37.8, 23.6
    for i in range(200000):                 # a 400 km zig-zag: 2 m steps
        tr.add((T0 + i, la0 + (i % 2000) * 2.0 / 111320.0, lo0 + (i // 2000) * 0.002, 5.0, 0.0))
    m = 0.5
    cx, cy = la0 + 500 * 2.0 / 111320.0, lo0 + 50 * 0.002
    k = math.cos(math.radians(la0))

    def P(la, lo):
        return ((lo - cy) * k * 111320.0 / m + 800, (cx - la) * 111320.0 / m + 450)
    tr.runs(P, 1600, 900, m)                # the first frame at this zoom builds the simplification once
    t0 = time.perf_counter()
    for _ in range(5):
        runs = tr.runs(P, 1600, 900, m)
    per = (time.perf_counter() - t0) / 5
    drawn = sum(len(r) // 2 for r in runs)
    x, y = P(cx, cy)
    t1 = time.perf_counter()
    hit = tr.nearest(P, x, y, 8, m)
    t_hit = time.perf_counter() - t1
    # far zoom: the whole track in view - at most MAX_DRAW points drawn
    m2 = 400.0

    def P2(la, lo):
        return ((lo - lo0) * k * 111320.0 / m2 + 10, (la0 + 0.02 - la) * 111320.0 / m2 + 10)
    all_pts = sum(len(r) // 2 for r in tr.runs(P2, 1600, 900, m2))
    # live: RAM bounded, the file complete, NEW TRACK = every point
    d, st = fresh()
    old = track.LIVE_RAM_MAX
    track.LIVE_RAM_MAX = 2000
    try:
        lv = st.select("boat_X", "X")
        walk(lv, 5000)
        in_ram = len(lv.track.pts)
        with open(lv.path, encoding="utf-8") as f:
            in_file = sum(1 for ln in f if ln.startswith("["))
        saved = lv.save_as(st.saved_dir, "X")
        with open(saved, encoding="utf-8") as f:
            in_saved = sum(1 for ln in f if ln.startswith("["))
        st.close()
        lv2 = track.TrackStore(d).select("boat_X", "X")
        restored_empty = len(lv2.track.pts) == 0
        lv2.close()
    finally:
        track.LIVE_RAM_MAX = old
    shutil.rmtree(d, True)
    report("TR10 200 000 points at 0.5 m/px: drawn within 50 ms (only the visible chunks), a click within 50 ms, at most "
           "MAX_DRAW drawn at a far zoom; the live RAM bounded, the file + NEW TRACK keep every point; GPX import "
           "<= 200 000 points",
           per < 0.05 and drawn > 0 and hit is not None and t_hit < 0.05 and all_pts <= track.MAX_DRAW + 50 and
           in_ram <= 2000 + track.CHUNK and in_file == 5000 and in_saved == 5000 and restored_empty and
           track.GPX_MAX_POINTS <= 200000,
           "draw %.1f ms (%d points), click %.1f ms (%s), far zoom %d points; RAM %d / file %d / NEW TRACK %d" % (
               per * 1000, drawn, t_hit * 1000, hit and round(hit["t"] - T0), all_pts, in_ram, in_file, in_saved))


def tr11():
    """Review 13n3 LOW 1 / 4 / 5: boats never share a file (another key with the same file name = its own file); a
    clock that steps back never records every frame; times after 2100 are refused (and GPX export never raises)."""
    d, st = fresh()
    a = st.select("boat_A/B", "A/B")            # safe_name -> "boat_A-B"
    walk(a, 3)
    pa = a.path
    b = st.select("boat_A-B", "A-B")            # the same file name, another boat
    b_empty = len(b.track.pts) == 0 and b.path != pa
    walk(b, 2, lat=38.0)
    st.close()
    a2 = track.TrackStore(d).select("boat_A/B", "A/B")
    a_ok = len(a2.track.pts) == 3 and a2.path == pa
    a2.close()
    lv = track.TrackStore(d).select("boat_C", "C")
    lv.feed(T0 + 100, 37.8, 23.6, 5.0, 0.0)
    n0 = len(lv.track.pts)
    for i in range(50):                         # the PC clock jumps back 1 h: the boat lies still
        lv.feed(T0 + 100 - 3600 + i * 0.1, 37.8, 23.6, 5.0, 0.0)
    still = len(lv.track.pts) - n0
    lv.feed(T0 - 3000, 37.8 + 50 / 111320.0, 23.6, 5.0, 0.0)      # ... and then moves: recorded
    moved = len(lv.track.pts) - n0
    lv.close()
    late = track.valid_point((5e10, 37.0, 23.0, None, None))
    ok_2099 = track.valid_point((4102444700.0, 37.0, 23.0, None, None)) is not None
    iso = track._iso(4102444799.0)
    shutil.rmtree(d, True)
    report("TR11 never two boats in one file; a clock stepped back records only real moves; times after 2100 refused",
           b_empty and a_ok and still == 0 and moved == 1 and late is None and ok_2099 and iso.startswith("2099"),
           "same file name: B empty %s, A restored %s; clock back: still %d, moved %d; t 5e10 -> %s; 2099 ok %s (%s)" % (
               b_empty, a_ok, still, moved, late, ok_2099, iso))


def tr12():
    """Review 13n3 LOW 7: the temporary track folder only for a test run (TRYGONS_TEST) - a user's own
    TRYGONS_GS_SETTINGS never moves this PC's real tracks."""
    keep = {k: os.environ.get(k) for k in ("TRYGONS_TRACKS", "TRYGONS_TEST", "TRYGONS_GS_SETTINGS")}
    try:
        os.environ.pop("TRYGONS_TRACKS", None)
        os.environ.pop("TRYGONS_TEST", None)
        os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(tempfile.gettempdir(), "x.json")
        real = track.root_dir()
        os.environ["TRYGONS_TEST"] = "1"
        tmp = track.root_dir()
    finally:
        for k, v in keep.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    report("TR12 the real track folder unless a test run: TRYGONS_GS_SETTINGS alone keeps groundstation/tracks",
           real == os.path.join(track.HERE, "tracks") and tmp != real,
           "GS_SETTINGS only -> %s; TRYGONS_TEST -> a temp folder %s" % (real, tmp != real))


def main():
    for fn in (tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9, tr10, tr11, tr12):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__, False, "crashed: %r" % e)
    bad = results.count(False)
    print("TRACK TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
