"""ADS-B receive only (CLAUDE.md X K4, step PF5): parsing, aircraft state, filters, CPA, warning levels, proposals.

    python test_adsb.py        (exit code 0 = all passed)

  AB1  SBS parsing (types 1 / 3 / 4 / 5 / 8, broken lines counted, out-of-range position refused)
  AB2  Beast: framing (escapes, a split feed, garbage), CRC-24, DF17 identity / altitude / velocity / CPR position
  AB3  aircraft state: stale / removed, a short track, merge of the partial messages
  AB4  filters: altitude, range, on the ground; an unknown altitude is KEPT
  AB5  CPA geometry: head-on, crossing, diverging, vertical separation, an old position moved along its velocity
  AB6  warning levels: CAUTION / WARNING limits + times, unknown altitude = CAUTION only, only UAVs in the air
  AB7  proposals: HOLD / DESCEND, one per UAV, held 10 s, never without an event
  AB8  no receiver / no data states (the real worker process): NO_RX, NO_DATA, RX; never "clear sky"
  AB9  the simulated helicopter end to end (worker + sim): a CAUTION then a WARNING for the UAV, a proposal
  AB10 receive-only proof: no ADS-B module imports / names a UAV command, MAVLink, the AURIX or the allow-list
"""
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import adsb  # noqa: E402
import adsbmgr  # noqa: E402
import simadsb  # noqa: E402

results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


CFG = adsb.merged({})
LAT0, LON0 = 37.82, 23.60


def sbs(t, icao, **kw):
    f = ["MSG", str(t), "1", "1", icao, "1", "2026/11/09", "10:00:00.000", "2026/11/09", "10:00:00.000"] + [""] * 12
    for k, i in (("cs", 10), ("alt", 11), ("gs", 12), ("trk", 13), ("lat", 14), ("lon", 15), ("vs", 16), ("gnd", 21)):
        if k in kw:
            f[i] = str(kw[k])
    return ",".join(f)


def ll(dx, dy, lat=LAT0, lon=LON0):
    """a point dx m east, dy m north of (lat, lon)"""
    import math
    return lat + dy / 111320.0, lon + dx / (111320.0 * math.cos(math.radians(lat)))


def ac(dx, dy, alt_ft, gs_kn, trk, age=0.0, vs=0.0, icao="ABC123"):
    la, lo = ll(dx, dy)
    return {"icao": icao, "cs": "TEST1", "lat": la, "lon": lo, "alt_ft": alt_ft, "gs_kn": gs_kn, "trk": trk,
            "vs_fpm": vs, "age": age, "gnd": False}


def uav(alt=50.0, gs=0.0, cog=0.0, climb=0.0, armed=True):
    return {"slot": 1, "lat": LAT0, "lon": LON0, "alt": alt, "gs": gs, "cog": cog, "climb": climb, "armed": armed,
            "landed": False, "in_slot": False}


def beast_frame(msg, typ=0x33):
    body = bytes(6) + b"\x40" + msg
    out = bytearray([0x1A, typ])
    for b in body:
        out.append(b)
        if b == 0x1A:
            out.append(0x1A)
    return bytes(out)


def ab1():
    p = adsb.parse_sbs
    a = p(sbs(1, "4840d6", cs="KLM1023"))
    b = p(sbs(3, "4840D6", alt=1500, lat=37.8, lon=23.6))
    c = p(sbs(4, "4840D6", gs=159, trk=182.9, vs=-640))
    d = p(sbs(5, "4840D6", alt=2000))
    e = p(sbs(8, "4840D6", gnd=-1))
    bad = [p("garbage"), p(""), p("MSG,3,1,1,ZZZZZZ,1,,,,,,,,,,,,,,,,"), p(sbs(3, "4840D6", alt=1, lat=95.0, lon=10.0)),
           p("MSG,9,1,1,4840D6,1,,,,,,,,,,,,,,,,"), p("MSG,3,1,1,4840D6"), p(sbs(3, "4840D6")), p(sbs(3, "4840D6", lat="nan", lon="x"))]
    ok = (a == {"icao": "4840D6", "cs": "KLM1023"} and b["alt_ft"] == 1500 and b["lat"] == 37.8 and
          c["gs_kn"] == 159 and abs(c["trk"] - 182.9) < 1e-6 and c["vs_fpm"] == -640 and d["alt_ft"] == 2000 and
          e["gnd"] is True and all(x is None for x in bad))
    report("AB1 SBS parsing", ok, "types 1 / 3 / 4 / 5 / 8 read, %d broken lines refused" % len(bad))


def ab2():
    ident = bytes.fromhex("8D4840D6202CC371C32CE0576098")
    even = bytes.fromhex("8D40621D58C382D690C8AC2863A7")
    odd = bytes.fromhex("8D40621D58C386435CC412692AD6")
    vel = bytes.fromhex("8D485020994409940838175B284F")
    crc_ok = all(adsb.crc24(m) == 0 for m in (ident, even, odd, vel))
    bad = bytearray(ident)
    bad[5] ^= 0x04
    crc_bad = adsb.crc24(bytes(bad)) != 0
    bp = adsb.BeastParser()
    s1 = bp.decode(ident, 100.0)
    p1 = bp.decode(odd, 100.0)                                       # the NEWEST frame decides: even last
    p2 = bp.decode(even, 101.0)
    v1 = bp.decode(vel, 101.0)
    ok_dec = (s1 == {"icao": "4840D6", "cs": "KLM1023"} and p1["alt_ft"] == 38000 and "lat" not in p1 and
              abs(p2["lat"] - 52.2572) < 0.002 and abs(p2["lon"] - 3.9194) < 0.002 and
              abs(v1["gs_kn"] - 159.2) < 0.3 and abs(v1["trk"] - 182.88) < 0.2 and v1["vs_fpm"] == -832)
    # framing: two frames in one feed, the second split, an escape, garbage before
    stream = b"\x00\x01garbage" + beast_frame(ident) + beast_frame(odd) + beast_frame(even)
    bp2 = adsb.BeastParser()
    got = []
    for i in range(0, len(stream), 5):                        # 5-byte pieces: every split point
        got += bp2.feed(stream[i:i + 5], 200.0)
    got_ids = [g["icao"] for g in got]
    # an escaped 0x1a inside a body (the timestamp) must survive
    esc = bytearray([0x1A, 0x33]) + bytes([0x1A, 0x1A, 1, 2, 3, 4, 5, 0x40]) + ident
    bp3 = adsb.BeastParser()
    r3 = bp3.feed(bytes(esc), 1.0)
    # a bad CRC counts, decodes nothing; a short frame / unknown type is skipped
    badframe = beast_frame(bytes(bad))
    bp4 = adsb.BeastParser()
    r4 = bp4.feed(badframe + b"\x1a\x39xx" + beast_frame(ident, 0x32)[:5], 1.0)
    ok = (crc_ok and crc_bad and ok_dec and got_ids == ["4840D6", "40621D", "40621D"] and
          abs(got[-1].get("lat", 0) - 52.2572) < 0.002 and r3 and r3[0]["cs"] == "KLM1023" and
          r4 == [] and bp4.bad >= 1)
    report("AB2 Beast framing + CRC-24 + DF17 (identity, altitude, velocity, CPR position)", ok,
           "crc ok %s / bad detected %s, decoded %s, split feed %s, escape %s, bad counted %d" %
           (crc_ok, crc_bad, ok_dec, got_ids, bool(r3), bp4.bad))


def ab3():
    t = adsb.AdsbTargets({"track_s": 60.0})
    t.update({"icao": "A1", "cs": "HEL1"}, 0.0)
    t.update({"icao": "A1", "alt_ft": 500.0, "lat": 37.0, "lon": 23.0}, 1.0)
    t.update({"icao": "A1", "gs_kn": 70.0, "trk": 90.0}, 1.5)
    for i in range(2, 80):
        t.update({"icao": "A1", "lat": 37.0, "lon": 23.0 + i * 1e-4}, float(i))
    a = t.ac["A1"]
    t.update({"icao": "A2", "cs": "NOPOS"}, 80.0)                   # no position yet = not in the snapshot
    snap = t.snapshot(80.0)
    first, last = a["track"][0][2], a["track"][-1][2]
    t.tick(79.0 + 190.0)
    ok = (a["cs"] == "HEL1" and a["alt_ft"] == 500.0 and a["gs_kn"] == 70.0 and len(snap) == 1 and
          snap[0]["icao"] == "A1" and snap[0]["age"] <= 1.0 and last - first <= 60.0 and len(a["track"]) > 5 and
          "A1" not in t.ac and "A2" not in t.ac)
    report("AB3 aircraft state: merge, short track, removed after 180 s", ok,
           "track %d points over %.0f s, removed" % (len(a["track"]), last - first))


def ab4():
    refs = [(LAT0, LON0)]
    near = ac(5000, 0, 1000, 80, 270)
    far = ac(30000, 0, 1000, 80, 270)
    high = ac(5000, 0, 8000, 400, 270)
    unk = ac(5000, 0, None, 80, 270)
    gnd = dict(ac(5000, 0, 0, 0, 0), gnd=True)
    nopos = {"icao": "X", "lat": None, "lon": None}
    ok = (adsb.keep(near, refs, CFG) and not adsb.keep(far, refs, CFG) and not adsb.keep(high, refs, CFG) and
          adsb.keep(unk, refs, CFG) and not adsb.keep(gnd, refs, CFG) and not adsb.keep(nopos, refs, CFG) and
          adsb.keep(far, [], CFG) and adsb.keep(far, [(LAT0, LON0), ll(29000, 0)], CFG))
    cfg2 = adsb.merged({"max_alt_ft": 500, "max_km": 2})
    ok2 = not adsb.keep(near, refs, cfg2) and not adsb.keep(ac(1000, 0, 1000, 80, 270), refs, cfg2) and \
        adsb.keep(ac(1000, 0, 400, 80, 270), refs, cfg2)
    report("AB4 filters (3000 ft, 20 km, on the ground; unknown altitude kept; settings)", ok and ok2, "ok %s %s" % (ok, ok2))


def ab5():
    u = uav(alt=100.0)
    head_on = adsb.cpa(ac(2000, 0, 330, 80, 270), u)               # 2 km east, flying west, 100 m = ~328 ft
    s = 80 * adsb.KN
    ok_head = abs(head_on["t"] - 2000.0 / s) < 1.0 and head_on["d_h"] < 5.0 and head_on["d_v"] < 5.0
    cross = adsb.cpa(ac(2000, -2000, 330, 80, 315), u)               # crosses over the UAV from the south-east
    ok_cross = cross["t"] > 5 and cross["d_h"] < 5.0
    moving = adsb.cpa(ac(2000, 0, 330, 80, 270), uav(alt=100.0, gs=10.0, cog=0.0))    # the UAV flies north 10 m/s
    ok_cross = ok_cross and moving["d_h"] > 100 and moving["t"] < 49
    div = adsb.cpa(ac(1500, 0, 330, 80, 90), u)                     # moving away
    ok_div = div["t"] == 0.0 and abs(div["d_h"] - 1500) < 5 and div["d_h0"] == div["d_h"]
    vert = adsb.cpa(ac(2000, 0, 1500, 80, 270), u)                 # same track, 457 m up
    ok_vert = vert["d_h"] < 5.0 and 340 < vert["d_v"] < 370
    old = adsb.cpa(ac(2000, 0, 330, 80, 270, age=20.0), u)         # 20 s old: it has already flown 823 m
    ok_old = abs(old["d_h0"] - (2000 - 20 * s)) < 3.0
    unk = adsb.cpa(ac(2000, 0, None, 80, 270), u)
    ok_unk = unk["d_v"] is None and unk["ac_alt_m"] is None
    # a climbing aircraft: the vertical distance at the CPA uses its climb rate
    climb = adsb.cpa(ac(2000, 0, 100.0 / 0.3048 - 400.0, 80, 270, vs=600.0), u)
    ok_climb = climb["d_v"] < 60.0
    report("AB5 CPA geometry (head-on, crossing, diverging, vertical, age, unknown altitude, climbing)",
           ok_head and ok_cross and ok_div and ok_vert and ok_old and ok_unk and ok_climb,
           "head-on t=%.0f s dh=%.0f, crossing dh=%.0f, diverging t=%.0f dh=%.0f, vertical dv=%.0f, climb dv=%.0f" %
           (head_on["t"], head_on["d_h"], cross["d_h"], div["t"], div["d_h"], vert["d_v"], climb["d_v"]))


def lv(a, u=None):
    ev, _f = adsb.evaluate([a], {"1": u or uav(alt=100.0)}, CFG)
    return ev[0] if ev else None


def ab6():
    # head-on at 80 kn: 41 m/s; 2000 m = 49 s (WARNING needs <= 60 s and < 500 m / < 100 m)
    w = lv(ac(2000, 0, 330, 80, 270))
    c = lv(ac(4000, 0, 330, 80, 270))                               # 97 s away: CAUTION only
    far_t = lv(ac(7000, 0, 330, 80, 270))                           # 170 s: neither
    sep_v = lv(ac(2000, 0, 330 + 400, 80, 270))                     # 122 m above: d_v 122 -> CAUTION not WARNING
    sep_v2 = lv(ac(2000, 0, 330 + 700, 80, 270))                    # 213 m above: nothing
    side = lv(ac(2000, 800, 330, 80, 270))                          # passes 800 m north: CAUTION (< 1000, > 500)
    wide = lv(ac(2000, 1500, 330, 80, 270))                         # 1500 m: nothing
    unk = lv(ac(2000, 0, None, 80, 270))                            # no altitude: CAUTION, never WARNING
    old = lv(ac(2000, 0, 330, 80, 270, age=40.0))                   # too old to judge
    gnd = lv(dict(ac(2000, 0, 330, 80, 270), gnd=True))
    ground_uav = adsb.evaluate([ac(2000, 0, 330, 80, 270)], {"1": uav(armed=False)}, CFG)[0]
    slot_uav = adsb.evaluate([ac(2000, 0, 330, 80, 270)], dict({"1": dict(uav(), in_slot=True)}), CFG)[0]
    landed = adsb.evaluate([ac(2000, 0, 330, 80, 270)], {"1": dict(uav(), landed=True)}, CFG)[0]
    nopos = adsb.evaluate([ac(2000, 0, 330, 80, 270)], {"1": dict(uav(), lat=None)}, CFG)[0]
    near_now = lv(ac(300, 0, 330, 80, 90))                          # already close, moving away: still a WARNING now
    ok = (w and w["level"] == 2 and c and c["level"] == 1 and far_t is None and sep_v and sep_v["level"] == 1 and
          sep_v2 is None and side and side["level"] == 1 and wide is None and unk and unk["level"] == 1 and
          old is None and gnd is None and ground_uav == [] and slot_uav == [] and landed == [] and nopos == [] and
          near_now and near_now["level"] == 2)
    report("AB6 warning levels (CAUTION 1000 m / 150 m / 120 s, WARNING 500 m / 100 m / 60 s; only UAVs in the air)",
           bool(ok), "head-on 2 km=%s, 4 km=%s, 7 km=%s, 122 m above=%s, side 800 m=%s, no altitude=%s, now close=%s" %
           (w and w["level"], c and c["level"], far_t, sep_v and sep_v["level"], side and side["level"],
            unk and unk["level"], near_now and near_now["level"]))


class FakeGw:
    class uav:
        enabled = True
        d = {}

        @staticmethod
        def boat():
            return {"pos": (LAT0, LON0)}

        @classmethod
        def drones(cls):
            return cls.d


def mgr(cfg=None):
    return adsbmgr.AdsbManager(dict({"enabled": True}, **(cfg or {})), FakeGw())


def ab7():
    k = adsb.proposal_kind
    hi = {"ac_alt_m": 150.0, "uav_alt_m": 100.0}
    lo = {"ac_alt_m": 30.0, "uav_alt_m": 100.0}
    low_uav = {"ac_alt_m": 150.0, "uav_alt_m": 35.0}
    unk = {"ac_alt_m": None, "uav_alt_m": 100.0}
    ok_k = k(hi, CFG) == "descend" and k(lo, CFG) == "hold" and k(low_uav, CFG) == "hold" and k(unk, CFG) == "hold"
    m = mgr()
    u1, u2 = uav(alt=100.0), dict(uav(alt=60.0), slot=2)
    heli = ac(2000, 0, 330 + 200, 80, 270)
    ev = m.judge(0.0, [heli], {"1": u1, "2": u2}, state="RX")
    p1 = {s: dict(p) for s, p in m.props.items()}
    m.judge(1.0, [heli], {"1": u1, "2": u2}, state="RX")
    same_id = m.props[1]["id"] == p1[1]["id"]
    # the aircraft goes: the warning is HELD for hold_s, then the proposal ends
    m.judge(5.0, [], {"1": u1, "2": u2}, state="RX")
    held = 1 in m.props
    m.judge(20.0, [], {"1": u1, "2": u2}, state="RX")
    gone = not m.props and not m.warn
    # a new aircraft = a new proposal id
    m.judge(21.0, [heli], {"1": u1}, state="RX")
    new_id = m.props[1]["id"] != p1[1]["id"]
    # no receiver / no data: no judgement at all
    m2 = mgr()
    m2.judge(0.0, [heli], {"1": u1}, state="NO_RX")
    m3 = mgr()
    m3.judge(0.0, [heli], {"1": u1}, state="NO_DATA")
    st = m.status()
    ok = (ok_k and len(ev) == 2 and sorted(p1) == [1, 2] and p1[1]["kind"] == "descend" and same_id and held and gone and
          new_id and not m2.props and not m2.warn and not m3.props and st["proposals"] and st["state"] in
          (adsbmgr.NO_RX, adsbmgr.NO_DATA, adsbmgr.RX))
    report("AB7 proposals: HOLD / DESCEND by the geometry, one per UAV, held 10 s, none without a receiver", ok,
           "events %d, proposals %s, kind %s" % (len(ev), sorted(p1), p1[1]["kind"]))


def tcp_server(send=None, rate=0.2):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    s.listen(2)
    stop = threading.Event()
    conns = []

    def run():
        s.settimeout(0.3)
        while not stop.is_set():
            try:
                c, _ = s.accept()
            except (socket.timeout, OSError):
                continue
            conns.append(c)
            while send and not stop.is_set():
                try:
                    c.sendall(send)
                except OSError:
                    break
                time.sleep(rate)
    threading.Thread(target=run, daemon=True).start()
    return s, stop, conns


def worker_state(port, secs, proto="sbs", extra=None):
    cmd = [sys.executable, os.path.join(HERE, "adsb_worker.py"), "--source", "tcp:127.0.0.1:%d" % port, "--proto", proto,
           "--cfg", json.dumps({"no_data_s": 2.0})] + (extra or [])
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    states, t0 = [], time.time()
    lines = []

    def rd():
        for ln in p.stdout:
            try:
                lines.append(json.loads(ln.decode()))
            except ValueError:
                pass
    threading.Thread(target=rd, daemon=True).start()
    time.sleep(secs)
    try:
        p.stdin.close()
    except OSError:
        pass
    try:
        p.wait(5)
    except subprocess.TimeoutExpired:
        p.kill()
    return [m for m in lines if m.get("ev") == "state"], p.returncode, time.time() - t0


def ab8():
    s0 = socket.socket()
    s0.bind(("127.0.0.1", 0))
    closed_port = s0.getsockname()[1]
    s0.close()                                                       # nothing listens here
    a, _rc, _t = worker_state(closed_port, 5.5)
    no_rx = bool(a) and all(m["state"] == "NO_RX" for m in a) and bool(a[-1]["err"])
    srv, stop, _c = tcp_server(send=None)                           # connected, silent
    b, _rc, _t = worker_state(srv.getsockname()[1], 5.0)
    no_data = bool(b) and b[-1]["state"] == "NO_DATA" and b[-1]["targets"] == []
    stop.set()
    srv.close()
    line = (sbs(3, "4840D6", alt=1500, lat=LAT0, lon=LON0) + "\r\n" + sbs(4, "4840D6", gs=80, trk=270) + "\r\nbad line\r\n").encode()
    srv2, stop2, _c2 = tcp_server(send=line)
    c, _rc, _t = worker_state(srv2.getsockname()[1], 4.5)
    stop2.set()
    srv2.close()
    rx = bool(c) and c[-1]["state"] == "RX" and c[-1]["targets"] and c[-1]["targets"][0]["icao"] == "4840D6" and \
        c[-1]["bad"] >= 1
    # the manager says the same: a worker that is not there = NO_RX; never RX without a state line
    m = mgr()
    m.proc = None
    st_off = m.state(10.0)
    m_off = adsbmgr.AdsbManager({"enabled": False}, FakeGw())
    ok = no_rx and no_data and rx and st_off == adsbmgr.NO_RX and m_off.status()["state"] == adsbmgr.OFF and \
        m_off.status()["targets"] == []
    report("AB8 no receiver = NO_RX, no frames = NO_DATA, frames = RX (never an empty 'clear sky')", ok,
           "closed port %s, silent server %s, frames %s, manager %s / disabled %s" %
           (no_rx, no_data, rx, st_off, m_off.status()["state"]))


def beast_server(frames):
    return tcp_server(send=b"".join(beast_frame(f) for f in frames), rate=0.3)


def ab9():
    # (a) the Beast path through the real worker
    even = bytes.fromhex("8D40621D58C382D690C8AC2863A7")
    odd = bytes.fromhex("8D40621D58C386435CC412692AD6")
    srv, stop, _c = beast_server([even, odd])
    st, _rc, _t = worker_state(srv.getsockname()[1], 4.0, proto="beast")
    stop.set()
    srv.close()
    beast_ok = bool(st) and st[-1]["state"] == "RX" and st[-1]["targets"] and \
        abs(st[-1]["targets"][0]["lat"] - 52.2572) < 0.01
    # (b) the simulated helicopter, end to end: sim worker + the manager judging a UAV at the origin
    FakeGw.uav.d = {"1": dict(uav(alt=60.0), lat=LAT0, lon=LON0)}
    m = adsbmgr.AdsbManager({"enabled": True, "sim": True, "sim_start_m": 2800.0, "hold_s": 3.0}, FakeGw())
    levels, kinds, seen_rx, t0 = set(), set(), False, time.time()
    try:
        while time.time() - t0 < 40.0:
            m.tick()
            stt = m.status()
            seen_rx = seen_rx or stt["state"] == adsbmgr.RX
            for e in stt["warn"]:
                levels.add(e["level"])
            for p in stt["proposals"]:
                kinds.add(p["kind"])
            if 2 in levels and 1 in levels:
                break
            time.sleep(1.0)
        listed = [a["cs"] for a in m.status()["targets"]]
    finally:
        m.stop()
    ok = beast_ok and seen_rx and levels == {1, 2} and bool(kinds) and "HEL123" in listed
    report("AB9 Beast via the worker; the simulated helicopter gives CAUTION then WARNING + a proposal", ok,
           "beast %s, RX %s, levels %s, proposals %s, listed %s, %.0f s" % (beast_ok, seen_rx, sorted(levels),
                                                                            sorted(kinds), listed, time.time() - t0))
    FakeGw.uav.d = {}


def ab10():
    banned = re.compile(r"uav_hold|uav_route|hold_msg|route_msg|pilots|mavlink|allow\b|aurix|import\s+uav|from\s+uav|"
                        r"launch|navreq|send_msg|proto\.", re.I)
    files = ["adsb.py", "adsbmgr.py", "adsb_worker.py", "simadsb.py"]
    hits = []
    for f in files:
        src = open(os.path.join(HERE, f), encoding="utf-8").read()
        # the docstrings / comments say what is NOT done; only code lines count
        code = "\n".join(ln.split("#")[0] for ln in re.sub(r'""".*?"""', "", src, flags=re.S).splitlines())
        for mt in banned.finditer(code):
            hits.append((f, mt.group(0)))
    mgr_src = open(os.path.join(HERE, "adsbmgr.py"), encoding="utf-8").read()
    uav_calls = set(re.findall(r"gw\.uav\.(\w+)", mgr_src))
    gw_src = open(os.path.join(HERE, "gateway.py"), encoding="utf-8").read()
    adsb_ext = re.findall(r"stations\.ext\[[^\]]*adsb", gw_src)
    status_only = "self.adsb.status(" in gw_src
    ok = not hits and uav_calls <= {"boat", "drones", "enabled"} and not adsb_ext and status_only
    report("AB10 receive only: no UAV command / MAVLink / AURIX / allow-list in any ADS-B module, no station command",
           ok, "hits %s, gw.uav calls %s" % (hits[:4], sorted(uav_calls)))


def main():
    for f in (ab1, ab2, ab3, ab4, ab5, ab6, ab7, ab8, ab9, ab10):
        try:
            f()
        except Exception as e:                              # a crash is a failure, with its reason
            import traceback
            traceback.print_exc()
            report(f.__name__.upper(), False, "crashed: %r" % e)
    bad = results.count(False)
    print("ADS-B TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
