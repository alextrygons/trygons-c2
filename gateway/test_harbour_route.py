"""Harbour routes + the read-only picture for the stations (CLAUDE.md Z, step HX6; gateway side).

    python test_harbour_route.py        (exit code 0 = all passed)

  HR1  Gateway.harbour_route_check: the centre line AND both corridor edges on the charts, NO 50 m clearance (a harbour
       route runs near the quay), geofence 120, no charts = 114 (no data = shallow), a bad corridor 129
  HR2  the station message harbour_route / op check: the ack (ok / 114 / 120 / 129 / bad request), bad numbers, 1 or 51
       waypoints, never anything but the ack (read only: no AURIX, no store)
  HR3  the program upload: PROG_BEGIN's 3rd byte (the harbour flag) makes the COMMIT a harbour check (a route 30 m from the
       quay passes; the SAME points as a plain route are refused by the 50 m clearance); a plain upload is unchanged
  HR4  HarbourSensors.view_tracks: compact tracks (no covariance), nearest first, at most the limit, no position = left out
  HR5  harbour_view: pushed (2.5 Hz from harbour_loop) only to the stations that asked, a gone station forgotten, nothing
       while the harbour is off; HarbourPlanner.view carries the AURIX state + the planner's explanation, and nothing in it
       changes what the planner decides
"""
import math
import os
import struct
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gateway as G  # noqa: E402
from common import proto  # noqa: E402
from harbour import planner as P  # noqa: E402
from harbour.manager import HarbourSensors  # noqa: E402

results = []
LAT, LON = 37.82, 23.60
KY = 111320.0


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def ll(x, y):
    return LAT + y / KY, LON + x / (KY * math.cos(math.radians(LAT)))


class Charts:
    """Deep everywhere except east of x = 60 m (a quay) and a shoal band 8..12 m WEST of the line x = 0..  (when shoal)."""

    def __init__(self, shoal=False):
        self.shoal = shoal
        self.calls = []

    def check_leg(self, lat1, lon1, lat2, lon2, limit, clearance, step_m=50.0):
        self.calls.append(clearance)
        n = 40
        for i in range(n + 1):
            la = lat1 + (lat2 - lat1) * i / n
            lo = lon1 + (lon2 - lon1) * i / n
            x = (lo - LON) * KY * math.cos(math.radians(LAT))
            if x > 60.0:
                return (i * 5.0, la, lo, 1.0, "1.0 m water (limit %.1f m)" % limit)
            if self.shoal and -12.0 < x < -8.0:
                return (i * 5.0, la, lo, 1.5, "1.5 m water (limit %.1f m)" % limit)
            if clearance > 0 and x > 10.0:                                  # the 50 m clearance of a plain route
                return (i * 5.0, la, lo, None, "shallow water closer than %.0f m" % clearance)
        return None


class Shallow:
    def __init__(self, charts):
        self.charts = charts

    def limit(self):
        return 3.0

    def check_program(self, wps, start=None):
        for a, b in zip(wps, wps[1:]):
            bad = self.charts.check_leg(a[0], a[1], b[0], b[1], 3.0, 50.0)
            if bad:
                return "leg: %s" % bad[4], bad
        return None


class Fence:
    def __init__(self, why=None):
        self.why = why

    def check(self, wps, start=None):
        return self.why


class St:
    def __init__(self, i=1):
        self.id, self.user, self.role = i, "u", "HELM"
        self.sent = []

    def send(self, m):
        self.sent.append(m)


def gw_stub(charts=None, fence=None, cfg=None):
    g = types.SimpleNamespace()
    g.cfg = cfg or {}
    g.shallow = Shallow(charts)
    g.shallow.charts = charts
    g.geofence = Fence(fence)
    g.audits = []
    g.audit = lambda kind, **kw: g.audits.append(dict(kw, kind=kind))
    g.harbour_route_check = types.MethodType(G.Gateway.harbour_route_check, g)
    g.prog_pending = None
    g.prog_pending_kn = None
    return g


def route(x0=-30.0, x1=-30.0, n=4, step=60.0):
    return [ll(x0 + (x1 - x0) * i / max(1, n - 1), i * step) for i in range(n)]


def hr1():
    ok_route = route()
    c = Charts()
    g = gw_stub(c)
    a = g.harbour_route_check(ok_route, 10.0)
    clear_zero = a is None and c.calls and all(x == 0.0 for x in c.calls)
    # the corridor edge reaches the quay: a route at x = 55 with a +-10 m corridor -> the right (east) edge is at 65 m
    q = g.harbour_route_check(route(55.0, 55.0), 10.0)
    ok_q = q is not None and q[0] == 129 and "right edge" in q[1]
    # the centre line itself in the shallows = 114
    s = g.harbour_route_check(route(70.0, 70.0), 10.0)
    ok_s = s is not None and s[0] == 114
    # a shoal 8..12 m to the west of the route at x = 0: the left edge (-10 m) = 129
    gs = gw_stub(Charts(shoal=True))
    sh = gs.harbour_route_check(route(0.0, 0.0), 10.0)
    ok_sh = sh is not None and sh[0] == 129 and "left edge" in sh[1] and gs.harbour_route_check(route(0.0, 0.0), 4.0) is None
    nochart = gw_stub(None).harbour_route_check(ok_route, 10.0)
    fence = gw_stub(Charts(), fence="leg 1 leaves the fence").harbour_route_check(ok_route, 10.0)
    corr = [g.harbour_route_check(ok_route, v) for v in (2.0, 31.0, float("nan"))]
    dflt = gw_stub(Charts(), cfg={"harbour": {"corr_m": 8.0}}).harbour_route_check(route(55.0, 55.0), None)
    ok_dflt = dflt is not None and dflt[0] == 129                             # 55 + 8 = 63 m > 60 m: the default corridor is used
    report("HR1 the harbour route check: centre line + both corridor edges on the charts with NO 50 m clearance (a plain "
           "check would refuse the same route), the corridor edge at the quay = 129, shallow centre = 114, a shoal beside "
           "the line = 129 (a narrower corridor passes), geofence = 120, no charts = 114, a bad corridor = 129",
           clear_zero and ok_q and ok_s and ok_sh and nochart[0] == 114 and fence[0] == 120 and all(x and x[0] == 129 for x in corr)
           and ok_dflt,
           "clearance 0 %s quay %s shallow %s shoal %s nochart %s fence %s corr %s default %s" % (
               clear_zero, q, s, sh, nochart, fence, corr, dflt))


def hr2():
    c = Charts()
    g = gw_stub(c)
    g.harbour_route_msg = types.MethodType(G.Gateway.harbour_route_msg, g)
    st = St()

    def ask(**m):
        st.sent.clear()
        g.harbour_route_msg(st, dict({"t": "harbour_route", "op": "check", "seq": 7}, **m))
        return st.sent[0] if len(st.sent) == 1 else st.sent

    def wl(pts, kn=3.0):
        return [{"lat": a, "lon": b, "kn": kn} for a, b in pts]
    ok = ask(wps=wl(route()), corr_m=10.0)
    q = ask(wps=wl(route(55.0, 55.0)), corr_m=10.0)
    bad = [ask(wps=[]), ask(wps=wl(route())[:1]), ask(wps=wl(route(n=51, step=12.0))), ask(wps="x"),
           ask(wps=[{"lat": "a", "lon": 1}]), ask(wps=[{"lat": float("nan"), "lon": 1}] * 3),
           ask(wps=[{"lat": 95.0, "lon": 1}, {"lat": 1.0, "lon": 1.0}]), ask()]
    only_ack = all(isinstance(b, dict) and b["t"] == "harbour_ack" and b["ok"] is False for b in bad)
    ok_ack = ok["ok"] is True and ok["code"] == 0 and ok["seq"] == 7 and ok["op"] == "check"
    q_ack = q["ok"] is False and q["code"] == 129
    no_state = g.prog_pending is None and not g.audits
    report("HR2 harbour_route check -> exactly one harbour_ack (ok / 129 / refused), bad numbers / 0, 1, 51 waypoints / "
           "garbage = refused, nothing stored, no audit, nothing else sent",
           ok_ack and q_ack and only_ack and no_state, "ok %s quay %s bad %s state %s" % (ok, q, [b["why"] for b in bad
                                                                                                 if isinstance(b, dict)][:3], no_state))


def hr3():
    c = Charts()
    g = gw_stub(c)
    g.prog_ack_new = False
    g.telem = {}
    g.check_hdg_shore = lambda st, payload: None
    g.hplan = types.SimpleNamespace(note_start=lambda: None)
    g.own = lambda: {"lat": None}
    g.prog_wps = None

    def upload(pts, flag):
        st = St()
        pb = struct.pack("<BB", len(pts), 0) + (b"\x01" if flag else b"")
        r = [G.Gateway.check_prog(g, st, proto.T_PROG_BEGIN, pb)]
        for i, (la, lo) in enumerate(pts):
            r.append(G.Gateway.check_prog(g, st, proto.T_PROG_WP, struct.pack("<Biif", i, round(la * 1e7), round(lo * 1e7), 3.0)
                                          + bytes(6)))
        r.append(G.Gateway.check_prog(g, st, proto.T_PROG_COMMIT, b""))
        return st, r[-1]
    near = route(25.0, 25.0)                                         # 25 m from the quay side: inside the plain 50 m clearance
    st_h, res_h = upload(near, True)
    st_p, res_p = upload(near, False)
    ok_flag = st_h.prog_up["harbour"] is True and st_p.prog_up["harbour"] is False
    ok_h = res_h is None and g.prog_pending is not None
    g.prog_pending = None
    ok_p = res_p is not None and ("shallow" in str(res_p) or "leg" in str(res_p))
    # a harbour route whose corridor touches the quay is refused 129 with the reason, audited
    st_q, res_q = upload(route(55.0, 55.0), True)
    ok_q = isinstance(res_q, tuple) and res_q[0] == 129 and g.audits and g.audits[-1]["kind"] == "program_refused" and \
        g.audits[-1].get("harbour") is True
    # the 2-byte PROG_BEGIN (every older station) is a plain upload
    ok_two = upload(near, False)[0].prog_up["harbour"] is False
    report("HR3 the program upload: PROG_BEGIN's flag byte makes the commit a harbour check (a route 25 m from the quay passes "
           "it, the same points as a plain route are refused), a corridor at the quay = 129 + audit, a plain upload unchanged",
           ok_flag and ok_h and ok_p and ok_q and ok_two, "flag %s harbour %s plain %s quay %s two %s" % (
               ok_flag, ok_h, res_p, res_q, ok_two))


class FakeTracker:
    def __init__(self, tracks):
        self.tracks = tracks

    def tracks_out(self):
        return list(self.tracks)


def tr(i, x, y, lat=LAT, steer=True):
    return {"id": i, "t": 1.0, "x": x, "y": y, "lat": lat, "lon": LON, "vx": 0.1, "vy": 0.2, "speed": 0.22, "course": 10.0,
            "P": [[1, 0, 0, 0]] * 4, "sd_m": 2.0, "len_m": 5.0, "beam_m": 2.0, "hgt_m": None, "cls": "vessel", "conf": 0.7,
            "src": "RL", "last": {}, "hits_R": 3, "hits_L": 5, "confirmed": True, "steer_ok": steer, "range_ok": steer,
            "coasting": False, "static": False, "in_static_map": False, "surface": False, "ghost_suspect": False,
            "mmsi": None, "age_s": 3.0, "born": 0.0}


def hr4():
    hs = types.SimpleNamespace()
    hs.tracker = FakeTracker([tr(1, 90, 0), tr(2, 10, 0), tr(3, 0, 40, lat=None), tr(4, 0, 5)] + [tr(10 + i, 100 + i, 0) for i in range(80)])
    v = HarbourSensors.view_tracks(hs, limit=10)
    ids = [t["id"] for t in v]
    ok = len(v) == 10 and ids[:3] == [4, 2, 1] and 3 not in ids and all("P" not in t and "x" not in t and "y" not in t for t in v) \
        and v[0]["steer_ok"] is True and set(v[0]) >= {"id", "lat", "lon", "speed", "course", "cls", "conf", "src", "steer_ok"}
    report("HR4 view_tracks: compact tracks (no covariance / local x y), nearest first, capped, a track without a position left "
           "out", ok, "ids %s" % ids)


def hr5():
    sent = []
    g = types.SimpleNamespace()
    st1, st2 = St(1), St(2)
    g.stations = types.SimpleNamespace(stations={1: st1, 2: st2})
    g.hview_subs = set()
    g.harbour = types.SimpleNamespace(enabled=True, view_tracks=lambda: [{"id": 1}])
    g.hplan = types.SimpleNamespace(view=lambda: {"state": 1})
    g.harbour_view_msg = types.MethodType(G.Gateway.harbour_view_msg, g)
    g.harbour_view_push = types.MethodType(G.Gateway.harbour_view_push, g)
    g.harbour_view_push()
    nobody = not st1.sent and not st2.sent
    g.harbour_view_msg(st1, {"t": "harbour_view_on", "on": True})
    g.harbour_view_msg(st2, {"t": "harbour_view_on", "on": True})
    g.harbour_view_msg(st2, {"t": "harbour_view_on", "on": False})
    g.harbour_view_push()
    ok_sub = len(st1.sent) == 1 and not st2.sent and st1.sent[0]["t"] == "harbour_view" and st1.sent[0]["tracks"] == [{"id": 1}] \
        and st1.sent[0]["plan"] == {"state": 1}
    g.hview_subs.add(99)
    g.harbour_view_push()
    ok_gone = 99 not in g.hview_subs
    g.harbour.enabled = False
    n = len(st1.sent)
    g.harbour_view_push()
    ok_off = len(st1.sent) == n
    # the planner's view
    import test_harbour_planner as TP
    hp = P.HarbourPlanner(TP.FakeGw())
    hp.on_stat({"harb_state": 1, "harb_state_name": "RUN", "harb_why": 0, "harb_why_text": "-", "harb_kn": 4.0,
                "harb_corr_m": 10.0, "harb_xte_m": 1.0, "harb_route": True, "harb_entry_ok": True, "harb_lease": True,
                "harb_grace": False, "harb_grace_left_s": 0.0, "harb_max_kn": 5.0})
    hp.dec = {"stop": False, "heading": 12.0, "cap_kn": 3.0, "why": "harb.why_clear", "text_el": "a", "text_en": "b",
              "colregs": [], "turn": False, "reasons": ["x"] * 9, "hflags": 2}
    before = dict(hp.dec)
    v = hp.view()
    ok_view = v["state_name"] == "RUN" and v["kn"] == 4.0 and v["heading"] == 12.0 and v["text_en"] == "b" and len(v["unsure"]) == 6 \
        and v["stat_fresh"] is True and v["hflags"] == 2 and hp.dec == before
    report("HR5 harbour_view is pushed only to the stations that asked (a gone one forgotten, nothing while the harbour is "
           "off); the planner's view carries the AURIX state + the explanation and changes nothing",
           nobody and ok_sub and ok_gone and ok_off and ok_view, "nobody %s sub %s gone %s off %s view %s" % (
               nobody, ok_sub, ok_gone, ok_off, ok_view))
    _ = sent


def main():
    for fn in (hr1, hr2, hr3, hr4, hr5):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "crashed: %r" % e)
    bad = results.count(False)
    print("HARBOUR ROUTE TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
