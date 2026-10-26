"""Screen 2 (navigation & control) + the shared look of both screens (CLAUDE.md section E).

  S1 layout matrix: 1366x768 and 1920x1080 windows (and the same screens maximised: minus the
     taskbar / title / menu), Windows scaling 100 / 125 / 150 %, Greek and English, every panel
     full (boats, HELM, link, AIS, alarms, route, decisions): nothing clipped (every text,
     button and table cell inside its panel, every panel inside the window), STANDBY visible,
     no panel over the boat, the logo under no panel, panels not over each other
  S2 every panel collapsed: STANDBY still visible and clickable; a header click opens the panel
  S3 roles: no MANUAL / MANEUVER on screen 2 (captain or copilot); single-operator + licence:
     shown; screen 1 shows REMOTE MANUAL / MANEUVER only for the captain with the licence
  S4 MANEUVER on screen 1: REV / N / FWD with the engaged gear lit, RELEASE button
  S5 the map: boat in the centre, north-up, the boat symbol turns with the heading, zoom,
     a pan snaps back to the boat by itself and with "back to boat"
  S6 editing on the map: route (click adds, drag moves, right click deletes, Send = upload),
     geofence (corners, NOT SENT state, only through the gateway), max speed
  S7 link panel: channel quality, CREWED / UNCREWED, near shore / open sea + distance, the
     lost-link countdown, ENGINE KILLED
  S8 boat selector: no "boats" panel any more; the dropdown lists the boats of boats/ by name,
     ONLINE / OFFLINE / IP not set, the IP only as a detail; "name verified" only after the confirm
     step, small
  S9 one look: no colour / font written in a screen module (all from theme.py); dark / light
     switches BOTH screens; the meaning colours (warning / caution) are the same on both; the
     logo on both screens, under nothing on screen 1 either
  S10 the tools pages: page names never cut off, every table heading fits its column
  S11 the Greek screen: no English words left (panels, tools pages, dialogs, the HUD texts)
  S12 alarms panel (CLAUDE.md G): top-right, always drawn, outside the boat circle, one row per
      alarm, unacknowledged rows flash on the MASTER lamp clock, ACK ALARM, no alarm banner
  S13 boat dropdown (CLAUDE.md H): the FIRST line of the status panel = the dropdown + the lamp
      (CONNECTED green only when found + connected + welcome name check passed, else grey), choosing
      a boat opens the login / confirm step with it selected, the line fits in the S1 matrix
(The captain's ACCEPT of a copilot's route / geofence through a real gateway: gateway test GF.)

Run:  python test_screen2.py        (exit code 0 = all passed)
"""
import os
import queue
import re
import socket
import subprocess
import sys
import tempfile
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
TMP = tempfile.mkdtemp(prefix="trygons_s2_")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(TMP, "settings.json")
os.environ["TRYGONS_HUD_LAYOUT"] = os.path.join(TMP, "hud_layout.json")
os.environ["TRYGONS_SOUND"] = "off"
os.environ.pop("TRYGONS_LANG", None)
import i18n  # noqa: E402
import theme  # noqa: E402
import overlay  # noqa: E402
import proto  # noqa: E402
import app as A  # noqa: E402

results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


SHIPS = ["NISSOS MYKONOS", "BLUE STAR DELOS", "AGIOS NEKTARIOS AIGINAS", "FLYINGCAT 4", "SEAJETS POWER",
         "TUG HERCULES II", "KALLISTI", "OLYMPIC CHAMPION", "SAILING ATHENA", "FISHING 12"]


def fake_gw(t, captain=True, single_op=False, licence=True):
    la, lo = t["est_lat"], t["est_lon"]
    me = {"id": 3, "name": "ground station (alice)", "user": "alice", "crew": "captain" if captain else "copilot",
          "licence": licence, "single_op": single_op, "role": "HELM", "local": False, "rtt_ms": 80, "tmode": "full"}
    other = {"id": 4, "name": "ground station (bob)", "user": "bob", "crew": "copilot" if captain else "captain",
             "licence": True, "single_op": False, "role": "HELM", "local": True, "rtt_ms": 12, "tmode": "full"}
    ch = lambda n, act, up, inet, lat, loss: {"name": n, "label": n, "active": act, "up": up, "internet": inet,  # noqa
                                              "latency_ms": lat, "loss_pct": loss, "signal": None, "note": None,
                                              "metered": n == "STARLINK", "today_mb": 1.0, "month_mb": 20.0}
    ships = [{"mmsi": 237000000 + i, "name": n, "lat": la + 0.01 * (i + 1) * (1 if i % 2 else -1),
              "lon": lo + 0.012 * (i + 1), "cls": "A" if i % 3 else "B", "sog": 12.0, "cog": 200.0, "heading": 200,
              "age_s": 5 + i, "dist_nm": 0.6 * (i + 1), "stale": i == 9} for i, n in enumerate(SHIPS)]
    threats = [{"mmsi": 237000000, "name": SHIPS[0], "level": "danger", "cpa_nm": 0.08, "tcpa_min": 4.2,
                "bearing": 45.0}, {"mmsi": 237000001, "name": SHIPS[1], "level": "warn", "cpa_nm": 0.4,
                                   "tcpa_min": 9.0, "bearing": 120.0}]
    return {"you": 3, "stations": [me, other], "helm": {"name": "x"},
            "net": {"mode": "AUTO", "manual": None, "active": "4G", "fallback": False, "emergency": False,
                    "saver": False, "sync_allowed": True, "trial": None, "starlink": None,
                    "channels": [ch("4G", True, True, True, 85, 0), ch("4G2", False, True, True, 420, 3),
                                 ch("RF", False, True, False, None, None), ch("STARLINK", False, False, False, None, None)]},
            "avoid": {"state": "PROPOSE", "why": "", "threats": threats, "route": [], "shallow": None,
                      "shallow_limit_m": 3.0, "proposal": {"id": 1, "text": "BLUE STAR DELOS: CPA 0.08 nm in 4.2 "
                                                                            "min. Steer 230 and slow to 5 kn?"}},
            "ais": {"list": ships}, "geofence": {"poly": [[la - 0.01, lo - 0.01], [la + 0.02, lo - 0.01],
                                                          [la + 0.02, lo + 0.02]], "by": "bob"}}


class Rig:
    """The real screen 2 on the simulator (tcp), with fake gateway data on top."""

    def __init__(self):
        self.port = free_port()
        sim_dir = os.path.join(HERE, "..", "sim")
        self.sim = subprocess.Popen([os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim"),
                                     "server", str(self.port), "1"], cwd=sim_dir, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", self.port), timeout=0.2).close()
                break
            except OSError:
                time.sleep(0.1)
        self.root = tk.Tk()
        self.errors = []
        self.root.report_callback_exception = lambda *a: self.errors.append(repr(a[1]))
        self.ap = A.App(self.root, "tcp:127.0.0.1:%d" % self.port, layout=os.path.join(TMP, "layout.json"))
        self.root.state("normal")
        self.extra = {}
        self.pump(30, lambda: self.ap.t is not None)
        orig = self.ap.on_telem
        self.ap.on_telem = lambda d: orig(dict(d, **self.extra))

    def pump(self, s, until=None):
        end = time.time() + s
        while time.time() < end:
            self.root.update()
            if until and until():
                return True
            time.sleep(0.02)
        return bool(until and until())

    def gateway_like(self, on=True):
        """Make the tcp link look like a gateway link (panels only: nothing is sent through it)."""
        A.GatewayLink = A.Link if on else self.real_gw
        if on:
            self.ap.link.send_msg = lambda m: None
            self.ap.link.send = lambda *a, **k: 0           # commands (e.g. the boat file push, CLAUDE.md L) too
        else:
            self.ap.link.__dict__.pop("send", None)         # the real send again (the class's)

    real_gw = A.GatewayLink

    def size(self, W, H, dpi):
        self.root.tk.call("tk", "scaling", dpi * 96.0 / 72.0)
        self.root.geometry("%dx%d+0+0" % (W, H))
        self.pump(0.6, lambda: self.ap.cv.winfo_width() == W and self.ap.cv.winfo_height() == H)
        self.pump(0.3)
        self.ap.mapwin.redraw()
        self.ap.render(force=True)
        self.root.update()

    def close(self):
        try:
            self.ap.quit()
        except tk.TclError:
            pass
        self.sim.terminate()


def layout_problems(ap):
    """Everything the layout test checks, as a list of problems (empty = fine)."""
    ov = ap.ov
    W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
    win = (0, 0, W, H)
    probs = list(ov.problems)
    panels = {p.key: p.r for p in ov.panels}
    for op in ov.ops:
        r = op["r"]
        if not overlay.inside(r, win, tol=1.0):
            probs.append("%s %r clipped by the window" % (op["k"], op.get("s", "")[:30]))
        if op.get("panel") and op["panel"] in panels and not overlay.inside(r, panels[op["panel"]], tol=1.0):
            probs.append("%s %r sticks out of panel %s" % (op["k"], op.get("s", "")[:30], op["panel"]))
    sb = [o for o in ov.ops if o.get("tag") == "standby"]
    if len(sb) != 1:
        probs.append("STANDBY drawn %d times" % len(sb))
    else:
        for k, r in panels.items():
            if overlay.overlaps(sb[0]["r"], r):
                probs.append("panel %s over STANDBY" % k)
    ko = ap.mapwin.keepout(W, H)
    for k, r in panels.items():
        if overlay.rect_hits_circle(r, ko):
            probs.append("panel %s over the boat" % k)
    for o in ov.ops:
        if o["k"] == "banner" and overlay.rect_hits_circle(o["r"], ko):
            probs.append("banner %s over the boat" % o["tag"])
    lg = [o for o in ov.ops if o.get("tag") == "logo"]
    if len(lg) != 1:
        probs.append("logo drawn %d times" % len(lg))
    else:
        for o in ov.ops:
            if o is lg[0] or o["k"] in ("text", "cell", "badge") and o.get("panel"):
                continue
            if overlay.overlaps(lg[0]["r"], o["r"]):
                probs.append("the logo overlaps %s %s" % (o["k"], o.get("tag") or o.get("panel")))
    ks = list(panels)
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            if overlay.overlaps(panels[ks[i]], panels[ks[j]]):
                probs.append("panels %s and %s overlap" % (ks[i], ks[j]))
    return probs


def fill_everything(rig):
    ap = rig.ap
    d = ap.t
    rig.gateway_like(True)
    rig.extra.update(helm=3, profile=proto.PROFILE_UNCREWED, ll_state=1, ll_flags=0x20, near_shore=False,
                     coast_valid=True, coast_dist_m=450, ll_left_s=7.0, alarm_bits=0x1 | 0x40, alarm=True,
                     power_raised=True)
    ap.gw_status = fake_gw(d)
    la, lo = d["est_lat"], d["est_lon"]
    ap.prog["wps"] = [{"lat": la + 0.002 * i, "lon": lo + 0.001 * i, "kn": 8, "radius": 25, "action": 0}
                      for i in range(1, 7)]
    ap.route_dirty = True
    ap.maxkn_edit = 12
    now = time.time()
    for st in ("wait", "ok", "no"):
        ap.decisions.append({"what": A.TE("what.geofence", n=5), "seq": st, "state": st, "t": now, "why": ""})
    ap.say(A.T("cap.wait_captain", what=A.T("what.geofence", n=5)), A.ORANGE)
    rig.pump(0.8)


def s1_matrix(rig):
    ap = rig.ap
    fill_everything(rig)
    cases = []
    for lang in ("el", "en"):
        i18n.set_lang(lang, save=False)
        for W, H in ((1366, 768), (1366, 658), (1920, 1080), (1920, 970)):
            for dpi in (1.0, 1.25, 1.5):
                rig.size(W, H, dpi)
                p = layout_problems(ap)
                cases.append(((lang, W, H, dpi), ap.ov.k, len(ap.ov.panels), p))
    bad = [c for c in cases if c[3]]
    report("S1 layout 1366x768 / 1920x1080 (+ maximised) at 100 / 125 / 150 %, Greek + English, every panel full: "
           "nothing clipped, STANDBY visible, the boat free, the logo under no panel",
           not bad and all(c[2] == 6 for c in cases) and not rig.errors,
           "%d layouts checked (text size factors %s); problems: %s; errors %s" % (
               len(cases), sorted({round(c[1], 2) for c in cases}), [(c[0], c[3][:3]) for c in bad][:4],
               rig.errors[:2]))
    i18n.set_lang("el", save=False)


def s2_collapsed(rig):
    ap = rig.ap
    rig.size(1366, 768, 1.25)
    for p in list(ap.ov.panels):
        ap.ov.collapsed.add(p.key)
    ap.render(force=True)
    sb = [o for o in ap.ov.ops if o.get("tag") == "standby"]
    # the alarms panel is never collapsed (CLAUDE.md G): every other panel = its header only
    heads_only = all(o["k"] in ("panel", "text") for o in ap.ov.ops if o.get("panel") and o["panel"] != "alarms")
    alarms_stay = any(o.get("tag") == "alarm_row" for o in ap.ov.ops)
    sent = []
    orig = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))
    r = sb[0]["r"] if sb else (0, 0, 0, 0)
    ap.cv_press(type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})())
    ap.cv_release(type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})())
    ap.cmd = orig
    clicked = sent == [(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))]
    hd = [p for p in ap.ov.panels if p.key == "control"][0].r
    ap.cv_press(type("E", (), {"x": hd[0] + 20, "y": hd[1] + 5})())
    ap.cv_release(type("E", (), {"x": hd[0] + 20, "y": hd[1] + 5})())
    opened = "control" not in ap.ov.collapsed
    ap.ov.collapsed.clear()
    ap.render(force=True)
    report("S2 every panel collapsed to its header: STANDBY still visible and works; a header click opens the panel; "
           "the alarms panel stays open",
           len(sb) == 1 and heads_only and alarms_stay and clicked and opened and not layout_problems(ap) if sb
           else False,
           "STANDBY shown %d; only headers %s; alarm rows still drawn %s; STANDBY click sends %s; header click opens "
           "%s" % (len(sb), heads_only, alarms_stay, sent, opened))


def tags(ap):
    return {o.get("tag") for o in ap.ov.ops}


def s3_roles(rig):
    ap = rig.ap
    d = ap.t
    out = {}
    for name, cap, so, lic in (("captain", True, False, True), ("copilot", False, False, True),
                               ("single-op licensed", True, True, True), ("single-op no licence", True, True, False)):
        ap.gw_status = fake_gw(d, captain=cap, single_op=so, licence=lic)
        ap.single_op = so
        ap.render(force=True)
        out[name] = ("btn_manual" in tags(ap), "btn_maneuver" in tags(ap), ap.manual_allowed())
    ap.single_op = False
    ap.gw_status = fake_gw(d)
    ok = out["captain"] == (False, False, True) and out["copilot"] == (False, False, False) and \
        out["single-op licensed"] == (True, True, True) and out["single-op no licence"][:2] == (False, False)
    # screen 1: the buttons for the captain with the licence
    ap.open_driver()
    dv = ap.driver
    dv.w.geometry("1600x900+0+0")
    rig.pump(1.5)
    cap1 = dv.has("drv_remote_req") and dv.has("drv_maneuver_req")
    ap.gw_status = fake_gw(d, captain=False)
    rig.pump(0.6)
    cop1 = not dv.has("drv_remote_req") and not dv.has("drv_maneuver_req")
    ap.gw_status = fake_gw(d)
    report("S3 roles: screen 2 has no MANUAL / MANEUVER (captain or copilot) - only single-operator with the licence; "
           "screen 1 has REMOTE MANUAL / MANEUVER for the captain with the licence, not for the copilot",
           ok and cap1 and cop1, "screen 2 (manual, maneuver, screen-1 allowed): %s; screen 1 captain %s, copilot "
           "none %s" % (out, cap1, cop1))


def s4_maneuver(rig):
    ap = rig.ap
    dv = ap.driver
    rig.extra.update(mode=proto.MODE_MANEUVER, mode_name="MANEUVER", gear=1)
    rig.pump(1.0)
    lit = [o for o in dv.ops if o.get("tag") == "mv_gear"]
    on_txt = dv.texts("mv_on")
    rel = dv.has("drv_release")
    no_req = not dv.has("drv_remote_req")
    rig.extra.update(gear=2)
    rig.pump(0.8)
    rev = dv.texts("mv_on")
    # the box lit for FWD is the right-most of the three
    boxes = sorted([o["r"] for o in dv.ops if o.get("tag") == "mv_box"])
    rig.extra.update(mode=proto.MODE_STANDBY, mode_name="STANDBY", gear=0)
    rig.pump(0.6)
    gone = not dv.has("mv_box")
    report("S4 MANEUVER on screen 1: REV / N / FWD, the engaged gear lit (FWD, then REV), RELEASE instead of the "
           "request buttons; gone in other modes",
           len(lit) == 1 and on_txt == [A.T("mv.fwd")] and rev == [A.T("mv.rev")] and rel and no_req and
           len(boxes) == 3 and gone and not rig.errors,
           "lit %d, text %s then %s; boxes %d; RELEASE %s; requests hidden %s; gone after %s" % (
               len(lit), on_txt, rev, len(boxes), rel, no_req, gone))


def boat_poly(ap):
    c = ap.cv
    ids = [i for i in c.find_withtag("boat") if c.type(i) == "polygon"]
    return c.coords(ids[0]) if ids else None


def s5_map(rig):
    ap = rig.ap
    mw = ap.mapwin
    rig.extra.update(heading=90.0)
    rig.size(1366, 768, 1.0)
    rig.pump(0.8)
    mw.redraw()
    pts = boat_poly(ap)
    W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
    cx, cy = (pts[2] + pts[6]) / 2, (pts[3] + pts[7]) / 2 if pts else (0, 0)
    bx, by = mw.boat_xy()
    centred = abs(bx - W / 2) < 1.5 and abs(by - H / 2) < 1.5
    tip_east = pts is not None and pts[0] > bx + 10 and abs(pts[1] - by) < 3          # heading 090: points east
    rig.extra.update(heading=0.0)
    rig.pump(0.6)
    mw.redraw()
    pts0 = boat_poly(ap)
    tip_north = pts0 is not None and pts0[1] < by - 10 and abs(pts0[0] - bx) < 3
    north_up = mw.view is not None and len(mw.view) == 4                    # no rotation in the view at all
    m0 = mw.m
    mw.zoom(0.6)
    zoomed = abs(mw.m - m0 * 0.6) < 1e-6
    A.mapwin.PAN_BACK_S = 0.8
    mw.press(600, 400)
    mw.motion(500, 300)
    mw.release(500, 300)
    panned = not mw.follow
    ap.render(force=True)
    bar = [o["s"] for o in ap.ov.ops if o.get("tag") == "bar"]
    back_btn = A.T("ov.back_to_boat") in bar
    rig.pump(1.5)
    snapped = mw.follow
    mw.press(600, 400)
    mw.motion(500, 300)
    mw.release(500, 300)
    ap.render(force=True)
    btn = [o for o in ap.ov.ops if o.get("tag") == "bar" and o["s"] == A.T("ov.back_to_boat")]
    if btn:
        r = btn[0]["r"]
        ap.cv_press(type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})())
        ap.cv_release(type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})())
    back = mw.follow
    A.mapwin.PAN_BACK_S = 10.0
    report("S5 map: the boat in the centre, north-up, the symbol turns with the heading; zoom; a pan snaps back after "
           "the time and at once with 'back to boat'",
           centred and tip_east and tip_north and north_up and zoomed and panned and back_btn and snapped and back,
           "centre %s (%.0f, %.0f in %dx%d); heading 090 -> tip east %s, 000 -> north %s; zoom %s; pan %s (button %s), "
           "snapped back %s, button back %s" % (centred, bx, by, W, H, tip_east, tip_north, zoomed, panned, back_btn,
                                               snapped, back))


def click_map(ap, x, y, right=False):
    E = type("E", (), {"x": x, "y": y})
    if right:
        ap.mapwin.right(x, y)
        return
    ap.cv_press(E())
    ap.cv_release(E())


def s6_edit(rig):
    ap = rig.ap
    mw = ap.mapwin
    rig.gateway_like(False)
    ap.gw_status = None
    ap.prog["wps"] = []
    ap.route_dirty = False
    charts0 = ap.charts
    ap.charts = A.chart.Charts(os.path.join(TMP, "no_charts"))      # the simulator's sea has no chart here
    rig.size(1366, 768, 1.0)
    W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
    ap.set_edit("route")
    for x, y in ((W / 2 + 60, H / 2 - 120), (W / 2 + 120, H / 2 - 200), (W / 2 + 40, H / 2 - 260)):
        click_map(ap, x, y)
    added = len(ap.prog["wps"]) == 3 and ap.route_dirty
    P = mw.P()[0]
    x1, y1 = P(ap.prog["wps"][1]["lat"], ap.prog["wps"][1]["lon"])
    lat_before = ap.prog["wps"][1]["lat"]
    mw.press(x1, y1)
    mw.motion(x1 + 30, y1 + 30)
    mw.release(x1 + 30, y1 + 30)
    moved = ap.prog["wps"][1]["lat"] < lat_before and len(ap.prog["wps"]) == 3
    x2, y2 = P(ap.prog["wps"][2]["lat"], ap.prog["wps"][2]["lon"])
    click_map(ap, x2, y2, right=True)
    deleted = len(ap.prog["wps"]) == 2
    ap.render(force=True)
    not_sent = A.T("ov.not_sent").strip() in " ".join(o["s"] for o in ap.ov.ops if o.get("tag") == "route")
    ap.set_edit("route")
    ap.route_send()                                     # tcp = USB laptop: straight to the boat
    sent = rig.pump(4.0, lambda: not ap.route_dirty)
    on_boat = rig.pump(3.0, lambda: ap.t["prog_n"] == 2)
    # the geofence: corners on the map, NOT SENT until sent; only through the gateway
    ap.set_edit("fence")
    for x, y in ((W / 2 - 200, H / 2 - 200), (W / 2 + 200, H / 2 - 200), (W / 2 + 200, H / 2 + 200),
                 (W / 2 - 200, H / 2 + 200)):
        click_map(ap, x, y)
    corners = len(ap.fence_draft) == 4 and ap.fence_dirty
    ap.fence_send()
    only_gw = A.T("ov.fence_only_gw") in ap.msg_text and ap.fence_dirty
    ap.set_edit(None) if ap.mapwin.edit else None
    ap.mapwin.set_edit(None) if ap.mapwin.edit else None
    # max speed: -/+ then Set -> PARAM_SET max_leg_kn
    rig.pump(1.0, lambda: A.P_MAX_LEG in ap.params)
    cur = ap.params.get(A.P_MAX_LEG, {}).get("value")
    ap.maxkn_step(-1)
    ap.maxkn_step(-1)
    want = ap.maxkn_edit
    ap.maxkn_send()
    ap.read_params()
    set_ok = rig.pump(4.0, lambda: ap.params.get(A.P_MAX_LEG, {}).get("value") == want)
    ap.charts = charts0
    report("S6 editing on the map: route click / drag / right click, NOT SENT until sent, Send -> the boat has it; "
           "geofence corners (NOT SENT, only through the gateway); max speed - / + / Set",
           added and moved and deleted and not_sent and sent and on_boat and corners and only_gw and set_ok,
           "added %s, moved %s, deleted %s, 'not sent' shown %s; sent %s, boat has 2 waypoints %s; fence corners %s, "
           "refused without a gateway %s; max speed %s -> %s on the boat %s%s" % (
               added, moved, deleted, not_sent, sent, on_boat, corners, only_gw, cur, want, set_ok,
               "" if sent else "; messages %s" % ap.ev_texts()[-5:]))


def s7_link(rig):
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = fake_gw(ap.t)
    rig.extra.update(profile=proto.PROFILE_UNCREWED, ll_state=1, near_shore=False, coast_valid=True, coast_dist_m=450,
                     ll_left_s=7.0)
    rig.pump(0.6)
    ap.render(force=True)
    cells = [o["s"] for o in ap.ov.ops if o.get("tag") == "channels"]
    chans = all(any(n in c for c in cells) for n in ("4G", "RF", "STARLINK")) and "▮▮▮▮" in cells
    prof = [o["s"] for o in ap.ov.ops if o.get("tag") == "profile"]
    shore = [o["s"] for o in ap.ov.ops if o.get("tag") == "shore"]
    count = [o["s"] for o in ap.ov.ops if o.get("tag") == "lostlink"]
    rig.extra.update(near_shore=True, coast_dist_m=60)
    rig.pump(0.5)
    ap.render(force=True)
    near = [o["s"] for o in ap.ov.ops if o.get("tag") == "shore"]
    rig.extra.update(ll_state=2)
    rig.pump(0.5)
    ap.render(force=True)
    kill = [o["s"] for o in ap.ov.ops if o.get("tag") == "engine"]
    rig.extra.update(ll_state=0, profile=proto.PROFILE_CREWED, near_shore=False)
    report("S7 link panel: quality per channel (4G / RF / STARLINK), CREWED / UNCREWED, open sea / near shore with the "
           "distance, lost-link countdown, ENGINE KILLED",
           chans and prof == [A.T("prof.2")] and shore == [A.T("ov.open_sea", m="450")] and
           count == [A.T("ov.ll_count", s="7")] and near == [A.T("ov.near_shore", m="60")] and
           kill == [A.T("ov.engine_killed")],
           "channels %s; profile %s; shore %s -> %s; countdown %s; kill %s" % (cells[:10], prof, shore, near, count,
                                                                              kill))


def s8_boats(rig):
    """The top-right "boats" panel is gone (CLAUDE.md H): its content = the dropdown + lamp in the status panel."""
    ap = rig.ap
    rig.size(1366, 768, 1.25)
    no_panel = "boats" not in {p.key for p in ap.ov.panels}
    ents = [e[0] for e in ap.boat_menu_entries() if e is not None]
    names = [e.split("    ")[0] for e in ents[:len(ap.boat_list)]]
    ip_detail = all(e.endswith("· " + A.T("ov.ip_not_set")) and A.T("boat.st_no_ip") in e
                    for e in ents[:len(ap.boat_list)])
    extra = ents[len(ap.boat_list):] == [A.T("ov.other_address"), A.T("cap.disconnect")]
    ver0 = [o["s"] for o in ap.ov.ops if o.get("tag") == "verified"]
    ap.boat, ap.boat_ok = ap.boat_list[0], False
    ap.render(force=True)
    ver1 = [o["s"] for o in ap.ov.ops if o.get("tag") == "verified"]
    ap.boat_ok = True
    ap.render(force=True)
    ver2 = [o for o in ap.ov.ops if o.get("tag") == "verified"]
    dd = [o for o in ap.ov.ops if o.get("tag") == "boat_dd"]
    small = bool(ver2) and bool(dd) and (ver2[0]["r"][3] - ver2[0]["r"][1]) < (dd[0]["r"][3] - dd[0]["r"][1])
    ap.boat, ap.boat_ok = None, False
    ap.render(force=True)
    report("S8 boat selector: no 'boats' panel; the dropdown lists the boats of boats/ by name (ER69D, RF49D) with "
           "the status, the IP only as a detail; 'name verified' only after the confirm step, small",
           no_panel and names == ["ER69D", "RF49D"] and ip_detail and extra and not ver0 and
           ver1 == [A.T("ov.not_verified")] and [o["s"] for o in ver2] == [A.T("ov.verified")] and small,
           "boats panel gone %s; dropdown %s; IP as a detail %s; other address + disconnect %s; verified: none %s, "
           "connected %s, confirmed %s; smaller than the dropdown %s" % (
               no_panel, names, ip_detail, extra, ver0, ver1, [o["s"] for o in ver2], small))


LITERAL = re.compile(r'''["']#[0-9a-fA-F]{6}["']|["'](?:Segoe UI|Consolas|Arial|Helvetica)["']''')


def s9_look(rig):
    ap = rig.ap
    found = []
    for fn in ("app.py", "driver.py", "hud.py", "hudar.py", "mapwin.py", "overlay.py", "crewui.py", "soundtest.py"):
        for n, line in enumerate(open(os.path.join(HERE, fn), encoding="utf-8"), 1):
            if LITERAL.search(line):
                found.append("%s:%d" % (fn, n))
    dv = ap.driver
    t0 = theme.name()
    bg0, pal0 = ap.cv.cget("bg"), dv.hud.pal["panel"]
    theme.toggle()
    rig.pump(0.8)
    ap.mapwin.redraw()
    bg1, pal1 = ap.cv.cget("bg"), dv.hud.pal["panel"]
    tools_bg = ap.tools.cget("bg")
    theme.toggle()
    rig.pump(0.6)
    both = theme.name() == t0 and bg0 != bg1 and pal0 != pal1 and tools_bg == theme.THEMES["light"]["bg"]
    same = dv.hud.pal["warn"] == theme.WARN and dv.hud.pal["caution"] == theme.CAUTION and \
        A.RED == theme.WARN and A.ORANGE == theme.CAUTION
    logo1 = [o for o in dv.ops if o.get("tag") == "logo"]
    over = []
    if logo1:
        lr = logo1[0]["r"]
        W, H = dv.cv.winfo_width(), dv.cv.winfo_height()
        for name in dv.hud.layout.rects:
            if name in dv.hud.layout.hidden:
                continue
            if overlay.overlaps(lr, dv.layout.px(name, W, H)):
                over.append(name)
        inside_w = overlay.inside(lr, (0, 0, W, H))
    logo2 = [o for o in ap.ov.ops if o.get("tag") == "logo"]
    have_file = theme.logo_path() is not None
    report("S9 one look: no colour / font written in the screen modules (theme.py); dark / light switches both screens "
           "and the tools window; warning / caution colours the same on both; the logo on both screens, over nothing",
           not found and both and same and len(logo1) == 1 and not over and inside_w and len(logo2) == 1 and have_file,
           "literals %s; theme %s -> map bg %s -> %s, HUD panels %s -> %s, tools %s; meaning colours same %s; logo "
           "screen 1 %d (overlaps %s), screen 2 %d; file %s" % (
               found[:5], t0, bg0, bg1, pal0, pal1, tools_bg, same, len(logo1), over, len(logo2), theme.logo_path()))


def s10_tools(rig):
    ap = rig.ap
    import tkinter.font as tkf
    res = {}
    for lang in ("el", "en"):
        i18n.set_lang(lang, save=False)
        rig.root.tk.call("tk", "scaling", 1.5 * 96 / 72)
        ap.tools.deiconify()
        ap.tools_fit()
        rig.pump(0.5)
        cut = []
        for b in ap.nb.btns:
            f = tkf.Font(root=b, font=b.cget("font"))
            if b.winfo_width() < f.measure(b.cget("text")) + 10:
                cut.append(b.cget("text"))
        heads = []
        for tree in (ap.ptree, ap.ais_tree, ap.pg_tree):
            f = tkf.Font(root=tree, font=theme.font("button"))
            for c in ("#0",) + tuple(tree["columns"]):
                h = tree.heading(c, "text")
                if h and int(tree.column(c, "width")) < f.measure(h) + 8:
                    heads.append(h)
        res[lang] = (cut, heads)
        ap.tools.withdraw()
    i18n.set_lang("el", save=False)
    rig.root.tk.call("tk", "scaling", 96 / 72)
    report("S10 the tools pages at 150 %: no page name cut off, every table heading fits its column (Greek + English)",
           all(not c and not h for c, h in res.values()), "cut page names / headings: %s" % res)


LATIN = re.compile(r"[A-Za-z][A-Za-z\-]{2,}")
ALLOWED = {"STANDBY", "AUTO", "GPS", "AIS", "CPA", "TCPA", "SOG", "COG", "USB", "IMU", "RF", "STARLINK", "CONFIRM",
           "SAVE", "ONLINE", "OFFLINE", "SIM", "UBX", "NMEA", "MMSI", "Tailscale", "gateway", "EMODnet", "GEBCO",
           "OpenSeaMap", "OpenStreetMap", "ODbL", "CC", "Bathymetry", "Consortium", "Digital", "DTM", "contributors",
           "Sea", "marks", "Depth", "tcp", "COM", "Trygons", "TRYGONS", "DRONES", "MediaMTX", "ffmpeg", "Piper",
           "ACK", "HDG", "LOCK", "CREWED", "UNCREWED", "FWD", "REV", "BENCH", "TEST", "FIRST", "MANEUVER", "HELM",
           "TAKE", "SAVE", "eWaSR", "onnx", "ONNX", "Thrustmaster", "TWCS", "Teensy", "TELEM", "FAT", "BNO", "UTC",
           "FRONT", "REAR", "LEFT", "RIGHT", "SIM", "CRC", "LTE", "SIM", "kn", "nm", "Hz", "fps", "kbps", "ms", "km",
           "dBm", "GB", "MB", "KB", "AR", "CDI", "ROT", "ETA", "TTG", "SD", "OK", "PC", "ADS", "EL", "EN", "ID", "IP",
           "Ctrl", "Shift", "Tab", "Esc", "Space", "x", "v", "on", "off", "lat", "lon", "WP", "Tools", "AURIX",
           "English", "png", "json", "txt", "tlg", "TLG", "CSV", "csv", "Not", "for", "navigation", "free", "data",
           "may", "be", "inaccurate", "near", "the", "coast", "GATEWAY", "FRONT-", "UART", "kHz", "python", "PWM",
           "CPU", "alice", "bob"}


def english_words(text):
    """Latin words of a screen text that are not names / units / technical tokens."""
    t = (text or "").replace("\\n", " ")
    t = " ".join(w for w in t.split() if not any(ch in w for ch in "./\\_<>"))      # files, paths, config keys
    return [w for w in LATIN.findall(t) if w not in ALLOWED and not re.fullmatch(r"[A-Z]{1,3}\d*|[A-Z]+\d+[A-Z]*", w)]


def texts_of(widget):
    out = []
    stack = [widget]
    while stack:
        w = stack.pop()
        try:
            cls = w.winfo_class()
            if cls in ("Label", "Button", "Checkbutton", "Radiobutton", "Message"):
                out.append(str(w.cget("text")))
            elif cls == "Canvas":
                out += [str(w.itemcget(i, "text")) for i in w.find_all() if w.type(i) == "text"]
            elif cls == "Listbox":
                out += [str(w.get(i)) for i in range(w.size())]
            elif cls == "Treeview":
                for c in ("#0",) + tuple(w["columns"]):
                    out.append(str(w.heading(c, "text")))
            stack.extend(w.winfo_children())
        except tk.TclError:
            continue
    return out


def s11_greek(rig):
    ap = rig.ap
    i18n.set_lang("el", save=False)
    fill_everything(rig)
    rig.size(1920, 1080, 1.0)
    ap.render(force=True)
    texts = [o.get("s", "") for o in ap.ov.ops] + texts_of(ap.root) + texts_of(ap.tools)
    texts += [op["s"] for op in ap.driver.ops if op.get("k") == "text"]
    rig.pump(0.3)
    name_words = {w for n in SHIPS for w in n.split()}          # the ships' own AIS names stay as broadcast
    try:
        import tts                                              # installed voices keep their product names
        for lg in i18n.LANGS:
            name_words |= set((tts.describe(lg) or "").replace("-", " ").split())
    except Exception:
        pass
    english = {}
    for t in texts:
        for w in english_words(t):
            if w in name_words or any(n.startswith(w) for n in name_words):
                continue
            english.setdefault(w, t[:60])
    # every Greek text in the file, also the ones this run did not show
    lang_words = {}
    for ln in open(os.path.join(HERE, "lang", "el.txt"), encoding="utf-8"):
        if ln.startswith("#") or " = " not in ln:
            continue
        k, v = ln.rstrip("\n").split(" = ", 1)
        for w in english_words(re.sub(r"\{\w+\}", "", v)):
            lang_words.setdefault(w, k)
    report("S11 the Greek screen has no English words left (panels, banners, tools pages, screen 1, every entry of "
           "el.txt)", not english and not lang_words,
           "%d texts checked; English on the screen: %s; in el.txt: %s" % (
               len(texts), sorted(english.items())[:20], sorted(lang_words.items())[:20]))


def s12_alarms(rig):
    """CLAUDE.md G: the alarms panel = the top-right corner, always drawn, never into the keep-out circle; one
    row per alarm, most severe first; unacknowledged rows flash on the MASTER lamp clock (fill / outline
    alternates, the text stays), acknowledged = steady; no alarm banner any more; the moved panels fit."""
    ap = rig.ap
    import alerts
    rig.size(1366, 768, 1.25)
    rig.extra.update(alarm_bits=0x001 | 0x002 | 0x020, alarm=True)      # compass lost (W), GPS lost (C), arrived (A)
    ap.lamps.acked = {alerts.WARNING: 0, alerts.CAUTION: 0}             # nothing acknowledged on this PC
    rig.pump(1.2)                                                       # alarm_tick: the lamps know the bits
    orig_flash = alerts.flash
    ops = {}
    for ph in (True, False):
        alerts.flash = lambda now=None, ph=ph: ph
        ap.render(force=True)
        ops[ph] = [dict(o) for o in ap.ov.ops if o.get("tag") in ("alarm_row", "alarm_more")]
    pn = {p.key: p for p in ap.ov.panels}
    al = pn.get("alarms")
    ne = [p for p in ap.ov.panels if p.corner == "ne"]
    topmost = al is not None and all(p.r[1] >= al.r[3] for p in ne if p is not al)
    right = al is not None and al.r[2] >= ap.cv.winfo_width() - 40
    n_rows = len([o for o in ops[True] if o["tag"] == "alarm_row"])
    texts = [o["s"] for o in ops[True] if o["tag"] == "alarm_row"]
    order_ok = texts == [A.T("alarm.compass_lost"), A.T("alarm.gps_lost"), A.T("alarm.arrived")]
    fills_on = [o.get("fill") for o in ops[True] if o["k"] == "badge"]
    fills_off = [o.get("fill") for o in ops[False] if o["k"] == "badge"]
    flashing = fills_on == [A.theme.WARN, A.theme.CAUTION] and fills_off == ["", ""] and \
        [o["s"] for o in ops[True]] == [o["s"] for o in ops[False]]
    advisory_steady = [o["k"] for o in ops[True] if o["s"] == A.T("alarm.arrived")] == ["text"]
    ap.lamps.ack()
    for ph in (True, False):
        alerts.flash = lambda now=None, ph=ph: ph
        ap.render(force=True)
        ops[ph] = [dict(o) for o in ap.ov.ops if o.get("tag") == "alarm_row"]
    steady = ops[True] == ops[False] and all(o["k"] == "text" for o in ops[True]) and len(ops[True]) == 3
    alerts.flash = orig_flash
    no_banner = not ap.banner_on("alarm")
    probs = layout_problems(ap)
    # ACK ALARM in the panel sends the boat's alarm acknowledge
    sent = []
    orig = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))
    ack = [o for o in ap.ov.ops if o.get("tag") == "ack"]
    if ack:
        r = ack[0]["r"]
        ap.cv_press(type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})())
        ap.cv_release(type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})())
    ap.cmd = orig
    ack_ok = sent == [(proto.T_ALARM_ACK, b"")]
    # every panel collapsed: the alarms panel is still there with its rows
    for p in list(ap.ov.panels):
        ap.ov.collapsed.add(p.key)
    ap.render(force=True)
    still = any(o.get("tag") == "alarm_row" for o in ap.ov.ops) and not al.collapsible
    ap.ov.collapsed.clear()
    # no alarm: a one-line header only
    rig.extra.update(alarm_bits=0, alarm=False)
    rig.pump(1.2)
    ap.render(force=True)
    pn = {p.key: p for p in ap.ov.panels}
    header_only = "alarms" in pn and not any(o.get("panel") == "alarms" and o["k"] not in ("panel", "text")
                                             for o in ap.ov.ops) and pn["alarms"].title == A.T("ov.no_alarms")
    report("S12 alarms panel top-right: always drawn (also collapsed; no alarm = a header), outside the boat circle, "
           "one row per alarm most severe first, unacknowledged rows flash on the lamp clock (text stays), "
           "acknowledged / advisory steady, ACK ALARM sends the boat's ack, no alarm banner, the moved panels fit",
           topmost and right and n_rows == 3 and order_ok and flashing and advisory_steady and steady and no_banner
           and not probs and ack_ok and still and header_only and not rig.errors,
           "topmost %s right %s; rows %d %s; fills lit %s / dark %s; advisory steady %s; after ack steady %s; banner "
           "gone %s; layout problems %s; ack sends %s; collapsed still drawn %s; empty = header %s; errors %s" % (
               topmost, right, n_rows, texts, fills_on, fills_off, advisory_steady, steady, no_banner, probs[:3],
               sent, still, header_only, rig.errors[:2]))


def s13_boat_line(rig):
    """CLAUDE.md H: the status panel's FIRST line = the boat dropdown + the lamp; CONNECTED (green) only
    when the boat's gateway was found by the prober AND the station is connected AND the welcome name
    check passed, otherwise grey; choosing a boat opens the login (then the confirm step) with that boat
    selected; the line fits in the S1 matrix."""
    ap = rig.ap
    import boats
    rig.size(1366, 768, 1.25)

    def line():
        ap.render(force=True)
        st = [o for o in ap.ov.ops if o.get("panel") == "status" and o["k"] != "panel"]
        dd = [o for o in st if o.get("tag") == "boat_dd"]
        lamp = [o for o in st if o.get("tag") == "boat_lamp"]
        # the first line: nothing of the panel's content above or beside it (the header is above it)
        first = bool(dd) and all(o["r"][1] >= dd[0]["r"][3] - 1.0 for o in st
                                 if o.get("tag") not in ("boat_dd", "boat_lamp", "verified") and
                                 o["r"][3] > dd[0]["r"][1] + 1.0)
        same = bool(lamp) and bool(dd) and lamp[0]["r"][0] >= dd[0]["r"][2] - 1.0 and \
            lamp[0]["r"][1] < dd[0]["r"][3] and lamp[0]["r"][3] > dd[0]["r"][1]
        return first, same, (dd[0]["s"] if dd else ""), ((lamp[0]["s"], lamp[0].get("fill")) if lamp else None)
    out = {}
    out["address"] = line()                                   # tcp to the simulator: no boat check
    ap.boat_pick = ap.boat_list[-1]                            # RF49D: placeholder IP
    out["no_ip"] = line()
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(5)
    bt = boats.Boat("t.json", {"id": "tt", "name": "TESTBOAT", "tailscale_ip": "127.0.0.1",
                               "gateway_port": srv.getsockname()[1]})
    off = boats.Boat("o.json", {"id": "oo", "name": "OFFBOAT", "tailscale_ip": "127.0.0.1",
                                "gateway_port": free_port()})
    ap.prober.result["oo"] = ("offline", time.time())
    ap.boat_pick = off
    out["offline"] = line()
    ap.prober.result.pop("oo", None)
    ap.boat_pick = bt
    out["checking"] = line()
    ap.prober.found(bt)
    out["online"] = line()                                    # found, not connected to it
    had = hasattr(ap.link, "welcome")
    ap.boat, ap.boat_ok = bt, False
    ap.link.welcome = {"boat": "OTHER"}
    out["wrong_name"] = line()                                # the welcome name check fails: grey
    ap.link.welcome = {"boat": "TESTBOAT"}
    out["connected"] = line()
    ap.link.connected = False
    out["link_down"] = line()
    ap.link.connected = True
    G, grey = A.theme.GOOD, A.theme.C["head"]
    lit = {k: v[3] for k, v in out.items()}
    ok_states = lit["address"] == ("● " + A.T("ov.no_boat_check"), grey) and \
        lit["no_ip"] == ("● " + A.T("boat.st_no_ip"), grey) and lit["offline"] == ("● " + A.T("boat.st_offline"), grey) \
        and lit["checking"] == ("● " + A.T("boat.st_checking"), grey) and \
        lit["online"] == ("● " + A.T("boat.st_online"), grey) and lit["wrong_name"] == lit["online"] and \
        lit["connected"] == ("● " + A.T("ov.connected"), G) and lit["link_down"] == lit["online"] and \
        out["no_ip"][2].startswith("RF49D") and out["connected"][2].startswith("TESTBOAT") and \
        out["address"][2].startswith(ap.port_var.get())
    first_ok = all(v[0] and v[1] for v in out.values())
    # the S1 matrix with the widest line (connected + "name verified"): still fits, the line still first
    cases = []
    for lang in ("el", "en"):
        i18n.set_lang(lang, save=False)
        for W, H in ((1366, 768), (1366, 658), (1920, 1080), (1920, 970)):
            for dpi in (1.0, 1.25, 1.5):
                rig.size(W, H, dpi)
                f = line()
                cases.append(((lang, W, H, dpi), f[0] and f[1] and "verified" in tags(ap), layout_problems(ap)))
    i18n.set_lang("el", save=False)
    bad = [c for c in cases if not c[1] or c[2]]
    # choosing a boat in the dropdown: the login opens with that boat selected (then the confirm step, test_boats B4)
    m = ap.boat_dropdown(post=False)
    labels = [m.entrycget(i, "label") for i in range(m.index("end") + 1) if m.type(i) == "command"]
    m.invoke(1)                                               # the second boat of the list
    rig.pump(0.3)
    sel = ap.boatsel.selected()[0] if ap.boatsel else None
    chosen = sel is not None and sel.id == ap.boat_list[1].id and ap.boat_pick is ap.boat_list[1]
    if ap.boatsel:
        ap.boatsel.close()
    ap.boat, ap.boat_ok, ap.boat_pick = None, False, None
    if not had:
        del ap.link.welcome
    srv.close()
    ap.render(force=True)
    report("S13 boat dropdown = the status panel's first line + the lamp: CONNECTED (green) only when found + "
           "connected + name check passed, else grey (offline / IP not set / checking / online / no boat check); "
           "choosing opens the login with the boat selected; fits the S1 matrix",
           ok_states and first_ok and not bad and chosen and len(labels) >= len(ap.boat_list) and not rig.errors,
           "lamp %s; first line + lamp beside it %s; matrix %d layouts, bad %s; menu %s; chosen -> login with %s; "
           "errors %s" % (lit, {k: v[:2] for k, v in out.items() if not (v[0] and v[1])} or "all",
                          len(cases), [(c[0], c[2][:2]) for c in bad][:3], labels, sel, rig.errors[:2]))


def main():
    exe = os.path.join(HERE, "..", "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    rig = Rig()
    try:
        for fn in (s1_matrix, s2_collapsed, s3_roles, s4_maneuver, s5_map, s6_edit, s7_link, s8_boats, s9_look,
                   s10_tools, s11_greek, s12_alarms, s13_boat_line):
            try:
                fn(rig)
            except Exception as e:
                import traceback
                traceback.print_exc()
                report(fn.__name__, False, "crashed: %r" % e)
    finally:
        rig.gateway_like(False)
        rig.close()
    bad = results.count(False)
    print("SCREEN 2 TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
