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
     ONLINE / OFFLINE / IP not set, the IP only as a detail; "name verified" only after the name
     check (13n2: automatic at connect), small
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
  S15 DP (CLAUDE.md P, step 11): the DP rows of the control panel (only with both thrusters fitted or in DP),
      SPOT / CRUISE / TRIP / STOP + speed, JOG arrows + the heading buttons in DP, "continue", CRUISE speed SET,
      the status panel's DP line, off spot colour, copilot (start asked, STOP free), the S1 matrix in DP, Greek
  S14 relay buttons (CLAUDE.md O): the "relays" panel (boat file relays by group, placed by the fit
      test - the S1 matrix includes it), an interlocked pair on one line, the engine line with START
      (hold) + RESTART, lit = on / grey = unknown from the readback, momentary = press-and-hold (pulse,
      OFF at release), START hold = CRANK / RELEASE, nothing while the table is not active / pending,
      collapsible; a copilot's trim / extra relay is not asked, an engine relay waits for the captain
  S16 silence on this PC (step 13m): screen-2 ACK ALARM / Space / A = the MASTER acknowledge here, the
      fallback beep only while MASTER WARNING is lit, a newer warning sounds again, disconnect = no old
      telemetry / alarm rows / sound, reconnect = fresh alarms only
  S19 shore stick override (step 13q3): MANUAL STEERING - AUTO / HDG LOCK waiting, NEW HEADING, the lever
      ceiling, the AUTO data-lost stop on screens 2 + 1 (UNCREWED only)
  S21 mode bar (step 13n1, owner 2026-10-31 E1): top centre, always drawn, the 7 modes + BATTLE SHORT, the
      active one inverted, STANDBY the only red button, greyed = says why + sends nothing, MANUAL never from the PC
  S22 the two bottom bars: their items, a sub-panel above bar 1, hide to a handle, dock top / bottom
  S23 panels draggable (saved per PC, never over the boat), RESET TO DEFAULT
  S24 BATTLE SHORT: opens from the mode bar, every code INVALID, nothing sent, the log line, the tooltip
  S25 camera view (step 13n2, owner 2026-10-31 E2): CAMERA on bar 2 (gateway only) -> the camera full screen on
      screen 2 (the gateway asked for it, the lighter decoding for a copilot), the four cameras + MAP on bar 1, the
      green HUD basics, the mode bar / STANDBY / alarms stay, no map editing under it, "reduced for the captain" /
      "no video" banners, MAP = back (the stream closed); the "no video" warning of a dead front camera
  S26 track (step 13n3, owner 2026-10-31 E3): TRACKS / NEW TRACK on bar 2, the live track (every point, white / black
      by the background), a click on the line = time + speed there, NEW TRACK = YES / NO then <date>_<HHMM>_<boat> + a
      new line, one track per boat, the TRACKS list (show grey dashed + date / hide / remove - the file stays / GPX
      export + import, a bad file refused), the screen 1 mini map, nothing sent to the boat
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
os.environ["TRYGONS_SCREENREC"] = os.path.join(TMP, "screenrec")     # step 13n4: never this PC's real recordings
os.environ.pop("TRYGONS_LANG", None)
import i18n  # noqa: E402
import theme  # noqa: E402
import overlay  # noqa: E402
import canvaspic  # noqa: E402
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
        # a posted menu / a question box blocks the Tk loop until a human answers (on Windows): in the tests that is
        # a click that hit the wrong place - record it as an error instead of hanging the gate (seen once at step 11)
        from tkinter import messagebox, simpledialog
        tk.Menu.tk_popup = lambda m, *a, **k: self.errors.append("a menu was posted (blocking)")
        for mod, names in ((messagebox, ("askyesno", "askokcancel", "showinfo", "showwarning", "showerror")),
                           (simpledialog, ("askstring", "askinteger", "askfloat"))):
            for n in names:
                setattr(mod, n, lambda *a, _n=n, **k: self.errors.append("dialog %s opened (blocking)" % _n))
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
            # (no ACK ever comes: the DP thruster push would repeat every 15 s and leave its line in the event log in
            # whatever language was on at that moment - S11 then sees an English line from the S1 English pass)
            self.ap.thr_push_at = time.time() + 1e9
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


RELAY_EXTRA = ("relay_table", "relay_hash", "relay_pending", "relay_ok", "relay_on", "relay_unknown", "eng_crank",
               "relay_stop")


def relay_boat(rig, on=(1, 6), unknown=(), pending=False, hash_ok=True, stop=False):
    """The boat RF49D of boats/ (7 relays, format 2) with its relay table active on the boat (TELEM2 group M)."""
    import boats
    ap = rig.ap
    ap.boat = [b for b in boats.load() if b.id == "rf49d"][0]
    ap.boat_ok = True
    h = proto.relay_table_hash(ap.boat.relays)
    rig.extra.update(relay_table=True, relay_hash=h if hash_ok else h ^ 1, relay_pending=pending, relay_ok=True,
                     relay_on=list(on), relay_unknown=list(unknown), eng_crank=0, relay_stop=stop)
    rig.pump(0.4)
    return h


def no_relay_boat(rig):
    rig.ap.boat, rig.ap.boat_ok = None, False
    rig.ap.sub_open = None                              # (fill_everything opened bar 1's ROUTE sub-panel)
    for k in RELAY_EXTRA:
        rig.extra.pop(k, None)
    rig.pump(0.3)


def fill_everything(rig):
    ap = rig.ap
    d = ap.t
    rig.gateway_like(True)
    relay_boat(rig)                                     # the relays panel too (CLAUDE.md O)
    rig.extra.update(helm=3, profile=proto.PROFILE_UNCREWED, ll_state=1, ll_flags=0x20, near_shore=False,
                     coast_valid=True, coast_dist_m=450, ll_left_s=7.0, alarm_bits=0x1 | 0x40, alarm=True,
                     power_raised=True)
    ap.gw_status = fake_gw(d)
    la, lo = d["est_lat"], d["est_lon"]
    ap.prog["wps"] = [{"lat": la + 0.002 * i, "lon": lo + 0.001 * i, "kn": 8, "radius": 25, "action": 0}
                      for i in range(1, 7)]
    ap.route_dirty = True
    ap.maxkn_edit = 12
    ap.sub_open = "route"                               # bar 1's ROUTE sub-panel open too (13n1)
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
           not bad and all(c[2] == 8 for c in cases) and not rig.errors,
           "%d layouts checked (text size factors %s); problems: %s; errors %s" % (
               len(cases), sorted({round(c[1], 2) for c in cases}), [(c[0], c[3][:3]) for c in bad][:4],
               rig.errors[:2]))
    i18n.set_lang("el", save=False)
    rig.size(1366, 768, 1.25)
    relays = [p for p in ap.ov.panels if p.key == "relays"]
    s1_matrix.placed = relays[0].placed if relays else None
    no_relay_boat(rig)


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
        en = {o["tag"]: o.get("enabled") for o in ap.ov.ops if o.get("k") == "button"}
        out[name] = (bool(en.get("mode_REMOTE_MANUAL")), bool(en.get("mode_MANEUVER")), ap.manual_allowed())
    ap.single_op = False
    ap.gw_status = fake_gw(d)
    # step 13n1 (owner 2026-10-31 C2 / E1): REMOTE / MANEUVER are on screen 2's mode bar too - pressable for the
    # captain with the licence (or a licensed single operator), greyed for a copilot / without the licence
    ok = out["captain"] == (True, True, True) and out["copilot"] == (False, False, False) and \
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
    report("S3 roles: the mode bar's REMOTE / MANEUVER pressable for the captain with the licence (or a licensed "
           "single operator), greyed for the copilot / without the licence; screen 1 has REMOTE MANUAL / MANEUVER for "
           "the captain with the licence, not for the copilot",
           ok and cap1 and cop1, "screen 2 (manual, maneuver, screen-1 allowed): %s; screen 1 captain %s, copilot "
           "none %s" % (out, cap1, cop1))


def s4_maneuver(rig):
    ap = rig.ap
    dv = ap.driver
    # step 13q5 (owner 2026-10-31 C5): the outlined box = the AURIX's gear ask from the stick Y (TELEM2 D u16 in
    # MANEUVER: 0x8000 | ask | held zone << 4 -> proto mv_stick_ask / mv_stick_hold)
    rig.extra.update(mode=proto.MODE_MANEUVER, mode_name="MANEUVER", gear=1, mv_stick_ask=1, mv_stick_hold=1,
                     rjoy_age_ms=40)
    rig.pump(1.0)
    lit = [o for o in dv.ops if o.get("tag") == "mv_gear"]
    ask_f = sorted([(o["r"][0], o["w"]) for o in dv.ops if o.get("tag") == "mv_box"])
    on_txt = dv.texts("mv_on")
    # step 13q5b (review MED-1): the Remote tools page shows the stick's forward / back the station SENDS (+ = forward)
    # and the boat's own view of it in MANEUVER (full forward held -> asks F)
    import types
    ap.rsender = types.SimpleNamespace(sent=42, missing=False, last={"x": 0.0, "lever": 0.5, "y": 1.0})
    rig.pump(0.6)
    rtxt = ap.remote_lbl.cget("text")
    ap.rsender = None
    y_ok = (A.T("cap.rm_sending_y", y="+1.00") in rtxt and
            A.T("cap.rm_mv_sees", zone=A.T("cap.rm_mv_zone_1"), gear=A.T("gear.1")) in rtxt)
    shots = os.environ.get("TRYGONS_SHOT_DIR")
    if shots:
        ap.rsender = types.SimpleNamespace(sent=42, missing=False, last={"x": 0.0, "lever": 0.5, "y": 1.0})
    if shots and dv.last_frame is not None:             # step 13q2: MANEUVER lit, REMOTE MANUAL beside it, no RELEASE
        dv.last_frame.save(os.path.join(shots, "driver_maneuver_el.png"))
    if shots:                                           # the Remote tools page: no RELEASE, the new help text
        ap.open_tab("tab.remote")
        rig.pump(0.6)
        shot(ap.tools, os.path.join(shots, "tools_remote_el.png"))
        ap.tools.withdraw()
        ap.rsender = None
    rel = not dv.has("drv_release")                     # step 13q2: there is no RELEASE
    no_req = dv.has("drv_remote_req") and dv.has("drv_maneuver_req")   # MANEUVER lit, REMOTE = a straight switch
    rig.extra.update(gear=2, mv_stick_ask=0, mv_stick_hold=2)       # let go = N asked, ... a full back being held
    rig.pump(0.8)
    rev = dv.texts("mv_on")
    ask_n = sorted([(o["r"][0], o["w"]) for o in dv.ops if o.get("tag") == "mv_box"])
    ask_ok = ([w for _x, w in ask_f] == [1, 1, 4] and [w for _x, w in ask_n] == [2, 4, 1])   # REV | N | FWD
    # the box lit for FWD is the right-most of the three
    boxes = sorted([o["r"] for o in dv.ops if o.get("tag") == "mv_box"])
    rig.extra.update(mode=proto.MODE_STANDBY, mode_name="STANDBY", gear=0)
    for k in ("mv_stick_ask", "mv_stick_hold", "rjoy_age_ms"):
        rig.extra.pop(k, None)
    rig.pump(0.6)
    gone = not dv.has("mv_box")
    report("S4 MANEUVER on screen 1: REV / N / FWD, the engaged gear lit (FWD, then REV), the stick's ask outlined (13q5), "
           "no RELEASE (13q2): MANEUVER "
           "and REMOTE MANUAL stay (the other one = a straight switch); gone in other modes",
           len(lit) == 1 and on_txt == [A.T("mv.fwd")] and rev == [A.T("mv.rev")] and rel and no_req and
           len(boxes) == 3 and gone and ask_ok and y_ok and not rig.errors,
           "lit %d, text %s then %s; boxes %d; no RELEASE %s; both drive buttons %s; gone after %s; stick ask outlines "
           "(13q5) F %s, N + REV held %s; Remote page stick Y (13q5b) %s: %r" % (
               len(lit), on_txt, rev, len(boxes), rel, no_req, gone, ask_f, ask_n, y_ok, rtxt[-260:]))


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
           "the status, the IP only as a detail; 'name verified' only after the name check, small",
           no_panel and names == ["ER69D", "RF49D"] and ip_detail and extra and not ver0 and
           ver1 == [A.T("ov.not_verified")] and [o["s"] for o in ver2] == [A.T("ov.verified")] and small,
           "boats panel gone %s; dropdown %s; IP as a detail %s; other address + disconnect %s; verified: none %s, "
           "connected %s, confirmed %s; smaller than the dropdown %s" % (
               no_panel, names, ip_detail, extra, ver0, ver1, [o["s"] for o in ver2], small))


LITERAL = re.compile(r'''["']#[0-9a-fA-F]{6}["']|["'](?:Segoe UI|Consolas|Arial|Helvetica)["']''')


def s9_look(rig):
    ap = rig.ap
    found = []
    for fn in ("app.py", "driver.py", "hud.py", "hudar.py", "mapwin.py", "overlay.py", "crewui.py", "soundtest.py",
               "splash.py", "about.py", "trackui.py", "track.py"):
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
           "CPU", "alice", "bob", "DroneCAN", "Drones", "BATTLE", "SHORT", "Google", "Earth", "OpenCPN"}  # (13n3 GPX)


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
    no_relay_boat(rig)


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
    """CLAUDE.md H: the status panel's FIRST line = the boat dropdown + the lamp; CONNECTED (white, 13n1) only
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
    G, grey = A.theme.CONNECTED[0], A.theme.C["head"]     # the "connected" lamp is WHITE (13n1)
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
    report("S13 boat dropdown = the status panel's first line + the lamp: CONNECTED (white, 13n1) only when found + "
           "connected + name check passed, else grey (offline / IP not set / checking / online / no boat check); "
           "choosing opens the login with the boat selected; fits the S1 matrix",
           ok_states and first_ok and not bad and chosen and len(labels) >= len(ap.boat_list) and not rig.errors,
           "lamp %s; first line + lamp beside it %s; matrix %d layouts, bad %s; menu %s; chosen -> login with %s; "
           "errors %s" % (lit, {k: v[:2] for k, v in out.items() if not (v[0] and v[1])} or "all",
                          len(cases), [(c[0], c[2][:2]) for c in bad][:3], labels, sel, rig.errors[:2]))


def s14_relays(rig):
    """CLAUDE.md O: the relays panel on screen 2 (DISPLAY + requests only)."""
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = fake_gw(ap.t)
    h = relay_boat(rig, on=(1, 6), unknown=(7,))
    rig.size(1366, 768, 1.25)
    ops = lambda: [o for o in ap.ov.ops if o.get("panel") == "relays"]          # noqa: E731
    btn = {o["tag"]: o for o in ops() if o["k"] == "button"}
    have = all("relay_%d" % n in btn for n in range(1, 8)) and "relay_restart" in btn
    cy = lambda t: (btn[t]["r"][1] + btn[t]["r"][3]) / 2 if t in btn else -1    # noqa: E731
    pair = have and abs(cy("relay_4") - cy("relay_5")) < 1.0 and abs(cy("relay_4") - cy("relay_1")) > 1.0
    engine = have and abs(cy("relay_1") - cy("relay_2")) < 1.0 and abs(cy("relay_1") - cy("relay_3")) < 1.0 and \
        cy("relay_restart") > cy("relay_1")
    states = have and btn["relay_1"]["on"] and btn["relay_6"]["on"] and not btn["relay_3"]["on"] and \
        not btn["relay_7"]["enabled"] and btn["relay_6"]["enabled"]
    groups = [o["s"] for o in ops() if o.get("tag") == "relay_grp" and o["s"]]
    grp_ok = groups == [A.T("ov.relay_grp_engine"), A.T("ov.relay_grp_trim"), A.T("ov.relay_grp_extra")]

    placed_ok = getattr(s1_matrix, "placed", None) in ("se", "sw", "ne", "nw") and not layout_problems(ap)
    # clicks: a toggle, press-and-hold (momentary), START hold, a grey (unknown) relay
    sent = []
    orig_cmd, orig_send = ap.cmd, ap._send_cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))
    ap._send_cmd = lambda f, p, w, confirm=None: sent.append((f, p))

    def click(tag, hold=0.0):
        r = btn[tag]["r"]
        e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
        ap.cv_press(e)
        if hold:
            rig.pump(hold)
        ap.cv_release(e)
        out = list(sent)
        sent.clear()
        return out
    try:
        c_toggle = click("relay_6")
        c_hold = click("relay_4", 0.2)
        c_start = click("relay_2", 0.5)
        c_grey = click("relay_7")
    finally:
        ap.cmd, ap._send_cmd = orig_cmd, orig_send
        if ap.crank_job is not None:
            ap.root.after_cancel(ap.crank_job)
            ap.crank_job = None
    toggle_ok = c_toggle == [(proto.T_RELAY, proto.p_relay(6, proto.RELAY_CMD_OFF))]
    hold_ok = c_hold == [(proto.T_RELAY, proto.p_relay(4, proto.RELAY_CMD_PULSE, 3000)),
                         (proto.T_RELAY, proto.p_relay(4, proto.RELAY_CMD_OFF))]
    start_ok = bool(c_start) and c_start[0] == (proto.T_RESTART, proto.p_restart(proto.ENG_CRANK)) and \
        c_start[-1] == (proto.T_RESTART, proto.p_restart(proto.ENG_RELEASE))
    grey_ok = c_grey == []
    # the table not active (another hash) / waiting for the CONFIRM: a note, every button grey
    relay_boat(rig, hash_ok=False)
    ap.render(force=True)
    na = [o for o in ops() if o["k"] == "button" and o["tag"].startswith("relay_") and o["tag"] != "relay_restart"]
    note_na = [o["s"] for o in ops() if o.get("tag") == "relay_note"]
    relay_boat(rig, hash_ok=False, pending=True)
    ap.render(force=True)
    note_p = [o["s"] for o in ops() if o.get("tag") == "relay_note"]
    inactive_ok = bool(na) and not any(o["enabled"] for o in na) and note_na and \
        A.T("ov.relay_not_active") in note_na[0].replace("\n", " ") and note_p and \
        A.T("ov.relay_pending") in note_p[0].replace("\n", " ")
    # step 13h (review ce2e4d0 LOW-1): TELEM2 M flag 128 (a stop / timer pending) with the boat file's table not
    # active = the note says why the table change waits (in place of "not active": the panel keeps its height,
    # checked in the S1 matrix below); the table active = no note; the flag gone = "not active" again
    relay_boat(rig, hash_ok=False, stop=True)
    ap.render(force=True)
    note_s = [o["s"].replace("\n", " ") for o in ops() if o.get("tag") == "relay_note"]
    relay_boat(rig, hash_ok=True, stop=True)
    ap.render(force=True)
    note_a = [o for o in ops() if o.get("tag") == "relay_note"]
    relay_boat(rig, hash_ok=False, stop=False)
    ap.render(force=True)
    note_n = [o["s"].replace("\n", " ") for o in ops() if o.get("tag") == "relay_note"]
    stop_ok = bool(note_s) and A.T("ov.relay_stop") in note_s[0] and not note_a and bool(note_n) and \
        A.T("ov.relay_not_active") in note_n[0]
    # the S1 matrix with the stop note (Greek + English, every size): nothing clipped
    relay_boat(rig, hash_ok=False, stop=True)
    probs = []
    for lang in ("el", "en"):
        i18n.set_lang(lang, save=False)
        for W, H in ((1366, 768), (1366, 658), (1920, 1080), (1920, 970)):
            for dpi in (1.0, 1.25, 1.5):
                rig.size(W, H, dpi)
                pr = layout_problems(ap)
                if pr:
                    probs.append(((lang, W, H, dpi), pr[:2]))
    i18n.set_lang("el", save=False)
    rig.size(1366, 768, 1.25)
    stop_ok = stop_ok and not probs
    relay_boat(rig, on=(1, 6), unknown=(7,))
    ap.ov.collapsed.add("relays")
    ap.render(force=True)
    collapsed_ok = all(o["k"] in ("panel", "text") and o.get("tag") in ("relays", "relays_head") for o in ops())
    ap.ov.collapsed.discard("relays")
    # a copilot (through the gateway): trim / extra not asked, an engine relay waits for the captain; only when the
    # gateway knows the active table (gw_status "relays"), else every relay waits
    ap.gw_status = dict(fake_gw(ap.t, captain=False), relays={"hash": h, "known": True})
    rig.pump(0.3)
    groups_known = ap.relay_groups_known()
    seqs = []
    ap._send_cmd = lambda f, p, w, confirm=None: seqs.append(f) or len(seqs)
    try:
        n0 = len(ap.decisions)
        ap.relay_press(6)                              # extra
        free_extra = len(ap.decisions) == n0
        ap.relay_press(1)                              # engine (IGNITION)
        asked_engine = len(ap.decisions) == n0 + 1 and ap.decisions[-1]["state"] == "wait"
        ap.gw_status = dict(fake_gw(ap.t, captain=False), relays={"hash": h, "known": False})
        n1 = len(ap.decisions)
        ap.relay_press(6)
        unknown_asked = len(ap.decisions) == n1 + 1 and ap.relay_groups_known() is None
    finally:
        ap._send_cmd = orig_send
    ap.decisions.clear()
    ap.gw_status = fake_gw(ap.t)
    no_relay_boat(rig)
    ap.render(force=True)
    gone = "relays" not in {p.key for p in ap.ov.panels}
    report("S14 relay buttons: the relays panel (by group, placed by the fit test, an interlocked pair on one line, the "
           "engine line + START hold + RESTART), lit = on / grey = unknown, toggle / press-and-hold / START hold sent "
           "right, nothing for a grey relay or an inactive / pending table, collapsible; a copilot's extra relay not "
           "asked, an engine relay (or any relay of a table the gateway doesn't know) waits for the captain; no relays "
           "= no panel",
           have and pair and engine and states and grp_ok and placed_ok and toggle_ok and hold_ok and start_ok and
           grey_ok and inactive_ok and stop_ok and collapsed_ok and groups_known == proto.groups_of(
               [r for r in __import__("boats").load() if r.id == "rf49d"][0].relays) and free_extra and asked_engine
           and unknown_asked and gone and not rig.errors,
           "buttons %s; pair line %s; engine line %s; states %s; groups %s; placed %s (%s); toggle %s; hold %s; START %s; "
           "grey %s; inactive / pending %s (%s / %s); stop note %s; collapsed %s; copilot: groups %s, extra free %s, "
           "engine asked %s, "
           "unknown table asked %s; no relays no panel %s; errors %s" % (
               sorted(btn), pair, engine, states, groups, placed_ok, getattr(s1_matrix, "placed", None), c_toggle,
               c_hold, c_start[:1] + c_start[-1:], c_grey, inactive_ok, note_na[:1], note_p[:1], (stop_ok, probs[:2]), collapsed_ok,
               bool(groups_known), free_extra, asked_engine, unknown_asked, gone, rig.errors[:2]))


# DP (CLAUDE.md P, step 11): both thrusters fitted (TELEM2 group N) + a DP SPOT state
DP_THR = dict(thr_cfg=True, thr_active=True, thr_fitted_bow=True, thr_fitted_stern=True, thr_bow=-35, thr_stern=60,
              thr_heat_b=40, thr_heat_s=20, thr_hot_bow=False, thr_hot_stern=False)
DP_SPOT_T = dict(mode=proto.MODE_DP, mode_name="DP", dp_sub=proto.DP_SPOT, dp_err_fwd=2.0, dp_err_stbd=-1.5,
                 dp_err_m=2.5, dp_heading=123.0, dp_set_kn=0.0, dp_trip_kn=0.0, dp_wp=0, dp_offspot=False,
                 dp_pending=False, dp_trip_done=False, dp_overpower=False)


def dp_clear(rig):
    for k in list(DP_THR) + list(DP_SPOT_T):
        rig.extra.pop(k, None)
    rig.pump(0.3)


def s15_dp(rig):
    """CLAUDE.md P step 11: the DP rows of screen 2's control panel + the DP line of the status panel (DISPLAY +
    requests only)."""
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = fake_gw(ap.t)
    rig.size(1366, 768, 1.25)
    dp_clear(rig)
    ap.render(force=True)
    ops = lambda: [o for o in ap.ov.ops if o.get("panel") == "control" and str(o.get("tag")).startswith("dp_")]  # noqa
    absent = not ops()
    sent = []
    orig_cmd, orig_send = ap.cmd, ap._send_cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))

    def buttons():
        ap.render(force=True)
        return {o["tag"]: o for o in ops() if o["k"] == "button"}

    def click(tag):
        b = buttons()
        if tag not in b:
            return ["no button %s" % tag]
        r = b[tag]["r"]
        e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
        ap.cv_press(e)
        ap.cv_release(e)
        out = list(sent)
        sent.clear()
        return out
    D = proto.T_DP
    try:
        # STANDBY, thrusters fitted: SPOT / CRUISE / TRIP / STOP + the speed; no JOG, no heading
        rig.extra.update(DP_THR)
        rig.pump(0.4)
        b = buttons()
        stb_ok = bool({"dp_spot", "dp_cruise", "dp_trip", "dp_kn_minus", "dp_kn_plus"} <= set(b)) and \
            not [t for t in b if t.startswith(("dp_jog_", "dp_kn_set", "dp_stop"))] and \
            bool([o for o in ap.ov.ops if o.get("panel") == "control" and o.get("tag") == "auto"])   # AUTO rows stay
        ap.dp_kn = 1.0
        c_spot = click("dp_spot")
        c_plus = click("dp_kn_plus") + click("dp_kn_plus") + click("dp_kn_plus") + click("dp_kn_plus") + \
            click("dp_kn_plus") + click("dp_kn_plus")                     # 1.0 -> 3.0 (the ceiling), nothing sent
        top = ap.dp_kn
        c_minus = click("dp_kn_minus") + click("dp_kn_minus")             # 2.0
        c_cruise = click("dp_cruise")
        c_trip = click("dp_trip")
        rig.extra.update(dp_pending=True)                                 # a request waits: STOP cancels it
        rig.pump(0.3)
        pend_txt = [o for o in ops() if o.get("tag") == "dp_pending"]
        c_stop = click("dp_stop")
        rig.extra.update(dp_pending=False)
        start_ok = c_spot == [(D, proto.p_dp(proto.DP_START, proto.DP_SPOT, 0))] and c_plus == [] and top == 3.0 and \
            c_minus == [] and c_cruise == [(D, proto.p_dp(proto.DP_START, proto.DP_CRUISE, 200))] and \
            c_trip == [(D, proto.p_dp(proto.DP_START, proto.DP_TRIP, 200))] and \
            c_stop == [(D, proto.p_dp(proto.DP_STOP))] and bool(pend_txt)
        # in DP SPOT: the state + where the spot is, JOG arrows, DP STOP; the AUTO rows gone; the panel's +-1 / +-10 =
        # HEADING_ADJ (a DP heading change on the boat); the status panel's DP line
        rig.extra.update(DP_SPOT_T)
        rig.pump(0.4)
        b = buttons()
        st_txt = " ".join(o.get("s", "") for o in ops() if o.get("tag") in ("dp_state", "dp_err"))
        status_txt = " ".join(str(o.get("s", "")) for o in ap.ov.ops if o.get("panel") == "status")
        no_auto = not [o for o in ap.ov.ops if o.get("panel") == "control" and o.get("tag") == "auto"]
        spot_ok = all("dp_jog_%d" % k in b for k in range(4)) and "dp_stop" in b and no_auto \
            and A.T("dp.sub_1") in st_txt and "2.5" in st_txt and A.T("dp.ahead") in st_txt and \
            A.T("dp.port") in st_txt and A.T("dp.sub_1") in status_txt and "123.0" in status_txt and \
            "dp_trip_cont" not in b and b["dp_spot"]["on"]
        c_jog = click("dp_jog_%d" % proto.DP_JOG_PORT)
        hb = [o for o in ap.ov.ops if o.get("tag") == "hdg_+10" and o["k"] == "button"]     # bar 1 (13n1)
        c_hdg = []
        if hb:
            r = hb[0]["r"]
            e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
            ap.cv_press(e)
            ap.cv_release(e)
            c_hdg, sent[:] = list(sent), []
        jog_ok = c_jog == [(D, proto.p_dp(proto.DP_JOG_ACT, proto.DP_JOG_PORT))] and \
            c_hdg == [(proto.T_HEADING_ADJ, proto.p_heading(10))]
        # SPOT after a STOP waypoint of a TRIP: "continue"
        rig.extra.update(dp_wp=2)
        rig.pump(0.3)
        c_cont = click("dp_trip_cont")
        cont_ok = c_cont == [(D, proto.p_dp(proto.DP_START, proto.DP_TRIP | proto.DP_TRIP_CONTINUE, 200))]
        # CRUISE at 1.0 kn: the speed SET sends 2.0 kn; no JOG in CRUISE
        rig.extra.update(dp_sub=proto.DP_CRUISE, dp_set_kn=1.0, dp_wp=0)
        rig.pump(0.3)
        b = buttons()
        c_set = click("dp_kn_set")
        cruise_ok = not [t for t in b if t.startswith("dp_jog_")] and b["dp_cruise"]["on"] and \
            c_set == [(D, proto.p_dp(proto.DP_SPEED, 0, 200))]
        # off the spot: the error in the bad colour
        rig.extra.update(DP_SPOT_T, dp_offspot=True, dp_err_m=17.0, dp_err_fwd=17.0, dp_err_stbd=0.0)
        rig.pump(0.3)
        ap.render(force=True)
        err_op = [o for o in ops() if o.get("tag") == "dp_err"]
        off_ok = bool(err_op) and err_op[0].get("colour") == A.C["bad_txt"]
    finally:
        ap.cmd = orig_cmd
    # a copilot (through the gateway): a DP start waits for the captain, DP STOP goes straight
    rig.extra.update(DP_SPOT_T)
    ap.gw_status = fake_gw(ap.t, captain=False)
    rig.pump(0.3)
    seqs = []
    ap._send_cmd = lambda f, p, w, confirm=None: seqs.append(f) or len(seqs)
    try:
        n0 = len(ap.decisions)
        ap.dp_stop()
        stop_free = len(ap.decisions) == n0 and seqs == [proto.T_DP]
        ap.dp_start(proto.DP_SPOT)
        start_asked = len(ap.decisions) == n0 + 1 and ap.decisions[-1]["state"] == "wait"
    finally:
        ap._send_cmd = orig_send
    ap.decisions.clear()
    ap.gw_status = fake_gw(ap.t)
    # the S1 matrix with the DP rows (every panel full; STANDBY with both thrusters fitted, then DP SPOT): 7 panels,
    # nothing clipped / over the boat; the Greek DP rows have no English words
    fill_everything(rig)
    rig.extra.update(DP_THR)
    cases = []
    greek_en = []
    for state in ("standby", "spot"):
        if state == "spot":
            rig.extra.update(DP_SPOT_T)
        for lang in ("el", "en"):
            i18n.set_lang(lang, save=False)
            for W, H in ((1366, 768), (1366, 658), (1920, 1080), (1920, 970)):
                for dpi in (1.0, 1.25, 1.5):
                    rig.size(W, H, dpi)
                    cases.append(((state, lang, W, H, dpi), len(ap.ov.panels), layout_problems(ap)))
                    if lang == "el" and W == 1920 and dpi == 1.0:
                        for o in ops() + [o for o in ap.ov.ops if o.get("panel") == "status"]:
                            greek_en += english_words(o.get("s", ""))
    bad = [c for c in cases if c[2] or c[1] != 8]          # (+ bar 1's ROUTE sub-panel, 13n1)
    i18n.set_lang("el", save=False)
    rig.size(1366, 768, 1.25)
    no_relay_boat(rig)
    dp_clear(rig)
    ap.render(force=True)
    gone = not ops()
    report("S15 DP on screen 2's control panel: the DP rows only with both thrusters fitted (or in DP); SPOT / CRUISE / "
           "TRIP (+ the speed, ceiling 3 kn) from STANDBY, STOP cancels a waiting request; in DP (instead of the AUTO "
           "rows) the state + where the spot is, DP STOP, JOG arrows, the panel's heading +-1 / +-10 (HEADING_ADJ), "
           "\"continue\" after a STOP waypoint, the CRUISE speed SET; the status panel's DP line; off spot in the bad "
           "colour; a copilot's start waits for the captain, DP STOP goes straight; the S1 matrix in DP fits; no "
           "English in the Greek rows",
           absent and stb_ok and start_ok and spot_ok and jog_ok and cont_ok and cruise_ok and off_ok and stop_free and
           start_asked and not bad and not greek_en and gone and not rig.errors,
           "absent first %s; STANDBY buttons %s; starts %s (spot %s, cruise %s, trip %s, stop %s, ceiling %.1f); DP SPOT %s "
           "(\"%s\"); JOG / heading %s %s; continue %s %s; CRUISE SET %s %s; off spot %s; copilot: STOP free %s, start "
           "asked %s; matrix problems %s; English in Greek %s; gone %s; errors %s" % (
               absent, stb_ok, start_ok, c_spot, c_cruise, c_trip, c_stop, top, spot_ok, st_txt[:80], jog_ok,
               c_jog + c_hdg, cont_ok, c_cont, cruise_ok, c_set, off_ok, stop_free, start_asked,
               [(c[0], c[1], c[2][:3]) for c in bad][:4], greek_en[:8], gone, rig.errors[:2]))


def shot(win, path):
    """A picture of a window as on the screen (docs/screens)."""
    try:
        from PIL import ImageGrab
    except ImportError:                                 # a Python without Pillow: no picture (the test still runs)
        print("(no Pillow: %s not made)" % path)
        return
    win.attributes("-topmost", True)                   # in front of every other window, or the grab shows them
    win.deiconify()
    win.lift()
    win.focus_force()
    end = time.time() + 1.0
    while time.time() < end:
        win.update()
        time.sleep(0.02)
    x, y, w, h = win.winfo_rootx(), win.winfo_rooty(), win.winfo_width(), win.winfo_height()
    ImageGrab.grab(bbox=(x, y, x + w, y + h), all_screens=True).save(path)
    win.attributes("-topmost", False)


def s17_link_time(rig):
    """Step 13q1 (owner 2026-10-31 C1): LINK TIME. Screen 2's control panel (geofence line) shows the AURIX's value with - / + for the
    HELM captain only (one step of alerts.LINK_TIME_STEPS, sent as PROTO_T_LINK_TIME); above 3 s the banner
    "LINK TIME 30 s (OVERRIDE)" on screen 2 AND screen 1 all the time, with "not in force now: near the shore" when
    the boat says so; at 3 s no banner. The S1 matrix still fits (the panel line is there in every S1 case)."""
    ap = rig.ap
    rig.gateway_like(True)
    rig.size(1366, 768, 1.0)
    ap.gw_status = fake_gw(ap.t)
    sent = []
    orig_cmd = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p, w))
    rig.extra.update(helm=3, profile=proto.PROFILE_UNCREWED, ll_state=0, near_shore=False, coast_valid=True,
                     coast_dist_m=800, lt_set=30, lt_flags=0, lt_override=True, lt_in_force=True, lt_limit_s=30,
                     lt_silent_s=None, lt_reported=True)
    try:
        rig.pump(0.6)
        ap.render(force=True)
        ban = [b[1] for b in ap.banner_list() if b[0] == "linktime"]
        lab = [o["s"] for o in ap.ov.ops if o.get("tag") == "lt_label"]
        btn = {o["tag"]: o for o in ap.ov.ops if o.get("k") == "button" and o.get("tag") in ("lt_up", "lt_down")}

        def click(tag):
            r = btn[tag]["r"]
            e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
            ap.cv_press(e)
            ap.cv_release(e)
            out = list(sent)
            sent.clear()
            return out
        up = click("lt_up") if "lt_up" in btn else []
        down = click("lt_down") if "lt_down" in btn else []
        cap_ok = ban == [A.T("lt.banner", s=30)] and ap.banner_on("linktime") and lab == [A.T("lt.label", s=30)] and             all(btn[k]["enabled"] for k in btn) and len(btn) == 2 and             [(f, p) for f, p, _w in up] == [(proto.T_LINK_TIME, proto.p_link_time(45))] and             [(f, p) for f, p, _w in down] == [(proto.T_LINK_TIME, proto.p_link_time(20))]
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            shot(ap.root, os.path.join(shots, "screen2_linktime_el.png"))
        # screen 1: the same banner in the notices
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.pump(1.5)
        s1 = [n[1] for n in dv.notices(ap.t, ap.gw_status) if n[0] == "linktime"]
        s1_box = any(o.get("s") == A.T("lt.banner", s=30) for o in dv.ops)     # the mode box line too
        if shots and dv.last_frame is not None:
            dv.last_frame.save(os.path.join(shots, "driver_linktime_el.png"))
        # near the shore: still shown, "not in force now"
        rig.extra.update(near_shore=True, coast_dist_m=40, lt_in_force=False, lt_flags=2, lt_limit_s=3)
        rig.pump(0.6)
        ap.render(force=True)
        near = [b[1] for b in ap.banner_list() if b[0] == "linktime"]
        near_ok = near == [A.T("lt.banner_off", s=30, why=A.T("lt.why_near"))]
        # the copilot: shown, but - / + disabled (the gateway refuses it too: 118)
        ap.gw_status = fake_gw(ap.t, captain=False)
        ap.render(force=True)
        cop = [o for o in ap.ov.ops if o.get("k") == "button" and o.get("tag") in ("lt_up", "lt_down")]
        cop_ok = len(cop) == 2 and not any(o["enabled"] for o in cop)
        # 3 s: no banner on either screen
        ap.gw_status = fake_gw(ap.t)
        rig.extra.update(lt_set=3, lt_override=False, lt_in_force=False, lt_flags=2, near_shore=False)
        rig.pump(0.6)
        ap.render(force=True)
        none_ok = not ap.banner_on("linktime") and not [n for n in dv.notices(ap.t, ap.gw_status) if n[0] == "linktime"]
        ap.close_driver()
        fits = not layout_problems(ap)
    finally:
        ap.cmd = orig_cmd
        for k in ("lt_set", "lt_flags", "lt_override", "lt_in_force", "lt_limit_s", "lt_silent_s", "lt_reported",
                  "helm", "coast_valid", "coast_dist_m"):
            rig.extra.pop(k, None)
        rig.extra.update(profile=proto.PROFILE_CREWED, near_shore=False)
    report("S17 LINK TIME (13q1): screen 2 control panel value + - / + for the HELM captain (one step, PROTO_T_LINK_TIME), "
           "the copilot's disabled; above 3 s 'LINK TIME 30 s (OVERRIDE)' on screens 2 + 1, 'not in force' near the "
           "shore; none at 3 s",
           cap_ok and s1 == [A.T("lt.banner", s=30)] and s1_box and near_ok and cop_ok and none_ok and fits,
           "banner %s, label %s, buttons %s, + sent %s, - sent %s; screen 1 %s (mode box %s); near %s; copilot disabled %s; at 3 s "
           "none %s; fits %s" % (ban, lab, sorted(btn), [p for _f, p, _w in up], [p for _f, p, _w in down], s1, s1_box, near,
                                 cop_ok, none_ok, fits))


def s18_uncrewed_hdg(rig):
    """Step 13q2c (owner 2026-11-01): UNCREWED, screen 2's HDG LOCK button sends the HELM captain's mode command
    (PROTO_T_SET_MODE HDG_LOCK - the AURIX checks it: only from REMOTE / MANEUVER, stick centred, throttle held);
    CREWED it only says "the panel button on board" (nothing sent). The new alarm GEOFENCE BREACH (second word, bit
    0x80) is a WARNING row in the alarms panel (Greek text) and on screen 1."""
    ap = rig.ap
    import alerts
    rig.gateway_like(True)
    rig.size(1366, 768, 1.0)
    ap.gw_status = fake_gw(ap.t)
    sent = []
    orig_cmd = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p, w))
    try:
        rig.extra.update(profile=proto.PROFILE_UNCREWED, mode=proto.MODE_REMOTE, mode_name="REMOTE_MANUAL", helm=3)
        rig.pump(0.6)
        ap.render(force=True)
        btn = [o for o in ap.ov.ops if o.get("k") == "button" and o.get("tag") == "mode_HDG_LOCK"]     # the mode bar

        def click():
            r = btn[0]["r"]
            e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
            ap.cv_press(e)
            ap.cv_release(e)
            out = list(sent)
            sent.clear()
            return out
        un = click() if btn else []
        un_ok = [(f, p) for f, p, _w in un] == [(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_HDG_LOCK))]
        rig.extra.update(profile=proto.PROFILE_CREWED)
        rig.pump(0.4)
        ap.render(force=True)
        btn = [o for o in ap.ov.ops if o.get("k") == "button" and o.get("tag") == "mode_HDG_LOCK"]
        cr = click() if btn and not btn[0]["enabled"] else ["no greyed button"]     # CREWED: greyed, says why
        # UNCREWED HDG LOCK running + the geofence alarm (WARNING, Greek)
        rig.extra.update(profile=proto.PROFILE_UNCREWED, mode=proto.MODE_STANDBY, mode_name="STANDBY",
                         alarm_bits2=0x80, alarm=True)
        ap.lamps.acked = {alerts.WARNING: 0, alerts.CAUTION: 0}
        rig.pump(1.2)
        ap.render(force=True)
        rows = [o.get("s") for o in ap.ov.ops if o.get("tag") == "alarm_row"]
        lv = alerts.ALARM_LEVELS.get(0x80 << alerts.WORD2)
        al_ok = A.T("alarm.geofence") in rows and lv == ("geofence", alerts.WARNING)
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            shot(ap.root, os.path.join(shots, "screen2_geofence_el.png"))
            ap.open_driver()
            dv = ap.driver
            dv.w.geometry("1600x900+0+0")
            rig.pump(1.5)
            if dv.last_frame is not None:
                dv.last_frame.save(os.path.join(shots, "driver_geofence_el.png"))
            ap.close_driver()
        fits = not layout_problems(ap)
    finally:
        ap.cmd = orig_cmd
        for k in ("alarm_bits2", "helm"):
            rig.extra.pop(k, None)
        rig.extra.update(profile=proto.PROFILE_CREWED, mode=proto.MODE_STANDBY, mode_name="STANDBY", alarm=False)
    report("S18 UNCREWED HDG LOCK (13q2c): screen 2's HDG LOCK button = the mode command (UNCREWED), nothing sent "
           "CREWED; the GEOFENCE BREACH alarm is a WARNING row",
           un_ok and cr == [] and al_ok and fits,
           "UNCREWED REMOTE: sent %s; CREWED: sent %s; alarm rows %s (level %s); fits %s" % (
               [(f, p) for f, p, _w in un], cr, rows[:3], lv, fits))


def s20_slam_restore(rig):
    """Step 13q4 (owner 2026-10-31 C4): the slamming reduction never goes away by itself. Screen 2's control panel
    shows the framed RESTORE POWER button (TELEM2 T b1, after a REJECT / no answer) for the captain only - a click
    sends PROTO_T_SLAM_RESTORE ACCEPT; the boat's question opens on screen 1 (driver.ask, ACCEPT / REJECT); a copilot
    sees neither; the layout still fits at the small sizes in both languages."""
    ap = rig.ap
    rig.gateway_like(True)
    rig.size(1366, 768, 1.0)
    ap.gw_status = fake_gw(ap.t)
    sent = []
    orig_send = ap._send_cmd
    ap._send_cmd = lambda f, p, w, confirm=None: sent.append((f, bytes(p)))
    keys = ("helm", "slr_flags", "slr_before", "slam_ask", "slam_restore_ok", "slam_restoring", "slam_declined",
            "slam_ask_left", "slam_cap")
    rig.extra.update(helm=3, slr_flags=2 | 8, slr_before=60, slam_ask=False, slam_restore_ok=True,
                     slam_restoring=False, slam_declined=True, slam_ask_left=0, slam_cap=50)
    shots = os.environ.get("TRYGONS_SHOT_DIR")
    # step 13p (gate finding, also at b50413b): the copilot-answer rows of the earlier tests (GF / roles: shown 60 s
    # after the answer, a WAITING row without an ack for ever) are not this test's subject - they added 3 lines to the
    # control panel, so "fits" depended on how fast the run reached S20. Start without them, put them back after.
    old_dec = list(ap.decisions)
    ap.decisions.clear()
    try:
        rig.pump(0.6)
        ap.render(force=True)
        btn = [o for o in ap.ov.ops if o.get("k") == "button" and o.get("tag") == "slam_restore"]
        clicked = []
        if btn:
            r = btn[0]["r"]
            e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
            ap.cv_press(e)
            ap.cv_release(e)
            clicked = list(sent)
            sent.clear()
        btn_ok = len(btn) == 1 and btn[0]["s"] == A.T("slam.btn") and clicked == [(proto.T_SLAM_RESTORE, b"\x01")]
        if shots:
            shot(ap.root, os.path.join(shots, "screen2_slam_restore_el.png"))
        fits = []
        for W, H in ((1366, 658), (1920, 970)):
            for lang in ("el", "en"):
                i18n.set_lang(lang)
                rig.size(W, H, 1.0)
                fits.append(not layout_problems(ap))
        i18n.set_lang("el")
        rig.size(1366, 768, 1.0)
        # screen 1: the boat's question
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.extra.update(slr_flags=1 | 2 | (9 << 4), slam_ask=True, slam_ask_left=9, slam_declined=False)
        rig.pump(1.5, lambda: dv.question is not None)
        asked = dv.question is not None and dv.qhead.cget("text") == A.T("slam.head") and \
            A.T("slam.ask", pct=60) == dv.qtext.cget("text")
        if shots:
            rig.pump(0.5)
            shot(dv.w, os.path.join(shots, "driver_slam_ask_el.png"))
            dv.select(False)                            # step 13p: NO selected (the stick back / hat down)
            shot(dv.w, os.path.join(shots, "driver_question_no_el.png"))
            dv.select(True)
            ap.captain_confirm(A.TE("what.auto_start"), lambda: None)   # the single operator's own confirmation
            if ap.cconf is not None:
                shot(ap.cconf, os.path.join(shots, "screen2_own_confirm_el.png"))
                ap.cconf_answer(False)
        dv.answer(True)
        acc = (proto.T_SLAM_RESTORE, b"\x01") in sent
        ap.close_driver()
        # a copilot: no button, no question
        sent.clear()
        ap.gw_status = fake_gw(ap.t, captain=False)
        rig.extra.update(slr_flags=2 | 8, slam_ask=False, slam_declined=True)
        rig.pump(0.6)
        ap.render(force=True)
        cop_btn = any(o.get("tag") == "slam_restore" for o in ap.ov.ops)
    finally:
        for k in keys:
            rig.extra.pop(k, None)
        ap._send_cmd = orig_send
        ap.slam_tag = None
        ap.gw_status = fake_gw(ap.t)
        ap.decisions.extend(old_dec)
    report("S20 slamming power restore (13q4): RESTORE POWER on screen 2 for the captain (sends ACCEPT), the boat's "
           "question on screen 1, nothing for a copilot, fits",
           btn_ok and all(fits) and asked and acc and not cop_btn,
           "button %s, click sent %s; fits %s; screen 1 question %s, ACCEPT sent %s; copilot button %s" % (
               [b["s"] for b in btn], clicked, fits, asked, acc, cop_btn))


def s19_override(rig):
    """Step 13q3 (owner 2026-10-31 C3, DISPLAY ONLY): the shore stick override in UNCREWED AUTO / HDG LOCK on both
    screens - "MANUAL STEERING - AUTO waiting" / "... HDG LOCK waiting (trigger = new heading)" (+ "back in 2 s" once
    the stick is centred), "NEW HEADING 245°" after the trigger, the lever ceiling notice, the AUTO stale-data stop
    (warning colour); CREWED: nothing (the onboard joystick takes MANUAL); the S1 layout still fits."""
    ap = rig.ap
    rig.gateway_like(True)
    rig.size(1366, 768, 1.0)
    ap.gw_status = fake_gw(ap.t)
    keys = ("ov_hand", "ov_back", "ov_newhdg", "ov_ceiling", "ov_lost", "helm", "ov_lever_wait", "rjoy_pickup1000")
    tags = ("sovr_hand", "sovr_newhdg", "sovr_ceiling", "sovr_lost", "sovr_lever")
    try:
        def screen2():
            rig.pump(0.4)
            ap.render(force=True)
            return {b[0]: b[1] for b in ap.banner_list() if b[0] in tags}
        rig.extra.update(profile=proto.PROFILE_UNCREWED, mode=proto.MODE_AUTO, mode_name="AUTO", helm=3, ov_hand=True,
                         ov_back=False, ov_newhdg=False, ov_ceiling=True, ov_lost=False, target=245.0)
        auto = screen2()
        auto_ok = auto == {"sovr_hand": A.T("sovr.auto"), "sovr_ceiling": A.T("sovr.ceiling")}
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            shot(ap.root, os.path.join(shots, "screen2_override_el.png"))
        rig.extra.update(mode=proto.MODE_HDG_LOCK, mode_name="HDG_LOCK", ov_back=True, ov_newhdg=True, ov_ceiling=False)
        hdg = screen2()
        hdg_ok = hdg == {"sovr_hand": A.T("sovr.hdg") + " – " + A.T("sovr.back"),
                         "sovr_newhdg": A.T("sovr.newhdg", h="245")}
        # screen 1: the same lines in the notices
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.pump(1.5)
        s1 = {n[0]: n[1] for n in dv.notices(ap.t, ap.gw_status) if n[0] in tags}
        box = [o.get("s") for o in dv.ops if o.get("tag") == "override"]     # the mode box line (driver looks here)
        s1_ok = s1 == hdg and box == ["   ".join([hdg["sovr_hand"], hdg["sovr_newhdg"]])]
        if shots and dv.last_frame is not None:
            dv.last_frame.save(os.path.join(shots, "driver_override_el.png"))
        ap.close_driver()
        rig.extra.update(mode=proto.MODE_AUTO, mode_name="AUTO", ov_hand=False, ov_back=False, ov_newhdg=False,
                         ov_lost=True)
        rig.pump(0.4)
        ap.render(force=True)
        lost = [b for b in ap.banner_list() if b[0] == "sovr_lost"]
        lost_ok = len(lost) == 1 and lost[0][1] == A.T("sovr.lost") and lost[0][3] == theme.WARN
        # step 13q3b (review MED-2): the AUTO lever below the throttle but not engaged yet = "move the lever" (both)
        rig.extra.update(ov_lost=False, ov_lever_wait=True, rjoy_pickup1000=674)
        wait = screen2()
        wait_ok = wait == {"sovr_lever": A.T("sovr.lever_wait", to="67 %")}
        if shots:
            shot(ap.root, os.path.join(shots, "screen2_lever_wait_el.png"))
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.pump(1.5)
        s1w = {n[0]: n[1] for n in dv.notices(ap.t, ap.gw_status) if n[0] in tags}
        if shots and dv.last_frame is not None:
            dv.last_frame.save(os.path.join(shots, "driver_lever_wait_el.png"))
        ap.close_driver()
        wait_ok = wait_ok and s1w == wait
        rig.extra.update(ov_lever_wait=False)
        rig.extra.update(profile=proto.PROFILE_CREWED, ov_hand=True, ov_newhdg=True, ov_ceiling=True, ov_lost=True,
                         ov_lever_wait=True)
        crew = screen2()
        fits = not layout_problems(ap)
    finally:
        for k in keys:
            rig.extra.pop(k, None)
        rig.extra.update(profile=proto.PROFILE_CREWED, mode=proto.MODE_STANDBY, mode_name="STANDBY")
    report("S19 shore stick override (13q3): MANUAL STEERING - AUTO / HDG LOCK waiting, back in 2 s, NEW HEADING, the "
           "lever ceiling, the AUTO data-lost stop (warning), 13q3b the lever not engaged, on screens 2 + 1; CREWED "
           "nothing",
           auto_ok and hdg_ok and s1_ok and lost_ok and wait_ok and crew == {} and fits,
           "AUTO %s; HDG LOCK %s; screen 1 %s, mode box %s; lost %s; lever not engaged %s / screen 1 %s; CREWED %s; "
           "fits %s" % (auto, hdg, s1, box, lost, wait, s1w, crew, fits))


def s16_silence(rig):
    """Step 13m (owner 2026-10-31 B): an alarm this station could not silence. Screen 2's ACK ALARM silences THIS
    PC first (MASTER lamps + the repeating sound) and then sends the boat's ack; Space / A on screen 2 = the same
    MASTER acknowledge (not while typing into a field); the fallback beep (no sound system) only while the MASTER
    WARNING lamp is lit; a newer warning sounds again; disconnect -> no telemetry, no alarm rows, silence, the
    HUD says "not connected"; reconnect -> alarms from fresh data only. Runs LAST (it disconnects)."""
    ap = rig.ap
    import alerts
    import hud
    rig.gateway_like(False)
    if ap.driver:
        ap.close_driver()
    rig.size(1366, 768, 1.0)
    beeps = []
    ap._beep = lambda f: beeps.append(f)
    ap.root.bell = lambda *a: beeps.append(0)
    sent = []
    orig_cmd = ap.cmd
    ap.cmd = lambda f, p, w: sent.append(f)
    rig.extra.update(alarm_bits=0x001, alarm=True)                     # COMPASS LOST (WARNING)
    ap.lamps = alerts.MasterLamps()
    rig.pump(2.6)
    beeping = len(beeps) >= 1 and ap.lamps.on(alerts.WARNING)
    ap.render(force=True)
    ack = [o for o in ap.ov.ops if o.get("tag") == "ack"]
    if ack:
        r = ack[0]["r"]
        e = type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()
        ap.cv_press(e)
        ap.cv_release(e)
    acked = not ap.lamps.on(alerts.WARNING) and sent == [proto.T_ALARM_ACK]
    rows = ap.lamps.rows()
    steady = bool(rows) and all(st for _t, _lv, st in rows)
    n0 = len(beeps)
    rig.pump(2.6)
    quiet_after_ack = len(beeps) == n0
    shots = os.environ.get("TRYGONS_SHOT_DIR")         # docs/screens (step 13m pictures)
    if shots:
        shot(ap.root, os.path.join(shots, "screen2_acked_el.png"))
    # a newer warning lights the lamp and sounds again; Space / A on screen 2 acknowledge it
    rig.extra.update(alarm_bits=0x001 | 0x040)                          # + GEAR FAULT (WARNING)
    rig.pump(2.6)
    again = len(beeps) > n0 and ap.lamps.on(alerts.WARNING)
    keys_bound = all(ap.root.bind(k) for k in ("<space>", "a", "A"))
    ent = tk.Entry(ap.root)
    ap.key_ack(type("E", (), {"widget": ent})())                        # typing an 'a' in a field: not an ack
    typing_ignored = ap.lamps.on(alerts.WARNING)
    ent.destroy()
    ap.key_ack(type("E", (), {"widget": ap.root})())
    key_acked = not ap.lamps.on(alerts.WARNING)
    n1 = len(beeps)
    rig.pump(2.6)
    quiet_after_key = len(beeps) == n1 and len(sent) == 1              # Space / A: nothing sent to the boat
    # disconnect: no boat = no telemetry, no rows, no NO DATA, silence (the old data never rings again)
    rig.extra.update(alarm_bits=0x001 | 0x040)
    ap.lamps = alerts.MasterLamps()
    rig.pump(1.2)
    lit_before = ap.lamps.on(alerts.WARNING)
    ap.cmd = orig_cmd
    ap.disconnect()
    n2 = len(beeps)
    rig.pump(4.0)                                                       # past the 3 s NO DATA time too
    gone = ap.t is None and not ap.t_rx and ap.lamps.rows() == [] and not ap.lamps.on(alerts.WARNING) and \
        len(beeps) == n2 and not (ap.sound and ap.sound.sched.active)
    ap.render(force=True)
    no_rows = not any(o.get("tag") == "alarm_row" for o in ap.ov.ops) and \
        {p.key: p for p in ap.ov.panels}["alarms"].title == A.T("ov.no_alarms")
    status_nc = any(o.get("s") == A.T("cap.not_connected") for o in ap.ov.ops if o.get("panel") == "status")
    if shots:
        rig.pump(0.5)
        shot(ap.root, os.path.join(shots, "screen2_disconnected_el.png"))
        ap.open_driver()
        rig.pump(3.0, lambda: ap.driver is not None and ap.driver.last_frame is not None)
        rig.pump(1.0)
        if ap.driver is not None and ap.driver.last_frame is not None:
            ap.driver.last_frame.save(os.path.join(shots, "driver_disconnected_el.png"))
        ap.close_driver()
    hd = hud.Hud(hud.Layout(os.path.join(TMP, "nc_layout.json")))
    hops, _ = hd.build(1600, 900, hud.Snapshot(t=None, gw={}, lamps=ap.lamps, flash=True, not_connected=True,
                                               declutter="FULL", alarm_rows=[]))
    hud_nc = any(o.get("s") == A.T("cap.not_connected") for o in hops) and \
        not any(o.get("s") == A.T("alarm.no_data") for o in hops)
    # reconnect: only the fresh data's alarms (the injected ones are gone with the old data)
    rig.extra.clear()
    ap.connect()
    rig.pump(15, lambda: ap.t is not None)
    rig.pump(1.2)
    fresh = ap.t is not None and not alerts.bits_of(ap.t) & 0x041 and \
        A.T("alarm.compass_lost") not in [t for t, _l, _s in ap.lamps.rows()]
    report("S16 silence on this PC (13m): screen-2 ACK ALARM = MASTER ack + the boat's ack, the fallback beep stops "
           "after the ack, a newer warning sounds again, Space / A on screen 2 = MASTER ack (not while typing, nothing "
           "sent), disconnect = no telemetry / rows / NO DATA / sound, HUD + status 'not connected', reconnect = "
           "fresh alarms only",
           beeping and acked and steady and quiet_after_ack and again and keys_bound and typing_ignored and key_acked
           and quiet_after_key and lit_before and gone and no_rows and status_nc and hud_nc and fresh
           and not rig.errors,
           "beeping %s acked %s (sent %s) steady %s quiet %s again %s keys %s typing %s key ack %s quiet2 %s lit %s "
           "gone %s no rows %s status %s hud %s fresh %s errors %s" % (
               beeping, acked, sent, steady, quiet_after_ack, again, keys_bound, typing_ignored, key_acked,
               quiet_after_key, lit_before, gone, no_rows, status_nc, hud_nc, fresh, rig.errors[:2]))



def ev_at(r):
    return type("E", (), {"x": (r[0] + r[2]) / 2, "y": (r[1] + r[3]) / 2})()


def click_r(ap, r, dx=0, dy=0):
    e0 = ev_at(r)
    ap.cv_press(e0)
    if dx or dy:
        for k in range(1, 6):
            ap.cv_motion(type("E", (), {"x": e0.x + dx * k / 5.0, "y": e0.y + dy * k / 5.0})())
    ap.cv_release(type("E", (), {"x": e0.x + dx, "y": e0.y + dy})())


def op(ap, tag):
    o = [x for x in ap.ov.ops if x.get("tag") == tag]
    return o[0] if o else None


def s21_mode_bar(rig):
    """Step 13n1 (owner 2026-10-31 E1): the MODE BAR - top centre, always drawn (also every panel collapsed), never
    hideable: STANDBY HDG LOCK AUTO REMOTE MANEUVER DP MANUAL · BATTLE SHORT; the active mode inverted (white fill,
    larger); STANDBY the only red button; not allowed = grey, a click says why and sends nothing; MANUAL never from
    the PC; AUTO greyed without a route, a click on it with a route = AUTO START; STANDBY always sends."""
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = fake_gw(ap.t)
    rig.size(1366, 768, 1.25)
    sent = []
    orig = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))
    try:
        rig.extra.update(mode=proto.MODE_STANDBY, mode_name="STANDBY", prog_ok=False, profile=proto.PROFILE_CREWED)
        rig.pump(0.5)
        ap.render(force=True)
        names = ["mode_" + n for n, _m in A.App.MODE_BAR]
        ops = [op(ap, t) for t in names] + [op(ap, "battle_short")]
        drawn = all(ops)
        W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
        xs = [o["r"][0] for o in ops if o]
        order = xs == sorted(xs)
        top = all(o["r"][1] < H * 0.12 for o in ops if o)
        cx = (ops[0]["r"][0] + ops[-1]["r"][2]) / 2 if drawn else 0
        centred = abs(cx - W / 2) < W * 0.05
        act = op(ap, "mode_STANDBY")
        inverted = act["on"] and act["fill"].lower() == A.theme.WHITE.lower() and \
            act["r"][3] - act["r"][1] > op(ap, "mode_AUTO")["r"][3] - op(ap, "mode_AUTO")["r"][1]
        reds = [o["tag"] for o in ap.ov.ops if o.get("k") == "button" and str(o.get("fill", "")).lower() ==
                A.theme.WARN.lower()]
        only_red = reds == ["standby"]                # (the active STANDBY on the bar is white = inverted)
        auto_grey = not op(ap, "mode_AUTO")["enabled"]
        manual_grey = not op(ap, "mode_MANUAL")["enabled"]
        ap.msg_text = ""
        click_r(ap, op(ap, "mode_AUTO")["r"])
        why = A.T("mb.why_no_route") in (ap.msg_text or "") and not sent
        click_r(ap, op(ap, "mode_MANUAL")["r"])
        why_man = A.T("mb.why_manual") in (ap.msg_text or "") and not sent
        rig.extra.update(prog_ok=True)
        rig.pump(0.4)
        ap.render(force=True)
        click_r(ap, op(ap, "mode_AUTO")["r"])
        auto_sent = sent == [(proto.T_AUTO_START, b"")]
        sent.clear()
        click_r(ap, op(ap, "mode_STANDBY")["r"])
        sb_sent = sent == [(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY))]
        sent.clear()
        # every panel collapsed: the mode bar stays; it has no hide handle
        for p in list(ap.ov.panels):
            ap.ov.collapsed.add(p.key)
        ap.render(force=True)
        stays = all(op(ap, t) for t in names)
        ap.ov.collapsed.clear()
        ap.render(force=True)
        # in AUTO: AUTO inverted, STANDBY red
        rig.extra.update(mode=proto.MODE_AUTO, mode_name="AUTO")
        rig.pump(0.4)
        ap.render(force=True)
        auto_on = op(ap, "mode_AUTO")["on"] and op(ap, "mode_STANDBY")["fill"].lower() == A.theme.WARN.lower()
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            canvaspic.save(ap.cv, os.path.join(shots, "screen2_modebar_el.png"))     # (the canvas itself: no desktop needed)
            ap.open_driver()                            # screen 1's ONE bottom bar + the heading tape above it
            dv = ap.driver
            dv.w.geometry("1600x900+0+0")
            rig.pump(1.5)
            if dv.last_frame is not None:
                dv.last_frame.save(os.path.join(shots, "driver_bottom_bar_el.png"))
            ap.close_driver()
        fits = not layout_problems(ap)
    finally:
        ap.cmd = orig
        rig.extra.update(mode=proto.MODE_STANDBY, mode_name="STANDBY")
        for k in ("prog_ok", "profile"):
            rig.extra.pop(k, None)
        rig.pump(0.3)
    report("S21 mode bar (13n1): top centre, the 7 modes + BATTLE SHORT in order, always drawn, active inverted + "
           "larger, STANDBY the only red button, greyed = a click says why + nothing sent, MANUAL never from the PC, "
           "AUTO with a route = AUTO START, STANDBY always", drawn and order and top and centred and inverted and
           only_red and auto_grey and manual_grey and why and why_man and auto_sent and sb_sent and stays and
           auto_on and fits and not rig.errors,
           "drawn %s order %s top %s centred %s inverted %s red %s; AUTO grey %s MANUAL grey %s; why %s / %s; AUTO "
           "sent %s; STANDBY sent %s; stays collapsed %s; AUTO lit %s; fits %s; errors %s" % (
               drawn, order, top, centred, inverted, reds, auto_grey, manual_grey, why, why_man, auto_sent, sb_sent,
               stays, auto_on, fits, rig.errors[:2]))


def s22_bars(rig):
    """Step 13n1: the TWO thin full-width bars at the bottom (between STANDBY and the logo): bar 1 heading -10 -1
    value +1 +10, AUTO pause / resume / skip, ROUTE, MAX SPEED, GEOFENCE; bar 2 nearest ship + CPA, messages, zoom,
    BOAT, TOOLS, Aa, RESET. ROUTE opens its sub-panel just above the bar (again = closed); a bar hides to its handle
    (STANDBY stays); the bars move to the top (under the mode bar) and back."""
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = fake_gw(ap.t)
    rig.size(1366, 768, 1.0)
    sent = []
    orig = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))
    import settings
    try:
        ap.sub_open = None
        ap.render(force=True)
        b1, b2 = op(ap, "bar1"), op(ap, "bar2")
        sb, lg = op(ap, "standby"), op(ap, "logo")
        W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
        placed = b1 and b2 and b1["r"][3] <= b2["r"][1] and b2["r"][3] >= H - 20 and b1["r"][0] >= sb["r"][2] and \
            b1["r"][2] <= lg["r"][0] and b1["r"][2] - b1["r"][0] > W * 0.55
        t1 = ["hdg_-10", "hdg_-1", "hdg_value", "hdg_+1", "hdg_+10", "auto_pause", "auto_resume", "auto_skip",
              "b_route", "b_maxkn", "b_fence"]
        t2 = ["nearest", "message", "bar", "bar_boat", "text_size", "reset_layout"]
        in1 = all(op(ap, t) and overlay.inside(op(ap, t)["r"], b1["r"], 1.5) for t in t1)
        in2 = all(op(ap, t) and overlay.inside(op(ap, t)["r"], b2["r"], 1.5) for t in t2)
        near = A.T("ov.nearest", name="X", nm="0", cpa="0").split(":")[0] in op(ap, "nearest")["s"]
        click_r(ap, op(ap, "hdg_+10")["r"])
        hdg = sent == [(proto.T_HEADING_ADJ, proto.p_heading(10))]
        sent.clear()
        click_r(ap, op(ap, "b_route")["r"])
        sp = [p for p in ap.ov.panels if p.key == "sub_route"]
        sub_ok = bool(sp) and sp[0].r[3] <= op(ap, "bar1")["r"][1] + 1 and sp[0].r[0] < W * 0.3
        click_r(ap, op(ap, "b_route")["r"])
        sub_closed = not [p for p in ap.ov.panels if p.key == "sub_route"]
        click_r(ap, op(ap, "bar1_hide")["r"])
        hidden = op(ap, "hdg_+10") is None and op(ap, "bar1") is not None and op(ap, "standby") is not None and \
            1 in ap.bars_cfg()["hidden"]
        click_r(ap, op(ap, "bar1_hide")["r"])
        shown = op(ap, "hdg_+10") is not None
        click_r(ap, op(ap, "bar_dock")["r"])
        mb = op(ap, "mode_STANDBY")
        top = settings.get("s2_bars")["dock"] == "top" and op(ap, "bar1")["r"][1] >= mb["r"][1] and \
            op(ap, "bar2")["r"][3] < H * 0.3 and op(ap, "standby") is not None
        fits_top = not layout_problems(ap)
        click_r(ap, op(ap, "bar_dock")["r"])
        back = settings.get("s2_bars")["dock"] == "bottom" and op(ap, "bar2")["r"][3] >= H - 20
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            ap.sub_open = "route"
            ap.render(force=True)
            canvaspic.save(ap.cv, os.path.join(shots, "screen2_bars_el.png"))     # (the canvas itself: no desktop needed)
            ap.sub_open = None
            ap.render(force=True)
        fits = not layout_problems(ap)
    finally:
        ap.cmd = orig
        settings.put("s2_bars", {"hidden": [], "dock": "bottom"})
        ap.sub_open = None
    report("S22 bars (13n1): bar 1 + bar 2 full width between STANDBY and the logo with their items, heading +10 "
           "sent, ROUTE sub-panel above the bar (again = closed), a bar hides to its handle (STANDBY stays), dock top "
           "and back", placed and in1 and in2 and near and hdg and sub_ok and sub_closed and hidden and shown and
           top and fits_top and back and fits and not rig.errors,
           "placed %s; bar 1 items %s; bar 2 items %s; nearest %s; +10 %s; sub-panel %s / closed %s; hide %s / show "
           "%s; top %s (fits %s); back %s; fits %s; errors %s" % (
               placed, in1, in2, near, hdg, sub_ok, sub_closed, hidden, shown, top, fits_top, back, fits,
               rig.errors[:2]))


def s23_drag(rig):
    """Step 13n1: a panel header DRAGGED moves the panel (saved per PC), never over the boat (pushed sideways), a
    header click without moving still collapses; RESET TO DEFAULT = every panel back in its column + open."""
    ap = rig.ap
    import settings
    rig.size(1366, 768, 1.0)
    ap.render(force=True)
    W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
    tp = [p for p in ap.ov.panels if p.key == "traffic"][0]
    r0 = tp.r
    head = (r0[0], r0[1], r0[2], r0[1] + tp.head_h)
    click_r(ap, head, dx=-(r0[0] - W * 0.25), dy=-(r0[1] - H * 0.15))
    tp = [p for p in ap.ov.panels if p.key == "traffic"][0]
    moved = abs(tp.r[0] - r0[0]) > 50 and "traffic" not in ap.ov.collapsed
    saved = "traffic" in (settings.get("s2_panel_pos") or {})
    # drag onto the boat: pushed out of the keep-out circle
    r1 = tp.r
    click_r(ap, (r1[0], r1[1], r1[2], r1[1] + tp.head_h), dx=W / 2 - (r1[0] + r1[2]) / 2, dy=H / 2 - r1[1])
    tp = [p for p in ap.ov.panels if p.key == "traffic"][0]
    free = not overlay.rect_hits_circle(tp.r, ap.mapwin.keepout(W, H))
    # a click on a header without moving = collapse
    st = [p for p in ap.ov.panels if p.key == "status"][0]
    click_r(ap, (st.r[0], st.r[1], st.r[2], st.r[1] + st.head_h))
    coll = "status" in ap.ov.collapsed
    ap.reset_layout()
    tp = [p for p in ap.ov.panels if p.key == "traffic"][0]
    reset = not ap.ov.pos and not ap.ov.collapsed and not settings.get("s2_panel_pos") and \
        tp.r[2] > W - 30 and not layout_problems(ap)
    report("S23 panels draggable (13n1): a dragged header moves the panel + saved per PC, never over the boat, a "
           "click still collapses, RESET TO DEFAULT puts everything back", moved and saved and free and coll and
           reset and not rig.errors,
           "moved %s (%s -> %s) saved %s; boat free %s; click collapses %s; reset %s; errors %s" % (
               moved, [int(v) for v in r0], [int(v) for v in tp.r], saved, free, coll, reset, rig.errors[:2]))


def s24_battle_short(rig):
    """Step 13n1: BATTLE SHORT on the mode bar opens the window; any code = INVALID, nothing is sent to the boat /
    the gateway, nothing changes; the station log line "battle short attempt"; the tooltip on hover."""
    ap = rig.ap
    rig.gateway_like(True)
    rig.size(1366, 768, 1.0)
    ap.render(force=True)
    sent = []
    orig = (ap.cmd, ap._send_cmd, ap.link.send_msg, ap.link.send)
    ap.cmd = lambda *a, **k: sent.append(("cmd", a))
    ap._send_cmd = lambda *a, **k: sent.append(("send_cmd", a))
    ap.link.send_msg = lambda *a, **k: sent.append(("msg", a))
    ap.link.send = lambda *a, **k: sent.append(("send", a)) or 0
    try:
        b = op(ap, "battle_short")
        e = ev_at(b["r"])
        ap.cv_hover(e)
        tip = op(ap, "bs_tooltip")
        tip_ok = tip is not None and tip["s"] == A.T("mb.tooltip")
        click_r(ap, b["r"])
        w = getattr(ap, "bs_win", None)
        opened = w is not None and w.alive()
        mode0 = ap.t["mode"]
        for code in ("", "1234", "BATTLE SHORT", "9" * 64):
            w.var.set(code)
            w.authorise()
        rig.pump(0.8)
        refused = w.msg.cget("text") == A.T("bs.invalid") and w.attempts == 4
        logged = sum(1 for t in ap.ev_texts() if t == "battle short attempt") >= 4
        # (the station's own periodic traffic - heartbeat, the screen-1 beat - is not BATTLE SHORT's)
        sent = [x for x in sent if not (x[0] == "send" and x[1][:1] == (proto.T_HEARTBEAT,)) and
                not (x[0] == "msg" and (x[1][0] or {}).get("t") == "driver")]
        nothing = not sent and ap.t["mode"] == mode0
        w.close()
        rig.pump(0.2)
        closed = not w.alive()
        ap.cv_hover(type("E", (), {"x": 5, "y": 5})())
    finally:
        ap.cmd, ap._send_cmd, ap.link.send_msg, ap.link.send = orig
    report("S24 BATTLE SHORT (13n1): opens from the mode bar, every code INVALID, nothing sent / changed, the log "
           "line, the tooltip", tip_ok and opened and refused and logged and nothing and closed and not rig.errors,
           "tooltip %s; opened %s; refused %s; logged %s; nothing sent %s (%s); closed %s; errors %s" % (
               tip_ok, opened, refused, logged, nothing, sent[:2], closed, rig.errors[:2]))


class ViewStream:
    """A stand-in for video.VideoStream (no ffmpeg): records its size / frame rate, delivers a test picture."""
    made = []

    def __init__(self, url, w=640, h=360, name="", fps=None):
        self.url, self.w, self.h, self.name, self.fps = url, w, h, name, fps
        self.frame = bytes([150, 190, 230]) * (w * h * 2 // 5) + bytes([20, 60, 110]) * (w * h - w * h * 2 // 5)
        self.live = True
        ViewStream.made.append(self)

    def start(self):
        return self

    def stop(self):
        self.live = False

    def frozen(self):
        return not self.live

    def ppm(self):
        return b"P6 %d %d 255\n" % (self.w, self.h) + self.frame


def s25_camview(rig):
    """Step 13n2 (owner 2026-10-31 E2): the copilot's camera view on screen 2."""
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = fake_gw(ap.t, captain=False)
    rig.size(1366, 768, 1.0)
    msgs = []
    ap.link.send_msg = msgs.append
    cvm = lambda: [m for m in msgs if m.get("t") == "cam_view"]       # noqa: E731 (the heartbeats go there too)
    ap.link.video_url = lambda d, n: ("rtsp://gw/%s" % d["streams"][n]) if n in d.get("streams", {}) else None
    real_vs = A.video.VideoStream
    A.video.VideoStream = ViewStream
    presses = []
    real_press = ap.mapwin.press
    ap.mapwin.press = lambda x, y: presses.append((x, y))
    try:
        ap.render(force=True)
        b = op(ap, "cv_open")
        click_r(ap, b["r"])
        opened = ap.camview == "FRONT-1" and cvm()[-1:] == [{"t": "cam_view", "cam": "FRONT-1"}]
        # the gateway's answer for a copilot: FRONT-1 LOW + view_light -> decoded 960 x 540 at 10 fps here
        ap.on_cams_ack({"t": "cams_ack", "ok": True, "streams": {"FRONT-1": "cam_front1_lo"}, "levels": {},
                        "view": "FRONT-1", "view_light": True, "view_reduced": False, "notice": ""})
        vs = ap.vstreams.get("FRONT-1")
        light = vs is not None and (vs.w, vs.h, vs.fps) == (960, 540, 10)
        rig.pump(0.5)
        ap.render(force=True)
        imgs = [i for i in ap.cv.find_withtag("camview") if ap.cv.type(i) == "image"]
        top = ap.cv.find_all()[-1] if ap.cv.find_all() else None
        picture = bool(imgs) and "ov" in ap.cv.gettags(top)          # the picture under the panels / bars
        panels = sorted(p.key for p in ap.ov.panels)
        kept = panels == ["alarms"] and op(ap, "standby") is not None and op(ap, "mode_STANDBY") is not None
        cams = all(op(ap, "cv_" + c) for c in ap.CAMVIEW_CAMS) and op(ap, "cv_map") is not None
        hud = [o["s"] for o in ap.ov.ops if o.get("tag") == "cv_hud"]
        hud_ok = len(hud) >= 4 and hud[0].startswith("FRONT-1") and A.T("mode." + ap.t["mode_name"]) in hud
        fits = not layout_problems(ap)
        W, H = ap.cv.winfo_width(), ap.cv.winfo_height()
        ap.cv_press(type("E", (), {"x": W // 2, "y": H // 2})())
        ap.cv_release(type("E", (), {"x": W // 2, "y": H // 2})())
        no_map = presses == []
        click_r(ap, op(ap, "cv_REAR-RIGHT")["r"])
        switched = ap.camview == "REAR-RIGHT" and cvm()[-1] == {"t": "cam_view", "cam": "REAR-RIGHT"}
        ap.on_cams_ack({"t": "cams_ack", "ok": True, "streams": {"FRONT-1": "cam_front1_lo"}, "levels": {},
                        "view": "REAR-RIGHT", "view_light": True, "view_reduced": True,
                        "notice": "VIDEO REDUCED FOR THE CAPTAIN: camera view off"})
        ap.render(force=True)
        reduced = "camview_reduced" in ap.banner_tags
        ap.on_cams_ack({"t": "cams_ack", "ok": True, "streams": {"FRONT-1": "cam_front1_lo"}, "levels": {},
                        "view": "REAR-RIGHT", "view_light": True, "view_reduced": False, "notice": ""})
        ap.render(force=True)
        novideo = "camview_novideo" in ap.banner_tags              # REAR-RIGHT has no stream: no video (yet)
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            click_r(ap, op(ap, "cv_FRONT-1")["r"])
            ap.on_cams_ack({"t": "cams_ack", "ok": True, "streams": {"FRONT-1": "cam_front1_lo"}, "levels": {},
                            "view": "FRONT-1", "view_light": True, "view_reduced": False, "notice": ""})
            rig.pump(0.4)
            ap.render(force=True)
            canvaspic.save(ap.cv, os.path.join(shots, "screen2_camview_el.png"))
        click_r(ap, op(ap, "cv_map")["r"])
        rig.pump(0.2)
        back = ap.camview is None and cvm()[-1] == {"t": "cam_view", "cam": None} and             not ap.cv.find_withtag("camview") and "status" in {p.key for p in ap.ov.panels}
        esc = "camview_close" in ap.root.bind("<Escape>") or bool(ap.root.bind("<Escape>"))
        # a dead front camera: the "no video" warning only (nothing else changes)
        ap.gw_status = dict(fake_gw(ap.t), cameras={"front_live": False, "cameras": []})
        ap.render(force=True)
        warn = "no_video" in ap.banner_tags
        ap.gw_status = dict(fake_gw(ap.t), cameras={"front_live": True, "cameras": []})
        ap.render(force=True)
        no_warn = "no_video" not in ap.banner_tags
        rig.gateway_like(False)
        ap.render(force=True)
        no_btn = op(ap, "cv_open") is None
    finally:
        A.video.VideoStream = real_vs
        ap.mapwin.press = real_press
        ap.camview = None
        ap.cv.delete("camview")
        for n in list(ap.vstreams):
            ap.vstreams.pop(n).stop()
        ap.gw_status = None
        ap.cam_ack = None
        rig.gateway_like(False)
        ap.render(force=True)
    report("S25 camera view (13n2): CAMERA on bar 2 asks the gateway, a copilot's view decoded lighter (960 x 540, "
           "10 fps), the picture under the bars; only the alarms panel + mode bar + STANDBY stay, the 4 cameras + MAP "
           "on bar 1, green HUD basics, no map editing under it, reduced / no-video banners, MAP = back (stream "
           "closed), Esc bound; a dead front camera = the 'no video' warning only; no button without a gateway",
           opened and light and picture and kept and cams and hud_ok and fits and no_map and switched and reduced and
           novideo and back and esc and warn and no_warn and no_btn and not rig.errors,
           "opened %s; light %s; picture %s; panels %s kept %s; cams %s; hud %s; fits %s; map press %s; switched %s; "
           "reduced %s; no video %s; back %s; esc %s; no-video warning %s / gone %s; no button without gateway %s; "
           "errors %s" % (opened, light, picture, panels, kept, cams, hud[:5], fits, presses, switched, reduced,
                          novideo, back, esc, warn, no_warn, no_btn, rig.errors[:2]))


def s26_track(rig):
    """Step 13n3 (owner 2026-10-31 E3): the track on screen 2 (+ the mini map of screen 1)."""
    ap = rig.ap
    import mapview
    import settings
    import track as trackmod
    rig.size(1366, 768, 1.0)
    sent = []
    orig = ap.cmd
    ap.cmd = lambda f, p, w: sent.append((f, p))
    real_boat = ap.boat
    m0 = ap.mapwin.m
    ap.mapwin.m, ap.mapwin.pan, ap.mapwin.edit = 10.0, None, None
    gpx_dir = tempfile.mkdtemp(prefix="trygons_s26_")
    try:
        ap.render(force=True)
        btns = op(ap, "tracks") is not None and op(ap, "track_new") is not None and \
            overlay.inside(op(ap, "tracks")["r"], op(ap, "bar2")["r"], 1.5) and not layout_problems(ap)
        lv = ap.track_select()
        key0 = lv.key
        lv.save_as(gpx_dir, "earlier")                               # (the sim's own points so far: elsewhere)
        la, lo = ap.t["est_lat"], ap.t["est_lon"]
        now = time.time()
        for i in range(200):          # 200 points from 1 km south up to the boat, 5 m each, 4.0 .. 7.98 kn
            lv.feed(now - 400 + 2 * i, la - (199 - i) * 5.0 / 111320.0, lo, 4.0 + i * 0.02, 0.0)
        drawn = lambda tag: [i for i in ap.cv.find_withtag(tag) if ap.cv.type(i) == "line"]   # noqa: E731
        ap.mapwin.redraw()
        th = mapview.THEMES[theme.map_theme()]
        live_lines = drawn("track")
        colour = bool(live_lines) and all(ap.cv.itemcget(i, "fill") == th["track"] for i in live_lines) and \
            th["track"] in ("#ffffff", "#000000")
        full = len(ap.track) == 200                                  # every point kept (no 5000 limit any more)
        # a click on the line = the time + speed there; a click away = the mark goes
        P = ap.mapwin.P()[0]
        x, y = P(la - 10 * 5.0 / 111320.0, lo)                        # point 189: ~ 5 px south of the boat
        ap.cv_press(type("E", (), {"x": x + 2, "y": y})())
        ap.cv_release(type("E", (), {"x": x + 2, "y": y})())
        pk = ap.track_pick
        want_kn = "%.1f" % (4.0 + 189 * 0.02)
        click_ok = pk is not None and want_kn in pk["text"] and \
            time.strftime("%H:%M", time.localtime(now - 400 + 2 * 189)) in pk["text"] and \
            bool(ap.cv.find_withtag("track_pick"))
        ap.cv_press(type("E", (), {"x": x + 90, "y": y})())
        ap.cv_release(type("E", (), {"x": x + 90, "y": y})())
        away = ap.track_pick is None and not ap.cv.find_withtag("track_pick")
        # NEW TRACK: YES / NO first; NO keeps it, YES saves it as <date>_<HHMM>_<boat> and starts a new line
        click_r(ap, op(ap, "track_new")["r"])
        asked = getattr(ap, "yn_win", None) is not None
        ap.yn_answer(False)
        kept = len(ap.track) == 200 and not os.listdir(ap.tracks.saved_dir) if os.path.isdir(ap.tracks.saved_dir) \
            else len(ap.track) == 200
        click_r(ap, op(ap, "track_new")["r"])
        ap.yn_answer(True)
        files = [f for f in os.listdir(ap.tracks.saved_dir) if f.endswith(".jsonl")]
        stamp = time.strftime("%Y-%m-%d_%H%M", time.localtime(now - 400))
        saved = len(files) == 1 and files[0].startswith(stamp + "_") and len(ap.track) <= 1
        path = os.path.join(ap.tracks.saved_dir, files[0]) if files else None
        # per boat: a boat of boats/ = its own (empty) track, never the address's; back = the address's again
        import boats
        bt = boats.load()[0]
        ap.boat = bt
        rig.pump(0.4)
        mine = ap.tracks.live.key == "boat_" + bt.name and all(abs(a - la) < 0.01 for a, _ in ap.track)
        n_boat = len(ap.track)
        ap.boat = real_boat
        rig.pump(0.4)
        back = ap.tracks.live.key == key0
        # TRACKS: the list, SHOW = grey dashed + the date label, HIDE, REMOVE (the file stays), GPX out + in
        win = ap.open_tracks()
        rows = list(win.tree.get_children())
        listed = rows == [path]
        win.tree.selection_set(rows)
        win.show()
        ap.mapwin.redraw()
        old = drawn("track_old")
        shown = bool(old) and all(ap.cv.itemcget(i, "fill") == th["track_old"] and ap.cv.itemcget(i, "dash")
                                  for i in old) and \
            any(ap.cv.type(i) == "text" and ap.track_label(ap.tracks.load(path)) == ap.cv.itemcget(i, "text")
                for i in ap.cv.find_withtag("track_old")) and settings.get("tracks_shown") == [[path, True]]
        win.hide()
        ap.mapwin.redraw()
        hid = not drawn("track_old") and path in ap.track_shown
        win.show()
        out = win.export(gpx_dir)
        exported = bool(out) and os.path.exists(out[0])
        imp = win.import_gpx(out[0]) if exported else None
        imported = imp is not None and imp in ap.track_shown and len(win.tree.get_children()) == 2
        bad = os.path.join(gpx_dir, "bad.gpx")
        with open(bad, "w") as f:
            f.write("<<< not a gpx")
        refused = win.import_gpx(bad) is None and A.T("track.err_not_xml") in win.msg.cget("text")
        win.tree.selection_set(list(win.tree.get_children()))
        win.remove()
        removed = not ap.track_shown and os.path.exists(path) and imp and os.path.exists(imp)
        # the mini map of screen 1 draws the live track too
        ap.open_driver()
        rig.pump(0.8)
        lv = ap.tracks.live
        for i in range(30):
            lv.feed(time.time() + 10 + 2 * i, la + i * 5.0 / 111320.0, lo + i * 5.0 / 111320.0, 5.0, 45.0)
        ap.driver.draw_map()
        mini = any(ap.driver.map.type(i) == "line" and ap.driver.map.itemcget(i, "fill") ==
                   mapview.THEMES["dark"]["track"] for i in ap.driver.map.find_all())
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            ap.track_show(path, True)
            ap.mapwin.zoom(0.5)
            ap.render(force=True)
            ap.mapwin.redraw()
            canvaspic.save(ap.cv, os.path.join(shots, "screen2_tracks_el.png"))
            ap.track_remove(path)
            ap.mapwin.zoom(2.0)
        nothing = sent == []
        fits = not layout_problems(ap)
    finally:
        ap.cmd = orig
        ap.boat = real_boat
        ap.mapwin.m = m0
        if ap.tracks_win is not None:
            ap.tracks_win.close()
        ap.track_shown.clear()
        ap.track_pick = None
        settings.put("tracks_shown", [])
        ap.render(force=True)
    report("S26 track (13n3): TRACKS / NEW TRACK on bar 2; the live track white / black by the background, every point "
           "kept; a click on the line = time + speed there, away = gone; NEW TRACK asks YES / NO (NO keeps it), YES "
           "saves <date>_<HHMM>_<boat> + a new line; one track per boat; TRACKS list SHOW (grey dashed + date) / HIDE / "
           "REMOVE (file stays) / GPX export + import (a bad file refused); the screen 1 mini map; nothing sent",
           btns and colour and full and click_ok and away and asked and kept and saved and mine and back and listed and
           shown and hid and exported and imported and refused and removed and mini and nothing and fits and
           not rig.errors,
           "buttons %s; colour %s; 200 kept %s; click %s (%s); away %s; asked %s; NO kept %s; saved %s (%s); own boat "
           "%s (%d); back %s; listed %s; shown %s; hidden %s; GPX out %s in %s; bad refused %s; removed %s; mini map "
           "%s; sent %s; fits %s; errors %s" % (
               btns, colour, full, click_ok, pk and pk["text"], away, asked, kept, saved, files, mine, n_boat, back,
               listed, shown, hid, exported, imported, refused, removed, mini, sent, fits, rig.errors[:2]))


def s27_screenrec(rig):
    """Step 13n4 (owner 2026-10-31 E4): the recording group on bar 2 [SCR · 5' · 1h] (screen 1's bar in test_driver /
    test_screenrec): drawn inside bar 2, fits at 1366x768, SCR lit ("●SCR") while recording, the clicks reach the
    station, KEEP sends ONE gateway message and nothing to the boat."""
    ap = rig.ap
    rig.size(1366, 768, 1.0)
    sent, msgs = [], []
    orig_cmd, orig_start, orig_rec = ap.cmd, ap.scr.start, ap.scr.recording
    ap.cmd = lambda f, p, w: sent.append((f, p))
    state = {"on": False}
    ap.scr.start = lambda: (state.update(on=True) or (True, "scr.on"))
    ap.scr.recording = lambda: state["on"]
    try:
        ap.render(force=True)
        b = {t: op(ap, t) for t in ("scr", "keep5", "keep60")}
        inside = all(o is not None and overlay.inside(o["r"], op(ap, "bar2")["r"], 1.5) for o in b.values())
        label_off = b["scr"] is not None and b["scr"]["s"] == i18n.T("scr.btn")
        [it for it in ap.bar_items(2) if it[0] == "btn" and it[1] == "scr"][0][3]()     # the bar's own action
        ap.render(force=True)
        on = op(ap, "scr")
        label_on = on is not None and on["s"] == "●SCR"
        ap.scr.stop = lambda why=None: state.update(on=False)
        ap.scr_toggle()
        ap.render(force=True)
        off_again = op(ap, "scr")["s"] == i18n.T("scr.btn")
        rig.gateway_like(True)
        real_send = ap.link.send_msg
        ap.link.send_msg = lambda m: msgs.append(m)
        eid = ap.keep_save(60)
        ap.link.send_msg = real_send
        one = msgs == [{"t": "save_keep", "min": 60, "id": eid}] and not sent
        fits = not layout_problems(ap)
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            state["on"] = True
            ap.render(force=True)
            canvaspic.save(ap.cv, os.path.join(shots, "screen2_screenrec_el.png"))
            state["on"] = False
    finally:
        ap.cmd, ap.scr.start, ap.scr.recording = orig_cmd, orig_start, orig_rec
        ap.keep_pending.clear()
        rig.gateway_like(False)
    report("S27 recording group (13n4): SCR · 5' · 1h inside bar 2 at 1366x768, ●SCR lit while recording, off again, "
           "KEEP = ONE gateway message, nothing to the boat",
           inside and label_off and label_on and off_again and one and fits and not rig.errors,
           "inside bar 2 %s; label off %s / on %s / off again %s; KEEP sent %s, to the boat %s; fits %s; errors %s" % (
               inside, label_off, label_on, off_again, msgs, sent, fits, rig.errors[:2]))


def main():
    exe = os.path.join(HERE, "..", "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    rig = Rig()
    try:
        for fn in (s1_matrix, s2_collapsed, s3_roles, s4_maneuver, s5_map, s6_edit, s7_link, s8_boats, s9_look,
                   s10_tools, s11_greek, s12_alarms, s13_boat_line, s14_relays, s15_dp, s17_link_time, s18_uncrewed_hdg, s19_override, s20_slam_restore,
                   s21_mode_bar, s22_bars, s23_drag, s24_battle_short, s25_camview, s26_track, s27_screenrec, s16_silence):
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
