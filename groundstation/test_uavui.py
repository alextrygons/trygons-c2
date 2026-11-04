"""The UAV magazine on the ground station (CLAUDE.md V, steps U2 / U3; DISPLAY + requests only).

  UV1 the owner's UAV picture: assets/uav.png + the small sizes (tools/make_uav_icons.py: 16 / 24 / 32 / 48 / 64 px,
      square, transparent corners), uavpics picks the size, Tk loads it; the USV placeholder draws (W65)
  UV2 a slot row's texts (pure): no cable, empty, not registered, reading, ready, NOT launchable + the gateway's
      reason in Greek, ARMED, the NEXT marker on slot 1; no English word left in the Greek rows
  UV3 the Tools page «ΜΑΓΓΕΜΑ UAV» in the real station: the page exists, 6 slot rows with the picture, filled from
      gw_status "uav", REGISTER sends {"t": "uav_register", "slot"} only (nothing to the AURIX), disabled while not
      connected / without the magazine; the station hooks gw_status "uav" + "uav_ack"
  UV4 uav_ack: the answer and every problem shown in the screen language (from_en templates gw.uav_*)
  UV5 (U3) the launch on the page: hatch + sequence + notices in Greek, PREPARE / STOP / HATCH CLOSE enabled by state
      and sending only their station messages; a row's route / mission / flight
  UV6 (U3) "Send route": the vertical target box with the pictures, the map's right-click menu, uav_route
  UV7 (U3) the boat file's uav hatch_relay (validated) = a LAMP on every relay panel, a press sends nothing
  UV8 (U4) the UAVs on the map: the picture turned with the heading, own track style, label, a click = its video
  UV9 (U4) a UAV's video in screen 2's camera view: only that stream asked for, bar 1, its HUD + AI boxes, close
  UV10 (U4) the battery minimum's local CAUTION once per stage; the router OFFLINE on both screens' notices
  UV11 (U4) the flight logs: list + CSV / GPX export to a file
  UV12 (U5) uavfly.py: the stick -> x / y / z / r (lever neutral zone), take / give / answers, STANDBY rule
  UV13 (U5) the station flying a UAV: no rjoy, the selector without the stick, the banner on both screens, RETURN TO
       BOAT, the boat leaving STANDBY, the captain's takeover message (owner W61), the page's fly buttons
  UV14 (U5) screen 1's UAV HUD (uavhud.py): green values, the arrow to the boat, the boat's instruments hidden

Run:  python test_uavui.py      (exit code 0 = all passed)
"""
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_screen2 as ts  # noqa: E402  (its environment: temp settings, sound off, the Rig on the simulator)
import i18n  # noqa: E402
import theme  # noqa: E402
import uavpanel  # noqa: E402
import uavpics  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
results = []


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def slot(n, **kw):
    s = {"slot": n, "usb": True, "present": True, "registered": True, "name": "UAV-%d" % n, "sysid": 10 + n,
         "fw": "4.5.7", "battery_pct": 97, "battery_v": 16.7, "gps_fix": 1, "sats": 0, "armed": False, "vtx": "R%d" % n,
         "elrs": "tx%d" % n, "mission": False, "problems": [], "launchable": True, "why": "", "registering": False,
         "assist": {"age_s": 12, "what": "time+pos", "ok": True, "why": ""}}
    s.update(kw)
    return s


def status():
    return {"enabled": True, "state": "ACTIVE", "why": "", "sim": True, "next": 1,
            "slots": [slot(1), slot(2, launchable=False, why="check failed: output SERVO9_FUNCTION = 28: only the 4 "
                                                               "motors may be configured",
                               problems=["output SERVO9_FUNCTION = 28: only the 4 motors may be configured",
                                         "GRIP_ENABLE = 1 (must be 0: no release / aux function)"]),
                      slot(3, registered=False, launchable=False, why="not registered", name=None, sysid=None,
                           present=False, assist=None),
                      slot(4, registering=True), slot(5, armed=True),
                      slot(6, usb=False, present=False, registered=False, launchable=False, why="not registered")],
            "router": {"stats": {"rx": 100, "tx": 50, "denied_in": 3, "denied_out": 0}, "starts": 1}}


def uv1():
    from PIL import Image
    sizes, transparent = {}, True
    for s in uavpics.SIZES:
        p = os.path.join(theme.ASSETS, "uav_%d.png" % s)
        im = Image.open(p).convert("RGBA")
        sizes[s] = im.size
        transparent &= im.getpixel((0, 0))[3] == 0 and im.getpixel((s - 1, s - 1))[3] == 0
    big = Image.open(uavpics.path()).size
    pick = [os.path.basename(uavpics.path(p)) for p in (10, 20, 32, 40, 100)]
    import tkinter as tk
    root = tk.Tk()
    try:
        ph = uavpics.photo(32, master=root)
        c = tk.Canvas(root, width=80, height=80)
        uavpics.draw_usv(c, 40, 40, 64)
        drawn = len(c.find_withtag("usv"))
        ok_tk = ph is not None and ph.width() == 32
    finally:
        root.destroy()
    report("UV1 the owner's UAV picture (assets/uav.png %s) + 16 / 24 / 32 / 48 / 64 px (square, transparent), the "
           "size chosen per use, loaded by Tk; the USV placeholder drawn (W65)",
           all(sizes[s] == (s, s) for s in uavpics.SIZES) and transparent and big == (1024, 1024) and
           pick == ["uav_16.png", "uav_24.png", "uav_32.png", "uav_48.png", "uav_64.png"] and ok_tk and drawn == 2,
           "sizes %s, transparent corners %s, picks %s, Tk %s, USV items %d" % (sizes, transparent, pick, ok_tk, drawn))


def uv2():
    i18n.set_lang("el", save=False)
    st = status()
    rows = [uavpanel.row_view(s, st["next"]) for s in st["slots"]]
    eng = {}
    for r in rows:
        for k in ("head", "state", "details", "assist"):
            for w in ts.english_words(r[k]):
                eng.setdefault(w, r[k][:60])
        for p in r["problems"]:
            for w in ts.english_words(p):
                eng.setdefault(w, p[:60])
    i18n.set_lang("en", save=False)
    en = [uavpanel.row_view(s, st["next"]) for s in st["slots"]]
    i18n.set_lang("el", save=False)
    ok = ("ΕΠΟΜΕΝΟ" in rows[0]["head"] and "ΕΠΟΜΕΝΟ" not in rows[1]["head"] and rows[0]["kind"] == "good" and
          rows[0]["state"].startswith("έτοιμο") and rows[1]["kind"] == "caution" and
          "ΔΕΝ εκτοξεύεται" in rows[1]["state"] and "επιτρέπονται μόνο οι 4 κινητήρες" in rows[1]["state"] and
          "χωρίς καταχώριση" in rows[2]["state"] and "κανένα UAV" in rows[2]["state"] and
          rows[3]["state"] == "ανάγνωση ..." and not rows[3]["can_register"] and rows[4]["state"].startswith("ΟΠΛΙΣΜΕΝΟ")
          and rows[5]["kind"] == "off" and not rows[5]["can_register"] and en[1]["state"].startswith("NOT launchable")
          and not eng)
    report("UV2 a slot row (pure): NEXT on slot 1, ready / NOT launchable + the gateway's reason in Greek / not "
           "registered + empty / reading / ARMED / no cable (no REGISTER); no English word in the Greek rows", ok,
           "states %s; English left %s" % ([r["state"][:40] for r in rows], sorted(eng.items())[:6]))


class FakeLink:
    connected = True

    def __init__(self):
        self.sent = []

    def send_msg(self, obj):
        self.sent.append(obj)


class FakeApp:
    def __init__(self):
        self.link = FakeLink()


def uv3_4(rig):
    ap = rig.ap
    i18n.set_lang("el", save=False)
    src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    hooks = ('self.uav_page.update(d.get("uav"))' in src and 'elif kind == "uav_ack":' in src and
             "self.uav_page.on_ack(d)" in src and "self.uav_page.update(None)" in src)
    has_tab = "tab.uav" in ap.tab_keys and getattr(ap, "uav_page", None) is not None
    ap.uav_page.update(status())
    ap.open_tab("tab.uav")
    rig.pump(0.3)
    pg_real_rows = len(ap.uav_page.rows)
    pics = sum(1 for labels, _b in ap.uav_page.rows if str(labels["head"].master.master.winfo_children()[0].cget(
        "image")))
    ap.tools.withdraw()
    # a page on a fake app: what REGISTER sends, disabled states, the answers
    import tkinter as tk
    top = tk.Toplevel(rig.root)
    try:
        fa = FakeApp()
        pg = uavpanel.Page(tk.Frame(top), fa)
        pg.fr.pack()
        pg.update(None)
        off = all(b.cget("state") == "disabled" for _l, b in pg.rows) and "σύνδεση" in pg.head.cget("text").lower()
        pg.update({"enabled": False})
        no_mag = "δεν έχει μαγγέμα" in pg.head.cget("text")
        pg.update(status())
        states = [b.cget("state") for _l, b in pg.rows]
        pg.rows[1][1].invoke()
        pg.rows[0][1].invoke()
        sent = list(fa.link.sent)
        fa.link.connected = False
        pg.rows[0][1].invoke()
        not_conn = len(fa.link.sent) == 2 and "Πρώτα σύνδεση" in pg.msg.cget("text")
        fa.link.connected = True
        report("UV3 the Tools page «ΜΑΓΓΕΜΑ UAV» in the real station: 6 rows with the UAV picture, the station hooks "
               "gw_status \"uav\" + uav_ack (+ cleared on disconnect); REGISTER sends only {t: uav_register, slot}; "
               "disabled while not connected / reading / without a cable / without the magazine",
               hooks and has_tab and pg_real_rows == 6 and pics == 6 and off and no_mag and
               states == ["normal", "normal", "normal", "disabled", "normal", "disabled"] and
               sent == [{"t": "uav_register", "slot": 2}, {"t": "uav_register", "slot": 1}] and not_conn and
               not rig.errors,
               "hooks %s tab %s rows %d pictures %d; off %s no magazine %s; buttons %s; sent %s; not connected %s; "
               "errors %s" % (hooks, has_tab, pg_real_rows, pics, off, no_mag, states, sent, not_conn, rig.errors[:2]))
        pg.on_ack({"t": "uav_ack", "slot": 2, "ok": False, "code": 122,
                   "text": "slot 2 not launchable: output SERVO9_FUNCTION = 28: only the 4 motors may be configured",
                   "problems": ["output SERVO9_FUNCTION = 28: only the 4 motors may be configured",
                                "RC_OPTIONS: the RC receiver is not ignored (bit 0 'Ignore RC Receiver' must be set)",
                                "sysid 12 is also slot 1", "FS_GCS_ENABLE is 1 (must be 0)"]})
        msg, prob = pg.msg.cget("text"), pg.prob.cget("text")
        eng = [w for t in (msg, prob) for w in ts.english_words(t)]
        pg.on_ack({"t": "uav_ack", "slot": 1, "ok": True, "code": 0, "text": "slot 1 registered: UAV-1", "problems": []})
        ok_msg = pg.msg.cget("text")
        report("UV4 uav_ack in the screen language: the refusal + every problem translated (gw.uav_* templates), the "
               "registration answer; no English left",
               "θέση 2 δεν εκτοξεύεται" in msg and "επιτρέπονται μόνο οι 4 κινητήρες" in prob and
               "ο δέκτης RC δεν αγνοείται" in prob and "είναι και στη θέση 1" in prob and
               "FS_GCS_ENABLE είναι 1 (πρέπει 0)" in prob and not eng and "καταχωρίστηκε" in ok_msg and pg.prob.cget(
                   "text") == "",
               "message %r; problems %r; English %s; ok %r" % (msg[:80], prob[:160], eng, ok_msg))
    finally:
        top.destroy()


def u3_status(**launch):
    st = status()
    st["slots"][0].update(route=2, mission=True)
    st["slots"][4].update(flying="mission")
    st["hatch"] = {"state": "CLOSED", "why": "", "relay": 8, "open_sw": False, "closed_sw": True, "switch_ok": True}
    st["launch"] = dict({"state": "IDLE", "why": "", "launched": [], "cur": None, "phase": ""}, **launch)
    st["notices"] = [{"t": 1.0, "text": "UAV launch stopped: HATCH FAULT: both hatch switches closed",
                      "level": "caution"}]
    return st


def uv5(rig):
    """The launch part of the «ΜΑΓΓΕΜΑ UAV» page (U3): hatch + sequence + PREPARE / STOP / HATCH CLOSE requests."""
    import tkinter as tk
    i18n.set_lang("el", save=False)
    idle = uavpanel.launch_view(u3_status())
    run = uavpanel.launch_view(dict(u3_status(state="RUN", cur=2, phase="CLIMB", launched=[1])))
    st_f = u3_status()
    st_f["hatch"].update(state="FAULT", why="both hatch switches closed")
    fault = uavpanel.launch_view(st_f)
    st_o = u3_status(state="STOPPED", why="UAV 1 did not leave the hatch", keep_open=True)
    st_o["hatch"].update(state="OPEN")
    opened = uavpanel.launch_view(st_o)
    eng = [w for v in (idle, run, fault, opened) for t in (v["hatch"], v["seq"]) for w in ts.english_words(t)]
    top = tk.Toplevel(rig.root)
    try:
        fa = FakeApp()
        pg = uavpanel.Page(tk.Frame(top), fa)
        pg.update(u3_status())
        b_idle = (pg.b_prep.cget("state"), pg.b_stop.cget("state"), pg.b_close.cget("state"))
        pg.b_prep.invoke()
        pg.update(u3_status(state="RUN", cur=1, phase="GPS"))
        b_run = (pg.b_prep.cget("state"), pg.b_stop.cget("state"), pg.b_close.cget("state"))
        pg.b_stop.invoke()
        pg.update(st_o)
        pg.b_close.invoke()
        rows = [lb["details"].cget("text") for lb, _b in pg.rows]
        sent = list(fa.link.sent)
        # U3 review M3: a single operator answers his OWN YES / NO first (captain_confirm), only then the request
        fa.single_op, asked = True, []
        fa.captain_confirm = lambda what, on_yes: asked.append((what, on_yes))
        fa.link.sent.clear()
        pg.update(u3_status())
        pg.b_prep.invoke()
        so_before = list(fa.link.sent)
        if asked:
            asked[0][1]()
        so_after = list(fa.link.sent)
    finally:
        top.destroy()
    so_ok = (len(asked) == 1 and so_before == [] and so_after == [{"t": "uav_launch_prepare", "confirm": "self"}])
    report("UV5 the launch on the «ΜΑΓΓΕΜΑ UAV» page: hatch state + sequence + the gateway's notices in Greek; PREPARE "
           "LAUNCH only idle with the hatch CLOSED / OPEN, STOP only while it runs, HATCH CLOSE after a stop with the "
           "hatch open; they send ONLY uav_launch_prepare / uav_launch_stop / uav_hatch_close (the gateway asks the "
           "captain); a row shows its route / mission / flight",
           "ΚΛΕΙΣΤΗ" in idle["hatch"] and idle["can_prepare"] and not idle["can_stop"] and not idle["can_close"] and
           run["can_stop"] and not run["can_prepare"] and "UAV 2" in run["seq"] and "ΣΦΑΛΜΑ" in fault["hatch"] and
           fault["hatch_kind"] == "caution" and not fault["can_prepare"] and opened["can_close"] and
           "ΣΤΑΜΑΤΗΣΕ" in opened["seq"] and "σταμάτησε" in idle["notices"][0] and not eng and
           b_idle == ("normal", "disabled", "disabled") and b_run == ("disabled", "normal", "disabled") and
           sent == [{"t": "uav_launch_prepare"}, {"t": "uav_launch_stop"}, {"t": "uav_hatch_close"}] and
           "αποστολή φορτωμένη" in rows[0] and "εκτελεί" in rows[4] and so_ok,
           "single operator %s; idle %s; run %s; fault %s; stopped open %s; English %s; buttons %s / %s; sent %s; rows %s" % (so_ok, 
               idle, run["seq"], fault["hatch"], opened["can_close"], eng, b_idle, b_run, sent, rows[0][-40:]))


def uv6(rig):
    """The "Send route" target box + the map's right-click menu (U3, owner H2)."""
    import tkinter as tk
    import uavroute
    i18n.set_lang("el", save=False)
    st = u3_status()
    tg = uavroute.targets(st)
    keys = [t["key"] for t in tg]
    none = uavroute.targets(None)
    menu = uavroute.menu_items(st, (37.82, 23.60))
    menu_nopos = uavroute.menu_items(st, None)
    msg = uavroute.route_msg("next", [(37.8211111111, 23.6), {"lat": 37.83, "lon": 23.61, "alt": 70}])
    chosen = []
    box = uavroute.TargetBox(rig.root, st, chosen.append)
    rig.pump(0.2)
    btns = [w for row in box.top.winfo_children() if isinstance(row, tk.Frame) for w in row.winfo_children()
            if isinstance(w, tk.Button)]
    pics = [w for row in box.top.winfo_children() if isinstance(row, tk.Frame) for w in row.winfo_children()
            if isinstance(w, (tk.Label, tk.Canvas))]
    texts = [b.cget("text") for b in btns]
    btns[1].invoke()
    # the real station: route_target sends the map route to the gateway, "boat" = the normal upload
    ap = rig.ap
    sent = []

    class L:
        connected = True

        def send_msg(self, m):
            sent.append(m)
    old_link = ap.link
    import gwclient

    class GL(gwclient.GatewayLink):
        def __init__(self):
            pass
        connected = True

        def send_msg(self, m):
            sent.append(m)
    try:
        ap.link = GL()
        ap.prog = {"final": 0, "wps": [{"lat": 37.821, "lon": 23.601, "kn": 8, "radius": 25, "action": 0}]}
        ap.route_target(2)
    finally:
        ap.link = old_link
    src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    mw = open(os.path.join(HERE, "mapwin.py"), encoding="utf-8").read()
    report("UV6 'Send route' with a UAV magazine: the vertical target box ΣΚΑΦΟΣ (the USV placeholder) / ΕΠΟΜΕΝΟ UAV "
           "(θέση n) / a UAV in the magazine / a FLYING UAV (new mission), each with its picture; no magazine = the "
           "boat only; the map's right click «Πήγαινε εδώ» (the boat, only with a position) + «Αποστολή UAV εδώ» "
           "(the next / a flying one); uav_route = target + every point with its altitude (default 50 m)",
           keys[:2] == ["boat", "next"] and 5 in keys and 2 in keys and len(none) == 1 and
           [m[0] for m in menu][:2] == ["go_here", "uav_here"] and any(m[1] == 5 for m in menu) and
           menu_nopos[0][0] == "uav_here" and msg == {"t": "uav_route", "target": "next", "wps": [
               {"lat": 37.8211111, "lon": 23.6, "alt": 50.0}, {"lat": 37.83, "lon": 23.61, "alt": 70.0}]} and
           texts[0] == "ΣΚΑΦΟΣ" and "ΕΠΟΜΕΝΟ UAV (θέση 1)" in texts[1] and len(pics) == len(btns) and
           chosen == ["next"] and sent and sent[-1]["t"] == "uav_route" and sent[-1]["target"] == 2 and
           "uavroute.TargetBox(" in src and "self.app.map_menu(" in mw and not rig.errors,
           "targets %s; menu %s; box %s (%d pictures); chosen %s; sent %s" % (
               keys, [(a, t) for a, t, _x, _k in menu], texts, len(pics), chosen, sent[-1:]))


def uv7(rig):
    """The UAV hatch relay = a LAMP on every relay panel (also the USB laptop's): never a button (U3)."""
    import json as _json
    import tempfile
    import boats
    i18n.set_lang("el", save=False)
    d = _json.load(open(os.path.join(os.path.dirname(HERE), "boats", "rf49d.json"), encoding="utf-8"))
    d["relays"].append({"n": 8, "name_el": "ΚΑΤΑΠΑΚΤΗ UAV", "name_en": "UAV HATCH", "type": "onoff",
                        "group": "extra", "pair": None, "max_s": 0, "show_hud": False, "power_up": "off"})
    tmp = tempfile.mkdtemp(prefix="uvtest_")
    try:
        def boat_with(uav, mod=None):
            dd = _json.loads(_json.dumps(d))
            dd["uav"] = uav
            if mod:
                mod(dd)
            p = os.path.join(tmp, "b.json")
            with open(p, "w", encoding="utf-8") as f:
                _json.dump(dd, f)
            return boats.load(tmp)[0]
        good = boat_with({"hatch_relay": 8})
        bad_engine = boat_with({"hatch_relay": 1})
        bad_missing = boat_with({"hatch_relay": 9})
        bad_hud = boat_with({"hatch_relay": 8}, lambda dd: dd["relays"][-1].update(show_hud=True))
        bad_pu = boat_with({"hatch_relay": 8}, lambda dd: dd["relays"][-1].update(power_up="on"))
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    import proto
    ap = rig.ap
    old = (ap.boat, ap.t)
    said = []
    old_say, old_cmd = ap.say, ap.cmd
    cmds = []
    try:
        ap.boat = good
        ap.t = dict(ap.t or {}, relay_table=True, relay_pending=False, relay_ok=True,
                    relay_hash=proto.relay_table_hash(good.relays), relay_on=[8], relay_unknown=[])
        active, rows = ap.relay_view()
        lamp = [r for r in rows if r["n"] == 8]
        ap.say = lambda text, colour=None: said.append(text)
        ap.cmd = lambda *a: cmds.append(a)
        ap.relay_press(8)
        ap.relay_press(6)
    finally:
        ap.boat, ap.t = old
        ap.say, ap.cmd = old_say, old_cmd
    src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    report("UV7 the boat file's \"uav\": {\"hatch_relay\": n} = an extra onoff relay, power_up off, no pair, not on the "
           "HUD (else the file is INVALID); the relay panels show it as a LAMP (never a button: a press only says why, "
           "nothing is sent - also on the USB laptop, which is not behind the gateway's refusal 125)",
           good.error is None and good.hatch_relay == 8 and bad_engine.error and bad_missing.error and
           bad_hud.error and bad_pu.error and active and lamp and lamp[0]["lamp"] and lamp[0]["state"] == "on" and
           said and "καταπακτή" in said[0].lower() and len(cmds) == 1 and cmds[0][1][0] == 6 and
           'if r.get("lamp"):' in src,
           "good %s / %s; bad: engine %s, missing %s, hud %s, power_up %s; lamp %s; said %s; sent %s" % (
               good.error, good.hatch_relay, bad_engine.error, bad_missing.error, bad_hud.error, bad_pu.error,
               lamp and lamp[0].get("state"), said, cmds))


# --- U4 ---------------------------------------------------------------------------------------------------------------

def drone(n, **kw):
    d = {"slot": n, "name": "UAV-%d" % n, "sysid": 10 + n, "lat": 37.8215, "lon": 23.6012, "alt": 52.0, "hdg": 90.0,
         "gs": 8.2, "climb": 0.0, "batt": 78, "batt_v": 15.8, "batt_a": 15.0, "mode": "AUTO", "armed": True,
         "landed": False, "in_slot": False, "flying": "mission", "lq": 100, "rssi": 200, "link_age": 0.3,
         "land_eta": 400.0, "landing": False, "stage": None, "gps_fix": 3, "text": None, "video": "RUN",
         "dist_m": 180}
    d.update(kw)
    return d


class _GL:
    """A gateway link stand-in (isinstance GatewayLink) that records what the station sends."""

    @staticmethod
    def make(sent):
        import gwclient

        class GL(gwclient.GatewayLink):
            def __init__(self):
                import queue
                self.host, self.connected, self.events = "127.0.0.1", True, queue.Queue()

            def send_msg(self, m):
                sent.append(m)

            def video_url(self, ack, cam):
                p = (ack or {}).get("streams", {}).get(cam)
                return "rtsp://u:p@127.0.0.1:8554/%s?station=1" % p if p else None
        return GL()


def uv8(rig):
    """The UAVs on screen 2's map: the owner's picture turned with the heading at the drone's position, its own track
    style (dashed, the map theme's "uav" colour, never the boat's track), the label with altitude / battery / link
    (CAUTION colour with a low battery / weak link), a drone in its slot not drawn; a click on a symbol opens its video."""
    import uavmap
    ap = rig.ap
    i18n.set_lang("el", save=False)
    rig.pump(1.0)
    ap.uav_store.clear()
    t0 = time.time()
    for k in range(6):
        ap.uav_store.update({"1": drone(1, lat=37.8215 + 0.0002 * k), "2": drone(2, batt=20, lq=40, lat=37.8190),
                             "3": drone(3, in_slot=True)}, now=t0 + 1.5 * k)
    ap.mapwin.redraw()
    c = ap.cv
    syms = c.find_withtag("uav_sym")
    imgs = [i for i in syms if c.type(i) == "image"]
    tracks = c.find_withtag("uav_track")
    labels = [c.itemcget(i, "text") for i in c.find_withtag("uav_label")]
    cols = [c.itemcget(i, "fill") for i in c.find_withtag("uav_label")]
    dashed = all(c.itemcget(i, "dash") for i in tracks)
    in_map = all("map" in c.gettags(i) for i in syms)
    sent = []
    old = ap.link
    hit = None
    try:
        ap.link = _GL.make(sent)
        P = ap.mapwin.P()[0]
        x, y = P(ap.uav_store.drones[1]["lat"], ap.uav_store.drones[1]["lon"])
        hit = ap.uav_map_hit(x, y)
    finally:
        ap.camview_close(send=False)
        ap.link = old
    ok_lab = len(labels) == 2 and any("UAV 1" in t and "52 m" in t and "78 %" in t for t in labels) and \
        cols.count(theme.CAUTION) == 1
    report("UV8 the UAVs on screen 2's map: the owner's picture (turned with the heading) for every flying drone, a "
           "drone in its slot not drawn, their own dashed track, label with altitude / battery / link (CAUTION colour "
           "for a low battery / weak link), under the panels (tag map); a click on the symbol opens its video",
           len(imgs) == 2 and len(tracks) == 1 and dashed and ok_lab and in_map and hit and
           {"t": "uav_view", "uav": 1, "open": True} in sent and not rig.errors,
           "symbols %d (images %d), tracks %d dashed %s, labels %s colours %s, in map %s, click %s sent %s" % (
               len(syms), len(imgs), len(tracks), dashed, labels, cols, in_map, hit, sent))
    ap.uav_store.clear()
    ap.mapwin.redraw()


def uv9(rig):
    """A UAV's video in screen 2's camera view: only that stream is asked for (uav_view open; the boat camera view
    closed), the flying UAVs in bar 1, the UAV's own green basics + the AI boxes on the picture, the stream from
    cams_ack only while open, closing sends uav_view close; data saver = the notice banner."""
    ap = rig.ap
    i18n.set_lang("el", save=False)
    sent = []
    old = ap.link
    ap.uav_store.clear()
    ap.uav_store.update({"2": drone(2, stage=20, batt=24)})
    try:
        ap.link = _GL.make(sent)
        ap.gw_status = dict(ap.gw_status or {}, uav={"enabled": True, "state": "ACTIVE", "drones": {},
                                                    "ai": {"feeds": {"2": {"boxes": [[0.1, 0.2, 0.3, 0.4, 0.8]]}}}})
        ap.uav_view_open(2)
        opened = list(sent)
        bar = [it[1] for it in ap.bar_items(1) if it[0] == "btn"]
        ap.on_cams_ack({"t": "cams_ack", "ok": True, "streams": {"UAV-2": "uav_2"}, "levels": {}, "notice": "",
                        "uav_open": [2], "uav_reduced": False, "user": "u", "password": "p", "rtsp_port": 8554,
                        "query": "station=1"})
        has_stream = "UAV-2" in ap.vstreams
        ap.render(force=True)
        texts = [o.get("s", "") for o in ap.ov.ops]
        hud = [t for t in texts if "UAV 2" in t or "ύψος" in t or "μπαταρία" in t]
        ai_line = any(t.startswith("AI") for t in texts)
        ap.cam_ack = {"uav_reduced": True, "notice": "DATA SAVER: no UAV video"}
        ban = [b for b in ap.banner_list() if b[0] == "camview_reduced"]
        sent.clear()
        ap.camview_close()
        closed = list(sent)
    finally:
        for n in list(ap.vstreams):
            if n.startswith("UAV-"):
                ap.vstreams.pop(n).stop()
        ap.link = old
        ap.gw_status = None
        ap.uav_store.clear()
    report("UV9 a UAV's video on screen 2: uav_view open (+ the boat camera view closed), the flying UAVs in bar 1, the "
           "stream started from cams_ack, the UAV's own HUD (altitude, battery, link, battery minimum) + the AI line, "
           "data saver = the notice banner, closing = uav_view close (no uplink for it any more)",
           opened == [{"t": "cam_view", "cam": None}, {"t": "uav_view", "uav": 2, "open": True}] and
           "cv_UAV-2" in bar and has_stream and len(hud) >= 3 and ai_line and ban and
           "χωρίς βίντεο UAV" in ban[0][1] and closed == [{"t": "uav_view", "uav": 2, "open": False}] and not rig.errors,
           "opened %s; bar %s; stream %s; hud %s; ai %s; banner %s; closed %s" % (opened, bar[-3:], has_stream, hud,
                                                                                 ai_line, ban[:1], closed))


def uv10(rig):
    """The battery minimum's CAUTION on THIS station (60 s / 20 s before + the landing, MasterLamps.set_local, never
    a boat alarm) raised once per stage, gone when the drone is down; the UAV router OFFLINE on both screens'
    notices (U2 review LOW-12)."""
    import uavmap
    ap = rig.ap
    i18n.set_lang("el", save=False)
    st = uavmap.Store()
    rows = []
    for stg in (None, 60, 60, 20, "land"):
        st.update({"1": drone(1, stage=stg)})
        rows.append(st.local_cautions())
    st.update({"1": drone(1, stage=None, armed=False, landed=True)})
    rows.append(st.local_cautions())
    seq = [[(c, t is not None) for c, t in r] for r in rows]
    texts = [t for r in rows for _c, t in r if t]
    old_link, old_gs = ap.link, ap.gw_status
    sent = []
    try:
        ap.link = _GL.make(sent)
        ap.gw_status = {"uav": {"enabled": True, "state": "OFFLINE", "why": "the router process ended - restarting"}}
        ban = [b[1] for b in ap.banner_list() if b[0] == "uav_offline"]
        notes = [n[1] for n in ap.driver.notices(None, ap.gw_status)] if getattr(ap, "driver", None) else None
        ap.uav_notices({"enabled": True, "state": "ACTIVE", "drones": {"1": drone(1, stage=60)}, "notices": []})
        lamp = [t for t, _lv, _s in ap.lamps.rows() if "UAV 1" in t]
        asked = {"t": "uav_telem_on", "on": True} in sent
    finally:
        ap.link, ap.gw_status = old_link, old_gs
        ap.lamps.set_local("uav_land_1", None)
        ap.uav_store.clear()
        ap.uav_telem_asked = False
    eng = [w for t in texts + ban for w in ts.english_words(t)]
    report("UV10 the battery minimum's CAUTION on this station: once at 60 s, once at 20 s, at the landing, cleared "
           "when the drone is down (station-local, never a boat alarm); the UAV router OFFLINE on screen 2's banners "
           "and screen 1's notices; uav_telem asked once; Greek texts",
           seq == [[], [("uav_land_1", True)], [], [("uav_land_1", True)], [("uav_land_1", True)],
                   [("uav_land_1", False)]] and ban and "ΕΚΤΟΣ ΛΕΙΤΟΥΡΓΙΑΣ" in ban[0] and
           (notes is None or any("ΕΚΤΟΣ ΛΕΙΤΟΥΡΓΙΑΣ" in n for n in notes)) and lamp and asked and not eng,
           "stages %s; texts %s; banner %s; screen 1 %s; lamp %s; asked %s; English %s" % (
               seq, texts, ban, notes, lamp, asked, eng))


def uv11(rig):
    """The flight logs on the «ΜΑΓΓΕΜΑ UAV» page: FLIGHT LOGS asks the gateway (uav_logs), the list shown, EXPORT
    CSV / GPX asks for one (uav_log) and the answer is written to the chosen file; a refusal shown in Greek."""
    import tempfile
    import tkinter as tk
    ap = rig.ap
    i18n.set_lang("el", save=False)
    sent = []
    old = ap.link
    tmp = tempfile.mkdtemp(prefix="uvlog_")
    top = tk.Toplevel(rig.root)
    try:
        ap.link = _GL.make(sent)
        pg = uavpanel.Page(tk.Frame(top), ap)
        pg.connected = lambda: True
        pg.update(status())
        pg.ask_logs()
        pg.on_logs({"t": "uav_logs", "logs": [{"name": "UAV-1", "file": "20261104-160924.csv", "bytes": 4096,
                                               "t": 2.0}, {"name": "UAV-2", "file": "20261104-150000.csv",
                                                           "bytes": 100, "t": 1.0}]})
        items = list(pg.lb_logs.get(0, "end"))
        pg.lb_logs.selection_set(0)
        path = os.path.join(tmp, "f.gpx")
        pg.export("gpx", path=path)
        ap.uav_log_reply({"t": "uav_log", "ok": True, "name": "UAV-1", "file": "20261104-160924.csv", "fmt": "gpx",
                          "data": "<gpx>ok</gpx>"})
        wrote = open(path, encoding="utf-8").read() if os.path.exists(path) else None
        said = []
        old_say = ap.say
        ap.say = lambda text, colour=None: said.append(text)
        ap.uav_log_reply({"t": "uav_log", "ok": False, "text": "no such flight log"})
        ap.say = old_say
    finally:
        ap.link = old
        top.destroy()
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)
    report("UV11 the flight logs on the UAV page: FLIGHT LOGS = uav_logs, the list (newest first), EXPORT GPX = uav_log "
           "{name, file, fmt} and the answer written to the chosen file, a refusal in Greek",
           {"t": "uav_logs"} in sent and len(items) == 2 and items[0].startswith("UAV-1") and
           {"t": "uav_log", "name": "UAV-1", "file": "20261104-160924.csv", "fmt": "gpx"} in sent and
           wrote == "<gpx>ok</gpx>" and said and "δεν υπάρχει τέτοιο αρχείο πτήσης" in said[0],
           "sent %s; list %s; wrote %r; said %s" % (sent, items, wrote, said))


class _Js:
    """A fake shorejoy.Joysticks: dev 0 = the stick (X, Y, R twist, buttons), dev 1 = the TWCS lever (Z)."""

    def __init__(self, x=0.0, y=0.0, r=0.0, lever_axis=0.0, buttons=0, ok=True):
        self.v = {"X": x, "Y": y, "R": r, "Z": lever_axis}
        self.buttons, self.ok = buttons, ok

    def read(self, dev):
        return {"ok": self.ok, "axes": dict(self.v), "buttons": self.buttons, "pov": None}


def uv12():
    """uavfly.py (pure): the stick -> {x, y, z, r} (the stick Y pushed forward = +, the lever's neutral zone 45-55 %
    = hold the altitude, a station without a lever = altitude held, the twist = yaw), a NaN / a missing device = no
    packet; take / give / the gateway's answers; the captain's control ends when the boat is not in STANDBY."""
    import proto
    import shorejoy
    import uavfly
    cfg = dict(shorejoy.DEFAULT)
    lc = [uavfly.lever_climb(v) for v in (0.0, 0.45, 0.5, 0.55, 1.0, float("nan"))]
    # the TWCS axis Z inverted: -1 = forward = lever 1.0
    full = uavfly.stick(_Js(x=0.5, y=-1.0, r=0.3, lever_axis=-1.0), cfg)
    mid = uavfly.stick(_Js(lever_axis=0.0), cfg)
    nan = uavfly.stick(_Js(x=float("nan")), cfg)
    gone = uavfly.stick(_Js(ok=False), cfg)
    nolever = uavfly.stick(_Js(y=0.4), dict(cfg, thr=dict(cfg["thr"], lever=False, axis="Y", dev=0, invert=True)))
    f = uavfly.UavFly()
    idle_src = f.source(_Js(), cfg)
    m_take = f.take(2)
    act_asked = f.active()
    f.on_ack({"what": "pilot", "ok": True, "take": True, "uav": 2, "captain": True})
    src = f.source(_Js(y=-1.0), cfg)
    end_auto = f.must_end(proto.MODE_AUTO, True)
    end_cop = f.must_end(proto.MODE_AUTO, False)
    end_sb = f.must_end(proto.MODE_STANDBY, True)
    m_give = f.give()
    f.take(3)
    f.on_ack({"what": "pilot", "ok": False, "code": 127, "uav": 3})
    refused = f.active()
    f.take(1)
    f.on_ack({"what": "pilot", "ok": True, "take": True, "uav": 1})
    f.on_end({"t": "uav_pilot_end", "uav": 1, "why": "captain"})
    ok = (lc == [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0] and full == {"x": 0.5, "y": 1.0, "z": 1.0, "r": 0.3} and
          mid["z"] == 0.0 and nan is None and gone is None and nolever["z"] == 0.0 and abs(nolever["y"] + 0.4) < 1e-9
          and idle_src is None and m_take == {"t": "uav_pilot", "uav": 2, "take": True} and act_asked and
          src["t"] == "uav_ctl" and src["uav"] == 2 and src["y"] == 1.0 and end_auto and not end_cop and
          not end_sb and m_give == {"t": "uav_pilot", "uav": 2, "take": False} and not refused and not f.active() and
          f.last_end[1] == "captain")
    report("UV12 uavfly.py: the stick -> x / y / z (lever neutral zone = hold the altitude) / r, NaN / missing device "
           "= no packet, no lever = altitude held; take / give / refusal / the captain's takeover end; the captain's "
           "control ends outside STANDBY (a copilot's not)",
           ok, "lever %s full %s mid %s nan %s gone %s nolever %s src %s ends %s/%s/%s give %s refused %s end %s" % (
               lc, full, mid, nan, gone, nolever, src, end_auto, end_cop, end_sb, m_give, refused, f.last_end))


def uv13(rig):
    """The real station flying a UAV: «ΧΕΙΡΙΣΜΟΣ UAV n» sends uav_pilot (the captain refused locally outside
    STANDBY), the gateway's yes = the 10 Hz uav_ctl sender + its video + the banner «JOYSTICK → UAV n» on BOTH screens;
    meanwhile NO rjoy packet to the boat and the YES / NO selector gets no stick (mouse only); the camera view's bar
    1 = RETURN TO BOAT; the boat leaving STANDBY gives it back at once; the captain's takeover = «Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ
    ΤΟΝ ΕΛΕΓΧΟ» + a local CAUTION; the UAV page's fly buttons (fly_view)."""
    import proto
    ap = rig.ap
    i18n.set_lang("el", save=False)
    sent = []
    old_link, old_t, old_gs, old_sj, old_feed = ap.link, ap.t, ap.gw_status, getattr(ap, "sj", None), ap.yn.feed
    fed = []
    try:
        ap.link = _GL.make(sent)
        ap.link.send_rjoy = lambda m: sent.append(m)
        ap.link.clock_offset = 0.0
        ap.gw_status = {"you": 5, "stations": [{"id": 5, "crew": "captain", "licence": True}],
                        "uav": {"enabled": True, "state": "ACTIVE", "drones": {"1": drone(1)}, "pilots": {}}}
        ap.uav_store.update({"1": drone(1)})
        ap.t = dict(ap.t or {}, mode=proto.MODE_AUTO, mode_name="AUTO", helm=5)
        said = []
        old_say = ap.say
        ap.say = lambda text, colour=None: said.append(text)
        ap.uav_take(1)
        refused_local = not sent and said and "STANDBY" in said[-1]
        ap.t["mode"], ap.t["mode_name"] = proto.MODE_STANDBY, "STANDBY"
        ap.uav_take(1)
        asked = sent[-1] if sent else None
        ap.sj = _Js(buttons=1)                   # the trigger held
        rj_asked = ap.rjoy_source()
        ap.uav_pilot_ack({"t": "uav_ack", "what": "pilot", "ok": True, "take": True, "uav": 1, "captain": True})
        flying = ap.uav_fly.slot
        sender = ap.uav_sender is not None
        ap.yn.feed = lambda c, now, mode: fed.append(c)
        ap.shore_answer_tick()
        rj = ap.rjoy_source()
        ban2 = [b[1] for b in ap.banner_list() if b[0] == "uav_fly"]
        ban1 = [n[1] for n in ap.driver.notices(ap.t, ap.gw_status) if n[0] == "uav_fly"] if ap.driver else None
        ap.camview_open("UAV-1")
        bar = [it[1] for it in ap.bar_items(1) if it[0] == "btn"]
        view = [m for m in sent if m.get("t") == "uav_view"]
        sent.clear()
        ap.t["mode"] = proto.MODE_HDG_LOCK          # the boat leaves STANDBY: the joystick back at once
        ap.shore_answer_tick()
        gave = [m for m in sent if m.get("t") == "uav_pilot"]
        back = ap.uav_fly.slot is None and ap.uav_sender is None
        # a copilot's station: the captain took over
        ap.uav_pilot_ack({"t": "uav_ack", "what": "pilot", "ok": True, "take": True, "uav": 1})
        ap.uav_pilot_end({"t": "uav_pilot_end", "uav": 1, "why": "captain", "text": "the captain took control of UAV 1"})
        took = said[-1]
        lamp = [t for t, _lv, _s in ap.lamps.rows() if "ΚΥΒΕΡΝΗΤΗΣ" in t]
        ap.say = old_say
        fv = uavpanel.fly_view({"drones": {"1": drone(1), "2": drone(2, in_slot=True, armed=False, landed=True)},
                                "pilots": {"1": {"name": "anna", "captain": False}}}, None)
    finally:
        ap.camview_close(send=False)
        ap.uav_fly_stop()
        for n in list(ap.vstreams):
            if n.startswith("UAV-"):
                ap.vstreams.pop(n).stop()
        ap.link, ap.t, ap.gw_status, ap.sj, ap.yn.feed = old_link, old_t, old_gs, old_sj, old_feed
        ap.lamps.set_local("uav_pilot", None)
        ap.uav_store.clear()
    ok = (refused_local and asked == {"t": "uav_pilot", "uav": 1, "take": True} and rj_asked is None and
          flying == 1 and sender and fed[:1] == [None] and rj is None and ban2 and "JOYSTICK → UAV 1" in ban2[0] and
          (ban1 is None or ban1) and "uav_give" in bar and gave == [{"t": "uav_pilot", "uav": 1, "take": False}] and
          back and "Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟΝ ΕΛΕΓΧΟ" in took and lamp and fv["can_take"] == [1] and
          "anna" in fv["text"] and not rig.errors)
    report("UV13 the station flying a UAV: the captain refused here outside STANDBY, uav_pilot sent, then the 10 Hz "
           "sender, NO rjoy to the boat, the YES / NO selector gets no stick (mouse only), «JOYSTICK → UAV 1» on both "
           "screens, RETURN TO BOAT in the camera view's bar, the boat leaving STANDBY gives it back at once, the "
           "captain's takeover = «Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟΝ ΕΛΕΓΧΟ» + a local CAUTION, the page's fly buttons",
           ok, "local refusal %s asked %s rjoy %s/%s flying %s sender %s fed %s banners %s / %s bar %s views %s gave %s "
               "back %s took %r lamp %s fly view %s errors %s" % (
                   refused_local, asked, rj_asked, rj, flying, sender, fed, ban2, ban1, bar[-3:], view, gave, back,
                   took, lamp, fv, rig.errors[:2]))


def uv14():
    """uavhud.py on screen 1: the UAV's own green HUD (banner, altitude, speed, battery, link, heading, the arrow +
    distance to the boat, the battery stage, the gateway's clip reason, RETURN TO BOAT = hit uav_give); of the boat's
    HUD only the mode, MASTER lamps, alarms, notices and the bottom bar stay; the arrow maths."""
    import hud
    import uavhud
    i18n.set_lang("el", save=False)
    h = hud.Hud(hud.Layout(os.path.join(os.environ.get("TEMP", "."), "uv14_layout.json")))
    s = hud.Snapshot(t={"mode": 0, "mode_name": "STANDBY", "target": 0.0, "heading": 10.0, "compass_ok": True,
                        "est_sog": 0.0, "throttle": 0.0, "gear": 1, "alarm_bits": 0, "power_fwd": 30,
                        "power_rev": 5}, gw={}, lamps=None, declutter="FULL", hud_colour="green")
    s["uav_fly"] = 1
    s["uav"] = drone(1, hdg=90.0, lat=37.8215, lon=23.6012, stage=60)
    s["uav_boat"] = (37.8215, 23.5990)            # west of the drone heading east: the boat is astern (arrow down)
    s["uav_clip"] = ["minimum altitude 10 m"]
    try:
        ops, hits = h.build(1600, 900, s)
    except Exception as e:                         # (a Snapshot without lamps may not draw every boat box)
        ops, hits = [], []
        report("UV14 uavhud", False, "build crashed: %r" % e)
        return
    tags = {o.get("tag") for o in ops}
    texts = [o.get("s", "") for o in ops if o.get("k") == "text"]
    green = hud.palette("green", False)["main"]
    uav_txt = [o for o in ops if str(o.get("tag", "")).startswith("uav_") and o.get("k") == "text"]
    rel = uavhud.to_boat(s["uav"], s["uav_boat"])[1]
    arrows = [uavhud.arrow_char(a) for a in (0, 45, 90, 180, 270, 359)]
    ok = ("JOYSTICK → UAV 1" in texts and any("52 m" in t for t in texts) and any("8.2 m/s" in t for t in texts) and
          any("μπαταρία 78" in t for t in texts) and any("σκάφος" in t and " m" in t for t in texts) and
          any("ελάχιστο ύψος" in t for t in texts) and any("ΠΡΟΣΓΕΙΩΣΗ" in t or "60" in t for t in texts) and
          ("uav_give" in [a for _r, a in hits]) and "throttle" not in tags and "tape" not in tags and
          "speed" not in tags and all(o["fill"] == green for o in uav_txt if o["tag"] not in ("uav_batt", "uav_stage"))
          and 170 < rel < 190 and arrows == ["↑", "↗", "→", "↓", "←", "↑"])
    report("UV14 screen 1's UAV HUD (uavhud.py): banner «JOYSTICK → UAV 1», altitude / speed / battery / link / "
           "heading in green, the arrow + distance to the boat (relative to the UAV's heading), the battery stage, "
           "the clip reason in Greek, RETURN TO BOAT = hit uav_give; the boat's instruments hidden (throttle, tape, "
           "speed), the mode / lamps / alarms / bottom bar kept",
           ok, "tags %s texts %s rel %.0f arrows %s hits %s" % (sorted(t for t in tags if t)[:30], texts[:14], rel,
                                                               arrows, [a for _r, a in hits][:10]))


def main():
    uv1()
    uv2()
    for fn in (uv12, uv14):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__, False, "crashed: %r" % e)
    rig = ts.Rig()
    try:
        for fn in (uv5, uv6, uv7, uv8, uv9, uv10, uv11, uv13):
            try:
                fn(rig)
            except Exception as e:
                import traceback
                traceback.print_exc()
                report(fn.__name__, False, "crashed: %r" % e)
        try:
            uv3_4(rig)
        except Exception as e:
            import traceback
            traceback.print_exc()
            report("uv3_4", False, "crashed: %r" % e)
    finally:
        rig.close()
    bad = results.count(False)
    print("UAV STATION TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    t0 = time.time()
    rc = main()
    sys.exit(rc)
