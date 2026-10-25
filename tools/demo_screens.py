"""Screenshots of the ground station for review (section G1) -> docs/screens/.

    python tools/demo_screens.py

Restarts the demo (demo/demo.py: simulator + gateway + cameras + AIS, without
its ground-station windows), runs the real ground station in this process
(through the gateway, like the shore station), drives the demo AUTO mission,
sends the collision ferry (DEMO_SHIP), switches the GPS off for an alarm and
captures the driver screen (day / night, Greek / English, AIS threat, alarm,
AR, MINIMAL), the map window and the captain screen. At the end the demo is
started again as usual (START_DEMO). Needs a visible desktop (screen grab).
Uses its own settings / layout files (never this PC's).
"""
import json
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
os.chdir(os.path.join(REPO, "groundstation"))      # the ground station writes its logs/ there
OUT = os.path.join(REPO, "docs", "screens")
TMP = tempfile.mkdtemp(prefix="trygons_screens_")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(TMP, "gs_settings.json")
os.environ["TRYGONS_HUD_LAYOUT"] = os.path.join(TMP, "hud_layout.json")
os.environ["TRYGONS_SOUND"] = "off"
os.environ.pop("TRYGONS_LANG", None)
sys.path.insert(0, os.path.join(REPO, "demo"))
sys.path.insert(0, os.path.join(REPO, "groundstation"))
sys.path.insert(0, os.path.join(REPO, "gateway"))

if os.name == "nt":                      # real pixels (crisp pictures on a scaled display)
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

import demo  # noqa: E402
import tkinter as tk  # noqa: E402
from PIL import ImageGrab  # noqa: E402

shots = []


def say(t):
    print(t, flush=True)


def main():
    import faulthandler
    faulthandler.dump_traceback_later(120, repeat=True)          # a hang shows where it is
    os.makedirs(OUT, exist_ok=True)
    say("restarting the demo with the current code ...")
    demo.stop()
    time.sleep(1.5)
    if demo.start() != 0:
        say("the demo did not start")
        return 1
    with open(demo.PIDS, encoding="utf-8") as f:
        pids = json.load(f)
    for name in ("shore", "onboard"):                 # this script is the ground station
        if name in pids:
            demo.kill_tree(pids[name])
    charts = demo.demo_charts()
    import app as A
    import i18n
    import settings
    import program
    import proto
    from link import Link
    say("starting the ground station ...")
    root = tk.Tk()
    root.tk.call("tk", "scaling", 96 / 72.0)          # the normal 100 % sizes, in real pixels
    errors = []
    root.report_callback_exception = lambda *a: errors.append(repr(a[1]))
    ap = A.App(root, "gw:127.0.0.1:5761", login=(demo.USER, demo.PASSWORD), charts_dir=charts,
               layout=os.path.join(TMP, "layout.json"), cams="Full")

    def pump(s, until=None):
        end = time.time() + s
        while time.time() < end:
            root.update()
            if until and until():
                return True
            time.sleep(0.02)
        return bool(until and until())

    def grab(win, name, what):
        win.lift()
        win.focus_force()
        pump(0.8)
        x, y = win.winfo_rootx(), win.winfo_rooty()
        w, h = win.winfo_width(), win.winfo_height()
        im = ImageGrab.grab(bbox=(x, y, x + w, y + h), all_screens=True)
        path = os.path.join(OUT, name + ".png")
        im.save(path)
        shots.append((name, what))
        say("  saved %s (%dx%d) - %s" % (name, w, h, what))

    say("connecting through the gateway ...")
    root.state("normal")
    root.geometry("1600x1000+0+940")
    if not pump(25, lambda: ap.t is not None and ap.gw_status is not None):
        say("no data through the gateway")
        return 1
    ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
    pump(1.0)
    ap.open_driver()
    dv = ap.driver
    dv.w.geometry("1600x900+0+0")
    ap.open_map()                                  # screen 2 = the map with the corner panels
    pump(2.0)
    # the onboard side: the simulator's USB joystick (centred, lever in neutral) on the USB link
    sim = Link("tcp:127.0.0.1:5760")
    sim.start()
    jm = proto.default_joymap(1)

    def bridge(lever=0.5, codes=()):
        b, hats = [0, 0], [15, 15]
        for c in codes:
            if 1 <= c <= 64:
                b[(c - 1) // 32] |= 1 << ((c - 1) % 32)
        sim.send(proto.T_SIM_BRIDGE, proto.p_sim_bridge(0.0, 0.0, lever, b[0], b[1], hats[0], hats[1], True, True,
                                                        False, 0))
    bridge()
    pump(1.0)
    # the demo AUTO mission: upload, AUTO start, the driver's ACCEPT, CONFIRM on board
    ap.prog = program.load(os.path.join(REPO, "demo", "demo_mission.json"))
    ap.prog_refresh()
    ap.prog_upload()
    pump(4.0)
    for _try in range(3):
        ap.auto_start()
        pump(5.0, lambda: dv.question is not None)
        if dv.question:
            dv.answer(True)
        pump(1.5)
        bridge(codes=[jm["btn"][proto.JM_CONFIRM]])
        pump(0.3)
        bridge()
        if pump(6.0, lambda: ap.t and ap.t["mode"] == proto.MODE_AUTO):
            break
    say("mode %s; driving the mission ..." % ap.t["mode_name"])
    pump(45.0, lambda: ap.t["est_sog"] > 5.5)
    pump(20.0)
    say("the collision ferry (DEMO_SHIP) ...")
    demo.ship()
    pump(30.0, lambda: any(t.get("level") == "danger" for t in ((ap.gw_status or {}).get("avoid") or {}).get(
        "threats") or []))
    pump(3.0)

    # --- the driver screen ---------------------------------------------------------------------
    for lang in ("el", "en"):
        i18n.set_lang(lang)
        pump(1.5)
        grab(dv.w, "driver_day_%s" % lang, "driver HUD, day, %s: AUTO mission, AR overlays (horizon, route ribbon, "
                                           "waypoint, predicted path, velocity vector), AIS threat DEMO FERRY"
             % ("Greek" if lang == "el" else "English"))
    i18n.set_lang("el")
    sim.send(proto.T_SIM_GPS, proto.p_sim_gps(True))                 # GPS lost: a CAUTION alarm
    pump(8.0, lambda: ap.t["alarm_bits"] & 0x002)
    pump(2.0)
    grab(dv.w, "driver_alarm_el", "driver HUD with active alarms (GPS LOST, OBSTACLE AHEAD): MASTER CAUTION lit, alarm bar")
    i18n.set_lang("en")
    pump(1.0)
    grab(dv.w, "driver_alarm_en", "the same in English")
    sim.send(proto.T_SIM_GPS, proto.p_sim_gps(False))
    for lang in ("el", "en"):
        i18n.set_lang(lang)
        settings.put("night", True)
        pump(1.5)
        grab(dv.w, "driver_night_%s" % lang, "driver HUD, NIGHT mode (dim red), %s" % lang)
    settings.put("night", False)
    i18n.set_lang("el")
    settings.put("declutter", "MINIMAL")
    pump(1.5)
    grab(dv.w, "driver_minimal_el", "driver HUD, declutter MINIMAL (speed, heading, mode, gear, alarms)")
    settings.put("declutter", "FULL")
    dv.action("edit")
    pump(1.0)
    grab(dv.w, "driver_edit_layout_el", "driver HUD, edit-layout mode (drag / resize / hide every box)")
    dv.action("edit")
    # --- screen 2 (navigation & control): the map + the corner panels, both themes ------------------------
    import theme
    for th, lang in (("dark", "el"), ("light", "en"), ("dark", "en")):
        i18n.set_lang(lang)
        theme.set_theme(th)
        ap.mapwin.redraw()
        ap.render(force=True)
        pump(1.5)
        grab(root, "screen2_%s_%s" % (th, lang), "screen 2 (navigation & control), %s theme, %s: the map with the "
             "boat in the centre, the four corner panels, STANDBY, the logo" % (th, lang))
    theme.set_theme("dark")
    # --- the tools window ---------------------------------------------------------------------------------
    i18n.set_lang("el")
    for key in ("tab.connection", "tab.sound"):
        ap.open_tab(key)
        pump(1.5)
        grab(ap.tools, "tools_%s_el" % key.split(".")[1], "tools window, %s page, Greek" % key.split(".")[1])
    ap.tools.withdraw()
    sim.stop()
    ap.quit()
    with open(os.path.join(OUT, "README.md"), "w", encoding="utf-8") as f:
        f.write("# Ground station screenshots (review)\n\nMade from the demo by `python tools/demo_screens.py` "
                "on %s.\n\n| File | What it shows |\n|---|---|\n" % time.strftime("%Y-%m-%d"))
        for name, what in shots:
            f.write("| [%s.png](%s.png) | %s |\n" % (name, name, what))
    say("errors in the ground station: %d %s" % (len(errors), errors[:3]))
    say("restarting the normal demo ...")
    demo.stop()
    time.sleep(1.5)
    demo.start()
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
