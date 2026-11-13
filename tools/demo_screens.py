"""Screenshots of the ground station from the running demo.

    python tools/demo_screens.py                 the tour pictures + the presentation pack (both below)
    python tools/demo_screens.py --tour          only the guided tour's REAL pictures (step 13t a)
    python tools/demo_screens.py --tour --if-stale   the same, only when academy/tourshots.py says they are stale
                                                     (tools/check.sh: a layout change makes them again)
    python tools/demo_screens.py --presentation  only the presentation pack docs/presentation/ (step 13t b)
    python tools/demo_screens.py --review        the older review set docs/screens/ (section G1)

Restarts the demo (demo/demo.py: simulator + gateway + cameras + AIS + the UAV magazine, without its
ground-station windows), runs the real ground station in this process (through the gateway, like the shore
station, with the demo boat file: relay 8 = the UAV hatch), takes HELM, lets the simulated AURIX take the
boat file's relay table (the onboard CONFIRM of the simulated USB joystick), registers UAV 1 and launches it
(the captain's YES on screen 1), drives the demo AUTO mission and sends the collision ferry (DEMO_SHIP).

--tour: screen 1 (the HUD over the camera video) + screen 1 with a YES / NO question, screen 2 (the map with
the route, the AIS traffic and the UAV, the mode bar, the panels, the bars) and every page of the tools window,
in Greek and English -> academy/tour_shots/*.jpg + shots.json = the picture of every tour stop and the REAL
place of every item it names (the HUD's own boxes + hit boxes, the overlay's panels / buttons, the widgets of
the tools window; academy/tourshots.py). For the tour picture the HUD shows every button of its bottom bar
(CLEAR SLOWDOWN / RESTORE POWER / the drive buttons appear on the boat only in their situations: here they are
forced ON in the picture so the tour can show them; screen 2 shows the COLLISION RISK banner of the nearest AIS
target when no banner is up at that moment - display only, nothing is sent).

--presentation: high-resolution screenshots (el + en) of a scripted scenario (AUTO route, ferry collision risk
+ the HDG LOCK proposal, joystick override in AUTO, drone launch + UAV video, loudhailer, mode bar, BATTLE
SHORT) + short MP4 clips (ffmpeg gdigrab = the same capture as REC SCREEN, H.264 -preset veryfast) + README.md.

At the end the demo is started again as usual (START_DEMO) when it was running before. Windows only: the pictures are PrintWindow pictures of the station's OWN windows (never a screen grab -
another window in front, e.g. a browser, is never captured).
Uses its own settings / layout files (never this PC's).
"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
os.chdir(os.path.join(REPO, "groundstation"))      # the ground station writes its logs/ there
OUT = os.path.join(REPO, "docs", "screens")
TOUR_OUT = os.path.join(REPO, "academy", "tour_shots")
PRES_OUT = os.path.join(REPO, "docs", "presentation")
TMP = tempfile.mkdtemp(prefix="trygons_screens_")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(TMP, "gs_settings.json")
os.environ.setdefault("TRYGONS_TEST", "1")      # temp track folder (review 13n3 L7)
os.environ["TRYGONS_HUD_LAYOUT"] = os.path.join(TMP, "hud_layout.json")
os.environ["TRYGONS_SOUND"] = "off"
os.environ["TRYGONS_SCREENREC"] = os.path.join(TMP, "screenrec")
os.environ.pop("TRYGONS_LANG", None)
sys.path.insert(0, os.path.join(REPO, "demo"))
sys.path.insert(0, os.path.join(REPO, "groundstation"))
sys.path.insert(0, os.path.join(REPO, "gateway"))
sys.path.insert(0, os.path.join(REPO, "academy"))

if os.name == "nt":                      # real pixels (crisp pictures on a scaled display)
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

shots = []


def say(t):
    print(t, flush=True)


def tour_stale():
    """True when the tour pictures are missing or do not match the code's layout (academy/tourshots.py)."""
    import tour
    import tourshots
    man = tourshots.load()
    return man is None or man.get("signature") != tourshots.signature(tour) or bool(tourshots.check(tour, man))


# --- pictures of OUR windows only: Windows PrintWindow (the window's own content, also when another window covers it
# - a screen grab would take whatever is in front, e.g. the user's browser) ------------------------------------------
def window_image(widget):
    """The widget's area of its top-level window as a PIL RGB image (PrintWindow PW_RENDERFULLCONTENT)."""
    import ctypes
    from ctypes import wintypes
    from PIL import Image
    if os.name != "nt":
        raise RuntimeError("window pictures need Windows (PrintWindow)")
    u32, g32 = ctypes.windll.user32, ctypes.windll.gdi32
    H_ = ctypes.c_void_p
    u32.GetWindowRect.argtypes = [H_, ctypes.POINTER(wintypes.RECT)]
    u32.GetWindowDC.argtypes = [H_]
    u32.GetWindowDC.restype = H_
    u32.ReleaseDC.argtypes = [H_, H_]
    u32.PrintWindow.argtypes = [H_, H_, ctypes.c_uint]
    g32.CreateCompatibleDC.argtypes = [H_]
    g32.CreateCompatibleDC.restype = H_
    g32.CreateCompatibleBitmap.argtypes = [H_, ctypes.c_int, ctypes.c_int]
    g32.CreateCompatibleBitmap.restype = H_
    g32.SelectObject.argtypes = [H_, H_]
    g32.SelectObject.restype = H_
    g32.GetDIBits.argtypes = [H_, H_, ctypes.c_uint, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]
    g32.DeleteObject.argtypes = [H_]
    g32.DeleteDC.argtypes = [H_]
    top = widget.winfo_toplevel()
    hwnd = int(top.wm_frame(), 16)
    r = wintypes.RECT()
    u32.GetWindowRect(hwnd, ctypes.byref(r))
    W, H = r.right - r.left, r.bottom - r.top
    hdc = u32.GetWindowDC(hwnd)
    mdc = g32.CreateCompatibleDC(hdc)
    bmp = g32.CreateCompatibleBitmap(hdc, W, H)
    old = g32.SelectObject(mdc, bmp)
    try:
        u32.PrintWindow(hwnd, mdc, 2)

        class BIH(ctypes.Structure):
            _fields_ = [("biSize", ctypes.c_uint32), ("biWidth", ctypes.c_int32), ("biHeight", ctypes.c_int32),
                        ("biPlanes", ctypes.c_uint16), ("biBitCount", ctypes.c_uint16),
                        ("biCompression", ctypes.c_uint32), ("biSizeImage", ctypes.c_uint32),
                        ("biXPelsPerMeter", ctypes.c_int32), ("biYPelsPerMeter", ctypes.c_int32),
                        ("biClrUsed", ctypes.c_uint32), ("biClrImportant", ctypes.c_uint32)]
        bi = BIH(ctypes.sizeof(BIH), W, -H, 1, 32, 0, 0, 0, 0, 0, 0)
        buf = ctypes.create_string_buffer(W * H * 4)
        g32.GetDIBits(mdc, bmp, 0, H, buf, ctypes.byref(bi), 0)
    finally:
        g32.SelectObject(mdc, old)
        g32.DeleteObject(bmp)
        g32.DeleteDC(mdc)
        u32.ReleaseDC(hwnd, hdc)
    im = Image.frombuffer("RGB", (W, H), buf.raw, "raw", "BGRX", 0, 1)
    x, y = widget.winfo_rootx() - r.left, widget.winfo_rooty() - r.top
    return im.crop((x, y, x + widget.winfo_width(), y + widget.winfo_height()))


# --- the demo session: the real ground station in this process ---------------------------------------------------
class Session:
    def __init__(self):
        import demo
        self.demo = demo
        self.was_running = os.path.exists(demo.PIDS)     # start it again at the end only if it was running
        say("restarting the demo with the current code ...")
        demo.stop()
        time.sleep(1.5)
        if demo.start() != 0:
            raise RuntimeError("the demo did not start")
        try:
            self._open()
        except BaseException:
            # CL3: from here on the demo runs; a failure before main()'s try/finally reaches close() left the whole
            # demo (simulator, gateway, MediaMTX, the test cameras' ffmpeg) running - stop it (restart it when it ran)
            demo.stop()
            if self.was_running:
                time.sleep(1.5)
                demo.start()
            raise

    def _open(self):
        demo = self.demo
        with open(demo.PIDS, encoding="utf-8") as f:
            pids = json.load(f)
        for name in ("shore", "onboard"):                 # this script is the ground station
            if name in pids:
                demo.kill_tree(pids[name])
        import tkinter as tk
        import app as A
        import i18n
        import proto
        from link import Link
        self.A, self.i18n, self.proto = A, i18n, proto
        self.recorder = None                              # a running Clip: one frame per pump step when due
        say("starting the ground station ...")
        self.root = root = tk.Tk()
        root.tk.call("tk", "scaling", 96 / 72.0)          # the normal 100 % sizes, in real pixels
        self.errors = []
        root.report_callback_exception = lambda *a: self.errors.append(repr(a[1]))
        boat_file = os.path.join(REPO, "demo_run", "boat_demo.json")
        self.ap = A.App(root, "gw:127.0.0.1:5761", login=(demo.USER, demo.PASSWORD), charts_dir=demo.demo_charts(),
                        layout=os.path.join(TMP, "layout.json"), cams="Full",
                        boat_file=boat_file if os.path.isfile(boat_file) else None)
        root.state("normal")
        root.geometry("1600x900+0+960")
        say("connecting through the gateway ...")
        if not self.pump(25, lambda: self.ap.t is not None and self.ap.gw_status is not None):
            raise RuntimeError("no data through the gateway")
        self.ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
        self.pump(1.0)
        self.ap.open_driver()
        self.dv = self.ap.driver
        self.dv.w.geometry("1600x900+0+0")
        self.ap.open_map()                                # screen 2 = the map with the corner panels
        self.pump(2.0)
        # the onboard side: the simulator's USB joystick (centred, lever in neutral) on the USB link
        self.sim = Link("tcp:127.0.0.1:5760")
        self.sim.start()
        self.jm = proto.default_joymap(1)
        self.bridge()
        self.pump(1.0)

    def pump(self, s, until=None):
        end = time.time() + s
        import _tkinter
        while time.time() < end:
            # a bounded number of events per step (root.update() never returns when the station's timers are due
            # faster than they run - seen with the plot page on a busy PC)
            for _i in range(400):
                if not self.root.tk.dooneevent(_tkinter.ALL_EVENTS | _tkinter.DONT_WAIT):
                    break
            if self.recorder is not None:
                self.recorder.tick()
            if until and until():
                return True
            time.sleep(0.02)
        return bool(until and until())

    def bridge(self, x=0.0, y=0.0, lever=0.5, codes=()):
        b = [0, 0]
        for c in codes:
            if 1 <= c <= 64:
                b[(c - 1) // 32] |= 1 << ((c - 1) % 32)
        self.sim.send(self.proto.T_SIM_BRIDGE, self.proto.p_sim_bridge(x, y, lever, b[0], b[1], 15, 15, True, True,
                                                                       False, 0))

    def stick(self, x=0.0, y=0.0):
        """The onboard stick X / Y (the USB stick or the analog joystick, as the boat's source)."""
        if self.t().get("usb"):
            self.bridge(x=x, y=y)
        else:
            self.sim.send(self.proto.T_SIM_JOYSTICK, self.proto.p_sim_joystick(True, x, y))

    def confirm(self):
        """The onboard joystick CONFIRM, stick centred: the simulated USB stick when the boat's source is the USB
        bridge (TELEM "usb"), else the simulated analog joystick (as the station's sim_confirm)."""
        proto = self.proto
        if self.t().get("usb"):
            self.bridge(codes=[self.jm["btn"][proto.JM_CONFIRM]])
            self.pump(0.3)
            self.bridge()
        else:
            self.sim.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
            self.pump(0.3)
            self.sim.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))

    def t(self):
        return self.ap.t or {}

    def uav(self):
        return (self.ap.gw_status or {}).get("uav") or {}

    def grab(self, win, path):
        """The window's client area as a PIL image (and saved when path)."""
        self.pump(0.6)
        im = window_image(win)
        if path:
            if path.endswith(".jpg"):
                im.convert("RGB").save(path, quality=86, optimize=True)
            else:
                im.save(path, optimize=True)
        return im

    # --- the scenario steps --------------------------------------------------------------------------------------
    def relay_table(self):
        """The demo boat file's relay table on the simulated AURIX (relay 8 = the UAV hatch): the station pushes it
        in STANDBY (the driver screen's ACCEPT), the onboard CONFIRM lets the AURIX take it."""
        proto = self.proto
        if self.ap.boat is None or self.ap.boat.error:
            say("  no demo boat file: the UAV launch will be refused (125)")
            return False
        want = proto.relay_table_hash(self.ap.boat.relays)
        end = time.time() + 40
        last = 0
        while time.time() < end:
            if time.time() - last > 3:
                last = time.time()
                say("  relay: hash %s want %s confirm %s pending %s profile %s joy %s q %s msg %r" % (
                    self.t().get("relay_hash"), want, self.t().get("confirm_kind"), self.t().get("relay_pending"),
                    self.t().get("profile_name"), (self.t().get("joy_status"), self.t().get("usb")),
                    bool(self.dv.question), getattr(self.ap, "msg_text", "")))
            if self.t().get("relay_hash") == want:
                say("  relay table active on the AURIX")
                return True
            if self.dv.question:
                self.dv.answer(True)
            if self.t().get("confirm_kind"):
                self.pump(0.5)
                self.confirm()
            self.pump(0.5)
        say("  the relay table was not taken")
        return False

    def uav_launch(self):
        """UAV 1: REGISTER, a route ahead of the boat, PREPARE LAUNCH, the captain's YES -> airborne."""
        ap = self.ap
        ap.link.send_msg({"t": "uav_register", "slot": 1})
        ok = self.pump(40, lambda: ((self.uav().get("slots") or [{}])[0] or {}).get("registered"))
        say("  UAV 1 registered: %s" % ok)
        d = self.t()
        lat, lon = d.get("est_lat"), d.get("est_lon")
        if lat is None:
            return False
        wps = [{"lat": lat + 0.0030, "lon": lon + 0.0025, "alt": 50}, {"lat": lat + 0.0055, "lon": lon - 0.0010,
                                                                         "alt": 60}]
        ap.link.send_msg({"t": "uav_route", "target": 1, "wps": wps})
        self.pump(3.0)
        ap.link.send_msg({"t": "uav_launch_prepare", "count": 1})
        if self.pump(20, lambda: self.dv.question is not None):
            self.pump(1.0)
            self.dv.answer(True)
        ok = self.pump(120, lambda: 1 in ap.uav_flying())
        say("  UAV 1 flying: %s (launch %s)" % (ok, (self.uav().get("launch") or {}).get("state")))
        return ok

    def auto_mission(self):
        import program
        proto = self.proto
        ap, dv = self.ap, self.dv
        ap.prog = program.load(os.path.join(REPO, "demo", "demo_mission.json"))
        ap.prog_refresh()
        ap.prog_upload()
        self.pump(4.0)
        self.auto_go()
        say("mode %s; driving the mission ..." % self.t().get("mode_name"))
        self.pump(45.0, lambda: self.t().get("est_sog", 0) > 5.5)
        self.pump(10.0)

    def auto_go(self):
        """AUTO START (or again after a takeover): the driver screen's ACCEPT + the onboard CONFIRM."""
        proto = self.proto
        ap, dv = self.ap, self.dv
        for _try in range(3):
            ap.auto_start()
            self.pump(5.0, lambda: dv.question is not None)
            if dv.question:
                dv.answer(True)
            self.pump(1.5)
            self.confirm()
            if self.pump(6.0, lambda: self.t().get("mode") == proto.MODE_AUTO):
                break
            say("  AUTO not started: confirm %s msg %r alarms %#x" % (self.t().get("confirm_kind"),
                getattr(ap, "msg_text", ""), self.t().get("alarm_bits", 0)))

    def ferry(self):
        say("the collision ferry (DEMO_SHIP) ...")
        self.demo.ship()
        ok = self.pump(150.0, lambda: "collision" in [b[0] for b in self.ap.banner_list()])
        say("  COLLISION RISK banner: %s" % ok)
        self.pump(2.0)

    def close(self):
        if self.recorder is not None:                     # CL3: a clip cut short by an error: its ffmpeg ends here
            try:
                self.recorder.p.stdin.close()
                self.recorder.p.wait(10)
            except Exception:
                try:
                    self.recorder.p.kill()
                except Exception:
                    pass
            self.recorder = None
        try:
            self.sim.stop()
        except Exception:
            pass
        try:
            self.ap.quit()
        except Exception:
            pass
        say("errors in the ground station: %d %s" % (len(self.errors), self.errors[:3]))
        self.demo.stop()
        if self.was_running:
            say("restarting the normal demo ...")
            time.sleep(1.5)
            self.demo.start()


# --- the guided tour's pictures (step 13t a) ---------------------------------------------------------------------
def frac(r, W, H):
    """(x0, y0, x1, y1) pixels -> [fx, fy, fw, fh] fractions, rounded."""
    x0, y0, x1, y1 = r
    return [round(x0 / float(W), 5), round(y0 / float(H), 5), round((x1 - x0) / float(W), 5),
            round((y1 - y0) / float(H), 5)]


def union_px(rs):
    rs = [r for r in rs if r]
    if not rs:
        return None
    return (min(r[0] for r in rs), min(r[1] for r in rs), max(r[2] for r in rs), max(r[3] for r in rs))


def widget_px(w, win):
    """A widget's rectangle relative to the window it is grabbed with, None when it is not shown."""
    if not w.winfo_ismapped():
        return None
    x, y = w.winfo_rootx() - win.winfo_rootx(), w.winfo_rooty() - win.winfo_rooty()
    return (x, y, x + w.winfo_width(), y + w.winfo_height())


def force_hud(se):
    """Every button of screen 1's bottom bar + the drive buttons + a relay line in the TOUR picture (display only:
    on the boat they appear only in their situations). Returns the undo."""
    dv, T = se.dv, se.i18n.T
    orig = dv.snapshot

    def snap():
        s = orig()
        s["manual_ok"] = True
        if not s.get("vision"):
            s["vision"] = (T("hud.vision", state=T("vis.ACTIVE"), fps="2.0"), "good")
        s["vision_cap"] = True
        s["slam_restore"] = True
        if not s.get("hail"):
            s["hail"] = {"label": T("hail.ptt"), "talking": False,
                         "enabled": True, "text": "", "caution": False}
        if not s.get("relay_lines"):
            s["relay_lines"] = [[{"n": 1, "name": "TRIM UP", "state": "off", "click": True},
                                 {"n": 2, "name": "TRIM DOWN", "state": "off", "click": True}],
                                [{"n": 3, "name": "NAV LIGHTS", "state": "on", "click": True}]]
        return s
    dv.snapshot = snap
    return lambda: setattr(dv, "snapshot", orig)


def tour_s1(se, lang, man):
    import hud
    dv = se.dv
    dv.draw()
    se.pump(1.5)
    im = se.grab(dv.cv, None)
    W, H = dv.size_int
    if (im.width, im.height) != (W, H):                     # zoom 1: the canvas = the composed picture
        raise RuntimeError("screen 1 picture %dx%d != composed %dx%d" % (im.width, im.height, W, H))
    s = dv.snapshot()
    items = {}
    for name in hud.OVERLAYS:
        if dv.hud.shown(name, s):
            items["hud:" + name] = frac(dv.layout.px(name, W, H), W, H)
    items["hud:logo"] = frac(dv.hud.logo_rect(), W, H)
    items["hud:center"] = [0.25, 0.22, 0.50, 0.53]          # the free centre (CLAUDE.md G)
    acts = {}
    for r, act in dv.hits:
        acts.setdefault(act, []).append(r)
    for act, rs in acts.items():
        items["hudbtn:" + act] = frac(union_px(rs), W, H)
    name = "s1_%s" % lang
    im.convert("RGB").save(os.path.join(TOUR_OUT, name + ".jpg"), quality=86, optimize=True)
    man["shots"][name] = {"file": name + ".jpg", "w": W, "h": H, "items": items}
    say("  tour %s: %d items" % (name, len(items)))
    # the YES / NO question box (step 13p: lower part, YES on top)
    dv.ask(se.i18n.T("tour.question") if se.i18n.has("tour.question") else "?", lambda why: None, lambda why: None,
           30.0, tag="tour_shot")
    se.pump(1.5)
    im = se.grab(dv.cv, None)
    items2 = dict(items)
    items2["hudbtn:accept"] = frac(union_px([widget_px(dv.q_yes, dv.cv), widget_px(dv.q_no, dv.cv)]), W, H)
    items2["hud:center"] = frac(widget_px(dv.qbox, dv.cv), W, H)
    name = "s1q_%s" % lang
    im.convert("RGB").save(os.path.join(TOUR_OUT, name + ".jpg"), quality=86, optimize=True)
    man["shots"][name] = {"file": name + ".jpg", "w": W, "h": H, "items": items2}
    dv.cancel("tour_shot")
    se.pump(0.5)


def force_banner(se):
    """Screen 2's banner row in the TOUR picture: when no banner is up now, the COLLISION RISK banner of the
    nearest AIS target (as the station shows it at the danger level; display only). Returns the undo."""
    import theme
    ap, T = se.ap, se.i18n.T
    orig = ap.banner_list

    def banners():
        out = orig()
        if out:
            return out
        th = sorted(((ap.gw_status or {}).get("avoid") or {}).get("threats") or [], key=lambda t: t.get("cpa_nm", 9))
        t = th[0] if th else {"name": "DEMO FERRY", "cpa_nm": 0.1, "tcpa_min": 4.0, "bearing": 90.0}
        return [("collision", T("drv.collision", name=t.get("name") or "?", cpa="%.2f" % t.get("cpa_nm", 0.1),
                                tcpa="%.1f" % t.get("tcpa_min", 4.0), brg="%03.0f" % t.get("bearing", 90.0)),
                 theme.WHITE, theme.WARN, None)]
    ap.banner_list = banners
    return lambda: setattr(ap, "banner_list", orig)


def force_assistant(se):
    """The talking manual's panel in the TOUR picture (step AM5): it appears only once a question was heard - here a
    canned answer from the committed index (assistant/show.demo_answer: no model, no microphone, nothing spoken -
    the sound is off in this process). Returns the undo."""
    import assistant.show as show
    ap = se.ap
    a = ap.asst
    orig = a.last, a.answer
    sv = ap.asst_voice.speak
    ap.asst_voice.speak = lambda info, user=False: False      # display only: no TTS run for the picture

    def set_lang(lang):
        info = show.demo_answer(lang)
        if info:
            a.last = {"q": info["q"], "lang": lang, "audio_s": 2.0, "ms": 800, "found": info["found"]}
            a.answer = dict(info, speak=False, answer_ms=2100)
            ap.asst_shown = None
    se.asst_demo = set_lang

    def undo():
        a.last, a.answer = orig
        ap.asst_voice.speak = sv
        se.asst_demo = None
    return undo


def force_trends(se):
    """The trend panel + the screen-1 strip in the TOUR picture (step TR4): the panel OPEN with the acceleration graph
    and the strip ON (display only: both start closed / off on the boat; settings of THIS process's temp file only).
    Returns the undo."""
    import settings
    ap = se.ap
    old = {k: settings.get(k) for k in ("trends_hud", "trends_show", "trends_hud_show", "trends_zoom")}
    was_collapsed = "trends" in ap.ov.collapsed
    settings.put("trends_hud", True)
    settings.put("trends_show", [1, 2, 4, 5, 9, 8])
    settings.put("trends_hud_show", [1, 4, 9])
    settings.put("trends_zoom", 2)
    ap.ov.collapsed.discard("trends")

    def undo():
        for k, v in old.items():
            settings.put(k, v)
        if was_collapsed:
            ap.ov.collapsed.add("trends")
    return undo


def force_harbour(se):
    """The harbour panel in the TOUR picture (step HX6): it appears only while a harbour route / mode exists - here a canned
    picture (the committed Daskalio TEST sample route, a fixed RUN state with two tracks), display only, nothing is sent
    (the model is replaced in this process only). Returns the undo."""
    import time as _t
    import harbourroute
    import harbourview
    ap = se.ap
    orig = ap.harbour_state
    route = harbourroute.load(os.path.join(harbourroute.SAMPLES_DIR, "daskalio", "sample_exit.json"))
    gwh = {"enabled": True, "sensors": {"radar": {"state": "OK"}, "lidar": {"state": "OK"}}, "tracker": {"unsure": []},
           "planner": {"stat_fresh": True}}

    def tr(i, src, steer):
        return {"id": i, "lat": 37.8201 + 0.0001 * i, "lon": 23.6004, "speed": 1.2, "course": 80.0, "cls": "vessel",
                "conf": 0.8, "src": src, "steer_ok": steer, "static": False}

    def fake():
        v = {"t": "harbour_view", "_rx": _t.time() + 1e6, "tracks": [tr(1, "R", True), tr(2, "C", False)],
             "plan": {"state_name": "RUN", "stat_fresh": True, "kn": 3.0, "corr_m": 10.0, "xte_m": 1.2, "route": True,
                      "entry_ok": True, "max_kn": 5.0, "stop": False, "heading": 60.0, "cap_kn": 3.0,
                      "text_el": "ΔΙΑΔΡΟΜΟΣ ΕΛΕΥΘΕΡΟΣ", "text_en": "ROUTE CLEAR", "unsure": [], "hflags": 0}}
        ap.harbour_view = v
        ap.harbour_model = harbourview.model(gwh, v, _t.time(), se.i18n.lang())
        return ap.harbour_model
    ap.harbour_state = fake
    ap.harbour_route = route

    def undo():
        ap.harbour_state = orig
        ap.harbour_route = ap.harbour_view = ap.harbour_model = None
    return undo


def trend_shots(se):
    """docs/screens/trends/: screen 2's open panel (acceleration graph) and screen 1's edge strip, el + en (PrintWindow
    pictures of the station's own windows). Called inside tour_shots (the panel is forced open there)."""
    out = os.path.join(REPO, "docs", "screens", "trends")
    os.makedirs(out, exist_ok=True)
    for lang in ("el", "en"):
        se.i18n.set_lang(lang)
        se.pump(1.5)
        se.ap.render(force=True)
        se.dv.draw()
        se.pump(1.5)
        se.grab(se.ap.root, os.path.join(out, "screen2_trends_%s.png" % lang))
        se.grab(se.dv.w, os.path.join(out, "screen1_strip_%s.png" % lang))
        say("  saved trends pictures (%s)" % lang)
    se.i18n.set_lang("el")


def tour_s2(se, lang, man):
    ap = se.ap
    if getattr(se, "asst_demo", None):
        se.asst_demo(lang)
    ap.mapwin.redraw()
    ap.render(force=True)
    se.pump(1.5)
    ap.render(force=True)
    im = se.grab(ap.cv, None)
    W, H = ap.ov.W, ap.ov.H
    if (im.width, im.height) != (int(W), int(H)):
        raise RuntimeError("screen 2 picture %dx%d != canvas %dx%d" % (im.width, im.height, W, H))
    ops = ap.ov.ops
    by = {}
    for op in ops:
        by.setdefault(op.get("tag"), []).append(op["r"])
    items = {}
    for op in ops:
        if op["k"] == "panel":
            items["s2:" + op["panel"]] = frac(op["r"], W, H)
    mb = [it["tag"] for it in ap.mode_bar_items()]
    groups = {"modebar": mb, "battle_short": ["battle_short"], "standby": ["standby"], "bar1": ["bar1"],
              "bar2": ["bar2"], "camview": ["cv_open"], "tracks": ["tracks", "track_new"],
              "screenrec": ["scr", "keep5", "keep60"], "loudhailer": ["ptt"]}
    for key, tags in groups.items():
        u = union_px([r for tg in tags for r in by.get(tg, [])])
        if u:
            items["s2:" + key] = frac(u, W, H)
    ban = union_px([op["r"] for op in ops if op["k"] == "banner"])
    if ban:
        items["s2:banners"] = frac(ban, W, H)
    cx, cy, kr = ap.mapwin.keepout(W, H)
    items["s2:boat"] = items["s2:edit"] = frac((cx - kr, cy - kr, cx + kr, cy + kr), W, H)
    name = "s2_%s" % lang
    im.convert("RGB").save(os.path.join(TOUR_OUT, name + ".jpg"), quality=86, optimize=True)
    man["shots"][name] = {"file": name + ".jpg", "w": int(W), "h": int(H), "items": items}
    say("  tour %s: %d items (banners: %s)" % (name, len(items), "yes" if ban else "NONE"))


def tour_tools(se, lang, man):
    ap = se.ap
    win = ap.tools
    ap.open_tab(ap.tab_keys[0])
    win.geometry("1500x1150+40+40")                  # tall enough for every button of the UAV magazine page
    for i, key in enumerate(ap.tab_keys):
        ap.open_tab(key)
        se.pump(1.2)
        im = se.grab(win, None)
        W, H = im.width, im.height
        items = {}
        for j, k in enumerate(ap.tab_keys):
            r = widget_px(ap.nb.btns[j], win)
            if r:
                items["tools:" + k] = frac(r, W, H)
        if key == "tab.uav":
            pg = ap.uav_page
            wl = {"register": [b for _l, b in pg.rows], "fly": list(pg.b_fly), "prep": [pg.b_prep],
                  "stop": [pg.b_stop], "close": [pg.b_close], "logs": [pg.b_logs], "csv": [pg.b_csv],
                  "gpx": [pg.b_gpx], "tlog": [pg.b_tlog], "give": [pg.b_give]}
            for k, ws in wl.items():
                u = union_px([widget_px(w, win) for w in ws])
                if u:
                    items["uav:" + k] = frac(u, W, H)
        name = "tools_%s_%s" % (key.replace("tab.", ""), lang)
        im.convert("RGB").save(os.path.join(TOUR_OUT, name + ".jpg"), quality=86, optimize=True)
        man["shots"][name] = {"file": name + ".jpg", "w": W, "h": H, "items": items}
    say("  tour tools pages %s: %d" % (lang, len(ap.tab_keys)))
    win.withdraw()


def tour_shots(se):
    import tour
    import tourshots
    os.makedirs(TOUR_OUT, exist_ok=True)
    for f in os.listdir(TOUR_OUT):
        if f.endswith(".jpg"):
            os.remove(os.path.join(TOUR_OUT, f))
    man = {"kind": "tour_shots", "_help": "Made by python tools/demo_screens.py --tour from the running demo "
           "(step 13t): the picture of every tour stop (s1 / s1q / s2 / tools_<page>, el + en) and the REAL place of "
           "every screen item in it, as fractions [x, y, w, h] of the picture. academy/tourshots.py reads it; "
           "'signature' = the layout these pictures belong to (stale = make them again).",
           "made": time.strftime("%Y-%m-%d %H:%M"), "signature": tourshots.signature(tour), "shots": {}}
    undo_hud, undo_ban, undo_asst = force_hud(se), force_banner(se), force_assistant(se)
    undo_trends = force_trends(se)
    undo_harbour = force_harbour(se)
    try:
        trend_shots(se)
        for lang in ("el", "en"):
            se.i18n.set_lang(lang)
            se.pump(1.5)
            tour_s1(se, lang, man)
            tour_s2(se, lang, man)
            tour_tools(se, lang, man)
    finally:
        undo_hud()
        undo_ban()
        undo_asst()
        undo_trends()
        undo_harbour()
        se.i18n.set_lang("el")
    with open(os.path.join(TOUR_OUT, "shots.json"), "w", encoding="utf-8") as f:
        json.dump(man, f, indent=1, ensure_ascii=False)
    probs = tourshots.check(tour, man)
    for p in probs:
        say("  TOUR PROBLEM " + p)
    say("tour pictures: %d, problems %d" % (len(man["shots"]), len(probs)))
    return not probs


# --- the presentation pack (step 13t b) ---------------------------------------------------------------------------
class Clip:
    """A window recorded as an MP4: PrintWindow pictures at 10 fps (our window only, also when covered) piped into
    ffmpeg, H.264 -preset veryfast (the encoder settings of REC SCREEN)."""

    FPS = 10

    def __init__(self, se, win, path, seconds):
        import toolpaths
        self.se, self.win, self.path = se, win, path
        win.update_idletasks()
        self.size = (win.winfo_width() // 2 * 2, win.winfo_height() // 2 * 2)
        self.n = int(seconds * self.FPS)
        self.done = 0
        self.t0 = time.time()
        self.p = subprocess.Popen([toolpaths.ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                   "-s", "%dx%d" % self.size, "-r", str(self.FPS), "-i", "-",
                                   "-vf", "scale='min(1280,iw)':-2", "-c:v", "libx264", "-preset", "veryfast", "-crf", "26",
                                   "-pix_fmt", "yuv420p", "-movflags", "+faststart", path],
                                  stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        se.recorder = self

    def tick(self):
        """One frame when it is due (frames keep the time: a slow step repeats the picture)."""
        due = min(self.n, int((time.time() - self.t0) * self.FPS) + 1)
        if self.done >= due:
            return
        try:
            im = window_image(self.win)
        except Exception:
            return
        if im.size != self.size:
            im = im.resize(self.size)
        data = im.tobytes()
        while self.done < due:
            self.p.stdin.write(data)
            self.done += 1

    def wait(self):
        se = self.se
        while self.done < self.n:
            se.pump(0.05)
        se.recorder = None
        self.p.stdin.close()
        self.p.wait()
        err = self.p.stderr.read().decode("utf-8", "replace").strip()
        ok = self.p.returncode == 0 and os.path.isfile(self.path) and os.path.getsize(self.path) > 10000
        say("  clip %s: %s %s" % (os.path.basename(self.path), "OK" if ok else "FAILED", err[:200]))
        return ok


def presentation(se):
    os.makedirs(PRES_OUT, exist_ok=True)
    for f in os.listdir(PRES_OUT):                   # this script's own old pictures / clips (made again below)
        if f.endswith((".png", ".mp4")):
            os.remove(os.path.join(PRES_OUT, f))
    ap, dv, proto, i18n = se.ap, se.dv, se.proto, se.i18n
    made = []

    def shot(win, name, what, langs=("el", "en")):
        for lang in langs:
            i18n.set_lang(lang)
            ap.render(force=True)
            se.pump(1.2)
            fn = "%s_%s.png" % (name, lang)
            se.grab(win, os.path.join(PRES_OUT, fn))
            made.append((fn, what))
            say("  saved %s" % fn)
        i18n.set_lang("el")
        se.pump(0.5)

    def clip(win, name, what, seconds, during=None):
        fn = name + ".mp4"
        c = Clip(se, win, os.path.join(PRES_OUT, fn), seconds)
        if during:
            during()
        if c.wait():
            made.append((fn, what))

    # 1 AUTO route (the mission runs, the UAV flies)
    shot(dv.w, "01_screen1_auto", "Screen 1 in AUTO: the camera video with the green HUD, the route ribbon and the "
         "waypoint (AR), speed, heading tape, the bottom bar")
    shot(ap.root, "02_screen2_auto", "Screen 2 in AUTO: the map with the route, the boat in the centre, the AIS "
         "traffic, UAV 1 in the air, the mode bar (AUTO lit) and the corner panels")
    clip(dv.w, "clip1_auto_screen1", "AUTO on screen 1 (25 s)", 25)
    # 2 HDG LOCK (the panel button), then the ferry: COLLISION RISK -> the avoidance proposal on screen 1 (in AUTO the
    # gateway slows / turns by itself; in HDG LOCK it proposes and the captain answers)
    se.sim.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))      # the panel's HDG LOCK button
    se.pump(10.0, lambda: se.t().get("mode") == proto.MODE_HDG_LOCK)
    say("  mode %s" % se.t().get("mode_name"))
    se.pump(90.0, lambda: (se.t().get("est_sog") or 0.0) < 0.5)      # HDG LOCK: the throttle on the stick = idle
    se.ferry()                                       # (aimed at where the boat is now: it has stopped)
    shot(ap.root, "03_screen2_collision", "Screen 2 in HDG LOCK: the DEMO FERRY on a collision course - COLLISION "
         "RISK, the traffic panel with CPA / TCPA")
    shot(dv.w, "04_screen1_collision", "Screen 1: the collision line to the ferry on the HUD and the alarm")
    # 3 the avoidance proposal on the captain's screen 1
    got = se.pump(90.0, lambda: dv.question is not None)
    if got:
        se.pump(1.0)
        shot(dv.w, "05_screen1_proposal", "HDG LOCK: the gateway's avoidance proposal on the captain's screen 1 - "
             "YES on top, NO underneath (the trigger answers with the selection)")
        clip(dv.w, "clip2_collision_proposal", "The ferry and the avoidance proposal, the captain answers YES (25 s)",
             25, during=lambda: (se.pump(8.0), dv.answer(True) if dv.question else None))
    else:
        av = (ap.gw_status or {}).get("avoid") or {}
        say("  no avoidance proposal came (avoid state %s, %d threats)" % (av.get("state"), len(av.get("threats") or [])))
    # 4 AUTO again, then the joystick override IN AUTO: the onboard stick takes over (CREWED: MANUAL at once)
    se.auto_go()
    say("  mode %s (for the override)" % se.t().get("mode_name"))
    se.pump(8.0)

    def override():
        se.pump(5.0)
        se.stick(0.5)
        se.pump(6.0)
        se.stick(0.0)
    clip(ap.root, "clip3_joystick_override", "Joystick override in AUTO: the onboard stick moved -> MANUAL at "
         "once, the autopilot steps back (screen 2, 25 s)", 25, during=override)
    shot(ap.root, "06_screen2_manual", "Screen 2 after the joystick override: MANUAL lit on the mode bar")
    # 5 the UAV video
    if ap.uav_video_slots():
        ap.uav_view_open(ap.uav_video_slots()[0])
        se.pump(3.0)
        shot(ap.root, "07_screen2_uav_video", "Screen 2's camera view with UAV 1's video, its green values and the "
             "buttons of the cameras / UAVs")
        clip(ap.root, "clip4_uav_video", "UAV 1's video on screen 2 (20 s)", 20)
        ap.camview_close()
        se.pump(1.0)
    ap.open_tab("tab.uav")
    se.pump(1.5)
    shot(ap.tools, "08_tools_uav_magazine", "Tools -> UAV magazine: six slots, REGISTER, PREPARE LAUNCH, FLY UAV, "
         "flight logs")
    ap.tools.withdraw()
    # 5 the loudhailer: TALK held
    if ap.hail.view() is not None:
        def talk():
            se.pump(4.0)
            ap.hail.press()
            se.pump(10.0)
            ap.hail.release()
        ap.hail.press()
        se.pump(2.0)
        shot(ap.root, "09_screen2_loudhailer", "The loudhailer: TALK held on screen 2's bar 2 (press and hold)")
        ap.hail.release()
        se.pump(1.0)
        clip(ap.root, "clip5_loudhailer", "The loudhailer: TALK pressed and held, let go (20 s)", 20, during=talk)
    # 6 the mode bar (a crop of screen 2) + BATTLE SHORT
    for lang in ("el", "en"):
        i18n.set_lang(lang)
        ap.render(force=True)
        se.pump(1.2)
        im = se.grab(ap.cv, None)
        rs = [op["r"] for op in ap.ov.ops if op.get("tag") in [it["tag"] for it in ap.mode_bar_items()] +
              ["battle_short"]]
        u = union_px(rs)
        if u:
            fn = "10_mode_bar_%s.png" % lang
            im.crop((int(u[0]) - 12, int(u[1]) - 10, int(u[2]) + 12, int(u[3]) + 10)).save(os.path.join(PRES_OUT, fn))
            made.append((fn, "The mode bar: every mode on one line, the active one inverted, BATTLE SHORT at the end"))
    i18n.set_lang("el")
    for lang in ("el", "en"):
        i18n.set_lang(lang)
        bs = ap.battle_short()
        se.pump(1.0)
        fn = "11_battle_short_%s.png" % lang
        se.grab(bs.w, os.path.join(PRES_OUT, fn))
        made.append((fn, "BATTLE SHORT (demo theatre only: every code is refused, nothing is sent anywhere)"))
        bs.w.destroy()
        se.pump(0.5)
    i18n.set_lang("el")
    bs = ap.battle_short()
    se.pump(1.0)

    def attempt():
        for _ in range(3):
            se.pump(3.0)
            try:
                bs.entry.delete(0, "end")
                bs.entry.insert(0, "1234")
                bs.authorise()
            except Exception as e:
                say("  battle short: %r" % e)
                return
        se.pump(3.0)
    clip(bs.w, "clip6_battle_short", "BATTLE SHORT: an authorisation code typed -> INVALID AUTHORISATION CODE "
         "(red flash + shake), every time", 15, during=attempt)
    try:
        bs.w.destroy()
    except Exception:
        pass
    with open(os.path.join(PRES_OUT, "README.md"), "w", encoding="utf-8") as f:
        f.write(PRES_README % time.strftime("%Y-%m-%d"))
        for fn, what in made:
            f.write("| [%s](%s) | %s |\n" % (fn, fn, what))
        f.write(PRES_NOTES)
    say("presentation pack: %d files in docs/presentation/" % len(made))
    return made


PRES_README = """# Trygons C2 - presentation pack

Made by `python tools/demo_screens.py --presentation` on %s from the running demo (simulated boat, gateway,
cameras, AIS traffic, the UAV magazine with simulated drones). Every picture is the REAL ground station
(screen 1 = the driver's camera screen with the HUD, screen 2 = navigation & control). Greek (`_el`) and
English (`_en`) versions. The MP4 clips are recordings of the station's own windows (Windows PrintWindow
pictures - never the rest of the desktop - piped into ffmpeg), H.264 `-preset veryfast` as REC SCREEN, at most
1280 wide, 10 fps.

| File | What it shows |
|---|---|
"""
PRES_NOTES = """
Notes
- The front camera is the demo's sea video (MaSTr1325 pictures, CC BY 4.0 - demo/media/ATTRIBUTION.txt) when
  `tools/get_vision_samples.py` was run, else the test pictures.
- The joystick override is the CREWED rule: the onboard stick moved = MANUAL at once (the joystick always wins).
- BATTLE SHORT is demo theatre only: no code exists, nothing is sent to the boat or the gateway.
- AIS does not show small craft (kayaks, swimmers): the crew keeps watch.
- Regenerate: `python tools/demo_screens.py --presentation` (Windows; the demo stops and starts again).
"""


# --- the older review set (section G1) ----------------------------------------------------------------------------
def review(se):
    import settings
    import theme
    ap, dv, proto, i18n = se.ap, se.dv, se.proto, se.i18n

    def grab(win, name, what):
        se.grab(win, os.path.join(OUT, name + ".png"))
        shots.append((name, what))
        say("  saved %s - %s" % (name, what))
    os.makedirs(OUT, exist_ok=True)
    for lang in ("el", "en"):
        i18n.set_lang(lang)
        se.pump(1.5)
        grab(dv.w, "driver_day_%s" % lang, "driver HUD, day, %s: AUTO mission, AR overlays, AIS threat DEMO FERRY"
             % ("Greek" if lang == "el" else "English"))
    i18n.set_lang("el")
    se.sim.send(proto.T_SIM_GPS, proto.p_sim_gps(True))                 # GPS lost: a CAUTION alarm
    se.pump(8.0, lambda: se.t().get("alarm_bits", 0) & 0x002)
    se.pump(2.0)
    grab(dv.w, "driver_alarm_el", "driver HUD with active alarms: MASTER CAUTION lit, alarm bar")
    i18n.set_lang("en")
    se.pump(1.0)
    grab(dv.w, "driver_alarm_en", "the same in English")
    se.sim.send(proto.T_SIM_GPS, proto.p_sim_gps(False))
    for lang in ("el", "en"):
        i18n.set_lang(lang)
        settings.put("night", True)
        se.pump(1.5)
        grab(dv.w, "driver_night_%s" % lang, "driver HUD, NIGHT mode (dim red), %s" % lang)
    settings.put("night", False)
    i18n.set_lang("el")
    for th, lang in (("dark", "el"), ("light", "en"), ("dark", "en")):
        i18n.set_lang(lang)
        theme.set_theme(th)
        ap.mapwin.redraw()
        ap.render(force=True)
        se.pump(1.5)
        grab(ap.root, "screen2_%s_%s" % (th, lang), "screen 2 (navigation & control), %s theme, %s" % (th, lang))
    theme.set_theme("dark")
    i18n.set_lang("el")


def main(argv=None):
    import faulthandler
    argv = list(sys.argv[1:] if argv is None else argv)
    do_tour = "--tour" in argv or not any(a in argv for a in ("--presentation", "--review"))
    do_pres = "--presentation" in argv or not any(a in argv for a in ("--tour", "--review"))
    do_review = "--review" in argv
    if do_tour and "--if-stale" in argv and not do_pres and not do_review:
        if not tour_stale():
            say("tour pictures: current (nothing to do)")
            return 0
        say("tour pictures: STALE - making them again from the demo")
    faulthandler.dump_traceback_later(600, repeat=True)          # a hang shows where it is
    se = Session()
    ok = True
    try:
        se.relay_table()
        if "--probe" in argv:
            return 0
        se.uav_launch()
        se.auto_mission()
        if do_tour or do_review:
            se.ferry()                               # the AIS threat on the map + its banner (screen 2)
        if do_review:
            review(se)
        if do_tour:
            ok = tour_shots(se) and ok
        if do_pres:
            presentation(se)
    finally:
        se.close()
    return 0 if ok and not se.errors else 1


if __name__ == "__main__":
    sys.exit(main())
