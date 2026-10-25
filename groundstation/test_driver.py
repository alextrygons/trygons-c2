"""Two-monitor ground station tests (section I): the DRIVER screen.

Starts  sim/autopilot_sim server <port> 1  and the real ground station
(app.App) on it, opens the driver screen and checks:
  I1 HUD: mode, speed, compass tape centred on the heading, throttle bar with
     the power limit, gear letter, G-meter, mini map - from the live telemetry
  I2 a driving-affecting captain action waits for the driver: no answer ->
     rejected (nothing sent); ACCEPT -> sent; STANDBY never waits
  I3 G-meter for slamming (replaces the slamming graph): peak, peak hold, yellow
     and red limit lines, colours
  I4 rear cameras shown as mirrors (picture flipped left <-> right)
  I5 VIDEO FROZEN on the driver screen within 1 s; FRONT-1 lost -> FRONT-2;
     remote driving without a live front camera -> warning
  I6 window places remembered (both monitors); F12 swaps the screens
  I7 AIS list (name, distance, bearing, CPA, TCPA, age; danger first, details
     on click), collision line on the driver HUD, HEADING LOCK proposal as
     ACCEPT / REJECT on the driver screen (gone at the gateway -> closed)
  I8 obstacle detection (vision): mask overlay on the front camera (toggle),
     boxes with bearing + rough distance, status, map sectors, the HEADING
     LOCK slow-down proposal on the driver screen
  I10 full-screen camera: the front picture fills the window (fill = cover,
     cropped evenly / fit), every overlay inside its own box, no text outside
     its box in Greek AND English (E4), main sizes (speed > mode > comm > rest)
  I11 MASTER WARNING / CAUTION lamps: on with an alarm of that level, one click
     acknowledges (lamp off, sound silenced) - nothing is sent to the boat
  I12 night mode + declutter FULL / NORMAL / MINIMAL (MINIMAL = speed, heading,
     mode, gear, alarms), both remembered
  I13 edit-layout mode: move, resize, hide / show, reset, saved to the layout
     file; outside the edit mode a drag moves nothing
  I14 the HUD never changes control behaviour: every HUD button / key pressed,
     no frame to the boat
  I15 AR maths: horizon from the camera tilt, heel turns it, points on the water
     below the horizon, out of view detected, predicted path, route points,
     calibration checked (unknown / out of range = none)
  I16 AR on the front video: without a calibration "calibration unknown" and no
     AR; with one: horizon + heel scale, velocity vector, waypoint diamond (or
     edge arrow), route ribbon, AIS boxes (or edge arrows), predicted path,
     shallow-water tint; each one switched off on its own; vision overlay stays
  I17 MAP window: chart layers, AIS list, own boat, track, route, active
     waypoint, uncertainty, range rings, warnings; follow / free pan, zoom,
     north-up / course-up, day / dark / night; settings remembered; view only
     (nothing sent); moved to a monitor
  (I6 also covers the MAP window: open + place remembered over a restart)
  I18 sound in the real ground station: mode callouts from the live telemetry,
     a WARNING repeats until the MASTER lamp is acknowledged on the driver
     screen, a command answer gives the confirm sound; nothing sent

Run:  python test_driver.py        (exit code 0 = all passed)
"""
import os
import socket
import subprocess
import sys
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
# the ground station in English with a throw-away settings file (never the user's language / HUD settings)
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")                    # silent: no sound, no voice making
os.environ.setdefault("TRYGONS_GS_SETTINGS", os.path.join(__import__("tempfile").gettempdir(), "trygons_gs_test.json"))
TMP = __import__("tempfile").mkdtemp(prefix="trygons_drv_")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(TMP, "gs_settings.json")      # fresh HUD settings every run
os.environ["TRYGONS_HUD_LAYOUT"] = os.path.join(TMP, "hud_layout.json")
sys.path.insert(0, HERE)
import app as A  # noqa: E402
import driver  # noqa: E402
import proto  # noqa: E402

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


class FakeStream:
    """A video stream stand-in: a test picture, or frozen."""

    def __init__(self, w=64, h=36, frozen=False, colour=(200, 40, 40)):
        self.w, self.h, self.is_frozen = w, h, frozen
        px = bytearray()
        for _y in range(h):
            for x in range(w):
                px += bytes(colour) if x < w // 2 else b"\x10\x10\x10"     # left half coloured
        self.data = b"P6\n%d %d\n255\n" % (w, h) + bytes(px)

    def ppm(self):
        return self.data

    def frozen(self):
        return self.is_frozen

    def stop(self):
        pass


def items_with_tag(canvas, tag):
    return canvas.find_withtag(tag)


def main():
    sim_dir = os.path.join(HERE, "..", "sim")
    exe = os.path.join(sim_dir, "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    saved_layout = None
    if os.path.exists(A.LAYOUT):
        with open(A.LAYOUT, "rb") as f:
            saved_layout = f.read()
        os.remove(A.LAYOUT)
    port = free_port()
    sim = subprocess.Popen([exe, "server", str(port), "1"], cwd=sim_dir, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
    root = None
    try:
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
                break
            except OSError:
                time.sleep(0.1)
        time.sleep(0.3)
        root = tk.Tk()
        errors = []
        root.report_callback_exception = lambda *a: errors.append(repr(a[1]))
        app = A.App(root, "tcp:127.0.0.1:%d" % port)

        def pump(seconds, until=None):
            end = time.time() + seconds
            while time.time() < end:
                root.update()
                if until and until():
                    return True
                time.sleep(0.02)
            return bool(until and until())

        pump(30, lambda: app.t is not None and app.link and app.link.connected)     # a busy PC: slow start
        root.state("normal")
        root.geometry("1200x800+40+40")
        app.open_driver()
        dv = app.driver
        dv.w.geometry("1600x900+100+60")
        pump(2.0)
        d = app.t

        # I1 the HUD from live telemetry
        mode_txt = (dv.texts("mode") or [None])[0]
        bx = dv.layout.px("bottom", dv.cv.winfo_width(), dv.cv.winfo_height())
        hdg = dv.items("hdg")
        tape_w = (bx[2] - bx[0]) * 0.52
        hdg_ok = bool(hdg) and abs((hdg[0]["r"][0] + hdg[0]["r"][2]) / 2 - (bx[0] + 4 + tape_w / 2)) < 1
        bar = [o for o in dv.items("thr") if o["k"] == "rect" and o.get("outline")]
        plim = dv.items("plim")
        plim_ok = bool(bar and plim) and abs(plim[0]["pts"][1] - (bar[0]["r"][1] + (bar[0]["r"][3] - bar[0]["r"][1]) *
                                                                  (1 - d["power_fwd"] / 100.0))) < 2
        speed = (dv.texts("speed") or ["-"])[0]
        gear = (dv.texts("gear") or ["-"])[0]
        report("I1 driver HUD: mode, speed, compass tape, throttle bar with the power limit, gear, G-meter, map",
               mode_txt == A.T("mode." + d["mode_name"]) and hdg_ok and plim_ok and speed == "%.1f" % d["est_sog"]
               and gear == A.T("gear.%d" % d["gear"]) and dv.has("g_limit") and dv.map.winfo_manager() == "place"
               and not errors,
               "mode \"%s\"; heading box centred on the tape %s; power limit line at %.0f %% %s; speed \"%s\"; "
               "gear \"%s\"; G-meter %s; mini map %s; errors %d" % (
                   mode_txt, hdg_ok, d["power_fwd"], "OK" if plim_ok else "WRONG", speed, gear,
                   "drawn" if dv.has("g_limit") else "MISSING", dv.map.winfo_manager() or "MISSING", len(errors)))

        # I2 the captain's own actions go straight to the boat (no question on its own screen 1; a copilot's
        # wait for the captain's ACCEPT - through the gateway, gateway test F1e); STANDBY never waits
        sea0 = app.t["sea_state"]
        new_sea = 3 if sea0 != 3 else 2
        app.cmd(proto.T_SEA_STATE, proto.p_sea_state(new_sea), "sea state %d" % new_sea)
        no_question = dv.question is None
        changed = pump(5.0, lambda: app.t["sea_state"] == new_sea)
        app.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY), "mode STANDBY")
        sb_direct = dv.question is None
        report("I2 the captain's own action is sent at once (no question on screen 1); STANDBY never waits",
               no_question and changed and sb_direct,
               "no question on screen 1 %s; boat sea state %s -> %s; STANDBY sent at once %s" % (
                   no_question, sea0, app.t["sea_state"], sb_direct))

        # I3 G-meter (replaces the slamming graph): peak, hold, yellow + red lines, colours
        import hud
        import alerts
        gh = hud.Hud(hud.Layout(os.path.join(TMP, "gm_layout.json")))
        base_t = dict(d, est_sog=4.0)

        def gm(peak, hold):
            s = hud.Snapshot(t=base_t, gw={}, lamps=alerts.MasterLamps(), declutter="FULL",
                             gmeter={"peak": peak, "hold": hold, "limit": 3.0, "yellow": 2.1})
            ops, _ = gh.build(1600, 900, s)
            g = {o["tag"]: o for o in ops if o.get("tag", "").startswith("g_")}
            return g
        g1, g2, g3 = gm(1.0, 1.0), gm(2.5, 3.6), gm(3.4, 3.4)
        y = lambda o: o["pts"][1] if o["k"] == "line" else o["pts"][1]          # noqa: E731
        hold_y = g2["g_hold"]["pts"][1]
        peak_top = g2["g_peak"]["r"][1]
        pal = hud.palette()
        report("I3 G-meter for slamming: peak bar, peak hold marker, yellow + red limit lines, colours",
               y(g1["g_limit"]) < y(g1["g_yellow"]) and g1["g_peak"]["fill"] == pal["main"] and
               g2["g_peak"]["fill"] == pal["caution"] and g3["g_peak"]["fill"] == pal["warn"] and hold_y < peak_top,
               "limit line y=%.0f above yellow y=%.0f; 1.0 g %s, 2.5 g %s, 3.4 g %s; hold 3.6 g marker y=%.0f above the "
               "2.5 g bar top y=%.0f" % (y(g1["g_limit"]), y(g1["g_yellow"]), g1["g_peak"]["fill"],
                                         g2["g_peak"]["fill"], g3["g_peak"]["fill"], hold_y, peak_top))

        # I4 mirrors: rear pictures flipped left <-> right (the canvas path + the real HUD picture)
        fs = FakeStream()
        fl = driver.flip_ppm(fs.data)
        hdr = len(b"P6\n64 36\n255\n")
        left_px, right_px = fl[hdr:hdr + 3], fl[hdr + 63 * 3:hdr + 64 * 3]
        app.vstreams = {"FRONT-1": FakeStream(colour=(30, 90, 160)), "REAR-LEFT": FakeStream()}
        app.front_sel = A.video.FrontSelector()
        pump(0.6)
        W_, H_ = dv.cv.winfo_width(), dv.cv.winfo_height()
        mx0, my0, mx1, my1 = dv.layout.px("mirror_l", W_, H_)
        fr = dv.last_frame
        yy = int((my0 + my1) / 2)
        pl = fr.getpixel((int(mx0 + (mx1 - mx0) * 0.15), yy)) if fr else None
        pr = fr.getpixel((int(mx0 + (mx1 - mx0) * 0.85), yy)) if fr else None
        hud_mirror = bool(pl and pr) and pr[0] > 150 and pl[0] < 60          # coloured half now on the right
        report("I4 rear cameras shown as mirrors (flipped left <-> right)",
               fs.data[hdr:hdr + 3] == bytes((200, 40, 40)) and left_px == b"\x10\x10\x10" and
               right_px == bytes((200, 40, 40)) and len(fl) == len(fs.data) and hud_mirror,
               "first pixel before %s, after %s; last pixel after %s; HUD mirror left %s right %s" % (
                   tuple(fs.data[hdr:hdr + 3]), tuple(left_px), tuple(right_px), pl, pr))

        # I5 VIDEO FROZEN within 1 s; FRONT-1 lost -> FRONT-2; remote without front camera -> warning
        app.vstreams = {"FRONT-1": FakeStream(), "FRONT-2": FakeStream(colour=(40, 40, 200)),
                        "REAR-LEFT": FakeStream(), "REAR-RIGHT": FakeStream()}
        app.front_sel = A.video.FrontSelector()
        pump(0.5)
        ok_live = not dv.has("frozen")
        app.vstreams["REAR-LEFT"].is_frozen = True
        t0 = time.time()
        fr = pump(2.0, lambda: dv.has("mirror_l_frozen"))
        t_fr = time.time() - t0
        app.vstreams["FRONT-1"].is_frozen = True
        sw = pump(3.0, lambda: app.front_sel.use == "FRONT-2")
        notice = app.front_sel.notice
        app.vstreams["FRONT-2"].is_frozen = True
        orig = app.on_telem                                 # the boat says REMOTE MANUAL from now on
        app.on_telem = lambda d: orig(dict(d, mode=proto.MODE_REMOTE))
        warn = pump(1.0, lambda: dv.has("nofront"))
        report("I5 driver video: VIDEO FROZEN within 1 s, FRONT-1 lost -> FRONT-2, remote without front camera -> warning",
               ok_live and fr and t_fr < 1.0 and sw and warn,
               "live picture without warning %s; mirror frozen -> VIDEO FROZEN after %.2f s; front switched to %s (\"%s\"); "
               "REMOTE without a front camera -> warning %s" % (ok_live, t_fr, app.front_sel.use, notice,
                                                                 "shown" if warn else "MISSING"))
        app.vstreams = {}
        app.on_telem = orig

        # I7 AIS list + collision avoidance on both screens (gateway status injected)
        d = app.t
        lat0, lon0 = d["est_lat"], d["est_lon"]
        ships = [{"mmsi": 239000001, "cls": "A", "name": "FAR AWAY", "lat": lat0 + 0.05, "lon": lon0, "sog": 12.0,
                  "cog": 90.0, "heading": 90, "age_s": 4.0, "stale": False, "dist_nm": 3.0, "type": 70, "length": 120,
                  "beam": 20, "draught": 6.0, "dest": "PIRAEUS", "callsign": "SVAB1", "src": "local", "turn_dpm": 0.0},
                 {"mmsi": 239000002, "cls": "A", "name": "BLUE STAR", "lat": lat0, "lon": lon0 + 0.02, "sog": 18.0,
                  "cog": 270.0, "heading": 270, "age_s": 2.0, "stale": False, "dist_nm": 0.6, "type": 60,
                  "length": 140, "beam": 22, "draught": 5.5, "dest": "SYROS", "callsign": "SVXY2", "src": "local",
                  "turn_dpm": 0.0},
                 {"mmsi": 239000003, "cls": "B", "name": "ELENI", "lat": lat0 - 0.01, "lon": lon0, "sog": 6.0,
                  "cog": 0.0, "heading": None, "age_s": 400.0, "stale": True, "dist_nm": 0.9, "src": "local",
                  "turn_dpm": 0.0}]
        threats = [{"mmsi": 239000002, "name": "BLUE STAR", "cpa_nm": 0.05, "tcpa_min": 1.8, "dist_nm": 0.6,
                    "bearing": 90.0, "level": "danger", "stale": False},
                   {"mmsi": 239000003, "name": "ELENI", "cpa_nm": 0.4, "tcpa_min": 5.0, "dist_nm": 0.9,
                    "bearing": 180.0, "level": "warn", "stale": True}]
        prop = {"id": 7, "kind": "avoid", "heading": 74, "speed_kn": 4.0,
                "text": "BLUE STAR: CPA 0.05 nm in 1.8 min. Steer 074 and slow to 4 kn?"}
        answers = []
        app.avoid_answer = lambda pid, yes: answers.append((pid, yes))
        app.gw_status = {"ais": {"list": ships}, "avoid": {"state": "PROPOSE", "why": "proposal", "threats": threats,
                                                           "proposal": prop, "route": []}}
        app.draw_ais()
        app.drv_proposal()
        rows = app.ais_tree.get_children()
        first = app.ais_tree.item(rows[0]) if rows else {}
        app.ais_tree.selection_set(rows[0])
        pump(0.3)
        detail = app.ais_detail.cget("text")
        dv.draw()
        hud_txt = dv.texts("collision")
        asked = dv.question is not None and "COLLISION AVOIDANCE" in dv.qhead.cget("text")
        dv.answer(True)
        acc = answers[-1:] == [(7, True)]
        prop2 = dict(prop, id=8)
        app.gw_status["avoid"]["proposal"] = prop2
        app.drv_proposal()
        asked2 = dv.question is not None
        dv.answer(False)
        rej = answers[-1:] == [(8, False)]
        app.gw_status["avoid"]["proposal"] = dict(prop, id=9)
        app.drv_proposal()
        app.gw_status["avoid"]["proposal"] = None
        app.drv_proposal()
        closed = dv.question is None and answers[-1:] == [(8, False)]
        report("I7 AIS list (danger first, CPA / TCPA / age, details on click), collision line on the driver HUD, "
               "HEADING LOCK proposal ACCEPT / REJECT on the driver screen",
               len(rows) == 3 and first.get("text") == "BLUE STAR" and "danger" in first.get("tags", []) and
               first["values"][3] == "0.05" and first["values"][4] == "1.8" and "SYROS" in detail and
               "SVXY2" in detail and hud_txt and "BLUE STAR" in hud_txt[0] and asked and acc and asked2 and rej and
               closed,
               "rows %s; first %s %s; details \"%s\"; HUD \"%s\"; proposal asked %s -> ACCEPT sent %s; next one "
               "REJECT sent %s; withdrawn at the gateway -> closed without an answer %s" % (
                   [app.ais_tree.item(r)["text"] for r in rows], first.get("text"), first.get("values"),
                   detail.replace("\n", " / ")[:120], hud_txt[0] if hud_txt else "MISSING", asked, acc, rej, closed))
        app.gw_status = None
        del app.avoid_answer

        # I8 vision on the driver screen (gateway status injected)
        import base64
        cells = [2] * (64 * 20) + [0] * (64 * 4) + [1] * (64 * 24)          # sky, a band of obstacle, water
        packed = bytes(cells[i] | (cells[i + 1] << 2) | (cells[i + 2] << 4) | (cells[i + 3] << 6)
                       for i in range(0, len(cells), 4))
        vis = {"state": "ACTIVE", "reason": "2.0 fps", "fps": 2.0, "stages": {"stage1": True, "stage2": True,
                                                                         "stage3": True},
               "horizon": "IMU", "obstacles": [{"id": 3, "rel": 2.0, "bearing": 92.0, "dist": 120, "range": [80, 260],
                                                "conf": 0.9, "confirmed": True, "box": [0.45, 0.4, 0.55, 0.55],
                                                "in_corridor": True, "ais": None, "synthetic": False}],
               "threats": 1, "cap_kn": 3.0, "cap_held": True, "proposal": {"id": 5, "text": "OBSTACLE AHEAD ~120 m "
                                                                           "(rough), bearing 092: slow down"},
               "mask": {"w": 64, "h": 48, "b64": base64.b64encode(packed).decode()}, "restarts": 1, "model": "x"}
        sent = []
        app.vision_send = lambda m: sent.append(m)
        app.gw_status = {"vision": vis, "avoid": {}}

        class Pic:
            def __init__(s, w, h):
                s.w, s.h = w, h

            def width(s):
                return s.w

            def height(s):
                return s.h
        ops = dv.draw_vision(dv.front, 640, 360, 640, 360)
        n_mask = len([o for o in ops if o.get("tag") == "vmask"])
        box_txt = [o["s"] for o in ops if o.get("tag") == "vbox" and o["k"] == "text"]
        dv.toggle_overlay()
        ops = dv.draw_vision(dv.front, 640, 360, 640, 360)
        n_off = len([o for o in ops if o.get("tag") == "vmask"])
        dv.toggle_overlay()
        dv.draw()
        hud_txt = " ".join(dv.texts("vision"))
        clear_shown = any(act == "vision_clear" for _r, act in dv.hits)
        dv.action("vision_clear")
        dv.draw_map()
        n_sector = len(items_with_tag(dv.map, "vision"))
        app.drv_vision()
        asked = dv.question is not None and "OBSTACLE DETECTION" in dv.qhead.cget("text")
        dv.answer(True)
        acked = {"t": "vision_ack", "id": 5} in sent
        report("I8 obstacle detection on the driver screen: mask overlay (toggle), box with bearing + rough distance, "
               "status, CLEAR SLOWDOWN, map sector, HDG LOCK slow-down proposal",
               n_mask > 0 and n_off == 0 and box_txt and "092" in box_txt[0] and "rough" in box_txt[0] and
               "VISION ACTIVE" in hud_txt and "HELD" in hud_txt and clear_shown and {"t": "vision_clear"} in sent and
               n_sector > 0 and asked and acked and not errors,
               "overlay %d items (off: %d); box \"%s\"; HUD \"%s\"; clear button %s, sent %s; map sector items %d; "
               "proposal asked %s, acknowledged %s" % (n_mask, n_off, box_txt[0] if box_txt else "MISSING",
                                                       hud_txt[-90:], clear_shown, sent[:1], n_sector, asked, acked))
        app.gw_status = None
        del app.vision_send

        # I10 full-screen camera, every text inside its box in both languages, sizes
        import settings
        import i18n
        app.vstreams = {"FRONT-1": FakeStream(w=640, h=360), "REAR-LEFT": FakeStream(), "REAR-RIGHT": FakeStream()}
        app.front_sel = A.video.FrontSelector()
        W_, H_ = dv.cv.winfo_width(), dv.cv.winfo_height()
        fills = {}
        for sc in ("fill", "fit"):
            settings.put("video_scale", sc)
            pump(0.5)
            fills[sc] = dv.pic
        settings.put("video_scale", "fill")
        f_ok = fills["fill"][0] <= 0 and fills["fill"][1] <= 0 and fills["fill"][2] >= W_ - 1 and \
            fills["fill"][3] >= H_ - 1 and abs(fills["fill"][0] * 2 + fills["fill"][2] - W_) < 2
        t_ok = fills["fit"][2] <= W_ + 1 and fills["fit"][3] <= H_ + 1
        outside = {}
        for lang in ("el", "en"):
            i18n.set_lang(lang, save=False)
            pump(0.4)
            bad = [(t, s_) for t, bb, box, s_ in dv.renderer.text_boxes
                   if box and (bb[0] < box[0] - 3 or bb[2] > box[2] + 3 or bb[1] < box[1] - 3 or bb[3] > box[3] + 3)]
            outside[lang] = bad
        i18n.set_lang("en", save=False)
        pump(0.3)
        sz = {t: max([o["size"] for o in dv.items(t) if o["k"] == "text"] or [0]) for t in ("speed", "mode", "comm",
                                                                                           "wp", "steer")}
        report("I10 full-screen camera (fill = cover cropped evenly / fit = whole picture), every text inside its box "
               "in Greek and English, speed > mode > communication > other texts",
               f_ok and t_ok and not outside["el"] and not outside["en"] and
               sz["speed"] > sz["mode"] > sz["comm"] > max(sz["wp"], sz["steer"]) and not errors,
               "window %dx%d, fill picture %s, fit picture %s; texts outside their box: el %s, en %s; sizes %s" % (
                   W_, H_, [round(v) for v in fills["fill"]], [round(v) for v in fills["fit"]], outside["el"][:3],
                   outside["en"][:3], sz))

        # I11 MASTER lamps: on with an alarm of that level; one click acknowledges; nothing sent to the boat
        sent_frames = []
        orig_send = app.link.send
        app.link.send = lambda *a, **k: (sent_frames.append(a[0]), orig_send(*a, **k))[1]
        orig_t = app.on_telem
        app.on_telem = lambda d_: orig_t(dict(d_, alarm_bits=d_["alarm_bits"] | 0x001 | 0x002))  # compass + GPS
        pump(0.6)
        on_w, on_c = app.lamps.on(3), app.lamps.on(2)
        lamp_hit = [r for r, act in dv.hits if act == "ack"]
        ev = type("E", (), {"x": int((lamp_hit[0][0] + lamp_hit[0][2]) / 2), "y": int((lamp_hit[0][1] +
                                                                                       lamp_hit[0][3]) / 2)})()
        dv.on_press(ev)
        pump(0.3)
        off_w, off_c = app.lamps.on(3), app.lamps.on(2)
        app.on_telem = lambda d_: orig_t(dict(d_, alarm_bits=d_["alarm_bits"] | 0x001 | 0x002 | 0x100))
        pump(0.5)
        again = app.lamps.on(3) and not app.lamps.on(2)
        app.on_telem = orig_t
        hb = [f for f in sent_frames if f not in (proto.T_HEARTBEAT,)]
        report("I11 MASTER WARNING / CAUTION: lit by a new alarm of that level, one click acknowledges both (this PC "
               "only), a newer alarm lights it again; nothing sent to the boat",
               on_w and on_c and not off_w and not off_c and again and not hb,
               "compass lost + GPS lost: warning %s caution %s; after the click %s / %s; joystick fault next: warning "
               "%s; frames sent by the click %s" % (on_w, on_c, off_w, off_c, again, hb))
        app.link.send = orig_send
        pump(0.3)

        # I12 night + declutter, remembered
        dv.action("night")
        pump(0.3)
        night_col = (dv.texts("mode") and [o["fill"] for o in dv.items("mode") if o["k"] == "text"][0])
        night_dark = dv.last_frame is not None
        levels = {}
        for _ in range(3):
            dv.action("declutter")
            pump(0.3)
            levels[settings.get("declutter")] = sorted({o["tag"] for o in dv.ops if o.get("tag")})
        mini = levels.get("MINIMAL", [])
        mini_ok = "speed" in mini and "hdg" in mini and "mode" in mini and "gear" in mini and \
            "comm" not in mini and "wp" not in mini and "mirror_l" not in mini and "thr" not in mini
        normal_ok = "rot" not in levels.get("NORMAL", []) and "wp" in levels.get("NORMAL", [])
        settings.reload()
        remembered = settings.get("night") is True and settings.get("declutter") == "FULL"
        dv.action("night")
        pump(0.2)
        report("I12 night mode (dim red) and declutter FULL / NORMAL / MINIMAL (MINIMAL = speed, heading, mode, "
               "gear, alarms), remembered",
               night_col == hud.palette(night=True)["main"] and night_dark and mini_ok and normal_ok and remembered,
               "night mode colour %s; MINIMAL shows %s; NORMAL hides the rate of turn %s; remembered %s" % (
                   night_col, [t for t in mini if not t.startswith("ctl_")], normal_ok, remembered))

        # I13 edit layout: move / resize / hide, saved, reset; no move outside the edit mode
        r0 = dv.layout.rects["speed"]
        x0, y0, x1, y1 = dv.layout.px("speed", W_, H_)
        E = lambda x, y: type("E", (), {"x": int(x), "y": int(y), "x_root": 0, "y_root": 0})()   # noqa: E731
        dv.on_press(E(x0 + 30, y0 + 30))
        dv.on_motion(E(x0 - 70, y0 + 10))
        dv.on_release(E(x0 - 70, y0 + 10))
        not_moved = dv.layout.rects["speed"] == r0
        dv.action("edit")
        dv.on_press(E(x0 + 30, y0 + 30))
        dv.on_motion(E(x0 - 70, y0 + 10))
        dv.on_release(E(x0 - 70, y0 + 10))
        moved = dv.layout.rects["speed"]
        x0, y0, x1, y1 = dv.layout.px("speed", W_, H_)
        dv.on_press(E(x1 - 4, y1 - 4))
        dv.on_motion(E(x1 + 40, y1 + 30))
        dv.on_release(E(x1 + 40, y1 + 30))
        resized = dv.layout.rects["speed"]
        dv.on_press(E(x1 + 40 - 8, y0 + 6))                # the x / + corner: hide
        hidden = "speed" in dv.layout.hidden
        dv.action("edit")
        pump(0.3)
        gone = not dv.has("speed")
        saved = A.json.load(open(os.environ["TRYGONS_HUD_LAYOUT"], encoding="utf-8"))
        dv.action("edit")
        dv.action("edit_reset")
        dv.action("edit")
        pump(0.3)
        reset = dv.layout.rects["speed"] == hud.DEFAULT_LAYOUT["speed"] and not dv.layout.hidden and dv.has("speed")
        report("I13 edit-layout mode: drag moves, the corner resizes, x hides; saved to the layout file; RESET; "
               "outside the edit mode nothing moves",
               not_moved and moved[0] < r0[0] and resized[2] > moved[2] and hidden and gone and
               "speed" in saved.get("hidden", []) and reset,
               "normal mode drag moved it %s; edit: x %.3f -> %.3f, width %.3f -> %.3f, hidden %s (gone from the "
               "HUD %s), file %s; after RESET default %s" % (not not_moved, r0[0], moved[0], moved[2], resized[2],
                                                             hidden, gone, saved.get("hidden"), reset))

        # I14 every HUD button and key: nothing to the boat
        sent_frames = []
        app.link.send = lambda *a, **k: (sent_frames.append(a[0]), orig_send(*a, **k))[1]
        for act in ("night", "night", "declutter", "declutter", "declutter", "scale", "scale", "lang", "lang", "ack",
                    "edit", "edit", "ar_all", "ar_all", "vis_overlay", "vis_overlay"):
            dv.action(act)
        for key in ("n", "n", "d", "d", "d", "v", "v", "a", "o", "o", "g", "g"):
            dv.w.event_generate("<KeyPress-%s>" % key)
        pump(0.5)
        app.link.send = orig_send
        hb = [f for f in sent_frames if f != proto.T_HEARTBEAT]
        report("I14 HUD buttons and keys (night, declutter, fill / fit, language, acknowledge, layout, AR, overlay) "
               "never send a frame to the boat", not hb and not errors,
               "frames sent: %s; errors %d" % ([hex(f) for f in hb], len(errors)))
        i18n.set_lang("en", save=False)

        # I15 AR maths
        import hudar
        cal = {"height_m": 2.0, "tilt_down_deg": 3.0, "hfov_deg": 80.0, "vfov_deg": 45.0, "yaw_deg": 0.0}
        c0 = hudar.Cam(cal)
        fy = 0.5 / A.math.tan(A.math.radians(22.5))
        (ul, vl), (ur, vr) = c0.horizon()
        hz_ok = abs(vl - (0.5 - fy * A.math.tan(A.math.radians(3.0)))) < 1e-6 and abs(vl - vr) < 1e-9
        c10 = hudar.Cam(cal, roll=10.0)
        (ul2, vl2), (ur2, vr2) = c10.horizon()
        heel_ok = vr2 < vl2                                     # starboard down: the horizon rises to the right
        p20 = c0.project(20.0, 0.0)
        water_ok = p20 is not None and abs(p20[0] - 0.5) < 1e-9 and p20[1] > vl
        behind = c0.project(-10.0, 0.0) is None
        _, _, inv = c0.project_bearing(30.0)
        _, _, outv = c0.project_bearing(60.0)
        straight = hudar.path_points(10.0, 0.0, seconds=30.0)
        turn = hudar.path_points(10.0, 3.0, seconds=30.0)
        path_ok = abs(straight[-1][1] - 30 * 10 * 0.514444) < 0.5 and abs(straight[-1][2]) < 1e-6 and turn[-1][2] > 20
        rp = hudar.route_points(37.8, 23.6, [(37.8 + 1000 / 111320.0, 23.6)], step_m=25.0)
        route_ok = len(rp) == 41 and abs(rp[-1][2] - 1000.0) < 1e-6
        cal_ok = hudar.calibration({}) is None and hudar.calibration({"vision": {"camera": dict(cal, hfov_deg=0)}}) is \
            None and hudar.calibration({"vision": {"camera": cal}}) == cal
        report("I15 AR maths: horizon from the tilt, heel turns it, water points below the horizon, field of view, "
               "predicted path, route points, calibration checked",
               hz_ok and heel_ok and water_ok and behind and inv and not outv and path_ok and route_ok and cal_ok,
               "horizon v %.3f (level %s); heel 10 deg: left %.3f right %.3f; 20 m ahead v %.3f; behind -> None %s; "
               "30 deg in view %s, 60 deg %s; 30 s at 10 kn %.1f m straight, turning 3 deg/s %.1f m right; route "
               "points %d; calibration checks %s" % (vl, hz_ok, vl2, vr2, p20[1] if p20 else -1, behind, inv, outv,
                                                       straight[-1][1], turn[-1][2], len(rp), cal_ok))

        # I16 AR on the HUD
        class FakeCharts:
            def __bool__(s):
                return True

            def depth(s, la, lo):
                return 1.5 if la > app.t["est_lat"] + 0.0002 else 40.0         # shallow north of the boat

            def area_at(s, la, lo):
                return True

            attribution = lambda s: []                         # noqa: E731
            areas = []
        real_charts = app.charts
        app.charts = FakeCharts()
        dv.chartview = driver.mapview.ChartView(app.charts)
        settings.put("camera", None)
        orig_t = app.on_telem

        def moving(d_):
            la, lo = d_["est_lat"], d_["est_lon"]
            orig_t(dict(d_, est_sog=7.0, est_cog=4.0, heading=0.0, yaw_rate=2.0, att_roll=5.0, att_pitch=0.0,
                        pos_source=1, wp_set=True, wp_lat=la + 0.006, wp_lon=lo + 0.001, prog_n=2, auto_wp=0,
                        mode=proto.MODE_HDG_LOCK, mode_name="HDG_LOCK"))
        app.on_telem = moving
        app.vstreams = {"FRONT-1": FakeStream(w=640, h=360)}
        app.front_sel = A.video.FrontSelector()
        pump(0.6)
        nocal = dv.has("ar_nocal") and not dv.has("ar_horizon")
        la, lo = app.t["est_lat"], app.t["est_lon"]
        app.gw_status = {"vision": {"state": "ACTIVE", "fps": 2.0, "camera": cal, "obstacles": [], "mask": {
            "w": 64, "h": 48, "b64": base64.b64encode(packed).decode()}},
                         "ais": {"list": [{"mmsi": 5, "name": "AHEAD", "lat": la + 0.004, "lon": lo, "sog": 9.0,
                                           "cog": 180.0, "dist_nm": 0.24},
                                          {"mmsi": 6, "name": "ABEAM", "lat": la, "lon": lo + 0.006, "sog": 9.0,
                                           "cog": 270.0, "dist_nm": 0.28}]},
                         "avoid": {"threats": [{"mmsi": 6, "name": "ABEAM", "cpa_nm": 0.1, "tcpa_min": 1.5,
                                                "dist_nm": 0.28, "bearing": 90.0, "level": "danger"}]}}
        pump(1.4)
        tags = {"horizon": "ar_horizon", "velocity": "ar_velocity", "waypoint": "ar_wp", "ribbon": "ar_ribbon",
                "ais": "ar_ais", "path": "ar_path", "shallow": "ar_shallow"}
        shown = {k: dv.has(t) for k, t in tags.items()}
        edge = dv.has("ar_ais_edge")
        roll = dv.has("ar_roll")
        off_ok = {}
        for k, t in tags.items():
            settings.put("ar", dict({kk: True for kk in tags}, **{k: False}))
            pump(0.35)
            off_ok[k] = not dv.has(t) and all(dv.has(tt) for kk, tt in tags.items() if kk != k and kk != "shallow")
        settings.put("ar", {kk: True for kk in tags})
        vis_kept = dv.has("vmask")
        app.on_telem = orig_t
        app.gw_status = None
        app.charts = real_charts
        dv.chartview = driver.mapview.ChartView(app.charts)
        app.vstreams = {}
        report("I16 AR overlays: 'calibration unknown' without a calibration; horizon + heel, velocity vector, "
               "waypoint, route ribbon, AIS box + edge arrow, predicted path, shallow tint; each one off on its own; "
               "the vision overlay stays",
               nocal and all(shown.values()) and edge and roll and all(off_ok.values()) and vis_kept and not errors,
               "no calibration -> notice %s; shown %s, AIS edge arrow %s, heel scale %s; each off alone %s; vision "
               "mask still drawn %s; errors %d" % (nocal, shown, edge, roll, off_ok, vis_kept, len(errors)))

        # I17 the MAP window
        sent_frames = []
        app.link.send = lambda *a, **k: (sent_frames.append(a[0]), orig_send(*a, **k))[1]
        app.prog["wps"] = [{"lat": app.t["est_lat"] + 0.002, "lon": app.t["est_lon"] + 0.002, "kn": 6, "radius": 25,
                            "action": 0},
                           {"lat": app.t["est_lat"] + 0.004, "lon": app.t["est_lon"], "kn": 6, "radius": 25, "action": 0}]
        la, lo = app.t["est_lat"], app.t["est_lon"]
        app.gw_status = {"ais": {"list": ships}, "avoid": {"state": "PROPOSE", "threats": threats, "route": [
            (la, lo), (la + 0.003, lo + 0.003)], "shallow": {"lat": la + 0.001, "lon": lo, "in_s": 40, "depth": 2.1,
                                                            "limit": 3.0}}}
        orig_t = app.on_telem
        app.on_telem = lambda d_: orig_t(dict(d_, pos_source=1, heading=75.0, est_sog=5.0, est_cog=80.0, wp_set=True,
                                              wp_lat=la + 0.002, wp_lon=lo + 0.002, unc_m=40.0))
        app.open_map()                                  # screen 2's Map tab (no separate window)
        mw = app.mapwin
        pump(1.2)
        tags = {t: len(mw.cv.find_withtag(t)) for t in ("ring", "track", "prog", "active_wp", "unc", "boat", "ais",
                                                        "route", "shallow", "warning", "north", "note")}
        rows = mw.rows
        x_n = mw.cv.coords(mw.cv.find_withtag("north")[0])
        mw.toggle_orient()
        pump(0.6)
        x_c = mw.cv.coords(mw.cv.find_withtag("north")[0])
        rotated = mw.view and mw.view[2] == 75.0 and abs(x_c[0] - x_n[0]) > 5
        m0 = mw.m
        mw.zoom(0.6)
        zoomed = abs(mw.m - m0 * 0.6) < 1e-6
        c0 = mw.center
        E = lambda x, y: type("E", (), {"x": x, "y": y})()            # noqa: E731
        mw.on_press(E(400, 300))
        mw.on_motion(E(300, 250))
        panned = not mw.follow and mw.center != c0
        mw.toggle_follow()
        pump(0.6)
        back = mw.follow and abs(mw.center[0] - app.t["est_lat"]) < 1e-4
        themes = []
        for _ in range(3):
            mw.next_theme()
            themes.append((mw.theme, mw.cv.cget("bg")))
        settings.reload()
        remembered = (settings.get("map") or {}).get("course_up") is True and abs(
            (settings.get("map") or {}).get("m_per_px", 0) - mw.m) < 1e-3
        moved = mw.embedded and str(mw.w.winfo_toplevel()) == str(root)
        mons = A.monitors.list_monitors(root)
        on_mon = moved
        hb = [f for f in sent_frames if f != proto.T_HEARTBEAT]
        app.link.send = orig_send
        app.on_telem = orig_t
        mw.toggle_orient()
        report("I17 MAP in screen 2: chart layers, AIS list, boat, track, route, waypoint, uncertainty, range rings, "
               "warnings; follow / pan / zoom / course-up / themes; remembered; view only; part of screen 2",
               all(v > 0 for k, v in tags.items() if k != "track") and len(rows) == 3 and rotated and zoomed and
               panned and back and len({t for t, _b in themes}) == 3 and remembered and moved and on_mon and not hb
               and not errors,
               "items %s; AIS list %d rows; course-up turns the map (north arrow moved %s); zoom %s; pan %s, follow "
               "again %s; themes %s; remembered %s; monitors %d, inside screen 2 %s; frames sent %s" % (
                   tags, len(rows), rotated, zoomed, panned, back, themes, remembered, len(mons), on_mon, hb))
        app.gw_status = None
        pump(0.3)

        # I18 sound callouts in the real app (silent player: TRYGONS_SOUND=off)
        snd = app.sound
        sent_frames = []
        app.link.send = lambda *a, **k: (sent_frames.append(a[0]), orig_send(*a, **k))[1]
        orig_t = app.on_telem
        fake = {"mode": proto.MODE_HDG_LOCK, "mode_name": "HDG_LOCK"}
        app.on_telem = lambda d_: orig_t(dict(d_, **fake))
        pump(0.8)
        fake.update(mode=proto.MODE_STANDBY, mode_name="STANDBY")
        pump(0.8)
        played = lambda code: [h for h in snd.sched.history if h[1] == code and h[3] == "played"]   # noqa: E731
        modes_ok = bool(played("mode.hdg_lock")) and bool(played("mode.standby"))
        fake["alarm_bits"] = 0x001                                  # compass lost: a WARNING
        app.on_telem = lambda d_: orig_t(dict(d_, **fake))
        pump(9.0)
        n_warn = len(played("alarm.compass_lost"))
        dv.action("ack")
        pump(8.0)
        n_after = len(played("alarm.compass_lost"))
        fake["alarm_bits"] = 0
        app.on_telem = orig_t
        pump(0.5)
        hb = [f for f in sent_frames if f != proto.T_HEARTBEAT]
        app.link.send = orig_send
        app.cmd(proto.T_SEA_STATE, proto.p_sea_state(app.t["sea_state"]), "sea state %d" % app.t["sea_state"])
        pump(0.3)
        dv.answer(True)
        conf = pump(3.0, lambda: bool(played("fb.confirm")))
        report("I18 sound in the ground station: mode callouts from the live telemetry, a WARNING repeats until the "
               "driver acknowledges the MASTER lamp, a command answer gives the confirm sound; the sound sends nothing",
               modes_ok and n_warn >= 2 and n_after == n_warn and conf and not hb and not errors,
               "heading lock / standby said %s; compass warning played %d times in 9 s, after ACK %d more; confirm "
               "sound %s; frames sent by the sound %s" % (modes_ok, n_warn, n_after - n_warn, conf, hb))

        # I9 BENCH TEST tab (USB-style link): enter, live sensors, a steering pulse, a button, leave
        app.bench(A.proto.BENCH_ENTER, 0, "bench test: enter")
        in_bench = pump(3.0, lambda: app.t and app.t.get("mode") == A.proto.MODE_BENCH and app.bench_d is not None)
        sens = app.bench_sens.cget("text")
        app.bench_ms.set(200)
        app.bench(A.proto.BENCH_RIGHT, app.bench_ms.get(), "pulse RIGHT 200 ms")
        pulsed = pump(3.0, lambda: app.bench_d and app.bench_d["pulses"] >= 1)
        app.link.send(A.proto.T_SIM_BUTTON, A.proto.p_sim_button(5))          # HDG LOCK on the panel
        seen = pump(3.0, lambda: app.bench_d and app.bench_d["button_counts"][5] >= 1)
        hdg_lbl = [lb for b, lb in app.bench_btn if b == 5][0]
        btn_green = hdg_lbl.cget("bg") == "#7de07d"
        servo = app.bench_servo.cget("text")
        state = app.bench_state.cget("text")
        still = app.t.get("mode") == A.proto.MODE_BENCH
        app.bench(A.proto.BENCH_LEAVE, 0, "bench test: leave")
        left = pump(3.0, lambda: app.t.get("mode") == A.proto.MODE_STANDBY)
        pump(0.5)
        off_txt = app.bench_state.cget("text")
        report("I9 BENCH TEST tab: enter, every sensor live, a steering pulse, a panel button shown (no mode change), leave",
               in_bench and "IMU" in sens and "GPS" in sens and "SD card" in sens and pulsed and seen and btn_green and
               still and "us" in servo and "BENCH TEST" in state and left and "not in the bench test" in off_txt and
               not errors,
               "in bench %s; sensors \"%s ...\"; pulse counted %s; HDG LOCK shown %s (green %s), mode still BENCH %s; "
               "servo \"%s\"; state \"%s\"; left %s -> \"%s\"" % (in_bench, sens[:60].replace("\n", " | "), pulsed, seen,
                                                                  btn_green, still, servo, state[:50], left, off_txt))

        # I6 window places remembered; F12 swaps the screens
        g_cap, g_drv = root.geometry(), dv.w.geometry()
        app.save_layout()
        with open(A.LAYOUT, encoding="utf-8") as f:
            lay = A.json.load(f)
        app.swap_screens()
        pump(1.0)
        swapped = root.geometry().split("+")[1:] == g_drv.split("+")[1:] and \
            dv.w.geometry().split("+")[1:] == g_cap.split("+")[1:]
        app.swap_screens()
        pump(1.5)
        app.quit()
        root = tk.Tk()
        app2 = A.App(root)
        pump(1.5)
        restored = app2.driver is not None and root.geometry().split("+")[1:] == g_cap.split("+")[1:] and \
            app2.driver.w.geometry().split("+")[1:] == g_drv.split("+")[1:] and app2.mapwin is not None and \
            app2.mapwin.embedded
        report("I6 window places remembered (monitors) and restored: screen 2 and screen 1 (the map is inside "
               "screen 2); F12 swaps them",
               lay.get("driver_open") and "map" not in lay and swapped and restored,
               "saved %s; after F12 screen 2 at %s, screen 1 at %s (swapped %s); after a restart: screen 2 %s, "
               "screen 1 %s, map inside screen 2 %s" % (lay, root.geometry(),
                                                        app2.driver.w.geometry() if app2.driver else None, swapped,
                                                        root.geometry(), app2.driver.w.geometry() if app2.driver
                                                        else "NOT OPEN", app2.mapwin is not None))
        app2.quit()
        root = None
    finally:
        if root:
            try:
                root.destroy()
            except tk.TclError:
                pass
        sim.terminate()
        if saved_layout is not None:
            with open(A.LAYOUT, "wb") as f:
                f.write(saved_layout)
        elif os.path.exists(A.LAYOUT):
            os.remove(A.LAYOUT)
    bad = results.count(False)
    print("DRIVER SCREEN TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
