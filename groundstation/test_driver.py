"""Two-monitor ground station tests (section I): the DRIVER screen.

Starts  sim/autopilot_sim server <port> 1  and the real ground station
(app.App) on it, opens the driver screen and checks:
  I1 HUD: mode, speed, compass tape centred on the heading, throttle bar with
     the power limit, slamming graph, mini map - from the live telemetry
  I2 a driving-affecting captain action waits for the driver: no answer ->
     rejected (nothing sent); ACCEPT -> sent; STANDBY never waits
  I3 slamming bar graph: centred on the running average, limit line; absolute
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
os.environ.setdefault("TRYGONS_GS_SETTINGS", os.path.join(__import__("tempfile").gettempdir(), "trygons_gs_test.json"))
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

        pump(10, lambda: app.t is not None and app.link and app.link.connected)
        root.state("normal")
        root.geometry("1200x800+40+40")
        app.open_driver()
        dv = app.driver
        dv.w.geometry("1600x900+100+60")
        pump(2.0)
        d = app.t

        # I1 the HUD from live telemetry
        mode_items = items_with_tag(dv.status, "mode")
        mode_txt = dv.status.itemcget(mode_items[0], "text") if mode_items else None
        tw = dv.tape.winfo_width()
        hdg = dv.tape.coords(items_with_tag(dv.tape, "hdg")[0]) if items_with_tag(dv.tape, "hdg") else []
        plim = dv.thr.coords(items_with_tag(dv.thr, "plim")[0]) if items_with_tag(dv.thr, "plim") else []
        h_thr = max(50, dv.thr.winfo_height()) - 40
        plim_ok = bool(plim) and abs(plim[1] - (20 + h_thr * (1 - d["power_fwd"] / 100.0))) < 2
        report("I1 driver HUD: mode, speed, compass tape, throttle bar with the power limit, slamming graph, map",
               mode_txt == d["mode_name"].replace("_", " ") and bool(hdg) and abs(hdg[4] - tw / 2) < 1 and plim_ok and
               dv.speed.cget("text").endswith("kn") and items_with_tag(dv.slam, "limit") != () and not errors,
               "mode \"%s\"; tape heading mark at x=%s of %d (centre); power limit line at %.0f %% %s; speed \"%s\"; "
               "slamming graph %s; errors %d" % (mode_txt, hdg[4] if hdg else None, tw, d["power_fwd"],
                                                 "OK" if plim_ok else "WRONG", dv.speed.cget("text"),
                                                 "drawn" if items_with_tag(dv.slam, "limit") else "MISSING", len(errors)))

        # I2 captain actions that change the driving wait for the driver
        app.confirm_timeout_s = 3.0
        sea0 = app.t["sea_state"]
        new_sea = 3 if sea0 != 3 else 2
        n0 = len(app.pending)
        app.cmd(proto.T_SEA_STATE, proto.p_sea_state(new_sea), "sea state %d" % new_sea)
        waiting = dv.question is not None and len(app.pending) == n0
        pump(4.0)
        rejected = dv.question is None and dv.answers and "timeout" in dv.answers[-1][1] and len(app.pending) == n0
        pump(1.0)
        unchanged = app.t["sea_state"] == sea0
        app.cmd(proto.T_SEA_STATE, proto.p_sea_state(new_sea), "sea state %d" % new_sea)
        pump(0.3)
        dv.answer(True)
        changed = pump(5.0, lambda: app.t["sea_state"] == new_sea)
        app.cmd(proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY), "mode STANDBY")
        sb_direct = dv.question is None
        report("I2 captain action needs the driver's ACCEPT (no answer in time = rejected); STANDBY never waits",
               waiting and rejected and unchanged and changed and sb_direct,
               "sea state request waiting on the driver screen %s; after the timeout: %s, nothing sent %s; "
               "ACCEPT -> boat sea state %s; STANDBY sent at once %s" % (
                   waiting, dv.answers[0][1] if dv.answers else "-", unchanged, app.t["sea_state"], sb_direct))

        # I3 slamming bar graph: around the running average, limit line, absolute
        bars, ly, base = driver.slam_geom([1.0, 1.5, 2.0, 3.5], 1.5, 3.0, 200)
        abars, aly, abase = driver.slam_geom([1.0, 1.5, 2.0, 3.5], 1.5, 3.0, 200, absolute=True)
        avg_bar = [b for b in bars if b[0] == 1][0]
        report("I3 slamming graph: bars around the running average, limit line, colours; absolute mode",
               base == 100 and abs(avg_bar[2] - 100) < 0.01 and bars[0][2] > 100 and bars[2][2] < 100 and
               ly < bars[2][2] and bars[3][3] == driver.RED and bars[2][3] == driver.GRN and abase == 200 and
               abars[0][2] < 200 and abs(aly - (200 - 3.0 / 3.675 * 200)) < 1,
               "centre line y=%d (average 1.5 g: bar of length %.1f); 1.0 g below, 2.0 g above; limit 3 g at y=%.0f; "
               "3.5 g red, 2.0 g green; absolute: base y=%d, limit y=%.0f" % (
                   base, abs(avg_bar[2] - avg_bar[1]), ly, abase, aly))

        # I4 mirrors: rear pictures flipped left <-> right
        fs = FakeStream()
        fl = driver.flip_ppm(fs.data)
        hdr = len(b"P6\n64 36\n255\n")
        left_px, right_px = fl[hdr:hdr + 3], fl[hdr + 63 * 3:hdr + 64 * 3]
        report("I4 rear cameras shown as mirrors (flipped left <-> right)",
               fs.data[hdr:hdr + 3] == bytes((200, 40, 40)) and left_px == b"\x10\x10\x10" and
               right_px == bytes((200, 40, 40)) and len(fl) == len(fs.data),
               "first pixel before %s, after %s; last pixel after %s" % (
                   tuple(fs.data[hdr:hdr + 3]), tuple(left_px), tuple(right_px)))

        # I5 VIDEO FROZEN within 1 s; FRONT-1 lost -> FRONT-2; remote without front camera -> warning
        app.vstreams = {"FRONT-1": FakeStream(), "FRONT-2": FakeStream(colour=(40, 40, 200)),
                        "REAR-LEFT": FakeStream(), "REAR-RIGHT": FakeStream()}
        app.front_sel = A.video.FrontSelector()
        pump(0.5)
        ok_live = not items_with_tag(dv.front, "frozen")
        app.vstreams["REAR-LEFT"].is_frozen = True
        t0 = time.time()
        fr = pump(2.0, lambda: bool(items_with_tag(dv.mirror_l, "frozen")))
        t_fr = time.time() - t0
        app.vstreams["FRONT-1"].is_frozen = True
        sw = pump(3.0, lambda: app.front_sel.use == "FRONT-2")
        notice = app.front_sel.notice
        app.vstreams["FRONT-2"].is_frozen = True
        orig = app.on_telem                                 # the boat says REMOTE MANUAL from now on
        app.on_telem = lambda d: orig(dict(d, mode=proto.MODE_REMOTE))
        warn = pump(1.0, lambda: bool(items_with_tag(dv.front, "nofront")))
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
        dv.draw_status(app.t)
        hud = [dv.status.itemcget(i, "text") for i in items_with_tag(dv.status, "collision")]
        asked = dv.question is not None and "COLLISION AVOIDANCE" in dv.qtext.cget("text")
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
               "SVXY2" in detail and hud and "BLUE STAR" in hud[0] and asked and acc and asked2 and rej and closed,
               "rows %s; first %s %s; details \"%s\"; HUD \"%s\"; proposal asked %s -> ACCEPT sent %s; next one "
               "REJECT sent %s; withdrawn at the gateway -> closed without an answer %s" % (
                   [app.ais_tree.item(r)["text"] for r in rows], first.get("text"), first.get("values"),
                   detail.replace("\n", " / ")[:120], hud[0] if hud else "MISSING", asked, acc, rej, closed))
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
        dv.front.delete("all")
        dv.draw_vision(dv.front, 640, 360, 640, 360)
        n_mask = len(items_with_tag(dv.front, "vmask"))
        box_txt = [dv.front.itemcget(i, "text") for i in items_with_tag(dv.front, "vbox")
                   if dv.front.type(i) == "text"]
        dv.toggle_overlay()
        dv.front.delete("all")
        dv.draw_vision(dv.front, 640, 360, 640, 360)
        n_off = len(items_with_tag(dv.front, "vmask"))
        dv.toggle_overlay()
        dv.draw_status(app.t)
        hud = " ".join(dv.status.itemcget(i, "text") for i in items_with_tag(dv.status, "joy"))
        clear_shown = dv.vis_clear_btn.winfo_manager() == "pack"
        dv.vis_clear_btn.invoke()
        dv.draw_map(app.t)
        n_sector = len(items_with_tag(dv.map, "vision"))
        app.drv_vision()
        asked = dv.question is not None and "OBSTACLE DETECTION" in dv.qtext.cget("text")
        dv.answer(True)
        acked = {"t": "vision_ack", "id": 5} in sent
        report("I8 obstacle detection on the driver screen: mask overlay (toggle), box with bearing + rough distance, "
               "status, CLEAR SLOWDOWN, map sector, HDG LOCK slow-down proposal",
               n_mask > 0 and n_off == 0 and box_txt and "092" in box_txt[0] and "rough" in box_txt[0] and
               "VISION ACTIVE" in hud and "HELD" in hud and clear_shown and {"t": "vision_clear"} in sent and
               n_sector > 0 and asked and acked and not errors,
               "overlay %d items (off: %d); box \"%s\"; HUD \"%s\"; clear button %s, sent %s; map sector items %d; "
               "proposal asked %s, acknowledged %s" % (n_mask, n_off, box_txt[0] if box_txt else "MISSING",
                                                       hud[-90:], clear_shown, sent[:1], n_sector, asked, acked))
        app.gw_status = None
        del app.vision_send

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
            app2.driver.w.geometry().split("+")[1:] == g_drv.split("+")[1:]
        report("I6 both window places remembered (monitors) and restored; F12 swaps captain and driver screens",
               lay.get("driver_open") and swapped and restored,
               "saved %s; after F12 captain at %s, driver at %s (swapped %s); after a restart: captain %s, driver %s" % (
                   lay, root.geometry(), app2.driver.w.geometry() if app2.driver else None, swapped,
                   root.geometry(), app2.driver.w.geometry() if app2.driver else "NOT OPEN"))
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
