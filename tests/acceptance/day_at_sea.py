"""Part 4.2, part B: "a day at sea" through the WHOLE system in real time -
simulator (the boat) + gateway (cameras through MediaMTX, recording, AIS
traffic, collision avoidance, shallow-water check, vision, channels, sync to
the shore) + ground stations (captain, driver screen, shore remote joystick).

    python tests/acceptance/day_at_sea.py [--seed 1]

The physical faults the simulator server can't be told from outside (sea state
4 / slamming, GPS spoof, compass glitch, power-up interlock, TRAINING, power
raises) run in part A: test_results/storm.exe --dayatsea (fast, simulated time).
Here: HEADING LOCK with a collision proposal, an AUTO program past a shallow
area, an AIS crossing ship, a kayak without AIS (vision STAGE 3 off / on), a
vision blip, GPS lost for 4 minutes, 4G -> RF -> STARLINK -> 4G, remote driving
with a 2 s link dropout, a joystick takeover with SAVE, arrival, and the sync of
all logs + saved events to the shore with checksums.
-> test_results/day_at_sea_B.md (timeline: what happened, what the system did,
expected, PASS / FAIL, with the gateway's log time)
"""
import argparse
import glob
import hashlib
import os
import random
import shutil
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "test_results")
sys.path.insert(0, os.path.join(REPO, "gateway"))
os.chdir(os.path.join(REPO, "gateway"))
import test_gateway as TG  # noqa: E402
from common import proto  # noqa: E402
from simais import AisSim  # noqa: E402
from simcam import SimCamera  # noqa: E402
from syncrecv import SyncReceiver  # noqa: E402
from shorejoy import RemoteSender  # noqa: E402

ROWS = []
T0 = time.time()


def row(g, what, did, exp, ok):
    t = time.time() - T0
    stamp = g.clock.stamp()[11:19] if g else "-"
    ROWS.append((t, stamp, what, did, exp, ok))
    print("| %6.0f s | %s | %s | %s | %s | %s |" % (t, stamp, what, did, exp, "PASS" if ok else "FAIL"), flush=True)


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    random.seed(a.seed)
    TG.setup_logging(40)
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="acc_day_")
    sim = gw = asim = recv = snd = None
    cams = []
    beat = None
    try:
        sim = TG.Sim(1.0)
        root = os.path.join(tmp, "ssd")
        shore = os.path.join(tmp, "shore")
        os.makedirs(root)
        charts = os.path.join(tmp, "charts")
        TG.sim_chart(charts, n_lat=400, n_lon=400)
        recv = SyncReceiver(shore, {"trygons-c2": TG.SYNC_KEY}, port=TG.free_port(), listen="127.0.0.1").start()
        rtsp, api, web, wu, aport = (TG.free_port() for _ in range(5))
        ccfg = {"enabled": True, "mediamtx_bin": TG.MTX_BIN, "mediamtx_conf": os.path.join(tmp, "mtx.yml"),
                "rtsp_port": rtsp, "api_port": api, "webrtc_port": web, "webrtc_udp_port": wu,
                "list": [{"name": n, "role": r, "url_main": "rtsp://127.0.0.1:%d/simcam_%d" % (rtsp, i),
                          "url_sub": "rtsp://127.0.0.1:%d/simcam_%d" % (rtsp, i)}
                         for i, (n, r) in enumerate(zip(TG.CAM_NAMES, TG.CAM_ROLES))],
                "extra_paths": ["simcam_%d" % i for i in range(4)], "mediamtx_log": os.path.join(tmp, "mtx.log")}
        cfg = TG.gw_cfg_b(sim, root, charts_dir=charts, cameras=ccfg, avoid=TG.CA_AVOID,
                          stations={"driver_timeout_s": 6.0},
                          recording={"enabled": True, "ring_min": 5, "segment_s": 20, "post_s": 15, "stall_s": 60.0},
                          vision={"enabled": True, "source": "fake", "allow_inject": True, "max_fps": 4.0,
                                  "stage2": True, "stage3": False, "clear_s": 4.0},
                          ais={"enabled": True, "source": "tcp:127.0.0.1:%d" % aport, "silent_s": 600.0},
                          sync={"enabled": True, "shore": "127.0.0.1:%d" % recv.port, "settle_s": 1.0, "scan_s": 2.0},
                          channels={"driver": "sim", "check_s": 0.5, "stable_s": 15.0, "fail_s": 5.0,
                                    "starlink_status": False})
        cfg["secrets"] = {"sync_key": TG.SYNC_KEY}
        asim = AisSim(aport, seed=a.seed, noise_m=5.0, start_offset=0.2).start()
        gw = TG.GwRunner(cfg)
        g = gw.gw
        time.sleep(1.5)
        cams = [SimCamera("rtsp://127.0.0.1:%d/simcam_%d" % (rtsp, i)).start() for i in range(4)]
        sp = cfg["stations"]["port"]
        link, w = sim.station()                                   # the boat: panel, joystick, CONFIRM
        alice = TG.Client(sp, "alice", TG.PW["alice"], name="captain (shore)")
        alice.pump(1.0)
        alice.cmd(proto.T_HELM, proto.p_helm(0))
        bob = TG.Client(sp, "bob", TG.PW["bob"], name="copilot")
        beat = TG.DriverBeat(alice)                                # the captain's screen 1 (CLAUDE.md D)
        print("| t | gateway time | what happened | what the system did | expected | result |\n|---|---|---|---|---|---|")

        def joy(x=0.0, y=0.0, **kw):
            link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, x, y, **kw))

        def confirm():
            joy(confirm=True)
            time.sleep(0.2)
            joy()

        def driver_yes(timeout=1.0):
            """A question on the captain's screen 1 (only for copilot actions: the captain's own go direct)."""
            k, q = TG.next_event(alice, ("driver_ask",), timeout)
            if q:
                alice.l.send_msg({"t": "driver_answer", "id": q["id"], "yes": True})
            return q

        def auto_go(resume=False):
            seq = alice.l.send(proto.T_AUTO_RESUME if resume else proto.T_AUTO_START, b"", what="AUTO")
            q = driver_yes()
            alice.wait_ack(seq)
            confirm()
            return q, w.until(lambda d: d["mode"] == proto.MODE_AUTO, 5)

        # 1. HEADING LOCK out of the harbour; a crossing ship -> proposal, accepted by the driver
        w.until(lambda d: d.get("pos_source", 0) != 0, 15)
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_HDG_LOCK))
        w.until(lambda d: d["mode"] == proto.MODE_HDG_LOCK, 4)
        joy(0.0, 0.5)
        w.until(lambda d: d.get("est_sog", 0) > 5.0, 60)
        d1 = w.telem
        h0 = d1["target"]
        asim.add(TG.crossing_ship(d1, 239500001, 12.0, 200.0, "HARBOUR FERRY", from_side=-90.0))
        prop = TG.wait_for(lambda: (g.nav_state.get("proposal") or {}).get("kind") == "avoid", 60)
        p = g.nav_state.get("proposal")
        alice.l.send_msg({"t": "avoid_answer", "id": p and p["id"], "yes": True})    # the captain's screen 1
        turned = w.until(lambda d: p and abs((d["target"] - p["heading"] + 180) % 360 - 180) < 1.5, 8)
        row(g, "HEADING LOCK, crossing ship", "proposal \"%s\"; accepted -> locked heading %s" % (
            p and p["text"][:60], "%03.0f" % turned["target"] if turned else "NOT changed"),
            "proposal on the driver screen, ACCEPT changes the heading", bool(prop and turned))
        TG.wait_for(lambda: not g.nav_state.get("threats") or all(t["level"] is None for t in g.nav_state["threats"]),
                    120)
        asim.remove(239500001)
        g.ais.targets.pop(239500001, None)
        joy()
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 8)
        alice.cmd(proto.T_ALARM_ACK)

        # 2. AUTO program: 6 waypoints, past the shallow area (2 km east of the start), shallow check OK
        d = w.telem
        base_n = (d["est_lat"] - TG.SIM_LAT) * 111320.0
        base_e = (d["est_lon"] - TG.SIM_LON) * 111320.0 * 0.5
        legs = [(base_n + 500, base_e), (base_n + 600, base_e + 700), (420, 1700), (450, 2400), (900, 2300),
                (900, 1700)]
        wps = [TG.east_of(e, n) for n, e in legs]
        up = TG.upload(alice, wps, kn=6.0)
        q, auto = auto_go()
        row(g, "AUTO program (6 waypoints, past the 1.5 m shoal)", "upload %s, a question on screen 1 %s (the captain's own start: none), AUTO %s" % (
            up and up["result"], bool(q), "running" if auto else "NOT running"),
            "accepted by the depth check, AUTO after the captain's start + CONFIRM", bool(up and up["result"] == 0 and auto))
        w.until(lambda d: d.get("est_sog", 0) > 4.5, 60)

        # 3. AIS crossing ship in AUTO -> slow down, pass astern, rejoin
        d3 = w.telem
        asim.add(TG.crossing_ship(d3, 239500002, 12.0, 180.0, "CARGO CROSSING"))
        slow = w.until(lambda d: d.get("nav_applied"), 40)
        TG.wait_for(lambda: g.nav_state.get("state") in ("AVOID", "SLOW", "DRIVER DECISION"), 60)
        st = g.nav_state.get("state")
        save0 = w.telem.get("save_seq")
        clear = w.until(lambda d: not d.get("nav_applied") and g.nav_state.get("state") == "CLEAR", 300)
        row(g, "AIS crossing ship during AUTO", "request applied %s (planner %s), back to the leg %s" % (
            "yes" if slow else "NO", st, "yes" if clear else "NO"),
            "slow down, route astern (or DRIVER DECISION slow), then rejoin", bool(slow and clear))
        asim.remove(239500002)
        g.ais.targets.pop(239500002, None)
        alice.cmd(proto.T_ALARM_ACK)

        # 4. a kayak WITHOUT AIS: STAGE 3 off -> alarm + SAVE only; a blip -> nothing; STAGE 3 on -> slows
        v = g.vision
        c0 = v.counts["confirmed"]
        v.inject(0.0, 60.0, seconds=1.0, frames=1)
        w.until(lambda d: False, 2.0)            # keep reading telemetry (Waiter.telem updates only in pump)
        blip_ok = v.counts["confirmed"] == c0
        save0 = w.telem.get("save_seq")
        v.inject(0.0, 70.0, seconds=8.0)
        al = w.until(lambda d: d["alarm_bits"] & 0x4000000, 10)
        sv = w.until(lambda d: d.get("save_seq") != save0, 6)
        no_cap = not (w.telem.get("nav_applied") and w.telem.get("nav_cap_kn") == 0.0)
        row(g, "vision blip + kayak without AIS, STAGE 3 off", "blip filtered %s; OBSTACLE AHEAD %s, SAVE %s, no cap %s" % (
            blip_ok, "yes" if al else "NO", "yes" if sv else "NO", no_cap),
            "blip ignored; alarm + SAVE, no effect on the boat", bool(blip_ok and al and sv and no_cap))
        TG.wait_for(lambda: not v.threats, 15)
        alice.l.send_msg({"t": "vision_stage", "stage": "stage3", "on": True})
        driver_yes()
        TG.next_event(alice, ("vision_ack",), 6)
        thr0 = w.telem["throttle"]
        v.inject(0.0, 35.0, seconds=8.0)
        capd = w.until(lambda d: d.get("nav_applied") and d.get("nav_cap_kn") is not None, 10)
        w.until(lambda d: False, 4.0)
        row(g, "kayak without AIS, STAGE 3 on (driver ACCEPT)", "speed cap %s kn applied %s, throttle %.1f -> %.1f %%" % (
            capd and capd.get("nav_cap_kn"), "yes" if capd else "NO", thr0, w.telem["throttle"]),
            "AUTO slows / stops, never more throttle", bool(capd and w.telem["throttle"] <= thr0 + 0.5))
        TG.wait_for(lambda: v.cap_kn is None, 30)
        alice.cmd(proto.T_ALARM_ACK)

        # 5. GPS lost for 4 minutes (the simulator's GPS off): dead reckoning, AUTO goes on
        link.send(proto.T_SIM_GPS, proto.p_sim_gps(True))
        dr = w.until(lambda d: d.get("pos_source") == 2, 15)
        # 6. internet: 4G fails -> RF; RF fails -> STARLINK (data saver, no sync); 4G back -> 4G, sync again
        drv = g.net.driver
        drv.state["4G"]["internet"] = False
        drv.state["4G2"]["internet"] = False
        rf = TG.wait_for(lambda: g.net.active == "RF", 30)
        drv.state["RF"]["internet"] = False
        star = TG.wait_for(lambda: g.net.active == "STARLINK", 30)
        saver = g.net.data_saver() and not g.net.sync_allowed()
        flags = g.status_flags()
        row(g, "4G fails -> RF; RF fails -> STARLINK", "RF %s, STARLINK %s, data saver + no sync %s, banner flag %s" % (
            rf, star, saver, bool(flags & proto.GWF_STARLINK)), "RF first, then STARLINK emergency: saver, no sync, banner",
            bool(rf and star and saver and flags & proto.GWF_STARLINK))
        drv.state["4G"]["internet"] = True
        back4g = TG.wait_for(lambda: g.net.active == "4G", 60)
        row(g, "4G returns", "back on 4G %s (after %s s stable), sync allowed %s" % (
            back4g, cfg["channels"]["stable_s"], g.net.sync_allowed()), "automatic switch back, sync resumes",
            bool(back4g and g.net.sync_allowed()))
        drv.state["RF"]["internet"] = drv.state["4G2"]["internet"] = True
        t_gps = time.time()
        while time.time() - t_gps < 200:                          # the rest of the 4 minutes
            w.pump(0.5)
        mode_dr = w.telem["mode"]
        link.send(proto.T_SIM_GPS, proto.p_sim_gps(False))
        gps = w.until(lambda d: d.get("pos_source") == 1, 120)
        row(g, "GPS lost for 4 min", "dead reckoning %s, mode after 4 min %s, GPS trusted again %s" % (
            "yes" if dr else "NO", proto.MODES[mode_dr], "yes" if gps else "NO"),
            "DR, AUTO continues, GPS back", bool(dr and gps and mode_dr in (proto.MODE_AUTO,)))
        alice.cmd(proto.T_ALARM_ACK)

        # 7. remote driving from the shore (HELM + onboard CONFIRM), a 2 s link dropout
        stick = {"x": 0.0, "lever": 0.55}
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        r_ask = alice.result(alice.cmd(proto.T_REMOTE, proto.p_remote(0)))
        time.sleep(1.0)
        confirm()
        rem = w.until(lambda d: d["mode"] == proto.MODE_REMOTE, 4)
        time.sleep(8.0)
        snd.stop()
        t_cut = time.time()
        lost = w.until(lambda d: d.get("remote_state") == 2, 3)
        t_lost = time.time() - t_cut
        time.sleep(max(0.0, 2.0 - t_lost))
        snd = RemoteSender(alice.l, lambda: dict(stick))
        snd.start()
        w.until(lambda d: False, 1.0)
        d7 = w.telem
        still_lost = d7.get("remote_state") == 2 and d7["throttle"] < 1.0
        sb = w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 7)
        snd.stop()
        snd = None
        row(g, "remote driving, 2 s dropout", "request %s, REMOTE %s; LOST after %.2f s; data back -> still LOST + idle %s; "
            "STANDBY %s" % (r_ask, "yes" if rem else "NO", t_lost, still_lost, "yes" if sb else "NO"),
            "idle + neutral + gyro hold, never resumes, STANDBY after 5 s",
            bool(r_ask == 0 and rem and lost and t_lost < 1.0 and still_lost and sb))
        alice.cmd(proto.T_ALARM_ACK)

        # 8. back to AUTO; onboard joystick takeover -> MANUAL + SAVE; resume with PC + CONFIRM
        q, auto = auto_go(resume=True)
        if not auto:
            q, auto = auto_go()
        ev0 = len(glob.glob(os.path.join(root, "video", "events", "*")))
        sv0, n0 = w.telem.get("save_seq"), g.rec.saves + g.rec.merged
        joy(0.7, 0.0)
        man = w.until(lambda d: d["mode"] == proto.MODE_MANUAL, 2)
        joy()
        req = w.until(lambda d: d.get("save_seq") != sv0 and d.get("save_trig") == 4, 5)
        # a SAVE request opens a new event, or is added to one that is still open (merged)
        evt = req is not None and TG.wait_for(
            lambda: g.rec.saves + g.rec.merged > n0 or len(glob.glob(os.path.join(root, "video", "events", "*"))) > ev0,
            15)
        time.sleep(1.0)
        q2, back = auto_go(resume=True)
        row(g, "joystick takeover during AUTO, resume", "MANUAL %s, SAVE request %s -> event (new or merged) %s, "
            "resumed %s" % ("yes" if man else "NO", "yes" if req else "NO", evt, "yes" if back else "NO"), "MANUAL at once + SAVE; resume = PC + CONFIRM",
            bool(man and evt and back))

        # 9. arrival at the last waypoint (STOP), STANDBY
        arr = w.until(lambda d: d["alarm_bits"] & 0x20 or d["mode"] == proto.MODE_STANDBY, 1800)
        w.until(lambda d: False, 5.0)
        d9 = w.telem
        row(g, "arrival at the last waypoint", "arrived %s, mode %s, throttle %.1f %%, gear %s" % (
            "yes" if arr else "NO", d9["mode_name"], d9["throttle"], d9["gear"]), "STOP: idle + neutral",
            bool(arr and d9["throttle"] < 1.0))
        link.send(proto.T_SIM_BUTTON, proto.p_sim_button(proto.BTN_STANDBY))
        w.until(lambda d: d["mode"] == proto.MODE_STANDBY, 6)

        # 10. sync of all logs + saved events to the shore, checksums
        t9 = time.time()
        while time.time() - t9 < 180:
            st = g.sync.status()
            if not st["queue"] and time.time() - t9 > 20:
                break
            time.sleep(2.0)
        synced = [p for p in glob.glob(os.path.join(root, "**", "*.synced"), recursive=True)]
        same = bad = missing = changed = 0
        bad_names = []
        for m in synced:
            src = m[:-len(".synced")]
            rel = os.path.relpath(src, root)
            dst = [x for x in glob.glob(os.path.join(shore, "**", rel), recursive=True)]
            if not dst:
                missing += 1
            elif sha(src) == sha(dst[0]):
                same += 1
            elif os.path.getmtime(src) > os.path.getmtime(m) + 0.5:
                changed += 1                                      # changed after the confirmation: queued again
            else:
                bad += 1
                bad_names.append(rel)
        events = len(glob.glob(os.path.join(root, "video", "events", "*")))
        row(g, "sync to the shore receiver", "%d files confirmed: %d checksums equal, %d different %s, %d changed after "
            "the confirmation (queued again), %d not found on shore; %d saved events on the SSD; queue %s" % (
                len(synced), same, bad, bad_names[:3], changed, missing, events, g.sync.status()["queue"]),
            "every confirmed file identical on shore", bool(synced and bad == 0 and missing == 0))
        beat.stop()
        beat = None
        for c in (alice, bob):
            c.close()
        link.stop()
    finally:
        if beat:
            beat.stop()
        if snd:
            snd.stop()
        for c in cams:
            c.stop()
        if asim:
            asim.stop()
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        if recv:
            try:
                recv.stop()
            except Exception:
                pass
        shutil.rmtree(tmp, ignore_errors=True)
    fails = sum(1 for r in ROWS if not r[5])
    with open(os.path.join(OUT, "day_at_sea_B.md"), "w", encoding="utf-8") as f:
        f.write("| t | gateway time | what happened | what the system did | expected | result |\n|---|---|---|---|---|---|\n")
        for t, stamp, what, did, exp, ok in ROWS:
            f.write("| %.0f s | %s | %s | %s | %s | %s |\n" % (t, stamp, what, did, exp, "PASS" if ok else "FAIL"))
    print("\nDAY AT SEA (whole system): %d steps, %d failed, %.0f min" % (len(ROWS), fails, (time.time() - T0) / 60.0))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
