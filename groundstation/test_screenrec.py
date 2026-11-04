"""Screen recording + KEEP 5' / 1h on THIS PC (step 13n4, owner 2026-10-31 E4) - groundstation/screenrec.py + the
station's buttons (app.py / driver.py / hud.py).

  SR1 the ffmpeg command: the window itself (gdigrab hwnd= / x11grab -window_id), 5 fps, at most 1280 wide (even),
      NVENC when the probe works else x264 veryfast, 1-minute MPEG-TS files with UTC names, its own process at a LOW
      priority, no window of its own
  SR2 the ring: files older than 60 min (+ one file) deleted, never the newest one; kept (hard-linked) files stay
      when the ring deletes its own name
  SR3 SAVE: the screen part = pre + post like the cameras (finished files only; the file being written is taken
      once it is finished); KEEP = post 0, permanent; a ring shorter than asked = what exists + how much (have_min)
  SR4 low disk: REC SCREEN does not start (message), stops by itself while recording (message, setting kept),
      a SAVE / KEEP is refused with a message, nothing already kept is touched
  SR5 failures never raise: ffmpeg missing, a window closed (NO WINDOW, tried again), ffmpeg dying (restarted),
      a folder that cannot be written
  SR6 the station: KEEP 5' sends exactly ONE gateway message {"t": "save_keep", "min": 5, "id"} (same id as the
      screen part) and NOTHING to the boat; the answer message «ΑΠΟΘΗΚΕΥΤΗΚΕ – ΟΘΟΝΕΣ + ΚΑΜΕΡΕΣ 5 ΛΕΠΤΑ», a short
      camera ring «ΚΑΜΕΡΕΣ: μόνο 20 λεπτά διαθέσιμα», no gateway / no answer / refused; SAVE carries the id too
  SR7 the buttons: screen 1's bottom bar [REC · SCR · SAVE · 5' · 1h] (●SCR white while recording, hits), screen 2's
      bar 2 group SCR · 5' · 1h; the clicks reach the station; screenrec imports no link / protocol module
  SR8 real ffmpeg (when installed): a Tk window recorded for a few seconds -> playable H.264 .ts files, kept by KEEP

Run:  python test_screenrec.py        (exit code 0 = all passed)
"""
import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("TRYGONS_LANG", "el")
os.environ["TRYGONS_SCREENREC"] = tempfile.mkdtemp(prefix="trygons_sr_")
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(tempfile.mkdtemp(prefix="trygons_sr_set_"), "gs_settings.json")
os.environ.setdefault("TRYGONS_SOUND", "off")
import screenrec  # noqa: E402

results = []
T0 = 1793000000.0           # 2026-10-26 (fixed: file names reproducible)


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


class FakeProc:
    def __init__(self, cmd, **kw):
        self.cmd, self.kw, self.code = cmd, kw, None

    def poll(self):
        return self.code

    def terminate(self):
        self.code = -15

    def kill(self):
        self.code = -9

    def wait(self, t=None):
        return self.code


class Rig:
    def __init__(self, free_gb=100.0, handles=None, encoder="libx264", ffmpeg=None):
        self.root = tempfile.mkdtemp(prefix="trygons_sr_")
        self.free = free_gb * 1e9
        self.procs = []
        self.now = T0
        self.handles = handles if handles is not None else {"SCREEN-1": 0x1234, "SCREEN-2": 0x5678}
        ff = ffmpeg
        if ff is None:
            ff = os.path.join(self.root, "ffmpeg.exe")
            open(ff, "w").close()
        self.r = screenrec.ScreenRec({n: (lambda n=n: self.handles.get(n)) for n in screenrec.SCREENS}, root=self.root,
                                     ffmpeg=ff, encoder=encoder, popen=self.popen, free=lambda p: self.free,
                                     clock=lambda: self.now)

    def popen(self, cmd, **kw):
        p = FakeProc(cmd, **kw)
        self.procs.append(p)
        return p

    def seg(self, name, t, size=1000):
        d = os.path.join(self.root, "ring", name)
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, time.strftime("%Y%m%d-%H%M%S.ts", time.gmtime(t)))
        with open(p, "wb") as f:
            f.write(b"x" * size)
        return p

    def ring(self, name):
        return [os.path.basename(p) for _t, p in self.r.grabs[name].segments()]

    def done(self):
        self.r.stop()
        shutil.rmtree(self.root, ignore_errors=True)


def ev_json(rig, eid):
    with open(os.path.join(rig.r.events_dir(), eid, "event.json"), encoding="utf-8") as f:
        return json.load(f)


def sr1():
    cmd = screenrec.grab_cmd("ff", 0x2a, "/x", "libx264", platform="win32")
    cmd_x = screenrec.grab_cmd("ff", 0x2a, "/x", "libx264", platform="linux")
    cmd_n = screenrec.grab_cmd("ff", 0x2a, "/x", "h264_nvenc", platform="win32")
    j, jx, jn = " ".join(cmd), " ".join(cmd_x), " ".join(cmd_n)
    rig = Rig()
    rig.r.start()
    kw = rig.procs[0].kw if rig.procs else {}
    low = (kw.get("creationflags", 0) & 0x4000 and kw.get("creationflags", 0) & 0x08000000) if os.name == "nt" \
        else callable(kw.get("preexec_fn"))
    ok = ("-f gdigrab" in j and "hwnd=42" in j and "-framerate 5" in j and "min(1280,iw)" in j and "/2)*2" in j and
          "libx264 -preset veryfast" in j and "-segment_time 60" in j and "-segment_format mpegts" in j and
          "%Y%m%d-%H%M%S.ts" in j and "-an" in cmd and "x11grab" in jx and "-window_id 0x2a" in jx and
          "h264_nvenc" in jn and "veryfast" not in jn and len(rig.procs) == 2 and low and
          rig.procs[0].kw.get("env", {}).get("TZ") == "UTC0" and rig.r.recording())
    report("SR1 ffmpeg: the window itself, 5 fps, <= 1280 wide, NVENC / x264 veryfast, 1-min UTC .ts, low priority",
           ok, "Windows: %s ...; X11 window id: %s; NVENC: %s; processes %d, low priority %s" % (
               j[:120], "-window_id 0x2a" in jx, "h264_nvenc" in jn, len(rig.procs), bool(low)))
    rig.done()


def sr2():
    rig = Rig()
    rig.r.start()
    for i in range(70):
        rig.seg("SCREEN-1", T0 + i * 60)
    rig.now = T0 + 70 * 60
    kept_ok, _k, _h = rig.r.save("20261026-000000_keep5", 300.0, 0.0, "keep5", keep=True)
    kept_dir = os.path.join(rig.r.events_dir(), "20261026-000000_keep5", "SCREEN-1")
    kept0 = sorted(os.listdir(kept_dir)) if os.path.isdir(kept_dir) else []
    rig.now = T0 + 140 * 60                         # 70 min later: the whole old ring is older than 60 min
    rig.seg("SCREEN-1", rig.now - 30)
    rig.r.housekeeping(rig.now)
    left = rig.ring("SCREEN-1")
    kept1 = sorted(os.listdir(kept_dir)) if os.path.isdir(kept_dir) else []
    rig2 = Rig()
    for i in range(3):
        rig2.seg("SCREEN-2", T0 - 10 * 3600 + i * 60)
    rig2.r.housekeeping(T0)
    newest_left = rig2.ring("SCREEN-2")
    report("SR2 ring: older than 60 min deleted, never the newest file; kept (hard-linked) files survive the ring",
           kept_ok and len(kept0) >= 5 and kept1 == kept0 and len(left) == 1 and len(newest_left) == 1,
           "KEEP 5' took %d files; after 70 min the ring holds %s, the kept folder still %d files; a ring of only very "
           "old files keeps its newest: %s" % (len(kept0), left, len(kept1), newest_left))
    rig.done()
    rig2.done()


def sr3():
    rig = Rig()
    rig.r.start()
    for i in range(20):                             # 20 min of both screens, the last file still being written
        rig.seg("SCREEN-1", T0 + i * 60)
        rig.seg("SCREEN-2", T0 + i * 60)
    rig.now = T0 + 19 * 60 + 30
    ok, key, have = rig.r.save("20261026-001930_button", 300.0, 120.0, "save")
    ev = ev_json(rig, "20261026-001930_button")
    n0 = len(ev["screens"]["SCREEN-1"])
    open0 = ev["state"] == "recording"
    rig.seg("SCREEN-1", T0 + 20 * 60)               # time goes on: new files, the old newest one is finished
    rig.seg("SCREEN-2", T0 + 20 * 60)
    rig.seg("SCREEN-1", T0 + 21 * 60)
    rig.seg("SCREEN-2", T0 + 21 * 60)
    rig.seg("SCREEN-1", T0 + 22 * 60)
    rig.seg("SCREEN-2", T0 + 22 * 60)
    rig.now = T0 + 21 * 60 + 40
    rig.r.tick()
    rig.now = T0 + 23 * 60
    rig.r.tick()
    ev = ev_json(rig, "20261026-001930_button")
    f1 = ev["screens"]["SCREEN-1"]
    ts = [screenrec.seg_time(f) for f in f1]
    window_ok = ts and min(ts) <= T0 + 14 * 60 + 30 and min(ts) >= T0 + 13 * 60 and max(ts) >= T0 + 21 * 60
    # KEEP 60 with only 20 min in the ring: what exists + how much
    ok60, _k, have60 = rig.r.save("20261026-002300_keep60", 3600.0, 0.0, "keep60", keep=True)
    e60 = ev_json(rig, "20261026-002300_keep60")
    report("SR3 SAVE = pre + post like the cameras (files finished later are taken); KEEP = post 0, permanent; "
           "a shorter ring = what exists + how much",
           ok and key == "scr.saved" and open0 and n0 == 5 and window_ok and ev["state"] in ("complete", "recording")
           and ok60 and e60["keep"] and e60["post_s"] == 0 and 20 <= have60 <= 24 and
           len(e60["screens"]["SCREEN-2"]) >= 20,
           "SAVE at 19:30 (5 min before + 2 after): %d files at once (the open one waits), later %d files from %s to %s; "
           "KEEP 1h: have %s min, %d files" % (
               n0, len(f1), time.strftime("%H:%M", time.gmtime(min(ts))) if ts else None,
               time.strftime("%H:%M", time.gmtime(max(ts))) if ts else None, have60, len(e60["screens"]["SCREEN-2"])))
    rig.done()


def sr4():
    rig = Rig(free_gb=1.0)
    ok, key = rig.r.start()
    refused = not ok and key == "scr.low_disk" and not rig.procs and not rig.r.recording()
    rig.free = 100e9
    ok2, _ = rig.r.start()
    for i in range(6):
        rig.seg("SCREEN-1", T0 + i * 60)
    rig.now = T0 + 6 * 60
    kept, _k, _h = rig.r.save("20261026-000600_keep5", 300.0, 0.0, "keep5", keep=True)
    before = sorted(os.listdir(os.path.join(rig.r.events_dir(), "20261026-000600_keep5", "SCREEN-1")))
    rig.free = 0.5e9                                    # the disk fills up while recording
    rig.r.tick()
    stopped = not rig.r.recording() and all(p.code is not None for p in rig.procs)
    notes = [k for k, _kw in rig.r.pop_notes()]
    s_ok, s_key, _ = rig.r.save("20261026-000700_keep5", 300.0, 0.0, "keep5", keep=True)
    after = sorted(os.listdir(os.path.join(rig.r.events_dir(), "20261026-000600_keep5", "SCREEN-1")))
    report("SR4 low disk: not started / stopped by itself with a message; SAVE / KEEP refused; nothing kept is lost",
           refused and ok2 and kept and stopped and "scr.low_disk_stop" in notes and not s_ok and
           s_key == "scr.low_disk" and set(before) <= set(after) and len(before) >= 5,
           "start with 1 GB free: %s; recording then disk 0.5 GB: stopped %s, message %s; KEEP then: %s; the earlier "
           "KEEP still %d files" % (key, stopped, notes, s_key, len(after)))
    rig.done()


def sr5():
    rig = Rig(ffmpeg=os.path.join(tempfile.gettempdir(), "no_such_ffmpeg_13n4.exe"))
    ok, key = rig.r.start()
    nf = not ok and key == "scr.no_ffmpeg" and not rig.procs
    rig.done()
    rig = Rig(handles={"SCREEN-1": None, "SCREEN-2": 0x99})
    rig.r.start()
    st = dict(rig.r.status()["screens"])
    one = len(rig.procs) == 1
    rig.procs[0].code = 1                                # ffmpeg dies (e.g. the window closed)
    rig.handles["SCREEN-1"] = 0x77                      # screen 1 opened meanwhile
    rig.now += 1
    rig.r.tick()
    soon = len(rig.procs)                               # not at once (RESTART_S)
    rig.now += screenrec.RESTART_S + 1
    rig.r.tick()
    later = len(rig.procs)
    st2 = dict(rig.r.status()["screens"])

    def boom():
        raise RuntimeError("window gone")
    rig.r.sources["SCREEN-1"] = boom
    rig.r.grabs["SCREEN-1"].proc.code = 1
    rig.now += screenrec.RESTART_S + 1
    try:
        rig.r.tick()
        no_raise = True
    except Exception:
        no_raise = False
    rig.done()
    bad = tempfile.mkdtemp(prefix="trygons_sr_")
    blocker = os.path.join(bad, "file")
    open(blocker, "w").close()
    r3 = screenrec.ScreenRec({"SCREEN-1": lambda: 1}, root=os.path.join(blocker, "sub"), ffmpeg=blocker,
                             encoder="libx264", popen=lambda *a, **k: FakeProc(a), free=lambda p: 1e12)
    ok3, key3 = r3.start()
    shutil.rmtree(bad, ignore_errors=True)
    report("SR5 failures never raise: ffmpeg missing, a window closed / not there, ffmpeg dying, a bad folder",
           nf and one and st["SCREEN-1"] == "NO WINDOW" and st["SCREEN-2"] == "REC" and soon == 1 and later == 3 and
           st2["SCREEN-1"] == "REC" and st2["SCREEN-2"] == "REC" and no_raise and not ok3 and key3 == "scr.no_folder",
           "ffmpeg missing: %s; screen 1 closed: %s; after ffmpeg died: %d processes at once, %d after %g s (%s); a "
           "source that raises: no exception %s; folder under a file: %s" % (
               key, st, soon, later, screenrec.RESTART_S, st2, no_raise, key3))


def app_rig():
    import tkinter as tk
    import app as appmod
    root = tk.Tk()
    root.withdraw()
    a = appmod.App(root)
    return tk, appmod, root, a


class FakeGw:
    def __init__(self):
        self.msgs, self.frames = [], []

    def send_msg(self, m):
        self.msgs.append(m)

    def send(self, *a, **k):
        self.frames.append(a)

    def stop(self):
        pass


def sr6_7():
    import i18n
    i18n.set_lang("el", save=False)
    tk, appmod, root, a = app_rig()
    said = []
    a.say = lambda text, col=None: said.append(text)
    fake = FakeGw()
    fake.__class__ = type("FakeGwLink", (FakeGw, appmod.GatewayLink), {})
    real = a.link
    a.link = fake
    try:
        rr = a.scr
        rr.ffmpeg = None
        for i in range(8):
            d = os.path.join(rr.root, "ring", "SCREEN-1")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S.ts", time.gmtime(time.time() - 480 + i * 60))),
                      "wb") as f:
                f.write(b"x")
        eid = a.keep_save(5)
        m = list(fake.msgs)
        sent_ok = m == [{"t": "save_keep", "min": 5, "id": eid}] and not fake.frames
        ev_dir = os.path.join(rr.events_dir(), eid)
        a.on_keep_ack({"t": "keep_ack", "ok": True, "id": eid, "ask": eid, "min": 5, "have_min": 60.0})
        msg_both = said[-1] if said else ""
        eid2 = a.keep_save(60)
        a.on_keep_ack({"t": "keep_ack", "ok": True, "id": eid2, "ask": eid2, "min": 60, "have_min": 20.0})
        msg_short = said[-1]
        eid3 = a.keep_save(5)
        a.on_keep_ack({"t": "keep_ack", "ok": False, "text": "VIEWER: no SAVE", "id": None, "ask": eid3})
        msg_ref = said[-1]
        eid4 = a.keep_save(5)
        a.keep_pending[eid4]["t"] -= appmod.KEEP_ACK_S + 1
        a.scr_tick()
        msg_to = said[-1]
        fake.msgs.clear()
        a.video_save("captain")
        save_m = list(fake.msgs)
        a.link = None
        eid5 = a.keep_save(5)
        msg_nogw = said[-1]
        report("SR6 KEEP 5' / 1h: ONE gateway message with the screen part's id, nothing to the boat; the messages",
               sent_ok and os.path.isdir(ev_dir) and msg_both == "ΑΠΟΘΗΚΕΥΤΗΚΕ – ΟΘΟΝΕΣ + ΚΑΜΕΡΕΣ 5 ΛΕΠΤΑ" and
               "ΑΠΟΘΗΚΕΥΤΗΚΕ – ΟΘΟΝΕΣ + ΚΑΜΕΡΕΣ 1 ΩΡΑ" in msg_short and "ΚΑΜΕΡΕΣ: μόνο 20 λεπτά διαθέσιμα" in msg_short
               and "ΟΘΟΝΕΣ: μόνο" in msg_short and msg_ref.startswith("ΑΠΟΘΗΚΕΥΤΗΚΕ – ΟΘΟΝΕΣ 5 ΛΕΠΤΑ") and
               "ΘΕΑΤΗΣ: χωρίς SAVE" in msg_ref and "καμία απάντηση" in msg_to and len(save_m) == 1 and
               save_m[0]["t"] == "save" and screenrec.EVENT_ID_RE.match(save_m[0].get("id", "")) and
               "μόνο μέσω του gateway" in msg_nogw and eid5,
               "sent %s; frames to the boat %d; answers: «%s» / «%s» / «%s» / «%s» / without a gateway «%s»; SAVE "
               "sends %s" % (m, len(fake.frames), msg_both, msg_short, msg_ref, msg_to, msg_nogw, save_m))
        # SR7 the buttons
        import hud
        h = hud.Hud()
        snap = hud.Snapshot(t=None, gw={}, lamps=a.lamps, scr=False)
        g = [it for grp in h.bar_groups(snap) for it in grp]
        acts = [it[2] for it in g]
        order = acts.index("scr") < acts.index("save") < acts.index("keep5") < acts.index("keep60")
        snap_on = hud.Snapshot(t=None, gw={}, lamps=a.lamps, scr=True)
        on = [it for grp in h.bar_groups(snap_on) for it in grp if it[2] == "scr"][0]
        h.build(1600, 900, snap_on)
        hits = {hh[1] for hh in h.hits} if h.hits and isinstance(h.hits[0], tuple) else \
            {getattr(hh, "action", None) or (hh.get("action") if isinstance(hh, dict) else None) for hh in h.hits}
        b2 = [it[1] for it in a.bar_items(2) if it[0] == "btn"]
        clicked = []
        a.scr_toggle = lambda: clicked.append("scr")
        a.keep_save = lambda mn: clicked.append(mn)
        for it in a.bar_items(2):
            if it[0] == "btn" and it[1] in ("scr", "keep5", "keep60"):
                it[3]()
        a.driver = None
        tree = ast.parse(open(os.path.join(HERE, "screenrec.py"), encoding="utf-8").read())
        imps = set()
        for n in ast.walk(tree):
            if isinstance(n, ast.Import):
                imps |= {x.name for x in n.names}
            elif isinstance(n, ast.ImportFrom):
                imps.add(n.module)
        clean = not imps & {"link", "proto", "gwclient", "socket", "stations"}
        report("SR7 buttons: screen 1 [SCR · SAVE · 5' · 1h] (●SCR white while recording, clickable), screen 2 bar 2 "
               "SCR · 5' · 1h; the clicks reach the station; screenrec talks to nobody",
               order and on[1] == "●SCR" and on[5] == "#ffffff" and {"scr", "keep5", "keep60", "save"} <= hits and
               {"scr", "keep5", "keep60"} <= set(b2) and clicked == ["scr", 5, 60] and clean,
               "screen 1 bar: %s; on: %s; hits %s; screen 2 bar 2: %s; clicks %s; screenrec imports %s" % (
                   [x for x in acts if x], on[1:2] + on[5:6], sorted(x for x in hits if x in ("scr", "keep5", "keep60",
                                                                                            "save")),
                   [x for x in b2 if x in ("scr", "keep5", "keep60")], clicked, sorted(i for i in imps if i)))
    finally:
        a.link = real
        try:
            root.destroy()
        except Exception:
            pass


def sr8():
    ff = screenrec.ffmpeg_bin()
    if not ff or os.name != "nt":
        report("SR8 real ffmpeg: a Tk window recorded -> playable .ts files, kept by KEEP", True,
               "N/A here (no ffmpeg / not Windows): %s" % ff)
        return
    import tkinter as tk
    root = tk.Tk()
    root.title("screenrec test")
    root.geometry("640x360+40+40")
    lab = tk.Label(root, text="TRYGONS 13n4", bg="#202020", fg="white", font=("Arial", 40))
    lab.pack(fill="both", expand=1)
    root.update()
    d = tempfile.mkdtemp(prefix="trygons_sr8_")
    r = screenrec.ScreenRec({"SCREEN-2": lambda: int(root.wm_frame(), 16)}, root=d, seg_s=2, encoder=None)
    ok, key = r.start()
    end = time.time() + 12
    while time.time() < end:
        root.update()
        lab.configure(text=time.strftime("%H:%M:%S"))
        r.tick()
        time.sleep(0.1)
    segs = r.grabs["SCREEN-2"].segments()
    eid = screenrec.event_id(time.time(), "keep5")
    k_ok, _k, have = r.save(eid, 300.0, 0.0, "keep5", keep=True)
    r.stop()
    r.tick()
    kept = os.listdir(os.path.join(d, "events", eid, "SCREEN-2")) if os.path.isdir(os.path.join(d, "events", eid,
                                                                                             "SCREEN-2")) else []
    play = None
    if segs:
        p = segs[0][1]
        q = subprocess.run([ff, "-hide_banner", "-loglevel", "error", "-i", p, "-frames:v", "1", "-f", "rawvideo",
                            "-pix_fmt", "gray", "-"], capture_output=True, timeout=30)
        pr = subprocess.run([ff, "-hide_banner", "-i", p], capture_output=True, text=True, timeout=30)
        play = (len(q.stdout), "h264" in pr.stderr, "640x" in pr.stderr or "x360" in pr.stderr)
    root.destroy()
    report("SR8 real ffmpeg: a Tk window recorded (encoder %s) -> playable H.264 .ts files, kept by KEEP" % r.encoder,
           ok and len(segs) >= 2 and play and play[0] > 0 and play[1] and k_ok and len(kept) >= 1,
           "start %s; %d files; first file: picture bytes %s, H.264 %s, window size %s; KEEP took %d files (have %s "
           "min)" % (key, len(segs), play and play[0], play and play[1], play and play[2], len(kept), have))
    shutil.rmtree(d, ignore_errors=True)


def main():
    for fn in (sr1, sr2, sr3, sr4, sr5, sr6_7, sr8):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            report(fn.__name__.upper(), False, "exception %r" % e)
    n = len(results)
    print("\nSCREEN RECORDING TESTS: %d / %d passed" % (sum(results), n))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
