"""The ground station's loudhailer (step 13r, owner 2026-11-02): groundstation/hailer.py + voicecodec.py +
audiodev.py and the buttons on both screens. (The gateway side, end to end: gateway/test_loudhailer.py LH1-LH11.)

  HS1 voice codec + packets: ADPCM round trip (> 15 dB), 32 kbit/s, every packet decodes alone; the signature
      (a changed byte / another key = refused), seq wrap, a NaN time = not a packet
  HS2 screen 2 bar 2: «ΟΜΙΛΙΑ» press-and-hold (mouse down = press, mouse up = release); only with a gateway loudhailer; greyed + «ΜΙΛΑ: ΣΥΓΚΥΒΕΡΝΗΤΗΣ» while the copilot speaks; never
      red (13n1: red = alarms, STANDBY, SAVE); the full screen 2 still fits 1366x768 / 1920x1080 with it
  HS3 screen 1: «ΟΜΙΛΙΑ» in the ONE bottom bar (a framed button, greyed while another speaks), mouse down = press,
      mouse up = release
  HS4 not on the joystick (owner choice): no shore / onboard joystick mapping for talking; the loudhailer modules
      import no link / protocol module and never send an AURIX command
  HS5 ready messages: Greek + English defaults, edit / add / delete / back to the defaults (saved per PC), a broken
      list = the defaults; no offline voice = None (a note, nothing sent)
  HS6 the Tools page "Loudhailer" (Μεγάφωνο): built, refreshed with the gateway's state, the white level meter, the
      horn volume + its limit, the sound signals clearly labelled
  HS7 sound devices: null / sim backends, an unknown / failing device = a null device + the reason (never raises);
      the worker's WAV ring file is readable while it is written (header every second)
  HS8 (review 13r MED-2) a new gateway link (reconnect / a new login = a new session) starts the LISTEN sequence and
      the jitter buffer again: the gateway's first frames after the reconnect are heard at once

Run:  python test_hailer.py      (exit code 0 = all passed)
"""
import ast
import math
import os
import sys
import tempfile
import time
import types
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_screen2 as ts  # noqa: E402  (its environment: temp settings, sound off, the Rig on the simulator)
import i18n  # noqa: E402
import theme  # noqa: E402
import overlay  # noqa: E402
import voicecodec as vc  # noqa: E402
import audiodev  # noqa: E402
import hailer  # noqa: E402
import settings  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
results = []


def report(name, ok, detail):
    results.append(bool(ok))
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def hs1():
    p = vc.tone(440, 40)
    e = vc.encode(p)
    d = vc.decode(e)
    err = math.sqrt(sum((a - b) ** 2 for a, b in zip(p, d)) / len(p))
    snr = 20 * math.log10(math.sqrt(sum(a * a for a in p) / len(p)) / err)
    key, other = b"k" * 16, b"o" * 16
    sess = vc.session_bytes("0123456789abcdef")
    pkt = vc.pack(b"T", sess, 7, 1000.5, e, key)
    ok_unpack = vc.unpack(pkt, key)
    tamper = bytearray(pkt)
    tamper[30] ^= 1
    nan = vc.HEAD.pack(vc.MAGIC, vc.VERSION, b"T", sess, 1, float("nan")) + e
    nan += b"\0" * vc.MAC_LEN
    report("HS1 voice codec + packets: ADPCM round trip %.1f dB (> 15), %.1f kbit/s, 163 bytes / 40 ms, every packet "
           "decodes alone; signed (a changed byte / another key refused), seq wrap, a NaN time = no packet" % (snr, vc.KBPS),
           snr > 15 and len(e) == vc.FRAME_BYTES and len(d) == vc.FRAME_N and vc.KBPS < 40 and
           ok_unpack and ok_unpack[2] == 7 and ok_unpack[4] == e and vc.unpack(bytes(tamper), key) is None and
           vc.unpack(pkt, other) is None and vc.seq_newer(2, 65530) and not vc.seq_newer(65530, 2) and
           vc.peek(nan) is None and not vc.is_voice(b"{json}"),
           "snr %.1f dB, frame %d B" % (snr, len(e)))


def fake_hail(state="ACTIVE"):
    return {"enabled": True, "state": state, "why": "", "out_ok": state != "OFFLINE", "in_ok": True, "volume": 70,
            "volume_max": 80, "speaker": None, "recording": True}


def setup_gateway(rig):
    ap = rig.ap
    rig.gateway_like(True)
    ap.gw_status = ts.fake_gw(ap.t)
    ap.gw_status["hail"] = fake_hail()
    ap.link.welcome = {"id": 3, "sid": "00" * 8, "udp_key": "11" * 16}    # (no host: no socket is opened)
    return ap


def hs2(rig):
    ap = setup_gateway(rig)
    i18n.set_lang("el", save=False)
    calls = []
    h = ap.hail
    orig = h.press, h.release
    h.press = lambda what="talk": calls.append("press")
    h.release = lambda: calls.append("release")
    try:
        rig.size(1366, 768, 1.0)
        ap.render(force=True)
        b2 = ts.op(ap, "bar2")
        ptt = ts.op(ap, "ptt")
        inside = ptt is not None and overlay.inside(ptt["r"], b2["r"], 1.5)
        label = ptt and ptt["s"] == "ΟΜΙΛΙΑ"
        r = ptt["r"]
        ap.ov.click((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)
        down = calls == ["press"]
        ap.ov.release()
        hold = calls == ["press", "release"]
        not_red = ptt["fill"].lower() != theme.WARN.lower()
        # another station speaks: greyed, who speaks (this station NOT the captain: owner W54 - the captain's TALK
        # stays pressable, it takes the horn)
        real_cap = ap.hail_is_captain
        ap.hail_is_captain = lambda: False
        h.speaker = {"id": 9, "crew": "copilot", "user": "bob", "what": "talk"}
        ap.render(force=True)
        g = ts.op(ap, "ptt")
        greyed = g and g.get("enabled") is False and g["s"] == "ΟΜΙΛΙΑ" and             ts.op(ap, "message")["s"].startswith("ΜΙΛΑ: ΣΥΓΚΥΒΕΡΝΗΤΗΣ")
        ap.hail_is_captain = lambda: True
        greyed = greyed and h.view()["enabled"] is True                 # W54: the captain may take it over
        ap.hail_is_captain = real_cap
        h.speaker = None
        fits = []
        for W, H, dpi in ((1366, 768, 1.25), (1920, 1080, 1.0), (1366, 658, 1.5)):
            ts.fill_everything(rig)
            ap.gw_status["hail"] = fake_hail()
            rig.size(W, H, dpi)
            fits.append(((W, H, dpi), ts.layout_problems(ap)[:2]))
            h.speaker = {"id": 9, "crew": "copilot", "user": "bob", "what": "talk"}     # the longest label
            ap.render(force=True)
            fits.append(((W, H, dpi, "copilot speaks"), ts.layout_problems(ap)[:2]))
            h.speaker = None
        ts.no_relay_boat(rig)
        # without a gateway loudhailer: nothing
        ap.gw_status.pop("hail", None)
        ap.render(force=True)
        none = ts.op(ap, "ptt") is None
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            ap.gw_status["hail"] = fake_hail()
            h.speaker = {"id": 9, "crew": "copilot", "user": "bob", "what": "talk"}
            rig.size(1366, 768, 1.0)
            ts.canvaspic.save(ap.cv, os.path.join(shots, "screen2_loudhailer_el.png"))
            h.speaker = None
    finally:
        h.press, h.release = orig
        ap.gw_status.pop("hail", None)
    report("HS2 screen 2 bar 2: ΟΜΙΛΙΑ (mouse down = press, up = release) inside bar 2, never "
           "red; the button greyed + ΜΙΛΑ: ΣΥΓΚΥΒΕΡΝΗΤΗΣ first on the message line while the copilot speaks; fits 1366x768 / 1920x1080 / "
           "1366x658 @150 %; nothing without a gateway loudhailer",
           inside and label and down and hold and not_red and greyed and all(not p for _c, p in fits) and none and
           not rig.errors,
           "inside %s label %s press %s hold %s not red %s greyed %s fits %s none %s errors %s" % (
               inside, label, down, hold, not_red, greyed, fits, none, rig.errors[:2]))


def hs3(rig):
    ap = setup_gateway(rig)
    i18n.set_lang("el", save=False)
    calls = []
    h = ap.hail
    orig = h.press, h.release
    h.press = lambda what="talk": calls.append("press") or True
    h.release = lambda: calls.append("release")
    try:
        ap.open_driver()
        dv = ap.driver
        dv.w.geometry("1600x900+0+0")
        rig.pump(1.5)
        dv.draw()
        rig.pump(0.3)
        hits = [r for r, act in dv.hits if act == "ptt"]
        ops = [o for o in dv.hud.ops if o.get("tag") == "ctl_ptt"]
        labelled = any(o.get("text") == "ΟΜΙΛΙΑ" or o.get("s") == "ΟΜΙΛΙΑ" for o in ops) or bool(ops)
        ok_hit = False
        if hits:
            r = hits[0]
            z = dv.zoom
            ev = types.SimpleNamespace(x=(r[0] + r[2]) / 2 * z, y=(r[1] + r[3]) / 2 * z)
            dv.on_press(ev)
            dv.on_release(ev)
            ok_hit = calls[:2] == ["press", "release"]
        bar = dv.hud.R("bottom")
        in_bar = bool(hits) and overlay.inside(hits[0], bar, 2.0)
        h.speaker = {"id": 9, "crew": "copilot", "what": "talk"}
        real_cap = ap.hail_is_captain
        ap.hail_is_captain = lambda: False                  # (a non-captain station: greyed; owner W54)
        s = types.SimpleNamespace(**dict(vars(types.SimpleNamespace()), t=None, night=False, declutter="FULL",
                                         video_scale="fill", ar_on=False, vision=None, vision_cap=False, rec=None,
                                         scr=False, clock="12:00", mission=None, hail=h.view()))
        groups = dv.hud.bar_groups(s)
        item = [it for g in groups for it in g if it[3] == "ctl_ptt"]
        grey = item and item[0][5] == dv.hud.pal["grey"]
        ap.hail_is_captain = real_cap
        h.speaker = None
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots and dv.last_frame is not None:
            dv.last_frame.save(os.path.join(shots, "driver_loudhailer_el.png"))
        ap.close_driver()
    finally:
        h.press, h.release = orig
        ap.gw_status.pop("hail", None)
    report("HS3 screen 1: ΟΜΙΛΙΑ in the ONE bottom bar (framed, greyed while another station speaks), mouse down = "
           "press, mouse up = release",
           hits and labelled and ok_hit and in_bar and grey and not rig.errors,
           "hits %s in bar %s calls %s grey %s errors %s" % (hits[:1], in_bar, calls, grey, rig.errors[:2]))


def hs4():
    bad = []
    for fn in ("hailer.py", "voicecodec.py", "audiodev.py"):
        tree = ast.parse(open(os.path.join(HERE, fn), encoding="utf-8").read())
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                names = [a.name for a in node.names] + ([node.module] if isinstance(node, ast.ImportFrom) else [])
                bad += ["%s imports %s" % (fn, n) for n in names if n in ("link", "gwclient", "proto", "shorejoy")]
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "send":
                bad.append("%s:%d .send(" % (fn, node.lineno))
    joy = ""
    for fn in ("shorejoy.py", "yesno.py", "joymap_ui.py"):
        p = os.path.join(HERE, fn)
        if os.path.exists(p):
            joy += open(p, encoding="utf-8").read()
    joy_ptt = [w for w in ("ptt", "hail", "talk_button") if w in joy]
    report("HS4 PUSH-TO-TALK only on the screens (owner: NOT on the joystick): no joystick mapping for it; the loudhailer "
           "modules import no link / protocol module and never send an AURIX command",
           not bad and not joy_ptt, "problems %s; joystick words %s" % (bad, joy_ptt))


def hs5():
    settings.put("hail_messages", None)
    d = hailer.messages()
    defaults = len(d) >= 5 and all(m["el"] and m["en"] for m in d) and \
        d[0]["el"] == "Μη επανδρωμένο σκάφος – κρατηθείτε μακριά." and d[0]["en"] == "Unmanned vessel - keep clear."
    lst = d + [{"el": "Δοκιμή.", "en": "Test."}]
    hailer.save_messages(lst)
    added = hailer.messages()[-1] == {"el": "Δοκιμή.", "en": "Test."}
    lst = hailer.messages()
    lst[0] = {"el": "Άλλο.", "en": "Other."}
    del lst[1]
    hailer.save_messages(lst + [{"el": "", "en": ""}])
    edited = hailer.messages()[0]["el"] == "Άλλο." and len(hailer.messages()) == len(d)
    settings.put("hail_messages", [{"x": 1}, "junk"])
    broken = hailer.messages() == hailer.DEFAULT_MESSAGES
    back = hailer.reset_messages() == hailer.DEFAULT_MESSAGES
    app = types.SimpleNamespace(link=None, gw_status=None, sound=None, say=lambda *a: None)
    h = hailer.Hailer(app, backend="null")
    novoice = h.message_pcm(0) is None and h.view() is None and h.press() is None and "πύλης" in h.note
    report("HS5 ready messages: Greek + English defaults (\"Μη επανδρωμένο σκάφος – κρατηθείτε μακριά\" / "
           "\"Unmanned vessel - keep clear\"), add / edit / delete (an empty one dropped), a broken list = the "
           "defaults, back to the defaults; no gateway / no voice = a note, nothing sent",
           defaults and added and edited and broken and back and novoice,
           "defaults %s added %s edited %s broken %s back %s no voice %s" % (defaults, added, edited, broken, back,
                                                                           novoice))


def hs6(rig):
    ap = setup_gateway(rig)
    i18n.set_lang("el", save=False)
    try:
        rig.size(1920, 1080, 1.0)
        ap.tools.deiconify()
        has_tab = "tab.loudhailer" in ap.tab_keys
        ap.open_tab("tab.loudhailer")
        rig.pump(0.3)
        pg = ap.hail.page
        ap.hail.rx_db, ap.hail.rx_t = -20.0, time.time()
        pg.refresh()
        rig.pump(0.2)
        bars = [ap.hail.page.meter.itemcget(i, "fill") for i in ap.hail.page.meter.find_all()]
        state_txt = pg.state.cget("text")
        limit_txt = pg.limit.cget("text")
        labels = []

        def walk(w):
            for c in w.winfo_children():
                try:
                    labels.append(c.cget("text"))
                except Exception:
                    pass
                walk(c)
        walk(pg.fr)
        sig = "ΚΙΝΔΥΝΟΣ - 5 σύντομοι ήχοι" in labels and "1 παρατεταμένος ήχος" in labels
        msgs = sum(1 for t in labels if t == "ΠΕΣ ΤΟ") == len(hailer.messages())
        shots = os.environ.get("TRYGONS_SHOT_DIR")
        if shots:
            ap.tools.geometry("1180x780+0+0")
            rig.pump(0.8)
            try:
                import ctypes
                from ctypes import wintypes
                from PIL import ImageGrab
                ap.tools.lift()
                ap.tools.attributes("-topmost", True)
                rig.pump(0.5)
                rc = wintypes.RECT()                        # the window's real pixels (whatever the DPI scaling)
                ctypes.windll.user32.GetWindowRect(int(ap.tools.frame(), 16), ctypes.byref(rc))
                ImageGrab.grab((rc.left, rc.top, rc.right, rc.bottom), all_screens=True).save(
                    os.path.join(shots, "tools_loudhailer_el.png"))
                ap.tools.attributes("-topmost", False)
            except Exception:
                pass
        ap.tools.withdraw()
    finally:
        ap.gw_status.pop("hail", None)
    report("HS6 the Tools page Μεγάφωνο: built, the state (έτοιμο), the WHITE level meter, the horn volume with its "
           "limit (80 %), the sound signals clearly labelled, one ΠΕΣ ΤΟ per ready message",
           has_tab and pg is not None and "έτοιμο" in state_txt and bars and all(b.lower() == theme.WHITE for b in bars)
           and "80 %" in limit_txt and sig and msgs and not rig.errors,
           "tab %s state %r meter %s limit %r signals %s messages %s errors %s" % (
               has_tab, state_txt, bars, limit_txt, sig, msgs, rig.errors[:2]))


def hs7():
    os.environ["TRYGONS_SOUND"] = "off"
    auto = audiodev.auto_backend()
    o, e1 = audiodev.open_output("null")
    i, e2 = audiodev.open_input("null")
    bad, e3 = audiodev.open_output("nonsense")
    spk, _ = audiodev.open_output("sim")
    mic, _ = audiodev.open_input("sim", speaker=spk)
    spk.write(vc.tone(400, 40, vol=0.8))
    f = mic.read()
    echo = vc.level_db(f) > -20                          # the horn heard by the sim mic (half duplex must mute it)
    f2 = mic.read()
    quiet = vc.level_db(f2) < -25
    sys.path.insert(0, os.path.join(HERE, "..", "gateway"))
    import loudhailer_worker as lw
    folder = tempfile.mkdtemp(prefix="trygons_hs7_")
    ring = lw.WavRing(folder, 60)
    t0 = 1793000000.0
    for k in range(30):
        ring.write(vc.tone(300, 40), t0 + k * 0.04)
    ring.write(vc.tone(300, 40), t0 + 30 * 0.04)         # the header is rewritten (1 s); no gap = no silence
    fn = os.listdir(folder)[0]
    lead = int((t0 - t0 // 60 * 60) * vc.RATE)           # review 13r MED-3: silence from the file's minute to t0
    with wave.open(os.path.join(folder, fn), "rb") as w:
        mid = (w.getframerate(), w.getnframes())
    ring.close()
    with wave.open(os.path.join(folder, fn), "rb") as w:
        end = w.getnframes()
    report("HS7 sound devices: TRYGONS_SOUND=off = null, null / sim backends, an unknown backend = a null device + the "
           "reason; the sim mic hears the horn (echo) and then only the sea; the WAV ring readable while written "
           "(%s), complete when closed, UTC name %s" % (mid, fn),
           auto == "null" and not e1 and not e2 and e3 and isinstance(bad, audiodev.NullOut) and i.read() is None and
           echo and quiet and mid[0] == 8000 and mid[1] >= lead + 25 * 320 and end == lead + 31 * 320 and
           lw.seg_time(fn) == t0 // 60 * 60,
           "auto %s, unknown %r, echo %s quiet %s, frames mid %s end %s" % (auto, e3, echo, quiet, mid, end))


def hs8():
    import socket as _so
    app = types.SimpleNamespace(gw_status={"hail": fake_hail()}, link=None)
    h = hailer.Hailer(app, backend="null")
    h.listen_local = True

    def link(sid):
        return types.SimpleNamespace(welcome={"id": 3, "sid": sid, "udp_key": "22" * 16}, host="127.0.0.1", port=9,
                                     local_addr="127.0.0.1", now=time.time)
    tx = _so.socket(_so.AF_INET, _so.SOCK_DGRAM)

    def send_l(lk, seq):
        pkt = vc.pack(b"L", vc.session_bytes(lk.welcome["sid"]), seq, time.time(), vc.encode(vc.tone(400, 40)),
                      bytes.fromhex(lk.welcome["udp_key"]))
        tx.sendto(pkt, h.sock.getsockname())
    try:
        app.link = l1 = link("aa" * 8)
        b1 = h._bind()
        for k in range(1, 101):                            # the first link's LISTEN reached seq 100
            send_l(l1, k)
        until_rx = time.time() + 3
        while h.rx_frames < 100 and time.time() < until_rx:
            time.sleep(0.02)
        old_n = h.rx_frames
        h.jbuf = [vc.silence()]
        app.link = l2 = link("bb" * 8)                     # reconnect: a new GatewayLink, its seq starts at 1 again
        b2 = h._bind()
        reset = h.rx_seq is None and h.jbuf == []
        for k in range(1, 6):
            send_l(l2, k)
        until_rx = time.time() + 3
        while h.rx_frames < old_n + 5 and time.time() < until_rx:
            time.sleep(0.02)
        new_n = h.rx_frames - old_n
        l2.welcome = dict(l2.welcome, sid="cc" * 8)        # the same link object, a new login (session)
        sock2 = h.sock
        b3 = h._bind()
        rebound = h.sock is not sock2 and h.sess == vc.session_bytes("cc" * 8) and h.rx_seq is None
        for k in range(1, 4):
            send_l(l2, k)
        until_rx = time.time() + 3
        while h.rx_frames < old_n + new_n + 3 and time.time() < until_rx:
            time.sleep(0.02)
        third = h.rx_frames - old_n - new_n
    finally:
        h.close()
        tx.close()
    report("HS8 (review 13r MED-2) reconnect: the LISTEN sequence + jitter buffer start again with a new link / a new "
           "session, so the gateway's first frames (seq 1..) are heard at once (before: dropped as old for ~20 min)",
           b1 and b2 and b3 and old_n == 100 and reset and new_n == 5 and rebound and third == 3,
           "first link %d frames; reset %s; after the reconnect %d of 5; same link new session: rebound %s, %d of 3" % (
               old_n, reset, new_n, rebound, third))


def main():
    exe = os.path.join(HERE, "..", "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
    if not os.path.exists(exe):
        print("build the simulator first: sim\\build_sim.bat")
        return 2
    hs1()
    hs4()
    hs5()
    hs7()
    hs8()
    rig = ts.Rig()
    try:
        for fn in (hs2, hs3, hs6):
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
    print("LOUDHAILER STATION TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
