"""The UAV magazine on the ground station (CLAUDE.md V, step U2; DISPLAY + requests only).

  UV1 the owner's UAV picture: assets/uav.png + the small sizes (tools/make_uav_icons.py: 16 / 24 / 32 / 48 / 64 px,
      square, transparent corners), uavpics picks the size, Tk loads it; the USV placeholder draws (W65)
  UV2 a slot row's texts (pure): no cable, empty, not registered, reading, ready, NOT launchable + the gateway's
      reason in Greek, ARMED, the NEXT marker on slot 1; no English word left in the Greek rows
  UV3 the Tools page «ΜΑΓΓΕΜΑ UAV» in the real station: the page exists, 6 slot rows with the picture, filled from
      gw_status "uav", REGISTER sends {"t": "uav_register", "slot"} only (nothing to the AURIX), disabled while not
      connected / without the magazine; the station hooks gw_status "uav" + "uav_ack"
  UV4 uav_ack: the answer and every problem shown in the screen language (from_en templates gw.uav_*)

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


def main():
    uv1()
    uv2()
    rig = ts.Rig()
    try:
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
