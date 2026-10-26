"""Boats + two screens (CLAUDE.md sections E + F + H): boats/ list, online / offline,
name check, the confirm step before HELM, screen 1 / 2 on one monitor, the map
inside screen 2.

    python test_boats.py            (tools/check.sh runs it)
"""
import json
import os
import socket
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
os.environ.setdefault("TRYGONS_LANG", "en")
os.environ.setdefault("TRYGONS_SOUND", "off")
os.environ.setdefault("TRYGONS_GS_SETTINGS", os.path.join(tempfile.mkdtemp(prefix="gs_boats_"), "gs_settings.json"))
os.environ.setdefault("TRYGONS_HUD_LAYOUT", os.path.join(tempfile.mkdtemp(prefix="hud_boats_"), "hud_layout.json"))
sys.path.insert(0, HERE)
import boats  # noqa: E402

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


def b1_repo_boats():
    """The repository's own boats/: rf49d + er69d, no secrets, placeholder IP = not set."""
    bs = boats.load()
    names = [b.name for b in bs]
    secret_words = ("password", "pw", "key", "secret", "token")
    secrets = []
    for b in bs:
        with open(b.path, encoding="utf-8") as f:
            data = json.load(f)
        secrets += [k for k in data if any(w == k.lower() or k.lower().startswith(w) for w in secret_words)]
    placeholder = all(not b.ip_ok() and "PLACEHOLDER" in b.ip for b in bs)
    report("B1 boats/ in git: RF49D + ER69D, name + Tailscale IP (placeholder), no secrets",
           sorted(names) == ["ER69D", "RF49D"] and not secrets and all(b.port == 5761 for b in bs) and
           (placeholder or all(b.ip_ok() for b in bs)),
           "boats %s; placeholder IPs %s (shown as IP NOT SET); secret-looking keys %s" % (
               [(b.name, b.ip) for b in bs], placeholder, secrets or "none"))


def b2_list_and_probe():
    """A folder with an online boat, an offline one, one without IP, a broken file, extra fields."""
    d = tempfile.mkdtemp(prefix="boats_")
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(5)
    on_port = srv.getsockname()[1]
    files = {"aa.json": {"id": "aa", "name": "ALPHA", "tailscale_ip": "127.0.0.1", "gateway_port": on_port,
                         "length_m": 4.9, "cameras": ["FRONT"]},
             "bb.json": {"id": "bb", "name": "BRAVO", "tailscale_ip": "127.0.0.1", "gateway_port": free_port()},
             "cc.json": {"id": "cc", "name": "CHARLIE", "tailscale_ip": "PLACEHOLDER-100.x.y.z"}}
    for n, v in files.items():
        with open(os.path.join(d, n), "w", encoding="utf-8") as f:
            json.dump(v, f)
    with open(os.path.join(d, "zz.json"), "w", encoding="utf-8") as f:
        f.write("{broken")
    bs = boats.load(d)
    st = {b.name: boats.probe(b, timeout=1.0) for b in bs}
    pr = boats.Prober(bs, period_s=0.5).start()
    end = time.time() + 5
    while time.time() < end and any(pr.status(b) == "?" for b in bs):
        time.sleep(0.1)
    st2 = {b.name: pr.status(b) for b in bs}
    pr.stop()
    srv.close()
    alpha = [b for b in bs if b.name == "ALPHA"][0]
    report("B2 boat list: online / offline / IP not set / broken file; extra fields kept (format can grow)",
           st == {"ALPHA": "online", "BRAVO": "offline", "CHARLIE": "no_ip", "zz": "no_ip"} and st2 == st and
           alpha.extra == {"length_m": 4.9, "cameras": ["FRONT"]} and [b.name for b in bs if b.error] == ["zz"],
           "probe %s; background prober %s; extra %s" % (st, st2, alpha.extra))


def b3_name_and_confirm():
    b = boats.Boat("x.json", {"id": "rf49d", "name": "RF49D", "tailscale_ip": "100.101.102.103"})
    ok = boats.check_welcome(b, {"boat": "RF49D"}) is None
    bad = boats.check_welcome(b, {"boat": "ER69D"})
    c = [boats.confirmed("rf49d", b, True), boats.confirmed(" RF 49D", b, True), boats.confirmed("RF49D", b, False),
         boats.confirmed("ER69D", b, True), boats.confirmed("", b, True)]
    report("B3 name check (refused when the gateway is another boat); confirm = name typed + live camera",
           ok and bad and "wrong boat" in bad and c == [True, True, False, False, False],
           "right %s; wrong -> %r; confirm (name ok + camera, spaces, no camera, wrong name, empty) %s" % (ok, bad, c))


class FakeStream:
    def __init__(self, live):
        self.live = live

    def frozen(self):
        return not self.live

    def ppm(self):
        return None


def b4_app_end_to_end():
    """The real ground station + gateway + simulator: choose the boat, HELM refused until the name + live
    camera are confirmed; the wrong boat is refused; one monitor: F11 toggles screen 1 / 2; the map is in
    screen 2."""
    sys.path.insert(0, os.path.join(REPO, "gateway"))
    import test_gateway as TG
    import tkinter as tk
    import app as A
    import proto
    tmp = tempfile.mkdtemp(prefix="boats_e2e_")
    sim = TG.Sim(1.0)
    root_dir = os.path.join(tmp, "ssd")
    os.makedirs(root_dir)
    cfg = TG.gw_cfg_b(sim, root_dir, boat="RF49D")
    gw = TG.GwRunner(cfg)
    sp = cfg["stations"]["port"]
    good = boats.Boat("rf.json", {"id": "rf49d", "name": "RF49D", "tailscale_ip": "127.0.0.1", "gateway_port": sp})
    wrong = boats.Boat("er.json", {"id": "er69d", "name": "ER69D", "tailscale_ip": "127.0.0.1", "gateway_port": sp})
    troot = tk.Tk()
    ap = A.App(troot, layout=os.path.join(tmp, "lay.json"))

    def pump(s, until=None):
        end = time.time() + s
        while time.time() < end:
            troot.update()
            if until and until():
                return True
            time.sleep(0.02)
        return bool(until and until())
    try:
        pump(1.0)
        err_wrong = ap.connect_boat(wrong, "alice", TG.PW["alice"], "captain")
        err = ap.connect_boat(good, "alice", TG.PW["alice"], "captain")
        pump(8, lambda: ap.t is not None)
        conf_open = ap.boatconf is not None and ap.driver is not None
        ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
        pump(2.0)
        helm_before = ap.t.get("helm") if ap.t else None
        # confirm without a live camera / with the wrong name: refused
        ap.boatconf.name.insert(0, "RF49D")
        ap.boatconf.cam.set(True)
        ap.boatconf.ok()
        no_cam = not ap.boat_ok
        ap.vstreams = {"FRONT": FakeStream(True)}                # a live front picture arrives
        pump(1.0)
        ap.boatconf.name.delete(0, "end")
        ap.boatconf.name.insert(0, "ER69D")
        ap.boatconf.cam.set(True)
        ap.boatconf.ok()
        wrong_name = not ap.boat_ok
        ap.boatconf.name.delete(0, "end")
        ap.boatconf.name.insert(0, "rf49d")
        ap.boatconf.cam.set(True)
        ap.boatconf.ok()
        confirmed = ap.boat_ok and ap.boatconf is None
        ap.cmd(proto.T_HELM, proto.p_helm(0), "take HELM")
        me = ap.link.welcome["id"]
        took = pump(5, lambda: ap.t and ap.t.get("helm") == me)
        crew = A.crewui.crew_of(ap)[0]
        report("B4 choose the boat: wrong boat refused; HELM refused until its name + LIVE camera are confirmed",
               err_wrong and "wrong boat" in err_wrong and err is None and conf_open and helm_before != me and
               no_cam and wrong_name and confirmed and took and crew == "captain",
               "ER69D (gateway is RF49D) -> %r; RF49D -> connected %s, confirm dialog + screen 1 %s; HELM before the "
               "confirm: holder %s (not us %s); confirm without camera refused %s, wrong name refused %s, name + live "
               "camera -> confirmed %s; HELM taken %s; crew %s" % (
                   err_wrong, err is None, conf_open, helm_before, me, no_cam, wrong_name, confirmed, took, crew))
        # one monitor: F11 toggles screen 1 / 2; the map is a part of screen 2
        A.monitors.list_monitors = lambda root=None: [(0, 0, 1920, 1080)]
        ap.screen_shown = 2
        ap.toggle_screen()
        pump(0.3)
        s1 = ap.screen_shown == 1
        ap.toggle_screen()
        pump(0.3)
        s2 = ap.screen_shown == 2
        ap.open_map()
        pump(0.3)
        embedded = ap.mapwin is not None and ap.mapwin.cv is ap.cv and str(ap.cv.winfo_toplevel()) == str(ap.root) \
            and ap.screen_shown == 2
        report("B5 one monitor: F11 shows screen 1, F11 again screen 2; the map IS screen 2 (no map window)",
               s1 and s2 and embedded, "screen 1 %s, screen 2 %s, the map fills screen 2 %s" % (s1, s2, embedded))
        b6_dropdown(ap, pump, good)
    finally:
        ap.quit()
        gw.stop()
        sim.stop()


def b6_dropdown(ap, pump, good):
    """CLAUDE.md H: the boat dropdown (screen 2 status panel, first line) + the lamp. CONNECTED only when the
    boat's gateway was found by the prober AND the station is connected to it AND the welcome name check
    passed; otherwise grey with the reason. ap is connected to `good` (B4) through a real gateway."""
    import theme
    T = __import__("i18n").T

    def lamp():
        ap.render(force=True)
        dd = [o["s"] for o in ap.ov.ops if o.get("tag") == "boat_dd"]
        lp = [(o["s"], o.get("fill")) for o in ap.ov.ops if o.get("tag") == "boat_lamp"]
        return (dd[0] if dd else None), (lp[0] if lp else None), ap.boat_state()[1]
    st = {}
    st["connected"] = lamp()                                     # the real gateway, name check passed
    w = dict(ap.link.welcome)
    ap.link.welcome["boat"] = "ER69D"                            # the welcome name check fails
    st["wrong_name"] = lamp()
    ap.link.welcome.clear()
    ap.link.welcome.update(w)
    ap.prober.result[good.id] = ("offline", time.time())         # the prober can't find the gateway
    st["not_found"] = lamp()
    ap.prober.found(good)
    st["again"] = lamp()
    # the other boats of a folder: online (not connected), offline, IP not set
    d = tempfile.mkdtemp(prefix="boats_dd_")
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(5)
    for n, v in {"aa": {"name": "ALPHA", "tailscale_ip": "127.0.0.1", "gateway_port": srv.getsockname()[1]},
                 "bb": {"name": "BRAVO", "tailscale_ip": "127.0.0.1", "gateway_port": free_port()},
                 "cc": {"name": "CHARLIE", "tailscale_ip": "PLACEHOLDER-100.x.y.z"}}.items():
        with open(os.path.join(d, n + ".json"), "w", encoding="utf-8") as f:
            json.dump(dict(v, id=n), f)
    ap.boat_list = boats.load(d)
    ap.prober.stop()
    ap.prober = boats.Prober(ap.boat_list, period_s=0.5).start()
    pump(5, lambda: all(ap.prober.status(b) != "?" for b in ap.boat_list))
    menu = [e[0].split("    ")[:2] for e in ap.boat_menu_entries() if e is not None][:3]
    picked = {}
    for b in ap.boat_list:
        ap.disconnect()                                          # not connected: only the prober's view
        ap.boat_pick = b
        picked[b.name] = lamp()
    ap.boat_pick = None
    st["none"] = lamp()
    srv.close()
    G, grey = theme.GOOD, theme.C["head"]
    ok = st["connected"][1] == ("● " + T("ov.connected"), G) and st["connected"][0].startswith("RF49D") and \
        st["wrong_name"][1][1] == grey and st["not_found"][1] == ("● " + T("boat.st_offline"), grey) and \
        st["again"][1][1] == G and \
        picked["ALPHA"][1] == ("● " + T("boat.st_online"), grey) and \
        picked["BRAVO"][1] == ("● " + T("boat.st_offline"), grey) and \
        picked["CHARLIE"][1] == ("● " + T("boat.st_no_ip"), grey) and \
        st["none"][1] is None and st["none"][0].startswith(T("ov.choose_boat")) and \
        menu == [["ALPHA", T("boat.st_online")], ["BRAVO", T("boat.st_offline")], ["CHARLIE", T("boat.st_no_ip")]]
    report("B6 boat dropdown + lamp: CONNECTED (green) only when found + connected + name check passed; else grey "
           "(name check failed, not found, online but not connected, offline, IP not set); nothing chosen = no lamp",
           ok, "connected %s; wrong name %s; not found %s; found again %s; picked %s; nothing chosen %s; menu %s" % (
               st["connected"][1:], st["wrong_name"][1:], st["not_found"][1:], st["again"][1:],
               {k: v[1] for k, v in picked.items()}, st["none"][:2], menu))


def main():
    b1_repo_boats()
    b2_list_and_probe()
    b3_name_and_confirm()
    b4_app_end_to_end()
    bad = results.count(False)
    print("BOAT TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
