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
    report("B2 boat list: online / offline / IP not set / broken file = INVALID; extra fields kept (format can grow)",
           st == {"ALPHA": "online", "BRAVO": "offline", "CHARLIE": "no_ip", "zz": "invalid"} and st2 == st and
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


V2 = {"format": 2, "id": "v2", "name": "VICTOR", "tailscale_ip": "127.0.0.1", "gateway_port": 5761,
      "generator_max_pct": 40, "length_m": 5.2, "draft_m": 0.6,
      "relays": [{"n": 1, "name_el": "ΑΝΑΦΛΕΞΗ", "name_en": "IGNITION", "type": "onoff", "group": "engine",
                  "pair": None, "max_s": 0, "show_hud": True, "power_up": "off", "role": "ignition"},
                 {"n": 2, "name_el": "ΜΙΖΑ", "name_en": "STARTER", "type": "momentary", "group": "engine",
                  "pair": None, "max_s": 1, "show_hud": False, "power_up": "off", "role": "starter"},
                 {"n": 4, "name_el": "ΤΡΙΜ ΠΑΝΩ", "name_en": "TRIM UP", "type": "momentary", "group": "trim",
                  "pair": 5, "max_s": 3, "show_hud": True, "power_up": "off"},
                 {"n": 5, "name_el": "ΤΡΙΜ ΚΑΤΩ", "name_en": "TRIM DOWN", "type": "momentary", "group": "trim",
                  "pair": 4, "max_s": 3, "show_hud": True, "power_up": "off"},
                 {"n": 7, "name_el": "ΑΝΤΛΙΑ", "name_en": "PUMP", "type": "timed", "group": "extra", "pair": None,
                  "max_s": 120, "power_up": "keep"}],
      "thrusters": {"bow": {"max_pct": 80, "run_s": 60, "rest_s": 120}, "stern": {"max_pct": 100, "run_s": 120,
                                                                                   "rest_s": 90}, "jog_m": 2.0}}


def b7_file_formats():
    """CLAUDE.md L: a v1 file (no "format") loads with defaults; a v2 file gives the generator cap, the relay
    table and the thruster settings; every bad file is INVALID (listed with its error, never connected)."""
    import copy
    d = tempfile.mkdtemp(prefix="boats_v2_")

    def bad(mut):
        x = copy.deepcopy(V2)
        mut(x)
        return x

    def relay_mut(i, **kw):
        def f(x):
            x["relays"][i].update(kw)
        return f
    files = {
        "v1.json": {"id": "v1", "name": "OLD", "tailscale_ip": "127.0.0.1", "generator_max_pct": 50,
                    "relays": [{"n": 1}], "length_m": 4.0},                 # v1: the v2 fields are ignored
        "v2.json": V2,
        "f3.json": bad(lambda x: x.__setitem__("format", 3)),               # unknown format
        "gen_txt.json": bad(lambda x: x.__setitem__("generator_max_pct", "25")),
        "gen_hi.json": bad(lambda x: x.__setitem__("generator_max_pct", 101)),
        "pair.json": bad(relay_mut(2, pair=1)),                             # TRIM UP pairs IGNITION: not mutual
        "pair_self.json": bad(relay_mut(2, pair=4)),
        "mom6.json": bad(relay_mut(1, max_s=6)),                            # momentary longer than 5 s
        "mom0.json": bad(relay_mut(1, max_s=0)),
        "timed0.json": bad(relay_mut(4, max_s=0)),
        "eng_on.json": bad(relay_mut(0, power_up="on")),                    # engine relay switched automatically
        "eng_keep.json": bad(relay_mut(0, power_up="keep")),
        "type.json": bad(relay_mut(0, type="pulse")),
        "group.json": bad(relay_mut(0, group="lights")),
        "dup_n.json": bad(relay_mut(1, n=1)),
        "no_name.json": bad(relay_mut(0, name_el="")),
        "thr_hi.json": bad(lambda x: x["thrusters"]["bow"].__setitem__("max_pct", 101)),
        "thr_run.json": bad(lambda x: x["thrusters"]["bow"].__setitem__("run_s", 0)),
        "thr_jog.json": bad(lambda x: x["thrusters"].__setitem__("jog_m", 0)),
        "relays_obj.json": bad(lambda x: x.__setitem__("relays", {"n": 1})),
        # step 7b: the engine roles (the STARTER is a role, never just a type)
        "starter_onoff.json": bad(relay_mut(1, type="onoff", max_s=0)),     # the STARTER typed on / off
        "starter_timed.json": bad(relay_mut(1, type="timed", max_s=2)),
        "eng_no_role.json": bad(relay_mut(0, role=None)),                   # an engine relay without a role
        "eng_bad_role.json": bad(relay_mut(0, role="horn")),
        "role_extra.json": bad(relay_mut(4, role="ignition")),              # a role outside the engine group
        "two_starters.json": bad(relay_mut(0, role="starter", type="momentary", max_s=1)),
        "ign_momentary.json": bad(relay_mut(0, type="momentary", max_s=1)),
        "named_starter.json": bad(relay_mut(4, name_en="STARTER", type="momentary", max_s=1)),  # extra "STARTER"
    }
    for n, v in files.items():
        with open(os.path.join(d, n), "w", encoding="utf-8") as f:
            json.dump(v, f, ensure_ascii=False)
    with open(os.path.join(d, "broken.json"), "w", encoding="utf-8") as f:
        f.write("{broken")
    bs = {os.path.basename(b.path): b for b in boats.load(d)}
    v1, v2 = bs["v1.json"], bs["v2.json"]
    v1_ok = v1.error is None and v1.format == 1 and v1.generator_max_pct == 0 and v1.relays == [] and \
        v1.thrusters["bow"]["max_pct"] == 0 and v1.thrusters["stern"]["max_pct"] == 0 and v1.length_m == 4.0 and \
        v1.fixed_status() == None and boats.aurix_settings(v1) == {"gen_max_pct": 0.0, "relays": [],
                                                                    "thrusters": boats.aurix_settings(v1)["thrusters"]}
    s2 = boats.aurix_settings(v2)
    v2_ok = v2.error is None and v2.format == 2 and v2.generator_max_pct == 40 and s2["gen_max_pct"] == 30 and \
        boats.aurix_settings(v2, 20)["gen_max_pct"] == 20 and [r["n"] for r in v2.relays] == [1, 2, 4, 5, 7] and \
        v2.relay(4)["pair"] == 5 and v2.relay(5)["pair"] == 4 and v2.relay(7)["show_hud"] is False and \
        v2.relay(7)["power_up"] == "keep" and v2.relay(2)["max_s"] == 1.0 and v2.relay(2)["role"] == "starter" and \
        v2.relay(1)["role"] == "ignition" and v2.relay(7)["role"] is None and \
        v2.thrusters["bow"] == {"max_pct": 80.0, "run_s": 60.0, "rest_s": 120.0} and v2.thrusters["jog_m"] == 2.0 and \
        v2.thrusters["spot_alarm_m"] == 15.0 and v2.length_m == 5.2 and v2.draft_m == 0.6 and \
        s2["relays"] == v2.relays and s2["relays"] is not v2.relays and v2.fixed_status() is None
    invalid = {n: (b.error, b.fixed_status(), boats.probe(b)) for n, b in bs.items() if n not in ("v1.json", "v2.json")}
    inv_ok = all(e and st == "invalid" and pr == "invalid" for e, st, pr in invalid.values()) and \
        len(invalid) == len(files) - 2 + 1
    # an invalid file pushes the defaults (never its contents) and is never connected
    bad_b = bs["eng_on.json"]
    defaults = boats.aurix_settings(bad_b) == boats.aurix_settings(v1)
    repo = boats.load()
    repo_ok = all(b.error is None and b.format == 2 and 0 < boats.aurix_settings(b)["gen_max_pct"] <= 30 and
                  b.relays and all(r["power_up"] == "off" for r in b.relays if r["group"] == "engine") and
                  [r["role"] for r in b.relays if r["group"] == "engine"] == ["ignition", "starter", "shutoff"]
                  for b in repo)
    report("B7 boat file v2: v1 loads with defaults (generator off, no relays / thrusters); v2 parsed, generator "
           "capped at the AURIX ceiling; %d bad files INVALID (format, generator, pair not mutual, momentary > 5 s, "
           "engine power_up, type, group, duplicate n, thrusters, engine roles, broken JSON); repo files valid v2"
           % len(invalid),
           v1_ok and v2_ok and inv_ok and defaults and repo_ok,
           "v1 %s; v2 %s (gen 40 -> push %s); invalid %s; bad file = defaults %s; repo %s" % (
               v1_ok, v2_ok, s2["gen_max_pct"], {n: v[0] for n, v in invalid.items()}, defaults,
               [(b.name, b.format, b.error) for b in repo]))


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
    good = boats.Boat("rf.json", {"format": 2, "id": "rf49d", "name": "RF49D", "tailscale_ip": "127.0.0.1",
                                  "gateway_port": sp, "generator_max_pct": 20})
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
        pushed_before_helm = ap.boat_pushed                      # B8: nothing pushed without HELM
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
        b8_push(ap, pump, pushed_before_helm)
        b9_relay_push(ap, pump, sim)
        b6_dropdown(ap, pump, good)
    finally:
        ap.quit()
        gw.stop()
        sim.stop()


def b8_push(ap, pump, pushed_before_helm):
    """CLAUDE.md L: once this station holds HELM (STANDBY), the boat file's generator_max_pct reaches the AURIX
    (parameter gen_max_pct, RAM only); the AURIX's own ceiling (30) wins over a bigger file value; nothing is
    pushed outside STANDBY; a NACK / restart re-arms the push."""
    pid = next((k for k, p in ap.params.items() if p.get("name") == "gen_max_pct"), None)
    val = lambda: ap.params.get(pid, {}).get("value")
    pushed = pid is not None and pump(8, lambda: ap.boat_pushed and val() == 20.0)
    v20 = val()
    # outside STANDBY: not pushed (PARAM_SET is STANDBY only)
    ap.boat_pushed, ap.boat_push_at = False, 0.0
    seq0 = ap.boat_push_seq
    ap.boat_push(dict(ap.t, mode_name="MANUAL"))
    held = not ap.boat_pushed and ap.boat_push_seq == seq0
    # the file says 40: the AURIX ceiling (parameter max 30) wins
    ap.boat.generator_max_pct = 40.0
    ap.boat_push(ap.t)
    capped = pump(8, lambda: val() == 30.0)
    report("B8 boat file -> AURIX: generator max pushed once HELM is held (STANDBY only), capped at the AURIX "
           "ceiling; nothing before HELM",
           not pushed_before_helm and pushed and held and capped,
           "before HELM pushed %s; after HELM gen_max_pct = %s (file 20) %s; MANUAL: held %s; file 40 -> %s (max 30) %s"
           % (pushed_before_helm, v20, pushed, held, val(), capped))


def b9_relay_push(ap, pump, sim):
    """CLAUDE.md M + owner clarification (step 6): the boat file's relay table reaches the AURIX (PROTO_T_RELAY_CFG)
    once HELM is held in STANDBY; a table that differs from the AURIX's stored one waits there for the joystick
    CONFIRM (TELEM2 group M "relay_pending") and only then becomes active (its hash = the file's); the same table is
    never sent again; a file without relays (v1) = the empty table = nothing to send to a boat without a table."""
    import proto
    t2 = lambda: ap.link.t2 if ap.link is not None else {}
    empty = t2().get("relay_hash") == proto.relay_table_hash([]) and not t2().get("relay_table")
    seq0 = ap.relay_push_seq
    nothing_for_empty = seq0 is None                             # the file had no relays: nothing sent
    ap.boat.relays = boats._parse_relays([
        {"n": 1, "name_el": "ΑΝΑΦΛΕΞΗ", "name_en": "IGNITION", "type": "onoff", "group": "engine", "role": "ignition"},
        {"n": 4, "name_el": "TRIM ΠΑΝΩ", "name_en": "TRIM UP", "type": "momentary", "group": "trim", "pair": 5,
         "max_s": 3},
        {"n": 5, "name_el": "TRIM ΚΑΤΩ", "name_en": "TRIM DOWN", "type": "momentary", "group": "trim", "pair": 4,
         "max_s": 3},
        {"n": 6, "name_el": "ΦΩΤΑ", "name_en": "NAV LIGHTS", "type": "onoff", "group": "extra", "power_up": "keep"}])
    want = proto.relay_table_hash(ap.boat.relays)
    # step 7b: never while another CONFIRM is pending on the boat; never confirmed by itself in single-operator mode
    seq_a = ap.relay_push_seq
    ap.relay_push_at = 0.0
    ap.relay_push({"relay_hash": want ^ 1, "relay_pending": False, "confirm_kind": 1})
    skip_confirm = ap.relay_push_seq == seq_a
    single, ap.single_op = ap.single_op, True
    ap.relay_push_at = 0.0
    ap.relay_push({"relay_hash": want ^ 1, "relay_pending": False, "confirm_kind": 0})
    asked = ap.cconf is not None and ap.relay_push_seq == seq_a          # the operator's question, nothing sent
    if ap.cconf is not None:
        ap.cconf.destroy()
        ap.cconf = None
    ap.single_op = single
    ap.relay_push_at = 0.0
    pending = pump(8, lambda: t2().get("relay_pending") and ap.t and ap.t.get("confirm_kind") == 7)
    active_before = t2().get("relay_table")
    link, _w = sim.station()                                     # the person on board: joystick CONFIRM
    try:
        time.sleep(0.5)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0, confirm=True))
        time.sleep(0.3)
        link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
        active = pump(8, lambda: t2().get("relay_table") and t2().get("relay_hash") == want and
                      t2().get("relay_count") == 4 and not t2().get("relay_pending"))
    finally:
        link.stop()
    seq1 = ap.relay_push_seq
    ap.relay_push_at = 0.0
    pump(2.0)
    no_resend = ap.relay_push_seq == seq1                        # the same table: never sent again
    report("B9 boat file relay table -> AURIX: sent with HELM in STANDBY, waits for the joystick CONFIRM, then "
           "active (hash = the file's); the same table never re-sent; no relays = nothing to send",
           empty and nothing_for_empty and skip_confirm and asked and pending and not active_before and active and
           no_resend,
           "empty table on the boat %s, nothing sent for a file without relays %s; not sent while another CONFIRM is "
           "pending %s; single operator: asked, not self-confirmed %s; 4 relays sent -> CONFIRM pending %s "
           "(not active yet %s); after CONFIRM active, hash %s = file %s: %s; no re-send %s" % (
               empty, nothing_for_empty, skip_confirm, asked, pending, not active_before, t2().get("relay_hash"), want,
               active, no_resend))


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
    b7_file_formats()
    b4_app_end_to_end()
    bad = results.count(False)
    print("BOAT TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
