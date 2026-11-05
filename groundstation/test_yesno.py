"""YES / NO answers on the shore joystick (step 13p, owner 2026-10-31, docs/owner_findings_2026-10-31.md D).
Pure logic - no window, no boat (groundstation/yesno.py, shorejoy.controls, gateway/stations.py's trigger mask).

  YN1 trigger = ACCEPT of the selection (YES pre-selected): only a FRESH press made >= 0.5 s after the question
      appeared - a press held from before, or made within 0.5 s, never answers (it needs a new press)
  YN2 the thumb button = a quick REJECT (fresh press, the same guard); the hat up / down selects YES / NO in EVERY
      mode (also MANEUVER / REMOTE / a station without a lever)
  YN3 the stick Y selects (forward = YES, back = NO, > 40 %, a fresh move from the centre) ONLY at a lever station in
      STANDBY / AUTO / HDG LOCK / DP; never in MANEUVER (= the gear), REMOTE, MANUAL, without a lever (= throttle)
  YN4 one press never does two things: the trigger is never sent to the boat while a question is open, nor the press
      that answered (until released), nor a press held into a new question; with no question it goes as before
  YN5 shorejoy.controls: the default map (trigger 1, thumb 2, SAVE 4, CONFIRM TWCS 4), the hat (Windows POV, Linux
      raw axis), the stick Y only at a lever station, a button sharing another's mapping is not used
  YN6 the gateway keeps a trigger back while it has a question open for that station (the captain's ACCEPT of a
      copilot action, an avoidance / vision proposal, the slamming restore) and until the press is released
  YN7 (step WA, owner W53) the settings questions (strict channel, emergency, trip test, HELM release UNCREWED,
      single operator, crew swap) = ynbox.ask in the vertical layout: question, YES on top, NO underneath, full width,
      NO pre-selected, Up / Down / Enter / Esc; no OS message box left in the station's code
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import proto  # noqa: E402
import shorejoy  # noqa: E402
import yesno  # noqa: E402

results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def ctl(trigger=False, reject=False, hat=0, y=0.0, lever=True):
    return {"trigger": trigger, "reject": reject, "hat": hat, "y": y, "lever": lever}


def yn1():
    s = yesno.Selector()
    S = proto.MODE_STANDBY
    s.feed(ctl(trigger=True), 0.0, S)                     # held from before the question
    s.question("q1", 0.0)
    held = [s.feed(ctl(trigger=True), t, S) for t in (0.1, 0.6, 1.0)]
    s.feed(ctl(), 1.1, S)                                 # released
    ok_after = s.feed(ctl(trigger=True), 1.2, S)
    s.question("q2", 2.0)
    s.feed(ctl(), 2.05, S)
    early = s.feed(ctl(trigger=True), 2.3, S)             # within 0.5 s: ignored, and it stays ignored while held
    still = s.feed(ctl(trigger=True), 2.8, S)
    s.feed(ctl(), 2.9, S)
    late = s.feed(ctl(trigger=True), 3.0, S)
    report("YN1 trigger = ACCEPT of the selection (YES pre-selected): a fresh press >= 0.5 s after the question; held "
           "from before / within 0.5 s never answers",
           held == [None, None, None] and ok_after == ("answer", True) and early is None and still is None and
           late == ("answer", True) and s.guarded == 1,
           "held from before %s; fresh after release %s; within 0.5 s %s, still held %s, new press %s; guarded %d" % (
               held, ok_after, early, still, late, s.guarded))


def yn2():
    out = {}
    for name, mode, lever in (("MANEUVER", proto.MODE_MANEUVER, True), ("REMOTE", proto.MODE_REMOTE, True),
                              ("no lever", proto.MODE_AUTO, False), ("AUTO", proto.MODE_AUTO, True)):
        s = yesno.Selector()
        s.question("q", 0.0)
        a = s.feed(ctl(hat=-1, lever=lever), 0.1, mode)
        s.feed(ctl(hat=0, lever=lever), 0.2, mode)
        b = s.feed(ctl(hat=1, lever=lever), 0.3, mode)
        s.feed(ctl(hat=-1, lever=lever), 0.4, mode)
        c = s.feed(ctl(trigger=True, hat=-1, lever=lever), 0.7, mode)
        out[name] = (a, b, c)
    s = yesno.Selector()
    s.question("q", 0.0)
    early = s.feed(ctl(reject=True), 0.2, proto.MODE_STANDBY)
    s.feed(ctl(), 0.3, proto.MODE_STANDBY)
    rej = s.feed(ctl(reject=True), 0.6, proto.MODE_STANDBY)
    want = (("select", False), ("select", True), ("answer", False))
    report("YN2 thumb = quick REJECT (fresh, 0.5 s guard); the hat selects YES / NO in every mode",
           all(v == want for v in out.values()) and early is None and rej == ("answer", False),
           "hat %s; thumb within 0.5 s %s, after %s" % (out, early, rej))


def yn3():
    got = {}
    for name in ("STANDBY", "AUTO", "HDG_LOCK", "DP", "MANEUVER", "REMOTE", "MANUAL"):
        mode = getattr(proto, "MODE_" + name)
        for lever in (True, False):
            s = yesno.Selector()
            s.question("q", 0.0)
            s.feed(ctl(y=0.0, lever=lever), 0.05, mode)   # centred first
            a = s.feed(ctl(y=-0.6, lever=lever), 0.1, mode)
            b = s.feed(ctl(y=-0.9, lever=lever), 0.15, mode)  # no fresh move: nothing
            s.feed(ctl(y=0.05, lever=lever), 0.2, mode)
            c = s.feed(ctl(y=0.35, lever=lever), 0.25, mode)  # below 40 %: nothing
            d = s.feed(ctl(y=0.5, lever=lever), 0.3, mode)
            got[(name, lever)] = (a, b, c, d)
    on = {k for k, v in got.items() if v == (("select", False), None, None, ("select", True))}
    off = {k for k, v in got.items() if v == (None, None, None, None)}
    want_on = {(m, True) for m in ("STANDBY", "AUTO", "HDG_LOCK", "DP")}
    s = yesno.Selector()                                   # held at the question's start: needs the centre first
    s.feed(ctl(y=-0.8), 0.0, proto.MODE_AUTO)
    s.question("q", 0.0)
    held = s.feed(ctl(y=-0.8), 0.1, proto.MODE_AUTO)
    report("YN3 the stick Y selects (> 40 %, fresh from the centre) only at a lever station in STANDBY / AUTO / HDG "
           "LOCK / DP; never MANEUVER / REMOTE / MANUAL / without a lever",
           on == want_on and off == set(got) - want_on and held is None,
           "selects in %s; nothing in %d other cases; held from before: %s" % (sorted(on), len(off), held))


def yn4():
    s = yesno.Selector()
    S = proto.MODE_STANDBY
    free = s.send_trigger(True)                            # no question: the boat gets it (13q3 new heading)
    s.send_trigger(False)
    s.question("q", 0.0)
    s.feed(ctl(), 0.1, S)
    open_q = s.send_trigger(True)                          # pressed while the question is open
    ans = s.feed(ctl(trigger=True), 0.7, S)                # (this press: inside the guard? no - 0.7 s: answers)
    s.question(None, 0.75)
    after = [s.send_trigger(True) for _ in range(3)]       # still held after the answer
    s.send_trigger(False)
    s.feed(ctl(), 0.8, S)
    again = s.send_trigger(True)                           # a new press, no question
    s.send_trigger(False)
    # the Tk loop answers BEFORE the sender thread ever saw the press: still never sent
    s2 = yesno.Selector()
    s2.question("q", 0.0)
    s2.feed(ctl(), 0.1, S)
    s2.feed(ctl(trigger=True), 0.7, S)
    s2.question(None, 0.7)
    race = s2.send_trigger(True)
    # held INTO a new question: kept back from then on
    s3 = yesno.Selector()
    s3.feed(ctl(trigger=True), 0.0, S)
    first = s3.send_trigger(True)
    s3.question("q", 0.1)
    into = s3.send_trigger(True)
    report("YN4 one press never does two things: no trigger to the boat while a question is open / for the press "
           "that answered (until released) / held into a question; free otherwise",
           free and not open_q and ans == ("answer", True) and after == [False] * 3 and again and not race and
           first and not into,
           "no question %s; question open %s; answered %s; held after %s; new press %s; answered before the sender "
           "saw it %s; held into a question %s -> %s" % (free, open_q, ans, after, again, race, first, into))


class FakeJs:
    def __init__(self):
        self.dev = {0: {"ok": True, "axes": {"X": 0.0, "Y": 0.0}, "buttons": 0, "pov": None},
                    1: {"ok": True, "axes": {"Z": -1.0}, "buttons": 0, "pov": None}}

    def read(self, dev):
        return self.dev.get(dev, {"ok": False, "axes": {}, "buttons": 0})


def yn5():
    cfg = {k: (dict(v) if isinstance(v, dict) else v) for k, v in shorejoy.DEFAULT.items()}
    cfg["steer"], cfg["thr"] = dict(cfg["steer"]), dict(cfg["thr"])
    js = FakeJs()
    dflt = (cfg["trigger_button"] == {"dev": 0, "button": 1} and cfg["reject_button"] == {"dev": 0, "button": 2} and
            cfg["save_button"] == {"dev": 0, "button": 4} and cfg["confirm_button"] == {"dev": 1, "button": 4})
    js.dev[0]["buttons"] = 1 | 2
    js.dev[0]["axes"]["Y"] = -0.7                          # pushed forward (inverted axis) = +0.7
    js.dev[0]["pov"] = 18000
    c = shorejoy.controls(js, cfg)
    win = c == {"trigger": True, "reject": True, "hat": -1, "y": 0.7, "lever": True}
    js.dev[0]["pov"] = 0
    up = shorejoy.controls(js, cfg)["hat"] == 1
    lin = dict(js.dev[0], pov=None, raw={5: -1.0})
    js.dev[0] = lin
    lin_up = shorejoy.controls(js, cfg)["hat"] == 1
    lin["raw"] = {5: 0.0}
    lin_c = shorejoy.controls(js, cfg)["hat"] == 0
    cfg["thr"]["lever"] = False
    nolev = shorejoy.controls(js, cfg)
    cfg["thr"]["lever"] = True
    cfg["reject_button"] = {"dev": 1, "button": 4}         # = the CONFIRM button: never a REJECT
    js.dev[1]["buttons"] = 8
    shared = shorejoy.controls(js, cfg)["reject"] is False and shorejoy.reject_conflict(cfg)
    cfg["reject_button"] = {"dev": 0, "button": 2}
    cfg["trigger_button"] = {"dev": 0, "button": 2}        # = the thumb: neither is used
    both = shorejoy.controls(js, cfg)
    tconf = both["trigger"] is False and both["reject"] is False and shorejoy.trigger_conflict(cfg)
    report("YN5 shorejoy.controls: default map (trigger 1, thumb 2, SAVE 4, CONFIRM TWCS 4), hat (Windows POV / Linux "
           "axis), stick Y only with a lever, a shared button never answers",
           dflt and win and up and lin_up and lin_c and nolev["y"] is None and not nolev["lever"] and shared and tconf,
           "defaults %s; Windows %s, POV up %s; Linux up %s / centred %s; no lever y %s; thumb = CONFIRM -> %s; "
           "trigger = thumb -> %s" % (dflt, c, up, lin_up, lin_c, nolev["y"], shared, tconf))


def yn6():
    sys.path.insert(0, os.path.join(HERE, "..", "gateway"))
    import stations as gst                                 # noqa: E402  (the gateway's own module)
    sent = []

    class Aurix:
        def send(self, ftype, payload):
            sent.append((ftype, payload))

    gw = types.SimpleNamespace(telem2={"helm": 5}, aurix=Aurix(), avoider=types.SimpleNamespace(proposal=None),
                               vision=types.SimpleNamespace(proposal=None))
    srv = gst.StationServer.__new__(gst.StationServer)
    srv.gw, srv.asks = gw, {}
    st = types.SimpleNamespace(id=5, role="CAPTAIN", crew="captain", single_op=False, rtt_ms=40,
                               offset=lambda: 0.0)
    seq = [0]

    def pkt(trig):
        seq[0] += 1
        import time
        srv._rjoy(st, {"seq": seq[0], "ts": time.time(), "x": 0.0, "y": 0.0, "lever": 0.5, "trigger": trig})
        return bool(sent[-1][1][-1] & 8)

    free = pkt(True)
    pkt(False)
    srv.asks = {1: {"id": 1, "target": 5}}
    ask = pkt(True)
    srv.asks = {}
    held = pkt(True)                                       # the same press, the question gone: still kept back
    pkt(False)
    other = {}
    for name, setup in (("avoid", lambda: setattr(gw.avoider, "proposal", {"id": 3})),
                        ("vision", lambda: setattr(gw.vision, "proposal", {"id": 4})),
                        ("slam", lambda: gw.telem2.update(slam_ask=True))):
        setup()
        other[name] = pkt(True)
        gw.avoider.proposal = gw.vision.proposal = None
        gw.telem2.pop("slam_ask", None)
        pkt(False)
    gw.vision.proposal = {"id": 4, "acked": True}
    acked = pkt(True)
    pkt(False)
    gw.vision.proposal = None
    srv.asks = {1: {"id": 1, "target": 9}}                 # another station's question
    other_st = pkt(True)
    pkt(False)
    report("YN6 the gateway keeps the trigger back while this station has a question open (ACCEPT, avoidance / "
           "vision proposal, slamming restore) and until released; free otherwise",
           free and not ask and not held and not any(other.values()) and acked and other_st and
           st.rjoy_trig_masked == 5,
           "free %s; with an ACCEPT question %s, still held %s; %s; an acknowledged vision note %s; another station's "
           "question %s; masked %d" % (free, ask, held, other, acked, other_st, st.rjoy_trig_masked))


def yn7():
    """Owner W53: the OS-native YES / NO boxes became the ONE vertical layout (ynbox.ask)."""
    import re
    import tkinter as tk
    os.environ.setdefault("TRYGONS_LANG", "en")
    import ynbox
    left = []
    for fn in sorted(os.listdir(HERE)):
        if fn.endswith(".py") and not fn.startswith("test_"):
            src = open(os.path.join(HERE, fn), encoding="utf-8").read()
            if re.search(r"messagebox[.]ask(yesno|okcancel|question|yesnocancel)[(]", src):
                left.append(fn)
    app_src = open(os.path.join(HERE, "app.py"), encoding="utf-8").read()
    crew_src = open(os.path.join(HERE, "crewui.py"), encoding="utf-8").read()
    calls = len(re.findall(r"ynbox[.]ask[(]", app_src)) + len(re.findall(r"ynbox[.]ask[(]", crew_src))
    res = {}
    try:
        root = tk.Tk()
    except tk.TclError as e:
        report("YN7 settings questions vertical (W53)", not left and calls >= 6, "no display (%s); OS boxes left %s, "
               "ynbox calls %d" % (e, left, calls))
        return
    try:
        root.withdraw()
        w, st = ynbox.build(root, "Title", "A question?", warn=True)
        w.update()
        y, n = st["yes"], st["no"]
        res["vertical"] = (y.master.pack_slaves() == [y, n] and y.pack_info().get("fill") == "x" and
                           n.pack_info().get("fill") == "x")
        res["no_default"] = st["sel"] is False and int(str(n.cget("highlightthickness"))) == 4
        res["keys"] = all(w.bind(k) for k in ("<Up>", "<Down>", "<Return>", "<Escape>"))
        foc = []
        y.focus_set = lambda: foc.append("yes")                    # (the OS may not give this test window the
        n.focus_set = lambda: foc.append("no")                     #  keyboard focus: the call itself is checked)
        st["select"](True)
        res["up"] = st["sel"] is True and int(str(y.cget("highlightthickness"))) == 4
        res["focus_yes"] = foc[-1:] == ["yes"]                      # WA review L3 / L4: the focus follows
        st["select"](False)
        res["focus_no"] = foc[-1:] == ["no"]
        res["break"] = "break" in str(n.bind("<space>")) or bool(n.bind("<space>"))
        st["select"](True)
        st["done"](st["sel"])
        res["yes"] = st["answer"] is True
        w2, st2 = ynbox.build(root, "T", "Q")
        w2.update()
        st2["select"](True)
        st2["done"](False)                                  # Esc / NO / closing the window
        res["esc"] = st2["answer"] is False
        ynbox.AUTO = True
        res["auto"] = ynbox.ask(root, "T", "Q") is True and ynbox.LAST.get("text") == "Q"
        ynbox.AUTO = None
    finally:
        root.destroy()
    report("YN7 (W53) the settings questions in the vertical layout: YES above NO, the same width, NO pre-selected, "
           "Up selects YES, Enter answers the selection, Esc / closing = NO; no OS message box left in the station "
           "(ynbox.ask calls: %d)" % calls, not left and calls >= 6 and all(res.values()),
           "OS boxes left %s, %s" % (left, res))


def main():
    yn1()
    yn2()
    yn3()
    yn4()
    yn5()
    yn6()
    yn7()
    n = len(results)
    print("YES / NO TESTS: %d/%d %s" % (sum(results), n, "PASS" if all(results) else "FAIL"))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
