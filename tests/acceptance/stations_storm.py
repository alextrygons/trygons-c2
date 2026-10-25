"""Part 4.4, I16: random stations against the REAL gateway + simulator.

    python tests/acceptance/stations_storm.py [--seconds 180] [--seed 1]

Random commands from random stations and crew roles (CLAUDE.md section D):
a remote captain and a captain station on the boat (both with a screen 1 that
answers yes / no / not at all and crashes; the one on the boat takes HELM now
and then and so becomes the captain), a remote copilot, a viewer;
single-operator mode with and without its own confirmation, fake answers from
stations that are not the captain's screen 1, disconnects. Every frame the
gateway forwards to the AURIX is checked: a DRIVING command (proto.DRIVING_CMDS,
never STANDBY) must be the captain's own, a single-operator station's own
confirmed one, or a copilot's after the captain's ACCEPT (AUTO PAUSE = hold:
always) - and a copilot's command must travel in the captain's (HELM) envelope.
"""
import argparse
import os
import random
import sys
import tempfile
import shutil
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "gateway"))
os.chdir(os.path.join(REPO, "gateway"))
import test_gateway as TG  # noqa: E402 - its simulator / gateway / client helpers
from common import proto  # noqa: E402
import stations as stmod  # noqa: E402

FREE = (proto.T_AUTO_PAUSE,)          # copilot actions that never wait for the captain (CLAUDE.md D)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    TG.setup_logging(40)
    tmp = tempfile.mkdtemp(prefix="acc_stations_")
    sim = gw = None
    stats = {"forwarded": 0, "driving_forwarded": 0, "violations": 0, "asks": 0, "answers": 0, "first": None,
             "copilot_driving": 0, "captain_own": 0}
    try:
        sim = TG.Sim(1.0)
        root = os.path.join(tmp, "ssd")
        os.makedirs(root)
        cfg = TG.gw_cfg_b(sim, root, stations={"driver_timeout_s": 4.0})
        gw = TG.GwRunner(cfg)
        g = gw.gw
        stn = g.stations
        ctx = {"why": None}

        orig_cmd, orig_ans, orig_fwd = stn.command, stn._driver_answer, stn._forward

        def command(st, m):
            if st.single_op and m.get("confirm") == "self":
                ctx["why"] = "single"
            elif st.crew == "captain":
                ctx["why"] = "captain"
            else:
                ctx["why"] = None
            try:
                return orig_cmd(st, m)
            finally:
                ctx["why"] = None

        def driver_answer(st, m):
            q = stn.asks.get(m.get("id"))
            ok = bool(m.get("yes")) and q is not None and st in stn.drivers() and st.id == q.get("target")
            ctx["why"] = "accept" if ok else None
            stats["answers"] += 1
            try:
                return orig_ans(st, m)
            finally:
                ctx["why"] = None

        def bad(text):
            stats["violations"] += 1
            if stats["first"] is None:
                stats["first"] = text

        def forward(st, seq, ftype, payload, age, env=None, done=None):
            stats["forwarded"] += 1
            envelope = env or st
            if done is None and st.crew == "copilot" and not st.single_op and ftype not in stmod.READ_ONLY \
                    and ftype != proto.T_HELM and envelope is st:
                bad("copilot station %s forwarded type 0x%02X in its own envelope" % (st.id, ftype))
            if proto.is_driving_cmd(ftype, payload):
                stats["driving_forwarded"] += 1
                why = ctx["why"]
                if why == "captain" and st.crew == "captain":
                    stats["captain_own"] += 1
                elif why == "single":
                    pass
                elif why == "accept":
                    stats["copilot_driving"] += 1
                elif st.crew == "copilot" and ftype in FREE:
                    pass
                else:
                    bad("type 0x%02X from station %s (crew %s, single_op %s) without the captain's ACCEPT" % (
                        ftype, st.id, st.crew, st.single_op))
            return orig_fwd(st, seq, ftype, payload, age, env=env, done=done)
        stn.command, stn._driver_answer, stn._forward = command, driver_answer, forward
        stn.ext["driver_answer"] = driver_answer

        sp = cfg["stations"]["port"]
        mk = {"alice": lambda: TG.Client(sp, "alice", TG.PW["alice"], name="captain", crew="captain"),
              "carol": lambda: TG.Client(sp, "bob", TG.PW["bob"], local=True, name="captain on board"),
              "dave": lambda: TG.Client(sp, "bob", TG.PW["bob"], name="copilot", crew="copilot"),
              "vic": lambda: TG.Client(sp, "vic", TG.PW["vic"], name="viewer")}
        cl = {k: f() for k, f in mk.items()}
        cl["alice"].cmd(proto.T_HELM, proto.p_helm(0))
        beats = {k: TG.DriverBeat(cl[k]) for k in ("alice", "carol")}
        driving = sorted(proto.DRIVING_CMDS) + [proto.T_ALARM_ACK, proto.T_PARAM_GET, proto.T_HELM]
        end = time.time() + a.seconds
        while time.time() < end:
            who = rng.choice(["alice", "carol", "dave", "vic", "dave", "alice"])
            c = cl[who]
            p = rng.random()
            try:
                if p < 0.45:
                    t = rng.choice(driving)
                    if t == proto.T_SET_MODE:
                        pl = bytes([rng.randrange(0, 6)])
                    elif t == proto.T_HELM:
                        pl = bytes([rng.randrange(0, 2)])
                    else:
                        pl = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 6)))
                    c.l.send(t, pl, what="storm", confirm="self" if rng.random() < 0.5 else None)
                elif p < 0.50:
                    c.l.send_msg({"t": "single_op", "on": rng.random() < 0.4})
                elif p < 0.60:
                    # the captains' screen 1 questions: yes / no / nothing; fake answers from others
                    for k in ("alice", "carol"):
                        for _ in range(20):
                            try:
                                kind, d = cl[k].l.events.get_nowait()
                            except Exception:
                                break
                            if kind == "driver_ask":
                                stats["asks"] += 1
                                r = rng.random()
                                if r < 0.45:
                                    cl[k].l.send_msg({"t": "driver_answer", "id": d["id"], "yes": True})
                                elif r < 0.75:
                                    cl[k].l.send_msg({"t": "driver_answer", "id": d["id"], "yes": False})
                                if rng.random() < 0.3:          # a fake ACCEPT from a copilot / viewer
                                    cl[rng.choice(["dave", "vic"])].l.send_msg(
                                        {"t": "driver_answer", "id": d["id"], "yes": True})
                elif p < 0.63:
                    k = rng.choice(["alice", "carol"])
                    beats[k].stop()                             # a screen 1 crashes ...
                    time.sleep(rng.uniform(0.5, 3.0))
                    beats[k] = TG.DriverBeat(cl[k])             # ... and comes back
                elif p < 0.65:
                    if who in beats:
                        beats[who].stop()
                    cl[who].close()
                    cl[who] = mk[who]()
                    if who in beats:
                        beats[who] = TG.DriverBeat(cl[who])
                for k in ("dave", "vic"):                       # keep the queues short
                    try:
                        while True:
                            cl[k].l.events.get_nowait()
                    except Exception:
                        pass
            except Exception:
                pass
            time.sleep(rng.uniform(0.02, 0.3))
        # self-test: a planted copilot driving frame without any ACCEPT must be caught
        v0, first0 = stats["violations"], stats["first"]
        cop = next((s for s in stn.stations.values() if s.crew == "copilot"), None)
        if cop is not None:
            gw.call(forward, cop, 0, proto.T_SET_MODE, bytes([proto.MODE_AUTO]), 0.0)
        stats["selftest"] = stats["violations"] >= v0 + 1
        stats["violations"], stats["first"] = v0, first0
        stats["driving_forwarded"] -= 1
        for b in beats.values():
            b.stop()
        for c in cl.values():
            c.close()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)
    print("=== I16 STATION STORM: seed %d, %.0f s ===" % (a.seed, a.seconds))
    print("%s I16  driving commands only the captain's own, single-operator self-confirmed, or a copilot's after the "
          "captain's ACCEPT (in the captain's envelope): %d frames forwarded, %d driving (%d captain's own, %d copilot "
          "after ACCEPT), %d violations; %d questions, %d answers%s" % (
              "FAIL" if stats["violations"] else "PASS", stats["forwarded"], stats["driving_forwarded"],
              stats["captain_own"], stats["copilot_driving"], stats["violations"], stats["asks"], stats["answers"],
              ("; first: " + stats["first"]) if stats["first"] else ""))
    print("%s self-test: a planted copilot driving frame without ACCEPT was caught" % (
        "PASS" if stats.get("selftest") else "FAIL"))
    return 1 if stats["violations"] or stats["driving_forwarded"] == 0 or not stats.get("selftest") else 0


if __name__ == "__main__":
    sys.exit(main())
