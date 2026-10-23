"""Part 4.4, I16: random stations against the REAL gateway + simulator.

    python tests/acceptance/stations_storm.py [--seconds 180] [--seed 1]

Random commands from random stations and roles (captain remote / on the boat,
viewer), a driver screen that answers yes / no / not at all and crashes,
single-operator mode with and without its own confirmation, fake answers from
stations that are not a driver screen, disconnects. Every frame the gateway
forwards to the AURIX is checked: a DRIVING command (proto.DRIVING_CMDS, never
STANDBY) must come from a driver screen's ACCEPT or a single-operator station's
own confirmation - nothing else.
"""
import argparse
import os
import random
import struct
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
             "refusals": {}}
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
            ctx["why"] = "single" if (st.single_op and m.get("confirm") == "self") else None
            try:
                return orig_cmd(st, m)
            finally:
                ctx["why"] = None

        def driver_answer(st, m):
            ok = bool(m.get("yes")) and st in stn.drivers() and m.get("id") in stn.asks
            ctx["why"] = "driver" if ok else None
            stats["answers"] += 1
            try:
                return orig_ans(st, m)
            finally:
                ctx["why"] = None

        def forward(st, seq, ftype, payload, age):
            stats["forwarded"] += 1
            if proto.is_driving_cmd(ftype, payload):
                stats["driving_forwarded"] += 1
                if ctx["why"] is None:
                    stats["violations"] += 1
                    if stats["first"] is None:
                        stats["first"] = "type 0x%02X from station %s (single_op %s) without ACCEPT" % (
                            ftype, st.id, st.single_op)
            return orig_fwd(st, seq, ftype, payload, age)
        stn.command, stn._driver_answer, stn._forward = command, driver_answer, forward
        stn.ext["driver_answer"] = driver_answer

        sp = cfg["stations"]["port"]
        mk = {"alice": lambda: TG.Client(sp, "alice", TG.PW["alice"], name="captain"),
              "carol": lambda: TG.Client(sp, "bob", TG.PW["bob"], local=True, name="captain on board"),
              "vic": lambda: TG.Client(sp, "vic", TG.PW["vic"], name="viewer"),
              "bob": lambda: TG.Client(sp, "bob", TG.PW["bob"], name="driver screen")}
        cl = {k: f() for k, f in mk.items()}
        beat = TG.DriverBeat(cl["bob"])
        driving = sorted(proto.DRIVING_CMDS) + [proto.T_ALARM_ACK, proto.T_PARAM_GET, proto.T_HELM]
        end = time.time() + a.seconds
        while time.time() < end:
            who = rng.choice(["alice", "carol", "vic", "alice", "carol"])
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
                    msg = {"t": "cmd", "type": t}
                    seq = c.l.send(t, pl, what="storm", confirm="self" if rng.random() < 0.5 else None)
                elif p < 0.52:
                    c.l.send_msg({"t": "single_op", "on": rng.random() < 0.5})
                elif p < 0.60:
                    # the driver screen's questions: yes / no / nothing
                    for _ in range(20):
                        try:
                            k, d = cl["bob"].l.events.get_nowait()
                        except Exception:
                            break
                        if k == "driver_ask":
                            stats["asks"] += 1
                            r = rng.random()
                            if r < 0.45:
                                cl["bob"].l.send_msg({"t": "driver_answer", "id": d["id"], "yes": True})
                            elif r < 0.75:
                                cl["bob"].l.send_msg({"t": "driver_answer", "id": d["id"], "yes": False})
                            if rng.random() < 0.2:                   # a fake ACCEPT from a non-driver
                                c.l.send_msg({"t": "driver_answer", "id": d["id"], "yes": True})
                elif p < 0.63:
                    beat.stop()                                       # the driver screen crashes ...
                    time.sleep(rng.uniform(0.5, 4.0))
                    beat = TG.DriverBeat(cl["bob"])                    # ... and comes back
                elif p < 0.65:
                    old = cl[who]
                    old.close()
                    cl[who] = mk[who]()
                for k in ("alice", "carol", "vic"):                    # keep the queues short
                    try:
                        while True:
                            cl[k].l.events.get_nowait()
                    except Exception:
                        pass
            except Exception:
                pass
            time.sleep(rng.uniform(0.02, 0.3))
        # self-test: a planted driving frame without any ACCEPT must be caught
        v0, first0 = stats["violations"], stats["first"]
        any_st = next(iter(stn.stations.values()), None)
        if any_st is not None:
            gw.call(forward, any_st, 0, proto.T_SET_MODE, bytes([proto.MODE_AUTO]), 0.0)
        stats["selftest"] = stats["violations"] == v0 + 1
        stats["violations"], stats["first"] = v0, first0
        stats["driving_forwarded"] -= 1
        beat.stop()
        for c in cl.values():
            c.close()
    finally:
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        shutil.rmtree(tmp, ignore_errors=True)
    print("=== I16 STATION STORM: seed %d, %.0f s ===" % (a.seed, a.seconds))
    print("%s I16  driving commands only with the driver's ACCEPT or single-operator self-confirmation: "
          "%d frames forwarded, %d driving, %d violations; %d driver questions, %d answers%s" % (
              "FAIL" if stats["violations"] else "PASS", stats["forwarded"], stats["driving_forwarded"],
              stats["violations"], stats["asks"], stats["answers"],
              ("; first: " + stats["first"]) if stats["first"] else ""))
    print("%s self-test: a planted driving frame without ACCEPT was caught" % (
        "PASS" if stats.get("selftest") else "FAIL"))
    return 1 if stats["violations"] or stats["driving_forwarded"] == 0 or not stats.get("selftest") else 0


if __name__ == "__main__":
    sys.exit(main())
