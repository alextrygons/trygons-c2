"""AUTO waypoint programs: upload / download over the link, save / load as JSON.

A program is a dict:
    {"final": 0 (stop + alarm) | 1 (hold heading slow),
     "wps": [{"lat": .., "lon": .., "kn": .., "radius": .., "action": 0|1|2}, ...]}
action: 0 continue, 1 slow down, 2 stop in neutral (resume = PC + joystick CONFIRM)
The boat checks everything again (app/program.c) and refuses bad programs.
"""
import json

import proto

MAX_WP = 50


def new_program():
    return {"final": 0, "wps": []}


def check(prog, max_kn=25.0):
    """Quick local check (the boat checks again). Returns a list of problems."""
    problems = []
    n = len(prog["wps"])
    if not 1 <= n <= MAX_WP:
        problems.append("1..%d waypoints needed (have %d)" % (MAX_WP, n))
    for i, w in enumerate(prog["wps"]):
        if not (-90 <= w["lat"] <= 90 and -180 <= w["lon"] <= 180):
            problems.append("waypoint %d: bad coordinates" % (i + 1))
        if not (1.0 <= w["kn"] <= max_kn):
            problems.append("waypoint %d: speed %.1f kn outside 1..%.0f" % (i + 1, w["kn"], max_kn))
        if not (5 <= w["radius"] <= 500):
            problems.append("waypoint %d: radius %.0f m outside 5..500" % (i + 1, w["radius"]))
    return problems


def upload_frames(prog):
    """(type, payload) list to send, in order."""
    out = [(proto.T_PROG_BEGIN, proto.p_prog_begin(len(prog["wps"]), prog["final"]))]
    for i, w in enumerate(prog["wps"]):
        out.append((proto.T_PROG_WP, proto.p_prog_wp(i, w["lat"], w["lon"], w["kn"], w["radius"], w["action"])))
    out.append((proto.T_PROG_COMMIT, b""))
    return out


def upload(waiter, prog, timeout=3.0):
    """Upload with a link.Waiter. Returns (ok, text)."""
    for t, p in upload_frames(prog):
        a = waiter.command(t, p, timeout)
        if a is None:
            return False, "no answer from the boat"
        if not a["ok"]:
            if t == proto.T_PROG_COMMIT:
                waiter.pump(0.2)
                e = waiter.prog_error
                if e:
                    where = "" if e["index"] == 255 else " (waypoint %d)" % (e["index"] + 1)
                    return False, "program refused: %s%s" % (e["text"], where)
            return False, "%s: %s" % (a["cmd_name"], a["text"])
    return True, "program stored on the boat (%d waypoints)" % len(prog["wps"])


def from_wp_values(values):
    """Program from PROG_WP_VALUE dicts (download)."""
    if not values:
        return new_program()
    n = values[0]["n"]
    prog = {"final": values[0]["final"], "wps": []}
    for i in range(n):
        v = next((x for x in values if x["index"] == i), None)
        if v is None:
            break
        prog["wps"].append({"lat": v["lat"], "lon": v["lon"], "kn": v["kn"], "radius": v["radius"],
                            "action": v["action"]})
    return prog


def save(prog, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(prog, f, indent=1)


def load(path):
    with open(path, encoding="utf-8") as f:
        prog = json.load(f)
    prog.setdefault("final", 0)
    for w in prog["wps"]:
        w.setdefault("radius", 25.0)
        w.setdefault("action", 0)
    return prog
