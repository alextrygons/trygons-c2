"""Basic COLREGs for harbour autonomy (CLAUDE.md Z M0-7, docs/harbour_design.md 7.3; step HX5). PURE: no I/O.

Only for range-confirmed (steer_ok) MOVING vessels (speed > moving_mps, not static, class vessel / unknown); small
craft (kayak / person / dinghy) and buoys are not a COLREGs case (the planner keeps its minimum distances).
An encounter is classified when the CPA (constant velocities) is < 2 x min_moving_m within the horizon:

  HEAD_ON     target bearing within +-6 deg of our bow AND its course reciprocal +-15 deg -> both alter to STARBOARD
              (a port turn is infeasible)
  CROSS_STBD  bearing 6..112.5 deg to starboard -> we GIVE WAY: slow / pass astern, never a port turn across its bow
  CROSS_PORT  bearing -112.5..-6 deg -> we are stand-on (Rule 17); never a port turn; the planner slows / stops if it
              does not act (W145 owner choice beyond Rule 17)
  OVERTAKING  we come up from more than 22.5 deg abaft its beam (within 67.5 deg of its stern) and are faster -> keep
              clear, any side, slowing preferred
  OVERTAKEN   it comes up on us from our stern sector, faster -> stand-on (hold course / speed)
  AMBIGUOUS   anything else (bearing ahead but not reciprocal, an unknown aspect = a velocity too uncertain for its
              speed) -> slow, then STOP (U8)

Bearings / courses in degrees true, positions ENU metres (x east, y north)."""
import math

HEAD_ON, CROSS_STBD, CROSS_PORT, OVERTAKING, OVERTAKEN, AMBIGUOUS = (
    "head_on", "cross_stbd", "cross_port", "overtaking", "overtaken", "ambiguous")
# the classes where WE keep clear (hflags b2 give-way)
GIVE_WAY = (HEAD_ON, CROSS_STBD, OVERTAKING)
# the classes where a turn to PORT is never allowed
NO_PORT = (HEAD_ON, CROSS_STBD, CROSS_PORT, OVERTAKEN, AMBIGUOUS)
COLREG_CLS = ("vessel", "unknown")


def wrap(a):
    return (a + 540.0) % 360.0 - 180.0


def cpa(px, py, vx, vy):
    """Relative position / velocity -> (cpa m, tcpa s); tcpa < 0 = opening."""
    vv = vx * vx + vy * vy
    if vv < 1e-9:
        return math.hypot(px, py), 0.0
    t = -(px * vx + py * vy) / vv
    return math.hypot(px + vx * t, py + vy * t), t


def vel_sd(tr):
    P = tr.get("P")
    try:
        return math.sqrt(max(0.0, 0.5 * (float(P[2][2]) + float(P[3][3]))))
    except (TypeError, IndexError, ValueError):
        return 9.9                                  # unknown = very uncertain (fail closed)


def classify(own, tr, cfg):
    """own = {x, y, hdg, vx, vy, speed (m/s)}, tr = a tracker track -> None (no COLREGs encounter) or
    {id, cls, cpa, tcpa, rel_b}."""
    if not tr.get("steer_ok") or tr.get("static") or tr.get("cls") not in COLREG_CLS:
        return None
    sp = float(tr.get("speed") or 0.0)
    if sp <= float(cfg["moving_mps"]):
        return None
    dx, dy = float(tr["x"]) - own["x"], float(tr["y"]) - own["y"]
    c, t = cpa(dx, dy, float(tr["vx"]) - own["vx"], float(tr["vy"]) - own["vy"])
    if not (0.0 <= t <= float(cfg["horizon_s"]) and c < 2.0 * float(cfg["min_moving_m"])):
        return None
    brg = math.degrees(math.atan2(dx, dy)) % 360.0
    rel_b = wrap(brg - own["hdg"])                  # + = starboard
    course = float(tr.get("course") or 0.0)
    recip = wrap(course - (own["hdg"] + 180.0))
    b_t = wrap(brg + 180.0 - course)                # where we are, seen from its bow
    if vel_sd(tr) > 0.5 * sp:
        k = AMBIGUOUS                               # its aspect is unknown
    elif abs(rel_b) <= 6.0 and abs(recip) <= 15.0:
        k = HEAD_ON
    elif abs(b_t) >= 112.5 and own["speed"] > sp:
        k = OVERTAKING
    elif abs(rel_b) >= 112.5 and sp > own["speed"]:
        k = OVERTAKEN
    elif 6.0 < rel_b < 112.5:
        k = CROSS_STBD
    elif -112.5 < rel_b < -6.0:
        k = CROSS_PORT
    else:
        k = AMBIGUOUS
    return {"id": tr.get("id"), "cls": k, "cpa": round(c, 1), "tcpa": round(t, 1), "rel_b": round(rel_b, 1)}


def encounters(own, tracks, cfg):
    """Every COLREGs encounter + the overall judgement: (list, ambiguous: bool). Two encounters of DIFFERENT classes
    at once = ambiguous (design 7.3)."""
    out = [e for e in (classify(own, tr, cfg) for tr in tracks) if e is not None]
    kinds = {e["cls"] for e in out}
    amb = AMBIGUOUS in kinds or len(kinds) > 1
    return out, amb
