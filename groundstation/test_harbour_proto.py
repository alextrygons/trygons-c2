"""Step HX2: the harbour autonomy protocol mirrors on the PC side (groundstation/proto.py = app/proto.h + link.c).

  HP0a T_HARBOUR 0x3C / T_HARBOUR_STAT 0x90 / the 18-byte layout match app/proto.h; GW_NAVREQ kind 4
  HP0b p_harbour + decode_harbour_stat (fields, flags, the "no proposal" age); a short frame is refused
  HP0c is_driving_cmd: a copilot's START / RESUME needs the captain's ACCEPT, HOLD goes straight
  HP0d the alarm SB_ALM2_HARBOUR_STOP 0x100, log event 32 (tlg), CONFIRM kind 12, gateway result 129 (+ lang)

    python test_harbour_proto.py
"""
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "gateway"))
os.environ.setdefault("TRYGONS_TEST", "1")
import proto  # noqa: E402
import tlg  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" - " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


def cdefine(text, name):
    m = re.search(r"#define\s+%s\s+(0x[0-9A-Fa-f]+|\d+)u?" % name, text)
    return int(m.group(1), 0) if m else None


def main():
    h = open(os.path.join(ROOT, "app", "proto.h"), encoding="utf-8").read()
    sbh = open(os.path.join(ROOT, "app", "smallboat.h"), encoding="utf-8").read()
    hh = open(os.path.join(ROOT, "app", "harbour.h"), encoding="utf-8").read()
    ok = (cdefine(h, "PROTO_T_HARBOUR") == proto.T_HARBOUR == 0x3C and
          cdefine(h, "PROTO_T_HARBOUR_STAT") == proto.T_HARBOUR_STAT == 0x90 and
          cdefine(h, "PROTO_HARBOUR_STAT_LEN") == proto.HARBOUR_STAT_LEN == struct.calcsize("<BBhHHHHBBHBB") and
          cdefine(sbh, "SB_NAV_HARBOUR") == proto.NAV_HARBOUR and
          [cdefine(hh, n) for n in ("HARB_ACT_START", "HARB_ACT_RESUME", "HARB_ACT_HOLD")] ==
          [proto.HARB_START, proto.HARB_RESUME, proto.HARB_HOLD] and
          cdefine(hh, "HARB_WHY_COUNT") == len(proto.HARB_WHY) and len(proto.HARB_STATES) == 5)
    check("HP0a T_HARBOUR / T_HARBOUR_STAT / layout / kind 4 / actions / why codes = app/", ok)

    pl = proto.p_harbour(proto.HARB_START, 10.0, 5.0)
    raw = struct.pack("<BBhHHHHBBHBB", 3, 2, -153, 100, 25, 9050, 65534, 7, 1 | 4 | 8, 123, 1, 50)
    d = proto.decode_harbour_stat(raw)
    try:
        proto.decode_harbour_stat(raw[:10])
        short = False
    except ValueError:
        short = True
    ok = (pl == bytes([0, 100, 50]) and d["harb_state_name"] == "HOLD" and d["harb_why"] == 2 and
          abs(d["harb_xte_m"] + 15.3) < 1e-6 and d["harb_corr_m"] == 10.0 and d["harb_kn"] == 2.5 and
          abs(d["harb_hdg"] - 90.5) < 1e-6 and d["harb_prop_age_ms"] is None and d["harb_route"] and
          d["harb_lease"] and d["harb_hold_pending"] and not d["harb_grace"] and d["harb_grace_left_s"] == 12.3 and
          d["harb_max_kn"] == 5.0 and short)
    check("HP0b p_harbour / decode_harbour_stat", ok, str(d))

    ok = (proto.is_driving_cmd(proto.T_HARBOUR, proto.p_harbour(proto.HARB_START)) and
          proto.is_driving_cmd(proto.T_HARBOUR, proto.p_harbour(proto.HARB_RESUME)) and
          not proto.is_driving_cmd(proto.T_HARBOUR, proto.p_harbour(proto.HARB_HOLD)))
    check("HP0c a copilot's harbour START / RESUME = the captain's ACCEPT, HOLD free", ok)

    import stations  # noqa: E402
    en = open(os.path.join(HERE, "lang", "en.txt"), encoding="utf-8").read()
    el = open(os.path.join(HERE, "lang", "el.txt"), encoding="utf-8").read()
    ok = (cdefine(sbh, "SB_ALM2_HARBOUR_STOP") == 0x100 and (0x100, "HARBOUR STOP") in proto.ALARMS2 and
          tlg.EVENTS_NOT_IN_HEADER.get(32, ("",))[0] == "HARBOUR" and proto.CONFIRM_KINDS[12] == "HARBOUR" and
          stations.GW_RESULTS.get(129) == "harbour route refused" and "result.129 =" in en and "result.129 =" in el)
    check("HP0d alarm 0x100, log event 32, CONFIRM kind 12, result 129 (+ lang)", ok)
    print("TOTAL: %d failed" % len(FAILS))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
