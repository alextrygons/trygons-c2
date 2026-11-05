"""Bench tab "Stack use" (step 13s; 13s review LOW-3): BENCH_DIAG's stack block decoded + drawn. DISPLAY ONLY -
no window, no boat (proto.decode_bench_stacks / decode_bench_diag, app.App.draw_bench_stack on a fake label).

  BS1 decode_bench_stacks: 6 stacks after the 86 sensor bytes - name / core / kind / size, used only when known,
      the overflow bit, the "measured on the board" flag
  BS2 an older AURIX (86 bytes, no stack block) = None; a cut block = only the whole entries; more than 6 = "s<n>"
  BS3 decode_bench_diag carries the same block; the lengths = app/proto.h (PROTO_BENCH_DIAG_STACK_OFF / _LEN)
  BS4 draw_bench_stack: one line per core (user + interrupt), "measuring" while unknown, amber when a stack is more
      than 2/3 used (< 50 % margin), red on OVERFLOW (red wins), the simulator note when not from the board
  BS5 draw_bench_stack: no data (older firmware) = the "no stack data" line, dim; no Bench tab built = nothing
"""
import os
import re
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
TMP = tempfile.mkdtemp(prefix="trygons_bs_")
os.environ["TRYGONS_LANG"] = "en"
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(TMP, "settings.json")
os.environ.setdefault("TRYGONS_TEST", "1")
os.environ["TRYGONS_SOUND"] = "off"
import i18n  # noqa: E402
import proto  # noqa: E402
import app as A  # noqa: E402

results = []


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def block(stacks, board=True, n=None):
    """stacks = [(size, used, known, overflow)] -> the 86 sensor bytes + the stack block"""
    known = sum(1 << i for i, s in enumerate(stacks) if s[2])
    over = sum(1 << i for i, s in enumerate(stacks) if s[3])
    out = bytes(proto.BENCH_DIAG_STACK_OFF) + struct.pack("<BBBB", len(stacks) if n is None else n,
                                                          1 if board else 0, known & 0xFF, over & 0xFF)
    for size, used, _k, _o in stacks:
        out += struct.pack("<HH", size, used)
    return out


SIX = [(4096, 900, True, False), (4096, 1100, True, False), (2048, 160, True, False), (1024, 0, False, False),
       (2048, 400, True, False), (1024, 32, True, False)]


class Label:
    def __init__(self):
        self.kw = {}

    def configure(self, **kw):
        self.kw.update(kw)


class Fake:
    def __init__(self):
        self.bench_stack = Label()


def draw(st):
    f = Fake()
    A.App.draw_bench_stack(f, st)
    return f.bench_stack.kw


def main():
    # BS1
    st = proto.decode_bench_stacks(block(SIX))
    s = st["stacks"] if st else []
    ok = (st is not None and st["board"] and len(s) == 6 and [x["name"] for x in s] == proto.STACK_NAMES and
          [x["core"] for x in s] == [0, 0, 1, 1, 2, 2] and [x["kind"] for x in s] == ["user", "interrupt"] * 3 and
          s[0]["used"] == 900 and s[0]["size"] == 4096 and s[3]["used"] is None and not s[3]["known"] and
          not any(x["overflow"] for x in s))
    report("BS1 the stack block decoded: 6 stacks, user / interrupt per core, used only when known", ok,
           "%s" % [(x["name"], x["used"], x["size"]) for x in s])
    st2 = proto.decode_bench_stacks(block([(1024, 1024, True, True)] + SIX[1:], board=False))
    report("BS1b the overflow bit + the board flag", st2["stacks"][0]["overflow"] and not st2["board"],
           "overflow %s, board %s" % (st2["stacks"][0]["overflow"], st2["board"]))
    # BS2
    old = proto.decode_bench_stacks(bytes(86))
    cut = proto.decode_bench_stacks(block(SIX)[:proto.BENCH_DIAG_STACK_OFF + 4 + 3 * 4 + 2])
    more = proto.decode_bench_stacks(block(SIX + [(512, 8, True, False)]))
    report("BS2 an older AURIX = None; a cut block = whole entries only; a 7th stack = s6",
           old is None and len(cut["stacks"]) == 3 and more["stacks"][6]["name"] == "s6",
           "old %s, cut %d, 7th %s" % (old, len(cut["stacks"]), more["stacks"][6]["name"]))
    # BS3
    pay = block(SIX)
    d = proto.decode_bench_diag(pay)
    h = open(os.path.join(HERE, "..", "app", "proto.h"), encoding="utf-8").read()
    off = int(re.search(r"PROTO_BENCH_DIAG_STACK_OFF\s+(\d+)u", h).group(1))
    ln = int(re.search(r"PROTO_BENCH_DIAG_LEN\s+(\d+)u", h).group(1))
    report("BS3 decode_bench_diag carries the stack block; offsets = app/proto.h (%d / %d)" % (off, ln),
           d["stack"] == proto.decode_bench_stacks(pay) and len(pay) == ln == proto.BENCH_DIAG_LEN and
           off == proto.BENCH_DIAG_STACK_OFF == A.proto.BENCH_DIAG_STACK_OFF,
           "payload %d B" % len(pay))
    # BS4
    kw = draw(proto.decode_bench_stacks(block(SIX)))
    lines = kw.get("text", "").split("\n")
    normal = kw.get("fg") == A.C["fg"]
    report("BS4 one line per core, the values + 'measuring', normal colour below 2/3",
           len(lines) == 3 and "900 / 4096 B (22 %)" in lines[0] and "1100 / 4096 B (27 %)" in lines[0] and
           "measuring" in lines[1] and "CPU1" in lines[1] and normal, " | ".join(lines))
    amber = draw(proto.decode_bench_stacks(block(SIX[:4] + [(2048, 1400, True, False)] + SIX[5:])))
    red = draw(proto.decode_bench_stacks(block([(4096, 4096, True, True)] + SIX[1:4] +
                                               [(2048, 1400, True, False)] + SIX[5:])))
    sim = draw(proto.decode_bench_stacks(block(SIX, board=False)))
    report("BS4b > 2/3 used = amber (< 50 % margin), OVERFLOW = red (wins), simulated values noted",
           amber.get("fg") == A.ORANGE and red.get("fg") == A.RED and "OVERFLOW" in red.get("text", "") and
           sim.get("text", "").split("\n")[-1] == i18n.T("bench.stack_sim"),
           "amber %s, red %s, sim line %r" % (amber.get("fg"), red.get("fg"), sim.get("text", "").split("\n")[-1]))
    # BS5
    none = draw(None)
    empty = draw({"board": True, "stacks": []})
    nob = object.__new__(A.App)            # no Bench tab built: no bench_stack attribute
    try:
        A.App.draw_bench_stack(nob, proto.decode_bench_stacks(block(SIX)))
        quiet = True
    except Exception as e:      # noqa: BLE001
        quiet = False
        print(e)
    report("BS5 no stack data = 'no stack data' (dim); no Bench tab = nothing, no error",
           none.get("text") == i18n.T("bench.stack_none") and none.get("fg") == A.C["dim"] and
           empty.get("text") == none.get("text") and quiet, "text %r" % none.get("text"))
    ok = all(results)
    print("BENCH STACK TESTS: %s (%d/%d)" % ("PASS" if ok else "FAIL", sum(results), len(results)))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
