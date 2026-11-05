"""Memory report from a build map.

    python tools/mapreport.py "TriCore Debug (TASKING)/TRYGONS_C2.map"
        ADS / TASKING map: flash and RAM used vs available per memory area,
        and the 10 largest items (functions / variables).

    python tools/mapreport.py --gcc build.elf
        Same for a TriCore GCC build (needs tricore-elf-size / -nm from ADS).

    python tools/mapreport.py --objects <folder with .o files>
        EXPECTED memory per core before any ADS build: adds up the sections of
        the compiled app / hal / Cpu*_Main objects the way the linker scripts
        place them, plus each core's stacks and CSA. The iLLD libraries and
        the start-up code are NOT included (a few KB more on core 0).

Every mode ends with RAM per core and the CPU1 check: CPU1's data RAM (DSPR1,
only the safety monitor lives there) must stay below 60 % - above it the
report says FAIL and exits with code 2. The data shared between cores
(app/memplace.h: g_mon, the log queues, g_logstat, g_timing) must be in core
0's DSPR (never cached): --objects checks their section, an ADS map that
none of them is at a cached LMU address (0x9...) - else FAIL, exit 2.

The ADS map mode also checks the STACKS (step 13s, stack_map): the linker's estimate, its "recursive"
flag (resolved with tools/stackcheck.py: a cycle only through function pointers is no recursion; --strict
fails on the flag alone) and, per core, the deepest call chain / every interrupt nested / the CSA x 1.5
inside what the map reserves - else FAIL, exit 2.

    python tools/mapreport.py --strict "<map>"     the same, "recursive: yes" alone fails too
    python tools/mapreport.py --selftest           the stack check on fixture map text (tools/check.sh)

Build the project in ADS first (hammer icon) so the map is up to date.
"""
import glob
import os
import re
import subprocess
import sys

CPU1_LIMIT = 60.0           # % of DSPR1 (the safety monitor's core) that may be used
BINDIR = r"C:\Infineon\AURIX-Studio-1.10.40\tools\Compilers\tricore-gcc11\bin"
SIZES = {"dsram0": 240 * 1024, "dsram1": 240 * 1024, "dsram2": 96 * 1024, "cpu0_dlmu": 64 * 1024,
         "cpu1_dlmu": 64 * 1024, "cpu2_dlmu": 64 * 1024, "pfls0": 3 * 1024 * 1024, "pfls1": 3 * 1024 * 1024}
CORES = [("core 0 (control)", ["dsram0"]), ("CPU1 (safety monitor)", ["dsram1"]), ("core 2 (logger)", ["dsram2"]),
         ("LMU block 0", ["cpu0_dlmu"]), ("LMU block 1", ["cpu1_dlmu"]), ("LMU block 2", ["cpu2_dlmu"])]

FLASH = {"pfls0": "program flash 0", "pfls1": "program flash 1", "dfls0": "data flash (EEPROM)",
         "ucb": "user config blocks"}
RAM = {"dsram0": "core 0 data RAM (DSPR0)", "dsram1": "core 1 data RAM (DSPR1)", "dsram2": "core 2 data RAM (DSPR2)",
       "psram0": "core 0 program RAM", "psram1": "core 1 program RAM", "psram2": "core 2 program RAM",
       "cpu0_dlmu": "LMU block 0 (cpu0_dlmu)", "cpu1_dlmu": "LMU block 1 (cpu1_dlmu)",
       "cpu2_dlmu": "LMU block 2 (cpu2_dlmu)"}


def kb(n):
    return "%.1f KB" % (n / 1024.0)


def table(rows, top):
    """rows: (area, used, total). Prints flash and RAM blocks."""
    for title, names in (("FLASH", FLASH), ("RAM", RAM)):
        print("\n%s" % title)
        print("  %-30s %12s %12s %7s" % ("area", "used", "size", "used %"))
        tu = tt = 0
        for area, used, total in rows:
            if area in names:
                print("  %-30s %12s %12s %6.1f%%" % (names[area], kb(used), kb(total), 100.0 * used / total))
                tu += used
                tt += total
        print("  %-30s %12s %12s %6.1f%%" % ("TOTAL " + title, kb(tu), kb(tt), 100.0 * tu / tt if tt else 0))
    print("\n10 largest items")
    print("  %-50s %10s  %s" % ("item", "size", "kind"))
    for name, size, kind in top[:10]:
        print("  %-50s %10s  %s" % (name[:50], "%d B" % size, kind))


SHARED = ["g_mon", "g_logq0", "g_logq1", "g_logq_gw", "s_q0", "s_q1", "s_qgw", "g_logstat", "g_timing",
          "s_stack_filled"]      # step 13s: hal_tc375.c, each core says it filled its stacks


def shared_objects(folder):
    """Every shared symbol defined in the objects sits in core 0's DSPR section."""
    out = subprocess.run([os.path.join(BINDIR, "tricore-elf-objdump.exe"), "-t"] +
                         sorted(glob.glob(os.path.join(folder, "*.o"))), capture_output=True, text=True).stdout
    found, bad = set(), []
    for line in out.splitlines():
        p = line.split()
        if len(p) >= 5 and p[-1] in SHARED and "*UND*" not in line:
            found.add(p[-1])
            sec = p[-3]
            if not re.match(r"\.z?bss_cpu0$", sec):
                bad.append("%s in %s" % (p[-1], sec))
    missing = [s for s in SHARED if s not in found]
    ok = not bad and not missing
    print("\nSHARED DATA (core to core, must not be cached): %s" % (
        "OK - all %d in core 0's DSPR (.bss_cpu0)" % len(found) if ok else
        "FAIL - %s" % "; ".join(bad + ["%s not found" % m for m in missing])))
    return ok


def shared_map(txt):
    """ADS map: no shared symbol at a cached LMU address (0x9...)."""
    bad = []
    for s in SHARED:
        for m in re.finditer(r"\b%s\b[^\n]*?0x(9[0-9a-f]{7})\b" % re.escape(s), txt):
            bad.append("%s at 0x%s" % (s, m.group(1)))
    print("\nSHARED DATA (core to core, must not be cached): %s" % ("FAIL - " + "; ".join(bad) if bad else "OK"))
    return not bad


def per_core(rows):
    """RAM per core + the CPU1 check. rows: (area, used, total). False if CPU1 is too full."""
    used = {a: (u, t) for a, u, t in rows}
    print("\nRAM PER CORE")
    print("  %-26s %12s %12s %7s" % ("core / block", "used", "size", "used %"))
    ok = True
    for name, areas in CORES:
        u = sum(used.get(a, (0, 0))[0] for a in areas)
        t = sum(used.get(a, (0, SIZES.get(a, 1)))[1] for a in areas)
        pct = 100.0 * u / t if t else 0.0
        flag = ""
        if areas == ["dsram1"] and pct > CPU1_LIMIT:
            flag, ok = "  <-- FAIL: above %.0f %%" % CPU1_LIMIT, False
        print("  %-26s %12s %12s %6.1f%%%s" % (name, kb(u), kb(t), pct, flag))
    print("\nCPU1 RAM check (max %.0f %%): %s" % (CPU1_LIMIT, "OK" if ok else "FAIL"))
    return ok


def lsl_reserved(lsl="Lcf_Tasking_Tricore_Tc.lsl"):
    """Stacks + CSA each core reserves in its DSPR (from the linker script)."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    txt = open(os.path.join(here, lsl), encoding="utf-8", errors="replace").read()
    out = {}
    for c in range(3):
        tot = 1024                                       # the 1 KB gap below the CSA
        for k in ("CSA", "USTACK", "ISTACK"):
            m = re.search(r"#define LCF_%s%d_SIZE\s+(\d+)k" % (k, c), txt)
            tot += int(m.group(1)) * 1024 if m else 0
        out["dsram%d" % c] = tot
    return out


def objects(folder):
    """Expected placement of the compiled objects (see the module help)."""
    used = {a: 0 for a in SIZES}
    for a, n in lsl_reserved().items():
        used[a] += n
    top = []
    for o in sorted(glob.glob(os.path.join(folder, "*.o"))):
        base = os.path.basename(o)
        out = subprocess.run([os.path.join(BINDIR, "tricore-elf-size.exe"), "-A", o], capture_output=True,
                             text=True).stdout
        for line in out.splitlines():
            p = line.split()
            if len(p) != 3 or not p[1].isdigit() or int(p[1]) == 0:
                continue
            sec, sz = p[0], int(p[1])
            if sec.startswith(".debug") or sec in (".comment", ".callinfo"):
                continue
            m = re.match(r"\.z?(lmu)?(bss|data)_cpu(\d)", sec)
            if m:
                area = ("cpu%s_dlmu" if m.group(1) else "dsram%s") % m.group(3)
            elif sec.startswith((".lmubss", ".lmudata", ".zlmubss")):
                area = "cpu0_dlmu"
            elif sec.startswith((".bss", ".data", ".zbss", ".zdata", ".sbss", ".sdata")):
                area = "dsram1" if base.startswith("Cpu1_Main") else "dsram2" if base.startswith("Cpu2_Main") \
                    else "dsram0"                       # LCF_DEFAULT_HOST = CPU0
            elif sec.startswith((".text", ".rodata", ".zrodata")):
                area = "pfls0"
            else:
                continue
            used[area] += sz
            if "data" in sec and not sec.startswith((".rodata", ".zrodata")):
                used["pfls0"] += sz                     # initial values are stored in flash too
            top.append(("%s (%s)" % (base, sec), sz, "code" if sec.startswith(".text") else "data"))
    rows = [(a, used[a], SIZES[a]) for a in SIZES]
    print("EXPECTED memory from the objects in %s (+ stacks / CSA; iLLD and start-up code not included)" % folder)
    table(rows, sorted(top, key=lambda t: -t[1]))
    ok = per_core(rows)
    return shared_objects(folder) and ok


def tasking(path, strict=False):
    txt = open(path, encoding="utf-8", errors="replace").read()
    rows = []
    for m in re.finditer(r"^\| mpe:(\S+)\s*\|\s*0x([0-9a-f]+) \|\s*0x([0-9a-f]+) \|\s*0x([0-9a-f]+) \|\s*0x([0-9a-f]+) \|"
                         r"\s*0x([0-9a-f]+) \|", txt, re.M):
        area = m.group(1)
        code, data, res, free, total = (int(m.group(i), 16) for i in range(2, 7))
        if area == "dfls0":
            res = 0                         # reserved for the EEPROM emulation, not used by us
        rows.append((area, code + data + res, total))
    items = {}
    for m in re.finditer(r"^\| (\S+)\s+\| (\.(?:text|bss|data|rodata|zbss|zdata)\.\S+)[^|]*\| 0x([0-9a-f]+)", txt, re.M):
        sec, size = m.group(2), int(m.group(3), 16)
        kind = {"text": "code"}.get(sec.split(".")[1], "data")
        name = ".".join(sec.split(".")[2:]) or sec
        if size > items.get(name, (0, kind))[0]:
            items[name] = (size, kind)
    top = sorted(((n, s, k) for n, (s, k) in items.items()), key=lambda t: -t[1])
    print("Map: %s" % path)
    table(rows, top)
    st = re.search(r"\* Estimated stack usage.*?\n(.*?)\n\n", txt, re.S)
    if st:
        print("\nStack (linker estimate):\n" + "\n".join("  " + l for l in st.group(1).splitlines()[:8]))
    ok = per_core(rows)
    ok = shared_map(txt) and ok
    return stack_map(txt, strict) and ok


def stack_map(txt, strict=False):
    """Step 13s: the stacks of an ADS / TASKING map. FAIL (False) when
      - the linker's estimate ("Estimated stack usage": one number per stack name, every root tree of the
        call graph added up - all cores and every interrupt together) > all reserved user + interrupt stacks,
      - the linker says "recursive: yes" and the call graph has a REAL cycle: the TASKING linker marks every
        chain through __INDIRECT__ (= any call through a function pointer, which it links to EVERY function
        whose address is taken - incl. core<n>_main and the iLLD start-up) as recursive; stackcheck resolves
        each pointer call to its real targets (stackcheck.INDIRECT) and fails on any cycle left
        (strict=True: "recursive: yes" alone fails too),
      - per core (stackcheck.analyse on the map's own frame sizes): user / interrupt stack estimate x 1.5 >
        reserved, the CSA, an unlisted pointer call, an interrupt of unknown core.
    No call graph in the map (an older ADS setting) = FAIL: nothing proves the stacks."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import stackcheck
    print("\nSTACK CHECK (step 13s)")
    est = [(m.group(1).strip(), int(m.group(2), 16))
           for m in re.finditer(r"^\|\s*(stack\s+\d+)\s*\|\s*0x([0-9a-f]+)\s*\|", txt, re.M)]
    rec = re.search(r"^\|\s*recursive\s*\|\s*(\w+)\s*\|", txt, re.M)
    recursive = bool(rec and rec.group(1).lower() == "yes")
    reserved = {}
    for m in re.finditer(r"\|\s*(ustack|istack|csa)_tc(\d) \(\d+\)\s*\|\s*0x([0-9a-f]+)", txt):
        reserved["%s%s" % (m.group(1), m.group(2))] = int(m.group(3), 16)
    problems = []
    if not est:
        problems.append("no \"Estimated stack usage\" table in the map")
    total_res = sum(v for k, v in reserved.items() if not k.startswith("csa"))
    total_est = sum(v for _n, v in est)
    print("  linker estimate: %s; recursive: %s; all user + interrupt stacks reserved: %d B" % (
        ", ".join("%s = %d B" % e for e in est) or "-", "yes" if recursive else "no", total_res))
    if est and total_est > total_res:
        problems.append("the linker's estimate %d B > %d B reserved for all stacks" % (total_est, total_res))
    g = stackcheck.parse_map(txt)
    if g is None:
        problems.append("no Call Graph section in the map (nothing proves the stacks)")
        ok_a = False
    else:
        memo = {}
        traps = [stackcheck.worst(g, f, memo) for f in g.self_b if f.startswith("IfxCpu_Trap_")]
        trap = (max(t[0] for t in traps), max(t[1] for t in traps)) if traps else None
        direct = stackcheck.cycles(g)            # before resolving: only the calls the code really makes
        for c in direct:
            problems.append("DIRECT RECURSION: " + " -> ".join(c[:12]))
        ok_a, lines, _ = stackcheck.analyse(g, reserved, stackcheck.isr_table(), trap=trap, label="TASKING map")
        print("\n".join("  " + l for l in lines))
        if recursive and ok_a and not direct:
            print("  the linker's \"recursive: yes\" = only through __INDIRECT__ (function pointers): resolved, no cycle")
            if strict:
                problems.append("strict: the linker says recursive: yes")
    for p in problems:
        print("  FAIL: " + p)
    ok = ok_a and not problems
    print("STACK CHECK: %s" % ("OK" if ok else "FAILED"))
    return ok


def selftest():
    """Fixture map text (the real TASKING format) for stack_map: tools/check.sh runs it."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import stackcheck
    fails = []

    def check(name, cond):
        print("[%s] %s" % ("PASS" if cond else "FAIL", name))
        if not cond:
            fails.append(name)

    def fixture(stack0=0x620, recursive="yes", extra_graph="", ustack0=0x1000):
        head = ("* Estimated stack usage\n========================\n+-------------------------+\n"
                "| Stack Name | Used       |\n|=========================|\n| stack   0  | 0x%08x |\n"
                "|-------------------------|\n| recursive  | %s        |\n+-------------------------+\n\n" %
                (stack0, recursive))
        secs = "".join("| mpe:dsram%d | %s%d | %s_tc%d (1) | 0x%08x | 0x0 | 0x0 | 0x8 |\n" % (
            c, k, c, k, c, size) for c, k, size in (
                (0, "ustack", ustack0), (0, "istack", 0x1000), (1, "ustack", 0x800), (1, "istack", 0x400),
                (2, "ustack", 0x800), (2, "istack", 0x400))) + "".join(
            "| mpe:dsram%d |  | csa_tc%d (1) | 0x00004000 | 0x0 | 0x0 | 0x40 |\n" % (c, c) for c in range(3))
        graph = stackcheck.MAP_FIXTURE.replace("\n\nNote:", "\n" + extra_graph + "\ncore1_main [s0:0,0]\n\n"
                                               "core2_main [s0:0,0]\n\nNote:")
        return head + secs + graph

    import io
    import contextlib

    def run(txt, strict=False):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            ok = stack_map(txt, strict)
        return ok, out.getvalue()
    ok, out = run(fixture())
    check("MR1 a map whose \"recursive: yes\" is only __INDIRECT__ (pointer calls, C start-up) passes, resolved",
          ok and "resolved, no cycle" in out)
    ok, _ = run(fixture(), strict=True)
    check("MR2 --strict: \"recursive: yes\" alone fails (exit 2)", not ok)
    ok, out = run(fixture(stack0=0x4000))
    check("MR3 the linker's estimate 16 KB > all reserved stacks = FAIL", not ok and "estimate" in out)
    ok, out = run(fixture(extra_graph="*R smallboat.src:loop_a [s0:8,8]\n|\n+-- smallboat.src:loop_b [s0:8,8]\n"
                                      "    |\n    +-- smallboat.src:loop_a *R\n"))
    check("MR4 a direct recursion (a -> b -> a) = FAIL", not ok and "DIRECT RECURSION" in out)
    ok, out = run(fixture(ustack0=0x800))
    check("MR5 core 0's chain (816 + 520 + ... B) x 1.5 > a 2 KB user stack = FAIL", not ok and "user stack core 0" in out)
    ok, out = run(fixture().replace("*R tc_fat32.src:read_block", "*R newdrv.src:read_block").replace(
        "+-- tc_fat32.src:read_block *R", "+-- newdrv.src:read_block *R"))
    check("MR6 a function-pointer call in a module stackcheck.INDIRECT does not list = FAIL",
          not ok and "not in stackcheck.INDIRECT" in out)
    ok, out = run(fixture().split("**  Call Graph  **")[0] + "\n")
    check("MR7 no call graph in the map = FAIL", not ok)
    print("MAPREPORT SELFTEST: %s" % ("PASS" if not fails else "FAIL (%d)" % len(fails)))
    return not fails


def gcc(elf):
    size = subprocess.run([os.path.join(BINDIR, "tricore-elf-size.exe"), "-A", elf], capture_output=True,
                          text=True).stdout
    regions = {"pfls0": (0x80000000, 0x300000), "pfls1": (0x80300000, 0x300000), "dfls0": (0xAF000000, 0x40000),
               "ucb": (0xAF400000, 0x6000), "dsram0": (0x70000000, 0x3C000), "dsram1": (0x60000000, 0x3C000),
               "dsram2": (0x50000000, 0x18000), "psram0": (0x70100000, 0x10000), "psram1": (0x60100000, 0x10000),
               "psram2": (0x50100000, 0x10000), "cpu0_dlmu": (0x90000000, 0x10000),
               "cpu1_dlmu": (0x90010000, 0x10000), "cpu2_dlmu": (0x90020000, 0x10000)}
    alias = [(0xA0000000, 0x80000000)]          # non-cached view of the program flash (6 MB)
    used = {k: 0 for k in regions}
    for line in size.splitlines():
        p = line.split()
        if len(p) != 3 or not p[1].isdigit() or p[0].startswith(".debug") or p[0] == ".comment":
            continue
        sz, addr = int(p[1]), int(p[2])
        for a0, a1 in alias:
            if a0 <= addr < a0 + 0x600000:
                addr = addr - a0 + a1
        for k, (o, l) in regions.items():
            if o <= addr < o + l:
                used[k] += sz
                # initialised data is stored in flash too
                if p[0] in (".data",) or p[0].endswith(".data"):
                    used["pfls0"] += sz
    rows = [(k, used[k], regions[k][1]) for k in regions]
    nm = subprocess.run([os.path.join(BINDIR, "tricore-elf-nm.exe"), "-S", "--size-sort", "-r", elf],
                        capture_output=True, text=True).stdout
    top = []
    for line in nm.splitlines():
        p = line.split()
        if len(p) == 4:
            top.append((p[3], int(p[1], 16), "code" if p[2] in "tT" else "data"))
    print("ELF (TriCore GCC): %s" % elf)
    table(rows, top)
    return per_core(rows)


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--gcc":
        good = gcc(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "--objects":
        good = objects(sys.argv[2])
    elif len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        good = selftest()
    elif len(sys.argv) == 3 and sys.argv[1] == "--strict":
        good = tasking(sys.argv[2], strict=True)
    elif len(sys.argv) == 2:
        good = tasking(sys.argv[1])
    else:
        print(__doc__)
        sys.exit(1)
    sys.exit(0 if good else 2)
