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
report says FAIL and exits with code 2.

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
    return per_core(rows)


def tasking(path):
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
    return per_core(rows)


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
    elif len(sys.argv) == 2:
        good = tasking(sys.argv[1])
    else:
        print(__doc__)
        sys.exit(1)
    sys.exit(0 if good else 2)
