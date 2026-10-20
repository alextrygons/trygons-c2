"""Memory report from a build map.

    python tools/mapreport.py "TriCore Debug (TASKING)/TRYGONS_C2.map"
        ADS / TASKING map: flash and RAM used vs available per memory area,
        and the 10 largest items (functions / variables).

    python tools/mapreport.py --gcc build.elf
        Same for a TriCore GCC build (needs tricore-elf-size / -nm from ADS).

Build the project in ADS first (hammer icon) so the map is up to date.
"""
import os
import re
import subprocess
import sys

FLASH = {"pfls0": "program flash 0", "pfls1": "program flash 1", "dfls0": "data flash (EEPROM)",
         "ucb": "user config blocks"}
RAM = {"dsram0": "core 0 data RAM (DSPR0)", "dsram1": "core 1 data RAM (DSPR1)", "dsram2": "core 2 data RAM (DSPR2)",
       "psram0": "core 0 program RAM", "psram1": "core 1 program RAM", "psram2": "core 2 program RAM",
       "cpu0_dlmu": "core 0 LMU RAM", "cpu1_dlmu": "core 1 LMU RAM", "cpu2_dlmu": "core 2 LMU RAM"}


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


def gcc(elf):
    bindir = r"C:\Infineon\AURIX-Studio-1.10.40\tools\Compilers\tricore-gcc11\bin"
    size = subprocess.run([os.path.join(bindir, "tricore-elf-size.exe"), "-A", elf], capture_output=True,
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
    nm = subprocess.run([os.path.join(bindir, "tricore-elf-nm.exe"), "-S", "--size-sort", "-r", elf],
                        capture_output=True, text=True).stdout
    top = []
    for line in nm.splitlines():
        p = line.split()
        if len(p) == 4:
            top.append((p[3], int(p[1], 16), "code" if p[2] in "tT" else "data"))
    print("ELF (TriCore GCC): %s" % elf)
    table(rows, top)


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--gcc":
        gcc(sys.argv[2])
    elif len(sys.argv) == 2:
        tasking(sys.argv[1])
    else:
        print(__doc__)
        sys.exit(1)
