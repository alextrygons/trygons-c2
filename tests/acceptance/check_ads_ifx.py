"""Every iLLD function the board firmware calls must be BUILT by ADS - else the
TASKING link fails with "unresolved external" (TASKING runs only inside ADS,
so tools/check.sh can't link; this is the file-level check instead).

    python tests/acceptance/check_ads_ifx.py [--symbols FILE] [--cproject FILE]

--symbols: the Ifx* names left undefined by the compiled hal/ and Cpu*_Main.c
objects (tools/check.sh writes it with tricore-elf-nm -u). Without it the
Ifx*( calls in hal/*.c and Cpu*_Main.c are scanned (less exact).

For each name the iLLD .c that defines it is looked up under Libraries/ and
Configurations/; the Ifx functions THAT file calls are followed too (e.g.
IfxAsclin_Asc.c -> IfxAsclin.c). FAILS (exit 1) if a name has no defining
.c file, or a needed .c file is excluded in any build configuration of
.cproject (a parent folder exclusion hides its sub-folders too).
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
LIB_DIRS = ("Libraries", "Configurations")
CALL = re.compile(r"\b(Ifx\w+)\s*\(")
# a definition starts at column 0: "void IfxAsclin_Asc_initModule(" or "IfxAsclin_Rx_In IfxAsclin0_RXA_P15_3_IN = {"
DEF = re.compile(r"^[A-Za-z_][\w \t\*]*?\b(Ifx\w+)\s*(\(|=|\[)")
COMMENT = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)


def configs(cproject):
    """(name, set of excluded paths) for every build configuration."""
    out = []
    for cfg in ET.parse(cproject).getroot().iter("configuration"):
        if cfg.get("artifactName") is None:
            continue
        excl = set()
        for e in cfg.iter("entry"):
            if e.get("kind") == "sourcePath" and e.get("name", "") == "":
                excl = set(p.strip().rstrip("/") for p in e.get("excluding", "").split("|") if p.strip())
        out.append((cfg.get("name", "?"), excl))
    return out


def excluded(rel, excl):
    return any(rel == e or rel.startswith(e + "/") for e in excl)


def read(path):
    with open(path, encoding="latin-1") as f:
        return COMMENT.sub("", f.read())


def package(cproject):
    """Device package of the build ("LQFP176"): selects the pin map file (IFX_PIN_PACKAGE_*)."""
    m = re.search(r'name="Device package"[^>]*value="(\w+)"', open(cproject, encoding="utf-8").read())
    return m.group(1).upper() if m else "LQFP176"


def index(pkg):
    """Ifx name -> .c file (repo-relative) that defines it (pin maps: the board's package)."""
    defs = {}
    for top in LIB_DIRS:
        for d, _, fs in os.walk(os.path.join(REPO, top)):
            for f in fs:
                if not f.endswith(".c"):
                    continue
                p = os.path.join(d, f)
                rel = os.path.relpath(p, REPO).replace("\\", "/")
                for line in read(p).splitlines():
                    if line.startswith(("#", "static ", "IFX_STATIC ")):
                        continue
                    m = DEF.match(line)
                    if m and not (m.group(2) == "(" and ";" in line):      # a prototype is no definition
                        old = defs.get(m.group(1))
                        if old is None or ("_" + pkg + ".c" in f and "_" + pkg + ".c" not in old):
                            defs[m.group(1)] = rel
    return defs


def used_by_firmware():
    names = set()
    files = [os.path.join("hal", f) for f in os.listdir(os.path.join(REPO, "hal")) if f.endswith(".c")]
    files += ["Cpu0_Main.c", "Cpu1_Main.c", "Cpu2_Main.c"]
    for f in files:
        names |= set(CALL.findall(read(os.path.join(REPO, f))))
    return names, True          # True: scanned, names without a .c may be header inlines / macros


def main(argv):
    cproject = os.path.join(REPO, ".cproject")
    symbols = None
    i = 0
    while i < len(argv):
        if argv[i] == "--symbols":
            symbols = argv[i + 1]; i += 2
        elif argv[i] == "--cproject":
            cproject = argv[i + 1]; i += 2
        else:
            print(__doc__); return 2
    if symbols:
        with open(symbols) as f:
            names = set(w for w in f.read().split() if w.startswith("Ifx"))
        loose = False
    else:
        names, loose = used_by_firmware()
    defs = index(package(cproject))
    bad = []
    need = {}                    # .c file -> first Ifx name that needs it
    todo = sorted(names)
    seen = set()
    while todo:
        n = todo.pop()
        if n in seen:
            continue
        seen.add(n)
        src = defs.get(n)
        if src is None:
            if n in names and not loose:
                bad.append("%s: no .c file under Libraries/ or Configurations/ defines it" % n)
            continue
        if src not in need:
            need[src] = n
            todo += [c for c in set(CALL.findall(read(os.path.join(REPO, src)))) if c not in seen]
    cfgs = configs(cproject)
    if not cfgs:
        bad.append("no build configuration found in %s" % cproject)
    for name, excl in cfgs:
        for src in sorted(need):
            if excluded(os.path.dirname(src), excl) or excluded(src, excl):
                bad.append("%s: %s is EXCLUDED (needed for %s)" % (name, src, need[src]))
    for b in bad:
        print("ADS IFX FAIL  " + b)
    if not bad:
        print("ADS iLLD sources OK: %d Ifx names used by hal/ + Cpu*_Main.c -> %d iLLD .c files, "
              "all present and built in %d configurations" % (len(names), len(need), len(cfgs)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
