"""The ADS (TriCore) build compiles every C/C++ file in the project folder
except what .cproject excludes. Only the board firmware may be compiled:
Cpu*_Main.c (top level), app/, hal/ (hal_tc375.c), Libraries/, Configurations/.

    python tests/acceptance/check_ads_excludes.py

FAILS (exit 1) if, in any build configuration of .cproject, a top-level
folder that is not board firmware is not excluded - e.g. a new folder that
was added without excluding it (CLAUDE.md: "Any new top-level folder must be
excluded from the ADS build in .cproject") - or if, walking the project the
way ADS does, any C / C++ / assembler source outside the board firmware
would be compiled (e.g. a stray .c file in the project root).
"""
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
FIRMWARE = {"app", "hal", "Libraries", "Configurations"}
ROOT_SOURCES = {"Cpu0_Main.c", "Cpu1_Main.c", "Cpu2_Main.c"}
SOURCE_EXT = (".c", ".cpp", ".cc", ".cxx", ".s", ".S", ".asm")


def compiled_sources(excl, outputs):
    """Sources ADS would compile with these exclusions that are NOT board firmware."""
    bad = []
    for d, ds, fs in os.walk(REPO):
        rel = os.path.relpath(d, REPO).replace("\\", "/")
        top = rel.split("/")[0]
        if rel != "." and (top.startswith(".") or top in outputs or
                           any(rel == e or rel.startswith(e + "/") for e in excl)):
            ds[:] = []
            continue
        for f in fs:
            if not f.endswith(SOURCE_EXT):
                continue
            if rel == "." and f in ROOT_SOURCES:
                continue
            if top in FIRMWARE:
                continue
            bad.append(f if rel == "." else rel + "/" + f)
    return bad


def main():
    root = ET.parse(os.path.join(REPO, ".cproject")).getroot()
    configs = []
    for cfg in root.iter("configuration"):
        if cfg.get("artifactName") is None:
            continue
        name = cfg.get("name", "?")
        excl = None
        for se in cfg.iter("sourceEntries"):
            for e in se.iter("entry"):
                if e.get("kind") == "sourcePath" and e.get("name", "") == "":
                    excl = set(p.strip().rstrip("/") for p in e.get("excluding", "").split("|") if p.strip())
        configs.append((name, excl))
    outputs = {c[0] for c in configs}                     # ADS build output folders ("TriCore Debug (TASKING)")
    folders = sorted(d for d in os.listdir(REPO) if os.path.isdir(os.path.join(REPO, d))
                     and not d.startswith(".") and d not in FIRMWARE and d not in outputs)
    bad = []
    for name, excl in configs:
        if excl is None:
            bad.append("%s: no project source entry found" % name)
            continue
        missing = [d for d in folders if d not in excl]
        if missing:
            bad.append("%s: not excluded: %s" % (name, ", ".join(missing)))
        stray = compiled_sources(excl, outputs)
        if stray:
            bad.append("%s: would compile non-firmware sources: %s" % (name, ", ".join(stray[:8])))
    if not configs:
        bad.append("no build configuration found in .cproject")
    for b in bad:
        print("ADS EXCLUDES FAIL  " + b)
    if not bad:
        print("ADS excludes OK: %d configurations exclude every non-firmware folder (%s); only "
              "Cpu*_Main.c, app/, hal/, Libraries/, Configurations/ are compiled" % (len(configs), ", ".join(folders)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
