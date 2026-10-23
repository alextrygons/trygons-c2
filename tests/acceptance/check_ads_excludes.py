"""The ADS (TriCore) build compiles every C/C++ file in the project folder
except what .cproject excludes. Only the board firmware may be compiled:
Cpu*_Main.c (top level), app/, hal/ (hal_tc375.c), Libraries/, Configurations/.

    python tests/acceptance/check_ads_excludes.py

FAILS (exit 1) if, in any build configuration of .cproject, a top-level
folder that is not board firmware is not excluded - e.g. a new folder that
was added without excluding it (CLAUDE.md: "Any new top-level folder must be
excluded from the ADS build in .cproject").
"""
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
FIRMWARE = {"app", "hal", "Libraries", "Configurations"}


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
    if not configs:
        bad.append("no build configuration found in .cproject")
    for b in bad:
        print("ADS EXCLUDES FAIL  " + b)
    if not bad:
        print("ADS excludes OK: %d configurations exclude every non-firmware folder (%s)" % (
            len(configs), ", ".join(folders)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
