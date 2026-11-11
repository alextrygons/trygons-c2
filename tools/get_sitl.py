"""Get ArduPilot's ArduCopter SITL (software in the loop) for the SITL test suite (CLAUDE.md V, step X2).

    python tools/get_sitl.py            download the pinned build for this PC into tools/bin/sitl/<platform>/
    python tools/get_sitl.py --check    is it installed and does every file still match its pinned sha256? (exit 0 / 1)
    python tools/get_sitl.py --where    print what works on this PC (Windows native / WSL / Linux)
    python tools/get_sitl.py --print-sha   (owner only) download WITHOUT installing and print the sha256 of every file:
                                         a new build is pinned by pasting these into PINS after reviewing them

tools/bin/ is never in git (.gitignore). Every file is PINNED by size + sha256: a download that differs from the pin is
deleted and REFUSED (an unverified binary is never run). gateway/toolpaths.py sitl() finds the installed build (and
re-checks the pins), gateway/test_sitl.py + tools/run_sitl.py use it.

What works where (measured 2026-11-11):
  Windows 11 (native, no WSL needed)  the Cygwin build ArduPilot publishes for Mission Planner's simulator
                                      (firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/: ArduCopter.elf +
                                      the Cygwin DLLs). Ran here: arms, flies GUIDED / AUTO / LOITER, 6 at once.
                                      NOTE: it is a ROLLING upload (no version in the URL): on 2026-11-11 it was
                                      ArduCopter V4.7.3-beta1 (the stable branch's next beta), not 4.7.2. When ArduPilot
                                      replaces it the sha256 differs = REFUSED until the owner re-pins (--print-sha).
  Linux x86-64 (Ubuntu 24.04)         the official 4.7.2 SITL build (firmware.ardupilot.org/Copter/stable-4.7.2/
                                      SITL_x86_64_linux_gnu/arducopter), version-pinned. Also usable inside WSL 2.
  WSL                                 not needed on Windows (the native build works); on this PC WSL is not installed.
The ArduPilot version = the drones' version: TO CONFIRM by the owner (W60); default = the latest stable 4.x (4.7.2;
Windows: the 4.7.3-beta1 Cygwin build until a 4.7.x stable Windows SITL build is published).
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
BIN = os.path.join(HERE, "bin", "sitl")
MARKER = "trygons_sitl.json"

WIN_URL = "https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/"
LIN_URL = "https://firmware.ardupilot.org/Copter/stable-4.7.2/SITL_x86_64_linux_gnu/"

# platform -> {"version", "url", "exe", "files": [(name, url-name, size, sha256)]}
PINS = {
    "windows": {
        "version": "ArduCopter V4.7.3-beta1 (Cygwin build 'CopterStable', pinned 2026-11-11)",
        "url": WIN_URL, "exe": "ArduCopter.elf",
        "files": [
            ("ArduCopter.elf", "ArduCopter.elf", 10997765,
             "d440df2672199b4b7a62e49983060e18a333b3f1f8886b2d67d1e0f929f054c5"),
            ("cygatomic-1.dll", "cygatomic-1.dll", 89614,
             "87f4cf57db1befe777b3e22e3cc4e81b2dbcff386a587e7f3d265f706340bb15"),
            ("cyggcc_s-1.dll", "cyggcc_s-1.dll", 111118,
             "002738f3cf6e984a8454af4204dd55d65780664d0fca8b4971a8f1e6c85ccd73"),
            ("cyggcc_s-seh-1.dll", "cyggcc_s-seh-1.dll", 142867,
             "f9a352fa9215ceb9a35b67ef8c7f178906a9dcc40b3819182b63f93fecb4faea"),
            ("cyggomp-1.dll", "cyggomp-1.dll", 159261,
             "46b4dd8bbc76ea82b50ac3f70397cd295896e23ae4bf0a1a7a013769599bf00c"),
            ("cygiconv-2.dll", "cygiconv-2.dll", 1104915,
             "e433ab6680eb509b5395f637137775dacebbc6e1ed3b564474aa87f0ca35379a"),
            ("cygintl-8.dll", "cygintl-8.dll", 108051,
             "9d0c063a226965e61664bdd9b771b5d0ee16b5214720b6f0086868e04fe67cdc"),
            ("cygquadmath-0.dll", "cygquadmath-0.dll", 509966,
             "75c70089e945bb5530eff463391383d6edc125b0b992ea29769a71c371d8c88e"),
            ("cygssp-0.dll", "cygssp-0.dll", 10766,
             "d32da41f13052d0fe4ee3fa1be11823425a8a73d1979df5a4e55280539009133"),
            ("cygstdc++-6.dll", "cygstdc%2B%2B-6.dll", 2316819,
             "a2d175169715f7fd69d40a581dc7c6502bc2bb75601b77a03baef6ce41da8f32"),
            ("cygwin1.dll", "cygwin1.dll", 3246645,
             "ed9d5379061e5de79c3375cc62246c2142ea1c97d94f4d16bfbea6fb1b7fc889"),
        ]},
    "linux": {
        "version": "ArduCopter V4.7.2 (official SITL_x86_64_linux_gnu, git fb77356)",
        "url": LIN_URL, "exe": "arducopter",
        "files": [
            ("arducopter", "arducopter", 7235704,
             "caed35e4cb796ec06cac40b09566a18251771486b1cbac5e2bf16b7a5ed5045d"),
        ]},
}


def plat():
    if os.name == "nt":
        return "windows"
    if sys.platform.startswith("linux"):
        return "linux"
    return None


def folder(p=None):
    return os.path.join(BIN, p or plat() or "none")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def verify(p=None):
    """(ok, why): every pinned file present with its size + sha256 (the marker alone is never trusted)."""
    p = p or plat()
    pin = PINS.get(p)
    if not pin:
        return False, "no SITL build for this platform (%s)" % sys.platform
    d = folder(p)
    for name, _u, size, sha in pin["files"]:
        f = os.path.join(d, name)
        if not os.path.isfile(f):
            return False, "not installed (%s missing: python tools/get_sitl.py)" % name
        if os.path.getsize(f) != size or sha256(f) != sha:
            return False, "%s does not match its pinned sha256 (python tools/get_sitl.py again)" % name
    return True, ""


def exe(p=None):
    """The verified SITL binary or None."""
    p = p or plat()
    ok, _why = verify(p)
    return os.path.join(folder(p), PINS[p]["exe"]) if ok else None


def _download(url):
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def install(p=None, out=print):
    p = p or plat()
    pin = PINS.get(p)
    if not pin:
        out("REFUSED: no pinned SITL build for this platform (%s); see --where" % sys.platform)
        return 2
    d = folder(p)
    tmp = d + ".part"
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp)
    try:
        for name, uname, size, sha in pin["files"]:
            data = _download(pin["url"] + uname)
            got = hashlib.sha256(data).hexdigest()
            if len(data) != size or got != sha:
                out("REFUSED: %s is %d bytes sha256 %s; pinned %d bytes %s. ArduPilot replaced the build: nothing "
                    "installed. The owner reviews and re-pins it (python tools/get_sitl.py --print-sha)." % (
                        name, len(data), got, size, sha))
                return 2
            with open(os.path.join(tmp, name), "wb") as f:
                f.write(data)
        if p == "linux":
            os.chmod(os.path.join(tmp, pin["exe"]), 0o755)
        with open(os.path.join(tmp, MARKER), "w", encoding="utf-8") as f:
            json.dump({"version": pin["version"], "files": {n: s for n, _u, _z, s in pin["files"]}}, f, indent=1)
        shutil.rmtree(d, ignore_errors=True)
        os.replace(tmp, d)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    ok, why = verify(p)
    out("SITL %s -> %s (%s)" % (pin["version"], d, "verified" if ok else why))
    return 0 if ok else 2


def print_sha(p=None, out=print):
    p = p or plat()
    pin = PINS.get(p)
    if not pin:
        out("no SITL source for %s" % sys.platform)
        return 2
    for name, uname, _size, _sha in pin["files"]:
        data = _download(pin["url"] + uname)
        out('("%s", "%s", %d, "%s"),' % (name, uname, len(data), hashlib.sha256(data).hexdigest()))
    out("(review the build first; nothing was installed)")
    return 0


def where(out=print):
    p = plat()
    out("this PC: %s" % (p or sys.platform))
    if p == "windows":
        out("  native Windows SITL: the Cygwin build (%s) - no WSL needed" % PINS["windows"]["version"])
        try:
            r = subprocess.run(["wsl", "--status"], capture_output=True, timeout=20)
            out("  WSL: %s" % ("installed (the Linux build could also run there)" if r.returncode == 0 else
                               "not installed (not needed)"))
        except (OSError, subprocess.SubprocessError):
            out("  WSL: not installed (not needed)")
    elif p == "linux":
        out("  Linux SITL: %s" % PINS["linux"]["version"])
    ok, why = verify(p) if p else (False, "no build")
    out("  installed: %s" % ("yes, verified" if ok else why))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--where", action="store_true")
    ap.add_argument("--print-sha", action="store_true")
    a = ap.parse_args()
    if a.where:
        return where()
    if a.check:
        ok, why = verify()
        print("SITL: %s" % ("OK (%s)" % PINS[plat()]["version"] if ok else why))
        return 0 if ok else 1
    if a.print_sha:
        return print_sha()
    return install()


if __name__ == "__main__":
    sys.exit(main())
