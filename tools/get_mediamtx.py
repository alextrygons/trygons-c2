"""Download MediaMTX (the video relay) into tools/bin/mediamtx/ (not in git).

    python tools/get_mediamtx.py            the pinned version for this PC
    python tools/get_mediamtx.py --arm64    the Raspberry Pi 5 build (linux arm64)

MediaMTX: https://github.com/bluenviron/mediamtx (MIT licence). The version
is pinned so every PC and the Pi run the same one; change VERSION to update.
"""
import io
import os
import platform
import sys
import tarfile
import urllib.request
import zipfile

VERSION = "v1.21.2"
URL = "https://github.com/bluenviron/mediamtx/releases/download/{v}/mediamtx_{v}_{plat}.{ext}"
HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(HERE, "bin", "mediamtx")


def target(arm64=False):
    if arm64:
        return "linux_arm64", "tar.gz"
    if os.name == "nt":
        return "windows_amd64", "zip"
    m = platform.machine().lower()
    return ("linux_arm64" if m in ("aarch64", "arm64") else "linux_amd64"), "tar.gz"


def main():
    arm64 = "--arm64" in sys.argv
    plat, ext = target(arm64)
    url = URL.format(v=VERSION, plat=plat, ext=ext)
    dest = DEST + ("_arm64" if arm64 else "")
    os.makedirs(dest, exist_ok=True)
    print("downloading %s ..." % url)
    with urllib.request.urlopen(url, timeout=60) as r:
        data = r.read()
    if ext == "zip":
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            for n in z.namelist():
                if os.path.basename(n) in ("mediamtx.exe", "mediamtx", "LICENSE"):
                    with open(os.path.join(dest, os.path.basename(n)), "wb") as f:
                        f.write(z.read(n))
    else:
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as t:
            for m in t.getmembers():
                if os.path.basename(m.name) in ("mediamtx", "LICENSE") and m.isfile():
                    with open(os.path.join(dest, os.path.basename(m.name)), "wb") as f:
                        f.write(t.extractfile(m).read())
            exe = os.path.join(dest, "mediamtx")
            if os.path.exists(exe):
                os.chmod(exe, 0o755)
    print("MediaMTX %s -> %s" % (VERSION, dest))
    return 0


if __name__ == "__main__":
    sys.exit(main())
