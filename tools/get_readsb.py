"""Get the ADS-B decoder (readsb / dump1090) for the gateway's 1090 MHz receiver (CLAUDE.md X K4, step PF5).

    Ubuntu:   sudo apt install readsb            (or dump1090-fa); gateway.json "adsb": {"decoder": "auto"} finds it
    Windows:  python tools/get_readsb.py --url <zip of a dump1090 / readsb Windows build> --sha256 <hash>
                                          puts it into tools/bin/readsb/ (not in git)

A download is PINNED: without --sha256 nothing is installed (an unverified binary is never run on the gateway). The
Windows build to use is TO CONFIRM (W132: the owner picks a build and its hash; there is no official readsb release
for Windows). The RTL-SDR driver (WinUSB via Zadig) is installed by hand.

The decoder is only started by the gateway's ADS-B worker (gateway/adsb_worker.py), which reads its SBS port 30003 and
never sends anything anywhere.
"""
import argparse
import hashlib
import io
import os
import sys
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(HERE, "bin", "readsb")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="")
    ap.add_argument("--sha256", default="")
    a = ap.parse_args()
    if not a.url:
        print(__doc__)
        return 0 if sys.platform.startswith("linux") else 2
    if len(a.sha256) != 64:
        print("REFUSED: give the pinned --sha256 (64 hex characters) of the download.")
        return 2
    with urllib.request.urlopen(a.url, timeout=60) as r:
        data = r.read()
    got = hashlib.sha256(data).hexdigest()
    if got.lower() != a.sha256.lower():
        print("REFUSED: sha256 is %s, expected %s" % (got, a.sha256))
        return 2
    os.makedirs(DEST, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        for n in z.namelist():
            base = os.path.basename(n)
            if base and not n.endswith("/") and (base.lower().endswith((".exe", ".dll", ".txt")) or "licen" in base.lower()):
                with open(os.path.join(DEST, base), "wb") as f:
                    f.write(z.read(n))
    print("ADS-B decoder -> %s" % DEST)
    return 0


if __name__ == "__main__":
    sys.exit(main())
