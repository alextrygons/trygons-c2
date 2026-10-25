"""Download Piper (offline neural text-to-speech) + a GREEK and an ENGLISH voice
into tools/bin/piper/ (not in git). The ground station speaks its callouts with it.

    python tools/get_piper.py            Windows build (this PC)
    python tools/get_piper.py --linux    linux x86_64 build

Piper: https://github.com/rhasspy/piper (MIT licence), works without internet.
Voices (https://huggingface.co/rhasspy/piper-voices, see each voice's MODEL_CARD):
  el_GR-rapunzelina-low   Greek
  en_GB-alan-medium       English
Pinned versions so every PC sounds the same; change them here to update.
"""
import io
import os
import sys
import tarfile
import urllib.request
import zipfile

VERSION = "2023.11.14-2"
URL = "https://github.com/rhasspy/piper/releases/download/{v}/{name}"
VOICE_URL = "https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/{path}"
VOICES = {"el": "el/el_GR/rapunzelina/low/el_GR-rapunzelina-low",
          "en": "en/en_GB/alan/medium/en_GB-alan-medium"}
HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(HERE, "bin", "piper")


def fetch(url):
    print("downloading %s ..." % url, flush=True)
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def main():
    linux = "--linux" in sys.argv or os.name != "nt"
    os.makedirs(os.path.join(DEST, "voices"), exist_ok=True)
    if linux:
        data = fetch(URL.format(v=VERSION, name="piper_linux_x86_64.tar.gz"))
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as t:
            for m in t.getmembers():
                name = m.name.split("/", 1)[1] if "/" in m.name else m.name
                if not name or ".." in name or name.startswith("/"):
                    continue
                out = os.path.join(DEST, name)
                if m.isdir():
                    os.makedirs(out, exist_ok=True)
                elif m.isfile():
                    os.makedirs(os.path.dirname(out), exist_ok=True)
                    with open(out, "wb") as f:
                        f.write(t.extractfile(m).read())
        exe = os.path.join(DEST, "piper")
        if os.path.exists(exe):
            os.chmod(exe, 0o755)
    else:
        data = fetch(URL.format(v=VERSION, name="piper_windows_amd64.zip"))
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            for n in z.namelist():
                name = n.split("/", 1)[1] if "/" in n else n
                if not name or ".." in name or name.endswith("/"):
                    continue
                out = os.path.join(DEST, name)
                os.makedirs(os.path.dirname(out), exist_ok=True)
                with open(out, "wb") as f:
                    f.write(z.read(n))
    for lang, path in VOICES.items():
        for ext in (".onnx", ".onnx.json"):
            out = os.path.join(DEST, "voices", os.path.basename(path) + ext)
            if not os.path.exists(out):
                with open(out, "wb") as f:
                    f.write(fetch(VOICE_URL.format(path=path + ext)))
        print("voice %s: %s" % (lang, os.path.basename(path)))
    print("Piper %s -> %s" % (VERSION, DEST))
    print("Restart the ground station; the Sound tab shows the voices it found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
