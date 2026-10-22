"""A separate Python for the obstacle detection (gateway/vision) on a Windows PC.

    C:\\msys64\\ucrt64\\bin\\python.exe tools\\get_vision_python.py

MSYS2's onnxruntime package can't be imported (no Python module inside), so
the vision process runs with the official Windows "embeddable" Python from
python.org plus the official onnxruntime / numpy wheels, all inside
tools/bin/vision-python/ (not in git, nothing installed system-wide).
On the Raspberry Pi the normal Python is used instead (gateway/vision/README.md).
Versions are pinned (same results on every PC).
"""
import io
import os
import subprocess
import sys
import urllib.request
import zipfile

PY_VERSION = "3.12.10"
PY_URL = "https://www.python.org/ftp/python/{v}/python-{v}-embed-amd64.zip"
GET_PIP = "https://bootstrap.pypa.io/get-pip.py"
PACKAGES = ["onnxruntime==1.20.1", "numpy==2.1.3"]
HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(HERE, "bin", "vision-python")


def get(url):
    print("downloading %s ..." % url, flush=True)
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def main():
    if os.name != "nt":
        print("only for Windows: on the Pi use  pip install %s" % " ".join(PACKAGES))
        return 1
    exe = os.path.join(DEST, "python.exe")
    if not os.path.exists(exe):
        os.makedirs(DEST, exist_ok=True)
        with zipfile.ZipFile(io.BytesIO(get(PY_URL.format(v=PY_VERSION)))) as z:
            z.extractall(DEST)
        # the embeddable Python ignores site-packages until "import site" is enabled
        for fn in os.listdir(DEST):
            if fn.endswith("._pth"):
                p = os.path.join(DEST, fn)
                with open(p, encoding="utf-8") as f:
                    t = f.read().replace("#import site", "import site")
                with open(p, "w", encoding="utf-8") as f:
                    f.write(t)
        gp = os.path.join(DEST, "get-pip.py")
        with open(gp, "wb") as f:
            f.write(get(GET_PIP))
        subprocess.check_call([exe, gp, "--no-warn-script-location"])
    subprocess.check_call([exe, "-m", "pip", "install", "--no-warn-script-location"] + PACKAGES)
    out = subprocess.check_output([exe, "-c", "import onnxruntime, numpy; print(onnxruntime.__version__, "
                                   "numpy.__version__)"], text=True)
    print("vision Python ready: %s (onnxruntime / numpy %s)" % (exe, out.strip()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
