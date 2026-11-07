"""Speech recognition for the talking manual (step AM2): a separate Python + faster-whisper + two Whisper models.

    C:\\msys64\\ucrt64\\bin\\python.exe tools\\get_whisper.py          (everything, ~3.5 GB)
    ... tools\\get_whisper.py --cpu-only                             (no CUDA libraries, no large model)

Everything goes into tools/bin/ (not in git, nothing installed system-wide):
  tools/bin/assistant-python/          the official Windows "embeddable" Python + pinned wheels
                                       (faster-whisper, CTranslate2, and the CUDA 12 cuBLAS / cuDNN 9 wheels)
  tools/bin/assistant/whisper/<name>/  the CTranslate2 Whisper models at a pinned revision, sha256 checked

A file that fails its sha256 is deleted and reported. Nothing is fetched at run time: the worker loads the
models from these folders only (offline).  On Ubuntu: pip install the PACKAGES into a venv and set
TRYGONS_ASSISTANT_PYTHON; the models are fetched by this script the same way.
"""
import argparse
import hashlib
import io
import os
import subprocess
import sys
import time
import urllib.request
import zipfile

PY_VERSION = "3.12.10"
PY_URL = "https://www.python.org/ftp/python/{v}/python-{v}-embed-amd64.zip"
GET_PIP = "https://bootstrap.pypa.io/get-pip.py"
PACKAGES = ["faster-whisper==1.2.1", "ctranslate2==4.6.2", "numpy==2.1.3"]
CUDA_PACKAGES = ["nvidia-cublas-cu12==12.8.4.1", "nvidia-cudnn-cu12==9.8.0.87"]
HF = "https://huggingface.co/{repo}/resolve/{rev}/{f}"
# name -> (repo, revision, [(file, size, sha256)])
MODELS = {
    "large-v3-turbo": ("dropbox-dash/faster-whisper-large-v3-turbo", "0a363e9161cbc7ed1431c9597a8ceaf0c4f78fcf", [
        ("config.json", 2263, "b0253ea6c0d3bea6b1e19e91a02acfd3b53f4467362efcb5a3e6b16c9b3a9b7e"),
        ("preprocessor_config.json", 340, "7ccc62c6f2765af1f3b46c00c9b5894426835a05021c8b9c01eecb6dfb542711"),
        ("tokenizer.json", 2710337, "297b13372ac43916285644fb9687add3cc62ee2a1adb60da3dc25cc94c1871fd"),
        ("vocabulary.json", 1068114, "c69260f2ab26d659b7c398f9a2b2b48ed0df16c3b47d7326782fd9cba71690c1"),
        ("model.bin", 1617884929, "e76620f83d5f5b69efd3d87e3dc180c1bd21df9fbebacfd4335e5e1efcc018da")]),
    "small": ("Systran/faster-whisper-small", "536b0662742c02347bc0e980a01041f333bce120", [
        ("config.json", 2370, "b55496ac7940a7ae47d2c01eab40edfd8701feec1229d9cce3b40014383fb828"),
        ("tokenizer.json", 2203239, "fb7b63191e9bb045082c79fd742a3106a12c99513ab30df4a0d47fa6cb6fd0ab"),
        ("vocabulary.txt", 459861, "34ce3fe1c5041027b3f8d42912270993f986dbc4bb34cf27f951e34a1e453913"),
        ("model.bin", 483546902, "3e305921506d8872816023e4c273e75d2419fb89b24da97b4fe7bce14170d671")]),
}
HERE = os.path.dirname(os.path.abspath(__file__))
PYDEST = os.path.join(HERE, "bin", "assistant-python")
MDEST = os.path.join(HERE, "bin", "assistant", "whisper")


def get(url):
    print("downloading %s ..." % url, flush=True)
    with urllib.request.urlopen(url, timeout=120) as r:
        return r.read()


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def fetch(url, path, size, digest):
    """Download to path (resumes nothing: a partial file is fetched again), then check size + sha256."""
    if os.path.isfile(path) and os.path.getsize(path) == size and sha256(path) == digest:
        return True
    tmp = path + ".part"
    for attempt in range(1, 6):                       # a dropped connection = try again (up to 5 times)
        print("downloading %s (%.1f MB) ..." % (url, size / 1e6), flush=True)
        try:
            with urllib.request.urlopen(url, timeout=120) as r, open(tmp, "wb") as f:
                while True:
                    b = r.read(1 << 20)
                    if not b:
                        break
                    f.write(b)
            break
        except OSError as e:
            print("  attempt %d failed: %s" % (attempt, e), flush=True)
            if attempt == 5:
                if os.path.exists(tmp):
                    os.remove(tmp)
                return False
            time.sleep(5 * attempt)
    got = sha256(tmp)
    if os.path.getsize(tmp) != size or got != digest:
        os.remove(tmp)
        print("FAILED: %s sha256 %s (expected %s) - deleted" % (os.path.basename(path), got, digest))
        return False
    os.replace(tmp, path)
    return True


def get_python(cuda):
    exe = os.path.join(PYDEST, "python.exe")
    if not os.path.exists(exe):
        os.makedirs(PYDEST, exist_ok=True)
        with zipfile.ZipFile(io.BytesIO(get(PY_URL.format(v=PY_VERSION)))) as z:
            z.extractall(PYDEST)
        for fn in os.listdir(PYDEST):                 # the embeddable Python ignores site-packages until "import site"
            if fn.endswith("._pth"):
                p = os.path.join(PYDEST, fn)
                with open(p, encoding="utf-8") as f:
                    t = f.read().replace("#import site", "import site")
                with open(p, "w", encoding="utf-8") as f:
                    f.write(t)
        gp = os.path.join(PYDEST, "get-pip.py")
        with open(gp, "wb") as f:
            f.write(get(GET_PIP))
        subprocess.check_call([exe, gp, "--no-warn-script-location"])
    subprocess.check_call([exe, "-m", "pip", "install", "--no-warn-script-location"] + PACKAGES +
                          (CUDA_PACKAGES if cuda else []))
    out = subprocess.check_output([exe, "-c", "import faster_whisper, ctranslate2; print(faster_whisper.__version__,"
                                   " ctranslate2.__version__, ctranslate2.get_cuda_device_count())"], text=True)
    print("assistant Python ready: %s (faster-whisper / CTranslate2 / CUDA devices: %s)" % (exe, out.strip()))
    return exe


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cpu-only", action="store_true", help="no CUDA wheels, only the small model")
    ap.add_argument("--models-only", action="store_true", help="only the models (Ubuntu: your own venv)")
    a = ap.parse_args()
    if not a.models_only:
        if os.name != "nt":
            print("on Ubuntu: pip install %s in a venv, set TRYGONS_ASSISTANT_PYTHON, then --models-only"
                  % " ".join(PACKAGES))
            return 1
        get_python(not a.cpu_only)
    ok = True
    for name, (repo, rev, files) in MODELS.items():
        if a.cpu_only and name != "small":
            continue
        d = os.path.join(MDEST, name)
        os.makedirs(d, exist_ok=True)
        for f, size, digest in files:
            ok = fetch(HF.format(repo=repo, rev=rev, f=f), os.path.join(d, f), size, digest) and ok
        print("model %s: %s" % (name, d if ok else "INCOMPLETE"))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
