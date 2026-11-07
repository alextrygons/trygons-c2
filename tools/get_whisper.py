"""Speech recognition for the talking manual (step AM2): a separate Python + faster-whisper + two Whisper models.

    C:\\msys64\\ucrt64\\bin\\python.exe tools\\get_whisper.py          (everything, ~3.5 GB)
    ... tools\\get_whisper.py --cpu-only                             (no CUDA libraries, no large model)

Everything goes into tools/bin/ (not in git, nothing installed system-wide):
  tools/bin/assistant-python/          the official Windows "embeddable" Python + pinned wheels
                                       (faster-whisper, CTranslate2, and the CUDA 12 cuBLAS / cuDNN 9 wheels)
  tools/bin/assistant/whisper/<name>/  the CTranslate2 Whisper models at a pinned revision, sha256 checked

An install lock marker tools/bin/assistant-python/trygons_install_lock.json records the sha256 of the requirements
file(s) and of the bootstrap: a missing / different marker = pip again; another bootstrap = installed afresh (AM2-r2
review LOW-2). A Windows Firewall outbound block rule for the assistant Python is added when allowed (an
administrator), else the ONE command is printed; --firewall = only that step (AM2-r2 review LOW-1).
A file that fails its sha256 is deleted and reported. Nothing is fetched at run time: the worker loads the
models from these folders only (offline).

Supply chain (AM2 review MED-4): the embeddable Python zip and get-pip.py (a get-pip COMMIT, not the moving
bootstrap URL) are sha256 checked before use; the packages are installed ONLY from the requirements files
tools/assistant_requirements.txt (+ tools/assistant_requirements_cuda.txt) with EVERY dependency pinned and
`--require-hashes` (a wheel with another hash, or an unpinned dependency = pip refuses the whole install).
The files are made by `get_whisper.py --make-requirements` from a working assistant Python (pip freeze --all +
the sha256 of every file of that version on PyPI) - review the diff before committing them.
On Ubuntu: `pip install --require-hashes -r tools/assistant_requirements.txt` in a venv (the files list every
platform's files of each version; a Linux-only dependency missing = pip refuses: then --make-requirements in
that venv) and set TRYGONS_ASSISTANT_PYTHON; then --models-only.
"""
import argparse
import hashlib
import io
import json
import os
import subprocess
import sys
import time
import urllib.request
import zipfile

PY_VERSION = "3.12.10"
PY_URL = "https://www.python.org/ftp/python/{v}/python-{v}-embed-amd64.zip"
PY_SHA = "4acbed6dd1c744b0376e3b1cf57ce906f9dc9e95e68824584c8099a63025a3c3"    # its md5 = python.org's published one
GET_PIP_COMMIT = "c7135ee10cd1b1fcb8fbf63d5d0004885cba7aa4"                    # pypa/get-pip: pip 26.3
GET_PIP = "https://raw.githubusercontent.com/pypa/get-pip/%s/public/get-pip.py" % GET_PIP_COMMIT
GET_PIP_SHA = "356e0d0de21cdc348c3394084d90939b85e4246c4d14c76aee6ca4f46646ea98"
PACKAGES = ["faster-whisper==1.2.1", "ctranslate2==4.6.2", "numpy==2.1.3"]     # the top level (the full set: REQ)
CUDA_PACKAGES = ["nvidia-cublas-cu12==12.8.4.1", "nvidia-cudnn-cu12==9.8.0.87"]
REQ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assistant_requirements.txt")
REQ_CUDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assistant_requirements_cuda.txt")
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


def get(url, digest=None):
    """Download into RAM; with digest: the sha256 must match (else nothing is used)."""
    print("downloading %s ..." % url, flush=True)
    with urllib.request.urlopen(url, timeout=120) as r:
        data = r.read()
    got = hashlib.sha256(data).hexdigest()
    if digest is not None and got != digest:
        raise SystemExit("FAILED: %s sha256 %s (expected %s) - not used" % (url, got, digest))
    return data


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


LOCK = os.path.join(PYDEST, "trygons_install_lock.json")
FW_RULE = "Trygons C2 assistant python - no network"


def req_files(cuda):
    return [REQ] + ([REQ_CUDA] if cuda else [])


def lock_marker(cuda):
    """AM2-r2 review LOW-2: what the install was made from - the sha256 of the requirements file(s) used and of the
    bootstrap (the embeddable zip, get-pip.py) - written after a successful install."""
    return {"version": 1, "cuda": bool(cuda), "python": PY_VERSION, "py_zip_sha256": PY_SHA,
            "get_pip_sha256": GET_PIP_SHA,
            "requirements": {os.path.basename(f): sha256(f) for f in req_files(cuda)}}


def install_state(cuda, dest=PYDEST):
    """("ok" | "missing" | "stale" | "bootstrap", why): "bootstrap" = the existing install's get-pip.py / recorded
    bootstrap sha256 differ from the pinned ones (= a fresh install from the checked zip); "stale" / "missing" =
    the lock marker differs / is absent (= pip again from the hash-locked requirements)."""
    exe = os.path.join(dest, "python.exe")
    if not os.path.exists(exe):
        return "missing", "no assistant Python"
    gp = os.path.join(dest, "get-pip.py")
    if not os.path.isfile(gp) or sha256(gp) != GET_PIP_SHA:
        return "bootstrap", "get-pip.py missing or not the pinned one"
    try:
        with open(os.path.join(dest, os.path.basename(LOCK)), encoding="utf-8") as f:
            have = json.load(f)
    except (OSError, ValueError):
        return "missing", "no install lock marker"
    if have.get("py_zip_sha256") != PY_SHA or have.get("get_pip_sha256") != GET_PIP_SHA or \
            have.get("python") != PY_VERSION:
        return "bootstrap", "made from another Python zip / get-pip.py"
    want = lock_marker(cuda)
    if have.get("requirements") != want["requirements"] or have.get("cuda") != want["cuda"]:
        return "stale", "the requirements changed since the install"
    return "ok", ""


def firewall_cmd(exe):
    """AM2-r2 review LOW-1: the OS-level outbound block for the assistant Python (Windows Firewall) - the
    Python-level blocker cannot see native code (CTranslate2 C++, hf_xet Rust)."""
    return 'netsh advfirewall firewall add rule name="%s" dir=out action=block program="%s" enable=yes' % (
        FW_RULE, os.path.abspath(exe))


def firewall_rule_present(exe):
    """True / False = the block rule for exe is there / not; None = can't tell (not Windows, netsh failed)."""
    if os.name != "nt":
        return None
    try:
        out = subprocess.run(["netsh", "advfirewall", "firewall", "show", "rule", "name=%s" % FW_RULE, "verbose"],
                             capture_output=True, text=True, timeout=20, errors="replace").stdout
    except (OSError, subprocess.SubprocessError):
        return None
    low = out.lower()
    return os.path.abspath(exe).lower() in low and "block" in low


def firewall(exe):
    """Add the block rule when allowed (an administrator); else print the ONE command for the owner."""
    if os.name != "nt":
        print("firewall: not Windows - block the assistant Python's network with your OS firewall (docs/"
              "assistant_design.md 2)")
        return None
    if firewall_rule_present(exe):
        print("firewall: the outbound block rule for %s is there" % exe)
        return True
    r = subprocess.run(firewall_cmd(exe), shell=True, capture_output=True, text=True, errors="replace")
    if r.returncode == 0 and firewall_rule_present(exe):
        print("firewall: outbound block rule added for %s" % exe)
        return True
    print("firewall: NOT added (needs an administrator). Please run this ONE command in an administrator "
          "terminal:\n  %s" % firewall_cmd(exe))
    return False


def get_python(cuda):
    exe = os.path.join(PYDEST, "python.exe")
    st, why = install_state(cuda)
    if st == "bootstrap":                             # never trust an install made from another bootstrap
        print("assistant Python: %s - installed again from the checked zip" % why)
        import shutil
        if os.path.abspath(PYDEST).startswith(os.path.join(HERE, "bin") + os.sep):
            shutil.rmtree(PYDEST)
    elif st == "ok":
        print("assistant Python: up to date (the lock marker matches the requirements)")
    else:
        print("assistant Python: %s - pip from the hash-locked requirements" % why)
    if not os.path.exists(exe):
        os.makedirs(PYDEST, exist_ok=True)
        with zipfile.ZipFile(io.BytesIO(get(PY_URL.format(v=PY_VERSION), PY_SHA))) as z:
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
            f.write(get(GET_PIP, GET_PIP_SHA))
        subprocess.check_call([exe, gp, "--no-warn-script-location", "--no-setuptools", "--no-wheel"])
    if st != "ok":
        if os.path.exists(LOCK):
            os.remove(LOCK)                           # no marker while pip runs (a cut install = installed again)
        subprocess.check_call(install_cmd(exe, cuda))
    out = subprocess.check_output([exe, "-c", "import faster_whisper, ctranslate2; print(faster_whisper.__version__,"
                                   " ctranslate2.__version__, ctranslate2.get_cuda_device_count())"], text=True)
    if st != "ok":
        with open(LOCK, "w", encoding="utf-8") as f:
            json.dump(lock_marker(cuda), f, indent=1)
    print("assistant Python ready: %s (faster-whisper / CTranslate2 / CUDA devices: %s)" % (exe, out.strip()))
    firewall(exe)
    return exe


def install_cmd(exe, cuda):
    """pip from the hash-locked requirements only (every dependency pinned, --require-hashes, wheels only)."""
    cmd = [exe, "-m", "pip", "install", "--no-warn-script-location", "--require-hashes", "--only-binary", ":all:",
           "-r", REQ]
    return cmd + (["-r", REQ_CUDA] if cuda else [])


def pypi_hashes(name, version):
    """sha256 of every file of name==version on PyPI (all platforms: the same list serves Windows + Ubuntu)."""
    d = json.loads(get("https://pypi.org/pypi/%s/%s/json" % (name, version)))
    return sorted({u["digests"]["sha256"] for u in d["urls"]})


def requirement(name, ver, hashes):
    return "%s==%s \\\n" % (name, ver) + " \\\n".join("    --hash=sha256:%s" % h for h in hashes) + "\n"


def make_requirements(exe):
    """tools/assistant_requirements(_cuda).txt from a WORKING assistant Python: pip freeze --all, every line
    pinned, every hash of that version (pip-compile --generate-hashes style); the nvidia-* wheels go into the
    CUDA file; pip / wheel are left out (pip = the hash-checked get-pip.py). Review the diff."""
    out = subprocess.check_output([exe, "-m", "pip", "freeze", "--all"], text=True)
    base, cuda = [], []
    for line in sorted(out.splitlines(), key=str.lower):
        line = line.strip()
        if not line:
            continue
        if line.startswith("#") or "==" not in line:
            raise SystemExit("not pinned: %r" % line)
        name, ver = line.split("==", 1)
        if name.lower() in ("pip", "wheel"):              # setuptools: ctranslate2 needs it
            continue
        (cuda if name.lower().startswith("nvidia-") else base).append(requirement(name, ver, pypi_hashes(name, ver)))
    head = ("# Talking manual (AM2): the assistant Python's packages - EVERY dependency pinned + hashed.\n"
            "# Made by: python tools/get_whisper.py --make-requirements (review the diff before committing).\n"
            "# Installed by tools/get_whisper.py with pip --require-hashes --only-binary :all:.\n")
    for path, rows in ((REQ, base), (REQ_CUDA, cuda)):
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(head + "".join(rows))
        print("wrote %s (%d packages)" % (path, len(rows)))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cpu-only", action="store_true", help="no CUDA wheels, only the small model")
    ap.add_argument("--models-only", action="store_true", help="only the models (Ubuntu: your own venv)")
    ap.add_argument("--make-requirements", action="store_true",
                    help="write tools/assistant_requirements*.txt from the assistant Python named by "
                         "TRYGONS_ASSISTANT_PYTHON (refused without it)")
    ap.add_argument("--firewall", action="store_true",
                    help="only add the Windows Firewall outbound block rule for the assistant Python (administrator)")
    a = ap.parse_args()
    if a.make_requirements:
        # AM2-r2 review LOW-3: only from the assistant Python named explicitly (never this Python by accident)
        exe = os.environ.get("TRYGONS_ASSISTANT_PYTHON")
        if not exe:
            print("REFUSED: --make-requirements needs TRYGONS_ASSISTANT_PYTHON = the WORKING assistant Python, e.g.\n"
                  "  TRYGONS_ASSISTANT_PYTHON=tools/bin/assistant-python/python.exe python tools/get_whisper.py "
                  "--make-requirements")
            return 2
        return make_requirements(exe)
    if a.firewall:
        exe = os.environ.get("TRYGONS_ASSISTANT_PYTHON") or os.path.join(PYDEST, "python.exe")
        return 0 if firewall(exe) else 1
    if not a.models_only:
        if os.name != "nt":
            print("on Ubuntu: pip install --require-hashes -r %s in a venv, set TRYGONS_ASSISTANT_PYTHON, "
                  "then --models-only" % REQ)
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
