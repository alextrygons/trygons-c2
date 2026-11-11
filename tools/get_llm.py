"""The talking manual's answer step (step AM4): llama.cpp's Python bindings + the local language models.

    C:\\msys64\\ucrt64\\bin\\python.exe tools\\get_llm.py               (the bindings + the CHOSEN model: ~0.5 GB +
                                                                      2.5 GB = ~3 GB)
    ... tools\\get_llm.py --all                                       (every LICENCE-CLEAN candidate of the comparison)
    ... tools\\get_llm.py --model phi4-mini                           (one model by name)
    ... tools\\get_llm.py --all --allow-restricted                    (+ the Llama Community / Qwen Research models:
                                                                      comparison only, a warning is printed)
    ... tools\\get_llm.py --list                                      (the candidates, their files and licences)

Licences (step AM7, owner W135; docs/licences.md): only MIT / Apache-2.0 models are "clean" and downloaded by default;
a model under the Llama 3.x Community or the Qwen Research licence is downloaded ONLY with --allow-restricted and is
never the default (CHOSEN is always clean - test AS23).
    ... tools\\get_llm.py --check                                     (what is installed; downloads nothing)

Everything goes into tools/bin/ (not in git, nothing installed system-wide):
  tools/bin/assistant/llm/py/          llama-cpp-python (CUDA 12.4 build) + its small dependencies, installed with
                                       pip --target (the assistant Python of tools/get_whisper.py runs it; its own
                                       lock is not touched)
  tools/bin/assistant/llm/<name>/      one GGUF file per model at a PINNED Hugging Face revision, sha256 checked

How llama.cpp runs (CLAUDE.md W, AM1-r L4): INSIDE the assistant worker through these Python bindings - no exe,
no llama-cli, NEVER llama-server, never a TCP port. The worker is started only by groundstation/assistant/client.py.

Supply chain (like get_whisper.py): the llama-cpp-python wheel is a GitHub release asset (abetlen/llama-cpp-python,
tag v0.3.36-cu124, 2026-10-01) - its size + sha256 (= the release's own published digest) are checked before pip
sees it; its dependencies come ONLY from tools/assistant_requirements_llm.txt (pinned, every hash, --require-hashes
--only-binary :all: --no-deps). A lock marker tools/bin/assistant/llm/py/trygons_llm_lock.json records the wheel +
requirements sha256: missing / different = installed afresh. A file that fails its sha256 is deleted and reported.
Nothing is fetched at run time (the worker loads a local .gguf file only, offline).
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
DEST = os.path.join(HERE, "bin", "assistant", "llm")
PYT = os.path.join(DEST, "py")
LOCK = os.path.join(PYT, "trygons_llm_lock.json")
REQ = os.path.join(HERE, "assistant_requirements_llm.txt")
APY = os.path.join(HERE, "bin", "assistant-python", "python.exe")

WHEEL = ("https://github.com/abetlen/llama-cpp-python/releases/download/v0.3.36-cu124/"
         "llama_cpp_python-0.3.36-py3-none-win_amd64.whl", 511381574,
         "7086b98420cf9cca5e3f14181edc4332038bfb0a0e380370a693806bb527911f")
HF = "https://huggingface.co/{repo}/resolve/{rev}/{f}"

# name -> (repo, revision, file, size, sha256 (= the Hugging Face LFS oid), licence, note). Q4_K_M everywhere.
MODELS = {
    "qwen2.5-7b": ("bartowski/Qwen2.5-7B-Instruct-GGUF", "8911e8a47f92bac19d6f5c64a2e2095bd2f7d031",
                   "Qwen2.5-7B-Instruct-Q4_K_M.gguf", 4683074240,
                   "65b8fcd92af6b4fefa935c625d1ac27ea29dcb6ee14589c55a8f115ceaaa1423", "Apache-2.0",
                   "Qwen2.5-7B-Instruct"),
    "meltemi-7b": ("mradermacher/Meltemi-7B-Instruct-v1.5-GGUF", "cd3ac27843783aef6d6a4628a73b2e36630b7450",
                   "Meltemi-7B-Instruct-v1.5.Q4_K_M.gguf", 4535764352,
                   "15d0ad45f6c8911eadfae8f0c3a8a7f59b8053332bc99f76cf146f88f975ec14", "Apache-2.0",
                   "ILSP Meltemi-7B-Instruct v1.5 (Greek)"),
    "krikri-8b": ("ilsp/Llama-Krikri-8B-Instruct-GGUF", "3c1ce101a3e0d783a0f2223f212ccc35b4657532",
                  "llama-krikri-8b-instruct-q4_k_m.gguf", 5041147264,
                  "e3f93427fe0fcc016e31ed4bfbc1d10916fb97f6b2b3298445196490d315376f", "Llama 3.1 Community",
                  "ILSP Llama-Krikri-8B-Instruct (Greek), the official ILSP GGUF"),
    "llama3.1-8b": ("bartowski/Meta-Llama-3.1-8B-Instruct-GGUF", "bf5b95e96dac0462e2a09145ec66cae9a3f12067",
                    "Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf", 4920739232,
                    "7b064f5842bf9532c91456deda288a1b672397a54fa729aa665952863033557c", "Llama 3.1 Community",
                    "Meta Llama-3.1-8B-Instruct"),
    "qwen2.5-3b": ("Qwen/Qwen2.5-3B-Instruct-GGUF", "7dabda4d13d513e3e842b20f0d435c732f172cbe",
                   "qwen2.5-3b-instruct-q4_k_m.gguf", 2104932768,
                   "626b4a6678b86442240e33df819e00132d3ba7dddfe1cdc4fbb18e0a9615c62d", "Qwen Research",
                   "Qwen2.5-3B-Instruct (the 3B option), the official Qwen GGUF"),
    "llama3.2-3b": ("bartowski/Llama-3.2-3B-Instruct-GGUF", "5ab33fa94d1d04e903623ae72c95d1696f09f9e8",
                    "Llama-3.2-3B-Instruct-Q4_K_M.gguf", 2019377696,
                    "6c1a2b41161032677be168d354123594c0e6e67d2b9227c84f296ad037c728ff", "Llama 3.2 Community",
                    "Meta Llama-3.2-3B-Instruct (the 3B option)"),
    # step AM7 (owner W135): licence-clean candidates (MIT / Apache-2.0) that fit beside Whisper on a 6 GB GPU
    "phi4-mini": ("bartowski/microsoft_Phi-4-mini-instruct-GGUF", "7ff82c2aaa4dde30121698a973765f39be5288c0",
                  "microsoft_Phi-4-mini-instruct-Q4_K_M.gguf", 2491874688,
                  "01999f17c39cc3074afae5e9c539bc82d45f2dd7faa3917c66cbef76fce8c0c2", "MIT",
                  "Microsoft Phi-4-mini-instruct (3.8B)"),
    "phi3.5-mini": ("bartowski/Phi-3.5-mini-instruct-GGUF", "6d70da17e749a471ccb62ade694486011a75cda3",
                    "Phi-3.5-mini-instruct-Q4_K_M.gguf", 2393232672,
                    "e4165e3a71af97f1b4820da61079826d8752a2088e313af0c7d346796c38eff5", "MIT",
                    "Microsoft Phi-3.5-mini-instruct (3.8B)"),
    "qwen2.5-1.5b": ("Qwen/Qwen2.5-1.5B-Instruct-GGUF", "91cad51170dc346986eccefdc2dd33a9da36ead9",
                     "qwen2.5-1.5b-instruct-q4_k_m.gguf", 1117320736,
                     "6a1a2eb6d15622bf3c96857206351ba97e1af16c30d7a74ee38970e434e9407e", "Apache-2.0",
                     "Qwen2.5-1.5B-Instruct, the official Qwen GGUF"),
    "qwen3-4b": ("bartowski/Qwen_Qwen3-4B-Instruct-2507-GGUF", "ae44f08e1392f39c0e474af10c3ff8355c8b6688",
                 "Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf", 2497280736,
                 "2fde00ce69dd4899c70d020845e2638353015bba0fdf161b3eb965f2bca4464e", "Apache-2.0",
                 "Qwen3-4B-Instruct-2507 (no thinking mode)"),
    "ministral3-3b": ("mistralai/Ministral-3-3B-Instruct-2512-GGUF", "eb599d408350ea2bb60452cb86be7c7b2fc28227",
                      "Ministral-3-3B-Instruct-2512-Q4_K_M.gguf", 2147023008,
                      "9ed150d4367e68df0ac8e1540f6ddc65b42d0ee26378329d1ecbca60f93fc5f8", "Apache-2.0",
                      "Mistral Ministral-3-3B-Instruct-2512, the official Mistral GGUF (text only, no mmproj)"),
    "eurollm-1.7b": ("mradermacher/EuroLLM-1.7B-Instruct-GGUF", "2951f08f66429c934c8b01a94347161362430808",
                     "EuroLLM-1.7B-Instruct.Q4_K_M.gguf", 1045157088,
                     "1cade17f491ea46a686dbee51fbd52442e0f001f102380c3b9d66b4a77f84093", "Apache-2.0",
                     "utter-project EuroLLM-1.7B-Instruct (EU languages incl. Greek)"),
}
# AM7 (owner W135): only these licences are "clean" (commercial use, no use policy / user cap / attribution rule
# beyond the licence text); the others (Llama Community, Qwen Research) are downloaded ONLY with --allow-restricted
# and are never the default (docs/licences.md)
CLEAN_LICENCES = ("MIT", "Apache-2.0")


def licence_clean(name):
    return MODELS[name][5] in CLEAN_LICENCES


# the model the comparison chose (docs/assistant_design.md 5b, AM7 5n) = what a plain run downloads and the worker
# loads; ALWAYS a licence-clean one (AS23 checks it). AM7: Qwen3-4B-Instruct-2507 (Apache-2.0) replaces llama3.2-3b
CHOSEN = "qwen3-4b"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def fetch(url, path, size, digest):
    """Download to path (a dropped connection = up to 5 attempts), then check size + sha256 (else deleted)."""
    if os.path.isfile(path) and os.path.getsize(path) == size and sha256(path) == digest:
        print("ok (sha256 checked): %s" % path, flush=True)
        return True
    tmp = path + ".part"
    for attempt in range(1, 6):
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


def lock_marker():
    return {"version": 1, "wheel": WHEEL[0], "wheel_sha256": WHEEL[2], "requirements_sha256": sha256(REQ)}


def install_state():
    """("ok" | "missing" | "stale", why) of tools/bin/assistant/llm/py."""
    if not os.path.isdir(os.path.join(PYT, "llama_cpp")):
        return "missing", "no llama_cpp bindings"
    try:
        with open(LOCK, encoding="utf-8") as f:
            have = json.load(f)
    except (OSError, ValueError):
        return "missing", "no lock marker"
    if have != lock_marker():
        return "stale", "the wheel / requirements changed since the install"
    return "ok", ""


def install_bindings(exe):
    st, why = install_state()
    if st == "ok":
        print("llama.cpp bindings: up to date (%s)" % PYT)
        return True
    print("llama.cpp bindings: %s - installed afresh" % why)
    if os.path.isdir(PYT) and os.path.abspath(PYT).startswith(os.path.join(HERE, "bin") + os.sep):
        shutil.rmtree(PYT)
    dl = os.path.join(DEST, "dl")
    os.makedirs(dl, exist_ok=True)
    whl = os.path.join(dl, WHEEL[0].rsplit("/", 1)[1])
    if not fetch(WHEEL[0], whl, WHEEL[1], WHEEL[2]):
        return False
    base = [exe, "-m", "pip", "install", "--no-warn-script-location", "--target", PYT, "--no-deps",
            "--only-binary", ":all:", "--disable-pip-version-check"]
    subprocess.check_call(base + ["--require-hashes", "-r", REQ])
    subprocess.check_call(base + ["--no-index", whl])          # sha256 checked above (fetch)
    with open(LOCK, "w", encoding="utf-8") as f:
        json.dump(lock_marker(), f, indent=1)
    print("llama.cpp bindings installed: %s" % PYT)
    return True


def model_path(name):
    repo, rev, fn, size, digest, lic, note = MODELS[name]
    return os.path.join(DEST, name, fn)


def get_model(name):
    repo, rev, fn, size, digest, lic, note = MODELS[name]
    os.makedirs(os.path.join(DEST, name), exist_ok=True)
    print("model %s = %s (licence %s)" % (name, note, lic))
    return fetch(HF.format(repo=repo, rev=rev, f=fn), model_path(name), size, digest)


def check(verify=False):
    """What is installed (size check; --verify also the sha256 of every present model = minutes for 23 GB)."""
    st, why = install_state()
    print("bindings: %s %s" % (st, why))
    bad = 0
    for n in MODELS:
        p = model_path(n)
        ok = os.path.isfile(p) and os.path.getsize(p) == MODELS[n][3]
        extra = ""
        if ok and verify:
            good = sha256(p) == MODELS[n][4]
            extra = "  sha256 %s" % ("ok" if good else "MISMATCH")
            bad += not good
        print("%-12s %s %s%s" % (n, "present" if ok else "missing", p if ok else "", extra))
    return 0 if st == "ok" and not bad else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--all", action="store_true", help="every candidate model (the comparison)")
    ap.add_argument("--model", action="append", default=[], help="one model by name (repeatable)")
    ap.add_argument("--models-only", action="store_true", help="no bindings (Ubuntu: pip install llama-cpp-python)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--verify", action="store_true", help="with --check: the sha256 of every present model too")
    ap.add_argument("--allow-restricted", action="store_true",
                    help="also models under a RESTRICTED licence (Llama Community, Qwen Research): comparison only, "
                         "never the default - see docs/licences.md")
    a = ap.parse_args(argv)
    if a.list:
        for n, (repo, rev, fn, size, digest, lic, note) in MODELS.items():
            print("%-14s %5.2f GB  %-22s %-10s %s%s" % (n, size / 1e9, lic, "clean" if licence_clean(n) else
                                                        "RESTRICTED", note, "   <- chosen" if n == CHOSEN else ""))
        return 0
    if a.check:
        return check(a.verify)
    for n in a.model:
        if n not in MODELS:
            print("unknown model %r (known: %s)" % (n, ", ".join(MODELS)))
            return 2
    restricted = [n for n in a.model if not licence_clean(n)]
    if restricted and not a.allow_restricted:
        print("REFUSED: %s under a restricted licence (%s) - only with --allow-restricted (comparison only, never "
              "the default; docs/licences.md)" % (", ".join(restricted), ", ".join(MODELS[n][5] for n in restricted)))
        return 2
    if not a.models_only:
        if os.name != "nt":
            print("on Ubuntu: in the assistant venv install the SAME pinned versions - the wheel llama-cpp-python==%s "
                  "(a CUDA 12.4 build of its release v0.3.36, not a plain `pip install llama-cpp-python`) and the "
                  "hash-locked dependencies: pip install --require-hashes --only-binary :all: --no-deps -r %s; then "
                  "--models-only (the model files are sha256-checked the same way)" % (
                      "0.3.36", os.path.relpath(REQ, os.path.dirname(HERE))))
            return 1
        exe = os.environ.get("TRYGONS_ASSISTANT_PYTHON") or APY
        if not os.path.isfile(exe):
            print("no assistant Python: run tools/get_whisper.py first")
            return 1
        if not install_bindings(exe):
            return 2
    if a.all:
        names = [n for n in MODELS if licence_clean(n) or a.allow_restricted]
        skipped = [n for n in MODELS if n not in names]
        if skipped:
            print("not downloaded (restricted licence; --allow-restricted to include): %s" % ", ".join(skipped))
    else:
        names = a.model or [CHOSEN]
    for n in names:
        if not licence_clean(n):
            print("WARNING: %s is under the %s licence - NOT licence-clean (use policy / attribution / limits; "
                  "docs/licences.md). For the comparison only, never the station default." % (n, MODELS[n][5]))
    ok = True
    for n in names:
        ok = get_model(n) and ok
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
