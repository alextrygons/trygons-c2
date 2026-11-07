"""Where the talking manual's files are (step AM2) - path helpers only, no logic, no network.

    assistant Python   $TRYGONS_ASSISTANT_PYTHON, <repo>/tools/bin/assistant-python/ (tools/get_whisper.py,
                       Windows), else this Python (Ubuntu: pip install from tools/assistant_requirements.txt -
                       faster-whisper==1.2.1 pinned + hashed; never an unpinned `pip install faster-whisper`)
    language model     $TRYGONS_LLM_DIR, <repo>/tools/bin/assistant/llm/<name>/<file>.gguf (tools/get_llm.py);
                       the worker loads a model file ONLY after its sha256 matches LLM_PINNED (AM4-r LOW-2) - an
                       unpinned file needs the explicit per-PC setting assistant_llm_unpinned (W108)
    Whisper models     $TRYGONS_WHISPER_DIR, <repo>/tools/bin/assistant/whisper/<name>/ (tools/get_whisper.py)
    gateway PC         gateway.json in the gateway's config folder (gateway/osdep.py) = the assistant never runs

Kept in groundstation/ (AM1 review M1): nothing of the assistant lives in gateway/.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GS = os.path.dirname(HERE)                               # groundstation/
REPO = os.path.dirname(GS)
EXE = ".exe" if os.name == "nt" else ""

# the Whisper models (CTranslate2 form): GPU = large-v3-turbo, CPU fallback = small (AM2 measurement,
# docs/assistant_design.md section 3)
MODEL_GPU = "large-v3-turbo"
MODEL_CPU = "small"
MODEL_FILES = ("config.json", "model.bin", "tokenizer.json")


def bin_dir():
    return os.path.join(REPO, "tools", "bin")


def assistant_python():
    """The Python that runs the worker (needs faster-whisper); None = missing."""
    for c in (os.environ.get("TRYGONS_ASSISTANT_PYTHON"),
              os.path.join(bin_dir(), "assistant-python", "python" + EXE)):
        if c and os.path.isfile(c):
            return os.path.abspath(c)
    if os.name != "nt":
        return sys.executable
    return None


def whisper_root():
    return os.environ.get("TRYGONS_WHISPER_DIR") or os.path.join(bin_dir(), "assistant", "whisper")


def whisper_model(name):
    """The local folder of one Whisper model, or None when it is not (completely) downloaded."""
    d = os.path.join(whisper_root(), name)
    if all(os.path.isfile(os.path.join(d, f)) for f in MODEL_FILES):
        return os.path.abspath(d)
    return None


def models_installed():
    return assistant_python() is not None and (whisper_model(MODEL_GPU) or whisper_model(MODEL_CPU)) is not None


def gateway_config_paths(env=None):
    """Where a gateway PC keeps its config (gateway/osdep.py config_dir, both OSes) - like academy/simguard.py."""
    env = os.environ if env is None else env
    win = os.path.join(env.get("PROGRAMDATA") or r"C:\ProgramData", "trygons", "gateway.json")
    extra = env.get("TRYGONS_GATEWAY_CONFIG")             # tests: a pretend gateway PC
    return ([extra] if extra else []) + [win, "/etc/trygons/gateway.json"]


def on_gateway_pc(env=None):
    """True on a boat's gateway PC: the assistant never runs there (CLAUDE.md W)."""
    return any(os.path.isfile(p) for p in gateway_config_paths(env))


# --- the language model (step AM4, tools/get_llm.py) ---------------------------------------------------------------
# the model the AM4 comparison chose (docs/assistant_design.md 5b; = tools/get_llm.py CHOSEN); another one by
# $TRYGONS_ASSISTANT_LLM = a model folder name under llm_root() or a local .gguf path
LLM_DEFAULT = "llama3.2-3b"


def llm_root():
    return os.environ.get("TRYGONS_LLM_DIR") or os.path.join(bin_dir(), "assistant", "llm")


def llm_py_dir():
    """The folder with the llama.cpp Python bindings (pip --target, tools/get_llm.py), or None."""
    d = os.path.join(llm_root(), "py")
    return os.path.abspath(d) if os.path.isdir(os.path.join(d, "llama_cpp")) else None


def llm_model(name=None):
    """The local .gguf file of a language model (a folder name under llm_root(), or a .gguf path), or None."""
    name = name or os.environ.get("TRYGONS_ASSISTANT_LLM") or LLM_DEFAULT
    if name.endswith(".gguf"):
        return os.path.abspath(name) if os.path.isfile(name) else None
    d = os.path.join(llm_root(), name)
    try:
        g = sorted(f for f in os.listdir(d) if f.endswith(".gguf"))
    except OSError:
        return None
    return os.path.abspath(os.path.join(d, g[0])) if len(g) == 1 else None


def llm_installed():
    return llm_py_dir() is not None and llm_model() is not None


# the pinned model files (AM4-r LOW-2): file name -> (size, sha256) = tools/get_llm.py MODELS (test AS23 keeps the
# two tables equal). The worker computes the sha256 of the file it is about to load (~2 GB = a few seconds, once per
# start, in the below-normal worker) and refuses a mismatch; a file that is not in this table is refused too unless
# the owner set assistant_llm_unpinned (a model of his own choice - W108).
LLM_PINNED = {
    "Qwen2.5-7B-Instruct-Q4_K_M.gguf": (4683074240, "65b8fcd92af6b4fefa935c625d1ac27ea29dcb6ee14589c55a8f115ceaaa1423"),
    "Meltemi-7B-Instruct-v1.5.Q4_K_M.gguf": (4535764352, "15d0ad45f6c8911eadfae8f0c3a8a7f59b8053332bc99f76cf146f88f975ec14"),
    "llama-krikri-8b-instruct-q4_k_m.gguf": (5041147264, "e3f93427fe0fcc016e31ed4bfbc1d10916fb97f6b2b3298445196490d315376f"),
    "Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf": (4920739232, "7b064f5842bf9532c91456deda288a1b672397a54fa729aa665952863033557c"),
    "qwen2.5-3b-instruct-q4_k_m.gguf": (2104932768, "626b4a6678b86442240e33df819e00132d3ba7dddfe1cdc4fbb18e0a9615c62d"),
    "Llama-3.2-3B-Instruct-Q4_K_M.gguf": (2019377696, "6c1a2b41161032677be168d354123594c0e6e67d2b9227c84f296ad037c728ff"),
}


def sha256_of(path, chunk=1 << 20):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(chunk), b""):
            h.update(b)
    return h.hexdigest()


_VERIFIED = {}          # (abs path, size, mtime_ns) -> sha256: a guard reload re-hashes nothing (AM4-r2 LOW-d)


def llm_verify(path, allow_unpinned=False, cache=_VERIFIED):
    """(ok, why) for a model file about to be loaded: a pinned file = size AND sha256 as in LLM_PINNED; an unpinned
    file = refused unless allow_unpinned (then "unpinned" is said in why). Reads the whole file (seconds) ONCE per
    process: a file already hashed with the same path, size and mtime is not read again (the GPU guard's CPU / GPU
    reloads; cache = None hashes always)."""
    name = os.path.basename(str(path or ""))
    pinned = LLM_PINNED.get(name)
    if pinned is None:
        return (True, "unpinned model file (assistant_llm_unpinned set): %s" % name) if allow_unpinned else \
            (False, "model file not pinned in paths.LLM_PINNED: %s (set assistant_llm_unpinned to allow it)" % name)
    try:
        st = os.stat(path)
        size = st.st_size
    except OSError as e:
        return False, "model file unreadable: %s" % str(e)[:80]
    if size != pinned[0]:
        return False, "model file size %d != pinned %d: %s" % (size, pinned[0], name)
    key = (os.path.abspath(path), size, st.st_mtime_ns)
    got = cache.get(key) if cache is not None else None
    cached = got is not None
    if got is None:
        got = sha256_of(path)
    if got != pinned[1]:
        return False, "model file sha256 %s... != pinned %s...: %s" % (got[:12], pinned[1][:12], name)
    if cache is not None:
        cache[key] = got
    return True, "sha256 verified" + (" (cached)" if cached else "")
