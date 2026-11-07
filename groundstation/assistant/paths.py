"""Where the talking manual's files are (step AM2) - path helpers only, no logic, no network.

    assistant Python   $TRYGONS_ASSISTANT_PYTHON, <repo>/tools/bin/assistant-python/ (tools/get_whisper.py,
                       Windows), else this Python (Ubuntu: pip install faster-whisper)
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
