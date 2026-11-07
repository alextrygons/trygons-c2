"""The talking manual's GPU memory guard (step AM4): where the language model runs.

    free_mb()                    -> (free_mb, total_mb) of GPU 0 from NVML (the driver's nvml.dll / libnvidia-ml),
                                    None = no NVIDIA GPU / no driver (= the CPU)
    gguf_meta(path)              -> the GGUF header numbers the plan needs (layers, KV heads, sizes) - pure Python
    plan(meta, free, reserve_mb) -> {"mode": "gpu" | "gpu+cpu" | "cpu", "layers": n_gpu_layers, "need_mb", "why"}

The rule (CLAUDE.md W, docs/assistant_design.md 5): the model goes on the GPU only while it leaves MARGIN_MB free
for the station's own users of the GPU (the cameras' video decode, the HUD, the academy's 3D sim); what does not
fit runs on the CPU (some layers = "gpu+cpu", none = "cpu"). Before every answer the worker checks again: less than
LOW_MB free = the other programs are short = the model is reloaded with a new plan (usually the CPU).
No process is started (no nvidia-smi here: the worker starts nothing) - NVML through ctypes only.
Tests: $TRYGONS_ASSISTANT_FAKE_GPU = "free_mb,total_mb" or "none".
"""
import ctypes
import os
import struct

MARGIN_MB = 800          # always left free for the station's video / HUD / 3D sim (docs 5, MEASURE -> 5b)
LOW_MB = 300             # less than this free before an answer = the model makes room (reloaded, a new plan)
COMPUTE_MB = 450         # llama.cpp's compute buffers on the GPU (measured with the candidates, 5b)
OUTPUT_FRACTION = 0.08   # the token embedding / output layer share of the file (kept on the GPU first)


class _Mem(ctypes.Structure):
    _fields_ = [("total", ctypes.c_ulonglong), ("free", ctypes.c_ulonglong), ("used", ctypes.c_ulonglong)]


def free_mb():
    """(free MB, total MB) of GPU 0, or None (no GPU / no driver / NVML failed)."""
    fake = os.environ.get("TRYGONS_ASSISTANT_FAKE_GPU")
    if fake:
        if fake == "none":
            return None
        f, t = (int(x) for x in fake.split(","))
        return f, t
    try:
        lib = ctypes.CDLL("nvml.dll" if os.name == "nt" else "libnvidia-ml.so.1")
    except OSError:
        return None
    try:
        if lib.nvmlInit_v2() != 0:
            return None
        h = ctypes.c_void_p()
        if lib.nvmlDeviceGetHandleByIndex_v2(0, ctypes.byref(h)) != 0:
            return None
        m = _Mem()
        if lib.nvmlDeviceGetMemoryInfo(h, ctypes.byref(m)) != 0:
            return None
        return int(m.free // (1 << 20)), int(m.total // (1 << 20))
    except (OSError, AttributeError):
        return None
    finally:
        try:
            lib.nvmlShutdown()
        except (OSError, AttributeError):
            pass


# --- GGUF header (only the numbers the plan needs; the tensors are never read) ------------------------------------
_SCALAR = {0: "<B", 1: "<b", 2: "<H", 3: "<h", 4: "<I", 5: "<i", 6: "<f", 7: "<?", 10: "<Q", 11: "<q", 12: "<d"}
WANT = ("general.architecture", "general.name", "block_count", "attention.head_count", "attention.head_count_kv",
        "embedding_length", "context_length")


def gguf_meta(path):
    """{"arch", "name", "layers", "heads", "kv_heads", "embd", "ctx_train", "file_mb"} from a GGUF file's header."""
    out = {"file_mb": os.path.getsize(path) / (1 << 20)}
    with open(path, "rb") as f:
        if f.read(4) != b"GGUF":
            raise ValueError("not a GGUF file: %s" % path)
        ver, = struct.unpack("<I", f.read(4))
        if ver < 2:
            raise ValueError("GGUF version %d too old" % ver)
        _n_tensors, n_kv = struct.unpack("<QQ", f.read(16))

        def rstr():
            n, = struct.unpack("<Q", f.read(8))
            return f.read(n).decode("utf-8", "replace")

        def rval(t):
            if t in _SCALAR:
                fmt = _SCALAR[t]
                return struct.unpack(fmt, f.read(struct.calcsize(fmt)))[0]
            if t == 8:
                return rstr()
            if t == 9:
                it, n = struct.unpack("<IQ", f.read(12))
                if it in _SCALAR:                              # a big numeric array: skipped, never kept
                    f.seek(n * struct.calcsize(_SCALAR[it]), 1)
                    return None
                for _ in range(n):
                    rval(it)
                return None
            raise ValueError("GGUF value type %d" % t)

        raw = {}
        for _ in range(n_kv):
            k = rstr()
            t, = struct.unpack("<I", f.read(4))
            v = rval(t)
            if any(k == w or k.endswith("." + w) for w in WANT):
                raw[k] = v
    arch = raw.get("general.architecture", "")
    g = (lambda w, d=0: raw.get("%s.%s" % (arch, w), d))
    heads = g("attention.head_count", 32) or 32
    out.update(arch=arch, name=raw.get("general.name", ""), layers=int(g("block_count", 32)), heads=int(heads),
               kv_heads=int(g("attention.head_count_kv", heads) or heads), embd=int(g("embedding_length", 4096)),
               ctx_train=int(g("context_length", 4096)))
    return out


def kv_mb(meta, n_ctx):
    """The KV cache in MB (f16 keys + values) for n_ctx tokens."""
    head_dim = meta["embd"] / float(meta["heads"])
    return meta["layers"] * 2 * meta["kv_heads"] * head_dim * 2 * n_ctx / float(1 << 20)


def plan(meta, free, n_ctx, reserve_mb=0, margin_mb=MARGIN_MB, device="auto"):
    """Where the model runs. free = free_mb() (None = no GPU); reserve_mb = memory another part of the worker
    still needs on the GPU (none today: Whisper is loaded first and already counted in `free`)."""
    need = meta["file_mb"] + kv_mb(meta, n_ctx) + COMPUTE_MB
    if device == "cpu" or free is None:
        return {"mode": "cpu", "layers": 0, "need_mb": int(need),
                "why": "the CPU (setting)" if device == "cpu" else "no NVIDIA GPU"}
    avail = free[0] - margin_mb - reserve_mb
    if avail >= need:
        return {"mode": "gpu", "layers": -1, "need_mb": int(need), "why": "fits: %d MB free" % free[0]}
    per_layer = meta["file_mb"] * (1.0 - OUTPUT_FRACTION) / meta["layers"] + kv_mb(meta, n_ctx) / meta["layers"]
    n = int((avail - COMPUTE_MB - meta["file_mb"] * OUTPUT_FRACTION) // per_layer) if per_layer > 0 else 0
    if n >= 1:
        n = min(n, meta["layers"] - 1)
        return {"mode": "gpu+cpu", "layers": n, "need_mb": int(need),
                "why": "%d of %d layers fit (%d MB free)" % (n, meta["layers"], free[0])}
    return {"mode": "cpu", "layers": 0, "need_mb": int(need), "why": "too little GPU memory: %d MB free" % free[0]}
