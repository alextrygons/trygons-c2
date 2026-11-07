"""Speech -> text for the talking manual (step AM2): faster-whisper (CTranslate2) - chosen over whisper.cpp in
the AM2 measurement (docs/assistant_design.md section 3).

OFFLINE ONLY (AM1 review M2): a model is given as a LOCAL FOLDER PATH (paths.whisper_model); a model NAME
("small", "large-v3", "org/repo") is refused - faster-whisper would download it. HF_HUB_OFFLINE /
TRANSFORMERS_OFFLINE are set before the library is imported, so nothing can reach the internet even by mistake.

    s = load("faster", model_dir, device="auto", hints="...")      -> Stt
    s.transcribe(samples_int16_16k, lang="el") -> {"text", "lang", "ms", "device"}
    load("stub", text="πού κλείνω την καταπακτή")                   tests: no model, a fixed answer
"""
import os
import time

RATE = 16000
MIN_S = 0.4                       # shorter = nothing said (client.py ignores it before; here a second guard)


class Refused(Exception):
    pass


def offline_env():
    for k in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE", "HF_DATASETS_OFFLINE"):
        os.environ[k] = "1"
    os.environ["HF_HUB_DISABLE_TELEMETRY"] = "1"


def check_model_dir(model):
    """A model must be an existing local folder with a CTranslate2 model.bin (never a name to download)."""
    if not isinstance(model, str) or not model:
        raise Refused("no model folder")
    if not os.path.isabs(model):
        raise Refused("model must be a local folder path, not a name: %r" % model)
    if not os.path.isfile(os.path.join(model, "model.bin")):
        raise Refused("no model.bin in %s" % model)
    return model


def _cuda_dll_dirs():
    """Windows: the nvidia-cublas / cudnn wheels keep their DLLs in site-packages/nvidia/*/bin."""
    if os.name != "nt":
        return
    import site
    for sp in site.getsitepackages():
        base = os.path.join(sp, "nvidia")
        if not os.path.isdir(base):
            continue
        for sub in os.listdir(base):
            b = os.path.join(base, sub, "bin")
            if os.path.isdir(b):
                try:
                    os.add_dll_directory(b)
                except OSError:
                    pass
                os.environ["PATH"] = b + os.pathsep + os.environ.get("PATH", "")


class StubStt:
    """Tests: no model; returns the configured text for any capture >= MIN_S."""
    device = "stub"

    def __init__(self, text="", lang="el"):
        self.text, self.lang = text, lang

    def transcribe(self, samples, lang=None, hints=""):
        t0 = time.monotonic()
        txt = self.text if len(samples) >= MIN_S * RATE else ""
        return {"text": txt, "lang": lang or self.lang, "ms": int((time.monotonic() - t0) * 1000),
                "device": self.device}


class FasterStt:
    def __init__(self, model_dir, device="auto", threads=4, cpu_model_dir=None):
        """model_dir = the GPU model; cpu_model_dir = the smaller one used when it runs on the CPU."""
        check_model_dir(model_dir)
        if cpu_model_dir:
            check_model_dir(cpu_model_dir)
        offline_env()
        _cuda_dll_dirs()
        from faster_whisper import WhisperModel
        self.device = device
        if device == "auto":
            try:
                import ctranslate2
                device = "cuda" if ctranslate2.get_cuda_device_count() > 0 else "cpu"
            except Exception:
                device = "cpu"
        last = None
        tries = [(device, "int8_float16" if device == "cuda" else "int8")] + ([("cpu", "int8")] if device != "cpu"
                                                                             else [])
        for dev, ct in tries:
            md = cpu_model_dir if dev == "cpu" and cpu_model_dir else model_dir
            try:
                self.model = WhisperModel(md, device=dev, compute_type=ct, cpu_threads=threads,
                                          local_files_only=True)
                self.device = dev
                self.model_name = os.path.basename(md)
                break
            except Exception as e:             # no CUDA libraries / no GPU memory: the CPU
                last = e
                self.model = None
        if self.model is None:
            raise Refused("model did not load: %s" % str(last)[:120])
        try:                                   # warm up: the first real question is not the slow one
            import numpy as np
            list(self.model.transcribe(np.zeros(RATE, dtype=np.float32), language="el", beam_size=1,
                                       without_timestamps=True)[0])
        except Exception:
            pass

    def transcribe(self, samples, lang=None, hints=""):
        import numpy as np
        t0 = time.monotonic()
        a = np.frombuffer(bytes(samples), dtype=np.int16).astype(np.float32) / 32768.0
        if len(a) < MIN_S * RATE:
            return {"text": "", "lang": lang, "ms": 0, "device": self.device}
        # the station language is a hint only; Whisper's own detection decides (a Greek speaker may say an
        # English button name) - detection among el / en only
        # beam 5 everywhere: free on the GPU, on the CPU beam 1 saved only ~10 % and lost words (AM2 numbers)
        kw = dict(initial_prompt=hints or None, beam_size=5, vad_filter=True, condition_on_previous_text=False,
                  without_timestamps=True)
        segs, info = self.model.transcribe(a, language=None, **kw)
        det = info.language
        if det not in ("el", "en"):
            # only Greek or English: a Greek question heard as Polish (seen in AM2) is decoded again as the more
            # likely of the two
            probs = dict(info.all_language_probs or [])
            det = max(("el", "en"), key=lambda k: (probs.get(k, 0.0), k == (lang or "el")))
            segs, info = self.model.transcribe(a, language=det, **kw)
        text = " ".join(s.text.strip() for s in segs).strip()
        return {"text": text, "lang": det, "ms": int((time.monotonic() - t0) * 1000),
                "device": self.device + "/" + self.model_name}


def load(kind, model_dir=None, device="auto", text="", lang="el", cpu_model_dir=None):
    if kind == "stub":
        return StubStt(text, lang)
    if kind == "faster":
        return FasterStt(model_dir, device, cpu_model_dir=cpu_model_dir)
    raise Refused("unknown speech recognition %r" % kind)
