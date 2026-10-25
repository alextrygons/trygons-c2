"""Offline speech for the ground station's voice callouts (section D4). No internet.

Engines, in this order:
  1. Piper (https://github.com/rhasspy/piper, MIT): free offline neural voices,
     a GREEK one (el_GR-rapunzelina-low) and an ENGLISH one (en_GB-alan-medium).
     Install: python tools/get_piper.py  (-> tools/bin/piper/, not in git), or set
     PIPER_BIN (piper.exe) + PIPER_VOICES (folder with the .onnx voices).
  2. The Windows voices (System.Speech / SAPI 5) when one for the language is
     installed (Windows has English ones; a Greek one only with an installed
     Greek speech voice).
  3. None: the callouts are tones only (the screen still shows everything).

synth(lang, [(text, wav_path), ...]) makes every file in ONE engine run (the
sound library pre-generates and caches all fixed phrases, so a callout starts
at once).
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
VOICE_FILES = {"el": ("el_GR-rapunzelina-low", "el_GR"), "en": ("en_GB-alan-medium", "en_GB", "en_US")}
NO_WINDOW = 0x08000000 if os.name == "nt" else 0


def piper_bin():
    p = os.environ.get("PIPER_BIN")
    if p and os.path.exists(p):
        return p
    exe = "piper.exe" if os.name == "nt" else "piper"
    p = os.path.join(REPO, "tools", "bin", "piper", exe)
    if os.path.exists(p):
        return p
    return shutil.which("piper")


def piper_voice(lang):
    """The .onnx voice model for the language (the pinned one first, else any of that language)."""
    folders = [os.environ.get("PIPER_VOICES"), os.path.join(REPO, "tools", "bin", "piper", "voices")]
    names = VOICE_FILES.get(lang, ())
    for folder in folders:
        if not folder or not os.path.isdir(folder):
            continue
        files = sorted(f for f in os.listdir(folder) if f.endswith(".onnx"))
        for f in files:
            if f.startswith(names[0]) if names else False:
                return os.path.join(folder, f)
        for f in files:
            if any(f.startswith(n) for n in names[1:]):
                return os.path.join(folder, f)
    return None


_sapi = None


def sapi_voices():
    """{culture prefix 'el' / 'en': voice name} of the installed Windows (SAPI 5) voices."""
    global _sapi
    if _sapi is not None:
        return _sapi
    _sapi = {}
    if os.name != "nt":
        return _sapi
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command",
                              "Add-Type -AssemblyName System.Speech; "
                              "(New-Object System.Speech.Synthesis.SpeechSynthesizer).GetInstalledVoices() | "
                              "% { $_.VoiceInfo.Culture.Name + '|' + $_.VoiceInfo.Name }"],
                             capture_output=True, timeout=20, creationflags=NO_WINDOW)
        for line in out.stdout.decode("utf-8", "replace").splitlines():
            if "|" in line:
                cult, name = line.strip().split("|", 1)
                _sapi.setdefault(cult[:2].lower(), name)
    except (OSError, subprocess.SubprocessError):
        pass
    return _sapi


def engine(lang):
    """Which engine speaks this language: "piper", "windows" or None."""
    if piper_bin() and piper_voice(lang):
        return "piper"
    if lang in sapi_voices():
        return "windows"
    return None


def describe(lang):
    e = engine(lang)
    if e == "piper":
        return "Piper - %s" % os.path.basename(piper_voice(lang))
    if e == "windows":
        return "Windows - %s" % sapi_voices()[lang]
    return None


def synth(lang, items, timeout=300):
    """[(text, wav_path)] -> the number of files made."""
    items = [(t, p) for t, p in items if t]
    if not items:
        return 0
    e = engine(lang)
    for _t, p in items:
        os.makedirs(os.path.dirname(p), exist_ok=True)
    if e == "piper":
        lines = "".join(json.dumps({"text": t, "output_file": p}, ensure_ascii=False) + "\n" for t, p in items)
        try:
            subprocess.run([piper_bin(), "--model", piper_voice(lang), "--json-input", "--quiet"],
                           input=lines.encode("utf-8"), capture_output=True, timeout=timeout,
                           cwd=os.path.dirname(piper_bin()), creationflags=NO_WINDOW)
        except (OSError, subprocess.SubprocessError):
            return 0
    elif e == "windows":
        fd, lst = tempfile.mkstemp(suffix=".txt")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            for t, p in items:
                f.write("%s\t%s\n" % (p, t.replace("\t", " ").replace("\n", " ")))
        script = ("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                  "$s.SelectVoice('%s'); $fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo 22050, "
                  "([System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen), "
                  "([System.Speech.AudioFormat.AudioChannel]::Mono); "
                  "foreach ($l in (Get-Content -Encoding UTF8 '%s')) { $p = $l.Split([char]9); "
                  "$s.SetOutputToWaveFile($p[0], $fmt); $s.Speak($p[1]) }; $s.SetOutputToNull()"
                  % (sapi_voices()[lang].replace("'", "''"), lst))
        try:
            subprocess.run(["powershell", "-NoProfile", "-Command", script], capture_output=True, timeout=timeout,
                           creationflags=NO_WINDOW)
        except (OSError, subprocess.SubprocessError):
            return 0
        finally:
            os.remove(lst)
    else:
        return 0
    return sum(1 for _t, p in items if os.path.exists(p) and os.path.getsize(p) > 100)


if __name__ == "__main__":
    for lg in ("el", "en"):
        print(lg, describe(lg) or "NO VOICE (python tools/get_piper.py)")
    sys.exit(0)
