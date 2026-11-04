"""Offline speech for the ground station's voice callouts (section D4). No internet.

Engines (step 13o, owner 2026-10-31 F: the engine is a SOUND SETTING, "voice_engine"):
  1. Piper (https://github.com/rhasspy/piper, MIT): free offline neural voices,
     a GREEK one (el_GR-rapunzelina-low) and an ENGLISH one (en_GB-alan-medium).
     Install: python tools/get_piper.py  (-> tools/bin/piper/, not in git), or set
     PIPER_BIN (piper.exe) + PIPER_VOICES (folder with the .onnx voices).
     The default. Step 13o: calmer settings (PIPER_TUNE: lower noise_scale / noise_w - the
     owner heard a "sick" voice in the first seconds of every sentence with the model's
     defaults) and a short lead-in silence (LEAD_IN_MS) with a fade-in in front of every file.
  2. The Windows voices: the modern ones (Windows.Media.SpeechSynthesis = the "OneCore"
     voices, e.g. the Greek "Microsoft Stefanos" of an installed Greek speech pack) and
     the classic SAPI 5 ones (System.Speech). Chosen with voice_engine = "windows"; used
     as well when Piper is missing.
  3. None: the callouts are tones only (the screen still shows everything).

synth(lang, [(text, wav_path), ...]) makes every file in ONE engine run (the
sound library pre-generates and caches all fixed phrases, so a callout starts
at once). voice_id(lang) names the voice + its settings: the sound library
keeps it next to its cache and remakes every sentence when it changes.
"""
import array
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
VOICE_FILES = {"el": ("el_GR-rapunzelina-low", "el_GR"), "en": ("en_GB-alan-medium", "en_GB", "en_US")}
NO_WINDOW = 0x08000000 if os.name == "nt" else 0
ENGINES = ("piper", "windows")
VOICE_VERSION = 2                       # 2 = step 13o (tuned Piper + lead-in); a change remakes every sentence
PIPER_DEFAULT = {"noise_scale": 0.667, "noise_w": 0.8, "length_scale": 1.0}    # the model files' own values
PIPER_TUNE = {"noise_scale": 0.333, "noise_w": 0.333, "length_scale": 1.05}     # 13o: steadier, a little slower
LEAD_IN_MS = 250                        # silence in front of every sentence (13o)
FADE_MS = 30                            # fade-in of the first speech samples (no click / burst at the start)

_prefer = "piper"


def set_prefer(name):
    """The engine chosen in the sound settings ("piper" / "windows"); unknown = piper."""
    global _prefer
    _prefer = name if name in ENGINES else "piper"


def prefer():
    return _prefer


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


# --- the Windows voices ---------------------------------------------------------------------------------
# WinRT (Windows.Media.SpeechSynthesis) sees the modern "OneCore" voices - the Greek Stefanos of a Greek
# speech pack is one of them and is NOT visible to the classic System.Speech; System.Speech is kept for the
# classic SAPI 5 voices. One PowerShell script lists / speaks both.
_PS_HEAD = r"""
$ErrorActionPreference = 'Stop'
$rt = $null
try {
  Add-Type -AssemblyName System.Runtime.WindowsRuntime
  $null = [Windows.Media.SpeechSynthesis.SpeechSynthesizer, Windows.Media.SpeechSynthesis, ContentType = WindowsRuntime]
  $null = [Windows.Storage.Streams.DataReader, Windows.Storage.Streams, ContentType = WindowsRuntime]
  $asTask = ([System.WindowsRuntimeSystemExtensions].GetMethods() | ? { $_.Name -eq 'AsTask' -and
    $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
  $rt = $true
} catch { $rt = $null }
function Await($op, [Type]$t) { $k = $asTask.MakeGenericMethod($t).Invoke($null, @($op)); $null = $k.Wait(20000); $k.Result }
"""
_PS_LIST = _PS_HEAD + r"""
if ($rt) { try { [Windows.Media.SpeechSynthesis.SpeechSynthesizer]::AllVoices | % { 'rt|' + $_.Language + '|' + $_.DisplayName } } catch {} }
try { Add-Type -AssemblyName System.Speech
  (New-Object System.Speech.Synthesis.SpeechSynthesizer).GetInstalledVoices() | % { 'sapi|' + $_.VoiceInfo.Culture.Name + '|' + $_.VoiceInfo.Name } } catch {}
"""
_PS_SPEAK_RT = _PS_HEAD + r"""
$s = New-Object Windows.Media.SpeechSynthesis.SpeechSynthesizer
$s.Voice = [Windows.Media.SpeechSynthesis.SpeechSynthesizer]::AllVoices | ? { $_.DisplayName -eq '%s' } | select -First 1
foreach ($l in (Get-Content -Encoding UTF8 '%s')) { $p = $l.Split([char]9)
  try { $st = Await ($s.SynthesizeTextToStreamAsync($p[1])) ([Windows.Media.SpeechSynthesis.SpeechSynthesisStream])
    $r = New-Object Windows.Storage.Streams.DataReader($st.GetInputStreamAt(0)); $n = [uint32]$st.Size
    $null = Await ($r.LoadAsync($n)) ([uint32]); $b = New-Object byte[] $n; $r.ReadBytes($b)
    [IO.File]::WriteAllBytes($p[0], $b) } catch {} }
"""
_PS_SPEAK_SAPI = (
    "Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
    "$s.SelectVoice('%s'); $fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo 22050, "
    "([System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen), "
    "([System.Speech.AudioFormat.AudioChannel]::Mono); "
    "foreach ($l in (Get-Content -Encoding UTF8 '%s')) { $p = $l.Split([char]9); "
    "$s.SetOutputToWaveFile($p[0], $fmt); $s.Speak($p[1]) }; $s.SetOutputToNull()")

_win = None                 # None = not looked up yet (or the lookup failed: tried again later)
_win_retry_t = 0.0          # (review 13o L3) a failed lookup is tried again after this time, never in a loop
_win_bg = None              # the background lookup thread (review 13o L1: never PowerShell on the Tk thread)
WIN_RETRY_S = 60.0


def _ps(script, timeout):
    fd, path = tempfile.mkstemp(suffix=".ps1")
    with os.fdopen(fd, "w", encoding="utf-8-sig") as f:
        f.write(script)
    try:
        return subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", path],
                              capture_output=True, timeout=timeout, creationflags=NO_WINDOW)
    finally:
        os.remove(path)


def windows_voices():
    """{language 'el' / 'en': (kind 'rt' / 'sapi', voice name)} of the installed Windows voices; a modern
    (OneCore) voice wins over a classic SAPI one of the same language."""
    global _win, _win_retry_t
    if _win is not None:
        return _win
    if os.name != "nt":
        _win = {}
        return _win
    if time.time() < _win_retry_t:
        return {}                               # a lookup failed a moment ago: not known (yet), no PowerShell now
    # review 13o L2: fill a LOCAL dict and publish it only when complete - windows_known() is never true for a
    # half-filled (or empty because still running) answer
    found_v = {}
    try:
        out = _ps(_PS_LIST, 30)
        if out.returncode != 0:
            raise OSError("PowerShell voice list: exit %s" % out.returncode)
        found = [line.strip().split("|", 2) for line in out.stdout.decode("utf-8", "replace").splitlines()
                 if line.count("|") >= 2]
        for kind in ("rt", "sapi"):
            for k, cult, name in found:
                if k == kind:
                    found_v.setdefault(cult[:2].lower(), (kind, name))
    except (OSError, subprocess.SubprocessError):
        _win_retry_t = time.time() + WIN_RETRY_S   # review 13o L3: unknown, NOT "no voices" - retried later
        return {}
    _win = found_v
    return _win


def lookup_background():
    """Look the Windows voices up in a helper thread (once at a time): the screens never wait for PowerShell."""
    global _win_bg
    if _win is not None or os.name != "nt" or (_win_bg is not None and _win_bg.is_alive()):
        return
    _win_bg = threading.Thread(target=windows_voices, daemon=True)
    _win_bg.start()


def windows_known():
    """True once the Windows voices were looked up (windows_voices() then answers at once)."""
    return _win is not None


def sapi_voices():
    """{language: voice name} (kept for older callers)."""
    return {lg: v[1] for lg, v in windows_voices().items()}


def available(lang, wait=False):
    """The engines that can speak this language here. wait=False (the screens, review 13o L1): the Windows voices
    only when they are already known - otherwise they are looked up in the background and Windows does not count
    yet (never a PowerShell run on the Tk thread)."""
    out = []
    if piper_bin() and piper_voice(lang):
        out.append("piper")
    if windows_known() or wait:
        if lang in windows_voices():
            out.append("windows")
    else:
        lookup_background()
    return out


def engine(lang, choice=None):
    """Which engine speaks this language: the chosen one when it can, else the other one, else None."""
    have = available(lang)
    want = choice or _prefer
    if want in have:
        return want
    return have[0] if have else None


def voice_id(lang, choice=None):
    """The voice + its settings in one string (None = no voice): a change = every sentence is made again."""
    e = engine(lang, choice)
    if e == "piper":
        return "v%d|piper|%s|%s|lead%d" % (VOICE_VERSION, os.path.basename(piper_voice(lang)),
                                           json.dumps(PIPER_TUNE, sort_keys=True), LEAD_IN_MS)
    if e == "windows":
        return "v%d|windows|%s|%s|lead%d" % ((VOICE_VERSION,) + windows_voices()[lang] + (LEAD_IN_MS,))
    return None


def describe(lang, choice=None):
    e = engine(lang, choice)
    if e == "piper":
        return "Piper - %s" % os.path.basename(piper_voice(lang))
    if e == "windows":
        return "Windows - %s" % windows_voices()[lang][1]
    return None


def lead_in(path, ms=LEAD_IN_MS, fade_ms=FADE_MS):
    """Silence in front of the speech + a short fade-in (16-bit PCM files; anything else is left alone)."""
    try:
        with wave.open(path, "rb") as w:
            ch, sw, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
            data = w.readframes(n)
    except (OSError, EOFError, wave.Error):
        return False
    if sw != 2 or n == 0:
        return False
    a = array.array("h", data)
    if sys.byteorder == "big":
        a.byteswap()
    k = min(len(a), int(rate * fade_ms / 1000.0) * ch)
    for i in range(k):
        a[i] = int(a[i] * (i / float(k)))
    pad = array.array("h", [0]) * (int(rate * ms / 1000.0) * ch)
    out = pad + a
    if sys.byteorder == "big":
        out.byteswap()
    tmp = path + ".lead.wav"
    with wave.open(tmp, "wb") as w:
        w.setnchannels(ch)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(out.tobytes())
    os.replace(tmp, path)
    return True


def synth(lang, items, timeout=300, choice=None, piper_params=None, lead_ms=LEAD_IN_MS):
    """[(text, wav_path)] -> the number of files made. choice / piper_params / lead_ms only for the voice
    samples (tools/voice_samples.py); the callouts use the sound setting + PIPER_TUNE + LEAD_IN_MS."""
    items = [(t, p) for t, p in items if t]
    if not items:
        return 0
    e = engine(lang, choice)
    for _t, p in items:
        os.makedirs(os.path.dirname(p), exist_ok=True)
    if e == "piper":
        prm = PIPER_TUNE if piper_params is None else piper_params
        lines = "".join(json.dumps({"text": t, "output_file": p}, ensure_ascii=False) + "\n" for t, p in items)
        args = [piper_bin(), "--model", piper_voice(lang), "--json-input", "--quiet"]
        for k in ("noise_scale", "noise_w", "length_scale"):
            if k in prm:
                args += ["--" + k, "%.3f" % prm[k]]
        try:
            subprocess.run(args, input=lines.encode("utf-8"), capture_output=True, timeout=timeout,
                           cwd=os.path.dirname(piper_bin()), creationflags=NO_WINDOW)
        except (OSError, subprocess.SubprocessError):
            return 0
    elif e == "windows":
        kind, name = windows_voices()[lang]
        fd, lst = tempfile.mkstemp(suffix=".txt")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            for t, p in items:
                f.write("%s\t%s\n" % (p, t.replace("\t", " ").replace("\n", " ")))
        tpl = _PS_SPEAK_RT if kind == "rt" else _PS_SPEAK_SAPI
        try:
            _ps(tpl % (name.replace("'", "''"), lst.replace("'", "''")), timeout)   # (review 13o L4: a ' in the path)
        except (OSError, subprocess.SubprocessError):
            return 0
        finally:
            os.remove(lst)
    else:
        return 0
    made = 0
    for _t, p in items:
        if os.path.exists(p) and os.path.getsize(p) > 100:
            made += 1
            if lead_ms:
                lead_in(p, lead_ms)
    return made


if __name__ == "__main__":
    for lg in ("el", "en"):
        print(lg, describe(lg) or "NO VOICE (python tools/get_piper.py)", "| engines here:", available(lg))
    sys.exit(0)
