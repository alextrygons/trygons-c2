# Licences of the models and the bundled / downloaded tools (step AM7, owner W135, 2026-11-11)

What every model and every third-party program / library of this project is licensed under, where it comes from,
which exact version / revision is pinned, and whether COMMERCIAL use is allowed. Collected 2026-11-11 from the
licence texts / model cards at the pinned revisions (Hugging Face API `cardData.license` + the LICENSE / MODEL_CARD
files, PyPI metadata of the pinned versions, the vendored LICENSE files, `ffmpeg -L`). This is an engineering
inventory, NOT legal advice: before a product is sold, a lawyer checks the "check" rows and the distribution duties.

Legend: **commercial use** = yes / yes with duties / NO / check. "Duties" = what the licence asks when the program is
DISTRIBUTED (installed on a customer's PC / boat): keep the licence + copyright notice (MIT / BSD / Apache: also the
NOTICE file), the source code offer (GPL / LGPL), attribution lines (CC BY, Llama). Nothing here is in git except the
vendored libcanard (tools/bin/ and the model folders are .gitignored and downloaded by the tools/get_*.py scripts).

## 1. Language models of the talking manual (tools/get_llm.py, Q4_K_M GGUF, sha256 + revision pinned)

"Clean" = MIT or Apache-2.0: commercial use allowed, no use policy / user cap / naming rule beyond the licence
text. Only clean models are the default and downloaded by default (`get_llm.py` / `--all`); the RESTRICTED ones need
`--allow-restricted` (a printed warning) and are kept ONLY for the comparison runs (docs/assistant_design.md 5b / 5n).

| name (get_llm.py) | model (base) | GGUF repo @ revision | licence | commercial use | status |
|---|---|---|---|---|---|
| **qwen3-4b (the DEFAULT since AM7)** | Qwen/Qwen3-4B-Instruct-2507 | bartowski/Qwen_Qwen3-4B-Instruct-2507-GGUF @ ae44f08e | Apache-2.0 | yes | clean - CHOSEN (docs/assistant_design.md 5n) |
| phi4-mini | microsoft/Phi-4-mini-instruct (3.8B) | bartowski/microsoft_Phi-4-mini-instruct-GGUF @ 7ff82c2a | MIT | yes | clean |
| phi3.5-mini | microsoft/Phi-3.5-mini-instruct (3.8B) | bartowski/Phi-3.5-mini-instruct-GGUF @ 6d70da17 | MIT | yes | clean |
| qwen2.5-1.5b | Qwen/Qwen2.5-1.5B-Instruct | Qwen/Qwen2.5-1.5B-Instruct-GGUF @ 91cad511 | Apache-2.0 | yes | clean |
| ministral3-3b | mistralai/Ministral-3-3B-Instruct-2512 (text part only) | mistralai/Ministral-3-3B-Instruct-2512-GGUF @ eb599d40 | Apache-2.0 | yes | clean |
| eurollm-1.7b | utter-project/EuroLLM-1.7B-Instruct | mradermacher/EuroLLM-1.7B-Instruct-GGUF @ 2951f08f | Apache-2.0 | yes | clean |
| qwen2.5-7b | Qwen/Qwen2.5-7B-Instruct | bartowski/Qwen2.5-7B-Instruct-GGUF @ 8911e8a4 | Apache-2.0 | yes | clean (does not fit beside Whisper on 6 GB) |
| meltemi-7b | ilsp/Meltemi-7B-Instruct-v1.5 (Mistral-7B based, Greek) | mradermacher/Meltemi-7B-Instruct-v1.5-GGUF @ cd3ac278 | Apache-2.0 | yes | clean (partly on the CPU on 6 GB) |
| llama3.2-3b | meta-llama/Llama-3.2-3B-Instruct | bartowski/Llama-3.2-3B-Instruct-GGUF @ 5ab33fa9 | Llama 3.2 Community | yes with duties: < 700 M monthly users, Meta's Acceptable Use Policy, "Built with Llama" shown, the licence copy passed on | RESTRICTED (the AM4-AM6 default until AM7) |
| llama3.1-8b | meta-llama/Llama-3.1-8B-Instruct | bartowski/Meta-Llama-3.1-8B-Instruct-GGUF @ bf5b95e9 | Llama 3.1 Community | as above | RESTRICTED |
| krikri-8b | ilsp/Llama-Krikri-8B-Instruct (Llama-3.1-8B derived, Greek) | ilsp/Llama-Krikri-8B-Instruct-GGUF @ 3c1ce101 | Llama 3.1 Community (cardData "llama3.1") | as above + a derivative's name must begin with "Llama" | RESTRICTED |
| qwen2.5-3b | Qwen/Qwen2.5-3B-Instruct | Qwen/Qwen2.5-3B-Instruct-GGUF @ 7dabda4d | Qwen RESEARCH licence (cardData "other") | NO (non-commercial / research; commercial = a separate licence from Alibaba) | RESTRICTED |

A GGUF conversion / quantisation (bartowski, mradermacher) carries the BASE model's licence; the repo revisions are
the 40-hex revisions in tools/get_llm.py MODELS (shortened here), the file sha256 = paths.LLM_PINNED.

## 2. Speech recognition (tools/get_whisper.py)

| component | version / revision | source | licence | commercial use |
|---|---|---|---|---|
| Whisper large-v3-turbo (CTranslate2 form) | dropbox-dash/faster-whisper-large-v3-turbo @ 0a363e91 | https://huggingface.co/dropbox-dash/faster-whisper-large-v3-turbo (from openai/whisper-large-v3-turbo) | MIT | yes |
| Whisper small (CTranslate2 form) | Systran/faster-whisper-small @ 536b0662 | https://huggingface.co/Systran/faster-whisper-small (from openai/whisper-small) | MIT | yes |
| faster-whisper | 1.2.1 | https://github.com/SYSTRAN/faster-whisper | MIT | yes |
| CTranslate2 | 4.6.2 | https://github.com/OpenNMT/CTranslate2 | MIT | yes |
| the assistant Python (embeddable CPython) | 3.12.10 | https://www.python.org/ftp/python/3.12.10/ | PSF-2.0 | yes |
| get-pip.py | pinned pypa/get-pip commit (get_whisper.py) | https://github.com/pypa/get-pip | MIT | yes |

## 3. The LLM runtime (tools/get_llm.py)

| component | version | source | licence | commercial use |
|---|---|---|---|---|
| llama-cpp-python (CUDA 12.4 wheel, contains llama.cpp + ggml) | 0.3.36 (release v0.3.36-cu124) | https://github.com/abetlen/llama-cpp-python | MIT (llama.cpp / ggml: MIT) | yes |
| NVIDIA CUDA runtime / cuBLAS / cuDNN wheels | nvidia-cuda-runtime-cu12 12.8.90, nvidia-cublas-cu12 12.8.4.1, nvidia-cudnn-cu12 9.8.0.87 | PyPI (NVIDIA) | NVIDIA proprietary (CUDA / cuDNN EULA) | yes, redistribution only as the EULA allows (the wheels are downloaded on each PC, not shipped by us) |

## 4. Python libraries pinned in the requirements files

tools/assistant_requirements.txt (+ _cuda.txt, _llm.txt; every one pinned + hashed), tools/requirements_api.txt,
tools/get_vision_python.py PACKAGES, groundstation/requirements.txt (minimum versions, not pinned):

| package | version | licence | commercial use |
|---|---|---|---|
| anyio | 4.16.0 | MIT | yes |
| av (PyAV; its wheel bundles FFmpeg libraries built LGPL) | 19.0.2 | BSD-3-Clause (bundled FFmpeg: LGPL-2.1+) | yes (LGPL: keep the libraries replaceable + the notice) |
| click | 8.5.1 | BSD-3-Clause | yes |
| colorama | 0.4.6 | BSD | yes |
| ctranslate2 | 4.6.2 | MIT | yes |
| faster-whisper | 1.2.1 | MIT | yes |
| filelock | 4.2.9 | MIT | yes |
| flatbuffers | 25.12.19 | Apache-2.0 | yes |
| fsspec | 2026.9.0 | BSD-3-Clause | yes |
| h11 | 0.16.0 | MIT | yes |
| hf-xet | 1.8.0 | Apache-2.0 | yes |
| httpcore2 / httpx2 | 2.16.0 | BSD-3-Clause | yes |
| huggingface_hub | 2.4.0 | Apache-2.0 | yes |
| idna | 3.20 | BSD-3-Clause | yes |
| numpy | 2.1.3 | BSD-3-Clause | yes |
| onnxruntime | 1.31.1 (assistant) / 1.20.1 (vision Python) | MIT | yes |
| packaging | 26.3 | Apache-2.0 OR BSD-2-Clause | yes |
| protobuf | 7.36.2 | BSD-3-Clause | yes |
| PyYAML | 6.0.3 | MIT | yes |
| setuptools | 84.2.1 | MIT | yes |
| tokenizers | 0.23.3 | Apache-2.0 | yes |
| tqdm | 4.70.1 | MPL-2.0 AND MIT | yes (MPL: changes to tqdm's own files stay MPL) |
| truststore | 0.10.5 | MIT | yes |
| typing_extensions | 4.16.0 | PSF-2.0 | yes |
| diskcache | 5.6.3 | Apache-2.0 | yes |
| jinja2 | 3.1.6 | BSD-3-Clause | yes |
| MarkupSafe | 3.0.2 | BSD-3-Clause | yes |
| anthropic (tools/api_review.py only - a development tool, never on a station / boat) | 2.3.0 | Apache-2.0 | yes |
| pyserial (station + gateway) | >= 3.5 | BSD-3-Clause | yes |
| Pillow (station) | >= 10.1 (MSYS2 build) | MIT-CMU (HPND) | yes |
| pymavlink (OPTIONAL cross-check only: UA2 / the SITL suite when installed; never imported by the product) | not pinned | LGPL-3.0 | yes (LGPL duties if shipped) |

## 5. Voice output (tools/get_piper.py)

| component | version | source | licence | commercial use |
|---|---|---|---|---|
| Piper (piper.exe, piper_phonemize) | release 2023.11.14-2 | https://github.com/rhasspy/piper | MIT | yes |
| espeak-ng (espeak-ng.dll + espeak-ng-data, INSIDE the Piper release; the phonemiser) | as in that release | https://github.com/espeak-ng/espeak-ng | GPL-3.0-or-later | yes with duties: the GPL source offer + licence when the Piper folder is distributed |
| onnxruntime.dll (inside the Piper release) | as in that release | https://github.com/microsoft/onnxruntime | MIT | yes |
| voice el_GR-rapunzelina-low (Greek) | piper-voices v1.0.0 | https://huggingface.co/rhasspy/piper-voices | repo: MIT; its MODEL_CARD: dataset CC0, but "Finetuned from U.S. English Ryan voice" - the Ryan voice's dataset (RyanSpeech) is **CC BY-NC-SA 4.0** | **check / NO for now** (weights derived from a non-commercial dataset) |
| voice en_GB-alan-medium (English) | piper-voices v1.0.0 | same | repo: MIT; MODEL_CARD: dataset = Mycroft mimic3 apope ("All Rights Reserved"), "Finetuned from U.S. English lessac voice" - the lessac (Blizzard 2013) data licence is **non-commercial** | **check / NO for now** |
| the alternative voice engine "windows" (gs_settings voice_engine; WinRT OneCore / SAPI voices) | the PC's Windows | Microsoft | part of the Windows licence | yes (used through the OS) |
| the owner's RECORDED Greek tour clips (academy/tour_voice/el) | committed | the owner | the owner's own recordings | yes |

FINDING (W141): BOTH Piper voices are fine-tunes of voices trained on NON-COMMERCIAL data, so their commercial use is
not clear even though the piper-voices repository is tagged MIT. The safe commercial path today is the "windows" voice
engine (already a station setting) or a voice trained only on CC0 / CC BY data. Not changed in AM7 (a station
default, the owner decides).

## 6. Vision, video, ADS-B, UAV, CAN, other programs

| component | version / revision | source | licence | commercial use |
|---|---|---|---|---|
| eWaSR ewasr_resnet18 ONNX (obstacle detection; tools/get_vision_model.py) | release 0.1.0 | https://github.com/tersekmatija/eWaSR | Apache-2.0 (code + weights; LICENSE copied next to the model) | yes (check: the ResNet-18 backbone was ImageNet-pretrained; ImageNet's own terms are research-oriented - the status of weights trained from it is not settled) |
| MaSTr1325 sample images (tools/get_vision_samples.py; tests only) | - | https://box.vicos.si/borja/viamaro/ | CC BY 4.0 | yes with attribution (written next to the files) |
| the vision Python (embeddable CPython + onnxruntime 1.20.1 + numpy 2.1.3) | 3.12.10 | python.org / PyPI | PSF-2.0 / MIT / BSD-3 | yes |
| MediaMTX (video server; tools/get_mediamtx.py) | v1.21.2 | https://github.com/bluenviron/mediamtx | MIT | yes |
| ffmpeg (NOT downloaded by us: MSYS2 `pacman -S mingw-w64-ucrt-x86_64-ffmpeg` / the Ubuntu package; gateway/toolpaths.py) | 9.0.2 (MSYS2 build on the owner's PC) | https://ffmpeg.org | GPL-3.0-or-later (this build: `--enable-gpl`, x264 etc.; `ffmpeg -L`) | yes with duties (GPL source offer when shipped); H.264 / HEVC encoding may need patent licences (MPEG LA / Access Advance) in a commercial product - check |
| readsb (ADS-B decoder; tools/get_readsb.py, NOT pinned yet - W132) | not chosen | https://github.com/wiedehopf/readsb | GPL-3.0-or-later | yes with duties (GPL; it runs as its own process) |
| dump1090 (the alternative decoder) | not chosen | https://github.com/flightaware/dump1090 | GPL-2.0-or-later | yes with duties |
| libcanard (DroneCAN; VENDORED in hal/libcanard, compiled into the AURIX firmware) | commit 601ed354 (2025-07-14) | https://github.com/dronecan/libcanard | MIT (hal/libcanard/LICENSE) | yes (keep the LICENSE with the sources / in the product documentation) |
| ArduPilot SITL (ArduCopter; tools/get_sitl.py; tests only, never on a station / boat) | Windows V4.7.3-beta1 Cygwin build (sha256-pinned), Linux 4.7.2 | https://firmware.ardupilot.org | GPL-3.0 (Cygwin DLLs: LGPL-3.0 / GPL with exceptions) | yes (a test tool; GPL duties only if redistributed) |
| ArduPilot on the drones (owner-installed, W60 / W140) | owner | https://ardupilot.org | GPL-3.0 | yes with duties (the firmware is the drone maker's / owner's) |
| Mission Planner (workshop tool, never bundled) | owner | https://github.com/ArduPilot/MissionPlanner | GPL-3.0 | yes |
| Infineon iLLD (Libraries/, generated by ADS) | ADS project | Infineon | Infineon's iLLD licence (use on Infineon devices) | yes on Infineon MCUs (check the ADS licence terms) |

## 7. Chart / map data (tools/chartprep.py; charts/ is not in git)

| data | source | licence | commercial use |
|---|---|---|---|
| EMODnet bathymetry DTM (ERDDAP) | https://emodnet.ec.europa.eu | CC BY 4.0 | yes with attribution (shown on the map) |
| GEBCO grid (gap fill) | https://www.gebco.net | GEBCO terms: public domain, attribution requested, NOT for navigation | yes with attribution |
| OpenSeaMap / OpenStreetMap seamarks (Overpass) | https://www.openseamap.org | ODbL 1.0 | yes with attribution; a distributed DERIVED DATABASE stays ODbL |
| the Dhaskalio Garmin screen photos (docs/daskalio/) | the owner | the owner's photos of a Garmin screen (the chart data shown is Garmin's / its supplier's) | academy / demo pictures only (never charts) - check before publishing |

## 8. Summary

- Clean (MIT / Apache / BSD / PSF): the chosen language model (section 1), Whisper + faster-whisper + CTranslate2,
  llama.cpp / llama-cpp-python, MediaMTX, eWaSR (+ the ImageNet note), libcanard, every pinned Python library.
- Duties when shipped (GPL / LGPL): ffmpeg, espeak-ng (inside Piper), readsb / dump1090, ArduPilot SITL, PyAV's
  FFmpeg, pymavlink (optional). They all run as SEPARATE programs or are optional test tools (no GPL code is linked
  into the station / gateway Python code or the AURIX firmware).
- NOT clean / check: the two Piper voices (W141), the Llama-family models and Qwen2.5-3B (kept only for the
  comparison, never the default: tools/get_llm.py refuses them without --allow-restricted), H.264 patents, the
  ImageNet-pretrained backbone of eWaSR, the iLLD terms.
