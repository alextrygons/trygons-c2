"""Make the ground station's voice files now (step 13o, owner 2026-10-31 F).

  python tools/make_voice.py            every callout sentence + the guided tour, Greek and English, with the
                                        voice chosen in the sound settings (gs_settings.json "voice_engine");
                                        sentences of an older voice / older settings are deleted and made again
  python tools/make_voice.py --samples  only the voice samples: 5 sentences per option into
                                        groundstation/sound_cache/samples/<lang>/<option>/ (+ README.txt)
  --lang el                             one language only
  --engine piper|windows                use this engine (and save it as the sound setting)

The station does the same by itself in the background (at its start: the callouts + the tour of the voice
language; Tools -> Sound: "Voice" / "Voice samples"). Display only: nothing goes to a boat.
"""
import argparse
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GS = os.path.join(REPO, "groundstation")
sys.path.insert(0, GS)

import i18n  # noqa: E402
import settings  # noqa: E402
import sound  # noqa: E402
import tts  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--samples", action="store_true", help="only the voice samples")
    ap.add_argument("--lang", choices=i18n.LANGS)
    ap.add_argument("--engine", choices=tts.ENGINES)
    a = ap.parse_args(argv)
    if a.engine:
        settings.put("voice_engine", a.engine)
    tts.set_prefer(settings.get("voice_engine", "piper"))
    langs = [a.lang] if a.lang else list(i18n.LANGS)
    lib = sound.Library()
    for lg in langs:
        if a.samples:
            folder = os.path.join(lib.cache, "samples", lg)
            print(lg, "samples ->", folder, sound.make_samples(folder, lg), flush=True)
            continue
        if not tts.engine(lg):
            print(lg, "NO VOICE (python tools/get_piper.py)")
            continue
        t0 = time.time()
        texts = lib.all_sentences(lg)
        tour = sound.tour_texts(lg)
        lib.make(lg, texts + tour)
        ok = sum(1 for t in texts + tour if os.path.exists(lib.sentence_path(lg, t)))
        print("%s: %s - %d callout + %d tour sentences, %d ready, %d old files deleted, %.0f s" % (
            lg, tts.describe(lg), len(texts), len(tour), ok, lib.remade.get(lg, 0), time.time() - t0), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
