"""Recorded Greek narration for the guided tour (owner 2026-11-06): WAV originals -> Opus/OGG + manifest.

    python tools/make_tour_voice.py [--src voice_raw/TRYGONS_C2_149_Numbered_WAV]

Clip NNN_<three words>.wav = tour stop number NNN (the order of academy/tour.py stops("el")). For every clip
the first three words of the filename are compared with the start of that stop's spoken sentence
(tour.sentence): a clip whose words differ is NOT placed (reported, never guessed). The audio is converted
only (libopus, 64 kbit/s, mono) - no trim, no normalisation. Writes
    academy/tour_voice/el/NNN_<stop id>.ogg + manifest.json   (read by academy/tour.py recording())
    docs/voice/MAPPING.md
The WAV originals stay local (voice_raw/ is in .gitignore).
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import unicodedata

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "academy"))
import tour                                                              # noqa: E402

BITRATE = "64k"


def words(text, n=3):
    t = re.sub(r"[^\w\s]", " ", text.replace("_", " "), flags=re.UNICODE)
    return t.split()[:n]


def norm(w):
    d = unicodedata.normalize("NFD", w.lower())
    return "".join(c for c in d if unicodedata.category(c) != "Mn").replace("ς", "σ")


def duration(path):
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path],
                       capture_output=True, text=True)
    return float(r.stdout.strip() or 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.join(ROOT, "voice_raw", "TRYGONS_C2_149_Numbered_WAV"))
    a = ap.parse_args()
    stops = tour.stops("el")
    out_dir = os.path.join(ROOT, "academy", "tour_voice", "el")
    os.makedirs(out_dir, exist_ok=True)
    clips, no_step, mismatch = {}, [], []
    for f in sorted(os.listdir(a.src)):
        m = re.match(r"^(\d{3})_(.*)\.wav$", f)
        if not m:
            continue
        n = int(m.group(1))
        if n < 1 or n > len(stops):
            no_step.append(f)
            continue
        st = stops[n - 1]
        want = [norm(w) for w in words(tour.sentence(st))]
        got = [norm(w) for w in words(m.group(2))]
        src = os.path.join(a.src, f)
        row = {"n": n, "src": f, "id": st["id"], "got": " ".join(words(m.group(2))),
               "want": " ".join(words(tour.sentence(st))), "dur": duration(src), "ok": got == want}
        if not row["ok"]:
            mismatch.append(row)
            continue
        name = "%03d_%s.ogg" % (n, st["id"])
        dst = os.path.join(out_dir, name)
        subprocess.run(["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", src, "-ac", "1",
                        "-c:a", "libopus", "-b:a", BITRATE, "-application", "audio", dst], check=True)
        row["file"] = name
        row["size"] = os.path.getsize(dst)
        # the sentence the clip was recorded for: a later text change makes the clip stale (tour.recording)
        row["sha"] = hashlib.sha1(tour.sentence(st).encode("utf-8")).hexdigest()
        clips[n] = row
    man = {"format": 1, "lang": "el", "codec": "opus/ogg %s mono" % BITRATE,
           "clips": [{"n": r["n"], "id": r["id"], "file": r["file"], "sha": r["sha"],
                      "dur_s": round(r["dur"], 2)} for r in clips.values()]}
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(man, fh, ensure_ascii=False, indent=1)
    # MAPPING.md
    L = ["# Tour narration (Greek): clip <-> tour step", "",
         "Source: `voice_raw/` (local, not in git). Conversion: `tools/make_tour_voice.py`, Opus/OGG 64 kbit/s mono, no "
         "trim / normalise / other processing (the 44.1 kHz audio is resampled to Opus' 48 kHz inside the encoder). "
         "Files: `academy/tour_voice/el/` + `manifest.json`. Clip number = the stop's number in `academy/tour.py` "
         "`stops('el')`. Clips 085-126 are tour narration of the alarms (stops 85-86 lamps, 87-126 alarm.*), "
         "not the live alarm callouts.", "",
         "Played by the tour in Greek; a stop without a (matching) clip uses the synthetic Greek voice; English "
         "always uses the synthetic English voice.", "",
         "| # | file | duration s | tour step id | first 3 words match |", "|---|------|-----------:|--------------|----|"]
    allrows = sorted(list(clips.values()) + mismatch, key=lambda r: r["n"])
    for r in allrows:
        L.append("| %03d | %s | %.2f | %s | %s |" % (r["n"], r.get("file", r["src"] + " (NOT placed)"), r["dur"],
                                                      r["id"], "yes" if r["ok"] else "**NO**"))
    L += ["", "## Tour steps without a clip", ""]
    miss = [s["id"] for i, s in enumerate(stops, 1) if i not in clips]
    L += ["- %s" % x for x in miss] or ["none"]
    L += ["", "## Clips without a step", ""]
    L += ["- %s" % x for x in no_step] or ["none"]
    L += ["", "## Mismatches (first three words differ; clip NOT placed)", ""]
    L += ["- %03d %s: file says `%s`, tour text starts `%s`" % (r["n"], r["id"], r["got"], r["want"])
          for r in mismatch] or ["none"]
    os.makedirs(os.path.join(ROOT, "docs", "voice"), exist_ok=True)
    with open(os.path.join(ROOT, "docs", "voice", "MAPPING.md"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(L) + "\n")
    print("%d placed, %d mismatches, %d without step, %d steps without clip" %
          (len(clips), len(mismatch), len(no_step), len(miss)))
    for r in mismatch:
        print("MISMATCH", r["n"], r["id"], "|", r["got"], "|", r["want"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
