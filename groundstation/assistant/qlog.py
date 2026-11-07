"""The talking manual's local log (step AM5, owner I1.6): THIS PC only - client.log_dir() (groundstation/assistant/log/,
gitignored; a TRYGONS_TEST run = a temp folder), one JSON line per answer in YYYY-MM-DD.jsonl: the question text,
the chosen passage (id + chapter), the answer, the times, how it was spoken. TEXT ONLY: never audio (no samples, no
WAV), never sent anywhere (no gateway sync, no academy push - nothing reads it but the owner). A disk error is
swallowed (the log never stops an answer)."""
import json
import os
import time

from . import client

KEEP = ("q", "lang", "a", "src", "cite", "id", "mode", "ms", "gen_ms", "answer_ms", "problems", "picked")


def entry(info, voice=None):
    """The log line of an answer (a dict): the texts and times only, never the audio."""
    e = {"t": time.strftime("%Y-%m-%dT%H:%M:%S")}
    for k in KEEP:
        if k in info and info[k] not in (None, "", []):
            e[k] = info[k]
    p = info.get("passage") or {}
    if p:
        e["passage_id"], e["passage_cite"] = p.get("id", ""), p.get("cite", "")
    hits = ((info.get("found") or {}).get("hits")) or []
    if hits:
        e["hits"] = [{"id": h.get("id"), "score": h.get("score")} for h in hits[:3]]
    if voice is not None:
        e["voice"] = voice
    return e


def write(info, voice=None, folder=None):
    """Append one line; returns the file path ("" on a disk error)."""
    try:
        d = folder or client.log_dir()
        p = os.path.join(d, time.strftime("%Y-%m-%d") + ".jsonl")
        with open(p, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry(info, voice), ensure_ascii=False) + "\n")
        return p
    except (OSError, TypeError, ValueError):
        return ""
