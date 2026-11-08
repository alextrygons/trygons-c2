"""The talking manual's local log (step AM5, owner I1.6): THIS PC only - client.log_dir() (groundstation/assistant/log/,
gitignored; a TRYGONS_TEST run = a temp folder), one JSON line per answer in YYYY-MM-DD.jsonl: the question text,
the chosen passage (id + chapter), the answer, the times, how it was spoken (and why it failed: AM5-r LOW-3
"detail"). TEXT ONLY: never audio (no samples, no WAV), never sent anywhere (no gateway sync, no academy push -
nothing reads it but the owner). A disk error is swallowed (the log never stops an answer).
AM5-r LOW-2 size cap + retention: a day's file above MAX_FILE_BYTES (5 MB) continues in YYYY-MM-DD_2.jsonl, _3 ...;
files older than KEEP_DAYS (30 days, by the date in their name) are deleted at the next write, oldest first (only
the log's own *.jsonl names are touched). AM5-r2 LOW-5 limits: there is NO limit on the number of parts per day
(a day of non-stop questions makes as many 5 MB parts as it needs - the retention bounds the whole); prune TRUSTS
the PC clock (the date in a file's name is compared with "now"): a clock stepped far ahead (> 31 days, e.g. a PC
without its battery whose time was set wrong) makes every file look old, so a single run deletes at most
MAX_PRUNE_PER_RUN of the oldest files - a wrong clock loses a few files per answer, never the whole log at once (a
limit, not a protection: with the clock still wrong the next writes go on)."""
import json
import os
import re
import time

from . import client

KEEP = ("q", "lang", "a", "src", "cite", "id", "mode", "ms", "gen_ms", "answer_ms", "problems", "picked")
MAX_FILE_BYTES = 5 * 1024 * 1024    # a day's file above this continues in a numbered part
KEEP_DAYS = 30                      # older files are deleted at the next write
MAX_PRUNE_PER_RUN = 5               # AM5-r2 LOW-5: at most this many (the oldest) files go per prune run / write
_NAME_RE = re.compile(r"^(\d{4}-\d{2}-\d{2})(?:_(\d+))?\.jsonl$")


def entry(info, voice=None, detail=None):
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
    if detail:
        e["detail"] = str(detail)[:200]
    return e


def log_files(folder):
    """[(date, part, name)] of the log's own files in the folder, oldest first."""
    out = []
    try:
        names = os.listdir(folder)
    except OSError:
        return out
    for fn in names:
        m = _NAME_RE.match(fn)
        if m:
            out.append((m.group(1), int(m.group(2) or 1), fn))
    out.sort()
    return out


def prune(folder, now=None, keep_days=KEEP_DAYS, max_per_run=MAX_PRUNE_PER_RUN):
    """Delete the log files older than keep_days (by the date in the name), oldest first, at most max_per_run of them
    (AM5-r2 LOW-5: a PC clock jumped ahead never empties the log in one run). -> the names deleted."""
    now = time.time() if now is None else now
    gone = []
    for date, _part, fn in log_files(folder):
        if len(gone) >= max_per_run:
            break
        try:
            age_days = (now - time.mktime(time.strptime(date, "%Y-%m-%d"))) / 86400.0
        except (ValueError, OverflowError):
            continue
        if age_days > keep_days:
            try:
                os.remove(os.path.join(folder, fn))
                gone.append(fn)
            except OSError:
                pass
    return gone


def current_path(folder, day, max_bytes=MAX_FILE_BYTES):
    """Today's file to append to: the highest part of the day; a new part when it is at / above the cap."""
    parts = [(part, fn) for date, part, fn in log_files(folder) if date == day]
    if not parts:
        return os.path.join(folder, day + ".jsonl")
    part, fn = max(parts)
    p = os.path.join(folder, fn)
    try:
        if os.path.getsize(p) >= max_bytes:
            return os.path.join(folder, "%s_%d.jsonl" % (day, part + 1))
    except OSError:
        pass
    return p


def write(info, voice=None, folder=None, detail=None, max_bytes=MAX_FILE_BYTES, keep_days=KEEP_DAYS):
    """Append one line; returns the file path ("" on a disk error)."""
    try:
        d = folder or client.log_dir()
        prune(d, keep_days=keep_days)
        p = current_path(d, time.strftime("%Y-%m-%d"), max_bytes)
        with open(p, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry(info, voice, detail), ensure_ascii=False) + "\n")
        return p
    except (OSError, TypeError, ValueError):
        return ""
