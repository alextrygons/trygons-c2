"""The local log of the trend highlights (step TR3, CLAUDE.md Y, docs/trends_design.md 7): THIS PC only, one JSON line
per highlight on / off in groundstation/trends/log/YYYY-MM-DD.jsonl (gitignored; a TRYGONS_TEST run = a temp folder;
TRYGONS_TRENDS_LOG overrides). 5 MB per file (then _2, _3 ...), 30 days, at most 5 of the oldest files pruned per
write. TEXT ONLY, never sent anywhere. A disk error is swallowed (the log never stops the screens). PURE: os / json /
time only."""
import json
import os
import re
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_FILE_BYTES = 5 * 1024 * 1024
KEEP_DAYS = 30
MAX_PRUNE_PER_RUN = 5
_NAME_RE = re.compile(r"^(\d{4}-\d{2}-\d{2})(?:_(\d+))?\.jsonl$")


def log_dir():
    d = os.environ.get("TRYGONS_TRENDS_LOG")
    if not d:
        d = os.path.join(tempfile.gettempdir(), "trygons_test_trends") if os.environ.get("TRYGONS_TEST") \
            else os.path.join(HERE, "trends", "log")
    return d


def log_files(folder):
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
    now = time.time() if now is None else now
    gone = []
    for date, _p, fn in log_files(folder):
        if len(gone) >= max_per_run:
            break
        try:
            age = (now - time.mktime(time.strptime(date, "%Y-%m-%d"))) / 86400.0
        except (ValueError, OverflowError):
            continue
        if age > keep_days:
            try:
                os.remove(os.path.join(folder, fn))
                gone.append(fn)
            except OSError:
                pass
    return gone


def entry(ev, boat="", now=None):
    """The log line of a detector event: the design's fields, the UTC time and the boat's name."""
    now = time.time() if now is None else now
    e = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(now)) + ".%dZ" % int((now % 1) * 10), "boat": boat}
    for k in ("ev", "val", "kind", "dir", "base_m", "base_s", "rec_m", "rec_s", "stat", "thr", "mode", "dur_s", "why"):
        v = ev.get(k)
        e[k] = round(v, 3) if isinstance(v, float) else v
    return e


def append(e, folder=None, now=None):
    """Write one line (the file of today, a numbered part when it is full). -> the file name or None."""
    try:
        folder = folder or log_dir()
        os.makedirs(folder, exist_ok=True)
        now = time.time() if now is None else now
        day = time.strftime("%Y-%m-%d", time.localtime(now))
        part = 1
        while True:
            fn = day + ("" if part == 1 else "_%d" % part) + ".jsonl"
            p = os.path.join(folder, fn)
            if not os.path.exists(p) or os.path.getsize(p) < MAX_FILE_BYTES:
                break
            part += 1
        prune(folder, now)
        with open(p, "a", encoding="utf-8") as f:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
        return fn
    except (OSError, ValueError, TypeError):
        return None
