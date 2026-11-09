"""This station's own copy of the search and rescue markers (CLAUDE.md X, docs/sar_design.md 1; step PF3).

DISPLAY + BOOKKEEPING ONLY. The gateway keeps the shared copy (gateway/sar.py); this PC keeps what it made itself so a
MOB marker exists even with NO gateway / a dead link (the alarm never waits for the gateway). Markers made while the
gateway is away are sent when it is back - the same id is idempotent there.

  LocalSar(path)         RAM + the file <folder>/local.jsonl (one JSON line per change; TRYGONS_TEST = a temp folder)
    add(m) / get(id) / close(id, ...) / move(id, ...) / merge(remote_list) / view() / unsent() / mark_sent(id)
"""
import json
import os
import tempfile
import time


def default_folder():
    if os.environ.get("TRYGONS_TEST"):
        return os.path.join(tempfile.gettempdir(), "trygons_sar_test_%d" % os.getpid())
    return os.environ.get("TRYGONS_SAR") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "sar")


class LocalSar:
    def __init__(self, folder=None):
        self.folder = folder or default_folder()
        self.path = os.path.join(self.folder, "local.jsonl")
        self.markers = {}               # id -> marker (this station's view: local + the gateway's)
        self.sent = set()               # ids the gateway has acknowledged
        self.err = None
        self._load()

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                for line in f:
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        continue                      # a cut last line
                    if rec.get("op") == "put" and isinstance(rec.get("m"), dict) and rec["m"].get("id"):
                        self.markers[rec["m"]["id"]] = rec["m"]
                        if rec.get("sent"):
                            self.sent.add(rec["m"]["id"])
        except OSError:
            pass

    def _log(self, m, sent=False):
        try:
            os.makedirs(self.folder, exist_ok=True)
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(json.dumps({"op": "put", "m": m, "sent": sent}) + "\n")
            self.err = None
        except OSError as e:
            self.err = str(e)                         # a disk error = RAM only; the alarm still works

    def add(self, m):
        self.markers[m["id"]] = m
        self._log(m)
        return m

    def get(self, mid):
        return self.markers.get(mid)

    def change(self, mid, **kw):
        m = self.markers.get(mid)
        if m is None:
            return None
        m.update(kw)
        self._log(m, mid in self.sent)
        return m

    def mark_sent(self, mid):
        self.sent.add(mid)

    def unsent(self):
        return [m for i, m in self.markers.items() if i not in self.sent]

    def merge(self, remote):
        """The gateway's list wins for every id it holds (it is the shared copy); local-only ones stay. -> new ids."""
        new = []
        for m in remote or []:
            if not isinstance(m, dict) or not m.get("id"):
                continue
            if m["id"] not in self.markers:
                new.append(m["id"])
            self.markers[m["id"]] = dict(m)
            self.sent.add(m["id"])
        return new

    def view(self):
        return sorted(self.markers.values(), key=lambda m: (m.get("t_utc") or "", m["id"]))

    def open_mobs(self):
        return [m for m in self.view() if m.get("kind") == "mob" and not m.get("closed")]


def utc_stamp(t=None):
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() if t is None else t))
