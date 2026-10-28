"""Audit log on the gateway (section B7): who connected, who held HELM,
every command with its result, channel changes, SAVE presses, sync runs.

One JSON line per entry in <ssd>/audit/YYYYMMDD.jsonl (UTC, GPS time).
If the data drive is missing it goes to the gateway PC's system disk (fallback) so nothing is
lost. The last entries are kept in memory for the stations' event list.
"""
import collections
import json
import os

from common import log


class Audit:
    def __init__(self, storage, clock, fallback_dir=None, keep=500):
        self.storage = storage
        self.clock = clock
        self.fallback = fallback_dir
        self.recent = collections.deque(maxlen=keep)
        self.count = 0

    def _folder(self):
        if self.storage.usable:
            try:
                return self.storage.folder("audit")
            except OSError:
                pass
        if self.fallback:
            os.makedirs(self.fallback, exist_ok=True)
            return self.fallback
        return None

    def __call__(self, kind, **fields):
        e = {"utc": self.clock.stamp(), "kind": kind}
        e.update(fields)
        self.recent.append(e)
        self.count += 1
        folder = self._folder()
        if not folder:
            return e
        try:
            day = self.clock.stamp()[:10].replace("-", "")
            with open(os.path.join(folder, "%s.jsonl" % day), "a", encoding="utf-8") as f:
                f.write(json.dumps(e, separators=(",", ":")) + "\n")
        except OSError as ex:
            log.warning("audit log: %s", ex)
        return e
