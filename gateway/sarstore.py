"""Search and rescue markers (CLAUDE.md X, docs/sar_design.md 1; step PF3): the gateway's ONE copy of every marker
of this boat (MOB, points, finds), state/sar_markers.json, every change in the audit log (kind "sar").

DISPLAY / BOOKKEEPING ONLY: a marker is a place on the map and a row in the log. Nothing here steers the boat,
sends to the AURIX or starts a UAV; routes and missions made FROM a marker go through the normal checks.

    SarStore(path).apply(op, m, user) -> (ok, id, text)   op add | move | rename | person | close | delete
    SarStore.list() / .rev / .n_open() / .status()
"""
import json
import math
import os
import re
import threading

KINDS = ("mob", "point", "find")
ID_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,48}$")
MAX_MARKERS = 200


def _num(v, lo, hi):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) and lo <= v <= hi else None


MAX_DELETED = 500


class SarStore:
    def __init__(self, path=None):
        self.path = path
        self.markers = {}
        self.deleted = []                           # PF4-r LOW: ids deleted here (newest last, kept MAX_DELETED)
        self.rev = 0
        self._lock = threading.RLock()
        self._file_lock = threading.Lock()          # one writer of the file at a time (the helper thread + flush())
        self._dirty = threading.Event()
        self._thread = None
        if path:
            self._load()

    # --- file ------------------------------------------------------------------------------------------------
    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.load(f)
            for m in doc.get("list") or []:
                if isinstance(m, dict) and ID_RE.match(str(m.get("id", ""))):
                    self.markers[m["id"]] = m
            self.rev = int(doc.get("rev") or 0)
            self.deleted = [str(i) for i in (doc.get("deleted") or []) if ID_RE.match(str(i))][-MAX_DELETED:]
        except (OSError, ValueError, TypeError, AttributeError):
            self.markers, self.rev = {}, 0          # a damaged file = no markers (the stations keep theirs, local)

    def _save(self):
        if not self.path:
            return
        self._dirty.set()
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._writer, name="sar-writer", daemon=True)
            self._thread.start()

    def _writer(self):
        while self._dirty.wait(5.0):
            self._dirty.clear()
            self.flush()

    def flush(self):
        with self._lock:
            doc = {"rev": self.rev, "list": self.list(), "deleted": list(self.deleted)}
        with self._file_lock:
            try:
                os.makedirs(os.path.dirname(self.path), exist_ok=True)
                tmp = self.path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(doc, f)
                os.replace(tmp, self.path)
            except OSError:
                pass

    # --- the markers -----------------------------------------------------------------------------------------
    def list(self):
        with self._lock:
            return sorted((dict(m) for m in self.markers.values()), key=lambda m: (m.get("t_utc") or "", m["id"]))

    def n_open(self):
        with self._lock:
            return sum(1 for m in self.markers.values() if not m.get("closed"))

    def status(self):
        return {"rev": self.rev, "n_open": self.n_open()}

    def set_still(self, mid, name):
        """PF4: the small still image of a marked point is stored (file name only; the bytes are in <data>/sar/)."""
        with self._lock:
            cur = self.markers.get(mid)
            if cur is None:
                return False
            cur["still"] = str(name)[:80]
            self.rev += 1
            self._save()
            return True

    def _next_n(self, kind):
        return 1 + max([int(m.get("n") or 0) for m in self.markers.values() if m.get("kind") == kind] or [0])

    def apply(self, op, m, user=""):
        """-> (ok, id, text). Idempotent: the same "add" twice changes nothing (a station sends its local marker when
        the gateway is back)."""
        with self._lock:
            ok, mid, text = self._apply(op, m if isinstance(m, dict) else {}, user)
            if ok and text != "same":
                self.rev += 1
                self._save()
            return ok, mid, text

    def _apply(self, op, m, user):
        mid = str(m.get("id") or "")
        if not ID_RE.match(mid):
            return False, None, "bad marker id"
        cur = self.markers.get(mid)
        if op == "add":
            if cur is not None:
                return True, mid, "same"
            kind = m.get("kind")
            lat, lon = _num(m.get("lat"), -90, 90), _num(m.get("lon"), -180, 180)
            if kind not in KINDS or lat is None or lon is None:
                return False, mid, "bad marker"
            if len(self.markers) >= MAX_MARKERS:
                return False, mid, "too many markers"
            n = m.get("n")
            if not (isinstance(n, int) and not isinstance(n, bool) and n > 0):
                n = self._next_n(kind)
            mk = {"id": mid, "kind": kind, "n": n, "name": str(m.get("name") or "")[:40], "lat": lat, "lon": lon,
                  "t_utc": str(m.get("t_utc") or "")[:32], "r_m": _num(m.get("r_m"), 0, 1e6) or 0.0,
                  "est": bool(m.get("est")), "person": bool(m.get("person", kind == "mob")),
                  "src": m.get("src") if isinstance(m.get("src"), dict) else None, "still": None, "closed": None,
                  "drift": m.get("drift") if isinstance(m.get("drift"), dict) else None, "by": str(user)[:32]}
            self.markers[mid] = mk
            return True, mid, "added"
        if cur is None:
            return False, mid, "no such marker"
        if op == "move":
            lat, lon = _num(m.get("lat"), -90, 90), _num(m.get("lon"), -180, 180)
            if lat is None or lon is None:
                return False, mid, "bad position"
            cur["lat"], cur["lon"] = lat, lon
            return True, mid, "moved"
        if op == "rename":
            cur["name"] = str(m.get("name") or "")[:40]
            return True, mid, "renamed"
        if op == "person":
            cur["person"] = bool(m.get("person"))
            return True, mid, "person"
        if op == "close":
            if cur.get("closed"):
                return True, mid, "same"
            why = m.get("why") if m.get("why") in ("found", "deleted") else "found"
            cur["closed"] = {"t_utc": str(m.get("t_utc") or "")[:32], "by": str(user)[:32], "why": why}
            return True, mid, "closed"
        if op == "delete":
            del self.markers[mid]
            # PF4-r LOW: a delete is said EXPLICITLY to the stations (sar_markers "deleted") - a station never takes
            # a point missing from the list for a deleted one (an empty / restarted gateway list deletes nothing)
            self.deleted = [i for i in self.deleted if i != mid][-(MAX_DELETED - 1):] + [mid]
            return True, mid, "deleted"
        return False, mid, "bad operation"
