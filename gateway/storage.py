"""The gateway's data drive (logs and video): the gateway PC's own SSD (default),
or an external SSD (require_mount: then it must be its own mounted drive).

Checked every check_s seconds:
  - the folder exists and (require_mount) is on a mounted drive, not the system disk
  - a small test file can be written, flushed to disk and read back
  - free space: below low_free_gb -> "low" (section F deletes old SYNCED data),
    below full_free_gb -> "full" (STORAGE FULL: only the ring buffer runs)
A problem gives the alarm GATEWAY STORAGE on the boat (GW_STATUS flag); the
gateway itself keeps running (control, telemetry, video relay, ...).
"""
import os
import shutil
import time

from common import log

OK, MISSING, FAILING, LOW, FULL = "ok", "MISSING", "FAILING", "low", "FULL"


class Storage:
    def __init__(self, cfg):
        self.root = cfg["root"]
        self.require_mount = cfg.get("require_mount", True)
        self.check_s = cfg.get("check_s", 30)
        self.low_b = cfg.get("low_free_gb", 20.0) * 1e9
        self.full_b = cfg.get("full_free_gb", 2.0) * 1e9
        q = cfg.get("quota_gb")                 # optional: Trygons may use at most this much of the drive
        self.quota_b = q * 1e9 if q else None
        self.state = MISSING
        self.free_b = 0
        self.total_b = 0
        self.reason = "not checked yet"
        self.last_check = 0.0
        self.fail_inject = False            # tests: pretend the drive fails

    # usable for writing? (low is still usable; full only for the ring buffer)
    @property
    def usable(self):
        return self.state in (OK, LOW, FULL)

    @property
    def alarm(self):
        """GATEWAY STORAGE alarm: drive missing or failing."""
        return self.state in (MISSING, FAILING)

    def folder(self, *parts):
        """A folder on the SSD (created if needed)."""
        p = os.path.join(self.root, *parts)
        os.makedirs(p, exist_ok=True)
        return p

    def _mounted(self):
        if not self.require_mount:
            return True
        p = os.path.abspath(self.root)
        while not os.path.ismount(p):
            parent = os.path.dirname(p)
            if parent == p:
                return False
            p = parent
        return p != "/"                     # the SD card is "/" - not allowed for data

    def check(self, force=False):
        now = time.time()
        if not force and now - self.last_check < self.check_s:
            return self.state
        self.last_check = now
        old = self.state
        try:
            if not os.path.isdir(self.root) or not self._mounted():
                self.state, self.reason = MISSING, "SSD not mounted at %s" % self.root
            elif self.fail_inject:
                raise OSError("injected drive failure")
            else:
                test = os.path.join(self.root, ".write_test")
                data = ("%.3f" % now).encode()
                with open(test, "wb") as f:
                    f.write(data)
                    f.flush()
                    os.fsync(f.fileno())
                with open(test, "rb") as f:
                    if f.read() != data:
                        raise OSError("read back differs")
                free = self.free_now()
                if free < self.full_b:
                    self.state, self.reason = FULL, "only %.1f GB free" % (free / 1e9)
                elif free < self.low_b:
                    self.state, self.reason = LOW, "%.1f GB free" % (free / 1e9)
                else:
                    self.state, self.reason = OK, "%.0f of %.0f GB free" % (free / 1e9, self.total_b / 1e9)
        except OSError as e:
            self.state, self.reason = FAILING, "SSD write test failed: %s" % e
        if self.state != old:
            (log.warning if self.state in (MISSING, FAILING, FULL) else log.info)(
                "storage %s -> %s (%s)", old, self.state, self.reason)
        return self.state

    def used_now(self):
        """Bytes in the data folder (a hard link counts twice: the safe side)."""
        used = 0
        for dirpath, _dirs, files in os.walk(self.root):
            for n in files:
                try:
                    used += os.stat(os.path.join(dirpath, n)).st_size
                except OSError:
                    pass
        return used

    def free_now(self):
        """Free bytes now (the drive, or what is left of quota_gb)."""
        du = shutil.disk_usage(self.root)
        self.free_b, self.total_b = du.free, du.total
        if self.quota_b:
            used = self.used_now()
            self.free_b = max(0, min(du.free, int(self.quota_b - used)))
            self.total_b = int(self.quota_b)
        return self.free_b

    def status(self):
        return {"state": self.state, "reason": self.reason, "free_gb": round(self.free_b / 1e9, 1),
                "total_gb": round(self.total_b / 1e9, 1), "root": self.root}
