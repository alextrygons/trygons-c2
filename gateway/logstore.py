"""The Pi's full copy of the AURIX log (section A2).

The AURIX sends every record its logger handles (the same bytes it writes
to its SD card), numbered, in LOG_CHUNK frames. Here they are put back
together and written as .TLG files on the SSD:

  <ssd>/aurix_logs/YYYYMMDD/P<boot>_<file>.TLG   one per AURIX log file
  <ssd>/aurix_logs/YYYYMMDD/gaps.csv            records the Pi never got

A new file starts whenever the AURIX starts one (a HEADER record); like on
the SD card each file starts with "TLG1", the header, the boot event and the
config, so every file can be read on its own (groundstation/tlg.py,
logview.py). If the UART was too slow the AURIX dropped records for the Pi
(never control traffic); the numbers show the gap and it is noted in
gaps.csv. The AURIX SD card stays the complete, independent black box.
"""
import os
import re
import struct
import time

from common import log

SYNC, HEADER, CONFIG, DATA, EVENT = 0xA5, 1, 2, 0x10, 0x20
EV_BOOT = 1


class LogStore:
    def __init__(self, storage, clock, on_file_closed=None):
        self.storage = storage
        self.clock = clock
        self.on_file_closed = on_file_closed    # callback(path) - sync (section G)
        self.cur = None                         # (seq, length, bytearray)
        self.last_seq = None
        self.records = 0
        self.gaps = 0
        self.missing = 0
        self.broken = 0                         # chunks out of order / bad CRC
        self.lost_storage = 0                   # records not written: SSD problem
        self.boot = None
        self.file_no = None
        self.header_rec = None
        self.boot_rec = None
        self.config_rec = None
        self.fh = None
        self.path = None
        self.last_flush = 0.0
        self.bytes_written = 0

    # --- chunks -> records ---------------------------------------------------------
    def on_chunk(self, payload):
        seq, off, ln = struct.unpack_from("<IHH", payload)
        data = payload[8:]
        if off == 0:
            if self.cur is not None:
                self.broken += 1                # previous record never completed
            self.cur = (seq, ln, bytearray())
        if self.cur is None or self.cur[0] != seq or len(self.cur[2]) != off:
            self.broken += 1
            self.cur = None
            return
        self.cur[2].extend(data)
        if len(self.cur[2]) >= ln:
            rec = bytes(self.cur[2][:ln])
            self.cur = None
            self.on_record(seq, rec)

    # --- records -------------------------------------------------------------------
    def on_record(self, seq, rec):
        if len(rec) < 10 or rec[0] != SYNC or not self._crc_ok(rec):
            self.broken += 1
            return
        if self.last_seq is not None and seq != (self.last_seq + 1) & 0xFFFFFFFF:
            if seq > self.last_seq:
                self._gap(self.last_seq + 1, seq - 1)
            else:
                log.info("AURIX log numbers restarted (%d -> %d): AURIX reset", self.last_seq, seq)
        self.last_seq = seq
        self.records += 1
        rtype = rec[1]
        if rtype == HEADER:
            self.header_rec = rec
            text = rec[8:-2].decode("utf-8", "replace")
            m = re.search(r"boot=(\d+)", text)
            f = re.search(r"file=(\d+)", text)
            self.boot = int(m.group(1)) if m else None
            self.file_no = int(f.group(1)) if f else 0
            self._open_new()                    # the AURIX started a file: so do we
            return
        if rtype == CONFIG:
            self.config_rec = rec
        elif rtype == EVENT and len(rec) >= 12 and (rec[8] | rec[9] << 8) == EV_BOOT:
            self.boot_rec = rec
        self._write(rec)

    @staticmethod
    def _crc_ok(rec):
        from common import proto
        return proto.crc16(rec[1:-2]) == (rec[-2] | rec[-1] << 8)

    def _gap(self, first, last):
        n = last - first + 1
        self.gaps += 1
        self.missing += n
        log.warning("AURIX log gap: records %d..%d (%d) not received (UART too slow)", first, last, n)
        if self.storage.usable:
            try:
                day = time.strftime("%Y%m%d", self.clock.utc())
                p = os.path.join(self.storage.folder("aurix_logs", day), "gaps.csv")
                new = not os.path.exists(p)
                with open(p, "a", encoding="utf-8") as f:
                    if new:
                        f.write("utc,boot,first_record,last_record,count\n")
                    f.write("%s,%s,%d,%d,%d\n" % (self.clock.stamp(), self.boot, first, last, n))
            except OSError as e:
                log.warning("gaps.csv: %s", e)

    # --- files ---------------------------------------------------------------------
    def _close(self):
        if self.fh:
            try:
                self.fh.close()
            except OSError:
                pass
            if self.on_file_closed and self.path:
                self.on_file_closed(self.path)
        self.fh = None

    def _open_new(self):
        self._close()
        if not self.storage.usable:
            return
        try:
            day = time.strftime("%Y%m%d", self.clock.utc())
            name = "P%05d_%03d.TLG" % (self.boot or 0, self.file_no or 0)
            folder = self.storage.folder("aurix_logs", day)
            self.path = os.path.join(folder, name)
            if os.path.exists(self.path):               # same name again (AURIX reset without a new boot no.)
                self.path = os.path.join(folder, name[:-4] + "_%d.TLG" % int(time.time()))
            self.fh = open(self.path, "wb")
            self.fh.write(b"TLG1")
            for r in (self.header_rec, self.boot_rec, self.config_rec):
                if r:
                    self.fh.write(r)
            log.info("AURIX log copy: %s", self.path)
        except OSError as e:
            log.warning("cannot start %s: %s", self.path, e)
            self.fh = None

    def _write(self, rec):
        if self.fh is None:
            if self.header_rec is None or not self.storage.usable:
                self.lost_storage += 1
                return
            self._open_new()
            if self.fh is None:
                self.lost_storage += 1
                return
        try:
            self.fh.write(rec)
            self.bytes_written += len(rec)
            now = time.time()
            if now - self.last_flush > 2.0:             # like the AURIX: at most ~2 s lost
                self.last_flush = now
                self.fh.flush()
                os.fsync(self.fh.fileno())
        except OSError as e:
            log.warning("log copy write failed: %s", e)
            self.lost_storage += 1
            self._close()

    def flush(self):
        if self.fh:
            try:
                self.fh.flush()
            except OSError:
                pass

    def close(self):
        self._close()

    def status(self):
        return {"records": self.records, "last_seq": self.last_seq, "gaps": self.gaps, "missing": self.missing,
                "broken": self.broken, "lost_storage": self.lost_storage, "file": self.path,
                "mb": round(self.bytes_written / 1e6, 2)}
