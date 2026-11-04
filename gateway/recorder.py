"""Video recording on the gateway PC (section F): ring buffer, SAVE, storage policy.

Ring buffer
  One ffmpeg per camera copies the camera's main stream (from the MediaMTX
  relay, no re-encoding) into 1-minute files on the SSD:
      <ssd>/video/ring/<CAMERA>/YYYYMMDD-HHMMSS.ts      (UTC start time)
  MPEG-TS: a file cut short by a power loss still plays. Files older than
  the ring length (5 / 10 / 20 / 30 / 60 min, set by the captain) are
  deleted. A camera that stops is restarted every few seconds; the others
  never notice.

SAVE
  A SAVE keeps the whole ring window before it and the post-time after it
  (default 2 min):
      <ssd>/video/events/YYYYMMDD-HHMMSS_<trigger>/<CAMERA>/*.ts
      <ssd>/video/events/.../event.json     time, position, mode, alarms,
                                            triggers, link to the AURIX log
  The files are hard links to the ring files (instant, no extra space until
  the ring deletes its copy). A trigger while an event is still open (its
  post-time running) is merged into it and extends it.
  Triggers: SAVE buttons (captain / driver / shore joystick), the SAVE
  button on the boat's USB joystick and the AURIX's automatic requests
  (alarm, fault, joystick takeover, STANDBY by fault, slamming - TELEM2).
  A station's SAVE may name the event id (the station keeps its own screen
  recording under the same id, step 13n4).

KEEP 5' / 1h (step 13n4, owner 2026-10-31 E4)
  A station's "keep the last 5 / 60 min": an event with pre = 5 / 60 min and
  post = 0, never merged with another SAVE, "keep": true = PERMANENT: the
  storage policy never deletes it (also not once synced) and it is NOT
  uploaded to the shore (sync.py skips its video). A ring shorter than asked
  = what exists is kept and the answer says how much (have_min).

Storage policy (SSD free space below "low"):
  1. the oldest ring files (only synced ones if ring sync is switched on)
  2. the oldest saved events that the shore PC has confirmed (synced)
  never a saved event that is not synced yet. Still below "full": STORAGE
  FULL (GATEWAY STORAGE alarm on the boat), new SAVEs are refused, the ring
  keeps running.
Video is never safety-critical: nothing here can change anything on the boat.
"""
import calendar
import json
import os
import re
import shutil
import subprocess
import threading
import time

from common import log, proto

SEG_RE = re.compile(r"^(\d{8})-(\d{6})\.ts$")
EVENT_ID_RE = re.compile(r"^\d{8}-\d{6}_[A-Za-z0-9-]{1,40}$")    # an event id a station may name
KEEP_CHOICES_MIN = (5, 60)                                          # the KEEP buttons (step 13n4)


def is_keep(folder):
    """A permanent KEEP event (step 13n4): never deleted automatically, never uploaded."""
    try:
        with open(os.path.join(folder, "event.json"), encoding="utf-8") as f:
            return bool(json.load(f).get("keep"))
    except (OSError, ValueError):
        return False


def seg_time(name):
    """UTC start time (Unix s) of a ring file name, None if not one."""
    m = SEG_RE.match(name)
    if not m:
        return None
    d, t = m.group(1), m.group(2)
    return calendar.timegm((int(d[:4]), int(d[4:6]), int(d[6:]), int(t[:2]), int(t[2:4]), int(t[4:]), 0, 0, 0))


def is_synced(path):
    """A file the shore PC has confirmed (section G writes <file>.synced)."""
    return os.path.exists(path + ".synced")


def event_synced(folder):
    """Every file of a saved event confirmed by the shore PC."""
    n = 0
    for dirpath, _dirs, files in os.walk(folder):
        for f in files:
            if f.endswith((".synced", ".tmp")):
                continue
            if not is_synced(os.path.join(dirpath, f)):
                return False
            n += 1
    return n > 0


def slug(text):
    return re.sub(r"[^A-Za-z0-9]+", "-", text).strip("-")[:40] or "save"


def dur(s):
    return "%d s" % round(s) if s < 120 else "%g min" % round(s / 60.0, 1)


def ffmpeg_bin():
    import toolpaths
    return toolpaths.ffmpeg() or "ffmpeg"


class CamRecorder:
    """One camera's ring recording (one ffmpeg process)."""

    def __init__(self, name, url, folder, seg_s, ffmpeg, stall_s=10.0):
        self.name, self.url, self.folder, self.seg_s, self.ffmpeg = name, url, folder, seg_s, ffmpeg
        self.stall_s = stall_s
        self.proc = None
        self.state = "OFF"              # REC / STALLED / DOWN / OFF
        self.restarts = 0
        self.next_start = 0.0
        self.last_size = None
        self.last_grow = 0.0
        self.since = time.time()

    def start(self):
        os.makedirs(self.folder, exist_ok=True)
        env = dict(os.environ, TZ="UTC0")       # file names in UTC (strftime)
        cmd = [self.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-rtsp_transport", "tcp",
               "-timeout", "5000000", "-i", self.url, "-map", "0:v", "-c", "copy",
               "-f", "segment", "-segment_time", str(self.seg_s), "-segment_atclocktime", "1",
               "-strftime", "1", "-reset_timestamps", "1", "-segment_format", "mpegts",
               # every packet into the file at once (the inner muxer's buffer is 32 KB = 16 s of a
               # nearly still picture): a slow stream still grows, the stall check sees it
               "-segment_format_options", "flush_packets=1",
               os.path.join(self.folder, "%Y%m%d-%H%M%S.ts")]
        try:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL, env=env)
        except OSError as e:
            log.warning("recorder %s: ffmpeg did not start: %s", self.name, e)
            self.proc = None
        self.last_grow = time.time()
        self.last_size = None

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def segments(self):
        """[(start_t, path)] oldest first."""
        out = []
        try:
            for n in os.listdir(self.folder):
                t = seg_time(n)
                if t is not None:
                    out.append((t, os.path.join(self.folder, n)))
        except OSError:
            pass
        out.sort()
        return out

    def tick(self, now):
        """Keep ffmpeg running; notice a stream that stopped growing."""
        old = self.state
        if self.proc is None or self.proc.poll() is not None:
            if self.proc is not None:
                self.restarts += 1
                self.proc = None
            self.state = "DOWN"
            if now >= self.next_start:
                self.next_start = now + 3.0
                self.start()
        else:
            segs = self.segments()
            size = 0
            if segs and os.path.exists(segs[-1][1]):    # the newest file: a new one or more bytes = growth
                size = (segs[-1][0], os.path.getsize(segs[-1][1]))
            if size != self.last_size:
                self.last_size, self.last_grow = size, now
                self.state = "REC"
            elif now - self.last_grow > self.stall_s:
                self.state = "STALLED"                  # no data: restart (the camera may be back later)
                log.warning("recorder %s: no new video, restarting ffmpeg", self.name)
                self.stop()
                self.restarts += 1
                self.next_start = now + 1.0
            else:
                self.state = "REC" if self.last_size else "STARTING"
        if self.state != old:
            self.since = now


class Recorder:
    def __init__(self, cfg, gw):
        self.cfg = cfg
        self.gw = gw
        self.storage = gw.storage
        self.enabled = cfg.get("enabled", True)
        self.seg_s = cfg.get("segment_s", 60)
        self.choices = cfg.get("ring_choices_min", [5, 10, 20, 30, 60])
        self.post_s = cfg.get("post_s", 120)
        self.pre_s = cfg.get("pre_s", 600)              # None = the whole ring window
        self.ring_sync = cfg.get("ring_sync", False)    # section G: ring video to the shore (off)
        self.ring_min = cfg.get("ring_min", 60)
        self.cams = {}
        self.events = []                                # open events (dicts)
        self.recent = []                                # last few finished (status)
        self.saves = self.merged = self.refused = 0
        self.deleted = {"ring": 0, "events": 0}
        self.last_seq = None                            # AURIX SAVE request numbers (TELEM2)
        self.last_house = 0.0
        self.full = False
        self.state_path = None
        self.lock = threading.RLock()                   # events: the event loop (SAVE) + the worker (tick)
        if self.storage.usable:
            self.state_path = os.path.join(self.storage.folder("state"), "recording.json")
            try:
                with open(self.state_path, encoding="utf-8") as f:
                    v = json.load(f).get("ring_min")
                if v in self.choices:
                    self.ring_min = v
            except (OSError, ValueError):
                pass
        if self.ring_min not in self.choices:
            self.ring_min = self.choices[min(1, len(self.choices) - 1)]

    # --- folders ---------------------------------------------------------------------
    def ring_dir(self, cam=None):
        p = os.path.join(self.storage.root, "video", "ring")
        return os.path.join(p, cam) if cam else p

    def events_dir(self):
        return os.path.join(self.storage.root, "video", "events")

    # --- start / stop ------------------------------------------------------------------
    def start(self, rtsp_port, cams):
        """cams: {name: path on the relay} (the main streams)."""
        if not self.enabled:
            return
        ff = self.cfg.get("ffmpeg_bin") or ffmpeg_bin()
        for name, path in cams.items():
            self.cams[name] = CamRecorder(name, "rtsp://127.0.0.1:%d/%s" % (rtsp_port, path),
                                          self.ring_dir(name), self.seg_s, ff, self.cfg.get("stall_s", 10.0))

    def stop(self):
        for c in self.cams.values():
            c.stop()
        for ev in self.events:
            self._write(ev)

    # --- settings ----------------------------------------------------------------------
    def set_ring(self, minutes, user="?"):
        if minutes not in self.choices:
            return False, "ring length must be one of %s min" % self.choices
        self.ring_min = minutes
        if self.state_path:
            try:
                with open(self.state_path, "w", encoding="utf-8") as f:
                    json.dump({"ring_min": minutes}, f)
            except OSError:
                pass
        self.gw.audit("ring_length", user=user, minutes=minutes)
        return True, "ring buffer %g min" % minutes

    # --- SAVE ----------------------------------------------------------------------------
    def snapshot(self):
        """Boat state for the event metadata."""
        d = self.gw.telem or {}
        lg = self.gw.logs
        snap = {"utc": self.gw.clock.stamp()}
        if d:
            snap.update({"lat": round(d.get("est_lat", 0.0), 6), "lon": round(d.get("est_lon", 0.0), 6),
                         "sog_kn": round(d.get("est_sog", 0.0), 1), "heading": round(d.get("heading", 0.0), 1),
                         "mode": d.get("mode_name"),
                         "alarms": proto.alarm_texts(d.get("alarm_bits", 0)) +
                         proto.alarm_texts2(getattr(self.gw, "telem2", {}).get("alarm_bits2", 0)),
                         "throttle": d.get("throttle"), "boat_t_ms": d.get("t_ms")})
        else:
            snap["boat"] = "no telemetry"
        snap["log"] = {"file": os.path.relpath(lg.path, self.storage.root) if lg.path else None,
                       "aurix_record": lg.last_seq, "boot": lg.boot}
        return snap

    def trigger(self, kind, text, who=None, now=None, eid=None):
        """A SAVE. Returns (ok, message, event id). eid: the id a station chose (its screen part, 13n4)."""
        with self.lock:
            return self._trigger(kind, text, who, now, eid)

    def ring_have_s(self, now=None):
        """How far back the ring reaches now (the longest camera), s."""
        now = time.time() if now is None else now
        have = 0.0
        for c in self.cams.values():
            segs = c.segments()
            if segs:
                have = max(have, now - segs[0][0])
        return min(have, self.ring_min * 60 + self.seg_s)

    def _new_folder(self, eid, now, kind):
        if not (isinstance(eid, str) and EVENT_ID_RE.match(eid)):
            eid = time.strftime("%Y%m%d-%H%M%S", time.gmtime(now)) + "_" + slug(kind)
        path = os.path.join(self.events_dir(), eid)
        k = 1
        while os.path.exists(path):
            k += 1
            path = os.path.join(self.events_dir(), "%s-%d" % (eid, k))
        os.makedirs(path, exist_ok=True)
        return os.path.basename(path), path

    def keep(self, minutes, who=None, now=None, eid=None):
        """KEEP the last 5 / 60 min of every camera permanently (step 13n4). -> (ok, message, id, have_min)."""
        with self.lock:
            now = time.time() if now is None else now
            if minutes not in KEEP_CHOICES_MIN:
                return False, "keep: 5 or 60 min only", None, None
            if not self.enabled or not self.storage.usable:
                self.refused += 1
                return False, "no recording (SSD %s)" % self.storage.state, None, None
            if self.storage.state == "FULL":
                self.refused += 1
                self.gw.audit("save_refused", trigger="keep%d" % minutes, reason="STORAGE FULL")
                return False, "STORAGE FULL: KEEP not possible - free space on the SSD (sync / delete events)",                     None, None
            pre = minutes * 60.0
            have = self.ring_have_s(now)
            eid, path = self._new_folder(eid, now, "keep%d" % minutes)
            trig = {"utc": self.gw.clock.stamp(now), "t": round(now, 3), "kind": "keep%d" % minutes,
                    "text": "KEEP %d min (%s)" % (minutes, who or "?")}
            if who:
                trig["who"] = who
            ev = {"id": eid, "path": path, "state": "recording", "keep": True, "keep_min": minutes,
                  "have_min": round(have / 60.0, 1), "start_t": round(now - pre, 3), "end_t": now,
                  "start_utc": self.gw.clock.stamp(now - pre), "end_utc": self.gw.clock.stamp(now),
                  "ring_min": self.ring_min, "pre_s": pre, "post_s": 0, "segment_s": self.seg_s,
                  "triggers": [trig], "boat": self.snapshot(), "cameras": {n: [] for n in self.cams}, "bytes": 0}
            self.events.append(ev)
            self.saves += 1
            self._link(ev)
            self._write(ev)
            self.gw.audit("save_keep", id=eid, minutes=minutes, have_min=ev["have_min"], user=who)
            log.info("KEEP %s: %d min asked, %.1f min in the ring", eid, minutes, have / 60.0)
            short = have < pre - self.seg_s
            return True, ("KEEP %s: %d min" % (eid, minutes)) + (
                " - cameras: only %g min available" % ev["have_min"] if short else ""), eid, ev["have_min"]

    def _trigger(self, kind, text, who, now, eid=None):
        now = time.time() if now is None else now
        trig = {"utc": self.gw.clock.stamp(now), "t": round(now, 3), "kind": kind, "text": text}
        if who:
            trig["who"] = who
        for ev in self.events:
            if now <= ev["end_t"] and not ev.get("keep"):   # still open: merge (never into a KEEP)
                ev["triggers"].append(trig)
                ev["end_t"] = max(ev["end_t"], now + self.post_s)
                ev["end_utc"] = self.gw.clock.stamp(ev["end_t"])
                self.merged += 1
                self._write(ev)
                self.gw.audit("save_merged", id=ev["id"], trigger=kind, text=text, user=who)
                return True, "added to the open SAVE %s (now until %s)" % (ev["id"], ev["end_utc"][11:19]), ev["id"]
        if not self.enabled or not self.storage.usable:
            self.refused += 1
            return False, "no recording (SSD %s)" % self.storage.state, None
        if self.storage.state == "FULL":
            self.refused += 1
            self.gw.audit("save_refused", trigger=kind, text=text, reason="STORAGE FULL")
            return False, "STORAGE FULL: SAVE not possible - free space on the SSD (sync / delete events)", None
        pre = self.pre_s if self.pre_s is not None else self.ring_min * 60
        eid, path = self._new_folder(eid, now, kind)
        ev = {"id": eid, "path": path, "state": "recording", "start_t": round(now - pre, 3), "end_t": now + self.post_s,
              "start_utc": self.gw.clock.stamp(now - pre), "end_utc": self.gw.clock.stamp(now + self.post_s),
              "ring_min": self.ring_min, "pre_s": pre, "post_s": self.post_s, "segment_s": self.seg_s,
              "triggers": [trig], "boat": self.snapshot(), "cameras": {n: [] for n in self.cams},
              "bytes": 0}
        self.events.append(ev)
        self.saves += 1
        self._link(ev)
        self._write(ev)
        self.gw.audit("save", id=eid, trigger=kind, text=text, user=who)
        log.info("SAVE %s (%s)", eid, text)
        return True, "SAVE %s: %s before + %s after" % (eid, dur(pre), dur(self.post_s)), eid

    def on_telem2(self, d):
        """The AURIX's SAVE requests (TELEM2 group E): a new number = a SAVE."""
        seq = d.get("save_seq")
        if seq is None:
            return
        if self.last_seq is None:                       # gateway (re)started: old requests are history
            self.last_seq = seq
            return
        if seq == self.last_seq or seq == 0:
            self.last_seq = seq
            return
        n = (seq - self.last_seq) & 0xFFFF if seq > self.last_seq else seq
        self.last_seq = seq
        kinds = d.get("save_kinds", 0) | (1 << d.get("save_trig", 0))
        names = [proto.SAVE_TRIGGERS[k] for k in sorted(proto.SAVE_TRIGGERS) if kinds & (1 << k)]
        text = proto.save_trigger_text(d.get("save_trig", 0), d.get("save_arg", 0))
        if len(names) > 1:
            text += " (+ " + ", ".join(nm for nm in names if not text.startswith(nm)) + ")"
        if n > 1:
            text += " [%d requests]" % n
        kind = {1: "button", 2: "alarm", 3: "fault", 4: "takeover", 5: "standby-fault", 6: "slamming"}.get(
            d.get("save_trig", 0), "aurix")
        self.trigger(kind, text, who="AURIX", now=time.time() - d.get("save_age_ms", 0) / 1000.0)

    # --- events ----------------------------------------------------------------------------
    def _link(self, ev):
        """Hard-link every finished ring file inside the event window."""
        done = True
        for name, c in self.cams.items():
            segs = c.segments()
            have = set(ev["cameras"].setdefault(name, []))
            for i, (t, p) in enumerate(segs):
                if t + self.seg_s < ev["start_t"] or t > ev["end_t"]:
                    continue
                if i == len(segs) - 1 and c.proc is not None and c.proc.poll() is None:
                    if t <= ev["end_t"]:
                        done = False                    # still being written
                    continue
                fn = os.path.basename(p)
                if fn in have:
                    continue
                dst = os.path.join(ev["path"], name)
                os.makedirs(dst, exist_ok=True)
                try:
                    os.link(p, os.path.join(dst, fn))
                except OSError:
                    try:
                        shutil.copy2(p, os.path.join(dst, fn))
                    except OSError as e:
                        log.warning("SAVE %s: %s not kept: %s", ev["id"], fn, e)
                        continue
                ev["cameras"][name].append(fn)
                ev["bytes"] += os.path.getsize(p)
            ev["cameras"][name].sort()
        return done

    def _write(self, ev):
        out = {k: v for k, v in ev.items() if k != "path"}
        tmp = os.path.join(ev["path"], "event.json.tmp")
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=1)
            os.replace(tmp, os.path.join(ev["path"], "event.json"))
        except OSError as e:
            log.warning("SAVE %s: event.json: %s", ev["id"], e)

    def list_events(self):
        """[(start_t, folder, event dict)] of every saved event on the SSD, oldest first."""
        out = []
        root = self.events_dir()
        try:
            names = os.listdir(root)
        except OSError:
            return out
        for n in names:
            p = os.path.join(root, n, "event.json")
            try:
                with open(p, encoding="utf-8") as f:
                    ev = json.load(f)
                out.append((ev.get("start_t", 0), os.path.join(root, n), ev))
            except (OSError, ValueError):
                continue
        out.sort(key=lambda x: x[0])
        return out

    # --- housekeeping ----------------------------------------------------------------------
    def tick(self, now=None):
        """Once a second, in a worker thread (file work must never stall the event loop)."""
        now = time.time() if now is None else now
        for c in self.cams.values():
            c.tick(now)
        with self.lock:
            self._tick_events(now)
        if now - self.last_house >= self.cfg.get("house_s", 5.0):
            self.last_house = now
            self.housekeeping(now)

    def _tick_events(self, now):
        for ev in list(self.events):
            done = self._link(ev)
            if now > ev["end_t"] and (done or now > ev["end_t"] + self.seg_s + 30):
                ev["state"] = "complete" if done else "incomplete"
                self._write(ev)
                self.events.remove(ev)
                self.recent = (self.recent + [{"id": ev["id"], "state": ev["state"], "mb": round(ev["bytes"] / 1e6, 1),
                                               "keep": bool(ev.get("keep")),
                                               "triggers": len(ev["triggers"]),
                                               "text": ev["triggers"][0]["text"]}])[-5:]
                log.info("SAVE %s %s (%.1f MB)", ev["id"], ev["state"], ev["bytes"] / 1e6)
            else:
                self._write(ev)

    def _rm(self, p):
        for q in (p, p + ".synced"):
            try:
                os.remove(q)
            except OSError:
                pass

    def housekeeping(self, now=None):
        """Ring length, then the storage policy."""
        now = time.time() if now is None else now
        keep_from = now - self.ring_min * 60 - self.seg_s
        for c in self.cams.values():
            segs = c.segments()
            for t, p in segs[:-1]:                      # never the file being written
                if t < keep_from:
                    self._rm(p)
                    self.deleted["ring"] += 1
        if not self.storage.usable:
            return
        free = self.storage.free_now()
        if free >= self.storage.low_b:
            self._set_full(False)
            return
        # 1. the oldest ring files (only synced ones if ring sync is on)
        ring = []
        for c in self.cams.values():
            segs = c.segments()
            ring += [(t, p) for t, p in segs[:-1] if not self.ring_sync or is_synced(p)]
        ring.sort()
        for t, p in ring:
            if free >= self.storage.low_b:
                break
            sz = os.path.getsize(p) if os.path.exists(p) else 0
            self._rm(p)
            self.deleted["ring"] += 1
            free = self.storage.free_now() if sz else free
        # 2. the oldest saved events the shore PC has confirmed
        with self.lock:
            open_ids = {e["id"] for e in self.events}
        for t, folder, ev in self.list_events():
            if free >= self.storage.low_b:
                break
            if not event_synced(folder) or ev.get("id") in open_ids or ev.get("keep"):
                continue                                # 3. never an event that is not on shore yet
                                                        #    (every file confirmed by the shore: <file>.synced)
                                                        #    and never a KEEP 5' / 1h (permanent, step 13n4)
            shutil.rmtree(folder, ignore_errors=True)
            self.deleted["events"] += 1
            self.gw.audit("event_deleted", id=ev.get("id"), reason="SSD space (synced)")
            log.info("storage: deleted synced event %s", ev.get("id"))
            free = self.storage.free_now()
        self.storage.check(force=True)
        self._set_full(self.storage.state == "FULL")

    def _set_full(self, full):
        if full != self.full:
            self.full = full
            (log.warning if full else log.info)("STORAGE FULL %s", "ON: SAVE refused" if full else "over")
            self.gw.audit("storage_full", on=full)

    # --- status ----------------------------------------------------------------------------
    def status(self):
        now = time.time()
        cams, ring_b, ring_s = [], 0, 0.0
        for n, c in self.cams.items():
            segs = c.segments()
            b = sum(os.path.getsize(p) for t, p in segs if os.path.exists(p))
            span = (now - segs[0][0]) if segs else 0.0
            ring_b += b
            ring_s = max(ring_s, span)
            cams.append({"name": n, "state": c.state, "for_s": int(now - c.since), "files": len(segs),
                         "mb": round(b / 1e6, 1), "restarts": c.restarts})
        gb_h = (ring_b / ring_s * 3600 / 1e9) if ring_s > 10 else None
        return {"enabled": self.enabled, "ring_min": self.ring_min, "choices": self.choices, "post_s": self.post_s,
                "pre_s": self.pre_s if self.pre_s is not None else self.ring_min * 60,
                "cameras": cams, "ring_mb": round(ring_b / 1e6, 1), "ring_have_min": round(ring_s / 60.0, 1),
                "gb_per_hour": round(gb_h, 2) if gb_h is not None else None,
                "open": [{"id": e["id"], "until": e["end_utc"][11:19], "triggers": len(e["triggers"])}
                         for e in self.events],
                "recent": self.recent, "saves": self.saves, "merged": self.merged, "refused": self.refused,
                "deleted": self.deleted, "full": self.full}
