"""Real-trip video lessons (ACADEMY_SPEC F, step 20) - the model (no window, no process, no link).

Source = a SAVED EVENT of the boat (gateway/recorder.py: <root>/video/events/<id>/event.json + one folder of
1-minute .ts files per camera, named by their UTC start time; the shore PC gets them by the sync, section G)
and the AURIX log it points to (event.json boat.log.file, a .TLG; its TIME events give UTC). Everything is
opened ONLY through simguard (check_recording / video_frame / video_copy): recorded files, never a camera,
a gateway or a boat.

The LESSON EDITOR (Draft) makes a lesson from an event:
  trim (clip start / end), cameras, pause points = DECISION questions (choices, el + en) and SPOT THE HAZARD
  questions (areas to click on a camera's picture), BLUR areas (faces, number plates: a box on a camera from
  / to a time) and the SMALL TRIMMED COPIES (simguard.video_copy: one .mp4 per camera, scaled, blurred, no
  sound) + the HUD timeline (hud.jsonl: the log's values 5 times a second over the clip, so the player redraws
  the station's own HUD (groundstation/hud.py) over the video, in sync).
A lesson = academy/data/videolessons/<id>/ (lesson.json kind "videolesson" + the copies + hud.jsonl);
gitignored - real trip video never goes into git. The player plays ONLY the lesson's own copies (blurred),
never the original event files.

The PLAYER model (Run): the clock runs from 0 to the clip length; at every pause point it stops and asks;
a decision = pick a choice, a hazard = click on the picture (inside an area = right); one answer per point,
the explanation after it; score = right / points in %, passed at PASS_PCT; logbook kind "videolesson".
"""
import bisect
import calendar
import json
import os
import re
import sys

import content
import simguard

HERE = os.path.dirname(os.path.abspath(__file__))
GS_DIR = os.path.join(os.path.dirname(HERE), "groundstation")
if GS_DIR not in sys.path:
    sys.path.append(GS_DIR)                 # tlg.py / proto.py / hud.py (read only, no link)

SEG_RE = re.compile(r"^(\d{8})-(\d{6})\.ts$")
EV_TIME = 16                    # app/logrec.h EV_TIME: GPS date, GPS time of day ms, local t_ms
HUD_RATE_HZ = 5.0               # the HUD timeline
MAX_CLIP_S = 600.0              # a lesson clip: at most 10 min
MIN_CLIP_S = 3.0
COPY_WIDTH = 640                # the small copies
PASS_PCT = 70
POINT_TYPES = ("decision", "hazard")
ID_RE = re.compile(r"^[A-Za-z0-9_\-]{1,60}$")
CAM_RE = re.compile(r"^[A-Za-z0-9_\-]{1,40}$")
# the HUD's values taken from the log (boat SD log names first, then the ground station's telemetry copy)
HUD_ALIASES = {
    "mode": ["mode"], "heading": ["heading"], "target": ["target"], "yaw_rate": ["yaw_rate"],
    "outboard": ["outboard", "outboard_est"], "throttle": ["throttle", "thr_out"],
    "throttle_demand": ["throttle_demand", "thr_mode"], "gear": ["gear"], "power_fwd": ["power_fwd"],
    "power_rev": ["power_rev"], "est_sog": ["est_sog", "gps_sog"], "est_cog": ["est_cog", "gps_cog"],
    "gps_sog": ["gps_sog"], "est_lat": ["est_lat", "gps_lat"], "est_lon": ["est_lon", "gps_lon"],
    "alarm_bits": ["alarm_bits"], "alarm_bits2": ["alarm_bits2"], "compass_ok": ["compass_ok"],
    "flags": ["flags"], "speed_set_kn": ["speed_set_kn"], "pos_source": ["pos_source"], "gps_fix": ["gps_fix"],
    "unc_m": ["unc_m", "unc"], "auto_state": ["auto_state"], "auto_wp": ["auto_wp"], "bank_v": ["bank_v"],
}


def lessons_dir(env=None):
    return simguard.videolessons_dir(env)


# --- the saved event ----------------------------------------------------------------------------
def seg_time(name):
    m = SEG_RE.match(name)
    if not m:
        return None
    d, t = m.group(1), m.group(2)
    return calendar.timegm((int(d[:4]), int(d[4:6]), int(d[6:]), int(t[:2]), int(t[2:4]), int(t[4:]), 0, 0, 0))


def gps_epoch(date, tod_ms):
    y, m, d = date // 10000, (date // 100) % 100, date % 100
    return calendar.timegm((y, m, d, 0, 0, 0, 0, 0, 0)) + tod_ms / 1000.0


def log_utc(log):
    """boat t_ms -> UTC seconds from the log's TIME events (as groundstation/videolog.py); None without GPS time."""
    pts = sorted((t, gps_epoch(a[0], a[1]) - a[2] / 1000.0) for t, _txt, code, a in log.events
                 if code == EV_TIME and a[0] > 19700101)
    if not pts:
        return None
    ts = [p[0] for p in pts]

    def f(t_ms):
        i = min(max(bisect.bisect_right(ts, t_ms) - 1, 0), len(pts) - 1)
        return pts[i][1] + t_ms / 1000.0
    return f


class Event(object):
    """A saved event opened from its event.json (through simguard). cams = {name: [(utc start, path)]}."""

    def __init__(self, event_json, env=None):
        p = simguard.check_recording(event_json, env)
        if os.path.basename(p).lower() != "event.json":
            raise simguard.Refused("%s is not a saved event (event.json)" % event_json)
        with open(p, encoding="utf-8") as f:
            self.doc = json.load(f)
        self.path, self.folder = p, os.path.dirname(p)
        self.id = str(self.doc.get("id") or os.path.basename(self.folder))
        self.seg_s = float(self.doc.get("segment_s", 60.0) or 60.0)
        self.cams = {}
        for cam in sorted(os.listdir(self.folder)):
            d = os.path.join(self.folder, cam)
            if not os.path.isdir(d) or not CAM_RE.match(cam):
                continue
            segs = sorted((seg_time(n), os.path.join(d, n)) for n in os.listdir(d) if seg_time(n) is not None)
            if segs:
                self.cams[cam] = segs
        trig = (self.doc.get("triggers") or [{}])[0]
        self.trigger_utc = trig.get("t")
        self.trigger_text = trig.get("text", "")
        starts = [s[0][0] for s in self.cams.values()]
        ends = [s[-1][0] + self.seg_s for s in self.cams.values()]
        self.start_utc = max(min(starts) if starts else 0.0, float(self.doc.get("start_t") or 0.0))
        self.end_utc = min(max(ends) if ends else 0.0, float(self.doc.get("end_t") or 1e12))
        if starts and self.end_utc <= self.start_utc:                  # metadata vs files: trust the files
            self.start_utc, self.end_utc = min(starts), max(ends)

    def cameras(self):
        return sorted(self.cams)

    def segments(self, cam, t0, t1):
        """The camera's files covering [t0, t1] (UTC) and the offset of t0 into the first one."""
        segs = self.cams.get(cam) or []
        use = [(s, p) for s, p in segs if s < t1 and s + self.seg_s > t0]
        if not use:
            return [], 0.0
        return [p for _s, p in use], max(0.0, t0 - use[0][0])

    def locate(self, cam, utc):
        """(file, seconds into it) of a camera at a UTC time, or None (the editor's preview)."""
        segs = self.cams.get(cam) or []
        starts = [s for s, _p in segs]
        i = bisect.bisect_right(starts, utc) - 1
        if i < 0 or utc - starts[i] > self.seg_s + 1.0:
            return None
        return segs[i][1], utc - starts[i]

    def log_path(self):
        """The AURIX log of the event (event.json boat.log.file, relative to the sync root), or None."""
        lf = ((self.doc.get("boat") or {}).get("log") or {}).get("file")
        if not lf:
            return None
        root = os.path.dirname(os.path.dirname(os.path.dirname(self.folder)))      # <root>/video/events/<id>
        p = os.path.join(root, lf.replace("\\", os.sep).replace("/", os.sep))
        return p if os.path.isfile(p) else None


def read_log(path, env=None):
    import tlg
    return tlg.read(simguard.check_recording(path, env))


def hud_values(sample):
    """The HUD's values of one log sample (aliases resolved)."""
    out = {}
    for key, names in HUD_ALIASES.items():
        for n in names:
            if n in sample and sample[n] is not None:
                v = sample[n]
                out[key] = round(v, 5) if isinstance(v, float) else v
                break
    return out


def hud_timeline(log, t0_utc, t1_utc, rate=HUD_RATE_HZ):
    """[{"t": seconds into the clip, "d": HUD values}] over [t0, t1] from the log, `rate` per second; [] when the
    log has no GPS time (then the video can't be synced)."""
    utc = log_utc(log)
    if utc is None or not log.samples:
        return []
    pts = [(utc(s["t_ms"]), s) for s in log.samples]
    times = [p[0] for p in pts]
    out = []
    n = int((t1_utc - t0_utc) * rate) + 1
    for k in range(n):
        t = t0_utc + k / rate
        i = bisect.bisect_right(times, t) - 1
        if i < 0 or t - times[i] > 2.0:
            continue
        out.append({"t": round(k / rate, 3), "d": hud_values(pts[i][1])})
    return out


def hud_state(values):
    """A full telemetry dict for groundstation/hud.py: an all-zero frame + the logged values + the derived keys."""
    import proto
    d = proto.decode_telemetry(bytes(proto.TELEM_SIZE))
    d.update(values or {})
    m = int(d.get("mode") or 0)
    d["mode"] = m
    d["mode_name"] = proto.MODES[m] if 0 <= m < len(proto.MODES) else "?"
    if "compass_ok" in (values or {}):
        d["compass_ok"] = bool(values["compass_ok"])
    elif "flags" in (values or {}):
        d["compass_ok"] = bool(int(values["flags"]) & 1)
    for k in ("alarm_bits", "alarm_bits2", "gear"):
        d[k] = int(d.get(k) or 0)
    return d


def at_time(timeline, t):
    """The HUD values at clip time t (the newest sample not after t), or None."""
    if not timeline:
        return None
    ts = [p["t"] for p in timeline]
    i = bisect.bisect_right(ts, t + 1e-6) - 1
    return timeline[max(i, 0)]["d"] if i >= 0 or ts[0] - t < 0.5 else None


# --- the lesson file -----------------------------------------------------------------------------
def _box_ok(b):
    try:
        x, y, w, h = (float(b[k]) for k in ("x", "y", "w", "h"))
    except (KeyError, TypeError, ValueError):
        return False
    return 0.0 <= x < 1.0 and 0.0 <= y < 1.0 and 0.0 < w <= 1.0 - x + 1e-9 and 0.0 < h <= 1.0 - y + 1e-9


def check_doc(doc, folder=None):
    """Problems of a video lesson (empty = OK). folder = the lesson's folder: the media must be there."""
    p = list(content.check(doc, doc.get("id", "?") if isinstance(doc, dict) else "?"))
    if not isinstance(doc, dict):
        return p
    name = doc.get("id", "?")
    if doc.get("kind") != "videolesson":
        p.append("%s: kind must be videolesson" % name)
    if not isinstance(doc.get("id"), str) or not ID_RE.match(doc["id"]):
        p.append("%s: id must be letters / digits / _ -" % name)
    for k in ("title_el", "title_en"):
        if not isinstance(doc.get(k), str) or not doc[k].strip():
            p.append("%s: no %s" % (name, k))
    clip = doc.get("clip") or {}
    try:
        length = float(clip["end_utc"]) - float(clip["start_utc"])
    except (KeyError, TypeError, ValueError):
        length = -1.0
    if not MIN_CLIP_S <= length <= MAX_CLIP_S:
        p.append("%s: clip length %.1f s (%g..%g s)" % (name, length, MIN_CLIP_S, MAX_CLIP_S))
    cams = doc.get("cameras") or []
    if not cams or not all(isinstance(c, str) and CAM_RE.match(c) for c in cams):
        p.append("%s: cameras missing / bad names" % name)
    media = doc.get("media") or {}
    for c in cams:
        m = media.get(c)
        if not isinstance(m, str) or os.path.basename(m) != m or not m.lower().endswith(".mp4"):
            p.append("%s: camera %s: media must be a plain .mp4 file name in the lesson folder" % (name, c))
        elif folder is not None and not os.path.isfile(os.path.join(folder, m)):
            p.append("%s: camera %s: %s missing" % (name, c, m))
    if doc.get("main_camera") not in cams:
        p.append("%s: main_camera must be one of the cameras" % name)
    tl = doc.get("timeline")
    if tl is not None and (not isinstance(tl, str) or os.path.basename(tl) != tl or not tl.endswith(".jsonl")):
        p.append("%s: timeline must be a plain .jsonl name" % name)
    for i, b in enumerate(doc.get("blur") or []):
        if b.get("camera") not in cams or not _box_ok(b) or not 0.0 <= float(b.get("from_s", 0)) < \
                float(b.get("to_s", 0)) <= length + 1e-6:
            p.append("%s: blur %d: camera / box 0..1 / from_s < to_s inside the clip" % (name, i))
    last = -1.0
    pts = doc.get("points") or []
    for i, pt in enumerate(pts):
        where = "%s: point %d" % (name, i)
        t = pt.get("t_s")
        if not isinstance(t, (int, float)) or not 0.0 <= t <= length or t < last:
            p.append("%s: t_s inside the clip, in time order" % where)
        else:
            last = t
        if pt.get("type") not in POINT_TYPES:
            p.append("%s: type must be %s" % (where, " / ".join(POINT_TYPES)))
        for k in ("q_el", "q_en", "why_el", "why_en"):
            if not isinstance(pt.get(k), str) or not pt[k].strip():
                p.append("%s: no %s" % (where, k))
        if pt.get("type") == "decision":
            n = len(pt.get("choices") or [])
            if n < 2 or not isinstance(pt.get("answer"), int) or not 0 <= pt["answer"] < n:
                p.append("%s: answer must index its %d choices" % (where, n))
        elif pt.get("type") == "hazard":
            if pt.get("camera") not in cams:
                p.append("%s: hazard camera must be one of the cameras" % where)
            areas = pt.get("areas") or []
            if not areas or not all(_box_ok(a) for a in areas):
                p.append("%s: hazard areas (x, y, w, h inside 0..1) missing" % where)
    if doc.get("runs_on") != "recording":
        p.append("%s: runs_on must be \"recording\" (a recorded trip, never a live boat)" % name)
    return p


def clip_length(doc):
    c = doc["clip"]
    return float(c["end_utc"]) - float(c["start_utc"])


def load_lessons(folder=None, env=None):
    """[(doc, lesson folder, problems)] of every lesson under the video-lesson folder (bad ones included with
    their problems: the window lists them greyed)."""
    root = folder or lessons_dir(env)
    out = []
    try:
        names = sorted(os.listdir(root))
    except OSError:
        return out
    for n in names:
        f = os.path.join(root, n, "lesson.json")
        if not os.path.isfile(f):
            continue
        try:
            with open(simguard.check_recording(f, env), encoding="utf-8") as fh:
                doc = json.load(fh)
        except (OSError, ValueError, simguard.Refused) as e:
            out.append(({"id": n, "title_el": n, "title_en": n}, os.path.dirname(f), [str(e)]))
            continue
        out.append((doc, os.path.dirname(f), check_doc(doc, os.path.dirname(f))))
    return out


def media_path(doc, folder, cam, env=None):
    """The lesson's OWN small copy of a camera (never the event's original); checked by simguard."""
    m = (doc.get("media") or {}).get(cam)
    if not isinstance(m, str) or os.path.basename(m) != m:
        raise simguard.Refused("camera %s: no media in the lesson" % cam)
    p = os.path.join(folder, m)
    if not simguard._inside(p, lessons_dir(env)):
        raise simguard.Refused("%s is outside the video-lesson folder" % p)
    return simguard.check_video(p, env)


def read_timeline(doc, folder, env=None):
    tl = doc.get("timeline")
    if not tl:
        return []
    out = []
    try:
        with open(simguard.check_recording(os.path.join(folder, tl), env), encoding="utf-8") as f:
            for line in f:
                try:
                    e = json.loads(line)
                    if isinstance(e, dict) and isinstance(e.get("d"), dict):
                        out.append(e)
                except ValueError:
                    pass
    except (OSError, simguard.Refused):
        return []
    return out


# --- the player model ----------------------------------------------------------------------------
def inside(areas, x, y):
    return any(a["x"] <= x <= a["x"] + a["w"] and a["y"] <= y <= a["y"] + a["h"] for a in areas)


class Run(object):
    """One go through a lesson. advance(t) moves the clock (stops at a pause point: .asking), answer_choice /
    click answer it (one answer per point), resume() goes on."""

    def __init__(self, doc, lang="el"):
        self.doc, self.lang = doc, lang
        self.points = list(doc.get("points") or [])
        self.length = clip_length(doc)
        self.t = 0.0
        self.next = 0                       # index of the next pause point
        self.asking = None                  # the point being asked (index) or None
        self.answered = None                # (index, right) after an answer, until resume()
        self.results = []                   # [(index, right)]
        self.finished = False

    def tx(self, obj, key):
        return obj.get("%s_%s" % (key, self.lang)) or obj.get(key + "_el") or ""

    def advance(self, t):
        """Move the clock to t (never past the next pause point). Returns the point index when one is reached."""
        if self.finished or self.asking is not None or self.answered is not None:
            return None
        if self.next < len(self.points) and t >= self.points[self.next]["t_s"]:
            self.t = float(self.points[self.next]["t_s"])
            self.asking = self.next
            return self.asking
        self.t = min(max(t, 0.0), self.length)
        if self.t >= self.length - 1e-6 and self.next >= len(self.points):
            self.finished = True
        return None

    def point(self):
        return self.points[self.asking] if self.asking is not None else None

    def _answer(self, right):
        i = self.asking
        self.results.append((i, bool(right)))
        self.answered = (i, bool(right))
        self.asking = None
        self.next = i + 1
        return bool(right)

    def answer_choice(self, k):
        pt = self.point()
        if pt is None or pt.get("type") != "decision":
            return None
        return self._answer(k == pt.get("answer"))

    def click(self, cam, x, y):
        """A click on the picture of camera `cam` at (x, y) = parts 0..1 of the picture."""
        pt = self.point()
        if pt is None or pt.get("type") != "hazard":
            return None
        return self._answer(cam == pt.get("camera") and inside(pt.get("areas") or [], x, y))

    def explanation(self):
        if self.answered is None:
            return ""
        return self.tx(self.points[self.answered[0]], "why")

    def resume(self):
        self.answered = None
        if self.next >= len(self.points) and self.t >= self.length - 1e-6:
            self.finished = True

    def score(self):
        if not self.points:
            return 100
        return int(round(100.0 * sum(1 for _i, r in self.results if r) / len(self.points)))

    def log_entry(self):
        sc = self.score()
        return {"id": self.doc.get("id"), "topic": self.doc.get("id"), "title": self.doc.get("title_en"),
                "score": sc, "done": bool(self.finished and len(self.results) == len(self.points) and sc >= PASS_PCT),
                "right": sum(1 for _i, r in self.results if r), "of": len(self.points),
                "source": (self.doc.get("source") or {}).get("event")}


# --- the lesson editor model ---------------------------------------------------------------------
class Draft(object):
    """A lesson being made from a saved event. Times in the draft = seconds from the EVENT start."""

    def __init__(self, event, lesson_id=None):
        self.ev = event
        self.id = lesson_id or ("v_" + re.sub(r"[^A-Za-z0-9_\-]", "-", event.id))[:60]
        self.title = {"el": "", "en": ""}
        self.intro = {"el": "", "en": ""}
        self.level = "DRIVER"
        self.length_event = max(0.0, event.end_utc - event.start_utc)
        trig = (event.trigger_utc or event.start_utc) - event.start_utc
        self.start = max(0.0, trig - 30.0)
        self.end = min(self.length_event, trig + 30.0)
        if self.end - self.start < MIN_CLIP_S:
            self.start, self.end = 0.0, self.length_event
        self.cameras = event.cameras()
        self.main = self.cameras[0] if self.cameras else None
        self.points = []                    # times = seconds from the EVENT start
        self.blur = []
        self.width = COPY_WIDTH

    # -- editing
    def trim(self, start, end):
        start, end = max(0.0, float(start)), min(self.length_event, float(end))
        if end - start < MIN_CLIP_S or end - start > MAX_CLIP_S:
            raise ValueError("clip %.1f s: %g..%g s" % (end - start, MIN_CLIP_S, MAX_CLIP_S))
        self.start, self.end = start, end

    def set_cameras(self, cams, main=None):
        cams = [c for c in self.ev.cameras() if c in cams]
        if not cams:
            raise ValueError("at least one camera")
        self.cameras = cams
        self.main = main if main in cams else (self.main if self.main in cams else cams[0])

    def add_decision(self, t, q, choices, answer, why):
        """q / why = {"el", "en"}; choices = [{"el", "en"}]; answer = index."""
        pt = {"t": float(t), "type": "decision", "q_el": q["el"], "q_en": q["en"], "why_el": why["el"],
              "why_en": why["en"], "choices": [{"text_el": c["el"], "text_en": c["en"]} for c in choices],
              "answer": int(answer)}
        self.points.append(pt)
        self.points.sort(key=lambda p: p["t"])
        return pt

    def add_hazard(self, t, cam, q, why, areas):
        pt = {"t": float(t), "type": "hazard", "camera": cam, "q_el": q["el"], "q_en": q["en"],
              "why_el": why["el"], "why_en": why["en"], "areas": [dict(a) for a in areas]}
        self.points.append(pt)
        self.points.sort(key=lambda p: p["t"])
        return pt

    def add_blur(self, cam, box, t0, t1):
        b = {"camera": cam, "x": box["x"], "y": box["y"], "w": box["w"], "h": box["h"], "t0": float(t0),
             "t1": float(t1)}
        self.blur.append(b)
        return b

    def remove_point(self, i):
        del self.points[i]

    def remove_blur(self, i):
        del self.blur[i]

    # -- the lesson document
    def to_doc(self):
        s0 = self.ev.start_utc + self.start
        L = self.end - self.start
        blur = []
        for b in self.blur:
            f, t = max(0.0, b["t0"] - self.start), min(L, b["t1"] - self.start)
            if b["camera"] in self.cameras and t > f:
                blur.append({"camera": b["camera"], "x": b["x"], "y": b["y"], "w": b["w"], "h": b["h"],
                             "from_s": round(f, 2), "to_s": round(t, 2)})
        pts = []
        for p in self.points:
            if self.start <= p["t"] <= self.end:
                q = dict(p)
                q["t_s"] = round(p["t"] - self.start, 2)
                del q["t"]
                pts.append(q)
        return {
            "_help": "Real-trip video lesson (ACADEMY_SPEC F, step 20), made by the lesson editor from a saved event. "
                     "clip = UTC seconds; points t_s / blur from_s, to_s = seconds into the clip; areas / blur boxes "
                     "x, y, w, h = parts 0..1 of the picture; media = this folder's small trimmed copies (blurred). "
                     "Never in git (real trip video).",
            "kind": "videolesson", "id": self.id, "level": self.level, "runs_on": "recording",
            "title_el": self.title["el"] or self.id, "title_en": self.title["en"] or self.id,
            "intro_el": self.intro["el"] or self.ev.trigger_text or "-",
            "intro_en": self.intro["en"] or self.ev.trigger_text or "-",
            "source": {"event": self.ev.id, "trigger": self.ev.trigger_text},
            "clip": {"start_utc": round(s0, 3), "end_utc": round(s0 + L, 3)},
            "cameras": list(self.cameras), "main_camera": self.main,
            "media": {c: c + ".mp4" for c in self.cameras}, "timeline": "hud.jsonl", "width": self.width,
            "blur": blur, "points": pts}

    def problems(self):
        return check_doc(self.to_doc())

    def make(self, root=None, copy=None, env=None, progress=None):
        """Writes the lesson: the small copies (simguard.video_copy per camera), hud.jsonl from the event's log,
        lesson.json last (= the lesson exists only when everything is there). -> (folder, doc, notes)."""
        doc = self.to_doc()
        probs = [x for x in check_doc(doc) if "missing" not in x]
        if probs:
            raise ValueError("; ".join(probs))
        root = root or lessons_dir(env)
        folder = os.path.join(root, doc["id"])
        if not simguard._inside(folder, lessons_dir(env)):
            raise simguard.Refused("the lesson folder must be inside %s" % lessons_dir(env))
        os.makedirs(folder, exist_ok=True)
        copy = copy or simguard.video_copy
        s0, s1 = doc["clip"]["start_utc"], doc["clip"]["end_utc"]
        notes = []
        for k, cam in enumerate(doc["cameras"]):
            if progress:
                progress(k, len(doc["cameras"]), cam)
            segs, off = self.ev.segments(cam, s0, s1)
            if not segs:
                raise ValueError("camera %s has no video in the clip" % cam)
            copy(segs, off, s1 - s0, os.path.join(folder, doc["media"][cam]), self.width,
                 [b for b in doc["blur"] if b["camera"] == cam], env=env)
        tl = []
        lp = self.ev.log_path()
        if lp:
            try:
                tl = hud_timeline(read_log(lp, env), s0, s1)
            except (OSError, ValueError, simguard.Refused) as e:
                notes.append("log: %s" % e)
        if not tl:
            notes.append("no HUD: the log is missing or has no GPS time")
        with open(os.path.join(folder, "hud.jsonl"), "w", encoding="utf-8") as f:
            for e in tl:
                f.write(json.dumps(e, separators=(",", ":")) + "\n")
        doc["hud"] = bool(tl)
        tmp = os.path.join(folder, "lesson.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)
        os.replace(tmp, os.path.join(folder, "lesson.json"))
        return folder, doc, notes
