"""The interface for a FUTURE 3D simulator (ACADEMY_SPEC H, step 22) - no 3D world now.

docs/academy_sim_interface.md is the full description (for whoever builds the Godot world later).

Rules (CLAUDE.md section S, ACADEMY_SPEC ground rules + D2 / D3):
  * the 3D simulator NEVER replaces the simulated AURIX: every lesson / scenario / mission still runs
    against this repo's autopilot_sim on 127.0.0.1:5760 (simguard.SimLink), with its own unchanged
    safety code. The 3D world is a VIEW of that simulated boat (and a mission scorer) - nothing more;
  * the exchange is FILES ONLY in academy/data/sim3d/ (gitignored): no network, no process - the academy
    never starts Godot (D3 keeps the academy's processes to the simulator + ffmpeg), the trainee /
    instructor starts it; every file the academy reads goes through simguard.check_recording;
  * the academy never takes a COMMAND from the 3D simulator: it reads only its hello (present + version)
    and its mission RESULTS (score, status) - a result is logged, it never changes a level (levels go up
    only through the practical test, step 21);
  * DJI drone photos are imported as a picture / terrain for the 3D view ONLY (import_photos): copies
    without metadata in data/sim3d/terrain/<area>/, manifest "not_for_navigation": true - never a chart
    (the safety checks read only charts/; test_academy AC11 / AC48).

Backends (the "lesson simulator" choice, data/sim3d/settings.json):
  builtin  = autopilot_sim's own world, the academy's 2D pictures (default, always available)
  godot3d  = the same autopilot_sim + the 3D view fed by live.jsonl; usable only while the 3D simulator's
             hello.json is fresh with this INTERFACE_VERSION - else the academy says so and uses builtin.

Files in data/sim3d/:
  settings.json            academy   {"backend": "builtin" | "godot3d"}
  hello.json               3D sim    {"kind": "sim3d_hello", "version": 1, "t": epoch s, "name": ...} rewritten ~1 s
  live.jsonl               academy   header line {"kind": "sim3d_live", ...} then 5 Hz samples of the SIMULATED
                                     boat (written only while the godot3d backend is in use)
  missions/<attempt>.json  academy   {"kind": "sim3d_mission", "version", "attempt", "mission": <content doc>, ...}
  results/<attempt>.json   3D sim    {"kind": "sim3d_result", "version", "attempt", "mission", "status", "score", ...}
  results/done/            academy   results already logged (moved, never read twice)
  terrain/<area>/          academy   imported drone photos (no metadata) + manifest.json
"""
import json
import os
import re
import time

import content
import simguard

HERE = os.path.dirname(os.path.abspath(__file__))
INTERFACE_VERSION = 1
BACKENDS = ("builtin", "godot3d")
DEFAULT_BACKEND = "builtin"
HELLO_MAX_AGE_S = 5.0                    # the 3D sim rewrites hello.json about every second
LIVE_RATE_S = 0.2                        # 5 Hz, the scenario replay rate
RESULT_STATUS = ("passed", "failed", "aborted")
GOAL_TYPES = ("reach", "keep_clear", "heading", "time_limit", "mode")
TERRAIN_MAX_PX = 2048                    # imported photo copies: at most this wide / high
MODE_NAMES = ("STANDBY", "MANUAL", "HDG_LOCK", "AUTO", "TRAINING", "REMOTE_MANUAL", "BENCH_TEST", "MANEUVER",
              "DP")                      # = groundstation/proto.py MODES (AC45 checks it)
ATTEMPT_RE = re.compile(r"^[A-Za-z0-9_\-]{1,120}$")
MISSIONS_DIR = os.path.join(content.CONTENT_DIR, "missions")


# --- folders + backend choice ---------------------------------------------------------------------
def sim3d_dir(env=None):
    """academy/data/sim3d (gitignored; TRYGONS_ACADEMY_DATA overrides the data folder)."""
    env = os.environ if env is None else env
    return os.path.join(env.get("TRYGONS_ACADEMY_DATA") or os.path.join(HERE, "data"), "sim3d")


def _sub(name, env=None, make=True):
    d = os.path.join(sim3d_dir(env), name)
    if make:
        os.makedirs(d, exist_ok=True)
    return d


def _write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def _read_json(path, env=None):
    p = simguard.check_recording(path, env)          # an ordinary file only (never a device / address)
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def chosen_backend(env=None):
    """The backend the instructor / trainee chose (settings.json), builtin when none / unknown."""
    try:
        b = _read_json(os.path.join(sim3d_dir(env), "settings.json"), env).get("backend")
    except (simguard.Refused, OSError, ValueError, AttributeError):
        return DEFAULT_BACKEND
    return b if b in BACKENDS else DEFAULT_BACKEND


def select_backend(name, env=None):
    if simguard.running_on_gateway(env):
        raise simguard.Refused("academy never runs on a gateway PC")
    if name not in BACKENDS:
        raise ValueError("unknown simulator %r" % name)
    _write_json(os.path.join(sim3d_dir(env), "settings.json"), {"backend": name})
    return name


def hello(env=None, now=None):
    """(present, why): the 3D simulator's hello.json is fresh and speaks INTERFACE_VERSION."""
    now = time.time() if now is None else now
    p = os.path.join(sim3d_dir(env), "hello.json")
    if not os.path.isfile(p):
        return False, "3D simulator not running (no hello.json)"
    try:
        h = _read_json(p, env)
    except (simguard.Refused, OSError, ValueError) as e:
        return False, "3D simulator hello unreadable: %s" % e
    if not isinstance(h, dict) or h.get("kind") != "sim3d_hello":
        return False, "hello.json is not a 3D simulator hello"
    if h.get("version") != INTERFACE_VERSION:
        return False, "3D simulator speaks interface %r, the academy %d" % (h.get("version"), INTERFACE_VERSION)
    t = h.get("t")
    if not isinstance(t, (int, float)) or abs(now - t) > HELLO_MAX_AGE_S:
        return False, "3D simulator hello is stale"
    return True, ""


def active_backend(env=None, now=None):
    """(backend in use, why): godot3d only when chosen AND its hello is fresh; else builtin + the reason."""
    b = chosen_backend(env)
    if b == "godot3d":
        ok, why = hello(env, now)
        if not ok:
            return "builtin", why
    return b, ""


def lesson_target():
    """Whatever the backend: the lessons' AURIX = the simulator on this PC (host, port). Never changes."""
    return "127.0.0.1", simguard.LESSON_PORT


# --- the live view feed ---------------------------------------------------------------------------
LIVE_FIELDS = (("mode", "mode_name"), ("hdg", "heading"), ("sog", "est_sog"), ("cog", "est_cog"),
               ("thr", "throttle"), ("lat", "est_lat"), ("lon", "est_lon"), ("alm", "alarm_bits"),
               ("alm2", "alarm_bits2"), ("gear", "gear"), ("outboard", "outboard"), ("rot", "yaw_rate"))


class LiveFeed(object):
    """live.jsonl: a header, then the simulated boat at 5 Hz. Written ONLY by the academy (the 3D sim reads it)."""

    def __init__(self, source, item_id, env=None, now=None):
        if simguard.running_on_gateway(env):
            raise simguard.Refused("academy never runs on a gateway PC")
        self.path = os.path.join(sim3d_dir(env), "live.jsonl")
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.t0 = time.time() if now is None else now
        self.t_last = None
        self.n = 0
        with open(self.path, "w", encoding="utf-8") as f:        # a new run = a new file (the reader restarts)
            f.write(json.dumps({"kind": "sim3d_live", "version": INTERFACE_VERSION, "source": source,
                                "id": item_id, "t0": round(self.t0, 3)}) + "\n")

    def write(self, state, now=None):
        """One sample when LIVE_RATE_S has passed (returns True when written)."""
        now = time.time() if now is None else now
        if self.t_last is not None and now - self.t_last < LIVE_RATE_S - 1e-9:
            return False
        self.t_last = now
        s = {"t": round(now - self.t0, 2)}
        for k, f in LIVE_FIELDS:
            v = state.get(f)
            if isinstance(v, float):
                v = round(v, 7 if k in ("lat", "lon") else 2)
            if v is not None:
                s[k] = v
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(s) + "\n")
        self.n += 1
        return True


def feed_for(source, item_id, env=None, now=None):
    """A LiveFeed when the godot3d backend is in use, else None (the builtin backend writes nothing)."""
    b, _why = active_backend(env, now)
    return LiveFeed(source, item_id, env, now) if b == "godot3d" else None


def read_live(path, env=None):
    """(header, samples) of a live file - through simguard like every recording."""
    p = simguard.check_recording(path, env)
    with open(p, encoding="utf-8") as f:
        lines = [ln for ln in f.read().splitlines() if ln.strip()]
    head = json.loads(lines[0])
    if head.get("kind") != "sim3d_live":
        raise ValueError("not a 3D live feed")
    out = []
    for ln in lines[1:]:
        try:
            out.append(json.loads(ln))
        except ValueError:
            pass                                    # a line cut while written
    return head, out


# --- missions -------------------------------------------------------------------------------------
def missions(content_dir=None):
    d = os.path.join(content_dir, "missions") if content_dir else MISSIONS_DIR
    out = []
    if os.path.isdir(d):
        for f in sorted(os.listdir(d)):
            if f.endswith(".json"):
                out.append(content.load(os.path.join(d, f)))
    return sorted(out, key=lambda m: (content.LEVELS.index(m.get("level", "DRIVER"))
                                      if m.get("level") in content.LEVELS else 9, m.get("order", 99)))


def by_id(mission_id, content_dir=None):
    for m in missions(content_dir):
        if m.get("id") == mission_id:
            return m
    raise KeyError(mission_id)


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def check_mission(doc, name="?", lesson_ids=None):
    """Problems of a mission file (empty = OK), on top of content.check."""
    p = list(content.check(doc, name))
    if doc.get("kind") != "mission":
        p.append("%s: kind must be mission" % name)
    if not ATTEMPT_RE.match(str(doc.get("id", ""))):
        p.append("%s: id missing or not a plain name" % name)
    if doc.get("runs_on") != "sim":
        p.append("%s: runs_on must be \"sim\"" % name)
    for k in ("title_el", "brief_el"):
        if not doc.get(k):
            p.append("%s: %s missing" % (name, k))
    if not isinstance(doc.get("world"), str) or not doc["world"].strip():
        p.append("%s: world (the terrain area name) missing" % name)
    ps = doc.get("pass_score")
    if not _num(ps) or not 0 < ps <= 100:
        p.append("%s: pass_score must be 1..100" % name)
    ma = doc.get("max_attempts", 0)
    if not isinstance(ma, int) or isinstance(ma, bool) or ma < 0:
        p.append("%s: max_attempts must be >= 0 (0 = no limit)" % name)
    goals = doc.get("goals")
    if not isinstance(goals, list) or not goals:
        p.append("%s: no goals" % name)
        goals = []
    for i, g in enumerate(goals):
        t = g.get("type") if isinstance(g, dict) else None
        if t not in GOAL_TYPES:
            p.append("%s: goal %d: unknown type %r" % (name, i, t))
            continue
        if t in ("reach", "keep_clear"):
            if not (_num(g.get("n_m")) and _num(g.get("e_m")) and _num(g.get("radius_m")) and g["radius_m"] > 0):
                p.append("%s: goal %d: %s needs n_m, e_m, radius_m > 0" % (name, i, t))
        if t == "heading" and not (_num(g.get("deg")) and 0 <= g["deg"] < 360 and _num(g.get("tol_deg"))):
            p.append("%s: goal %d: heading needs deg 0..359 + tol_deg" % (name, i))
        if t == "time_limit" and not (_num(g.get("s")) and g["s"] > 0):
            p.append("%s: goal %d: time_limit needs s > 0" % (name, i))
        if t == "mode" and g.get("mode") not in MODE_NAMES:
            p.append("%s: goal %d: mode needs a mode name" % (name, i))
    les = doc.get("lesson")
    if les is not None and lesson_ids is not None and les not in lesson_ids:
        p.append("%s: unknown lesson %r" % (name, les))
    return p


def _entries(store, user, kind, mission_id):
    return [e for e in store.entries() if e.get("user") == user and e.get("kind") == kind
            and e.get("mission") == mission_id]


def attempts(store, user, mission_id):
    """Attempts started (a mission package written = an attempt, also when no result ever comes)."""
    return len(_entries(store, user, "mission_start", mission_id))


def best_score(store, user, mission_id):
    s = [e.get("score", 0) for e in _entries(store, user, "mission", mission_id)]
    return max(s) if s else None


def refused(store, user, doc, env=None):
    """Why this mission may NOT be started now (None = may). English (logs); the window translates."""
    if simguard.running_on_gateway(env):
        return "academy never runs on a gateway PC"
    if user is not None and store is not None:
        ma = doc.get("max_attempts", 0)
        if ma and attempts(store, user, doc["id"]) >= ma:
            return "no attempts left (%d)" % ma
    return None


def issue(doc, user=None, store=None, lang="el", env=None, now=None):
    """Write the mission package for the 3D simulator. Returns its path. Logs the attempt (kind mission_start)."""
    why = refused(store, user, doc, env)
    if why:
        raise simguard.Refused(why)
    now = time.time() if now is None else now
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    attempt = "%s_%s_%s_%03d" % (user or "practice", doc["id"], stamp, int(now * 1000) % 1000)
    if not ATTEMPT_RE.match(attempt):
        raise simguard.Refused("account name not usable in an attempt id")
    pkg = {"kind": "sim3d_mission", "version": INTERFACE_VERSION, "attempt": attempt, "user": user,
           "lang": lang, "issued": round(now, 3), "mission": doc,
           "aurix": {"host": lesson_target()[0], "port": lesson_target()[1],
                     "_note": "the simulated AURIX: the 3D sim only shows it, it never sends to it"},
           "terrain": os.path.join("terrain", doc.get("world", ""), "manifest.json").replace(os.sep, "/")}
    path = os.path.join(_sub("missions", env), attempt + ".json")
    _write_json(path, pkg)
    if store is not None and user:
        store.log(user, "mission_start", mission=doc["id"], attempt=attempt)
    return path


def check_result(res, issued_ids):
    """Problems of a 3D simulator result (empty = OK)."""
    p = []
    if not isinstance(res, dict):
        return ["not a JSON object"]
    if res.get("kind") != "sim3d_result":
        p.append("kind must be sim3d_result")
    if res.get("version") != INTERFACE_VERSION:
        p.append("interface version %r" % res.get("version"))
    a = res.get("attempt")
    if not isinstance(a, str) or a not in issued_ids:
        p.append("attempt %r was never issued by the academy" % a)
    elif res.get("mission") != issued_ids[a].get("mission", {}).get("id"):
        p.append("mission %r does not match the attempt" % res.get("mission"))
    if res.get("status") not in RESULT_STATUS:
        p.append("status %r" % res.get("status"))
    s = res.get("score")
    if not _num(s) or not 0 <= s <= 100:
        p.append("score must be 0..100")
    d = res.get("duration_s")
    if d is not None and (not _num(d) or d < 0):
        p.append("duration_s must be >= 0")
    for k in ("level", "licence", "role", "expiry", "command", "commands"):
        if k in res:
            p.append("%s: a 3D result never carries a level / role / command" % k)
    return p


def issued(env=None):
    """{attempt: package} of every mission package the academy wrote."""
    out = {}
    d = _sub("missions", env)
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            try:
                pkg = _read_json(os.path.join(d, f), env)
            except (simguard.Refused, OSError, ValueError):
                continue
            if isinstance(pkg, dict) and pkg.get("kind") == "sim3d_mission":
                out[pkg.get("attempt")] = pkg
    return out


def collect_results(store=None, env=None):
    """Read every new result in results/, log the valid ones (kind "mission"), move every one read to
    results/done/. Returns (logged entries, refused [(file, why)]). A result never changes a level."""
    if simguard.running_on_gateway(env):
        raise simguard.Refused("academy never runs on a gateway PC")
    rd = _sub("results", env)
    done = _sub(os.path.join("results", "done"), env)
    ids = issued(env)
    logged_attempts = {e.get("attempt") for e in (store.entries() if store is not None else [])
                       if e.get("kind") == "mission"}
    ok, bad = [], []
    for f in sorted(os.listdir(rd)):
        path = os.path.join(rd, f)
        if not f.endswith(".json") or not os.path.isfile(path):
            continue
        try:
            res = _read_json(path, env)
            probs = check_result(res, ids)
        except (simguard.Refused, OSError, ValueError) as e:
            probs, res = [str(e)], None
        if not probs and res["attempt"] in logged_attempts:
            probs = ["attempt already logged"]
        if probs:
            bad.append((f, "; ".join(probs)))
        else:
            pkg = ids[res["attempt"]]
            m = pkg["mission"]
            passed = res["status"] == "passed" and res["score"] >= m.get("pass_score", 100)
            e = {"mission": m["id"], "attempt": res["attempt"], "score": int(round(res["score"])),
                 "status": res["status"], "passed": passed, "duration_s": res.get("duration_s"),
                 "topic": m["id"], "done": passed}
            user = pkg.get("user")
            if store is not None and user:
                e = store.log(user, "mission", **e)
                logged_attempts.add(res["attempt"])
            ok.append(e)
        try:
            os.replace(path, os.path.join(done, f))
        except OSError:
            pass
    return ok, bad


# --- DJI photos -> terrain for the 3D view (NEVER a chart) -----------------------------------------
def _ratio(v):
    try:
        return float(v[0]) / float(v[1]) if isinstance(v, tuple) else float(v)
    except (TypeError, ValueError, ZeroDivisionError):
        return None


def _dms(vals, ref):
    try:
        d, m, s = (_ratio(x) for x in vals)
        v = d + m / 60.0 + s / 3600.0
    except (TypeError, ValueError):
        return None
    return -v if ref in ("S", "W", b"S", b"W") else v


XMP_KEYS = {"rel_alt_m": "RelativeAltitude", "yaw_deg": "FlightYawDegree", "gimbal_pitch_deg": "GimbalPitchDegree",
            "gimbal_yaw_deg": "GimbalYawDegree"}


def photo_info(path, env=None):
    """{lat, lon, alt_m, rel_alt_m, yaw_deg, gimbal_pitch_deg, ...} of a drone photo (EXIF GPS + DJI XMP).
    Raises ValueError when the photo has no GPS position."""
    from PIL import Image
    p = simguard.check_image(path, env)
    with open(p, "rb") as f:
        raw = f.read()
    info = {}
    with Image.open(p) as im:
        info["w"], info["h"] = im.size
        gps = {}
        try:
            gps = im.getexif().get_ifd(0x8825) or {}
        except Exception:                              # no / broken EXIF
            gps = {}
    lat = _dms(gps.get(2), gps.get(1)) if gps.get(2) else None
    lon = _dms(gps.get(4), gps.get(3)) if gps.get(4) else None
    if lat is None or lon is None or not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("no GPS position in the photo")
    info["lat"], info["lon"] = round(lat, 7), round(lon, 7)
    if gps.get(6) is not None:
        alt = _ratio(gps.get(6))
        if alt is not None:
            info["alt_m"] = round(-alt if gps.get(5) in (1, b"\x01") else alt, 1)
    for k, tag in XMP_KEYS.items():                    # DJI writes drone-dji:<tag>="+12.30"
        m = re.search((r'drone-dji:%s="([+\-]?[0-9.]+)"' % tag).encode(), raw)
        if m:
            info[k] = round(float(m.group(1)), 2)
    return info


def terrain_dir(area, env=None):
    if not ATTEMPT_RE.match(area or ""):
        raise simguard.Refused("area name must be a plain name")
    return os.path.join(sim3d_dir(env), "terrain", area)


def import_photos(paths, area, env=None, now=None):
    """Import drone photos as the 3D view's picture / terrain of an area: a copy WITHOUT metadata (<= 2048 px)
    + manifest.json (position, height, yaw of each photo; not_for_navigation). Photos without GPS are skipped.
    Returns (manifest, skipped [(file, why)]). Never writes outside data/sim3d/terrain/<area>/."""
    from PIL import Image
    if simguard.running_on_gateway(env):
        raise simguard.Refused("academy never runs on a gateway PC")
    out = terrain_dir(area, env)
    os.makedirs(out, exist_ok=True)
    mpath = os.path.join(out, "manifest.json")
    manifest = {"kind": "sim3d_terrain", "version": INTERFACE_VERSION, "area": area, "not_for_navigation": True,
                "_note": "drone photos for the 3D TRAINING view only - NOT a navigation chart, never used by a "
                         "safety check (shallow water, route, geofence, collision use only charts/)",
                "photos": []}
    if os.path.isfile(mpath):
        try:
            old = _read_json(mpath, env)
            if isinstance(old, dict) and isinstance(old.get("photos"), list):
                manifest["photos"] = old["photos"]
        except (simguard.Refused, OSError, ValueError):
            pass
    skipped = []
    have = {ph.get("file") for ph in manifest["photos"]}
    for src in paths:
        try:
            info = photo_info(src, env)
        except (simguard.Refused, OSError, ValueError) as e:
            skipped.append((os.path.basename(str(src)), str(e)))
            continue
        base = re.sub(r"[^A-Za-z0-9_\-]", "_", os.path.splitext(os.path.basename(src))[0])[:60] or "photo"
        name = base + ".jpg"
        n = 1
        while name in have:
            n += 1
            name = "%s_%d.jpg" % (base, n)
        with Image.open(simguard.check_image(src, env)) as im:
            im = im.convert("RGB")
            im.info = {}                                           # no comment / XMP / ICC carried over
            im.thumbnail((TERRAIN_MAX_PX, TERRAIN_MAX_PX))
            info["w"], info["h"] = im.size
            im.save(os.path.join(out, name), "JPEG", quality=88)   # no exif= -> no metadata in the copy
        info["file"] = name
        manifest["photos"].append(info)
        have.add(name)
    manifest["updated"] = round(time.time() if now is None else now, 3)
    _write_json(mpath, manifest)
    return manifest, skipped
