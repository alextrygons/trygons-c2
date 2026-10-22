"""Cameras (section E): 4 x TP-Link Tapo C310 on the camera network.

  FRONT-1 (main), FRONT-2 (backup), REAR-LEFT, REAR-RIGHT
  main stream  rtsp://USER:PASS@IP:554/stream1  (3 MP, HIGH)
  sub stream   rtsp://USER:PASS@IP:554/stream2  (LOW)
Logins in secrets.json ("cameras": {"FRONT-1": {"user", "password"}}) - never in git.

Relay: MediaMTX (one program, ARM64 builds, RTSP in -> RTSP / WebRTC / HLS
out, NO transcoding; H.264 from the camera goes through as it is). The
gateway writes its config and keeps it running. Paths:
  cam_<name>_hi   main stream, pulled all the time (recording + health, LAN only)
  cam_<name>_lo   sub stream, pulled only while somebody watches it
Stations read them from MediaMTX (rtsp://<gateway>:8554/<path>, or WebRTC
on the iPhone) with the user / password the gateway gave them at login.
MediaMTX asks the gateway for every reader (HTTP auth, /mtxauth on
127.0.0.1): a station may only read what its camera preset allows - and
on STARLINK / data saver only the front camera LOW (no override on
STARLINK). So the gateway forwards only what a station asked for, and an OFF
stream uses no uplink at all.

Health: every 0.25 s the gateway reads the MediaMTX API: a camera whose
main stream received no new data for 1 s is FROZEN / DOWN. All health checks
run on the boat LAN, no internet used. Cameras are never needed for
safety: a camera problem never changes the mode (it is only shown).
"""
import json
import os
import subprocess
import threading
import time
import urllib.parse
import urllib.request

from common import log

ROLES = ("front_main", "front_backup", "rear_left", "rear_right")
DEFAULT_CAMERAS = [
    {"name": "FRONT-1", "role": "front_main", "ip": "10.43.0.11"},
    {"name": "FRONT-2", "role": "front_backup", "ip": "10.43.1.12"},
    {"name": "REAR-LEFT", "role": "rear_left", "ip": "10.43.0.13"},
    {"name": "REAR-RIGHT", "role": "rear_right", "ip": "10.43.0.14"},
]
# presets: camera role -> OFF / LOW / HIGH
PRESETS = {
    "Full":         {"front": "HIGH", "rear_left": "LOW", "rear_right": "LOW"},
    "Front only":   {"front": "LOW", "rear_left": "OFF", "rear_right": "OFF"},
    "Mirrors only": {"front": "OFF", "rear_left": "LOW", "rear_right": "LOW"},
    "All off":      {"front": "OFF", "rear_left": "OFF", "rear_right": "OFF"},
}


def path_name(cam, quality):
    return "cam_%s_%s" % (cam.lower().replace("-", ""), "hi" if quality == "HIGH" else "lo")


class Camera:
    def __init__(self, c, secrets):
        self.name = c["name"]
        self.role = c.get("role", "rear_left")
        self.ip = c.get("ip")
        login = (secrets or {}).get(self.name, {})
        user, pw = login.get("user", ""), login.get("password", "")
        auth = "%s:%s@" % (urllib.parse.quote(user, safe=""), urllib.parse.quote(pw, safe="")) if user else ""
        self.url_main = c.get("url_main") or "rtsp://%s%s:554/stream1" % (auth, self.ip)
        self.url_sub = c.get("url_sub") or "rtsp://%s%s:554/stream2" % (auth, self.ip)
        self.bytes_in = 0
        self.last_change = 0.0              # last time new bytes arrived (main stream)
        self.ready = False
        self.state = "unknown"              # OK / FROZEN / DOWN
        self.since = time.time()
        self.kbps_in = 0.0
        self.answered_once = False


class CameraManager:
    def __init__(self, cfg, gw=None):
        self.cfg = cfg
        self.gw = gw
        sec = (gw.cfg.get("secrets", {}) if gw else {}).get("cameras", {})
        self.cams = {c["name"]: Camera(c, sec) for c in (cfg.get("list") or DEFAULT_CAMERAS)}
        self.api = "http://127.0.0.1:%d" % cfg.get("api_port", 9997)
        self.rtsp_port = cfg.get("rtsp_port", 8554)
        self.webrtc_port = cfg.get("webrtc_port", 8889)
        self.frozen_s = cfg.get("frozen_s", 1.0)
        self.proc = None
        self.station_cams = {}              # station id -> {"preset", "levels", "override"}
        self.sessions = {}                  # MediaMTX reader id -> (path, station id, bytes, t)
        self.station_kbps = {}              # station id -> kbit/s of video it receives
        self.path_kbps = {}                 # path -> kbit/s sent to stations
        self.total_kbps = 0.0
        self.internet_until = 0.0           # camera internet allowed (firmware update) until
        self.mtx_ok = False
        self.last_poll = 0.0
        self.lock = threading.Lock()

    # --- MediaMTX ---------------------------------------------------------------------
    def mediamtx_yaml(self, auth_url):
        lines = [
            "# written by the Trygons gateway (cameras.py) - edits are overwritten",
            "logLevel: %s" % self.cfg.get("log_level", "warn"),
            "readTimeout: 5s",
            "api: yes",
            "apiAddress: 127.0.0.1:%d" % self.cfg.get("api_port", 9997),
            "rtsp: yes",
            "rtspTransports: [tcp]",
            "rtspAddress: :%d" % self.rtsp_port,
            "rtmp: no",
            "hls: no",
            "srt: no",
            "moq: no",
            "webrtc: yes",
            "webrtcAddress: :%d" % self.webrtc_port,
            "webrtcLocalUDPAddress: :%d" % self.cfg.get("webrtc_udp_port", 8189),
            "authMethod: http",
            "authHTTPAddress: %s" % auth_url,
            "authHTTPExclude:",
            "  - action: api",
            "  - action: metrics",
            "pathDefaults:",
            "  rtspTransport: tcp",
            "  sourceOnDemandStartTimeout: 10s",
            "  sourceOnDemandCloseAfter: 5s",
            "paths:",
        ]
        for c in self.cams.values():
            lines += ["  %s:" % path_name(c.name, "HIGH"), "    source: %s" % c.url_main, "    sourceOnDemand: no",
                      "  %s:" % path_name(c.name, "LOW"), "    source: %s" % c.url_sub, "    sourceOnDemand: yes"]
        for extra in self.cfg.get("extra_paths", []):       # tests: the simulated cameras publish here
            lines += ["  %s:" % extra, "    source: publisher"]
        return "\n".join(lines) + "\n"

    def start(self, auth_url):
        import toolpaths
        binary = self.cfg.get("mediamtx_bin") or toolpaths.mediamtx() or "/usr/local/bin/mediamtx"
        conf = self.cfg.get("mediamtx_conf") or "/var/lib/trygons/mediamtx.yml"
        os.makedirs(os.path.dirname(conf), exist_ok=True)
        with open(conf, "w", encoding="utf-8") as f:
            f.write(self.mediamtx_yaml(auth_url))
        if not os.path.exists(binary):
            log.warning("cameras: MediaMTX not found (%s) - no video relay", binary)
            return
        self._bin, self._conf = binary, conf
        self._spawn()

    def _spawn(self):
        out = open(self.cfg["mediamtx_log"], "a") if self.cfg.get("mediamtx_log") else subprocess.DEVNULL
        self.proc = subprocess.Popen([self._bin, self._conf], stdout=out, stderr=subprocess.STDOUT)
        log.info("cameras: MediaMTX started (pid %d)", self.proc.pid)

    def stop(self):
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None

    def _get(self, path):
        with urllib.request.urlopen(self.api + path, timeout=1.0) as r:
            return json.loads(r.read())

    def _post(self, path):
        req = urllib.request.Request(self.api + path, data=b"", method="POST")
        with urllib.request.urlopen(req, timeout=1.0) as r:
            return r.status

    # --- health + bitrates (every 0.25 s, in a thread: urllib blocks) ------------------
    def poll(self):
        now = time.time()
        if self.proc and self.proc.poll() is not None:
            log.warning("cameras: MediaMTX stopped - restarting")
            self._spawn()
        try:
            paths = {p["name"]: p for p in self._get("/v3/paths/list?itemsPerPage=1000").get("items", [])}
            sess = self._get("/v3/rtspsessions/list?itemsPerPage=1000").get("items", [])
            self.mtx_ok = True
        except (OSError, ValueError):
            self.mtx_ok = False
            paths, sess = {}, []
        with self.lock:
            for c in self.cams.values():
                p = paths.get(path_name(c.name, "HIGH"))
                b = int(p.get("bytesReceived", 0)) if p else 0
                c.ready = bool(p and p.get("ready"))
                if b != c.bytes_in:
                    if c.last_change:
                        c.kbps_in = 0.8 * c.kbps_in + 0.2 * (b - c.bytes_in) * 8 / 1000 / max(0.05, now - self.last_poll)
                    c.bytes_in = b
                    c.last_change = now
                    c.answered_once = True
                state = "OK" if c.ready and now - c.last_change <= self.frozen_s else (
                    "FROZEN" if c.ready else "DOWN")
                if state != c.state:
                    log.warning("camera %s: %s -> %s", c.name, c.state, state) if state != "OK" else \
                        log.info("camera %s: OK", c.name)
                    if self.gw:
                        self.gw.audit("camera", name=c.name, state=state)
                    c.state, c.since = state, now
            # what goes to the stations: their readers carry "?station=<id>"
            # (the gateway's own readers - recording, health - carry none)
            kb, kst = {}, {}
            seen = set()
            for s in sess:
                if s.get("state") != "read":
                    continue
                q = s.get("query", "") or ""
                if not q.startswith("station="):
                    continue
                try:
                    st_id = int(q.split("=", 1)[1].split("&")[0])
                except ValueError:
                    continue
                sid = s.get("id")
                seen.add(sid)
                b = int(s.get("bytesSent", 0))
                old = self.sessions.get(sid)
                if old:
                    rate = (b - old[2]) * 8 / 1000 / max(0.05, now - old[3])
                    kb[s.get("path")] = kb.get(s.get("path"), 0.0) + rate
                    kst[st_id] = kst.get(st_id, 0.0) + rate
                self.sessions[sid] = (s.get("path"), st_id, b, now)
            for sid in list(self.sessions):
                if sid not in seen:
                    del self.sessions[sid]
            self.path_kbps = {k: round(v) for k, v in kb.items()}
            self.station_kbps = {k: round(v) for k, v in kst.items()}
            self.total_kbps = round(sum(kb.values()))
            self.last_poll = now

    # --- presets / permissions ----------------------------------------------------------
    def forced_low(self):
        """data saver / slow link: front only, LOW."""
        return bool(self.gw and self.gw.data_saver())

    def levels_for(self, st_id):
        """What a station may receive now: camera name -> OFF / LOW / HIGH."""
        req = self.station_cams.get(st_id, {"preset": "Front only", "levels": None, "override": False})
        lv = self.levels_from_preset(req["preset"]) if req.get("levels") is None else dict(req["levels"])
        if self.forced_low() and not (req.get("override") and not (self.gw and self.gw.net.on_emergency)):
            lv = {n: ("LOW" if self.cams[n].role.startswith("front") and lv.get(n) != "OFF" else "OFF") for n in lv}
            front = [n for n in lv if self.cams[n].role.startswith("front")]
            if not any(lv[n] == "LOW" for n in front):
                for n in front:
                    lv[n] = "LOW"
        return lv

    def levels_from_preset(self, preset):
        pr = PRESETS.get(preset, PRESETS["Front only"])
        out = {}
        for n, c in self.cams.items():
            key = "front" if c.role.startswith("front") else c.role
            out[n] = pr.get(key, "OFF")
        return out

    def allowed_paths(self, st_id):
        return {path_name(n, q) for n, q in self.levels_for(st_id).items() if q != "OFF"}

    def request(self, st, m):
        """{"t":"cams", "preset": "Full" | ..., "levels": {name: OFF/LOW/HIGH} (optional),
        "override": bool} -> the streams this station may open."""
        preset = m.get("preset", "Front only")
        levels = m.get("levels")
        if levels is not None and not all(v in ("OFF", "LOW", "HIGH") and k in self.cams for k, v in levels.items()):
            return st.send({"t": "cams_ack", "ok": False, "text": "bad camera levels"})
        self.station_cams[st.id] = {"preset": preset, "levels": levels, "override": bool(m.get("override"))}
        self.kick_disallowed()
        if self.gw:
            self.gw.audit("cameras", station=st.id, user=st.user, preset=preset, levels=levels,
                          override=bool(m.get("override")))
        st.send(self.streams_msg(st))

    def streams_msg(self, st):
        lv = self.levels_for(st.id)
        notice = ""
        if self.forced_low():
            notice = "DATA SAVER: front camera only, LOW" + (" (STARLINK: no override)" if self.gw.net.on_emergency
                                                            else "")
        return {"t": "cams_ack", "ok": True, "levels": lv, "notice": notice,
                "streams": {n: path_name(n, q) for n, q in lv.items() if q != "OFF"},
                "rtsp_port": self.rtsp_port, "webrtc_port": self.webrtc_port, "query": "station=%d" % st.id,
                "user": st.session, "password": st.video_key}

    def auth(self, req):
        """MediaMTX asks: may this reader read / publish this path?"""
        action, path, ip = req.get("action"), req.get("path", ""), req.get("ip", "")
        if ip in ("127.0.0.1", "::1") and not req.get("user") and not req.get("query"):
            return True                     # the gateway itself (recording, health, relaying) / test cameras
        if action != "read" or not self.gw:
            return False
        st = next((s for s in self.gw.stations.stations.values() if s.session == req.get("user")), None)
        if st is None or not hmac_eq(getattr(st, "video_key", ""), req.get("password", "")):
            return False
        if req.get("query", "") != "station=%d" % st.id:
            return False                    # its own station id, so its use is counted
        return path in self.allowed_paths(st.id)

    def kick_disallowed(self):
        """Close readers that may not read any more (preset changed, data saver)."""
        with self.lock:
            sessions = list(self.sessions.items())
        for sid, (path, st_id, _, _) in sessions:
            if path not in self.allowed_paths(st_id):
                try:
                    self._post("/v3/rtspsessions/kick/%s" % sid)
                    log.info("cameras: closed %s (station %s): not allowed any more", path, st_id)
                except OSError:
                    pass

    # --- camera internet (firmware updates only) -------------------------------------------
    def internet(self, on, minutes=30):
        """Cameras normally have NO internet. For a firmware update it can be
        allowed for a while (then it closes by itself)."""
        self.internet_until = time.time() + 60 * minutes if on else 0.0
        if self.gw:
            self.gw.audit("camera_internet", on=on, minutes=minutes if on else 0)
        cmd = self.cfg.get("internet_cmd")          # e.g. ["sudo", "/home/trygons/trygons-c2/gateway/tools/camera_internet.sh"]
        if cmd:
            subprocess.run(cmd + ["on" if on else "off"], check=False)

    def internet_allowed(self):
        if self.internet_until and time.time() > self.internet_until:
            self.internet(False)
        return time.time() < self.internet_until

    def status(self):
        with self.lock:
            cams = [{"name": c.name, "role": c.role, "state": c.state, "for_s": round(time.time() - c.since),
                     "kbps_in": round(c.kbps_in),
                     "kbps_out": {q: self.path_kbps.get(path_name(c.name, q), 0) for q in ("HIGH", "LOW")}}
                    for c in self.cams.values()]
        front_live = any(c["state"] == "OK" for c in cams if c["role"].startswith("front"))
        return {"relay": self.mtx_ok, "cameras": cams, "total_kbps": self.total_kbps, "front_live": front_live,
                "station_kbps": self.station_kbps,
                "forced_low": self.forced_low(), "internet": self.internet_allowed()}


def hmac_eq(a, b):
    import hmac
    return bool(a) and hmac.compare_digest(str(a), str(b))
