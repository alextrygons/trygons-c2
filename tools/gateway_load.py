"""CPU / RAM of the gateway with 4 camera streams + vision on the CPU (step 12).

    python tools/gateway_load.py                 (10 min, 1080p main + 360p sub streams)
    python tools/gateway_load.py --minutes 3 --viewers 0
    python tools/gateway_load.py --harbour --minutes 3   (step HX3: ONLY the harbour sensor workers - the simulated
                                                      radar + LiDAR processes driven by a boat circling in the
                                                      harbour; CPU per worker, the gateway-side manager, latency)
    python tools/gateway_load.py --uav 6 --uav-ai 2   (step U4: + the UAV magazine: 6 simulated VRX captures encoded
                                                      640x480 15 fps ~400 kbit/s into MediaMTX + the ring, the UAV AI
                                                      on 2 feeds - measured per part)

What runs (all on this PC, nothing on the network, no boat hardware):
  - the AURIX simulator (sim/autopilot_sim server 5770 1: USB 5770, gateway UART 5772)
  - 4 camera generators (ffmpeg, NOT measured): like a Tapo C310 each one sends a
    MAIN stream 1920x1080 15 fps H.264 ~2 Mbit/s and a SUB stream 640x360 15 fps
    ~0.4 Mbit/s; FRONT-1 = demo/media/sea.mp4 (if present) so the vision model sees sea
  - simulated AIS traffic (gateway/simais.py)
  - the gateway (MEASURED, with every child process): gateway.py + MediaMTX (relay)
    + 4 ffmpeg recorders (ring buffer, main streams, no re-encoding) + the vision
    worker (eWaSR ONNX on the CPU, its ffmpeg decoding the front SUB stream)
  - --viewers N: N stations watching all 4 main streams through MediaMTX (ffmpeg -c copy,
    NOT measured; MediaMTX's relay work is)
After a 60 s warm-up it samples every 5 s: CPU % of the whole PC and of one core
(100 % = one logical CPU busy), RAM (working set / RSS) per part, the whole PC's
CPU and RAM, and the vision frame rate from the gateway's STATUS line (--status).
Results: test_results/gateway_load/result.json + summary.txt (not in git).
No extra Python packages (ctypes on Windows, /proc on Ubuntu).
"""
import argparse
import ctypes
import json
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
OUT = os.path.join(REPO, "test_results", "gateway_load")
sys.path.insert(0, os.path.join(REPO, "gateway"))
import toolpaths  # noqa: E402

WIN = os.name == "nt"
NAMES = ["FRONT-1", "FRONT-2", "REAR-LEFT", "REAR-RIGHT"]
ROLES = ["front_main", "front_backup", "rear_left", "rear_right"]
P = {"usb": 5770, "uart": 5772, "stations": 5781, "rtsp": 8564, "api": 9987, "webrtc": 8899, "webrtc_udp": 8199,
     "web": 8090, "ais": 5795}


# --- process sampling ------------------------------------------------------------------
if WIN:
    from ctypes import wintypes

    class PE32(ctypes.Structure):
        _fields_ = [("dwSize", wintypes.DWORD), ("cntUsage", wintypes.DWORD), ("th32ProcessID", wintypes.DWORD),
                    ("th32DefaultHeapID", ctypes.c_size_t), ("th32ModuleID", wintypes.DWORD),
                    ("cntThreads", wintypes.DWORD), ("th32ParentProcessID", wintypes.DWORD),
                    ("pcPriClassBase", ctypes.c_long), ("dwFlags", wintypes.DWORD),
                    ("szExeFile", ctypes.c_wchar * 260)]

    class PMC(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                    ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]

    class MEMSTAT(ctypes.Structure):
        _fields_ = [("dwLength", wintypes.DWORD), ("dwMemoryLoad", wintypes.DWORD),
                    ("ullTotalPhys", ctypes.c_ulonglong), ("ullAvailPhys", ctypes.c_ulonglong),
                    ("ullTotalPageFile", ctypes.c_ulonglong), ("ullAvailPageFile", ctypes.c_ulonglong),
                    ("ullTotalVirtual", ctypes.c_ulonglong), ("ullAvailVirtual", ctypes.c_ulonglong),
                    ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]

    K32 = ctypes.WinDLL("kernel32", use_last_error=True)
    K32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
    K32.OpenProcess.restype = wintypes.HANDLE
    K32.Process32FirstW.argtypes = K32.Process32NextW.argtypes = [wintypes.HANDLE, ctypes.POINTER(PE32)]
    K32.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
    K32.K32GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(PMC), wintypes.DWORD]
    K32.CloseHandle.argtypes = [wintypes.HANDLE]


def processes():
    """{pid: (parent pid, exe name)}"""
    out = {}
    if WIN:
        snap = K32.CreateToolhelp32Snapshot(0x2, 0)
        e = PE32()
        e.dwSize = ctypes.sizeof(PE32)
        ok = K32.Process32FirstW(snap, ctypes.byref(e))
        while ok:
            out[e.th32ProcessID] = (e.th32ParentProcessID, e.szExeFile)
            ok = K32.Process32NextW(snap, ctypes.byref(e))
        K32.CloseHandle(snap)
    else:
        for d in os.listdir("/proc"):
            if d.isdigit():
                try:
                    with open("/proc/%s/stat" % d) as f:
                        s = f.read()
                    name = s[s.index("(") + 1:s.rindex(")")]
                    out[int(d)] = (int(s[s.rindex(")") + 2:].split()[1]), name)
                except (OSError, ValueError):
                    pass
    return out


def tree(root):
    procs = processes()
    keep, todo = {}, [root]
    while todo:
        p = todo.pop()
        if p in procs and p not in keep:
            keep[p] = procs[p][1]
            todo += [c for c, (pp, _) in procs.items() if pp == p]
    return keep


def proc_sample(pid):
    """(cpu seconds, ram bytes) or None"""
    if WIN:
        h = K32.OpenProcess(0x1000 | 0x0010, False, pid)     # QUERY_LIMITED_INFORMATION | VM_READ
        if not h:
            return None
        try:
            c, e, k, u = (wintypes.FILETIME() for _ in range(4))
            if not K32.GetProcessTimes(h, ctypes.byref(c), ctypes.byref(e), ctypes.byref(k), ctypes.byref(u)):
                return None
            t = ((k.dwHighDateTime << 32 | k.dwLowDateTime) + (u.dwHighDateTime << 32 | u.dwLowDateTime)) / 1e7
            m = PMC()
            m.cb = ctypes.sizeof(PMC)
            K32.K32GetProcessMemoryInfo(h, ctypes.byref(m), m.cb)
            return t, m.WorkingSetSize
        finally:
            K32.CloseHandle(h)
    try:
        with open("/proc/%d/stat" % pid) as f:
            v = f.read().rsplit(")", 1)[1].split()
        tck = os.sysconf("SC_CLK_TCK")
        return (int(v[11]) + int(v[12])) / tck, int(v[21]) * os.sysconf("SC_PAGE_SIZE")
    except (OSError, ValueError, IndexError):
        return None


def system_sample():
    """(busy seconds summed over CPUs, total seconds summed, RAM used bytes, RAM total bytes)"""
    if WIN:
        i, k, u = (ctypes.c_ulonglong() for _ in range(3))
        K32.GetSystemTimes(ctypes.byref(i), ctypes.byref(k), ctypes.byref(u))
        m = MEMSTAT()
        m.dwLength = ctypes.sizeof(MEMSTAT)
        K32.GlobalMemoryStatusEx(ctypes.byref(m))
        tot = (k.value + u.value) / 1e7
        return tot - i.value / 1e7, tot, m.ullTotalPhys - m.ullAvailPhys, m.ullTotalPhys
    with open("/proc/stat") as f:
        v = [int(x) for x in f.readline().split()[1:]]
    tck = os.sysconf("SC_CLK_TCK")
    mem = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, val = line.split(":")
            mem[k] = int(val.split()[0]) * 1024
    return (sum(v) - v[3] - v[4]) / tck, sum(v) / tck, mem["MemTotal"] - mem["MemAvailable"], mem["MemTotal"]


def part_of(name, pid, vision_pids, uav=None):
    n = name.lower()
    for k, label in (("video", "UAV video worker (+ its ffmpeg encoders)"), ("ai", "UAV AI worker (eWaSR + ffmpeg)"),
                     ("router", "UAV MAVLink router"), ("sim", "simulated drones (NOT the gateway)")):
        if uav and pid in uav.get(k, ()):
            return label
    if pid in vision_pids:
        return "vision worker (eWaSR + its ffmpeg)"
    if "mediamtx" in n:
        return "MediaMTX (relay)"
    if "ffmpeg" in n:
        return "recorders (4 x ffmpeg, copy)"
    return "gateway.py"


# --- the run -----------------------------------------------------------------------------
def port_busy(port):
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0


def wait_port(port, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if port_busy(port):
            return True
        time.sleep(0.3)
    return False


def camera(ffmpeg, i, src_video):
    main = "rtsp://127.0.0.1:%d/cam_%d_main" % (P["rtsp"], i)
    sub = "rtsp://127.0.0.1:%d/cam_%d_sub" % (P["rtsp"], i)
    if i == 0 and src_video:
        inp = ["-re", "-stream_loop", "-1", "-i", src_video]
    else:
        inp = ["-re", "-f", "lavfi", "-i", "testsrc2=size=1920x1080:rate=15"]
    enc = ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-g", "30", "-pix_fmt", "yuv420p"]
    args = ([ffmpeg, "-hide_banner", "-loglevel", "error"] + inp + ["-an"] +
            ["-vf", "fps=15,scale=1920:1080"] + enc + ["-b:v", "2000k", "-maxrate", "2000k", "-bufsize", "2000k",
                                                       "-f", "rtsp", "-rtsp_transport", "tcp", main] +
            ["-an", "-vf", "fps=15,scale=640:360"] + enc + ["-b:v", "400k", "-maxrate", "400k", "-bufsize", "400k",
                                                            "-f", "rtsp", "-rtsp_transport", "tcp", sub])
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def harbour_load(a):
    """HX3 + HX4: CPU of the two harbour sensor worker processes (simulated radar 0.8 Hz + LiDAR 10 Hz, the default harbour
    world), the gateway-side manager (this process: HarbourSensors.tick at 5 Hz + the reader threads) and the latency
    sensor time -> the gateway (p50 / p99 / max); HX4: the tracker's cycle ms (p50 / p99 / max), its latency
    (gateway clock - sensor time at the fusion) and the track counts (budget: cycle < 30 ms p99). The boat circles the basin (radius 40 m, 2 m/s)."""
    import math
    from harbour.manager import HarbourSensors
    from harbour import simworld

    class Gw:
        t0 = time.time()

        @property
        def telem_rx(self):
            return time.time()

        def own(self):
            t = time.time() - self.t0
            ang = t * 2.0 / 40.0
            lat, lon = simworld.xy_to_ll(37.82, 23.60, 40.0 * math.sin(ang), 40.0 * math.cos(ang) - 40.0)
            return {"fresh": True, "lat": lat, "lon": lon, "heading": (math.degrees(ang) + 90.0) % 360.0,
                    "sog": 3.9, "cog": (math.degrees(ang) + 90.0) % 360.0}
    m = HarbourSensors({"sim": True}, Gw())
    secs = max(10.0, a.minutes * 60.0)
    warm = min(a.warmup, 20.0)
    t_end = time.time() + warm + secs
    samples = {"radar": [], "lidar": [], "manager": []}
    base, p0, wall0 = None, None, None
    try:
        while time.time() < t_end:
            m.tick()
            time.sleep(0.2)
            now = time.time()
            if base is None and now >= t_end - secs:
                base = {k: proc_sample(p) for k, p in m.pids().items() if p}
                p0, wall0 = time.process_time(), now
                for sp in m.procs.values():
                    sp.lat.clear()
        cpu = {}
        wall = time.time() - wall0
        for k, p in m.pids().items():
            s1 = proc_sample(p) if p else None
            if s1 and base and base.get(k):
                cpu[k] = {"cpu_pct_one_core": round(100.0 * (s1[0] - base[k][0]) / wall, 2),
                          "rss_mb": round(s1[1] / 1e6, 1)}
        cpu["manager"] = {"cpu_pct_one_core": round(100.0 * (time.process_time() - p0) / wall, 2)}
        st = m.status()
        res = {"seconds": round(wall), "cpu": cpu,
               "latency": {k: v["latency"] for k, v in st["sensors"].items()},
               "items": {k: v["items"] for k, v in st["sensors"].items()},
               "rate_hz": {k: v["rate_hz"] for k, v in st["sensors"].items()},
               "state": {k: v["state"] for k, v in st["sensors"].items()}, "restarts": {k: v["restarts"]
                                                                                       for k, v in st["sensors"].items()},
               # HX4: the fusion tracker in the manager's tick (per batch feed + per tick cycle, ms), its tracks
               "tracker": {k: (st.get("tracker") or {}).get(k) for k in ("cycle_ms", "latency_ms", "tracks",
                                                                         "steer_ok", "confirmed", "unsure")}}
    finally:
        m.stop()
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "harbour.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    print("HARBOUR %s" % json.dumps(res))
    ok = all(v == "OK" for v in res["state"].values()) and not any(res["restarts"].values())
    p99 = ((res.get("tracker") or {}).get("cycle_ms") or {}).get("p99")
    ok = ok and p99 is not None and p99 < 30.0
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=60.0)
    ap.add_argument("--viewers", type=int, default=1)
    ap.add_argument("--no-vision", action="store_true")
    ap.add_argument("--uav", type=int, default=0, help="U4: N simulated UAV video captures (640x480 15 fps)")
    ap.add_argument("--uav-ai", type=int, default=0, help="U4: the UAV AI on this many of the UAV feeds")
    ap.add_argument("--uav-encoder", default="auto", help="auto (Quick Sync if it works) / x264")
    ap.add_argument("--harbour", action="store_true", help="HX3: the harbour sensor workers alone (simulated)")
    a = ap.parse_args()
    if a.harbour:
        return harbour_load(a)
    ffmpeg, mtx, vpy = toolpaths.ffmpeg(), toolpaths.mediamtx(), toolpaths.vision_python()
    model = os.path.join(REPO, "gateway", "vision", "models", "ewasr_resnet18.onnx")
    video = os.path.join(REPO, "demo", "media", "sea.mp4")
    sim = os.path.join(REPO, "sim", "autopilot_sim.exe" if WIN else "autopilot_sim")
    vision_ok = (not a.no_vision and os.path.exists(model) and vpy and
                 subprocess.run([vpy, "-c", "import onnxruntime, numpy"], capture_output=True).returncode == 0)
    miss = [n for n, v in (("ffmpeg", ffmpeg), ("MediaMTX", mtx), ("simulator", os.path.exists(sim) and sim)) if not v]
    if miss:
        print("missing: %s (tools/get_mediamtx.py, MSYS2 ffmpeg, tools/check.sh builds the simulator)" % miss)
        return 1
    busy = [p for p in P.values() if port_busy(p)]
    if busy:
        print("ports in use: %s - another run?" % busy)
        return 1
    os.makedirs(os.path.join(OUT, "data"), exist_ok=True)
    cfg = {
        "boat": "trygons-load", "aurix": "tcp:127.0.0.1:%d" % P["uart"], "set_clock": False,
        "stations": {"listen": "127.0.0.1", "port": P["stations"], "local_nets": ["127.0.0.1/32"]},
        "web_port": P["web"],
        "channels": {"driver": "sim", "starlink_status": False},
        "cameras": {"enabled": True, "mediamtx_bin": mtx, "mediamtx_conf": os.path.join(OUT, "mediamtx.yml"),
                    "mediamtx_log": os.path.join(OUT, "mediamtx.log"),
                    "rtsp_port": P["rtsp"], "api_port": P["api"], "webrtc_port": P["webrtc"],
                    "webrtc_udp_port": P["webrtc_udp"],
                    "list": [{"name": n, "role": r,
                              "url_main": "rtsp://127.0.0.1:%d/cam_%d_main" % (P["rtsp"], i),
                              "url_sub": "rtsp://127.0.0.1:%d/cam_%d_sub" % (P["rtsp"], i)}
                             for i, (n, r) in enumerate(zip(NAMES, ROLES))],
                    "extra_paths": ["cam_%d_%s" % (i, k) for i in range(4) for k in ("main", "sub")]},
        "recording": {"enabled": True, "ring_min": 5, "stall_s": 60.0},
        "sync": {"enabled": False},
        "ais": {"enabled": True, "source": "tcp:127.0.0.1:%d" % P["ais"]},
        "vision": {"enabled": True, "source": "camera" if vision_ok else "fake"},
        "print_status_s": 5.0,
        "uav": {"enabled": bool(a.uav), "sim": True, "sim_base_port": 14700,
                "video": {"enabled": bool(a.uav), "always": list(range(1, a.uav + 1)), "w": 640, "h": 480, "fps": 15,
                          "kbps": 400, "encoder": a.uav_encoder},
                "ai": {"measure_feeds": list(range(1, a.uav_ai + 1))}},
        "audit_fallback": os.path.join(OUT, "audit_fallback"),
        "storage": {"root": os.path.join(OUT, "data"), "require_mount": False, "check_s": 5, "low_free_gb": 0.0,
                    "full_free_gb": 0.0},
    }
    cpath = os.path.join(OUT, "gateway.json")
    with open(cpath, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=1)
    procs, logs = [], {}
    try:
        def start(name, args, cwd=None):
            lf = open(os.path.join(OUT, name + ".log"), "w", encoding="utf-8")
            logs[name] = lf
            p = subprocess.Popen(args, cwd=cwd, stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            procs.append(p)
            return p
        start("simulator", [sim, "server", str(P["usb"]), "1"], os.path.join(REPO, "sim"))
        if not wait_port(P["uart"], 15):
            print("the simulator did not start")
            return 1
        from simais import AisSim, traffic
        ais = AisSim(P["ais"], seed=3).start()
        for sh in traffic(37.82, 23.60, 15, seed=3, radius_nm=6.0):
            ais.add(sh)
        gw = start("gateway", [sys.executable, os.path.join(REPO, "gateway", "gateway.py"), "--config", cpath],
                   os.path.join(REPO, "gateway"))
        if not wait_port(P["rtsp"], 20):
            print("the gateway / MediaMTX did not start (test_results/gateway_load/gateway.log)")
            return 1
        cams = [camera(ffmpeg, i, video if os.path.exists(video) else None) for i in range(4)]
        procs += cams
        time.sleep(5.0)
        for v in range(a.viewers):
            for i, n in enumerate(NAMES):
                procs.append(subprocess.Popen(
                    [ffmpeg, "-hide_banner", "-loglevel", "error", "-rtsp_transport", "tcp", "-i",
                     "rtsp://127.0.0.1:%d/cam_%s_hi" % (P["rtsp"], n.lower().replace("-", "")),   # cameras.path_name
                     "-c", "copy", "-f", "null", "-"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        print("warm-up %.0f s (gateway pid %d, vision %s) ..." % (a.warmup, gw.pid, "eWaSR" if vision_ok else "fake"),
              flush=True)
        time.sleep(a.warmup)
        samples, prev, sprev = [], {}, system_sample()
        tprev = time.time()
        end = time.time() + a.minutes * 60.0
        ncpu = os.cpu_count() or 1
        while time.time() < end:
            time.sleep(5.0)
            now = time.time()
            members = tree(gw.pid)
            uav = uav_pids(os.path.join(OUT, "gateway.log"))
            vis = set()
            for pid, name in members.items():                     # the worker = a python child of gateway.py
                if pid != gw.pid and "python" in name.lower() and not any(pid in v for v in uav.values()):
                    vis |= set(tree(pid))
            parts, cur = {}, {}
            for pid, name in members.items():
                s = proc_sample(pid)
                if s is None:
                    continue
                cur[pid] = s
                k = part_of(name, pid, vis, uav)
                d = parts.setdefault(k, {"cpu_s": 0.0, "ram": 0, "n": 0})
                d["cpu_s"] += max(0.0, s[0] - prev.get(pid, s)[0]) if pid in prev else 0.0
                d["ram"] += s[1]
                d["n"] += 1
            ss = system_sample()
            dt = now - tprev
            if prev:
                samples.append({"t": now, "dt": dt, "parts": parts,
                                "sys_cpu_pct": 100.0 * (ss[0] - sprev[0]) / max(1e-6, ss[1] - sprev[1]),
                                "sys_ram": ss[2], "ram_total": ss[3]})
            prev, sprev, tprev = cur, ss, now
        st = open(os.path.join(OUT, "gateway.log"), encoding="utf-8", errors="replace").read()
        fps = [float(x) for x in re.findall(r"STATUS .* vision \w+ fps ([0-9.]+)", st)]
        infer = [float(x) for x in re.findall(r"STATUS .* infer_ms ([0-9.]+)", st)]
        cams_ok = re.findall(r"STATUS .* cameras (\d+/\d+ OK)", st)
        lag = [int(x) for x in re.findall(r"STATUS .* lag_ms (\d+)", st)]
        res = summarise(samples, ncpu, fps, infer, cams_ok, lag, vision_ok, a)
        if a.uav:
            u = re.findall(r"UAVSTATUS .*; video (\S+) encoder (\S+); uav_ai (\S+) ai_fps ([0-9.]+) ai_ms (\S+)", st)
            if u:
                vs, enc, ast, afps, ams = u[-1]
                res["lines"].append("UAV: %d simulated VRX captures, encoder %s, states %s; UAV AI %s on %d feed(s): "
                                    "%s fps (all feeds together), inference %s ms" % (a.uav, enc, vs, ast, a.uav_ai,
                                                                                       afps, ams))
            res["uav"] = {"n": a.uav, "ai_feeds": a.uav_ai, "status": u[-1] if u else None}
        with open(os.path.join(OUT, "result.json"), "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
        with open(os.path.join(OUT, "summary.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(res["lines"]) + "\n")
        print("\n".join(res["lines"]))
        return 0
    finally:
        for p in reversed(procs):
            if WIN:
                subprocess.run(["taskkill", "/PID", str(p.pid), "/T", "/F"], capture_output=True)
            else:
                p.kill()
        for lf in logs.values():
            lf.close()


def uav_pids(path):
    """{"router" | "sim" | "video" | "ai": {pid + its children}} from the gateway's newest UAVSTATUS line (U4)."""
    try:
        st = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        return {}
    m = re.findall(r"UAVSTATUS pids router=(\d+) sim=(\d+) video=(\d+) ai=(\d+)", st)
    if not m:
        return {}
    out = {}
    for k, v in zip(("router", "sim", "video", "ai"), m[-1]):
        out[k] = set(tree(int(v))) if int(v) else set()
    return out


def summarise(samples, ncpu, fps, infer, cams_ok, lag, vision_ok, a):
    import platform
    keys = sorted({k for s in samples for k in s["parts"]})
    rows = {}
    for k in keys:
        cpu = [100.0 * s["parts"][k]["cpu_s"] / s["dt"] for s in samples if k in s["parts"]]
        ram = [s["parts"][k]["ram"] / 1e6 for s in samples if k in s["parts"]]
        rows[k] = {"core_pct_avg": sum(cpu) / max(1, len(cpu)), "core_pct_max": max(cpu or [0]),
                   "ram_mb_avg": sum(ram) / max(1, len(ram)), "ram_mb_max": max(ram or [0])}
    tot_cpu = [sum(100.0 * p["cpu_s"] / s["dt"] for p in s["parts"].values()) for s in samples]
    tot_ram = [sum(p["ram"] for p in s["parts"].values()) / 1e6 for s in samples]
    cpu_name = platform.processor()
    if WIN:
        try:
            cpu_name = subprocess.run(["powershell", "-NoProfile", "-Command",
                                       "(Get-CimInstance Win32_Processor).Name"], capture_output=True, text=True,
                                      timeout=20).stdout.strip() or cpu_name
        except (OSError, subprocess.SubprocessError):
            pass
    avg = lambda v: sum(v) / max(1, len(v))                                         # noqa: E731
    lines = ["Gateway load: 4 cameras (1080p15 main ~2 Mbit/s + 360p15 sub ~0.4 Mbit/s each), recording all 4 "
             "main streams, %d viewer(s) of all 4 main streams, vision %s on the CPU" % (
                 a.viewers, "eWaSR ewasr_resnet18" if vision_ok else "FAKE (no model)"),
             "PC: %s, %d logical CPUs, %.1f GB RAM, %s %s; %d samples x 5 s after %.0f s warm-up" % (
                 cpu_name, ncpu, (samples[0]["ram_total"] if samples else 0) / 1e9, platform.system(),
                 platform.release(), len(samples), a.warmup),
             "%-38s %14s %14s %12s" % ("part", "CPU avg/max", "% of the PC", "RAM avg MB")]
    for k, r in rows.items():
        lines.append("%-38s %6.0f/%-6.0f%% %13.1f%% %12.0f" % (k, r["core_pct_avg"], r["core_pct_max"],
                                                              r["core_pct_avg"] / ncpu, r["ram_mb_avg"]))
    lines.append("%-38s %6.0f/%-6.0f%% %13.1f%% %12.0f  (max %.0f MB)" % (
        "GATEWAY TOTAL", avg(tot_cpu), max(tot_cpu or [0]), avg(tot_cpu) / ncpu, avg(tot_ram), max(tot_ram or [0])))
    lines.append("(CPU avg/max: 100 % = one logical CPU fully busy)")
    lines.append("whole PC during the run (everything: the gateway + the camera generators + the simulator + other programs): CPU %.0f %%, "
                 "RAM used %.1f GB" % (avg([s["sys_cpu_pct"] for s in samples]),
                                       avg([s["sys_ram"] for s in samples]) / 1e9))
    lines.append("vision: %.2f fps avg (min %.2f), inference %.0f ms avg; cameras %s; gateway loop lag max %d ms" % (
        avg(fps[-len(samples):] if samples else fps), min(fps or [0]), avg(infer), cams_ok[-1] if cams_ok else "?",
        max(lag or [0])))
    return {"rows": rows, "total_core_pct_avg": avg(tot_cpu), "total_core_pct_max": max(tot_cpu or [0]),
            "total_ram_mb_avg": avg(tot_ram), "total_ram_mb_max": max(tot_ram or [0]), "ncpu": ncpu,
            "cpu": cpu_name, "vision_fps": fps, "infer_ms": infer, "lines": lines, "samples": len(samples)}


if __name__ == "__main__":
    sys.exit(main())
