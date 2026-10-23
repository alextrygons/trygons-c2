"""Part 4.3: system soak in REAL time - simulator + gateway (cameras through
MediaMTX, recording, vision, AIS, sync to a shore receiver, channel changes)
with continuous mixed activity. The 24 h of simulated AURIX time run in the
C soak (storm.exe --soak 24 SEED --log); the gateway runs in real time, so
here it runs for --minutes and the trends are extrapolated.

    python tests/acceptance/soak_system.py [--minutes 10] [--seed 1] [--sample 20]

Measured every --sample s: memory (working set, private bytes), handles, TCP
sockets, CPU of every process (gateway, simulator, MediaMTX, vision worker,
ffmpeg recorders / cameras); storage, ring files and deletions, sync backlog,
the AURIX control timing / overruns / log drops, vision fps and pauses.
-> test_results/soak_samples.csv, test_results/soak_report.txt
"""
import argparse
import ctypes
import csv
import os
import random
import subprocess
import sys
import tempfile
import shutil
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "test_results")
sys.path.insert(0, os.path.join(REPO, "gateway"))
os.chdir(os.path.join(REPO, "gateway"))
import test_gateway as TG  # noqa: E402
from common import proto  # noqa: E402
from simais import AisSim, traffic  # noqa: E402
from simcam import SimCamera  # noqa: E402
from syncrecv import SyncReceiver  # noqa: E402

if os.name == "nt":
    from ctypes import wintypes
    K = ctypes.WinDLL("kernel32", use_last_error=True)
    P = ctypes.WinDLL("psapi", use_last_error=True)

    class PMC(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                    ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]
    K.OpenProcess.restype = wintypes.HANDLE
    K.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    K.CloseHandle.argtypes = [wintypes.HANDLE]
    K.GetProcessHandleCount.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    K.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
    P.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(PMC), wintypes.DWORD]


def proc_info(pid):
    """(working set MB, private MB, handles, CPU seconds) or None."""
    if not pid or os.name != "nt":
        return None
    h = K.OpenProcess(0x1000 | 0x0010, False, pid)           # QUERY_LIMITED_INFORMATION | VM_READ
    if not h:
        return None
    try:
        m = PMC()
        m.cb = ctypes.sizeof(PMC)
        P.GetProcessMemoryInfo(h, ctypes.byref(m), m.cb)
        hc = wintypes.DWORD()
        K.GetProcessHandleCount(h, ctypes.byref(hc))
        t = [wintypes.FILETIME() for _ in range(4)]
        K.GetProcessTimes(h, *[ctypes.byref(x) for x in t])
        cpu = ((t[2].dwHighDateTime << 32 | t[2].dwLowDateTime) + (t[3].dwHighDateTime << 32 | t[3].dwLowDateTime)) / 1e7
        return m.WorkingSetSize / 1e6, m.PagefileUsage / 1e6, hc.value, cpu
    finally:
        K.CloseHandle(h)


def sockets_by_pid():
    out = {}
    try:
        txt = subprocess.run(["netstat", "-ano", "-p", "TCP"], capture_output=True, text=True, timeout=20).stdout
    except (OSError, subprocess.SubprocessError):
        return out
    for line in txt.splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[0] == "TCP" and parts[-1].isdigit():
            out[int(parts[-1])] = out.get(int(parts[-1]), 0) + 1
    return out


def du(root):
    n, b = 0, 0
    for d, _s, fs in os.walk(root):
        for f in fs:
            try:
                b += os.path.getsize(os.path.join(d, f))
                n += 1
            except OSError:
                pass
    return n, b


def slope(xs, ys):
    """Least-squares slope per hour and R^2."""
    n = len(xs)
    if n < 3:
        return 0.0, 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return 0.0, 0.0
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx
    ss = sum((y - my) ** 2 for y in ys)
    r2 = (sxy * sxy) / (sxx * ss) if ss else 0.0
    return b * 3600.0, r2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sample", type=float, default=20.0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    TG.setup_logging(40)
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix="acc_soak_")
    sim = gw = asim = recv = None
    cams = []
    rows = []
    try:
        sim = TG.Sim(1.0)
        root = os.path.join(tmp, "ssd")
        shore = os.path.join(tmp, "shore")
        os.makedirs(root)
        recv = SyncReceiver(shore, {"trygons-c2": TG.SYNC_KEY}, port=TG.free_port(), listen="127.0.0.1").start()
        rtsp, api, web, wu, aport = (TG.free_port() for _ in range(5))
        names, roles = TG.CAM_NAMES, TG.CAM_ROLES
        ccfg = {"enabled": True, "mediamtx_bin": TG.MTX_BIN, "mediamtx_conf": os.path.join(tmp, "mtx.yml"),
                "rtsp_port": rtsp, "api_port": api, "webrtc_port": web, "webrtc_udp_port": wu,
                "list": [{"name": n, "role": r, "url_main": "rtsp://127.0.0.1:%d/simcam_%d" % (rtsp, i),
                          "url_sub": "rtsp://127.0.0.1:%d/simcam_%d" % (rtsp, i)} for i, (n, r) in enumerate(zip(names, roles))],
                "extra_paths": ["simcam_%d" % i for i in range(4)], "mediamtx_log": os.path.join(tmp, "mtx.log")}
        model = os.path.join(REPO, "gateway", "vision", "models", "ewasr_resnet18.onnx")
        vis = {"enabled": True, "source": "camera" if os.path.exists(model) else "fake", "allow_inject": True,
               "stage2": False, "samples": {"enabled": True, "max_mb": 20.0, "random_per_hour": 60.0}}
        cfg = TG.gw_cfg_b(sim, root, cameras=ccfg, vision=vis,
                          recording={"enabled": True, "ring_min": 5, "segment_s": 30, "stall_s": 60.0},
                          ais={"enabled": True, "source": "tcp:127.0.0.1:%d" % aport, "silent_s": 600.0},
                          sync={"enabled": True, "shore": "127.0.0.1:%d" % recv.port, "settle_s": 2.0, "scan_s": 5.0},
                          channels={"driver": "sim", "check_s": 1.0, "stable_s": 20.0, "starlink_status": False})
        cfg["secrets"] = {"sync_key": TG.SYNC_KEY}
        asim = AisSim(aport, seed=a.seed, drop=0.05).start()
        for sh in traffic(TG.SIM_LAT, TG.SIM_LON, 12, seed=a.seed):
            asim.add(sh)
        gw = TG.GwRunner(cfg)
        g = gw.gw
        time.sleep(1.5)
        cams = [SimCamera("rtsp://127.0.0.1:%d/simcam_%d" % (rtsp, i)).start() for i in range(4)]
        link, w = sim.station()
        t0 = time.time()
        end = t0 + a.minutes * 60.0
        next_sample = t0
        saves = channel_changes = mode_changes = 0
        while time.time() < end:
            # activity: modes, joystick, SAVEs, channel changes, vision obstacles
            p = rng.random()
            if p < 0.15:
                link.send(proto.T_SIM_BUTTON, proto.p_sim_button(rng.choice((proto.BTN_HDG_LOCK, proto.BTN_STANDBY))))
                mode_changes += 1
            elif p < 0.30:
                link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, rng.uniform(-0.5, 0.5), rng.uniform(0, 0.6)))
            elif p < 0.35:
                link.send(proto.T_SIM_JOYSTICK, proto.p_sim_joystick(True, 0.0, 0.0))
            elif p < 0.38:
                gw.call(g.rec.trigger, "button", "soak SAVE")
                saves += 1
            elif p < 0.42:
                n = rng.choice(list(g.net.driver.state))
                g.net.driver.state[n]["internet"] = rng.random() < 0.7
                g.net.driver.state[n]["up"] = g.net.driver.state[n]["internet"]
                channel_changes += 1
            elif p < 0.45:
                g.vision.inject(rng.uniform(-20, 20), rng.uniform(40, 200), seconds=10.0)
            w.pump(0.2)
            if time.time() >= next_sample:
                next_sample += a.sample
                socks = sockets_by_pid()
                d = w.telem or {}
                vst = g.vision.status()
                procs = {"gateway": os.getpid(), "simulator": sim.p.pid,
                         "mediamtx": g.cams.proc.pid if getattr(g.cams, "proc", None) else None,
                         "vision": g.vision.proc.pid if g.vision.proc else None}
                for n, c in g.rec.cams.items():
                    procs["rec_" + n] = c.proc.pid if getattr(c, "proc", None) else None
                row = {"t_s": round(time.time() - t0, 1)}
                for k, pid in procs.items():
                    inf = proc_info(pid)
                    if inf:
                        row[k + "_ws_mb"], row[k + "_priv_mb"], row[k + "_handles"], row[k + "_cpu_s"] = \
                            round(inf[0], 1), round(inf[1], 1), inf[2], round(inf[3], 1)
                    row[k + "_sockets"] = socks.get(pid, 0) if pid else 0
                nf, nb = du(root)
                ss = g.sync.status()
                row.update({"storage_files": nf, "storage_mb": round(nb / 1e6, 1),
                            "ring_deleted": g.rec.deleted["ring"], "events_deleted": g.rec.deleted["events"],
                            "sync_queue_mb": round(sum(q["mb"] for q in ss["queue"].values()), 1),
                            "sync_confirmed": ss["confirmed"], "sync_state": ss["state"],
                            "channel": g.net.active, "core0_max_us": d.get("t_core0_max"),
                            "overruns": d.get("ovr_core0"), "log_dropped": d.get("log_dropped"),
                            "vision_state": vst["state"], "vision_fps": vst["fps"], "vision_restarts": vst["restarts"],
                            "vision_paused": int(vst["paused"]), "saves": saves})
                rows.append(row)
                print("soak %5.0f s  gw %s MB  sim %s MB  vision %s MB %s fps  storage %s MB  sync queue %s MB  "
                      "channel %s" % (row["t_s"], row.get("gateway_ws_mb"), row.get("simulator_ws_mb"),
                                      row.get("vision_ws_mb"), row["vision_fps"], row["storage_mb"],
                                      row["sync_queue_mb"], row["channel"]), flush=True)
            time.sleep(rng.uniform(1.0, 4.0))
        link.stop()
    finally:
        for c in cams:
            c.stop()
        if asim:
            asim.stop()
        if gw:
            gw.stop()
        if sim:
            sim.stop()
        if recv:
            try:
                recv.stop()
            except Exception:
                pass
        shutil.rmtree(tmp, ignore_errors=True)
    if not rows:
        print("no samples")
        return 1
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(OUT, "soak_samples.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)
    lines = ["=== SYSTEM SOAK: %.0f min real time, seed %d, %d samples ===" % (a.minutes, a.seed, len(rows)), "",
             "| measure | first | last | trend per hour | R^2 | verdict |", "|---|---|---|---|---|---|"]
    leaks = 0
    xs = [r["t_s"] for r in rows]
    for k in keys:
        if not (k.endswith("_priv_mb") or k.endswith("_handles") or k.endswith("_sockets")):
            continue
        pts = [(r["t_s"], r[k]) for r in rows if isinstance(r.get(k), (int, float))]
        if len(pts) < 3:
            continue
        sl, r2 = slope([p[0] for p in pts], [p[1] for p in pts])
        lim = 20.0 if k.endswith("_mb") else 50.0
        steady = sl < lim or r2 < 0.6                   # (only growth counts; shrinking is fine)
        leaks += 0 if steady else 1
        lines.append("| %s | %s | %s | %+.1f | %.2f | %s |" % (k, pts[0][1], pts[-1][1], sl, r2,
                                                           "steady" if steady else "GROWS (possible leak)"))
    last = rows[-1]
    lines += ["", "storage %s MB in %s files at the end; ring files deleted %s; saved events deleted %s" % (
        last["storage_mb"], last["storage_files"], last["ring_deleted"], last["events_deleted"]),
        "sync: %s files confirmed, backlog %s MB at the end (max %s MB)" % (
            last["sync_confirmed"], last["sync_queue_mb"], max(r["sync_queue_mb"] for r in rows)),
        "AURIX: control step max %s us (PC), overruns %s, log drops %s" % (
            max((r["core0_max_us"] or 0) for r in rows), last["overruns"], last["log_dropped"]),
        "vision: state at the end %s, fps min %s / max %s, restarts %s, samples paused %d" % (
            last["vision_state"], min(r["vision_fps"] for r in rows), max(r["vision_fps"] for r in rows),
            last["vision_restarts"], sum(r["vision_paused"] for r in rows)),
        "activity: %d SAVEs, %d channel changes, %d mode button presses" % (saves, channel_changes, mode_changes),
        "NOTE: real time only - %.0f min. The trends above are per hour; 24 h of AURIX time: storm.exe --soak." % a.minutes]
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "soak_report.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(txt)
    return 1 if leaks else 0


if __name__ == "__main__":
    sys.exit(main())
