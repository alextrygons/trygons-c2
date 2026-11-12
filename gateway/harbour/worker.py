"""The harbour sensor worker loop (docs/harbour_design.md 5, step HX3) - shared by radar_worker.py / lidar_worker.py.

Each sensor = its OWN process (like the vision / ADS-B workers): below-normal priority, ONE thread of work (no thread
pools; OMP / MKL / OpenBLAS limited to 1 in case a vendor SDK pulls numpy in), JSON lines:
  stdin   {"op": "pose", "t", "lat", "lon", "heading", "sog", "cog"}   the own boat (the gateway, 5 Hz)
          {"op": "fault", "name": "lost" | "degraded" | "clutter" | "ghost" | "skew" | "crash" | "freeze" | "clear",
           "on": bool}                                                  (SIMULATOR only - the real drivers ignore it)
          {"op": "stop"}
  stdout  {"ev": "status", ...} once a second, {"ev": "radar" | "lidar", ...} per sweep / scan (sensors.py frames)
It ends when stdin closes (the gateway's end). It sends NOTHING but its stdout: no socket to the boat, no AURIX.
A pose older than 2 s = no batch (a sensor batch without the own pose is useless to the tracker) + self "degraded".
"""
import json
import os
import sys
import threading
import time

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ[_v] = "1"

from harbour import sensors, simworld  # noqa: E402

out_lock = threading.Lock()
POSE_MAX_AGE_S = 2.0


def emit(obj):
    try:
        with out_lock:
            sys.stdout.write(json.dumps(obj) + "\n")
            sys.stdout.flush()
    except (OSError, ValueError):
        os._exit(0)


def low_priority():
    try:
        if os.name == "nt":
            import ctypes
            from ctypes import wintypes
            k = ctypes.WinDLL("kernel32", use_last_error=True)
            k.GetCurrentProcess.restype = wintypes.HANDLE
            k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
            return bool(k.SetPriorityClass(k.GetCurrentProcess(), 0x00004000))     # BELOW_NORMAL
        os.nice(10)
        return True
    except (OSError, AttributeError, ValueError):
        return False


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None
        self.faults = {}
        self.stop = False


def stdin_reader(st):
    try:
        for line in sys.stdin:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            op = m.get("op")
            with st.lock:
                if op == "pose":
                    p = {k: m.get(k) for k in ("t", "lat", "lon", "heading", "sog", "cog")}
                    if sensors.valid_pose(p):
                        st.pose = p
                elif op == "fault":
                    n = m.get("name")
                    if n == "clear":
                        st.faults = {}
                    elif isinstance(n, str):
                        st.faults[n] = m.get("on", True)
                elif op == "stop":
                    st.stop = True
                    return
    except (OSError, ValueError):
        pass
    st.stop = True                      # stdin closed = the gateway ended


def run(kind, argv=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="{}", help="the harbour.<kind> config (JSON)")
    ap.add_argument("--world", default="", help="a world file (simworld.py) - simulator only")
    ap.add_argument("--world-json", default="", help="the world itself (JSON; the gateway fixes its origin)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)
    c = dict(sensors.DEFAULTS[kind])
    c.update(json.loads(a.cfg))
    low_priority()
    st = State()
    threading.Thread(target=stdin_reader, args=(st,), daemon=True).start()
    drv = c.get("driver", "sim")
    src = None
    why_off = ""
    if drv == "sim":
        if a.world_json:
            d = json.loads(a.world_json)
            world = simworld.World(d.get("objects"), d.get("origin"), d.get("t0"))
        else:
            world = simworld.World.load(a.world) if a.world else simworld.World()
        if kind == "radar":
            from harbour.simradar import RadarSim
            src = RadarSim(world, c, seed=a.seed or 11)
        else:
            from harbour.simlidar import LidarSim
            src = LidarSim(world, c, seed=a.seed or 12)
    else:
        sk = sensors.RealDriverSkeleton(kind, drv, c)
        _ok, why_off = sk.open()
    period = 1.0 / float(c["rate_hz"])
    seq, batches, sent_t = 0, 0, []
    next_t, next_status = time.time(), 0.0
    t_start = time.time()
    while not st.stop:
        now = time.time()
        with st.lock:
            pose, f = st.pose, dict(st.faults)
        if f.get("crash"):
            os._exit(3)                                 # the test: a crashed worker
        if f.get("freeze"):
            time.sleep(0.2)                             # the test: a hung worker (no status, no data)
            continue
        if now >= next_status:
            next_status = now + 1.0
            sent_t = [t for t in sent_t if now - t <= 5.0]
            rate = (len(sent_t) - 1) / max(1e-3, sent_t[-1] - sent_t[0]) if len(sent_t) > 1 else 0.0
            if src is None:
                self_st, why = "offline", why_off
            elif pose is None and now - t_start < POSE_MAX_AGE_S + 1.0:
                self_st, why = "ok", "waiting for the own pose"
            elif pose is None or now - pose["t"] > POSE_MAX_AGE_S:
                self_st, why = "degraded", "no own pose"
            elif f.get("degraded"):
                self_st, why = "degraded", "sensor reports degraded (sim)"
            else:
                self_st, why = "ok", ""
            emit({"ev": "status", "kind": kind, "driver": drv, "self": self_st, "why": why,
                  "rate_hz": round(rate, 2), "batches": batches, "t": now, "faults": sorted(k for k in f if f[k])})
        if now < next_t:
            time.sleep(min(0.05, next_t - now))
            continue
        next_t += period
        if next_t < now - period:
            next_t = now + period                       # fell behind (a stall): no burst of catch-up sweeps
        if src is None or pose is None or now - pose["t"] > POSE_MAX_AGE_S or f.get("lost"):
            continue
        t_meas = now - float(c["latency_s"])
        if isinstance(src.w, simworld.World):
            src.w.place(pose["lat"], pose["lon"], t_meas)
        raw = src.sweep(t_meas, pose, f) if kind == "radar" else src.scan(t_meas, pose, f)
        if raw is None:
            continue                                    # a dropped sweep / scan
        if kind == "lidar":
            raw = sensors.lidar_filter(raw, c)
        valid = sensors.valid_plot if kind == "radar" else sensors.valid_object
        items = [i for i in raw if valid(i)][:int(c.get("max_items", 400) or 400)]
        seq += 1
        batches += 1
        sent_t.append(now)
        t_stamp = t_meas + (0.4 if f.get("skew") else 0.0)
        emit({"ev": kind, "seq": seq, "t": round(t_stamp, 3), "t_tx": round(time.time(), 3), "pose": pose,
              "items": items, "bad": len(raw) - len(items)})
    return 0
