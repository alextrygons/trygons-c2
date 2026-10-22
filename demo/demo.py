"""One-click demo: the whole C2 on this PC, no boat hardware.

    python demo/demo.py start     (START_DEMO.bat)
    python demo/demo.py stop      (STOP_DEMO.bat)
    python demo/demo.py check     only check the tools
    python demo/demo.py ship      a ferry on a collision course with the boat (DEMO_SHIP.bat)
    python demo/demo.py kayak     a kayak without AIS ahead, seen only by the camera (DEMO_KAYAK.bat)

Starts, each in the background with a log in demo_run/logs/:
  1. the AURIX simulator   sim/autopilot_sim.exe server 5760 1 origin=<Saronic Gulf>
                            (USB link 5760, gateway UART 5762)
  2. the gateway (PC/sim)  gateway/gateway.py --config demo_run/gateway.json
                            (stations 5761, starts MediaMTX itself: RTSP 8554)
  3. the demo world        demo/demo_world.py: 4 camera test streams + AIS traffic (5790)
  4. the ground stations   SHORE: through the gateway, captain + driver screen
                            ON BOARD: straight on the simulator (panel buttons, joystick, CONFIRM)
Everything it makes goes into demo_run/ (not in git). Users: captain / demo.
"""
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
RUN = os.path.join(REPO, "demo_run")
LOGS = os.path.join(RUN, "logs")
PIDS = os.path.join(RUN, "pids.json")
CMD = os.path.join(RUN, "world_cmd.json")
sys.path.insert(0, os.path.join(REPO, "gateway"))
sys.path.insert(0, os.path.join(REPO, "groundstation"))

ORIGIN = (37.82, 23.60)             # Saronic Gulf, between Piraeus and Aegina (~70 m deep)
PORTS = {"sim USB link": 5760, "gateway stations": 5761, "sim gateway UART": 5762, "AIS": 5790,
         "MediaMTX RTSP": 8554, "MediaMTX API": 9997, "gateway web": 8080}
USER, PASSWORD = "captain", "demo"
EXE = os.path.join(REPO, "sim", "autopilot_sim.exe" if os.name == "nt" else "autopilot_sim")
NO_WINDOW = 0x08000000 if os.name == "nt" else 0     # CREATE_NO_WINDOW


def say(text):
    print(text, flush=True)


# --- checks --------------------------------------------------------------------------
def check_tools(interactive=True):
    import toolpaths
    ok = True
    say("Python   %s" % sys.executable)
    try:
        import tkinter  # noqa: F401
    except ImportError:
        say("  MISSING tkinter: install Python with Tcl/Tk (MSYS2: pacman -S mingw-w64-ucrt-x86_64-tk)")
        ok = False
    ff = toolpaths.ffmpeg()
    say("ffmpeg   %s" % (ff or "MISSING"))
    if not ff:
        say("  install it: MSYS2 terminal -> pacman -S mingw-w64-ucrt-x86_64-ffmpeg\n"
            "  (or download ffmpeg and set FFMPEG_BIN=C:\\path\\to\\ffmpeg.exe)")
        ok = False
    mtx = toolpaths.mediamtx()
    if not mtx and interactive:
        ans = input("MediaMTX (video relay) is missing. Download it now into tools\\bin (about 20 MB)? [Y/n] ")
        if ans.strip().lower() in ("", "y", "yes"):
            subprocess.call([sys.executable, os.path.join(REPO, "tools", "get_mediamtx.py")])
            mtx = toolpaths.mediamtx()
    say("MediaMTX %s" % (mtx or "MISSING"))
    if not mtx:
        say("  get it: python tools\\get_mediamtx.py   (or set MEDIAMTX_BIN)")
        ok = False
    return ok, mtx


def port_busy(port):
    s = socket.socket()
    s.settimeout(0.3)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def wait_port(port, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if port_busy(port):
            return True
        time.sleep(0.2)
    return False


# --- preparation ---------------------------------------------------------------------
def build_sim():
    if os.path.exists(EXE):
        newest = max(os.path.getmtime(os.path.join(d, f)) for d in ("app", "sim")
                     for f in os.listdir(os.path.join(REPO, d)) if f.endswith((".c", ".h")))
        if os.path.getmtime(EXE) >= newest:
            return True
    say("building the simulator (sim\\build_sim.bat) ...")
    if os.name == "nt":
        r = subprocess.run(["cmd", "/c", os.path.join("sim", "build_sim.bat")], cwd=REPO, capture_output=True,
                           text=True)
    else:
        r = subprocess.run(["sh", os.path.join("sim", "build_sim.sh")], cwd=REPO, capture_output=True, text=True)
    if not os.path.exists(EXE) or "BUILD OK" not in r.stdout:
        say(r.stdout[-2000:] + r.stderr[-2000:])
        return False
    return True


def demo_charts():
    """The real charts when prepared (tools/chartprep.py), else a synthetic DEMO chart."""
    import chart
    real = os.path.join(REPO, "charts")
    c = chart.Charts(real)
    if c.depth(*ORIGIN) is not None:
        say("charts   %s (real, prepared with tools/chartprep.py)" % real)
        return real
    folder = os.path.join(RUN, "charts")
    lat0, lon0, dlat, dlon = ORIGIN[0] - 0.15, ORIGIN[1] - 0.2, 0.002, 0.0025
    rows = []
    for i in range(150):
        r = []
        for j in range(160):
            la, lo = lat0 + i * dlat, lon0 + j * dlon
            d = chart.dist_m(la, lo, ORIGIN[0] - 0.012, ORIGIN[1] + 0.03)   # a 2 m shoal 2.9 km SE
            r.append(2.0 if d < 300 else 40.0)
        rows.append(r)
    chart.write_area(os.path.join(folder, "demo_sea"), "demo_sea", lat0, lon0, dlat, dlon, rows, ["synthetic"],
                     "SYNTHETIC DEMO CHART - NOT REAL DEPTHS (run tools/chartprep.py for the real one)")
    say("charts   SYNTHETIC demo chart (no real charts prepared: tools\\chartprep.py, needs internet)")
    return folder


def vision_config():
    import toolpaths
    model = os.path.join(REPO, "gateway", "vision", "models", "ewasr_resnet18.onnx")
    py = toolpaths.vision_python()
    ok = os.path.exists(model) and os.path.exists(os.path.join(HERE, "media", "sea.mp4")) and py and         subprocess.run([py, "-c", "import onnxruntime, numpy"], capture_output=True).returncode == 0
    say("vision   %s" % ("eWaSR on the sea video (FRONT-1)" if ok else
                         "synthetic detections only (for the real model: tools\\get_vision_python.py, "
                         "get_vision_model.py, get_vision_samples.py)"))
    return {"enabled": True, "source": "camera" if ok else "fake", "allow_inject": True}


def gateway_config(mtx, charts):
    from users import hash_pw
    names = ["FRONT-1", "FRONT-2", "REAR-LEFT", "REAR-RIGHT"]
    roles = ["front_main", "front_backup", "rear_left", "rear_right"]
    url = "rtsp://127.0.0.1:8554/simcam_%d"
    cfg = {
        "boat": "trygons-demo",
        "aurix": "tcp:127.0.0.1:5762", "set_clock": False,
        # the demo ground station counts as a SHORE station (only 127.0.0.2 is "on the boat")
        "stations": {"listen": "127.0.0.1", "port": 5761, "local_nets": ["127.0.0.2/32"]},
        "users": {USER: {"role": "HELM", "pw": hash_pw(PASSWORD)},
                  "tool": {"role": "VIEWER", "pw": hash_pw(PASSWORD)}},
        "channels": {"driver": "sim", "starlink_status": False, "check_s": 1.0, "stable_s": 10.0},
        "cameras": {"enabled": True, "mediamtx_bin": mtx, "mediamtx_conf": os.path.join(RUN, "mediamtx.yml"),
                    "mediamtx_log": os.path.join(LOGS, "mediamtx.log"),
                    "list": [{"name": n, "role": r, "url_main": url % i, "url_sub": url % i}
                             for i, (n, r) in enumerate(zip(names, roles))],
                    "extra_paths": ["simcam_%d" % i for i in range(4)]},
        # (the test pictures are nearly static: tiny bitrate, files grow slowly -> longer stall time)
        "recording": {"enabled": True, "ring_min": 5, "stall_s": 60.0},
        "sync": {"enabled": False},
        "ais": {"enabled": True, "source": "tcp:127.0.0.1:5790", "silent_s": 120.0},
        "charts_dir": charts,
        # obstacle detection: the real model on the sea video when both exist, else synthetic detections only;
        # DEMO_KAYAK.bat injects a kayak without AIS ahead
        "vision": vision_config(),
        "audit_fallback": os.path.join(RUN, "audit_fallback"),
        "storage": {"root": os.path.join(RUN, "ssd"), "require_mount": False, "check_s": 1, "low_free_gb": 0.0,
                    "full_free_gb": 0.0},
    }
    path = os.path.join(RUN, "gateway.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=1)
    return path


# --- processes -----------------------------------------------------------------------
def launch(name, args, cwd, pids):
    log = open(os.path.join(LOGS, name + ".log"), "w", encoding="utf-8")
    p = subprocess.Popen(args, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                         creationflags=NO_WINDOW)
    pids[name] = p.pid
    with open(PIDS, "w", encoding="utf-8") as f:
        json.dump(pids, f, indent=1)
    return p


def kill_tree(pid):
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True)
    else:
        try:
            os.killpg(pid, 9)
        except OSError:
            pass


def stop():
    if not os.path.exists(PIDS):
        say("demo not running (no demo_run/pids.json)")
        return 0
    with open(PIDS, encoding="utf-8") as f:
        pids = json.load(f)
    for name, pid in reversed(list(pids.items())):          # ground stations first, the simulator last
        kill_tree(pid)
        say("stopped %s (pid %d)" % (name, pid))
    os.remove(PIDS)
    return 0


def start():
    say("=== Trygons C2 demo ===")
    if os.path.exists(PIDS):
        say("an old demo is still registered: stopping it first")
        stop()
        time.sleep(1.0)
    ok, mtx = check_tools(interactive=sys.stdin is not None and sys.stdin.isatty())
    if not ok:
        say("\nSOMETHING IS MISSING (see above) - the demo can't start.")
        return 1
    busy = [("%s %d" % (n, p)) for n, p in PORTS.items() if port_busy(p)]
    if busy:
        say("these ports are in use (another demo / simulator?): %s\nclose it (STOP_DEMO.bat) and try again"
            % ", ".join(busy))
        return 1
    if not build_sim():
        say("the simulator did not build (see above)")
        return 1
    os.makedirs(LOGS, exist_ok=True)
    charts = demo_charts()
    cfg = gateway_config(mtx, charts)
    py = sys.executable
    pids = {}

    # "usb": the onboard joystick = the simulated USB joystick (lever + stick sliders in the ON BOARD window)
    launch("simulator", [EXE, "server", "5760", "1", "usb", "origin=%.4f,%.4f" % ORIGIN], os.path.join(REPO, "sim"), pids)
    if not wait_port(5762, 10):
        say("the simulator did not start (demo_run/logs/simulator.log)")
        stop()
        return 1
    say("started  simulator (Saronic Gulf %.2f N %.2f E)" % ORIGIN)
    launch("gateway", [py, os.path.join(REPO, "gateway", "gateway.py"), "--config", cfg],
           os.path.join(REPO, "gateway"), pids)
    if not wait_port(5761, 15) or not wait_port(8554, 15):
        say("the gateway / MediaMTX did not start (demo_run/logs/gateway.log, mediamtx.log)")
        stop()
        return 1
    say("started  gateway + MediaMTX")
    video = os.path.join(HERE, "media", "sea.mp4")
    launch("world", [py, os.path.join(HERE, "demo_world.py"), "--lat", str(ORIGIN[0]), "--lon", str(ORIGIN[1]),
                     "--cmd", CMD] + (["--front-video", video] if os.path.exists(video) else []), HERE, pids)
    if not wait_port(5790, 10):
        say("the demo world (cameras + AIS) did not start (demo_run/logs/world.log)")
        stop()
        return 1
    say("started  4 camera test streams + AIS traffic")
    gs = os.path.join(REPO, "groundstation", "app.py")
    launch("onboard", [py, gs, "tcp:127.0.0.1:5760", "--title", "ON BOARD (simulator panel + joystick)",
                       "--charts", charts, "--layout", os.path.join(RUN, "layout_onboard.json")],
           os.path.join(REPO, "groundstation"), pids)
    time.sleep(1.0)
    launch("shore", [py, gs, "gw:127.0.0.1:5761", "--login", "%s:%s" % (USER, PASSWORD), "--driver", "--cams", "Full",
                     "--title", "SHORE (captain)", "--charts", charts,
                     "--layout", os.path.join(RUN, "layout_shore.json")],
           os.path.join(REPO, "groundstation"), pids)
    say("started  ground stations: SHORE (captain + driver screen) and ON BOARD\n\n"
        "The demo is running. MANUAL.txt chapter 'DEMO' says what to try.\n"
        "A ferry on a collision course: DEMO_SHIP.bat.   Stop everything: STOP_DEMO.bat.")
    return 0


def ship():
    """A ferry from starboard that would hit the boat in about 6 minutes (keeps course and speed)."""
    from gwclient import GatewayLink
    from ais import move
    link = GatewayLink("127.0.0.1:5761", "tool", password=PASSWORD, station="demo tool")
    link.start()
    d = None
    end = time.time() + 10
    while time.time() < end and d is None:
        try:
            k, v = link.events.get(timeout=0.5)
        except Exception:
            continue
        if k == "telemetry" and v.get("pos_source"):
            d = v
    link.stop()
    if d is None:
        say("no position from the boat (is the demo running?)")
        return 1
    kn, t_s = 14.0, 360.0
    sog = max(d.get("est_sog") or 0.0, 0.0)
    cog = d.get("est_cog") or d.get("heading") or 0.0
    x_lat, x_lon = move(d["est_lat"], d["est_lon"], cog, sog, t_s)          # where we will be
    s_cog = (cog - 90.0) % 360.0                                             # coming from starboard
    s_lat, s_lon = move(x_lat, x_lon, (s_cog + 180.0) % 360.0, kn, t_s)
    os.makedirs(RUN, exist_ok=True)
    with open(CMD, "w", encoding="utf-8") as f:
        json.dump({"ship": {"lat": s_lat, "lon": s_lon, "sog": kn, "cog": s_cog, "name": "DEMO FERRY"}}, f)
    say("DEMO FERRY: %.1f kn, course %03.0f, from starboard, collision in about %.0f min "
        "(boat %.1f kn course %03.0f)" % (kn, s_cog, t_s / 60.0, sog, cog))
    return 0


def kayak():
    """A kayak WITHOUT AIS 120 m straight ahead for 40 s - only the camera obstacle detection sees it
    (injected as a detection: the demo cameras show no real kayak)."""
    from gwclient import GatewayLink
    link = GatewayLink("127.0.0.1:5761", "tool", password=PASSWORD, station="demo tool")
    link.start()
    link.send_msg({"t": "vision_inject", "rel": 0.0, "dist": 120.0, "seconds": 40.0, "conf": 0.85})
    ans = None
    end = time.time() + 5
    while time.time() < end and ans is None:
        try:
            k, v = link.events.get(timeout=0.5)
        except Exception:
            continue
        if k == "vision_ack":
            ans = v
    link.stop()
    say("KAYAK (no AIS) 120 m ahead for 40 s: %s" % (ans and ans.get("text") or "no answer (demo running?)"))
    return 0 if ans and ans.get("ok") else 1


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "start"
    if cmd == "start":
        return start()
    if cmd == "stop":
        return stop()
    if cmd == "check":
        return 0 if check_tools(interactive=False)[0] else 1
    if cmd == "ship":
        return ship()
    if cmd == "kayak":
        return kayak()
    say(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
