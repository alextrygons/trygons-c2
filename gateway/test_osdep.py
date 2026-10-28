"""The gateway PC on BOTH operating systems (step 12: Ubuntu 24.04 LTS and
Windows 11) - the OS-dependent bits of gateway/osdep.py with fakes, so every
case runs on any PC (no adapter, no admin rights, no pyserial needed).

    python test_osdep.py        (exit code 0 = all passed)

  OS1  serial device by USB hardware id (usb:VID:PID[:SERIAL]) on Windows (COMn) and
       Ubuntu (/dev/ttyUSBn): exact, wildcard, not plugged in, two matches = refused
  OS2  Ubuntu /dev/serial/by-id/... -> the real device; missing / on Windows = refused
  OS3  COMn only on Windows, /dev/... only on Ubuntu, tcp: passes, empty / unknown refused
  OS4  config check: not set, Raspberry Pi UARTs, unstable names, placeholders, the same
       device twice, bad baud, a path of the other OS, a good config on each OS
  OS5  folders per OS; the defaults carry no Pi device and no fixed Linux path on Windows
  OS6  --list-serial: by-id links + usb: names, the RS232 caution, the NEVER-RS232 line
  OS7  clean stop: SIGINT / SIGTERM on Ubuntu, Ctrl+Break on Windows (fake loop / signals)
  OS8  CPU temperature / throttle counters from a fake sysfs; none on Windows
  OS9  the AURIX link resolves the adapter again at EVERY reconnect (COM3 -> COM7 after a
       re-plug), the AIS reader too; a missing adapter = an error, retried
  OS10 the vision resource guard: CPU hot / throttling -> PAUSED reason, no Pi words
  OS11 gateway.py --check-config / --list-serial (exit codes)
  OS12 install files: systemd (Ubuntu), Task Scheduler + power + firewall (Windows),
       lid closed, charge limit 80 %, USB never suspended; no Pi-only part left in the code
"""
import asyncio
import os
import shutil
import subprocess
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import osdep  # noqa: E402

results = []


def report(name, ok, detail=""):
    results.append(bool(ok))
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" - " + detail) if detail else ""))


class Port:
    def __init__(self, device, vid=None, pid=None, sn=None, desc=""):
        self.device, self.vid, self.pid, self.serial_number, self.description = device, vid, pid, sn, desc


FTDI, CP = (0x0403, 0x6001), (0x10C4, 0xEA60)


def err(fn):
    try:
        return "OK:%s" % fn()
    except osdep.SerialSpecError as e:
        return "ERR:%s" % e


def os1():
    win = [Port("COM3", *FTDI, "A50285BI"), Port("COM4", *CP, "0001"), Port("COM1")]
    lin = [Port("/dev/ttyUSB0", *FTDI, "A50285BI"), Port("/dev/ttyUSB1", *CP, "0001"), Port("/dev/ttyS0")]
    r = {
        "win exact": osdep.resolve_serial("usb:0403:6001:A50285BI", win, "win32"),
        "lin exact": osdep.resolve_serial("usb:0403:6001:A50285BI", lin, "linux"),
        "win vid:pid": osdep.resolve_serial("usb:10c4:ea60", win, "win32"),
        "lin wildcard": osdep.resolve_serial("usb:*:*:0001", lin, "linux"),
        "upper case hex": osdep.resolve_serial("usb:10C4:EA60:0001", lin, "linux"),
    }
    ok = r == {"win exact": "COM3", "lin exact": "/dev/ttyUSB0", "win vid:pid": "COM4",
               "lin wildcard": "/dev/ttyUSB1", "upper case hex": "/dev/ttyUSB1"}
    gone = err(lambda: osdep.resolve_serial("usb:0403:6001:OTHER", win, "win32"))
    two = err(lambda: osdep.resolve_serial("usb:*:*", lin + [Port("/dev/ttyUSB2", *FTDI, "B")], "linux"))
    ok = ok and gone.startswith("ERR:") and "not plugged in" in gone and two.startswith("ERR:") and "3 adapters" in two
    report("OS1 serial device by USB hardware id on Windows and Ubuntu", ok, "%s; missing: %s; ambiguous: %s"
           % (r, gone, two[:70]))


def os2():
    link = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_A50285BI-if00-port0"
    real = {link: "/dev/ttyUSB3"}
    a = osdep.resolve_serial(link, [], "linux", exists=lambda p: p in real, realpath=lambda p: real.get(p, p))
    b = err(lambda: osdep.resolve_serial(link + "X", [], "linux", exists=lambda p: p in real, realpath=lambda p: p))
    c = err(lambda: osdep.resolve_serial(link, [], "win32", exists=lambda p: True, realpath=lambda p: p))
    report("OS2 Ubuntu /dev/serial/by-id -> the real device; missing / on Windows refused",
           a == "/dev/ttyUSB3" and "not plugged in" in b and "Ubuntu name" in c, "%s; %s; %s" % (a, b, c))


def os3():
    cases = [("COM7", "win32", "OK:COM7"), ("\\\\.\\COM12", "win32", "OK:\\\\.\\COM12"), ("COM7", "linux", "ERR"),
             ("/dev/ttyACM0", "linux", "OK:/dev/ttyACM0"), ("/dev/ttyACM0", "win32", "ERR"),
             ("tcp:127.0.0.1:5762", "win32", "OK:tcp:127.0.0.1:5762"), ("tcp:127.0.0.1:5762", "linux", "OK:tcp:"),
             ("", "linux", "ERR"), ("   ", "win32", "ERR"), ("ttyUSB0", "linux", "ERR"), ("serial please", "win32", "ERR")]
    bad = []
    for spec, plat, want in cases:
        got = err(lambda: osdep.resolve_serial(spec, [], plat, exists=lambda p: True, realpath=lambda p: p))
        if not got.startswith(want):
            bad.append((spec, plat, got))
    report("OS3 COMn only on Windows, /dev/ only on Ubuntu, tcp: passes, empty / unknown refused", not bad,
           "%d cases%s" % (len(cases), (", wrong: %s" % bad) if bad else ""))


def cfg(**kw):
    c = {"aurix": "usb:0403:6001:A50285BI", "baud": 921600, "set_clock": False,
         "ais": {"enabled": True, "source": "usb:10c4:ea60:0001", "baud": 38400},
         "storage": {"root": "/var/lib/trygons/data"}}
    for k, v in kw.items():
        if k == "ais_source":
            c["ais"]["source"] = v
        elif k == "ais_baud":
            c["ais"]["baud"] = v
        elif k == "root":
            c["storage"]["root"] = v
        else:
            c[k] = v
    return c


def levels(problems):
    return sorted(set(l for l, _ in problems))


def os4():
    lin, win = "linux", "win32"
    good_l = osdep.check_config(cfg(), lin)
    good_w = osdep.check_config(cfg(root="C:\\ProgramData\\trygons\\data"), win)
    checks = {
        "good Ubuntu": good_l == [],
        "good Windows": good_w == [],
        "aurix not set": levels(osdep.check_config(cfg(aurix=""), lin)) == ["error"],
        "Pi UART ttyAMA0": "Raspberry Pi" in str(osdep.check_config(cfg(aurix="/dev/ttyAMA0"), lin)),
        "Pi serial0 for AIS": "Raspberry Pi" in str(osdep.check_config(cfg(ais_source="/dev/serial0"), lin)),
        "COM5 = warning": levels(osdep.check_config(cfg(aurix="COM5", root="D:\\gw"), win)) == ["warning"],
        "COM5 on Ubuntu = error": "error" in levels(osdep.check_config(cfg(aurix="COM5"), lin)),
        "ttyUSB0 = warning": levels(osdep.check_config(cfg(aurix="/dev/ttyUSB0"), lin)) == ["warning"],
        "by-id on Windows = error": "error" in levels(osdep.check_config(
            cfg(aurix="/dev/serial/by-id/usb-x", root="D:\\gw"), win)),
        "placeholder": "placeholder" in str(osdep.check_config(cfg(aurix="usb:VID:PID:SERIAL"), lin)),
        "no serial number = warning": levels(osdep.check_config(cfg(aurix="usb:0403:6001"), lin)) == ["warning"],
        "same device twice": "same device" in str(osdep.check_config(cfg(ais_source="usb:0403:6001:A50285BI"), lin)),
        "AIS off = not checked": osdep.check_config(dict(cfg(), ais={"enabled": False, "source": ""}), lin) == [],
        "bad baud": "error" in levels(osdep.check_config(cfg(baud="fast"), lin)),
        "57600 = warning": levels(osdep.check_config(cfg(baud=57600), lin)) == ["warning"],
        "bad AIS baud": "error" in levels(osdep.check_config(cfg(ais_baud=0), lin)),
        "Windows path on Ubuntu": "Windows path" in str(osdep.check_config(cfg(root="C:\\data"), lin)),
        "Ubuntu path on Windows": "Ubuntu path" in str(osdep.check_config(cfg(), win)),
        "set_clock on Windows = warning": "offset" in str(osdep.check_config(
            cfg(set_clock=True, root="D:\\gw"), win)),
        "tcp sim ok": osdep.check_config(cfg(aurix="tcp:127.0.0.1:5762", ais_source="tcp:127.0.0.1:5763"), lin) == [],
        "tcp without port": "error" in levels(osdep.check_config(cfg(aurix="tcp:localhost"), lin)),
    }
    bad = [k for k, v in checks.items() if not v]
    report("OS4 config check on both OSes (not set, Pi UARTs, unstable names, placeholders, duplicates, baud, paths)",
           not bad, "%d checks%s" % (len(checks), (", wrong: %s" % bad) if bad else ""))


def os5():
    import common
    env = {"PROGRAMDATA": "D:\\PD"}
    w = lambda p: p.replace("/", "\\")              # MSYS2's Python joins Windows paths with "/"
    d = {"win data": w(osdep.data_dir("win32", env)), "win cfg": w(osdep.config_dir("win32", env)),
         "lin data": osdep.data_dir("linux", env), "lin cfg": osdep.config_dir("linux", env)}
    ok = d == {"win data": "D:\\PD\\trygons", "win cfg": "D:\\PD\\trygons",
               "lin data": "/var/lib/trygons", "lin cfg": "/etc/trygons"}
    D = common.DEFAULTS
    here = osdep.data_dir()
    paths = [D["storage"]["root"], D["audit_fallback"], D["cameras"]["mediamtx_conf"]]
    ok_paths = all(p.startswith(here) for p in paths)
    no_pi = D["aurix"] not in osdep.PI_UARTS and D["ais"]["source"] not in osdep.PI_UARTS
    report("OS5 folders per OS; the defaults: no Pi device, data on this OS's folder, own SSD (no mount needed)",
           ok and ok_paths and no_pi and D["storage"]["require_mount"] is False,
           "%s; defaults %s, aurix %r, ais %r" % (d, paths, D["aurix"], D["ais"]["source"]))


def os6():
    ports = [Port("/dev/ttyUSB0", *FTDI, "A50285BI", "TTL232R-3V3"), Port("/dev/ttyUSB1", 0x067B, 0x2303, None, "PL2303")]
    links = {"/dev/ttyUSB0": ["/dev/serial/by-id/usb-FTDI_TTL232R-3V3_A50285BI-if00-port0"]}
    out = "\n".join(osdep.list_serial(ports, links, "linux"))
    w = "\n".join(osdep.list_serial([Port("COM3", *CP, "0001", "CP210x")], {}, "win32"))
    none = osdep.list_serial([], {}, "win32")
    ok = ("use: /dev/serial/by-id/usb-FTDI" in out and "use: usb:0403:6001:A50285BI" in out and
          "use: usb:067b:2303" in out and "CAUTION: Prolific" in out and "NEVER RS232" in out and
          "COM3" in w and "use: usb:10c4:ea60:0001" in w and "NEVER RS232" in w and "no serial ports" in none[0])
    report("OS6 --list-serial: stable names (by-id + usb:), RS232 caution, the NEVER-RS232 line", ok,
           out.replace("\n", " | ")[:230])


class FakeLoop:
    def __init__(self):
        self.handlers, self.soon = {}, []

    def add_signal_handler(self, s, cb):
        self.handlers[s] = cb

    def call_soon_threadsafe(self, cb):
        self.soon.append(cb)


def os7():
    calls = []
    sig = types.SimpleNamespace(SIGINT=2, SIGTERM=15, SIGBREAK=21, set={})
    sig.signal = lambda s, h: sig.set.__setitem__(s, h)
    lo = FakeLoop()
    used_l = osdep.install_stop(lo, lambda: calls.append("stop"), "linux", sig)
    lw = FakeLoop()
    used_w = osdep.install_stop(lw, lambda: calls.append("stop"), "win32", sig)
    sig.set[21](21, None)                          # Ctrl+Break arrives (another thread on Windows)
    for cb in lw.soon:
        cb()
    ok = used_l == [2, 15] and set(lo.handlers) == {2, 15} and used_w == [21] and calls == ["stop"]
    report("OS7 clean stop: SIGINT + SIGTERM (Ubuntu / systemd), Ctrl+Break through the loop (Windows)", ok,
           "Ubuntu %s, Windows %s, stop called %d x" % (used_l, used_w, len(calls)))


def os8():
    tmp = tempfile.mkdtemp(prefix="osdep_")
    try:
        th = os.path.join(tmp, "thermal")
        for i, (t, v) in enumerate([("acpitz", "45000"), ("x86_pkg_temp", "71500"), ("bad", "x"), ("iwlwifi", "-5")]):
            z = os.path.join(th, "thermal_zone%d" % i)
            os.makedirs(z)
            open(os.path.join(z, "type"), "w").write(t)
            open(os.path.join(z, "temp"), "w").write(v)
        cpu = os.path.join(tmp, "cpu")
        for c, n in (("cpu0", "3"), ("cpu1", "4")):
            d = os.path.join(cpu, c, "thermal_throttle")
            os.makedirs(d)
            open(os.path.join(d, "package_throttle_count"), "w").write(n)
        t = osdep.cpu_temp_c("linux", th)
        n = osdep.throttle_count("linux", cpu)
        none = (osdep.cpu_temp_c("win32", th), osdep.throttle_count("win32", cpu),
                osdep.cpu_temp_c("linux", os.path.join(tmp, "nothing")), osdep.throttle_count("linux", tmp))
        report("OS8 CPU temperature (hottest zone) + throttle counters from sysfs; none on Windows / without sysfs",
               t == 71.5 and n == 7 and none == (None, None, None, None), "%.1f C, %s throttles, %s" % (t, n, none))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def os9():
    import aurix
    import gateway as G
    opened = []
    plugged = {"dev": "COM3"}

    class FakeSerial:
        def __init__(self, dev, baud, timeout=None):
            opened.append((dev, baud))
            self.n = 0

        def read(self, n):
            raise OSError("unplugged")              # the reader thread ends -> reconnect

        def readline(self):
            raise OSError("unplugged")

        def write(self, d):
            pass

        def close(self):
            pass

    fake = types.ModuleType("serial")
    fake.Serial = FakeSerial
    saved_mod, saved_cp = sys.modules.get("serial"), osdep.comports
    sys.modules["serial"] = fake
    osdep.comports = lambda: ([Port(plugged["dev"], *FTDI, "A50285BI")] if plugged["dev"] else [])
    try:
        link = aurix.AurixLink("usb:0403:6001:A50285BI", 921600)

        async def run():
            link._loop = asyncio.get_running_loop()
            await link._run_serial()                   # COM3
            plugged["dev"] = "COM7"                    # re-plugged into another USB port
            await link._run_serial()
            plugged["dev"] = None                      # pulled out
            try:
                await link._run_serial()
            except osdep.SerialSpecError as e:
                return str(e)
            return ""
        gone = asyncio.run(run())
        targets = types.SimpleNamespace(feed_line=lambda l: None)
        rd = G.AisReader("usb:0403:6001:A50285BI", 38400, targets)
        plugged["dev"] = "/dev/ttyUSB4"

        async def ais():
            await rd._serial()
        asyncio.run(ais())
        ok = (opened[:2] == [("COM3", 921600), ("COM7", 921600)] and "not plugged in" in gone and
              opened[2:] == [("/dev/ttyUSB4", 38400)])
        report("OS9 AURIX link + AIS reader resolve the adapter again at every reconnect (re-plug, unplug)", ok,
               "opened %s; unplugged: %s" % (opened, gone))
    finally:
        osdep.comports = saved_cp
        if saved_mod is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules["serial"] = saved_mod


def os10():
    from vision import manager as M
    saved = (osdep.cpu_temp_c, osdep.throttle_count)
    try:
        cnt = {"n": 10}
        osdep.cpu_temp_c = lambda *a, **k: 85.0
        osdep.throttle_count = lambda *a, **k: cnt["n"]
        s = M.SysLoad()
        first = s.throttling()                     # the first reading only sets the base
        cnt["n"] = 12
        s.next_thr = 0.0
        second = s.throttling()
        s.next_thr = 0.0
        third = s.throttling()                     # no new throttles -> not throttling
        src = open(os.path.join(HERE, "vision", "manager.py"), encoding="utf-8").read()
        ok = (s.temp_c() == 85.0 and first is False and second is True and third is False and
              "vcgencmd" not in src and '"Pi %' not in src and "CPU %.0f C" in src and "CPU throttling" in src)
        report("OS10 vision resource guard: CPU temperature + throttling through osdep, no Pi-only vcgencmd", ok,
               "85 C, throttling %s / %s / %s" % (first, second, third))
    finally:
        osdep.cpu_temp_c, osdep.throttle_count = saved


def os11():
    tmp = tempfile.mkdtemp(prefix="osdep_cfg_")
    try:
        good = os.path.join(tmp, "good.json")
        root = os.path.join(tmp, "data")
        import json
        json.dump({"aurix": "tcp:127.0.0.1:5762", "set_clock": False, "ais": {"enabled": False},
                   "storage": {"root": root}}, open(good, "w"))
        gw = os.path.join(HERE, "gateway.py")
        a = subprocess.run([sys.executable, gw, "--config", good, "--check-config"], capture_output=True, text=True,
                           timeout=60)
        b = subprocess.run([sys.executable, gw, "--config", os.path.join(HERE, "config.example.json"),
                            "--check-config"], capture_output=True, text=True, timeout=60)
        c = subprocess.run([sys.executable, gw, "--list-serial"], capture_output=True, text=True, timeout=60)
        ok = (a.returncode == 0 and "0 error" in a.stdout and b.returncode == 1 and "placeholder" in b.stderr and
              (c.returncode == 0 and "NEVER RS232" in c.stdout or "pyserial is missing" in c.stdout))
        report("OS11 gateway.py --check-config (good = 0, the example's placeholders = 1) and --list-serial", ok,
               "good %d, example %d, list %d" % (a.returncode, b.returncode, c.returncode))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def read(*p):
    with open(os.path.join(HERE, *p), encoding="utf-8") as f:
        return f.read()


def os12():
    svc, lid, tlp = read("systemd", "trygons-gateway.service"), read("systemd", "logind-trygons.conf"), \
        read("systemd", "tlp-trygons.conf")
    inst_u, inst_w, bat = read("systemd", "install_ubuntu.sh"), read("windows", "install_gateway.ps1"), \
        read("windows", "run_gateway.bat")
    checks = {
        "systemd restarts": "Restart=always" in svc,
        "systemd: external SSD optional": "-/mnt/ssd" in svc and "ReadWritePaths=/etc/trygons /var/lib/trygons" in svc,
        "systemd: serial group": "SupplementaryGroups=dialout" in svc,
        "Ubuntu lid closed": "HandleLidSwitch=ignore" in lid and "HandleLidSwitchExternalPower=ignore" in lid,
        "Ubuntu charge limit 80": "STOP_CHARGE_THRESH_BAT0=80" in tlp and "USB_AUTOSUSPEND=0" in tlp,
        "Ubuntu: sleep masked, no auto reboot": "systemctl mask sleep.target" in inst_u and
        'Automatic-Reboot "false"' in inst_u,
        "Windows task keeps running on battery": "-DontStopIfGoingOnBatteries" in inst_w and
        "-AllowStartIfOnBatteries" in inst_w,
        "Windows task: at boot, no time limit, restarts": "-AtStartup" in inst_w and "[TimeSpan]::Zero" in inst_w and
        "-RestartCount" in inst_w,
        "Windows lid closed (AC + battery)": "/setacvalueindex SCHEME_CURRENT SUB_BUTTONS LIDACTION 0" in inst_w and
        "/setdcvalueindex SCHEME_CURRENT SUB_BUTTONS LIDACTION 0" in inst_w,
        "Windows never sleeps": "standby-timeout-dc 0" in inst_w and "standby-timeout-ac 0" in inst_w,
        "Windows USB selective suspend off": inst_w.count("48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0") == 2,
        "Windows firewall: VPN + boat LAN only": "100.64.0.0/10" in inst_w and "10.42.0.0/24" in inst_w,
        "Windows no auto restart": "NoAutoRebootWithLoggedOnUsers" in inst_w,
        "Windows charge limit reminder": "80 %" in inst_w,
        "Windows restart loop": ":again" in bat and "goto again" in bat and "\r\n" in open(
            os.path.join(HERE, "windows", "run_gateway.bat"), "rb").read().decode("ascii"),
    }
    # no Pi-only part left in the gateway code (osdep names the Pi UARTs only to refuse them)
    left = []
    for dp, dn, fn in os.walk(HERE):
        dn[:] = [d for d in dn if d not in ("__pycache__", "gw_data", "logs")]
        for f in fn:
            if f.endswith((".py", ".sh", ".service", ".conf", ".json", ".ps1", ".bat")) and \
                    f not in ("osdep.py", "test_osdep.py", "test_gateway.py"):
                s = open(os.path.join(dp, f), encoding="utf-8", errors="replace").read()
                for w in ("vcgencmd", "ttyAMA0", "/dev/serial0", "dAISy", "raspi-config", "Raspberry"):
                    if w in s:
                        left.append("%s: %s" % (f, w))
    checks["no Pi-only part in the gateway code"] = not left
    bad = [k for k, v in checks.items() if not v]
    report("OS12 install files for both OSes (service / task, lid, battery 80 %, USB, firewall, updates); "
           "no Pi-only part left", not bad, "%d checks%s%s" % (len(checks), (", wrong: %s" % bad) if bad else "",
                                                         (", Pi left: %s" % left) if left else ""))


def main():
    for t in (os1, os2, os3, os4, os5, os6, os7, os8, os9, os10, os11, os12):
        try:
            t()
        except Exception as e:                       # a crash is a failure, never a skip
            import traceback
            traceback.print_exc()
            report("%s crashed" % t.__name__.upper(), False, repr(e))
    bad = results.count(False)
    print("OS TESTS: %d passed, %d failed" % (len(results) - bad, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
