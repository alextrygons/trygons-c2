"""Everything that differs between the gateway PC's two operating systems
(Ubuntu 24.04 LTS = recommended at sea, Windows 11) - in ONE place, so the
rest of the gateway stays OS-neutral.

  data_dir() / config_dir()   /var/lib/trygons, /etc/trygons  |  %PROGRAMDATA%\\trygons
  resolve_serial(spec)        a STABLE serial device name -> the device to open:
      "tcp:HOST:PORT"                    the simulator (unchanged)
      "usb:VID:PID" / "usb:VID:PID:SERIAL"  the USB adapter by its hardware id
                                         (hex VID / PID, "*" = any) - both OSes
      "/dev/serial/by-id/usb-..."        Ubuntu: the udev link (never changes)
      "COM7"                             Windows: works, but the number can change
      "/dev/ttyUSB0", "/dev/ttyACM0"     Ubuntu: works, but the number can change
  list_serial()               every serial port with its stable names (--list-serial)
  check_config(cfg)           config problems: [(level, text)], level "error" / "warning"
  install_stop(loop, cb)      Ctrl+C / SIGTERM (Ubuntu, systemd) / Ctrl+Break (Windows)
  cpu_temp_c(), throttle_count()   for the vision resource guard (None = not known)

The AURIX link is ALWAYS a 3.3 V TTL USB-UART adapter (FTDI / CP2102 set to
3.3 V). NEVER an RS232 port or cable: its +-12 V destroys the AURIX pins.
"""
import glob
import os
import re
import signal
import sys

WIN = os.name == "nt"

# the Raspberry Pi's own UART names: the gateway PC has none (step 12)
PI_UARTS = ("/dev/ttyAMA0", "/dev/ttyAMA10", "/dev/serial0", "/dev/serial1", "/dev/ttyS0")
# USB-serial chips mostly sold inside RS232 (+-12 V) DB9 cables - a warning, never a block
RS232_TYPICAL = {(0x067B, 0x2303): "Prolific PL2303"}


class SerialSpecError(Exception):
    """The configured serial device can't be used (text = what to do)."""


def is_windows(platform=None):
    return (platform or sys.platform).startswith("win")


# --- folders -----------------------------------------------------------------------------

def data_dir(platform=None, env=None):
    env = os.environ if env is None else env
    if is_windows(platform):
        import ntpath                              # "\\" also under MSYS2's Python
        return ntpath.join(env.get("PROGRAMDATA") or r"C:\ProgramData", "trygons")
    return "/var/lib/trygons"


def config_dir(platform=None, env=None):
    if is_windows(platform):
        return data_dir(platform, env)
    return "/etc/trygons"


# --- serial devices ----------------------------------------------------------------------

_USB = re.compile(r"^usb:([0-9a-fA-F]{1,4}|\*):([0-9a-fA-F]{1,4}|\*)(?::(.+))?$")
_COM = re.compile(r"^(?:\\\\\.\\)?COM([0-9]+)$", re.I)


def parse_usb(spec):
    """'usb:0403:6001:A50285BI' -> (0x0403, 0x6001, 'A50285BI'); None = wildcard."""
    m = _USB.match(spec or "")
    if not m:
        return None
    vid = None if m.group(1) == "*" else int(m.group(1), 16)
    pid = None if m.group(2) == "*" else int(m.group(2), 16)
    return vid, pid, m.group(3)


def comports():
    """pyserial's port list (imported only when needed: the tests pass fakes)."""
    from serial.tools import list_ports        # pyserial
    return list(list_ports.comports())


def usb_spec(p):
    """The stable 'usb:...' name of a port object (None = not a USB port)."""
    vid, pid = getattr(p, "vid", None), getattr(p, "pid", None)
    if vid is None or pid is None:
        return None
    sn = getattr(p, "serial_number", None)
    return "usb:%04x:%04x" % (vid, pid) + (":%s" % sn if sn else "")


def resolve_serial(spec, ports=None, platform=None, exists=None, realpath=None):
    """The device to open for a configured name, or SerialSpecError.
    Called again at every (re)connect: a re-plugged adapter is found again."""
    exists = exists or os.path.exists
    realpath = realpath or os.path.realpath
    spec = (spec or "").strip()
    if not spec:
        raise SerialSpecError("not set in gateway.json (python gateway.py --list-serial shows the names)")
    if spec.startswith("tcp:"):
        return spec
    u = parse_usb(spec)
    if u is not None:
        vid, pid, sn = u
        found = []
        for p in (comports() if ports is None else ports):
            if getattr(p, "vid", None) is None:
                continue
            if vid is not None and p.vid != vid:
                continue
            if pid is not None and p.pid != pid:
                continue
            if sn is not None and (getattr(p, "serial_number", None) or "") != sn:
                continue
            found.append(p.device)
        if not found:
            raise SerialSpecError("%s: not plugged in (no USB adapter with this id)" % spec)
        if len(found) > 1:
            raise SerialSpecError("%s: %d adapters match (%s) - add the serial number: usb:VID:PID:SERIAL"
                                  % (spec, len(found), ", ".join(sorted(found))))
        return found[0]
    if spec.startswith("/dev/serial/by-id/") or spec.startswith("/dev/serial/by-path/"):
        if is_windows(platform):
            raise SerialSpecError("%s is an Ubuntu name: on Windows use usb:VID:PID:SERIAL or COMn" % spec)
        if not exists(spec):
            raise SerialSpecError("%s: not plugged in" % spec)
        return realpath(spec)
    if _COM.match(spec):
        if not is_windows(platform):
            raise SerialSpecError("%s is a Windows name: on Ubuntu use /dev/serial/by-id/... or usb:VID:PID" % spec)
        return spec
    if spec.startswith("/dev/"):
        if is_windows(platform):
            raise SerialSpecError("%s is an Ubuntu name: on Windows use usb:VID:PID:SERIAL or COMn" % spec)
        return spec
    raise SerialSpecError("%s: unknown serial device name (use usb:VID:PID[:SERIAL], /dev/serial/by-id/..., COMn)"
                          % spec)


def serial_warnings(spec, what, platform=None):
    """Warnings for a name that works but is not stable / not for the gateway PC."""
    spec = (spec or "").strip()
    out = []
    if spec.startswith("usb:VID") or "PUT-" in spec:
        out.append(("error", "%s = %s is the placeholder of config.example.json: put the real name "
                             "(python gateway.py --list-serial)" % (what, spec)))
    elif spec in PI_UARTS:
        out.append(("error", "%s = %s is a Raspberry Pi UART: the gateway PC has none - use the USB adapter "
                             "(usb:VID:PID:SERIAL or /dev/serial/by-id/...)" % (what, spec)))
    elif parse_usb(spec) is not None and parse_usb(spec)[2] is None:
        out.append(("warning", "%s = %s has no serial number: two adapters of the same kind can't be told "
                               "apart - add it (usb:VID:PID:SERIAL, python gateway.py --list-serial)" % (what, spec)))
    elif _COM.match(spec) or re.match(r"^/dev/tty(USB|ACM)[0-9]+$", spec):
        out.append(("warning", "%s = %s: the number can change when the adapter is re-plugged - use "
                               "usb:VID:PID:SERIAL%s" % (what, spec, "" if is_windows(platform)
                                                         else " or /dev/serial/by-id/...")))
    return out


def byid_links(platform=None, pattern="/dev/serial/by-id/*", realpath=None):
    """Ubuntu: {device: [by-id link, ...]} (empty on Windows)."""
    if is_windows(platform):
        return {}
    realpath = realpath or os.path.realpath
    out = {}
    for link in sorted(glob.glob(pattern)):
        out.setdefault(realpath(link), []).append(link)
    return out


def list_serial(ports=None, links=None, platform=None):
    """Lines for --list-serial: every port and the stable names to put in gateway.json."""
    ports = comports() if ports is None else ports
    links = byid_links(platform) if links is None else links
    lines = []
    for p in sorted(ports, key=lambda q: q.device):
        names = list(links.get(p.device, []))
        u = usb_spec(p)
        if u:
            names.append(u)
        lines.append("%-14s %s" % (p.device, getattr(p, "description", "") or ""))
        for n in names:
            lines.append("    use: %s" % n)
        if (getattr(p, "vid", None), getattr(p, "pid", None)) in RS232_TYPICAL:
            lines.append("    CAUTION: %s chips are mostly in RS232 (+-12 V) cables - the AURIX needs a 3.3 V TTL "
                         "adapter" % RS232_TYPICAL[(p.vid, p.pid)])
    if not lines:
        lines.append("no serial ports found (plug the USB adapter in)")
    lines.append("AURIX: ONLY a 3.3 V TTL USB-UART adapter (FTDI / CP2102 at 3.3 V). NEVER RS232: +-12 V destroys "
                 "the AURIX pins.")
    return lines


# --- configuration check -----------------------------------------------------------------

def check_config(cfg, platform=None):
    """[(level, text)]: "error" = that part can't work (the rest of the gateway still
    runs: every part keeps running when another one fails), "warning" = works, but fix it."""
    out = []
    a = cfg.get("aurix")
    if not isinstance(a, str) or not a.strip():
        out.append(("error", "aurix: not set - the 3.3 V USB-UART adapter (python gateway.py --list-serial)"))
    else:
        w = serial_warnings(a, "aurix", platform)
        out += w
        if not _has_error(w):
            try:
                _syntax_only(a, platform)
            except SerialSpecError as e:
                out.append(("error", "aurix: %s" % e))
    b = cfg.get("baud")
    if not isinstance(b, int) or isinstance(b, bool) or not 1200 <= b <= 3000000:
        out.append(("error", "baud: %r is not a serial speed (the AURIX uses 921600)" % (b,)))
    elif b != 921600:
        out.append(("warning", "baud %d: the AURIX gateway UART runs at 921600" % b))
    ac = cfg.get("ais") or {}
    if ac.get("enabled"):
        s = ac.get("source")
        if not isinstance(s, str) or not s.strip():
            out.append(("error", "ais.source: not set - the USB AIS receiver (python gateway.py --list-serial), "
                                 "or \"enabled\": false"))
        else:
            w = serial_warnings(s, "ais.source", platform)
            out += w
            if not _has_error(w):
                try:
                    _syntax_only(s, platform)
                except SerialSpecError as e:
                    out.append(("error", "ais.source: %s" % e))
            if not s.startswith("tcp:") and not _has_error(w) and isinstance(a, str) and s.strip() == a.strip():
                out.append(("error", "ais.source and aurix are the same device (%s)" % s))
        ab = ac.get("baud", 38400)
        if not isinstance(ab, int) or isinstance(ab, bool) or not 1200 <= ab <= 3000000:
            out.append(("error", "ais.baud: %r is not a serial speed (AIS receivers: 38400)" % (ab,)))
    st = cfg.get("storage") or {}
    if not isinstance(st.get("root"), str) or not st.get("root"):
        out.append(("error", "storage.root: not set"))
    elif is_windows(platform) and st["root"].startswith("/"):
        out.append(("error", "storage.root %s is an Ubuntu path on Windows" % st["root"]))
    elif not is_windows(platform) and re.match(r"^[A-Za-z]:\\", st["root"]):
        out.append(("error", "storage.root %s is a Windows path on Ubuntu" % st["root"]))
    if cfg.get("set_clock") and is_windows(platform):
        out.append(("warning", "set_clock: Windows keeps a GPS time offset instead of setting the clock "
                               "(logs and video still use GPS time)"))
    return out


def _has_error(items):
    return any(level == "error" for level, _ in items)


def _syntax_only(spec, platform):
    """Check a name's form without looking at the plugged-in hardware."""
    if spec.startswith("tcp:"):
        parts = spec.split(":")
        if len(parts) != 3 or not parts[2].isdigit():
            raise SerialSpecError("%s: use tcp:HOST:PORT" % spec)
        return
    if parse_usb(spec) is not None:
        return
    resolve_serial(spec, ports=[], platform=platform, exists=lambda p: True, realpath=lambda p: p)


# --- signals -----------------------------------------------------------------------------

def install_stop(loop, cb, platform=None, sig=signal):
    """A clean stop on Ctrl+C / SIGTERM (systemd stop) on Ubuntu and on Ctrl+Break
    on Windows (Ctrl+C there = KeyboardInterrupt, handled by main). A hard kill
    (Task Scheduler "End", power loss) is safe too: everything on disk is
    written so that a cut costs at most the last record. Returns the signals used."""
    used = []
    if not is_windows(platform):
        for s in (sig.SIGINT, sig.SIGTERM):
            loop.add_signal_handler(s, cb)
            used.append(s)
    else:
        brk = getattr(sig, "SIGBREAK", None)
        if brk is not None:
            sig.signal(brk, lambda *_: loop.call_soon_threadsafe(cb))
            used.append(brk)
    return used


# --- temperature / throttling (vision resource guard) ---------------------------------------

def cpu_temp_c(platform=None, root="/sys/class/thermal"):
    """The hottest CPU / package thermal zone in deg C (Ubuntu); None on Windows
    (no reading without admin rights / extra programs) or when there is none."""
    if is_windows(platform):
        return None
    best = None
    for z in glob.glob(os.path.join(root, "thermal_zone*")):
        try:
            with open(os.path.join(z, "temp")) as f:
                t = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
        if 0.0 < t < 150.0 and (best is None or t > best):
            best = t
    return best


def throttle_count(platform=None, root="/sys/devices/system/cpu"):
    """Sum of the Intel thermal-throttle counters (Ubuntu); None when unknown."""
    if is_windows(platform):
        return None
    n, seen = 0, False
    for f in glob.glob(os.path.join(root, "cpu*", "thermal_throttle", "*_throttle_count")):
        try:
            with open(f) as fh:
                n += int(fh.read().strip())
                seen = True
        except (OSError, ValueError):
            continue
    return n if seen else None
