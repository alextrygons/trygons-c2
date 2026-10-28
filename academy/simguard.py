"""The academy's ONLY door to the outside (CLAUDE.md section S, ACADEMY_SPEC ground rules).

Every academy connection or recording goes through this module; test_academy.py proves that
no other academy module imports a network / serial module or a ground-station / gateway module
that leads to one (AC1, AC2) and that nothing connects while the modules load (AC3).

Allowed (and nothing else):
  * the simulator on THIS PC: host 127.0.0.1 / localhost / ::1 and a port of SIM_PORTS
      5760 = autopilot_sim server, USB link (the boat's simulated AURIX)
      5761 = the demo gateway's station port (demo/demo.py, listening on 127.0.0.1 only)
    and, after connecting to a gateway port, the welcome's boat name must be a SIM_BOAT_NAMES name
    and NOT a boat of boats/ (a real gateway tunnelled to localhost is refused by its name);
  * recorded files (RECORDING_EXT) that are ordinary files - never a serial device, a COM port,
    a pipe or a "tcp:" address.
Refused: every other address (LAN, Tailscale 100.64.0.0/10, the IPs of boats/*.json, host names),
every other port (5762 = the simulated AURIX's gateway UART: the academy never plays gateway),
and the whole academy on a gateway PC (the gateway's config file exists: spec J).
"""
import ipaddress
import json
import os
import socket                      # only here (test_academy AC1)

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
BOATS_DIR = os.path.join(REPO, "boats")

SIM_PORTS = {5760: "sim", 5761: "demo gateway"}
GATEWAY_PORTS = {5761}                         # a welcome name check is needed on these
SIM_BOAT_NAMES = {"trygons-demo"}              # demo/demo.py gateway config "boat"
LOOPBACK_NAMES = {"localhost"}
RECORDING_EXT = (".tlg", ".csv", ".jsonl", ".json", ".mp4", ".mkv", ".wav", ".log", ".txt")
TAILSCALE = ipaddress.ip_network("100.64.0.0/10")


class Refused(Exception):
    """The academy may not use this target. str() = the English reason (logs / audit)."""


def boat_files(boats_dir=None):
    """(names, ips) of every boat of boats/ (placeholders included)."""
    names, ips = set(), set()
    d = boats_dir or BOATS_DIR
    try:
        files = sorted(f for f in os.listdir(d) if f.endswith(".json"))
    except OSError:
        files = []
    for f in files:
        try:
            with open(os.path.join(d, f), encoding="utf-8") as fh:
                b = json.load(fh)
        except (OSError, ValueError):
            continue
        for k in ("name", "id"):
            if isinstance(b.get(k), str):
                names.add(b[k].strip().lower())
        if isinstance(b.get("tailscale_ip"), str):
            ips.add(b["tailscale_ip"].strip())
    return names, ips


def gateway_config_paths(env=None):
    """Where a gateway PC keeps its config (gateway/osdep.py config_dir, both OSes)."""
    env = os.environ if env is None else env
    win = os.path.join(env.get("PROGRAMDATA") or r"C:\ProgramData", "trygons", "gateway.json")
    return [win, "/etc/trygons/gateway.json"]


def running_on_gateway(env=None):
    """True on a boat's gateway PC: the academy never runs there (spec J)."""
    return any(os.path.isfile(p) for p in gateway_config_paths(env))


def check_target(host, port, boats_dir=None, env=None):
    """Raise Refused unless (host, port) is the simulator on this PC. Returns the SIM_PORTS name."""
    if running_on_gateway(env):
        raise Refused("academy never runs on a gateway PC")
    if not isinstance(host, str) or not host.strip():
        raise Refused("no host")
    h = host.strip().lower()
    _names, ips = boat_files(boats_dir)
    if host.strip() in ips:
        raise Refused("boat address %s (boats/)" % host)
    if h not in LOOPBACK_NAMES:
        try:
            ip = ipaddress.ip_address(h.strip("[]"))
        except ValueError:
            raise Refused("host name %s: only the simulator on this PC (127.0.0.1)" % host)
        if ip.version == 4 and ip in TAILSCALE:
            raise Refused("Tailscale address %s: a boat network" % host)
        if not ip.is_loopback:
            raise Refused("address %s is not this PC: only the simulator on this PC" % host)
    if not isinstance(port, int) or port not in SIM_PORTS:
        raise Refused("port %r is not a simulator port %s" % (port, sorted(SIM_PORTS)))
    return SIM_PORTS[port]


def check_welcome_name(name, boats_dir=None):
    """A gateway's welcome boat name: only the demo / sim gateway (never a boats/ boat)."""
    n = (name or "").strip().lower()
    names, _ips = boat_files(boats_dir)
    if n in names:
        raise Refused("gateway of the real boat %s" % name)
    if n not in SIM_BOAT_NAMES:
        raise Refused("gateway %r is not the simulator / demo" % name)
    return n


def connect(host, port, timeout=3.0, boats_dir=None):
    """The only way an academy module opens a TCP connection: checked first, then connected.
    A gateway port still needs check_welcome_name() on its welcome before anything is sent."""
    check_target(host, port, boats_dir)
    return socket.create_connection((host, port), timeout=timeout)


def check_recording(path, env=None):
    """Raise Refused unless path is an ordinary recorded file. Returns the absolute path."""
    if running_on_gateway(env):
        raise Refused("academy never runs on a gateway PC")
    p = str(path or "").strip()
    low = p.lower()
    if not p:
        raise Refused("no file")
    if (low.startswith(("tcp:", "udp:", "usb:", "/dev/", "\\\\.\\", "//./")) or
            (low.startswith("com") and low[3:].rstrip(":").isdigit())):
        raise Refused("%s is a device / address, not a recorded file" % p)
    if not low.endswith(RECORDING_EXT):
        raise Refused("%s is not a recording file type" % p)
    if not os.path.isfile(p):
        raise Refused("%s is not an ordinary file" % p)
    return os.path.abspath(p)
