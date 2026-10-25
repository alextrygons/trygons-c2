"""The boats this station can drive (CLAUDE.md section F).

One JSON file per boat in <repo>/boats/ (in git, NO secrets):
    {"id": "rf49d", "name": "RF49D", "tailscale_ip": "100.101.102.103", "gateway_port": 5761, ...}
More fields can be added later (length_m, draft_m, outboard, cameras, limits):
they are kept in Boat.extra and never break the list.

A new computer = clone the repository: the boats come with it.

  load(folder)            -> [Boat] (sorted by name); a broken file is listed with its error
  Prober(boats).start()   -> background reachability check (TCP to the gateway port every
                             period_s): status(boat) = "online" / "offline" / "no_ip" / "?"
  check_welcome(boat, w)  -> None, or the reason to refuse the connection (the gateway's
                             own boat name must be the file's name)
  confirmed(name_typed, boat, camera_live) -> the selection may take HELM
"""
import ipaddress
import json
import os
import socket
import threading
import time

BOATS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "boats")


class Boat:
    def __init__(self, path, data=None, error=None):
        self.path = path
        d = data or {}
        self.id = str(d.get("id") or os.path.splitext(os.path.basename(path))[0])
        self.name = str(d.get("name") or self.id)
        self.ip = str(d.get("tailscale_ip") or "")
        try:
            self.port = int(d.get("gateway_port") or 5761)
        except (TypeError, ValueError):
            self.port = 5761
        self.error = error
        self.extra = {k: v for k, v in d.items() if k not in ("id", "name", "tailscale_ip", "gateway_port")
                      and not k.startswith("_")}

    def ip_ok(self):
        """A real address (the placeholder "PLACEHOLDER-..." of a fresh file is not)."""
        try:
            ipaddress.ip_address(self.ip)
            return True
        except ValueError:
            return False

    def address(self):
        """The ground station's connection text for this boat."""
        return "gw:%s:%d" % (self.ip, self.port)

    def __repr__(self):
        return "Boat(%s, %s:%d)" % (self.name, self.ip, self.port)


def load(folder=None):
    folder = folder or BOATS_DIR
    out = []
    if not os.path.isdir(folder):
        return out
    for n in sorted(os.listdir(folder)):
        if not n.endswith(".json"):
            continue
        p = os.path.join(folder, n)
        try:
            with open(p, encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("not a JSON object")
            out.append(Boat(p, data))
        except (OSError, ValueError) as e:
            out.append(Boat(p, None, error=str(e)))
    return sorted(out, key=lambda b: b.name)


def probe(boat, timeout=1.5):
    """"online" (the gateway port answers), "offline", "no_ip" (placeholder / bad address)."""
    if boat.error or not boat.ip_ok():
        return "no_ip"
    try:
        socket.create_connection((boat.ip, boat.port), timeout=timeout).close()
        return "online"
    except OSError:
        return "offline"


class Prober:
    """Checks every boat in the background; never blocks the screen."""

    def __init__(self, boats, period_s=10.0, timeout=1.5):
        self.boats = list(boats)
        self.period = period_s
        self.timeout = timeout
        self.result = {}            # id -> (status, time)
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def stop(self):
        self.running = False

    def _run(self):
        while self.running:
            for b in list(self.boats):
                self.result[b.id] = (probe(b, self.timeout), time.time())
            end = time.time() + self.period
            while self.running and time.time() < end:
                time.sleep(0.2)

    def status(self, boat):
        return self.result.get(boat.id, ("?", 0.0))[0]


def check_welcome(boat, welcome):
    """The gateway's own boat name must be the file's: else the connection is refused."""
    got = (welcome or {}).get("boat")
    if str(got) != str(boat.name):
        return "wrong boat: expected %s, the gateway is %s" % (boat.name, got)
    return None


def confirmed(name_typed, boat, camera_live):
    """Selecting a boat: its name typed back (case and spaces ignored) AND its live camera seen."""
    return bool(camera_live) and (name_typed or "").replace(" ", "").upper() == boat.name.replace(" ", "").upper()
