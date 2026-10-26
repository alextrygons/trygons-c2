"""The boats this station can drive (CLAUDE.md sections F + L).

One JSON file per boat in <repo>/boats/ (in git, NO secrets):
    {"format": 2, "id": "rf49d", "name": "RF49D", "tailscale_ip": "100.101.102.103", "gateway_port": 5761,
     "generator_max_pct": 25, "relays": [...], "thrusters": {...}, "length_m": 4.9, "draft_m": 0.5}
A file without "format" is v1: it loads with defaults (no relays, generator_max_pct 0 = generator
disabled, no thrusters). A file that fails the checks below is INVALID: listed, never connected.
Unknown fields are kept in Boat.extra and never break the list (the format can grow).

Format 2 (section L + the owner clarifications 2026-10-26):
  generator_max_pct  0..100 % (0 / missing = generator disabled); the AURIX keeps its own ceiling
                     (parameter gen_max_pct, 30 %): the station pushes min(file, AURIX max)
  relays             [{"n", "name_el", "name_en", "type": onoff | momentary | latching | timed,
                       "group": engine | trim | extra, "pair": n | null, "max_s", "show_hud", "power_up"}]
                     pair must be mutual; momentary: 0 < max_s <= 5; timed: max_s > 0;
                     power_up off | on | keep - the ENGINE group only "off" (never switched automatically)
  thrusters          {"bow": {"max_pct", "run_s", "rest_s"}, "stern": {...}, "jog_m": 1.5, "spot_alarm_m": 15}
  length_m, draft_m  optional

A new computer = clone the repository: the boats come with it.

  load(folder)            -> [Boat] (sorted by name); a broken / invalid file is listed with its error
  Prober(boats).start()   -> background reachability check (TCP to the gateway port every
                             period_s): status(boat) = "online" / "offline" / "no_ip" / "invalid" / "?";
                             found(boat) = the station just connected to it (online now)
  check_welcome(boat, w)  -> None, or the reason to refuse the connection (the gateway's
                             own boat name must be the file's name)
  confirmed(name_typed, boat, camera_live) -> the selection may take HELM
  aurix_settings(boat, gen_max_ceiling) -> what the HELM station pushes to the AURIX on connect
"""
import ipaddress
import json
import os
import socket
import threading
import time

BOATS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "boats")

FORMAT = 2                              # the newest boat file format this station reads
BASE_KEYS = ("format", "id", "name", "tailscale_ip", "gateway_port")
RELAY_TYPES = ("onoff", "momentary", "latching", "timed")
RELAY_GROUPS = ("engine", "trim", "extra")
POWER_UP = ("off", "on", "keep")
MOMENTARY_MAX_S = 5.0                   # a board-timed pulse is never longer (section M)
RELAY_N_MAX = 32                        # relay channel numbers 1..32 (8-16 channel boards, TBD)
GEN_MAX_CEILING = 30.0                  # the AURIX's own ceiling for the generator (parameter gen_max_pct max)
THRUSTER_DEFAULT = {"max_pct": 0.0, "run_s": 0.0, "rest_s": 0.0}     # no thruster


def _num(v, lo, hi, what):
    """A finite number in lo..hi, else ValueError (bool is not a number here)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not (lo <= v <= hi):
        raise ValueError("%s must be a number %g..%g (got %r)" % (what, lo, hi, v))
    return float(v)


def _parse_relays(lst):
    if lst is None:
        return []
    if not isinstance(lst, list):
        raise ValueError("relays must be a list")
    out, seen = [], set()
    for i, r in enumerate(lst):
        if not isinstance(r, dict):
            raise ValueError("relay %d: not an object" % i)
        n = r.get("n")
        if isinstance(n, bool) or not isinstance(n, int) or not (1 <= n <= RELAY_N_MAX):
            raise ValueError("relay %d: n must be 1..%d" % (i, RELAY_N_MAX))
        if n in seen:
            raise ValueError("relay %d: n %d used twice" % (i, n))
        seen.add(n)
        name_el, name_en = str(r.get("name_el") or "").strip(), str(r.get("name_en") or "").strip()
        if not name_el or not name_en:
            raise ValueError("relay %d: name_el and name_en are needed" % n)
        typ, grp = r.get("type"), r.get("group")
        if typ not in RELAY_TYPES:
            raise ValueError("relay %d: type must be one of %s" % (n, ", ".join(RELAY_TYPES)))
        if grp not in RELAY_GROUPS:
            raise ValueError("relay %d: group must be one of %s" % (n, ", ".join(RELAY_GROUPS)))
        pair = r.get("pair")
        if pair is not None and (isinstance(pair, bool) or not isinstance(pair, int) or pair == n):
            raise ValueError("relay %d: pair must be another relay's n or null" % n)
        max_s = _num(r.get("max_s", 0), 0.0, 3600.0, "relay %d: max_s" % n)
        if typ == "momentary" and not (0.0 < max_s <= MOMENTARY_MAX_S):
            raise ValueError("relay %d: a momentary relay needs 0 < max_s <= %g" % (n, MOMENTARY_MAX_S))
        if typ == "timed" and max_s <= 0.0:
            raise ValueError("relay %d: a timed relay needs max_s > 0" % n)
        show = r.get("show_hud", False)
        if not isinstance(show, bool):
            raise ValueError("relay %d: show_hud must be true / false" % n)
        pu = r.get("power_up", "off")
        if pu not in POWER_UP:
            raise ValueError("relay %d: power_up must be one of %s" % (n, ", ".join(POWER_UP)))
        if grp == "engine" and pu != "off":
            # owner clarification: engine-group relays are never switched automatically
            raise ValueError("relay %d: an engine relay can't have power_up %r (only off)" % (n, pu))
        out.append({"n": n, "name_el": name_el, "name_en": name_en, "type": typ, "group": grp, "pair": pair,
                    "max_s": max_s, "show_hud": show, "power_up": pu})
    by_n = {r["n"]: r for r in out}
    for r in out:
        p = r["pair"]
        if p is not None and (p not in by_n or by_n[p]["pair"] != r["n"]):
            raise ValueError("relay %d: pair %s is not mutual" % (r["n"], p))
    return out


def _parse_thrusters(d):
    if d is None:
        d = {}
    if not isinstance(d, dict):
        raise ValueError("thrusters must be an object")
    out = {}
    for k in ("bow", "stern"):
        t = d.get(k)
        if t is None:
            out[k] = dict(THRUSTER_DEFAULT)
            continue
        if not isinstance(t, dict):
            raise ValueError("thrusters.%s must be an object" % k)
        out[k] = {"max_pct": _num(t.get("max_pct", 0), 0.0, 100.0, "thrusters.%s.max_pct" % k),
                  "run_s": _num(t.get("run_s", 0), 0.0, 3600.0, "thrusters.%s.run_s" % k),
                  "rest_s": _num(t.get("rest_s", 0), 0.0, 3600.0, "thrusters.%s.rest_s" % k)}
        if out[k]["max_pct"] > 0.0 and out[k]["run_s"] <= 0.0:
            raise ValueError("thrusters.%s: run_s > 0 is needed (the heat budget)" % k)
    out["jog_m"] = _num(d.get("jog_m", 1.5), 0.5, 10.0, "thrusters.jog_m")
    out["spot_alarm_m"] = _num(d.get("spot_alarm_m", 15), 5.0, 100.0, "thrusters.spot_alarm_m")
    return out


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
        self.extra = {k: v for k, v in d.items() if k not in BASE_KEYS and not k.startswith("_")}
        # format 2 fields: defaults first (a v1 file keeps them), then the file's values, checked
        self.format = 1
        self.generator_max_pct = 0.0
        self.relays = []
        self.thrusters = _parse_thrusters(None)
        self.length_m = None
        self.draft_m = None
        if error is None:
            try:
                self._parse_v2(d)
            except ValueError as e:
                self.error = str(e)
                self.format, self.generator_max_pct, self.relays = 1, 0.0, []
                self.thrusters = _parse_thrusters(None)

    def _parse_v2(self, d):
        fmt = d.get("format", 1)
        if isinstance(fmt, bool) or not isinstance(fmt, int) or not (1 <= fmt <= FORMAT):
            raise ValueError("format must be 1..%d (got %r)" % (FORMAT, fmt))
        self.format = fmt
        for k in ("length_m", "draft_m"):
            if d.get(k) is not None:
                setattr(self, k, _num(d[k], 0.1, 100.0, k))
        if fmt < 2:
            return                              # v1: the format-2 fields stay at their defaults
        self.generator_max_pct = _num(d.get("generator_max_pct", 0), 0.0, 100.0, "generator_max_pct")
        self.relays = _parse_relays(d.get("relays"))
        self.thrusters = _parse_thrusters(d.get("thrusters"))

    def ip_ok(self):
        """A real address (the placeholder "PLACEHOLDER-..." of a fresh file is not)."""
        try:
            ipaddress.ip_address(self.ip)
            return True
        except ValueError:
            return False

    def fixed_status(self):
        """The status that needs no probing: "invalid" (bad file), "no_ip" (placeholder address), else None."""
        if self.error:
            return "invalid"
        if not self.ip_ok():
            return "no_ip"
        return None

    def address(self):
        """The ground station's connection text for this boat."""
        return "gw:%s:%d" % (self.ip, self.port)

    def relay(self, n):
        for r in self.relays:
            if r["n"] == n:
                return r
        return None

    def __repr__(self):
        return "Boat(%s, %s:%d, v%d)" % (self.name, self.ip, self.port, self.format)


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
    """"online" (the gateway port answers), "offline", "no_ip" (placeholder / bad address), "invalid" (bad file)."""
    st = boat.fixed_status()
    if st:
        return st
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
                r = probe(b, self.timeout)
                if any(x is b for x in self.boats):      # not replaced by found() meanwhile
                    self.result[b.id] = (r, time.time())
            end = time.time() + self.period
            while self.running and time.time() < end:
                time.sleep(0.2)

    def status(self, boat):
        return boat.fixed_status() or self.result.get(boat.id, ("?", 0.0))[0]

    def found(self, boat):
        """The station just connected to this boat's gateway: it is found now (and checked from now on)."""
        # the connected boat replaces a list entry with the same id (else the next round would probe the
        # old entry's address and overwrite this result)
        self.boats = [b for b in self.boats if b.id != boat.id] + [boat]
        self.result[boat.id] = ("online", time.time())


def check_welcome(boat, welcome):
    """The gateway's own boat name must be the file's: else the connection is refused."""
    got = (welcome or {}).get("boat")
    if str(got) != str(boat.name):
        return "wrong boat: expected %s, the gateway is %s" % (boat.name, got)
    return None


def confirmed(name_typed, boat, camera_live):
    """Selecting a boat: its name typed back (case and spaces ignored) AND its live camera seen."""
    return bool(camera_live) and (name_typed or "").replace(" ", "").upper() == boat.name.replace(" ", "").upper()


def aurix_settings(boat, gen_max_ceiling=GEN_MAX_CEILING):
    """What the HELM station pushes to the AURIX after connecting (RAM only, section L): the generator
    cap (parameter gen_max_pct = min(file, the AURIX's own ceiling)), the relay table (PROTO_T_RELAY_CFG,
    section M - step 6) and the thruster settings (section P - step 9). A v1 / invalid file = the
    defaults: generator disabled, no relays, no thrusters."""
    gen = min(float(boat.generator_max_pct), float(gen_max_ceiling)) if not boat.error else 0.0
    return {"gen_max_pct": gen,
            "relays": [dict(r) for r in boat.relays] if not boat.error else [],
            "thrusters": json.loads(json.dumps(boat.thrusters)) if not boat.error else _parse_thrusters(None)}
