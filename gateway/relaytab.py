"""The relay tables the gateway has seen (CLAUDE.md O): which relay of the boat is in which group.

A copilot's relay switch (PROTO_T_RELAY) goes straight to the AURIX for a trim / extra relay and needs
the captain's ACCEPT for an engine-group relay. The frame only carries the relay number, so the gateway
remembers every relay table (PROTO_T_RELAY_CFG) a station sent, by its CRC-16 - the same hash the
AURIX shows in TELEM2 group M for its ACTIVE table. The groups are only used when the AURIX reports
that hash as active; an unknown hash / no table / an unknown relay = engine = the captain's ACCEPT
(fail safe). Kept on the SSD (state/relay_tables.json) so a gateway restart still knows the table the
AURIX loaded from its data flash.
"""
import json
import os
import struct

import proto

KEEP = 8                    # tables remembered (the newest)


class RelayTables:
    def __init__(self, path=None):
        self.path = path
        self.tables = {}                    # hash -> {n: group}  (insertion order = age)
        self.details = {}                   # hash -> {n: {"type", "group", "pair", "power_up"}} (UAV hatch, U3)
        if path:
            try:
                with open(path, encoding="utf-8") as f:
                    d = json.load(f)
                for h, g in (d.get("tables") or {}).items():
                    grp = {int(n): str(v) for n, v in g.items() if str(v) in proto.RELAY_GROUPS}
                    self.tables[int(h)] = grp
                for h, g in (d.get("details") or {}).items():
                    self.details[int(h)] = {int(n): dict(v) for n, v in g.items() if isinstance(v, dict)}
            except (OSError, ValueError, TypeError, AttributeError):
                self.tables = {}

    def learn(self, payload):
        """A PROTO_T_RELAY_CFG payload a station sends -> its hash (None if unreadable)."""
        g = proto.relay_groups(payload)
        if g is None:
            return None
        h = proto.crc16(payload)
        det = details_of(payload)
        if self.tables.get(h) == g and self.details.get(h) == det:
            return h
        self.tables.pop(h, None)
        self.tables[h] = g
        self.details[h] = det
        while len(self.tables) > KEEP:
            self.details.pop(next(iter(self.tables)), None)
            self.tables.pop(next(iter(self.tables)))
        self._save()
        return h

    def active(self, telem2):
        """{n: group} of the table active on the AURIX (TELEM2 group M), None if not known."""
        if not telem2 or not telem2.get("relay_table") or telem2.get("relay_hash") is None:
            return None
        return self.tables.get(int(telem2["relay_hash"]))

    def detail(self, telem2, n):
        """{"type", "group", "pair", "power_up"} of relay n in the table ACTIVE on the AURIX, None if not known."""
        if self.active(telem2) is None:
            return None
        return (self.details.get(int(telem2["relay_hash"])) or {}).get(int(n))

    def _save(self):
        if not self.path:
            return
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"tables": {str(h): {str(n): v for n, v in g.items()} for h, g in self.tables.items()},
                           "details": {str(h): {str(n): v for n, v in g.items()} for h, g in self.details.items()}},
                          f)
            os.replace(tmp, self.path)
        except OSError:
            pass


def details_of(payload):
    """{n: {"type", "group", "pair", "power_up"}} of a PROTO_T_RELAY_CFG payload (proto.p_relay_cfg), {} if broken."""
    out = {}
    try:
        for i in range(payload[0]):
            n, ty, grp, pair, _ms, fl, role = struct.unpack_from("<BBBBHBB", payload, 1 + 8 * i)
            out[n] = {"type": proto.RELAY_TYPES[ty] if ty < len(proto.RELAY_TYPES) else "?",
                      "group": "engine" if role else (proto.RELAY_GROUPS[grp] if grp < len(proto.RELAY_GROUPS)
                                                      else "?"),
                      "pair": pair or None, "power_up": "on" if fl & 2 else ("keep" if fl & 4 else "off")}
    except (IndexError, struct.error):
        return {}
    return out
