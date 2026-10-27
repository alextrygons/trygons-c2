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

import proto

KEEP = 8                    # tables remembered (the newest)


class RelayTables:
    def __init__(self, path=None):
        self.path = path
        self.tables = {}                    # hash -> {n: group}  (insertion order = age)
        if path:
            try:
                with open(path, encoding="utf-8") as f:
                    d = json.load(f)
                for h, g in (d.get("tables") or {}).items():
                    grp = {int(n): str(v) for n, v in g.items() if str(v) in proto.RELAY_GROUPS}
                    self.tables[int(h)] = grp
            except (OSError, ValueError, TypeError, AttributeError):
                self.tables = {}

    def learn(self, payload):
        """A PROTO_T_RELAY_CFG payload a station sends -> its hash (None if unreadable)."""
        g = proto.relay_groups(payload)
        if g is None:
            return None
        h = proto.crc16(payload)
        if self.tables.get(h) == g:
            return h
        self.tables.pop(h, None)
        self.tables[h] = g
        while len(self.tables) > KEEP:
            self.tables.pop(next(iter(self.tables)))
        self._save()
        return h

    def active(self, telem2):
        """{n: group} of the table active on the AURIX (TELEM2 group M), None if not known."""
        if not telem2 or not telem2.get("relay_table") or telem2.get("relay_hash") is None:
            return None
        return self.tables.get(int(telem2["relay_hash"]))

    def _save(self):
        if not self.path:
            return
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"tables": {str(h): {str(n): v for n, v in g.items()} for h, g in self.tables.items()}}, f)
            os.replace(tmp, self.path)
        except OSError:
            pass
