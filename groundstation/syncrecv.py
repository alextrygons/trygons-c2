"""Shore receiver for the boat's sync (section G) - runs on the shore PC.

    python syncrecv.py                       uses sync_receiver.json (next to this file)
    python syncrecv.py --root D:\\trygons_sync --port 5770 --boat trygons-c2 --key <key>

sync_receiver.json (NOT in git - it holds the key, the same as sync_key in
the Pi's /etc/trygons/secrets.json):
    {"root": "D:\\\\trygons_sync", "port": 5770, "listen": "0.0.0.0",
     "boats": {"trygons-c2": "<long random key>"}}
The ground station starts it by itself when that file exists.

Files land in <root>/<boat>/ with the boat's folder layout
(aurix_logs/..., audit/..., video/events/...), so the log viewer's
"Open event..." works on them. A file is first written as <file>.part and
only renamed when its SHA-256 matches; the boat marks it synced only after
that answer. Listen only on the VPN (Tailscale) address or behind a firewall
that lets in only the VPN.
"""
import argparse
import asyncio
import hashlib
import hmac
import json
import os
import secrets
import shutil
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "sync_receiver.json")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


class SyncReceiver:
    def __init__(self, root, boats, port=5770, listen="0.0.0.0"):
        self.root = root
        self.boats = {b: k.encode() for b, k in boats.items()}
        self.port = port
        self.listen = listen
        self.files = 0
        self.bytes = 0
        self.bad = 0
        self.refused = 0
        self.last = None
        self.connected = {}
        self.loop = None
        self.server = None
        self.fail_done = False                  # tests: answer "bad" once

    def _path(self, boat, rel):
        rel = rel.replace("\\", "/")
        if rel.startswith("/") or ".." in rel.split("/") or ":" in rel:
            raise ValueError("bad path %r" % rel)
        p = os.path.join(self.root, boat, *rel.split("/"))
        os.makedirs(os.path.dirname(p), exist_ok=True)
        return p

    async def _client(self, reader, writer):
        def send(obj):
            writer.write(json.dumps(obj).encode() + b"\n")
        boat = None
        try:
            nonce = secrets.token_hex(16)
            send({"t": "challenge", "nonce": nonce})
            a = json.loads(await asyncio.wait_for(reader.readline(), 10))
            key = self.boats.get(a.get("boat"))
            good = key and hmac.compare_digest(hmac.new(key, nonce.encode(), hashlib.sha256).hexdigest(),
                                               a.get("mac", ""))
            if not good:
                self.refused += 1
                send({"t": "refused", "why": "unknown boat / wrong key"})
                await writer.drain()
                return
            boat = a["boat"]
            self.connected[boat] = time.time()
            send({"t": "welcome"})
            while True:
                line = await reader.readline()
                if not line:
                    return
                m = json.loads(line)
                t = m.get("t")
                if t == "ping":
                    send({"t": "pong", "ts": m["ts"]})
                elif t == "stat":
                    p = self._path(boat, m["path"])
                    if os.path.exists(p + ".part"):
                        send({"t": "have", "path": m["path"], "size": os.path.getsize(p + ".part"),
                              "sha": sha256_file(p + ".part"), "complete": False})
                    elif os.path.exists(p):
                        send({"t": "have", "path": m["path"], "size": os.path.getsize(p), "sha": sha256_file(p),
                              "complete": True})
                    else:
                        send({"t": "have", "path": m["path"], "size": 0})
                elif t == "data":
                    data = await reader.readexactly(m["len"])
                    p = self._path(boat, m["path"])
                    part = p + ".part"
                    if m["off"] == 0:
                        open(part, "wb").close()
                    elif not os.path.exists(part) and os.path.exists(p):
                        os.replace(p, part)                     # a growing file: continue the copy
                    have = os.path.getsize(part) if os.path.exists(part) else -1
                    if have != m["off"]:
                        send({"t": "bad", "path": m["path"], "why": "offset %d, have %d" % (m["off"], have)})
                        continue
                    with open(part, "ab") as f:
                        f.write(data)
                    self.bytes += len(data)
                elif t == "done":
                    p = self._path(boat, m["path"])
                    part = p + ".part"
                    if not os.path.exists(part) and os.path.exists(p):
                        part = p                                # already complete (nothing new was sent)
                    ok = (os.path.exists(part) and os.path.getsize(part) == m["size"] and
                          sha256_file(part) == m["sha"] and not self.fail_done)
                    if ok:
                        if part != p:
                            os.replace(part, p)
                        self.files += 1
                        self.last = m["path"]
                        send({"t": "ok", "path": m["path"], "sha": m["sha"]})
                    else:
                        self.fail_done = False
                        self.bad += 1
                        if os.path.exists(part) and part != p:
                            os.remove(part)                     # start this file again
                        send({"t": "bad", "path": m["path"], "why": "checksum / size"})
                await writer.drain()
        except (OSError, ValueError, KeyError, asyncio.TimeoutError, asyncio.IncompleteReadError):
            pass
        finally:
            if boat:
                self.connected.pop(boat, None)
            writer.close()

    async def serve(self):
        self.server = await asyncio.start_server(self._client, self.listen, self.port)
        async with self.server:
            await self.server.serve_forever()

    def start(self):
        """In a thread (the ground station / tests)."""
        self.loop = asyncio.new_event_loop()

        def run():
            try:
                self.loop.run_until_complete(self.serve())
            except (asyncio.CancelledError, RuntimeError):
                pass
        threading.Thread(target=run, daemon=True).start()
        for _ in range(50):
            if self.server:
                break
            time.sleep(0.05)
        return self

    def stop(self):
        if self.loop and self.server:
            self.loop.call_soon_threadsafe(self.server.close)

    def status(self):
        free = shutil.disk_usage(self.root).free / 1e9 if os.path.isdir(self.root) else None
        return {"files": self.files, "mb": round(self.bytes / 1e6, 1), "bad": self.bad, "refused": self.refused,
                "last": self.last, "boats_connected": sorted(self.connected), "root": self.root,
                "free_gb": round(free, 1) if free is not None else None}


def load_config():
    try:
        with open(CONFIG, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description="Trygons shore sync receiver")
    ap.add_argument("--root")
    ap.add_argument("--port", type=int)
    ap.add_argument("--boat")
    ap.add_argument("--key")
    a = ap.parse_args()
    c = load_config() or {}
    root = a.root or c.get("root")
    boats = dict(c.get("boats", {}))
    if a.boat and a.key:
        boats[a.boat] = a.key
    if not root or not boats:
        print("set root and a boat key (sync_receiver.json or --root/--boat/--key)")
        return 2
    os.makedirs(root, exist_ok=True)
    r = SyncReceiver(root, boats, a.port or c.get("port", 5770), c.get("listen", "0.0.0.0"))
    print("sync receiver on port %d -> %s" % (r.port, root))
    try:
        asyncio.run(r.serve())
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
