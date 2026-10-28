"""The gateway's connection to the AURIX: a 3.3 V TTL USB-UART adapter on the
gateway PC (921600 baud; NEVER RS232: +-12 V destroys the AURIX pins), named
by a stable id in gateway.json (osdep.resolve_serial: "usb:VID:PID:SERIAL",
"/dev/serial/by-id/...", "COMn"), or, for the simulator, "tcp:127.0.0.1:5762".

Same frames as the USB link (groundstation/proto.py). Received frames go to
the handlers registered per frame type (and to "any" handlers, e.g. the
station server that forwards telemetry). Reconnects by itself.
"""
import asyncio
import threading
import time

from common import log, proto
import osdep


class AurixLink:
    def __init__(self, address, baud=921600):
        self.address = address
        self.baud = baud
        self.handlers = {}          # frame type -> [callback(frame)]
        self.any_handlers = []      # callback(frame) for every frame
        self.rx = proto.Receiver()
        self.seq = 0
        self.connected = False
        self.last_rx = 0.0
        self.frames_in = 0
        self.bytes_in = 0
        self.bytes_out = 0
        self._writer = None         # asyncio writer (TCP) or serial port
        self._serial = None
        self._loop = None
        self.error = ""             # last connect problem (e.g. "adapter not plugged in")

    def on(self, ftype, cb):
        self.handlers.setdefault(ftype, []).append(cb)

    def on_any(self, cb):
        self.any_handlers.append(cb)

    # --- sending ------------------------------------------------------------------
    def send(self, ftype, payload=b""):
        """Queue one frame; returns its sequence number (None if not connected)."""
        if not self.connected:
            return None
        self.seq = (self.seq + 1) & 0xFF
        data = proto.encode(ftype, self.seq, payload)
        self.bytes_out += len(data)
        try:
            if self._serial is not None:
                self._serial.write(data)
            elif self._writer is not None:
                self._writer.write(data)
        except Exception as e:                  # cable pulled while writing
            log.warning("AURIX write failed: %s", e)
        return self.seq

    # --- receiving ------------------------------------------------------------------
    def _feed(self, data):
        self.bytes_in += len(data)
        for f in self.rx.frames_from(data):
            self.frames_in += 1
            self.last_rx = time.time()          # "AURIX alive": only a frame that passed the CRC, never noise
            for cb in self.handlers.get(f.type, ()):
                try:
                    cb(f)
                except Exception:
                    log.exception("handler for frame 0x%02x", f.type)
            for cb in self.any_handlers:
                try:
                    cb(f)
                except Exception:
                    log.exception("frame listener")

    async def run(self):
        self._loop = asyncio.get_running_loop()
        while True:
            try:
                if self.address.startswith("tcp:"):
                    await self._run_tcp()
                else:
                    await self._run_serial()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                if str(e) != self.error:            # once per new problem, not every second
                    log.warning("AURIX link: %s", e)
                self.error = str(e)
            self.connected = False
            await asyncio.sleep(1.0)

    async def _run_tcp(self):
        _, host, port = self.address.split(":")
        reader, writer = await asyncio.open_connection(host, int(port))
        self._writer = writer
        self.connected, self.error = True, ""
        log.info("AURIX connected (%s)", self.address)
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    raise ConnectionError("AURIX connection closed")
                self._feed(data)
        finally:
            self._writer = None
            writer.close()

    async def _run_serial(self):
        import serial                               # pyserial
        dev = osdep.resolve_serial(self.address)    # again at every reconnect (re-plugged adapter)
        port = serial.Serial(dev, self.baud, timeout=0.05)
        self._serial = port
        self.connected, self.error = True, ""
        log.info("AURIX connected (%s = %s, %d baud)", self.address, dev, self.baud)
        stop = threading.Event()
        dead = asyncio.Event()

        def reader():
            try:
                while not stop.is_set():
                    data = port.read(4096)
                    if data:
                        self._loop.call_soon_threadsafe(self._feed, data)
            except Exception as e:
                log.warning("UART read: %s", e)
            self._loop.call_soon_threadsafe(dead.set)

        th = threading.Thread(target=reader, daemon=True)
        th.start()
        try:
            await dead.wait()
        finally:
            stop.set()
            self._serial = None
            port.close()
