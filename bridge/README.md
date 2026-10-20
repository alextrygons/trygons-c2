# USB joystick bridge (Teensy 4.1)

The bridge reads the USB joystick(s) and forwards them to the AURIX. That is
its **only** job: no Wi-Fi, no control logic. The AURIX checks every frame
(CRC, sequence counter, 150 ms timeout, devices connected, values in range).
If anything is wrong it declares a JOYSTICK FAULT, exactly like a broken
wire on the analog joystick.

```
 T.16000M stick ─┐
                 ├─ powered USB hub ── USB host cable ── Teensy 4.1 ── Serial1 (3.3 V UART) ── AURIX
 TWCS throttle ──┘                                     (5-pin host header)   pin 1 TX → AURIX RX
                                                                             pin 0 RX ← (unused)
```

* Frame: `app/jsframe.h`. The same C file is compiled into the bridge and
  into the AURIX, so the two sides can never disagree.
* Speed: 230400 baud, 8N1, one 57-byte frame every 10 ms (100 Hz).
* Device 0 = stick, device 1 = throttle. Thrustmaster T.16000M (044F:B10A)
  and TWCS (044F:B687) are recognised automatically. Any other standard USB
  HID joystick works after learn mode + calibration (below).
* Wiring and power: see `WIRING.md`, section 6.

## 1. Install (once)

1. In VS Code: **Extensions** (the four-squares icon on the left) → search
   **PlatformIO IDE** → **Install**. Wait until it says it is ready, then
   restart VS Code.
2. Windows only: install the Teensy USB driver/loader by installing
   **Teensyduino** from pjrc.com, or just let PlatformIO do it at the first
   upload (it brings `teensy_loader_cli`).

## 2. Open the project

1. **File → Open Folder…** → choose the `bridge` folder (not the whole repo).
   PlatformIO recognises it from `platformio.ini`.
2. The first time, PlatformIO downloads the Teensy toolchain. This takes a
   few minutes; the progress shows in the bottom bar.

## 3. Build and upload

1. Connect the Teensy to the PC with a normal micro-USB cable.
2. Click the **PlatformIO** (ant head) icon → `teensy41` → **Build**. It
   should end with `SUCCESS`.
3. Click **Upload**. If the Teensy Loader window asks, press the small white
   button on the Teensy once.

(Command line alternative: `pio run` and `pio run -t upload` inside `bridge/`.)

## 4. Learn mode: which axis / button is which

1. Keep the Teensy connected to the PC. Plug the hub (with its power supply)
   into the Teensy's USB host header, and the joystick(s) into the hub.
2. PlatformIO → `teensy41` → **Monitor** (115200 baud).
3. Type `?` + Enter: the bridge lists the connected devices.
4. Type `l` + Enter: learn mode ON. Now move things on the joystick. Each
   change is printed with its code, for example:

   ```
   dev0 (stick) button 1 PRESSED   (code 1)
   dev1 (throttle) hat LEFT   (code 76)
   dev1 (throttle) axis 2 Z:  +87.3 %   (axis code 10)
   ```

   These codes go into the AURIX button map. The ground station's
   **Map buttons** routine does it for you: it asks you to press each
   button and reads the code itself.
5. Type `l` again to leave learn mode. While learn mode is on, the AURIX
   keeps the USB joystick disarmed. This is intentional.

## 5. Calibrate (any joystick other than the T.16000M; recommended for the TWCS)

1. In the monitor type `c` + Enter.
2. Move **every** axis slowly to both ends: stick X/Y, twist, throttle lever
   all the way forward and back, rocker, mini-stick.
3. Type `c` again. The ranges are saved in the Teensy's EEPROM and replace
   the built-in table. An axis you did not move is sent as "invalid". The
   AURIX refuses a mapped axis that is invalid.
4. If stick and throttle came out the wrong way round (`?` shows dev0/dev1),
   type `w` to swap them (saved).
5. `r` erases all saved calibrations and swaps.

## 6. Check on the AURIX side

Ground station → **Joystick** tab:

* source "USB bridge";
* about 100 frames/s;
* both devices "connected";
* the stick and lever move live.

Power the boat up with the lever in the neutral zone and the stick centred,
otherwise you get JOYSTICK NOT IN NEUTRAL.

## ESP32-S3 alternative

The ESP32-S3 also has a USB host port, but USB host + hub + HID on the
ESP32-S3 Arduino core is much less mature than the Teensy's USBHost_t36.
This firmware is written for the Teensy 4.1 only. Porting means replacing
the USB part (`USBHost`, `USBHub`, `JoystickController`) with the ESP-IDF
`usb_host` + HID class driver. The frame code (`app/jsframe.c`) stays the
same. The ESP32-S3 port is **not done and not tested**.

## Safety notes

* The bridge has its own watchdog (0.5 s, library `WDT_T4`, downloaded by
  PlatformIO at the first build). A hung bridge stops sending, and the AURIX declares a fault
  after 150 ms.
* USB joysticks usually report only when something changes. The bridge
  repeats the last state every 10 ms. A device that freezes while still
  connected cannot be detected by the bridge (see MANUAL.txt, "What is
  still missing").
* The T.16000M and TWCS are **not waterproof**. Mount them in a dry console.
