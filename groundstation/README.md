# Trygons C2 ground station

A PC program that shows the boat live and sends commands. It connects to the
**real board** (USB cable, virtual COM port) or to the **simulator** (TCP).

The boat always decides: every command is checked by the same rules as the
buttons and answered with OK or a reason ("JOYSTICK IS IN CONTROL", "only in
STANDBY", ...). **The PC can never take control from the joystick.**

## 1. Install (once)

You need Python 3 with tkinter, plus pyserial.

**Option A: MSYS2 (already installed on this PC).** Open the *MSYS2 UCRT64*
terminal and run:

```
pacman -S mingw-w64-ucrt-x86_64-python mingw-w64-ucrt-x86_64-tk mingw-w64-ucrt-x86_64-python-pyserial
```

Then Python is `C:\msys64\ucrt64\bin\python.exe`.

**Option B: python.org.** Install Python 3 from https://www.python.org/downloads/
and tick "Add python.exe to PATH" (tkinter is included). Then, in a
terminal:

```
pip install -r requirements.txt
```

## 2. Start

### With the simulator (no board needed)

Terminal 1, which starts the simulated boat:

```
cd C:\trygons\TRYGONS_C2\sim
build_sim.bat
autopilot_sim.exe server
```

(`autopilot_sim.exe server 5760 5` runs 5× faster than real time.)

Terminal 2, which starts the ground station:

```
cd C:\trygons\TRYGONS_C2\groundstation
C:\msys64\ucrt64\bin\python.exe app.py tcp:127.0.0.1:5760
```

### With the board

1. Plug in the board's USB cable.
2. Find the COM port in Windows Device Manager → "Ports (COM & LPT)".
3. Run:

```
C:\msys64\ucrt64\bin\python.exe app.py COM5
```

You can also start `app.py` without an argument, choose the port in the
drop-down list and press **Connect**.

## 3. Using it

| Area | What it does |
|------|--------------|
| Top bar | connection, link status, name of the log file |
| Orange banner | **JOYSTICK IN CONTROL** (MANUAL). The PC can't change the mode now; centre the stick and press joystick AUTO or panel HDG LOCK |
| Yellow banner | **PRESS JOYSTICK CONFIRM** with a countdown: the boat waits for the joystick CONFIRM button (AUTO start / resume) |
| Yellow banner (USB lever) | **MOVE THROTTLE TO PICK UP: lever to xx %** - after a takeover the lever does nothing until it reaches that point (or the neutral zone) |
| Red banner + beep | active alarms (also "NO DATA FROM THE BOAT" if the link drops). "ACK ALARM" works in STANDBY |
| Left column | mode, heading, target, yaw rate, outboard angle, throttle, gear, GPS, position source, uncertainty, current, active waypoint ... |
| Middle column | **STANDBY**, the three driving modes (the active one is lit; MANUAL = joystick, HDG LOCK = panel button, AUTO START = PC + CONFIRM), AUTO pause / resume / skip, heading ±1/±10 (HDG LOCK), sea state, training |
| Joystick strip | live stick position, lever bar (neutral zone grey, pickup point yellow), TAKE / AUTO / CONFIRM, status + fault reason, source and USB link (frames/s, CRC errors, stick / throttle connected), analog calibration |
| Joystick tab | source (analog / USB bridge), throttle input (stick Y / lever), neutral zone, lever override, pickup window, the button map with live "now pressed", **Map buttons (guided)** (a button already used is refused), **SAVE TO BOAT** (STANDBY only) |
| Map tab | boat, track, the program (numbered), active waypoint, uncertainty circle, drift arrow. Tick "Click adds a program waypoint" to build a program by clicking. Offline |
| Program tab | the AUTO program editor: add / change / delete / reorder waypoints (lat, lon, leg speed, arrival radius, action continue / slow / stop), final action, Upload to boat, Download, Save / Load file. The boat checks the program again and says what is wrong |
| Plot tab | heading vs target and the outboard angle, last 2 minutes |
| Config tab | "Read all" shows the settings. Select one, type a value and press "Write selected" (STANDBY only, range-checked, lost at power-off) |
| Sound tab | sound / voice settings, the voices found, every sound and callout to play in Greek or English |
| Events tab | every command, answer, mode change and alarm |
| Driver screen | full-screen front camera with the HUD on top (MANUAL.txt chapter 5B) |
| Map window | a full-size map for another monitor, view only (chapter 5C) |
| ΕΛ / EN | screen language, Greek by default, switches live (chapter 5A; texts in `lang/`) |
| Simulator box | (only with the simulator) panel HDG LOCK / STANDBY and joystick CONFIRM buttons; USB joystick lever, stick, buttons, unplug and link-fault controls (used when the source is the USB bridge; or start the simulator with `server 5760 1 usb`) |

**Starting AUTO:** make a program, "Upload to boat", press **AUTO START**, then
press **CONFIRM** on the joystick within 10 s (stick centred).

The boat raises the alarm **"PC link lost"** if it hears nothing from the
PC for 3 s. It keeps its current mode: losing the PC never stops or
starts anything.

## 4. Logs and replay

Every frame (telemetry and commands) is written to
`logs/log_YYYYMMDD_HHMMSS.jsonl`, one JSON object per line.

```
python replay.py logs\log_....jsonl --sim          # re-send the commands to the simulator, compare modes
python replay.py logs\log_....jsonl --view         # play the telemetry back, then connect app.py to tcp:127.0.0.1:5761
```

## 5. Boat logs (SD card) and the log tools

The boat logs everything on its microSD card (`.TLG` files; the simulator
writes them to `sim/sdcard`). The ground station also saves its own copy of
the telemetry as `.TLG` next to the `.jsonl` file.

```
python logview.py B00012_001.TLG                 # plots, map, events (click = jump)
python log2csv.py B00012_001.TLG                 # -> CSV (all fields), Excel
python log2csv.py B00012_001.TLG -f heading,target,thr_out --events
python log2csv.py B00012_001.TLG --list          # the field names
python tlgreplay.py B00012_001.TLG               # replay the drive in the simulator
```

## 6. Test

```
python test_e2e.py
```

This starts the simulator, uploads an AUTO program from the ground station
(and checks a bad one is refused), starts it with CONFIRM, lets a simulated
joystick take over, checks that the PC is refused, and checks the log.
Then, with a second simulator: switches to the USB joystick, checks a map
with TAKE = AUTO is refused, takes over with the lever (throttle pickup) and
unplugs the throttle (JOYSTICK FAULT). 10 tests.

## Files

* `proto.py`: frame format and messages. Must match `app/proto.h` and `app/link.c`.
* `link.py`: serial / TCP connection, heartbeat, logger.
* `app.py`: the window.
* `program.py`: AUTO programs (upload, download, save / load).
* `replay.py`: replay of the ground station's .jsonl logs.
* `test_e2e.py`: end-to-end test.
* `i18n.py`, `lang/el.txt`, `lang/en.txt`: every screen text (Greek / English); `settings.py`: per-PC settings.
* `driver.py` + `hud.py`: the driver screen (camera HUD); `hudar.py`: AR overlays; `mapwin.py`: the map window;
  `mapview.py`: chart layers; `monitors.py`: window -> monitor; `alerts.py`: alarm levels + MASTER lamps.
* `sound.py`, `tts.py`, `soundtest.py`: sounds, voice callouts (Piper, offline), the Sound tab.
* `test_driver.py`, `test_lang.py`, `test_sound.py`, `test_charts.py`: tests of the screens, languages, sounds, maps.
* `tlg.py`: read / write .TLG logs; `log2csv.py`, `logview.py`, `tlgreplay.py`: the log tools;
  `test_logs.py`: their tests.
