# Trygons C2 – main project handover (state 25 Oct 2026)

Upload this file to the Claude project together with CLAUDE.md, MANUAL.txt, WIRING.md, TEST_REPORT.md (section 0), PARTS spreadsheet and the Academy/Simulator handover. The repository (github.com/alextrygons/trygons-c2) and its CLAUDE.md are the source of truth; this file is the summary for chats.

## 1. What it is
A command-and-control (C2) system for a small outboard boat in the Saronic Gulf (Attica): autopilot, manual/remote driving, cameras, AIS, internet links, shore ground station. Manned tests first; the boat always has a person on board able to take over.

## 2. Architecture
- **AURIX TC375 lite kit** (KITA2GTC375LITETOBO1): all control and safety. Core 0 = control loop 20 Hz; CPU1 = independent safety monitor; core 2 = logger. Watchdogs on. Built with AURIX Development Studio (TASKING).
- **Sensors/actuators:** BNO085 IMU (compass, gyro, accel/slamming), u-blox MAX-M10S GPS, Pololu G2 18v17 steering driver (no angle feedback; angle estimated, TRAINING calibrates gap/speed/end stops), limit switches, neutral switch, throttle + gear PWM servos (via 74AHCT125), panel buttons, buzzer, microSD (mikroBUS), e-stop.
- **Joystick:** analog or Thrustmaster T.16000M + TWCS via Teensy 4.1 USB-host bridge (UART). Joystick always wins.
- **Gateway (on board):** fanless Intel N150 mini PC (replaces the planned Raspberry Pi): VPN (Tailscale), users/roles/HELM, internet channels (Teltonika RUT901 4G, RF, Starlink = emergency only), 4 Tapo C310 cameras (FRONT-1/2, REAR-LEFT/RIGHT) via MediaMTX, ring recording + SAVE, sync to shore (4G/RF only), AIS receive (dAISy), eWaSR obstacle vision, collision avoidance proposals. Never controls outputs directly – only validated requests to the AURIX.
- **Shore ground station (Python):** driver screen (camera HUD), captain screen, separate map window; two/three monitors; Greek default.
- **PC simulation:** whole system runs without hardware (START_DEMO.bat); acceptance tests (tests\acceptance\RUN_ACCEPTANCE.bat).

## 3. Modes and key safety rules (details in CLAUDE.md)
- Modes: STANDBY, MANUAL (joystick), HEADING LOCK, AUTO (waypoint program), REMOTE MANUAL (shore joystick, 20 %/5 % cap), TRAINING (outboard calibration).
- Priority: STANDBY button > onboard joystick > remote joystick > autopilot. PC/captain never override the joystick or STANDBY.
- Startup power limit 30 % fwd / 5 % rev, never saved; raise = PC command + code + onboard joystick CONFIRM within 10 s, one step at a time; lowering instant.
- Gear only at idle (idle → neutral → gear → throttle). Every fault: idle + neutral first; steering held (joystick or gyro) until then.
- Captain actions affecting driving need driver-screen ACCEPT (enforced in the gateway; single-operator mode exists).
- Collision: AUTO = automatic slow down + pass astern; HEADING LOCK = proposal; MANUAL = alarm only. Shallow-water check on all routes. Vision never steers and never adds throttle.
- **FIRST_TEST** is ON in git: steering only (throttle/gear held at idle/neutral), only STANDBY/TRAINING/HDG LOCK.

## 4. Status
- Software: all features built and tested in simulation; full acceptance test (2000 random fault runs, 24 h soak, invariants I1–I21) passed; all findings fixed except R7 (documented design decision).
- Board build: ADS build clean (0 errors, 0 warnings); memory OK (CPU1 < 60 %, shared data uncached; LMU block 2 at 84 % – watch it).
- Real board drivers written but NOT yet tested on hardware. Bench test tab + FIRST_TEST.md checklist ready.
- Running / next in Claude Code: driver HUD redesign (full-screen camera, aircraft-style overlays, AR, sounds/voice, Greek/English, map window, Greek manual). Then the Academy (see Academy handover).

## 5. Next steps
1. Order Stage 1 parts (spreadsheet), then bench test → dock test → first steering-only water test (FIRST_TEST.md).
2. Hardware-in-the-loop test: real AURIX fed by the PC simulator before the water.
3. After first tests: MOB button, geofence, battery/engine monitoring, water-in-hull sensor; later virtual anchor and lost-link return-home.
4. Fix the 3 flaky camera-vision timing tests (pass alone, fail in the full gate).
5. Gateway software: make it platform-independent for the N150 mini PC (x86 Debian/Ubuntu, USB AIS receiver, USB-TTL UART to the AURIX).

## 6. Things to measure / decide
- Outboard draft (leg down), steering motor voltage/current, boat 12 V vs 24 V.
- Tapo C310 adapter voltage and the cameras' own video delay.
- Teltonika RUT901 RutOS version; Starlink model.
- Home marina/ramp location (for the simulator and the practical test).

## 7. Working conventions
- Every Claude Code prompt in a code box; before it: "type /clear, press Enter, then paste". Interrupted: "continue from PROGRESS.md" (no /clear).
- One commit per section, tests must pass, then git push. Never commit secrets (auto.crt, keys, secrets.json).
- Python on this PC: C:\msys64\ucrt64\bin\python.exe. ADS: F5 → Project → Clean → Build after pulling.
- Claude Pro: check /usage before big prompts; weekly limit is the real constraint.
