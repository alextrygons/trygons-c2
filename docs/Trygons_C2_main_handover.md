# Trygons C2 – main project handover (state 11 Nov 2026, HEAD e502095 + CL4)

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

## 4. Status (11 Nov 2026)
- Software: all features built and tested in simulation. Done since the 25 Oct state (details: CLAUDE.md sections, PROGRESS.md rows): DroneCAN actuators (11d-11g, 13b-13k), crew profiles / two screens / MANEUVER / UNCREWED HDG LOCK / shore stick override (13n-13q), UAV magazine (U1-U6, WA), talking manual (AM1-AM6), SAR tools (PF1-PF6: MOB, search patterns, mark from video, ADS-B, demo), trend panel (TR1-TR4), acceleration telemetry (FW-TR / FW-TR-r, firmware), cleanup CL1-CL4 (stale texts, stray-process guard tools/strays.py, flaky-test harness fixes CL2 / CL2-b / CL2-c, archive of old PROGRESS tasks, unused lang keys).
- Board build: ADS build clean at the last gate; NOT yet tested on hardware (FIRST_TEST.md). FIRST_TEST is ON in git.
- Gate: tools/check.sh (now needs MORE than 45 min in one run, the gateway suite alone is ~25 min; split proposed in docs/cleanup_report.md CL4) + tests/acceptance/RUN_ACCEPTANCE.bat.

## 5. Open items / next steps
Owner:
1. W84 (recorded questions for the talking manual - AS17 needs them), W89 (question list review), W97 (firewall command), W99 (blind answer review); W132 (tools/get_readsb.py needs the owner's --sha256).
2. Tour voice clips pending for the MOB / trends stops (voice_pending in academy/content/tour.json); other stale tour lines listed in docs/cleanup_report.md (CL1).
3. DEMO_SAR.bat live click-through has not been run by a person.
4. Order hardware (PARTS.md), bench test, dock test, first steering-only water test (FIRST_TEST.md; section 6 = UAV / loudhailer bench checks).
Tests known open: SR8 (needs the NVENC environment), AS17 (needs W84), LH2 flaky under load (documented, CL2-c).
Firmware awaiting the next flash-gate deep review: FW-TR / FW-TR-r (hal_read_accel3, ACCEL frame 0x8F, tools/stackcheck.py POLLED_ONLY). Review notes (C:/trygons/private/reviews/review_c522e77.txt), recorded in PROGRESS.md): MED-A stackcheck POLLED_ONLY should also fail when reachable from core1 / core2 trees; MED-B the proof depends on GCC inlining and an iLLD assumption; LOW-1 hal_read_accel_z / accel3 call order grep; LOW-2 AX6 should assert TELEM2 survives; LOW-3 AX4 popen may pass when the child never ran; LOW-4 AX5 log header margin only partly proven.
Cleanup: unused-key list, dead-file list and the check.sh split proposal = docs/cleanup_report.md (CL4).

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
