# Trygons C2 — boat autopilot
- Target: Infineon KIT_A2G_TC375_LITE (AURIX TC375, TriCore). Built with AURIX Development Studio (TASKING).
- No hardware yet: all logic must also compile and run on PC with gcc for simulation.

## Architecture
- app/  — autopilot logic, pure C99, NO hardware or iLLD includes. Uses only hal.h.
- hal/hal.h — hardware interface (heading, rudder angle, rudder motor, buttons, time).
- hal/hal_tc375.c — real implementation with iLLD (written later, when the board arrives).
- sim/  — PC simulation: boat model + hal_sim.c + main. Excluded from the ADS build.
  `autopilot_sim server` = simulated boat on TCP 5760 for the ground station.
- groundstation/ — PC ground station (Python + tkinter). Frame format in app/proto.h,
  telemetry/commands in app/link.c; groundstation/proto.py must match them.
- Cpu0_Main.c — core 0: starts the small-boat controller (smallboat.c) by default; `#define BOAT_VARIANT_BIG` selects the big-boat autopilot.
- Cpu1_Main.c — CPU1: the independent safety monitor (app/monitor.c), nothing else.
- Cpu2_Main.c — CPU2: the logger (SD card).
- bridge/ — Teensy 4.1 USB joystick bridge firmware (PlatformIO, C++). Excluded from the ADS build. Frame format app/jsframe.c/.h is shared (compiled into both).
- Joystick sources: app/joystick.c is the one joystick interface; analog (joy_step) or USB bridge (app/joybridge.c) fill the same joy_input_t. Source + button/axis map: app/joymap.c (NV slot 4, CRC, STANDBY only).
- app/servo.c — the ONLY code that writes the throttle servo (power-limit clamp). app/powerlim.c — power limit.
- Logging: app/logcore0.c (core 0 producer, fields in app/logfields.h) → app/logq.c (lock-free queues) → app/logger.c (core 2, SD card, black box). File format app/logrec.h; PC tools groundstation/tlg.py, log2csv.py, logview.py, tlgreplay.py.
- tools/mapreport.py — memory report (ADS map, GCC elf, or `--objects` = expected memory per core before a build); FAILS if CPU1 RAM > 60 %. tools/check.sh — the test gate (sim all + bridge, TriCore compile, HAL/servo checks, memory, gateway tests); run before every commit.
- Black box (app/logger.c): 60 s = 1200 x 184 B, split in 4 chunks (DSPR2 + cpu0/1/2_dlmu LMU); NEVER in CPU1's DSPR1 (only the safety monitor lives there).
- PARTS.md — the one shopping list (boat, gateway, cameras, sensors, AIS, shore). Keep it updated with every hardware change.
- gateway/ — Raspberry Pi 5 gateway (Python, systemd services). The AURIX is NEVER on the network: it talks only to the Pi over a UART (ASCLIN1 P15.0/P15.1, 921600, same frames + CRC as the USB link, app/link.c port LINK_PORT_GW). The Pi keeps the full AURIX log copy (LOG_CHUNK, dropped first when the UART is busy, never control traffic), sets its clock from GPS (TIME), sends GW_STATUS at 1 Hz (silent 3 s → GATEWAY LOST alarm, no mode change). Sim: `autopilot_sim server` puts the gateway UART on port + 2; tests: gateway/test_gateway.py.
- AIS (section A, gateway/ais.py): RECEIVE ONLY (dAISy HAT / USB receiver, NMEA AIVDM); decoded on the Pi, NEVER on the AURIX. Targets stale 3 min (A) / 6 min (B), removed 15 min; AIS RECEIVER SILENT (GW_STATUS flag → alarm, advisory). Optional online feed (AISHub) only on sync-allowed channels, never STARLINK. Optional second GPS (AIVDO / GNSS of the AIS device) → GW_GPS2 → dr.c check gps2_check_m (off by default).
- Charts (section B): tools/chartprep.py (EMODnet DTM via ERDDAP CSV, GEBCO .asc gap fill, OpenSeaMap via Overpass) → charts/<area>/ (not in git); groundstation/chart.py reads them (depth = shallowest cell around a point, no data = shallow), mapview.py draws the layers. Always the note "Not for navigation - free data, may be inaccurate near the coast" + attribution. Works offline.
- Navigation request (GW_NAVREQ, section C/D): the gateway decides (shallow water, collision avoidance), the AURIX validates and applies ONLY in AUTO (speed cap = reduction only, avoidance heading) or as a one-shot HDG LOCK heading after the driver's ACCEPT; MANUAL / REMOTE: alarm only. Lease 3 s: lost in AUTO while steering / slowing → hold heading, the lower of nav_lost_kn (4) and the last cap, DRIVER DECISION alarm. Joystick / STANDBY win. Shallow limit = max(3 m, draft + margin) (gateway shallow.py); AUTO programs / AUTO start through the gateway refused across or near shallow water (code 114); no data = shallow.
- Collision avoidance (section D, gateway/avoid.py + Gateway.nav_compose): CPA/TCPA per AIS target, warn 0.5 nm / 10 min, danger 0.25 nm / 6 min (COLLISION RISK + SAVE). AUTO: slow → re-check → route astern (starboard first, never ahead of a bow, safe_nm, depth-checked) → rejoin when the way back is safe; HDG LOCK: proposal + driver ACCEPT (avoid_answer, only driver screens / single-op) incl. the return; MANUAL / REMOTE: alarm only. Unsolvable (≥ 2 dangers, turning target, stale data, no safe route) → DRIVER DECISION: min_kn, keep course, never an unchecked route. Every decision in the audit log (kind "avoid"). Tests: gateway/test_avoid.py (planner) + test_gateway.py section CA (end-to-end). AIS doesn't show small craft: the docs must keep saying so.
- Section E: simulated AIS traffic = gateway/simais.py (AisSim, traffic() mix of Class A / B, CLI for `autopilot_sim server`). Ground station: AIS tab (list joined with CPA / TCPA, details on click), collision line on the driver HUD, HDG LOCK proposal as ACCEPT / REJECT on the driver screen (app.drv_proposal → avoid_answer); tests test_driver.py I7, test_avoid.py E1.
- Demo (no hardware): START_DEMO.bat / STOP_DEMO.bat / DEMO_SHIP.bat -> demo/demo.py (sim `server 5760 1 usb origin=37.82,23.60`, gateway with demo_run/gateway.json, demo/demo_world.py = 4 test cameras + AIS, ground stations SHORE + ON BOARD). Output in demo_run/ (gitignored). External programs are found by gateway/toolpaths.py (MediaMTX in tools/bin/ via tools/get_mediamtx.py, ffmpeg) - never hard-code temp or PC-specific paths; tools/check.sh runs from any clone.
- Vision (Part 2, gateway/vision/): eWaSR (Apache-2.0) ewasr_resnet18 ONNX in its OWN process (worker.py: IDLE / nice 19, limited cores + threads, newest frame only), manager.py on the gateway (health ACTIVE / PAUSED / DEGRADED / OFFLINE = notices, never alarms or mode changes; persistence filter; stages). RULES: vision NEVER steers and NEVER increases throttle; its only output to the AURIX is a speed cap (navreq reason 8, alarm 8 = SB_ALM_OBSTACLE advisory) applied only in AUTO; STAGE 3 (cap) is OFF by default and switching it on needs the driver screen's ACCEPT (stations.ask_local); a held cap is never released silently (driver CLEAR SLOWDOWN). BNO085 attitude (hal_read_attitude, TELEM2 group H) is only for the camera horizon, never for control. Tests: test_gateway VI (fake source) + VR (real model), sim NV7/NV8, test_driver I8.
- Acceptance test (Part 4): tests/acceptance/ (RUN_ACCEPTANCE.bat quick / full): fresh_build.py (clone from GitHub, every suite), storm/storm.c (C fault storm on app/ + sim/: invariants I1-I13, I18-I20 after every control step, --replay SEED, --selftest, --wrap, --soak, --dayatsea), storm_gateway.py (I14, I15, I17, I20, I21), stations_storm.py (I16), soak_system.py, day_at_sea.py, coverage_run.py. Outputs in test_results/ (gitignored); results in TEST_REPORT.md. Acceptance tests never change app/, hal/, gateway or groundstation logic - findings are reported with a reproduction.
- Libraries/ and Configurations/ are generated by ADS — never modify.
- ADS compiles every C/C++ file in the project folder: .cproject excludes everything that is not board firmware (only Cpu*_Main.c, app/, hal/, Libraries/, Configurations/ are built), in all 4 configurations. Any new top-level folder must be excluded from the ADS build in .cproject. tests/acceptance/check_ads_excludes.py checks it (tools/check.sh + RUN_ACCEPTANCE).
- tools/bin/ (MediaMTX, the vision Python with onnxruntime) is downloaded, never in git (.gitignore): recreate with python tools/get_mediamtx.py and python tools/get_vision_python.py.

## Rules
- Floats OK (TC375 has FPU). No malloc. Fixed control loop rate (e.g. 20 Hz).
- Safety first: rudder angle limits, rate limits, sensor-timeout -> standby + alarm.
- The joystick always wins over the autopilot and the PC. Only a centered joystick (+ AUTO / HDG LOCK button, or PC + CONFIRM) leaves MANUAL.
- A held stick must always be able to take over: HDG LOCK, AUTO start / resume, REMOTE and TRAINING are refused while the onboard stick is not centred (joy.off_centre, also while it is ignored after STANDBY; the USB lever doesn't count - its pickup keeps the throttle idle) → PC NACK 8 "centre the joystick", HDG LOCK button: refused + chirp.
- The STANDBY panel button always wins (every mode; the PC can never override it).
- Joystick, panel buttons and link wiring: see WIRING.md (section 6: USB bridge).
- Tests run with BOTH joystick sources: `autopilot_sim all` and `autopilot_sim all --bridge` must both pass.

## Small-boat features
Outboard boat (small-boat variant, `AP_STEER_OUTBOARD`). The big-boat variant (rudder feedback) must keep working unchanged.

Actuators
- Steering motor without angle feedback (angle estimated from motor run time, backlash compensated).
- Throttle servo (0–100 %).
- Controls: joystick (X steering, Y throttle, TAKE / AUTO / CONFIRM), panel buttons STANDBY, HDG LOCK, -1, +1, -10, +10.
- Or USB joystick (Thrustmaster T.16000M stick + TWCS throttle via the Teensy bridge): non-returning lever, neutral zone 45–55 %, maps to the CURRENT power limit (a raised limit never makes the throttle jump), throttle pickup on takeover, lever override (10 % from the hand-back position), button map with no shared TAKE/AUTO/CONFIRM.
- Gear servo F / N / R, positioned by PWM pulse width (configurable positions).

Sensors
- Compass (heading), gyro (yaw rate; roll rate if available).
- GPS via UART: lat, lon, SOG, COG, fix, timestamp, receiver spoof/jam flag.
- Optional: speed sensor (paddlewheel), neutral switch, steering motor current, steering limit switches.
- app/dr.c: position estimate — GPS when trusted, dead reckoning otherwise (learned current + throttle→speed table, uncertainty). Every fix passes an integrity check (spoof flag, jump, speed, course, slow drag); a rejected fix counts as GPS failure.
  - A fix with NaN / infinite / out-of-range numbers (dr_fix_valid: lat ±90, lon ±180, SOG 0..100 kn, COG 0..360) is a rejected fix too (DR_WHY_INVALID, recovers like after a suspect fix) and its numbers are never used (sb gps.last keeps the previous values). Second GPS frames out of range are ignored (gps2.bad); non-finite DR inputs (heading, yaw, water speed, throttle) are not used.

Modes — three driving modes + STANDBY + TRAINING
- STANDBY: idle, neutral, steering off. Power-up and after every fault.
- MANUAL (joystick): X = outboard angle, Y = throttle fwd/rev via the gear state machine. Entered from ANY mode by X deflection (> 100 ms), Y deflection (not in HDG LOCK) or TAKE. Left only with the stick centred.
- HDG LOCK: panel button locks the current heading, ±1/±10 adjust (buttons or PC); throttle stays on joystick Y; joystick X → MANUAL.
- AUTO: waypoint program (app/program.c, ≤ 50 waypoints, validated, stored with checksum) from the PC; start / resume = PC command + joystick CONFIRM within 10 s; pause / resume / skip from the PC; joystick → MANUAL, program suspended.
- TRAINING: outboard calibration, only from STANDBY by the PC.
- REMOTE MANUAL (MODE_REMOTE): the shore PC's joystick through the gateway (someone always on board). Enabled only by the HELM station's request + joystick CONFIRM on board; from STANDBY / HDG LOCK / AUTO, never from MANUAL. Priority: STANDBY button > onboard joystick > remote joystick > autopilot. Cap 20 % fwd / 5 % rev (never above the power limit), steering rate-limited, lever neutral zone + pickup. Data older than 300 ms / gap > 300 ms / latency > 400 ms → idle + neutral + gyro hold + alarm, STANDBY after 5 s, never resumes by itself. Ends on release, onboard joystick, STANDBY, any fault, HELM taken on board.
- HELM (section B): only the HELM station's commands are carried out (AURIX checks it, link.c); a station on the boat can always take HELM; remote HELM lost → REMOTE LINK LOST, HELM to nobody, no mode change.
- Driver confirmation (fix F1): captain actions that change the driving (proto.DRIVING_CMDS, never STANDBY) are held by the GATEWAY (stations.py) and only forwarded after the DRIVER screen's ACCEPT; no driver screen open → refused "driver screen not open"; REJECT / 10 s timeout / driver screen lost (1 s heartbeat) → refused. SINGLE-OPERATOR mode (captain screen, warning, banners on both screens, audited) → the captain's own confirmation instead; off when that station disconnects.
- CONFIRM (joystick) only answers a pending PC request, only with the stick centred and the joystick armed; it never changes the mode on its own.
- One mode manager owns all transitions (explicit allowed-transition table, every transition logged). Nothing else changes the mode.

Safety rules — never break them
- Gear change only at idle: throttle idle → wait → neutral → wait → new gear → wait → throttle.
- Every fault / timeout / STANDBY → throttle idle + neutral FIRST, then steering off.
  - Until idle + neutral the boat is still steered: from MANUAL the onboard stick keeps steering (also a held, disarmed stick: joy.steer_x, STOP_STEER_STICK); from REMOTE the remote stick is ignored at once and the gyro holds; joystick fault → gyro hold (no gyro: centre). AUTO / HDG LOCK: heading hold (gyro if the compass is gone).
  - STANDBY has its own, faster throttle closing rate (throttle.rate_stop_pct_s, parameter stop_rate_pc_s, default 100 %/s = idle within 1 s); normal driving and a planned arrival stop (WHY_ARRIVED) keep rate_down (40 %/s).
  - Compass lost while moving (AUTO / TRAINING): keep steering on the gyro (target yaw rate 0) until throttle is idle and gear is neutral, then steering off. If the gyro fails too: drive the outboard to its estimated centre, then off.
  - Compass lost in HDG LOCK: gyro hold + alarm, throttle stays with the user; once idle + neutral: steering off, STANDBY.
  - Compass lost in MANUAL → alarm only, stay in MANUAL (manual steering doesn't need the compass).
- GPS lost / suspect in AUTO: alarm, continue by dead reckoning at reduced speed; uncertainty > limit (200 m) or DR > 10 min → AUTO holds the current heading, slow, alarm. Never declare arrival while the uncertainty is larger than the arrival radius.
- After an integrity alarm, GPS is trusted again only after N consistent fixes that agree with dead reckoning; then blend smoothly (no steering jump).
  - Along the heading only (never across), DR's speed error may be allowed during that recovery - but ONLY with an independent check: the second GPS agrees (recover_along_frac 25 % of the distance run / water speed) or a paddlewheel (5 %). Then ~60 s after a spoof at speed; without either the old rule (can take minutes at speed: a slow along-course drag can't be told from a speed-table error). Test T3.
- Throttle changes are rate-limited.
- Reverse power is limited.
- Power limit: every power-up / reset starts at 30 % forward / 5 % reverse; the raised limit is RAM only, NEVER saved.
  - Raising needs ALL of: PC request + the code typed by the user + joystick CONFIRM within 10 s. One step at a time (30 → 50 → 75 → 100; reverse up to 15 %).
  - Lowering is always instant (PC, TAKE / override, STANDBY button, any alarm / fault, PC link lost, GPS lost, TRAINING, long low speed); the throttle then comes down at its normal rate, never up.
  - Exceptions (they never lower the power limit; their reaction is defined explicitly): navigation advisories (SB_NAV_ADVISORY: AIS RECEIVER SILENT, COLLISION RISK, DRIVER DECISION, SHALLOW WATER, OBSTACLE AHEAD - in AUTO the navreq slows / steers, otherwise alarm only, so MANUAL / REMOTE throttle never changes) and the slamming RED alarm when "slamming RED action" = STEP (parameter slam_red_reset = 0, the default) does NOT lower the power limit (a sudden drop can be dangerous in a following sea); it reduces the throttle one step (app/slam.c), warns red and SAVEs. LIMIT_RESET (slam_red_reset = 1) = the normal alarm rule. Slamming never raises the throttle.
  - The clamp lives only in the servo output function (app/servo.c); no other code may write the throttle servo (the build check greps for it).
- Watchdogs are enabled on all cores; a frozen loop resets the chip.
- CPU1 runs an independent safety monitor (throttle vs limit, gear change at throttle, core-0 heartbeat > 200 ms) that forces idle + neutral + steering stop + alarm, latched in the HAL.
  - Big boat (BOAT_VARIANT_BIG): Cpu0_Main.c calls monitor_publish_steering_only() after every autopilot_step (heartbeat + running, throttle 0, limits 100 %): the monitor guards against a frozen core 0 there too (trip stops the rudder motor). tools/check.sh compiles that variant too.
  - The heartbeat is watched only once core 0's control loop wrote g_mon.running = MON_RUNNING (every step; CPU1 clears it at its start): a slow boot can't trip it; core 0 that never starts trips after MON_BOOT_MS (3 s, MON_TRIP_BOOT).
  - Board (hal_tc375.c): hal_safe_outputs sets the latch FIRST, then writes idle / neutral itself into the GTM TOM shadow registers (taken at the next 20 ms period end, never a cut pulse), steering motor pin low, relay open - so it works with core 0 frozen. Every core-0 output call re-checks the latch after its write (no race). Servo PWM: TOM0 ch 15 throttle (P02.7), ch 14 gear (P02.6), 50 Hz from CMU FXCLK2; low-level Gtm/Std driver (the Gtm/Tom/Pwm driver is not in the ADS build).
- Neutral-start interlock: outputs only arm at power-up with throttle idle, neutral confirmed and the joystick centred (USB: stick centred AND lever in the neutral zone).
- USB bridge: no valid frame 150 ms / > 5 CRC errors per s / stuck counter / device unplugged / invalid axis / learn mode = JOYSTICK FAULT (same as a wire break). Hand-back needs stick centred AND lever neutral.
- Logging must never block or slow control: core 0 / CPU1 only push into lock-free queues; full queue → drop + count.
- Every feature has sim tests for its failure cases.

## Working rules
- For long multi-part tasks, keep PROGRESS.md updated after every step so the work can resume in a new session.

## About me
- I know programming but I'm new to VS Code, git and AURIX. Explain steps simply.