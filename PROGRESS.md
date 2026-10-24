# PROGRESS

Current task: "First test preparation": (1) generated certificates in .gitignore,
(2) .cproject excludes checked, (3) FIRST TEST configuration (steering only),
(4) real hal_tc375.c Stage 1 drivers (BNO085, MAX-M10S, Pololu G2, limit
switches, buttons, buzzer, microSD, USB link), (5) BENCH TEST mode,
(6) FIRST_TEST.md checklist. One commit per item, all tests pass, git push.
If interrupted: say "continue from PROGRESS.md" - continue with the first
step that is not DONE below, without redoing finished work.

## Steps (first test preparation)

| Step | What | State | Commit |
|---|---|---|---|
| 1 | auto.crt + generated certs / keys in .gitignore | DONE | 24404ce |
| 2 | .cproject: only board sources compiled (check + fix) | DONE (was already right; check now also catches stray sources) | ce2ad44 |
| 3 | FIRST TEST configuration (steering only), telemetry + ground station | DONE (tests FT1-FT5, e-stop in every config; gate OK) | 42d1c2b |
| 4 | hal_tc375.c Stage 1 drivers (WIRING.md), timeouts / no-data detection | DONE (tc_uart/spi/bno085/gps/adc/sd/fat32, hal_pins.h, FAT32 PC test, WIRING 9 + 11; gate OK) | c02ae06 |
| 5 | BENCH TEST mode (ground station, dock) | DONE (MODE_BENCH, sb_bench, BENCH_DIAG, hal_get_diag, monitor trip test, tab Bench, BT1-BT6, I9; gate OK, storm 200 OK) | e57f815 |
| 6 | FIRST_TEST.md checklist | DONE (multimeter, power-up order, bench, dock, TRAINING, HDG LOCK) | (this commit) |

## Notes (first test preparation)

- Step 4: iLLD modules Asclin, Qspi, Dma, Evadc, _Lib/DataHandling, Stm un-excluded in the TASKING Debug
  configuration of .cproject (parents too: a parent exclusion hides its sub-folders).
- No data flash yet: boot number always 1 -> tc_fat32 adds -2, -3 to names already on the card
  (hash table of the long names from the boot scan). TRAINING result lost at power-off (FIRST_TEST.md).
- P40.6 (A3, motor current) = iLLD EVADC group 8 ch 4 (the board manual's "G4").

# Previous task: TEST_REPORT fixes (done, last commit 65e9eb2)

## Steps (fix task, done)

| Step | What | State | Commit |
|---|---|---|---|
| 1 | T11 .gitignore /TriCore*/, commit Libraries/iLLD + .cproject, fresh-clone TriCore check | DONE (fresh clone: 1002 library files, 30/30 TriCore OK) | 6a35b25 |
| 2 | R1 STANDBY: steering held (joystick in MANUAL, gyro in REMOTE) until idle + neutral; fast STANDBY ramp | DONE (storm worst STANDBY 2.16 -> 1.43 s) | 5768d2e |
| 3 | T2 HDG LOCK / AUTO refused unless the joystick is centred; held stick always takes over | DONE (storm NOTE 97 -> 0) | d90d6ac |
| 4 | R3 hal_safe_outputs drives the PWM (board path) | DONE (GTM TOM servo PWM written; TriCore compile + symbols checked; needs the bench test) | 5449c11 |
| 5 | R5 GPS NaN / inf / range rejected in every GPS path | DONE (8/8 kinds rejected, GPS2 range, DR inputs) | cf896af |
| 6 | R4 CPU1 monitor: start after core 0 "running", boot timeout | DONE (boot 3 s, test C3b) | 9d6ab75 |
| 7 | R2 big-boat CPU1 heartbeat | DONE (test C3c, gate compiles the big variant) | 1312250 |
| 8 | T1 STARLINK never auto-selected while a normal channel works | DONE (channel storm 500 runs: 0; test C11) | a38afbc |
| 9 | T3 faster GPS re-trust after a spoof at speed | DONE (~60 s with 2nd GPS / paddlewheel; old rule without - explained; test T3) | affa89f |
| 9a | (user request) ADS build: .cproject excludes every non-firmware folder, check_ads_excludes.py, CLAUDE.md rule | DONE | 2da572a |
| 10 | T4 START_DEMO builds the simulator if missing | DONE (tested on a fresh checkout) | 8b9e68a |
| 11 | review points (section 5 order): R6, R9, R11, R10, R8, T5, R20, R21, R7, R12, LOW (R13 R14 R16 R17 R18 R19 R22 T7 T12), coverage servo.c | DONE (commits d8381b5 .. 8054948) | 8054948 |
| 12 | RUN_ACCEPTANCE quick, report, push | DONE (quick 6/6 PASS, gate OK on f44f237) | (this commit) |

## Notes (fix task)

# Previous task: Parts 0-4 (done, last commit 6936f9d)

## Steps (Parts 0-4)

| Step | What | State | Commit |
|---|---|---|---|
| 0 | PROGRESS.md + CLAUDE.md rule | DONE | a27f492 |
| 1 | START_DEMO.bat / STOP_DEMO.bat, tool check, MANUAL "Demo", fresh-terminal test | DONE | 7ba8d9c |
| 2 | eWaSR: license OK (Apache-2.0 code + weights, MaSTr1325 CC BY 4.0); vision process, stages, display, tests, docs | DONE | b25ada4 |
| 3 | MANUAL "Future ideas" (chapter 17 = "What is still missing" in this MANUAL) | DONE | e385f86 |
| 4.1 | fresh clone from GitHub, build + all suites + demo | DONE: 12/13 PASS, gate fails in the clone = finding T11 (iLLD not in git) | |
| 4.2 | "a day at sea" scenario | DONE: part A 15/15; part B (whole system, real time) see TEST_REPORT 3.2 | |
| 4.3 | 24 h soak + 32-bit time wraparound | DONE (C 24 h + wrap; system soak real time) | |
| 4.4 | random fault storm, invariants I1-I21 | DONE | |
| 4.5 | independent code review (separate subagent) | DONE (report in test_results) | |
| 4.6 | coverage (gcov / coverage.py) | DONE (TEST_REPORT 3.7) | |
| 4.7 | RUN_ACCEPTANCE.bat quick / full, MANUAL "Testing" | DONE (MANUAL chapter 19) | |
| R | TEST_REPORT.md, final PROGRESS.md, git push | DONE | 09b6e0d |

## Open problems (for TEST_REPORT.md)

- Recorder stall check (file growth within stall_s 10 s) restarts recording on very low-bitrate streams
  (the nearly static demo test pictures); the demo uses stall_s 60. LOW.
- Fixed in Part 1 (tooling): test_gateway.py looked for MediaMTX in a temp folder of one session;
  tools/check.sh had `cd /c/trygons/TRYGONS_C2` and one ADS version path hard-coded; MANUAL chapter 3
  did not list ffmpeg / MediaMTX.

- Part 2 found + fixed: on gateway loss the AURIX went to nav_lost_kn (4 kn) even when a lower cap
  (e.g. an obstacle stop) was active -> now the lower of the two (NV7).
- Vision on the Pi 5 is NOT measured yet (PC: ~120-330 ms per picture, 2 fps reached).
- Part 4 notes so far: the storm runner (tests/acceptance/storm) needs the run() stub (sim_main.c is
  not linked); a 32-bit wrap needs wrap-safe loops in the harness.

## Part 4 findings so far (record only - no fixes in Part 4)
- F-I15 (MEDIUM): channels.py trial revert in AUTO goes back to STARLINK although a normal channel
  (RF) works; then stays >= 60 s (stable_s). Repro: storm_gateway.py --part channels --seed 7.
  Fix idea: on revert, call decide() unless the previous mode was a MANUAL / STRICT STARLINK choice.
  Same class: MANUAL trial revert (13/120 runs) and MANUAL fallback when the chosen channel works again
  but is not stable yet and the fallback dies -> STARLINK instead of the working chosen one (15/120).
- F-JOY (MEDIUM): STANDBY pressed with the stick not centred -> stick ignored until centred; HDG LOCK can
  then be engaged from the panel and the held stick does NOT take over (storm "NOTE" counter, seed 836).
- C storm triage: 20 first-run failures were harness issues (fixed) except F-JOY.
- F-GPS (MEDIUM): after a GPS spoof ends at ~7 kn the GPS is trusted again only after ~6.7 min (DR
  uncertainty 106 -> 177 m meanwhile); at 3.6 kn after 60 s. Repro: STORM_SLAM=1.6 STORM_DEBUG=1 storm.exe --dayatsea.
- F-DEMO (MEDIUM): demo.py build_sim() runs cmd /c with os.path.join('sim','build_sim.bat') -> 'sim/build_sim.bat'
  with MSYS2 Python -> cmd error; START_DEMO fails on a fresh clone until the sim is built by hand. Fix: 'sim\build_sim.bat'.
- OPS note: power raise in HDG LOCK needs the stick centred (= throttle 0) for the CONFIRM.
- 4.4 C storm 2000 episodes / 588 h: all invariants PASS; gateway storm: I15 finding (above), rest PASS;
  I16 station storm PASS. 4.3: C soak 24 h PASS (0 overruns, 0 drops), wrap PASS. 4.2 part A (C) 15/15 PASS.
- F-REC (LOW): recorder stall check restarts recording on very low-bitrate streams (file growth).
- F-EVT (LOW): event.json "synced" flag is never set True (dead condition in housekeeping).
- Tooling fixed in Part 4: storm runner needed a run() stub; wrap-safe loops; parser column.
- 4.5 code review DONE: test_results/code_review.md (not in git) - CRITICAL 1 (R1 steering off on STANDBY from
  MANUAL/REMOTE while throttle still ramps), HIGH 4 (R2 big-boat build no CPU1 heartbeat, R3 hal_safe_outputs only a
  flag, R4 monitor may trip at boot, R5 NaN GPS fix passes dr.c checks), MEDIUM 7, LOW 10. Copy into TEST_REPORT.md.
- I16 station storm PASS (+ self-test).

## Previous task (done)

Fixes F1-F3 + AIS / charts / shallow water / collision avoidance (sections
A-E): all done and pushed, last commit 033cfbd.

## Part 4 finished
- All results, every finding (R1-R22, T1-T11) and the fix order: TEST_REPORT.md.
- Tooling fixed at the end: day_at_sea.py read stale telemetry after time.sleep (Waiter.telem only
  updates in pump) -> waits now keep pumping; fresh_build.py keeps the gate output (test_results/fresh_check.txt).
- Next (not started, needs the user's decision): fix in the order of TEST_REPORT.md section 5, first R1 + T11.
