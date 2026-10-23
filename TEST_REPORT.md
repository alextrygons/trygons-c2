# Trygons C2 – acceptance test report

Date: 2026-10-23. Code under test: commit `e385f86` (Parts 0–3), tested with the tools in
`tests/acceptance/` (run them again: `tests\acceptance\RUN_ACCEPTANCE.bat [full]`, MANUAL chapter 19).
Rule for this test: no product code was changed – findings are reported with a reproduction and a
proposed fix. Big outputs (logs, coverage pages, soak data) are in `test_results/` (not in git).

## 1. Summary (10 lines)

1. **The control software (AURIX) is in good shape in the simulator.** 2000 random "bad days"
   (588 simulated hours) and a 24-hour soak broke none of the safety rules that are checked on the
   boat side (I1–I13, I18–I20), and the checks were proven to catch planted violations.
2. The STANDBY button won in 0.0 s (mode) / at most 2.9 s to idle + neutral; the onboard joystick
   took over within 0.09 s; remote-data loss gave idle + neutral within 0.55 s; the CPU1 monitor
   reacted within 0.21 s – every time.
3. **One CRITICAL point (R1) needs your decision:** on STANDBY from MANUAL or REMOTE the steering
   stops at once while the throttle is still coming down (up to ~3.5 s). CLAUDE.md says "idle +
   neutral first, then steering off". Either the code or the rule has to change.
4. **Five HIGH points, all for the real board:** the driver library for the board is missing on
   GitHub (T11, a one-line .gitignore fix); the big-boat build would trip the CPU1 monitor
   (R2), the real "force safe outputs" does not drive the servos yet (R3), the monitor can trip
   during a slow boot (R4), a NaN GPS value would pass all GPS checks (R5).
5. **MEDIUM:** the gateway can pick STARLINK while a normal channel works (after a channel trial,
   or in MANUAL fallback); after STANDBY with the stick held, HDG LOCK can be engaged and the held
   stick then does not take over; GPS trust comes back slowly (~7 min) after a spoof at speed;
   START_DEMO fails on a fresh clone until the simulator was built by hand; and 7 review points.
6. The day at sea (part A 15 / 15, part B 12 / 12) and the gateway storms behaved as specified,
   apart from these points.
7. The real board is not tested at all: hal_tc375.c is still mostly stubs (no sensors, PWM, UARTs,
   buttons, joystick, flash, SD card) – see section 4.
8. The camera obstacle detection is licensed correctly (Apache-2.0 / CC BY 4.0) and works on the PC
   (~2 fps on 2 cores); its speed on the Pi 5 is NOT known yet.
9. Coverage is high on the AURIX code (smallboat.c 97 % of lines); the gaps (servo.c branches, the
   big boat, the real router / UART code) are listed in 3.7.
10. **Fix order:** R1 decision, T11 now (5 min), R3 + R5 + R4 before the first powered bench test, then the MEDIUM
    gateway / joystick points, then the rest (section 5).

## 2. All findings (CRITICAL first)

ID: R = independent code review (subagent, code reading, `tests/acceptance/code_review.md`),
T = found by the tests here. "checked" = I confirmed it in the code or by a test.

| ID | Severity | Where | Finding (plain) | Proposed fix |
|---|---|---|---|---|
| R1 | CRITICAL (checked: code + sim tests expect it) | app/smallboat.c:459-475, 1810 | On STANDBY from MANUAL / REMOTE the steering stops in the same step while the engine still pushes for up to ~3.5 s (throttle ramp + 1 s gear wait). The rule says idle + neutral first. | Hold the last outboard angle (or gyro hold) until idle + neutral, then steering off – or write the MANUAL exception into CLAUDE.md. |
| T11 | HIGH (build, test; not a safety problem) | .gitignore line 6 | The Infineon driver library (778 files in Libraries/iLLD/TC3xx/Tricore/) is not on GitHub: the ignore rule `TriCore*/` for the ADS build folder also matches "Tricore" (git on Windows ignores case). A fresh clone can't build the board firmware; tools/check.sh fails there. Repro: `fresh_build.py` -> `Ifx_Types.h: No such file`. | Change the rule to `/TriCore*/` (only the build folder at the top), then `git add Libraries/iLLD` and commit. |
| R2 | HIGH (checked) | Cpu0_Main.c (big boat), app/monitor.c | The big-boat build never sends the CPU1 heartbeat: the monitor trips after 200 ms and the rudder can never move. No test. | Publish the heartbeat (and safe throttle/gear values) from autopilot_step; add a big-boat + monitor sim test. |
| R3 | HIGH | hal/hal_tc375.c:167-176 | The real "force safe outputs" only sets a flag; idle / neutral pulses reach the servos only when core 0 writes again – never if core 0 is frozen. Small race too. | Write idle / neutral directly into the PWM hardware in hal_safe_outputs; re-check the flag after every write; bench test. |
| R4 | HIGH | Cpu1_Main.c:196-199, app/monitor.c:97-107 | CPU1 waits only 1 ms for core 0 at start; the heartbeat timeout counts before the first heartbeat. A slow boot (> 200 ms) trips the monitor and the boat won't start. | Check the heartbeat only after the first one, with a separate boot timeout (e.g. 2 s). |
| R5 | HIGH (checked) | app/dr.c:277-397 | A GPS fix is never checked for NaN / impossible values; a NaN passes every integrity test and spoils the position while GPS still looks trusted. | Reject fixes that are not finite or out of range (and count them as rejected); sim test with a garbage fix. |
| T1 | MEDIUM (test) | gateway/channels.py | STARLINK chosen while 4G / RF works: (a) a channel trial that runs out goes back to STARLINK (AUTO and MANUAL); (b) MANUAL fallback when the chosen channel works again but is not "stable" yet. 33 of 120 random 2-hour runs. Repro: `storm_gateway.py --part channels --seed 7`. | On a trial revert re-decide (unless the previous setting was an explicit STARLINK choice); in fallback prefer the healthy chosen channel over STARLINK. |
| T2 | MEDIUM (test) | app/smallboat.c (HDG LOCK button, STANDBY "ignored until centred") | STANDBY pressed with the stick not centred → the stick is ignored until centred; HDG LOCK (or AUTO / REMOTE) can still be engaged, and the held stick then does NOT take over. 97 times in 2000 episodes. Repro: storm seed 836 / 12. | Engage HDG LOCK / AUTO / REMOTE only with the joystick centred (like the hand-back), or let a deflection count once the stick was centred after STANDBY. |
| T3 | MEDIUM (test) | app/dr.c (GPS re-trust) | After a GPS spoof ends at ~7 kn, GPS is trusted again only after ~6.7 min (dead reckoning 106 → 177 m meanwhile); at 3.6 kn after 60 s. Safe (AUTO keeps going on DR / holds), but slow. Repro: `STORM_SLAM=1.6 STORM_DEBUG=1 storm.exe --dayatsea`. | Re-check the acceptance window (fix vs DR within the DR uncertainty) and the N-consistent-fixes rule at speed. |
| T4 | MEDIUM (test) | demo/demo.py build_sim() | START_DEMO fails on a fresh clone until the simulator was built by hand: `cmd /c sim/build_sim.bat` (MSYS2 Python joins with "/") is not understood by cmd. | Use `sim\build_sim.bat`. |
| R6 | MEDIUM | app/link.c:647-668, smallboat.c:239-270 | A navigation HEADING from the Pi is accepted with any value; an accepted HDG LOCK heading can still be applied up to 3 s after entering HDG LOCK. | Limit the deviation from the track, require a cap with a heading, apply HDG_SET only if it arrived while already in HDG LOCK. |
| R7 | MEDIUM | app/link.c:686-819 | The AURIX trusts the station id / "local" flag sent by the Pi (a faulty Pi could act as a boat station). CONFIRM still protects AUTO / REMOTE / power. | Document it as design, or keep a fixed list of local station ids on the AURIX. |
| R8 | MEDIUM | app/monitor.c, logq.c | Memory placement of the shared monitor data / log queues (cache) is not fixed; unverified until the first ADS build. | Place them in non-cached memory; check with tools/mapreport.py after the first build. |
| R9 | MEDIUM | app/smallboat.c:1567 | If no joystick is detected at boot, the outputs arm without the stick check. | Wait for the joystick (or a "no joystick" setting) before arming. |
| R10 | MEDIUM | Cpu2_Main.c:47-52 | CPU2 watchdog 500 ms; a slow SD-card write could reset the whole chip. | Bounded SD writes, kick the watchdog between chunks. |
| R11 | MEDIUM | app/link.c:530-534 | ALARM_ACK from any HELM station clears every alarm; link-lost alarms are not raised again while the link is still down. | Clear only alarms whose cause is gone; restrict ACK from remote stations. |
| R12 | MEDIUM | tests | No tests for the error paths of R2, R4, R5, R6, R11, R16. | Add sim tests. |
| T5 | LOW (test) | gateway/recorder.py | The recording stall check (file growth within 10 s) restarts recording on very low-bitrate streams (seen with the demo's test pictures). | Use the MediaMTX byte counter or a longer stall time. |
| T6 | LOW (test) | MediaMTX (third party) | 30-min soak: MediaMTX handles 227 → 289 (weak upward trend, R² 0.64); memory steady. | Watch it in a longer soak on the Pi; restart MediaMTX daily if it grows. |
| T7 | LOW (test) | gateway/recorder.py | The `"synced"` flag in event.json is never set to True (dead condition). | Remove it or set it when the shore confirms. |
| R13 | LOW | smallboat.c:168-173 | The sim test hook sb_test_drive is compiled into the firmware. | `#ifdef SIM`. |
| R14 | LOW | smallboat.c:471-474 | With fault_steer ≠ GYRO a compass loss while moving turns the steering off at once. | Remove the option or centre instead. |
| R15 | – (not confirmed) | CLAUDE.md | Review said OBSTACLE AHEAD is missing from the exception list – CLAUDE.md already lists it. | none |
| R16 | LOW | app/logq.c:53-66 | A corrupted length in the log queue breaks it for good. | Check the length, resync. |
| R17 | LOW | app/link.c:246-370 | Telemetry is 193 of 200 bytes; one more field = telemetry silently never sent. | Static assert / test on the size. |
| R18 | LOW | app/link.c:1059 | > 64 frames in one step: further bytes overwrite unparsed frames. | Stop reading at 64 frames. |
| R19 | LOW | app/link.c:792-800 | GW_GPS2 lat / lon not range-checked. | Range-check. |
| R20 | LOW | gateway/aurix.py:56-58 | "AURIX alive" updated on any byte (also noise). | Only on a good frame. |
| R21 | LOW | gateway/vision/geometry.py | No guard for a zero field of view; a NaN distance hides an obstacle. | Validate the camera config; treat NaN as "in the path". |
| R22 | LOW | smallboat.c:1882, monitor.c:91 | Monitor data written field by field (a false trip just after a limit change is possible but unlikely). | Seqlock, or write the limits before the throttle. |
| T12 | LOW (review, checked) | .gitignore | `gateway/secrets.json` (real sync key / passwords on the Pi) is not git-ignored; nothing secret was ever committed (whole history searched), but one `git add .` on the Pi would. | Add `gateway/secrets.json` and `*.key` to .gitignore. |
| T8 | INFO | spec wording | I1 "never above the power limit": by design the throttle may be above a just-lowered limit while it comes down at 40 %/s (CPU1 allows 2.5 s). Tested that way. | Say so in CLAUDE.md. |
| T9 | INFO | operation | Raising the power in HDG LOCK needs the stick centred for CONFIRM = throttle to zero for a moment. | Mention in the MANUAL (power limit chapter). |
| T10 | INFO | gateway/stations.py | The driver-ACCEPT list (DRIVING_CMDS) does not include the remote-driving request or power raise; both need the onboard CONFIRM anyway. | Confirm this is intended. |

## 3. Results per part

### 3.1 Fresh build from GitHub (4.1)
`fresh_build.py`: clone from GitHub (`e385f86`) into an empty folder, then only the steps in
MANUAL chapters 3, 4 and 4A. **12 of 13 steps PASS; 1 FAIL = finding T11.**

| step | result | detail |
|---|---|---|
| git clone | PASS | 1 s |
| sim\build_sim.bat | PASS | BUILD OK |
| autopilot_sim all | PASS | 146 passed, 0 failed |
| autopilot_sim all --bridge | PASS | 145 passed, 0 failed, 1 not applicable |
| python tools\get_mediamtx.py | PASS | MediaMTX v1.21.2 into the clone's tools\bin |
| sh tools/check.sh (the gate) | **FAIL** | everything passes (sim 146 + 145, charts 5, avoidance 14, gateway 82) **except the TriCore compile of hal_tc375.c and Cpu0/1/2_Main.c: `Ifx_Types.h: No such file`** – T11 |
| groundstation test_e2e / test_logs / test_driver | PASS | 10 / 3 / 8 passed |
| START_DEMO from the clone | PASS | all 5 ports up, STOP_DEMO stopped everything (here the simulator was already built by the step above; on a clone where nobody built it, T4) |
| paths that only exist on this PC | PASS | none found |
| not in git but made by documented tools | PASS | charts (tools/chartprep.py), vision model (tools/get_vision_model.py), vision Python (tools/get_vision_python.py) |

**T11:** `.gitignore` line 6 `TriCore*/` is meant for the ADS build folder "TriCore Debug (TASKING)",
but git on Windows ignores letter case, so it also hides `Libraries/iLLD/TC3xx/Tricore/` –
778 files of the Infineon driver library are not on GitHub. The board firmware can't be built from
a clone (and would be lost with this PC).

### 3.2 A day at sea (4.2)
Two parts (the simulator server can't be told sea state, compass glitches or a GPS spoof from
outside, and Part 4 may not change the simulator): part A in the C harness (simulated time),
part B through the whole system in real time.

**Part A (AURIX, `storm.exe --dayatsea`, ~50 simulated minutes): 15 / 15 PASS, invariants held all day.**

| time | what happened | what the system did | expected | result |
|---|---|---|---|---|
| 3 s | power-up with the joystick held | armed 0, throttle 0.0 %, gear N | outputs not armed, idle, neutral | PASS |
| 4 s | joystick centred, neutral confirmed | armed 1, power limit 30 / 5 % | armed; power limit 30 / 5 | PASS |
| 281 s | TRAINING (PC, calm water), result accepted | started 1, result reached (code 0), accepted 1, mode 0 | calibration runs, idle + neutral, accept -> STANDBY | PASS |
| 341 s | HEADING LOCK, joystick throttle, +1 +10 -1 -10 | mode 2, 7.0 kn, target +1 +11 +10 +0 | mode HDG LOCK, speed > 3 kn, target +1 +11 +10 0 | PASS |
| 358 s | power limit raised 30 -> 50 -> 75 % | without CONFIRM 30 %; then 50 %, 75 % | no CONFIRM: stays 30; each step needs CONFIRM | PASS |
| 360 s | AUTO program (6 waypoints, 6-16 kn) started with CONFIRM | program 0, start 0, CONFIRM 1, mode 3 | program accepted, AUTO after CONFIRM | PASS |
| 450 s | AUTO on leg 1 | mode 3, state 0, 8.0 kn (target 8.0 kn) | running, about the leg speed (limit 75 %) | PASS |
| 956 s | sea state 4, slamming on the fast leg | impacts 9, lowest cap 30 %, power limit 75 %, state 0 | impacts, throttle reduced in steps, power limit kept | PASS |
| 1209 s | GPS lost for 4 min | DR 1, after 4 min mode 3, uncertainty 44 m; GPS trusted again 1 | dead reckoning, AUTO continues (slower), GPS trusted again | PASS |
| 1270 s | GPS spoof attempt (800 m jump) | rejected 1, alarm 1, recovered 1, mode 3 | rejected, GPS SUSPECT, dead reckoning, recovers | PASS |
| 1276 s | compass glitch 0.8 s | mode 3, new compass alarm 0 | no reaction (below the 2 s timeout) | PASS |
| 1280 s | joystick takeover in AUTO, then resume | MANUAL 1, SAVE 1, resume 0 + CONFIRM 1 -> mode 3 | MANUAL at once + SAVE; resume = PC + CONFIRM | PASS |
| 1296 s | remote driving, 2 s link dropout | request 0, CONFIRM 1, driving 1; LOST after 0.30 s, data back -> still LOST/idle 1, STANDBY 1 | LOST within 0.4 s: idle + neutral + gyro hold, never resumes, STANDBY after 5 s | PASS |
| 2966 s | arrival at the last waypoint (STOP) | resume 0 + CONFIRM 1, arrived 1, throttle 0.0 %, gear N, mode 0 | ARRIVED, idle + neutral | PASS |
| 2966 s | STANDBY | mode 0 | STANDBY | PASS |

**Part B (whole system, `day_at_sea.py`, ~30 min real time: simulator + gateway + 4 cameras + AIS + vision + channels + sync + captain / driver screens): 12 / 12 PASS.**

| t | gateway time | what happened | what the system did | expected | result |
|---|---|---|---|---|---|
| 23 s | 10:00:22 | HEADING LOCK, crossing ship | proposal "239500001: CPA 0.01 nm in 3.3 min. Steer 180 and slow to 5 k"; accepted -> locked heading 180 | proposal on the driver screen, ACCEPT changes the heading | PASS |
| 45 s | 10:00:44 | AUTO program (6 waypoints, past the 1.5 m shoal) | upload 0, driver asked True, AUTO running | accepted by the depth check, AUTO after ACCEPT + CONFIRM | PASS |
| 188 s | 10:03:07 | AIS crossing ship during AUTO | request applied yes (planner SLOW), back to the leg yes | slow down, route astern (or DRIVER DECISION slow), then rejoin | PASS |
| 191 s | 10:03:10 | vision blip + kayak without AIS, STAGE 3 off | blip filtered True; OBSTACLE AHEAD yes, SAVE yes, no cap True | blip ignored; alarm + SAVE, no effect on the boat | PASS |
| 200 s | 10:03:19 | kayak without AIS, STAGE 3 on (driver ACCEPT) | speed cap 0.0 kn applied yes, throttle 24.0 -> 0.6 % | AUTO slows / stops, never more throttle | PASS |
| 211 s | 10:03:31 | 4G fails -> RF; RF fails -> STARLINK | RF True, STARLINK True, data saver + no sync True, banner flag True | RF first, then STARLINK emergency: saver, no sync, banner | PASS |
| 228 s | 10:03:47 | 4G returns | back on 4G True (after 15.0 s stable), sync allowed True | automatic switch back, sync resumes | PASS |
| 437 s | 10:07:17 | GPS lost for 4 min | dead reckoning yes, mode after 4 min AUTO, GPS trusted again yes | DR, AUTO continues, GPS back | PASS |
| 452 s | 10:07:31 | remote driving, 2 s dropout | request 0, REMOTE yes; LOST after 0.38 s; data back -> still LOST + idle True; STANDBY yes | idle + neutral + gyro hold, never resumes, STANDBY after 5 s | PASS |
| 454 s | 10:07:33 | joystick takeover during AUTO, resume | MANUAL yes, SAVE request yes -> event (new or merged) True, resumed yes | MANUAL at once + SAVE; resume = PC + CONFIRM | PASS |
| 1780 s | 10:29:40 | arrival at the last waypoint | arrived yes, mode STANDBY, throttle 0.0 %, gear 0 | STOP: idle + neutral | PASS |
| 1801 s | 10:30:00 | sync to the shore receiver | 181 files confirmed: 180 checksums equal, 0 different [], 1 changed after the confirmation (queued again), 0 not found on shore; 4 saved events on the SSD; queue {} | every confirmed file identical on shore | PASS |

(The first run showed a FAIL at the arrival; it was the test reading an old telemetry frame - fixed in the test tool, the boat was idle + neutral.)

### 3.3 Soak (4.3)
- **AURIX, 24 simulated hours with logging** (`storm.exe --soak 24 24 --log`): all invariants held,
  1,725,520 control steps, 225 reboots (resets in the schedule), 0 core-0 overruns, 0 log queue drops,
  1.38 million log records (251 MB) written.
- **Control step time with / without logging (PC, 2 h):** longest 498 / 167 µs, average 2 / 0 µs,
  99.9 % below 100 µs – of a 50,000 µs budget (I18). The TC375 must be measured on the board.
- **32-bit time wraparound:** started 3 min before 2^32 ms, ran 10 min across it: all invariants held,
  11,997 control steps (expected ~12,000), 7 reboots, 0 overruns.
- **System soak, 30 min real time** (gateway + 4 cameras via MediaMTX + recording + vision + AIS +
  sync + channel changes, 31 SAVEs, 25 channel changes, 115 mode presses): memory steady for the
  gateway (+7 MB/h, startup), simulator, recorders, vision; MediaMTX handles +54/h (T6); 193 ring
  files deleted on time; 228 files synced, backlog max 28.5 MB, 0 at the end; AURIX 0 overruns,
  0 log drops; vision ACTIVE at the end (0–2 fps on the busy PC, 1.2 GB private memory).
  Real time only: 24 h of gateway time must be run on the Pi (`soak_system.py --minutes 1440`).

### 3.4 Fault storm – safety invariants (4.4)
`run_storm.py --episodes 2000`: **2000 episodes, 588 simulated hours, 3.9 min on this PC (10 workers).**
Self-test first: 3 planted violations (power raised without CONFIRM, a forbidden mode change, a
throttle written above the limit) were all caught.

| invariant | checks | fails | worst reaction |
|---|---|---|---|
| I1 throttle ≤ power limit (coming down at the closing rate allowed) / remote cap | 41,518,013 | 0 | – |
| I2 30 % / 5 % after every power-up / reset | 25,714 | 0 | – |
| I3 gear only at idle; idle → neutral → gear → throttle | 277,431 | 0 | – |
| I4 STANDBY always wins | 15,600 | 0 | 2.88 s to idle + neutral |
| I5 the onboard joystick wins | 7,398 | 0 | 0.09 s |
| I6 PC / remote never take control from the joystick | 6,130 | 0 | – |
| I7 power raise / AUTO / REMOTE only with CONFIRM within 10 s | 5,854 | 0 | – |
| I8 CONFIRM alone never changes the mode | 21,009 | 0 | – |
| I9 idle + neutral commanded before / with steering off | 206,934 | 0 | – (see R1: *reached* is not checked) |
| I10 only allowed mode transitions | 37,073 | 0 | – |
| I11 disarmed at power-up until neutral + centred | 13,276,473 | 0 | – |
| I12 remote data old / missing → idle + neutral, never resumes | 177,312 | 0 | 0.55 s |
| I13 slamming / caps never increase the throttle | 41,528,945 | 0 | – |
| I14 avoidance / programs never over shallow water, never across a bow (gateway storm) | 23,631 | 0 | – |
| I15 STARLINK never for sync / never auto-selected while a normal channel works | 864,418 | 33 | – (T1) |
| I16 driving actions only with driver ACCEPT / single-operator (station storm + self-test) | 54 forwarded / 17 driving | 0 | – |
| I17 un-synced saved events never deleted | 1,486 | 0 | – |
| I18 logging never delays the control step | 2 × 143,410 steps | 0 | longest 498 µs (PC) |
| I19 CPU1 monitor safe state on every injected violation | 14,288 | 0 | 0.21 s |
| I20 navigation / vision caps never steer, never raise the throttle (C + gateway) | 41.5 M + 1.29 M | 0 | – |
| I21 vision crash / overload never releases a cap by itself | 864,000 | 0 | – |

AUTO was entered 1122 times (always PC + CONFIRM), REMOTE 3687 times. The first run showed 20
failing seeds; 19 were mistakes in my own checks (fixed in the test tools, never in the product)
and one was the real finding T2. Every failing seed can be replayed: `test_results\storm.exe --replay SEED`.

### 3.5 Vision (Part 2)
- Licence: eWaSR code + weights Apache-2.0, MaSTr1325 CC BY 4.0 – allowed (checked twice:
  by me and by the reviewer).
- Model: ewasr_resnet18 (official ONNX, 512 × 384). PC speed: 115–190 ms per picture on 2
  threads (with "denormal as zero"; 1500 ms without), 7.8 fps on the MaSTr1325 pictures, ~2 fps in
  the gateway (limited by max_fps). 99.9 % of pixels right on 100 MaSTr1325 pictures (but the
  model was trained on them – optimistic).
- False blips: single-frame detections never became obstacles (gateway tests VI2, day at sea B, the
  vision storm). Worker crash → OFFLINE notice, restart, no mode change, an active cap held until the
  driver's CLEAR (VI7, VR4, storm I21).
- **Not known: the speed on the Pi 5** (expect several hundred ms per picture; the INT8 model is
  ready to try).

### 3.6 Independent code review (4.5)
A separate subagent with no history read CLAUDE.md, MANUAL.txt, WIRING.md and the code:
1 CRITICAL, 4 HIGH, 7 MEDIUM, 10 LOW (one, R15, not confirmed). All are in the table in section 2;
the full review (with the hal_tc375.c stub list) is `tests/acceptance/code_review.md`.
Secrets: none in the git history; add `gateway/secrets.json` to .gitignore (T12); make sure the demo /
test users never exist on the real gateway.

### 3.7 Coverage (4.6)
Measured with gcov (C: simulator suites with both joysticks, the fault storm, day at sea A, time
wrap) and coverage.py (Python: every gateway section, avoid, the gateway / station storms, the
ground-station suites). Full tables: `test_results/coverage_summary.txt`.

**AURIX code (app/), lines / branches:** smallboat.c 97 % / 88 %, training.c 99 / 90, dr.c 100 / 91,
joystick.c 100 / 93, slam.c 100 / 99, monitor.c 100 / 88, powerlim.c 98 / 78, link.c 89 / 78,
logger.c 93 / 75, autopilot.c (big boat) 91 / 68, servo.c 100 / **60**, modes.c 100 / 69.

**Gateway / ground station (lines):** avoid.py 96 %, shallow.py 95, sync.py 94, recorder.py 90,
stations.py 89, gateway.py 88, ais.py 88, vision/manager.py 83, channels.py 80, aurix.py 62,
groundstation/app.py 70.

**Safety-relevant code that no test runs (the gaps):**
- `app/servo.c`: every line runs, but only 60 % of its branches – the clamp's corner cases
  (e.g. reverse at the limit, limit lowered during reverse) are not all exercised. Add unit tests.
- `app/autopilot.c` (big boat): `manual_loop`, `autopilot_manual`, `autopilot_load_calibration`,
  `autopilot_set_outboard_config` – and no big-boat test runs with the CPU1 monitor (see R2).
- `app/link.c`: `send_prog_wp`, `link_param_info`, `link_param_count` (reading a program / the
  parameter list back to the PC).
- `app/powerlim.c`: `plim_result_name` (text only).
- `gateway/channels.py`: the real router driver (`_ssh`, `apply`, `status`, `read`) – only the
  simulated network is tested; the RutOS router must be tested on the bench.
- `gateway/gateway.py`: `_serial` / `reader` – the real UART to the AURIX (only TCP in the sim);
  `gateway/aurix.py` 62 % for the same reason (and R20).
- `gateway/vision/manager.py`: `throttle_bits` (Pi throttling / heat check – PC has none),
  `_front_url` (camera switch while vision runs).
- `gateway/vision/worker.py`: runs in its own Python and is not measured; its parts are covered by
  the VR tests (real model) and the evaluation tool, not line by line.
- Ground-station tools never run by a test: logview.py, log2csv.py, replay.py, tlgreplay.py,
  program.py (45 %) – not safety-relevant, but program.py loads waypoint files (the AURIX validates
  them again).

## 4. What can't be tested without the real board, the Pi and the boat
- **The board:** hal_tc375.c is mostly stubs – compass, gyro / BNO085, GPS (NMEA), throttle / gear /
  steering PWM, panel buttons (STANDBY!), joystick ADC, both UARTs, the USB bridge input, DFLASH,
  SD card (list: code_review.md). Real timing of the 20 Hz loop, the watchdogs, the CPU1 monitor
  latch with real PWM (R3), boot timing (R4), cache / memory placement (R8), the TASKING build.
- **The Pi 5:** vision speed and temperature, MediaMTX + 4 cameras + recording + sync together over
  hours, the real router (RutOS) and Starlink, the dAISy HAT, the real SSD.
- **The boat:** the outboard / steering motor model, sea behaviour, slamming levels, the real
  compass / GPS noise, TRAINING on the water, AIS reception range, camera views, the lookout.

## 5. Suggested order for fixing
1. **R1** – decide: hold the steering until idle + neutral, or change the rule (it is about safety).
2. **Now, 5 minutes:** T11 – fix the .gitignore line and commit the driver library, so the
   firmware can be built from GitHub.
3. **Before the first powered bench test:** R3 (safe outputs drive the PWM), R5 (GPS NaN / range
   checks + test), R4 (boot vs heartbeat), R2 (big boat heartbeat, if the big boat is used), and
   hal_get_button_presses / hal_read_joystick (STANDBY and CONFIRM must work on the board).
4. **Before sea trials:** T2 (engage autopilot only with a centred stick), R6 (limit navreq heading),
   R9 (arm only with a joystick seen), R11 (alarm ACK), R10 (SD watchdog), R8 (check the map file).
5. **Gateway:** T1 (STARLINK selection), T5, R20, R21; T4 (START_DEMO on a fresh clone).
6. **Then:** T3 (GPS re-trust time), R7, R12 tests, the LOW points, the coverage gaps.
After each fix: `sh tools/check.sh`, then `tests\acceptance\RUN_ACCEPTANCE.bat`.
