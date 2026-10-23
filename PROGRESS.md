# PROGRESS

Current task: "Five parts: (0) progress protocol, (1) one-click demo,
(2) eWaSR AI obstacle detection, (3) future-ideas notes, (4) full system
acceptance test" (+ TEST_REPORT.md, git push).
If interrupted: say "continue from PROGRESS.md" - continue with the first
step that is not DONE below, without redoing finished work.

## Steps

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
