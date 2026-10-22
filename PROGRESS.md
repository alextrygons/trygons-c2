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
| 3 | MANUAL "Future ideas" (chapter 17 = "What is still missing" in this MANUAL) | DONE | (this commit) |
| 4.1 | fresh clone from GitHub, build + all suites + demo | next | |
| 4.2 | "a day at sea" scenario | open | |
| 4.3 | 24 h soak + 32-bit time wraparound | open | |
| 4.4 | random fault storm, invariants I1-I21 | open | |
| 4.5 | independent code review (separate subagent) | open | |
| 4.6 | coverage (gcov / coverage.py) | open | |
| 4.7 | RUN_ACCEPTANCE.bat quick / full, MANUAL "Testing" | open | |
| R | TEST_REPORT.md, final PROGRESS.md, git push | open | |

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

## Previous task (done)

Fixes F1-F3 + AIS / charts / shallow water / collision avoidance (sections
A-E): all done and pushed, last commit 033cfbd.
