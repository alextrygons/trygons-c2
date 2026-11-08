# Cleanup 2026-11-07 (owner: "credits left this week, cleanup type work"). NO product logic change in app/ or hal/ anywhere in CL1-CL4.

## 13v (only if not DONE yet) - manual fixes, docs only
Fix in BOTH MANUAL_EL.txt and MANUAL.txt: (1) header date -> today; (2) ch. 7 «ΚΟΜΒΙΟ ΕΚΤΑΚΤΗΣ ΑΝΑΓΚΗΣ": e-stop -> STANDBY + alarm, stops the ENGINE through the hard-wired kill circuit in EVERY profile (OD-1), cuts the DP thrusters' contactor, does NOT cut the servo power (OD-2); (3) ch. 12 «ΕΠΙΤΗΡΗΤΗΣ ΑΣΦΑΛΕΙΑΣ»: steering servo HOLDS (OD-10), P10.5 opens the kill circuit ONLY in UNCREWED, latched until reset; (4) ch. 4: all info on the video GREEN (white removed in 13n1), red alarms only, amber caution, night dim red; (5) ch. 11 UAV heading: remove "coming in later steps" (U3-U6 done). Do NOT edit academy/content/tour.json; list matching stale tour lines. PROGRESS row 13v, commit "MANUAL: fix stale e-stop / monitor / HUD colour / UAV lines (13v)".

## CL1 Stale-text sweep (docs + lang text only)
- Compare against the CURRENT CLAUDE.md (incl. OD-1..OD-12, 13l-13u decisions, section V UAV, 13n1 colours / layout / mode bar, 13p joystick buttons, 13q lost link / LINK TIME / near shore 50 m / no RELEASE): MANUAL.txt, MANUAL_EL.txt, groundstation/lang/el.txt + en.txt, academy lessons / quizzes / scenarios / exam texts, FIRST_TEST.md, WIRING.md, PARTS.md, README files, gateway/README.md, docs/*.md that describe current behaviour (NOT the dated owner_findings_* docs and NOT historical review notes: those stay as written).
- Typical stale items: e-stop cutting servo power, "steering off" for the position servo, white HUD info, a RELEASE button, near shore 100 m, LINK TIME missing, «ταχύτητα» meaning gear (must be «σχέση»), "confirm live camera before HELM", slamming auto-restore, UAV steps "coming later".
- Fix the text (MANUAL pair together; lang el + en together). academy/content/tour.json: do NOT edit (recorded Greek clips) - LIST each stale tour line (stop id, text, what is wrong) in docs/cleanup_report.md for the owner.
- A quiz / exam answer that was wrong = fix it and say so in the report.
- Output docs/cleanup_report.md (section CL1: file, line, old -> new, reason). Commit "docs: stale-text sweep (CL1)".

## CL3 Stray pythonw / autopilot_sim processes (W10/W11) - run BEFORE CL2 (strays are a suspected cause of the flaky tests)
- Find which tests / tools / START_DEMO / TOUR paths can leave python(w).exe, autopilot_sim.exe, mediamtx, ffmpeg or gateway workers running (no try/finally, no Popen cleanup, no job object, parent killed first, daemon threads).
- Fix in the TEST / TOOL / LAUNCHER code only: every spawned child is stopped on exit and on failure (try/finally, atexit, Windows Job Object with KILL_ON_JOB_CLOSE where it fits), the test runner checks before and after each suite and reports leftovers (name + PID + which suite) and fails the "no leftovers" check.
- A tools/strays.py helper: list / stop leftovers of THIS project only (never other Python programs of the owner: match by command line inside the repo folder).
- Report in docs/cleanup_report.md (CL3). W10/W11 -> answered or updated. Commit "tests: no stray processes (CL3)".

## CL2 Flaky tests (test_driver I4 / I12 / I19, gateway vision VR2 / VR3 / VR5) - high effort
- Goal = find the REAL cause of the failures under PC load, not hide them. Reproduce first: run each suite N times under a CPU load generator (tools/ script, e.g. 2-4 busy processes) and record the failure rate before the fix.
- Allowed fixes: test harness timing (wait for a condition with a timeout instead of fixed sleeps, sim time instead of wall time, deterministic seeds, ports picked free, readiness handshakes, process priority of the test itself). NOT allowed: wider tolerances without a measured reason, removed / skipped assertions, any change of product logic in app/, hal/, gateway/, groundstation/ (if the cause IS a product bug: do NOT fix it here; write it up with a reproduction as a WAITING FOR OWNER / new fix step proposal - for a firmware bug it needs the serial safety review).
- After: the same N runs under the same load, failure rate after the fix. Numbers in docs/cleanup_report.md (CL2) + TEST_REPORT.md.
- Timebox: if a cause is not found after a reasonable effort, commit only what is proven (e.g. the load reproducer + measurements), keep rule W50, and describe what was ruled out. Never guess-fix.
- Commit "tests: fix flaky driver / vision tests under load (CL2)".

## CL4 Tidy
- PROGRESS.md: move all finished tasks older than the current one into docs/progress_archive.md (verbatim, nothing lost); PROGRESS.md keeps the current task, WAITING FOR OWNER, LOOP RULES and a link to the archive. A check in tools/check.sh: every step id that was DONE before is found in PROGRESS.md or the archive.
- Lang keys: list keys in el.txt / en.txt used nowhere (search ALL code incl. academy/, tour, gateway, f-strings / key prefixes built at run time). Remove only keys proven unused; anything dynamic or unsure = keep and list. test_lang must pass.
- Dead files: LIST scripts / files nothing references (tools/, demo/, old docs) in docs/cleanup_report.md (CL4) - do NOT delete them; the owner decides.
- Refresh the repo HANDOVER.md (date, HEAD, status, next steps; 13v + CL + AM task). Commit "tidy: progress archive, unused lang keys, handover (CL4)".
