# Cleanup 2026-11-07 - report (CL1-CL4)

Spec: docs/owner_findings_2026-11-07.md. No product logic in app/ or hal/ was changed in any CL step
(`git diff` of app/ and hal/ is empty in every CL commit).

## CL1 - stale-text sweep (docs + lang text only)

Compared against the CURRENT CLAUDE.md (OD-1..OD-12, 13l-13u, section V UAV, 13n1 colours / layout /
mode bar, 13p joystick buttons, 13q lost link / LINK TIME / near shore 50 m / no RELEASE): MANUAL.txt,
MANUAL_EL.txt, groundstation/lang/el.txt + en.txt, academy lessons / quizzes / scenarios / exams,
FIRST_TEST.md, WIRING.md, PARTS.md, the README files (gateway/, groundstation/, bridge/, gateway/vision/,
docs/screens/, docs/presentation/), docs/HANDOVER.md, docs/uav_design.md, docs/ACADEMY_SPEC.md.
Not swept (as the spec says): the dated docs/owner_findings_*.md, the review notes / design history
(docs/dronecan_design.md sections that record the state before a decision), PROGRESS.md.
Step 13v (2026-11-07) had already fixed the manuals' e-stop / monitor / HUD-colour / UAV-heading lines.

### Changed (file, line, old -> new, reason)

| File | Line | Old -> new | Reason |
|---|---|---|---|
| MANUAL.txt | 1301 | "the gyro holds the course until idle + neutral, then the steering stops." -> "... then the steering is held (the position servo keeps its angle, OD-10)." | OD-10: STANDBY steering = HOLD for the DroneCAN position servo (CLAUDE.md T (7)) |
| MANUAL.txt | 1312 | "then the steering stops;" -> "then the steering is held;" | same (THE STANDBY BUTTON ALWAYS WINS, the MANUAL case) |
| MANUAL.txt | 2023 | "then the steering stops." -> "then the steering is held." | same (USB bridge fault) |
| MANUAL.txt | 3774-3775 | "This step does not answer yet (text and voice come with the next steps)." -> "The answer itself (text and voice) is described below (THE ANSWER, THE VOICE AND THE PANEL)." | AM4 / AM5 are DONE: the answer and the voice exist |
| MANUAL.txt | 3809-3810 | "The next step (the language model answers ONLY from the passages, with a check) has to improve the refusals." -> "The answer step below (the language model picks ONLY from the passages, with a check) improves the refusals." | same; "picks" = the extract mode (AM4-r) |
| MANUAL_EL.txt | 2626-2627 | «Σε αυτό το βήμα δεν λέει ακόμα απάντηση (κείμενο και φωνή έρχονται στα επόμενα βήματα).» -> «Η ίδια η απάντηση (κείμενο και φωνή) περιγράφεται παρακάτω (Η ΑΠΑΝΤΗΣΗ, Η ΦΩΝΗ ΚΑΙ ΤΟ ΠΛΑΙΣΙΟ).» | the Greek pair of MANUAL.txt 3774 |
| MANUAL_EL.txt | 2664-2665 | «Το επόμενο βήμα (...) πρέπει να βελτιώσει τις αρνήσεις.» -> «Το βήμα της απάντησης παρακάτω (το μοντέλο γλώσσας διαλέγει ΜΟΝΟ από τα κομμάτια, με έλεγχο) βελτιώνει τις αρνήσεις.» | the Greek pair of MANUAL.txt 3809 |
| MANUAL_EL.txt | 600 | «τετράγωνο λευκό περίγραμμα» -> «τετράγωνο περίγραμμα» | 13n1: no white on the HUD; the edit-layout frames are theme.ADVISORY (hud.py edit_frames). MANUAL.txt 795 already says "a square frame" without a colour |
| WIRING.md | 113 | STANDBY row "idle + neutral + steering off" -> "idle + neutral first, then the steering held (position servo, OD-10) / the steering motor off" | OD-10 (the table covers both steering actuators) |
| WIRING.md | 191 | "Recommended: 1 + 2 (servo power through the relay) + 3." -> "Recommended BEFORE step 11e (PWM servos + steering motor): 1 + 2 ... + 3. From step 11e the next paragraph applies instead." | OD-2: the servo power is never cut; the old recommendation contradicted the 11e paragraph right under it |
| WIRING.md | 208 | near_shore_m "100 m" -> "50 m - owner 2026-10-31, the parameter's minimum" | 13q1: near shore = 50 m (CLAUDE.md B) |
| WIRING.md | 784 | "neutral first, then steering off)" -> "neutral first, then the steering held - OD-10 - or the steering motor off)" | OD-10 |
| academy/content/quizzes/q01_safety_basics.json | 140-146 | choice «Γκάζι στο ρελαντί και νεκρό, μετά σταματά το πηδάλιο» / "then the steering stops" -> «... μετά το τιμόνι (κρατά τη θέση του)» / "then the steering (it holds its position)"; distractor «Σταματά πρώτα το πηδάλιο» / "The steering stops first" -> «Πρώτα το τιμόνι, μετά το γκάζι» / "The steering first, then the throttle" | OD-10; the right answer (index 0) is unchanged |
| academy/content/quizzes/q04_modes.json | 90-91 | why «πηδάλιο σταματημένο» / "steering off" -> «το τιμόνι κρατά τη θέση του» / "the steering holds" | OD-10 |
| academy/content/lessons/l01_hdg_lock.json | 73-74 | hint «μετά σταματά το πηδάλιο» / "then the steering stops" -> «μετά το τιμόνι (κρατά τη θέση του)» / "then the steering holds" | OD-10 |
| academy/content/scenarios/s01_compass_fail.json | 114-115 | debrief «κλείνει το τιμόνι και περνά σε STANDBY» / "it switches the steering off and goes to STANDBY" -> «το τιμόνι κρατά τη θέση του και περνά σε STANDBY» / "the steering holds its angle and it goes to STANDBY" | OD-10 (compass lost in HDG LOCK) |
| groundstation/assistant/index/manual_el.json, manual_en.json | - | rebuilt (`python tools/assistant_index.py build`) | the manuals changed: the committed assistant index must stay current (`assistant_index.py check` in tools/check.sh). NOTE: every change to MANUAL.txt / MANUAL_EL.txt or academy/content/tour.json changes the assistant's index - rebuild + commit it with the text change |

### Quiz / exam answers

Every `answer` index of q01-q05 was checked against CLAUDE.md: NONE was wrong. Only the wording of
q01 (fault / STANDBY order) and q04 (STANDBY state) was updated for the position servo (above); the
right choice stays index 0. The two exam files (driver_exam.json, captain_exam.json) hold no answers
(pools + pass marks only).

### Checked and found current (no change)

- Lost link / LINK TIME / near shore 50 m / no RELEASE (REMOTE / MANEUVER) / UNCREWED HDG LOCK from
  REMOTE / MANEUVER / the 13p stick answers (trigger, thumb, hat, stick Y) / the mode bar / the white
  CONNECTED lamp / green-only HUD / slamming restore only by the captain's ACCEPT / UAV heading
  without "later steps": MANUAL.txt, MANUAL_EL.txt, lang el + en, the tour, the academy texts.
- The "100 m" lines of the manuals (MANUAL.txt 1145 / 1161 / 1231, MANUAL_EL.txt 796 / 809 / 871) are the
  UNCREWED HDG LOCK / hand-steered AUTO STOP distance (SB_SHDG_STOP_M = the gateway's stop_m 100 m), not
  the lost-link near-shore distance - correct.
- FIRST_TEST.md 5b "STANDBY button: steering off at once" / "emergency stop: steering without power"
  describe the FIRST TEST hardware (the G2 steering-motor driver fed through the e-stop power contact,
  HAL_CAN_SERVOS 0); section 3j holds the DroneCAN servo checks (HOLD, OD-7 / OD-10 / OD-2) - correct.
- PARTS.md 14 / 19 keep the pre-11e wording WITH the "From step 11e (OD-1 / OD-2)" notes - correct as
  written.
- gateway/README.md, groundstation/README.md, bridge/README.md, docs/screens/README.md: no stale item
  found.
- "Confirm the live camera / type the boat name before HELM" (replaced in 13n2): no text left anywhere.
- «ταχύτητα» meaning the gear: none left (MANUAL_EL.txt 1946 states the rule; test_lang L6 guards the lang
  files).

### Left for CL4 (unused lang keys, listed here, not removed)

- groundstation/lang/el.txt 1080-1081 / en.txt 1078-1079: `hud.colour_green` / `hud.colour_white` ("HUD
  colour: white") - the white HUD colour setting is gone since 13n1 (hud.py: "the old white setting is
  gone"); the keys are referenced by no code. CL4 removes them with the other unused keys.
- docs/HANDOVER.md (date / HEAD / status of 2026-11-06): refreshed by CL4, not touched here.

### academy/content/tour.json - stale lines (NOT edited: recorded Greek clips, owner 2026-11-06)

For the owner: each needs a new recording (el) when the text is changed. 7 stops:

| Stop id | Lines | Text (el / en) | What is wrong |
|---|---|---|---|
| s1.btn_layout | 142-143 | «κάθε όργανο έχει ένα τετράγωνο λευκό πλαίσιο» / "every instrument gets a square white frame" | 13n1: no white on the HUD; the edit-layout frames are theme.ADVISORY (blue) - hud.py edit_frames |
| alarm.monitor_tripped | 456-457 | «ρελαντί, νεκρό, τιμόνι σταματά» / "idle, neutral, steering stops" | OD-10: with the DroneCAN position servo CPU1's safe frames HOLD the steering (last published command); "stops" only for the old steering motor |
| alarm.emergency_stop | 498-499 | «STANDBY και καμία άλλη λειτουργία όσο είναι πατημένο» / "STANDBY and no other mode while it is pressed" | INCOMPLETE (not wrong): OD-1 - the e-stop also stops the ENGINE through the hard-wired kill circuit in every profile and cuts the DP thrusters' contactor (the manuals say so since 13v) |
| panel.estop | 600-601 | «Το κόκκινο κουμπί έκτακτης ανάγκης: STANDBY, και όσο είναι πατημένο κανένας άλλος τρόπος» / "... STANDBY, and no other mode while it is pressed" | INCOMPLETE as above (OD-1) |
| tools.engine | 328-329 | «IGNITION, START ..., SHUTOFF και RESTART. Μόνο στο ρελαντί και στο νεκρό.» / "... Only at idle and in neutral." | step 11b: SHUTOFF ON (= stop the engine) is ALWAYS allowed; only IGNITION ON, START and RESTART need idle + neutral |
| s2.assistant (voice_pending) | 237-238 | «όσο ισχύει συναγερμός η απάντηση μένει μόνο κείμενο» / "while an alarm is active the answer stays text only" | W125 (AM5-r2): only the MASTER WARNING lamp (or a sounding / queued callout) silences the voice; a CAUTION alone does not. No clip exists yet (voice_pending) - the text can be fixed before the owner records it |
| alarm.geofence | 531-532 | «Χωρίς πλήρωμα, στο ΚΛΕΙΔΩΜΑ ΠΟΡΕΙΑΣ, το σκάφος βγήκε από τον γεωφράχτη: STANDBY» / "Uncrewed, in HDG LOCK, the boat left the geofence: STANDBY" | INCOMPLETE: 13q3 - also UNCREWED AUTO steered by hand with the shore stick (the fence bit 16 in UNCREWED AUTO) |

Gate for this step: tools/check.sh (full) + tests\acceptance\RUN_ACCEPTANCE.bat quick (the step touches
lang / docs / academy content) - results in PROGRESS.md row CL1.

## CL3 - stray processes (W10 / W11): the paths, the fixes, tools/strays.py, the runner's "no leftovers" check

Test / tool / launcher code only: `git diff` of app/ and hal/ is EMPTY; gateway/ and groundstation/ product code is
unchanged (the one groundstation/ file touched is a test; academy/lesson_app.py + academy/simguard.py are the academy's
LAUNCHER of the simulator). No test was weakened: every change ADDS a cleanup or a check.

### Starting point

No process of this project was running when CL3 started (`tools/strays.py list`: none; also no pythonw). The two
pythonw of W10 (2026-10-27) could not be examined any more: they were gone before CL1 (the owner closed them or the
PC restarted). What follows was found by reading EVERY process-starting site of the repo (`subprocess.Popen` /
`.run` / `os.system` / `startfile`: 162 sites in 59 files) and by running the fixed gate.

### Paths that could leave python(w).exe / autopilot_sim.exe / mediamtx / ffmpeg / gateway workers behind

| # | Path | What stayed behind, when | Fix (test / tool / launcher code) |
|---|---|---|---|
| 1 | tests/acceptance/run_acceptance.py | `subprocess.run(timeout)` ends ONLY the suite's own python on a TIMEOUT: everything under it (fresh_build.py -> tools/check.sh -> autopilot_sim.exe + the python suites + their simulators; soak_system / day_at_sea's gateway with its vision / loudhailer / UAV workers, MediaMTX, ffmpeg cameras) kept running | every suite runs in a `strays.Job` (Windows job object, KILL_ON_JOB_CLOSE; POSIX: process group): a TIMEOUT ends the WHOLE tree; after EVERY suite `strays.cmd_check(suite)` reports what it left (name, PID, parent, age, command), stops it and remembers it; the summary gets the row "no leftover processes (tools/strays.py)" = FAIL when any suite left one |
| 2 | tests/acceptance/fresh_build.py `run()` | the same (`subprocess.run(timeout)`): the clone's `sh tools/check.sh` (3 h limit), `autopilot_sim all`, the python suites | `run()` = a job per step: a TIMEOUT ends the tree; a project process still alive after a NORMAL end is written into the step's output ("LEFTOVER PROCESSES after this step", the step FAILS, rc -3) and then ended by the job. The two demo steps (`demo.py start` / `stop`) run WITHOUT the job (`tree=False`): the demo MUST outlive its starter (pids.json + STOP_DEMO end it) - what it leaves is the acceptance runner's check after the whole fresh-build suite |
| 3 | tests/acceptance/coverage_run.py `sh()` | the same (`subprocess.run(timeout)`, 2 h limit per suite) | a job per suite |
| 4 | tests/acceptance/run_storm.py | the storm.exe workers (4 parallel, up to 2000 episodes) outlive a KILLED runner (LOOP RULE 2 / 5: the 45 min kill) | `strays.dies_with_parent(p)` on every worker |
| 5 | tools/demo_screens.py `Session.__init__` | `demo.start()` succeeded, then an exception BEFORE main()'s `try/finally` reached `Session.close()` (the ground station's construction, a dead gateway login, `kill_tree` ...) = the WHOLE demo stayed: autopilot_sim server 5760, the gateway 5761 with its workers, MediaMTX, the 4 ffmpeg test cameras + demo_world. tools/check.sh runs `demo_screens.py --tour --if-stale` (and restarts the owner's demo when it ran) | the part after `demo.start()` moved into `_open()` under `try/except BaseException: demo.stop(); (restart when it ran); raise`. `Session.close()` also ends a presentation Clip's ffmpeg cut short by an error |
| 6 | academy/lesson_app.py (lessons + scenarios: scenario_app.py inherits it) | "Start the simulator" (`start_sim` -> `self.proc = simguard.start_simulator()`) - `close()` never stopped it: closing the lesson / scenario window left `autopilot_sim server 5760` running (the next lesson said "port busy"; a real leftover an operator makes without any test) | `close()` terminates `self.proc`; `simguard.start_simulator()` puts the simulator into the academy process's KILL_ON_JOB_CLOSE job (`_dies_with_us` -> groundstation/childproc.py): the simulator also dies when the window is killed by Task Manager / a crash. tools/academy_screens.py (terminate in finally) inherits it |
| 7 | tests that `Popen` a simulator / worker without a job object: groundstation/test_e2e.py (2 simulators), test_driver.py, test_screen2.py; academy/test_academy.py `_TestSim` (AC28 / AC31 / AC43 ...), academy/practical/test_practical.py P7; gateway/test_uav.py `Worker` (router_worker.py, UA10 ...), UA24 (video_worker.py with its ffmpeg + MediaMTX) | the child stays when the TEST PROCESS is ended from outside (the coordinator's 45 min kill, Task Manager, a Tk crash) - `finally: kill()` never runs then | `childproc.dies_with_parent(sim)` (groundstation) / `strays.dies_with_parent(p)` (gateway test) right after each Popen. gateway/test_gateway.py already had it (W50s `_Job` + `_Leftovers`); test_look.py / test_screenrec.py kill their 60 s sleep probes in `finally` (a kill from outside leaves them for <= 60 s: not changed) |
| 8 | the assistant (worker, ffmpeg, TTS) | examined, NO lasting leftover path: the worker (groundstation/assistant/worker.py) ends on stdin EOF - `client.stop()` closes stdin, waits 2 s, kills; `_crash()` kills; `App.quit` -> `asst.stop()`; a station killed hard = EOF -> the worker exits after its current op (a model load, an answer <= ANSWER_TIMEOUT_S); the TTS = `tts.synth(timeout=TTS_TIMEOUT_S)` Piper child; the academy's ffmpeg = `subprocess.run(timeout=AUDIO_TIMEOUT_S)`; the voice WAVs are files, not processes (AM5-r) | no change (product code; nothing proven). The gate's check after test_assistant (AS1-AS31 incl. the real worker of AS4 / AS24) is the empirical proof: see "Results" |
| 9 | NOT changed, by design | demo/demo.py `launch()`: the demo processes MUST outlive START_DEMO (pythonw demo.py start exits; pids.json with pid + creation time, STOP_DEMO kills the trees); the gateway's own workers (vision, loudhailer, UAV router, video) end on stdin EOF when the gateway dies; groundstation/screenrec.py's ffmpeg has its own kill-on-close job + atexit (13n4b) | `strays.py list` shows a running demo as [DEMO] and `stop` keeps it unless `--all`; the gate never counts the demo as a leftover |

### tools/strays.py (new)

- `list` / `stop [--all]` / `mark --begin` / `check SUITE` / `report` / `selftest`; no third-party module (psutil is not
  installed here): Windows = one PowerShell `Get-CimInstance Win32_Process` call (pid, parent, name, executable,
  command line, creation time), Ubuntu = `ps`.
- A process is OURS when its program is one the project starts (python / pythonw, autopilot_sim, storm, mediamtx,
  ffmpeg / ffprobe, piper) AND its executable or command line is inside THIS repo folder (`C:\trygons\TRYGONS_C2`, both
  slash styles, the MSYS `/c/trygons/trygons_c2` form; followed by a separator - `C:\trygons\TRYGONS_C2_other` is
  NOT ours) or names a test folder of ours in the TEMP folder (`tempfile.mkdtemp` prefixes `trygons_` / `gwtest_`),
  OR its parent is ours (a gateway worker, a test's simulator, an ffmpeg with no path in its command line). Never:
  this process and its ancestors (the shell / runner), another folder (`../review-wt`, a clone next to the repo), the
  owner's other Python programs (no marker, parent not ours). The demo (demo_run/pids.json, pid + creation time) is
  shown as [DEMO], stopped only with `--all`, never a leftover.
- Stopping = `taskkill /PID /T /F` per process, parents first, then a wait of up to 8 s with the creation time
  re-checked (never a reused pid).
- Library for the test / tool code: `Job()` (KILL_ON_JOB_CLOSE job: `popen` / `kill` / `close`), `dies_with_parent(p)`
  (one job per process: the child ends with THIS process whatever ends it), `ours()` / `processes()`.
- `selftest` (also in tools/check.sh): positive = a python with the repo folder in an argument, one with a
  `%TEMP%\trygons_*` folder, the child of a project process; NEGATIVE CONTROLS = a foreign python (no marker, parent =
  the excluded selftest process - like the owner's other Python programs), a python of the SIBLING folders
  `..\review-wt` + `TRYGONS_C2_other`, the child of a foreign process - none may be matched; `in_repo` / `program`
  table; `mark` -> a new project process -> `check "suite X"` names it, stops it, `check "suite Y"` is clean, `report`
  lists suite X only; `dies_with_parent`: a grandchild in the job dies with its killed parent, WITHOUT the job it
  survives (so the selftest proves the mechanism matters).

### The runner's "no leftovers" check

- tools/check.sh: `strays.py selftest` + `mark --begin` at the start (what already runs is listed as "already
  running before this run (not a leftover of it)" and never blamed), `strays_check "<suite>"` after each of the 25
  process-starting suites (sim tests, hal_pc checks, charts, lang, boats, screen 2, look, track, screenrec, yes / no,
  bench stack, hailer, assistant index / tests, sound, avoid, osdep, setup, tour pictures, academy, practical,
  loudhailer, UAV, UAV station, gateway) = "LEFTOVER after <suite>: <name> PID <pid> (parent) age <s> -- <command>",
  stopped, `fail=1`; at the end `strays.py report` = "NO LEFTOVER PROCESSES: n suite(s) checked" or the list
  (suite, name, PID, age, stopped / STILL RUNNING) -> GATE: FAILED.
- tests/acceptance/run_acceptance.py: the job per suite + the same check after every suite + the summary row
  "no leftover processes (tools/strays.py)" (PASS / FAIL; `test_results/strays_state.json` keeps the details).
- fresh_build.py: per step (above).

### Results of this run (the fixed gate on the working tree)

- Before the gate: no project process running ("already running before this run": none).
- `strays.py selftest`: OK (all positive cases matched, all three negative controls NOT matched, mark / check /
  report, the job object: the grandchild dies with its parent only WITH the job).
- Leftovers found per path by the gate's per-suite check: NONE - "NO LEFTOVER PROCESSES: 25 suite(s) checked"
  (every suite incl. test_assistant with the real Whisper / language-model worker, test_uav with the router / video
  worker / MediaMTX, test_gateway's 155 tests, the academy's real-simulator tests, the tour-picture check). So the
  paths of rows 1-7 leave something behind only on the FAILURE branches (a timeout, a kill from outside, an exception
  before the cleanup, a window closed): exactly the W10 situation, which a clean run never shows. They are fixed in
  the code; the gate now catches any new one.
- tools/check.sh: everything passed (sim all 385 / --bridge / --can / --can-fast, TriCore + stack + memory, hal_pc
  25, charts 6, lang 6, boats 12, screen 2 28, look 26, track 12, yes / no 7, bench stack 7, hailer 8, assistant 36 +
  1 N/A (AS4b firewall rule, W91), sound 17, avoid 15, osdep 12, setup 53, tour pictures current, academy 56,
  practical 8, loudhailer 22, UAV 42 + 15, gateway 155 - no stall) EXCEPT the known environmental SR8 (test_screenrec
  9 / 10: the real h264_nvenc recording makes 1 segment in 12 s on this PC - the same failure as in AM2 / AM6 / CL1,
  identical alone; screenrec.py / test_screenrec.py untouched) - re-run alone per LOOP RULE 3: the same, recorded.
- RUN_ACCEPTANCE quick (on the CL3 commit, ACCEPT_CLONE_URL = this folder): 8/8 PASS in 6 min - ADS excludes +
  iLLD, fresh build from the clone (sim all / --bridge, MediaMTX, charts, planner, 3 gateway sections, test_driver,
  START_DEMO / STOP_DEMO from the clone: no step left a process), storm 200 episodes 0 fails (I59 / I63 key cases),
  time wrap, day at sea, gateway storm I80-I84, and the NEW row "no leftover processes (tools/strays.py)": NO LEFTOVER
  PROCESSES in 7 suites.

### W10 / W11

- W10 (two pythonw running since 2026-10-27, "probably your ground stations"): they were gone before CL3 started
  (no project process ran: `strays.py list`), so their origin is not proven. Everything a pythonw of this project
  can be (START_DEMO's `demo.py start --splash`, the two demo ground stations, TOUR.bat's tour_app, the academy
  windows) shows in `python tools/strays.py list` with its command line and age, and `python tools/strays.py stop`
  ends them (the demo kept unless `--all`) - the owner no longer needs Task Manager for them. The one path found that
  leaves a simulator behind WITHOUT any test (an academy lesson / scenario window closed) is fixed (row 6).
- W11 (flaky test_driver I4 / I12 / I19 on this PC): CL3 removes the suspected cause (stray simulators / stations
  of an earlier run on the same ports / CPU); from now on the gate FAILS on a leftover, so a flaky run can no longer
  have an unseen stray behind it. The failure rates under load themselves = CL2 (next).
