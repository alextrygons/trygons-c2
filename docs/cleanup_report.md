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
