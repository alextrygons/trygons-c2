# Trygons C2 – Academy & 3D Simulator: handover

This file carries everything decided so far about the training academy and the 3D simulator into a new chat or project. Upload it to the project's files together with CLAUDE.md, MANUAL.txt, MANUAL_EL.txt (once it exists) and WIRING.md from the repository.

---

## 1. Context in short

- **Project:** Trygons C2, a USV/autopilot controller for a small outboard boat (Saronic Gulf, Attica). AURIX TC375 lite kit (safety-critical control), a fanless N150 mini PC gateway on board (cameras, AIS, sync, eWaSR vision), and a shore ground station (driver HUD screen, captain screen, map window).
- **Code:** GitHub `alextrygons/trygons-c2`, written with Claude Code in VS Code. CLAUDE.md holds the rules; PROGRESS.md tracks long tasks.
- **Modes:** STANDBY, MANUAL (joystick), HEADING LOCK, AUTO (waypoint program), REMOTE MANUAL, TRAINING (outboard calibration – not the academy).
- **Safety rules that the academy must teach:** STANDBY button always wins; onboard joystick always wins; startup power limit 30 % fwd / 5 % rev; raising power, starting AUTO and enabling remote driving need the onboard joystick CONFIRM; gear only changes at idle; captain actions affecting driving need driver-screen ACCEPT.
- **The user:** knows programming, new to embedded/VS Code; prefers step-by-step guidance; Greek speaker; Claude Pro (watch the weekly allowance).
- **Prompt conventions:** every prompt for Claude Code comes in a code box; before it: "type `/clear`, press Enter, then paste". Interrupted work: "continue from PROGRESS.md".

## 2. Decisions made

### Academy («Σχολή Χειριστών»)
- Built into the ground station, runs **only against the simulator**, with a permanent banner «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ». It can never send commands to the real boat.
- **Greek by default**, English option (same language files as the rest of the ground station).
- **Many trainees:** personal accounts, logbook, instructor view, exportable certificates.
- **Certificates linked to gateway roles:** Παρατηρητής (Observer, view only) → Χειριστής (Driver: MANUAL + HEADING LOCK) → Κυβερνήτης (Captain: AUTO, remote driving, power raise, can examine others).
- **Content:** guided tour with Greek voice; interactive lessons in the simulator (the app checks the action really happened); quizzes (multiple choice, picture, **alarm-sound recognition**); emergency scenarios with scoring; final exam.
- **Real-trip videos:** replay of saved events (all cameras, time-synced with the log) with the HUD redrawn from logged data; **decision points** ("τι θα έκανες τώρα;"); **spot the hazard** (click on the obstacle); best-practice clips. A **lesson editor** for the instructor (trim, pause points, questions, hazard areas, camera choice, optional face/registration blur).
- **Practical test on the real boat:** the same tasks as in the simulator, as a checklist; the boat records the whole check ride as evidence; the **captain approves each task with their own login** and confirms the final approval with the **onboard joystick CONFIRM**; only then is the role upgraded. Safe conditions: calm water, startup power limit, captain beside the trainee with the onboard joystick. Optional expiry/re-check and a logbook of hours.
- **Order:** build after the HUD + Greek prompt has finished.

### 3D simulator (later, in stages)
- **Godot** (free), low-poly graphics. The 3D world **does not replace the controls**: the real autopilot code + boat simulator compute the physics; Godot renders 4 virtual cameras (FRONT-1, FRONT-2, REAR-LEFT, REAR-RIGHT) into the same camera pipeline (MediaMTX), so trainees drive with the **real driver screen**, HUD, sounds and the Thrustmaster.
- World: sea with waves matching sea state, AIS ships (also on the map, triggering collision logic), swimmer / kayak / jet ski without AIS, buoys, land and harbour — ideally **the user's real place**, built from the filming below plus free map/depth data.
- **Trailer challenge:** load the boat onto a trailer at the ramp, with wind and current; scored on alignment, speed at contact and corrections.
- Missions, weather, day/night, scores and replays feeding the academy logbook.
- Stages: (1) basic world + boat + 4 virtual cameras into the C2; (2) traffic, hazards, weather, day/night; (3) trailer challenge; (4) academy integration. Later: tune the simulated boat with real TRAINING data.

### Open questions
- Exact location of the home marina and ramp (for the 3D world and the practical test).
- Drone available or not.
- Optional: certificate expiry period.

---

## 3. ACADEMY PROMPT (for Claude Code)

Run it only after the HUD + Greek prompt has finished.
**Step 1:** type `/clear` and press Enter. **Step 2:** paste:

```
Read CLAUDE.md, MANUAL.txt, MANUAL_EL.txt and PROGRESS.md. Build the operator training ACADEMY («Σχολή Χειριστών») into the ground station. Show a short plan first, then proceed without waiting. One commit per section, keep PROGRESS.md updated after every section, all existing tests must keep passing.

GROUND RULES
- The academy runs ONLY against the simulator (or recorded data). It can never connect to or send commands to the real boat; a permanent banner «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» is shown. Add a test that proves no academy code path can reach a real-boat link.
- Greek by default, English option, using the existing language files (all academy texts in el/en files, editable by hand).
- All lessons, questions, scenarios and exams are content files (plain, commented, editable), not hard-coded, so I can add and change them without code.

=== SECTION A: ACCOUNTS, ROLES, CERTIFICATES ===
A1. Trainee accounts (many users): name, login, logbook (lessons done, scores, attempts, exam results, dates, hours), stored on the shore PC and included in the normal backup/sync.
A2. Certificate levels linked to the gateway roles: OBSERVER (view only, no training needed) -> DRIVER (MANUAL + HEADING LOCK) -> CAPTAIN (AUTO, remote driving, power raise, may examine others). The gateway refuses HELM / remote driving / power raise / AUTO start for users without the needed level (keep all existing safety checks; this is an extra gate).
A3. Instructor view: list of trainees, progress, weak areas, attempts, export/print certificates (PDF), optional expiry date and re-check reminder.

=== SECTION B: GUIDED TOUR ===
B1. Step-by-step tour of the driver HUD, captain screen and map window: each element highlighted with a Greek explanation, spoken by the Greek voice (existing TTS), next/back, can be repeated.
B2. Covers every HUD item, button, lamp, alarm, mode, the joystick and the panel buttons.

=== SECTION C: INTERACTIVE LESSONS (simulator) ===
C1. Short lessons where the app asks the trainee to DO something in the simulator and verifies it from the sim state (e.g. engage HDG LOCK, change course +10, take over with the joystick, hand back, raise the power limit with CONFIRM, react to an alarm). Hints when stuck, retry when wrong.
C2. Lessons grouped by certificate level; a level's lessons must be completed before its exam.

=== SECTION D: QUIZZES ===
D1. Question types: multiple choice, picture questions (screenshot with a marked area: "what does this show?"), alarm-sound recognition (play a sound/callout: "which alarm is this?").
D2. Random order, question pools, explanations after answering.

=== SECTION E: EMERGENCY SCENARIOS (simulator) ===
E1. Scripted scenarios: link lost while remote driving, compass failure, AIS ship on collision course, shallow water ahead, GPS spoof, slamming, joystick fault, power limit refusal, gateway lost.
E2. Scoring: correct actions, reaction time, mistakes; replay of the attempt with the HUD and a short debrief.

=== SECTION F: REAL-TRIP VIDEO LESSONS ===
F1. Use saved events / ring recordings (all cameras, time-synced with the boat log). Replay with the driver HUD redrawn from the logged data.
F2. Decision points: the replay pauses at a set moment, the trainee chooses an action, the video continues and shows what really happened, with an explanation.
F3. Spot the hazard: a frame is shown, the trainee clicks the hazard; checked against a marked area.
F4. LESSON EDITOR for instructors: pick a saved event, trim, choose cameras, set pause points, write questions/answers (el + en), mark hazard areas, optional face/registration-plate blur, publish into a lesson. Training clips are trimmed copies (configurable resolution), so the course stays small.

=== SECTION G: EXAMS AND PRACTICAL TEST ===
G1. Final exam per level: mixed quiz + scenarios, pass mark configurable, limited attempts with cool-down (configurable).
G2. PRACTICAL TEST on the real boat (this part uses the normal boat link, NOT the academy simulator mode): a checklist of the same tasks as in the simulator (leave harbour, HDG LOCK course change, joystick takeover and hand-back, react to an alarm triggered by the examiner, return, approach and load onto the trailer). The boat records the whole check ride (all cameras + log) and links it to the trainee's logbook.
G3. The examiner must be a CAPTAIN-level user, logged in with their own account. They approve each task in the app; the final approval also requires the onboard joystick CONFIRM within 10 s (same rules as other CONFIRM actions). Only then is the role upgraded automatically, and it's logged in the audit log.
G4. Conditions shown before the test: calm water, startup power limit, examiner on board with the onboard joystick. The test can be aborted at any time; nothing in the test may bypass any safety rule.

=== SECTION H: HOOKS FOR THE FUTURE 3D SIMULATOR ===
H1. Define a simple interface so a future 3D simulator (Godot) can register missions, report scores and attempts, and be selected as the simulator for lessons and scenarios. Document it in docs/academy_sim_interface.md. Don't build the 3D world now.

=== SECTION I: MANUALS AND TESTS ===
I1. Add an academy chapter to MANUAL.txt and MANUAL_EL.txt (for trainees and for instructors: how to create lessons from real trips, how to run the practical test).
I2. Tests: academy can't reach the real boat; role gating works (and never weakens existing safety); lesson verification; scoring; certificate/role upgrade only with examiner approval + onboard CONFIRM; content files load in both languages with no missing keys.
I3. Screenshots of the tour, a lesson, a quiz, a scenario debrief, a video lesson with a decision point and the instructor view in docs/screens/academy/.

FINAL REPORT: what was built per section, where the content files are and how to add my own lessons, screenshots location, anything uncertain. Commit and push.
```

---

## 4. FILMING GUIDE (your place, for the 3D world, video lessons and the practical test)

### Equipment
- Phone with 4K video and stabilisation; a drone if you have one; the boat's own cameras (they record with the HUD data, which makes the best lessons).
- Phone apps: a GPS/compass app to note positions, and a level/inclinometer app for the ramp slope.
- Tape measure (≥ 10 m) or laser distance meter; notebook or a simple spreadsheet for measurements.

### A. Overview of the place (for the 3D layout)
1. From the air (drone, 50–100 m up): slow 360° circle around the marina and the ramp; straight top-down photos covering the whole harbour (overlapping ~60 %).
2. Without a drone: panoramas from the highest points nearby (breakwater end, a hill, a building), plus the satellite view (screenshot from a map service) as reference.
3. Note GPS positions of key points: ramp top and bottom, pier ends, harbour entrance (both sides), buoys, fuel dock, any shallow spots you know.

### B. The ramp (most important for the trailer challenge)
1. Measurements: ramp width, length (from top to water line and under water if possible), slope in degrees, water depth at the bottom end, distance to the nearest pier/wall/obstacle on each side.
2. Video: walk slowly down the ramp filming ahead; then film the ramp from the water side (from the boat or a pier) at 20 m, 10 m and 5 m.
3. Photos: straight-on photos of the ramp surface, walls and nearby objects (for textures).
4. Note anything that matters when docking: usual wind direction, current, wakes from passing boats, lights at night.

### C. The trailer and the boat
1. Trailer: overall length and width, distance between the guides/rollers, height of the rollers, position of the winch post; photos from all four sides and from above.
2. The trailer in the water at loading depth: how much is submerged, from the side and from behind.
3. The boat: length, width, outboard position; photos from all sides; the boat on the trailer (side, front, rear).

### D. The harbour and approach
1. From the boat, with its cameras recording: leaving the ramp, the harbour exit, the open-water approach back, and the full approach onto the trailer — several times, in different conditions.
2. Landmarks you use for orientation (buildings, lights, buoys) — photos from the water.
3. Typical traffic: ferries, fishing boats, swimming areas in summer, kayak spots.

### E. Conditions to capture over time
- Calm and windy days, different wind directions.
- Morning, noon, evening, dusk and (carefully) night with lights.
- Sun glare towards the camera (east in the morning, west in the evening).
- Busy days (traffic) and quiet days.

### F. Sounds (for realism in the simulator)
- Your outboard at idle, low speed and cruise (phone close to the engine, and from the helm position).
- Harbour ambience, waves on the hull, the trailer winch.

### G. Practical tips
- Start every boat-camera clip and phone clip with a clap or a phone stopwatch in view, so they can be synced later.
- Name files with date, place and what it shows (e.g. `2026-11-02_ramp_approach_wind-N_03.mp4`).
- Keep the measurements in one table with photos referenced.
- Privacy: avoid filming identifiable people up close; ask the marina for permission; the lesson editor can blur faces and registrations later.
- Safety first while filming from the boat: one person drives, another films.
