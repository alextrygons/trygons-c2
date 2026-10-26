# Trygons C2 Academy - specification (owner's spec, 2026-10-26, updated for the current C2)

Design in CLAUDE.md section "C2 extensions + Academy", part S. Implemented in steps 15-22 of the task
"C2 extensions + Academy" (PROGRESS.md). This file is the owner's text; the rules here are binding.

## GROUND RULES
- The academy runs only against the simulator or recorded data, never a real-boat link (banner
  «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ»; a test proves no academy path reaches a boat link).
- Exception: the PRACTICAL TEST (G2-G4) is a separate module on the normal boat link with ALL safety
  checks; it only observes (telemetry + audit log) and grades, never sends its own driving commands; a
  second test proves it can't bypass any safety rule.
- Greek default, English option, all texts in lang files.
- All lessons, questions, scenarios, exams, limits = commented content / config files.
- Existing safety always wins (STANDBY button, onboard joystick priority, 30 / 5 % start power limit,
  onboard CONFIRM, gear only at idle, captain's screen 1 ACCEPT for copilot actions); the academy only
  ADDS gates. Software can never block the onboard joystick.

## A. Accounts
Accounts on the shore PC (logbook: lessons, scores, attempts, exams, dates, hours, logged driving
time), in backup / sync. Levels OBSERVER -> DRIVER -> CAPTAIN (CAPTAIN = gateway "licence": HELM,
REMOTE, MANEUVER, power raise, examiner). The gateway user list stays the authority: the academy pushes
level + expiry; the gateway uses its last copy if the academy is unreachable; never revoked mid-trip
(takes effect at next login / HELM take). Instructor view: trainees, progress, weak areas, expiry,
currency, certificate PDF.

## B. Guided tour
Guided tour of screen 1 (HUD) and screen 2 (map + corner panels + tools window), Greek voice (existing
TTS), next / back; every HUD item, button, lamp, alarm, mode, joystick, panel button.

## C. Interactive simulator lessons
Verified from the sim state (HDG LOCK, course +10, takeover, hand-back, power raise with CONFIRM, alarm
reaction), hints, retry; grouped by level, required before the exam.

## D. Quizzes
Multiple choice, picture (marked area), alarm-sound recognition; pools, random order, explanations.

## E. Emergency scenarios
Remote link lost, compass failure, AIS collision course, shallow water, GPS spoof, slamming, joystick
fault, power limit refusal, gateway lost, swimmer / kayak (no AIS) near the beach; scoring, reaction
time, replay + debrief.

## F. Real-trip video lessons
From saved events (all cameras, synced log, HUD redrawn), decision points, spot the hazard, LESSON
EDITOR (trim, cameras, pause points, questions el + en, hazard areas, face / plate blur, small trimmed
copies).

## G. Exams and the PRACTICAL TEST
Exam per level (pass mark, limited attempts, cool-down). PRACTICAL TEST on the real boat, CREWED profile
only: leave harbour, HDG LOCK course change, joystick takeover + hand-back, examiner-triggered alarm,
return, trailer approach + loading in MANEUVER; whole ride recorded and linked to the logbook.
Examiner = valid CAPTAIN with own login, approves each task; final approval + onboard joystick CONFIRM
within 10 s via a new confirm-only request that changes nothing on the boat; then the role upgrade,
audit-logged. Shown before start: calm water, start power limit active, examiner on board, inside the
test area, wind within limits. Abort anytime.

## H. Interface for a future 3D simulator (Godot)
Missions, scores, attempts, selectable as the lesson simulator; import the owner's DJI drone photos of
the area as map / terrain; docs/academy_sim_interface.md. No 3D world now.

## I. Manual and tests
Academy chapter in MANUAL.txt + MANUAL_EL.txt (trainees + instructors); tests for all rules above (no
boat link, no bypass, role gating, expired / non-current refused, upgrade only with examiner + CONFIRM,
nothing online upgrades, area / wind refusals, both languages no missing keys); screenshots in
docs/screens/academy/.

## J. Online academy
Online academy through the remote link, content only, no route to any boat (Tailscale rules: trainee
devices reach only the academy host, never a boat; the academy never runs on a boat gateway). 2FA for
CAPTAIN + instructor. Exams online only when an instructor marks the session supervised. Role upgrades
only via the practical test. Validity 12 months (config), reminder 30 days; DRIVER renewal = online
refresher + quiz + one emergency scenario; CAPTAIN = same yearly + practical check ride every 24 months;
expired > 3 months = full practical test; expired = OBSERVER. Currency: no logged driving in 90 days ->
supervised refresher trip (CAPTAIN on board approves + onboard CONFIRM, audit-logged) before solo rights
return. Test area config: Daskalio bay + ramp, Keratea (clearly marked PLACEHOLDER polygon + ramp top /
bottom points), max wind, blocked directions (default: strong north AND from the right side at the
ramp), allowed hours; wind is typed in + confirmed by the examiner; instructor tool to set the area (map
taps, boat GPS "add point", .geojson import; CAPTAIN only, audit-logged); the test refuses outside the
area / limits and says why, and refuses until the area is set; leaving the area during a test = alert +
test stopped, never a mode change.
