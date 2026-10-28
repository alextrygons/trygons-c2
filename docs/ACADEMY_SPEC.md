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
- Chart PHOTOS (docs/daskalio/, depth-chart photos of the Daskalio area): for the academy test area, the demo
  map and the 3D sim ONLY. A photo is NOT a navigation chart: it is NEVER used for the shallow-water,
  collision / route or any other safety check unless it has been georeferenced AND checked against an
  official chart (a separate, reviewed step). The safety checks use only charts/ (tools/chartprep.py).
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

## Design decisions (step 15, 2026-10-28) - where the owner's text above leaves things open
Recorded by step 15; the owner's text above stays binding. Open owner questions: PROGRESS.md
"WAITING FOR OWNER" W27-W31 (recommended defaults used until answered).

D1 Folder + build. academy/ = Python only (flat modules like groundstation/, run with academy/ on
   sys.path), excluded from the ADS build in every .cproject configuration (check_ads_excludes.py +
   test AC13). Its tests academy/test_academy.py run in tools/check.sh.
D2 ONE door out: academy/simguard.py is the only academy module allowed to import a network module
   or open a recording. Allowed: the simulator on THIS PC only = host 127.0.0.1 / localhost / ::1 AND
   port 5760 (autopilot_sim server, USB link) or 5761 (the demo gateway's station port); on 5761 the
   gateway welcome's boat name must be a simulator name (SIM_BOAT_NAMES = "trygons-demo", demo/demo.py)
   and never a boats/ name - so a real gateway tunnelled to localhost is still refused. Refused: LAN,
   Tailscale 100.64.0.0/10, every boats/*.json address, host names, every other port (5762 = the sim
   AURIX's gateway UART: the academy never plays gateway). Recordings: ordinary files with a recording
   extension (.tlg, .csv, .jsonl, .json, .mp4, .mkv, .wav, .log, .txt) - never COMn, /dev/..., tcp:,
   usb:, a folder. On a gateway PC (its config file gateway.json exists in %PROGRAMDATA%\trygons or
   /etc/trygons) the academy refuses everything (spec J "never runs on a boat gateway").
D3 The proof (tests AC1-AC3): (1) static - no academy module except simguard imports socket / ssl /
   serial / http / urllib / requests / asyncio / subprocess / multiprocessing / selectors ... nor loads
   code by name (__import__, importlib, exec, eval, os.system ...); (2) transitive - no academy import
   reaches such a module through ground-station / gateway / demo / tools modules (the checker is
   proven to see link.py, gwclient.py, boats.py, aurix.py, gateway.py as boat links); (3) runtime -
   loading every academy module + its content with socket.connect / create_connection patched makes
   0 connection attempts and loads no boat-link module. Starting the simulator from the academy (a
   process) will go through simguard too (a later step adds it there, tested).
D4 PRACTICAL TEST exception (spec G, step 21): its own folder academy/practical/, the ONLY academy
   code allowed on the normal boat link, read-only (telemetry + audit log) + the one confirm-only
   request (PROTO_T_CONFIRM_ONLY 0x30); no other academy module may import it (AC14) and it must
   ship its own academy/practical/test_practical.py (no bypass of any safety rule).
D5 Banner: the text lives in academy/lang/el.txt / en.txt key academy.banner = exactly
   «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» (EN DASH U+2013) / "TRAINING – SIMULATION"; academy/banner.py holds the
   same text as constants and AC8 checks both. Every academy screen draws banner_text() at the top,
   always visible, in the theme's CAUTION colour (groundstation/theme.py, one look); AC8 fails for an
   academy module that uses tkinter without it.
D6 Languages: academy/lang/el.txt (default) + en.txt, the ground station's file format; same keys
   and {names} (AC9). Content files carry their own trainee texts in both languages ("<name>_el" +
   "<name>_en"); English stays in logs / audit.
D7 Content format: JSON with "_help" / "_note..." keys as comments (the boats/*.json convention), one
   file per lesson / quiz pool / scenario / exam + limits.json + test_area.json under academy/content/
   (lessons/, quizzes/, scenarios/, exams/); academy/content.py checks every file (AC10): kind, _help,
   both languages, lessons + scenarios "runs_on": "sim", quiz answers index their choices. Step 15
   ships commented EXAMPLE files (names example_*); the real content comes in steps 18 / 19 / 21.
D8 Levels (step 15 = model only, academy/accounts.py): OBSERVER < DRIVER < CAPTAIN; licence = CAPTAIN
   only; an unknown level or an expired one (or a level without an expiry) = OBSERVER; the record
   pushed to the gateway = user, level, licence, expiry. Storage, logbook, instructor view and the
   push itself = step 16 (push_to_gateway raises NotImplementedError until then).
D9 Chart photos (docs/daskalio/): the test area picture, the demo map and the 3D sim only.
   content/test_area.json carries "not_for_navigation": true and "placeholder": true (the Daskalio /
   Keratea polygon and the ramp points are NOT surveyed); the practical test refuses while the area is
   a placeholder. AC11: no safety code (app/, hal/, gateway/, groundstation/chart.py,
   tools/chartprep.py) mentions docs/daskalio or reads the academy.
D10 Ground station menu "Academy": NOT added in step 15 (the spec gives no screen for this step);
   step 16 adds it (display only: it starts the academy window, which shows the banner and uses
   simguard - the station's own boat link is never handed to the academy).
D11 Academy data on the shore PC (step 16): academy/data/ (inside the excluded academy/ folder,
   gitignored, never in git; TRYGONS_ACADEMY_DATA overrides it for tests), one JSON file per account
   + an append-only logbook; the backup / sync target = W28.
