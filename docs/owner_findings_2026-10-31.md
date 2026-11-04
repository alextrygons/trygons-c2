# Owner findings + decisions 2026-10-30 / 31 (demo on the owner's laptop, guided tour, chat with Claude)

## GENERAL RULES for all these steps
- The owner's decisions below are final; where they change a CLAUDE.md rule, CLAUDE.md is updated in the step that implements them.
- Everything that changes the boat's behaviour is in the AURIX (app/), checked by sim + storm tests; the station only shows / sends.
- STANDBY always wins, is never hidden, never needs anyone's approval.
- Colour code (E1) applies to everything new.

## A. 13l High-DPI freeze (display)
Owner laptop ~3840x2400. START_DEMO opens but all 3 windows freeze; screen 1 shows only a 200x150 HUD in the top-left corner, the rest black / white (not repainted). Cause: theme.dpi_aware() + driver.py render_pil() composes the full window in Pillow every 100-200 ms on the Tk thread (with the captain screen and the video) - ~6x the pixels of 1600x900. Workaround the owner uses now: pythonw.exe compatibility DPI override "System".
Fix: (1) the screen 1 HUD (and the captain map if Pillow) composed at a capped internal size (setting render_max_px, default 1920 on the long side) and shown scaled (Tk zoom or one Pillow resize - measure which is cheaper); mouse hits scaled the same way; (2) never draw before the first real <Configure> size; (3) the adaptive tick measures render + PhotoImage together, a slow frame never starves the other windows; (4) tests: 3840x2400 render < 60 ms on the dev PC, mouse hits correct at 1600x900 and 3840x2400, with and without the "System" override; (5) MANUAL high-DPI note.

## B. 13m Alarm the owner could not silence (display)
(a) Screen-2 ACK ALARM only sends T_ALARM_ACK to the boat; this PC is silenced only by app.master_ack() from screen 1 (Space / A / GENERAL WARNING lamp) - with screen 1 closed (e.g. the demo's ON BOARD station) nothing silences that station. (b) app.alarm_tick(): the fallback winsound.Beep every 1 s ignores the acknowledge. (c) After app.disconnect() (owner answered "not this boat") self.t / t_rx keep the LAST telemetry -> old alarms + "NO DATA FROM THE BOAT" keep ringing with no boat.
Fix: (1) ACK ALARM on screen 2 also calls master_ack() before the boat ACK; Space / A on screen 2 = master_ack(); (2) the fallback beep only while lamps.on(WARNING); (3) disconnect() clears the telemetry (self.t = None, t_rx = 0, lamps + sound warnings reset, HUD / plots "not connected", no alarm rows); (4) tests: screen-2 ACK silences the repeat, fallback beep stops after ack, a newer alarm sounds again, disconnect -> silence + no rows, reconnect -> alarms from fresh data only.

## C. 13q BOAT FIRMWARE (each sub-step: sim + storm tests, safety review)
### C1. 13q1 Lost link + "max time without connection"
- near_shore_m 100 -> 50 m.
- New captain setting LINK TIME = the link-loss detection time: default 3 s, the HELM captain may set up to 60 s; RAM only / this session only; back to 3 s on ground-station restart, HELM change and AURIX reset; NOT applied near shore (< 50 m: always 3 s); REMOTE MANUAL data freshness (0.3 s -> neutral) unchanged. Shown permanently while > 3 s: "LINK TIME 30 s (OVERRIDE)" (screen 1 + 2).
- After detection: idle + neutral + alarm; near shore -> engine kill at once; open sea -> engine kill if still no link 10 s after neutral (lostlink_grace_s 10, unchanged); link back in time -> STANDBY in neutral, waits, never resumes by itself. CREWED unchanged (alarm only). Max total = LINK TIME + 10 s.
### C2. 13q2 Entering / leaving REMOTE MANUAL (and MANEUVER)
- CREWED (someone on board with the onboard joystick): unchanged = request + onboard CONFIRM.
- UNCREWED: NO request / CONFIRM procedure. The HELM captain (licence) selects REMOTE (or MANEUVER) in the mode bar = one action, immediately; only condition: shore stick centred + TWCS lever at neutral at that moment (else "centre the stick", nothing starts). Caps 20 % / 5 %, rate limits, 0.3 s freshness, STANDBY wins - unchanged.
- There is NO "release" anywhere (removed from the joystick: nobody on board 99.9 %, an accidental release mid-manoeuvre is dangerous; no screen button either). Leaving REMOTE = selecting another mode in the mode bar: the AURIX must allow a direct switch REMOTE -> HDG LOCK (holds the current heading) / AUTO (AUTO start rules) / DP / STANDBY. A copilot's mode change while the captain drives = a driving change -> captain's ACCEPT (STANDBY free).
### C3. 13q3 Shore stick override inside AUTO / HDG LOCK (UNCREWED, shore joystick only; the onboard joystick keeps "takes full MANUAL at once")
Owner: "In AUTO I can always drive by hand, except throttle up; to stop AUTO totally I choose another mode."
- AUTO (waypoint route): stick X beyond the override threshold (> 15 %) = temporary steering (REMOTE rate limits); stick centred for 2 s -> AUTO resumes and rejoins the route line to the active waypoint (like after an avoidance turn). HUD «ΧΕΙΡΟΚΙΝΗΤΟ ΤΙΜΟΝΙ – AUTO σε αναμονή». The captain's steering overrides the avoidance heading while held.
- HDG LOCK: stick X turns the boat; the TRIGGER pressed while steering (no request pending) = the CURRENT heading becomes the new lock heading (HUD «ΝΕΑ ΠΟΡΕΙΑ 245°», heading-tape mark moves); released WITHOUT the trigger -> after 2 s back to the OLD heading.
- TWCS lever = power CEILING in AUTO / HDG LOCK: reduction only, never above the mode's demand / power limit; lever up = full power again; lever at idle = steers at idle; pickup: the ceiling only engages once the lever is moved below the current throttle. The lever never switches modes (not even into reverse).
- Vision slowdown, shallow / navreq caps, geofence alarm, STANDBY, data freshness -> neutral: all still apply.
### C4. 13q4 Slamming power restore only with the captain's ACCEPT (app/slam.c)
- Today after 30 s calm the reduction is lifted automatically (+5 % / 10 s). New: reduction unchanged (immediate, automatic); when calm 30 s -> a request on screen 1 «Θάλασσα ηρεμότερη – επαναφορά ισχύος στο 60 %;» (60 = the level before the reduction). ACCEPT -> back gradually at the normal up rate; new impacts -> stop + reduce again. REJECT / no answer in 10 s -> stays reduced; framed «ΕΠΑΝΑΦΟΡΑ ΙΣΧΥΟΣ» button in the bar; asked again after the next 2 min calm. AUTO gets its speed back after the ACCEPT. Onboard kick-down unchanged. CLAUDE.md "nothing raises the throttle by itself": remove the slamming auto-restore exception; SL tests + storm.
### C5. 13q5 MANEUVER gear on the shore stick
- In MANEUVER (throttle always idle): stick Y full forward held ~0.3 s = FORWARD, centre = NEUTRAL (spring-centred: let go = neutral), full back held ~0.3 s = REVERSE; X = steering. The AURIX keeps all gear safety (shift only at idle, F <-> R always through N with its dwell, refuse anything unsafe). Tests MV + storm.

## D. 13p Shore joystick buttons + Y/N answers (station + gateway; after C)
- Trigger = ACCEPT / confirm; thumb button = quick REJECT; button 4 SAVE; TWCS button CONFIRM (CREWED-related uses) unchanged; NO release button. Trigger counts only if the request was shown >= 0.5 s and on a fresh press. Mouse / Space / A still work; single-operator: trigger = own confirmation.
- One press never does two things: a pending request -> the trigger answers it; else steering in HDG LOCK -> new heading (C3); else nothing.
- Stick Y (free because the TWCS lever is the throttle) - one thing per mode: AUTO / HDG LOCK / STANDBY / DP: Y = YES / NO selection (forward = YES, back = NO, > 40 % deflection, fresh move from centre, YES pre-selected so the trigger alone = fast ACCEPT); MANEUVER: Y = gear (C5), requests via the hat (up = YES, down = NO) + trigger; REMOTE MANUAL: Y does nothing (gear on the TWCS lever as today). Hat up / down works as an alternative everywhere. If a station uses the stick Y as throttle (lever false): Y selection / gear disabled there, hat instead - shown in the Remote tab.
- Every Y/N request on screen 1 is drawn in the LOWER part of the screen (above the bottom bar), never in the centre; VERTICAL layout: question, [ΝΑΙ] on top, [ΟΧΙ] underneath (full-width framed), selected = inverted, countdown below. Same vertical Y/N layout everywhere in the program.
- Tests: Y in AUTO / HDG LOCK / STANDBY / DP / REMOTE moves no actuator and changes no mode; trigger priority; 0.5 s guard.

## E. 13n Display
### E1. 13n1 Look, layout, mode bar, BATTLE SHORT
- Colours: red = alarms (WARNING) only, amber = CAUTION; on the camera video (screen 1 HUD) all info GREEN; everywhere else info black or white by background; "connected" lamp white; NIGHT = dim red (as today); STANDBY = the only red button.
- A square frame around everything that can be pressed or typed into.
- Only 3 text sizes everywhere (very important / heading / info) + a text-size setting (small / normal / large, "Aa") in every window.
- Same layout principles for EVERY station and window (captain's and copilot's PCs, screen 1, screen 2, tools window, copilot camera view, academy windows): panels = slim vertical side strips or thin horizontal bars, collapsible AND draggable, centre (map / video) as free as possible. Layout + text size saved per PC, RESET TO DEFAULT. Copilot default: vertical strips left + right of the map.
- Screen 2 MODE BAR at the top centre, always visible, never hideable: [STANDBY] [HDG LOCK] [AUTO] [REMOTE] [MANEUVER] [DP] [MANUAL] · [BATTLE SHORT]. Active mode inverted (white fill, black bold, larger); not allowed now = greyed, a click says why ("no route", "captain only"); a click changes the mode (C2 rules); MANUAL shown but not pressable from the PC. Screen 1 keeps the mode in green.
- Screen 2 bottom: TWO thin full-width bars (bar 1: heading -10 -1 value +1 +10, AUTO pause / resume / skip, ROUTE, MAX SPEED, GEOFENCE, LINK TIME, ΕΠΑΝΑΦΟΡΑ ΙΣΧΥΟΣ when offered; bar 2: nearest ship + CPA, messages, zoom, BOAT, TOOLS, TRACKS / NEW TRACK, recording group [●SCR · ⏺5' · ⏺1h]); STANDBY also bottom-left (always drawn); sub-panels pop up above the bar only while used; bars hideable (STANDBY never) and movable.
- Screen 1 bottom: ONE thin full-width bar, grouped with small gaps: [ΕΛ/EN · NIGHT · DECLUTTER · FILL/FIT · Aa] [AR · OBSTACLES AI] [LAYOUT] [REC · ●SCR · SAVE (big enough) · ⏺5' · ⏺1h] [clock + mission time]; heading tape above it; SAVE + GENERAL WARNING / CAUTION lamps never hidden.
- Mirror resize exists (E / LAYOUT): restyle only (square white frames) + clearer tour text.
- AI labels (honest): a small framed white "AI" badge ONLY where a neural network is used: vision obstacle detection (eWaSR: button / status on screen 1 + 2, a small AI tag on obstacle boxes; tour: "AI only warns and may slow down - it never steers") and the Piper voice («Φωνή AI (Piper, εκτός σύνδεσης)»). NOT on collision avoidance / autopilot / shallow checks.
- BATTLE SHORT (marketing / demo theatre): grey bold framed button with black / yellow hazard stripes at the end of the mode bar. A click opens a dramatic window (dark, red border) that looks REAL - no "not installed" wording anywhere:
  «ΠΡΟΕΙΔΟΠΟΙΗΣΗ – BATTLE SHORT. Η λειτουργία BATTLE SHORT καταργεί ΟΛΑ τα όρια και τους ελέγχους ασφαλείας: όριο ισχύος και ταχύτητας, όριο στροφών κινητήρα (RPM), έλεγχο ρηχών νερών, αποφυγή σύγκρουσης AIS, ανίχνευση εμποδίων AI, γεωφράχτη και διαδικασία απώλειας σύνδεσης. Το σκάφος εκτελεί κάθε εντολή χωρίς καμία παρέμβαση. Ενεργοποίηση μόνο με εντολή του διοικητή. Ο χειριστής / η χειρίστρια και ο φορέας λειτουργίας αναλαμβάνουν ολόκληρη την ευθύνη. Η Trygons δεν φέρει καμία ευθύνη για ζημίες, τραυματισμούς ή απώλειες.» (+ English), masked field ΚΩΔΙΚΟΣ ΕΞΟΥΣΙΟΔΟΤΗΣΗΣ / AUTHORISATION CODE, [ΕΞΟΥΣΙΟΔΟΤΗΣΗ] on top, [ΑΚΥΡΩΣΗ] below. Any input -> «ΜΗ ΕΓΚΥΡΟΣ ΚΩΔΙΚΟΣ» / "INVALID AUTHORISATION CODE" (red flash + shake); Cancel closes. NO code exists anywhere (no stored hash, no check that can ever pass), NO function, NO path to the boat or gateway; a test proves any input is refused, nothing is sent, nothing changes, no lockout. Station log line "battle short attempt" only. Tooltip «BATTLE SHORT – απαιτείται κωδικός εξουσιοδότησης». Tour: "BATTLE SHORT - requires the commander's authorisation code" only.
### E2. 13n2 Boats, copilot camera, TOUR.bat
- Boat dropdown (boats/): pick -> connect -> automatic name check (welcome) -> HELM at once. NO live-camera requirement any more (a dead camera = warning "no video" only); connection / name failure = alarm. Change boat in one move (dropdown -> select); leaving a boat that is UNCREWED and moving -> one warning + OK (it starts its lost-link procedure). Rewrite CLAUDE.md section F ("confirm name AND live camera before HELM") as this owner decision.
- Copilot: a camera button on screen 2 -> any camera (FRONT-1/2, REAR-L/R) full screen on the copilot's own PC, Esc / "Map" back, green HUD basics on it; the copilot gets a lighter stream (lower res / fps), the captain's stream has priority (copilot's drops first: "reduced for the captain"), stream only while open.
- TOUR.bat next to START_DEMO.bat (academy/tour_app.py --lang el with the checked pythonw; no demo needed).
### E3. 13n3 Track
- Today app.py self.track = memory only, last 5000 points. New: live track on screen 2 (+ screen 1 mini map), no point limit (simplified only for drawing), written to disk continuously (crash-safe, restored at start); each point time, position, speed, heading; click on the line = time + speed there.
- One live track PER BOAT (switching boats shows that boat's own track; never mixed).
- NEW TRACK (ΝΕΟ ΙΧΝΟΣ, Yes/No check): the current track saved automatically as date_time_boat (e.g. 2026-10-31_0834_TRYGON-1), a new line starts.
- TRACKS (ΙΧΝΗ): list of saved tracks (date, boat, duration, nm); select one or more -> drawn embedded on the map (grey dashed, date label), any day / any boat; hide / show / remove any time (file stays). GPX export + import (Google Earth, OpenCPN, plotters).
- Live track white / black by background (not red).
### E4. 13n4 Screen recording + save buttons
- REC SCREEN: records BOTH screens (1 + 2) on this PC at lower resolution (~1280 wide, 5-10 fps, NVENC when available else x264 veryfast, ~0.3 GB/h), ring of the last 60 min (1-minute files), "●SCR" (white) while recording; stops with a message on low disk; never affects driving. The normal SAVE also keeps the screen part (same event id / time, pre + post like the cameras).
- Two framed buttons ⏺5' and ⏺1h on BOTH screens: keep permanently the last 5 / 60 min of BOTH screens (this PC) AND the boat's 4 cameras at FULL resolution (a gateway SAVE request with pre = 5 / 60 min, post = 0). One event id / time; screen part in a PC event folder, camera part on the boat SSD (video/events; not uploaded). Saved files leave the rings (never deleted automatically). Message «ΑΠΟΘΗΚΕΥΤΗΚΕ – ΟΘΟΝΕΣ + ΚΑΜΕΡΕΣ 5 ΛΕΠΤΑ / 1 ΩΡΑ»; boat ring shorter than asked -> save what exists and say so («ΚΑΜΕΡΕΣ: μόνο 20 λεπτά διαθέσιμα»). Gateway default ring_min -> 60 (~3.6 GB/h; check the SSD free-space policy). Not enough disk -> message; nothing already saved is lost.

## F. 13o Greek texts + voice
- "ταχύτητα" alone = SPEED only; gear = «σχέση» everywhere (tour, screens, MANUAL_EL, quizzes, lessons, voice), e.g. «Η σχέση (μπροστά / νεκρό / όπισθεν) αλλάζει μόνο στο ρελαντί» (the tour said «Η ταχύτητα αλλάζει μόνο στο ρελαντί» = wrong meaning).
- Tour voice sounds "sick" in the first ~2 s of every sentence; the owner checked: the Piper files themselves (groundstation\sound_cache\el\p_*.wav) have it. Fix: lower noise_scale / noise_w, a short lead-in silence, delete + remake all sentences (tour + callouts); option in the sound settings to use the Windows Greek voice (SAPI, e.g. Stefanos, when installed) instead of Piper; 5 sample sentences per option in a folder for the owner to listen and choose.
- Regenerate the tour voice after all the text changes of 13n / 13p / 13q.

## G. 13r Loudhailer + microphone on the USV (owner 2026-11-02) - gateway + station + PARTS / WIRING; NO AURIX change
Goal: the shore crew can talk with any contact near the USV (swimmers, fishing boats, coast guard, divers).
- Hardware (PARTS.md + WIRING.md): marine waterproof horn speaker / loudhailer 25-40 W IP67 + 12 V class-D amplifier; outdoor waterproof microphone with wind protection, mounted away from the engine (or a loudhailer with listen-back through the horn); USB sound card on the GATEWAY PC. Nothing on the AURIX, never in any control path.
- PUSH-TO-TALK, half duplex: while someone talks the boat mic is muted (no echo / howl). PTT = a framed SCREEN button «ΟΜΙΛΙΑ» (press and hold), in the bottom bar of BOTH screens and on the copilot's station - NOT on the joystick (owner choice). Captain AND copilot can speak, one at a time: the first wins (the gateway decides), the other sees «ΜΙΛΑ: ΚΥΒΕΡΝΗΤΗΣ / ΣΥΓΚΥΒΕΡΝΗΤΗΣ» and a greyed button; the current speaker is shown on every station.
- LISTEN: the boat mic plays continuously on the station's headset / speakers with mute + volume and a level meter (white, colour code of 13n1).
- Ready messages (one click, Greek + English, editable list), e.g. «Μη επανδρωμένο σκάφος – κρατηθείτε μακριά» / "Unmanned vessel - keep clear"; sound signals through the speaker (5 short blasts = danger, 1 long), clearly labelled.
- Link: low-bitrate voice (e.g. Opus 16-24 kbps), priority above the copilot's video, never above the captain's driving data / video; drops first on a weak link with a note; latency shown.
- Recording: both directions into the gateway ring recording and into SAVE / ⏺5' / ⏺1h events (evidence), on the camera timeline.
- Safety: audio is its own gateway process; failures = a notice only, never an alarm or a mode change; the demo / simulator has a simulated speaker / mic on this PC (academy + tests). Volume limit setting on the gateway.
- NO VHF radio (owner 2026-11-02).
- Tests: PTT one at a time, mic muted while talking, recording in SAVE, link priority, no effect on driving. MANUAL / MANUAL_EL / guided tour.

## H. UAV magazine (owner 2026-11-04) - steps U1-U6 - gateway + station + demo / sim + academy; NO AURIX change except the hatch relay through the existing relay table
Scope: camera / observation drones only - launch, flight, observation, video, telemetry, landing. NO payload carriage / release / target-strike functions anywhere (not in code, UI, docs or tests).
Owner's setup: 6 small 7" ArduPilot quads STACKED inside the USV under a hatch. Launch strictly in series, top drone (slot 1) first. They can NOT land on the USV (one-way).

H1 Registry / magazine
- Panel «ΜΑΓΓΕΜΑ UAV»: 6 slots (1 = top = next out); per drone: MAVLink sysid / serial, name, firmware, battery, health check (GPS, compass, sensors), VTX channel, ELRS binding, mission loaded. "Register" reads + checks over USB (USB-C data to each slot through a hub on the gateway). Unregistered / failing = never launched.
- PICTURES: wherever the user chooses a vehicle (the "Send route" target box, the map right-click menu, the magazine panel, the UAV list, map symbols) the UAV is shown with the OWNER'S UAV PICTURE and the USV with the boat picture. Owner file: C:\trygons\private\uav.png -> copy into assets/uav.png (+ small icon sizes like 13i's make_icon). If the file is missing: a simple placeholder + a WAITING FOR OWNER item.

H2 Routes and missions
- Map right click: «Πήγαινε εδώ» (BOAT: a quick 2-point route from the current position, checked like any route, then AUTO rules) and «Αποστολή UAV εδώ».
- "Send route" gets a target choice (vertical box, pictures): ΣΚΑΦΟΣ / ΕΠΟΜΕΝΟ UAV (θέση 1) / a specific UAV (one already flying = change its mission in flight).
- Route -> ArduPilot mission: TAKEOFF, waypoints with ALTITUDE per waypoint (default 50 m, editable per waypoint), speeds converted / capped, end = HOVER over the last point until a new route or the battery minimum.
- NO FORCED LANDING except a finished battery (owner): no GPS, lost link, warnings = hold / hover, never land; LAND only at the battery's absolute minimum (reserve = just enough to descend from the current altitude), where it is (over sea = drone lost, accepted).
- UAV route checks (not shallow water): battery / range with that landing reserve, distance from the boat (RF range), max altitude, air no-fly zones / air geofence, start = boat position at launch.
- UAV routes / tracks drawn in their own style + name, distinct from the boat's.

H3 Launch sequence + hatch
- Per drone: health check -> mission upload -> hatch OPEN (OPEN limit switch confirms) -> GPS ready (H4) -> ARM -> vertical climb -> "clear of the boat" confirmed -> next drone after a minimum gap (~15 s) at a different initial altitude; hatch closes after the last (CLOSED switch confirms).
- Conditions: boat slow / STANDBY; COPILOT prepares (registry, routes), CAPTAIN approves the launch (Y/N box + trigger); any warning stops the sequence (drones already in the air go on).
- Hatch: ONE relay on/off (ON = open) through the EXISTING relay table (type onoff, extras group; relays never affect driving - storm I36 keeps passing) + TWO limit switches OPEN / CLOSED read by the gateway (or an AURIX input reported in telemetry - choose the simpler, document it); no ARM unless "fully open"; hatch fault (no switch change in time / both / none) = sequence stops + notice.

H4 GPS start
- WARM START: while in the slot (USB), the gateway injects the boat's own u-blox data (time, position, ephemeris / AssistNow-style MGA data) into each drone's GPS via MAVLink GPS_INJECT_DATA -> hot start, fix within seconds after the hatch opens (needs u-blox GPS on the drones; note: magazine top not metal).
- FALLBACK "climb, then catch up": no good fix within a few seconds after opening (boat in STANDBY only) -> ARM in an altitude-hold mode, climb vertically to ~15 m, hover until GPS + yaw are good (EKF-GSF yaw in flight; compass near the hull not trusted; boat heading only as a hint), then start the mission. No landing because of missing GPS - it keeps hovering until GPS or battery minimum.

H5 Telemetry, video, AI
- Telemetry: ELRS with MAVLink through the BOAT: one ELRS TX module per drone on the boat (1:1, up to 6) -> gateway MAVLink router (one sysid per drone) -> shore over the existing link. Low bandwidth: telemetry + mission / guided commands; manual control = position / velocity setpoints, not raw RC.
- Video: ANALOG VTX -> SIX analog VRX on the boat (one fixed channel per slot, no two drones on one channel) -> USB capture into the gateway -> recorded in the gateway ring / SAVE / ⏺5' / ⏺1h and sent to shore as extra camera streams UAV-1..6 (lighter than the captain's main camera on a weak link; the station pulls only the streams open on a screen).
- AI: the same obstacle / object detection (eWaSR, "AI" badge) on the received UAV video; the boat's FRONT camera AI always keeps priority; UAV AI only on the feeds open on a screen / round-robin at low fps; measure with tools/gateway_load.py and write the numbers in gateway/README.md.
- Map: each drone's position, heading, altitude, track, battery, link quality; its picture as the symbol.

H6 Manual control + UAV HUD
- CAPTAIN: may fly a UAV with the shore joystick ONLY with the USV in STANDBY («ΧΕΙΡΙΣΜΟΣ UAV n», top banner «JOYSTICK → UAV n», the boat ignores the joystick). COPILOT: may fly a UAV from their own station any time (the captain keeps the boat). One pilot per drone at a time.
- Assisted flying only (LOITER / POSHOLD-style setpoints): stick = direction / speed, TWCS lever = climb / descend, twist = yaw, let go = hold position; speed / altitude limits. «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» = the UAV hovers, joystick back to the boat.
- The screen shows the UAV video with its own green HUD (altitude, speed, battery, distance + arrow to the boat, heading); the boat's alarms stay visible. Lost link = UAV holds position.

H7 Simulation, academy, docs, legal
- Demo / tests: 6 simulated drones (ArduPilot SITL copters if practical on Windows, else a small pymavlink drone simulator with the same MAVLink behaviour) + simulated hatch switches + simulated VRX test pictures; academy lesson "UAV launch and observation" (simulator only, like the other lessons).
- Flight log per drone (exportable). Air no-fly zones file. MANUAL / MANUAL_EL chapter + guided tour stops. Legal note in the manual: BVLOS in Greece / EU = Specific category + HCAA authorisation + licensed remote pilot; state / Navy rules differ; responsibility with the pilot.
- PARTS.md / WIRING.md: 6 ELRS TX modules (MAVLink-capable, ELRS 3.x), 6 analog 5.8 GHz VRX + USB capture, powered USB hubs, antenna placement, hatch actuator + 2 limit switches, USB-C data to each slot. Charging: OUT OF SCOPE (only the battery state is shown).
- UAV as RF relay for the boat's link: NOT in this task (later).

Steps (each its own commit + review; rules of PROGRESS.md LOOP RULES):
U1 design: CLAUDE.md section V "UAV magazine" (scope, rules above) + docs/uav_design.md + steps table. U2 registry + USB checks + GPS warm start + simulated drones + pictures. U3 route -> mission + launch sequence + hatch + switches. U4 map + telemetry + video streams + AI on UAV feeds. U5 manual control + UAV HUD. U6 academy lesson + MANUAL / MANUAL_EL / tour + PARTS / WIRING.
