## K. Presentation features (owner 2026-11-08) - steps PF1-PF6 - station + gateway + demo / sim + academy + docs; NO app / hal change (every boat movement goes through the EXISTING route / AUTO / UAV rules)
Inspired by what QGroundControl, UgCS, Mission Planner and Cockpit advertise (Claude's comparison 2026-11-08). Audience: Navy / coast guard / SAR evaluators. Scope rules of CLAUDE.md stay: camera drones only (observation / search / rescue wording - no "target" wording, no payload / strike functions), nothing raises the throttle by itself, the AURIX validates everything, red = alarms only.

K0 Ground rules (CLAUDE.md: a new section with the next free letter, "Search and rescue tools")
- Every tool here only CREATES routes / missions / markers / warnings. A boat route goes through the normal route checks (shallow water, geofence, 50-waypoint limit, max speed) and the AUTO start rules with the CAPTAIN's YES; a UAV mission through the UAV route checks (battery + landing reserve, RF range, altitude, no-fly) and the UAV launch / in-flight change rules. The copilot prepares, the captain approves (as today). Nothing starts by itself.
- Approach rule near a person in the water (MOB, a marked point, a SAR find): a boat route to such a point ENDS at a stand-off distance (default 30 m, TO CONFIRM) and the boat goes to STANDBY (idle + neutral) there; the final approach is manual by the captain. Text on screen + manual: propeller danger.
- Positions estimated by the C2 (marked from video, MOB drift) are shown as ESTIMATES with an uncertainty circle and «ΕΚΤΙΜΗΣΗ - όχι για ναυσιπλοΐα».
- All markers / patterns are shared to every station of this boat, logged (station log + flight / mission log) and kept in the SAVE / debrief files.
- Greek default (lang files, gender-inclusive), 13n1 look (framed buttons, 3 text sizes, slim draggable panels, vertical Y/N boxes answerable with the shore stick), MANUAL + MANUAL_EL, tour stops, screenshots docs/screens/sar/.

K1 Search patterns (IAMSAR Vol. II)
- Panel «ΜΟΤΙΒΑ ΕΡΕΥΝΑΣ»: Expanding square (SS), Sector search (VS), Parallel track (PS) (+ Creeping line if cheap). Inputs: datum (map click / MOB / marked point / typed position), track spacing S, first leg direction (default: into the wind / current when known, else the typed course), size (legs / radius / area), speed; for UAVs also altitude.
- Drawn on the map before sending (numbered legs, total length, time at the speed, area covered); target choice (vertical box with pictures, as "Send route"): ΣΚΑΦΟΣ / ΕΠΟΜΕΝΟ UAV / a specific UAV.
- Boat: converted into a normal route (if > 50 waypoints: refused with "split" or cut, say which) -> normal checks + AUTO rules. UAV: converted into a mission (altitude per waypoint) -> UAV checks; end = hover over the last point (V rules).
- Tests: geometry of every pattern against IAMSAR formulas (leg lengths, turns, spacing), the 50-waypoint rule, every route passes / fails the same checks as a hand-made route, nothing is sent without the captain's YES.

K2 MOB - man overboard / person in the water
- A large framed «MOB» button on screen 1 (bottom bar, recording group side) and screen 2 (bar 1) + key (default M, setting; not on the joystick). Captain AND copilot may press it (it changes no driving -> no ACCEPT needed).
- Press = at once: MOB marker at the boat's position of that moment (numbered MOB 1, 2 ...; time), WARNING alarm + voice callout «Άνθρωπος στη θάλασσα» (alarm rules: ack silences), an automatic SAVE event (cameras + screens, as ⏺5' with post), the MOB line on screen 1 + 2: distance + bearing from the boat, elapsed time; a drift ESTIMATE only when wind / current data exist (else none, say so).
- Offers (vertical Y/N boxes, captain answers): «ΕΠΙΣΤΡΟΦΗ ΣΤΟ MOB» = a 2-point route that ends at the stand-off distance (K0), approach from the down-wind side when wind is known; «UAV ΠΑΝΩ ΑΠΟ ΤΟ MOB» = next / chosen UAV hovers over the point (launch rules); «ΜΟΤΙΒΟ ΕΡΕΥΝΑΣ» = K1 with the MOB as datum. Marker editable (drag) and closable («ΒΡΕΘΗΚΕ» with time, kept in the log).
- CREWED: someone fell off the boat; UNCREWED: the operator saw a person in the water (same function, texts fit both).
- Tests: marker position = boat position at the press time (not later), alarm + SAVE + callout, offers need the captain, the return route ends at stand-off + STANDBY, works with a dead camera / no gateway (marker + alarm still on the station), single-operator mode.

K3 Mark a point from the video
- A click (or shore-stick hat / key while the cursor is on the video) on screen 1's camera video, the copilot's camera view or a UAV video = «ΣΗΜΑΝΣΗ ΣΗΜΕΙΟΥ»: the position from the camera calibration already used for the AR overlays (gateway vision / 13 AR) + the boat's (or UAV's) position, heading, attitude, camera height / UAV altitude and camera angle (UAV camera angle per drone in the registry, TO CONFIRM). Above the horizon / beyond a max range = refused with a reason.
- Marker «ΣΗΜΕΙΟ n» with an uncertainty circle that grows with range (computed from the calibration error + attitude error; the formula in the design doc), time, the source camera; a small still image of the clicked spot kept with it (debrief).
- Actions: route there (ends at stand-off when the marker is set as "person in the water"), send a UAV there, datum for K1, rename / delete.
- Accuracy measured in the simulator (known points), numbers in the manual; later in the 3D simulator.
- Tests: known-point geometry (boat camera + UAV camera), horizon refusal, the marker is shared to every station, nothing moves without the captain.

K4 ADS-B - manned aircraft near the UAVs (receive only, like AIS)
- Gateway reads a 1090 MHz ADS-B receiver (RTL-SDR dongle + readsb / dump1090, SBS port 30003 or Beast; own gateway process, pinned download to tools/bin) on the boat. Aircraft on the map: symbol by heading, callsign, altitude, speed, short track; filters (altitude below e.g. 3000 ft, within e.g. 20 km - settings).
- Warning while any UAV is in the air: predicted closest approach (horizontal + vertical, like the AIS CPA) inside limits -> CAUTION + voice («Αεροσκάφος κοντά στο UAV 3»), closer -> WARNING; a PROPOSAL (Y/N box) for that UAV's pilot / the captain: descend to a safe altitude / hold - NEVER automatic.
- No receiver / no data = notice «ADS-B: χωρίς δέκτη» (never shown as "clear sky"). Manual text: not every aircraft transmits ADS-B (military, gliders, some helicopters) - eyes and the authorisation's procedures stay first.
- Demo / sim: simulated aircraft (a helicopter crossing the UAV area) for the presentation scenario.
- PARTS.md: RTL-SDR (v4 or similar) + 1090 MHz antenna (+ filter / LNA), mounting on the boat; gateway load measured (tools/gateway_load.py).
- Tests: SBS / Beast parsing, CPA geometry, warning levels, proposal never acts by itself, no-data notice.

K5 Presentation scenario
- demo/: a scripted "SAR demo" (one button in the demo launcher): boat in AUTO, MOB pressed, return route offered, UAV launched over the MOB, an expanding-square search for a second person, a mark from the UAV video, a helicopter crossing (ADS-B warning, UAV holds) - every step needing the captain's YES waits for it (the presenter clicks / uses the stick). Academy: lesson + scenario "MOB and search" (simulator only).

Steps (each its own commit + independent review in parallel; PROGRESS.md LOOP RULES incl. the 2-fix-round cap; any allow.py change = SERIAL review):
PF1 design: CLAUDE.md section + docs/sar_design.md (geometry formulas, uncertainty formula, data flow, tests) + steps table + WAITING items (stand-off 30 m, ADS-B filters, UAV camera angles).
PF2 search patterns (K1). PF3 MOB (K2). PF4 mark from video (K3). PF5 ADS-B (K4). PF6 presentation scenario + academy + MANUAL / tour / PARTS + screenshots (K5).
