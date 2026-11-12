## M. Supervised autonomous harbour exit / entry with obstacle avoidance (owner 2026-11-12, after the Navy presentation) - steps HX1-HX7
Why: Navy feedback - a competitor's USV left the harbour for the open sea fully automatically and AVOIDED other boats. The C2 must do the same, under SUPERVISION (the captain watches and can take over at any moment). Radar and LiDAR can be added to the boats.

M0 Ground rules (CLAUDE.md: a new section with the next free letter, "Harbour autonomy"; the owner rule "the vision AI never steers" is REPLACED by M0-3 below, owner 2026-11-12)
1. Supervised only: the CAPTAIN (HELM) starts it with one YES; STANDBY, the e-stop, the joystick and any mode change ALWAYS win at once; the onboard joystick still takes full MANUAL at once.
2. Only inside a new mode / sub-mode "ΑΥΤΟΝΟΜΗ ΚΙΝΗΣΗ ΛΙΜΕΝΑ" (harbour autonomy), only on a checked harbour route (K1-style checks + a corridor), only at harbour speed (default max 5 kn, TO CONFIRM; lower near moored boats).
3. Steering for avoidance is allowed ONLY from FUSED tracks: an obstacle confirmed by RADAR or LIDAR (camera / AIS add confidence and classification). The CAMERA ALONE never steers (it may slow / stop, as today).
4. When unsure (sensor lost or degraded, conflicting tracks, no free path inside the corridor, planner timeout) = SLOW, then STOP and hold position (DP SPOT if fitted, else idle + neutral), alarm, wait for the captain. Never "guess".
5. The gateway only PROPOSES heading / speed (navreq lease, as today); the AURIX validates: max offset from the route corridor, max turn rate, harbour speed cap, lease timeout (no data = slow / stop). Nothing raises the throttle above the route's speed; avoidance only lowers speed or turns.
6. Lost link in harbour mode (the near-shore rule "< 50 m = kill at once" does not fit a harbour): STOP + hold, alarm; engine kill only after the grace time with no link (default LINK TIME + 60 s, TO CONFIRM) or on any fault that STANDBY would cut today. FIRMWARE - full SERIAL safety review.
7. Basic COLREGs in encounters with moving vessels: head-on = both alter to starboard; crossing from starboard = give way (slow / pass astern); overtaking = keep clear; when in doubt = slow / stop (M0-4). Shown on screen ("ΔΙΝΩ ΔΡΟΜΟ ΣΕ ΣΚΑΦΟΣ ΔΕΞΙΑ").
8. Berthing / unberthing (lines, the first metres from the quay) stays MANUAL or crew; autonomy starts in the harbour basin.

M1 Sensors (HX1 chooses, HX3 integrates)
- Marine radar with a documented open interface usable from Linux (gateway), short-range harbour mode; LiDAR (360 deg, marine-rated, e.g. a 360 solid-state unit) for 5-150 m; existing cameras + eWaSR; AIS. HX1 compares 2-3 options each (interface, Linux driver / SDK licence, range, price, power, mounting), the owner picks; PARTS.md / WIRING.md.
- Simulated radar targets and LiDAR scans in the PC simulator FIRST (all logic tested before hardware), later from the 3D simulator (Daskalio harbour).

M2 Perception (gateway): tracker that fuses radar + LiDAR + camera + AIS into tracks (position, velocity, size, class, confidence, sources); static map of moored boats / piers from the chart + LiDAR; latency + CPU measured (gateway_load.py).

M3 Local planner (gateway): inside the route corridor, velocity-obstacle / dynamic-window style; keeps a minimum passing distance (default 15 m moving / 8 m static, TO CONFIRM), prefers slowing over turning in narrow places, COLREGs M0-7; outputs navreq (heading offset + speed cap) every cycle; explains each decision in one line for the screen.

M4 Supervision UI: the corridor, every track (colour by source, confidence), the planned path, the reason for each manoeuvre, "AUTONOMY ACTIVE" status, big framed STOP and TAKE OVER; the captain's single YES to start; copilot sees all, captain decides.

M5 Teach and repeat: a recorded manual harbour exit / entry (the 13n3 track) becomes a checked harbour route with a corridor (default +-10 m, TO CONFIRM); routes per harbour (Daskalio first).

M6 Verification: a scenario library in the simulator (moored boats, crossing / head-on / overtaking traffic, a kayak and a swimmer without AIS, a buoy, sensor loss, link loss, GPS degradation) + a randomized "avoidance storm" with invariants: never closer than the minimum distance, never outside the corridor, always STOP when unsure, never above the speed cap, STANDBY always wins. Academy lesson + scenario. Bench / sea plan in FIRST_TEST.md (owner, step by step, empty harbour first).

Steps (each its own commit + review; 2-fix-round cap; reviews via api_review.py; FIRMWARE parts = FULL SERIAL review + part of the next flash gate):
HX1 design: CLAUDE.md section + docs/harbour_design.md (rules M0, architecture, sensor comparison for the owner, planner method, tests); WAITING items (speed, distances, corridor, lost-link grace, sensor choice). Opus.
HX2 FIRMWARE: harbour mode entry / exit (captain YES), harbour speed cap, corridor + turn-rate limits on navreq, lost-link M0-6; sim + storm. Opus, SERIAL review.
HX3 sensors: radar + LiDAR drivers behind an interface + SIMULATED radar / LiDAR in the sim + demo. Opus.
HX4 perception: fusion tracker + static map; tests with the simulated sensors. Opus.
HX5 planner + COLREGs + "unsure = stop"; navreq out; avoidance storm. Opus, SERIAL review (it steers the boat).
HX6 supervision UI + teach-and-repeat routes. Sonnet.
HX7 scenario library, academy lesson, MANUAL / tour, FIRST_TEST sea plan, presentation demo "harbour exit with traffic". Sonnet.
Later (3D project): the same scenarios in the Daskalio 3D world with rendered camera + simulated LiDAR / radar.
