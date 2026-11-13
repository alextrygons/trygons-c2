# Harbour autonomy - design (step HX1)

Owner spec: docs/owner_findings_2026-11-12.md section M (M0-M6). CLAUDE.md section Z is the short
source of truth; this file is what the implementing steps HX2-HX7 build from. Defaults marked
TO CONFIRM are PROGRESS.md W144-W149. Names (functions, frames, alarms, tests) are TARGETS: a step
may rename them inside its own commit and then updates this file and CLAUDE.md Z.

Contents: 0 ground rules - 1 architecture - 2 the AURIX side (HX2, FIRMWARE) - 3 lost link (M0-6) -
4 frames, messages, parameters, logs - 5 sensors interface (HX3) - 6 perception / fused tracks (HX4) -
7 the local planner + COLREGs + "unsure = stop" (HX5) - 8 supervision UI + teach and repeat (HX6) -
9 verification (M6) + tests per step - 10 failure analysis - 11 SENSOR COMPARISON (W148) -
12 risks + open questions - 13 one-page sensor summary for the owner.

## 0. Ground rules (M0) - how each one is kept

| M0 | Rule | Kept by |
|----|------|---------|
| 1 | Supervised: the captain (HELM) starts it with one YES; STANDBY, e-stop, joystick, any mode change win at once; the onboard joystick takes MANUAL at once | harbour autonomy is a SUB-STATE of MODE_AUTO (2.1): every existing AUTO exit (STANDBY button, e-stop, CREWED joystick takeover, any mode change, every fault) ends it unchanged. The "one YES" = the AURIX's CONFIRM (CREWED onboard joystick, UNCREWED the shore captain's joystick CONFIRM) - the only YES the AURIX can verify (R7) |
| 2 | Only in «ΑΥΤΟΝΟΜΗ ΚΙΝΗΣΗ ΛΙΜΕΝΑ», only on a checked harbour route + corridor, only at harbour speed (5 kn, lower near moored boats) | the AURIX: entry only with a program flagged HARBOUR that the gateway checked (via_gw, code 121 as for a DP TRIP), the speed cap harb_max_kn clamps the cruise target in firmware; the gateway: route check 114 / 120 + the new harbour check 129; the planner slows in narrow places |
| 3 | Avoidance STEERING only from FUSED tracks confirmed by RADAR or LIDAR; the camera alone never steers (may slow / stop) | gateway planner (7.2): a track may cause a TURN only with track.range_ok (radar or LiDAR hits, 6.2); camera-only and AIS-only tracks may only slow / stop. Storm I87 checks it. REPLACES the old vision rule "the vision AI never steers" INSIDE harbour autonomy only |
| 4 | Unsure = SLOW, then STOP and hold, alarm, wait for the captain | planner unsure list (7.4) -> navreq STOP; the AURIX's own backstops (lease lapse, corridor exceeded, GPS / compass) -> HOLD (2.5); only the captain's RESUME (+ CONFIRM) leaves HOLD |
| 5 | The gateway only PROPOSES; the AURIX validates corridor offset, turn rate, harbour cap, lease; nothing raises the throttle above the route's speed | navreq kind HARBOUR (4.1); the AURIX checks (2.3); a cap only lowers; the throttle path = the normal AUTO cruise + servo.c clamp |
| 6 | Lost link in harbour: STOP + hold, alarm; kill only after LINK TIME + 60 s or on any STANDBY-cutting fault | FIRMWARE change to section B (3); full SERIAL safety review in HX2 |
| 7 | Basic COLREGs, shown on screen | planner encounter classes (7.3); the explanation line (7.5) |
| 8 | Berthing / unberthing stays manual / crew; autonomy starts in the basin | the harbour route begins at a basin waypoint (teach and repeat cuts the first / last metres, 8.2); entry needs the boat inside the first leg's corridor and slow (2.2) |

Kept unchanged: FIRST TEST (harbour refused there), BENCH, DP (separate mode), the big boat (no
harbour mode), CREWED / UNCREWED profiles, power limit + raise rules, the throttle asymmetry,
gear-at-idle, the CPU1 monitor (no new monitor rule needed: harbour only ever LOWERS the AUTO
throttle), relay rules, UAV rules.

## 1. Architecture

```
 radar ---\                                    gateway PC                              AURIX (core 0)
 LiDAR ----+-> sensor workers (own processes) -> perception (tracker + static map)  ->
 cameras --/   gateway/harbour/radar_worker.py   gateway/harbour/tracker.py
 AIS (ais.py)  gateway/harbour/lidar_worker.py   gateway/harbour/staticmap.py
               (or the simulators simradar.py /                 |
                simlidar.py)                                    v
                                                  planner (harbour/planner.py + colregs.py)
                                                  5 Hz: heading offset + speed cap + why
                                                                |
                                     Gateway.nav_compose -> GW_NAVREQ kind HARBOUR (lease 1 s) --> validate:
                                                                                                corridor, turn rate,
                                                                                                harbour cap, lease
                                                                                                -> AUTO steering /
                                                                                                   cruise or HOLD
 stations <- gw_status "harbour" + harbour_view (tracks, plan, why) <- gateway       <- HARBOUR_STAT 0x90 (AURIX)
```

- The gateway does all perception and planning; the AURIX never sees a track. The AURIX's job is the
  same as for collision avoidance (section D): take or refuse a proposal, and fall back to a safe
  state on its own clock.
- Everything new on the gateway lives in the package gateway/harbour/ (+ gateway.py wiring). No
  module of it imports aurix.py or sends to the AURIX: the ONE output is the navreq that
  Gateway.nav_compose already sends (a writer check like servo.c: only nav_compose builds
  GW_NAVREQ; HX5 test HA-writer).
- Sensor workers are OWN processes (like the vision / UAV workers): a crash = a sensor lost = the
  planner's unsure rule (7.4), restart; the AURIX lease covers a frozen gateway.
- Rates: radar ~0.4-1 Hz rotation (a sweep; radar-dependent) or per-spoke; LiDAR 10-20 Hz scans;
  tracker 10 Hz; planner + navreq 5 Hz; HARBOUR_STAT from the AURIX 5 Hz.

## 2. The AURIX side (HX2 = FIRMWARE; full SERIAL safety review; part of the next flash gate)

Goal: as small as possible. NO new mode number: harbour autonomy = MODE_AUTO with sb_t.harb.on
(sub-state). Reason: AUTO already has the program, the leg guidance, the navreq lease, the cruise
speed loop, the CONFIRM start, the joystick / STANDBY / fault exits, the 13q3 shore override and the
UNCREWED protections (13q3b sovr_stop_why); a new mode would copy all of them and need a new row in
the transition table, CPU1 publishing, the storm mode tables and every screen. The screens name it
«ΑΥΤΟΝΟΜΗ ΚΙΝΗΣΗ ΛΙΜΕΝΑ» / "HARBOUR AUTONOMY" from HARBOUR_STAT.

### 2.1 State (app/harbour.c + harbour.h, pure C, sb_t.harb)

harb.state: HARB_OFF / HARB_RUN (following the route, planner proposals applied) / HARB_SLOW (a
degraded cap in force) / HARB_HOLD (stopped: idle + neutral, heading held while moving, then the
steering holds; DP SPOT controller when fitted = HX2b) / HARB_DONE (arrived: PROG_END_STOP as today).
harb.why (the last HOLD / refusal reason, 4.3). harb.corr_m, harb.max_kn (from the HARBOUR command,
clamped by the parameters). harb.ll (lost link began in harbour, 3).

### 2.2 Entry (captain's YES) - PROTO_T_HARBOUR (station command, HELM only) + CONF_HARBOUR

- Station command PROTO_T_HARBOUR {u8 action 0 START / 1 RESUME / 2 HOLD, u8 corridor in 0.1 m units
  (0..255 = 0..25.5 m), u8 max_kn x10} through GW_CMD (proposal 0x3C: free - link.c dispatches the
  gateway-only frames by their exact type, no range check). The AURIX CLAMPS both station values:
  harb.corr_m = clamp(frame, 4 m, harb_corr_m) (a value below 4 m = NACK, round-2 LOW-3), harb.max_kn = min(frame, harb_max_kn, HARB_MAX_KN_CEIL)
  (review LOW-6; tested in HB3). A copilot's START / RESUME = the captain's ACCEPT (proto.DRIVING_CMDS);
  HOLD is free for every crew station (like AUTO PAUSE).
- START / RESUME -> a pending CONF_HARBOUR (new confirm kind) answered within confirm_ms 10 s by the
  CONFIRM (CREWED onboard joystick; UNCREWED the shore captain's joystick, GW_RJOY b2), same rules as
  every CONFIRM (stick centred, armed, only for a pending request; refused while another is pending).
- Conditions checked at the request AND at the CONFIRM (any missing = NACK with the reason, nothing
  changes): HARBOUR START and RESUME are added BY NAME to the "held stick" refusal list (joy.off_centre,
  also while ignored after STANDBY: PC NACK 8 "centre the joystick", chirp - review LOW-1); not FIRST TEST, not BENCH, not the big boat; mode STANDBY (not stopping) / HDG LOCK /
  AUTO / MANUAL with the stick centred / UNCREWED REMOTE or MANEUVER (the AUTO START set + the 13q2
  rules); the stored program is flagged HARBOUR (prog flags b0, set by the gateway's harbour route
  upload) AND loaded over the gateway link + checked by it (autop.via_gw, the DP TRIP rule 121);
  GPS trusted with unc <= harb_unc_m (5 m TO CONFIRM; no dead reckoning in harbour); compass OK;
  outputs armed; a FRESH navreq of kind HARBOUR (never received / lapsed = NACK: the planner must be
  running) without its STOP flag; the boat inside the first (or the active, for RESUME) leg's corridor
  (|xte| <= corr_m) and SOG <= harb_max_kn + 0.5 kn; no CAN node / bus fault; no engine kill.
- At the CONFIRM: MODE_AUTO (if not already) at waypoint 1 (START) or the active one (RESUME),
  harb.state RUN, the AUTO rules apply plus 2.3-2.5. EV_HARBOUR what 1 start / 2 resume.

### 2.3 Validation of each harbour proposal (gw_navreq kind SB_NAV_HARBOUR = 4)

Every step while harb.state is RUN / SLOW:
1. Lease: a navreq of kind HARBOUR fresher than the EFFECTIVE lease = min(the frame's lease,
   harb_lease_ms 1000) - the gateway can only shorten it, never lengthen it (review MED-2; frame
   limits 500..3000 ms for this kind). Lapsed / another kind / never = HOLD (why LEASE). NOT
   "carry on at nav_lost_kn" (that stays AUTO's rule outside harbour).
2. STOP flag (hflags b0) or cap < SB_SHDG_STOP_KN = HOLD (why PLANNER) - the planner's "unsure".
3. Speed: target = min(leg speed, harb.max_kn, harb_max_kn param, the navreq cap, the slamming cap).
   Only ever lowered; harb_max_kn has a firmware ceiling HARB_MAX_KN_CEIL 8 kn.
4. Heading: accepted only if |heading - leg track| <= harb_max_dev_deg (60 deg TO CONFIRM) AND
   (|xte| <= corr_m - harb_corr_margin_m (2 m) OR the heading does not increase |xte|). Not accepted
   = the AURIX steers its own leg guidance back to the line, the cap still slows, the count
   harb.refused++ (no alarm for one; 2 s of refusals in a row = HOLD why HEADING).
5. Corridor: |xte| > corr_m = the AURIX's own guidance back to the line at min(cap, harb_back_kn 2 kn);
   |xte| > corr_m + harb_corr_stop_m (5 m) = HOLD (why CORRIDOR). xte from the AUTO leg (autop.leg).
6. Turn rate: the heading target given to autopilot_set_target moves at most harb_rot_dps (8 deg/s
   TO CONFIRM) - a slew limit on the target, so a jumping proposal never makes a step change.
7. Integrity: GPS untrusted / suspect / unc > harb_unc_m = HOLD (why GPS: no DR in a harbour);
   compass lost = HOLD (the AUTO compass-lost rule: gyro hold until idle + neutral); the 13q3b
   sovr_stop_why STOPs in UNCREWED AUTO apply unchanged.

### 2.4 Exits

- Any mode change (STANDBY button / command, joystick takeover, REMOTE, MANEUVER, DP, HDG LOCK,
  every fault STANDBY) = harbour OFF in that step (+ the existing heading hold rules of the new mode).
- AUTO PAUSE or PROTO_T_HARBOUR HOLD = HOLD (why USER).
- Arrival at the last waypoint = the program's final action (a harbour route ends with STOP or
  HOLD: the teach tool only offers those; never "continue at speed").
- A new program upload / AUTO START of a non-harbour program = harbour OFF.
- The UNCREWED shore stick override (13q3) works as in AUTO: hand steering inside the mode, the
  planner's heading ignored while held, the caps + HOLD still apply; letting go = back to the line.

### 2.5 HOLD (M0-4: "stop and hold position")

- HX2a (required): throttle idle. Which rate (review MED-1, explicit): ONLY why USER (the station's
  HOLD / AUTO PAUSE) and the planned arrival use the normal rate_down; EVERY other HOLD reason
  (LEASE, PLANNER = the gateway's STOP incl. U5 / U7 an object too close, CORRIDOR, HEADING, GPS,
  COMPASS, LINK, FAULT) = emergency() = idle AT ONCE with no rate limit (sb emerg; storm I47 counts
  these harbour HOLD reasons among its emergency conditions). The emergency is judged EVERY STEP from
  the conditions in force, never from the stored first why: a fault arriving during a USER HOLD's
  ramp makes it idle at once (round-2 LOW-1; I47 covers that case). Then NEUTRAL by the gear state machine, heading held (gyro if no compass) until idle +
  neutral, then the steering holds (OD-10). Alarm SB_ALM2_HARBOUR_STOP (0x100, an ordinary alarm,
  not an advisory: buzzer + SAVE). The mode stays AUTO (harb HOLD) so the captain sees why and can
  RESUME; the boat drifts at idle + neutral.
- HX2b (optional, only with both thrusters fitted, W149): after idle + neutral + SOG < dp_max_entry_kn
  the dp.c SPOT controller holds the spot inside harbour HOLD (dp_active published to CPU1 as in DP,
  every DP rule: heat budget, off-spot limits, GPS lost = thrusters 0). Not after a lost link (3).
- Leaving HOLD: RESUME + CONFIRM (2.2), or any other mode. Never by itself.

### 2.6 What the AURIX publishes

HARBOUR_STAT frame (4.2), log event EV_HARBOUR (4.4), one log field "harb" (u8 state | why << 4;
the black-box slot has 2 bytes left), monitor: nothing new (harbour only lowers the AUTO throttle).

### 2.7 As built (HX2)
- Every 2.2-2.6 item is in app/harbour.c (pure validation) + smallboat.c (entry, HOLD, exits, lost link) + link.c (frames,
  parameters); CLAUDE.md Z "Step HX2 AS BUILT" lists the names. Choices made in HX2 (within the design):
  - START begins on the route leg waypoint 1 -> 2 (the boat must be inside ITS corridor, along -corr .. length + corr);
    RESUME on the active leg k-1 -> k - the corridor is always the CHECKED line, never a leg from where the boat is.
    A harbour route needs >= 2 waypoints.
  - The HARBOUR STOP alarm is raised for every HOLD reason except USER and the planned ARRIVED (a STOP waypoint / the
    final HOLD; why 15, added): those close at rate_down, without the alarm; every other reason = emergency() each
    step (also a fault arriving during a USER / ARRIVED ramp).
  - A kind-4 request is stored but INERT outside harbour autonomy: it never counts as AUTO's / UNCREWED HDG LOCK's /
    the shore override's lease (nav_fresh), never caps / steers (counted in harb.k4_idle).
  - AUTO PAUSE in harbour = HOLD USER; AUTO RESUME / SKIP / RETURN are refused while it runs (only HARBOUR RESUME +
    CONFIRM leaves a HOLD). HX2 review MED-1: a route flagged HARBOUR is NEVER driven by plain AUTO START / RESUME /
    RETURN or a DP TRIP (WRONG_MODE) - only harbour autonomy with its lease / caps / corridor; HX5's HA-fold test
    therefore uses a plain (unflagged) route for "AUTO without harbour".
  - The speed's leg input is min(cruise target, the leg's set speed). Storm I86 found that AUTO's rough-water
    slow-down RAISED a target below nerv_min_kn (5 kn) to 5 kn; fixed for every AUTO (a slow-down never raises).
  - Found in HB4 (sim): at 4 kn the boat turns slowly (~2 deg/s), so a proposal steered to the corridor edge can
    overshoot the edge rule by several metres before the own guidance brings it back - the CORRIDOR HOLD (+5 m) is the
    backstop that then acts. The planner (HX5) must not lead the boat to the edge; W146 may want a wider margin.
  - The drift cut (SOG > 1 kn for 10 s at idle + neutral): a boat coasting down from 5 kn in neutral may stay above
    1 kn for ~10 s (the sim's 6 s speed constant) - with the shipped grace 0 nothing changes; W147 should look at it.
  - A cap below the in-gear idle speed (sim: 1.5 kn) can only bring the throttle to idle - the boat still makes its
    idle speed in forward; only the STOP flag / a cap < 0.05 kn (HOLD) stops it. HX5's planner sends STOP for that.
  - Review notes (serial opus review of 417a0d8, not fixed): MED-3 the HOLD needs HELM like every AURIX command (R7; the
    gateway forwards a copilot's HOLD without an ACCEPT); LOW-1 harb_ll_grace_s is limited only by PARAM_SET's 0..0 range
    (sb_default_config = 0, no other writer; the sim / storm set it directly to test the mechanism) - the W147 commit
    raises the range and adds the STANDBY + CONFIRM path; LOW-3 storm oracle: I47's harbour clause uses the AURIX's why,
    I86 checks harb.cap_kn + the true SOG, I85 assumes the 3 s near-shore detection (open sea / fuzz excused).
  - Round-2 review of b70fac4 (the last round): no HIGH; its MEDs were checked against the code - harb_entry has its own
    checks (not auto_common_checks; HB1-HB13 pass), sb_program_set refuses a new program while AUTO / a DP TRIP runs, no
    automatic path starts AUTO RETURN / a DP TRIP, steer_pause's flag is AUTO's own (auto_mode releases it every RUN step);
    its LOW fixed: the harbour-route refusal comes after the held-stick NACK 8.

## 3. Lost link in harbour (M0-6) - FIRMWARE, SERIAL review

### 3.1 Today (section B)
UNCREWED: link lost -> STANDBY (idle + neutral first); near shore (< near_shore_m 50 m or no fresh
shore information) -> ENGINE KILL as soon as idle + neutral; open sea -> after lostlink_grace_s 10 s.
LINK TIME near shore is always 3 s. In a harbour the boat is ALWAYS "near shore", so today a 3 s
link drop kills the engine of a boat in the harbour basin - with no steerage left in traffic.

### 3.2 The change (W147 default) - SHIPS DISABLED until the owner answers W147
Review MED-4: delaying a kill is a change on the unsafe side, and "TO CONFIRM = never guessed in
code". HX2 builds the mechanism with harb_ll_grace_s DEFAULT 0 = today's section B behaviour
(near-shore kill as soon as idle + neutral); the 60 s value is set only after the owner's YES to
W147 (a separate one-line commit, part of a flash gate). Everything below describes the mechanism
with a non-zero grace.
- A loss that begins while harbour autonomy runs (harb.state != OFF at the moment of the loss;
  latched in ll.harbour) = STANDBY idle + neutral first as today (the hold = idle + neutral, steering
  holds), alarm LINK LOST, BUT the kill waits: ENGINE KILL only once idle + neutral AND
  (the time since the HELM station was last heard >= the detection time in force (3 s near shore) +
  harb_ll_grace_s (60 s TO CONFIRM)). I.e. ~63 s in a harbour.
- During that grace the kill is IMMEDIATE (as soon as idle + neutral, or at once where the existing
  rule says so) on ANY of: e-stop, monitor trip, CAN node / bus fault (OD-3: throttle node lost =
  kill at once, unchanged), compass + gyro both lost, GPS lost / suspect, gateway lost (GATEWAY LOST),
  an engine / gear fault, geofence breach, the STANDBY button pressed on board (someone is there:
  UNCREWED would not apply, but a press is a STANDBY-cutting event). "Any fault that STANDBY would cut
  today" = the WHY_ list of HX2 table harb_ll_cut[] (one place, tested by HB-LL).
- The latch ll.harbour (review MED-3): set on the LOSS EDGE from the harb.state of the START of the
  step (lostlink_step reads a copy taken before any mode handling of that step, so the STANDBY it
  forces cannot clear it first); CLEARED when the link returns, at the kill, and at every reset - a
  later loss in plain STANDBY gets the normal near-shore rule. HB11 tests both (loss in the same step
  as a mode change; loss -> return -> second loss in STANDBY = the near-shore kill).
- Link back within the grace: stays STANDBY at idle + neutral, alarm at the station, never resumes
  by itself (unchanged rule) - the captain decides.
- CREWED: unchanged (never a kill; REMOTE LINK LOST alarm). FIRST TEST: harbour refused anyway.
- The max engine time without a link becomes 3 + 60 s in a harbour (was 3 s), shown on screen 2's
  link panel countdown.
- CPU1 / the OA-3 dead-man: the lost-link kill timing lives ONLY in core 0's lostlink_step; CPU1's
  dead-man pulse conditions (UNCREWED + no trip + core-0 heartbeat + throttle node OK + actuator
  frames + no kill latch) do not include the link, so they are unchanged. HX2's SERIAL review checks
  this against the code.
- Gear during the grace: "neutral" is the existing confirmation (the neutral switch when fitted, else
  the gear state machine + the gear frame sent: servo.gear_sent, 13q2d); a DroneCAN gear servo has no
  position feedback, so a gear stuck in forward at idle is seen only by the neutral switch or a SOG
  rise - HX2 adds to harb_ll_cut "trusted GPS SOG > harb_ll_drift_kn (1 kn TO CONFIRM) for 10 s at
  idle + neutral during the grace" = kill at once - NO "rising" condition (a gear stuck in forward
  settles at a steady 2-3 kn; round-2 MED-3; HB11 has a steady stuck-forward case; W147 asks).
- Not changed: the open-sea rule (10 s), the LINK TIME override (13q1) outside harbour, the latch
  (power-on reset / remote restart), I31 / I38.

### 3.3 Rules / invariants that change (named exactly)
- CLAUDE.md section B "near shore ... -> ENGINE KILL as soon as idle + neutral": gets ONE exception
  "a loss that began in harbour autonomy: after detection + harb_ll_grace_s, earlier only on a
  harb_ll_cut fault". Section B "Max engine time without a link = LINK TIME + lostlink_grace_s"
  gets the harbour value.
- Storm I27 / I28 (near-shore kill timing) get the same exception; new I85 (below) checks the
  harbour grace both ways (never before the grace without a cut fault, always by grace + 1 step).
- Nothing else in B / T changes (OD-3 exception stays, the kill latch stays).

### 3.4 Why not DP hold after a lost link
An automatic DP entry with nobody watching moves thrusters without supervision; idle + neutral +
the kill after 63 s is the conservative choice. W149 asks the owner.

## 4. Frames, messages, parameters, logs

### 4.1 GW_NAVREQ kind 4 SB_NAV_HARBOUR (gateway -> AURIX, 5 Hz while a harbour route is armed)
Same 10-byte head (seq, kind, reason, alarm, u16 heading x100, u16 cap x10, u16 lease) + appended:
u8 hflags (b0 STOP, b1 degraded = a sensor lost, b2 give-way (COLREGs), b3 static-only (no moving
traffic), b4 camera-only obstacle slowing), u8 nearest_m (0..250, 255 none), u8 n_tracks.
gw_navreq accepts kind 4 only with len >= 13; lease 500..3000 for kind 4. A harbour request never
needs a cap = 0xFFFF check change: kind 4 always carries a cap (the route speed when nothing slows).
Outside harbour autonomy a kind-4 request = ignored (counted), never applied. ONE navreq slot: Gateway.nav_compose FOLDS every other cap into the kind-4 frame (the shallow
cap, the collision cap, a held vision cap - still never released silently: CLEAR SLOWDOWN) ONLY
while the AURIX's OWN fresh HARBOUR_STAT reports RUN / SLOW / HOLD (never on gateway-side state: a
route "armed" on the gateway while the boat runs plain AUTO gets kind 1 / 2 as today - round-2
MED-1; test HA-fold: a harbour route stored, AUTO started without HARBOUR, a held vision cap is
still applied); the folded caps keep their reasons: the reason byte = the lowest cap's reason and
hflags b5 shallow / b6 collision / b7 vision (+ the alarm byte as today: 8 OBSTACLE stays raised on
the AURIX - round-2 LOW-4); a kind 1 / 2 frame arriving in harbour = "another kind" = HOLD
(review LOW-3).

### 4.2 PROTO_T_HARBOUR_STAT (AURIX -> stations, NEW frame, 5 Hz; TELEM2 is FULL)
Proposal 0x90 (the next free telemetry number): u8 state | u8 why | i16 xte dm | u16 corr dm |
u16 cap applied kn x10 | u16 heading target x100 | u16 proposal age ms | u8 refused (count mod 256) |
u8 flags (b0 harbour route stored + gateway-checked, b1 entry possible now, b2 lease fresh, b3 HOLD
pending idle + neutral, b4 lost-link harbour grace running) | u16 ll grace left s x10. Both ports
(USB + gateway) when room (the ACCEL rule: after TELEMETRY, GW port only with gw_room + reserve).
groundstation/proto.py decode_harbour_stat.

### 4.3 why codes (HOLD / NACK)
1 USER, 2 LEASE, 3 PLANNER, 4 CORRIDOR, 5 HEADING, 6 GPS, 7 COMPASS, 8 LINK, 9 FAULT, 10 NOT_HARBOUR
route, 11 NOT_CHECKED (121), 12 OUT_OF_CORRIDOR at entry, 13 TOO_FAST at entry, 14 NO_LEASE at entry.

### 4.4 Logs / alarms / results
- Alarm SB_ALM2_HARBOUR_STOP 0x100 (an ordinary alarm). Notice (station) HARBOUR DEGRADED.
- EV_HARBOUR = 32 (what 1 start, 2 resume, 3 hold + why, 4 off + mode, 5 arrived, 6 lost-link grace
  started, 7 kill after grace, 8 heading refused burst); groundstation/tlg.py.
- Gateway result 129 "harbour route refused" (corridor not clear of the static map / shallow, no
  harbour data, > 50 waypoints, leg speed > harb_max_kn) beside 114 / 120 / 121; audit kind "harbour".

### 4.5 Parameters (PARAM_SET, STANDBY only, limits in link.c)
harb_max_kn 5 (1..8, W144), harb_corr_m 10 (4..20, W146; the route's own value <= it),
harb_corr_margin_m 2, harb_corr_stop_m 5 (2..10), harb_max_dev_deg 60 (20..90),
harb_rot_dps 8 (2..15), harb_lease_ms 1000 (500..3000), harb_unc_m 5 (2..10),
harb_back_kn 2, harb_ll_grace_s: a compile-time range 0..0 until W147 is answered (PARAM_SET can't
set any grace - round-2 MED-2); afterwards the ceiling = the owner-signed value and a change needs
STANDBY + CONFIRM + a log entry, like thr_trav_s / CONF_THR_TRAVEL (OD-4). Gateway (gateway.json "harbour"): min_moving_m 15,
min_static_m 8 (W145), horizon_s 40, cycle_ms 200, sensor stale limits, degraded_kn 2, narrow_kn 3.

### 4.6 Station <-> gateway JSON (HX5 / HX6)
harbour_route {op "check" | "store", name, harbour, wps [{lat, lon, kn}], corr_m} -> harbour_ack
{ok, code 0 / 114 / 120 / 129, why}; harbour_teach {op "from_track", track id, cut_start_m, cut_end_m};
harbour_view_on {} -> harbour_view 5 Hz {tracks [...], plan {heading, kn, path}, why, colregs,
sensors {radar, lidar, cameras, ais}}; gw_status "harbour" {state, armed, why, sensors}.

## 5. Sensors interface (HX3)

gateway/harbour/sensors.py: one interface for every source, so the tracker never knows the brand:
- RadarSource.read() -> RadarSweep {t, own pose at t, spokes or a polar image (range bins x angle),
  range_m per bin, gain / sea / rain settings} or a list of radar plots {t, range, bearing, size,
  strength} (a radar's own target tracking if it gives one = MARPA / ARPA plots, used as plots).
- LidarSource.read() -> LidarScan {t, pose, points N x 3 (sensor frame), intensity}; the worker
  removes the boat's own hull box, the water plane band (+-0.3 m around the expected water height,
  from the IMU attitude) and spray (single isolated returns < 3 m), clusters (Euclidean, 0.5 m) into
  objects {centroid, extent, n_points}.
- Each worker: OWN process, below normal, JSON lines (as the UAV router): status {ok, rate, last_t,
  why}, objects batches; a time sync check (sensor time vs gateway clock: > 100 ms skew = degraded).
- Simulators (HX3, first): gateway/harbour/simradar.py + simlidar.py produce the same objects from a
  scripted harbour world (gateway/harbour/simworld.py: piers, moored boats, moving vessels with AIS or
  not, a kayak, a swimmer, a buoy) driven by the simulated boat's telemetry, with noise, drop-outs,
  clutter, specular misses on calm water, minimum-range blind zones. The demo runs them.
- Real drivers (HX3, after W148): radar via Mayara / Signal K Radar API (HTTP / WebSocket spokes) or
  the vendor SDK; LiDAR via the vendor SDK (ouster-sdk / Livox-SDK2 Python bindings or a small C
  helper). Never on the AURIX.

### 5a. As built (HX3) - sensors behind an interface + the SIMULATED radar / LiDAR

Package gateway/harbour/ (no app / hal / AURIX change; nothing in it imports aurix.py / stations / proto or a network
module - HS9 scans it and runs a worker with sockets trapped):
- sensors.py = the interface: the frames pose / RadarPlot {range_m, brg_deg RELATIVE TO THE BOW, speed_mps, course_deg,
  size_m, size_cls small / medium / large, strength} / LidarObject {x ahead, y STARBOARD, z up from the sensor, len_m,
  wid_m, hgt_m, n_points} / batch {ev, seq, t sensor time, t_tx, pose, items, bad} / status {self ok | degraded |
  offline, why, rate_hz}; validation (NaN / out of range / bad class = refused + counted in "bad", never used);
  lidar_filter (own hull box, the water band +-0.3 m, isolated spray < 3 m); HealthJudge = the GATEWAY's judgement
  from what it receives: OFF / OFFLINE / STARTING / OK / DEGRADED (the worker says so, rate < half nominal, time skew
  |(t_tx - t) - latency_s| > 0.1 s, > 20 % invalid items) / LOST (no batch for lost_s: radar 4 s, LiDAR 1 s; worker
  gone), back to OK only after 2 s good; check_config (osdep.check_config / gateway.py --check-config carries it);
  RealDriverSkeleton = the place of the real drivers "mayara" (radar) / "livox" / "ouster" (LiDAR): UNTESTED, NOT
  CHOSEN (W148) - they report OFFLINE with the reason and produce nothing.
- worker.py + radar_worker.py / lidar_worker.py = OWN processes (below normal, 1 work thread, OMP / MKL threads 1),
  JSON lines (stdin pose 5 Hz / fault (simulator only) / stop; stdout status 1 Hz + a batch per sweep / scan); ends when
  stdin closes; a pose older than 2 s = no batch + self degraded "no own pose".
- simworld.py = the scripted harbour (ENU metres from an origin; a breakwater + a quay, 3 moored boats, a ferry WITH AIS
  (ping-pong path), a fishing boat, a kayak and a swimmer WITHOUT AIS, a buoy; positions a function of time only =
  one truth for radar, LiDAR, tests and HX4); a world file (JSON) with origin null = placed at the own boat's FIRST
  fresh position by the manager (both workers get the same world on their command line).
- simradar.py: 0.8 Hz sweeps; Pd = rcs + 0.3 fading with range (small objects first), range sd 3 m + 0.5 %, bearing sd
  1 deg, ARPA-like speed / course sd 0.3 m/s / 5 deg, a quay = a return every 15 m, the MINIMUM-RANGE blind zone 25 m
  (HALO value NV: bench it), max 3000 m, a dropped sweep 2 %, sea clutter ~1 weak plot per sweep within 250 m.
- simlidar.py: 10 Hz; range by reflectivity (40 m at 10 %, 70 m at 80 %), mounted 2.5 m above the water with a
  -7..+52 deg vertical FOV = a LOW object close by is under the field of view (a swimmer closer than ~18 m is not
  seen), points ~ visible area / range^2, a calm-water swimmer missed in ~40 % of the scans, the raw scan holds the own
  hull, water returns and spray (removed by lidar_filter), drop 1 %.
- Faults (both simulators, the manager's fault(kind, name): refused for a real driver): lost, degraded (radar noise x3 +
  Pd x0.5; LiDAR range x0.5, Pd x0.7), clutter (radar x25; LiDAR rain / fog 20 spurious clusters within 15 m - they PASS
  the filter: the tracker's problem, HX4), ghost (radar multipath at twice the strongest return's range with a speed;
  LiDAR a phantom 12 m ahead), skew (timestamps +0.4 s), crash (exit), freeze (hang).
- manager.py HarbourSensors (gateway.py: harbour_loop 5 Hz, gw_status "harbour" {enabled, sim, origin, sensors {radar,
  lidar: state, why, driver, rate_hz, items, batches, starts, restarts, latency p50 / p99 / max, skew_ms}, notices}):
  feeds the pose from gw.own() (FRESH telemetry only), restarts a crashed (exit) or hung (no line for silent_s 6 s)
  worker after restart_s 5 s with a NOTICE ("RADAR WORKER CRASHED - restarting", "LIDAR LOST - no data for 1.2 s",
  "RADAR OK"); notices only - never an alarm, a mode change or a frame to the AURIX; latest(kind) / health(kind) /
  on_batch callbacks for HX4 / HX5. Config gateway.json "harbour" (config.example.json: disabled, real drivers named);
  the demo: "harbour": {"sim": true, "world": demo_run/harbour_world.json} (demo_world.harbour_world()).
- Measured (tools/gateway_load.py --harbour; numbers in gateway/README.md 27 + TEST_REPORT.md): CPU per worker,
  the manager, latency sensor time -> gateway.
- Tests gateway/test_harbour_sensors.py HS1-HS10 (tools/check.sh stage gateway-misc). The design's HS1-HS8 list is
  covered: HS1 interface, HS2 / HS3 objects vs the truth, HS3 / HS4 blind zones, HS4 drop-outs, HS5 faults, HS6 health
  incl. the time skew, HS7 the worker restart; + HS8 the skeleton, HS9 no AURIX / network, HS10 the world.
- Review of 1fc5206 (opus, tools/api_review.py; findings: the owner's private reviews folder, review_1fc5206.txt) fixed in
  HX3-r: H1 the water band removes only FLAT clusters (< 0.15 m high) - judging the top alone could drop a swimmer's
  head at the noise edge; HS3 / HS4 now count the swimmer + kayak AFTER the filter as the worker sends them (= raw);
  M1 every age / timer in the manager on time.monotonic() (a GPS clock step never fakes an OK or a restart); M2 a
  worker that cannot start = LOST "worker cannot start: <error>" (no endless STARTING), the world goes to the worker
  on stdin (op world, --wait-world), never on argv; M3 the pose time = the gateway's telemetry RECEIVE time
  (gw.telem_rx), a batch whose pose is > pose_dt_max_s 0.5 s from its measurement = DEGRADED; L1 items nearest first,
  the cap counted apart ("capped"); L2 a restart's kill never blocks the gateway loop (own thread); L3 every batch
  carries "sim": true for simulated data + a check_config warning when "sim" replaces a configured real driver (a
  refusal of sim with a real AURIX link = HX4 note); L4 dead code out; L5 the delivery delay worker -> gateway >
  delay_max_s 0.2 s = DEGRADED.
- Round 2 (review of 8554afd, the last round, LOOP RULE 7): MED-1 the LiDAR filter NEVER deletes a flat cluster in the
  water band any more - it is KEPT and marked "surface": true (a person floating flat, a paddleboard 0.12 m, a log;
  the band ignores pitch / roll) and the tracker (HX4) decides; HS3 a 0.12 m paddleboard is kept in every scan. MED-2
  the pose time = gw.telem_rx (epoch, set with every TELEMETRY frame in gateway.py) with NO fallback: no receive time
  = no pose; HS7 a pose 1 s old = DEGRADED "pose ... from the measurement". Notes for HX4 / HX5 (not fixed here):
  the worker's pose age is epoch-based (a backward clock step fails safe through pose_dt); a restart's new real-driver
  worker may overlap the dying one (UDP bind, W148); a "sim" config with a REAL AURIX link must be REFUSED before HX5
  can steer on harbour data (a gate item for HX5); attitude compensation of the water band (BNO085 roll / pitch)
  belongs to HX4.
- NOT in HX3: the tracker / fusion / AIS association (HX4 - the world's AIS MMSI is there for it), the planner (HX5),
  any screen (HX6), real drivers (W148).

## 6. Perception (HX4) - the fused track data model

### 6.1 Track (gateway/harbour/tracker.py)
```
Track {id, t, x, y (local metres, ENU from the harbour origin) + lat, lon,
       vx, vy, P (4x4 covariance), len_m, beam_m, heading,
       cls "vessel" | "small" (kayak / dinghy / swimmer) | "buoy" | "static" | "unknown",
       conf 0..1, src bitmask R radar, L lidar, C camera, A AIS,
       last {R, L, C, A} times, hits_R, hits_L (in the last 2 s),
       range_ok (radar or LiDAR confirmed: >= 3 hits in the last 1 s and the last < 0.5 s),
       static (in the static map or speed < 0.2 m/s for 20 s), mmsi | None, age_s}
```
- Filter: a constant-velocity Kalman filter per track (state x, y, vx, vy), gating by Mahalanobis
  distance, global nearest neighbour assignment (Hungarian, scipy-free implementation); new tracks
  tentative until 3 hits within 1 s; deleted after 3 s without a range hit (camera / AIS-only
  tracks after 5 s).
- Camera: eWaSR obstacle boxes -> bearing + a range from the waterline geometry (gateway/vision
  geometry.py) with a large range variance; associated only by bearing to an existing range track or
  kept as a camera-only track (never range_ok).
- AIS: position + SOG / COG + size + class from ais.py targets, associated to range tracks by
  distance (< 30 m) and velocity; an AIS-only track (no range hit) is never range_ok.

### 6.2 Static map (gateway/harbour/staticmap.py)
Piers / breakwaters from the chart (charts/<area>/ coastline + OpenSeaMap piers), a per-harbour
occupancy grid (0.5 m) built from LiDAR over the visits (moored boats move: a cell decays in 1 h),
stored in state/harbour/<harbour>/static.npz; the route check (129) and the planner use it.

### 6.3 Measured (HX4)
Tracker latency (sensor time -> track) and CPU at 2 sensors + 4 cameras + vision with
tools/gateway_load.py --harbour; budgets: tracker cycle < 30 ms p99, planner < 50 ms p99 on the
ThinkPad L390.

### 6a. As built (HX4) - the fusion tracker + the static map

No app / hal / AURIX change; nothing is sent anywhere. Process model: the tracker is PURE Python and runs in the
manager's 5 Hz tick in the gateway thread (HarbourSensors._track; no own process - measured far under the budget,
below), fed every received batch at once; the sensor workers stay their own processes (5a).
- gateway/harbour/tracker.py: Tracker.feed(kind, batch) (radar plots -> ENU with the polar covariance range sd 3 m +
  0.5 % + size / 4, bearing sd 1 deg + size; LiDAR clusters sensor frame -> boat frame (+ mount_x) -> ENU, sd 0.25 m +
  size / 4); a constant-velocity Kalman filter per track (q 0.3 m/s^2; an out-of-order batch is predicted back with
  Q(|dt|)); the radar's ARPA speed / course = a VELOCITY update (sd 0.5 m/s - without it a radar-only moored boat's
  velocity wandered to 0.9 m/s); gating chi2 13.8 confirmed / 9.21 tentative after a spatial-hash Euclidean pre-gate;
  global nearest neighbour = Hungarian (own implementation) per connected component of the gated pairs; birth from
  every unassociated range measurement (velocity from ARPA, else 0 +- 1.5 m/s); duplicate tracks merged (< 1.5 m,
  < 1 m/s); camera(obstacles) (gw.vision.obstacles: rel / dist / range) = associated by BEARING (4 deg + half size,
  range ratio 0.5..2) to a range-CONFIRMED track (adds C only, the position stays the range sensors'), else a
  camera-only track (polar covariance, range sd >= 8 m or half the vision range interval); ais(targets)
  (gw.ais.list(), stale ignored) = linked to a range-confirmed track within 30 m and 2 m/s (MMSI + class vessel; kept
  by MMSI, unlinked beyond 90 m), else an AIS-only track (state = the report); cycle() = confirmation, steer_ok,
  coasting, deletion, static, class, confidence, ghosts, the static-map accumulation, the flags.
- DEVIATIONS from 6.1 (measured): steer_ok (= range_ok) per sensor - LiDAR >= 3 hits in 1 s, the last < 0.5 s (as
  6.1); RADAR >= 3 hits in 4 s, the last < 2 s (a 0.8 Hz radar can never give 3 hits in 1 s). Confirmation: radar 3
  hits in 4 s, LiDAR 5 hits in 1 s (10 Hz). Deletion: no range hit for radar 5 s / LiDAR 3 s (whichever saw it last),
  camera / AIS-only 5 s; a tentative track not confirmed within 3 s + its window. steer_ok is decided in ONE function
  (steer_ok_from) from the RANGE_SRC sources only (R, L); camera / AIS hits never count; a track whose range hits stop
  loses steer_ok at once (coasting) but is KEPT until deleted.
- Classes: vessel (AIS-linked, or >= 6 m moving), static (chart cell under the centroid, an accumulated LiDAR cell
  while slow, or the average speed < 0.25 m/s for 20 s, hysteresis 0.5 m/s), small (1.5..6 m, moving / low / flat:
  kayak, dinghy), person (person-like: < 1.5 m and low <= 0.6 m or flat), buoy (< 1.5 m, taller, static), unknown.
  "surface" = most LiDAR hits flat in the water band; a flat cluster is tracked like any other (HX3 round 2 MED-1):
  a LiDAR-confirmed paddleboard = steer_ok person-like. The water band is re-judged with the BNO085 attitude when
  TELEM2 has it (att_roll / att_pitch: z_water = -h cos r cos p - x sin p + y sin r); it changes only the label.
- CONSERVATIVE: a doubt makes an obstacle, never removes one: a radar multipath ghost (on the bearing of a nearer
  track at ~2x its range, radar only) is KEPT, marked ghost_suspect and stays steer_ok; a LiDAR phantom is an obstacle.
- gateway/harbour/staticmap.py: 0.5 m cells; chart layer (add_chart: the offline chart within 300 m - shallower than
  the shallow limit or no data inside a charted area; add_segment: piers / breakwaters) never decays; LiDAR layer:
  LiDAR-fresh static steer_ok tracks add weight to their footprint, decay tau 1 h, a cell counts at weight >= 20;
  JSON save / load (no numpy on the gateway; the design's .npz = JSON; "static_file" in the harbour config = saved
  every 5 min in a helper thread). A track whose centroid sits in a chart cell is static whatever its speed (a long
  quay's radar returns slide along it).
- Flags (tracker.flags() / HarbourSensors.tracker_flags(), gw_status "harbour" "tracker" {tracks, steer_ok,
  confirmed, cycle_ms, latency_ms, counts, flags, unsure, static_cells}): range_lost (U1: neither sensor OK /
  DEGRADED with data younger than stale_s radar 2.5 s / LiDAR 0.5 s), degraded [sensors not OK or stale] (U2), stale
  {sensor: age}, time_skew (U11, > 0.1 s), conflicts (U6: ais_range_disagree > 20 m on an object < 50 m;
  ais_without_range = an AIS track < 50 m not range-confirmed; camera_without_range = a camera track ahead
  (+-30 deg) < 30 m not range-confirmed), clutter (U9: > 15 new range tracks / s within 25 m), crowded (U9: > 20
  moving steer_ok tracks within 100 m), no_static_map (U10), no_pose, overload (> 300 tracks, sensor items over the
  cap, a cycle > 30 ms), error (a tracker exception: held 5 s by the manager, the tick never stops), ghost_suspect,
  attitude; "unsure" = the list of every active reason. The tracker never decides slow / stop - HX5 does.
- Measured: HT12 (the default harbour, both sensors + camera + AIS, 60 s simulated) feed / cycle p50 0.12 ms, p99
  0.61 ms, max 0.82 ms, 0.5 % of one core; tools/gateway_load.py --harbour (60 s, real worker processes, the boat
  circling): the manager's tick incl. the tracker p50 0.8 ms / p99 9.9 ms / max 11.9 ms, 47 tracks (26 steer_ok),
  the manager 4.3 % of one core (HX3: 1.4 %), latency gateway clock - sensor time at the fusion p50 170 / p99 500 ms
  (the radar's 300 ms + up to one 200 ms tick). Budget 30 ms p99: met. The ThinkPad L390 run with 4 cameras + vision
  is still open (R-H3).
- Tests gateway/test_harbour_tracker.py HT1-HT16 (stage gateway-misc): HT1 single, HT2 the default harbour (ferry
  AIS-linked, moored static, quay in the chart), HT3 crossing ids, HT4 head-on / overtaking, HT5 static map (LiDAR
  layer, chart layer, save / load, decay), HT6 kayak / swimmer / buoy / flat board + the attitude band, HT7 sea
  clutter never confirms, radar ghost kept + suspect, LiDAR rain = clutter flag with the swimmer still steer_ok, HT8
  camera-only / AIS-only never steer_ok + conflicts, HT9 loss / degraded / stale / skew flags, HT10 AIS-range
  conflict, HT11 lifetime, HT12 timing + the overload flag, HT13 determinism, HT14 read only (static + a run with
  sockets trapped), HT15 mutations (camera / AIS as a range source, no fresh-hit rule), HT16 the manager.
- Limits (for HX5 / the owner): LiDAR rain clutter DOES confirm clutter tracks (32 in 20 s in HT7) - they are
  obstacles (conservative) and the clutter flag says U9; a radar-only extended object's track can slide along it
  without a chart; the swimmer at the LiDAR's edge of view (1 point per scan, Pd 0.6) is steer_ok ~80 % of the time
  (HT6: 41 / 50 ticks) - between, it is a track that is not steer_ok (slow / stop only); real sensors (W148) will need
  the noise / gate values measured; the tracker's clock = the newest sensor / gateway epoch time, advanced by the gateway
  clock in every cycle and never back (a stopped data flow ages every track out: HT9); the data ages for the flags
  are monotonic (the manager).
- Review of a154416 (opus, tools/api_review.py; findings in the owner's private reviews folder) fixed in HX4-r: H1 AIS
  never removes or overwrites a track with range history (only AIS-only twins; a jump unlinks, the range track stays;
  a coasting linked track hands the MMSI to a steer_ok track at the report; range plots never gate against an AIS-only
  track); H2 a batch without pose / heading is never silent (own pose cleared = no_pose, "input rejected" unsure held
  2 s); H3 the tracker clock is MONOTONIC: the manager maps each batch onto it (monotonic receive time - the epoch
  age; an age outside -0.5..10 s = refused), camera / AIS / cycle use the monotonic clock, any time > 10 s from the
  tracker clock is refused + said; M1 a stored static map of another origin is refused (notice) and the tracker's
  origin is fixed from a stored map; M2 a camera object is absorbed by a range track only when the track's near side
  is within max(3 m, 30 %) of the camera estimate and inside the vision range interval (a swimmer in front of a
  moored boat = a camera-only track + the conflict); M3 flags fail closed before the first cycle ("tracker not
  started", range_lost) and the manager adds cycle_age_s ("tracker cycle old / not run" > 0.5 s); M4 two tracks hit
  by the same batch are never merged (rafted boats). LOWs: a vessel > 1.5 m/s over a chart cell is not static; the
  static map is serialised in the tick, written in the thread; the capped-items overload is held 2 s, not latched.
  Note (not fixed): tracks() / tracker_flags() have no lock - HX5 must read them from the gateway thread (the tick).
  Test HT17.
- Round 2 (review of 858168e, the last round, LOOP RULE 7): H-1 an AIS MMSI is handed from a coasting track only to
  a steer_ok track within 5 m of the report that is >= 10 m nearer than the old one (never to a moored boat 20 m
  away), and a report > 10 m from its range track keeps an AIS-only twin AT the report (never steer_ok, always an
  obstacle, ais_without_range U6); M-1 the manager accepts a batch age -0.2..2.5 s only (the real latency), else
  refused = "input rejected" + U11 "clock"; M-2 pose["t"] stays epoch (the tracker never uses it); M-3 the cycle age
  is set only by a cycle that ran; L-1 a stored map is used for the frame only when the boat is within
  chart_radius_m of its origin, else refused; L-3 a chart-cell track is static only below 0.5 m/s. Notes: L-2 an
  MMSI may move between two close tracks; L-4 a poor camera range on a ranged object = a camera-only duplicate +
  U6 (fail safe, HX5 tuning); L-5 a LiDAR-split hull stays two tracks; U10 judges the map's contents, not its
  coverage at the boat (HX5 gate item).

## 7. Local planner (HX5, SERIAL review: it steers the boat)

### 7.1 Method
A sampling velocity-obstacle / dynamic-window planner, 5 Hz:
- Candidates: heading offsets from the leg track -harb_max_dev..+harb_max_dev in 5 deg steps,
  reachable within 2 s at harb_rot_dps (dynamic window), x speeds {route kn, 0.75, 0.5, 0.25, 0}.
- Each candidate simulated 40 s (horizon_s) with a simple turn model (the AURIX's slew + the
  boat's turn rate at that speed) against every track's prediction (constant velocity, uncertainty
  grown with time: radius = 2 sigma + half size) and the static map.
- HARD constraints (any violated = the candidate is out): the path stays inside the corridor minus
  half the beam minus 1 m; the closest point to a moving track >= min_moving_m (15 m), to a static
  object / moored boat >= min_static_m (8 m); speed <= cap; a track that is NOT range_ok can only
  be avoided by a candidate with the leg heading (slow / stop only - M0-3).
- COST: deviation from the line + the time lost + the COLREGs penalty (7.3) + a "prefer slowing over
  turning" weight in narrow places (corridor free width < 3 x min_static_m: turning costs 3x) + a
  change penalty (no flip-flop: the previous choice gets a bonus).
- Output: the best candidate's heading + speed as navreq kind 4; nothing feasible = STOP (hflags b0).

### 7.2 Steering eligibility (M0-3)
Only a candidate whose heading differs from the leg track by > 3 deg counts as a TURN; a turn is
allowed only if every track that makes the straight candidate infeasible is range_ok. A camera-only
or AIS-only obstacle that blocks the straight path = slow, then STOP (never a turn). Logged per
decision (the explanation line says "camera only - slowing").

### 7.3 COLREGs (M0-7) for range_ok MOVING vessels (speed > 0.5 m/s)
Relative bearing + aspect classify each encounter with a CPA < 2 x min_moving_m within the horizon:
- HEAD-ON (target bearing within +-6 deg of the bow and its heading reciprocal +-15 deg): only
  starboard turns allowed (port turns = infeasible); «ΑΝΤΙΘΕΤΗ ΠΟΡΕΙΑ – ΣΤΡΟΦΗ ΔΕΞΙΑ».
- CROSSING FROM STARBOARD (bearing 6..112.5 deg): we give way: slow / pass astern; a turn to port
  across its bow infeasible; «ΔΙΝΩ ΔΡΟΜΟ ΣΕ ΣΚΑΦΟΣ ΔΕΞΙΑ».
- CROSSING FROM PORT: we are stand-on (Rule 17: keep course and speed) but - an owner choice beyond
  Rule 17, W145 asks - slow if it does not act (CPA shrinking
  under min_moving_m for 5 s = slow, then STOP); never a turn to port across its bow.
- OVERTAKING (COLREG 13: we come up from more than 22.5 deg abaft its beam = within 67.5 deg either
  side of its stern, and we are faster; review LOW-4): keep clear by min_moving_m,
  slowing preferred in a narrow corridor; BEING OVERTAKEN: hold course and speed (stand-on).
- Ambiguous classification (aspect unknown, a track turning, two encounters at once) = SLOW, then
  STOP (M0-4). Small craft (kayak / swimmer / dinghy) and buoys: not a COLREGs case = keep
  min_static_m.. min_moving_m, slow preferred.

### 7.4 "Unsure = stop" list (each one tested)
U1 radar AND LiDAR both lost / stale > 0.5 s = STOP; U2 one of them lost = degraded: cap degraded_kn
2 kn, turns only on the other sensor's range_ok tracks; U3 GPS untrusted / unc > harb_unc_m (the
AURIX holds too); U4 planner cycle late > 400 ms or a crash (the lease runs out at the AURIX);
U5 no feasible candidate; U6 conflicting data: a range track and an AIS / camera track disagree by
> 20 m on an object < 50 m, or a camera obstacle ahead < 30 m with no range return = slow then stop;
U7 a track inside min_static_m (something got too close) = STOP; U8 ambiguous COLREGs (7.3);
U9 more than 20 range tracks inside 100 m (clutter / a crowded quay) = slow; U10 static map / charts
missing for the harbour; U11 sensor time skew > 100 ms; U12 the boat's own heading / yaw rate
not fresh in the telemetry. STOP = navreq STOP flag; the AURIX holds; the alarm HARBOUR STOP; the
captain resumes.

### 7.5 The explanation line
One line per decision, Greek + English (lang keys harb.why_*), e.g. «ΔΙΝΩ ΔΡΟΜΟ ΣΕ ΣΚΑΦΟΣ ΔΕΞΙΑ –
επιβράδυνση 2 kn», «ΚΑΓΙΑΚ ΜΠΡΟΣΤΑ (κάμερα μόνο) – επιβράδυνση», «ΑΒΕΒΑΙΟΤΗΤΑ: χάθηκε το LiDAR –
ΣΤΑΣΗ»; in the navreq reason byte (code) + gw_status / harbour_view (text) + the audit log.

### 7a. As built (HX5) - the planner, COLREGs, "unsure = STOP" and the kind-4 output

No app / hal / AURIX change (HX2's firmware covers everything the planner needs - no gap found). Gateway only.
- gateway/harbour/planner.py: Planner (PURE: no I/O, `now` given, deterministic) + HarbourPlanner (the gateway glue,
  in the gateway thread: tracks() / tracker_flags() read there, the HX4 no-lock note). gateway/harbour/colregs.py
  (PURE: classify / encounters).
- DEVIATION from 7.1 (the same idea, sampled differently): a candidate = a LATERAL OFFSET LINE inside the usable
  corridor (corr - beam / 2 - 1 m - edge_margin_m 2 m: HX2's note "never lead the boat to the edge") every 1.5 m + the
  current xte (= the STRAIGHT candidate, heading = the leg track) x speeds cap x {1, 0.75, 0.5, 0.35}; the proposal =
  leg track + atan((offset - xte) / 15 m), clamped to +-45 deg (inside the AURIX's 60). Every candidate is simulated
  40 s at dt 1 s with the AURIX's 8 deg/s slew, the boat's turn rate (0.6 deg/s per kn: ~2.4 deg/s at 4 kn, HX2 note),
  the speed coming down with a 6 s constant (the coast-down: W147 note). A candidate path never leaves the usable
  corridor (or comes back to it), so a plan inside the corridor is guaranteed by construction (HP9, storm I89h).
- Distances: HULL TO HULL - own boat a 3 m disc (half of 6 m), the object half its size + 2 sigma (position sd, the
  velocity sd grown for at most unc_t_s 10 s); >= min_moving_m 15 for a moving track (> 0.5 m/s, not static), >=
  min_static_m 8 for static tracks and the static map (chart + LiDAR cells sampled every 1 m in the band ahead,
  bucketed by along-track distance; a large static track checked at every step).
- M0-3: a TURN = a proposal > 3 deg off the leg track (as 7.2); allowed only when EVERY track that blocks the straight
  candidate (at any speed) is steer_ok, and with a sensor degraded (U2) seen by a sensor still OK; otherwise only the
  straight candidate (slower) or STOP. A TENTATIVE range track (radar / LiDAR hits, not confirmed: <= 1 s LiDAR,
  <= 4 s radar) is not planned around until it is confirmed - else every spray / clutter return latched a HOLD
  (measured in HP15: a LiDAR spray return 7 m off the bow stopped the boat at once). Camera-only / AIS-only tracks
  (never "confirmed" by the tracker) ARE obstacles (slow / STOP only).
- "Slow, then STOP": nothing feasible over 40 s = the slowest STRAIGHT candidate judged over slow_horizon_s 15 s
  (creeping on while the danger is far); that fails too = STOP (U5). A cap below min_move_kn 1.5 kn is never sent:
  STOP instead (HX2 note: below the in-gear idle speed only idle). Narrow places (a static object within corr +
  min_static of the line in the next 40 m) = cap narrow_kn 3 (W144); turning costs x3 when 2 x corr < 3 x min_static
  (always with the defaults: slowing preferred).
- COLREGs (7.3) for steer_ok moving vessels / unknown with CPA < 30 m within 40 s: HEAD_ON / CROSS_STBD (give way:
  hflags b2) / CROSS_PORT (stand-on) / OVERTAKING (give way, any side) / OVERTAKEN (stand-on) / AMBIGUOUS (an unknown
  aspect = velocity sd > half the speed; two classes at once) = slow to 2 kn, STOP after ambig_stop_s 3 s (U8). A port
  turn is infeasible in every class but OVERTAKING. In a +-10 m corridor a 15 m pass of a moving vessel is mostly
  impossible: the planner then gives way by slowing / STOP (HP2 / HP3).
- Unsure (7.4) -> STOP in the same cycle: U1, U3 (pos_source 1 + unc <= 5 m), U4 (a cycle > 0.4 s after the last =
  STOP held 1 s; the budget 150 ms per plan, over = STOP; an exception = STOP "U4 planner error"), U5, U7 (a confirmed
  / camera / AIS track inside 8 m), U10, U11, U12 (no fresh position / heading), the tracker's stale / no pose / input
  rejected / overload / error / not started / cycle older than 0.5 s, ANY unknown unsure reason (fail closed), no
  checked route leg / the boat's route not the gateway's copy (prog_gw + count + the leg speeds kept from PROG_WP),
  the sim gate. U2 / U9 = cap degraded_kn 2; U6 = 2 kn, STOP after conflict_stop_s 3 s; U8 as above.
- Output: HarbourPlanner.fold() at the END of Gateway.nav_compose: while ACTIVE every cap of nav_compose (shallow,
  collision, vision) folds into ONE kind-4 request (cap = the lowest, the reason byte kept, hflags b5 / b6 / b7; a
  folded cap < 1.5 kn = STOP); the avoider's AUTO heading is not used (the planner steers). ACTIVE = the AURIX's own
  HARBOUR_STAT fresh (< 1 s) AND (state RUN / SLOW / HOLD, or OFF with a gateway-checked harbour route stored and the
  boat in STANDBY / MANUAL / CREWED HDG LOCK = waiting for the captain's START (never in UNCREWED HDG LOCK / REMOTE /
  MANEUVER / AUTO / DP: their kind 1 / 2 lease is needed - a kind-4 request is inert there): the entry needs a fresh kind-4 request without STOP).
  Otherwise nav_compose is unchanged (kind 0-3, HA-fold). Gateway.nav_send builds the 13-byte frame
  (proto.p_gw_navreq_harbour) with lease 1000 ms; harbour_loop calls nav_tick every 0.2 s while active (5 Hz; HP12);
  no fresh telemetry = nothing sent = the AURIX's lease HOLD. The captain starts it (T_HARBOUR START + CONFIRM: HX2; a
  copilot's START = the captain's ACCEPT); nothing here starts or resumes anything.
- The sim gate: harbour "sim" with an AURIX link that is not tcp:127.0.0.1 / localhost = every decision STOP + audit
  "harbour" what "refused" (HP11).
- Explanation (7.5): WHY keys harb.why_* (el + en in groundstation/lang, HP14) + the detail; gw_status "harbour"
  "planner" {active, stat, gate, stop, why, text_el, text_en, cap_kn, heading, reasons, colregs, turn, decisions,
  stops, errors, ms}; audit kind "harbour" on every CHANGE of decision (stop / why / reasons / cap / heading in 5 deg
  steps), not 5 per second. The HX6 UI shows it.
- Measured: a plan with synthetic tracks 1.4-1.6 ms on the development PC; the storm's closed loop (sensors + tracker
  + planner) ~3 s per simulated 60 s episode.
- Tests gateway/test_harbour_planner.py HP1-HP16 (stage gateway-misc) + the AVOIDANCE STORM
  tests/acceptance/storm_harbour.py (I86h planner cap, I87 a turn away from the line only with a steer_ok track within
  200 m, I88 15 / 8 m hull to hull to every SEEN world object or a HOLD begun in time, I89h corridor, I90 every
  STOP-class unsure / both sensors lost = STOP in the same cycle; random worlds + sensor faults; reach counters;
  --mutation-check: the steer_ok gate / the unsure STOP / the distance check removed each FAIL it). check.sh: 30
  episodes + a 15-episode mutation check; RUN_ACCEPTANCE quick 200 episodes + mutation check 40, full 600.
- DEVIATION from 9.2: the storm runs the gateway modules in-process against a kinematic boat that follows the AURIX's
  validation rules (Boat in test_harbour_planner.py), not the autopilot_sim server - the AURIX side of the invariants
  (I85, I86, I89 (a), I91, I92) is storm.c's (HX2); the end-to-end runs with the real simulator + gateway = HX7's
  scenario library (H1-H20).
- Limits (for HX6 / HX7 / the owner): a +-10 m corridor rarely lets the planner pass a moving vessel at 15 m - it
  stops and waits (the HOLD is latched: the captain RESUMEs); the turn model is the simulator's, not measured; a
  swimmer inside the LiDAR's blind zone (< ~18 m) is never seen (residual risk, MANUAL); the static map's coverage at
  the boat is not judged (U10 = contents, HX4 note); a tentative range track is ignored for up to its confirmation
  time.
- Review of 9eb0818 (opus, tools/api_review.py; findings in the owner's private reviews folder) fixed in HX5-r:
  MED-1 a tentative range return is never ignored entirely: within tentative_ahead_m 30 m ahead in the corridor band
  (+ min_static) or within 8 m of the hull = slow to degraded_kn 2 (never a turn, never a STOP by itself); MED-2 the
  pre-start kind-4 fold only for start_window_s 30 s after a station's T_HARBOUR START / RESUME (stations
  PROG_CHECK_TYPES -> Gateway.check_prog -> HarbourPlanner.note_start; never refused there) and only in STANDBY /
  MANUAL / CREWED HDG LOCK - the rest of the trip nav_compose is unchanged; the own pose older than own_max_age_s
  0.5 s = U12 STOP (gw.own() alone allows 3 s); a vessel-class track faster than 0.2 m/s = MOVING (15 m); the
  package imports no proto (local constants, HP13 compares them; HS9). Storm: I87 also checks the CAUSE (every
  track the planner turned for steer_ok with a fresh range hit: radar <= 2 s, LiDAR <= 0.5 s), I88 also during the
  HOLD's coast-down towards a static object and 15 m for a vessel > 0.2 m/s, a lost sensor goes SILENT first and
  LOST only after the HealthJudge's lost_s (radar 4 s, LiDAR 1 s), injected stale tracker / no pose / a blocked
  route = STOP (I90, a separate planner), a tracker step without flags = FAIL; the cap / corridor mutants are caught by
  HP16 (through HP9; the AURIX-like boat clamps them again in the closed loop); check.sh mutation check 25 episodes. NOT fixed
  (note for the owner / the flash gate): MED-3 the sim gate trusts the "aurix" link string (tcp to loopback = the
  simulator) - a real board behind a local TCP relay would pass; an AURIX identity (sim / board) in a frame needs
  FIRMWARE (a new field: TELEM2 is full) = a later step; until then the operating rule: never "sim": true on a boat.
  LOW notes: HP9's corridor check uses the planner's own offset (the storm I89h is the closed-loop one); a heading-only
  collision request in harbour keeps only its flag (the avoider's AUTO request always carries a cap).
- Round 2 (review of 1822580, the LAST fix round, LOOP RULE 7) fixed in HX5-r2: MED-1 the horizon is never cut at
  the leg's end (the line extended for the full 40 s; past the end + corr with the next leg not active = STOP);
  MED-2 the planner judges the sensor health itself (neither radar nor LiDAR OK = STOP, one not OK = 2 kn); MED-3
  the pre-start kind-4 window only in STANDBY with fresh telemetry (MANUAL / HDG LOCK keep their kind 1 / 2); the
  tentative rule = a REPEATED return (>= 2 range hits) from the bow to 30 m ahead within the corridor (single spray
  returns and the near field made the boat crawl at 2 kn for ever in the storm's passable harbour); a "static-map"
  blocker (chart cells + LiDAR cells laid only by steer_ok LiDAR tracks) may justify a turn. Storm: three scenario
  families - "mixed" (as before), "pass" (a passable harbour, 90 s: an off-line buoy or a slow boat to overtake at >= 15 m in a +-20 m
  corridor, moored boats clear, no fault: >= half of them must MAKE WAY past y 80 m - the original reach counter, met by the scenarios, not
  loosened), "ghost" (an AIS-only ghost report ON the line in a wide corridor: only the steer_ok gate stops a turn);
  I87 = the cause only (turn_for non-empty, each steer_ok + fresh range hit + within 200 m; the "any track nearby"
  shortcut removed); I88 during a HOLD for every object we close on ourselves; I86h also from the injected health;
  the mutation check requires each mutant's OWN invariant to fail (steer_ok -> I87, unsure -> I90, distance -> I88,
  degraded cap -> I86h). Notes (not fixed, LOOP RULE 7): the line is extended past a corner (a crosser on the next
  leg is judged on the extension, HX7 scenario); the budget is checked between candidates only; the injected
  stale / no-pose checks use a fresh planner; leg speed = waypoint k's (the AURIX's set_speed(wp[k])); unseen objects
  are not counted.

## 8. Supervision UI + teach and repeat (HX6, DISPLAY + requests)

### 8.1 Screens
Screen 2: the corridor (two thin lines), every track (colour by source theme.HARB_SRC: radar /
LiDAR / camera / AIS, a filled symbol = range_ok, hollow = not, the confidence as opacity), the
planned path (dashed, 40 s), the min-distance circles, the explanation line, «ΑΥΤΟΝΟΜΗ ΚΙΝΗΣΗ ΛΙΜΕΝΑ
ΕΝΕΡΓΗ» banner, mode bar item HARBOUR (allowed only with a stored checked harbour route), big framed
STOP (= HARBOUR HOLD; STANDBY stays where it is) and TAKE OVER (CREWED: "use the onboard joystick";
UNCREWED: REMOTE with the 13q2 rules or the 13q3 stick override). Screen 1: the corridor + tracks
in the AR overlay (green), the explanation line, the COLREGs line, STOP. The captain's start = the
vertical YES / NO + the CONFIRM; the copilot sees everything, the captain decides.

### 8.2 Teach and repeat (M5)
A recorded manual harbour exit / entry (the 13n3 live track) -> TRACKS window "Make harbour route":
Douglas-Peucker to <= 50 waypoints (2 m tolerance), cut the first / last cut_m (default 15 m from
the quay, M0-8), leg speeds = min(recorded speed, harb_max_kn), corridor corr_m (10 m) -> the gateway
check 129 (corridor clear of the static map + shallow, 114 / 120) -> stored per harbour
(state/harbour/<harbour>/routes.json, Daskalio first) and uploaded with the harbour flag.

## 9. Verification (M6) + the tests per step

### 9.1 Scenario library (HX7, sim + gateway simulators; every one also a storm seed family)
H1 empty harbour exit, H2 entry, H3 moored boats both sides (narrow), H4 head-on vessel, H5 crossing
from starboard, H6 crossing from port, H7 overtaking a slow boat, H8 being overtaken, H9 kayak without
AIS (LiDAR only), H10 swimmer (camera + LiDAR, small), H11 buoy on the line, H12 radar lost, H13
LiDAR lost, H14 both lost, H15 link lost (grace + kill), H16 GPS degraded, H17 a camera-only false
obstacle (slow, never turn), H18 AIS ghost (AIS-only, no range return: slow, never turn),
H19 a vessel that does not give way (stand-on then stop), H20 planner crash.

### 9.2 Avoidance storm invariants (tests/acceptance: storm.c for the AURIX side, storm_harbour.py
for the gateway planner against the simulated world; next free numbers)
- I85 lost link in harbour: no kill before detection + harb_ll_grace_s unless a harb_ll_cut fault;
  always a kill by then once idle + neutral (UNCREWED); never in CREWED.
- I86 the storm recomputes the cap ITSELF from its own inputs (leg speed, the parameters, the last
  navreq it sent, the station values) and checks the AURIX's cruise target <= it AND the true world
  SOG <= cap + 0.5 kn after a 10 s settle (review MED-5); the mutation "cap removed" must fail it.
- I87 never a heading change for a track that is not range_ok (gateway: the decision record).
- I88 never closer than min_moving_m / min_static_m to any world object when the sensors saw it
  (world truth), except a HOLD already commanded (then the closing speed only from the object).
- I89 (a) the AURIX-visible xte (its GPS) beyond corr_m + harb_corr_stop_m => HOLD in the same step:
  the COMMANDED throttle = idle and the modelled thr_est never rising while beyond; (b) the
  world-truth xte <= corr_m + harb_corr_stop_m + harb_unc_m unless a HOLD was already in force
  (drift at idle + neutral may carry it further: allowed); the mutation "corridor clamp removed"
  must fail it (review MED-5, round-2 LOW-2).
- I90 every unsure condition U1-U12 -> STOP within its time (planner) and HOLD on the AURIX within
  lease + 1 step.
- I91 STANDBY button / e-stop / joystick takeover win in the same step in harbour (as AUTO).
- I92 a lapsed / missing / wrong-kind lease in harbour = HOLD within the EFFECTIVE lease
  (min(frame, harb_lease_ms)) + 1 step; never the AUTO nav_lost_kn rule.
- Mutation checks (HX5 / HX7): disable the range_ok gate, the corridor clamp, the lease HOLD, the
  turn-rate slew, the lost-link grace each in turn -> the storm must FAIL.

### 9.3 Tests per step
- HX2 (FIRMWARE): sim HB1 entry refusals one by one (every condition 2.2 + 4.3 why), HB2 START + the
  CONFIRM (CREWED onboard / UNCREWED shore), HB3 cap clamp (route / param / navreq / ceiling), HB4
  heading accepted / refused (dev, xte growth), HB5 corridor back-steer + HOLD, HB6 turn-rate slew,
  HB7 lease lapse = HOLD, STOP flag = HOLD, HB8 GPS / compass HOLD, HB9 every exit (STANDBY, joystick,
  mode change, pause, arrival, upload), HB10 RESUME, HB11 lost-link grace (UNCREWED: kill at 63 s;
  each harb_ll_cut fault = immediate; link back = STANDBY; CREWED none), HB12 HARBOUR_STAT + EV_HARBOUR
  + the log field, HB13 FIRST TEST / big boat refused; both joystick sources + all --can; storm
  I85, I86, I89, I91, I92 (the storm's gateway sends kind-4 requests); proto.py + test_proto.
- HX3: gateway HS1-HS8 (interface, worker restart, sim radar / LiDAR objects vs the world truth,
  blind zone, drop-out, time skew), the demo world.
- HX4: HT1-HT10 (association, tentative / confirmed, range_ok, camera / AIS-only never range_ok,
  static map, deletion, latency / CPU numbers).
- HX5: HP1-HP15 (each COLREGs class, narrow = slow, every U1-U12, the explanation, the writer check,
  no flip-flop), storm_harbour I87, I88, I90 + mutations; gateway UH-style end-to-end with the
  simulated AURIX.
- HX6: station HU1-HU8 (corridor / tracks / plan drawn, STOP / TAKE OVER, the YES, teach: cut, <= 50,
  129), test_screen2 S1 matrix with the harbour layers.
- HX7: the scenario library H1-H20 as a suite, academy lesson + scenario (simulator only), MANUAL /
  MANUAL_EL chapter, tour stops, FIRST_TEST.md sea plan (empty harbour first, then one known boat,
  then traffic; the captain on board with the onboard joystick for every first run), the demo.

## 10. Failure analysis (what happens when ...)

| Failure | Detected by | Result |
|---|---|---|
| Gateway frozen / crashed | AURIX lease 1 s | HOLD (idle + neutral), alarm |
| Planner slow / crashed | lease (U4) | HOLD |
| Radar lost | worker status (U2) | degraded 2 kn, turns from LiDAR only |
| LiDAR lost | worker status (U2) | degraded 2 kn, turns from radar only (radar minimum range: close objects only from the camera = slow / stop) |
| Both lost | U1 | STOP |
| A sensor gives ghosts | tracker gating, U6 / U9 | slow / stop, never a turn on an unconfirmed track |
| A wrong turn proposal | AURIX dev / corridor / slew checks | refused, own guidance back to the line, HOLD if it repeats / the corridor is left |
| GPS degraded | AURIX (harb_unc_m) + U3 | HOLD |
| Compass lost | AURIX AUTO rule | gyro hold until idle + neutral, HOLD |
| Link lost (UNCREWED) | lostlink_step | STANDBY idle + neutral, kill after ~63 s or at once on a cut fault |
| Link lost (CREWED) | as today | alarm only; the crew drives |
| Throttle node lost | OD-3 | UNCREWED kill at once (unchanged) |
| Captain not watching | (procedural) | the MANUAL + FIRST_TEST: a captain on screen 1 is required; the station's driver-screen heartbeat (F1) refuses the start without screen 1 open |
| Swimmer not seen by any sensor | none (residual risk) | the speed cap 5 kn / slower in narrow places and the camera slow-down limit the harm; supervision required (MANUAL says so, as for AIS small craft) |

## 11. SENSOR COMPARISON (for the owner, W148)

Research 2026-11-12 (web; each fact with its source; NV = NOT VERIFIED - never guessed; prices are
retail listings that change). The owner chooses; until then HX3-HX5 run on simulated sensors.

### 11.1 Radar

| | R1 Navico HALO20+ (also HALO24 / Broadband 4G; Simrad / B&G / Lowrance badges) | R2 Navtech (CTS / "Colossus" protocol radars) | R3 Echodyne EchoShield |
|---|---|---|---|
| Type | 20" pulse-compression dome, marine | FMCW 360 deg radar (industrial / security) | electronically scanned (MESA), sector |
| Interface | Ethernet, UDP multicast | Ethernet, binary "Colossus Network Protocol" | Ethernet (1 Gbps detections / tracks) |
| Interface status | REVERSE-ENGINEERED (no official SDK) | OFFICIAL, documented | OFFICIAL SDK, docs behind a customer portal |
| Linux software | Mayara (Apache-2.0, Rust, Signal K Radar API) [1][2]; OpenCPN radar_pi (older) [3][4] | SDK with C++ source, .NET for Windows + Linux, ROS / ROS 2 examples [5] (SDK licence NV) | EchoWare API (licence NV) [6] |
| Range | max 36 nm (retail) [7]; minimum range 50 m vs 6 m in different listings = NV | NV | NV |
| Power | ~20 W typ / 29 W max, 3.9 W standby at 13.8 V (one listing) [8] | NV | NV |
| IP / marine | marine product; IP rating NV | marine suitability NV | NV (security / counter-UAS product) |
| Weight | ~5.9 kg [7] | NV | NV |
| Price | US$2,299 (US retail) / GBP 2,299.95 (UK) [7][8][9] | NV (quote) | NV (quote) |
| Harbour fit | good small-boat radar, but every pulse radar has a close-range blind zone + sea clutter; the open interface may break with a firmware update | documented protocol, used on land; marine use NV | not a marine navigation radar; NV |

Mayara also lists Raymarine Quantum, Furuno DRS-NXT and Garmin as supported / awaiting validation
(the project's own claim [2]); no official Furuno or Garmin SDK was found (NV). The old Mayara repo
was archived 2026-03-17, development moved to MarineYachtRadar/mayara-server [1][2].

### 11.2 LiDAR

| | L1 Livox Mid-360 | L2 Ouster OS1 (Rev7) | L3 Hesai XT32M2X |
|---|---|---|---|
| FOV | 360 deg H, -7..+52 deg V [10] | 128 ch, 45 deg V, 20 Hz [11] (360 deg H expected, NV on the page) | 32 ch, 360 deg [13] (V FOV NV) |
| Range | 40 m at 10 % / 70 m at 80 % reflectivity (100 klx), minimum 0.1 m [10] | 90 m at 10 % (200 m max) [11]; minimum range NV | 80 m at 10 %, 0.5-300 m [13] |
| Interface | 100BASE-TX Ethernet, PTPv2 / GPS sync [10] | Ethernet [11] | 100BASE-TX Ethernet [13] |
| SDK / licence | Livox-SDK2 C/C++, MIT, Linux + Windows [12] | ouster-sdk C++ / Python, BSD-3-Clause; ouster-ros [14] | HesaiLidar_SDK, licence NV |
| Power | 6.5 W avg (14 W peak heating < 0 C), 9-27 V [10] | NV; 9-34 V input per Clearpath [15] | 10 W typ, 9-36 V [13] |
| IP | IP67 [10] | IP68 + IP69K [11] | IP6K7 [13] |
| Weight | 265 g [10] | 447 g [15] | 490 g (another listing 800 g: NV) [13] |
| Price | US$899-1,092 (retail, varies) [16] | NV (quote) | NV |
| Harbour fit | cheap, light, close range (docks, kayaks); 40 m at 10 % is short | longest range, best sealing, open SDK; price NV | middle; SDK licence NV |

Water: calm water reflects the beam away (specular) = few or no water returns, which helps object
detection but sun glint, spray, rain and fog reduce range and add false points near the water - no
marine test data for these three models was found (NV) [17][18]; an Ouster OS2-128 was used on a
maritime platform in a research paper [19].

### 11.3 Recommendation (the owner chooses, W148)
- RADAR: Navico HALO20+ through Mayara / the Signal K Radar API on the gateway PC: the best supported
  open (reverse-engineered) route and the cheapest; risks = unofficial interface, minimum range NV
  (bench it), firmware updates. Documented alternative: Navtech (quote first: price + marine fitness).
- LIDAR: Livox Mid-360 for the close range (docks, small craft) with its MIT SDK; or Ouster OS1 for
  90 m and IP68 / 69K if the quote fits. Radar covers the long range, LiDAR the close range: they
  complement each other (the radar blind zone is the LiDAR's best range).

### 11.4 What PARTS.md / WIRING.md will need after the choice (NOT edited in HX1)
PARTS: the radar (+ its cable / power), the LiDAR (+ a marine mount, a sun / spray hood), a gigabit
switch port or a second NIC on the gateway, fused 12 V feeds, a PTP / time source if the LiDAR needs
it. WIRING: 12 V feeds + fuses, Ethernet to the gateway switch (boatlan0, nftables rule), the mounting
height / blind zones drawing, the e-stop does NOT cut the sensors (they are not actuators).

### 11.5 Sources (fetched 2026-11-12)
[1] https://github.com/keesverruijt/mayara [2] https://github.com/MarineYachtRadar/mayara-server
[3] https://opencpn-manuals.github.io/main/radar/0.1/index.html [4] https://github.com/opencpn-radar-pi/radar_pi/wiki
[5] https://navtechradar.atlassian.net/wiki/x/BgB7pg (the page came back empty; the claims are from the search snippet = treat as NV until read)
[6] https://www.echodyne.com/media/nvufa2ud/echodyne-ts-echoshield.pdf, https://apis.io/providers/echodyne/
[7] https://defender.com/en_us/b-g-halo20-radar-000-14539-001 [8] https://www.marinesuperstore.com/gps-radar/radar-scanners/lowrance-halo20plus-radar-dome
[9] https://www.campingworld.com/bg-halo20-20-radar-dome-w-20m-cable-358332.html?cgid=navico
[10] https://www.livoxtech.com/mid-360/specs [11] https://ouster.com/products/hardware/os1
[12] https://github.com/Livox-SDK/Livox-SDK2 (LICENSE.txt: MIT) [13] https://openelab.io/a/s/products/hesai-xt32m2x-32-channel-lidar
[14] https://github.com/ouster-lidar/ouster-sdk [15] https://docs.clearpathrobotics.com/docs/robots/accessories/sensors/lidar_3d/ouster
[16] https://dronexl.co/2026/09/21/livox-mid-360l-lidar-half-point-rate-robots/ (and retail listings)
[17] https://helpcenter.yellowscan.com/knowledge/why-cant-i-see-the-bottom-of-the-water-body
[18] https://ouster.com/blog/lidar-vs-camera-comparison-in-the-rain/ [19] https://arxiv.org/pdf/2603.08457
NOT verified: every NV cell above; the HALO minimum range; Navtech / Echodyne / Hesai prices and SDK
licences; Ouster power / minimum range / price; marine performance of all three LiDARs; Raymarine /
Furuno / Garmin open interfaces beyond Mayara's own list.

## 12. Risks + open questions

- R-H1 the radar's open interface is unofficial (R1): a firmware update can break it -> pin the
  firmware, a startup self-check, degraded mode U2.
- R-H2 small objects (swimmer, kayak) may be missed by every sensor -> low speed, supervision, the
  MANUAL says it like the AIS small-craft note.
- R-H3 the gateway PC load with 2 more sensors + vision (measured in HX4; the ThinkPad may need the
  vision worker paused in harbour mode).
- R-H4 COLREGs in a crowded harbour are often ambiguous -> the unsure rule stops often; acceptable
  (the owner's M0-4), tuned on the sea plan.
- R-H5 the lost-link grace (63 s with an idling engine and nobody in control) is a deliberate change of
  a safety rule -> SERIAL review in HX2, the owner's W147.
- Open: W144 (5 kn), W145 (15 / 8 m), W146 (+-10 m), W147 (grace 60 s + the cut list), W148 (sensors),
  NEW W149 (DP SPOT as the harbour HOLD when thrusters are fitted; never after a lost link).
- Review of 0491d5d (opus, api_review.py): MED-1..MED-5 + LOW-1/3/4/6 folded into this file
  (HOLD rates listed, effective lease, the ll.harbour latch, the grace ships at 0 until W147, I86 /
  I89 reworded); LOW-2 (the vision line) + LOW-5 (section B / I27 / I28 are amended by HX2 in the
  same commit as the code) in CLAUDE.md. Checked: alarm_bits2 is 32 bits (0x100 free), EV 32 free,
  0x90 / 0x3C free.
- Round-2 review of c86eb46 (the last round, LOOP RULE 7): MED-1 (fold only on the AURIX's own
  HARBOUR_STAT), MED-2 (grace range 0..0 until W147, then signed), MED-3 (no "rising" in the drift
  kill) + LOW-1..LOW-4 folded in. Left for HX2 to prove in code: the gear reaches N while the mode
  stays AUTO (harb HOLD), emergency() honoured outside STANDBY, the numbers free.

## 13. One-page sensor summary for the owner

RADAR - pick one:
1. Navico HALO20+ (Simrad / B&G / Lowrance) - about US$2,300 retail. Ethernet to the gateway PC,
   open software Mayara (Apache-2.0) speaks its (reverse-engineered) protocol on Linux / Windows.
   ~20 W, 5.9 kg dome. Unknown: true minimum range (50 m or 6 m?), IP rating. RECOMMENDED.
2. Navtech (Colossus protocol) - an OFFICIAL documented protocol + C++ / ROS SDK; price, marine
   suitability, licence: ask for a quote.
3. Echodyne EchoShield - official SDK behind a login, a security radar, price unknown: not for now.

LIDAR - pick one:
1. Livox Mid-360 - about US$900-1,100, 265 g, 6.5 W, IP67, 360 deg, 40 m at 10 % reflectivity, MIT SDK.
   Best for the close range (quays, kayaks). RECOMMENDED for a first boat.
2. Ouster OS1 - 90 m at 10 %, IP68 / IP69K, BSD-3 SDK; price / power on quote. The upgrade.
3. Hesai XT32M2X - 80 m at 10 %, 10 W, IP6K7; SDK licence and price unknown.

Why both: radar sees far and through rain but is blind very close; LiDAR sees close and sharp but
less far and suffers in spray / fog. Until you choose, everything is built and tested with
SIMULATED radar + LiDAR. Your answer: PROGRESS.md W148.
