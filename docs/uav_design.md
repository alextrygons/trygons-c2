# UAV magazine - design (step U1, 2026-11-04)

Source of truth for the rules: CLAUDE.md section V "UAV magazine". This file holds the architecture,
the choices and their reasons, the failure analysis and the open questions. Owner spec:
docs/owner_findings_2026-10-31.md section H (owner 2026-11-04). Steps U2-U6: PROGRESS.md.

Everything marked TO CONFIRM is not known and is never guessed in code: the code uses the default named
here and the owner confirms it (PROGRESS.md "WAITING FOR OWNER"; the owner answered W52-W76 on 2026-11-04: section
13 "Owner answers 2026-11-04 (step WA)").

## 0. Scope (binding)

- Camera / observation drones ONLY: launch, flight, observation, video, telemetry, landing.
- NO payload carriage, NO release mechanism, NO target / strike function - not in code, UI, docs, tests,
  content or the academy. Nothing in this design adds an output, a command or a screen element for one.
  How this is enforced (not only promised): section 4.3 (the MAVLink ALLOW-list: only listed messages /
  commands pass, the list holds no actuator / servo / relay / mount command) and section 3.2 (registration
  refuses a drone with any configured output other than its 4 motors).
- The boat: NO AURIX change. The ONLY thing the UAV function does on the boat is the hatch relay, through
  the EXISTING relay table (an "extra" group on / off relay, step 6 / 11f rules unchanged, storm I36 keeps
  passing). No new AURIX frame, no TELEM2 field (TELEM2 is FULL anyway), no alarm bit, no mode, no navreq,
  no speed cap, no joystick change on the AURIX.
- Not in this task: the UAV as an RF relay for the boat's link (later); charging (only the battery state
  is shown).

## 1. Owner's setup (spec H)

Six small 7" ArduPilot quads, STACKED inside the USV under a hatch: slot 1 = the top = the next one out.
Launch strictly in series, slot 1 first. One-way: they can NOT land on the USV. Each slot has a USB-C data
cable to the gateway PC (through a powered hub). Each drone: an analog 5.8 GHz VTX (a fixed channel per
slot) and an ELRS receiver bound 1:1 to an ELRS TX module on the boat.

## 2. Architecture

```
 shore station(s)                      gateway PC (on the boat)                          drones
 ---------------                       ------------------------                          ------
 screen 2: magazine panel,     JSON    gateway.py (asyncio)                     USB-C   slot 1..6 (in the
 UAV list, map symbols /   <-------->    uav/manager.py  (state, roles,  <-------------  magazine): register,
 tracks, Send route target,  (existing    pilot lock, checks, launch)    hub            GPS warm start
 right-click "send UAV here"  station     uav/hatch.py (relay + switches)
 screen 1 / camera view:      link)          |  T_RELAY (HELM station's
 UAV video + green UAV HUD                   |  behalf, extras group)  ---> AURIX relay table (unchanged)
 shore stick -> UAV setpoints                |
                                           uav/router_worker.py  (OWN process)    ELRS TX 1..6    ELRS RX
                                             MAVLink 2 router, one sysid / drone  (UART, MAVLink) ~~~~~~~ (MAVLink)
                                             ALLOW-list, signing, rates
                                           uav/video_worker.py (OWN process)      VRX 1..6 +      analog VTX
                                             capture -> ring -> MediaMTX UAV-1..6 USB capture <~~~~~~
                                           vision/ (existing worker): UAV feeds at low priority
```

- Gateway side, new package gateway/uav/ (inside gateway/ = already excluded from the ADS build; no new
  top-level folder). The time-critical / blocking parts run in their OWN processes like the loudhailer
  (13r) and vision: router_worker.py (serial ports of the 6 ELRS TX modules + the 6 slot USB ports) and
  video_worker.py (capture + encode). A crash = a notice + restart; the drones are never left in a mode
  that needs the gateway to stay safe (section 6: they hold / follow their mission on their own).
- Nothing in gateway/uav/ imports gateway/aurix.py or sends a frame to the AURIX itself. The hatch relay
  command goes through the SAME path as a station's relay button (stations.py forwards T_RELAY on the HELM
  station's behalf; the AURIX checks HELM, the relay table, FIRST TEST as always). A test proves it
  (U3: test_uav UA-import, like academy AC1).
- Station side: groundstation/uavpanel.py (the «ΜΑΓΓΕΜΑ UAV» panel: 6 slots), uavmap.py (symbols, routes,
  tracks), uavhud.py (green UAV HUD), uavfly.py (shore stick -> setpoints), uavpics.py (the pictures).
  DISPLAY + requests only; the gateway checks every request again.

## 3. Registry / magazine (H1, step U2)

### 3.1 Slots and identity
- A slot = a fixed USB port path on the hub (osdep.py resolves it, like the serial names: Linux
  /dev/serial/by-path, Windows the hub port location). The slot number is NEVER taken from the drone.
- Registry file on the gateway: state/uav_registry.json (sysid, FC serial number / board UID, name, VTX
  channel, ELRS binding = which TX module, firmware version, last check). Changed only by "Register" in
  STANDBY of the launch sequence (no launch running), copilot or captain.
- "Register" (per slot) reads over USB: HEARTBEAT, AUTOPILOT_VERSION (firmware, UID), the parameters of 3.2,
  SYS_STATUS / BATTERY_STATUS, GPS_RAW_INT, EKF_STATUS_REPORT, the pre-arm state (STATUSTEXT). A duplicate
  sysid, an unknown UID in a registered slot, a failed check = NOT launchable (shown red with the reason).
- Unregistered / failing = never launched (the launch sequencer re-checks every drone right before its own
  launch, section 5).

### 3.2 Parameter check (read only, the gateway NEVER writes a parameter)
The owner sets the drones with Mission Planner; registration only reads and refuses a mismatch (W60;
step X2: the parameter set is now docs/uav_params.param and was MEASURED against ArduCopter 4.7 SITL - section 14;
the drones' own version on the bench = still TO CONFIRM):
- the RC receiver ignored (RC_OPTIONS bit 0, U1 review HIGH-1, W68) and radio failsafe off (FS_THR_ENABLE 0: the
  ELRS RC channels are not a control path), GCS failsafe off
  (FS_GCS_ENABLE 0), EKF failsafe = AltHold (FS_EKF_ACTION 2, never Land), dead-reckoning failsafe off
  (FS_DR_ENABLE 0), fence action = report only (FENCE_ACTION 0: the gateway decides, section 6),
  battery low action = warn only (BATT_FS_LOW_ACT 0), battery critical action = Land at a fixed backstop
  value (BATT_FS_CRT_ACT 1 + BATT_CRT_VOLT / BATT_CRT_MAH: the last line of defence when the gateway is
  gone). RTL / SmartRTL are never a failsafe action (the "home" is the launch point at sea: it would fly
  there and land in the water).
- Outputs: ONLY the 4 motors configured (SERVOn_FUNCTION 33-36 once each; every other output 0 - also never a GPIO
  -1); anything else = refused. Every other release path must be off when the parameter exists (U1 review MED-1,
  built in U2): RELAYn_FUNCTION 0, RELAYn_PIN / RELAY_PINn -1, GRIP_ENABLE 0, CHUTE_ENABLED 0, WINCH_TYPE 0,
  MNT1_TYPE / MNT_TYPE 0 (no mount output), SCR_ENABLE 0 (no Lua scripts), RC1..16_OPTION 0 (no aux switch
  function). (Whether a camera tilt output should be allowed: W60 - default no.)
- Battery backstop values (BATT_CRT_VOLT / BATT_CRT_MAH, the drone's own Land when the gateway is gone): W71.
- MAVLink 2 signing key present (set by the owner with Mission Planner; the gateway keeps the same key in
  secrets.json "uav_keys") - W64. AS BUILT (U2 review LOW-7): NOT a registration check yet and the router does not
  sign / check frames yet (the codec can) - until W64 is answered (does ELRS MAVLink mode pass signed frames?).

### 3.3 Pictures
- The OWNER'S UAV PICTURE: assets/uav.png (copied in U1 from the owner's file, 1024 x 1024 transparent)
  + small sizes made by a tool like tools/make_icon.py (U2: tools/make_uav_icons.py -> assets/uav_16 /
  24 / 32 / 48 / 64.png, committed). Used wherever a vehicle is chosen or shown: the "Send route" target
  box, the map right-click menu, the magazine panel, the UAV list, the map symbol (turned with the drone's
  heading).
- The USV picture (owner W65 2026-11-04: a drawn symbol, no owner picture): assets/usv.png + usv_16..64.png drawn
  by tools/make_usv_icons.py - a simple boat seen from above, bow up, in the style of uav.png (dark grey hull, red
  outboard, dark outline); groundstation/uavpics.py draw_usv shows it (a line drawing only if the file is missing).

## 4. Telemetry, router, MAVLink (H5, steps U2 / U4)

### 4.1 Link
- One ELRS TX module per drone on the boat (1:1, up to 6), each on its own USB-UART (3.3 V TTL) of the
  gateway, ELRS 3.x MAVLink mode. ELRS is low bandwidth (a few kbit/s per link, TO CONFIRM by packet rate):
  telemetry at reduced stream rates (position / attitude 2-5 Hz, battery / EKF / GPS 1 Hz), mission
  upload, mode / guided commands, manual velocity setpoints at 10 Hz. NEVER raw RC (W59 / 6.3).
- The RC channels: in ArduCopter LOITER / ALT_HOLD / POSHOLD the RC sticks are LIVE inputs (throttle = climb /
  descend), so "radio failsafe off" alone is NOT enough - a frozen or low ELRS throttle channel would turn a
  "hold" into a descent into the sea (a forced landing, forbidden by the owner's rule 6.1). Review of U1 HIGH-1:
  registration REQUIRES the drone to IGNORE its RC receiver - RC_OPTIONS bit 0 "Ignore RC Receiver" (read only,
  3.2; built in U2) - and the bench confirms it for the drones' firmware (W68): with the bit set the drone in
  LOITER / ALT_HOLD / GUIDED / AUTO holds altitude whatever the RC channels say, and MAVLink over ELRS keeps
  working without a handset (ELRS MAVLink mode, W70). No RC override / MANUAL_CONTROL is on the allow-list.
- The shore never speaks MAVLink: stations send JSON requests (section 9) over the existing station link;
  the gateway turns them into MAVLink and sends UAV state to the stations in gw_status "uav" (1 Hz) +
  a "uav_telem" message (2-5 Hz, only the drones a station shows).

### 4.2 Router (gateway/uav/router_worker.py, OWN process)
- One sysid per drone (the registry), the gateway = its own GCS sysid / component. A frame from a sysid not
  in the registry, or on the wrong port, is dropped + counted (a drone is bound to its slot / TX module).
- Talks to the gateway process over a local pipe (JSON lines); reconnects serial ports by their stable
  names; never blocks the gateway's event loop.
- MAVLink codec: an OWN minimal MAVLink 2 encoder / decoder for the allow-listed messages only (no new
  dependency; CRC_EXTRA values from the official XML, checked by tests like dronecan_check.py) - pymavlink
  (LGPL-3.0) only as an optional cross-check in a test when installed (W64).

### 4.3 ALLOW-list (the code-level scope guard)
Only these pass in either direction; everything else is dropped and counted (router stats "denied"):
- from the drones: HEARTBEAT, SYS_STATUS, BATTERY_STATUS, GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD,
  GPS_RAW_INT, EKF_STATUS_REPORT, STATUSTEXT, HOME_POSITION, MISSION_CURRENT, MISSION_ITEM_REACHED,
  MISSION_REQUEST_INT, MISSION_ACK, COMMAND_ACK, PARAM_VALUE, AUTOPILOT_VERSION, TIMESYNC, SYSTEM_TIME,
  RADIO_STATUS.
- the mission READ-BACK (U1 review LOW): from the drones also MISSION_COUNT / MISSION_ITEM_INT, to the drones
  MISSION_REQUEST_LIST / MISSION_REQUEST_INT / MISSION_ACK.
- MISSION_ITEM_INT may hold ONLY NAV_TAKEOFF / NAV_WAYPOINT / DO_CHANGE_SPEED / NAV_LOITER_UNLIM (frames
  GLOBAL_RELATIVE_ALT(_INT); MISSION for the speed) - never DO_SET_SERVO / DO_REPEAT_SERVO / DO_SET_RELAY /
  DO_REPEAT_RELAY / DO_GRIPPER / DO_WINCH / DO_PARACHUTE / DO_MOUNT_* / DO_DIGICAM_* / scripting (U1 review MED-1).
  ARM / DISARM with param2 != 0 (force, 21196) is refused; REQUEST_MESSAGE / SET_MESSAGE_INTERVAL only for an
  allow-listed message; non-finite numbers are refused.
- to the drones: HEARTBEAT, MISSION_COUNT / MISSION_ITEM_INT / MISSION_CLEAR_ALL / MISSION_SET_CURRENT,
  PARAM_REQUEST_READ / PARAM_REQUEST_LIST (read only - never PARAM_SET), SET_POSITION_TARGET_LOCAL_NED
  (velocity + yaw rate fields only, body frame), TIMESYNC, SYSTEM_TIME, GPS injection (GPS_RTCM_DATA /
  GPS_INJECT_DATA, ONLY on the slot's USB port and only before ARM), and COMMAND_LONG / COMMAND_INT with
  ONLY these commands: arm / disarm (disarm only on the ground or in the slot), set mode (GUIDED, AUTO,
  LOITER, ALT_HOLD, BRAKE - never RTL / SmartRTL), takeoff, mission start, change speed, land (ONLY from
  the battery rule, 6.2), set message interval / request message.
- The list is one table in gateway/uav/allow.py; the test (U2) checks every other message id / command id
  is dropped, and greps that no other module builds MAVLink frames (like servo.c / relays.c writer checks).

## 5. Routes, missions, launch (H2, H3, step U3)

### 5.1 Route -> mission
- The station's "Send route" gets a target choice (vertical box with the pictures): ΣΚΑΦΟΣ / ΕΠΟΜΕΝΟ UAV
  (θέση 1) / a specific UAV (a flying one = its mission changed in flight). Map right click: «Πήγαινε εδώ»
  (BOAT: a 2-point route from the current position -> the normal prog_upload + every AUTO rule, nothing
  new on the boat side) and «Αποστολή UAV εδώ» (the next / a chosen UAV).
- The mission (gateway/uav/mission.py): TAKEOFF to the drone's initial altitude, then NAV_WAYPOINT per
  route point with its own ALTITUDE (default uav_wp_alt_m 50 m, editable per point), DO_CHANGE_SPEED
  capped at uav_max_speed_ms, and at the end LOITER_UNLIM over the last point (hover until a new route or
  the battery minimum). Frame = relative altitude to the launch point (home = the boat's position at
  launch, sea level + deck height).
- In flight: a new mission is uploaded and started from item 1 (the drone hovers in GUIDED / BRAKE while
  it uploads; a refused upload leaves the old mission running).
  AS BUILT (U3): the drone keeps flying its OLD mission while the new one uploads over ELRS (a hover first would
  leave it hovering after a refused upload); after the read-back MISSION START from seq 2 (the speed item: no second
  take-off). Seq 0 = ArduPilot's home slot, never flown, not compared in the read-back.

### 5.2 UAV route checks (gateway/uav/uavcheck.py; NOT shallow water)
Refused with a code (section 9) and the reason:
- energy: the route length + climb at the drone's measured consumption (registry: hover / cruise current
  from the last flights, conservative default until known) + the LANDING RESERVE + a wind margin must fit
  the battery now (in flight: the battery left);
- distance from the boat <= uav_rf_range_m (ELRS + VTX range, default 1500 m TO CONFIRM: EU 25 mW analog
  video) - from the boat's position at launch and its current position; while flying, a notice when the
  boat's own movement takes a drone towards the range limit;
- altitude <= uav_max_alt_m (120 m default, by the authorisation - W66);
- air no-fly zones (gateway state/uav_nofly.json: polygons / circles with an altitude band, a name and a
  source; a hand-made file, like the charts, "not authoritative") and the AIR geofence (state/
  uav_geofence.json, the same polygon rules as geofence.py, separate from the boat's);
- start = the boat's position at launch (trusted GPS; no trusted boat GPS = no launch).
- UAV routes / tracks are drawn in their own style (dashed, the UAV colour, the drone's name) - never
  mixed with the boat's route / track.

### 5.3 Launch sequence (gateway/uav/launch.py: a state machine, one step per tick, every wait timed)
Conditions to START (checked again before EACH drone): OWNER W59 (2026-11-04) - a launch goes WHEN THE COPILOT SAYS
SO: NO boat mode / speed condition (the earlier "STANDBY or DP SPOT / JOG below 2 kn" is REMOVED); above
high_speed_kn 5 kn the gateway gives the NON-blocking notice «ΥΨΗΛΗ ΤΑΧΥΤΗΤΑ – κίνδυνος στην απογείωση» (once per
crossing while a launch is prepared / runs, 0.5 kn hysteresis; gw_status uav high_speed = the line on the magazine
page; the captain's YES / NO question says it) - never a refusal or a stop. The rest stays: fresh telemetry; trusted boat GPS;
relay board OK + the hatch relay in the ACTIVE table; hatch switches healthy; no boat WARNING alarm;
the drone registered + checked + a mission loaded. The COPILOT (or captain) prepares (registry, routes,
"PREPARE LAUNCH"); the CAPTAIN approves it on screen 1 (the 13p vertical YES / NO box, trigger = YES);
a single operator answers his own question. A copilot can never launch alone.

Per drone (slot n, n = the top one still in the magazine):
1. health check (registry + live pre-arm, battery, EKF, sensors);
2. mission upload + read back (count + items);
3. hatch OPEN (first drone only): relay ON, wait for the OPEN switch within hatch_open_s (TO CONFIRM);
4. GPS ready (4.4 / section 7): good fix + EKF position within uav_gps_wait_s (30 s since UAV-r1); else the
   sequence STOPS - NO launch without GPS (owner W138 2026-11-11; the "climb, then catch up" fallback is removed,
   section 15);
5. ARM (only with the hatch FULLY OPEN by its switch);
6. vertical climb (TAKEOFF / AUTO) to its initial altitude (slot-staggered, default 30 + 5 m x (n-1));
7. "clear of the boat": relative altitude >= uav_clear_alt_m (10 m) AND climbing as commanded within
   uav_clear_s; then the slot's USB is gone (the breakaway cable, W58);
8. the next drone after uav_gap_s (15 s) at least.
After the last one (or a stop with no drone still in the hatch): hatch CLOSE (relay OFF), CLOSED switch
within hatch_close_s.

ANY warning stops the sequence (drones already in the air go on with their missions): a failed check,
the hatch fault, boat link / telemetry stale (NOT the boat's mode / speed: owner W59), a boat WARNING
alarm, relay board silent, the captain's or copilot's STOP, the station of the captain lost. A drone that
armed but did not climb (motor / prop fault in the hatch) = DISARM at once (it is on the ground), the
sequence stops, the hatch stays OPEN (closing on a drone is worse) + notice "UAV n did not leave the
hatch - check before closing", the close is then the captain's own button.

AS BUILT (U3): gateway/uav/launch.py - CHECK, UPLOAD (USB), HATCH, GPS, CMDS (GUIDED, ARM, TAKEOFF with their
COMMAND_ACKs), CLIMB (no climb in no_climb_s 6 s = DISARM), CLEAR (USB gone in usb_gone_s 6 s, MISSION START over
ELRS), GAP (gap_s from the previous ARM); every failure / warning = STOPPED (an armed drone still on the ground is
disarmed). The router owns the ELRS links too (gateway.json slot "radio") and builds allow.py's ctx itself from its
own state (hatch OPEN from the manager's 4 Hz op, stale after 1 s; landed from EXTENDED_SYS_STATE / the relative
altitude). (UAV-r1: no climb fallback / catch-up any more; a DISARM the drone refuses is sent once more FORCED,
allowed by allow.py only on the ground in its slot - section 15.)

### 5.4 Hatch + limit switches (gateway/uav/hatch.py)
- Relay: ONE on / off relay (ON = open) in the EXISTING boat-file relay table, group "extra", type
  "onoff", power_up "off", show_hud false; the boat file names it: "uav": {"hatch_relay": n, ...}
  (boats.py validates: an extras onoff relay, not in a pair). The station's relay panel shows it as a LAMP
  only; the gateway refuses a T_RELAY for the hatch relay from any station (code 125) - only the launch
  sequencer (and the captain's own HATCH CLOSE after a stop) sends it, on the HELM station's behalf. The
  AURIX can't tell (R7) and doesn't need to: the hatch never touches driving.
- Channel count: the boats already use relays 1-7; the DroneCAN relay board has 6 channels (TO CONFIRM),
  so the hatch needs relay 8 on a board with >= 8 channels (the Modbus option) or a bigger DroneCAN board
  (W57).
- Limit switches: CHOICE = read by the GATEWAY, through the modem-status inputs of a 3.3 V FTDI USB-UART
  (U1 review LOW: the TTL-232R CABLE brings out CTS but NOT DSR - use an FT232R BREAKOUT board that brings out
  CTS + DSR, with an external pull-up (e.g. 10 kOhm to 3.3 V) on each line: the FT232R's internal pull-ups are
  weak - W57a) (the same chip as the AURIX adapter, W17): CTS = OPEN switch, DSR = CLOSED switch, each switch
  closes to GND at its end position, the FTDI pull-up keeps the line high otherwise -> a broken wire reads
  "not at that end" (fail safe: "fully open" can only be read with an intact wire). pyserial
  getCTS / getDSR (already a gateway dependency), polled at 20 Hz, debounce 50 ms. Sealed (IP67) magnetic
  reed / limit switches. Reason: an AURIX input would need firmware + a NEW frame (TELEM2 is full) - the
  spec allows no AURIX change but the relay; a USB GPIO board would be a new part + driver. (W57.)
- Hatch states: CLOSED (CLOSED only), OPENING, OPEN (OPEN only), CLOSING, FAULT. FAULT = no switch change
  within the travel time / BOTH switches / NONE for longer than the travel time / a switch changing
  without a command / the switch cable unplugged -> the sequence stops + notice HATCH FAULT. Never ARM
  unless OPEN. Simulated hatch (gateway/uav/simhatch.py) with fault injection for tests + demo.

### 5.5 Two hatch hazards (recorded, not hidden)
- A relay-board reboot / power loss switches every relay OFF (R-2, 11f): with "ON = open" the hatch CLOSES
  by itself, possibly onto a drone climbing out. Mitigations in the design: the sequence watches the
  switches every tick (closing without a command = FAULT, stop, hold the remaining drones disarmed); the
  hardware should close SLOWLY with a soft edge; the owner chooses the actuator (W57).
- The open hatch lets spray / rain into the magazine: open only for the sequence, closed at once after the last
  drone clear (owner W59: a launch at any speed - above 5 kn the HIGH SPEED notice; slow down when you can).

### 5.6 The hatch when HELM is gone (U1 review MED-2)
- Every hatch relay command is a T_RELAY on the HELM station's behalf; the AURIX refuses it (NOT HELM) when no
  station holds HELM - the relay rules of M / N / 11f are NOT weakened for the hatch. So:
  - HELM lost / given up, the remote HELM link lost (CREWED or UNCREWED), the UNCREWED lost-link procedure, an
    ENGINE KILL, the e-stop, a monitor trip: the launch sequence STOPS at once (no further drone, no ARM);
    drones already flying go on; the hatch relay is NOT switched by anything (relays are never switched
    automatically - an open hatch stays open, a closed one closed) until a station holds HELM again and the
    captain (or the sequence after a new approval) closes it. Spray in an open magazine is accepted rather than
    a hatch that closes on a drone. The station shows HATCH OPEN as a notice.
  - A relay-board reboot still closes the hatch by itself (R-2, 5.5): FAULT, as above.
- W55 race: the gateway forwards the hatch command with the gateway's OWN HELM grant (never only gw.telem2
  ["helm"], which lags one TELEM2) and only once TELEM2 agrees; a NOT HELM refusal = one retry after TELEM2
  agrees, else a notice "HATCH NOT CLOSED - take HELM and close it" (U3 test).
- The USB laptop on board bypasses the gateway's refusal 125 (it is not behind the gateway): its relay panel
  treats the hatch relay as a LAMP too (no button) - the station knows the hatch relay from the boat file
  "uav" (U3; U1 review LOW).
- AS BUILT (U3): the gateway's own HELM grant = the holder of the last T_HELM the AURIX accepted (take / hand-over),
  else TELEM2's holder once it stayed 2 s; stations.helm_station() only when grant and TELEM2 agree; one retry within
  5 s, else the notice. The gateway knows the hatch relay from gateway.json "uav" "hatch_relay"; the stations from
  the boat file "uav" "hatch_relay" (boats.py validates both rules).
- Owner question W72: should the hatch close by itself when the sequence stops with no drone in the hatch and
  HELM still held? (Default: no - the captain's own HATCH CLOSE.)

## 6. Flight rules (H2, H6)

### 6.1 No forced landing except the battery (owner)
No GPS, lost link, warnings = HOLD / hover, never land. LOITER when the position is good, ALT_HOLD when
it is not (drifts with the wind - accepted by the owner's rule), BRAKE for an immediate stop. RTL /
SmartRTL / Land are never chosen for anything but 6.2.

### 6.2 The battery minimum
The gateway computes per drone every second: reserve = the energy to descend from the CURRENT altitude
at the landing descent rate (+ uav_land_margin_pct 30 %, TO CONFIRM). Battery left <= reserve -> LAND
where it is (over the sea = the drone is lost, accepted by the owner). 60 s and 20 s before: a CAUTION
on every station "UAV n LANDING IN 60 s - battery minimum" (station-local alarm, MasterLamps.set_local,
never an AURIX alarm). The drone's own BATT_FS_CRT_ACT Land is the backstop when the gateway is gone.

### 6.3 Manual control (H6, step U5)
- CAPTAIN: may fly a UAV with the shore joystick ONLY while the boat is in STANDBY: «ΧΕΙΡΙΣΜΟΣ UAV n»,
  top banner «JOYSTICK → UAV n» on both screens. The station then sends NO GW_RJOY to the boat (so no
  REMOTE / MANEUVER / shore CONFIRM / UNCREWED HDG LOCK can start: their AURIX rules need fresh packets)
  and the gateway refuses the captain's driving commands while the joystick is on a UAV (code 126;
  STANDBY always free). The boat leaving STANDBY (telemetry) ends it at once: the UAV holds, the joystick
  goes back to the boat (packets again, the normal "centred" rules apply).
- Input routing while the captain's joystick is on a UAV (U1 review MED-3): the 13p YES / NO selector IGNORES the
  stick (trigger, thumb, stick Y, hat) - a question on that station is answered with the MOUSE only (the trigger
  never answers a question and flies the drone at the same time); SAVE stays on its button. Refusal 126 covers
  every DRIVING_CMD of the captain's station AND every copilot command that would need the captain's ACCEPT
  (refused 126 at once, never queued for an ACCEPT: the captain is flying, not watching the boat) - incl.
  GENERATOR ON and RESTART, which need only the ACCEPT; STANDBY and the copilot-free commands (AUTO PAUSE, DP
  STOP) stay free.
- COPILOT: may fly a UAV from his own station any time (his joystick never drives the boat anyway); the
  captain keeps the boat.
- One pilot per drone at a time (gateway pilot lock): the FIRST pilot wins - another station gets 127 until the
  pilot gives the drone back or his station is stale / lost. OWNER DECISION W61 (2026-11-04, replaces the U1-review
  default "no takeover"): the CAPTAIN CAN TAKE OVER a drone from a copilot pilot AT ONCE - the copilot sees «Ο
  ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟΝ ΕΛΕΓΧΟ», the drone holds position during the hand-over (zero setpoint until the captain's
  stick has been seen centred). A copilot never takes a drone from the captain (127). The pilot's station link stale
  0.5 s = the gateway sends a zero setpoint (hold); 3 s / lost / logged out = the lock ends (the drone hovers).
- Assisted flying only (GUIDED velocity setpoints, POSHOLD-like): stick X / Y = direction / horizontal
  speed (<= uav_man_speed_ms, 10 m/s TO CONFIRM), TWCS lever = climb / descend (<= 2 m/s, never below
  uav_min_alt_m 10 m above the launch level, never above uav_max_alt_m), twist = yaw rate, let go = zero
  = hold position. The air geofence / no-fly zones hold the drone at the edge (the gateway clips the
  setpoint). «ΕΠΙΣΤΡΟΦΗ ΣΤΟ ΣΚΑΦΟΣ» = the UAV hovers (BRAKE -> LOITER), joystick back to the boat.
  It never means "fly back and land on the boat" (one-way drones).
- Lost link (drone <-> boat, or boat <-> shore) = the UAV holds (ArduPilot's GUIDED command timeout
  stops a velocity setpoint after ~3 s by itself; the gateway also sends zero when the shore is stale).
- AS BUILT (U5): gateway/uav/pilot.py (Pilots in the UavManager: uav_pilot {uav, take}, uav_ctl {uav, seq, ts, x, y,
  z, r} = the NORMALISED stick -1..1 at 10 Hz over UDP signed like rjoy (stations.udp_packet), uav_hold {uav}; answers
  uav_ack what "pilot" / "hold", uav_pilot_end {uav, why: captain | standby | lost | landed | ...}; gw_status "uav"
  "pilots"). The gateway makes the setpoint (pilot.setpoint, pure): x / y -> body-frame vx / vy <= uav_man_speed_ms 10
  m/s (diagonal too), the lever's climb <= uav_man_climb_ms 2 m/s slowing to the floor uav_min_alt_m / the ceiling
  uav_max_alt_m, yaw <= uav_man_yaw_dps 60 deg/s, a dead band 8 %; the air geofence / no-fly zones checked at the point
  reached in 3 s + 10 m = no horizontal speed (hold at the edge; already outside: only back in); an unknown altitude /
  position = no vertical / horizontal speed. Taking = DO_SET_MODE GUIDED (the lock dropped if refused); every new
  pilot's stick must be seen centred first. Giving back / an end = zero setpoint + BRAKE, LOITER 1 s later. The router
  op "ctl" = SET_POSITION_TARGET_LOCAL_NED frame 9 (velocity + yaw rate only), refused while landing for the battery,
  and the router's OWN zero setpoint 0.5 s after the last moving one (a gateway hang). stations.py: 126 for the
  captain's DRIVING_CMDs + REMOTE / MANEUVER asks while his joystick is on a UAV, for every copilot command that would
  need his ACCEPT and for the gateway's own ACCEPT questions (_ask_driver: never queued); STANDBY / AUTO PAUSE / DP STOP
  free; his rjoy dropped (rjoy_uav). Station: groundstation/uavfly.py (take / give / the 10 Hz sender; the lever's
  neutral zone 45-55 % = hold the altitude; no lever = the altitude held; the twist = shore_joystick.json "twist",
  default stick axis R), uavhud.py (screen 1: the UAV's video + its green HUD, the boat's mode / lamps / alarms /
  notices / bottom bar kept = hud.UAV_KEEP; RETURN TO BOAT on the "drive" place), app.py (no rjoy + the selector
  without the stick while flying, the banner on both screens, the camera view's bar 1 FLY / RETURN TO BOAT, the arrow
  to the boat in screen 2's UAV view, «Ο ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟΝ ΕΛΕΓΧΟ» + a local CAUTION), uavpanel.py (FLY UAV n
  buttons). Tests gateway UA30-UA32, station UV12-UV14.

### 6.4 The UAV never touches boat control
UAV modules never send to the AURIX (only the hatch relay through stations.py's normal relay path),
never a navreq, never a speed cap, never a boat alarm, never a mode change, never the boat's joystick
data. UAV AI (section 8) gives notices / boxes only - the boat's vision stage-3 speed cap stays from
the boat's own cameras only.

## 7. GPS warm start (H4, step U2)

- Goal: a fix within seconds after the hatch opens. While the drone sits in its slot (USB), the gateway
  sends it time + position (+ ephemeris when available) through the flight controller's GPS injection
  (GPS_RTCM_DATA / GPS_INJECT_DATA carry raw bytes the FC writes to its GPS port) as u-blox UBX-MGA
  messages (MGA-INI-TIME_UTC, MGA-INI-POS_LLH, MGA-GPS/GAL-EPH or an MGA-DBD dump). Needs u-blox GPS on
  the drones; whether the drones' ArduPilot forwards UBX-MGA bytes to the u-blox untouched: SITL (step X2,
  section 14): the GPS_RTCM_DATA frames are taken by ArduPilot without error; the forwarding to a REAL u-blox and the
  faster fix = still bench only (SITL's GPS has no time-to-fix model).
- FINDING: the spec says "the boat's own u-blox data", but the boat's u-blox is wired to the AURIX (UART),
  not to the gateway, and the gateway only sees the AURIX's position / time in telemetry. Reading UBX
  ephemeris from it would need an AURIX change (forbidden here). Options (W56): (a) time + position from
  telemetry only (MGA-INI: a WARM start, no ephemeris); (b) a small USB u-blox on the gateway as the
  "assist receiver" (MGA-DBD / EPH dump, near the magazine) [recommended, with (a) as fallback];
  (c) u-blox AssistNow Online / Offline cached when the gateway has internet (a u-blox token).
- Note: the magazine top must not be metal (the drone may even get its own fix in the slot).
- REMOVED in step UAV-r1 (owner W138 2026-11-11: no launch without GPS, section 15). The old text, kept for the
  record: FALLBACK "climb, then catch up" (any boat mode / speed: owner W59 2026-11-04): no good fix within uav_gps_wait_s -> ARM in
  ALT_HOLD, climb vertically to uav_fallback_alt_m 15 m - the climb is the allow-listed MAV_CMD_NAV_TAKEOFF in
  ALT_HOLD with param3 = 1 ("horizontal navigation by the pilot acceptable": ArduCopter's ALT_HOLD user takeoff
  without position), NEVER a throttle / RC input (none is on the allow-list; the RC receiver is ignored, W68);
  that ALT_HOLD then holds the altitude with the RC ignored: MEASURED in SITL (step X2, section 14): IN THE AIR
  ALT_HOLD with the RC ignored HOLDS the altitude; ON THE GROUND the ALT_HOLD take-off (TAKEOFF param3 = 1) is
  ACCEPTED but does NOT climb, and the following DISARM is refused (the drone stays armed in the slot) - the fallback
  does not work as designed: owner decision W138; hover until GPS + yaw are good (EKF-GSF yaw in
  flight; the compass near the hull is not trusted; the boat's heading only as a hint), then switch to
  AUTO and the mission. No landing because of missing GPS - it hovers until GPS or the battery minimum.
  In ALT_HOLD it drifts with the wind: the sequence waits with the next drone until this one is clear.

## 8. Video + AI (H5, step U4)

- ANALOG VTX -> six analog 5.8 GHz VRX on the boat (one fixed channel per slot; no two drones on one
  channel) -> USB capture into the gateway (video_worker.py, OWN low-priority process) -> the gateway
  ring (video/ring/UAV-n, the recorder's tracks in every SAVE / ⏺5' / ⏺1h) and MediaMTX streams UAV-1..6
  to the shore, lighter than the captain's main camera (LOW profile, ~300-500 kbit/s TO MEASURE), counted
  in cameras.update_priority below the captain's video; the station pulls only the streams open on a
  screen; data saver / STARLINK = NO UAV video for anyone (the 13n2 rule for every camera view wins; U1 review LOW).
- Capture load (TO MEASURE in U4 with tools/gateway_load.py, numbers into gateway/README.md): six
  uncompressed USB 2 captures at 720 x 576 YUY2 25 fps (~166 Mbit/s each) do NOT fit one USB 2 bus ->
  design target 640 x 480 at 15 fps, MJPEG-output capture devices preferred, spread over the laptop's
  root ports; encode with Intel Quick Sync (VAAPI on Ubuntu / QSV on Windows) on the i5-8265U, x264
  ultrafast as the fallback; only drones that are flying are encoded (a slot drone's VTX in pit mode).
- AI: the same eWaSR worker (the "AI" badge) on the UAV video: the boat's FRONT camera ALWAYS first; UAV
  feeds only when open on a screen, round-robin at <= 0.5 fps each, only with CPU headroom (the existing
  temperature / throttle guard). Output = boxes + notices only. NOTE: eWaSR was trained on views from a
  boat at water level; a top-down aerial view is outside its training - its results there may be poor
  (U4 measures; an aerial model = W62).
- Map: each drone's position, heading, altitude, track, battery, link quality (RADIO_STATUS / ELRS
  LQ), its picture as the symbol.
- AS BUILT (U4): one capture + ONE encode per running slot (video_worker.py, ffmpeg tee: MediaMTX uav_<n> + the
  ring UAV-<n>); Quick Sync when a 5-frame probe encodes, else x264; only flying drones (+ "always" slots) run; the
  UAV AI = a SECOND eWaSR worker process (1 thread) so the boat's FRONT camera worker is never shared - it waits while
  the boat's vision is PAUSED / DEGRADED or the CPU is busy; link quality = ELRS HEARTBEATs heard in 5 s (+ RADIO_STATUS
  rssi when sent, TO CONFIRM W70); measured numbers: gateway/README.md section 26. W62: only the simulated aerial
  test picture was analysed (no real aerial footage) - the detection quality on real UAV video is still open.
- U4 review fixes (review of 4756f97): MED-1 a battery % not heard for BATT_FRESH_S 5 s is STALE = never judged (no
  LAND on an old value; the drone's own BATT_FS_CRT_ACT backstop) + the notice "UAV n BATTERY DATA STALE"; MED-2 when
  the battery rule latches, a command already on its way (mode / mission start / take-off / speed / ARM) is dropped and
  never re-sent, and allow.py itself refuses those + setpoints / mission changes while ctx battery_land (LAND / DISARM
  pass); LOW landed() needs |rel_alt| < 0.5 AND |vz| < 0.3 (or EXTENDED_SYS_STATE), the deck fallback for a DISARM only
  from a low last altitude, MISSION_SET_CURRENT answered only once MISSION_CURRENT shows the seq (re-sent 1 s, 4 s), the
  landing time in two stages (uav_land_final_m 10 m at uav_land_final_ms 0.5 m/s, W71 / W75 TO CONFIRM) in the battery
  rule and the route reserve, the battery stage latched per flight (no 60 <-> none bouncing), the video worker's first
  hello before the encoder probe (+ 30 s until its final hello), the UAV AI's core = the highest one the boat's vision
  does not use and its pipe written by a writer thread, UA26's grep also over the gateway's nav_* code; documented: an
  AURIX answer after the 5 s hatch-relay timeout is ignored - the limit switches show what the hatch did.
- AS BUILT (U4) 6.2: the battery rule lives INSIDE the router process (it holds battery_land; the gateway process
  cannot ask for LAND) - gateway/uav/battery.py; capacity = BATT_CAPACITY from the registration.

## 9. Names for U2-U6 (targets; a step may rename within its own commit and updates CLAUDE.md V)

Gateway (gateway/uav/): manager.py, registry.py, usbslots.py, allow.py, mavlink.py (codec),
router_worker.py, gpsassist.py, mission.py, uavcheck.py, nofly.py, launch.py, hatch.py, simhatch.py,
pilot.py (manual control + pilot lock), video_worker.py, flightlog.py (per-drone flight log, CSV / GPX
export), simdrone.py (the simulated drones). Config gateway.json "uav": {enabled, slots, tx_ports,
vrx_devices, hatch_switch_port, ...}; boat file "uav": {"hatch_relay": n, "slots": 6}.
State files: state/uav_registry.json, state/uav_nofly.json, state/uav_geofence.json, flight logs
under the data folder (uav_logs/<name>/<date>.csv).

Station <-> gateway messages (JSON, the existing station link; English in the protocol):
uav_register {slot}, uav_route {target: "next" | slot (AS BUILT U3: the slot, not the sysid), wps: [{lat, lon, alt}]},
uav_launch_prepare {count, confirm}, (the captain's YES / NO = AS BUILT U3 the existing screen-1 driver_ask /
driver_answer, always on the captain's screen; no uav_launch_answer), uav_launch_stop, uav_hatch_close, uav_pilot {uav, take:
bool}, uav_ctl {uav, seq, ts, x, y, z, r} (AS BUILT U5: the normalised stick, 10 Hz, UDP signed; the gateway makes the
velocity setpoint), uav_hold {uav}, uav_view {uav}; from the
gateway: gw_status "uav" (slots, hatch, sequence, pilots), uav_telem, uav_ack {code}.
No new AURIX frame. No new app/proto.h define.

Gateway refusal codes (proto.py ERR, next free): 122 UAV not registered / check failed, 123 UAV route
refused (energy / range / altitude), 124 UAV route in a no-fly zone / outside the air geofence,
125 launch conditions not met (boat telemetry / GPS / WARNING, hatch, relay board, captain - never the boat's speed
or mode: owner W59) - also the hatch
relay from a station, 126 the joystick is on a UAV (a driving command refused; STANDBY free), 127
another pilot flies this UAV.

Station: groundstation/uavpanel.py, uavmap.py, uavhud.py, uavfly.py, uavpics.py; lang keys uav.*;
assets/uav.png + uav_<px>.png; tools/make_uav_icons.py.

Tests: gateway/test_uav.py UA1-UA.. (registry / USB checks / parameter check / allow-list / codec CRC
/ warm start bytes U2; mission build + checks + launch sequence + hatch faults U3; router + telemetry +
video priority U4; pilot lock + setpoint limits + STANDBY rule U5), groundstation/test_uavui.py UV1-UV..
(panel, pictures, map symbols + own route style, HUD, banner), academy AC51.. (the lesson on the
simulator, U6). Gateway storm (tests/acceptance/storm_gateway.py) invariants reserved I80-I84:
I80 no UAV module sends anything to the AURIX but the hatch relay through stations.py; I81 never ARM
without the hatch OPEN switch; I82 never a LAND command except the battery rule, never RTL; I83 a
captain's UAV control only with the boat in STANDBY, no GW_RJOY to the boat during it; I84 only
allow-listed MAVLink leaves the router.

Simulation (U2): gateway/uav/simdrone.py = 6 small simulated copters speaking the same MAVLink 2 subset
(modes, mission, battery model, GPS time-to-fix incl. the warm start, faults: no fix, motor fail, link
loss, battery), over UDP / a virtual serial pair; simhatch.py; simulated VRX = test pictures through the
existing simcam.py. (REPLACED by step X2, 2026-11-11: ArduPilot's own Cygwin SITL build runs NATIVELY on
Windows 11 - no WSL - and the official Linux build on Ubuntu; tools/get_sitl.py + gateway/test_sitl.py = real
ArduCopter behind the real router, see section 14. simdrone.py stays for the fast tests.) `autopilot_sim server` is not changed (the drones
are gateway-side only).

## 10. Failure cases (each gets a test in its step)

| Case | Reaction |
|---|---|
| hatch: no switch change in time / both / none / moved without a command / switch cable unplugged | FAULT: sequence stops, notice HATCH FAULT, no ARM |
| relay board silent / table not active / hatch relay missing | no launch (125); a running sequence stops |
| relay board reboot during launch (relay OFF = hatch closes) | FAULT seen by the switches, stop, remaining drones disarmed; 5.5 |
| drone health / parameter / pre-arm fail | not launchable (122), the sequence stops at that drone |
| mission upload / read-back fails | stop, the drone stays disarmed |
| no GPS fix in uav_gps_wait_s | the sequence STOPS, the drone never armed (UAV-r1, W138: no climb fallback) |
| armed but no climb (motor / prop) | DISARM at once (refused = FORCED, only in its slot: UAV-r1), stop, hatch stays OPEN, notice |
| boat WARNING / telemetry stale during the sequence (W59: the boat's mode / speed = the HIGH SPEED notice only) | stop; flying drones go on |
| ELRS link lost to a drone | it holds / follows its mission, ends hovering; notice UAV n LINK LOST |
| video lost | notice; nothing else changes |
| boat <-> shore link lost | the gateway keeps the drones holding / on mission; manual control -> hold |
| gateway / router process crash | drones go on (AUTO) or hold (GUIDED timeout); router restarted, notice |
| boat moves away from a hovering drone | notice "UAV n towards the range limit"; the crew decides |
| no-fly zone / air geofence reached in manual flight | setpoint clipped, hold at the edge, notice |
| GPS lost / EKF bad in flight | ALT_HOLD (FS_EKF_ACTION 2), drift, never land; notice |
| battery minimum | LAND where it is (60 s / 20 s CAUTION first) |
| two stations try to fly one drone | pilot lock (127) |
| captain flies, someone sends a driving command | refused 126 (STANDBY free) |
| a frame from an unknown sysid / wrong port, or not allow-listed | dropped + counted |
| duplicate sysid / unknown UID in a slot | not launchable |
| slot n launched while slot n-1 still there | impossible: the top slot is the lowest-numbered slot still present by USB + registry; a mismatch = stop |

## 11. Legal note (for MANUAL / MANUAL_EL in U6; not legal advice)

EU Regulations 2019/947 and 2019/945 apply in Greece (competent authority for drones: the Hellenic Civil Aviation
Authority, HCAA = Αρχή Πολιτικής Αεροπορίας, ΑΠΑ - U6 review L9: not the older ΥΠΑ; the current name confirmed by the owner 2026-11-05, W78).
UAS geographical zones (2019/947 Art. 15) + airspace coordination near airports / restricted areas; third-party
liability insurance as the authorisation / Greek law require (Regulation (EC) 785/2004 for 20 kg and more). BVLOS flight is in the SPECIFIC category: an operational authorisation (SORA, or a PDRA
where one fits), a registered UAS operator, a remote pilot with the required competency, insurance; take-off
from a moving vessel and flights near people / vessels need to be covered by the authorisation. Radio:
EU limits (analog 5.8 GHz video typically 25 mW EIRP, 2.4 GHz 100 mW EIRP) limit the range. State / Navy
operations are outside the EASA rules (Regulation 2018/1139 Art. 2(3)) and follow their own national
rules. Cameras: data protection (GDPR) applies to recordings of people. Responsibility stays with the
operator and the remote pilot. Also (U1 review LOW): UAS OPERATOR REGISTRATION with the HCAA (the operator number
on every drone) and DIRECT REMOTE ID broadcast (EU 2019/945 / Delegated Regulation 2020/1058; a drone without a
class mark needs an add-on Remote ID module in the Specific category, as the authorisation says).

## 10a. U5 review fixes (review of 8f8441e)

MED-1 the pilot lock is exclusive: a take is refused (122) while the launch sequence owns the drone (its slot is the
sequence's current one) or it is still "launching" / in the climb-fallback "catchup"; uav_route for a flown drone =
127 ("give it back first"); catchup_tick and the launch's CLEAR never start a mission under a pilot (CLEAR: notice,
the drone counts as launched). MED-2 pilot.setpoint: the drone's data age (alt_age = the router's + the status' age)
> data_stale_s 1 s = NO motion; within it the position is moved on with the ground speed and the altitude floor /
ceiling use the climb rate x the age. MED-3 a climb / descent is checked against the no-fly zones' altitude bands (with
the look-ahead + margin), the horizontal look-ahead at the altitude after the climb. MED-4 the captain's take refuses
every question already open for his ACCEPT (126, stations.refuse_asks_for). MED-5 BATTERY_STATUS refreshes the battery
time only with a battery_remaining % (-1 = not refreshed). LOWs: batt_stale told also when armed with already-stale
data (once per arming); a give-back after the gateway ended the lock = OK; pilot_msg allows any non-VIEWER station (by
design, documented); hover() without a GPS position = ALT_HOLD (no BRAKE / LOITER). NOT fixed (LOW, recorded in
PROGRESS): the MISSION_SET_CURRENT confirmation still accepts a MISSION_CURRENT after the request that shows the seq
(a drone already at that seq confirms at once). Test UA34 (+ UA30a altitude-band zone).

## 11a. Step U6 AS BUILT

The academy lesson "UAV launch and observation" runs on the academy's OWN in-process UAV simulator (academy/uavsim.py):
a simulated boat + magazine + hatch + 6 camera drones that drive the gateway's pure rule modules (launch.py as its
"io", allow.check_out for every simulated command / mission item, uavcheck / mission / battery) - no network, no
process, never a gateway (the academy's simguard rules: AC1-AC3, AC51-AC54). Tour chapter "uav" (every UAV page
button + the UAV screen parts + the legal note). MANUAL ch. 21 / MANUAL_EL ch. 16 with the legal note of section 11.
PARTS.md "UAV magazine" + WIRING.md 3H. Storm invariants I80-I84 in tests/acceptance/storm_gateway.py part "uav"
(the router in this process with fake links + drones; the pilot lock + rjoy; the launch sequence + the hatch relay
through uavlink / stations; a static scan). The gateway load numbers stay those of gateway/README.md section 26.

## 11b. U6 review fixes (review of f7aed5d)
- MED-1: academy test AC2 follows `from uav import x` submodules and relative imports (`from . import x`;
  test_academy.submodules_of), AC1 checks the gateway/uav rule modules the academy runs in its process, AC3 imports + RUNS
  them (a short UavSim launch) and fails on a network module inside them. Mutation: `import socket` in
  gateway/uav/launch.py fails AC1, AC2 and AC3.
- MED-2: storm I82 decides "may land" with its OWN battery formula (storm_gateway._own_land: the slowest landing rate,
  30 A, the margin, x 1.5 tolerance, from what the fake drone told the router in the last 6 s) - never the router's
  battery_land latch. Mutation: battery.judge "land" always fails I82 (952 fails in 100 episodes).
- MED-3: a launch STOPPED while a drone climbs out lets it go (launch.stop -> io.launch_ended -> phase "hold"); a
  "launching" phase blocks the pilot only while the sequence still owns that drone (seq.busy and seq.cur). Test UA36.
- LOW: L1 a climb / descent samples the vertical leg every 2 m (a thin no-fly band is seen); L2 an old position is
  moved along the course over ground (GLOBAL_POSITION_INT vx / vy -> cog), the heading only without one; L3
  data_stale_s 1.8 s (one lost position packet tolerated, two are not); L4 SYS_STATUS battery_remaining -1 never
  refreshes the battery time; L5 I83 uses the storm's own "drives the boat" test; L6 I81 also checks strictly in
  series / once / never after a STOP, the I80 static scan relative imports + `.aurix` / getattr "aurix"; L8 WIRING 3H
  (FAULT only after the travel time, a changeover / controller stage for the actuator, the 6-channel DroneCAN board
  can't host relay 8); L9 the legal note: UAS geographical zones + airspace coordination, the authority = HCAA /
  Αρχή Πολιτικής Αεροπορίας (ΑΠΑ, not the older ΥΠΑ), third-party liability insurance (EC 785/2004 from 20 kg); L10
  AC54 also scans the tour's UAV chapter and the manuals' UAV chapters (a sentence that says what is excluded is the
  scope statement).
- NOT changed, recorded: L7 (asks of a HELM-holding copilot counted captain-like by refuse_asks_for vs the 126 rule):
  pilot.captain_like (crew captain / single operator / the HELM holder) is used for BOTH the 126 refusal
  (stations.uav_joy_refusal) and refuse_asks_for at the take, and asks target the HELM holder - no case found where
  they disagree; to be reviewed again with the reviewer's exact case.

## 12. Open questions

PROGRESS.md "WAITING FOR OWNER" (each with the steps it blocks and the recommended default). W52-W76 were answered
by the owner on 2026-11-04 (section 13); W52b (CREWED MANEUVER entry, a firmware step) = step WB.

## 13. Owner answers 2026-11-04 (step WA)

All recommended defaults accepted, except where written. Recorded here (CLAUDE.md V follows):
- W54 (loudhailer) CHANGED: the CAPTAIN's press (crew captain / single operator) takes the horn AT ONCE from anyone
  talking (gateway/loudhailer.py floor_captain / _take_over: the displaced station gets ptt_ack "taken" = «Ο
  ΚΥΒΕΡΝΗΤΗΣ ΠΗΡΕ ΤΟ ΜΕΓΑΦΩΝΟ»; audit hail_taken); a sound signal already playing (a few seconds) plays to its end;
  the other stations: first wins, as built. Tests LH20 (LH1 updated).
- W55: the gateway forwards a copilot's command for the HELM holder by its OWN grant while that is newer than
  TELEM2's last HELM change (stations.helm_forward; the AURIX checks HELM again), and gateway test B3 waits for
  TELEM2 to agree before the next command (the old race: a refusal 21 NOT HELM only, never a drive).
- W56 (GPS warm start): (b) a small USB u-blox on the gateway NEAR THE MAGAZINE as the assist receiver (MGA-DBD /
  ephemeris dump, gpsassist "assist_extra") + (a) time + position from telemetry as the fallback; the MGA injection
  through the drones' ArduPilot is a BENCH check (FIRST_TEST.md 6g). Parts: PARTS.md "UAV magazine".
- W57 (hatch): (a) the gateway reads the limit switches through an FT232R BREAKOUT (CTS = OPEN, DSR = CLOSED,
  external pull-ups ~10 kOhm to 3.3 V, switches to GND) - not the TTL-232R cable (no DSR); (b) an 8+ channel relay
  board (relay 8 = the hatch; the 6-channel DroneCAN board can't host it); (c) a SLOW, soft-edged actuator, and the
  sequence closes the hatch only after the switches / the slot USB show the LAST drone gone. Bench: FIRST_TEST.md 6f.
- W59 CHANGED: no boat-speed / mode launch condition (above, 5.3); the captain's YES stays (W74); the climb
  fallback at any speed; the other W59 values as listed (clear 10 m, gap 15 s, 30 + 5 m per slot, 120 m, 1500 m,
  50 m, 10 m/s + 2 m/s, min 10 m, landing margin 30 %, GPS wait 5 s, fallback climb 15 m). Tests UA35, UA16, AC52,
  UV15.
- W62: eWaSR on the UAV video as built (uavai.py); MEASURE it on real aerial clips at the first flights, decide on
  an aerial model later. The boat's FRONT camera AI keeps its priority; UAV AI never gives a speed cap.
- W65: the drawn USV symbol (3.3).
- W66: the Specific category note of section 11 (MANUAL ch. 21 / MANUAL_EL ch. 16) is the legal frame.
- W72: the captain's own HATCH CLOSE after a stop, as built (launch.py stop: the hatch switched by nothing;
  manager.hatch_close_msg: the captain / a single operator / a station on the boat; tests UA16 / UA18).
- W76: demo.py writes the demo boat file demo_run/boat_demo.json (relay 8 = the UAV hatch) and gives it to the
  SHORE station (--boat-file, after the welcome name check); once it holds HELM in STANDBY it pushes the table and
  the simulated AURIX takes it after the onboard joystick CONFIRM - then PREPARE LAUNCH works in the demo (test_boats
  B12).
- W58, W63, W68, W69, W70, W71, W75 (+ W52 (d) loudhailer card, W56 injection, W57 switches / actuator, the manual
  flight's twist axis): BENCH checks before the first flight = FIRST_TEST.md section 6.
- W60, W64, W67, W73, W74: as listed / as built. W53 (station): the OS-native settings questions are now the
  vertical YES / NO layout (groundstation/ynbox.py, test_yesno YN7).


## 14. Step X2 AS BUILT: real ArduPilot SITL + the required parameter file (2026-11-11)

Owner spec J (C:/trygons/private/step_sitl.txt, X2). NO app / hal / AURIX change, gateway/uav/allow.py NOT changed.

- tools/get_sitl.py: pinned ArduCopter SITL into tools/bin/sitl/<platform>/ (never in git), every file size + sha256
  pinned, a different download REFUSED (nothing installed; the owner re-pins with --print-sha after reviewing it);
  --check / --where. gateway/toolpaths.py sitl() returns the binary only while every pin verifies (no unpinned
  override). What works where (measured on the owner's PC, Windows 11, no WSL installed): Windows = ArduPilot's own
  Cygwin build for Mission Planner's simulator (firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/:
  ArduCopter.elf + 10 Cygwin DLLs) runs NATIVELY - 6 copters at once; it is a ROLLING upload, on 2026-11-11
  ArduCopter V4.7.3-beta1 (there is no version-pinned Windows SITL); Linux / WSL 2 = the official 4.7.2 build
  (Copter/stable-4.7.2/SITL_x86_64_linux_gnu/arducopter, pinned, not run here). Default version = the latest stable
  4.x (4.7.2); the drones' real version = TO CONFIRM by the owner (W140).
- docs/uav_params.param = the REQUIRED parameter set (Mission Planner / SITL --defaults format; SAFETY / SAFETY
  if-present / bits / max / min / backstop / ADVICE / PER-SLOT marks). registry.py reads every SAFETY line at
  "Register" (READ ONLY - the gateway never writes a parameter) and refuses a drone that differs, the problem AND a
  station notice ("slot n not launchable: ...") NAME the parameter; ADVICE differences are listed in the record
  ("advice"), never refused; the file missing / unmarked = every drone refused. The parameters registry.py already
  checked by its own rules are checked once (UA41 proves the file and the rules agree). SAFETY parameters NEW in the
  file: GUID_TIMEOUT <= 3 s (a manual setpoint not renewed = stop), ARMING_RUDDER 0 (if present), SERVO17..32 0 (if
  present), MAV1_* telemetry stream rates >= the values (USB port). Measured findings that shaped the file:
  ArduCopter 4.7 sends NO telemetry stream the router does not ask for (and the router never asks) - without MAVn_*
  rates the gateway has no altitude / battery = no battery minimum rule; SERVO17..32 default to -1 (GPIO) and were
  refused; an ENABLED fence refuses to ARM without GPS (also altitude-only) -> FENCE_ENABLE 0 (FENCE_ACTION 0 only
  reports anyway); 4.7 renamed SYSID_THISMAV -> MAV_SYSID (the registry reads the HEARTBEAT sysid).
- gateway/test_sitl.py (SI1-SI14) + tools/run_sitl.py (background --bg, log test_results/sitl/, one Windows job,
  45 min limit): the REAL router process + allow.py + the REAL UavManager (registry, launch, routes, pilot lock,
  battery rule) + simhatch, the boat simulated (STANDBY telemetry, relay board, the captain's YES given by the test),
  the USB cable / ELRS radio = a TCP <-> UDP bridge per link (cut = the breakaway / a lost link), a "bench"
  connection per copter (SERIAL2, sysid 250) for the world only (SIM_* parameters, the true state). Without SITL
  every test is N/A (never PASS); tools/check.sh runs it only when get_sitl.py --check passes. ~9 min.
- MEASURED (ArduCopter V4.7.3-beta1 SITL, Windows):
  - HIGH (blocks every real launch, NOT fixed here): ArduCopter answers MISSION_COUNT with MISSION_REQUEST (id 40) -
    never MISSION_REQUEST_INT, also after the GCS used MISSION_REQUEST_INT - and the router's codec + allow.py
    FROM_DRONE do not know MISSION_REQUEST: NO mission can be uploaded to a real ArduPilot (simdrone sends
    MISSION_REQUEST_INT, so the fast tests never saw it). The fix (FROM_DRONE + MISSION_REQUEST, the codec entry
    (id 40, CRC extra 230), the router answering it with MISSION_ITEM_INT like MISSION_REQUEST_INT) changes allow.py =
    a SERIAL safety review (W110 / W139) - stopped here as the step's hard rule requires. SI3 / SI4 / SI5 / SI14b FAIL
    for this reason; SI6-SI10 / SI14c fly the copters up with GUIDED / ARM / TAKEOFF through the router instead.
  - W60 / 6.1 lost link = hold: a 20 s radio loss -> the same mode at the same altitude (30.0 m), no RTL / LAND
    (SI6; six copters SI14c). W68 in the air: GPS lost -> ArduPilot's EKF failsafe ALT_HOLD after ~28 s, the
    altitude held with the RC receiver ignored (30.0 -> 30.0 m over 20 s), never a landing (SI8).
  - W68 / W59 on the ground (SI13 FAIL): ALT_HOLD + ARM + TAKEOFF param3 = 1 are all ACCEPTED (COMMAND_ACK 0) but
    the copter does NOT climb (ArduCopter's ALT_HOLD take-off needs pilot throttle; the RC receiver is ignored), and
    the no-climb DISARM (never forced) is REFUSED (MAV_RESULT 4, 4 tries) - it stayed armed on the ground for 30+ s.
    The climb fallback does not work and launch.py's no-climb DISARM cannot disarm it: owner decision W138.
  - Manual flying (SI7): GUIDED velocity setpoints 8.0 m/s forward, centred -> 0.1 m/s in 6 s at the same altitude;
    give back = BRAKE then LOITER. Battery minimum (SI10): the manager's NAV_LAND refused by allow.py; the router's
    rule landed it where it was (LAND, down, disarmed), ArduPilot never landed by itself before (BATT_FS_LOW_ACT 0).
  - Allow-list vs ArduPilot (SI9): 17 servo / relay / gripper / winch / mount / parachute / RTL / LAND / force-DISARM /
    RTL-SmartRTL-LAND-mode / parameter-storage / flight-termination commands never left the router, the drone
    unchanged; ArduPilot's other messages (SERVO_OUTPUT_RAW, RC_CHANNELS ...) never decoded (the codec knows only the
    list). The SITL-made .tlog read by pymavlink (SI11: ~2600 packets, in order, no denied command / PARAM_SET); every
    SAFETY parameter unchanged after the flights (SI12).
  - Warm start (SI2): GPS_RTCM_DATA delivered and taken; the forwarding to a real u-blox = still bench only.

## 15. Step UAV-r1 AS BUILT: MISSION_REQUEST in, no launch without GPS, the force DISARM in the slot (2026-11-11)

Owner decisions W139 + W138 (2026-11-11), binding. gateway/uav + tests + docs only: NO app / hal / AURIX change.
allow.py changed = the SERIAL safety review (LOOP RULE 6(3), W110).

- W139 MISSION_REQUEST (id 40, CRC extra 230, the same fields as MISSION_REQUEST_INT; checked against pymavlink in
  UA2): in the codec (mavlink.py) and on allow.FROM_DRONE ONLY - the outbound list allow.TO_DRONE is UNCHANGED
  (UA3 compares it with the exact list). The router answers it in the upload phase exactly like
  MISSION_REQUEST_INT (the same MISSION_ITEM_INT, the same allow.check_out); the read-back stays
  MISSION_REQUEST_LIST / MISSION_REQUEST_INT. SITL also showed: ArduCopter reads a DO item back with frame 0 -
  mission.same accepts frame 0 for DO_CHANGE_SPEED only (never for a position item). simdrone now asks with
  MISSION_REQUEST like ArduCopter (fault req_int = the INT form).
- W138 (a) NO launch without GPS: the ALT_HOLD "climb, then catch up" fallback is REMOVED (launch.py, the
  manager's catchup / catchup_tick, the pilot lock's "catchup" phase, gateway.json fallback_alt_m). The GPS step
  waits gps_wait_s (now 30 s, TO CONFIRM W143) with the hatch OPEN; no 3D fix + EKF position = the sequence STOPS
  ("no GPS fix in 30 s with the hatch open - not launched"), the drone is never armed, the hatch is switched by
  nothing (W72). The slot reports it (gw_status uav slots[].gps_ok / gps_why); with the hatch OPEN a slot without a
  fix is NOT launchable ("no GPS fix - no launch without GPS") and PREPARE is refused 125 naming it. With the hatch
  CLOSED the drone usually has no sky (docs 7: the warm start), so the check is the GPS step after the opening -
  owner to confirm this reading of "the slot reports no GPS" (W143).
- W138 (b) the FORCE DISARM only on the ground in its slot: allow.py lets COMMAND_LONG / _INT ARM_DISARM param1 0
  param2 21196 out ONLY with ctx in_slot; a force ARM and any other param2 stay refused. in_slot is built by the
  ROUTER from its own data (Router.in_slot): a fresh HEARTBEAT on the slot's USB cable (the breakaway cable, W58)
  AND a fresh relative altitude (< 2 s) |alt| < 0.5 m with a known |vz| < 0.3 m/s AND the drone not saying IN_AIR /
  LANDING (fresh EXTENDED_SYS_STATE). No fallback for stale / missing data (unlike on_deck's DISARM fallback).
  launch.py _disarm (no-climb, and a STOP with a drone armed on deck): the normal DISARM first; refused by the drone
  and the drone still on its slot cable = once more FORCED over USB ("disarmed in the hatch - FORCED"); else the
  notice "DISARM not confirmed - check the magazine". Never sent / never let out in flight.
- Tests: UA3 (in_slot cases, force ARM, near-magic, NaN, the exact TO_DRONE list, MISSION_REQUEST inbound only),
  UA16 (no GPS = STOPPED + never armed, in STANDBY and under way; the forced DISARM on the cable; none once the
  cable parted), UA42 (the router: MISSION_REQUEST upload + read-back, force DISARM out only in the slot, refused in
  flight / USB still heard at altitude / off the cable / IN_AIR / moving / stale; mutations: Router.in_slot always
  True and allow.py without its in_slot test are both caught; the DO frame-0 read-back), storm I84 (its OWN in-slot
  record: a force DISARM only then; the in_slot mutation fails I84), SITL SI13 (no GPS = no launch with ArduCopter)
  + SI15a (a copter stuck armed on deck: forced DISARM in the slot) + SI15b (in flight refused, also with the USB
  link heard at 20 m), and SI3 / SI4 / SI5 / SI14b (the mission upload) must now pass.
- Serial safety review of 5f038f0 (opus, tools/api_review.py; C:/trygons/private/reviews/review_5f038f0.txt) - fixed:
  HIGH-1 a force DISARM on telemetry up to 2 s old during a lift-off: Router.in_slot now needs the cable HEARTBEAT
  < 1.5 s, the drone's OWN fresh landed state ON_GROUND or TAKEOFF (missing / IN_AIR / LANDING = not in the slot),
  the newest altitude sample < 1 s and every sample of the last 2 s (ON_GROUND) / 4 s (TAKEOFF) on the ground and
  still (>= 2 samples over >= half the window). MEASURED in SITL: ArduCopter 4.7 streams NO EXTENDED_SYS_STATE by
  itself - the router now asks for it (SET_MESSAGE_INTERVAL 245, 2 Hz, allow-listed) on a registered slot cable
  while it is not heard; and the stuck drone (an accepted ALT_HOLD take-off that does not climb) reports TAKEOFF
  (3), hence the longer still-window for TAKEOFF. And launch.py
  forces only on a REFUSAL - the drone's MAV_RESULT or allow.py's own "refused by the allow-list" (SITL: the stuck
  drone's TAKEOFF state makes the router refuse the normal DISARM) - never a timeout / "replaced" (LOW-1) while the sequence knows the drone cannot
  be lifting off (its TAKEOFF not sent yet, or the no-climb judgement) - a STOP during the climb-out = the normal
  DISARM only (UA16 10b). M3 the force DISARM also needs port "usb" (never over ELRS). MED-1 the launch's
  MISSION_START [0, 0] follows a MISSION_SET_CURRENT(1) (the first item pinned) and docs/uav_params.param requires
  MIS_RESTART 0 (SAFETY if-present: resume at the chosen item). MED-2 storm I84: its own stricter in-slot record
  (HEARTBEAT / GLOBAL_POSITION_INT by name, the window rule), the force DISARM branch must be reached (9 forced
  frames in 60 episodes; the in_slot-always-True mutation = 60 I84 fails). L1 CLAUDE.md's stale U3 / WA lines.
  Not changed: LOW-2 vz has the altitude's timestamp (both come only from GLOBAL_POSITION_INT); the academy uavsim
  has no "no GPS" fault (the lesson never needed the fallback); AC22 / AC54 / AC55 pre-existing since X1.
