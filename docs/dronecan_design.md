# DroneCAN actuators - design (step 11c, 2026-10-27)

Status: DESIGN ONLY. No firmware code in this step. Implementation: steps 11d (CAN driver +
libcanard + messages), 11e (servos + the new CPU1 safety path + the steering model / TRAINING),
11f (relay board over DroneCAN). The build loop STOPS after 11c and after 11f for the owner's
safety review.

Source of truth for the rules: CLAUDE.md section "T. DroneCAN actuators". This document holds the
reasons, the options, the failure analysis and every open item. Where this document says
**TO CONFIRM** the value / fact is NOT known and must not be guessed in code: the hardware flags
stay 0 until the owner confirms. **OPEN DECISION** = a safety question the owner must answer
before 11e / 11f implements it.

Contents
1. What changes on the boat
2. What the ArduPilot parameter file tells us (docs/ardupilot.param)
3. CAN hardware on the KIT_A2G_TC375_LITE
4. Bus, node IDs and messages
5. Node health and command timeouts
6. The new safety path (CPU1 can no longer write the servo pins)
7. Throttle rates (asymmetric) and the slow servo
8. Steering model + TRAINING for position servos without feedback
9. Relay board over DroneCAN
10. Software structure (11d / 11e / 11f), tests and invariants
11. Chart photos (docs/daskalio) - the photo rule
12. TO CONFIRM list
13. OPEN DECISIONS for the owner

---------------------------------------------------------------------------------------------------

## 1. What changes on the boat

The AURIX (KIT_A2G_TC375_LITE) REPLACES the ArduPilot flight controller (Pixhawk) on the boat.

| Function | Before (ArduPilot boat) | Before (this repo, steps 1-11b) | After (11d-11f) |
|----------|-------------------------|---------------------------------|-----------------|
| Throttle | DroneCAN servo node, actuator 1 | GTM PWM P02.7 (TOM0 ch 15) | DroneCAN servo node (actuator 1) |
| Steering | DroneCAN servo node, actuator 2 (REVERSED) | DC motor + Pololu G2, angle from run time | DroneCAN **360 deg position servo** (actuator 2), no position sent back |
| Gear | actuator 3 fixed at MIN (TO CONFIRM) | GTM PWM P02.6 (TOM0 ch 14) | DroneCAN servo node (actuator 3, TO CONFIRM) |
| Relays | DroneCAN hardpoints 0-5 | RS485 Modbus RTU board (step 6) | DroneCAN relay board (hardpoints 0-5), Modbus kept as an option (section 9) |
| GNSS + compass | DroneCAN node 124 (GPS + compass 2) | u-blox MAX-M10S UART + BNO085 | unchanged: the AURIX's own u-blox + BNO085; node 124 = optional backup LATER |
| DP thrusters | - | GTM PWM (TOM0 ch 12 / 13) | unchanged for now |
| E-stop, engine kill, STANDBY button | - | hard-wired | **hard-wired, never over CAN** |

The servo nodes are primitive: a DroneCAN command becomes a PWM pulse, the servo has its own
internal position loop, and NOTHING is sent back except the node's heartbeat (NodeStatus). The
servos are 360 deg position servos on Matek CAN-L431 nodes (AP_Periph firmware - TO CONFIRM the
firmware and its version on each node).

Consequences that drive the whole design:
- No feedback: the AURIX never knows where a servo IS, only what it COMMANDED. Every rule that
  says "idle" or "neutral" must use a MODEL of the servo position with a worst-case speed, never
  the command alone (section 7).
- The servos are slow (measured with ArduPilot driving, its own limits included: steering ~12 s
  lock to lock, throttle ~6 s idle to full). Their own speed is unknown (TO CONFIRM on the bench).
- CPU1 can no longer force idle by writing a pin: the safety path must be redesigned (section 6).
- A servo whose node loses power or commands HOLDS its last position (TO CONFIRM per servo): the
  throttle may stay open. Stopping the engine must therefore NEVER depend on the throttle servo
  (kill cord, engine kill pin, e-stop: all hard-wired).

## 2. What the ArduPilot parameter file tells us (docs/ardupilot.param)

docs/ardupilot.param is the owner's last ArduPilot Rover (boat, FRAME_CLASS 2) parameter dump.
Read-out (only what is in the file; the meaning of each function number is ArduPilot's):

| Parameter | Value | Meaning for us |
|-----------|-------|----------------|
| CAN_P1_BITRATE / CAN_P2_BITRATE | 1000000 | bus 1 Mbit/s |
| CAN_P1_DRIVER 1, CAN_D1_PROTOCOL 1 | DroneCAN on bus 1 | all actuators on bus 1 |
| CAN_D1_UC_NODE | 10 | ArduPilot's own node ID on bus 1 (the AURIX takes it: section 4) |
| CAN_D1_UC_SRV_BM | 1023 | servo channels 1-10 sent over DroneCAN |
| CAN_D1_UC_SRV_RT | 100 | actuator commands at 100 Hz |
| CAN_D1_UC_OPTION | 0 | servos sent as ArrayCommand UNITLESS (not PWM) |
| CAN_D1_UC_ESC_BM | 0 | no ESC commands |
| CAN_D1_UC_RLY_RT | 50 | relay (hardpoint) commands at 50 Hz |
| CAN_D2_* | protocol 1, no servos, no relays | bus 2 unused for actuators |
| SERVO1_FUNCTION 70, MIN 1000, MAX 2000, TRIM 1500, REVERSED 0 | Throttle | actuator 1 = throttle |
| SERVO2_FUNCTION 26, MIN 1000, MAX 2000, TRIM 1499, REVERSED **1** | GroundSteering | actuator 2 = steering, REVERSED |
| SERVO3_FUNCTION 134, MIN 1000, MAX 1850, TRIM 1000 | "SERVOn_MIN" = output fixed at MIN | actuator 3 fixed at 1000 us - gear? **TO CONFIRM** |
| SERVO4_FUNCTION 135, SERVO5_FUNCTION 135, TRIM 1500 | "SERVOn_TRIM" = output fixed at TRIM | actuators 4 / 5 fixed at 1500 us - what they drive: **TO CONFIRM** |
| SERVO6-10_FUNCTION | 0 | disabled |
| RELAY1..6_PIN 1000..1005, FUNCTION 1, DEFAULT 0, INVERTED 0 | DroneCAN hardpoints 0-5 | 6 relays, default OFF |
| GPS1_TYPE 9, GPS1_CAN_NODEID 124 | DroneCAN GPS on node 124 | optional backup later |
| COMPASS_DEV_ID2 97283 (= 0x017C03: bus type 3 = DroneCAN, address 0x7C = 124) | compass 2 on node 124 | optional backup later |
| CRUISE_SPEED 2.5, CRUISE_THROTTLE 52 | 2.5 m/s at 52 % | throttle -> speed table start value (dr.c) |
| ATC_STR_RAT_MAX 17 | 17 deg/s | steering yaw-rate maximum: start value + TRAINING abort limit |
| MOT_SLEWRATE 15 | 15 %/s | ArduPilot's throttle slew: our throttle-UP start value (section 7) |
| ATC_STR_RAT_P 0.12, FF 0.25, ANG_P 1, ACC_MAX 30 | steering rate controller | reference only (our controller is different) |
| TURN_RADIUS 11, WP_RADIUS 5, WP_SPEED 2.5 | navigation | reference only |
| FS_ACTION 5, FS_TIMEOUT 1.5, FS_GCS_TIMEOUT 10 | ArduPilot failsafes | reference only (ours: CLAUDE.md) |

How ArduPilot puts a servo on the wire (to be checked on the bench with a scope on the node's PWM
output, not assumed in code): ArrayCommand, command_type UNITLESS, value =
(pulse_us - 1000) / 500 - 1 clamped to -1..+1, where pulse_us is the channel's output AFTER
MIN / MAX / TRIM / REVERSED. So:
- throttle idle (1000 us) = **-1.0** on the wire, full (2000 us) = +1.0. **A value of 0.0 is HALF
  throttle.** A node that falls back to "0" (or to its trim) on a timeout would open the throttle
  to ~50 %: section 5 makes this a bench check that must pass before the boat moves.
- steering: SERVO2_REVERSED = 1 means ArduPilot flipped the sign before sending. The AURIX must
  put the SAME sign on the wire for the same physical direction: wire = -(our steering, + = right)
  - TO CONFIRM on the bench (command "right", see the outboard turn the boat to starboard).
- gear (actuator 3): 1000 us = -1.0 was a FIXED output; which gear 1000 / 1850 us means, and where
  N / R are, is unknown: **TO CONFIRM**. The AURIX keeps its gear positions configurable
  (CLAUDE.md "Gear servo F / N / R ... configurable positions") in unitless values.
- actuator IDs on the wire: ArduPilot sends actuator_id = servo channel number (1..10); the node
  maps an actuator ID to one of its outputs. Which node / output carries which ID: **TO CONFIRM**
  (read each node's parameters with the DroneCAN GUI Tool).

## 3. CAN hardware on the KIT_A2G_TC375_LITE

The kit (V2) has its own CAN transceiver **TLE9251VSJ** and a CAN connector.

What is CONFIRMED (from the iLLD pin map in this repo,
Libraries/iLLD/TC3xx/Tricore/_PinMap/TC37x/IfxCan_PinMap_TC37x_*.h):
- P20.8 can be CAN0 node 0 TXD (`IfxCan_TXD00_P20_8_OUT`).
- P20.7 can be CAN0 node 0 RXD, input B (`IfxCan_RXD00B_P20_7_IN`).
- P20.6 has no CAN0 node 0 function (it is RXD12A = CAN1 node 2); on the kit it is said to drive
  the transceiver's STB pin as a plain GPIO.
- P20.6 / P20.7 / P20.8 are not used by anything else in hal/hal_pins.h (the USB bridge uses
  P20.0 / P20.3).

What is NOT confirmed (the kit user manual is NOT in the repo; nothing is guessed):
- that the kit wires P20.8 -> TLE9251V TXD, P20.7 <- RXD and P20.6 -> STB: **TO CONFIRM** in the
  KIT_A2G_TC375_LITE user manual / schematic.
- the STB polarity (TLE9251V: STB high = stand-by, low = normal operation per its datasheet - **TO
  CONFIRM** with the datasheet and the kit schematic, incl. any pull resistor on the kit).
- the CAN connector type and pinout (CAN_H, CAN_L, GND, any supply pin): **TO CONFIRM**.
- whether a 120 Ohm termination is fitted on the kit, switchable (jumper / solder bridge) or
  absent: **TO CONFIRM**. The bus needs exactly 120 Ohm at BOTH physical ends (measure ~60 Ohm
  between CAN_H and CAN_L with everything off).
- the transceiver's supply (5 V from the kit's USB / VIN?) and that it works while the kit runs
  from VIN only: **TO CONFIRM**.

hal/hal_pins.h holds these as `HAL_CAN_FITTED 0` + unverified definitions (CAN_TXD / CAN_RXD /
PIN_CAN_STB) marked TO CONFIRM; nothing uses them until 11d, and 11d keeps HAL_CAN_FITTED 0 until
the owner confirms the pins. The iLLD Can driver (Libraries/iLLD/TC3xx/Tricore/Can) is EXCLUDED
in .cproject today: 11d must un-exclude it in all 4 configurations, and
tests/acceptance/check_ads_ifx.py must prove it.

Power for the nodes: the Matek CAN-L431 nodes were powered with 5 V through the Pixhawk's CAN
port. Without the Pixhawk they need **their own fused 5 V supply** (a 12 V -> 5 V converter from
the bank, fuse at the bank end, its own fuse per branch), NOT from the AURIX kit (WIRING.md 3E,
PARTS.md). The servos' own power (the 360 deg servos can draw amps) is a separate rail per the
node / servo wiring: **TO CONFIRM** how each node powers its servo. CAN GND common with the AURIX.

## 4. Bus, node IDs and messages

Bus: 1 Mbit/s classic CAN 2.0B (29-bit IDs, DroneCAN v0), one line, 120 Ohm at both ends,
twisted pair, stubs short. Node IDs are STATIC (ArduPilot was the dynamic node-ID server, so the
nodes' current IDs are unknown; the AURIX will NOT run a dynamic-ID server - a node without a
static ID stays silent and shows as missing, which is the safe failure).

Planned static IDs (**TO CONFIRM** - set on each node with the DroneCAN GUI Tool, then fixed in
the AURIX's compile-time table; never taken from a boat file or the PC, R7):

| Node | Planned ID | Actuator IDs / hardpoints |
|------|------------|---------------------------|
| AURIX | 10 (ArduPilot's CAN_D1_UC_NODE) | - |
| throttle servo node | TO CONFIRM (proposal 20) | actuator 1 |
| steering servo node | TO CONFIRM (proposal 21) | actuator 2 |
| gear servo node | TO CONFIRM (proposal 22) | actuator 3 (TO CONFIRM) |
| (actuators 4 / 5) | TO CONFIRM | fixed TRIM in ArduPilot - not driven by the AURIX until confirmed |
| relay board | TO CONFIRM (proposal 30) | hardpoints 0-5 |
| GPS + compass | 124 (from the parameter file) | not used now |

Whether one node drives several actuators (one Matek node can have several outputs) is unknown:
the design supports both (the AURIX watches the node IDs that carry the actuators it needs; a
shared node lost = all its actuators lost).

Messages (DroneCAN v0 DSDL, standard IDs; the encoding is checked in 11d against the reference
pydronecan, never by eye):

| Message | ID | Direction | Rate | Use |
|---------|----|-----------|------|-----|
| uavcan.protocol.NodeStatus | 341 | AURIX TX + RX | 1 Hz TX; RX from every node | heartbeats (section 5) |
| uavcan.equipment.actuator.ArrayCommand | 1010 | TX | 100 Hz (CAN_D1_UC_SRV_RT) | throttle / steering / gear, UNITLESS; ONE command per transfer (4 bytes + tail = a single CAN frame, no multi-frame CRC; also what CPU1 sends, section 6) |
| uavcan.equipment.hardpoint.Command | 1070 | TX | 50 Hz per hardpoint (CAN_D1_UC_RLY_RT) | relays (section 9) |
| uavcan.equipment.hardpoint.Status | 1071 | RX if the board sends it | - | relay readback (TO CONFIRM that the board sends it) |
| uavcan.protocol.GetNodeInfo | 1 (service) | AURIX = server | on request | so the GUI Tool sees the AURIX |
| uavcan.protocol.dynamic_node_id.Allocation | 1 | RX only | - | a node ASKING for an ID = notice "CAN node without a static ID" (never answered) |
| uavcan.equipment.actuator.Status | 1011 | ignored | - | the servo nodes send no position |
| gnss.Fix2 / ahrs.MagneticFieldStrength2 (node 124) | 1063 / 1002 | later | - | optional backup, NOT in 11d-11f |

Bus load: 3 actuator frames x 100 Hz + 6 hardpoints x 50 Hz + heartbeats ~ 600 frames/s,
~ 0.13 ms per frame at 1 Mbit/s = ~ 8 % - fine.

Command values: the AURIX keeps its own units (throttle 0..100 %, steering -1..+1 + = right,
gear F / N / R) and converts only in the HAL: throttle wire = -1 + 2 x pct / 100 (idle = -1.0),
steering wire = -steer (REVERSED, TO CONFIRM), gear wire = the configured unitless position per
gear (TO CONFIRM). The power-limit clamp stays in app/servo.c (the only throttle writer).

## 5. Node health and command timeouts

The AURIX sees only the nodes' heartbeats (NodeStatus: uptime, health OK / WARNING / ERROR /
CRITICAL, mode OPERATIONAL / INITIALIZATION / MAINTENANCE / SOFTWARE_UPDATE / OFFLINE).

A required actuator node (throttle, steering, gear) is **LOST** when ANY of:
- no NodeStatus for `can_node_lost_ms` (proposal 1500 ms; AP_Periph sends 1 Hz, DroneCAN's own
  offline timeout is 3 s - TO CONFIRM the real rate on the bench, then the limit = 1.5 x period);
- mode is not OPERATIONAL;
- health is ERROR or CRITICAL (WARNING = notice only: OPEN DECISION OD-6);
- its uptime went BACKWARDS (the node rebooted: its outputs went through its own boot state, which
  we do not know) - latched until the node has been OPERATIONAL again for 2 s AND the boat is in
  STANDBY (acknowledge);
- the CAN controller is bus-off / error-passive for longer than 100 ms (all nodes lost at once).

Reaction (owner rule: node silent / not OPERATIONAL = alarm + STANDBY): alarm SB_ALM2_CAN_NODE
(WARNING, NOT an advisory: lowers the power limit like every alarm) + STANDBY (WHY_CAN_NODE) with
the normal STANDBY order (idle at once, neutral when the modelled throttle is idle, steering per
the safe-stop rules), and the mode may not leave STANDBY while a required node is lost
(neutral-start interlock: "all required nodes OPERATIONAL" is added). Per lost node:
- throttle node lost: the AURIX can't command idle - the node's own timeout must (below). Gear:
  wait the worst-case throttle travel time (`thr_travel_s`, 6 s) before commanding neutral (the
  throttle may still be coming down). UNCREWED: OPEN DECISION OD-3 (kill the engine at once?).
  CREWED: alarm, the crew stops the engine (kill cord) if needed - no automatic kill (CLAUDE.md B).
- gear node lost: idle at once; the gear is unknown (alarm); steering keeps holding / steering per
  the safe-stop rules until the modelled throttle is idle.
- steering node lost: idle at once, neutral after the modelled idle; the steering holds where it
  is (node timeout = hold); alarm.
- relay board lost: RELAY BOARD SILENT (advisory, unchanged from step 6 - section 9).

The AURIX sends every actuator command at 100 Hz from a 10 ms core-0 timer task (the 20 Hz control
step publishes the demands; the task only encodes and queues - never blocking). The node-side
**command timeout** (no command for `node_cmd_timeout_ms`, proposal 300 ms, TO CONFIRM what the
node firmware offers) must be set on every node and BENCH-TESTED before the boat moves:

| Node | On command timeout | Why |
|------|--------------------|-----|
| throttle | **IDLE** (-1.0 = 1000 us, the node's configured failsafe pulse) | the only safe throttle; must NOT be "0" (= ~50 %) or "hold" |
| gear | **HOLD** (keep the last position) | see below |
| steering | **HOLD** | a position servo that holds keeps the boat on its course; a jump to centre at speed is a turn the AURIX did not ask for |

Gear: HOLD, not NEUTRAL. Reasons:
1. Gear change only at idle (CLAUDE.md safety rule). A command timeout hits all nodes at the same
   moment (AURIX reset, bus fault, cable). The throttle servo then needs up to `thr_travel_s`
   (6 s worst case) to reach idle; a gear node that went to NEUTRAL by itself would shift under
   throttle at once - the one thing the gear rules forbid (shift cable load, gearbox damage, a
   shift that does not complete).
2. With HOLD the AURIX (when it comes back) or the crew decides when to shift; the AURIX's boot
   order (section 6.4) shifts to neutral only after the throttle travel time.
3. A node that can only do "hold" or "neutral after a delay" may be set to neutral after a delay
   >= thr_travel_s + 2 s - acceptable ONLY if the owner decides so (OPEN DECISION OD-5).
4. Propulsion is stopped by the throttle going idle and, if needed, by the engine stop (kill
   cord / kill pin / e-stop) - never by the gear alone.

Bench test (FIRST_TEST.md, engine OFF, done with the DroneCAN GUI Tool on a laptop connected
directly to the bus - the AURIX never commands throttle above idle in BENCH): for each node, set
its timeout + failsafe, command a position, unplug / stop the command source, check with a scope /
by eye that throttle goes to idle (and NOT to mid), gear holds, steering holds, within the
timeout. Record the measured times in FIRST_TEST.md. Until this passes: no engine running with
the DroneCAN servos (hardware flag HAL_CAN_FITTED stays 0 and the servos are not "required").

## 6. The new safety path (CPU1 can no longer write the servo pins)

### 6.1 What stays

- CPU1 runs the independent safety monitor (throttle vs limit, gear change with the throttle up,
  core-0 heartbeat 200 ms, generator / thruster rules). Its HAL latch, the trip reasons and the
  big-boat variant are unchanged.
- E-stop contact, engine kill pin, STANDBY button: hard-wired GPIO, never over CAN.
- DP thrusters and their GTM safe state (hal_safe_outputs) unchanged for now.
- Stopping the engine NEVER depends on the throttle servo.

### 6.2 Options

| | Option | Normal traffic | Safe state on a trip | Core 0 frozen | Main weakness |
|-|--------|----------------|----------------------|---------------|---------------|
| A | CPU1 owns all of CAN | CPU1 sends what core 0 publishes | CPU1 sends the safe commands | CPU1 keeps sending (safe) | CPU1 grows (libcanard, RX, 100 Hz task) - against "CPU1: the safety monitor, nothing else" and its 60 % RAM limit; a CPU1 bug stops ALL actuation |
| B | core 0 owns CAN; CPU1 has a RESERVED dedicated TX buffer on the same MCMCAN node + may cancel core 0's TX requests | core 0 (libcanard) | CPU1 latches, cancels core 0's pending frames, sends pre-encoded safe frames (single-frame ArrayCommand) from its own buffer at 50 Hz | CPU1 heartbeat trip -> CPU1 sends (if the chip was not reset) | both cores touch the MCMCAN registers (only write-1 bit registers + its own buffer; access protection to check); bus-off kills both |
| C | as B, but CPU1 on a SECOND MCMCAN node + a second transceiver on the same bus | core 0 | CPU1 from its own node | as B | extra hardware + a second node ID; still one bus (cable / bus-off) |
| D | no CAN path for CPU1; only hardware (safety relay cuts servo power, engine kill) | core 0 | relay opens | relay opens | a position servo without power HOLDS - the throttle stays where it is; only the engine stop helps |

### 6.3 Recommendation: B (+ the hardware backstops of D)

- Core 0 owns the MCMCAN module: init, the RX FIFO, libcanard, all normal TX (TX FIFO), node
  health. Nothing else on core 0 changes (app/servo.c stays the only throttle writer, now through
  hal_set_throttle -> the CAN queue).
- One dedicated TX buffer (proposal: buffer 0, highest priority by CAN ID among ours) is RESERVED
  for CPU1; core 0 never writes it. CPU1 holds three PRE-ENCODED single-frame ArrayCommand
  transfers (throttle idle, steering = hold, gear = hold / neutral), its own small encoder for
  the tail byte (transfer ID) and the float16 value (steering / gear hold come from core 0's last
  published commands), no libcanard, no RX on CPU1 -> CPU1's RAM grows by < 200 B
  (tools/mapreport.py keeps the 60 % rule).
- On a trip (any MON_TRIP_*), CPU1: (1) sets the HAL latch FIRST (as today), (2) cancels core 0's
  pending actuator frames (TXBCR, write-1 bits), (3) sends its safe frames every 20 ms from its
  buffer: throttle IDLE at once; steering = the last steering command core 0 published (hold);
  gear = HOLD the last published gear position until `thr_travel_s` (6 s worst case) has passed,
  then NEUTRAL; (4) keeps the safety relay open / buzzer on (as today).
- Every core-0 CAN queue call checks the latch BEFORE and AFTER writing (the existing pattern:
  "every core-0 output call re-checks the latch after its write") and cancels its own frame if
  the latch was set in between - so core 0 can't overwrite CPU1's safe frames.
- Transfer IDs: CPU1 sends with the AURIX's node ID (10) and continues core 0's ArrayCommand
  transfer-ID counter (core 0 publishes it in g_mon; CPU1 starts at the last value + 1) so a node
  never drops CPU1's frames as duplicates. TO CONFIRM on the bench with the real nodes.
- NEW CPU1 check (MON_TRIP_CAN_TX): CPU1 reads the MCMCAN TX event / a core-0 TX counter: no
  actuator frame went out for 100 ms while core 0 is "running" -> trip (core 0's loop alive but its
  CAN path stuck).
- CPU1 also keeps its OWN throttle position envelope (the last command decayed at the worst-case
  servo speed, 100 % / thr_travel_s): the gear check "gear moved with the throttle up" uses that
  envelope, not the command (a slow servo is still open after the command went to idle).
- Shared data (latch, transfer ID, published commands) stays in core 0's DSPR with SHARED_BEGIN /
  SHARED_END (app/memplace.h); MCMCAN message RAM is in the peripheral (no LMU, no cache).
- To check on the board in 11e (TO CONFIRM): MCMCAN access protection (ACCEN) lets CPU1 write the
  node's TX registers; a dedicated buffer + TX FIFO mix on one node works as the iLLD describes;
  CPU1's frames win arbitration (lowest CAN ID = priority field of the DroneCAN ID).

### 6.4 Failure analysis

| Failure | What happens | Backstop |
|---------|--------------|----------|
| core 0 control loop frozen | its watchdog (100 ms) resets the chip -> ALL CAN silent during reset + boot (~0.3 s + driver start) -> node timeouts: throttle IDLE, gear HOLD, steering HOLD; at boot (below) | node timeout bench-tested; throttle return spring; kill cord / kill pin / e-stop |
| core 0 alive but wrong commands (above the limit, gear with the throttle up) | CPU1 trip -> latch -> CPU1 safe frames | node timeouts if CPU1's frames don't arrive |
| core 0 loop alive, its CAN path stuck | MON_TRIP_CAN_TX after 100 ms -> CPU1 safe frames | node timeouts |
| CPU1 frozen | its own watchdog resets the chip (as today) | node timeouts |
| CAN controller bus-off / transceiver dead / STB stuck in stand-by | nobody can send -> node timeouts (throttle idle, gear + steering hold); AURIX: all nodes lost -> alarm + STANDBY | engine stop is hard-wired |
| CAN cable cut / short | as bus-off for the nodes behind the cut | as above |
| one node loses power | that servo HOLDS where it is (TO CONFIRM per servo: a position servo without power may also be back-driven); node lost -> alarm + STANDBY | throttle return spring (if fitted and strong enough); engine stop |
| node firmware falls back to "0" on timeout | THROTTLE ~50 %: NOT acceptable -> the bench test of section 5 must pass first | engine stop |
| node reboot (brown-out) | outputs go through the node's boot state; uptime goes back -> node lost (latched) -> STANDBY | |
| chip reset (watchdog / SMU / software / power-on) | boot order: (1) throttle IDLE commanded as the very first CAN frame; (2) gear: NO gear frame (node HOLDS) until thr_travel_s has passed since the boot, then NEUTRAL; (3) steering: after a warm reset re-send the last steering command kept in reset-surviving RAM (like the kill latch) = no jump; after a power-on command centre (the boat is normally at the dock: OPEN DECISION OD-7); (4) the neutral-start interlock as today + all required nodes OPERATIONAL | |
| e-stop pressed | software: STANDBY + idle at once (CAN); hardware: the e-stop must stop the ENGINE through the kill circuit (OPEN DECISION OD-1), and cut the thruster contactor. If the e-stop also cuts the servo power, the throttle servo freezes where it is - so the e-stop must NOT rely on the throttle servo | hard-wired |
| safety relay (P10.5) opens on a trip | TODAY it cuts servo power. With position servos that freezes the throttle -> OPEN DECISION OD-2: wire it into the engine kill circuit instead (stop the engine on a trip) or leave the servos powered | |

## 7. Throttle rates (asymmetric) and the slow servo

Owner decision (2026-10-27): throttle UP is gentle, throttle DOWN is much faster, emergencies are
immediate.

| Parameter | Start value | Replaces | Used for |
|-----------|-------------|----------|----------|
| `rate_up_pct_s` (parameter thr_up_pc_s) | 15 %/s (ArduPilot MOT_SLEWRATE) | rate_up 15 %/s (same value) | every throttle rise (incl. generator steps) |
| `rate_down_pct_s` (parameter thr_dn_pc_s) | 50 %/s | rate_down 40 %/s | normal driving, lowered power limit, slamming step, planned arrival stop (WHY_ARRIVED), generator stop |
| (none) | idle AT ONCE, no rate limit | rate_stop_pct_s / parameter stop_rate_pc_s 100 %/s | STANDBY (button / any mode), e-stop, link-loss failsafe (REMOTE / MANEUVER stale data, UNCREWED lost link), node loss, every fault / alarm STANDBY, monitor trip, generator GEAR FAULT |

The servo's own speed then limits the physical move (worst case ~6 s idle <-> full: `thr_travel_s`
= 6 s; steering `steer_travel_s` = 12 s lock to lock). These are the WORST CASE for safety and the
sim start values until the bench (TO CONFIRM) / TRAINING measure them; the sim must ALSO run with
fast servos (e.g. 3 s steering / 2 s throttle) so no rule depends on slowness.

Throttle model (new, app/servo.c side, 11e): `thr_est` = the servo position estimate = the command
followed at the servo speed (1 / thr_travel_s, 100 %/6 s). "Idle" for every rule that needs it
(gear change, "idle + neutral FIRST, then steering off", ENGINE KILL only after idle + neutral,
STARTER only at idle, generator, DP entry, remote restart) means **thr_est at idle**, i.e. the
command at idle AND at least (last command / 100) x thr_travel_s since it went there. The gear
state machine's "throttle idle -> wait -> neutral" uses thr_est. thr_travel_s used for these
safety waits is NEVER shorter than the configured worst case unless the owner signs a measured
value (OPEN DECISION OD-4); a TRAINING / bench value can make it longer, never shorter on its own.

CLAUDE.md wording that this replaces (implemented in 11e; until then the code keeps today's
numbers): the Safety-rules line "STANDBY has its own, faster throttle closing rate
(throttle.rate_stop_pct_s, parameter stop_rate_pc_s, default 100 %/s = idle within 1 s); normal
driving and a planned arrival stop (WHY_ARRIVED) keep rate_down (40 %/s)" becomes the table above
(CLAUDE.md section T + the Safety-rules line).

Tests that change in 11e (today's checks use rate_down 40 / rate_stop 100 or "idle within ~1 s"):
- sim P2 "STANDBY at full throttle: idle within about 1 s" -> idle COMMANDED in the same step,
  physical idle (servo model) within thr_travel_s; neutral only after that.
- sim P2 "reverse power limited, throttle rate-limited" -> rate_down 50, rate_up 15.
- sim P3 R1 "STANDBY in MANUAL: the stick steers until idle" -> until the modelled idle (up to 6 s).
- sim U6 / U7 (USB lever pickup, power raise at full lever) -> the new rate bounds.
- sim SL2 (slamming STEP) and the servo_clamp tests ("limit drop at rate_down") -> 50 %/s.
- sim GN* generator stop paths (normal rate_down; GEAR FAULT now at once), ER* (STARTER / restart
  idle conditions on thr_est), LL* (kill only after the modelled idle), DP entry (thr_est idle),
  FT1-FT5 (FIRST TEST: throttle idle every step - behaviour unchanged).
- link.c parameter stop_rate_pc_s retired (groundstation config page, proto.py, MANUAL / MANUAL_EL),
  new parameters thr_up_pc_s / thr_dn_pc_s (limits: up 2..30 %/s, down 20..200 %/s - TO CONFIRM).
- storm I1 (closing envelope at the new rate_down), I3 (gear vs the SIMULATED servo position, not
  the command), I4 (bound = standby throttle x thr_travel_s / 100 + idle_wait + margin), I9, I27,
  I33 (generator envelope), I37.
- New storm invariants (11e):
  - I47 no software limit ever slows a move toward idle in an emergency (STANDBY / e-stop /
    link-loss / node-loss / trip: idle commanded in that same step); a normal decrease is never
    slower than rate_down.
  - I48 the throttle command never rises faster than rate_up (and the generator's steps stay
    inside it).
  - I49 a gear change is only commanded while the SIMULATED throttle servo is at idle (fast and
    slow servo runs).
  - I50 a required node lost / not OPERATIONAL / rebooted -> alarm + STANDBY within
    can_node_lost_ms + one step; never out of STANDBY while one is lost.
  - I51 commands stopped (reset / bus fault) -> the simulated nodes go throttle idle, gear hold,
    steering hold within their timeout; after the boot no gear frame before thr_travel_s.
  - I52 monitor trip (also with core 0 frozen in the sim): CPU1's safe frames within 20 ms,
    core 0's actuator frames stop (latched).
  - I53 the steering adaptation never leaves the TRAINING bounds (section 8) and falls back to the
    stored values on doubt.

## 8. Steering model + TRAINING for position servos without feedback

Why ArduPilot steered badly: it could not learn the rudder (servo) speed, the left-right gap
(backlash in the linkage), or adapt by itself. The AURIX already does this for the DC motor
(calib.c / training.c, motor-time units); the position servo needs a new model.

### 8.1 Model (all in servo units: -1..+1 = full left..full right)

1. command `u` (what the AURIX sends, after backlash compensation);
2. servo position `p`: follows u at the servo speed `r_s` (units/s; worst case 2 / 12 s =
   0.167 /s until measured);
3. outboard angle `a`: follows p through a dead band (backlash) of width `b_lr` (reversing from
   left to right) / `b_rl` (right to left) - a "play" element;
4. boat: yaw rate `w` with first-order response `tau dw/dt = K(V) (a - a0) - w`, K proportional
   to the speed through the water (SOG until a paddlewheel exists), a0 = trim offset.

The AURIX runs the same model forward every step (`p_est`, `a_est`) - it never knows p or a.

### 8.2 TRAINING (from STANDBY by the PC, as today; CREWED; open water; steady speed)

Phases (each aborts on: any joystick deflection -> MANUAL, STANDBY, yaw rate > 17 deg/s
(ATC_STR_RAT_MAX), roll / slamming alarm, GPS / compass / gyro doubt, a required node lost; abort =
servo back to the centre estimate at the measured / worst-case speed, then STANDBY rules):
1. CENTRE: u = 0 for 20 s -> a0 (trim yaw rate) and the gyro bias.
2. SMALL STEPS +-0.08 (servo time short): K and tau.
3. LARGE STEPS +-0.3 (limited so the yaw rate stays < 17 deg/s): the slope of the yaw-rate rise
   while the servo travels gives r_s (separately for each direction).
4. REVERSAL STAIRCASE: from a held position, reverse by 0.005, 0.01, 0.015 ... until the yaw rate
   answers (> 3 x noise within 2 tau): the smallest answered reversal = b_lr / b_rl.
5. VERIFY: a mixed sequence, model fit R^2 and the agreement of phases = confidence.
Limits: servo excursion <= +-0.5 (TO CONFIRM), never faster than the worst-case speed before
phase 3 measured it. A result with confidence < 0.6 is shown but NOT used (stored values / defaults
stay).

### 8.3 Use while driving

- Command rate limit = the measured r_s (never faster than the servo: p_est stays true).
- Backlash compensation on every direction change: u = target + sign(direction) x b/2 (the play
  inverse), never more than the TRAINING b (owner rule: never larger steps than TRAINING found).
- Heading controller (autopilot.c) works on a_est; HDG LOCK / AUTO / DP heading hold unchanged
  otherwise.

### 8.4 Slow adaptation (RAM only; HDG LOCK / AUTO steady straight running)

- Only while: speed within +-20 % of the TRAINING speed band, no turn > 5 deg in 30 s, no alarm
  (not even an advisory of slamming), gyro + compass healthy, GPS trusted.
- K: recursive estimate with a long memory (time constant >= 5 min), clamped to 0.7..1.3 x the
  TRAINING K; tau the same band.
- Backlash: may SHRINK (if reversals answer earlier) down to 0.5 x the TRAINING b, never grow above
  it; r_s is never assumed faster than the TRAINING r_s.
- Doubt (model residual above the TRAINING residual x 3 for 10 s, any limit hit, a sensor fault,
  a node lost) -> back to the stored values at once; notice STEERING MODEL RESET.
- Adapted values are NEVER saved by themselves (OPEN DECISION OD-8: offer "save adapted values" to
  the captain in STANDBY?). Every reset starts from the TRAINING values.

### 8.5 Storage, screens, tests

- NV slot 5 record version 2 (calib_data_t v2: r_s left / right, b_lr / b_rl, K at the reference
  speed, tau, a0, gyro bias, fit R^2, confidence, TRAINING speed, date / boot number, servo full
  angle entered by the user). A v1 record (DC-motor units) is NOT converted: defaults + notice NV
  DATA RESET + "TRAINING needed". Defaults before TRAINING: r_s worst case, b = 0 (no compensation
  = never an overshoot), a conservative K.
- Ground station: TRAINING page shows the stored values + date + confidence, the live adapted
  values (+ "stored" / "adapted"), and why the last adaptation was reset; TELEM2 group Q (steering
  model); log event 30 STEERMODEL.
- Sim (11e): a servo model with rate limit + backlash (+ fast / slow variants), boat response with
  K(V) and tau; tests SM1-SM8: TRAINING finds r_s / b / K within 15 % on the model; abort paths;
  compensation never above b; adaptation bounds; doubt -> fallback; v1 record -> defaults; slow
  and fast servo; node lost during TRAINING.
- Big-boat variant (rudder feedback): unchanged.

## 9. Relay board over DroneCAN

The relay board becomes a DroneCAN node: hardpoints 0-5 (RELAY1..6_PIN 1000..1005 in ArduPilot),
uavcan.equipment.hardpoint.Command (hardpoint_id, command 0 / 1) at 50 Hz per hardpoint. Board
model: **TO CONFIRM**.

EVERY relay rule stays (CLAUDE.md M, N, owner clarifications 2026-10-26, steps 7b / 11b):
app/relays.c the only writer; the table from the boat file through PROTO_T_RELAY_CFG; the table in
NV slot 7, a different table only in STANDBY + CONFIRM; power_up only ON, only with the first
table after the AURIX's OWN power-on reset, never after a watchdog / software reset or a
reconnect; never switch a relay OFF by itself; engine group never automatic; interlocked pairs;
momentary <= max_s (<= 5 s); timed; STARTER only by the remote-restart rules (role STARTER,
hold-to-crank, idle + neutral confirmed - now thr_est idle - e-stop released, <= 5 s per hold,
cool-down); SHUTOFF ON always allowed; the kill latch on its own pin survives a watchdog reset;
RELAY BOARD SILENT = advisory; no relay in any control path (I36-I38 kept).

What DroneCAN changes and how the rules are kept:
- No board-timed pulse in hardpoint.Command: momentary / STARTER pulses are timed by the AURIX
  (ON frames while allowed, then OFF). The backstop for a STARTER that must stop when the AURIX
  stops sending is the BOARD's own command timeout (<= 300 ms -> OFF for that hardpoint). If the
  board can't do that, the STARTER may NOT be on this board (a hardware time-limit relay instead):
  requirement R-3 below.
- Readback: only if the board sends hardpoint.Status (TO CONFIRM); otherwise the screens show the
  COMMANDED state, marked "no readback".
- After an AURIX watchdog / software reset: the commanded states are kept in reset-surviving RAM
  (like the kill latch) and re-sent unchanged. A board that HELD its states during the reset sees no
  change (rule kept: nothing switches). A board that DROPPED them on its timeout would be switched
  back ON = an automatic switch after a reset - conflict with the rules -> requirement R-2.

Requirements for the DroneCAN relay board (all TO CONFIRM with the board's documentation / bench):
- R-1 static node ID, NodeStatus at >= 1 Hz.
- R-2 non-momentary hardpoints HOLD their state on a command timeout (so an AURIX reset changes
  nothing), and power up OFF.
- R-3 a per-hardpoint (or board-wide) command timeout for the momentary / STARTER channel(s):
  OFF after <= 300 ms without a command - or the STARTER gets a hardware time limit.
- R-4 hardpoint.Status readback (nice to have).
R-2 and R-3 together usually mean two different behaviours on one board; if the board offers only
one, OPEN DECISION OD-9.

Does the RS485 Modbus driver stay? **Recommendation: YES, as a build option** (hal_pins.h
RELAY_BUS = MODBUS / DRONECAN, both behind the same HAL API hal_relay_set / pulse / read / ok, so
app/relays.c and every relay test stay as they are). Reasons: (1) the DroneCAN board is not chosen
and may fail R-2 / R-3, while a Modbus board with board-timed pulses ("flash on") + coil readback
meets every rule today; (2) tc_modbus.c is written and PC-tested (RB1-RB14), keeping it costs no
RAM when not selected; (3) the relay board is NOT a safety path, so a second bus for it adds no
safety risk; (4) removing it later is one commit. Default after 11f: DRONECAN if the chosen board
meets R-1..R-3, else MODBUS - the owner decides at the 11f review.

## 10. Software structure (11d / 11e / 11f), tests and invariants

11d - CAN driver + DroneCAN (no actuator use yet):
- hal/tc_can.c (board only): MCMCAN node 0 via iLLD IfxCan (un-excluded), 1 Mbit/s, RX FIFO 0,
  TX FIFO for core 0, dedicated TX buffer 0 reserved for CPU1, bus-off / error counters, STB pin;
  no blocking waits (a full TX FIFO = drop + count); HAL_CAN_FITTED 0 until the pins are confirmed.
- hal/libcanard/ = DroneCAN libcanard v0 (MIT, vendored with its LICENSE, static memory pool, no
  malloc); hal/dronecan.c (pure C, also in the sim): encode / decode of the messages of section 4,
  node table + health (section 5). NOT a new top-level folder (hal/ is built by ADS; nothing to
  exclude); tools/check.sh: host tests tests/hal_pc/dronecan_check.py (frames compared with
  pydronecan) like fat32_check.py / nv_check.py.
- sim: a simulated bus in hal_sim.c (the real hal/dronecan.c encodes; simulated nodes decode,
  with heartbeats, health / mode faults, reboot, silence, bus-off).
- telemetry TELEM2 group P (node table: present, health, mode, uptime, restarts; bus errors), log
  event 29 CAN, alarms SB_ALM2_CAN_NODE (0x20, WARNING) + SB_ALM2_CAN_BUS (0x40, WARNING), notice
  "CAN node without a static ID"; ground station: node table on the Bench / system page.
11e - servos + CPU1 path + steering model: hal_set_throttle / hal_set_gear -> DroneCAN; new
  hal_set_steer_pos (the small-boat steering HAL changes from motor on / off to a position;
  AP_STEER_OUTBOARD keeps the outboard logic, the big boat unchanged); servo models in the sim
  (rate limit, backlash, fast / slow); section 6 CPU1 path + MON_TRIP_CAN_TX; section 7 rates +
  thr_est; section 8 TRAINING / model / adaptation; storm I47-I53 + the changed invariants and
  tests listed in section 7; BENCH: node table + steering steps (no throttle above idle).
11f - relay board over DroneCAN (section 9), RELAY_BUS option, every RB / ER / I36-I38 test on
  both buses.

## 11. Chart photos (docs/daskalio) - the photo rule

docs/daskalio/ = photos of depth charts of the Daskalio area (the academy test area). RULE: these
photos are for the academy test area, the demo map and the 3D sim ONLY. A photo is NOT a
navigation chart: it is NEVER used for the shallow-water check (gateway shallow.py / chart.py),
the collision / route checks or any other safety check - unless it has been georeferenced AND
checked against an official chart, and that is a separate, reviewed step. The safety checks use
only charts/ (tools/chartprep.py data) with "no data = shallow". (The folder was not found on this
PC in step 11c: it is committed when the owner copies the photos in.)

## 12. TO CONFIRM list (nothing below is guessed in code; hardware flags stay 0)

| # | Item | How to confirm |
|---|------|----------------|
| C1 | Servo node model + firmware (Matek CAN-L431, AP_Periph version) on each actuator | DroneCAN GUI Tool: GetNodeInfo |
| C2 | How many nodes, which node / output carries actuator 1 / 2 / 3 (4 / 5?) | GUI Tool: node parameters |
| C3 | Static node IDs (proposal AURIX 10, throttle 20, steering 21, gear 22, relays 30) | set with the GUI Tool, then fixed in the AURIX table |
| C4 | Actuator 3 = gear? which unitless value = F / N / R (ArduPilot fixed it at 1000 us; MAX 1850) | bench, engine off |
| C5 | Actuators 4 / 5 (fixed TRIM 1500 us in ArduPilot): what they drive | owner / wiring |
| C6 | Steering wire sign (REVERSED in ArduPilot): AURIX "right" = negative wire value? | bench: command right, check the boat turns to starboard |
| C7 | Unitless mapping on the node: -1 = 1000 us, +1 = 2000 us; throttle idle = -1.0 | scope on the node PWM |
| C8 | Node command timeout + failsafe options and values (throttle -> idle, NOT 0 / mid; gear hold; steering hold) | node parameters + bench test (section 5) |
| C9 | NodeStatus rate of each node (-> can_node_lost_ms) | GUI Tool bus monitor |
| C10 | Servo speeds: steering lock to lock, throttle idle to full (worst case now 12 s / 6 s) | bench stopwatch / video, then TRAINING |
| C11 | Servo behaviour without power / without pulses (hold, limp, back-drivable?) | bench |
| C12 | Kit CAN pins: P20.8 TXD / P20.7 RXD / P20.6 STB to the TLE9251V, STB polarity | KIT_A2G_TC375_LITE user manual / schematic, TLE9251V datasheet |
| C13 | Kit CAN connector type + pinout | kit user manual |
| C14 | 120 Ohm termination on the kit (fitted / jumper / none) | kit manual + meter (60 Ohm between H and L with both ends terminated) |
| C15 | Transceiver supply on the kit when powered from VIN | kit manual |
| C16 | 5 V supply for the nodes (converter rating, fuses) and the servos' own power | node / servo data, current measured |
| C17 | Relay board model, node ID, hardpoints 0-5, timeout / hold behaviour (R-1..R-4), readback | board documentation + bench |
| C18 | MCMCAN: CPU1 may write the TX registers (ACCEN), dedicated buffer + FIFO on one node, CPU1 frame priority | on the board in 11e |
| C19 | Transfer-ID hand-over core 0 -> CPU1 accepted by the nodes | bench in 11e |
| C20 | ThinkPad L390 Yoga gateway details (step 12): RJ45 or USB-Ethernet, USB-C PD input power, lid-closed operation, battery charge limit tool | step 12 |
| C21 | Throttle rate parameter limits (up 2..30 %/s, down 20..200 %/s) | owner |

## 13. OPEN DECISIONS for the owner (answer before 11e / 11f)

| # | Question | Recommendation |
|---|----------|----------------|
| OD-1 | Must the e-stop stop the ENGINE through the hard-wired kill circuit (also in CREWED)? With position servos the throttle may stay open when servo power is cut. | YES: a third NC contact / relay of the e-stop in the kill circuit (with the engine dealer). This is hard-wired and separate from the software ENGINE KILL rule ("never in CREWED") |
| OD-2 | Safety relay P10.5 (opens on a monitor trip): today it cuts the servo power - with position servos that freezes the throttle. Re-wire it? | Do NOT cut the servo node power with it; wire it into the engine kill circuit (a trip stops the engine) OR leave it unused - owner + engine dealer |
| OD-3 | Throttle node lost in UNCREWED: kill the engine at once (nobody on board, the throttle may be stuck open)? | YES in UNCREWED (after the gear rule: no shift), never in CREWED (alarm + the crew) |
| OD-4 | May a measured (bench / TRAINING) throttle travel time SHORTEN the safety waits (gear after idle, kill after idle) below 6 s? | Only a value the owner signs (parameter set in STANDBY + CONFIRM), never TRAINING by itself |
| OD-5 | Gear node timeout: HOLD (recommended) or NEUTRAL after a delay >= thr_travel_s + 2 s? | HOLD |
| OD-6 | Node health WARNING: notice only, or alarm + STANDBY like ERROR? | notice only (owner rule names "silent / not OPERATIONAL"; ERROR / CRITICAL = lost) |
| OD-7 | After a POWER-ON (not a warm reset) the steering position is unknown: command centre at boot? | YES (normally at the dock); after a warm reset re-send the last command (no jump) |
| OD-8 | Save the slowly adapted steering values? | No (RAM only); maybe later "save adapted values" in STANDBY by the captain |
| OD-9 | The relay board offers only one timeout behaviour (all hold OR all off) | keep the STARTER on a hardware time-limit relay, or keep the Modbus board (section 9) |
| OD-10 | STANDBY "steering off" for a position servo = HOLD the position (servo powered, outboard can't be moved by hand) or make the servo limp (no pulses: may still hold)? | HOLD (the boat keeps its course while stopping); a manual tiller needs the servo unpowered / disconnected - owner |
| OD-11 | Throttle return spring with a 360 deg position servo: can the spring back-drive it when unpowered? | TO CONFIRM on the bench; if not, the engine stop is the only backstop |
| OD-12 | Does any of this apply to the big-boat variant? | No (unchanged, rudder feedback, own drivers) |
