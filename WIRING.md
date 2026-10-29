# Wiring: joysticks, control panel, sensors, actuators and links (KIT_A2G_TC375_LITE)

Every pin the software uses is also in hal/hal_pins.h (one table, the same
names). Stage 1 (first steering test) parts: sections 2, 3, 4, 9 and 11.

Source: *AURIX TC375 lite Kit V2 user manual*, Figure 7 "Mapping of Arduino
Functions to AURIX Pin Functions". **Always check the pin labels printed on
the board before you connect anything.**

> **3.3 V only.** The board's Arduino pins are 3.3 V logic and the analog
> inputs must never be higher than VAREF = 3.3 V. Do NOT power the joystick
> from 5 V: this can damage the TC375.

## 1. Joystick (2 potentiometers + 3 push buttons)

| Joystick wire          | Board pin (Arduino name) | Connector / pin | TC375 pin           |
|------------------------|--------------------------|-----------------|---------------------|
| X pot, wiper (middle)  | A0 (ADC0)                | X301 pin 1      | P40.9 = AN39 (VADC G4.7) |
| Y pot, wiper (middle)  | A1 (ADC1)                | X301 pin 2      | P40.8 = AN38 (VADC G4.6) |
| Pot "+" ends (both)    | 3V3 (through 1 k, see below) | X302 pin 4  | +3.3 V              |
| Pot "-" ends (both)    | GND (through 1 k, see below) | X302 pin 6 (or 7) | GND           |
| TAKE button            | D2                       | X304 pin 3      | P02.0 (input, internal pull-up) |
| AUTO button            | D3                       | X304 pin 4      | P02.1 (input, internal pull-up) |
| CONFIRM button         | D4                       | X304 pin 5      | P10.4 (input, internal pull-up) |
| All buttons, other side | GND                     | X302 pin 6/7    | GND                 |

Direction: X right = steer to starboard (positive). Y forward (away from you) =
ahead. If an axis works the wrong way round, swap the two outer wires of that pot.

Software (step 13): hal_read_joystick reads EVADC group 8 channels 7 (X, P40.9) and 6 (Y, P40.8) - the
same group as the motor current and the supply divider - and the three buttons; it is switched on with
`HAL_JOY_FITTED 1` in hal/hal_pins.h once the joystick is wired (0 today: the USB bridge or no joystick).
The pins are marked CHECK there: compare them with the board before setting the flag.

### Why the 1 k resistors? (wire-break detection)

Put a **1 kΩ resistor between 3V3 and the top of each pot**, and another
**1 kΩ between the bottom of each pot and GND**:

```
 3V3 ──[1k]──┬─────────┐
             │  10k    │
             │  pot  ◄─┼── wiper ──► A0 (or A1)
             │         │
 GND ──[1k]──┴─────────┘
```

With a 10 k pot, the wiper can then only reach 1/12 … 11/12 of 3.3 V
(ADC ≈ 341 … 3754 counts). A good joystick never reads near 0 or 4095. A
reading below 100 or above 3995 therefore means a broken or shorted wire,
and the software raises a **joystick fault** within one 50 ms step:

* wiper wire broken → input floats; add a **100 k pull-down from A0/A1 to GND**
  so it goes to 0 → fault.
* 3V3 wire broken → reading goes to ~0 → fault.
* GND wire broken → reading goes to ~3.3 V → fault.

The software also flags an "impossible jump": a change of more than 2600 counts
in one 50 ms step, which is more than a hand can do. A 100 nF capacitor from
each wiper to GND, close to the board, filters noise from long cables.

### Buttons

TAKE, AUTO and CONFIRM are normally-open push buttons. Choose three buttons
that feel clearly different (CONFIRM well away from TAKE). Each one goes from the input pin
to GND, and the TC375's internal pull-up keeps the pin high when the button is
released, so pressed = low. A broken button wire only means that button stops
working. It can never make the joystick take control.

### What happens on the boat

* **At power-up** the stick must be **centred** (within 10 % for 0.2 s) before
  it is armed. If someone holds it deflected, you get the alarm "joystick not
  centred" and the stick is ignored until it is released.
* **Taking control (MANUAL):** push the stick sideways (X) past 15 % for more
  than 0.1 s, or forward/back (Y) when not in HDG LOCK, or press **TAKE**.
  This works from every mode. You hear a chirp.
* **In MANUAL:** X steers the outboard over its full range. Y is throttle:
  up = ahead, down = astern, centre = idle and neutral. Gear changes always go
  through idle → neutral → gear. The reverse limit is 30 %.
* **In HDG LOCK:** the autopilot steers, Y is still your throttle (ahead and
  astern). X takes over (MANUAL).
* **Leaving MANUAL:** centre the stick, then press joystick **AUTO** (back to HDG
  LOCK if that was interrupted, otherwise STANDBY) or the panel **HDG LOCK**
  button. If the stick is not centred, it is refused with a chirp. The panel
  **STANDBY** button always works; if the stick is still held, it is ignored
  until it has been centred. The PC can never take control from the joystick.
* **CONFIRM** only answers a request from the ground station (AUTO start /
  resume). It counts only with the stick centred and the joystick armed. It
  never changes the mode on its own.
* **Joystick fault while it is driving (MANUAL, or throttle in HDG LOCK)** →
  throttle to idle, then neutral, steering stops, STANDBY and alarm.

### Centre calibration

With the boat in STANDBY:
1. Start calibration (ground station: Config → "Joystick calibrate start").
2. Move the stick slowly to all four ends, then let it spring back to centre.
3. Finish calibration. The new min/centre/max values are saved to flash
   with a CRC-32 checksum. If they are implausible (stick not moved, centre
   too close to an end), the calibration is refused and the old one stays in
   use. A corrupted calibration in flash is detected at start-up and the
   defaults are used instead.

## 2. Control panel buttons

Six normally-open push buttons, each from its input pin to GND (internal
pull-up, pressed = low). They use the GPIO pins of the two Shield2Go
connectors, which nothing else in this project uses.

| Button   | Function                                   | Connector / pin       | TC375 pin |
|----------|--------------------------------------------|-----------------------|-----------|
| STANDBY  | idle + neutral + steering off, every mode; in STANDBY: acknowledge alarms | Shield2Go 1, pin 8 | P00.4 |
| HDG LOCK | lock the current heading (MANUAL: stick centred) | Shield2Go 2, pin 8 | P10.8 |
| -1       | heading -1 deg (HDG LOCK)                  | Shield2Go 1, pin 12   | P23.4 |
| +1       | heading +1 deg                             | Shield2Go 2, pin 12   | P23.5 |
| -10      | heading -10 deg                            | Shield2Go 1, pin 13   | P32.2 |
| +10      | heading +10 deg                            | Shield2Go 2, pin 13   | P32.3 |
| all, other side | GND                                 | Shield2Go pin 6 (GND) | GND |

* **STANDBY** must be a big, red, easy-to-hit button. The PC can never
  override it. (Safety note: with a normally-open button, a broken wire means
  STANDBY can't be pressed. A normally-closed button that opens on press,
  plus a check that the line is normally low, would catch that - see "still
  missing" in MANUAL.txt.)
* Use marine-grade (IP67) buttons; keep the wires short and away from the
  ignition and the motor cables.
* The software debounces the buttons: a level must be the same in two samples
  50 ms apart (hal_tc375.c `hal_get_button_presses`). A press is reported
  once, so a button held down does not repeat.

## 3. Actuators, reset behaviour and fail-safe

### Pin plan (outputs and the neutral switch)

From step 11e the small boat drives the throttle, gear and steering servos over DroneCAN (section 3E); the servo PWM rows below stay for the big boat and until then.

| Signal | Board pin | Connector / pin | TC375 pin | Notes |
|--------|-----------|-----------------|-----------|-------|
| Throttle servo signal | D9 | X303 pin 2 | P02.7 (GTM TOM0 ch 15, TOUT7) | 50 Hz, 1000 us = idle, 2000 us = full (calibrated) |
| Gear servo signal | D8 | X303 pin 1 | P02.6 (GTM TOM0 ch 14, TOUT6) | 1900 F / 1500 N / 1100 R us (configurable) |
| Steering motor PWM | D6 | X304 pin 7 | P02.5 | to the Pololu G2 **PWM**: high = motor runs at full speed (on / off; the angle is estimated from the run time), low = brake |
| Steering motor direction | D7 | X304 pin 8 | P02.4 | to the G2 **DIR** |
| Steering motor driver sleep | X1/X2 header | CHECK | P33.7 | to the G2 **SLP**: high = driver awake; low = asleep, outputs open (power-up, monitor trip). The G2 pulls SLP low itself |
| Steering motor current | A3 | X301 pin 4 | P40.6 = AN36 (EVADC group 8 ch 4) | from the G2 **CS** (about 20 mV per amp + 50 mV) |
| Safety relay driver | D10 | X303 pin 3 | P10.5 | high = relay closed (actuators allowed); through a transistor / relay driver, never the coil directly. **From step 11e (owner decision OD-2, 2026-10-28): P10.5 drives the ENGINE KILL CIRCUIT, used only in UNCREWED; it never cuts the servo power** |
| Neutral switch | D5 | X304 pin 6 | P02.3 (input, pull-up) | switch to GND, CLOSED when the gearbox is in neutral; also the precondition of GENERATOR mode (CLAUDE.md K): without it the generator never runs |

The Arduino SPI pins D11-D13 are used by the BNO085 (section 9); the SD card
uses the mikroBUS socket. The steering end stops, the emergency stop contact
and the buzzer are in section 11 (Stage 1 parts).

### What happens at reset and before the software runs

* `hal_init()` is the very first thing the software does: all actuator pins
  become outputs driven LOW (no servo pulses, steering motor off, safety relay
  open), before the other cores start.
* **During a reset and the ~0.3 s boot** the pins are inputs. Put a
  **10 kOhm pull-down resistor** from each servo signal, the motor PWM and the
  relay driver input to GND, so they are a defined LOW, not floating.
* A watchdog reset (frozen software, 100 ms) restarts everything: the power
  limit is back to 30 % / 5 %, the mode is STANDBY, and the outputs only arm
  again when the gear is confirmed in neutral and the stick is centred.

### What each actuator does when its signal stops (IMPORTANT)

| Actuator | No signal (reset, broken wire, board dead) | Risk |
|----------|---------------------------------------------|------|
| Throttle servo | a normal analog servo goes limp (stays where it is or follows the throttle spring); **many digital servos HOLD the last position** | the engine may stay at speed |
| Gear servo | stays where it is | the boat may stay in gear |
| Steering motor | stops (driver off) | the outboard stays at its angle |

The software can't do anything when it is not running, so **use a hardware
fail-safe** (one or more of these):

1. **Throttle return spring** on the throttle linkage, strong enough to pull
   the throttle to idle when the servo is limp, plus an **analog servo** (or a
   digital servo with **"fail-safe = idle" programmed**, which goes to idle on
   signal loss).
2. **Safety relay** (driven by D10): it is open at power-up, closes only when
   the outputs are armed (neutral-start check passed), and opens when the CPU1
   safety monitor trips. Wire it so that "open" is safe:
   * it cuts the **servo power** (then the throttle spring pulls to idle), and/or
   * it is in series with the **engine kill-cord circuit** (open = engine
     stops). Strongest action; decide with your engine dealer.
   Because the relay is open whenever the board is in reset or dead, a crashed
   board also ends up safe.
3. The normal **kill cord (lanyard)** must stay fitted and worn. It works
   without any electronics.

Recommended: 1 + 2 (servo power through the relay) + 3.

**From step 11e (DroneCAN position servos, owner decisions 2026-10-28, docs/dronecan_design.md
section 13):** the servo power is NEVER cut (OD-2: a position servo without power freezes where it
is, the throttle may stay open). The safety relay on P10.5 goes into the **engine kill circuit**
instead and is used **only in UNCREWED** (a monitor trip stops the engine; in CREWED the crew has
the kill cord + e-stop). The **e-stop stops the engine through the kill circuit in every profile**,
CREWED too (OD-1). Whether the return spring can back-drive the position servo is decided on the
bench (OD-11, FIRST_TEST.md 3j).


### 3A. Engine kill relay (UNCREWED lost link) - hardware TBD

UNCREWED boat (no onboard joystick at power-up, CLAUDE.md section B): when the
link to the captain's shore station is lost, the AURIX first goes to STANDBY
(idle + neutral), then stops the engine through a **relay in the outboard's
kill-switch circuit** - at once near the shore (closer than `near_shore_m`,
100 m, or no valid shore distance from the gateway), at open sea after
`lostlink_grace_s` (10 s) unless the link came back. Once killed it stays
killed until the AURIX is **power cycled on board** (a watchdog or software
reset keeps the kill: the latch is kept in RAM that the startup does not clear) or the HELM station
sends a remote RESTART (CLAUDE.md N: only with the throttle idle, the gear in
neutral confirmed and the e-stop released - checked on the AURIX). A CREWED
boat never uses it. The kill stays on its own AURIX pin, never the relay board.

From step 11e (owner decisions 2026-10-28): the same kill circuit is also opened by the
**e-stop in every profile** (a hard-wired e-stop contact, OD-1), by a **monitor trip in UNCREWED**
(P10.5, OD-2) and **at once** when the throttle node is lost in UNCREWED (OD-3, no wait for idle).
The SOFTWARE kill is still never used in CREWED.

| Item | Decision | State |
|------|----------|-------|
| AURIX pin | `PIN_ENGINE_KILL` in hal/hal_pins.h; P10.5 (OD-2) drives the kill circuit - one pin for both or a second relay in the same circuit | **TBD** (not defined: `hal_set_engine_kill` drives no pin yet) |
| E-stop contact | an e-stop NC contact (or a relay it drives) in series with the kill circuit, CREWED and UNCREWED (OD-1) | **TBD** (with the engine dealer) |
| Relay type | automotive / marine relay, coil via a transistor driver (never the pin directly), contact rating for the kill circuit | **TBD** (check the outboard's kill-switch circuit with the engine dealer) |
| Fail-safe direction | which state stops the engine: relay energised = kill, or de-energised = kill (then the engine stops whenever the board is unpowered / in reset) | **TBD** (`ENGINE_KILL_ACTIVE_HIGH` in hal_pins.h follows the choice) |
| Wiring | in the kill-switch circuit, in addition to the kill cord (the cord must stay) | **TBD** |
| Reset on board | how the crew restarts after a kill: AURIX power cycle + the outboard's normal start | **TBD** (procedure in MANUAL 7A) |
| Remote restart | the HELM station's RESTART clears the kill (software only: the same pin goes back to "run"); then IGNITION + START through the relay board (3D) | done (software); the relay wiring TBD |
| UNCREWED dead-man (owner addition OA-3) | a **retriggerable timer relay** ("watchdog" / missing-pulse relay, 12 V) in series in the kill circuit, held "run" by an AURIX pulse; no pulse for ~1.5 s = kill BY ITSELF (a dead / frozen / unpowered AURIX stops the engine; the DroneCAN servos HOLD when the commands stop). Armed ONLY in UNCREWED (CREWED: out of the circuit - by an AURIX arming output or, recommended, an on-board UNCREWED switch the AURIX checks against the profile). Pulse output: P10.5 itself (recommended: a trip = no pulses = kill within ~1.5 s) or its own pin. docs/dronecan_design.md 14.1 | **TO CONFIRM** (part, time, contact type, fail-safe direction, arming, pin) |

The kill circuit then has, in series: the kill cord, the e-stop kill contact
(every profile), the software ENGINE KILL relay (PIN_ENGINE_KILL) and the
UNCREWED dead-man contact. The ENGINE KILL relay's fail-safe direction must
make sure a reset / dead board never kills a **CREWED** engine (recommended:
"energised = kill"); the "dead board = kill" job in UNCREWED is the dead-man's.

Until all of this is decided and tested on the bench, **no uncrewed operation**.

### 3B. Supply voltage of the 12 V bank + the starter battery isolator (CLAUDE.md J)

Power on the boat = **3 x 12 V batteries in parallel = ONE bank** for everything
(electronics, actuators, thrusters later). The starter battery is cut from the
bank **only while cranking** (a battery isolator / split-charge relay: TBD).
The AURIX measures the bank voltage through a **resistor divider on an EVADC
input**; `app/battery.c` filters it (~30 s for the LOW BATTERY alarm and the
displays, 1 s for the generator checks) - the software never raises anything
from it: LOW BATTERY is an advisory (no mode change, the power limit stays).

| Item | Decision | State |
|------|----------|-------|
| AURIX input | Arduino **A2 = P40.7 / AN37 CHECK** (EVADC group 8 channel 5, the same group as the motor current on A3); `ADC_SUPPLY_GROUP` / `ADC_SUPPLY_CHANNEL` in hal/hal_pins.h | **TBD** (pin to check on the silk screen; `HAL_SUPPLY_FITTED 0` until the divider is on the board - the boat then shows the notice NO VOLTAGE DATA, no alarm) |
| Divider | 47 kOhm from the bank (+, after the main fuse) to A2, 10 kOhm from A2 to GND, 100 nF across the 10 k: ratio 5.7, 15 V -> 2.63 V (below the 3.3 V reference). Measure the real ratio with a meter and put it in `SUPPLY_DIV_RATIO` | **TBD** |
| Protection | NEVER more than 3.3 V on the pin: the divider must be fitted before the bank is connected; a 3.3 V zener / Schottky clamp to 3.3 V on A2 is cheap insurance; 1 A fuse in the sense wire at the battery end | **TBD** |
| Starter isolator | battery isolator / split-charge relay rated for the outboard's starter current: the bank stays connected to the electronics while cranking, the starter battery is isolated; type and wiring with the engine dealer | **TBD** (PARTS.md) |
| Where it shows | screen 2 status panel "Battery", screen 1 bottom line; log field `bank_v`; TELEM2 group L; parameter `batt_low_v` (11.8 V TO CONFIRM, 10..13.5) | done |

### 3C. DP thrusters + the e-stop contactor (CLAUDE.md P) - hardware TBD

Dynamic positioning (DP, Minn Kota style) uses two small electric thrusters on
the 12 V bank: a **bow TUNNEL thruster** (sideways; + = push the bow to
starboard) and a **fixed transom electric motor** (fore / aft; + = ahead).
Both are proportional. Only `app/thruster.c` writes them (clamp to the boat
file's `max_pct`, rate limit 50 %/s, heat budget `run_s` / `rest_s` per
thruster, 0 whenever DP is not active); CPU1 trips on any thrust without DP or
above the cap, and its safe state (`hal_safe_outputs`) sets both to the stop
pulse in the GTM shadow registers - also with core 0 frozen. FIRST TEST and
the big boat: never.

| Item | Decision | State |
|------|----------|-------|
| Bow thruster | small tunnel thruster, 12 V, bidirectional | **TBD** (PARTS.md) |
| Transom motor | small fixed electric motor (fore / aft), 12 V | **TBD** (PARTS.md) |
| Controllers | bidirectional motor controllers with an **RC-style input** (50 Hz pulses: `THR_STOP_US` 1500 us = stop, 1100 / 1900 us = full one way / the other, `THR_SPAN_US`); "no pulses = stop" must be their fail-safe (check the datasheet; a controller that holds the last value is NOT acceptable). A PWM + DIR controller needs a HAL change (TBD) | **TBD** |
| AURIX pins | GTM TOM0 **ch 12 -> P22.3 (TOUT50)** bow, **ch 13 -> P11.10 (TOUT99)** stern (`PIN_THR_BOW_PWM` / `PIN_THR_STERN_PWM`, `THR_BOW_TOUT` / `THR_STERN_TOUT` in hal/hal_pins.h); 10 kOhm pull-down on each signal like the servos | **TBD** (CHECK on the X1/X2 headers) |
| Fitted | `HAL_THRUSTERS_FITTED 0` until wired: the TOM channels run at the stop pulse, the pins are never touched / routed | done |
| E-stop contactor | a **contactor in the thruster power feed** (bank + -> fuse -> contactor -> controllers), its coil through the e-stop's power contact (like the actuator relay in section 11): pressed = no thruster power, whatever the software does; the software also sends 0 within one control step while pressed (sim test DP4) | **TBD** (rating per the thrusters' current; PARTS.md) |
| Fuses | one fuse per thruster at the bank end, sized per the thruster | **TBD** |
| Bank load | the thrusters pull the bank down while running: the generator (section K) pauses its voltage judgement while the thrust changes | done (software) |
### 3D. RS485 Modbus RTU relay board (CLAUDE.md M) - hardware TBD

An 8-16 channel **Modbus RTU relay board** (with board-timed pulses, e.g. the
"flash on" command) switches the boat file's relays: lights, pumps, trim and
the engine switches (IGNITION / STARTER / SHUTOFF). **No relay is in a
control or safety path**: the engine kill (3A) and the e-stop contactor stop
the engine / actuators, never this board. Driver: `hal/tc_modbus.c` (master,
one request in flight, CRC-16, functions 0x01 / 0x05 / 0x0F, readback every
100 ms, no reply 3 s = RELAY BOARD SILENT advisory); only `app/relays.c`
uses it; core 0 only (never CPU1).

| Item | Decision | State |
|------|----------|-------|
| UART | **ASCLIN4** (ASCLIN0-3 = USB, gateway, GPS, bridge): TX **P00.9**, RX **P00.12** (`RELAY_TX` / `RELAY_RX` in hal/hal_pins.h) | **TBD** (CHECK on the X1/X2 headers) |
| Driver enable | DE and /RE of the transceiver tied together on **P00.11** (`PIN_RELAY_DE`, high = transmit, released by the ASCLIN TX-complete interrupt right after the last stop bit; FIRST_TEST.md 3h checks it on the scope) | **TBD** (CHECK) |
| Transceiver | 3.3 V RS485 (MAX3485 class) at the AURIX end, 120 Ohm termination at both ends of the line, A / B twisted pair, common GND | **TBD** (PARTS.md) |
| Board | 8-16 channel Modbus RTU relay board, 12 V supply from the bank, address `RELAY_SLAVE_ADDR` (1), channels `RELAY_CHANNELS` (8) | **TBD** |
| Baud | 9600-115200 8N1 per the board (`RELAY_BAUD`, 9600 now) | **TBD** |
| Fitted | `HAL_RELAY_FITTED 0` until the board is wired: the driver is not started; a boat file with relays then shows RELAY BOARD SILENT (advisory only) | done |

The relays power up OFF on the board. Contact ratings per the loads (fuse each
load); the STARTER relay only drives the starter solenoid's control wire,
never the starter current. The STARTER is driven only by the remote-restart
rules (app/engine.c: hold-to-crank, board-timed 300 ms pulses, at most 5 s per
hold, 10 s cool-down, only at idle + neutral with the e-stop released). Check
with the engine dealer that the outboard has its own neutral-start switch in
the starter circuit (it must stay: the software check is not the only one).
No tacho input yet (TBD): the starter is limited by time only.
**Owner decision OD-9 (2026-10-28): the STARTER is ALWAYS also behind a hardware time-limit relay**
(a timer relay in the starter-solenoid control wire that opens after a fixed time, a little longer
than the software limit of 5 s - proposal 6 s, TO CONFIRM - and resets only when the START signal
goes away), whatever relay board / bus drives it (Modbus or DroneCAN). PARTS.md.

**A relay wired to the starter (solenoid control wire) MUST be declared in the
boat file in the engine group with role STARTER** (`"group": "engine"`,
`"role": "starter"`, momentary, max_s <= 5) - **never as an `extra` or
`trim` relay**. Only the STARTER role gets the hold-to-crank rules (idle +
neutral confirmed, e-stop released, HELM station, at most 5 s, cool-down); an
`extra` relay is switched by the plain relay command without any of them.
The same for the ignition and the engine stop: role IGNITION / SHUTOFF in
the engine group. (boats.py already refuses a relay named STARTER / ΜΙΖΑ
without the role - but it can't see what a channel is wired to: the wiring
and the boat file must agree.)

### 3E. DroneCAN bus: servos + relay board (CLAUDE.md T, docs/dronecan_design.md) - DESIGN, hardware TO CONFIRM

> **WARNING - node ID 10 (owner addition OA-4):** the Pixhawk (ArduPilot) is
> ALSO DroneCAN node **10** (CAN_D1_UC_NODE), and node 10 is the AURIX's
> static ID. The Pixhawk must **NEVER be on the CAN bus together with the
> AURIX**: unplug the Pixhawk's CAN cable (it no longer powers the nodes
> either) BEFORE the AURIX is connected. A bench session with the Pixhawk
> (e.g. to read the nodes' parameters) is done with the AURIX unplugged.

The AURIX replaces the ArduPilot (Pixhawk): the throttle, steering (360 deg
position servo) and gear servos stay on their **DroneCAN servo nodes**
(Matek CAN-L431), the relay board becomes a DroneCAN node too. From step 11e
the servo PWM pins of the pin plan above (P02.7 / P02.6) and the steering
motor driver are no longer used on the small boat (the big boat is
unchanged). Nothing is wired or enabled before the owner confirms the items
below (`HAL_CAN_FITTED 0` in hal/hal_pins.h).

Software (step 11d): the driver exists (hal/tc_can.c = MCMCAN CAN0 node 0 with
the pins / STB level / 1 Mbit/s / 87.5 % sample point of hal_pins.h, hal/dronecan.c
+ libcanard): the AURIX's NodeStatus (node 10) 1 Hz, the node table, CAN NODE LOST /
CAN BUS FAULT -> STANDBY. With `HAL_CAN_FITTED 0` it is compiled and built but never
started; no servo is driven over CAN before step 11e.

| Item | Decision | State |
|------|----------|-------|
| CAN controller | MCMCAN CAN0 node 0, owned by core 0 (CPU1 has one reserved TX buffer for the safe frames) | design |
| TXD / RXD | **P20.8** = `IfxCan_TXD00_P20_8_OUT`, **P20.7** = `IfxCan_RXD00B_P20_7_IN` (the iLLD pin map confirms these functions) | **TO CONFIRM** that the kit wires them to its transceiver (kit user manual / schematic, not in the repo) |
| Transceiver | on the kit: **TLE9251VSJ**; STB on **P20.6** (GPIO), STB high = stand-by / low = normal per the TLE9251V datasheet | **TO CONFIRM** (pin + polarity + any pull resistor on the kit) |
| Connector | the kit's CAN connector | **TO CONFIRM** type and pinout (CAN_H, CAN_L, GND) |
| Termination | 120 Ohm at BOTH physical ends of the line; with everything off, CAN_H-CAN_L must read ~60 Ohm | **TO CONFIRM** whether the kit has it on board / on a jumper / not at all |
| Bus | 1 Mbit/s (ArduPilot CAN_P1_BITRATE), twisted pair, short stubs, common GND, away from the steering / starter cables | design |
| Node IDs | STATIC (no dynamic-ID server): proposal AURIX 10, throttle 20, steering 21, gear 22, relay board 30 (GPS + compass node 124 not used) | **TO CONFIRM** (set with the DroneCAN GUI Tool) |
| Node power | the nodes were powered with 5 V through the Pixhawk's CAN port: now their **own fused 5 V supply** (12 V -> 5 V converter from the bank, fuse at the bank end, a fuse per branch), NEVER from the AURIX kit | **TO CONFIRM** converter rating; how each node powers its servo (the servos can draw amps) |
| Command timeouts (set on each node: AP_Periph `SRV_CMD_TIME_OUT`, ms, 0 = never) | AP_Periph has **no "idle on timeout"**: after the timeout it "zeros" the output - the pulses stop, OR 0.0 = 1500 us = **~50 % throttle** (unknown!). Gear **0 = HOLD**, steering **0 = HOLD**; throttle **0 = HOLD** unless the bench shows the pulses STOP AND the return spring pulls the lever to idle (owner addition OA-1) | **TO CONFIRM** on the bench, engine OFF, before the boat moves (FIRST_TEST.md 3j) |
| Throttle node lost | gear **HOLD**, never an automatic neutral (OA-2); UNCREWED: engine kill at once (OD-3) | decided |
| E-stop / engine kill / STANDBY | stay hard-wired GPIO, never over CAN; stopping the engine never depends on the throttle servo | owner decisions 2026-10-28: the e-stop stops the engine through the kill circuit in every profile (OD-1); the servo node power is NEVER cut; P10.5 -> the engine kill circuit, UNCREWED only (OD-2); throttle node lost in UNCREWED = engine kill at once (OD-3) |
| Steering at power-up | no steering frame until a mode commands steering, no auto-centre (OD-7); STANDBY = hold (OD-10) | decided |
| STARTER | always behind a hardware time-limit relay, on either relay bus (OD-9, section 3D) | **TBD** part |
| Relay board on DroneCAN (step 11f) | node **30**, relays 1-6 = hardpoints 0-5 (ArduPilot RELAY1..6), hardpoint.Command at 50 Hz while a state is new / a pulse runs (hal/can_relay.c). A BUILD OPTION: `RELAY_BUS` in hal_pins.h = `RELAY_BUS_MODBUS` (default, section 3D) or `RELAY_BUS_DRONECAN` (+ `HAL_CAN_FITTED` 1 + `HAL_RELAY_FITTED` 1). The board MUST: R-1 static node ID 30 + NodeStatus >= 1 Hz; R-2 the non-momentary relays HOLD their state when the commands stop and power up OFF; R-3 the momentary relays (TRIM UP / DOWN, the STARTER) switch OFF by themselves <= 300 ms after the last command; nice: R-4 hardpoint.Status readback (without it the screens show the commanded states, "no readback") | **TO CONFIRM** board model, node ID, timeout settings, readback (C17); the default stays MODBUS until the owner chooses (11f review) |

**Bench test of the command timeouts (engine OFF, DroneCAN GUI Tool on a
laptop directly on the bus, not through the AURIX):** every node with
`SRV_CMD_TIME_OUT` 0: command a position, stop the commands - every servo
HOLDS. Then the throttle node with a test timeout (e.g. 300 ms): command mid
throttle, stop the commands, scope the node output: do the pulses STOP or
does it go to 1500 us (= half throttle)? Does the return spring then pull
the lever to idle? Only "pulses stop AND spring to idle" allows a throttle
timeout; otherwise it stays 0 (hold). Never assume "timeout = idle". Write the measured times
and the servo speeds (steering lock to lock, throttle idle to full) into
FIRST_TEST.md. Until then the software uses 12 s / 6 s as the worst case.

### 3F. Paddlewheel speed sensor (optional, step 13) - hardware TBD

The dead reckoning (app/dr.c) uses a paddlewheel's speed through water when it is fitted (better DR, the
faster GPS re-trust of part T3). Sensor TBD (PARTS.md): a paddlewheel with a pulse output, K factor =
pulses per second at 1 kn (typical ~4.9 Hz per kn, `PADDLE_HZ_PER_KN` in hal_pins.h - set the real one).

| Signal            | Board pin (CHECK X1/X2)        | TC375 pin            | Note |
|-------------------|--------------------------------|----------------------|------|
| Pulse output      | X1/X2 header (TBD)             | P00.7 = GPT12 T2INA  | counted by the GPT12 timer T2 in hardware (no interrupt) |
| Sensor supply     | its own fused supply (TBD)     | -                    | as the sensor needs (often 12 V) |
| GND               | GND                            | GND                  | common with the board |

* **3.3 V on the pin, never more.** An open-collector output: pull-up to the board's 3V3 (4.7 kOhm) on the
  pin side. A sensor that gives 12 V pulses: an optocoupler or a level shifter (TBD) - never 12 V on P00.7.
* 100 nF to GND near the board against noise on a long cable.
* Software: hal/tc_pulse.c (pure C, sim tests PW1-PW4) - 0.5 s windows, a boat that slows to a stop reads
  0 kn; pulses that stop at once from speed (cut cable, stuck wheel) or an implausible rate (> 40 kn)
  = NO DATA (the DR then uses its throttle table). `HAL_PADDLE_FITTED 1` in hal_pins.h once it is wired.

## 4. microSD card (log)

A **microSD click** board (MikroElektronika MIKROE-924, 3.3 V) plugged into the
board's **mikroBUS socket** is easiest. A 3.3 V microSD breakout board wired
to the same pins works too. **Never use a 5 V module.**

| microSD signal | mikroBUS pin | TC375 pin | Function |
|----------------|--------------|-----------|----------|
| CS             | 3 (CS)       | P14.7     | chip select |
| SCK            | 4 (SCK)      | P15.8     | QSPI2 clock |
| MISO (DO)      | 5 (MISO)     | P15.7     | QSPI2 MRST (data from the card) |
| MOSI (DI)      | 6 (MOSI)     | P15.6     | QSPI2 MTSR (data to the card) |
| card detect    | 15 (INT)     | P10.7     | low = card inserted (on the click) |
| 3.3 V          | 7 (3.3V)     | -         | |
| GND            | 8 / 9 (GND)  | -         | |

* Card: a good-brand **industrial / high-endurance microSD, 8-32 GB**,
  formatted **FAT32**. Read it on the PC with a normal card reader.
* About 20 MB per hour of logging. A 16 GB card holds weeks.
* Files: `Bbbbbb_fff.TLG` (boot number, file number; with the date and time
  once GPS time is known), a new file every 10 min. `BB..._nnn.TLG` are the
  black-box files (the 60 s before an alarm / fault and 30 s after).
* Pull the card only in STANDBY. At most the last ~2 s are lost; older files
  are never damaged.
* Check the pin functions against the TC37x data sheet before you wire
  (QSPI2 alternate functions on P15.6/7/8 and P14.7).

## 5. Ground-station link (PC)

Nothing to wire. Use the board's **USB cable**: the on-board debugger
(miniWiggler) also provides a **virtual COM port**, connected to ASCLIN0
(P14.0 = TX, P14.1 = RX), at **921600 baud, 8N1**. In Windows Device Manager
it shows as "USB Serial Port (COMx)".

The gateway PC uses the same protocol over a 3.3 V UART (section 7).
Do not use D0/D1 (P15.3/P15.2) at the same time, because they are an
alternative pin pair of the same ASCLIN0.

## 6. USB joystick bridge (Thrustmaster T.16000M FCS HOTAS)

The USB joystick is read by a separate small board, a **Teensy 4.1**. Its
only job is to read the joystick and forward it (firmware and setup:
`bridge/README.md`). The Teensy sends a checked frame (CRC, counter) to the
AURIX 100 times a second. The AURIX sends nothing safety-critical back.
The hard-wired analog joystick (section 1) stays possible: the source is
chosen in the ground station (**Joystick** tab).

```
 12 V boat ──fuse 2 A──► 5 V DC-DC converter (3 A) ──┬──► powered USB hub (DC in)
                                                     └──► Teensy 4.1 VIN
 T.16000M stick ──USB──┐
                       ├── powered USB hub ──USB──► Teensy 4.1 USB host header (5-pin, with the host cable)
 TWCS throttle ───USB──┘
 Teensy pin 1 (TX1) ──[100 Ω]──────────────────────► AURIX P20.3 (ASCLIN3 RX)
 Teensy GND ────────────────────────────────────────► AURIX GND   (common ground, same wire bundle)
```

| Signal | Teensy 4.1 | AURIX board | TC375 pin | Notes |
|--------|------------|-------------|-----------|-------|
| bridge data (Teensy -> AURIX) | pin 1 (TX1) | Shield2Go 2 connector, RX pin | P20.3 (ASCLIN3 RXC) | 230400 baud 8N1, 3.3 V; 100 Ω series resistor at the Teensy end |
| (spare, not used) | pin 0 (RX1) | Shield2Go 2 connector, TX pin | P20.0 (ASCLIN3 TX) | leave unconnected |
| ground | GND | GND (Shield2Go pin 6) | - | **must** be connected |
| power | VIN (5 V) | - | - | from the 5 V converter, not from the AURIX |

* **Pins:** P20.3 / P20.0 are free in this pin plan. Shield2Go 1 (P33.8 / P33.9,
  ASCLIN2) stays reserved for the GPS. The iLLD pin map confirms
  `IfxAsclin3_RXC_P20_3_IN` and `IfxAsclin3_TX_P20_0_OUT`. Check on the
  board's silk screen / user manual which Shield2Go 2 pin carries P20.3
  before you solder.
* **Both boards are 3.3 V logic**, so no level shifter is needed. Never
  connect a 5 V signal to the AURIX.
* **Power:**
  * Supply the hub and the Teensy from one **12 V → 5 V DC-DC converter**
    rated at least **3 A**, with a 2 A fuse on its 12 V side.
  * Typical draw: Teensy 0.1 A + T.16000M 0.1 A + TWCS 0.1 A + hub
    0.1 A = about 0.5 A. The rest is margin for plug-in surges.
  * A separate 5 V supply (e.g. a marine USB socket) works too, if it gives
    at least 2 A.
  * The hub must have its **own** power input. The Teensy's USB host port
    can't power two joysticks.
* **Teensy on the PC for setup:** if the Teensy gets 5 V on VIN *and* you
  plug it into the PC by USB, the two 5 V sources meet. Either cut the
  VIN-VUSB pad on the Teensy (see the PJRC "external power" page), or
  disconnect the 5 V converter while it is plugged into the PC.
* **Common ground:** run the TX wire and a GND wire together (twisted, or a
  shielded 2-core cable), as short as possible (< 1 m). The AURIX, the
  Teensy and the 5 V converter must share one ground.
* **Strain relief:**
  * Clamp every USB cable (P-clips or cable ties to a fixed point) within
    10 cm of each plug, so a pull on the joystick cable never pulls a plug.
  * Glue or clip the USB host cable to the Teensy header.
  * Leave a drip loop below each cable entry.
* **Mounting:** the T.16000M, the TWCS, the hub and the Teensy are **not
  waterproof**.
  * Mount them in a **dry console** under a cover. Put the hub and the
    Teensy in a closed box with cable glands, the glands facing down, with a
    desiccant bag inside.
  * Spray or wet hands on the joystick are fine. Standing water or rain on
    it is not.
* **What happens when something breaks:**
  * USB unplugged, hub without power, Teensy crashed, UART wire broken: the
    AURIX sees no valid frames for 150 ms (or a device flag goes off). That
    is a JOYSTICK FAULT.
  * If the joystick was in control: throttle idle + neutral, steering
    stop, STANDBY, alarm.

## 7. Gateway PC (remote access): Lenovo ThinkPad L390 Yoga

The gateway PC is the only device on the network; the AURIX talks to it over a UART
(same framed protocol with CRC as the USB link) through a **3.3 V TTL USB-UART
adapter**. Software and setup (Ubuntu 24.04 LTS or Windows 11): `gateway/README.md`.

> **WARNING: ONLY a 3.3 V TTL USB-UART adapter (FTDI FT232R / FT231X "3V3", or a
> CP2102 set to 3.3 V). NEVER an RS232 port, a "USB to RS232 / DB9" cable or a PC
> COM port: RS232 swings +-12 V and DESTROYS the AURIX pins at once. Never a 5 V TTL
> adapter either.** Check with a meter before connecting: adapter TXD idle = 3.3 V
> to GND (not 5 V, not -5..-12 V).

| Signal | AURIX | TC375 pin | USB-UART adapter (3.3 V TTL) | Notes |
|--------|-------|-----------|------------------------------|-------|
| AURIX → PC | ASCLIN1 TX | P15.0 (`IfxAsclin1_TX_P15_0_OUT`) | RXD | 921600 baud 8N1, 100 Ω in series at the AURIX end |
| PC → AURIX | ASCLIN1 RX | P15.1 (`IfxAsclin1_RXA_P15_1_IN`) | TXD | 100 Ω in series at the adapter end |
| ground | GND | - | GND | **must** be connected |
| (adapter VCC / 5 V / 3V3 out) | - | - | **leave unconnected** | the AURIX has its own supply; never feed it from the adapter |

* **Pins:** P15.0 / P15.1 are not used by anything else in this pin plan.
  The iLLD pin map confirms the ASCLIN1 functions. The board manual says
  "most pins are on the expansion headers X1/X2". **Check which X1/X2
  pins carry P15.0 / P15.1 before you solder.** Not P15.2 / P15.3 (D0/D1):
  those belong to ASCLIN0, the USB laptop link.
* **Both sides are 3.3 V.** The adapter must be a 3.3 V one: a cable labelled
  "TTL-232R-3V3", or a board whose VCCIO jumper is set to 3.3 V (check it).
  It must do 921600 baud (FT232R / FT231X / CP2102(N): yes).
* **Put the adapter INSIDE the AURIX box**, TTL wires < 30 cm, and run the USB
  cable (up to 3-5 m, shielded, ferrite at the PC end) to the gateway PC. The
  USB cable is the long part, not the 3.3 V wires.
* The serial port name in gateway.json is a stable one (`usb:VID:PID:SERIAL`, or
  `/dev/serial/by-id/...` on Ubuntu): `python gateway.py --list-serial` shows it.
* **Power for the gateway PC:** a **12 V → USB-C PD car charger, 65 W** (car/marine
  grade, fused at the bank: fuse rating per the charger's manual, model TBD), from the same bank
  as the AURIX (common ground). The **built-in battery is the UPS**: when the bank
  power drops or the charger fails, the laptop runs on its battery. Battery charge
  limit **80 %** (long battery life when always on the charger): Ubuntu TLP
  (`gateway/systemd/tlp-trygons.conf`), Windows Lenovo Vantage.
* **Lid closed must keep running** (both OSes: the install scripts set it,
  `gateway/README.md` section 3).
* **Ethernet:** the boat LAN (Teltonika router LAN1) and the camera network (TSW101
  PoE switch port 5) are two cables. The laptop's RJ45 port if it has one, else a
  **USB 3 Gigabit Ethernet adapter** for each (Realtek RTL8153 class) - RJ45 / port
  count on this laptop: TBD (PROGRESS.md WAITING FOR OWNER W17).
* Mount the laptop dry (it is not waterproof), in a ventilated place, closed, held
  down (vibration), cables with strain relief, away from the compass (> 50 cm).
* **If the gateway PC fails** (crash, power, cable): the AURIX sees no status for
  3 s and shows **GATEWAY LOST**. There is no mode change (unless remote
  driving was active). The USB laptop, joystick and STANDBY button are
  unaffected.

## 8. Cameras (4 x Tapo C310) and the camera switch (Teltonika TSW101)

The Tapo C310 runs on **9 V DC, 0.6 A** (its adapter says "9.0V ⎓ 0.6A";
check the label of yours and the plug size before buying splitters). No
camera is ever needed for a safety function.

| Camera | Power | Network |
|--------|-------|---------|
| FRONT-1 (main) | TSW101 PoE+ port 1 → 802.3af PoE splitter **9 V** at the camera | TSW101 → gateway PC cam0 (camera network) |
| REAR-LEFT | TSW101 PoE+ port 2 → PoE splitter 9 V | TSW101 → gateway PC cam0 |
| REAR-RIGHT | TSW101 PoE+ port 3 → PoE splitter 9 V | TSW101 → gateway PC cam0 |
| FRONT-2 (backup) | its **own** 12 V → 9 V DC-DC converter (≥ 1 A) with its **own 2 A fuse**, separate cable run | its own cable to RUT901 LAN3 (network cam2) |

* **TSW101:** 9-30 V DC terminal block, from 12 V through a **5 A fuse**.
  * PoE+ ports give up to 30 W each, 60 W in total; four cameras need
    about 4 × 5.4 W.
  * Each PoE port is current-limited: a short on one camera cable doesn't
    take the others down.
  * TSW101 port 5 (no PoE) → the gateway PC's camera port cam0 (its RJ45 or a USB 3
    Gigabit Ethernet adapter, section 7).
* **FRONT-1 and FRONT-2 never share anything:** separate cables, separate
  power (PoE port vs. own converter + fuse), separate switch port / router
  port, separate cable entries. One failure can take out only one of them.
* If you prefer DC-DC converters for all cameras (no PoE): one 12 V → 9 V
  converter per camera, each with its own 2 A fuse, and plain Ethernet into
  the TSW101.
* Cable entries: IP68 cable glands, glands facing down, drip loops. Seal
  the RJ45 plugs at the cameras with waterproof RJ45 couplers or
  self-amalgamating tape. The PoE splitters and DC-DC converters go inside
  a closed box, not on deck.
* Outdoor cable: Cat6 outdoor / UV-resistant, at least 20 cm away from VHF
  and engine cables, never across the compass (> 50 cm).
* Mount FRONT-1 and FRONT-2 a little apart (spray / one dirty lens doesn't
  blind both).
* **Lens care against salt:**
  * rinse with fresh water and wipe with a soft microfibre cloth after
    every trip;
  * never wipe dry salt (scratches);
  * a little hydrophobic glass coating helps;
  * check the seals every season.

## 9. IMU = compass + gyro + attitude + slamming sensor (BNO085)

The BNO085 is the boat's **compass** (heading from its rotation vector, with
the magnetometer), its **gyro** (yaw rate), the roll / pitch for the camera
horizon and the vertical acceleration for the slamming measurement (MANUAL
chapter 9 part H). Driver: hal/tc_bno085.c (SPI, SHTP protocol). It runs on
QSPI1 = the Arduino SPI pins; the control loop (core 0) reads it.

| BNO085 breakout | Board pin | TC375 pin | Notes |
|-----------------|-----------|-----------|-------|
| VIN | 3.3 V | - | **not 5 V** |
| GND | GND | - | |
| SCL / SCK | D13 | P10.2 (QSPI1 SCLK) | CHECK on the board |
| SDA / MISO | D12 | P10.1 (QSPI1 MRST A) | CHECK |
| DI / MOSI | D11 | P10.3 (QSPI1 MTSR) | CHECK |
| CS | X1/X2 header | P10.0 | CHECK; software chip select |
| INT (H_INTN) | X1/X2 header | P33.5 | CHECK; low = data ready |
| RST (NRST) | X1/X2 header | P33.4 | CHECK; low = reset |
| PS0 / WAKE | X1/X2 header | P33.6 | CHECK; in SPI mode PS0 is the WAKE input |
| PS1 | 3.3 V | - | PS1 = 1 and PS0 = 1 at reset = SPI mode |

* Reports used: rotation vector 50 Hz (heading), calibrated gyro 50 Hz (yaw
  rate), game rotation vector 50 Hz (roll / pitch), linear acceleration 200 Hz
  (slamming). No report for 0.5 s = "no data": the heading / gyro timeout
  alarms of the app follow, and the driver resets the sensor (at most every 2 s).
* Mount it **rigidly** to the hull structure (not on a panel that vibrates),
  near the driver's seat / slightly forward of the centre of gravity, flat,
  with its **X axis pointing forward** (+ roll = starboard down, + pitch =
  bow up). Glue / screw - no foam.
* It is a **magnetic compass**: keep it **> 50 cm from steel, the outboard,
  the steering motor, speakers and current-carrying cables** (the motor and
  battery cables especially). Calibrate it by turning the boat slowly through
  two full circles (FIRST_TEST.md); the bench test shows its accuracy (0-3).
* Directions are checked in the bench test (FIRST_TEST.md): turn it clockwise
  seen from above -> the heading goes up and the yaw rate is positive. If one is
  the wrong way, flip the sign in hal/hal_pins.h (`BNO_..._SIGN`); a heading
  error from the mounting goes into `BNO_HEADING_OFFSET`.
* Cable max ~1 m, shielded, away from the steering motor and ignition cables.

## 10. AIS receiver (receive only) and antenna

| Part | Connection | Notes |
|------|------------|-------|
| USB AIS receiver (receive only; model TBD, PARTS.md) | gateway PC USB | NMEA 0183, 38400 baud; set `ais.source` in gateway.json to its stable name (`python gateway.py --list-serial`: `usb:VID:PID:SERIAL`, Ubuntu also `/dev/serial/by-id/...`) |
| AIS / VHF antenna | BNC / SMA on the receiver | its own antenna (best), or a passive VHF/AIS splitter on the existing VHF antenna |

* The AIS receiver and the AURIX's USB-UART adapter are two USB serial devices:
  each one by its own stable name; `--check-config` refuses the same name twice. If
  both use the same chip without a serial number, the `usb:VID:PID` name can't tell
  them apart - use `/dev/serial/by-id/...` / the serial number (UNVERIFIED until the
  parts are here).
* Antenna: as high as possible, vertical, at least 1 m from the VHF
  antenna if it is a separate one; good-quality coax (RG-58 up to 5 m,
  RG-8X / LMR-240 longer), weatherproof the connectors (self-amalgamating
  tape), drip loop.
* A passive splitter must be a real AIS/VHF splitter (it protects the
  receiver while the VHF transmits) - never a plain T-piece.
* Radar reflector: AIS receive-only does not make us visible to ships.
  Mount a good radar reflector (e.g. an Echomax / Tri-Lens, as high as
  possible) so ships' radars see us.

## 11. Stage 1 parts: GPS, steering motor driver, end stops, emergency stop, buzzer

Everything for the first steering test (FIRST_TEST.md). Pins marked CHECK are
on the X1 / X2 expansion headers: find the port pin on the board's silk
screen / the Lite Kit user manual BEFORE you wire it, and change hal/hal_pins.h
if needed. The bench test shows each input live, so a wrong pin is found at
once.

### GPS: u-blox MAX-M10S breakout (3.3 V)

| GPS breakout | TC375 pin | Notes |
|--------------|-----------|-------|
| VCC (3V3) | 3.3 V | not 5 V unless the breakout has its own regulator |
| GND | GND | |
| TXD (from the GPS) | P33.8 (ASCLIN2 RX E) | Shield2Go 1 UART pins, CHECK |
| RXD (to the GPS) | P33.9 (ASCLIN2 TX) | CHECK |
| antenna | - | active patch antenna with a clear view of the sky, away from the cameras' and the gateway PC's cables |

* The module starts at 9600 baud. The software switches it to 38400 and turns
  on the u-blox binary messages NAV-PVT (the fix, 1 Hz), NAV-STATUS
  (**spoofing** detection) and MON-RF (**jamming**) - in its RAM only, so it is
  done again at every power-up and whenever the module stops sending them.
  If the module ignores the setup, the standard NMEA RMC sentence is used
  (no spoofing / jamming flag then).
* No message for 3 s = "no data": GPS LOST in the app (dead reckoning, see
  MANUAL). The first fix after a cold start can take ~30 s outdoors.

### Steering motor driver: Pololu G2 High-Power Motor Driver 18v17

| G2 pin | Connect to | Notes |
|--------|------------|-------|
| VIN / GND (big pads) | 12 V battery through the **emergency stop relay** and a **20 A fuse** | thick wires (2.5 mm2), short |
| OUTA / OUTB | steering motor | swap them if the motor runs the wrong way (or flip the direction in the TRAINING calibration) |
| PWM | D6 = P02.5 | |
| DIR | D7 = P02.4 | |
| SLP | P33.7 (CHECK) | 10 kOhm pull-down to GND is on the G2 already |
| CS | A3 = P40.6 | max 3.3 V: at 17 A it is ~0.4 V |
| GND (logic) | board GND | one common ground point |

* The G2's logic inputs work with 3.3 V. Never connect the motor supply to the
  board.
* Put a 100 uF / 35 V low-ESR capacitor across VIN near the G2 if the wires
  to the battery are longer than ~50 cm.

### Steering end stops (limit switches)

Two **normally closed** micro switches (IP67), one at each steering end, each
from its input to GND. **Open = at the end stop or the wire is broken**: the
software treats both the same (the motor stops in that direction).

| Switch | TC375 pin | Notes |
|--------|-----------|-------|
| LEFT end (port) | P33.1 (CHECK) | internal pull-up; add 4.7 kOhm to 3.3 V + 100 nF to GND at the board for noise |
| RIGHT end (starboard) | P33.2 (CHECK) | same |

### Emergency stop

A red **mushroom switch with two normally closed contacts** (twist to release):

1. **Power contact** - in series with the coil of a 30 A automotive relay that
   feeds the **steering motor driver** and the **servo power** (VIN of the G2,
   the servos' 5-6 V supply), and the coil of the **DP thruster contactor**
   (section 3C). Pressed = the relay / contactor drop = no actuator or
   thruster power, whatever the software does. (Engine kill: see section 3.)
   **From step 11e (owner decisions OD-1 / OD-2, 2026-10-28):** the DroneCAN
   servo nodes are NOT fed through this relay any more (never cut servo
   power); instead the e-stop ALSO opens the **engine kill circuit** (a third
   NC contact or a relay, section 3A), in CREWED and UNCREWED.
2. **Signal contact** - from P33.0 (CHECK) to GND, so the software knows:
   open (high) = pressed, or the wire is broken. 4.7 kOhm to 3.3 V + 100 nF to
   GND at the board.

The software sees "pressed" at once and STANDBY follows (throttle idle +
neutral first, then steering off) with the EMERGENCY STOP alarm; every
command except STANDBY is refused while it is pressed. After releasing: press
STANDBY to acknowledge. `HAL_ESTOP_FITTED 0` in hal/hal_pins.h only for a
bench without the switch.

### Buzzer

| Part | Connect to | Notes |
|------|------------|-------|
| 12 V active piezo buzzer (loud, > 85 dB) | + to 12 V (fused 1 A), - to the transistor's collector / drain | |
| NPN transistor (BC337) or logic-level MOSFET (2N7000 / AO3400) | base through 1 kOhm (MOSFET gate through 100 Ohm + 100 kOhm to GND) from P33.3 (CHECK) | high = buzzer on |
| diode 1N4148 across the buzzer | cathode to + | only needed for a magnetic (not piezo) buzzer |

The buzzer sounds with LED1 whenever an alarm is active, and stays on after a
safety monitor trip. The bench test switches it on and off by itself.
