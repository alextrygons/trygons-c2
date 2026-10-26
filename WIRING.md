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

| Signal | Board pin | Connector / pin | TC375 pin | Notes |
|--------|-----------|-----------------|-----------|-------|
| Throttle servo signal | D9 | X303 pin 2 | P02.7 (GTM TOM0 ch 15, TOUT7) | 50 Hz, 1000 us = idle, 2000 us = full (calibrated) |
| Gear servo signal | D8 | X303 pin 1 | P02.6 (GTM TOM0 ch 14, TOUT6) | 1900 F / 1500 N / 1100 R us (configurable) |
| Steering motor PWM | D6 | X304 pin 7 | P02.5 | to the Pololu G2 **PWM**: high = motor runs at full speed (on / off; the angle is estimated from the run time), low = brake |
| Steering motor direction | D7 | X304 pin 8 | P02.4 | to the G2 **DIR** |
| Steering motor driver sleep | X1/X2 header | CHECK | P33.7 | to the G2 **SLP**: high = driver awake; low = asleep, outputs open (power-up, monitor trip). The G2 pulls SLP low itself |
| Steering motor current | A3 | X301 pin 4 | P40.6 = AN36 (EVADC group 8 ch 4) | from the G2 **CS** (about 20 mV per amp + 50 mV) |
| Safety relay driver | D10 | X303 pin 3 | P10.5 | high = relay closed (actuators allowed); through a transistor / relay driver, never the coil directly |
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


### 3A. Engine kill relay (UNCREWED lost link) - hardware TBD

UNCREWED boat (no onboard joystick at power-up, CLAUDE.md section B): when the
link to the captain's shore station is lost, the AURIX first goes to STANDBY
(idle + neutral), then stops the engine through a **relay in the outboard's
kill-switch circuit** - at once near the shore (closer than `near_shore_m`,
100 m, or no valid shore distance from the gateway), at open sea after
`lostlink_grace_s` (10 s) unless the link came back. Once killed it stays
killed until the AURIX is reset **on board** (power cycle); there is no remote
restart. A CREWED boat never uses it.

| Item | Decision | State |
|------|----------|-------|
| AURIX pin | `PIN_ENGINE_KILL` in hal/hal_pins.h | **TBD** (not defined: `hal_set_engine_kill` drives no pin yet) |
| Relay type | automotive / marine relay, coil via a transistor driver (never the pin directly), contact rating for the kill circuit | **TBD** (check the outboard's kill-switch circuit with the engine dealer) |
| Fail-safe direction | which state stops the engine: relay energised = kill, or de-energised = kill (then the engine stops whenever the board is unpowered / in reset) | **TBD** (`ENGINE_KILL_ACTIVE_HIGH` in hal_pins.h follows the choice) |
| Wiring | in the kill-switch circuit, in addition to the kill cord (the cord must stay) | **TBD** |
| Reset on board | how the crew restarts after a kill: AURIX power cycle + the outboard's normal start | **TBD** (procedure in MANUAL 7A) |

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

Later, a Raspberry Pi or an ESP32 can use the same protocol over a 3.3 V UART.
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

## 7. Raspberry Pi 5 gateway (remote access)

The Pi is the only device on the network; the AURIX talks to it over a UART
(same framed protocol with CRC as the USB link). Software and setup:
`gateway/README.md`.

| Signal | AURIX | TC375 pin | Raspberry Pi 5 | Notes |
|--------|-------|-----------|----------------|-------|
| AURIX → Pi | ASCLIN1 TX | P15.0 (`IfxAsclin1_TX_P15_0_OUT`) | GPIO15 = RXD, header pin 10 | 921600 baud 8N1, 100 Ω in series at the AURIX end |
| Pi → AURIX | ASCLIN1 RX | P15.1 (`IfxAsclin1_RXA_P15_1_IN`) | GPIO14 = TXD, header pin 8 | 100 Ω in series at the Pi end |
| ground | GND | - | GND, header pin 6 | **must** be connected |

* **Pins:** P15.0 / P15.1 are not used by anything else in this pin plan.
  The iLLD pin map confirms the ASCLIN1 functions. The board manual says
  "most pins are on the expansion headers X1/X2". **Check which X1/X2
  pins carry P15.0 / P15.1 before you solder.** Not P15.2 / P15.3 (D0/D1):
  those belong to ASCLIN0, the USB laptop link.
* **Both sides are 3.3 V.** Never connect a 5 V UART (no USB-serial
  adapters at 5 V).
* Cable: shielded 3-core, **< 1 m**, AURIX box to Pi box. If the Pi must be
  further away or on a different supply, use a digital isolator (e.g.
  ISO7721) or RS-485 transceivers at both ends.
* **Power for the Pi:**
  * 12 V → 5.1 V **5 A** USB-C converter (car/marine grade), 5 A fuse on
    the 12 V side, from the same battery as the AURIX.
  * The Pi 5 needs 5 A to give its USB ports enough current for the SSD.
  * With a 3 A supply the Pi limits USB to 600 mA and the SSD can drop
    out (= GATEWAY STORAGE alarm).
* **USB SSD** directly on a blue USB 3 port of the Pi, cable tied down
  (vibration). The Pi's own microSD card holds only the operating system.
* Ethernet from the Pi to the Teltonika router (LAN port). The router
  setup, VLANs and the cameras: sections C/E of the manual and
  `gateway/README.md`.
* Mount the Pi in a closed box with a heat sink / the active cooler, cable
  glands facing down, away from the compass (> 50 cm).
* **If the Pi fails** (crash, power, cable): the AURIX sees no status for
  3 s and shows **GATEWAY LOST**. There is no mode change (unless remote
  driving was active). The USB laptop, joystick and STANDBY button are
  unaffected.

## 8. Cameras (4 x Tapo C310) and the camera switch (Teltonika TSW101)

The Tapo C310 runs on **9 V DC, 0.6 A** (its adapter says "9.0V ⎓ 0.6A";
check the label of yours and the plug size before buying splitters). No
camera is ever needed for a safety function.

| Camera | Power | Network |
|--------|-------|---------|
| FRONT-1 (main) | TSW101 PoE+ port 1 → 802.3af PoE splitter **9 V** at the camera | TSW101 → Pi eth1 (camera network) |
| REAR-LEFT | TSW101 PoE+ port 2 → PoE splitter 9 V | TSW101 → Pi eth1 |
| REAR-RIGHT | TSW101 PoE+ port 3 → PoE splitter 9 V | TSW101 → Pi eth1 |
| FRONT-2 (backup) | its **own** 12 V → 9 V DC-DC converter (≥ 1 A) with its **own 2 A fuse**, separate cable run | its own cable to RUT901 LAN3 (network cam2) |

* **TSW101:** 9-30 V DC terminal block, from 12 V through a **5 A fuse**.
  * PoE+ ports give up to 30 W each, 60 W in total; four cameras need
    about 4 × 5.4 W.
  * Each PoE port is current-limited: a short on one camera cable doesn't
    take the others down.
  * TSW101 port 5 (no PoE) → the Pi's USB 3 Gigabit Ethernet adapter (eth1).
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
| dAISy HAT (Wegmatt) | sits on the Pi's 40-pin header | NMEA on the Pi's serial port `/dev/serial0` (38400 baud): enable the serial port, disable the serial console (`sudo raspi-config` → Interface → Serial: login shell NO, hardware YES) |
| or a USB AIS receiver | Pi USB | appears as `/dev/ttyACM0` / `/dev/ttyUSB0`: set `ais.source` in gateway.json |
| AIS / VHF antenna | BNC / SMA on the receiver | its own antenna (best), or a passive VHF/AIS splitter on the existing VHF antenna |

* The Pi's UART pins GPIO14/15 are already used for the AURIX (section 3).
  The dAISy HAT also uses `/dev/serial0` - so with the HAT, move the AURIX
  link to a USB-UART adapter (`aurix: "/dev/ttyUSB0"` in gateway.json) or
  use a USB AIS receiver instead. UNVERIFIED until the parts are here:
  check which serial port each one gets (`ls -l /dev/serial*`).
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
| antenna | - | active patch antenna with a clear view of the sky, away from the cameras' and the Pi's cables |

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
   the servos' 5-6 V supply). Pressed = the relay drops = no actuator power,
   whatever the software does. (Engine kill: see section 3.)
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
