# Wiring: joystick, control panel and ground-station link (KIT_A2G_TC375_LITE)

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
* The software debounces the buttons (TODO in hal_tc375.c).

## 3. Actuators, reset behaviour and fail-safe

### Pin plan (outputs and the neutral switch)

| Signal | Board pin | Connector / pin | TC375 pin | Notes |
|--------|-----------|-----------------|-----------|-------|
| Throttle servo signal | D9 | X303 pin 2 | P02.7 (GTM TOUT7) | 50 Hz, 1000 us = idle, 2000 us = full (calibrated) |
| Gear servo signal | D8 | X303 pin 1 | P02.6 (GTM TOUT6) | 1900 F / 1500 N / 1100 R us (configurable) |
| Steering motor driver PWM | D6 | X304 pin 7 | P02.5 (GTM TOUT3) | to the H-bridge (motor driver) |
| Steering motor direction | D7 | X304 pin 8 | P02.4 | |
| Safety relay driver | D10 | X303 pin 3 | P10.5 | high = relay closed (actuators allowed); through a transistor / relay driver, never the coil directly |
| Neutral switch | D5 | X304 pin 6 | P02.3 (input, pull-up) | switch to GND, CLOSED when the gearbox is in neutral |

(The Arduino SPI pins D11-D13 stay free; the SD card uses the mikroBUS socket.)

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

## 4. Ground-station link (PC)

Nothing to wire. Use the board's **USB cable**: the on-board debugger
(miniWiggler) also provides a **virtual COM port**, connected to ASCLIN0
(P14.0 = TX, P14.1 = RX), at **921600 baud, 8N1**. In Windows Device Manager
it shows as "USB Serial Port (COMx)".

Later, a Raspberry Pi or an ESP32 can use the same protocol over a 3.3 V UART.
Do not use D0/D1 (P15.3/P15.2) at the same time, because they are an
alternative pin pair of the same ASCLIN0.
