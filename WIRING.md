# Wiring: joystick and ground-station link (KIT_A2G_TC375_LITE)

Source: *AURIX TC375 lite Kit V2 user manual*, Figure 7 "Mapping of Arduino
Functions to AURIX Pin Functions". **Always check the pin labels printed on
the board before you connect anything.**

> **3.3 V only.** The board's Arduino pins are 3.3 V logic and the analog
> inputs must never be higher than VAREF = 3.3 V. Do NOT power the joystick
> from 5 V: this can damage the TC375.

## 1. Joystick (2 potentiometers + 2 push buttons)

| Joystick wire          | Board pin (Arduino name) | Connector / pin | TC375 pin           |
|------------------------|--------------------------|-----------------|---------------------|
| X pot, wiper (middle)  | A0 (ADC0)                | X301 pin 1      | P40.9 = AN39 (VADC G4.7) |
| Y pot, wiper (middle)  | A1 (ADC1)                | X301 pin 2      | P40.8 = AN38 (VADC G4.6) |
| Pot "+" ends (both)    | 3V3 (through 1 k, see below) | X302 pin 4  | +3.3 V              |
| Pot "-" ends (both)    | GND (through 1 k, see below) | X302 pin 6 (or 7) | GND           |
| TAKE button            | D2                       | X304 pin 3      | P02.0 (input, internal pull-up) |
| AUTO button            | D3                       | X304 pin 4      | P02.1 (input, internal pull-up) |
| Both buttons, other side | GND                    | X302 pin 6/7    | GND                 |

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

TAKE and AUTO are normally-open push buttons. Each one goes from the input pin
to GND, and the TC375's internal pull-up keeps the pin high when the button is
released, so pressed = low. A broken button wire only means that button stops
working. It can never make the joystick take control.

### What happens on the boat

* **At power-up** the stick must be **centred** (within 10 % for 0.2 s) before
  it is armed. If someone holds it deflected, you get the alarm "joystick not
  centred" and the stick is ignored until it is released.
* **Taking control:** push the stick past 15 % for more than 0.1 s, or press
  **TAKE**. This works from every mode. You hear a chirp, and the mode becomes
  JOYSTICK.
* **In JOYSTICK:** X steers the outboard over its full range. Y is throttle:
  up = ahead, down = astern, centre = idle and neutral. Gear changes always go
  through idle → neutral → gear. The throttle limit is 40 % ahead (configurable)
  and the reverse limit is 30 %.
* **Handing back:** centre the stick, then press **AUTO** → STANDBY. If the stick
  is not centred, AUTO is refused with a chirp. The PC and the Standby button
  can never take control from the joystick.
* **Joystick fault while in JOYSTICK** → throttle to idle, then neutral, steering
  stops, STANDBY and alarm.

### Centre calibration

With the boat in STANDBY:
1. Start calibration (ground station: Config → "Joystick calibrate start").
2. Move the stick slowly to all four ends, then let it spring back to centre.
3. Finish calibration. The new min/centre/max values are saved to flash
   with a CRC-32 checksum. If they are implausible (stick not moved, centre
   too close to an end), the calibration is refused and the old one stays in
   use. A corrupted calibration in flash is detected at start-up and the
   defaults are used instead.

## 2. Ground-station link (PC)

Nothing to wire. Use the board's **USB cable**: the on-board debugger
(miniWiggler) also provides a **virtual COM port**, connected to ASCLIN0
(P14.0 = TX, P14.1 = RX), at **921600 baud, 8N1**. In Windows Device Manager
it shows as "USB Serial Port (COMx)".

Later, a Raspberry Pi or an ESP32 can use the same protocol over a 3.3 V UART.
Do not use D0/D1 (P15.3/P15.2) at the same time, because they are an
alternative pin pair of the same ASCLIN0.
