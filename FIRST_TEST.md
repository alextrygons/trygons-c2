# FIRST TEST - step by step

The first time the real board drives the real steering. Work through it **in
order**, tick every box, and **stop at the first box that fails** (write down
what you saw; fix it before going on). Wiring details: WIRING.md. Pins:
hal/hal_pins.h.

What this test is: **steering only**. The throttle and gear servos are held at
idle / neutral by the software; a person drives the engine by hand. Takeover
is always: **the STANDBY button** and **the emergency stop**.

Profile: the first test runs in the **CREWED** profile (someone on board). The
FIRST TEST build is always CREWED, whatever is plugged in - so the UNCREWED
lost-link engine kill is never used in this test.

You need: a multimeter, the laptop with the ground station, the USB cable,
(for part 4) an oscilloscope or a servo tester, (for part 7) a second person.

---

## 0. Build the FIRST TEST firmware

- [ ] In VS Code / ADS open `Cpu0_Main.c` and remove the comment marks around
      `#define FIRST_TEST` so the line reads exactly `#define FIRST_TEST`.
- [ ] In `hal/hal_pins.h`: `HAL_ESTOP_FITTED 1` (the e-stop contact is wired),
      `HAL_LIMITS_FITTED 1`, `HAL_NEUTRAL_SW_FITTED 0`.
- [ ] Check every pin marked CHECK in `hal/hal_pins.h` against the board's
      silk screen / the Lite Kit user manual. Change the file, not the wiring,
      if a pin is different.
- [ ] Run the test gate in Git Bash: `sh tools/check.sh` -> ends with `GATE: OK`.
- [ ] ADS: press F5 on the project, then *Project -> Clean*, then *Build*.
      No errors.

Not stored yet (no data flash driver): a TRAINING result is **lost at
power-off** (do TRAINING again after every power-up before HDG LOCK) and the
boot number is always 1 (log files get -2, -3 ... added).

## 1. Wiring checks with the multimeter (everything UNPOWERED)

Battery disconnected, board not on USB.

- [ ] **No short** between 12 V and GND at the G2 motor driver input (> 1 kOhm
      after the capacitor has charged from the meter).
- [ ] **No short** between 3.3 V and GND on the board (pin header 3V3 - GND):
      more than ~100 Ohm.
- [ ] **20 A fuse** in the G2 supply, **1 A fuse** for the buzzer, fuses for the
      servo supply are fitted and the right size.
- [ ] **Common ground**: board GND to G2 logic GND, GPS GND, BNO085 GND: < 1 Ohm.
- [ ] **Emergency stop power contact**: released = continuity through it
      (relay coil circuit closed), pressed = open.
- [ ] **Emergency stop signal contact** (P33.0 to GND, measured at the board
      connector): released = < 1 Ohm, pressed = open.
- [ ] **Limit switches** (each, at the board connector, to GND): steering in
      the middle = < 1 Ohm (normally closed); move the outboard by hand to the
      end stop = open. Left switch on the LEFT input, right on the RIGHT input.
- [ ] **Buttons** (each, at the board connector, to GND): released = open,
      pressed = < 1 Ohm.
- [ ] **Pull-downs** 10 kOhm from D9 (throttle), D8 (gear), D6 (motor PWM) and
      D10 (relay) to GND: about 10 kOhm each.
- [ ] **Nothing 5 V** on a board pin: the BNO085, GPS and microSD click are on
      3.3 V (check VIN of each module goes to 3V3, not 5V).
- [ ] Steering motor wires on G2 OUTA / OUTB, not on VIN.

## 2. Power-up order

1. [ ] Emergency stop **pressed**. Throttle lever by hand at idle, gear in
       neutral, engine OFF.
2. [ ] microSD card (FAT32) in the click.
3. [ ] Board on USB (laptop): LED and power LEDs on. Measure 3.3 V on the
       3V3 pin: 3.2-3.4 V.
4. [ ] Start the ground station on the laptop, connect to the board's COM
       port (MANUAL chapter 5). Telemetry runs; the magenta banner
       **FIRST TEST - steering only** is shown; mode STANDBY.
5. [ ] The screen shows the alarm **EMERGENCY STOP** (it is pressed).
6. [ ] Connect the 12 V battery (actuator side). Nothing moves.
7. [ ] Release the emergency stop (twist). Press STANDBY on the panel: the
       alarm clears. The relay closes once the outputs arm (you hear it).

## 3. Bench test (boat on the trailer / dock, engine OFF)

Ground station, tab **Bench**, button **Enter BENCH TEST** (only from STANDBY,
only from the laptop on the USB cable, not when the boat moves). The mode
shows BENCH TEST. Throttle and gear stay at idle / neutral the whole time.
Every 10 minutes without a bench command the board goes back to STANDBY.

### 3a. Every sensor live

- [ ] **IMU**: "IMU ok", reports counting up. Turn the boat (or the box with
      the BNO085) slowly **clockwise seen from above**: the heading goes up,
      the yaw rate is positive. Lift the bow side: pitch positive. Push the
      starboard side down: roll positive. (Wrong sign: flip `BNO_..._SIGN` in
      hal/hal_pins.h, rebuild.)
- [ ] **Compass calibration**: drive / turn slowly through two full circles;
      the accuracy goes to 2 or 3. Compare the heading with a hand-bearing
      compass: within 5 deg (else set `BNO_HEADING_OFFSET`).
- [ ] **GPS** (antenna outside): "GPS ok", UBX yes, fix 3D within ~1 min,
      satellites > 6, SOG < 0.5 kn at rest, spoofing "none", jamming "ok".
- [ ] **SD card**: "card yes, mounted yes", free space shown.
- [ ] **Motor current**: about 0 A with the motor stopped.

### 3b. Steering motor in small pulses

Outboard roughly in the middle, nobody near the steering.

- [ ] **Pulse LEFT 100 ms**: the outboard moves a little to port (the first
      pulse after a direction change may only take up the play in the
      steering: then the estimate stays the same - do a few more). The
      estimated angle goes negative. Current shows a value while it runs.
- [ ] **Pulse RIGHT 100 ms**: moves back to starboard.
- [ ] If LEFT moves it to STARBOARD: swap the motor wires on OUTA / OUTB.
- [ ] Pulses of 200 ms and 300 ms: larger steps, never more than 300 ms.
- [ ] Repeated quick clicks: at most one pulse per 0.2 s (the rest are
      refused, shown on the screen).

### 3c. Limit switches

- [ ] Move the outboard by hand (motor off) to the LEFT end stop: "LIMIT L"
      lights. **Pulse LEFT** is refused ("at the left end stop"); **Pulse
      RIGHT** works and LIMIT L goes out.
- [ ] Same on the RIGHT end.
- [ ] Unplug one limit switch wire: its light comes on (a broken wire = at
      the stop). Plug it back.

### 3d. Each button

Press each panel button once; its box on the screen turns green with a count:

- [ ] HDG LOCK  - [ ] -1  - [ ] +1  - [ ] -10  - [ ] +10
      (none of them changes the mode in the bench test)
- [ ] **STANDBY last**: the mode goes to **STANDBY** at once (the STANDBY
      button always wins). Enter BENCH TEST again for the next steps.

### 3e. Buzzer

- [ ] **Buzzer test 1 s**: the buzzer sounds and LED1 lights for 1 s, then
      both stop.

### 3f. Emergency stop

- [ ] Press the emergency stop: the mode goes to **STANDBY** within 0.1 s, the
      alarm EMERGENCY STOP shows, the buzzer sounds, the relay drops, the
      G2 has no power (measure VIN: 0 V).
- [ ] While pressed: Enter BENCH TEST is refused.
- [ ] Release, press STANDBY: the alarm clears.

### 3g. Servo PWM outputs (R3) - scope or servo tester

Servos disconnected (or the engine OFF and the linkages free).

- [ ] Scope on D9 (throttle) to GND: 50 Hz (period 20 ms), pulse width =
      the idle value shown on the Bench tab (about 1000 us), 3.3 V high.
- [ ] Scope on D8 (gear): 50 Hz, the neutral pulse (about 1500 us).
- [ ] Or a servo tester in "measure" mode / a servo on the output: it goes
      to idle / neutral and stays there.
- [ ] **Monitor trip test** (last, needs a power cycle afterwards): press
      *Monitor trip test* and confirm. CPU1 trips: alarm MONITOR TRIP, mode
      STANDBY, buzzer on, relay open, G2 asleep (SLP low). The scope still
      shows the idle / neutral pulses on D9 / D8 (they come from the monitor
      itself now). Nothing can switch it back: **power the board off and on**.

### 3h. Reset reason + relay board (only once the RS485 relay board is wired, HAL_RELAY_FITTED 1)

Engine OFF, nothing connected to the relay outputs except a test lamp on a relay whose boat file
entry has `"power_up": "on"` (e.g. NAV LIGHTS).

- [ ] Power the board off and on: the Events tab shows the start as POWER-ON, the test lamp comes on
      (power_up, once).
- [ ] Switch the lamp OFF from the ground station. Then make a REAL watchdog reset: in ADS halt core 0
      with the debugger for more than 1 s and let it run again (or disconnect the debugger while it is
      halted) - the chip resets by the watchdog. Events tab: the start is NOT a power-on (SYSTEM /
      watchdog), and the lamp stays OFF (no power_up after a watchdog reset). If the lamp comes on: STOP,
      the reset reason is read wrong (hal_reset_reason / tc_rstcause.h) - report it.
- [ ] Do it twice in a row (the cold-reset bits are cleared after the first boot: the second watchdog
      reset must also read as SYSTEM, not as a power-on).
- [ ] RS485 driver enable (scope on PIN_RELAY_DE, P00.11, and on the RS485 A/B lines): DE goes high
      just before the request and low right after its LAST stop bit (well under 1 ms at 9600 baud - not
      one control step of 50 ms later); the board's reply is seen on the bus after DE is low.

### 3i. ENGINE KILL latch through a reset (later: normal build, UNCREWED, kill relay fitted - not in FIRST TEST)

The FIRST TEST build never kills the engine, so this check waits for the first UNCREWED test with the
kill relay wired (engine OFF, a test lamp on the kill relay output).

- [ ] Make a lost-link kill (UNCREWED, HELM station disconnected): ENGINE KILLED, lamp on.
- [ ] Make a REAL watchdog reset (halt core 0 in ADS > 1 s, as in 3h): after the restart the boat is
      STILL killed (ENGINE KILLED alarm, lamp on). If the kill is gone: STOP - the no-clear RAM of
      hal_kill_keep_set / _get (hal_tc375.c, DSPR0) is cleared by the startup code - report it.
- [ ] RESTART from the HELM station (idle, neutral, e-stop released): the kill goes (lamp off).
- [ ] Kill again, then a real power cycle of the board: the kill goes (power-on).

### 3j. DroneCAN servos (later: from step 11e, HAL_CAN_FITTED 1 - owner decisions 2026-10-28)

Engine OFF, the DroneCAN GUI Tool on a laptop directly on the bus (WIRING.md 3E). Write every
measured value / answer here.

- [ ] Pixhawk UNPLUGGED from the CAN bus (it is node 10 too, like the AURIX: never both on the
      bus - OA-4).
- [ ] Node command timeouts, every node with `SRV_CMD_TIME_OUT` 0 (AP_Periph has NO "idle on
      timeout" - OA-1): command a position, stop the commands: gear HOLDS (OD-5), steering HOLDS,
      throttle HOLDS.
- [ ] Throttle node with a TEST timeout (e.g. 300 ms), engine OFF: command mid throttle, stop the
      commands, scope the node output. After the timeout: pulses STOP ____ / go to 1500 us
      (= half throttle) ____ / other ____ . Then: does the return spring pull the lever to idle?
      ____ . ONLY "pulses stop AND spring to idle" allows a throttle timeout on the boat (value
      ____ ms, owner signs ________); otherwise the throttle node stays at 0 (hold). Until this
      line is filled in, nothing in the docs or the code may assume "node timeout = idle".
- [ ] Servo speeds: steering lock to lock ____ s, throttle idle to full ____ s. A value shorter
      than 6 s is used for the safety waits ONLY if the owner signs it here (OD-4):
      signed ________ date ______.
- [ ] **Throttle return spring vs the 360 deg position servo (OD-11):** servo node unpowered (and
      powered without pulses): does the spring pull the throttle back to idle? Answer ____ . If NOT,
      the engine stop (kill cord, e-stop kill contact, kill circuit) is the only backstop - note it
      in WIRING.md 3 and PARTS.md.
- [ ] Power-on with the outboard turned off-centre: the steering does NOT move until a mode
      commands steering (OD-7). STANDBY: the steering holds (OD-10).
- [ ] **Throttle idle on the wire (step 11g):** the AURIX sends idle as -1.0 (= 1000 us) - in its
      boot frame, every idle step and CPU1's trip frames; servo.idle_us must stay 1000 with the
      servos on CAN (otherwise the outputs never arm: NOT ARMED). Engine OFF: with -1.0 on the
      wire the lever sits exactly at idle ____ (any offset / reversal is set IN THE NODE, never in
      the AURIX calibration). Also: STANDBY press during a planned arrival stop -> the throttle
      command goes to -1.0 at once ____ .
- [ ] E-stop: pressed = the engine kill circuit opens (test lamp), CREWED and UNCREWED (OD-1); the
      servo nodes stay powered (OD-2).
- [ ] UNCREWED monitor trip test: P10.5 opens the kill circuit (test lamp); CREWED: it does not (OD-2).
- [ ] UNCREWED dead-man (OA-3, once the timer relay is fitted - part TO CONFIRM): UNCREWED, stop
      the AURIX's pulses (halt core 0 / CPU1 in ADS, or pull the pulse wire): the kill circuit
      opens by itself after ____ s (~1.5 s). A real watchdog reset alone: killed? ____ . CREWED:
      the dead-man is out of the circuit (no kill without pulses).
- [ ] STARTER time-limit relay (OD-9): hold START on the relay output longer than its time: the
      timer relay opens by itself (test lamp off) after ____ s.
- [ ] (step 13b) Monitor trip test with the throttle node unplugged right after the trip, plugged in
      again after > 6 s: the gear node goes to NEUTRAL only ~thr_trav_s AFTER the throttle node is
      back (gear servo arm, stopwatch) ____ s.
- [ ] (step 13b, DroneCAN relay board) SHUTOFF from the Engine page right after a watchdog reset
      (within 1 s): the SHUTOFF relay switches as soon as the board is seen again ____ (never lost);
      with the board unplugged it is NOT switched when the board comes back after > 10 s ____ .
- [ ] (step 13d) Boot trip: halt core 0 in ADS right after the CPU sync (before its first control
      step), wait > 3 s (MON_TRIP_BOOT): scope the throttle / gear PWM = the configured idle /
      neutral pulse ____ us / ____ us (NEVER ~500 us); with the DroneCAN servos: CAN trace shows
      CPU1's throttle IDLE (-1.0) frames every 20 ms ____ , no steering / gear frame ____ .
- [ ] (step 13e, W43) Boot time: core 0's time from reset to its first control step (log / GPIO
      toggle / ADS trace) ____ ms - must be < 1.5 s (CPU1's MON_BOOT_MS 3 s counts from CPU1's
      start, which is ~1 ms after core 0's); also with the SD card out and after a data-flash
      sector swap ____ ms.
- [ ] (step 13f, W43 + review 94afdc4 MED-1) Boot throttle IDLE: CAN trace from reset - core 0's
      FIRST throttle IDLE (-1.0) frame ____ ms after reset (hal_init sends it right after dc_init,
      before the NV boot loop + sb_init), then -1.0 every 10 ms through the NV boot loop ____ ; no
      steering / gear frame before a mode commands one ____ . With the throttle node holding ~50 %
      from before a warm reset (watchdog reset while driving): the lever starts closing within
      ____ ms of the reset (not after the boot).
      (step 13g, review facf265 LOW-2) The -1.0 frames every 10 ms come only WHILE the NV boot loop
      runs (a normal boot-counter write ends it within a few ms; NV FAULT at once): measure the
      LONGEST gap between two core-0 throttle frames from hal_init's frame to the control loop's
      first actuator tick (= the NV loop + sb_init + link_init) ____ ms (normal boot) / ____ ms
      (after a data-flash sector swap) - a frame the node drops (C19) is repeated only after that gap.
- [ ] (step 13f, OPEN ITEM C19, review 94afdc4 LOW-3) Transfer-ID after a boot: CPU1 (and core 0)
      start their ArrayCommand transfer ID at 0 after every reset. If the node's expected ID from
      before the reset happens to be 31, libcanard on the node DROPS the first frame (the next one,
      10 / 20 ms later, is taken). Check on the bench that the node takes the frames after a warm
      reset at different moments (CAN trace + the servo moves to idle) ____ ; the cost is one
      period (10-20 ms) - NOT changed in code, owner decision if the bench shows more. (step 13g:
      right after a reset the next core-0 frame may come only after the boot gap measured above,
      not 10 ms later - note it here too: ____ ms.)
- [ ] (step 13e, review 1f7817c HIGH-1) Boot trip with the DroneCAN servos, core 0 halted in ADS
      INSIDE hal_init right after dc_init (not after the sync): CAN trace shows CPU1's throttle
      IDLE (-1.0) frames every 20 ms after ~3 s ____ ; halted BEFORE dc_init: no CAN frame at all
      (the controller is not started), throttle / gear pins low ____ .
- [ ] (step 13d, review 49353b2 LOW-3) CAN trace at a monitor trip while driving (bench trip test
      with the servos on CAN): count core 0's ArrayCommands after the trip - at most ONE (already
      in transmission, TXBCR can't pull it back) ____ , CPU1's idle frame within 20 ms ____ .
- [ ] (step 13d, DroneCAN relay board) SHUTOFF from the Engine page, then pull the relay board's
      CAN cable within ~0.1 s (or power it off): plugged in again within 10 s, the SHUTOFF relay
      is ON ____ and was never switched OFF in between ____ .
- [ ] (step 13h, W42, DroneCAN relay board) NAV LIGHTS (or another on/off EXTRA relay, a test lamp)
      ON, then pull the relay board's CAN cable within ~0.1 s: (a) plugged in again within 10 s: the
      relay ends ON ____ and was never switched OFF by the AURIX ____ ; (b) plugged in after > 10 s:
      NOTHING is sent (CAN trace: no hardpoint.Command) ____ , the panel shows it unknown ____ ;
      (c) power-cycle the board while unplugged, plug in within 10 s: nothing sent, the relay OFF
      (the board's own boot state) ____ ; (d) the STARTER: START hold + cable pulled: never ON again
      when plugged back ____ ; (e) (step 13h2) power the board up, switch the relay ON within 2 s of
      the board answering, pull the cable, power-cycle the board, plug in after ~5 s: nothing sent,
      the relay OFF, the panel shows it unknown ____ .
- [ ] (step 13k, W44, DroneCAN relay board) AP_Periph NodeStatus TX deadline / late delivery: CAN
      trace (DroneCAN GUI Tool, timestamps) while the board's CAN cable is pulled for ~5 s and plugged
      back: is a NodeStatus queued during the outage delivered late (its uptime older than the bus
      time)? how late ____ ms (the AURIX waits for a SECOND status before it calls such a lag a
      reboot: a delay up to ~1 s is expected) ; are two queued statuses ever delivered out of order
      (an uptime 1 s lower after a higher one) ____ ; the NodeStatus period ____ ms and its jitter
      ____ ms. Then NAV LIGHTS ON, cable pulled within ~0.1 s, plugged in after ~4 s: the relay ends
      ON ____ (finished ~1 s after the board answers, never OFF in between) ____ .

- [ ] Leave the bench test (button **Leave**): mode STANDBY.

## 4. Dock test (boat in the water, tied up, engine running in NEUTRAL)

Second person at the engine (throttle and gear by hand, kill cord worn).

- [ ] Engine started in neutral. The board stays in STANDBY, throttle /
      gear servos don't move (they are held at idle / neutral).
- [ ] Bench test again: the IMU heading is steady (engine vibration: the
      heading moves less than 2 deg), GPS ok, motor pulses LEFT / RIGHT move
      the outboard. Leave the bench test.
- [ ] Press the emergency stop: STANDBY + alarm, steering motor without
      power. (The engine keeps running: the e-stop cuts only the actuator
      power unless it is wired to the kill circuit.) Release + STANDBY.
- [ ] Unplug the USB cable for 5 s: the ground station shows the link lost;
      plug it back: telemetry returns, mode still STANDBY.

## 5. First on-water steering test

Calm water (wind < 10 kn, waves < 0.3 m), daylight, plenty of free space, no
other boats close. Two people: **the driver at the throttle** (by hand,
engine controls as normal, kill cord worn, hand near the STANDBY button) and
**the operator** with the laptop.

### 5a. TRAINING (outboard calibration)

1. [ ] Outboard **centred by hand** (straight ahead), mode STANDBY.
2. [ ] Ground station: TRAINING *Start*. The board waits.
3. [ ] The driver puts it in forward and drives slowly straight ahead
       (~3-4 kn; it needs SOG >= 2 kn and the course within 60 deg of the
       heading - "moving ahead" on the screen).
4. [ ] The steering moves in steps by itself; the boat turns a little each
       way. **Driver: if anything looks wrong - STANDBY button.**
5. [ ] TRAINING shows a result: check it is plausible (MANUAL chapter 6A),
       *Accept*.
6. [ ] If the boat stops or turns away from its course, TRAINING pauses
       (the steering stops) - that is correct.

### 5b. HEADING LOCK

1. [ ] Driving slowly straight ahead (3-4 kn), press **HDG LOCK** on the
       panel: the boat holds the heading. Hands off the wheel / tiller.
2. [ ] **+10** on the panel: the boat turns 10 deg to starboard, smoothly,
       and holds the new heading. **-10**: back.
3. [ ] **+1 / -1**: small corrections.
4. [ ] Driver slows to idle and neutral: the steering stops ("not moving
       ahead"), the mode stays HDG LOCK; speed up again: it steers again.
5. [ ] **STANDBY button**: steering off at once.
6. [ ] HDG LOCK again, then **emergency stop**: STANDBY + alarm, steering
       without power. Release, STANDBY.
7. [ ] Heading error while holding (plot tab): within +/- 3 deg in calm water.

### After the test

- [ ] Copy the log files from the microSD card (in STANDBY): `B00001_...TLG`
      and look at them with `groundstation/logview.py`.
- [ ] Write down what you saw (heading error, any alarm, the TRAINING result)
      for the next session.
- [ ] Before the normal (throttle) configuration: comment `#define FIRST_TEST`
      out again, rebuild, and do the full neutral-start / joystick checks
      (WIRING.md section 3, MANUAL).
