# Trygons C2 - parts to buy

One list for everything. Tick in your own copy. Details and wiring: WIRING.md;
why each part is needed: MANUAL.txt.

## Boat: controller and actuators

- [ ] Joystick with 2 potentiometer axes (10 k) and THREE buttons (TAKE, AUTO, CONFIRM - CONFIRM clearly separate from TAKE), marine grade; 4 x 1 kOhm (end resistors), 2 x 100 kOhm (pull-downs), 2 x 100 nF.
- [ ] Panel: 6 x IP67 push buttons (one big red STANDBY, one HDG LOCK, -1, +1, -10, +10), cable.
- [ ] Neutral switch on the gearbox / shift linkage (closes in neutral). Also required by GENERATOR mode (the engine charges the bank in neutral only while this switch confirms neutral).
- [ ] microSD click (MIKROE-924) + industrial microSD card 8-32 GB.
- [ ] Throttle servo (analog, or digital with "fail-safe = idle") + a throttle return spring; gear servo.
- [ ] Steering motor driver: Pololu G2 High-Power Motor Driver 18v17 (#2991), 20 A blade fuse + holder, 100 uF / 35 V low-ESR capacitor, 2.5 mm2 wire.
- [ ] Safety relay module (3.3 V input via transistor driver) for the servo power and/or the engine kill circuit.
- [ ] 10 kOhm pull-down resistors for the servo / driver / relay signals.
- [ ] GPS: u-blox MAX-M10S breakout, 3.3 V (e.g. SparkFun GPS-18037 or a similar MAX-M10S board) + active GPS patch antenna (SMA / u.FL as the board needs) - spoofing / jamming flags (WIRING.md section 11).
- [ ] Compass + gyro: the BNO085 (see "IMU" below).
- [ ] 2 x normally closed IP67 micro limit switches (steering end stops), 2 x 4.7 kOhm, 2 x 100 nF.
- [ ] Emergency stop: red mushroom switch with TWO normally closed contacts (twist release), IP65 enclosure; 30 A automotive relay (12 V coil) + socket for the actuator power; 4.7 kOhm + 100 nF.
- [ ] Buzzer: 12 V active piezo buzzer > 85 dB, BC337 (or 2N7000 / AO3400), 1 kOhm, 1 A fuse.
- [ ] Optional: paddlewheel speed sensor.
- [ ] Supply voltage divider for A2 (WIRING.md 3B): 47 kOhm + 10 kOhm (1 %), 100 nF, 1 A fuse in the sense wire; optional 3.3 V clamp on the pin - RATIO TO MEASURE, PIN TO CHECK.
- [ ] Battery bank: 3 x 12 V in parallel (one bank for everything) + a battery isolator / split-charge relay that cuts the starter battery from the bank only while cranking - TYPE TBD (decide with the engine dealer).
- [ ] Keep the engine kill cord fitted and worn.
- [ ] ENGINE KILL relay for the UNCREWED lost-link rule (WIRING.md 3A): relay + transistor driver + fuse in the outboard kill-switch circuit - TYPE, PIN AND FAIL-SAFE DIRECTION TBD (decide with the engine dealer). (The remote RESTART clears it in software - no extra part.)
- [ ] Tacho / rpm input for the remote START (CLAUDE.md N: "engine running" detection, today time limit only) - TBD.
- [ ] RS485 Modbus RTU relay board, 8-16 channels, 12 V, with board-timed pulses ("flash on"), + a 3.3 V RS485 transceiver module (MAX3485 class), 2 x 120 Ohm termination, twisted pair cable, a fuse per load (WIRING.md 3D) - BOARD, BAUD RATE AND PINS TBD.

## Gateway (remote access)

- [ ] Raspberry Pi 5, 8 GB, with the official active cooler - REQUIRED for the camera obstacle
      detection (eWaSR on the CPU; without the cooler the Pi throttles and vision pauses).
- [ ] Later, optional (not implemented): Raspberry Pi AI HAT+ (Hailo NPU) or a Luxonis OAK-D camera
      to run the obstacle detection many times faster (gateway/vision/README.md).
- [ ] 12 V -> 5.1 V 5 A USB-C supply for the Pi (car/marine grade; 5 A is needed so the Pi can power the SSD), 5 A fuse on the 12 V side.
- [ ] USB 3 SSD 512 GB - 1 TB (e.g. Samsung T7) - logs and video (4 cameras ~3-4 GB per hour; 1 TB if you use the 60-minute ring buffer and SAVE often).
- [ ] microSD card 32 GB (A2) - only the Pi's operating system.
- [ ] UART cable AURIX <-> Pi: 3 wires (TX, RX, GND), shielded, < 1 m, 2 x 100 Ohm resistors.
- [ ] IP65 box with cable glands for the Pi + SSD, ventilated / heat sink.

## Cameras (you already have the RUT901 router and the TSW101 PoE switch)

- [ ] 4 x TP-Link Tapo C310 (FRONT-1, FRONT-2, REAR-LEFT, REAR-RIGHT).
- [ ] 3 x 802.3af PoE splitter with 9 V output (for FRONT-1 and the two rear cameras; check the C310 plug size).
- [ ] 1 x 12 V -> 9 V DC-DC converter >= 1 A (FRONT-2, its own power).
- [ ] Fuses + holders: 5 A (TSW101), 2 A (FRONT-2 converter).
- [ ] USB 3 Gigabit Ethernet adapter for the Pi (camera network eth1).
- [ ] Outdoor / UV-resistant Cat6 cable, IP68 cable glands, waterproof RJ45 couplers or self-amalgamating tape, a closed box for splitters.
- [ ] 4 x high-endurance microSD 64-128 GB (the cameras' own loop recording).
- [ ] Microfibre cloths (lens care).

## IMU (compass, gyro, attitude, slamming)

- [ ] BNO085 breakout board, 3.3 V, SPI (e.g. Adafruit 4754 or SparkFun SEN-22857) - Stage 1: it is the compass and the gyro.
- [ ] 4-pair shielded cable (SPI + INT + RST), max ~1 m to the AURIX box, a small IP67 box, M3 nylon standoffs.

## USB joystick (instead of, or as well as, the analog joystick)

- [ ] Thrustmaster T.16000M FCS HOTAS (stick + TWCS throttle). NOT waterproof: dry console.
- [ ] Teensy 4.1 (PJRC) + its USB host cable (5-pin header to USB-A socket).
- [ ] Small powered USB 2.0 hub (4 ports, with its own 5 V input).
- [ ] 12 V -> 5 V DC-DC converter, at least 3 A (marine / automotive grade), 2 A fuse; or a separate 5 V supply of at least 2 A.
- [ ] Shielded 2-core cable (TX + GND) to the AURIX, 100 Ohm resistor.
- [ ] Closed box with cable glands + desiccant for the hub and Teensy, P-clips for cable strain relief.

## AIS (receive only)

- [ ] dAISy HAT AIS receiver for the Raspberry Pi (Wegmatt) - or a USB AIS receiver (e.g. dAISy USB, Quark-Elec QK-A026).
- [ ] VHF/AIS antenna (marine, 162 MHz, e.g. a 1 m stainless whip) + mount, or a passive AIS/VHF splitter (e.g. Digital Yacht SPL1500) for the existing VHF antenna.
- [ ] Coax: RG-58 (up to 5 m) or RG-8X / LMR-240 (longer) with the right connectors (BNC / SMA / PL-259), self-amalgamating tape.
- [ ] If the dAISy HAT takes the Pi's serial port: a USB-UART adapter (3.3 V, FTDI / CP2102) for the AURIX link.
- [ ] Radar reflector (we are NOT visible on AIS): e.g. Echomax EM230 or Tri-Lens, mounted high.

## Shore PC

- [ ] A second monitor for the DRIVER screen (MANUAL chapter 9 part I).
- [ ] USB joystick for remote driving (Thrustmaster T.16000M + TWCS recommended).
- [ ] Tailscale account (free) for the VPN.
