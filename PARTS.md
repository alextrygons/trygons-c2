# Trygons C2 - parts to buy

One list for everything. Tick in your own copy. Details and wiring: WIRING.md;
why each part is needed: MANUAL.txt.

## Boat: controller and actuators

- [ ] Joystick with 2 potentiometer axes (10 k) and THREE buttons (TAKE, AUTO, CONFIRM - CONFIRM clearly separate from TAKE), marine grade; 4 x 1 kOhm (end resistors), 2 x 100 kOhm (pull-downs), 2 x 100 nF.
- [ ] Panel: 6 x IP67 push buttons (one big red STANDBY, one HDG LOCK, -1, +1, -10, +10), cable.
- [ ] Neutral switch on the gearbox / shift linkage (closes in neutral). Also required by GENERATOR mode (the engine charges the bank in neutral only while this switch confirms neutral).
- [ ] microSD click (MIKROE-924) + industrial microSD card 8-32 GB.
- [ ] Throttle servo (analog, or digital with "fail-safe = idle") + a throttle return spring; gear servo. (From step 11e the small boat uses the DroneCAN servos below; whether the return spring can back-drive the position servo is decided on the bench: owner decision OD-11, FIRST_TEST.md 3j.)
- [ ] Steering motor driver: Pololu G2 High-Power Motor Driver 18v17 (#2991), 20 A blade fuse + holder, 100 uF / 35 V low-ESR capacitor, 2.5 mm2 wire.
- [ ] Safety relay module (3.3 V input via transistor driver) for the servo power and/or the engine kill circuit. (From step 11e, owner decision OD-2: ONLY in the engine kill circuit, driven by P10.5, used in UNCREWED only - it never cuts the servo power.)
- [ ] 10 kOhm pull-down resistors for the servo / driver / relay signals.
- [ ] GPS: u-blox MAX-M10S breakout, 3.3 V (e.g. SparkFun GPS-18037 or a similar MAX-M10S board) + active GPS patch antenna (SMA / u.FL as the board needs) - spoofing / jamming flags (WIRING.md section 11).
- [ ] Compass + gyro: the BNO085 (see "IMU" below).
- [ ] 2 x normally closed IP67 micro limit switches (steering end stops), 2 x 4.7 kOhm, 2 x 100 nF.
- [ ] Emergency stop: red mushroom switch with TWO normally closed contacts (twist release), IP65 enclosure; 30 A automotive relay (12 V coil) + socket for the actuator power; 4.7 kOhm + 100 nF. (From step 11e, owner decisions OD-1 / OD-2: the e-stop also opens the engine kill circuit in every profile - a switch with a THIRD NC contact or an extra relay, TBD with the engine dealer - and it no longer feeds the DroneCAN servo power.)
- [ ] Buzzer: 12 V active piezo buzzer > 85 dB, BC337 (or 2N7000 / AO3400), 1 kOhm, 1 A fuse.
- [ ] Optional: paddlewheel speed sensor.
- [ ] Supply voltage divider for A2 (WIRING.md 3B): 47 kOhm + 10 kOhm (1 %), 100 nF, 1 A fuse in the sense wire; optional 3.3 V clamp on the pin - RATIO TO MEASURE, PIN TO CHECK.
- [ ] STARTER hardware time-limit relay (owner decision OD-9): a 12 V timer relay ("single shot" / "interval" type) in the starter-solenoid control wire, opens after a fixed time a little above the software 5 s (proposal 6 s, TO CONFIRM), resets only when START goes away; on either relay bus (Modbus / DroneCAN) - TYPE TBD (WIRING.md 3D).
- [ ] Battery bank: 3 x 12 V in parallel (one bank for everything) + a battery isolator / split-charge relay that cuts the starter battery from the bank only while cranking - TYPE TBD (decide with the engine dealer).
- [ ] Keep the engine kill cord fitted and worn.
- [ ] ENGINE KILL relay for the UNCREWED lost-link rule (WIRING.md 3A): relay + transistor driver + fuse in the outboard kill-switch circuit - TYPE, PIN AND FAIL-SAFE DIRECTION TBD (decide with the engine dealer; a reset / dead board must never kill a CREWED engine). (The remote RESTART clears it in software - no extra part.)
- [ ] UNCREWED dead-man (owner addition OA-3, WIRING.md 3A, docs/dronecan_design.md 14.1): a 12 V retriggerable timer relay ("watchdog" / missing-pulse relay) in series in the kill circuit, held "run" by an AURIX pulse, kill after ~1.5 s without pulses; armed only in UNCREWED (recommended: an on-board UNCREWED switch / key that puts it in the circuit, read by the AURIX) + transistor driver - PART, TIME, CONTACT TYPE, FAIL-SAFE DIRECTION, ARMING AND PIN TO CONFIRM.
- [ ] Tacho / rpm input for the remote START (CLAUDE.md N: "engine running" detection, today time limit only) - TBD.
- [ ] RS485 Modbus RTU relay board, 8-16 channels, 12 V, with board-timed pulses ("flash on"), + a 3.3 V RS485 transceiver module (MAX3485 class), 2 x 120 Ohm termination, twisted pair cable, a fuse per load (WIRING.md 3D) - BOARD, BAUD RATE AND PINS TBD.
- [ ] DP thrusters (CLAUDE.md P, WIRING.md 3C): a small 12 V bow TUNNEL thruster (sideways) + a small fixed 12 V transom electric motor (fore / aft), each with a bidirectional controller with an RC-style input (1500 us = stop, "no pulses = stop" fail-safe), a fuse per thruster, a contactor in the thruster feed whose coil goes through the e-stop's power contact, 2 x 10 kOhm pull-downs - MODELS, CONTROLLERS, CONTACTOR RATING AND PINS TBD.

- [ ] DroneCAN (CLAUDE.md T, WIRING.md 3E, docs/dronecan_design.md - the AURIX replaces the ArduPilot): the existing DroneCAN servo nodes (Matek CAN-L431, throttle / steering 360 deg position servo / gear) and the DroneCAN relay board are REUSED - MODELS, NODE IDS AND TIMEOUT SETTINGS TO CONFIRM (the relay board on DroneCAN, step 11f: node 30, relays 1-6 = hardpoints 0-5; it must hold its non-momentary relays and switch its momentary ones (TRIM, STARTER) off <= 300 ms after the commands stop - requirements R-1..R-3 of WIRING.md 3E; otherwise the RS485 Modbus board below stays the relay board, the build option RELAY_BUS). New: a 12 V -> 5 V DC-DC converter for the nodes (they were powered through the Pixhawk's CAN port; rating TO CONFIRM by measuring the nodes' current) + a fuse at the bank end and one per branch; CAN cable (twisted pair, JST-GH or as the nodes need) from the kit's CAN connector (pinout TO CONFIRM); 2 x 120 Ohm terminations if not on the kit / the last node (TO CONFIRM); a USB-CAN adapter for the laptop (for the DroneCAN GUI Tool: set static node IDs, timeouts, bench tests). The Pixhawk is NOT reused: it is node 10 like the AURIX and must never be on the bus with it (OA-4).

## Gateway PC (remote access) - Lenovo ThinkPad L390 Yoga (gateway/README.md, WIRING.md 7)

- [x] Lenovo ThinkPad L390 Yoga: i5-8265U (4 cores / 8 threads), 8 GB RAM, 500 GB SSD (you have it). Ubuntu 24.04 LTS (recommended at sea) or Windows 11. Its own SSD holds the logs and video (~3-4 GB per hour). Vision runs on its CPU (no NVIDIA); measured load: gateway/README.md section 21.
- [ ] 12 V -> USB-C PD car charger, 65 W (USB-C PD 20 V, car/marine grade) + a 5 A-rated USB-C cable, fuse at the bank per the charger's manual - MODEL TBD. The laptop's built-in battery is the UPS; charge limit 80 % (TLP on Ubuntu / Lenovo Vantage on Windows).
- [ ] 3.3 V TTL USB-UART adapter for the AURIX link: FTDI TTL-232R-3V3 cable (or an FT232R / FT231X / CP2102(N) board set to 3.3 V, 921600 baud) - NEVER RS232 (+-12 V destroys the AURIX pins), never 5 V TTL. Mounted inside the AURIX box, 2 x 100 Ohm resistors, shielded USB cable (3-5 m) with a ferrite to the PC.
- [ ] USB 3 Gigabit Ethernet adapters (Realtek RTL8153 class) for the boat LAN and / or the camera network, as many as the laptop lacks RJ45 ports (one per network) - RJ45 / port count TBD (WAITING FOR OWNER W17).
- [ ] Small powered USB 3 hub - only if the laptop's USB ports run out (adapter, AIS, Ethernet adapters).
- [ ] Optional: external USB 3 SSD 1 TB (e.g. Samsung T7) if the 500 GB SSD is too small for the ring buffer + SAVEs.
- [ ] A dry, ventilated mount for the laptop (not waterproof), held down against vibration; strain relief for every cable.
- [ ] Later, optional (not implemented): a Luxonis OAK-D camera or a USB NPU to run the obstacle detection faster (gateway/vision/README.md).
- [ ] REMOVED with step 12 (not needed any more): Raspberry Pi 5 + active cooler, its 5 V 5 A supply, its microSD card, the Pi UART cable, the dAISy HAT.

## Cameras (you already have the RUT901 router and the TSW101 PoE switch)

- [ ] 4 x TP-Link Tapo C310 (FRONT-1, FRONT-2, REAR-LEFT, REAR-RIGHT).
- [ ] 3 x 802.3af PoE splitter with 9 V output (for FRONT-1 and the two rear cameras; check the C310 plug size).
- [ ] 1 x 12 V -> 9 V DC-DC converter >= 1 A (FRONT-2, its own power).
- [ ] Fuses + holders: 5 A (TSW101), 2 A (FRONT-2 converter).
- [ ] (the camera network port on the gateway PC: see "Gateway PC" above - its RJ45 or a USB 3 Gigabit Ethernet adapter).
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

- [ ] USB AIS receiver, receive only, NMEA 0183 over USB (e.g. dAISy USB or a Quark-Elec QK-A02x class receiver) - MODEL TBD.
- [ ] VHF/AIS antenna (marine, 162 MHz, e.g. a 1 m stainless whip) + mount, or a passive AIS/VHF splitter (e.g. Digital Yacht SPL1500) for the existing VHF antenna.
- [ ] Coax: RG-58 (up to 5 m) or RG-8X / LMR-240 (longer) with the right connectors (BNC / SMA / PL-259), self-amalgamating tape.
- [ ] Radar reflector (we are NOT visible on AIS): e.g. Echomax EM230 or Tri-Lens, mounted high.

## Shore PC

- [ ] A second monitor for the DRIVER screen (MANUAL chapter 9 part I).
- [ ] USB joystick for remote driving (Thrustmaster T.16000M + TWCS recommended).
- [ ] Tailscale account (free) for the VPN.
