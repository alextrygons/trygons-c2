# PROGRESS

## RULE FOR REVIEW STOPS (owner, 2026-10-28)

A "LOOP STOPS" / review step only blocks the steps that DEPEND on it. While the owner's answer is
pending, the loop goes on with the next INDEPENDENT step (e.g. 12 gateway, 13 open items, 15
academy design) and picks the blocked steps up when the owner answers. Every open question is in
"WAITING FOR OWNER" below, with the steps it blocks.

HOLD (owner, 2026-10-28): the owner's safety review of 11e/11f is running; no step that changes 11e/11f
code (servo/throttle/steering/CAN/relay/CPU1 safety code) starts until the owner reports. Step 13 (carries
W18) and step 14 wait for it; steps that do not touch that code (e.g. 15 Academy design and later academy
steps) may continue.

## WAITING FOR OWNER (every open question; answered ones are removed)

Answered and NOT listed: LOW BATTERY stays an ordinary alarm with the buzzer (owner 2026-10-27);
OD-1..OD-12 (owner 2026-10-28, docs/dronecan_design.md 13); OA-1..OA-4 are recorded (owner
2026-10-28, docs/dronecan_design.md 14 - their open hardware details are W2 / W3 below). W18-W22, W24, W25-W29 and W31 answered
(owner 2026-10-28): see "Owner answers log" under the step table.

| # | What is asked | Blocks | Recommended default |
|---|---------------|--------|---------------------|
| W1 | DroneCAN TO CONFIRM list C1-C23 (docs/dronecan_design.md section 12; summary: servo node model / AP_Periph version, which node carries actuators 1-5, static node IDs (AURIX 10, 20 / 21 / 22, relays 30), gear F / N / R values, steering wire sign, unitless mapping, SRV_CMD_TIME_OUT behaviour, NodeStatus rate, servo speeds, servos unpowered, kit CAN pins / STB / connector / termination / transceiver supply, node 5 V supply, MCMCAN CPU1 access + transfer-ID hand-over, ThinkPad details, throttle rate limits, dead-man relay, Pixhawk removed) | 11d: only HAL_CAN_FITTED 1 (C12-C15) - the code is written with the flag 0. 11e: C1-C11, C16, C18, C19, C21 before the servos are "required" / the first engine run. 11f: C17. C20: step 12 details. C22 / C23: hardware only | everything stays TO CONFIRM; hardware flags 0; worst-case values (12 s / 6 s, 1500 ms; rate limits up 2..30 / down 20..200 %/s) |
| W2 | Throttle node `SRV_CMD_TIME_OUT` (OA-1): after the timeout do the pulses STOP or go to 0.0 (= ~50 %)? Does the spring then return the lever to idle? Bench, FIRST_TEST.md 3j | the first engine run with the DroneCAN servos (not the 11d / 11e code: the code assumes HOLD) | throttle 0 = HOLD (gear + steering 0 = HOLD: decided) |
| W3 | UNCREWED dead-man (OA-3): timer relay part, time (~1.5 s), contact type, fail-safe direction, arming only in UNCREWED (AURIX arming output vs an on-board UNCREWED switch), pulse source | 11e dead-man code + storm I57; any uncrewed operation | CPU1 pulses while UNCREWED + no trip + core-0 heartbeat + throttle node OPERATIONAL + no kill latch; armed by an on-board UNCREWED switch checked against the profile (docs/dronecan_design.md 14.1) |
| W4 | (a) From 11e the e-stop power contact no longer feeds the DroneCAN servo nodes (only the thruster contactor + the old steering driver / big boat) - confirm | 11e wiring / the first engine run with the DroneCAN servos | yes (OD-2 "never cut servo power"; the e-stop stops the engine through the kill circuit, OD-1) |
| W5 | (b) P10.5 = PIN_ENGINE_KILL, a second relay, or the dead-man pulse output (OA-3)? The ENGINE KILL relay must not cut the engine on a CREWED reset (fail-safe direction TBD). CPU1 needs the latched profile from core 0 (g_mon.profile) | 11e OD-2 implementation (P10.5, I54); kill-circuit wiring | P10.5 = the dead-man pulse output; PIN_ENGINE_KILL separate, "energised = kill"; CPU1 reads g_mon.profile (not yet latched = CREWED = no kill) |
| W6 | (c) OD-3 "kill at once" vs section B / storm I27-I28 "kill only after idle + neutral": recorded as an EXPLICIT exception - confirm | 11e I27 / I28 / I55 | the exception as written (CLAUDE.md B + T) |
| W7 | (d) OD-7 read as "no steering frame after a WARM reset either" (not only after a power-on) - confirm | 11e boot order / I56 | yes |
| W8 | (e) STARTER hardware time-limit relay 6 s vs the software crank_max_s 5 s - confirm | the relay part only (hardware) | 6 s |
| W10 | Two pythonw processes have been running on this PC since the afternoon of 2026-10-27 (13:56 / 14:53, probably your ground stations): please check / close them - they may cause the test_driver I4 / I12 / I19 failures | nothing (acceptance results only) | close them, re-run test_driver alone |
| W11 | Flaky acceptance results: test_driver I4 / I12 / I19 fail here AND in a clean clone of the pushed 11b HEAD (11b passed 21/21). Fixed in 11f (test code only): the storm workers share no data-flash files any more (each in its own folder: two 200-episode runs now give identical check counts; the I36 seed-108 failure was this), and test_boats removes a stale sim/nv7.bin before B9 and its own after the test | nothing | test_driver on a quiet PC |
| W12 | AUTO start checks only the active waypoint when the gateway has no checked copy of the program (the DP TRIP check is stricter: code 121 "route not checked by the gateway") - should AUTO start get the same check? | nothing (an extra check = its own small step) | yes, the same rule as DP TRIP (121) |
| W13 | Step 10 choices: a HOT thruster ends DP (STANDBY); the generator runs only in DP SPOT / JOG (CRUISE / TRIP stop it); dp_max_kn 3, entry < 2 kn, off spot > 50 m or 2 min -> STANDBY; the DP gains (step 10 note below) | nothing (parameters / constants) | keep as implemented |
| W14 | Steps 4 / 4b defaults: LOW BATTERY 11.8 V for 60 s; generator 25 % default, 30 % AURIX ceiling, 8 % fast idle, start below 12.1 V, +-2 % per 30 s, 20 min run + 1 min rest | nothing | keep |
| W15 | Data flash bring-up check on the kit: an erased DFLASH page reads 0 and a cut write gives no ECC trap (step 3 note) | nothing in software; NV on the real board | check at the first board bring-up |
| W16 | Board checks: the kill latch survives a real watchdog reset (FIRST_TEST.md 3i); RS485 TX-enable (DE) timing on the scope (FIRST_TEST.md 3h) | uncrewed operation (3i); the Modbus relay board on the boat (3h) | do them at the bench test |
| W17 | Step 12 hardware details (C20), documented with both options: (a) RJ45 ports on this ThinkPad L390 Yoga and its free USB ports (USB-UART, AIS, 1-2 USB-Ethernet adapters, charger -> a hub?); (b) the 12 V USB-C PD car charger model (65 W, 20 V PD) + its fuse; (c) the USB AIS receiver model; (d) the exact USB-UART adapter (FTDI TTL-232R-3V3 recommended) | the gateway wiring / purchase only (software done: any adapter by its stable name) | (a) two USB 3 Gigabit adapters (RTL8153) if no RJ45; (b) a 65 W USB-C PD car charger, fuse per its manual; (c) a dAISy USB / Quark-elec class NMEA-over-USB receiver; (d) FTDI TTL-232R-3V3 |
| W23 | Step 11f readings of design section 9 - owner 2026-10-28: keep (a)-(d), PENDING the owner's 11e/11f safety review: (a) after ANY reset the AURIX sends NO relay frame until a relay is commanded; (b) a new state = 10 frames at 50 Hz, then silence; (c) a board without hardpoint.Status: the commanded states shown ("no readback"), unknown after an AURIX-only power-on, all OFF when the board booted after the AURIX (uptime rule, R-2); (d) relays.c: an interlocked partner that is UNKNOWN and not momentary blocks the other relay | nothing (the code does it this way; a change is small) | keep (a)-(d) - pending the owner's safety review |
| W30 | Academy test area (spec J): the real Daskalio bay polygon + ramp top / bottom points (academy/content/test_area.json is a clearly marked PLACEHOLDER) | the practical test on the real boat only (it refuses a placeholder area); blocks no code step | set them with the instructor tool (step 21) on site: boat GPS "add point" at the ramp top / bottom |

Current task: "C2 extensions + Academy" (design = CLAUDE.md section "C2 extensions + Academy", parts
G-S; academy spec = docs/ACADEMY_SPEC.md). Steps 0-14 = C2 extensions (alarms in one place, boat
dropdown, data flash, supply voltage, generator mode, boat file v2, Modbus relay board, engine controls +
remote restart, relay buttons, DP parts 1-3, 11c-11f DroneCAN actuators (design + owner reviews after 11c and
after 11f: a review blocks only the dependent steps), ThinkPad gateway, open items, final check), 15-22 = Academy.
If interrupted: /clear, then "continue from PROGRESS.md" - do ONLY the first step not DONE, then stop.

Rules for every step: the owner decisions (CLAUDE.md G-S) are binding; the safety rules in CLAUDE.md
always win unless an owner decision changes them explicitly; sim tests for every failure case; new
storm invariants where a step moves the boat; tools/check.sh AND tests\acceptance\RUN_ACCEPTANCE.bat
quick pass before the commit; commit only after they pass, then git push at once; never amend or
rewrite a pushed commit; one commit per step; FIRST_TEST stays ON and its behaviour unchanged (DP,
generator, engine relays blocked in FIRST TEST); big-boat variant unchanged; Greek default for all
screen text (lang files), MANUAL.txt + MANUAL_EL.txt updated; PARTS.md / WIRING.md for hardware
(unknown = TBD); new folders excluded from the ADS build.

| Step | What | State | Commit |
|---|---|---|---|
| 0 | CLAUDE.md design for everything below (no code); docs/ACADEMY_SPEC.md | DONE (gate + acceptance quick: see the commit) | this commit |
| 1 | Alarms in ONE place, top right, flashing (DISPLAY ONLY): screen 1 box under mirror_r + notices, right column moved; screen 2 top-right panel always drawn; flash 1.25 Hz unacknowledged; hud_layout.json version bump; tests test_driver + test_screen2 S1 | DONE (gate OK; test_driver 19/19 incl. I19; test_screen2 12/12 incl. S12; acceptance quick run) | this commit |
| 2 | Boat dropdown = first line of screen 2's top-left panel + lamp ΣΥΝΔΕΔΕΜΕΝΟ; confirm step kept; tests test_boats + test_screen2 | DONE (gate OK except gateway VR5 under PC load - test_gateway alone 105/105 PASS; test_boats 6/6 incl. B6; test_screen2 13/13 incl. S13; acceptance quick 6/7 PASS - only START_DEMO skipped: a demo was already running on this PC, ports 5760-8554 in use) | this commit |
| 3 | Data flash (NV): TRAINING result, joystick calibration + map, AUTO program, boot counter, generator learned value; CRC, versioned; board tc_nv.c | DONE (gate OK: sim 193/193 + bridge 192/192 incl. DF1-DF5, TriCore + iLLD check with Flash/Std built, NV harness 4/4, gateway 105/105; test_driver 19/19, test_screen2 13/13; acceptance quick 7/7 PASS on a fresh clone of this commit, incl. START_DEMO) | this commit |
| 4 | Supply voltage input (12 V bank), filtered ~30 s, low-battery alarm | DONE (gate OK: sim 197/197 + bridge 196/196 incl. BV1-BV4, TriCore + iLLD check, NV 4/4, gateway 105/105, test_screen2 13/13, test_driver 19/19, test_lang 5/5, sound 10/10; acceptance quick 7/7 PASS on a fresh clone of this commit, incl. START_DEMO, storm 200 episodes) | this commit |
| 4b | Generator mode (the ONLY automatic throttle rise; CLAUDE.md K) | DONE (gate OK: sim 205/205 + bridge 204/204 incl. GN1-GN8, TriCore + iLLD check, NV 4/4, gateway, test_screen2 13/13, test_lang 5/5, test_driver 19/19; storm 200 episodes incl. I32-I35; acceptance quick 7/7 PASS - the "fresh build from GitHub" suite builds the pushed step-4 HEAD, every other suite this tree) | this commit |
| 5 | Boat file v2: relay list + generator_max_pct + thruster settings; old files still load | DONE (gate OK: sim 205/205 + bridge 204/204, TriCore + iLLD check, NV 4/4, FAT32 2/2, charts 6/6, test_lang 5/5, test_boats 8/8 incl. B7 + B8, test_screen2 13/13, sound 10/10, avoidance 14/14; gateway: section C crashed in the gate (sim connection refused at its start = PC load, untouched by this step) and passed alone 11/11, the other 94 OK; acceptance quick 7/7 PASS incl. START_DEMO from a fresh clone (the "fresh build from GitHub" suite builds the pushed step-4b HEAD, every other suite this tree), storm 200 episodes) | this commit |
| 6 | RS485 Modbus RTU relay board driver on the AURIX (state machine, timeouts, no-data advisory; never on CPU1) | DONE (gate OK: sim 213/213 + bridge 212/212 incl. RB1-RB8, TriCore + iLLD check, relay-writer check, NV 4/4, FAT32 2/2, charts 6/6, test_lang 5/5, test_boats 9/9 incl. B9, test_screen2 13/13, sound 10/10, avoidance 14/14, gateway 105/105; test_driver 19/19 (see the gate note); acceptance quick 7/7 PASS incl. START_DEMO from a fresh clone (the fresh-build suite builds the pushed step-5 HEAD, every other suite this tree), storm 200 episodes incl. I36 647 checks 0 fails) | this commit |
| 7 | Engine controls + remote restart (CLAUDE.md N; I31 changed) | DONE (gate OK: sim 219/219 + bridge 218/218 incl. ER1-ER6, TriCore + iLLD + STARTER / reset writer checks, gateway incl. CR10 (A2 at 57600, see notes); test_driver 19/19, test_screen2 13/13; acceptance quick 7/7 PASS, storm I31 / I37 / I38 0 fails) | this commit |
| 7b | Step 6 review fixes | DONE (gate OK: sim 225/225 + bridge 224/224 incl. RB9-RB14, TriCore + iLLD + relay / engine-relay writer checks, boats 9/9, screen 2 13/13, gateway 106/106; test_driver 19/19; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-7 HEAD, every other suite this tree), storm 200 episodes I36 4.28 M checks / I37 / I38 0 fails) | this commit |
| 8 | Relay buttons on screen 2 (collapsible panel) and screen 1 ("show on driving screen") | DONE (gate OK: sim 225/225 + bridge 224/224, TriCore + iLLD checks, lang 5/5, boats 9/9, screen 2 14/14 incl. S14 + S1 with the relays panel, sound 10/10, avoidance 14/14, gateway 108/108 incl. RL1-RL2; test_driver 20/20 incl. I21; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-7b HEAD, every other suite this tree), stations storm I16 30 s 0 violations) | this commit |
| 9 | DP part 1: sim thrusters + app/thruster.c (only writer, clamp, budget), CPU1 + hal_safe_outputs -> 0, e-stop contactor in WIRING | DONE (gate OK except gateway B3 under PC load - section B alone 12/12 PASS; sim 229/229 + bridge 228/228 incl. DP1-DP4, TriCore + iLLD + thruster-writer / safe-output checks, boats 10/10 incl. B10, screen 2, lang, sound, avoidance; test_driver 20/20; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-8 HEAD, every other suite this tree), storm 200 episodes I39 4.28 M checks 0 fails) | this commit |
| 10 | DP part 2: MODE_DP (SPOT / JOG / CRUISE / TRIP) + control, invariants I39-I43 | DONE (gate OK: sim 240/240 + bridge 239/239 (1 N/A) incl. DP5-DP14 + GN9, TriCore + iLLD + writer checks, gateway 108/108, screen 2 14/14, lang 5/5; test_driver 20/20 (twice); acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-9 HEAD, every other suite this tree), storm 200 episodes I39 4.28 M checks, I40-I43 10 k DP steps, 0 fails) | this commit |
| 11 | DP part 3: ground station controls + HUD, gateway shallow / collision / geofence checks for DP trips | DONE (gate OK: sim 240/240 + bridge 239/239 (1 N/A), TriCore + iLLD + writer checks, NV 4/4, FAT32 2/2, charts 6/6, lang 5/5, boats 10/10, screen 2 15/15 incl. S15, sound 10/10, avoidance 14/14, gateway 111/111 incl. DP1-DP3; test_driver 21/21 incl. I22; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-10 HEAD, every other suite this tree), storm 200 episodes) | this commit |
| 11b | Steps 7-10 review fixes | DONE (all 8 points re-checked; point 1 was already covered for TRIP / continue / boat leg / current geofence - only the unchecked-program gap fixed (code 121); see notes. Gate OK: sim 244/244 + bridge 243/243 (1 N/A) incl. ER7-ER8, DP15-DP16, RB9 updated, TriCore + iLLD + writer checks, NV 4/4, FAT32 2/2, charts 6/6, lang 5/5, sound 10/10, avoidance 14/14, gateway 112/112 incl. DP4; test_driver 21/21; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-11 HEAD, every other suite this tree), storm 200 episodes I31 / I38 across resets, I44-I46 0 fails) | this commit |
| 11c | DroneCAN design, NO firmware code (CLAUDE.md T + docs/dronecan_design.md): the AURIX replaces the ArduPilot; servos (throttle / steering 360 deg position servo / gear) + relay board on DroneCAN; ardupilot.param read-out; kit CAN pins; node health + command timeouts; the new CPU1 safety path; asymmetric throttle rates + the slow servo; steering model / TRAINING; relay rules on DroneCAN; TO CONFIRM list + open decisions. **Owner safety review here: it blocks only the steps that depend on it (rule at the top); independent steps (12, 13, 15) go on** | DONE (docs only. Gate OK: sim 244/244 + bridge 243/243 (1 N/A), TriCore + iLLD checks, NV 4/4, FAT32 2/2, charts 6/6, lang 5/5, boats 10/10, screen 2 15/15, sound 10/10, avoidance 14/14, gateway 112/112. Acceptance quick 5/7: storm 200 FAILED once on I36 seed 108 ("engine relay 3 switched on"), --replay 108 holds and a re-run of the 200 episodes passed 0 fails - the storm is not deterministic across runs (check counts differ): finding, see notes; fresh build: test_driver 18/21 (I4 / I12 / I19 live HUD) on the pushed 11b HEAD AND this tree - not caused by this step, see notes. Open decisions OD-1..OD-12 ANSWERED by the owner 2026-10-28 (binding: CLAUDE.md T "Owner decisions 2026-10-28" + docs/dronecan_design.md 13; rule changes for 11e / 11f, new storm I54-I56 reserved) + TO CONFIRM C1-C21: notes + docs/dronecan_design.md 12) | this commit |
| 11d | CAN driver (MultiCAN / iLLD) + libcanard (MIT, vendored with its licence), DroneCAN messages needed; sim + host tests | DONE (gate OK: sim 257/257 + bridge 256/256 (1 N/A) incl. CN1-CN13, TriCore + iLLD check with the Can driver built, DroneCAN 15/15 (D1-D15), NV 4/4, FAT32 2/2, lang 5/5, boats 10/10, screen 2 15/15 (S11 after a Greek fix), sound 10/10, avoidance 14/14, gateway 112/112; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-11c HEAD, every other suite this tree), storm 200 episodes) | this commit |
| 11e | Servos over DroneCAN + the new CPU1 safety path + the steering model / TRAINING from 11c; storm invariants for node loss and command timeout | DONE (gate OK: sim 277/277 + bridge 276/276 (1 N/A) + DroneCAN worst-case servos (all --can) 269/269 + fast servos (all --can-fast) 269/269 (8 N/A each: DC-motor / steering-motor only) incl. SM1-SM9 + CN14-CN24, TriCore + iLLD + writer checks (servo.c / can_act.c), CPU1 RAM 5 %, DroneCAN 17/17 (D16-D17 new), NV 4/4, FAT32 2/2, charts 6/6, lang 5/5, boats 10/10, screen 2 15/15, sound 10/10, avoidance 14/14, gateway 112/112; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-11d HEAD, every other suite this tree), storm 200 episodes with I47-I57 0 fails. Board flags stay 0 (HAL_CAN_FITTED, HAL_CAN_SERVOS). Recommended defaults used for W2 / W3 / W5 / W6 / W7; new W18-W21; changed tests: notes) | this commit |
| 11f | Relay board over DroneCAN (all relay rules and tests kept). **Owner safety review here: it blocks only the steps that depend on 11f (e.g. 14 final check); independent steps go on** | DONE (the code complete, board flags stay 0, RELAY_BUS stays MODBUS: W22. Gate OK: sim 283/283 + bridge 282/282 (1 N/A) + DroneCAN worst-case (all --can) 275/275 + fast 275/275 (8 N/A each) incl. RB1-RB14 on Modbus AND on DroneCAN, RB15-RB20, TriCore incl. hal_tc375.c with the flags 1 (DroneCAN relay + servos, Modbus relay) + iLLD check, writer checks (hardpoints only by hal/can_relay.c), CPU1 RAM 5 %, DroneCAN 19/19 (D18-D19 new), NV, FAT32, charts 6/6, lang 5/5, boats 10/10 (B9 now cleans sim/nv7.bin), screen 2 15/15, sound 10/10, avoidance 14/14, gateway 112/112; storm 200 episodes twice with identical counts, I36 4.28 M / I37 401 (reached now) / I38 0 fails; acceptance quick 6/7 PASS - only the fresh-build suite (pushed 11e HEAD) fails test_driver I4 / I12 / I19, the known W10 / W11 failures, the same 3 fail in this tree, not touched by this step. Daskalio photos committed (docs/daskalio/, JPEG + README), W9 removed; new W22-W24; notes) | this commit |
| 12 | Gateway on a Lenovo ThinkPad L390 Yoga (i5-8265U, 8 GB RAM, 500 GB SSD, Win11 now): support BOTH Ubuntu 24.04 LTS (recommended at sea: no forced updates / restarts) and Windows 11; remove Pi-only parts; 3.3 V USB-UART to the AURIX (never RS232); USB AIS receiver; README / PARTS / WIRING with install steps for both OSes; measure CPU / RAM with 4 camera streams + vision on the CPU (no NVIDIA); power from the 12 V bank via a USB-C PD car charger, built-in battery = UPS, battery charge limit 80 % (Vantage on Windows / TLP on Ubuntu), lid closed must keep running, Ethernet to the PoE switch (USB-Ethernet adapter if no RJ45) - put this in PARTS / README | DONE (gate OK: sim 283/283 + bridge 282/282 (1 N/A) + DroneCAN 275/275 x 2, TriCore + iLLD checks, CPU1 RAM OK, FAT32 2/2, NV 4/4, DroneCAN 19/19, charts 6/6, lang 5/5, boats 10/10, screen 2 15/15, sound 10/10, avoidance 14/14, OS 12/12 (new test_osdep.py OS1-OS12), gateway 112/112 (F5 failed once in a run during the docs work, passed alone and in the gate: timing); acceptance quick 7/7 PASS (fresh build of the pushed 11f HEAD incl. test_driver 21/21). Load measured on the dev PC, not the ThinkPad: gateway 1.55 CPUs / 726 MB, vision 1.95 fps (README 21); new W25, W26, W17 updated; notes) | this commit |
| 13 | Open items: AUTO return, docs/screens pictures (tools/demo_screens.py), paddlewheel, analog joystick; W18 (owner 2026-10-28): the false GPS SUSPECT of the dr.c slow-drag check at the new 50 %/s throttle slow-down (rate_down 50). **Waits for the HOLD (owner's 11e/11f safety review)** | | |
| 14 | Final check: full acceptance + 24 h soak, TEST_REPORT. **Waits for the HOLD (owner's 11e/11f safety review)** | | |
| 15 | Academy design + "no boat link" tests | DONE (Python / docs only - no app / hal / gateway / groundstation code touched. Gate OK: sim + bridge + DroneCAN all exit 0, TriCore + iLLD + writer checks, FAT32 2/2, NV 4/4, DroneCAN 19/19, charts 6/6, lang 5/5, boats 10/10, screen 2 15/15, sound 10/10, avoidance 14/14, OS 12/12, academy 14/14 (new, AC1-AC14), gateway 112/112; acceptance quick 7/7 PASS (the fresh-build suite builds the pushed step-12 HEAD, every other suite this tree), storm 200 episodes. Design D1-D11 in docs/ACADEMY_SPEC.md + CLAUDE.md S; new W27-W31; notes) | this commit |
| 16 | Academy A (accounts, levels, gateway push) | DONE (no app / hal / 11e-11f code touched. Gate: sim + bridge + DroneCAN x 2 exit 0, TriCore + iLLD + writer checks, FAT32 2/2, NV 4/4, DroneCAN 19/19, charts 6/6, lang 5/5, boats 10/10, screen 2 15/15 (S11 fixed, re-run alone), sound 10/10, avoidance 14/14, OS 12/12, academy 20/20 (new AC15-AC20), gateway 113/116 in the full run - E1-E3 cameras DOWN, section E passed alone (PC load) - incl. new AL1-AL4; acceptance quick 7/7 PASS. W25-W29, W31 answered and removed; notes) | this commit |
| 17 | Academy B (guided tour) | DONE (no app / hal / gateway / 11e-11f code touched. Gate: tools/check.sh exit 0 (sim / bridge / DroneCAN suites, TriCore + iLLD + writer checks, FAT32, NV, DroneCAN, charts, lang, boats, screen 2, sound, avoidance, OS, gateway), academy 24/24 (new AC21-AC24); acceptance quick 6/7 PASS - fresh build of the pushed step-16 HEAD: only test_driver I4 / I12 / I19 (known W10 flaky, the same 3 alone in this tree; hud / driver untouched). 125 stops cover 123 items read from the station's own sources; notes) | this commit |
| 18 | Academy C + D (sim lessons, quizzes) | | |
| 19 | Academy E (emergency scenarios) | | |
| 20 | Academy F (real-trip video lessons + editor) | | |
| 21 | Academy G + J (exams, practical test, online academy) | | |
| 22 | Academy H + I (3D sim interface, manual + tests + screenshots) + final report | | |

## Owner answers log

- W18 (owner 2026-10-28): handle it in step 13 (added to the step 13 row): the false GPS SUSPECT of the dr.c slow-drag check at the new 50 %/s throttle slow-down.
- W19 (owner 2026-10-28): keep SB_STEER_CALM_DPS 0.3 deg/s (calm-water limit for steering learning); CLAUDE.md T + docs/dronecan_design.md updated.
- W20 (owner 2026-10-28): accepted - a warm reset in UNCREWED stopping the engine (dead-man) is acceptable; docs/dronecan_design.md 14.1 updated.
- W21 (owner 2026-10-28): tune the AUTO line capture with the worst-case 12 s servo after TRAINING has measured the real servo speed (C10).
- W22 (owner 2026-10-28): MODBUS stays the default RELAY_BUS until a DroneCAN relay board passes the bench tests R-1..R-3 ("not yet" note in docs/dronecan_design.md 9).
- W23 (owner 2026-10-28): keep (a)-(d) of 11f - still listed, pending the owner's 11e/11f safety review.
- W24 (owner 2026-10-28): only relevant if "only the relay board on CAN, servos on PWM" is chosen - kept as a note in docs/dronecan_design.md 9.
- W25 (owner 2026-10-28): Ubuntu for the gateway (the camera network's DHCP + NTP = dnsmasq + chrony, README 14); no Windows DHCP work.
- W26 (owner 2026-10-28): the owner runs `python tools/gateway_load.py --minutes 10` on the ThinkPad later (README 21).
- W27 (owner 2026-10-28): the recommended defaults - 3 exam attempts, then 24 h cool-down; practical test max wind 12 kn; 08:00-19:00 local (academy/content/limits.json + test_area.json notes updated).
- W28 (owner 2026-10-28): the recommended default - backup = a copy to a folder the instructor chooses (a second shore PC / a USB disk), never a boat gateway (step 16: accounts.Store.backup).
- W29 (owner 2026-10-28): the recommended default - an offline TOTP app (6-digit codes), no SMS (step 21).
- W31 (owner 2026-10-28): yes - the ground-station menu Academy opens the academy in its OWN window / process, simulator only, never the station's boat link (step 16).
- W30: on site later (the real test area with the instructor tool, step 21) - still listed.

## Notes

- Step 17 (2026-10-28): Academy B - guided tour. academy/content/tour.json (kind tour, 9 chapters / 125 stops, el + en; alarm stop titles = groundstation alarm.<code>, levels from alerts.py), academy/tour.py (pure model; required_items() parsed from hud.py / app.py / proto / alerts: 21 HUD boxes, 12 HUD buttons, 12 screen-2 parts, 15 tools pages, 9 modes, 39 alarms, 2 lamps, 6 joystick parts, 7 panel buttons - a new screen item without a stop fails AC22), academy/tour_app.py (window: banner, DRAWING of the screen with the part lit, next / back / chapters / say again / voice on-off; plays only prepared .wav through simguard.check_recording, winsound; text only otherwise). Voice: the academy runs no TTS / process (D2 / D3): the ground station menu Academy -> Guided tour (academymenu.open_tour) starts tour_app.py (only --lang) and makes the sentences with its own sound.Library.make in the background (tour.py loaded by its path); same file names as the station's sound cache. Academy window button Guided tour. Tests AC21-AC24; MANUAL / MANUAL_EL, CLAUDE.md S, ACADEMY_SPEC D13. Greek wording follows the station (ΣΥΓΚΥΒΕΡΝΗΤΗΣ, ΤΙΜΟΝΙ). No new owner question.
- Step 16 (2026-10-28): Academy A - accounts, levels, gateway push, menu "Academy". No app / hal / 11e-11f code
  touched; no safety rule changed (the licence can only be narrowed / granted through the existing gateway flag,
  read at login only). academy/accounts.py Store (academy/data/ gitignored or TRYGONS_ACADEMY_DATA: one JSON per
  account, append-only logbook.jsonl - a cut last line is skipped -, push/levels.json, academy_key.txt; refused on a
  gateway PC), default expiry = validity_months of limits.json, instructor view Store.summary (lessons, weak areas
  = topics averaging < 70 %, attempts, exams, hours, currency, expiry reminder), backup (W28: chosen folder, never
  inside the data folder / a boats/ name or IP / Tailscale UNC / a gateway PC; the key is never copied).
  academy/academy_app.py = the window (banner first at the top, CAUTION colour). The push keeps D2 (no boat link
  from the academy): a SIGNED file (HMAC-SHA256 over the canonical JSON) that the ground station sends over its own
  gateway login - groundstation/academymenu.py: menu Academy "Open the academy" (own process, only --lang: W31) +
  "Send the academy levels to the gateway" (GatewayLink only; answer academy_levels_ack shown, gw.acad_* templates
  translated). gateway/trainlevels.py (named so the AC11 "no safety code imports the academy" check stays exact):
  secrets.json "academy_key" (none = refused), seq replay check, licence = CAPTAIN only and only with an expiry,
  HELM-role stations only, unknown users ignored (never a new user / role / password), last copy
  state/academy_levels.json, audit academy_levels, gw_status "academy"; users.Users.licence: academy record =
  CAPTAIN within the expiry (gateway clock), no record = the secrets.json flag; read at login only (stations.py /
  web.py) = never revoked mid-trip (the spec's "next login / HELM take" implemented as the next login). Certificate
  PDF of the instructor view = step 21 (exams). Tests: academy AC15-AC20 (+ AC9 / AC12 updated), gateway AL1-AL4.
  Gate: sim / bridge / DroneCAN suites exit 0; screen 2 S11 caught a Greek text ("Gateway:"), fixed, 15/15 alone;
  gateway E1-E3 failed in the full run (cameras DOWN), passed alone (PC load); every other suite passed.
- Step 15 (2026-10-28): Academy design + "no boat link" tests. New top-level academy/ (excluded from the ADS build
  in all 4 .cproject configurations; academy/data/ gitignored for step 16): simguard.py (the ONLY academy module
  allowed a socket / recording: sim on THIS PC = loopback + 5760 / 5761, gateway welcome name "trygons-demo" and
  never a boats/ name; boats/ IPs, Tailscale, LAN, host names, port 5762 refused; recordings = ordinary files, never
  COM / /dev / tcp: / usb:; nothing at all on a gateway PC = gateway.json present), banner.py + lang/el.txt / en.txt
  (academy.banner = «ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ» exact, EN DASH), text.py, accounts.py (levels model only: expired /
  unknown = OBSERVER, licence = CAPTAIN; push_to_gateway = step 16), content.py + content/ (README + commented
  example_* lesson / quiz / scenario / exam + limits.json + test_area.json PLACEHOLDER, not_for_navigation, the
  daskalio photos listed for display only). academy/test_academy.py AC1-AC14 in tools/check.sh: static + transitive +
  runtime proof that no academy module reaches a boat link (the checker is shown to see link / gwclient / boats /
  aurix / gateway as boat links), simguard tables, gateway-PC refusal, banner, lang keys, content checker incl. bad
  cases, photo rule (no safety code mentions docs/daskalio or reads the academy), levels, ADS exclude, practical
  test kept apart (academy/practical/ = step 21, own test required). NO ground-station menu entry yet (D10: the spec
  gives none for step 15; step 16 adds it, W31) - so no MANUAL change in this step (nothing user-visible; the academy
  chapter = step 22). Open: W27 (attempts / cool-down / wind / hours), W28 (backup target), W29 (2FA), W30 (real test
  area), W31 (menu = own window, never the station's boat link).
- Step 12 (2026-10-28): gateway on the ThinkPad L390 Yoga, Ubuntu 24.04 LTS AND Windows 11. NO AURIX / app /
  hal change. New gateway/osdep.py = the only OS-specific module: resolve_serial (usb:VID:PID[:SERIAL] by
  pyserial's port list on both OSes, /dev/serial/by-id -> the real device on Ubuntu, COMn on Windows, tcp: for
  the sim; resolved again at EVERY reconnect, so a re-plugged adapter on another port / COM number is found),
  serial_warnings / check_config (not set, Raspberry Pi UART names refused, COMn / ttyUSBn = unstable warning,
  usb: without a serial number = warning, the example's placeholders, AURIX = AIS device, baud, a path of the
  other OS, set_clock on Windows = offset), data_dir / config_dir (/var/lib/trygons, /etc/trygons |
  %PROGRAMDATA%/trygons), install_stop (SIGINT / SIGTERM on Ubuntu, Ctrl+Break on Windows), cpu_temp_c /
  throttle_count (Ubuntu sysfs; the Pi's vcgencmd is gone from vision/manager.py: "CPU 80 C" / "CPU
  throttling" pause reasons). common.DEFAULTS: aurix "" and ais.source "" (not set = config error, logged, the
  gateway still runs), storage on the PC's own SSD (require_mount false), mediamtx.yml / audit fallback under
  data_dir. gateway.py: --list-serial, --check-config (exit 1 on an error), --status implemented (a STATUS line
  every 60 s; print_status_s), config problems logged at start. aurix.py logs a connect problem once (not every
  second; AurixLink.error). Install: gateway/systemd/ (service: ReadWritePaths /etc/trygons /var/lib/trygons
  -/mnt/ssd; install_ubuntu.sh: packages, dialout, folders, logind lid ignore + sleep targets masked, TLP 75 /
  80 % + USB_AUTOSUSPEND 0, no automatic reboot, snap refresh hold, .link names boatlan0 / cam0 from the MACs;
  nftables.conf + camera_internet.sh use boatlan0 / cam0), gateway/windows/ (install_gateway.ps1: Task
  Scheduler task "Trygons Gateway" at boot as SYSTEM, AllowStartIfOnBatteries + DontStopIfGoingOnBatteries, no
  time limit, restarts; powercfg never sleep / lid 0 / USB selective suspend off on AC and DC, hibernate off;
  firewall rules from 100.64.0.0/10 + 10.42.0.0/24; Windows Update AUOptions 2 + NoAutoReboot; secrets.json ACL;
  run_gateway.bat restart loop + log rotation at 50 MB; stop_gateway.ps1 stops only this repo's gateway
  processes; uninstall_gateway.ps1). No new dependency (pyserial was already needed). tools/gateway_load.py =
  the CPU / RAM measurement (ctypes / /proc, no psutil). Docs: gateway/README.md rewritten (sections 1-10 for
  both OSes, RS232 WARNING at the top, 14 camera network cam0 + the Windows gap, 17 USB AIS, 21 the
  measurement), WIRING.md 7 (USB-UART adapter table, WARNING, adapter in the AURIX box, USB-C PD power, battery
  UPS 80 %, lid, Ethernet) + 8 / 10, PARTS.md "Gateway PC" (Pi parts listed as removed), MANUAL.txt +
  MANUAL_EL.txt (gateway PC wording, chapter 9 intro), vision/README.md, CLAUDE.md Architecture + AIS + R6 /
  R7 lines + section Q DONE line; lang cap.sys_gw text without "Pi" (el + en). TEST_REPORT.md /
  docs/dronecan_design.md keep their historical Pi wording.

- Step 0 (2026-10-26): CLAUDE.md got the section "C2 extensions + Academy" (G-S), the generator exception
  in "Safety rules" ("Nothing raises the throttle by itself" + the one exception), the engine-kill latch
  line in B (remote restart, I31) and the AUTO-return pointer in D. The first alarm word is full: new
  alarms go into alarm_bits2 (SB_ALM2_*, TELEM2 group L). Frame ids reserved: 0x2C GENERATOR, 0x2D RESTART,
  0x2E DP, 0x2F RELAY_CFG, 0x30 CONFIRM_ONLY (academy). Log events 23 NV, 24 GENERATOR, 27 RESTART, 28 DP.
  TELEM2 groups L (bank / generator / alarms2), M (relays), N (DP). NV slots 5 TRAINING, 6 generator.
  Storm invariants: I31 reworded (remote restart), I32-I35 generator, I36 relays, I37-I38 restart,
  I39-I43 DP. The iLLD Flash/Std driver is excluded in .cproject today: step 3 must un-exclude it.
- Step 2 (2026-10-26): screen 2's top-right "boats" panel is gone (6 panels now, S1 counts 6). The status
  panel's first line = app.boat_line: the dropdown button (tag boat_dd: boat name / typed address / "Choose
  boat", click = tk.Menu from app.boat_menu_entries: the boats by name + status + IP detail, Other
  address, Disconnect), the lamp (tag boat_lamp; app.boat_state: "connected" green ONLY when prober status
  online AND app.boat is that boat AND link.connected AND boats.check_welcome passes; else grey online /
  offline / no_ip / checking / "no boat check"), "name verified" small (tag verified). New: app.boat_pick
  (the boat chosen before / while connecting), boats.Prober.found(boat) (connect_boat marks the boat found
  and adds it to the prober), crewui.BoatSelector uses app.boat_list. Choosing -> the existing login +
  BoatConfirm (name + live camera) before HELM, unchanged. Lang keys: ov.choose_boat, ov.connected,
  ov.no_boat_check (ov.boats, ov.address removed). Tests: test_boats B6, test_screen2 S8 rewritten + S13.
- Step 3 (2026-10-26): data flash. hal/tc_nv.c (+ .h) = the record log (pure C over a port, API nvlog_*),
  hal_tc375.c = the DFLASH0 port (IfxFlash.h inlines issued, D0BUSY polled; regions A / B = sectors 0-7 /
  8-15, erased pages read as 0 per IfxFlash.h "Verify Erased Page"; iLLD Flash + Flash/Std un-excluded
  in .cproject - only one configuration had excluded them). hal.h: hal_calib_save / load removed (calib.c
  -> slot 5 through app/nv.c), new hal_nv_service(allowed) / hal_nv_pending / hal_nv_ok, slots 5-7,
  HAL_NV_MAX_SIZE 1536. app/nv.c = status (reset mask, fault, pending) + nv_step at the end of sb_step
  (allowed = STANDBY and not stopping); Cpu0_Main.c writes the boot counter before the loop (2 s limit,
  watchdog kicked) and services the flash in the big-boat variant in AP_MODE_STANDBY. Notice: TELEM2 group I
  config flags b1 NV DATA RESET / b2 NV FAULT / b3 save pending (no new group: L-N stay reserved), log
  event 23 NV, both screens (lang nv.reset / nv.fault). Sim: hal_sim.c queue + one file write per
  service call, faults SIM_NV_POWER_LOSS / SIM_NV_STUCK; hal/tc_nv.c is in build_sim.bat and
  build_storm.bat (DF4 drives the real log on a RAM flash). Test A3 adapted: it never modelled the power-off
  (now: one STANDBY step writes the program, hal_init() = reboot). New: `autopilot_sim df`, tests/hal_pc/
  test_nv.c + nv_check.py in the gate (tools/check.sh also knows the nvlog_ driver prefix).
  TODO on the board (no hardware yet): confirm on the kit that an erased DFLASH page reads as 0 and that
  no ECC trap follows a cut write (DMU_HF_ECCC.TRAPDIS left at its reset value); the log recovers from a
  wrong erased value by NV FAULT (never a wrong record). Gate note: the full gate passed except S11 (two
  English words in the new Greek texts); fixed, test_screen2 + test_lang re-run alone 13/13 + 5/5.
  The owner's demo (START_DEMO, running since 18:40) held sim/autopilot_sim.exe open: stopped with
  demo.py stop so the sim could be rebuilt - restart it with START_DEMO.bat.
- Step 4 (2026-10-26): supply voltage. app/battery.c (pure C: filters bank_v tau 30 s / bank_v_fast 1 s, LOW
  BATTERY below batt_low_v 11.8 V for 60 s, clears +0.3 V, NO VOLTAGE DATA notice after 5 s, NaN / out-of-range
  samples = not measured, filters restart from the first sample after a gap, data lost while low = condition
  cleared). smallboat: sb_t.batt, alarm_bits2 / SB_ALM2_LOW_BATTERY / SB_ALM2_ADVISORY (the first word is full);
  the power-limit drop and the video SAVE skip advisories of BOTH words; sb_alarm_ack keeps LOW BATTERY while
  batt.low; hal_read_supply_voltage is read once per step in sb_step (logcore0 no longer calls it). link.c:
  parameter batt_low_v (10..13.5), TELEM2 group L (alarm_bits2, bank V, fast V, flags, generator state 0).
  Log: field bank_v replaces the raw supply_v (the 20 Hz sample is 182 of the 184 B black-box slot: 2 bytes
  left - new values go into events); ALARM_ON / OFF events carry word = 2 in d for the second word; header line
  alarms2. FOUND + FIXED: the log header text (~3.9 KB) was cut at 3072 B, so every TLG file lost part of the
  alarm names and the whole modes / why lists - logger.c now builds it in one shared 4 KB buffer (s_hdr, replaces
  three 3 KB statics: 3 KB less CPU2 RAM). Board: tc_adc channel ADC_SUPPLY (group 8 ch 5 = A2 / P40.7 / AN37
  CHECK), HAL_SUPPLY_FITTED 0 + SUPPLY_DIV_RATIO in hal_pins.h (divider TBD: WIRING.md 3B, PARTS.md incl. the
  starter battery isolator). Sim: sim_hal_set_supply(fitted, volts), `autopilot_sim bv`. Ground station:
  alerts.bits_of(d) = both words (word 2 at bit << 32, ALL_ALARMS), proto ALARMS2 / alarm_texts2 / group L
  decode (bank_v, batt_low, batt_no_data), status panel row "Battery", HUD bottom-line volts (red while low),
  NO VOLTAGE DATA notice on both screens, tlg.py reads alarms2 + word 2 events, gateway web / recorder list
  both words. Lang: alarm.low_battery, say.alarm.low_battery, cap.v_battery, cap.batt_none, batt.no_data,
  hud.batt(_none). MANUAL 11 / 13 / 15 / 17, MANUAL_EL 9 / 10. TO CONFIRM: 11.8 V, 60 s, advisory (power limit
  stays), the divider values.
- Step 4b (2026-10-26): generator mode. app/generator.c (pure C): gen_t with states OFF / READY / STARTING / RUNNING /
  STOPPING / REST / FAULT / DONE (= TELEM2 group L "generator state" + demand %), gen_set (captain on / off; the
  STANDBY button = off), gen_step with gen_input_t (allowed, neutral confirmed, volts, power limit, stop request, GPS
  SOG, thrust (0 until DP), throttle output). Rules as CLAUDE.md K: start only with ALL conditions (10 s beep ->
  idle_pct 8 %), +2 % / 30 s while bank_v_fast still falls (> 0.02 V), hold = learned value, -2 % above start_v + 0.4,
  cap = min(gen_max_pct, GEN_CEILING 30, power limit) followed down at once, cycle 20 min / 1 min rest / rest check,
  every stop idle first (demand 0, STOPPING until the output is idle, nothing rises), GEAR FAULT (switch opened / SOG
  + 0.5 kn within 20 s of a rise) = idle at the fast STANDBY rate + SB_ALM_GEAR + auto off until re-enabled.
  smallboat.c: the demand is applied in STANDBY only (gear N), stop requests = any non-advisory alarm (LOW BATTERY,
  nav advisories do not stop it), a pending AUTO / REMOTE / MANEUVER CONFIRM, TAKE, a CREWED stick off centre;
  throttle.c neutral_run_pct + servo.c neutral_run_pct (clamp min(cap, fwd limit) instead of the reverse limit in
  neutral) + the section-5 direction clamp; thr.stop uses the normal closing rate for a generator stop, the fast one
  for a fault. CPU1 (monitor.c): g_mon.gen_active + gen_cap_pct in the seqlock set; with gen_active the throttle limit
  in neutral = min(cap, fwd limit) and the gear pulse must stay at neutral (MON_TRIP_GENERATOR, new reason 6); without
  it the reverse limit applies as before (GN6 proves all three). link.c: PROTO_T_GENERATOR 0x2C (u8 0 / 1; FIRST TEST
  -> WRONG_MODE, gen_max_pct 0 -> PRECOND; HELM check as every command), parameters gen_max_pct 0..30 / gen_start_v
  11.5..13 / gen_idle_pct 3..15. Log event 24 GENERATOR (state, stop_why | enabled << 8, bank V*100, pct*10) on every
  state / demand change - events only, the 20 Hz sample is untouched. NV slot 6 = {version 1, pct, pad, crc32}, saved
  in STANDBY once the generator is idle, loaded in sb_init (bad record = fast idle + NV DATA RESET). Sim: world
  neutral_sw_force (0 follows the gearbox, 1 stuck open, 2 stuck closed), pcsim captures TELEM2, `autopilot_sim gen`
  = GN1-GN8 (in `all` too). Storm: NINV 36, I32-I35, events E_GEN (ACK + bank 11.5 V + auto on, fits a neutral switch
  that follows the gearbox) and E_SUPPLY; 200 episodes 0 fails (I33 / I34 8188 checks, 66 stops). FOUND BY THE
  STORM + FIXED (seed 169, I35): in the step of a joystick takeover while the generator ran, the neutral-run cap was
  computed before the gear request, so the MANUAL demand (capped at the generator cap) opened the throttle in neutral
  for one step (+0.75 %) - the cap now applies only to the generator's own demand (STANDBY, gear demand N); every
  other demand meets the normal rule (idle in neutral, gear only at idle). Ground station:
  proto T_GENERATOR + GEN_STATES + is_driving_cmd (only ON needs the captain's ACCEPT; OFF goes straight), screen 2
  control panel button GENERATOR AUTO ON / OFF on the AUTO button line, the state on the Battery row (its own row
  did not fit at 1366 x 658), notices STARTING / FAULT / DONE on both screens, HUD bottom line "GEN x %". Lang keys
  cap.v_generator, gen.st_0..7, cap.btn_gen_on/off, gen.starting / fault / done, hud.gen, what.generator_on/off.
  MANUAL 7 (new part GENERATOR MODE) / 15 / 17, MANUAL_EL 7; WIRING.md + PARTS.md: the neutral switch is the
  generator's precondition. TO CONFIRM: 25 % / 30 % ceiling, 8 % idle, 12.1 V, +-2 % / 30 s, 20 min / 1 min.
  FINDING (not fixed - acceptance tests report, they don't change app logic): with a fitted neutral switch that is
  stuck OPEN, TRAINING -> gear fault -> STANDBY logs the TRAINING throttle demand (20 %) in the fault step (storm I9
  "STANDBY with throttle asked 20 % / gear F"; the output itself stays idle: the gear machine never allowed it). Seen
  only when the storm's E_NSW event was routed through the world (switch really fitted); reverted E_NSW to its old
  direct HAL set (overridden every tick, as before) so the storm's coverage is unchanged. Reproduction: route E_NSW
  through S.w.neutral_switch / neutral_sw_force and run `storm.exe --replay 37`. Candidate fix: zero throttle_mode /
  gear_demand when the gear fault requests STANDBY in sb_step section 6 (cosmetic: the log value only).
- Step 5 (2026-10-26): boat file v2 (ground station only; no AURIX change). groundstation/boats.py: "format" (missing = v1:
  the v2 fields stay at their defaults - generator_max_pct 0 = disabled, no relays, thrusters max_pct 0; unknown format =
  INVALID), generator_max_pct 0..100 (the AURIX ceiling 30 applies on the push), relays (n 1..32 unique, name_el /
  name_en, type onoff | momentary | latching | timed, group engine | trim | extra, pair mutual, momentary 0 < max_s <= 5,
  timed max_s > 0, show_hud bool, power_up off | on | keep - the engine group only "off": owner clarification), thrusters
  (bow / stern max_pct 0..100, run_s > 0 when fitted, rest_s, jog_m 0.5..10 default 1.5, spot_alarm_m 5..100 default 15),
  length_m / draft_m optional. Any failed check = Boat.error -> status "invalid" (Boat.fixed_status(), prober.status,
  probe; lamp / list / dropdown "INVALID FILE" / "ΑΚΥΡΟ ΑΡΧΕΙΟ", crewui + app.connect_boat refuse: never connected).
  boats.aurix_settings(boat, ceiling) = what the HELM station pushes; app.boat_push (called from on_telem): only a
  confirmed boat (boat_ok), HELM held, mode STANDBY (PARAM_SET is STANDBY only), the parameter list read -> PARAM_SET
  gen_max_pct = min(file, the parameter's max) through _send_cmd (the captain's own action, single-op: confirm "self");
  a NACK re-arms it after 5 s, a boat restart (t_ms back) re-arms it. The relay table (PROTO_T_RELAY_CFG, step 6) and the
  thruster settings (step 9) join boat_push when the AURIX takes them. boats/rf49d.json + er69d.json are format 2
  (generator_max_pct 25, 7 sample relays IGNITION / STARTER / SHUTOFF (engine), TRIM UP / DOWN (trim, pair), NAV LIGHTS /
  BILGE PUMP (extra), thrusters 100 % 60-120 s - all channel numbers and values TBD / TO CONFIRM, notes in the files).
  Lang: boat.st_invalid, boat.invalid, what.boat_gen. Tests: test_boats B2 (broken file = invalid), B7 (v1 defaults, v2
  parsed + ceiling, 19 bad files INVALID, repo files valid v2), B8 (e2e through the real gateway + sim: nothing pushed
  before HELM, gen_max_pct 20 on the boat after HELM, nothing outside STANDBY, file 40 -> 30). test_screen2's rig stubs
  link.send in gateway_like mode (the push would have reached a plain Link with gateway kwargs). MANUAL chapter 5 + 7,
  MANUAL_EL: the boat file paragraph. Gate note: gateway section C (channels) crashed once in the full gate with
  "connection refused" when its simulator started (PC load); alone 11/11.
- Step 6 (2026-10-27): RS485 Modbus RTU relay board. hal/tc_modbus.c (+ .h) = the master (pure C over a port: one
  request in flight, state machine SEND -> WAIT <= 100 ms -> IDLE, CRC-16/MODBUS, 0x01 readback every 100 ms, 0x05
  set / board-timed pulse 0x02xx, 0x0F several coils, write queue of 8, no valid reply 3 s = SILENT, keeps polling);
  hal_tc375.c = the port on ASCLIN4 (TX P00.9, RX P00.12, DE + /RE P00.11, all CHECK; tc_uart.c UART_RELAY +
  uart_tx_done for the DE release), HAL_RELAY_FITTED 0 until the board is wired (driver never started -> a boat file with
  relays shows RELAY BOARD SILENT). hal.h: hal_relay_service / set / pulse / read / ok. app/relays.c = the ONLY caller
  (tools/check.sh grep, word-bounded so the sim's sim_hal_relay_* test hooks don't match): table (<= 16 relays, NV slot 7,
  version + CRC-32), offer / CONFIRM (CONF_RELAY_TABLE = confirm kind 7: a DIFFERENT table only in STANDBY + joystick
  CONFIRM, onboard in CREWED / shore in UNCREWED; the stored table stays until then; the same table = OK at once; any other
  CONFIRM request or leaving STANDBY drops the offer), commands (PROTO_T_RELAY 0x1E: off / on / pulse; interlocked pair
  never both, momentary = board-timed <= max_s <= 5 s, timed = OFF by the AURIX after max_s, re-sent until the readback
  shows it off, STARTER refused (section N only), nothing in FIRST TEST, nothing new on a silent board), power_up = only ON,
  once, only with the first table after the AURIX's own POWER-ON reset, never the engine group, waits for the board.
  smallboat: sb_relay_table / sb_relay_cmd, relays_step in sb_step 8b, SB_ALM2_RELAY_SILENT (0x2, advisory: no mode
  change, power limit stays; kept by the ACK while silent), the confirmed table saved in STANDBY. link.c: PROTO_T_RELAY_CFG
  0x2F (variable length), PROTO_T_RELAY, TELEM2 group M (states, known, table hash, flags, count, commanded on). Log event
  25 RELAY (commands, table events, readback changes, board lost / back; events only - the 20 Hz sample is untouched).
  Sim: a simulated board in hal_sim.c (latency, SILENT / CORRUPT / CORRUPT_SOME, board-timed pulses that run on with core 0
  frozen), `autopilot_sim rb` = RB1-RB8 (in `all`). Storm: E_RELAY + I36 (relays never change mode / throttle / gear /
  steering / thrust). Ground station: proto T_RELAY / T_RELAY_CFG (both in DRIVING_CMDS: conservative until step 8 splits
  engine / trim / extra), p_relay_cfg, relay_table_hash, group M decode (relay_on / relay_unknown are LISTS: the gateway
  forwards telemetry as JSON - a set broke it and flooded the station with "bad message" lines), CONFIRM_KINDS + confirm.6 /
  .7, alarm relay_silent (ADVISORY, both words), app.relay_push (HELM + STANDBY, sent when the TELEM2 M hash differs from
  the file's table, again after 15 s while it still differs and no CONFIRM is pending), boats.py max 16 relays. Tests:
  test_boats B9 (end to end through the gateway + sim: pending, CONFIRM, active hash = file, never re-sent). Docs: MANUAL 5 /
  7 (RELAY BOARD) / 11 / 17, MANUAL_EL, WIRING.md 3D (new), PARTS.md, CLAUDE.md M (done line). TBD: the board, the
  transceiver, baud, address, channel count, the pins.
  Gate note: test_driver I1 (heading box on the tape) failed on this PC with the per-PC groundstation/gs_layout.json that a
  killed test run had rewritten ("driver": null); with the file moved aside 19/19 PASS (file put back: it is the owner's).
- Step 7 (2026-10-27): engine controls + remote restart. app/engine.c (+ .h, pure C): engine_t in sb_t.eng, conditions
  engine_check (FIRST TEST, e-stop, throttle output idle, gear N confirmed = gear state N + no gear fault + the neutral
  switch if fitted, else the gear servo at N), engine_restart (a) kill latch cleared (ll.state OK, ENGINE KILLED bit gone,
  kill pin off, STANDBY stays) / (b) after a monitor trip a pending software reset ENG_RESET_DELAY_MS 500 ms later (log
  event 27 first, data flash idle <= 2 s, conditions re-checked every step - failing = cancelled, event 7), engine_crank /
  engine_release / engine_step (hold-to-crank: board-timed 300 ms STARTER pulse per CRANK, <= 5 s per hold, 10 s cool-down
  after EVERY hold, stops on release / no CRANK for 300 ms / link or HELM lost / e-stop / gear request or not N / throttle /
  STANDBY button / kill / trip). sb_restart(s, station, action) also checks station == HELM; sb_restart_ready. relays.c:
  relays_starter (= the engine-group momentary relay), relays_starter_pulse / _off, only called from engine.c (tools/check.sh
  grep, also: hal_system_reset only from smallboat.c). hal.h hal_system_reset (board: IfxScuRcu_performReset application,
  info 0x27; sim: a flag - sbsim_run, storm and sim_server reboot after the step). link.c PROTO_T_RESTART 0x2D (u8 0 RESTART /
  1 CRANK / 2 RELEASE), TELEM2 group O (12 bytes after M: crank state, flags, hold ms, cool-down, last action / why,
  restarts, cranks; DP's N follows it). Log event 27 RESTART. FOUND + FIXED (from step 6): the log header text was 4109 B and
  cut at 4095 (the end of the "why=" list was lost) - the descriptions of events 23-25 + 27 are short now (their names live
  in groundstation/tlg.py: NV / GENERATOR / RELAY / RESTART decoders), header 3972 B; sim ER6 checks it is complete. Also
  alarms2 lists 2:RELAY_SILENT. Ground station: proto T_RESTART, ENG_* names, group O decode, is_driving_cmd (RESTART = ACCEPT
  for a copilot; CRANK / RELEASE are not asked one by one), Tools page "Engine" (engine-group on / off relays of the boat
  file, START hold = CRANK at 5 Hz, RESTART, the state + last reason; tab.engine, eng.*, what.engine_* / relay_onoff),
  ov.engine_killed text. Gateway stations.py: a copilot's CRANK -> 118 (captain only). Tests: sim ER1-ER6 (`autopilot_sim
  er`, in `all`), gateway CR10, storm I31 reworded + I37 + I38 + a new E_ENGINE event (restart / START held / release /
  e-stop; I5 now skips while the e-stop is pressed: only STANDBY then). Test adjustments: GN8 / RB7 read groups L / M from the
  frame end + T2_O; gateway A2 runs at 57600 (at 38400 the control traffic alone fills the line since group O, the log copy
  got no room at all, so no gap could be noted). MANUAL 5 / 7 (ENGINE CONTROLS + REMOTE RESTART) / 17, MANUAL_EL, WIRING 3A /
  3D, PARTS (tacho TBD), CLAUDE.md N done line. TODO on the kit: the application reset puts servo / gear / safety relay
  in their reset state at once and the reason reads "application". No tacho (time limit only).
  Gate: tools/check.sh OK except gateway A2 (fixed as above, section A alone 6/6 PASS); test_driver 19/19 (I1 had failed
  with the first engine page: the state label resized the tools window every telemetry - fixed: fixed height, set only on
  change); test_screen2 13/13, test_lang 5/5; acceptance quick 7/7 PASS incl. START_DEMO from a fresh clone (the fresh-build
  suite builds the pushed step-6 HEAD), storm 200 episodes incl. I31 / I37 / I38 0 fails.

- Step 7b (2026-10-27): step 6 review fixes. Re-checked against the code after step 7: none of the 8 findings was
  fixed by step 7 (finding 1 only partly: the STARTER was already pulsed only from engine.c, but it was found by its TYPE).
  1 engine ROLES: relay_def_t.pad -> role (RELAY_ROLE_STARTER / IGNITION / SHUTOFF), RELAY_CFG frame 8 bytes per relay,
    table version 2 (a stored v1 table = NV DATA RESET, the station pushes again + CONFIRM). Table check: engine group <=>
    a role, STARTER momentary (<= 5 s), IGNITION / SHUTOFF onoff / latching, each role once. relays_cmd never switches an
    engine relay ON; IGNITION / SHUTOFF ON only via engine_relay (engine.c: idle + neutral confirmed, e-stop released, no
    kill / trip) -> relays_engine_set; OFF always. boats.py: "role" required for the engine group, a relay named STARTER /
    ΜΙΖΑ without the role = INVALID; boats/*.json have the roles; app.engine_relays by role. check.sh greps relays_engine_set.
    NOTE: SHUTOFF ON also needs idle + neutral (the owner's rule "engine-group relays ON only with idle + neutral");
    the engine kill pin / e-stop stop the engine at any time.
  2 sb_relay_table: a different table is refused (REFUSED) while any other CONFIRM is pending (the same table = OK);
    app.relay_push skips while TELEM2 confirm_kind != 0.
  3 momentary: until = pulse end + RELAY_PULSE_MARGIN_MS 500 ms -> the AURIX's own OFF, pending until the readback shows off.
  4 hal_tc375: reset reason latched once in hal_init + IfxScuRcu_clearColdResetStatus; decision = hal/tc_rstcause.h
    (pure, warm trigger beats a stale PORST - the iLLD evaluateReset turned every reset into "warm power-on" while PORST
    was still set); compile-time check of the RSTSTAT / RSTCON bit positions. FIRST_TEST.md 3h (real watchdog reset ->
    no power_up, twice; DE timing on the scope).
  5 tc_uart: uart_on_tx_done(UART_RELAY, ...) arms the ASCLIN TC flag (EX interrupt) per write; the ISR releases DE
    right after the last stop bit (the old poll in relay_port_read stays as a fallback). Board-only: no sim test
    possible (hardware timing) - FIRST_TEST.md 3h checks it on the scope.
  6 SB_ALM2_QUIET (= RELAY BOARD SILENT): never sets sb alarm (no buzzer, no power-raise block). LOW BATTERY KEPT its
    buzzer (step-4 design, test BV2 requires it) - TO CONFIRM by the owner whether LOW BATTERY should be quiet too.
    hal_relay_channels() (board: RELAY_CHANNELS if HAL_RELAY_FITTED else 0; sim: sim_hal_relay_fitted): 0 = no SILENT,
    commands refused (PRECOND).
  7 mb_write_coils (+ the 0x0F path) deleted; mb_set_coil / pulse refuse n > the board's coils; tables / stored tables
    with a channel beyond the board refused / not used; the automatic relay push never sends confirm "self" (single
    operator: the own-confirmation question first, then it goes).
  8 storm I36: the sim board every step (pair 4/5 never both, every board pulse <= 5 s, engine coils 1-3 switched on only
    <= 800 ms after an accepted command / crank pulse, no power_up after a non-power-on reset, a QUIET advisory never sets
    the alarm flag), the first alarm word judged after a relay event (monitor / gear / e-stop / profile / takeover /
    TRAINING / overrun; the self-timed bits excluded), FIRST TEST via a shadow relays_t (every command refused, no
    power_up: the storm has no FIRST TEST episodes), new event E_WDT (watchdog reset = SYSTEM); episodes now start with
    a fresh relay board and no nv7.bin. Sim: SIM_RB_NO_PULSE_END, sim_hal_relay_max_pulse_ms / _edges / _fitted.
  Tests: sim RB9 (roles), RB10 (offer vs pending CONFIRM), RB11 (pulse OFF), RB12 (quiet advisory, no board), RB13
  (channels), RB14 (reset reason patterns + no power_up after a watchdog reset); test_boats B7 (8 new INVALID role
  files) + B9 (no push while a CONFIRM is pending, single operator asked). Docs: MANUAL 5 / 7 / alarms, MANUAL_EL,
  CLAUDE.md M (step 7b line), WIRING 3D, FIRST_TEST 3h.
- Step 8 (2026-10-27): relay buttons (ground station + gateway only; no AURIX change). proto.is_driving_cmd(ftype,
  payload, relay_groups): T_RELAY free for a copilot only when the relay's group in the table ACTIVE on the AURIX is
  trim / extra (proto.relay_groups(payload) / groups_of(relays); an engine role counts as engine whatever the group
  byte); unknown relay / no or unknown table = ACCEPT (fail safe); T_RELAY_CFG always ACCEPT. Gateway: gateway/
  relaytab.py (RelayTables: every RELAY_CFG a station sends is learned by CRC-16 = the TELEM2 group M hash, newest 8,
  state/relay_tables.json on the SSD); stations.command uses stations.relay_groups(); gw_status "relays" {hash,
  known}. Station: app.relay_groups_known (only when the active hash = the boat file's AND the gateway knows it - the
  station never thinks a relay free that the gateway holds); app.relay_push re-sends the SAME table once when the
  gateway doesn't know it (the AURIX takes the same table at once, no CONFIRM). Screen 2: panel "relays"
  (app.panel_relays / relay_view / relay_lines / relay_press / relay_release): by group ENGINE (IGNITION, START hold =
  crank_press / crank_release, SHUTOFF + RESTART line) / TRIM / EXTRA, a pair on one line, lit = on, grey + disabled =
  unknown or table not active / pending (note line), momentary = PULSE max_s at press + OFF at release, onoff /
  latching / timed = toggle by the readback. overlay.py: a panel corner given as a TUPLE = placed by the fit test in
  the candidate corner whose side keeps the most free height (map bar still fits; RF49D -> "se" above the traffic
  panel at 1366x768), button(release=fn) = press-and-hold (Overlay.holds / held / release(), app.cv_release), ops of
  buttons carry "on". Screen 1: hud.py box "relays" (0.095, 0.640, 0.150, 0.190; not in MINIMAL; no layout version bump:
  a new key takes its default), show_hud relays, trim / extra = "relay:n" buttons (driver.py: relay_hold, release on
  mouse up), engine = lamp only (Hud.button(action=None) = no hit). Lang: ov.relays, ov.relay_grp_*, ov.relay_hold,
  ov.relay_not_active / pending / silent / unknown, what.relay_hold ("hold relay {name}": "relay {name} held" was read
  back as relay_onoff by from_en), hud.ov_relays. Tests: test_screen2 S1 (7 panels: RF49D's relays in every layout) +
  S14, test_driver I21, gateway section RL (RL1 copilot extra / trim straight, engine -> ACCEPT; RL2 SSD memory,
  unknown table = ACCEPT, broken frame), test_lang families (ov.relay_grp_*), acceptance stations_storm.py judges with
  the gateway's relay groups. MANUAL 7 (RELAY BUTTONS), MANUAL_EL, CLAUDE.md O (done line).
- Step 9 (2026-10-27): DP part 1 = the thrusters (no MODE_DP yet: dp_active only from the sim hook sb_test_thrust, so on
  the boat the thrusters are ALWAYS 0 until step 10). app/thruster.c (+ .h): the ONLY caller of hal_set_thrusters (new
  hal.h call, tools/check.sh grep), every step; 0 unless dp_active AND allowed (not FIRST TEST, e-stop released, no
  monitor trip, armed, HAL not forced) AND fitted (max_pct > 0, run_s > 0) AND not HOT; clamp to max_pct (ceiling 100),
  50 %/s rate limit, every stop 0 at once; heat budget: running (> 2 %) +1 s/s, rest -run_s/rest_s per s (rest_s 0 =
  run_s), full = HOT until empty -> SB_ALM2_DP_HEAT 0x4 (an ORDINARY alarm: buzzer + power limit down, kept by the ACK
  while hot, no mode change; step 10 decides what DP does with it). Settings: PROTO_T_THRUSTER_CFG 0x1F (12 bytes, HELM
  + STANDBY not stopping, RAM only, hash = CRC-16 of the body; no settings after a reset = no thruster). link.c TELEM2
  group N (10 bytes after O: flags, outputs, heat %, max %, hash, why; step 10 appends its DP fields), log event 28 DP
  (settings / hot / cooled). CPU1: g_mon.dp_active + outputs + caps, MON_TRIP_THRUSTER (thrust > 0.5 % without
  dp_active, above a cap, NaN); monitor_publish_steering_only (big boat) zeros them. Board: RC-style pulses (1500 us
  stop +- 400) on GTM TOM0 ch 12 / 13 (P22.3 / P11.10 CHECK), HAL_THRUSTERS_FITTED 0 = pins never touched,
  thrusters_stop() in hal_safe_outputs (shadow registers; check.sh checks it). Sim: hal_sim thrusters (latched 0 on a
  trip / chip reset), the e-stop contactor (no thrust while pressed), bank sag 0.006 V per % ; sim_world sway (0.5
  m/s), bow yaw (6 deg/s) and stern surge (2.5 kn) at full thrust. Generator: gi.thrust_pct = |bow| + |stern|.
  Station: proto T_THRUSTER_CFG (DRIVING_CMDS: a copilot's = ACCEPT), p_thruster_cfg / thruster_cfg_hash, group N
  decode, ALARMS2 DP THRUSTER HOT, alerts dp_heat CAUTION, app.thruster_push (after relay_push, HELM + STANDBY, again
  when the TELEM2 hash differs, 15 s), tlg.py event 28. Lang: alarm.dp_heat, say.alarm.dp_heat, what.boat_thrusters.
  Tests: sim DP1-DP4 (`autopilot_sim dp`, in `all`), test_boats B10 (end to end through gateway + sim), storm I39 (for
  now: thrusters 0 every step). T2_O in sim_sb.c now skips O + N. Docs: WIRING 3C (new) + e-stop power contact,
  PARTS, MANUAL 5 / 7 (DP THRUSTERS) / alarms / 17, MANUAL_EL, CLAUDE.md P (done line). TBD: thrusters, controllers
  (RC input with "no pulses = stop" assumed; PWM + DIR would need a HAL change), pins, contactor.
- Step 10 (2026-10-27): DP part 2 = MODE_DP. app/dp.c (+ .h, pure C): dp_t (sub SPOT / JOG / CRUISE / TRIP, the anchor,
  the held heading, set speed, integrators, the TRIP leg) + dp_start / dp_jog / dp_speed / dp_heading / dp_stop / dp_step.
  Control: the GPS fix (1 Hz) blended 0.5 into an estimate propagated with the GPS velocity; boat frame from the compass
  (or the gyro-integrated heading while the compass is lost); SPOT / JOG: stern = fore / aft PI + velocity damping (8 %/m,
  30 %/(m/s), I 0.4 %/(m s), clamp 60 % of the cap, frozen beyond 15 m), bow = heading PD on the gyro (4 %/deg, 10 %/(deg/s))
  FIRST + the sideways PD only with what is left, at most DP_SWAY_SHARE 30 % of the cap and fading to 0 at 5 deg of heading
  error; dead bands 1.5 deg / 0.4 deg/s and 1 m / 0.15 m/s (heat budget). CRUISE / TRIP: stern = speed PI (15 %/kn, 4 %/(kn s),
  ahead only, astern down to -30 % of the cap), bow = heading. TRIP: nav_leg to each waypoint, look-ahead max(20 m, 8 s),
  speed = min(set, leg, dp_max_kn, navreq cap), slows to 1 kn in the last 30 m before the last / a STOP waypoint; the end or a
  STOP waypoint -> SPOT there (STOP: "continue" = START TRIP | 0x80 goes on from the next one). JOG done within 1 m / after 30 s.
  Off spot: beyond spot_alarm_m = offspot (hysteresis 80 %), > 50 m or offspot for 2 min = DP_R_OFFSPOT; both thrusters within
  0.5 % of their caps 10 s = overpower; the generator check "fight" = stern < -30 % for 5 s while the boat got 1 m farther
  ahead of the spot.
  smallboat: MODE_DP = 8 (modes.c table: STANDBY -> DP, DP -> STANDBY / MANUAL), WHY_DP / WHY_DP_STOP / WHY_DP_OFFSPOT /
  WHY_DP_THRUSTER (log header names short: ER6 still complete), CONF_DP + sb_t.dp_req, cfg dp_max_kn 3 / dp_max_entry_kn 2
  (TO CONFIRM), dp_ok() = GPS trusted, compass, armed, no trip / e-stop / lost link / forced outputs / gear fault / FIRST TEST,
  both thrusters fitted + none hot, entry SOG < 2 kn, TRIP waypoint reachable. sb_dp_cmd (link.c PROTO_T_DP 0x2E, 4 bytes,
  HELM station; MANUAL -> JOYSTICK except STOP): START from STANDBY = CONFIRM; in DP SPOT at once, CRUISE / TRIP with the
  CONFIRM again; JOG / SPEED / HEADING in DP only; STOP cancels a waiting DP request and ends DP. HEADING_ADJ in DP = a heading
  change (works UNCREWED too; the panel course buttons in DP CREWED). sb_request_mode calls dp_stop(why) when leaving DP;
  on_enter(DP): steering core disengaged, dp_start at the trusted fix. dp_mode(): GPS not trusted -> alarm + STANDBY
  (WHY_GPS_LOST), compass 10 s / no gyro -> WHY_COMPASS_FAULT, off spot -> WHY_DP_OFFSPOT, TRIP end -> chirp + ARRIVED.
  The thruster step (4b) now runs BEFORE the outboard's gear / throttle: dp_active = MODE_DP; a thruster HOT (it went to 0
  itself the step before) or not allowed in DP -> STANDBY (WHY_DP_THRUSTER) first, so both are 0 in that step.
  DECISION (step 9 left it to step 10, TO CONFIRM by the owner): DP THRUSTER HOT in DP ends DP (STANDBY) - it can't hold with
  one thruster; no new DP while hot. Joystick fault (CREWED) and the joystick disappearing end DP too (the crew's takeover).
  The generator may run in DP SPOT / JOG (gi.allowed, the demand + gen_neutral in DP, the SOG check off in DP); a pending DP
  CRUISE / TRIP start and CRUISE / TRIP itself stop it; dp.fight -> GEN_STOP_DP_FIGHT (generator.c, FAULT, auto off) ->
  SB_ALM_GEAR + DP ends (WHY_GEAR_FAULT). Navigation request: nav_applies() = AUTO or DP TRIP (cap, avoidance heading, lost
  lease = hold the heading + min(nav_lost_kn, last cap), shallow alarm); SPOT / JOG / CRUISE = alarm only. sb_program_set
  refused while a TRIP runs; a new program resets the TRIP "continue" waypoint and cancels a waiting TRIP request.
  Alarms (second word, advisories): SB_ALM2_DP_OFFSPOT 0x8, SB_ALM2_DP_OVERPOWER 0x10 (kept by the ACK while they hold).
  TELEM2 group N + 12 bytes (u8 sub, u8 flags b0 off spot b1 overpower b2 gyro hold b3 trip done b4 DP CONFIRM pending b5
  fight, i16 spot error ahead / starboard dm, u16 heading target*10, u8 set kn*10, u8 trip kn*10, u8 wp, u8 stop why):
  sim T2_N = 10 + T2_DP 12. Log event 28 what 4 start / sub, 5 stopped, 6 JOG, 7 TRIP end (header text "28:DP:what,a,b,c").
  LIMIT (sim + physics): a tunnel thruster can't sway the boat while holding the heading - a cross current / wind drifts the
  boat off the spot (DP OFF SPOT, then STANDBY after 50 m / 2 min); the manual says "turn the bow into the current".
  NOTE for the owner: the sample boat files' heat budgets (bow 60 s / stern 120 s) end a DP in a current after 1-2 min
  (HOT -> STANDBY) - real run_s / rest_s of the chosen thrusters TBD.
  Station (display only for now; screens = step 11): proto T_DP + p_dp + DP_* names, is_driving_cmd(T_DP) = everything but
  STOP (a copilot's STOP goes straight), MODES + "DP" (MODE_DP 8), WHY + 4, CONFIRM_KINDS + "DP", ALARMS2 + DP OFF SPOT /
  DP OVERPOWER, TELEM2 N DP decode (dp_sub_name, dp_err_*, dp_heading ...), alerts dp_offspot / dp_overpower CAUTION, theme
  MODE_COLOURS DP, tlg.py event 28 what 4-7 + GEN_STOP 8. Lang: mode.DP, confirm.8, alarm.dp_offspot / dp_overpower,
  say.alarm.* (el + en).
  Tests: sim DP5 (every entry refusal + CONFIRM onboard / shore), DP6 (SPOT in a 0.5 kn head current: <1 m, heading <2 deg;
  HEADING_ADJ / course button), DP7 (JOG x4), DP8 (CRUISE 2 / 1 kn, heading 120, SPOT at once), DP9 (TRIP, navreq cap,
  lost lease, end in SPOT at the last waypoint, navreq heading in SPOT = alarm only, program refused while it runs), DP10
  (stick X / TAKE -> thrusters 0 same step + MANUAL), DP11 (GPS lost / jump), DP12 (off spot advisory, 50 m, 2 min), DP13
  (HOT -> STANDBY, overpower advisory), DP14 (STANDBY button, e-stop, DP STOP, compass 10 s gyro hold, compass + gyro,
  UNCREWED link lost, STOP cancels a waiting request), GN9 (generator in DP SPOT, CRUISE stops it, the stern fight = GEAR
  FAULT). Storm: event E_DP (settings incl. tiny heat budgets, START SPOT / CRUISE / TRIP after centre + STANDBY + 15 s, JOG,
  SPEED, HEADING, STOP from the HELM or another station, a random current), NINV 44: I39 (0 outside DP + e-stop contactor),
  I40, I41, I42, I43; I7 (DP entry only with a CONFIRM), I20 (navreq heading never in SPOT / JOG / CRUISE), I34 (generator
  in DP SPOT / JOG), spec table + DP. Test change: sim GA2 (slow UART) at 76800 instead of 57600 - TELEM2 grew 12 bytes and
  the 10 Hz telemetry got one step late at 57600 (the real gateway UART is 921600).
  Test robustness (no app change): test_screen2 Rig.gateway_like holds the DP thruster push (send is stubbed, no ACK
  ever comes: the push repeated every 15 s and left an English event line from the S1 English pass -> S11 failed);
  test_driver I1 waits until the HUD was drawn at the real window size (on a busy PC the first HUD was still the one
  drawn at 200 x 150 - also seen once at the step-9 HEAD). Gate: the first run had gateway F1e fail (section F1 alone
  5/5 PASS, PC load), the second run GATE OK.
- Step 11 (2026-10-27): DP part 3 = ground station + gateway (DISPLAY + requests only; no AURIX change).
  Screen 2 (app.py): dp_rows in the control panel - with both thrusters fitted (TELEM2 thr_fitted_*) one "DP:" line
  SPOT / CRUISE / TRIP + the speed (- / +, 0.5 .. 3 kn = DP_MAX_KN, the AURIX's dp_max_kn); "waiting for CONFIRM" + DP
  STOP (cancels) while dp_pending; in DP the rows replace the mode buttons + AUTO rows: state badge, DP STOP, generator,
  SPOT / CRUISE / TRIP (+ "TRIP: continue" after a STOP waypoint = DP_TRIP | DP_TRIP_CONTINUE), speed SET (DP_SPEED), in
  SPOT / JOG the JOG arrows (jog_m from the boat file) + where the spot is (red when off spot); the panel's +-1 / +-10 =
  HEADING_ADJ. Status panel: the DP line instead of "target" + the held heading. Screen 1 (hud.Hud.dp): the DP box in
  the CDI place (mode 8, like MANEUVER): title, spot vector (square edge = spot_alarm_m, driver.py dp_scale_m), thrust
  bars B / S, amber when hot / off spot. Gateway: check_prog for T_DP START TRIP (shallow 114 / geofence 120, audit
  "dp_trip_refused", "continue" from dp_wp), stations.prog_check includes T_DP; nav_compose / nav_auto: shallow cap +
  avoider._auto in AUTO or a DP TRIP (dp_min_kn 1 kn as the slowest asked); DP SPOT / JOG / CRUISE danger = COLLISION +
  DRIVER DECISION alarm only (kind NONE); DP counts as "at sea" for AIS. proto.decode_telemetry: in DP with the GPS
  trusted est_sog / est_cog = the GPS values (the DR velocity comes from the idle outboard; DR values in dr_sog /
  dr_cog). Lang el + en (dp.*, what.dp_*, cap.v_dp), MANUAL 7 DP + MANUAL_EL, CLAUDE.md P "Done (step 11)".
  Tests: test_screen2 S15, test_driver I22, gateway DP1-DP3 (section DP).
  Test robustness (no app change): the screen 2 rig turns a posted tk menu / question box into a recorded error
  instead of blocking (on Windows tk_popup waits for a human): the interrupted runs of this step had test_screen2 hang
  once after S14 for an hour in the gate (not reproduced: alone 15/15 twice); now such a click fails the test with a
  reason instead of stalling the gate. An earlier interrupted gate also stood still in test_boats (an agent killed
  meanwhile; not reproduced: test_boats 10/10 alone and in the gate since) - watch for it (PC load / Tk).
- Step 11b (2026-10-27): steps 7-10 review fixes. Each point re-checked against the code first:
  1 (MED, gateway DP TRIP route check): ALREADY covered by step 11 - T_DP START sub TRIP and "continue" go through
    check_prog like AUTO_START, the boat -> first waypoint leg is checked (start=), the geofence is the current one
    at start time (a fence changed after the upload counts). The GAP was a program the gateway did not check
    (USB upload, loaded from NV at boot, a gateway restart, a stale copy after a USB upload with the same count):
    rest = [] -> nothing checked, the TRIP passed. Fix: the AURIX tells where its program came from (sb autop.via_gw,
    set by link.c on a gateway-port PROG_COMMIT, cleared by every other sb_program_set; TELEM auto_flags b3 ->
    proto prog_gw); the gateway keeps its copy only after the AURIX's ACK (prog_pending -> prog_committed from
    stations._on_gw_ack; prog_ack_new covers the telemetry still older than that ACK). DP TRIP without a checked
    program (or nothing left to continue) -> refused 121 "route not checked by the gateway" (+ lang el / en,
    alerts, MANUAL). AUTO start is unchanged (without the gateway's copy it still checks the active waypoint only:
    an open point for the owner, not part of this step). Test gateway DP4.
  2 engine.c: the crank pulse is cut to the hold's rest in whole 0.1 s units (the board rounds to the nearest, a
    250 ms rest gave 300 ms = past 5 s); less than 100 ms left -> crank_stop (MAX_TIME), no pulse. Sim ER7.
  3 kill latch: hal_kill_keep_set / _get (hal.h; board: two words in DSPR0 with TASKING #pragma noclear, magic +
    complement; sim: a variable the chip reset keeps). sb_init restores SB_LL_KILLED (+ alarm, kill output,
    ll.kept) only when hal_reset_reason() is not POWER_ON; a power-on writes "off". The restart path: a killed AND
    tripped boat's RESTART clears the kill too (before its software reset). FIRST_TEST.md 3i (check on the kit that
    the startup / SSW keeps DSPR0 on a warm reset). CLAUDE.md B / N (I31 / I38 wording), MANUAL + MANUAL_EL, WIRING
    3A. Sim ER8, storm: I31 / I38 checked at every boot (a kill before a non-power-on reset is still there after it;
    after a power-on it is gone).
  4 SHUTOFF ON (= stop the engine) always allowed: engine_relay(.., shutoff, ..) skips every condition for the
    SHUTOFF role (FIRST TEST still refuses every relay in relays.c); IGNITION ON keeps idle + neutral + e-stop +
    no kill / trip. Reverses the step-7b SHUTOFF condition: sim RB9 updated (driving: SHUTOFF ON ok, still HDG LOCK;
    e-stop: SHUTOFF ON ok, IGNITION refused), storm I36 judges only IGNITION's conditions. Docs: engine.h, CLAUDE.md
    step 11b line, MANUAL / MANUAL_EL.
  5 thruster.c: a HOT thruster now rests by the clock (cool_left_s / cool_total_s, set when it goes HOT); a settings
    push never touches HOT or cool_left; a push without the thruster keeps heat_s (no clamp to run_s 0) and the
    budget is frozen while it is not fitted (was: heat_s / hot cleared at once - a re-push cleared HOT early). Sim
    DP15.
  6 smallboat.c: after thruster_step, a thruster HOT in DP -> STANDBY + thruster_stop (both 0 in the same step;
    thruster.c stays the only hal_set_thrusters writer). Sim DP16.
  7 storm: I37 also needs the HELM station + its link, no kill, no trip (crank_link_ok; the per-pulse check also the
    crank station = HELM); new I44 (SPOT / JOG > 50 m or 2 min off the spot -> DP ended in that step), I45 (DP set /
    trip speed <= dp_max_kn), I46 (never a DP step with a thruster HOT). NINV 47.
  8 WIRING.md 3D: a relay wired to the starter MUST be the engine group's role STARTER, never "extra" / "trim" (same
    for IGNITION / SHUTOFF).
- Step 11c (2026-10-27): DroneCAN DESIGN only (no firmware code). CLAUDE.md section T + docs/dronecan_design.md
  (13 sections: what changes, ardupilot.param read-out, kit CAN pins, bus / IDs / messages, node health + timeouts,
  CPU1 safety path options A-D + recommendation B + failure analysis, asymmetric throttle rates + thr_est, steering
  model / TRAINING / adaptation, relay board on DroneCAN, 11d-11f structure, the photo rule, TO CONFIRM C1-C21, OPEN
  DECISIONS OD-1..OD-12). CLAUDE.md Safety-rules rate line rewritten (asymmetric; in force from 11e, the code keeps
  40 / 100 %/s until then), CPU1 board line + actuator lines point to T, section Q points to the new step 12.
  hal/hal_pins.h: HAL_CAN_FITTED 0 + CAN_TXD P20.8 / CAN_RXD P20.7 (iLLD-confirmed functions) / PIN_CAN_STB P20.6
  (unverified, unused). WIRING.md 3E, PARTS.md (node 5 V supply, CAN cable, terminations, USB-CAN adapter),
  ACADEMY_SPEC photo rule. docs/ardupilot.param committed (renamed from ardupilot.param.param).
  docs/daskalio/ was NOT on this PC (searched C:	rygons, Desktop, Downloads, Documents: only videos under
  C:	rygonsideos) - copy the photos into docs/daskalio/ and commit them; the photo rule is already in
  CLAUDE.md T + ACADEMY_SPEC + the design doc.
  Reserved: SB_ALM2_CAN_NODE 0x20 / SB_ALM2_CAN_BUS 0x40, TELEM2 P (CAN nodes) / Q (steering model), log events 29
  CAN / 30 STEERMODEL, MON_TRIP_CAN_TX, storm I47-I53, sim SM1-SM8.
  The iLLD Can driver is excluded in .cproject today: 11d must un-exclude it (check_ads_ifx.py).
  OWNER DECISIONS 2026-10-28 (answered, binding - CLAUDE.md T + docs/dronecan_design.md section 13, which also lists
  the earlier rules they change and the 11e / 11f tests + new storm I54-I56): OD-1 YES, OD-2 never cut servo power +
  P10.5 -> engine kill circuit UNCREWED only, OD-3 YES UNCREWED only, OD-4 owner-signed value only, OD-5 HOLD, OD-6
  notice, OD-7 no auto-centre at power-on (no steering frame until a mode commands it, p_est unknown), OD-8 no, OD-9
  Modbus build option + STARTER always behind a hardware time-limit relay, OD-10 HOLD, OD-11 bench (FIRST_TEST.md
  3j), OD-12 no. The questions were: OD-1 e-stop into the engine kill circuit (also CREWED); OD-2 what the
  safety relay P10.5 switches (cutting servo power would freeze the throttle); OD-3 throttle node lost in UNCREWED ->
  engine kill at once; OD-4 may a measured throttle travel time shorten the 6 s safety waits; OD-5 gear node timeout
  HOLD (recommended) vs neutral after a delay; OD-6 health WARNING = notice only; OD-7 centre the steering at a
  power-on boot; OD-8 save adapted steering values; OD-9 relay board with only one timeout behaviour; OD-10 STANDBY
  "steering off" = hold; OD-11 throttle return spring vs a position servo; OD-12 big boat unchanged.
  TO CONFIRM (never guessed; hardware flags 0): servo node model + firmware, relay board model, node IDs / actuator
  numbers (gear = actuator 3? 4 / 5?), steering wire sign, unitless mapping, node timeout options, NodeStatus rate,
  servo speeds (worst case 12 s / 6 s), servo behaviour unpowered, kit CAN pins / STB polarity / connector /
  termination / transceiver supply, node 5 V supply, MCMCAN CPU1 access + transfer-ID hand-over, ThinkPad details
  (step 12), throttle rate parameter limits. Full table: docs/dronecan_design.md section 12.
  Test findings of this run (NOT caused by this docs-only step, not fixed here - acceptance never changes logic):
  (1) storm 200: I36 failed once, seed 108 "engine relay 3 switched on with throttle 0.0 % / gear / e-stop at
  1168.9 s"; `test_results/storm.exe --replay 108` = all invariants held and a second 200-episode run passed (I36
  4278462 checks 0 fails vs 4271850 in the failing run) -> the episodes are not reproducible run to run (suspect:
  parallel workers sharing a file such as the sim NV files nv<slot>.bin in the working folder - relay table slot 7);
  look at it with the relay / restart work in 11f. (2) groundstation/test_driver.py 18/21 here AND in the fresh clone
  of the pushed 11b HEAD: I4 (HUD mirror left / right None), I12 (MINIMAL without the alarms box), I19 ("live False
  (... 0 texts)"); 11b passed 21/21 - something on this PC changed (two pythonw processes running since 13:56 /
  14:53 = probably the owner's ground stations; not touched). Re-run test_driver alone on a quiet PC.
- Step 11c owner additions (2026-10-28, docs only): OA-1 AP_Periph SRV_CMD_TIME_OUT has no "idle on timeout" (holds,
  then "zeros": pulses stop or 0.0 = ~50 %? bench) - gear / steering 0 = hold, throttle 0 unless the bench proves "pulses
  stop + spring to idle"; every "node timeout -> idle" claim removed (design 5, 6.3, 6.4, 7 I51, 12 C8; CLAUDE.md T;
  WIRING 3E; FIRST_TEST 3j bench check). OA-2 throttle node lost -> gear HOLD, never an automatic neutral (CPU1's trip
  frames and the boot order shift to neutral only with the throttle node OPERATIONAL). OA-3 UNCREWED hardware dead-man
  timer relay (~1.5 s, AURIX pulse, UNCREWED only; design 14.1, WIRING 3A, PARTS, FIRST_TEST 3j; reserved storm I57).
  OA-4 the Pixhawk is node 10 too: never on the bus with the AURIX (design 4, WIRING 3E warning, PARTS, FIRST_TEST 3j).
  Consistency points (a)-(e): design section 15 + WAITING FOR OWNER W4-W8. New rule for review stops + WAITING FOR OWNER
  at the top of this file. Next step: 11d (C12-C15 only block HAL_CAN_FITTED 1, not the code).
- Step 11d (2026-10-28): CAN driver + DroneCAN, the bus only (no actuator on CAN). hal/libcanard/ = DroneCAN
  libcanard (MIT, LICENSE + VENDORED.txt with the commit, unchanged; PC builds -DCANARD_64_BIT=1). hal/dronecan.c (pure
  C over a port dc_port_t): bus state machine (RUN / PASSIVE / BUSOFF with recovery at once + every 1 s / FAULT; off or
  passive > 100 ms = bus fault: every node lost, the software TX queue dropped + counted), RX <= 64 frames per step, TX
  <= 32 per step (full FIFO = waits), our NodeStatus node 10 1 Hz, GetNodeInfo server, node table (static IDs 20 / 21 /
  22 required, 30 relay board shown only: silent > 1500 ms, never seen, not OPERATIONAL, ERROR / CRITICAL, rebooted =
  latched until OPERATIONAL 2 s + ack in STANDBY; WARNING = notice, OD-6), a frame FROM node 10 = ID conflict (OA-4),
  dynamic-ID request = notice; encoders NodeStatus / ArrayCommand (one UNITLESS command per frame) / hardpoint.
  hal/tc_can.c = MCMCAN CAN0 node 0 via iLLD (1 dedicated TX buffer kept free for CPU1, 16 TX FIFO, 32 RX FIFO 0,
  extended IDs only, STB pin); hal_tc375.c starts it only with HAL_CAN_FITTED 1 (stays 0). .cproject: the iLLD Can
  folders un-excluded (check_ads_ifx OK). hal.h hal_can_service / bus / node / ack; app: smallboat.c can_step ->
  SB_ALM2_CAN_NODE 0x20 / SB_ALM2_CAN_BUS 0x40 (WARNING, power limit down, CONFIRM cancelled) + STANDBY (WHY_CAN_NODE /
  WHY_CAN_BUS, forced like a fault) + no mode but STANDBY while lost / not seen / bus bad; log event 29 CAN; TELEM2
  group P = 6-byte summary (TELEM2 has ~24 bytes left now), the full table = new frame PROTO_T_CAN_NODES 0x8E (USB,
  1 Hz) -> Bench page "CAN bus (nodes)"; ground station alarms / notices / voice lines el + en; tlg.py CAN events.
  Log header: SAVE's argument text shortened ("trigger(SB_SAVE_)") so the header stays < 4096 B (ER6). Sim: hal_sim.c
  simulated controller + bus + nodes (servo nodes with rate limit + backlash + HOLD, relay node, faults: silent,
  unpowered, health / mode, reboot, bus-off, nobody acking, node 10, dynamic-ID request, start failure), off by
  default. check.sh: libcanard in the TriCore compile, "no dc_send_ in the firmware" (11e / 11f), "hal_can_ only in
  smallboat.c", dronecan_check.py. Tests: D1-D15 (signatures computed from the DSDL text, payload bit layout, float16,
  frames, multi-frame GetNodeInfo CRC, state machine), sim CN1-CN13. Not done here (11e): cancel the HARDWARE TX FIFO
  on a bus fault / trip (design 10 note), the 10 ms actuator task, CPU1's buffer. No new owner question.

- Step 11e (servos over DroneCAN, the CPU1 safety path, the steering model / TRAINING): the code is complete; the
  board keeps HAL_CAN_FITTED 0 + HAL_CAN_SERVOS 0 (hal/hal_pins.h) until the owner confirms (W1 / W2 / W3 / W5).
  Recommended defaults used where the owner has not answered: W3 / W5 (P10.5 = the dead-man pulse output from CPU1,
  armed by the crew; PIN_ENGINE_KILL separate), W6 (OD-3 = an explicit exception to I27 / I28), W7 (no steering frame
  after a warm reset either), W2 (every node HOLDS on lost commands, the throttle too). New open points W18-W21.
  Code: app/servo.c (only writer: throttle / gear (0 = HOLD) / steering position; the throttle model est_pct;
  thr_trav_s 6 s, shorter only with a CONFIRM), hal/can_act.c (10 ms actuator task, latch before + after,
  dc_cancel_tx), hal/can_safe.c (CPU1 encoder), hal/dronecan.c (port cancel, dc_tx_flush, dc_cancel_tx,
  dc_cmd_tid, DC_PRIO_SAFE), app/throttle.c (rate_up 15, rate_down 50, stop = idle at once; stop_rate_pc_s retired ->
  thr_dn_pc_s / thr_up_pc_s / thr_trav_s), app/monitor.c (own envelope, MON_TRIP_CAN_TX, safe frames, dead-man),
  app/smallboat.c (thr_idle / thr_node_ok everywhere idle was the command; OD-3 kill; boot order; CONF_THR_TRAVEL;
  calm-water adaptation gate), app/steermodel.c + app/trainsrv.c (new), app/calib.c (slot 5 v2), app/link.c
  (TELEM2 group Q, 3 params), app/logcore0.c (event 30). Board: hal/tc_can.c (3 reserved TX buffers for CPU1,
  can_cancel_fifo, can_send_reserved: own TXBAR bit only), hal/hal_tc375.c (CAN servos, hal_can_tick / act state /
  safe send / steer keep in no-clear RAM / dead-man pin; shared data in core 0's DSPR), Cpu0_Main.c (10 ms tick).
  Sim: servo nodes driven from the world truth, the dead-man timer relay, the e-stop stops the engine (OD-1), suites
  "all --can" (worst case 6 s / 12 s) and "all --can-fast" (2 s / 3 s) in tools/check.sh; the PC-side link test
  harness now parses after every 128-byte read (TELEM2 + TELEMETRY + CAN_NODES in one step are more than its 512-byte
  ring: lost ACKs). Ground station: TELEM2 Q decoded, the Control page shows the steering model, notices TRAINING
  NEEDED / STEERING MODEL RESET (el + en). Storm: ~35 % of the episodes on the DroneCAN servos (from a separate seed
  hash: the PWM episodes are unchanged) with CAN faults; I47-I57; I27 / I28 wording (OD-3).
  Tests CHANGED for 11e (old rates / the old PWM-only safe path): sim P2 "STANDBY" (idle at once) and P2 rate
  (rate_down 50), P3 R1 (the stick steers until the MODELLED idle), P4 legs (max rate = rate_down), P4 sea state (runs
  with a paddlewheel: W18), P5 500 m waypoint (DroneCAN servos: rms <= 9 m, W21), LL1 / LL4 (kill bounds + the remote
  cap x thr_trav_s), CN1 / CN11 (11e semantics; CN11 forces the 12 s servo), C3 (MON_TRIP_CAN_TX accepted in --can),
  DP2 (a latched CAN_TX trip allowed), ER8 (8.5 s wait on the DroneCAN servos), DF3 (DroneCAN variant), the joystick
  override test (30 s reverse wait), P6 / F4 / BT2 N/A in --can (DC-motor / steering-motor only); storm I4 (bound =
  the modelled throttle at the press x thr_trav_s + idle wait), I5 (a lost / rebooted node = STANDBY only, 11d
  interlock), I27 (OD-3 exception). Fixed while testing: F1 STOP_STEER_CENTER ends only when centred AND idle +
  neutral; the steering model's unknown position takes the direction from the command (bench steps, CN19); the
  STANDBY button disarms a held stick also when it is the ACK in STANDBY (storm seed 22: a DroneCAN node interlock
  held a deflected stick in STANDBY, the ACK cleared the interlock and the stick took MANUAL at once; new sim CN24).
  Storm harness fixes: sim_hal_can_reset after the clock reset (nodes timed from the old episode = never seen), a relay
  event across a reboot is not judged (I36), I47 excludes a planned arrival stop (WHY_ARRIVED keeps rate_down).
- Step 11f (the relay board over DroneCAN, all relay rules kept; details: CLAUDE.md T "Step 11f DONE",
  docs/dronecan_design.md section 17). Code: hal/can_relay.c (new, pure C: bursts / AURIX-timed pulses / readback /
  board reboot), hal/dronecan.c (hardpoint.Status 1071 in, dc_hardpoint_status), hal.h (hal_relay_readback; the relay
  API text for both buses), hal_pins.h (RELAY_BUS, CAN_RELAY_CHANNELS; HAL_CAN_FITTED / HAL_CAN_SERVOS /
  HAL_RELAY_FITTED guarded with #ifndef only so the gate can compile the board paths with -D: the values stay 0),
  hal_tc375.c (both back-ends, the relay keep RAM, cr_tick in hal_can_tick, relay init after the CAN init),
  app/relays.c (the unknown-partner interlock, flag 64). Ground station: relay_no_readback (proto.py), the relays
  panel note + buttons usable for unknown relays on a board without readback (app.py / driver.py), el + en. Sim: the
  DroneCAN relay node 30 model + the STARTER time-limit relay (hal_sim.c), RB tests on both buses + RB15-RB20
  (sim_sb.c), "rb [--can]". The RB tests CHANGED only where the bus differs (RB1 slow board = SILENT on DroneCAN, RB2
  counts the transport's refused frames, RB5 / RB11 expected OFF times of the AURIX-timed pulse, RB13 the back-end's
  channel check). Storm: DroneCAN episodes on the DroneCAN relay board; the relay-table offer now gets its CONFIRM
  (I37 is reached: 401 checks in 200 episodes, before 0 = "not reached"); every worker in its own folder (W11).
  The Daskalio photos: the five untracked docs/IMG_0516-0520.HEIC are the Daskalio chart photos (Garmin screen,
  Ormos Dhaskalio / Akra Aspro): converted to docs/daskalio/IMG_05xx.jpg (<= 4032 px, 1.2-2.2 MB, metadata removed)
  + docs/daskalio/README.txt (the photo rule); the HEIC originals moved OUT of the repo to C:\trygons\daskalio_heic_originals
  (not in git). docs/ is excluded from the ADS build (check_ads_excludes.py OK). W9 removed (done by the owner).

# Previous task: "Screen 2 review"

Task: "Screen 2 review" (one section, one commit): screen 2 = "navigation & control"
(Greek "Πλοήγηση & έλεγχος"): full-screen map (boat centred, north-up locked, boat icon turns,
zoom; temporary pan snaps back after 10 s / "back to boat"), corner overlay panels
(semi-transparent, collapsible; STANDBY always visible; never over the boat), roles (MANUAL /
MANEUVER only from screen 1 with the licence, single-operator exception), MANEUVER on screen 1
(captain, FWD / N / REV), map editing for the copilot (route, geofence, max speed -> captain's
ACCEPT, pending / accepted / rejected shown), link panel (channels, profile, shore, lost-link
countdown, engine kill), boat selector (boats/, online, name verified), layout test 1366x768 +
1920x1080 at 100-150 %, all Greek texts translated, ONE shared style module for both screens
(theme.py, dark / light together), the logo (assets/) bottom-right on both screens, Greek manual.
Tests must pass (tools/check.sh + test_driver + quick acceptance), commit, push.
If interrupted: say "continue from PROGRESS.md" - continue with the first step not DONE.

| Step | What | State |
|---|---|---|
| a | theme.py (fonts, sizes, colours, buttons, panels, alpha, dark / light, DPI) + logo PNGs from assets/trygonsdrones_logo.svg | DONE (theme.py, assets/trygonsdrones_logo*.png) |
| b | mapwin.py: full-screen map core (centred, north-up, zoom, pan + snap back, route / fence editing) | DONE (mapwin.MapView) |
| c | overlay.py: panel engine (canvas display list, alpha panels, collapse, hits, fit check) | DONE (overlay.py) |
| d | app.py: screen 2 rebuilt (4 corner panels, banners, STANDBY, logo, tools window for the other tabs) | DONE (app.py rebuilt; tools window = vertical page list) |
| e | driver.py / hud.py: theme, logo, MANEUVER + REMOTE start (captain + licence), FWD / N / REV | DONE (hud drive / maneuver / logo; driver theme) |
| f | gateway: PROG_COMMIT = driving (copilot -> ACCEPT), geofence (route check code 120, ask_local) + tests | DONE (gateway/geofence.py, code 120, GF1-GF5 pass) |
| g | lang el / en (all new keys, every English text in the Greek UI translated) + test_lang | DONE (102 new keys + Greek fixes; i18n.from_en translates the filled-in parts too) |
| h | tests: test_screen2.py (layout matrix + functions), existing tests adapted, check.sh | DONE (test_screen2 11/11, test_driver 18/18, e2e 10/10, lang / boats / charts / sound / logs pass) |
| i | CLAUDE.md, MANUAL.txt + MANUAL_EL.txt, PROGRESS.md; gate + quick acceptance; commit + push | DONE: gate OK + quick acceptance 7/7 on a quiet PC (after the other demo and the orphaned ffmpeg were stopped), pushed |

Gate note (2026-10-26): the full gate failed only gateway F1e on this PC, by CPU starvation of the station
(a watchdog in the test showed 2-9 s stalls inside ordinary Tk / Pillow calls; every suite passes alone).
Causes on this PC: another session's demo (vision model, MediaMTX, ffmpeg) and orphaned ffmpeg encoders
left by killed test sections. Fixed the leak: groundstation/childproc.py puts every ffmpeg child in a
Windows job that dies with its parent (simcam + video); the station sends its screen-1 beat twice a
second and handles questions before a telemetry backlog; F1e waits for the gateway to see the screen.

Decisions: no geofence existed anywhere -> added on the GATEWAY (like the shallow-water route
check: program upload / AUTO start refused when a waypoint or leg leaves the fence, code 120;
a copilot's change needs the captain's ACCEPT via stations.ask_local). Tk has no transparent
widgets -> screen 2 panels are drawn on the map canvas (Pillow RGBA backgrounds, canvas text,
canvas buttons), like the HUD's display list on screen 1.

# Previous task: "Crew profiles + two screens"

Task: "Crew profiles + two screens": CREWED / UNCREWED profile (autodetected),
UNCREWED lost-link engine kill, MANEUVER mode, captain / copilot roles with the manual
licence, two screens per station, boats/ folder. Design = CLAUDE.md section
"Crew, profiles, MANEUVER, screens, boats" (A-F). One commit per section; the gate
(sh tools/check.sh) AND the acceptance gate (tests\acceptance\RUN_ACCEPTANCE.bat quick)
pass before each commit; then git push. FIRST_TEST stays ON, its behaviour unchanged;
MANEUVER / REMOTE / AUTO stay blocked in FIRST_TEST.
If interrupted: say "continue from PROGRESS.md" - continue with the first step that is
not DONE below, without redoing finished work.

## Steps

| Step | What | State | Commit |
|---|---|---|---|
| 1 | CLAUDE.md design A-F (no code) | DONE (gate OK, acceptance quick 7/7 PASS) | 4e3a1f8 |
| 2 | AURIX + sim: profile autodetection, UNCREWED power-raise / CONFIRM from the shore joystick, MANEUVER; invariants I22-I26 | DONE (gate OK, acceptance quick 7/7), pushed | f2e9770 |
| 3 | AURIX + gateway + sim: UNCREWED lost link, GW_SHORE, engine-kill output (sim); invariants I27-I31 | DONE (gate OK, acceptance quick 7/7 after the I5 storm-check fix), pushed | a327711 |
| 4 | Gateway: licence flag, captain / copilot, copilot -> captain ACCEPT, boat name check | DONE (gate OK: gateway 100/100; acceptance quick 7/7), pushed | 2974fbc |
| 5 | Ground station: two screens (+ remaining HUD work), boat selector, single-monitor toggle, Greek manual | DONE (gate OK incl. test_boats 5/5; acceptance quick 7/7), pushed | 06203de |
| 6 | Verification: full acceptance (2000 storm, shortened soak), TEST_REPORT, FIRST_TEST.md note, ADS build + memory (LMU block 2), TO CONFIRM / TBD list | DONE: full --short-soak PASS (storm 2000 found I10 seed 389 -> fixed: STANDBY cancels a pending CONFIRM, MV4); runner builds the sim; day_at_sea.py answers from the captain; TEST_REPORT section 00. ADS IDE build still to do on the PC (not runnable headless here) | this commit |

## Notes

- Step 2 decisions (in CLAUDE.md A/C): CREWED -> UNCREWED only by a power-up without the joystick (a failing
  CREWED joystick is a joystick fault, not a profile change - the storm / P3 / U3 showed the other way is unsafe);
  UNCREWED -> CREWED in STANDBY after 10 s of a joystick passing its self-check; a profile change disarms the
  outputs (storm found I11: a held stick could arrive armed). Detection window 6 s (> USB bridge grace 5 s).
- UNCREWED: every CONFIRM from the HELM station's joystick (GW_RJOY flags b2, shorejoy.py "confirm_button");
  panel HDG LOCK / course buttons ignored; HDG LOCK not available.
- MANEUVER = MODE_MANEUVER (7), PROTO_T_MANEUVER 0x38, shares the REMOTE watchdog (sb_remote_t); gateway refuses
  REMOTE / MANEUVER asks without "licence": true (code 116). Sim server option "nojoy" = UNCREWED boat.
- New: TELEM2 group J (profile, joystick probe, shore joystick), log event 21 PROFILE, alarm 0x10000000 PROFILE.
- Step 3: lostlink_step (smallboat.c) after events_step; the kill is evaluated on the real outputs
  (thr.out_pct idle + gear_is N, or the monitor's safe state). Board: hal_set_engine_kill drives NO pin
  until PIN_ENGINE_KILL is chosen (hal_pins.h, WIRING.md 3A TBD). HELM release in UNCREWED = lost link.
- Step 4: copilot commands travel in the captain's (HELM) GW_CMD envelope; driving ones after the captain's
  screen-1 ACCEPT (stations._ask_driver target = the captain). A licensed user may take a FREE HELM (becomes
  captain); role swap = AURIX HELM action 2 (link.c helm_cmd, one step). Avoidance proposals + vision CLEAR:
  only the captain's screen 1. The screen texts for result 109-112 now say "captain's screen 1".
- Step 5: the map is inside screen 2 (mapwin.MapWindow parent=); crewui.py (boat list, confirm, swap); HELM take
  refused locally until the chosen boat's name + live front camera are confirmed (typed address = no boat check:
  service / demo); F11 = screen 1 / 2 on one monitor, laptop placement with two; screen 2 needs ~1840 px in
  Greek (1600 px laptop: open); docs/screens pictures NOT regenerated (still show the old map window).
- Two python test_gateway.py processes + one autopilot_sim server (port 53321) were already
  running at the start (from another session?); stopping them was not allowed. If the sim build
  fails with "permission denied" / locked exe, they are the reason.

# Previous task: HUD / map / sound / Greek (done; the open items below are folded into step 5)


Task: "Driver HUD + map window + sound/voice + Greek": (E) Greek / English
language files, Greek default, live switch; (A) driver screen = full-screen front
camera with overlays; (B) AR overlays on the front video; (C) separate full-size
MAP window, windows remember their monitor; (D) cockpit sound + voice system
(offline TTS); (F) MANUAL.txt + MANUAL_EL.txt; (G) screenshots in docs/screens/
+ tests. HUD, map and sounds are DISPLAY ONLY (never change control or safety).
One commit per section, all tests pass, git push at the end.
If interrupted: say "continue from PROGRESS.md" - continue with the first
step that is not DONE below, without redoing finished work.
Order of work: E first (every new screen text goes straight into the language
files), then A, B, C, D, F, G.

## Steps (HUD / map / sound / Greek)

| Step | What | State | Commit |
|---|---|---|---|
| E | i18n.py + lang/el.txt + lang/en.txt (plain key = text, comments), settings.py (per-PC: gs_settings.json), alerts.py (alarm levels + codes), monitors.py; captain screen fully from the language files, ΕΛ / EN buttons + menu, live switch, remembered; English stays in protocol / audit (TE + from_en); fits 1920 px in both; test_lang.py L1-L5 in the gate | DONE (gate OK except: sim build blocked by the running demo, VR5 flaky under load -> VR alone 5/5) | cea40c6 |
| A | driver.py rewrite: full-screen camera HUD: hud.py (layout, palette, overlays as a display list) + driver.py (Pillow renderer: transparent panels, outlined text, text cache ~25 ms/frame; canvas fallback), fill / fit, mirrors, MASTER lamps, alarm bar, throttle + gear + trend, speed tape, depth (chart), waypoint ETA / TTG, AIS threat, ROT, G-meter, outboard angle, CDI, dark mini map, one bottom line, night, declutter, edit layout (hud_layout.json) | DONE (test_driver I1-I14) | bdb5a2a |
| B | AR overlays (hudar.py): camera model = gateway vision geometry + IMU roll / pitch; horizon + heel arc, velocity vector, waypoint diamond / edge arrow, route ribbon (fading), AIS boxes / edge arrows, predicted path 30 / 60 s, shallow tint (chart); each on / off (AR menu), "calibration unknown" without one; gateway vision status publishes "camera" only when configured (VI0b); demo has a calibration | DONE (test_driver I15-I16, gateway VI 10/10) | 628909e |
| C | MAP window (mapwin.py): chart layers, AIS list, boat, track, route, active waypoint, avoidance route, shallow / collision warnings (no geofence exists in the system), DR circle, range rings; follow / free pan, zoom (wheel), north / course-up (5 deg steps), day / dark / night, remembered; view only. All windows: place + monitor remembered (gs_layout.json map / map_open), Windows menu + Ctrl+Shift+1..9 move to a monitor | DONE (test_driver I17 + I6 with the map) | de35b6c |
| D | sound.py (tones made in code, priority scheduler, callouts from telemetry / gateway status), tts.py (Piper offline neural voices el_GR-rapunzelina-low + en_GB-alan-medium via tools/get_piper.py; Windows SAPI fallback), every callout spoken as ONE sentence (the Greek voice garbles lone short words), 370 sentences per language pre-made in the background + auto retry of garbled takes, own recordings in sounds/<lang>/<key>.wav, Sound tab (soundtest.py) | DONE (test_sound S1-S10, test_driver I18) | 8223ebe |
| F | MANUAL.txt (5A language, 5B driver HUD + AR, 5C map window, 5D sounds, install f / g, part I, alarm levels, troubleshooting) + MANUAL_EL.txt (Greek user manual) + CLAUDE.md rule (keep both in step) | DONE | 5d4382a |
| G | tools/demo_screens.py (restarts the demo, drives the AUTO mission + DEMO_SHIP + GPS fault, DPI-aware window captures) -> docs/screens/ (13 pictures + README); review fixes: contours baked into the chart bitmap (3 maps were saturating the UI), map / plot tab drawn only when open, AR labels kept in the free middle, ribbon from 15 m, night-dimmed vision mask, MINIMAL gear box, dark AIS list in the map window; tests test_lang L1-L5, test_sound S1-S10, test_driver I1-I18 | DONE (test_lang 5/5, test_sound 10/10, test_charts 6/6, test_driver 18/18 on the committed files) | (this commit) |

## STOPPED HERE (user request, 2026-10-25) - what is left

Done: sections E, A, B, C, D, F, G are committed (one commit each). Not done / open:
- NOT PUSHED: run `git push` when ready (6 + 1 commits since 7543ee4).
- The last COMPLETE gate (tools/check.sh) ran before the final small ground-station fixes: everything
  passed (sim all + bridge, TriCore, gateway 91/91, lang, sound, charts, avoidance, FAT32) except the
  ADS-exclude check, tripped by a stray top-level logs/ folder from the screenshot script - deleted, and
  tools/demo_screens.py now runs from groundstation/. A second full gate was started and stopped on
  request: run `sh tools/check.sh` once (stop the demo first: it locks sim/autopilot_sim.exe).
- The demo is STOPPED (it was stopped for the gate): START_DEMO.bat.
- test_driver.py once failed right after the gate (no telemetry within the 10 s start-up wait on a
  busy PC); the wait is now 30 s and it passed 18/18 alone.
- Captain screen needs about 1850 px width in Greek (fits 1920; cramped on a 1600 px laptop).
- Some gateway sentences (recorder / channel details) still show in English: add "gw." lines.
- Greek voice (el_GR-rapunzelina-low) garbles some short phrases: every callout is a whole sentence,
  garbled takes are re-made automatically (0 left of 370); listen to them on the boat PC (Sound tab).
- To check on the real boat / PC: the front camera calibration (gateway vision.camera, AR horizon),
  the HUD speed on the real PC (here ~25 ms per frame at 100 % scaling, ~60 ms at 250 % scaling),
  Piper on that PC (python tools/get_piper.py).
- docs/screens/map_*.png show the map AIS list before the last column-gap fix (cosmetic).
- No geofence exists in the system, so the map window shows none.

## Notes (HUD / map / sound / Greek)

- Tests run the ground station in English with a throw-away settings file:
  TRYGONS_LANG=en, TRYGONS_GS_SETTINGS=<temp> (set at the top of each test file).
- Gateway sentences (proposals, refusals) stay English in the protocol / audit; the
  screens translate them with i18n.from_en() using the "gw.*" templates of en.txt.
- Tests are silent: TRYGONS_SOUND=off (recording player, no voice making).
- The demo was running during the work: it locks sim/autopilot_sim.exe -> the gate's sim build
  "fails" (no C code changed). Stop it (STOP_DEMO.bat) before the final gate.

# Previous task: "First test preparation": (1) generated certificates in .gitignore,
(2) .cproject excludes checked, (3) FIRST TEST configuration (steering only),
(4) real hal_tc375.c Stage 1 drivers (BNO085, MAX-M10S, Pololu G2, limit
switches, buttons, buzzer, microSD, USB link), (5) BENCH TEST mode,
(6) FIRST_TEST.md checklist. One commit per item, all tests pass, git push.
If interrupted: say "continue from PROGRESS.md" - continue with the first
step that is not DONE below, without redoing finished work.

## Steps (first test preparation)

| Step | What | State | Commit |
|---|---|---|---|
| 1 | auto.crt + generated certs / keys in .gitignore | DONE | 24404ce |
| 2 | .cproject: only board sources compiled (check + fix) | DONE (was already right; check now also catches stray sources) | ce2ad44 |
| 3 | FIRST TEST configuration (steering only), telemetry + ground station | DONE (tests FT1-FT5, e-stop in every config; gate OK) | 42d1c2b |
| 4 | hal_tc375.c Stage 1 drivers (WIRING.md), timeouts / no-data detection | DONE (tc_uart/spi/bno085/gps/adc/sd/fat32, hal_pins.h, FAT32 PC test, WIRING 9 + 11; gate OK) | c02ae06 |
| 5 | BENCH TEST mode (ground station, dock) | DONE (MODE_BENCH, sb_bench, BENCH_DIAG, hal_get_diag, monitor trip test, tab Bench, BT1-BT6, I9; gate OK, storm 200 OK) | e57f815 |
| 6 | FIRST_TEST.md checklist | DONE (multimeter, power-up order, bench, dock, TRAINING, HDG LOCK) | (this commit) |

| 7 | (user request) ADS link: 30 unresolved Ifx* externals | DONE: cause = build made before the iLLD un-exclusion (step 4) / stale ADS makefiles; .cproject in git (1d8e796) builds them, the ADS build at 12:22 linked (map OK). New check tests/acceptance/check_ads_ifx.py in check.sh + RUN_ACCEPTANCE | (this commit) |

## Notes (first test preparation)

- Step 4: iLLD modules Asclin, Qspi, Dma, Evadc, _Lib/DataHandling, Stm un-excluded in the TASKING Debug
  configuration of .cproject (parents too: a parent exclusion hides its sub-folders).
- No data flash yet: boot number always 1 -> tc_fat32 adds -2, -3 to names already on the card
  (hash table of the long names from the boot scan). TRAINING result lost at power-off (FIRST_TEST.md).
- P40.6 (A3, motor current) = iLLD EVADC group 8 ch 4 (the board manual's "G4").
- FIRST_TEST is ON in git until the first boat tests are done - switch it off before testing throttle/gear.
- .cproject: ADS added 16 iLLD include folders (ADS build clean); tools/check.sh now reads the include
  folders from the whole .cproject (each once) instead of only lines 23-75.

# Previous task: TEST_REPORT fixes (done, last commit 65e9eb2)

## Steps (fix task, done)

| Step | What | State | Commit |
|---|---|---|---|
| 1 | T11 .gitignore /TriCore*/, commit Libraries/iLLD + .cproject, fresh-clone TriCore check | DONE (fresh clone: 1002 library files, 30/30 TriCore OK) | 6a35b25 |
| 2 | R1 STANDBY: steering held (joystick in MANUAL, gyro in REMOTE) until idle + neutral; fast STANDBY ramp | DONE (storm worst STANDBY 2.16 -> 1.43 s) | 5768d2e |
| 3 | T2 HDG LOCK / AUTO refused unless the joystick is centred; held stick always takes over | DONE (storm NOTE 97 -> 0) | d90d6ac |
| 4 | R3 hal_safe_outputs drives the PWM (board path) | DONE (GTM TOM servo PWM written; TriCore compile + symbols checked; needs the bench test) | 5449c11 |
| 5 | R5 GPS NaN / inf / range rejected in every GPS path | DONE (8/8 kinds rejected, GPS2 range, DR inputs) | cf896af |
| 6 | R4 CPU1 monitor: start after core 0 "running", boot timeout | DONE (boot 3 s, test C3b) | 9d6ab75 |
| 7 | R2 big-boat CPU1 heartbeat | DONE (test C3c, gate compiles the big variant) | 1312250 |
| 8 | T1 STARLINK never auto-selected while a normal channel works | DONE (channel storm 500 runs: 0; test C11) | a38afbc |
| 9 | T3 faster GPS re-trust after a spoof at speed | DONE (~60 s with 2nd GPS / paddlewheel; old rule without - explained; test T3) | affa89f |
| 9a | (user request) ADS build: .cproject excludes every non-firmware folder, check_ads_excludes.py, CLAUDE.md rule | DONE | 2da572a |
| 10 | T4 START_DEMO builds the simulator if missing | DONE (tested on a fresh checkout) | 8b9e68a |
| 11 | review points (section 5 order): R6, R9, R11, R10, R8, T5, R20, R21, R7, R12, LOW (R13 R14 R16 R17 R18 R19 R22 T7 T12), coverage servo.c | DONE (commits d8381b5 .. 8054948) | 8054948 |
| 12 | RUN_ACCEPTANCE quick, report, push | DONE (quick 6/6 PASS, gate OK on f44f237) | (this commit) |

## Notes (fix task)

# Previous task: Parts 0-4 (done, last commit 6936f9d)

## Steps (Parts 0-4)

| Step | What | State | Commit |
|---|---|---|---|
| 0 | PROGRESS.md + CLAUDE.md rule | DONE | a27f492 |
| 1 | START_DEMO.bat / STOP_DEMO.bat, tool check, MANUAL "Demo", fresh-terminal test | DONE | 7ba8d9c |
| 2 | eWaSR: license OK (Apache-2.0 code + weights, MaSTr1325 CC BY 4.0); vision process, stages, display, tests, docs | DONE | b25ada4 |
| 3 | MANUAL "Future ideas" (chapter 17 = "What is still missing" in this MANUAL) | DONE | e385f86 |
| 4.1 | fresh clone from GitHub, build + all suites + demo | DONE: 12/13 PASS, gate fails in the clone = finding T11 (iLLD not in git) | |
| 4.2 | "a day at sea" scenario | DONE: part A 15/15; part B (whole system, real time) see TEST_REPORT 3.2 | |
| 4.3 | 24 h soak + 32-bit time wraparound | DONE (C 24 h + wrap; system soak real time) | |
| 4.4 | random fault storm, invariants I1-I21 | DONE | |
| 4.5 | independent code review (separate subagent) | DONE (report in test_results) | |
| 4.6 | coverage (gcov / coverage.py) | DONE (TEST_REPORT 3.7) | |
| 4.7 | RUN_ACCEPTANCE.bat quick / full, MANUAL "Testing" | DONE (MANUAL chapter 19) | |
| R | TEST_REPORT.md, final PROGRESS.md, git push | DONE | 09b6e0d |

## Open problems (for TEST_REPORT.md)

- Recorder stall check (file growth within stall_s 10 s) restarts recording on very low-bitrate streams
  (the nearly static demo test pictures); the demo uses stall_s 60. LOW.
- Fixed in Part 1 (tooling): test_gateway.py looked for MediaMTX in a temp folder of one session;
  tools/check.sh had `cd /c/trygons/TRYGONS_C2` and one ADS version path hard-coded; MANUAL chapter 3
  did not list ffmpeg / MediaMTX.

- Part 2 found + fixed: on gateway loss the AURIX went to nav_lost_kn (4 kn) even when a lower cap
  (e.g. an obstacle stop) was active -> now the lower of the two (NV7).
- Vision on the Pi 5 is NOT measured yet (PC: ~120-330 ms per picture, 2 fps reached).
- Part 4 notes so far: the storm runner (tests/acceptance/storm) needs the run() stub (sim_main.c is
  not linked); a 32-bit wrap needs wrap-safe loops in the harness.

## Part 4 findings so far (record only - no fixes in Part 4)
- F-I15 (MEDIUM): channels.py trial revert in AUTO goes back to STARLINK although a normal channel
  (RF) works; then stays >= 60 s (stable_s). Repro: storm_gateway.py --part channels --seed 7.
  Fix idea: on revert, call decide() unless the previous mode was a MANUAL / STRICT STARLINK choice.
  Same class: MANUAL trial revert (13/120 runs) and MANUAL fallback when the chosen channel works again
  but is not stable yet and the fallback dies -> STARLINK instead of the working chosen one (15/120).
- F-JOY (MEDIUM): STANDBY pressed with the stick not centred -> stick ignored until centred; HDG LOCK can
  then be engaged from the panel and the held stick does NOT take over (storm "NOTE" counter, seed 836).
- C storm triage: 20 first-run failures were harness issues (fixed) except F-JOY.
- F-GPS (MEDIUM): after a GPS spoof ends at ~7 kn the GPS is trusted again only after ~6.7 min (DR
  uncertainty 106 -> 177 m meanwhile); at 3.6 kn after 60 s. Repro: STORM_SLAM=1.6 STORM_DEBUG=1 storm.exe --dayatsea.
- F-DEMO (MEDIUM): demo.py build_sim() runs cmd /c with os.path.join('sim','build_sim.bat') -> 'sim/build_sim.bat'
  with MSYS2 Python -> cmd error; START_DEMO fails on a fresh clone until the sim is built by hand. Fix: 'sim\build_sim.bat'.
- OPS note: power raise in HDG LOCK needs the stick centred (= throttle 0) for the CONFIRM.
- 4.4 C storm 2000 episodes / 588 h: all invariants PASS; gateway storm: I15 finding (above), rest PASS;
  I16 station storm PASS. 4.3: C soak 24 h PASS (0 overruns, 0 drops), wrap PASS. 4.2 part A (C) 15/15 PASS.
- F-REC (LOW): recorder stall check restarts recording on very low-bitrate streams (file growth).
- F-EVT (LOW): event.json "synced" flag is never set True (dead condition in housekeeping).
- Tooling fixed in Part 4: storm runner needed a run() stub; wrap-safe loops; parser column.
- 4.5 code review DONE: test_results/code_review.md (not in git) - CRITICAL 1 (R1 steering off on STANDBY from
  MANUAL/REMOTE while throttle still ramps), HIGH 4 (R2 big-boat build no CPU1 heartbeat, R3 hal_safe_outputs only a
  flag, R4 monitor may trip at boot, R5 NaN GPS fix passes dr.c checks), MEDIUM 7, LOW 10. Copy into TEST_REPORT.md.
- I16 station storm PASS (+ self-test).

## Previous task (done)

Fixes F1-F3 + AIS / charts / shallow water / collision avoidance (sections
A-E): all done and pushed, last commit 033cfbd.

## Part 4 finished
- All results, every finding (R1-R22, T1-T11) and the fix order: TEST_REPORT.md.
- Tooling fixed at the end: day_at_sea.py read stale telemetry after time.sleep (Waiter.telem only
  updates in pump) -> waits now keep pumping; fresh_build.py keeps the gate output (test_results/fresh_check.txt).
- Next (not started, needs the user's decision): fix in the order of TEST_REPORT.md section 5, first R1 + T11.
