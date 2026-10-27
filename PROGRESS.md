# PROGRESS

Current task: "C2 extensions + Academy" (design = CLAUDE.md section "C2 extensions + Academy", parts
G-S; academy spec = docs/ACADEMY_SPEC.md). Steps 0-14 = C2 extensions (alarms in one place, boat
dropdown, data flash, supply voltage, generator mode, boat file v2, Modbus relay board, engine controls +
remote restart, relay buttons, DP parts 1-3, x86 gateway, open items, final check), 15-22 = Academy.
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
| 8 | Relay buttons on screen 2 (collapsible panel) and screen 1 ("show on driving screen") | | |
| 9 | DP part 1: sim thrusters + app/thruster.c (only writer, clamp, budget), CPU1 + hal_safe_outputs -> 0, e-stop contactor in WIRING | | |
| 10 | DP part 2: MODE_DP (SPOT / JOG / CRUISE / TRIP) + control, invariants I39-I43 | | |
| 11 | DP part 3: ground station controls + HUD, gateway shallow / collision / geofence checks for DP trips | | |
| 12 | Gateway on a rugged x86 mini PC: remove Pi-only parts, 3.3 V USB-UART (never RS232), USB AIS receiver; README / PARTS / WIRING | | |
| 13 | Open items: AUTO return, docs/screens pictures (tools/demo_screens.py), paddlewheel, analog joystick | | |
| 14 | Final check: full acceptance + 24 h soak, TEST_REPORT | | |
| 15 | Academy design + "no boat link" tests | | |
| 16 | Academy A (accounts, levels, gateway push) | | |
| 17 | Academy B (guided tour) | | |
| 18 | Academy C + D (sim lessons, quizzes) | | |
| 19 | Academy E (emergency scenarios) | | |
| 20 | Academy F (real-trip video lessons + editor) | | |
| 21 | Academy G + J (exams, practical test, online academy) | | |
| 22 | Academy H + I (3D sim interface, manual + tests + screenshots) + final report | | |

## Notes

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
