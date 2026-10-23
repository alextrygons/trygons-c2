# Trygons C2 – independent safety code review

Date: 2026-10-23. Scope: CLAUDE.md rules vs. app/, hal/, Cpu0/1/2_Main.c, gateway/ (incl. vision), groundstation/proto.py.
Method: I read the code only. I did not run the simulator or any tests. Every finding gives the file and line I read.

## Summary (plain language)

- Overall the code is careful. Throttle writes go through one function (app/servo.c). The power limit, gear sequence, CONFIRM and HELM rules are written down in the code and mostly tested.
- **1 CRITICAL:** when the boat goes to STANDBY from MANUAL or REMOTE driving, the steering motor stops at once. The engine is still slowing down at that moment (up to about 2.5 s, plus 1 s before neutral). CLAUDE.md says "idle + neutral FIRST, then steering off". The simulator tests accept this behaviour, so it looks deliberate, but it breaks the written rule. Either the code or the rule must change.
- **4 HIGH:**
  - the big-boat build will trip the CPU1 safety monitor right after start-up, so its rudder can never move on the real board;
  - the real-board "force safe outputs" function does not yet drive the servo pulses itself;
  - the safety monitor can trip during boot if core 0 starts slowly;
  - a broken GPS value (NaN) would pass every GPS integrity check and spoil the position estimate.
- The real-board HAL (hal/hal_tc375.c) is still mostly stubs: no sensors, PWM, UARTs, flash or SD card yet. See the list below.
- No real secrets were found in the git history. Key and secret files are git-ignored. Only demo/test passwords are in the code.
- eWaSR code and weights are Apache-2.0 and the MaSTr1325 dataset is CC BY 4.0. Both allow commercial use with attribution. I checked both on the web.

Counts: CRITICAL 1, HIGH 4, MEDIUM 7, LOW 10.

## Findings

| ID | Severity | file:line | Finding | Why it matters | Proposed fix |
|---|---|---|---|---|---|
| R1 | CRITICAL | app/smallboat.c:459-475, 1810-1811 (tests accept it: sim/sim_sb.c:2224, 4152) | On STANDBY from MANUAL or REMOTE (STANDBY button, joystick fault, remote release, HELM taken on board at :1651), the steering is turned off in the same step. In those modes `ap.mode` is ANGLE, not AUTO, so `stop_steer` = NONE and `autopilot_disengage` runs. The throttle is still ramping down at 40 %/s (throttle.c:11) and the gear waits 1 s at idle (gear.c:12). | CLAUDE.md: "Every fault / timeout / STANDBY -> throttle idle + neutral FIRST, then steering off". For up to ~3.5 s the engine pushes while the outboard is left at its last angle and nobody steers it. | Hold the outboard actively until it is safe: keep ANGLE mode on the last commanded angle (or the gyro hold, as in the AUTO stop) until `safe`, then disengage. For a joystick fault, hold the last valid angle instead of following the stick. Or, if this is intended, write the MANUAL exception into CLAUDE.md explicitly. |
| R2 | HIGH | Cpu0_Main.c:75-76; app/smallboat.c:1890; app/monitor.c:97-107; Cpu1_Main.c:183-212 | In the BOAT_VARIANT_BIG build nothing increments `g_mon.heartbeat`: autopilot.c never touches g_mon, but CPU1 still runs the monitor. After 200 ms it trips MON_TRIP_HEARTBEAT, and `hal_safe_outputs` latches the rudder motor at 0 (hal_tc375.c:163). | CLAUDE.md: "The big-boat variant must keep working unchanged". On the board the big boat could never steer. No sim test covers this. | Publish the heartbeat (and safe defaults for throttle/gear) from `autopilot_step`, or give the monitor a big-boat mode. Add a sim test that runs the big-boat variant with the monitor. |
| R3 | HIGH | hal/hal_tc375.c:167-176, 279-291 | `hal_safe_outputs` only stores the idle/neutral values and sets `s_forced` (TODO at :172). The idle and neutral pulses reach the servos only when core 0 next calls `hal_set_throttle_pwm` / `hal_set_gear_pwm`. On a heartbeat trip core 0 is frozen, so it never calls them. There is also a check-then-write race: core 0 reads `s_forced`, CPU1 then forces, and core 0 writes the old pulse anyway. | Rule: "CPU1 ... forces idle + neutral + steering stop + alarm, latched in the HAL". Once the GTM PWM is written, a frozen core 0 would leave the last throttle pulse running. Only the relay and the chip watchdog would remain. | In `hal_safe_outputs`, write the idle/neutral duty straight into the GTM registers. Make the normal setters re-check `s_forced` after the write (or have CPU1 own the PWM update). Keep the relay opening. Test it on the bench. |
| R4 | HIGH | Cpu1_Main.c:196-199; Cpu0_Main.c:36-53; app/monitor.c:97-107; hal/hal_tc375.c:42, 87-95 | `IfxCpu_waitEvent(&cpuSyncEvent, 1)` has a 1 ms timeout (iLLD IfxCpu.c:620-647). CPU1 can start the monitor before core 0 finishes its init (`hal_millis()` returns 0 until `hal_init` sets the non-volatile `s_stm_ticks_per_ms`). The heartbeat timeout counts even before the first heartbeat (`started` is false). If boot takes more than 200 ms until the first `sb_step` (DFLASH loads, calibration, link/log init), the monitor trips. | This is fail-safe, but it latches the outputs safe until reset, so the boat may not start at all. It depends on the real flash timing (uncertain). | Only check the heartbeat timeout after the first heartbeat (`m->started`), with a separate boot timeout (e.g. 2 s). Make `s_stm_ticks_per_ms` volatile, or let CPU1 wait for a "HAL ready" flag. |
| R5 | HIGH | app/dr.c:277-397; app/smallboat.c:565-570; hal/hal.h:127-141 | GPS fields are never checked for NaN, infinity or range. Every integrity test is a `>` comparison (dr.c:310-328), and comparisons with NaN are false, so a NaN fix is "trusted". `d->n`/`d->e` become NaN for good while the state stays DR_GPS. AUTO then gets NaN targets: the steering freezes, arrival never happens, and GPS-loss/DR handling is never triggered. | Rule: "Every fix passes an integrity check ... a rejected fix counts as GPS failure". The NMEA parser for the board is not written yet, so this is a likely future input. | In `dr_step` and in the HAL parser, reject a fix unless `isfinite()` holds and the values are in range: abs(lat) ≤ 90, abs(lon) ≤ 180, 0 ≤ sog < 100, 0 ≤ cog < 360. Count it as a rejected fix. Add a sim test with a NaN/garbage fix. |
| R6 | MEDIUM | app/link.c:647-668; app/smallboat.c:239-244, 263-270 | The navreq HEADING is accepted with any value from 0 to 359.99°. There is no limit on how far it may differ from the leg track, and no check that a speed cap comes with it. A fresh HDG_SET is applied whenever the mode becomes HDG LOCK within its lease, even if HDG LOCK was entered after the request arrived. The driver's ACCEPT cannot be checked on the AURIX at all. | Gateway trust boundary. A faulty or compromised Pi can steer AUTO anywhere (speed only reduced), and an accepted heading can surprise the driver up to 3 s later. | On the AURIX: limit the deviation from the leg track (e.g. ±90°), require a cap for HEADING, and apply HDG_SET only if it arrived while already in HDG LOCK. Optionally add a sequence number tied to an ACCEPT token. |
| R7 | MEDIUM | app/link.c:810-819, 686-707 | GW_CMD carries the station id and the "local" flag from the Pi, and the AURIX does not check them. A local station can always take HELM. | A faulty Pi can impersonate a boat station, take HELM away from anyone and send every PC command. AUTO start, REMOTE and power raise still need the on-board CONFIRM. | Accept this as part of the design (and document it), or keep a fixed table of local station ids on the AURIX. Log every HELM change caused by a "local" flag. |
| R8 | MEDIUM | app/monitor.h:38-39; app/monitor.c:10; app/logq.c:8-11; app/logger.c:24-46 | The comment says `g_mon` "lives in core 0's DSPR ... read uncached", but there is no section pragma for g_mon or for the logq buffers. The black-box pragmas are marked "UNVERIFIED until the first ADS build". | If the linker places these in cached LMU, CPU1/CPU2 can read stale data: false monitor trips, or garbage log records. `hal_mem_barrier` (dsync) does not invalidate the reader's cache. | Place g_mon and the queue buffers explicitly in non-cached memory (or the writer's DSPR, read through the global address). Check the .map file with tools/mapreport.py after the first build. |
| R9 | MEDIUM | app/smallboat.c:1567 | Interlock: `stick = !s->joy.present || ...`. If the joystick is not present at boot (USB bridge not yet sending, analog not detected), the outputs arm without checking stick and lever. | Rule: "outputs only arm ... with the joystick centred (USB: stick centred AND lever in neutral)". The risk is reduced because the joystick itself only acts after it was centred (joystick.c:203-223). | Wait for the joystick (or a configured "no joystick" setting) before arming, or treat "joystick appears later" like a re-arm with the full interlock. |
| R10 | MEDIUM | Cpu2_Main.c:47-52 | The CPU2 watchdog is 500 ms and is serviced only after `logger_step` returns. A slow SD card write (FatFs cluster allocation, card busy) longer than that resets the whole chip. | Rule: "Logging must never block or slow control". A logger reset would stop core 0 and CPU1 too. | Use bounded, non-blocking SD writes (small chunks per step, DMA), service the watchdog between chunks, and/or make the CPU2 watchdog reset only that core. |
| R11 | MEDIUM | app/link.c:530-534, 1081-1092; app/smallboat.c:1456 | PROTO_T_ALARM_ACK from any HELM station (also a remote one through the gateway) clears all alarm bits in any mode. GATEWAY LOST / PC LINK alarms are raised only on the lost transition, so after an ACK they stay silent while the link is still down. The ACK also removes the "alarm active" block on raising the power limit. | Alarms of conditions that are still active can be hidden from the people on board. | Clear only the bits whose cause is gone (or re-raise them each step while the cause persists). Restrict ACK from remote stations. |
| R12 | MEDIUM | (untested error paths) sim/sim_sb.c, gateway/test_*.py | I found no tests for: big-boat variant + CPU1 monitor (R2), a monitor trip at boot (R4), NaN/garbage GPS (R5), a navreq HEADING far off the track (R6), ALARM_ACK while the link is still lost (R11), queue corruption (R16). | Error paths that are never run tend to hide bugs. | Add sim tests for each of these cases. |
| R13 | LOW | app/smallboat.c:168-173, 1772-1775 | The test hook `sb_test_drive` overrides the gear and throttle demand in every non-STANDBY mode. It is compiled into the firmware (only sim calls it today). | A stray call would bypass the mode logic (the power limit still applies). | Put it behind `#ifdef SIM`. |
| R14 | LOW | app/smallboat.c:471-474, 1786-1789 | Gyro/centre fault steering runs only if `cfg.fault_steer == SB_FAULT_STEER_GYRO`. With the other setting, compass loss while moving turns the steering off at once. | Rule "Compass lost while moving: keep steering on the gyro ... until idle + neutral". The default (GYRO) is fine. | Remove the option, or make the other setting centre the outboard instead of switching off. |
| R15 | LOW | app/smallboat.h:111-112 vs CLAUDE.md | SB_ALM_OBSTACLE (vision) is in SB_NAV_ADVISORY, so it does not lower the power limit. CLAUDE.md lists only AIS SILENT, COLLISION, DECISION and SHALLOW as exceptions. | The doc and the code disagree (the behaviour itself is reasonable). | Add OBSTACLE AHEAD to the exception list in CLAUDE.md. |
| R16 | LOW | app/logq.c:53-66 | `logq_pop` trusts the stored length. A corrupted length moves `tail` past `head`, and the queue stays broken (unsigned wrap at :38). | Logging would stop or produce garbage. Control is not affected. | Check `len + 2 <= head - tail` and resync/flush if it fails. |
| R17 | LOW | app/link.c:246-370 | The telemetry payload is 193 of 200 bytes. One more field sets `w.overflow`, and the telemetry is then silently never sent (:370). | The PC would see "link lost" with no clear reason. | Add a test or static assert on the telemetry size. |
| R18 | LOW | app/link.c:1059-1063; app/proto.c:45-48 | After 64 frames in one step, further bytes are still pushed into the 512-byte ring and overwrite unparsed frames. | Commands could be lost under a burst. | Stop reading the port when `frames >= 64`. |
| R19 | LOW | app/link.c:792-800 | GW_GPS2 lat/lon are not range-checked. | This only matters with gps2_check_m > 0. | Range-check them, or ignore the frame. |
| R20 | LOW | gateway/aurix.py:56-58 | `last_rx` is updated on any received byte, even garbage, not only on valid frames. | The Pi's "AURIX alive" check can be fooled by line noise. | Update it only when a frame passes the CRC. |
| R21 | LOW | gateway/vision/geometry.py:22-23, 38-39, 57-70 | There is no guard for fov = 0 (division by zero). A NaN distance makes `in_corridor` return False, so the obstacle is silently ignored. | This is advisory only, but it fails toward "no obstacle". | Validate the camera config at load time. Treat a non-finite distance as "unknown, in corridor". |
| R22 | LOW | app/smallboat.c:1882-1890; app/monitor.c:91-94 | The g_mon fields are written one by one, and the monitor reads them as a non-atomic set (throttle is written before the limit). | A false trip is possible right after a limit change (the 1 % margin makes it unlikely). | Use a sequence counter (seqlock), or write the limits before the throttle. |

Units, conversions and timers checked, no problem found: heading wrap (autopilot.c:58-72, dr.c:22-28), knots vs m/s (0.514444 used consistently in dr.c, smallboat.c:1150, geometry.py:64), deg/rad (dr.c:11-12, avoid.py), 32-bit ms differences use unsigned subtraction or a signed cast everywhere I looked (Cpu0/1/2 loops, smallboat confirm/nav/remote, powerlim, link heartbeat), uint16 remote sequence uses an int16 difference (smallboat.c:771), and CPA divides only when v² ≥ 1e-6 (avoid.py:57-59).

## hal_tc375.c stubs

| Function | Returns now | What the real board needs |
|---|---|---|
| hal_read_supply_voltage (:102) | false | EVADC divider (A2/AN37) |
| hal_sd_present (:109) | false | card detect P10.7 + f_mount |
| hal_sd_open (:114) | -1 | f_open |
| hal_sd_write / sync / close (:120-136) | false | f_write / f_sync / f_close |
| hal_sd_free_kb (:138) | 0 | f_getfree |
| hal_read_heading (:149) | false (sensor timeout alarm after 2 s) | compass driver (UART/I2C) |
| hal_read_rudder_angle (:155) | false | EVADC pot (big boat) |
| hal_set_rudder_motor (:161) | does nothing | PWM P02.5 + DIR P02.4 to the motor driver |
| hal_safe_outputs (:167) | sets latch, motor pin low, relay open, LED on; does NOT write the servo pulses | write idle/neutral PWM directly (R3) |
| hal_read_yaw_rate (:226) | false | gyro / BNO085 |
| hal_read_motor_current (:232) | false | EVADC current sense |
| hal_read_limit_switches (:238) | false | digital inputs |
| hal_read_roll_rate (:245) | false | IMU roll axis |
| hal_read_accel_z (:251) | 0 | BNO085 linear accel ring buffer |
| hal_read_attitude (:258) | false | BNO085 game rotation vector |
| hal_calib_save / load (:265-277) | false | DFLASH (IfxFlash) |
| hal_set_throttle_pwm (:279) | clamps, then discards | GTM TOM 50 Hz on P02.7 |
| hal_set_gear_pwm (:287) | discards (no range clamp) | GTM TOM 50 Hz on P02.6 (add a min/max clamp) |
| hal_read_neutral_switch (:293) | false | digital input |
| hal_read_gps (:299) | false | NMEA parser on ASCLIN + spoof/jam flag (add the R5 checks) |
| hal_read_joystick (:305) | false (no joystick, so AUTO/REMOTE/power raise are refused) | EVADC AN39/AN38 + P02.0/P02.1/P10.4 |
| hal_bridge_read (:313) | 0 | ASCLIN3 RX P20.3, 230400 |
| hal_nv_save / load (:321-331) | false (boot counter, program, joystick map are never kept) | DFLASH, one sector per slot |
| hal_read_water_speed (:333) | false | GPT12/GTM TIM pulse input |
| hal_get_button_presses (:339) | 0 (STANDBY / HDG LOCK panel buttons do nothing) | debounced digital inputs |
| hal_link_read / write (:354-366) | 0 / false | ASCLIN0 921600 with rings |
| hal_gw_read / write / tx_free (:368-386) | 0 / false / 0 | ASCLIN1 P15.0/P15.1, RX 2 KB / TX 8 KB rings |

Already real: hal_init (safe pin states first), hal_millis, hal_micros, hal_mem_barrier, hal_outputs_forced, hal_set_safety_relay, hal_watchdog_init/kick (CPU0 + safety WDT), hal_reset_reason, hal_set_alarm (LED P00.5).

Note: with no panel buttons and no joystick on the board today, the STANDBY button rule cannot work yet. hal_get_button_presses must be done before any sea trial.

## Secrets in git history

- I ran `git log --all -p` and searched it for password, passwd, secret, token, api_key, PRIVATE KEY, ssh-rsa/ed25519, AKIA, ghp_ and sk-.
  - No private keys, API tokens or real passwords were found.
  - No .key, .crt, .pem or secrets.json file was ever committed. Only `gateway/secrets.example.json` is tracked.
- `.gitignore` covers `gateway/*.key`, `gateway/*.crt` and `test_results/`. `gateway/auto.key` / `auto.crt` exist on disk but are not tracked.
- `secrets.json` is NOT listed in .gitignore. Recommendation: add `gateway/secrets.json` (and `*.key` everywhere) to it.
- Demo/test values only:
  - `demo/demo.py:38` has `USER, PASSWORD = "captain", "d***"` (demo login);
  - the test files have a test sync key `"test-sync-key-…"` and test user passwords.
  - Make sure these demo users never exist on the real gateway.

## eWaSR licence

- **eWaSR code:** Apache-2.0. Verified on github.com/tersekmatija/eWaSR: "The code in this repository and the pretrained eWaSR model weights provided here are licensed under the Apache License 2.0".
- **eWaSR weights:** also Apache-2.0, same sentence. Commercial use is allowed ("including in commercial applications and proprietary products, subject to the license terms").
- **MaSTr1325 dataset:** CC BY 4.0. Verified on box.vicos.si/borja/viamaro: "This work is licensed under CC BY 4.0". Commercial use is allowed with attribution and a citation of Bovcon et al., IROS 2019.
- gateway/vision/README.md:8-14 and tools/get_vision_model.py:7-36 state this correctly. The script downloads LICENSE-eWaSR.txt next to the model.
- The eWaSR README adds that third-party dependencies and datasets keep their own licences. If other datasets (e.g. LaRS) are used later, check them separately.

## Rules checked and found OK

- Only app/servo.c calls hal_set_throttle_pwm (grep of all app/ and Cpu*.c; tools/check.sh:54 enforces it). The servo clamps to the power limit, only comes down after a limit drop, and turns NaN or negative into idle (servo.c:34-50).
- Gear change only at idle: the gear.c state machine (WAIT_IDLE → N → wait → gear → wait) and `gear_throttle_allowed`. The CPU1 monitor also checks gear moves with the throttle open.
- Power limit: always 30 %/5 % at start (plim_init, RAM only, never saved). Raising needs a code + CONFIRM within 10 s, one step at a time; the code is single-use. Lowering happens on TAKE, STANDBY button, any new alarm, PC link lost, GPS lost, TRAINING, joystick fault and low speed. Slamming STEP and the navigation advisories are excluded as specified.
- STANDBY button wins in every mode, and the PC can only ask for STANDBY or TRAINING (link.c:425-439). MANUAL is entered only by the joystick and left only with the stick centred (precondition, smallboat.c:366-396).
- CONFIRM only answers a pending request, needs an armed and centred joystick, and never changes the mode by itself (confirm_step).
- Compass lost: AUTO/TRAINING → STANDBY with gyro hold or centring until stopped; HDG LOCK → gyro hold, then STANDBY once idle + neutral; MANUAL → alarm only.
- GPS lost in AUTO: DR at reduced speed; hold heading + slow when uncertainty > 200 m or DR > 10 min; no arrival while the uncertainty is larger than the arrival radius (smallboat.c:1155-1175).
- REMOTE: needs HELM + on-board CONFIRM, refused from MANUAL, capped at min(20/5 %, power limit), steering rate-limited, stale/old/slow data → idle + neutral + gyro hold, STANDBY after 5 s, never resumes by itself.
- Navreq: speed cap is a reduction only, applied only in AUTO; lease lost → hold heading, slow to nav_lost_kn, DRIVER DECISION alarm. The vision side only adds a speed cap and an alarm (gateway.py:242-249).
- Protocol parsers: proto.c checks the length, checks the CRC before use, has a bounded ring and bounded readers; link.c checks each command's length; jsframe.c checks the CRC; the Python FrameParser is bounded and checks the CRC.
- Watchdogs are enabled on all three cores plus the safety watchdog. Logging uses SPSC queues that drop and count when full; core 0 and CPU1 never wait.
- Sensor input sanity: heading, gyro and attitude reject NaN or out-of-range values (autopilot.c:211-234, smallboat.c:1719); remote joystick values are integers and clamped.
