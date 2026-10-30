/*
 * hal.h - Hardware Abstraction Layer for the Trygons C2 autopilot.
 *
 * The autopilot logic (app/) talks to the hardware ONLY through these
 * functions. There are two implementations:
 *   - sim/hal_sim.c     : PC simulation (boat model)
 *   - hal/hal_tc375.c   : real AURIX TC375 board (drivers hal/tc_*.c, pins
 *                         hal/hal_pins.h)
 *
 * Conventions (important, used everywhere):
 *   - Heading: degrees, 0 <= h < 360, 0 = North, increases clockwise.
 *   - Rudder angle: degrees, positive = boat turns to STARBOARD
 *     (heading increases). Mechanical range is about +/-35 deg.
 *   - Rudder motor command: -1.0 .. +1.0, positive moves the rudder
 *     toward positive angles. 0 = motor off.
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Mechanical rudder limit reported by the rudder angle sensor. */
#define HAL_RUDDER_MAX_DEG  35.0f

/* Buttons on the control panel (see WIRING.md).
 * Small boat: STANDBY (works in every mode, the PC can't override it),
 * HDG LOCK, -1, +1, -10, +10.
 * Big boat: button 0 toggles STANDBY <-> AUTO; HDG LOCK is not used. */
typedef enum {
    HAL_BTN_STANDBY_AUTO = 0,   /* small boat: STANDBY; big boat: STANDBY <-> AUTO */
    HAL_BTN_MINUS_1,            /* course -1 deg  */
    HAL_BTN_PLUS_1,             /* course +1 deg  */
    HAL_BTN_MINUS_10,           /* course -10 deg */
    HAL_BTN_PLUS_10,            /* course +10 deg */
    HAL_BTN_HDG_LOCK,           /* small boat: lock the current heading */
    HAL_BTN_COUNT
} hal_button_t;
#define HAL_BTN_STANDBY HAL_BTN_STANDBY_AUTO

/* Bit for a button in the mask returned by hal_get_button_presses(). */
#define HAL_BTN_BIT(b)  (1u << (unsigned)(b))

/* Microseconds from the STM timer (wraps after ~71 min; use differences). */
uint32_t hal_micros(void);

/* Initialise all hardware. Call once at startup. */
void hal_init(void);

/* Milliseconds since startup (wraps after ~49 days; use unsigned math). */
uint32_t hal_millis(void);

/*
 * Compass heading. Returns true and writes *heading_deg ONLY when a new
 * sample has arrived since the last call. Returns false if there is no
 * new data (the autopilot uses this to detect a dead sensor).
 */
bool hal_read_heading(float *heading_deg);

/*
 * Rudder angle sensor. Same rule: true + new value only when a new
 * sample has arrived since the last call.
 */
bool hal_read_rudder_angle(float *rudder_deg);

/*
 * Gyro yaw rate, deg/s, positive = turning to starboard (heading
 * increasing). Same rule: true + new value only when a new sample has
 * arrived. Returns false always if no gyro is fitted.
 */
bool hal_read_yaw_rate(float *yaw_dps);

/* Gyro roll rate, deg/s (optional sensor). Same rule as above; returns
 * false always if not available. */
bool hal_read_roll_rate(float *roll_dps);

/* Vertical linear acceleration (gravity removed), g, + up, from the BNO085
 * (slamming, app/slam.c). Copies the samples that arrived since the last
 * call (the sensor runs at 100-400 Hz) into g[0..max-1], oldest first, and
 * returns how many. 0 if not fitted / nothing new. */
uint32_t hal_read_accel_z(float *g, uint32_t max);

/* Boat attitude from the BNO085 (game rotation vector, no magnetometer):
 * roll deg (+ = starboard side down), pitch deg (+ = bow up). Only passed on
 * to the gateway (camera horizon for the obstacle detection), never used for
 * control. Returns false if not fitted / no fresh value. */
bool hal_read_attitude(float *roll_deg, float *pitch_deg);

/* Steering motor current, amps (optional current sense). Rises sharply
 * when the motor pushes against an end stop. false if not fitted. */
bool hal_read_motor_current(float *amps);

/* Optional end-stop (limit) switches of the steering, true = at the stop.
 * Returns false if not fitted. */
bool hal_read_limit_switches(bool *left, bool *right);

/* Rudder motor command, -1.0 .. +1.0 (clamped by the HAL). 0 = off. */
void hal_set_rudder_motor(float command);

/*
 * Button presses since the last call, as a bit mask of HAL_BTN_BIT().
 * Each physical press is reported once (edge detected, debounced).
 */
uint32_t hal_get_button_presses(void);

/* Alarm output (buzzer / LED). */
void hal_set_alarm(bool on);

/* ---- Small boat: engine and GPS ---------------------------------------- */

/* Throttle servo pulse width, us. ONLY app/servo.c may call this (it applies
 * the power limit and the calibrated endpoints first). The HAL clamps to
 * HAL_THROTTLE_PWM_MIN..MAX as a last guard. */
#define HAL_THROTTLE_PWM_MIN  800u
#define HAL_THROTTLE_PWM_MAX  2200u
void hal_set_throttle_pwm(uint16_t pulse_us);

/* Gear servo position as an RC-style PWM pulse width in microseconds
 * (e.g. 1100 = R, 1500 = N, 1900 = F; the positions are configured in the
 * app, see gear.h). ONLY app/servo.c calls it. DroneCAN servos (step 11e):
 * 0 = no gear command at all (no frame: the node HOLDS - the boot order and a
 * lost throttle node, docs/dronecan_design.md 5 / 6.4). */
void hal_set_gear_pwm(uint16_t pulse_us);

/*
 * DroneCAN servos (CLAUDE.md T, docs/dronecan_design.md; step 11e). true =
 * throttle, gear and steering are DroneCAN servo nodes (decided at hal_init:
 * board HAL_CAN_FITTED && HAL_CAN_SERVOS in hal_pins.h, both 0 until the owner
 * confirms; sim: the simulated CAN bus fitted). Then hal_set_throttle_pwm /
 * hal_set_gear_pwm become ArrayCommands (unitless = (us - 1000) / 500 - 1,
 * hal/can_act.c), the steering is a 360 deg POSITION servo (hal_set_steer_pos;
 * hal_set_rudder_motor does nothing) and the CPU1 trip path sends its own safe
 * frames (hal_can_safe_send). false = the PWM servos + the steering motor of
 * the old hardware (FIRST TEST, the board today).
 */
bool hal_servos_can(void);

/* The BIG boat (BOAT_VARIANT_BIG, OD-12 unchanged) has no DroneCAN servos:
 * core 0 calls this BEFORE hal_init (Cpu0_Main.c), then hal_servos_can() stays
 * false whatever the HAL CAN flags say - no actuator frame, also not CPU1's
 * trip frames (step 13f, review 94afdc4 LOW-4). The small boat never calls it. */
void hal_can_servos_off(void);

/* The steering POSITION servo (DroneCAN): pos -1..+1 of the servo travel,
 * + = the boat turns to STARBOARD (the HAL puts the wire sign on, TO CONFIRM
 * C6). command = false: NO steering frame (the node holds where it is) - after
 * every boot until a mode commands steering (owner decision OD-7: no
 * auto-centre). ONLY app/servo.c calls it. */
void hal_set_steer_pos(bool command, float pos);

/* The 10 ms actuator task (DroneCAN): core 0's main loop calls it between the
 * control steps (the sim harness every 10 ms): one ArrayCommand per commanded
 * servo, never a blocking wait (hal/can_act.c). Nothing when not fitted.
 * hal_init already sends the first tick (throttle IDLE only: a node that held
 * a pre-reset throttle closes at once, not after the boot - step 13f) and the
 * small boat's NV boot loop calls it every 10 ms until the control loop runs. */
void hal_can_tick(void);

/* Ticks of the actuator task so far (CPU1 reads it: no tick for 100 ms while
 * core 0 runs = MON_TRIP_CAN_TX; board: a word in core 0's DSPR). */
uint32_t hal_can_act_count(void);

/* What the actuator task last SENT (core 0 publishes it for CPU1's safe frames:
 * the steering / gear wire values are replayed, never made up). false = no
 * DroneCAN servos. */
typedef struct {
    float    thr_wire;
    bool     gear_have;         /* a gear frame went out since the boot */
    float    gear_wire;
    bool     steer_have;        /* a steering frame went out since the boot (OD-7) */
    float    steer_wire;
    uint32_t frames;            /* ArrayCommands queued */
} hal_can_act_t;
bool hal_can_act_state(hal_can_act_t *a);
/* the DroneCAN wire value of an RC pulse (us): (us - 1000) / 500 - 1 (TO CONFIRM C7) */
float hal_can_wire_us(uint16_t us);

/* CPU1's safe frames (monitor trip, DroneCAN servos; design 6.3): the first
 * call cancels core 0's pending actuator frames (the latch is already set by
 * hal_safe_outputs); every call sends from the RESERVED TX buffer (never core
 * 0's FIFO) throttle = thr_wire, steering = steer_wire if steer, gear =
 * gear_wire if gear (false = HOLD: no frame), with OUR node ID and core 0's
 * transfer-ID counter continued. CPU1 calls it every MON_SAFE_FRAME_MS. */
typedef struct {
    float thr_wire;             /* -1.0 = idle */
    bool  steer;                /* a steering frame (only if core 0 ever commanded steering: OD-7) */
    float steer_wire;
    bool  gear;                 /* a gear frame (false = HOLD) */
    float gear_wire;
} hal_can_safe_t;
void hal_can_safe_send(const hal_can_safe_t *f);

/* The last steering command across a reset (step 11e, OD-7): a warm reset
 * (watchdog / SMU / software) keeps it in RAM the startup does not clear (like
 * hal_kill_keep_*): the node HELD that position, so the model starts there
 * (still no frame until a mode commands steering). After a power-on it is not
 * valid (written "none"): the position is UNKNOWN (worst-case travel). */
void hal_steer_keep_set(bool valid, float pos);
bool hal_steer_keep_get(float *pos);

/* UNCREWED hardware dead-man (owner addition OA-3, docs/dronecan_design.md
 * 14.1; part / time / pin TO CONFIRM - WAITING FOR OWNER W3 / W5): a
 * retriggerable timer relay in the outboard's kill circuit, held "run" by
 * pulses; no edge for ~1.5 s = the engine stops BY ITSELF. CPU1 (monitor.c)
 * toggles the level only while UNCREWED + no trip + core 0 alive + the throttle
 * node OPERATIONAL + actuator frames going out + no kill latch. Board: P10.5
 * on the DroneCAN boat (recommended default of W5), nothing on the old PWM
 * hardware; sim: a simulated timer relay. */
void hal_deadman_pulse(bool level);

/* Optional neutral switch on the gearbox / shift linkage. Writes true if
 * the gear is in neutral. Returns false if no switch is fitted. */
bool hal_read_neutral_switch(bool *in_neutral);

/* One GPS fix (from NMEA RMC/GGA on a UART). */
typedef struct {
    double   lat_deg;       /* + north */
    double   lon_deg;       /* + east */
    float    sog_kn;        /* speed over ground */
    float    cog_deg;       /* course over ground, 0..360 */
    bool     fix;           /* position valid */
    bool     jam_spoof;     /* receiver reports jamming / spoofing (e.g. u-blox
                               spoofDetState / jamInd); false if not supported */
    uint32_t time_ms;       /* receiver timestamp (ms of the day, UTC) */
    uint32_t date;          /* UTC date as YYYYMMDD, 0 = not known yet */
    uint8_t  sats;          /* satellites used */
} hal_gps_t;

/* true + new data only when a new GPS message arrived since the last call. */
bool hal_read_gps(hal_gps_t *gps);

/* Joystick wired directly to the board (see WIRING.md): X and Y as raw
 * 12-bit ADC values (0..4095 = 0..VAREF 3.3 V; the app calibrates and
 * checks them for wire breaks), plus the TAKE, AUTO and CONFIRM buttons
 * (true = pressed). Returns false if no joystick is fitted. */
typedef struct {
    uint16_t x_raw;
    uint16_t y_raw;
    bool     take;
    bool     autob;
    bool     confirm;
} hal_joystick_t;

bool hal_read_joystick(hal_joystick_t *j);

/* USB joystick bridge (Teensy 4.1, bridge/): UART 230400 8N1, 3.3 V,
 * frames in app/jsframe.h, ~100 frames/s. Copies up to max bytes received
 * since the last call into buf and returns how many (0 = nothing new).
 * Only used when the joystick source is "USB bridge" (joymap.h). */
uint32_t hal_bridge_read(uint8_t *buf, uint32_t max);

/*
 * Small non-volatile storage slots in the data flash (board: hal/tc_nv.c =
 * an append-only record log on DFLASH0, two sectors A / B, CRC-32 per
 * record; sim: files nv<slot>.bin). The app adds its own version + checksum
 * inside the record (app/nv.c).
 *
 * hal_nv_save only QUEUES the record (copied; a newer request for the same
 * slot replaces the queued one): true = queued, false = bad slot / size or
 * the queue is full. The flash write itself runs in hal_nv_service(), called
 * once per control step: a bounded step of a state machine (issue a flash
 * command, poll busy, time limit) - NEVER a blocking wait. allowed = true
 * only in STANDBY (not stopping): a request made while driving waits for
 * STANDBY. The one write outside STANDBY is the boot counter, before the
 * control loop starts (core 0 calls hal_nv_service(true) until
 * !hal_nv_pending(), with a time limit).
 * hal_nv_load returns the newest valid record of the slot (a queued one
 * first); it needs exactly `size` bytes. A power loss during a write keeps
 * the previous record. hal_nv_ok() = the data flash works; false = NV FAULT
 * (nothing is saved any more, loads keep working). Core 0 only (one flash
 * master; never CPU1 / CPU2).
 */
#define HAL_NV_SLOT_JOYSTICK  1u
#define HAL_NV_SLOT_PROGRAM   2u     /* AUTO waypoint program */
#define HAL_NV_SLOT_BOOT      3u     /* boot counter (log file names) */
#define HAL_NV_SLOT_JOYMAP    4u     /* joystick source + button map (joymap.c) */
#define HAL_NV_SLOT_CALIB     5u     /* TRAINING result (calib.c) */
#define HAL_NV_SLOT_GENERATOR 6u     /* generator learned value (generator mode) */
#define HAL_NV_SLOT_RELAYS    7u     /* relay table (relay board) */
#define HAL_NV_MAX_SLOT       15u
#define HAL_NV_MAX_SIZE       1536u  /* bytes per record (the AUTO program is 1020) */
bool hal_nv_save(uint8_t slot, const void *data, uint32_t size);
bool hal_nv_load(uint8_t slot, void *data, uint32_t size);
void hal_nv_service(bool allowed);
bool hal_nv_pending(void);
bool hal_nv_ok(void);

/*
 * DP thrusters (CLAUDE.md section P): a bow TUNNEL thruster (bow_pct: + =
 * push the bow to starboard) and a fixed transom electric motor (stern_pct:
 * + = ahead), -100..100 % (clamped by the HAL; NaN = 0). ONLY app/thruster.c
 * calls this, once per control step (clamp, rate limit, heat budget, 0
 * outside DP; the build check greps for it). Board: RC-style pulses to
 * bidirectional controllers on GTM TOM0 (1500 us = stop, shadow registers
 * like the servos; pins / controllers TBD, HAL_THRUSTERS_FITTED 0 = no pulses
 * = stopped). The e-stop contactor cuts the thruster power in hardware. After
 * hal_safe_outputs both stay at 0 (latched). The big boat never calls it. */
void hal_set_thrusters(float bow_pct, float stern_pct);

/* Optional speed sensor (paddlewheel): speed through water, kn.
 * Returns false if not fitted. */
bool hal_read_water_speed(float *kn);

/*
 * The relay board (CLAUDE.md sections M + T). Two back-ends behind the SAME
 * calls, chosen at build time (hal_pins.h RELAY_BUS, owner decision OD-9):
 *   RELAY_BUS_MODBUS   RS485 Modbus RTU board (hal/tc_modbus.c on RELAY_ASC +
 *                      the DE pin): board-timed pulses ("flash on"), coil readback.
 *   RELAY_BUS_DRONECAN the DroneCAN relay node (hal/can_relay.c over
 *                      hal/dronecan.c, step 11f): hardpoint.Command at 50 Hz,
 *                      AURIX-timed pulses + the board's own command timeout
 *                      (<= 300 ms, R-3), readback = hardpoint.Status if the board
 *                      sends it (else the commanded states: hal_relay_readback).
 * Sim: both (a simulated Modbus slave and the simulated relay node 30). ONLY
 * app/relays.c may call these (the build check greps for it): it holds the
 * relay table, the interlocks and the time limits - and does not know the
 * bus. Nothing here is in any control path (never on CPU1).
 *   service: one bounded step of the driver's state machine, every control
 *            step (one request in flight, replies collected, time limits).
 *   set:     relay n (1..HAL_RELAY_MAX) on / off (queued; false = no room /
 *            no board fitted).
 *   pulse:   a pulse of ms that ends by itself: Modbus = BOARD-timed (100 ms
 *            steps, <= 25.5 s), DroneCAN = AURIX-timed, and the board's own
 *            command timeout switches it off when the AURIX stops sending.
 *   read:    the last readback of relay n: false = unknown (no valid reply
 *            within HAL_RELAY_LOST_MS, or n beyond the board).
 *   ok:      the board answered within HAL_RELAY_LOST_MS (false = SILENT).
 *   readback: the states of read are the board's own (false: a DroneCAN board
 *            without hardpoint.Status - read gives the COMMANDED states).
 *   channels: the relays the fitted board has (hal_pins.h RELAY_CHANNELS /
 *            CAN_RELAY_CHANNELS); 0 = no board fitted (HAL_RELAY_FITTED 0): then
 *            nothing can be switched and a silent board is NOT an alarm.
 */
#define HAL_RELAY_MAX       32u
#define HAL_RELAY_LOST_MS   3000u
void hal_relay_service(void);
bool hal_relay_set(uint8_t n, bool on);
bool hal_relay_pulse(uint8_t n, uint16_t ms);
bool hal_relay_read(uint8_t n, bool *on);
bool hal_relay_ok(void);
bool hal_relay_readback(void);
uint8_t hal_relay_channels(void);
/* step 13d: hal_relay_cut is true once when the bus driver dropped an ON it had taken for relay n (DroneCAN: the
 * board went away during the ON burst; the Modbus board: never). hal_relay_cut_hold(mask, bit n-1): for these relays
 * the driver sends NO automatic OFF after such a cut (app/relays.c: the SHUTOFF relay, whose ON it queues again for
 * RELAY_SHUTOFF_RETRY_MS). Step 13h (W42, owner 2026-10-29): the other relays get NO automatic OFF either -
 * hal_relay_on_want(mask, bit n-1) = the relays app/relays.c still wants ON (not momentary, not the engine group):
 * a cut ON of one of them is finished when the board is back, only within 10 s of the command and without a board
 * reboot; given up = hal_relay_on_drop(n) true once (relays.c clears its command; the state unknown / OFF). Modbus:
 * never cut, both no-ops. */
bool hal_relay_cut(uint8_t n);
void hal_relay_cut_hold(uint32_t mask);
void hal_relay_on_want(uint32_t mask);
bool hal_relay_on_drop(uint8_t n);

/*
 * DroneCAN bus (CLAUDE.md section T, docs/dronecan_design.md; step 11d).
 * Board: hal/tc_can.c (MCMCAN CAN0 node 0, iLLD) + hal/dronecan.c (pure C:
 * the bus state machine, libcanard hal/libcanard/, the messages, the node
 * table); HAL_CAN_FITTED 0 in hal_pins.h until the kit pins are confirmed
 * (then nothing is started: hal_can_bus() returns false). Sim: the same
 * hal/dronecan.c on a simulated bus with simulated nodes (sim/hal_sim.c).
 * Step 11d: the bus, the heartbeats (our own node 10 NodeStatus 1 Hz, every
 * node's NodeStatus received) and the node table ONLY - no actuator is
 * commanded over CAN yet (the servos stay on PWM until step 11e).
 *   service: one bounded step every control step: received frames (at most
 *            a fixed number), node table, our NodeStatus, queued frames into
 *            the controller (a full FIFO = kept for the next step, a stale
 *            queue on a dead bus = dropped + counted). Never a blocking wait.
 *   bus:     the bus state; false = no CAN fitted (no alarm then).
 *   node:    table entry i (0..HAL_CAN_NODES-1, the compile-time table of
 *            static node IDs in dronecan.h, TO CONFIRM). A REQUIRED node
 *            (throttle / steering / gear) is LOST when: silent >
 *            HAL_CAN_NODE_LOST_MS, mode not OPERATIONAL, health ERROR /
 *            CRITICAL, rebooted (uptime went back: latched until OPERATIONAL
 *            for 2 s + hal_can_ack in STANDBY), or the bus is off / error
 *            passive > 100 ms. Health WARNING = notice only (OD-6).
 *   ack:     the alarm acknowledge in STANDBY: clears the reboot latches of
 *            nodes OPERATIONAL again for 2 s (nothing else).
 * Core 0 only (CPU1's reserved TX buffer comes with step 11e).
 */
#define HAL_CAN_NODES         4u
#define HAL_CAN_NODE_LOST_MS  1500u     /* TO CONFIRM with the real NodeStatus rate (1.5 x the period) */
/* why a node is lost (hal_can_node_t.why) */
#define HAL_CAN_OK            0u
#define HAL_CAN_NEVER         1u        /* no NodeStatus since the boot (after HAL_CAN_NODE_LOST_MS) */
#define HAL_CAN_SILENT        2u        /* NodeStatus stopped */
#define HAL_CAN_NOT_OPER      3u        /* mode not OPERATIONAL */
#define HAL_CAN_HEALTH        4u        /* health ERROR / CRITICAL */
#define HAL_CAN_REBOOTED      5u        /* uptime went backwards (latched) */
#define HAL_CAN_BUS           6u        /* the bus is off / error passive > 100 ms / the controller failed */
/* node roles (hal_can_node_t.role) */
#define HAL_CAN_ROLE_THROTTLE 0u
#define HAL_CAN_ROLE_STEERING 1u
#define HAL_CAN_ROLE_GEAR     2u
#define HAL_CAN_ROLE_RELAY    3u
/* bus states (hal_can_bus_t.state) */
#define HAL_CAN_ST_OFF        0u        /* not started */
#define HAL_CAN_ST_RUN        1u        /* error active / warning */
#define HAL_CAN_ST_PASSIVE    2u        /* error passive (e.g. no node acknowledges our frames) */
#define HAL_CAN_ST_BUSOFF     3u        /* bus-off: recovery running */
#define HAL_CAN_ST_FAULT      4u        /* the controller did not start */
typedef struct {
    uint8_t  id;            /* static node ID */
    uint8_t  role;          /* HAL_CAN_ROLE_ */
    bool     required;      /* its loss = alarm + STANDBY (throttle / steering / gear) */
    bool     seen;          /* a NodeStatus arrived since the boot */
    bool     lost;
    uint8_t  why;           /* HAL_CAN_ (OK when not lost) */
    bool     warning;       /* health WARNING (notice) */
    bool     rebooted;      /* reboot latch */
    uint8_t  health, mode;  /* last NodeStatus (DroneCAN numbers) */
    uint32_t uptime_s;
    uint32_t age_ms;        /* since the last NodeStatus (0xFFFFFFFF never) */
    uint16_t restarts;      /* reboots seen */
    uint32_t statuses;      /* NodeStatus received */
    uint32_t back_s;        /* step 13k (W44): how far the uptime went back at the last restart (s) - a 1-2 s step
                               can be a stale NodeStatus delivered after a newer one, not a proven reboot */
} hal_can_node_t;
typedef struct {
    uint8_t  state;         /* HAL_CAN_ST_ */
    bool     fault;         /* off / passive > 100 ms, or the controller failed: every node lost */
    bool     id_conflict;   /* a frame FROM our own node ID 10 within 3 s (the Pixhawk on the bus, OA-4) */
    bool     anon_node;     /* a node asking for a dynamic ID within 5 s (notice "CAN node without a static ID") */
    bool     silent;        /* no frame at all received for HAL_CAN_NODE_LOST_MS */
    uint8_t  tec, rec;      /* error counters */
    uint32_t rx, tx;        /* frames received / sent */
    uint32_t tx_drops;      /* frames dropped (queue full, stale queue on a dead bus) */
    uint32_t rx_errors;     /* frames refused by the transport (bad tail / CRC / toggle) */
    uint32_t busoffs;       /* bus-off events */
} hal_can_bus_t;
void hal_can_service(void);
bool hal_can_bus(hal_can_bus_t *b);
bool hal_can_node(uint8_t i, hal_can_node_t *n);
void hal_can_ack(void);

/*
 * Ground-station link: a plain byte stream (board: USB virtual COM =
 * ASCLIN0 at 921600 baud; sim: TCP socket). Framing is done by the app
 * (proto.c).
 * read:  non-blocking, copies up to `max` received bytes, returns the count.
 * write: queues all `len` bytes and returns true, or queues nothing and
 *        returns false if there is not enough room (frame dropped).
 */
uint32_t hal_link_read(uint8_t *buf, uint32_t max);
bool     hal_link_write(const uint8_t *buf, uint32_t len);

/*
 * Gateway link (Raspberry Pi 5 on the boat LAN): the same frames as the
 * ground-station link, on a UART (board: ASCLIN1, TX P15.0 / RX P15.1,
 * 921600 8N1, 3.3 V; sim: TCP socket with a baud-rate model).
 * tx_free: bytes that fit in the transmit buffer now (the app sends its
 * log copy only into the room left after control frames).
 */
#define HAL_GW_TX_RING 8192u    /* size of the gateway UART transmit buffer */
uint32_t hal_gw_read(uint8_t *buf, uint32_t max);
bool     hal_gw_write(const uint8_t *buf, uint32_t len);
uint32_t hal_gw_tx_free(void);

/* ---- Hardening ----------------------------------------------------------- */

/* Safety monitor (CPU1) trip: throttle servo to idle_us, gear servo to
 * neutral_us, steering motor off, both DP thrusters 0, alarm on, safety relay open. LATCHED
 * until reset: after this the normal output functions can't change these
 * outputs any more (core 0 can't override the monitor). DroneCAN servos (step
 * 11e): the latch is set FIRST and core 0's pending actuator frames are
 * cancelled; the servo commands then come from CPU1's safe frames
 * (hal_can_safe_send: throttle idle at once, steering hold, gear hold until the
 * throttle travel time, then neutral); the servo power is NEVER cut (OD-2).
 * Step 13d: a pulse outside HAL_THROTTLE_PWM_MIN..MAX (0 = core 0 never
 * published) is replaced by HAL_SAFE_THR_US / HAL_SAFE_GEAR_US - never 0 /
 * the 500 us clamp (= past reverse on the gear, past full on a reversed
 * throttle); the safety relay is left alone with DroneCAN servos (P10.5 =
 * the dead-man pulse, OD-2) on the board AND in the sim. */
#define HAL_SAFE_THR_US   1000u
#define HAL_SAFE_GEAR_US  1500u
void hal_safe_outputs(uint16_t throttle_idle_us, uint16_t gear_neutral_us);
bool hal_outputs_forced(void);

/* Safety relay: closed = actuators may be powered (engine kill / servo
 * power, see WIRING.md). Opened by the monitor trip; closed by core 0 once
 * the outputs are armed (neutral-start interlock). DroneCAN servos (step 11e,
 * OD-2: never cut the servo power): nothing - P10.5 is the UNCREWED dead-man
 * pulse there (hal_deadman_pulse). */
void hal_set_safety_relay(bool closed);

/* ENGINE KILL (UNCREWED lost link, CLAUDE.md section B): true = the relay in
 * the outboard's kill-switch circuit stops the engine. Core 0 latches it
 * (stays killed until a POWER-ON reset on board - hal_kill_keep_* below - or the HELM station's
 * remote restart with idle + neutral confirmed and the e-stop released:
 * section N). Its own AURIX pin, never the relay board.
 * Board: pin, relay type and fail-safe direction TBD (WIRING.md 3A). */
void hal_set_engine_kill(bool kill);

/* The ENGINE KILL latch across a reset (step 11b): it survives a watchdog /
 * SMU / software reset and goes only with a real POWER-ON reset or the HELM
 * station's remote restart. core 0 stores the latch here every time it
 * changes; sb_init reads it back ONLY when hal_reset_reason() is not
 * HAL_RESET_POWER_ON (after a power-on the memory is not valid: written
 * "off"). Board: two words in core 0's DSPR (never cached) that the startup
 * code does not clear, a magic + its complement; sim: a variable the
 * simulated chip reset keeps. */
void hal_kill_keep_set(bool killed);
bool hal_kill_keep_get(void);

/* Controlled software reset of the whole AURIX (remote restart after a
 * monitor trip, section N): like a watchdog reset, everything starts again
 * (the monitor latch goes, neutral-start interlock, startup power limits,
 * boot counter); hal_reset_reason() then says HAL_RESET_APPLICATION.
 * Board: never returns. Sim: the harness reboots after the step. */
void hal_system_reset(void);

/* Emergency stop: the mushroom switch cuts the actuator power itself; its
 * second (normally closed) contact goes to an input so the software knows.
 * Returns false if no e-stop input is fitted / configured. */
bool hal_read_estop(bool *pressed);

/* Watchdogs (CPU0 + safety watchdog). Not kicked within timeout_ms -> chip
 * reset (outputs to their reset state, see WIRING.md). */
void hal_watchdog_init(uint32_t timeout_ms);
void hal_watchdog_kick(void);
/* Service only the CALLING core's own watchdog (CPU2's logger: between SD
 * card operations, so several slow ones in one step can't add up to a
 * watchdog reset of the whole chip). */
void hal_watchdog_kick_core(void);

/* Why the chip started (for the log). raw = chip reset status register. */
typedef enum {
    HAL_RESET_POWER_ON = 0,     /* cold / warm power-on */
    HAL_RESET_SYSTEM,           /* system reset (e.g. watchdog / SMU alarm) */
    HAL_RESET_APPLICATION,      /* application / software reset */
    HAL_RESET_DEBUG,            /* debugger */
    HAL_RESET_OTHER
} hal_reset_t;
hal_reset_t hal_reset_reason(uint32_t *raw);

/* Memory barrier: everything written before is visible to the other
 * cores before anything written after (lock-free log queues). */
void hal_mem_barrier(void);

/* Driver diagnostics for the BENCH TEST screen (never used for control):
 * what only the drivers know. Returns false if not available. */
typedef struct {
    bool     imu_ok;            /* IMU reports arriving (< 0.5 s) */
    uint8_t  imu_accuracy;      /* magnetometer calibration 0 (unreliable) .. 3 (high) */
    uint32_t imu_reports, imu_resets;
    float    accel_z_g;         /* newest vertical acceleration */
    bool     gps_ok;            /* a GPS message within 3 s */
    bool     gps_ubx;           /* u-blox binary messages (configured), else NMEA */
    uint8_t  gps_fix_type;      /* 0 none, 2 2D, 3 3D */
    uint8_t  gps_spoof;         /* spoofing: 0 unknown, 1 none, 2 indicated, 3 multiple */
    uint8_t  gps_jam;           /* jamming: 0 unknown, 1 ok, 2 warning, 3 critical */
    bool     sd_card, sd_mounted;
    uint8_t  sd_state;          /* card start-up state (board: 0 no card .. 8 ready, 9 failed) */
    uint32_t sd_free_kb;        /* 0xFFFFFFFF = not known */
    uint32_t uart_errors;       /* all UARTs: parity / frame / overflow */
} hal_diag_t;
bool hal_get_diag(hal_diag_t *d);

/* Supply voltage (battery), V. false if not measured. */
bool hal_read_supply_voltage(float *volts);

/* ---- microSD card (FAT32, logger on core 2 only) --------------------------
 * Simple file API (board: own FAT32 writer hal/tc_fat32.c; sim: a folder).
 * open = create; handles 0..HAL_SD_FILES-1, -1 = error. The board never
 * overwrites: a name already on the card gets "-2", "-3", ... (the sim
 * overwrites).
 * Data is only safe on the card after hal_sd_sync / hal_sd_close.
 * TIME BOUND: every hal_sd_* call returns within HAL_SD_MAX_MS - a card or
 * driver that is slower returns false (-> LOG FAILURE, control goes on);
 * it must never block longer (the CPU2 watchdog is 500 ms). */
#define HAL_SD_MAX_MS 200u
#define HAL_SD_FILES 2
bool     hal_sd_present(void);
int      hal_sd_open(const char *name);
bool     hal_sd_write(int h, const void *data, uint32_t len);
bool     hal_sd_sync(int h);
bool     hal_sd_close(int h);
uint32_t hal_sd_free_kb(void);

#endif /* HAL_H */
