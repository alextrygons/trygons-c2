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
 * app, see gear.h). */
void hal_set_gear_pwm(uint16_t pulse_us);

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

/* Small non-volatile storage slots (e.g. slot 1 = joystick calibration).
 * The app adds its own version + checksum. */
#define HAL_NV_SLOT_JOYSTICK  1u
#define HAL_NV_SLOT_PROGRAM   2u     /* AUTO waypoint program */
#define HAL_NV_SLOT_BOOT      3u     /* boot counter (log file names) */
#define HAL_NV_SLOT_JOYMAP    4u     /* joystick source + button map (joymap.c) */
bool hal_nv_save(uint8_t slot, const void *data, uint32_t size);
bool hal_nv_load(uint8_t slot, void *data, uint32_t size);

/* Optional speed sensor (paddlewheel): speed through water, kn.
 * Returns false if not fitted. */
bool hal_read_water_speed(float *kn);

/*
 * Non-volatile storage for the calibration record (an opaque block of
 * bytes; the app adds its own version and checksum). Save returns true
 * when written. Load returns true when exactly `size` bytes were read;
 * the caller must still check version and checksum.
 */
bool hal_calib_save(const void *data, uint32_t size);
bool hal_calib_load(void *data, uint32_t size);

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
 * neutral_us, steering motor off, alarm on, safety relay open. LATCHED
 * until reset: after this the normal output functions can't change these
 * outputs any more (core 0 can't override the monitor). */
void hal_safe_outputs(uint16_t throttle_idle_us, uint16_t gear_neutral_us);
bool hal_outputs_forced(void);

/* Safety relay: closed = actuators may be powered (engine kill / servo
 * power, see WIRING.md). Opened by the monitor trip; closed by core 0 once
 * the outputs are armed (neutral-start interlock). */
void hal_set_safety_relay(bool closed);

/* ENGINE KILL (UNCREWED lost link, CLAUDE.md section B): true = the relay in
 * the outboard's kill-switch circuit stops the engine. Core 0 latches it
 * (stays killed until the AURIX is reset on board - no remote restart).
 * Board: pin, relay type and fail-safe direction TBD (WIRING.md 3A). */
void hal_set_engine_kill(bool kill);

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
