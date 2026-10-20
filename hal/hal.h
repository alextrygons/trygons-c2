/*
 * hal.h - Hardware Abstraction Layer for the Trygons C2 autopilot.
 *
 * The autopilot logic (app/) talks to the hardware ONLY through these
 * functions. There are two implementations:
 *   - sim/hal_sim.c     : PC simulation (boat model)
 *   - hal/hal_tc375.c   : real AURIX TC375 board (written later)
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
    uint32_t time_ms;       /* receiver timestamp (ms of the day) */
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

/* Small non-volatile storage slots (e.g. slot 1 = joystick calibration).
 * The app adds its own version + checksum. */
#define HAL_NV_SLOT_JOYSTICK  1u
#define HAL_NV_SLOT_PROGRAM   2u     /* AUTO waypoint program */
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

#endif /* HAL_H */
