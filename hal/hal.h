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

/* Buttons on the control head. */
typedef enum {
    HAL_BTN_STANDBY_AUTO = 0,   /* toggles STANDBY <-> AUTO */
    HAL_BTN_MINUS_1,            /* course -1 deg  */
    HAL_BTN_PLUS_1,             /* course +1 deg  */
    HAL_BTN_MINUS_10,           /* course -10 deg */
    HAL_BTN_PLUS_10,            /* course +10 deg */
    HAL_BTN_COUNT
} hal_button_t;

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

/* Throttle servo, 0 (idle) .. 100 % (full). Clamped by the HAL. */
void hal_set_throttle(float percent);

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
    uint32_t time_ms;       /* receiver timestamp (ms of the day) */
} hal_gps_t;

/* true + new data only when a new GPS message arrived since the last call. */
bool hal_read_gps(hal_gps_t *gps);

/* Operator controls for MANEUVER (helm lever / joystick / buttons). */
typedef enum {
    HAL_SEL_N = 0,
    HAL_SEL_F,
    HAL_SEL_R
} hal_gear_sel_t;

typedef struct {
    float          steer;       /* -1 (left) .. +1 (right), 0 = centre */
    float          throttle;    /* 0 .. 1 lever position */
    hal_gear_sel_t gear;        /* gear selector */
} hal_controls_t;

/* Current control positions. Returns false if no control panel. */
bool hal_read_controls(hal_controls_t *c);

/*
 * Non-volatile storage for the calibration record (an opaque block of
 * bytes; the app adds its own version and checksum). Save returns true
 * when written. Load returns true when exactly `size` bytes were read;
 * the caller must still check version and checksum.
 */
bool hal_calib_save(const void *data, uint32_t size);
bool hal_calib_load(void *data, uint32_t size);

#endif /* HAL_H */
