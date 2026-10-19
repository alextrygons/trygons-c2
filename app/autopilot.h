/*
 * autopilot.h - Heading-hold autopilot logic (pure C99, uses only hal.h).
 *
 * Two steering variants, chosen in ap_config_t.steering:
 *   AP_STEER_RUDDER_FEEDBACK  big boat: rudder angle sensor, proportional motor
 *   AP_STEER_OUTBOARD         small boat: NO angle sensor, slow on/off motor
 *                             with backlash; the angle is estimated
 *
 * Usage:
 *   autopilot_t ap;
 *   ap_config_t cfg;
 *   autopilot_default_config(&cfg);          (big boat)
 *   or autopilot_default_config_outboard(&cfg);  (small boat)
 *   autopilot_init(&ap, &cfg);
 *   every AP_PERIOD_MS milliseconds:  autopilot_step(&ap);
 */
#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdbool.h>
#include <stdint.h>

#include "calib.h"
#include "training.h"

/* Fixed control loop rate: 20 Hz. */
#define AP_RATE_HZ          20u
#define AP_PERIOD_MS        (1000u / AP_RATE_HZ)
#define AP_DT_S             (1.0f / (float)AP_RATE_HZ)

/* Safety */
#define AP_SENSOR_STALE_MS     500u   /* no new data this long -> freeze rudder */
#define AP_SENSOR_TIMEOUT_MS  2000u   /* no new data this long -> STANDBY + alarm */
#define AP_OFF_COURSE_DEG     20.0f   /* this far off the steered course ... */
#define AP_OFF_COURSE_MS     30000u   /* ... for this long -> alarm (stays in AUTO) */

/* Outboard: the integrator is slowly moved into the angle estimate so the
 * estimate can't drift away (e.g. motor faster one way than the other). */
#define AP_OUTBOARD_RECENTER_TAU_S  60.0f

/* Gyro hold (compass lost while moving): rudder = start - KP * yaw rate
 * - KI * integral(yaw rate), i.e. steer for zero turn rate. */
#define AP_GYRO_HOLD_KP   2.0f    /* deg rudder per deg/s */
#define AP_GYRO_HOLD_KI   1.0f    /* deg rudder per deg of heading change */

/* Calibration -> settings. The outboard gains were tuned on a reference
 * boat (sim at 6 kn) that turns K_REF deg/s per degree of outboard. A
 * calibrated boat gets all gains scaled by K_REF / its own K (same loop
 * gain), and its turn rate is limited to what a quarter of its outboard
 * angle can do. (Tested in sim against other rules: this one held course
 * best; using the time constant too made it worse.) */
#define AP_CALIB_REF_K          0.3f
#define AP_CALIB_TURN_FRACTION  0.25f
#define AP_CALIB_MARGIN_DEG     3.0f    /* keep max_angle this far from the end stop */

/* Sea state 1 (calm) .. 5 (rough). Higher = more wave filtering and a
 * wider rudder deadband: less motor wear, slightly looser course. */
#define AP_SEA_STATE_MIN      1u
#define AP_SEA_STATE_MAX      5u
#define AP_SEA_STATE_DEFAULT  3u

/* Steering */
#define AP_RUDDER_LIMIT_DEG   30.0f   /* max rudder the PID may ask for (big boat) */
#define AP_RUDDER_RATE_DPS    10.0f   /* max change of rudder target, deg/s */

typedef enum {
    AP_MODE_STANDBY = 0,
    AP_MODE_AUTO,
    AP_MODE_TRAINING,       /* outboard calibration, see training.h */
    AP_MODE_MANUAL,         /* steering motor follows the operator (MANEUVER) */
    AP_MODE_GYRO_HOLD,      /* no compass: steer for zero yaw rate (gyro) */
    AP_MODE_CENTER          /* no compass, no gyro: drive the outboard to centre */
} ap_mode_t;

typedef enum {
    AP_STEER_RUDDER_FEEDBACK = 0,   /* big boat (rudder angle sensor) */
    AP_STEER_OUTBOARD               /* small boat (no angle sensor)   */
} ap_steering_t;

/*
 * Small-boat outboard steering settings (used when steering ==
 * AP_STEER_OUTBOARD). Change them while running with
 * autopilot_set_outboard_config(); invalid values are rejected.
 */
typedef struct {
    /* Maximum outboard angle each side, deg. The angle estimate and the
     * PID's request are both clamped to this. Default 30. */
    float max_angle_deg;

    /* How fast the motor turns the outboard, deg/s. Used to turn motor run
     * time into an angle estimate. Measure it on the boat. Default 3. */
    float turn_speed_deg_s;

    /* Dead gap (backlash) when the motor changes direction, in degrees of
     * outboard angle: the motor runs but the outboard doesn't move until
     * the gap is taken up. Extra run time gap/turn_speed is added on every
     * reversal and not counted as movement. Measure it. Default 2. */
    float gap_deg;

    /* Shortest motor pulse that really moves the outboard, ms. Pulses are
     * never shorter, and the pause between pulses is at least this long
     * too, so the motor never buzzes. Default 100. */
    uint32_t min_pulse_ms;
} ap_outboard_config_t;

typedef struct {
    /* Outer loop: heading error (deg) -> rudder target (deg) */
    float kp;               /* deg rudder per deg error */
    float ki;               /* deg rudder per (deg error * s) */
    float kd;               /* deg rudder per deg/s yaw rate */
    float i_limit_deg;      /* max integrator contribution, deg rudder */
    float i_zone_deg;       /* integrate only when |error| below this */
    float turn_rate_dps;    /* course changes are steered at this rate */

    /* Inner loop (big boat): rudder error (deg) -> motor command */
    float motor_gain;       /* motor command per deg rudder error */

    /* Sets yaw-rate wave filtering and the rudder deadband. */
    uint8_t sea_state;      /* AP_SEA_STATE_MIN .. AP_SEA_STATE_MAX */

    ap_steering_t        steering;
    ap_outboard_config_t outboard;   /* only used with AP_STEER_OUTBOARD */
} ap_config_t;

typedef struct {
    ap_config_t cfg;

    ap_mode_t mode;
    bool  alarm;            /* latched until Standby/Auto is pressed */

    float target_heading;   /* course set by the user, deg, 0..360 */
    float ref_heading;      /* steered course: moves to target at turn rate */
    float ref_rate;         /* current turn rate of ref_heading, deg/s */
    float heading;          /* last valid heading, deg */
    float rudder_angle;     /* last valid rudder angle, deg (big boat) */
    float yaw_rate;         /* wave-filtered, deg/s */

    float integrator;       /* deg rudder */
    float rudder_target;    /* deg, after limits and rate limit */
    float motor_cmd;        /* last command sent, -1..+1 */
    bool  motor_moving;     /* deadband hysteresis state (big boat) */

    /* Outboard angle estimate and pulse state (small boat) */
    float    est_angle;     /* estimated outboard angle, deg (0 at AUTO) */
    float    slack_deg;     /* where the motor sits inside the backlash gap:
                               +gap/2 = engaged pushing right, -gap/2 =
                               engaged pushing left, 0 = middle (unknown) */
    int8_t   motor_dir;     /* -1, 0, +1: what the motor is doing now */
    uint32_t pulse_ms;      /* how long the current pulse has run */
    uint32_t pause_ms;      /* how long the motor has been off */
    uint32_t last_step_ms;

    uint32_t off_course_ms; /* how long we have been far off course */

    /* Set by the small-boat controller (speed dependent); neutral = 0 / 1 */
    float    steer_limit_deg; /* extra steering angle limit, 0 = none */
    float    gain_scale;      /* multiplies kp, ki, kd (1 = as configured) */

    float    gyro_base;       /* gyro hold: rudder at the start, deg */
    float    gyro_integ;      /* gyro hold: integral part, deg */
    bool     stale_gyro;      /* AUTO with stale compass: steering on the gyro */
    bool     gyro_assist;     /* allowed to steer on the gyro when the compass is stale (set by smallboat.c) */

    /* Gyro (optional) and training */
    float    gyro_dps;      /* last gyro yaw rate (raw) */
    bool     gyro_valid;
    uint32_t last_gyro_ms;
    bool     heading_new;   /* new compass sample this step */
    training_t train;
    bool     calibrated;    /* outboard settings come from a calibration */

    bool     sensor_fault;  /* a sensor is currently timed out */
    bool     heading_valid;
    bool     rudder_valid;
    uint32_t last_heading_ms;
    uint32_t last_rudder_ms;
} autopilot_t;

/* Fill cfg with default settings: big boat with rudder feedback. */
void autopilot_default_config(ap_config_t *cfg);

/* Fill cfg with default settings: small boat, outboard, no feedback. */
void autopilot_default_config_outboard(ap_config_t *cfg);

/* Initialise (starts in STANDBY, motor off). Calls nothing in the HAL
 * except hal_millis(). Invalid outboard settings are replaced by defaults. */
void autopilot_init(autopilot_t *ap, const ap_config_t *cfg);

/* Change sea state (clamped to 1..5). Safe to call at any time. */
void autopilot_set_sea_state(autopilot_t *ap, uint8_t sea_state);

/* Change outboard settings while running. Returns false (and keeps the
 * old settings) if a value is out of range. */
bool autopilot_set_outboard_config(autopilot_t *ap, const ap_outboard_config_t *ob);

/* Start TRAINING (outboard only, from STANDBY, compass working).
 * Conditions: flat water, no current, slow constant speed, outboard
 * centred by hand. Standby/Auto aborts at any time. Returns false if
 * training is not allowed now. */
bool autopilot_start_training(autopilot_t *ap);

/* After training reaches TR_RESULT: accept (save + use) or reject.
 * full_angle_deg = measured lock-to-lock outboard angle, 0 if unknown
 * (then degrees are nominal, based on the configured turn speed).
 * Returns true if the calibration was accepted and applied. */
bool autopilot_confirm_training(autopilot_t *ap, bool accept, float full_angle_deg);

/* Turn a calibration into outboard settings + scaled gains. */
void autopilot_apply_calibration(autopilot_t *ap, const calib_data_t *c);

/* Load a saved calibration (if valid) and apply it. */
bool autopilot_load_calibration(autopilot_t *ap);

/* Run one control cycle. Call exactly every AP_PERIOD_MS.
 * Reads the sensors and buttons through the HAL, computes, and writes the
 * steering motor and alarm through the HAL. */
void autopilot_step(autopilot_t *ap);

/* Everything the autopilot reads in one cycle. */
typedef struct {
    uint32_t now_ms;
    bool     gyro_new;    float gyro_dps;      /* new gyro sample? */
    bool     heading_new; float heading_deg;   /* new compass sample? */
    bool     rudder_new;  float rudder_deg;    /* new rudder sample? (big boat) */
    uint32_t buttons;                          /* HAL_BTN_BIT() presses */
    int8_t   manual_steer;                     /* -1 / 0 / +1, used in AP_MODE_MANUAL */
} ap_inputs_t;

/* The same cycle without any HAL access: the caller supplies the inputs
 * and sends ap->motor_cmd / ap->alarm itself. Used by the small-boat
 * controller (smallboat.c), which owns all outputs. */
void autopilot_update(autopilot_t *ap, const ap_inputs_t *in);

/* Heading hold on the current heading (AUTO). keep_estimate: the outboard
 * angle estimate is kept (outboard not centred by hand). Returns false if
 * the sensors aren't OK. */
bool autopilot_engage(autopilot_t *ap, bool keep_estimate);

/* Manual steering (AP_MODE_MANUAL): the motor runs while manual_steer is
 * held, never past max_angle (estimate), pulses >= min_pulse_ms.
 * keep_estimate: false = the outboard was centred by hand. */
void autopilot_manual(autopilot_t *ap, bool keep_estimate);

/* Compass lost while moving: steer for zero yaw rate with the gyro
 * (AP_MODE_GYRO_HOLD), starting from where the rudder / outboard is. */
void autopilot_gyro_hold(autopilot_t *ap);

/* No compass and no gyro: drive the outboard to its estimated centre
 * (AP_MODE_CENTER). autopilot_centered() tells when it is there. */
void autopilot_center(autopilot_t *ap);
bool autopilot_centered(const autopilot_t *ap);

/* Gyro data fresh (last sample < AP_SENSOR_STALE_MS old)? */
bool autopilot_gyro_ok(const autopilot_t *ap);

/* STANDBY: steering motor off. */
void autopilot_disengage(autopilot_t *ap);

/* New course to steer (AUTO); reached at the configured turn rate. */
void autopilot_set_target(autopilot_t *ap, float heading_deg);

/* Shortest signed angle from heading to target, in -180..+180 deg.
 * Positive = target is to starboard (turn right). */
float autopilot_heading_error(float target_deg, float heading_deg);

/* Wrap any angle into 0..360. */
float autopilot_wrap360(float deg);

#endif /* AUTOPILOT_H */
