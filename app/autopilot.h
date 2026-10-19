/*
 * autopilot.h - Heading-hold autopilot logic (pure C99, uses only hal.h).
 *
 * Usage:
 *   autopilot_t ap;
 *   ap_config_t cfg;
 *   autopilot_default_config(&cfg);
 *   autopilot_init(&ap, &cfg);
 *   every AP_PERIOD_MS milliseconds:  autopilot_step(&ap);
 */
#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdbool.h>
#include <stdint.h>

/* Fixed control loop rate: 20 Hz. */
#define AP_RATE_HZ          20u
#define AP_PERIOD_MS        (1000u / AP_RATE_HZ)
#define AP_DT_S             (1.0f / (float)AP_RATE_HZ)

/* Safety */
#define AP_SENSOR_STALE_MS     500u   /* no new data this long -> freeze rudder */
#define AP_SENSOR_TIMEOUT_MS  2000u   /* no new data this long -> STANDBY + alarm */

/* Sea state 1 (calm) .. 5 (rough). Higher = more wave filtering and a
 * wider rudder deadband: less motor wear, slightly looser course. */
#define AP_SEA_STATE_MIN      1u
#define AP_SEA_STATE_MAX      5u
#define AP_SEA_STATE_DEFAULT  3u

/* Steering */
#define AP_RUDDER_LIMIT_DEG   30.0f   /* max rudder the PID may ask for */
#define AP_RUDDER_RATE_DPS    10.0f   /* max change of rudder target, deg/s */
#define AP_TURN_RATE_DPS       4.0f   /* course changes are steered at this rate */

typedef enum {
    AP_MODE_STANDBY = 0,
    AP_MODE_AUTO
} ap_mode_t;

typedef struct {
    /* Outer loop: heading error (deg) -> rudder target (deg) */
    float kp;               /* deg rudder per deg error */
    float ki;               /* deg rudder per (deg error * s) */
    float kd;               /* deg rudder per deg/s yaw rate */
    float i_limit_deg;      /* max integrator contribution, deg rudder */
    float i_zone_deg;       /* integrate only when |error| below this */

    /* Inner loop: rudder error (deg) -> motor command */
    float motor_gain;       /* motor command per deg rudder error */

    /* Sets yaw-rate wave filtering and the rudder deadband. */
    uint8_t sea_state;      /* AP_SEA_STATE_MIN .. AP_SEA_STATE_MAX */
} ap_config_t;

typedef struct {
    ap_config_t cfg;

    ap_mode_t mode;
    bool  alarm;            /* latched until Standby/Auto is pressed */

    float target_heading;   /* course set by the user, deg, 0..360 */
    float ref_heading;      /* steered course: moves to target at turn rate */
    float ref_rate;         /* current turn rate of ref_heading, deg/s */
    float heading;          /* last valid heading, deg */
    float rudder_angle;     /* last valid rudder angle, deg */
    float yaw_rate;         /* wave-filtered, deg/s */

    float integrator;       /* deg rudder */
    float rudder_target;    /* deg, after limits and rate limit */
    float motor_cmd;        /* last command sent, -1..+1 */
    bool  motor_moving;     /* deadband hysteresis state */

    bool     sensor_fault;  /* a sensor is currently timed out */
    bool     heading_valid;
    bool     rudder_valid;
    uint32_t last_heading_ms;
    uint32_t last_rudder_ms;
} autopilot_t;

/* Fill cfg with default gains/settings. */
void autopilot_default_config(ap_config_t *cfg);

/* Initialise (starts in STANDBY, motor off). Calls nothing in the HAL
 * except hal_millis(). */
void autopilot_init(autopilot_t *ap, const ap_config_t *cfg);

/* Change sea state (clamped to 1..5). Safe to call at any time. */
void autopilot_set_sea_state(autopilot_t *ap, uint8_t sea_state);

/* Run one control cycle. Call exactly every AP_PERIOD_MS. */
void autopilot_step(autopilot_t *ap);

/* Shortest signed angle from heading to target, in -180..+180 deg.
 * Positive = target is to starboard (turn right). */
float autopilot_heading_error(float target_deg, float heading_deg);

/* Wrap any angle into 0..360. */
float autopilot_wrap360(float deg);

#endif /* AUTOPILOT_H */
