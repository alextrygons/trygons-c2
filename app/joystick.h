/*
 * joystick.h - Hard-wired joystick for instant manual override.
 *
 * Raw ADC values -> calibrated -1..+1 per axis, plus:
 *   - armed only after it has been seen centred (power-up check)
 *   - wire break / out of range / impossible jump -> fault (latched until
 *     the stick is valid and centred again for fault_clear_ms)
 *   - "engage" requests per axis: deflection beyond the deadband for
 *     engage_ms (engage_x / engage_y), and the TAKE button
 *   - CONFIRM button: counts only when armed, not faulted and centred while
 *     pressed (joy_confirm). It never takes or hands back control.
 *   - calibration routine (centre, min, max per axis) stored with checksum
 *
 * The joystick always wins over the autopilot and the PC; only a centred
 * stick + AUTO / HDG LOCK / STANDBY leaves MANUAL (smallboat.c).
 */
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#define JOY_CAL_MAGIC    0x4A4F5943u   /* "JOYC" */
#define JOY_CAL_VERSION  1u

typedef struct {
    uint32_t magic, version, size;
    uint16_t center[2];         /* raw ADC at rest, X / Y */
    uint16_t min[2];            /* raw at full left / full down */
    uint16_t max[2];            /* raw at full right / full up */
    uint32_t checksum;          /* CRC-32 of everything above */
} joy_cal_t;

typedef struct {
    float    deadband;          /* deflection below this counts as zero (0.15) */
    uint32_t engage_ms;         /* deflection this long takes control (100) */
    float    center_tol;        /* stick counts as centred below this (0.10) */
    uint32_t center_ms;         /* centred this long to arm at power-up (200) */
    uint16_t fault_low;         /* raw below this = wire break / short (100) */
    uint16_t fault_high;        /* raw above this = wire break / short (3995) */
    uint16_t jump_max;          /* raw change in one 50 ms step above this = impossible (2600, ~3/4 of the range:
                                 * centre->end in one step is a fast hand, end->end is not) */
    uint32_t fault_clear_ms;    /* valid + centred this long clears a fault (2000) */
} joy_config_t;

typedef struct {
    joy_config_t cfg;
    joy_cal_t    cal;
    bool     present;           /* a joystick is fitted (HAL) */
    bool     armed;             /* seen centred since power-up / last fault */
    bool     fault;             /* wire break / out of range / jump (latched) */
    uint8_t  fault_axis;        /* 1 = X, 2 = Y */
    bool     not_centered;      /* power-up: stick was not centred */
    float    x, y;              /* calibrated -1..+1 (0 while not usable) */
    uint16_t raw[2];
    bool     take, autob, confirm;              /* buttons now */
    bool     take_edge, auto_edge, confirm_edge; /* pressed this step */
    bool     engage_x, engage_y;    /* axis deflected beyond the deadband for engage_ms */
    uint32_t deflect_x_ms, deflect_y_ms, centered_ms, valid_ms;
    bool     have_last;
    bool     cal_active;
    uint16_t cal_min[2], cal_max[2];
} joy_t;

void joy_default_config(joy_config_t *c);
void joy_default_cal(joy_cal_t *cal);      /* 10 k pot with 1 k end resistors */
void joy_init(joy_t *j, const joy_config_t *c, const joy_cal_t *cal);

/* One step. in = NULL if no joystick is fitted. */
void joy_step(joy_t *j, const hal_joystick_t *in, uint32_t dt_ms);

/* Stick inside centre_tol on both axes (and usable). */
bool joy_centered(const joy_t *j);

/* CONFIRM pressed this step AND joystick armed, not faulted, centred. */
bool joy_confirm(const joy_t *j);

/* Not usable until it has been seen centred again (e.g. after STANDBY was
 * pressed with the stick deflected, so it can't grab control right back). */
void joy_disarm(joy_t *j);

/* Calibration: start (move the stick to all corners), then finish with
 * the stick released -> new calibration in *out (sealed). False if the
 * ranges are implausible. */
void joy_cal_start(joy_t *j);
bool joy_cal_finish(joy_t *j, joy_cal_t *out);

void joy_cal_seal(joy_cal_t *cal);
bool joy_cal_valid(const joy_cal_t *cal);

#endif /* JOYSTICK_H */
