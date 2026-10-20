/*
 * joystick.h - Joystick for instant manual override, whatever its source.
 *
 * Two sources fill the same joy_input_t (joy_update does the rest):
 *   - hard-wired analog joystick: joy_step() - raw ADC -> calibrated
 *     -1..+1, wire break / jump detection, calibration (below)
 *   - USB joystick through the bridge: joybridge.c (frame checks, map)
 * Common part:
 *   - armed only after it has been seen centred (power-up check)
 *   - source fault (wire break / out of range / impossible jump, or for the
 *     USB bridge: frame timeout, CRC errors, stuck counter, device
 *     unplugged, invalid value) -> fault (latched until the joystick is
 *     valid and centred again for fault_clear_ms)
 *   - optional non-returning throttle lever (USB): "centred" then also
 *     means the lever is in its neutral zone; lever moved lever_delta from
 *     its hand-back position for engage_ms = override request
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
    float    nz_lo, nz_hi;      /* lever neutral zone, 0..1 (0.45 / 0.55; set from the joymap) */
    float    lever_delta;       /* lever override distance (0.10) */
} joy_config_t;

/* Fault reasons (joy_t.fault_axis) */
#define JOY_WHY_X        1u     /* analog X wire break / jump */
#define JOY_WHY_Y        2u     /* analog Y wire break / jump */
#define JOY_WHY_TIMEOUT  3u     /* USB bridge: no valid frame for 150 ms */
#define JOY_WHY_CRC      4u     /* USB bridge: too many CRC errors */
#define JOY_WHY_SEQ      5u     /* USB bridge: sequence counter stuck */
#define JOY_WHY_DEV0     6u     /* USB bridge: stick unplugged */
#define JOY_WHY_DEV1     7u     /* USB bridge: throttle unplugged */
#define JOY_WHY_RANGE    8u     /* USB bridge: mapped axis invalid / out of range */
#define JOY_WHY_LEARN    9u     /* USB bridge: in learn mode */

/* One step of input from a source. */
typedef struct {
    bool     present;           /* a joystick is fitted / configured */
    bool     ready;             /* false = still starting up (USB enumeration): not armed, no fault yet */
    bool     bad;               /* source fault this step */
    uint8_t  why;               /* JOY_WHY_* */
    float    x, y;              /* -1..+1 (y = 0 when the throttle is a lever) */
    bool     has_lever;
    float    lever;             /* 0 (full back) .. 1 (full forward) */
    bool     take, autob, confirm;
    bool     save;              /* video SAVE button (USB joystick only) */
    uint32_t buttons;           /* panel-button functions held on the joystick (HAL_BTN_BIT mask) */
} joy_input_t;

typedef struct {
    joy_config_t cfg;
    joy_cal_t    cal;
    bool     present;           /* a joystick is fitted (HAL) */
    bool     armed;             /* seen centred since power-up / last fault */
    bool     fault;             /* wire break / out of range / jump (latched) */
    uint8_t  fault_axis;        /* fault reason JOY_WHY_* (1 = X, 2 = Y, ...) */
    bool     not_centered;      /* power-up: stick was not centred */
    float    x, y;              /* calibrated -1..+1 (0 while not usable) */
    uint16_t raw[2];
    bool     take, autob, confirm;              /* buttons now */
    bool     take_edge, auto_edge, confirm_edge; /* pressed this step */
    bool     save, save_edge;   /* video SAVE button (never changes the mode) */
    bool     engage_x, engage_y;    /* axis deflected beyond the deadband for engage_ms */
    uint32_t deflect_x_ms, deflect_y_ms, centered_ms, valid_ms;
    bool     has_lever;         /* throttle is a non-returning lever (USB) */
    float    lever;             /* 0..1, usable value (neutral-zone centre while not usable) */
    float    lever_in;          /* 0..1, as read (display) */
    float    lever_ref;         /* position at the last hand-back / arming */
    bool     lever_moved;       /* moved lever_delta from lever_ref for engage_ms */
    uint32_t deflect_l_ms;
    uint32_t buttons;           /* panel functions held on the joystick (HAL_BTN_BIT) */
    uint32_t buttons_edge;      /* ... pressed this step (STANDBY even when not armed) */
    bool     have_last;
    bool     cal_active;
    uint16_t cal_min[2], cal_max[2];
} joy_t;

void joy_default_config(joy_config_t *c);
void joy_default_cal(joy_cal_t *cal);      /* 10 k pot with 1 k end resistors */
void joy_init(joy_t *j, const joy_config_t *c, const joy_cal_t *cal);

/* Analog source: one step. in = NULL if no joystick is fitted. */
void joy_step(joy_t *j, const hal_joystick_t *in, uint32_t dt_ms);

/* Common part, any source. */
void joy_update(joy_t *j, const joy_input_t *in, uint32_t dt_ms);

/* Stick inside centre_tol on both axes AND (lever fitted) lever in its
 * neutral zone - and usable. Arming, hand-back, leaving MANUAL. */
bool joy_centered(const joy_t *j);

/* Stick alone inside centre_tol (lever ignored) - and usable. */
bool joy_stick_centered(const joy_t *j);

/* Lever position inside the neutral zone. */
bool joy_lever_neutral(const joy_t *j, float lever);

/* Lever override reference = here (call at every hand-back). */
void joy_lever_ref_here(joy_t *j);

/* CONFIRM pressed this step AND joystick armed, not faulted, stick centred. */
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
