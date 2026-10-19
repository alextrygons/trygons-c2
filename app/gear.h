/*
 * gear.h - Gear servo (F / N / R) state machine.
 *
 * Safety rule: a gear change only happens at idle:
 *   throttle idle -> wait -> neutral -> wait -> new gear -> wait -> throttle
 * While a change is in progress (and in neutral) the throttle is held at
 * idle: gear_throttle_allowed() is false.
 *
 * Requests can come at any time and as fast as the user likes; the
 * machine always finishes the safe sequence and ends in the last request.
 *
 * With a neutral switch fitted, neutral must be confirmed by the switch
 * within neutral_confirm_ms; otherwise GS_FAULT (stuck servo / linkage).
 */
#ifndef GEAR_H
#define GEAR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GEAR_N = 0,
    GEAR_F,
    GEAR_R
} gear_t;

typedef struct {
    uint16_t pwm_f_us;          /* servo pulse for forward  (default 1900) */
    uint16_t pwm_n_us;          /* servo pulse for neutral  (default 1500) */
    uint16_t pwm_r_us;          /* servo pulse for reverse  (default 1100) */
    float    idle_max_pct;      /* throttle at or below this counts as idle (1 %) */
    uint32_t idle_wait_ms;      /* at idle this long before leaving a gear (1000) */
    uint32_t neutral_wait_ms;   /* in neutral this long before a new gear (1000) */
    uint32_t engage_wait_ms;    /* after engaging, before throttle is allowed (800) */
    uint32_t neutral_confirm_ms;/* neutral switch must confirm within this (2000) */
} gear_config_t;

typedef enum {
    GS_STEADY = 0,      /* in `current`, nothing to do */
    GS_WAIT_IDLE,       /* throttle going to idle / staying idle */
    GS_TO_NEUTRAL,      /* servo moving to neutral */
    GS_TO_GEAR,         /* servo moving to the new gear */
    GS_FAULT            /* neutral not confirmed: no more gear commands,
                           throttle idle, alarm (until power cycle) */
} gear_phase_t;

typedef struct {
    gear_config_t cfg;
    gear_t       current;       /* settled gear (N while changing) */
    gear_t       target;        /* last request */
    gear_t       engaging;      /* gear being engaged in GS_TO_GEAR */
    gear_phase_t phase;
    uint32_t     timer_ms;
    uint16_t     pwm_us;        /* servo output */
    uint32_t     shifts;        /* completed changes (info) */
} gear_sm_t;

void gear_default_config(gear_config_t *cfg);
void gear_init(gear_sm_t *g, const gear_config_t *cfg);

/* Ask for a gear. Any time; takes effect through the safe sequence. */
void gear_request(gear_sm_t *g, gear_t gear);

/* One step. throttle_out_pct = what is actually being sent to the throttle
 * servo now (the machine waits for it to be at idle). neutral_sw: -1 = no
 * neutral switch fitted, 0 = switch says in gear, 1 = switch says neutral. */
void gear_step(gear_sm_t *g, uint32_t dt_ms, float throttle_out_pct, int8_t neutral_sw);

/* Neutral was commanded but the switch did not confirm it: fault. */
bool gear_fault(const gear_sm_t *g);

/* May the throttle be opened? Only when settled in F or R. */
bool gear_throttle_allowed(const gear_sm_t *g);

/* Settled in `gear` with nothing pending? */
bool gear_is(const gear_sm_t *g, gear_t gear);

const char *gear_name(gear_t g);

#endif /* GEAR_H */
