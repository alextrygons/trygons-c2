/*
 * throttle.h - Throttle output: limits and rate limiting.
 *
 * Every throttle change goes through here: the demand is clamped (reverse
 * power limit, gear interlock) and then rate-limited - ASYMMETRIC (owner
 * decision 2026-10-27, CLAUDE.md T, step 11e): up gently (rate_up, 15 %/s),
 * down much faster (rate_down, 50 %/s: normal driving, a planned arrival stop,
 * a lowered limit). An EMERGENCY (stop: STANDBY, e-stop, link-loss failsafe,
 * node loss, every fault STANDBY, generator GEAR FAULT) commands idle AT ONCE
 * with no rate limit - only the servo's own speed then (the model in servo.c).
 * No software limit ever slows a move toward idle.
 */
#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdbool.h>

#include "gear.h"

typedef struct {
    float max_fwd_pct;      /* forward limit (100) */
    float max_rev_pct;      /* reverse power limit (30) */
    float rate_up_pct_s;    /* opening rate (15 %/s; parameter thr_up_pc_s 2..30) */
    float rate_down_pct_s;  /* closing rate (50 %/s; parameter thr_dn_pc_s 20..200) */
} throttle_config_t;

typedef struct {
    throttle_config_t cfg;
    float out_pct;          /* what goes to the throttle servo */
    bool  stop;             /* emergency (STANDBY safe stop, link-loss failsafe ...): idle AT ONCE */
    float neutral_run_pct;  /* generator mode (section K): the throttle may open in NEUTRAL up to this
                             * (set by smallboat.c only while the generator is active, gear settled in N);
                             * 0 = the normal rule: idle in neutral */
} throttle_t;

void  throttle_default_config(throttle_config_t *cfg);
void  throttle_init(throttle_t *t, const throttle_config_t *cfg);

/* One step. demand_pct = wanted throttle; gear = gear that is settled now;
 * allowed = false forces idle (gear change in progress, fault, STANDBY).
 * Returns the new output. */
float throttle_step(throttle_t *t, float demand_pct, gear_t gear, bool allowed, float dt_s);

#endif /* THROTTLE_H */
