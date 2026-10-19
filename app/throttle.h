/*
 * throttle.h - Throttle output: limits and rate limiting.
 *
 * Every throttle change goes through here: the demand is clamped (reverse
 * power limit, gear interlock) and then rate-limited. A separate, faster
 * (but still limited) rate is used to go down, e.g. for a safe stop.
 */
#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdbool.h>

#include "gear.h"

typedef struct {
    float max_fwd_pct;      /* forward limit (100) */
    float max_rev_pct;      /* reverse power limit (30) */
    float rate_up_pct_s;    /* opening rate (15 %/s) */
    float rate_down_pct_s;  /* closing rate (40 %/s), also for a safe stop */
} throttle_config_t;

typedef struct {
    throttle_config_t cfg;
    float out_pct;          /* what goes to the throttle servo */
} throttle_t;

void  throttle_default_config(throttle_config_t *cfg);
void  throttle_init(throttle_t *t, const throttle_config_t *cfg);

/* One step. demand_pct = wanted throttle; gear = gear that is settled now;
 * allowed = false forces idle (gear change in progress, fault, STANDBY).
 * Returns the new output. */
float throttle_step(throttle_t *t, float demand_pct, gear_t gear, bool allowed, float dt_s);

#endif /* THROTTLE_H */
