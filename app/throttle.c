/*
 * throttle.c - Throttle output: limits and rate limiting. See throttle.h.
 */
#include "throttle.h"

void throttle_default_config(throttle_config_t *cfg)
{
    cfg->max_fwd_pct = 100.0f;
    cfg->max_rev_pct = 30.0f;
    cfg->rate_up_pct_s = 15.0f;
    cfg->rate_down_pct_s = 40.0f;
    cfg->rate_stop_pct_s = 100.0f;
}

void throttle_init(throttle_t *t, const throttle_config_t *cfg)
{
    t->cfg = *cfg;
    t->out_pct = 0.0f;
    t->stop = false;
}

float throttle_step(throttle_t *t, float demand_pct, gear_t gear, bool allowed, float dt_s)
{
    float target = demand_pct;
    if (!allowed || gear == GEAR_N) target = 0.0f;
    if (gear == GEAR_R && target > t->cfg.max_rev_pct) target = t->cfg.max_rev_pct;
    if (gear == GEAR_F && target > t->cfg.max_fwd_pct) target = t->cfg.max_fwd_pct;
    if (target < 0.0f) target = 0.0f;
    if (target > 100.0f) target = 100.0f;

    float up = t->cfg.rate_up_pct_s * dt_s;
    float down = (t->stop && t->cfg.rate_stop_pct_s > t->cfg.rate_down_pct_s ? t->cfg.rate_stop_pct_s
                                                                            : t->cfg.rate_down_pct_s) * dt_s;
    if (target > t->out_pct + up)        t->out_pct += up;
    else if (target < t->out_pct - down) t->out_pct -= down;
    else                                 t->out_pct = target;
    return t->out_pct;
}
