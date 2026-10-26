/*
 * servo.c - Throttle servo output with the power-limit clamp. See servo.h.
 */
#include "servo.h"

#include "hal.h"

void servo_default_config(servo_config_t *c)
{
    c->idle_us = 1000u;
    c->full_us = 2000u;
    c->rate_down_pct_s = 40.0f;
}

void servo_init(servo_t *sv, const servo_config_t *c)
{
    sv->cfg = *c;
    sv->out_pct = 0.0f;
    sv->out_us = c->idle_us;
    sv->limit_pct = 0.0f;
    sv->clamped = false;
}

uint16_t servo_pulse_for(const servo_config_t *c, float pct)
{
    float lo = (float)((c->idle_us < c->full_us) ? c->idle_us : c->full_us);
    float hi = (float)((c->idle_us < c->full_us) ? c->full_us : c->idle_us);
    if (!(pct >= 0.0f)) pct = 0.0f;     /* NaN / negative -> IDLE (not the lower pulse: on a reversed
                                         * servo, full_us < idle_us, that would be full throttle) */
    if (pct > 100.0f) pct = 100.0f;
    float us = (float)c->idle_us + pct * 0.01f * ((float)c->full_us - (float)c->idle_us);
    if (!(us >= lo)) us = lo;
    if (us > hi) us = hi;
    return (uint16_t)(us + 0.5f);
}

float servo_throttle_write(servo_t *sv, const plim_t *lim, bool forward_engaged, float pct, float dt_s,
                           float neutral_run_pct)
{
    float limit = forward_engaged ? lim->fwd_pct : lim->rev_pct;
    /* generator mode: in neutral, never above the generator cap nor the forward limit */
    if (!forward_engaged && neutral_run_pct >= 0.0f)
        limit = (neutral_run_pct < lim->fwd_pct) ? neutral_run_pct : lim->fwd_pct;
    if (limit > 100.0f) limit = 100.0f;
    if (limit < 0.0f) limit = 0.0f;
    /* just after a limit drop: may still be above it, but only coming down */
    float allowed = sv->out_pct - sv->cfg.rate_down_pct_s * dt_s;
    if (allowed < limit) allowed = limit;
    sv->clamped = pct > limit;
    if (!(pct >= 0.0f)) pct = 0.0f;     /* negative or NaN -> idle */
    if (pct > allowed) pct = allowed;
    sv->limit_pct = limit;
    sv->out_pct = pct;
    sv->out_us = servo_pulse_for(&sv->cfg, pct);
    hal_set_throttle_pwm(sv->out_us);
    return pct;
}
