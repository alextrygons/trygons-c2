/*
 * battery.c - Supply voltage filter + LOW BATTERY condition. See battery.h.
 */
#include "battery.h"

#include <math.h>
#include <string.h>

void batt_default_config(batt_config_t *c)
{
    memset(c, 0, sizeof *c);
    c->low_v = 11.8f;           /* TO CONFIRM */
    c->hyst_v = 0.3f;
    c->low_ms = 60000u;
    c->tau_slow_s = 30.0f;
    c->tau_fast_s = 1.0f;
    c->no_data_ms = 5000u;
    c->max_v = 60.0f;
}

void batt_init(batt_t *b, const batt_config_t *c)
{
    memset(b, 0, sizeof *b);
    b->cfg = *c;
}

static float lp(float y, float x, float dt_s, float tau_s)
{
    float a = (tau_s > 0.0f) ? dt_s / (tau_s + dt_s) : 1.0f;
    return y + a * (x - y);
}

void batt_step(batt_t *b, bool measured, float volts, uint32_t dt_ms)
{
    float dt = (float)dt_ms * 0.001f;
    b->low_edge = false;
    if (measured && isfinite(volts) && volts >= 0.0f && volts <= b->cfg.max_v) {
        if (!b->valid) {                        /* first sample (power-up, or after a gap): no ramp from 0 */
            b->bank_v = volts;
            b->bank_v_fast = volts;
            b->valid = true;
        } else {
            b->bank_v = lp(b->bank_v, volts, dt, b->cfg.tau_slow_s);
            b->bank_v_fast = lp(b->bank_v_fast, volts, dt, b->cfg.tau_fast_s);
        }
        b->raw_v = volts;
        b->gap_ms = 0u;
        b->no_data = false;
        b->samples++;
    } else {
        if (b->gap_ms < 0xFFFF0000u) b->gap_ms += dt_ms;
        if (b->gap_ms >= b->cfg.no_data_ms) {   /* the data is gone: nothing can be judged */
            b->no_data = true;
            b->valid = false;
            b->raw_v = 0.0f;
            b->low = false;
            b->low_ms = 0u;
        }
    }
    if (!b->valid) return;
    if (!b->low) {
        if (b->bank_v < b->cfg.low_v) {
            if (b->low_ms < 0xFFFF0000u) b->low_ms += dt_ms;
            if (b->low_ms >= b->cfg.low_ms) {
                b->low = true;
                b->low_edge = true;
                b->lows++;
            }
        } else {
            b->low_ms = 0u;
        }
    } else if (b->bank_v > b->cfg.low_v + b->cfg.hyst_v) {
        b->low = false;
        b->low_ms = 0u;
    }
}
