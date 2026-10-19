/*
 * gear.c - Gear servo (F / N / R) state machine. See gear.h.
 */
#include "gear.h"

void gear_default_config(gear_config_t *cfg)
{
    cfg->pwm_f_us = 1900u;
    cfg->pwm_n_us = 1500u;
    cfg->pwm_r_us = 1100u;
    cfg->idle_max_pct = 1.0f;
    cfg->idle_wait_ms = 1000u;
    cfg->neutral_wait_ms = 1000u;
    cfg->engage_wait_ms = 800u;
}

static uint16_t pwm_for(const gear_sm_t *g, gear_t gear)
{
    switch (gear) {
    case GEAR_F: return g->cfg.pwm_f_us;
    case GEAR_R: return g->cfg.pwm_r_us;
    default:     return g->cfg.pwm_n_us;
    }
}

void gear_init(gear_sm_t *g, const gear_config_t *cfg)
{
    g->cfg = *cfg;
    g->current = GEAR_N;
    g->target = GEAR_N;
    g->engaging = GEAR_N;
    g->phase = GS_STEADY;
    g->timer_ms = 0u;
    g->pwm_us = cfg->pwm_n_us;
    g->shifts = 0u;
}

void gear_request(gear_sm_t *g, gear_t gear)
{
    if (gear == g->target) return;
    g->target = gear;

    switch (g->phase) {
    case GS_STEADY:
        if (gear != g->current) {
            g->phase = GS_WAIT_IDLE;
            g->timer_ms = 0u;
        }
        break;
    case GS_WAIT_IDLE:
    case GS_TO_NEUTRAL:
        /* already on the way through neutral: just the new target */
        break;
    case GS_TO_GEAR:
        /* changed our mind while engaging: back to neutral first, the
         * full safe sequence again (throttle is still idle) */
        g->phase = GS_WAIT_IDLE;
        g->timer_ms = 0u;
        break;
    }
}

void gear_step(gear_sm_t *g, uint32_t dt_ms, float throttle_out_pct)
{
    switch (g->phase) {
    case GS_STEADY:
        break;

    case GS_WAIT_IDLE:
        /* the throttle is forced to idle meanwhile; count only real idle */
        if (throttle_out_pct > g->cfg.idle_max_pct) {
            g->timer_ms = 0u;
        } else {
            g->timer_ms += dt_ms;
            if (g->timer_ms >= g->cfg.idle_wait_ms) {
                g->current = GEAR_N;            /* leaving the old gear */
                g->pwm_us = g->cfg.pwm_n_us;
                g->phase = GS_TO_NEUTRAL;
                g->timer_ms = 0u;
            }
        }
        break;

    case GS_TO_NEUTRAL:
        g->timer_ms += dt_ms;
        if (g->timer_ms >= g->cfg.neutral_wait_ms) {
            if (g->target == GEAR_N) {
                g->phase = GS_STEADY;
                g->shifts++;
            } else {
                g->engaging = g->target;
                g->pwm_us = pwm_for(g, g->target);
                g->phase = GS_TO_GEAR;
                g->timer_ms = 0u;
            }
        }
        break;

    case GS_TO_GEAR:
        g->timer_ms += dt_ms;
        if (g->timer_ms >= g->cfg.engage_wait_ms) {
            g->current = g->engaging;
            g->phase = GS_STEADY;
            g->shifts++;
            if (g->target != g->current) {      /* (request came in late) */
                g->phase = GS_WAIT_IDLE;
                g->timer_ms = 0u;
            }
        }
        break;
    }
}

bool gear_throttle_allowed(const gear_sm_t *g)
{
    return g->phase == GS_STEADY && g->current != GEAR_N;
}

bool gear_is(const gear_sm_t *g, gear_t gear)
{
    return g->phase == GS_STEADY && g->current == gear && g->target == gear;
}

const char *gear_name(gear_t g)
{
    return (g == GEAR_F) ? "F" : (g == GEAR_R) ? "R" : "N";
}
