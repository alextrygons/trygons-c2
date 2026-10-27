/*
 * thruster.c - The DP thrusters: clamp, rate limit, heat budget. See thruster.h.
 */
#include "thruster.h"

#include <math.h>

#include "hal.h"
#include <string.h>

void thruster_init(thruster_t *t)
{
    memset(t, 0, sizeof *t);
    t->cfg.jog_m = 1.5f;
    t->cfg.spot_alarm_m = 15.0f;
    t->why = THR_WHY_IDLE;
}

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

bool thruster_cfg_decode(const uint8_t *p, uint32_t len, thr_cfg_t *c)
{
    if (p == NULL || len != THR_FRAME_BYTES) return false;
    memset(c, 0, sizeof *c);
    for (uint8_t u = 0u; u < THR_N; u++) {
        const uint8_t *q = p + 5u * u;
        uint16_t run = rd16(q + 1), rest = rd16(q + 3);
        if (q[0] > (uint8_t)THR_CEILING_PCT || run > THR_MAX_S || rest > THR_MAX_S) return false;
        if (q[0] > 0u && run == 0u) return false;                   /* a fitted thruster needs a heat budget */
        c->unit[u].max_pct = (float)q[0];
        c->unit[u].run_s = run;
        c->unit[u].rest_s = rest;
    }
    if (p[10] < 5u || p[10] > 100u || p[11] < 5u || p[11] > 100u) return false;
    c->jog_m = (float)p[10] * 0.1f;
    c->spot_alarm_m = (float)p[11];
    return true;
}

void thruster_cfg_set(thruster_t *t, const thr_cfg_t *c, uint16_t hash)
{
    t->cfg = *c;
    for (uint8_t u = 0u; u < THR_N; u++) {
        if (!(t->cfg.unit[u].max_pct >= 0.0f)) t->cfg.unit[u].max_pct = 0.0f;
        if (t->cfg.unit[u].max_pct > THR_CEILING_PCT) t->cfg.unit[u].max_pct = THR_CEILING_PCT;
        /* a budget that is used up stays used up (the motor is hot whatever the file says):
         * heat_s at most the new budget, HOT keeps its own rest time (cool_left_s) */
        float run = (float)t->cfg.unit[u].run_s;
        if (!thruster_fitted(t, u)) continue;           /* settings without it: its budget is kept as it is */
        if (t->heat_s[u] > run) t->heat_s[u] = run;
        if (t->hot[u]) t->heat_s[u] = run;
    }
    t->have_cfg = true;
    t->cfg_hash = hash;
    t->cfg_count++;
    t->log_what = 1u;
    t->log_seq++;
}

bool thruster_fitted(const thruster_t *t, uint8_t u)
{
    return u < THR_N && t->cfg.unit[u].max_pct > 0.0f && t->cfg.unit[u].run_s > 0u;
}

float thruster_cap(const thruster_t *t, uint8_t u)
{
    if (!thruster_fitted(t, u)) return 0.0f;
    float m = t->cfg.unit[u].max_pct;
    return (m > THR_CEILING_PCT) ? THR_CEILING_PCT : m;
}

float thruster_heat(const thruster_t *t, uint8_t u)
{
    if (!thruster_fitted(t, u)) return 0.0f;
    float h = t->heat_s[u] / (float)t->cfg.unit[u].run_s;
    return (h > 1.0f) ? 1.0f : (h < 0.0f ? 0.0f : h);
}

bool thruster_any_hot(const thruster_t *t)
{
    return t->hot[THR_BOW] || t->hot[THR_STERN];
}

float thruster_load_pct(const thruster_t *t)
{
    return fabsf(t->out_pct[THR_BOW]) + fabsf(t->out_pct[THR_STERN]);
}

uint8_t thruster_flags(const thruster_t *t)
{
    return (uint8_t)((t->have_cfg ? 1u : 0u) | (t->active ? 2u : 0u) | (t->hot[THR_BOW] ? 4u : 0u) |
                     (t->hot[THR_STERN] ? 8u : 0u) | (thruster_fitted(t, THR_BOW) ? 16u : 0u) |
                     (thruster_fitted(t, THR_STERN) ? 32u : 0u));
}

static void heat_step(thruster_t *t, uint8_t u, float dt)
{
    const thr_unit_cfg_t *c = &t->cfg.unit[u];
    bool fitted = thruster_fitted(t, u);
    float run = (float)c->run_s;
    if (t->hot[u]) {
        /* HOT: held at 0 for the rest time it got when it went HOT - by the clock, whatever
         * settings arrive meanwhile (step 11b) */
        t->cool_left_s[u] -= dt;
        if (t->cool_left_s[u] <= 0.0f) {
            t->cool_left_s[u] = 0.0f;
            t->heat_s[u] = 0.0f;
            t->hot[u] = false;                                   /* rested: may run again */
            t->cool_edge = true;
            t->log_what = 3u;
            t->log_unit = u;
            t->log_seq++;
        } else if (fitted && t->cool_total_s[u] > 0.0f) {
            float h = run * t->cool_left_s[u] / t->cool_total_s[u];
            t->heat_s[u] = h > run ? run : h;                    /* (the budget shown drains with the rest) */
        }
        return;
    }
    if (!fitted) return;            /* not fitted (settings without it): the budget is kept as it is */
    float rest = (c->rest_s > 0u) ? (float)c->rest_s : run;     /* rest_s 0 = as long as it may run */
    bool running = fabsf(t->out_pct[u]) > THR_RUN_PCT;
    if (running) t->heat_s[u] += dt;
    else t->heat_s[u] -= dt * run / rest;
    if (t->heat_s[u] < 0.0f) t->heat_s[u] = 0.0f;
    if (t->heat_s[u] >= run) {
        t->heat_s[u] = run;
        t->hot[u] = true;
        t->cool_left_s[u] = rest;
        t->cool_total_s[u] = rest;
        t->hot_edge = true;
        t->log_what = 2u;
        t->log_unit = u;
        t->log_seq++;
    }
}

void thruster_step(thruster_t *t, const thr_input_t *in)
{
    float dt = (float)in->dt_ms * 0.001f;
    if (!(dt >= 0.0f) || dt > 1.0f) dt = 0.05f;
    t->hot_edge = false;
    t->cool_edge = false;
    t->active = in->dp_active && in->allowed;
    t->why = !in->dp_active ? THR_WHY_IDLE : !in->allowed ? THR_WHY_BLOCKED : THR_WHY_NONE;

    for (uint8_t u = 0u; u < THR_N; u++) {
        float want = 0.0f;
        if (t->active && thruster_fitted(t, u) && !t->hot[u]) {
            float cap = thruster_cap(t, u);
            want = in->demand_pct[u];
            if (!isfinite(want)) want = 0.0f;                    /* a broken demand = stop */
            if (want > cap) want = cap;
            if (want < -cap) want = -cap;
            /* rate limit (the stop below is never ramped) */
            float step = THR_RATE_PCT_S * dt;
            float prev = t->out_pct[u];
            if (want > prev + step) want = prev + step;
            if (want < prev - step) want = prev - step;
            t->out_pct[u] = want;
        } else {
            t->out_pct[u] = 0.0f;                                /* every stop: 0 at once */
        }
        heat_step(t, u, dt);
        if (t->hot[u]) t->out_pct[u] = 0.0f;                     /* got hot this step: 0 now */
    }
    if (t->out_pct[THR_BOW] != 0.0f || t->out_pct[THR_STERN] != 0.0f) t->run_ms += in->dt_ms;
    hal_set_thrusters(t->out_pct[THR_BOW], t->out_pct[THR_STERN]);     /* the ONE writer, every step */
}

void thruster_stop(thruster_t *t)
{
    t->out_pct[THR_BOW] = 0.0f;
    t->out_pct[THR_STERN] = 0.0f;
    t->active = false;
    hal_set_thrusters(0.0f, 0.0f);
}
