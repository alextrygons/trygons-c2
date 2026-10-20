/*
 * powerlim.c - Throttle power limit. See powerlim.h.
 */
#include "powerlim.h"

#include <string.h>

void plim_default_config(plim_config_t *c)
{
    c->fwd_steps[0] = 30.0f;
    c->fwd_steps[1] = 50.0f;
    c->fwd_steps[2] = 75.0f;
    c->fwd_steps[3] = 100.0f;
    c->rev_start = 5.0f;
    c->rev_step = 5.0f;
    c->rev_max = 15.0f;
    c->code_ms = 60000u;
    c->low_speed_kn = 3.0f;
    c->low_speed_ms = 120000u;
}

static void log_add(plim_t *p, uint32_t now, float of, float orv, plim_source_t src, uint8_t dir,
                    plim_result_t r, bool attempt)
{
    plim_log_t *e = &p->log[p->log_count % PLIM_LOG_N];
    e->t_ms = now;
    e->old_fwd = of;
    e->old_rev = orv;
    e->new_fwd = p->fwd_pct;
    e->new_rev = p->rev_pct;
    e->source = (uint8_t)src;
    e->dir = dir;
    e->result = (uint8_t)r;
    e->attempt = attempt ? 1u : 0u;
    p->log_count++;
}

void plim_init(plim_t *p, const plim_config_t *c, uint32_t seed)
{
    memset(p, 0, sizeof *p);
    p->cfg = *c;
    p->fwd_step = 0u;
    p->fwd_pct = c->fwd_steps[0];
    p->rev_pct = c->rev_start;
    p->rng = seed ? seed : 0x1234567u;
    log_add(p, 0u, p->fwd_pct, p->rev_pct, PLIM_SRC_STARTUP, 0u, PLIM_OK, false);
}

uint16_t plim_code_request(plim_t *p, uint8_t dir, uint32_t now_ms)
{
    /* xorshift, mixed with the time: not secret, just "type what you see" */
    p->rng ^= now_ms * 2654435761u;
    p->rng ^= p->rng << 13;
    p->rng ^= p->rng >> 17;
    p->rng ^= p->rng << 5;
    p->code = (uint16_t)(1000u + p->rng % 9000u);   /* 1000..9999 */
    p->code_dir = dir ? 1u : 0u;
    p->code_until_ms = now_ms + p->cfg.code_ms;
    return p->code;
}

plim_result_t plim_code_check(plim_t *p, uint8_t dir, uint16_t code, uint32_t now_ms)
{
    bool ok = p->code != 0u && code == p->code && (dir ? 1u : 0u) == p->code_dir &&
              (int32_t)(p->code_until_ms - now_ms) > 0;
    p->code = 0u;                                   /* used up either way */
    plim_result_t r = PLIM_OK;
    if (!ok) r = PLIM_ERR_WRONG_CODE;
    else if (dir == 0u && p->fwd_step + 1u >= PLIM_FWD_STEPS) r = PLIM_ERR_AT_MAX;
    else if (dir != 0u && p->rev_pct >= p->cfg.rev_max - 0.01f) r = PLIM_ERR_AT_MAX;
    if (r != PLIM_OK) log_add(p, now_ms, p->fwd_pct, p->rev_pct, PLIM_SRC_PC, dir, r, true);
    return r;
}

plim_result_t plim_step_up(plim_t *p, uint8_t dir, uint32_t now_ms)
{
    float of = p->fwd_pct, orv = p->rev_pct;
    plim_result_t r = PLIM_OK;
    if (dir == 0u) {
        if (p->fwd_step + 1u >= PLIM_FWD_STEPS) r = PLIM_ERR_AT_MAX;
        else p->fwd_pct = p->cfg.fwd_steps[++p->fwd_step];
    } else {
        if (p->rev_pct >= p->cfg.rev_max - 0.01f) r = PLIM_ERR_AT_MAX;
        else {
            p->rev_pct += p->cfg.rev_step;
            if (p->rev_pct > p->cfg.rev_max) p->rev_pct = p->cfg.rev_max;
        }
    }
    if (r == PLIM_OK) p->changes++;
    p->low_timer_ms = 0u;
    log_add(p, now_ms, of, orv, PLIM_SRC_CONFIRM, dir, r, true);
    return r;
}

void plim_log_attempt(plim_t *p, uint8_t dir, plim_result_t r, uint32_t now_ms)
{
    log_add(p, now_ms, p->fwd_pct, p->rev_pct, PLIM_SRC_CONFIRM, dir, r, true);
}

bool plim_raised(const plim_t *p)
{
    return p->fwd_step > 0u || p->rev_pct > p->cfg.rev_start + 0.01f;
}

void plim_drop(plim_t *p, plim_source_t src, uint32_t now_ms)
{
    p->code = 0u;                                   /* a half-done raise is cancelled too */
    if (!plim_raised(p)) return;
    float of = p->fwd_pct, orv = p->rev_pct;
    p->fwd_step = 0u;
    p->fwd_pct = p->cfg.fwd_steps[0];
    p->rev_pct = p->cfg.rev_start;
    p->low_timer_ms = 0u;
    p->changes++;
    log_add(p, now_ms, of, orv, src, 0u, PLIM_OK, false);
}

void plim_step(plim_t *p, float speed_kn, uint32_t dt_ms, uint32_t now_ms)
{
    if (p->code != 0u && (int32_t)(p->code_until_ms - now_ms) <= 0) p->code = 0u;   /* expired */
    if (!plim_raised(p) || speed_kn >= p->cfg.low_speed_kn) {
        p->low_timer_ms = 0u;
        return;
    }
    p->low_timer_ms += dt_ms;
    if (p->low_timer_ms >= p->cfg.low_speed_ms) plim_drop(p, PLIM_SRC_LOW_SPEED, now_ms);
}

float plim_code_left_s(const plim_t *p, uint32_t now_ms)
{
    if (p->code == 0u) return 0.0f;
    int32_t left = (int32_t)(p->code_until_ms - now_ms);
    return left > 0 ? (float)left * 0.001f : 0.0f;
}

uint32_t plim_log_size(const plim_t *p)
{
    return (p->log_count < PLIM_LOG_N) ? p->log_count : PLIM_LOG_N;
}

const plim_log_t *plim_log_get(const plim_t *p, uint32_t i)
{
    uint32_t n = plim_log_size(p);
    if (i >= n) return 0;
    return &p->log[(p->log_count - n + i) % PLIM_LOG_N];
}

const char *plim_source_name(plim_source_t s)
{
    static const char *const n[PLIM_SRC_COUNT] = {
        "startup", "PC", "CONFIRM", "joystick override", "STANDBY button", "alarm", "PC link lost",
        "GPS lost", "compass fault", "joystick fault", "TRAINING", "low speed", "fault"
    };
    return ((unsigned)s < PLIM_SRC_COUNT) ? n[s] : "?";
}

const char *plim_result_name(plim_result_t r)
{
    static const char *const n[] = { "ok", "wrong code", "already at max", "joystick not ready",
                                     "alarm active", "CONFIRM too late", "stick not centred" };
    return ((unsigned)r <= (unsigned)PLIM_ERR_NOT_CENTRED) ? n[r] : "?";
}
