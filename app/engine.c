/*
 * engine.c - Engine controls + remote restart. See engine.h.
 * The STARTER relay only through relays_starter_pulse / relays_starter_off,
 * IGNITION / SHUTOFF only through relays_engine_set (this file is their only
 * caller: tools/check.sh).
 */
#include "engine.h"

#include <string.h>

void engine_init(engine_t *e)
{
    memset(e, 0, sizeof *e);
}

eng_why_t engine_check(const eng_cond_t *c)
{
    if (c->first_test) return ENG_WHY_FIRST_TEST;
    if (c->estop_pressed) return ENG_WHY_ESTOP;
    if (!c->idle) return ENG_WHY_THROTTLE;
    if (!c->neutral) return ENG_WHY_GEAR;
    return ENG_WHY_NONE;
}

static void note(engine_t *e, uint8_t action, eng_why_t why)
{
    e->last_action = action;
    e->last_why = (uint8_t)why;
    e->log_seq++;
}

eng_why_t engine_restart(engine_t *e, const eng_cond_t *c, uint8_t station, uint32_t now_ms, bool *kill_clear)
{
    eng_why_t why = engine_check(c);
    *kill_clear = false;
    if (why == ENG_WHY_NONE && (station == 0u || station != c->helm)) why = ENG_WHY_NOT_HELM;
    if (why == ENG_WHY_NONE && !c->killed && !c->tripped) why = ENG_WHY_NOTHING;
    if (why != ENG_WHY_NONE) {
        e->refused++;
        note(e, ENG_LOG_REFUSED, why);
        return why;
    }
    e->restarts++;
    if (c->tripped) {
        /* the HAL latch goes only with a reset: the event first, then the reset */
        if (!e->reset_pending) {
            e->reset_pending = true;
            e->reset_req_ms = now_ms;
            e->reset_at_ms = now_ms + ENG_RESET_DELAY_MS;
            e->resets++;
        }
        note(e, ENG_LOG_RESET, ENG_WHY_NONE);
        return ENG_WHY_NONE;
    }
    *kill_clear = true;
    e->kill_clears++;
    note(e, ENG_LOG_KILL_CLEARED, ENG_WHY_NONE);
    return ENG_WHY_NONE;
}

static void crank_stop(engine_t *e, relays_t *r, uint32_t now_ms, eng_why_t why)
{
    relays_starter_off(r);
    e->hold_ms = now_ms - e->hold_t0_ms;
    e->crank = ENG_CRANK_COOL;
    e->cool_until_ms = (now_ms + ENG_COOL_MS) ? now_ms + ENG_COOL_MS : 1u;
    e->stop_why = (uint8_t)why;
    note(e, ENG_LOG_CRANK_STOP, why);
}

/* the crank conditions on top of the restart ones */
static eng_why_t crank_check(const engine_t *e, const eng_cond_t *c, const relays_t *r, uint8_t station)
{
    eng_why_t why = engine_check(c);
    if (why != ENG_WHY_NONE) return why;
    if (c->gear_request) return ENG_WHY_GEAR;
    if (!c->helm_link) return ENG_WHY_LINK;             /* (HELM gone to nobody: the link is lost too) */
    if (station == 0u || station != c->helm) return ENG_WHY_NOT_HELM;
    if (c->killed) return ENG_WHY_KILLED;
    if (c->tripped) return ENG_WHY_TRIPPED;
    if (relays_starter(r) == 0u) return ENG_WHY_NO_STARTER;
    if (e->crank == ENG_CRANK_ON && station != e->crank_station) return ENG_WHY_NOT_HELM;
    return ENG_WHY_NONE;
}

eng_why_t engine_crank(engine_t *e, const eng_cond_t *c, relays_t *r, uint8_t station, uint32_t now_ms)
{
    eng_why_t why = crank_check(e, c, r, station);
    if (why == ENG_WHY_NONE && e->crank == ENG_CRANK_COOL) why = ENG_WHY_COOLING;
    if (why != ENG_WHY_NONE) {
        if (e->crank == ENG_CRANK_ON) crank_stop(e, r, now_ms, why);
        else { e->refused++; note(e, ENG_LOG_CRANK_REFUSED, why); }
        return why;
    }
    uint32_t held = (e->crank == ENG_CRANK_ON) ? now_ms - e->hold_t0_ms : 0u;
    if (held >= ENG_CRANK_MAX_MS) {
        crank_stop(e, r, now_ms, ENG_WHY_MAX_TIME);
        return ENG_WHY_MAX_TIME;
    }
    uint32_t len = ENG_CRANK_PULSE_MS;
    if (len > ENG_CRANK_MAX_MS - held) len = ENG_CRANK_MAX_MS - held;
    if (len < 100u) len = 100u;                         /* the board's pulse unit is 0.1 s */
    if (relays_starter_pulse(r, (uint16_t)len, now_ms) != RELAY_R_OK) {
        if (e->crank == ENG_CRANK_ON) crank_stop(e, r, now_ms, ENG_WHY_RELAY);
        else { e->refused++; note(e, ENG_LOG_CRANK_REFUSED, ENG_WHY_RELAY); }
        return ENG_WHY_RELAY;
    }
    e->pulses++;
    e->last_req_ms = now_ms;
    if (e->crank != ENG_CRANK_ON) {
        e->crank = ENG_CRANK_ON;
        e->crank_station = station;
        e->hold_t0_ms = now_ms;
        e->hold_ms = 0u;
        e->cranks++;
        note(e, ENG_LOG_CRANK_START, ENG_WHY_NONE);
    }
    return ENG_WHY_NONE;
}

eng_why_t engine_relay(engine_t *e, const eng_cond_t *c, relays_t *r, uint8_t n, bool on, relay_result_t *res)
{
    (void)e;
    if (on) {
        eng_why_t why = engine_check(c);
        if (why == ENG_WHY_NONE && c->killed) why = ENG_WHY_KILLED;
        if (why == ENG_WHY_NONE && c->tripped) why = ENG_WHY_TRIPPED;
        if (why != ENG_WHY_NONE) {
            *res = relays_note(r, n, RELAY_CMD_ON, why == ENG_WHY_FIRST_TEST ? RELAY_R_FIRST_TEST : RELAY_R_ENGINE);
            return why;
        }
    }
    *res = relays_engine_set(r, n, on);
    return *res == RELAY_R_OK ? ENG_WHY_NONE : ENG_WHY_RELAY;
}

void engine_release(engine_t *e, relays_t *r, uint8_t station, uint32_t now_ms)
{
    if (e->crank == ENG_CRANK_ON && station == e->crank_station) crank_stop(e, r, now_ms, ENG_WHY_RELEASED);
}

bool engine_step(engine_t *e, const eng_cond_t *c, relays_t *r, uint32_t now_ms)
{
    if (e->crank == ENG_CRANK_ON) {
        eng_why_t why = crank_check(e, c, r, e->crank_station);
        if (why == ENG_WHY_NONE && c->standby_button) why = ENG_WHY_STANDBY_BTN;
        if (why == ENG_WHY_NONE && now_ms - e->last_req_ms >= ENG_RELEASE_MS) why = ENG_WHY_RELEASED;
        if (why == ENG_WHY_NONE && now_ms - e->hold_t0_ms >= ENG_CRANK_MAX_MS) why = ENG_WHY_MAX_TIME;
        if (why != ENG_WHY_NONE) crank_stop(e, r, now_ms, why);
        else e->hold_ms = now_ms - e->hold_t0_ms;
    } else if (e->crank == ENG_CRANK_COOL && (int32_t)(now_ms - e->cool_until_ms) >= 0) {
        e->crank = ENG_CRANK_OFF;
    }
    if (!e->reset_pending) return false;
    /* the conditions hold until the reset itself (checked every step) */
    eng_why_t why = engine_check(c);
    if (why != ENG_WHY_NONE) {
        e->reset_pending = false;
        note(e, ENG_LOG_RESET_CANCEL, why);
        return false;
    }
    if ((int32_t)(now_ms - e->reset_at_ms) < 0) return false;
    if (c->nv_busy && now_ms - e->reset_req_ms < ENG_RESET_NV_MAX_MS) return false;
    return true;
}

float engine_cool_left_s(const engine_t *e, uint32_t now_ms)
{
    if (e->crank != ENG_CRANK_COOL) return 0.0f;
    int32_t left = (int32_t)(e->cool_until_ms - now_ms);
    return left > 0 ? (float)left * 0.001f : 0.0f;
}
