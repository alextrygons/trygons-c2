/*
 * generator.c - Generator mode state machine. See generator.h.
 */
#include "generator.h"

#include <math.h>
#include <string.h>

void gen_default_config(gen_config_t *c)
{
    memset(c, 0, sizeof *c);
    c->max_pct = 25.0f;             /* TO CONFIRM (boat file generator_max_pct; 0 = disabled) */
    c->start_v = 12.1f;
    c->rise_v = 0.4f;
    c->idle_pct = 8.0f;             /* TO CONFIRM */
    c->step_pct = 2.0f;
    c->step_ms = 30000u;
    c->start_delay_ms = 10000u;
    c->fall_v = 0.02f;
    c->run_ms = 20u * 60000u;
    c->rest_ms = 60000u;
    c->move_kn = 0.5f;
    c->move_ms = 20000u;
    c->thrust_pct = 10.0f;
    c->thrust_ms = 10000u;
}

void gen_init(gen_t *g, const gen_config_t *c, float learned_pct)
{
    memset(g, 0, sizeof *g);
    g->cfg = *c;
    if (isfinite(learned_pct) && learned_pct > 0.0f && learned_pct <= GEN_CEILING_PCT) g->learned_pct = learned_pct;
    g->state = GEN_OFF;
}

static float cap_now(const gen_t *g, float limit_pct)
{
    float cap = g->cfg.max_pct;
    if (!(cap >= 0.0f)) cap = 0.0f;                     /* NaN / negative = disabled */
    if (cap > GEN_CEILING_PCT) cap = GEN_CEILING_PCT;   /* whatever the boat file says */
    if (!(limit_pct >= 0.0f)) limit_pct = 0.0f;
    if (cap > limit_pct) cap = limit_pct;
    return cap;
}

/* leave RUNNING: demand 0 at once, STOPPING until the output is idle */
static void stop(gen_t *g, uint8_t why, bool fault)
{
    g->demand_pct = 0.0f;
    g->stop_why = why;
    g->fast_stop = g->fast_stop || fault;   /* (an emergency of this step stays fast: gen_emergency, 11g) */
    g->state = GEN_STOPPING;
    g->timer_ms = 0u;
    g->move_ms = 0u;
    g->stops++;
    if (fault) {
        g->faults++;
        g->fault_edge = true;
        g->enabled = false;             /* auto off until re-enabled */
    }
}

void gen_emergency(gen_t *g)
{
    if (g->active || g->state == GEN_RUNNING || g->state == GEN_STOPPING) g->fast_stop = true;
}

void gen_set(gen_t *g, bool on)
{
    if (on) {
        if (g->enabled) return;
        g->enabled = true;
        if (g->state == GEN_OFF || g->state == GEN_FAULT) g->state = GEN_READY;
    } else {
        g->enabled = false;
        if (g->state == GEN_RUNNING) stop(g, GEN_STOP_OFF, false);
        else if (g->state != GEN_STOPPING) { g->state = GEN_OFF; g->demand_pct = 0.0f; }
    }
}

static bool runnable(const gen_t *g, const gen_input_t *in)
{
    return g->enabled && cap_now(g, in->limit_pct) >= g->cfg.idle_pct - 0.001f && in->allowed &&
           in->neutral_confirmed && in->volts_valid && !in->stop_request;
}

void gen_step(gen_t *g, const gen_input_t *in)
{
    uint32_t dt = in->dt_ms;
    g->start_edge = false;
    g->fault_edge = false;
    g->done_edge = false;
    g->cap_pct = cap_now(g, in->limit_pct);

    switch (g->state) {
    case GEN_OFF:
        g->demand_pct = 0.0f;
        if (g->enabled) g->state = GEN_READY;
        break;

    case GEN_FAULT:
        g->demand_pct = 0.0f;
        if (g->enabled) g->state = GEN_READY;           /* re-enabled by the captain */
        break;

    case GEN_READY:
    case GEN_DONE:
        g->demand_pct = 0.0f;
        if (!g->enabled) { g->state = GEN_OFF; break; }
        if (runnable(g, in) && in->bank_v < g->cfg.start_v) {
            g->state = GEN_STARTING;
            g->timer_ms = 0u;
            g->start_edge = true;
        }
        break;

    case GEN_STARTING:
        g->demand_pct = 0.0f;
        if (!runnable(g, in)) { g->state = g->enabled ? GEN_READY : GEN_OFF; break; }
        g->timer_ms += dt;
        if (g->timer_ms >= g->cfg.start_delay_ms) {
            float first = (g->learned_pct > 0.0f) ? g->learned_pct : g->cfg.idle_pct;
            if (first < g->cfg.idle_pct) first = g->cfg.idle_pct;
            if (first > g->cap_pct) first = g->cap_pct;
            g->demand_pct = first;
            g->state = GEN_RUNNING;
            g->run_ms = 0u;
            g->judge_ms = 0u;
            g->v_ref = in->bank_v_fast;
            g->move_ms = 1u;                            /* the first rise: watch the SOG */
            g->sog_ref = in->gps_trusted ? in->sog_kn : -1.0f;
            g->thrust_ref = in->thrust_pct;
            g->thrust_ms = 0u;
            g->stopped_falling = false;
            g->starts++;
        }
        break;

    case GEN_RUNNING:
        /* 1. the conditions (every step) */
        if (!g->enabled) { stop(g, GEN_STOP_OFF, false); break; }
        if (!in->neutral_confirmed) { stop(g, GEN_STOP_SWITCH, true); break; }
        if (!in->volts_valid) { stop(g, GEN_STOP_NO_VOLTS, false); break; }
        if (!in->allowed || g->cap_pct < g->cfg.idle_pct - 0.001f) { stop(g, GEN_STOP_CONDITIONS, false); break; }
        if (in->stop_request) { stop(g, GEN_STOP_REQUEST, false); break; }
        /* 2a. DP: the stern motor fighting a forward push it can't explain = the gearbox may be in gear */
        if (in->dp_fight) { stop(g, GEN_STOP_DP_FIGHT, true); break; }
        /* 2. the boat must not move: a SOG rise within move_ms of a throttle rise = GEAR FAULT */
        if (g->move_ms) {
            g->move_ms += dt;
            if (in->gps_trusted) {
                if (g->sog_ref < 0.0f) g->sog_ref = in->sog_kn;
                else if (in->sog_kn - g->sog_ref > g->cfg.move_kn) { stop(g, GEN_STOP_MOVED, true); break; }
            }
            if (g->move_ms > g->cfg.move_ms) g->move_ms = 0u;
        }
        /* 3. the cap may have dropped (power limit lowered): follow it down at once, never up */
        if (g->demand_pct > g->cap_pct) g->demand_pct = g->cap_pct;
        /* 4. a thruster load change pauses the judgement (the voltage sag is not the engine's) */
        if (fabsf(in->thrust_pct - g->thrust_ref) > g->cfg.thrust_pct) {
            g->thrust_ms = g->cfg.thrust_ms;
            g->thrust_ref = in->thrust_pct;
        }
        if (g->thrust_ms) g->thrust_ms = (g->thrust_ms > dt) ? g->thrust_ms - dt : 0u;
        /* 5. the cycle */
        g->run_ms += dt;
        if (g->run_ms >= g->cfg.run_ms) { stop(g, GEN_STOP_REST, false); break; }
        /* 6. the judgement every step_ms */
        g->judge_ms += dt;
        if (g->judge_ms >= g->cfg.step_ms) {
            g->judge_ms = 0u;
            float v = in->bank_v_fast;
            if (g->thrust_ms) {
                g->holds++;                             /* paused: hold, never raise */
            } else if (v < g->v_ref - g->cfg.fall_v) {
                float next = g->demand_pct + g->cfg.step_pct;
                if (next > g->cap_pct) next = g->cap_pct;
                if (next > g->demand_pct + 0.001f) {
                    g->demand_pct = next;
                    g->steps_up++;
                    g->move_ms = 1u;                    /* watch the SOG after this rise */
                    g->sog_ref = in->gps_trusted ? in->sog_kn : -1.0f;
                } else {
                    g->holds++;                         /* at the cap */
                }
            } else if (v > g->cfg.start_v + g->cfg.rise_v) {
                float next = g->demand_pct - g->cfg.step_pct;
                if (next < g->cfg.idle_pct) next = g->cfg.idle_pct;
                if (next < g->demand_pct - 0.001f) { g->demand_pct = next; g->steps_down++; }
                else g->holds++;
            } else {
                g->holds++;                             /* stopped falling: this is the learned value */
                if (!g->stopped_falling || fabsf(g->learned_pct - g->demand_pct) > 0.001f) {
                    g->learned_pct = g->demand_pct;
                    g->learned_new = true;
                }
                g->stopped_falling = true;
            }
            g->v_ref = v;
        }
        break;

    case GEN_STOPPING:
        g->demand_pct = 0.0f;                           /* never raises while a stop is pending */
        if (in->throttle_out_pct <= in->idle_max_pct + 0.001f) {
            g->fast_stop = false;
            if (g->stop_why == GEN_STOP_SWITCH || g->stop_why == GEN_STOP_MOVED || g->stop_why == GEN_STOP_DP_FIGHT)
                g->state = GEN_FAULT;
            else if (!g->enabled) g->state = GEN_OFF;
            else if (g->stop_why == GEN_STOP_REST) { g->state = GEN_REST; g->timer_ms = 0u; }
            else g->state = GEN_READY;
        }
        break;

    case GEN_REST:
        g->demand_pct = 0.0f;
        if (!g->enabled) { g->state = GEN_OFF; break; }
        if (!in->allowed || !in->neutral_confirmed) { g->state = GEN_READY; break; }
        g->timer_ms += dt;
        if (g->timer_ms >= g->cfg.rest_ms) {
            g->cycles++;
            if (in->volts_valid && in->bank_v < g->cfg.start_v) {
                g->state = GEN_STARTING;                /* still low: again (beep + notice) */
                g->timer_ms = 0u;
                g->start_edge = true;
            } else {
                g->state = GEN_DONE;
                g->done_edge = true;
            }
        }
        break;

    default:
        g->state = GEN_OFF;
        g->demand_pct = 0.0f;
        break;
    }
    g->active = g->state == GEN_RUNNING || g->state == GEN_STOPPING;
    if (!g->active) { g->demand_pct = 0.0f; g->fast_stop = false; }
}

const char *gen_state_name(uint8_t state)
{
    static const char *const n[] = { "off", "ready", "starting", "running", "stopping", "rest", "fault", "done" };
    return (state <= GEN_DONE) ? n[state] : "?";
}
