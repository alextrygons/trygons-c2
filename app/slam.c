/*
 * slam.c - Slamming measurement and throttle reduction. See slam.h.
 */
#include "slam.h"

#include <math.h>
#include <string.h>

void slam_default_config(slam_config_t *c)
{
    c->limit_g = 3.0f;
    c->red_reset = 0.0f;                /* STEP */
    c->yellow_frac = 0.7f;
    c->impact_g = 1.0f;
    c->refractory_ms = 150u;
    c->level_alpha = 0.25f;
    c->level_tau_s = 30.0f;
    c->step_pct = 10.0f;
    c->step_ms = 5000u;
    c->min_cap_pct = 30.0f;
    c->calm_ms = 30000u;
    c->ask_ms = 10000u;
    c->ask_again_ms = 120000u;
    c->up_pct_s = 15.0f;                /* smallboat.c sets the throttle's own up rate */
    c->override_ms = 1000u;
    c->lost_ms = 1000u;
}

void slam_init(slam_t *s, const slam_config_t *c)
{
    memset(s, 0, sizeof *s);
    s->cfg = *c;
    s->cap_pct = 100.0f;
}

static void impact(slam_t *s, float peak, uint32_t now_ms)
{
    s->impacts++;
    s->level += s->cfg.level_alpha * (peak - s->level);
    s->ring[s->ring_i] = now_ms;
    s->ring_i = (uint8_t)((s->ring_i + 1u) % SLAM_RING);
    if (s->ring_n < SLAM_RING) s->ring_n++;
    if (peak >= s->cfg.limit_g) {
        s->reds++;
        s->red_edge = true;
        s->red_until = now_ms + s->cfg.calm_ms;
    }
}

/* One step down from what is really used now (a cap above the demand would
 * change nothing). */
static void step_down(slam_t *s, float demand_pct)
{
    float from = fminf(s->cap_pct, fmaxf(demand_pct, s->cfg.min_cap_pct));
    if (s->cap_pct >= 100.0f) s->before_pct = fminf(100.0f, fmaxf(demand_pct, s->cfg.min_cap_pct));
    s->cap_pct = fmaxf(s->cfg.min_cap_pct, from - s->cfg.step_pct);
    s->steps++;
    s->kick_armed = false;              /* an override needs a new kick-down */
    /* a new reduction: a running restore stops, an open question closes, the first question again after calm_ms */
    s->restoring = false;
    s->asking = false;
    s->declined = false;
    s->ask_at = s->cfg.calm_ms;
}

bool slam_restore_avail(const slam_t *s)
{
    return s->cap_pct < 100.0f && !s->restoring && !s->lost && s->seen && s->state == SLAM_CALM &&
           s->calm_timer >= s->cfg.calm_ms;
}

bool slam_restore(slam_t *s, bool accept)
{
    if (!accept) {
        if (!s->asking) return s->cap_pct < 100.0f;  /* nothing open: harmless */
        s->asking = false;
        s->declined = true;
        s->rejects++;
        s->ask_at = s->calm_timer + s->cfg.ask_again_ms;
        return true;
    }
    if (!slam_restore_avail(s)) return false;
    s->asking = false;
    s->restoring = true;
    s->accepts++;
    return true;
}

void slam_step(slam_t *s, const float *samples, uint32_t n, uint32_t now_ms, uint32_t dt_ms,
               float demand_pct, bool onboard)
{
    const slam_config_t *c = &s->cfg;
    uint32_t imp0 = s->impacts;
    s->red_edge = s->yellow_edge = false;
    s->peak = 0.0f;

    /* samples -> impacts (one per pulse) */
    for (uint32_t i = 0; i < n; i++) {
        float a = fabsf(samples[i]);
        if (a > s->peak) s->peak = a;
        if (s->in_pulse) {
            if (a > s->pulse_max) s->pulse_max = a;
            if (a < 0.5f * c->impact_g) {
                s->in_pulse = false;
                s->refractory_until = now_ms + c->refractory_ms;
                impact(s, s->pulse_max, now_ms);
            }
        } else if (a >= c->impact_g && (int32_t)(now_ms - s->refractory_until) >= 0) {
            s->in_pulse = true;
            s->pulse_max = a;
        }
    }
    if (s->peak > s->peak_hold) s->peak_hold = s->peak;
    if (n > 0u) {
        s->seen = true;
        s->lost = false;
        s->last_sample_ms = now_ms;
    } else if (s->seen && now_ms - s->last_sample_ms > c->lost_ms) {
        s->lost = true;
        s->in_pulse = false;
    }

    /* level decays when calm; rate = impacts in the last 60 s */
    s->level *= expf(-(float)dt_ms * 0.001f / c->level_tau_s);
    uint16_t r = 0u;
    for (uint8_t i = 0; i < s->ring_n; i++)
        if (now_ms - s->ring[i] < 60000u) r++;
    s->rate_pm = r;

    /* state */
    uint8_t was = s->state;
    bool yellow = s->level >= c->yellow_frac * c->limit_g;
    if ((int32_t)(s->red_until - now_ms) > 0 && s->reds > 0u) s->state = SLAM_RED;
    else s->state = yellow ? SLAM_YELLOW : SLAM_CALM;
    if (s->state != SLAM_CALM && was == SLAM_CALM) s->yellow_edge = true;

    /* an ACCEPTED restore that meets a new impact (or the sensor stops): it stops at once, an impact
     * also steps the cap down again (step 13q4) */
    bool hit = s->impacts != imp0;
    if (s->restoring && (hit || s->lost || s->state != SLAM_CALM)) {
        s->restoring = false;
        s->restore_stops++;
        if (hit && !s->red_edge) step_down(s, demand_pct);
    }

    /* reduction (only ever down here) */
    if (s->red_edge) {
        step_down(s, demand_pct);
        s->step_timer = 0u;
    } else if (s->state != SLAM_CALM && yellow) {
        s->step_timer += dt_ms;
        if (s->step_timer >= c->step_ms || was == SLAM_CALM) {
            s->step_timer = 0u;
            step_down(s, demand_pct);
        }
    } else {
        s->step_timer = 0u;
    }

    /* calm water: NO recovery by itself (step 13q4, owner 2026-10-31) - the question is offered; only
     * slam_restore(accept) raises the cap, and then only at the normal throttle up rate */
    bool calm = s->state == SLAM_CALM && s->level < 0.8f * c->yellow_frac * c->limit_g;
    if (calm) {
        s->calm_timer += dt_ms;
    } else {
        s->calm_timer = 0u;
        if (s->asking) s->asking = false;           /* rough again: the question closes, not an answer */
        if (s->declined) s->ask_at = c->ask_again_ms;   /* declined once: the next question after 2 min calm */
        else s->ask_at = c->calm_ms;
    }
    if (s->cap_pct >= 100.0f) {
        s->asking = s->declined = false;
        s->ask_at = c->calm_ms;
    }
    if (s->asking && (!slam_restore_avail(s) || now_ms - s->ask_t >= c->ask_ms)) {
        bool timed_out = slam_restore_avail(s);
        s->asking = false;
        if (timed_out) {                           /* no answer in 10 s = REJECT */
            s->declined = true;
            s->timeouts++;
            s->ask_at = s->calm_timer + c->ask_again_ms;
        }
    }
    if (!s->asking && slam_restore_avail(s) && s->calm_timer >= s->ask_at) {
        s->asking = true;
        s->ask_t = now_ms;
        s->asks++;
    }
    if (s->restoring) {
        s->cap_pct += c->up_pct_s * (float)dt_ms * 0.001f;
        if (s->cap_pct >= 100.0f) {
            s->cap_pct = 100.0f;
            s->restoring = false;
            s->before_pct = 0.0f;
        }
    }

    /* override by kick-down: the onboard driver insists (never the remote driver) */
    if (!onboard || s->cap_pct >= 100.0f) {
        s->override = false;
        s->kick_armed = false;
        s->over_ms = 0u;
    } else if (s->override) {
        if (demand_pct < s->cap_pct) s->override = false;          /* eased below the cap: cap again */
    } else {
        if (demand_pct < 85.0f) s->kick_armed = true;
        if (s->kick_armed && demand_pct >= 95.0f) {
            s->over_ms += dt_ms;
            if (s->over_ms >= c->override_ms) {
                s->override = true;
                s->overrides++;
                s->over_ms = 0u;
            }
        } else {
            s->over_ms = 0u;
        }
    }
}

float slam_cap(const slam_t *s)
{
    return s->override ? 100.0f : s->cap_pct;
}
