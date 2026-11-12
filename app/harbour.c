/*
 * harbour.c - Harbour autonomy: the validation of the planner's proposals (see harbour.h). Pure C99, no HAL.
 */
#include "harbour.h"

#include <math.h>

#define HARB_STOP_KN 0.05f      /* = SB_SHDG_STOP_KN: a cap below this = STOP */

/* HARB_MUTATE (storm mutation runs ONLY, tests/acceptance/storm/mutate_harbour.sh; never defined in any firmware or
 * sim build - tools/check.sh greps that no build line sets it): 1 the cap removed, 2 the corridor HOLD removed,
 * 3 the lease HOLD removed, 4 the turn-rate slew removed (smallboat.c: 5 the lost-link grace removed) */
#ifndef HARB_MUTATE
#define HARB_MUTATE 0
#endif

void harb_default_config(harb_cfg_t *c)
{
    c->max_kn = 5.0f;           /* W144 */
    c->corr_m = 10.0f;          /* W146 */
    c->corr_margin_m = 2.0f;
    c->corr_stop_m = 5.0f;
    c->max_dev_deg = 60.0f;
    c->rot_dps = 8.0f;
    c->lease_ms = 1000.0f;
    c->unc_m = 5.0f;
    c->back_kn = 2.0f;
    c->ll_grace_s = 0.0f;       /* W147: disabled until the owner signs a value */
    c->ll_drift_kn = 1.0f;
}

float harb_wrap180(float a)
{
    if (!isfinite(a)) return 0.0f;
    a = fmodf(a, 360.0f);
    if (a > 180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

static float wrap360(float a)
{
    if (!isfinite(a)) return 0.0f;
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

float harb_slew(float cur, float want, float max_step)
{
    float d = harb_wrap180(want - cur);
    if (!(max_step > 0.0f)) max_step = 0.0f;
    if (HARB_MUTATE == 4) max_step = 360.0f;
    if (d > max_step) d = max_step;
    if (d < -max_step) d = -max_step;
    return wrap360(cur + d);
}

uint32_t harb_eff_lease_ms(const harb_cfg_t *c, uint16_t frame_lease_ms)
{
    float own = c->lease_ms;
    if (!(own >= (float)HARB_LEASE_FRAME_MIN)) own = (float)HARB_LEASE_FRAME_MIN;
    if (own > (float)HARB_LEASE_FRAME_MAX) own = (float)HARB_LEASE_FRAME_MAX;
    uint32_t o = (uint32_t)own;
    return (frame_lease_ms < o) ? frame_lease_ms : o;
}

bool harb_clamp_request(const harb_cfg_t *c, float corr_m, float kn, float *corr_out, float *kn_out)
{
    if (!isfinite(corr_m) || !isfinite(kn) || corr_m < HARB_CORR_MIN_M - 1e-4f || !(kn > 0.0f)) return false;
    float cmax = c->corr_m;
    if (!(cmax >= HARB_CORR_MIN_M)) cmax = HARB_CORR_MIN_M;
    *corr_out = corr_m > cmax ? cmax : corr_m;
    float k = kn;
    if (k > c->max_kn) k = c->max_kn;
    if (k > HARB_MAX_KN_CEIL) k = HARB_MAX_KN_CEIL;
    *kn_out = k;
    return true;
}

bool harb_grows_xte(float track_deg, float hdg_deg, float xte_m)
{
    float d = harb_wrap180(hdg_deg - track_deg);    /* + = to the right of the track = toward + xte */
    if (fabsf(d) < 0.5f) return false;              /* (parallel to the line) */
    return (d > 0.0f && xte_m > 0.0f) || (d < 0.0f && xte_m < 0.0f);
}

bool harb_why_planned(uint8_t why)
{
    return why == HARB_WHY_USER || why == HARB_WHY_ARRIVED;
}

void harb_judge(const harb_cfg_t *c, harb_t *h, const harb_in_t *in, harb_out_t *o)
{
    o->hold_why = HARB_WHY_NONE;
    o->want_deg = in->own_deg;
    o->back = false;
    o->prop_used = false;
    o->refused = false;
    float kmax = h->max_kn;
    if (kmax > c->max_kn) kmax = c->max_kn;
    if (kmax > HARB_MAX_KN_CEIL) kmax = HARB_MAX_KN_CEIL;
    float kn = in->leg_kn;
    if (!(kn >= 0.0f)) kn = 0.0f;
    if (kn > kmax) kn = kmax;
    o->kn = kn;
    /* 7. integrity first (no dead reckoning in a harbour) */
    if (!in->gps_ok || !(in->unc_m <= c->unc_m + 1e-3f)) { o->hold_why = HARB_WHY_GPS; o->kn = 0.0f; return; }
    if (!in->compass_ok) { o->hold_why = HARB_WHY_COMPASS; o->kn = 0.0f; return; }
    /* 1. the lease (the gateway can only shorten it) */
    if (!in->lease_ok && HARB_MUTATE != 3) { o->hold_why = HARB_WHY_LEASE; o->kn = 0.0f; return; }
    /* 2. the planner's STOP */
    if (in->stop_flag || (in->cap_kn >= 0.0f && in->cap_kn < HARB_STOP_KN)) {
        o->hold_why = HARB_WHY_PLANNER;
        o->kn = 0.0f;
        return;
    }
    /* 5. the corridor (the AURIX's own backstop) */
    float ax = fabsf(in->xte_m);
    if ((!isfinite(in->xte_m) || ax > h->corr_m + c->corr_stop_m) && HARB_MUTATE != 2) {
        o->hold_why = HARB_WHY_CORRIDOR;
        o->kn = 0.0f;
        return;
    }
    /* 3. the speed: only ever lowered */
    if (in->cap_kn >= 0.0f && in->cap_kn < kn) kn = in->cap_kn;
    if (HARB_MUTATE == 1) kn = in->leg_kn;
    if (ax > h->corr_m) {
        /* back to the line: the own guidance, slowly; the proposal is not used (not a refusal) */
        if (kn > c->back_kn) kn = c->back_kn;
        o->kn = kn;
        o->back = true;
        h->refuse_ms = 0u;
        return;
    }
    o->kn = kn;
    /* 4. the heading proposal */
    bool dev_ok = isfinite(in->prop_deg) && fabsf(harb_wrap180(in->prop_deg - in->track_deg)) <= c->max_dev_deg;
    bool edge_ok = ax <= h->corr_m - c->corr_margin_m || !harb_grows_xte(in->track_deg, in->prop_deg, in->xte_m);
    if (dev_ok && edge_ok) {
        o->want_deg = in->prop_deg;
        o->prop_used = true;
        h->refuse_ms = 0u;
        return;
    }
    o->refused = true;
    if (h->refused_seq != in->seq || h->refuse_ms == 0u) {
        h->refused = (uint8_t)(h->refused + 1u);
        h->refusals++;
    }
    h->refused_seq = in->seq;
    if (h->refuse_ms == 0u) h->refuse_ms = in->now_ms ? in->now_ms : 1u;
    else if (in->now_ms - h->refuse_ms >= HARB_REFUSE_HOLD_MS) {
        o->hold_why = HARB_WHY_HEADING;
        o->kn = 0.0f;
    }
}
