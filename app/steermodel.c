/*
 * steermodel.c - Steering model of the DroneCAN position servo. See steermodel.h.
 */
#include "steermodel.h"

#include <math.h>
#include <string.h>

#define SM_KNOWN_TOL  0.002f            /* the position interval this narrow = known */

static float clampf(float x, float lo, float hi)
{
    if (!(x >= lo)) return lo;
    if (x > hi) return hi;
    return x;
}

void sm_defaults(sm_params_t *p)
{
    memset(p, 0, sizeof *p);
    p->r_right = p->r_left = SM_RATE_WORST;     /* never faster than the worst-case servo */
    p->b_lr = p->b_rl = 0.0f;                   /* no compensation = never an overshoot */
    p->k = SM_K_DEFAULT;
    p->tau = SM_TAU_DEFAULT;
    p->a0 = 0.0f;
    p->v_ref = 0.0f;
    p->res = 0.0f;
    p->trained = false;
}

static void limits(steer_t *s, float half_deg, float max_deg)
{
    s->half_deg = (half_deg >= 5.0f && half_deg <= 90.0f) ? half_deg : SM_HALF_DEG;
    s->max_u = clampf(max_deg / s->half_deg, 0.05f, 1.0f);
}

void sm_init(steer_t *s, const sm_params_t *stored, float half_deg, float max_deg, bool known, float p0)
{
    memset(s, 0, sizeof *s);
    s->stored = *stored;
    s->use = *stored;
    limits(s, half_deg, max_deg);
    if (known && isfinite(p0)) {
        p0 = clampf(p0, -1.0f, 1.0f);
        s->p_lo = s->p_hi = p0;                 /* a warm reset: the node held the last command */
        s->u = p0;
        s->a = p0;
    } else {
        s->p_lo = -1.0f;                        /* a power-on: anywhere (worst-case travel) */
        s->p_hi = 1.0f;
        s->u = 0.0f;
        s->a = 0.0f;
    }
    s->have = false;
}

void sm_set_stored(steer_t *s, const sm_params_t *stored, float half_deg, float max_deg)
{
    s->stored = *stored;
    s->use = *stored;
    s->adapted = false;
    s->ad.primed = false;
    limits(s, half_deg, max_deg);
}

bool sm_known(const steer_t *s)
{
    return s->p_hi - s->p_lo < SM_KNOWN_TOL;
}

float sm_play(const steer_t *s)
{
    float b = 0.5f * (s->use.b_lr + s->use.b_rl), bs = 0.5f * (s->stored.b_lr + s->stored.b_rl);
    if (b > bs) b = bs;                         /* never more than TRAINING's play */
    return clampf(b, 0.0f, 0.5f);
}

float sm_angle_deg(const steer_t *s)
{
    return s->a * s->half_deg;
}

/* the bound moves toward the command at the servo speed */
static void bound_move(float *b, float u, float r_up, float r_dn, float dt)
{
    if (*b < u) { *b += r_up * dt; if (*b > u) *b = u; }
    else if (*b > u) { *b -= r_dn * dt; if (*b < u) *b = u; }
}

static void play(float *a, float p, float h)
{
    if (p - *a > h) *a = p - h;
    else if (*a - p > h) *a = p + h;
}

static void track(steer_t *s, float dt)
{
    if (s->have) {
        /* unknown position: the worst-case speed (the slowest servo) closes the interval; known: the servo
         * speed in use (the command never moves faster, so p stays on the command) */
        bool known = sm_known(s);
        float ru = known ? s->use.r_right : SM_RATE_WORST, rd = known ? s->use.r_left : SM_RATE_WORST;
        bound_move(&s->p_lo, s->u, ru, rd, dt);
        bound_move(&s->p_hi, s->u, ru, rd, dt);
    }
    play(&s->a, 0.5f * (s->p_lo + s->p_hi), 0.5f * sm_play(s));
}

float sm_drive(steer_t *s, float target, float db, float dt)
{
    if (!isfinite(target)) target = s->a;
    target = clampf(target, -s->max_u, s->max_u);
    s->target = target;
    /* the position unknown (a power-on: the worst-case travel closes the interval): the direction from the
     * command (the estimate is only the middle of the interval), no play compensation (where the play sits is
     * unknown too) and the worst-case speed */
    bool known = sm_known(s);
    float err = target - ((known || !s->have) ? s->a : s->u);
    int8_t nd = s->dir;
    if (nd >= 0 && err < -db) nd = -1;
    else if (nd <= 0 && err > db) nd = 1;
    if (nd != s->dir && s->dir != 0) s->reversals++;
    s->dir = nd;
    float h = known ? 0.5f * sm_play(s) : 0.0f;
    /* pushing right the outboard sits h left of the servo (a = p - h): p = target + h; never back off
     * inside the play while the direction stays (no dithering) */
    float want;
    if (nd > 0) {
        want = target + h;
        if (s->have && want < s->u) want = s->u;
    } else if (nd < 0) {
        want = target - h;
        if (s->have && want > s->u) want = s->u;
    } else {
        want = s->have ? s->u : target;         /* no direction yet (inside the deadband) */
    }
    float r = !known ? SM_RATE_WORST : (want > s->u) ? s->use.r_right : s->use.r_left;
    float step = r * dt;
    s->u = clampf(s->u + clampf(want - s->u, -step, step), -1.0f, 1.0f);
    s->have = true;
    track(s, dt);
    return s->u;
}

void sm_direct(steer_t *s, float u, float dt)
{
    if (!isfinite(u)) u = s->u;
    u = clampf(u, -1.0f, 1.0f);
    if (u > s->u + 1e-6f) { if (s->dir < 0) s->reversals++; s->dir = 1; }
    else if (u < s->u - 1e-6f) { if (s->dir > 0) s->reversals++; s->dir = -1; }
    s->u = u;
    s->have = true;
    track(s, dt);
}

void sm_hold(steer_t *s, float dt)
{
    track(s, dt);
}

bool sm_at(const steer_t *s, float target, float tol)
{
    return sm_known(s) && fabsf(0.5f * (s->p_lo + s->p_hi) - s->u) < 1e-3f && fabsf(s->a - target) <= tol;
}

/* --- slow adaptation (RAM only) ----------------------------------------------------- */

void sm_adapt_reset(steer_t *s, uint8_t why)
{
    s->use = s->stored;
    s->adapted = false;
    s->ad.primed = false;
    s->ad.doubt_ms = 0u;
    s->ad.better = 0u;
    s->reset_why = why;
    s->resets++;
}

bool sm_within_bounds(const steer_t *s)
{
    const sm_params_t *u = &s->use, *t = &s->stored;
    const float e = 1e-4f;
    if (!(u->r_right <= t->r_right + e && u->r_left <= t->r_left + e)) return false;    /* never faster */
    if (!(u->b_lr <= t->b_lr + e && u->b_rl <= t->b_rl + e)) return false;              /* b only shrinks ... */
    if (!(u->b_lr >= SM_ADAPT_B_LO * t->b_lr - e && u->b_rl >= SM_ADAPT_B_LO * t->b_rl - e)) return false;
    if (!t->trained) return u->k == t->k && u->tau == t->tau;                         /* no TRAINING: no adaptation */
    return u->k >= SM_ADAPT_K_LO * t->k - e && u->k <= SM_ADAPT_K_HI * t->k + e &&
           u->tau >= SM_ADAPT_K_LO * t->tau - e && u->tau <= SM_ADAPT_K_HI * t->tau + e;
}

uint8_t sm_adapt_step(steer_t *s, const sm_adapt_in_t *in)
{
    sm_adapt_t *ad = &s->ad;
    ad->active = false;
    if (!s->stored.trained) return SM_RESET_NONE;      /* defaults: nothing to adapt */
    if (in->node_lost || in->sensor_fault) {
        uint8_t why = in->node_lost ? SM_RESET_NODE : SM_RESET_SENSOR;
        if (s->adapted) { sm_adapt_reset(s, why); return why; }
        ad->primed = false;
        return SM_RESET_NONE;
    }
    float dt = in->dt;
    float y = in->yaw_dps;
    if (!in->steady || !sm_known(s) || !(dt > 0.0f) || !isfinite(y) || !(in->v_kn > 0.5f) ||
        !(s->stored.v_ref > 0.5f)) {
        ad->primed = false;                    /* paused: the values stay, the predictor restarts */
        ad->doubt_ms = 0u;
        return SM_RESET_NONE;
    }
    float tau = s->use.tau, k = s->use.k;
    float vr = in->v_kn / s->stored.v_ref;
    if (!ad->primed) {
        memset(ad, 0, sizeof *ad);
        ad->w = ad->w2 = y;
        ad->a2 = s->a;
        ad->win_rev = s->reversals;
        float r0 = fmaxf(s->stored.res, SM_RES_FLOOR_DPS);
        ad->res2 = r0 * r0;
        ad->primed = true;
        return SM_RESET_NONE;
    }
    ad->active = true;
    ad->steps++;
    /* the model forward (open loop + a slow offset for wind / current / trim) */
    float x = (s->a - s->use.a0) * vr;
    float drive = k * x + ad->c - ad->w;
    float st_new = ad->st + dt * (-drive / (tau * tau) - ad->st / tau);
    ad->sk += dt * (x - ad->sk) / tau;
    ad->st = st_new;
    ad->w += dt * drive / tau;
    /* the parallel model with a 15 % smaller play (b may only shrink) */
    float b = sm_play(s), bs = 0.5f * (s->stored.b_lr + s->stored.b_rl);
    float b2 = fmaxf(SM_ADAPT_B_LO * bs, 0.85f * b);
    play(&ad->a2, 0.5f * (s->p_lo + s->p_hi), 0.5f * b2);
    float x2 = (ad->a2 - s->use.a0) * vr;
    ad->w2 += dt * (k * x2 + ad->c - ad->w2) / tau;
    float e = y - ad->w, e2 = y - ad->w2;
    ad->c += e * dt / 30.0f;
    /* K and tau: the least-squares direction over ~30 s (the residual against the sensitivities, their slow mean
     * removed: the offset c takes the steady part), applied with the long memory SM_ADAPT_MEMORY_S; nothing moves
     * without enough excitation */
    ad->skm += (ad->sk - ad->skm) * dt / 60.0f;
    ad->stm += (ad->st - ad->stm) * dt / 60.0f;
    float skh = ad->sk - ad->skm, sth = ad->st - ad->stm;
    ad->pkk += (skh * skh - ad->pkk) * dt / 30.0f;
    ad->pek += (e * skh - ad->pek) * dt / 30.0f;
    ad->ptt += (sth * sth - ad->ptt) * dt / 30.0f;
    ad->pet += (e * sth - ad->pet) * dt / 30.0f;
    /* only with excitation FROM THE STEERING (the caller adapts in calm water only: in waves the controller fighting
     * the sea would bias a closed-loop estimate) */
    float mu = dt / SM_ADAPT_MEMORY_S;
    bool excited = ad->pkk > 1e-6f;
    float nk = k + (excited ? mu * ad->pek / ad->pkk : 0.0f);
    float nt = tau + ((excited && ad->ptt > 1e-6f) ? mu * ad->pet / ad->ptt : 0.0f);
    float klo = SM_ADAPT_K_LO * s->stored.k, khi = SM_ADAPT_K_HI * s->stored.k;
    float tlo = SM_ADAPT_K_LO * s->stored.tau, thi = SM_ADAPT_K_HI * s->stored.tau;
    if (!isfinite(nk) || !isfinite(nt) || nk <= klo || nk >= khi || nt <= tlo || nt >= thi) {
        sm_adapt_reset(s, SM_RESET_LIMIT);     /* a bound reached: doubt - the stored values */
        return SM_RESET_LIMIT;
    }
    if (fabsf(nk - s->stored.k) > 1e-6f || fabsf(nt - s->stored.tau) > 1e-6f) s->adapted = true;
    s->use.k = nk;
    s->use.tau = nt;
    /* the play: windows of 60 s with >= 2 reversals; 3 in a row clearly better with the smaller play */
    ad->j += e * e * dt;
    ad->j2 += e2 * e2 * dt;
    ad->win_ms += (uint32_t)(dt * 1000.0f + 0.5f);
    if (ad->win_ms >= 60000u) {
        bool enough = (uint16_t)(s->reversals - ad->win_rev) >= 2u;
        if (enough && b2 < b - 1e-5f) ad->better = (ad->j2 < 0.95f * ad->j) ? (uint8_t)(ad->better + 1u) : 0u;
        if (ad->better >= 3u) {
            float f = (b > 1e-6f) ? b2 / b : 1.0f;
            s->use.b_lr *= f;
            s->use.b_rl *= f;
            s->adapted = true;
            ad->better = 0u;
        }
        ad->j = ad->j2 = 0.0f;
        ad->win_ms = 0u;
        ad->win_rev = s->reversals;
    }
    /* doubt: the residual well above TRAINING's for 10 s */
    ad->res2 += (e * e - ad->res2) * dt / 10.0f;
    float lim = SM_DOUBT_FACTOR * fmaxf(s->stored.res, SM_RES_FLOOR_DPS);
    if (ad->res2 > lim * lim) {
        ad->doubt_ms += (uint32_t)(dt * 1000.0f + 0.5f);
        if (ad->doubt_ms >= SM_DOUBT_MS) {
            if (s->adapted) {
                sm_adapt_reset(s, SM_RESET_RESIDUAL);
                return SM_RESET_RESIDUAL;
            }
            ad->primed = false;                /* nothing adapted: just start the predictor again */
        }
    } else {
        ad->doubt_ms = 0u;
    }
    return SM_RESET_NONE;
}
