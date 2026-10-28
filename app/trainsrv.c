/*
 * trainsrv.c - TRAINING of the DroneCAN steering position servo. See trainsrv.h.
 */
#include "trainsrv.h"

#include <math.h>
#include <string.h>

#define S1          0.04f       /* SMALL holds */
#define S2          0.12f
#define HOLD_MS     10000u
#define AVG_MS      4000u       /* the steady value = the mean of the last AVG_MS of a hold */
#define PRE_MS      8000u       /* LARGE: settle before a step */
#define STEP_MS     12000u      /* LARGE: record after a step */
#define STAIR_BASE  0.06f
#define STAIR_ITERS 7u
#define VER_MS      8000u       /* VERIFY: per target */
#define FIT_NB      41u         /* LARGE fit grid: play 0 .. 0.2 */
#define FIT_NT      40u         /* ... travel time 0.1 .. 40 s (log) */

static float clampf(float x, float lo, float hi)
{
    if (!(x >= lo)) return lo;
    if (x > hi) return hi;
    return x;
}

static float wrap180(float d)
{
    d = fmodf(d, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

const char *trs_phase_name(trs_phase_t p)
{
    static const char *const n[] = { "idle", "position", "centre", "small steps", "large steps", "fit", "reversal staircase",
                                     "verify", "return", "result", "aborted" };
    return ((unsigned)p <= (unsigned)TRS_ABORTED) ? n[p] : "?";
}

double trs_model_sse(const float *u, const float *y, uint32_t n, float dt, float p0, float a_start, float r_up,
                     float r_dn, float b, float k, float tau, float lag_s, float w0, float a_ref)
{
    /* 10 ms sub-steps; the command u[i] (sent at sample i, after y[i] was measured) acts lag_s later (the
     * CAN path + the servo's own reaction); before the trace the servo was commanded to p0 */
    const float h_dt = 0.01f;
    uint32_t sub = (uint32_t)(dt / h_dt + 0.5f);
    if (sub < 1u) sub = 1u;
    uint32_t lag = (uint32_t)(lag_s / h_dt + 0.5f);
    float p = p0, a = a_start, w = w0, h = 0.5f * b;
    if (!(tau > 0.01f)) tau = 0.01f;
    float alpha = 1.0f - expf(-h_dt / tau);
    double sse = 0.0;
    for (uint32_t i = 0; i < n; i++) {
        double e = (double)y[i] - (double)w;
        sse += e * e;
        for (uint32_t j = 0; j < sub; j++) {
            /* the command in force at this sub-step: the one sent `lag` sub-steps ago */
            int32_t back = (int32_t)(i * sub + j) - (int32_t)lag;
            float c = (back < 0) ? p0 : u[(uint32_t)back / sub];
            if (p < c) { p += r_up * h_dt; if (p > c) p = c; }
            else if (p > c) { p -= r_dn * h_dt; if (p < c) p = c; }
            if (p - a > h) a = p - h;
            else if (a - p > h) a = p + h;
            w += (w0 + k * (a - a_ref) - w) * alpha;
        }
    }
    return sse;
}

void trs_abort(trs_t *t, tr_result_t why)
{
    if (t->phase == TRS_RESULT || t->phase == TRS_ABORTED) return;
    t->phase = TRS_ABORTED;
    t->result = why;
}

static void phase(trs_t *t, trs_phase_t p, uint32_t now)
{
    t->phase = p;
    t->sub = 0u;
    t->iter = 0u;
    t->phase_ms = t->sub_ms = now;
    t->max_turn = 0.0f;
}

void trs_start(trs_t *t, uint32_t now_ms, float heading_deg)
{
    memset(t, 0, sizeof *t);
    t->start_ms = t->last_ms = now_ms;
    t->rate = SM_RATE_WORST;            /* never faster than the worst case before LARGE measured it */
    t->phase_heading = heading_deg;
    t->c_prev_h = heading_deg;
    t->result = TR_OK;
    phase(t, TRS_POSITION, now_ms);
}

/* the command toward target at the ramp speed: true when there */
static bool ramp(trs_t *t, float target, float dt)
{
    float step = t->rate * dt;
    t->cmd += clampf(target - t->cmd, -step, step);
    return fabsf(t->cmd - target) < 1e-6f;
}

static void hold_reset(trs_t *t)
{
    t->h_sum = 0.0;
    t->h_n = 0u;
}

/* the mean yaw of the last `last_ms` of a hold that began at sub_ms and lasts total_ms */
static void hold_acc(trs_t *t, uint32_t now, uint32_t total_ms, uint32_t last_ms, float y)
{
    if (now - t->sub_ms + last_ms >= total_ms) {
        t->h_sum += y;
        t->h_n++;
    }
}

static float hold_mean(const trs_t *t)
{
    return t->h_n ? (float)(t->h_sum / (double)t->h_n) : 0.0f;
}

/* a hold long enough to settle: 6 tau + the averaging window, at least min_ms, the trace buffer the limit */
static uint32_t settle_ms(const trs_t *t, uint32_t min_ms)
{
    float ms = 1000.0f * (6.0f * t->tau) + (float)AVG_MS;
    float cap = 50.0f * (float)(TRS_BUF - 10u);
    if (ms > cap) ms = cap;
    return (ms > (float)min_ms) ? (uint32_t)ms : min_ms;
}

static void rec(trs_trace_t *tr, float u, float y)
{
    if (tr->n < TRS_BUF) {
        tr->u[tr->n] = u;
        tr->y[tr->n] = y;
        tr->n++;
    }
}

/* the unit response of the SMALL step +S1 -> +S2 (no reversal: the outboard follows the command, which never
 * moves faster than the servo) for tau / lag, and the best step size for it (linear least squares): the step
 * size is FITTED, not taken from the hold means (a hold of a slow boat may not have settled) */
static double small_fit(const trs_t *t, float tau, float lag, float dt, float *step)
{
    const trs_trace_t *tr = &t->tsmall;
    const float h_dt = 0.01f;
    uint32_t sub = (uint32_t)(dt / h_dt + 0.5f), lagn = (uint32_t)(lag / h_dt + 0.5f);
    if (sub < 1u) sub = 1u;
    float alpha = 1.0f - expf(-h_dt / (tau > 0.01f ? tau : 0.01f));
    float g = 0.0f, k = 1.0f / (S2 - S1);
    double sgg = 0.0, sgy = 0.0, syy = 0.0;
    for (uint32_t i = 0; i < tr->n; i++) {
        double yy = (double)tr->y[i] - (double)t->hold[0];
        sgg += (double)g * g;
        sgy += (double)g * yy;
        syy += yy * yy;
        for (uint32_t j = 0; j < sub; j++) {
            int32_t back = (int32_t)(i * sub + j) - (int32_t)lagn;
            float c = (back < 0) ? S1 : tr->u[(uint32_t)back / sub];
            g += (k * (c - S1) - g) * alpha;
        }
    }
    double d = (sgg > 1e-12) ? sgy / sgg : 0.0;
    if (step) *step = (float)d;
    return syy - d * sgy;
}

static void fit_tau(trs_t *t, float dt)
{
    double best = 1e30;
    float bt = 2.0f, bl = 0.0f, st = 0.0f;
    for (uint32_t l = 0; l <= 15u; l++) {
        float lag = 0.01f * (float)l;
        for (uint32_t i = 0; i < 30u; i++) {
            float tau = 0.2f * powf(150.0f, (float)i / 29.0f);
            double e = small_fit(t, tau, lag, dt, NULL);
            if (e < best) { best = e; bt = tau; bl = lag; }
        }
    }
    float lo = bt / 1.2f, hi = bt * 1.2f;
    for (uint32_t i = 0; i < 24u; i++) {
        float m1 = lo + 0.382f * (hi - lo), m2 = lo + 0.618f * (hi - lo);
        if (small_fit(t, m1, bl, dt, NULL) < small_fit(t, m2, bl, dt, NULL)) hi = m2; else lo = m1;
    }
    t->tau = 0.5f * (lo + hi);
    t->lag = bl;
    (void)small_fit(t, t->tau, t->lag, dt, &st);
    t->k_r = st / (S2 - S1);                    /* K to the right from the fitted step */
}

/* LARGE fit, one direction: SSE for play b and travel time T (over 2 x amp) */
static double large_sse(const trs_t *t, int d, float b, float T, float dt)
{
    const trs_trace_t *tr = d ? &t->tl_dn : &t->tl_up;
    float r = 2.0f * t->amp / T, h = 0.5f * b;
    float p0 = t->tl_p0[d];
    float a0 = d ? p0 - h : p0 + h;     /* up: left-engaged before the step; down: right-engaged */
    float w0 = t->lw0[d];
    return trs_model_sse(tr->u, tr->y, tr->n, dt, p0, a0, r, r, b, t->k, t->tau, t->lag, w0, a0);
}

static float grid_T(uint32_t j)
{
    return 0.1f * powf(400.0f, (float)j / (float)(FIT_NT - 1u));
}

float trs_step(trs_t *t, const trs_in_t *in, const steer_t *sm)
{
    uint32_t now = in->now_ms;
    float dt = (float)(uint32_t)(now - t->last_ms) * 0.001f;
    t->last_ms = now;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.05f;
    if (t->phase == TRS_IDLE || t->phase == TRS_RESULT || t->phase == TRS_ABORTED) return t->cmd;

    /* safety: every step */
    if (!in->gyro_ok || !in->heading_ok) { trs_abort(t, TR_ABORT_SENSOR); return t->cmd; }
    float y = in->gyro_dps - t->bias;
    if (fabsf(in->gyro_dps - t->bias) > TRS_YAW_MAX_DPS) { trs_abort(t, TR_ABORT_YAW); return t->cmd; }
    if (now - t->start_ms > TRS_MAX_MS) { trs_abort(t, TR_ABORT_TIME); return t->cmd; }
    float turn = fabsf(wrap180(in->heading_deg - t->phase_heading));
    if (turn > t->max_turn) t->max_turn = turn;
    if (t->phase != TRS_POSITION && t->max_turn > TR_MAX_PHASE_HEADING_DEG) { trs_abort(t, TR_ABORT_HEADING); return t->cmd; }
    if (in->v_kn > 0.5f) { t->v_sum += in->v_kn; t->v_n++; }

    switch (t->phase) {
    case TRS_POSITION:
        /* after a power-on the position is unknown: the first command (0, or what was commanded) and the
         * worst-case travel; then the centre at the worst-case speed, settle */
        if (t->iter == 0u) {
            t->cmd = sm->have ? sm->u : 0.0f;
            t->iter = 1u;
        }
        if (!sm_known(sm)) {
            t->sub_ms = now;
            break;
        }
        if (t->sub == 0u) {
            if (ramp(t, 0.0f, dt)) { t->sub = 1u; t->sub_ms = now; }
            break;
        }
        if (now - t->sub_ms >= 3000u) {
            phase(t, TRS_CENTRE, now);
            t->phase_heading = in->heading_deg;
            t->c_prev_h = in->heading_deg;
            t->c_unwrap = 0.0f;
        }
        break;

    case TRS_CENTRE: {
        t->cmd = 0.0f;
        if (in->heading_new) {
            float d = wrap180(in->heading_deg - t->c_prev_h);
            t->c_prev_h = in->heading_deg;
            t->c_unwrap += d;
            double ts = (double)(now - t->phase_ms) * 0.001;
            t->c_t += ts; t->c_tt += ts * ts; t->c_h += t->c_unwrap; t->c_th += ts * t->c_unwrap; t->c_n++;
        }
        t->c_g += in->gyro_dps;
        t->s1_acc += in->gyro_dps;
        t->s1_cnt++;
        if (now - t->s1_ms >= 1000u || t->s1_ms == 0u) {
            if (t->s1_ms != 0u && t->s1_cnt) {
                float m = t->s1_acc / (float)t->s1_cnt;
                t->s1_sum += m; t->s1_sq += (double)m * m; t->s1_n++;
            }
            t->s1_acc = 0.0f;
            t->s1_cnt = 0u;
            t->s1_ms = now;
        }
        if (now - t->phase_ms >= 20000u) {
            uint32_t ng = (uint32_t)((now - t->phase_ms) / 50u);
            float gm = (float)(t->c_g / (double)(ng ? ng : 1u));
            double nn = (double)t->c_n, den = nn * t->c_tt - t->c_t * t->c_t;
            float slope = (t->c_n > 10u && fabs(den) > 1e-9) ? (float)((nn * t->c_th - t->c_t * t->c_h) / den) : gm;
            t->bias = gm - slope;
            t->w_c = slope;
            double m = t->s1_n ? t->s1_sum / (double)t->s1_n : 0.0;
            double var = t->s1_n > 1u ? t->s1_sq / (double)t->s1_n - m * m : 0.0;
            t->noise = fmaxf(0.005f, (float)sqrt(var > 0.0 ? var : 0.0));
            if (fabsf(t->w_c) > 2.0f * TR_MAX_TRIM_DPS) { trs_abort(t, TR_ABORT_NOT_CENTRED); break; }
            phase(t, TRS_SMALL, now);
            t->phase_heading = in->heading_deg;
        }
        break;
    }

    case TRS_SMALL: {
        /* sub 2i: the ramp to hold i; sub 2i+1: hold i (HOLD_MS). Holds +S1, +S2 (both approached moving
         * right), -S1, -S2 (both moving left): the differences have no play in them */
        static const float tg[4] = { S1, S2, -S1, -S2 };
        uint8_t i = (uint8_t)(t->sub >> 1);
        /* the +S1 -> +S2 step response (tau): each sample = the yaw measured this step + the command sent
         * this step (the model applies it from the next sample on) */
        bool rec_now = (t->sub == 2u || t->sub == 3u);
        if ((t->sub & 1u) == 0u) {
            bool there = ramp(t, tg[i], dt);
            if (rec_now) rec(&t->tsmall, t->cmd, y);
            if (there) { t->sub++; t->sub_ms = now; hold_reset(t); }
            break;
        }
        if (rec_now) rec(&t->tsmall, t->cmd, y);
        /* the holds after the first step response last long enough for THIS boat (6 tau + the average) */
        uint32_t hold_ms = (i < 2u) ? HOLD_MS : t->hold_ms;
        hold_acc(t, now, hold_ms, AVG_MS, y);
        if (now - t->sub_ms < hold_ms) break;
        t->hold[i] = hold_mean(t);
        if (i == 0u) t->tsmall.n = 0u;                /* the trace starts with the ramp to +S2 */
        if (i == 1u) {
            /* tau, the lag and K to the right: fitted on the +S1 -> +S2 response (not the hold means) */
            fit_tau(t, 0.05f);
            if (!(t->tau > 0.1f && t->tau < 60.0f)) { trs_abort(t, TR_ABORT_IMPLAUSIBLE); break; }
            t->hold[1] = t->hold[0] + t->k_r * (S2 - S1);     /* the settled value */
            t->hold_ms = settle_ms(t, HOLD_MS);
        }
        if (i < 3u) { t->sub++; break; }
        /* K to the left (settled holds, no play in the difference), the play from the reversal +S2 -> -S1 */
        t->k_l = (t->hold[2] - t->hold[3]) / (S2 - S1);
        t->k = 0.5f * (t->k_r + t->k_l);
        if (!(fabsf(t->k) >= 0.3f)) { trs_abort(t, TR_ABORT_NO_RESPONSE); break; }
        if (t->k < 0.0f || t->k_r < 0.0f || t->k_l < 0.0f) { trs_abort(t, TR_ABORT_REVERSED); break; }
        t->b_small = clampf((S1 + S2) + (t->hold[2] - t->hold[1]) / t->k, 0.0f, 0.3f);
        t->amp = clampf(0.3f * TRS_YAW_MAX_DPS / t->k, 0.05f, 0.15f);
        phase(t, TRS_LARGE, now);
        t->phase_heading = in->heading_deg;
        break;
    }

    case TRS_LARGE:
        if (t->sub == 0u) {                           /* to -A (moving left: left-engaged), settle */
            if (!ramp(t, -t->amp, dt)) { t->sub_ms = now; hold_reset(t); break; }
            hold_acc(t, now, settle_ms(t, PRE_MS), 3000u, y);
            if (now - t->sub_ms < settle_ms(t, PRE_MS)) break;
            t->w_before = hold_mean(t);
            t->lw0[0] = t->w_before;
            t->tl_p0[0] = t->cmd;
            t->tl_up.n = 0u;
            t->cmd = t->amp;                          /* a STEP: the servo travels at its own speed */
            rec(&t->tl_up, t->cmd, y);                /* (this step's yaw, before the step acts) */
            t->sub = 1u;
            t->sub_ms = now;
            hold_reset(t);
            break;
        }
        if (t->sub == 1u) {
            rec(&t->tl_up, t->cmd, y);
            hold_acc(t, now, settle_ms(t, STEP_MS), 3000u, y);
            if (now - t->sub_ms < settle_ms(t, STEP_MS)) break;
            t->lw0[1] = hold_mean(t);                 /* (the steady yaw at +A: the start of the down step) */
            t->tl_p0[1] = t->cmd;
            t->tl_dn.n = 0u;
            t->cmd = -t->amp;
            rec(&t->tl_dn, t->cmd, y);
            t->sub = 2u;
            t->sub_ms = now;
            break;
        }
        rec(&t->tl_dn, t->cmd, y);
        if (now - t->sub_ms < settle_ms(t, STEP_MS)) break;
        phase(t, TRS_FIT, now);
        t->fit_best[0] = t->fit_best[1] = 1e30;
        t->phase_heading = in->heading_deg;
        break;

    case TRS_FIT:
        /* one play value per step (both directions), then the refinement: spread over the steps */
        if (t->fit_i < FIT_NB) {
            float b = 0.2f * (float)t->fit_i / (float)(FIT_NB - 1u);
            for (int d = 0; d < 2; d++)
                for (uint32_t j = 0; j < FIT_NT; j++) {
                    float T = grid_T(j);
                    double e = large_sse(t, d, b, T, 0.05f);
                    if (e < t->fit_best[d]) { t->fit_best[d] = e; t->fit_b[d] = b; t->fit_T[d] = T; }
                }
            t->fit_i++;
            break;
        }
        for (int d = 0; d < 2; d++) {
            float b0 = t->fit_b[d], T0 = t->fit_T[d];
            for (int ib = -5; ib <= 5; ib++)
                for (int it = -10; it <= 10; it++) {
                    float b = b0 + 0.001f * (float)ib, T = T0 * (1.0f + 0.015f * (float)it);
                    if (b < 0.0f || T < 0.05f) continue;
                    double e = large_sse(t, d, b, T, 0.05f);
                    if (e < t->fit_best[d]) { t->fit_best[d] = e; t->fit_b[d] = b; t->fit_T[d] = T; }
                }
        }
        t->r_right = 2.0f * t->amp / t->fit_T[0];
        t->r_left = 2.0f * t->amp / t->fit_T[1];
        t->b_large = 0.5f * (t->fit_b[0] + t->fit_b[1]);
        if (!(t->r_right > 0.02f && t->r_left > 0.02f)) { trs_abort(t, TR_ABORT_IMPLAUSIBLE); break; }
        /* from now the command never moves faster than the measured servo */
        t->rate = fminf(t->r_right, t->r_left);
        phase(t, TRS_STAIR, now);
        t->stair_dir = 0u;
        t->phase_heading = in->heading_deg;
        break;

    case TRS_STAIR: {
        float sgn = t->stair_dir ? 1.0f : -1.0f;      /* the test reversal: dir 0 to the left, dir 1 to the right */
        float base = t->stair_dir ? -STAIR_BASE : STAIR_BASE;
        uint32_t settle = (uint32_t)fmaxf(3000.0f, 1000.0f * (6.0f * t->tau + 1.0f));   /* settled: 6 tau */
        uint32_t test = (uint32_t)fmaxf(3000.0f, 1000.0f * (2.0f * t->tau + 1.0f));
        float thr = 3.0f * fmaxf(t->noise, 0.01f);
        switch (t->sub) {
        case 0:                                       /* the base, approached against the test direction */
            if (!ramp(t, base, dt)) { t->sub_ms = now; hold_reset(t); break; }
            hold_acc(t, now, settle, 1000u, y);
            if (now - t->sub_ms < settle) break;
            t->w_before = hold_mean(t);
            t->lo = 0.0f;
            t->hi = clampf(2.0f * fmaxf(fmaxf(t->b_small, t->b_large), 0.01f) + 0.02f, 0.03f, 0.25f);
            t->iter = 0u;
            t->sub = 1u;
            t->sub_ms = now;
            hold_reset(t);
            break;
        case 1: {                                     /* a test reversal of delta */
            float delta = 0.5f * (t->lo + t->hi);
            if (!ramp(t, base + sgn * delta, dt)) { t->sub_ms = now; hold_reset(t); break; }
            hold_acc(t, now, test, 1000u, y);
            if (now - t->sub_ms < test) break;
            float dw = (hold_mean(t) - t->w_before) * sgn;
            bool answered = dw > thr;
            if (answered) t->hi = delta; else t->lo = delta;
            t->iter++;
            t->sub = answered ? 2u : 4u;              /* answered: the outboard moved - set the base up again */
            t->sub_ms = now;
            hold_reset(t);
            break;
        }
        case 2:                                       /* beyond the test point, then back to the base */
            if (!ramp(t, base + sgn * (t->hi + 0.08f), dt)) break;
            t->sub = 3u;
            t->sub_ms = now;
            hold_reset(t);
            break;
        case 3:                                       /* (after a moved outboard) */
        default: {                                    /* 4: not answered - the outboard may still have moved a
                                                         little (below the threshold): settle the same way, else
                                                         the next baseline is biased (the play found too large) */
            uint32_t wait = settle;
            if (!ramp(t, base, dt)) { t->sub_ms = now; hold_reset(t); break; }
            hold_acc(t, now, wait, 1000u, y);
            if (now - t->sub_ms < wait) break;
            t->w_before = hold_mean(t);
            if (t->iter < STAIR_ITERS) { t->sub = 1u; t->sub_ms = now; hold_reset(t); break; }
            float b = clampf(0.5f * (t->lo + t->hi) - thr / t->k, 0.0f, 0.3f);
            if (t->stair_dir == 0u) {
                t->b_rl = b;                          /* reversing right -> left */
                t->stair_dir = 1u;
                t->sub = 0u;
            } else {
                t->b_lr = b;                          /* reversing left -> right */
                /* VERIFY: driven by the model with the new values (rate limit + play compensation) */
                sm_params_t p;
                sm_defaults(&p);
                p.r_right = t->r_right; p.r_left = t->r_left; p.b_lr = t->b_lr; p.b_rl = t->b_rl;
                p.k = t->k; p.tau = t->tau;
                sm_init(&t->vm, &p, 30.0f, 30.0f, true, t->cmd);
                t->vm.stored = p;
                t->vm.use = p;
                t->vm.a = t->cmd + 0.5f * sm_play(&t->vm);  /* left-engaged (the base was approached moving left) */
                t->vm.dir = -1;
                t->vm.have = true;
                t->v_targets[0] = 0.08f; t->v_targets[1] = -0.04f; t->v_targets[2] = 0.04f; t->v_targets[3] = 0.0f;
                t->vn = 0u;
                t->v_p0 = t->cmd;                     /* the VERIFY start */
                t->v_a0 = t->vm.a;
                phase(t, TRS_VERIFY, now);
                t->phase_heading = in->heading_deg;
            }
            break;
        }
        }
        break;
    }

    case TRS_VERIFY: {
        uint32_t el = now - t->phase_ms;
        uint32_t k = el / VER_MS;
        if (t->vn < TRS_VBUF) { t->vy[t->vn] = y; }
        if (k < 4u) {
            t->cmd = sm_drive(&t->vm, t->v_targets[k], 0.005f, dt);
            if (t->vn < TRS_VBUF) { t->vu[t->vn] = t->cmd; t->vn++; }
            break;
        }
        /* fit: the model with every value found, against the yaw while driving */
        float b = 0.5f * (t->b_lr + t->b_rl);
        double sse = trs_model_sse(t->vu, t->vy, t->vn, 0.05f, t->v_p0, t->v_a0, t->r_right, t->r_left, b,
                                   t->k, t->tau, t->lag, t->w_before, t->v_a0);
        double m = 0.0, sst = 0.0;
        for (uint32_t i = 0; i < t->vn; i++) m += t->vy[i];
        m /= (double)(t->vn ? t->vn : 1u);
        for (uint32_t i = 0; i < t->vn; i++) sst += ((double)t->vy[i] - m) * ((double)t->vy[i] - m);
        t->r2 = (sst > 1e-9) ? (float)(1.0 - sse / sst) : 0.0f;
        t->res = (float)sqrt(sse / (double)(t->vn ? t->vn : 1u));
        float kag = 1.0f - fabsf(t->k_r - t->k_l) / fmaxf(t->k_r + t->k_l, 1e-3f);
        float bs = 0.5f * (t->b_lr + t->b_rl);
        float bag = 1.0f - fabsf(t->b_large - bs) / fmaxf(fmaxf(t->b_large, bs), 0.02f);
        t->confidence = clampf(t->r2, 0.0f, 1.0f) * (0.5f + 0.5f * clampf(kag, 0.0f, 1.0f)) *
                        (0.5f + 0.5f * clampf(bag, 0.0f, 1.0f));
        phase(t, TRS_RETURN, now);
        t->phase_heading = in->heading_deg;
        break;
    }

    case TRS_RETURN:
        if (!ramp(t, 0.0f, dt)) { t->sub_ms = now; break; }
        if (now - t->sub_ms < 1000u) break;
        sm_defaults(&t->out);
        t->out.r_right = t->r_right;
        t->out.r_left = t->r_left;
        t->out.b_lr = t->b_lr;
        t->out.b_rl = t->b_rl;
        t->out.k = t->k;
        t->out.tau = t->tau;
        t->out.a0 = -t->w_c / t->k;
        t->out.v_ref = t->v_n ? t->v_sum / (float)t->v_n : 0.0f;
        t->out.res = t->res;
        t->out.trained = true;
        t->phase = TRS_RESULT;
        t->result = TR_OK;
        break;

    default:
        break;
    }
    t->cmd = clampf(t->cmd, -TRS_MAX_U, TRS_MAX_U);
    return t->cmd;
}
