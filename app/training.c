/*
 * training.c - TRAINING (calibration) mode for the outboard variant.
 * See training.h for the sequence. Pure C99, no HAL calls: the autopilot
 * feeds in sensor data and sends out the motor command.
 */
#include "training.h"

#include <math.h>
#include <string.h>

/* Whole-training log at 10 Hz (static: no malloc). */
static int8_t  s_log_motor[TR_LOG_MAX];   /* ms of motor run per 100 ms, signed */
static int16_t s_log_yaw[TR_LOG_MAX];     /* yaw rate, 0.01 deg/s */

/* "No end stop" travel for the quick fits, ms */
#define NO_STOP_MS  100000.0f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* a - b in -180..+180 deg */
static float heading_diff(float a, float b)
{
    float d = fmodf(a - b, 360.0f);
    if (d < -180.0f) d += 360.0f;
    if (d > 180.0f) d -= 360.0f;
    return d;
}

/* Pulse lengths are run in whole 50 ms control steps. */
static uint32_t round50(float ms)
{
    if (ms < 50.0f) return 50u;
    return (uint32_t)(ms / 50.0f + 0.5f) * 50u;
}

const char *training_phase_name(tr_phase_t p)
{
    static const char *const names[] = {
        "IDLE", "CENTER", "STEP", "GAP", "END_RIGHT", "END_LEFT",
        "SWEEP", "RETURN", "FIT", "RESULT", "ABORTED"
    };
    return ((unsigned)p < sizeof names / sizeof names[0]) ? names[p] : "?";
}

const char *training_result_text(tr_result_t r)
{
    static const char *const texts[] = {
        "ok", "aborted by user", "compass timed out", "turned too far in one phase",
        "took too long", "turning too fast", "motor does not turn the boat",
        "motor turns the boat the wrong way", "end stop not found",
        "outboard not centred at start", "results implausible", "rejected by user"
    };
    return ((unsigned)r < sizeof texts / sizeof texts[0]) ? texts[r] : "?";
}

/* --- small helpers ------------------------------------------------------ */

static void enter_phase(training_t *t, tr_phase_t p)
{
    t->phase = p;
    t->sub = 0;
    t->count = 0;
    t->phase_start_ms = t->last_ms;
    t->phase_heading = t->heading;
    t->motor = 0;
    t->pulse_left_ms = 0;
    t->move_dir = 0;
    t->move_left_ms = 0;
}

void training_abort(training_t *t, tr_result_t why)
{
    if (t->phase == TR_ABORTED) return;
    t->motor = 0;
    t->pulse_left_ms = 0;
    t->move_dir = 0;
    t->move_left_ms = 0;
    t->result = why;
    t->phase = TR_ABORTED;
}

static void start_pulse(training_t *t, int8_t dir, uint32_t ms)
{
    t->pulse_dir = dir;
    t->pulse_left_ms = ms;
}

static void start_move(training_t *t, int8_t dir, uint32_t on_ms)
{
    t->move_dir = dir;
    t->move_left_ms = on_ms;
    t->move_cycle_ms = 0;
    t->move_done_ms = 0;
}

static void stop_move(training_t *t)
{
    t->move_dir = 0;
    t->move_left_ms = 0;
}

static uint32_t settle_min_ms(const training_t *t)
{
    float ms = 3000.0f * t->tconst;
    return (ms < 4000.0f) ? 4000u : (uint32_t)ms;
}

static void start_wait(training_t *t, uint32_t min_ms)
{
    t->wait_ms = 0;
    t->wait_min_ms = min_ms;
}

/* Steady yaw reached? Compares the last 2 s with the 2 s before.
 * *steady = mean of the last 3 s. Gives up waiting after TR_SETTLE_MAX_MS. */
static bool settled(const training_t *t, float *steady)
{
    if (t->wait_ms < t->wait_min_ms || t->hist_n < 4) return false;
    const float *h = &t->hist[t->hist_n - 4];
    float newer = 0.5f * (h[2] + h[3]);
    float older = 0.5f * (h[0] + h[1]);
    if (fabsf(newer - older) < 0.03f + t->noise || t->wait_ms >= TR_SETTLE_MAX_MS) {
        *steady = (h[1] + h[2] + h[3]) / 3.0f;
        return true;
    }
    return false;
}

/* --- motor accounting and measurements (every step) --------------------- */

static void account(training_t *t, uint32_t elapsed)
{
    int8_t m = t->motor;

    t->log_motor_ms += (int32_t)m * (int32_t)elapsed;
    t->wait_ms += elapsed;

    if (m != 0 && t->pulse_left_ms > 0) {
        t->pulse_left_ms = (elapsed >= t->pulse_left_ms) ? 0u : t->pulse_left_ms - elapsed;
    }
    if (t->move_dir != 0) {
        if (m != 0) {
            t->move_left_ms = (elapsed >= t->move_left_ms) ? 0u : t->move_left_ms - elapsed;
            t->move_done_ms += elapsed;
        }
        t->move_cycle_ms = (t->move_cycle_ms + elapsed) % (TR_MOVE_ON_MS + TR_MOVE_OFF_MS);
    }
}

static int8_t motor_output(const training_t *t)
{
    if (t->pulse_left_ms > 0) return t->pulse_dir;
    if (t->move_dir != 0 && t->move_left_ms > 0 && t->move_cycle_ms < TR_MOVE_ON_MS)
        return t->move_dir;
    return 0;
}

static void push_hist(training_t *t, float v)
{
    if (t->hist_n == TR_HIST_N) {
        memmove(&t->hist[0], &t->hist[1], (TR_HIST_N - 1) * sizeof t->hist[0]);
        t->hist_n--;
    }
    t->hist[t->hist_n++] = v;
    if (t->phase == TR_CENTER) {
        t->c_hist_sum += v;
        t->c_hist_sq += (double)v * v;
        t->c_hist_n++;
    }
}

static void measure(training_t *t, const tr_input_t *in, uint32_t elapsed)
{
    float y = in->yaw_dps - (in->from_gyro ? t->gyro_bias : 0.0f);
    float dt = (float)elapsed * 0.001f;
    t->yaw = y;
    if (fabsf(t->ys) > t->max_yaw) t->max_yaw = fabsf(t->ys);

    /* CENTER needs raw gyro and the heading trend */
    if (t->phase == TR_CENTER) {
        if (in->from_gyro) { t->c_gyro += in->yaw_dps; t->c_gyro_n++; }
        if (in->heading_new) {
            t->c_unwrap += heading_diff(in->heading_deg, t->c_prev_heading);
            t->c_prev_heading = in->heading_deg;
            double ts = (double)(in->now_ms - t->phase_start_ms) * 0.001;
            t->c_st += ts; t->c_stt += ts * ts;
            t->c_sh += t->c_unwrap; t->c_sth += ts * t->c_unwrap;
            t->c_n++;
        }
    }

    /* 1-s averages */
    t->acc_1s += y; t->n_1s++; t->ms_1s += elapsed;
    if (t->ms_1s >= 1000u) {
        push_hist(t, t->acc_1s / (float)t->n_1s);
        t->acc_1s = 0.0f; t->n_1s = 0; t->ms_1s -= 1000u;
    }

    /* smoothed yaw and a 2 s ring of it (10 Hz) */
    t->ys += (y - t->ys) * (dt / (1.0f + dt));
    t->ys_ms += elapsed;
    if (t->ys_ms >= 100u) {
        t->ys_ms -= 100u;
        t->ys_ring[t->ys_i] = t->ys;
        t->ys_i = (uint8_t)((t->ys_i + 1u) % TR_YS_N);
        if (t->ys_n < TR_YS_N) t->ys_n++;
    }

    /* 10 Hz log */
    t->log_yaw_acc += y; t->log_yaw_n++; t->log_ms += elapsed;
    if (t->log_ms >= 1000u / TR_LOG_HZ) {
        t->log_ms -= 1000u / TR_LOG_HZ;
        if (t->log_count < TR_LOG_MAX) {
            float yy = clampf(t->log_yaw_acc / (float)t->log_yaw_n, -300.0f, 300.0f);
            s_log_motor[t->log_count] = (int8_t)clampf((float)t->log_motor_ms, -100.0f, 100.0f);
            s_log_yaw[t->log_count] = (int16_t)lrintf(yy * 100.0f);
            t->log_count++;
        }
        t->log_yaw_acc = 0.0f; t->log_yaw_n = 0; t->log_motor_ms = 0;
    }
}

/* --- boat model fit (used after STEP, after GAP and at the end) --------- */

/* Simulate the outboard (backlash, end stops, response lag) over the log
 * and solve gain + trim by linear least squares. Returns the sum of
 * squared errors. q = [time const s, gap ms, travel right ms, travel left
 * ms, start slack ms]. *end_angle (optional) = model outboard angle (ms
 * units) at the end of the log. */
static double fit_cost(const training_t *t, const float q[TR_FIT_NQ],
                       float *gain, float *trim, float *end_angle)
{
    float half = 0.5f * q[1], lr = q[2], ll = q[3];
    float a = 0.0f, d = q[4], x = 0.0f;
    float alpha = 1.0f - expf(-1.0f / ((float)TR_LOG_HZ * q[0]));
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;

    for (uint32_t i = 0; i < t->log_count; i++) {
        d += (float)s_log_motor[i];
        if (d - a > half) a = d - half;
        else if (a - d > half) a = d + half;
        if (a > lr)  { a = lr;  if (d > lr + half)  d = lr + half; }
        if (a < -ll) { a = -ll; if (d < -ll - half) d = -ll - half; }
        x += alpha * (a - x);
        double y = (double)s_log_yaw[i] * 0.01;
        n += 1; sx += x; sy += y; sxx += (double)x * x; sxy += x * y; syy += y * y;
    }
    double den = n * sxx - sx * sx;
    double g = (den > 1e-9) ? (n * sxy - sx * sy) / den : 0.0;
    double c = (n > 0) ? (sy - g * sx) / n : 0.0;
    *gain = (float)g;
    *trim = (float)c;
    if (end_angle) *end_angle = a;
    double sse = syy - 2 * g * sxy - 2 * c * sy + g * g * sxx + 2 * g * c * sx + n * c * c;
    return (sse > 0.0) ? sse : 0.0;
}

double training_fit_eval(const training_t *t, const float q[TR_FIT_NQ],
                         float *gain, float *trim)
{
    return fit_cost(t, q, gain, trim, NULL);
}

/* After STEP (all moves to the right, no end stop): fit time constant and
 * the motor time lost in the gap before the outboard first moved. */
static double step_model(const training_t *t, float tc, float lost, float *g, float *c)
{
    float q[TR_FIT_NQ] = { tc, lost, NO_STOP_MS, NO_STOP_MS, -0.5f * lost };
    return fit_cost(t, q, g, c, NULL);
}

static void quick_fit_step(training_t *t)
{
    float cum = (float)t->cum_ms, g, c;
    float best_tc = t->tconst, best_lost = 0.0f;
    double best = 1e300;

    for (int i = 0; i < 16; i++) {                  /* T 0.5 .. 15 s */
        float tc = 0.5f * powf(30.0f, (float)i / 15.0f);
        for (int j = 0; j <= 12; j++) {
            float lost = cum * (float)j / 16.0f;
            double e = step_model(t, tc, lost, &g, &c);
            if (e < best) { best = e; best_tc = tc; best_lost = lost; }
        }
    }
    float dtc = 0.15f * best_tc, dl = fmaxf(50.0f, cum / 32.0f);
    for (int it = 0; it < 60; it++) {
        bool better = false;
        float cand[4][2] = { { best_tc + dtc, best_lost }, { best_tc - dtc, best_lost },
                             { best_tc, best_lost + dl }, { best_tc, best_lost - dl } };
        for (int k = 0; k < 4; k++) {
            float tc = clampf(cand[k][0], 0.2f, 30.0f);
            float lost = clampf(cand[k][1], 0.0f, cum);
            double e = step_model(t, tc, lost, &g, &c);
            if (e < best) { best = e; best_tc = tc; best_lost = lost; better = true; }
        }
        if (!better) { dtc *= 0.5f; dl *= 0.5f; }
    }
    step_model(t, best_tc, best_lost, &g, &c);
    t->tconst = best_tc;
    t->step_lost_ms = best_lost;
    t->gain = g;
}

/* After GAP: fit the gap (and refine T). The start slack follows from the
 * motor time lost in STEP: first right move lost gap/2 - s0. */
static double gap_model(const training_t *t, float tc, float gap, float *g, float *c, float *end)
{
    float s0 = clampf(0.5f * gap - t->step_lost_ms, -0.5f * gap, 0.5f * gap);
    float q[TR_FIT_NQ] = { tc, gap, NO_STOP_MS, NO_STOP_MS, s0 };
    return fit_cost(t, q, g, c, end);
}

static void quick_fit_gap(training_t *t)
{
    float pmax = (float)t->gap_pulse_ms, g, c, end;
    float best_gap = 0.0f, best_tc = t->tconst;
    double best = 1e300;

    for (int j = 0; j <= 20; j++) {
        float gap = pmax * (float)j / 20.0f;
        double e = gap_model(t, best_tc, gap, &g, &c, NULL);
        if (e < best) { best = e; best_gap = gap; }
    }
    float dg = fmaxf(25.0f, pmax / 40.0f), dtc = 0.1f * best_tc;
    for (int it = 0; it < 60; it++) {
        bool better = false;
        float cand[4][2] = { { best_tc, best_gap + dg }, { best_tc, best_gap - dg },
                             { best_tc + dtc, best_gap }, { best_tc - dtc, best_gap } };
        for (int k = 0; k < 4; k++) {
            float tc = clampf(cand[k][0], 0.2f, 30.0f);
            float gap = clampf(cand[k][1], 0.0f, pmax);
            double e = gap_model(t, tc, gap, &g, &c, NULL);
            if (e < best) { best = e; best_gap = gap; best_tc = tc; better = true; }
        }
        if (!better) { dg *= 0.5f; dtc *= 0.5f; }
    }
    gap_model(t, best_tc, best_gap, &g, &c, &end);
    t->tconst = best_tc;
    t->gap_fit = best_gap;
    t->gain = g;
    t->r_before = end;          /* reused: outboard position now, ms */
}

/* --- phases --------------------------------------------------------------- */

static void finish_center(training_t *t)
{
    /* Trim from the compass heading slope (not affected by gyro bias) */
    double n = (double)t->c_n;
    double den = n * t->c_stt - t->c_st * t->c_st;
    t->trim = (t->c_n > 10 && den > 0.0)
            ? (float)((n * t->c_sth - t->c_st * t->c_sh) / den) : 0.0f;

    /* Gyro bias = gyro mean - true mean yaw; correct what is logged so far */
    if (t->c_gyro_n > 0) {
        t->gyro_bias = (float)(t->c_gyro / (double)t->c_gyro_n) - t->trim;
        int16_t corr = (int16_t)lrintf(t->gyro_bias * 100.0f);
        for (uint32_t i = 0; i < t->log_count; i++) s_log_yaw[i] = (int16_t)(s_log_yaw[i] - corr);
        for (uint8_t i = 0; i < t->hist_n; i++) t->hist[i] -= t->gyro_bias;
        for (uint8_t i = 0; i < TR_YS_N; i++) t->ys_ring[i] -= t->gyro_bias;
        t->ys -= t->gyro_bias;
    }

    /* Noise of 1-s averages (waves + sensor), used for settle detection */
    if (t->c_hist_n > 2) {
        double m = t->c_hist_sum / (double)t->c_hist_n;
        double v = t->c_hist_sq / (double)t->c_hist_n - m * m;
        t->noise = (v > 0.0) ? (float)sqrt(v) : 0.0f;
    }
}

static void phase_center(training_t *t)
{
    t->motor = 0;
    if (t->last_ms - t->phase_start_ms < TR_CENTER_MS) return;
    finish_center(t);
    if (fabsf(t->trim) > TR_MAX_TRIM_DPS) {
        training_abort(t, TR_ABORT_NOT_CENTRED);
        return;
    }
    t->r_before = t->trim;
    t->step_pulse_ms = TR_STEP_PULSE_MS;
    enter_phase(t, TR_STEP);
}

static void phase_step(training_t *t)
{
    float r;
    switch (t->sub) {
    case 0:
        start_pulse(t, +1, t->step_pulse_ms);
        t->sub = 1;
        break;
    case 1:
        if (t->pulse_left_ms == 0) {
            /* wait ~3 response times so gain and lag can be told apart */
            start_wait(t, (uint32_t)clampf(3000.0f * t->tconst, 8000.0f, 25000.0f));
            t->sub = 2;
        }
        break;
    default:
        if (!settled(t, &r)) break;
        t->cum_ms += t->step_pulse_ms;
        t->step_cum[t->step_n] = (float)t->cum_ms;
        t->step_r[t->step_n] = r;
        t->step_n++;

        float resp = r - t->trim;
        if (resp < -fmaxf(0.15f, 3.0f * t->noise)) { training_abort(t, TR_ABORT_REVERSED); return; }

        bool enough = (t->step_n >= 3 && resp >= 0.6f) ||
                      (t->step_n >= 2 && resp >= 1.2f) ||
                      t->step_n >= TR_STEP_MAX;
        if (resp >= 0.1f) quick_fit_step(t);    /* better time constant for the next wait */
        if (!enough) {
            /* boat barely reacts (or still in the gap): longer pulses */
            if (r - t->r_before < 0.1f && t->step_pulse_ms < TR_STEP_PULSE_MAX_MS)
                t->step_pulse_ms *= 2u;
            t->r_before = r;
            t->sub = 0;
            break;
        }
        if (resp < 0.15f) { training_abort(t, TR_ABORT_NO_RESPONSE); return; }
        quick_fit_step(t);
        if (t->gain <= 0.0f) { training_abort(t, TR_ABORT_REVERSED); return; }
        t->r_before = r;
        enter_phase(t, TR_GAP);
        break;
    }
}

static void phase_gap(training_t *t)
{
    float r;
    switch (t->sub) {
    case 0: {
        if (t->count == 0) {
            /* swing the outboard from where it is (right) to about as far
             * on the left: symmetric, so the boat doesn't keep turning one
             * way, and well inside the end stops */
            float pos = (float)t->cum_ms - t->step_lost_ms;
            t->gap_pulse_ms = round50(clampf(2.0f * pos + t->step_lost_ms, 600.0f, 6000.0f));
        }
        int8_t dir = (t->count % 2u == 0u) ? -1 : +1;   /* left first: right->left gap */
        start_pulse(t, dir, t->gap_pulse_ms);
        t->sub = 1;
        break;
    }
    case 1:
        if (t->pulse_left_ms == 0) { start_wait(t, settle_min_ms(t)); t->sub = 2; }
        break;
    default:
        if (!settled(t, &r)) break;
        t->gap_dr[t->count] = r - t->r_before;
        t->r_before = r;
        t->count++;
        if (t->count < 2u * TR_GAP_REPEATS) { t->sub = 0; break; }

        /* per direction: gap = pulse - part of the pulse that moved the outboard */
        float rl = 0.0f, lr = 0.0f;
        for (int k = 0; k < 2 * TR_GAP_REPEATS; k++) {
            float moved = fabsf(t->gap_dr[k]) / t->gain;
            float g = fmaxf(0.0f, (float)t->gap_pulse_ms - moved);
            if (k % 2 == 0) rl += g; else lr += g;
        }
        t->gap_rl = rl / TR_GAP_REPEATS;
        t->gap_lr = lr / TR_GAP_REPEATS;
        quick_fit_gap(t);       /* also leaves the outboard position in r_before */
        enter_phase(t, TR_END_RIGHT);
        break;
    }
}

/* Model error with an end stop at `l` on one side (the other side as
 * already known, or none). */
static double stop_model2(const training_t *t, float lr, float ll)
{
    float g, c;
    float s0 = clampf(0.5f * t->gap_fit - t->step_lost_ms, -0.5f * t->gap_fit, 0.5f * t->gap_fit);
    float q[TR_FIT_NQ] = { t->tconst, t->gap_fit, lr, ll, s0 };
    return fit_cost(t, q, &g, &c, NULL);
}

/* For the left stop the right one (found earlier) is re-fitted too: an
 * error there would otherwise look like a left stop. */
static double stop_model(const training_t *t, bool right, float l, float *lr_used)
{
    if (right) {
        *lr_used = l;
        return stop_model2(t, l, t->travel_l > 0.0f ? t->travel_l : NO_STOP_MS);
    }
    double best = 1e300;
    for (int k = -6; k <= 6; k++) {
        float lr = t->travel_r * (1.0f + 0.025f * (float)k);
        double e = stop_model2(t, lr, l);
        if (e < best) { best = e; *lr_used = lr; }
    }
    return best;
}

/*
 * Model-based end stop check during a move. `hi` = where the outboard
 * would be now if nothing stopped it (ms from centre). Fits the stop
 * position over [lo, hi] against the whole log. The stop counts as found
 * when the model with a stop explains the yaw clearly better than without,
 * and the outboard would by now be well past it (the yaw shortfall is
 * larger than the noise). The model includes the boat's response lag, so
 * the result doesn't depend on a noisy yaw derivative.
 */
static bool end_stop_check(training_t *t, bool right, float lo, float hi, float *where)
{
    if (hi <= lo + 100.0f) return false;
    float lr, best_lr = t->travel_r;
    double free_cost = stop_model(t, right, NO_STOP_MS, &lr);
    double best = 1e300;
    float best_l = hi, a = lo, b = hi;

    for (int it = 0; it < 3; it++) {
        float step = (b - a) / 16.0f;
        for (int j = 0; j <= 16; j++) {
            float l = a + step * (float)j;
            double e = stop_model(t, right, l, &lr);
            if (e < best) { best = e; best_l = l; best_lr = lr; }
        }
        a = fmaxf(lo, best_l - step);
        b = fminf(hi, best_l + step);
    }
    *where = best_l;
    if (!right) t->q[2] = best_lr;      /* refined right stop (applied when confirmed) */
    double need = fmax(0.2, 500.0 * (double)t->noise * (double)t->noise);
    float shortfall = (hi - best_l) * t->gain;          /* deg/s */
    bool seen = (free_cost - best) > need && shortfall >= fmaxf(0.3f, 6.0f * t->noise);

    /* A best fit right at the lower edge of the search is a model mismatch
     * (e.g. gap estimate off in waves), not a stop: keep moving, the real
     * stop will show up clearly further on. */
    if (best_l < lo + 0.1f * (hi - lo)) seen = false;

    /* A real stop stays where it is as more data comes in; a wrong one
     * (noise, small model errors) moves. Need two checks that agree. */
    bool confirmed = seen && t->pred_best > 0.0f &&
                     fabsf(best_l - t->pred_best) < fmaxf(300.0f, 0.05f * best_l);
    t->pred_best = seen ? best_l : -1.0f;              /* reused: last candidate */
    return confirmed;
}

static void phase_end(training_t *t, int8_t dir)
{
    if (t->sub == 0) {
        start_move(t, dir, TR_END_MAX_MS);
        t->on_at_best = 0;              /* reused: on-time of the last check */
        t->pred_best = -1.0f;           /* no stop candidate yet */
        t->sub = 1;
        return;
    }
    /* check every 500 ms of motor run; right: engaged after GAP,
     * left: the gap must be crossed first */
    uint32_t ignore = (dir > 0) ? 1000u : (uint32_t)t->gap_fit + 1000u;
    if (t->move_done_ms >= ignore && t->move_done_ms - t->on_at_best >= 500u) {
        t->on_at_best = t->move_done_ms;
        float where, run = (float)t->move_done_ms;
        bool found;
        if (dir > 0) {
            float pos0 = t->r_before;                   /* position from the GAP fit */
            found = end_stop_check(t, true, pos0, pos0 + run, &where);
            if (found) t->travel_r = where;
        } else {
            float hi = run - t->gap_fit - t->travel_r;  /* past centre, to the left */
            /* the two sides can't differ by more than 2x (see plausibility):
             * an "end stop" earlier than half the right travel is not real */
            found = end_stop_check(t, false, 0.5f * t->travel_r, hi, &where);
            if (found) { t->travel_l = where; t->travel_r = t->q[2]; }
        }
        if (found) {
            stop_move(t);               /* reverse away from the stop at once */
            enter_phase(t, (dir > 0) ? TR_END_LEFT : TR_SWEEP);
            return;
        }
    }
    if (t->move_left_ms == 0) training_abort(t, TR_ABORT_NO_END_STOP);
}

static void phase_sweep(training_t *t)
{
    float r;
    switch (t->sub) {
    case 0:     /* full left -> centre */
        start_move(t, +1, round50(t->gap_fit + t->travel_l));
        t->sub = 1;
        break;
    case 1:
        if (t->move_left_ms == 0) { stop_move(t); start_wait(t, settle_min_ms(t)); t->sub = 2; }
        break;
    case 2:
        if (!settled(t, &r)) break;
        start_move(t, +1, round50(t->travel_r + 3000.0f));    /* centre -> full right */
        t->pred_best = -1.0f;
        t->on_at_best = 0;
        t->sub = 3;
        break;
    default: {
        /* stop once the model sees the right end stop again (or at the cap) */
        bool found = false;
        float where;
        if (t->move_done_ms >= (uint32_t)(0.8f * t->travel_r) &&
            t->move_done_ms - t->on_at_best >= 500u) {
            t->on_at_best = t->move_done_ms;
            found = end_stop_check(t, true, 0.5f * t->travel_r, (float)t->move_done_ms, &where);
        }
        if (found || t->move_left_ms == 0) {
            stop_move(t);
            enter_phase(t, TR_RETURN);
        }
        break;
    }
    }
}

static void phase_return(training_t *t)
{
    float r;
    switch (t->sub) {
    case 0:
        start_move(t, -1, round50(t->gap_fit + t->travel_r));
        t->sub = 1;
        break;
    case 1:
        if (t->move_left_ms == 0) { stop_move(t); start_wait(t, settle_min_ms(t)); t->sub = 2; }
        break;
    default:
        if (settled(t, &r)) enter_phase(t, TR_FIT);
        break;
    }
}

/* --- final least-squares fit ---------------------------------------------- */

static void fit_clamp(float q[TR_FIT_NQ])
{
    q[0] = clampf(q[0], 0.2f, 30.0f);
    q[2] = clampf(q[2], 300.0f, 120000.0f);
    q[3] = clampf(q[3], 300.0f, 120000.0f);
    q[1] = clampf(q[1], 0.0f, 0.5f * fminf(q[2], q[3]));
    q[4] = clampf(q[4], -0.5f * q[1], 0.5f * q[1]);
}

static void fit_init(training_t *t)
{
    float gap0 = t->gap_fit;
    t->q[0] = t->tconst;
    t->q[1] = gap0;
    t->q[2] = t->travel_r;
    t->q[3] = t->travel_l;
    t->q[4] = 0.5f * gap0 - t->step_lost_ms;
    t->dq[0] = 0.2f * t->tconst;
    t->dq[1] = 0.2f * gap0 + 50.0f;
    t->dq[2] = 0.05f * t->travel_r;
    t->dq[3] = 0.05f * t->travel_l;
    t->dq[4] = 0.25f * gap0 + 25.0f;
    fit_clamp(t->q);
    t->best_cost = fit_cost(t, t->q, &t->fit_gain, &t->fit_trim, NULL);
    t->fit_k = 0;
    t->fit_dir = +1;
    t->fit_evals = 1;

    double m = 0, s = 0;
    for (uint32_t i = 0; i < t->log_count; i++) m += s_log_yaw[i] * 0.01;
    m /= (double)(t->log_count ? t->log_count : 1u);
    for (uint32_t i = 0; i < t->log_count; i++) {
        double e = s_log_yaw[i] * 0.01 - m;
        s += e * e;
    }
    t->sst = s;
}

/* One pattern-search trial. Returns true when the fit has converged. */
static bool fit_iterate(training_t *t)
{
    static const float tol[TR_FIT_NQ] = { 0.02f, 5.0f, 10.0f, 10.0f, 5.0f };
    float trial[TR_FIT_NQ], g, c;

    memcpy(trial, t->q, sizeof trial);
    trial[t->fit_k] += (float)t->fit_dir * t->dq[t->fit_k];
    fit_clamp(trial);
    double cost = fit_cost(t, trial, &g, &c, NULL);
    t->fit_evals++;

    if (cost < t->best_cost) {
        memcpy(t->q, trial, sizeof trial);
        t->best_cost = cost;
        t->fit_gain = g;
        t->fit_trim = c;
        t->dq[t->fit_k] *= 1.5f;        /* going well: take bigger steps */
        t->fit_dir = +1;
        t->fit_k = (uint8_t)((t->fit_k + 1u) % TR_FIT_NQ);
    } else if (t->fit_dir > 0) {
        t->fit_dir = -1;
    } else {
        t->dq[t->fit_k] *= 0.5f;
        t->fit_dir = +1;
        t->fit_k = (uint8_t)((t->fit_k + 1u) % TR_FIT_NQ);
    }

    bool small = true;
    for (int k = 0; k < TR_FIT_NQ; k++) if (t->dq[k] > tol[k]) small = false;
    return small || t->fit_evals >= 1500u;
}

static void fit_finish(training_t *t)
{
    calib_data_t *c = &t->calib;
    float gain = t->fit_gain;
    float travel = t->q[2] + t->q[3];

    if (gain <= 0.0f) { training_abort(t, TR_ABORT_REVERSED); return; }

    memset(c, 0, sizeof *c);
    c->gain_dps_per_ms = gain;
    c->time_const_s = t->q[0];
    c->gap_ms = t->q[1];
    c->travel_right_ms = t->q[2];
    c->travel_left_ms = t->q[3];
    c->trim_dps = t->fit_trim;
    c->gyro_bias_dps = t->gyro_bias;

    /* per-direction gaps from the GAP phase, recomputed with the fitted gain */
    float rl = 0.0f, lr = 0.0f;
    for (int k = 0; k < 2 * TR_GAP_REPEATS; k++) {
        float g = fmaxf(0.0f, (float)t->gap_pulse_ms - fabsf(t->gap_dr[k]) / gain);
        if (k % 2 == 0) rl += g; else lr += g;
    }
    c->gap_rl_ms = rl / TR_GAP_REPEATS;
    c->gap_lr_ms = lr / TR_GAP_REPEATS;

    c->fit_r2 = (t->sst > 0.0) ? (float)(1.0 - t->best_cost / t->sst) : 0.0f;

    /* Confidence: fit quality, reduced if the phase estimates disagree */
    float d = fabsf(t->gain - gain) / gain;
    d = fmaxf(d, fabsf(t->travel_r - t->q[2]) / t->q[2]);
    d = fmaxf(d, fabsf(t->travel_l - t->q[3]) / t->q[3]);
    d = fmaxf(d, 5.0f * fabsf(t->gap_fit - t->q[1]) / travel);
    c->confidence = clampf(c->fit_r2, 0.0f, 1.0f) * (1.0f - fminf(d, 0.5f));

    bool ok = c->fit_r2 >= TR_MIN_FIT_R2
           && c->gap_ms <= TR_MAX_GAP_FRACTION * travel
           && c->time_const_s > 0.2f && c->time_const_s < 30.0f
           && t->q[2] < 2.0f * t->q[3] && t->q[3] < 2.0f * t->q[2];
    if (!ok) { training_abort(t, TR_ABORT_IMPLAUSIBLE); return; }

    calib_seal(c);
    enter_phase(t, TR_RESULT);
}

static void phase_fit(training_t *t)
{
    t->motor = 0;
    if (t->sub == 0) {
        fit_init(t);
        t->sub = 1;
        return;
    }
    /* spread the work over several control steps */
    for (int i = 0; i < 25; i++) {
        if (fit_iterate(t)) { fit_finish(t); return; }
    }
}

/* --- public ---------------------------------------------------------------- */

void training_start(training_t *t, uint32_t now_ms, float heading_deg)
{
    memset(t, 0, sizeof *t);
    t->last_ms = now_ms;
    t->start_ms = now_ms;
    t->heading = heading_deg;
    t->c_prev_heading = heading_deg;
    t->tconst = 4.0f;           /* guess until STEP measures it */
    enter_phase(t, TR_CENTER);
}

int8_t training_step(training_t *t, const tr_input_t *in)
{
    if (t->phase == TR_IDLE || t->phase == TR_RESULT || t->phase == TR_ABORTED) {
        t->motor = 0;
        return 0;
    }

    uint32_t elapsed = in->now_ms - t->last_ms;
    account(t, elapsed);
    t->last_ms = in->now_ms;
    t->heading = in->heading_deg;
    measure(t, in, elapsed);

    /* Safety limits while the boat is being steered */
    if (t->phase < TR_FIT) {
        float dh = fabsf(heading_diff(t->heading, t->phase_heading));
        if (dh > t->max_heading_change) t->max_heading_change = dh;
        if (in->now_ms - t->start_ms > TR_MAX_TOTAL_MS) training_abort(t, TR_ABORT_TIME);
        else if (dh > TR_MAX_PHASE_HEADING_DEG)         training_abort(t, TR_ABORT_HEADING);
        else if (fabsf(t->ys) > TR_MAX_YAW_DPS)         training_abort(t, TR_ABORT_YAW);
        if (t->phase == TR_ABORTED) return 0;
    }

    switch (t->phase) {
    case TR_CENTER:    phase_center(t);     break;
    case TR_STEP:      phase_step(t);       break;
    case TR_GAP:       phase_gap(t);        break;
    case TR_END_RIGHT: phase_end(t, +1);    break;
    case TR_END_LEFT:  phase_end(t, -1);    break;
    case TR_SWEEP:     phase_sweep(t);      break;
    case TR_RETURN:    phase_return(t);     break;
    case TR_FIT:       phase_fit(t);        break;
    default:                                break;
    }

    t->motor = (t->phase < TR_FIT) ? motor_output(t) : 0;
    return t->motor;
}
