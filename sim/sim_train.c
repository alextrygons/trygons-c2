/*
 * sim_train.c - Tests for the outboard TRAINING (calibration) mode.
 *
 * Usage: autopilot_sim train [seed]
 *
 * 1. 5 random "true" boats (motor speed, gap, end stops, boat gain, response
 *    time, trim), each trained at sea state 1 and 2 with a gyro (small bias
 *    and noise). Table: true vs learned vs error.
 * 2. Course keeping with default settings, learned settings and settings
 *    from the true values.
 * 3. Safety: abort button, compass failure, implausible gap, boat turning
 *    too fast, motor wired backwards, training refused when not allowed,
 *    calibration storage checksum.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "autopilot.h"
#include "boat_model.h"
#include "hal_sim.h"
#include "sim_run.h"

#define TRAIN_SPEED_KN   3.0f       /* "slow constant speed" */
#define TRAIN_MAX_MS     (20u * 60u * 1000u)
#define TRAIN_START_MS   30000u     /* boat cruising steadily before training starts */
#define GYRO_BIAS_DPS    0.05f
#define GYRO_NOISE_DPS   0.02f

typedef struct {
    float speed;        /* motor turning speed, deg/s (negative = wired backwards) */
    float gap;          /* backlash, deg */
    float travel_r;     /* hand-centred position -> right stop, deg */
    float travel_l;     /* ... -> left stop, deg */
    float K;            /* boat: deg/s yaw per deg outboard (at training speed) */
    float T;            /* boat response time constant, s */
    float trim_deg;     /* hand-centring error + prop walk, equivalent deg */
    bool  start_left;   /* backlash initially taken up to the left */
} true_set_t;

typedef struct {
    bool         started;
    tr_result_t  result;
    tr_phase_t   last_phase;      /* phase reached (or where it aborted) */
    calib_data_t calib;
    float        duration_s;
    float        max_heading_change;
    float        max_yaw;
    float        stall_total_s, stall_max_s;
    int          motor_steps_after_stop;   /* motor on after training ended */
    float        stop_delay_s;    /* abort request -> motor off */
    bool         alarm;
    ap_config_t  cfg;             /* learned config (if accepted) */
    bool         accepted;
    training_t   train;           /* copy at the end (diagnostics) */
} train_out_t;

/* ---------------------------------------------------------------- random -- */

static bool g_trace;
static uint32_t s_rng;
static float urand(float lo, float hi)
{
    s_rng ^= s_rng << 13; s_rng ^= s_rng >> 17; s_rng ^= s_rng << 5;
    return lo + (hi - lo) * (float)(s_rng >> 8) / 16777216.0f;
}

static true_set_t random_set(void)
{
    true_set_t s;
    s.speed = urand(1.5f, 5.0f);
    s.gap = urand(0.5f, 3.5f);
    s.travel_r = urand(25.0f, 38.0f);
    s.travel_l = s.travel_r + urand(-4.0f, 4.0f);
    s.K = urand(0.08f, 0.20f);
    s.T = urand(1.5f, 6.0f);
    s.trim_deg = urand(-2.0f, 2.0f);
    s.start_left = urand(0.0f, 1.0f) < 0.5f;
    return s;
}

/* What a perfect calibration of this boat would contain. */
static calib_data_t true_calib(const true_set_t *s)
{
    calib_data_t c;
    memset(&c, 0, sizeof c);
    float sp = fabsf(s->speed);
    c.gain_dps_per_ms = s->K * sp / 1000.0f;
    c.time_const_s = s->T;
    c.gap_ms = s->gap / sp * 1000.0f;
    c.gap_rl_ms = c.gap_lr_ms = c.gap_ms;
    c.travel_right_ms = s->travel_r / sp * 1000.0f;
    c.travel_left_ms = s->travel_l / sp * 1000.0f;
    c.trim_dps = s->K * s->trim_deg;
    c.fit_r2 = 1.0f;
    c.confidence = 1.0f;
    c.full_angle_deg = s->travel_r + s->travel_l;
    calib_seal(&c);
    return c;
}

static ap_config_t config_from_calib(const calib_data_t *c, uint8_t ss)
{
    autopilot_t tmp;
    ap_config_t cfg;
    autopilot_default_config_outboard(&cfg);
    cfg.sea_state = ss;
    autopilot_init(&tmp, &cfg);
    autopilot_apply_calibration(&tmp, c);
    return tmp.cfg;
}

/* -------------------------------------------------------------- training -- */

/* Run TRAINING on a simulated boat. abort_ms: press Standby/Auto then;
 * fail_ms: compass dies then; mode_big: use the big-boat config (training
 * must be refused). */
static train_out_t run_training(const true_set_t *s, uint8_t ss, uint32_t seed,
                                uint32_t abort_ms, uint32_t fail_ms, bool accept)
{
    train_out_t o;
    memset(&o, 0, sizeof o);
    o.stop_delay_s = -1.0f;

    boat_t boat;
    rudder_actuator_t rud;
    autopilot_t ap;
    ap_config_t cfg;

    boat_init(&boat, 90.0f, TRAIN_SPEED_KN, seed);
    boat.K_override = s->K;
    boat.T_override = s->T;
    boat.wave_amp_deg = (float)ss;
    boat.current_deg = s->trim_deg;      /* flat water, no current: only trim */
    rudder_init_outboard(&rud, s->speed, s->speed, s->gap);
    rud.max_angle_deg = s->travel_r;
    rud.max_angle_neg_deg = s->travel_l;
    if (s->start_left) rud.drive_deg = -0.5f * s->gap;

    sim_hal_set_time_ms(0);
    sim_hal_set_truth(boat.heading_deg, rud.angle_deg);
    sim_hal_set_yaw_truth(boat.yaw_rate_dps);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, GYRO_BIAS_DPS, GYRO_NOISE_DPS);
    hal_init();
    autopilot_default_config_outboard(&cfg);
    cfg.sea_state = ss;
    autopilot_init(&ap, &cfg);

    uint32_t next_ap = 0, ended_at = 0;
    float heading_ref = boat.heading_deg;
    (void)heading_ref;

    for (uint32_t t = 0; t <= TRAIN_MAX_MS; t += PHYSICS_DT_MS) {
        float ts = (float)t * 0.001f;
        sim_hal_set_time_ms(t);
        sim_hal_set_truth(boat.heading_deg, rud.angle_deg);
        sim_hal_set_yaw_truth(boat.yaw_rate_dps);
        if (fail_ms && t >= fail_ms) sim_hal_fail_heading(true);
        if (abort_ms && t == abort_ms) sim_hal_press(HAL_BTN_STANDBY_AUTO);

        if (t == next_ap) {
            next_ap += AP_PERIOD_MS;
            autopilot_step(&ap);
            if (g_trace && ap.mode == AP_MODE_TRAINING && t % 500u == 0u) {
                float pr = 0.0f;
                const training_t *tt = &ap.train;
                if (tt->ys_n >= TR_YS_N) pr = tt->ys + tt->tconst * (tt->ys - tt->ys_ring[tt->ys_i]) / 2.0f;
                printf("TR %6.1f %-9s sub%u mot%+d done%5u best@%5u p%+6.2f best%+6.2f ys%+6.2f "
                       "true_yaw%+6.2f angle%+6.1f stop%s\n",
                       ts, training_phase_name(tt->phase), tt->sub, tt->motor, (unsigned)tt->move_done_ms,
                       (unsigned)tt->on_at_best, pr, tt->pred_best, tt->ys, boat.yaw_rate_dps, rud.angle_deg,
                       rud.stall_run_s > 0 ? "*" : "");
            }
            if (t == TRAIN_START_MS) o.started = autopilot_start_training(&ap);

            float m = sim_hal_motor_command();
            if (o.started && ap.mode == AP_MODE_TRAINING) {
                if (ap.train.phase != TR_ABORTED) o.last_phase = ap.train.phase;
                if (ap.train.phase == TR_RESULT) { ended_at = t; break; }
            }
            if (o.started && ap.mode != AP_MODE_TRAINING) {
                if (!ended_at) ended_at = t;
                if (m != 0.0f) o.motor_steps_after_stop++;
                if (t - ended_at >= 3000u) break;      /* watch 3 s after the stop */
            }
            uint32_t req = abort_ms ? abort_ms : fail_ms;
            if (req && t >= req && o.stop_delay_s < 0.0f && m == 0.0f && ap.mode != AP_MODE_TRAINING)
                o.stop_delay_s = (float)(t - req) * 0.001f;
            if (!o.started && t > TRAIN_START_MS) break;
        }
        rudder_step(&rud, sim_hal_motor_command(), PHYSICS_DT_MS * 0.001f);
        boat_step(&boat, rud.angle_deg, ts, PHYSICS_DT_MS * 0.001f);
    }

    o.train = ap.train;
    o.result = ap.train.result;
    o.calib = ap.train.calib;
    o.duration_s = (float)(ended_at ? ended_at : next_ap) * 0.001f - TRAIN_START_MS * 0.001f;
    o.max_heading_change = ap.train.max_heading_change;
    o.max_yaw = ap.train.max_yaw;
    o.stall_total_s = rud.stall_s;
    o.stall_max_s = rud.stall_max_s;
    o.alarm = ap.alarm;

    if (o.started && ap.mode == AP_MODE_TRAINING && ap.train.phase == TR_RESULT) {
        o.last_phase = TR_RESULT;
        if (accept) {
            o.accepted = autopilot_confirm_training(&ap, true, s->travel_r + s->travel_l);
            o.cfg = ap.cfg;
            o.calib = ap.train.calib;
        }
    }
    return o;
}

static results_t course_test(const true_set_t *s, uint8_t ss, uint32_t seed,
                             const ap_config_t *cfg)
{
    scenario_t sc;
    memset(&sc, 0, sizeof sc);
    sc.sea_state = ss;
    sc.seed = seed;
    sc.start_heading = 90.0f;
    sc.end_ms = 300000u;
    sc.turn_at_ms = 60000u;
    sc.outboard = true;
    sc.true_speed_pos = sc.true_speed_neg = s->speed;
    sc.true_gap = s->gap;
    sc.boat_K = s->K;
    sc.boat_T = s->T;
    sc.set_current = true;
    sc.current_deg = s->trim_deg;
    sc.stop_right = s->travel_r;
    sc.stop_left = s->travel_l;
    sc.cfg = cfg;
    return run(&sc);
}

/* ------------------------------------------------------------- reporting -- */

static float pct(float learned, float truth)
{
    return (truth != 0.0f) ? 100.0f * (learned - truth) / truth : 0.0f;
}

static int g_inaccurate, g_unsafe;

static void row(const char *name, float truth, float learned, float lim_pct)
{
    float e = pct(learned, truth);
    const char *flag = (fabsf(e) > lim_pct) ? "  <-- INACCURATE" : "";
    if (fabsf(e) > lim_pct) g_inaccurate++;
    printf("    %-24s %10.3f %10.3f %+8.1f%%%s\n", name, truth, learned, e, flag);
}

static void row_abs(const char *name, float truth, float learned, float lim_abs, const char *unit)
{
    float e = learned - truth;
    const char *flag = (fabsf(e) > lim_abs) ? "  <-- INACCURATE" : "";
    if (fabsf(e) > lim_abs) g_inaccurate++;
    printf("    %-24s %10.3f %10.3f %+8.3f %s%s\n", name, truth, learned, e, unit, flag);
}

static void print_set(int i, const true_set_t *s)
{
    printf("\nTrue boat %d: motor %.2f deg/s, gap %.2f deg, stops R %.1f / L %.1f deg, "
           "K %.3f, T %.2f s, trim %+.2f deg, gap starts %s\n",
           i + 1, s->speed, s->gap, s->travel_r, s->travel_l, s->K, s->T, s->trim_deg,
           s->start_left ? "left" : "right");
}

int training_trace(uint32_t seed, int set, int ss)
{
    true_set_t sets[5];
    s_rng = seed ? seed : 1u;
    for (int i = 0; i < 5; i++) sets[i] = random_set();
    g_trace = true;
    train_out_t o = run_training(&sets[set - 1], (uint8_t)ss, seed * 31u + (uint32_t)(set - 1) * 7u + (uint32_t)ss, 0, 0, false);
    g_trace = false;
    printf("result: %s, est T %.2f gain %.3f gap_fit %.0f lost %.0f travel %.0f/%.0f\n",
           training_result_text(o.result),
           o.train.tconst, o.train.gain * 1000.0f, o.train.gap_fit, o.train.step_lost_ms, o.train.travel_r, o.train.travel_l);
    return 0;
}

int training_tests(uint32_t seed)
{
    const int nsets = 5;
    true_set_t sets[5];
    s_rng = seed ? seed : 1u;
    for (int i = 0; i < nsets; i++) sets[i] = random_set();
    g_inaccurate = 0;
    g_unsafe = 0;

    printf("\n=== Outboard TRAINING, %d random true boats x sea state 1 and 2, seed %u ===\n",
           nsets, (unsigned)seed);
    printf("Gyro bias %.2f deg/s, noise %.2f deg/s. Boat speed %.0f kn, flat water + trim.\n",
           GYRO_BIAS_DPS, GYRO_NOISE_DPS, TRAIN_SPEED_KN);
    printf("Learned values are in motor-time units; degrees use the true lock-to-lock\n"
           "angle as if the user measured it. INACCURATE = error > 15 %% (trim: > 0.05 deg/s).\n");

    float worst[8] = { 0 };
    const char *wname[8] = { "gain", "time const", "gap", "travel R", "travel L",
                             "trim (abs, deg/s)", "turn speed (deg/s)", "gap (deg)" };
    results_t cr[5][3][3];   /* [set][ss index][default, learned, true] */

    for (int i = 0; i < nsets; i++) {
        const true_set_t *s = &sets[i];
        calib_data_t tc = true_calib(s);
        print_set(i, s);
        for (uint8_t ss = 1; ss <= 2; ss++) {
            uint32_t wseed = seed * 31u + (uint32_t)i * 7u + ss;
            train_out_t o = run_training(s, ss, wseed, 0, 0, true);
            printf("  Sea state %u: %s after %.0f s (last phase %s). Max heading change in a phase %.0f deg,"
                   " max yaw %.1f deg/s, end-stop push %.1f s total / %.1f s longest\n",
                   ss, o.accepted ? "trained + accepted" : training_result_text(o.result),
                   o.duration_s, training_phase_name(o.last_phase), o.max_heading_change,
                   o.max_yaw, o.stall_total_s, o.stall_max_s);
            if (!o.accepted) {
                const training_t *tt = &o.train;
                printf("    TRAINING FAILED. Phase estimates: gain %.3f, T %.2f s, gap %.0f/%.0f ms, "
                       "travel %.0f/%.0f ms, trim %.3f. Fit: T %.2f, gap %.0f, travel %.0f/%.0f, R2 %.3f\n",
                       tt->gain * 1000.0f, tt->tconst, tt->gap_fit, tt->gap_lr, tt->travel_r,
                       tt->travel_l, tt->trim, tt->q[0], tt->q[1], tt->q[2], tt->q[3],
                       tt->sst > 0.0 ? 1.0 - tt->best_cost / tt->sst : 0.0);
                g_unsafe++;
                continue;
            }
            {
                /* is the fit at the best answer the data allows? */
                float qt[TR_FIT_NQ] = { tc.time_const_s, tc.gap_ms, tc.travel_right_ms,
                                        tc.travel_left_ms,
                                        (s->start_left ? -0.5f : 0.5f) * tc.gap_ms };
                float gg, cc;
                double ct = training_fit_eval(&o.train, qt, &gg, &cc);
                printf("    [diag] model error at learned values %.3f, at TRUE values %.3f\n",
                       o.train.best_cost, ct);
            }
            const calib_data_t *c = &o.calib;
            printf("    fit R2 %.3f, confidence %.2f, gap R->L %.0f ms / L->R %.0f ms, gyro bias found %.3f\n",
                   c->fit_r2, c->confidence, c->gap_rl_ms, c->gap_lr_ms, c->gyro_bias_dps);
            printf("    %-24s %10s %10s %9s\n", "parameter", "true", "learned", "error");
            row("gain (deg/s per s run)", tc.gain_dps_per_ms * 1000.0f, c->gain_dps_per_ms * 1000.0f, 15.0f);
            row("time const (s)", tc.time_const_s, c->time_const_s, 15.0f);
            row("gap (ms)", tc.gap_ms, c->gap_ms, 15.0f);
            row("travel right (ms)", tc.travel_right_ms, c->travel_right_ms, 15.0f);
            row("travel left (ms)", tc.travel_left_ms, c->travel_left_ms, 15.0f);
            row_abs("trim (deg/s)", tc.trim_dps, c->trim_dps, 0.05f, "deg/s");
            ap_config_t lc = o.cfg;
            ap_config_t tcfg = config_from_calib(&tc, ss);
            row("-> turn speed (deg/s)", tcfg.outboard.turn_speed_deg_s, lc.outboard.turn_speed_deg_s, 15.0f);
            row("-> gap (deg)", tcfg.outboard.gap_deg, lc.outboard.gap_deg, 15.0f);
            printf("    -> max angle %.1f deg (true stops %.1f/%.1f), gains kp %.2f kd %.2f (true-value: %.2f %.2f)\n",
                   lc.outboard.max_angle_deg, s->travel_r, s->travel_l, lc.kp, lc.kd, tcfg.kp, tcfg.kd);

            float errs[8] = {
                pct(c->gain_dps_per_ms, tc.gain_dps_per_ms), pct(c->time_const_s, tc.time_const_s),
                pct(c->gap_ms, tc.gap_ms), pct(c->travel_right_ms, tc.travel_right_ms),
                pct(c->travel_left_ms, tc.travel_left_ms), c->trim_dps - tc.trim_dps,
                pct(lc.outboard.turn_speed_deg_s, tcfg.outboard.turn_speed_deg_s),
                pct(lc.outboard.gap_deg, tcfg.outboard.gap_deg)
            };
            for (int k = 0; k < 8; k++) if (fabsf(errs[k]) > worst[k]) worst[k] = fabsf(errs[k]);
            if (lc.outboard.max_angle_deg > fminf(s->travel_r, s->travel_l)) {
                printf("    UNSAFE: max angle beyond the real end stop\n");
                g_unsafe++;
            }
            if (o.stall_max_s > 3.0f) {
                printf("    UNSAFE: motor pushed against an end stop for %.1f s\n", o.stall_max_s);
                g_unsafe++;
            }

            ap_config_t dc;
            autopilot_default_config_outboard(&dc);
            dc.sea_state = ss;
            cr[i][ss][0] = course_test(s, ss, wseed + 100u, &dc);
            cr[i][ss][1] = course_test(s, ss, wseed + 100u, &lc);
            cr[i][ss][2] = course_test(s, ss, wseed + 100u, &tcfg);
        }
    }

    printf("\nWorst error over all trainings:\n");
    for (int k = 0; k < 8; k++)
        printf("  %-20s %6.2f%s\n", wname[k], worst[k], k == 5 ? " deg/s" : " %");

    printf("\nCourse keeping, 5 min (hold 90, +30 at 60 s), same boat and sea state:\n");
    printf("%-6s %-3s | %-31s | %-31s | %-31s\n", "boat", "ss",
           "default settings", "LEARNED settings", "TRUE-value settings");
    printf("%-6s %-3s | %-31s | %-31s | %-31s\n", "", "",
           "hold max/rms reach over puls", "hold max/rms reach over puls", "hold max/rms reach over puls");
    for (int i = 0; i < nsets; i++) {
        for (uint8_t ss = 1; ss <= 2; ss++) {
            printf("%-6d %-3u", i + 1, ss);
            for (int k = 0; k < 3; k++) {
                const results_t *r = &cr[i][ss][k];
                if (r->reach_s >= 0.0f)
                    printf(" | %5.2f/%4.2f %4.0fs %4.1f %4d   ", r->hold_max, r->hold_rms, r->reach_s, r->overshoot, r->pulses);
                else
                    printf(" | %5.2f/%4.2f  >40s  n/a %4d   ", r->hold_max, r->hold_rms, r->pulses);
            }
            const results_t *l = &cr[i][ss][1], *tv = &cr[i][ss][2];
            if (l->hold_max > tv->hold_max * 1.3f + 0.5f || l->overshoot > tv->overshoot * 1.3f + 1.0f) {
                printf("  <-- learned clearly worse");
                g_inaccurate++;
            }
            if (l->stall_s > 0.0f || l->hold_max > 10.0f || l->overshoot > 10.0f) {
                printf("  <-- UNSAFE");
                g_unsafe++;
            }
            printf("\n");
        }
    }

    /* ---- safety tests ---- */
    printf("\nSafety tests (true boat 1, sea state 2):\n");
    const true_set_t *s0 = &sets[0];
    train_out_t a = run_training(s0, 2, 99u, 150000u, 0, false);
    bool ok = a.result == TR_ABORT_USER && a.motor_steps_after_stop == 0 && !a.alarm &&
              a.stop_delay_s >= 0.0f && a.stop_delay_s <= 0.05f;
    printf("  Standby/Auto pressed at 150 s (phase %s): %s, motor off after %.2f s, "
           "motor steps after %d, alarm %s -> %s\n",
           training_phase_name(a.last_phase), training_result_text(a.result), a.stop_delay_s,
           a.motor_steps_after_stop, a.alarm ? "on" : "off", ok ? "OK" : "UNSAFE");
    if (!ok) g_unsafe++;

    train_out_t f = run_training(s0, 2, 99u, 0, 200000u, false);
    ok = f.result == TR_ABORT_SENSOR && f.motor_steps_after_stop == 0 && f.alarm;
    printf("  Compass fails at 200 s (phase %s): %s, motor off after %.2f s, alarm %s -> %s\n",
           training_phase_name(f.last_phase), training_result_text(f.result), f.stop_delay_s,
           f.alarm ? "on" : "off", ok ? "OK" : "UNSAFE");
    if (!ok) g_unsafe++;

    true_set_t big_gap = *s0;
    big_gap.travel_r = big_gap.travel_l = 15.0f;
    big_gap.gap = 8.0f;          /* 27 % of lock-to-lock travel */
    train_out_t g = run_training(&big_gap, 1, 5u, 0, 0, false);
    ok = g.result != TR_OK && g.motor_steps_after_stop == 0;
    printf("  Gap 8 deg on 30 deg travel (27 %%): %s (phase %s) -> %s\n",
           g.result == TR_OK ? "ACCEPTED" : training_result_text(g.result),
           training_phase_name(g.last_phase), ok ? "OK, rejected" : "UNSAFE, not rejected");
    if (!ok) g_unsafe++;

    true_set_t fast = *s0;
    fast.K = 0.7f; fast.speed = 6.0f; fast.T = 1.5f;
    train_out_t y = run_training(&fast, 1, 6u, 0, 0, false);
    ok = (y.result == TR_ABORT_YAW || y.result == TR_ABORT_HEADING) && y.motor_steps_after_stop == 0;
    printf("  Boat turning very fast (K 0.7, up to %.0f deg/s at lock): %s (phase %s), "
           "max yaw seen %.1f deg/s -> %s\n", fast.K * fast.travel_r,
           y.result == TR_OK ? "finished" : training_result_text(y.result),
           training_phase_name(y.last_phase), y.max_yaw, ok ? "OK, aborted" : "CHECK");
    if (!ok && y.result != TR_OK) g_unsafe++;

    true_set_t rev = *s0;
    rev.speed = -rev.speed;
    train_out_t w = run_training(&rev, 1, 7u, 0, 0, false);
    ok = w.result == TR_ABORT_REVERSED && w.motor_steps_after_stop == 0;
    printf("  Motor wired backwards: %s (phase %s) -> %s\n", training_result_text(w.result),
           training_phase_name(w.last_phase), ok ? "OK" : "UNSAFE");
    if (!ok) g_unsafe++;

    /* not allowed: big boat, or while in AUTO */
    {
        autopilot_t ap;
        ap_config_t cfg;
        sim_hal_set_time_ms(0);
        sim_hal_fail_heading(false);
        sim_hal_set_rudder_feedback(true);
        hal_init();
        autopilot_default_config(&cfg);
        autopilot_init(&ap, &cfg);
        autopilot_step(&ap);
        bool big_refused = !autopilot_start_training(&ap);

        autopilot_default_config_outboard(&cfg);
        hal_init();                 /* fresh compass sample at t = 0 */
        autopilot_init(&ap, &cfg);
        sim_hal_press(HAL_BTN_STANDBY_AUTO);
        autopilot_step(&ap);
        bool auto_refused = (ap.mode == AP_MODE_AUTO) && !autopilot_start_training(&ap);
        ok = big_refused && auto_refused;
        printf("  Training refused for the big boat: %s, refused while in AUTO: %s -> %s\n",
               big_refused ? "yes" : "NO", auto_refused ? "yes" : "NO", ok ? "OK" : "UNSAFE");
        if (!ok) g_unsafe++;
    }

    /* storage */
    {
        calib_data_t c = true_calib(s0), back;
        bool saved = calib_save(&c);
        bool loaded = calib_load(&back) && memcmp(&c, &back, sizeof c) == 0;
        back.gap_ms += 1.0f;                    /* corrupt one value */
        bool caught = !calib_valid(&back);
        calib_data_t old = c;
        old.version = 99u;
        old.checksum = calib_checksum(&old);
        bool version_caught = !calib_valid(&old);
        ok = saved && loaded && caught && version_caught;
        printf("  Calibration file: save %s, load %s, corrupted value rejected %s, "
               "wrong version rejected %s -> %s\n", saved ? "ok" : "FAIL", loaded ? "ok" : "FAIL",
               caught ? "yes" : "NO", version_caught ? "yes" : "NO", ok ? "OK" : "FAIL");
        if (!ok) g_unsafe++;
    }

    printf("\nSummary: %d unsafe, %d inaccurate\n", g_unsafe, g_inaccurate);
    return g_unsafe ? 2 : 0;
}
