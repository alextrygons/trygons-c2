/*
 * trainsrv.h - TRAINING of the DroneCAN steering POSITION servo (CLAUDE.md T,
 * docs/dronecan_design.md 8.2; step 11e). Pure C99, no HAL. The DC-motor TRAINING
 * of the old hardware (training.c) is unchanged.
 *
 * From STANDBY by the PC (as today; CREWED; open water; steady speed - the
 * smallboat TRAINING mode holds the slow throttle). Learns, in servo units
 * (-1..+1, + = starboard): the servo speed per direction r_right / r_left, the
 * play b_lr / b_rl, the boat's K (yaw deg/s per unit at the TRAINING speed) and
 * tau, the trim a0, the gyro bias, the model fit and a confidence.
 *
 * Phases (each command ramp at the WORST-CASE servo speed, except the LARGE
 * steps - they measure the speed; servo excursion <= +-0.5):
 *   POSITION  position unknown after a power-on (OD-7): command 0 and wait the
 *             worst-case travel; then settle
 *   CENTRE    u = 0 for 20 s: gyro bias (compass slope), trim, noise
 *   SMALL     holds at +0.04, +0.12, -0.04, -0.12 (approached in one direction
 *             each: no play in the differences): K right / left, tau (fit of
 *             the +0.04 -> +0.12 response), a first play estimate (the
 *             reversal +0.12 -> -0.04)
 *   LARGE     steps -A -> +A -> -A (A <= 0.15, the yaw kept below 17 deg/s):
 *             the yaw-rate rise while the servo travels gives r per direction
 *             (a model fit with K / tau known; also a play estimate)
 *   STAIR     from a held position reverse by a bisection of steps until the yaw
 *             rate answers (> 3 x noise): the smallest answered reversal = the
 *             play, both directions
 *   VERIFY    a mixed sequence driven WITH the new values (rate limit + play
 *             compensation): model fit R^2, the agreement of the phases =
 *             confidence (< 0.6: shown, NOT used)
 *   RETURN    back to the centre at the measured speed, then RESULT (accept /
 *             reject from the PC as today)
 * Aborts: yaw > 17 deg/s (ArduPilot ATC_STR_RAT_MAX), gyro / compass stale,
 * turning more than 90 deg in a phase, 15 min in total, no response, the boat
 * turning the wrong way (the steering wire sign, TO CONFIRM C6) or implausible
 * results; the caller aborts too (joystick, STANDBY, a node lost, slamming).
 */
#ifndef TRAINSRV_H
#define TRAINSRV_H

#include <stdbool.h>
#include <stdint.h>

#include "steermodel.h"
#include "training.h"

#define TRS_YAW_MAX_DPS     17.0f       /* ATC_STR_RAT_MAX */
#define TRS_MAX_U           0.5f        /* servo excursion (TO CONFIRM) */
#define TRS_MAX_MS          (15u * 60u * 1000u)
#define TRS_MIN_CONFIDENCE  0.6f        /* below: shown, not used */
#define TRS_BUF             420u        /* one 21 s trace at 20 Hz */
#define TRS_VBUF            720u        /* the VERIFY trace (36 s) */

typedef enum {
    TRS_IDLE = 0, TRS_POSITION, TRS_CENTRE, TRS_SMALL, TRS_LARGE, TRS_FIT, TRS_STAIR, TRS_VERIFY, TRS_RETURN,
    TRS_RESULT, TRS_ABORTED
} trs_phase_t;

typedef struct {
    uint32_t now_ms;
    float    gyro_dps;                  /* raw gyro */
    bool     gyro_ok;                   /* fresh */
    float    heading_deg;
    bool     heading_new;
    bool     heading_ok;
    float    v_kn;                      /* speed (GPS), 0 if unknown */
} trs_in_t;

typedef struct {
    float u[TRS_BUF], y[TRS_BUF];
    uint16_t n;
} trs_trace_t;

typedef struct {
    trs_phase_t phase;
    tr_result_t result;
    uint8_t  sub, iter;
    uint32_t start_ms, phase_ms, sub_ms, last_ms;
    float    cmd;                       /* the command this step */
    float    rate;                      /* command ramp speed (worst case until LARGE measured) */
    float    target;                    /* ramp target */
    float    phase_heading, max_turn;
    /* CENTRE */
    double   c_t, c_tt, c_h, c_th, c_g;
    uint32_t c_n;
    float    c_unwrap, c_prev_h;
    double   s1_sum, s1_sq;
    uint32_t s1_n, s1_ms;
    float    s1_acc;
    uint32_t s1_cnt;
    float    bias, w_c, noise;
    /* hold averages */
    double   h_sum;
    uint32_t h_n;
    float    hold[4];                   /* SMALL steady yaw at +s1, +s2, -s1, -s2 */
    float    w_before;                  /* STAIR / LARGE: the steady yaw before a test */
    uint32_t hold_ms;                   /* the holds after the first fit: 6 tau + the average */
    /* estimates */
    float    k_r, k_l, k, tau, lag, b_small, b_large, b_lr, b_rl, r_right, r_left, a0, amp;
    float    lo, hi, base;              /* STAIR bisection */
    uint8_t  stair_dir;                 /* 0: right-engaged base, reversals to the left (b_rl); 1: the mirror */
    /* traces */
    trs_trace_t tsmall, tl_up, tl_dn;
    float    tl_p0[2];
    float    lw0[2];                    /* LARGE: the steady yaw before each step */
    float    vu[TRS_VBUF], vy[TRS_VBUF];
    uint16_t vn;
    float    v_p0, v_a0;                /* the VERIFY start: servo position, outboard */
    steer_t  vm;                        /* VERIFY: the model with the new values (drives the commands) */
    float    v_targets[4];
    /* fit progress (spread over the steps) */
    uint16_t fit_i;
    double   fit_best[2];
    float    fit_b[2], fit_T[2];
    /* result */
    float    r2, res, confidence, v_sum;
    uint32_t v_n;
    sm_params_t out;
} trs_t;

void  trs_start(trs_t *t, uint32_t now_ms, float heading_deg);
/* one 20 Hz step: returns the servo command (units). sm = the steering model (its position known?) */
float trs_step(trs_t *t, const trs_in_t *in, const steer_t *sm);
void  trs_abort(trs_t *t, tr_result_t why);
const char *trs_phase_name(trs_phase_t p);

/* test / diagnostic: the SSE of the model on a trace (servo speed r up / down, play b, start position p0,
 * start outboard a0 side, K, tau, the command lag (s), the start yaw w0 at outboard a_ref) */
double trs_model_sse(const float *u, const float *y, uint32_t n, float dt, float p0, float a_start, float r_up,
                     float r_dn, float b, float k, float tau, float lag_s, float w0, float a_ref);

#endif /* TRAINSRV_H */
