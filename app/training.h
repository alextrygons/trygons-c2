/*
 * training.h - TRAINING (calibration) mode for the outboard variant.
 *
 * Conditions: flat water, no current, slow constant boat speed, outboard
 * centred by hand at the start. Learns, in motor-time units (ms):
 *   gain      steady yaw rate per ms of motor run
 *   time const boat response time (63 % of a yaw-rate change)
 *   gap       backlash, both directions
 *   travel    centre -> each end stop
 *   trim      yaw rate with the outboard at the hand-centred position
 *
 * Phases (one at a time, with settling waits):
 *   CENTER  20 s motor off: trim (from compass heading slope), gyro bias
 *   STEP    small pulses right, steady yaw after each: gain, time const
 *   GAP     alternate known pulses left/right (3x each): gap per direction
 *   END_R   pulse train right until yaw stops increasing: right end stop
 *   END_L   pulse train left until yaw stops decreasing: left end stop
 *   SWEEP   full left -> centre (settle) -> full right
 *   RETURN  back to centre
 *   FIT     least-squares fit of all parameters to the whole log
 *   RESULT  parameters + quality; wait for the user to accept or reject
 *
 * The motor never holds full lock longer than needed: as soon as the yaw
 * rate shows the end stop, the motor reverses away from it.
 */
#ifndef TRAINING_H
#define TRAINING_H

#include <stdbool.h>
#include <stdint.h>

#include "calib.h"

/* Safety limits */
#define TR_MAX_TOTAL_MS          (15u * 60u * 1000u)
#define TR_MAX_PHASE_HEADING_DEG 90.0f    /* heading change allowed per phase */
#define TR_MAX_YAW_DPS           15.0f
#define TR_MAX_GAP_FRACTION      0.20f    /* gap > 20 % of lock-to-lock travel: implausible */
#define TR_MAX_TRIM_DPS          3.0f     /* outboard clearly not centred */
#define TR_MIN_FIT_R2            0.98f    /* good trainings reach 0.996+ in sim */

/* Timing */
#define TR_CENTER_MS         20000u
#define TR_STEP_PULSE_MS       500u
#define TR_STEP_MAX               8
#define TR_GAP_REPEATS            3
#define TR_SETTLE_MAX_MS     30000u
#define TR_MOVE_ON_MS          450u      /* END/SWEEP pulse train: on ...   */
#define TR_MOVE_OFF_MS          50u      /* ... and off                     */
#define TR_END_DETECT_MS      2000u      /* on-time without more yaw change = stop */
#define TR_STEP_PULSE_MAX_MS  2000u      /* STEP pulses grow up to this if the boat barely reacts */
#define TR_END_MAX_MS        60000u      /* no stop found within this: abort  */
#define TR_LOG_HZ               10u
#define TR_LOG_MAX            9000u      /* 15 min at 10 Hz */
#define TR_HIST_N                 8      /* 1-s yaw averages kept */
#define TR_YS_N                  21      /* 2 s of 10 Hz smoothed yaw */
#define TR_FIT_NQ                 5      /* nonlinear fit parameters */

typedef enum {
    TR_IDLE = 0,
    TR_CENTER,
    TR_STEP,
    TR_GAP,
    TR_END_RIGHT,
    TR_END_LEFT,
    TR_SWEEP,
    TR_RETURN,
    TR_FIT,
    TR_RESULT,      /* finished, waiting for accept / reject */
    TR_ABORTED
} tr_phase_t;

typedef enum {
    TR_OK = 0,
    TR_ABORT_USER,          /* Standby/Auto pressed */
    TR_ABORT_SENSOR,        /* compass timed out */
    TR_ABORT_HEADING,       /* turned more than allowed in one phase */
    TR_ABORT_TIME,          /* training took too long */
    TR_ABORT_YAW,           /* turning too fast */
    TR_ABORT_NO_RESPONSE,   /* motor pulses don't turn the boat */
    TR_ABORT_REVERSED,      /* motor turns the boat the wrong way */
    TR_ABORT_NO_END_STOP,   /* end stop not found */
    TR_ABORT_NOT_CENTRED,   /* trim too large at the start */
    TR_ABORT_IMPLAUSIBLE,   /* results don't make sense */
    TR_ABORT_REJECTED       /* user rejected the result */
} tr_result_t;

typedef struct {
    uint32_t now_ms;
    float    heading_deg;   /* compass */
    bool     heading_new;   /* a new compass sample arrived this step */
    float    yaw_dps;       /* gyro (raw) if from_gyro, else compass-derived */
    bool     from_gyro;
    int8_t   end_stop_hit;  /* +1 / -1: end stop detected (current / switch), 0 none */
    uint32_t end_stop_lag_ms; /* motor time at the stop before it was noticed */
} tr_input_t;

typedef struct {
    tr_phase_t  phase;
    tr_result_t result;
    uint8_t     sub;            /* step inside the phase */
    uint8_t     count;          /* repeats inside the phase */

    uint32_t last_ms, start_ms, phase_start_ms;
    float    phase_heading;     /* heading at phase start */
    float    heading;
    float    max_heading_change;/* worst |heading - phase start| */
    float    max_yaw;
    int8_t   motor;             /* current output -1/0/+1 */
    int8_t   hit;               /* end stop detected this step (from tr_input_t) */
    uint32_t hit_lag_ms;

    /* motor actions */
    int8_t   pulse_dir;         /* single pulse */
    uint32_t pulse_left_ms;     /* ... still to run */
    int8_t   move_dir;          /* pulse-train move */
    uint32_t move_left_ms;
    uint32_t move_cycle_ms;
    uint32_t move_done_ms;      /* on-time since the move started */
    uint32_t wait_ms;           /* time waited for settling */
    uint32_t wait_min_ms;

    /* yaw processing */
    float    gyro_bias;
    float    yaw;               /* latest, bias-corrected */
    float    acc_1s;
    uint32_t n_1s, ms_1s;
    float    hist[TR_HIST_N];   /* 1-s averages, newest last */
    uint8_t  hist_n;
    float    ys;                /* smoothed yaw (tau 1 s) */
    float    ys_ring[TR_YS_N];
    uint8_t  ys_i, ys_n;
    uint32_t ys_ms;
    float    noise;             /* std of 1-s averages during CENTER */

    /* log window (100 ms) */
    float    log_yaw_acc;
    uint32_t log_yaw_n;
    int32_t  log_motor_ms;
    uint32_t log_ms;
    uint32_t log_count;

    /* CENTER: heading regression and gyro mean */
    double   c_st, c_stt, c_sh, c_sth;
    uint32_t c_n;
    double   c_gyro;
    uint32_t c_gyro_n;
    float    c_unwrap, c_prev_heading;
    double   c_hist_sum, c_hist_sq;
    uint32_t c_hist_n;

    /* phase estimates */
    float    trim, gain, tconst;
    float    gap_rl, gap_lr, travel_r, travel_l;
    float    gap_fit;           /* gap from the model fit after GAP */
    float    step_lost_ms;      /* first STEP motor time lost in the gap */
    uint32_t step_pulse_ms;     /* current STEP pulse length */
    float    r_before;
    uint32_t cum_ms;
    uint8_t  step_n;
    float    step_cum[TR_STEP_MAX], step_r[TR_STEP_MAX];
    uint32_t gap_pulse_ms;
    float    gap_dr[2 * TR_GAP_REPEATS];   /* yaw change per reversal */
    float    pred_best;
    uint32_t on_at_best;

    /* fit */
    float    q[TR_FIT_NQ], dq[TR_FIT_NQ];
    double   best_cost, sst;
    uint8_t  fit_k;
    int8_t   fit_dir;
    uint16_t fit_evals;
    float    fit_gain, fit_trim;

    calib_data_t calib;         /* result (valid in TR_RESULT) */
} training_t;

/* Start training (motor off, CENTER phase). */
void training_start(training_t *t, uint32_t now_ms, float heading_deg);

/* One 20 Hz step. Returns the motor command: -1, 0 or +1. */
int8_t training_step(training_t *t, const tr_input_t *in);

/* Stop at once (motor off) with the given reason. */
void training_abort(training_t *t, tr_result_t why);

/* Short texts for display / logs. */
const char *training_phase_name(tr_phase_t p);
const char *training_result_text(tr_result_t r);

/* Test/diagnostic: model error (sum of squares) of the fit for parameters
 * q = [time const s, gap ms, travel right ms, travel left ms, start slack ms]
 * over the current log. Also returns the matching gain and trim. */
double training_fit_eval(const training_t *t, const float q[TR_FIT_NQ],
                         float *gain, float *trim);

#endif /* TRAINING_H */
