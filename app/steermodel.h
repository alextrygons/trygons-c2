/*
 * steermodel.h - Steering model of the DroneCAN 360 deg POSITION servo
 * (CLAUDE.md T, docs/dronecan_design.md section 8; step 11e). Pure C99, no HAL.
 *
 * The servo node sends NOTHING back: the AURIX only knows what it COMMANDED.
 * Model (servo units: -1 .. +1 = full left .. full right, + = to starboard):
 *   1. command u (what is sent, after the backlash compensation);
 *   2. servo position p: follows u at the servo speed r (units/s, per direction;
 *      worst case 2 / 12 s until TRAINING measured it);
 *   3. outboard a: follows p through a play (backlash) of width b: pushing right
 *      a = p - b/2, pushing left a = p + b/2 (b_lr / b_rl = the play measured
 *      reversing left -> right / right -> left; the model uses their mean);
 *   4. boat: tau dw/dt = K (V / V_ref) (a - a0) - w (yaw rate w, trim a0).
 * The AURIX runs it forward every step (p, a) and never commands faster than r
 * (so p stays true). The position is UNKNOWN after a power-on (owner decision
 * OD-7: no auto-centre, no frame until a mode commands steering): p is then the
 * interval [-1, 1] that closes at the worst-case speed once a command goes out;
 * every rule that needs the position waits for it (worst-case travel).
 *
 * Use while driving (sm_drive): the command rate never faster than r; backlash
 * compensation on every direction change u = target + dir x b/2, never more than
 * TRAINING's b.
 *
 * Slow adaptation (sm_adapt_step, RAM only - never saved, OD-8; only in steady
 * HDG LOCK / AUTO straight running, the caller decides): K and tau with a long
 * memory (>= 5 min) inside 0.7 .. 1.3 x TRAINING's, b may only SHRINK (down to
 * 0.5 x TRAINING's b), r is never adapted (never assumed faster). Doubt (model
 * residual > 3 x TRAINING's residual for 10 s, any bound hit, a sensor fault, a
 * node lost) -> back to the stored values at once (the notice STEERING MODEL
 * RESET, log event 30 STEERMODEL).
 */
#ifndef STEERMODEL_H
#define STEERMODEL_H

#include <stdbool.h>
#include <stdint.h>

#define SM_TRAVEL_S        12.0f                  /* worst-case steering servo lock to lock (TO CONFIRM C10) */
#define SM_RATE_WORST      (2.0f / SM_TRAVEL_S)   /* units / s */
#define SM_HALF_DEG        30.0f                  /* outboard angle at servo +1 (no full angle entered) */
#define SM_K_DEFAULT       4.0f                   /* conservative K: yaw deg/s per servo unit (before TRAINING) */
#define SM_TAU_DEFAULT     3.0f
#define SM_ADAPT_K_LO      0.7f                   /* K / tau bounds around TRAINING's */
#define SM_ADAPT_K_HI      1.3f
#define SM_ADAPT_B_LO      0.5f                   /* b may shrink down to this x TRAINING's b */
#define SM_ADAPT_MEMORY_S  300.0f                 /* K / tau adaptation memory (>= 5 min) */
#define SM_DOUBT_FACTOR    3.0f                   /* residual above this x TRAINING's residual ... */
#define SM_DOUBT_MS        10000u                 /* ... for this long = doubt */
#define SM_RES_FLOOR_DPS   0.05f                  /* TRAINING residual never taken below this */

/* why the adaptation went back to the stored values (TELEM2 group Q, log event STEERMODEL) */
#define SM_RESET_NONE      0u
#define SM_RESET_RESIDUAL  1u                     /* model residual too large for 10 s */
#define SM_RESET_LIMIT     2u                     /* an adapted value reached its bound */
#define SM_RESET_SENSOR    3u                     /* compass / gyro / GPS doubt */
#define SM_RESET_NODE      4u                     /* the steering node lost */
#define SM_RESET_TRAINING  5u                     /* new TRAINING values */

typedef struct {
    float r_right, r_left;      /* servo speed moving right / left, units / s */
    float b_lr, b_rl;           /* play reversing left -> right / right -> left, units */
    float k;                    /* yaw rate per unit of outboard at v_ref, deg/s */
    float tau;                  /* boat yaw response, s */
    float a0;                   /* trim: the outboard position of zero yaw rate, units */
    float v_ref;                /* TRAINING speed, kn (0 = none) */
    float res;                  /* TRAINING model residual, deg/s RMS */
    bool  trained;              /* from a TRAINING (else defaults: worst-case r, b = 0, a conservative K) */
} sm_params_t;

typedef struct {
    /* predictor: the model run forward with a slow offset */
    float w;                    /* predicted yaw rate */
    float c;                    /* offset (wind / current / trim), deg/s */
    float sk, st;               /* sensitivities dw/dK, dw/dtau */
    float skm, stm;             /* ... their slow means (removed: the offset takes the steady part) */
    float pkk, pek, ptt, pet;   /* excitation + correlation with the residual, ~30 s */
    float w2, a2;               /* the parallel model with a smaller play (b shrink test) */
    float j, j2;                /* squared-error sums of both over the window */
    uint32_t win_ms;
    uint16_t win_rev;           /* direction changes in the window */
    uint8_t  better;            /* windows in a row the smaller play fitted clearly better */
    float res2;                 /* low-passed squared residual (10 s) */
    uint32_t doubt_ms;
    bool  active;               /* updating this step */
    bool  primed;
    uint32_t steps;             /* adaptation steps so far */
} sm_adapt_t;

typedef struct {
    sm_params_t stored;         /* TRAINING (NV slot 5 v2) or the defaults */
    sm_params_t use;            /* what control uses: stored, or adapted inside the bounds */
    float half_deg;             /* outboard degrees at unit 1 */
    float max_u;                /* command limit, units (max_angle / half) */
    float u;                    /* last command (units) */
    float target;               /* the outboard target of the last sm_drive (units) */
    bool  have;                 /* commanded since the boot (OD-7) */
    float p_lo, p_hi;           /* the servo position: an interval while unknown */
    float a;                    /* outboard estimate, units (from the middle of p) */
    int8_t dir;                 /* direction of the last move (+1 right, -1 left, 0 none yet) */
    uint16_t reversals;         /* direction changes of the command (adaptation window) */
    sm_adapt_t ad;
    uint8_t  reset_why;         /* SM_RESET_ of the last reset (0 none) */
    uint16_t resets;
    bool     adapted;           /* use differs from stored */
} steer_t;

void  sm_defaults(sm_params_t *p);
/* known = the position is known (a warm reset: the last command p0 from reset-surviving RAM) */
void  sm_init(steer_t *s, const sm_params_t *stored, float half_deg, float max_deg, bool known, float p0);
void  sm_set_stored(steer_t *s, const sm_params_t *stored, float half_deg, float max_deg);
/* one step of driving: target outboard position (units); db = direction-change deadband (units) */
float sm_drive(steer_t *s, float target, float db, float dt);
/* TRAINING: the command set directly (its own rate limits; no compensation) */
void  sm_direct(steer_t *s, float u, float dt);
/* no new command (STANDBY = hold, OD-10): the model runs on */
void  sm_hold(steer_t *s, float dt);
bool  sm_known(const steer_t *s);
float sm_angle_deg(const steer_t *s);
/* the servo has reached the command and the outboard is within tol (units) of target, position known */
bool  sm_at(const steer_t *s, float target, float tol);
/* the play in use (the mean of b_lr / b_rl, never above the stored one) */
float sm_play(const steer_t *s);

typedef struct {
    float dt;
    float yaw_dps;              /* gyro, bias-corrected */
    float v_kn;                 /* speed (GPS) */
    bool  steady;               /* the caller's conditions hold (steady straight HDG LOCK / AUTO, no alarm,
                                   sensors healthy, GPS trusted, speed near v_ref) */
    bool  sensor_fault;         /* compass / gyro / GPS doubt now */
    bool  node_lost;            /* the steering node lost now */
} sm_adapt_in_t;
/* returns the SM_RESET_ reason when it went back to the stored values this step (else SM_RESET_NONE) */
uint8_t sm_adapt_step(steer_t *s, const sm_adapt_in_t *in);
/* back to the stored values now */
void  sm_adapt_reset(steer_t *s, uint8_t why);
/* the adapted bounds hold (storm I53) */
bool  sm_within_bounds(const steer_t *s);

#endif /* STEERMODEL_H */
