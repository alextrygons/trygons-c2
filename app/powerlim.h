/*
 * powerlim.h - Throttle power limit (forward / reverse), RAM only.
 *
 *   - Every power-up / reset: 30 % forward, 5 % reverse (of the calibrated
 *     full servo travel). The raised limit is never saved.
 *   - Raising, one step at a time (30 -> 50 -> 75 -> 100 forward; reverse in
 *     steps up to rev_max, default 15 %), needs ALL of:
 *       1. the PC asks for a code: the boat makes a random 4-digit code
 *          (shown on the ground station),
 *       2. the user types it back (PC sends it, valid code_ms),
 *       3. the joystick CONFIRM button within 10 s (smallboat.c, same rules
 *          as every CONFIRM: stick centred, joystick armed, not faulted).
 *     A wrong code invalidates it (ask for a new one).
 *   - Lowering is always instant (PC, TAKE / override, STANDBY button, any
 *     fault / alarm, PC link lost, GPS lost, compass / joystick fault,
 *     entering MANUAL or TRAINING, a long time at low speed).
 *   - Every change and every raise attempt is logged (plim_log_get).
 *
 * The limit is enforced in the lowest layer: servo.c clamps every throttle
 * output with it.
 */
#ifndef POWERLIM_H
#define POWERLIM_H

#include <stdbool.h>
#include <stdint.h>

#define PLIM_FWD_STEPS  4
#define PLIM_LOG_N      32u

typedef struct {
    float    fwd_steps[PLIM_FWD_STEPS]; /* {30, 50, 75, 100} - [0] = startup */
    float    rev_start;                 /* 5 (startup) */
    float    rev_step;                  /* 5 */
    float    rev_max;                   /* 15 */
    uint32_t code_ms;                   /* a code is valid this long (60 s) */
    float    low_speed_kn;              /* below this ... (3) */
    uint32_t low_speed_ms;              /* ... this long -> back to startup (120 s) */
} plim_config_t;

typedef enum {
    PLIM_SRC_STARTUP = 0, PLIM_SRC_PC, PLIM_SRC_CONFIRM, PLIM_SRC_OVERRIDE, PLIM_SRC_STANDBY_BUTTON,
    PLIM_SRC_ALARM, PLIM_SRC_PC_LINK, PLIM_SRC_GPS_LOST, PLIM_SRC_COMPASS, PLIM_SRC_JOYSTICK,
    PLIM_SRC_TRAINING, PLIM_SRC_LOW_SPEED, PLIM_SRC_FAULT,
    PLIM_SRC_COUNT
} plim_source_t;

typedef enum {
    PLIM_OK = 0,
    PLIM_ERR_WRONG_CODE,        /* code wrong / expired / never asked */
    PLIM_ERR_AT_MAX,            /* already at the top step */
    PLIM_ERR_JOY_NOT_READY,     /* no CONFIRM possible */
    PLIM_ERR_ALARM,             /* acknowledge the alarm first */
    PLIM_ERR_LATE,              /* CONFIRM too late */
    PLIM_ERR_NOT_CENTRED        /* CONFIRM with the stick off centre / not armed */
} plim_result_t;

typedef struct {
    uint32_t t_ms;
    float    old_fwd, old_rev, new_fwd, new_rev;
    uint8_t  source;            /* plim_source_t */
    uint8_t  dir;               /* 0 forward, 1 reverse (raise attempts) */
    uint8_t  result;            /* plim_result_t, 0 = accepted / done */
    uint8_t  attempt;           /* 1 = raise attempt, 0 = change */
} plim_log_t;

typedef struct {
    plim_config_t cfg;
    float    fwd_pct, rev_pct;  /* current limits, % of full travel */
    uint8_t  fwd_step;
    uint16_t code;              /* pending code (0 = none) */
    uint8_t  code_dir;
    uint32_t code_until_ms;
    uint32_t rng;
    uint32_t low_timer_ms;
    plim_log_t log[PLIM_LOG_N];
    uint32_t log_count;
    uint32_t changes;           /* info */
} plim_t;

void plim_default_config(plim_config_t *c);
void plim_init(plim_t *p, const plim_config_t *c, uint32_t seed);

/* 1. PC: new 4-digit code for a raise in `dir` (0 forward, 1 reverse). */
uint16_t plim_code_request(plim_t *p, uint8_t dir, uint32_t now_ms);
/* 2. PC: the typed code. PLIM_OK = ready for CONFIRM (caller arms it). The
 *    code is used up either way. */
plim_result_t plim_code_check(plim_t *p, uint8_t dir, uint16_t code, uint32_t now_ms);
/* 3. CONFIRM accepted: one step up. */
plim_result_t plim_step_up(plim_t *p, uint8_t dir, uint32_t now_ms);
/* Record a refused attempt (late / not centred / ...). */
void plim_log_attempt(plim_t *p, uint8_t dir, plim_result_t r, uint32_t now_ms);

/* Back to the startup limits (instant). Logged when it changes something. */
void plim_drop(plim_t *p, plim_source_t src, uint32_t now_ms);

/* Every step: low-speed auto drop. */
void plim_step(plim_t *p, float speed_kn, uint32_t dt_ms, uint32_t now_ms);

bool plim_raised(const plim_t *p);
float plim_code_left_s(const plim_t *p, uint32_t now_ms);
const plim_log_t *plim_log_get(const plim_t *p, uint32_t i);     /* 0 = oldest kept */
uint32_t plim_log_size(const plim_t *p);
const char *plim_source_name(plim_source_t s);
const char *plim_result_name(plim_result_t r);

#endif /* POWERLIM_H */
