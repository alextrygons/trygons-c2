/*
 * thruster.h - The DP thrusters (CLAUDE.md section P, step 9): a small bow
 * TUNNEL thruster (sideways: + = push the bow to starboard) and a small fixed
 * transom electric motor (fore / aft: + = ahead), both proportional on the
 * 12 V bank. Pure C99; its only HAL call is hal_set_thrusters.
 *
 * thruster.c is the ONLY writer of the thrusters: thruster_step() sends
 * out_pct[] to hal_set_thrusters() once per control step, nothing else
 * calls it (tools/check.sh greps for the HAL call). Rules:
 *   - 0 whenever dp_active is false (DP is the only user: MODE_DP, step 10),
 *     and 0 at once (no ramp) on every stop: not allowed (FIRST TEST, e-stop,
 *     monitor trip, outputs not armed), not fitted, overheated;
 *   - clamped to the boat file's max_pct per thruster (PROTO_T_THRUSTER_CFG,
 *     RAM only; the AURIX's own ceiling THR_CEILING_PCT 100); no settings
 *     received = max_pct 0 = no thruster fitted = always 0 (fail safe);
 *   - rate limited (THR_RATE_PCT_S) on every change while running (a
 *     reversal passes through 0 at that rate);
 *   - heat budget per thruster (run_s / rest_s): running (|out| > THR_RUN_PCT)
 *     fills the budget 1 s per s, standing still empties it run_s / rest_s s
 *     per s; full (run_s of running) = that thruster HOT: 0 until the budget
 *     is empty again (rest_s of rest) - the caller raises SB_ALM2_DP_HEAT.
 * CPU1 knows dp_active, the outputs and the caps (monitor.c): thrust without
 * dp_active or above the cap trips it, and its safe state sets both to 0.
 * FIRST TEST and the big boat: no thrusters (never active).
 */
#ifndef THRUSTER_H
#define THRUSTER_H

#include <stdbool.h>
#include <stdint.h>

#define THR_BOW          0u
#define THR_STERN        1u
#define THR_N            2u
#define THR_CEILING_PCT  100.0f     /* the AURIX never lets a thruster above this, whatever the boat file says */
#define THR_RATE_PCT_S   50.0f      /* thrust change while running: 0 -> 100 % in 2 s */
#define THR_RUN_PCT      2.0f       /* |output| above this counts as running (heat budget) */
#define THR_MAX_S        3600u      /* run_s / rest_s limit of the settings frame */

/* why the thrusters are held at 0 (thruster_t.why, telemetry / log) */
#define THR_WHY_NONE     0u     /* running (dp_active, allowed) */
#define THR_WHY_IDLE     1u     /* dp_active false: no DP */
#define THR_WHY_BLOCKED  2u     /* not allowed: FIRST TEST / e-stop / monitor trip / not armed */

typedef struct {
    float    max_pct;           /* 0 = not fitted (default: no settings received) */
    uint16_t run_s;             /* heat budget: continuous running allowed */
    uint16_t rest_s;            /* ... rest needed after a full budget */
} thr_unit_cfg_t;

typedef struct {
    thr_unit_cfg_t unit[THR_N];
    float    jog_m;             /* DP JOG step (step 10; 1.5 m) */
    float    spot_alarm_m;      /* DP off-spot alarm (step 10; 15 m) */
} thr_cfg_t;

typedef struct {
    bool     dp_active;         /* DP wants thrust (step 10: MODE_DP) */
    bool     allowed;           /* not FIRST TEST, e-stop released, no monitor trip, outputs armed */
    float    demand_pct[THR_N]; /* wanted -100..100 */
    uint32_t dt_ms;
} thr_input_t;

typedef struct {
    thr_cfg_t cfg;
    bool     have_cfg;          /* a settings frame was taken since the power-up */
    uint16_t cfg_hash;          /* CRC-16 of the settings frame (the station compares its boat file) */
    uint32_t cfg_count;         /* settings frames taken */
    float    out_pct[THR_N];    /* what goes to hal_set_thrusters (-max..max) */
    float    heat_s[THR_N];     /* heat budget used, s (0 .. run_s) */
    bool     hot[THR_N];        /* budget full: held at 0 until it is empty again */
    bool     active;            /* dp_active and allowed this step (published to CPU1 as dp_active) */
    uint8_t  why;               /* THR_WHY_ */
    bool     hot_edge, cool_edge;   /* a thruster got HOT / cooled down this step */
    uint32_t run_ms;            /* time with any thruster running (statistics) */
    uint32_t log_seq;           /* +1 per event for the log (settings, hot, cool) */
    uint8_t  log_what;          /* 1 settings, 2 hot, 3 cooled */
    uint8_t  log_unit;          /* THR_BOW / THR_STERN of a hot / cool event */
} thruster_t;

void thruster_init(thruster_t *t);

/* The settings frame body (PROTO_T_THRUSTER_CFG, THR_FRAME_BYTES): per
 * thruster (bow, stern) u8 max_pct, u16 run_s, u16 rest_s; u8 jog_m*10,
 * u8 spot_alarm_m. decode: false = bad values (max_pct > 100, run_s 0 with
 * max_pct > 0, times > THR_MAX_S, jog 0.5..10 m, spot 5..100 m). */
#define THR_FRAME_BYTES 12u
bool thruster_cfg_decode(const uint8_t *p, uint32_t len, thr_cfg_t *c);
/* Take the settings (the caller checks HELM + STANDBY). The heat budget is kept. */
void thruster_cfg_set(thruster_t *t, const thr_cfg_t *c, uint16_t hash);

/* One control step: out_pct[] for hal_set_thrusters. */
void thruster_step(thruster_t *t, const thr_input_t *in);

/* fitted = max_pct > 0 and run_s > 0 */
bool thruster_fitted(const thruster_t *t, uint8_t u);
/* the cap CPU1 checks (max_pct clamped to the ceiling; 0 = not fitted) */
float thruster_cap(const thruster_t *t, uint8_t u);
/* heat budget used, 0..1 */
float thruster_heat(const thruster_t *t, uint8_t u);
bool thruster_any_hot(const thruster_t *t);
/* |bow| + |stern| output (the generator's load-change check) */
float thruster_load_pct(const thruster_t *t);
/* TELEM2 group N flags (b0 settings received, b1 active, b2 bow HOT, b3 stern HOT, b4 bow fitted,
 * b5 stern fitted) */
uint8_t thruster_flags(const thruster_t *t);

#endif /* THRUSTER_H */
