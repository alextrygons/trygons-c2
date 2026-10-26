/*
 * generator.h - Generator mode: the outboard charges the 12 V bank at a
 * fast idle in NEUTRAL (CLAUDE.md section K). Pure C99, no HAL.
 *
 * This is the ONE exception to "nothing raises the throttle by itself":
 * the generator may open the throttle, but only
 *   - while "generator auto" is switched on by the captain (default off
 *     after every power-up; the STANDBY button switches it off again),
 *   - in STANDBY (not stopping) [MODE_DP later], outputs armed, gear N
 *     CONFIRMED by a fitted neutral switch, no engine kill, no e-stop, no
 *     monitor trip, not FIRST TEST, the bank voltage known and below
 *     start_v,
 *   - in small steps (idle_pct, then +step_pct every step_ms while the
 *     fast-filtered voltage still falls; hold when it stops falling;
 *     -step_pct when it rises above start_v + rise_v),
 *   - never above cap = min(max_pct, GEN_CEILING_PCT, the power limit).
 * The demand goes through the normal throttle path (rate limits, the
 * servo.c clamp); CPU1 knows gen_active + the cap and trips on anything
 * else. Every stop is idle FIRST: the demand drops to 0 and the state is
 * STOPPING until the throttle output is idle; nothing raises meanwhile.
 * A neutral switch that opens, or a speed rise after a throttle rise, is
 * a GEAR FAULT (the caller raises SB_ALM_GEAR): idle at once, generator
 * auto OFF until re-enabled. Cycle: run_ms, then rest_ms at idle and a
 * rest-voltage check: still low -> again, else DONE (notice).
 * The learned value (the throttle where the voltage stopped falling) is
 * the first step next time (NV slot 6, saved by the caller in STANDBY).
 */
#ifndef GENERATOR_H
#define GENERATOR_H

#include <stdbool.h>
#include <stdint.h>

#define GEN_CEILING_PCT   30.0f     /* the AURIX never lets the generator above this (TO CONFIRM) */

/* gen_t.state (TELEM2 group L "generator state") */
#define GEN_OFF       0u    /* generator auto off (captain / STANDBY button / power-up) */
#define GEN_READY     1u    /* auto on: waiting for the conditions (STANDBY, neutral, low bank ...) */
#define GEN_STARTING  2u    /* conditions met: beep + notice, start_delay_ms before the fast idle */
#define GEN_RUNNING   3u    /* throttle open in neutral (demand_pct) */
#define GEN_STOPPING  4u    /* demand 0, waiting for the throttle output to reach idle */
#define GEN_REST      5u    /* cycle: rest_ms at idle, then the rest-voltage check */
#define GEN_FAULT     6u    /* GEAR FAULT (switch opened / boat moved): auto off until re-enabled */
#define GEN_DONE      7u    /* the bank recovered at rest: waiting (auto still on) */

/* why the last stop started (gen_t.stop_why, log event GENERATOR) */
#define GEN_STOP_NONE       0u
#define GEN_STOP_CONDITIONS 1u  /* a run condition went away (mode / armed / neutral / kill / e-stop / trip) */
#define GEN_STOP_OFF        2u  /* switched off (captain, STANDBY button) */
#define GEN_STOP_REQUEST    3u  /* a stop request: joystick, alarm, mode / gear request, link lost */
#define GEN_STOP_NO_VOLTS   4u  /* the voltage data went away */
#define GEN_STOP_REST       5u  /* cycle: rest */
#define GEN_STOP_SWITCH     6u  /* FAULT: the neutral switch opened */
#define GEN_STOP_MOVED      7u  /* FAULT: SOG rose after a throttle rise */

typedef struct {
    float    max_pct;           /* parameter gen_max_pct (boat file generator_max_pct): 0 = disabled (25, TO CONFIRM) */
    float    start_v;           /* runs while the bank is below this (12.1 V) */
    float    rise_v;            /* step down when the fast voltage is above start_v + this (0.4 V) */
    float    idle_pct;          /* the fast idle: first step (8 %, TO CONFIRM) */
    float    step_pct;          /* one step (2 %) */
    uint32_t step_ms;           /* between two judgements (30 s) */
    uint32_t start_delay_ms;    /* beep + notice, then the fast idle (10 s) */
    float    fall_v;            /* the fast voltage fell by more than this over step_ms = "still falling" (0.02 V) */
    uint32_t run_ms;            /* one run (20 min) */
    uint32_t rest_ms;           /* then idle this long before the rest-voltage check (1 min) */
    float    move_kn;           /* SOG rise above this within move_ms of a throttle rise = GEAR FAULT (0.5 kn) */
    uint32_t move_ms;           /* (20 s) */
    float    thrust_pct;        /* thrust demand change above this within thrust_ms = judgement paused (10 %) */
    uint32_t thrust_ms;         /* (10 s) */
} gen_config_t;

/* What the caller knows this step. */
typedef struct {
    bool     allowed;           /* STANDBY (not stopping) / DP, armed, not FIRST TEST, no kill / e-stop / trip */
    bool     neutral_confirmed; /* neutral switch fitted + closed AND the gear state machine settled in N */
    bool     volts_valid;       /* bank_v / bank_v_fast mean something */
    float    bank_v;            /* slow filter (start / rest checks) */
    float    bank_v_fast;       /* fast filter (the step judgement) */
    float    limit_pct;         /* the forward power limit now */
    bool     stop_request;      /* joystick deflection, a non-advisory alarm, a mode / gear request, link lost ... */
    bool     gps_trusted;
    float    sog_kn;
    float    thrust_pct;        /* |thruster demand| (DP, later): 0 without thrusters */
    float    throttle_out_pct;  /* what the servo got last step */
    float    idle_max_pct;      /* "idle" for the stop */
    uint32_t dt_ms;
} gen_input_t;

typedef struct {
    gen_config_t cfg;
    bool     enabled;           /* generator auto (captain) */
    uint8_t  state;             /* GEN_ */
    uint8_t  stop_why;          /* GEN_STOP_ of the current / last stop */
    float    demand_pct;        /* the throttle demand (gear N), 0 unless RUNNING */
    float    cap_pct;           /* min(max_pct, ceiling, power limit) this step */
    bool     active;            /* RUNNING or STOPPING: the throttle may be above idle in neutral (CPU1) */
    bool     fast_stop;         /* a FAULT stop: the STANDBY closing rate instead of the normal one */
    float    learned_pct;       /* where the voltage stopped falling (0 = unknown) - NV slot 6 */
    bool     learned_new;       /* learned_pct changed since the last save (the caller saves and clears it) */
    bool     start_edge;        /* this step: STARTING began (beep + notice) */
    bool     fault_edge;        /* this step: a GEAR FAULT (the caller raises SB_ALM_GEAR) */
    bool     done_edge;         /* this step: DONE (notice) */
    uint32_t timer_ms;          /* state timer */
    uint32_t judge_ms;          /* since the last judgement */
    float    v_ref;             /* fast voltage at the last judgement */
    uint32_t run_ms;            /* this run so far */
    uint32_t move_ms;           /* since the last throttle rise (0 = none pending) */
    float    sog_ref;           /* SOG at that rise */
    uint32_t thrust_ms;         /* judgement paused this long still */
    float    thrust_ref;
    bool     stopped_falling;   /* the voltage stopped falling at least once this run (learned value valid) */
    uint32_t starts, steps_up, steps_down, holds, stops, faults, cycles;
} gen_t;

void gen_default_config(gen_config_t *c);
void gen_init(gen_t *g, const gen_config_t *c, float learned_pct);
/* The captain's switch (also the STANDBY button: off). Switching off while
 * running = a stop (idle first). */
void gen_set(gen_t *g, bool on);
/* One control step. */
void gen_step(gen_t *g, const gen_input_t *in);
const char *gen_state_name(uint8_t state);

#endif /* GENERATOR_H */
