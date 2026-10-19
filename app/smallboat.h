/*
 * smallboat.h - Small-boat controller (outboard): modes, engine, steering.
 *
 * Owns ALL small-boat outputs (throttle, gear servo, steering motor,
 * alarm). Each 20 Hz step:
 *   read sensors -> steering (autopilot core) -> faults -> mode behaviour
 *   -> safe stop -> gear + throttle -> write outputs (throttle, gear first,
 *   steering last)
 *
 * Safety rules (CLAUDE.md):
 *   - gear changes only at idle (gear.c)
 *   - every fault / timeout / STANDBY: throttle idle + neutral FIRST, then
 *     steering off. If the compass still works and the autopilot was
 *     steering, it keeps holding the heading until idle + neutral is
 *     reached; without compass the steering motor simply stops.
 *   - throttle rate-limited, reverse power limited (throttle.c)
 *
 * Modes:
 *   STANDBY   engine idle + neutral, steering off
 *   MANEUVER  operator steers (full angle), F/N/R, throttle <= 30 %
 *   CRUISE    forward only, heading hold, set speed on GPS SOG, steering
 *             limit and gains by speed, slows down in rough water
 */
#ifndef SMALLBOAT_H
#define SMALLBOAT_H

#include <stdbool.h>
#include <stdint.h>

#include "autopilot.h"
#include "gear.h"
#include "hal.h"
#include "modes.h"
#include "throttle.h"

#define SB_LIMIT_N 4        /* points in the steering-limit-by-speed table */

typedef struct {
    ap_config_t       steer;            /* steering (outboard autopilot core) */
    gear_config_t     gear;
    throttle_config_t throttle;
    uint32_t          gps_timeout_ms;   /* no GPS fix this long = GPS lost (3000) */

    /* MANEUVER */
    float maneuver_throttle_pct;        /* lever full = this much throttle, F and R (30) */
    float steer_deadzone;               /* lever below this = no steering (0.3) */

    /* CRUISE: speed */
    float cruise_speed_kn;              /* set speed when CRUISE starts (8) */
    float cruise_max_kn;                /* highest set speed allowed (25) */
    float spd_kp;                       /* throttle % per kn of speed error (4) */
    float spd_ki;                       /* throttle % per (kn * s) (0.6) */
    float gps_loss_throttle_factor;     /* GPS lost: hold throttle * this (0.6) */

    /* CRUISE: steering by speed. Linear between the points, held at the
     * ends. Gains are scaled by gain_ref_kn / speed (sim boat turns
     * harder the faster it goes). */
    float limit_kn[SB_LIMIT_N];         /* {3, 8, 14, 20} */
    float limit_deg[SB_LIMIT_N];        /* {30, 20, 12, 8} */
    float gain_ref_kn;                  /* speed the steering gains were tuned at (6) */
    float gain_scale_min, gain_scale_max;   /* (0.3, 2.0) */

    /* CRUISE: nervousness (rough water) */
    float    nerv_on_dps;               /* yaw-rate wobble (std) above this ... (2.5) */
    float    nerv_off_dps;              /* ... calm again below this (1.6) */
    float    nerv_roll_on_dps;          /* same for roll rate, if a roll sensor exists */
    float    nerv_roll_off_dps;
    uint32_t nerv_on_ms;                /* must stay above this long to count (5 s) */
    uint32_t nerv_off_ms;               /* must stay below this long (30 s) */
    float    nerv_step_kn;              /* nervous: slow down by this ... (2) */
    uint32_t nerv_step_ms;              /* ... every this long (15 s) */
    float    nerv_recover_kn;           /* calm: speed up by this ... (1) */
    uint32_t nerv_recover_ms;           /* ... every this long (30 s) */
    uint32_t nerv_memory_ms;            /* after an episode, stay below that speed this long (300 s) */
    float    nerv_min_kn;               /* never slower than this (5) */
} sb_config_t;

typedef struct {
    hal_gps_t last;         /* last message */
    bool      valid;        /* any message yet */
    uint32_t  last_ms;      /* time of the last message with a fix */
} sb_gps_t;

typedef struct {
    float    set_kn;        /* user set speed */
    float    target_kn;     /* after rough-water reduction */
    float    integ;         /* speed PI integrator, throttle % */
    float    sog_kn;        /* last good SOG */
    bool     gps_lost;      /* running open loop without GPS */
    float    open_loop_pct; /* throttle held while GPS is lost */
} sb_cruise_t;

typedef struct {
    float    yaw_lp, yaw_var;       /* slow mean and wobble of the yaw rate */
    float    roll_lp, roll_var;
    bool     roll_ok;
    float    yaw_std, roll_std;     /* current wobble, deg/s */
    bool     nervous;
    uint32_t above_ms, below_ms;
    uint32_t step_timer_ms, recover_timer_ms;
    float    ceiling_kn;            /* don't recover above this ... */
    uint32_t ceiling_ms;            /* ... for this much longer */
    uint32_t episodes;              /* times it became nervous */
    uint32_t transitions;           /* nervous <-> calm changes */
} sb_nerv_t;

typedef struct {
    sb_config_t cfg;
    mode_mgr_t  modes;
    autopilot_t ap;
    gear_sm_t   gear;
    throttle_t  thr;
    sb_gps_t    gps;
    sb_cruise_t cruise;
    sb_nerv_t   nerv;

    uint32_t now_ms, last_ms;
    bool     stopping;          /* safe stop running (idle + neutral first) */
    bool     hold_while_stopping;
    bool     alarm;             /* small-boat alarms (the autopilot has its own) */
    bool     compass_fault_seen;

    float    throttle_demand;   /* this step, before limits */
    float    throttle_demand_prev;
    gear_t   gear_demand;

    hal_controls_t ctl;         /* operator controls (MANEUVER) */
    bool     ctl_ok;

    /* sim tests only: raw gear/throttle demand in the driving modes */
    struct { bool on; gear_t gear; float throttle_pct; } test_drive;
} sb_t;

void sb_default_config(sb_config_t *cfg);
void sb_init(sb_t *s, const sb_config_t *cfg);

/* One 20 Hz step: reads the HAL, writes the HAL. */
void sb_step(sb_t *s);

/* Ask the mode manager for a new mode. Returns true if accepted. */
bool sb_request_mode(sb_t *s, sb_mode_t to, mode_reason_t why);

static inline sb_mode_t sb_mode(const sb_t *s) { return s->modes.mode; }

bool sb_compass_ok(const sb_t *s);
bool sb_gps_ok(const sb_t *s);

/* CRUISE set speed (kn), clamped to 0 .. cruise_max_kn. */
void sb_set_cruise_speed(sb_t *s, float kn);

/* Steering angle limit for a speed (from the table). */
float sb_steer_limit_for(const sb_config_t *cfg, float sog_kn);

/* Sim tests only: drive with a raw gear / throttle demand. */
void sb_test_drive(sb_t *s, bool on, gear_t gear, float throttle_pct);

#endif /* SMALLBOAT_H */
