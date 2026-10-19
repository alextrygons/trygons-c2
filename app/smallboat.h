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

typedef struct {
    ap_config_t       steer;            /* steering (outboard autopilot core) */
    gear_config_t     gear;
    throttle_config_t throttle;
    uint32_t          gps_timeout_ms;   /* no GPS fix this long = GPS lost (3000) */

    /* MANEUVER */
    float             maneuver_throttle_pct; /* lever full = this much throttle, F and R (30) */
    float             steer_deadzone;        /* lever below this = no steering (0.3) */
} sb_config_t;

typedef struct {
    hal_gps_t last;         /* last message */
    bool      valid;        /* any message yet */
    uint32_t  last_ms;      /* time of the last message with a fix */
} sb_gps_t;

typedef struct {
    sb_config_t cfg;
    mode_mgr_t  modes;
    autopilot_t ap;
    gear_sm_t   gear;
    throttle_t  thr;
    sb_gps_t    gps;

    uint32_t now_ms, last_ms;
    bool     stopping;          /* safe stop running (idle + neutral first) */
    bool     hold_while_stopping;
    bool     alarm;             /* small-boat alarms (the autopilot has its own) */
    bool     compass_fault_seen;

    float    throttle_demand;   /* this step, before limits */
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

/* Sim tests only: drive with a raw gear / throttle demand. */
void sb_test_drive(sb_t *s, bool on, gear_t gear, float throttle_pct);

#endif /* SMALLBOAT_H */
