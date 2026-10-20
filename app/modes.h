/*
 * modes.h - Small-boat mode manager.
 *
 * The ONLY place where the mode changes. Every request is checked against
 * an explicit allowed-transition table (and preconditions given by the
 * caller), and every request - accepted or rejected - is logged.
 *
 *   Driving modes: MANUAL (joystick), HDG_LOCK (heading lock, throttle on
 *   the joystick), AUTO (waypoint program). Plus STANDBY (safe state) and
 *   TRAINING (setup).
 *
 *   from \ to   STANDBY MANUAL HDG_LOCK AUTO TRAINING
 *   STANDBY        -      yes     yes    yes    yes
 *   MANUAL        yes      -      yes    yes    no
 *   HDG_LOCK      yes     yes      -     yes    no
 *   AUTO          yes     yes     yes     -     no
 *   TRAINING      yes     yes     no     no      -
 *
 * smallboat.c adds the rules on top (preconditions):
 *   - only the joystick enters MANUAL (it always wins);
 *   - MANUAL is left only with the stick centred (AUTO / HDG LOCK button,
 *     PC + CONFIRM), by the STANDBY button, or by a fault;
 *   - AUTO starts only with a PC command + joystick CONFIRM;
 *   - TRAINING only from STANDBY by the PC.
 */
#ifndef MODES_H
#define MODES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MODE_STANDBY = 0,       /* idle, neutral, steering off */
    MODE_MANUAL,            /* joystick: X steers, Y throttle */
    MODE_HDG_LOCK,          /* autopilot holds the heading, throttle on joystick Y */
    MODE_AUTO,              /* waypoint program */
    MODE_TRAINING,          /* outboard calibration */
    MODE_COUNT
} sb_mode_t;

typedef enum {
    WHY_USER = 0,           /* user request */
    WHY_STANDBY_BUTTON,     /* Standby/Auto button */
    WHY_COMPASS_FAULT,
    WHY_GPS_LOST,
    WHY_ARRIVED,
    WHY_TRAINING_DONE,
    WHY_TRAINING_ABORT,
    WHY_TEST,               /* sim test setup */
    WHY_GEAR_FAULT,         /* neutral switch did not confirm */
    WHY_DR_LIMIT,           /* dead reckoning too uncertain / too long */
    WHY_ARRIVAL_UNCERTAIN,  /* at the waypoint by DR, but not sure enough */
    WHY_JOYSTICK,           /* joystick deflected / TAKE pressed */
    WHY_JOY_HANDBACK,       /* AUTO / HDG LOCK button with the stick centred */
    WHY_JOY_FAULT,          /* joystick wire break / out of range */
    WHY_PC,                 /* ground station command */
    WHY_HDG_BUTTON,         /* HDG LOCK button */
    WHY_CONFIRM,            /* PC request + joystick CONFIRM */
    WHY_NOT_ARMED,          /* outputs not armed (neutral-start interlock) */
    WHY_MONITOR,            /* independent safety monitor tripped */
    WHY_COUNT
} mode_reason_t;

typedef enum {
    MODE_OK = 0,            /* accepted */
    MODE_REJECT_TABLE,      /* not in the allowed-transition table */
    MODE_REJECT_PRECOND     /* allowed, but conditions not met (sensor, GPS ...) */
} mode_verdict_t;

#define MODE_LOG_N 32u

typedef struct {
    uint32_t       t_ms;
    sb_mode_t      from, to;
    mode_reason_t  why;
    mode_verdict_t verdict;
} mode_log_entry_t;

typedef struct {
    sb_mode_t        mode;
    uint32_t         since_ms;          /* time of the last change */
    mode_log_entry_t log[MODE_LOG_N];   /* ring buffer */
    uint32_t         log_count;         /* total entries ever written */
} mode_mgr_t;

void mode_init(mode_mgr_t *m, uint32_t now_ms);

/* Is from -> to in the table? */
bool mode_allowed(sb_mode_t from, sb_mode_t to);

/* Request a change. precond_ok = the caller's checks for the new mode
 * (sensors, GPS fix ...). Logs the request. Returns the verdict; the mode
 * changes only on MODE_OK. */
mode_verdict_t mode_request(mode_mgr_t *m, sb_mode_t to, mode_reason_t why,
                            bool precond_ok, uint32_t now_ms);

/* Log entry i (0 = oldest still kept). NULL if out of range. */
const mode_log_entry_t *mode_log_get(const mode_mgr_t *m, uint32_t i);
uint32_t mode_log_size(const mode_mgr_t *m);

const char *mode_name(sb_mode_t m);
const char *mode_reason_name(mode_reason_t r);

#endif /* MODES_H */
