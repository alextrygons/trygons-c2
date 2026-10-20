/*
 * smallboat.h - Small-boat controller (outboard): modes, engine, steering.
 *
 * Owns ALL small-boat outputs (throttle, gear servo, steering motor,
 * alarm). Each 20 Hz step:
 *   read sensors -> steering (autopilot core) -> faults, buttons, joystick
 *   -> mode behaviour -> safe stop -> gear + throttle -> write outputs
 *   (throttle, gear first, steering last)
 *
 * Driving modes (exactly three) plus STANDBY and TRAINING:
 *   STANDBY   engine idle + neutral, steering off. Power-up and after faults.
 *   MANUAL    joystick: X = outboard angle (full range), Y = throttle
 *             ahead / astern through the gear state machine.
 *             Entered from ANY mode by X deflection (> 100 ms), by Y
 *             deflection when not in HDG_LOCK, or by TAKE: it always wins.
 *             Left only with the stick centred: joystick AUTO (back to
 *             HDG_LOCK if that was interrupted, else STANDBY), the HDG LOCK
 *             button, PC + CONFIRM (resume AUTO) - or the STANDBY button
 *             (always works; a deflected stick is then disarmed until it
 *             has been centred again).
 *   HDG_LOCK  the HDG LOCK button locks the current heading; -1/+1/-10/+10
 *             (buttons or PC) adjust it. Throttle stays on joystick Y
 *             (ahead / astern, gear state machine, all limits) without
 *             leaving HDG_LOCK. Joystick X -> MANUAL. Sea-state filter and
 *             speed-dependent steering limit as in AUTO. In reverse gear
 *             the outboard is held centred (heading hold only works ahead).
 *             Compass lost: gyro hold + alarm, throttle stays with the
 *             user; once idle + neutral: steering off, STANDBY.
 *   AUTO      waypoint program (program.h): line following, speed control
 *             with rough-water slow-down, dead reckoning without GPS,
 *             GPS integrity check. Starts only with a PC command + joystick
 *             CONFIRM within 10 s. Pause / resume / skip from the PC.
 *             Joystick -> MANUAL at once, program suspended (resume = PC
 *             + CONFIRM).
 *   TRAINING  outboard calibration (training.c), started from STANDBY by
 *             the PC; fixed slow throttle ahead; result accepted/rejected.
 *
 * Safety rules (CLAUDE.md):
 *   - gear changes only at idle (gear.c)
 *   - every fault / timeout / STANDBY: throttle idle + neutral FIRST, then
 *     steering off (compass lost while moving: gyro hold, or outboard to
 *     centre, until idle + neutral)
 *   - throttle rate-limited, reverse power limited (throttle.c)
 *   - power limit (powerlim.c): 30 % forward / 5 % reverse after every
 *     reset; raised one step at a time by PC + typed code + joystick
 *     CONFIRM; dropped at once on any fault / alarm / override / STANDBY.
 *     Enforced in the lowest layer (servo.c writes the throttle servo).
 *   - the joystick always wins over the autopilot and the PC; the STANDBY
 *     button always wins over the PC
 *   - neutral-start interlock: after power-up the outputs only arm when the
 *     throttle is idle, neutral is confirmed (neutral switch if fitted,
 *     otherwise neutral commanded for arm_ms) and the joystick is centred;
 *     until then STANDBY only, safety relay open, alarm after 3 s
 *   - publishes its outputs + heartbeat for the independent monitor
 *     (monitor.c, CPU1); a monitor trip -> STANDBY + alarm, locked
 */
#ifndef SMALLBOAT_H
#define SMALLBOAT_H

#include <stdbool.h>
#include <stdint.h>

#include "autopilot.h"
#include "gear.h"
#include "joystick.h"
#include "hal.h"
#include "dr.h"
#include "modes.h"
#include "nav.h"
#include "powerlim.h"
#include "program.h"
#include "servo.h"
#include "throttle.h"

#define SB_LIMIT_N 4        /* points in the steering-limit-by-speed table */

/* What an alarm is about (sb_t.alarm_bits, status page). */
#define SB_ALM_COMPASS            0x01u
#define SB_ALM_GPS_LOST           0x02u
#define SB_ALM_GPS_SUSPECT        0x04u   /* fix rejected by the integrity check */
#define SB_ALM_DR_LIMIT           0x08u   /* DR too uncertain / too long -> AUTO holds, slow */
#define SB_ALM_ARRIVAL_UNCERTAIN  0x10u
#define SB_ALM_ARRIVED            0x20u
#define SB_ALM_GEAR               0x40u
#define SB_ALM_TRAINING           0x80u
#define SB_ALM_JOY_FAULT          0x100u  /* joystick wire break / out of range */
#define SB_ALM_JOY_NOT_CENTERED   0x200u  /* joystick not centred at power-up */
#define SB_ALM_JOYSTICK           0x400u  /* joystick took control from an automatic mode */
#define SB_ALM_PC_LINK            0x800u  /* ground station link lost */
#define SB_ALM_NOT_ARMED          0x1000u /* outputs not armed at start: gear not neutral / stick not centred */
#define SB_ALM_MONITOR            0x2000u /* independent safety monitor (CPU1) tripped */

#define SB_FAULT_STEER_OFF   0u
#define SB_FAULT_STEER_GYRO  1u

/* How the steering behaves during a safe stop (STANDBY), or in HDG_LOCK
 * after a compass failure. */
typedef enum {
    STOP_STEER_NONE = 0,    /* off */
    STOP_STEER_HEADING,     /* compass OK: hold the heading */
    STOP_STEER_GYRO,        /* compass lost: zero yaw rate with the gyro */
    STOP_STEER_CENTER       /* compass and gyro lost: outboard to centre */
} sb_stop_steer_t;

typedef struct {
    ap_config_t       steer;            /* steering (outboard autopilot core) */
    gear_config_t     gear;
    throttle_config_t throttle;
    plim_config_t     plim;             /* power limit */
    servo_config_t    servo;            /* throttle servo calibration */
    uint32_t          gps_timeout_ms;   /* no GPS fix this long = GPS lost (3000) */

    /* Compass lost while moving: how to steer during the safe stop.
     * SB_FAULT_STEER_GYRO (default): gyro hold (zero yaw rate) until idle +
     * neutral, then off; no gyro -> outboard to centre, then off.
     * SB_FAULT_STEER_OFF: steering stops at once (old behaviour). */
    uint8_t           fault_steer;

    /* AUTO: speed */
    float cruise_max_kn;                /* highest leg speed allowed (25) */
    float spd_kp;                       /* throttle % per kn of speed error (4) */
    float spd_ki;                       /* throttle % per (kn * s) (0.6) */
    float gps_loss_throttle_factor;     /* no position at all: hold throttle * this (0.6) */

    /* Steering by speed (HDG_LOCK and AUTO). Linear between the points,
     * held at the ends. Gains are scaled by gain_ref_kn / speed (the boat
     * turns harder the faster it goes). */
    float limit_kn[SB_LIMIT_N];         /* {3, 8, 14, 20} */
    float limit_deg[SB_LIMIT_N];        /* {30, 20, 12, 8} */
    float gain_ref_kn;                  /* speed the steering gains were tuned at (6) */
    float gain_scale_min, gain_scale_max;   /* (0.3, 2.0) */

    /* AUTO: nervousness (rough water) */
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

    /* AUTO: navigation */
    float    slow_radius_m;             /* SLOW / STOP / last waypoint: start slowing down here (150) */
    float    approach_kn;               /* speed at the arrival radius (4) */
    float    lookahead_min_m;           /* aim this far ahead on the line, at least (40) */
    float    lookahead_s;               /* ... or this many seconds of travel (6) */
    float    crab_tau_s;                /* drift (current) estimate filter (20) */
    prog_limits_t prog;                 /* program validation limits */
    uint32_t confirm_ms;                /* PC request -> joystick CONFIRM window (10 s) */
    uint32_t arm_ms;                    /* neutral-start: conditions OK this long to arm (500 ms) */
    uint32_t arm_alarm_ms;              /* not armed this long -> alarm (3 s) */

    /* TRAINING */
    float    training_throttle_pct;     /* fixed slow throttle, forward gear (20) */
    uint32_t training_spinup_ms;        /* at that throttle this long before calibrating (30 s) */

    /* GPS-denied navigation (dead reckoning, dr.c) */
    dr_config_t dr;
    float    dr_speed_factor;           /* AUTO in DR: speed x this (0.7) */
    float    dr_max_unc_m;              /* uncertainty above this ... (200) */
    uint32_t dr_max_ms;                 /* ... or DR longer than this (10 min) -> hold, slow (600000) */
    float    dr_slow_kn;                /* speed after that (4) */

    /* Joystick (joystick.c) */
    joy_config_t joy;
    float    joy_max_throttle_pct;      /* stick full = this much throttle demand, F and R (100;
                                           the power limit and the reverse limit still apply) */
} sb_config_t;

typedef struct {
    hal_gps_t last;         /* last message */
    bool      valid;        /* any message yet */
    uint32_t  last_ms;      /* time of the last message with a fix */
} sb_gps_t;

typedef struct {
    float    set_kn;        /* wanted speed (leg speed, slow-downs) */
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

/* AUTO program state */
typedef enum {
    AUTO_RUN = 0,           /* on a leg */
    AUTO_PAUSED,            /* PC pause: idle + neutral, heading held */
    AUTO_WP_STOP,           /* stopped at a STOP waypoint: idle + neutral */
    AUTO_HOLD               /* fallback (DR limit / uncertain arrival / no position / end HOLD):
                               hold the heading at a slow speed, alarm */
} auto_state_t;

typedef struct {
    program_t  prog;            /* stored program */
    bool       prog_ok;         /* valid program loaded */
    uint8_t    wp;              /* active waypoint (next to reach) */
    bool       suspended;       /* interrupted (override / STANDBY): resume = PC + CONFIRM */
    bool       finished;        /* last waypoint reached */
    auto_state_t state;
    float      hold_kn;         /* AUTO_HOLD speed */
    nav_leg_t  leg;
    float      crab_deg;        /* heading - course over ground (current / wind) */
    float      course_deg;      /* course over ground wanted */
    uint32_t   last_fix_ms;
    bool       in_dr;           /* navigating on dead reckoning */
    uint32_t   next_dr_ms;      /* 1 Hz updates in DR */
    bool       arrival_uncertain;
    uint32_t   legs_done;       /* waypoints reached (info) */
} sb_auto_t;

/* Pending PC request waiting for the joystick CONFIRM button */
typedef enum {
    CONF_NONE = 0,
    CONF_AUTO_START,        /* start the program at waypoint 1 */
    CONF_AUTO_RESUME,       /* resume at the active waypoint */
    CONF_RAISE_FWD,         /* raise the forward power limit one step */
    CONF_RAISE_REV          /* raise the reverse power limit one step */
} confirm_kind_t;

typedef struct {
    confirm_kind_t kind;
    uint32_t       deadline_ms;
    uint32_t       accepted, refused, ignored;     /* CONFIRM presses (info / tests) */
} sb_confirm_t;

typedef enum {
    TS_SPINUP = 0,          /* forward gear, slow throttle, settle */
    TS_RUN,                 /* calibration running (training.c) */
    TS_RESULT               /* finished: idle + neutral, waiting for accept / reject */
} sb_train_stage_t;

typedef struct {
    sb_train_stage_t stage;
    uint32_t timer_ms;
    double   sog_sum;           /* average speed during the run */
    uint32_t sog_n;
} sb_train_t;

/* Result of a PC request (link.c turns it into an ACK / NACK reason). */
typedef enum {
    SB_OK = 0,
    SB_ERR_JOYSTICK,        /* joystick is in control */
    SB_ERR_JOY_NOT_READY,   /* joystick missing / faulted / not armed: no CONFIRM possible */
    SB_ERR_NO_PROGRAM,      /* no valid program */
    SB_ERR_PRECOND,         /* compass / GPS / gear / first leg not OK */
    SB_ERR_WRONG_MODE,      /* not possible in this mode / state */
    SB_ERR_BAD_PROGRAM,     /* program refused by the validation */
    SB_ERR_WRONG_CODE,      /* power raise: code wrong / expired */
    SB_ERR_AT_MAX,          /* power raise: already at the top */
    SB_ERR_ALARM            /* power raise: an alarm is active (acknowledge first) */
} sb_result_t;

typedef struct {
    sb_config_t cfg;
    mode_mgr_t  modes;
    autopilot_t ap;
    gear_sm_t   gear;
    throttle_t  thr;
    sb_gps_t    gps;
    sb_cruise_t cruise;
    sb_nerv_t   nerv;
    sb_auto_t   autop;
    sb_train_t  train;
    sb_confirm_t confirm;
    plim_t      plim;          /* power limit (RAM only) */
    servo_t     servo;         /* throttle servo output (the only writer) */
    uint32_t    alarm_bits_seen;   /* for "new alarm" detection */
    bool        alarm_seen;
    bool        speed_limited; /* AUTO: speed limited by the power limit */
    uint32_t    sat_ms;        /* how long the speed loop has been at the limit */
    bool        armed;         /* neutral-start interlock passed (until reset) */
    uint32_t    arm_ok_ms, unarmed_ms;
    bool        mon_tripped;   /* the safety monitor tripped (until reset) */
    dr_t        dr;            /* position estimate (GPS / dead reckoning) */
    bool        gps_new_msg;   /* a GPS message arrived this step */
    uint32_t    alarm_bits;    /* SB_ALM_... (what the alarm is about) */
    joy_t       joy;           /* hard-wired joystick */
    sb_mode_t   manual_from;   /* mode the joystick took over from */
    bool        joy_fault_seen, joy_nc_seen;
    uint32_t    chirp_ms;      /* short alarm chirp still to sound */
    uint32_t    ext_buttons;   /* course buttons from the ground station (HAL_BTN_BIT), used by the next step */

    uint32_t now_ms, last_ms;
    bool     stopping;          /* safe stop running (idle + neutral first) */
    sb_stop_steer_t stop_steer; /* steering during the safe stop / HDG_LOCK compass loss */
    bool     stop_from_auto;    /* the stop started in HDG_LOCK / AUTO / TRAINING */
    bool     hdg_compass_lost;  /* HDG_LOCK: compass failed, fault steering until stopped */
    bool     hdg_reverse;       /* heading hold paused (reverse / no propulsion): outboard held at hold_angle */
    float    hdg_target;        /* heading kept while paused */
    float    hold_angle;        /* outboard angle held while paused (0 = centre in reverse) */
    bool     alarm;             /* small-boat alarms (the autopilot has its own) */
    bool     compass_fault_seen;
    bool     gps_was_ok;        /* GPS trusted last step (loss detection) */
    uint32_t gps_losses;        /* trusted GPS lost (events) */

    float    throttle_demand;   /* this step, before limits */
    float    throttle_demand_prev;
    gear_t   gear_demand;

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

/* --- AUTO program (PC) --- */
/* Validate and store (non-volatile, checksum). Refused while AUTO runs it. */
sb_result_t sb_program_set(sb_t *s, const program_t *p, prog_error_t *err, uint8_t *bad_index);
/* Start at waypoint 1 / resume at the active waypoint: arms a CONFIRM
 * request (confirm_ms); the joystick CONFIRM press then starts AUTO. */
sb_result_t sb_auto_start(sb_t *s);
sb_result_t sb_auto_resume(sb_t *s);
sb_result_t sb_auto_pause(sb_t *s);     /* idle + neutral, heading held, stays in AUTO */
sb_result_t sb_auto_skip(sb_t *s);      /* go on to the next waypoint */
/* Seconds left for the pending CONFIRM, 0 if none. */
float sb_confirm_left_s(const sb_t *s);

/* --- power limit (PC) --- */
/* 1. a new 4-digit code for a raise (dir 0 forward, 1 reverse) */
sb_result_t sb_power_code(sb_t *s, uint8_t dir, uint16_t *code);
/* 2. the typed code: OK = waiting for the joystick CONFIRM (confirm_ms) */
sb_result_t sb_power_raise(sb_t *s, uint8_t dir, uint16_t code);
/* lower to the startup limits at once */
void sb_power_lower(sb_t *s);

/* TRAINING finished (stage TS_RESULT): accept (apply + save) or reject.
 * full_angle_deg = measured lock-to-lock angle, 0 if unknown. Goes to
 * STANDBY either way. Returns true if accepted and applied. */
bool sb_confirm_training(sb_t *s, bool accept, float full_angle_deg);

/* Steering angle limit for a speed (from the table). */
float sb_steer_limit_for(const sb_config_t *cfg, float sog_kn);

/* Sim tests only: drive with a raw gear / throttle demand. */
void sb_test_drive(sb_t *s, bool on, gear_t gear, float throttle_pct);

/* Status for displays (e.g. the ground station). */
typedef struct {
    sb_mode_t   mode;
    dr_state_t  pos_source;     /* GPS / dead reckoning / recovering / no fix */
    dr_reason_t dr_reason;      /* why dead reckoning started (last time) */
    float       unc_m;          /* position uncertainty radius */
    float       since_fix_s;    /* time since the last trusted GPS fix */
    float       dr_time_s;      /* time in dead reckoning, 0 on GPS */
    double      lat, lon;       /* estimated position */
    float       sog_kn, cog_deg;/* estimated */
    uint32_t    alarm_bits;     /* SB_ALM_... */
    uint32_t    gps_suspects;   /* fixes rejected by the integrity check (events) */
} sb_status_t;

sb_status_t sb_status(const sb_t *s);

/* Joystick calibration (STANDBY only): start, move the stick to all
 * corners, release, finish -> stored with checksum. */
bool sb_joystick_cal_start(sb_t *s);
bool sb_joystick_cal_finish(sb_t *s);

#endif /* SMALLBOAT_H */
