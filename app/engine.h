/*
 * engine.h - Engine controls + remote restart (CLAUDE.md section N). Pure C99.
 *
 * Remote restart = PROTO_T_RESTART from the HELM station only (link.c checks
 * HELM; sb_restart checks it again). NO code, NO joystick CONFIRM, NO waiting
 * time, NO attempt limit - but the conditions are checked here EVERY time:
 *   throttle output idle AND gear N confirmed (neutral switch if fitted, else
 *   the gear servo at N + the gear state N), e-stop not pressed, not FIRST TEST.
 * What it does:
 *   (a) the ENGINE KILL latch (UNCREWED lost link) is cleared (invariant I31:
 *       "the kill stays latched until a reset on board OR a remote restart from
 *       the HELM station with idle + neutral confirmed and the e-stop released");
 *   (b) after a monitor trip: the HAL latch is cleared only by a reset (R3), so
 *       the restart asks for a controlled software reset of the AURIX (log event
 *       27 RESTART first: the reset follows ENG_RESET_DELAY_MS later, after the
 *       data flash is idle (at most ENG_RESET_NV_MAX_MS), the conditions checked
 *       again every step until then) - everything starts again (neutral-start
 *       interlock, startup power limits, boot counter) and the trip re-arms;
 *   (c) hold-to-crank: the station sends RESTART action CRANK at 5 Hz while its
 *       START button is held; each one refreshes a board-timed STARTER pulse
 *       (ENG_CRANK_PULSE_MS) through relays.c, a hold lasts at most
 *       ENG_CRANK_MAX_MS, then (and after every hold) ENG_COOL_MS cool-down. The
 *       cranking stops on release (no CRANK for ENG_RELEASE_MS, or action
 *       RELEASE), HELM / gateway link loss, e-stop, gear not N (or a gear
 *       request), throttle not idle, the STANDBY button, an ENGINE KILL or a
 *       monitor trip. IGNITION first is the station's order (engine page).
 *   (d) IGNITION / SHUTOFF (the engine-group relays with that ROLE in the
 *       relay table): switched ON only here (engine_relay -> relays_engine_set),
 *       with the same conditions (idle + neutral confirmed, e-stop released,
 *       not FIRST TEST) and not while the ENGINE KILL is latched or the monitor
 *       tripped; OFF always. The plain relay command never switches an engine
 *       relay ON (relays.c), whatever its type.
 * No tacho yet: the time limit only. The engine kill pin stays its own AURIX
 * output (hal_set_engine_kill), never the relay board. This module only
 * decides; the STARTER relay is driven by relays.c (relays_starter_pulse /
 * relays_starter_off, called only from here: tools/check.sh).
 */
#ifndef ENGINE_H
#define ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include "relays.h"

#define ENG_CRANK_PULSE_MS   300u    /* board-timed STARTER pulse, refreshed by every CRANK */
#define ENG_CRANK_MAX_MS     5000u   /* crank_max_s: one hold never longer */
#define ENG_COOL_MS          10000u  /* cool-down after a hold */
#define ENG_RELEASE_MS       300u    /* no CRANK this long = released */
#define ENG_RESET_DELAY_MS   500u    /* log event RESTART reaches the logger / gateway before the reset */
#define ENG_RESET_NV_MAX_MS  2000u   /* a data-flash write in progress: wait at most this */

/* PROTO_T_RESTART actions (u8) */
#define ENG_ACT_RESTART      0u      /* clear the ENGINE KILL / reset after a monitor trip */
#define ENG_ACT_CRANK        1u      /* START held: refresh the STARTER pulse (5 Hz) */
#define ENG_ACT_RELEASE      2u      /* START released */

/* engine_t.crank */
#define ENG_CRANK_OFF        0u
#define ENG_CRANK_ON         1u
#define ENG_CRANK_COOL       2u

/* why something was refused / stopped (TELEM2 group O, log event RESTART) */
typedef enum {
    ENG_WHY_NONE = 0,
    ENG_WHY_FIRST_TEST,     /* 1 FIRST TEST: never */
    ENG_WHY_THROTTLE,       /* 2 throttle output not idle */
    ENG_WHY_GEAR,           /* 3 gear N not confirmed (or a gear request) */
    ENG_WHY_ESTOP,          /* 4 e-stop pressed */
    ENG_WHY_NOTHING,        /* 5 restart: nothing latched (no kill, no trip) */
    ENG_WHY_KILLED,         /* 6 crank: the ENGINE KILL is latched (restart first) */
    ENG_WHY_TRIPPED,        /* 7 crank: the monitor tripped (restart = reset first) */
    ENG_WHY_NO_STARTER,     /* 8 crank: no STARTER (the relay with the STARTER role) in the table */
    ENG_WHY_COOLING,        /* 9 crank: cool-down after the last hold */
    ENG_WHY_RELAY,          /* 10 crank: relay board silent / refused */
    ENG_WHY_NOT_HELM,       /* 11 not the HELM station */
    ENG_WHY_LINK,           /* 12 crank stopped: HELM / gateway link lost */
    ENG_WHY_STANDBY_BTN,    /* 13 crank stopped: STANDBY button */
    ENG_WHY_RELEASED,       /* 14 crank stopped: START released (or no CRANK for ENG_RELEASE_MS) */
    ENG_WHY_MAX_TIME,       /* 15 crank stopped: ENG_CRANK_MAX_MS reached */
    ENG_WHY_COUNT
} eng_why_t;

/* log event RESTART action (a) */
#define ENG_LOG_KILL_CLEARED 1u
#define ENG_LOG_RESET        2u      /* a reset after a monitor trip is coming */
#define ENG_LOG_CRANK_START  3u
#define ENG_LOG_CRANK_STOP   4u
#define ENG_LOG_REFUSED      5u      /* restart refused */
#define ENG_LOG_CRANK_REFUSED 6u
#define ENG_LOG_RESET_CANCEL 7u      /* the conditions failed before the reset */

/* the boat as the conditions see it (smallboat.c fills it every step) */
typedef struct {
    bool idle;              /* throttle output at idle */
    bool neutral;           /* gear N confirmed */
    bool gear_request;      /* the gear demand this step is not N */
    bool estop_pressed;
    bool first_test;
    bool killed;            /* ENGINE KILL latched */
    bool tripped;           /* the safety monitor tripped (HAL latch) */
    bool helm_link;         /* the HELM station's link is up */
    uint8_t helm;           /* the HELM holder */
    bool standby_button;    /* pressed this step */
    bool nv_busy;           /* a data-flash save is pending */
} eng_cond_t;

typedef struct {
    uint8_t  crank;             /* ENG_CRANK_ */
    uint8_t  crank_station;     /* who holds START */
    uint32_t hold_t0_ms;        /* start of the current hold */
    uint32_t last_req_ms;       /* last CRANK */
    uint32_t cool_until_ms;
    uint32_t hold_ms;           /* length of the current / last hold */
    bool     reset_pending;     /* (b): the software reset is coming */
    uint32_t reset_at_ms, reset_req_ms;
    uint8_t  last_action;       /* ENG_LOG_ (log producer) */
    uint8_t  last_why;          /* eng_why_t of the last refusal / stop */
    uint8_t  stop_why;          /* eng_why_t of the last crank stop */
    uint32_t restarts;          /* accepted restarts (kill clears + resets) */
    uint32_t kill_clears, resets, cranks, refused;
    uint32_t pulses;            /* STARTER pulses sent (refreshes) */
    uint32_t log_seq;           /* +1 per event for the log producer */
} engine_t;

void      engine_init(engine_t *e);
/* the restart conditions: ENG_WHY_NONE = met */
eng_why_t engine_check(const eng_cond_t *c);
/* RESTART from `station`: *kill_clear = clear the ENGINE KILL now (caller);
 * a reset after a trip is queued here (engine_step says when). */
eng_why_t engine_restart(engine_t *e, const eng_cond_t *c, uint8_t station, uint32_t now_ms, bool *kill_clear);
/* CRANK (START held) / RELEASE from `station` */
eng_why_t engine_crank(engine_t *e, const eng_cond_t *c, relays_t *r, uint8_t station, uint32_t now_ms);
void      engine_release(engine_t *e, relays_t *r, uint8_t station, uint32_t now_ms);
/* IGNITION / SHUTOFF relay n on / off (the HELM station's relay command; link.c checks HELM).
 * ON needs engine_check + not killed / tripped; *res = the relay layer's answer
 * (RELAY_R_ENGINE when the conditions refused it). */
eng_why_t engine_relay(engine_t *e, const eng_cond_t *c, relays_t *r, uint8_t n, bool on, relay_result_t *res);
/* every control step (after the outputs): stops a crank whose conditions
 * failed, ends the cool-down, re-checks a pending reset. True = do the
 * software reset NOW (the caller calls hal_system_reset). */
bool      engine_step(engine_t *e, const eng_cond_t *c, relays_t *r, uint32_t now_ms);
/* TELEM2 group O */
float     engine_cool_left_s(const engine_t *e, uint32_t now_ms);

#endif /* ENGINE_H */
