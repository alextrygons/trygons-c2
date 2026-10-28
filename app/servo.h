/*
 * servo.h - The ONLY place that writes the throttle, gear and steering
 * outputs (lowest layer; the build check greps for hal_set_throttle_pwm,
 * hal_set_gear_pwm and hal_set_steer_pos). With the DroneCAN servos (CLAUDE.md
 * T, step 11e) the HAL turns these calls into ArrayCommands (hal/can_act.c).
 *
 * servo_throttle_write() clamps every throttle command:
 *   1. to the current power limit (powerlim.c): forward limit only when
 *      the gearbox is confirmed in FORWARD, otherwise the (smaller)
 *      reverse limit - so a wrong gear state can never give forward power;
 *   2. to 0..100 % of the calibrated full travel;
 *      When the limit drops, the output comes down at rate_down_pct_s (no
 *      sudden throttle cut at speed), but it can never go up above it; an
 *      EMERGENCY (STANDBY, fault, link loss ...: throttle.c stop) goes to idle
 *      at once - no software limit ever slows a move toward idle.
 *   3. the pulse width to the calibrated physical endpoints (idle_us ..
 *      full_us), then hal_set_throttle_pwm().
 *
 * The throttle MODEL (docs/dronecan_design.md 7): the servo has no feedback, so
 * every rule that waits for "idle" (gear change, steering off after a stop,
 * ENGINE KILL, STARTER, generator, DP, restart) uses est_pct = an UPPER BOUND of
 * the real servo position: it follows the command up at once and comes down at
 * the worst-case servo speed (100 % per travel_s, thr_travel_s 6 s - only an
 * owner-signed value may shorten it, OD-4). "Idle" = est_pct at idle, i.e. the
 * command at idle AND long enough for the slow servo to have got there.
 * DroneCAN: after every boot, and while the throttle node is lost, the position
 * is UNKNOWN (est_pct = 100 %): no gear frame until the node is OPERATIONAL again
 * and has been commanded idle for travel_s (owner addition OA-2, gear HOLD).
 *
 * servo_gear_write(): the gear state machine's pulse; DroneCAN: HOLD (no frame)
 * until the first gear frame may go out (boot order: the modelled throttle idle
 * with the throttle node OPERATIONAL), and while the throttle node is lost.
 * servo_steer_write(): the steering motor (old hardware) or the steering
 * position servo (DroneCAN: no frame until a mode commanded steering, OD-7).
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "powerlim.h"

#define SERVO_THR_TRAVEL_S   6.0f    /* worst-case throttle servo idle <-> full (11c: ArduPilot measured; TO CONFIRM C10) */
#define SERVO_THR_TRAVEL_MIN 1.0f    /* an owner-signed value never below this */
#define SERVO_THR_TRAVEL_MAX 30.0f

/* DroneCAN servos: the UNITLESS wire convention (docs/ardupilot.param) - idle = -1.0 = 1000 us; the node does any
 * mapping / reversal. Core 0's boot frame and CPU1's safe frames send -1.0, so idle_us must be this (step 11g). */
#define SERVO_CAN_IDLE_US 1000u

typedef struct {
    uint16_t idle_us;           /* pulse at idle (calibrated) (1000) */
    uint16_t full_us;           /* pulse at full throttle (calibrated) (2000); may be < idle_us */
    float    rate_down_pct_s;   /* after a limit drop, come down this fast (50, as throttle.c) */
    float    travel_s;          /* thr_travel_s: worst-case servo travel idle <-> full (6) */
} servo_config_t;

typedef struct {
    servo_config_t cfg;
    float    out_pct;           /* what was really sent, % of full travel */
    uint16_t out_us;            /* pulse sent */
    float    limit_pct;         /* limit applied this time */
    bool     clamped;           /* the command was above the limit */
    /* the model + the DroneCAN boot order (step 11e) */
    bool     can;               /* the servos are DroneCAN nodes (hal_servos_can at init) */
    bool     cfg_bad;           /* DroneCAN: idle_us is not SERVO_CAN_IDLE_US (step 11g): the throttle pulse is
                                 * then ALWAYS SERVO_CAN_IDLE_US (wire -1.0 = idle, like the boot frame and CPU1's
                                 * safe frames) and the outputs never arm (smallboat.c interlock, NOT ARMED) */
    float    est_pct;           /* modelled throttle servo position (upper bound), % */
    bool     node_ok;           /* the throttle node OPERATIONAL (DroneCAN; always true otherwise) */
    bool     gear_sent;         /* a gear command went out since the boot */
    uint16_t gear_us;           /* the last gear pulse sent (0 = none yet) */
    bool     gear_hold;         /* this step: no gear frame (the node holds) */
    bool     steer_have;        /* a steering command went out since the boot (OD-7) */
    float    steer_pos;         /* the last one (position servo, -1..+1) */
} servo_t;

void servo_default_config(servo_config_t *c);
void servo_init(servo_t *sv, const servo_config_t *c);

/* Write the throttle servo. forward_engaged = gearbox confirmed in F.
 * Returns the throttle actually sent (%). */
/* neutral_run_pct: generator mode (section K) - the gear is settled in NEUTRAL
 * and the generator is active: the clamp is min(neutral_run_pct, forward limit)
 * instead of the reverse limit; < 0 = not active (the normal rule). */
float servo_throttle_write(servo_t *sv, const plim_t *lim, bool forward_engaged, float pct, float dt_s,
                           float neutral_run_pct);

/* The model, once per step after the write: node_ok = the throttle node
 * OPERATIONAL (DroneCAN; pass true on the PWM hardware). */
void servo_model_step(servo_t *sv, bool node_ok, float dt_s);

/* the modelled throttle position at idle (<= idle_max_pct) */
bool servo_idle(const servo_t *sv, float idle_max_pct);

/* The gear servo pulse (the gear state machine's). */
void servo_gear_write(servo_t *sv, uint16_t pulse_us, float idle_max_pct);

/* Steering: the motor command (old hardware: -1..+1) or the position servo
 * (DroneCAN: command = a mode commands steering now or did since the boot,
 * pos -1..+1). */
void servo_steer_write(servo_t *sv, float motor_cmd, bool command, float pos);

/* Pulse for a throttle %, inside the calibrated endpoints. */
uint16_t servo_pulse_for(const servo_config_t *c, float pct);

#endif /* SERVO_H */
