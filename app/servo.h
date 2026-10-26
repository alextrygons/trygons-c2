/*
 * servo.h - The ONLY place that writes the throttle servo (lowest layer).
 *
 * servo_throttle_write() clamps every throttle command:
 *   1. to the current power limit (powerlim.c): forward limit only when
 *      the gearbox is confirmed in FORWARD, otherwise the (smaller)
 *      reverse limit - so a wrong gear state can never give forward power;
 *   2. to 0..100 % of the calibrated full travel;
 *      When the limit drops, the output comes down at rate_down_pct_s (no
 *      sudden throttle cut at speed), but it can never go up above it.
 *   3. the pulse width to the calibrated physical endpoints (idle_us ..
 *      full_us), then hal_set_throttle_pwm().
 * No other code may call hal_set_throttle_pwm (the build check greps for it).
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "powerlim.h"

typedef struct {
    uint16_t idle_us;           /* pulse at idle (calibrated) (1000) */
    uint16_t full_us;           /* pulse at full throttle (calibrated) (2000); may be < idle_us */
    float    rate_down_pct_s;   /* after a limit drop, come down this fast (40, as throttle.c) */
} servo_config_t;

typedef struct {
    servo_config_t cfg;
    float    out_pct;           /* what was really sent, % of full travel */
    uint16_t out_us;            /* pulse sent */
    float    limit_pct;         /* limit applied this time */
    bool     clamped;           /* the command was above the limit */
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

/* Pulse for a throttle %, inside the calibrated endpoints. */
uint16_t servo_pulse_for(const servo_config_t *c, float pct);

#endif /* SERVO_H */
