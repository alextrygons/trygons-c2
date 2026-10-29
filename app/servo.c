/*
 * servo.c - Throttle / gear / steering outputs with the power-limit clamp and
 * the throttle model. See servo.h.
 */
#include "servo.h"

#include <math.h>

#include "hal.h"

void servo_default_config(servo_config_t *c)
{
    c->idle_us = 1000u;
    c->full_us = 2000u;
    c->rate_down_pct_s = 50.0f;
    c->travel_s = SERVO_THR_TRAVEL_S;
}

void servo_init(servo_t *sv, const servo_config_t *c)
{
    sv->cfg = *c;
    if (!(sv->cfg.travel_s >= SERVO_THR_TRAVEL_MIN)) sv->cfg.travel_s = SERVO_THR_TRAVEL_S;
    if (sv->cfg.travel_s > SERVO_THR_TRAVEL_MAX) sv->cfg.travel_s = SERVO_THR_TRAVEL_MAX;
    sv->out_pct = 0.0f;
    sv->out_us = c->idle_us;
    sv->limit_pct = 0.0f;
    sv->clamped = false;
    sv->can = hal_servos_can();
    sv->cfg_bad = sv->can && (c->idle_us != SERVO_CAN_IDLE_US || c->full_us != SERVO_CAN_FULL_US);
    /* DroneCAN: after a boot the throttle position is UNKNOWN (the node held whatever it had) */
    sv->est_pct = sv->can ? 100.0f : 0.0f;
    sv->node_ok = !sv->can;
    sv->gear_sent = false;
    sv->gear_us = 0u;
    sv->gear_hold = sv->can;
    sv->steer_have = false;
    sv->steer_pos = 0.0f;
}

uint16_t servo_pulse_for(const servo_config_t *c, float pct)
{
    float lo = (float)((c->idle_us < c->full_us) ? c->idle_us : c->full_us);
    float hi = (float)((c->idle_us < c->full_us) ? c->full_us : c->idle_us);
    if (!(pct >= 0.0f)) pct = 0.0f;     /* NaN / negative -> IDLE (not the lower pulse: on a reversed
                                         * servo, full_us < idle_us, that would be full throttle) */
    if (pct > 100.0f) pct = 100.0f;
    float us = (float)c->idle_us + pct * 0.01f * ((float)c->full_us - (float)c->idle_us);
    if (!(us >= lo)) us = lo;
    if (us > hi) us = hi;
    return (uint16_t)(us + 0.5f);
}

float servo_throttle_write(servo_t *sv, const plim_t *lim, bool forward_engaged, float pct, float dt_s,
                           float neutral_run_pct)
{
    float limit = forward_engaged ? lim->fwd_pct : lim->rev_pct;
    /* generator mode: in neutral, never above the generator cap nor the forward limit */
    if (!forward_engaged && neutral_run_pct >= 0.0f)
        limit = (neutral_run_pct < lim->fwd_pct) ? neutral_run_pct : lim->fwd_pct;
    if (limit > 100.0f) limit = 100.0f;
    if (limit < 0.0f) limit = 0.0f;
    /* just after a limit drop: may still be above it, but only coming down (an emergency
     * asks for idle and gets it at once: pct below `allowed` is never held up) */
    float allowed = sv->out_pct - sv->cfg.rate_down_pct_s * dt_s;
    if (allowed < limit) allowed = limit;
    sv->clamped = pct > limit;
    if (!(pct >= 0.0f)) pct = 0.0f;     /* negative or NaN -> idle */
    if (pct > allowed) pct = allowed;
    if (sv->cfg_bad) pct = 0.0f;        /* DroneCAN with idle_us != 1000: idle only (never armed) */
    sv->limit_pct = limit;
    sv->out_pct = pct;
    sv->out_us = sv->cfg_bad ? (uint16_t)SERVO_CAN_IDLE_US : servo_pulse_for(&sv->cfg, pct);
    hal_set_throttle_pwm(sv->out_us);
    return pct;
}

void servo_model_step(servo_t *sv, bool node_ok, float dt_s)
{
    float travel = sv->cfg.travel_s;
    if (!(travel >= SERVO_THR_TRAVEL_MIN)) travel = SERVO_THR_TRAVEL_S;
    sv->node_ok = node_ok || !sv->can;
    if (!sv->node_ok) {
        sv->est_pct = 100.0f;           /* the node does what it does: position unknown (worst case) */
        return;
    }
    float down = sv->est_pct - 100.0f / travel * dt_s;
    sv->est_pct = (sv->out_pct > down) ? sv->out_pct : down;
    if (sv->est_pct < 0.0f) sv->est_pct = 0.0f;
}

bool servo_idle(const servo_t *sv, float idle_max_pct)
{
    return sv->est_pct <= idle_max_pct && sv->out_pct <= idle_max_pct;
}

void servo_gear_write(servo_t *sv, uint16_t pulse_us, float idle_max_pct)
{
    /* DroneCAN: the first gear frame only with the throttle node OPERATIONAL and the modelled throttle at
     * idle (the boot order); no gear frame at all while the throttle node is lost (gear HOLD, OA-2) */
    (void)idle_max_pct;
    /* (the first frame: the model fully at idle - the whole worst-case travel time since the boot / the node came) */
    sv->gear_hold = sv->can && (!sv->node_ok || (!sv->gear_sent && !(sv->est_pct <= 0.0f && sv->out_pct <= 0.0f)));
    if (sv->gear_hold) {
        hal_set_gear_pwm(0u);
        return;
    }
    sv->gear_sent = true;
    sv->gear_us = pulse_us;
    hal_set_gear_pwm(pulse_us);
}

void servo_steer_write(servo_t *sv, float motor_cmd, bool command, float pos)
{
    if (!sv->can) {
        hal_set_rudder_motor(motor_cmd);
        return;
    }
    if (command && isfinite(pos)) {
        sv->steer_have = true;
        sv->steer_pos = pos;
    }
    /* once a mode commanded steering the last position keeps being sent (STANDBY = HOLD, OD-10) */
    hal_set_steer_pos(sv->steer_have, sv->steer_pos);
}
