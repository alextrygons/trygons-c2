/*
 * autopilot.c - Heading-hold autopilot logic.
 *
 * Two cascaded loops, both run at AP_RATE_HZ:
 *   outer: heading error  -> rudder target   (PID, limited to +/-30 deg)
 *   inner: rudder error   -> rudder motor    (P with deadband hysteresis)
 */
#include "autopilot.h"
#include "hal.h"

#include <math.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float autopilot_wrap360(float deg)
{
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    if (w >= 360.0f) w -= 360.0f;   /* guards against -0.0001 + 360 rounding */
    return w;
}

float autopilot_heading_error(float target_deg, float heading_deg)
{
    /* e.g. target 350, heading 10: 340 -> wraps to -20 (turn left 20) */
    float e = autopilot_wrap360(target_deg - heading_deg);
    if (e > 180.0f) e -= 360.0f;
    return e;
}

void autopilot_default_config(ap_config_t *cfg)
{
    cfg->kp                = 1.0f;
    cfg->ki                = 0.05f;
    cfg->kd                = 1.0f;
    cfg->i_limit_deg       = 10.0f;
    cfg->i_zone_deg        = 10.0f;
    cfg->rate_filter_tau_s = 0.5f;

    cfg->motor_gain        = 0.25f;  /* full speed at 4 deg rudder error */
    cfg->deadband_on_deg   = 1.0f;
    cfg->deadband_off_deg  = 0.3f;
}

void autopilot_init(autopilot_t *ap, const ap_config_t *cfg)
{
    uint32_t now = hal_millis();

    ap->cfg = *cfg;
    ap->mode = AP_MODE_STANDBY;
    ap->alarm = false;
    ap->target_heading = 0.0f;
    ap->heading = 0.0f;
    ap->rudder_angle = 0.0f;
    ap->yaw_rate = 0.0f;
    ap->integrator = 0.0f;
    ap->rudder_target = 0.0f;
    ap->motor_cmd = 0.0f;
    ap->motor_moving = false;
    ap->heading_valid = false;
    ap->rudder_valid = false;
    /* Timeouts count from startup: no data within 2 s -> alarm. */
    ap->last_heading_ms = now;
    ap->last_rudder_ms = now;
    ap->sensor_fault = false;
}

/* --- sensors ------------------------------------------------------------ */

static void read_sensors(autopilot_t *ap, uint32_t now)
{
    float h, r;

    /* A reading that is out of range counts as "no new data". */
    if (hal_read_heading(&h) && h >= 0.0f && h < 360.0f) {
        if (ap->heading_valid) {
            float dt = (float)(now - ap->last_heading_ms) * 0.001f;
            if (dt > 0.0f) {
                float raw_rate = autopilot_heading_error(h, ap->heading) / dt;
                float alpha = dt / (ap->cfg.rate_filter_tau_s + dt);
                ap->yaw_rate += alpha * (raw_rate - ap->yaw_rate);
            }
        } else {
            ap->yaw_rate = 0.0f;
        }
        ap->heading = h;
        ap->heading_valid = true;
        ap->last_heading_ms = now;
    }

    if (hal_read_rudder_angle(&r) && fabsf(r) <= HAL_RUDDER_MAX_DEG + 5.0f) {
        ap->rudder_angle = r;
        ap->rudder_valid = true;
        ap->last_rudder_ms = now;
    }
}

static bool sensors_timed_out(const autopilot_t *ap, uint32_t now)
{
    /* Unsigned subtraction is correct even when hal_millis() wraps. */
    return (now - ap->last_heading_ms) >= AP_SENSOR_TIMEOUT_MS
        || (now - ap->last_rudder_ms)  >= AP_SENSOR_TIMEOUT_MS;
}

/* --- modes -------------------------------------------------------------- */

static void enter_standby(autopilot_t *ap)
{
    ap->mode = AP_MODE_STANDBY;
    ap->motor_moving = false;
    ap->motor_cmd = 0.0f;
}

static void enter_auto(autopilot_t *ap)
{
    ap->mode = AP_MODE_AUTO;
    ap->target_heading = ap->heading;     /* lock current heading */
    ap->integrator = 0.0f;
    ap->rudder_target = ap->rudder_angle; /* bumpless: start from where it is */
    ap->motor_moving = false;
}

static void handle_buttons(autopilot_t *ap)
{
    uint32_t presses = hal_get_button_presses();
    bool sensors_ok = ap->heading_valid && ap->rudder_valid && !ap->sensor_fault;

    if (presses & HAL_BTN_BIT(HAL_BTN_STANDBY_AUTO)) {
        if (ap->mode == AP_MODE_AUTO) {
            enter_standby(ap);
        } else {
            ap->alarm = false;            /* pressing also acknowledges alarm */
            if (sensors_ok) {
                enter_auto(ap);
            }
        }
    }

    if (ap->mode == AP_MODE_AUTO) {
        float delta = 0.0f;
        if (presses & HAL_BTN_BIT(HAL_BTN_MINUS_1))  delta -= 1.0f;
        if (presses & HAL_BTN_BIT(HAL_BTN_PLUS_1))   delta += 1.0f;
        if (presses & HAL_BTN_BIT(HAL_BTN_MINUS_10)) delta -= 10.0f;
        if (presses & HAL_BTN_BIT(HAL_BTN_PLUS_10))  delta += 10.0f;
        ap->target_heading = autopilot_wrap360(ap->target_heading + delta);
    }
}

/* --- control loops ------------------------------------------------------ */

/* Outer loop: heading -> rudder target (deg). */
static void heading_loop(autopilot_t *ap)
{
    const ap_config_t *c = &ap->cfg;
    float e = autopilot_heading_error(ap->target_heading, ap->heading);

    float p = c->kp * e;
    float d = -c->kd * ap->yaw_rate;  /* on measurement: no kick on course change */
    float unsat = p + ap->integrator + d;

    /* Anti-windup: integrate only near the course, and never further
     * into saturation. Integrator itself is clamped as well. */
    bool sat_hi = unsat >=  AP_RUDDER_LIMIT_DEG && e > 0.0f;
    bool sat_lo = unsat <= -AP_RUDDER_LIMIT_DEG && e < 0.0f;
    if (fabsf(e) < c->i_zone_deg && !sat_hi && !sat_lo) {
        ap->integrator += c->ki * e * AP_DT_S;
        ap->integrator = clampf(ap->integrator, -c->i_limit_deg, c->i_limit_deg);
    }

    float cmd = clampf(p + ap->integrator + d,
                       -AP_RUDDER_LIMIT_DEG, AP_RUDDER_LIMIT_DEG);

    /* Rate limit the rudder target. */
    float max_step = AP_RUDDER_RATE_DPS * AP_DT_S;
    ap->rudder_target += clampf(cmd - ap->rudder_target, -max_step, max_step);
}

/* Inner loop: rudder target -> motor command. */
static float rudder_loop(autopilot_t *ap)
{
    const ap_config_t *c = &ap->cfg;
    float re = ap->rudder_target - ap->rudder_angle;

    /* Deadband with hysteresis so the motor doesn't hunt. */
    if (ap->motor_moving) {
        if (fabsf(re) < c->deadband_off_deg) ap->motor_moving = false;
    } else {
        if (fabsf(re) > c->deadband_on_deg) ap->motor_moving = true;
    }

    float m = ap->motor_moving ? clampf(c->motor_gain * re, -1.0f, 1.0f) : 0.0f;

    /* Never push past the mechanical limit. */
    if (ap->rudder_angle >=  HAL_RUDDER_MAX_DEG && m > 0.0f) m = 0.0f;
    if (ap->rudder_angle <= -HAL_RUDDER_MAX_DEG && m < 0.0f) m = 0.0f;
    return m;
}

void autopilot_step(autopilot_t *ap)
{
    uint32_t now = hal_millis();

    read_sensors(ap, now);

    /* Sensor timeout: STANDBY + alarm, once at the moment it happens. */
    bool fault = sensors_timed_out(ap, now);
    if (fault && !ap->sensor_fault) {
        enter_standby(ap);
        ap->alarm = true;
    }
    ap->sensor_fault = fault;

    handle_buttons(ap);

    if (ap->mode == AP_MODE_AUTO) {
        heading_loop(ap);
        ap->motor_cmd = rudder_loop(ap);
    } else {
        ap->motor_cmd = 0.0f;
    }

    hal_set_rudder_motor(ap->motor_cmd);
    hal_set_alarm(ap->alarm);
}
