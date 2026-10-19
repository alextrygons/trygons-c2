/*
 * smallboat.c - Small-boat controller. See smallboat.h.
 */
#include "smallboat.h"

#include <math.h>
#include <string.h>

void sb_default_config(sb_config_t *cfg)
{
    autopilot_default_config_outboard(&cfg->steer);
    gear_default_config(&cfg->gear);
    throttle_default_config(&cfg->throttle);
    cfg->gps_timeout_ms = 3000u;
}

void sb_init(sb_t *s, const sb_config_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->now_ms = s->last_ms = hal_millis();
    mode_init(&s->modes, s->now_ms);
    autopilot_init(&s->ap, &cfg->steer);
    gear_init(&s->gear, &cfg->gear);
    throttle_init(&s->thr, &cfg->throttle);
    s->gear_demand = GEAR_N;
}

bool sb_compass_ok(const sb_t *s)
{
    return s->ap.heading_valid && !s->ap.sensor_fault;
}

bool sb_gps_ok(const sb_t *s)
{
    return s->gps.valid && s->gps.last.fix &&
           (s->now_ms - s->gps.last_ms) < s->cfg.gps_timeout_ms;
}

void sb_test_drive(sb_t *s, bool on, gear_t gear, float throttle_pct)
{
    s->test_drive.on = on;
    s->test_drive.gear = gear;
    s->test_drive.throttle_pct = throttle_pct;
}

/* --- modes ---------------------------------------------------------------- */

static bool precondition(const sb_t *s, sb_mode_t to)
{
    switch (to) {
    case MODE_STANDBY:  return true;
    case MODE_MANEUVER: return true;
    case MODE_CRUISE:   return sb_compass_ok(s);
    case MODE_NAV:      return sb_compass_ok(s) && sb_gps_ok(s);
    case MODE_TRAINING: return false;               /* not implemented yet */
    default:            return false;
    }
}

static void on_enter(sb_t *s, sb_mode_t from, sb_mode_t to)
{
    switch (to) {
    case MODE_STANDBY:
        /* safe stop: throttle idle + neutral first (below), steering last */
        s->stopping = true;
        s->hold_while_stopping = (s->ap.mode == AP_MODE_AUTO) && sb_compass_ok(s);
        if (s->hold_while_stopping) autopilot_set_target(&s->ap, s->ap.heading);
        break;
    case MODE_CRUISE:
    case MODE_NAV:
        s->stopping = false;
        if (s->ap.mode != AP_MODE_AUTO)
            autopilot_engage(&s->ap, from == MODE_MANEUVER);
        break;
    default:
        s->stopping = false;
        autopilot_disengage(&s->ap);
        break;
    }
}

bool sb_request_mode(sb_t *s, sb_mode_t to, mode_reason_t why)
{
    sb_mode_t from = s->modes.mode;
    mode_verdict_t v = mode_request(&s->modes, to, why, precondition(s, to), s->now_ms);
    if (v != MODE_OK) return false;
    on_enter(s, from, to);
    return true;
}

/* --- step ------------------------------------------------------------------ */

static void read_inputs(sb_t *s, ap_inputs_t *in)
{
    hal_gps_t g;

    memset(in, 0, sizeof *in);
    in->now_ms = s->now_ms;
    in->gyro_new = hal_read_yaw_rate(&in->gyro_dps);
    in->heading_new = hal_read_heading(&in->heading_deg);
    in->buttons = 0u;                       /* buttons are handled here, not in the core */

    if (hal_read_gps(&g)) {
        s->gps.last = g;
        s->gps.valid = true;
        if (g.fix) s->gps.last_ms = s->now_ms;
    }
}

void sb_step(sb_t *s)
{
    ap_inputs_t in;
    float dt;

    s->last_ms = s->now_ms;
    s->now_ms = hal_millis();
    dt = (float)(s->now_ms - s->last_ms) * 0.001f;
    uint32_t dt_ms = s->now_ms - s->last_ms;

    read_inputs(s, &in);
    uint32_t buttons = hal_get_button_presses();

    /* 1. steering core (also detects compass timeout) */
    autopilot_update(&s->ap, &in);

    /* 2. faults and the Standby button -> STANDBY (always allowed) */
    if (s->ap.sensor_fault && !s->compass_fault_seen) {
        s->compass_fault_seen = true;
        s->alarm = true;
        if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_COMPASS_FAULT);
    }
    if (!s->ap.sensor_fault) s->compass_fault_seen = false;
    if ((buttons & HAL_BTN_BIT(HAL_BTN_STANDBY_AUTO)) && sb_mode(s) != MODE_STANDBY) {
        sb_request_mode(s, MODE_STANDBY, WHY_STANDBY_BUTTON);
    }

    /* 3. what the mode wants */
    s->throttle_demand = 0.0f;
    s->gear_demand = GEAR_N;
    if (sb_mode(s) != MODE_STANDBY && s->test_drive.on) {
        s->gear_demand = s->test_drive.gear;
        s->throttle_demand = s->test_drive.throttle_pct;
    }

    /* 4. safe stop: idle + neutral first, then steering off */
    if (sb_mode(s) == MODE_STANDBY) {
        s->throttle_demand = 0.0f;
        s->gear_demand = GEAR_N;
        if (s->stopping) {
            bool safe = s->thr.out_pct <= s->cfg.gear.idle_max_pct && gear_is(&s->gear, GEAR_N);
            if (!s->hold_while_stopping || !sb_compass_ok(s) || safe) {
                if (s->ap.mode != AP_MODE_STANDBY) autopilot_disengage(&s->ap);
            }
            if (safe) s->stopping = false;
        } else if (s->ap.mode != AP_MODE_STANDBY && s->ap.mode != AP_MODE_TRAINING) {
            autopilot_disengage(&s->ap);
        }
    }

    /* 5. gear and throttle (order: throttle limited by the gear state) */
    gear_request(&s->gear, s->gear_demand);
    float thr = throttle_step(&s->thr, s->throttle_demand, s->gear.current,
                              gear_throttle_allowed(&s->gear), dt);
    gear_step(&s->gear, dt_ms, thr);

    /* 6. outputs: engine first, steering last */
    hal_set_throttle(s->thr.out_pct);
    hal_set_gear_pwm(s->gear.pwm_us);
    hal_set_rudder_motor(s->ap.motor_cmd);
    hal_set_alarm(s->alarm || s->ap.alarm);
}
