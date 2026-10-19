/*
 * smallboat.c - Small-boat controller. See smallboat.h.
 */
#include "smallboat.h"

#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void sb_default_config(sb_config_t *cfg)
{
    static const float lkn[SB_LIMIT_N]  = { 3.0f, 8.0f, 14.0f, 20.0f };
    static const float ldeg[SB_LIMIT_N] = { 30.0f, 20.0f, 12.0f, 8.0f };

    autopilot_default_config_outboard(&cfg->steer);
    gear_default_config(&cfg->gear);
    throttle_default_config(&cfg->throttle);
    cfg->gps_timeout_ms = 3000u;

    cfg->maneuver_throttle_pct = 30.0f;
    cfg->steer_deadzone = 0.3f;

    cfg->cruise_speed_kn = 8.0f;
    cfg->cruise_max_kn = 25.0f;
    cfg->spd_kp = 4.0f;
    cfg->spd_ki = 0.6f;
    cfg->gps_loss_throttle_factor = 0.6f;

    memcpy(cfg->limit_kn, lkn, sizeof lkn);
    memcpy(cfg->limit_deg, ldeg, sizeof ldeg);
    cfg->gain_ref_kn = 6.0f;
    cfg->gain_scale_min = 0.3f;
    cfg->gain_scale_max = 2.0f;

    cfg->nerv_on_dps = 2.5f;
    cfg->nerv_off_dps = 1.6f;
    cfg->nerv_roll_on_dps = 8.0f;
    cfg->nerv_roll_off_dps = 5.0f;
    cfg->nerv_on_ms = 5000u;
    cfg->nerv_off_ms = 30000u;
    cfg->nerv_step_kn = 2.0f;
    cfg->nerv_step_ms = 15000u;
    cfg->nerv_recover_kn = 1.0f;
    cfg->nerv_recover_ms = 30000u;
    cfg->nerv_memory_ms = 300000u;
    cfg->nerv_min_kn = 5.0f;

    cfg->arrival_radius_m = 25.0f;
    cfg->slow_radius_m = 150.0f;
    cfg->approach_kn = 4.0f;
    cfg->arrive_stop = true;
    cfg->lookahead_min_m = 40.0f;
    cfg->lookahead_s = 6.0f;
    cfg->crab_tau_s = 20.0f;
}

void sb_set_waypoint(sb_t *s, double lat, double lon, float speed_kn)
{
    s->nav.wp_set = true;
    s->nav.wp_lat = lat;
    s->nav.wp_lon = lon;
    s->nav.speed_kn = clampf(speed_kn, 0.0f, s->cfg.cruise_max_kn);
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
    s->cruise.set_kn = cfg->cruise_speed_kn;
    s->cruise.target_kn = cfg->cruise_speed_kn;
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

void sb_set_cruise_speed(sb_t *s, float kn)
{
    s->cruise.set_kn = clampf(kn, 0.0f, s->cfg.cruise_max_kn);
    /* rough water still limits it; otherwise take the new speed */
    if (!s->nerv.nervous && s->nerv.ceiling_ms == 0u) s->cruise.target_kn = s->cruise.set_kn;
    else s->cruise.target_kn = fminf(s->cruise.target_kn, s->cruise.set_kn);
}

float sb_steer_limit_for(const sb_config_t *cfg, float sog_kn)
{
    if (sog_kn <= cfg->limit_kn[0]) return cfg->limit_deg[0];
    for (int i = 1; i < SB_LIMIT_N; i++) {
        if (sog_kn <= cfg->limit_kn[i]) {
            float f = (sog_kn - cfg->limit_kn[i - 1]) / (cfg->limit_kn[i] - cfg->limit_kn[i - 1]);
            return cfg->limit_deg[i - 1] + f * (cfg->limit_deg[i] - cfg->limit_deg[i - 1]);
        }
    }
    return cfg->limit_deg[SB_LIMIT_N - 1];
}

/* --- modes ---------------------------------------------------------------- */

static bool precondition(const sb_t *s, sb_mode_t to)
{
    switch (to) {
    case MODE_STANDBY:  return true;
    case MODE_MANEUVER: return true;
    case MODE_CRUISE:   return sb_compass_ok(s);
    case MODE_NAV:      return sb_compass_ok(s) && sb_gps_ok(s) && s->nav.wp_set;
    case MODE_TRAINING: return false;               /* not implemented yet */
    default:            return false;
    }
}

static void cruise_start(sb_t *s)
{
    /* bumpless: the speed loop starts from the throttle that is open now */
    s->cruise.integ = s->thr.out_pct;
    s->cruise.gps_lost = false;
    s->cruise.target_kn = s->cruise.set_kn;
    memset(&s->nerv, 0, sizeof s->nerv);
}

static void on_enter(sb_t *s, sb_mode_t from, sb_mode_t to)
{
    /* speed dependent steering only in the auto-steering modes */
    s->ap.steer_limit_deg = 0.0f;
    s->ap.gain_scale = 1.0f;

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
        if (from != MODE_CRUISE && from != MODE_NAV) cruise_start(s);
        if (to == MODE_NAV) {
            /* the line starts where the boat is now */
            nav_leg_init(&s->nav.leg, s->gps.last.lat_deg, s->gps.last.lon_deg,
                         s->nav.wp_lat, s->nav.wp_lon);
            s->nav.crab_deg = 0.0f;
            s->nav.arrived = false;
            s->nav.last_fix_ms = s->gps.last_ms;
            s->nav.course_deg = s->nav.leg.track_deg;
            autopilot_set_target(&s->ap, s->nav.leg.track_deg);
            sb_set_cruise_speed(s, s->nav.speed_kn);
        }
        break;
    case MODE_MANEUVER:
        /* from STANDBY the outboard is assumed centred by hand */
        s->stopping = false;
        autopilot_manual(&s->ap, from != MODE_STANDBY);
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

/* --- inputs ---------------------------------------------------------------- */

static void read_inputs(sb_t *s, ap_inputs_t *in, uint32_t buttons)
{
    hal_gps_t g;

    memset(in, 0, sizeof *in);
    in->now_ms = s->now_ms;
    in->gyro_new = hal_read_yaw_rate(&in->gyro_dps);
    in->heading_new = hal_read_heading(&in->heading_deg);

    /* course buttons go to the steering core in the auto-steering modes;
     * Standby/Auto is handled here (mode manager), never in the core */
    if (sb_mode(s) == MODE_CRUISE)
        in->buttons = buttons & ~HAL_BTN_BIT(HAL_BTN_STANDBY_AUTO);

    if (hal_read_gps(&g)) {
        s->gps.last = g;
        s->gps.valid = true;
        if (g.fix) s->gps.last_ms = s->now_ms;
    }

    s->ctl_ok = hal_read_controls(&s->ctl);
    if (s->ctl_ok && sb_mode(s) == MODE_MANEUVER) {
        if (s->ctl.steer > s->cfg.steer_deadzone) in->manual_steer = 1;
        else if (s->ctl.steer < -s->cfg.steer_deadzone) in->manual_steer = -1;
    }
}

/* --- MANEUVER ---------------------------------------------------------------- */

/* Operator gear (through the gear state machine) and throttle, limited in
 * both directions. No controls = neutral, idle. */
static void maneuver(sb_t *s)
{
    if (!s->ctl_ok) return;
    float lever = clampf(s->ctl.throttle, 0.0f, 1.0f);
    s->gear_demand = (s->ctl.gear == HAL_SEL_F) ? GEAR_F
                   : (s->ctl.gear == HAL_SEL_R) ? GEAR_R : GEAR_N;
    s->throttle_demand = lever * s->cfg.maneuver_throttle_pct;
}

/* --- CRUISE ------------------------------------------------------------------ */

/* Rough water: wobble (std after removing the slow part) of the gyro yaw
 * rate (and roll rate if there is a sensor). Hysteresis + minimum times so
 * it doesn't flip. Nervous: slow down in steps. Calm: speed up slowly, but
 * stay below the speed where it got rough for nerv_memory_ms. */
static void nervousness(sb_t *s, uint32_t dt_ms, bool roll_new, float roll_dps)
{
    const sb_config_t *c = &s->cfg;
    sb_nerv_t *n = &s->nerv;
    float dt = (float)dt_ms * 0.001f;

    if (s->ap.gyro_valid) {
        float r = s->ap.gyro_dps;
        n->yaw_lp += (r - n->yaw_lp) * dt / 10.0f;
        float hp = r - n->yaw_lp;
        n->yaw_var += (hp * hp - n->yaw_var) * dt / 10.0f;
        n->yaw_std = sqrtf(n->yaw_var);
    }
    if (roll_new) {
        n->roll_ok = true;
        n->roll_lp += (roll_dps - n->roll_lp) * dt / 10.0f;
        float hp = roll_dps - n->roll_lp;
        n->roll_var += (hp * hp - n->roll_var) * dt / 10.0f;
        n->roll_std = sqrtf(n->roll_var);
    }

    bool rough = n->yaw_std > c->nerv_on_dps || (n->roll_ok && n->roll_std > c->nerv_roll_on_dps);
    bool calm = n->yaw_std < c->nerv_off_dps && (!n->roll_ok || n->roll_std < c->nerv_roll_off_dps);

    if (!n->nervous) {
        n->above_ms = rough ? n->above_ms + dt_ms : 0u;
        if (n->above_ms >= c->nerv_on_ms) {
            n->nervous = true;
            n->episodes++;
            n->transitions++;
            n->below_ms = 0u;
            n->step_timer_ms = c->nerv_step_ms;     /* first step at once */
        }
    } else {
        n->below_ms = calm ? n->below_ms + dt_ms : 0u;
        if (n->below_ms >= c->nerv_off_ms) {
            n->nervous = false;
            n->transitions++;
            n->above_ms = 0u;
            n->recover_timer_ms = 0u;
        }
    }

    if (n->nervous) {
        /* only keep slowing down while still clearly rough (above the
         * middle of the hysteresis band): the wobble measurement and the
         * boat speed both react slowly, so stepping on would overshoot */
        float mid = 0.5f * (c->nerv_on_dps + c->nerv_off_dps);
        bool still_rough = n->yaw_std > mid ||
                           (n->roll_ok && n->roll_std > 0.5f * (c->nerv_roll_on_dps + c->nerv_roll_off_dps));
        if (still_rough) n->step_timer_ms += dt_ms;
        if (n->step_timer_ms >= c->nerv_step_ms) {
            n->step_timer_ms = 0u;
            s->cruise.target_kn = fmaxf(c->nerv_min_kn, s->cruise.target_kn - c->nerv_step_kn);
            n->ceiling_kn = s->cruise.target_kn;    /* remember: rough above this */
            n->ceiling_ms = c->nerv_memory_ms;
        }
    } else {
        if (n->ceiling_ms > 0u) n->ceiling_ms = (n->ceiling_ms > dt_ms) ? n->ceiling_ms - dt_ms : 0u;
        float top = (n->ceiling_ms > 0u) ? fminf(n->ceiling_kn, s->cruise.set_kn) : s->cruise.set_kn;
        n->recover_timer_ms += dt_ms;
        if (n->recover_timer_ms >= c->nerv_recover_ms) {
            n->recover_timer_ms = 0u;
            if (s->cruise.target_kn < top)
                s->cruise.target_kn = fminf(top, s->cruise.target_kn + c->nerv_recover_kn);
        }
        if (s->cruise.target_kn > s->cruise.set_kn) s->cruise.target_kn = s->cruise.set_kn;
    }
}

/* Forward only. Speed on GPS SOG (PI, anti-windup); without GPS the
 * throttle is held reduced and the alarm sounds. Steering limit and gains
 * follow the speed. */
static void cruise(sb_t *s, float dt, uint32_t dt_ms, bool roll_new, float roll_dps)
{
    const sb_config_t *c = &s->cfg;
    sb_cruise_t *k = &s->cruise;

    s->gear_demand = GEAR_F;
    nervousness(s, dt_ms, roll_new, roll_dps);

    bool gps = sb_gps_ok(s);
    if (gps) {
        k->sog_kn = s->gps.last.sog_kn;
        k->gps_lost = false;
    }

    if (!gear_is(&s->gear, GEAR_F)) {
        s->throttle_demand = 0.0f;          /* wait for forward gear */
    } else if (gps) {
        float e = k->target_kn - k->sog_kn;
        float u = c->spd_kp * e + k->integ;
        bool sat_hi = u >= c->throttle.max_fwd_pct && e > 0.0f;
        bool sat_lo = u <= 0.0f && e < 0.0f;
        bool lagging = fabsf(s->throttle_demand_prev - s->thr.out_pct) > 3.0f;   /* rate-limited */
        if (!sat_hi && !sat_lo && !lagging)
            k->integ = clampf(k->integ + c->spd_ki * e * dt, 0.0f, c->throttle.max_fwd_pct);
        s->throttle_demand = clampf(c->spd_kp * e + k->integ, 0.0f, c->throttle.max_fwd_pct);
    } else {
        if (!k->gps_lost) {
            k->gps_lost = true;
            k->open_loop_pct = s->thr.out_pct * c->gps_loss_throttle_factor;
            k->integ = k->open_loop_pct;
            s->alarm = true;
        }
        s->throttle_demand = k->open_loop_pct;
    }

    float v = k->gps_lost ? k->sog_kn * c->gps_loss_throttle_factor : k->sog_kn;
    s->ap.steer_limit_deg = sb_steer_limit_for(c, v);
    s->ap.gain_scale = clampf(c->gain_ref_kn / fmaxf(v, 0.5f), c->gain_scale_min, c->gain_scale_max);
}

/* --- NAV ----------------------------------------------------------------------- */

/* Follow the start -> waypoint line. On each GPS fix: cross-track error ->
 * wanted course over ground; plus the drift angle (heading - COG, from
 * current / wind) -> heading for the steering core. Speed: travel speed,
 * slowing to approach_kn inside slow_radius. Speed control, steering
 * limit and rough-water handling are the same as CRUISE. */
static void nav_mode(sb_t *s, float dt, uint32_t dt_ms, bool roll_new, float roll)
{
    const sb_config_t *c = &s->cfg;
    sb_nav_t *n = &s->nav;

    if (!sb_gps_ok(s)) {
        /* GPS lost: CRUISE on the current heading, reduced speed, alarm */
        s->alarm = true;
        autopilot_set_target(&s->ap, s->ap.heading);
        sb_request_mode(s, MODE_CRUISE, WHY_GPS_LOST);
        cruise(s, dt, dt_ms, roll_new, roll);
        return;
    }

    if (s->gps.last_ms != n->last_fix_ms) {
        float fix_dt = (float)(s->gps.last_ms - n->last_fix_ms) * 0.001f;
        n->last_fix_ms = s->gps.last_ms;
        nav_leg_update(&n->leg, s->gps.last.lat_deg, s->gps.last.lon_deg);

        float sog = s->gps.last.sog_kn;
        if (sog > 2.0f) {
            float drift = autopilot_heading_error(s->ap.heading, s->gps.last.cog_deg);
            float a = clampf(fix_dt / c->crab_tau_s, 0.0f, 1.0f);
            n->crab_deg += a * (drift - n->crab_deg);
            n->crab_deg = clampf(n->crab_deg, -45.0f, 45.0f);
        }
        float look = fmaxf(c->lookahead_min_m, c->lookahead_s * sog * 0.514444f);
        n->course_deg = nav_leg_course(&n->leg, look);
        autopilot_set_target(&s->ap, n->course_deg + n->crab_deg);

        float v = n->speed_kn;
        if (n->leg.dist_m < c->slow_radius_m) {
            float f = clampf((n->leg.dist_m - c->arrival_radius_m) /
                             (c->slow_radius_m - c->arrival_radius_m), 0.0f, 1.0f);
            v = c->approach_kn + (v - c->approach_kn) * f;
        }
        sb_set_cruise_speed(s, fminf(v, n->speed_kn));

        if (n->leg.dist_m <= c->arrival_radius_m || n->leg.along_m >= n->leg.length_m) {
            n->arrived = true;
            s->alarm = true;
            if (c->arrive_stop) {
                sb_request_mode(s, MODE_STANDBY, WHY_ARRIVED);      /* idle + neutral */
            } else {
                autopilot_set_target(&s->ap, s->ap.heading);
                sb_request_mode(s, MODE_CRUISE, WHY_ARRIVED);
            }
            return;
        }
    }
    cruise(s, dt, dt_ms, roll_new, roll);
}

/* --- step ------------------------------------------------------------------ */

void sb_step(sb_t *s)
{
    ap_inputs_t in;
    float roll = 0.0f;

    s->last_ms = s->now_ms;
    s->now_ms = hal_millis();
    uint32_t dt_ms = s->now_ms - s->last_ms;
    float dt = (float)dt_ms * 0.001f;

    uint32_t buttons = hal_get_button_presses();
    read_inputs(s, &in, buttons);
    bool roll_new = hal_read_roll_rate(&roll);

    /* 1. steering core (also detects compass timeout) */
    autopilot_update(&s->ap, &in);

    /* 2. faults and the Standby button -> STANDBY (always allowed) */
    if (s->ap.sensor_fault && !s->compass_fault_seen) {
        s->compass_fault_seen = true;
        s->alarm = true;
        if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_COMPASS_FAULT);
    }
    if (!s->ap.sensor_fault) s->compass_fault_seen = false;
    if (buttons & HAL_BTN_BIT(HAL_BTN_STANDBY_AUTO)) {
        if (sb_mode(s) != MODE_STANDBY) {
            sb_request_mode(s, MODE_STANDBY, WHY_STANDBY_BUTTON);
        } else {
            s->alarm = false;           /* in STANDBY the button acknowledges */
            s->ap.alarm = false;
        }
    }

    /* 3. what the mode wants */
    s->throttle_demand_prev = s->throttle_demand;
    s->throttle_demand = 0.0f;
    s->gear_demand = GEAR_N;
    if (sb_mode(s) == MODE_MANEUVER) maneuver(s);
    if (sb_mode(s) == MODE_CRUISE) cruise(s, dt, dt_ms, roll_new, roll);
    else if (sb_mode(s) == MODE_NAV) nav_mode(s, dt, dt_ms, roll_new, roll);
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
