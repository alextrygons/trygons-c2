/*
 * autopilot.c - Heading-hold autopilot logic.
 *
 * Two cascaded loops, both run at AP_RATE_HZ:
 *   outer: heading error  -> rudder target   (PID, limited)
 *   inner, big boat:  rudder error (sensor)   -> motor (P with deadband)
 *   inner, outboard:  rudder error (estimate) -> motor on/off pulses,
 *                     with backlash compensation
 */
#include "autopilot.h"
#include "hal.h"

#include <math.h>
#include <string.h>

/* Per sea state: wave filter on the yaw rate (D term) and rudder deadband.
 *
 * Waves make the measured yaw rate swing back and forth; the D term turns
 * that straight into rudder movement. Filtering it (rather than the heading
 * itself - that adds lag and overshoot, tested in sim) plus a wider
 * deadband lets the rudder ignore waves the boat can't fight anyway.
 * The outboard (no angle feedback, slow motor) needs a narrower deadband:
 * its own estimate error is the only thing it can react to.
 * Sea state 3 was tuned in the simulation; 2, 4 and 5 are scaled from it. */
typedef struct {
    float rate_tau_s;       /* low-pass on yaw rate (D term) */
    float deadband_on_deg;  /* motor starts when |rudder error| above this */
    float deadband_off_deg; /* motor stops when |rudder error| below this */
    float ob_deadband_on_deg; /* outboard: start a pulse when |estimate error| above this */
} sea_params_t;

static const sea_params_t k_sea[AP_SEA_STATE_MAX] = {
    /* tau_r  db_on  db_off  outboard db_on */
    {  0.5f,  1.0f,  0.3f,  1.0f },   /* 1 calm     */
    {  1.0f,  2.0f,  0.3f,  1.5f },   /* 2          */
    {  2.0f,  4.0f,  0.3f,  2.0f },   /* 3 moderate */
    {  2.5f,  5.0f,  0.3f,  2.5f },   /* 4          */
    {  3.0f,  6.0f,  0.3f,  3.0f },   /* 5 rough    */
};

static const sea_params_t *sea(const autopilot_t *ap)
{
    return &k_sea[ap->cfg.sea_state - 1u];
}

static bool is_outboard(const autopilot_t *ap)
{
    return ap->cfg.steering == AP_STEER_OUTBOARD;
}

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

/* --- configuration ------------------------------------------------------ */

static const ap_outboard_config_t k_outboard_default = {
    30.0f,  /* max_angle_deg    */
    3.0f,   /* turn_speed_deg_s */
    2.0f,   /* gap_deg          */
    100u    /* min_pulse_ms     */
};

void autopilot_default_config(ap_config_t *cfg)
{
    cfg->kp                = 1.2f;
    cfg->ki                = 0.05f;
    cfg->kd                = 1.2f;
    cfg->i_limit_deg       = 10.0f;
    cfg->i_zone_deg        = 10.0f;
    cfg->turn_rate_dps     = 4.0f;

    cfg->motor_gain        = 0.25f;  /* full speed at 4 deg rudder error */

    cfg->sea_state         = AP_SEA_STATE_DEFAULT;

    cfg->steering          = AP_STEER_RUDDER_FEEDBACK;
    cfg->outboard          = k_outboard_default;
}

void autopilot_default_config_outboard(ap_config_t *cfg)
{
    autopilot_default_config(cfg);
    cfg->steering = AP_STEER_OUTBOARD;
    /* Slower steering motor: gentler gains, slower course changes. */
    cfg->kp            = 0.8f;
    cfg->kd            = 2.5f;
    cfg->ki            = 0.05f;
    cfg->turn_rate_dps = 2.0f;
}

static bool outboard_config_valid(const ap_outboard_config_t *ob)
{
    return ob->max_angle_deg    >= 5.0f  && ob->max_angle_deg    <= 45.0f
        && ob->turn_speed_deg_s >= 0.5f  && ob->turn_speed_deg_s <= 60.0f
        && ob->gap_deg          >= 0.0f  && ob->gap_deg          <= 15.0f
        && ob->min_pulse_ms     <= 2000u;
}

static uint8_t clamp_sea_state(uint8_t s)
{
    if (s < AP_SEA_STATE_MIN) return AP_SEA_STATE_MIN;
    if (s > AP_SEA_STATE_MAX) return AP_SEA_STATE_MAX;
    return s;
}

void autopilot_set_sea_state(autopilot_t *ap, uint8_t sea_state)
{
    ap->cfg.sea_state = clamp_sea_state(sea_state);
}

bool autopilot_set_outboard_config(autopilot_t *ap, const ap_outboard_config_t *ob)
{
    if (!outboard_config_valid(ob)) return false;
    ap->cfg.outboard = *ob;
    return true;
}

static void outboard_reset(autopilot_t *ap)
{
    ap->est_angle = 0.0f;
    ap->slack_deg = 0.0f;   /* unknown side of the gap: assume the middle */
    ap->motor_dir = 0;
    ap->pulse_ms = 0u;
    ap->pause_ms = 0u;
}

/* Stop the outboard motor (the pause counter restarts). */
static void outboard_stop(autopilot_t *ap)
{
    ap->motor_dir = 0;
    ap->pause_ms = 0u;
}

void autopilot_init(autopilot_t *ap, const ap_config_t *cfg)
{
    uint32_t now = hal_millis();

    ap->cfg = *cfg;
    ap->cfg.sea_state = clamp_sea_state(cfg->sea_state);
    if (!outboard_config_valid(&ap->cfg.outboard)) {
        ap->cfg.outboard = k_outboard_default;
    }
    ap->mode = AP_MODE_STANDBY;
    ap->alarm = false;
    ap->target_heading = 0.0f;
    ap->heading = 0.0f;
    ap->rudder_angle = 0.0f;
    ap->yaw_rate = 0.0f;
    ap->ref_heading = 0.0f;
    ap->ref_rate = 0.0f;
    ap->integrator = 0.0f;
    ap->rudder_target = 0.0f;
    ap->motor_cmd = 0.0f;
    ap->motor_moving = false;
    outboard_reset(ap);
    ap->last_step_ms = now;
    ap->off_course_ms = 0u;
    ap->heading_valid = false;
    ap->rudder_valid = false;
    /* Timeouts count from startup: no data within 2 s -> alarm. */
    ap->last_heading_ms = now;
    ap->last_rudder_ms = now;
    ap->sensor_fault = false;
    ap->gyro_dps = 0.0f;
    ap->gyro_valid = false;
    ap->last_gyro_ms = now;
    ap->heading_new = false;
    memset(&ap->train, 0, sizeof ap->train);
    ap->calibrated = false;
}

/* --- sensors ------------------------------------------------------------ */

static void read_sensors(autopilot_t *ap, const ap_inputs_t *in)
{
    uint32_t now = in->now_ms;
    float h = in->heading_deg, r = in->rudder_deg, g = in->gyro_dps;

    /* Gyro (optional): used by TRAINING, compass is the backup. */
    if (in->gyro_new && fabsf(g) < 200.0f) {
        ap->gyro_dps = g;
        ap->gyro_valid = true;
        ap->last_gyro_ms = now;
    }

    /* A reading that is out of range counts as "no new data". */
    ap->heading_new = false;
    if (in->heading_new && h >= 0.0f && h < 360.0f) {
        ap->heading_new = true;
        if (ap->heading_valid) {
            float dt = (float)(now - ap->last_heading_ms) * 0.001f;
            if (dt > 0.0f) {
                float raw_rate = autopilot_heading_error(h, ap->heading) / dt;
                float alpha = dt / (sea(ap)->rate_tau_s + dt);
                ap->yaw_rate += alpha * (raw_rate - ap->yaw_rate);
            }
        } else {
            ap->yaw_rate = 0.0f;
        }
        ap->heading = h;
        ap->heading_valid = true;
        ap->last_heading_ms = now;
    }

    /* The outboard has no angle sensor: don't read or supervise it. */
    if (!is_outboard(ap) &&
        in->rudder_new && fabsf(r) <= HAL_RUDDER_MAX_DEG + 5.0f) {
        ap->rudder_angle = r;
        ap->rudder_valid = true;
        ap->last_rudder_ms = now;
    }
}

/* True if a sensor we depend on has had no new data for at least
 * limit_ms. Unsigned subtraction is correct even when hal_millis() wraps. */
static bool sensors_older_than(const autopilot_t *ap, uint32_t now, uint32_t limit_ms)
{
    if ((now - ap->last_heading_ms) >= limit_ms) return true;
    return !is_outboard(ap) && (now - ap->last_rudder_ms) >= limit_ms;
}

/* --- modes -------------------------------------------------------------- */

static void enter_standby(autopilot_t *ap)
{
    if (ap->mode == AP_MODE_TRAINING &&
        ap->train.phase > TR_IDLE && ap->train.phase < TR_RESULT) {
        training_abort(&ap->train, TR_ABORT_USER);   /* still running: stop it */
    }
    ap->mode = AP_MODE_STANDBY;
    ap->motor_moving = false;
    outboard_stop(ap);
    ap->motor_cmd = 0.0f;
}

static void enter_auto(autopilot_t *ap)
{
    ap->mode = AP_MODE_AUTO;
    ap->target_heading = ap->heading;     /* lock current heading */
    ap->ref_heading = ap->target_heading;
    ap->ref_rate = 0.0f;
    ap->integrator = 0.0f;
    ap->off_course_ms = 0u;
    ap->motor_moving = false;
    if (is_outboard(ap)) {
        /* The user has centred the outboard by hand. */
        outboard_reset(ap);
        ap->rudder_target = 0.0f;
    } else {
        ap->rudder_target = ap->rudder_angle; /* bumpless: start where it is */
    }
}

static void handle_buttons(autopilot_t *ap, uint32_t presses)
{
    bool sensors_ok = ap->heading_valid && !ap->sensor_fault &&
                      (is_outboard(ap) || ap->rudder_valid);

    if (presses & HAL_BTN_BIT(HAL_BTN_STANDBY_AUTO)) {
        if (ap->mode == AP_MODE_TRAINING) {
            /* abort training (or reject a finished result): motor off */
            training_abort(&ap->train, ap->train.phase == TR_RESULT
                                       ? TR_ABORT_REJECTED : TR_ABORT_USER);
            enter_standby(ap);
        } else if (ap->mode == AP_MODE_AUTO) {
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

static float rudder_limit(const autopilot_t *ap)
{
    if (is_outboard(ap)) return ap->cfg.outboard.max_angle_deg;
    return AP_RUDDER_LIMIT_DEG;
}

/* Outer loop: heading -> rudder target (deg). */
static void heading_loop(autopilot_t *ap)
{
    const ap_config_t *c = &ap->cfg;
    float limit = rudder_limit(ap);

    /* Course changes: steer to a reference heading that swings toward the
     * new course at a limited turn rate, instead of jumping there. */
    float to_go = autopilot_heading_error(ap->target_heading, ap->ref_heading);
    float max_turn = c->turn_rate_dps * AP_DT_S;
    if (fabsf(to_go) <= max_turn) {
        ap->ref_heading = ap->target_heading;
        ap->ref_rate = 0.0f;
    } else {
        ap->ref_heading = autopilot_wrap360(ap->ref_heading + copysignf(max_turn, to_go));
        ap->ref_rate = copysignf(c->turn_rate_dps, to_go);
    }

    float e = autopilot_heading_error(ap->ref_heading, ap->heading);

    /* Off-course alarm: steering isn't keeping up (or doesn't work). */
    if (fabsf(e) > AP_OFF_COURSE_DEG) {
        ap->off_course_ms += AP_PERIOD_MS;
        if (ap->off_course_ms >= AP_OFF_COURSE_MS) ap->alarm = true;
    } else {
        ap->off_course_ms = 0u;
    }

    float p = c->kp * e;
    float d = -c->kd * (ap->yaw_rate - ap->ref_rate);  /* damp vs. intended turn rate */
    float unsat = p + ap->integrator + d;

    /* Anti-windup: integrate only near the course, and never further
     * into saturation. Integrator itself is clamped as well. */
    bool sat_hi = unsat >=  limit && e > 0.0f;
    bool sat_lo = unsat <= -limit && e < 0.0f;
    if (fabsf(e) < c->i_zone_deg && !sat_hi && !sat_lo) {
        ap->integrator += c->ki * e * AP_DT_S;
        ap->integrator = clampf(ap->integrator, -c->i_limit_deg, c->i_limit_deg);
    }

    /* Outboard re-centring: move a little of the integrator into the angle
     * estimate. The motor command doesn't change (target and estimate both
     * shift by the same amount); it just stops the estimate drifting away
     * from the true angle. Assumes the boat needs ~0 rudder on average, so
     * the estimate ends up off by the real weather helm - bounded, unlike
     * drift. */
    if (is_outboard(ap)) {
        float shift = ap->integrator * (AP_DT_S / AP_OUTBOARD_RECENTER_TAU_S);
        ap->integrator -= shift;
        ap->rudder_target -= shift;
        ap->est_angle = clampf(ap->est_angle - shift, -limit, limit);
    }

    float cmd = clampf(p + ap->integrator + d, -limit, limit);

    /* Rate limit the rudder target. */
    float max_step = AP_RUDDER_RATE_DPS * AP_DT_S;
    ap->rudder_target += clampf(cmd - ap->rudder_target, -max_step, max_step);
    ap->rudder_target = clampf(ap->rudder_target, -limit, limit);
}

/* Inner loop, big boat: rudder target -> motor command (sensor feedback). */
static float rudder_loop(autopilot_t *ap)
{
    const ap_config_t *c = &ap->cfg;
    const sea_params_t *sp = sea(ap);
    float re = ap->rudder_target - ap->rudder_angle;

    /* Deadband with hysteresis so the motor doesn't hunt. */
    if (ap->motor_moving) {
        if (fabsf(re) < sp->deadband_off_deg) ap->motor_moving = false;
    } else {
        if (fabsf(re) > sp->deadband_on_deg) ap->motor_moving = true;
    }

    float m = ap->motor_moving ? clampf(c->motor_gain * re, -1.0f, 1.0f) : 0.0f;

    /* Never push past the mechanical limit. */
    if (ap->rudder_angle >=  HAL_RUDDER_MAX_DEG && m > 0.0f) m = 0.0f;
    if (ap->rudder_angle <= -HAL_RUDDER_MAX_DEG && m < 0.0f) m = 0.0f;
    return m;
}

/* How much of the backlash gap is still to take up before the outboard
 * moves in direction d, in degrees. */
static float slack_to_go(const autopilot_t *ap, int8_t d)
{
    float half = 0.5f * ap->cfg.outboard.gap_deg;
    return (d > 0) ? half - ap->slack_deg : ap->slack_deg + half;
}

/*
 * Outboard: account for what the motor did since the last step.
 * Motor travel = run time x turn_speed. It first takes up the backlash gap
 * (no movement of the outboard), the rest moves the angle estimate.
 * On a full reversal that is exactly gap/turn_speed of extra run time.
 * Called every step, in every mode.
 */
static void outboard_account(autopilot_t *ap, uint32_t now)
{
    const ap_outboard_config_t *ob = &ap->cfg.outboard;
    uint32_t elapsed = now - ap->last_step_ms;
    float half = 0.5f * ob->gap_deg;

    /* gap_deg may have been changed at runtime */
    ap->slack_deg = clampf(ap->slack_deg, -half, half);

    if (ap->motor_dir == 0) {
        ap->pause_ms += elapsed;
        return;
    }
    int8_t d = ap->motor_dir;
    float travel = ob->turn_speed_deg_s * (float)elapsed * 0.001f;
    float take = fminf(travel, slack_to_go(ap, d));

    ap->slack_deg += (float)d * take;
    ap->est_angle += (float)d * (travel - take);
    ap->est_angle = clampf(ap->est_angle, -ob->max_angle_deg, ob->max_angle_deg);
    ap->pulse_ms += elapsed;
}

/* Inner loop, outboard: rudder target -> motor pulses (no feedback). */
static float outboard_loop(autopilot_t *ap)
{
    const ap_outboard_config_t *ob = &ap->cfg.outboard;
    const sea_params_t *sp = sea(ap);
    float err = ap->rudder_target - ap->est_angle;

    if (ap->motor_dir != 0) {
        int8_t d = ap->motor_dir;
        bool at_limit = (d > 0) ? ap->est_angle >= ob->max_angle_deg
                                : ap->est_angle <= -ob->max_angle_deg;
        bool reached = slack_to_go(ap, d) <= 1e-4f &&   /* gap taken up */
                       (float)d * err <= sp->deadband_off_deg;
        /* At the angle limit: stop at once (safety beats the min pulse).
         * Otherwise stop once the target is reached and the pulse is long
         * enough to have really moved the motor. */
        if (at_limit || (reached && ap->pulse_ms >= ob->min_pulse_ms)) {
            outboard_stop(ap);
        }
    } else if (ap->pause_ms >= ob->min_pulse_ms &&
               fabsf(err) > sp->ob_deadband_on_deg) {
        int8_t d = (err > 0.0f) ? 1 : -1;
        bool at_limit = (d > 0) ? ap->est_angle >= ob->max_angle_deg
                                : ap->est_angle <= -ob->max_angle_deg;
        if (!at_limit) {
            ap->motor_dir = d;   /* backlash is handled in outboard_account */
            ap->pulse_ms = 0u;
        }
    }
    return (float)ap->motor_dir;
}

/* --- training / calibration ---------------------------------------------- */

/* One TRAINING step. Returns the motor command. On abort: motor off,
 * STANDBY, alarm (unless the user aborted). */
static float training_mode_step(autopilot_t *ap, uint32_t now, bool stale)
{
    training_t *t = &ap->train;

    if (t->phase == TR_RESULT) return 0.0f;      /* waiting for accept / reject */
    if (stale) training_abort(t, TR_ABORT_SENSOR);

    if (t->phase != TR_ABORTED) {
        tr_input_t in;
        in.now_ms = now;
        in.heading_deg = ap->heading;
        in.heading_new = ap->heading_new;
        in.from_gyro = ap->gyro_valid && (now - ap->last_gyro_ms) < AP_SENSOR_STALE_MS;
        in.yaw_dps = in.from_gyro ? ap->gyro_dps : ap->yaw_rate;  /* compass = backup */
        int8_t m = training_step(t, &in);
        if (t->phase != TR_ABORTED) return (float)m;
    }
    if (t->result != TR_ABORT_USER) ap->alarm = true;
    enter_standby(ap);
    return 0.0f;
}

bool autopilot_start_training(autopilot_t *ap)
{
    if (!is_outboard(ap) || ap->mode != AP_MODE_STANDBY ||
        !ap->heading_valid || ap->sensor_fault) {
        return false;
    }
    training_start(&ap->train, ap->last_step_ms, ap->heading);
    outboard_stop(ap);
    ap->alarm = false;
    ap->mode = AP_MODE_TRAINING;
    return true;
}

bool autopilot_confirm_training(autopilot_t *ap, bool accept, float full_angle_deg)
{
    if (ap->mode != AP_MODE_TRAINING || ap->train.phase != TR_RESULT) return false;

    if (accept) {
        calib_data_t c = ap->train.calib;
        c.full_angle_deg = (full_angle_deg >= 10.0f && full_angle_deg <= 120.0f)
                         ? full_angle_deg : 0.0f;
        calib_seal(&c);
        accept = calib_valid(&c);
        if (accept) {
            ap->train.calib = c;
            (void)calib_save(&c);   /* no storage yet on the board: still use it */
            autopilot_apply_calibration(ap, &c);
            accept = ap->calibrated;
        }
    }
    if (!accept) training_abort(&ap->train, TR_ABORT_REJECTED);
    enter_standby(ap);
    return accept;
}

void autopilot_apply_calibration(autopilot_t *ap, const calib_data_t *c)
{
    ap_config_t d;
    autopilot_default_config_outboard(&d);

    /* Degrees per ms of motor run: only known if the user measured the
     * lock-to-lock angle. Otherwise keep the configured (nominal) speed;
     * the loop gain is still right because the gains are scaled below. */
    float travel_ms = c->travel_left_ms + c->travel_right_ms;
    float speed = (c->full_angle_deg > 0.0f) ? c->full_angle_deg / (travel_ms * 0.001f)
                                             : ap->cfg.outboard.turn_speed_deg_s;

    ap_outboard_config_t ob = ap->cfg.outboard;
    ob.turn_speed_deg_s = speed;
    ob.gap_deg = c->gap_ms * speed * 0.001f;
    ob.max_angle_deg = clampf(fminf(c->travel_left_ms, c->travel_right_ms) * speed * 0.001f
                              - AP_CALIB_MARGIN_DEG, 5.0f, 45.0f);
    if (!outboard_config_valid(&ob)) {
        ap->calibrated = false;
        return;
    }

    /* Gain scheduling (see AP_CALIB_REF_K): same loop gain as the boat the
     * gains were tuned on, and a turn rate this boat can actually follow. */
    float k = c->gain_dps_per_ms * 1000.0f / speed;     /* deg/s yaw per deg */
    float scale = clampf(AP_CALIB_REF_K / k, 0.25f, 4.0f);
    float turn = fminf(d.turn_rate_dps, AP_CALIB_TURN_FRACTION * k * ob.max_angle_deg);

    ap->cfg.outboard = ob;
    ap->cfg.kp = d.kp * scale;
    ap->cfg.ki = d.ki * scale;
    ap->cfg.kd = d.kd * scale;
    ap->cfg.turn_rate_dps = fmaxf(0.5f, turn);
    ap->calibrated = true;
}

bool autopilot_load_calibration(autopilot_t *ap)
{
    calib_data_t c;
    if (!is_outboard(ap) || !calib_load(&c)) return false;
    autopilot_apply_calibration(ap, &c);
    return ap->calibrated;
}

bool autopilot_engage(autopilot_t *ap, bool keep_estimate)
{
    bool sensors_ok = ap->heading_valid && !ap->sensor_fault &&
                      (is_outboard(ap) || ap->rudder_valid);
    if (!sensors_ok) return false;
    if (ap->mode == AP_MODE_TRAINING) return false;
    float est = ap->est_angle, slack = ap->slack_deg;
    enter_auto(ap);
    if (keep_estimate && is_outboard(ap)) {
        /* outboard not centred by hand (e.g. coming from manual steering) */
        ap->est_angle = est;
        ap->slack_deg = slack;
        ap->rudder_target = est;
    }
    return true;
}

void autopilot_disengage(autopilot_t *ap)
{
    enter_standby(ap);
    ap->motor_cmd = 0.0f;
}

void autopilot_set_target(autopilot_t *ap, float heading_deg)
{
    ap->target_heading = autopilot_wrap360(heading_deg);
}

void autopilot_update(autopilot_t *ap, const ap_inputs_t *in)
{
    uint32_t now = in->now_ms;

    if (is_outboard(ap)) outboard_account(ap, now);
    ap->last_step_ms = now;

    read_sensors(ap, in);

    /* Sensor timeout: STANDBY + alarm, once at the moment it happens. */
    bool fault = sensors_older_than(ap, now, AP_SENSOR_TIMEOUT_MS);
    if (fault && !ap->sensor_fault) {
        if (ap->mode == AP_MODE_TRAINING) training_abort(&ap->train, TR_ABORT_SENSOR);
        enter_standby(ap);
        ap->alarm = true;
    }
    ap->sensor_fault = fault;

    handle_buttons(ap, in->buttons);

    /* Stale data (0.5 s): stop the motor so the rudder holds where it is,
     * and don't touch the PID state, until fresh data arrives or the
     * 2 s timeout above drops us to STANDBY. */
    bool stale = sensors_older_than(ap, now, AP_SENSOR_STALE_MS);

    if (ap->mode == AP_MODE_TRAINING) {
        ap->motor_cmd = training_mode_step(ap, now, stale);
    } else if (ap->mode == AP_MODE_AUTO && stale) {
        ap->motor_moving = false;
        outboard_stop(ap);
        ap->motor_cmd = 0.0f;
    } else if (ap->mode == AP_MODE_AUTO) {
        heading_loop(ap);
        ap->motor_cmd = is_outboard(ap) ? outboard_loop(ap) : rudder_loop(ap);
    } else {
        outboard_stop(ap);
        ap->motor_cmd = 0.0f;
    }
}

void autopilot_step(autopilot_t *ap)
{
    /* Read the hardware (same order as always), compute, write outputs. */
    ap_inputs_t in;
    memset(&in, 0, sizeof in);
    in.now_ms = hal_millis();
    in.gyro_new = hal_read_yaw_rate(&in.gyro_dps);
    in.heading_new = hal_read_heading(&in.heading_deg);
    if (!is_outboard(ap)) in.rudder_new = hal_read_rudder_angle(&in.rudder_deg);
    in.buttons = hal_get_button_presses();

    autopilot_update(ap, &in);

    hal_set_rudder_motor(ap->motor_cmd);
    hal_set_alarm(ap->alarm);
}
