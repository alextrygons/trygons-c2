/*
 * smallboat.c - Small-boat controller. See smallboat.h.
 */
#include "smallboat.h"

#include <math.h>

#include "monitor.h"
#include "timing.h"
#include "logger.h"
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
    plim_default_config(&cfg->plim);
    servo_default_config(&cfg->servo);
    cfg->gps_timeout_ms = 3000u;
    cfg->fault_steer = SB_FAULT_STEER_GYRO;

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

    cfg->slow_radius_m = 150.0f;
    cfg->approach_kn = 4.0f;
    cfg->lookahead_min_m = 40.0f;
    cfg->lookahead_s = 6.0f;
    cfg->crab_tau_s = 20.0f;
    cfg->prog.max_speed_kn = cfg->cruise_max_kn;
    cfg->prog.min_speed_kn = 1.0f;
    cfg->prog.min_radius_m = 5.0f;
    cfg->prog.max_radius_m = 500.0f;
    cfg->prog.min_leg_m = 10.0f;
    cfg->prog.max_leg_m = 50000.0f;
    cfg->confirm_ms = 10000u;
    cfg->arm_ms = 500u;
    cfg->arm_alarm_ms = 3000u;
    cfg->joy_optional = false;
    cfg->first_test = false;
    cfg->first_test_min_kn = 2.0f;
    cfg->first_test_cog_deg = 60.0f;
    cfg->bench_idle_ms = 600000u;
    cfg->bench_pulse_min_ms = 100u;            /* = the steering core's min_pulse_ms */
    cfg->bench_pulse_max_ms = 300u;
    cfg->bench_gap_ms = 200u;
    cfg->bench_buzz_max_ms = 3000u;
    cfg->bench_max_kn = 1.5f;

    cfg->training_throttle_pct = 20.0f;
    cfg->training_spinup_ms = 30000u;

    dr_default_config(&cfg->dr);
    cfg->dr_speed_factor = 0.7f;
    cfg->dr_max_unc_m = 200.0f;
    cfg->dr_max_ms = 600000u;
    cfg->dr_slow_kn = 4.0f;

    joy_default_config(&cfg->joy);
    cfg->joy_max_throttle_pct = 100.0f;
    slam_default_config(&cfg->slam);
    cfg->nav_lost_kn = 4.0f;
    cfg->nav_max_dev_deg = 150.0f;
    jb_default_config(&cfg->jb);
    cfg->remote_cap_fwd_pct = 20.0f;
    cfg->remote_cap_rev_pct = 5.0f;
    cfg->remote_steer_rate_dps = 20.0f;
    cfg->remote_max_age_ms = 300u;
    cfg->remote_max_lat_ms = 400u;
    cfg->remote_lost_ms = 5000u;
}

static bool lever_has_throttle(sb_mode_t m);
static void lever_pickup_start(sb_t *s, sb_mode_t from);
static void remote_enter(sb_t *s, sb_mode_t from);
static float remote_steer(sb_t *s, float dt);

static void lever_standard(sb_t *s)
{
    s->lev.pickup = false;
    s->lev.have_prev = false;
    s->lev.fwd.u = s->lev.rev.u = 1.0f;
    s->lev.fwd.d = s->lev.fwd.lim = s->plim.fwd_pct;
    s->lev.rev.d = s->lev.rev.lim = s->plim.rev_pct;
}

/* Use the joystick map: lever settings into the joystick, bridge input
 * restarted, joystick disarmed until it is centred again. */
static void joymap_apply(sb_t *s)
{
    s->joy.cfg.nz_lo = s->jmap.nz_lo;
    s->joy.cfg.nz_hi = s->jmap.nz_hi;
    s->joy.cfg.lever_delta = s->jmap.lever_delta;
    jb_init(&s->jb, &s->cfg.jb, s->now_ms);
    joy_disarm(&s->joy);
    lever_standard(s);
}

void sb_init(sb_t *s, const sb_config_t *cfg)
{
    static program_t p;         /* large: not on the stack */
    uint8_t bad;

    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->now_ms = s->last_ms = hal_millis();
    mode_init(&s->modes, s->now_ms);
    autopilot_init(&s->ap, &cfg->steer);
    s->ap.gyro_assist = (cfg->fault_steer == SB_FAULT_STEER_GYRO);
    gear_init(&s->gear, &cfg->gear);
    throttle_init(&s->thr, &cfg->throttle);
    slam_init(&s->slam, &cfg->slam);
    plim_init(&s->plim, &cfg->plim, s->now_ms * 2654435761u + 12345u);     /* always 30 / 5 */
    hal_set_safety_relay(false);                /* closed once armed (interlock) */
    servo_init(&s->servo, &cfg->servo);
    s->gear_demand = GEAR_N;
    dr_init(&s->dr, &cfg->dr);
    s->dr.cfg.gps_timeout_ms = cfg->gps_timeout_ms;
    {
        joy_cal_t cal;
        bool have = hal_nv_load(HAL_NV_SLOT_JOYSTICK, &cal, (uint32_t)sizeof cal) && joy_cal_valid(&cal);
        joy_init(&s->joy, &cfg->joy, have ? &cal : NULL);
    }
    if (!(hal_nv_load(HAL_NV_SLOT_JOYMAP, &s->jmap, (uint32_t)sizeof s->jmap) && joymap_valid(&s->jmap)))
        joymap_default(&s->jmap);
    joymap_apply(s);
    if (hal_nv_load(HAL_NV_SLOT_PROGRAM, &p, (uint32_t)sizeof p) &&
        program_validate(&p, &s->cfg.prog, &bad) == PROG_OK) {
        s->autop.prog = p;
        s->autop.prog_ok = true;
    }
}

bool sb_compass_ok(const sb_t *s)
{
    return s->ap.heading_valid && !s->ap.sensor_fault;
}

/* GPS usable = fresh fix AND the integrity check trusts it (dr.c). */
bool sb_gps_ok(const sb_t *s)
{
    return s->gps.valid && s->gps.last.fix &&
           (s->now_ms - s->gps.last_ms) < s->cfg.gps_timeout_ms &&
           s->dr.state == DR_GPS;
}

void sb_alarm_ack(sb_t *s)
{
    uint32_t keep = 0u;
    if (s->ap.sensor_fault) keep |= SB_ALM_COMPASS;
    if (!sb_gps_ok(s)) keep |= SB_ALM_GPS_LOST | SB_ALM_GPS_SUSPECT;
    if (s->joy.fault) keep |= SB_ALM_JOY_FAULT;
    if (s->joy.not_centered && !s->joy.armed) keep |= SB_ALM_JOY_NOT_CENTERED;
    if (!s->armed) keep |= SB_ALM_NOT_ARMED;
    if (s->mon_tripped) keep |= SB_ALM_MONITOR;
    if (g_logstat.failure) keep |= SB_ALM_LOG;
    if (gear_fault(&s->gear)) keep |= SB_ALM_GEAR;
    if (s->gw.flags & (SB_GWF_STORAGE | SB_GWF_STORAGE_FULL)) keep |= SB_ALM_GW_STORAGE;
    if (s->gw.flags & SB_GWF_AIS_SILENT) keep |= SB_ALM_AIS_SILENT;
    if (s->gw.flags & SB_GWF_FALLBACK) keep |= SB_ALM_CHANNEL;
    if (s->gw.state == SB_GW_LOST) keep |= SB_ALM_GATEWAY;
    if (s->nav.lost) keep |= SB_ALM_DECISION;
    if (s->estop_state == 2u) keep |= SB_ALM_ESTOP;
    s->alarm_bits &= keep;              /* (the link-lost bits are raised again while lost, link.c) */
    s->alarm = s->alarm_bits != 0u;
    if (!s->ap.sensor_fault) s->ap.alarm = false;
}

#ifdef SB_TEST_HOOKS
void sb_test_drive(sb_t *s, bool on, gear_t gear, float throttle_pct)
{
    s->test_drive.on = on;
    s->test_drive.gear = gear;
    s->test_drive.throttle_pct = throttle_pct;
}
#endif

/* AUTO speed wanted (leg speed, slow-downs); rough water may lower it. */
static void set_speed(sb_t *s, float kn)
{
    float v = clampf(kn, 0.0f, s->cfg.cruise_max_kn);
    bool raised = v > s->cruise.set_kn + 0.01f;     /* a new, higher wanted speed (next leg) */
    s->cruise.set_kn = v;
    /* lower: at once. Higher: at once only for a new wanted speed in calm
     * water; otherwise the rough-water recovery raises it slowly (called
     * on every GPS fix, it must not undo a slow-down) */
    if (raised && !s->nerv.nervous && s->nerv.ceiling_ms == 0u) s->cruise.target_kn = v;
    else s->cruise.target_kn = fminf(s->cruise.target_kn, v);
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

/* Speed for the steering limit / gain scaling: GPS, else the estimate. */
static float speed_now(const sb_t *s)
{
    if (sb_gps_ok(s)) return s->gps.last.sog_kn;
    return dr_have_position(&s->dr) ? dr_sog_kn(&s->dr) : 0.0f;
}

static void steer_by_speed(sb_t *s, float v)
{
    s->ap.steer_limit_deg = sb_steer_limit_for(&s->cfg, v);
    s->ap.gain_scale = clampf(s->cfg.gain_ref_kn / fmaxf(v, 0.5f), s->cfg.gain_scale_min, s->cfg.gain_scale_max);
}

static void alarm_on(sb_t *s, uint32_t bits)
{
    s->alarm = true;
    s->alarm_bits |= bits;
}

/* --- gateway navigation request (section D) --------------------------------- */

static bool nav_fresh(const sb_t *s)
{
    return s->nav.rx_ms != 0u && s->now_ms - s->nav.rx_ms <= s->nav.lease_ms;
}

/* Speed cap (kn) the navigation request puts on AUTO; < 0 = none. */
static float nav_cap(const sb_t *s)
{
    if (sb_mode(s) != MODE_AUTO) return -1.0f;
    if (s->nav.lost) {                  /* never faster than the last cap: a slowdown stays */
        float c = s->cfg.nav_lost_kn;
        return (s->nav.lost_cap_kn >= 0.0f && s->nav.lost_cap_kn < c) ? s->nav.lost_cap_kn : c;
    }
    if (nav_fresh(s) && (s->nav.kind == SB_NAV_CAP || s->nav.kind == SB_NAV_HEADING)) return s->nav.cap_kn;
    return -1.0f;
}

/* AUTO: the avoidance heading (or the held heading when the request was lost). */
static void nav_steer(sb_t *s)
{
    if (sb_mode(s) != MODE_AUTO) return;
    if (s->nav.lost) {
        autopilot_set_target(&s->ap, s->nav.lost_heading);
    } else if (nav_fresh(s) && s->nav.kind == SB_NAV_HEADING) {
        /* the Pi may turn the boat away from the leg, but not anywhere: too far
         * off the track -> not steered (the cap still slows), DRIVER DECISION */
        if (fabsf(autopilot_heading_error(s->autop.leg.track_deg, s->nav.heading)) <= s->cfg.nav_max_dev_deg) {
            autopilot_set_target(&s->ap, s->nav.heading);
        } else {
            if (s->nav.too_far_seq != s->nav.seq || !s->nav.too_far) s->nav.too_far++;
            s->nav.too_far_seq = s->nav.seq;
            alarm_on(s, SB_ALM_DECISION);
        }
    }
}

static float auto_target_kn(const sb_t *s)
{
    float cap = nav_cap(s);
    return (cap >= 0.0f && cap < s->cruise.target_kn) ? cap : s->cruise.target_kn;
}

static void nav_step(sb_t *s)
{
    sb_navreq_t *n = &s->nav;
    bool fresh = nav_fresh(s);
    if (sb_mode(s) != MODE_AUTO) n->lost = false;
    if (fresh) {
        n->lost = false;
        if (n->alarm & 1u) alarm_on(s, SB_ALM_COLLISION);
        if (n->alarm & 2u) alarm_on(s, SB_ALM_DECISION);
        if ((n->alarm & 4u) && sb_mode(s) == MODE_AUTO) alarm_on(s, SB_ALM_SHALLOW);
        if (n->alarm & 8u) alarm_on(s, SB_ALM_OBSTACLE);
        if (n->kind == SB_NAV_HDG_SET && (!n->hdg_any || n->seq != n->hdg_seq_done) && sb_mode(s) == MODE_HDG_LOCK &&
            n->rx_mode == (uint8_t)MODE_HDG_LOCK && !s->hdg_compass_lost) {
            n->hdg_seq_done = n->seq;
            n->hdg_any = true;
            n->applied_hdg++;
            s->hdg_target = n->heading;
            autopilot_set_target(&s->ap, n->heading);
        }
    } else if (sb_mode(s) == MODE_AUTO && !n->lost && n->rx_ms != 0u &&
               (n->kind == SB_NAV_CAP || n->kind == SB_NAV_HEADING)) {
        /* the gateway went quiet in the middle of an avoidance: don't carry on blindly */
        n->lost = true;
        n->losses++;
        n->lost_heading = s->ap.heading;
        n->lost_cap_kn = n->cap_kn;
        alarm_on(s, SB_ALM_DECISION);
    }
}

/* --- video SAVE requests (gateway, section F) ------------------------------ */

static void save_req(sb_t *s, uint8_t trig, uint8_t arg)
{
    sb_save_t *v = &s->save;
    if (v->seq == 0u || s->now_ms - v->t_ms >= 2000u) v->kinds = 0u;
    v->kinds |= (uint8_t)(1u << trig);
    v->seq = (uint16_t)(v->seq + 1u);
    if (v->seq == 0u) v->seq = 1u;
    v->trig = trig;
    v->arg = arg;
    v->t_ms = s->now_ms;
}

static bool fault_why(mode_reason_t w)
{
    return w == WHY_COMPASS_FAULT || w == WHY_GPS_LOST || w == WHY_GEAR_FAULT || w == WHY_DR_LIMIT ||
           w == WHY_JOY_FAULT || w == WHY_NOT_ARMED || w == WHY_MONITOR || w == WHY_REMOTE_LOST ||
           w == WHY_TRAINING_ABORT;
}

/* Rising edges only; the AURIX never waits for the Pi. */
static void save_triggers(sb_t *s)
{
    sb_save_t *v = &s->save;
    bool gf = gear_fault(&s->gear), cf = s->ap.sensor_fault, jf = s->joy.fault, mf = s->mon_tripped;
    if (!v->init) {
        v->init = true;
        v->k_alarms = s->alarm_bits;
        v->k_modes = s->modes.log_count;
        v->k_overruns = g_timing.core0.overruns;
        v->k_gear = gf; v->k_compass = cf; v->k_joy = jf; v->k_mon = mf;
        return;
    }
    if (s->joy.save_edge) save_req(s, SB_SAVE_BUTTON, 0u);
    if (s->slam.red_edge) save_req(s, SB_SAVE_SLAM, 0u);
    uint32_t on = s->alarm_bits & ~v->k_alarms & ~SB_ALM_ARRIVED & ~SB_ALM_SLAM &
                  ~SB_ALM_AIS_SILENT;                         /* arriving / AIS silent are no incidents */
    if (on) {
        uint8_t bit = 0u;
        while (!(on & (1u << bit))) bit++;
        save_req(s, SB_SAVE_ALARM, bit);
    }
    v->k_alarms = s->alarm_bits;
    if (gf && !v->k_gear) save_req(s, SB_SAVE_FAULT, 1u);
    if (cf && !v->k_compass) save_req(s, SB_SAVE_FAULT, 2u);
    if (jf && !v->k_joy) save_req(s, SB_SAVE_FAULT, 3u);
    if (mf && !v->k_mon) save_req(s, SB_SAVE_FAULT, 4u);
    if (g_timing.core0.overruns != v->k_overruns) save_req(s, SB_SAVE_FAULT, 5u);
    v->k_gear = gf; v->k_compass = cf; v->k_joy = jf; v->k_mon = mf;
    v->k_overruns = g_timing.core0.overruns;
    if (s->modes.log_count - v->k_modes > MODE_LOG_N) v->k_modes = s->modes.log_count - MODE_LOG_N;
    while (v->k_modes < s->modes.log_count) {
        const mode_log_entry_t *e = &s->modes.log[v->k_modes % MODE_LOG_N];
        v->k_modes++;
        if (e->verdict != MODE_OK) continue;
        if (e->to == MODE_STANDBY && fault_why((mode_reason_t)e->why))
            save_req(s, SB_SAVE_STANDBY_FAULT, (uint8_t)e->why);
        else if (e->to == MODE_MANUAL && e->why == WHY_JOYSTICK &&
                 (e->from == MODE_HDG_LOCK || e->from == MODE_AUTO || e->from == MODE_REMOTE ||
                  e->from == MODE_TRAINING))
            save_req(s, SB_SAVE_TAKEOVER, (uint8_t)e->from);
    }
}

/* --- modes ---------------------------------------------------------------- */

static bool joy_ready(const sb_t *s)
{
    return s->joy.present && s->joy.armed && !s->joy.fault;
}

/* The active waypoint can be reached from here (position known, leg not
 * absurdly long). */
static bool auto_leg_ok(const sb_t *s)
{
    const sb_auto_t *a = &s->autop;
    double lat, lon;
    if (!a->prog_ok || a->wp >= a->prog.n || !dr_have_position(&s->dr)) return false;
    dr_latlon(&s->dr, &lat, &lon);
    const prog_wp_t *w = &a->prog.wp[a->wp];
    return program_dist_m(lat, lon, program_lat(w), program_lon(w)) <= s->cfg.prog.max_leg_m;
}

/* BENCH TEST: the GPS says the boat moves (trusted fix, SOG above bench_max_kn) */
static bool bench_moving(const sb_t *s)
{
    return sb_gps_ok(s) && s->gps.last.sog_kn > s->cfg.bench_max_kn;
}

static bool precondition(const sb_t *s, sb_mode_t to, mode_reason_t why)
{
    bool forced = why == WHY_STANDBY_BUTTON || why == WHY_JOY_FAULT || why == WHY_GEAR_FAULT ||
                  why == WHY_NOT_ARMED || why == WHY_MONITOR || why == WHY_ESTOP;
    if (s->estop_state == 2u && to != MODE_STANDBY) return false;     /* emergency stop pressed */
    /* FIRST TEST: steering only - STANDBY, TRAINING, HDG LOCK */
    if (s->cfg.first_test && (to == MODE_MANUAL || to == MODE_AUTO || to == MODE_REMOTE)) return false;

    /* The joystick always wins: MANUAL is left only with the stick centred
     * (AUTO / HDG LOCK button, PC + CONFIRM), the STANDBY button or a fault. */
    if (sb_mode(s) == MODE_MANUAL && !forced) {
        if (why != WHY_JOY_HANDBACK && why != WHY_HDG_BUTTON && why != WHY_CONFIRM) return false;
        if (!joy_centered(&s->joy)) return false;
    }
    if ((!s->armed || s->mon_tripped) && to != MODE_STANDBY) return false;   /* interlock / monitor */
    if (to == MODE_MANUAL) return why == WHY_JOYSTICK;      /* only the joystick enters it */
    /* a held stick (also one ignored since STANDBY) must always be able to
     * take over: no autopilot / remote / training mode while it is held */
    if (to != MODE_STANDBY && s->joy.off_centre) return false;
    if (to == MODE_REMOTE) return why == WHY_REMOTE && !gear_fault(&s->gear);   /* HELM request + CONFIRM */

    switch (to) {
    case MODE_STANDBY:
        return true;
    case MODE_HDG_LOCK:     /* physical button (or joystick AUTO back to it), never the PC alone */
        return (why == WHY_HDG_BUTTON || why == WHY_JOY_HANDBACK) &&
               sb_compass_ok(s) && !gear_fault(&s->gear);
    case MODE_AUTO:         /* PC request + joystick CONFIRM */
        return why == WHY_CONFIRM && sb_compass_ok(s) && sb_gps_ok(s) &&
               !gear_fault(&s->gear) && auto_leg_ok(s);
    case MODE_TRAINING:
        return (why == WHY_PC || why == WHY_USER || why == WHY_TEST) &&
               sb_compass_ok(s) && s->ap.mode != AP_MODE_TRAINING && !gear_fault(&s->gear);
    case MODE_BENCH:        /* the USB laptop (link.c), boat not moving; no sensor needed (it shows them) */
        return why == WHY_PC && !bench_moving(s) && s->ap.mode != AP_MODE_TRAINING;
    default:
        return false;
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

/* Compass lost while moving: gyro hold (zero yaw rate); no gyro either:
 * outboard to its estimated centre. */
static void start_fault_steer(sb_t *s)
{
    if (autopilot_gyro_ok(&s->ap)) {
        autopilot_gyro_hold(&s->ap);
        s->stop_steer = STOP_STEER_GYRO;
    } else {
        autopilot_center(&s->ap);
        s->stop_steer = STOP_STEER_CENTER;
    }
}

/* Compass lost while moving, as configured: gyro hold (default), or the
 * outboard to its centre (SB_FAULT_STEER_CENTER) - never just off. */
static void fault_steer_start(sb_t *s)
{
    if (s->cfg.fault_steer == SB_FAULT_STEER_GYRO) {
        start_fault_steer(s);
    } else {
        autopilot_center(&s->ap);
        s->stop_steer = STOP_STEER_CENTER;
    }
}

/* New leg from where the boat is now to the active waypoint. */
static void auto_begin_leg(sb_t *s)
{
    sb_auto_t *a = &s->autop;
    double lat, lon;
    const prog_wp_t *w = &a->prog.wp[a->wp];
    dr_latlon(&s->dr, &lat, &lon);
    nav_leg_init(&a->leg, lat, lon, program_lat(w), program_lon(w));
    a->crab_deg = 0.0f;
    a->in_dr = !sb_gps_ok(s);
    a->next_dr_ms = s->now_ms;
    a->arrival_uncertain = false;
    a->last_fix_ms = s->gps.last_ms;
    a->course_deg = a->leg.track_deg;
    a->state = AUTO_RUN;
    autopilot_set_target(&s->ap, a->leg.track_deg);
    set_speed(s, w->speed_kn);
}

static void on_enter(sb_t *s, sb_mode_t from, sb_mode_t to)
{
    /* speed dependent steering only in the auto-steering modes (a safe
     * stop keeps the values: gyro hold uses the speed-scaled gain) */
    if (to != MODE_STANDBY) {
        s->ap.steer_limit_deg = 0.0f;
        s->ap.gain_scale = 1.0f;
    }
    if (from == MODE_AUTO && !s->autop.finished) s->autop.suspended = true;   /* resume: PC + CONFIRM */
    if (from == MODE_REMOTE) s->remote.state = SB_REMOTE_OFF;
    bool hdg_fault_steer = s->hdg_compass_lost &&
                           (s->stop_steer == STOP_STEER_GYRO || s->stop_steer == STOP_STEER_CENTER);
    s->hdg_compass_lost = false;
    s->hdg_reverse = false;
    if (s->joy.has_lever) {
        if (lever_has_throttle(to) && !lever_has_throttle(from)) lever_pickup_start(s, from);
        if (!lever_has_throttle(to) && lever_has_throttle(from)) joy_lever_ref_here(&s->joy);   /* hand-back position */
    }

    switch (to) {
    case MODE_STANDBY: {
        /* safe stop: throttle idle + neutral first (below), steering last */
        bool moving = s->thr.out_pct > s->cfg.gear.idle_max_pct || !gear_is(&s->gear, GEAR_N);
        bool was_auto = (from == MODE_HDG_LOCK || from == MODE_AUTO || from == MODE_TRAINING ||
                         from == MODE_REMOTE);
        s->stop_from_auto = was_auto;
        s->stopping = true;
        if (hdg_fault_steer) break;                 /* keep the gyro hold / centring running */
        s->stop_steer = STOP_STEER_NONE;
        if (from == MODE_MANUAL && moving) {
            /* the onboard stick keeps steering until idle + neutral (a held
             * stick is disarmed for taking control, not for this); stick
             * gone (fault): gyro hold / centring */
            if (s->joy.steer_ok) s->stop_steer = STOP_STEER_STICK;
            else start_fault_steer(s);
        } else if (from == MODE_REMOTE && moving) {
            start_fault_steer(s);                   /* remote stick ignored from now: gyro holds */
        } else if (s->ap.mode == AP_MODE_AUTO && sb_compass_ok(s)) {
            s->stop_steer = STOP_STEER_HEADING;
            autopilot_set_target(&s->ap, s->ap.heading);
        } else if (!sb_compass_ok(s) && moving && was_auto) {
            fault_steer_start(s);
        }
        break;
    }
    case MODE_HDG_LOCK:
        s->stopping = false;
        autopilot_engage(&s->ap, from == MODE_MANUAL || from == MODE_AUTO);
        autopilot_set_target(&s->ap, s->ap.heading);        /* lock the current heading */
        break;
    case MODE_AUTO:
        s->stopping = false;
        if (s->ap.mode != AP_MODE_AUTO) autopilot_engage(&s->ap, from == MODE_MANUAL || from == MODE_HDG_LOCK);
        s->autop.suspended = false;
        cruise_start(s);
        auto_begin_leg(s);
        break;
    case MODE_MANUAL:
        /* override: steering follows the stick (estimate kept), a running
         * training is aborted, chirp */
        s->stopping = false;
        s->manual_from = from;
        plim_drop(&s->plim, PLIM_SRC_OVERRIDE, s->now_ms);
        autopilot_angle(&s->ap);
        s->chirp_ms = 300u;
        if (from == MODE_HDG_LOCK || from == MODE_AUTO || from == MODE_TRAINING)
            s->alarm_bits |= SB_ALM_JOYSTICK;
        break;
    case MODE_REMOTE:
        /* the shore joystick: angle steering like MANUAL, its own lever / caps */
        s->stopping = false;
        s->manual_from = from;
        autopilot_angle(&s->ap);
        remote_enter(s, from);
        break;
    case MODE_BENCH:
        /* dock test: throttle idle + neutral (no demand), the steering motor
         * only in pulses (manual steering of the core: estimate, max angle and
         * end stops respected) */
        s->stopping = false;
        s->confirm.kind = CONF_NONE;
        autopilot_manual(&s->ap, true);
        memset(&s->bench, 0, sizeof s->bench);
        s->bench.last_cmd_ms = s->now_ms;
        break;
    case MODE_TRAINING:
        /* outboard centred by hand; steering off until calibration starts */
        s->stopping = false;
        plim_drop(&s->plim, PLIM_SRC_TRAINING, s->now_ms);
        autopilot_disengage(&s->ap);
        memset(&s->train, 0, sizeof s->train);
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
    mode_verdict_t v = mode_request(&s->modes, to, why, precondition(s, to, why), s->now_ms);
    if (v != MODE_OK) return false;
    if (to == MODE_STANDBY) s->stop_planned = (why == WHY_ARRIVED);   /* arrival: the normal closing rate */
    on_enter(s, from, to);
    return true;
}

/* --- inputs ---------------------------------------------------------------- */

/* Joystick, from its source (always read: it must be able to take over
 * from any mode). Analog: wires + ADC; USB: bridge frames + map. */
static void joystick_read(sb_t *s, uint32_t dt_ms)
{
    if (s->jmap.source == JOY_SRC_BRIDGE) {
        joy_input_t u;
        jb_step(&s->jb, &s->jmap, s->now_ms, &u);
        joy_update(&s->joy, &u, dt_ms);
    } else {
        hal_joystick_t js;
        bool ok = hal_read_joystick(&js);
        joy_step(&s->joy, ok ? &js : NULL, dt_ms);
    }
}

/* X -> outboard angle over the full range (deadband around centre) */
static float stick_angle(const sb_t *s, float x)
{
    float db = s->joy.cfg.deadband;
    float xs = (fabsf(x) <= db) ? 0.0f : (x - copysignf(db, x)) / (1.0f - db);
    return xs * s->ap.cfg.outboard.max_angle_deg;
}

static void read_inputs(sb_t *s, ap_inputs_t *in, uint32_t buttons)
{
    hal_gps_t g;

    memset(in, 0, sizeof *in);
    in->now_ms = s->now_ms;
    in->gyro_new = hal_read_yaw_rate(&in->gyro_dps);
    in->heading_new = hal_read_heading(&in->heading_deg);
    in->current_ok = hal_read_motor_current(&in->current_a);
    in->limits_ok = hal_read_limit_switches(&in->limit_left, &in->limit_right);

    /* course buttons go to the steering core in HDG_LOCK; STANDBY and
     * HDG LOCK are handled here (mode manager), never in the core */
    if (sb_mode(s) == MODE_HDG_LOCK && !s->hdg_reverse && !s->hdg_compass_lost)
        in->buttons = buttons & (HAL_BTN_BIT(HAL_BTN_MINUS_1) | HAL_BTN_BIT(HAL_BTN_PLUS_1) |
                                 HAL_BTN_BIT(HAL_BTN_MINUS_10) | HAL_BTN_BIT(HAL_BTN_PLUS_10));

    s->gps_new_msg = false;
    s->gps_bad_msg = false;
    if (hal_read_gps(&g)) {
        s->gps_new_msg = true;
        if (g.fix && !dr_fix_valid(&g)) {
            /* NaN / impossible numbers: a bad fix (dr.c rejects it, GPS
             * failure); the numbers are never used anywhere */
            s->gps_bad_msg = true;
            s->gps.last.fix = false;
        } else {
            s->gps.last = g;
            s->gps.valid = true;
            if (g.fix) s->gps.last_ms = s->now_ms;
        }
    }

    if (sb_mode(s) != MODE_MANUAL && s->hdg_reverse) in->angle_cmd_deg = s->hold_angle;   /* hold still */
    if (sb_mode(s) == MODE_HDG_LOCK && s->hdg_reverse && !s->hdg_compass_lost) {
        /* heading hold paused: the course buttons still move the locked heading */
        float dlt = 0.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_MINUS_1))  dlt -= 1.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_PLUS_1))   dlt += 1.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_MINUS_10)) dlt -= 10.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_PLUS_10))  dlt += 10.0f;
        s->hdg_target = autopilot_wrap360(s->hdg_target + dlt);
        autopilot_set_target(&s->ap, s->hdg_target);         /* shown as the target */
    }
    if (sb_mode(s) == MODE_REMOTE) in->angle_cmd_deg = remote_steer(s, (float)(s->now_ms - s->last_ms) * 0.001f);
    if (sb_mode(s) == MODE_MANUAL) in->angle_cmd_deg = stick_angle(s, s->joy.x);
    if (sb_mode(s) == MODE_STANDBY && s->stop_steer == STOP_STEER_STICK && s->joy.steer_ok)
        in->angle_cmd_deg = stick_angle(s, s->joy.steer_x);     /* safe stop from MANUAL */
}

/* --- joystick throttle (MANUAL and HDG_LOCK) ---------------------------------- */

/* Y -> throttle: up = forward, down = reverse, centre = idle + neutral.
 * Direction changes go through the gear state machine; reverse limit and
 * rate limits apply as everywhere (throttle.c). */
static void lever_throttle(sb_t *s);

static void joystick_throttle(sb_t *s)
{
    if (s->joy.has_lever) {
        lever_throttle(s);
        return;
    }
    float y = s->joy.y, db = s->joy.cfg.deadband;
    float mag = (fabsf(y) <= db) ? 0.0f : (fabsf(y) - db) / (1.0f - db);
    if (y > db)       s->gear_demand = GEAR_F;
    else if (y < -db) s->gear_demand = GEAR_R;
    else              s->gear_demand = GEAR_N;
    s->throttle_demand = mag * s->cfg.joy_max_throttle_pct;
}

/* --- USB throttle lever (non-returning) ---------------------------------------- */

/* Throttle for travel u on one side: 0 -> 0, anchor u -> anchor d, 1 -> lim. */
static float lever_side(const sb_lever_side_t *a, float u)
{
    if (u <= 0.0f) return 0.0f;
    if (u <= a->u) return (a->u > 1e-4f) ? a->d * u / a->u : a->d;
    return (a->u < 1.0f) ? a->d + (a->lim - a->d) * (u - a->u) / (1.0f - a->u) : a->d;
}

/* Full lever = the CURRENT power limit, but a raised limit never makes the
 * throttle jump: the anchor keeps today's throttle where the lever is now,
 * and more power only comes as the lever moves further forward. */
static void lever_side_track(sb_lever_side_t *a, float u, float lim)
{
    if (u <= 0.0f || lim < a->lim - 1e-3f) {        /* neutral / limit lowered: standard */
        a->u = 1.0f;
        a->d = a->lim = lim;
    } else if (lim > a->lim + 1e-3f) {              /* limit raised: hold here */
        a->d = lever_side(a, u);
        a->u = u;
        a->lim = lim;
    } else if (u < a->u) {                          /* moved back: anchor follows */
        a->d = lever_side(a, u);
        a->u = u;
    }
}

static bool lever_has_throttle(sb_mode_t m)
{
    return m == MODE_MANUAL || m == MODE_HDG_LOCK;
}

/* Throttle authority passes to a lever (MANUAL / HDG LOCK entered from
 * STANDBY, AUTO or TRAINING; REMOTE MANUAL): the lever does nothing until it
 * matches what the engine does now (or is in the neutral zone). From AUTO /
 * TRAINING / HDG LOCK the current throttle is held meanwhile; from STANDBY
 * the stop goes on. */
static void lever_pickup_start_v(sb_t *s, sb_lever_t *v, sb_mode_t from)
{
    bool moving = from != MODE_STANDBY && s->gear.current != GEAR_N &&
                  s->thr.out_pct > s->cfg.gear.idle_max_pct;
    v->pickup = true;
    v->have_prev = false;
    v->hold_gear = moving ? s->gear.current : GEAR_N;
    v->hold_pct = moving ? s->thr.out_pct : 0.0f;
    v->target = -1.0f;
}

static void lever_pickup_start(sb_t *s, sb_mode_t from)
{
    lever_pickup_start_v(s, &s->lev, from);
}

/* Lever L (0 back .. 1 forward) -> gear + throttle. Neutral zone (45..55 %)
 * = idle + neutral, above = forward, below = reverse; full lever = lim_f /
 * lim_r (the power limit, or less: the remote cap). Gear changes go through
 * the gear state machine. */
static void lever_drive(sb_t *s, sb_lever_t *v, float L, float lim_f, float lim_r)
{
    float lo = s->joy.cfg.nz_lo, hi = s->joy.cfg.nz_hi;
    float uf = (L > hi) ? (L - hi) / (1.0f - hi) : 0.0f;
    float ur = (L < lo) ? (lo - L) / lo : 0.0f;

    if (v->pickup) {
        float lim = (v->hold_gear == GEAR_R) ? lim_r : lim_f;
        if (v->hold_pct > lim) v->hold_pct = lim;           /* follows a lowered limit */
        bool moving = v->hold_gear != GEAR_N && v->hold_pct > s->cfg.gear.idle_max_pct;
        float f = v->hold_pct / fmaxf(lim, 1.0f);
        v->target = !moving ? -1.0f : (v->hold_gear == GEAR_F) ? hi + f * (1.0f - hi) : lo - f * lo;
        bool hit = (L >= lo && L <= hi) ||
                   (v->target >= 0.0f && (fabsf(L - v->target) <= s->jmap.pickup_win ||
                                          (v->have_prev && (v->prev - v->target) * (L - v->target) < 0.0f)));
        v->prev = L;
        v->have_prev = true;
        if (!hit) {
            s->gear_demand = moving ? v->hold_gear : GEAR_N;
            s->throttle_demand = moving ? v->hold_pct : 0.0f;
            return;
        }
        /* picked up: go on from the held throttle, no step */
        v->pickup = false;
        v->fwd.u = v->rev.u = 1.0f;
        v->fwd.d = v->fwd.lim = lim_f;
        v->rev.d = v->rev.lim = lim_r;
        if (moving && v->hold_gear == GEAR_F && uf > 0.0f) { v->fwd.u = uf; v->fwd.d = v->hold_pct; }
        if (moving && v->hold_gear == GEAR_R && ur > 0.0f) { v->rev.u = ur; v->rev.d = v->hold_pct; }
    }

    lever_side_track(&v->fwd, uf, lim_f);
    lever_side_track(&v->rev, ur, lim_r);
    if (uf > 0.0f) {
        s->gear_demand = GEAR_F;
        s->throttle_demand = lever_side(&v->fwd, uf);
    } else if (ur > 0.0f) {
        s->gear_demand = GEAR_R;
        s->throttle_demand = lever_side(&v->rev, ur);
    } else {
        s->gear_demand = GEAR_N;
        s->throttle_demand = 0.0f;
    }
}

static void lever_throttle(sb_t *s)
{
    lever_drive(s, &s->lev, s->joy.lever, s->plim.fwd_pct, s->plim.rev_pct);
}

/* --- REMOTE MANUAL (section D): the shore joystick drives ------------------------
 *
 * Priority, always: STANDBY button > onboard joystick > remote joystick >
 * autopilot modes. Someone is always on board. Enabled only by the HELM
 * station's request AND the joystick CONFIRM on board; ends when the remote
 * driver releases it, the onboard joystick takes over (MANUAL), STANDBY,
 * or any fault. Throttle capped (20 % fwd / 5 % rev, never above the power
 * limit), steering rate-limited. Data too old / missing / too slow: idle +
 * neutral at once, gyro heading hold, alarm; after 5 s STANDBY. It never
 * comes back by itself. */

static float remote_cap_f(const sb_t *s)
{
    return fminf(s->cfg.remote_cap_fwd_pct, s->plim.fwd_pct);
}

static float remote_cap_r(const sb_t *s)
{
    return fminf(s->cfg.remote_cap_rev_pct, s->plim.rev_pct);
}

sb_result_t sb_remote_request(sb_t *s, uint8_t station)
{
    if (s->cfg.first_test) return SB_ERR_WRONG_MODE;              /* FIRST TEST: steering only */
    if (sb_mode(s) == MODE_MANUAL) return SB_ERR_JOYSTICK;        /* the onboard joystick has it */
    if (sb_mode(s) == MODE_REMOTE || sb_mode(s) == MODE_TRAINING) return SB_ERR_WRONG_MODE;
    if (!s->armed || s->mon_tripped) return SB_ERR_PRECOND;
    if (s->joy.off_centre) return SB_ERR_NOT_CENTERED;            /* centre the joystick */
    if (!joy_ready(s)) return SB_ERR_JOY_NOT_READY;              /* nobody could press CONFIRM */
    s->remote.station = station;
    s->confirm.kind = CONF_REMOTE;
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
    s->remote.requests++;
    return SB_OK;
}

sb_result_t sb_remote_release(sb_t *s, uint8_t station)
{
    if (s->confirm.kind == CONF_REMOTE && s->remote.station == station) s->confirm.kind = CONF_NONE;
    if (sb_mode(s) != MODE_REMOTE) return SB_OK;
    if (station != s->remote.station) return SB_ERR_WRONG_MODE;
    return sb_request_mode(s, MODE_STANDBY, WHY_REMOTE_END) ? SB_OK : SB_ERR_PRECOND;
}

void sb_remote_input(sb_t *s, uint8_t station, uint16_t seq, uint16_t age_ms, uint16_t lat_ms,
                     float x, float y, float lever, bool has_lever)
{
    sb_remote_t *r = &s->remote;
    if (sb_mode(s) != MODE_REMOTE || station != r->station) {
        r->ignored++;
        return;
    }
    if (r->have && (int16_t)(seq - r->seq) <= 0) {                /* old / repeated packet */
        r->ignored++;
        return;
    }
    r->have = true;
    r->seq = seq;
    r->rx_ms = s->now_ms;
    r->age_ms = age_ms;
    r->lat_ms = lat_ms;
    r->x = clampf(x, -1.0f, 1.0f);
    r->y = clampf(y, -1.0f, 1.0f);
    r->lever = clampf(lever, 0.0f, 1.0f);
    r->has_lever = has_lever;
    r->packets++;
}

static void remote_enter(sb_t *s, sb_mode_t from)
{
    sb_remote_t *r = &s->remote;
    r->state = SB_REMOTE_DRIVING;
    r->entered_ms = s->now_ms;
    r->have = false;
    r->steer_cmd = s->ap.est_angle;                   /* no steering jump */
    lever_pickup_start_v(s, &r->lev, from);
    r->lev.fwd.u = r->lev.rev.u = 1.0f;
    r->lev.fwd.d = r->lev.fwd.lim = remote_cap_f(s);
    r->lev.rev.d = r->lev.rev.lim = remote_cap_r(s);
}

/* Steering for REMOTE (read_inputs): stick X -> outboard angle, rate-limited. */
static float remote_steer(sb_t *s, float dt)
{
    sb_remote_t *r = &s->remote;
    if (r->state != SB_REMOTE_DRIVING) return r->steer_cmd;
    float x = r->x, db = s->joy.cfg.deadband;
    float xs = (fabsf(x) <= db) ? 0.0f : (x - copysignf(db, x)) / (1.0f - db);
    float want = xs * s->ap.cfg.outboard.max_angle_deg;
    float step = s->cfg.remote_steer_rate_dps * dt;
    r->steer_cmd += clampf(want - r->steer_cmd, -step, step);
    return r->steer_cmd;
}

/* The freshness watchdog + throttle, every step in REMOTE. */
static void remote_mode(sb_t *s)
{
    sb_remote_t *r = &s->remote;
    const sb_config_t *c = &s->cfg;
    if (r->state == SB_REMOTE_DRIVING) {
        uint32_t since = s->now_ms - (r->have ? r->rx_ms : r->entered_ms);
        bool stale = since > c->remote_max_age_ms + (r->have ? 0u : 200u);   /* first packet: a little grace */
        bool old = r->have && r->age_ms > c->remote_max_age_ms;
        bool slow = r->have && r->lat_ms > c->remote_max_lat_ms;
        if (stale || old || slow) {
            r->state = SB_REMOTE_LOST;
            r->lost_ms = s->now_ms;
            r->lost_why = stale ? 1u : old ? 2u : 3u;
            r->losses++;
            alarm_on(s, SB_ALM_REMOTE_DRIVE);
            /* idle + neutral at once; hold the heading on the gyro */
            if (autopilot_gyro_ok(&s->ap)) autopilot_gyro_hold(&s->ap);
            else autopilot_center(&s->ap);
        }
    }
    if (r->state == SB_REMOTE_LOST) {
        s->gear_demand = GEAR_N;
        s->throttle_demand = 0.0f;
        if (s->now_ms - r->lost_ms >= c->remote_lost_ms)
            sb_request_mode(s, MODE_STANDBY, WHY_REMOTE_LOST);
        return;
    }
    if (!r->have) {
        /* no remote data yet: keep what the engine does now (like a pickup) - a
         * neutral demand here would start a gear change for nothing */
        bool moving = r->lev.hold_gear != GEAR_N && r->lev.hold_pct > c->gear.idle_max_pct;
        float lim = (r->lev.hold_gear == GEAR_R) ? remote_cap_r(s) : remote_cap_f(s);
        s->gear_demand = moving ? r->lev.hold_gear : GEAR_N;
        s->throttle_demand = moving ? fminf(r->lev.hold_pct, lim) : 0.0f;
        return;
    }
    if (r->has_lever) {
        lever_drive(s, &r->lev, r->lever, remote_cap_f(s), remote_cap_r(s));
    } else {
        float y = r->y, db = s->joy.cfg.deadband;
        float mag = (fabsf(y) <= db) ? 0.0f : (fabsf(y) - db) / (1.0f - db);
        s->gear_demand = (y > db) ? GEAR_F : (y < -db) ? GEAR_R : GEAR_N;
        s->throttle_demand = mag * ((y > 0.0f) ? remote_cap_f(s) : remote_cap_r(s));
    }
}

/* Joystick AUTO button in MANUAL, stick centred: back to HDG_LOCK if that
 * was interrupted, otherwise STANDBY. Not centred: refused (logged). */
static void joystick_handback(sb_t *s)
{
    if (!joy_centered(&s->joy)) {
        mode_request(&s->modes, MODE_STANDBY, WHY_JOY_HANDBACK, false, s->now_ms);
        s->chirp_ms = 300u;
        return;
    }
    s->alarm_bits &= ~SB_ALM_JOYSTICK;
    if (s->manual_from == MODE_HDG_LOCK && sb_request_mode(s, MODE_HDG_LOCK, WHY_JOY_HANDBACK)) return;
    sb_request_mode(s, MODE_STANDBY, WHY_JOY_HANDBACK);
}

bool sb_joystick_cal_start(sb_t *s)
{
    if (sb_mode(s) != MODE_STANDBY || !s->joy.present) return false;
    joy_cal_start(&s->joy);
    return true;
}

bool sb_joystick_cal_finish(sb_t *s)
{
    joy_cal_t cal;
    if (sb_mode(s) != MODE_STANDBY || !joy_cal_finish(&s->joy, &cal)) return false;
    s->cfg_version++;
    return hal_nv_save(HAL_NV_SLOT_JOYSTICK, &cal, (uint32_t)sizeof cal);
}

bool sb_joymap_set(sb_t *s, const joymap_t *m, bool save, jm_error_t *err)
{
    joymap_t n = *m;
    joymap_seal(&n);
    *err = joymap_check(&n);
    if (sb_mode(s) != MODE_STANDBY || *err != JM_OK) return false;
    s->jmap = n;
    joymap_apply(s);
    s->cfg_version++;
    return !save || hal_nv_save(HAL_NV_SLOT_JOYMAP, &n, (uint32_t)sizeof n);
}

/* --- heading hold paused ------------------------------------------------------- */

/* Water flowing past the outboard: in forward gear (or engaging it) and
 * not just idling in neutral. Without it the heading can't be steered. */
static bool propulsion(const sb_t *s)
{
    if (s->cfg.first_test) return s->moving_ahead;      /* the throttle is driven by hand */
    return gear_is(&s->gear, GEAR_F) ||
           (s->gear_demand == GEAR_F && s->gear.target == GEAR_F && s->thr.out_pct > s->cfg.gear.idle_max_pct);
}

/* Pause the heading hold (reverse, or no propulsion): the outboard is held
 * at `angle` and the motor rests, so the angle estimate doesn't wander by
 * hunting without steerage. Resume: heading hold again, same target. */
static void steer_pause(sb_t *s, bool pause, float angle)
{
    if (pause) {
        if (!s->hdg_reverse) {
            s->hdg_reverse = true;
            s->hdg_target = s->ap.target_heading;
            autopilot_angle(&s->ap);
        }
        s->hold_angle = angle;
    } else if (s->hdg_reverse) {
        s->hdg_reverse = false;
        float t = (sb_mode(s) == MODE_AUTO) ? s->ap.target_heading : s->hdg_target;
        autopilot_engage(&s->ap, true);
        autopilot_set_target(&s->ap, t);
    }
}

/* --- HDG_LOCK ------------------------------------------------------------------ */

/* Steering holds the locked heading; throttle on joystick Y. In reverse the
 * outboard is held centred (heading hold only works going ahead). Compass
 * lost: gyro hold / centring until the user has stopped, then STANDBY. */
static void hdg_lock_mode(sb_t *s)
{
    if (!s->cfg.first_test) joystick_throttle(s);       /* FIRST TEST: throttle by hand */

    if (s->hdg_compass_lost) {
        bool safe = s->thr.out_pct <= s->cfg.gear.idle_max_pct &&
                    (gear_is(&s->gear, GEAR_N) || gear_fault(&s->gear));
        if (s->stop_steer == STOP_STEER_GYRO && !autopilot_gyro_ok(&s->ap)) {
            autopilot_center(&s->ap);               /* gyro lost too */
            s->stop_steer = STOP_STEER_CENTER;
        } else if (s->stop_steer == STOP_STEER_CENTER && autopilot_centered(&s->ap)) {
            s->stop_steer = STOP_STEER_NONE;        /* centred: motor off */
            autopilot_disengage(&s->ap);
        }
        if (safe && s->gear_demand == GEAR_N) {
            s->stop_steer = STOP_STEER_NONE;
            s->hdg_compass_lost = false;
            sb_request_mode(s, MODE_STANDBY, WHY_COMPASS_FAULT);   /* steering off */
        }
        return;
    }

    bool reverse = s->gear_demand == GEAR_R || s->gear.current == GEAR_R ||
                   (s->gear.phase == GS_TO_GEAR && s->gear.engaging == GEAR_R);
    if (reverse) steer_pause(s, true, 0.0f);                    /* reverse: outboard centred */
    else if (!propulsion(s)) steer_pause(s, true, s->hdg_reverse ? s->hold_angle : s->ap.est_angle);
    else steer_pause(s, false, 0.0f);
    if (!s->hdg_reverse) steer_by_speed(s, speed_now(s));
}

/* --- AUTO: speed --------------------------------------------------------------- */

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

    /* slamming counts too (yellow / red = rough; calm only when it is calm) */
    bool rough = n->yaw_std > c->nerv_on_dps || (n->roll_ok && n->roll_std > c->nerv_roll_on_dps) ||
                 s->slam.state != SLAM_CALM;
    bool calm = n->yaw_std < c->nerv_off_dps && (!n->roll_ok || n->roll_std < c->nerv_roll_off_dps) &&
                s->slam.state == SLAM_CALM;

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
        bool still_rough = n->yaw_std > mid || s->slam.state != SLAM_CALM ||
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
    nav_steer(s);

    bool gps = sb_gps_ok(s);
    if (gps) {
        k->sog_kn = s->gps.last.sog_kn;
        k->gps_lost = false;
    }

    float top = fminf(c->throttle.max_fwd_pct, s->plim.fwd_pct);     /* power limit */
    if (!gear_is(&s->gear, GEAR_F)) {
        s->throttle_demand = 0.0f;          /* wait for forward gear */
    } else if (gps) {
        float e = auto_target_kn(s) - k->sog_kn;
        float u = c->spd_kp * e + k->integ;
        bool sat_hi = u >= top && e > 0.0f;
        bool sat_lo = u <= 0.0f && e < 0.0f;
        bool lagging = fabsf(s->throttle_demand_prev - s->thr.out_pct) > 3.0f;   /* rate-limited */
        if (!sat_hi && !sat_lo && !lagging)
            k->integ = clampf(k->integ + c->spd_ki * e * dt, 0.0f, top);
        if (k->integ > top) k->integ = top;
        s->throttle_demand = clampf(c->spd_kp * e + k->integ, 0.0f, top);
        /* "speed limited by the power limit": at the limit and still too slow */
        s->sat_ms = (s->throttle_demand >= top - 0.01f && top < c->throttle.max_fwd_pct &&
                     e > 0.5f) ? s->sat_ms + dt_ms : 0u;
        s->speed_limited = s->sat_ms >= 3000u;
    } else {
        if (!k->gps_lost) {
            k->gps_lost = true;
            k->open_loop_pct = s->thr.out_pct * c->gps_loss_throttle_factor;
            k->integ = k->open_loop_pct;
            alarm_on(s, (s->dr.why == DR_WHY_LOST || s->dr.state == DR_NO_FIX || s->dr.state == DR_GPS)
                        ? SB_ALM_GPS_LOST : SB_ALM_GPS_SUSPECT);
        }
        s->throttle_demand = k->open_loop_pct;
    }

    steer_by_speed(s, k->gps_lost ? k->sog_kn * c->gps_loss_throttle_factor : k->sog_kn);
}

/* --- AUTO: navigation ---------------------------------------------------------- */

/* Fallback inside AUTO: hold the current heading at a slow speed, alarm.
 * open_loop: throttle from the learned table (position not trusted). The
 * program stays where it is; resume = PC + CONFIRM, or take the joystick. */
static void auto_hold(sb_t *s, float kn, uint32_t alarm_bit, bool open_loop)
{
    alarm_on(s, alarm_bit);
    autopilot_set_target(&s->ap, s->ap.heading);
    s->autop.state = AUTO_HOLD;
    s->autop.hold_kn = kn;
    set_speed(s, kn);
    if (open_loop) {
        s->cruise.gps_lost = true;
        s->cruise.open_loop_pct = dr_throttle_for(&s->dr, kn);
    }
}

/* Last waypoint reached: final action. */
static void auto_finish(sb_t *s)
{
    s->autop.finished = true;
    s->autop.suspended = false;
    if (s->autop.prog.final_action == (uint8_t)PROG_END_HOLD) {
        auto_hold(s, s->cfg.approach_kn, SB_ALM_ARRIVED, false);
    } else {
        alarm_on(s, SB_ALM_ARRIVED);
        sb_request_mode(s, MODE_STANDBY, WHY_ARRIVED);          /* idle + neutral */
    }
}

/* Follow the line to the active waypoint from the position estimate (GPS,
 * or dead reckoning when GPS is lost / not trusted): cross-track error ->
 * wanted course over ground; plus the drift angle (heading - course over
 * ground: measured COG on GPS, predicted from the learned current in DR)
 * -> heading for the steering core. Speed: leg speed (x dr_speed_factor
 * in DR), slowing to approach_kn before SLOW / STOP / the last waypoint. */
static void nav_update(sb_t *s, float fix_dt)
{
    const sb_config_t *c = &s->cfg;
    sb_auto_t *a = &s->autop;
    const prog_wp_t *w = &a->prog.wp[a->wp];
    bool last = (uint16_t)(a->wp + 1u) >= a->prog.n;
    double lat, lon;

    dr_latlon(&s->dr, &lat, &lon);
    nav_leg_update(&a->leg, lat, lon);

    float sog = a->in_dr ? dr_sog_kn(&s->dr) : s->gps.last.sog_kn;
    float cog = a->in_dr ? dr_cog_deg(&s->dr) : s->gps.last.cog_deg;
    if (sog > 2.0f) {
        float drift = autopilot_heading_error(s->ap.heading, cog);
        float k = clampf(fix_dt / c->crab_tau_s, 0.0f, 1.0f);
        a->crab_deg += k * (drift - a->crab_deg);
        a->crab_deg = clampf(a->crab_deg, -45.0f, 45.0f);
    }
    float look = fmaxf(c->lookahead_min_m, c->lookahead_s * sog * 0.514444f);
    a->course_deg = nav_leg_course(&a->leg, look);
    autopilot_set_target(&s->ap, a->course_deg + a->crab_deg);

    /* in DR the waypoint may be anywhere within the uncertainty */
    float unc = a->in_dr ? s->dr.unc_m : 0.0f;
    bool uncertain = unc > w->radius_m;
    float leg_kn = fminf(w->speed_kn, c->cruise_max_kn);
    float v = leg_kn * (a->in_dr ? c->dr_speed_factor : 1.0f);
    bool slows = last || w->action != (uint8_t)WP_ACT_CONTINUE;
    if (slows && a->leg.dist_m < c->slow_radius_m + unc) {
        float f = clampf((a->leg.dist_m - w->radius_m - unc) / (c->slow_radius_m - w->radius_m), 0.0f, 1.0f);
        v = c->approach_kn + (v - c->approach_kn) * f;
    }
    if (uncertain && !a->arrival_uncertain && a->leg.dist_m < c->slow_radius_m + unc) {
        a->arrival_uncertain = true;            /* early warning */
        alarm_on(s, SB_ALM_ARRIVAL_UNCERTAIN);
    }
    set_speed(s, fminf(v, leg_kn));

    if (a->leg.dist_m <= w->radius_m || a->leg.along_m >= a->leg.length_m) {
        if (uncertain) {
            /* don't declare arrival: hold slow, the skipper takes over */
            auto_hold(s, c->approach_kn, SB_ALM_ARRIVAL_UNCERTAIN, true);
            return;
        }
        a->legs_done++;
        if (last) {
            auto_finish(s);
            return;
        }
        a->wp++;
        if (w->action == (uint8_t)WP_ACT_STOP) {
            /* stop in neutral here (chirp, not an alarm); the next leg starts at resume */
            s->chirp_ms = 600u;
            autopilot_set_target(&s->ap, s->ap.heading);
            a->state = AUTO_WP_STOP;
            return;
        }
        /* next leg: the line from this waypoint to the next */
        const prog_wp_t *nw = &a->prog.wp[a->wp];
        nav_leg_init(&a->leg, program_lat(w), program_lon(w), program_lat(nw), program_lon(nw));
        nav_leg_update(&a->leg, lat, lon);
        a->arrival_uncertain = false;
        set_speed(s, fminf(nw->speed_kn, c->cruise_max_kn));
    }
}

/* Speed in DR without GPS: PI on the paddlewheel if fitted, otherwise the
 * learned throttle -> speed table (open loop). */
static void dr_speed(sb_t *s, float dt, uint32_t dt_ms, bool roll_new, float roll)
{
    const sb_config_t *c = &s->cfg;
    sb_cruise_t *k = &s->cruise;
    s->gear_demand = GEAR_F;
    nervousness(s, dt_ms, roll_new, roll);
    nav_steer(s);
    if (!gear_is(&s->gear, GEAR_F)) {
        s->throttle_demand = 0.0f;
    } else if (s->dr.speed_sensor) {
        float e = auto_target_kn(s) - s->dr.water_kn;
        bool sat = (c->spd_kp * e + k->integ >= c->throttle.max_fwd_pct && e > 0.0f) ||
                   (c->spd_kp * e + k->integ <= 0.0f && e < 0.0f);
        if (!sat) k->integ = clampf(k->integ + c->spd_ki * e * dt, 0.0f, c->throttle.max_fwd_pct);
        s->throttle_demand = clampf(c->spd_kp * e + k->integ, 0.0f, c->throttle.max_fwd_pct);
    } else {
        s->throttle_demand = dr_throttle_for(&s->dr, auto_target_kn(s));
        k->integ = s->throttle_demand;
    }
    if (s->throttle_demand > s->plim.fwd_pct) {             /* power limit */
        s->throttle_demand = s->plim.fwd_pct;
        if (k->integ > s->plim.fwd_pct) k->integ = s->plim.fwd_pct;
    }
    steer_by_speed(s, dr_sog_kn(&s->dr));
}

/* AUTO running a leg / holding: heading hold on (after a pause). */
static void auto_steer_when_moving(sb_t *s)
{
    steer_pause(s, false, 0.0f);
}

static bool auto_running(const sb_t *s)
{
    return sb_mode(s) == MODE_AUTO && s->autop.state == AUTO_RUN;
}

static void auto_mode(sb_t *s, float dt, uint32_t dt_ms, bool roll_new, float roll)
{
    const sb_config_t *c = &s->cfg;
    sb_auto_t *a = &s->autop;

    switch (a->state) {
    case AUTO_PAUSED:
    case AUTO_WP_STOP:
        /* idle + neutral (gear / throttle default); no steerage: outboard held still */
        steer_pause(s, true, s->hdg_reverse ? s->hold_angle : s->ap.est_angle);
        return;
    case AUTO_HOLD:
        auto_steer_when_moving(s);
        cruise(s, dt, dt_ms, roll_new, roll);
        return;
    default:
        break;
    }
    auto_steer_when_moving(s);

    if (!sb_gps_ok(s)) {
        if (!dr_have_position(&s->dr)) {
            /* no position at all: hold the current heading, alarm */
            auto_hold(s, s->cruise.set_kn, SB_ALM_GPS_LOST, false);
            cruise(s, dt, dt_ms, roll_new, roll);
            return;
        }
        if (!a->in_dr) {
            /* GPS lost or not trusted: carry on by dead reckoning */
            a->in_dr = true;
            a->next_dr_ms = s->now_ms;
            /* (the estimate may notice the plain loss one step later) */
            alarm_on(s, (s->dr.state == DR_GPS || s->dr.why == DR_WHY_LOST) ? SB_ALM_GPS_LOST : SB_ALM_GPS_SUSPECT);
        }
        if (s->dr.unc_m > c->dr_max_unc_m ||
            dr_dr_time_s(&s->dr, s->now_ms) * 1000.0f > (float)c->dr_max_ms) {
            /* too uncertain / too long: hold the current heading, slow */
            auto_hold(s, c->dr_slow_kn, SB_ALM_DR_LIMIT, true);
            cruise(s, dt, dt_ms, roll_new, roll);
            return;
        }
        if ((int32_t)(s->now_ms - a->next_dr_ms) >= 0) {
            a->next_dr_ms += 1000u;
            nav_update(s, 1.0f);
            if (!auto_running(s)) {
                if (sb_mode(s) == MODE_AUTO && a->state == AUTO_HOLD) cruise(s, dt, dt_ms, roll_new, roll);
                return;
            }
        }
        dr_speed(s, dt, dt_ms, roll_new, roll);
        return;
    }

    if (a->in_dr) {
        /* GPS trusted again (dr.c blends the position over smoothly) */
        a->in_dr = false;
        s->cruise.integ = s->thr.out_pct;           /* bumpless speed loop */
    }
    if (s->gps.last_ms != a->last_fix_ms) {
        float fix_dt = (float)(s->gps.last_ms - a->last_fix_ms) * 0.001f;
        a->last_fix_ms = s->gps.last_ms;
        nav_update(s, fix_dt);
        if (!auto_running(s) && !(sb_mode(s) == MODE_AUTO && a->state == AUTO_HOLD)) return;
    }
    cruise(s, dt, dt_ms, roll_new, roll);
}

/* --- AUTO: PC requests ------------------------------------------------------------- */

static sb_result_t auto_common_checks(const sb_t *s)
{
    if (s->cfg.first_test) return SB_ERR_WRONG_MODE;              /* FIRST TEST: no AUTO */
    if (!s->autop.prog_ok) return SB_ERR_NO_PROGRAM;
    if (s->joy.off_centre) return SB_ERR_NOT_CENTERED;            /* centre the joystick */
    if (!joy_ready(s)) return SB_ERR_JOY_NOT_READY;
    if (!sb_compass_ok(s) || !sb_gps_ok(s) || gear_fault(&s->gear)) return SB_ERR_PRECOND;
    return SB_OK;
}

static void confirm_arm(sb_t *s, confirm_kind_t kind)
{
    s->confirm.kind = kind;
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
}

sb_result_t sb_program_set(sb_t *s, const program_t *p, prog_error_t *err, uint8_t *bad_index)
{
    *bad_index = 0xFFu;
    *err = PROG_OK;
    if (sb_mode(s) == MODE_AUTO) return SB_ERR_WRONG_MODE;   /* not while it runs */
    *err = program_validate(p, &s->cfg.prog, bad_index);
    if (*err != PROG_OK) return SB_ERR_BAD_PROGRAM;
    s->autop.prog = *p;
    s->autop.prog_ok = true;
    s->autop.wp = 0u;
    s->autop.suspended = false;
    s->autop.finished = false;
    s->autop.legs_done = 0u;
    s->cfg_version++;
    if (s->confirm.kind == CONF_AUTO_START || s->confirm.kind == CONF_AUTO_RESUME) s->confirm.kind = CONF_NONE;
    (void)hal_nv_save(HAL_NV_SLOT_PROGRAM, p, (uint32_t)sizeof *p);
    return SB_OK;
}

sb_result_t sb_auto_start(sb_t *s)
{
    if (sb_mode(s) == MODE_AUTO || sb_mode(s) == MODE_TRAINING) return SB_ERR_WRONG_MODE;
    sb_result_t r = auto_common_checks(s);
    if (r != SB_OK) return r;
    uint8_t save = s->autop.wp;
    s->autop.wp = 0u;
    bool leg = auto_leg_ok(s);
    s->autop.wp = save;
    if (!leg) return SB_ERR_PRECOND;
    confirm_arm(s, CONF_AUTO_START);
    return SB_OK;
}

sb_result_t sb_auto_resume(sb_t *s)
{
    const sb_auto_t *a = &s->autop;
    bool can = (sb_mode(s) == MODE_AUTO && a->state != AUTO_RUN && !a->finished) ||
               (sb_mode(s) != MODE_AUTO && sb_mode(s) != MODE_TRAINING && a->suspended && !a->finished);
    if (!can) return a->prog_ok ? SB_ERR_WRONG_MODE : SB_ERR_NO_PROGRAM;
    sb_result_t r = auto_common_checks(s);
    if (r != SB_OK) return r;
    if (!auto_leg_ok(s)) return SB_ERR_PRECOND;
    confirm_arm(s, CONF_AUTO_RESUME);
    return SB_OK;
}

sb_result_t sb_auto_pause(sb_t *s)
{
    if (sb_mode(s) != MODE_AUTO || (s->autop.state != AUTO_RUN && s->autop.state != AUTO_HOLD))
        return SB_ERR_WRONG_MODE;
    s->autop.state = AUTO_PAUSED;
    autopilot_set_target(&s->ap, s->ap.heading);
    return SB_OK;
}

sb_result_t sb_auto_skip(sb_t *s)
{
    sb_auto_t *a = &s->autop;
    if (!(sb_mode(s) == MODE_AUTO || a->suspended) || a->finished || !a->prog_ok) return SB_ERR_WRONG_MODE;
    a->wp++;
    if (a->wp >= a->prog.n) {
        a->wp = (uint8_t)(a->prog.n - 1u);
        if (sb_mode(s) == MODE_AUTO) {
            auto_finish(s);
        } else {
            a->finished = true;
            a->suspended = false;
        }
    } else if (auto_running(s) || (sb_mode(s) == MODE_AUTO && a->state == AUTO_HOLD)) {
        auto_begin_leg(s);
    }
    return SB_OK;
}

float sb_confirm_left_s(const sb_t *s)
{
    if (s->confirm.kind == CONF_NONE) return 0.0f;
    int32_t left = (int32_t)(s->confirm.deadline_ms - s->now_ms);
    return left > 0 ? (float)left * 0.001f : 0.0f;
}

/* Joystick CONFIRM pressed with a pending PC request. */
static bool confirm_execute(sb_t *s, confirm_kind_t kind)
{
    sb_auto_t *a = &s->autop;
    switch (kind) {
    case CONF_AUTO_START:
        if (sb_mode(s) == MODE_AUTO) return false;
        a->wp = 0u;
        a->finished = false;
        a->legs_done = 0u;
        return sb_request_mode(s, MODE_AUTO, WHY_CONFIRM);
    case CONF_AUTO_RESUME:
        if (sb_mode(s) == MODE_AUTO) {
            if (a->state == AUTO_RUN || !auto_leg_ok(s) || !sb_gps_ok(s)) return false;
            cruise_start(s);
            auto_begin_leg(s);
            return true;
        }
        return sb_request_mode(s, MODE_AUTO, WHY_CONFIRM);
    case CONF_REMOTE:
        /* the HELM station must still be the one that asked */
        if (s->helm != s->remote.station) return false;
        return sb_request_mode(s, MODE_REMOTE, WHY_REMOTE);
    case CONF_RAISE_FWD:
    case CONF_RAISE_REV:
        /* one step up; never changes the mode */
        return plim_step_up(&s->plim, kind == CONF_RAISE_REV ? 1u : 0u, s->now_ms) == PLIM_OK;
    default:
        return false;
    }
}

static bool is_raise(confirm_kind_t k)
{
    return k == CONF_RAISE_FWD || k == CONF_RAISE_REV;
}

sb_result_t sb_power_code(sb_t *s, uint8_t dir, uint16_t *code)
{
    *code = 0u;
    if (!joy_ready(s)) return SB_ERR_JOY_NOT_READY;
    if (s->alarm || s->ap.alarm) return SB_ERR_ALARM;
    if (dir == 0u ? s->plim.fwd_step + 1u >= PLIM_FWD_STEPS
                  : s->plim.rev_pct >= s->plim.cfg.rev_max - 0.01f) return SB_ERR_AT_MAX;
    *code = plim_code_request(&s->plim, dir ? 1u : 0u, s->now_ms);
    return SB_OK;
}

sb_result_t sb_power_raise(sb_t *s, uint8_t dir, uint16_t code)
{
    dir = dir ? 1u : 0u;
    if (!joy_ready(s)) {
        plim_log_attempt(&s->plim, dir, PLIM_ERR_JOY_NOT_READY, s->now_ms);
        return SB_ERR_JOY_NOT_READY;
    }
    if (s->alarm || s->ap.alarm) {
        plim_log_attempt(&s->plim, dir, PLIM_ERR_ALARM, s->now_ms);
        return SB_ERR_ALARM;
    }
    plim_result_t r = plim_code_check(&s->plim, dir, code, s->now_ms);
    if (r == PLIM_ERR_WRONG_CODE) return SB_ERR_WRONG_CODE;
    if (r == PLIM_ERR_AT_MAX) return SB_ERR_AT_MAX;
    confirm_arm(s, dir ? CONF_RAISE_REV : CONF_RAISE_FWD);
    return SB_OK;
}

void sb_power_lower(sb_t *s)
{
    if (is_raise(s->confirm.kind)) s->confirm.kind = CONF_NONE;
    plim_drop(&s->plim, PLIM_SRC_PC, s->now_ms);
}

static void confirm_step(sb_t *s)
{
    sb_confirm_t *c = &s->confirm;
    if (c->kind != CONF_NONE && (int32_t)(s->now_ms - c->deadline_ms) >= 0) {
        if (is_raise(c->kind)) plim_log_attempt(&s->plim, c->kind == CONF_RAISE_REV, PLIM_ERR_LATE, s->now_ms);
        c->kind = CONF_NONE;            /* too late */
    }
    if (!s->joy.confirm_edge) return;
    if (c->kind == CONF_NONE) {
        c->ignored++;                   /* nothing pending: does nothing */
        return;
    }
    if (!joy_confirm(&s->joy)) {
        c->refused++;                   /* stick not centred / not armed / faulted */
        if (is_raise(c->kind)) plim_log_attempt(&s->plim, c->kind == CONF_RAISE_REV, PLIM_ERR_NOT_CENTRED, s->now_ms);
        s->chirp_ms = 300u;
        return;
    }
    confirm_kind_t kind = c->kind;
    c->kind = CONF_NONE;
    if (confirm_execute(s, kind)) {
        c->accepted++;
    } else {
        c->refused++;
        s->chirp_ms = 300u;
    }
}

/* --- TRAINING ------------------------------------------------------------------ */

/* Forward gear and a fixed slow throttle; once settled, the calibration
 * (training.c, inside the steering core) runs. Finished -> idle + neutral,
 * wait for sb_confirm_training(). Any abort -> STANDBY (safe stop). */
static void training_mode(sb_t *s, uint32_t dt_ms)
{
    sb_train_t *t = &s->train;

    switch (t->stage) {
    case TS_SPINUP:
        s->gear_demand = GEAR_F;
        s->throttle_demand = s->cfg.training_throttle_pct;
        /* FIRST TEST: someone at the throttle - the boat moving ahead counts */
        if (s->cfg.first_test ? s->moving_ahead : gear_is(&s->gear, GEAR_F)) t->timer_ms += dt_ms;
        if (t->timer_ms >= s->cfg.training_spinup_ms) {
            if (autopilot_start_training(&s->ap)) {
                t->stage = TS_RUN;
            } else {
                s->alarm = true;
                sb_request_mode(s, MODE_STANDBY, WHY_TRAINING_ABORT);
            }
        }
        break;

    case TS_RUN:
        s->gear_demand = GEAR_F;
        s->throttle_demand = s->cfg.training_throttle_pct;
        if (sb_gps_ok(s)) { t->sog_sum += s->gps.last.sog_kn; t->sog_n++; }
        if (s->ap.mode == AP_MODE_TRAINING && s->ap.train.phase == TR_RESULT) {
            t->stage = TS_RESULT;               /* engine to idle + neutral below */
        } else if (s->ap.mode != AP_MODE_TRAINING) {
            /* the calibration stopped itself (limit, plausibility, sensor) */
            s->alarm = true;
            sb_request_mode(s, MODE_STANDBY, WHY_TRAINING_ABORT);
        }
        break;

    case TS_RESULT:
        s->gear_demand = GEAR_N;
        s->throttle_demand = 0.0f;
        break;
    }
}

bool sb_confirm_training(sb_t *s, bool accept, float full_angle_deg)
{
    if (sb_mode(s) != MODE_TRAINING || s->train.stage != TS_RESULT) return false;
    bool ok = autopilot_confirm_training(&s->ap, accept, full_angle_deg);
    if (ok && s->train.sog_n > 0u) {
        /* the scaled gains fit the training speed: steering scales from there */
        s->cfg.gain_ref_kn = (float)(s->train.sog_sum / (double)s->train.sog_n);
    }
    if (ok) s->cfg_version++;
    sb_request_mode(s, MODE_STANDBY, ok ? WHY_TRAINING_DONE : WHY_TRAINING_ABORT);
    return ok;
}

/* --- step ------------------------------------------------------------------ */

/* Faults, joystick and buttons (in this order of priority). */
/* Neutral-start interlock (power-up): throttle idle, neutral confirmed,
 * joystick seen and centred (cfg.joy_optional: a boat without one) for
 * arm_ms -> outputs armed until reset. */
static void interlock_step(sb_t *s, int8_t neutral_sw, uint32_t dt_ms)
{
    if (s->armed) return;
    bool neutral = (neutral_sw >= 0) ? (neutral_sw == 1 && gear_is(&s->gear, GEAR_N)) : gear_is(&s->gear, GEAR_N);
    if (s->cfg.first_test) neutral = gear_is(&s->gear, GEAR_N);      /* no neutral switch needed (gear held N) */
    /* the joystick must be there (a USB bridge still starting or an analog
     * stick not detected is no reason to skip the check) - unless the boat is
     * configured without one */
    bool stick = s->joy.present ? (s->joy.armed && joy_centered(&s->joy)) : s->cfg.joy_optional;
    if (s->cfg.first_test) stick = true;                              /* no joystick needed (no MANUAL) */
    bool idle = s->thr.out_pct <= s->cfg.gear.idle_max_pct;
    s->arm_ok_ms = (neutral && stick && idle) ? s->arm_ok_ms + dt_ms : 0u;
    s->unarmed_ms += dt_ms;
    if (s->arm_ok_ms >= s->cfg.arm_ms && !s->mon_tripped) {
        s->armed = true;
        hal_set_safety_relay(true);
        return;
    }
    if (s->unarmed_ms >= s->cfg.arm_alarm_ms && !(s->alarm_bits & SB_ALM_NOT_ARMED)) alarm_on(s, SB_ALM_NOT_ARMED);
}

static void events_step(sb_t *s, uint32_t buttons)
{
    /* EMERGENCY STOP (its contact; the switch itself cuts the actuator power):
     * every mode -> STANDBY + alarm, nothing leaves STANDBY while pressed */
    {
        bool pressed;
        s->estop_state = hal_read_estop(&pressed) ? (pressed ? 2u : 1u) : 0u;
        if (s->estop_state == 2u) {
            if (!s->estop_seen) {
                s->estop_seen = true;
                alarm_on(s, SB_ALM_ESTOP);
                plim_drop(&s->plim, PLIM_SRC_FAULT, s->now_ms);
                s->confirm.kind = CONF_NONE;
            }
            if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_ESTOP);
        } else {
            s->estop_seen = false;
        }
    }

    /* the logger (core 2) can't write: alarm; control goes on as normal */
    if (g_logstat.failure && !s->log_fail_seen) alarm_on(s, SB_ALM_LOG);
    s->log_fail_seen = g_logstat.failure != 0u;

    /* a control cycle overran its period (measured by the loop, timing.c) */
    if (g_timing.core0.overruns != s->overruns_seen) {
        s->overruns_seen = g_timing.core0.overruns;
        alarm_on(s, SB_ALM_OVERRUN);
    }

    /* independent safety monitor tripped: outputs are forced safe by the HAL */
    if (g_mon.tripped && !s->mon_tripped) {
        s->mon_tripped = true;
        alarm_on(s, SB_ALM_MONITOR);
        plim_drop(&s->plim, PLIM_SRC_FAULT, s->now_ms);
        sb_request_mode(s, MODE_STANDBY, WHY_MONITOR);
    }

    /* compass */
    if (s->ap.sensor_fault && !s->compass_fault_seen) {
        s->compass_fault_seen = true;
        alarm_on(s, SB_ALM_COMPASS);
        plim_drop(&s->plim, PLIM_SRC_COMPASS, s->now_ms);
        if (sb_mode(s) == MODE_AUTO || sb_mode(s) == MODE_TRAINING || sb_mode(s) == MODE_REMOTE) {
            sb_request_mode(s, MODE_STANDBY, WHY_COMPASS_FAULT);
        } else if (sb_mode(s) == MODE_HDG_LOCK) {
            /* throttle stays with the user: gyro hold / centring until stopped */
            s->hdg_compass_lost = true;
            if (s->hdg_reverse) {
                s->hdg_reverse = false;
                autopilot_center(&s->ap);
                s->stop_steer = STOP_STEER_CENTER;
            } else {
                fault_steer_start(s);
            }
        }
        /* MANUAL / STANDBY: manual steering doesn't need the compass -> alarm only */
    }
    if (!s->ap.sensor_fault) s->compass_fault_seen = false;

    /* GPS: lost / no longer trusted (AUTO handles its own, with DR) */
    bool gps = sb_gps_ok(s);
    if (s->gps_was_ok && !gps) {
        s->gps_losses++;
        plim_drop(&s->plim, PLIM_SRC_GPS_LOST, s->now_ms);
        if (sb_mode(s) == MODE_HDG_LOCK)
            alarm_on(s, (s->dr.state == DR_GPS || s->dr.why == DR_WHY_LOST) ? SB_ALM_GPS_LOST : SB_ALM_GPS_SUSPECT);
    }
    s->gps_was_ok = gps;

    /* joystick fault: if it was driving (MANUAL) or giving throttle (HDG_LOCK) -> STANDBY */
    if (s->joy.fault && !s->joy_fault_seen) {
        s->joy_fault_seen = true;
        alarm_on(s, SB_ALM_JOY_FAULT);
        plim_drop(&s->plim, PLIM_SRC_JOYSTICK, s->now_ms);
        if (is_raise(s->confirm.kind)) s->confirm.kind = CONF_NONE;
        if (sb_mode(s) == MODE_MANUAL || sb_mode(s) == MODE_HDG_LOCK || sb_mode(s) == MODE_REMOTE)
            sb_request_mode(s, MODE_STANDBY, WHY_JOY_FAULT);
    }
    if (!s->joy.fault) s->joy_fault_seen = false;
    if (s->joy.not_centered && !s->joy_nc_seen) {
        s->joy_nc_seen = true;
        alarm_on(s, SB_ALM_JOY_NOT_CENTERED);
    }
    if (s->joy.armed) s->joy_nc_seen = false;

    /* REMOTE MANUAL needs its station to hold HELM (a station on board took command) */
    if (sb_mode(s) == MODE_REMOTE && s->helm != s->remote.station) sb_request_mode(s, MODE_STANDBY, WHY_REMOTE_END);

    /* override: it always wins (Y / the lever is the throttle in HDG_LOCK) */
    bool grab = s->joy.take_edge || s->joy.engage_x || (s->joy.engage_y && sb_mode(s) != MODE_HDG_LOCK) ||
                (s->joy.lever_moved && !lever_has_throttle(sb_mode(s)));
    if (s->joy.take_edge) plim_drop(&s->plim, PLIM_SRC_OVERRIDE, s->now_ms);      /* TAKE always lowers */
    if (grab && sb_mode(s) != MODE_MANUAL && !s->cfg.first_test) {     /* FIRST TEST: no MANUAL */
        if (s->confirm.kind == CONF_AUTO_START || s->confirm.kind == CONF_AUTO_RESUME ||
            s->confirm.kind == CONF_REMOTE) s->confirm.kind = CONF_NONE;
        sb_request_mode(s, MODE_MANUAL, WHY_JOYSTICK);
    }
    if (sb_mode(s) == MODE_MANUAL && s->joy.auto_edge) joystick_handback(s);

    confirm_step(s);

    /* STANDBY button: works in every mode, the PC can't override it */
    if (buttons & HAL_BTN_BIT(HAL_BTN_STANDBY)) {
        s->confirm.kind = CONF_NONE;
        plim_drop(&s->plim, PLIM_SRC_STANDBY_BUTTON, s->now_ms);
        if (sb_mode(s) != MODE_STANDBY) {
            if (!joy_centered(&s->joy)) joy_disarm(&s->joy);   /* can't grab control right back */
            sb_request_mode(s, MODE_STANDBY, WHY_STANDBY_BUTTON);
        } else {
            sb_alarm_ack(s);            /* in STANDBY the button acknowledges (what is still wrong stays) */
        }
    }
    if (buttons & HAL_BTN_BIT(HAL_BTN_HDG_LOCK)) {
        if (sb_mode(s) == MODE_HDG_LOCK) {
            if (!s->hdg_compass_lost) {                             /* lock again, here */
                s->hdg_target = s->ap.heading;
                autopilot_set_target(&s->ap, s->ap.heading);
            }
        } else if ((sb_mode(s) == MODE_MANUAL && !joy_centered(&s->joy)) || s->joy.off_centre) {
            /* centre the joystick first: refused, logged, chirp */
            mode_request(&s->modes, MODE_HDG_LOCK, WHY_HDG_BUTTON, false, s->now_ms);
            s->refused_centre++;
            s->chirp_ms = 300u;
        } else {
            if (sb_mode(s) == MODE_MANUAL) s->alarm_bits &= ~SB_ALM_JOYSTICK;
            sb_request_mode(s, MODE_HDG_LOCK, WHY_HDG_BUTTON);
        }
    }
}

/* BENCH TEST, every step: panel buttons are recorded and do nothing but
 * STANDBY; the steering pulse of this step (-1 / 0 / +1); ends by itself
 * after bench_idle_ms without a command, when HELM is no longer the USB
 * laptop or when the boat moves. */
static int8_t bench_step(sb_t *s, uint32_t *buttons, uint32_t dt_ms)
{
    if (sb_mode(s) != MODE_BENCH) {
        s->bench.pulse_dir = 0;                         /* left the bench test: no pulse carries on */
        s->bench.pulse_left_ms = 0u;
        return 0;
    }
    for (unsigned b = 0; b < (unsigned)HAL_BTN_COUNT; b++) {
        if (*buttons & HAL_BTN_BIT(b)) {
            s->bench.buttons_seen |= (uint8_t)HAL_BTN_BIT(b);
            if (s->bench.button_count[b] < 255u) s->bench.button_count[b]++;
        }
    }
    *buttons &= HAL_BTN_BIT(HAL_BTN_STANDBY);           /* STANDBY always wins; the rest only shown */
    if ((uint32_t)(s->now_ms - s->bench.last_cmd_ms) > s->cfg.bench_idle_ms || s->helm != SB_HELM_USB ||
        bench_moving(s)) {
        sb_request_mode(s, MODE_STANDBY, WHY_BENCH_END);
        return 0;
    }
    if (s->bench.pulse_dir == 0) return 0;
    bool at_stop = (s->bench.pulse_dir < 0) ? (s->limit_bits & 1u) && s->limit_bits != 0x80u
                                            : (s->limit_bits & 2u) && s->limit_bits != 0x80u;
    if (s->bench.pulse_left_ms == 0u || at_stop) {      /* done, or reached the end stop */
        s->bench.pulse_dir = 0;
        s->bench.pulse_left_ms = 0u;
        s->bench.pulse_end_ms = s->now_ms;
        return 0;
    }
    int8_t dir = s->bench.pulse_dir;
    s->bench.pulse_left_ms = (s->bench.pulse_left_ms > dt_ms) ? s->bench.pulse_left_ms - dt_ms : 0u;
    return dir;
}

sb_result_t sb_bench(sb_t *s, uint8_t action, uint16_t arg)
{
    if (action == SB_BENCH_ENTER) {
        if (sb_mode(s) == MODE_BENCH) return SB_OK;
        if (sb_mode(s) == MODE_MANUAL) return SB_ERR_JOYSTICK;
        if (sb_mode(s) != MODE_STANDBY) return SB_ERR_WRONG_MODE;
        if (s->joy.off_centre) {
            (void)sb_request_mode(s, MODE_BENCH, WHY_PC);           /* refused + logged */
            return SB_ERR_NOT_CENTERED;
        }
        return sb_request_mode(s, MODE_BENCH, WHY_PC) ? SB_OK : SB_ERR_PRECOND;
    }
    if (sb_mode(s) != MODE_BENCH) return SB_ERR_WRONG_MODE;
    s->bench.last_cmd_ms = s->now_ms;
    switch (action) {
    case SB_BENCH_LEAVE:
        return sb_request_mode(s, MODE_STANDBY, WHY_PC) ? SB_OK : SB_ERR_PRECOND;
    case SB_BENCH_LEFT:
    case SB_BENCH_RIGHT: {
        int8_t dir = (action == SB_BENCH_LEFT) ? -1 : 1;
        uint8_t why = SB_BENCH_R_NONE;
        if (arg < s->cfg.bench_pulse_min_ms || arg > s->cfg.bench_pulse_max_ms) why = SB_BENCH_R_RANGE;
        else if (s->bench.pulse_dir != 0 || (s->bench.pulses > 0u &&
                 (uint32_t)(s->now_ms - s->bench.pulse_end_ms) < s->cfg.bench_gap_ms)) why = SB_BENCH_R_BUSY;
        else if (s->limit_bits != 0x80u && (s->limit_bits & (dir < 0 ? 1u : 2u))) why = SB_BENCH_R_LIMIT;
        if (why != SB_BENCH_R_NONE) {
            s->bench.last_refuse = why;
            s->bench.refused++;
            return SB_ERR_PRECOND;
        }
        s->bench.last_refuse = SB_BENCH_R_NONE;
        s->bench.pulse_dir = dir;
        s->bench.pulse_left_ms = arg;
        s->bench.pulses++;
        return SB_OK;
    }
    case SB_BENCH_BUZZER:
        if (arg < 100u || arg > s->cfg.bench_buzz_max_ms) {
            s->bench.last_refuse = SB_BENCH_R_RANGE;
            return SB_ERR_PRECOND;
        }
        s->chirp_ms = arg;
        return SB_OK;
    case SB_BENCH_TRIP:
        if (arg != SB_BENCH_TRIP_CODE) return SB_ERR_WRONG_CODE;
        s->bench.trip_req = true;                   /* -> g_mon.test_trip: CPU1 trips, latched */
        return SB_OK;
    default:
        return SB_ERR_WRONG_MODE;
    }
}

void sb_step(sb_t *s)
{
    ap_inputs_t in;
    float roll = 0.0f;

    s->last_ms = s->now_ms;
    s->now_ms = hal_millis();
    uint32_t dt_ms = s->now_ms - s->last_ms;
    float dt = (float)dt_ms * 0.001f;

    joystick_read(s, dt_ms);
    /* panel buttons + course buttons from the PC + the same functions on a USB joystick */
    uint32_t buttons = hal_get_button_presses() | s->ext_buttons | s->joy.buttons_edge;
    s->ext_buttons = 0u;
    int8_t bench_steer = bench_step(s, &buttons, dt_ms);
    read_inputs(s, &in, buttons);
    if (sb_mode(s) == MODE_BENCH) in.manual_steer = bench_steer;
    bool roll_new = hal_read_roll_rate(&roll);
    float acc[96];
    uint32_t n_acc = hal_read_accel_z(acc, 96u);
    bool nsw;
    int8_t neutral_sw = hal_read_neutral_switch(&nsw) ? (nsw ? 1 : 0) : -1;
    s->buttons_now = buttons;
    s->roll_ok = roll_new;
    if (roll_new) s->roll_dps = roll;
    float att_r, att_p;
    if (hal_read_attitude(&att_r, &att_p) && isfinite(att_r) && isfinite(att_p)) {
        s->att_roll_deg = att_r;
        s->att_pitch_deg = att_p;
        s->att_ms = s->now_ms ? s->now_ms : 1u;
    }
    s->motor_a = in.current_ok ? in.current_a : -1.0f;
    s->limit_bits = in.limits_ok ? (uint8_t)((in.limit_left ? 1u : 0u) | (in.limit_right ? 2u : 0u)) : 0x80u;
    s->neutral_sw = neutral_sw;

    /* 1. steering core (also detects compass timeout) */
    autopilot_update(&s->ap, &in);
    /* no propulsion (idle / not in forward): the heading can't be held, so
     * "off course" means nothing - don't let it build up to an alarm */
    if (!gear_is(&s->gear, GEAR_F) || s->thr.out_pct <= s->cfg.gear.idle_max_pct) s->ap.off_course_ms = 0u;

    /* position estimate: GPS when trusted, dead reckoning otherwise */
    {
        dr_input_t di;
        di.now_ms = s->now_ms;
        di.heading_deg = s->ap.heading;
        di.heading_ok = sb_compass_ok(s);
        di.yaw_dps = s->ap.gyro_valid ? s->ap.gyro_dps : s->ap.yaw_rate;
        di.throttle_pct = s->thr.out_pct;           /* what the engine got */
        di.gear = (s->gear.current == GEAR_F) ? 1 : (s->gear.current == GEAR_R) ? -1 : 0;
        di.water_ok = hal_read_water_speed(&di.water_kn);
        di.gps_new = s->gps_new_msg;
        di.gps_bad = s->gps_bad_msg;
        di.gps = s->gps.last;
        di.gps2_ok = s->gps2.rx_ms != 0u && s->now_ms - s->gps2.rx_ms < 3000u && s->gps2.age_ms < 2000u;
        di.gps2_lat = s->gps2.lat;
        di.gps2_lon = s->gps2.lon;
        dr_step(&s->dr, &di);
    }

    /* FIRST TEST: the boat moves ahead (GPS) = steering allowed in HDG LOCK / TRAINING */
    if (s->cfg.first_test) {
        s->moving_ahead = sb_gps_ok(s) && s->gps.last.sog_kn >= s->cfg.first_test_min_kn &&
                          fabsf(autopilot_heading_error(s->ap.heading, s->gps.last.cog_deg)) <= s->cfg.first_test_cog_deg;
        neutral_sw = -1;                            /* gearbox not connected: the switch is ignored */
    } else {
        s->moving_ahead = false;
    }

    /* 2. interlock, faults, joystick, buttons */
    interlock_step(s, neutral_sw, dt_ms);
    events_step(s, buttons);

    /* gateway navigation request: alarms, lease, HDG LOCK one-shot heading */
    nav_step(s);

    /* 3. what the mode wants */
    s->throttle_demand_prev = s->throttle_demand;
    s->throttle_demand = 0.0f;
    s->gear_demand = GEAR_N;
    if (sb_mode(s) == MODE_MANUAL) joystick_throttle(s);
    else if (sb_mode(s) == MODE_HDG_LOCK) hdg_lock_mode(s);
    else if (sb_mode(s) == MODE_AUTO) auto_mode(s, dt, dt_ms, roll_new, roll);
    else if (sb_mode(s) == MODE_TRAINING) training_mode(s, dt_ms);
    else if (sb_mode(s) == MODE_REMOTE) remote_mode(s);
    if (sb_mode(s) == MODE_STANDBY) {
        s->throttle_demand = 0.0f;
        s->gear_demand = GEAR_N;
    }
    if (s->cfg.first_test) {                       /* FIRST TEST: throttle / gear outputs held safe */
        s->throttle_demand = 0.0f;
        s->gear_demand = GEAR_N;
    }
#ifdef SB_TEST_HOOKS
    if (sb_mode(s) != MODE_STANDBY && s->test_drive.on) {
        s->gear_demand = s->test_drive.gear;
        s->throttle_demand = s->test_drive.throttle_pct;
    }
#endif

    /* 4. safe stop: idle + neutral first, then steering off */
    if (sb_mode(s) == MODE_STANDBY) {
        if (s->stopping) {
            /* a gear fault (neutral not confirmed) also ends the stop: the
             * engine is held at idle and the gear can't be commanded */
            bool safe = s->thr.out_pct <= s->cfg.gear.idle_max_pct &&
                        (gear_is(&s->gear, GEAR_N) || gear_fault(&s->gear));

            /* compass lost during the stop of an auto-steering mode */
            if ((s->stop_steer == STOP_STEER_HEADING || s->stop_steer == STOP_STEER_NONE) &&
                !sb_compass_ok(s) && !safe && s->stop_from_auto && s->stop_steer != STOP_STEER_CENTER) {
                fault_steer_start(s);
            }

            switch (s->stop_steer) {
            case STOP_STEER_HEADING:
                if (safe) s->stop_steer = STOP_STEER_NONE;
                break;
            case STOP_STEER_STICK:
                if (safe) s->stop_steer = STOP_STEER_NONE;
                else if (!s->joy.steer_ok) start_fault_steer(s);   /* stick failed during the stop */
                break;
            case STOP_STEER_GYRO:
                if (!autopilot_gyro_ok(&s->ap)) {       /* gyro lost too */
                    autopilot_center(&s->ap);
                    s->stop_steer = STOP_STEER_CENTER;
                } else if (safe) {
                    s->stop_steer = STOP_STEER_NONE;
                }
                break;
            case STOP_STEER_CENTER:
                if (autopilot_centered(&s->ap)) s->stop_steer = STOP_STEER_NONE;
                break;
            default:
                break;
            }
            if (s->stop_steer == STOP_STEER_NONE && s->ap.mode != AP_MODE_STANDBY)
                autopilot_disengage(&s->ap);                 /* steering off */
            if (safe && s->stop_steer == STOP_STEER_NONE) s->stopping = false;
        } else if (s->ap.mode != AP_MODE_STANDBY && s->ap.mode != AP_MODE_TRAINING) {
            autopilot_disengage(&s->ap);
        }
    }

    /* slamming: measure; reduce the forward throttle (never raise). A red
     * impact is an alarm (-> power limit down below, SAVE); yellow warns. */
    {
        bool was_over = s->slam.override;
        sb_mode_t m = sb_mode(s);
        slam_step(&s->slam, acc, n_acc, s->now_ms, dt_ms, (s->gear_demand == GEAR_F) ? s->throttle_demand : 0.0f,
                  m == MODE_MANUAL || m == MODE_HDG_LOCK);
        if (s->slam.red_edge) alarm_on(s, SB_ALM_SLAM);
        if ((s->slam.yellow_edge || (s->slam.override && !was_over)) && s->chirp_ms < 300u) s->chirp_ms = 300u;
    }

    /* power limit: any new alarm (also from the PC link) -> startup limit
     * at once (the throttle then comes down at its normal rate, servo.c);
     * a long time at low speed too */
    if (gear_fault(&s->gear)) alarm_on(s, SB_ALM_GEAR);
    {
        uint32_t fresh = s->alarm_bits & ~s->alarm_bits_seen;
        bool any = s->alarm || s->ap.alarm;
        /* the one exception: slamming RED in STEP mode reduces in steps (slam.c),
         * the power limit stays - a sudden drop can be dangerous in a following sea */
        if (s->slam.cfg.red_reset < 0.5f) {
            fresh &= ~SB_ALM_SLAM;
            if ((s->alarm_bits & ~SB_ALM_SLAM) == 0u && !s->ap.alarm) any = false;
        }
        /* navigation advisories never touch the power limit (their reaction is
         * defined explicitly: AIS silent = warning only) */
        fresh &= ~SB_NAV_ADVISORY;
        if ((s->alarm_bits & ~(SB_NAV_ADVISORY | (s->slam.cfg.red_reset < 0.5f ? SB_ALM_SLAM : 0u))) == 0u &&
            !s->ap.alarm) any = false;
        if (fresh & SB_ALM_PC_LINK) plim_drop(&s->plim, PLIM_SRC_PC_LINK, s->now_ms);
        else if (fresh || (any && !s->alarm_seen)) plim_drop(&s->plim, PLIM_SRC_ALARM, s->now_ms);
        if ((fresh || (any && !s->alarm_seen)) && is_raise(s->confirm.kind)) s->confirm.kind = CONF_NONE;
        s->alarm_bits_seen = s->alarm_bits;
        s->alarm_seen = any;
        plim_step(&s->plim, speed_now(s), dt_ms, s->now_ms);
    }
    if (sb_mode(s) != MODE_AUTO) { s->speed_limited = false; s->sat_ms = 0u; }

    /* 5. gear and throttle (order: throttle limited by the gear state),
     *    demand limited by the power limit of its direction */
    s->throttle_mode = s->throttle_demand;
    float lim_dir = (s->gear_demand == GEAR_F) ? fminf(s->plim.fwd_pct, slam_cap(&s->slam)) : s->plim.rev_pct;
    if (s->throttle_demand > lim_dir) s->throttle_demand = lim_dir;
    gear_request(&s->gear, s->gear_demand);
    /* STANDBY (button, fault, alarm ...): the faster closing rate; a planned
     * arrival stop and normal driving keep the normal one */
    s->thr.stop = sb_mode(s) == MODE_STANDBY && !s->stop_planned;
    s->throttle_rate = throttle_step(&s->thr, s->throttle_demand, s->gear.current, gear_throttle_allowed(&s->gear), dt);

    /* 6. outputs: engine first, steering last. The servo module clamps to
     *    the power limit once more (lowest layer); the rate limiter goes on
     *    from what was really sent. */
    s->thr.out_pct = servo_throttle_write(&s->servo, &s->plim, gear_is(&s->gear, GEAR_F), s->thr.out_pct, dt);
    gear_step(&s->gear, dt_ms, s->thr.out_pct, neutral_sw);
    if (gear_fault(&s->gear)) {
        alarm_on(s, SB_ALM_GEAR);
        /* neutral not confirmed: driving modes end in STANDBY (throttle
         * stays idle); MANUAL keeps the steering on the stick */
        if (sb_mode(s) != MODE_STANDBY && sb_mode(s) != MODE_MANUAL)
            sb_request_mode(s, MODE_STANDBY, WHY_GEAR_FAULT);
    }
    hal_set_gear_pwm(s->gear.pwm_us);
    hal_set_rudder_motor(s->ap.motor_cmd);
    if (s->chirp_ms > 0u) s->chirp_ms = (s->chirp_ms > dt_ms) ? s->chirp_ms - dt_ms : 0u;
    hal_set_alarm(s->alarm || s->ap.alarm || s->chirp_ms > 0u);

    /* 7. publish for the independent safety monitor (CPU1), as one set (seqlock) */
    monitor_write_begin(&g_mon);
    g_mon.throttle_pct = s->servo.out_pct;
    g_mon.fwd_limit = s->plim.fwd_pct;
    g_mon.rev_limit = s->plim.rev_pct;
    g_mon.fwd_engaged = gear_is(&s->gear, GEAR_F) ? 1u : 0u;
    g_mon.gear_pwm_us = s->gear.pwm_us;
    g_mon.idle_max_pct = s->cfg.gear.idle_max_pct;
    g_mon.thr_idle_us = s->cfg.servo.idle_us;
    g_mon.gear_n_us = s->cfg.gear.pwm_n_us;
    monitor_write_end(&g_mon);
    if (s->bench.trip_req) g_mon.test_trip = MON_TEST_TRIP;     /* BENCH TEST: trip test (latched) */
    g_mon.heartbeat = g_mon.heartbeat + 1u;
    g_mon.running = MON_RUNNING;                    /* the control loop runs (monitor watches from now) */

    /* 8. video SAVE requests for the gateway */
    save_triggers(s);
}

sb_status_t sb_status(const sb_t *s)
{
    sb_status_t st;
    st.mode = sb_mode(s);
    st.pos_source = s->dr.state;
    st.dr_reason = s->dr.why;
    st.unc_m = s->dr.unc_m;
    st.since_fix_s = dr_time_since_fix_s(&s->dr, s->now_ms);
    st.dr_time_s = dr_dr_time_s(&s->dr, s->now_ms);
    if (dr_have_position(&s->dr)) dr_latlon(&s->dr, &st.lat, &st.lon);
    else { st.lat = 0.0; st.lon = 0.0; }
    st.sog_kn = dr_sog_kn(&s->dr);
    st.cog_deg = dr_cog_deg(&s->dr);
    st.alarm_bits = s->alarm_bits | (s->ap.alarm ? SB_ALM_COMPASS : 0u);
    st.gps_suspects = s->dr.suspects;
    return st;
}
