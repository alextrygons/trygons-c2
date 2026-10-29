/*
 * smallboat.c - Small-boat controller. See smallboat.h.
 */
#include "smallboat.h"

#include <math.h>
#include <stddef.h>

#include "monitor.h"
#include "timing.h"
#include "logger.h"
#include "nv.h"
#include <string.h>

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

#ifdef SB_TEST_HOOKS
static bool s_test_boot_idle;   /* sim tests only (sb_test_boot_idle) */
#endif

/* "Idle" for every rule that waits for it (gear change, steering off after a stop, ENGINE KILL, STARTER,
 * generator, DP, restart): the MODELLED throttle servo position at idle (servo.c est_pct: the command at
 * idle AND long enough for the worst-case servo speed; DroneCAN: unknown after a boot / with the throttle
 * node lost) - never the command alone (CLAUDE.md Safety rules, docs/dronecan_design.md 7, step 11e). */
static bool thr_idle(const sb_t *s)
{
    return servo_idle(&s->servo, s->cfg.gear.idle_max_pct);
}

/* DroneCAN: the throttle node OPERATIONAL (seen and not lost); always true on the PWM hardware */
static bool thr_node_ok(const sb_t *s)
{
    if (!s->servo.can) return true;
    const hal_can_node_t *n = &s->can.node[HAL_CAN_ROLE_THROTTLE];
    return s->can.fitted && n->seen && !n->lost;
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
    cfg->profile_ok_ms = 200u;
    cfg->profile_detect_ms = 6000u;
    cfg->profile_switch_ms = 10000u;
    cfg->near_shore_m = 100.0f;                 /* TO CONFIRM */
    cfg->lostlink_grace_s = 10.0f;              /* TO CONFIRM */
    cfg->shore_max_age_ms = 3000u;
    batt_default_config(&cfg->batt);
    gen_default_config(&cfg->gen);
    cfg->dp_max_kn = 3.0f;                      /* TO CONFIRM */
    cfg->dp_max_entry_kn = 2.0f;
}

/* NV slot 6: the generator's learned value (version + CRC-32, like calib.c):
 * a record that fails the check = defaults + NV DATA RESET, never half used */
#define GEN_NV_VERSION 1u
typedef struct { uint8_t version; uint8_t pct; uint8_t pad[2]; uint32_t crc; } gen_nv_t;

static uint32_t gen_nv_crc(const gen_nv_t *r)
{
    const uint8_t *p = (const uint8_t *)r;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < offsetof(gen_nv_t, crc); i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static bool gen_nv_valid(const gen_nv_t *r)
{
    return r->version == GEN_NV_VERSION && r->pct > 0u && (float)r->pct <= GEN_CEILING_PCT && r->crc == gen_nv_crc(r);
}

static bool lever_has_throttle(sb_mode_t m);
static void lever_pickup_start(sb_t *s, sb_mode_t from);
static void remote_enter(sb_t *s, sb_mode_t from);
static bool remote_like(sb_mode_t m);
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
    s->joy_probe.cfg = s->joy.cfg;
    jb_init(&s->jb, &s->cfg.jb, s->now_ms);
    joy_disarm(&s->joy);
    joy_disarm(&s->joy_probe);
    lever_standard(s);
}

void sb_bootnv_begin(sb_bootnv_t *b, uint32_t now_ms, uint32_t max_ms, bool can_tick)
{
    b->t0 = now_ms;
    b->next_tick = now_ms;
    b->max_ms = max_ms;
    b->ticks = 0u;
    b->can_tick = can_tick;
}

bool sb_bootnv_step(sb_bootnv_t *b, uint32_t now_ms)
{
    if (!hal_nv_pending() || !hal_nv_ok() || now_ms - b->t0 >= b->max_ms) return false;
    hal_nv_service(true);
    hal_watchdog_kick();
    if (b->can_tick && (int32_t)(now_ms - b->next_tick) >= 0) {
        hal_can_tick();                         /* throttle IDLE every 10 ms while the flash works (13f MED-1) */
        b->ticks++;
        b->next_tick += 10u;
        if ((int32_t)(now_ms - b->next_tick) >= 0) b->next_tick = now_ms + 10u;   /* late: never a burst */
    }
    return true;
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
    batt_init(&s->batt, &cfg->batt);
    plim_init(&s->plim, &cfg->plim, s->now_ms * 2654435761u + 12345u);     /* always 30 / 5 */
    hal_set_safety_relay(false);                /* closed once armed (interlock) */
    servo_init(&s->servo, &cfg->servo);
#ifdef SB_TEST_HOOKS
    if (s_test_boot_idle && s->servo.can) s->servo.est_pct = 0.0f;  /* sim tests only: the throttle known at idle */
#endif
    s->gear_demand = GEAR_N;
    dr_init(&s->dr, &cfg->dr);
    s->dr.cfg.gps_timeout_ms = cfg->gps_timeout_ms;
    /* stored settings: a record that loads but fails its version / checksum
     * check = defaults + the notice NV DATA RESET (nv_invalid) */
    nv_init();
    {
        joy_cal_t cal;
        bool have = false;
        if (nv_load(HAL_NV_SLOT_JOYSTICK, &cal, (uint32_t)sizeof cal)) {
            have = joy_cal_valid(&cal);
            if (!have) nv_invalid(HAL_NV_SLOT_JOYSTICK);
        }
        joy_init(&s->joy, &cfg->joy, have ? &cal : NULL);
        joy_init(&s->joy_probe, &cfg->joy, have ? &cal : NULL);
    }
    s->boot_ms = s->now_ms;
    /* the ENGINE KILL latch (step 11b): kept through a watchdog / software reset; only a real
     * power-on reset (or the HELM station's remote restart) clears it */
    if (hal_reset_reason(NULL) != HAL_RESET_POWER_ON && hal_kill_keep_get()) {
        s->ll.state = SB_LL_KILLED;
        s->ll.kept = true;
        s->alarm_bits |= SB_ALM_ENGINE_KILLED;
        s->alarm = true;
        hal_set_engine_kill(true);
    } else {
        hal_kill_keep_set(false);
        hal_set_engine_kill(false);
    }
    engine_init(&s->eng);
    thruster_init(&s->thrust);                  /* no settings = no thruster fitted (the station pushes them) */
    dp_init(&s->dp);
    /* FIRST TEST is always CREWED (steering only, its own arming rule; no lost-link kill) */
    s->profile = cfg->first_test ? SB_PROFILE_CREWED : SB_PROFILE_DETECTING;
    if (nv_load(HAL_NV_SLOT_JOYMAP, &s->jmap, (uint32_t)sizeof s->jmap)) {
        if (!joymap_valid(&s->jmap)) { nv_invalid(HAL_NV_SLOT_JOYMAP); joymap_default(&s->jmap); }
    } else {
        joymap_default(&s->jmap);
    }
    joymap_apply(s);
    if (nv_load(HAL_NV_SLOT_PROGRAM, &p, (uint32_t)sizeof p)) {
        if (program_validate(&p, &s->cfg.prog, &bad) == PROG_OK) {
            s->autop.prog = p;
            s->autop.prog_ok = true;
        } else {
            nv_invalid(HAL_NV_SLOT_PROGRAM);
        }
    }
    /* generator mode: auto OFF after every power-up; the learned value (NV slot 6) is its first step */
    {
        gen_nv_t r;
        float learned = 0.0f;
        if (nv_load(HAL_NV_SLOT_GENERATOR, &r, (uint32_t)sizeof r)) {
            if (gen_nv_valid(&r)) learned = (float)r.pct;
            else nv_invalid(HAL_NV_SLOT_GENERATOR);
        }
        gen_init(&s->gen, &cfg->gen, learned);
    }
    /* relay board: the stored table (NV slot 7) stays in use until a different one is
     * confirmed; power_up states only after the AURIX's own power-on reset (relays.h) */
    {
        static relay_table_t t;     /* 140 bytes: not on the stack */
        bool have = nv_load(HAL_NV_SLOT_RELAYS, &t, (uint32_t)sizeof t);
        if (have && (!relay_table_valid(&t) || !relays_table_fits(&t))) { nv_invalid(HAL_NV_SLOT_RELAYS); have = false; }
        relays_init(&s->relays, have ? &t : NULL, hal_reset_reason(NULL) == HAL_RESET_POWER_ON, cfg->first_test);
    }
    /* DroneCAN steering POSITION servo (step 11e): the model from the TRAINING record (NV slot 5 version 2;
     * a version-1 DC-motor record = defaults + NV DATA RESET + TRAINING needed); the position after a WARM
     * reset from reset-surviving RAM (the node held it), after a power-on UNKNOWN - no steering frame either
     * way until a mode commands steering (OD-7) */
    s->steer.needs_training = false;
    if (s->servo.can) {
        sm_params_t p;
        float half = SM_HALF_DEG;
        calib2_t c;
        bool v1 = false;
        sm_defaults(&p);
        s->steer.needs_training = true;
        if (calib2_load(&c, &v1)) {
            p.r_right = c.r_right;
            p.r_left = c.r_left;
            p.b_lr = c.b_lr;
            p.b_rl = c.b_rl;
            p.k = c.k;
            p.tau = c.tau;
            p.a0 = c.a0;
            p.v_ref = c.v_ref_kn;
            p.res = c.res_dps;
            p.trained = true;
            if (c.full_angle_deg > 0.0f) half = 0.5f * c.full_angle_deg;
            s->ap.servo_calib = c;
            s->steer.needs_training = false;
        }
        s->steer.v1_refused = v1;
        if (v1) { s->steer.log_what = 3u; s->steer.log_seq++; }
        float p0 = 0.0f;
        bool warm = hal_reset_reason(NULL) != HAL_RESET_POWER_ON;
        bool known = warm && hal_steer_keep_get(&p0);
        if (!warm) hal_steer_keep_set(false, 0.0f);
        autopilot_set_servo(&s->ap, &p, half, known, p0);
        if (p.trained) autopilot_apply_servo_params(&s->ap, &p, half);
        s->ap.servo_boot = g_logstat.boot;
    }
    /* step 13d (HIGH-1): CPU1's safe pulses from the start - a monitor trip before the first control step
     * forces THIS idle / neutral (Cpu0_Main.c also publishes them before the CPU sync event) */
    monitor_publish_safe(&g_mon, s->cfg.servo.idle_us, s->cfg.gear.pwm_n_us);
}

sb_result_t sb_thr_travel_set(sb_t *s, float seconds)
{
    if (!(seconds >= SERVO_THR_TRAVEL_MIN && seconds <= SERVO_THR_TRAVEL_MAX)) return SB_ERR_BAD_VALUE;
    if (sb_mode(s) != MODE_STANDBY || s->stopping) return SB_ERR_STANDBY_ONLY;
    if (seconds >= s->servo.cfg.travel_s) {     /* longer = safer: at once */
        s->servo.cfg.travel_s = seconds;
        s->travel.changes++;
        s->steer.log_why = 0u;
        s->steer.log_what = 4u;
        s->steer.log_seq++;
        return SB_OK;
    }
    /* shorter: only an owner-signed value - the CONFIRM on the joystick (it never changes the mode) */
    if (s->confirm.kind != CONF_NONE && s->confirm.kind != CONF_THR_TRAVEL) return SB_ERR_REFUSED;
    if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;
    s->travel.pending_s = seconds;
    s->confirm.kind = CONF_THR_TRAVEL;
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
    /* step 11g: the asked (shorter, still pending) value is logged too - why = the value in 0.1 s */
    s->steer.log_what = 5u;
    {   /* (step 13b: clamped - travel_s goes up to SERVO_THR_TRAVEL_MAX 30 s, 25.5 s and more log as 255) */
        long v = lroundf(seconds * 10.0f);
        s->steer.log_why = (uint8_t)(v > 255 ? 255 : (v < 0 ? 0 : v));
    }
    s->steer.log_seq++;
    return SB_OK;
}

sb_result_t sb_confirm_only(sb_t *s, uint8_t station, uint16_t token)
{
    /* the academy practical test's final approval (ACADEMY_SPEC G): a question for the onboard joystick's
     * CONFIRM that changes NOTHING - no mode, power limit, output, relay, program or setting */
    if (s->cfg.first_test) return SB_ERR_WRONG_MODE;
    if (s->profile != SB_PROFILE_CREWED) return SB_ERR_NOT_ALLOWED;        /* the ONBOARD joystick only */
    if (s->confirm.kind != CONF_NONE && s->confirm.kind != CONF_CHECK) return SB_ERR_REFUSED;
    if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;
    s->conf_only.token = token;
    s->conf_only.station = station;
    s->conf_only.state = SB_CO_PENDING;
    s->conf_only.asked++;
    s->confirm.kind = CONF_CHECK;
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
    return SB_OK;
}

sb_result_t sb_thruster_cfg(sb_t *s, const thr_cfg_t *c, uint16_t hash)
{
    if (sb_mode(s) != MODE_STANDBY || s->stopping || s->thrust.active) return SB_ERR_STANDBY_ONLY;
    thruster_cfg_set(&s->thrust, c, hash);
    return SB_OK;
}

sb_result_t sb_relay_table(sb_t *s, const relay_table_t *t)
{
    bool standby_idle = sb_mode(s) == MODE_STANDBY && !s->stopping;
    /* another CONFIRM is pending (power raise, AUTO, REMOTE ...): a table offer never replaces
     * it - the same table is still OK at once, a different one is refused (the station tries
     * again later; its relay_push waits while any CONFIRM is pending) */
    bool other = s->confirm.kind != CONF_NONE && s->confirm.kind != CONF_RELAY_TABLE;
    if (other && relay_table_valid(t) && relays_table_fits(t) && !relays_offer_same(&s->relays, t))
        return SB_ERR_REFUSED;
    switch (relays_offer(&s->relays, t, standby_idle)) {
    case RELAY_OFFER_SAME:
        if (s->confirm.kind == CONF_RELAY_TABLE) s->confirm.kind = CONF_NONE;
        return SB_OK;
    case RELAY_OFFER_STANDBY: return SB_ERR_STANDBY_ONLY;
    case RELAY_OFFER_BAD:     return SB_ERR_BAD_VALUE;
    case RELAY_OFFER_STOP:    return SB_ERR_RELAY_STOP;   /* step 13g: no CONFIRM asked (it could only fail) */
    default: break;
    }
    if (!sb_confirm_ready(s)) {                 /* nobody can confirm: the offer is dropped */
        relays_cancel_offer(&s->relays);
        return SB_ERR_JOY_NOT_READY;
    }
    s->confirm.kind = CONF_RELAY_TABLE;         /* (no other request is pending: checked above) */
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
    return SB_OK;
}

static void eng_cond(const sb_t *s, eng_cond_t *c, bool standby_button);
static void can_poll(sb_t *s);

sb_result_t sb_relay_cmd(sb_t *s, uint8_t n, uint8_t action, uint16_t ms)
{
    relay_result_t res;
    const relay_def_t *d = s->relays.have_table ? relay_table_find(&s->relays.table, n) : NULL;
    if (d != NULL && action <= RELAY_CMD_PULSE &&
        (d->role == RELAY_ROLE_IGNITION || d->role == RELAY_ROLE_SHUTOFF)) {
        /* IGNITION / SHUTOFF: through the engine rules (section N: IGNITION ON = idle + neutral +
         * e-stop released; SHUTOFF ON = stop the engine, always - step 11b) */
        eng_cond_t c;
        eng_cond(s, &c, false);
        eng_why_t why = engine_relay(&s->eng, &c, &s->relays, n, action != RELAY_CMD_OFF,
                                     d->role == RELAY_ROLE_SHUTOFF, &res);
        if (why == ENG_WHY_FIRST_TEST) return SB_ERR_WRONG_MODE;
        if (why == ENG_WHY_ESTOP) return SB_ERR_REFUSED;
        if (why != ENG_WHY_NONE && why != ENG_WHY_RELAY) return SB_ERR_PRECOND;
    } else {
        res = relays_cmd(&s->relays, n, action, ms, s->now_ms);
    }
    switch (res) {
    case RELAY_R_OK:         return SB_OK;
    case RELAY_R_NO_TABLE:   return SB_ERR_PRECOND;
    case RELAY_R_NOT_FITTED: return SB_ERR_PRECOND;
    case RELAY_R_UNKNOWN:    return SB_ERR_BAD_VALUE;
    case RELAY_R_SILENT:     return SB_ERR_REFUSED;
    case RELAY_R_PAIR:       return SB_ERR_REFUSED;
    case RELAY_R_ENGINE:     return SB_ERR_NOT_ALLOWED;
    case RELAY_R_FIRST_TEST: return SB_ERR_WRONG_MODE;
    default:                 return SB_ERR_REFUSED;
    }
}

sb_result_t sb_generator_set(sb_t *s, bool on)
{
    if (on) {
        if (s->cfg.first_test) return SB_ERR_WRONG_MODE;              /* FIRST TEST: throttle stays idle */
        if (!(s->gen.cfg.max_pct > 0.0f)) return SB_ERR_PRECOND;      /* gen_max_pct 0 = disabled */
    }
    gen_set(&s->gen, on);
    return SB_OK;
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
    if (s->ll.state == SB_LL_LOST) keep |= SB_ALM_LINK_LOST;
    if (s->ll.state == SB_LL_KILLED) keep |= SB_ALM_ENGINE_KILLED | SB_ALM_LINK_LOST;
    s->alarm_bits &= keep;              /* (the link-lost bits are raised again while lost, link.c) */
    uint32_t keep2 = 0u;                /* the second word, the same rule */
    if (s->batt.low) keep2 |= SB_ALM2_LOW_BATTERY;
    if (relays_silent(&s->relays)) keep2 |= SB_ALM2_RELAY_SILENT;
    if (thruster_any_hot(&s->thrust)) keep2 |= SB_ALM2_DP_HEAT;
    if (s->dp.offspot) keep2 |= SB_ALM2_DP_OFFSPOT;
    if (s->dp.overpower) keep2 |= SB_ALM2_DP_OVERPOWER;
    if (sb_mode(s) == MODE_STANDBY) {   /* a rebooted CAN node: its latch goes only in STANDBY, OPERATIONAL 2 s */
        hal_can_ack();
        can_poll(s);
    }
    if (s->can.node_lost) keep2 |= SB_ALM2_CAN_NODE;
    if (s->can.bus_bad) keep2 |= SB_ALM2_CAN_BUS;
    s->alarm_bits2 &= keep2;
    s->alarm = s->alarm_bits != 0u || (s->alarm_bits2 & ~SB_ALM2_QUIET) != 0u;
    if (!s->ap.sensor_fault) s->ap.alarm = false;
}

#ifdef SB_TEST_HOOKS
void sb_test_boot_idle(bool on)
{
    s_test_boot_idle = on;
}

void sb_test_drive(sb_t *s, bool on, gear_t gear, float throttle_pct)
{
    s->test_drive.on = on;
    s->test_drive.gear = gear;
    s->test_drive.throttle_pct = throttle_pct;
}

void sb_test_thrust(sb_t *s, bool on, float bow_pct, float stern_pct)
{
    s->test_thrust.on = on;
    s->test_thrust.bow_pct = bow_pct;
    s->test_thrust.stern_pct = stern_pct;
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

/* the second alarm word (SB_ALM2_). The QUIET advisories (RELAY BOARD SILENT)
 * never set s->alarm: no buzzer, no power-raise block, no power-limit drop -
 * only the bit (shown on the screens). */
static void alarm2_on(sb_t *s, uint32_t bits)
{
    if (bits & ~SB_ALM2_QUIET) s->alarm = true;
    s->alarm_bits2 |= bits;
}

/* --- gateway navigation request (section D) --------------------------------- */

static bool nav_fresh(const sb_t *s)
{
    return s->nav.rx_ms != 0u && s->now_ms - s->nav.rx_ms <= s->nav.lease_ms;
}

/* The navigation request applies (speed cap, avoidance heading, the lost lease) in AUTO and in a DP TRIP
 * (section P: "collision avoidance navreq as in AUTO"); in SPOT / JOG / CRUISE and every other mode it is
 * an alarm only. */
static bool nav_applies(const sb_t *s)
{
    return sb_mode(s) == MODE_AUTO || (sb_mode(s) == MODE_DP && s->dp.sub == DP_TRIP);
}

/* Speed cap (kn) the navigation request puts on AUTO / a DP TRIP; < 0 = none. */
static float nav_cap(const sb_t *s)
{
    if (!nav_applies(s)) return -1.0f;
    if (s->nav.lost) {                  /* never faster than the last cap: a slowdown stays */
        float c = s->cfg.nav_lost_kn;
        return (s->nav.lost_cap_kn >= 0.0f && s->nav.lost_cap_kn < c) ? s->nav.lost_cap_kn : c;
    }
    if (nav_fresh(s) && (s->nav.kind == SB_NAV_CAP || s->nav.kind == SB_NAV_HEADING)) return s->nav.cap_kn;
    return -1.0f;
}

/* AUTO / DP TRIP: the heading the navigation request wants (the avoidance heading, or the held
 * heading when the request was lost); false = none. track = the leg's track. */
static bool nav_heading(sb_t *s, float track, float *hdg)
{
    if (!nav_applies(s)) return false;
    if (s->nav.lost) {
        *hdg = s->nav.lost_heading;
        return true;
    }
    if (nav_fresh(s) && s->nav.kind == SB_NAV_HEADING) {
        /* the Pi may turn the boat away from the leg, but not anywhere: too far
         * off the track -> not steered (the cap still slows), DRIVER DECISION */
        if (fabsf(autopilot_heading_error(track, s->nav.heading)) <= s->cfg.nav_max_dev_deg) {
            *hdg = s->nav.heading;
            return true;
        }
        if (s->nav.too_far_seq != s->nav.seq || !s->nav.too_far) s->nav.too_far++;
        s->nav.too_far_seq = s->nav.seq;
        alarm_on(s, SB_ALM_DECISION);
    }
    return false;
}

/* AUTO: the avoidance heading (or the held heading when the request was lost). */
static void nav_steer(sb_t *s)
{
    float h;
    if (sb_mode(s) == MODE_AUTO && nav_heading(s, s->autop.leg.track_deg, &h)) autopilot_set_target(&s->ap, h);
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
    if (!nav_applies(s)) n->lost = false;
    if (fresh) {
        n->lost = false;
        if (n->alarm & 1u) alarm_on(s, SB_ALM_COLLISION);
        if (n->alarm & 2u) alarm_on(s, SB_ALM_DECISION);
        if ((n->alarm & 4u) && nav_applies(s)) alarm_on(s, SB_ALM_SHALLOW);
        if (n->alarm & 8u) alarm_on(s, SB_ALM_OBSTACLE);
        if (n->kind == SB_NAV_HDG_SET && (!n->hdg_any || n->seq != n->hdg_seq_done) && sb_mode(s) == MODE_HDG_LOCK &&
            n->rx_mode == (uint8_t)MODE_HDG_LOCK && !s->hdg_compass_lost) {
            n->hdg_seq_done = n->seq;
            n->hdg_any = true;
            n->applied_hdg++;
            s->hdg_target = n->heading;
            autopilot_set_target(&s->ap, n->heading);
        }
    } else if (nav_applies(s) && !n->lost && n->rx_ms != 0u &&
               (n->kind == SB_NAV_CAP || n->kind == SB_NAV_HEADING)) {
        /* the gateway went quiet in the middle of an avoidance: don't carry on blindly */
        n->lost = true;
        n->losses++;
        n->lost_heading = (sb_mode(s) == MODE_DP) ? s->dp.hdg_est : s->ap.heading;
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
           w == WHY_TRAINING_ABORT || w == WHY_PROFILE || w == WHY_LINK_LOST || w == WHY_DP_OFFSPOT ||
           w == WHY_DP_THRUSTER || w == WHY_CAN_NODE || w == WHY_CAN_BUS;
}

/* Rising edges only; the AURIX never waits for the Pi. */
static void save_triggers(sb_t *s)
{
    sb_save_t *v = &s->save;
    bool gf = gear_fault(&s->gear), cf = s->ap.sensor_fault, jf = s->joy.fault, mf = s->mon_tripped;
    if (!v->init) {
        v->init = true;
        v->k_alarms = s->alarm_bits;
        v->k_alarms2 = s->alarm_bits2;
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
    /* the second word: advisories (LOW BATTERY) are no incidents; arg = 32 + bit */
    uint32_t on2 = s->alarm_bits2 & ~v->k_alarms2 & ~SB_ALM2_ADVISORY;
    if (on2) {
        uint8_t bit = 0u;
        while (!(on2 & (1u << bit))) bit++;
        save_req(s, SB_SAVE_ALARM, (uint8_t)(32u + bit));
    }
    v->k_alarms2 = s->alarm_bits2;
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
                  e->from == MODE_TRAINING || e->from == MODE_MANEUVER || e->from == MODE_DP))
            save_req(s, SB_SAVE_TAKEOVER, (uint8_t)e->from);
    }
}

/* --- modes ---------------------------------------------------------------- */

static bool joy_ready(const sb_t *s)
{
    return s->joy.present && s->joy.armed && !s->joy.fault;
}

static bool shore_ready(const sb_t *s);

bool sb_confirm_ready(const sb_t *s)
{
    if (s->profile == SB_PROFILE_UNCREWED) return shore_ready(s);
    return s->profile == SB_PROFILE_CREWED && joy_ready(s);
}

/* The active waypoint can be reached from here (position known, leg not
 * absurdly long). */
static bool wp_reachable(const sb_t *s, const prog_wp_t *w)
{
    double lat, lon;
    if (!s->autop.prog_ok || !dr_have_position(&s->dr)) return false;
    dr_latlon(&s->dr, &lat, &lon);
    return program_dist_m(lat, lon, program_lat(w), program_lon(w)) <= s->cfg.prog.max_leg_m;
}

static bool auto_leg_ok(const sb_t *s)
{
    const sb_auto_t *a = &s->autop;
    return a->prog_ok && a->wp < a->prog.n && wp_reachable(s, &a->prog.wp[a->wp]);
}

const program_t *sb_program(const sb_t *s)
{
    return s->autop.returning ? &s->autop.orig : &s->autop.prog;
}

uint8_t sb_auto_wp_stored(const sb_t *s)
{
    const sb_auto_t *a = &s->autop;
    if (!a->returning) return a->wp;
    return a->wp <= a->ret_base ? (uint8_t)(a->ret_base - a->wp) : 0u;
}

/* DP TRIP: waypoint wp can be reached from here (a valid program, position known, leg not absurd) */
static bool dp_trip_ok(const sb_t *s, uint8_t wp)
{
    const program_t *p = sb_program(s);              /* (the stored program also during an AUTO RETURN) */
    if (!s->autop.prog_ok || wp >= p->n || !sb_gps_ok(s)) return false;
    const prog_wp_t *w = &p->wp[wp];
    return program_dist_m(s->gps.last.lat_deg, s->gps.last.lon_deg, program_lat(w), program_lon(w)) <=
           s->cfg.prog.max_leg_m;
}

/* DP may start (entry: from STANDBY, slow) / change to sub now: GPS trusted (no dead reckoning in DP),
 * compass, both thrusters fitted (boat file settings) and not hot, allowed (no trip, e-stop released,
 * outputs not forced), no gear fault, no lost link; TRIP: the program's waypoint reachable. */
static bool dp_ok(const sb_t *s, uint8_t sub, bool cont, bool entry)
{
    if (!sb_gps_ok(s) || !sb_compass_ok(s) || gear_fault(&s->gear) || s->mon_tripped || !s->armed) return false;
    if (s->estop_state == 2u || s->ll.state != SB_LL_OK || hal_outputs_forced() || s->cfg.first_test) return false;
    if (!thruster_fitted(&s->thrust, THR_BOW) || !thruster_fitted(&s->thrust, THR_STERN) ||
        thruster_any_hot(&s->thrust)) return false;
    if (entry && !(s->gps.last.sog_kn < s->cfg.dp_max_entry_kn)) return false;
    /* step 11g: the outboard at the MODELLED idle (thr_est: DroneCAN unknown after a boot / a lost throttle node =
     * not idle) and in neutral - the one exception is the generator's own neutral run into SPOT (it may run in
     * SPOT / JOG, section K; gear N confirmed by the switch, CPU1 knows its cap) */
    if (entry && !gear_is(&s->gear, GEAR_N)) return false;
    if (entry && !thr_idle(s) && !(sub == DP_SPOT && s->gen.state == GEN_RUNNING && s->neutral_sw == 1)) return false;
    if (sub == DP_TRIP) return dp_trip_ok(s, cont ? s->dp.wp : 0u);
    return sub == DP_SPOT || sub == DP_CRUISE;
}

/* BENCH TEST: the GPS says the boat moves (trusted fix, SOG above bench_max_kn) */
static bool bench_moving(const sb_t *s)
{
    return sb_gps_ok(s) && s->gps.last.sog_kn > s->cfg.bench_max_kn;
}

static bool precondition(const sb_t *s, sb_mode_t to, mode_reason_t why)
{
    bool forced = why == WHY_STANDBY_BUTTON || why == WHY_JOY_FAULT || why == WHY_GEAR_FAULT ||
                  why == WHY_NOT_ARMED || why == WHY_MONITOR || why == WHY_ESTOP || why == WHY_CAN_NODE ||
                  why == WHY_CAN_BUS;
    if (s->estop_state == 2u && to != MODE_STANDBY) return false;     /* emergency stop pressed */
    /* DroneCAN: a required node lost / not seen yet, a bus fault, our ID on the bus -> STANDBY only */
    if ((s->can.node_block || s->can.bus_bad) && to != MODE_STANDBY) return false;
    if (s->ll.state != SB_LL_OK && to != MODE_STANDBY) return false;  /* UNCREWED link lost / engine killed */
    /* FIRST TEST: steering only - STANDBY, TRAINING, HDG LOCK */
    if (s->cfg.first_test && (to == MODE_MANUAL || to == MODE_AUTO || to == MODE_REMOTE || to == MODE_MANEUVER ||
                              to == MODE_DP))
        return false;
    /* UNCREWED: the onboard joystick is ignored (no MANUAL), nobody on board for the HDG LOCK button */
    if (s->profile == SB_PROFILE_UNCREWED && (to == MODE_MANUAL || to == MODE_HDG_LOCK)) return false;

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
    if (to == MODE_MANEUVER) return why == WHY_MANEUVER && !gear_fault(&s->gear);

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
    case MODE_DP:           /* HELM station request + CONFIRM, from STANDBY (not stopping) */
        return why == WHY_DP && !s->stopping && dp_ok(s, s->dp_req.sub, s->dp_req.cont, true);
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
    if (from == MODE_REMOTE || from == MODE_MANEUVER) s->remote.state = SB_REMOTE_OFF;
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
        bool moving = !thr_idle(s) || !gear_is(&s->gear, GEAR_N);
        bool was_auto = (from == MODE_HDG_LOCK || from == MODE_AUTO || from == MODE_TRAINING ||
                         from == MODE_REMOTE || from == MODE_MANEUVER);
        s->stop_from_auto = was_auto;
        s->stopping = true;
        /* a STANDBY (button, fault, HELM lost ...) cancels any pending CONFIRM request:
         * a CONFIRM in the same step must not start what was asked before it */
        s->confirm.kind = CONF_NONE;
        if (hdg_fault_steer) break;                 /* keep the gyro hold / centring running */
        s->stop_steer = STOP_STEER_NONE;
        if (s->ap.servo && from == MODE_TRAINING) {
            /* the position servo's TRAINING stopped (STANDBY / an abort / a fault): the servo back to the centre
             * estimate at the measured / worst-case speed, then the STANDBY rules (hold) */
            if (s->ap.mode == AP_MODE_TRAINING) trs_abort(&s->ap.trs, TR_ABORT_USER);
            autopilot_center(&s->ap);
            s->stop_steer = STOP_STEER_CENTER;
            break;
        }
        if (from == MODE_MANUAL && moving) {
            /* the onboard stick keeps steering until idle + neutral (a held
             * stick is disarmed for taking control, not for this); stick
             * gone (fault): gyro hold / centring */
            if (s->joy.steer_ok) s->stop_steer = STOP_STEER_STICK;
            else start_fault_steer(s);
        } else if ((from == MODE_REMOTE || from == MODE_MANEUVER) && moving) {
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
        if (from == MODE_HDG_LOCK || from == MODE_AUTO || from == MODE_TRAINING || from == MODE_DP)
            s->alarm_bits |= SB_ALM_JOYSTICK;
        break;
    case MODE_REMOTE:
    case MODE_MANEUVER:
        /* the shore joystick: angle steering like MANUAL, its own lever / caps
         * (MANEUVER: gear only, the throttle stays idle) */
        s->stopping = false;
        s->manual_from = from;
        autopilot_angle(&s->ap);
        remote_enter(s, from);
        break;
    case MODE_DP: {
        /* the thrusters hold / move the boat; the outboard stays idle + neutral (no demand) with its
         * steering off; the anchor / heading / route from where the boat is now */
        s->stopping = false;
        autopilot_disengage(&s->ap);
        uint8_t from_wp = s->dp_req.cont ? s->dp.wp : 0u;
        dp_start(&s->dp, s->dp_req.sub, s->gps.last.lat_deg, s->gps.last.lon_deg, s->ap.heading, s->dp_req.kn,
                 s->cfg.dp_max_kn, sb_program(s), from_wp);
        break;
    }
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

/* why DP ended, from the mode change reason */
static uint8_t dp_stop_why(mode_reason_t why, sb_mode_t to)
{
    if (to == MODE_MANUAL) return DP_STOP_JOYSTICK;
    switch (why) {
    case WHY_DP_STOP:       return DP_STOP_CMD;
    case WHY_GPS_LOST:      return DP_STOP_GPS;
    case WHY_COMPASS_FAULT: return DP_STOP_COMPASS;
    case WHY_DP_THRUSTER:   return DP_STOP_THRUSTER;
    case WHY_DP_OFFSPOT:    return DP_STOP_OFFSPOT;
    case WHY_STANDBY_BUTTON: case WHY_ESTOP: case WHY_LINK_LOST: case WHY_PC: case WHY_USER: case WHY_TEST:
        return DP_STOP_STANDBY;
    default:                return DP_STOP_FAULT;
    }
}

/* an emergency in this step (step 11g): idle AT ONCE whatever stop is running - also when the boat is already
 * in STANDBY during a planned arrival stop or a normal generator stop (they keep rate_down only without one) */
static void emergency(sb_t *s)
{
    s->emerg = true;
    s->stop_planned = false;
    gen_emergency(&s->gen);
}

bool sb_request_mode(sb_t *s, sb_mode_t to, mode_reason_t why)
{
    sb_mode_t from = s->modes.mode;
    mode_verdict_t v = mode_request(&s->modes, to, why, precondition(s, to, why), s->now_ms);
    if (v != MODE_OK) return false;
    if (from == MODE_DP && to != MODE_DP) dp_stop(&s->dp, dp_stop_why(why, to));   /* thrusters 0 in this step */
    if (to == MODE_STANDBY) s->stop_planned = (why == WHY_ARRIVED);   /* arrival: the normal closing rate */
    on_enter(s, from, to);
    return true;
}

/* --- inputs ---------------------------------------------------------------- */

/* Joystick, from its source (always read: it must be able to take over
 * from any mode). Analog: wires + ADC; USB: bridge frames + map. */
static void joystick_read(sb_t *s, uint32_t dt_ms)
{
    /* UNCREWED: the onboard joystick is ignored completely - control sees
     * "no joystick"; the probe still watches it (a joystick appearing) */
    bool use = s->profile != SB_PROFILE_UNCREWED;
    joy_input_t none;
    memset(&none, 0, sizeof none);
    none.ready = true;
    if (s->jmap.source == JOY_SRC_BRIDGE) {
        joy_input_t u;
        jb_step(&s->jb, &s->jmap, s->now_ms, &u);
        joy_update(&s->joy_probe, &u, dt_ms);
        s->probe_ready = u.ready;
        joy_update(&s->joy, use ? &u : &none, dt_ms);
    } else {
        hal_joystick_t js;
        bool ok = hal_read_joystick(&js);
        joy_step(&s->joy_probe, ok ? &js : NULL, dt_ms);
        s->probe_ready = true;
        if (use) joy_step(&s->joy, ok ? &js : NULL, dt_ms);
        else joy_update(&s->joy, &none, dt_ms);
    }
}

const char *sb_profile_name(uint8_t profile)
{
    return profile == SB_PROFILE_CREWED ? "CREWED" : profile == SB_PROFILE_UNCREWED ? "UNCREWED" : "DETECTING";
}

static void profile_set(sb_t *s, uint8_t p)
{
    bool change = s->profile != SB_PROFILE_DETECTING;
    s->profile = p;
    s->profile_changes++;
    if (change || p == SB_PROFILE_UNCREWED) {
        /* control starts from a clean joystick state (disarmed, no latched fault) */
        joy_config_t jc = s->joy.cfg;
        joy_cal_t cal = s->joy.cal;
        joy_init(&s->joy, &jc, &cal);
    }
    if (change) {
        alarm_on(s, SB_ALM_PROFILE);
        s->confirm.kind = CONF_NONE;
        plim_drop(&s->plim, PLIM_SRC_FAULT, s->now_ms);
        /* the neutral-start interlock again, with the new profile's rule (CREWED:
         * the joystick seen centred) - a held stick can't arrive armed */
        s->armed = false;
        s->arm_ok_ms = 0u;
        s->unarmed_ms = 0u;
        hal_set_safety_relay(false);
    }
}

/* CREWED / UNCREWED (CLAUDE.md section A): decided at power-up, changed only
 * in STANDBY with the boat stopped (UNCREWED -> CREWED); a joystick appearing
 * while driving UNCREWED is a fault (STANDBY, alarm), never a profile change. */
static void profile_step(sb_t *s, uint32_t dt_ms)
{
    const joy_t *p = &s->joy_probe;
    bool ok = p->present && s->probe_ready && !p->fault;
    s->probe_ok_ms = ok ? s->probe_ok_ms + dt_ms : 0u;
    if (s->cfg.first_test) {
        s->profile = SB_PROFILE_CREWED;
        return;
    }
    if (s->profile == SB_PROFILE_DETECTING) {
        if (s->probe_ok_ms >= s->cfg.profile_ok_ms) profile_set(s, SB_PROFILE_CREWED);
        else if (s->now_ms - s->boot_ms >= s->cfg.profile_detect_ms) profile_set(s, SB_PROFILE_UNCREWED);
        return;
    }
    /* In STANDBY (stopped) only UNCREWED -> CREWED: a joystick plugged in on
     * board and passing its self-check. A CREWED boat whose joystick fails or
     * is unplugged keeps CREWED (that is a joystick fault, the crew is still
     * there): UNCREWED needs a power-up without the joystick. */
    bool calm = sb_mode(s) == MODE_STANDBY && !s->stopping;
    if (calm && s->profile == SB_PROFILE_UNCREWED && s->probe_ok_ms >= s->cfg.profile_switch_ms)
        profile_set(s, SB_PROFILE_CREWED);
    /* CREWED: the joystick disappears (not detected any more) = a joystick
     * fault: the modes it drives end in STANDBY (idle + neutral first) */
    if (s->profile == SB_PROFILE_CREWED && s->joy_was_present && !s->joy.present) {
        alarm_on(s, SB_ALM_JOY_FAULT);
        plim_drop(&s->plim, PLIM_SRC_JOYSTICK, s->now_ms);
        if (sb_mode(s) == MODE_MANUAL || sb_mode(s) == MODE_HDG_LOCK || remote_like(sb_mode(s)) || sb_mode(s) == MODE_DP)
            sb_request_mode(s, MODE_STANDBY, WHY_JOY_FAULT);
    }
    s->joy_was_present = s->profile == SB_PROFILE_CREWED && s->joy.present;
    if (s->profile == SB_PROFILE_UNCREWED && sb_mode(s) != MODE_STANDBY && s->probe_ok_ms >= s->cfg.profile_ok_ms &&
        !s->profile_fault) {
        s->profile_fault = true;
        alarm_on(s, SB_ALM_PROFILE);
        sb_request_mode(s, MODE_STANDBY, WHY_PROFILE);          /* idle + neutral first */
    }
    if (!ok) s->profile_fault = false;
}

void sb_shore_joy(sb_t *s, uint8_t station, uint16_t age_ms, float x, float y, float lever, bool has_lever,
                  bool confirm)
{
    sb_shore_t *h = &s->shore;
    if (station < 2u || station != s->helm) return;             /* only the HELM station's joystick */
    float tol = s->joy.cfg.center_tol;
    h->station = station;
    h->rx_ms = s->now_ms ? s->now_ms : 1u;
    h->age_ms = age_ms;
    h->centred = isfinite(x) && isfinite(y) && fabsf(x) < tol && fabsf(y) < tol &&
                 (!has_lever || (isfinite(lever) && joy_lever_neutral(&s->joy, lever)));
    if (confirm && !h->confirm) {
        h->confirm_edge = true;
        h->presses++;
    }
    h->confirm = confirm;
}

/* --- UNCREWED lost link (CLAUDE.md section B) ----------------------------------- */

void sb_gw_shore(sb_t *s, uint8_t flags, uint16_t dist_m)
{
    s->coast.rx_ms = s->now_ms ? s->now_ms : 1u;
    s->coast.flags = flags;
    s->coast.dist_m = dist_m;
    s->coast.count++;
}

bool sb_near_shore(const sb_t *s)
{
    const uint32_t age = s->now_ms - s->coast.rx_ms;
    if (s->coast.rx_ms == 0u || age > s->cfg.shore_max_age_ms || s->gw.state != SB_GW_OK) return true;
    if (!(s->coast.flags & SB_SHORE_VALID) || (s->coast.flags & SB_SHORE_NEAR)) return true;
    return s->coast.dist_m == 0xFFFFu || (float)s->coast.dist_m <= s->cfg.near_shore_m;
}

float sb_lostlink_left_s(const sb_t *s)
{
    if (s->ll.state != SB_LL_LOST) return 0.0f;
    float left = s->cfg.lostlink_grace_s - (float)(s->now_ms - s->ll.lost_ms) * 0.001f;
    return left > 0.0f ? left : 0.0f;
}

/* The link to the HELM station: a gateway station holds HELM and the
 * gateway is there (the USB laptop on board counts while its link is up). */
static bool helm_link_ok(const sb_t *s)
{
    if (s->helm == SB_HELM_USB) return !(s->alarm_bits & SB_ALM_PC_LINK);
    return s->helm >= 2u && s->gw.state == SB_GW_OK;
}

/* Every step, UNCREWED only: link lost -> STANDBY (idle + neutral first);
 * then ENGINE KILL near shore (or without fresh shore info) as soon as idle +
 * neutral, at open sea after lostlink_grace_s unless the link came back. The
 * kill is latched until a POWER-ON reset (a watchdog / software reset keeps it,
 * hal_kill_keep_*) or the HELM station's remote restart
 * (sb_restart, section N); CREWED never kills. */
static void lostlink_step(sb_t *s)
{
    if (s->ll.state == SB_LL_KILLED) {
        hal_set_engine_kill(true);
        emergency(s);
        if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_LINK_LOST);
        return;
    }
    hal_set_engine_kill(false);
    if (s->profile != SB_PROFILE_UNCREWED) {
        s->ll.state = SB_LL_OK;                         /* CREWED: the current behaviour, never a kill */
        s->ll.armed = false;
        return;
    }
    bool ok = helm_link_ok(s);
    if (ok) s->ll.armed = true;
    if (s->ll.state == SB_LL_OK) {
        if (!s->ll.armed || ok) return;
        s->ll.state = SB_LL_LOST;
        s->ll.lost_ms = s->now_ms;
        s->ll.losses++;
        alarm_on(s, SB_ALM_LINK_LOST);
        s->confirm.kind = CONF_NONE;
        emergency(s);
        if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_LINK_LOST);
        return;
    }
    /* lost */
    if (ok) {
        s->ll.state = SB_LL_OK;                         /* back in time: stays in STANDBY, the alarm stays */
        s->ll.returns++;
        return;
    }
    emergency(s);
    if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_LINK_LOST);
    bool safe = s->mon_tripped || (thr_idle(s) && (gear_is(&s->gear, GEAR_N) || gear_fault(&s->gear)));
    bool near = sb_near_shore(s);
    bool late = (float)(s->now_ms - s->ll.lost_ms) >= s->cfg.lostlink_grace_s * 1000.0f;
    if (safe && (near || late)) {
        s->ll.state = SB_LL_KILLED;
        s->ll.near_at_kill = near;
        s->ll.kills++;
        alarm_on(s, SB_ALM_ENGINE_KILLED);
        hal_kill_keep_set(true);                        /* (kept through a watchdog / software reset) */
        hal_set_engine_kill(true);
    }
}

/* The engine-control conditions (section N) as they are this step. */
static void eng_cond(const sb_t *s, eng_cond_t *c, bool standby_button)
{
    memset(c, 0, sizeof *c);
    c->idle = s->thr.out_pct <= s->cfg.gear.idle_max_pct && thr_idle(s);      /* the modelled servo at idle */
    /* gear N confirmed: the neutral switch if fitted, else the gear servo at N; the gear state N always */
    c->neutral = gear_is(&s->gear, GEAR_N) && !gear_fault(&s->gear) &&
                 (s->neutral_sw >= 0 ? s->neutral_sw == 1 : s->gear.pwm_us == s->cfg.gear.pwm_n_us);
    c->gear_request = s->gear_demand != GEAR_N;
    c->estop_pressed = s->estop_state == 2u;
    c->first_test = s->cfg.first_test;
    c->killed = s->ll.state == SB_LL_KILLED;
    c->tripped = s->mon_tripped;
    c->helm_link = helm_link_ok(s);
    c->helm = s->helm;
    c->standby_button = standby_button;
    c->nv_busy = hal_nv_pending();
}

bool sb_restart_ready(const sb_t *s)
{
    eng_cond_t c;
    eng_cond(s, &c, false);
    return engine_check(&c) == ENG_WHY_NONE;
}

sb_result_t sb_restart(sb_t *s, uint8_t station, uint8_t action)
{
    eng_cond_t c;
    eng_why_t why;
    eng_cond(s, &c, false);
    if (action == ENG_ACT_RELEASE) {
        engine_release(&s->eng, &s->relays, station, s->now_ms);
        return SB_OK;
    }
    if (action == ENG_ACT_RESTART) {
        bool clear = false;
        why = engine_restart(&s->eng, &c, station, s->now_ms, &clear);
        if (clear) {
            /* (a) the ENGINE KILL latch: cleared; STANDBY stays, nothing starts by itself */
            s->ll.state = SB_LL_OK;
            s->ll.kept = false;
            s->ll.node_kill = false;
            s->alarm_bits &= ~SB_ALM_ENGINE_KILLED;
            if (s->alarm_bits == 0u && (s->alarm_bits2 & ~SB_ALM2_QUIET) == 0u && !s->ap.alarm) s->alarm = false;
            hal_kill_keep_set(false);
            hal_set_engine_kill(false);
        }
    } else if (action == ENG_ACT_CRANK) {
        why = engine_crank(&s->eng, &c, &s->relays, station, s->now_ms);
    } else {
        return SB_ERR_BAD_VALUE;
    }
    switch (why) {
    case ENG_WHY_NONE:       return SB_OK;
    case ENG_WHY_FIRST_TEST: return SB_ERR_WRONG_MODE;
    case ENG_WHY_NOT_HELM:   return SB_ERR_NOT_ALLOWED;
    case ENG_WHY_ESTOP: case ENG_WHY_COOLING: case ENG_WHY_RELAY: case ENG_WHY_LINK: return SB_ERR_REFUSED;
    default:                 return SB_ERR_PRECOND;
    }
}

/* the shore joystick counts: HELM station, packets fresh and not old */
static bool shore_ready(const sb_t *s)
{
    const sb_shore_t *h = &s->shore;
    return s->helm >= 2u && h->station == s->helm && h->rx_ms != 0u &&
           s->now_ms - h->rx_ms <= s->cfg.remote_max_age_ms && h->age_ms <= s->cfg.remote_max_age_ms;
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
    if (sb_mode(s) == MODE_REMOTE || sb_mode(s) == MODE_MANEUVER)
        in->angle_cmd_deg = remote_steer(s, (float)(s->now_ms - s->last_ms) * 0.001f);
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
    if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;              /* nobody could press CONFIRM */
    s->remote.station = station;
    s->confirm.kind = CONF_REMOTE;
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
    s->remote.requests++;
    return SB_OK;
}

sb_result_t sb_maneuver_request(sb_t *s, uint8_t station)
{
    if (s->cfg.first_test) return SB_ERR_WRONG_MODE;              /* FIRST TEST: steering only */
    if (sb_mode(s) == MODE_MANUAL) return SB_ERR_JOYSTICK;        /* the onboard joystick has it */
    if (sb_mode(s) != MODE_STANDBY && sb_mode(s) != MODE_REMOTE) return SB_ERR_WRONG_MODE;
    if (!s->armed || s->mon_tripped) return SB_ERR_PRECOND;
    if (s->joy.off_centre) return SB_ERR_NOT_CENTERED;            /* centre the joystick */
    if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;        /* nobody could press CONFIRM */
    s->remote.station = station;
    s->confirm.kind = CONF_MANEUVER;
    s->confirm.deadline_ms = s->now_ms + s->cfg.confirm_ms;
    s->remote.requests++;
    return SB_OK;
}

static bool remote_like(sb_mode_t m)
{
    return m == MODE_REMOTE || m == MODE_MANEUVER;
}

sb_result_t sb_remote_release(sb_t *s, uint8_t station)
{
    if ((s->confirm.kind == CONF_REMOTE || s->confirm.kind == CONF_MANEUVER) && s->remote.station == station)
        s->confirm.kind = CONF_NONE;
    if (!remote_like(sb_mode(s))) return SB_OK;
    if (station != s->remote.station) return SB_ERR_WRONG_MODE;
    return sb_request_mode(s, MODE_STANDBY, WHY_REMOTE_END) ? SB_OK : SB_ERR_PRECOND;
}

void sb_remote_input(sb_t *s, uint8_t station, uint16_t seq, uint16_t age_ms, uint16_t lat_ms,
                     float x, float y, float lever, bool has_lever)
{
    sb_remote_t *r = &s->remote;
    if (!remote_like(sb_mode(s)) || station != r->station) {
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
    if (sb_mode(s) == MODE_MANEUVER) {
        /* MANEUVER: the throttle is ALWAYS idle; the captain's joystick only
         * chooses the gear (forward / neutral zone / reverse) - gear changes go
         * through the gear state machine (idle -> neutral -> gear) */
        s->throttle_demand = 0.0f;
        if (!r->have) {
            s->gear_demand = s->gear.current;               /* no data yet: no gear change for nothing */
        } else if (r->has_lever) {
            s->gear_demand = (r->lever > s->joy.cfg.nz_hi) ? GEAR_F : (r->lever < s->joy.cfg.nz_lo) ? GEAR_R : GEAR_N;
        } else {
            float db = s->joy.cfg.deadband;
            s->gear_demand = (r->y > db) ? GEAR_F : (r->y < -db) ? GEAR_R : GEAR_N;
        }
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
    s->joy_probe.cal = cal;
    s->cfg_version++;
    return nv_save(HAL_NV_SLOT_JOYSTICK, &cal, (uint32_t)sizeof cal);
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
    return !save || nv_save(HAL_NV_SLOT_JOYMAP, &n, (uint32_t)sizeof n);
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
        bool safe = thr_idle(s) && (gear_is(&s->gear, GEAR_N) || gear_fault(&s->gear));
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
    if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;
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
    if (sb_mode(s) == MODE_AUTO || (sb_mode(s) == MODE_DP && s->dp.sub == DP_TRIP))
        return SB_ERR_WRONG_MODE;                               /* not while it runs (AUTO / a DP TRIP) */
    *err = program_validate(p, &s->cfg.prog, bad_index);
    if (*err != PROG_OK) return SB_ERR_BAD_PROGRAM;
    s->autop.prog = *p;
    s->autop.prog_ok = true;
    s->autop.wp = 0u;
    s->autop.suspended = false;
    s->autop.finished = false;
    s->autop.legs_done = 0u;
    s->autop.via_gw = false;                                /* (link.c sets it for a gateway upload) */
    s->autop.returning = false;                             /* a new program ends an AUTO RETURN (its way back) */
    s->dp.wp = 0u;                                          /* a DP TRIP "continue" starts the new one at waypoint 1 */
    s->cfg_version++;
    if (s->confirm.kind == CONF_AUTO_START || s->confirm.kind == CONF_AUTO_RESUME ||
        s->confirm.kind == CONF_AUTO_RETURN ||
        (s->confirm.kind == CONF_DP && s->dp_req.sub == DP_TRIP)) s->confirm.kind = CONF_NONE;
    (void)nv_save(HAL_NV_SLOT_PROGRAM, p, (uint32_t)sizeof *p);   /* written in STANDBY */
    return SB_OK;
}

sb_result_t sb_auto_start(sb_t *s)
{
    if (sb_mode(s) == MODE_AUTO || sb_mode(s) == MODE_TRAINING) return SB_ERR_WRONG_MODE;
    sb_result_t r = auto_common_checks(s);
    if (r != SB_OK) return r;
    if (!wp_reachable(s, &sb_program(s)->wp[0])) return SB_ERR_PRECOND;     /* the stored program's waypoint 1 */
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

/* AUTO RETURN: the way back into rp from the program p with the active waypoint k (finished: k = the last
 * one, where the boat is): waypoints k-1, k-2 ... 1 (index 0); on the first leg (k = 0) just waypoint 1.
 * The way back to waypoint i drives back over the leg i -> i+1: that leg's speed; radius + action of
 * waypoint i; at waypoint 1 stop in neutral (PROG_END_STOP). The same validation as an upload (the
 * limits may have changed since: max_leg_kn). */
static program_t s_ret_scratch;     /* (core 0 only; no 1 KB program on the stack) */

static bool return_build(const sb_t *s, const program_t *p, uint8_t k, program_t *rp, uint8_t *base)
{
    uint8_t bad;
    if (p->n == 0u) return false;
    if (k >= p->n) k = (uint8_t)(p->n - 1u);
    program_clear(rp);
    rp->final_action = (uint8_t)PROG_END_STOP;
    if (k == 0u) {
        rp->n = 1u;
        rp->wp[0] = p->wp[0];
        *base = 0u;
    } else {
        rp->n = k;
        for (uint8_t j = 0u; j < k; j++) {
            uint8_t i = (uint8_t)(k - 1u - j);
            rp->wp[j] = p->wp[i];
            rp->wp[j].speed_kn = p->wp[i + 1u].speed_kn;
        }
        *base = (uint8_t)(k - 1u);
    }
    program_seal(rp);
    return program_validate(rp, &s->cfg.prog, &bad) == PROG_OK;
}

sb_result_t sb_auto_return(sb_t *s)
{
    const sb_auto_t *a = &s->autop;
    sb_mode_t m = sb_mode(s);
    if (m == MODE_TRAINING || m == MODE_BENCH || m == MODE_DP) return SB_ERR_WRONG_MODE;
    if (a->returning) return SB_ERR_WRONG_MODE;                      /* already on the way back */
    sb_result_t r = auto_common_checks(s);
    if (r != SB_OK) return r;
    uint8_t base;
    if (!return_build(s, &a->prog, a->wp, &s_ret_scratch, &base)) return SB_ERR_BAD_PROGRAM;
    if (!wp_reachable(s, &s_ret_scratch.wp[0])) return SB_ERR_PRECOND;
    confirm_arm(s, CONF_AUTO_RETURN);
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
    {
        if (sb_mode(s) == MODE_AUTO) return false;
        /* step 13b: the route / waypoint change only sticks when the mode change succeeds (as AUTO RETURN) */
        bool was_ret = a->returning, done = a->finished;
        uint8_t wp = a->wp;
        uint32_t legs = a->legs_done;
        if (was_ret) {                      /* a stopped / finished AUTO RETURN: start the stored program */
            s_ret_scratch = a->prog;        /* (the way back, put back if the start is refused) */
            a->prog = a->orig;
            a->returning = false;
        }
        a->wp = 0u;
        a->finished = false;
        a->legs_done = 0u;
        bool ok = sb_request_mode(s, MODE_AUTO, WHY_CONFIRM);
        if (!ok) {                          /* refused now: everything as it was */
            if (was_ret) {
                a->orig = a->prog;
                a->prog = s_ret_scratch;
                a->returning = true;
            }
            a->wp = wp;
            a->finished = done;
            a->legs_done = legs;
        }
        return ok;
    }
    case CONF_AUTO_RESUME:
        if (sb_mode(s) == MODE_AUTO) {
            if (a->state == AUTO_RUN || !auto_leg_ok(s) || !sb_gps_ok(s)) return false;
            cruise_start(s);
            auto_begin_leg(s);
            return true;
        }
        return sb_request_mode(s, MODE_AUTO, WHY_CONFIRM);
    case CONF_AUTO_RETURN: {
        /* the way back replaces the active program in RAM (the stored one kept in orig, never saved) */
        uint8_t base, wp = a->wp;
        bool done = a->finished;
        uint32_t legs = a->legs_done;
        if (a->returning || !return_build(s, &a->prog, a->wp, &s_ret_scratch, &base)) return false;
        bool in_auto = sb_mode(s) == MODE_AUTO;
        a->orig = a->prog;
        a->prog = s_ret_scratch;
        a->returning = true;
        a->ret_base = base;
        a->wp = 0u;
        a->finished = false;
        a->legs_done = 0u;
        bool ok;
        if (in_auto) {
            ok = auto_leg_ok(s) && sb_gps_ok(s);
            if (ok) {
                /* step 13b: the speed loop restarts bumpless, but the rough-water slow-down and its
                 * memory stay (same sea: the return never jumps back to the full leg speed) */
                sb_nerv_t keep = s->nerv;
                float keep_target = s->cruise.target_kn;
                a->suspended = false;
                cruise_start(s);
                s->nerv = keep;
                if (keep.nervous || keep.ceiling_ms > 0u) s->cruise.target_kn = fminf(keep_target, s->cruise.set_kn);
                auto_begin_leg(s);
            }
        } else {
            ok = sb_request_mode(s, MODE_AUTO, WHY_CONFIRM);
        }
        if (!ok) {                          /* refused now: everything as it was */
            a->prog = a->orig;
            a->returning = false;
            a->wp = wp;
            a->finished = done;
            a->legs_done = legs;
        }
        return ok;
    }
    case CONF_REMOTE:
        /* the HELM station must still be the one that asked */
        if (s->helm != s->remote.station) return false;
        return sb_request_mode(s, MODE_REMOTE, WHY_REMOTE);
    case CONF_MANEUVER:
        if (s->helm != s->remote.station) return false;
        return sb_request_mode(s, MODE_MANEUVER, WHY_MANEUVER);
    case CONF_RAISE_FWD:
    case CONF_RAISE_REV:
        /* one step up; never changes the mode */
        return plim_step_up(&s->plim, kind == CONF_RAISE_REV ? 1u : 0u, s->now_ms) == PLIM_OK;
    case CONF_RELAY_TABLE:
        /* the offered relay table becomes the active one (saved in STANDBY, below); never a mode change */
        return relays_accept(&s->relays, sb_mode(s) == MODE_STANDBY && !s->stopping);
    case CONF_THR_TRAVEL:
        /* the owner-signed shorter throttle travel time (OD-4): STANDBY only, never a mode change, logged */
        if (sb_mode(s) != MODE_STANDBY || s->stopping) return false;
        if (!(s->travel.pending_s >= SERVO_THR_TRAVEL_MIN)) return false;
        s->servo.cfg.travel_s = s->travel.pending_s;
        s->travel.changes++;
        s->steer.log_why = 0u;
        s->steer.log_what = 4u;
        s->steer.log_seq++;
        return true;
    case CONF_CHECK:
        /* CONFIRM_ONLY: the answer is recorded (TELEM2 group R) - nothing else changes */
        s->conf_only.state = SB_CO_CONFIRMED;
        s->conf_only.confirmed++;
        return true;
    case CONF_DP:
        /* the HELM station must still be the one that asked; the conditions again, now */
        if (s->helm != s->dp_req.station) return false;
        if (sb_mode(s) == MODE_DP) {
            if (!dp_ok(s, s->dp_req.sub, s->dp_req.cont, false)) return false;
            dp_start(&s->dp, s->dp_req.sub, s->gps.last.lat_deg, s->gps.last.lon_deg, s->dp.hdg_est, s->dp_req.kn,
                     s->cfg.dp_max_kn, sb_program(s), s->dp_req.cont ? s->dp.wp : 0u);
            return true;
        }
        return sb_request_mode(s, MODE_DP, WHY_DP);
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
    if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;
    if (s->alarm || s->ap.alarm) return SB_ERR_ALARM;
    if (dir == 0u ? s->plim.fwd_step + 1u >= PLIM_FWD_STEPS
                  : s->plim.rev_pct >= s->plim.cfg.rev_max - 0.01f) return SB_ERR_AT_MAX;
    *code = plim_code_request(&s->plim, dir ? 1u : 0u, s->now_ms);
    return SB_OK;
}

sb_result_t sb_power_raise(sb_t *s, uint8_t dir, uint16_t code)
{
    dir = dir ? 1u : 0u;
    if (!sb_confirm_ready(s)) {
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

sb_result_t sb_dp_cmd(sb_t *s, uint8_t station, uint8_t action, uint8_t arg, int16_t value)
{
    bool in_dp = sb_mode(s) == MODE_DP;
    if (station == SB_HELM_NONE || station != s->helm) return SB_ERR_NOT_ALLOWED;
    if (s->cfg.first_test) return SB_ERR_WRONG_MODE;              /* FIRST TEST: no DP */
    switch (action) {
    case DP_ACT_STOP:
        if (s->confirm.kind == CONF_DP) s->confirm.kind = CONF_NONE;
        if (!in_dp) return SB_OK;
        return sb_request_mode(s, MODE_STANDBY, WHY_DP_STOP) ? SB_OK : SB_ERR_PRECOND;
    case DP_ACT_START: {
        uint8_t sub = (uint8_t)(arg & 0x7Fu);
        bool cont = (arg & DP_TRIP_CONTINUE) != 0u;
        float kn = (float)value * 0.01f;
        if ((sub != DP_SPOT && sub != DP_CRUISE && sub != DP_TRIP) || (cont && sub != DP_TRIP) || value < 0 ||
            kn > s->cfg.dp_max_kn + 0.001f) return SB_ERR_BAD_VALUE;
        if (sub == DP_TRIP && !(kn > 0.0f)) kn = s->cfg.dp_max_kn;  /* TRIP 0 = as fast as DP may (and the legs say) */
        if (in_dp && sub == DP_SPOT) {
            /* a hold: SPOT here at once (no CONFIRM - it only stops the boat where it is) */
            dp_start(&s->dp, DP_SPOT, s->gps.last.lat_deg, s->gps.last.lon_deg, s->dp.hdg_est, 0.0f, s->cfg.dp_max_kn,
                     NULL, 0u);
            return SB_OK;
        }
        if (!in_dp) {
            if (sb_mode(s) == MODE_MANUAL) return SB_ERR_JOYSTICK;  /* the onboard joystick has it */
            if (sb_mode(s) != MODE_STANDBY || s->stopping) return SB_ERR_STANDBY_ONLY;
        }
        if (!s->armed || s->mon_tripped) return SB_ERR_PRECOND;
        if (s->joy.off_centre) return SB_ERR_NOT_CENTERED;        /* centre the joystick */
        if (!sb_confirm_ready(s)) return SB_ERR_JOY_NOT_READY;    /* nobody could press CONFIRM */
        if (sub == DP_TRIP && !s->autop.prog_ok) return SB_ERR_NO_PROGRAM;
        if (!dp_ok(s, sub, cont, !in_dp)) return SB_ERR_PRECOND;
        s->dp_req.sub = sub;
        s->dp_req.cont = cont;
        s->dp_req.kn = kn;
        s->dp_req.station = station;
        s->dp_req.requests++;
        confirm_arm(s, CONF_DP);
        return SB_OK;
    }
    case DP_ACT_JOG:
        if (!in_dp) return SB_ERR_WRONG_MODE;
        if (arg > DP_JOG_STBD) return SB_ERR_BAD_VALUE;
        return dp_jog(&s->dp, arg, s->thrust.cfg.jog_m) ? SB_OK : SB_ERR_WRONG_MODE;
    case DP_ACT_SPEED:
        if (!in_dp) return SB_ERR_WRONG_MODE;
        if (value < 0 || (float)value * 0.01f > s->cfg.dp_max_kn + 0.001f) return SB_ERR_BAD_VALUE;
        return dp_speed(&s->dp, (float)value * 0.01f, s->cfg.dp_max_kn) ? SB_OK : SB_ERR_WRONG_MODE;
    case DP_ACT_HEADING:
        if (!in_dp) return SB_ERR_WRONG_MODE;
        if (arg > 1u || (arg == 0u && (value < 0 || value > 3599)) || (arg == 1u && (value < -1800 || value > 1800)))
            return SB_ERR_BAD_VALUE;
        return dp_heading(&s->dp, (float)value * 0.1f, arg == 1u) ? SB_OK : SB_ERR_WRONG_MODE;
    default:
        return SB_ERR_BAD_VALUE;
    }
}

static void confirm_step(sb_t *s)
{
    sb_confirm_t *c = &s->confirm;
    if (c->kind != CONF_NONE && (int32_t)(s->now_ms - c->deadline_ms) >= 0) {
        if (is_raise(c->kind)) plim_log_attempt(&s->plim, c->kind == CONF_RAISE_REV, PLIM_ERR_LATE, s->now_ms);
        if (c->kind == CONF_CHECK) s->conf_only.state = SB_CO_LATE;
        c->kind = CONF_NONE;            /* too late */
    }
    /* a pending CONFIRM_ONLY taken away by STANDBY / another request / a fault */
    if (s->conf_only.state == SB_CO_PENDING && c->kind != CONF_CHECK) s->conf_only.state = SB_CO_CANCELLED;
    /* a relay table waits only in STANDBY (not stopping); any other state drops the offer */
    if (c->kind == CONF_RELAY_TABLE && (sb_mode(s) != MODE_STANDBY || s->stopping)) c->kind = CONF_NONE;
    if (c->kind == CONF_THR_TRAVEL && (sb_mode(s) != MODE_STANDBY || s->stopping)) c->kind = CONF_NONE;
    if (c->kind != CONF_RELAY_TABLE && s->relays.offer_pending) relays_cancel_offer(&s->relays);
    /* CREWED: the onboard joystick's CONFIRM; UNCREWED: the HELM station's
     * (shore captain's) joystick, same rules (stick centred, ready, in time) */
    bool shore = s->profile == SB_PROFILE_UNCREWED;
    bool edge = shore ? s->shore.confirm_edge : s->joy.confirm_edge;
    s->shore.confirm_edge = false;
    if (!edge) return;
    if (c->kind == CONF_NONE) {
        c->ignored++;                   /* nothing pending: does nothing */
        return;
    }
    if (shore ? !(shore_ready(s) && s->shore.centred) : !joy_confirm(&s->joy)) {
        c->last_kind = c->kind;
        c->refused++;                   /* stick not centred / not armed / faulted */
        if (is_raise(c->kind)) plim_log_attempt(&s->plim, c->kind == CONF_RAISE_REV, PLIM_ERR_NOT_CENTRED, s->now_ms);
        s->chirp_ms = 300u;
        return;
    }
    confirm_kind_t kind = c->kind;
    c->kind = CONF_NONE;
    c->last_kind = kind;
    if (confirm_execute(s, kind)) {
        c->accepted++;
    } else {
        c->refused++;
        s->chirp_ms = 300u;
    }
}

/* --- DP (section P) ----------------------------------------------------------------- */

/* Every step in MODE_DP: the outboard stays idle + neutral (the step's default demand), its steering
 * off; dp.c computes the thrust. GPS lost / suspect -> STANDBY (no dead reckoning in DP); the compass
 * lost -> the gyro holds the heading, STANDBY after DP_COMPASS_MS; too far / too long off the spot ->
 * STANDBY; off the spot / both thrusters saturated = advisories. The thrusters go to 0 in the same
 * step as the mode leaves DP (the thruster step below sees dp_active false). */
static void dp_mode(sb_t *s, float dt, uint32_t buttons)
{
    dp_t *d = &s->dp;
    if (!sb_gps_ok(s)) {
        alarm_on(s, (s->dr.why == DR_WHY_LOST || s->dr.state == DR_NO_FIX || s->dr.state == DR_GPS)
                    ? SB_ALM_GPS_LOST : SB_ALM_GPS_SUSPECT);
        sb_request_mode(s, MODE_STANDBY, WHY_GPS_LOST);
        return;
    }
    if (d->sub != DP_TRIP) {
        /* the course buttons (CREWED; HDG LOCK's buttons) move the held heading */
        float dlt = 0.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_MINUS_1))  dlt -= 1.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_PLUS_1))   dlt += 1.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_MINUS_10)) dlt -= 10.0f;
        if (buttons & HAL_BTN_BIT(HAL_BTN_PLUS_10))  dlt += 10.0f;
        if (dlt != 0.0f) (void)dp_heading(d, dlt, true);
    }
    dp_in_t in;
    memset(&in, 0, sizeof in);
    in.dt = dt;
    in.fix_new = s->gps_new_msg && !s->gps_bad_msg && s->gps.last.fix;
    in.lat = s->gps.last.lat_deg;
    in.lon = s->gps.last.lon_deg;
    in.sog_kn = s->gps.last.sog_kn;
    in.cog_deg = s->gps.last.cog_deg;
    in.compass_ok = sb_compass_ok(s);
    in.heading = s->ap.heading;
    in.gyro_ok = autopilot_gyro_ok(&s->ap);
    in.yaw_dps = s->ap.gyro_dps;
    in.cap_bow = thruster_cap(&s->thrust, THR_BOW);
    in.cap_stern = thruster_cap(&s->thrust, THR_STERN);
    in.out_bow = s->thrust.out_pct[THR_BOW];
    in.out_stern = s->thrust.out_pct[THR_STERN];
    in.max_kn = s->cfg.dp_max_kn;
    in.spot_alarm_m = s->thrust.cfg.spot_alarm_m;
    in.nav_cap_kn = nav_cap(s);
    in.nav_heading_on = d->sub == DP_TRIP && nav_heading(s, d->leg.track_deg, &in.nav_heading);
    uint8_t r = dp_step(d, &in);                         /* (TRIP: dp_start's own copy of sb_program(s), step 13b) */
    if (d->offspot) alarm2_on(s, SB_ALM2_DP_OFFSPOT);
    if (d->overpower) alarm2_on(s, SB_ALM2_DP_OVERPOWER);
    if (r == DP_R_COMPASS) {
        sb_request_mode(s, MODE_STANDBY, WHY_COMPASS_FAULT);    /* gyro hold for DP_COMPASS_MS, or no gyro */
    } else if (r == DP_R_OFFSPOT) {
        sb_request_mode(s, MODE_STANDBY, WHY_DP_OFFSPOT);
    } else if (r == DP_R_TRIP_END) {
        s->chirp_ms = 600u;                                     /* SPOT at the end of the route */
        if (d->trip_done) alarm_on(s, SB_ALM_ARRIVED);
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
    if (ok && s->ap.servo) {                     /* the position servo: the TRAINING record (v2) stored */
        s->steer.needs_training = false;
        s->steer.v1_refused = false;
        s->steer.log_what = 2u;
        s->steer.log_why = SM_RESET_TRAINING;
        s->steer.log_seq++;
    }
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
    if (s->profile == SB_PROFILE_UNCREWED) stick = true;              /* the onboard joystick is ignored */
    if (s->profile == SB_PROFILE_DETECTING) stick = false;            /* not decided yet: wait */
    if (s->cfg.first_test) stick = true;                              /* no joystick needed (no MANUAL) */
    bool idle = thr_idle(s);
    /* DroneCAN: idle on the wire is -1.0 = 1000 us (the boot frame, CPU1's safe frames); a throttle calibration
     * with another idle pulse never arms (step 11g: NOT ARMED, the throttle stays at the wire idle) */
    if (s->servo.cfg_bad) idle = false;
    /* DroneCAN: neutral counts only once the gear frame really went out (the boot order holds it until
     * the modelled throttle is idle with the throttle node OPERATIONAL) */
    if (s->servo.can && (!s->servo.gear_sent || s->servo.gear_hold)) neutral = false;
    s->arm_ok_ms = (neutral && stick && idle) ? s->arm_ok_ms + dt_ms : 0u;
    /* (the detection window is no fault; nor the DroneCAN boot order: no gear frame before the modelled
     * throttle is idle with the throttle node OPERATIONAL - a node that never comes is CAN NODE LOST) */
    if (s->profile != SB_PROFILE_DETECTING && !(s->servo.can && !s->servo.gear_sent)) s->unarmed_ms += dt_ms;
    if (s->arm_ok_ms >= s->cfg.arm_ms && !s->mon_tripped) {
        s->armed = true;
        hal_set_safety_relay(true);
        return;
    }
    if (s->unarmed_ms >= s->cfg.arm_alarm_ms && !(s->alarm_bits & SB_ALM_NOT_ARMED)) alarm_on(s, SB_ALM_NOT_ARMED);
}

/* DroneCAN (CLAUDE.md T, docs/dronecan_design.md section 5; step 11d): the bus state + the node table
 * from the HAL. A REQUIRED node LOST, a bus fault or OUR node ID on the bus = an alarm (WARNING, lowers
 * the power limit, cancels a pending CONFIRM) + STANDBY (idle + neutral first, like every fault); no mode
 * but STANDBY while it lasts (precondition). A required node not seen since the boot blocks too (no
 * alarm before HAL_CAN_NODE_LOST_MS). Health WARNING / a node without a static ID = notices only (OD-6).
 * No CAN fitted (hal_can_bus false) = nothing. Step 11d: nothing is commanded over CAN yet. */
static void can_event(sb_t *s, uint8_t what, uint8_t node, uint8_t why)
{
    s->can.log_what = what;
    s->can.log_node = node;
    s->can.log_why = why;
    s->can.log_seq++;
}

static void can_poll(sb_t *s)
{
    s->can.fitted = hal_can_bus(&s->can.bus);
    if (!s->can.fitted) {
        s->can.node_block = s->can.node_lost = s->can.bus_bad = false;
        return;
    }
    uint8_t lost = 0u, warn = 0u, seen = 0u, reb = 0u;
    bool block = false, req_lost = false;
    for (uint8_t i = 0; i < HAL_CAN_NODES; i++) {
        hal_can_node_t *n = &s->can.node[i];
        if (!hal_can_node(i, n)) continue;
        uint8_t b = (uint8_t)(1u << i);
        if (n->lost) lost |= b;
        if (n->warning) warn |= b;
        if (n->seen) seen |= b;
        if (n->rebooted) reb |= b;
        if (n->required && n->lost) req_lost = true;
        if (n->required && (n->lost || !n->seen)) block = true;
        if ((n->lost ? 1u : 0u) != ((s->can.lost_mask & b) ? 1u : 0u))
            can_event(s, n->lost ? 1u : 2u, n->id, n->why);
        else if (n->warning && !(s->can.warn_mask & b))
            can_event(s, 3u, n->id, n->why);
        if (n->rebooted && !(s->can.reboot_mask & b)) can_event(s, 4u, n->id, n->why);
    }
    const hal_can_bus_t *bus = &s->can.bus;
    bool bad = bus->fault || bus->id_conflict;
    if (bus->fault && !s->can.fault_prev) can_event(s, 5u, 0u, bus->state);
    if (!bus->fault && s->can.fault_prev) can_event(s, 6u, 0u, bus->state);
    if (bus->id_conflict && !s->can.conflict_prev) can_event(s, 7u, 10u, 0u);     /* our own node ID 10 */
    if (bus->anon_node && !s->can.anon_prev) can_event(s, 8u, 0u, 0u);
    s->can.fault_prev = bus->fault;
    s->can.conflict_prev = bus->id_conflict;
    s->can.anon_prev = bus->anon_node;
    s->can.lost_mask = lost;
    s->can.warn_mask = warn;
    s->can.seen_mask = seen;
    s->can.reboot_mask = reb;
    s->can.node_block = block;
    if (req_lost && !s->can.node_lost) s->can.losses++;
    s->can.node_lost = req_lost;
    s->can.bus_bad = bad;
}

static void can_step(sb_t *s)
{
    hal_can_service();
    can_poll(s);
    if (!s->can.fitted) return;
    bool req_lost = s->can.node_lost, bad = s->can.bus_bad;
    if (req_lost && !s->can.node_seen) {
        s->can.node_seen = true;
        alarm2_on(s, SB_ALM2_CAN_NODE);
        plim_drop(&s->plim, PLIM_SRC_FAULT, s->now_ms);
        s->confirm.kind = CONF_NONE;
    }
    if (!req_lost) s->can.node_seen = false;
    if (bad && !s->can.bus_seen) {
        s->can.bus_seen = true;
        alarm2_on(s, SB_ALM2_CAN_BUS);
        plim_drop(&s->plim, PLIM_SRC_FAULT, s->now_ms);
        s->confirm.kind = CONF_NONE;
    }
    if (!bad) s->can.bus_seen = false;
    if (bad || req_lost) emergency(s);
    if (sb_mode(s) != MODE_STANDBY) {
        if (bad) sb_request_mode(s, MODE_STANDBY, WHY_CAN_BUS);
        else if (req_lost) sb_request_mode(s, MODE_STANDBY, WHY_CAN_NODE);
    }
    /* owner decision OD-3 (an EXPLICIT exception to "ENGINE KILL only after idle + neutral"): UNCREWED and the
     * throttle node lost = the AURIX can no longer command idle and can't know where the throttle is (the
     * node holds) -> ENGINE KILL AT ONCE, latched like every kill; the gear is NOT shifted (servo.c holds the
     * gear frames while the throttle node is lost, OA-2). CREWED: alarm + STANDBY only, the crew stops the
     * engine if needed. Never in FIRST TEST (always CREWED). */
    if (s->servo.can && s->profile == SB_PROFILE_UNCREWED && !s->cfg.first_test && s->ll.state != SB_LL_KILLED &&
        s->can.node[HAL_CAN_ROLE_THROTTLE].lost) {
        s->ll.state = SB_LL_KILLED;
        s->ll.near_at_kill = false;
        s->ll.node_kill = true;
        s->ll.kills++;
        alarm_on(s, SB_ALM_ENGINE_KILLED);
        s->confirm.kind = CONF_NONE;
        emergency(s);
        hal_kill_keep_set(true);                        /* (kept through a watchdog / software reset) */
        hal_set_engine_kill(true);
        if (sb_mode(s) != MODE_STANDBY) sb_request_mode(s, MODE_STANDBY, WHY_CAN_NODE);
    }
}

static void events_step(sb_t *s, uint32_t buttons)
{
    can_step(s);

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
            emergency(s);
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
    if (g_mon.tripped) emergency(s);
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
        if (sb_mode(s) == MODE_AUTO || sb_mode(s) == MODE_TRAINING || remote_like(sb_mode(s))) {
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
        /* (DP, CREWED: the joystick is the crew's takeover - without it DP ends too) */
        if (sb_mode(s) == MODE_MANUAL || sb_mode(s) == MODE_HDG_LOCK || remote_like(sb_mode(s)) ||
            (sb_mode(s) == MODE_DP && s->profile == SB_PROFILE_CREWED))
            sb_request_mode(s, MODE_STANDBY, WHY_JOY_FAULT);
    }
    if (!s->joy.fault) s->joy_fault_seen = false;
    if (s->joy.not_centered && !s->joy_nc_seen) {
        s->joy_nc_seen = true;
        alarm_on(s, SB_ALM_JOY_NOT_CENTERED);
    }
    if (s->joy.armed) s->joy_nc_seen = false;

    /* REMOTE MANUAL needs its station to hold HELM (a station on board took command) */
    if (remote_like(sb_mode(s)) && s->helm != s->remote.station) sb_request_mode(s, MODE_STANDBY, WHY_REMOTE_END);

    /* override: it always wins (Y / the lever is the throttle in HDG_LOCK) */
    bool grab = s->joy.take_edge || s->joy.engage_x || (s->joy.engage_y && sb_mode(s) != MODE_HDG_LOCK) ||
                (s->joy.lever_moved && !lever_has_throttle(sb_mode(s)));
    if (s->joy.take_edge) plim_drop(&s->plim, PLIM_SRC_OVERRIDE, s->now_ms);      /* TAKE always lowers */
    if (grab && sb_mode(s) != MODE_MANUAL && !s->cfg.first_test) {     /* FIRST TEST: no MANUAL */
        if (s->confirm.kind == CONF_AUTO_START || s->confirm.kind == CONF_AUTO_RESUME ||
            s->confirm.kind == CONF_AUTO_RETURN ||
            s->confirm.kind == CONF_REMOTE || s->confirm.kind == CONF_MANEUVER || s->confirm.kind == CONF_DP)
            s->confirm.kind = CONF_NONE;
        sb_request_mode(s, MODE_MANUAL, WHY_JOYSTICK);
    }
    if (sb_mode(s) == MODE_MANUAL && s->joy.auto_edge) joystick_handback(s);

    confirm_step(s);

    /* STANDBY button: works in every mode, the PC can't override it */
    if (buttons & HAL_BTN_BIT(HAL_BTN_STANDBY)) {
        s->confirm.kind = CONF_NONE;
        emergency(s);                   /* idle at once, also in STANDBY (a planned / generator stop) */
        plim_drop(&s->plim, PLIM_SRC_STANDBY_BUTTON, s->now_ms);
        gen_set(&s->gen, false);        /* generator auto OFF until re-enabled (a running one stops idle first) */
        /* a held stick can't grab control right back - also when the press is the ACK in STANDBY (step 11e: a
         * DroneCAN node interlock may have held a deflected stick in STANDBY; the ACK must not hand it MANUAL) */
        if (!joy_centered(&s->joy)) joy_disarm(&s->joy);
        if (sb_mode(s) != MODE_STANDBY) {
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

/* DroneCAN position servo (step 11e): the slow adaptation (RAM only, OD-8) under its conditions -
 * steady straight HDG LOCK / AUTO ahead, speed within +-20 % of the TRAINING speed, no turn > 5 deg in 30 s,
 * no alarm (not even an advisory), compass + gyro healthy, GPS trusted, calm water - and the doubt resets
 * (sensor, node lost, residual, bounds) with the notice STEERING MODEL RESET + log event 30; the last
 * command into reset-surviving RAM (OD-7: a warm reset knows the position, a power-on does not). */
static void steer_model_step(sb_t *s, float dt)
{
    autopilot_t *ap = &s->ap;
    if (ap->servo_have) hal_steer_keep_set(true, ap->servo_u);
    /* heading once a second: the turn in the last 30 s */
    if (s->now_ms - s->steer.hms >= 1000u || s->steer.hn == 0u) {
        s->steer.hms = s->now_ms;
        s->steer.hring[s->steer.hi] = ap->heading;
        s->steer.hi = (uint8_t)((s->steer.hi + 1u) % 30u);
        if (s->steer.hn < 30u) s->steer.hn++;
    }
    float turn = 0.0f;
    for (uint8_t i = 0; i < s->steer.hn; i++) {
        float d = fabsf(autopilot_heading_error(ap->heading, s->steer.hring[i]));
        if (d > turn) turn = d;
    }
    sb_mode_t m = sb_mode(s);
    float v = sb_gps_ok(s) ? s->gps.last.sog_kn : 0.0f, vr = ap->sm.stored.v_ref;
    /* calm water: the yaw-rate wobble (the gyro minus its 10 s mean) over ~30 s - in waves the controller fighting
     * the sea would bias a closed-loop estimate, so the adaptation waits for calm water */
    if (ap->gyro_valid) {
        s->steer.yaw_lp += (ap->gyro_dps - s->steer.yaw_lp) * dt / 10.0f;
        float hp = ap->gyro_dps - s->steer.yaw_lp;
        s->steer.yaw_var += (hp * hp - s->steer.yaw_var) * dt / 30.0f;
    }
    bool calm = s->steer.yaw_var < SB_STEER_CALM_DPS * SB_STEER_CALM_DPS;
    bool steady = (m == MODE_HDG_LOCK || (m == MODE_AUTO && s->autop.state == AUTO_RUN)) && ap->mode == AP_MODE_AUTO &&
                  !s->hdg_reverse && !s->hdg_compass_lost && gear_is(&s->gear, GEAR_F) && s->steer.hn >= 30u &&
                  turn <= 5.0f && vr > 0.5f && v >= 0.8f * vr && v <= 1.2f * vr && s->alarm_bits == 0u &&
                  s->alarm_bits2 == 0u && !ap->alarm && !s->nerv.nervous && calm && sb_compass_ok(s) &&
                  autopilot_gyro_ok(ap) && sb_gps_ok(s);
    s->steer.steady = steady;
    sm_adapt_in_t ai;
    ai.dt = dt;
    ai.yaw_dps = ap->gyro_dps - ap->servo_calib.gyro_bias_dps;
    ai.v_kn = v;
    ai.steady = steady;
    ai.sensor_fault = ap->sensor_fault || (ap->gyro_valid && !autopilot_gyro_ok(ap));
    ai.node_lost = s->can.fitted && s->can.node[HAL_CAN_ROLE_STEERING].lost;
    uint8_t why = sm_adapt_step(&ap->sm, &ai);
    if (why != SM_RESET_NONE) {
        s->steer.reset_ms = s->now_ms ? s->now_ms : 1u;   /* notice STEERING MODEL RESET */
        s->steer.reset_why = why;
        s->steer.log_what = 1u;
        s->steer.log_why = why;
        s->steer.log_seq++;
    }
}

void sb_step(sb_t *s)
{
    ap_inputs_t in;
    float roll = 0.0f;

    s->last_ms = s->now_ms;
    s->now_ms = hal_millis();
    s->emerg = false;                   /* set again by every emergency of this step (emergency()) */
    uint32_t dt_ms = s->now_ms - s->last_ms;
    float dt = (float)dt_ms * 0.001f;

    joystick_read(s, dt_ms);
    profile_step(s, dt_ms);
    /* panel buttons + course buttons from the PC + the same functions on a USB joystick */
    uint32_t buttons = hal_get_button_presses() | s->ext_buttons | s->joy.buttons_edge;
    s->ext_buttons = 0u;
    /* UNCREWED: nobody on board - only the STANDBY button counts (it always does) */
    if (s->profile == SB_PROFILE_UNCREWED) buttons &= HAL_BTN_BIT(HAL_BTN_STANDBY);
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
    /* supply voltage of the 12 V bank: filtered here (battery.c), the raw
     * value comes from the HAL. LOW BATTERY is an ADVISORY: it is raised
     * while the condition holds (the acknowledge keeps it then), it never
     * changes the mode and never lowers the power limit (SB_ALM2_ADVISORY).
     * No data = a notice only (batt.no_data), never an alarm. */
    {
        float volts = 0.0f;
        bool measured = hal_read_supply_voltage(&volts);
        batt_step(&s->batt, measured, volts, dt_ms);
        if (s->batt.low) alarm2_on(s, SB_ALM2_LOW_BATTERY);
    }

    /* 1. steering core (also detects compass timeout) */
    s->ap.servo_v_kn = sb_gps_ok(s) ? s->gps.last.sog_kn : 0.0f;
    autopilot_update(&s->ap, &in);
    if (s->ap.servo) steer_model_step(s, dt);
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
    lostlink_step(s);

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
    else if (remote_like(sb_mode(s))) remote_mode(s);
    else if (sb_mode(s) == MODE_DP) dp_mode(s, dt, buttons);
    if (sb_mode(s) != MODE_DP) dp_stop(&s->dp, DP_STOP_FAULT);     /* (left DP above: already stopped) */
    if (sb_mode(s) == MODE_STANDBY || sb_mode(s) == MODE_DP) {      /* DP: the outboard idle + neutral throughout */
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

    /* generator mode (section K): the ONE automatic throttle rise - only in
     * STANDBY (not stopping) with gear N confirmed by the neutral switch, the
     * outputs armed, a known low bank voltage, no kill / e-stop / trip and no
     * alarm that is not an advisory, never in FIRST TEST. The demand is in
     * neutral and goes through the normal throttle path (rate limits, the
     * servo.c clamp with the generator cap); CPU1 gets gen_active + the cap.
     * Every stop is idle FIRST (generator.c); a neutral switch that opens or
     * a speed rise = GEAR FAULT (alarm, generator auto off). */
    {
        gen_input_t gi;
        sb_mode_t gm = sb_mode(s);
        bool kind_stop = s->confirm.kind == CONF_AUTO_START || s->confirm.kind == CONF_AUTO_RESUME ||
                         s->confirm.kind == CONF_AUTO_RETURN || s->confirm.kind == CONF_REMOTE || s->confirm.kind == CONF_MANEUVER ||
                         (s->confirm.kind == CONF_DP && s->dp_req.sub != DP_SPOT);      /* a DP trip / cruise start */
        /* DP: the generator may run while DP holds the boat (SPOT / JOG); CRUISE / TRIP move the boat */
        bool dp_hold = gm == MODE_DP && (s->dp.sub == DP_SPOT || s->dp.sub == DP_JOG);
        bool alarm_now = (s->alarm_bits & ~SB_NAV_ADVISORY) != 0u || (s->alarm_bits2 & ~SB_ALM2_ADVISORY) != 0u ||
                         s->ap.alarm;
        memset(&gi, 0, sizeof gi);
        gi.allowed = ((gm == MODE_STANDBY && !s->stopping) || dp_hold) && s->armed && !s->cfg.first_test &&
                     !s->mon_tripped && s->estop_state != 2u && s->ll.state == SB_LL_OK;
        gi.neutral_confirmed = neutral_sw == 1 && gear_is(&s->gear, GEAR_N) && !gear_fault(&s->gear);
        gi.volts_valid = s->batt.valid;
        gi.bank_v = s->batt.bank_v;
        gi.bank_v_fast = s->batt.bank_v_fast;
        gi.limit_pct = s->plim.fwd_pct;
        gi.stop_request = alarm_now || kind_stop || s->joy.take_edge ||
                          (s->profile == SB_PROFILE_CREWED && s->joy.off_centre);
        /* outside DP a SOG rise after a throttle rise = GEAR FAULT; in DP the boat moves anyway - there the
         * stern motor fighting an unexplained forward push is the check (dp.c fight) */
        gi.gps_trusted = sb_gps_ok(s) && gm != MODE_DP;
        gi.sog_kn = s->gps.last.sog_kn;
        gi.dp_fight = gm == MODE_DP && s->dp.fight;
        gi.thrust_pct = thruster_load_pct(&s->thrust);  /* DP thruster load (last step): pauses the judgement */
        gi.throttle_out_pct = thr_idle(s) ? 0.0f : fmaxf(s->thr.out_pct, s->cfg.gear.idle_max_pct + 1.0f);
        gi.idle_max_pct = s->cfg.gear.idle_max_pct;
        gi.dt_ms = dt_ms;
        gen_step(&s->gen, &gi);
        if (s->gen.start_edge && s->chirp_ms < 300u) s->chirp_ms = 300u;    /* beep: GENERATOR STARTING */
        if (s->gen.fault_edge) alarm_on(s, SB_ALM_GEAR);                     /* neutral opened / the boat moved */
        if (s->gen.fault_edge && gm == MODE_DP) sb_request_mode(s, MODE_STANDBY, WHY_GEAR_FAULT);   /* a fault: DP ends */
        if ((gm == MODE_STANDBY || gm == MODE_DP) && s->gen.demand_pct > 0.0f) {
            s->throttle_demand = s->gen.demand_pct;
            s->gear_demand = GEAR_N;
        }
    }

    /* 4. safe stop: idle + neutral first, then steering off */
    if (sb_mode(s) == MODE_STANDBY) {
        if (s->stopping) {
            /* a gear fault (neutral not confirmed) also ends the stop: the
             * engine is held at idle and the gear can't be commanded */
            bool safe = thr_idle(s) && (gear_is(&s->gear, GEAR_N) || gear_fault(&s->gear));

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
                /* centred AND idle + neutral (the slow modelled idle may come after the centring): only then
                 * steering off - until then the outboard is held at its centre */
                if (autopilot_centered(&s->ap) && safe) s->stop_steer = STOP_STEER_NONE;
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
        /* the second word: its advisories (LOW BATTERY) never touch the power limit either */
        uint32_t fresh2 = s->alarm_bits2 & ~s->alarm_bits2_seen & ~SB_ALM2_ADVISORY;
        bool any = s->alarm || s->ap.alarm;
        /* the one exception: slamming RED in STEP mode reduces in steps (slam.c),
         * the power limit stays - a sudden drop can be dangerous in a following sea */
        if (s->slam.cfg.red_reset < 0.5f) fresh &= ~SB_ALM_SLAM;
        /* navigation advisories never touch the power limit (their reaction is
         * defined explicitly: AIS silent = warning only) */
        fresh &= ~SB_NAV_ADVISORY;
        if ((s->alarm_bits & ~(SB_NAV_ADVISORY | (s->slam.cfg.red_reset < 0.5f ? SB_ALM_SLAM : 0u))) == 0u &&
            (s->alarm_bits2 & ~SB_ALM2_ADVISORY) == 0u && !s->ap.alarm) any = false;
        if (fresh & SB_ALM_PC_LINK) plim_drop(&s->plim, PLIM_SRC_PC_LINK, s->now_ms);
        else if (fresh || fresh2 || (any && !s->alarm_seen)) plim_drop(&s->plim, PLIM_SRC_ALARM, s->now_ms);
        if ((fresh || fresh2 || (any && !s->alarm_seen)) && is_raise(s->confirm.kind)) s->confirm.kind = CONF_NONE;
        s->alarm_bits_seen = s->alarm_bits;
        s->alarm_bits2_seen = s->alarm_bits2;
        s->alarm_seen = any;
        plim_step(&s->plim, speed_now(s), dt_ms, s->now_ms);
    }
    if (sb_mode(s) != MODE_AUTO) { s->speed_limited = false; s->sat_ms = 0u; }

    /* 4b. DP thrusters (section P; thruster.c is their only writer) - FIRST, before the
     *     outboard's gear and throttle: 0 unless MODE_DP runs (dp.c demands) AND
     *     allowed - not FIRST TEST, e-stop released (its contactor also cuts their
     *     power), no monitor trip, outputs armed; clamped to the boat file's max_pct,
     *     rate limited, heat budget per thruster (used up = that one 0 + DP THRUSTER
     *     HOT). Every exit from DP (STANDBY, a fault, the joystick) happened above
     *     in this step: the thrusters are 0 now. A thruster HOT (it went to 0
     *     itself, last step) or the thrusters blocked ends DP (it can't hold with
     *     one): STANDBY first, so both are 0 in this step. */
    {
        thr_input_t ti;
        memset(&ti, 0, sizeof ti);
        ti.allowed = !s->cfg.first_test && s->estop_state != 2u && !s->mon_tripped && s->armed &&
                     !hal_outputs_forced();
        if (sb_mode(s) == MODE_DP && (thruster_any_hot(&s->thrust) || !ti.allowed))
            sb_request_mode(s, MODE_STANDBY, WHY_DP_THRUSTER);
        ti.dp_active = sb_mode(s) == MODE_DP && s->dp.sub != DP_OFF;
        ti.demand_pct[THR_BOW] = ti.dp_active ? s->dp.bow : 0.0f;
        ti.demand_pct[THR_STERN] = ti.dp_active ? s->dp.stern : 0.0f;
#ifdef SB_TEST_HOOKS
        if (s->test_thrust.on && !ti.dp_active) {   /* (the step-9 thruster tests, without MODE_DP) */
            ti.dp_active = true;
            ti.demand_pct[THR_BOW] = s->test_thrust.bow_pct;
            ti.demand_pct[THR_STERN] = s->test_thrust.stern_pct;
        }
#endif
        bool dp_ran = sb_mode(s) == MODE_DP && s->dp.sub != DP_OFF;   /* (not the step-9 test hook) */
        ti.dt_ms = dt_ms;
        thruster_step(&s->thrust, &ti);
        if (thruster_any_hot(&s->thrust)) {
            alarm2_on(s, SB_ALM2_DP_HEAT);
            /* a thruster went HOT in this very step (thruster_step put only that one to 0): DP
             * ends NOW and the other one stops in the same step too (step 11b) */
            if (dp_ran) {
                sb_request_mode(s, MODE_STANDBY, WHY_DP_THRUSTER);
                thruster_stop(&s->thrust);
            }
        }
    }

    /* 5. gear and throttle (order: throttle limited by the gear state),
     *    demand limited by the power limit of its direction */
    s->throttle_mode = s->throttle_demand;
    /* the generator: throttle in NEUTRAL, at most its cap (section K) - only
     * for the generator's own demand (STANDBY, gear demand N); any other demand
     * (a takeover in the same step) meets the normal rule: idle in neutral, the
     * reverse limit out of forward, the gear only moves at idle */
    bool gen_neutral = s->gen.active && gear_is(&s->gear, GEAR_N) && s->gear_demand == GEAR_N &&
                       (sb_mode(s) == MODE_STANDBY || sb_mode(s) == MODE_DP);
    float lim_dir = (s->gear_demand == GEAR_F) ? fminf(s->plim.fwd_pct, slam_cap(&s->slam))
                  : gen_neutral ? fminf(s->gen.cap_pct, s->plim.fwd_pct)
                  : s->plim.rev_pct;
    if (s->throttle_demand > lim_dir) s->throttle_demand = lim_dir;
    gear_request(&s->gear, s->gear_demand);
    /* STANDBY (button, fault, alarm ...): the faster closing rate; a planned
     * arrival stop and normal driving keep the normal one */
    /* an EMERGENCY = idle AT ONCE, no rate limit (step 11e): STANDBY (button / fault / e-stop / node loss /
     * link loss ...) - not a planned arrival stop, not a normal generator stop - and the REMOTE / MANEUVER
     * link-loss failsafe (stale data) */
    s->thr.stop = (sb_mode(s) == MODE_STANDBY && !s->stop_planned && !(s->gen.active && !s->gen.fast_stop)) ||
                  (remote_like(sb_mode(s)) && s->remote.state == SB_REMOTE_LOST) ||
                  s->emerg;                     /* step 11g: an emergency in STANDBY too (arrival / generator stop) */
    s->thr.neutral_run_pct = gen_neutral ? s->gen.cap_pct : 0.0f;   /* 0 = the normal rule: idle in neutral */
    s->throttle_rate = throttle_step(&s->thr, s->throttle_demand, s->gear.current, gear_throttle_allowed(&s->gear), dt);

    /* 6. outputs: engine first, steering last. The servo module clamps to
     *    the power limit once more (lowest layer); the rate limiter goes on
     *    from what was really sent. */
    s->thr.out_pct = servo_throttle_write(&s->servo, &s->plim, gear_is(&s->gear, GEAR_F), s->thr.out_pct, dt,
                                          gen_neutral ? s->gen.cap_pct : -1.0f);
    servo_model_step(&s->servo, thr_node_ok(s), dt);
    /* the gear moves only when the MODELLED throttle servo is at idle (DroneCAN: never while the throttle
     * node is lost - its position is unknown, est 100 %) */
    gear_step(&s->gear, dt_ms, s->servo.est_pct, neutral_sw);
    if (gear_fault(&s->gear)) {
        alarm_on(s, SB_ALM_GEAR);
        /* neutral not confirmed: driving modes end in STANDBY (throttle
         * stays idle); MANUAL keeps the steering on the stick */
        if (sb_mode(s) != MODE_STANDBY && sb_mode(s) != MODE_MANUAL)
            sb_request_mode(s, MODE_STANDBY, WHY_GEAR_FAULT);
    }
    servo_gear_write(&s->servo, s->gear.pwm_us, s->cfg.gear.idle_max_pct);
    servo_steer_write(&s->servo, s->ap.motor_cmd, s->ap.servo_have, s->ap.servo_u);
    if (s->chirp_ms > 0u) s->chirp_ms = (s->chirp_ms > dt_ms) ? s->chirp_ms - dt_ms : 0u;
    hal_set_alarm(s->alarm || s->ap.alarm || s->chirp_ms > 0u);

    /* 6b. engine controls (section N): a crank whose conditions failed stops now
     *     (the STARTER OFF goes to the relay board in 8b), a pending restart
     *     reset is checked again (it happens at the end of this step) */
    bool eng_reset;
    {
        eng_cond_t ec;
        eng_cond(s, &ec, (buttons & HAL_BTN_BIT(HAL_BTN_STANDBY)) != 0u);
        eng_reset = engine_step(&s->eng, &ec, &s->relays, s->now_ms);
    }

    /* 7. publish for the independent safety monitor (CPU1), as one set (seqlock) */
    monitor_write_begin(&g_mon);
    g_mon.throttle_pct = s->servo.out_pct;
    g_mon.fwd_limit = s->plim.fwd_pct;
    g_mon.rev_limit = s->plim.rev_pct;
    g_mon.fwd_engaged = gear_is(&s->gear, GEAR_F) ? 1u : 0u;
    g_mon.gear_pwm_us = s->servo.gear_sent ? s->servo.gear_us : s->gear.pwm_us;   /* what the gear servo got */
    g_mon.idle_max_pct = s->cfg.gear.idle_max_pct;
    g_mon.thr_idle_us = s->cfg.servo.idle_us;
    g_mon.gear_n_us = s->cfg.gear.pwm_n_us;
    g_mon.gen_active = s->gen.active ? 1u : 0u;     /* throttle above idle in neutral allowed up to the cap */
    g_mon.gen_cap_pct = s->gen.cap_pct;
    g_mon.dp_active = s->thrust.active ? 1u : 0u;   /* thrust allowed only while DP runs, within the caps */
    g_mon.thr_bow_pct = s->thrust.out_pct[THR_BOW];
    g_mon.thr_stern_pct = s->thrust.out_pct[THR_STERN];
    g_mon.thr_bow_cap = thruster_cap(&s->thrust, THR_BOW);
    g_mon.thr_stern_cap = thruster_cap(&s->thrust, THR_STERN);
    /* DroneCAN servos (step 11e): what CPU1 needs for its safe frames (it replays the last steering /
     * gear wire values, never makes up a position), its own throttle envelope, MON_TRIP_CAN_TX and the
     * UNCREWED dead-man (profile, kill latch, the throttle node) */
    {
        hal_can_act_t ca;
        bool can = s->servo.can && hal_can_act_state(&ca);
        g_mon.can_servos = can ? 1u : 0u;
        g_mon.steer_have = (can && ca.steer_have) ? 1u : 0u;
        g_mon.steer_wire = can ? ca.steer_wire : 0.0f;
        g_mon.gear_have = (can && ca.gear_have) ? 1u : 0u;
        g_mon.gear_wire = can ? ca.gear_wire : 0.0f;
        g_mon.gear_n_wire = hal_can_wire_us(s->cfg.gear.pwm_n_us);
        g_mon.thr_node_ok = thr_node_ok(s) ? 1u : 0u;
        g_mon.thr_travel_ms = (uint32_t)(s->servo.cfg.travel_s * 1000.0f + 0.5f);
        g_mon.profile = s->profile;
        g_mon.killed = s->ll.state == SB_LL_KILLED ? 1u : 0u;
    }
    monitor_write_end(&g_mon);
    if (s->bench.trip_req) g_mon.test_trip = MON_TEST_TRIP;     /* BENCH TEST: trip test (latched) */
    g_mon.heartbeat = g_mon.heartbeat + 1u;
    g_mon.running = MON_RUNNING;                    /* the control loop runs (monitor watches from now) */

    /* 8. video SAVE requests for the gateway */
    save_triggers(s);

    /* 8b. relay board (section M): the driver's bounded step, readback, time
     *     limits, the one-shot power_up. Nothing of it reaches the control
     *     path above; a silent board is an ADVISORY (states unknown) */
    relays_step(&s->relays, s->now_ms);
    if (relays_silent(&s->relays)) alarm2_on(s, SB_ALM2_RELAY_SILENT);

    /* 9. non-volatile storage: queued saves reach the data flash only in
     *    STANDBY (not stopping); one bounded step, never a wait */
    if (s->relays.table_new && sb_mode(s) == MODE_STANDBY && !s->stopping) {
        s->relays.table_new = false;                /* the confirmed relay table (slot 7) */
        (void)nv_save(HAL_NV_SLOT_RELAYS, &s->relays.table, (uint32_t)sizeof s->relays.table);
    }
    if (s->gen.learned_new && sb_mode(s) == MODE_STANDBY && !s->stopping && !s->gen.active) {
        gen_nv_t r;                                     /* the generator's learned value (slot 6) */
        memset(&r, 0, sizeof r);
        r.version = GEN_NV_VERSION;
        r.pct = (uint8_t)(s->gen.learned_pct + 0.5f);
        r.crc = gen_nv_crc(&r);
        (void)nv_save(HAL_NV_SLOT_GENERATOR, &r, (uint32_t)sizeof r);
        s->gen.learned_new = false;
    }
    nv_step(sb_mode(s) == MODE_STANDBY && !s->stopping, dt_ms);

    /* 10. remote restart after a monitor trip (section N): the controlled
     *     software reset, once the event is logged and the flash is idle, the
     *     conditions met until now. Everything starts again (board: never returns). */
    if (eng_reset) {
        s->eng.reset_pending = false;
        hal_system_reset();
    }
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
    st.alarm_bits2 = s->alarm_bits2;
    st.gps_suspects = s->dr.suspects;
    return st;
}
