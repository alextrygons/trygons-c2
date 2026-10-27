/*
 * logcore0.c - Core 0 log producer. See logcore0.h.
 */
#include "logcore0.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "logq.h"
#include "logrec.h"
#include "monitor.h"
#include "timing.h"
#include "nv.h"

/* What was seen last step (for the "something changed" events). */
static struct {
    bool     init;
    uint32_t mode_log_count;
    uint16_t save_seq;
    uint32_t slam_impacts, slam_code;
    uint32_t nav_code, nav_hdg;
    uint8_t  profile, ll_state;
    uint16_t nv_mask;
    bool     nv_fault;
    uint8_t  gen_state;
    bool     gen_enabled;
    float    gen_pct;
    uint32_t relay_seq, relay_states, relay_known;
    bool     relay_ok;
    uint32_t eng_seq;
    uint32_t thr_seq;
    uint32_t dp_seq;
    uint32_t alarm_bits, alarm_bits2;
    bool     ap_alarm;
    uint32_t plim_log_count;
    uint8_t  tr_stage, tr_phase, tr_result;
    uint8_t  dr_state;
    bool     gear_fault, compass_fault, joy_fault, mon_tripped;
    uint32_t overruns;
    uint8_t  auto_state, auto_wp;
    bool     auto_susp, auto_fin;
    uint32_t legs_done;
    uint32_t conf_acc, conf_ref, conf_ign;
    uint32_t cfg_version;
    uint32_t gps_losses;
    uint32_t time_minute;
} k;

static uint8_t s_rec[LOGR_MAX + LOGR_OVERHEAD];

/* --- the 20 Hz sample ------------------------------------------------------- */

typedef struct {
    float  heading_err, pid_p, pid_d;
    uint32_t gps_age_ms;
    double est_lat, est_lon;
    float  est_sog, est_cog, cur_conf;
    float  water_kn, motor_a;
    uint8_t joy_buttons, joy_status, joy_src;
    uint32_t t_core0, t_mon, t_log;
} derived_t;

static uint8_t *put(uint8_t *p, char type, double scale, double value)
{
    double x = value * scale;
    if (!(x == x)) x = 0.0;                         /* NaN */
    switch (type) {
    case 'B': { double c = x < 0 ? 0 : (x > 255 ? 255 : x); *p++ = (uint8_t)(c + 0.5); break; }
    case 'c': { double c = x < -128 ? -128 : (x > 127 ? 127 : x); *p++ = (uint8_t)(int8_t)lround(c); break; }
    case 'H': { double c = x < 0 ? 0 : (x > 65535 ? 65535 : x); uint16_t u = (uint16_t)(c + 0.5);
                *p++ = (uint8_t)u; *p++ = (uint8_t)(u >> 8); break; }
    case 'h': { double c = x < -32768 ? -32768 : (x > 32767 ? 32767 : x); uint16_t u = (uint16_t)(int16_t)lround(c);
                *p++ = (uint8_t)u; *p++ = (uint8_t)(u >> 8); break; }
    case 'I': { double c = x < 0 ? 0 : (x > 4294967295.0 ? 4294967295.0 : x); uint32_t u = (uint32_t)(c + 0.5);
                *p++ = (uint8_t)u; *p++ = (uint8_t)(u >> 8); *p++ = (uint8_t)(u >> 16); *p++ = (uint8_t)(u >> 24);
                break; }
    case 'i': { double c = x < -2147483648.0 ? -2147483648.0 : (x > 2147483647.0 ? 2147483647.0 : x);
                uint32_t u = (uint32_t)(int32_t)llround(c);
                *p++ = (uint8_t)u; *p++ = (uint8_t)(u >> 8); *p++ = (uint8_t)(u >> 16); *p++ = (uint8_t)(u >> 24);
                break; }
    default:  { float f = (float)value; uint32_t u; memcpy(&u, &f, 4);
                *p++ = (uint8_t)u; *p++ = (uint8_t)(u >> 8); *p++ = (uint8_t)(u >> 16); *p++ = (uint8_t)(u >> 24);
                break; }
    }
    return p;
}

#define FIELD_PUT(n, t, sc, e) p = put(p, (#t)[0], (double)(sc), (double)(e));
#include "logfields.h"

static uint16_t data_sample(const sb_t *s, uint8_t *out)
{
    derived_t v;
    memset(&v, 0, sizeof v);
    v.heading_err = autopilot_heading_error(s->ap.ref_heading, s->ap.heading);
    v.pid_p = s->ap.cfg.kp * s->ap.gain_scale * v.heading_err;
    v.pid_d = -s->ap.cfg.kd * s->ap.gain_scale * (s->ap.yaw_rate - s->ap.ref_rate);
    v.gps_age_ms = s->gps.valid ? s->now_ms - s->gps.last_ms : 65535u;
    if (dr_have_position(&s->dr)) dr_latlon(&s->dr, &v.est_lat, &v.est_lon);
    v.est_sog = dr_sog_kn(&s->dr);
    v.est_cog = dr_cog_deg(&s->dr);
    float tau4 = 4.0f * s->dr.cfg.current_tau_s;
    v.cur_conf = (tau4 > 0.0f) ? fminf(1.0f, s->dr.cur_learn_s / tau4) : 0.0f;
    v.water_kn = s->dr.speed_sensor ? s->dr.water_kn : 0.0f;
    v.motor_a = s->motor_a >= 0.0f ? s->motor_a : 655.35f;
    v.joy_buttons = (uint8_t)((s->joy.take ? 1u : 0u) | (s->joy.autob ? 2u : 0u) | (s->joy.confirm ? 4u : 0u));
    v.joy_status = (uint8_t)(!s->joy.present ? 0u : s->joy.fault ? 3u : s->joy.armed ? 2u : 1u);
    /* b0 USB source, b1 lever, b2 pickup pending, b3/b4 USB stick/throttle connected, b5 learn */
    v.joy_src = (uint8_t)((s->jmap.source == JOY_SRC_BRIDGE ? 1u : 0u) | (s->joy.has_lever ? 2u : 0u) |
                          (s->lev.pickup ? 4u : 0u) | ((s->jb.last.flags & JSF_F_DEV0) ? 8u : 0u) |
                          ((s->jb.last.flags & JSF_F_DEV1) ? 16u : 0u) | ((s->jb.last.flags & JSF_F_LEARN) ? 32u : 0u));
    v.t_core0 = g_timing.core0.last_us;
    v.t_mon = g_timing.monitor.pub_max_us;
    v.t_log = g_timing.logger.pub_max_us;
    uint8_t *p = out;
    LOG_FIELDS(FIELD_PUT)
    return (uint16_t)(p - out);
}

/* --- config text ------------------------------------------------------------ */

uint16_t logcore0_config_text(const sb_t *s, char *out, uint16_t max)
{
    const sb_config_t *c = &s->cfg;
    const ap_config_t *a = &s->ap.cfg;
    const ap_outboard_config_t *o = &a->outboard;
    int n = snprintf(out, max,
        "steer.kp=%g\nsteer.ki=%g\nsteer.kd=%g\nsteer.turn_rate_dps=%g\nsteer.sea_state=%u\n"
        "outboard.max_angle_deg=%g\noutboard.turn_speed_deg_s=%g\noutboard.gap_deg=%g\ncalibrated=%u\n"
        "gear.pwm_f_us=%u\ngear.pwm_n_us=%u\ngear.pwm_r_us=%u\ngear.idle_wait_ms=%lu\ngear.neutral_wait_ms=%lu\n"
        "gear.engage_wait_ms=%lu\nthrottle.max_rev_pct=%g\nthrottle.rate_up=%g\nthrottle.rate_down=%g\nthrottle.rate_stop=%g\n"
        "servo.idle_us=%u\nservo.full_us=%u\nplim.fwd_start=%g\nplim.rev_start=%g\nplim.rev_max=%g\n"
        "cruise_max_kn=%g\nspd_kp=%g\nspd_ki=%g\ngain_ref_kn=%g\nslow_radius_m=%g\napproach_kn=%g\n"
        "dr_max_unc_m=%g\ndr_max_ms=%lu\ndr_speed_factor=%g\ndr_slow_kn=%g\ntraining_throttle_pct=%g\n"
        "joy.deadband=%g\njoy.max_throttle_pct=%g\njoy.cal.center=%u,%u\njoy.cal.min=%u,%u\njoy.cal.max=%u,%u\n"
        "confirm_ms=%lu\narm_ms=%lu\nfault_steer=%u\n",
        a->kp, a->ki, a->kd, a->turn_rate_dps, (unsigned)a->sea_state,
        o->max_angle_deg, o->turn_speed_deg_s, o->gap_deg, (unsigned)s->ap.calibrated,
        (unsigned)c->gear.pwm_f_us, (unsigned)c->gear.pwm_n_us, (unsigned)c->gear.pwm_r_us,
        (unsigned long)c->gear.idle_wait_ms, (unsigned long)c->gear.neutral_wait_ms,
        (unsigned long)c->gear.engage_wait_ms, c->throttle.max_rev_pct, c->throttle.rate_up_pct_s,
        c->throttle.rate_down_pct_s, s->thr.cfg.rate_stop_pct_s, (unsigned)c->servo.idle_us, (unsigned)c->servo.full_us,
        c->plim.fwd_steps[0], c->plim.rev_start, c->plim.rev_max,
        c->cruise_max_kn, c->spd_kp, c->spd_ki, c->gain_ref_kn, c->slow_radius_m, c->approach_kn,
        c->dr_max_unc_m, (unsigned long)c->dr_max_ms, c->dr_speed_factor, c->dr_slow_kn, c->training_throttle_pct,
        s->joy.cfg.deadband, c->joy_max_throttle_pct, (unsigned)s->joy.cal.center[0], (unsigned)s->joy.cal.center[1],
        (unsigned)s->joy.cal.min[0], (unsigned)s->joy.cal.min[1], (unsigned)s->joy.cal.max[0],
        (unsigned)s->joy.cal.max[1], (unsigned long)c->confirm_ms, (unsigned long)c->arm_ms,
        (unsigned)c->fault_steer);
    if (n > 0 && n < (int)max) {
        const joymap_t *m = &s->jmap;
        n += snprintf(out + n, (size_t)(max - n),
                      "joy.source=%s\njoy.thr_input=%s\njoy.axis_steer=%u\njoy.axis_thr=%u\njoy.invert=%u,%u\n"
                      "joy.buttons=%u,%u,%u,%u,%u,%u,%u,%u,%u\njoy.neutral_zone=%g,%g\njoy.lever_delta=%g\n"
                      "joy.pickup_win=%g\n",
                      m->source == JOY_SRC_BRIDGE ? "usb" : "analog", m->thr_input == JOY_THR_LEVER ? "lever" : "stick_y",
                      (unsigned)m->axis_steer, (unsigned)m->axis_thr, (unsigned)m->inv_steer, (unsigned)m->inv_thr,
                      (unsigned)m->btn[0], (unsigned)m->btn[1], (unsigned)m->btn[2], (unsigned)m->btn[3],
                      (unsigned)m->btn[4], (unsigned)m->btn[5], (unsigned)m->btn[6], (unsigned)m->btn[7],
                      (unsigned)m->btn[8], m->nz_lo, m->nz_hi, m->lever_delta, m->pickup_win);
    }
    if (n > 0 && n < (int)max) {
        const sb_auto_t *p = &s->autop;
        n += snprintf(out + n, (size_t)(max - n), "program.n=%u\nprogram.final=%u\n",
                      p->prog_ok ? (unsigned)p->prog.n : 0u, (unsigned)p->prog.final_action);
        for (unsigned i = 0; p->prog_ok && i < p->prog.n && n > 0 && n < (int)max - 80; i++) {
            const prog_wp_t *w = &p->prog.wp[i];
            n += snprintf(out + n, (size_t)(max - n), "program.wp%u=%.7f,%.7f,%.1f,%.0f,%u\n", i + 1u,
                          program_lat(w), program_lon(w), w->speed_kn, w->radius_m, (unsigned)w->action);
        }
    }
    return (n < 0) ? 0u : (uint16_t)((n < (int)max) ? n : (int)max - 1);
}

static void push_config(const sb_t *s)
{
    static char txt[LOGR_MAX];
    uint16_t n = logcore0_config_text(s, txt, sizeof txt);
    uint16_t len = logrec_make(s_rec, LOGR_CONFIG, s->now_ms, txt, n);
    (void)logq_push(&g_logq0, s_rec, len);
}

/* --- public ------------------------------------------------------------------- */

void logcore0_init(const sb_t *s, uint32_t boot_no)
{
    uint32_t raw = 0u;
    hal_reset_t why = hal_reset_reason(&raw);
    memset(&k, 0, sizeof k);
    /* the "seen" state starts now: everything after the boot event is logged */
    k.init = true;
    k.mode_log_count = s->modes.log_count;
    k.plim_log_count = s->plim.log_count;
    k.dr_state = (uint8_t)s->dr.state;
    k.auto_state = (uint8_t)s->autop.state;
    k.auto_wp = s->autop.wp;
    k.alarm_bits = s->alarm_bits;
    k.conf_acc = s->confirm.accepted;
    k.conf_ref = s->confirm.refused;
    k.conf_ign = s->confirm.ignored;
    k.gps_losses = s->gps_losses;
    (void)logrec_event(&g_logq0, s->now_ms, EV_BOOT, (int32_t)why, (int32_t)raw, (int32_t)boot_no,
                       (int32_t)LOG_FORMAT_VERSION);
    push_config(s);
    k.cfg_version = s->cfg_version;
}

void logcore0_pc(uint32_t t_ms, uint8_t type, uint8_t seq, uint8_t result, const uint8_t *payload, uint8_t len,
                 uint8_t station)
{
    uint8_t pl[4 + 200 + 1];
    pl[0] = type; pl[1] = seq; pl[2] = result; pl[3] = len;
    if (len) memcpy(&pl[4], payload, len);
    pl[4 + len] = station;                      /* after the command: older readers ignore it */
    uint16_t n = logrec_make(s_rec, LOGR_PCCMD, t_ms, pl, (uint16_t)(5u + len));
    (void)logq_push(&g_logq0, s_rec, n);
}

void logcore0_helm(uint32_t t_ms, uint8_t station, uint8_t action, uint8_t refused, uint8_t holder)
{
    (void)logrec_event(&g_logq0, t_ms, EV_HELM, station, action, refused, holder);
}

void logcore0_param(const sb_t *s, uint8_t id, float old_v, float new_v)
{
    (void)logrec_event(&g_logq0, s->now_ms, EV_CONFIG, 1, id, (int32_t)lroundf(old_v * 1000.0f),
                       (int32_t)lroundf(new_v * 1000.0f));
}

static void ev(const sb_t *s, uint16_t code, int32_t a, int32_t b, int32_t c, int32_t d)
{
    (void)logrec_event(&g_logq0, s->now_ms, code, a, b, c, d);
}

static void changes(const sb_t *s)
{
    if (!k.init) {
        k.init = true;
        k.mode_log_count = s->modes.log_count;
        k.plim_log_count = s->plim.log_count;
        k.dr_state = (uint8_t)s->dr.state;
        k.auto_state = (uint8_t)s->autop.state;
        k.auto_wp = s->autop.wp;
        return;
    }
    /* mode requests (accepted and refused) */
    while (k.mode_log_count < s->modes.log_count) {
        const mode_log_entry_t *e = &s->modes.log[k.mode_log_count % MODE_LOG_N];
        ev(s, EV_MODE, e->from, e->to, e->why, e->verdict);
        k.mode_log_count++;
    }
    /* alarms */
    uint32_t on = s->alarm_bits & ~k.alarm_bits, off = k.alarm_bits & ~s->alarm_bits;
    if (on || (s->ap.alarm && !k.ap_alarm)) ev(s, EV_ALARM_ON, (int32_t)on, (int32_t)s->alarm_bits, s->ap.alarm, 0);
    if (off) ev(s, EV_ALARM_OFF, (int32_t)off, (int32_t)s->alarm_bits, 0, 0);
    k.alarm_bits = s->alarm_bits;
    k.ap_alarm = s->ap.alarm;
    /* the second alarm word: the same events with word = 2 (d) */
    uint32_t on2 = s->alarm_bits2 & ~k.alarm_bits2, off2 = k.alarm_bits2 & ~s->alarm_bits2;
    if (on2) ev(s, EV_ALARM_ON, (int32_t)on2, (int32_t)s->alarm_bits2, 0, 2);
    if (off2) ev(s, EV_ALARM_OFF, (int32_t)off2, (int32_t)s->alarm_bits2, 0, 2);
    k.alarm_bits2 = s->alarm_bits2;
    /* slamming: every impact, and every change of state / cap / override (not
     * in the 20 Hz sample: the black box has no room left) */
    {
        const slam_t *sl = &s->slam;
        uint32_t code = (uint32_t)lroundf(slam_cap(sl) * 10.0f) | ((uint32_t)sl->state << 16) |
                        ((sl->override ? 1u : 0u) << 20);
        bool hit = sl->impacts != k.slam_impacts;
        if (hit || code != k.slam_code)
            ev(s, EV_SLAM, hit ? (int32_t)lroundf(sl->peak * 100.0f) : 0, (int32_t)lroundf(sl->level * 100.0f),
               (int32_t)sl->rate_pm, (int32_t)code);
        k.slam_impacts = sl->impacts;
        k.slam_code = code;
    }
    /* gateway navigation requests: every change (what was asked, what is applied) */
    {
        const sb_navreq_t *nv = &s->nav;
        bool fresh = nv->rx_ms != 0u && s->now_ms - nv->rx_ms <= nv->lease_ms;
        bool applied = (sb_mode(s) == MODE_AUTO || (sb_mode(s) == MODE_DP && s->dp.sub == DP_TRIP)) && (nv->lost || (fresh && (nv->kind == SB_NAV_CAP ||
                                                                          nv->kind == SB_NAV_HEADING)));
        uint32_t code = (uint32_t)(fresh ? nv->kind : 0u) | ((uint32_t)nv->reason << 8) | ((uint32_t)nv->alarm << 16) |
                        ((nv->lost ? 1u : 0u) << 24) | ((applied ? 1u : 0u) << 25);
        uint32_t hd = (uint32_t)lroundf(nv->heading * 10.0f) + nv->applied_hdg * 4000u;
        if (code != k.nav_code || (fresh && hd != k.nav_hdg))
            ev(s, EV_NAV, (int32_t)code, (int32_t)lroundf(nv->heading * 100.0f),
               nv->cap_kn < 0.0f ? -1 : (int32_t)lroundf(nv->cap_kn * 10.0f), nv->seq);
        k.nav_code = code;
        k.nav_hdg = hd;
    }
    /* operating profile (CREWED / UNCREWED): every change */
    if (s->profile != k.profile)
        ev(s, EV_PROFILE, s->profile, s->joy_probe.present && s->probe_ready && !s->joy_probe.fault ? 1 : 0,
           s->profile_changes, 0);
    k.profile = s->profile;
    if (s->ll.state != k.ll_state)
        ev(s, EV_LINKLOST, s->ll.state, sb_near_shore(s) ? 1 : 0, s->coast.dist_m, (int32_t)s->ll.losses);
    k.ll_state = s->ll.state;
    /* non-volatile storage: a stored record refused (defaults in use), NV FAULT */
    {
        const nv_status_t *nv = nv_status();
        uint16_t fresh = (uint16_t)(nv->reset_mask & ~k.nv_mask);
        for (uint8_t slot = 1; slot < 16u && fresh; slot++) {
            if (fresh & (1u << slot)) ev(s, EV_NV, 1, slot, nv->resets, 0);
        }
        if (nv->fault && !k.nv_fault) ev(s, EV_NV, 2, 0, nv->resets, 0);
        k.nv_mask = nv->reset_mask;
        k.nv_fault = nv->fault;
    }
    /* generator mode: every state change, switch on / off and throttle step
     * (events, not the 20 Hz sample: the black box has 2 bytes left) */
    if (s->gen.state != k.gen_state || s->gen.enabled != k.gen_enabled || s->gen.demand_pct != k.gen_pct) {
        ev(s, EV_GENERATOR, s->gen.state, (int32_t)s->gen.stop_why | (s->gen.enabled ? 256 : 0),
           s->batt.valid ? (int32_t)(s->batt.bank_v * 100.0f) : -1, (int32_t)(s->gen.demand_pct * 10.0f));
        k.gen_state = s->gen.state;
        k.gen_enabled = s->gen.enabled;
        k.gen_pct = s->gen.demand_pct;
    }
    /* relay board: every command / table event, readback change, board lost / back (events only) */
    {
        const relays_t *rl = &s->relays;
        int32_t cd = (int32_t)(rl->hash | ((uint32_t)relays_flags(rl) << 16) | ((uint32_t)(rl->have_table ? rl->table.count : 0u) << 24));
        int32_t cc = (int32_t)((rl->known & 0x7FFFFFFFu) | (rl->board_ok ? 0x80000000u : 0u));
        if (rl->log_seq != k.relay_seq)
            ev(s, EV_RELAY, (int32_t)(rl->last_n | ((uint32_t)rl->last_action << 8) | ((uint32_t)rl->last_result << 16) |
                                      ((uint32_t)rl->last_what << 24)), (int32_t)rl->states, cc, cd);
        else if (rl->states != k.relay_states || rl->known != k.relay_known)
            ev(s, EV_RELAY, 3 << 24, (int32_t)rl->states, cc, cd);
        else if (rl->board_ok != k.relay_ok)
            ev(s, EV_RELAY, 4 << 24, (int32_t)rl->states, cc, cd);
        k.relay_seq = rl->log_seq;
        k.relay_states = rl->states;
        k.relay_known = rl->known;
        k.relay_ok = rl->board_ok;
    }
    /* engine controls / remote restart (section N): every restart, crank start / stop, refusal.
     * The reset after a trip waits ENG_RESET_DELAY_MS so this event is in the queue first. */
    if (s->eng.log_seq != k.eng_seq)
        ev(s, EV_RESTART, s->eng.last_action, s->eng.last_why, (int32_t)s->eng.hold_ms,
           (int32_t)((s->eng.restarts & 0xFFFFu) | ((s->eng.cranks & 0xFFFFu) << 16)));
    k.eng_seq = s->eng.log_seq;
    /* DP thrusters (section P): settings taken, a thruster HOT / cooled down (events only) */
    if (s->thrust.log_seq != k.thr_seq) {
        const thruster_t *t = &s->thrust;
        ev(s, EV_DP, t->log_what, t->log_unit,
           (int32_t)((uint32_t)thruster_cap(t, THR_BOW) | ((uint32_t)thruster_cap(t, THR_STERN) << 8)),
           (int32_t)((uint32_t)(thruster_heat(t, t->log_unit) * 100.0f + 0.5f) | ((uint32_t)t->cfg_hash << 8)));
    }
    k.thr_seq = s->thrust.log_seq;
    /* DP (section P): start / sub-state change, JOG, the end of a TRIP, the stop (events only: the 20 Hz
     * sample is full) - what 4 start / sub, 5 stopped, 6 jog, 7 trip end; sub | stop why << 8 | wp << 16,
     * spot error m*10, heading target*10 | set speed kn*10 << 16 */
    if (s->dp.log_seq != k.dp_seq) {
        const dp_t *d = &s->dp;
        ev(s, EV_DP, d->log_what, (int32_t)((uint32_t)d->sub | ((uint32_t)d->stop_why << 8) | ((uint32_t)d->wp << 16)),
           (int32_t)lroundf(d->err_m * 10.0f),
           (int32_t)((uint32_t)lroundf(d->hdg_target * 10.0f) | ((uint32_t)lroundf(d->set_kn * 10.0f) << 16)));
    }
    k.dp_seq = s->dp.log_seq;
    /* video SAVE requests */
    if (s->save.seq != k.save_seq) ev(s, EV_SAVE, s->save.trig, s->save.arg, s->save.seq, 0);
    k.save_seq = s->save.seq;
    /* buttons */
    if (s->buttons_now) ev(s, EV_BUTTON, (int32_t)s->buttons_now, 0, 0, 0);
    if (s->joy.take_edge || s->joy.auto_edge || s->joy.confirm_edge)
        ev(s, EV_JOY_BUTTON, (s->joy.take_edge ? 1 : 0) | (s->joy.auto_edge ? 2 : 0) | (s->joy.confirm_edge ? 4 : 0),
           (int32_t)lroundf(s->joy.x * 1000.0f), (int32_t)lroundf(s->joy.y * 1000.0f), s->joy.armed);
    /* power limit: every change and attempt */
    while (k.plim_log_count < s->plim.log_count) {
        const plim_log_t *e = &s->plim.log[k.plim_log_count % PLIM_LOG_N];
        if (s->plim.log_count - k.plim_log_count <= PLIM_LOG_N)
            ev(s, EV_PLIM, (int32_t)(e->source | (e->result << 8) | (e->attempt << 16) | ((uint32_t)e->dir << 24)),
               (int32_t)lroundf(e->old_fwd * 10.0f), (int32_t)lroundf(e->new_fwd * 10.0f),
               (int32_t)(((uint32_t)lroundf(e->old_rev * 10.0f) << 16) | (uint32_t)lroundf(e->new_rev * 10.0f)));
        k.plim_log_count++;
    }
    /* training */
    if ((uint8_t)s->train.stage != k.tr_stage || (uint8_t)s->ap.train.phase != k.tr_phase ||
        (uint8_t)s->ap.train.result != k.tr_result) {
        k.tr_stage = (uint8_t)s->train.stage;
        k.tr_phase = (uint8_t)s->ap.train.phase;
        k.tr_result = (uint8_t)s->ap.train.result;
        if (sb_mode(s) == MODE_TRAINING || s->ap.train.result != TR_OK)
            ev(s, EV_TRAINING, k.tr_stage, k.tr_phase, k.tr_result, 0);
    }
    /* GPS / position source */
    if ((uint8_t)s->dr.state != k.dr_state || s->gps_losses != k.gps_losses) {
        k.dr_state = (uint8_t)s->dr.state;
        k.gps_losses = s->gps_losses;
        ev(s, EV_GPS, k.dr_state, s->dr.why, (int32_t)lroundf(s->dr.unc_m * 10.0f), (int32_t)s->gps_losses);
    }
    /* faults */
    bool gf = gear_fault(&s->gear), cf = s->ap.sensor_fault, jf = s->joy.fault, mf = s->mon_tripped;
    if (gf && !k.gear_fault) ev(s, EV_FAULT, 1, s->gear.pwm_us, s->neutral_sw, 0);
    if (cf && !k.compass_fault) ev(s, EV_FAULT, 2, 0, 0, 0);
    if (jf && !k.joy_fault) ev(s, EV_FAULT, 3, s->joy.fault_axis, s->joy.raw[0], s->joy.raw[1]);
    if (mf && !k.mon_tripped) ev(s, EV_FAULT, 4, g_mon.reason, 0, 0);
    if (g_timing.core0.overruns != k.overruns) ev(s, EV_FAULT, 5, (int32_t)g_timing.core0.last_us, 0, 0);
    k.gear_fault = gf; k.compass_fault = cf; k.joy_fault = jf; k.mon_tripped = mf;
    k.overruns = g_timing.core0.overruns;
    /* AUTO program */
    const sb_auto_t *a = &s->autop;
    if ((uint8_t)a->state != k.auto_state || a->wp != k.auto_wp || a->suspended != k.auto_susp ||
        a->finished != k.auto_fin || a->legs_done != k.legs_done) {
        int32_t action = a->finished && !k.auto_fin ? 5 : a->legs_done != k.legs_done ? 6 :
                         a->state == AUTO_PAUSED ? 2 : (a->wp != k.auto_wp && a->legs_done == k.legs_done) ? 4 :
                         (uint8_t)a->state == AUTO_RUN && k.auto_state != AUTO_RUN ? 3 : 7;
        ev(s, EV_PROGRAM, action, a->wp, a->state, (int32_t)a->legs_done);
        k.auto_state = (uint8_t)a->state; k.auto_wp = a->wp; k.auto_susp = a->suspended;
        k.auto_fin = a->finished; k.legs_done = a->legs_done;
    }
    /* CONFIRM presses */
    if (s->confirm.accepted != k.conf_acc) ev(s, EV_CONFIRM, s->confirm.kind, 0, (int32_t)s->confirm.accepted, 0);
    if (s->confirm.refused != k.conf_ref) ev(s, EV_CONFIRM, s->confirm.kind, 1, (int32_t)s->confirm.refused, 0);
    if (s->confirm.ignored != k.conf_ign) ev(s, EV_CONFIRM, 0, 2, (int32_t)s->confirm.ignored, 0);
    k.conf_acc = s->confirm.accepted; k.conf_ref = s->confirm.refused; k.conf_ign = s->confirm.ignored;
    /* GPS date / time once a minute (the logger names files with it) */
    if (s->gps.valid && s->gps.last.date != 0u && s->gps.last.time_ms / 60000u != k.time_minute) {
        k.time_minute = s->gps.last.time_ms / 60000u;
        ev(s, EV_TIME, (int32_t)s->gps.last.date, (int32_t)s->gps.last.time_ms, (int32_t)s->now_ms, 0);
    }
    /* config / calibration / program changed: the whole config again */
    if (s->cfg_version != k.cfg_version) {
        k.cfg_version = s->cfg_version;
        ev(s, EV_CONFIG, 0, (int32_t)s->cfg_version, 0, 0);
        push_config(s);
    }
}

void logcore0_step(const sb_t *s)
{
    uint8_t pl[512];
    changes(s);
    uint16_t n = data_sample(s, pl);
    uint16_t len = logrec_make(s_rec, LOGR_DATA, s->now_ms, pl, n);
    (void)logq_push(&g_logq0, s_rec, len);
}
