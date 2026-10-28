/*
 * link.c - Ground-station link. See link.h, frame format in proto.h.
 */
#include "link.h"

#include <math.h>
#include <string.h>

#include "hal.h"
#include "timing.h"
#include "logcore0.h"
#include "logger.h"
#include "logq.h"
#include "logrec.h"
#include "nv.h"

#define TELEM_PERIOD_MS 100u
#define PI_F 3.14159265f

/* --- parameters ----------------------------------------------------------- */

typedef enum {
    P_STEER_KP = 0, P_STEER_KI, P_STEER_KD, P_TURN_RATE,
    P_CRUISE_MAX, P_SPD_KP, P_SPD_KI,
    P_SLOW_R, P_APPROACH_KN, P_TRAINING_THR,
    P_DR_MAX_UNC, P_DR_SPEED_F, P_DR_SLOW_KN,
    P_JOY_MAX_THR, P_JOY_DEADBAND, P_PC_TIMEOUT, P_SLAM_LIMIT, P_SLAM_RED_RESET, P_GPS2_CHECK, P_NAV_LOST_KN,
    P_STOP_RATE, P_NAV_MAX_DEV, P_NEAR_SHORE, P_LL_GRACE, P_BATT_LOW, P_GEN_MAX, P_GEN_START_V, P_GEN_IDLE,
    P_COUNT
} param_id_t;

static const link_param_info_t s_params[P_COUNT] = {
    { "steer_kp",        0.0f,   20.0f },
    { "steer_ki",        0.0f,    5.0f },
    { "steer_kd",        0.0f,   20.0f },
    { "turn_rate_dps",   1.0f,   30.0f },
    { "max_leg_kn",      3.0f,   35.0f },
    { "spd_kp",          0.0f,   20.0f },
    { "spd_ki",          0.0f,    5.0f },
    { "slow_r_m",       20.0f, 2000.0f },
    { "approach_kn",     1.0f,   15.0f },
    { "training_thr_pc", 5.0f,   40.0f },
    { "dr_max_unc_m",   20.0f, 2000.0f },
    { "dr_speed_fact",   0.2f,    1.0f },
    { "dr_slow_kn",      1.0f,   15.0f },
    { "joy_max_thr_pc",  5.0f,  100.0f },
    { "joy_deadband",    0.05f,   0.5f },
    { "pc_timeout_s",    1.0f,   30.0f },
    { "slam_limit_g",    1.0f,    8.0f },     /* captain sets, driver confirms (ground station) */
    { "slam_red_reset",  0.0f,    1.0f },     /* slamming RED: 0 = STEP (default), 1 = LIMIT_RESET */
    { "gps2_check_m",    0.0f, 2000.0f },     /* GPS fix vs the second GPS (AIS device); 0 = off */
    { "nav_lost_kn",     2.0f,   10.0f },     /* avoidance request lost (gateway gone): slow to this */
    { "stop_rate_pc_s", 40.0f,  300.0f },     /* STANDBY safe stop: throttle closing rate (100 = idle in 1 s) */
    { "nav_max_dev",    30.0f,  180.0f },     /* avoidance heading: max deg off the leg track (150) */
    { "near_shore_m",   20.0f, 2000.0f },     /* UNCREWED lost link: closer = engine kill at once (100, TO CONFIRM) */
    { "lostlink_grc_s",  0.0f,  120.0f },     /* UNCREWED lost link, open sea: kill after this (10, TO CONFIRM) */
    { "batt_low_v",     10.0f,   13.5f },     /* LOW BATTERY below this for 60 s (11.8 V, TO CONFIRM; advisory) */
    { "gen_max_pct",     0.0f,   30.0f },     /* generator: throttle cap in neutral (25, TO CONFIRM; 0 = disabled; boat file) */
    { "gen_start_v",    11.5f,   13.0f },     /* generator runs while the bank is below this (12.1 V) */
    { "gen_idle_pct",    3.0f,   15.0f },     /* generator: the fast idle, first step (8 %, TO CONFIRM) */
};

uint8_t link_param_count(void)
{
    return (uint8_t)P_COUNT;
}

const link_param_info_t *link_param_info(uint8_t id)
{
    return (id < P_COUNT) ? &s_params[id] : NULL;
}

/* Where each parameter lives. All of these are read live by the code. */
static float *param_ptr(link_t *l, sb_t *s, uint8_t id)
{
    switch ((param_id_t)id) {
    case P_STEER_KP:     return &s->ap.cfg.kp;
    case P_STEER_KI:     return &s->ap.cfg.ki;
    case P_STEER_KD:     return &s->ap.cfg.kd;
    case P_TURN_RATE:    return &s->ap.cfg.turn_rate_dps;
    case P_CRUISE_MAX:   return &s->cfg.cruise_max_kn;
    case P_SPD_KP:       return &s->cfg.spd_kp;
    case P_SPD_KI:       return &s->cfg.spd_ki;
    case P_SLOW_R:       return &s->cfg.slow_radius_m;
    case P_APPROACH_KN:  return &s->cfg.approach_kn;
    case P_TRAINING_THR: return &s->cfg.training_throttle_pct;
    case P_DR_MAX_UNC:   return &s->cfg.dr_max_unc_m;
    case P_DR_SPEED_F:   return &s->cfg.dr_speed_factor;
    case P_DR_SLOW_KN:   return &s->cfg.dr_slow_kn;
    case P_JOY_MAX_THR:  return &s->cfg.joy_max_throttle_pct;
    case P_JOY_DEADBAND: return &s->joy.cfg.deadband;
    case P_PC_TIMEOUT:   return &l->pc_timeout_s;
    case P_SLAM_LIMIT:   return &s->slam.cfg.limit_g;
    case P_SLAM_RED_RESET: return &s->slam.cfg.red_reset;
    case P_GPS2_CHECK:   return &s->dr.cfg.gps2_max_m;
    case P_NAV_LOST_KN:  return &s->cfg.nav_lost_kn;
    case P_STOP_RATE:    return &s->thr.cfg.rate_stop_pct_s;
    case P_NAV_MAX_DEV:  return &s->cfg.nav_max_dev_deg;
    case P_NEAR_SHORE:   return &s->cfg.near_shore_m;
    case P_LL_GRACE:     return &s->cfg.lostlink_grace_s;
    case P_BATT_LOW:     return &s->batt.cfg.low_v;
    case P_GEN_MAX:      return &s->gen.cfg.max_pct;
    case P_GEN_START_V:  return &s->gen.cfg.start_v;
    case P_GEN_IDLE:     return &s->gen.cfg.idle_pct;
    default:             return NULL;
    }
}

/* --- sending -------------------------------------------------------------- */

static uint32_t port_read(const link_t *l, uint8_t *buf, uint32_t max)
{
    return (l->port == LINK_PORT_GW) ? hal_gw_read(buf, max) : hal_link_read(buf, max);
}

static void send(link_t *l, uint8_t type, uint8_t seq, const uint8_t *payload, size_t len)
{
    uint8_t frame[PROTO_MAX_FRAME];
    size_t n = proto_encode(frame, type, seq, payload, (uint8_t)len);
    bool ok = n != 0u && ((l->port == LINK_PORT_GW) ? hal_gw_write(frame, (uint32_t)n)
                                                     : hal_link_write(frame, (uint32_t)n));
    if (!ok) l->tx_dropped++;
}

static void send_ack(link_t *l, const proto_frame_t *cmd, uint8_t result)
{
    uint8_t p[3] = { cmd->type, cmd->seq, result };
    l->last_result = result;
    if (result != PROTO_R_OK) l->nacks++;
    send(l, PROTO_T_ACK, cmd->seq, p, sizeof p);
}

/*
 * Joystick map body (JOYMAP_VALUE, JOYMAP_SET), 31 bytes:
 *   u8 source (0 analog, 1 USB) | u8 thr_input (0 stick Y, 1 lever) | u8 axis_steer | u8 axis_thr |
 *   u8 inv_steer | u8 inv_thr | u8 btn[9] (TAKE, AUTO, CONFIRM, HDG LOCK, STANDBY, -1, +1, -10, +10;
 *   codes app/jsframe.h) | f32 nz_lo | f32 nz_hi | f32 lever_delta | f32 pickup_win
 */
#define JOYMAP_BODY 32u

static void wr_joymap(proto_wr_t *w, const joymap_t *m)
{
    proto_wr_u8(w, m->source);
    proto_wr_u8(w, m->thr_input);
    proto_wr_u8(w, m->axis_steer);
    proto_wr_u8(w, m->axis_thr);
    proto_wr_u8(w, m->inv_steer);
    proto_wr_u8(w, m->inv_thr);
    for (int i = 0; i < JM_COUNT; i++) proto_wr_u8(w, m->btn[i]);
    proto_wr_f32(w, m->nz_lo);
    proto_wr_f32(w, m->nz_hi);
    proto_wr_f32(w, m->lever_delta);
    proto_wr_f32(w, m->pickup_win);
}

static void rd_joymap(proto_rd_t *r, joymap_t *m)
{
    memset(m, 0, sizeof *m);
    m->source = proto_rd_u8(r);
    m->thr_input = proto_rd_u8(r);
    m->axis_steer = proto_rd_u8(r);
    m->axis_thr = proto_rd_u8(r);
    m->inv_steer = proto_rd_u8(r);
    m->inv_thr = proto_rd_u8(r);
    for (int i = 0; i < JM_COUNT; i++) m->btn[i] = proto_rd_u8(r);
    m->nz_lo = proto_rd_f32(r);
    m->nz_hi = proto_rd_f32(r);
    m->lever_delta = proto_rd_f32(r);
    m->pickup_win = proto_rd_f32(r);
}

static void send_joymap(link_t *l, const sb_t *s)
{
    uint8_t buf[JOYMAP_BODY];
    proto_wr_t w;
    proto_wr_init(&w, buf, sizeof buf);
    wr_joymap(&w, &s->jmap);
    send(l, PROTO_T_JOYMAP_VALUE, l->tx_seq++, buf, w.n);
}

/* Last bridge frame as received (all axes / buttons / hats of both
 * devices) - the ground station's button-mapping routine reads it. */
static void send_joy_raw(link_t *l, const sb_t *s)
{
    uint8_t buf[2 + JSF_DEVS * (JSF_AXES * 2 + 5)];
    proto_wr_t w;
    const jsf_t *f = &s->jb.last;
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u8(&w, s->jb.have ? f->flags : 0u);
    proto_wr_u8(&w, f->seq);
    for (int d = 0; d < JSF_DEVS; d++) {
        for (int a = 0; a < JSF_AXES; a++) proto_wr_u16(&w, (uint16_t)f->dev[d].axis[a]);
        proto_wr_u32(&w, f->dev[d].buttons);
        proto_wr_u8(&w, f->dev[d].hats);
    }
    send(l, PROTO_T_JOY_RAW, l->tx_seq++, buf, w.n);
}

static void send_param(link_t *l, sb_t *s, uint8_t id)
{
    uint8_t buf[32];
    proto_wr_t w;
    char name[LINK_PARAM_NAME];
    memset(name, 0, sizeof name);
    strncpy(name, s_params[id].name, LINK_PARAM_NAME - 1u);
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u8(&w, id);
    proto_wr_f32(&w, *param_ptr(l, s, id));
    proto_wr_f32(&w, s_params[id].min);
    proto_wr_f32(&w, s_params[id].max);
    for (unsigned i = 0; i < LINK_PARAM_NAME; i++) proto_wr_u8(&w, (uint8_t)name[i]);
    send(l, PROTO_T_PARAM_VALUE, l->tx_seq++, buf, w.n);
}

static void send_prog_wp(link_t *l, const program_t *p, uint8_t i)
{
    uint8_t buf[32];
    proto_wr_t w;
    const prog_wp_t *wp = &p->wp[i];
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u8(&w, i);
    proto_wr_u8(&w, (uint8_t)p->n);
    proto_wr_u8(&w, p->final_action);
    proto_wr_i32(&w, wp->lat_e7);
    proto_wr_i32(&w, wp->lon_e7);
    proto_wr_f32(&w, wp->speed_kn);
    proto_wr_f32(&w, wp->radius_m);
    proto_wr_u8(&w, wp->action);
    send(l, PROTO_T_PROG_WP_VALUE, l->tx_seq++, buf, w.n);
}

/*
 * Telemetry payload (little-endian), groundstation/proto.py must match:
 *   u32 t_ms | u8 mode | u8 ap_mode | u8 sea_state | u8 flags
 *     flags: b0 compass ok, b1 GPS ok, b2 alarm output, b3 program loaded,
 *            b4 PC link was lost (still latched), b5 stopping
 *   f32 heading | f32 target | f32 yaw_rate | f32 outboard_est | f32 throttle_pct
 *   u8 gear (0 N, 1 F, 2 R) | u8 gps_fix
 *   i32 gps_lat | i32 gps_lon | f32 gps_sog | f32 gps_cog
 *   u8 pos_source (dr_state_t) | u8 dr_reason
 *   f32 unc_m | f32 dr_time_s | i32 est_lat | i32 est_lon | f32 est_sog | f32 est_cog
 *   f32 set_deg (current flows toward) | f32 drift_kn | f32 current_conf (0..1)
 *   u32 alarm_bits | f32 motor_current_a (-1 = no sensor)
 *   f32 joy_x | f32 joy_y | u8 joy_buttons (b0 TAKE, b1 AUTO, b2 CONFIRM)
 *   u8 joy_status (0 none, 1 not armed / not centred, 2 armed, 3 fault) | u8 joy_fault_axis
 *   u8 train_stage | u8 train_phase | u8 train_result
 *   f32 speed_set_kn (AUTO) | i32 wp_lat | i32 wp_lon | f32 wp_kn  (active waypoint)
 *   u16 rx_frames | u16 rx_crc_errors
 *   u8 prog_n | u8 auto_wp | u8 auto_state (auto_state_t) | u8 auto_flags (b0 program ok,
 *     b1 suspended, b2 finished, b3 loaded over the gateway link) | u8 confirm_kind (confirm_kind_t) | u16 confirm_left (0.1 s)
 *   f32 power_fwd_pct | f32 power_rev_pct | u16 power_code (0 = none) | u8 power_code_dir |
 *     u16 power_code_left (0.1 s) | u8 power_flags (b0 speed limited by the power limit,
 *     b1 limit raised) | f32 throttle_demand (before the limits)
 *   u16 core0 avg us | u16 core0 max us | u16 monitor avg | u16 monitor max | u16 logger avg |
 *     u16 logger max | u16 core0 overruns | u8 monitor overruns | u8 logger overruns   (us capped at 65535)
 *   u8 log_flags (b0 LOG FAILURE, b1 card present) | u16 log files | u16 log records dropped
 *   u8 joy_source (0 analog, 1 USB bridge) | u8 usb_flags (b0 stick connected, b1 throttle
 *     connected, b2 learn mode, b3 hub, b4 lever throttle, b5 pickup pending, b6 bridge starting)
 *     | u8 usb_frames_per_s | u16 usb_crc_errors | u8 joy_why (fault reason / current problem, JOY_WHY_*)
 *     | u16 lever x1000 (0xFFFF = no lever) | u16 pickup target x1000 (0xFFFF = neutral zone)
 *     | u32 usb buttons stick | u32 usb buttons throttle | u8 hats (low: stick hat, high: throttle hat)
 *     | u8 neutral zone low % | u8 neutral zone high %
 */
static void telemetry(link_t *l, sb_t *s)
{
    uint8_t buf[PROTO_MAX_PAYLOAD];
    proto_wr_t w;
    sb_status_t st = sb_status(s);
    const sb_auto_t *a = &s->autop;
    proto_wr_init(&w, buf, sizeof buf);

    uint8_t flags = 0u;
    if (sb_compass_ok(s)) flags |= 0x01u;
    if (sb_gps_ok(s)) flags |= 0x02u;
    if (s->alarm || s->ap.alarm || s->chirp_ms > 0u) flags |= 0x04u;
    if (a->prog_ok) flags |= 0x08u;
    if (s->alarm_bits & SB_ALM_PC_LINK) flags |= 0x10u;
    if (s->stopping) flags |= 0x20u;

    proto_wr_u32(&w, s->now_ms);
    proto_wr_u8(&w, (uint8_t)sb_mode(s));
    proto_wr_u8(&w, (uint8_t)s->ap.mode);
    proto_wr_u8(&w, s->ap.cfg.sea_state);
    proto_wr_u8(&w, flags);

    proto_wr_f32(&w, s->ap.heading);
    proto_wr_f32(&w, s->ap.target_heading);
    proto_wr_f32(&w, s->ap.yaw_rate);
    proto_wr_f32(&w, s->ap.est_angle);
    proto_wr_f32(&w, s->thr.out_pct);
    proto_wr_u8(&w, (uint8_t)s->gear.current);
    proto_wr_u8(&w, (uint8_t)(s->gps.valid && s->gps.last.fix && sb_gps_ok(s)));
    proto_wr_latlon(&w, s->gps.last.lat_deg);
    proto_wr_latlon(&w, s->gps.last.lon_deg);
    proto_wr_f32(&w, s->gps.last.sog_kn);
    proto_wr_f32(&w, s->gps.last.cog_deg);

    proto_wr_u8(&w, (uint8_t)st.pos_source);
    proto_wr_u8(&w, (uint8_t)st.dr_reason);
    proto_wr_f32(&w, st.unc_m);
    proto_wr_f32(&w, st.dr_time_s);
    proto_wr_latlon(&w, st.lat);
    proto_wr_latlon(&w, st.lon);
    proto_wr_f32(&w, st.sog_kn);
    proto_wr_f32(&w, st.cog_deg);

    float cn = s->dr.cur_n, ce = s->dr.cur_e;
    float set_deg = atan2f(ce, cn) * 180.0f / PI_F;
    if (set_deg < 0.0f) set_deg += 360.0f;
    float tau4 = 4.0f * s->dr.cfg.current_tau_s;
    float conf = (tau4 > 0.0f) ? s->dr.cur_learn_s / tau4 : 0.0f;
    proto_wr_f32(&w, set_deg);
    proto_wr_f32(&w, sqrtf(cn * cn + ce * ce) / 0.514444f);
    proto_wr_f32(&w, conf > 1.0f ? 1.0f : conf);

    float amps;
    proto_wr_u32(&w, s->alarm_bits);
    proto_wr_f32(&w, hal_read_motor_current(&amps) ? amps : -1.0f);

    const joy_t *j = &s->joy;
    proto_wr_f32(&w, j->present ? j->x : 0.0f);
    proto_wr_f32(&w, j->present ? j->y : 0.0f);
    proto_wr_u8(&w, (uint8_t)((j->take ? 1u : 0u) | (j->autob ? 2u : 0u) | (j->confirm ? 4u : 0u)));
    proto_wr_u8(&w, (uint8_t)(!j->present ? 0u : j->fault ? 3u : j->armed ? 2u : 1u));
    proto_wr_u8(&w, j->fault ? j->fault_axis : 0u);

    proto_wr_u8(&w, (uint8_t)s->train.stage);
    proto_wr_u8(&w, (uint8_t)s->ap.train.phase);
    proto_wr_u8(&w, (uint8_t)s->ap.train.result);

    proto_wr_f32(&w, s->cruise.target_kn);
    if (a->prog_ok && a->wp < a->prog.n) {
        proto_wr_i32(&w, a->prog.wp[a->wp].lat_e7);
        proto_wr_i32(&w, a->prog.wp[a->wp].lon_e7);
        proto_wr_f32(&w, a->prog.wp[a->wp].speed_kn);
    } else {
        proto_wr_i32(&w, 0);
        proto_wr_i32(&w, 0);
        proto_wr_f32(&w, 0.0f);
    }
    proto_wr_u16(&w, (uint16_t)(l->rx.frames & 0xFFFFu));
    proto_wr_u16(&w, (uint16_t)(l->rx.crc_errors & 0xFFFFu));

    proto_wr_u8(&w, a->prog_ok ? (uint8_t)a->prog.n : 0u);
    proto_wr_u8(&w, a->wp);
    proto_wr_u8(&w, (uint8_t)a->state);
    proto_wr_u8(&w, (uint8_t)((a->prog_ok ? 1u : 0u) | (a->suspended ? 2u : 0u) | (a->finished ? 4u : 0u) |
                              (a->prog_ok && a->via_gw ? 8u : 0u)));
    proto_wr_u8(&w, (uint8_t)s->confirm.kind);
    proto_wr_u16(&w, (uint16_t)(sb_confirm_left_s(s) * 10.0f + 0.5f));

    proto_wr_f32(&w, s->plim.fwd_pct);
    proto_wr_f32(&w, s->plim.rev_pct);
    proto_wr_u16(&w, s->plim.code);
    proto_wr_u8(&w, s->plim.code_dir);
    proto_wr_u16(&w, (uint16_t)(plim_code_left_s(&s->plim, s->now_ms) * 10.0f + 0.5f));
    proto_wr_u8(&w, (uint8_t)((s->speed_limited ? 1u : 0u) | (plim_raised(&s->plim) ? 2u : 0u)));
    proto_wr_f32(&w, s->throttle_demand);

    const cyc_t *cy[3] = { &g_timing.core0, &g_timing.monitor, &g_timing.logger };
    for (int i = 0; i < 3; i++) {
        proto_wr_u16(&w, (uint16_t)(cy[i]->pub_avg_us > 65535u ? 65535u : cy[i]->pub_avg_us));
        proto_wr_u16(&w, (uint16_t)(cy[i]->pub_max_us > 65535u ? 65535u : cy[i]->pub_max_us));
    }
    proto_wr_u16(&w, (uint16_t)(g_timing.core0.overruns > 65535u ? 65535u : g_timing.core0.overruns));
    proto_wr_u8(&w, (uint8_t)(g_timing.monitor.overruns > 255u ? 255u : g_timing.monitor.overruns));
    proto_wr_u8(&w, (uint8_t)(g_timing.logger.overruns > 255u ? 255u : g_timing.logger.overruns));
    proto_wr_u8(&w, (uint8_t)((g_logstat.failure ? 1u : 0u) | (g_logstat.card ? 2u : 0u)));
    proto_wr_u16(&w, (uint16_t)(g_logstat.files > 65535u ? 65535u : g_logstat.files));
    uint32_t drops = g_logq0.dropped + g_logq1.dropped;
    proto_wr_u16(&w, (uint16_t)(drops > 65535u ? 65535u : drops));

    const jsf_t *jf = &s->jb.last;
    bool usb = s->jmap.source == JOY_SRC_BRIDGE;
    proto_wr_u8(&w, s->jmap.source);
    proto_wr_u8(&w, (uint8_t)((usb && s->jb.have ? (jf->flags & 0x0Fu) : 0u) | (j->has_lever ? 0x10u : 0u) |
                              (s->lev.pickup ? 0x20u : 0u) | (usb && s->jb.grace ? 0x40u : 0u)));
    proto_wr_u8(&w, (uint8_t)(s->jb.fps > 255u ? 255u : s->jb.fps));
    proto_wr_u16(&w, (uint16_t)(s->jb.rx.crc_errors > 65535u ? 65535u : s->jb.rx.crc_errors));
    proto_wr_u8(&w, j->fault ? j->fault_axis : (usb ? s->jb.why : 0u));
    proto_wr_u16(&w, j->has_lever ? (uint16_t)(j->lever_in * 1000.0f + 0.5f) : 0xFFFFu);
    proto_wr_u16(&w, (s->lev.pickup && s->lev.target >= 0.0f) ? (uint16_t)(s->lev.target * 1000.0f + 0.5f) : 0xFFFFu);
    proto_wr_u32(&w, usb ? jf->dev[0].buttons : 0u);
    proto_wr_u32(&w, usb ? jf->dev[1].buttons : 0u);
    proto_wr_u8(&w, usb ? (uint8_t)((jf->dev[0].hats & 0x0Fu) | (uint8_t)(jf->dev[1].hats << 4)) : 0xFFu);
    proto_wr_u8(&w, (uint8_t)(s->jmap.nz_lo * 100.0f + 0.5f));
    proto_wr_u8(&w, (uint8_t)(s->jmap.nz_hi * 100.0f + 0.5f));

    l->telem_len = (uint16_t)w.n;
    if (!w.overflow) send(l, PROTO_T_TELEMETRY, l->tx_seq++, buf, w.n);
    else l->telem_overflows++;          /* never silently: counted (a test fails on it) */
}

/* --- commands ------------------------------------------------------------- */

/* Expected payload length per command, -1 = unknown command. */
static int cmd_len(uint8_t type)
{
    switch (type) {
    case PROTO_T_HEARTBEAT:    return 0;
    case PROTO_T_SET_MODE:     return 1;
    case PROTO_T_HEADING_ADJ:  return 1;
    case PROTO_T_SEA_STATE:    return 1;
    case PROTO_T_TRAINING:     return 5;
    case PROTO_T_PARAM_GET:    return 1;
    case PROTO_T_PARAM_SET:    return 5;
    case PROTO_T_JOY_CAL:      return 1;
    case PROTO_T_ALARM_ACK:    return 0;
    case PROTO_T_JOYMAP_GET:   return 0;
    case PROTO_T_JOYMAP_SET:   return (int)JOYMAP_BODY + 1;
    case PROTO_T_JOY_RAW_GET:  return 0;
    case PROTO_T_PROG_BEGIN:   return 2;
    case PROTO_T_PROG_WP:      return 18;
    case PROTO_T_PROG_COMMIT:  return 0;
    case PROTO_T_PROG_GET:     return 1;
    case PROTO_T_AUTO_START:   return 0;
    case PROTO_T_AUTO_PAUSE:   return 0;
    case PROTO_T_AUTO_RESUME:  return 0;
    case PROTO_T_AUTO_SKIP:    return 0;
    case PROTO_T_POWER_CODE:   return 1;
    case PROTO_T_POWER_RAISE:  return 3;
    case PROTO_T_POWER_LOWER:  return 0;
    case PROTO_T_BENCH:        return 3;
    case PROTO_T_GENERATOR:    return 1;
    case PROTO_T_RELAY:        return 4;
    case PROTO_T_RELAY_CFG:    return -2;       /* variable: checked by relay_table_decode */
    case PROTO_T_THRUSTER_CFG: return (int)THR_FRAME_BYTES;
    case PROTO_T_RESTART:      return 1;
    case PROTO_T_DP:           return 4;
    default:                   return -1;
    }
}

static uint8_t sb_result_to_proto(sb_result_t r)
{
    switch (r) {
    case SB_OK:                return PROTO_R_OK;
    case SB_ERR_JOYSTICK:      return PROTO_R_JOYSTICK;
    case SB_ERR_JOY_NOT_READY: return PROTO_R_JOY_NOT_READY;
    case SB_ERR_NO_PROGRAM:    return PROTO_R_NO_PROGRAM;
    case SB_ERR_PRECOND:       return PROTO_R_PRECOND;
    case SB_ERR_WRONG_MODE:    return PROTO_R_WRONG_MODE;
    case SB_ERR_BAD_PROGRAM:   return PROTO_R_BAD_PROGRAM;
    case SB_ERR_WRONG_CODE:    return PROTO_R_WRONG_CODE;
    case SB_ERR_AT_MAX:        return PROTO_R_AT_MAX;
    case SB_ERR_ALARM:         return PROTO_R_ALARM_ACTIVE;
    case SB_ERR_NOT_CENTERED:  return PROTO_R_NOT_CENTERED;
    case SB_ERR_STANDBY_ONLY:  return PROTO_R_STANDBY_ONLY;
    case SB_ERR_BAD_VALUE:     return PROTO_R_BAD_VALUE;
    case SB_ERR_NOT_ALLOWED:   return PROTO_R_NOT_ALLOWED;
    default:                   return PROTO_R_REFUSED;
    }
}

/* Mode change through the mode manager; the result says why if refused.
 * The PC may only ask for STANDBY and TRAINING. */
static uint8_t pc_mode(sb_t *s, sb_mode_t to)
{
    if (sb_mode(s) == MODE_MANUAL) {
        (void)sb_request_mode(s, to, WHY_PC);                    /* refused + logged */
        return PROTO_R_JOYSTICK;
    }
    if (to != MODE_STANDBY && to != MODE_TRAINING) {
        (void)sb_request_mode(s, to, WHY_PC);                    /* refused + logged */
        return PROTO_R_NOT_ALLOWED;
    }
    if (sb_mode(s) == to) return PROTO_R_OK;                    /* already there */
    if (to != MODE_STANDBY && s->joy.off_centre) {
        (void)sb_request_mode(s, to, WHY_PC);                    /* refused + logged */
        return PROTO_R_NOT_CENTERED;                             /* centre the joystick */
    }
    if (sb_request_mode(s, to, WHY_PC)) return PROTO_R_OK;
    const mode_log_entry_t *e = mode_log_get(&s->modes, mode_log_size(&s->modes) - 1u);
    return (e && e->verdict == MODE_REJECT_TABLE) ? PROTO_R_NOT_ALLOWED : PROTO_R_PRECOND;
}

static uint8_t do_command(link_t *l, sb_t *s, const proto_frame_t *f)
{
    proto_rd_t r;
    proto_rd_init(&r, f->payload, f->len);
    bool joy = (sb_mode(s) == MODE_MANUAL);

    switch (f->type) {
    case PROTO_T_GENERATOR: {
        /* generator auto on / off (section K): the HELM station (checked by the
         * caller); the generator itself runs only in STANDBY with neutral confirmed */
        uint8_t on = proto_rd_u8(&r);
        if (on > 1u) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_generator_set(s, on == 1u));
    }
    case PROTO_T_RELAY_CFG: {
        /* the boat file's relay table (section M): HELM station (checked by the
         * caller); a table that differs from the stored one waits for the CONFIRM */
        static relay_table_t t;                 /* not on the stack (140 bytes) */
        if (!relay_table_decode(f->payload, f->len, &t))
            return (f->len < 1u || f->len != 1u + (uint32_t)f->payload[0] * RELAY_FRAME_BYTES) ? PROTO_R_BAD_LENGTH
                                                                                             : PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_relay_table(s, &t));
    }
    case PROTO_T_THRUSTER_CFG: {
        /* the boat file's DP thruster settings (section P): HELM station (checked by
         * the caller), STANDBY only, RAM only; the hash goes back in TELEM2 group N */
        thr_cfg_t c;
        if (!thruster_cfg_decode(f->payload, f->len, &c)) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_thruster_cfg(s, &c, proto_crc16(f->payload, f->len)));
    }
    case PROTO_T_RESTART: {
        /* remote restart / hold-to-crank (section N): the HELM station (checked by
         * the caller, and again by sb_restart); the conditions are on the AURIX */
        uint8_t act = proto_rd_u8(&r);
        if (act > ENG_ACT_RELEASE) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_restart(s, s->helm, act));
    }
    case PROTO_T_DP: {
        /* DP (section P): the HELM station (checked by the caller, and again by sb_dp_cmd) */
        uint8_t act = proto_rd_u8(&r), arg = proto_rd_u8(&r);
        int16_t v = (int16_t)proto_rd_u16(&r);
        if (joy && act != DP_ACT_STOP) return PROTO_R_JOYSTICK;
        return sb_result_to_proto(sb_dp_cmd(s, s->helm, act, arg, v));
    }
    case PROTO_T_RELAY: {
        uint8_t n = proto_rd_u8(&r), act = proto_rd_u8(&r);
        uint16_t ms = proto_rd_u16(&r);
        if (n == 0u || n > RELAY_N_MAX || act > RELAY_CMD_PULSE) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_relay_cmd(s, n, act, ms));
    }
    case PROTO_T_SET_MODE: {
        uint8_t m = proto_rd_u8(&r);
        if (m >= (uint8_t)MODE_COUNT) return PROTO_R_BAD_VALUE;
        return pc_mode(s, (sb_mode_t)m);
    }
    case PROTO_T_HEADING_ADJ: {
        int8_t d = proto_rd_i8(&r);
        uint32_t bit;
        if (d == -1) bit = HAL_BTN_BIT(HAL_BTN_MINUS_1);
        else if (d == 1) bit = HAL_BTN_BIT(HAL_BTN_PLUS_1);
        else if (d == -10) bit = HAL_BTN_BIT(HAL_BTN_MINUS_10);
        else if (d == 10) bit = HAL_BTN_BIT(HAL_BTN_PLUS_10);
        else return PROTO_R_BAD_VALUE;
        if (joy) return PROTO_R_JOYSTICK;
        if (sb_mode(s) == MODE_DP)                                     /* DP: the held heading (also UNCREWED) */
            return sb_result_to_proto(sb_dp_cmd(s, s->helm, DP_ACT_HEADING, 1u, (int16_t)(d * 10)));
        if (sb_mode(s) != MODE_HDG_LOCK) return PROTO_R_WRONG_MODE;   /* same as the buttons */
        s->ext_buttons |= bit;
        return PROTO_R_OK;
    }
    case PROTO_T_SEA_STATE: {
        uint8_t ss = proto_rd_u8(&r);
        if (ss < AP_SEA_STATE_MIN || ss > AP_SEA_STATE_MAX) return PROTO_R_BAD_VALUE;
        autopilot_set_sea_state(&s->ap, ss);
        return PROTO_R_OK;
    }
    case PROTO_T_TRAINING: {
        uint8_t act = proto_rd_u8(&r);
        float full = proto_rd_f32(&r);
        if (act == 0u) return pc_mode(s, MODE_TRAINING);
        if (act > 2u || !(full >= 0.0f && full <= 180.0f)) return PROTO_R_BAD_VALUE;
        if (sb_mode(s) != MODE_TRAINING || s->train.stage != TS_RESULT) return PROTO_R_WRONG_MODE;
        if (act == 2u) {
            (void)sb_confirm_training(s, false, 0.0f);
            return PROTO_R_OK;
        }
        return sb_confirm_training(s, true, full) ? PROTO_R_OK : PROTO_R_REFUSED;
    }
    case PROTO_T_PARAM_GET: {
        uint8_t id = proto_rd_u8(&r);
        if (id >= P_COUNT) return PROTO_R_BAD_PARAM;
        send_param(l, s, id);
        return PROTO_R_OK;
    }
    case PROTO_T_PARAM_SET: {
        uint8_t id = proto_rd_u8(&r);
        float v = proto_rd_f32(&r);
        if (id >= P_COUNT) return PROTO_R_BAD_PARAM;
        if (sb_mode(s) != MODE_STANDBY) return PROTO_R_STANDBY_ONLY;
        if (!(v >= s_params[id].min && v <= s_params[id].max)) return PROTO_R_BAD_VALUE;
        logcore0_param(s, id, *param_ptr(l, s, id), v);
        *param_ptr(l, s, id) = v;
        s->cfg_version++;
        if (id == P_CRUISE_MAX) s->cfg.prog.max_speed_kn = v;   /* program validation uses it too */
        send_param(l, s, id);
        return PROTO_R_OK;
    }
    case PROTO_T_JOY_CAL: {
        uint8_t act = proto_rd_u8(&r);
        if (act > 1u) return PROTO_R_BAD_VALUE;
        if (sb_mode(s) != MODE_STANDBY) return PROTO_R_STANDBY_ONLY;
        bool ok = (act == 0u) ? sb_joystick_cal_start(s) : sb_joystick_cal_finish(s);
        return ok ? PROTO_R_OK : PROTO_R_REFUSED;
    }
    case PROTO_T_JOYMAP_GET:
        send_joymap(l, s);
        return PROTO_R_OK;
    case PROTO_T_JOYMAP_SET: {
        joymap_t m;
        jm_error_t e;
        rd_joymap(&r, &m);
        uint8_t save = proto_rd_u8(&r);
        if (save > 1u) return PROTO_R_BAD_VALUE;
        if (sb_mode(s) != MODE_STANDBY) return PROTO_R_STANDBY_ONLY;
        bool ok = sb_joymap_set(s, &m, save != 0u, &e);
        send_joymap(l, s);
        if (e == JM_ERR_DUPLICATE) return PROTO_R_MAP_DUPLICATE;
        if (e == JM_ERR_MISSING) return PROTO_R_MAP_MISSING;
        if (e != JM_OK) return PROTO_R_MAP_BAD;
        return ok ? PROTO_R_OK : PROTO_R_REFUSED;
    }
    case PROTO_T_JOY_RAW_GET:
        send_joy_raw(l, s);
        return PROTO_R_OK;
    case PROTO_T_ALARM_ACK:             /* any mode (the panel STANDBY button: in STANDBY) */
        sb_alarm_ack(s);                /* only alarms whose cause is gone - also from a remote station */
        return PROTO_R_OK;

    /* --- AUTO program upload --- */
    case PROTO_T_PROG_BEGIN: {
        uint8_t n = proto_rd_u8(&r), fin = proto_rd_u8(&r);
        if (n == 0u || n > PROG_MAX_WP || fin > (uint8_t)PROG_END_HOLD) return PROTO_R_BAD_VALUE;
        program_clear(&l->staging);
        l->staging.n = n;
        l->staging.final_action = fin;
        l->staging_n = n;
        l->staging_have = 0u;
        return PROTO_R_OK;
    }
    case PROTO_T_PROG_WP: {
        uint8_t i = proto_rd_u8(&r);
        if (l->staging_n == 0u || i >= l->staging_n) return PROTO_R_BAD_VALUE;
        prog_wp_t *wp = &l->staging.wp[i];
        wp->lat_e7 = proto_rd_i32(&r);
        wp->lon_e7 = proto_rd_i32(&r);
        wp->speed_kn = proto_rd_f32(&r);
        wp->radius_m = proto_rd_f32(&r);
        wp->action = proto_rd_u8(&r);
        l->staging_have |= (uint64_t)1u << i;
        return PROTO_R_OK;
    }
    case PROTO_T_PROG_COMMIT: {
        if (l->staging_n == 0u) return PROTO_R_BAD_VALUE;
        uint64_t all = (l->staging_n >= 64u) ? ~(uint64_t)0u : (((uint64_t)1u << l->staging_n) - 1u);
        prog_error_t err = PROG_OK;
        uint8_t bad = 0xFFu;
        sb_result_t res;
        if ((l->staging_have & all) != all) {
            err = PROG_BAD_COUNT;               /* waypoints missing */
            res = SB_ERR_BAD_PROGRAM;
        } else {
            program_seal(&l->staging);
            res = sb_program_set(s, &l->staging, &err, &bad);
        }
        if (res == SB_ERR_BAD_PROGRAM) {
            uint8_t p[2] = { (uint8_t)err, bad };
            send(l, PROTO_T_PROG_ERROR, f->seq, p, sizeof p);
        }
        if (res == SB_OK) {
            l->staging_n = 0u;
            s->autop.via_gw = l->port == LINK_PORT_GW;     /* the gateway checked this route (TELEM auto_flags b3) */
        }
        return sb_result_to_proto(res);
    }
    case PROTO_T_PROG_GET: {
        uint8_t i = proto_rd_u8(&r);
        if (!s->autop.prog_ok) return PROTO_R_NO_PROGRAM;
        if (i >= s->autop.prog.n) return PROTO_R_BAD_VALUE;
        send_prog_wp(l, &s->autop.prog, i);
        return PROTO_R_OK;
    }

    /* --- AUTO control --- */
    case PROTO_T_AUTO_START:
        return sb_result_to_proto(sb_auto_start(s));
    case PROTO_T_AUTO_RESUME:
        return sb_result_to_proto(sb_auto_resume(s));
    case PROTO_T_AUTO_PAUSE:
        if (joy) return PROTO_R_JOYSTICK;
        return sb_result_to_proto(sb_auto_pause(s));
    case PROTO_T_AUTO_SKIP:
        return sb_result_to_proto(sb_auto_skip(s));

    /* --- power limit --- */
    case PROTO_T_POWER_CODE: {
        uint8_t dir = proto_rd_u8(&r);
        uint16_t code;
        if (dir > 1u) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_power_code(s, dir, &code));
    }
    case PROTO_T_POWER_RAISE: {
        uint8_t dir = proto_rd_u8(&r);
        uint16_t code = (uint16_t)(proto_rd_u8(&r) | ((uint16_t)proto_rd_u8(&r) << 8));
        if (dir > 1u) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_power_raise(s, dir, code));
    }
    case PROTO_T_POWER_LOWER:
        sb_power_lower(s);
        return PROTO_R_OK;
    case PROTO_T_BENCH: {
        uint8_t act = proto_rd_u8(&r);
        uint16_t arg = proto_rd_u16(&r);
        if (act > SB_BENCH_TRIP) return PROTO_R_BAD_VALUE;
        return sb_result_to_proto(sb_bench(s, act, arg));
    }
    default:
        return PROTO_R_UNKNOWN;
    }
}

static void gw_status(sb_t *s, const proto_frame_t *f);

/* GW_RJOY, 15 bytes: u8 station | u16 seq | u16 data age ms | u16 one-way latency ms |
 * i16 x*1000 | i16 y*1000 | u16 lever*1000 | u8 reserved | u8 flags (b0 lever, b1 release,
 * b2 CONFIRM held - counts only in UNCREWED) */
static void gw_rjoy(sb_t *s, const proto_frame_t *f)
{
    const uint8_t *p = f->payload;
    uint8_t st = p[0];
    uint16_t seq = (uint16_t)(p[1] | (p[2] << 8)), age = (uint16_t)(p[3] | (p[4] << 8));
    uint16_t lat = (uint16_t)(p[5] | (p[6] << 8));
    int16_t x = (int16_t)(p[7] | (p[8] << 8)), y = (int16_t)(p[9] | (p[10] << 8));
    uint16_t lev = (uint16_t)(p[11] | (p[12] << 8));
    uint8_t flags = p[14];
    if (flags & 2u) {
        (void)sb_remote_release(s, st);
        return;
    }
    /* every packet of the HELM station: its CONFIRM button counts in UNCREWED */
    sb_shore_joy(s, st, age, x * 0.001f, y * 0.001f, lev * 0.001f, (flags & 1u) != 0u, (flags & 4u) != 0u);
    sb_remote_input(s, st, seq, age, lat, x * 0.001f, y * 0.001f, lev * 0.001f, (flags & 1u) != 0u);
}

/* --- HELM (section B): only the HELM station's commands are accepted ------ */

/* Commands every station may send without HELM: they change nothing. */
/* GW_NAVREQ (2 Hz while it matters), 10 bytes:
 *   u8 seq | u8 kind (SB_NAV_) | u8 reason (1 collision, 2 shallow, 4 decision) |
 *   u8 alarm (1 collision, 2 driver decision, 4 shallow) | u16 heading*100 |
 *   u16 speed cap kn*10 (0xFFFF none) | u16 lease ms
 * Checked here; applied by smallboat.c only in AUTO / after ACCEPT in HDG LOCK. */
static void gw_navreq(sb_t *s, const proto_frame_t *f)
{
    proto_rd_t r;
    proto_rd_init(&r, f->payload, f->len);
    uint8_t seq = proto_rd_u8(&r), kind = proto_rd_u8(&r), reason = proto_rd_u8(&r), alarm = proto_rd_u8(&r);
    uint16_t hdg = proto_rd_u16(&r), cap = proto_rd_u16(&r), lease = proto_rd_u16(&r);
    if (kind > SB_NAV_HDG_SET || (kind != SB_NAV_NONE && kind != SB_NAV_CAP && hdg >= 36000u) ||
        (kind == SB_NAV_HEADING && cap == 0xFFFFu) ||               /* an avoidance heading always slows too */
        (cap != 0xFFFFu && cap > 600u) || lease < 500u || lease > 10000u) {
        s->nav.rejected++;
        return;                                                  /* not valid: ignored */
    }
    sb_navreq_t *n = &s->nav;
    n->seq = seq;
    n->kind = kind;
    n->reason = reason;
    n->alarm = alarm;
    n->heading = (float)hdg * 0.01f;
    n->cap_kn = (cap == 0xFFFFu) ? -1.0f : (float)cap * 0.1f;
    n->lease_ms = lease;
    n->rx_ms = s->now_ms ? s->now_ms : 1u;
    n->rx_mode = (uint8_t)sb_mode(s);                            /* HDG_SET: only if it came in HDG LOCK */
    n->count++;
}

static bool read_only(uint8_t type)
{
    return type == PROTO_T_PARAM_GET || type == PROTO_T_PROG_GET || type == PROTO_T_JOYMAP_GET ||
           type == PROTO_T_JOY_RAW_GET;
}

static void helm_set(sb_t *s, uint8_t holder, bool local)
{
    s->helm = holder;
    s->helm_local = local;
    s->helm_changes++;
}

/* HELM take (0) / release (1) / hand over (2, payload u8 2, u8 station, u8
 * local). A local station (USB laptop, boat LAN) can always take HELM - also
 * from a remote one; a remote station only when nobody holds it. Hand over:
 * only the holder, to another gateway station, in one step (the gateway's
 * captain / copilot role swap: no moment without HELM = no lost link). */
static uint8_t helm_cmd(sb_t *s, uint8_t station, bool local, uint8_t action, uint8_t to, bool to_local)
{
    if (action == 2u) {
        if (s->helm != station) return PROTO_R_NOT_HELM;
        if (to < 2u || to == 0xFFu || station < 2u) return PROTO_R_BAD_VALUE;
        helm_set(s, to, to_local);
        logcore0_helm(s->now_ms, station, 5u, 0u, to);
        return PROTO_R_OK;
    }
    if (action == 0u) {
        if (s->helm != station && s->helm != SB_HELM_NONE && !local) {
            logcore0_helm(s->now_ms, station, 0u, 1u, s->helm);
            return PROTO_R_HELM_TAKEN;
        }
        if (s->helm != station) {
            helm_set(s, station, local);
            if (local) s->alarm_bits &= ~SB_ALM_REMOTE_LINK;
        }
        logcore0_helm(s->now_ms, station, 0u, 0u, station);
        return PROTO_R_OK;
    }
    if (action == 1u) {
        if (s->helm != station) return PROTO_R_NOT_HELM;
        helm_set(s, SB_HELM_NONE, false);
        logcore0_helm(s->now_ms, station, 1u, 0u, SB_HELM_NONE);
        return PROTO_R_OK;
    }
    return PROTO_R_BAD_VALUE;
}

/* The HELM station is gone (left / link lost / gateway lost): HELM back to
 * nobody; a remote one also gives REMOTE LINK LOST. No mode change. */
static void helm_lost(sb_t *s, uint8_t station, uint8_t action)
{
    if (s->helm != station || station == SB_HELM_NONE) return;
    bool remote = !s->helm_local;
    helm_set(s, SB_HELM_NONE, false);
    logcore0_helm(s->now_ms, station, action, 0u, SB_HELM_NONE);
    if (remote) {
        s->alarm = true;
        s->alarm_bits |= SB_ALM_REMOTE_LINK;
    }
}

/* One command from a station (USB laptop = SB_HELM_USB, or through the
 * gateway); answered with ACK (USB) or GW_ACK (gateway). */
static void command(link_t *l, sb_t *s, const proto_frame_t *f, uint8_t station, bool local, uint16_t gw_seq)
{
    uint8_t r;
    int want = cmd_len(f->type);
    l->commands++;
    if (f->type >= PROTO_T_SIM_FIRST && f->type <= PROTO_T_SIM_LAST) {
        r = l->sim_fn ? l->sim_fn(l->sim_ctx, f) : PROTO_R_UNKNOWN;
    } else if (f->type == PROTO_T_HELM) {
        if (f->len == 1u && f->payload[0] != 2u) r = helm_cmd(s, station, local, f->payload[0], 0u, false);
        else if (f->len == 3u && f->payload[0] == 2u)
            r = helm_cmd(s, station, local, 2u, f->payload[1], f->payload[2] != 0u);
        else r = PROTO_R_BAD_LENGTH;
    } else if (f->type == PROTO_T_REMOTE) {
        /* remote driving: only a gateway station, only the HELM station */
        if (f->len != 1u || f->payload[0] > 1u) r = PROTO_R_BAD_VALUE;
        else if (station < 2u) r = PROTO_R_NOT_ALLOWED;
        else if (s->helm != station) r = PROTO_R_NOT_HELM;
        else r = sb_result_to_proto(f->payload[0] == 0u ? sb_remote_request(s, station)
                                                          : sb_remote_release(s, station));
    } else if (f->type == PROTO_T_MANEUVER) {
        /* MANEUVER: like remote driving (the gateway lets only a licensed captain ask) */
        if (f->len != 1u || f->payload[0] > 1u) r = PROTO_R_BAD_VALUE;
        else if (station < 2u) r = PROTO_R_NOT_ALLOWED;
        else if (s->helm != station) r = PROTO_R_NOT_HELM;
        else r = sb_result_to_proto(f->payload[0] == 0u ? sb_maneuver_request(s, station)
                                                          : sb_remote_release(s, station));
    } else if (f->type == PROTO_T_BENCH && station != SB_HELM_USB) {
        r = PROTO_R_NOT_ALLOWED;                         /* BENCH TEST: only the laptop on the USB cable */
    } else if (want == -1) {
        r = PROTO_R_UNKNOWN;
    } else if (want >= 0 && (int)f->len != want) {
        r = PROTO_R_BAD_LENGTH;
    } else {                                                /* want == -2: variable length, checked inside */
        if (!read_only(f->type) && s->helm == SB_HELM_NONE && station == SB_HELM_USB) {
            helm_set(s, SB_HELM_USB, true);             /* the USB laptop takes a free HELM by itself */
            logcore0_helm(s->now_ms, station, 3u, 0u, station);
        }
        r = (read_only(f->type) || s->helm == station) ? do_command(l, s, f) : PROTO_R_NOT_HELM;
    }
    if (l->port == LINK_PORT_GW) {
        uint8_t p[5] = { station, (uint8_t)gw_seq, (uint8_t)(gw_seq >> 8), f->type, r };
        l->last_result = r;
        if (r != PROTO_R_OK) l->nacks++;
        send(l, PROTO_T_GW_ACK, l->tx_seq++, p, sizeof p);
    } else {
        send_ack(l, f, r);
    }
    logcore0_pc(s->now_ms, f->type, f->seq, r, f->payload, f->len, station);    /* every command + answer */
}

static void handle(link_t *l, sb_t *s, const proto_frame_t *f)
{
    l->last_rx_ms = s->now_ms;
    uint32_t lost_bit = (l->port == LINK_PORT_GW) ? SB_ALM_GATEWAY : SB_ALM_PC_LINK;
    if (!l->connected || l->lost) {
        /* PC / gateway (back) online: its own alarm cause is gone */
        l->connected = true;
        l->lost = false;
        s->alarm_bits &= ~lost_bit;
        if (s->alarm_bits == 0u && !s->ap.alarm) s->alarm = false;
        if (l->port == LINK_PORT_GW) {
            s->gw.state = SB_GW_OK;
            g_logstat.gw_resend = 1u;               /* the Pi's copy restarts with a header */
        }
    }
    if (f->type == PROTO_T_HEARTBEAT && f->len == 0u) return;   /* no ACK for heartbeats */
    if (l->port != LINK_PORT_GW) {
        command(l, s, f, SB_HELM_USB, true, 0u);        /* the USB laptop: a local station */
        return;
    }
    /* gateway port: everything comes from the Pi */
    if (f->type == PROTO_T_GW_STATUS) {                          /* its heartbeat: no ACK */
        if (f->len == 12u) gw_status(s, f);
        return;
    }
    if (f->type == PROTO_T_GW_RJOY) {                            /* remote joystick, 30 Hz */
        if (f->len == 15u) gw_rjoy(s, f);
        return;
    }
    if (f->type == PROTO_T_GW_GPS2 && f->len >= 10u) {           /* second GPS (AIS device) */
        proto_rd_t r;
        proto_rd_init(&r, f->payload, f->len);
        double lat = (double)(int32_t)proto_rd_u32(&r) * 1e-7;
        double lon = (double)(int32_t)proto_rd_u32(&r) * 1e-7;
        uint16_t age = proto_rd_u16(&r);
        if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
            s->gps2.lat = lat;
            s->gps2.lon = lon;
            s->gps2.age_ms = age;
            s->gps2.rx_ms = s->now_ms ? s->now_ms : 1u;
            s->gps2.count++;
        } else {
            s->gps2.bad++;                                       /* impossible position: ignored */
        }
        return;
    }
    if (f->type == PROTO_T_GW_SHORE && f->len >= 3u) {           /* distance to the shore (lost-link rule) */
        sb_gw_shore(s, f->payload[0], (uint16_t)(f->payload[1] | (f->payload[2] << 8)));
        return;
    }
    if (f->type == PROTO_T_GW_NAVREQ && f->len >= 10u) {         /* collision avoidance / shallow water */
        gw_navreq(s, f);
        return;
    }
    if (f->type == PROTO_T_GW_STATION) {                         /* a station joined / left */
        if (f->len == 3u && f->payload[1] == 1u) helm_lost(s, f->payload[0], 2u);
        return;
    }
    if (f->type == PROTO_T_GW_CMD && f->len >= 5u && f->payload[0] >= 2u && f->payload[0] != 0xFFu) {
        /* u8 station | u8 flags (b0 local) | u16 command seq | u8 type | payload */
        proto_frame_t in;
        in.type = f->payload[4];
        in.seq = f->payload[2];
        in.len = (uint8_t)(f->len - 5u);
        memcpy(in.payload, &f->payload[5], in.len);
        command(l, s, &in, f->payload[0], (f->payload[1] & 1u) != 0u,
                (uint16_t)(f->payload[2] | (f->payload[3] << 8)));
        return;
    }
    /* anything else from the Pi without an envelope is refused */
    send_ack(l, f, PROTO_R_UNKNOWN);
}
/* --- gateway (Raspberry Pi) ---------------------------------------------------- */

/*
 * TELEM2 (both ports, with every telemetry frame), groundstation/proto.py
 * must match. Groups are only ever appended:
 *   A: u32 t_ms | u8 gw_state (0 never, 1 ok, 2 LOST) | u8 gw_flags | u8 gw_channel |
 *      u8 gw_stations | u32 log_sent_seq | u32 log_dropped_for_pi | u32 pi_log_seq |
 *      u16 pi_log_gaps | u32 log_skipped (no gateway)
 *   B: u8 helm (0 nobody, 1 USB laptop, 2.. gateway station) | u8 helm_flags (b0 local) |
 *      u16 helm_changes
 *   D: u8 remote_state (0 off, 1 driving, 2 LINK LOST) | u8 remote_station | u16 data age ms |
 *      u16 latency ms | i16 x*1000 | u16 lever*1000 | u8 flags (b0 lever, b1 pickup pending, b2 confirm
 *      pending) | u16 pickup target*1000 (0xFFFF neutral zone) | u8 cap fwd % | u8 cap rev % |
 *      u32 packets | u8 lost_why
 *   E: u16 save_seq (video SAVE requests, 0 none) | u8 trigger (SB_SAVE_) | u8 arg | u32 age ms |
 *      u8 kinds (bit per trigger of every request in the 2 s before the latest)
 *   F: u16 slam peak since the last telemetry g*100 | u16 level g*100 | u16 impacts / min |
 *      u8 forward cap % (100 = none) | u8 state (0 calm 1 yellow 2 red) | b4 override | b5 sensor lost |
 *      b6 sensor seen) | u16 limit g*100 | u32 impacts | u8 yellow part of the limit %
 *   G: navigation request (below)
 *   H: i16 roll deg*100 | i16 pitch deg*100 | u16 age ms (0xFFFF never) - BNO085 attitude for the
 *      camera horizon (gateway vision), never used for control
 *   I: u8 config flags (b0 FIRST TEST: steering only, b1 NV DATA RESET, b2 NV FAULT, b3 NV save
 *      pending) | u8 e-stop (0 not fitted, 1 released,
 *      2 pressed) | u8 moving ahead (FIRST TEST: steering allowed)
 *   J: u8 profile (0 detecting, 1 CREWED, 2 UNCREWED) | u8 onboard joystick (b0 detected + self-check OK,
 *      b1 present, b2 fault, b3 starting) | u8 shore joystick (b0 fresh from HELM, b1 centred, b2 CONFIRM held) |
 *      u16 profile changes
 *   K: u8 lost-link state (0 ok, 1 LOST, 2 ENGINE KILLED) | u8 flags (b0 armed: a HELM station held HELM,
 *      b1 near shore now, b2 the kill was near shore; b4.. GW_SHORE flags) | u16 shore distance m (0xFFFF unknown) |
 *      u16 shore info age ms (0xFFFF never) | u16 open-sea kill in (0.1 s) | u16 near_shore_m
 *   L: u32 alarm_bits2 (SB_ALM2_: the second alarm word) | u16 bank V*100 (0xFFFF no data) |
 *      u16 fast-filtered bank V*100 (0xFFFF) | u8 battery flags (b0 measured now, b1 NO VOLTAGE DATA
 *      notice, b2 LOW BATTERY condition) | u8 generator state (GEN_: 0 off 1 ready 2 starting 3 running
 *      4 stopping 5 rest 6 fault 7 done; section K) | u8 generator throttle demand %
 *   M: relay board (below)
 *   O: engine controls (section N; DP's group N follows it): u8 crank state (0 off, 1 cranking,
 *      2 cool-down) | u8 flags (b0 restart conditions met now: idle + neutral confirmed, e-stop
 *      released, not FIRST TEST; b1 software reset pending; b2 a STARTER in the relay table; b3 ENGINE
 *      KILL latched; b4 monitor tripped) | u16 hold ms | u16 cool-down left (0.1 s) | u8 last action
 *      (ENG_LOG_) | u8 last why (eng_why_t) | u16 restarts | u16 cranks
 *   N: DP thrusters (section P; step 10 appends the DP state): u8 flags (thruster_flags: b0 settings
 *      received, b1 active, b2 bow HOT, b3 stern HOT, b4 bow fitted, b5 stern fitted) | i8 bow output % |
 *      i8 stern output % | u8 bow heat budget used % | u8 stern heat budget used % | u8 bow max % |
 *      u8 stern max % | u16 settings hash (CRC-16 of the THRUSTER_CFG body, 0 none) | u8 why (THR_WHY_) |
 *      DP (step 10): u8 sub (dp.h DP_: 0 off 1 SPOT 2 JOG 3 CRUISE 4 TRIP) | u8 flags (b0 OFF SPOT, b1 OVERPOWER,
 *      b2 compass lost: gyro hold, b3 TRIP done, b4 a DP request waits for the CONFIRM, b5 generator check
 *      "stern fights") | i16 spot error ahead dm | i16 spot error to starboard dm | u16 heading target*10 |
 *      u8 set speed kn*10 | u8 TRIP speed now kn*10 | u8 TRIP waypoint | u8 last stop why (DP_STOP_)
 *   P: DroneCAN summary (step 11d; the full node table: CAN_NODES on the USB port): u8 flags (b0 a CAN bus
 *      fitted, b1 bus fault, b2 ID conflict (our node ID 10 seen), b3 a node without a static ID (notice),
 *      b4 bus silent, b5 STANDBY interlock: a required node lost / not seen yet) | u8 bus state (HAL_CAN_ST_) |
 *      u8 lost mask | u8 WARNING mask (notice) | u8 seen mask | u8 rebooted mask (bit = node table index:
 *      0 throttle 1 steering 2 gear 3 relay board)
 */
static void telemetry2(link_t *l, sb_t *s)
{
    uint8_t buf[PROTO_MAX_PAYLOAD];
    proto_wr_t w;
    const sb_gw_t *g = &s->gw;
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u32(&w, s->now_ms);
    proto_wr_u8(&w, g->state);
    proto_wr_u8(&w, g->flags);
    proto_wr_u8(&w, g->channel);
    proto_wr_u8(&w, g->stations);
    proto_wr_u32(&w, g->log_sent_seq);
    proto_wr_u32(&w, g_logq_gw.dropped);
    proto_wr_u32(&w, g->pi_log_seq);
    proto_wr_u16(&w, g->pi_log_gaps);
    proto_wr_u32(&w, g->log_skipped);
    proto_wr_u8(&w, s->helm);
    proto_wr_u8(&w, s->helm_local ? 1u : 0u);
    proto_wr_u16(&w, s->helm_changes);
    const sb_remote_t *r = &s->remote;
    proto_wr_u8(&w, (sb_mode(s) == MODE_REMOTE || sb_mode(s) == MODE_MANEUVER) ? r->state : SB_REMOTE_OFF);
    proto_wr_u8(&w, r->station);
    proto_wr_u16(&w, r->age_ms);
    proto_wr_u16(&w, r->lat_ms);
    proto_wr_u16(&w, (uint16_t)(int16_t)lroundf(r->x * 1000.0f));
    proto_wr_u16(&w, (uint16_t)lroundf(r->lever * 1000.0f));
    proto_wr_u8(&w, (uint8_t)((r->has_lever ? 1u : 0u) | (r->lev.pickup ? 2u : 0u) |
                              (s->confirm.kind == CONF_REMOTE ? 4u : 0u)));
    proto_wr_u16(&w, (r->lev.pickup && r->lev.target >= 0.0f) ? (uint16_t)lroundf(r->lev.target * 1000.0f) : 0xFFFFu);
    proto_wr_u8(&w, (uint8_t)lroundf(fminf(s->cfg.remote_cap_fwd_pct, s->plim.fwd_pct)));
    proto_wr_u8(&w, (uint8_t)lroundf(fminf(s->cfg.remote_cap_rev_pct, s->plim.rev_pct)));
    proto_wr_u32(&w, r->packets);
    proto_wr_u8(&w, r->lost_why);
    proto_wr_u16(&w, s->save.seq);
    proto_wr_u8(&w, s->save.trig);
    proto_wr_u8(&w, s->save.arg);
    proto_wr_u32(&w, s->save.seq ? s->now_ms - s->save.t_ms : 0u);
    proto_wr_u8(&w, s->save.kinds);
    const slam_t *sl = &s->slam;
    proto_wr_u16(&w, (uint16_t)lroundf(fminf(655.0f, l->slam_peak) * 100.0f));
    l->slam_peak = 0.0f;
    proto_wr_u16(&w, (uint16_t)lroundf(fminf(655.0f, sl->level) * 100.0f));
    proto_wr_u16(&w, sl->rate_pm);
    proto_wr_u8(&w, (uint8_t)lroundf(slam_cap(sl)));
    proto_wr_u8(&w, (uint8_t)(sl->state | (sl->override ? 0x10u : 0u) | (sl->lost ? 0x20u : 0u) |
                              (sl->seen ? 0x40u : 0u)));
    proto_wr_u16(&w, (uint16_t)lroundf(sl->cfg.limit_g * 100.0f));
    proto_wr_u32(&w, sl->impacts);
    proto_wr_u8(&w, (uint8_t)lroundf(sl->cfg.yellow_frac * 100.0f));
    /* G: u8 nav kind | u8 flags (b0 fresh, b1 LOST, b2 applied now) | u8 reason | u8 alarm |
     *    u16 heading*100 | u16 cap kn*10 (0xFFFF none) | u8 seq | u16 age ms (0xFFFF never) */
    const sb_navreq_t *nv = &s->nav;
    bool fresh = nv->rx_ms != 0u && s->now_ms - nv->rx_ms <= nv->lease_ms;
    bool applied = ((sb_mode(s) == MODE_AUTO || (sb_mode(s) == MODE_DP && s->dp.sub == DP_TRIP)) && (nv->lost || (fresh && (nv->kind == SB_NAV_CAP ||
                                                                       nv->kind == SB_NAV_HEADING))));
    proto_wr_u8(&w, nv->kind);
    proto_wr_u8(&w, (uint8_t)((fresh ? 1u : 0u) | (nv->lost ? 2u : 0u) | (applied ? 4u : 0u)));
    proto_wr_u8(&w, nv->reason);
    proto_wr_u8(&w, nv->alarm);
    proto_wr_u16(&w, (uint16_t)lroundf(nv->heading * 100.0f));
    proto_wr_u16(&w, nv->cap_kn < 0.0f ? 0xFFFFu : (uint16_t)lroundf(nv->cap_kn * 10.0f));
    proto_wr_u8(&w, nv->seq);
    proto_wr_u16(&w, nv->rx_ms ? (uint16_t)((s->now_ms - nv->rx_ms) > 65534u ? 65534u : s->now_ms - nv->rx_ms)
                               : 0xFFFFu);
    uint32_t att_age = s->att_ms ? s->now_ms - s->att_ms : 0xFFFFu;
    proto_wr_u16(&w, (uint16_t)(int16_t)lroundf(fmaxf(-180.0f, fminf(180.0f, s->att_roll_deg)) * 100.0f));
    proto_wr_u16(&w, (uint16_t)(int16_t)lroundf(fmaxf(-90.0f, fminf(90.0f, s->att_pitch_deg)) * 100.0f));
    proto_wr_u16(&w, (uint16_t)(att_age > 65534u ? (s->att_ms ? 65534u : 0xFFFFu) : att_age));
    /* I: config flags b0 FIRST TEST | b1 NV DATA RESET (a stored record was unusable: defaults) |
     *    b2 NV FAULT (data flash not working: nothing is saved) | b3 a save waits for STANDBY */
    const nv_status_t *nvs = nv_status();
    proto_wr_u8(&w, (uint8_t)((s->cfg.first_test ? 1u : 0u) | (nvs->reset_mask ? 2u : 0u) |
                              (nvs->fault ? 4u : 0u) | (nvs->pending ? 8u : 0u)));
    proto_wr_u8(&w, s->estop_state);
    proto_wr_u8(&w, (uint8_t)(s->moving_ahead ? 1u : 0u));
    /* J */
    const joy_t *pb = &s->joy_probe;
    bool shore_fresh = s->shore.rx_ms != 0u && s->now_ms - s->shore.rx_ms <= s->cfg.remote_max_age_ms &&
                       s->shore.station == s->helm;
    proto_wr_u8(&w, s->profile);
    proto_wr_u8(&w, (uint8_t)((pb->present && s->probe_ready && !pb->fault ? 1u : 0u) | (pb->present ? 2u : 0u) |
                              (pb->fault ? 4u : 0u) | (!s->probe_ready ? 8u : 0u)));
    proto_wr_u8(&w, (uint8_t)((shore_fresh ? 1u : 0u) | (s->shore.centred ? 2u : 0u) | (s->shore.confirm ? 4u : 0u)));
    proto_wr_u16(&w, s->profile_changes);
    /* K */
    uint32_t cage = s->coast.rx_ms ? s->now_ms - s->coast.rx_ms : 0xFFFFu;
    proto_wr_u8(&w, s->ll.state);
    proto_wr_u8(&w, (uint8_t)((s->ll.armed ? 1u : 0u) | (sb_near_shore(s) ? 2u : 0u) | (s->ll.near_at_kill ? 4u : 0u) |
                              (s->coast.flags << 4)));
    proto_wr_u16(&w, s->coast.dist_m);
    proto_wr_u16(&w, (uint16_t)(cage > 65534u ? (s->coast.rx_ms ? 65534u : 0xFFFFu) : cage));
    proto_wr_u16(&w, (uint16_t)(sb_lostlink_left_s(s) * 10.0f + 0.5f));
    proto_wr_u16(&w, (uint16_t)(s->cfg.near_shore_m + 0.5f));
    /* L */
    {
        const batt_t *b = &s->batt;
        proto_wr_u32(&w, s->alarm_bits2);
        proto_wr_u16(&w, b->valid ? (uint16_t)lroundf(fminf(655.0f, b->bank_v) * 100.0f) : 0xFFFFu);
        proto_wr_u16(&w, b->valid ? (uint16_t)lroundf(fminf(655.0f, b->bank_v_fast) * 100.0f) : 0xFFFFu);
        proto_wr_u8(&w, (uint8_t)((b->valid ? 1u : 0u) | (b->no_data ? 2u : 0u) | (b->low ? 4u : 0u)));
        proto_wr_u8(&w, s->gen.state);      /* generator mode (section K) */
        proto_wr_u8(&w, (uint8_t)(s->gen.demand_pct + 0.5f));
    }
    /* M: relay board (section M): u32 states (bit n-1, readback) | u32 known (readback valid) |
     *    u16 table hash (CRC-16 of the RELAY_CFG encoding; the station compares its file) |
     *    u8 flags (b0 board ok, b1 table active, b2 from NV, b3 CONFIRM pending, b4 power_up applied,
     *    b5 FIRST TEST: no relay changes) | u8 count | u32 commanded on (bit n-1) */
    {
        const relays_t *rl = &s->relays;
        proto_wr_u32(&w, rl->states);
        proto_wr_u32(&w, rl->known);
        proto_wr_u16(&w, rl->hash);
        proto_wr_u8(&w, relays_flags(rl));
        proto_wr_u8(&w, rl->have_table ? rl->table.count : 0u);
        proto_wr_u32(&w, rl->cmd_on);
    }
    /* O: engine controls (section N) */
    {
        const engine_t *e = &s->eng;
        proto_wr_u8(&w, e->crank);
        proto_wr_u8(&w, (uint8_t)((sb_restart_ready(s) ? 1u : 0u) | (e->reset_pending ? 2u : 0u) |
                                  (relays_starter(&s->relays) ? 4u : 0u) | (s->ll.state == SB_LL_KILLED ? 8u : 0u) |
                                  (s->mon_tripped ? 16u : 0u)));
        proto_wr_u16(&w, (uint16_t)(e->hold_ms > 65535u ? 65535u : e->hold_ms));
        proto_wr_u16(&w, (uint16_t)(engine_cool_left_s(e, s->now_ms) * 10.0f + 0.5f));
        proto_wr_u8(&w, e->last_action);
        proto_wr_u8(&w, e->last_why);
        proto_wr_u16(&w, (uint16_t)e->restarts);
        proto_wr_u16(&w, (uint16_t)e->cranks);
    }
    /* N: DP thrusters (section P) */
    {
        const thruster_t *t = &s->thrust;
        proto_wr_u8(&w, thruster_flags(t));
        proto_wr_u8(&w, (uint8_t)(int8_t)lroundf(t->out_pct[THR_BOW]));
        proto_wr_u8(&w, (uint8_t)(int8_t)lroundf(t->out_pct[THR_STERN]));
        proto_wr_u8(&w, (uint8_t)lroundf(thruster_heat(t, THR_BOW) * 100.0f));
        proto_wr_u8(&w, (uint8_t)lroundf(thruster_heat(t, THR_STERN) * 100.0f));
        proto_wr_u8(&w, (uint8_t)lroundf(thruster_cap(t, THR_BOW)));
        proto_wr_u8(&w, (uint8_t)lroundf(thruster_cap(t, THR_STERN)));
        proto_wr_u16(&w, t->have_cfg ? t->cfg_hash : 0u);
        proto_wr_u8(&w, t->why);
        const dp_t *d = &s->dp;
        proto_wr_u8(&w, d->sub);
        proto_wr_u8(&w, (uint8_t)((d->offspot ? 1u : 0u) | (d->overpower ? 2u : 0u) |
                                  (d->sub != DP_OFF && d->compass_lost_ms > 0u ? 4u : 0u) | (d->trip_done ? 8u : 0u) |
                                  (s->confirm.kind == CONF_DP ? 16u : 0u) | (d->fight ? 32u : 0u)));
        float ef = d->sub == DP_OFF ? 0.0f : fmaxf(-3276.0f, fminf(3276.0f, d->err_fwd));
        float es = d->sub == DP_OFF ? 0.0f : fmaxf(-3276.0f, fminf(3276.0f, d->err_stbd));
        proto_wr_u16(&w, (uint16_t)(int16_t)lroundf(ef * 10.0f));
        proto_wr_u16(&w, (uint16_t)(int16_t)lroundf(es * 10.0f));
        proto_wr_u16(&w, (uint16_t)lroundf(d->hdg_target * 10.0f) % 3600u);
        proto_wr_u8(&w, (uint8_t)lroundf(fminf(25.5f, d->set_kn) * 10.0f));
        proto_wr_u8(&w, (uint8_t)lroundf(fminf(25.5f, d->trip_kn) * 10.0f));
        proto_wr_u8(&w, d->wp);
        proto_wr_u8(&w, d->stop_why);
    }
    /* P: DroneCAN summary */
    {
        const hal_can_bus_t *b = &s->can.bus;
        bool f = s->can.fitted;
        proto_wr_u8(&w, (uint8_t)((f ? 1u : 0u) | (f && b->fault ? 2u : 0u) | (f && b->id_conflict ? 4u : 0u) |
                                  (f && b->anon_node ? 8u : 0u) | (f && b->silent ? 16u : 0u) |
                                  (f && s->can.node_block ? 32u : 0u)));
        proto_wr_u8(&w, f ? b->state : HAL_CAN_ST_OFF);
        proto_wr_u8(&w, f ? s->can.lost_mask : 0u);
        proto_wr_u8(&w, f ? s->can.warn_mask : 0u);
        proto_wr_u8(&w, f ? s->can.seen_mask : 0u);
        proto_wr_u8(&w, f ? s->can.reboot_mask : 0u);
    }
    l->telem2_len = (uint16_t)w.n;
    if (!w.overflow) send(l, PROTO_T_TELEM2, l->tx_seq++, buf, w.n);
    else l->telem_overflows++;
}

/* BENCH_DIAG (USB, BENCH TEST only), every sensor for the bench screen:
 *   u8 flags (b0 IMU ok, b1 GPS ok, b2 GPS UBX, b3 SD card, b4 SD mounted, b5 diag from the HAL,
 *             b6 compass ok, b7 gyro ok) |
 *   u8 IMU accuracy (0..3, 255 unknown) | u32 IMU reports | u16 IMU resets |
 *   f32 heading | f32 yaw rate | f32 roll | f32 pitch | f32 vertical accel g |
 *   u8 GPS fix type | u8 sats | u8 spoof state | u8 jam state | u8 fix | f32 SOG | f32 COG |
 *   f32 motor current A (-1 none) | u8 limits (b0 L, b1 R, 0x80 none) | u8 e-stop (0 none, 1 off, 2 pressed) |
 *   u8 SD state | u32 SD free kB | u32 UART errors |
 *   u16 throttle pulse us | u16 gear pulse us | u16 throttle idle us | u16 gear neutral us |
 *   u8 buttons seen | 6 x u8 count per button (hal_button_t) | u16 pulses | u16 refused | u8 last refuse |
 *   i8 pulse running | f32 outboard estimate deg | u8 monitor tripped | u32 idle ms left */
/* CAN_NODES (USB port, 1 Hz while a CAN bus is fitted; groundstation/proto.py decode_can_nodes):
 *   u8 bus state (HAL_CAN_ST_) | u8 flags (TELEM2 group P) | u8 TEC | u8 REC | u32 rx | u32 tx | u32 tx drops |
 *   u32 rx errors | u32 bus-offs | u8 n | n x { u8 node id | u8 role (HAL_CAN_ROLE_) | u8 flags (b0 required,
 *   b1 seen, b2 lost, b3 WARNING, b4 rebooted) | u8 why (HAL_CAN_) | u8 health | u8 mode | u32 uptime s |
 *   u32 NodeStatus age ms (0xFFFFFFFF never) | u16 reboots | u32 NodeStatus received } */
static void can_nodes(link_t *l, sb_t *s)
{
    uint8_t buf[PROTO_MAX_PAYLOAD];
    proto_wr_t w;
    const hal_can_bus_t *b = &s->can.bus;
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u8(&w, b->state);
    proto_wr_u8(&w, (uint8_t)(1u | (b->fault ? 2u : 0u) | (b->id_conflict ? 4u : 0u) | (b->anon_node ? 8u : 0u) |
                              (b->silent ? 16u : 0u) | (s->can.node_block ? 32u : 0u)));
    proto_wr_u8(&w, b->tec);
    proto_wr_u8(&w, b->rec);
    proto_wr_u32(&w, b->rx);
    proto_wr_u32(&w, b->tx);
    proto_wr_u32(&w, b->tx_drops);
    proto_wr_u32(&w, b->rx_errors);
    proto_wr_u32(&w, b->busoffs);
    proto_wr_u8(&w, (uint8_t)HAL_CAN_NODES);
    for (uint32_t i = 0; i < HAL_CAN_NODES; i++) {
        const hal_can_node_t *n = &s->can.node[i];
        proto_wr_u8(&w, n->id);
        proto_wr_u8(&w, n->role);
        proto_wr_u8(&w, (uint8_t)((n->required ? 1u : 0u) | (n->seen ? 2u : 0u) | (n->lost ? 4u : 0u) |
                                  (n->warning ? 8u : 0u) | (n->rebooted ? 16u : 0u)));
        proto_wr_u8(&w, n->why);
        proto_wr_u8(&w, n->health);
        proto_wr_u8(&w, n->mode);
        proto_wr_u32(&w, n->uptime_s);
        proto_wr_u32(&w, n->age_ms);
        proto_wr_u16(&w, n->restarts);
        proto_wr_u32(&w, n->statuses);
    }
    if (!w.overflow) send(l, PROTO_T_CAN_NODES, l->tx_seq++, buf, w.n);
    else l->telem_overflows++;
}

static void bench_diag(link_t *l, sb_t *s)
{
    uint8_t buf[PROTO_MAX_PAYLOAD];
    proto_wr_t w;
    hal_diag_t d;
    memset(&d, 0, sizeof d);
    bool hd = hal_get_diag(&d);
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u8(&w, (uint8_t)((d.imu_ok ? 1u : 0u) | (d.gps_ok ? 2u : 0u) | (d.gps_ubx ? 4u : 0u) |
                              (d.sd_card ? 8u : 0u) | (d.sd_mounted ? 16u : 0u) | (hd ? 32u : 0u) |
                              (sb_compass_ok(s) ? 64u : 0u) | (autopilot_gyro_ok(&s->ap) ? 128u : 0u)));
    proto_wr_u8(&w, hd ? d.imu_accuracy : 255u);
    proto_wr_u32(&w, d.imu_reports);
    proto_wr_u16(&w, (uint16_t)(d.imu_resets > 65535u ? 65535u : d.imu_resets));
    proto_wr_f32(&w, s->ap.heading);
    proto_wr_f32(&w, s->ap.gyro_dps);
    proto_wr_f32(&w, s->att_roll_deg);
    proto_wr_f32(&w, s->att_pitch_deg);
    proto_wr_f32(&w, d.accel_z_g);
    proto_wr_u8(&w, d.gps_fix_type);
    proto_wr_u8(&w, s->gps.last.sats);
    proto_wr_u8(&w, d.gps_spoof);
    proto_wr_u8(&w, d.gps_jam);
    proto_wr_u8(&w, (uint8_t)((s->gps.last.fix ? 1u : 0u) | (s->gps.last.jam_spoof ? 2u : 0u)));
    proto_wr_f32(&w, s->gps.last.sog_kn);
    proto_wr_f32(&w, s->gps.last.cog_deg);
    proto_wr_f32(&w, s->motor_a);
    proto_wr_u8(&w, s->limit_bits);
    proto_wr_u8(&w, s->estop_state);
    proto_wr_u8(&w, d.sd_state);
    proto_wr_u32(&w, d.sd_free_kb);
    proto_wr_u32(&w, d.uart_errors);
    proto_wr_u16(&w, s->servo.out_us);
    proto_wr_u16(&w, s->gear.pwm_us);
    proto_wr_u16(&w, s->cfg.servo.idle_us);
    proto_wr_u16(&w, s->cfg.gear.pwm_n_us);
    proto_wr_u8(&w, s->bench.buttons_seen);
    for (unsigned b = 0; b < (unsigned)HAL_BTN_COUNT; b++) proto_wr_u8(&w, s->bench.button_count[b]);
    proto_wr_u16(&w, s->bench.pulses);
    proto_wr_u16(&w, s->bench.refused);
    proto_wr_u8(&w, s->bench.last_refuse);
    proto_wr_u8(&w, (uint8_t)s->bench.pulse_dir);
    proto_wr_f32(&w, s->ap.est_angle);
    proto_wr_u8(&w, s->mon_tripped ? 1u : 0u);
    uint32_t idle = s->now_ms - s->bench.last_cmd_ms;
    proto_wr_u32(&w, idle < s->cfg.bench_idle_ms ? s->cfg.bench_idle_ms - idle : 0u);
    if (!w.overflow) send(l, PROTO_T_BENCH_DIAG, l->tx_seq++, buf, w.n);
    else l->telem_overflows++;
}

/* GW_STATUS from the Pi (1 Hz, also its heartbeat), 12 bytes:
 *   u8 version | u8 flags (SB_GWF_) | u8 channel | u8 stations |
 *   u32 last log record the Pi has | u16 log gaps | u16 reserved */
static void gw_status(sb_t *s, const proto_frame_t *f)
{
    proto_rd_t r;
    proto_rd_init(&r, f->payload, f->len);
    (void)proto_rd_u8(&r);
    uint8_t flags = proto_rd_u8(&r);
    sb_gw_t *g = &s->gw;
    if ((flags & (SB_GWF_STORAGE | SB_GWF_STORAGE_FULL)) & ~g->flags) {
        s->alarm = true;
        s->alarm_bits |= SB_ALM_GW_STORAGE;
    }
    if ((flags & SB_GWF_AIS_SILENT) & ~g->flags) {
        s->alarm = true;
        s->alarm_bits |= SB_ALM_AIS_SILENT;              /* warning only: no mode / power change */
    }
    if ((flags & SB_GWF_FALLBACK) & ~g->flags) {
        s->alarm = true;
        s->alarm_bits |= SB_ALM_CHANNEL;                 /* alarm only, no mode change */
    }
    g->flags = flags;
    g->channel = proto_rd_u8(&r);
    g->stations = proto_rd_u8(&r);
    g->pi_log_seq = proto_rd_u32(&r);
    g->pi_log_gaps = proto_rd_u16(&r);
}

/* GPS time for the Pi (it sets its clock from it, so logs and video agree),
 * 13 bytes: u8 valid | u32 date YYYYMMDD | u32 UTC ms of the day | u32 boat t_ms */
static void gw_time(link_t *l, sb_t *s)
{
    uint8_t buf[13];
    proto_wr_t w;
    const hal_gps_t *g = &s->gps.last;
    bool valid = s->gps.valid && g->fix && g->date != 0u && (s->now_ms - s->gps.last_ms) < 5000u;
    uint32_t tod = valid ? (g->time_ms + (s->now_ms - s->gps.last_ms)) % 86400000u : 0u;
    proto_wr_init(&w, buf, sizeof buf);
    proto_wr_u8(&w, valid ? 1u : 0u);
    proto_wr_u32(&w, valid ? g->date : 0u);
    proto_wr_u32(&w, tod);
    proto_wr_u32(&w, s->now_ms);
    send(l, PROTO_T_TIME, l->tx_seq++, buf, w.n);
}

/* The full log copy for the Pi: every record the logger handles, cut into
 * LOG_CHUNK frames, ONLY into the room the UART has left after a reserve for
 * control frames. If the UART is too slow the logger's queue fills and
 * records are dropped there (counted; the Pi sees the gap in the numbers).
 * Without a gateway the records are discarded (log_skipped). The SD card
 * copy is never affected.
 *   LOG_CHUNK: u32 record number | u16 offset | u16 record length | bytes */
#define GW_LOG_CHUNK     180u
#define GW_CTRL_RESERVE  1024u
#define GW_LOG_BUDGET    4096u      /* bytes per control step (bounded time) */
#define GW_LOG_MIN_INFLIGHT 256u
#define GW_LOG_MAX_INFLIGHT 4096u

static uint8_t s_gwrec[4096 + 64 + 4];

static void gw_log_pump(link_t *l, sb_t *s)
{
    uint32_t sent = 0u;
    bool up = l->connected && !l->lost;
    while (sent < GW_LOG_BUDGET) {
        if (l->log_len == 0u) {
            uint16_t n = logq_pop(&g_logq_gw, s_gwrec, (uint16_t)sizeof s_gwrec);
            if (n == 0u) break;
            if (n < 4u + LOGR_OVERHEAD) continue;
            if (!up) {
                s->gw.log_skipped++;
                continue;
            }
            l->log_len = n;
            l->log_off = 4u;
        }
        uint32_t chunk = l->log_len - l->log_off;
        if (chunk > GW_LOG_CHUNK) chunk = GW_LOG_CHUNK;
        /* never more log bytes waiting in the UART than it sent during the last
         * step: control frames queue behind them (FIFO), so they stay fast */
        uint32_t used = HAL_GW_TX_RING - hal_gw_tx_free();
        if (used + chunk + 8u + PROTO_OVERHEAD > l->gw_room ||
            hal_gw_tx_free() < chunk + 8u + PROTO_OVERHEAD + GW_CTRL_RESERVE) break;   /* UART busy */
        uint8_t p[GW_LOG_CHUNK + 8u];
        memcpy(p, s_gwrec, 4u);                                  /* record number */
        p[4] = (uint8_t)(l->log_off - 4u);
        p[5] = (uint8_t)((l->log_off - 4u) >> 8);
        p[6] = (uint8_t)(l->log_len - 4u);
        p[7] = (uint8_t)((l->log_len - 4u) >> 8);
        memcpy(&p[8], &s_gwrec[l->log_off], chunk);
        send(l, PROTO_T_LOG_CHUNK, l->tx_seq++, p, chunk + 8u);
        l->log_chunks++;
        l->log_off += chunk;
        sent += chunk + 8u + PROTO_OVERHEAD;
        if (l->log_off >= l->log_len) {
            s->gw.log_sent_seq = (uint32_t)s_gwrec[0] | ((uint32_t)s_gwrec[1] << 8) |
                                 ((uint32_t)s_gwrec[2] << 16) | ((uint32_t)s_gwrec[3] << 24);
            l->log_len = 0u;
        }
    }
}

/* --- public --------------------------------------------------------------- */

void link_init_port(link_t *l, sb_t *s, uint8_t port)
{
    memset(l, 0, sizeof *l);
    l->port = port;
    proto_rx_init(&l->rx);
    l->pc_timeout_s = 3.0f;
    l->next_telem_ms = s->now_ms;
    l->next_time_ms = s->now_ms;
}

void link_init(link_t *l, sb_t *s)
{
    link_init_port(l, s, LINK_PORT_USB);
}

void link_step(link_t *l, sb_t *s)
{
    uint8_t buf[128];
    uint32_t n;
    proto_frame_t f;

    /* bounded work per step: at most 4 kB in */
    uint32_t got = 0u;
    int frames = 0;
    if (l->port == LINK_PORT_GW) {
        /* how fast is the UART really? Emptied completely: allow twice what it
         * really sent in the last step (it grows only while the UART keeps up -
         * never by itself during quiet times, so a burst after a pause can't
         * queue more than ~2 steps of data in front of control frames);
         * otherwise half of what it sent */
        uint32_t used_now = HAL_GW_TX_RING - hal_gw_tx_free();
        uint32_t drained = l->gw_used_after - used_now;
        l->gw_room = (used_now == 0u) ? (l->gw_room > 2u * drained ? l->gw_room : 2u * drained) : drained / 2u;
        if (l->gw_room < GW_LOG_MIN_INFLIGHT) l->gw_room = GW_LOG_MIN_INFLIGHT;
        if (l->gw_room > GW_LOG_MAX_INFLIGHT) l->gw_room = GW_LOG_MAX_INFLIGHT;
    }
    /* stop reading after 64 frames: more bytes would overwrite frames in the
     * 512-byte ring that are not parsed yet - the rest waits in the port's
     * buffer for the next step */
    for (int k = 0; k < 32 && frames < 64 && (n = port_read(l, buf, sizeof buf)) > 0u; k++) {
        proto_rx_push(&l->rx, buf, n);          /* parse after every read: the ring holds 512 bytes */
        got += n;
        while (frames < 64 && proto_rx_next(&l->rx, &f)) { handle(l, s, &f); frames++; }
    }

    /* bytes stuck in an incomplete frame with nothing new for 2 steps
     * (100 ms; a real frame takes ~2 ms): rescan from the next byte, so a
     * corrupted length byte can't hold back the following good frame */
    if (got == 0u && proto_rx_pending(&l->rx) > 0u) {
        if (++l->rx_idle_steps >= 2u) {
            l->rx_idle_steps = 0u;
            for (int k = 0; k < 8 && proto_rx_pending(&l->rx) > 0u; k++) {
                proto_rx_stale(&l->rx);
                while (proto_rx_next(&l->rx, &f)) handle(l, s, &f);
            }
        }
    } else {
        l->rx_idle_steps = 0u;
    }

    /* heartbeat watch: alarm only, the mode stays */
    if (l->connected && !l->lost &&
        (float)(uint32_t)(s->now_ms - l->last_rx_ms) > l->pc_timeout_s * 1000.0f) {
        l->lost = true;
        s->alarm = true;
        if (l->port == LINK_PORT_GW) {
            s->alarm_bits |= SB_ALM_GATEWAY;            /* GATEWAY LOST: alarm only, no mode change */
            s->gw.state = SB_GW_LOST;
            if (s->helm >= 2u) helm_lost(s, s->helm, 4u);   /* its stations are gone too */
        } else {
            s->alarm_bits |= SB_ALM_PC_LINK;
            if (sb_mode(s) == MODE_BENCH) sb_request_mode(s, MODE_STANDBY, WHY_BENCH_END);   /* laptop gone */
        }
    } else if (l->lost) {
        /* still lost: an acknowledge can't make the alarm disappear */
        uint32_t bit = (l->port == LINK_PORT_GW) ? SB_ALM_GATEWAY : SB_ALM_PC_LINK;
        if (!(s->alarm_bits & bit)) {
            s->alarm_bits |= bit;
            s->alarm = true;
        }
    }

    if (s->slam.peak > l->slam_peak) l->slam_peak = s->slam.peak;     /* the hardest since the last telemetry */
    if ((int32_t)(s->now_ms - l->next_telem_ms) >= 0) {
        l->next_telem_ms = s->now_ms + TELEM_PERIOD_MS;
        telemetry2(l, s);                               /* first: the PC merges it into the next telemetry */
        telemetry(l, s);
        if (l->port != LINK_PORT_GW && sb_mode(s) == MODE_BENCH) bench_diag(l, s);
    }
    if (l->port != LINK_PORT_GW && s->can.fitted && (int32_t)(s->now_ms - l->next_can_ms) >= 0) {
        l->next_can_ms = s->now_ms + 1000u;
        can_nodes(l, s);
    }
    if (l->port == LINK_PORT_GW) {
        if ((int32_t)(s->now_ms - l->next_time_ms) >= 0) {
            l->next_time_ms = s->now_ms + 1000u;
            gw_time(l, s);
        }
        gw_log_pump(l, s);                              /* last: only the room left */
        l->gw_used_after = HAL_GW_TX_RING - hal_gw_tx_free();
    }
}
