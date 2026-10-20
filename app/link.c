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

#define TELEM_PERIOD_MS 100u
#define PI_F 3.14159265f

/* --- parameters ----------------------------------------------------------- */

typedef enum {
    P_STEER_KP = 0, P_STEER_KI, P_STEER_KD, P_TURN_RATE,
    P_CRUISE_MAX, P_SPD_KP, P_SPD_KI,
    P_SLOW_R, P_APPROACH_KN, P_TRAINING_THR,
    P_DR_MAX_UNC, P_DR_SPEED_F, P_DR_SLOW_KN,
    P_JOY_MAX_THR, P_JOY_DEADBAND, P_PC_TIMEOUT,
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
    default:             return NULL;
    }
}

/* --- sending -------------------------------------------------------------- */

static void send(link_t *l, uint8_t type, uint8_t seq, const uint8_t *payload, size_t len)
{
    uint8_t frame[PROTO_MAX_FRAME];
    size_t n = proto_encode(frame, type, seq, payload, (uint8_t)len);
    if (n == 0u || !hal_link_write(frame, (uint32_t)n)) l->tx_dropped++;
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
#define JOYMAP_BODY 31u

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
 *     b1 suspended, b2 finished) | u8 confirm_kind (confirm_kind_t) | u16 confirm_left (0.1 s)
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
    proto_wr_u8(&w, (uint8_t)((a->prog_ok ? 1u : 0u) | (a->suspended ? 2u : 0u) | (a->finished ? 4u : 0u)));
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

    if (!w.overflow) send(l, PROTO_T_TELEMETRY, l->tx_seq++, buf, w.n);
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
        s->alarm = false;
        s->alarm_bits = 0u;
        s->ap.alarm = false;
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
        if (res == SB_OK) l->staging_n = 0u;
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
    default:
        return PROTO_R_UNKNOWN;
    }
}

static void handle(link_t *l, sb_t *s, const proto_frame_t *f)
{
    l->last_rx_ms = s->now_ms;
    if (!l->connected || l->lost) {
        /* PC (back) online: its own alarm cause is gone */
        l->connected = true;
        l->lost = false;
        s->alarm_bits &= ~SB_ALM_PC_LINK;
        if (s->alarm_bits == 0u && !s->ap.alarm) s->alarm = false;
    }
    if (f->type == PROTO_T_HEARTBEAT && f->len == 0u) return;   /* no ACK for heartbeats */
    l->commands++;

    uint8_t r;
    int want = cmd_len(f->type);
    if (f->type >= PROTO_T_SIM_FIRST && f->type <= PROTO_T_SIM_LAST) r = l->sim_fn ? l->sim_fn(l->sim_ctx, f) : PROTO_R_UNKNOWN;
    else if (want < 0) r = PROTO_R_UNKNOWN;
    else if ((int)f->len != want) r = PROTO_R_BAD_LENGTH;
    else r = do_command(l, s, f);
    send_ack(l, f, r);
    logcore0_pc(s->now_ms, f->type, f->seq, r, f->payload, f->len);     /* every command + answer */
}

/* --- public --------------------------------------------------------------- */

void link_init(link_t *l, sb_t *s)
{
    memset(l, 0, sizeof *l);
    proto_rx_init(&l->rx);
    l->pc_timeout_s = 3.0f;
    l->next_telem_ms = s->now_ms;
}

void link_step(link_t *l, sb_t *s)
{
    uint8_t buf[128];
    uint32_t n;
    proto_frame_t f;

    /* bounded work per step: at most 4 kB in */
    uint32_t got = 0u;
    for (int k = 0; k < 32 && (n = hal_link_read(buf, sizeof buf)) > 0u; k++) {
        proto_rx_push(&l->rx, buf, n);
        got += n;
    }
    for (int k = 0; k < 64 && proto_rx_next(&l->rx, &f); k++) handle(l, s, &f);

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
        s->alarm_bits |= SB_ALM_PC_LINK;
    }

    if ((int32_t)(s->now_ms - l->next_telem_ms) >= 0) {
        l->next_telem_ms = s->now_ms + TELEM_PERIOD_MS;
        telemetry(l, s);
    }
}
