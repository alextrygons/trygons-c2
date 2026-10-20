/*
 * link.c - Ground-station link. See link.h, frame format in proto.h.
 */
#include "link.h"

#include <math.h>
#include <string.h>

#include "hal.h"

#define TELEM_PERIOD_MS 100u
#define PI_F 3.14159265f

/* --- parameters ----------------------------------------------------------- */

typedef enum {
    P_STEER_KP = 0, P_STEER_KI, P_STEER_KD, P_TURN_RATE,
    P_MANEUVER_THR, P_CRUISE_MAX, P_SPD_KP, P_SPD_KI,
    P_ARRIVAL_R, P_SLOW_R, P_APPROACH_KN, P_TRAINING_THR,
    P_DR_MAX_UNC, P_DR_SPEED_F, P_DR_SLOW_KN,
    P_JOY_MAX_THR, P_JOY_DEADBAND, P_PC_TIMEOUT,
    P_COUNT
} param_id_t;

static const link_param_info_t s_params[P_COUNT] = {
    { "steer_kp",        0.0f,   20.0f },
    { "steer_ki",        0.0f,    5.0f },
    { "steer_kd",        0.0f,   20.0f },
    { "turn_rate_dps",   1.0f,   30.0f },
    { "maneuver_thr_pc", 5.0f,   60.0f },
    { "cruise_max_kn",   3.0f,   35.0f },
    { "spd_kp",          0.0f,   20.0f },
    { "spd_ki",          0.0f,    5.0f },
    { "arrival_r_m",     5.0f,  500.0f },
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
    case P_MANEUVER_THR: return &s->cfg.maneuver_throttle_pct;
    case P_CRUISE_MAX:   return &s->cfg.cruise_max_kn;
    case P_SPD_KP:       return &s->cfg.spd_kp;
    case P_SPD_KI:       return &s->cfg.spd_ki;
    case P_ARRIVAL_R:    return &s->cfg.arrival_radius_m;
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

/*
 * Telemetry payload (little-endian), groundstation/proto.py must match:
 *   u32 t_ms | u8 mode | u8 ap_mode | u8 sea_state | u8 flags
 *     flags: b0 compass ok, b1 GPS ok, b2 alarm output, b3 waypoint set,
 *            b4 PC link was lost (still latched), b5 stopping
 *   f32 heading | f32 target | f32 yaw_rate | f32 outboard_est | f32 throttle_pct
 *   u8 gear (0 N, 1 F, 2 R) | u8 gps_fix
 *   i32 gps_lat | i32 gps_lon | f32 gps_sog | f32 gps_cog
 *   u8 pos_source (dr_state_t) | u8 dr_reason
 *   f32 unc_m | f32 dr_time_s | i32 est_lat | i32 est_lon | f32 est_sog | f32 est_cog
 *   f32 set_deg (current flows toward) | f32 drift_kn | f32 current_conf (0..1)
 *   u32 alarm_bits | f32 motor_current_a (-1 = no sensor)
 *   f32 joy_x | f32 joy_y | u8 joy_buttons (b0 TAKE, b1 AUTO)
 *   u8 joy_status (0 none, 1 not armed / not centred, 2 armed, 3 fault) | u8 joy_fault_axis
 *   u8 train_stage | u8 train_phase | u8 train_result
 *   f32 cruise_set_kn | i32 wp_lat | i32 wp_lon | f32 wp_kn
 *   u16 rx_frames | u16 rx_crc_errors
 */
static void telemetry(link_t *l, sb_t *s)
{
    uint8_t buf[PROTO_MAX_PAYLOAD];
    proto_wr_t w;
    sb_status_t st = sb_status(s);
    proto_wr_init(&w, buf, sizeof buf);

    uint8_t flags = 0u;
    if (sb_compass_ok(s)) flags |= 0x01u;
    if (sb_gps_ok(s)) flags |= 0x02u;
    if (s->alarm || s->ap.alarm || s->chirp_ms > 0u) flags |= 0x04u;
    if (s->nav.wp_set) flags |= 0x08u;
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
    proto_wr_u8(&w, (uint8_t)((j->take ? 1u : 0u) | (j->autob ? 2u : 0u)));
    proto_wr_u8(&w, (uint8_t)(!j->present ? 0u : j->fault ? 3u : j->armed ? 2u : 1u));
    proto_wr_u8(&w, j->fault ? j->fault_axis : 0u);

    proto_wr_u8(&w, (uint8_t)s->train.stage);
    proto_wr_u8(&w, (uint8_t)s->ap.train.phase);
    proto_wr_u8(&w, (uint8_t)s->ap.train.result);

    proto_wr_f32(&w, s->cruise.set_kn);
    proto_wr_latlon(&w, s->nav.wp_lat);
    proto_wr_latlon(&w, s->nav.wp_lon);
    proto_wr_f32(&w, s->nav.speed_kn);
    proto_wr_u16(&w, (uint16_t)(l->rx.frames & 0xFFFFu));
    proto_wr_u16(&w, (uint16_t)(l->rx.crc_errors & 0xFFFFu));

    if (!w.overflow) send(l, PROTO_T_TELEMETRY, l->tx_seq++, buf, w.n);
}

/* --- commands ------------------------------------------------------------- */

/* Expected payload length per command, -1 = unknown command. */
static int cmd_len(uint8_t type)
{
    switch (type) {
    case PROTO_T_HEARTBEAT:    return 0;
    case PROTO_T_SET_MODE:     return 1;
    case PROTO_T_JOY_HANDBACK: return 0;
    case PROTO_T_HEADING_ADJ:  return 1;
    case PROTO_T_WAYPOINT:     return 12;
    case PROTO_T_CRUISE_SPEED: return 4;
    case PROTO_T_SEA_STATE:    return 1;
    case PROTO_T_TRAINING:     return 5;
    case PROTO_T_PARAM_GET:    return 1;
    case PROTO_T_PARAM_SET:    return 5;
    case PROTO_T_JOY_CAL:      return 1;
    case PROTO_T_ALARM_ACK:    return 0;
    default:                   return -1;
    }
}

/* Mode change through the mode manager; the result says why if refused. */
static uint8_t pc_mode(sb_t *s, sb_mode_t to)
{
    if (to == MODE_JOYSTICK) {                                  /* only the stick takes control */
        (void)sb_request_mode(s, to, WHY_PC);                    /* refused + logged */
        return PROTO_R_NOT_ALLOWED;
    }
    if (sb_mode(s) == MODE_JOYSTICK) {
        (void)sb_request_mode(s, to, WHY_PC);                    /* refused + logged */
        return PROTO_R_JOYSTICK;
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
    bool joy = (sb_mode(s) == MODE_JOYSTICK);

    switch (f->type) {
    case PROTO_T_SET_MODE: {
        uint8_t m = proto_rd_u8(&r);
        if (m >= (uint8_t)MODE_COUNT) return PROTO_R_BAD_VALUE;
        return pc_mode(s, (sb_mode_t)m);
    }
    case PROTO_T_JOY_HANDBACK:
        if (!joy) return PROTO_R_WRONG_MODE;
        return sb_joystick_handback(s, WHY_JOY_HANDBACK) ? PROTO_R_OK : PROTO_R_NOT_CENTERED;

    case PROTO_T_HEADING_ADJ: {
        int8_t d = proto_rd_i8(&r);
        uint32_t bit;
        if (d == -1) bit = HAL_BTN_BIT(HAL_BTN_MINUS_1);
        else if (d == 1) bit = HAL_BTN_BIT(HAL_BTN_PLUS_1);
        else if (d == -10) bit = HAL_BTN_BIT(HAL_BTN_MINUS_10);
        else if (d == 10) bit = HAL_BTN_BIT(HAL_BTN_PLUS_10);
        else return PROTO_R_BAD_VALUE;
        if (joy) return PROTO_R_JOYSTICK;
        if (sb_mode(s) != MODE_CRUISE) return PROTO_R_WRONG_MODE;   /* same as the buttons */
        s->ext_buttons |= bit;
        return PROTO_R_OK;
    }
    case PROTO_T_WAYPOINT: {
        double lat = proto_rd_latlon(&r), lon = proto_rd_latlon(&r);
        float kn = proto_rd_f32(&r);
        if (!(fabs(lat) <= 90.0) || !(fabs(lon) <= 180.0) || !(kn > 0.0f && kn <= s->cfg.cruise_max_kn))
            return PROTO_R_BAD_VALUE;
        if (sb_mode(s) == MODE_NAV) return PROTO_R_WRONG_MODE;      /* the leg is fixed while NAV runs */
        sb_set_waypoint(s, lat, lon, kn);
        return PROTO_R_OK;
    }
    case PROTO_T_CRUISE_SPEED: {
        float kn = proto_rd_f32(&r);
        if (!(kn >= 0.0f && kn <= s->cfg.cruise_max_kn)) return PROTO_R_BAD_VALUE;
        sb_set_cruise_speed(s, kn);
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
        if (id == P_SLOW_R && v < s->cfg.arrival_radius_m) return PROTO_R_BAD_VALUE;
        if (id == P_ARRIVAL_R && v > s->cfg.slow_radius_m) return PROTO_R_BAD_VALUE;
        *param_ptr(l, s, id) = v;
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
    case PROTO_T_ALARM_ACK:
        if (sb_mode(s) != MODE_STANDBY) return PROTO_R_STANDBY_ONLY;
        s->alarm = false;
        s->alarm_bits = 0u;
        s->ap.alarm = false;
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

    if (f->type >= PROTO_T_SIM_FIRST && f->type <= PROTO_T_SIM_LAST) {
        send_ack(l, f, l->sim_fn ? l->sim_fn(l->sim_ctx, f) : PROTO_R_UNKNOWN);
        return;
    }
    int want = cmd_len(f->type);
    if (want < 0) { send_ack(l, f, PROTO_R_UNKNOWN); return; }
    if ((int)f->len != want) { send_ack(l, f, PROTO_R_BAD_LENGTH); return; }
    send_ack(l, f, do_command(l, s, f));
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
