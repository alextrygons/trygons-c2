/*
 * logfields.h - The 20 Hz log sample: one line per field.
 *
 *   X(name, type, scale, value)
 *   type: B u8, c i8, H u16, h i16, I u32, i i32, f f32
 *   stored = value * scale (rounded, clamped to the type's range)
 *
 * `value` is evaluated in logcore0.c with `s` = the small-boat controller
 * and `v` = a few values worked out before (see logcore0.c). The same list
 * writes the field description into every file header, so the PC tools
 * (groundstation/tlg.py) never need changing when a field is added here.
 */
#define LOG_FIELDS(X) \
    /* mode and alarms */ \
    X(mode,          B, 1,     sb_mode(s)) \
    X(ap_mode,       B, 1,     s->ap.mode) \
    X(auto_state,    B, 1,     s->autop.state) \
    X(auto_wp,       B, 1,     s->autop.wp) \
    X(armed,         B, 1,     s->armed) \
    X(alarm_bits,    I, 1,     s->alarm_bits) \
    X(alarm_out,     B, 1,     (s->alarm || s->ap.alarm)) \
    /* heading, sensors */ \
    X(heading,       H, 100,   s->ap.heading) \
    X(compass_ok,    B, 1,     sb_compass_ok(s)) \
    X(yaw_rate,      h, 100,   s->ap.yaw_rate) \
    X(gyro,          h, 100,   s->ap.gyro_dps) \
    X(gyro_ok,       B, 1,     s->ap.gyro_valid) \
    X(roll_rate,     h, 100,   s->roll_ok ? s->roll_dps : 0.0f) \
    /* steering controller */ \
    X(target,        H, 100,   s->ap.target_heading) \
    X(ref_heading,   H, 100,   s->ap.ref_heading) \
    X(heading_err,   h, 100,   v.heading_err) \
    X(pid_p,         h, 100,   v.pid_p) \
    X(pid_i,         h, 100,   s->ap.integrator) \
    X(pid_d,         h, 100,   v.pid_d) \
    X(rudder_target, h, 100,   s->ap.rudder_target) \
    X(outboard_est,  h, 100,   s->ap.est_angle) \
    X(motor_cmd,     c, 100,   s->ap.motor_cmd) \
    X(steer_limit,   H, 100,   s->ap.steer_limit_deg) \
    X(gain_scale,    H, 1000,  s->ap.gain_scale) \
    X(stop_block,    c, 1,     s->ap.stop_block) \
    X(steer_paused,  B, 1,     s->hdg_reverse) \
    /* throttle after every limit stage, gear */ \
    X(thr_mode,      H, 100,   s->throttle_mode) \
    X(thr_limited,   H, 100,   s->throttle_demand) \
    X(thr_rate,      H, 100,   s->throttle_rate) \
    X(thr_out,       H, 100,   s->servo.out_pct) \
    X(thr_us,        H, 1,     s->servo.out_us) \
    X(power_fwd,     H, 10,    s->plim.fwd_pct) \
    X(power_rev,     H, 10,    s->plim.rev_pct) \
    X(speed_limited, B, 1,     s->speed_limited) \
    X(gear_demand,   B, 1,     s->gear_demand) \
    X(gear,          B, 1,     s->gear.current) \
    X(gear_phase,    B, 1,     s->gear.phase) \
    X(gear_us,       H, 1,     s->gear.pwm_us) \
    X(neutral_sw,    c, 1,     s->neutral_sw) \
    /* GPS */ \
    X(gps_lat,       i, 1e7,   s->gps.last.lat_deg) \
    X(gps_lon,       i, 1e7,   s->gps.last.lon_deg) \
    X(gps_sog,       H, 100,   s->gps.last.sog_kn) \
    X(gps_cog,       H, 100,   s->gps.last.cog_deg) \
    X(gps_fix,       B, 1,     (s->gps.valid && s->gps.last.fix)) \
    X(gps_ok,        B, 1,     sb_gps_ok(s)) \
    X(gps_sats,      B, 1,     s->gps.last.sats) \
    X(gps_spoof,     B, 1,     s->gps.last.jam_spoof) \
    X(gps_age,       H, 1,     v.gps_age_ms) \
    X(gps_time,      I, 1,     s->gps.last.time_ms) \
    X(gps_date,      I, 1,     s->gps.last.date) \
    /* position estimate, current */ \
    X(pos_source,    B, 1,     s->dr.state) \
    X(dr_reason,     B, 1,     s->dr.why) \
    X(est_lat,       i, 1e7,   v.est_lat) \
    X(est_lon,       i, 1e7,   v.est_lon) \
    X(unc,           H, 10,    s->dr.unc_m) \
    X(est_sog,       H, 100,   v.est_sog) \
    X(est_cog,       H, 100,   v.est_cog) \
    X(cur_n,         h, 1000,  s->dr.cur_n) \
    X(cur_e,         h, 1000,  s->dr.cur_e) \
    X(cur_conf,      B, 100,   v.cur_conf) \
    X(cur_age,       H, 1,     s->dr.cur_learn_s) \
    /* navigation */ \
    X(xte,           h, 10,    s->autop.leg.xte_m) \
    X(dist_to_go,    I, 10,    s->autop.leg.dist_m) \
    X(crab,          h, 100,   s->autop.crab_deg) \
    X(course,        H, 100,   s->autop.course_deg) \
    X(speed_set,     H, 100,   s->cruise.set_kn) \
    X(speed_target,  H, 100,   s->cruise.target_kn) \
    X(nervous,       B, 1,     s->nerv.nervous) \
    X(yaw_std,       H, 100,   s->nerv.yaw_std) \
    /* other sensors */ \
    X(water_kn,      H, 100,   v.water_kn) \
    X(motor_a,       H, 100,   v.motor_a) \
    X(limit_sw,      B, 1,     s->limit_bits) \
    X(supply_v,      H, 100,   v.supply_v) \
    /* joystick, buttons */ \
    X(joy_x,         h, 1000,  s->joy.x) \
    X(joy_y,         h, 1000,  s->joy.y) \
    X(joy_raw_x,     H, 1,     s->joy.raw[0]) \
    X(joy_raw_y,     H, 1,     s->joy.raw[1]) \
    X(joy_buttons,   B, 1,     v.joy_buttons) \
    X(joy_status,    B, 1,     v.joy_status) \
    X(joy_lever,     H, 1000,  s->joy.lever_in) \
    X(joy_src,       B, 1,     v.joy_src) \
    X(joy_why,       B, 1,     s->joy.fault ? s->joy.fault_axis : s->jb.why) \
    X(panel_buttons, B, 1,     s->buttons_now) \
    X(confirm_kind,  B, 1,     s->confirm.kind) \
    /* timing, logging */ \
    X(t_core0_us,    H, 1,     v.t_core0) \
    X(t_mon_max_us,  H, 1,     v.t_mon) \
    X(t_log_max_us,  H, 1,     v.t_log) \
    X(overruns,      H, 1,     g_timing.core0.overruns) \
    X(log_dropped,   H, 1,     g_logq0.dropped)
