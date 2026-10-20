/*
 * sim_sb.c - Small-boat feature tests (one block per phase).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_sim.h"
#include "sim_run.h"
#include "sim_world.h"
#include "link.h"
#include "monitor.h"
#include "timing.h"
#include "smallboat.h"

/* ------------------------------------------------------------ harness -- */

typedef struct sbsim sbsim_t;
typedef void (*sb_hook_t)(sbsim_t *x, void *ctx);

struct sbsim {
    world_t  w;
    sb_t     sb;
    uint32_t t;             /* ms */
    uint32_t next_ctrl;
    sb_config_t cfg;        /* for a simulated reset */
    mon_t    mon;           /* independent safety monitor (CPU1 on the board) */
    bool     mon_on;
    bool     frozen;        /* core 0 hangs: no control steps, no watchdog kicks */
    uint32_t wdt_ms;        /* watchdog timeout, 0 = off */
    uint32_t last_kick;
    uint32_t boot_until;    /* after a reset: chip boots until then */
    uint32_t resets;
    uint32_t t_reset;       /* time of the last watchdog reset */
    uint32_t inject_us;     /* tests: extra execution time of the next control steps */
};

/* Joystick position (-1..+1, default calibration) -> raw ADC. */
static uint16_t joy_raw(float v)
{
    joy_cal_t c;
    joy_default_cal(&c);
    float r = (v >= 0.0f) ? c.center[0] + v * (float)(c.max[0] - c.center[0])
                          : c.center[0] + v * (float)(c.center[0] - c.min[0]);
    return (uint16_t)lrintf(r);
}

static void joy_set(float x, float y, bool take, bool autob)
{
    sim_hal_set_joystick(true, joy_raw(x), joy_raw(y), take, autob);
}

static void sbsim_init(sbsim_t *x, uint32_t seed, float heading, float speed_kn,
                       const sb_config_t *cfg)
{
    sb_config_t c;
    memset(x, 0, sizeof *x);
    world_init(&x->w, seed, heading, speed_kn);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, 0.05f, 0.02f);
    sim_hal_reset_engine();
    joy_set(0.0f, 0.0f, false, false);              /* joystick fitted, centred */
    sim_hal_joystick_confirm(false);
    remove("nv1.bin");                              /* default joystick calibration */
    remove("nv2.bin");                              /* no stored AUTO program */
    sim_hal_unforce();
    memset((void *)&g_mon, 0, sizeof g_mon);
    world_publish(&x->w, 0);
    hal_init();
    if (cfg) c = *cfg; else sb_default_config(&c);
    x->cfg = c;
    timing_init_all();
    sb_init(&x->sb, &c);
    monitor_init(&x->mon, 0u);
    x->mon_on = true;
    x->wdt_ms = 100u;
}

/* Run until absolute time until_ms; hook after every control step. */
static void sbsim_run(sbsim_t *x, uint32_t until_ms, sb_hook_t hook, void *ctx)
{
    while (x->t <= until_ms) {
        world_publish(&x->w, x->t);
        if (x->boot_until && x->t >= x->boot_until) {
            /* chip has rebooted after the watchdog reset */
            x->boot_until = 0u;
            x->frozen = false;
            memset((void *)&g_mon, 0, sizeof g_mon);
            hal_init();
            sb_init(&x->sb, &x->cfg);
            monitor_init(&x->mon, x->t);
            x->next_ctrl = x->t;
            x->last_kick = x->t;
        }
        if (!x->boot_until && x->t >= x->next_ctrl) {
            x->next_ctrl += AP_PERIOD_MS;
            if (!x->frozen) {
                uint32_t t0 = hal_micros();
                sb_step(&x->sb);
                (void)cyc_record(&g_timing.core0, hal_micros() - t0 + x->inject_us);
                x->last_kick = x->t;                /* hal_watchdog_kick() in the loop */
                if (hook) hook(x, ctx);
            }
        }
        /* watchdog: not kicked in time -> chip reset, outputs stop */
        if (x->wdt_ms && !x->boot_until && x->t - x->last_kick > x->wdt_ms) {
            x->resets++;
            x->t_reset = x->t;
            sim_hal_chip_reset(HAL_RESET_SYSTEM);
            x->boot_until = x->t + 200u;
        }
        /* the independent monitor (CPU1): its own check step */
        if (x->mon_on && !x->boot_until) {
            uint32_t t0 = hal_micros();
            monitor_step(&x->mon, &g_mon, x->t);
            (void)cyc_record(&g_timing.monitor, hal_micros() - t0);
        }
        world_step(&x->w, PHYSICS_DT_MS * 0.001f);
        x->t += PHYSICS_DT_MS;
    }
}

/* Press a panel button and let the controller see it. */
static void press(sbsim_t *x, hal_button_t b)
{
    sim_hal_press(b);
    sbsim_run(x, x->t + 60u, NULL, NULL);
}

/* Press the joystick CONFIRM button (0.1 s). True if it was accepted. */
static bool confirm_press(sbsim_t *x)
{
    uint32_t before = x->sb.confirm.accepted;
    sim_hal_joystick_confirm(true);
    sbsim_run(x, x->t + 100u, NULL, NULL);
    sim_hal_joystick_confirm(false);
    sbsim_run(x, x->t + 60u, NULL, NULL);
    return x->sb.confirm.accepted > before;
}

/* One power-limit step up the real way: code from the boat, typed back,
 * joystick CONFIRM. */
static bool power_step(sbsim_t *x, uint8_t dir)
{
    uint16_t code;
    if (sb_power_code(&x->sb, dir, &code) != SB_OK) return false;
    if (sb_power_raise(&x->sb, dir, code) != SB_OK) return false;
    return confirm_press(x);
}

/* Raise to 100 % forward / 15 % reverse (tests that need speed). */
static void power_full(sbsim_t *x)
{
    for (int i = 0; i < 3; i++) power_step(x, 0u);
    for (int i = 0; i < 2; i++) power_step(x, 1u);
}

/* Waypoint at world position (north, east metres). */
static prog_wp_t wp_world(const sbsim_t *x, double north_m, double east_m, float kn, float radius_m,
                          wp_action_t act)
{
    prog_wp_t w;
    double lat, lon;
    world_t tmp = x->w;
    memset(&w, 0, sizeof w);
    tmp.north_m = north_m;
    tmp.east_m = east_m;
    world_latlon(&tmp, &lat, &lon);
    w.lat_e7 = (int32_t)lround(lat * 1e7);
    w.lon_e7 = (int32_t)lround(lon * 1e7);
    w.speed_kn = kn;
    w.radius_m = radius_m;
    w.action = (uint8_t)act;
    return w;
}

static void prog_make(program_t *p, const prog_wp_t *wps, int n, prog_end_t fin)
{
    program_clear(p);
    p->n = (uint16_t)n;
    p->final_action = (uint8_t)fin;
    for (int i = 0; i < n && i < (int)PROG_MAX_WP; i++) p->wp[i] = wps[i];
    program_seal(p);
}

/* Upload a program and start AUTO (PC start + joystick CONFIRM). */
static bool auto_go(sbsim_t *x, const prog_wp_t *wps, int n, prog_end_t fin)
{
    static program_t p;
    prog_error_t err;
    uint8_t bad;
    prog_make(&p, wps, n, fin);
    power_full(x);
    if (sb_program_set(&x->sb, &p, &err, &bad) != SB_OK) return false;
    if (sb_auto_start(&x->sb) != SB_OK) return false;
    return confirm_press(x) && sb_mode(&x->sb) == MODE_AUTO;
}

/* One waypoint `dist` m away on bearing `brg`, radius 25 m, stop there. */
static bool auto_to(sbsim_t *x, float dist, float brg, float kn)
{
    double r = brg * 3.14159265358979 / 180.0;
    prog_wp_t w = wp_world(x, x->w.north_m + dist * cos(r), x->w.east_m + dist * sin(r), kn, 25.0f, WP_ACT_STOP);
    return auto_go(x, &w, 1, PROG_END_STOP);
}

/* HDG LOCK button (locks the current heading). */
static bool hdg_lock(sbsim_t *x)
{
    if (!plim_raised(&x->sb.plim)) power_full(x);
    press(x, HAL_BTN_HDG_LOCK);
    return sb_mode(&x->sb) == MODE_HDG_LOCK;
}

/* AUTO no longer running a leg: why (arrival, or the fallback hold). */
static bool auto_running(const sbsim_t *x)
{
    return sb_mode(&x->sb) == MODE_AUTO && x->sb.autop.state == AUTO_RUN;
}

static mode_reason_t auto_end_why(const sbsim_t *x)
{
    if (sb_mode(&x->sb) == MODE_AUTO && x->sb.autop.state == AUTO_HOLD) {
        uint32_t a = x->sb.alarm_bits;
        if (a & SB_ALM_ARRIVAL_UNCERTAIN) return WHY_ARRIVAL_UNCERTAIN;
        if (a & SB_ALM_DR_LIMIT) return WHY_DR_LIMIT;
        if (a & SB_ALM_ARRIVED) return WHY_ARRIVED;
        return WHY_GPS_LOST;
    }
    return mode_log_get(&x->sb.modes, mode_log_size(&x->sb.modes) - 1u)->why;
}

static float hdg_err(float a, float b)
{
    float d = fmodf(a - b, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

/* Event times seen from the outside (outputs). -1 = not seen. */
typedef struct {
    uint32_t t0;            /* event start (request) */
    float    thr_before;
    float    t_thr_down;    /* throttle first lower than before */
    float    t_idle;        /* throttle first <= 1 % */
    float    t_neutral;     /* gear PWM -> neutral */
    float    t_gear;        /* gear PWM -> new gear */
    float    t_thr_open;    /* throttle > 1 % again after the new gear */
    float    t_steer_off;   /* autopilot no longer steering */
    uint16_t want_pwm;
    int      steer_after_off;  /* steering motor commands after steering off */
    bool     armed;
} events_t;

static void ev_reset(events_t *e, uint32_t t0, float thr, uint16_t want_pwm)
{
    e->t0 = t0;
    e->thr_before = thr;
    e->t_thr_down = e->t_idle = e->t_neutral = e->t_gear = e->t_thr_open = e->t_steer_off = -1.0f;
    e->want_pwm = want_pwm;
    e->steer_after_off = 0;
    e->armed = true;
}

static void ev_watch(sbsim_t *x, void *ctx)
{
    events_t *e = (events_t *)ctx;
    if (!e->armed || x->t < e->t0) return;
    float ts = (float)(x->t - e->t0) * 0.001f;
    float thr = sim_hal_throttle();
    uint16_t pwm = sim_hal_gear_pwm();
    if (e->t_thr_down < 0.0f && thr < e->thr_before - 0.01f) e->t_thr_down = ts;
    if (e->t_idle < 0.0f && thr <= 1.0f) e->t_idle = ts;
    if (e->t_neutral < 0.0f && pwm == x->sb.cfg.gear.pwm_n_us) e->t_neutral = ts;
    if (e->t_gear < 0.0f && e->t_neutral >= 0.0f && pwm == e->want_pwm) e->t_gear = ts;
    if (e->t_thr_open < 0.0f && e->t_gear >= 0.0f && thr > 1.0f) e->t_thr_open = ts;
    if (e->t_steer_off < 0.0f && x->sb.ap.mode == AP_MODE_STANDBY) e->t_steer_off = ts;
    if (e->t_steer_off >= 0.0f && sim_hal_motor_command() != 0.0f) e->steer_after_off++;
}

/* ------------------------------------------------------------ phase 2 -- */

static void phase2(void)
{
    char d[256];
    printf("\n=== Phase 2: throttle, gear, mode manager ===\n");

    /* 2.1 every transition against the table (incl. all illegal ones) */
    {
        mode_mgr_t m;
        int wrong = 0, illegal = 0, logged_ok = 0;
        mode_init(&m, 0);
        for (int f = 0; f < MODE_COUNT; f++) {
            for (int to = 0; to < MODE_COUNT; to++) {
                m.mode = (sb_mode_t)f;
                mode_verdict_t v = mode_request(&m, (sb_mode_t)to, WHY_TEST, true, 0);
                bool expect = mode_allowed((sb_mode_t)f, (sb_mode_t)to);
                if (!expect) illegal++;
                if ((v == MODE_OK) != expect) wrong++;
                if (!expect && m.mode != (sb_mode_t)f) wrong++;
                const mode_log_entry_t *e = mode_log_get(&m, mode_log_size(&m) - 1u);
                if (e && e->from == (sb_mode_t)f && e->to == (sb_mode_t)to &&
                    (e->verdict == MODE_OK) == expect) logged_ok++;
            }
        }
        snprintf(d, sizeof d, "%d pairs, %d illegal rejected, %d wrong, %d/%d logged correctly",
                 MODE_COUNT * MODE_COUNT, illegal, wrong, logged_ok, MODE_COUNT * MODE_COUNT);
        test_report("P2 mode table: every illegal transition rejected and logged",
                    wrong == 0 && logged_ok == MODE_COUNT * MODE_COUNT, d);
    }

    /* 2.2 preconditions: AUTO without GPS, HDG LOCK without compass */
    {
        sbsim_t x;
        sbsim_init(&x, 11u, 90.0f, 0.0f, NULL);
        x.w.gps_fail = true;
        sbsim_run(&x, 3000u, NULL, NULL);
        bool aut = sb_request_mode(&x.sb, MODE_AUTO, WHY_CONFIRM);
        sim_hal_fail_heading(true);
        sbsim_run(&x, 6000u, NULL, NULL);
        bool hdg = hdg_lock(&x);
        const mode_log_entry_t *e = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        snprintf(d, sizeof d, "AUTO without GPS %s, HDG LOCK without compass %s, last log: %s->%s %s",
                 aut ? "ACCEPTED" : "rejected", hdg ? "ACCEPTED" : "rejected",
                 mode_name(e->from), mode_name(e->to), e->verdict == MODE_REJECT_PRECOND ? "precondition" : "?");
        test_report("P2 mode preconditions (sensors) enforced and logged",
                    !aut && !hdg && e->verdict == MODE_REJECT_PRECOND, d);
    }

    /* 2.3 gear shift request at high throttle (F 80 % -> R) */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 12u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 80.0f);
        sbsim_run(&x, 40000u, NULL, NULL);
        float v_fwd = x.w.speed_kn;
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_r_us);
        sb_test_drive(&x.sb, true, GEAR_R, 80.0f);       /* asks for 80 % reverse too */
        sbsim_run(&x, 80000u, ev_watch, &ev);
        const gear_config_t *g = &x.sb.cfg.gear;
        bool order = ev.t_idle >= 0.0f && ev.t_neutral >= ev.t_idle + g->idle_wait_ms * 0.001f - 0.06f &&
                     ev.t_gear >= ev.t_neutral + g->neutral_wait_ms * 0.001f - 0.06f &&
                     ev.t_thr_open >= ev.t_gear + g->engage_wait_ms * 0.001f - 0.06f;
        snprintf(d, sizeof d, "at %.1f kn: idle %.2f s, neutral %.2f s, reverse %.2f s, throttle %.2f s; "
                 "under-load shifts %d, F/R without neutral %d, max reverse throttle %.1f %%, end speed %.1f kn",
                 v_fwd, ev.t_idle, ev.t_neutral, ev.t_gear, ev.t_thr_open, x.w.shift_under_load,
                 x.w.fr_without_neutral, x.w.max_thr_rev, x.w.speed_kn);
        test_report("P2 gear shift at high throttle: idle->wait->N->wait->R->wait->throttle",
                    order && x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0, d);
        snprintf(d, sizeof d, "max reverse throttle %.1f %% (limit %.0f), fastest throttle change %.1f %%/s (limits %.0f/%.0f)",
                 x.w.max_thr_rev, x.sb.cfg.throttle.max_rev_pct, x.w.max_thr_rate,
                 x.sb.cfg.throttle.rate_up_pct_s, x.sb.cfg.throttle.rate_down_pct_s);
        test_report("P2 reverse power limited, throttle rate-limited",
                    x.w.max_thr_rev <= x.sb.cfg.throttle.max_rev_pct + 0.01f &&
                    x.w.max_thr_rate <= x.sb.cfg.throttle.rate_down_pct_s + 0.5f && x.w.speed_kn < -1.0f, d);
    }

    /* 2.4 STANDBY at full throttle (compass OK) */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 13u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 100.0f);
        sbsim_run(&x, 60000u, NULL, NULL);
        float v0 = x.w.speed_kn, h0 = x.w.heading_deg, hmax = 0.0f;
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sim_hal_press(HAL_BTN_STANDBY);
        uint32_t t_end = x.t + 30000u;
        while (x.t < t_end) {
            sbsim_run(&x, x.t + 50u, ev_watch, &ev);
            float e = fabsf(hdg_err(x.w.heading_deg, h0));
            if (ev.t_steer_off < 0.0f && e > hmax) hmax = e;
        }
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        bool order = ev.t_thr_down >= 0.0f && ev.t_thr_down <= 0.1f &&
                     ev.t_idle >= 0.0f && ev.t_neutral > ev.t_idle && ev.t_steer_off >= ev.t_neutral;
        snprintf(d, sizeof d, "at %.1f kn: throttle down after %.2f s, idle %.2f s, neutral %.2f s, "
                 "steering off %.2f s (held heading within %.1f deg); under-load shifts %d; log: %s->%s (%s)",
                 v0, ev.t_thr_down, ev.t_idle, ev.t_neutral, ev.t_steer_off, hmax,
                 x.w.shift_under_load, mode_name(le->from), mode_name(le->to), mode_reason_name(le->why));
        test_report("P2 STANDBY at full throttle: idle + neutral first, then steering off",
                    order && x.w.shift_under_load == 0 && ev.steer_after_off == 0 &&
                    le->to == MODE_STANDBY && le->why == WHY_STANDBY_BUTTON, d);
    }

    /* 2.5 compass failure at full throttle */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 14u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        auto_to(&x, 5000.0f, 90.0f, 20.0f);              /* AUTO, raw full throttle */
        sb_test_drive(&x.sb, true, GEAR_F, 100.0f);
        sbsim_run(&x, 60000u, NULL, NULL);
        sim_hal_fail_heading(true);
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sbsim_run(&x, x.t + 30000u, ev_watch, &ev);
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        snprintf(d, sizeof d, "STANDBY (%s), throttle down after %.2f s, idle %.2f s, neutral %.2f s, "
                 "steering off %.2f s, alarm %s, under-load shifts %d",
                 mode_reason_name(le->why), ev.t_thr_down, ev.t_idle, ev.t_neutral, ev.t_steer_off,
                 sim_hal_alarm() ? "on" : "OFF", x.w.shift_under_load);
        test_report("P2 compass failure at full throttle (AUTO): STANDBY, idle + neutral, alarm",
                    le->why == WHY_COMPASS_FAULT && sb_mode(&x.sb) == MODE_STANDBY && sim_hal_alarm() &&
                    ev.t_thr_down >= 0.0f && ev.t_thr_down <= 2.1f && ev.t_neutral > ev.t_idle &&
                    x.w.shift_under_load == 0 && sim_hal_throttle() <= 0.0f, d);
    }

    /* 2.6 sim sanity: engine, speed fwd / rev, GPS 1 Hz with noise */
    {
        sbsim_t x;
        int msgs = 0;
        double err_sq = 0.0;
        float sog_err = 0.0f;
        sbsim_init(&x, 15u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 100.0f);
        uint32_t last_gps = 0;
        while (x.t < 60000u) {
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            if (x.sb.gps.last_ms != last_gps && x.sb.gps.valid) {
                last_gps = x.sb.gps.last_ms;
                msgs++;
                double n, e;
                world_from_latlon(x.sb.gps.last.lat_deg, x.sb.gps.last.lon_deg, &n, &e);
                err_sq += (n - x.w.north_m) * (n - x.w.north_m) + (e - x.w.east_m) * (e - x.w.east_m);
                float se = fabsf(x.sb.gps.last.sog_kn - world_sog_kn(&x.w));
                if (se > sog_err) sog_err = se;
            }
        }
        float vf = x.w.speed_kn;
        sb_test_drive(&x.sb, true, GEAR_R, 100.0f);
        sbsim_run(&x, 120000u, NULL, NULL);
        float vr = x.w.speed_kn;
        snprintf(d, sizeof d, "fwd 100 %% -> %.1f kn, rev (power limit max 15 %%) -> %.1f kn, GPS %d msgs in 59 s, "
                 "position noise %.1f m rms, worst SOG error %.2f kn",
                 vf, vr, msgs, sqrt(err_sq / (msgs ? msgs : 1)), sog_err);
        test_report("P2 sim: engine thrust, speed fwd/rev, GPS 1 Hz with noise",
                    vf > 19.0f && vr < -1.0f && vr > -4.0f && msgs >= 57 && msgs <= 61, d);
    }
}

/* ------------------------------------------------------------ phase 3 -- */

/* A scripted operator on the joystick (MANUAL): from time t (s after
 * start) steer = X (-1..+1 = full angle), thr + gear = Y (F: +thr,
 * R: -thr, N: centre). */
typedef enum { SEL_N = 0, SEL_F, SEL_R } sel_t;

typedef struct {
    float t;
    float steer, thr;
    sel_t gear;
} ctl_step_t;

typedef struct {
    const ctl_step_t *steps;
    int      n;
    uint32_t t0;
    /* watch */
    float est_max, est_min, ang_max, ang_min, v_max, v_min;
    float thr_in_neutral;       /* worst throttle while the stick is centred in Y */
    float hdg_min, hdg_max, hdg_unwrap, hdg_prev;
} script_t;

static float ctl_y(const ctl_step_t *c)
{
    return (c->gear == SEL_F) ? c->thr : (c->gear == SEL_R) ? -c->thr : 0.0f;
}

static void script_hook(sbsim_t *x, void *ctx)
{
    script_t *s = (script_t *)ctx;
    float ts = (float)(x->t - s->t0) * 0.001f;
    int k = -1;
    for (int i = 0; i < s->n; i++) if (ts >= s->steps[i].t) k = i;
    if (k >= 0) joy_set(s->steps[k].steer, ctl_y(&s->steps[k]), false, false);

    float est = x->sb.ap.est_angle, ang = x->w.rud.angle_deg;
    if (est > s->est_max) s->est_max = est;
    if (est < s->est_min) s->est_min = est;
    if (ang > s->ang_max) s->ang_max = ang;
    if (ang < s->ang_min) s->ang_min = ang;
    if (x->w.speed_kn > s->v_max) s->v_max = x->w.speed_kn;
    if (x->w.speed_kn < s->v_min) s->v_min = x->w.speed_kn;
    if (k >= 0 && s->steps[k].gear == SEL_N && gear_is(&x->sb.gear, GEAR_N) &&
        sim_hal_throttle() > s->thr_in_neutral)
        s->thr_in_neutral = sim_hal_throttle();
    s->hdg_unwrap += hdg_err(x->w.heading_deg, s->hdg_prev);
    s->hdg_prev = x->w.heading_deg;
    if (s->hdg_unwrap > s->hdg_max) s->hdg_max = s->hdg_unwrap;
    if (s->hdg_unwrap < s->hdg_min) s->hdg_min = s->hdg_unwrap;
}

static void script_start(script_t *s, sbsim_t *x, const ctl_step_t *steps, int n)
{
    memset(s, 0, sizeof *s);
    s->steps = steps;
    s->n = n;
    s->t0 = x->t;
    s->hdg_prev = x->w.heading_deg;
}

/* MANUAL with a joystick limited to `max_pct` throttle (old MANEUVER: 30 %). */
static sb_config_t cfg_manual(float max_pct)
{
    sb_config_t c;
    sb_default_config(&c);
    c.joy_max_throttle_pct = max_pct;
    return c;
}

static void phase3(void)
{
    char d[300];
    printf("\n=== Phase 3: MANUAL (joystick) ===\n");

    /* 3.1 docking-style sequence */
    {
        static const ctl_step_t dock[] = {
            {   0.0f,  0.0f, 1.0f, SEL_F },  /* ahead, stick full (-> 30 %) */
            {  15.0f,  1.0f, 1.0f, SEL_F },  /* full right */
            {  35.0f, -1.0f, 1.0f, SEL_F },  /* back left */
            {  60.0f,  0.0f, 0.0f, SEL_N },  /* neutral */
            {  70.0f,  0.0f, 1.0f, SEL_R },  /* astern, stick full */
            {  75.0f, -1.0f, 1.0f, SEL_R },  /* full left while reversing */
            {  97.0f,  1.0f, 1.0f, SEL_R },  /* back right */
            { 122.0f,  0.0f, 0.0f, SEL_N },
            { 130.0f,  0.0f, 0.3f, SEL_F },
        };
        sbsim_t x;
        script_t s;
        sb_config_t c = cfg_manual(30.0f);
        sbsim_init(&x, 21u, 45.0f, 0.0f, &c);
        sbsim_run(&x, 1000u, NULL, NULL);
        script_start(&s, &x, dock, (int)(sizeof dock / sizeof dock[0]));
        sbsim_run(&x, 2000u, script_hook, &s);
        bool ok_mode = sb_mode(&x.sb) == MODE_MANUAL;       /* the stick took control */
        sbsim_run(&x, 141000u, script_hook, &s);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, 142000u, NULL, NULL);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, 155000u, NULL, NULL);
        float lim = x.sb.cfg.steer.outboard.max_angle_deg;
        snprintf(d, sizeof d, "max throttle fwd %.1f %% / rev %.1f %% (power limit %.0f / 5), speed %+.1f..%+.1f kn, "
                 "outboard est %+.1f..%+.1f deg, true %+.1f..%+.1f deg (stops +-35), end-stop push %.1f s, "
                 "heading swing %+.0f..%+.0f deg, throttle in N %.1f %%, under-load %d, F/R w/o N %d",
                 x.w.max_thr_fwd, x.w.max_thr_rev, c.joy_max_throttle_pct, s.v_min, s.v_max,
                 s.est_min, s.est_max, s.ang_min, s.ang_max, x.w.rud.stall_s, s.hdg_min, s.hdg_max,
                 s.thr_in_neutral, x.w.shift_under_load, x.w.fr_without_neutral);
        /* MANUAL after an override: startup power limit 30 % / 5 % */
        bool ok = ok_mode && x.w.max_thr_fwd <= x.sb.cfg.plim.fwd_steps[0] + 0.01f &&
                  x.w.max_thr_rev <= x.sb.cfg.plim.rev_start + 0.01f &&
                  s.v_max > 3.0f && s.v_min < -0.5f &&
                  s.est_max >= lim - 1.0f && s.est_min <= -lim + 1.0f &&     /* angle loop deadband */
                  s.est_max <= lim + 0.01f && s.est_min >= -lim - 0.01f &&
                  x.w.rud.stall_s == 0.0f && s.thr_in_neutral <= 0.0f &&
                  x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0 &&
                  (s.hdg_max - s.hdg_min) > 30.0f &&
                  sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f &&
                  gear_is(&x.sb.gear, GEAR_N) && sim_hal_motor_command() == 0.0f;
        test_report("P3 docking sequence: fwd, neutral, rev, full-angle turns, throttle <= limit", ok, d);
    }

    /* 3.2 fast gear toggling by the user (every 200 ms, half stick) */
    {
        static ctl_step_t tog[52];
        static const sel_t seq[3] = { SEL_F, SEL_R, SEL_N };
        for (int i = 0; i < 50; i++) {
            tog[i].t = 0.2f * (float)i;
            tog[i].steer = 0.0f;
            tog[i].thr = 0.5f;
            tog[i].gear = seq[i % 3];
        }
        tog[50].t = 10.0f; tog[50].steer = 0.0f; tog[50].thr = 1.0f; tog[50].gear = SEL_R;
        tog[51].t = 30.0f; tog[51].steer = 0.0f; tog[51].thr = 1.0f; tog[51].gear = SEL_R;
        sbsim_t x;
        script_t s;
        sb_config_t c = cfg_manual(30.0f);
        sbsim_init(&x, 22u, 0.0f, 0.0f, &c);
        sbsim_run(&x, 1000u, NULL, NULL);
        script_start(&s, &x, tog, 52);
        float thr_toggling = 0.0f;
        int engage_changes = 0;
        gear_t last = x.w.engaged;
        while (x.t < s.t0 + 10000u) {
            sbsim_run(&x, x.t + 50u, script_hook, &s);
            if (sim_hal_throttle() > thr_toggling) thr_toggling = sim_hal_throttle();
            if (x.w.engaged != last) { engage_changes++; last = x.w.engaged; }
        }
        sbsim_run(&x, s.t0 + 25000u, script_hook, &s);
        snprintf(d, sizeof d, "50 changes in 10 s: max throttle while toggling %.1f %%, gearbox engagements %d, "
                 "under-load %d, F/R w/o N %d; afterwards mode %s, gear %s, throttle %.1f %%, speed %+.1f kn",
                 thr_toggling, engage_changes, x.w.shift_under_load, x.w.fr_without_neutral,
                 mode_name(sb_mode(&x.sb)), gear_name(x.w.engaged), sim_hal_throttle(), x.w.speed_kn);
        test_report("P3 fast gear toggling: always via neutral at idle, ends in last request",
                    thr_toggling <= 1.0f && x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0 &&
                    x.w.engaged == GEAR_R && gear_is(&x.sb.gear, GEAR_R) && sb_mode(&x.sb) == MODE_MANUAL &&
                    sim_hal_throttle() <= x.sb.cfg.throttle.max_rev_pct + 0.01f && x.w.speed_kn < -1.0f, d);
    }

    /* 3.3 STANDBY while reversing with the stick full astern: idle +
     *     neutral, steering off, the held stick is ignored (disarmed) */
    {
        static const ctl_step_t rev[] = { { 0.0f, -1.0f, 1.0f, SEL_R } };
        sbsim_t x;
        script_t s;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 23u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        script_start(&s, &x, rev, 1);
        sbsim_run(&x, 20000u, script_hook, &s);
        bool was_manual = sb_mode(&x.sb) == MODE_MANUAL;
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sim_hal_press(HAL_BTN_STANDBY);
        sbsim_run(&x, 40000u, ev_watch, &ev);        /* operator still holds the stick */
        snprintf(d, sizeof d, "throttle down after %.2f s, idle %.2f s, neutral %.2f s, steering off %.2f s, "
                 "motor after off %d, under-load %d, mode now %s (stick still held), speed %+.1f kn",
                 ev.t_thr_down, ev.t_idle, ev.t_neutral, ev.t_steer_off, ev.steer_after_off,
                 x.w.shift_under_load, mode_name(sb_mode(&x.sb)), x.w.speed_kn);
        test_report("P3 STANDBY while reversing: idle + neutral, steering off, held stick ignored",
                    was_manual && ev.t_thr_down >= 0.0f && ev.t_thr_down <= 0.1f && ev.t_neutral > ev.t_idle &&
                    ev.steer_after_off == 0 && x.w.shift_under_load == 0 &&
                    sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f, d);
    }

    /* 3.4 compass failure in MANUAL -> alarm only, stay in MANUAL
     *     (manual steering doesn't need the compass) */
    {
        static const ctl_step_t seq[] = {
            {  0.0f, 0.0f, 1.0f, SEL_F },
            { 25.0f, 1.0f, 1.0f, SEL_F },     /* after the fault: steer right */
            { 30.0f, 0.0f, 1.0f, SEL_F },
        };
        sbsim_t x;
        script_t s;
        sb_config_t c = cfg_manual(30.0f);
        sbsim_init(&x, 24u, 0.0f, 0.0f, &c);
        sbsim_run(&x, 1000u, NULL, NULL);
        script_start(&s, &x, seq, 3);
        sbsim_run(&x, 20000u, script_hook, &s);
        sim_hal_fail_heading(true);
        sbsim_run(&x, 24000u, script_hook, &s);
        float est_before = x.sb.ap.est_angle;
        s.est_max = s.ang_max = -99.0f;               /* watch from here */
        sbsim_run(&x, 40000u, script_hook, &s);
        sb_mode_t after = sb_mode(&x.sb);
        snprintf(d, sizeof d, "mode after fault %s, alarm %s, throttle %.1f %%, gear %s, "
                 "stick right after the fault: estimate %+.1f -> up to %+.1f deg, true up to %+.1f deg",
                 mode_name(after), sim_hal_alarm() ? "on" : "OFF", sim_hal_throttle(), gear_name(x.w.engaged),
                 est_before, s.est_max, s.ang_max);
        test_report("P3 compass failure in MANUAL: alarm only, manual control keeps working",
                    after == MODE_MANUAL && sim_hal_alarm() && x.w.engaged == GEAR_F &&
                    sim_hal_throttle() > 29.0f && s.est_max > est_before + 10.0f &&
                    s.ang_max > 10.0f && x.w.shift_under_load == 0, d);
    }
}

/* ------------------------------------------------------------ phase 4 -- */

/* Watch an AUTO run along a straight line. Segments are the program legs
 * (several legs), or a sea-state schedule in time (one long leg). */
typedef struct {
    const float *at_s;          /* sea schedule times (s after t0) */
    const float *sea;           /* sea state from that time */
    int      n_sea;
    int      n_legs;
    uint32_t t0;
    int      seg;               /* current segment */
    uint32_t seg_t0;            /* when it started */
    /* per segment */
    float seg_err_sum[16]; int seg_err_n[16];
    float seg_over[16], seg_hold[16], seg_vmin[16], seg_vmax[16];
    /* whole run */
    float limit_excess;         /* commanded steering beyond the table limit */
    float true_excess;          /* true outboard beyond limit + 3 deg (settled) */
    float hold_max;
    int   gear_changes;
    gear_t last_gear;
    uint32_t last_transition_ms, min_gap_ms;
    uint32_t transitions_seen;
} cruise_watch_t;

static void cruise_hook(sbsim_t *x, void *ctx)
{
    cruise_watch_t *c = (cruise_watch_t *)ctx;
    float ts = (float)(x->t - c->t0) * 0.001f;
    if (c->n_legs > 1) {
        int g = x->sb.autop.wp;
        if (g != c->seg && g < 16) { c->seg = g; c->seg_t0 = x->t; }
    } else {
        while (c->seg + 1 < c->n_sea && ts >= c->at_s[c->seg + 1]) {
            c->seg++;
            c->seg_t0 = x->t;
            x->w.sea_state = c->sea[c->seg];
        }
    }
    int g = c->seg;
    float seg_t = (float)(x->t - c->seg_t0) * 0.001f;
    float sog = world_sog_kn(&x->w);
    float set = x->sb.cruise.target_kn;
    if (!auto_running(x)) return;
    bool last = (uint16_t)(x->sb.autop.wp + 1u) >= x->sb.autop.prog.n;
    if (last && x->sb.autop.leg.dist_m < x->sb.cfg.slow_radius_m + 150.0f) return;   /* final approach */
    if (seg_t > 40.0f) {                    /* settled part of the segment */
        c->seg_err_sum[g] += fabsf(sog - set);
        c->seg_err_n[g]++;
        float he = fabsf(hdg_err(x->w.heading_deg, x->sb.ap.target_heading));
        if (he > c->seg_hold[g]) c->seg_hold[g] = he;
        if (he > c->hold_max) c->hold_max = he;
    }
    if (seg_t > 5.0f && sog - x->sb.cruise.set_kn > c->seg_over[g]) c->seg_over[g] = sog - x->sb.cruise.set_kn;
    if (seg_t > 40.0f && (c->seg_vmin[g] == 0.0f || sog < c->seg_vmin[g])) c->seg_vmin[g] = sog;
    if (seg_t > 40.0f && sog > c->seg_vmax[g]) c->seg_vmax[g] = sog;

    if (x->sb.ap.steer_limit_deg > 0.0f) {
        float ex = fabsf(x->sb.ap.rudder_target) - x->sb.ap.steer_limit_deg;
        if (ex > c->limit_excess) c->limit_excess = ex;
        float lim_true = sb_steer_limit_for(&x->sb.cfg, sog) + 3.0f;
        if (seg_t > 20.0f && fabsf(x->w.rud.angle_deg) - lim_true > c->true_excess)
            c->true_excess = fabsf(x->w.rud.angle_deg) - lim_true;
    }
    if (x->w.engaged != c->last_gear) { c->gear_changes++; c->last_gear = x->w.engaged; }
    if (x->sb.nerv.transitions != c->transitions_seen) {
        uint32_t gap = x->t - c->last_transition_ms;
        if (c->transitions_seen > 0u && (c->min_gap_ms == 0u || gap < c->min_gap_ms)) c->min_gap_ms = gap;
        c->transitions_seen = x->sb.nerv.transitions;
        c->last_transition_ms = x->t;
    }
}

/* AUTO due east: legs of `leg_s` seconds at each speed in spd[] (all
 * CONTINUE, straight line), or one long leg at spd[0] (n_legs = 1) with the
 * sea schedule. */
static void cruise_run(sbsim_t *x, cruise_watch_t *c, uint32_t seed, const float *spd, int n_legs, float leg_s,
                       const float *at, const float *sea, int n_sea, uint32_t dur_ms)
{
    prog_wp_t wps[16];
    sbsim_init(x, seed, 90.0f, 0.0f, NULL);
    x->w.sea_state = sea[0];
    sbsim_run(x, 3000u, NULL, NULL);
    double e = x->w.east_m;
    for (int i = 0; i < n_legs; i++) {
        float len = (n_legs > 1) ? spd[i] * 0.514444f * leg_s : spd[0] * 0.514444f * (dur_ms * 0.001f) + 2000.0f;
        e += len;
        wps[i] = wp_world(x, x->w.north_m, e, spd[i], 25.0f, WP_ACT_CONTINUE);
    }
    memset(c, 0, sizeof *c);
    c->at_s = at; c->sea = sea; c->n_sea = n_sea; c->n_legs = n_legs;
    auto_go(x, wps, n_legs, PROG_END_STOP);
    c->t0 = c->seg_t0 = x->t;
    c->last_gear = x->w.engaged;
    sbsim_run(x, x->t + dur_ms, cruise_hook, c);
}

static void phase4(void)
{
    char d[400];
    printf("\n=== Phase 4: AUTO speed control, steering by speed ===\n");

    /* 4.1 legs at 3 -> 20 kn, sea state 1 */
    {
        static const float spd[] = { 3, 6, 10, 15, 20 };
        static const float at[]  = { 0 };
        static const float sea[] = { 1 };
        sbsim_t x;
        cruise_watch_t c;
        cruise_run(&x, &c, 41u, spd, 5, 90.0f, at, sea, 1, 470000u);
        bool ok = true;
        int len = 0;
        len += snprintf(d + len, sizeof d - (size_t)len, "leg speed/err/overshoot/hold:");
        for (int i = 0; i < 5; i++) {
            float err = c.seg_err_n[i] ? c.seg_err_sum[i] / (float)c.seg_err_n[i] : 99.0f;
            len += snprintf(d + len, sizeof d - (size_t)len, " %.0fkn %.2f/%.2f/%.1f", spd[i], err,
                            c.seg_over[i], c.seg_hold[i]);
            /* (in AUTO the target itself moves with the cross-track correction) */
            if (err > 0.5f || c.seg_over[i] > 1.5f || c.seg_hold[i] > 12.0f) ok = false;
        }
        snprintf(d + len, sizeof d - (size_t)len, "; steering beyond limit: cmd %.2f, true %.2f deg; "
                 "throttle rate max %.0f %%/s; gear changes after F %d; legs done %u",
                 c.limit_excess, c.true_excess, x.w.max_thr_rate, c.gear_changes - 1, x.sb.autop.legs_done);
        ok = ok && c.limit_excess <= 0.01f && c.true_excess <= 0.0f && x.w.max_thr_rate <= 40.5f &&
             c.gear_changes == 1 && x.w.shift_under_load == 0 && x.sb.autop.legs_done >= 4u;
        test_report("P4 AUTO legs 3->20 kn: speed control per leg, steering limit by speed", ok, d);
    }

    /* 4.2 sea state 1 -> 4 at 18 kn, then calm again */
    {
        static const float at[]  = { 0, 180, 360, 540, 1140 };
        static const float spd[] = { 18 };
        static const float sea[] = { 1, 2, 3, 4, 1 };
        sbsim_t x;
        cruise_watch_t c;
        cruise_run(&x, &c, 42u, spd, 1, 0.0f, at, sea, 5, 2100000u);
        snprintf(d, sizeof d, "speed by sea state: ss1 %.1f-%.1f, ss2 %.1f-%.1f, ss3 %.1f-%.1f, ss4 %.1f-%.1f, "
                 "calm again %.1f-%.1f kn (end %.1f); course error max ss3 %.1f, ss4 %.1f deg; "
                 "rough-water episodes %u, changes %u, shortest gap %.0f s",
                 c.seg_vmin[0], c.seg_vmax[0], c.seg_vmin[1], c.seg_vmax[1], c.seg_vmin[2], c.seg_vmax[2],
                 c.seg_vmin[3], c.seg_vmax[3], c.seg_vmin[4], c.seg_vmax[4], world_sog_kn(&x.w),
                 c.seg_hold[2], c.seg_hold[3], x.sb.nerv.episodes, x.sb.nerv.transitions,
                 c.min_gap_ms * 0.001f);
        bool ok = c.seg_vmin[0] > 17.0f && c.seg_vmin[1] > 17.0f &&         /* no reaction in ss1-2 */
                  x.sb.nerv.episodes >= 1u && c.seg_vmin[3] <= 14.5f &&     /* slowed down in ss4 */
                  world_sog_kn(&x.w) > 17.0f &&                               /* recovered */
                  x.sb.nerv.transitions <= 4u && auto_running(&x);
        test_report("P4 sea state 1->4->1: slows down when rough, recovers when calm", ok, d);
    }

    /* 4.3 no oscillation: 30 min at a borderline sea state */
    {
        static const float at[]  = { 0 };
        static const float spd[] = { 18 };
        static const float sea[] = { 3.5f };
        sbsim_t x;
        cruise_watch_t c;
        cruise_run(&x, &c, 43u, spd, 1, 0.0f, at, sea, 1, 1800000u);
        snprintf(d, sizeof d, "30 min at sea state 3.5, 18 kn leg: episodes %u, nervous/calm changes %u, "
                 "shortest time between changes %.0f s, speed %.1f-%.1f kn",
                 x.sb.nerv.episodes, x.sb.nerv.transitions, c.min_gap_ms * 0.001f, c.seg_vmin[0], c.seg_vmax[0]);
        bool ok = x.sb.nerv.transitions <= 6u &&
                  (x.sb.nerv.transitions < 2u || c.min_gap_ms >= 30000u);
        test_report("P4 no oscillation between nervous and calm", ok, d);
    }

    /* 4.5 GPS loss in HDG LOCK: alarm, still steering, throttle stays with the user */
    {
        sbsim_t x;
        sbsim_init(&x, 45u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        hdg_lock(&x);
        joy_set(0.0f, 0.6f, false, false);          /* throttle on the stick */
        sbsim_run(&x, 80000u, NULL, NULL);
        float thr0 = sim_hal_throttle(), v0 = world_sog_kn(&x.w);
        x.w.gps_fail = true;
        float thr_min = 100.0f, hold = 0.0f;
        while (x.t < 200000u) {
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            if (x.t > 90000u && sim_hal_throttle() < thr_min) thr_min = sim_hal_throttle();
            float he = fabsf(hdg_err(x.w.heading_deg, x.sb.ap.target_heading));
            if (x.t > 110000u && he > hold) hold = he;
        }
        snprintf(d, sizeof d, "before: %.1f %% at %.1f kn; after GPS loss: throttle >= %.1f %% (stick), speed %.1f kn, "
                 "alarm %s (GPS lost %s), mode %s, course error max %.1f deg",
                 thr0, v0, thr_min, world_sog_kn(&x.w), sim_hal_alarm() ? "on" : "OFF",
                 (x.sb.alarm_bits & SB_ALM_GPS_LOST) ? "yes" : "NO", mode_name(sb_mode(&x.sb)), hold);
        /* GPS lost drops the power limit: the stick still drives, below 30 % */
        test_report("P4 GPS loss in HDG LOCK: alarm, still steering, throttle stays with the user",
                    thr_min >= fminf(thr0, x.sb.cfg.plim.fwd_steps[0]) - 0.5f && sim_hal_alarm() && (x.sb.alarm_bits & SB_ALM_GPS_LOST) &&
                    sb_mode(&x.sb) == MODE_HDG_LOCK && hold < 10.0f, d);
    }

    /* 4.6 compass failure in AUTO at 20 kn: safe stop */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 46u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        auto_to(&x, 6000.0f, 90.0f, 20.0f);
        sbsim_run(&x, 120000u, NULL, NULL);
        float v0 = world_sog_kn(&x.w);
        sim_hal_fail_heading(true);
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sbsim_run(&x, x.t + 30000u, ev_watch, &ev);
        snprintf(d, sizeof d, "at %.1f kn: throttle down after %.2f s, idle %.2f s, neutral %.2f s, alarm %s, "
                 "under-load %d", v0, ev.t_thr_down, ev.t_idle, ev.t_neutral, sim_hal_alarm() ? "on" : "OFF",
                 x.w.shift_under_load);
        test_report("P4 compass failure in AUTO: STANDBY, idle + neutral first, alarm",
                    v0 > 18.0f && sb_mode(&x.sb) == MODE_STANDBY && ev.t_thr_down >= 0.0f && ev.t_thr_down <= 2.1f &&
                    ev.t_neutral > ev.t_idle && sim_hal_alarm() && x.w.shift_under_load == 0, d);
    }
}

/* ------------------------------------------------------------ phase 5 -- */

typedef struct {
    double wp_n, wp_e;          /* waypoint, world metres */
    float  track_deg, length;
    float  settle_m;            /* xte measured after this far along */
    float  xte_max, xte_sq;
    int    xte_n;
    float  crab_seen;           /* largest |heading - COG| estimate */
    float  hdg_min_seen, hdg_max_seen;
    bool   crossed_north;
    float  t_arrive;            /* AUTO stopped running the leg (s), -1 */
    mode_reason_t why;
    sb_mode_t mode_after;
    float  stop_dist;           /* distance to the waypoint when stopped */
    uint32_t t0;
    float  prev_hdg;
} nav_watch_t;

static void nav_hook(sbsim_t *x, void *ctx)
{
    nav_watch_t *w = (nav_watch_t *)ctx;
    float tn = (float)(w->wp_n / w->length), te = (float)(w->wp_e / w->length);
    float n = (float)x->w.north_m, e = (float)x->w.east_m;
    float along = n * tn + e * te;
    float xte = e * tn - n * te;
    if (auto_running(x)) {
        if (along > w->settle_m && along < w->length - 150.0f) {
            if (fabsf(xte) > w->xte_max) w->xte_max = fabsf(xte);
            w->xte_sq += xte * xte;
            w->xte_n++;
        }
        if (fabsf(x->sb.autop.crab_deg) > w->crab_seen) w->crab_seen = fabsf(x->sb.autop.crab_deg);
        float h = x->w.heading_deg;
        if ((w->prev_hdg > 300.0f && h < 60.0f) || (w->prev_hdg < 60.0f && h > 300.0f)) w->crossed_north = true;
        w->prev_hdg = h;
    } else if (w->t_arrive < 0.0f) {
        w->t_arrive = (float)(x->t - w->t0) * 0.001f;
        w->why = auto_end_why(x);
        w->mode_after = sb_mode(&x->sb);
    }
    double dn = w->wp_n - x->w.north_m, de = w->wp_e - x->w.east_m;
    w->stop_dist = (float)sqrt(dn * dn + de * de);
}

/* AUTO run with one waypoint: start at the world origin on `heading`,
 * waypoint at dist/bearing, radius 25 m, stop there. */
static void nav_run(sbsim_t *x, nav_watch_t *w, uint32_t seed, float heading, float dist, float bearing,
                    float speed, float sea, float cur_e, float cur_n, uint32_t gps_fail_after_m)
{
    sbsim_init(x, seed, heading, 0.0f, NULL);
    x->w.sea_state = sea;
    x->w.current_e = cur_e;
    x->w.current_n = cur_n;
    sbsim_run(x, 3000u, NULL, NULL);
    memset(w, 0, sizeof *w);
    w->wp_n = dist * cos(bearing * 3.14159265358979 / 180.0);
    w->wp_e = dist * sin(bearing * 3.14159265358979 / 180.0);
    w->track_deg = bearing;
    w->length = dist;
    w->settle_m = fmaxf(250.0f, 0.15f * dist);
    w->t_arrive = -1.0f;
    w->prev_hdg = heading;
    double lat, lon, n0 = x->w.north_m, e0 = x->w.east_m;
    world_t tmp = x->w;
    tmp.north_m = n0 + w->wp_n;
    tmp.east_m = e0 + w->wp_e;
    w->wp_n += n0;
    w->wp_e += e0;
    world_latlon(&tmp, &lat, &lon);
    (void)lat; (void)lon;
    prog_wp_t wp = wp_world(x, w->wp_n, w->wp_e, speed, 25.0f, WP_ACT_STOP);
    auto_go(x, &wp, 1, PROG_END_STOP);
    w->t0 = x->t;
    uint32_t t_end = x->t + (uint32_t)(dist / (speed * 0.5144f) * 1000.0f * 2.0f) + 120000u;
    while (x->t < t_end) {
        sbsim_run(x, x->t + 50u, nav_hook, w);
        if (gps_fail_after_m && !x->w.gps_fail) {
            double dn = x->w.north_m - n0, de = x->w.east_m - e0;
            if (sqrt(dn * dn + de * de) >= gps_fail_after_m) x->w.gps_fail = true;
        }
        if (w->t_arrive >= 0.0f && x->t > w->t0 + (uint32_t)(w->t_arrive * 1000.0f) + 40000u) break;
    }
}

static void phase5(void)
{
    char d[400];
    printf("\n=== Phase 5: AUTO line following (one waypoint) ===\n");

    struct { const char *name; float hdg, dist, brg, spd, sea, ce, cn, xmax; } cases[] = {
        { "P5 AUTO 500 m waypoint (12 kn, sea 1)",               90.0f,  500.0f,  45.0f, 12.0f, 1.0f, 0.0f,  0.0f, 15.0f },
        { "P5 AUTO 2 km waypoint (18 kn, sea 2)",               180.0f, 2000.0f, 200.0f, 18.0f, 2.0f, 0.0f,  0.0f, 15.0f },
        { "P5 AUTO 2 km with 1 m/s side current (10 kn)",        90.0f, 2000.0f,  90.0f, 10.0f, 1.0f, 0.0f, -1.0f, 20.0f },
        { "P5 AUTO crossing north (track 2 deg, current from E)", 330.0f, 1000.0f,   2.0f, 10.0f, 1.0f, -0.8f, 0.0f, 20.0f },
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        sbsim_t x;
        nav_watch_t w;
        nav_run(&x, &w, 51u + i, cases[i].hdg, cases[i].dist, cases[i].brg, cases[i].spd, cases[i].sea,
                cases[i].ce, cases[i].cn, 0u);
        float rms = w.xte_n ? sqrtf(w.xte_sq / (float)w.xte_n) : 0.0f;
        snprintf(d, sizeof d, "cross-track (true) max %.1f m, rms %.1f m; drift angle %.1f deg; "
                 "arrived after %.0f s (%s -> %s), stopped %.0f m from the waypoint; alarm %s, gear %s, throttle %.1f %%%s",
                 w.xte_max, rms, w.crab_seen, w.t_arrive, mode_reason_name(w.why), mode_name(w.mode_after),
                 w.stop_dist, sim_hal_alarm() ? "on" : "OFF", gear_name(x.w.engaged), sim_hal_throttle(),
                 i == 3 ? (w.crossed_north ? ", heading crossed north" : ", heading did NOT cross north") : "");
        bool ok = w.xte_n > 0 && w.xte_max <= cases[i].xmax && rms <= 6.0f &&
                  w.why == WHY_ARRIVED && w.mode_after == MODE_STANDBY && w.stop_dist < 60.0f &&
                  sim_hal_alarm() && x.w.engaged == GEAR_N && sim_hal_throttle() <= 0.0f &&
                  x.w.shift_under_load == 0 && (i != 3 || w.crossed_north);
        test_report(cases[i].name, ok, d);
    }

    /* GPS loss halfway */
    {
        sbsim_t x;
        nav_watch_t w;
        nav_run(&x, &w, 56u, 120.0f, 2000.0f, 120.0f, 14.0f, 1.0f, 0.0f, 0.0f, 1000u);
        snprintf(d, sizeof d, "GPS lost at 1 km: leg ended after %.0f s (%s, %s), alarm %s, gear %s, "
                 "throttle now %.1f %%, speed %.1f kn, heading %.0f (track 120)",
                 w.t_arrive, mode_reason_name(w.why), mode_name(sb_mode(&x.sb)), sim_hal_alarm() ? "on" : "OFF",
                 gear_name(x.w.engaged), sim_hal_throttle(), world_sog_kn(&x.w), x.w.heading_deg);
        /* AUTO carries on by dead reckoning, then (uncertain at the
         * waypoint) holds the heading slowly - not straight to a hold */
        test_report("P5 AUTO GPS loss halfway: continues by dead reckoning, alarm",
                    w.why != WHY_GPS_LOST && w.t_arrive > 60.0f && sim_hal_alarm() &&
                    (x.sb.alarm_bits & SB_ALM_GPS_LOST) && x.w.engaged == GEAR_F &&
                    fabsf(hdg_err(x.w.heading_deg, 120.0f)) < 15.0f, d);
    }

    /* AUTO needs a program */
    {
        sbsim_t x;
        sbsim_init(&x, 57u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        sb_result_t r = sb_auto_start(&x.sb);
        bool cf = confirm_press(&x);
        snprintf(d, sizeof d, "start without a program -> %d (no program = %d), CONFIRM accepted %s, mode %s",
                 (int)r, (int)SB_ERR_NO_PROGRAM, cf ? "YES" : "no", mode_name(sb_mode(&x.sb)));
        test_report("P5 AUTO refused without a program", r == SB_ERR_NO_PROGRAM && !cf &&
                    sb_mode(&x.sb) == MODE_STANDBY, d);
    }
}

/* ------------------------------------------------------------ phase 6 -- */

/* A "true" boat for training tests (values at the slow training speed). */
typedef struct {
    float speed, gap, travel_r, travel_l, K, T, trim_deg;
    bool  start_left;
} tset_t;

static uint32_t s6_rng;
static unsigned g_world_sensors;    /* 1 = motor current sense, 2 = limit switches */
static float u6(float lo, float hi)
{
    s6_rng ^= s6_rng << 13; s6_rng ^= s6_rng >> 17; s6_rng ^= s6_rng << 5;
    return lo + (hi - lo) * (float)(s6_rng >> 8) / 16777216.0f;
}

static tset_t tset_random(void)
{
    tset_t s;
    s.speed = u6(1.5f, 5.0f);
    s.gap = u6(0.5f, 3.5f);
    s.travel_r = u6(25.0f, 38.0f);
    s.travel_l = s.travel_r + u6(-4.0f, 4.0f);
    s.K = u6(0.08f, 0.20f);
    s.T = u6(1.5f, 6.0f);
    s.trim_deg = u6(-2.0f, 2.0f);
    s.start_left = u6(0.0f, 1.0f) < 0.5f;
    return s;
}

static calib_data_t tset_calib(const tset_t *s)
{
    calib_data_t c;
    memset(&c, 0, sizeof c);
    float sp = fabsf(s->speed);
    c.gain_dps_per_ms = s->K * sp / 1000.0f;
    c.time_const_s = s->T;
    c.gap_ms = c.gap_rl_ms = c.gap_lr_ms = s->gap / sp * 1000.0f;
    c.travel_right_ms = s->travel_r / sp * 1000.0f;
    c.travel_left_ms = s->travel_l / sp * 1000.0f;
    c.trim_dps = s->K * s->trim_deg;
    c.fit_r2 = c.confidence = 1.0f;
    c.full_angle_deg = s->travel_r + s->travel_l;
    calib_seal(&c);
    return c;
}

static void world_tset(world_t *w, const tset_t *s)
{
    rudder_init_outboard(&w->rud, s->speed, s->speed, s->gap);
    w->rud.max_angle_deg = s->travel_r;
    w->rud.max_angle_neg_deg = s->travel_l;
    if (s->start_left) w->rud.drive_deg = -0.5f * s->gap;
    w->K_override = s->K;
    w->T_override = s->T;
    w->trim_deg = s->trim_deg;
}

typedef struct {
    bool         reached_result, accepted;
    tr_result_t  result;
    sb_mode_t    mode_end;
    mode_reason_t why_end;
    calib_data_t calib;
    sb_config_t  cfg_after;     /* sb config incl. calibrated steering */
    float        duration_s, max_hdg_phase, stall_total, stall_longest;
    float        thr_run_min, thr_run_max;  /* throttle while calibrating */
    bool         gear_f_whole_run;
    float        v_train;
    int          motor_after_stop;
    float        t_thr_down, t_neutral;     /* after an abort request */
    int          under_load;
    bool         alarm;
    training_t   dbg_train;     /* diagnostics */
} train6_t;

static void sb_training(const tset_t *ts, uint8_t ss, uint32_t seed, uint32_t abort_at_s,
                        uint32_t compass_fail_at_s, int confirm /* 1 accept, 0 reject */, train6_t *o)
{
    sbsim_t x;
    sb_config_t cfg;
    events_t ev;
    memset(o, 0, sizeof *o);
    memset(&ev, 0, sizeof ev);
    sb_default_config(&cfg);
    cfg.steer.sea_state = ss;
    sbsim_init(&x, seed, 90.0f, 0.0f, &cfg);
    x.w.sea_state = ss;
    world_tset(&x.w, ts);
    x.w.current_sensor = (g_world_sensors & 1u) != 0u;
    x.w.limit_switches = (g_world_sensors & 2u) != 0u;
    sbsim_run(&x, 3000u, NULL, NULL);
    sb_request_mode(&x.sb, MODE_TRAINING, WHY_USER);
    uint32_t t0 = x.t, stopped_at = 0u;
    o->thr_run_min = 100.0f;
    o->gear_f_whole_run = true;
    bool req_sent = false;
    while (x.t < t0 + 25u * 60u * 1000u) {
        sbsim_run(&x, x.t + 50u, req_sent ? ev_watch : NULL, &ev);
        uint32_t ts_s = (x.t - t0) / 1000u;
        if (!req_sent && ((abort_at_s && ts_s >= abort_at_s) || (compass_fail_at_s && ts_s >= compass_fail_at_s))) {
            ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
            if (abort_at_s) sim_hal_press(HAL_BTN_STANDBY_AUTO); else sim_hal_fail_heading(true);
            req_sent = true;
        }
        if (sb_mode(&x.sb) == MODE_TRAINING && x.sb.train.stage == TS_RUN) {
            float thr = sim_hal_throttle();
            if (thr < o->thr_run_min) o->thr_run_min = thr;
            if (thr > o->thr_run_max) o->thr_run_max = thr;
            if (x.w.engaged != GEAR_F) o->gear_f_whole_run = false;
        }
        if (sb_mode(&x.sb) == MODE_TRAINING && x.sb.train.stage == TS_RESULT && !o->reached_result) {
            o->reached_result = true;
            sbsim_run(&x, x.t + 5000u, NULL, NULL);      /* engine settles to idle + neutral */
            o->calib = x.sb.ap.train.calib;
            o->v_train = x.sb.train.sog_n ? (float)(x.sb.train.sog_sum / x.sb.train.sog_n) : 0.0f;
            if (confirm >= 0) o->accepted = sb_confirm_training(&x.sb, confirm == 1, ts->travel_r + ts->travel_l);
            o->calib = x.sb.ap.train.calib;
        }
        if (sb_mode(&x.sb) == MODE_STANDBY) {
            if (!stopped_at) stopped_at = x.t;
            if (sim_hal_motor_command() != 0.0f && x.sb.ap.mode == AP_MODE_STANDBY) o->motor_after_stop++;
            if (x.t - stopped_at > 20000u) break;
        }
    }
    o->result = x.sb.ap.train.result;
    o->mode_end = sb_mode(&x.sb);
    const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
    o->why_end = le->why;
    o->cfg_after = x.sb.cfg;
    o->cfg_after.steer = x.sb.ap.cfg;
    o->duration_s = (float)((stopped_at ? stopped_at : x.t) - t0) * 0.001f;
    o->max_hdg_phase = x.sb.ap.train.max_heading_change;
    o->stall_total = x.w.rud.stall_s;
    o->stall_longest = x.w.rud.stall_max_s;
    o->t_thr_down = ev.t_thr_down;
    o->t_neutral = ev.t_neutral;
    o->under_load = x.w.shift_under_load;
    o->alarm = sim_hal_alarm();
    o->dbg_train = x.sb.ap.train;
}

/* sb config with a calibration applied (as accepting it would). */
static sb_config_t cfg_with_calib(const calib_data_t *c, uint8_t ss, float v_train)
{
    sb_config_t cfg;
    autopilot_t tmp;
    sb_default_config(&cfg);
    cfg.steer.sea_state = ss;
    autopilot_init(&tmp, &cfg.steer);
    autopilot_apply_calibration(&tmp, c);
    cfg.steer = tmp.cfg;
    if (v_train > 0.5f) cfg.gain_ref_kn = v_train;
    return cfg;
}

typedef struct { float hold_max, hold_sq; int n; float over; uint32_t t0; float target; } course6_t;

static void course6_hook(sbsim_t *x, void *ctx)
{
    course6_t *c = (course6_t *)ctx;
    float ts = (float)(x->t - c->t0) * 0.001f;
    if (ts >= 60.0f && ts < 60.1f) {
        sim_hal_press(HAL_BTN_PLUS_10);
    }
    if (ts >= 60.3f && ts < 60.4f) sim_hal_press(HAL_BTN_PLUS_10);
    if (ts >= 60.6f && ts < 60.7f) sim_hal_press(HAL_BTN_PLUS_10);
    float e = hdg_err(x->w.heading_deg, x->sb.ap.target_heading);
    if ((ts > 20.0f && ts < 60.0f) || ts > 180.0f) {    /* weak boats turn slowly: hold measured from 180 s */
        if (fabsf(e) > c->hold_max) c->hold_max = fabsf(e);
        c->hold_sq += e * e;
        c->n++;
    }
    if (ts > 60.0f && ts < 180.0f && e > c->over) c->over = e;
}

static course6_t course6(const tset_t *ts, uint8_t ss, uint32_t seed, const sb_config_t *cfg, float speed)
{
    sbsim_t x;
    course6_t c;
    memset(&c, 0, sizeof c);
    sbsim_init(&x, seed, 90.0f, 0.0f, cfg);
    x.w.sea_state = ss;
    world_tset(&x.w, ts);
    sbsim_run(&x, 3000u, NULL, NULL);
    (void)speed;                                    /* the training throttle gives the training speed */
    hdg_lock(&x);
    sb_test_drive(&x.sb, true, GEAR_F, x.sb.cfg.training_throttle_pct);
    c.t0 = x.t;
    sbsim_run(&x, x.t + 300000u, course6_hook, &c);
    return c;
}

static float pct6(float a, float b) { return (b != 0.0f) ? 100.0f * (a - b) / b : 0.0f; }

static void phase6(void)
{
    char d[500];
    printf("\n=== Phase 6: TRAINING (in the mode manager, throttle fixed, forward gear) ===\n");
    printf("%-9s | %7s %7s %7s %7s %7s %7s %7s %8s | %5s %4s %4s %5s\n", "boat/sea", "gain%", "T%",
           "gap%", "travR%", "travL%", "speed%", "gapdeg%", "trim d/s", "R2", "hdg", "push", "time");

    s6_rng = 2024u;
    tset_t sets[5];
    for (int i = 0; i < 5; i++) sets[i] = tset_random();

    int trained = 0, accurate = 0, total = 0, worse = 0, gap_off = 0, beyond_stop = 0;
    float worst_gain = 0, worst_travel = 0, worst_speed = 0, worst_gapdeg = 0, worst_T = 0;
    float hold_l_max = 0, hold_t_max = 0, over_l_max = 0, over_t_max = 0, push_max = 0, hdg_max = 0;
    bool throttle_fixed = true, gear_ok = true;
    char course_lines[10][160];

    for (int i = 0; i < 5; i++) {
        for (uint8_t ss = 1; ss <= 2; ss++) {
            const tset_t *s = &sets[i];
            train6_t o;
            total++;
            sb_training(s, ss, 600u + (uint32_t)i * 10u + ss, 0u, 0u, 1, &o);
            if (!o.accepted) {
                const training_t *tt = &o.dbg_train;
                printf("%d / %u     | TRAINING FAILED: %s (mode %s, %s) - phase est: gain %.3f T %.2f "
                       "gap %.0f trav %.0f/%.0f; fit: T %.2f gap %.0f trav %.0f/%.0f R2 %.3f\n", i + 1, ss,
                       training_result_text(o.result), mode_name(o.mode_end), mode_reason_name(o.why_end),
                       tt->gain * 1000.0f, tt->tconst, tt->gap_fit, tt->travel_r, tt->travel_l, tt->q[0], tt->q[1],
                       tt->q[2], tt->q[3], tt->sst > 0.0 ? 1.0 - tt->best_cost / tt->sst : 0.0);
                snprintf(course_lines[total - 1], sizeof course_lines[0], "%d / %u  | (no calibration)", i + 1, ss);
                continue;
            }
            trained++;
            calib_data_t tc = tset_calib(s);
            const calib_data_t *c = &o.calib;
            sb_config_t lc = o.cfg_after;
            sb_config_t tcfg = cfg_with_calib(&tc, ss, o.v_train);
            float e_gain = pct6(c->gain_dps_per_ms, tc.gain_dps_per_ms);
            float e_T = pct6(c->time_const_s, tc.time_const_s);
            float e_gap = pct6(c->gap_ms, tc.gap_ms);
            float e_tr = pct6(c->travel_right_ms, tc.travel_right_ms);
            float e_tl = pct6(c->travel_left_ms, tc.travel_left_ms);
            float e_sp = pct6(lc.steer.outboard.turn_speed_deg_s, tcfg.steer.outboard.turn_speed_deg_s);
            float e_gd = pct6(lc.steer.outboard.gap_deg, tcfg.steer.outboard.gap_deg);
            float gap_abs = fabsf(lc.steer.outboard.gap_deg - tcfg.steer.outboard.gap_deg);
            printf("%d / %u     | %+6.1f %+6.1f %+6.1f %+6.1f %+6.1f %+6.1f %+6.1f %+8.3f | %5.3f %3.0f %4.1f %4.0fs\n",
                   i + 1, ss, e_gain, e_T, e_gap, e_tr, e_tl, e_sp, e_gd, c->trim_dps - tc.trim_dps,
                   c->fit_r2, o.max_hdg_phase, o.stall_total, o.duration_s);
            /* safety relevant: gain, travel (max angle), turn speed (estimate) */
            if (fabsf(e_gain) <= 15 && fabsf(e_tr) <= 15 && fabsf(e_tl) <= 15 && fabsf(e_sp) <= 15) accurate++;
            /* gap: reported, not failed (wave-limited, see report) */
            if (fabsf(e_gd) > 15 && gap_abs > 0.5f) gap_off++;
            if (lc.steer.outboard.max_angle_deg >= fminf(s->travel_r, s->travel_l)) beyond_stop++;
            worst_gain = fmaxf(worst_gain, fabsf(e_gain));
            worst_T = fmaxf(worst_T, fabsf(e_T));
            worst_travel = fmaxf(worst_travel, fmaxf(fabsf(e_tr), fabsf(e_tl)));
            worst_speed = fmaxf(worst_speed, fabsf(e_sp));
            worst_gapdeg = fmaxf(worst_gapdeg, gap_abs);
            push_max = fmaxf(push_max, o.stall_total);
            hdg_max = fmaxf(hdg_max, o.max_hdg_phase);
            if (o.thr_run_max - o.thr_run_min > 0.01f ||
                fabsf(o.thr_run_max - lc.training_throttle_pct) > 0.01f) throttle_fixed = false;
            if (!o.gear_f_whole_run) gear_ok = false;

            /* course keeping at the training speed: learned vs true values */
            uint32_t cs = 700u + (uint32_t)i * 10u + ss;
            sb_config_t dc;
            sb_default_config(&dc);
            dc.steer.sea_state = ss;
            course6_t cd = course6(s, ss, cs, &dc, o.v_train);
            course6_t cl = course6(s, ss, cs, &lc, o.v_train);
            course6_t ct = course6(s, ss, cs, &tcfg, o.v_train);
            float rl = cl.n ? sqrtf(cl.hold_sq / cl.n) : 0, rt = ct.n ? sqrtf(ct.hold_sq / ct.n) : 0;
            float rd = cd.n ? sqrtf(cd.hold_sq / cd.n) : 0;
            snprintf(course_lines[total - 1], sizeof course_lines[0],
                     "%d / %u  | %4.1f kn | default %5.2f/%4.2f/%4.1f | learned %5.2f/%4.2f/%4.1f | true %5.2f/%4.2f/%4.1f",
                     i + 1, ss, o.v_train, cd.hold_max, rd, cd.over, cl.hold_max, rl, cl.over,
                     ct.hold_max, rt, ct.over);
            hold_l_max = fmaxf(hold_l_max, cl.hold_max);
            hold_t_max = fmaxf(hold_t_max, ct.hold_max);
            over_l_max = fmaxf(over_l_max, cl.over);
            over_t_max = fmaxf(over_t_max, ct.over);
            if (cl.hold_max > ct.hold_max * 1.3f + 1.0f || cl.over > ct.over * 1.3f + 2.0f) worse++;
        }
    }
    printf("\nCourse keeping in HDG LOCK at the training speed (hold max / rms / overshoot of a 30 deg turn, deg):\n");
    for (int k = 0; k < total; k++) printf("  %s\n", course_lines[k]);

    snprintf(d, sizeof d, "%d/%d trained and accepted; gain/travel/turn speed within 15 %%: %d/%d; "
             "max angle beyond the real end stop: %d; gap off by > 15 %% and > 0.5 deg: %d (not failed); worst errors: "
             "gain %.1f %%, time const %.1f %%, travel %.1f %%, turn speed %.1f %%, gap %.2f deg",
             trained, total, accurate, total, beyond_stop, gap_off, worst_gain, worst_T, worst_travel, worst_speed,
             worst_gapdeg);
    test_report("P6 training 5 random boats x sea state 1 + 2: learned vs true", trained == total &&
                accurate == total && beyond_stop == 0, d);
    snprintf(d, sizeof d, "worst course error learned %.1f deg vs true-value %.1f; overshoot %.1f vs %.1f; "
             "learned clearly worse in %d of %d", hold_l_max, hold_t_max, over_l_max, over_t_max, worse, trained);
    test_report("P6 course keeping with learned values as good as with true values",
                trained == total && worse == 0 && over_l_max < 10.0f, d);
    snprintf(d, sizeof d, "throttle constant at %.0f %% while calibrating: %s, forward gear the whole time: %s, "
             "max heading change in a phase %.0f deg (limit %.0f), end-stop push max %.1f s total",
             sets[0].speed > 0 ? 20.0f : 0.0f, throttle_fixed ? "yes" : "NO", gear_ok ? "yes" : "NO",
             hdg_max, TR_MAX_PHASE_HEADING_DEG, push_max);
    test_report("P6 training runs at fixed slow throttle in forward gear, within limits",
                throttle_fixed && gear_ok && hdg_max <= TR_MAX_PHASE_HEADING_DEG, d);

    /* safety: Standby during training */
    {
        train6_t o;
        sb_training(&sets[0], 2, 650u, 120u, 0u, -1, &o);
        snprintf(d, sizeof d, "%s, mode %s (%s), throttle down after %.2f s, neutral %.2f s, "
                 "steering motor after stop %d, under-load %d, alarm %s",
                 training_result_text(o.result), mode_name(o.mode_end), mode_reason_name(o.why_end),
                 o.t_thr_down, o.t_neutral, o.motor_after_stop, o.under_load, o.alarm ? "on" : "off");
        test_report("P6 Standby during training: abort, idle + neutral, steering off",
                    o.result == TR_ABORT_USER && o.mode_end == MODE_STANDBY && o.t_thr_down >= 0.0f &&
                    o.t_thr_down <= 0.1f && o.t_neutral > 0.0f && o.motor_after_stop == 0 && o.under_load == 0, d);
    }
    /* safety: compass failure during training */
    {
        train6_t o;
        sb_training(&sets[0], 2, 651u, 0u, 150u, -1, &o);
        snprintf(d, sizeof d, "%s, mode %s (%s), throttle down after %.2f s, neutral %.2f s, alarm %s, under-load %d",
                 training_result_text(o.result), mode_name(o.mode_end), mode_reason_name(o.why_end),
                 o.t_thr_down, o.t_neutral, o.alarm ? "on" : "OFF", o.under_load);
        test_report("P6 compass failure during training: abort, STANDBY, idle + neutral, alarm",
                    o.result == TR_ABORT_SENSOR && o.mode_end == MODE_STANDBY && o.alarm &&
                    o.t_neutral > 0.0f && o.under_load == 0, d);
    }
    /* safety: training's own limits and plausibility end in a safe stop */
    {
        tset_t fast = sets[0], gap = sets[0];
        fast.K = 0.7f; fast.speed = 6.0f; fast.T = 1.5f;
        gap.travel_r = gap.travel_l = 15.0f; gap.gap = 8.0f;
        train6_t a, b;
        sb_training(&fast, 1, 652u, 0u, 0u, -1, &a);
        sb_training(&gap, 1, 653u, 0u, 0u, -1, &b);
        snprintf(d, sizeof d, "boat turning 20 deg/s at lock: %s -> %s (%s); gap 27 %% of travel: %s -> %s (%s); "
                 "under-load %d/%d", training_result_text(a.result), mode_name(a.mode_end),
                 mode_reason_name(a.why_end), training_result_text(b.result), mode_name(b.mode_end),
                 mode_reason_name(b.why_end), a.under_load, b.under_load);
        test_report("P6 limits / implausible results: training aborts into a safe stop",
                    a.result != TR_OK && a.mode_end == MODE_STANDBY && a.why_end == WHY_TRAINING_ABORT &&
                    b.result != TR_OK && b.mode_end == MODE_STANDBY && b.why_end == WHY_TRAINING_ABORT &&
                    a.under_load == 0 && b.under_load == 0, d);
    }
    /* reject keeps the old settings; accept applies + saves (checksum) */
    {
        train6_t r;
        sb_training(&sets[1], 1, 611u, 0u, 0u, 0, &r);     /* same run as boat 2 / sea 1 above */
        sb_config_t def;
        sb_default_config(&def);
        bool unchanged = r.reached_result && !r.accepted &&
                         r.cfg_after.steer.kp == def.steer.kp &&
                         r.cfg_after.steer.outboard.turn_speed_deg_s == def.steer.outboard.turn_speed_deg_s;
        calib_data_t back;
        train6_t a;
        sb_training(&sets[1], 1, 611u, 0u, 0u, 1, &a);
        bool saved = a.accepted && calib_load(&back) && back.checksum == a.calib.checksum;
        snprintf(d, sizeof d, "reject (training %s): settings unchanged %s, mode %s; accept: applied %s, saved + checksum OK %s, mode %s",
                 training_result_text(r.result),
                 unchanged ? "yes" : "NO", mode_name(r.mode_end), a.accepted ? "yes" : "NO",
                 saved ? "yes" : "NO", mode_name(a.mode_end));
        test_report("P6 confirmation: reject keeps settings, accept applies + saves",
                    unchanged && saved && r.mode_end == MODE_STANDBY && a.mode_end == MODE_STANDBY, d);
    }
    /* training only from STANDBY */
    {
        sbsim_t x;
        sbsim_init(&x, 656u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        hdg_lock(&x);
        bool from_hdg = sb_request_mode(&x.sb, MODE_TRAINING, WHY_PC);
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        test_report("P6 TRAINING refused unless in STANDBY (table), and logged",
                    sb_mode(&x.sb) == MODE_HDG_LOCK && !from_hdg && le->verdict == MODE_REJECT_TABLE &&
                    le->to == MODE_TRAINING, "");
    }
}

/* -------------------------------------------------------------- fixes -- */

static bool g_stop_trace;

/* Compass lost at 20 kn with the outboard turned 15 deg. */
typedef struct {
    uint32_t t0;
    float hdg_prev, hdg_unwrap, hdg_abs_max;
    float t_fault, t_idle, t_neutral, t_off;
    float angle_at_fault, angle_end;
    int   motor_after_off;
    bool  done;
} stopturn_t;

static void stopturn_hook(sbsim_t *x, void *ctx)
{
    stopturn_t *s = (stopturn_t *)ctx;
    float ts = (float)(x->t - s->t0) * 0.001f;
    s->hdg_unwrap += hdg_err(x->w.heading_deg, s->hdg_prev);
    s->hdg_prev = x->w.heading_deg;
    if (fabsf(s->hdg_unwrap) > s->hdg_abs_max) s->hdg_abs_max = fabsf(s->hdg_unwrap);
    if (s->t_fault < 0.0f && sb_mode(&x->sb) == MODE_STANDBY) {
        s->t_fault = ts;
        s->angle_at_fault = x->w.rud.angle_deg;
    }
    if (s->t_fault >= 0.0f) {
        if (s->t_idle < 0.0f && sim_hal_throttle() <= 1.0f) s->t_idle = ts;
        if (s->t_neutral < 0.0f && sim_hal_gear_pwm() == x->sb.cfg.gear.pwm_n_us) s->t_neutral = ts;
        if (s->t_off < 0.0f && ts > s->t_fault + 0.1f && x->sb.stop_steer == STOP_STEER_NONE &&
            x->sb.ap.mode == AP_MODE_STANDBY) s->t_off = ts;
        if (s->t_off >= 0.0f && sim_hal_motor_command() != 0.0f) s->motor_after_off++;
    }
    s->angle_end = x->w.rud.angle_deg;
    if (g_stop_trace && ((x->t - s->t0) / 50u) % 5u == 0u)
        printf("ST %5.2f mode %s ap %d stop %d est %6.2f slack %5.2f true %6.2f tgt %6.2f mot %+.0f "
               "yaw %6.2f thr %5.1f spd %5.1f\n", ts,
               mode_name(sb_mode(&x->sb)), x->sb.ap.mode, x->sb.stop_steer, x->sb.ap.est_angle, x->sb.ap.slack_deg,
               x->w.rud.angle_deg, x->sb.ap.rudder_target, sim_hal_motor_command(), x->w.yaw_dps, sim_hal_throttle(), x->w.speed_kn);
}

static stopturn_t stop_turn(uint8_t fault_steer, bool gyro_fails, uint32_t seed)
{
    sbsim_t x;
    sb_config_t c;
    stopturn_t s;
    memset(&s, 0, sizeof s);
    s.t_fault = s.t_idle = s.t_neutral = s.t_off = -1.0f;
    sb_default_config(&c);
    c.fault_steer = fault_steer;
    sbsim_init(&x, seed, 90.0f, 0.0f, &c);
    x.w.sea_state = 1.0f;
    sbsim_run(&x, 3000u, NULL, NULL);
    auto_to(&x, 8000.0f, 90.0f, 20.0f);
    sbsim_run(&x, 120000u, NULL, NULL);

    /* the boat is in a hard turn: outboard at 15 deg right ... */
    x.w.rud.angle_deg = 15.0f;
    x.w.rud.drive_deg = 15.0f + 0.5f * x.w.rud.gap_deg;
    x.sb.ap.est_angle = 15.0f;
    x.sb.ap.slack_deg = 0.5f * x.sb.ap.cfg.outboard.gap_deg;
    x.sb.ap.rudder_target = 15.0f;
    /* ... when the compass (and maybe the gyro) fails */
    sim_hal_fail_heading(true);
    if (gyro_fails) sim_hal_set_gyro(false, 0.0f, 0.0f);
    s.t0 = x.t;
    s.hdg_prev = x.w.heading_deg;
    while (x.t < s.t0 + 90000u) {
        sbsim_run(&x, x.t + 50u, stopturn_hook, &s);
        if (s.t_off >= 0.0f && fabsf(x.w.speed_kn) < 0.5f) break;
    }
    return s;
}

/* Gear servo sticks in F while the operator asks for reverse. */
typedef struct {
    float thr_max_after;        /* throttle after the stick */
    float v_end;                /* boat speed at the end (+ = forward) */
    bool  alarm, fault, cmds_frozen, refused;
    float t_fault;              /* s after the reverse request */
    sb_mode_t mode_end;
    mode_reason_t why_end;
} stuck_t;

static stuck_t stuck_servo(bool switch_fitted, uint32_t seed)
{
    static const ctl_step_t fwd[] = { { 0.0f, 0.0f, 1.0f, SEL_F } };
    sbsim_t x;
    script_t s;
    stuck_t r;
    memset(&r, 0, sizeof r);
    r.t_fault = -1.0f;
    sb_config_t c = cfg_manual(30.0f);
    sbsim_init(&x, seed, 0.0f, 0.0f, &c);
    x.w.neutral_switch = switch_fitted;
    sbsim_run(&x, 1000u, NULL, NULL);
    script_start(&s, &x, fwd, 1);
    sbsim_run(&x, 20000u, script_hook, &s);
    x.w.servo_stuck = true;                         /* servo dies in forward */
    joy_set(0.0f, 0.0f, false, false);              /* operator wants reverse (via centre) */
    sbsim_run(&x, x.t + 50u, NULL, NULL);
    joy_set(0.0f, -1.0f, false, false);
    uint32_t t0 = x.t;
    uint16_t pwm_at_fault = 0u;
    r.cmds_frozen = true;
    while (x.t < t0 + 40000u) {
        sbsim_run(&x, x.t + 50u, NULL, NULL);
        if (sim_hal_throttle() > r.thr_max_after && x.t > t0 + 2000u) r.thr_max_after = sim_hal_throttle();
        if (gear_fault(&x.sb.gear) && r.t_fault < 0.0f) {
            r.t_fault = (float)(x.t - t0) * 0.001f;
            pwm_at_fault = sim_hal_gear_pwm();
        }
        if (r.t_fault >= 0.0f && sim_hal_gear_pwm() != pwm_at_fault) r.cmds_frozen = false;
        if (x.t == t0 + 20000u) joy_set(0.0f, 0.0f, false, false);
        if (x.t == t0 + 20050u) joy_set(0.0f, 1.0f, false, false);   /* tries again */
    }
    joy_set(0.0f, 0.0f, false, false);
    sbsim_run(&x, x.t + 500u, NULL, NULL);
    r.refused = !hdg_lock(&x);                      /* driving modes refused */
    r.v_end = x.w.speed_kn;
    r.alarm = sim_hal_alarm();
    r.fault = gear_fault(&x.sb.gear);
    r.mode_end = sb_mode(&x.sb);
    r.why_end = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u)->why;
    return r;
}

/* MANUAL, operator holds full right; the true motor is faster than
 * configured, so the outboard reaches the end stop before the estimate
 * says so. */
typedef struct { float push_s, est_end, true_end; uint32_t hits; } push_t;

static push_t maneuver_push(bool current_sensor, uint32_t seed)
{
    static const ctl_step_t right[] = { { 0.0f, 1.0f, 0.5f, SEL_F } };
    sbsim_t x;
    script_t s;
    push_t r;
    sbsim_init(&x, seed, 0.0f, 0.0f, NULL);
    rudder_init_outboard(&x.w.rud, 4.5f, 4.5f, 2.0f);      /* config says 3 deg/s */
    x.w.rud.max_angle_deg = x.w.rud.max_angle_neg_deg = 30.0f;
    x.w.current_sensor = current_sensor;
    sbsim_run(&x, 1000u, NULL, NULL);
    script_start(&s, &x, right, 1);
    sbsim_run(&x, 20000u, script_hook, &s);
    r.push_s = x.w.rud.stall_s;
    r.est_end = x.sb.ap.est_angle;
    r.true_end = x.w.rud.angle_deg;
    r.hits = x.sb.ap.stops_hit;
    return r;
}

static void fixes(void)
{
    char d[400];
    printf("\n=== Fixes ===\n");

    /* Fix 4: motor current / limit switches stop the push at the end stop */
    {
        s6_rng = 2024u;
        tset_t boat1 = tset_random();               /* boat 1 of the phase 6 table */
        train6_t t0, tc, tl;
        g_world_sensors = 0u; sb_training(&boat1, 1, 601u, 0u, 0u, 1, &t0);
        g_world_sensors = 1u; sb_training(&boat1, 1, 601u, 0u, 0u, 1, &tc);
        g_world_sensors = 2u; sb_training(&boat1, 1, 601u, 0u, 0u, 1, &tl);
        g_world_sensors = 0u;
        calib_data_t tv = tset_calib(&boat1);
        printf("Training, boat 1, sea state 1: end-stop push total / longest, travel R / L error:\n");
        const train6_t *rr[3] = { &t0, &tc, &tl };
        const char *nm[3] = { "before (yaw only)   ", "after, motor current", "after, limit switch " };
        for (int k = 0; k < 3; k++)
            printf("  %s %5.1f s / %.2f s, travel %+5.1f %% / %+5.1f %%, accepted %s\n", nm[k],
                   rr[k]->stall_total, rr[k]->stall_longest,
                   pct6(rr[k]->calib.travel_right_ms, tv.travel_right_ms),
                   pct6(rr[k]->calib.travel_left_ms, tv.travel_left_ms), rr[k]->accepted ? "yes" : "NO");
        snprintf(d, sizeof d, "push before %.1f s -> current %.1f s, limit switch %.1f s; travel errors with "
                 "current %+.1f/%+.1f %%, with switch %+.1f/%+.1f %%",
                 t0.stall_total, tc.stall_total, tl.stall_total,
                 pct6(tc.calib.travel_right_ms, tv.travel_right_ms), pct6(tc.calib.travel_left_ms, tv.travel_left_ms),
                 pct6(tl.calib.travel_right_ms, tv.travel_right_ms), pct6(tl.calib.travel_left_ms, tv.travel_left_ms));
        test_report("F4 training: end-stop push time well below before (current / switch)",
                    tc.accepted && tl.accepted && tc.stall_total < 0.25f * t0.stall_total &&
                    tl.stall_total < 0.25f * t0.stall_total &&
                    fabsf(pct6(tc.calib.travel_right_ms, tv.travel_right_ms)) < 15.0f &&
                    fabsf(pct6(tc.calib.travel_left_ms, tv.travel_left_ms)) < 15.0f, d);

        push_t pb = maneuver_push(false, 85u), pa = maneuver_push(true, 85u);
        printf("MANUAL full right, motor 50 %% faster than configured (end stop at 30 deg):\n");
        printf("  before: pushed %.1f s, estimate %.1f deg (true %.1f)\n", pb.push_s, pb.est_end, pb.true_end);
        printf("  after:  pushed %.1f s, estimate %.1f deg (true %.1f), end stop detected %u x\n",
               pa.push_s, pa.est_end, pa.true_end, pa.hits);
        snprintf(d, sizeof d, "push before %.1f s -> after %.2f s; estimate after %.1f deg (true %.1f)",
                 pb.push_s, pa.push_s, pa.est_end, pa.true_end);
        test_report("F4 normal steering: stops pushing as soon as the current rises",
                    pa.push_s < 0.5f && pa.push_s < 0.25f * pb.push_s && pa.hits >= 1u, d);
    }

    /* Fix 3: neutral switch catches a stuck gear servo */
    {
        stuck_t b = stuck_servo(false, 83u);
        stuck_t a = stuck_servo(true, 83u);
        printf("Gear servo stuck in F, operator selects R:\n");
        printf("  before (no switch): throttle up to %.1f %% while still in F, boat %+.1f kn, alarm %s\n",
               b.thr_max_after, b.v_end, b.alarm ? "on" : "off");
        printf("  after (switch):     fault after %.1f s, throttle up to %.1f %%, boat %+.1f kn, alarm %s, "
               "gear commands frozen %s, mode %s (%s), HDG LOCK refused %s\n",
               a.t_fault, a.thr_max_after, a.v_end, a.alarm ? "on" : "off", a.cmds_frozen ? "yes" : "NO",
               mode_name(a.mode_end), mode_reason_name(a.why_end), a.refused ? "yes" : "NO");
        snprintf(d, sizeof d, "before: throttle %.1f %% in the wrong gear, no alarm; after: fault %.1f s after "
                 "neutral was asked, throttle %.1f %%, alarm %s, gear commands frozen %s, driving modes refused %s",
                 b.thr_max_after, a.t_fault, a.thr_max_after, a.alarm ? "on" : "OFF",
                 a.cmds_frozen ? "yes" : "NO", a.refused ? "yes" : "NO");
        test_report("F3 stuck gear servo: neutral not confirmed -> alarm, idle, no gear commands",
                    a.fault && a.alarm && a.thr_max_after <= 0.0f && a.cmds_frozen && a.refused &&
                    a.t_fault > 0.0f && a.t_fault < 5.0f && a.mode_end == MODE_MANUAL, d);

        /* with a working servo the switch must not cause false alarms */
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 84u, 90.0f, 0.0f, NULL);
        x.w.neutral_switch = true;
        sbsim_run(&x, 1000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 80.0f);
        sbsim_run(&x, 40000u, NULL, NULL);
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_r_us);
        sb_test_drive(&x.sb, true, GEAR_R, 80.0f);
        sbsim_run(&x, 80000u, ev_watch, &ev);
        snprintf(d, sizeof d, "F 80 %% -> R with switch: idle %.2f s, neutral %.2f s, reverse %.2f s, throttle %.2f s, "
                 "fault %s, alarm %s, speed %+.1f kn",
                 ev.t_idle, ev.t_neutral, ev.t_gear, ev.t_thr_open, gear_fault(&x.sb.gear) ? "YES" : "no",
                 sim_hal_alarm() ? "ON" : "off", x.w.speed_kn);
        test_report("F3 neutral switch fitted, servo OK: normal shifting, no false alarm",
                    !gear_fault(&x.sb.gear) && !sim_hal_alarm() && ev.t_thr_open > 0.0f && x.w.speed_kn < -1.0f &&
                    x.w.shift_under_load == 0, d);
    }

    /* Fix 1: compass lost while moving -> gyro hold / centre during the stop */
    {
        stopturn_t b = stop_turn(SB_FAULT_STEER_OFF, false, 81u);
        stopturn_t g = stop_turn(SB_FAULT_STEER_GYRO, false, 81u);
        g_stop_trace = getenv("STOPTRACE") != NULL;
        stopturn_t c = stop_turn(SB_FAULT_STEER_GYRO, true, 81u);
        g_stop_trace = false;
        printf("Compass lost at 20 kn, outboard 15 deg (heading change until the boat stops):\n");
        printf("  before (steering off at once): %6.0f deg   outboard at fault %.1f, at end %.1f deg\n",
               b.hdg_unwrap, b.angle_at_fault, b.angle_end);
        printf("  after, gyro hold:              %6.0f deg   idle %.1f s, neutral %.1f s, steering off %.1f s, outboard at end %.1f deg\n",
               g.hdg_unwrap, g.t_idle, g.t_neutral, g.t_off, g.angle_end);
        printf("  after, gyro failed too:        %6.0f deg   centred and off at %.1f s, outboard at end %.1f deg\n",
               c.hdg_unwrap, c.t_off, c.angle_end);
        snprintf(d, sizeof d, "heading change before %.0f deg -> gyro hold %.0f deg; steering off at %.1f s "
                 "(neutral %.1f s); motor after off %d",
                 b.hdg_unwrap, g.hdg_unwrap, g.t_off, g.t_neutral, g.motor_after_off);
        test_report("F1 compass lost at 20 kn: gyro hold until idle + neutral, then off",
                    fabsf(g.hdg_unwrap) < 0.5f * fabsf(b.hdg_unwrap) && g.t_off >= g.t_neutral &&
                    g.t_neutral > g.t_idle && g.motor_after_off == 0, d);
        snprintf(d, sizeof d, "heading change before %.0f deg -> centring %.0f deg; outboard at end %.1f deg "
                 "(before %.1f); steering off at %.1f s; motor after off %d",
                 b.hdg_unwrap, c.hdg_unwrap, c.angle_end, b.angle_end, c.t_off, c.motor_after_off);
        test_report("F1 compass and gyro lost at 20 kn: outboard to centre, then off",
                    fabsf(c.hdg_unwrap) < fabsf(b.hdg_unwrap) && fabsf(c.angle_end) < 3.0f &&
                    c.t_off > 0.0f && c.motor_after_off == 0, d);
    }
}

/* --------------------------------------------------- GPS-denied NAV -- */

typedef enum { GF_NONE = 0, GF_LOSS, GF_DRAG, GF_JUMP, GF_FLAG, GF_RETURN } gfault_t;

typedef struct {
    /* results */
    float t_detect;             /* fault start -> GPS no longer trusted, s (-1 = never) */
    dr_reason_t why;
    uint32_t suspects_before;   /* integrity alarms before the fault (false alarms) */
    float est_err_fault;        /* worst |estimate - true| while the fault was on */
    float xte_true_max;         /* worst true cross-track error after 300 m */
    float end_err, end_unc;     /* estimate error / uncertainty when the leg ended */
    float ratio_max;            /* worst error / uncertainty while in DR */
    mode_reason_t end_why;
    float end_t;
    float t_recover;            /* fault cleared -> GPS trusted again, s (-1 = never) */
    float steer_step_max;       /* largest course-to-steer change in one 50 ms step after recovery */
    float steer_step_normal;    /* same, normal GPS navigation before the fault */
    float err_after_recover;    /* estimate error 30 s after recovery */
    uint32_t suspects_total;
} mission_t;

static mission_t gps_mission(gfault_t fault, float cur_e, float cur_n, bool paddlewheel, uint32_t seed)
{
    const float brg = 60.0f, dist = 3000.0f, speed = 12.0f;
    sbsim_t x;
    mission_t m;
    memset(&m, 0, sizeof m);
    m.t_detect = m.t_recover = -1.0f;
    m.end_t = -1.0f;

    sbsim_init(&x, seed, brg, 0.0f, NULL);
    x.w.sea_state = 1.0f;
    x.w.current_e = cur_e;
    x.w.current_n = cur_n;
    x.w.speed_sensor = paddlewheel;
    sbsim_run(&x, 3000u, NULL, NULL);

    double n0 = x.w.north_m, e0 = x.w.east_m, lat, lon;
    float tn = cosf(brg * 0.0174533f), te = sinf(brg * 0.0174533f);
    world_t tmp = x.w;
    tmp.north_m = n0 + dist * tn;
    tmp.east_m = e0 + dist * te;
    prog_wp_t wp = wp_world(&x, tmp.north_m, tmp.east_m, speed, 25.0f, WP_ACT_STOP);
    auto_go(&x, &wp, 1, PROG_END_STOP);

    uint32_t t_fault = 0u, t_clear = 0u, t_rec = 0u;
    float prev_target = x.sb.ap.target_heading;
    while (x.t < 1500000u) {
        sbsim_run(&x, x.t + 50u, NULL, NULL);
        float rn = (float)(x.w.north_m - n0), re = (float)(x.w.east_m - e0);
        float along = rn * tn + re * te, xte = re * tn - rn * te;
        double en, ee;
        dr_latlon(&x.sb.dr, &lat, &lon);
        world_from_latlon(lat, lon, &en, &ee);
        float err = (float)sqrt((en - x.w.north_m) * (en - x.w.north_m) + (ee - x.w.east_m) * (ee - x.w.east_m));
        bool nav = auto_running(&x);

        if (nav && along > 300.0f && fabsf(xte) > m.xte_true_max) m.xte_true_max = fabsf(xte);
        float step = fabsf(hdg_err(x.sb.ap.target_heading, prev_target));
        prev_target = x.sb.ap.target_heading;

        /* start the fault at 1 km */
        if (fault != GF_NONE && t_fault == 0u && along >= 1000.0f) {
            t_fault = x.t;
            m.suspects_before = x.sb.dr.suspects;
            switch (fault) {
            case GF_LOSS: case GF_RETURN: x.w.gps_fail = true; break;
            case GF_DRAG:   /* drag 300 m to the right of the track at 0.5 m/s */
                x.w.gps_dragging = true;
                x.w.gps_drag_n = -te * 0.5f; x.w.gps_drag_e = tn * 0.5f;
                x.w.gps_drag_max_m = 300.0f;
                break;
            case GF_JUMP:   /* 500 m to the left, for 2 minutes */
                x.w.gps_off_n = 500.0 * te; x.w.gps_off_e = -500.0 * tn;
                break;
            case GF_FLAG: x.w.gps_flag = true; break;
            default: break;
            }
        }
        if (t_fault) {
            float since = (float)(x.t - t_fault) * 0.001f;
            if (m.t_detect < 0.0f && x.sb.dr.state != DR_GPS) { m.t_detect = since; m.why = x.sb.dr.why; }
            /* clear the fault */
            if (!t_clear && ((fault == GF_RETURN && since >= 300.0f) || (fault == GF_JUMP && since >= 120.0f) ||
                             (fault == GF_FLAG && since >= 60.0f))) {
                t_clear = x.t;
                x.w.gps_fail = false;
                x.w.gps_off_n = x.w.gps_off_e = 0.0;
                x.w.gps_flag = false;
            }
            if (!t_clear && err > m.est_err_fault) m.est_err_fault = err;
            if (x.sb.dr.state == DR_DEAD || x.sb.dr.state == DR_RECOVER) {
                float r = err / fmaxf(x.sb.dr.unc_m, 1.0f);
                if (r > m.ratio_max) m.ratio_max = r;
            }
            if (t_clear && !t_rec && x.sb.dr.state == DR_GPS) {
                t_rec = x.t;
                m.t_recover = (float)(x.t - t_clear) * 0.001f;
            }
            if (t_rec && x.t <= t_rec + 60000u && nav && step > m.steer_step_max) m.steer_step_max = step;
            if (t_rec && x.t >= t_rec + 30000u && m.err_after_recover == 0.0f) m.err_after_recover = err + 1e-3f;
        } else if (nav && along > 300.0f && step > m.steer_step_normal) {
            m.steer_step_normal = step;
        }
        if (!nav && m.end_t < 0.0f) {
            m.end_t = (float)x.t * 0.001f;
            m.end_err = err;
            m.end_unc = x.sb.dr.unc_m;
            m.end_why = auto_end_why(&x);
            break;
        }
    }
    m.suspects_total = x.sb.dr.suspects;
    return m;
}

static void gps_denied(void)
{
    char d[500];
    printf("\n=== GPS-denied AUTO (3 km leg at 12 kn, fault at 1 km) ===\n");

    /* GPS loss: with / without current, with / without paddlewheel */
    struct { const char *name; float ce, cn; bool pw; } loss[] = {
        { "GPS loss at 1 km, no current, no speed sensor",       0.0f,  0.0f, false },
        { "GPS loss at 1 km, 0.8 m/s side current, no sensor",   0.4f, -0.7f, false },
        { "GPS loss at 1 km, no current, paddlewheel",           0.0f,  0.0f, true  },
        { "GPS loss at 1 km, 0.8 m/s side current, paddlewheel", 0.4f, -0.7f, true  },
    };
    printf("%-52s | %8s %8s %6s | %-18s\n", "case", "err end", "unc end", "ratio", "ended");
    for (unsigned i = 0; i < sizeof loss / sizeof loss[0]; i++) {
        mission_t m = gps_mission(GF_LOSS, loss[i].ce, loss[i].cn, loss[i].pw, 91u + i);
        printf("%-52s | %7.1f m %7.1f m %6.2f | %s after %.0f s\n", loss[i].name, m.end_err, m.end_unc,
               m.ratio_max, mode_reason_name(m.end_why), m.end_t);
        snprintf(d, sizeof d, "position error at the end %.1f m vs uncertainty %.1f m (worst ratio in DR %.2f); "
                 "true cross-track max %.0f m; ended: %s; false integrity alarms before the loss %u",
                 m.end_err, m.end_unc, m.ratio_max, m.xte_true_max, mode_reason_name(m.end_why), m.suspects_before);
        char name[96];
        snprintf(name, sizeof name, "DR %s", loss[i].name);
        test_report(name, m.t_detect >= 0.0f && m.why == DR_WHY_LOST && m.ratio_max <= 1.0f &&
                    m.suspects_before == 0u &&
                    (m.end_why == WHY_ARRIVAL_UNCERTAIN || m.end_why == WHY_ARRIVED || m.end_why == WHY_DR_LIMIT), d);
    }

    /* spoofer drags the position 300 m sideways */
    {
        mission_t m = gps_mission(GF_DRAG, 0.0f, 0.0f, false, 95u);
        snprintf(d, sizeof d, "caught after %.0f s (%s); estimate error while spoofed max %.0f m; boat pulled off "
                 "the line max %.0f m (spoofer: 300 m); ended %s with error %.0f m vs uncertainty %.0f m",
                 m.t_detect, dr_reason_name(m.why), m.est_err_fault, m.xte_true_max,
                 mode_reason_name(m.end_why), m.end_err, m.end_unc);
        test_report("GPS spoof dragging 300 m sideways: caught, boat not led away",
                    m.t_detect >= 0.0f && m.why != DR_WHY_LOST && m.t_detect < 180.0f &&
                    m.est_err_fault < 100.0f && m.xte_true_max < 100.0f && m.suspects_before == 0u, d);
    }
    /* sudden 500 m jump (2 min), then the GPS is right again */
    {
        mission_t m = gps_mission(GF_JUMP, 0.0f, 0.0f, false, 96u);
        snprintf(d, sizeof d, "caught after %.0f s (%s); estimate error during the jump max %.0f m; GPS trusted again "
                 "%.0f s after it was right; steering step after recovery %.1f deg (normal %.1f); ended %s",
                 m.t_detect, dr_reason_name(m.why), m.est_err_fault, m.t_recover, m.steer_step_max,
                 m.steer_step_normal, mode_reason_name(m.end_why));
        test_report("GPS jump of 500 m: caught at once, recovered smoothly",
                    m.t_detect >= 0.0f && m.t_detect <= 2.0f && m.why == DR_WHY_JUMP && m.est_err_fault < 30.0f &&
                    m.t_recover > 0.0f && m.steer_step_max <= fmaxf(5.0f, 1.5f * m.steer_step_normal) &&
                    m.end_why == WHY_ARRIVED, d);
    }
    /* receiver reports spoofing / jamming */
    {
        mission_t m = gps_mission(GF_FLAG, 0.0f, 0.0f, false, 97u);
        snprintf(d, sizeof d, "caught after %.0f s (%s), GPS trusted again %.0f s after the flag cleared, ended %s",
                 m.t_detect, dr_reason_name(m.why), m.t_recover, mode_reason_name(m.end_why));
        test_report("GPS receiver spoof/jam flag: treated as GPS failure",
                    m.t_detect >= 0.0f && m.t_detect <= 2.0f && m.why == DR_WHY_FLAG && m.t_recover > 0.0f, d);
    }
    /* GPS back after 5 minutes */
    {
        mission_t m = gps_mission(GF_RETURN, 0.0f, 0.0f, false, 98u);
        snprintf(d, sizeof d, "DR for 300 s, GPS trusted again %.0f s after it returned; steering step %.1f deg "
                 "(normal %.1f); estimate error 30 s later %.1f m; ended %s, error %.1f m",
                 m.t_recover, m.steer_step_max, m.steer_step_normal, m.err_after_recover,
                 mode_reason_name(m.end_why), m.end_err);
        test_report("GPS returning after 5 min: N consistent fixes, smooth blend, no steering jump",
                    m.t_recover > 0.0f && m.t_recover < 20.0f &&
                    m.steer_step_max <= fmaxf(5.0f, 1.5f * m.steer_step_normal) &&
                    m.err_after_recover < 30.0f && m.end_why == WHY_ARRIVED, d);
    }
    /* no false alarms with a good GPS (with current, turns at the start) */
    {
        mission_t a = gps_mission(GF_NONE, 0.0f, 0.0f, false, 99u);
        mission_t b = gps_mission(GF_NONE, 0.4f, -0.7f, false, 100u);
        snprintf(d, sizeof d, "integrity alarms with a good GPS: %u (no current), %u (side current); both %s / %s",
                 a.suspects_total, b.suspects_total, mode_reason_name(a.end_why), mode_reason_name(b.end_why));
        test_report("GPS integrity: no false alarms with a good GPS",
                    a.suspects_total == 0u && b.suspects_total == 0u &&
                    a.end_why == WHY_ARRIVED && b.end_why == WHY_ARRIVED, d);
    }
}

/* ------------------------------------------------------------ joystick -- */

typedef struct {
    float t_switch;             /* deflection -> MANUAL, s (-1 never) */
    sb_mode_t from;
    float v_before;
    float thr_fwd_max, thr_rev_max;
    int   under_load, fr_no_n;
    float est_at_x, true_at_x;  /* outboard with X = 0.6 */
    float v_fwd, v_rev;         /* boat speed after full forward / full reverse */
    tr_result_t train_result;
    bool  prog_suspended;
} override_t;

/* Put the boat in `mode`, then a person grabs the stick: X right 0.6,
 * then Y full forward, then straight to full reverse. */
static override_t joystick_override(sb_mode_t mode, uint32_t seed)
{
    sbsim_t x;
    override_t r;
    memset(&r, 0, sizeof r);
    r.t_switch = -1.0f;
    r.from = mode;
    sbsim_init(&x, seed, 90.0f, 0.0f, NULL);
    sbsim_run(&x, 3000u, NULL, NULL);

    switch (mode) {
    case MODE_HDG_LOCK:
        hdg_lock(&x);
        joy_set(0.0f, 1.0f, false, false);          /* full throttle on the stick, heading held */
        sbsim_run(&x, 120000u, NULL, NULL);
        break;
    case MODE_AUTO:
        auto_to(&x, 6000.0f, 90.0f, 20.0f);
        sbsim_run(&x, 120000u, NULL, NULL);
        break;
    case MODE_TRAINING:
        sb_request_mode(&x.sb, MODE_TRAINING, WHY_PC);
        sbsim_run(&x, 120000u, NULL, NULL);         /* spin-up + calibration running */
        break;
    default:
        break;
    }
    r.v_before = x.w.speed_kn;
    bool was = sb_mode(&x.sb) == mode;

    /* grab the stick (from HDG LOCK the Y stays where it is) */
    uint32_t t0 = x.t;
    joy_set(0.6f, mode == MODE_HDG_LOCK ? 1.0f : 0.0f, false, false);
    while (x.t < t0 + 2000u) {
        sbsim_run(&x, x.t + 10u, NULL, NULL);
        if (r.t_switch < 0.0f && sb_mode(&x.sb) == MODE_MANUAL) r.t_switch = (float)(x.t - t0) * 0.001f;
    }
    if (!was) r.t_switch = -1.0f;
    r.train_result = x.sb.ap.train.result;
    r.prog_suspended = x.sb.autop.suspended;
    if (mode == MODE_HDG_LOCK) {                    /* back to centre in Y first */
        joy_set(0.6f, 0.5f, false, false);
        sbsim_run(&x, x.t + 50u, NULL, NULL);
        joy_set(0.6f, 0.0f, false, false);
    }
    sbsim_run(&x, x.t + 15000u, NULL, NULL);
    r.est_at_x = x.sb.ap.est_angle;
    r.true_at_x = x.w.rud.angle_deg;

    /* the throttle limits only count from here (the autopilot mode before may use more) */
    x.w.max_thr_fwd = x.w.max_thr_rev = 0.0f;
    joy_set(0.6f, 1.0f, false, false);              /* full forward (centre -> end in one step) */
    sbsim_run(&x, x.t + 20000u, NULL, NULL);
    r.v_fwd = x.w.speed_kn;
    for (int k = 1; k <= 6; k++) {                  /* straight to full reverse, a hand takes ~0.3 s */
        joy_set(0.6f, 1.0f - 2.0f * (float)k / 6.0f, false, false);
        sbsim_run(&x, x.t + 50u, NULL, NULL);
    }
    sbsim_run(&x, x.t + 25000u, NULL, NULL);
    r.v_rev = x.w.speed_kn;
    r.thr_fwd_max = x.w.max_thr_fwd;
    r.thr_rev_max = x.w.max_thr_rev;
    r.under_load = x.w.shift_under_load;
    r.fr_no_n = x.w.fr_without_neutral;
    return r;
}

static void joystick_tests(void)
{
    char d[400];
    printf("\n=== Joystick override (MANUAL) ===\n");
    sb_config_t defc;
    sb_default_config(&defc);
    float expect = (0.6f - defc.joy.deadband) / (1.0f - defc.joy.deadband) * defc.steer.outboard.max_angle_deg;

    const sb_mode_t modes[4] = { MODE_STANDBY, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING };
    for (int i = 0; i < 4; i++) {
        override_t r = joystick_override(modes[i], 110u + (uint32_t)i);
        char name[96];
        snprintf(name, sizeof name, "Joystick override from %s", mode_name(modes[i]));
        snprintf(d, sizeof d, "at %.1f kn: control switched after %.2f s; outboard %.1f deg (est %.1f, wanted %.1f); "
                 "full fwd %.1f kn, full rev %.1f kn; throttle max fwd %.0f %% / rev %.0f %%; under-load %d, "
                 "F/R w/o N %d%s%s%s",
                 r.v_before, r.t_switch, r.true_at_x, r.est_at_x, expect, r.v_fwd, r.v_rev, r.thr_fwd_max,
                 r.thr_rev_max, r.under_load, r.fr_no_n, modes[i] == MODE_TRAINING ? "; training: " : "",
                 modes[i] == MODE_TRAINING ? training_result_text(r.train_result) : "",
                 modes[i] == MODE_AUTO ? (r.prog_suspended ? "; program suspended" : "; program NOT suspended") : "");
        bool ok = r.t_switch >= 0.0f && r.t_switch <= 0.2f &&
                  fabsf(r.est_at_x - expect) < 3.0f && fabsf(r.true_at_x - expect) < 4.0f &&
                  r.v_fwd > 3.0f && r.v_rev < -1.0f &&
                  r.thr_fwd_max <= defc.joy_max_throttle_pct + 0.01f &&
                  r.thr_rev_max <= defc.throttle.max_rev_pct + 0.01f &&
                  r.under_load == 0 && r.fr_no_n == 0 &&
                  (modes[i] != MODE_TRAINING || r.train_result == TR_ABORT_USER) &&
                  (modes[i] != MODE_AUTO || r.prog_suspended) &&
                  (modes[i] == MODE_STANDBY || r.v_before > 4.0f || modes[i] == MODE_TRAINING);
        test_report(name, ok, d);
    }

    /* PC can't leave MANUAL; the STANDBY button can (and disarms a held stick) */
    {
        sbsim_t x;
        sbsim_init(&x, 120u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        joy_set(0.0f, 0.5f, false, false);
        sbsim_run(&x, 5000u, NULL, NULL);
        bool in_man = sb_mode(&x.sb) == MODE_MANUAL;
        bool pc_sb = sb_request_mode(&x.sb, MODE_STANDBY, WHY_PC);
        bool pc_tr = sb_request_mode(&x.sb, MODE_TRAINING, WHY_PC);
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        bool pc_logged = le->why == WHY_PC && le->verdict != MODE_OK;
        press(&x, HAL_BTN_STANDBY);                 /* stick still held forward */
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool sb_now = sb_mode(&x.sb) == MODE_STANDBY, disarmed = !x.sb.joy.armed;
        joy_set(0.0f, 0.0f, false, false);          /* let go */
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool rearmed = x.sb.joy.armed;
        snprintf(d, sizeof d, "in MANUAL %s; PC -> STANDBY %s, PC -> TRAINING %s (logged %s); STANDBY button -> %s, "
                 "held stick disarmed %s (no grab back), re-armed after centring %s",
                 in_man ? "yes" : "NO", pc_sb ? "ACCEPTED" : "refused", pc_tr ? "ACCEPTED" : "refused",
                 pc_logged ? "yes" : "NO", sb_now ? "STANDBY" : mode_name(sb_mode(&x.sb)),
                 disarmed ? "yes" : "NO", rearmed ? "yes" : "NO");
        test_report("Joystick wins over the PC; the STANDBY button always works",
                    in_man && !pc_sb && !pc_tr && pc_logged && sb_now && disarmed && rearmed, d);
    }

    /* hand-back only with the stick centred: AUTO button -> STANDBY, or back to HDG LOCK */
    {
        sbsim_t x;
        sbsim_init(&x, 121u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        joy_set(0.0f, 0.5f, false, false);
        sbsim_run(&x, 5000u, NULL, NULL);
        joy_set(0.0f, 0.5f, false, true);           /* AUTO with the stick still forward */
        sbsim_run(&x, 5200u, NULL, NULL);
        joy_set(0.0f, 0.5f, false, false);
        sbsim_run(&x, 5400u, NULL, NULL);
        bool refused = sb_mode(&x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, 6000u, NULL, NULL);
        joy_set(0.0f, 0.0f, false, true);           /* centred + AUTO */
        sbsim_run(&x, 6200u, NULL, NULL);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, 9000u, NULL, NULL);
        bool back = sb_mode(&x.sb) == MODE_STANDBY;

        /* from HDG LOCK: X takes over, AUTO (centred) goes back to HDG LOCK */
        hdg_lock(&x);
        joy_set(0.5f, 0.0f, false, false);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool man2 = sb_mode(&x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        joy_set(0.0f, 0.0f, false, true);
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool hdg2 = sb_mode(&x.sb) == MODE_HDG_LOCK;
        snprintf(d, sizeof d, "AUTO with stick forward: %s; centred + AUTO: %s; from HDG LOCK: X -> %s, "
                 "centred AUTO -> %s",
                 refused ? "refused, stays MANUAL" : "HANDED BACK", back ? "STANDBY" : mode_name(sb_mode(&x.sb)),
                 man2 ? "MANUAL" : "?", hdg2 ? "HDG_LOCK" : mode_name(sb_mode(&x.sb)));
        test_report("Joystick hand-back only with the stick centred", refused && back && man2 && hdg2, d);
    }

    /* not centred at power-up */
    {
        sbsim_t x;
        remove("nv1.bin");
        sbsim_init(&x, 121u, 90.0f, 0.0f, NULL);
        joy_set(0.5f, 0.0f, false, false);          /* someone leans on the stick */
        sbsim_run(&x, 3000u, NULL, NULL);
        bool not_armed = !x.sb.joy.armed && (x.sb.alarm_bits & SB_ALM_JOY_NOT_CENTERED) && sim_hal_alarm();
        bool no_takeover = sb_mode(&x.sb) == MODE_STANDBY;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, 4000u, NULL, NULL);
        bool armed = x.sb.joy.armed;
        joy_set(0.5f, 0.0f, false, false);
        sbsim_run(&x, 5000u, NULL, NULL);
        snprintf(d, sizeof d, "not centred: armed %s, alarm %s, took control %s; after centring: armed %s, "
                 "deflection then takes control %s",
                 x.sb.joy.armed && !armed ? "?" : (not_armed ? "no" : "YES"), not_armed ? "on" : "OFF",
                 no_takeover ? "no" : "YES", armed ? "yes" : "NO", sb_mode(&x.sb) == MODE_MANUAL ? "yes" : "NO");
        test_report("Joystick not centred at power-up: ignored + alarm until centred",
                    not_armed && no_takeover && armed && sb_mode(&x.sb) == MODE_MANUAL, d);
    }

    /* wire breaks: X, Y (not in control), and while in control */
    {
        bool ok_axis[2];
        for (int a = 0; a < 2; a++) {
            sbsim_t x;
            remove("nv1.bin");
            sbsim_init(&x, 122u + (uint32_t)a, 90.0f, 0.0f, NULL);
            joy_set(0.0f, 0.0f, false, false);
            sbsim_run(&x, 2000u, NULL, NULL);
            if (a == 0) sim_hal_set_joystick(true, 0u, joy_raw(0.0f), false, false);       /* X wiper broken */
            else        sim_hal_set_joystick(true, joy_raw(0.0f), 4095u, false, false);    /* Y ground broken */
            sbsim_run(&x, 4000u, NULL, NULL);
            ok_axis[a] = x.sb.joy.fault && x.sb.joy.fault_axis == a + 1 &&
                         (x.sb.alarm_bits & SB_ALM_JOY_FAULT) && sb_mode(&x.sb) == MODE_STANDBY;
        }
        snprintf(d, sizeof d, "X wire break (ADC 0): %s; Y wire break (ADC 4095): %s",
                 ok_axis[0] ? "fault + alarm, ignored" : "NOT DETECTED", ok_axis[1] ? "fault + alarm, ignored" : "NOT DETECTED");
        test_report("Joystick wire break in X and in Y detected", ok_axis[0] && ok_axis[1], d);

        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        remove("nv1.bin");
        sbsim_init(&x, 124u, 90.0f, 0.0f, NULL);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, 2000u, NULL, NULL);
        joy_set(0.5f, 1.0f, false, false);          /* driving on the stick */
        sbsim_run(&x, 30000u, NULL, NULL);
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sim_hal_set_joystick(true, joy_raw(0.5f), 0u, false, false);   /* Y wire breaks */
        sbsim_run(&x, x.t + 15000u, ev_watch, &ev);
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        static const mode_log_entry_t none = { 0u, MODE_STANDBY, MODE_STANDBY, WHY_TEST, MODE_OK };
        if (!le) le = &none;
        snprintf(d, sizeof d, "mode %s (%s); throttle down after %.2f s, neutral %.2f s, steering off %.2f s, "
                 "motor after %d; alarm %s; under-load %d",
                 mode_name(sb_mode(&x.sb)), mode_reason_name(le->why), ev.t_thr_down, ev.t_neutral, ev.t_steer_off,
                 ev.steer_after_off, sim_hal_alarm() ? "on" : "OFF", x.w.shift_under_load);
        test_report("Joystick wire break while in control: idle + neutral, steering stop, STANDBY",
                    sb_mode(&x.sb) == MODE_STANDBY && le->why == WHY_JOY_FAULT && ev.t_thr_down >= 0.0f &&
                    ev.t_thr_down <= 0.1f && ev.t_neutral > 0.0f && ev.t_steer_off >= 0.0f && ev.t_steer_off <= 0.1f &&
                    ev.steer_after_off == 0 && sim_hal_alarm() && x.w.shift_under_load == 0, d);
    }

    /* calibration routine: stored with checksum */
    {
        sbsim_t x;
        remove("nv1.bin");
        sbsim_init(&x, 125u, 90.0f, 0.0f, NULL);
        sim_hal_set_joystick(true, 2000u, 2100u, false, false);
        sbsim_run(&x, 1000u, NULL, NULL);
        bool started = sb_joystick_cal_start(&x.sb);
        uint16_t path[6][2] = { {2000, 2100}, {500, 2100}, {3600, 2100}, {2000, 450}, {2000, 3700}, {2000, 2100} };
        for (int k = 0; k < 6; k++) {
            sim_hal_set_joystick(true, path[k][0], path[k][1], false, false);
            sbsim_run(&x, x.t + 300u, NULL, NULL);
        }
        bool saved = sb_joystick_cal_finish(&x.sb);
        joy_cal_t back;
        bool loaded = hal_nv_load(HAL_NV_SLOT_JOYSTICK, &back, (uint32_t)sizeof back) && joy_cal_valid(&back);
        bool values = loaded && back.center[0] == 2000u && back.min[0] == 500u && back.max[0] == 3600u &&
                      back.center[1] == 2100u && back.min[1] == 450u && back.max[1] == 3700u;
        back.max[1] ^= 1u;
        bool corrupt_caught = !joy_cal_valid(&back);
        /* a "calibration" without moving the stick is refused */
        bool started2 = sb_joystick_cal_start(&x.sb);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool refused = started2 && !sb_joystick_cal_finish(&x.sb);
        remove("nv1.bin");
        snprintf(d, sizeof d, "calibrate: started %s, saved %s, values %s, corrupted copy rejected %s, "
                 "no-movement calibration refused %s",
                 started ? "yes" : "NO", saved ? "yes" : "NO", values ? "correct" : "WRONG",
                 corrupt_caught ? "yes" : "NO", refused ? "yes" : "NO");
        test_report("Joystick calibration stored with checksum", started && saved && values && corrupt_caught && refused, d);
    }
}

/* ------------------------------------------------------- section A: modes -- */

/* 5-waypoint program from the boat, roughly east, with different speeds
 * and actions. */
static int prog5(const sbsim_t *x, prog_wp_t *w)
{
    double n = x->w.north_m, e = x->w.east_m;
    w[0] = wp_world(x, n + 100.0, e +  600.0,  8.0f, 25.0f, WP_ACT_CONTINUE);
    w[1] = wp_world(x, n - 100.0, e + 1400.0, 14.0f, 25.0f, WP_ACT_SLOW);
    w[2] = wp_world(x, n +   0.0, e + 2000.0, 10.0f, 25.0f, WP_ACT_STOP);
    w[3] = wp_world(x, n + 300.0, e + 3300.0, 16.0f, 30.0f, WP_ACT_CONTINUE);
    w[4] = wp_world(x, n + 300.0, e + 3900.0,  6.0f, 20.0f, WP_ACT_CONTINUE);
    return 5;
}

typedef struct {
    int    wp_seen;
    float  sog_sum[8]; int sog_n[8];      /* settled speed per leg */
    uint32_t leg_t0[8];
    float  v_at_stop;                     /* speed when stopped at the STOP waypoint */
    bool   stopped_ok;                    /* idle + neutral at the STOP waypoint */
    float  v_near_slow;                   /* speed passing the SLOW waypoint */
} p5_t;

static void p5_hook(sbsim_t *x, void *ctx)
{
    p5_t *p = (p5_t *)ctx;
    int g = x->sb.autop.wp;
    if (getenv("DBGC") && x->t % 2000u < 50u && x->t > 380000u && x->t < 480000u)
        printf("DBG t %.0f st %d wp %d sog %.1f hdg %.0f tgt %.0f ref %.0f oc %u thr %.0f gear %d lim %.0f est %.1f\n",
               x->t * 0.001, x->sb.autop.state, g, world_sog_kn(&x->w), x->w.heading_deg, x->sb.ap.target_heading,
               x->sb.ap.ref_heading, (unsigned)x->sb.ap.off_course_ms, sim_hal_throttle(), x->w.engaged,
               x->sb.plim.fwd_pct, x->sb.ap.est_angle);
    if (g != p->wp_seen && g < 8) { p->wp_seen = g; p->leg_t0[g] = x->t; }
    if (auto_running(x) && g < 8 && x->t > p->leg_t0[g] + 45000u && x->sb.autop.leg.dist_m > 200.0f) {
        p->sog_sum[g] += world_sog_kn(&x->w);
        p->sog_n[g]++;
    }
    if (g == 1 && x->sb.autop.leg.dist_m < 30.0f) p->v_near_slow = world_sog_kn(&x->w);
    if (x->sb.autop.state == AUTO_WP_STOP) {
        p->v_at_stop = world_sog_kn(&x->w);
        if (gear_is(&x->sb.gear, GEAR_N) && sim_hal_throttle() <= 0.0f) p->stopped_ok = true;
    }
}

static void modes_tests(void)
{
    char d[500];
    printf("\n=== Section A: MANUAL / HDG LOCK / AUTO ===\n");

    /* A1 HDG LOCK: lock, +-1/+-10, throttle on the stick incl. reverse, X -> MANUAL */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 201u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        x.w.heading_deg = 73.0f;                    /* boat drifted round a bit */
        sbsim_run(&x, 6000u, NULL, NULL);
        float h_press = x.sb.ap.heading;
        bool locked = hdg_lock(&x);
        float t_lock = x.sb.ap.target_heading;
        press(&x, HAL_BTN_PLUS_10);
        press(&x, HAL_BTN_PLUS_1);
        press(&x, HAL_BTN_PLUS_1);
        press(&x, HAL_BTN_MINUS_10);
        press(&x, HAL_BTN_MINUS_1);
        float t_adj = x.sb.ap.target_heading;       /* +10 +1 +1 -10 -1 = +1 */
        joy_set(0.0f, 0.7f, false, false);          /* throttle ahead on the stick */
        sbsim_run(&x, x.t + 60000u, NULL, NULL);
        bool still_hdg = sb_mode(&x.sb) == MODE_HDG_LOCK;
        float v_ahead = world_sog_kn(&x.w);
        float hold = fabsf(hdg_err(x.w.heading_deg, x.sb.ap.target_heading));
        float tgt_before_rev = x.sb.ap.target_heading;
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_r_us);
        for (int k = 1; k <= 6; k++) {              /* stick forward -> back */
            joy_set(0.0f, 0.7f - 1.4f * (float)k / 6.0f, false, false);
            sbsim_run(&x, x.t + 50u, ev_watch, &ev);
        }
        sbsim_run(&x, x.t + 30000u, ev_watch, &ev);
        bool hdg_in_rev = sb_mode(&x.sb) == MODE_HDG_LOCK;
        float v_rev = x.w.speed_kn, ang_rev = x.w.rud.angle_deg;
        const gear_config_t *g = &x.sb.cfg.gear;
        bool order = ev.t_idle >= 0.0f && ev.t_neutral >= ev.t_idle + g->idle_wait_ms * 0.001f - 0.06f &&
                     ev.t_gear >= ev.t_neutral + g->neutral_wait_ms * 0.001f - 0.06f &&
                     ev.t_thr_open >= ev.t_gear + g->engage_wait_ms * 0.001f - 0.06f;
        for (int k = 1; k <= 6; k++) {              /* and ahead again */
            joy_set(0.0f, -0.7f + 1.4f * (float)k / 6.0f, false, false);
            sbsim_run(&x, x.t + 50u, NULL, NULL);
        }
        sbsim_run(&x, x.t + 60000u, NULL, NULL);
        float hold2 = fabsf(hdg_err(x.w.heading_deg, x.sb.ap.target_heading));
        bool target_kept = fabsf(hdg_err(x.sb.ap.target_heading, tgt_before_rev)) < 0.01f;
        joy_set(0.6f, 0.7f, false, false);          /* X -> MANUAL */
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool manual = sb_mode(&x.sb) == MODE_MANUAL;
        snprintf(d, sizeof d, "locked %s at %.1f (heading %.1f); after +10 +1 +1 -10 -1: %+.1f deg; ahead %.1f kn "
                 "still HDG LOCK %s, course error %.1f; reverse: stays HDG LOCK %s, %.1f kn, outboard %.1f deg, "
                 "gear order idle %.2f / N %.2f / R %.2f / throttle %.2f s, under-load %d, F/R w/o N %d; "
                 "ahead again: target kept %s, course error %.1f; X -> %s",
                 locked ? "yes" : "NO", t_lock, h_press, hdg_err(t_adj, t_lock), v_ahead, still_hdg ? "yes" : "NO",
                 hold, hdg_in_rev ? "yes" : "NO", v_rev, ang_rev, ev.t_idle, ev.t_neutral, ev.t_gear, ev.t_thr_open,
                 x.w.shift_under_load, x.w.fr_without_neutral, target_kept ? "yes" : "NO", hold2,
                 manual ? "MANUAL" : mode_name(sb_mode(&x.sb)));
        test_report("A1 HDG LOCK: locks heading, +-1/+-10, stick throttle fwd/rev in HDG LOCK, X -> MANUAL",
                    locked && fabsf(hdg_err(t_lock, h_press)) < 0.5f && fabsf(hdg_err(t_adj, t_lock) - 1.0f) < 0.01f &&
                    still_hdg && v_ahead > 5.0f && hold < 5.0f && hdg_in_rev && v_rev < -1.0f &&
                    fabsf(ang_rev) < 3.0f && order && x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0 &&
                    target_kept && hold2 < 5.0f && manual, d);
    }

    /* A1b compass lost in HDG LOCK: gyro hold, alarm, throttle stays with the user, STANDBY when stopped */
    {
        sbsim_t x;
        sbsim_init(&x, 202u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        hdg_lock(&x);
        joy_set(0.0f, 0.8f, false, false);
        sbsim_run(&x, 60000u, NULL, NULL);
        float thr0 = sim_hal_throttle(), h0 = x.w.heading_deg;
        sim_hal_fail_heading(true);
        sbsim_run(&x, 75000u, NULL, NULL);
        bool kept = sb_mode(&x.sb) == MODE_HDG_LOCK && x.sb.ap.mode == AP_MODE_GYRO_HOLD;
        float thr1 = sim_hal_throttle(), drift = fabsf(hdg_err(x.w.heading_deg, h0));
        bool alarm = sim_hal_alarm() && (x.sb.alarm_bits & SB_ALM_COMPASS);
        joy_set(0.0f, 0.0f, false, false);          /* user brings it back */
        sbsim_run(&x, 95000u, NULL, NULL);
        snprintf(d, sizeof d, "after the fault: mode %s steering %s, alarm %s, throttle %.0f -> %.0f %% (user), "
                 "heading drift %.1f deg in 15 s; stick back: mode %s, steering %s",
                 mode_name(sb_mode(&x.sb)), kept ? "gyro hold" : "?", alarm ? "on" : "OFF", thr0, thr1, drift,
                 mode_name(sb_mode(&x.sb)), x.sb.ap.mode == AP_MODE_STANDBY ? "off" : "ON");
        /* the alarm drops the power limit to the startup 30 %: the stick still drives, below it */
        test_report("A1b compass lost in HDG LOCK: gyro hold + alarm, throttle stays with the user, then STANDBY",
                    kept && alarm && fabsf(thr1 - fminf(thr0, x.sb.cfg.plim.fwd_steps[0])) < 1.0f && drift < 10.0f &&
                    sb_mode(&x.sb) == MODE_STANDBY && x.sb.ap.mode == AP_MODE_STANDBY, d);
    }

    /* A2 AUTO: 5-waypoint program, different leg speeds and actions */
    {
        sbsim_t x;
        p5_t p;
        prog_wp_t w[5];
        memset(&p, 0, sizeof p);
        sbsim_init(&x, 203u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        int n = prog5(&x, w);
        bool started = auto_go(&x, w, n, PROG_END_STOP);
        p.leg_t0[0] = x.t;
        uint32_t t_end = x.t + 1500000u;
        bool resumed = false;
        while (x.t < t_end && sb_mode(&x.sb) == MODE_AUTO) {
            sbsim_run(&x, x.t + 50u, p5_hook, &p);
            if (x.sb.autop.state == AUTO_WP_STOP && !resumed && world_sog_kn(&x.w) < 0.5f) {
                sbsim_run(&x, x.t + 5000u, p5_hook, &p);
                resumed = sb_auto_resume(&x.sb) == SB_OK && confirm_press(&x);
            }
        }
        double fn, fe;
        world_from_latlon(program_lat(&w[4]), program_lon(&w[4]), &fn, &fe);
        float miss = (float)sqrt((fn - x.w.north_m) * (fn - x.w.north_m) + (fe - x.w.east_m) * (fe - x.w.east_m));
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        int len = snprintf(d, sizeof d, "started %s; leg speeds (set/measured):", started ? "yes" : "NO");
        bool speeds_ok = true;
        for (int i = 0; i < 5; i++) {
            float v = p.sog_n[i] ? p.sog_sum[i] / (float)p.sog_n[i] : -1.0f;
            len += snprintf(d + len, sizeof d - (size_t)len, " %.0f/%.1f", w[i].speed_kn, v);
            if (p.sog_n[i] > 20 && fabsf(v - w[i].speed_kn) > 1.0f) speeds_ok = false;
        }
        snprintf(d + len, sizeof d - (size_t)len, "; passing SLOW wp at %.1f kn; STOP wp: idle + neutral %s; "
                 "resumed with PC + CONFIRM %s; end: %s (%s), %.0f m from the last waypoint, alarm %s",
                 p.v_near_slow, p.stopped_ok ? "yes" : "NO", resumed ? "yes" : "NO", mode_name(sb_mode(&x.sb)),
                 mode_reason_name(le->why), miss, sim_hal_alarm() ? "on" : "OFF");
        test_report("A2 AUTO 5-waypoint program: leg speeds, SLOW / STOP actions, final stop + alarm",
                    started && speeds_ok && p.v_near_slow < 8.0f && p.stopped_ok && resumed &&
                    sb_mode(&x.sb) == MODE_STANDBY && le->why == WHY_ARRIVED && miss < 40.0f && sim_hal_alarm() &&
                    x.w.shift_under_load == 0, d);
    }

    /* A3 program validation */
    {
        sbsim_t x;
        static program_t p;
        prog_wp_t w[5];
        prog_error_t err;
        uint8_t bad;
        sbsim_init(&x, 204u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        int n = prog5(&x, w);
        char why[300] = "";
        int wrong = 0;
#define TRYP(label, want_err) do { sb_result_t r_ = sb_program_set(&x.sb, &p, &err, &bad); \
        if ((want_err) == PROG_OK ? r_ != SB_OK : (r_ != SB_ERR_BAD_PROGRAM || err != (want_err))) wrong++; \
        snprintf(why + strlen(why), sizeof why - strlen(why), " %s:%s", label, program_error_text(err)); } while (0)
        prog_make(&p, w, n, PROG_END_STOP);
        p.wp[2].lat_e7 = 950000000;  program_seal(&p); TRYP("lat95", PROG_BAD_COORD);
        prog_make(&p, w, n, PROG_END_STOP);
        p.wp[1].speed_kn = 30.0f;    program_seal(&p); TRYP("30kn", PROG_BAD_SPEED);
        prog_make(&p, w, n, PROG_END_STOP);
        p.n = 51;                    program_seal(&p); TRYP("51wp", PROG_BAD_COUNT);
        prog_make(&p, w, n, PROG_END_STOP);
        p.wp[3] = p.wp[2]; p.wp[3].lat_e7 += 10; program_seal(&p); TRYP("leg 0 m", PROG_LEG_SHORT);
        prog_make(&p, w, n, PROG_END_STOP);
        p.wp[4].lat_e7 += 9000000;   program_seal(&p); TRYP("leg 100 km", PROG_LEG_LONG);
        prog_make(&p, w, n, PROG_END_STOP);
        p.wp[0].radius_m = 1.0f;     program_seal(&p); TRYP("radius 1", PROG_BAD_RADIUS);
        prog_make(&p, w, n, PROG_END_STOP);
        p.wp[1].speed_kn = 9.0f;     /* changed after sealing */ TRYP("checksum", PROG_BAD_CHECKSUM);
        bool none_loaded = !x.sb.autop.prog_ok;
        prog_make(&p, w, n, PROG_END_STOP);
        TRYP("valid", PROG_OK);
#undef TRYP
        /* stored with checksum: survives a restart, a corrupted copy is not loaded */
        sbsim_t y;
        sb_config_t c;
        sb_default_config(&c);
        memset(&y, 0, sizeof y);
        world_init(&y.w, 205u, 90.0f, 0.0f);
        sb_init(&y.sb, &c);
        bool survived = y.sb.autop.prog_ok && y.sb.autop.prog.n == 5u;
        FILE *f = fopen("nv2.bin", "r+b");
        if (f) { fseek(f, 40, SEEK_SET); fputc(0x5A, f); fclose(f); }
        sb_init(&y.sb, &c);
        bool corrupt_refused = !y.sb.autop.prog_ok;
        remove("nv2.bin");
        snprintf(d, sizeof d, "refused (reason):%s; nothing loaded by bad uploads %s; after power-off: %s; corrupted "
                 "flash copy ignored %s", why, none_loaded ? "yes" : "NO", survived ? "program kept" : "LOST",
                 corrupt_refused ? "yes" : "NO");
        test_report("A3 AUTO program validation: bad coordinates, too many, speed > max, leg length, radius, checksum",
                    wrong == 0 && none_loaded && survived && corrupt_refused, d);
    }

    /* A4 CONFIRM rules */
    {
        sbsim_t x;
        prog_wp_t w[5];
        static program_t p;
        prog_error_t err;
        uint8_t bad;
        sbsim_init(&x, 206u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        int n = prog5(&x, w);
        prog_make(&p, w, n, PROG_END_STOP);
        sb_program_set(&x.sb, &p, &err, &bad);
        /* no CONFIRM: nothing happens, the request expires */
        sb_result_t r1 = sb_auto_start(&x.sb);
        sbsim_run(&x, x.t + 11000u, NULL, NULL);
        bool expired = x.sb.confirm.kind == CONF_NONE && sb_mode(&x.sb) == MODE_STANDBY;
        /* late CONFIRM (11 s) */
        sb_auto_start(&x.sb);
        sbsim_run(&x, x.t + 11000u, NULL, NULL);
        bool late = !confirm_press(&x) && sb_mode(&x.sb) == MODE_STANDBY;
        /* CONFIRM with the stick off centre (inside the deadband: no override) */
        sb_auto_start(&x.sb);
        joy_set(0.12f, 0.0f, false, false);
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool off_centre = !confirm_press(&x) && sb_mode(&x.sb) == MODE_STANDBY;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 11000u, NULL, NULL);
        /* CONFIRM with no request pending: nothing, in every mode */
        uint32_t ign0 = x.sb.confirm.ignored;
        bool nothing = true;
        nothing &= !confirm_press(&x) && sb_mode(&x.sb) == MODE_STANDBY;
        hdg_lock(&x);
        nothing &= !confirm_press(&x) && sb_mode(&x.sb) == MODE_HDG_LOCK;
        joy_set(0.0f, 0.5f, false, false); sbsim_run(&x, x.t + 300u, NULL, NULL);    /* Y in HDG LOCK = throttle */
        joy_set(0.5f, 0.5f, false, false); sbsim_run(&x, x.t + 300u, NULL, NULL);    /* X -> MANUAL */
        nothing &= !confirm_press(&x) && sb_mode(&x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false); sbsim_run(&x, x.t + 500u, NULL, NULL);
        press(&x, HAL_BTN_STANDBY);
        uint32_t ignored = x.sb.confirm.ignored - ign0;
        /* joystick faulted: start refused */
        sim_hal_set_joystick(true, 0u, joy_raw(0.0f), false, false);
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        sb_result_t r_fault = sb_auto_start(&x.sb);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        /* and finally a correct one */
        sb_auto_start(&x.sb);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool ok = confirm_press(&x) && sb_mode(&x.sb) == MODE_AUTO;
        snprintf(d, sizeof d, "start -> %d; no CONFIRM: expired %s; late CONFIRM refused %s; stick off centre refused %s; "
                 "CONFIRM with nothing pending does nothing in STANDBY/HDG LOCK/MANUAL %s (%u ignored); "
                 "joystick faulted -> start %d (not ready = %d); correct CONFIRM -> %s",
                 (int)r1, expired ? "yes" : "NO", late ? "yes" : "NO", off_centre ? "yes" : "NO",
                 nothing ? "yes" : "NO", ignored, (int)r_fault, (int)SB_ERR_JOY_NOT_READY,
                 ok ? "AUTO" : mode_name(sb_mode(&x.sb)));
        test_report("A4 AUTO start needs PC + CONFIRM within 10 s, stick centred; CONFIRM alone never changes mode",
                    r1 == SB_OK && expired && late && off_centre && nothing && ignored == 3u &&
                    r_fault == SB_ERR_JOY_NOT_READY && ok, d);
    }

    /* A5 pause / resume / skip, A6 joystick override mid-program and resume */
    {
        sbsim_t x;
        prog_wp_t w[5];
        sbsim_init(&x, 207u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        int n = prog5(&x, w);
        auto_go(&x, w, n, PROG_END_STOP);
        sbsim_run(&x, x.t + 40000u, NULL, NULL);
        sb_result_t rp = sb_auto_pause(&x.sb);
        sbsim_run(&x, x.t + 30000u, NULL, NULL);
        bool paused = sb_mode(&x.sb) == MODE_AUTO && x.sb.autop.state == AUTO_PAUSED &&
                      gear_is(&x.sb.gear, GEAR_N) && sim_hal_throttle() <= 0.0f;
        sb_auto_resume(&x.sb);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool no_conf_still_paused = x.sb.autop.state == AUTO_PAUSED;
        bool resumed = confirm_press(&x) && x.sb.autop.state == AUTO_RUN;
        uint8_t wp_before = x.sb.autop.wp;
        sb_result_t rs = sb_auto_skip(&x.sb);
        bool skipped = x.sb.autop.wp == wp_before + 1u && auto_running(&x);
        sbsim_run(&x, x.t + 30000u, NULL, NULL);
        /* override mid-program */
        uint8_t wp_at_override = x.sb.autop.wp;
        joy_set(-0.5f, 0.0f, false, false);
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool manual = sb_mode(&x.sb) == MODE_MANUAL && x.sb.autop.suspended;
        sbsim_run(&x, x.t + 10000u, NULL, NULL);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        sb_auto_resume(&x.sb);
        bool back = confirm_press(&x) && sb_mode(&x.sb) == MODE_AUTO && x.sb.autop.wp == wp_at_override;
        sbsim_run(&x, x.t + 60000u, NULL, NULL);
        snprintf(d, sizeof d, "pause -> %d: idle + neutral, still AUTO %s; resume without CONFIRM stays paused %s, "
                 "with CONFIRM runs %s; skip -> %d: next waypoint %s; stick mid-program -> MANUAL, program suspended %s; "
                 "PC resume + CONFIRM -> AUTO at the same waypoint %s; running after 60 s %s",
                 (int)rp, paused ? "yes" : "NO", no_conf_still_paused ? "yes" : "NO", resumed ? "yes" : "NO",
                 (int)rs, skipped ? "yes" : "NO", manual ? "yes" : "NO", back ? "yes" : "NO",
                 auto_running(&x) ? "yes" : "NO");
        test_report("A5/A6 AUTO pause / resume / skip; joystick override mid-program, resume with PC + CONFIRM",
                    rp == SB_OK && paused && no_conf_still_paused && resumed && rs == SB_OK && skipped && manual &&
                    back && auto_running(&x) && x.w.shift_under_load == 0, d);
    }

    /* A7 GPS loss mid-program: dead reckoning with the learned current, on to the next waypoints */
    {
        sbsim_t x;
        prog_wp_t w[5];
        sbsim_init(&x, 208u, 90.0f, 0.0f, NULL);
        x.w.current_e = 0.3f;
        x.w.current_n = -0.5f;
        sbsim_run(&x, 3000u, NULL, NULL);
        int n = prog5(&x, w);
        w[2].action = WP_ACT_CONTINUE;              /* no stop on the way */
        for (int i = 0; i < n; i++) w[i].radius_m = 60.0f;
        auto_go(&x, w, n, PROG_END_STOP);
        bool failed = false, in_dr = false;
        uint8_t wp_at_fail = 0;
        float cur_conf = 0.0f, xte_max = 0.0f;
        while (x.t < 1500000u && sb_mode(&x.sb) == MODE_AUTO && x.sb.autop.state == AUTO_RUN) {
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            if (!failed && x.sb.autop.wp == 1u && x.sb.autop.leg.along_m > 200.0f) {
                failed = true;
                x.w.gps_fail = true;
                wp_at_fail = x.sb.autop.wp;
                cur_conf = x.sb.dr.cur_learn_s;
            }
            if (failed && x.sb.autop.in_dr) in_dr = true;
            if (failed && fabsf(x.sb.autop.leg.xte_m) > xte_max) xte_max = fabsf(x.sb.autop.leg.xte_m);
        }
        snprintf(d, sizeof d, "GPS lost on leg %u (current learned for %.0f s): dead reckoning %s, waypoints reached "
                 "%u of 5, ended %s (%s), uncertainty %.0f m, alarm %s",
                 (unsigned)wp_at_fail + 1u, cur_conf, in_dr ? "yes" : "NO", x.sb.autop.legs_done,
                 mode_name(sb_mode(&x.sb)), mode_reason_name(auto_end_why(&x)), x.sb.dr.unc_m,
                 sim_hal_alarm() ? "on" : "OFF");
        test_report("A7 GPS loss mid-program: dead reckoning with the learned drift, carries on, alarm",
                    failed && in_dr && cur_conf > 60.0f && x.sb.autop.legs_done >= 2u && sim_hal_alarm() &&
                    (x.sb.alarm_bits & SB_ALM_GPS_LOST), d);
    }

    /* A9 STANDBY button from every mode; the PC can't override STANDBY */
    {
        const sb_mode_t ms[5] = { MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING };
        int ok_n = 0;
        int len = snprintf(d, sizeof d, "STANDBY button:");
        for (int i = 0; i < 5; i++) {
            sbsim_t x;
            prog_wp_t w[5];
            sbsim_init(&x, 210u + (uint32_t)i, 90.0f, 0.0f, NULL);
            sbsim_run(&x, 3000u, NULL, NULL);
            if (ms[i] == MODE_MANUAL) { joy_set(0.0f, 0.6f, false, false); sbsim_run(&x, x.t + 20000u, NULL, NULL); }
            if (ms[i] == MODE_HDG_LOCK) { hdg_lock(&x); joy_set(0.0f, 0.6f, false, false); sbsim_run(&x, x.t + 20000u, NULL, NULL); }
            if (ms[i] == MODE_AUTO) { prog5(&x, w); auto_go(&x, w, 5, PROG_END_STOP); sbsim_run(&x, x.t + 20000u, NULL, NULL); }
            if (ms[i] == MODE_TRAINING) { sb_request_mode(&x.sb, MODE_TRAINING, WHY_PC); sbsim_run(&x, x.t + 20000u, NULL, NULL); }
            bool was = sb_mode(&x.sb) == ms[i];
            press(&x, HAL_BTN_STANDBY);
            sbsim_run(&x, x.t + 15000u, NULL, NULL);
            /* the PC tries to get going again without the operator */
            bool pc1 = sb_request_mode(&x.sb, MODE_HDG_LOCK, WHY_PC);
            bool pc2 = sb_request_mode(&x.sb, MODE_AUTO, WHY_PC);
            bool pc3 = sb_request_mode(&x.sb, MODE_MANUAL, WHY_PC);
            bool good = was && sb_mode(&x.sb) == MODE_STANDBY && !pc1 && !pc2 && !pc3 &&
                        gear_is(&x.sb.gear, GEAR_N) && sim_hal_throttle() <= 0.0f && x.w.shift_under_load == 0;
            if (good) ok_n++;
            len += snprintf(d + len, sizeof d - (size_t)len, " %s %s;", mode_name(ms[i]), good ? "ok" : "FAILED");
        }
        snprintf(d + len, sizeof d - (size_t)len, " PC requests afterwards refused");
        test_report("A9 STANDBY button works from every mode; the PC can't override it", ok_n == 5, d);
    }
}

/* -------------------------------------------------------- ground link -- */

typedef struct {
    sbsim_t     x;
    link_t      link;
    proto_rx_t  pc_rx;          /* PC side receiver */
    uint8_t     seq;
    int         acks;           /* ACK frames seen */
    uint8_t     ack_type, ack_seq, ack_result;
    int         telem;          /* telemetry frames seen */
    uint8_t     telem_mode;
    uint32_t    telem_alarms;
    int         params;
    float       param_value;
    bool        heartbeat;      /* PC sends a heartbeat every second */
    uint32_t    next_hb;
} pcsim_t;

static void pc_send(pcsim_t *p, uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t fr[PROTO_MAX_FRAME];
    size_t n = proto_encode(fr, type, ++p->seq, payload, len);
    sim_hal_link_rx_push(fr, (uint32_t)n);
}

static void pc_hook(sbsim_t *x, void *ctx)
{
    pcsim_t *p = (pcsim_t *)ctx;
    link_step(&p->link, &x->sb);
    if (p->heartbeat && x->t >= p->next_hb) {
        p->next_hb = x->t + 1000u;
        pc_send(p, PROTO_T_HEARTBEAT, NULL, 0u);
    }
    uint8_t buf[512];
    uint32_t n;
    proto_frame_t f;
    while ((n = sim_hal_link_tx_pop(buf, sizeof buf)) > 0u) proto_rx_push(&p->pc_rx, buf, n);
    while (proto_rx_next(&p->pc_rx, &f)) {
        if (f.type == PROTO_T_ACK && f.len == 3u) {
            p->acks++;
            p->ack_type = f.payload[0];
            p->ack_seq = f.payload[1];
            p->ack_result = f.payload[2];
        } else if (f.type == PROTO_T_TELEMETRY) {
            proto_rd_t r;
            proto_rd_init(&r, f.payload, f.len);
            (void)proto_rd_u32(&r);
            p->telem_mode = proto_rd_u8(&r);
            p->telem++;
            r.n = 4u + 4u + 20u + 2u + 16u + 2u + 24u + 12u;      /* alarm_bits offset */
            p->telem_alarms = proto_rd_u32(&r);
        } else if (f.type == PROTO_T_PARAM_VALUE) {
            proto_rd_t r;
            proto_rd_init(&r, f.payload, f.len);
            (void)proto_rd_u8(&r);
            p->param_value = proto_rd_f32(&r);
            p->params++;
        }
    }
}

static void pcsim_init(pcsim_t *p, uint32_t seed)
{
    memset(p, 0, sizeof *p);
    remove("nv1.bin");
    sbsim_init(&p->x, seed, 90.0f, 0.0f, NULL);
    sim_hal_link_reset();
    link_init(&p->link, &p->x.sb);
    proto_rx_init(&p->pc_rx);
}

static void pc_run(pcsim_t *p, uint32_t ms)
{
    sbsim_run(&p->x, p->x.t + ms, pc_hook, p);
}

/* Send a command, run 200 ms, return the ACK result (0xFF = no ACK). */
static uint8_t pc_cmd(pcsim_t *p, uint8_t type, const uint8_t *payload, uint8_t len)
{
    int before = p->acks;
    pc_send(p, type, payload, len);
    uint8_t seq = p->seq;
    pc_run(p, 200u);
    return (p->acks > before && p->ack_seq == seq && p->ack_type == type) ? p->ack_result : 0xFFu;
}

static uint8_t pc_mode_cmd(pcsim_t *p, sb_mode_t m)
{
    uint8_t b = (uint8_t)m;
    return pc_cmd(p, PROTO_T_SET_MODE, &b, 1u);
}

static void link_tests(void)
{
    char d[400];
    printf("\n=== Ground-station link ===\n");

    /* protocol: corrupted frames are never accepted, good ones always found */
    {
        uint8_t pl[40], fr[PROTO_MAX_FRAME], stream[4096];
        for (int i = 0; i < 40; i++) pl[i] = (uint8_t)(i * 7 + 3);
        size_t n = proto_encode(fr, 0x42u, 9u, pl, 40u);
        proto_rx_t r;
        proto_frame_t f;
        int wrong_accepted = 0, good_lost = 0;

        /* every single-bit flip in every byte, each followed by a good frame */
        for (size_t byte = 0; byte < n; byte++) {
            for (int bit = 0; bit < 8; bit++) {
                proto_rx_init(&r);
                memcpy(stream, fr, n);
                stream[byte] ^= (uint8_t)(1u << bit);
                memcpy(stream + n, fr, n);
                proto_rx_push(&r, stream, 2u * n);
                int got = 0;
                for (;;) {      /* like link.c: if bytes wait with nothing new, rescan */
                    while (proto_rx_next(&r, &f)) {
                        if (f.type != 0x42u || f.seq != 9u || f.len != 40u || memcmp(f.payload, pl, 40u) != 0)
                            wrong_accepted++;
                        got++;
                    }
                    if (proto_rx_pending(&r) == 0u) break;
                    proto_rx_stale(&r);
                }
                if (got != 1) good_lost++;
            }
        }
        /* truncated frame, then a good one */
        proto_rx_init(&r);
        memcpy(stream, fr, n / 2u);
        memcpy(stream + n / 2u, fr, n);
        proto_rx_push(&r, stream, n / 2u + n);
        int trunc_ok = 0;
        while (proto_rx_next(&r, &f)) trunc_ok += (f.type == 0x42u && memcmp(f.payload, pl, 40u) == 0);
        /* random noise with good frames mixed in */
        uint32_t rng = 99u;
        int noise_frames = 0, noise_good = 0;
        proto_rx_init(&r);
        for (int k = 0; k < 200; k++) {
            uint8_t junk[300];
            for (int i = 0; i < 300; i++) { rng = rng * 1103515245u + 12345u; junk[i] = (uint8_t)(rng >> 16); }
            if (k % 3 == 0) { junk[10] = PROTO_SYNC1; junk[11] = PROTO_SYNC2; junk[12] = 30u; }   /* fake starts */
            proto_rx_push(&r, junk, sizeof junk);
            while (proto_rx_next(&r, &f)) noise_frames++;
            proto_rx_push(&r, fr, n);
            while (proto_rx_next(&r, &f)) {
                if (f.type == 0x42u && memcmp(f.payload, pl, 40u) == 0) noise_good++;
                else noise_frames++;
            }
        }
        snprintf(d, sizeof d, "%u single-bit errors: %d wrong frames accepted, %d following good frames lost; "
                 "truncated frame then good: %d found; 60 kB noise + 200 good frames: %d false frames, %d/200 good found "
                 "(CRC errors counted %u)",
                 (unsigned)(n * 8u), wrong_accepted, good_lost, trunc_ok, noise_frames, noise_good, (unsigned)r.crc_errors);
        test_report("Protocol: corrupted frames rejected, good frames still found",
                    wrong_accepted == 0 && good_lost == 0 && trunc_ok == 1 && noise_frames == 0 && noise_good == 200, d);
    }

    /* illegal / impossible commands are refused with the right reason */
    {
        pcsim_t *p = (pcsim_t *)calloc(1, sizeof *p);
        pcsim_init(p, 130u);
        p->heartbeat = true;
        pc_run(p, 3000u);
        int bad = 0;
        char why[300] = "";
        uint8_t two[2] = { 1u, 1u }, b;
        const uint8_t P_JOYTHR = 13u;               /* "joy_max_thr_pc" */
#define EXPECT(label, got, want) do { uint8_t g_ = (got); \
        if (g_ != (want)) { bad++; snprintf(why + strlen(why), sizeof why - strlen(why), " %s=%u", label, g_); } \
        if (sb_mode(&p->x.sb) == MODE_MANUAL) bad++; } while (0)
        EXPECT("unknown", pc_cmd(p, 0x55u, NULL, 0u), PROTO_R_UNKNOWN);
        EXPECT("len", pc_cmd(p, PROTO_T_SET_MODE, two, 2u), PROTO_R_BAD_LENGTH);
        b = 9u;  EXPECT("mode9", pc_cmd(p, PROTO_T_SET_MODE, &b, 1u), PROTO_R_BAD_VALUE);
        EXPECT("to_manual", pc_mode_cmd(p, MODE_MANUAL), PROTO_R_NOT_ALLOWED);
        EXPECT("to_hdg", pc_mode_cmd(p, MODE_HDG_LOCK), PROTO_R_NOT_ALLOWED);
        EXPECT("to_auto", pc_mode_cmd(p, MODE_AUTO), PROTO_R_NOT_ALLOWED);
        EXPECT("auto_no_prog", pc_cmd(p, PROTO_T_AUTO_START, NULL, 0u), PROTO_R_NO_PROGRAM);
        b = 5u;  EXPECT("hdg+5", pc_cmd(p, PROTO_T_HEADING_ADJ, &b, 1u), PROTO_R_BAD_VALUE);
        b = 10u; EXPECT("hdg_standby", pc_cmd(p, PROTO_T_HEADING_ADJ, &b, 1u), PROTO_R_WRONG_MODE);
        b = 7u;  EXPECT("sea7", pc_cmd(p, PROTO_T_SEA_STATE, &b, 1u), PROTO_R_BAD_VALUE);
        b = 99u; EXPECT("param99", pc_cmd(p, PROTO_T_PARAM_GET, &b, 1u), PROTO_R_BAD_PARAM);
        {
            uint8_t ps[5]; proto_wr_t w;
            proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, P_JOYTHR); proto_wr_f32(&w, 500.0f);
            EXPECT("joythr500", pc_cmd(p, PROTO_T_PARAM_SET, ps, 5u), PROTO_R_BAD_VALUE);
            proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, P_JOYTHR); proto_wr_f32(&w, 30.0f);
            EXPECT("joythr30", pc_cmd(p, PROTO_T_PARAM_SET, ps, 5u), PROTO_R_OK);
            bool param_set = fabsf(p->x.sb.cfg.joy_max_throttle_pct - 30.0f) < 1e-4f && fabsf(p->param_value - 30.0f) < 1e-4f;
            if (!param_set) { bad++; strcat(why, " param_not_applied"); }
            p->x.sb.cfg.joy_max_throttle_pct = 100.0f;
        }
        /* program upload: a bad one (speed 40 kn), then a good one */
        {
            prog_wp_t w5[5];
            prog5(&p->x, w5);
            uint8_t pb[2] = { 5u, (uint8_t)PROG_END_STOP };
            for (int pass = 0; pass < 2; pass++) {
                EXPECT("prog_begin", pc_cmd(p, PROTO_T_PROG_BEGIN, pb, 2u), PROTO_R_OK);
                for (uint8_t i = 0; i < 5u; i++) {
                    uint8_t pw[18]; proto_wr_t w; proto_wr_init(&w, pw, sizeof pw);
                    proto_wr_u8(&w, i); proto_wr_i32(&w, w5[i].lat_e7); proto_wr_i32(&w, w5[i].lon_e7);
                    proto_wr_f32(&w, (pass == 0 && i == 2u) ? 40.0f : w5[i].speed_kn);
                    proto_wr_f32(&w, w5[i].radius_m); proto_wr_u8(&w, w5[i].action);
                    EXPECT("prog_wp", pc_cmd(p, PROTO_T_PROG_WP, pw, 18u), PROTO_R_OK);
                }
                EXPECT(pass == 0 ? "prog_bad" : "prog_good", pc_cmd(p, PROTO_T_PROG_COMMIT, NULL, 0u),
                       pass == 0 ? PROTO_R_BAD_PROGRAM : PROTO_R_OK);
            }
            if (!p->x.sb.autop.prog_ok || p->x.sb.autop.prog.n != 5u) { bad++; strcat(why, " prog_not_stored"); }
        }
        bool still_standby = sb_mode(&p->x.sb) == MODE_STANDBY;
        EXPECT("training", pc_mode_cmd(p, MODE_TRAINING), PROTO_R_OK);
        EXPECT("train->hdg", pc_mode_cmd(p, MODE_HDG_LOCK), PROTO_R_NOT_ALLOWED);
        bool still_training = sb_mode(&p->x.sb) == MODE_TRAINING;
        EXPECT("standby", pc_mode_cmd(p, MODE_STANDBY), PROTO_R_OK);
        pc_run(p, 5000u);
        sim_hal_press(HAL_BTN_HDG_LOCK);            /* HDG LOCK on the panel */
        pc_run(p, 200u);
        {
            uint8_t ps[5]; proto_wr_t w; proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, P_JOYTHR); proto_wr_f32(&w, 30.0f);
            EXPECT("param_in_hdg", pc_cmd(p, PROTO_T_PARAM_SET, ps, 5u), PROTO_R_STANDBY_ONLY);
        }
        float tgt0 = p->x.sb.ap.target_heading;
        b = 10u; EXPECT("hdg+10", pc_cmd(p, PROTO_T_HEADING_ADJ, &b, 1u), PROTO_R_OK);
        b = (uint8_t)(int8_t)-1; EXPECT("hdg-1", pc_cmd(p, PROTO_T_HEADING_ADJ, &b, 1u), PROTO_R_OK);
        float dtgt = hdg_err(p->x.sb.ap.target_heading, tgt0);
        /* AUTO from the PC: start + joystick CONFIRM */
        EXPECT("auto_start", pc_cmd(p, PROTO_T_AUTO_START, NULL, 0u), PROTO_R_OK);
        sim_hal_joystick_confirm(true);
        pc_run(p, 100u);
        sim_hal_joystick_confirm(false);
        pc_run(p, 300u);
        bool in_auto = sb_mode(&p->x.sb) == MODE_AUTO;
        int pc_logged = 0;
        for (uint32_t i = 0; i < mode_log_size(&p->x.sb.modes); i++)
            if (mode_log_get(&p->x.sb.modes, i)->why == WHY_PC) pc_logged++;
#undef EXPECT
        pc_run(p, 10000u);
        int t0 = p->telem;
        pc_run(p, 10000u);
        int telem10 = p->telem - t0;
        snprintf(d, sizeof d, "wrong answers %d%s; refused commands changed nothing: %s/%s; heading +10 -1 -> target %+.0f deg; "
                 "AUTO via PC + CONFIRM %s; PC mode requests logged %d; telemetry %d frames in 10 s, shows %s; NACKs sent %u",
                 bad, why, still_standby ? "yes" : "NO", still_training ? "yes" : "NO", dtgt,
                 in_auto ? "yes" : "NO", pc_logged, telem10, mode_name((sb_mode_t)p->telem_mode), (unsigned)p->link.nacks);
        test_report("Link: illegal commands refused with a reason, legal ones accepted",
                    bad == 0 && still_standby && still_training && fabsf(dtgt - 9.0f) < 0.01f && in_auto &&
                    pc_logged >= 6 && telem10 >= 99 && telem10 <= 101 && p->telem_mode == MODE_AUTO, d);
        free(p);
    }

    /* PC link lost: alarm after 3 s, mode stays */
    {
        pcsim_t *p = (pcsim_t *)calloc(1, sizeof *p);
        pcsim_init(p, 131u);
        p->heartbeat = true;
        pc_run(p, 3000u);
        auto_to(&p->x, 6000.0f, 90.0f, 12.0f);
        pc_run(p, 30000u);
        p->heartbeat = false;               /* cable pulled */
        uint32_t t_off = p->next_hb - 1000u;    /* last heartbeat */
        float t_alarm = -1.0f;
        for (int k = 0; k < 400; k++) {
            pc_run(p, 50u);
            if (t_alarm < 0.0f && (p->x.sb.alarm_bits & SB_ALM_PC_LINK)) t_alarm = (float)(p->x.t - t_off) * 0.001f;
        }
        bool alarm_out = sim_hal_alarm();
        sb_mode_t m_lost = sb_mode(&p->x.sb);
        float v_lost = p->x.w.speed_kn;
        p->heartbeat = true;
        p->next_hb = p->x.t;
        pc_run(p, 2000u);
        bool cleared = !(p->x.sb.alarm_bits & SB_ALM_PC_LINK);
        snprintf(d, sizeof d, "alarm %.2f s after the last heartbeat (limit 3 s), alarm output %s; mode after 20 s without PC: "
                 "%s at %.1f kn; PC back: alarm %s",
                 t_alarm, alarm_out ? "on" : "OFF", mode_name(m_lost), v_lost, cleared ? "cleared" : "STILL ON");
        test_report("Link lost > 3 s: alarm 'PC link lost', mode kept",
                    t_alarm >= 3.0f && t_alarm <= 3.2f && alarm_out && m_lost == MODE_AUTO && v_lost > 5.0f && cleared, d);
        free(p);
    }

    /* PC during the joystick override */
    {
        pcsim_t *p = (pcsim_t *)calloc(1, sizeof *p);
        pcsim_init(p, 132u);
        p->heartbeat = true;
        pc_run(p, 3000u);
        auto_to(&p->x, 6000.0f, 90.0f, 12.0f);
        pc_run(p, 30000u);
        joy_set(-0.5f, 0.3f, false, false); /* grab the stick */
        pc_run(p, 1000u);
        bool in_man = sb_mode(&p->x.sb) == MODE_MANUAL;
        uint8_t r_sb = pc_mode_cmd(p, MODE_STANDBY);
        uint8_t r_tr = pc_mode_cmd(p, MODE_TRAINING);
        uint8_t b = 10u;
        uint8_t r_hd = pc_cmd(p, PROTO_T_HEADING_ADJ, &b, 1u);
        uint8_t r_pa = pc_cmd(p, PROTO_T_AUTO_PAUSE, NULL, 0u);
        uint8_t r_rs = pc_cmd(p, PROTO_T_AUTO_RESUME, NULL, 0u);    /* armed, but needs the stick centred + CONFIRM */
        sim_hal_joystick_confirm(true);
        pc_run(p, 100u);
        sim_hal_joystick_confirm(false);
        pc_run(p, 300u);
        bool still = sb_mode(&p->x.sb) == MODE_MANUAL;
        snprintf(d, sizeof d, "in MANUAL %s; PC STANDBY -> %u, TRAINING -> %u, heading -> %u, pause -> %u "
                 "(4 = joystick in control); resume request -> %u but CONFIRM with the stick deflected refused, "
                 "still MANUAL %s",
                 in_man ? "yes" : "NO", r_sb, r_tr, r_hd, r_pa, r_rs, still ? "yes" : "NO");
        test_report("Link: PC refused while the joystick is in control",
                    in_man && r_sb == PROTO_R_JOYSTICK && r_tr == PROTO_R_JOYSTICK && r_hd == PROTO_R_JOYSTICK &&
                    r_pa == PROTO_R_JOYSTICK && still, d);
        free(p);
    }
}

/* --------------------------------------------------- section B: power limit -- */

/* Put the boat into `m` (for the CONFIRM / clamp tests). */
static bool goto_mode(sbsim_t *x, sb_mode_t m)
{
    prog_wp_t w[5];
    switch (m) {
    case MODE_MANUAL:   joy_set(0.0f, 0.5f, false, false); sbsim_run(x, x->t + 1000u, NULL, NULL);
                        joy_set(0.0f, 0.0f, false, false); sbsim_run(x, x->t + 500u, NULL, NULL); break;
    case MODE_HDG_LOCK: press(x, HAL_BTN_HDG_LOCK); break;
    case MODE_AUTO:     prog5(x, w); auto_go(x, w, 5, PROG_END_STOP); break;
    case MODE_TRAINING: sb_request_mode(&x->sb, MODE_TRAINING, WHY_PC); break;
    default: break;
    }
    return sb_mode(&x->sb) == m;
}

static void powerlim_tests(void)
{
    char d[600];
    printf("\n=== Section B: power limit ===\n");
    const float F0 = 30.0f, R0 = 5.0f;

    /* B1 startup 30 / 5, also after a reset at 100 % */
    {
        sbsim_t x;
        sbsim_init(&x, 301u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        float f0 = x.sb.plim.fwd_pct, r0 = x.sb.plim.rev_pct;
        power_full(&x);
        float f1 = x.sb.plim.fwd_pct, r1 = x.sb.plim.rev_pct;
        sb_config_t c = x.sb.cfg;
        sb_init(&x.sb, &c);                         /* reset (watchdog / power cut) */
        float f2 = x.sb.plim.fwd_pct, r2 = x.sb.plim.rev_pct;
        snprintf(d, sizeof d, "power-up %.0f / %.0f %%; raised to %.0f / %.0f %%; after a reset %.0f / %.0f %%",
                 f0, r0, f1, r1, f2, r2);
        test_report("B1 startup limit 30 % fwd / 5 % rev, also after a reset at 100 %",
                    f0 == F0 && r0 == R0 && f1 == 100.0f && r1 == 15.0f && f2 == F0 && r2 == R0, d);
    }

    /* B2 the raise sequence: code + CONFIRM within 10 s, one step at a time */
    {
        sbsim_t x;
        uint16_t code;
        sbsim_init(&x, 302u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        /* no CONFIRM */
        sb_power_code(&x.sb, 0u, &code);
        sb_result_t r_ok = sb_power_raise(&x.sb, 0u, code);
        sbsim_run(&x, x.t + 11000u, NULL, NULL);
        bool no_conf = x.sb.plim.fwd_pct == F0 && x.sb.confirm.kind == CONF_NONE;
        /* late CONFIRM */
        sb_power_code(&x.sb, 0u, &code);
        sb_power_raise(&x.sb, 0u, code);
        sbsim_run(&x, x.t + 11000u, NULL, NULL);
        bool late = !confirm_press(&x) && x.sb.plim.fwd_pct == F0;
        /* wrong code (and the code is used up: the right one afterwards fails too) */
        sb_power_code(&x.sb, 0u, &code);
        sb_result_t r_wrong = sb_power_raise(&x.sb, 0u, (uint16_t)(code == 9999u ? 1000u : code + 1u));
        sb_result_t r_again = sb_power_raise(&x.sb, 0u, code);
        bool wrong = r_wrong == SB_ERR_WRONG_CODE && r_again == SB_ERR_WRONG_CODE && !confirm_press(&x) &&
                     x.sb.plim.fwd_pct == F0;
        /* expired code (60 s) */
        sb_power_code(&x.sb, 0u, &code);
        sbsim_run(&x, x.t + 61000u, NULL, NULL);
        bool expired = sb_power_raise(&x.sb, 0u, code) == SB_ERR_WRONG_CODE;
        /* correct: one step */
        bool one = power_step(&x, 0u) && x.sb.plim.fwd_pct == 50.0f;
        /* a second CONFIRM press without a new sequence does nothing */
        bool second = !confirm_press(&x) && x.sb.plim.fwd_pct == 50.0f;
        bool next = power_step(&x, 0u) && x.sb.plim.fwd_pct == 75.0f;
        power_step(&x, 0u);
        bool top = x.sb.plim.fwd_pct == 100.0f && sb_power_code(&x.sb, 0u, &code) == SB_ERR_AT_MAX;
        bool rev = power_step(&x, 1u) && x.sb.plim.rev_pct == 10.0f && power_step(&x, 1u) && x.sb.plim.rev_pct == 15.0f &&
                   sb_power_code(&x.sb, 1u, &code) == SB_ERR_AT_MAX;
        /* the log has every change and attempt */
        int changes = 0, refused = 0;
        for (uint32_t i = 0; i < plim_log_size(&x.sb.plim); i++) {
            const plim_log_t *e = plim_log_get(&x.sb.plim, i);
            if (e->result == PLIM_OK && e->new_fwd + e->new_rev != e->old_fwd + e->old_rev) changes++;
            if (e->result != PLIM_OK) refused++;
        }
        snprintf(d, sizeof d, "code accepted %d; no CONFIRM: unchanged %s; late CONFIRM refused %s; wrong code refused, "
                 "code used up %s; expired code refused %s; correct -> one step 30->50 %s; extra CONFIRM does nothing %s; "
                 "next step needs a new sequence -> 75 %s; at 100 no more %s; reverse 5->10->15 then max %s; "
                 "log: %d changes, %d refusals",
                 (int)r_ok, no_conf ? "yes" : "NO", late ? "yes" : "NO", wrong ? "yes" : "NO", expired ? "yes" : "NO",
                 one ? "yes" : "NO", second ? "yes" : "NO", next ? "yes" : "NO", top ? "yes" : "NO", rev ? "yes" : "NO",
                 changes, refused);
        test_report("B2 raise: code + CONFIRM within 10 s, one step per sequence; late / wrong / expired refused",
                    r_ok == SB_OK && no_conf && late && wrong && expired && one && second && next && top && rev &&
                    changes == 5 && refused >= 3, d);
    }

    /* B3 CONFIRM rules for a raise; CONFIRM never changes the mode */
    {
        sbsim_t x;
        uint16_t code;
        sbsim_init(&x, 303u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        sb_power_code(&x.sb, 0u, &code);
        sb_power_raise(&x.sb, 0u, code);
        joy_set(0.12f, 0.0f, false, false);         /* off centre, inside the deadband */
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool deflected = !confirm_press(&x) && x.sb.plim.fwd_pct == F0;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 11000u, NULL, NULL);
        /* joystick faulted: no raise possible at all */
        sim_hal_set_joystick(true, joy_raw(0.0f), 4095u, false, false);
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        sb_result_t r_fault = sb_power_code(&x.sb, 0u, &code);
        bool faulted = r_fault == SB_ERR_JOY_NOT_READY && !confirm_press(&x);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        sb_result_t r_alarm = sb_power_code(&x.sb, 0u, &code);     /* the joystick fault alarm is still active */
        press(&x, HAL_BTN_STANDBY);                 /* acknowledge */
        /* nothing pending: nothing happens */
        uint32_t ign = x.sb.confirm.ignored;
        bool nothing = !confirm_press(&x) && x.sb.confirm.ignored == ign + 1u && x.sb.plim.fwd_pct == F0;
        snprintf(d, sizeof d, "stick off centre: refused %s; joystick faulted: code -> %d (not ready = %d), CONFIRM nothing %s; "
                 "alarm active -> %d (alarm = %d); nothing pending: ignored %s",
                 deflected ? "yes" : "NO", (int)r_fault, (int)SB_ERR_JOY_NOT_READY, faulted ? "yes" : "NO",
                 (int)r_alarm, (int)SB_ERR_ALARM, nothing ? "yes" : "NO");
        test_report("B3a power raise CONFIRM: stick off centre / joystick faulted / nothing pending -> refused or nothing",
                    deflected && faulted && r_alarm == SB_ERR_ALARM && nothing, d);

        const sb_mode_t ms[5] = { MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING };
        int same = 0;
        int len = snprintf(d, sizeof d, "raise CONFIRMed in:");
        for (int i = 0; i < 5; i++) {
            sbsim_t y;
            sbsim_init(&y, 310u + (uint32_t)i, 90.0f, 0.0f, NULL);
            sbsim_run(&y, 3000u, NULL, NULL);
            bool in = goto_mode(&y, ms[i]);
            sbsim_run(&y, y.t + 2000u, NULL, NULL);
            sb_power_lower(&y.sb);
            uint32_t log_n = mode_log_size(&y.sb.modes);
            float before = y.sb.plim.fwd_pct;
            bool raised = power_step(&y, 0u);
            bool ok = in && raised && sb_mode(&y.sb) == ms[i] && mode_log_size(&y.sb.modes) == log_n &&
                      y.sb.plim.fwd_pct > before;
            if (ok) same++;
            len += snprintf(d + len, sizeof d - (size_t)len, " %s %s;", mode_name(ms[i]), ok ? "mode unchanged" : "FAILED");
        }
        test_report("B3b CONFIRM never changes the mode (power raise in every mode)", same == 5, d);
    }

    /* B4 every auto-drop condition */
    {
        int ok_n = 0, n = 0;
        int len = snprintf(d, sizeof d, "back to 30 / 5 on:");
#define DROP_CASE(label, setup) do { sbsim_t y; sbsim_init(&y, 320u + (uint32_t)n, 90.0f, 0.0f, NULL); \
        sbsim_run(&y, 3000u, NULL, NULL); bool pre = true; setup; n++; \
        bool ok_ = pre && y.sb.plim.fwd_pct == F0 && y.sb.plim.rev_pct == R0; if (ok_) ok_n++; \
        len += snprintf(d + len, sizeof d - (size_t)len, " %s %s;", label, ok_ ? "ok" : "FAILED"); } while (0)
        DROP_CASE("PC lower", { power_full(&y); pre = plim_raised(&y.sb.plim); sb_power_lower(&y.sb); });
        DROP_CASE("STANDBY button", { power_full(&y); pre = plim_raised(&y.sb.plim); press(&y, HAL_BTN_STANDBY); });
        DROP_CASE("TAKE", { power_full(&y); hdg_lock(&y); pre = plim_raised(&y.sb.plim);
                            joy_set(0.0f, 0.0f, true, false); sbsim_run(&y, y.t + 100u, NULL, NULL); });
        DROP_CASE("override (stick X)", { hdg_lock(&y); pre = plim_raised(&y.sb.plim);
                            joy_set(0.5f, 0.0f, false, false); sbsim_run(&y, y.t + 500u, NULL, NULL);
                            pre = pre && sb_mode(&y.sb) == MODE_MANUAL; });
        DROP_CASE("any alarm (gear fault)", { y.w.neutral_switch = true; hdg_lock(&y);
                            sb_test_drive(&y.sb, true, GEAR_F, 10.0f); sbsim_run(&y, y.t + 8000u, NULL, NULL);
                            pre = plim_raised(&y.sb.plim);
                            y.w.servo_stuck = true; sb_test_drive(&y.sb, true, GEAR_R, 10.0f);
                            sbsim_run(&y, y.t + 8000u, NULL, NULL); pre = pre && gear_fault(&y.sb.gear); });
        DROP_CASE("GPS lost", { hdg_lock(&y); pre = plim_raised(&y.sb.plim); y.w.gps_fail = true;
                            sbsim_run(&y, y.t + 6000u, NULL, NULL); });
        DROP_CASE("compass fault", { joy_set(0.0f, 0.4f, false, false); sbsim_run(&y, y.t + 1000u, NULL, NULL);
                            joy_set(0.0f, 0.0f, false, false); sbsim_run(&y, y.t + 500u, NULL, NULL);
                            power_full(&y); pre = plim_raised(&y.sb.plim) && sb_mode(&y.sb) == MODE_MANUAL;
                            sim_hal_fail_heading(true); sbsim_run(&y, y.t + 4000u, NULL, NULL); });
        DROP_CASE("joystick fault", { power_full(&y); pre = plim_raised(&y.sb.plim);
                            sim_hal_set_joystick(true, 0u, joy_raw(0.0f), false, false); sbsim_run(&y, y.t + 200u, NULL, NULL); });
        DROP_CASE("entering TRAINING", { power_full(&y); pre = plim_raised(&y.sb.plim);
                            sb_request_mode(&y.sb, MODE_TRAINING, WHY_PC); sbsim_run(&y, y.t + 100u, NULL, NULL); });
        DROP_CASE("2 min below 3 kn", { power_full(&y); pre = plim_raised(&y.sb.plim);
                            sbsim_run(&y, y.t + 110000u, NULL, NULL); pre = pre && plim_raised(&y.sb.plim);
                            sbsim_run(&y, y.t + 15000u, NULL, NULL); });
        {   /* PC link lost (through the link) */
            pcsim_t *p = (pcsim_t *)calloc(1, sizeof *p);
            pcsim_init(p, 330u);
            p->heartbeat = true;
            pc_run(p, 3000u);
            power_full(&p->x);
            bool pre = plim_raised(&p->x.sb.plim);
            p->heartbeat = false;
            pc_run(p, 5000u);
            bool ok_ = pre && p->x.sb.plim.fwd_pct == F0;
            n++;
            if (ok_) ok_n++;
            const plim_log_t *e = plim_log_get(&p->x.sb.plim, plim_log_size(&p->x.sb.plim) - 1u);
            len += snprintf(d + len, sizeof d - (size_t)len, " PC link lost %s (logged as %s);", ok_ ? "ok" : "FAILED",
                            plim_source_name((plim_source_t)e->source));
            free(p);
        }
#undef DROP_CASE
        test_report("B4 every auto-drop condition -> startup limit at once", ok_n == n, d);
    }

    /* B5 100 % commanded in every mode at limit 30 %: never more at the PWM output */
    {
        const sb_mode_t ms[5] = { MODE_STANDBY, MODE_MANUAL, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING };
        float worst = 0.0f;
        uint16_t worst_us = 0u;
        int len = snprintf(d, sizeof d, "max output with 100 %% commanded:");
        bool all_in = true;
        for (int i = 0; i < 5; i++) {
            sbsim_t y;
            sbsim_init(&y, 340u + (uint32_t)i, 90.0f, 0.0f, NULL);
            sbsim_run(&y, 3000u, NULL, NULL);
            bool in = goto_mode(&y, ms[i]);
            sb_power_lower(&y.sb);                  /* AUTO / HDG LOCK helpers raised it: back to 30 */
            if (ms[i] == MODE_MANUAL || ms[i] == MODE_HDG_LOCK) joy_set(0.0f, 1.0f, false, false);
            sb_test_drive(&y.sb, ms[i] != MODE_MANUAL && ms[i] != MODE_HDG_LOCK, GEAR_F, 100.0f);
            float mx = 0.0f;
            uint16_t mus = 0u;
            for (int k = 0; k < 1200; k++) {        /* 60 s */
                sbsim_run(&y, y.t + 50u, NULL, NULL);
                if (sim_hal_throttle() > mx) mx = sim_hal_throttle();
                if (sim_hal_throttle_us() > mus) mus = sim_hal_throttle_us();
            }
            if (ms[i] != MODE_STANDBY && !in) all_in = false;
            if (mx > worst) worst = mx;
            if (mus > worst_us) worst_us = mus;
            len += snprintf(d + len, sizeof d - (size_t)len, " %s %.1f %% (%u us);", mode_name(ms[i]), mx, mus);
        }
        test_report("B5 100 % commanded in every mode at limit 30 %: output never above 30 % (1300 us)",
                    all_in && worst <= F0 + 0.01f && worst_us <= 1300u, d);
    }

    /* B6 AUTO leg speed clamped by the power limit, telemetry flag */
    {
        sbsim_t x;
        static program_t p;
        prog_error_t err;
        uint8_t bad;
        sbsim_init(&x, 350u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        prog_wp_t w = wp_world(&x, x.w.north_m, x.w.east_m + 8000.0, 20.0f, 25.0f, WP_ACT_STOP);
        prog_make(&p, &w, 1, PROG_END_STOP);
        sb_program_set(&x.sb, &p, &err, &bad);
        sb_auto_start(&x.sb);
        confirm_press(&x);                          /* AUTO at the startup limit */
        sbsim_run(&x, x.t + 90000u, NULL, NULL);
        float v30 = world_sog_kn(&x.w), thr30 = sim_hal_throttle();
        bool flag30 = x.sb.speed_limited;
        for (int i = 0; i < 3; i++) power_step(&x, 0u);
        sbsim_run(&x, x.t + 90000u, NULL, NULL);
        float v100 = world_sog_kn(&x.w);
        bool flag100 = x.sb.speed_limited;
        snprintf(d, sizeof d, "20 kn leg at limit 30 %%: %.1f kn, throttle %.0f %%, 'speed limited by power limit' %s; "
                 "after raising to 100 %%: %.1f kn, flag %s", v30, thr30, flag30 ? "on" : "OFF", v100,
                 flag100 ? "ON" : "off");
        test_report("B6 AUTO leg speed clamped by the power limit, shown in telemetry",
                    sb_mode(&x.sb) == MODE_AUTO && thr30 <= F0 + 0.01f && v30 < 15.0f && flag30 && v100 > 19.0f &&
                    !flag100, d);
    }
}

/* ------------------------------------------------------ section C: hardening -- */

typedef struct {
    int      kind;              /* 1 = throttle 90 %, 2 = gear to R at throttle */
    uint32_t at_ms;             /* inject from this time */
    bool     injected;
    uint32_t t_inj;
} inject_t;

/* A bug in core 0: after its normal step it commands something unsafe. */
static void inject_hook(sbsim_t *x, void *ctx)
{
    inject_t *j = (inject_t *)ctx;
    if (x->t < j->at_ms) return;
    if (!j->injected) { j->injected = true; j->t_inj = x->t; }
    if (j->kind == 1) {
        hal_set_throttle_pwm(1900u);                    /* 90 % past every limit */
        g_mon.throttle_pct = 90.0f;
    } else {
        hal_set_gear_pwm(x->sb.cfg.gear.pwm_r_us);      /* F -> R with the throttle open */
        g_mon.gear_pwm_us = x->sb.cfg.gear.pwm_r_us;
    }
}

typedef struct { float t_react; float thr_after; uint16_t gear_after; bool alarm, standby, relay_open; mon_reason_t why; } react_t;

static react_t watch_reaction(sbsim_t *x, uint32_t t0, sb_hook_t hook, void *ctx, uint32_t dur)
{
    react_t r;
    memset(&r, 0, sizeof r);
    r.t_react = -1.0f;
    while (x->t < t0 + dur) {
        sbsim_run(x, x->t + 10u, hook, ctx);
        if (r.t_react < 0.0f && hal_outputs_forced()) r.t_react = (float)(x->t - t0);
    }
    r.thr_after = sim_hal_throttle();
    r.gear_after = sim_hal_gear_pwm();
    r.alarm = sim_hal_alarm();
    r.standby = sb_mode(&x->sb) == MODE_STANDBY && (x->sb.alarm_bits & SB_ALM_MONITOR);
    r.relay_open = !sim_hal_safety_relay();
    r.why = x->mon.reason;
    return r;
}

static void hardening_tests(void)
{
    char d[500];
    printf("\n=== Section C: hardening (watchdog, CPU1 monitor, neutral-start) ===\n");

    /* C1 injected 90 % throttle at limit 30 % */
    {
        sbsim_t x;
        inject_t j = { 1, 0u, false, 0u };
        sbsim_init(&x, 401u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        joy_set(0.0f, 0.6f, false, false);          /* MANUAL, ahead at the 30 % limit */
        sbsim_run(&x, 20000u, NULL, NULL);
        float before = sim_hal_throttle();
        j.at_ms = x.t + 1u;
        react_t r = watch_reaction(&x, x.t, inject_hook, &j, 5000u);
        r.t_react -= (float)(j.t_inj - (j.at_ms - 1u));     /* from the moment it was injected */
        snprintf(d, sizeof d, "throttle %.0f %% at limit %.0f %%; injected 90 %%: monitor (%s) forced idle after %.0f ms "
                 "(sim check every 10 ms; board 1 ms), throttle now %.0f %%, gear %u us, alarm %s, core 0 -> STANDBY %s, "
                 "safety relay open %s",
                 before, x.sb.plim.fwd_pct, monitor_reason_name(r.why), r.t_react, r.thr_after, r.gear_after,
                 r.alarm ? "on" : "OFF", r.standby ? "yes" : "NO", r.relay_open ? "yes" : "NO");
        test_report("C1 monitor catches 90 % throttle at limit 30 %: idle + neutral + alarm",
                    r.why == MON_TRIP_THROTTLE && r.t_react >= 0.0f && r.t_react <= 20.0f && r.thr_after <= 0.0f &&
                    r.gear_after == x.sb.cfg.gear.pwm_n_us && r.alarm && r.standby && r.relay_open, d);
    }

    /* C2 gear change at high throttle */
    {
        sbsim_t x;
        inject_t j = { 2, 0u, false, 0u };
        sbsim_init(&x, 402u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 60.0f);
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        float before = sim_hal_throttle();
        j.at_ms = x.t + 1u;
        react_t r = watch_reaction(&x, x.t, inject_hook, &j, 5000u);
        r.t_react -= (float)(j.t_inj - (j.at_ms - 1u));
        snprintf(d, sizeof d, "throttle %.0f %%, gear servo commanded to R: monitor (%s) after %.0f ms, throttle %.0f %%, "
                 "gear %u us (neutral %u), alarm %s, STANDBY %s, gearbox shifted under load %d",
                 before, monitor_reason_name(r.why), r.t_react, r.thr_after, r.gear_after, x.sb.cfg.gear.pwm_n_us,
                 r.alarm ? "on" : "OFF", r.standby ? "yes" : "NO", x.w.shift_under_load);
        test_report("C2 monitor catches a gear change at high throttle",
                    r.why == MON_TRIP_GEAR && r.t_react >= 0.0f && r.t_react <= 20.0f && r.thr_after <= 0.0f &&
                    r.gear_after == x.sb.cfg.gear.pwm_n_us && r.alarm && r.standby, d);
    }

    /* C3 core 0 freezes (watchdog off): the monitor's heartbeat check */
    {
        sbsim_t x;
        sbsim_init(&x, 403u, 90.0f, 0.0f, NULL);
        x.wdt_ms = 0u;
        sbsim_run(&x, 3000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 70.0f);
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        float before = sim_hal_throttle();
        x.frozen = true;                            /* the control loop hangs, outputs stay as they were */
        uint32_t t0 = x.last_kick;                  /* the last heartbeat */
        react_t r = watch_reaction(&x, t0, NULL, NULL, 3000u);
        snprintf(d, sizeof d, "core 0 frozen at %.0f %% throttle: monitor (%s) forced idle + neutral after %.0f ms "
                 "(heartbeat limit %u ms), throttle %.0f %%, gear %u us, alarm %s",
                 before, monitor_reason_name(r.why), r.t_react, MON_HB_MS, r.thr_after, r.gear_after,
                 r.alarm ? "on" : "OFF");
        test_report("C3 frozen core 0 (no watchdog): monitor heartbeat trip -> idle + neutral + alarm",
                    r.why == MON_TRIP_HEARTBEAT && r.t_react >= (float)MON_HB_MS && r.t_react <= (float)MON_HB_MS + 30.0f &&
                    r.thr_after <= 0.0f && r.gear_after == x.sb.cfg.gear.pwm_n_us && r.alarm, d);
    }

    /* C4 core 0 freezes with the watchdog on: reset, safe start again */
    {
        sbsim_t x;
        sbsim_init(&x, 404u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 70.0f);
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        float v0 = x.w.speed_kn;
        x.frozen = true;
        uint32_t t0 = x.last_kick;                  /* the last watchdog kick */
        float thr_reset = -1.0f;
        while (x.t < t0 + 15000u) {
            sbsim_run(&x, x.t + 10u, NULL, NULL);
            if (x.resets && thr_reset < 0.0f) thr_reset = sim_hal_throttle();
        }
        uint32_t raw;
        hal_reset_t why = hal_reset_reason(&raw);
        snprintf(d, sizeof d, "frozen at %.1f kn, 70 %%: watchdog reset after %.0f ms (timeout %u ms), throttle at reset "
                 "%.0f %%; after the reboot: reset reason %s, mode %s, power limit %.0f %%, gear %s, speed %.1f kn, armed %s",
                 v0, x.resets ? (float)(x.t_reset - t0) : -1.0f, 100u, thr_reset,
                 why == HAL_RESET_SYSTEM ? "system (watchdog)" : "?", mode_name(sb_mode(&x.sb)), x.sb.plim.fwd_pct,
                 gear_name(x.w.engaged), x.w.speed_kn, x.sb.armed ? "yes" : "no");
        test_report("C4 frozen core 0 with watchdog: reset, outputs stop, restarts in STANDBY at 30 %",
                    x.resets == 1u && x.t_reset - t0 >= 100u && x.t_reset - t0 <= 120u && thr_reset <= 0.0f &&
                    why == HAL_RESET_SYSTEM &&
                    sb_mode(&x.sb) == MODE_STANDBY && x.sb.plim.fwd_pct == 30.0f && x.w.engaged == GEAR_N &&
                    sim_hal_throttle() <= 0.0f, d);
    }

    /* C5 neutral-start interlock: gear not in neutral at power-up */
    {
        sbsim_t x;
        sbsim_init(&x, 405u, 90.0f, 0.0f, NULL);
        x.w.neutral_switch = true;
        x.w.engaged = GEAR_F;                       /* gear servo jammed in forward */
        x.w.gear_pos_us = 1900.0f;
        x.w.servo_stuck = true;
        sbsim_run(&x, 4000u, NULL, NULL);
        bool disarmed = !x.sb.armed && !sim_hal_safety_relay() && (x.sb.alarm_bits & SB_ALM_NOT_ARMED) &&
                        sim_hal_alarm();
        joy_set(0.0f, 0.6f, false, false);          /* try to drive */
        sbsim_run(&x, 6000u, NULL, NULL);
        bool refused = sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f && !hdg_lock(&x);
        joy_set(0.0f, 0.0f, false, false);
        x.w.servo_stuck = false;                    /* fixed: the servo goes to neutral */
        sbsim_run(&x, 9000u, NULL, NULL);
        bool armed = x.sb.armed && sim_hal_safety_relay();
        snprintf(d, sizeof d, "gear stuck in F at power-up: outputs disarmed + alarm %s; stick / HDG LOCK refused %s; "
                 "servo free -> neutral confirmed -> armed %s",
                 disarmed ? "yes" : "NO", refused ? "yes" : "NO", armed ? "yes" : "NO");
        test_report("C5 neutral-start: gear not neutral at power-up -> outputs disarmed + alarm",
                    disarmed && refused && armed, d);
    }

    /* C6 neutral-start interlock: joystick not centred at power-up */
    {
        sbsim_t x;
        sbsim_init(&x, 406u, 90.0f, 0.0f, NULL);
        joy_set(0.0f, 0.7f, false, false);          /* someone leans on the stick */
        sbsim_run(&x, 4000u, NULL, NULL);
        bool disarmed = !x.sb.armed && (x.sb.alarm_bits & SB_ALM_NOT_ARMED) && sb_mode(&x.sb) == MODE_STANDBY &&
                        sim_hal_throttle() <= 0.0f && !sim_hal_safety_relay();
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, 5500u, NULL, NULL);
        bool armed = x.sb.armed;
        snprintf(d, sizeof d, "stick held at power-up: disarmed + alarm %s; centred -> armed %s",
                 disarmed ? "yes" : "NO", armed ? "yes" : "NO");
        test_report("C6 neutral-start: joystick not centred at power-up -> outputs disarmed + alarm",
                    disarmed && armed, d);
    }
}

/* --------------------------------------------------------- section E: timing -- */

static void timing_tests(void)
{
    char d[400];
    printf("\n=== Section E: cycle timing ===\n");

    /* E1 a control cycle longer than its period: counted + alarm */
    {
        sbsim_t x;
        sbsim_init(&x, 501u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        uint32_t ov0 = g_timing.core0.overruns;
        bool alarm0 = (x.sb.alarm_bits & SB_ALM_OVERRUN) != 0u;
        x.inject_us = 62000u;                       /* one step takes 62 ms (period 50 ms) */
        sbsim_run(&x, x.t + 50u, NULL, NULL);
        x.inject_us = 0u;
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        uint32_t ov1 = g_timing.core0.overruns;
        bool alarm1 = (x.sb.alarm_bits & SB_ALM_OVERRUN) != 0u && sim_hal_alarm();
        snprintf(d, sizeof d, "normal steps: %u overruns, alarm %s; one 62 ms step: overruns %u, alarm %s, "
                 "all-time max %u us", ov0, alarm0 ? "ON" : "off", ov1, alarm1 ? "on" : "OFF",
                 (unsigned)g_timing.core0.max_us);
        test_report("E1 loop overrun (cycle longer than its period) counted + alarm",
                    ov0 == 0u && !alarm0 && ov1 == 1u && alarm1, d);
    }
}

/* PC reference: real execution time of each part (not the TC375!). */
typedef struct { uint64_t min, max, sum, n; } ns_stat_t;

static void ns_add(ns_stat_t *a, uint64_t v)
{
    if (a->n == 0u || v < a->min) a->min = v;
    if (v > a->max) a->max = v;
    a->sum += v;
    a->n++;
}

static void ns_print(const char *name, const ns_stat_t *a, double period_us)
{
    double avg = a->n ? (double)a->sum / (double)a->n * 1e-3 : 0.0;
    printf("  %-34s %9.2f %9.2f %9.2f %8u %9.4f%%\n", name, (double)a->min * 1e-3, avg, (double)a->max * 1e-3,
           (unsigned)a->n, 100.0 * avg / period_us);
}

int sb_timing_report(void)
{
    pcsim_t *p = (pcsim_t *)calloc(1, sizeof *p);
    prog_wp_t w[5];
    ns_stat_t c0, mon;
    memset(&c0, 0, sizeof c0);
    memset(&mon, 0, sizeof mon);
    pcsim_init(p, 520u);
    p->heartbeat = true;
    pc_run(p, 3000u);
    prog5(&p->x, w);
    auto_go(&p->x, w, 5, PROG_END_STOP);
    p->x.mon_on = false;                        /* timed by hand below */
    uint32_t until = p->x.t + 600000u;          /* 10 min of AUTO with the PC link */
    while (p->x.t < until) {
        world_publish(&p->x.w, p->x.t);
        if (p->x.t >= p->x.next_ctrl) {
            p->x.next_ctrl += AP_PERIOD_MS;
            uint64_t t0 = sim_hal_real_ns();
            sb_step(&p->x.sb);
            link_step(&p->link, &p->x.sb);
            ns_add(&c0, sim_hal_real_ns() - t0);
            pc_hook(&p->x, p);
        }
        uint64_t t1 = sim_hal_real_ns();
        monitor_step(&p->x.mon, &g_mon, p->x.t);
        ns_add(&mon, sim_hal_real_ns() - t1);
        world_step(&p->x.w, PHYSICS_DT_MS * 0.001f);
        p->x.t += PHYSICS_DT_MS;
    }
    printf("\nPC timing reference (this PC, gcc -O2; NOT the TC375):\n");
    printf("  %-34s %9s %9s %9s %8s %10s\n", "part", "min us", "avg us", "max us", "cycles", "avg of per.");
    ns_print("core 0: sb_step + link_step", &c0, (double)TIMING_CORE0_PERIOD_US);
    ns_print("CPU1: monitor_step", &mon, (double)TIMING_MONITOR_PERIOD_US);
    printf("  (AUTO mission + ground-station link, 10 min simulated; max includes Windows interruptions)\n");
    free(p);
    return 0;
}

/* ---------------------------------------------------------------- all -- */

void sb_tests(void)
{
    timing_tests();
    hardening_tests();
    powerlim_tests();
    modes_tests();
    link_tests();
    joystick_tests();
    gps_denied();
    fixes();
    phase2();
    phase3();
    phase4();
    phase5();
    phase6();
}
