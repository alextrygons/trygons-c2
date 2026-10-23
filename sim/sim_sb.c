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
#include "proto.h"
#include "monitor.h"
#include "timing.h"
#include "logq.h"
#include "logcore0.h"
#include "logrec.h"
#include "logger.h"
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
    bool     log_on;        /* core 0 log producer + logger step (off unless a test needs it) */
};

static logger_t s_logger;   /* core 2 logger (sim: a separate step) */

/* "all --bridge": every test drives the boat through the USB joystick
 * bridge (simulated Teensy frames) instead of the analog joystick. */
static bool s_bridge;
void sb_set_bridge(bool on) { s_bridge = on; }

/* Map used for the old tests in bridge mode: throttle on stick Y
 * (spring-centred, like the analog joystick), default buttons. */
static void bridge_test_map(joymap_t *m)
{
    joymap_default(m);
    m->source = JOY_SRC_BRIDGE;
    m->thr_input = JOY_THR_STICK_Y;
    m->axis_thr = JSF_AXIS_CODE(0u, 1u);
    m->inv_thr = 0u;
    joymap_seal(m);
}

/* Joystick position (-1..+1, default calibration) -> raw ADC. */
static uint16_t joy_raw(float v)
{
    joy_cal_t c;
    joy_default_cal(&c);
    float r = (v >= 0.0f) ? c.center[0] + v * (float)(c.max[0] - c.center[0])
                          : c.center[0] + v * (float)(c.center[0] - c.min[0]);
    return (uint16_t)lrintf(r);
}

/* USB joystick state for the USB tests: the tests drive the simulated
 * bridge directly (T.16000M stick X/Y, TWCS lever, buttons, hat, plugged). */
typedef struct {
    bool     on;
    float    x, y, lever;       /* lever 0 (back) .. 1 (forward) */
    uint32_t b0, b1;            /* buttons: stick, throttle */
    uint8_t  hat1;              /* TWCS hat 0: 0..7, 15 = centred */
    uint8_t  flags;             /* JSF_F_DEV0 | JSF_F_DEV1 | ... */
} usb_js_t;
static usb_js_t s_usb;

static int16_t usb_ax(float v)
{
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (int16_t)lrintf(v * 32767.0f);
}

static void usb_send(void)
{
    jsf_t f;
    memset(&f, 0, sizeof f);
    jsf_dev_clear(&f.dev[0]);
    jsf_dev_clear(&f.dev[1]);
    f.flags = s_usb.flags;
    if (s_usb.flags & JSF_F_DEV0) {
        f.dev[0].axis[0] = usb_ax(s_usb.x);
        f.dev[0].axis[1] = usb_ax(s_usb.y);
        f.dev[0].buttons = s_usb.b0;
        f.dev[0].vid = 0x044Fu;
        f.dev[0].pid = 0xB10Au;
    }
    if (s_usb.flags & JSF_F_DEV1) {
        f.dev[1].axis[2] = usb_ax(1.0f - 2.0f * s_usb.lever);     /* TWCS: forward = low (map inverts) */
        f.dev[1].buttons = s_usb.b1;
        f.dev[1].hats = (uint8_t)(0xF0u | (s_usb.hat1 & 0x0Fu));
        f.dev[1].vid = 0x044Fu;
        f.dev[1].pid = 0xB687u;
    }
    sim_hal_bridge_set(&f);
}

static void joy_set(float x, float y, bool take, bool autob)
{
    if (s_usb.on) {             /* USB tests: same stick / buttons through the bridge */
        s_usb.x = x;
        s_usb.y = y;
        s_usb.b0 = take ? (s_usb.b0 | 1u) : (s_usb.b0 & ~1u);
        s_usb.b1 = autob ? (s_usb.b1 | 1u) : (s_usb.b1 & ~1u);
        usb_send();
        return;
    }
    sim_hal_set_joystick(true, joy_raw(x), joy_raw(y), take, autob);
}

static void sbsim_init(sbsim_t *x, uint32_t seed, float heading, float speed_kn,
                       const sb_config_t *cfg)
{
    sb_config_t c;
    memset(x, 0, sizeof *x);
    memset(&s_usb, 0, sizeof s_usb);
    world_init(&x->w, seed, heading, speed_kn);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, 0.05f, 0.02f);
    sim_hal_set_accel(true);
    sim_hal_reset_engine();
    joy_set(0.0f, 0.0f, false, false);              /* joystick fitted, centred */
    sim_hal_joystick_confirm(false);
    remove("nv1.bin");                              /* default joystick calibration */
    remove("nv2.bin");                              /* no stored AUTO program */
    remove("nv4.bin");                              /* default joystick map (analog) */
    sim_hal_unforce();
    memset((void *)&g_mon, 0, sizeof g_mon);
    world_publish(&x->w, 0);
    hal_init();
    sim_hal_bridge_mode(s_bridge);
    if (s_bridge) {
        joymap_t m;
        bridge_test_map(&m);
        (void)hal_nv_save(HAL_NV_SLOT_JOYMAP, &m, (uint32_t)sizeof m);
    }
    if (cfg) c = *cfg; else sb_default_config(&c);
    x->cfg = c;
    timing_init_all();
    logq_init_all();
    memset((void *)&g_logstat, 0, sizeof g_logstat);
    sb_init(&x->sb, &c);
    memset((void *)&g_mon, 0, sizeof g_mon);         /* CPU1 starts: "running" only from core 0's loop */
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
                if (x->log_on) logcore0_step(&x->sb);
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
        if (x->log_on && x->t % 10u == 0u) {        /* the logger (core 2): its own step */
            uint32_t t0 = hal_micros();
            logger_step(&s_logger, x->t);
            (void)cyc_record(&g_timing.logger, hal_micros() - t0);
        }
        if (x->mon_on && !x->boot_until) {
            uint32_t t0 = hal_micros();
            monitor_step(&x->mon, &g_mon, x->t);
            (void)cyc_record(&g_timing.monitor, hal_micros() - t0);
        }
        world_step(&x->w, PHYSICS_DT_MS * 0.001f);
        x->t += PHYSICS_DT_MS;
    }
}

/* Logging on: SD card in folder `dir` (created / emptied), boot number. */
static void sbsim_log_start(sbsim_t *x, const char *dir, uint32_t boot)
{
    char cmd[300];
    snprintf(cmd, sizeof cmd, "mkdir %s 2>nul & del /q %s\\*.TLG 2>nul", dir, dir);
    (void)system(cmd);
    sim_hal_sd_setup(dir, true, 4000000u, false);
    g_logstat.boot = boot;
    logger_init(&s_logger, x->t);
    logcore0_init(&x->sb, boot);
    x->log_on = true;
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
    if (s_usb.on) { s_usb.b1 |= 8u; usb_send(); }          /* USB: TWCS button 4 */
    else sim_hal_joystick_confirm(true);
    sbsim_run(x, x->t + 100u, NULL, NULL);
    if (s_usb.on) { s_usb.b1 &= ~8u; usb_send(); }
    else sim_hal_joystick_confirm(false);
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
        sbsim_run(&x, 2000u, NULL, NULL);
        power_full(&x);                             /* really full throttle (R1: STANDBY closing rate) */
        hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 100.0f);
        sbsim_run(&x, 60000u, NULL, NULL);
        float v0 = x.w.speed_kn, h0 = x.w.heading_deg, hmax = 0.0f, thr0 = sim_hal_throttle();
        ev_reset(&ev, x.t, thr0, x.sb.cfg.gear.pwm_n_us);
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
        snprintf(d, sizeof d, "at %.1f kn, throttle %.0f %%: throttle down after %.2f s, idle %.2f s (STANDBY rate "
                 "%.0f %%/s), neutral %.2f s, steering off %.2f s (held heading within %.1f deg); under-load shifts %d; "
                 "log: %s->%s (%s)",
                 v0, thr0, ev.t_thr_down, ev.t_idle, x.sb.thr.cfg.rate_stop_pct_s, ev.t_neutral, ev.t_steer_off, hmax,
                 x.w.shift_under_load, mode_name(le->from), mode_name(le->to), mode_reason_name(le->why));
        test_report("P2 STANDBY at full throttle: idle within about 1 s, neutral, then steering off",
                    order && thr0 > 90.0f && ev.t_idle <= thr0 / x.sb.thr.cfg.rate_stop_pct_s + 0.1f &&
                    x.w.shift_under_load == 0 && ev.steer_after_off == 0 &&
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

    /* 3.3b R1: STANDBY in MANUAL in a turn - the onboard stick keeps steering
     *      until idle + neutral (here it swings to the other side during the
     *      stop, still held), then steering off; the held stick never takes
     *      control back */
    {
        static const ctl_step_t seq[] = {
            {  0.0f,  0.5f, 1.0f, SEL_F },
            { 25.4f, -0.5f, 1.0f, SEL_F },    /* 0.4 s after STANDBY: stick to the left, still held */
        };
        sbsim_t x;
        script_t s;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 233u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        script_start(&s, &x, seq, 2);
        sbsim_run(&x, s.t0 + 25000u, script_hook, &s);
        bool was_manual = sb_mode(&x.sb) == MODE_MANUAL;
        float thr0 = sim_hal_throttle(), tgt_press = x.sb.ap.rudder_target;
        ev_reset(&ev, x.t, thr0, x.sb.cfg.gear.pwm_n_us);
        sim_hal_press(HAL_BTN_STANDBY);
        float tgt_min = tgt_press;
        int motor_left = 0;
        bool steer_on_at_neutral = false, manual_again = false;
        while (x.t < s.t0 + 40000u) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, script_hook, &s);
            ev_watch(&x, &ev);
            if (x.sb.ap.mode == AP_MODE_ANGLE) {
                if (x.sb.ap.rudder_target < tgt_min) tgt_min = x.sb.ap.rudder_target;
                if (sim_hal_motor_command() < 0.0f) motor_left++;
            }
            if (ev.t_neutral >= 0.0f && ev.t_steer_off < 0.0f) steer_on_at_neutral = true;
            if (sb_mode(&x.sb) == MODE_MANUAL) manual_again = true;
        }
        snprintf(d, sizeof d, "throttle %.0f %% -> idle after %.2f s (STANDBY rate %.0f %%/s), neutral %.2f s, "
                 "steering off %.2f s; outboard command at STANDBY %+.1f deg, followed the stick to %+.1f deg "
                 "during the stop (motor %d steps left); motor after off %d, under-load %d, held stick took control %s",
                 thr0, ev.t_idle, x.sb.thr.cfg.rate_stop_pct_s, ev.t_neutral, ev.t_steer_off, tgt_press, tgt_min,
                 motor_left, ev.steer_after_off, x.w.shift_under_load, manual_again ? "YES" : "no");
        test_report("P3 R1 STANDBY in MANUAL: the stick steers until idle + neutral, then steering off",
                    was_manual && thr0 > 20.0f && ev.t_idle >= 0.0f && ev.t_idle <= thr0 / 100.0f + 0.1f &&
                    ev.t_neutral > ev.t_idle && ev.t_steer_off >= ev.t_neutral && steer_on_at_neutral &&
                    tgt_press > 5.0f && tgt_min < tgt_press - 3.0f && motor_left > 0 && ev.steer_after_off == 0 &&
                    x.w.shift_under_load == 0 && !manual_again && sb_mode(&x.sb) == MODE_STANDBY, d);
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
            /* USB bridge: the broken axis arrives as an invalid value */
            uint8_t why = s_bridge ? JOY_WHY_RANGE : (uint8_t)(a + 1);
            ok_axis[a] = x.sb.joy.fault && x.sb.joy.fault_axis == why &&
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
        /* the stick that steered is gone: gyro hold until idle + neutral, then steering off */
        test_report("Joystick wire break while in control: idle + neutral, gyro hold, then steering off, STANDBY",
                    sb_mode(&x.sb) == MODE_STANDBY && le->why == WHY_JOY_FAULT && ev.t_thr_down >= 0.0f &&
                    ev.t_thr_down <= 0.1f && ev.t_neutral > 0.0f && ev.t_steer_off >= ev.t_neutral &&
                    ev.steer_after_off == 0 && sim_hal_alarm() && x.w.shift_under_load == 0, d);
    }

    /* calibration routine: stored with checksum */
    if (s_bridge) test_na("Joystick calibration stored with checksum", "analog joystick only (the USB bridge calibrates itself)");
    else {
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
    uint8_t     tm[PROTO_MAX_PAYLOAD];  /* last telemetry payload */
    uint8_t     tm_len;
    int         joymaps, raws;
    uint8_t     jm[PROTO_MAX_PAYLOAD], jm_len;      /* last JOYMAP_VALUE */
    uint8_t     raw[PROTO_MAX_PAYLOAD], raw_len;    /* last JOY_RAW */
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
            memcpy(p->tm, f.payload, f.len);
            p->tm_len = f.len;
            r.n = 4u + 4u + 20u + 2u + 16u + 2u + 24u + 12u;      /* alarm_bits offset */
            p->telem_alarms = proto_rd_u32(&r);
        } else if (f.type == PROTO_T_JOYMAP_VALUE) {
            memcpy(p->jm, f.payload, f.len);
            p->jm_len = f.len;
            p->joymaps++;
        } else if (f.type == PROTO_T_JOY_RAW) {
            memcpy(p->raw, f.payload, f.len);
            p->raw_len = f.len;
            p->raws++;
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

    /* C3b R4: slow boot - before core 0 says "running" only the boot timeout
     *     counts (no heartbeat trip during a slow start); afterwards the
     *     heartbeat; core 0 that never starts trips after MON_BOOT_MS */
    {
        static mon_shared_t sh;
        mon_t m;
        memset((void *)&sh, 0, sizeof sh);
        sh.thr_idle_us = 1000u;
        sh.gear_n_us = 1500u;
        sh.fwd_limit = 30.0f;
        sh.rev_limit = 5.0f;
        monitor_init(&m, 0u);
        bool trip_boot = false;
        uint32_t t = 0u;
        for (; t <= 2500u; t++) trip_boot |= monitor_step(&m, &sh, t);        /* slow start, no heartbeat */
        sh.running = MON_RUNNING;
        bool trip_run = false;
        for (; t <= 5000u; t++) {
            if (t % 50u == 0u) sh.heartbeat++;
            trip_run |= monitor_step(&m, &sh, t);
        }
        uint32_t t_stop = 5000u, t_trip = 0u;                    /* the last heartbeat */
        for (; t <= 6000u && !m.tripped; t++) monitor_step(&m, &sh, t);     /* heartbeat stops */
        if (m.tripped) t_trip = m.trip_ms - t_stop;
        mon_reason_t r_hb = m.reason;
        /* core 0 never starts */
        mon_t n;
        memset((void *)&sh, 0, sizeof sh);
        monitor_init(&n, 0u);
        for (t = 0u; t <= 4000u && !n.tripped; t++) monitor_step(&n, &sh, t);
        /* the whole simulator: core 0 starts its loop 2.5 s late */
        sbsim_t x;
        sbsim_init(&x, 404u, 90.0f, 0.0f, NULL);
        x.wdt_ms = 0u;
        x.frozen = true;
        sbsim_run(&x, 2500u, NULL, NULL);
        bool sim_quiet = !x.mon.tripped;
        x.frozen = false;
        sbsim_run(&x, 6000u, NULL, NULL);
        bool sim_ok = !x.mon.tripped && x.mon.started;
        snprintf(d, sizeof d, "2.5 s without \"running\": trip %s; running + heartbeat 2.5 s: trip %s; heartbeat stops: "
                 "%s after %lu ms; core 0 never starts: %s after %lu ms (limit %u); simulator with core 0 2.5 s "
                 "late: no trip %s, watched afterwards %s",
                 trip_boot ? "YES" : "no", trip_run ? "YES" : "no", monitor_reason_name(r_hb), (unsigned long)t_trip,
                 monitor_reason_name(n.reason), (unsigned long)n.trip_ms, MON_BOOT_MS, sim_quiet ? "yes" : "NO",
                 sim_ok ? "yes" : "NO");
        test_report("C3b slow boot: no monitor trip before core 0 runs; never starting trips after 3 s",
                    !trip_boot && !trip_run && r_hb == MON_TRIP_HEARTBEAT && t_trip > MON_HB_MS &&
                    t_trip <= MON_HB_MS + 2u && n.tripped && n.reason == MON_TRIP_BOOT &&
                    n.trip_ms > MON_BOOT_MS && n.trip_ms <= MON_BOOT_MS + 2u && sim_quiet && sim_ok, d);
    }

    /* C3c R2: the big boat (rudder only) feeds the CPU1 monitor too: no trip
     *     while its loop runs, a frozen loop trips it and the rudder motor stops */
    {
        static autopilot_t ap;
        static mon_shared_t sh;
        mon_t m;
        ap_config_t c;
        hal_init();
        autopilot_default_config(&c);
        autopilot_init(&ap, &c);
        memset((void *)&sh, 0, sizeof sh);
        monitor_init(&m, 0u);
        bool trip_run = false;
        uint32_t t = 0u, last_step = 0u;
        for (; t <= 30000u; t++) {
            sim_hal_set_time_ms(t);
            if (t % AP_PERIOD_MS == 0u) {
                autopilot_step(&ap);
                monitor_publish_steering_only(&sh, 1000u, 1500u);
                last_step = t;
            }
            trip_run |= monitor_step(&m, &sh, t);
        }
        hal_set_rudder_motor(1.0f);                  /* the loop freezes with the motor running */
        for (; t <= 32000u && !m.tripped; t++) {
            sim_hal_set_time_ms(t);
            monitor_step(&m, &sh, t);
        }
        hal_set_rudder_motor(1.0f);                  /* (a late write can't start it again) */
        float motor = sim_hal_motor_command();
        uint32_t react = m.tripped ? m.trip_ms - last_step : 0u;
        snprintf(d, sizeof d, "big boat 30 s: monitor trip %s; loop frozen: %s after %lu ms, rudder motor %.0f, "
                 "outputs latched %s", trip_run ? "YES" : "no", monitor_reason_name(m.reason), (unsigned long)react,
                 motor, hal_outputs_forced() ? "yes" : "NO");
        test_report("C3c big boat: CPU1 monitor fed (no trip), frozen loop trips it, rudder motor stops",
                    !trip_run && m.tripped && m.reason == MON_TRIP_HEARTBEAT && react > MON_HB_MS &&
                    react <= MON_HB_MS + 2u && motor == 0.0f && hal_outputs_forced(), d);
        hal_init();                                 /* reset the latch for the next tests */
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

/* ------------------------------------------------------- section F: logging -- */

/* A minimal .TLG reader for the tests (the PC tools are in groundstation/). */
typedef struct {
    int      ok;                /* file opened, magic right */
    uint32_t records, data, events, headers, configs, crc_bad, tail_bytes;
    uint32_t t_first_data, t_last_data, t_last;
    char     fields[2048];      /* "name:type:scale,..." from the header */
} tlg_info_t;

typedef void (*tlg_cb)(void *ctx, uint8_t type, uint32_t t, const uint8_t *p, uint16_t len);

static void tlg_read(const char *path, tlg_info_t *in, tlg_cb cb, void *ctx)
{
    static uint8_t buf[32u * 1024u * 1024u];
    memset(in, 0, sizeof *in);
    FILE *f = fopen(path, "rb");
    if (!f) return;
    size_t n = fread(buf, 1, sizeof buf, f);
    fclose(f);
    if (n < 4 || memcmp(buf, "TLG1", 4) != 0) return;
    in->ok = 1;
    size_t i = 4;
    while (i + LOGR_OVERHEAD <= n) {
        if (buf[i] != LOGR_SYNC) { in->crc_bad++; i++; continue; }
        uint16_t len = (uint16_t)(buf[i + 2] | (buf[i + 3] << 8));
        if (i + LOGR_OVERHEAD + len > n) break;                 /* cut off at the end */
        uint16_t crc = proto_crc16(&buf[i + 1], (size_t)len + 7u);
        if ((uint8_t)crc != buf[i + LOGR_HDR + len] || (uint8_t)(crc >> 8) != buf[i + LOGR_HDR + len + 1]) {
            in->crc_bad++;
            i++;
            continue;
        }
        uint8_t type = buf[i + 1];
        uint32_t t = (uint32_t)buf[i + 4] | ((uint32_t)buf[i + 5] << 8) | ((uint32_t)buf[i + 6] << 16) |
                     ((uint32_t)buf[i + 7] << 24);
        in->records++;
        if (type == LOGR_DATA) {
            if (!in->data) in->t_first_data = t;
            in->data++;
            in->t_last_data = t;
        }
        if (type == LOGR_EVENT) in->events++;
        if (type == LOGR_CONFIG) in->configs++;
        if (type == LOGR_HEADER) {
            in->headers++;
            const char *h = (const char *)&buf[i + LOGR_HDR];
            const char *fs = strstr(h, "fields=");
            if (fs && fs < h + len) {
                const char *e = memchr(fs, '\n', (size_t)(h + len - fs));
                size_t k = (size_t)((e ? e : h + len) - fs - 7);
                if (k >= sizeof in->fields) k = sizeof in->fields - 1;
                memcpy(in->fields, fs + 7, k);
                in->fields[k] = 0;
            }
        }
        in->t_last = t;
        if (cb) cb(ctx, type, t, &buf[i + LOGR_HDR], len);
        i += LOGR_OVERHEAD + len;
    }
    in->tail_bytes = (uint32_t)(n - i);
}

/* Value of field `name` in a DATA payload, decoded with the header's list. */
static bool tlg_field(const char *fields, const uint8_t *p, const char *name, double *out)
{
    char nm[48], ty;
    double sc;
    const char *s = fields;
    size_t off = 0;
    while (*s) {
        int used = 0;
        if (sscanf(s, "%47[^:]:%c:%lf%n", nm, &ty, &sc, &used) != 3) return false;
        size_t sz = (ty == 'B' || ty == 'c') ? 1u : (ty == 'H' || ty == 'h') ? 2u : 4u;
        if (strcmp(nm, name) == 0) {
            const uint8_t *q = p + off;
            uint32_t u = q[0] | (sz > 1 ? (uint32_t)q[1] << 8 : 0u) | (sz > 2 ? (uint32_t)q[2] << 16 | (uint32_t)q[3] << 24 : 0u);
            double v;
            switch (ty) {
            case 'B': v = (double)(uint8_t)u; break;
            case 'c': v = (double)(int8_t)u; break;
            case 'H': v = (double)(uint16_t)u; break;
            case 'h': v = (double)(int16_t)u; break;
            case 'I': v = (double)u; break;
            case 'i': v = (double)(int32_t)u; break;
            default: { float fl; memcpy(&fl, &u, 4); *out = fl; return true; }
            }
            *out = v / sc;
            return true;
        }
        off += sz;
        s += used;
        if (*s == ',') s++;
    }
    return false;
}

/* All files in a folder that start with `prefix`, sorted by name. */
static int tlg_list(const char *dir, const char *prefix, char names[][128], int max)
{
    char cmd[300];
    snprintf(cmd, sizeof cmd, "dir /b /on %s\\%s*.TLG 2>nul", dir, prefix);
    FILE *p = popen(cmd, "r");
    int n = 0;
    if (!p) return 0;
    while (n < max && fgets(names[n], 128, p)) {
        names[n][strcspn(names[n], "\r\n")] = 0;
        if (names[n][0]) n++;
    }
    pclose(p);
    return n;
}

/* Round trip: values captured from the sim at given steps vs decoded. */
typedef struct {
    uint32_t t[8];
    double   heading[8], thr[8], est_lat[8], unc[8], joy_x[8];
    int      n;
    int      checked, worst_field;
    double   worst;             /* worst |difference| in units of the field's resolution */
} rt_t;

static void rt_cb(void *ctx, uint8_t type, uint32_t t, const uint8_t *p, uint16_t len)
{
    (void)len;
    struct { rt_t *r; const char *fields; } *c = ctx;
    if (type != LOGR_DATA) return;
    for (int i = 0; i < c->r->n; i++) {
        if (c->r->t[i] != t) continue;
        double v[5];
        const char *nm[5] = { "heading", "thr_out", "est_lat", "unc", "joy_x" };
        const double ref[5] = { c->r->heading[i], c->r->thr[i], c->r->est_lat[i], c->r->unc[i], c->r->joy_x[i] };
        const double res[5] = { 0.01, 0.01, 1e-7, 0.1, 0.001 };
        for (int k = 0; k < 5; k++) {
            if (!tlg_field(c->fields, p, nm[k], &v[k])) { c->r->worst = 1e9; continue; }
            double e = fabs(v[k] - ref[k]) / res[k];
            if (e > c->r->worst) { c->r->worst = e; c->r->worst_field = k; }
        }
        c->r->checked++;
    }
}

static void logging_tests(void)
{
    char d[600];
    static char names[64][128];
    static uint8_t drain[LOGR_MAX + 64];
    char path[512];
    printf("\n=== Section F: logging ===\n");

    /* F1 2 hours at full rate: nothing dropped, timing unchanged */
    {
        sbsim_t x;
        sbsim_init(&x, 601u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        sbsim_log_start(&x, "sd_f1", 1u);
        hdg_lock(&x);
        joy_set(0.0f, 0.5f, false, false);
        x.w.sea_state = 2.0f;
        uint32_t steps0 = x.sb.modes.log_count;
        (void)steps0;
        uint32_t t_start = x.t;
        sbsim_run(&x, x.t + 2u * 3600u * 1000u, NULL, NULL);
        uint32_t t_end = x.t;
        logger_close(&s_logger);
        x.log_on = false;
        int nf = tlg_list("sd_f1", "B", names, 64);
        uint32_t data = 0u, bad = 0u, events = 0u;
        bool all_headers = true;
        for (int i = 0; i < nf; i++) {
            tlg_info_t in;
            snprintf(path, sizeof path, "sd_f1/%.120s", names[i]);
            tlg_read(path, &in, NULL, NULL);
            data += in.data;
            bad += in.crc_bad;
            events += in.events;
            if (!in.ok || in.headers != 1u || in.configs < 1u) all_headers = false;
        }
        uint32_t expect = (t_end - t_start) / AP_PERIOD_MS;
        /* control step time on this PC with and without the log producer */
        uint64_t with = 0, without = 0, mx_with = 0;
        sbsim_t y;
        sbsim_init(&y, 602u, 90.0f, 0.0f, NULL);
        sbsim_run(&y, 3000u, NULL, NULL);
        hdg_lock(&y);
        logq_init_all();
        for (int k = 0; k < 4000; k++) {
            world_publish(&y.w, y.t);
            uint64_t a = sim_hal_real_ns();
            sb_step(&y.sb);
            uint64_t b = sim_hal_real_ns();
            logcore0_step(&y.sb);
            uint64_t c = sim_hal_real_ns();
            without += b - a;
            with += c - a;
            if (c - a > mx_with) mx_with = c - a;
            while (logq_pop(&g_logq0, drain, sizeof drain)) {}     /* keep the queue moving */
            for (int q = 0; q < 5; q++) { world_step(&y.w, PHYSICS_DT_MS * 0.001f); y.t += PHYSICS_DT_MS; }
        }
        snprintf(d, sizeof d, "2 h at 20 Hz: %u files (new every 10 min), %u data records written (2 h = %u, + the setup), "
                 "%u events, records dropped %u + %u, CRC errors %u, every file self-contained %s; "
                 "step time on this PC: %.2f us without, %.2f us with the log producer (max %.1f us)",
                 nf, data, expect, events, (unsigned)g_logq0.dropped, (unsigned)g_logq1.dropped, bad,
                 all_headers ? "yes" : "NO", without / 4000.0 / 1000.0, with / 4000.0 / 1000.0, mx_with / 1000.0);
        test_report("F1 2 h of full-rate logging: no dropped records, control timing unchanged",
                    nf >= 12 && nf <= 13 && data >= expect && data <= expect + 60u && bad == 0u &&
                    all_headers && g_logq0.dropped == 0u && g_logq1.dropped == 0u &&
                    with / 4000.0 / 1000.0 < 100.0, d);
    }

    /* F2 power cut while writing */
    {
        sbsim_t x;
        sbsim_init(&x, 603u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        sbsim_log_start(&x, "sd_f2", 2u);
        hdg_lock(&x);
        joy_set(0.0f, 0.4f, false, false);
        sbsim_run(&x, x.t + 25u * 60u * 1000u + 1300u, NULL, NULL);     /* 2 files closed, 3rd being written */
        uint32_t t_cut = x.t;
        sim_hal_sd_power_cut();                     /* power gone: unsynced data lost */
        x.log_on = false;
        int nf = tlg_list("sd_f2", "B", names, 64);
        bool older_ok = nf >= 3;
        uint32_t bad = 0u;
        tlg_info_t last;
        memset(&last, 0, sizeof last);
        for (int i = 0; i < nf; i++) {
            tlg_info_t in;
            snprintf(path, sizeof path, "sd_f2/%.120s", names[i]);
            tlg_read(path, &in, NULL, NULL);
            bad += in.crc_bad;
            if (i < nf - 1 && (!in.ok || in.tail_bytes != 0u || in.data < 11900u)) older_ok = false;
            if (i == nf - 1) last = in;
        }
        float lost_s = (float)(t_cut - last.t_last_data) * 0.001f;
        snprintf(d, sizeof d, "power cut at %.1f s: %d files; older files complete %s; latest file readable %s, "
                 "%u data records, last one %.1f s before the cut (sync every 2 s), CRC errors %u",
                 t_cut * 0.001f, nf, older_ok ? "yes" : "NO", last.ok ? "yes" : "NO", last.data, lost_s, bad);
        test_report("F2 power cut while writing: older files intact, latest readable up to the last sync",
                    older_ok && last.ok && last.data > 0u && lost_s <= 2.1f && bad == 0u, d);
    }

    /* F3 card missing / full / write errors: alarm, control keeps working */
    {
        int ok_n = 0;
        int len = snprintf(d, sizeof d, "LOG FAILURE alarm and AUTO still running with:");
        for (int c = 0; c < 3; c++) {
            sbsim_t x;
            prog_wp_t w[5];
            sbsim_init(&x, 610u + (uint32_t)c, 90.0f, 0.0f, NULL);
            sbsim_run(&x, 2000u, NULL, NULL);
            sbsim_log_start(&x, "sd_f3", 3u);
            if (c == 0) sim_hal_sd_setup("sd_f3", false, 4000000u, false);     /* no card */
            if (c == 1) sim_hal_sd_setup("sd_f3", true, 300u, false);          /* 300 KB left */
            sbsim_run(&x, x.t + 500u, NULL, NULL);
            bool alarm_start = (x.sb.alarm_bits & SB_ALM_LOG) != 0u;
            x.sb.alarm_bits &= ~SB_ALM_LOG;         /* acknowledged (a raise needs no active alarm) */
            x.sb.alarm = false;
            prog5(&x, w);
            auto_go(&x, w, 5, PROG_END_STOP);
            if (c == 2) {
                sbsim_run(&x, x.t + 30000u, NULL, NULL);
                sim_hal_sd_setup("sd_f3", true, 4000000u, true);              /* every write fails */
            }
            sbsim_run(&x, x.t + 120000u, NULL, NULL);
            bool alarm = (c == 0) ? alarm_start && g_logstat.failure : (x.sb.alarm_bits & SB_ALM_LOG) && g_logstat.failure;
            bool running = auto_running(&x) && world_sog_kn(&x.w) > 5.0f;
            /* the card comes back: logging goes on (retry), the alarm cause is gone */
            sim_hal_sd_setup("sd_f3", true, 4000000u, false);
            uint32_t files0 = g_logstat.files;
            sbsim_run(&x, x.t + 15000u, NULL, NULL);
            bool back = !g_logstat.failure && g_logstat.files > files0;
            logger_close(&s_logger);
            x.log_on = false;
            bool good = alarm && running && back;
            if (good) ok_n++;
            len += snprintf(d + len, sizeof d - (size_t)len, " %s: alarm %s, AUTO %s, card back -> logging again %s;",
                            c == 0 ? "no card" : c == 1 ? "card full" : "write errors", alarm ? "yes" : "NO",
                            running ? "running" : "STOPPED", back ? "yes" : "NO");
        }
        test_report("F3 card missing / full / write errors: LOG FAILURE alarm, control keeps working", ok_n == 3, d);
    }

    /* F4 black box on an injected fault (+ F5 round trip on the same run) */
    {
        sbsim_t x;
        prog_wp_t w[5];
        rt_t rt;
        memset(&rt, 0, sizeof rt);
        sbsim_init(&x, 620u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        sbsim_log_start(&x, "sd_f4", 4u);
        prog5(&x, w);
        auto_go(&x, w, 5, PROG_END_STOP);
        sbsim_run(&x, x.t + 240000u, NULL, NULL);
        /* capture a few samples exactly as the sim has them (for the round trip) */
        for (int i = 0; i < 8; i++) {
            sbsim_run(&x, x.t + 7000u, NULL, NULL);
            double lat, lon;
            dr_latlon(&x.sb.dr, &lat, &lon);
            rt.t[i] = x.sb.now_ms;
            rt.heading[i] = x.sb.ap.heading;
            rt.thr[i] = x.sb.servo.out_pct;
            rt.est_lat[i] = lat;
            rt.unc[i] = x.sb.dr.unc_m;
            rt.joy_x[i] = x.sb.joy.x;
            rt.n++;
        }
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        uint32_t t_fault = x.t;
        sim_hal_fail_heading(true);                 /* compass dies in AUTO */
        while (!(x.sb.alarm_bits & SB_ALM_COMPASS) && x.t < t_fault + 10000u) sbsim_run(&x, x.t + 10u, NULL, NULL);
        uint32_t t_det = x.t;                       /* detected after the 2 s compass timeout */
        sbsim_run(&x, x.t + 45000u, NULL, NULL);
        logger_close(&s_logger);
        x.log_on = false;
        int nb = tlg_list("sd_f4", "BB", names, 64);
        tlg_info_t bb;
        memset(&bb, 0, sizeof bb);
        if (nb > 0) {
            snprintf(path, sizeof path, "sd_f4/%.120s", names[0]);
            tlg_read(path, &bb, NULL, NULL);
        }
        float before = (float)(t_det - bb.t_first_data) * 0.001f;
        float after = (float)(bb.t_last_data - t_det) * 0.001f;
        snprintf(d, sizeof d, "compass fails at %.0f s, detected %.1f s later: %d black-box file(s), first one from "
                 "%.1f s before to %.1f s after the detection, %u data records, %u events, header + config %s, CRC errors %u",
                 t_fault * 0.001f, (t_det - t_fault) * 0.001f, nb, before, after, bb.data, bb.events,
                 (bb.headers == 1u && bb.configs >= 1u) ? "yes" : "NO", bb.crc_bad);
        test_report("F4 black box: fault -> separate file with the 60 s before (and 30 s after)",
                    nb >= 1 && before >= 59.0f && before <= 61.0f && after >= 29.0f && bb.crc_bad == 0u &&
                    bb.headers == 1u && bb.data >= 1750u, d);

        /* F5 round trip: the main log decoded with its own header vs the sim's values */
        int nf = tlg_list("sd_f4", "B0", names, 64);
        struct { rt_t *r; const char *fields; } ctx;
        for (int i = 0; i < nf; i++) {
            tlg_info_t in;
            snprintf(path, sizeof path, "sd_f4/%.120s", names[i]);
            tlg_read(path, &in, NULL, NULL);            /* first pass: the field list */
            static char fields[2048];
            memcpy(fields, in.fields, sizeof fields);
            ctx.r = &rt;
            ctx.fields = fields;
            tlg_read(path, &in, rt_cb, &ctx);
        }
        static const char *fn[5] = { "heading", "thr_out", "est_lat", "unc", "joy_x" };
        snprintf(d, sizeof d, "%d of %d captured samples found and decoded; worst difference %.2f x the field's "
                 "resolution (%s); i.e. exact to the stored resolution (heading 0.01 deg, throttle 0.01 %%, "
                 "lat 1e-7 deg, uncertainty 0.1 m, stick 0.001)",
                 rt.checked, rt.n, rt.worst, fn[rt.worst_field]);
        test_report("F5 round trip: log -> decoded values match the sim (to the stored resolution)",
                    rt.checked == rt.n && rt.worst <= 0.5001, d);
    }
}

/* Python round-trip test: a 5 min AUTO run logged into `dir`, plus
 * ref.csv with the sim's own (unrounded) values once a second. */
int sb_logtest(const char *dir)
{
    static char path[600];
    sbsim_t x;
    prog_wp_t w[5];
    sbsim_init(&x, 640u, 90.0f, 0.0f, NULL);
    sbsim_run(&x, 2000u, NULL, NULL);
    sbsim_log_start(&x, dir, 7u);
    prog5(&x, w);
    auto_go(&x, w, 5, PROG_END_STOP);
    snprintf(path, sizeof path, "%s/ref.csv", dir);
    FILE *f = fopen(path, "w");
    if (!f) return 1;
    fprintf(f, "t_ms,mode,heading,thr_out,est_lat,est_lon,unc,speed_target,gps_sog,alarm_bits\n");
    uint32_t t_end = x.t + 300000u;
    while (x.t < t_end) {
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        double lat, lon;
        dr_latlon(&x.sb.dr, &lat, &lon);
        fprintf(f, "%lu,%d,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%.10g,%lu\n", (unsigned long)x.sb.now_ms,
                (int)sb_mode(&x.sb), x.sb.ap.heading, x.sb.servo.out_pct, lat, lon, x.sb.dr.unc_m,
                x.sb.cruise.target_kn, x.sb.gps.last.sog_kn, (unsigned long)x.sb.alarm_bits);
    }
    fclose(f);
    logger_close(&s_logger);
    x.log_on = false;
    printf("log + ref.csv written to %s\n", dir);
    return 0;
}

/* ---------------------------------------------- USB joystick (bridge) -- */

#define USB_TAKE     JSF_CODE_BTN(0u, 0u)
#define USB_STANDBY  JSF_CODE_BTN(0u, 1u)
#define USB_AUTO     JSF_CODE_BTN(1u, 0u)
#define USB_HDG      JSF_CODE_BTN(1u, 2u)
#define USB_CONFIRM  JSF_CODE_BTN(1u, 3u)

/* Boat with the T.16000M + TWCS on the bridge (default map, lever
 * throttle), stick at x0, lever at lever0 from power-up; runs 1.5 s. */
static void usb_init_at(sbsim_t *x, uint32_t seed, float speed_kn, float x0, float lever0)
{
    joymap_t m;
    jm_error_t err;
    sbsim_init(x, seed, 90.0f, speed_kn, NULL);
    sim_hal_bridge_mode(true);
    s_usb.on = true;
    s_usb.x = x0;
    s_usb.lever = lever0;
    s_usb.hat1 = 15u;
    s_usb.flags = JSF_F_DEV0 | JSF_F_DEV1 | JSF_F_HUB;
    usb_send();
    joymap_default(&m);
    m.source = JOY_SRC_BRIDGE;
    (void)sb_joymap_set(&x->sb, &m, true, &err);    /* saved: survives a simulated reset */
    sbsim_run(x, 1500u, NULL, NULL);
}

static void usb_init(sbsim_t *x, uint32_t seed, float speed_kn)
{
    usb_init_at(x, seed, speed_kn, 0.0f, 0.5f);
}

static void usb_code(uint8_t code, bool down)
{
    if (code >= 1u && code <= 32u) {
        uint32_t b = 1u << (code - 1u);
        s_usb.b0 = down ? (s_usb.b0 | b) : (s_usb.b0 & ~b);
    } else if (code >= 33u && code <= 64u) {
        uint32_t b = 1u << (code - 33u);
        s_usb.b1 = down ? (s_usb.b1 | b) : (s_usb.b1 & ~b);
    } else if (code >= 73u && code <= 76u) {
        s_usb.hat1 = down ? (uint8_t)((code - 73u) * 2u) : 15u;
    }
    usb_send();
}

/* Press a USB button / hat direction for 150 ms. */
static void usb_press(sbsim_t *x, uint8_t code)
{
    usb_code(code, true);
    sbsim_run(x, x->t + 150u, NULL, NULL);
    usb_code(code, false);
    sbsim_run(x, x->t + 100u, NULL, NULL);
}

/* Move the lever to `to` at `rate` per second (a hand on the lever). */
static void usb_lever(sbsim_t *x, float to, float rate, sb_hook_t hook, void *ctx)
{
    while (fabsf(s_usb.lever - to) > 1e-4f) {
        float step = rate * 0.01f;
        s_usb.lever = (to > s_usb.lever) ? fminf(to, s_usb.lever + step) : fmaxf(to, s_usb.lever - step);
        usb_send();
        sbsim_run(x, x->t + 10u, hook, ctx);
    }
}

static void usb_stick(sbsim_t *x, float v, uint32_t ms)
{
    s_usb.x = v;
    usb_send();
    sbsim_run(x, x->t + ms, NULL, NULL);
}

/* MANUAL by TAKE, lever forward to `lever`, run `ms`. */
static void usb_drive(sbsim_t *x, float lever, uint32_t ms)
{
    usb_press(x, USB_TAKE);
    usb_lever(x, lever, 0.5f, NULL, NULL);
    sbsim_run(x, x->t + ms, NULL, NULL);
}

typedef struct {
    float thr_max, thr_min;
    float max_rate;             /* %/s between control steps */
    float last;
    bool  have;
} thr_watch_t;

static void thr_watch(sbsim_t *x, void *ctx)
{
    thr_watch_t *w = (thr_watch_t *)ctx;
    float t = sim_hal_throttle();
    if (t > w->thr_max) w->thr_max = t;
    if (t < w->thr_min) w->thr_min = t;
    if (w->have) {
        float r = fabsf(t - w->last) / (AP_PERIOD_MS * 0.001f);
        if (r > w->max_rate) w->max_rate = r;
    }
    w->last = t;
    w->have = true;
    (void)x;
}

static void thr_watch_reset(thr_watch_t *w)
{
    w->thr_max = -1.0f;
    w->thr_min = 1000.0f;
    w->max_rate = 0.0f;
    w->have = false;
}

typedef struct {
    uint32_t t0;
    int32_t  t_fault, t_standby;
} det_t;

static void det_watch(sbsim_t *x, void *ctx)
{
    det_t *d = (det_t *)ctx;
    if (d->t_fault < 0 && x->sb.joy.fault) d->t_fault = (int32_t)(x->t - d->t0);
    if (d->t_standby < 0 && sb_mode(&x->sb) == MODE_STANDBY) d->t_standby = (int32_t)(x->t - d->t0);
}

enum { UF_LOST, UF_CORRUPT, UF_FROZEN, UF_STICK, UF_THROTTLE, UF_COUNT };
static const char *const UF_NAME[UF_COUNT] = { "frames lost", "frames corrupted", "counter frozen",
                                               "stick unplugged", "throttle unplugged" };
static const uint8_t UF_WHY[UF_COUNT] = { JOY_WHY_TIMEOUT, JOY_WHY_CRC, JOY_WHY_SEQ, JOY_WHY_DEV0, JOY_WHY_DEV1 };

static void usb_fault_inject(int k)
{
    switch (k) {
    case UF_LOST:     sim_hal_bridge_fault(SIM_BR_STOP); break;
    case UF_CORRUPT:  sim_hal_bridge_fault(SIM_BR_CORRUPT); break;
    case UF_FROZEN:   sim_hal_bridge_fault(SIM_BR_FREEZE); break;
    case UF_STICK:    s_usb.flags &= (uint8_t)~JSF_F_DEV0; usb_send(); break;
    case UF_THROTTLE: s_usb.flags &= (uint8_t)~JSF_F_DEV1; usb_send(); break;
    default: break;
    }
}

/* Latency: TAKE pressed on the joystick -> MANUAL on the AURIX. */
typedef struct {
    uint32_t t_press;
    int32_t  lat;
} lat_t;

static void lat_watch(sbsim_t *x, void *ctx)
{
    lat_t *l = (lat_t *)ctx;
    if (l->lat < 0 && x->t >= l->t_press && sb_mode(&x->sb) == MODE_MANUAL) l->lat = (int32_t)(x->t - l->t_press);
}

static void usb_joystick_tests(void)
{
    char d[900];
    printf("\n=== USB joystick (T.16000M + TWCS through the Teensy bridge) ===\n");

    /* U1 frame format: CRC, junk, rescan, codes */
    {
        jsf_t f, g;
        uint8_t b[3 * JSF_FRAME_LEN + 7];
        jsf_rx_t r;
        memset(&f, 0, sizeof f);
        jsf_dev_clear(&f.dev[0]);
        jsf_dev_clear(&f.dev[1]);
        f.seq = 9u;
        f.flags = JSF_F_DEV0 | JSF_F_DEV1;
        f.dev[0].axis[0] = -12345;
        f.dev[0].buttons = 0x80000001u;
        f.dev[1].hats = 0xF6u;                      /* hat 0 = left */
        size_t n = jsf_encode(&f, b + 3);
        b[0] = 0xA5u; b[1] = 0x5Au; b[2] = 0x11u;   /* junk that looks like a start */
        memcpy(b + 3 + n, b + 3, n);
        b[3 + n + 20] ^= 0x10u;                     /* second copy corrupted */
        memcpy(b + 3 + 2 * n, b + 3, n);
        jsf_rx_init(&r);
        jsf_rx_push(&r, b, 3 + 3 * n);
        int good = 0;
        bool same = true;
        while (jsf_rx_next(&r, &g)) {
            good++;
            same = same && g.seq == 9u && g.dev[0].axis[0] == -12345 && g.dev[0].buttons == 0x80000001u &&
                   jsf_code_pressed(&g, JSF_CODE_HAT(1u, 0u, JSF_HAT_LEFT)) && jsf_code_pressed(&g, 32u) &&
                   !jsf_code_pressed(&g, JSF_CODE_HAT(1u, 0u, JSF_HAT_UP));
        }
        snprintf(d, sizeof d, "frame %u bytes, good frames %d of 2, CRC errors %lu, values and codes %s",
                 (unsigned)n, good, (unsigned long)r.crc_errors, same ? "OK" : "WRONG");
        test_report("U1 bridge frame: CRC, junk and rescan, button / hat codes", n == JSF_FRAME_LEN && good == 2 &&
                    r.crc_errors == 1u && same, d);
    }

    /* U2 / U3 link faults -> JOYSTICK FAULT, not in control / in control */
    {
        int ok_idle = 0, ok_drive = 0;
        char t1[400] = "", t2[500] = "";
        for (int k = 0; k < UF_COUNT; k++) {
            sbsim_t x;
            det_t dt = { 0u, -1, -1 };
            usb_init(&x, 700u + (uint32_t)k, 0.0f);
            sbsim_run(&x, 3000u, NULL, NULL);
            bool armed = x.sb.joy.armed;
            dt.t0 = x.t;
            usb_fault_inject(k);
            sbsim_run(&x, x.t + 1000u, det_watch, &dt);
            bool ok = armed && x.sb.joy.fault && x.sb.joy.fault_axis == UF_WHY[k] &&
                      (x.sb.alarm_bits & SB_ALM_JOY_FAULT) && sb_mode(&x.sb) == MODE_STANDBY && dt.t_fault <= 200;
            ok_idle += ok;
            snprintf(t1 + strlen(t1), sizeof t1 - strlen(t1), "%s%s %s %d ms", k ? ", " : "", UF_NAME[k],
                     ok ? "fault in" : "NOT DETECTED /", (int)dt.t_fault);
        }
        snprintf(d, sizeof d, "not in control: %s", t1);
        test_report("U2 bridge lost / corrupted / frozen / stick or throttle unplugged -> JOYSTICK FAULT",
                    ok_idle == UF_COUNT, d);

        for (int k = 0; k < UF_COUNT; k++) {
            sbsim_t x;
            events_t ev;
            det_t dt = { 0u, -1, -1 };
            memset(&ev, 0, sizeof ev);
            usb_init(&x, 710u + (uint32_t)k, 0.0f);
            usb_drive(&x, 0.9f, 15000u);
            float thr = sim_hal_throttle();
            bool driving = sb_mode(&x.sb) == MODE_MANUAL && thr > 5.0f && gear_is(&x.sb.gear, GEAR_F);
            dt.t0 = x.t;
            ev_reset(&ev, x.t, thr, 0u);
            usb_fault_inject(k);
            /* det_watch and ev_watch both */
            for (int i = 0; i < 300; i++) {
                sbsim_run(&x, x.t + AP_PERIOD_MS, ev_watch, &ev);
                det_watch(&x, &dt);
            }
            /* as the analog wire break: throttle down at once, the stick that steered is
             * gone -> gyro hold until idle + neutral, then steering off, no steering afterwards */
            bool order = ev.t_thr_down >= 0.0f && ev.t_thr_down <= 0.3f && ev.t_idle >= 0.0f &&
                         ev.t_neutral >= ev.t_idle && ev.t_steer_off >= ev.t_neutral &&
                         ev.steer_after_off == 0 && x.w.shift_under_load == 0;
            bool ok = driving && x.sb.joy.fault && dt.t_standby >= 0 && dt.t_standby <= 250 && order &&
                      sim_hal_throttle() <= 1.0f && sim_hal_gear_pwm() == x.sb.cfg.gear.pwm_n_us &&
                      x.sb.ap.mode == AP_MODE_STANDBY;
            ok_drive += ok;
                snprintf(t2 + strlen(t2), sizeof t2 - strlen(t2), "%s%s: STANDBY %d ms, throttle down %.2f s, steering stop %.2f s, idle %.1f s, N %.1f s%s",
                     k ? "; " : "", UF_NAME[k], (int)dt.t_standby, ev.t_thr_down, ev.t_steer_off, ev.t_idle, ev.t_neutral, ok ? "" : " FAIL");
        }
        snprintf(d, sizeof d, "driving at 90 %% lever: %s", t2);
        test_report("U3 same faults while the joystick drives -> idle + neutral (gyro hold), then steering off, STANDBY",
                    ok_drive == UF_COUNT, d);
    }

    /* U4 a few CRC errors (1 frame in 50) are tolerated */
    {
        sbsim_t x;
        usb_init(&x, 720u, 0.0f);
        sim_hal_bridge_fault(SIM_BR_CORRUPT_SOME);
        sbsim_run(&x, x.t + 10000u, NULL, NULL);
        snprintf(d, sizeof d, "10 s with 2 bad frames/s: CRC errors %lu, fault %s, armed %s",
                 (unsigned long)x.sb.jb.rx.crc_errors, x.sb.joy.fault ? "YES" : "no", x.sb.joy.armed ? "yes" : "NO");
        test_report("U4 occasional CRC errors below the threshold: no fault", !x.sb.joy.fault && x.sb.joy.armed &&
                    x.sb.jb.rx.crc_errors >= 15u, d);
    }

    /* U5 override from every mode: stick X, TAKE, lever */
    {
        static const sb_mode_t MODES[4] = { MODE_STANDBY, MODE_HDG_LOCK, MODE_AUTO, MODE_TRAINING };
        int good = 0, total = 0;
        char t[600] = "";
        for (int trig = 0; trig < 3; trig++) {
            for (int mi = 0; mi < 4; mi++) {
                sbsim_t x;
                usb_init(&x, 730u + (uint32_t)(trig * 4 + mi), 0.0f);
                if (MODES[mi] != MODE_STANDBY) {
                    power_full(&x);
                    goto_mode(&x, MODES[mi]);
                    sbsim_run(&x, x.t + 3000u, NULL, NULL);
                }
                bool in = sb_mode(&x.sb) == MODES[mi];
                if (trig == 0) { usb_stick(&x, 0.6f, 400u); usb_stick(&x, 0.0f, 100u); }
                else if (trig == 1) usb_press(&x, USB_TAKE);
                else { usb_lever(&x, 0.66f, 1.0f, NULL, NULL); sbsim_run(&x, x.t + 300u, NULL, NULL); }
                /* in HDG LOCK the lever IS the throttle: no override, HDG LOCK goes on */
                bool want_manual = !(trig == 2 && MODES[mi] == MODE_HDG_LOCK);
                bool ok = in && (want_manual ? sb_mode(&x.sb) == MODE_MANUAL : sb_mode(&x.sb) == MODE_HDG_LOCK);
                good += ok;
                total++;
                if (!ok) snprintf(t + strlen(t), sizeof t - strlen(t), " FAIL(%s from %s: %s)",
                                  trig == 0 ? "stick" : trig == 1 ? "TAKE" : "lever", mode_name(MODES[mi]),
                                  mode_name(sb_mode(&x.sb)));
            }
        }
        snprintf(d, sizeof d, "%d/%d: stick X, TAKE and lever (+16 %%) from STANDBY / HDG LOCK / AUTO / TRAINING -> MANUAL "
                 "(lever in HDG LOCK = throttle, stays HDG LOCK)%s", good, total, t);
        test_report("U5 override from every mode by stick, TAKE button and lever movement", good == total, d);
    }

    /* U6 throttle pickup: takeover with the lever left at 80 % */
    {
        sbsim_t x;
        thr_watch_t w;
        prog_wp_t wp[5];
        usb_init(&x, 740u, 0.0f);
        prog5(&x, wp);
        auto_go(&x, wp, 5, PROG_END_STOP);
        sbsim_run(&x, x.t + 30000u, NULL, NULL);
        float thr_auto = sim_hal_throttle();
        bool in_auto = sb_mode(&x.sb) == MODE_AUTO;
        /* shove the lever to 80 % within one step: override, lever not picked up */
        s_usb.lever = 0.8f;
        usb_send();
        thr_watch_reset(&w);
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        bool manual = sb_mode(&x.sb) == MODE_MANUAL, pend = x.sb.lev.pickup;
        float target = x.sb.lev.target;
        sbsim_run(&x, x.t + 5000u, thr_watch, &w);
        /* lever moved, but away from the pickup point (and not into neutral): still nothing */
        float away = (target > 0.8f) ? 0.62f : fminf(1.0f, target + 0.15f);
        usb_lever(&x, away, 0.2f, thr_watch, &w);
        sbsim_run(&x, x.t + 2000u, thr_watch, &w);
        float held_max = w.thr_max;
        bool still = x.sb.lev.pickup;
        float held = sim_hal_throttle();
        /* slowly to the pickup point: picked up there, then the lever drives */
        thr_watch_reset(&w);
        usb_lever(&x, x.sb.lev.target, 0.05f, thr_watch, &w);
        bool picked = !x.sb.lev.pickup;
        float at_pick = sim_hal_throttle();
        usb_lever(&x, 0.7f, 0.1f, thr_watch, &w);
        sbsim_run(&x, x.t + 3000u, thr_watch, &w);
        bool follows = sim_hal_throttle() < at_pick - 3.0f;
        snprintf(d, sizeof d, "AUTO %s at %.1f %%; lever shoved to 80 %%: MANUAL %s, pickup pending %s (target lever %.0f %% = "
                 "held %.0f %% at the 30 %% limit after the override); lever moved away to %.0f %%: throttle max %.1f %%, "
                 "held %.1f %%, still pending %s; lever to the target: picked up %s at %.1f %%, lever back to 70 %% -> %.1f %% "
                 "(fastest change %.1f %%/s)",
                 in_auto ? "yes" : "NO", thr_auto, manual ? "yes" : "NO", pend ? "yes" : "NO", target * 100.0f,
                 x.sb.plim.fwd_pct, away * 100.0f, held_max, held, still ? "yes" : "NO", picked ? "yes" : "NO", at_pick,
                 sim_hal_throttle(), w.max_rate);
        test_report("U6 throttle pickup: lever left at 80 % has no effect until it reaches the pickup point",
                    in_auto && manual && pend && held_max <= fmaxf(thr_auto, x.sb.plim.fwd_pct) + 0.5f && still && picked &&
                    fabsf(at_pick - held) < 1.0f && follows && w.max_rate <= x.sb.cfg.throttle.rate_down_pct_s + 1.0f, d);

        /* from STANDBY: lever at 80 % -> nothing until it was in the neutral zone */
        sbsim_t y;
        thr_watch_t w2;
        usb_init(&y, 741u, 0.0f);
        s_usb.lever = 0.8f;
        usb_send();
        thr_watch_reset(&w2);
        sbsim_run(&y, y.t + 5000u, thr_watch, &w2);
        bool m2 = sb_mode(&y.sb) == MODE_MANUAL, idle = w2.thr_max <= 1.0f && sim_hal_gear_pwm() == y.sb.cfg.gear.pwm_n_us;
        usb_lever(&y, 0.5f, 0.5f, NULL, NULL);
        usb_lever(&y, 0.8f, 0.5f, NULL, NULL);
        sbsim_run(&y, y.t + 8000u, NULL, NULL);
        snprintf(d, sizeof d, "STANDBY, lever pushed to 80 %%: MANUAL %s, throttle max %.1f %% and neutral %s; "
                 "via neutral and forward again: throttle %.1f %% gear %s",
                 m2 ? "yes" : "NO", w2.thr_max, idle ? "yes" : "NO", sim_hal_throttle(),
                 gear_is(&y.sb.gear, GEAR_F) ? "F" : "?");
        test_report("U6b pickup from STANDBY: lever at 80 % -> idle + neutral until it passed the neutral zone",
                    m2 && idle && sim_hal_throttle() > 5.0f && gear_is(&y.sb.gear, GEAR_F), d);
    }

    /* U7 power limit raised while the lever is at full forward */
    {
        sbsim_t x;
        thr_watch_t w;
        usb_init(&x, 750u, 0.0f);
        usb_drive(&x, 1.0f, 15000u);
        float before = sim_hal_throttle();
        bool raised = power_step(&x, 0u);           /* 30 -> 50 %: code + USB CONFIRM */
        thr_watch_reset(&w);
        sbsim_run(&x, x.t + 5000u, thr_watch, &w);
        float after5 = w.thr_max;
        usb_lever(&x, 0.85f, 0.5f, NULL, NULL);
        thr_watch_reset(&w);
        usb_lever(&x, 1.0f, 0.2f, thr_watch, &w);
        sbsim_run(&x, x.t + 10000u, thr_watch, &w);
        snprintf(d, sizeof d, "full lever: %.1f %% (limit 30); raised to %.0f %% %s; next 5 s max %.1f %%; "
                 "lever back to 85 %% and forward again: %.1f %% (fastest %.1f %%/s)",
                 before, x.sb.plim.fwd_pct, raised ? "yes" : "NO", after5, sim_hal_throttle(), w.max_rate);
        test_report("U7 power limit raised at full lever: no throttle jump, more only after the lever moves",
                    raised && before >= 29.0f && before <= 30.5f && after5 <= before + 0.5f &&
                    sim_hal_throttle() >= 49.0f && w.max_rate <= x.sb.cfg.throttle.rate_up_pct_s + 1.0f, d);
    }

    /* U8 power-up with the lever outside the neutral zone / stick not centred */
    {
        int ok = 0;
        char t[400] = "";
        for (int k = 0; k < 2; k++) {
            sbsim_t x;
            usb_init_at(&x, 760u + (uint32_t)k, 0.0f, k ? 0.4f : 0.0f, k ? 0.5f : 0.8f);
            sbsim_run(&x, 5000u, NULL, NULL);
            bool dis = !x.sb.joy.armed && (x.sb.alarm_bits & SB_ALM_JOY_NOT_CENTERED) && !x.sb.armed;
            usb_press(&x, USB_TAKE);
            bool ignored = sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 1.0f;
            if (k) usb_stick(&x, 0.0f, 100u); else usb_lever(&x, 0.5f, 0.5f, NULL, NULL);
            sbsim_run(&x, x.t + 1500u, NULL, NULL);
            bool armed = x.sb.joy.armed && x.sb.armed;
            ok += dis && ignored && armed;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s: disarmed + JOYSTICK NOT IN NEUTRAL %s, TAKE ignored %s, "
                     "armed after neutral %s", k ? "; " : "", k ? "stick at 40 %" : "lever at 80 %",
                     dis ? "yes" : "NO", ignored ? "yes" : "NO", armed ? "yes" : "NO");
        }
        test_report("U8 power-up with lever out of neutral or stick not centred -> disarmed + alarm", ok == 2, t);
    }

    /* U9 lever through the neutral zone, forward -> reverse: safe gear sequence */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        usb_init(&x, 770u, 0.0f);
        usb_drive(&x, 1.0f, 30000u);
        float v = x.w.speed_kn;
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_r_us);
        usb_lever(&x, 0.0f, 2.0f, ev_watch, &ev);            /* full forward -> full reverse in 0.5 s */
        sbsim_run(&x, x.t + 20000u, ev_watch, &ev);
        const gear_config_t *g = &x.sb.cfg.gear;
        bool order = ev.t_idle >= 0.0f && ev.t_neutral >= ev.t_idle + g->idle_wait_ms * 0.001f - 0.06f &&
                     ev.t_gear >= ev.t_neutral + g->neutral_wait_ms * 0.001f - 0.06f &&
                     ev.t_thr_open >= ev.t_gear + g->engage_wait_ms * 0.001f - 0.06f;
        snprintf(d, sizeof d, "at %.1f kn: idle %.2f s, neutral %.2f s, reverse %.2f s, throttle %.2f s; under-load shifts %d, "
                 "F/R without neutral %d, max reverse throttle %.1f %% (limit %.0f %%)", v, ev.t_idle, ev.t_neutral, ev.t_gear,
                 ev.t_thr_open, x.w.shift_under_load, x.w.fr_without_neutral, x.w.max_thr_rev, x.sb.plim.rev_pct);
        test_report("U9 lever forward -> reverse through the neutral zone: idle, wait, N, wait, R, wait, throttle",
                    order && x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0 &&
                    x.w.max_thr_rev <= x.sb.plim.rev_pct + 0.01f, d);
    }

    /* U10 hand-back refused unless stick centred AND lever in the neutral zone */
    {
        sbsim_t x;
        usb_init(&x, 780u, 0.0f);
        power_full(&x);
        press(&x, HAL_BTN_HDG_LOCK);
        usb_press(&x, USB_TAKE);                     /* MANUAL from HDG LOCK */
        bool manual = sb_mode(&x.sb) == MODE_MANUAL;
        s_usb.x = 0.3f;
        usb_send();
        usb_press(&x, USB_AUTO);
        bool r1 = sb_mode(&x.sb) == MODE_MANUAL;
        s_usb.x = 0.0f;
        usb_send();
        usb_lever(&x, 0.7f, 0.5f, NULL, NULL);
        usb_press(&x, USB_AUTO);
        bool r2 = sb_mode(&x.sb) == MODE_MANUAL;
        usb_press(&x, USB_HDG);
        bool r3 = sb_mode(&x.sb) == MODE_MANUAL;
        usb_lever(&x, 0.5f, 0.5f, NULL, NULL);
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        usb_press(&x, USB_AUTO);
        bool back = sb_mode(&x.sb) == MODE_HDG_LOCK;
        snprintf(d, sizeof d, "MANUAL from HDG LOCK %s; AUTO with stick at 30 %%: %s; AUTO with lever at 70 %%: %s; "
                 "HDG LOCK button with lever at 70 %%: %s; both neutral: back to HDG LOCK %s",
                 manual ? "yes" : "NO", r1 ? "refused" : "ACCEPTED", r2 ? "refused" : "ACCEPTED",
                 r3 ? "refused" : "ACCEPTED", back ? "yes" : "NO");
        test_report("U10 hand-back refused when the stick is not centred or the lever not in neutral",
                    manual && r1 && r2 && r3 && back, d);
    }

    /* U11 CONFIRM on the USB button: raise + AUTO start work, refusals still refused */
    {
        sbsim_t x;
        uint16_t code;
        prog_wp_t wp[5];
        usb_init(&x, 790u, 0.0f);
        bool raise = power_step(&x, 0u) && x.sb.plim.fwd_pct == 50.0f;
        prog5(&x, wp);
        bool aut = auto_go(&x, wp, 5, PROG_END_STOP);
        sbsim_t y;
        usb_init(&y, 791u, 0.0f);
        /* nothing pending: ignored */
        uint32_t ign = y.sb.confirm.ignored;
        usb_press(&y, USB_CONFIRM);
        bool r_none = y.sb.confirm.ignored == ign + 1u && y.sb.plim.fwd_pct == 30.0f;
        /* stick not centred (12 %, inside the deadband): refused */
        sb_power_code(&y.sb, 0u, &code);
        sb_power_raise(&y.sb, 0u, code);
        usb_stick(&y, 0.12f, 300u);
        usb_press(&y, USB_CONFIRM);
        bool r_stick = y.sb.plim.fwd_pct == 30.0f;
        usb_stick(&y, 0.0f, 300u);
        /* too late (> 10 s) */
        sb_power_code(&y.sb, 0u, &code);
        sb_power_raise(&y.sb, 0u, code);
        sbsim_run(&y, y.t + 10500u, NULL, NULL);
        usb_press(&y, USB_CONFIRM);
        bool r_late = y.sb.plim.fwd_pct == 30.0f;
        /* wrong code */
        sb_power_code(&y.sb, 0u, &code);
        bool r_code = sb_power_raise(&y.sb, 0u, (uint16_t)(code ^ 0x55u)) != SB_OK;
        usb_press(&y, USB_CONFIRM);
        r_code = r_code && y.sb.plim.fwd_pct == 30.0f;
        /* joystick faulted (throttle unplugged): no CONFIRM possible */
        s_usb.flags &= (uint8_t)~JSF_F_DEV1;
        usb_send();
        sbsim_run(&y, y.t + 500u, NULL, NULL);
        sb_power_code(&y.sb, 0u, &code);
        bool r_fault = sb_power_raise(&y.sb, 0u, code) == SB_ERR_JOY_NOT_READY;
        snprintf(d, sizeof d, "raise 30->50 with USB CONFIRM %s, AUTO start %s; refused: nothing pending %s, stick not centred %s, "
                 "late %s, wrong code %s, joystick faulted %s",
                 raise ? "OK" : "FAILED", aut ? "OK" : "FAILED", r_none ? "yes" : "NO", r_stick ? "yes" : "NO",
                 r_late ? "yes" : "NO", r_code ? "yes" : "NO", r_fault ? "yes" : "NO");
        test_report("U11 CONFIRM via the USB button: power raise and AUTO start; every refusal case still refused",
                    raise && aut && r_none && r_stick && r_late && r_code && r_fault, d);
    }

    /* U12 USB panel functions: STANDBY, HDG LOCK, hat +-1 / +-10; the panel STANDBY still wins */
    {
        sbsim_t x;
        usb_init(&x, 800u, 0.0f);
        power_full(&x);
        usb_press(&x, USB_HDG);
        bool hdg = sb_mode(&x.sb) == MODE_HDG_LOCK;
        float t0 = x.sb.ap.target_heading;
        usb_press(&x, JSF_CODE_HAT(1u, 0u, JSF_HAT_RIGHT));
        usb_press(&x, JSF_CODE_HAT(1u, 0u, JSF_HAT_UP));
        usb_press(&x, JSF_CODE_HAT(1u, 0u, JSF_HAT_LEFT));
        usb_press(&x, JSF_CODE_HAT(1u, 0u, JSF_HAT_LEFT));
        float dh = hdg_err(x.sb.ap.target_heading, t0);
        usb_press(&x, USB_STANDBY);
        bool sb_usb = sb_mode(&x.sb) == MODE_STANDBY;
        usb_press(&x, USB_TAKE);
        bool man = sb_mode(&x.sb) == MODE_MANUAL;
        usb_code(USB_TAKE, true);                   /* TAKE held, stick deflected ... */
        s_usb.x = 0.8f;
        usb_send();
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        sim_hal_press(HAL_BTN_STANDBY);             /* ... panel STANDBY */
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool panel = sb_mode(&x.sb) == MODE_STANDBY;
        usb_code(USB_TAKE, false);
        s_usb.x = 0.0f;
        usb_send();
        snprintf(d, sizeof d, "USB HDG LOCK %s; hat +1 +10 -1 -1 -> %+.0f deg; USB STANDBY %s; panel STANDBY with stick "
                 "deflected and TAKE held %s", hdg ? "OK" : "FAILED", dh, sb_usb ? "OK" : "FAILED",
                 panel ? "wins" : "LOST");
        test_report("U12 USB HDG LOCK / STANDBY / hat course buttons; the panel STANDBY button always wins",
                    hdg && fabsf(dh - 9.0f) < 0.01f && sb_usb && man && panel, d);
    }

    /* U13 button map: duplicates refused, saved with checksum, corrupted copy -> default */
    {
        sbsim_t x;
        joymap_t m;
        jm_error_t e1, e2, e3, e4, e5;
        usb_init(&x, 810u, 0.0f);
        joymap_default(&m);
        m.source = JOY_SRC_BRIDGE;
        m.btn[JM_AUTO] = m.btn[JM_TAKE];
        bool d1 = !sb_joymap_set(&x.sb, &m, true, &e1) && e1 == JM_ERR_DUPLICATE;
        joymap_default(&m); m.source = JOY_SRC_BRIDGE; m.btn[JM_CONFIRM] = m.btn[JM_AUTO];
        bool d2 = !sb_joymap_set(&x.sb, &m, true, &e2) && e2 == JM_ERR_DUPLICATE;
        joymap_default(&m); m.source = JOY_SRC_BRIDGE; m.btn[JM_CONFIRM] = m.btn[JM_TAKE];
        bool d3 = !sb_joymap_set(&x.sb, &m, true, &e3) && e3 == JM_ERR_DUPLICATE;
        joymap_default(&m); m.source = JOY_SRC_BRIDGE; m.btn[JM_TAKE] = 0u;
        bool d4 = !sb_joymap_set(&x.sb, &m, true, &e4) && e4 == JM_ERR_MISSING;
        /* a good new map: CONFIRM moved to TWCS button 5 */
        joymap_default(&m); m.source = JOY_SRC_BRIDGE; m.btn[JM_CONFIRM] = JSF_CODE_BTN(1u, 4u);
        bool good = sb_joymap_set(&x.sb, &m, true, &e5);
        sbsim_run(&x, x.t + 500u, NULL, NULL);       /* joystick re-arms (centred) */
        usb_press(&x, USB_TAKE);                     /* not in STANDBY now */
        jm_error_t e6;
        bool not_sb = !sb_joymap_set(&x.sb, &m, true, &e6) && e6 == JM_OK;
        sb_config_t c = x.sb.cfg;
        sb_init(&x.sb, &c);                          /* power cycle: map loaded back */
        bool kept = x.sb.jmap.btn[JM_CONFIRM] == JSF_CODE_BTN(1u, 4u) && x.sb.jmap.source == JOY_SRC_BRIDGE;
        FILE *f = fopen("nv4.bin", "r+b");
        if (f) { fseek(f, 20, SEEK_SET); fputc(0x77, f); fclose(f); }
        sb_init(&x.sb, &c);
        bool deflt = x.sb.jmap.source == JOY_SRC_ANALOG;
        snprintf(d, sizeof d, "refused: TAKE=AUTO %s, AUTO=CONFIRM %s, TAKE=CONFIRM %s, TAKE missing %s, outside STANDBY %s; "
                 "good map %s and kept after power cycle %s; corrupted copy -> factory default %s",
                 d1 ? "yes" : "NO", d2 ? "yes" : "NO", d3 ? "yes" : "NO", d4 ? "yes" : "NO", not_sb ? "yes" : "NO",
                 good ? "accepted" : "REFUSED", kept ? "yes" : "NO", deflt ? "yes" : "NO");
        test_report("U13 button map: shared TAKE / AUTO / CONFIRM refused, stored with checksum",
                    d1 && d2 && d3 && d4 && good && not_sb && kept && deflt, d);
        remove("nv4.bin");
    }

    /* U14 bridge still starting after power-up: not ready, no alarm; never seen -> fault */
    {
        sbsim_t x;
        sbsim_init(&x, 820u, 90.0f, 0.0f, NULL);
        joymap_t m;
        jm_error_t e;
        joymap_default(&m);
        m.source = JOY_SRC_BRIDGE;
        sim_hal_bridge_mode(true);
        s_usb.on = true;
        s_usb.lever = 0.5f;
        s_usb.hat1 = 15u;
        s_usb.flags = JSF_F_HUB;                     /* hub up, joysticks still enumerating */
        usb_send();
        (void)sb_joymap_set(&x.sb, &m, false, &e);
        sbsim_run(&x, 3000u, NULL, NULL);
        bool quiet = !x.sb.joy.fault && !(x.sb.alarm_bits & SB_ALM_JOY_FAULT) && !x.sb.joy.armed;
        s_usb.flags = JSF_F_DEV0 | JSF_F_DEV1 | JSF_F_HUB;
        usb_send();
        sbsim_run(&x, 4500u, NULL, NULL);
        bool up = x.sb.joy.armed && x.sb.armed;
        sbsim_t y;
        sbsim_init(&y, 821u, 90.0f, 0.0f, NULL);
        sim_hal_bridge_mode(true);
        sim_hal_bridge_fault(SIM_BR_STOP);           /* bridge never sends */
        (void)sb_joymap_set(&y.sb, &m, false, &e);
        sbsim_run(&y, 7000u, NULL, NULL);
        bool dead = y.sb.joy.fault && (y.sb.alarm_bits & SB_ALM_JOY_FAULT) && y.sb.joy.fault_axis == JOY_WHY_TIMEOUT;
        snprintf(d, sizeof d, "USB enumeration (3 s): no fault, no alarm, not armed %s; joysticks up: armed %s; "
                 "bridge silent 5 s after power-up: JOYSTICK FAULT %s", quiet ? "yes" : "NO", up ? "yes" : "NO",
                 dead ? "yes" : "NO");
        test_report("U14 bridge start-up: quiet while USB enumerates, fault if it never comes", quiet && up && dead, d);
    }

    /* U15 latency: TAKE on the joystick -> MANUAL on the AURIX, simulated
     * bridge (USB poll 1 ms + hub 1 ms, 10 ms frames, 2.5 ms UART), random
     * phases of the press, the bridge frame tick and the 20 Hz loop */
    {
        uint32_t rng = 12345u, n = 0u;
        int32_t lmin = 100000, lmax = -1;
        double sum = 0.0;
        int32_t hist[200];
        int bad = 0;
        for (int k = 0; k < 200; k++) {
            sbsim_t x;
            lat_t l;
            usb_init(&x, 900u + (uint32_t)k, 0.0f);
            rng = rng * 1103515245u + 12345u;
            sim_hal_bridge_timing(2u, 3u, (rng >> 16) % JSF_PERIOD_MS);
            rng = rng * 1103515245u + 12345u;
            sbsim_run(&x, x.t + 200u + (rng >> 16) % AP_PERIOD_MS, NULL, NULL);
            l.t_press = x.t;
            l.lat = -1;
            sim_hal_set_time_ms(x.t);               /* the press happens now (not at the last physics step) */
            usb_code(USB_TAKE, true);
            for (int i = 0; i < 400 && l.lat < 0; i++) sbsim_run(&x, x.t + 1u, lat_watch, &l);
            usb_code(USB_TAKE, false);
            if (l.lat < 0) { bad++; continue; }
            hist[n++] = l.lat;
            sum += l.lat;
            if (l.lat < lmin) lmin = l.lat;
            if (l.lat > lmax) lmax = l.lat;
        }
        /* 95th percentile */
        for (uint32_t i = 1; i < n; i++)
            for (uint32_t k = i; k > 0 && hist[k - 1] > hist[k]; k--) { int32_t t = hist[k]; hist[k] = hist[k - 1]; hist[k - 1] = t; }
        int32_t p95 = n ? hist[(n * 95u) / 100u] : -1;
        snprintf(d, sizeof d, "%lu presses: min %ld ms, average %.1f ms, 95 %% %ld ms, max %ld ms (target < 100 ms; "
                 "worst case by design 2 + 10 + 3 + 50 = 65 ms), missed %d",
                 (unsigned long)n, (long)lmin, n ? sum / n : 0.0, (long)p95, (long)lmax, bad);
        test_report("U15 latency TAKE button -> mode change, through the bridge and hub", bad == 0 && n == 200u && lmax < 100, d);
    }

    /* U16 ground-station link: source switch, map get / set / duplicate refused,
     * live telemetry (link status, lever, buttons), raw frame for the mapping routine */
    {
        static pcsim_t p;
        uint8_t body[33];                                   /* JOYMAP_BODY (32) + save flag */
        joymap_t m;
        pcsim_init(&p, 830u);
        sim_hal_bridge_mode(true);
        s_usb.on = true;
        s_usb.lever = 0.5f;
        s_usb.hat1 = 15u;
        s_usb.flags = JSF_F_DEV0 | JSF_F_DEV1 | JSF_F_HUB;
        usb_send();
        pc_run(&p, 500u);
        /* switch to the USB bridge (default T.16000M + TWCS map), saved */
        joymap_default(&m);
        m.source = JOY_SRC_BRIDGE;
        proto_wr_t w;
        proto_wr_init(&w, body, sizeof body);
        proto_wr_u8(&w, m.source); proto_wr_u8(&w, m.thr_input); proto_wr_u8(&w, m.axis_steer); proto_wr_u8(&w, m.axis_thr);
        proto_wr_u8(&w, m.inv_steer); proto_wr_u8(&w, m.inv_thr);
        for (int i = 0; i < JM_COUNT; i++) proto_wr_u8(&w, m.btn[i]);
        proto_wr_f32(&w, m.nz_lo); proto_wr_f32(&w, m.nz_hi); proto_wr_f32(&w, m.lever_delta); proto_wr_f32(&w, m.pickup_win);
        proto_wr_u8(&w, 1u);
        uint8_t r_set = pc_cmd(&p, PROTO_T_JOYMAP_SET, body, (uint8_t)w.n);
        pc_run(&p, 1500u);
        /* telemetry tail (after the 172 bytes before this feature) */
        proto_rd_t r;
        proto_rd_init(&r, p.tm, p.tm_len);
        r.n = 172u;
        uint8_t src = proto_rd_u8(&r), fl = proto_rd_u8(&r), fps = proto_rd_u8(&r);
        (void)proto_rd_u16(&r);
        uint8_t why = proto_rd_u8(&r);
        uint16_t lever = proto_rd_u16(&r);
        (void)proto_rd_u16(&r);
        uint32_t b0 = proto_rd_u32(&r);
        (void)proto_rd_u32(&r);
        (void)proto_rd_u8(&r);
        uint8_t nzl = proto_rd_u8(&r), nzh = proto_rd_u8(&r);
        bool tel = p.tm_len == 193u && src == 1u && (fl & 0x13u) == 0x13u && fps >= 95u && why == 0u &&
                   lever == 500u && nzl == 45u && nzh == 55u;
        /* TAKE held: telemetry + raw frame show stick button 1 */
        usb_code(USB_TAKE, true);
        pc_run(&p, 200u);
        uint8_t r_raw = pc_cmd(&p, PROTO_T_JOY_RAW_GET, NULL, 0u);
        proto_rd_init(&r, p.tm, p.tm_len);
        r.n = 172u + 10u;
        b0 = proto_rd_u32(&r);
        proto_rd_init(&r, p.raw, p.raw_len);
        r.n = 2u + 16u;
        uint32_t raw_b0 = proto_rd_u32(&r);
        usb_code(USB_TAKE, false);
        pc_run(&p, 300u);
        bool live = r_raw == PROTO_R_OK && p.raw_len == 44u && (b0 & 1u) && (raw_b0 & 1u);
        /* map change outside STANDBY refused (TAKE made it MANUAL) */
        uint8_t r_nsb = pc_cmd(&p, PROTO_T_JOYMAP_SET, body, (uint8_t)w.n);
        sim_hal_press(HAL_BTN_STANDBY);
        pc_run(&p, 500u);
        /* duplicate TAKE = AUTO refused through the link, map unchanged */
        body[6 + JM_AUTO] = body[6 + JM_TAKE];
        uint8_t r_dup = pc_cmd(&p, PROTO_T_JOYMAP_SET, body, (uint8_t)w.n);
        uint8_t r_get = pc_cmd(&p, PROTO_T_JOYMAP_GET, NULL, 0u);
        bool unchanged = p.jm_len == 32u && p.jm[0] == 1u && p.jm[6 + JM_AUTO] == JSF_CODE_BTN(1u, 0u);
        snprintf(d, sizeof d, "switch to USB %s; telemetry %u bytes: source %u, stick+throttle connected %s, %u frames/s, "
                 "lever %.1f %%, neutral zone %u-%u %%; TAKE seen in telemetry + raw frame %s; in MANUAL: result %u (9 = STANDBY only); "
                 "TAKE=AUTO map: result %u (18 = shared button), map unchanged %s",
                 r_set == PROTO_R_OK ? "OK" : "FAILED", (unsigned)p.tm_len, (unsigned)src, (fl & 3u) == 3u ? "yes" : "NO",
                 (unsigned)fps, lever / 10.0f, (unsigned)nzl, (unsigned)nzh, live ? "yes" : "NO", (unsigned)r_nsb, (unsigned)r_dup,
                 unchanged && r_get == PROTO_R_OK ? "yes" : "NO");
        test_report("U16 ground-station link: source, map get/set, shared button refused, live joystick status",
                    r_set == PROTO_R_OK && tel && live && r_nsb == PROTO_R_STANDBY_ONLY && r_dup == PROTO_R_MAP_DUPLICATE && r_get == PROTO_R_OK && unchanged, d);
        remove("nv4.bin");
    }
}

/* ------------------------------------------------- gateway (section A) -- */

/* The Raspberry Pi side, in C: reads the gateway UART, rebuilds the log
 * records, counts frames, sends GW_STATUS like the real gateway. */
typedef struct {
    link_t     link;            /* the AURIX's gateway port */
    proto_rx_t rx;
    bool       pi_on;           /* the Pi sends its 1 Hz status */
    uint8_t    flags;           /* GW_STATUS flags it sends */
    uint32_t   next_status;
    int        telem, telem2, times, chunks;
    uint32_t   recs, gaps, first_seq, last_seq, bad;
    bool       have_seq;
    uint32_t   cur_seq, cur_len, cur_got;
    uint8_t    cur[4200];
    uint32_t   time_date, time_tod, time_valid;
    uint32_t   max_telem_gap_ms, last_telem_ms, max_gap_at;
    int        acks;
    uint8_t    ack_station, ack_result;
    uint16_t   ack_seq;
    uint16_t   save_seq;        /* TELEM2 group E: latest video SAVE request */
    uint32_t   save_mask;       /* bit per trigger seen */
    uint32_t   save_changes;
    uint8_t    save_last_arg_take;
} pisim_t;

static void pi_frame(pisim_t *p, sbsim_t *x, const proto_frame_t *f)
{
    proto_rd_t r;
    proto_rd_init(&r, f->payload, f->len);
    if (f->type == PROTO_T_TELEMETRY) {
        if (p->telem && x->t - p->last_telem_ms > p->max_telem_gap_ms) {
            p->max_telem_gap_ms = x->t - p->last_telem_ms;
            p->max_gap_at = x->t;
        }
        p->last_telem_ms = x->t;
        p->telem++;
    } else if (f->type == PROTO_T_TELEM2) {
        p->telem2++;
        if (f->len >= 59u) {                        /* group E at byte 50 */
            uint16_t sq = (uint16_t)(f->payload[50] | (f->payload[51] << 8));
            if (sq != p->save_seq) {
                p->save_seq = sq;
                p->save_changes++;
                p->save_mask |= 1u << f->payload[52];
                p->save_mask |= f->payload[58];             /* every kind of the last 2 s */
                if (f->payload[52] == SB_SAVE_TAKEOVER) p->save_last_arg_take = f->payload[53];
            }
        }
    } else if (f->type == PROTO_T_GW_ACK && f->len == 5u) {
        p->ack_station = f->payload[0];
        p->ack_seq = (uint16_t)(f->payload[1] | (f->payload[2] << 8));
        p->ack_result = f->payload[4];
        p->acks++;
    } else if (f->type == PROTO_T_TIME) {
        p->time_valid = proto_rd_u8(&r);
        p->time_date = proto_rd_u32(&r);
        p->time_tod = proto_rd_u32(&r);
        p->times++;
    } else if (f->type == PROTO_T_LOG_CHUNK) {
        uint32_t seq = proto_rd_u32(&r);
        uint16_t off = proto_rd_u16(&r), len = proto_rd_u16(&r);
        uint32_t n = f->len - 8u;
        p->chunks++;
        if (off == 0u) {
            if (p->have_seq && seq != p->last_seq + 1u) p->gaps++;
            if (!p->have_seq) p->first_seq = seq;
            p->cur_seq = seq;
            p->cur_len = len;
            p->cur_got = 0u;
        }
        if (seq != p->cur_seq || off != p->cur_got || p->cur_got + n > sizeof p->cur) { p->bad++; return; }
        memcpy(&p->cur[p->cur_got], &f->payload[8], n);
        p->cur_got += n;
        if (p->cur_got == p->cur_len) {
            /* a whole record: sync byte + CRC as written to the SD card */
            uint32_t L = p->cur_len;
            uint16_t crc = (uint16_t)(p->cur[L - 2u] | (p->cur[L - 1u] << 8));
            if (p->cur[0] != LOGR_SYNC || proto_crc16(&p->cur[1], L - 3u) != crc) p->bad++;
            p->recs++;
            p->last_seq = seq;
            p->have_seq = true;
        }
    }
}

static void pi_hook(sbsim_t *x, void *ctx)
{
    pisim_t *p = (pisim_t *)ctx;
    link_step(&p->link, &x->sb);
    if (p->pi_on && x->t >= p->next_status) {
        p->next_status = x->t + 1000u;
        uint8_t pl[12] = { 1u, p->flags, 1u, 1u };
        memcpy(&pl[4], &p->last_seq, 4);
        uint8_t fr[PROTO_MAX_FRAME];
        size_t n = proto_encode(fr, PROTO_T_GW_STATUS, 0u, pl, sizeof pl);
        sim_hal_gw_rx_push(fr, (uint32_t)n);
    }
    uint8_t buf[2048];
    uint32_t n;
    proto_frame_t f;
    while ((n = sim_hal_gw_tx_pop(buf, 256u)) > 0u) {         /* like the real Pi: parse as it comes */
        proto_rx_push(&p->rx, buf, n);
        while (proto_rx_next(&p->rx, &f)) pi_frame(p, x, &f);
    }
}

static void pisim_init(pisim_t *p, sbsim_t *x, uint32_t baud)
{
    memset(p, 0, sizeof *p);
    sim_hal_gw_reset(baud);
    link_init_port(&p->link, &x->sb, LINK_PORT_GW);
    proto_rx_init(&p->rx);
    p->pi_on = true;
    p->next_status = x->t;
}

static void gateway_tests(void)
{
    char d[600];
    printf("\n=== Section A: gateway (Raspberry Pi) link ===\n");

    /* GA1 full log copy at 921600 baud: every record, in order, CRC good */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 901u, 90.0f, 0.0f, NULL);
        sbsim_log_start(&x, "gwtest_sd", 7u);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        sbsim_run(&x, 120000u, pi_hook, &p);
        uint32_t made = x.sb.gw.log_sent_seq;
        snprintf(d, sizeof d, "%lu records rebuilt on the Pi (numbers %lu..%lu, logger handled %lu), gaps %lu, "
                 "bad records %lu, dropped for the Pi %lu; telemetry %d, TELEM2 %d, TIME %d; SD card ok %s",
                 (unsigned long)p.recs, (unsigned long)p.first_seq, (unsigned long)p.last_seq,
                 (unsigned long)g_logstat.records, (unsigned long)p.gaps, (unsigned long)p.bad,
                 (unsigned long)g_logq_gw.dropped, p.telem, p.telem2, p.times, g_logstat.failure ? "NO" : "yes");
        test_report("GA1 full log copy to the Pi over the UART: every record, in order, CRC good",
                    p.recs > 2000u && p.gaps == 0u && p.bad == 0u && g_logq_gw.dropped == 0u &&
                    p.last_seq + 2u >= made && x.sb.gw.state == SB_GW_OK && !g_logstat.failure, d);
        logger_close(&s_logger);
        x.log_on = false;
    }

    /* GA2 UART too slow (19200 baud): control frames keep their rate, log
     * records are dropped for the Pi (gap noted), the SD card stays complete */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 902u, 90.0f, 0.0f, NULL);
        sbsim_log_start(&x, "gwtest_sd", 8u);
        pisim_init(&p, &x, 57600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        sbsim_run(&x, x.t + 500u, pi_hook, &p);
        p.max_telem_gap_ms = 0u;
        p.last_telem_ms = x.t;
        sbsim_run(&x, 60000u, pi_hook, &p);
        bool sd_ok = !g_logstat.failure && g_logq0.dropped == 0u;
        snprintf(d, sizeof d, "57600 baud: telemetry frames %d of 600, none lost (max gap %lu ms at %.1f s: 10 Hz + at most one 50 ms step late), TIME %d; log records to the Pi %lu, "
                 "dropped for the Pi %lu, gaps the Pi noticed %lu, bad %lu; SD: %lu records, none dropped %s",
                 p.telem, (unsigned long)p.max_telem_gap_ms, p.max_gap_at * 0.001, p.times, (unsigned long)p.recs,
                 (unsigned long)g_logq_gw.dropped, (unsigned long)p.gaps, (unsigned long)p.bad,
                 (unsigned long)g_logstat.records, sd_ok ? "yes" : "NO");
        test_report("GA2 slow UART: log copy dropped first (gap noted), control traffic and SD card unaffected",
                    p.telem >= 590 && p.max_telem_gap_ms <= 250u && p.times >= 56 && g_logq_gw.dropped > 0u &&
                    p.gaps > 0u && p.bad == 0u && sd_ok, d);
        logger_close(&s_logger);
        x.log_on = false;
    }

    /* GA3 Pi unplugged: GATEWAY LOST after 3 s, no mode change, USB link fine;
     * back: state OK again */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 903u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        bool ok0 = x.sb.gw.state == SB_GW_OK;
        hdg_lock(&x);
        sb_mode_t m0 = sb_mode(&x.sb);
        p.pi_on = false;
        uint32_t t_off = x.t;
        uint32_t t_alarm = 0u;
        while (x.t < t_off + 6000u) {
            sbsim_run(&x, x.t + 50u, pi_hook, &p);
            if (!t_alarm && (x.sb.alarm_bits & SB_ALM_GATEWAY)) t_alarm = x.t - t_off;
        }
        bool same = sb_mode(&x.sb) == m0;
        uint8_t st = x.sb.gw.state;
        /* the USB laptop link still answers */
        sim_hal_link_reset();
        static link_t usb;
        link_init(&usb, &x.sb);
        uint8_t fr[PROTO_MAX_FRAME], b = 3u;
        size_t n = proto_encode(fr, PROTO_T_SEA_STATE, 5u, &b, 1u);
        sim_hal_link_rx_push(fr, (uint32_t)n);
        link_step(&usb, &x.sb);
        p.pi_on = true;
        p.next_status = x.t;
        sbsim_run(&x, x.t + 3000u, pi_hook, &p);
        snprintf(d, sizeof d, "Pi OK %s; unplugged in HDG LOCK: GATEWAY LOST after %.1f s, mode still %s, state %s; "
                 "USB laptop command answered %s; Pi back: state %s",
                 ok0 ? "yes" : "NO", t_alarm * 0.001, mode_name(sb_mode(&x.sb)), st == SB_GW_LOST ? "LOST" : "?",
                 usb.last_result == PROTO_R_OK ? "OK" : "NO", x.sb.gw.state == SB_GW_OK ? "OK" : "NO");
        test_report("GA3 gateway lost: alarm GATEWAY LOST, no mode change, USB link still works",
                    ok0 && t_alarm >= 3000u && t_alarm <= 3600u && same && st == SB_GW_LOST &&
                    usb.last_result == PROTO_R_OK && x.sb.gw.state == SB_GW_OK, d);
    }

    /* GA4 Pi storage problem -> alarm GATEWAY STORAGE, nothing else changes */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 904u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        sb_mode_t m0 = sb_mode(&x.sb);
        p.flags = SB_GWF_STORAGE;
        sbsim_run(&x, x.t + 2000u, pi_hook, &p);
        snprintf(d, sizeof d, "SSD missing: alarm GATEWAY STORAGE %s, mode %s -> %s",
                 (x.sb.alarm_bits & SB_ALM_GW_STORAGE) ? "yes" : "NO", mode_name(m0), mode_name(sb_mode(&x.sb)));
        test_report("GA4 gateway storage problem -> alarm GATEWAY STORAGE, no mode change",
                    (x.sb.alarm_bits & SB_ALM_GW_STORAGE) && sb_mode(&x.sb) == m0, d);
    }

    /* GA5 GPS time for the Pi */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 905u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 5000u, pi_hook, &p);
        uint32_t want = (36000000u + x.sb.now_ms) % 86400000u;
        int32_t diff = (int32_t)(p.time_tod - want);
        snprintf(d, sizeof d, "%d TIME frames in 5 s, valid %lu, date %lu, UTC %02lu:%02lu:%02lu.%03lu (sim GPS - %ld ms)",
                 p.times, (unsigned long)p.time_valid, (unsigned long)p.time_date,
                 (unsigned long)(p.time_tod / 3600000u), (unsigned long)(p.time_tod / 60000u % 60u),
                 (unsigned long)(p.time_tod / 1000u % 60u), (unsigned long)(p.time_tod % 1000u), (long)-diff);
        test_report("GA5 GPS time sent to the Pi once a second", p.times >= 4 && p.time_valid == 1u &&
                    p.time_date == 20261020u && diff > -1500 && diff <= 0, d);
    }
}

/* ------------------------------------------- HELM / stations (section B) -- */

/* A command from gateway station `st` (GW_CMD envelope); runs 200 ms and
 * returns the GW_ACK result (0xFF = no answer). */
static uint8_t pi_cmd(sbsim_t *x, pisim_t *p, uint8_t st, bool local, uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint8_t buf[PROTO_MAX_PAYLOAD], fr[PROTO_MAX_FRAME];
    static uint16_t seq = 100u;
    seq++;
    buf[0] = st;
    buf[1] = local ? 1u : 0u;
    buf[2] = (uint8_t)seq;
    buf[3] = (uint8_t)(seq >> 8);
    buf[4] = type;
    if (len) memcpy(&buf[5], pl, len);
    size_t n = proto_encode(fr, PROTO_T_GW_CMD, 0u, buf, (uint8_t)(5u + len));
    int before = p->acks;
    sim_hal_gw_rx_push(fr, (uint32_t)n);
    sbsim_run(x, x->t + 200u, pi_hook, p);
    return (p->acks > before && p->ack_seq == seq && p->ack_station == st) ? p->ack_result : 0xFFu;
}

static void pi_station_left(uint8_t st)
{
    uint8_t pl[3] = { st, 1u, 0u }, fr[PROTO_MAX_FRAME];
    size_t n = proto_encode(fr, PROTO_T_GW_STATION, 0u, pl, 3u);
    sim_hal_gw_rx_push(fr, (uint32_t)n);
}

/* The USB laptop (local station 1): one command, the ACK result. */
static uint8_t usb_cmd(sbsim_t *x, link_t *usb, uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint8_t fr[PROTO_MAX_FRAME], out[1024];
    sim_hal_link_reset();
    size_t n = proto_encode(fr, type, 7u, pl, len);
    sim_hal_link_rx_push(fr, (uint32_t)n);
    usb->last_result = 0xFFu;
    link_step(usb, &x->sb);
    while (sim_hal_link_tx_pop(out, sizeof out) > 0u) {}
    return usb->last_result;
}

static void helm_tests(void)
{
    char d[700];
    const uint8_t TAKE = 0u, RELEASE = 1u, SEA3 = 3u;
    printf("\n=== Section B: HELM (one station commands) ===\n");

    /* GB1 the HELM rules */
    {
        static sbsim_t x;
        static pisim_t p;
        static link_t usb;
        sbsim_init(&x, 911u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        link_init(&usb, &x.sb);
        sbsim_run(&x, 2000u, pi_hook, &p);
        uint8_t r_nohelm = pi_cmd(&x, &p, 2u, false, PROTO_T_SEA_STATE, &SEA3, 1u);        /* remote, no HELM */
        uint8_t r_read = pi_cmd(&x, &p, 2u, false, PROTO_T_JOYMAP_GET, NULL, 0u);         /* read-only: fine */
        uint8_t r_take = pi_cmd(&x, &p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        uint8_t r_cmd = pi_cmd(&x, &p, 2u, false, PROTO_T_SEA_STATE, &SEA3, 1u);
        uint8_t r_take3 = pi_cmd(&x, &p, 3u, false, PROTO_T_HELM, &TAKE, 1u);            /* 2nd remote: no */
        uint8_t r_usb1 = usb_cmd(&x, &usb, PROTO_T_SEA_STATE, &SEA3, 1u);                 /* laptop, no HELM */
        uint8_t r_local = pi_cmd(&x, &p, 4u, true, PROTO_T_HELM, &TAKE, 1u);             /* boat LAN station */
        uint8_t r_old = pi_cmd(&x, &p, 2u, false, PROTO_T_SEA_STATE, &SEA3, 1u);          /* remote lost it */
        uint8_t r_usbt = usb_cmd(&x, &usb, PROTO_T_HELM, &TAKE, 1u);                       /* laptop takes */
        uint8_t r_usb2 = usb_cmd(&x, &usb, PROTO_T_SEA_STATE, &SEA3, 1u);
        uint8_t r_rel = usb_cmd(&x, &usb, PROTO_T_HELM, &RELEASE, 1u);
        uint8_t holder = x.sb.helm;
        snprintf(d, sizeof d, "remote without HELM: %u (21 = not helm), read-only: %u, remote takes: %u, then command: %u; "
                 "2nd remote takes: %u (22 = taken); laptop without HELM: %u; local LAN station takes from remote: %u; "
                 "old remote: %u; laptop takes from it: %u, command %u, releases %u -> holder %u",
                 r_nohelm, r_read, r_take, r_cmd, r_take3, r_usb1, r_local, r_old, r_usbt, r_usb2, r_rel, holder);
        test_report("GB1 only the HELM station commands; a local station always takes HELM from a remote one",
                    r_nohelm == PROTO_R_NOT_HELM && r_read == PROTO_R_OK && r_take == PROTO_R_OK && r_cmd == PROTO_R_OK &&
                    r_take3 == PROTO_R_HELM_TAKEN && r_usb1 == PROTO_R_NOT_HELM && r_local == PROTO_R_OK &&
                    r_old == PROTO_R_NOT_HELM && r_usbt == PROTO_R_OK && r_usb2 == PROTO_R_OK && r_rel == PROTO_R_OK &&
                    holder == SB_HELM_NONE, d);
    }

    /* GB2 remote HELM station lost -> REMOTE LINK LOST, HELM to nobody, no mode change */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 912u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        pi_cmd(&x, &p, 5u, false, PROTO_T_HELM, &TAKE, 1u);
        bool held = x.sb.helm == 5u;
        pi_station_left(5u);
        sbsim_run(&x, x.t + 500u, pi_hook, &p);
        snprintf(d, sizeof d, "remote station 5 held HELM %s; it left: alarm REMOTE LINK LOST %s, HELM -> %u, mode %s",
                 held ? "yes" : "NO", (x.sb.alarm_bits & SB_ALM_REMOTE_LINK) ? "yes" : "NO", x.sb.helm,
                 mode_name(sb_mode(&x.sb)));
        test_report("GB2 remote HELM station lost -> REMOTE LINK LOST, HELM to nobody, no mode change",
                    held && (x.sb.alarm_bits & SB_ALM_REMOTE_LINK) && x.sb.helm == SB_HELM_NONE &&
                    sb_mode(&x.sb) == MODE_HDG_LOCK, d);
    }

    /* GB3 gateway lost while a remote station holds HELM */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 913u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        pi_cmd(&x, &p, 6u, false, PROTO_T_HELM, &TAKE, 1u);
        p.pi_on = false;
        sbsim_run(&x, x.t + 4000u, pi_hook, &p);
        snprintf(d, sizeof d, "GATEWAY LOST %s, REMOTE LINK LOST %s, HELM -> %u, mode %s",
                 (x.sb.alarm_bits & SB_ALM_GATEWAY) ? "yes" : "NO", (x.sb.alarm_bits & SB_ALM_REMOTE_LINK) ? "yes" : "NO",
                 x.sb.helm, mode_name(sb_mode(&x.sb)));
        test_report("GB3 gateway lost with a remote HELM -> both alarms, HELM to nobody, no mode change",
                    (x.sb.alarm_bits & SB_ALM_GATEWAY) && (x.sb.alarm_bits & SB_ALM_REMOTE_LINK) &&
                    x.sb.helm == SB_HELM_NONE && sb_mode(&x.sb) == MODE_HDG_LOCK, d);
    }

    /* GB4 CONFIRM-protected actions from a remote HELM still need the joystick CONFIRM on board */
    {
        static sbsim_t x;
        static pisim_t p;
        prog_wp_t wp[5];
        static program_t prog;
        prog_error_t err;
        uint8_t bad;
        sbsim_init(&x, 914u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        pi_cmd(&x, &p, 7u, false, PROTO_T_HELM, &TAKE, 1u);
        prog5(&x, wp);
        prog_make(&prog, wp, 5, PROG_END_STOP);
        (void)sb_program_set(&x.sb, &prog, &err, &bad);
        uint8_t dir = 0u;
        uint8_t r_code = pi_cmd(&x, &p, 7u, false, PROTO_T_POWER_CODE, &dir, 1u);
        uint8_t raise[3] = { 0u, (uint8_t)x.sb.plim.code, (uint8_t)(x.sb.plim.code >> 8) };
        uint8_t r_raise = pi_cmd(&x, &p, 7u, false, PROTO_T_POWER_RAISE, raise, 3u);
        sbsim_run(&x, x.t + 11000u, pi_hook, &p);                  /* nobody presses CONFIRM */
        float lim1 = x.sb.plim.fwd_pct;
        uint8_t r_auto = pi_cmd(&x, &p, 7u, false, PROTO_T_AUTO_START, NULL, 0u);
        sbsim_run(&x, x.t + 2000u, pi_hook, &p);
        bool no_auto = sb_mode(&x.sb) != MODE_AUTO;
        bool conf = confirm_press(&x);                              /* the person on board */
        sbsim_run(&x, x.t + 500u, pi_hook, &p);
        snprintf(d, sizeof d, "remote HELM: power code %u, raise %u -> without CONFIRM still %.0f %%; AUTO START %u -> "
                 "still not AUTO %s; joystick CONFIRM on board -> %s",
                 r_code, r_raise, lim1, r_auto, no_auto ? "yes" : "NO", sb_mode(&x.sb) == MODE_AUTO && conf ? "AUTO" : "NO");
        test_report("GB4 remote HELM can't raise power or start AUTO alone: the joystick CONFIRM on board is needed",
                    r_code == PROTO_R_OK && r_raise == PROTO_R_OK && lim1 == 30.0f && r_auto == PROTO_R_OK && no_auto &&
                    conf && sb_mode(&x.sb) == MODE_AUTO, d);
    }
}

/* --------------------------------------- REMOTE MANUAL driving (section D) -- */

/* The remote joystick through the "gateway": one GW_RJOY per control step
 * while rj_on, with the age / latency the gateway would measure. */
typedef struct {
    pisim_t  p;
    bool     rj_on;
    uint8_t  st;
    uint16_t seq;
    float    x, y, lever;
    bool     has_lever;
    uint16_t age_ms, lat_ms;
    float    max_thr, max_rate, last_cmd;
    bool     have_cmd;
} rjsim_t;

static void rj_send(rjsim_t *r, uint8_t flags)
{
    uint8_t pl[15], fr[PROTO_MAX_FRAME];
    int16_t xi = (int16_t)lrintf(r->x * 1000.0f), yi = (int16_t)lrintf(r->y * 1000.0f);
    uint16_t li = (uint16_t)lrintf(r->lever * 1000.0f);
    r->seq++;
    pl[0] = r->st;
    pl[1] = (uint8_t)r->seq; pl[2] = (uint8_t)(r->seq >> 8);
    pl[3] = (uint8_t)r->age_ms; pl[4] = (uint8_t)(r->age_ms >> 8);
    pl[5] = (uint8_t)r->lat_ms; pl[6] = (uint8_t)(r->lat_ms >> 8);
    pl[7] = (uint8_t)xi; pl[8] = (uint8_t)((uint16_t)xi >> 8);
    pl[9] = (uint8_t)yi; pl[10] = (uint8_t)((uint16_t)yi >> 8);
    pl[11] = (uint8_t)li; pl[12] = (uint8_t)(li >> 8);
    pl[13] = 0u;
    pl[14] = (uint8_t)((r->has_lever ? 1u : 0u) | flags);
    size_t n = proto_encode(fr, PROTO_T_GW_RJOY, 0u, pl, sizeof pl);
    sim_hal_gw_rx_push(fr, (uint32_t)n);
}

static void rj_hook(sbsim_t *x, void *ctx)
{
    rjsim_t *r = (rjsim_t *)ctx;
    if (r->rj_on) rj_send(r, 0u);
    pi_hook(x, &r->p);
    if (sb_mode(&x->sb) == MODE_REMOTE) {
        float t = sim_hal_throttle();
        if (t > r->max_thr) r->max_thr = t;
        float c = x->sb.remote.steer_cmd;
        if (r->have_cmd) {
            float rate = fabsf(c - r->last_cmd) / (AP_PERIOD_MS * 0.001f);
            if (rate > r->max_rate) r->max_rate = rate;
        }
        r->last_cmd = c;
        r->have_cmd = true;
    } else {
        r->have_cmd = false;
    }
}

static void rjsim_init(rjsim_t *r, sbsim_t *x, uint32_t seed)
{
    memset(r, 0, sizeof *r);
    sbsim_init(x, seed, 90.0f, 0.0f, NULL);
    pisim_init(&r->p, x, 921600u);
    r->st = 2u;
    r->lever = 0.5f;
    r->has_lever = true;
    r->age_ms = 60u;
    r->lat_ms = 40u;
    sbsim_run(x, 2000u, rj_hook, r);
}

/* HELM for station 2, ask for remote driving, CONFIRM on board -> REMOTE. */
static bool rj_enable(sbsim_t *x, rjsim_t *r)
{
    const uint8_t TAKE = 0u, ASK = 0u;
    pi_cmd(x, &r->p, r->st, false, PROTO_T_HELM, &TAKE, 1u);
    if (pi_cmd(x, &r->p, r->st, false, PROTO_T_REMOTE, &ASK, 1u) != PROTO_R_OK) return false;
    r->rj_on = true;
    confirm_press(x);
    sbsim_run(x, x->t + 300u, rj_hook, r);
    return sb_mode(&x->sb) == MODE_REMOTE;
}

static void rj_lever(sbsim_t *x, rjsim_t *r, float to, float rate)
{
    while (fabsf(r->lever - to) > 1e-4f) {
        float step = rate * AP_PERIOD_MS * 0.001f;
        r->lever = (to > r->lever) ? fminf(to, r->lever + step) : fmaxf(to, r->lever - step);
        sbsim_run(x, x->t + AP_PERIOD_MS, rj_hook, r);
    }
}

static void remote_tests(void)
{
    char d[800];
    const uint8_t TAKE = 0u, ASK = 0u;
    printf("\n=== Section D: REMOTE MANUAL (shore joystick) ===\n");

    /* RD1 enabling needs the HELM request AND the joystick CONFIRM on board */
    {
        static sbsim_t x;
        static rjsim_t r;
        rjsim_init(&r, &x, 921u);
        uint8_t r_usb = 0xFFu;
        {
            static link_t usb;
            link_init(&usb, &x.sb);
            r_usb = usb_cmd(&x, &usb, PROTO_T_REMOTE, &ASK, 1u);              /* the USB laptop: no */
        }
        uint8_t r_nohelm = pi_cmd(&x, &r.p, 3u, false, PROTO_T_REMOTE, &ASK, 1u);
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        uint8_t r_ask = pi_cmd(&x, &r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        r.rj_on = true;
        sbsim_run(&x, x.t + 11000u, rj_hook, &r);                            /* nobody confirms */
        bool still = sb_mode(&x.sb) == MODE_STANDBY;
        uint8_t r_ask2 = pi_cmd(&x, &r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        bool conf = confirm_press(&x);
        sbsim_run(&x, x.t + 300u, rj_hook, &r);
        bool on = sb_mode(&x.sb) == MODE_REMOTE;
        /* not from MANUAL: the onboard joystick has it */
        static sbsim_t y;
        static rjsim_t q;
        rjsim_init(&q, &y, 922u);
        joy_set(0.0f, 0.5f, false, false);
        sbsim_run(&y, y.t + 1000u, rj_hook, &q);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&y, y.t + 500u, rj_hook, &q);
        pi_cmd(&y, &q.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        uint8_t r_man = pi_cmd(&y, &q.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        snprintf(d, sizeof d, "USB laptop asks: %u (5 = not allowed); station without HELM: %u (21); HELM station asks: %u, "
                 "no CONFIRM for 11 s -> still STANDBY %s; asks again %u + joystick CONFIRM on board %s -> %s; "
                 "asked in MANUAL (onboard joystick driving): %u (4 = joystick in control)",
                 r_usb, r_nohelm, r_ask, still ? "yes" : "NO", r_ask2, conf ? "accepted" : "NO",
                 mode_name(sb_mode(&x.sb)), r_man);
        test_report("RD1 remote driving starts only with the HELM request AND the joystick CONFIRM on board",
                    r_usb == PROTO_R_NOT_ALLOWED && r_nohelm == PROTO_R_NOT_HELM && r_ask == PROTO_R_OK && still &&
                    r_ask2 == PROTO_R_OK && conf && on && r_man == PROTO_R_JOYSTICK, d);
    }

    /* RD2 cap 20 % / 5 % (never above the power limit), steering rate limit, pickup from STANDBY */
    {
        static sbsim_t x;
        static rjsim_t r;
        rjsim_init(&r, &x, 923u);
        power_full(&x);
        r.lever = 0.9f;                              /* lever left forward: nothing until neutral */
        bool on = rj_enable(&x, &r);
        sbsim_run(&x, x.t + 3000u, rj_hook, &r);
        float before = r.max_thr;
        rj_lever(&x, &r, 0.5f, 0.5f);
        rj_lever(&x, &r, 1.0f, 0.5f);               /* full forward */
        sbsim_run(&x, x.t + 15000u, rj_hook, &r);
        float full = sim_hal_throttle();
        r.x = 1.0f;                                  /* full steering at once */
        sbsim_run(&x, x.t + 4000u, rj_hook, &r);
        rj_lever(&x, &r, 0.5f, 0.5f);
        rj_lever(&x, &r, 0.0f, 0.5f);               /* full reverse */
        sbsim_run(&x, x.t + 10000u, rj_hook, &r);
        float rev = x.w.max_thr_rev;
        snprintf(d, sizeof d, "REMOTE %s; lever left at 90 %%: throttle max %.1f %% until it passed neutral; full lever: "
                 "%.1f %% (cap 20, power limit %.0f %%); full reverse: max %.1f %% (cap 5); steering command rate max "
                 "%.1f deg/s (limit 20); highest throttle %.1f %%",
                 on ? "yes" : "NO", before, full, x.sb.plim.fwd_pct, rev, r.max_rate, r.max_thr);
        test_report("RD2 remote cap 20 % fwd / 5 % rev never exceeded, steering rate-limited, lever pickup",
                    on && before <= 1.0f && full >= 19.0f && full <= 20.01f && r.max_thr <= 20.01f &&
                    rev <= 5.01f && r.max_rate <= 20.5f && x.sb.plim.fwd_pct == 100.0f, d);
    }

    /* RD3 the onboard joystick takes over at once */
    {
        static sbsim_t x;
        static rjsim_t r;
        rjsim_init(&r, &x, 924u);
        rj_enable(&x, &r);
        rj_lever(&x, &r, 0.8f, 0.5f);
        sbsim_run(&x, x.t + 5000u, rj_hook, &r);
        uint32_t t0 = x.t;
        joy_set(0.6f, 0.0f, false, false);          /* someone on board moves the stick */
        while (sb_mode(&x.sb) == MODE_REMOTE && x.t < t0 + 1000u) sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r);
        uint32_t dt = x.t - t0;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 2000u, rj_hook, &r);
        bool stays = sb_mode(&x.sb) == MODE_MANUAL;
        /* TAKE too */
        static sbsim_t y;
        static rjsim_t q;
        rjsim_init(&q, &y, 925u);
        rj_enable(&y, &q);
        joy_set(0.0f, 0.0f, true, false);
        sbsim_run(&y, y.t + 100u, rj_hook, &q);
        joy_set(0.0f, 0.0f, false, false);
        bool take = sb_mode(&y.sb) == MODE_MANUAL;
        snprintf(d, sizeof d, "stick moved on board: MANUAL after %lu ms; remote packets afterwards ignored, stays MANUAL %s; "
                 "TAKE on board: %s", (unsigned long)dt, stays ? "yes" : "NO", take ? "MANUAL" : "NO");
        test_report("RD3 any onboard joystick movement or TAKE takes control from the remote driver at once",
                    dt <= 200u && stays && take, d);
    }

    /* RD3b R1: STANDBY while driving remotely - the remote stick is ignored at
     *      once, the gyro holds the heading until idle + neutral, then steering off */
    {
        static sbsim_t x;
        static rjsim_t r;
        rjsim_init(&r, &x, 926u);
        bool on = rj_enable(&x, &r);
        rj_lever(&x, &r, 1.0f, 0.5f);
        sbsim_run(&x, x.t + 15000u, rj_hook, &r);
        float thr0 = sim_hal_throttle();
        events_t ev;
        memset(&ev, 0, sizeof ev);
        ev_reset(&ev, x.t, thr0, x.sb.cfg.gear.pwm_n_us);
        sim_hal_press(HAL_BTN_STANDBY);
        r.x = 1.0f;                                  /* the remote driver steers hard right after STANDBY */
        bool gyro_whole_stop = true, remote_again = false;
        float h0 = x.w.heading_deg, hmax = 0.0f;
        for (int i = 0; i < 200; i++) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r);
            ev_watch(&x, &ev);
            if (ev.t_steer_off < 0.0f && i > 0 && x.sb.ap.mode != AP_MODE_GYRO_HOLD) gyro_whole_stop = false;
            if (ev.t_steer_off < 0.0f) hmax = fmaxf(hmax, fabsf(hdg_err(x.w.heading_deg, h0)));
            if (sb_mode(&x.sb) == MODE_REMOTE) remote_again = true;
        }
        snprintf(d, sizeof d, "REMOTE %s, throttle %.1f %% -> idle %.2f s, neutral %.2f s, steering off %.2f s; "
                 "gyro hold the whole stop %s; heading moved %.1f deg although the remote stick was full right; "
                 "back to REMOTE %s; motor after off %d",
                 on ? "yes" : "NO", thr0, ev.t_idle, ev.t_neutral, ev.t_steer_off, gyro_whole_stop ? "yes" : "NO",
                 hmax, remote_again ? "YES" : "no", ev.steer_after_off);
        test_report("RD3b R1 STANDBY in REMOTE: remote stick ignored, gyro hold until idle + neutral, then off",
                    on && thr0 > 15.0f && ev.t_idle >= 0.0f && ev.t_idle <= 0.4f && ev.t_neutral > ev.t_idle &&
                    ev.t_steer_off >= ev.t_neutral && gyro_whole_stop && hmax < 10.0f && !remote_again &&
                    ev.steer_after_off == 0 && sb_mode(&x.sb) == MODE_STANDBY, d);
    }

    /* RD4 packets stop (gap > 300 ms): idle + neutral, gyro hold, alarm; STANDBY after 5 s; no resume */
    {
        static sbsim_t x;
        static rjsim_t r;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        rjsim_init(&r, &x, 926u);
        rj_enable(&x, &r);
        rj_lever(&x, &r, 1.0f, 0.5f);
        sbsim_run(&x, x.t + 10000u, rj_hook, &r);
        float thr0 = sim_hal_throttle();
        uint32_t t0 = x.t;
        r.rj_on = false;                            /* the link dies */
        ev_reset(&ev, x.t, thr0, x.sb.cfg.gear.pwm_n_us);
        uint32_t t_lost = 0u;
        bool resumed = false;
        while (x.t < t0 + 7000u) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r);
            ev_watch(&x, &ev);
            if (!t_lost && x.sb.remote.state == SB_REMOTE_LOST) t_lost = x.t - t0;
            if (x.t > t0 + 2000u && x.t < t0 + 2100u) r.rj_on = true;     /* data comes back after 2 s */
            if (x.t > t0 + 2500u && sb_mode(&x.sb) == MODE_REMOTE && x.sb.remote.state == SB_REMOTE_DRIVING) resumed = true;
        }
        snprintf(d, sizeof d, "throttle %.1f %%; link dead: LOST after %lu ms (alarm %s, steering %s), idle %.2f s, neutral "
                 "%.2f s; data back after 2 s: driving again %s; STANDBY %s (why: %s)",
                 thr0, (unsigned long)t_lost, (x.sb.alarm_bits & SB_ALM_REMOTE_DRIVE) ? "yes" : "NO",
                 x.sb.remote.losses ? "gyro hold" : "?", ev.t_idle, ev.t_neutral, resumed ? "YES" : "no",
                 sb_mode(&x.sb) == MODE_STANDBY ? "after 5 s" : "NO", mode_reason_name(auto_end_why(&x)));
        test_report("RD4 remote data gap > 300 ms -> idle + neutral + gyro hold + alarm, STANDBY after 5 s, never resumes",
                    t_lost >= 300u && t_lost <= 400u && (x.sb.alarm_bits & SB_ALM_REMOTE_DRIVE) && ev.t_idle >= 0.0f &&
                    ev.t_idle <= 1.5f && ev.t_neutral > 0.0f && !resumed && sb_mode(&x.sb) == MODE_STANDBY &&
                    auto_end_why(&x) == WHY_REMOTE_LOST, d);
    }

    /* RD5 data too old / latency too high -> the same */
    {
        int ok = 0;
        char t[300] = "";
        for (int k = 0; k < 2; k++) {
            static sbsim_t x;
            static rjsim_t r;
            rjsim_init(&r, &x, 927u + (uint32_t)k);
            rj_enable(&x, &r);
            sbsim_run(&x, x.t + 2000u, rj_hook, &r);
            if (k == 0) r.age_ms = 350u; else r.lat_ms = 450u;
            sbsim_run(&x, x.t + 300u, rj_hook, &r);
            bool lost = x.sb.remote.state == SB_REMOTE_LOST && x.sb.remote.lost_why == (k == 0 ? 2u : 3u);
            ok += lost;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s -> %s", k ? "; " : "",
                     k ? "latency 450 ms (limit 400)" : "data 350 ms old (limit 300)", lost ? "LOST" : "still driving");
        }
        test_report("RD5 remote data too old or link latency too high -> link lost", ok == 2, t);
    }

    /* RD6 taking over from AUTO: the throttle is held until the remote lever picks it up */
    {
        static sbsim_t x;
        static rjsim_t r;
        rjsim_init(&r, &x, 929u);
        auto_to(&x, 2000.0f, 90.0f, 4.0f);
        sbsim_run(&x, x.t + 30000u, rj_hook, &r);
        float thr_auto = sim_hal_throttle();
        r.lever = 0.62f;                            /* forward, but far from the pickup point */
        bool on = rj_enable(&x, &r);
        r.max_thr = 0.0f;
        sbsim_run(&x, x.t + 3000u, rj_hook, &r);
        bool pend = x.sb.remote.lev.pickup;
        float held = r.max_thr;
        rj_lever(&x, &r, x.sb.remote.lev.target > 0.0f ? x.sb.remote.lev.target : 0.5f, 0.05f);
        bool picked = !x.sb.remote.lev.pickup;
        snprintf(d, sizeof d, "AUTO at %.1f %%, remote lever at 62 %%: REMOTE %s, pickup pending %s, throttle max %.1f %% "
                 "(cap %.0f %%), lever to the pickup point: picked up %s",
                 thr_auto, on ? "yes" : "NO", pend ? "yes" : "NO", held, fminf(20.0f, x.sb.plim.fwd_pct), picked ? "yes" : "NO");
        test_report("RD6 throttle pickup and neutral zone apply to the remote lever (no drop, no jump)",
                    on && pend && held <= fmaxf(thr_auto, 20.0f) + 0.5f && held >= thr_auto - 1.0f && picked, d);
    }

    /* RD7 release / STANDBY button / a station on board takes HELM -> STANDBY */
    {
        int ok = 0;
        char t[400] = "";
        for (int k = 0; k < 3; k++) {
            static sbsim_t x;
            static rjsim_t r;
            rjsim_init(&r, &x, 930u + (uint32_t)k);
            rj_enable(&x, &r);
            if (k == 0) { rj_send(&r, 2u); sbsim_run(&x, x.t + 200u, rj_hook, &r); }
            else if (k == 1) press(&x, HAL_BTN_STANDBY);
            else pi_cmd(&x, &r.p, 9u, true, PROTO_T_HELM, &TAKE, 1u);
            sbsim_run(&x, x.t + 200u, rj_hook, &r);
            bool sb = sb_mode(&x.sb) == MODE_STANDBY;
            ok += sb;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s -> %s", k ? "; " : "",
                     k == 0 ? "remote driver releases" : k == 1 ? "STANDBY button" : "station on board takes HELM",
                     mode_name(sb_mode(&x.sb)));
        }
        test_report("RD7 remote driving ends: released, STANDBY button, HELM taken on board -> STANDBY", ok == 3, t);
    }
}

/* ------------------------------------------ video SAVE requests (section F) -- */

static void save_tests(void)
{
    char d[700];
    printf("\n=== Section F: video SAVE requests to the gateway ===\n");

    /* GF1 joystick takeover, compass loss (alarm / fault / STANDBY by fault)
     * -> numbered SAVE requests in TELEM2; the Pi sees every one; the mode
     * is never touched by a SAVE */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 951u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        uint16_t s0 = x.sb.save.seq;
        hdg_lock(&x);
        sbsim_run(&x, x.t + 3000u, pi_hook, &p);
        joy_set(0.6f, 0.0f, false, false);                   /* the hand takes over */
        sbsim_run(&x, x.t + 1000u, pi_hook, &p);
        bool take = (p.save_mask & (1u << SB_SAVE_TAKEOVER)) != 0u && p.save_last_arg_take == MODE_HDG_LOCK;
        bool manual = sb_mode(&x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 1000u, pi_hook, &p);
        hdg_lock(&x);
        sbsim_run(&x, x.t + 2000u, pi_hook, &p);
        sim_hal_fail_heading(true);                          /* compass dies in HDG LOCK */
        sbsim_run(&x, x.t + 20000u, pi_hook, &p);
        bool alarm = (p.save_mask & (1u << SB_SAVE_ALARM)) != 0u;
        bool fault = (p.save_mask & (1u << SB_SAVE_FAULT)) != 0u;
        bool sbf = (p.save_mask & (1u << SB_SAVE_STANDBY_FAULT)) != 0u;
        uint16_t n = (uint16_t)(x.sb.save.seq - s0);
        snprintf(d, sizeof d, "takeover from HDG LOCK -> SAVE \"takeover\" %s (mode %s); compass lost -> alarm %s, fault %s, "
                 "STANDBY by fault %s (mode now %s); %u requests on the boat, the Pi saw seq %u (%u changes seen)",
                 take ? "yes" : "NO", manual ? "MANUAL" : "?", alarm ? "yes" : "NO", fault ? "yes" : "NO",
                 sbf ? "yes" : "NO", mode_name(sb_mode(&x.sb)), n, p.save_seq, p.save_changes);
        test_report("GF1 SAVE requests: joystick takeover, alarm, fault, STANDBY by fault -> Pi sees every request number",
                    take && manual && alarm && fault && sbf && sb_mode(&x.sb) == MODE_STANDBY && n >= 4u &&
                    p.save_seq == x.sb.save.seq, d);
        sim_hal_fail_heading(false);
    }

    /* GF2 the SAVE button on the USB joystick: a request, nothing else;
     * arriving at a waypoint is no incident (no request) */
    {
        static sbsim_t x;
        static pisim_t p;
        usb_init(&x, 952u, 0.0f);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, x.t + 1000u, pi_hook, &p);
        uint16_t s0 = x.sb.save.seq;
        sb_mode_t m0 = sb_mode(&x.sb);
        usb_press(&x, JSF_CODE_BTN(0u, 2u));
        sbsim_run(&x, x.t + 500u, pi_hook, &p);
        bool btn = x.sb.save.seq == (uint16_t)(s0 + 1u) && x.sb.save.trig == SB_SAVE_BUTTON &&
                   p.save_seq == x.sb.save.seq && sb_mode(&x.sb) == m0;
        uint16_t s1 = x.sb.save.seq;
        x.sb.alarm_bits |= SB_ALM_ARRIVED;
        sbsim_run(&x, x.t + 500u, pi_hook, &p);
        bool arr = x.sb.save.seq == s1;
        snprintf(d, sizeof d, "SAVE button (stick button 3) -> request %u trigger %u, the Pi saw %u, mode unchanged %s; "
                 "ARRIVED alarm -> no request %s", x.sb.save.seq, s1 ? x.sb.save.trig : 0u, p.save_seq,
                 sb_mode(&x.sb) == m0 ? "yes" : "NO", arr ? "yes" : "NO");
        test_report("GF2 USB joystick SAVE button -> one SAVE request, no mode change; ARRIVED is no SAVE trigger",
                    btn && arr, d);
    }
}

/* ------------------------------------------------------ slamming (section H) -- */

/* Watches the rule "slamming only ever reduces": the throttle never rises
 * while it is above the slamming cap (unless the onboard driver overrides). */
typedef struct {
    float last_out;
    int   rise_over_cap;        /* steps where the throttle went UP above the cap */
    float max_over_plim;        /* never above the power limit */
    float min_cap;
    int   steps_down, steps_up;
    float last_cap;
    float max_cap_jump;         /* biggest cap increase in one step */
    uint32_t last_rough_t;      /* last time slamming was not calm */
    uint32_t first_up_gap;      /* calm time before the first raise */
    bool  init;
} slamwatch_t;

static void slam_hook(sbsim_t *x, void *ctx)
{
    slamwatch_t *w = (slamwatch_t *)ctx;
    const sb_t *s = &x->sb;
    float out = s->servo.out_pct, cap = slam_cap(&s->slam);
    if (!w->init) { w->init = true; w->last_out = out; w->last_cap = cap; w->min_cap = cap; }
    if (out > w->last_out + 0.01f && out > cap + 0.5f && !s->slam.override) w->rise_over_cap++;
    if (gear_is(&s->gear, GEAR_F) && out - s->plim.fwd_pct > w->max_over_plim) w->max_over_plim = out - s->plim.fwd_pct;
    if (cap < w->last_cap - 0.01f) w->steps_down++;
    if (s->slam.state != SLAM_CALM) w->last_rough_t = x->t;
    if (cap > w->last_cap + 0.01f) {
        if (!w->steps_up) w->first_up_gap = x->t - w->last_rough_t;
        w->steps_up++;
        if (cap - w->last_cap > w->max_cap_jump) w->max_cap_jump = cap - w->last_cap;
    }
    if (cap < w->min_cap) w->min_cap = cap;
    w->last_cap = cap;
    w->last_out = out;
}

/* MANUAL, full power limit, stick full ahead (throttle 100 %). */
static void slam_go(sbsim_t *x, uint32_t seed, float sea)
{
    sbsim_init(x, seed, 90.0f, 0.0f, NULL);
    x->w.sea_state = sea;
    sbsim_run(x, 2000u, NULL, NULL);
    joy_set(0.0f, 0.3f, false, false);                     /* into MANUAL (the takeover lowers the limit) */
    sbsim_run(x, x->t + 1000u, NULL, NULL);
    joy_set(0.0f, 0.0f, false, false);
    sbsim_run(x, x->t + 500u, NULL, NULL);
    power_full(x);                                          /* then raise it, stick centred */
    joy_set(0.0f, 1.0f, false, false);
}

/* slamming fields read back from a log (SL8) */
typedef struct {
    char  fields[2048];
    bool  have, yellow;
    float peak, level, cap;
    unsigned rate;
} slamlog_t;

static void slamlog_cb(void *ctx, uint8_t type, uint32_t t, const uint8_t *p, uint16_t len)
{
    (void)t; (void)len;
    slamlog_t *s = (slamlog_t *)ctx;
    double pk, lv, rt, cp, st;
    if (type != LOGR_DATA) return;
    if (!tlg_field(s->fields, p, "slam_peak", &pk) || !tlg_field(s->fields, p, "slam_level", &lv) ||
        !tlg_field(s->fields, p, "slam_rate", &rt) || !tlg_field(s->fields, p, "slam_cap", &cp) ||
        !tlg_field(s->fields, p, "slam_state", &st)) return;
    if (!s->have) s->cap = 100.0f;
    s->have = true;
    if (pk > s->peak) s->peak = (float)pk;
    if (lv > s->level) s->level = (float)lv;
    if (rt > s->rate) s->rate = (unsigned)rt;
    if (cp < s->cap) s->cap = (float)cp;
    if (((int)st & 3) == SLAM_YELLOW) s->yellow = true;
}

static void slam_tests(void)
{
    char d[800];
    printf("\n=== Section H: slamming (BNO085 vertical acceleration) ===\n");

    /* SL1 measurement: impacts counted like the boat made them, peak, rate,
     * level; smooth water: nothing; every impact in the log (EV_SLAM) */
    {
        static sbsim_t x;
        slam_go(&x, 961u, 3.0f);
        x.cfg.slam.limit_g = 9.0f;                         /* measure only: no reduction */
        x.sb.slam.cfg.limit_g = 9.0f;
        x.sb.slam.cfg.yellow_frac = 0.9f;
        sbsim_run(&x, 30000u, NULL, NULL);
        uint32_t w0 = x.w.slam_impacts, s0 = x.sb.slam.impacts;
        float kn = x.w.speed_kn;
        sbsim_run(&x, 90000u, NULL, NULL);
        uint32_t made = x.w.slam_impacts - w0, seen = x.sb.slam.impacts - s0;
        float level = x.sb.slam.level;
        uint16_t rate = x.sb.slam.rate_pm;
        static sbsim_t y;
        slam_go(&y, 962u, 0.0f);
        sbsim_run(&y, 60000u, NULL, NULL);
        snprintf(d, sizeof d, "sea 3 at %.1f kn: %lu impacts made by the boat model in 60 s, %lu counted; level %.2f g, "
                 "%u per min (impacts of 1 g or more); smooth water at %.1f kn: %lu counted, peak %.2f g",
                 kn, (unsigned long)made, (unsigned long)seen, level, rate, y.w.speed_kn,
                 (unsigned long)y.sb.slam.impacts, y.sb.slam.peak);
        test_report("SL1 slamming measured: impacts, rate, level (sea 3, ~20 kn); smooth water: none",
                    made > 20u && seen + 3u >= made && seen <= made + 3u && level > 1.0f && rate > 15u &&
                    y.sb.slam.impacts == 0u, d);
    }

    /* SL2 an impact over the limit (red), default STEP: red alarm, cap one
     * step down at once, SAVE request "slamming" - the power limit STAYS, the
     * throttle comes down at its normal rate (no sudden drop to 30 %) */
    {
        static sbsim_t x;
        static slamwatch_t wt;
        memset(&wt, 0, sizeof wt);
        slam_go(&x, 963u, 3.0f);
        sbsim_run(&x, 20000u, slam_hook, &wt);
        float plim0 = x.sb.plim.fwd_pct;
        uint16_t save0 = x.sb.save.seq;
        x.w.slam_extra_g = 2.0f;
        uint32_t r0 = x.sb.slam.reds;
        float out0 = x.sb.servo.out_pct;
        while (x.sb.slam.reds == r0 && x.t < 60000u) {
            out0 = x.sb.servo.out_pct;
            sbsim_run(&x, x.t + 50u, slam_hook, &wt);
        }
        bool red = x.sb.slam.reds > r0;
        x.w.slam_extra_g = 0.0f;
        float drop_max = 0.0f, prev = x.sb.servo.out_pct;      /* fastest fall, %/s */
        uint32_t t_red = x.t, prev_t = x.t;
        while (x.t < t_red + 1000u) {
            sbsim_run(&x, x.t + 200u, slam_hook, &wt);
            drop_max = fmaxf(drop_max, (prev - x.sb.servo.out_pct) * 1000.0f / (float)(x.t - prev_t));
            prev = x.sb.servo.out_pct;
            prev_t = x.t;
        }
        bool alarm = (x.sb.alarm_bits & SB_ALM_SLAM) != 0u;
        float plim1 = x.sb.plim.fwd_pct, cap = slam_cap(&x.sb.slam), out1 = x.sb.servo.out_pct;
        bool save = x.sb.save.seq != save0 && (x.sb.save.kinds & (1u << SB_SAVE_SLAM));
        snprintf(d, sizeof d, "STEP (default): red impact %s; alarm SLAMMING %s; power limit %.0f -> %.0f %% (stays); "
                 "cap %.0f %%; throttle %.0f -> %.0f %% after 1 s, falling at most %.0f %%/s (normal rate %.0f); "
                 "SAVE %s; throttle rose above the cap %d times",
                 red ? "yes" : "NO", alarm ? "yes" : "NO", plim0, plim1, cap, out0, out1, drop_max,
                 x.cfg.throttle.rate_down_pct_s,
                 save ? "yes" : "NO", wt.rise_over_cap);
        test_report("SL2 red impact, STEP (default): red alarm + cap step + SAVE, power limit kept, no sudden drop",
                    red && alarm && plim0 >= 99.0f && plim1 >= 99.0f && cap < 100.0f && save && out1 <= cap + 0.5f &&
                    out1 > 31.0f && drop_max <= x.cfg.throttle.rate_down_pct_s + 1.0f &&
                    wt.rise_over_cap == 0, d);
    }

    /* SL2b the old behaviour, selectable: LIMIT_RESET -> power limit down at once */
    {
        static sbsim_t x;
        static slamwatch_t wt;
        memset(&wt, 0, sizeof wt);
        slam_go(&x, 963u, 3.0f);
        x.sb.slam.cfg.red_reset = 1.0f;
        sbsim_run(&x, 20000u, slam_hook, &wt);
        float plim0 = x.sb.plim.fwd_pct;
        uint16_t save0 = x.sb.save.seq;
        x.w.slam_extra_g = 2.0f;                            /* the next impacts are hard */
        uint32_t r0 = x.sb.slam.reds;
        while (x.sb.slam.reds == r0 && x.t < 60000u) sbsim_run(&x, x.t + 50u, slam_hook, &wt);
        bool red = x.sb.slam.reds > r0;
        uint32_t t_red = x.t;
        sbsim_run(&x, x.t + 100u, slam_hook, &wt);
        bool alarm = (x.sb.alarm_bits & SB_ALM_SLAM) != 0u;
        float plim1 = x.sb.plim.fwd_pct, cap = slam_cap(&x.sb.slam);
        bool save = x.sb.save.seq != save0 && (x.sb.save.kinds & (1u << SB_SAVE_SLAM));
        sbsim_run(&x, t_red + 8000u, slam_hook, &wt);
        snprintf(d, sizeof d, "red impact %s (%.2f g); alarm SLAMMING %s; power limit %.0f -> %.0f %%; cap %.0f %%; "
                 "SAVE request \"slamming\" %s; throttle 8 s later %.0f %% (never rose above the cap: %d), mode %s",
                 red ? "yes" : "NO", x.w.slam_last_g, alarm ? "yes" : "NO", plim0, plim1, cap, save ? "yes" : "NO",
                 x.sb.servo.out_pct, wt.rise_over_cap, mode_name(sb_mode(&x.sb)));
        test_report("SL2b red impact, LIMIT_RESET selected: SLAMMING alarm, power limit down at once, cap step, SAVE",
                    red && alarm && plim0 >= 99.0f && plim1 <= 30.5f && cap < 100.0f && save &&
                    x.sb.servo.out_pct <= 31.0f && wt.rise_over_cap == 0 && sb_mode(&x.sb) == MODE_MANUAL, d);
    }

    /* SL3 yellow (high level, no single hard impact): warning only, the cap
     * steps down every 5 s, never below 30 %; the power limit stays */
    {
        static sbsim_t x;
        static slamwatch_t wt;
        memset(&wt, 0, sizeof wt);
        slam_go(&x, 964u, 3.0f);
        x.sb.slam.cfg.limit_g = 9.0f;                       /* yellow at 0.12 x 9 = 1.1 g level, red never */
        x.sb.slam.cfg.yellow_frac = 0.12f;
        sbsim_run(&x, 70000u, slam_hook, &wt);
        bool yellow = x.sb.slam.steps > 0u;
        snprintf(d, sizeof d, "state %s, level %.2f g; cap steps down %d (lowest %.0f %%, floor 30); power limit %.0f %% "
                 "(unchanged), alarm bits 0x%lx; throttle %.0f %% (rose above the cap: %d); nervous %s",
                 x.sb.slam.state == SLAM_YELLOW ? "YELLOW" : x.sb.slam.state == SLAM_RED ? "RED" : "calm",
                 x.sb.slam.level, wt.steps_down, wt.min_cap, x.sb.plim.fwd_pct, (unsigned long)x.sb.alarm_bits,
                 x.sb.servo.out_pct, wt.rise_over_cap, x.sb.nerv.nervous ? "yes" : "no");
        test_report("SL3 yellow: warning, stepwise throttle reduction (5 s steps, floor 30 %), power limit kept",
                    yellow && wt.steps_down >= 3 && wt.min_cap >= 29.9f && x.sb.plim.fwd_pct >= 99.0f &&
                    !(x.sb.alarm_bits & SB_ALM_SLAM) && wt.rise_over_cap == 0 &&
                    x.sb.servo.out_pct <= slam_cap(&x.sb.slam) + 0.5f, d);

        /* SL4 calm water: after 30 s calm the cap comes back slowly (+5 % / 10 s) */
        x.w.sea_state = 0.0f;
        wt.steps_up = 0; wt.max_cap_jump = 0.0f; wt.rise_over_cap = 0;
        float cap0 = slam_cap(&x.sb.slam);
        uint32_t t0 = x.t, t_first = 0u;
        while (x.t < t0 + 400000u && slam_cap(&x.sb.slam) < 100.0f) {
            sbsim_run(&x, x.t + 500u, slam_hook, &wt);
            if (!t_first && wt.steps_up) t_first = x.t;
        }
        float rate_max = 0.0f;
        snprintf(d, sizeof d, "cap %.0f %% -> first raise after %.0f s of calm water, back to 100 %% %.0f s after the sea "
                 "went flat; biggest single raise %.1f %%; throttle rose above the cap: %d %s",
                 cap0, wt.first_up_gap / 1000.0, (x.t - t0) / 1000.0,
                 wt.max_cap_jump, wt.rise_over_cap, rate_max > 0.0f ? "" : "");
        test_report("SL4 recovery: only after a calm time, slowly (+5 % every 10 s), throttle at its normal rate",
                    t_first && wt.first_up_gap >= 30000u && slam_cap(&x.sb.slam) >= 100.0f && wt.max_cap_jump <= 5.01f &&
                    wt.rise_over_cap == 0, d);
    }

    /* SL5 every driving mode: HDG LOCK and AUTO are capped the same way */
    {
        int ok = 0;
        char t[500] = "";
        for (int k = 0; k < 2; k++) {
            static sbsim_t x;
            static slamwatch_t wt;
            memset(&wt, 0, sizeof wt);
            sbsim_init(&x, 965u + (uint32_t)k, 90.0f, 0.0f, NULL);
            x.w.sea_state = 3.0f;
            sbsim_run(&x, 2000u, NULL, NULL);
            if (k == 0) {
                hdg_lock(&x);
                joy_set(0.0f, 1.0f, false, false);            /* throttle on the stick in HDG LOCK */
            } else {
                auto_to(&x, 20000.0f, 90.0f, 25.0f);
            }
            x.sb.slam.cfg.limit_g = 9.0f;
            x.sb.slam.cfg.yellow_frac = 0.12f;
            sbsim_run(&x, 70000u, slam_hook, &wt);
            bool g = sb_mode(&x.sb) == (k == 0 ? MODE_HDG_LOCK : MODE_AUTO) && wt.steps_down >= 2 &&
                     wt.rise_over_cap == 0 && x.sb.servo.out_pct <= slam_cap(&x.sb.slam) + 0.5f;
            ok += g;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s: cap %.0f %%, throttle %.0f %%, steps %d%s",
                     k ? "; " : "", mode_name(sb_mode(&x.sb)), slam_cap(&x.sb.slam), x.sb.servo.out_pct,
                     wt.steps_down, k == 1 ? (x.sb.nerv.nervous ? ", nervous (AUTO slows too)" : ", NOT nervous") : "");
            if (k == 1 && !x.sb.nerv.nervous) ok = 0;
        }
        test_report("SL5 works in every mode: HDG LOCK and AUTO capped; AUTO also counts it as rough water (nervous)",
                    ok == 2, t);
    }

    /* SL6 override: the onboard driver holds the stick above the cap for 1 s
     * -> cap lifted (warning), never above the power limit; back below the
     * cap -> the cap applies again */
    {
        static sbsim_t x;
        static slamwatch_t wt;
        memset(&wt, 0, sizeof wt);
        slam_go(&x, 967u, 3.0f);
        x.sb.slam.cfg.limit_g = 9.0f;
        x.sb.slam.cfg.yellow_frac = 0.12f;
        sbsim_run(&x, 40000u, slam_hook, &wt);              /* full throttle all the time: no override */
        float cap = slam_cap(&x.sb.slam);
        bool no_over = !x.sb.slam.override && cap < 100.0f;
        joy_set(0.0f, 0.6f, false, false);                   /* kick-down: ease off ... */
        sbsim_run(&x, x.t + 500u, slam_hook, &wt);
        joy_set(0.0f, 1.0f, false, false);                   /* ... and push to full */
        sbsim_run(&x, x.t + 600u, slam_hook, &wt);
        bool not_yet = !x.sb.slam.override;
        sbsim_run(&x, x.t + 3000u, slam_hook, &wt);
        bool over = x.sb.slam.override;
        float out_over = x.sb.servo.out_pct;
        joy_set(0.0f, 0.2f, false, false);
        sbsim_run(&x, x.t + 3000u, slam_hook, &wt);
        bool back = !x.sb.slam.override;
        snprintf(d, sizeof d, "cap %.0f %% while at full throttle all along: no override %s; kick-down (60 %% -> full): after 0.6 s %s, "
                 "after 3.6 s %s (throttle %.0f %%, power limit %.0f %%, overrides %lu); stick back: override off %s",
                 cap, no_over ? "yes" : "NO", not_yet ? "not yet" : "TOO EARLY", over ? "ON" : "NO", out_over,
                 x.sb.plim.fwd_pct, (unsigned long)x.sb.slam.overrides, back ? "yes" : "NO");
        test_report("SL6 onboard override by kick-down (ease off, full for 1 s): cap lifted, power limit still applies",
                    cap < 100.0f && no_over && not_yet && over && out_over > cap + 5.0f &&
                    out_over <= x.sb.plim.fwd_pct + 0.5f && back, d);
    }

    /* SL7 BNO085 stops sending: warning (sensor lost), no new reduction, no mode change */
    {
        static sbsim_t x;
        slam_go(&x, 968u, 3.0f);
        sbsim_run(&x, 20000u, NULL, NULL);
        sim_hal_set_accel(false);
        sbsim_run(&x, 30000u, NULL, NULL);
        bool lost = x.sb.slam.lost;
        uint32_t steps = x.sb.slam.steps;
        x.w.sea_state = 5.0f;
        sbsim_run(&x, 50000u, NULL, NULL);
        snprintf(d, sizeof d, "sensor lost %s; mode %s; steps while blind %lu; alarm bits 0x%lx",
                 lost ? "yes" : "NO", mode_name(sb_mode(&x.sb)), (unsigned long)(x.sb.slam.steps - steps),
                 (unsigned long)x.sb.alarm_bits);
        test_report("SL7 slamming sensor lost: shown as a warning, nothing else changes", lost &&
                    sb_mode(&x.sb) == MODE_MANUAL && x.sb.slam.steps == steps && !(x.sb.alarm_bits & SB_ALM_SLAM), d);
        sim_hal_set_accel(true);
    }

    /* SL8 slamming in the 20 Hz log record and in the black box (CPU1 RAM free) */
    {
        static sbsim_t x;
        static char names[16][128];
        char path[512];
        slam_go(&x, 969u, 3.0f);
        sbsim_log_start(&x, "sd_sl8", 9u);
        x.sb.slam.cfg.limit_g = 9.0f;
        x.sb.slam.cfg.yellow_frac = 0.12f;
        sbsim_run(&x, 70000u, NULL, NULL);
        sim_hal_fail_heading(true);                         /* a fault -> black box */
        sbsim_run(&x, x.t + 35000u, NULL, NULL);
        sim_hal_fail_heading(false);
        logger_close(&s_logger);
        x.log_on = false;
        int nb = tlg_list("sd_sl8", "BB", names, 16);
        tlg_info_t bb;
        memset(&bb, 0, sizeof bb);
        static slamlog_t sl;
        memset(&sl, 0, sizeof sl);
        if (nb > 0) {
            snprintf(path, sizeof path, "sd_sl8/%.120s", names[0]);
            tlg_read(path, &bb, NULL, NULL);
            memcpy(sl.fields, bb.fields, sizeof sl.fields);
            tlg_read(path, &bb, slamlog_cb, &sl);
        }
        snprintf(d, sizeof d, "black box %d file(s), %u data records (%.0f s); fields slam_peak/level/rate/cap/state %s; "
                 "in the records: max peak %.2f g, max level %.2f g, max rate %u/min, lowest cap %.0f %%, yellow seen %s",
                 nb, bb.data, bb.data / 20.0, sl.have ? "present" : "MISSING", sl.peak, sl.level, sl.rate, sl.cap,
                 sl.yellow ? "yes" : "no");
        test_report("SL8 slamming values in the 20 Hz log record (and the black box)",
                    nb >= 1 && bb.data >= 1750u && sl.have && sl.peak > 1.0f && sl.level > 1.0f && sl.rate > 10u &&
                    sl.cap < 100.0f && sl.yellow, d);
    }
}

/* ------------------------------------------- AIS device: second GPS, silent (section A) -- */

typedef struct {
    pisim_t  p;
    bool     send_gps2;
    uint32_t next_gps2;
    uint32_t sent;
} gps2sim_t;

static void gps2_hook(sbsim_t *x, void *ctx)
{
    gps2sim_t *g = (gps2sim_t *)ctx;
    pi_hook(x, &g->p);
    if (g->send_gps2 && x->t >= g->next_gps2) {
        g->next_gps2 = x->t + 1000u;
        double lat, lon;
        world_latlon(&x->w, &lat, &lon);                    /* the AIS device's own GPS: the truth */
        uint8_t pl[10], fr[PROTO_MAX_FRAME];
        proto_wr_t w;
        proto_wr_init(&w, pl, sizeof pl);
        proto_wr_u32(&w, (uint32_t)(int32_t)lround(lat * 1e7));
        proto_wr_u32(&w, (uint32_t)(int32_t)lround(lon * 1e7));
        proto_wr_u16(&w, 200u);
        size_t n = proto_encode(fr, PROTO_T_GW_GPS2, 0u, pl, (uint8_t)w.n);
        sim_hal_gw_rx_push(fr, (uint32_t)n);
        g->sent++;
    }
}

/* AUTO 3 km north; from 1 km the main GPS is dragged sideways at 0.5 m/s.
 * Returns the detection time (s, -1 = not within 4 min) and the reason. */
static float gps2_drag_run(uint32_t seed, float check_m, bool feed, float drag_ms, uint32_t window_ms,
                           dr_reason_t *why, uint32_t *rx)
{
    static sbsim_t x;
    static gps2sim_t g;
    memset(&g, 0, sizeof g);
    sbsim_init(&x, seed, 0.0f, 0.0f, NULL);
    pisim_init(&g.p, &x, 921600u);
    g.send_gps2 = feed;
    sbsim_run(&x, 2000u, gps2_hook, &g);
    x.sb.dr.cfg.gps2_max_m = check_m;
    auto_to(&x, 9000.0f, 0.0f, 12.0f);
    double n0 = x.w.north_m;
    while (x.w.north_m - n0 < 1000.0 && x.t < 400000u) sbsim_run(&x, x.t + 100u, gps2_hook, &g);
    x.w.gps_dragging = true;
    x.w.gps_drag_n = 0.0f; x.w.gps_drag_e = drag_ms;
    x.w.gps_drag_max_m = 300.0f;
    uint32_t t0 = x.t;
    float found = -1.0f;
    *why = DR_WHY_NONE;
    while (x.t < t0 + window_ms) {
        sbsim_run(&x, x.t + 100u, gps2_hook, &g);
        if (x.sb.dr.state != DR_GPS) {
            found = (float)(x.t - t0) * 0.001f;
            *why = x.sb.dr.why;
            break;
        }
    }
    *rx = x.sb.gps2.count;
    return found;
}

static void ais_tests(void)
{
    char d[900];
    printf("\n=== Section A: AIS device (second GPS, receiver silent) ===\n");

    /* GP1 a slow GPS drag (spoofer) is caught by the second GPS (30 m) - the
     * existing checks miss drags of 0.2 m/s and slower; the check is off by default */
    {
        static const float sp[4] = { 0.05f, 0.1f, 0.2f, 0.3f };
        float on[4], off[4];
        dr_reason_t won[4], woff[4];
        uint32_t rx[4], rx2;
        int len = 0;
        for (int k = 0; k < 4; k++) {
            on[k] = gps2_drag_run(971u, 30.0f, true, sp[k], 600000u, &won[k], &rx[k]);
            off[k] = gps2_drag_run(971u, 0.0f, true, sp[k], 600000u, &woff[k], &rx2);
            char wo[64];
            if (off[k] < 0.0f) snprintf(wo, sizeof wo, "NOT caught in 10 min");
            else snprintf(wo, sizeof wo, "%.0f s (%s)", off[k], dr_reason_name(woff[k]));
            len += snprintf(d + len, sizeof d - (size_t)len, "%s%.2f m/s: with %.0f s (%s), without %s; ",
                            k ? "" : "drag ", sp[k], on[k], dr_reason_name(won[k]), wo);
        }
        dr_config_t defc;
        dr_default_config(&defc);
        snprintf(d + len, sizeof d - (size_t)len, "default check %.0f m = off", defc.gps2_max_m);
        bool ok = defc.gps2_max_m == 0.0f && rx[1] > 100u;
        for (int k = 0; k < 4; k++)
            ok = ok && on[k] > 0.0f && (off[k] < 0.0f ? won[k] == DR_WHY_GPS2 : on[k] <= off[k] + 1.0f);
        ok = ok && off[0] < 0.0f && off[1] < 0.0f && won[1] == DR_WHY_GPS2;
        test_report("GP1 second GPS (AIS device) catches slow GPS drags the other checks miss; off by default", ok, d);
    }

    /* GP2 second GPS missing / stale with the check on: no false rejection */
    {
        static sbsim_t x;
        static gps2sim_t g;
        memset(&g, 0, sizeof g);
        sbsim_init(&x, 972u, 0.0f, 0.0f, NULL);
        pisim_init(&g.p, &x, 921600u);
        g.send_gps2 = true;
        sbsim_run(&x, 2000u, gps2_hook, &g);
        x.sb.dr.cfg.gps2_max_m = 30.0f;
        auto_to(&x, 3000.0f, 0.0f, 12.0f);
        sbsim_run(&x, x.t + 30000u, gps2_hook, &g);
        g.send_gps2 = false;                                /* the AIS device stops */
        uint32_t s0 = x.sb.dr.suspects;
        sbsim_run(&x, x.t + 120000u, gps2_hook, &g);
        snprintf(d, sizeof d, "second GPS stopped for 120 s: position source %s, integrity rejections %lu",
                 dr_state_name(x.sb.dr.state), (unsigned long)(x.sb.dr.suspects - s0));
        test_report("GP2 second GPS stops: no false rejection (only a fresh second GPS is compared)",
                    x.sb.dr.state == DR_GPS && x.sb.dr.suspects == s0, d);
    }

    /* GP3 AIS RECEIVER SILENT from the gateway: alarm only - power limit, mode
     * unchanged, no video SAVE */
    {
        static sbsim_t x;
        static pisim_t p;
        sbsim_init(&x, 973u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        sbsim_run(&x, x.t + 2000u, pi_hook, &p);
        float plim0 = x.sb.plim.fwd_pct;
        uint16_t save0 = x.sb.save.seq;
        p.flags = SB_GWF_AIS_SILENT;
        sbsim_run(&x, x.t + 3000u, pi_hook, &p);
        bool alarm = (x.sb.alarm_bits & SB_ALM_AIS_SILENT) != 0u;
        snprintf(d, sizeof d, "alarm AIS RECEIVER SILENT %s; power limit %.0f -> %.0f %%; mode %s; SAVE requests %u",
                 alarm ? "yes" : "NO", plim0, x.sb.plim.fwd_pct, mode_name(sb_mode(&x.sb)),
                 (unsigned)(x.sb.save.seq - save0));
        test_report("GP3 AIS RECEIVER SILENT: alarm only (power limit, mode unchanged, no SAVE)",
                    alarm && x.sb.plim.fwd_pct == plim0 && plim0 > 30.0f && sb_mode(&x.sb) == MODE_HDG_LOCK &&
                    x.sb.save.seq == save0, d);
    }
}

/* ------------------------------------- gateway navigation request (section D) -- */

typedef struct {
    pisim_t  p;
    bool     on;                /* the "gateway" refreshes the request at 2 Hz */
    uint8_t  seq, kind, reason, alarm;
    float    heading, cap;      /* cap < 0 = none */
    uint16_t lease;
    uint32_t next;
    float    max_kn, min_hdg_err;
} navsim_t;

static void nav_push(navsim_t *n, uint8_t kind, uint8_t reason, uint8_t alarm, float hdg, float cap, uint16_t lease)
{
    uint8_t pl[10], fr[PROTO_MAX_FRAME];
    proto_wr_t w;
    proto_wr_init(&w, pl, sizeof pl);
    proto_wr_u8(&w, n->seq);
    proto_wr_u8(&w, kind);
    proto_wr_u8(&w, reason);
    proto_wr_u8(&w, alarm);
    proto_wr_u16(&w, (uint16_t)lroundf(hdg * 100.0f));
    proto_wr_u16(&w, cap < 0.0f ? 0xFFFFu : (uint16_t)lroundf(cap * 10.0f));
    proto_wr_u16(&w, lease);
    size_t len = proto_encode(fr, PROTO_T_GW_NAVREQ, 0u, pl, (uint8_t)w.n);
    sim_hal_gw_rx_push(fr, (uint32_t)len);
}

static void navreq_hook(sbsim_t *x, void *ctx)
{
    navsim_t *n = (navsim_t *)ctx;
    pi_hook(x, &n->p);
    if (n->on && x->t >= n->next) {
        n->next = x->t + 500u;
        nav_push(n, n->kind, n->reason, n->alarm, n->heading, n->cap, n->lease);
    }
}

static void navsim_init(navsim_t *n, sbsim_t *x)
{
    memset(n, 0, sizeof *n);
    pisim_init(&n->p, x, 921600u);
    n->lease = 3000u;
    n->cap = -1.0f;
}

static void nav_tests(void)
{
    char d[700];
    printf("\n=== Section D: gateway navigation request (collision avoidance, shallow water) ===\n");

    /* NV1 AUTO: the avoidance heading + speed cap are steered; request ends -> back to the leg */
    {
        static sbsim_t x;
        static navsim_t n;
        sbsim_init(&x, 981u, 0.0f, 0.0f, NULL);
        navsim_init(&n, &x);
        sbsim_run(&x, 2000u, navreq_hook, &n);
        auto_to(&x, 8000.0f, 0.0f, 14.0f);
        sbsim_run(&x, x.t + 60000u, navreq_hook, &n);
        float kn0 = x.w.speed_kn;
        n.on = true; n.seq = 1; n.kind = SB_NAV_HEADING; n.reason = 1; n.alarm = 1; n.heading = 60.0f; n.cap = 7.0f;
        sbsim_run(&x, x.t + 60000u, navreq_hook, &n);
        float hdg = x.w.heading_deg, kn = x.w.speed_kn;
        bool alarm = (x.sb.alarm_bits & SB_ALM_COLLISION) != 0u;
        float plim = x.sb.plim.fwd_pct;
        n.kind = SB_NAV_NONE; n.alarm = 0;
        sbsim_run(&x, x.t + 60000u, navreq_hook, &n);
        double lat, lon;
        dr_latlon(&x.sb.dr, &lat, &lon);
        float back = x.sb.ap.target_heading;
        snprintf(d, sizeof d, "before %.1f kn; request heading 060 / 7 kn -> heading %.0f, %.1f kn, alarm COLLISION %s, "
                 "power limit %.0f %% (unchanged); request ended -> target heading %.0f (back towards the waypoint "
                 "north), mode %s, %lu requests received",
                 kn0, hdg, kn, alarm ? "yes" : "NO", plim, back, mode_name(sb_mode(&x.sb)), (unsigned long)x.sb.nav.count);
        test_report("NV1 AUTO applies the avoidance heading + speed cap; when it ends the boat goes back to its leg",
                    fabsf(hdg_err(hdg, 60.0f)) < 6.0f && kn < 7.6f && kn0 > 12.0f && alarm && plim >= 99.0f &&
                    sb_mode(&x.sb) == MODE_AUTO && fabsf(hdg_err(back, 60.0f)) > 40.0f, d);
    }

    /* NV2 gateway lost during the avoidance: slow down, hold the heading, DRIVER DECISION alarm */
    {
        static sbsim_t x;
        static navsim_t n;
        sbsim_init(&x, 982u, 0.0f, 0.0f, NULL);
        navsim_init(&n, &x);
        sbsim_run(&x, 2000u, navreq_hook, &n);
        auto_to(&x, 8000.0f, 0.0f, 14.0f);
        sbsim_run(&x, x.t + 40000u, navreq_hook, &n);
        n.on = true; n.seq = 1; n.kind = SB_NAV_HEADING; n.reason = 1; n.alarm = 1; n.heading = 45.0f; n.cap = 9.0f;
        sbsim_run(&x, x.t + 40000u, navreq_hook, &n);
        n.on = false;                                       /* the Pi is gone */
        n.p.pi_on = false;
        uint32_t t0 = x.t;
        while (!x.sb.nav.lost && x.t < t0 + 10000u) sbsim_run(&x, x.t + 100u, navreq_hook, &n);
        float t_lost = (x.t - t0) * 0.001f;
        float held = x.sb.nav.lost_heading;
        sbsim_run(&x, x.t + 40000u, navreq_hook, &n);
        bool dec = (x.sb.alarm_bits & SB_ALM_DECISION) != 0u;
        snprintf(d, sizeof d, "request stopped -> LOST after %.1f s (lease 3 s); speed %.1f kn (target %.0f), heading "
                 "%.0f (held %.0f, not back to the old leg), alarm DRIVER DECISION %s, mode %s",
                 t_lost, x.w.speed_kn, x.cfg.nav_lost_kn, x.w.heading_deg, held, dec ? "yes" : "NO",
                 mode_name(sb_mode(&x.sb)));
        test_report("NV2 gateway lost during an avoidance: AURIX slows down, holds the heading, alarms (no blind plan)",
                    x.sb.nav.lost && t_lost >= 2.9f && t_lost < 3.6f && x.w.speed_kn < x.cfg.nav_lost_kn + 0.8f &&
                    fabsf(hdg_err(x.w.heading_deg, held)) < 6.0f && dec && sb_mode(&x.sb) == MODE_AUTO, d);
    }

    /* NV3 joystick takeover during the avoidance -> MANUAL at once; STANDBY button wins */
    {
        int ok = 0;
        char t[400] = "";
        for (int k = 0; k < 2; k++) {
            static sbsim_t x;
            static navsim_t n;
            sbsim_init(&x, 983u + (uint32_t)k, 0.0f, 0.0f, NULL);
            navsim_init(&n, &x);
            sbsim_run(&x, 2000u, navreq_hook, &n);
            auto_to(&x, 8000.0f, 0.0f, 14.0f);
            sbsim_run(&x, x.t + 30000u, navreq_hook, &n);
            n.on = true; n.seq = 1; n.kind = SB_NAV_HEADING; n.alarm = 1; n.heading = 80.0f; n.cap = 8.0f;
            sbsim_run(&x, x.t + 10000u, navreq_hook, &n);
            uint32_t t0 = x.t;
            if (k == 0) joy_set(-0.7f, 0.0f, false, false);
            else press(&x, HAL_BTN_STANDBY);
            while (sb_mode(&x.sb) == MODE_AUTO && x.t < t0 + 2000u) sbsim_run(&x, x.t + 10u, navreq_hook, &n);
            float dt = (x.t - t0) * 0.001f;
            sbsim_run(&x, x.t + 5000u, navreq_hook, &n);
            bool g = (k == 0) ? (sb_mode(&x.sb) == MODE_MANUAL && dt < 0.3f + 0.1f) : sb_mode(&x.sb) == MODE_STANDBY;
            ok += g;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s -> %s after %.2f s (request still sent: ignored)",
                     k ? "; " : "", k ? "STANDBY button" : "joystick", mode_name(sb_mode(&x.sb)), dt);
            joy_set(0.0f, 0.0f, false, false);
        }
        test_report("NV3 during an avoidance the joystick takes over instantly; STANDBY wins", ok == 2, t);
    }

    /* NV4 MANUAL and REMOTE MANUAL: alarm only - no steering, throttle or power-limit change */
    {
        static sbsim_t x, y;
        static navsim_t n, m;
        float out[2], rud[2];
        for (int k = 0; k < 2; k++) {
            sbsim_t *z = k ? &y : &x;
            navsim_t *nn = k ? &m : &n;
            sbsim_init(z, 985u, 0.0f, 0.0f, NULL);
            navsim_init(nn, z);
            sbsim_run(z, 2000u, navreq_hook, nn);
            joy_set(0.0f, 0.3f, false, false);
            sbsim_run(z, z->t + 1000u, navreq_hook, nn);
            joy_set(0.0f, 0.0f, false, false);
            sbsim_run(z, z->t + 500u, navreq_hook, nn);
            power_full(z);
            joy_set(0.2f, 0.8f, false, false);
            if (k) {                                        /* with the request (alarm + heading + cap) */
                nn->on = true; nn->seq = 1; nn->kind = SB_NAV_HEADING; nn->alarm = 3; nn->heading = 200.0f;
                nn->cap = 3.0f;
            }
            sbsim_run(z, z->t + 30000u, navreq_hook, nn);
            out[k] = z->sb.servo.out_pct;
            rud[k] = z->sb.ap.motor_cmd;
            joy_set(0.0f, 0.0f, false, false);
        }
        bool alarm = (y.sb.alarm_bits & SB_ALM_COLLISION) && (y.sb.alarm_bits & SB_ALM_DECISION);
        snprintf(d, sizeof d, "MANUAL without / with a request (heading 200, cap 3 kn, alarms): throttle %.1f / %.1f %%, "
                 "power limit %.0f / %.0f %%, alarms COLLISION + DECISION %s, mode %s; requests are never applied "
                 "outside AUTO (applied flag %s)",
                 out[0], out[1], x.sb.plim.fwd_pct, y.sb.plim.fwd_pct, alarm ? "yes" : "NO",
                 mode_name(sb_mode(&y.sb)), (sb_mode(&y.sb) == MODE_AUTO) ? "?" : "no");
        test_report("NV4 MANUAL: collision alarm only - steering, throttle and power limit unchanged",
                    fabsf(out[0] - out[1]) < 0.5f && y.sb.plim.fwd_pct == x.sb.plim.fwd_pct && alarm &&
                    sb_mode(&y.sb) == MODE_MANUAL, d);
        (void)rud;
    }

    /* NV5 HEADING LOCK: the accepted heading is set once (same number again: not again);
     * an AUTO-type request does nothing in HDG LOCK */
    {
        static sbsim_t x;
        static navsim_t n;
        sbsim_init(&x, 986u, 90.0f, 0.0f, NULL);
        navsim_init(&n, &x);
        sbsim_run(&x, 2000u, navreq_hook, &n);
        hdg_lock(&x);
        joy_set(0.0f, 0.5f, false, false);
        sbsim_run(&x, x.t + 20000u, navreq_hook, &n);
        n.on = true; n.seq = 7; n.kind = SB_NAV_HEADING; n.heading = 10.0f; n.cap = 3.0f;
        sbsim_run(&x, x.t + 5000u, navreq_hook, &n);
        float t_auto = x.sb.ap.target_heading;
        n.kind = SB_NAV_HDG_SET; n.seq = 8; n.heading = 130.0f;
        sbsim_run(&x, x.t + 3000u, navreq_hook, &n);
        float t_set = x.sb.ap.target_heading;
        press(&x, HAL_BTN_PLUS_10);                         /* the driver adjusts afterwards */
        sbsim_run(&x, x.t + 3000u, navreq_hook, &n);
        float t_adj = x.sb.ap.target_heading;
        n.seq = 9; n.heading = 90.0f;                       /* return proposal accepted */
        sbsim_run(&x, x.t + 2000u, navreq_hook, &n);
        float t_ret = x.sb.ap.target_heading;
        snprintf(d, sizeof d, "AUTO-type request in HDG LOCK -> target %.0f (unchanged 090); accepted 130 -> %.0f; "
                 "driver +10 -> %.0f (the repeated request does not undo it); accepted return 090 -> %.0f; applied %lu",
                 t_auto, t_set, t_adj, t_ret, (unsigned long)x.sb.nav.applied_hdg);
        test_report("NV5 HEADING LOCK: accepted avoidance heading set once, return heading after ACCEPT, driver keeps "
                    "the buttons",
                    fabsf(hdg_err(t_auto, 90.0f)) < 1.0f && fabsf(hdg_err(t_set, 130.0f)) < 0.6f &&
                    fabsf(hdg_err(t_adj, 140.0f)) < 0.6f && fabsf(hdg_err(t_ret, 90.0f)) < 0.6f &&
                    x.sb.nav.applied_hdg == 2u, d);
        joy_set(0.0f, 0.0f, false, false);
    }

    /* NV9 R6: the AURIX limits what the Pi may ask: an AUTO heading far off the
     *     leg track is not steered (cap still slows, DRIVER DECISION); a heading
     *     without a speed cap is refused; a HDG LOCK heading that arrived before
     *     HDG LOCK was entered is not applied */
    {
        static sbsim_t x;
        static navsim_t n;
        sbsim_init(&x, 991u, 0.0f, 0.0f, NULL);
        navsim_init(&n, &x);
        sbsim_run(&x, 2000u, navreq_hook, &n);
        auto_to(&x, 8000.0f, 0.0f, 14.0f);
        sbsim_run(&x, x.t + 60000u, navreq_hook, &n);
        n.on = true; n.seq = 1; n.kind = SB_NAV_HEADING; n.reason = 1; n.alarm = 0; n.heading = 170.0f; n.cap = 7.0f;
        sbsim_run(&x, x.t + 40000u, navreq_hook, &n);
        float tgt = x.sb.ap.target_heading, kn = x.w.speed_kn;
        bool dec = (x.sb.alarm_bits & SB_ALM_DECISION) != 0u;
        uint32_t far = x.sb.nav.too_far;
        n.on = false;
        uint32_t rej0 = x.sb.nav.rejected;
        nav_push(&n, SB_NAV_HEADING, 1u, 0u, 20.0f, -1.0f, 3000u);    /* heading without a cap */
        sbsim_run(&x, x.t + 200u, navreq_hook, &n);
        bool nocap_rej = x.sb.nav.rejected == rej0 + 1u;
        /* HDG_SET sent while still in AUTO, then the driver presses HDG LOCK within the lease */
        n.seq = 5;
        nav_push(&n, SB_NAV_HDG_SET, 1u, 0u, 250.0f, -1.0f, 3000u);
        sbsim_run(&x, x.t + 300u, navreq_hook, &n);
        press(&x, HAL_BTN_HDG_LOCK);
        float locked = x.sb.ap.heading;
        sbsim_run(&x, x.t + 2000u, navreq_hook, &n);
        bool hdg = sb_mode(&x.sb) == MODE_HDG_LOCK;
        float t_hdg = x.sb.ap.target_heading;
        snprintf(d, sizeof d, "AUTO leg north, request heading 170 (limit %.0f deg off the track) / 7 kn -> target %.0f, "
                 "%.1f kn, DRIVER DECISION %s, ignored %lu; heading without a cap: refused %s; HDG_SET 250 that arrived in "
                 "AUTO, then HDG LOCK -> target %.0f (locked %.0f, not 250), mode %s",
                 x.sb.cfg.nav_max_dev_deg, tgt, kn, dec ? "yes" : "NO", (unsigned long)far, nocap_rej ? "yes" : "NO",
                 t_hdg, locked, mode_name(sb_mode(&x.sb)));
        test_report("NV9 navigation request limits: heading far off the track not steered, heading needs a cap, "
                    "no stale HDG LOCK heading",
                    fabsf(hdg_err(tgt, 0.0f)) < 20.0f && kn < 7.6f && dec && far >= 1u && nocap_rej && hdg &&
                    fabsf(hdg_err(t_hdg, locked)) < 2.0f && fabsf(hdg_err(t_hdg, 250.0f)) > 30.0f, d);
    }

    /* NV6 invalid requests are ignored */
    {
        static sbsim_t x;
        static navsim_t n;
        sbsim_init(&x, 987u, 0.0f, 0.0f, NULL);
        navsim_init(&n, &x);
        sbsim_run(&x, 2000u, navreq_hook, &n);
        nav_push(&n, 9u, 1u, 0u, 10.0f, 5.0f, 3000u);       /* unknown kind */
        nav_push(&n, SB_NAV_HEADING, 1u, 0u, 10.0f, 70.0f, 3000u);   /* 70 kn cap */
        nav_push(&n, SB_NAV_CAP, 1u, 0u, 0.0f, 5.0f, 60000u);  /* 60 s lease */
        sbsim_run(&x, x.t + 500u, navreq_hook, &n);
        snprintf(d, sizeof d, "3 invalid requests -> rejected %lu, accepted %lu",
                 (unsigned long)x.sb.nav.rejected, (unsigned long)x.sb.nav.count);
        test_report("NV6 invalid navigation requests (kind, speed, lease) are ignored", x.sb.nav.rejected == 3u &&
                    x.sb.nav.count == 0u, d);
    }

    /* NV7 camera obstacle (vision stage 3) in AUTO: speed cap 0 = slow to idle (never more throttle);
     * the gateway lost during it -> the lower cap stays (not the 4 kn of nav_lost_kn) */
    {
        static sbsim_t x;
        static navsim_t n;
        sbsim_init(&x, 988u, 0.0f, 0.0f, NULL);
        navsim_init(&n, &x);
        sbsim_run(&x, 2000u, navreq_hook, &n);
        auto_to(&x, 8000.0f, 0.0f, 14.0f);
        sbsim_run(&x, x.t + 60000u, navreq_hook, &n);
        float kn0 = x.w.speed_kn, thr0 = x.sb.servo.out_pct, hdg0 = x.sb.ap.target_heading;
        n.on = true; n.seq = 1; n.kind = SB_NAV_CAP; n.reason = 8; n.alarm = 8; n.cap = 0.0f;
        float thr_max = 0.0f, hdg_dev = 0.0f;
        uint32_t t1 = x.t;
        while (x.t < t1 + 60000u) {
            sbsim_run(&x, x.t + 100u, navreq_hook, &n);
            thr_max = fmaxf(thr_max, x.sb.servo.out_pct);
            hdg_dev = fmaxf(hdg_dev, fabsf(hdg_err(x.sb.ap.target_heading, hdg0)));
        }
        float kn1 = x.w.speed_kn;
        bool obst = (x.sb.alarm_bits & SB_ALM_OBSTACLE) != 0u;
        float plim = x.sb.plim.fwd_pct;
        n.on = false;                                       /* the Pi is gone while the boat is slowed down */
        n.p.pi_on = false;
        float thr_lost = 0.0f;
        uint32_t t2 = x.t;
        while (x.t < t2 + 40000u) {
            sbsim_run(&x, x.t + 100u, navreq_hook, &n);
            thr_lost = fmaxf(thr_lost, x.sb.servo.out_pct);
        }
        snprintf(d, sizeof d, "AUTO %.1f kn, throttle %.1f %%; obstacle cap 0 kn -> %.1f kn after 60 s, throttle max "
                 "%.1f %% (never above the start), target heading within %.1f deg of its leg (a cap only, no heading), OBSTACLE "
                 "AHEAD %s, power limit %.0f %%; gateway lost -> LOST %s, speed %.1f kn, throttle max %.1f %% "
                 "(the 0 kn cap stays, not %.0f kn), mode %s",
                 kn0, thr0, kn1, thr_max, hdg_dev, obst ? "yes" : "NO", plim, x.sb.nav.lost ? "yes" : "NO",
                 x.w.speed_kn, thr_lost, x.cfg.nav_lost_kn, mode_name(sb_mode(&x.sb)));
        test_report("NV7 vision speed cap in AUTO slows to idle, never raises the throttle; gateway lost keeps the "
                    "lower cap", kn0 > 12.0f && kn1 < 3.0f && thr_max <= thr0 + 0.5f && hdg_dev < 10.0f && obst &&
                    plim >= 99.0f && x.sb.nav.lost && thr_lost <= thr_max + 0.5f && x.w.speed_kn < 3.0f &&
                    sb_mode(&x.sb) == MODE_AUTO, d);
    }

    /* NV8 OBSTACLE AHEAD in MANUAL (stage 2 / 3): alarm only - throttle, steering, power limit unchanged */
    {
        static sbsim_t x, y;
        static navsim_t n, m;
        float out[2], rud[2], plim[2];
        for (int k = 0; k < 2; k++) {
            sbsim_t *z = k ? &y : &x;
            navsim_t *nn = k ? &m : &n;
            sbsim_init(z, 989u, 0.0f, 0.0f, NULL);
            navsim_init(nn, z);
            sbsim_run(z, 2000u, navreq_hook, nn);
            joy_set(0.0f, 0.3f, false, false);
            sbsim_run(z, z->t + 1000u, navreq_hook, nn);
            joy_set(0.0f, 0.0f, false, false);
            sbsim_run(z, z->t + 500u, navreq_hook, nn);
            power_full(z);
            joy_set(0.1f, 0.6f, false, false);
            if (k) {
                nn->on = true; nn->seq = 1; nn->kind = SB_NAV_CAP; nn->reason = 8; nn->alarm = 8; nn->cap = 0.0f;
            }
            sbsim_run(z, z->t + 30000u, navreq_hook, nn);
            out[k] = z->sb.servo.out_pct;
            rud[k] = z->sb.ap.motor_cmd;
            plim[k] = z->sb.plim.fwd_pct;
            joy_set(0.0f, 0.0f, false, false);
        }
        bool obst = (y.sb.alarm_bits & SB_ALM_OBSTACLE) != 0u;
        snprintf(d, sizeof d, "MANUAL without / with an obstacle request (cap 0 kn, alarm): throttle %.1f / %.1f %%, "
                 "steering %.2f / %.2f, power limit %.0f / %.0f %%, OBSTACLE AHEAD %s, mode %s",
                 out[0], out[1], rud[0], rud[1], plim[0], plim[1], obst ? "yes" : "NO", mode_name(sb_mode(&y.sb)));
        test_report("NV8 OBSTACLE AHEAD in MANUAL: alarm only, throttle / steering / power limit unchanged",
                    fabsf(out[0] - out[1]) < 0.5f && fabsf(rud[0] - rud[1]) < 0.05f && plim[0] == plim[1] && obst &&
                    sb_mode(&y.sb) == MODE_MANUAL, d);
    }
}

/* ------------------------------------------- T2: held stick always wins -- */

/* STANDBY pressed while the stick is held: the stick is ignored until
 * centred - so nothing it could no longer take over from may be engaged:
 * HDG LOCK button, AUTO start, remote driving, TRAINING are refused
 * ("centre the joystick"); centred: HDG LOCK works and the stick takes over. */
static void centre_tests(void)
{
    char d[600];
    const uint8_t TAKE = 0u;
    printf("\n=== T2: autopilot modes only with the joystick centred ===\n");
    static sbsim_t x;
    static program_t p;
    static link_t usb;
    prog_error_t err;
    uint8_t bad, tr[5] = { 0u, 0u, 0u, 0u, 0u };
    sbsim_init(&x, 241u, 0.0f, 0.0f, NULL);
    sbsim_run(&x, 2000u, NULL, NULL);
    link_init(&usb, &x.sb);
    usb_cmd(&x, &usb, PROTO_T_HELM, &TAKE, 1u);
    prog_error_t e2;
    prog_wp_t w = wp_world(&x, x.w.north_m + 600.0, x.w.east_m, 5.0f, 25.0f, WP_ACT_STOP);
    prog_make(&p, &w, 1, PROG_END_STOP);
    bool prog = sb_program_set(&x.sb, &p, &e2, &bad) == SB_OK;
    (void)err;
    joy_set(0.6f, 0.5f, false, false);                  /* driving on the stick */
    sbsim_run(&x, x.t + 8000u, NULL, NULL);
    bool manual = sb_mode(&x.sb) == MODE_MANUAL;
    sim_hal_press(HAL_BTN_STANDBY);                     /* stick still held */
    sbsim_run(&x, x.t + 4000u, NULL, NULL);
    bool standby = sb_mode(&x.sb) == MODE_STANDBY;
    uint32_t ref0 = x.sb.refused_centre;
    sim_hal_press(HAL_BTN_HDG_LOCK);
    sbsim_run(&x, x.t + 200u, NULL, NULL);
    bool hdg_refused = sb_mode(&x.sb) == MODE_STANDBY && x.sb.refused_centre == ref0 + 1u;
    sb_result_t r_auto = sb_auto_start(&x.sb);
    sb_result_t r_rem = sb_remote_request(&x.sb, 1u);
    uint8_t r_tr = usb_cmd(&x, &usb, PROTO_T_TRAINING, tr, sizeof tr);
    sbsim_run(&x, x.t + 500u, NULL, NULL);
    bool still = sb_mode(&x.sb) == MODE_STANDBY;
    /* centred: everything works again, and a deflection takes over */
    joy_set(0.0f, 0.0f, false, false);
    sbsim_run(&x, x.t + 1500u, NULL, NULL);
    sim_hal_press(HAL_BTN_HDG_LOCK);
    sbsim_run(&x, x.t + 200u, NULL, NULL);
    bool hdg_ok = sb_mode(&x.sb) == MODE_HDG_LOCK;
    uint32_t t0 = x.t;
    joy_set(0.6f, 0.0f, false, false);
    while (sb_mode(&x.sb) != MODE_MANUAL && x.t < t0 + 1000u) sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
    uint32_t take_ms = x.t - t0;
    joy_set(0.0f, 0.0f, false, false);
    snprintf(d, sizeof d, "MANUAL %s, STANDBY with the stick held %s; then: HDG LOCK button %s (chirp), AUTO start "
             "result %d (want %d), remote request %d (want %d), TRAINING from the PC %u (want %u = centre the "
             "joystick), still STANDBY %s; stick centred: HDG LOCK %s, stick moved -> MANUAL after %lu ms",
             manual ? "yes" : "NO", standby ? "yes" : "NO", hdg_refused ? "refused" : "NOT REFUSED", (int)r_auto,
             (int)SB_ERR_NOT_CENTERED, (int)r_rem, (int)SB_ERR_NOT_CENTERED, r_tr, PROTO_R_NOT_CENTERED,
             still ? "yes" : "NO", hdg_ok ? "yes" : "NO", (unsigned long)take_ms);
    test_report("T2 HDG LOCK / AUTO / REMOTE / TRAINING refused while the stick is held; centred: it takes over",
                prog && manual && standby && hdg_refused && r_auto == SB_ERR_NOT_CENTERED &&
                r_rem == SB_ERR_NOT_CENTERED && r_tr == PROTO_R_NOT_CENTERED && still && hdg_ok &&
                sb_mode(&x.sb) == MODE_MANUAL && take_ms <= 200u, d);
}

/* ------------------------------------ R5: GPS fixes with impossible values -- */

/* In AUTO with a trusted GPS, single fixes with NaN / infinite / out-of-
 * range numbers arrive: each one is a rejected fix (GPS failure, dead
 * reckoning "invalid fix values"), nothing in the estimate or telemetry
 * becomes NaN, AUTO goes on by DR and the real GPS is trusted again after
 * the consistent fixes. A second GPS (gateway) with an impossible position
 * is ignored. */
static void gps_invalid_tests(void)
{
    char d[900];
    printf("\n=== R5: GPS fixes with NaN / impossible values ===\n");
    static sbsim_t x;
    static gps2sim_t g;
    static const char *const what[] = { "lat NaN", "lon +inf", "lat 95", "lon -200", "SOG NaN", "SOG 500 kn",
                                        "COG -10", "COG inf" };
    const int NK = 8;
    memset(&g, 0, sizeof g);
    sbsim_init(&x, 251u, 0.0f, 0.0f, NULL);
    pisim_init(&g.p, &x, 921600u);
    sbsim_run(&x, 2000u, gps2_hook, &g);
    bool autog = auto_to(&x, 20000.0f, 0.0f, 8.0f);
    sbsim_run(&x, x.t + 60000u, gps2_hook, &g);
    int rejected = 0, finite_all = 1, auto_kept = 1, back = 0;
    float worst_back = 0.0f;
    char per[400] = "";
    for (int k = 0; k < NK; k++) {
        bool trusted = x.sb.dr.state == DR_GPS;
        hal_gps_t b = x.sb.gps.last;
        b.fix = true;
        switch (k) {
        case 0: b.lat_deg = NAN; break;
        case 1: b.lon_deg = INFINITY; break;
        case 2: b.lat_deg = 95.0; break;
        case 3: b.lon_deg = -200.0; break;
        case 4: b.sog_kn = NAN; break;
        case 5: b.sog_kn = 500.0f; break;
        case 6: b.cog_deg = -10.0f; break;
        default: b.cog_deg = INFINITY; break;
        }
        uint32_t inv0 = x.sb.dr.invalid_fixes;
        sim_hal_gps_message(&b);
        sbsim_run(&x, x.t + AP_PERIOD_MS, gps2_hook, &g);
        bool rej = trusted && x.sb.dr.state != DR_GPS && x.sb.dr.why == DR_WHY_INVALID &&
                   x.sb.dr.invalid_fixes == inv0 + 1u;
        rejected += rej;
        double lat, lon;
        dr_latlon(&x.sb.dr, &lat, &lon);
        if (!isfinite(lat) || !isfinite(lon) || !isfinite(x.sb.dr.unc_m) || !isfinite(x.sb.gps.last.lat_deg) ||
            !isfinite(x.sb.gps.last.sog_kn) || !isfinite(x.sb.gps.last.cog_deg) || !isfinite(dr_sog_kn(&x.sb.dr)))
            finite_all = 0;
        /* the real GPS goes on: trusted again after the consistent fixes */
        uint32_t t0 = x.t;
        while (x.sb.dr.state != DR_GPS && x.t < t0 + 180000u) {
            sbsim_run(&x, x.t + 100u, gps2_hook, &g);
            if (sb_mode(&x.sb) != MODE_AUTO) auto_kept = 0;
        }
        float tb = (float)(x.t - t0) * 0.001f;
        if (x.sb.dr.state == DR_GPS) { back++; if (tb > worst_back) worst_back = tb; }
        snprintf(per + strlen(per), sizeof per - strlen(per), "%s%s %s", k ? ", " : "", what[k], rej ? "rejected" : "NOT");
    }
    /* second GPS from the gateway: impossible position ignored */
    uint32_t bad0 = x.sb.gps2.bad, cnt0 = x.sb.gps2.count;
    double lat2 = x.sb.gps2.lat;
    {
        uint8_t pl[10], fr[PROTO_MAX_FRAME];
        proto_wr_t w;
        proto_wr_init(&w, pl, sizeof pl);
        proto_wr_u32(&w, (uint32_t)(int32_t)950000000);          /* 95 deg */
        proto_wr_u32(&w, (uint32_t)(int32_t)230000000);
        proto_wr_u16(&w, 100u);
        size_t n = proto_encode(fr, PROTO_T_GW_GPS2, 0u, pl, (uint8_t)w.n);
        sim_hal_gw_rx_push(fr, (uint32_t)n);
        sbsim_run(&x, x.t + 200u, gps2_hook, &g);
    }
    bool gps2_ign = x.sb.gps2.bad == bad0 + 1u && x.sb.gps2.count == cnt0 && x.sb.gps2.lat == lat2;
    snprintf(d, sizeof d, "AUTO %s; %d of %d bad fixes rejected (%s); estimate / telemetry values all finite %s; "
             "AUTO kept on DR %s; GPS trusted again %d/%d (slowest %.0f s); second GPS at 95 deg: ignored %s",
             autog ? "yes" : "NO", rejected, NK, per, finite_all ? "yes" : "NO", auto_kept ? "yes" : "NO", back, NK,
             worst_back, gps2_ign ? "yes" : "NO");
    test_report("R5 GPS fixes with NaN / inf / out-of-range values: rejected (GPS failure), nothing becomes NaN",
                autog && rejected == NK && finite_all && auto_kept && back == NK && gps2_ign, d);
}

/* ------------------------------ T3: GPS trusted again after a spoof at speed -- */

/* AUTO north at 12 kn; at the spoof the boat becomes 15 % slower than the
 * learned speed table (fouling, head sea), so DR's speed is too high. A
 * spoofer jumps the GPS 800 m east for 20 s; afterwards the GPS is right
 * again, or the spoofer drags it (drag_n along the course, drag_e sideways).
 * setup 0: nothing independent; 1: second GPS (AIS device) check on and fed;
 * 2: paddlewheel. Returns the time from the end of the jump to "trusted"
 * (s), -1 = not within 10 min. *off_line = worst distance off the course line. */
static float gps_retrust_run(uint32_t seed, int setup, float drag_n, float drag_e, float *off_line)
{
    static sbsim_t x;
    static gps2sim_t g;
    memset(&g, 0, sizeof g);
    sbsim_init(&x, seed, 0.0f, 0.0f, NULL);
    if (setup == 2) x.w.speed_sensor = true;
    pisim_init(&g.p, &x, 921600u);
    g.send_gps2 = (setup == 1);
    sbsim_run(&x, 2000u, gps2_hook, &g);
    if (setup == 1) x.sb.dr.cfg.gps2_max_m = 60.0f;
    auto_to(&x, 30000.0f, 0.0f, 12.0f);
    sbsim_run(&x, x.t + 180000u, gps2_hook, &g);        /* the speed table learns this speed */
    x.w.vmax_fwd_kn *= 0.85f;                            /* now 15 % slower than the table says */
    x.w.gps_off_e = 800.0;
    sbsim_run(&x, x.t + 20000u, gps2_hook, &g);
    x.w.gps_off_e = 0.0;
    if (drag_n != 0.0f || drag_e != 0.0f) {
        x.w.gps_dragging = true;
        x.w.gps_drag_n = drag_n;
        x.w.gps_drag_e = drag_e;
        x.w.gps_drag_max_m = 2000.0f;
    }
    uint32_t t0 = x.t;
    float found = -1.0f;
    *off_line = 0.0f;
    while (x.t < t0 + 600000u) {
        sbsim_run(&x, x.t + 100u, gps2_hook, &g);
        if (fabs(x.w.east_m) > *off_line) *off_line = (float)fabs(x.w.east_m);
        if (x.sb.dr.state == DR_GPS) { found = (float)(x.t - t0) * 0.001f; break; }
    }
    return found;
}

static void gps_retrust_tests(void)
{
    char d[900];
    printf("\n=== T3: GPS trusted again after a spoof at speed ===\n");
    float o[7], om = 0.0f;
    float n_ok = gps_retrust_run(261u, 0, 0.0f, 0.0f, &o[0]);
    float n_side = gps_retrust_run(262u, 0, 0.0f, 0.6f, &o[1]);
    float n_along = gps_retrust_run(263u, 0, 1.5f, 0.0f, &o[2]);
    float g_ok = gps_retrust_run(264u, 1, 0.0f, 0.0f, &o[3]);
    float g_side = gps_retrust_run(265u, 1, 0.0f, 0.6f, &o[4]);
    float g_along = gps_retrust_run(266u, 1, 1.5f, 0.0f, &o[5]);
    float p_ok = gps_retrust_run(267u, 2, 0.0f, 0.0f, &o[6]);
    for (int i = 0; i < 7; i++) om = fmaxf(om, o[i]);
#define TS(v) ((v) < 0.0f ? -1.0f : (v))
    snprintf(d, sizeof d, "spoof at 12 kn, DR speed 15 %% too high. Nothing independent: GPS right -> trusted after %.0f s "
             "(-1 = >10 min: until GPS and DR agree), sideways drag 0.6 m/s %s, along drag 1.5 m/s %s. Second GPS: right "
             "-> %.0f s, sideways %s, along %s. Paddlewheel: right -> %.0f s. Boat off its line max %.0f m",
             TS(n_ok), n_side < 0.0f ? "never" : "TRUSTED", n_along < 0.0f ? "never" : "TRUSTED", TS(g_ok),
             g_side < 0.0f ? "never" : "TRUSTED", g_along < 0.0f ? "never" : "TRUSTED", TS(p_ok), om);
#undef TS
    test_report("T3 GPS re-trust after a spoof at speed: ~60 s with a 2nd GPS / paddlewheel; drags never trusted",
                n_side < 0.0f && n_along < 0.0f && g_ok >= 59.0f && g_ok <= 75.0f && g_side < 0.0f && g_along < 0.0f &&
                p_ok >= 59.0f && p_ok <= 75.0f && om < 30.0f, d);
}

/* ------------------------------------ R9: arming needs the joystick seen -- */

/* Power-up with no joystick seen (analog not detected / USB bridge not yet
 * sending): the outputs stay unarmed (NOT ARMED alarm after 3 s) until the
 * joystick appears centred. A boat configured without a joystick
 * (cfg.joy_optional) arms without it. */
static void arm_joystick_tests(void)
{
    char d[500];
    printf("\n=== R9: outputs arm only with the joystick seen ===\n");
    static sbsim_t x;
    sbsim_init(&x, 281u, 0.0f, 0.0f, NULL);
    sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);   /* (bridge: no device frames) */
    sbsim_run(&x, 5000u, NULL, NULL);
    bool unarmed = !x.sb.armed, alarm = (x.sb.alarm_bits & SB_ALM_NOT_ARMED) != 0u;
    sbsim_init(&x, 282u, 0.0f, 0.0f, NULL);              /* joystick fitted and centred from the start */
    sbsim_run(&x, 3000u, NULL, NULL);
    bool armed_with = x.sb.armed;
    sb_config_t c;
    sb_default_config(&c);
    c.joy_optional = true;
    sbsim_init(&x, 283u, 0.0f, 0.0f, &c);
    sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);
    sbsim_run(&x, 3000u, NULL, NULL);
    bool armed_opt = x.sb.armed;
    snprintf(d, sizeof d, "no joystick seen for 5 s: armed %s, NOT ARMED alarm %s; joystick fitted + centred: armed %s; "
             "boat configured without a joystick: armed %s%s",
             unarmed ? "no" : "YES", alarm ? "yes" : "NO", armed_with ? "yes" : "NO", armed_opt ? "yes" : "NO",
             s_bridge ? " (USB bridge source: a missing stick is a JOYSTICK FAULT, joy_optional does not apply)" : "");
    test_report("R9 outputs arm only with the joystick seen and centred (unless configured without one)",
                unarmed && alarm && armed_with && (armed_opt || s_bridge), d);
}

/* ---------------------------------------------------------------- all -- */

void sb_usb_tests(void)
{
    usb_joystick_tests();
}

void sb_gateway_tests(void)
{
    gateway_tests();
    helm_tests();
    remote_tests();
    save_tests();
    ais_tests();
    nav_tests();
}

void sb_slam_tests(void)
{
    slam_tests();
}

void sb_tests(void)
{
    gateway_tests();
    helm_tests();
    remote_tests();
    save_tests();
    ais_tests();
    nav_tests();
    slam_tests();
    logging_tests();
    timing_tests();
    hardening_tests();
    powerlim_tests();
    modes_tests();
    link_tests();
    joystick_tests();
    usb_joystick_tests();
    gps_denied();
    fixes();
    phase2();
    phase3();
    centre_tests();
    gps_invalid_tests();
    gps_retrust_tests();
    arm_joystick_tests();
    phase4();
    phase5();
    phase6();
}
