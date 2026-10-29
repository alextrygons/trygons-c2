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
#include "nv.h"
#include "tc_nv.h"
#include "tc_modbus.h"
#include "dronecan.h"
#include "can_act.h"
#include "can_relay.h"
#include "tc_pulse.h"     /* (hal/: the paddlewheel pulses, tests PW) */
#include "tc_rstcause.h"

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

/* CN tests: the next sbsim_init boots with the simulated CAN bus fitted (+ these sim faults) */
static bool     s_can_on;
static uint32_t s_can_fault;
/* step 11f: the next sbsim_init boots with the relay board on DroneCAN (the CAN bus fitted, the relay node 30
 * driven by hal/can_relay.c). "all --can" / "--can-fast" always use the DroneCAN relay board. */
static bool     s_rb_can;

/* TELEM2: the tests that read groups L / M from the end of the frame skip the
 * groups appended after them: O (engine controls, 12 bytes) + N (DP thrusters 10 bytes + the DP state
 * T2_DP bytes) + P (DroneCAN summary T2_P bytes) = T2_N */
#define T2_DP 12u
#define T2_R 3u                 /* group R (CONFIRM_ONLY, academy practical test, step 21) after Q */
#define T2_Q (17u + T2_R)       /* group Q (steering position servo + throttle model, step 11e) after P */
#define T2_P (6u + T2_Q)        /* group P (DroneCAN summary, step 11d) after N: its offset from the end */
#define T2_N (10u + T2_DP + T2_P)
#define T2_O (12u + T2_N)

/* "all --bridge": every test drives the boat through the USB joystick
 * bridge (simulated Teensy frames) instead of the analog joystick. */
static bool s_bridge;
void sb_set_bridge(bool on) { s_bridge = on; }

/* "all --can" / "all --can-fast" (step 11e): every test on the DroneCAN servos (the simulated bus with the
 * throttle / steering / gear nodes; slow = the worst-case speeds 6 s / 12 s, fast = 2 s / 3 s). The tests of
 * the old hardware only (steering motor pulses, end stops, motor current, the DC-motor TRAINING, FIRST TEST)
 * report N/A there. */
static bool s_can_all, s_can_fast;
void sb_set_can(bool on, bool fast) { s_can_all = on; s_can_fast = fast; }

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
    remove("nv5.bin");                              /* no TRAINING result (a DC-motor v1 one would be refused on the
                                                       DroneCAN servo: NV DATA RESET in every "--can" test) */
    remove("nv7.bin");                              /* no relay table */
    sim_hal_relay_board(true, 5u, 0u);              /* a fresh relay board, every relay off */
    sim_hal_relay_board_reset();
    sim_hal_relay_fitted(16u);                      /* (taken by hal_init below) */
    sim_hal_relay_bus((s_can_all || s_rb_can) ? SIM_RELAY_DRONECAN : SIM_RELAY_MODBUS);   /* step 11f */
    sim_hal_starter_limit(2u, 6000u);
    sim_hal_unforce();
    memset((void *)&g_mon, 0, sizeof g_mon);
    world_publish(&x->w, 0);
    sim_hal_can_reset();                            /* DroneCAN: every node fresh; not fitted unless a CN test */
    sim_hal_can_fault(s_can_fault);
    sim_hal_can_fitted(s_can_on || s_can_all || s_rb_can);
    sb_test_boot_idle(s_can_all && !s_can_on);      /* (the CN tests: the real boot order) */
    if (s_can_fast) {
        sim_hal_can_servo_speed(DC_NODE_THROTTLE, 2.0f, 0.0f);
        sim_hal_can_servo_speed(DC_NODE_STEERING, 3.0f, 0.03f);
    }
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
    x->mon.boot_known = s_can_all && !s_can_on;     /* (as sb_test_boot_idle above) */
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
            x->mon.boot_known = s_can_all && !s_can_on;
            if (x->log_on) logcore0_init(&x->sb, g_logstat.boot);
            x->next_ctrl = x->t;
            x->last_kick = x->t;
        }
        /* the 10 ms actuator task of core 0 (DroneCAN servos; frozen / booting: it does not run) */
        if (!x->boot_until && !x->frozen && x->t % 10u == 0u) hal_can_tick();
        if (!x->boot_until && x->t >= x->next_ctrl) {
            x->next_ctrl += AP_PERIOD_MS;
            if (!x->frozen) {
                uint32_t t0 = hal_micros();
                sb_step(&x->sb);
                if (x->log_on) logcore0_step(&x->sb);
                (void)cyc_record(&g_timing.core0, hal_micros() - t0 + x->inject_us);
                x->last_kick = x->t;                /* hal_watchdog_kick() in the loop */
                if (hook) hook(x, ctx);
                if (sim_hal_reset_requested()) {    /* hal_system_reset(): the remote restart after a trip */
                    sim_hal_chip_reset(HAL_RESET_APPLICATION);
                    x->boot_until = x->t + 200u;
                }
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
    float    t_true_idle;   /* the REAL (simulated) throttle servo first <= 1 % (step 11e) */
    float    t_steer_off;   /* autopilot no longer steering */
    uint16_t want_pwm;
    int      steer_after_off;  /* steering motor commands after steering off (position servo: command changes) */
    float    steer_last;
    bool     armed;
} events_t;

static void ev_reset(events_t *e, uint32_t t0, float thr, uint16_t want_pwm)
{
    e->t0 = t0;
    e->thr_before = thr;
    e->t_thr_down = e->t_idle = e->t_neutral = e->t_gear = e->t_thr_open = e->t_steer_off = e->t_true_idle = -1.0f;
    e->want_pwm = want_pwm;
    e->steer_after_off = 0;
    e->steer_last = sim_hal_steer_cmd(NULL);
    e->armed = true;
}

static void ev_watch(sbsim_t *x, void *ctx)
{
    events_t *e = (events_t *)ctx;
    if (!e->armed || x->t < e->t0) return;
    bool was_off = e->t_steer_off >= 0.0f;
    float ts = (float)(x->t - e->t0) * 0.001f;
    float thr = sim_hal_throttle();
    uint16_t pwm = sim_hal_gear_pwm();
    if (e->t_thr_down < 0.0f && thr < e->thr_before - 0.01f) e->t_thr_down = ts;
    if (e->t_idle < 0.0f && thr <= 1.0f) e->t_idle = ts;
    if (e->t_true_idle < 0.0f && sim_hal_thr_truth() <= 1.0f) e->t_true_idle = ts;
    if (e->t_neutral < 0.0f && pwm == x->sb.cfg.gear.pwm_n_us) e->t_neutral = ts;
    if (e->t_gear < 0.0f && e->t_neutral >= 0.0f && pwm == e->want_pwm) e->t_gear = ts;
    if (e->t_thr_open < 0.0f && e->t_gear >= 0.0f && thr > 1.0f) e->t_thr_open = ts;
    if (e->t_steer_off < 0.0f && x->sb.ap.mode == AP_MODE_STANDBY) e->t_steer_off = ts;
    if (e->t_steer_off >= 0.0f && sim_hal_motor_command() != 0.0f) e->steer_after_off++;
    /* the position servo: its command must not change after the step that turned the steering off (hold) */
    float sc = sim_hal_steer_cmd(NULL);
    if (was_off && sim_hal_servos_can() && fabsf(sc - e->steer_last) > 1e-4f) e->steer_after_off++;
    e->steer_last = sc;
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
        /* step 11e: idle COMMANDED at once (no rate limit); neutral only after the MODELLED servo is at idle
         * (thr0 / 100 x thr_travel_s) + the idle wait; the real (simulated) servo is idle within thr_travel_s */
        float model_idle = thr0 * 0.01f * x.sb.servo.cfg.travel_s;
        snprintf(d, sizeof d, "at %.1f kn, throttle %.0f %%: throttle down after %.2f s, idle commanded %.2f s, real "
                 "servo idle %.2f s (travel %.0f s), neutral %.2f s (model idle %.1f s + wait), steering off %.2f s "
                 "(held heading within %.1f deg); under-load shifts %d; log: %s->%s (%s)",
                 v0, thr0, ev.t_thr_down, ev.t_idle, ev.t_true_idle, x.sb.servo.cfg.travel_s, ev.t_neutral, model_idle,
                 ev.t_steer_off, hmax, x.w.shift_under_load, mode_name(le->from), mode_name(le->to),
                 mode_reason_name(le->why));
        test_report("P2 STANDBY at full throttle: idle commanded at once, neutral after the modelled idle, then steering off",
                    order && thr0 > 90.0f && ev.t_idle <= 0.1f && ev.t_true_idle >= 0.0f &&
                    ev.t_true_idle <= x.sb.servo.cfg.travel_s + 0.1f &&
                    ev.t_neutral >= model_idle * 0.99f + x.sb.cfg.gear.idle_wait_ms * 0.001f - 0.1f &&
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
        float steer_prev = sim_hal_steer_cmd(NULL);
        while (x.t < s.t0 + 40000u) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, script_hook, &s);
            ev_watch(&x, &ev);
            if (x.sb.ap.mode == AP_MODE_ANGLE) {
                if (x.sb.ap.rudder_target < tgt_min) tgt_min = x.sb.ap.rudder_target;
                if (sim_hal_motor_command() < 0.0f) motor_left++;
                if (sim_hal_servos_can() && sim_hal_steer_cmd(NULL) < steer_prev - 1e-4f) motor_left++;  /* the servo */
            }
            steer_prev = sim_hal_steer_cmd(NULL);
            if (ev.t_neutral >= 0.0f && ev.t_steer_off < 0.0f) steer_on_at_neutral = true;
            if (sb_mode(&x.sb) == MODE_MANUAL) manual_again = true;
        }
        snprintf(d, sizeof d, "throttle %.0f %% -> idle commanded after %.2f s (at once), neutral %.2f s (after the "
                 "modelled idle), steering off %.2f s; outboard command at STANDBY %+.1f deg, followed the stick to "
                 "%+.1f deg during the stop (motor %d steps left); motor after off %d, under-load %d, held stick took "
                 "control %s",
                 thr0, ev.t_idle, ev.t_neutral, ev.t_steer_off, tgt_press, tgt_min,
                 motor_left, ev.steer_after_off, x.w.shift_under_load, manual_again ? "YES" : "no");
        test_report("P3 R1 STANDBY in MANUAL: the stick steers until idle + neutral (the modelled idle), then steering off",
                    was_manual && thr0 > 20.0f && ev.t_idle >= 0.0f && ev.t_idle <= 0.1f &&
                    ev.t_neutral >= thr0 * 0.01f * x.sb.servo.cfg.travel_s &&
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
static bool s_cruise_paddle;    /* the next cruise_run with a paddlewheel (none since step 13: see P4.2) */
static void cruise_run(sbsim_t *x, cruise_watch_t *c, uint32_t seed, const float *spd, int n_legs, float leg_s,
                       const float *at, const float *sea, int n_sea, uint32_t dur_ms)
{
    prog_wp_t wps[16];
    sbsim_init(x, seed, 90.0f, 0.0f, NULL);
    x->w.speed_sensor = s_cruise_paddle;
    s_cruise_paddle = false;
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
                 "throttle rate max %.0f %%/s; gear changes after F %d; legs done %u; GPS suspects %u",
                 c.limit_excess, c.true_excess, x.w.max_thr_rate, c.gear_changes - 1, x.sb.autop.legs_done,
                 (unsigned)x.sb.dr.suspects);
        ok = ok && c.limit_excess <= 0.01f && c.true_excess <= 0.0f &&
             x.w.max_thr_rate <= x.sb.cfg.throttle.rate_down_pct_s + 0.5f &&     /* (step 11e: 50 %/s down) */
             c.gear_changes == 1 && x.w.shift_under_load == 0 && x.sb.autop.legs_done >= 4u &&
             x.sb.dr.suspects == 0u;
        test_report("P4 AUTO legs 3->20 kn: speed control per leg, steering limit by speed", ok, d);
    }

    /* 4.2 sea state 1 -> 4 at 18 kn, then calm again */
    {
        static const float at[]  = { 0, 180, 360, 540, 1140 };
        static const float spd[] = { 18 };
        static const float sea[] = { 1, 2, 3, 4, 1 };
        sbsim_t x;
        cruise_watch_t c;
        /* WITHOUT a paddlewheel again (step 13, W18): in steps 11e-11g it ran with one, because the dead
         * reckoning's slow-drag check (dr.c, 300 s / 60 m) raised a false GPS SUSPECT after the long
         * acceleration back to 18 kn: what DR learned in the 50 %/s slow-downs leaked into its current along
         * the heading. dr.c now pauses that learning while the throttle comes down fast (learn_thr_dev_pct) */
        cruise_run(&x, &c, 42u, spd, 1, 0.0f, at, sea, 5, 2100000u);
        snprintf(d, sizeof d, "speed by sea state: ss1 %.1f-%.1f, ss2 %.1f-%.1f, ss3 %.1f-%.1f, ss4 %.1f-%.1f, "
                 "calm again %.1f-%.1f kn (end %.1f); course error max ss3 %.1f, ss4 %.1f deg; "
                 "rough-water episodes %u, changes %u, shortest gap %.0f s; no paddlewheel: GPS suspects %u",
                 c.seg_vmin[0], c.seg_vmax[0], c.seg_vmin[1], c.seg_vmax[1], c.seg_vmin[2], c.seg_vmax[2],
                 c.seg_vmin[3], c.seg_vmax[3], c.seg_vmin[4], c.seg_vmax[4], world_sog_kn(&x.w),
                 c.seg_hold[2], c.seg_hold[3], x.sb.nerv.episodes, x.sb.nerv.transitions,
                 c.min_gap_ms * 0.001f, (unsigned)x.sb.dr.suspects);
        bool ok = c.seg_vmin[0] > 17.0f && c.seg_vmin[1] > 17.0f &&         /* no reaction in ss1-2 */
                  x.sb.nerv.episodes >= 1u && c.seg_vmin[3] <= 14.5f &&     /* slowed down in ss4 */
                  world_sog_kn(&x.w) > 17.0f &&                               /* recovered */
                  x.sb.nerv.transitions <= 4u && auto_running(&x) &&
                  x.sb.dr.suspects == 0u && x.sb.dr.state == DR_GPS;         /* W18: no false GPS SUSPECT */
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
                 "shortest time between changes %.0f s, speed %.1f-%.1f kn; GPS suspects %u (W18: 0)",
                 x.sb.nerv.episodes, x.sb.nerv.transitions, c.min_gap_ms * 0.001f, c.seg_vmin[0], c.seg_vmax[0],
                 (unsigned)x.sb.dr.suspects);
        bool ok = x.sb.nerv.transitions <= 6u &&
                  (x.sb.nerv.transitions < 2u || c.min_gap_ms >= 30000u) && x.sb.dr.suspects == 0u;
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
        /* DroneCAN position servo without TRAINING (step 11e, changed test): the model steers at the worst-case
         * speed (12 s lock to lock, TO CONFIRM C10) - the line capture after the 45 deg turn of the short 500 m
         * leg is slower (PWM rms 3.6 m, worst-case servo 8.3 m: finding in PROGRESS.md); max unchanged */
        float rms_max = (sim_hal_servos_can() && i == 0u) ? 9.0f : 6.0f;
        bool ok = w.xte_n > 0 && w.xte_max <= cases[i].xmax && rms <= rms_max &&
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
    /* the DroneCAN position servo has its own TRAINING (trainsrv.c, tests SM): the DC-motor runs are N/A */
    int n_boats = s_can_all ? 0 : 5;

    for (int i = 0; i < n_boats; i++) {
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
    if (s_can_all) test_na("P6 training 5 random boats x sea state 1 + 2: learned vs true", "DC-motor TRAINING (servo: SM1)");
    else test_report("P6 training 5 random boats x sea state 1 + 2: learned vs true", trained == total &&
                accurate == total && beyond_stop == 0, d);
    snprintf(d, sizeof d, "worst course error learned %.1f deg vs true-value %.1f; overshoot %.1f vs %.1f; "
             "learned clearly worse in %d of %d", hold_l_max, hold_t_max, over_l_max, over_t_max, worse, trained);
    if (s_can_all) test_na("P6 course keeping with learned values as good as with true values", "DC-motor TRAINING (servo: SM7)");
    else test_report("P6 course keeping with learned values as good as with true values",
                trained == total && worse == 0 && over_l_max < 10.0f, d);
    snprintf(d, sizeof d, "throttle constant at %.0f %% while calibrating: %s, forward gear the whole time: %s, "
             "max heading change in a phase %.0f deg (limit %.0f), end-stop push max %.1f s total",
             sets[0].speed > 0 ? 20.0f : 0.0f, throttle_fixed ? "yes" : "NO", gear_ok ? "yes" : "NO",
             hdg_max, TR_MAX_PHASE_HEADING_DEG, push_max);
    if (s_can_all) test_na("P6 training runs at fixed slow throttle in forward gear, within limits", "DC-motor TRAINING");
    else test_report("P6 training runs at fixed slow throttle in forward gear, within limits",
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
    if (s_can_all) {
        test_na("P6 limits / implausible results: training aborts into a safe stop", "DC-motor TRAINING (servo: SM2)");
        test_na("P6 confirmation: reject keeps settings, accept applies + saves", "DC-motor TRAINING (servo: SM6)");
    } else {
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
    if (!s_can_all) {
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

    /* Fix 4: motor current / limit switches stop the push at the end stop (the steering MOTOR of the old
     * hardware: N/A on the DroneCAN position servo - no end-stop push there) */
    if (s_can_all) {
        test_na("F4 training: end-stop push time well below before (current / switch)", "steering motor only");
        test_na("F4 normal steering: stops pushing as soon as the current rises", "steering motor only");
    } else {
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
        printf("  setting CENTER (no gyro hold):  %6.0f deg   outboard at fault %.1f, at end %.1f deg\n",
               b.hdg_unwrap, b.angle_at_fault, b.angle_end);
        printf("  after, gyro hold:              %6.0f deg   idle %.1f s, neutral %.1f s, steering off %.1f s, outboard at end %.1f deg\n",
               g.hdg_unwrap, g.t_idle, g.t_neutral, g.t_off, g.angle_end);
        printf("  after, gyro failed too:        %6.0f deg   centred and off at %.1f s, outboard at end %.1f deg\n",
               c.hdg_unwrap, c.t_off, c.angle_end);
        snprintf(d, sizeof d, "heading change with centring %.0f deg -> gyro hold %.0f deg; steering off at %.1f s "
                 "(neutral %.1f s); motor after off %d",
                 b.hdg_unwrap, g.hdg_unwrap, g.t_off, g.t_neutral, g.motor_after_off);
        test_report("F1 compass lost at 20 kn: gyro hold until idle + neutral, then off",
                    fabsf(g.hdg_unwrap) <= fabsf(b.hdg_unwrap) + 1.0f && g.t_off >= g.t_neutral &&
                    g.t_neutral > g.t_idle && g.motor_after_off == 0, d);
        snprintf(d, sizeof d, "compass + gyro lost: centring, heading change %.0f deg; outboard at end %.1f deg "
                 "(setting CENTER: %.1f deg); steering off at %.1f s (neutral %.1f s); motor after off %d",
                 c.hdg_unwrap, c.angle_end, b.angle_end, c.t_off, c.t_neutral, c.motor_after_off);
        test_report("F1 compass and gyro lost at 20 kn: outboard to centre, then off",
                    fabsf(c.angle_end) < 3.0f && fabsf(b.angle_end) < 3.0f && c.t_off >= c.t_neutral &&
                    c.motor_after_off == 0, d);
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
    /* (30 s: the DroneCAN throttle servo's own travel + the modelled idle before the shift, step 11e) */
    sbsim_run(&x, x.t + 30000u, NULL, NULL);
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
        /* stored with checksum (the save is queued: written by the next control
         * step in STANDBY): survives a restart, a corrupted copy is not loaded */
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        sbsim_t y;
        sb_config_t c;
        sb_default_config(&c);
        memset(&y, 0, sizeof y);
        world_init(&y.w, 205u, 90.0f, 0.0f);
        hal_init();                                  /* power-off: queued saves are gone, the flash stays */
        sb_init(&y.sb, &c);
        bool survived = y.sb.autop.prog_ok && y.sb.autop.prog.n == 5u;
        FILE *f = fopen("nv2.bin", "r+b");
        if (f) { fseek(f, 40, SEEK_SET); fputc(0x5A, f); fclose(f); }
        hal_init();
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
    uint8_t     t2[PROTO_MAX_PAYLOAD];              /* last TELEM2 */
    uint16_t    t2_len;
    uint8_t     can[PROTO_MAX_PAYLOAD];             /* last CAN_NODES */
    uint16_t    can_len;
    int         can_frames;
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
    /* parse after every read (the receive ring holds 512 bytes: TELEM2 + TELEMETRY + CAN_NODES in one step
     * are more than that) */
    while ((n = sim_hal_link_tx_pop(buf, 128u)) > 0u) {
        proto_rx_push(&p->pc_rx, buf, n);
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
            } else if (f.type == PROTO_T_TELEM2) {
                memcpy(p->t2, f.payload, f.len);
                p->t2_len = f.len;
            } else if (f.type == PROTO_T_CAN_NODES) {
                memcpy(p->can, f.payload, f.len);
                p->can_len = f.len;
                p->can_frames++;
            }
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
        /* (DroneCAN servos: the actuator task stops with core 0 - MON_TRIP_CAN_TX comes first, after 100 ms) */
        bool why_ok = s_can_all ? (r.why == MON_TRIP_CAN_TX && r.t_react >= (float)MON_CAN_TX_MS - 20.0f &&
                                   r.t_react <= (float)MON_HB_MS + 30.0f)
                                : (r.why == MON_TRIP_HEARTBEAT && r.t_react >= (float)MON_HB_MS &&
                                   r.t_react <= (float)MON_HB_MS + 30.0f);
        test_report("C3 frozen core 0 (no watchdog): monitor heartbeat trip -> idle + neutral + alarm",
                    why_ok && r.thr_after <= 0.0f && r.gear_after == x.sb.cfg.gear.pwm_n_us && r.alarm, d);
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

    /* C3d R22: the monitor never checks a half-written set (seqlock): a throttle
     *     above the limit while core 0 is writing is not judged; once the set is
     *     complete and still wrong, it trips */
    {
        static mon_shared_t sh;
        mon_t m;
        memset((void *)&sh, 0, sizeof sh);
        sh.thr_idle_us = 1000u; sh.gear_n_us = 1500u; sh.gear_pwm_us = 1500u;
        sh.fwd_limit = 30.0f; sh.rev_limit = 5.0f; sh.fwd_engaged = 1u; sh.idle_max_pct = 1.0f;
        sh.running = MON_RUNNING;
        monitor_init(&m, 0u);
        uint32_t t = 0u;
        for (; t < 100u; t++) { if (t % 50u == 0u) sh.heartbeat++; monitor_step(&m, &sh, t); }
        monitor_write_begin(&sh);                       /* core 0 in the middle of writing ... */
        sh.throttle_pct = 80.0f;                        /* ... the new throttle, the limit not yet */
        for (; t < 120u; t++) monitor_step(&m, &sh, t);
        bool no_trip_torn = !m.tripped && m.torn >= 20u;
        monitor_write_end(&sh);                         /* complete - and really above the limit */
        for (; t < 140u && !m.tripped; t++) monitor_step(&m, &sh, t);
        snprintf(d, sizeof d, "throttle 80 %% over a 30 %% limit while core 0 writes: tripped %s (skipped %lu checks); "
                 "set complete: %s after %lu ms", no_trip_torn ? "no" : "YES", (unsigned long)m.torn,
                 m.tripped ? monitor_reason_name(m.reason) : "NO TRIP", (unsigned long)(t - 120u));
        test_report("C3d monitor checks only complete sets (seqlock): no false trip while core 0 writes",
                    no_trip_torn && m.tripped && m.reason == MON_TRIP_THROTTLE, d);
        hal_init();
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

    /* F0 R16: a corrupted record length in a log queue: the consumer resyncs,
     *     the queue goes on working (it was broken for good before) */
    {
        static uint8_t qb[1024], rec[64], out[256];
        logq_t q;
        logq_init(&q, qb, sizeof qb);
        for (int i = 0; i < 64; i++) rec[i] = (uint8_t)i;
        for (int i = 0; i < 5; i++) logq_push(&q, rec, 40u);
        qb[(q.tail + 1u) % q.size] = 0x7Fu;              /* length of the oldest record -> 32 K */
        uint16_t got = logq_pop(&q, out, sizeof out);
        bool resynced = got == 0u && q.resyncs == 1u && logq_used(&q) == 0u;
        int ok_after = 0;
        for (int i = 0; i < 100; i++) {                  /* 100 more records through a 1 KB queue */
            if (logq_push(&q, rec, 40u) && logq_pop(&q, out, sizeof out) == 40u && out[39] == 39u) ok_after++;
        }
        snprintf(d, sizeof d, "length byte corrupted: pop returned %u, resyncs %lu, queue empty %s; afterwards %d of "
                 "100 records pushed + popped correctly, dropped %lu",
                 (unsigned)got, (unsigned long)q.resyncs, logq_used(&q) == 0u ? "yes" : "NO", ok_after,
                 (unsigned long)q.dropped);
        test_report("F0 log queue: a corrupted record length is resynced, the queue keeps working",
                    resynced && ok_after == 100 && q.dropped == 0u, d);
    }

    /* F1b R10: the logger services CPU2's own watchdog after every SD card call
     *     (several slow calls in one step can't add up to a chip reset) */
    {
        sbsim_t x;
        sbsim_init(&x, 609u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        uint32_t ops0 = sim_hal_sd_ops(), kicks0 = sim_hal_core_kicks();
        sbsim_log_start(&x, "sd_f1b", 1u);
        hdg_lock(&x);
        sim_hal_fail_heading(true);                     /* a fault -> black box file too (more SD calls) */
        sbsim_run(&x, x.t + 120000u, NULL, NULL);
        sim_hal_fail_heading(false);
        logger_close(&s_logger);
        x.log_on = false;
        uint32_t ops = sim_hal_sd_ops() - ops0, kicks = sim_hal_core_kicks() - kicks0;
        snprintf(d, sizeof d, "2 min of logging with a black-box file: %lu SD card calls, CPU2 watchdog serviced after "
                 "%lu of them (HAL contract: each call <= %u ms)", (unsigned long)ops, (unsigned long)kicks, HAL_SD_MAX_MS);
        test_report("F1b logger services the CPU2 watchdog after every SD card call", ops > 100u && kicks == ops, d);
    }

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
        /* never seen: the self-check fails (timeout) -> UNCREWED, the joystick is ignored */
        bool dead = y.sb.joy_probe.fault && y.sb.joy_probe.fault_axis == JOY_WHY_TIMEOUT &&
                    y.sb.profile == SB_PROFILE_UNCREWED && !y.sb.joy.present;
        snprintf(d, sizeof d, "USB enumeration (3 s): no fault, no alarm, not armed %s; joysticks up: armed %s (CREWED %s); "
                 "bridge silent after power-up: self-check fault (timeout) -> UNCREWED, joystick ignored %s",
                 quiet ? "yes" : "NO", up ? "yes" : "NO", x.sb.profile == SB_PROFILE_CREWED ? "yes" : "NO",
                 dead ? "yes" : "NO");
        test_report("U14 bridge start-up: quiet while USB enumerates; never comes -> UNCREWED", quiet && up && dead &&
                    x.sb.profile == SB_PROFILE_CREWED, d);
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
    bool       shore_on;        /* the Pi sends GW_SHORE with its status (lost-link rule) */
    uint8_t    shore_flags;
    uint16_t   shore_m;
    uint32_t   ev_restart;      /* log event RESTART records in the Pi's copy, the last one's action, when */
    int32_t    ev_restart_a;
    uint32_t   ev_restart_t;
    uint32_t   ev_reset_t;      /* first RESTART event with action 2 (reset coming) */
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
            else if (L >= LOGR_HDR + 6u && p->cur[1] == LOGR_EVENT && (p->cur[8] | (p->cur[9] << 8)) == EV_RESTART) {
                p->ev_restart++;
                p->ev_restart_a = (int32_t)(p->cur[10] | (p->cur[11] << 8) | (p->cur[12] << 16) | ((uint32_t)p->cur[13] << 24));
                p->ev_restart_t = x->t;
                if (p->ev_restart_a == (int32_t)ENG_LOG_RESET && !p->ev_reset_t) p->ev_reset_t = x->t;
            }
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
        if (p->shore_on) {
            uint8_t sp[3] = { p->shore_flags, (uint8_t)p->shore_m, (uint8_t)(p->shore_m >> 8) };
            n = proto_encode(fr, PROTO_T_GW_SHORE, 0u, sp, sizeof sp);
            sim_hal_gw_rx_push(fr, (uint32_t)n);
        }
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
        pisim_init(&p, &x, 76800u);
        sbsim_run(&x, 2000u, pi_hook, &p);
        hdg_lock(&x);
        sbsim_run(&x, x.t + 500u, pi_hook, &p);
        p.max_telem_gap_ms = 0u;
        p.last_telem_ms = x.t;
        sbsim_run(&x, 60000u, pi_hook, &p);
        bool sd_ok = !g_logstat.failure && g_logq0.dropped == 0u;
        snprintf(d, sizeof d, "76800 baud (TELEM2 grew with the DP state, step 10): telemetry frames %d of 600, none lost (max gap %lu ms at %.1f s: 10 Hz + at most one 50 ms step late), TIME %d; log records to the Pi %lu, "
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
    bool     conf;              /* the shore joystick's CONFIRM held (flags b2) */
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
    if (r->rj_on) rj_send(r, r->conf ? 4u : 0u);
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

/* ------------------- CREWED / UNCREWED profile, MANEUVER (CLAUDE.md A + C) -- */

/* Power-up with (crewed) or without (uncrewed) the onboard joystick, a Pi on
 * the gateway UART, station 2's remote joystick ready (not sending yet). */
static void crew_init(sbsim_t *x, rjsim_t *r, uint32_t seed, bool crewed, const sb_config_t *cfg)
{
    memset(r, 0, sizeof *r);
    sbsim_init(x, seed, 90.0f, 0.0f, cfg);
    if (!crewed) {
        sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);
        if (s_bridge) sim_hal_bridge_fault(SIM_BR_STOP);
    }
    pisim_init(&r->p, x, 921600u);
    r->st = 2u;
    r->lever = 0.5f;
    r->has_lever = true;
    r->age_ms = 60u;
    r->lat_ms = 40u;
    sbsim_run(x, 7000u, rj_hook, r);
}

/* the onboard joystick plugged in (centred) / back to normal after a test */
static void crew_plug(void)
{
    sim_hal_bridge_fault(0u);
    joy_set(0.0f, 0.0f, false, false);
}

/* CONFIRM on the shore captain's joystick (0.15 s). True if accepted. */
static bool shore_confirm(sbsim_t *x, rjsim_t *r)
{
    uint32_t before = x->sb.confirm.accepted;
    r->conf = true;
    sbsim_run(x, x->t + 150u, rj_hook, r);
    r->conf = false;
    sbsim_run(x, x->t + 100u, rj_hook, r);
    return x->sb.confirm.accepted > before;
}

static bool onboard_confirm(sbsim_t *x, rjsim_t *r)
{
    uint32_t before = x->sb.confirm.accepted;
    if (s_usb.on) { s_usb.b1 |= 8u; usb_send(); }
    else sim_hal_joystick_confirm(true);
    sbsim_run(x, x->t + 120u, rj_hook, r);
    if (s_usb.on) { s_usb.b1 &= ~8u; usb_send(); }
    else sim_hal_joystick_confirm(false);
    sbsim_run(x, x->t + 100u, rj_hook, r);
    return x->sb.confirm.accepted > before;
}

static void crew_tests(void)
{
    char d[900];
    const uint8_t TAKE = 0u, ASK = 0u;
    printf("\n=== CREWED / UNCREWED profile, MANEUVER ===\n");

    /* PR1 autodetection at power-up (and FIRST TEST always CREWED) */
    {
        static sbsim_t x;
        static rjsim_t r;
        crew_init(&x, &r, 1201u, true, NULL);
        bool crewed = x.sb.profile == SB_PROFILE_CREWED && x.sb.armed;
        sbsim_init(&x, 1202u, 90.0f, 0.0f, NULL);
        sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);
        if (s_bridge) sim_hal_bridge_fault(SIM_BR_STOP);
        sbsim_run(&x, 5000u, NULL, NULL);
        bool detecting = x.sb.profile == SB_PROFILE_DETECTING && !x.sb.armed;
        sbsim_run(&x, 7000u, NULL, NULL);
        bool unc = x.sb.profile == SB_PROFILE_UNCREWED && x.sb.armed && !x.sb.joy.present;
        sb_config_t c;
        sb_default_config(&c);
        c.first_test = true;
        sbsim_init(&x, 1203u, 90.0f, 0.0f, &c);
        sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);
        sbsim_run(&x, 7000u, NULL, NULL);
        bool ft = x.sb.profile == SB_PROFILE_CREWED;
        crew_plug();
        snprintf(d, sizeof d, "joystick at power-up: CREWED + armed %s; none: still detecting + not armed at 5 s %s, "
                 "UNCREWED + armed (joystick ignored) at 7 s %s; FIRST TEST without a joystick: CREWED %s",
                 crewed ? "yes" : "NO", detecting ? "yes" : "NO", unc ? "yes" : "NO", ft ? "yes" : "NO");
        test_report("PR1 profile autodetected at power-up: joystick -> CREWED, none -> UNCREWED; FIRST TEST CREWED",
                    crewed && detecting && unc && ft, d);
    }

    /* PR2 UNCREWED: the onboard joystick has no effect; STANDBY button + e-stop still work */
    {
        static sbsim_t x;
        static rjsim_t r;
        crew_init(&x, &r, 1204u, false, NULL);
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        r.rj_on = true;
        sbsim_run(&x, x.t + 500u, rj_hook, &r);
        press(&x, HAL_BTN_HDG_LOCK);
        bool no_hdg = sb_mode(&x.sb) == MODE_STANDBY;
        uint8_t ask = pi_cmd(&x, &r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        sim_hal_joystick_confirm(true);                    /* someone presses the onboard CONFIRM: nothing */
        sbsim_run(&x, x.t + 200u, rj_hook, &r);
        sim_hal_joystick_confirm(false);
        sbsim_run(&x, x.t + 200u, rj_hook, &r);
        bool not_onboard = sb_mode(&x.sb) == MODE_STANDBY && x.sb.confirm.kind == CONF_REMOTE;
        bool sc = shore_confirm(&x, &r);
        bool remote = sb_mode(&x.sb) == MODE_REMOTE;
        sbsim_run(&x, x.t + 2000u, rj_hook, &r);
        /* a joystick appears on board while driving and is pushed hard: never MANUAL, a profile fault */
        crew_plug();                                       /* (USB: a latched fault clears after 2 s centred) */
        bool manual = false;
        uint32_t t0 = x.t;
        while (x.t < t0 + 4000u) {
            if (x.t >= t0 + 150u) joy_set(0.9f, 0.9f, true, false);
            if (s_bridge && x.t < t0 + 2500u) joy_set(0.0f, 0.0f, false, false);
            sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r);
            if (sb_mode(&x.sb) == MODE_MANUAL) manual = true;
        }
        bool fault = sb_mode(&x.sb) == MODE_STANDBY && (x.sb.alarm_bits & SB_ALM_PROFILE) &&
                     auto_end_why(&x) == WHY_PROFILE && x.sb.profile == SB_PROFILE_UNCREWED;
        /* the STANDBY button in REMOTE */
        static sbsim_t y;
        static rjsim_t q;
        crew_init(&y, &q, 1205u, false, NULL);
        pi_cmd(&y, &q.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        q.rj_on = true;
        sbsim_run(&y, y.t + 300u, rj_hook, &q);
        pi_cmd(&y, &q.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        shore_confirm(&y, &q);
        bool rem2 = sb_mode(&y.sb) == MODE_REMOTE;
        press(&y, HAL_BTN_STANDBY);
        bool sbb = sb_mode(&y.sb) == MODE_STANDBY;
        crew_plug();
        snprintf(d, sizeof d, "UNCREWED: panel HDG LOCK ignored %s; REMOTE asked %u, onboard CONFIRM ignored %s, shore "
                 "CONFIRM %s -> REMOTE %s; joystick plugged in + pushed while driving: MANUAL %s, STANDBY + PROFILE alarm "
                 "(why %s) %s, still UNCREWED; STANDBY button in REMOTE %s -> %s",
                 no_hdg ? "yes" : "NO", ask, not_onboard ? "yes" : "NO", sc ? "accepted" : "NO", remote ? "yes" : "NO",
                 manual ? "YES" : "never", mode_reason_name(auto_end_why(&x)), fault ? "yes" : "NO",
                 rem2 ? "yes" : "NO", mode_name(sb_mode(&y.sb)));
        test_report("PR2 UNCREWED: the onboard joystick (stick, CONFIRM, buttons) has no effect; STANDBY button wins",
                    no_hdg && ask == PROTO_R_OK && not_onboard && sc && remote && !manual && fault && rem2 && sbb, d);
    }

    /* PR3 UNCREWED power raise: code + the shore captain's CONFIRM, same rules */
    {
        static sbsim_t x;
        static rjsim_t r;
        crew_init(&x, &r, 1206u, false, NULL);
        uint16_t code = 0u;
        sb_result_t no_shore = sb_power_code(&x.sb, 0u, &code);     /* no shore joystick yet */
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        r.rj_on = true;
        sbsim_run(&x, x.t + 500u, rj_hook, &r);
        sb_alarm_ack(&x.sb);
        (void)sb_power_code(&x.sb, 0u, &code);
        sb_result_t rq = sb_power_raise(&x.sb, 0u, code);
        bool onb = onboard_confirm(&x, &r);
        bool c50 = shore_confirm(&x, &r) && x.sb.plim.fwd_pct == 50.0f;
        (void)sb_power_code(&x.sb, 0u, &code);
        (void)sb_power_raise(&x.sb, 0u, code);
        sbsim_run(&x, x.t + 10500u, rj_hook, &r);                  /* too late */
        bool late = !shore_confirm(&x, &r) && x.sb.plim.fwd_pct == 50.0f;
        (void)sb_power_code(&x.sb, 0u, &code);
        (void)sb_power_raise(&x.sb, 0u, code);
        r.x = 0.6f;                                                /* shore stick held */
        sbsim_run(&x, x.t + 200u, rj_hook, &r);
        bool held = !shore_confirm(&x, &r) && x.sb.plim.fwd_pct == 50.0f;
        r.x = 0.0f;
        (void)sb_power_code(&x.sb, 0u, &code);
        (void)sb_power_raise(&x.sb, 0u, code);
        bool c75 = shore_confirm(&x, &r) && x.sb.plim.fwd_pct == 75.0f;
        press(&x, HAL_BTN_STANDBY);
        bool drop = x.sb.plim.fwd_pct == 30.0f;
        crew_plug();
        snprintf(d, sizeof d, "without shore joystick: code %d (2 = not ready); raise asked %d; onboard CONFIRM %s; "
                 "shore CONFIRM -> 50 %% %s; late (10.5 s) refused %s; shore stick held refused %s; next step 75 %% %s; "
                 "STANDBY button -> 30 %% %s",
                 (int)no_shore, (int)rq, onb ? "ACCEPTED" : "ignored", c50 ? "yes" : "NO", late ? "yes" : "NO",
                 held ? "yes" : "NO", c75 ? "yes" : "NO", drop ? "yes" : "NO");
        test_report("PR3 UNCREWED power raise: code + shore joystick CONFIRM in 10 s, one step, centred; lowering instant",
                    no_shore == SB_ERR_JOY_NOT_READY && rq == SB_OK && !onb && c50 && late && held && c75 && drop, d);
    }

    /* PR4 the profile changes only in STANDBY (stopped): UNCREWED -> CREWED when a joystick is
     * plugged in; a CREWED joystick unplugged stays CREWED */
    {
        static sbsim_t x;
        static rjsim_t r;
        crew_init(&x, &r, 1207u, false, NULL);
        crew_plug();
        sbsim_run(&x, x.t + 5000u, rj_hook, &r);
        bool not_yet = x.sb.profile == SB_PROFILE_UNCREWED;
        sbsim_run(&x, x.t + 9000u, rj_hook, &r);
        bool crewed = x.sb.profile == SB_PROFILE_CREWED && (x.sb.alarm_bits & SB_ALM_PROFILE) &&
                      x.sb.profile_changes >= 2u;
        static sbsim_t y;
        static rjsim_t q;
        crew_init(&y, &q, 1208u, true, NULL);
        joy_set(0.0f, 0.5f, false, false);                    /* driving in MANUAL ... */
        sbsim_run(&y, y.t + 3000u, rj_hook, &q);
        bool man = sb_mode(&y.sb) == MODE_MANUAL;
        sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);   /* ... and it is unplugged */
        if (s_bridge) sim_hal_bridge_fault(SIM_BR_STOP);
        sbsim_run(&y, y.t + 400u, rj_hook, &q);
        bool gone = sb_mode(&y.sb) == MODE_STANDBY && (y.sb.alarm_bits & SB_ALM_JOY_FAULT);
        sbsim_run(&y, y.t + 30000u, rj_hook, &q);
        bool stays = y.sb.profile == SB_PROFILE_CREWED && man && gone;
        crew_plug();
        snprintf(d, sizeof d, "UNCREWED, joystick plugged in at STANDBY: after 5 s still UNCREWED %s, after 14 s CREWED + "
                 "PROFILE alarm %s; CREWED joystick unplugged while driving: STANDBY + JOYSTICK FAULT, "
                 "then 30 s: still CREWED %s",
                 not_yet ? "yes" : "NO", crewed ? "yes" : "NO", stays ? "yes" : "NO");
        test_report("PR4 profile changes only in STANDBY: UNCREWED -> CREWED when plugged in; CREWED never by unplugging",
                    not_yet && crewed && stays, d);
    }

    /* MV1 MANEUVER: HELM request + CONFIRM; throttle always idle, gear F / N / R from the lever, steering */
    {
        static sbsim_t x;
        static rjsim_t r;
        crew_init(&x, &r, 1210u, true, NULL);
        power_full(&x);
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        r.rj_on = true;
        uint8_t ask = pi_cmd(&x, &r.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        bool waits = sb_mode(&x.sb) == MODE_STANDBY;
        bool cf = onboard_confirm(&x, &r);
        bool on = sb_mode(&x.sb) == MODE_MANEUVER;
        float thr_max = 0.0f;
        bool gf = false, gr = false, gn = false;
        int under0 = x.w.shift_under_load;
        rj_lever(&x, &r, 1.0f, 2.0f);                               /* full forward */
        for (int i = 0; i < 200; i++) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r);
            thr_max = fmaxf(thr_max, sim_hal_throttle());
        }
        gf = gear_is(&x.sb.gear, GEAR_F);
        float v_f = x.w.speed_kn;
        r.x = 1.0f;
        for (int i = 0; i < 80; i++) { sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r); thr_max = fmaxf(thr_max, sim_hal_throttle()); }
        float est = x.sb.ap.est_angle;
        r.x = 0.0f;
        rj_lever(&x, &r, 0.0f, 2.0f);                               /* full reverse */
        for (int i = 0; i < 200; i++) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r);
            thr_max = fmaxf(thr_max, sim_hal_throttle());
        }
        gr = gear_is(&x.sb.gear, GEAR_R);
        rj_lever(&x, &r, 0.5f, 2.0f);
        for (int i = 0; i < 100; i++) { sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r); thr_max = fmaxf(thr_max, sim_hal_throttle()); }
        gn = gear_is(&x.sb.gear, GEAR_N);
        bool still = sb_mode(&x.sb) == MODE_MANEUVER;
        snprintf(d, sizeof d, "asked %u, waits for CONFIRM %s, CONFIRM %s -> MANEUVER %s; power limit %.0f %%, lever full "
                 "forward: gear F %s (%.1f kn at idle), full right: outboard %+.1f deg; full reverse: gear R %s; neutral "
                 "zone: N %s; highest throttle %.2f %%; gear changes under load %d",
                 ask, waits ? "yes" : "NO", cf ? "accepted" : "NO", on ? "yes" : "NO", x.sb.plim.fwd_pct,
                 gf ? "yes" : "NO", v_f, est, gr ? "yes" : "NO", gn ? "yes" : "NO", thr_max,
                 x.w.shift_under_load - under0);
        test_report("MV1 MANEUVER: request + CONFIRM; throttle always idle, gear F / N / R from the lever, steering",
                    ask == PROTO_R_OK && waits && cf && on && gf && est > 10.0f && gr && gn && still &&
                    thr_max <= 0.01f && x.w.shift_under_load == under0, d);
    }

    /* MV2 refusals; data loss; the onboard joystick wins (CREWED); from REMOTE the throttle goes to idle */
    {
        static sbsim_t x;
        static rjsim_t r;
        static link_t usb;
        crew_init(&x, &r, 1211u, true, NULL);
        uint8_t no_helm = pi_cmd(&x, &r.p, 3u, false, PROTO_T_MANEUVER, &ASK, 1u);
        link_init(&usb, &x.sb);
        uint8_t by_usb = usb_cmd(&x, &usb, PROTO_T_MANEUVER, &ASK, 1u);
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        hdg_lock(&x);
        uint8_t in_hdg = pi_cmd(&x, &r.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 3000u, rj_hook, &r);
        /* REMOTE at speed -> MANEUVER: throttle to idle */
        r.rj_on = true;
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        onboard_confirm(&x, &r);
        rj_lever(&x, &r, 0.5f, 2.0f);
        rj_lever(&x, &r, 1.0f, 0.5f);
        sbsim_run(&x, x.t + 8000u, rj_hook, &r);
        float thr_rem = sim_hal_throttle();
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        bool cf = onboard_confirm(&x, &r);
        sbsim_run(&x, x.t + 2500u, rj_hook, &r);
        bool idle = sb_mode(&x.sb) == MODE_MANEUVER && sim_hal_throttle() <= 0.01f;
        /* data gap: neutral, STANDBY after 5 s */
        r.rj_on = false;
        sbsim_run(&x, x.t + 600u, rj_hook, &r);
        bool lost = x.sb.remote.state == SB_REMOTE_LOST && x.sb.gear_demand == GEAR_N;
        sbsim_run(&x, x.t + 5200u, rj_hook, &r);
        bool sb5 = sb_mode(&x.sb) == MODE_STANDBY;
        /* the onboard joystick takes over */
        r.rj_on = true;
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        onboard_confirm(&x, &r);
        bool on2 = sb_mode(&x.sb) == MODE_MANEUVER;
        joy_set(0.7f, 0.0f, false, false);
        sbsim_run(&x, x.t + 300u, rj_hook, &r);
        bool man = sb_mode(&x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        /* FIRST TEST: never */
        static sbsim_t y;
        static rjsim_t q;
        sb_config_t c;
        sb_default_config(&c);
        c.first_test = true;
        crew_init(&y, &q, 1212u, true, &c);
        pi_cmd(&y, &q.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        q.rj_on = true;
        uint8_t ft = pi_cmd(&y, &q.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        bool ftc = onboard_confirm(&y, &q);
        snprintf(d, sizeof d, "no HELM %u (21); USB laptop %u (5); in HDG LOCK %u (7); REMOTE at %.1f %% -> MANEUVER "
                 "(CONFIRM %s): idle after 2.5 s %s; data gap: LOST + neutral %s, STANDBY after 5 s %s; onboard stick "
                 "-> MANUAL %s (was MANEUVER %s); FIRST TEST: asked %u (7), CONFIRM %s, mode %s",
                 no_helm, by_usb, in_hdg, thr_rem, cf ? "yes" : "NO", idle ? "yes" : "NO", lost ? "yes" : "NO",
                 sb5 ? "yes" : "NO", man ? "yes" : "NO", on2 ? "yes" : "NO", ft, ftc ? "ACCEPTED" : "nothing",
                 mode_name(sb_mode(&y.sb)));
        test_report("MV2 MANEUVER refused without HELM / from USB / HDG LOCK / FIRST TEST; data loss -> neutral, "
                    "STANDBY; onboard joystick wins",
                    no_helm == PROTO_R_NOT_HELM && by_usb == PROTO_R_NOT_ALLOWED && in_hdg == PROTO_R_WRONG_MODE &&
                    thr_rem > 5.0f && cf && idle && lost && sb5 && on2 && man && ft == PROTO_R_WRONG_MODE && !ftc &&
                    sb_mode(&y.sb) == MODE_STANDBY, d);
    }

    /* MV3 UNCREWED MANEUVER: confirmed on the shore joystick */
    {
        static sbsim_t x;
        static rjsim_t r;
        crew_init(&x, &r, 1213u, false, NULL);
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        r.rj_on = true;
        sbsim_run(&x, x.t + 300u, rj_hook, &r);
        uint8_t ask = pi_cmd(&x, &r.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        bool cf = shore_confirm(&x, &r);
        rj_lever(&x, &r, 1.0f, 2.0f);
        float thr_max = 0.0f;
        for (int i = 0; i < 100; i++) { sbsim_run(&x, x.t + AP_PERIOD_MS, rj_hook, &r); thr_max = fmaxf(thr_max, sim_hal_throttle()); }
        bool on = sb_mode(&x.sb) == MODE_MANEUVER && gear_is(&x.sb.gear, GEAR_F);
        crew_plug();
        snprintf(d, sizeof d, "UNCREWED: asked %u, shore CONFIRM %s, MANEUVER + forward gear %s, highest throttle %.2f %%",
                 ask, cf ? "accepted" : "NO", on ? "yes" : "NO", thr_max);
        test_report("MV3 UNCREWED MANEUVER: confirmed on the shore captain's joystick, throttle idle",
                    ask == PROTO_R_OK && cf && on && thr_max <= 0.01f, d);
    }

    /* MV4 (storm seed 389) a request pending in MANEUVER (AUTO start), then HELM lost -> STANDBY: the
     * CONFIRM afterwards (or in the same step) must start nothing - a STANDBY cancels pending requests */
    {
        static sbsim_t x;
        static rjsim_t r;
        static program_t p;
        prog_error_t err;
        uint8_t bad;
        crew_init(&x, &r, 1214u, true, NULL);
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        r.rj_on = true;
        pi_cmd(&x, &r.p, 2u, false, PROTO_T_MANEUVER, &ASK, 1u);
        onboard_confirm(&x, &r);
        bool on = sb_mode(&x.sb) == MODE_MANEUVER;
        prog_wp_t w = wp_world(&x, x.w.north_m + 1500.0, x.w.east_m, 6.0f, 25.0f, WP_ACT_STOP);
        prog_make(&p, &w, 1, PROG_END_STOP);
        sb_program_set(&x.sb, &p, &err, &bad);
        sb_result_t as = sb_auto_start(&x.sb);
        bool pending = x.sb.confirm.kind == CONF_AUTO_START;
        r.rj_on = false;
        pi_station_left(2u);                                    /* HELM lost -> STANDBY */
        sbsim_run(&x, x.t + 2u * AP_PERIOD_MS, rj_hook, &r);
        bool sb = sb_mode(&x.sb) == MODE_STANDBY && x.sb.confirm.kind == CONF_NONE;
        bool cf = onboard_confirm(&x, &r);
        sbsim_run(&x, x.t + 500u, rj_hook, &r);
        snprintf(d, sizeof d, "MANEUVER %s; AUTO start asked %d (pending %s); HELM lost -> STANDBY + request cancelled %s; "
                 "CONFIRM afterwards %s, mode %s", on ? "yes" : "NO", (int)as, pending ? "yes" : "NO", sb ? "yes" : "NO",
                 cf ? "ACCEPTED" : "nothing", mode_name(sb_mode(&x.sb)));
        test_report("MV4 a STANDBY (HELM lost in MANEUVER) cancels a pending request: CONFIRM after it starts nothing",
                    on && as == SB_OK && pending && sb && !cf && sb_mode(&x.sb) == MODE_STANDBY, d);
    }
}

/* --------------------------------------------- UNCREWED lost link (CLAUDE.md B) -- */

/* What the outputs did when the ENGINE KILL first came on. */
typedef struct {
    rjsim_t  r;
    bool     killed;
    uint32_t t_kill;
    float    thr_at_kill;
    uint16_t gear_at_kill;
    bool     moving_at_kill;
} llsim_t;

static void ll_hook(sbsim_t *x, void *ctx)
{
    llsim_t *l = (llsim_t *)ctx;
    rj_hook(x, &l->r);
    if (sim_hal_engine_killed() && !l->killed) {
        l->killed = true;
        l->t_kill = x->t;
        l->thr_at_kill = sim_hal_throttle();
        l->gear_at_kill = sim_hal_gear_pwm();
    }
}

/* A boat (crewed or not) driven remotely at the remote cap, shore distance dist_m
 * from the gateway (0xFFFF = no GW_SHORE at all). True if it drives. */
static bool ll_drive(sbsim_t *x, llsim_t *l, uint32_t seed, bool crewed, uint16_t dist_m)
{
    const uint8_t TAKE = 0u, ASK = 0u;
    memset(l, 0, sizeof *l);
    crew_init(x, &l->r, seed, crewed, NULL);
    l->r.p.shore_on = dist_m != 0xFFFFu;
    l->r.p.shore_flags = (uint8_t)(SB_SHORE_VALID | (dist_m <= 100u ? SB_SHORE_NEAR : 0u));
    l->r.p.shore_m = dist_m;
    pi_cmd(x, &l->r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
    l->r.rj_on = true;
    sbsim_run(x, x->t + 300u, ll_hook, l);
    sb_alarm_ack(&x->sb);
    pi_cmd(x, &l->r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
    if (crewed) onboard_confirm(x, &l->r); else shore_confirm(x, &l->r);
    rj_lever(x, &l->r, 1.0f, 1.0f);
    sbsim_run(x, x->t + 8000u, ll_hook, l);
    return sb_mode(&x->sb) == MODE_REMOTE && sim_hal_throttle() > 10.0f && gear_is(&x->sb.gear, GEAR_F);
}

/* the HELM station disappears (the gateway drops it: lost_s) */
static void ll_cut(sbsim_t *x, llsim_t *l)
{
    l->r.rj_on = false;
    pi_station_left(2u);
    sbsim_run(x, x->t + 2u * AP_PERIOD_MS, ll_hook, l);         /* (read by the link after the next step) */
}

static void lostlink_tests(void)
{
    char d[900];
    const uint8_t TAKE = 0u, ASK = 0u;
    printf("\n=== UNCREWED lost link + ENGINE KILL ===\n");
    bool idle_neutral_ok = true;                 /* LL7: every kill came after idle + neutral */
    int kills = 0;

    /* LL1 near shore (50 m): STANDBY at once, kill as soon as idle + neutral */
    {
        static sbsim_t x;
        static llsim_t l;
        bool drv = ll_drive(&x, &l, 1301u, false, 50u);
        uint32_t t0 = x.t;
        ll_cut(&x, &l);
        bool sb = sb_mode(&x.sb) == MODE_STANDBY && (x.sb.alarm_bits & SB_ALM_LINK_LOST);
        sbsim_run(&x, x.t + 6000u, ll_hook, &l);
        float t_kill = l.killed ? (l.t_kill - t0) * 0.001f : -1.0f;
        bool safe = l.killed && l.thr_at_kill <= 1.0f && l.gear_at_kill == x.sb.cfg.gear.pwm_n_us;
        idle_neutral_ok &= safe;
        kills += l.killed;
        snprintf(d, sizeof d, "driving %s; HELM station lost: STANDBY + LINK LOST %s; ENGINE KILL after %.2f s at "
                 "throttle %.1f %%, gear %s (idle + neutral first %s); state %d, alarm ENGINE KILLED %s",
                 drv ? "yes" : "NO", sb ? "yes" : "NO", t_kill, l.thr_at_kill,
                 l.gear_at_kill == x.sb.cfg.gear.pwm_n_us ? "N" : "not N", safe ? "yes" : "NO", x.sb.ll.state,
                 (x.sb.alarm_bits & SB_ALM_ENGINE_KILLED) ? "yes" : "NO");
        test_report("LL1 UNCREWED lost link near shore: STANDBY, idle + neutral, then ENGINE KILL at once",
                    drv && sb && safe && t_kill > 0.0f &&
                    t_kill < 3.0f + x.sb.cfg.remote_cap_fwd_pct * 0.01f * x.sb.servo.cfg.travel_s &&   /* + the modelled idle */
                    x.sb.ll.state == SB_LL_KILLED, d);
    }

    /* LL2 open sea (800 m): no kill during the grace time, kill after 10 s */
    {
        static sbsim_t x;
        static llsim_t l;
        bool drv = ll_drive(&x, &l, 1302u, false, 800u);
        uint32_t t0 = x.t;
        ll_cut(&x, &l);
        sbsim_run(&x, t0 + 9000u, ll_hook, &l);
        bool not_yet = !l.killed && sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 1.0f;
        float left = sb_lostlink_left_s(&x.sb);
        sbsim_run(&x, t0 + 12000u, ll_hook, &l);
        float t_kill = l.killed ? (l.t_kill - t0) * 0.001f : -1.0f;
        bool safe = l.killed && l.thr_at_kill <= 1.0f && l.gear_at_kill == x.sb.cfg.gear.pwm_n_us;
        idle_neutral_ok &= safe;
        kills += l.killed;
        snprintf(d, sizeof d, "driving %s; lost at sea (800 m from the shore): at 9 s no kill, idle %s (%.1f s left); "
                 "ENGINE KILL after %.2f s (grace 10 s), idle + neutral %s",
                 drv ? "yes" : "NO", not_yet ? "yes" : "NO", left, t_kill, safe ? "yes" : "NO");
        test_report("LL2 UNCREWED lost link at open sea: STANDBY, kill only after the grace time (10 s)",
                    drv && not_yet && left > 0.5f && left < 1.5f && t_kill >= 10.0f && t_kill < 10.5f && safe, d);
    }

    /* LL3 open sea, the link returns after 5 s: no kill, STANDBY stays, alarm at the station */
    {
        static sbsim_t x;
        static llsim_t l;
        bool drv = ll_drive(&x, &l, 1303u, false, 800u);
        uint32_t t0 = x.t;
        ll_cut(&x, &l);
        sbsim_run(&x, t0 + 5000u, ll_hook, &l);
        uint8_t back = pi_cmd(&x, &l.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        l.r.rj_on = true;
        sbsim_run(&x, t0 + 25000u, ll_hook, &l);
        bool ok = !l.killed && x.sb.ll.state == SB_LL_OK && sb_mode(&x.sb) == MODE_STANDBY &&
                  (x.sb.alarm_bits & SB_ALM_LINK_LOST) && x.sb.ll.returns == 1u;
        /* and it can drive again (a new request + CONFIRM) */
        sb_alarm_ack(&x.sb);
        rj_lever(&x, &l.r, 0.5f, 2.0f);                          /* the captain centres the lever */
        uint8_t again = pi_cmd(&x, &l.r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        bool cf = shore_confirm(&x, &l.r);
        snprintf(d, sizeof d, "driving %s; lost at sea, HELM back after 5 s (%u): killed %s, state %d, mode %s, LINK LOST "
                 "alarm for the station %s; never resumed by itself; new REMOTE request %u + CONFIRM %s -> %s",
                 drv ? "yes" : "NO", back, l.killed ? "YES" : "no", x.sb.ll.state, mode_name(sb_mode(&x.sb)),
                 ok ? "yes" : "NO", again, cf ? "yes" : "NO", mode_name(sb_mode(&x.sb)));
        test_report("LL3 UNCREWED link back within the grace time: no kill, stays in STANDBY at idle / neutral, alarm",
                    drv && back == PROTO_R_OK && ok && again == PROTO_R_OK && cf && sb_mode(&x.sb) == MODE_REMOTE, d);
    }

    /* LL4 shore info missing / stale / not valid / gateway lost -> treated as near shore: kill at once */
    {
        char t[600] = "";
        int good = 0;
        for (int k = 0; k < 3; k++) {
            static sbsim_t x;
            static llsim_t l;
            bool drv = ll_drive(&x, &l, 1304u + (uint32_t)k, false, k == 0 ? 0xFFFFu : 800u);
            if (k == 1) l.r.p.shore_flags = 0u;                 /* no chart data at the boat */
            uint32_t t0 = x.t;
            if (k == 2) {                                        /* the whole gateway goes quiet */
                l.r.p.pi_on = false;
                l.r.p.shore_on = false;
                l.r.rj_on = false;
            } else {
                sbsim_run(&x, x.t + 1500u, ll_hook, &l);
                t0 = x.t;
                ll_cut(&x, &l);
            }
            sbsim_run(&x, t0 + 8000u, ll_hook, &l);
            float t_kill = l.killed ? (l.t_kill - t0) * 0.001f : -1.0f;
            bool safe = l.killed && l.thr_at_kill <= 1.0f && l.gear_at_kill == x.sb.cfg.gear.pwm_n_us;
            idle_neutral_ok &= safe;
            kills += l.killed;
            /* gateway lost: found after 3 s (pc_timeout), then at once */
            bool in_time = l.killed && t_kill < (k == 2 ? 6.0f : 3.0f) +
                                         x.sb.cfg.remote_cap_fwd_pct * 0.01f * x.sb.servo.cfg.travel_s;   /* modelled idle */
            good += drv && safe && in_time;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s: kill after %.2f s, idle + neutral first %s", k ? "; " : "",
                     k == 0 ? "never any shore info" : k == 1 ? "shore info not valid (no charts)" :
                     "gateway lost (shore info stale)", t_kill, safe ? "yes" : "NO");
        }
        test_report("LL4 shore info missing / not valid / stale (gateway lost) = near shore: immediate kill", good == 3, t);
    }

    /* LL5 CREWED never kills: same loss, the current behaviour (REMOTE LINK LOST, no mode change from the loss) */
    {
        static sbsim_t x;
        static llsim_t l;
        bool drv = ll_drive(&x, &l, 1308u, true, 50u);
        ll_cut(&x, &l);
        l.r.p.pi_on = false;                                     /* the gateway too */
        sbsim_run(&x, x.t + 30000u, ll_hook, &l);
        bool ok = !l.killed && !sim_hal_engine_killed() && x.sb.ll.state == SB_LL_OK &&
                  (x.sb.alarm_bits & SB_ALM_REMOTE_LINK) && !(x.sb.alarm_bits & SB_ALM_LINK_LOST);
        snprintf(d, sizeof d, "CREWED, driving %s; HELM station + gateway lost near shore for 30 s: engine killed %s, "
                 "REMOTE LINK LOST %s, LINK LOST (uncrewed) %s, mode %s",
                 drv ? "yes" : "NO", l.killed ? "YES" : "never", (x.sb.alarm_bits & SB_ALM_REMOTE_LINK) ? "yes" : "NO",
                 (x.sb.alarm_bits & SB_ALM_LINK_LOST) ? "YES" : "no", mode_name(sb_mode(&x.sb)));
        test_report("LL5 CREWED: never an engine kill (current lost-link behaviour)", drv && ok, d);
    }

    /* LL6 the kill stays latched: link back, HELM, requests - still killed; only a reset (on board) restarts */
    {
        static sbsim_t x;
        static llsim_t l;
        ll_drive(&x, &l, 1309u, false, 50u);
        ll_cut(&x, &l);
        sbsim_run(&x, x.t + 4000u, ll_hook, &l);
        bool k1 = x.sb.ll.state == SB_LL_KILLED;
        idle_neutral_ok &= l.killed && l.thr_at_kill <= 1.0f && l.gear_at_kill == x.sb.cfg.gear.pwm_n_us;
        kills += l.killed;
        uint8_t helm = pi_cmd(&x, &l.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        l.r.rj_on = true;
        sb_alarm_ack(&x.sb);
        uint8_t ask = pi_cmd(&x, &l.r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
        bool cf = shore_confirm(&x, &l.r);
        sbsim_run(&x, x.t + 20000u, ll_hook, &l);
        bool still = sim_hal_engine_killed() && x.sb.ll.state == SB_LL_KILLED && sb_mode(&x.sb) == MODE_STANDBY &&
                     (x.sb.alarm_bits & SB_ALM_ENGINE_KILLED) && fabsf(x.w.speed_kn) < 0.5f;
        /* power cycle on board */
        sim_hal_chip_reset(HAL_RESET_POWER_ON);
        sb_init(&x.sb, &x.cfg);
        sbsim_run(&x, x.t + 500u, ll_hook, &l);
        bool reset = !sim_hal_engine_killed() && x.sb.ll.state == SB_LL_OK;
        snprintf(d, sizeof d, "killed %s; HELM back %u, alarm ack, REMOTE request %u + CONFIRM %s: after 20 s still "
                 "killed + STANDBY + alarm, boat stopped (%.1f kn) %s; after a reset on board: kill output off %s",
                 k1 ? "yes" : "NO", helm, ask, cf ? "ACCEPTED" : "refused", x.w.speed_kn, still ? "yes" : "NO",
                 reset ? "yes" : "NO");
        test_report("LL6 ENGINE KILL latched until reset on board (or the HELM station's remote restart: ER4)", k1 && still && !cf && reset, d);
    }

    snprintf(d, sizeof d, "%d kills in LL1-LL6: every one with the throttle at idle and the gear in neutral %s",
             kills, idle_neutral_ok ? "yes" : "NO");
    test_report("LL7 every engine kill only after idle + neutral", kills >= 6 && idle_neutral_ok, d);
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

/* Power-up with no joystick seen (analog not detected / USB bridge not
 * sending): not armed (and no alarm) during the profile detection window,
 * then UNCREWED (CLAUDE.md section A): the outputs arm without it. A
 * joystick seen: CREWED, arms only once it is centred. */
static void arm_joystick_tests(void)
{
    char d[500];
    printf("\n=== R9: outputs arm only with the joystick seen ===\n");
    static sbsim_t x;
    sbsim_init(&x, 281u, 0.0f, 0.0f, NULL);
    sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);   /* (bridge: no device frames) */
    if (s_bridge) sim_hal_bridge_fault(SIM_BR_STOP);
    sbsim_run(&x, 5000u, NULL, NULL);
    bool unarmed = !x.sb.armed && x.sb.profile == SB_PROFILE_DETECTING, alarm = (x.sb.alarm_bits & SB_ALM_NOT_ARMED) != 0u;
    sbsim_run(&x, 8000u, NULL, NULL);
    bool unc = x.sb.profile == SB_PROFILE_UNCREWED && x.sb.armed && !(x.sb.alarm_bits & SB_ALM_NOT_ARMED);
    sim_hal_bridge_fault(0u);
    sbsim_init(&x, 282u, 0.0f, 0.0f, NULL);              /* joystick fitted and centred from the start */
    sbsim_run(&x, 3000u, NULL, NULL);
    bool armed_with = x.sb.armed && x.sb.profile == SB_PROFILE_CREWED;
    snprintf(d, sizeof d, "no joystick seen for 5 s: still detecting + not armed %s, NOT ARMED alarm %s; at 8 s: "
             "UNCREWED + armed %s (profile %s); joystick fitted + centred: CREWED + armed %s",
             unarmed ? "yes" : "NO", alarm ? "YES" : "no", unc ? "yes" : "NO", sb_profile_name(x.sb.profile),
             armed_with ? "yes" : "NO");
    test_report("R9 no joystick at power-up: UNCREWED after the detection window, arms without it; seen: CREWED",
                unarmed && !alarm && unc && armed_with, d);
}

/* ----------------------- R11: an acknowledge never hides an active condition -- */

static void usb_link_hook(sbsim_t *x, void *ctx)
{
    link_step((link_t *)ctx, &x->sb);
    uint8_t out[1024];
    while (sim_hal_link_tx_pop(out, sizeof out) > 0u) {}
}

static void alarm_ack_tests(void)
{
    char d[700];
    const uint8_t TAKE = 0u;
    printf("\n=== R11: alarm acknowledge clears only what is gone ===\n");
    static sbsim_t x;
    static link_t usb;
    sbsim_init(&x, 291u, 0.0f, 0.0f, NULL);
    sbsim_run(&x, 2000u, NULL, NULL);
    link_init(&usb, &x.sb);
    usb_cmd(&x, &usb, PROTO_T_HELM, &TAKE, 1u);           /* the laptop is connected ... */
    sbsim_run(&x, x.t + 5000u, usb_link_hook, &usb);       /* ... and goes silent: PC LINK LOST */
    bool lost = (x.sb.alarm_bits & SB_ALM_PC_LINK) != 0u;
    sim_hal_fail_heading(true);                            /* compass fails too */
    sbsim_run(&x, x.t + 3000u, usb_link_hook, &usb);
    bool comp = (x.sb.alarm_bits & SB_ALM_COMPASS) != 0u;
    press(&x, HAL_BTN_STANDBY);                           /* STANDBY: the button acknowledges */
    sbsim_run(&x, x.t + 300u, usb_link_hook, &usb);
    bool both_kept = (x.sb.alarm_bits & SB_ALM_PC_LINK) && (x.sb.alarm_bits & SB_ALM_COMPASS) && sim_hal_alarm();
    sim_hal_fail_heading(false);                           /* compass back */
    sbsim_run(&x, x.t + 2000u, usb_link_hook, &usb);
    press(&x, HAL_BTN_STANDBY);
    sbsim_run(&x, x.t + 300u, usb_link_hook, &usb);
    uint32_t after_comp = x.sb.alarm_bits;
    usb_cmd(&x, &usb, PROTO_T_HELM, &TAKE, 1u);            /* the laptop is back */
    sbsim_run(&x, x.t + 300u, usb_link_hook, &usb);
    usb_cmd(&x, &usb, PROTO_T_ALARM_ACK, NULL, 0u);
    sbsim_run(&x, x.t + 300u, usb_link_hook, &usb);
    uint32_t end = x.sb.alarm_bits;
    snprintf(d, sizeof d, "PC link lost %s, compass failed %s; acknowledged while both still wrong -> both still on %s; "
             "compass back, acknowledged -> alarms 0x%lx (only PC LINK left: %s); link back + ACK -> 0x%lx",
             lost ? "yes" : "NO", comp ? "yes" : "NO", both_kept ? "yes" : "NO", (unsigned long)after_comp,
             after_comp == SB_ALM_PC_LINK ? "yes" : "NO", (unsigned long)end);
    test_report("R11 alarm ACK clears only alarms whose cause is gone (link lost / compass stay while wrong)",
                lost && comp && both_kept && after_comp == SB_ALM_PC_LINK && end == 0u, d);
}

/* --------------------------------- R17: telemetry frames fit their payload -- */

static void telem_size_tests(void)
{
    char d[400];
    const uint8_t TAKE = 0u;
    printf("\n=== R17: telemetry sizes ===\n");
    static sbsim_t x;
    static link_t usb;
    sbsim_init(&x, 301u, 0.0f, 0.0f, NULL);
    sbsim_run(&x, 2000u, NULL, NULL);
    link_init(&usb, &x.sb);
    usb_cmd(&x, &usb, PROTO_T_HELM, &TAKE, 1u);
    for (int i = 0; i < 40; i++) {                       /* keep the link alive, telemetry at 10 Hz */
        usb_cmd(&x, &usb, PROTO_T_HEARTBEAT, NULL, 0u);
        sbsim_run(&x, x.t + 500u, usb_link_hook, &usb);
    }
    snprintf(d, sizeof d, "TELEMETRY %u of %u bytes (%u free), TELEM2 %u of %u (%u free); frames that did not fit %lu",
             usb.telem_len, PROTO_MAX_PAYLOAD, PROTO_MAX_PAYLOAD - usb.telem_len, usb.telem2_len, PROTO_MAX_PAYLOAD,
             PROTO_MAX_PAYLOAD - usb.telem2_len, (unsigned long)usb.telem_overflows);
    test_report("R17 telemetry frames fit (a new field that doesn't fit fails here, never silently)",
                usb.telem_len > 0u && usb.telem2_len > 0u && usb.telem_len <= PROTO_MAX_PAYLOAD &&
                usb.telem2_len <= PROTO_MAX_PAYLOAD && usb.telem_overflows == 0u, d);
}

/* ---------------------------- R18: a burst of frames is parsed, none lost -- */

static void link_burst_tests(void)
{
    char d[300];
    printf("\n=== R18: frame burst on a link ===\n");
    static sbsim_t x;
    static link_t usb;
    uint8_t fr[PROTO_MAX_FRAME];
    sbsim_init(&x, 311u, 0.0f, 0.0f, NULL);
    sbsim_run(&x, 1000u, NULL, NULL);
    link_init(&usb, &x.sb);
    sim_hal_link_reset();
    uint32_t pushed = 0u, bytes = 0u;
    for (int i = 0; i < 150; i++) {                      /* 150 frames arrive at once */
        size_t n = proto_encode(fr, PROTO_T_HEARTBEAT, (uint8_t)i, NULL, 0u);
        if (sim_hal_link_rx_push(fr, (uint32_t)n) == n) { pushed++; bytes += (uint32_t)n; }
    }
    uint32_t f0 = usb.rx.frames;
    sbsim_run(&x, x.t + 500u, usb_link_hook, &usb);       /* ~10 steps */
    uint32_t got = usb.rx.frames - f0;
    snprintf(d, sizeof d, "%lu frames (%lu bytes) in one burst: parsed %lu, ring overflows %lu",
             (unsigned long)pushed, (unsigned long)bytes, (unsigned long)got, (unsigned long)usb.rx.overflows);
    test_report("R18 a burst of 150 frames: all parsed over a few steps, none overwritten",
                pushed == 150u && got == 150u && usb.rx.overflows == 0u, d);
}

/* ------------------------- coverage gap: the throttle servo clamp (servo.c) -- */

static void servo_clamp_tests(void)
{
    char d[700];
    printf("\n=== servo.c: power-limit clamp corner cases ===\n");
    servo_config_t c;
    servo_t sv;
    plim_t lim;
    memset(&lim, 0, sizeof lim);
    servo_default_config(&c);
    servo_init(&sv, &c);
    const float dt = 0.05f;
    int fails = 0;
    char why[300] = "";
#define CHK(cond, txt) do { if (!(cond)) { fails++; snprintf(why + strlen(why), sizeof why - strlen(why), "%s; ", txt); } } while (0)
    /* forward 30 %: above it -> clamped */
    lim.fwd_pct = 30.0f; lim.rev_pct = 5.0f;
    float o = servo_throttle_write(&sv, &lim, true, 80.0f, dt, -1.0f);
    CHK(o <= 30.0f && sv.clamped, "fwd 80 over 30 not clamped");
    /* reverse uses the reverse limit, also at exactly the limit */
    o = servo_throttle_write(&sv, &lim, false, 5.0f, dt, -1.0f);
    CHK(o <= 5.0f + 1e-4f, "reverse at its limit");
    o = servo_throttle_write(&sv, &lim, false, 12.0f, dt, -1.0f);
    CHK(o <= 5.0f + 1e-4f && sv.clamped, "reverse 12 over 5 not clamped");
    /* limit lowered while above it: only comes down, at most rate_down per step */
    servo_init(&sv, &c);
    lim.fwd_pct = 100.0f;
    for (int i = 0; i < 60; i++) servo_throttle_write(&sv, &lim, true, 90.0f, dt, -1.0f);
    float before = sv.out_pct;
    lim.fwd_pct = 30.0f;
    o = servo_throttle_write(&sv, &lim, true, 90.0f, dt, -1.0f);
    CHK(o < before && o >= before - c.rate_down_pct_s * dt - 1e-3f, "limit drop: not coming down at the closing rate");
    for (int i = 0; i < 100; i++) o = servo_throttle_write(&sv, &lim, true, 90.0f, dt, -1.0f);
    CHK(o <= 30.0f + 1e-4f, "never reached the lowered limit");
    /* limit dropped during reverse */
    servo_init(&sv, &c);
    lim.rev_pct = 15.0f;
    for (int i = 0; i < 40; i++) servo_throttle_write(&sv, &lim, false, 15.0f, dt, -1.0f);
    lim.rev_pct = 5.0f;
    for (int i = 0; i < 40; i++) o = servo_throttle_write(&sv, &lim, false, 15.0f, dt, -1.0f);
    CHK(o <= 5.0f + 1e-4f, "reverse limit drop not followed");
    /* NaN / negative / limits out of range */
    o = servo_throttle_write(&sv, &lim, true, NAN, dt, -1.0f);
    CHK(o == 0.0f && sv.out_us == c.idle_us, "NaN demand not idle");
    o = servo_throttle_write(&sv, &lim, true, -20.0f, dt, -1.0f);
    CHK(o == 0.0f, "negative demand not idle");
    lim.fwd_pct = 250.0f;
    servo_init(&sv, &c);
    for (int i = 0; i < 400; i++) o = servo_throttle_write(&sv, &lim, true, 100.0f, dt, -1.0f);
    CHK(o <= 100.0f && sv.out_us <= c.full_us, "limit above 100 not capped");
    lim.fwd_pct = -5.0f;
    o = servo_throttle_write(&sv, &lim, true, 50.0f, dt, -1.0f);
    CHK(o <= sv.out_pct + 1e-4f && sv.limit_pct == 0.0f, "negative limit not 0");
    /* reversed servo (full < idle): pulses stay inside its calibrated range */
    servo_config_t r = c;
    r.idle_us = 2000u; r.full_us = 1000u;
    uint16_t p0 = servo_pulse_for(&r, 0.0f), p100 = servo_pulse_for(&r, 100.0f), pnan = servo_pulse_for(&r, NAN),
             pover = servo_pulse_for(&r, 150.0f);
    CHK(p0 == 2000u && p100 == 1000u && pnan == 2000u && pover == 1000u, "reversed servo pulses (NaN must be idle)");
#undef CHK
    snprintf(d, sizeof d, "fwd / rev clamp, limit drop (closing rate, fwd and rev), NaN / negative demand, limit out of "
             "range, reversed servo (idle 2000 us, full 1000 us: 0 %% -> %u, 100 %% -> %u, NaN -> %u, 150 %% -> %u): %d "
             "problem(s) %s", p0, p100, pnan, pover, fails, why);
    test_report("servo.c clamp corner cases (the only throttle writer)", fails == 0, d);
}

/* --------------------------------------- FIRST TEST (steering only) config -- */

static sb_config_t cfg_first_test(void)
{
    sb_config_t c;
    sb_default_config(&c);
    c.first_test = true;
    return c;
}

typedef struct {
    float thr_max;              /* throttle servo, highest seen (%) */
    uint16_t gear_min, gear_max;
    int motor_steps;            /* steering motor commands */
} ftwatch_t;

static void ft_hook(sbsim_t *x, void *ctx)
{
    ftwatch_t *f = (ftwatch_t *)ctx;
    if (sim_hal_throttle() > f->thr_max) f->thr_max = sim_hal_throttle();
    uint16_t g = sim_hal_gear_pwm();
    if (g < f->gear_min) f->gear_min = g;
    if (g > f->gear_max) f->gear_max = g;
    if (sim_hal_motor_command() != 0.0f) f->motor_steps++;
    (void)x;
}

static void first_test_tests(void)
{
    char d[900];
    printf("\n=== FIRST TEST configuration (steering only) ===\n");
    sb_config_t c = cfg_first_test();

    /* FT1 power-up without joystick, neutral switch or gateway: arms; throttle / gear held safe */
    {
        static sbsim_t x;
        ftwatch_t f = { 0.0f, 0xFFFFu, 0u, 0 };
        sbsim_init(&x, 321u, 0.0f, 0.0f, &c);
        sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);
        x.w.neutral_switch = true;
        x.w.servo_stuck = true;                         /* gear servo not connected: "not neutral" possible */
        x.w.gear_pos_us = 1900.0f;                      /* the switch reports NOT neutral */
        sbsim_run(&x, 2000u, ft_hook, &f);
        bool armed = x.sb.armed;
        bool not_armed_alarm = (x.sb.alarm_bits & SB_ALM_NOT_ARMED) != 0u;
        bool gear_alarm = (x.sb.alarm_bits & SB_ALM_GEAR) != 0u;
        snprintf(d, sizeof d, "no joystick, neutral switch says NOT neutral, no gateway: armed %s after 2 s, NOT ARMED alarm %s, "
                 "GEAR alarm %s; throttle servo max %.1f %%, gear pulse %u..%u us (neutral %u)",
                 armed ? "yes" : "NO", not_armed_alarm ? "YES" : "no", gear_alarm ? "YES" : "no", f.thr_max,
                 f.gear_min, f.gear_max, x.sb.cfg.gear.pwm_n_us);
        test_report("FT1 FIRST TEST: arms without joystick / neutral switch / gateway; throttle + gear held safe",
                    armed && !not_armed_alarm && !gear_alarm && f.thr_max <= 0.0f && f.gear_min == x.sb.cfg.gear.pwm_n_us &&
                    f.gear_max == x.sb.cfg.gear.pwm_n_us, d);
    }

    /* FT2 allowed modes: STANDBY, TRAINING, HDG LOCK; MANUAL / AUTO / REMOTE refused */
    {
        static sbsim_t x;
        static program_t p;
        prog_error_t e;
        uint8_t bad;
        sbsim_init(&x, 322u, 0.0f, 0.0f, &c);
        sbsim_run(&x, 3000u, NULL, NULL);
        joy_set(0.8f, 0.0f, false, false);              /* a joystick that happens to be connected: deflected */
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool no_manual = sb_mode(&x.sb) == MODE_STANDBY;
        joy_set(0.0f, 0.0f, false, false);
        prog_wp_t w = wp_world(&x, x.w.north_m + 600.0, x.w.east_m, 5.0f, 25.0f, WP_ACT_STOP);
        prog_make(&p, &w, 1, PROG_END_STOP);
        sb_program_set(&x.sb, &p, &e, &bad);
        sb_result_t r_auto = sb_auto_start(&x.sb);
        sb_result_t r_rem = sb_remote_request(&x.sb, 2u);
        press(&x, HAL_BTN_HDG_LOCK);
        bool hdg = sb_mode(&x.sb) == MODE_HDG_LOCK;
        press(&x, HAL_BTN_STANDBY);
        bool train = sb_request_mode(&x.sb, MODE_TRAINING, WHY_PC);
        snprintf(d, sizeof d, "joystick deflected -> stays STANDBY %s; AUTO start %d, remote request %d (refused = %d); "
                 "HDG LOCK button %s; TRAINING from the PC %s",
                 no_manual ? "yes" : "NO", (int)r_auto, (int)r_rem, (int)SB_ERR_WRONG_MODE, hdg ? "yes" : "NO",
                 train ? "yes" : "NO");
        test_report("FT2 FIRST TEST modes: STANDBY / TRAINING / HDG LOCK only (no MANUAL, AUTO, REMOTE)",
                    no_manual && r_auto == SB_ERR_WRONG_MODE && r_rem == SB_ERR_WRONG_MODE && hdg && train, d);
    }

    /* FT3 HDG LOCK steers while the boat moves ahead (someone at the throttle), pauses when it stops */
    {
        static sbsim_t x;
        ftwatch_t f = { 0.0f, 0xFFFFu, 0u, 0 };
        sbsim_init(&x, 323u, 90.0f, 0.0f, &c);
        sbsim_run(&x, 3000u, NULL, NULL);
        x.w.hand_drive = true; x.w.hand_gear = GEAR_F; x.w.hand_throttle = 35.0f;
        sbsim_run(&x, x.t + 30000u, ft_hook, &f);
        press(&x, HAL_BTN_HDG_LOCK);
        float locked = x.sb.ap.target_heading;
        press(&x, HAL_BTN_PLUS_10);
        sbsim_run(&x, x.t + 60000u, ft_hook, &f);
        float err = fabsf(hdg_err(x.w.heading_deg, locked + 10.0f));
        bool moving = x.sb.moving_ahead;
        x.w.hand_throttle = 0.0f; x.w.hand_gear = GEAR_N;   /* stops */
        sbsim_run(&x, x.t + 60000u, NULL, NULL);
        ftwatch_t g = { 0.0f, 0xFFFFu, 0u, 0 };
        sbsim_run(&x, x.t + 20000u, ft_hook, &g);
        snprintf(d, sizeof d, "hand throttle 35 %%: %.1f kn, moving ahead %s; HDG LOCK +10 -> heading error %.1f deg after 60 s; "
                 "throttle servo max %.1f %%; stopped: moving ahead %s, steering motor steps in 20 s %d, mode %s",
                 x.w.speed_kn, moving ? "yes" : "NO", err, f.thr_max, x.sb.moving_ahead ? "YES" : "no", g.motor_steps,
                 mode_name(sb_mode(&x.sb)));
        test_report("FT3 FIRST TEST HDG LOCK: steers while moving ahead (throttle by hand), rests when stopped",
                    moving && err < 3.0f && f.thr_max <= 0.0f && !x.sb.moving_ahead && g.motor_steps == 0 &&
                    sb_mode(&x.sb) == MODE_HDG_LOCK, d);
    }

    /* FT4 emergency stop: STANDBY at once, alarm, nothing leaves STANDBY while pressed (also the normal config) */
    {
        static sbsim_t x;
        sbsim_init(&x, 324u, 90.0f, 0.0f, &c);
        sim_hal_set_estop(true, false);
        sbsim_run(&x, 3000u, NULL, NULL);
        x.w.hand_drive = true; x.w.hand_gear = GEAR_F; x.w.hand_throttle = 35.0f;
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        press(&x, HAL_BTN_HDG_LOCK);
        bool hdg = sb_mode(&x.sb) == MODE_HDG_LOCK;
        uint32_t t0 = x.t;
        sim_hal_set_estop(true, true);
        while (sb_mode(&x.sb) != MODE_STANDBY && x.t < t0 + 1000u) sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
        uint32_t react = x.t - t0;
        bool alarm = (x.sb.alarm_bits & SB_ALM_ESTOP) != 0u;
        press(&x, HAL_BTN_HDG_LOCK);
        bool refused = sb_mode(&x.sb) == MODE_STANDBY;
        press(&x, HAL_BTN_STANDBY);                       /* acknowledge while still pressed: alarm stays */
        bool kept = (x.sb.alarm_bits & SB_ALM_ESTOP) != 0u;
        sim_hal_set_estop(true, false);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        press(&x, HAL_BTN_HDG_LOCK);
        bool again = sb_mode(&x.sb) == MODE_HDG_LOCK;
        /* the normal configuration too: MANUAL -> STANDBY */
        static sbsim_t y;
        sbsim_init(&y, 325u, 0.0f, 0.0f, NULL);
        sim_hal_set_estop(true, false);
        sbsim_run(&y, 2000u, NULL, NULL);
        joy_set(0.5f, 0.5f, false, false);
        sbsim_run(&y, y.t + 3000u, NULL, NULL);
        bool man = sb_mode(&y.sb) == MODE_MANUAL;
        sim_hal_set_estop(true, true);
        sbsim_run(&y, y.t + 200u, NULL, NULL);
        bool norm = sb_mode(&y.sb) == MODE_STANDBY && (y.sb.alarm_bits & SB_ALM_ESTOP);
        sim_hal_set_estop(false, false);
        joy_set(0.0f, 0.0f, false, false);
        snprintf(d, sizeof d, "HDG LOCK %s; e-stop -> STANDBY after %lu ms, alarm %s; HDG LOCK while pressed refused %s; "
                 "acknowledged while pressed: alarm kept %s; released -> HDG LOCK again %s; normal config MANUAL %s -> "
                 "STANDBY + alarm %s",
                 hdg ? "yes" : "NO", (unsigned long)react, alarm ? "yes" : "NO", refused ? "yes" : "NO", kept ? "yes" : "NO",
                 again ? "yes" : "NO", man ? "yes" : "NO", norm ? "yes" : "NO");
        test_report("FT4 emergency stop: STANDBY at once + alarm; nothing leaves STANDBY while pressed (every config)",
                    hdg && react <= 100u && alarm && refused && kept && again && man && norm, d);
    }

    /* FT5 TRAINING with the throttle by hand: starts once the boat moves ahead, throttle servo stays idle */
    {
        static sbsim_t x;
        ftwatch_t f = { 0.0f, 0xFFFFu, 0u, 0 };
        sbsim_init(&x, 326u, 0.0f, 0.0f, &c);
        sbsim_run(&x, 3000u, NULL, NULL);
        bool req = sb_request_mode(&x.sb, MODE_TRAINING, WHY_PC);
        sbsim_run(&x, x.t + 40000u, ft_hook, &f);
        bool waiting = x.sb.train.stage == TS_SPINUP;      /* not moving yet: waits */
        x.w.hand_drive = true; x.w.hand_gear = GEAR_F; x.w.hand_throttle = 20.0f;   /* slow, like TRAINING itself */
        uint32_t t0 = x.t;
        while (x.t < t0 + 600000u && sb_mode(&x.sb) == MODE_TRAINING && x.sb.train.stage != TS_RESULT)
            sbsim_run(&x, x.t + 1000u, ft_hook, &f);
        bool result = x.sb.train.stage == TS_RESULT && sb_mode(&x.sb) == MODE_TRAINING;
        snprintf(d, sizeof d, "TRAINING requested %s; not moving for 40 s: still spinning up %s; throttle by hand 20 %% -> "
                 "result after %.0f s %s (mode %s, training: %s, %.1f kn); throttle servo max %.1f %%, gear pulse %u..%u",
                 req ? "yes" : "NO", waiting ? "yes" : "NO", (x.t - t0) * 0.001f, result ? "yes" : "NO",
                 mode_name(sb_mode(&x.sb)), training_result_text(x.sb.ap.train.result), x.w.speed_kn, f.thr_max,
                 f.gear_min, f.gear_max);
        test_report("FT5 FIRST TEST TRAINING: waits until moving ahead (throttle by hand), reaches the result; servo idle",
                    req && waiting && result && f.thr_max <= 0.0f && f.gear_max == x.sb.cfg.gear.pwm_n_us, d);
    }
}


/* ---------------------------------------------------------- BENCH TEST -- */

typedef struct {
    link_t  *usb;
    int      diag_frames;       /* BENCH_DIAG frames the laptop got */
    bool     silent;            /* the laptop is unplugged: nothing comes in */
    proto_rx_t rx;
    float    thr_max;
    uint16_t gear_min, gear_max;
    int      left_steps, right_steps;
    int      alarm_steps;
} btwatch_t;

static void bt_hook(sbsim_t *x, void *ctx)
{
    btwatch_t *b = (btwatch_t *)ctx;
    if (!b->silent) {                                   /* the ground station's 1 Hz heartbeat */
        if (x->t % 1000u < AP_PERIOD_MS) {
            uint8_t fr[PROTO_MAX_FRAME];
            size_t n = proto_encode(fr, PROTO_T_HEARTBEAT, 1u, NULL, 0u);
            sim_hal_link_rx_push(fr, (uint32_t)n);
        }
    }
    link_step(b->usb, &x->sb);
    uint8_t out[1024];
    uint32_t n;
    while ((n = sim_hal_link_tx_pop(out, 256u)) > 0u) {
        proto_frame_t f;
        proto_rx_push(&b->rx, out, n);
        while (proto_rx_next(&b->rx, &f)) if (f.type == PROTO_T_BENCH_DIAG) b->diag_frames++;
    }
    if (sim_hal_throttle() > b->thr_max) b->thr_max = sim_hal_throttle();
    uint16_t g = sim_hal_gear_pwm();
    if (g < b->gear_min) b->gear_min = g;
    if (g > b->gear_max) b->gear_max = g;
    if (sim_hal_motor_command() < 0.0f) b->left_steps++;
    if (sim_hal_motor_command() > 0.0f) b->right_steps++;
    if (sim_hal_alarm()) b->alarm_steps++;
}

static void bt_watch_init(btwatch_t *b, link_t *usb)
{
    memset(b, 0, sizeof *b);
    b->usb = usb;
    b->gear_min = 0xFFFFu;
    proto_rx_init(&b->rx);
}

static uint8_t bench_cmd(sbsim_t *x, link_t *usb, uint8_t action, uint16_t arg)
{
    uint8_t p[3] = { action, (uint8_t)arg, (uint8_t)(arg >> 8) };
    return usb_cmd(x, usb, PROTO_T_BENCH, p, 3u);
}

static void bench_tests(void)
{
    char d[900];
    printf("\n=== BENCH TEST (dock: sensors, steering pulses, buttons, buzzer, e-stop, monitor trip) ===\n");

    /* BT1 who / when: only the USB laptop, only from STANDBY, not moving, not with the e-stop
     * pressed; also in the FIRST TEST build; BENCH_DIAG frames only in BENCH TEST */
    {
        static sbsim_t x, y;
        static pisim_t p;
        static link_t usb, usb2;
        btwatch_t b;
        sbsim_init(&x, 951u, 90.0f, 0.0f, NULL);
        pisim_init(&p, &x, 921600u);
        link_init(&usb, &x.sb);
        sim_hal_set_estop(true, false);
        sbsim_run(&x, 3000u, pi_hook, &p);
        uint8_t pl[3] = { SB_BENCH_ENTER, 0u, 0u };
        uint8_t r_gw = pi_cmd(&x, &p, 4u, true, PROTO_T_BENCH, pl, 3u);       /* boat LAN station: no */
        bt_watch_init(&b, &usb);
        sbsim_run(&x, x.t + 2000u, bt_hook, &b);
        int diag_standby = b.diag_frames;
        uint8_t r_usb = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        bool in_bench = sb_mode(&x.sb) == MODE_BENCH;
        b.diag_frames = 0;
        sbsim_run(&x, x.t + 2000u, bt_hook, &b);
        int diag_bench = b.diag_frames;
        bench_cmd(&x, &usb, SB_BENCH_LEAVE, 0u);
        bool left = sb_mode(&x.sb) == MODE_STANDBY;
        uint8_t r_mode = 0xFF;
        press(&x, HAL_BTN_HDG_LOCK);                                      /* HDG LOCK -> not from there */
        r_mode = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        press(&x, HAL_BTN_STANDBY);
        sim_hal_set_estop(true, true);
        sbsim_run(&x, x.t + 200u, bt_hook, &b);
        uint8_t r_estop = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        sim_hal_set_estop(true, false);
        press(&x, HAL_BTN_STANDBY);
        uint8_t r_cmd_out = bench_cmd(&x, &usb, SB_BENCH_LEFT, 100u);     /* not in BENCH TEST */
        /* moving (someone drives by hand): not on the dock */
        x.w.hand_drive = true; x.w.hand_gear = GEAR_F; x.w.hand_throttle = 30.0f;
        sbsim_run(&x, x.t + 30000u, bt_hook, &b);
        uint8_t r_moving = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        float kn = x.w.speed_kn;
        sim_hal_set_estop(false, false);
        /* the FIRST TEST build */
        sb_config_t c = cfg_first_test();
        sbsim_init(&y, 952u, 0.0f, 0.0f, &c);
        link_init(&usb2, &y.sb);
        sim_hal_set_joystick(false, joy_raw(0.0f), joy_raw(0.0f), false, false);
        sbsim_run(&y, 3000u, NULL, NULL);
        uint8_t r_ft = bench_cmd(&y, &usb2, SB_BENCH_ENTER, 0u);
        bool ft = sb_mode(&y.sb) == MODE_BENCH;
        sim_hal_set_joystick(true, joy_raw(0.0f), joy_raw(0.0f), false, false);
        snprintf(d, sizeof d, "boat LAN station: %u (5 = not allowed); USB laptop: %u -> BENCH %s; BENCH_DIAG frames in 2 s: "
                 "STANDBY %d, BENCH %d; leave -> STANDBY %s; from HDG LOCK: %u (7 = wrong mode); e-stop pressed: %u "
                 "(6 = conditions); pulse outside BENCH: %u; moving %.1f kn: %u; FIRST TEST build: %u -> BENCH %s",
                 r_gw, r_usb, in_bench ? "yes" : "NO", diag_standby, diag_bench, left ? "yes" : "NO", r_mode, r_estop,
                 r_cmd_out, kn, r_moving, r_ft, ft ? "yes" : "NO");
        test_report("BT1 BENCH TEST: USB laptop only, from STANDBY, not moving, not with the e-stop; diag frames; FIRST TEST too",
                    r_gw == PROTO_R_NOT_ALLOWED && r_usb == PROTO_R_OK && in_bench && diag_standby == 0 &&
                    diag_bench >= 15 && left && r_mode == PROTO_R_WRONG_MODE && r_estop == PROTO_R_PRECOND &&
                    r_cmd_out == PROTO_R_WRONG_MODE && kn > 2.0f && r_moving == PROTO_R_PRECOND && r_ft == PROTO_R_OK && ft, d);
    }

    /* BT2 steering pulses: left / right for the commanded time, estimate follows; too long, too soon,
     * toward an active end stop: refused; throttle idle + neutral the whole time (the steering MOTOR; the
     * DroneCAN position servo: CN19 steering steps) */
    if (s_can_all) {
        test_na("BT2 BENCH TEST steering pulses: as commanded, estimate follows; too long / too soon / toward an end stop refused",
                "steering motor only (position servo: CN19)");
    } else {
        static sbsim_t x;
        static link_t usb;
        btwatch_t b;
        sbsim_init(&x, 953u, 0.0f, 0.0f, NULL);
        link_init(&usb, &x.sb);
        x.w.limit_switches = true;
        sbsim_run(&x, 3000u, NULL, NULL);
        bt_watch_init(&b, &usb);
        uint8_t r_in = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        uint8_t r_l = bench_cmd(&x, &usb, SB_BENCH_LEFT, 200u);
        uint8_t r_soon = bench_cmd(&x, &usb, SB_BENCH_RIGHT, 100u);       /* the left one still runs */
        sbsim_run(&x, x.t + 1000u, bt_hook, &b);
        int l_steps = b.left_steps, r_steps0 = b.right_steps;
        float est_l = x.sb.ap.est_angle + x.sb.ap.slack_deg;       /* motor position: backlash first */
        uint8_t r_r = bench_cmd(&x, &usb, SB_BENCH_RIGHT, 200u);
        sbsim_run(&x, x.t + 1000u, bt_hook, &b);
        int r_steps = b.right_steps;
        float est_r = x.sb.ap.est_angle + x.sb.ap.slack_deg;
        uint8_t r_long = bench_cmd(&x, &usb, SB_BENCH_LEFT, 400u);
        uint8_t r_short = bench_cmd(&x, &usb, SB_BENCH_LEFT, 50u);
        x.w.rud.angle_deg = x.w.rud.drive_deg = -x.w.rud.max_angle_neg_deg;   /* at the left end stop (by hand) */
        sbsim_run(&x, x.t + 300u, bt_hook, &b);
        uint8_t r_lim = bench_cmd(&x, &usb, SB_BENCH_LEFT, 100u);
        uint8_t why = x.sb.bench.last_refuse;
        int l_before = b.left_steps;
        sbsim_run(&x, x.t + 500u, bt_hook, &b);
        uint8_t r_away = bench_cmd(&x, &usb, SB_BENCH_RIGHT, 100u);
        sbsim_run(&x, x.t + 1000u, bt_hook, &b);
        x.w.limit_switches = false;
        snprintf(d, sizeof d, "enter %u; left 200 ms: %u, motor left %d steps, right %d, motor position (estimate + backlash) %.2f deg; "
                 "right while it runs: %u; right 200 ms: %u, %d steps, position %.2f; 400 ms: %u, 50 ms: %u; at the left end stop: left %u "
                 "(refuse %u = end stop, no motor %s), right %u; throttle max %.1f %%, gear %u..%u (N %u)",
                 r_in, r_l, l_steps, r_steps0, est_l, r_soon, r_r, r_steps, est_r, r_long, r_short, r_lim, why,
                 b.left_steps == l_before ? "yes" : "NO", r_away, b.thr_max, b.gear_min, b.gear_max,
                 x.sb.cfg.gear.pwm_n_us);
        test_report("BT2 BENCH TEST steering pulses: as commanded, estimate follows; too long / too soon / toward an end stop refused",
                    r_in == PROTO_R_OK && r_l == PROTO_R_OK && l_steps >= 3 && l_steps <= 5 && r_steps0 == 0 && est_l < -0.2f &&
                    r_soon == PROTO_R_PRECOND && r_r == PROTO_R_OK && r_steps >= 3 && r_steps <= 5 && est_r > est_l + 0.2f &&
                    r_long == PROTO_R_PRECOND && r_short == PROTO_R_PRECOND && r_lim == PROTO_R_PRECOND &&
                    why == SB_BENCH_R_LIMIT && b.left_steps == l_before && r_away == PROTO_R_OK && b.thr_max <= 0.0f &&
                    b.gear_min == x.sb.cfg.gear.pwm_n_us && b.gear_max == x.sb.cfg.gear.pwm_n_us, d);
    }

    /* BT3 panel buttons: each one is shown, none changes the mode - except STANDBY, which always wins */
    {
        static sbsim_t x;
        static link_t usb;
        sbsim_init(&x, 954u, 0.0f, 0.0f, NULL);
        link_init(&usb, &x.sb);
        sbsim_run(&x, 3000u, NULL, NULL);
        bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        uint32_t log0 = x.sb.modes.log_count;
        press(&x, HAL_BTN_HDG_LOCK);
        press(&x, HAL_BTN_MINUS_1);
        press(&x, HAL_BTN_PLUS_1);
        press(&x, HAL_BTN_MINUS_10);
        press(&x, HAL_BTN_PLUS_10);
        press(&x, HAL_BTN_PLUS_10);
        bool still = sb_mode(&x.sb) == MODE_BENCH;
        uint8_t seen = x.sb.bench.buttons_seen;
        uint8_t n10 = x.sb.bench.button_count[HAL_BTN_PLUS_10];
        uint32_t logged = x.sb.modes.log_count - log0;
        press(&x, HAL_BTN_STANDBY);
        bool sb = sb_mode(&x.sb) == MODE_STANDBY;
        uint8_t want = (uint8_t)(HAL_BTN_BIT(HAL_BTN_HDG_LOCK) | HAL_BTN_BIT(HAL_BTN_MINUS_1) | HAL_BTN_BIT(HAL_BTN_PLUS_1) |
                                 HAL_BTN_BIT(HAL_BTN_MINUS_10) | HAL_BTN_BIT(HAL_BTN_PLUS_10));
        snprintf(d, sizeof d, "after HDG LOCK, -1, +1, -10, +10 x2: mode BENCH %s, seen 0x%02X (want 0x%02X), +10 count %u, "
                 "mode requests logged meanwhile %lu (want 0); STANDBY -> STANDBY %s",
                 still ? "yes" : "NO", seen, want, n10, (unsigned long)logged, sb ? "yes" : "NO");
        test_report("BT3 BENCH TEST buttons: each one shown, none changes the mode; STANDBY always wins",
                    still && seen == want && n10 == 2u && logged == 0u && sb, d);
    }

    /* BT4 it ends by itself: 10 min without a command, the USB laptop unplugged, the e-stop;
     * the joystick takes over (normal build) */
    {
        static sbsim_t x;
        static link_t usb;
        btwatch_t b;
        sbsim_init(&x, 955u, 0.0f, 0.0f, NULL);
        link_init(&usb, &x.sb);
        sim_hal_set_estop(true, false);
        sbsim_run(&x, 3000u, NULL, NULL);
        bt_watch_init(&b, &usb);
        bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        uint32_t t0 = x.t;
        while (sb_mode(&x.sb) == MODE_BENCH && x.t < t0 + 700000u) sbsim_run(&x, x.t + 1000u, bt_hook, &b);
        float idle_s = (x.t - t0) * 0.001f;
        bool idle_end = sb_mode(&x.sb) == MODE_STANDBY;
        bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        b.silent = true;                                            /* unplugged */
        t0 = x.t;
        while (sb_mode(&x.sb) == MODE_BENCH && x.t < t0 + 10000u) sbsim_run(&x, x.t + AP_PERIOD_MS, bt_hook, &b);
        float lost_s = (x.t - t0) * 0.001f;
        bool lost_end = sb_mode(&x.sb) == MODE_STANDBY;
        b.silent = false;
        sbsim_run(&x, x.t + 2000u, bt_hook, &b);
        sb_alarm_ack(&x.sb);
        uint8_t r_back = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        sim_hal_set_estop(true, true);
        sbsim_run(&x, x.t + 100u, bt_hook, &b);
        bool estop_end = sb_mode(&x.sb) == MODE_STANDBY && (x.sb.alarm_bits & SB_ALM_ESTOP);
        sim_hal_set_estop(true, false);
        sbsim_run(&x, x.t + 300u, bt_hook, &b);
        press(&x, HAL_BTN_STANDBY);
        bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        bool again = sb_mode(&x.sb) == MODE_BENCH;
        joy_set(0.6f, 0.0f, false, false);
        sbsim_run(&x, x.t + 500u, bt_hook, &b);
        bool manual = sb_mode(&x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        sim_hal_set_estop(false, false);
        const mode_log_entry_t *e = NULL;
        snprintf(d, sizeof d, "no command: STANDBY after %.0f s %s; laptop unplugged: STANDBY after %.1f s %s; back: %u; "
                 "e-stop -> STANDBY + alarm %s; again %s; joystick deflected -> MANUAL %s",
                 idle_s, idle_end ? "yes" : "NO", lost_s, lost_end ? "yes" : "NO", r_back, estop_end ? "yes" : "NO",
                 again ? "yes" : "NO", manual ? "yes" : "NO");
        (void)e;
        test_report("BT4 BENCH TEST ends by itself: 10 min idle, laptop unplugged, e-stop; the joystick takes over",
                    idle_end && idle_s >= 599.0f && idle_s <= 602.0f && lost_end && lost_s <= 4.0f && r_back == PROTO_R_OK &&
                    estop_end && again && manual, d);
    }

    /* BT5 buzzer test: the alarm output for the asked time, then off; out of range refused */
    {
        static sbsim_t x;
        static link_t usb;
        btwatch_t b;
        sbsim_init(&x, 956u, 0.0f, 0.0f, NULL);
        link_init(&usb, &x.sb);
        sbsim_run(&x, 3000u, NULL, NULL);
        bt_watch_init(&b, &usb);
        bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        sbsim_run(&x, x.t + 500u, bt_hook, &b);
        int before = b.alarm_steps;
        uint8_t r = bench_cmd(&x, &usb, SB_BENCH_BUZZER, 1000u);
        sbsim_run(&x, x.t + 3000u, bt_hook, &b);
        int on_steps = b.alarm_steps - before;
        bool off = !sim_hal_alarm();
        uint8_t r_long = bench_cmd(&x, &usb, SB_BENCH_BUZZER, 5000u);
        snprintf(d, sizeof d, "silent before %s; buzzer 1 s: %u, on for %d steps of 50 ms, off after %s; 5 s: %u",
                 before == 0 ? "yes" : "NO", r, on_steps, off ? "yes" : "NO", r_long);
        test_report("BT5 BENCH TEST buzzer: on for the asked time, then off",
                    before == 0 && r == PROTO_R_OK && on_steps >= 19 && on_steps <= 21 && off && r_long == PROTO_R_PRECOND, d);
    }

    /* BT6 monitor trip test (R3 on the board): a wrong code does nothing; the right one = a real trip,
     * latched: outputs forced safe, STANDBY, MONITOR alarm, nothing can leave STANDBY */
    {
        static sbsim_t x;
        static link_t usb;
        btwatch_t b;
        sbsim_init(&x, 957u, 0.0f, 0.0f, NULL);
        link_init(&usb, &x.sb);
        sbsim_run(&x, 3000u, NULL, NULL);
        bt_watch_init(&b, &usb);
        uint8_t r_out = bench_cmd(&x, &usb, SB_BENCH_TRIP, SB_BENCH_TRIP_CODE);   /* not in BENCH */
        bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        uint8_t r_bad = bench_cmd(&x, &usb, SB_BENCH_TRIP, 1234u);
        sbsim_run(&x, x.t + 500u, bt_hook, &b);
        bool no_trip = !x.mon.tripped;
        uint8_t r_ok = bench_cmd(&x, &usb, SB_BENCH_TRIP, SB_BENCH_TRIP_CODE);
        sbsim_run(&x, x.t + 500u, bt_hook, &b);
        bool tripped = x.mon.tripped && x.mon.reason == MON_TRIP_TEST;
        bool forced = hal_outputs_forced() && !sim_hal_safety_relay() && sim_hal_alarm();
        bool sb = sb_mode(&x.sb) == MODE_STANDBY && (x.sb.alarm_bits & SB_ALM_MONITOR);
        uint8_t r_again = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        press(&x, HAL_BTN_HDG_LOCK);
        bool stays = sb_mode(&x.sb) == MODE_STANDBY;
        snprintf(d, sizeof d, "outside BENCH: %u; wrong code: %u, no trip %s; right code: %u -> trip '%s' %s, outputs forced + "
                 "relay open + alarm %s, STANDBY + MONITOR alarm %s; enter BENCH again: %u, HDG LOCK: stays STANDBY %s",
                 r_out, r_bad, no_trip ? "yes" : "NO", r_ok, monitor_reason_name(x.mon.reason), tripped ? "yes" : "NO",
                 forced ? "yes" : "NO", sb ? "yes" : "NO", r_again, stays ? "yes" : "NO");
        test_report("BT6 BENCH TEST monitor trip test: only with the code; a real latched trip (outputs safe, STANDBY)",
                    r_out == PROTO_R_WRONG_MODE && r_bad == PROTO_R_WRONG_CODE && no_trip && r_ok == PROTO_R_OK && tripped &&
                    forced && sb && r_again == PROTO_R_PRECOND && stays, d);
        sim_hal_unforce();
    }
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
    crew_tests();
    lostlink_tests();
    save_tests();
    ais_tests();
    nav_tests();
}

void sb_slam_tests(void)
{
    slam_tests();
}

/* --- DF: data flash / non-volatile storage ---------------------------------- */

/* a tiny flash in RAM for the board's record log (hal/tc_nv.c), instant commands */
#define DF_SECTOR 512u
#define DF_REGION 1024u
static uint8_t  df_img[2][DF_REGION];
static uint32_t df_cmds;
static bool     df_busy;
static struct { bool erase; uint8_t region; uint32_t off; uint32_t w0, w1; } df_cmd;
static bool df_read(uint8_t r, uint32_t off, void *buf, uint32_t len)
{
    if (r > 1u || off + len > DF_REGION || df_busy) return false;
    memcpy(buf, &df_img[r][off], len);
    return true;
}
static bool df_erase(uint8_t r, uint32_t sector)
{
    if (df_busy || r > 1u || (sector + 1u) * DF_SECTOR > DF_REGION) return false;
    df_cmd.erase = true; df_cmd.region = r; df_cmd.off = sector * DF_SECTOR;
    df_busy = true; df_cmds++;
    return true;
}
static bool df_write(uint8_t r, uint32_t off, uint32_t w0, uint32_t w1)
{
    if (df_busy || r > 1u || off + 8u > DF_REGION) return false;
    df_cmd.erase = false; df_cmd.region = r; df_cmd.off = off; df_cmd.w0 = w0; df_cmd.w1 = w1;
    df_busy = true; df_cmds++;
    return true;
}
static bool df_is_busy(void)
{
    if (df_busy) {                              /* finishes at the first poll */
        uint8_t *p = &df_img[df_cmd.region][df_cmd.off];
        if (df_cmd.erase) memset(p, 0, DF_SECTOR);
        else { memcpy(p, &df_cmd.w0, 4u); memcpy(p + 4, &df_cmd.w1, 4u); }
        df_busy = false;
    }
    return false;
}
static bool df_failed(void) { return false; }
static uint32_t df_us;
static uint32_t df_micros(void) { return df_us += 5u; }

static void nv_tests(void)
{
    char d[700];
    printf("\n=== DF data flash (non-volatile storage) ===\n");

    /* DF1 a stored record with a bad checksum -> defaults + NV DATA RESET notice (log event 23) */
    {
        static sbsim_t x;
        sbsim_init(&x, 1401u, 90.0f, 0.0f, NULL);
        joymap_t m;
        joymap_default(&m);
        m.nz_lo = 0.40f;                                   /* not the default */
        joymap_seal(&m);
        FILE *f = fopen("nv4.bin", "wb");
        if (f) { uint8_t *b = (uint8_t *)&m; b[20] ^= 0x5Au; fwrite(&m, 1, sizeof m, f); fclose(f); }
        f = fopen("nv1.bin", "wb");                        /* joystick calibration: 28 bytes of nothing */
        if (f) { uint8_t z[28] = {0}; fwrite(z, 1, sizeof z, f); fclose(f); }
        hal_init();
        sb_init(&x.sb, &x.cfg);
        const nv_status_t *st = nv_status();
        bool defaults = x.sb.jmap.nz_lo != 0.40f && joymap_valid(&x.sb.jmap);
        bool flagged = (st->reset_mask & (1u << HAL_NV_SLOT_JOYMAP)) && (st->reset_mask & (1u << HAL_NV_SLOT_JOYSTICK)) &&
                       st->resets == 2u;
        /* the notice reaches the log: event 23 NV, one per slot (core 0's log producer) */
        logq_init_all();
        logcore0_init(&x.sb, 3u);
        logcore0_step(&x.sb);
        int ev_nv = 0;
        {
            static uint8_t rec[LOGR_MAX];
            uint16_t n;
            while ((n = logq_pop(&g_logq0, rec, sizeof rec)) != 0u) {
                if (n >= LOGR_HDR + 2u && rec[1] == LOGR_EVENT && (rec[8] | (rec[9] << 8)) == EV_NV) ev_nv++;
            }
        }
        snprintf(d, sizeof d, "corrupted joystick map + calibration on 'flash': defaults in use %s; reset mask 0x%x "
                 "(resets %u) %s; log events NV: %d", defaults ? "yes" : "NO", st->reset_mask, st->resets,
                 flagged ? "both slots flagged" : "NOT flagged", ev_nv);
        test_report("DF1 bad checksum in the flash -> defaults + NV DATA RESET notice (log event 23)",
                    defaults && flagged && ev_nv == 2, d);
        remove("nv1.bin"); remove("nv4.bin");
    }

    /* DF2 a power loss during the write keeps the old record */
    {
        static sbsim_t x;
        sbsim_init(&x, 1402u, 90.0f, 0.0f, NULL);
        joymap_t a, b, back;
        joymap_default(&a); a.nz_lo = 0.41f; joymap_seal(&a);
        joymap_default(&b); b.nz_lo = 0.42f; joymap_seal(&b);
        uint32_t w0 = sim_hal_nv_writes();
        bool qa = nv_save(HAL_NV_SLOT_JOYMAP, &a, (uint32_t)sizeof a);
        sbsim_run(&x, x.t + 200u, NULL, NULL);            /* STANDBY: written */
        bool wrote_a = sim_hal_nv_writes() == w0 + 1u && !nv_status()->pending;
        sim_hal_nv_fault(SIM_NV_POWER_LOSS);
        bool qb = nv_save(HAL_NV_SLOT_JOYMAP, &b, (uint32_t)sizeof b);
        bool shadow = nv_load(HAL_NV_SLOT_JOYMAP, &back, (uint32_t)sizeof back) && back.nz_lo == 0.42f;
        sbsim_run(&x, x.t + 200u, NULL, NULL);            /* the write is cut */
        hal_init();                                        /* reboot */
        sb_init(&x.sb, &x.cfg);
        bool old_kept = x.sb.jmap.nz_lo == 0.41f && sim_hal_nv_writes() == w0 + 1u;
        snprintf(d, sizeof d, "A queued %s + written in STANDBY %s; B queued %s (a load sees B meanwhile %s); power "
                 "loss during B's write: after the reboot the map is %s (nz_lo %.2f), writes %u, lost %u",
                 qa ? "yes" : "NO", wrote_a ? "yes" : "NO", qb ? "yes" : "NO", shadow ? "yes" : "NO",
                 old_kept ? "A (the old record)" : "NOT A", x.sb.jmap.nz_lo, sim_hal_nv_writes() - w0, sim_hal_nv_lost());
        test_report("DF2 power loss during a write keeps the old record", qa && wrote_a && qb && shadow && old_kept, d);
        remove("nv4.bin");
    }

    /* DF3 an old record version (valid checksum) is refused: defaults + notice */
    {
        static sbsim_t x;
        sbsim_init(&x, 1403u, 90.0f, 0.0f, NULL);
        calib_data_t c;
        memset(&c, 0, sizeof c);
        c.gain_dps_per_ms = 0.01f; c.time_const_s = 2.0f; c.travel_right_ms = 3000.0f; c.travel_left_ms = 3000.0f;
        c.gap_ms = 100.0f; c.full_angle_deg = 60.0f;
        calib_seal(&c);
        c.version = CALIB_VERSION + 1u;                    /* a format this firmware does not know */
        c.checksum = calib_checksum(&c);
        FILE *f = fopen("nv5.bin", "wb");
        if (f) { fwrite(&c, 1, sizeof c, f); fclose(f); }
        hal_init();
        sb_init(&x.sb, &x.cfg);
        bool loaded = autopilot_load_calibration(&x.sb.ap);
        const nv_status_t *st = nv_status();
        bool flagged = (st->reset_mask & (1u << HAL_NV_SLOT_CALIB)) != 0u;
        calib_seal(&c);                                    /* the right version: accepted */
        f = fopen("nv5.bin", "wb");
        if (f) { fwrite(&c, 1, sizeof c, f); fclose(f); }
        hal_init();
        sb_init(&x.sb, &x.cfg);
        bool loaded2 = autopilot_load_calibration(&x.sb.ap);
        bool v1_refused = x.sb.steer.v1_refused && (nv_status()->reset_mask & (1u << HAL_NV_SLOT_CALIB));
        snprintf(d, sizeof d, "TRAINING result with version %u on 'flash': loaded %s, NV DATA RESET flagged %s; the same "
                 "record with version %u: loaded %s%s", CALIB_VERSION + 1u, loaded ? "YES" : "no", flagged ? "yes" : "NO",
                 CALIB_VERSION, loaded2 ? "yes" : "NO",
                 x.sb.servo.can ? (v1_refused ? " (position servo: a DC-motor record refused + flagged, as wanted)"
                                              : " (position servo: v1 NOT refused / flagged)") : "");
        /* the DroneCAN position servo never uses a version-1 (DC-motor) record (SM6): refused + flagged */
        bool ok2 = x.sb.servo.can ? (!loaded2 && v1_refused) : loaded2;
        test_report("DF3 unknown record version refused (defaults + notice), the right one loads", !loaded && flagged && ok2, d);
        remove("nv5.bin");
    }

    /* DF4 the board's record log: sector swap carries every live record over; a power
     * loss in the middle of the swap loses nothing (both regions scanned at init) */
    {
        nvlog_port_t port = { DF_REGION, DF_SECTOR, 0u, 1000000u, df_read, df_erase, df_write, df_is_busy, df_failed, df_micros };
        memset(df_img, 0, sizeof df_img);
        df_busy = false;
        nvlog_init(&port);
        uint8_t v[6][64];
        uint32_t lens[6] = { 4u, 28u, 48u, 64u, 12u, 20u };
        bool ok = nvlog_ok(), kept = true;
        uint32_t swaps_seen = 0u, n = 0u;
        for (uint32_t round = 0; round < 100u && ok; round++) {
            uint8_t s = (uint8_t)(1u + round % 6u);
            for (uint32_t i = 0; i < lens[s - 1u]; i++) v[s - 1u][i] = (uint8_t)(round * 7u + i + s);
            ok = ok && nvlog_save(s, v[s - 1u], lens[s - 1u]);
            for (int k = 0; k < 2000 && nvlog_pending(); k++) nvlog_service(true, 0u);
            ok = ok && !nvlog_pending() && nvlog_ok();
            n++;
            if (nvlog_stats()->swaps > swaps_seen) {
                swaps_seen = nvlog_stats()->swaps;
                nvlog_init(&port);                         /* a reboot right after a swap */
            }
            for (uint8_t t = 1; t <= 6u && round >= 5u; t++) {
                uint8_t back[64];
                if (!nvlog_load(t, back, lens[t - 1u]) || memcmp(back, v[t - 1u], lens[t - 1u]) != 0) kept = false;
            }
        }
        /* a power loss while a swap is copying: cut after the first copied record */
        uint32_t cmds0 = df_cmds, swaps0 = nvlog_stats()->swaps;
        bool cut_ok = true, cut_hit = false;
        for (uint32_t round = 0; round < 60u && nvlog_stats()->swaps == swaps0; round++) {
            uint8_t s = (uint8_t)(1u + round % 6u);
            for (uint32_t i = 0; i < lens[s - 1u]; i++) v[s - 1u][i] = (uint8_t)(round * 3u + i + 9u * s);
            (void)nvlog_save(s, v[s - 1u], lens[s - 1u]);
            for (int k = 0; k < 2000 && nvlog_pending() && nvlog_stats()->swaps == swaps0; k++) nvlog_service(true, 0u);
            if (nvlog_stats()->swaps > swaps0) {
                cut_hit = true;
                /* the swap began: let ~2 records copy, then "power loss" = re-init without the rest */
                for (int k = 0; k < 6; k++) nvlog_service(true, 0u);
                nvlog_init(&port);
                for (int k = 0; k < 4000 && nvlog_pending(); k++) nvlog_service(true, 0u);   /* completes the swap */
                for (uint8_t t = 1; t <= 6u; t++) {
                    uint8_t back[64];
                    bool is_new = (t == s);
                    bool got = nvlog_load(t, back, lens[t - 1u]);
                    /* the slot being saved may show the old or the new value, every other the old */
                    if (!got) cut_ok = false;
                    else if (!is_new && memcmp(back, v[t - 1u], lens[t - 1u]) != 0) cut_ok = false;
                }
            }
        }
        snprintf(d, sizeof d, "%u saves into 2 x %u B regions: %u swaps, every slot reads its newest value after each "
                 "swap + reboot %s; power loss while a swap copies: nothing lost %s (%u flash commands, log ok %s)",
                 n, DF_REGION, swaps_seen, kept ? "yes" : "NO", cut_ok ? "yes" : "NO", df_cmds - cmds0,
                 nvlog_ok() ? "yes" : "NO");
        test_report("DF4 record log: sector swap keeps every live record, also through a power loss",
                    ok && kept && swaps_seen >= 2u && cut_hit && cut_ok && nvlog_ok(), d);
    }

    /* DF5 a save never blocks a step and never runs in a driving mode; a stuck flash = NV FAULT, the loop goes on */
    {
        static sbsim_t x;
        sbsim_init(&x, 1405u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 2000u, NULL, NULL);
        bool manual = goto_mode(&x, MODE_MANUAL);
        uint32_t w0 = sim_hal_nv_writes();
        joymap_t m;
        joymap_default(&m); m.nz_lo = 0.43f; joymap_seal(&m);
        bool queued = nv_save(HAL_NV_SLOT_JOYMAP, &m, (uint32_t)sizeof m);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool waited = sim_hal_nv_writes() == w0 && nv_status()->pending && nv_status()->pending_ms >= 2900u;
        uint32_t max_us_driving = g_timing.core0.max_us;
        joy_set(0.0f, 0.0f, false, false);                 /* stick centred, STANDBY button: written at once */
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 2u * AP_PERIOD_MS, NULL, NULL);
        bool written = sb_mode(&x.sb) == MODE_STANDBY && sim_hal_nv_writes() == w0 + 1u && !nv_status()->pending;
        /* a flash that never finishes: NV FAULT after the time limit, the control loop never waits */
        sim_hal_nv_fault(SIM_NV_STUCK);
        m.nz_lo = 0.44f; joymap_seal(&m);
        (void)nv_save(HAL_NV_SLOT_JOYMAP, &m, (uint32_t)sizeof m);
        uint32_t t0 = x.t, steps0 = g_timing.core0.w_n;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool fault = nv_status()->fault && !hal_nv_ok();
        bool refused = !nv_save(HAL_NV_SLOT_JOYMAP, &m, (uint32_t)sizeof m);
        bool kept_stepping = g_timing.core0.w_n - steps0 >= 35u && g_timing.core0.max_us < 20000u && x.resets == 0u;
        /* a load meanwhile returns the value that was asked for (the queued one), the
         * reboot shows what the flash really holds: the record written in STANDBY */
        joymap_t back;
        bool loads = nv_load(HAL_NV_SLOT_JOYMAP, &back, (uint32_t)sizeof back) && back.nz_lo == 0.44f;
        uint32_t pend_ms = nv_status()->pending_ms;
        sim_hal_nv_fault(0u);
        hal_init();
        sb_init(&x.sb, &x.cfg);
        bool after_reboot = x.sb.jmap.nz_lo == 0.43f && !nv_status()->fault;
        snprintf(d, sizeof d, "MANUAL: save queued %s, nothing written for 3 s %s, longest step %u us; "
                 "STANDBY: written %s; stuck flash: NV FAULT %s after %u ms (pending %u ms), further saves refused %s, "
                 "a load meanwhile %s, loop kept stepping %s (max step %u us, resets %u); after the reboot: the record "
                 "written before the fault %s",
                 queued ? "yes" : "NO", waited ? "yes" : "NO", max_us_driving,
                 written ? "yes" : "NO", fault ? "yes" : "NO", x.t - t0, pend_ms, refused ? "yes" : "NO",
                 loads ? "works" : "FAILS", kept_stepping ? "yes" : "NO", g_timing.core0.max_us, x.resets,
                 after_reboot ? "yes" : "NO");
        test_report("DF5 a save never blocks a step and never runs while driving; a stuck flash = NV FAULT only",
                    manual && queued && waited && written && fault && refused && loads && kept_stepping && after_reboot, d);
        remove("nv4.bin");
    }
}

/* ------------------------------------------ BV: supply voltage (section J) -- */

/* ----------------------------------------------- GN generator mode (generator.c) -- */

/* The bank as the tests need it: a fixed voltage, or one that falls slowly
 * (drain) and is lifted by the engine's alternator with the throttle; or no
 * data at all. Also watches every stop: once the generator is STOPPING the
 * throttle output must never rise until idle, and in STANDBY with the
 * generator not active the throttle must be idle in neutral. */
typedef struct {
    bool  none;                 /* not measured */
    bool  fixed;
    float fixed_v;
    float base_v, drain_v_s, alt_v_pct;
    uint32_t t0;
    /* watch */
    float prev_out;
    int   rises_in_stop;        /* output rose while STOPPING */
    int   stray;                /* throttle in neutral without the generator */
    float max_out;              /* highest output seen */
    float max_demand;
    int   gear_not_n;           /* gear left N while the generator was active */
    bool  started;              /* RUNNING seen */
} gensup_t;

static void gen_hook(sbsim_t *x, void *ctx)
{
    gensup_t *g = (gensup_t *)ctx;
    float out = x->sb.thr.out_pct;
    if (g->none) sim_hal_set_supply(false, 0.0f);
    else if (g->fixed) sim_hal_set_supply(true, g->fixed_v);
    else sim_hal_set_supply(true, g->base_v - g->drain_v_s * ((x->t - g->t0) * 0.001f) + g->alt_v_pct * out);
    const gen_t *gen = &x->sb.gen;
    if (gen->state == GEN_STOPPING && out > g->prev_out + 0.01f) g->rises_in_stop++;
    if (sb_mode(&x->sb) == MODE_STANDBY && !gen->active && gear_is(&x->sb.gear, GEAR_N) &&
        out > x->sb.cfg.gear.idle_max_pct + 0.01f && out > g->prev_out + 0.01f) g->stray++;
    if (gen->active && !gear_is(&x->sb.gear, GEAR_N) && x->sb.gear.target != GEAR_N &&
        out > x->sb.cfg.gear.idle_max_pct) g->gear_not_n++;
    if (gen->state == GEN_RUNNING) g->started = true;
    if (out > g->max_out) g->max_out = out;
    if (gen->demand_pct > g->max_demand) g->max_demand = gen->demand_pct;
    g->prev_out = out;
}

/* GN6: the "bug" where core 0 forgets to publish gen_active (cleared after every control step) */
static void gen_clear_hook(sbsim_t *x, void *ctx)
{
    gen_hook(x, ctx);
    g_mon.gen_active = 0u;
}

static void gensup_fixed(gensup_t *g, float v)
{
    memset(g, 0, sizeof *g);
    g->fixed = true;
    g->fixed_v = v;
}

/* a boat at the dock: neutral switch fitted (follows the gearbox), the bank
 * at v0 from the first sample (the slow filter starts there), armed in
 * STANDBY after 2 s */
static void gen_setup(sbsim_t *x, uint32_t seed, const sb_config_t *cfg, float v0)
{
    sbsim_init(x, seed, 90.0f, 0.0f, cfg);
    x->w.neutral_switch = true;
    x->w.neutral_sw_force = 0;
    sim_hal_set_supply(true, v0);
    sbsim_run(x, 2000u, NULL, NULL);
}

static bool gen_running_at(const sbsim_t *x, float pct)
{
    return x->sb.gen.state == GEN_RUNNING && fabsf(x->sb.gen.demand_pct - pct) < 0.01f &&
           fabsf(x->sb.thr.out_pct - pct) < 0.6f && gear_is(&x->sb.gear, GEAR_N) && sb_mode(&x->sb) == MODE_STANDBY;
}

static void generator_tests(void)
{
    char d[1200];
    printf("\n=== GN generator mode (generator.c): the ONE automatic throttle rise, in NEUTRAL only ===\n");

    /* GN1 the start conditions one by one: nothing starts until ALL hold; then STARTING (beep) -> 10 s -> the
     * fast idle in neutral, CPU1 told (gen_active), the boat does not move */
    {
        static sbsim_t x;
        static gensup_t g;
        int n_ok = 0, len = 0;
        d[0] = 0;
#define GN1_CASE(name, cond) do { bool c_ = (cond); if (c_) { n_ok++; } \
        len += snprintf(d + len, sizeof d - (size_t)len, "%s %s; ", name, c_ ? "ok" : "FAILED"); } while (0)
        /* a. auto off: a low bank alone starts nothing */
        gen_setup(&x, 1601u, NULL, 12.6f);
        gensup_fixed(&g, 11.5f);
        sbsim_run(&x, x.t + 40000u, gen_hook, &g);
        GN1_CASE("auto off", x.sb.gen.state == GEN_OFF && x.sb.thr.out_pct == 0.0f && !x.sb.gen.active &&
                 x.sb.gen.cfg.run_ms == 1200000u && x.sb.gen.cfg.rest_ms == 60000u && x.sb.gen.cfg.max_pct == 25.0f);
        /* b. auto on, bank fine (12.6 V): READY, waiting */
        gensup_fixed(&g, 12.6f);
        sbsim_run(&x, x.t + 40000u, gen_hook, &g);
        bool on_ok = sb_generator_set(&x.sb, true) == SB_OK;
        sbsim_run(&x, x.t + 40000u, gen_hook, &g);
        GN1_CASE("bank fine", on_ok && x.sb.gen.state == GEN_READY && x.sb.thr.out_pct == 0.0f);
        /* c. bank low but the neutral switch not fitted: never */
        x.w.neutral_switch = false;
        gensup_fixed(&g, 11.5f);
        sbsim_run(&x, x.t + 60000u, gen_hook, &g);
        GN1_CASE("no neutral switch", x.sb.gen.state == GEN_READY && x.sb.neutral_sw == -1 && x.sb.thr.out_pct == 0.0f);
        /* d. switch fitted but open (says "in gear") */
        x.w.neutral_switch = true;
        x.w.neutral_sw_force = 1;
        sbsim_run(&x, x.t + 40000u, gen_hook, &g);
        GN1_CASE("switch open", x.sb.gen.state == GEN_READY && x.sb.neutral_sw == 0 && x.sb.thr.out_pct == 0.0f);
        x.w.neutral_sw_force = 0;
        /* e. a non-advisory alarm (compass lost in STANDBY = alarm only) blocks it; acknowledged -> starts */
        sim_hal_fail_heading(true);
        sbsim_run(&x, x.t + 20000u, gen_hook, &g);
        bool alarm_blocks = (x.sb.alarm_bits & SB_ALM_COMPASS) && x.sb.gen.state == GEN_READY && x.sb.thr.out_pct == 0.0f;
        sim_hal_fail_heading(false);
        sbsim_run(&x, x.t + 5000u, gen_hook, &g);
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 100u, gen_hook, &g);
        bool starting = x.sb.gen.state == GEN_STARTING && x.sb.thr.out_pct == 0.0f && x.sb.chirp_ms > 0u;
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        GN1_CASE("alarm blocks, ACK -> STARTING (beep) -> RUNNING 8 %",
                 alarm_blocks && starting && gen_running_at(&x, 8.0f) && x.sb.gen.active && g_mon.gen_active == 1u &&
                 fabsf(g_mon.gen_cap_pct - 25.0f) < 0.01f && !x.mon.tripped && x.w.speed_kn < 0.2f &&
                 x.w.engaged == GEAR_N && x.w.shift_under_load == 0 && sim_hal_safety_relay());
        /* f. e-stop: stops (idle first) and blocks a restart while pressed */
        sim_hal_set_estop(true, true);
        sbsim_run(&x, x.t + 3000u, gen_hook, &g);
        bool estop_stops = x.sb.gen.state != GEN_RUNNING && x.sb.thr.out_pct == 0.0f && sb_mode(&x.sb) == MODE_STANDBY;
        sim_hal_set_estop(true, false);
        sbsim_run(&x, x.t + 3000u, gen_hook, &g);
        bool estop_alarm_blocks = x.sb.gen.state == GEN_READY && x.sb.thr.out_pct == 0.0f;
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        GN1_CASE("e-stop stops + blocks until ACK", estop_stops && estop_alarm_blocks && gen_running_at(&x, 8.0f) &&
                 g.rises_in_stop == 0 && g.stray == 0 && g.gear_not_n == 0);
        /* g. gen_max_pct 0 = disabled: the switch is refused */
        {
            static sbsim_t y;
            gen_setup(&y, 1602u, NULL, 11.5f);
            y.sb.gen.cfg.max_pct = 0.0f;
            bool refused = sb_generator_set(&y.sb, true) == SB_ERR_PRECOND && y.sb.gen.state == GEN_OFF;
            gensup_fixed(&g, 11.5f);
            sbsim_run(&y, y.t + 20000u, gen_hook, &g);
            GN1_CASE("gen_max_pct 0 refused", refused && y.sb.gen.state == GEN_OFF && y.sb.thr.out_pct == 0.0f);
        }
        /* h. not in STANDBY (HDG LOCK at the dock): READY, nothing */
        {
            static sbsim_t y;
            gen_setup(&y, 1603u, NULL, 11.5f);
            (void)sb_generator_set(&y.sb, true);
            press(&y, HAL_BTN_HDG_LOCK);
            gensup_fixed(&g, 11.5f);
            sbsim_run(&y, y.t + 20000u, gen_hook, &g);
            bool hdg = sb_mode(&y.sb) == MODE_HDG_LOCK && y.sb.gen.state == GEN_READY && y.sb.gen.demand_pct == 0.0f;
            press(&y, HAL_BTN_STANDBY);                     /* the STANDBY button: auto OFF */
            sbsim_run(&y, y.t + 20000u, gen_hook, &g);
            GN1_CASE("HDG LOCK: nothing; STANDBY button -> auto off", hdg && x.sb.gen.state != GEN_OFF &&
                     y.sb.gen.state == GEN_OFF && !y.sb.gen.enabled && y.sb.thr.out_pct == 0.0f);
        }
        /* i. FIRST TEST: refused, never */
        {
            static sbsim_t y;
            sb_config_t c;
            sb_default_config(&c);
            c.first_test = true;
            gen_setup(&y, 1604u, &c, 11.5f);
            bool refused = sb_generator_set(&y.sb, true) == SB_ERR_WRONG_MODE;
            y.sb.gen.enabled = true;                        /* even forced on: the conditions never hold */
            gensup_fixed(&g, 11.5f);
            sbsim_run(&y, y.t + 20000u, gen_hook, &g);
            GN1_CASE("FIRST TEST refused", refused && y.sb.gen.state == GEN_READY && y.sb.thr.out_pct == 0.0f &&
                     sim_hal_throttle() <= 0.0f);
        }
        /* j. no voltage data: never */
        {
            static sbsim_t y;
            gen_setup(&y, 1605u, NULL, 12.6f);
            (void)sb_generator_set(&y.sb, true);
            memset(&g, 0, sizeof g);
            g.none = true;
            sbsim_run(&y, y.t + 20000u, gen_hook, &g);
            GN1_CASE("no voltage data", y.sb.batt.no_data && y.sb.gen.state == GEN_READY && y.sb.thr.out_pct == 0.0f);
        }
#undef GN1_CASE
        test_report("GN1 start conditions one by one (auto off, bank fine, no switch, switch open, alarm, e-stop, "
                    "disabled, not STANDBY, FIRST TEST, no voltage data); all OK -> beep, 10 s, fast idle in N",
                    n_ok == 10, d);
        sim_hal_set_estop(false, false);
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN2 steps, hold, down, the cap: +2 % every 30 s while the fast voltage still falls, hold when it stops
     * (= the learned value), -2 % above start + 0.4 V, never above min(gen_max_pct, 30, power limit); a lower
     * cap is followed down at once, never up; parameter limits */
    {
        static pcsim_t p;
        static gensup_t g;
        pcsim_init(&p, 1606u);
        p.x.w.neutral_switch = true;
        sim_hal_set_supply(true, 12.0f);                   /* the slow filter starts here */
        pc_run(&p, 2000u);
        uint8_t id_max = 0xFFu, id_sv = 0xFFu, id_idle = 0xFFu;
        for (uint8_t i = 0; i < link_param_count(); i++) {
            if (strcmp(link_param_info(i)->name, "gen_max_pct") == 0) id_max = i;
            if (strcmp(link_param_info(i)->name, "gen_start_v") == 0) id_sv = i;
            if (strcmp(link_param_info(i)->name, "gen_idle_pct") == 0) id_idle = i;
        }
        bool params = id_max != 0xFFu && id_sv != 0xFFu && id_idle != 0xFFu &&
                      link_param_info(id_max)->min == 0.0f && link_param_info(id_max)->max == 30.0f &&
                      link_param_info(id_sv)->min == 11.5f && link_param_info(id_sv)->max == 13.0f &&
                      link_param_info(id_idle)->min == 3.0f && link_param_info(id_idle)->max == 15.0f &&
                      p.x.sb.gen.cfg.max_pct == 25.0f && p.x.sb.gen.cfg.start_v == 12.1f && p.x.sb.gen.cfg.idle_pct == 8.0f;
        uint8_t ps[5]; proto_wr_t w;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id_max); proto_wr_f32(&w, 40.0f);
        bool over_refused = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_BAD_VALUE && p.x.sb.gen.cfg.max_pct == 25.0f;
        /* the bank falls 0.002 V/s whatever the engine does (alternator too weak): every 30 s one step up */
        memset(&g, 0, sizeof g);
        g.base_v = 12.0f; g.drain_v_s = 0.002f; g.alt_v_pct = 0.0f; g.t0 = p.x.t;
        uint8_t one = 1u;
        bool cmd_on = pc_cmd(&p, PROTO_T_GENERATOR, &one, 1u) == PROTO_R_OK && p.x.sb.gen.enabled;
        sbsim_run(&p.x, p.x.t + 12000u, gen_hook, &g);
        bool run8 = gen_running_at(&p.x, 8.0f);
        sbsim_run(&p.x, p.x.t + 31000u, gen_hook, &g);
        bool step10 = gen_running_at(&p.x, 10.0f) && p.x.sb.gen.steps_up == 1u;
        sbsim_run(&p.x, p.x.t + 30000u, gen_hook, &g);
        bool step12 = gen_running_at(&p.x, 12.0f);
        sbsim_run(&p.x, p.x.t + 7u * 30000u, gen_hook, &g);     /* 7 more steps: 26 -> capped at 25 */
        bool cap25 = gen_running_at(&p.x, 25.0f) && g.max_demand <= 25.0f && g.max_out <= 25.6f && !p.x.mon.tripped;
        sbsim_run(&p.x, p.x.t + 60000u, gen_hook, &g);
        bool hold_cap = gen_running_at(&p.x, 25.0f) && g.max_demand <= 25.0f && g_mon.gen_cap_pct == 25.0f;
        /* the bank recovers to 12.3 V (between start_v and start_v + 0.4): stopped falling -> hold, learned 25 */
        gensup_fixed(&g, 12.3f);
        sbsim_run(&p.x, p.x.t + 65000u, gen_hook, &g);
        bool learned = gen_running_at(&p.x, 25.0f) && p.x.sb.gen.learned_pct == 25.0f && p.x.sb.gen.stopped_falling;
        /* above 12.5 V: -2 % every 30 s, never below the fast idle 8 % */
        gensup_fixed(&g, 12.8f);
        sbsim_run(&p.x, p.x.t + 35000u, gen_hook, &g);
        bool down23 = p.x.sb.gen.state == GEN_RUNNING && p.x.sb.gen.steps_down >= 1u && p.x.sb.gen.demand_pct <= 23.0f &&
                      p.x.sb.gen.demand_pct >= 21.0f;
        sbsim_run(&p.x, p.x.t + 9u * 30000u, gen_hook, &g);
        bool floor8 = gen_running_at(&p.x, 8.0f);
        /* the power limit raised to 50 % and gen_max_pct set to 30: cap = min(30, 30 ceiling, 50) = 30
         * (LOW BATTERY came and went meanwhile - an advisory: the generator kept running - but a raise
         * needs the alarms acknowledged first) */
        bool low_seen = (p.x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY) && p.x.sb.gen.state == GEN_RUNNING;
        sb_alarm_ack(&p.x.sb);
        bool raised = low_seen && power_step(&p.x, 0u) && p.x.sb.plim.fwd_pct == 50.0f && p.x.sb.gen.state == GEN_RUNNING;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id_max); proto_wr_f32(&w, 30.0f);
        bool set30 = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_OK && p.x.sb.gen.cfg.max_pct == 30.0f;
        memset(&g, 0, sizeof g);
        g.base_v = 11.9f; g.drain_v_s = 0.002f; g.t0 = p.x.t;
        sbsim_run(&p.x, p.x.t + 12u * 30000u, gen_hook, &g);    /* 8 -> 30 in 11 steps */
        bool cap30 = gen_running_at(&p.x, 30.0f) && g.max_demand <= 30.0f && g.max_out <= 30.6f && !p.x.mon.tripped &&
                     g_mon.gen_cap_pct == 30.0f;
        /* the cap lowered to 20 while running at 30: followed down at once (normal closing rate), never up */
        float out_before = p.x.sb.thr.out_pct;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id_max); proto_wr_f32(&w, 20.0f);
        bool set20 = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_OK && p.x.sb.gen.demand_pct == 20.0f;
        pc_run(&p, 1000u);
        bool at20 = gen_running_at(&p.x, 20.0f) && out_before > 29.0f && !p.x.mon.tripped;
        sbsim_run(&p.x, p.x.t + 2u * 30000u, gen_hook, &g);
        bool stays20 = gen_running_at(&p.x, 20.0f) && g.max_demand <= 30.0f;
        snprintf(d, sizeof d, "parameters gen_max_pct 0..30 / gen_start_v 11.5..13 / gen_idle_pct 3..15 with defaults "
                 "25 / 12.1 / 8 %s; 40 refused %s; GENERATOR on %s; RUNNING 8 %% %s; +2 %% at 30 s %s, 12 %% at 60 s "
                 "%s; capped at 25 %% %s (max demand %.0f, max output %.1f); holds %s; bank 12.3 V: hold, learned 25 "
                 "%s; 12.8 V: 23 %% %s ... floor 8 %% %s; power 50 %% %s, gen_max 30 %s -> 30 %% %s; gen_max 20 while "
                 "at 30: set %s, demand 20 at once %s, stays %s",
                 params ? "ok" : "NO", over_refused ? "yes" : "NO", cmd_on ? "yes" : "NO", run8 ? "yes" : "NO",
                 step10 ? "yes" : "NO", step12 ? "yes" : "NO", cap25 ? "yes" : "NO", g.max_demand, g.max_out,
                 hold_cap ? "yes" : "NO", learned ? "yes" : "NO", down23 ? "yes" : "NO", floor8 ? "yes" : "NO",
                 raised ? "yes" : "NO", set30 ? "yes" : "NO", cap30 ? "yes" : "NO", set20 ? "yes" : "NO",
                 at20 ? "yes" : "NO", stays20 ? "yes" : "NO");
        test_report("GN2 steps +2 % / 30 s while the voltage falls, hold = learned value, -2 % when high, floor at the "
                    "fast idle, cap = min(gen_max_pct, 30, power limit), a lowered cap followed down at once",
                    params && over_refused && cmd_on && run8 && step10 && step12 && cap25 && hold_cap && learned &&
                    down23 && floor8 && raised && set30 && cap30 && set20 && at20 && stays20, d);
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN3 the cycle: run_ms, then rest_ms at idle, the rest-voltage check: still low -> again (beep); recovered
     * -> DONE (notice) and waiting; low again later -> again */
    {
        static sbsim_t x;
        static gensup_t g;
        sb_config_t c;
        sb_default_config(&c);
        c.gen.run_ms = 90000u;                             /* short cycle for the test (defaults checked in GN1) */
        c.gen.rest_ms = 20000u;
        gen_setup(&x, 1607u, &c, 11.8f);
        (void)sb_generator_set(&x.sb, true);
        gensup_fixed(&g, 11.8f);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        bool run1 = gen_running_at(&x, 8.0f);
        sbsim_run(&x, x.t + 90000u, gen_hook, &g);         /* run over -> STOPPING -> REST */
        bool rest = x.sb.gen.state == GEN_REST && x.sb.thr.out_pct == 0.0f && gear_is(&x.sb.gear, GEAR_N) &&
                    x.sb.gen.stop_why == GEN_STOP_REST && !x.sb.gen.active;
        sbsim_run(&x, x.t + 21000u, gen_hook, &g);         /* still low: again */
        bool again = (x.sb.gen.state == GEN_STARTING || x.sb.gen.state == GEN_RUNNING) && x.sb.gen.cycles == 1u;
        sbsim_run(&x, x.t + 12000u, gen_hook, &g);
        bool run2 = gen_running_at(&x, 8.0f) && x.sb.gen.starts == 2u;
        gensup_fixed(&g, 12.6f);                           /* the bank recovers during the run */
        sbsim_run(&x, x.t + 90000u + 25000u, gen_hook, &g);
        bool done = x.sb.gen.state == GEN_DONE && x.sb.gen.cycles == 2u && x.sb.thr.out_pct == 0.0f && x.sb.gen.enabled;
        sbsim_run(&x, x.t + 60000u, gen_hook, &g);
        bool waits = x.sb.gen.state == GEN_DONE && x.sb.gen.starts == 2u;
        gensup_fixed(&g, 11.7f);                           /* low again later: again */
        sbsim_run(&x, x.t + 60000u, gen_hook, &g);
        bool run3 = gen_running_at(&x, 8.0f) && x.sb.gen.starts == 3u && g.rises_in_stop == 0 && g.stray == 0;
        snprintf(d, sizeof d, "run 1 %s; REST at idle + N after 90 s %s; still low: again %s, run 2 %s; bank 12.6 V "
                 "-> DONE after the rest %s; waits %s; 11.7 V later: run 3 %s (no rise in a stop, no stray throttle)",
                 run1 ? "yes" : "NO", rest ? "yes" : "NO", again ? "yes" : "NO", run2 ? "yes" : "NO",
                 done ? "yes" : "NO", waits ? "yes" : "NO", run3 ? "yes" : "NO");
        test_report("GN3 cycle: run, rest at idle, rest-voltage check (still low: again; recovered: DONE), again later",
                    run1 && rest && again && run2 && done && waits && run3, d);
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN4 the neutral checks while running: the switch opens -> idle at once + GEAR FAULT, auto off until
     * re-enabled; a SOG rise within 20 s of a throttle rise (the boat is moving: the gearbox is not in N
     * whatever the switch says) -> idle + GEAR FAULT */
    {
        static sbsim_t x;
        static gensup_t g;
        gen_setup(&x, 1608u, NULL, 11.8f);
        (void)sb_generator_set(&x.sb, true);
        gensup_fixed(&g, 11.8f);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        bool run1 = gen_running_at(&x, 8.0f);
        x.w.neutral_sw_force = 1;                          /* the switch opens */
        sbsim_run(&x, x.t + 100u, gen_hook, &g);
        bool at_once = x.sb.gen.demand_pct == 0.0f && x.sb.gen.state == GEN_STOPPING && x.sb.gen.fast_stop &&
                       (x.sb.alarm_bits & SB_ALM_GEAR) && !x.sb.gen.enabled && x.sb.throttle_mode == 0.0f;
        sbsim_run(&x, x.t + 400u, gen_hook, &g);
        bool idle_fast = x.sb.thr.out_pct == 0.0f && x.sb.gen.state == GEN_FAULT && sb_mode(&x.sb) == MODE_STANDBY;
        x.w.neutral_sw_force = 0;                          /* closed again: still off */
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 30000u, gen_hook, &g);
        bool stays_off = x.sb.gen.state == GEN_FAULT && x.sb.thr.out_pct == 0.0f && !x.sb.gen.enabled &&
                         !(x.sb.alarm_bits & SB_ALM_GEAR);
        (void)sb_generator_set(&x.sb, true);               /* re-enabled by the captain: runs again */
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        bool again = gen_running_at(&x, 8.0f) && x.sb.gen.faults == 1u;
        /* the boat starts moving (a current carries it: SOG rises 0.6 kn) 5 s after the next rise */
        x.w.neutral_sw_force = 2;                          /* the switch stays "closed" whatever happens */
        sbsim_run(&x, x.t + 26000u, gen_hook, &g);         /* the first judgement (30 s): hold (no rise yet) */
        memset(&g, 0, sizeof g);
        g.base_v = 11.8f; g.drain_v_s = 0.004f; g.t0 = x.t;  /* falling: the judgement at 60 s steps up */
        sbsim_run(&x, x.t + 40000u, gen_hook, &g);
        bool rose = x.sb.gen.state == GEN_RUNNING && x.sb.gen.demand_pct >= 10.0f && x.sb.gen.move_ms > 0u;
        bool gps_ok = sb_gps_ok(&x.sb);
        x.w.current_n = 0.31f;                             /* 0.6 kn northwards */
        sbsim_run(&x, x.t + 12000u, gen_hook, &g);
        bool moved = x.sb.gen.state != GEN_RUNNING && x.sb.gen.stop_why == GEN_STOP_MOVED && x.sb.gen.faults == 2u &&
                     (x.sb.alarm_bits & SB_ALM_GEAR) && x.sb.thr.out_pct == 0.0f && !x.sb.gen.enabled &&
                     sb_mode(&x.sb) == MODE_STANDBY;
        snprintf(d, sizeof d, "running %s; switch opens: demand 0 + STOPPING (fast) + GEAR FAULT + auto off at once %s; "
                 "idle within 0.5 s, FAULT %s; closed again + ACK: stays off %s; re-enabled: runs %s; a throttle rise "
                 "%s (GPS trusted %s); 0.6 kn SOG rise within 20 s: GEAR FAULT, idle, auto off %s (why %u)",
                 run1 ? "yes" : "NO", at_once ? "yes" : "NO", idle_fast ? "yes" : "NO", stays_off ? "yes" : "NO",
                 again ? "yes" : "NO", rose ? "yes" : "NO", gps_ok ? "yes" : "no", moved ? "yes" : "NO",
                 x.sb.gen.stop_why);
        test_report("GN4 neutral checks: switch opens -> idle at once + GEAR FAULT + auto off until re-enabled; "
                    "SOG rise after a throttle rise -> idle + GEAR FAULT",
                    run1 && at_once && idle_fast && stays_off && again && rose && gps_ok && moved, d);
        x.w.current_n = 0.0f;
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN5 every stop path is idle FIRST and never raises during the stop: STANDBY button (auto off),
     * joystick takeover (MANUAL: the gear only engages at idle), captain off, HDG LOCK button, a pending
     * AUTO start (CONFIRM arrives at idle), no voltage data, a new alarm */
    {
        static sbsim_t x;
        static gensup_t g;
        int n_ok = 0, len = 0;
        d[0] = 0;
#define GN5_START(seed) do { gen_setup(&x, (seed), NULL, 11.8f); (void)sb_generator_set(&x.sb, true); gensup_fixed(&g, 11.8f); \
        sbsim_run(&x, x.t + 15000u, gen_hook, &g); g.rises_in_stop = 0; g.stray = 0; g.gear_not_n = 0; } while (0)
#define GN5_CASE(name, cond) do { bool c_ = (cond) && g.rises_in_stop == 0 && g.stray == 0 && g.gear_not_n == 0; \
        if (c_) { n_ok++; } len += snprintf(d + len, sizeof d - (size_t)len, "%s %s%s; ", name, c_ ? "ok" : "FAILED", \
        g.rises_in_stop ? " (rose in the stop)" : ""); } while (0)
        /* 1. STANDBY button */
        GN5_START(1609u);
        bool r1 = gen_running_at(&x, 8.0f);
        press(&x, HAL_BTN_STANDBY);
        bool d0 = x.sb.gen.demand_pct == 0.0f && !x.sb.gen.enabled;
        sbsim_run(&x, x.t + 1000u, gen_hook, &g);
        GN5_CASE("STANDBY button -> auto off, idle", r1 && d0 && x.sb.gen.state == GEN_OFF && x.sb.thr.out_pct == 0.0f);
        /* 2. joystick X deflection: MANUAL - the gear waits for idle; back to STANDBY -> runs again */
        GN5_START(1610u);
        r1 = gen_running_at(&x, 8.0f);
        joy_set(0.6f, 0.0f, false, false);
        sbsim_run(&x, x.t + 300u, gen_hook, &g);
        bool manual = sb_mode(&x.sb) == MODE_MANUAL && x.sb.gen.demand_pct == 0.0f && x.sb.gen.state != GEN_RUNNING;
        if (s_usb.on) usb_lever(&x, 0.75f, 2.0f, gen_hook, &g);   /* and ahead (USB: the lever) */
        else joy_set(0.6f, 0.5f, false, false);
        sbsim_run(&x, x.t + 7000u, gen_hook, &g);
        bool drives = sb_mode(&x.sb) == MODE_MANUAL && gear_is(&x.sb.gear, GEAR_F) && x.sb.thr.out_pct > 5.0f &&
                      x.w.shift_under_load == 0 && !x.mon.tripped && !x.sb.gen.active;
        if (s_usb.on) usb_lever(&x, 0.5f, 2.0f, gen_hook, &g);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 3000u, gen_hook, &g);
        joy_set(0.0f, 0.0f, false, true);                  /* AUTO button with the stick centred: hand back */
        sbsim_run(&x, x.t + 200u, gen_hook, &g);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 20000u, gen_hook, &g);
        GN5_CASE("joystick -> MANUAL (gear at idle), back in STANDBY -> runs again",
                 r1 && manual && drives && sb_mode(&x.sb) == MODE_STANDBY && gen_running_at(&x, 8.0f));
        /* 3. the captain switches it off */
        GN5_START(1611u);
        r1 = gen_running_at(&x, 8.0f);
        (void)sb_generator_set(&x.sb, false);
        bool s3 = x.sb.gen.state == GEN_STOPPING && x.sb.gen.demand_pct == 0.0f;
        sbsim_run(&x, x.t + 1000u, gen_hook, &g);
        GN5_CASE("captain off", r1 && s3 && x.sb.gen.state == GEN_OFF && x.sb.thr.out_pct == 0.0f);
        /* 4. HDG LOCK button (a mode request) */
        GN5_START(1612u);
        r1 = gen_running_at(&x, 8.0f);
        press(&x, HAL_BTN_HDG_LOCK);
        sbsim_run(&x, x.t + 1000u, gen_hook, &g);
        GN5_CASE("HDG LOCK button", r1 && sb_mode(&x.sb) == MODE_HDG_LOCK && x.sb.thr.out_pct == 0.0f &&
                 x.sb.gen.state == GEN_READY && x.sb.gen.demand_pct == 0.0f);
        /* 5. a pending AUTO start: the stop begins with the request, the CONFIRM finds the engine idle */
        GN5_START(1613u);
        r1 = gen_running_at(&x, 8.0f);
        {
            prog_wp_t w5[5];
            program_t pr;
            (void)prog5(&x, w5);
            prog_make(&pr, w5, 5, PROG_END_STOP);
            prog_error_t err; uint8_t bad;
            bool stored = sb_program_set(&x.sb, &pr, &err, &bad) == SB_OK;
            bool asked = sb_auto_start(&x.sb) == SB_OK;
            sbsim_run(&x, x.t + 1000u, gen_hook, &g);
            bool stopped = x.sb.gen.state == GEN_READY && x.sb.thr.out_pct == 0.0f && sb_mode(&x.sb) == MODE_STANDBY;
            bool conf = confirm_press(&x) && sb_mode(&x.sb) == MODE_AUTO;
            sbsim_run(&x, x.t + 3000u, gen_hook, &g);
            GN5_CASE("pending AUTO start stops it; CONFIRM at idle -> AUTO", r1 && stored && asked && stopped && conf &&
                     sb_mode(&x.sb) == MODE_AUTO && !x.sb.gen.active && x.w.shift_under_load == 0);
        }
        /* 6. the voltage data goes away */
        GN5_START(1614u);
        r1 = gen_running_at(&x, 8.0f);
        g.none = true;
        sbsim_run(&x, x.t + 7000u, gen_hook, &g);
        bool s6 = x.sb.batt.no_data && x.sb.gen.state == GEN_READY && x.sb.thr.out_pct == 0.0f &&
                  x.sb.gen.stop_why == GEN_STOP_NO_VOLTS;
        g.none = false;
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        GN5_CASE("no voltage data -> idle, data back -> runs", r1 && s6 && gen_running_at(&x, 8.0f));
        /* 7. a new alarm (GPS lost in STANDBY is no alarm; the compass is): idle; ACK -> again */
        GN5_START(1615u);
        r1 = gen_running_at(&x, 8.0f);
        sim_hal_fail_heading(true);
        sbsim_run(&x, x.t + 5000u, gen_hook, &g);
        bool s7 = (x.sb.alarm_bits & SB_ALM_COMPASS) && x.sb.gen.state == GEN_READY && x.sb.thr.out_pct == 0.0f &&
                  sb_mode(&x.sb) == MODE_STANDBY;
        sim_hal_fail_heading(false);
        sbsim_run(&x, x.t + 3000u, gen_hook, &g);
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        GN5_CASE("alarm -> idle, ACK -> runs", r1 && s7 && gen_running_at(&x, 8.0f));
#undef GN5_START
#undef GN5_CASE
        test_report("GN5 every stop idle FIRST, nothing rises during a stop: STANDBY button, joystick, captain off, "
                    "HDG LOCK, pending AUTO start, no voltage data, new alarm", n_ok == 7, d);
        joy_set(0.0f, 0.0f, false, false);
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN6 CPU1: with gen_active the gear must stay at neutral and the throttle under min(cap, power limit);
     * without gen_active the old rule (idle in neutral: the reverse limit) */
    {
        static sbsim_t x;
        static gensup_t g;
        static inject_t j;
        int n_ok = 0, len = 0;
        d[0] = 0;
        /* a. gear pulse leaves neutral while the generator runs */
        gen_setup(&x, 1616u, NULL, 11.8f);
        (void)sb_generator_set(&x.sb, true);
        gensup_fixed(&g, 11.8f);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        bool r1 = gen_running_at(&x, 8.0f);
        memset(&j, 0, sizeof j);
        j.kind = 2; j.at_ms = x.t;
        react_t r = watch_reaction(&x, x.t, inject_hook, &j, 1000u);
        bool a = r1 && x.mon.tripped && x.mon.reason == MON_TRIP_GENERATOR && r.t_react >= 0.0f && r.t_react <= 150.0f &&
                 r.relay_open && x.sb.mon_tripped && sb_mode(&x.sb) == MODE_STANDBY && !x.sb.gen.active;
        len += snprintf(d + len, sizeof d - (size_t)len, "gear leaves N: trip %s (%s, %.0f ms); ", a ? "ok" : "FAILED",
                        monitor_reason_name(x.mon.reason), r.t_react);
        if (a) n_ok++;
        /* b. throttle above the generator cap (25 %) in neutral */
        gen_setup(&x, 1617u, NULL, 11.8f);
        (void)sb_generator_set(&x.sb, true);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        r1 = gen_running_at(&x, 8.0f);
        memset(&j, 0, sizeof j);
        j.kind = 1; j.at_ms = x.t;                         /* 90 %: past every limit */
        r = watch_reaction(&x, x.t, inject_hook, &j, 1000u);
        bool b = r1 && x.mon.tripped && x.mon.reason == MON_TRIP_THROTTLE && r.t_react >= 0.0f && r.t_react <= 150.0f &&
                 r.relay_open;
        len += snprintf(d + len, sizeof d - (size_t)len, "throttle 90 %% with gen_active: trip %s (%.0f ms); ",
                        b ? "ok" : "FAILED", r.t_react);
        if (b) n_ok++;
        /* c. throttle 10 % in neutral WITHOUT gen_active (generator off): the old rule trips (reverse limit) */
        gen_setup(&x, 1618u, NULL, 12.6f);
        {
            uint32_t t0 = x.t;
            bool tripped = false; float t_react = 0.0f;
            for (int i = 0; i < 60 && !tripped; i++) {
                sbsim_run(&x, x.t + 50u, NULL, NULL);
                hal_set_throttle_pwm(1100u);               /* 10 % in neutral, gen_active 0 */
                g_mon.throttle_pct = 10.0f;
                tripped = x.mon.tripped;
                t_react = (x.t - t0) * 0.001f;
            }
            bool c = tripped && x.mon.reason == MON_TRIP_THROTTLE && t_react <= 0.2f;
            len += snprintf(d + len, sizeof d - (size_t)len, "10 %% in N without gen_active: trip %s (%.2f s); ",
                            c ? "ok" : "FAILED", t_react);
            if (c) n_ok++;
        }
        /* d. the generator runs at 8 % but core 0 "forgets" gen_active: the throttle is above the reverse limit
         *    and not coming down -> trip after MON_OVER_MS */
        gen_setup(&x, 1619u, NULL, 11.8f);
        (void)sb_generator_set(&x.sb, true);
        sbsim_run(&x, x.t + 15000u, gen_hook, &g);
        r1 = gen_running_at(&x, 8.0f);
        {
            uint32_t t0 = x.t;
            bool tripped = false; float t_react = 0.0f;
            for (int i = 0; i < 80 && !tripped; i++) {
                sbsim_run(&x, x.t + 50u, gen_clear_hook, &g);  /* gen_active cleared right after every step */
                tripped = x.mon.tripped;
                t_react = (x.t - t0) * 0.001f;
            }
            bool dd = r1 && tripped && x.mon.reason == MON_TRIP_THROTTLE && t_react > 2.0f && t_react <= 3.0f;
            len += snprintf(d + len, sizeof d - (size_t)len, "8 %% in N, gen_active cleared: trip after %.2f s %s",
                            t_react, dd ? "ok" : "FAILED");
            if (dd) n_ok++;
        }
        test_report("GN6 CPU1 monitor: gear not N with gen_active -> trip; throttle above the cap -> trip; throttle "
                    "in neutral without gen_active -> trip (old rule)", n_ok == 4, d);
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN7 the learned value: saved in NV slot 6 (STANDBY), the first step of the next run after a reboot;
     * a bad record = the fast idle + NV DATA RESET */
    {
        static sbsim_t x;
        static gensup_t g;
        remove("nv6.bin");
        gen_setup(&x, 1620u, NULL, 12.0f);
        (void)sb_generator_set(&x.sb, true);
        memset(&g, 0, sizeof g);
        g.base_v = 12.0f; g.drain_v_s = 0.002f; g.t0 = x.t;
        sbsim_run(&x, x.t + 12000u + 2u * 30000u + 2000u, gen_hook, &g);   /* 8 -> 10 -> 12 */
        bool at12 = gen_running_at(&x, 12.0f);
        gensup_fixed(&g, 12.3f);                           /* stops falling at 12 %: learned */
        sbsim_run(&x, x.t + 35000u, gen_hook, &g);
        bool learned = x.sb.gen.learned_pct == 12.0f && x.sb.gen.learned_new;
        uint32_t writes0 = sim_hal_nv_writes();
        (void)sb_generator_set(&x.sb, false);              /* stop: saved in STANDBY once idle */
        sbsim_run(&x, x.t + 3000u, gen_hook, &g);
        bool saved = sim_hal_nv_writes() == writes0 + 1u && !x.sb.gen.learned_new && x.sb.gen.state == GEN_OFF;
        /* reboot: the record is read back; the first step is 12 % */
        gen_setup(&x, 1621u, NULL, 11.8f);
        bool loaded = x.sb.gen.learned_pct == 12.0f && !(nv_status()->reset_mask & (1u << HAL_NV_SLOT_GENERATOR));
        (void)sb_generator_set(&x.sb, true);
        gensup_fixed(&g, 11.8f);
        sbsim_run(&x, x.t + 12000u, gen_hook, &g);
        bool first12 = gen_running_at(&x, 12.0f) && x.sb.gen.steps_up == 0u;
        /* a record of another version: refused -> the fast idle, NV DATA RESET */
        {
            struct { uint8_t version; uint8_t pct; uint8_t pad[2]; uint32_t crc; } bad;   /* = smallboat.c gen_nv_t */
            memset(&bad, 0, sizeof bad);
            bad.version = 9u; bad.pct = 20u; bad.crc = 0x12345678u;
            (void)hal_nv_save(HAL_NV_SLOT_GENERATOR, &bad, (uint32_t)sizeof bad);
            for (int i = 0; i < 50 && hal_nv_pending(); i++) hal_nv_service(true);
        }
        gen_setup(&x, 1622u, NULL, 11.8f);
        bool reset = x.sb.gen.learned_pct == 0.0f && (nv_status()->reset_mask & (1u << HAL_NV_SLOT_GENERATOR));
        (void)sb_generator_set(&x.sb, true);
        sbsim_run(&x, x.t + 12000u, gen_hook, &g);
        bool first8 = gen_running_at(&x, 8.0f);
        snprintf(d, sizeof d, "12 %% reached %s; learned 12 %s; saved once idle in STANDBY %s; reboot: loaded %s, first "
                 "step 12 %% %s; a bad record: defaults + NV DATA RESET %s, first step 8 %% %s",
                 at12 ? "yes" : "NO", learned ? "yes" : "NO", saved ? "yes" : "NO", loaded ? "yes" : "NO",
                 first12 ? "yes" : "NO", reset ? "yes" : "NO", first8 ? "yes" : "NO");
        test_report("GN7 learned value in NV slot 6: saved in STANDBY, the first step after a reboot; a bad record = "
                    "defaults + NV DATA RESET", at12 && learned && saved && loaded && first12 && reset && first8, d);
        remove("nv6.bin");
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN8 the link: PROTO_T_GENERATOR on / off / bad value / bad length, TELEM2 group L carries state + %, the
     * stop_why / events, FIRST TEST refused over the link */
    {
        static pcsim_t p;
        static gensup_t g;
        pcsim_init(&p, 1623u);
        p.x.w.neutral_switch = true;
        sim_hal_set_supply(true, 11.8f);
        (void)g;
        pc_run(&p, 2000u);
        uint8_t v = 1u;
        bool on = pc_cmd(&p, PROTO_T_GENERATOR, &v, 1u) == PROTO_R_OK && p.x.sb.gen.enabled &&
                  (p.x.sb.gen.state == GEN_READY || p.x.sb.gen.state == GEN_STARTING);
        v = 2u;
        bool bad = pc_cmd(&p, PROTO_T_GENERATOR, &v, 1u) == PROTO_R_BAD_VALUE && p.x.sb.gen.enabled;
        bool badlen = pc_cmd(&p, PROTO_T_GENERATOR, NULL, 0u) == PROTO_R_BAD_LENGTH;
        /* the PC keeps its heartbeat (a silent PC = PC LINK LOST = an alarm that stops the generator) */
        p.heartbeat = true;
        p.next_hb = p.x.t;
        pc_run(&p, 15000u);
        /* group L ends 16 bytes (group M, the relay board) + group O before the end of the frame */
        uint8_t st1 = p.t2_len >= 18u ? p.t2[p.t2_len - T2_O - 18u] : 99u, pc1 = p.t2_len >= 17u ? p.t2[p.t2_len - T2_O - 17u] : 99u;
        const char *sbst1 = gen_state_name(p.x.sb.gen.state);
        bool t2 = p.t2_len >= 11u && st1 == GEN_RUNNING && pc1 == 8u;
        v = 0u;
        bool off = pc_cmd(&p, PROTO_T_GENERATOR, &v, 1u) == PROTO_R_OK && !p.x.sb.gen.enabled;
        pc_run(&p, 2000u);
        bool t2off = p.t2_len >= 27u && p.t2[p.t2_len - T2_O - 18u] == GEN_OFF && p.t2[p.t2_len - T2_O - 17u] == 0u &&
                     p.x.sb.thr.out_pct == 0.0f;
        /* FIRST TEST over the link: refused */
        {
            static pcsim_t q;
            sb_config_t c;
            sb_default_config(&c);
            c.first_test = true;
            memset(&q, 0, sizeof q);
            sbsim_init(&q.x, 1624u, 90.0f, 0.0f, &c);
            sim_hal_link_reset();
            link_init(&q.link, &q.x.sb);
            proto_rx_init(&q.pc_rx);
            pc_run(&q, 2000u);
            v = 1u;
            bool ft = pc_cmd(&q, PROTO_T_GENERATOR, &v, 1u) == PROTO_R_WRONG_MODE && !q.x.sb.gen.enabled;
            snprintf(d, sizeof d, "on %s; 2 = bad value %s; no payload = bad length %s; TELEM2 L: state RUNNING + 8 %% "
                     "%s (len %u, state %u pct %u, sb state %s); off %s -> TELEM2 OFF + 0 %% %s; FIRST TEST: refused %s",
                     on ? "ok" : "NO", bad ? "ok" : "NO", badlen ? "ok" : "NO", t2 ? "ok" : "NO", p.t2_len, st1, pc1,
                     sbst1,
                     off ? "ok" : "NO", t2off ? "ok" : "NO", ft ? "ok" : "NO");
            test_report("GN8 link: GENERATOR command on / off / bad value / bad length, TELEM2 group L state + %, "
                        "FIRST TEST refused", on && bad && badlen && t2 && off && t2off && ft, d);
        }
        sim_hal_set_supply(true, 12.6f);
    }

    /* GN10 (step 11g, owner review finding 1) a normal generator stop comes down at rate_down - an emergency DURING
     * that stop (the STANDBY button, the e-stop) = idle AT ONCE in that same step */
    {
        static sbsim_t x;
        static gensup_t g;
        int n_ok = 0, len = 0;
        d[0] = 0;
        for (int k = 0; k < 2; k++) {
            gen_setup(&x, 1620u + (uint32_t)k, NULL, 11.8f);
            (void)sb_generator_set(&x.sb, true);
            gensup_fixed(&g, 11.8f);
            sbsim_run(&x, x.t + 15000u, gen_hook, &g);
            bool r1 = gen_running_at(&x, 8.0f);
            (void)sb_generator_set(&x.sb, false);           /* the captain: a normal stop (rate_down) */
            sbsim_run(&x, x.t + AP_PERIOD_MS - 1u, gen_hook, &g);
            float o1 = x.sb.thr.out_pct;
            bool normal = x.sb.gen.state == GEN_STOPPING && !x.sb.gen.fast_stop && o1 > 1.0f && o1 < 8.0f;
            if (k == 0) sim_hal_press(HAL_BTN_STANDBY); else sim_hal_set_estop(true, true);
            sbsim_run(&x, x.t + AP_PERIOD_MS - 1u, gen_hook, &g);
            float o2 = x.sb.thr.out_pct;
            bool at_once = o2 == 0.0f && x.sb.thr.stop;
            sim_hal_set_estop(false, false);
            if (r1 && normal && at_once) n_ok++;
            len += snprintf(d + len, sizeof d - (size_t)len, "%s: running %s, normal stop %.1f %% %s, emergency -> %.1f %% %s; ",
                            k ? "e-stop" : "STANDBY button", r1 ? "yes" : "NO", (double)o1, normal ? "ok" : "NO", (double)o2,
                            at_once ? "ok" : "FAILED");
        }
        test_report("GN10 an emergency during a normal generator stop: idle at once (STANDBY button, e-stop)", n_ok == 2, d);
        sim_hal_set_supply(true, 12.6f);
    }
}

static void battery_tests(void)
{
    char d[700];
    printf("\n=== BV supply voltage of the 12 V bank (battery.c) ===\n");

    /* BV1 the filters: first sample sets both (no ramp from 0 V), fast ~1 s, slow ~30 s; NaN = no sample */
    {
        static sbsim_t x;
        sbsim_init(&x, 1501u, 90.0f, 0.0f, NULL);
        sim_hal_set_supply(true, 12.6f);
        sbsim_run(&x, 2000u, NULL, NULL);
        const batt_t *b = &x.sb.batt;
        bool first = b->valid && fabsf(b->bank_v - 12.6f) < 0.01f && fabsf(b->bank_v_fast - 12.6f) < 0.01f &&
                     !b->no_data && x.sb.alarm_bits2 == 0u;
        sim_hal_set_supply(true, 11.0f);                   /* a 1.6 V step down */
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        float fast_1s = b->bank_v_fast, slow_1s = b->bank_v;
        /* 1 s = one fast time constant: 63 % of the step; the slow filter has barely moved */
        bool fast_ok = fabsf(fast_1s - (12.6f - 1.6f * 0.632f)) < 0.08f && slow_1s > 12.5f;
        sbsim_run(&x, x.t + 29000u, NULL, NULL);          /* 30 s after the step */
        float slow_30s = b->bank_v;
        bool slow_ok = fabsf(slow_30s - (12.6f - 1.6f * 0.632f)) < 0.08f && fabsf(b->bank_v_fast - 11.0f) < 0.01f;
        sim_hal_set_supply(true, NAN);                     /* a broken sample: not a measurement */
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool nan_ignored = fabsf(b->bank_v - slow_30s) < 0.02f && b->valid && !b->no_data && isfinite(b->bank_v) &&
                           isfinite(b->bank_v_fast);
        sim_hal_set_supply(true, 11.0f);
        sbsim_run(&x, x.t + 120000u, NULL, NULL);
        bool settled = fabsf(b->bank_v - 11.0f) < 0.05f && b->raw_v == 11.0f;
        snprintf(d, sizeof d, "first sample 12.6 V into both filters %s; after a 1.6 V step: fast %.2f V slow %.2f V "
                 "at 1 s %s; slow %.2f V at 30 s %s; a NaN sample ignored %s; settled at 11.0 V %s",
                 first ? "yes" : "NO", fast_1s, slow_1s, fast_ok ? "ok" : "WRONG", slow_30s, slow_ok ? "ok" : "WRONG",
                 nan_ignored ? "yes" : "NO", settled ? "yes" : "NO");
        test_report("BV1 bank voltage filters: first sample, fast ~1 s, slow ~30 s, NaN / inf samples ignored",
                    first && fast_ok && slow_ok && nan_ignored && settled, d);
        sim_hal_set_supply(true, 12.6f);
    }

    /* BV2 LOW BATTERY on / off: below 11.8 V for 60 s -> alarm (advisory: no mode change, the power limit stays,
     * no SAVE); clears above 12.1 V; the acknowledge keeps it while the cause is there */
    {
        static sbsim_t x;
        sbsim_init(&x, 1502u, 90.0f, 0.0f, NULL);
        sim_hal_set_supply(true, 12.6f);
        sbsim_run(&x, 2000u, NULL, NULL);
        bool raised = power_step(&x, 0u) && x.sb.plim.fwd_pct == 50.0f;
        /* HDG LOCK with the throttle on the stick (entering MANUAL would lower the limit by design: override) */
        press(&x, HAL_BTN_HDG_LOCK);
        joy_set(0.0f, 0.5f, false, false);                 /* half throttle ahead, HDG LOCK stays */
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool driving = sb_mode(&x.sb) == MODE_HDG_LOCK && x.sb.plim.fwd_pct == 50.0f;
        uint32_t modes0 = x.sb.modes.log_count;
        uint16_t save0 = x.sb.save.seq;
        sim_hal_set_supply(true, 11.5f);                   /* bank_v < 11.8 after ~39 s, the alarm 60 s later */
        sbsim_run(&x, x.t + 90000u, NULL, NULL);
        bool not_yet = x.sb.alarm_bits2 == 0u && !x.sb.batt.low && x.sb.batt.bank_v < 11.8f;
        sbsim_run(&x, x.t + 20000u, NULL, NULL);          /* ~110 s */
        bool on = (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY) && x.sb.batt.low && x.sb.alarm && sim_hal_alarm();
        bool advisory = sb_mode(&x.sb) == MODE_HDG_LOCK && x.sb.modes.log_count == modes0 &&
                        x.sb.plim.fwd_pct == 50.0f && x.sb.save.seq == save0 && x.sb.thr.out_pct > 40.0f &&
                        x.sb.gps.last.sog_kn > 3.0f;
        sb_alarm_ack(&x.sb);                               /* acknowledged while still low: stays */
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        bool kept = (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY) != 0u;
        sim_hal_set_supply(true, 12.0f);                   /* better, but not above 11.8 + 0.3 */
        sbsim_run(&x, x.t + 150000u, NULL, NULL);
        bool hyst = x.sb.batt.low && fabsf(x.sb.batt.bank_v - 12.0f) < 0.05f;
        sb_alarm_ack(&x.sb);
        bool kept2 = (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY) != 0u;
        sim_hal_set_supply(true, 12.5f);                   /* above 12.1 V: the condition clears */
        sbsim_run(&x, x.t + 60000u, NULL, NULL);
        bool cleared_cond = !x.sb.batt.low && (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY);     /* shown until acknowledged */
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        bool off = x.sb.alarm_bits2 == 0u && !x.sb.alarm && !sim_hal_alarm() && sb_mode(&x.sb) == MODE_HDG_LOCK &&
                   x.sb.plim.fwd_pct == 50.0f;
        snprintf(d, sizeof d, "power 50 %% %s, HDG LOCK driving %s; at 11.5 V: no alarm at 90 s %s, LOW BATTERY at 110 s "
                 "%s (buzzer); still HDG LOCK, no transition, power limit 50 %%, no SAVE, throttle kept %s; ACK while "
                 "low keeps it %s; 12.0 V: still low (hysteresis) %s, ACK keeps %s; 12.5 V: condition gone %s; ACK -> "
                 "off %s", raised ? "yes" : "NO", driving ? "yes" : "NO", not_yet ? "ok" : "NO", on ? "yes" : "NO",
                 advisory ? "yes" : "NO", kept ? "yes" : "NO", hyst ? "yes" : "NO", kept2 ? "yes" : "NO",
                 cleared_cond ? "yes" : "NO", off ? "yes" : "NO");
        test_report("BV2 LOW BATTERY after 60 s below 11.8 V: advisory only (mode + power limit untouched), hysteresis "
                    "0.3 V, acknowledge clears it only once the bank recovered",
                    raised && driving && not_yet && on && advisory && kept && hyst && kept2 && cleared_cond && off, d);
        joy_set(0.0f, 0.0f, false, false);
        sim_hal_set_supply(true, 12.6f);
    }

    /* BV3 no voltage data: notice after 5 s, never an alarm, nothing changes; data back = filters restart;
     * data lost while LOW = the condition can't be judged (cleared) */
    {
        static sbsim_t x;
        static link_t usb;
        sbsim_init(&x, 1503u, 90.0f, 0.0f, NULL);
        sim_hal_set_supply(true, 12.6f);
        sbsim_run(&x, 2000u, NULL, NULL);
        link_init(&usb, &x.sb);
        uint32_t modes0 = x.sb.modes.log_count;
        sim_hal_set_supply(false, 0.0f);                   /* the board says "not measured" */
        sbsim_run(&x, x.t + 3000u, usb_link_hook, &usb);
        bool grace = !x.sb.batt.no_data && x.sb.batt.valid;              /* a 3 s gap is not "no data" yet */
        sbsim_run(&x, x.t + 3000u, usb_link_hook, &usb);
        bool notice = x.sb.batt.no_data && !x.sb.batt.valid && x.sb.alarm_bits2 == 0u && !x.sb.alarm &&
                      !sim_hal_alarm() && sb_mode(&x.sb) == MODE_STANDBY && x.sb.modes.log_count == modes0 &&
                      x.sb.plim.fwd_pct == 30.0f;
        uint16_t t2_len = usb.telem2_len;                  /* TELEM2 carries group L (11 bytes after K) */
        sim_hal_set_supply(true, 11.0f);                   /* data back: the filters start from this sample */
        sbsim_run(&x, x.t + 100u, usb_link_hook, &usb);
        bool restart = !x.sb.batt.no_data && x.sb.batt.valid && fabsf(x.sb.batt.bank_v - 11.0f) < 0.01f;
        sbsim_run(&x, x.t + 70000u, usb_link_hook, &usb);
        bool low = x.sb.batt.low && (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY);
        sim_hal_set_supply(false, 0.0f);                   /* the data goes while low */
        sbsim_run(&x, x.t + 6000u, usb_link_hook, &usb);
        bool unknown = !x.sb.batt.low && x.sb.batt.no_data && (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY);
        sb_alarm_ack(&x.sb);
        bool ack_clears = x.sb.alarm_bits2 == 0u;          /* the cause can't be seen any more */
        snprintf(d, sizeof d, "3 s gap: no notice %s; 6 s: NO VOLTAGE DATA notice %s (no alarm, STANDBY, no transition, "
                 "power limit 30 %%); TELEM2 %u bytes (>= 120 with group L) %s; data back at 11.0 V: filters restart "
                 "%s; LOW BATTERY after 70 s %s; data lost while low: condition cleared, notice %s; ACK clears %s",
                 grace ? "ok" : "NO", notice ? "yes" : "NO", t2_len, t2_len >= 120u ? "ok" : "SHORT",
                 restart ? "yes" : "NO", low ? "yes" : "NO", unknown ? "yes" : "NO", ack_clears ? "yes" : "NO");
        test_report("BV3 no voltage data: notice after 5 s, never an alarm, nothing else changes; data back restarts "
                    "the filters; data lost while low = cause unknown",
                    grace && notice && t2_len >= 120u && restart && low && unknown && ack_clears, d);
        sim_hal_set_supply(true, 12.6f);
    }

    /* BV4 the parameter batt_low_v: limits 10..13.5 V (PC), STANDBY only, applied at once */
    {
        static pcsim_t p;
        pcsim_init(&p, 1504u);
        sim_hal_set_supply(true, 12.6f);
        pc_run(&p, 2000u);
        uint8_t id = 0xFFu;
        for (uint8_t i = 0; i < link_param_count(); i++)
            if (strcmp(link_param_info(i)->name, "batt_low_v") == 0) id = i;
        bool found = id != 0xFFu && link_param_info(id)->min == 10.0f && link_param_info(id)->max == 13.5f &&
                     p.x.sb.batt.cfg.low_v == 11.8f;
        uint8_t ps[5]; proto_wr_t w;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 9.0f);
        bool low_refused = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_BAD_VALUE && p.x.sb.batt.cfg.low_v == 11.8f;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 14.0f);
        bool high_refused = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_BAD_VALUE && p.x.sb.batt.cfg.low_v == 11.8f;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 12.3f);
        bool set = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_OK && p.x.sb.batt.cfg.low_v == 12.3f &&
                   fabsf(p.param_value - 12.3f) < 1e-4f;
        sim_hal_set_supply(true, 12.0f);                   /* below the new limit (not below the default 11.8) */
        pc_run(&p, 100000u);
        bool alarm = (p.x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY) && p.x.sb.batt.low && sb_mode(&p.x.sb) == MODE_STANDBY;
        joy_set(0.0f, 0.5f, false, false);                 /* MANUAL: the parameter is refused while driving */
        pc_run(&p, 1000u);
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 11.0f);
        bool driving = sb_mode(&p.x.sb) == MODE_MANUAL &&
                       pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u) == PROTO_R_STANDBY_ONLY && p.x.sb.batt.cfg.low_v == 12.3f;
        joy_set(0.0f, 0.0f, false, false);
        snprintf(d, sizeof d, "parameter batt_low_v id %u, limits 10..13.5, default 11.8 %s; 9.0 refused %s; 14.0 refused "
                 "%s; 12.3 set + echoed %s; bank 12.0 V -> LOW BATTERY at 100 s %s; refused while driving %s",
                 id, found ? "ok" : "NO", low_refused ? "yes" : "NO", high_refused ? "yes" : "NO", set ? "yes" : "NO",
                 alarm ? "yes" : "NO", driving ? "yes" : "NO");
        test_report("BV4 parameter batt_low_v: limits 10..13.5 V, STANDBY only, used by the LOW BATTERY check at once",
                    found && low_refused && high_refused && set && alarm && driving, d);
        sim_hal_set_supply(true, 12.6f);
    }
}

/* ------------------------------------------------ RS485 relay board (M) -- */

/* the sample table of boats/rf49d.json (IGNITION / STARTER / SHUTOFF engine, TRIM UP / DOWN pair,
 * NAV LIGHTS, BILGE PUMP timed 120 s); power_on: NAV LIGHTS power_up "on"; variant: the bilge pump 60 s */
static void rb_table(relay_table_t *t, bool variant, bool power_on)
{
    static const relay_def_t R[7] = {
        { 1, RELAY_ONOFF,     RELAY_GRP_ENGINE, 0, 0,    RELAY_F_SHOW_HUD, RELAY_ROLE_IGNITION },   /* IGNITION */
        { 2, RELAY_MOMENTARY, RELAY_GRP_ENGINE, 0, 10,   0, RELAY_ROLE_STARTER },  /* STARTER */
        { 3, RELAY_ONOFF,     RELAY_GRP_ENGINE, 0, 0,    RELAY_F_SHOW_HUD, RELAY_ROLE_SHUTOFF },    /* SHUTOFF */
        { 4, RELAY_MOMENTARY, RELAY_GRP_TRIM,   5, 30,   RELAY_F_SHOW_HUD, 0 },   /* TRIM UP */
        { 5, RELAY_MOMENTARY, RELAY_GRP_TRIM,   4, 30,   RELAY_F_SHOW_HUD, 0 },   /* TRIM DOWN */
        { 6, RELAY_ONOFF,     RELAY_GRP_EXTRA,  0, 0,    RELAY_F_KEEP, 0 },       /* NAV LIGHTS */
        { 7, RELAY_TIMED,     RELAY_GRP_EXTRA,  0, 1200, 0, 0 },                  /* BILGE PUMP */
    };
    memset(t, 0, sizeof *t);
    t->count = 7u;
    memcpy(t->r, R, sizeof R);
    if (variant) t->r[6].max_ds = 600u;
    if (power_on) t->r[5].flags = RELAY_F_POWER_ON;
    relay_table_seal(t);
}

/* a fresh boat (CREWED, armed) with a fresh relay board; table loaded through the real path
 * (offer in STANDBY + joystick CONFIRM) when t != NULL */
static bool rb_setup(sbsim_t *x, uint32_t seed, const sb_config_t *cfg, const relay_table_t *t)
{
    sbsim_init(x, seed, 90.0f, 0.0f, cfg);          /* (also: no nv7.bin, a fresh board) */
    sbsim_run(x, 8000u, NULL, NULL);                 /* profile detected (CREWED), armed */
    if (t == NULL) return true;
    if (sb_relay_table(&x->sb, t) != SB_OK) return false;
    bool ok = confirm_press(x) && x->sb.relays.have_table;
    sbsim_run(x, x->t + 1000u, NULL, NULL);
    return ok;
}

/* the AURIX's own reset (the board on its own supply keeps its relays): the data flash files stay */
static void rb_power_cycle(sbsim_t *x, hal_reset_t why)
{
    memset((void *)&g_mon, 0, sizeof g_mon);
    if (why == HAL_RESET_POWER_ON) sim_hal_unforce(); else sim_hal_chip_reset(why);
    hal_init();
    sb_init(&x->sb, &x->cfg);
    monitor_init(&x->mon, x->t);
    x->next_ctrl = x->t;
    x->last_kick = x->t;
}

static void relay_bus_tests(void);

static uint32_t rb_bit(int n)
{
    return 1u << (n - 1);
}

/* the relay board of this boot is the DroneCAN one (step 11f: "all --can", or the RB15.. tests) */
static bool rb_can(void)
{
    return sim_hal_relay_bus_now() == SIM_RELAY_DRONECAN;
}

static void relay_tests(void)
{
    char d[1200];
    printf("\n=== RB relay board on %s (relays.c): advisory only, never a control path ===\n",
           (s_can_all || s_rb_can) ? "DroneCAN (can_relay.c, node 30)" : "RS485 Modbus (tc_modbus.c)");

    /* RB1 the driver: readback, a command reaches the board, a slow board (reply > 100 ms) = timeouts,
     * after 3 s RELAY BOARD SILENT (advisory), states unknown; back = found by itself, the ACK clears it */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1701u, NULL, &t);
        const mb_stats_t *st = mb_stats();
        bool can = rb_can();
        bool known = x.sb.relays.board_ok && (x.sb.relays.known & 0x7Fu) == 0x7Fu && x.sb.relays.states == 0u;
        bool on = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        bool on_board = sim_hal_relay_state(6u) && (x.sb.relays.states & rb_bit(6));
        bool off = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        bool off_board = !sim_hal_relay_state(6u) && !(x.sb.relays.states & rb_bit(6)) && x.sb.relays.off_pending == 0u;
        uint32_t to0 = st->timeouts;
        /* Modbus: replies later than the 100 ms limit; DroneCAN: the board stops sending (NodeStatus + Status) */
        if (can) sim_hal_relay_board(true, 5u, SIM_RB_SILENT); else sim_hal_relay_board(true, 150u, 0u);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool still_ok = x.sb.relays.board_ok && !(x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool silent = !x.sb.relays.board_ok && x.sb.relays.known == 0u && (x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT) &&
                      (can || st->timeouts > to0 + 10u) && sb_mode(&x.sb) == MODE_STANDBY;
        sim_hal_relay_board(true, 5u, 0u);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool back = x.sb.relays.board_ok && (x.sb.relays.known & 0x7Fu) == 0x7Fu;
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 100u, NULL, NULL);
        bool cleared = !(x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT);
        snprintf(d, sizeof d, "table loaded %s, readback %s, ON on board %s, OFF %s; slow board: ok for 2 s %s, SILENT after "
                 "3.5 s %s (timeouts %u), back %s, ACK clears %s",
                 loaded ? "yes" : "NO", known ? "ok" : "NO", on && on_board ? "yes" : "NO", off && off_board ? "yes" : "NO",
                 still_ok ? "yes" : "NO", silent ? "yes" : "NO", (unsigned)(st->timeouts - to0), back ? "yes" : "NO",
                 cleared ? "yes" : "NO");
        test_report("RB1 driver: readback, commands reach the board, timeouts -> SILENT after 3 s (advisory), back by itself",
                    loaded && known && on && on_board && off && off_board && still_ok && silent && back && cleared, d);
    }

    /* RB2 CRC errors: every reply bad -> counted, never used, SILENT; one in 7 bad -> the board stays OK */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1702u, NULL, &t);
        const mb_stats_t *st = mb_stats();
        bool can = rb_can();                        /* (DroneCAN: the transport refuses the broken frames) */
#define RB2_ERRS() (can ? dc_stats()->rx_errors : st->crc_errors)
        uint32_t c0 = RB2_ERRS();
        sim_hal_relay_board(true, 5u, SIM_RB_CORRUPT);
        sim_hal_relay_force(6u, true);              /* a hand switch on the board: the bad replies must not show it */
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool all_bad = RB2_ERRS() > c0 + 10u && !x.sb.relays.board_ok && x.sb.relays.known == 0u &&
                       !(x.sb.relays.states & rb_bit(6)) && (x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT);
        uint32_t c1 = RB2_ERRS();
        sim_hal_relay_board(true, 5u, SIM_RB_CORRUPT_SOME);
        sbsim_run(&x, x.t + 10000u, NULL, NULL);
        bool some = RB2_ERRS() > c1 && x.sb.relays.board_ok && (x.sb.relays.states & rb_bit(6));
        sim_hal_relay_board(true, 5u, 0u);
        snprintf(d, sizeof d, "all %s bad: %u errors, SILENT + states unknown %s; 1 in 7 bad: %u errors, board ok %s, "
                 "readback shows the hand-switched relay %s", can ? "frames" : "replies", (unsigned)(c1 - c0),
                 all_bad ? "yes" : "NO", (unsigned)(RB2_ERRS() - c1), x.sb.relays.board_ok ? "yes" : "NO",
                 some ? "yes" : "NO");
#undef RB2_ERRS
        test_report(can ? "RB2 broken frames: refused by the transport, never used (SILENT); occasional errors tolerated"
                        : "RB2 CRC errors: bad replies counted and never used (SILENT); occasional errors tolerated",
                    loaded && all_bad && some, d);
    }

    /* RB3 a silent board is an ADVISORY only: driving in HDG LOCK, the board dies -> same mode, same power limit,
     * same throttle, no first-word alarm; new commands refused; an OFF is kept and re-sent when the board is back */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1703u, NULL, &t);
        bool on = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        bool hdg = hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 40.0f);
        sbsim_run(&x, x.t + 15000u, NULL, NULL);
        float plim0 = x.sb.plim.fwd_pct, thr0 = x.sb.thr.out_pct;
        uint32_t alm0 = x.sb.alarm_bits;
        sim_hal_relay_board(false, 5u, 0u);          /* the bus is cut */
        sbsim_run(&x, x.t + 5000u, NULL, NULL);
        bool advisory = sb_mode(&x.sb) == MODE_HDG_LOCK && x.sb.plim.fwd_pct == plim0 &&
                        fabsf(x.sb.thr.out_pct - thr0) < 0.5f && x.sb.alarm_bits == alm0 &&
                        (x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT) && relays_silent(&x.sb.relays);
        bool refused = sb_relay_cmd(&x.sb, 7u, RELAY_CMD_ON, 0u) == SB_ERR_REFUSED &&
                       sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 1000u) == SB_ERR_REFUSED;
        bool off_asked = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u) == SB_OK;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool still_on = sim_hal_relay_state(6u) && (x.sb.relays.off_pending & rb_bit(6));
        sim_hal_relay_board(true, 5u, 0u);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool off_done = !sim_hal_relay_state(6u) && x.sb.relays.off_pending == 0u;
        snprintf(d, sizeof d, "HDG LOCK %s at %.0f %% (limit %.0f %%); board cut: mode %s, limit %.0f %%, throttle %.0f %%, alarm word 1 "
                 "unchanged %s, SILENT %s; new ON / pulse refused %s; OFF kept while silent %s, carried out when back %s",
                 hdg ? "yes" : "NO", thr0, plim0, mode_name(sb_mode(&x.sb)), x.sb.plim.fwd_pct, x.sb.thr.out_pct,
                 x.sb.alarm_bits == alm0 ? "yes" : "NO", advisory ? "yes" : "NO", refused ? "yes" : "NO",
                 off_asked && still_on ? "yes" : "NO", off_done ? "yes" : "NO");
        sb_test_drive(&x.sb, false, GEAR_N, 0.0f);
        test_report("RB3 silent board = advisory only: no mode / power limit / throttle change, no new command, OFF kept",
                    loaded && on && hdg && thr0 > 10.0f && advisory && refused && off_asked && still_on && off_done, d);
    }

    /* RB4 interlocked pair (TRIM UP / DOWN): never both - refused while the partner is commanded on or read
     * back on (a hand switch on the board); allowed once it is off */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1704u, NULL, &t);
        bool up = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 2000u) == SB_OK;
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool down_refused = sb_relay_cmd(&x.sb, 5u, RELAY_CMD_PULSE, 2000u) == SB_ERR_REFUSED && !sim_hal_relay_state(5u);
        sbsim_run(&x, x.t + 2500u, NULL, NULL);
        bool down_ok = !sim_hal_relay_state(4u) && sb_relay_cmd(&x.sb, 5u, RELAY_CMD_PULSE, 500u) == SB_OK;
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        sim_hal_relay_force(5u, true);               /* TRIM DOWN stuck on at the board */
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool up_refused = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_ON, 0u) == SB_ERR_REFUSED && !sim_hal_relay_state(4u);
        sim_hal_relay_force(5u, false);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool never_both = true;
        for (int i = 0; i < 60; i++) {
            (void)sb_relay_cmd(&x.sb, (uint8_t)(4 + (i & 1)), RELAY_CMD_PULSE, 300u);
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            if (sim_hal_relay_state(4u) && sim_hal_relay_state(5u)) never_both = false;
        }
        snprintf(d, sizeof d, "UP pulse %s; DOWN refused while UP on %s; DOWN after UP ended %s; UP refused while DOWN read back on %s; "
                 "alternating requests never both %s", up ? "ok" : "NO", down_refused ? "yes" : "NO", down_ok ? "yes" : "NO",
                 up_refused ? "yes" : "NO", never_both ? "yes" : "NO");
        test_report("RB4 interlocked pair: never both on (commanded or read back)",
                    loaded && up && down_refused && down_ok && up_refused && never_both, d);
    }

    /* RB5 time limits: a momentary relay = a BOARD-timed pulse <= max_s (a 20 s request is cut to 3 s; the board
     * ends it even with core 0 frozen); a timed relay is switched OFF by the AURIX after max_s */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1705u, NULL, &t);
        uint32_t p0 = sim_hal_relay_pulses();
        bool asked = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 20000u) == SB_OK;
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool pulsing = sim_hal_relay_state(4u) && sim_hal_relay_pulses() == p0 + 1u;
        x.wdt_ms = 0u;                               /* core 0 frozen, no reset: only the board can end it */
        x.frozen = true;
        uint32_t t0 = x.t, t_off = 0u;
        while (x.t < t0 + 6000u) {
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            if (!t_off && !sim_hal_relay_state(4u)) t_off = x.t - t0 + 300u;
        }
        x.frozen = false;
        x.wdt_ms = 100u;
        x.next_ctrl = x.t;
        x.last_kick = x.t;
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool timed = sb_relay_cmd(&x.sb, 7u, RELAY_CMD_ON, 0u) == SB_OK;
        uint32_t t1 = x.t;
        sbsim_run(&x, t1 + 119000u, NULL, NULL);
        bool on_119 = sim_hal_relay_state(7u);
        sbsim_run(&x, t1 + 121000u, NULL, NULL);
        bool off_121 = !sim_hal_relay_state(7u) && x.sb.relays.timed_offs == 1u;
        /* the timed OFF falls while the board is silent: carried out when it is back */
        (void)sb_relay_cmd(&x.sb, 7u, RELAY_CMD_ON, 0u);
        uint32_t t2 = x.t;
        sbsim_run(&x, t2 + 60000u, NULL, NULL);
        sim_hal_relay_board(false, 5u, 0u);
        sbsim_run(&x, t2 + 125000u, NULL, NULL);
        bool held = sim_hal_relay_state(7u) && x.sb.relays.timed_offs == 2u;
        sim_hal_relay_board(true, 5u, 0u);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool off_back = !sim_hal_relay_state(7u);
        snprintf(d, sizeof d, "20 s pulse asked %s, pulse %s, ended by the board after ~%u ms with core 0 frozen; timed relay "
                 "on at 119 s %s, off at 121 s by the AURIX %s; OFF during silence waits %s, done when back %s",
                 asked ? "ok" : "NO", pulsing ? "yes" : "NO", (unsigned)t_off, on_119 ? "yes" : "NO", off_121 ? "yes" : "NO",
                 held ? "yes" : "NO", off_back ? "yes" : "NO");
        /* Modbus: the board's own pulse timer ends it at 3 s; DroneCAN: the AURIX stops sending with core 0
         * frozen at 0.3 s - the board's command timeout (R-3, 300 ms) ends it */
        bool t_ok = rb_can() ? (t_off >= 450u && t_off <= 900u) : (t_off >= 2700u && t_off <= 3500u);
        test_report("RB5 time limits: momentary <= max_s, ended by the board also with core 0 frozen; timed OFF after max_s",
                    loaded && asked && pulsing && t_ok && timed && on_119 && off_121 && held && off_back, d);
    }

    /* RB6 refusals: no table, unknown relay, the STARTER (section N only), FIRST TEST (no relay change at all) */
    {
        static sbsim_t x, y;
        static relay_table_t t;
        rb_table(&t, false, false);
        (void)rb_setup(&x, 1706u, NULL, NULL);
        bool no_table = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_ERR_PRECOND && !x.sb.relays.have_table &&
                        !(x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT);
        sim_hal_relay_board(false, 5u, 0u);           /* no table: a missing board is no alarm either */
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool no_alarm = !(x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT);
        sim_hal_relay_board(true, 5u, 0u);
        bool loaded = sb_relay_table(&x.sb, &t) == SB_OK && confirm_press(&x);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        uint32_t w0 = sim_hal_relay_writes();
        bool unknown = sb_relay_cmd(&x.sb, 9u, RELAY_CMD_ON, 0u) == SB_ERR_BAD_VALUE;
        bool starter = sb_relay_cmd(&x.sb, 2u, RELAY_CMD_ON, 0u) == SB_ERR_NOT_ALLOWED &&
                       sb_relay_cmd(&x.sb, 2u, RELAY_CMD_PULSE, 300u) == SB_ERR_NOT_ALLOWED;
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool no_write = sim_hal_relay_writes() == w0 && !sim_hal_relay_state(2u);
        /* FIRST TEST */
        sb_config_t c;
        sb_default_config(&c);
        c.first_test = true;
        (void)rb_setup(&y, 1707u, &c, NULL);
        bool ft_store = sb_relay_table(&y.sb, &t) == SB_OK && confirm_press(&y) && y.sb.relays.have_table;
        uint32_t w1 = sim_hal_relay_writes();
        bool ft = sb_relay_cmd(&y.sb, 6u, RELAY_CMD_ON, 0u) == SB_ERR_WRONG_MODE &&
                  sb_relay_cmd(&y.sb, 6u, RELAY_CMD_OFF, 0u) == SB_ERR_WRONG_MODE &&
                  sb_relay_cmd(&y.sb, 4u, RELAY_CMD_PULSE, 500u) == SB_ERR_WRONG_MODE;
        sbsim_run(&y, y.t + 1000u, NULL, NULL);
        bool ft_none = sim_hal_relay_writes() == w1 && (relays_flags(&y.sb.relays) & 32u) && y.sb.relays.power_ups == 0u;
        snprintf(d, sizeof d, "no table: refused %s, no SILENT alarm without a table %s; unknown relay %s; STARTER refused (on / pulse) "
                 "%s, never written %s; FIRST TEST: table stored %s, ON / OFF / pulse refused %s, no write %s",
                 no_table ? "yes" : "NO", no_alarm ? "yes" : "NO", unknown ? "yes" : "NO", starter ? "yes" : "NO",
                 no_write ? "yes" : "NO", ft_store ? "yes" : "NO", ft ? "yes" : "NO", ft_none ? "yes" : "NO");
        test_report("RB6 refusals: no table, unknown relay, the STARTER, FIRST TEST (no relay change)",
                    no_table && no_alarm && loaded && unknown && starter && no_write && ft_store && ft && ft_none, d);
    }

    /* RB7 the relay table (owner clarification): NV slot 7; a different table only in STANDBY and only after the
     * joystick CONFIRM (no CONFIRM / mode change -> the stored one stays); the same table = OK at once; a bad one
     * refused; after a reboot the stored table is in use; the link: RELAY_CFG / RELAY frames, TELEM2 group M */
    {
        static pcsim_t p;
        static relay_table_t a, b, bad;
        rb_table(&a, false, false);
        rb_table(&b, true, false);
        rb_table(&bad, false, false);
        bad.r[3].pair = 6u;                           /* TRIM UP pairs NAV LIGHTS: not mutual */
        relay_table_seal(&bad);
        pcsim_init(&p, 1708u);
        p.heartbeat = true;
        p.next_hb = p.x.t;
        pc_run(&p, 8000u);
        uint8_t fr[1u + RELAY_TABLE_MAX * RELAY_FRAME_BYTES];
        uint32_t na = relay_table_encode(&a, fr, sizeof fr);
        uint16_t empty_hash = p.t2_len >= 16u ? (uint16_t)(p.t2[p.t2_len - T2_O - 8u] | (p.t2[p.t2_len - T2_O - 7u] << 8)) : 0u;
        bool empty_ok = p.t2_len >= 16u && empty_hash == p.x.sb.relays.hash && !p.x.sb.relays.have_table;
        /* offered, no CONFIRM: dropped after 10 s, nothing active */
        bool offered = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)na) == PROTO_R_OK && p.x.sb.relays.offer_pending &&
                       p.x.sb.confirm.kind == CONF_RELAY_TABLE;
        pc_run(&p, 1000u);
        offered = offered && (p.t2[p.t2_len - T2_O - 6u] & 8u);
        pc_run(&p, 10000u);
        bool dropped = !p.x.sb.relays.offer_pending && !p.x.sb.relays.have_table && p.x.sb.confirm.kind == CONF_NONE;
        /* offered + CONFIRM: active, saved in slot 7 */
        uint32_t nvw = sim_hal_nv_writes();
        bool sent = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)na) == PROTO_R_OK;
        bool conf = confirm_press(&p.x);
        pc_run(&p, 1500u);
        uint16_t t2_hash = (uint16_t)(p.t2[p.t2_len - T2_O - 8u] | (p.t2[p.t2_len - T2_O - 7u] << 8));
        bool active = sent && conf && p.x.sb.relays.have_table && t2_hash == relay_table_hash(&a) && p.t2[p.t2_len - T2_O - 5u] == 7u &&
                      (p.t2[p.t2_len - T2_O - 6u] & 3u) == 3u && sim_hal_nv_writes() > nvw;
        FILE *fp = fopen("nv7.bin", "rb");
        bool saved = fp != NULL;
        if (fp) fclose(fp);
        /* the same table again: OK at once, nothing pending */
        bool same = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)na) == PROTO_R_OK && !p.x.sb.relays.offer_pending &&
                    p.x.sb.confirm.kind == CONF_NONE;
        /* a relay command over the link + TELEM2 readback */
        uint8_t rc[4] = { 6u, RELAY_CMD_ON, 0u, 0u };
        bool cmd = pc_cmd(&p, PROTO_T_RELAY, rc, 4u) == PROTO_R_OK;
        pc_run(&p, 1500u);
        uint32_t t2_states = (uint32_t)p.t2[p.t2_len - T2_O - 16u] | ((uint32_t)p.t2[p.t2_len - T2_O - 15u] << 8);
        bool t2_on = cmd && (t2_states & rb_bit(6)) && sim_hal_relay_state(6u);
        rc[1] = RELAY_CMD_OFF;
        (void)pc_cmd(&p, PROTO_T_RELAY, rc, 4u);
        uint8_t r0[4] = { 0u, 1u, 0u, 0u };
        bool bad_n = pc_cmd(&p, PROTO_T_RELAY, r0, 4u) == PROTO_R_BAD_VALUE && pc_cmd(&p, PROTO_T_RELAY, rc, 3u) == PROTO_R_BAD_LENGTH;
        bool bad_len = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)(na - 1u)) == PROTO_R_BAD_LENGTH;
        uint32_t nbad = relay_table_encode(&bad, fr, sizeof fr);
        bool bad_tab = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)nbad) == PROTO_R_BAD_VALUE && p.x.sb.relays.table.r[6].max_ds == 1200u;
        /* a different table outside STANDBY: refused; offered in STANDBY then HDG LOCK before the CONFIRM: dropped */
        uint32_t nb = relay_table_encode(&b, fr, sizeof fr);
        bool hdg = hdg_lock(&p.x);
        bool not_standby = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)nb) == PROTO_R_STANDBY_ONLY;
        press(&p.x, HAL_BTN_STANDBY);
        pc_run(&p, 3000u);
        bool offered_b = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)nb) == PROTO_R_OK && p.x.sb.relays.offer_pending;
        bool hdg2 = hdg_lock(&p.x);
        pc_run(&p, 200u);
        bool dropped_b = hdg2 && !p.x.sb.relays.offer_pending && p.x.sb.relays.table.r[6].max_ds == 1200u;
        press(&p.x, HAL_BTN_STANDBY);
        pc_run(&p, 3000u);
        /* reboot: the stored table A is in use (from NV); B offered + CONFIRM replaces it, also after the next reboot */
        rb_power_cycle(&p.x, HAL_RESET_SYSTEM);
        link_init(&p.link, &p.x.sb);
        pc_run(&p, 8000u);
        bool from_nv = p.x.sb.relays.have_table && p.x.sb.relays.from_nv && p.x.sb.relays.hash == relay_table_hash(&a);
        bool b_ok = pc_cmd(&p, PROTO_T_RELAY_CFG, fr, (uint8_t)nb) == PROTO_R_OK && confirm_press(&p.x);
        pc_run(&p, 1000u);
        rb_power_cycle(&p.x, HAL_RESET_SYSTEM);
        link_init(&p.link, &p.x.sb);
        pc_run(&p, 2000u);
        bool b_nv = b_ok && p.x.sb.relays.from_nv && p.x.sb.relays.hash == relay_table_hash(&b);
        snprintf(d, sizeof d, "no table = the empty hash %s; offered + TELEM2 pending %s, no CONFIRM -> dropped %s; CONFIRM -> active + "
                 "TELEM2 hash / count / flags + slot 7 %s / %s; same table at once %s; RELAY over the link + TELEM2 states %s; bad n / "
                 "length %s, bad table length %s, not mutual %s; outside STANDBY refused %s (HDG LOCK %s); a mode change drops the "
                 "offer %s; after a reboot the stored table %s; the new one confirmed + stored %s",
                 empty_ok ? "yes" : "NO", offered ? "yes" : "NO", dropped ? "yes" : "NO", active ? "yes" : "NO", saved ? "yes" : "NO",
                 same ? "yes" : "NO", t2_on ? "yes" : "NO", bad_n ? "yes" : "NO", bad_len ? "yes" : "NO", bad_tab ? "yes" : "NO",
                 not_standby ? "yes" : "NO", hdg ? "yes" : "NO", offered_b && dropped_b ? "yes" : "NO", from_nv ? "yes" : "NO",
                 b_nv ? "yes" : "NO");
        test_report("RB7 relay table: slot 7, a different table only in STANDBY + CONFIRM, the stored one stays until then; link + TELEM2 M",
                    empty_ok && offered && dropped && active && saved && same && t2_on && bad_n && bad_len && bad_tab && hdg &&
                    not_standby && offered_b && dropped_b && from_nv && b_nv, d);
        remove("nv7.bin");
    }

    /* RB8 power_up (owner clarification): ONLY switching ON, only once, only with the first table after the AURIX's
     * own POWER-ON reset, never the engine group (such a table is refused), never switching a relay OFF; after a
     * watchdog / software reset the board is read back and its states kept */
    {
        static sbsim_t x;
        static relay_table_t a, a2, eng;
        rb_table(&a, false, true);                   /* NAV LIGHTS power_up "on" */
        rb_table(&a2, true, true);
        rb_table(&eng, false, false);
        eng.r[0].flags = RELAY_F_POWER_ON;            /* IGNITION power_up on: never */
        relay_table_seal(&eng);
        bool eng_bad = !relay_table_valid(&eng);
        /* the first table after a power-on reset: NAV LIGHTS on, once */
        bool loaded = rb_setup(&x, 1709u, NULL, &a);
        bool pu_first = sim_hal_relay_state(6u) && x.sb.relays.power_ups == 1u;
        bool eng_refused = sb_relay_table(&x.sb, &eng) == SB_ERR_BAD_VALUE;
        /* nothing more: a second (different) table, CONFIRMed - no new power_up */
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool second = sb_relay_table(&x.sb, &a2) == SB_OK && confirm_press(&x);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool once = second && !sim_hal_relay_state(6u) && x.sb.relays.power_ups == 1u;
        /* a watchdog reset with relays on the board: read back, kept, no power_up */
        sim_hal_relay_force(3u, true);                /* SHUTOFF on (power_up off: never switched OFF by the AURIX) */
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool wd = x.sb.relays.from_nv && x.sb.relays.power_ups == 0u && !sim_hal_relay_state(6u) && sim_hal_relay_state(3u) &&
                  (x.sb.relays.states & rb_bit(3));
        /* a real power-on reset with the stored table: NAV LIGHTS on again (only ON), SHUTOFF kept on, engine untouched */
        uint32_t w0 = sim_hal_relay_writes();
        rb_power_cycle(&x, HAL_RESET_POWER_ON);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool po = x.sb.relays.power_ups == 1u && sim_hal_relay_state(6u) && sim_hal_relay_state(3u) && !sim_hal_relay_state(1u) &&
                  sim_hal_relay_writes() == w0 + 1u;
        /* a board that is silent at power-on: power_up waits until it answers */
        sim_hal_relay_board(false, 5u, 0u);
        sim_hal_relay_board_reset();
        rb_power_cycle(&x, HAL_RESET_POWER_ON);
        sbsim_run(&x, x.t + 5000u, NULL, NULL);
        bool waits = x.sb.relays.power_ups == 0u && !sim_hal_relay_state(6u);
        sim_hal_relay_board(true, 5u, 0u);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool late = x.sb.relays.power_ups == 1u && sim_hal_relay_state(6u);
        snprintf(d, sizeof d, "engine power_up table invalid %s / refused %s; first table after power-on: NAV LIGHTS on once %s, not again "
                 "with a second table %s; watchdog reset: no power_up, board states kept %s; power-on reset: only ON, one write, "
                 "SHUTOFF kept on, IGNITION untouched %s; silent board at power-on: waits %s, applied when it answers %s",
                 eng_bad ? "yes" : "NO", eng_refused ? "yes" : "NO", pu_first ? "yes" : "NO", once ? "yes" : "NO", wd ? "yes" : "NO",
                 po ? "yes" : "NO", waits ? "yes" : "NO", late ? "yes" : "NO");
        test_report("RB8 power_up: only ON, once, only after a power-on reset, never the engine group, board states kept",
                    eng_bad && loaded && pu_first && eng_refused && once && wd && po && waits && late, d);
        remove("nv7.bin");
        sim_hal_relay_board_reset();
    }

    /* RB9 (step 7b, review 1) the engine ROLES: the STARTER = an explicit role, a momentary relay of <= 5 s
     * (typed otherwise = an INVALID table); every engine relay has a role, no other relay has one; each role
     * once. No engine relay is switched ON by the plain relay command: IGNITION only with idle + neutral
     * confirmed and the e-stop released (engine.c), SHUTOFF ON (= stop the engine) ALWAYS (step 11b: also while
     * driving and with the e-stop pressed), the STARTER never (hold-to-crank only); OFF always */
    {
        static sbsim_t x;
        static relay_table_t t, bad;
        rb_table(&t, false, false);
        int invalid = 0;
        bad = t; bad.r[1].type = RELAY_ONOFF; relay_table_seal(&bad); invalid += !relay_table_valid(&bad);  /* STARTER onoff */
        bad = t; bad.r[1].type = RELAY_TIMED; relay_table_seal(&bad); invalid += !relay_table_valid(&bad);  /* STARTER timed */
        bad = t; bad.r[1].max_ds = 60u; relay_table_seal(&bad); invalid += !relay_table_valid(&bad);        /* 6 s */
        bad = t; bad.r[0].role = RELAY_ROLE_NONE; relay_table_seal(&bad); invalid += !relay_table_valid(&bad); /* no role */
        bad = t; bad.r[5].role = RELAY_ROLE_IGNITION; relay_table_seal(&bad); invalid += !relay_table_valid(&bad); /* extra */
        bad = t; bad.r[2].role = RELAY_ROLE_STARTER; bad.r[2].type = RELAY_MOMENTARY; bad.r[2].max_ds = 10u;
        relay_table_seal(&bad); invalid += !relay_table_valid(&bad);                                       /* 2 STARTERs */
        bad = t; bad.r[0].type = RELAY_MOMENTARY; bad.r[0].max_ds = 10u; relay_table_seal(&bad);
        invalid += !relay_table_valid(&bad);                                                               /* IGNITION pulse */
        bad = t; bad.r[1].role = RELAY_ROLE_NONE; bad.r[1].group = RELAY_GRP_EXTRA; bad.r[3].role = RELAY_ROLE_STARTER;
        relay_table_seal(&bad); invalid += !relay_table_valid(&bad);                                       /* trim as STARTER */
        /* step 11g: an engine relay is never in a pair (SHUTOFF ON must never be refused by the pair rule) */
        bad = t; bad.r[2].pair = 1u; bad.r[0].pair = 3u; relay_table_seal(&bad); invalid += !relay_table_valid(&bad); /* SHUTOFF-IGN */
        bad = t; bad.r[2].pair = 6u; bad.r[5].pair = 3u; relay_table_seal(&bad); invalid += !relay_table_valid(&bad); /* SHUTOFF-extra */
        bool loaded = rb_setup(&x, 1710u, NULL, &t);
        bool link_bad = sb_relay_table(&x.sb, &bad) == SB_ERR_BAD_VALUE && relays_starter(&x.sb.relays) == 2u;
        /* STANDBY, idle, neutral: IGNITION on / off through the engine rules */
        bool ign_on = sb_relay_cmd(&x.sb, 1u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        ign_on = ign_on && sim_hal_relay_state(1u);
        bool ign_off = sb_relay_cmd(&x.sb, 1u, RELAY_CMD_OFF, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        ign_off = ign_off && !sim_hal_relay_state(1u);
        bool starter = sb_relay_cmd(&x.sb, 2u, RELAY_CMD_ON, 0u) == SB_ERR_NOT_ALLOWED &&
                       sb_relay_cmd(&x.sb, 2u, RELAY_CMD_PULSE, 300u) == SB_ERR_NOT_ALLOWED;
        /* driving (throttle not idle, gear F): IGNITION ON refused, SHUTOFF ON (stop the engine) and OFF go */
        uint32_t e1 = sim_hal_relay_edges(1u), e3 = sim_hal_relay_edges(3u);
        bool hdg = hdg_lock(&x);
        sb_test_drive(&x.sb, true, GEAR_F, 40.0f);
        sbsim_run(&x, x.t + 8000u, NULL, NULL);
        bool drive_ref = x.sb.thr.out_pct > 10.0f && sb_relay_cmd(&x.sb, 1u, RELAY_CMD_ON, 0u) == SB_ERR_PRECOND &&
                         sb_relay_cmd(&x.sb, 1u, RELAY_CMD_PULSE, 500u) == SB_ERR_PRECOND;
        bool shut_drive = sb_relay_cmd(&x.sb, 3u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        shut_drive = shut_drive && sim_hal_relay_state(3u) && sb_mode(&x.sb) == MODE_HDG_LOCK &&
                     sb_relay_cmd(&x.sb, 3u, RELAY_CMD_OFF, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        shut_drive = shut_drive && !sim_hal_relay_state(3u);
        sb_test_drive(&x.sb, false, GEAR_N, 0.0f);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 6000u, NULL, NULL);
        /* the e-stop pressed: IGNITION refused (REFUSED), SHUTOFF ON goes; released: IGNITION allowed again */
        sim_hal_set_estop(true, true);
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool estop_ref = sb_relay_cmd(&x.sb, 1u, RELAY_CMD_ON, 0u) == SB_ERR_REFUSED;
        bool shut_es = sb_relay_cmd(&x.sb, 3u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        shut_es = shut_es && sim_hal_relay_state(3u) && sim_hal_relay_edges(3u) == e3 + 2u;
        (void)sb_relay_cmd(&x.sb, 3u, RELAY_CMD_OFF, 0u);
        bool never = sim_hal_relay_edges(1u) == e1;
        /* step 11g: even a table that slipped a pair onto SHUTOFF (forced into RAM here: a valid table can't) - the
         * pair rule never refuses SHUTOFF ON while its partner is on */
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        x.sb.relays.table.r[2].pair = 6u;
        x.sb.relays.table.r[5].pair = 3u;
        bool nav_on = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        bool shut_pair = nav_on && sim_hal_relay_state(6u) && sb_relay_cmd(&x.sb, 3u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        shut_pair = shut_pair && sim_hal_relay_state(3u);
        (void)sb_relay_cmd(&x.sb, 3u, RELAY_CMD_OFF, 0u);
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u);
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        x.sb.relays.table.r[2].pair = 0u;
        x.sb.relays.table.r[5].pair = 0u;
        sim_hal_set_estop(true, false);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool again = sb_relay_cmd(&x.sb, 1u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        again = again && sim_hal_relay_state(1u);
        (void)sb_relay_cmd(&x.sb, 1u, RELAY_CMD_OFF, 0u);
        sim_hal_set_estop(false, false);
        snprintf(d, sizeof d, "bad tables INVALID %d / 10 (STARTER onoff / timed / 6 s, engine without role, role outside the "
                 "engine, two STARTERs, IGNITION momentary, a trim relay as STARTER, SHUTOFF paired with IGNITION / an "
                 "extra), refused over the link %s; SHUTOFF ON with a forced pair partner on %s; STANDBY: "
                 "IGNITION on %s / off %s, STARTER by the plain command refused %s; driving (HDG LOCK %s): IGNITION ON "
                 "refused %s, SHUTOFF ON (engine stop) + OFF ok, still HDG LOCK %s; e-stop: IGNITION refused %s, SHUTOFF ON "
                 "ok %s, IGNITION coil never on meanwhile %s; released: ON again %s",
                 invalid, link_bad ? "yes" : "NO", shut_pair ? "yes" : "NO", ign_on ? "yes" : "NO", ign_off ? "yes" : "NO", starter ? "yes" : "NO",
                 hdg ? "yes" : "NO", drive_ref ? "yes" : "NO", shut_drive ? "yes" : "NO", estop_ref ? "yes" : "NO",
                 shut_es ? "yes" : "NO", never ? "yes" : "NO", again ? "yes" : "NO");
        test_report("RB9 engine roles: STARTER only by role (momentary <= 5 s, else INVALID); IGNITION ON only idle + neutral "
                    "+ e-stop released; SHUTOFF ON (engine stop) always, never in a pair", invalid == 10 && loaded &&
                    link_bad && shut_pair && ign_on &&
                    ign_off && starter && hdg && drive_ref && shut_drive && estop_ref && shut_es && never && again, d);
        remove("nv7.bin");
    }

    /* RB10 (review 2) a relay-table offer never replaces another pending CONFIRM: with a power raise waiting,
     * a different table is refused (the raise stays pending and is confirmed), the same table is OK at once;
     * once nothing is pending the offer works again */
    {
        static sbsim_t x;
        static relay_table_t a, b;
        rb_table(&a, false, false);
        rb_table(&b, true, false);
        bool loaded = rb_setup(&x, 1711u, NULL, &a);
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        uint16_t code = 0u;
        bool raise = sb_power_code(&x.sb, 0u, &code) == SB_OK && sb_power_raise(&x.sb, 0u, code) == SB_OK &&
                     x.sb.confirm.kind == CONF_RAISE_FWD;
        bool refused = sb_relay_table(&x.sb, &b) == SB_ERR_REFUSED && x.sb.confirm.kind == CONF_RAISE_FWD &&
                       !x.sb.relays.offer_pending;
        bool same = sb_relay_table(&x.sb, &a) == SB_OK && x.sb.confirm.kind == CONF_RAISE_FWD;
        bool raised = confirm_press(&x) && x.sb.plim.fwd_pct > 30.0f && x.sb.relays.hash == relay_table_hash(&a);
        bool later = sb_relay_table(&x.sb, &b) == SB_OK && x.sb.confirm.kind == CONF_RELAY_TABLE && confirm_press(&x) &&
                     x.sb.relays.hash == relay_table_hash(&b);
        snprintf(d, sizeof d, "power raise pending %s; a different table refused, the raise kept %s; the same table OK %s; "
                 "the raise confirmed (limit %.0f %%), table unchanged %s; then the offer + CONFIRM works %s",
                 raise ? "yes" : "NO", refused ? "yes" : "NO", same ? "yes" : "NO", x.sb.plim.fwd_pct, raised ? "yes" : "NO",
                 later ? "yes" : "NO");
        test_report("RB10 relay-table offer refused while another CONFIRM is pending (never replaces it)",
                    loaded && raise && refused && same && raised && later, d);
        remove("nv7.bin");
    }

    /* RB11 (review 3) a momentary pulse whose board timer failed: RELAY_PULSE_MARGIN_MS after the pulse should
     * have ended the AURIX sends the OFF itself and keeps it pending until the readback shows the relay off -
     * also across a silent board */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1712u, NULL, &t);
        sim_hal_relay_board(true, 5u, SIM_RB_NO_PULSE_END);
        bool asked = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 1000u) == SB_OK;
        uint32_t t0 = x.t, t_off = 0u;
        while (x.t < t0 + 4000u) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
            if (!t_off && x.t > t0 + 200u && !sim_hal_relay_state(4u)) t_off = x.t - t0;
        }
        /* Modbus: the board's pulse timer is broken - the AURIX's own OFF at the deadline + margin (1.5 s);
         * DroneCAN: the pulse is AURIX-timed anyway (its OFF at 1 s), the AURIX's extra OFF follows at 1.5 s */
        bool t_ok = rb_can() ? (t_off >= 950u && t_off <= 1300u) : (t_off >= 1400u && t_off <= 1900u);
        bool off = t_ok && x.sb.relays.pulse_offs == 1u && !(x.sb.relays.off_pending & rb_bit(4));
        /* the bus cut while the pulse runs: the OFF waits (pending), goes when the board is back */
        bool asked2 = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 1000u) == SB_OK;
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        bool on2 = sim_hal_relay_state(4u);
        sim_hal_relay_board(false, 5u, SIM_RB_NO_PULSE_END);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool held = sim_hal_relay_state(4u) && (x.sb.relays.off_pending & rb_bit(4)) && x.sb.relays.pulse_offs == 2u;
        sim_hal_relay_board(true, 5u, SIM_RB_NO_PULSE_END);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool done = !sim_hal_relay_state(4u) && !(x.sb.relays.off_pending & rb_bit(4));
        sim_hal_relay_board(true, 5u, 0u);
        snprintf(d, sizeof d, "board timer broken: 1 s pulse %s, the AURIX's OFF after %u ms (1.5 s expected), readback off, "
                 "nothing pending %s; bus cut during a pulse: on %s / %s, the OFF pending while silent %s, done when back %s",
                 asked ? "ok" : "NO", (unsigned)t_off, off ? "yes" : "NO", asked2 ? "ok" : "NO", on2 ? "yes" : "NO",
                 held ? "yes" : "NO", done ? "yes" : "NO");
        test_report("RB11 momentary pulse: the AURIX's own OFF after the deadline + margin, pending until the readback shows off",
                    loaded && asked && off && asked2 && on2 && held && done, d);
        remove("nv7.bin");
    }

    /* RB12 (review 6) advisories never set the alarm flag: RELAY BOARD SILENT (and LOW BATTERY) = the bit only -
     * no buzzer, a power raise is still possible; with no board fitted (HAL_RELAY_FITTED 0) a table never gives
     * RELAY BOARD SILENT and relay commands are refused */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1713u, NULL, &t);
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool quiet0 = !x.sb.alarm && !sim_hal_alarm();
        sim_hal_relay_board(false, 5u, 0u);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        uint16_t code = 0u;
        bool silent = (x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT) && !x.sb.alarm && !sim_hal_alarm() &&
                      sb_power_code(&x.sb, 0u, &code) == SB_OK;
        sim_hal_set_supply(true, 11.2f);
        sbsim_run(&x, x.t + 100000u, NULL, NULL);
        /* LOW BATTERY keeps its buzzer (step 4 design, BV2; TO CONFIRM) - only RELAY BOARD SILENT is quiet */
        bool low = (x.sb.alarm_bits2 & SB_ALM2_LOW_BATTERY) && x.sb.alarm && sim_hal_alarm();
        sim_hal_set_supply(true, 12.6f);
        sim_hal_relay_board(true, 5u, 0u);
        /* no board fitted: the stored table (NV) is kept, but no SILENT, nothing switched */
        sim_hal_relay_fitted(0u);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 6000u, NULL, NULL);
        uint32_t w0 = sim_hal_relay_writes(), q0 = sim_hal_relay_requests();
        bool nf = x.sb.relays.have_table && x.sb.relays.channels == 0u && !relays_silent(&x.sb.relays) &&
                  !(x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT) && sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_ERR_PRECOND &&
                  sb_relay_cmd(&x.sb, 1u, RELAY_CMD_ON, 0u) == SB_ERR_PRECOND;
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        nf = nf && sim_hal_relay_writes() == w0 && sim_hal_relay_requests() == q0;
        sim_hal_relay_fitted(16u);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool back = x.sb.relays.channels == 16u && x.sb.relays.board_ok;
        snprintf(d, sizeof d, "quiet at the start %s; board silent: bit set, alarm flag / buzzer off, power raise possible %s; "
                 "LOW BATTERY (not quiet): buzzer as before %s; no board fitted: table kept, no SILENT, commands refused, nothing sent %s; "
                 "fitted again %s", quiet0 ? "yes" : "NO", silent ? "yes" : "NO", low ? "yes" : "NO", nf ? "yes" : "NO",
                 back ? "yes" : "NO");
        test_report("RB12 RELAY BOARD SILENT never sets the alarm flag (no buzzer, raise possible); no board fitted = no SILENT",
                    loaded && quiet0 && silent && low && nf && back, d);
        remove("nv7.bin");
    }

    /* RB13 (review 7) relay channels within the fitted board: an 8-channel board refuses a table with relay 9
     * (BAD) and the driver never queues a coil beyond it; a stored table that no longer fits (smaller board)
     * is not used (NV DATA RESET) */
    {
        static sbsim_t x;
        static relay_table_t t, big;
        rb_table(&t, false, false);
        rb_table(&big, false, false);
        big.r[6].n = 12u;                           /* the bilge pump on channel 12 */
        relay_table_seal(&big);
        bool loaded = rb_setup(&x, 1714u, NULL, &big) && x.sb.relays.hash == relay_table_hash(&big);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);     /* (saved in slot 7) */
        sim_hal_relay_fitted(8u);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool dropped = !x.sb.relays.have_table && (nv_status()->reset_mask & (1u << HAL_NV_SLOT_RELAYS)) &&
                       x.sb.relays.channels == 8u;
        bool refused = sb_relay_table(&x.sb, &big) == SB_ERR_BAD_VALUE;
        bool fits = sb_relay_table(&x.sb, &t) == SB_OK && confirm_press(&x) && x.sb.relays.have_table;
        bool drv = rb_can() ? (!cr_set(9u, true) && !cr_pulse(12u, 500u) && cr_set(8u, false))
                            : (!mb_set_coil(9u, true) && !mb_pulse_coil(12u, 500u) && mb_set_coil(8u, false));
        bool cmd9 = sb_relay_cmd(&x.sb, 9u, RELAY_CMD_ON, 0u) == SB_ERR_BAD_VALUE;
        sim_hal_relay_fitted(16u);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        snprintf(d, sizeof d, "16 channels: table with relay 12 stored %s; board of 8: the stored table not used + NV DATA "
                 "RESET %s, the same table refused %s, a fitting one confirmed %s; driver: coil 9 / 12 refused, 8 ok %s; "
                 "command for relay 9 refused %s", loaded ? "yes" : "NO", dropped ? "yes" : "NO", refused ? "yes" : "NO",
                 fits ? "yes" : "NO", drv ? "yes" : "NO", cmd9 ? "yes" : "NO");
        test_report("RB13 relay channels limited to the fitted board (table, stored table, driver)",
                    loaded && dropped && refused && fits && drv && cmd9, d);
        remove("nv7.bin");
    }

    /* RB14 (review 4) the reset reason (hal/tc_rstcause.h, latched once at boot by the board, cold bits cleared):
     * a warm trigger always wins over a stale PORST - a watchdog (SMU) or software reset never reads as a
     * power-on, so the relay power_up states are never applied after it */
    {
        const uint32_t P = 1u << RST_PORST, SMU = 1u << RST_SMU, SW = 1u << RST_SW;
        const uint32_t CON_SW_APP = 2u << RSTCON_SW, CON_SW_SYS = 1u << RSTCON_SW, CON_SMU_APP = 2u << RSTCON_SMU;
        struct { uint32_t st, con; hal_reset_t want; const char *what; } k[] = {
            { P, 0u, HAL_RESET_POWER_ON, "PORST" },
            { 1u << RST_EVR33, 0u, HAL_RESET_POWER_ON, "EVR33" },
            { P | SMU, 0u, HAL_RESET_SYSTEM, "stale PORST + SMU (watchdog)" },
            { P | SMU, CON_SMU_APP, HAL_RESET_APPLICATION, "stale PORST + SMU as application reset" },
            { P | SW, CON_SW_APP, HAL_RESET_APPLICATION, "stale PORST + software (application)" },
            { SW, CON_SW_SYS, HAL_RESET_SYSTEM, "software (system)" },
            { P | (1u << RST_STM0), 0u, HAL_RESET_SYSTEM, "stale PORST + STM0" },
            { P | (1u << RST_CB3), 0u, HAL_RESET_APPLICATION, "stale PORST + Cerberus application" },
            { 1u << RST_CB1, 0u, HAL_RESET_DEBUG, "debug" },
            { P | (1u << RST_ESR0), 0u, HAL_RESET_SYSTEM, "stale PORST + ESR0" },
            { 0u, 0u, HAL_RESET_OTHER, "nothing" },
        };
        int good = 0, n = (int)(sizeof k / sizeof k[0]);
        char bad[300] = "";
        for (int i = 0; i < n; i++) {
            if (rstcause_classify(k[i].st, k[i].con) == k[i].want) good++;
            else if (strlen(bad) < 200u) { strcat(bad, " "); strcat(bad, k[i].what); }
        }
        /* the consequence: with the stored power_up table, a watchdog reset applies nothing */
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, true);
        bool loaded = rb_setup(&x, 1715u, NULL, &t) && x.sb.relays.power_ups == 1u;
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        rb_power_cycle(&x, rstcause_classify(P | SMU, 0u));
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool kept = x.sb.relays.power_ups == 0u && !sim_hal_relay_state(6u);
        snprintf(d, sizeof d, "%d / %d register patterns right%s%s; NAV LIGHTS power_up after the first boot %s, after a "
                 "watchdog reset (stale PORST + SMU) nothing applied %s", good, n, bad[0] ? ", wrong:" : "", bad,
                 loaded ? "yes" : "NO", kept ? "yes" : "NO");
        test_report("RB14 reset reason: a watchdog / software reset never reads as power-on (no relay power_up after it)",
                    good == n && loaded && kept, d);
        remove("nv7.bin");
        sim_hal_relay_board_reset();
    }
}

void sb_relay_tests(void)
{
    relay_tests();                  /* RB1-RB14 on the Modbus board (or on DroneCAN with "--can") */
    if (!s_can_all) {
        s_rb_can = true;
        relay_tests();              /* ... and again on the DroneCAN board (step 11f) */
        s_rb_can = false;
    }
    relay_bus_tests();              /* RB15-RB22 */
}

/* ------------------------------ engine controls + remote restart (CLAUDE.md N) -- */

/* What the STARTER relay (channel 2 of the sample table) did, measured after every step. */
typedef struct {
    rjsim_t  r;
    uint32_t on_since;          /* the board's STARTER coil on since (0 = off) */
    uint32_t on_max_ms;         /* longest continuous on */
    uint32_t on_bad;            /* steps with the coil on > 350 ms after idle + neutral / e-stop went (stop latency) */
    uint32_t bad_since;
    uint32_t reset_t;           /* when hal_system_reset() was asked (0 = never) */
    uint32_t resets;
} ersim_t;

static int er_gear(const sbsim_t *x)
{
    uint16_t g = sim_hal_gear_pwm();
    return g == x->sb.cfg.gear.pwm_n_us ? 0 : 1;
}

static void er_hook(sbsim_t *x, void *ctx)
{
    ersim_t *e = (ersim_t *)ctx;
    rj_hook(x, &e->r);
    bool on = sim_hal_relay_state(2u);
    if (on && !e->on_since) e->on_since = x->t ? x->t : 1u;
    if (!on) e->on_since = 0u;
    if (on && x->t - e->on_since > e->on_max_ms) e->on_max_ms = x->t - e->on_since;
    bool bad = er_gear(x) != 0 || sim_hal_throttle() > 1.0f || x->sb.estop_state == 2u;
    if (!bad) e->bad_since = 0u;
    else if (!e->bad_since) e->bad_since = x->t ? x->t : 1u;
    if (on && bad && x->t - e->bad_since > 350u) e->on_bad++;
    if (sim_hal_sw_resets() != e->resets) {
        e->resets = sim_hal_sw_resets();
        e->reset_t = x->t;
    }
}

/* A RESTART frame (action) from gateway station st, run 200 ms; the GW_ACK result (0xFF none). */
static uint8_t er_cmd(sbsim_t *x, ersim_t *e, uint8_t st, uint8_t act)
{
    uint8_t buf[8], fr[PROTO_MAX_FRAME];
    static uint16_t seq = 3000u;
    seq++;
    buf[0] = st; buf[1] = 0u; buf[2] = (uint8_t)seq; buf[3] = (uint8_t)(seq >> 8);
    buf[4] = PROTO_T_RESTART; buf[5] = act;
    size_t n = proto_encode(fr, PROTO_T_GW_CMD, 0u, buf, 6u);
    int before = e->r.p.acks;
    sim_hal_gw_rx_push(fr, (uint32_t)n);
    sbsim_run(x, x->t + 200u, er_hook, e);
    return (e->r.p.acks > before && e->r.p.ack_seq == seq && e->r.p.ack_station == st) ? e->r.p.ack_result : 0xFFu;
}

/* START held ms long (CRANK at 5 Hz): how many were accepted */
static int er_hold(sbsim_t *x, ersim_t *e, uint32_t ms)
{
    int ok = 0;
    uint32_t end = x->t + ms;
    while (x->t + 100u < end) ok += er_cmd(x, e, 2u, ENG_ACT_CRANK) == PROTO_R_OK;
    return ok;
}

/* power-up (CREWED or not), station 2 takes HELM, the boat file's relay table confirmed */
static bool er_setup(sbsim_t *x, ersim_t *e, uint32_t seed, bool crewed, const sb_config_t *cfg)
{
    const uint8_t TAKE = 0u;
    static relay_table_t t;
    memset(e, 0, sizeof *e);
    remove("nv7.bin");
    crew_init(x, &e->r, seed, crewed, cfg);
    e->resets = sim_hal_sw_resets();
    bool helm = pi_cmd(x, &e->r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u) == PROTO_R_OK;
    e->r.rj_on = true;
    sbsim_run(x, x->t + 300u, er_hook, e);
    sb_alarm_ack(&x->sb);
    rb_table(&t, false, false);
    if (sb_relay_table(&x->sb, &t) != SB_OK) return false;
    bool cf = crewed ? onboard_confirm(x, &e->r) : shore_confirm(x, &e->r);
    sbsim_run(x, x->t + 500u, er_hook, e);
    return helm && cf && x->sb.relays.have_table && relays_starter(&x->sb.relays) == 2u;
}

static void engine_tests(void)
{
    char d[1400];
    const uint8_t TAKE = 0u;
    printf("\n=== ER engine controls + remote restart (section N) ===\n");

    /* ER1 the conditions, checked every time: not HELM, nothing latched, e-stop, throttle not idle,
     * gear not N (neutral switch fitted + open), FIRST TEST - all refused, the STARTER never pulsed;
     * idle + neutral (servo at N without a switch, the switch closed with one) = cranking allowed */
    {
        static sbsim_t x;
        static ersim_t e;
        bool set = er_setup(&x, &e, 1801u, true, NULL);
        uint32_t p0 = sim_hal_relay_pulses();
        uint8_t nothing = er_cmd(&x, &e, 2u, ENG_ACT_RESTART);
        uint8_t why_nothing = x.sb.eng.last_why;
        uint8_t other = er_cmd(&x, &e, 3u, ENG_ACT_CRANK);                     /* link.c: not the HELM station */
        sb_result_t direct = sb_restart(&x.sb, 3u, ENG_ACT_CRANK);              /* sb_restart checks it too */
        sim_hal_set_estop(true, true);
        sbsim_run(&x, x.t + 200u, er_hook, &e);
        uint8_t es = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        uint8_t why_es = x.sb.eng.last_why;
        sim_hal_set_estop(true, false);
        sbsim_run(&x, x.t + 300u, er_hook, &e);
        sb_alarm_ack(&x.sb);
        joy_set(0.0f, 0.7f, false, false);                                      /* MANUAL: forward, throttle up */
        sbsim_run(&x, x.t + 3000u, er_hook, &e);
        bool driving = sb_mode(&x.sb) == MODE_MANUAL && gear_is(&x.sb.gear, GEAR_F);
        uint8_t thr = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        uint8_t why_thr = x.sb.eng.last_why;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 4000u, er_hook, &e);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 1000u, er_hook, &e);
        bool none = sim_hal_relay_pulses() == p0 && !sim_hal_relay_state(2u);
        /* no neutral switch: the gear servo at N + gear state N -> allowed */
        uint8_t ok1 = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        bool on1 = sim_hal_relay_state(2u);
        er_cmd(&x, &e, 2u, ENG_ACT_RELEASE);
        sbsim_run(&x, x.t + 10500u, er_hook, &e);                               /* cool-down */
        /* a neutral switch fitted and closed -> allowed; stuck open -> refused (gear) */
        x.w.neutral_switch = true;
        x.w.neutral_sw_force = 0;
        sbsim_run(&x, x.t + 500u, er_hook, &e);
        uint8_t ok2 = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        bool on2 = sim_hal_relay_state(2u);
        er_cmd(&x, &e, 2u, ENG_ACT_RELEASE);
        sbsim_run(&x, x.t + 10500u, er_hook, &e);
        x.w.neutral_sw_force = 1;                                               /* stuck open */
        sbsim_run(&x, x.t + 300u, er_hook, &e);
        uint32_t p1 = sim_hal_relay_pulses();
        uint8_t ns = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        uint8_t why_ns = x.sb.eng.last_why;
        bool none2 = sim_hal_relay_pulses() == p1;
        x.w.neutral_switch = false;
        x.w.neutral_sw_force = 0;
        /* FIRST TEST: never (restart and crank) */
        static sbsim_t y;
        static ersim_t f;
        sb_config_t c;
        sb_default_config(&c);
        c.first_test = true;
        memset(&f, 0, sizeof f);
        crew_init(&y, &f.r, 1802u, true, &c);
        uint8_t fh = pi_cmd(&y, &f.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        uint8_t ft1 = er_cmd(&y, &f, 2u, ENG_ACT_RESTART), ft2 = er_cmd(&y, &f, 2u, ENG_ACT_CRANK);
        sim_hal_set_estop(false, false);
        snprintf(d, sizeof d, "set up %s; restart with nothing latched -> %u (why %u); another station -> %u, sb_restart -> %d; "
                 "e-stop -> %u (why %u); driving %s: throttle not idle -> %u (why %u); no STARTER pulse so far %s; no switch, "
                 "servo at N -> %u (STARTER on %s); switch closed -> %u (on %s); switch stuck open -> %u (why %u, no pulse %s); "
                 "FIRST TEST (HELM %u): restart %u, crank %u; STARTER on while not idle / not N / e-stop: %u steps",
                 set ? "yes" : "NO", nothing, why_nothing, other, (int)direct, es, why_es, driving ? "yes" : "NO", thr, why_thr,
                 none ? "yes" : "NO", ok1, on1 ? "yes" : "NO", ok2, on2 ? "yes" : "NO", ns, why_ns, none2 ? "yes" : "NO",
                 fh, ft1, ft2, (unsigned)e.on_bad);
        test_report("ER1 restart / crank conditions every time: HELM, idle, neutral confirmed, e-stop released, not FIRST TEST",
                    set && nothing == PROTO_R_PRECOND && why_nothing == ENG_WHY_NOTHING && other == PROTO_R_NOT_HELM &&
                    direct == SB_ERR_NOT_ALLOWED && es == PROTO_R_REFUSED && why_es == ENG_WHY_ESTOP && driving &&
                    thr == PROTO_R_PRECOND && (why_thr == ENG_WHY_THROTTLE || why_thr == ENG_WHY_GEAR) && none &&
                    ok1 == PROTO_R_OK && on1 && ok2 == PROTO_R_OK && on2 && ns == PROTO_R_PRECOND && why_ns == ENG_WHY_GEAR &&
                    none2 && fh == PROTO_R_OK && ft1 == PROTO_R_WRONG_MODE && ft2 == PROTO_R_WRONG_MODE && e.on_bad == 0u, d);
    }

    /* ER2 hold-to-crank: the STARTER on while START is held (board-timed 300 ms pulses refreshed at 5 Hz),
     * off within 0.3 s of the release, never longer than 5 s per hold, then 10 s cool-down; RELEASE = off at once */
    {
        static sbsim_t x;
        static ersim_t e;
        bool set = er_setup(&x, &e, 1803u, true, NULL);
        uint8_t ign[4] = { 1u, RELAY_CMD_ON, 0u, 0u };
        uint8_t ig = pi_cmd(&x, &e.r.p, 2u, false, PROTO_T_RELAY, ign, 4u);   /* IGNITION first (the station's order) */
        sbsim_run(&x, x.t + 300u, er_hook, &e);
        bool ign_on = sim_hal_relay_state(1u);
        e.on_max_ms = 0u;
        int ok = er_hold(&x, &e, 2000u);
        bool held_on = sim_hal_relay_state(2u) && x.sb.eng.crank == ENG_CRANK_ON;
        uint32_t t_rel = x.t;
        while (sim_hal_relay_state(2u) && x.t - t_rel < 2000u) sbsim_run(&x, x.t + 10u, er_hook, &e);
        uint32_t off_ms = x.t - t_rel;
        bool released = x.sb.eng.crank == ENG_CRANK_COOL && x.sb.eng.stop_why == ENG_WHY_RELEASED &&
                        x.sb.eng.hold_ms >= 1600u && x.sb.eng.hold_ms <= 2300u;
        uint32_t hold1 = e.on_max_ms, holdr = x.sb.eng.hold_ms;
        uint8_t cool = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        uint8_t why_cool = x.sb.eng.last_why;
        sbsim_run(&x, x.t + 10200u, er_hook, &e);
        bool cool_over = x.sb.eng.crank == ENG_CRANK_OFF;
        e.on_max_ms = 0u;
        int ok7 = er_hold(&x, &e, 7000u);                                       /* held 7 s: the limit is 5 s */
        uint32_t hold7 = x.sb.eng.hold_ms;
        bool limited = x.sb.eng.stop_why == ENG_WHY_MAX_TIME && hold7 >= 4800u && hold7 <= 5200u &&
                       !sim_hal_relay_state(2u) && e.on_max_ms <= 5300u && e.on_max_ms >= 4500u;
        sbsim_run(&x, x.t + 10500u, er_hook, &e);
        er_hold(&x, &e, 1000u);
        bool on_again = sim_hal_relay_state(2u);
        uint8_t rel = er_cmd(&x, &e, 2u, ENG_ACT_RELEASE);
        bool off_at_once = !sim_hal_relay_state(2u) && x.sb.eng.stop_why == ENG_WHY_RELEASED;
        snprintf(d, sizeof d, "set up %s; IGNITION %u (on %s); held 2 s: %d CRANK accepted, STARTER on %s; released: off after "
                 "%u ms, cool-down + why RELEASED + hold %u ms %s (on %u ms); CRANK in the cool-down -> %u (why %u); after 10 s "
                 "ready %s; held 7 s: %d accepted, stopped at %u ms (STARTER on %u ms, then cool-down) %s; RELEASE %u -> "
                 "off at once %s (was on %s); STARTER on while not idle / not N / e-stop: %u steps",
                 set ? "yes" : "NO", ig, ign_on ? "yes" : "NO", ok, held_on ? "yes" : "NO", (unsigned)off_ms,
                 (unsigned)holdr, released ? "yes" : "NO", (unsigned)hold1, cool, why_cool, cool_over ? "yes" : "NO",
                 ok7, (unsigned)hold7, (unsigned)e.on_max_ms, limited ? "yes" : "NO", rel, off_at_once ? "yes" : "NO",
                 on_again ? "yes" : "NO", (unsigned)e.on_bad);
        test_report("ER2 hold-to-crank: STARTER only while held, off <= 0.3 s after release, <= 5 s per hold, 10 s cool-down",
                    set && ig == PROTO_R_OK && ign_on && ok >= 9 && held_on && off_ms <= 450u && released && hold1 >= 1600u &&
                    cool == PROTO_R_REFUSED && why_cool == ENG_WHY_COOLING && cool_over && ok7 >= 23 && ok7 <= 26 && limited &&
                    on_again && rel == PROTO_R_OK && off_at_once && e.on_bad == 0u, d);
    }

    /* ER3 the cranking stops at once on: HELM station lost (link), e-stop, neutral switch opening, the STANDBY
     * button, a joystick takeover (gear request / throttle) - every time before the gear can leave N */
    {
        char t[900] = "";
        int good = 0;
        for (int k = 0; k < 5; k++) {
            static sbsim_t x;
            static ersim_t e;
            bool set = er_setup(&x, &e, 1804u + (uint32_t)k, true, NULL);
            if (k == 2) { x.w.neutral_switch = true; x.w.neutral_sw_force = 0; sbsim_run(&x, x.t + 500u, er_hook, &e); }
            er_hold(&x, &e, 1000u);
            bool on = sim_hal_relay_state(2u) && x.sb.eng.crank == ENG_CRANK_ON;
            uint8_t want;
            switch (k) {
            case 0: pi_station_left(2u); want = ENG_WHY_LINK; break;
            case 1: sim_hal_set_estop(true, true); want = ENG_WHY_ESTOP; break;
            case 2: x.w.neutral_sw_force = 1; want = ENG_WHY_GEAR; break;
            case 3: sim_hal_press(HAL_BTN_STANDBY); want = ENG_WHY_STANDBY_BTN; break;
            default: joy_set(0.0f, 0.8f, false, false); want = ENG_WHY_GEAR; break;
            }
            uint32_t t0 = x.t;
            /* the station keeps sending CRANK (START still held): only the condition can stop it */
            while (x.sb.eng.crank == ENG_CRANK_ON && x.t - t0 < 1000u) {
                if (k != 0) {
                    uint8_t buf[6] = { 2u, 0u, 0u, 0u, PROTO_T_RESTART, ENG_ACT_CRANK }, fr[PROTO_MAX_FRAME];
                    size_t n = proto_encode(fr, PROTO_T_GW_CMD, 0u, buf, 6u);
                    sim_hal_gw_rx_push(fr, (uint32_t)n);
                }
                sbsim_run(&x, x.t + AP_PERIOD_MS, er_hook, &e);
            }
            uint32_t stop_ms = x.t - t0;
            bool gear_n_at_stop = er_gear(&x) == 0;
            uint32_t t1 = x.t;
            while (sim_hal_relay_state(2u) && x.t - t1 < 1000u) sbsim_run(&x, x.t + 10u, er_hook, &e);
            uint32_t off_ms = x.t - t1;
            bool why_ok = x.sb.eng.stop_why == want || (k == 4 && x.sb.eng.stop_why == ENG_WHY_THROTTLE);
            bool g = set && on && stop_ms <= 4u * AP_PERIOD_MS && gear_n_at_stop && off_ms <= 350u && why_ok &&
                     e.on_bad == 0u;
            good += g;
            snprintf(t + strlen(t), sizeof t - strlen(t), "%s%s: stopped after %u ms (why %u), STARTER off %u ms later, gear N %s%s",
                     k ? "; " : "", k == 0 ? "HELM station lost" : k == 1 ? "e-stop" : k == 2 ? "neutral switch opens" :
                     k == 3 ? "STANDBY button" : "joystick takeover", (unsigned)stop_ms, x.sb.eng.stop_why, (unsigned)off_ms,
                     gear_n_at_stop ? "yes" : "NO", g ? "" : " FAIL");
            sim_hal_set_estop(false, false);
            x.w.neutral_switch = false;
            x.w.neutral_sw_force = 0;
            joy_set(0.0f, 0.0f, false, false);
        }
        test_report("ER3 cranking stops at once: link loss, e-stop, gear not N, STANDBY button, joystick takeover", good == 5, t);
    }

    /* ER4 after an ENGINE KILL (UNCREWED lost link): RESTART from the HELM station clears the latch only with
     * idle + neutral and the e-stop released; nothing drives by itself; START works again; the next loss kills again */
    {
        static sbsim_t x;
        static llsim_t l;
        static ersim_t e;
        static relay_table_t tb;
        remove("nv7.bin");
        bool drv = ll_drive(&x, &l, 1810u, false, 50u);
        ll_cut(&x, &l);
        sbsim_run(&x, x.t + 4000u, ll_hook, &l);
        bool killed = x.sb.ll.state == SB_LL_KILLED && sim_hal_engine_killed();
        memset(&e, 0, sizeof e);
        e.r = l.r;
        e.resets = sim_hal_sw_resets();
        uint8_t helm = pi_cmd(&x, &e.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        e.r.rj_on = true;
        sbsim_run(&x, x.t + 300u, er_hook, &e);
        rb_table(&tb, false, false);
        rj_lever(&x, &e.r, 0.5f, 1.0f);                                        /* the captain's lever back to neutral */
        bool tab = sb_relay_table(&x.sb, &tb) == SB_OK && shore_confirm(&x, &e.r) && relays_starter(&x.sb.relays) == 2u;
        uint8_t ck = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        uint8_t why_ck = x.sb.eng.last_why;
        sim_hal_set_estop(true, true);
        sbsim_run(&x, x.t + 200u, er_hook, &e);
        uint8_t es = er_cmd(&x, &e, 2u, ENG_ACT_RESTART);
        bool still = sim_hal_engine_killed() && x.sb.ll.state == SB_LL_KILLED;
        sim_hal_set_estop(true, false);
        sbsim_run(&x, x.t + 300u, er_hook, &e);
        uint8_t oth = er_cmd(&x, &e, 3u, ENG_ACT_RESTART);
        bool still2 = sim_hal_engine_killed() && x.sb.ll.state == SB_LL_KILLED;
        uint8_t rs = er_cmd(&x, &e, 2u, ENG_ACT_RESTART);
        bool cleared = !sim_hal_engine_killed() && x.sb.ll.state == SB_LL_OK && !(x.sb.alarm_bits & SB_ALM_ENGINE_KILLED) &&
                       x.sb.eng.kill_clears == 1u && sb_mode(&x.sb) == MODE_STANDBY;
        float thr_max = 0.0f;
        for (int i = 0; i < 100; i++) { sbsim_run(&x, x.t + AP_PERIOD_MS, er_hook, &e); thr_max = fmaxf(thr_max, sim_hal_throttle()); }
        bool calm = sb_mode(&x.sb) == MODE_STANDBY && thr_max <= 0.6f && !sim_hal_engine_killed();
        er_hold(&x, &e, 1000u);
        bool cranks = sim_hal_relay_state(2u);
        er_cmd(&x, &e, 2u, ENG_ACT_RELEASE);
        /* the lost-link rule still works: the next loss kills again */
        l.r = e.r;
        l.killed = false;
        ll_cut(&x, &l);
        sbsim_run(&x, x.t + 4000u, ll_hook, &l);
        bool again = x.sb.ll.state == SB_LL_KILLED && sim_hal_engine_killed();
        sim_hal_set_estop(false, false);
        snprintf(d, sizeof d, "driving %s, killed %s; HELM back %u, table %s; START while killed -> %u (why %u); RESTART with the "
                 "e-stop pressed -> %u (still killed %s); from another station -> %u (still killed %s); RESTART -> %u: kill off, "
                 "alarm gone, STANDBY %s; 5 s later nothing started by itself (throttle max %.1f %%) %s; START works %s; next "
                 "link loss kills again %s",
                 drv ? "yes" : "NO", killed ? "yes" : "NO", helm, tab ? "yes" : "NO", ck, why_ck, es, still ? "yes" : "NO", oth,
                 still2 ? "yes" : "NO", rs, cleared ? "yes" : "NO", thr_max, calm ? "yes" : "NO", cranks ? "yes" : "NO",
                 again ? "yes" : "NO");
        test_report("ER4 after an ENGINE KILL: the HELM station's RESTART (idle + neutral, e-stop released) clears the latch",
                    drv && killed && helm == PROTO_R_OK && tab && ck == PROTO_R_PRECOND && why_ck == ENG_WHY_KILLED &&
                    es == PROTO_R_REFUSED && still && oth == PROTO_R_NOT_HELM && still2 && rs == PROTO_R_OK && cleared && calm &&
                    cranks && again, d);
    }

    /* ER5 after a monitor trip: RESTART = a controlled software reset, the log event RESTART first (it reaches the
     * gateway's copy before the reset); everything starts again (startup power limits, interlock, no power_up), the
     * trip re-arms; conditions failing before the reset (e-stop) cancel it */
    {
        static sbsim_t x;
        static ersim_t e;
        bool set = er_setup(&x, &e, 1811u, true, NULL);
        sbsim_log_start(&x, "sd_er5", 5u);
        sbsim_run(&x, x.t + 1000u, er_hook, &e);
        g_mon.test_trip = MON_TEST_TRIP;
        sbsim_run(&x, x.t + 600u, er_hook, &e);
        bool tripped = x.sb.mon_tripped && hal_outputs_forced() && sb_mode(&x.sb) == MODE_STANDBY;
        uint8_t ck = er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        uint8_t why_ck = x.sb.eng.last_why;
        uint32_t ev0 = e.r.p.ev_restart;
        uint32_t r0 = sim_hal_sw_resets();
        uint8_t rs = er_cmd(&x, &e, 2u, ENG_ACT_RESTART);
        bool pending = x.sb.eng.reset_pending && sim_hal_sw_resets() == r0;
        sbsim_run(&x, x.t + 1500u, er_hook, &e);
        bool reset = sim_hal_sw_resets() == r0 + 1u && e.reset_t != 0u;
        bool ev_first = e.r.p.ev_restart > ev0 && e.r.p.ev_reset_t != 0u && e.r.p.ev_reset_t <= e.reset_t;
        sbsim_run(&x, x.t + 500u, er_hook, &e);
        bool fresh = !hal_outputs_forced() && !x.sb.mon_tripped && hal_reset_reason(NULL) == HAL_RESET_APPLICATION &&
                     x.sb.plim.fwd_pct <= 30.01f && x.sb.plim.rev_pct <= 5.01f && x.sb.relays.power_ups == 0u &&
                     x.sb.eng.restarts == 0u;
        sbsim_run(&x, x.t + 8000u, er_hook, &e);
        bool armed = x.sb.armed;
        g_mon.test_trip = MON_TEST_TRIP;                                        /* the monitor works again */
        sbsim_run(&x, x.t + 600u, er_hook, &e);
        bool rearmed = hal_outputs_forced() && x.sb.mon_tripped;
        x.log_on = false;
        sim_hal_sd_power_cut();                         /* the card's files closed (the next tests open their own) */
        /* cancelled: the e-stop pressed between the RESTART and the reset */
        static sbsim_t y;
        static ersim_t f;
        bool set2 = er_setup(&y, &f, 1812u, true, NULL);
        g_mon.test_trip = MON_TEST_TRIP;
        sbsim_run(&y, y.t + 600u, er_hook, &f);
        uint32_t r1 = sim_hal_sw_resets();
        uint8_t rs2 = er_cmd(&y, &f, 2u, ENG_ACT_RESTART);
        sim_hal_set_estop(true, true);
        sbsim_run(&y, y.t + 2000u, er_hook, &f);
        bool cancelled = sim_hal_sw_resets() == r1 && !y.sb.eng.reset_pending && y.sb.eng.last_action == ENG_LOG_RESET_CANCEL &&
                         hal_outputs_forced();
        sim_hal_set_estop(false, false);
        snprintf(d, sizeof d, "set up %s; tripped %s; START while tripped -> %u (why %u); RESTART -> %u, reset pending (not yet) %s; "
                 "software reset %s; log event RESTART in the gateway's copy before the reset %s (%u events); after the reboot: "
                 "latch gone, reason APPLICATION, power limit 30 / 5, no relay power_up %s; armed again %s; a new trip trips %s; "
                 "second boat %s: RESTART %u then e-stop -> reset cancelled, latch kept %s",
                 set ? "yes" : "NO", tripped ? "yes" : "NO", ck, why_ck, rs, pending ? "yes" : "NO", reset ? "yes" : "NO",
                 ev_first ? "yes" : "NO", (unsigned)(e.r.p.ev_restart - ev0), fresh ? "yes" : "NO", armed ? "yes" : "NO",
                 rearmed ? "yes" : "NO", set2 ? "yes" : "NO", rs2, cancelled ? "yes" : "NO");
        test_report("ER5 after a monitor trip: RESTART = software reset (event first), all starts again, the trip re-arms",
                    set && tripped && ck == PROTO_R_PRECOND && why_ck == ENG_WHY_TRIPPED && rs == PROTO_R_OK && pending && reset &&
                    ev_first && fresh && armed && rearmed && set2 && rs2 == PROTO_R_OK && cancelled, d);
        memset((void *)&g_mon, 0, sizeof g_mon);
    }

    /* ER6 no attempt limit, no code, no CONFIRM: three kills, three restarts in a row; the log header lists event 27
     * and is complete (not cut); a CREWED boat never has a kill to clear */
    {
        static sbsim_t x;
        static llsim_t l;
        static ersim_t e;
        int ok = 0;
        bool drv = ll_drive(&x, &l, 1813u, false, 50u);
        memset(&e, 0, sizeof e);
        e.resets = sim_hal_sw_resets();
        for (int i = 0; i < 3; i++) {
            l.killed = false;
            ll_cut(&x, &l);
            sbsim_run(&x, x.t + 4000u, ll_hook, &l);
            e.r = l.r;
            pi_cmd(&x, &e.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
            e.r.rj_on = true;
            sbsim_run(&x, x.t + 300u, er_hook, &e);
            uint32_t c0 = x.sb.confirm.accepted;
            bool k = x.sb.ll.state == SB_LL_KILLED;
            ok += k && er_cmd(&x, &e, 2u, ENG_ACT_RESTART) == PROTO_R_OK && !sim_hal_engine_killed() &&
                  x.sb.confirm.accepted == c0 && x.sb.confirm.kind == CONF_NONE;
            l.r = e.r;
        }
        static char hdr[LOGR_MAX], big[3u * LOGR_MAX];
        uint16_t hl = logrec_header_text(hdr, sizeof hdr, 1u, 1u);
        uint16_t full = logrec_header_text(big, sizeof big, 1u, 1u);
        bool listed = strstr(hdr, "27:RESTART:") != NULL;
        bool whole = hl < sizeof hdr - 1u && hl > 0u && hdr[hl - 1u] == '\n' && strstr(hdr, "\nwhy=") != NULL;
        static sbsim_t y;
        static ersim_t f;
        bool set = er_setup(&y, &f, 1814u, true, NULL);
        uint8_t cr = er_cmd(&y, &f, 2u, ENG_ACT_RESTART);
        snprintf(d, sizeof d, "driving %s; 3 kills -> 3 RESTARTs accepted at once (no code, no CONFIRM): %d; log header %u of %u "
                 "bytes (whole text %u), event 27 listed %s, complete %s; CREWED boat %s: RESTART -> %u (nothing latched)",
                 drv ? "yes" : "NO", ok, (unsigned)hl, (unsigned)sizeof hdr, (unsigned)full, listed ? "yes" : "NO", whole ? "yes" : "NO",
                 set ? "yes" : "NO", cr);
        test_report("ER6 no attempt limit / code / CONFIRM; log event 27 in the header (complete); CREWED: no kill",
                    drv && ok == 3 && listed && whole && set && cr == PROTO_R_PRECOND, d);
    }

    /* ER7 (step 11b) the 5 s crank limit is never passed: every board pulse ends by hold start + 5 s (a pulse
     * shorter than the hold's rest is cut down to whole 0.1 s units, the board rounds to the nearest); a CRANK
     * with less than 100 ms of the hold left ends the hold (MAX_TIME) instead of a 100 ms pulse */
    {
        static sbsim_t x;
        static ersim_t e;
        bool set = er_setup(&x, &e, 1815u, true, NULL);
        const relay_def_t *sd = relay_table_find(&x.sb.relays.table, 2u);
        uint8_t idx = sd ? (uint8_t)(sd - x.sb.relays.table.r) : 0u;
        static const uint32_t at[] = { 0u, 200u, 400u, 600u, 800u, 1000u, 1200u, 1400u, 1600u, 1800u, 2000u, 2200u,
                                       2400u, 2600u, 2800u, 3000u, 3200u, 3400u, 3600u, 3800u, 4000u, 4200u, 4400u,
                                       4600u, 4750u, 4950u };
        uint32_t t0 = 0u, end_max = 0u, p_late = 0u, len_4750 = 0u;
        sb_result_t r_last = SB_OK;
        bool ok_all = true;
        for (unsigned k = 0; k < sizeof at / sizeof at[0]; k++) {
            while (k > 0u && x.sb.now_ms - t0 < at[k]) sbsim_run(&x, x.t, er_hook, &e);
            uint32_t p0 = x.sb.eng.pulses;
            sb_result_t r = sb_restart(&x.sb, 2u, ENG_ACT_CRANK);
            if (k == 0u) t0 = x.sb.eng.hold_t0_ms;
            if (at[k] == 4950u) { r_last = r; p_late = x.sb.eng.pulses - p0; continue; }
            ok_all = ok_all && r == SB_OK;
            uint32_t end = x.sb.relays.until_ms[idx] - RELAY_PULSE_MARGIN_MS - t0;
            if (at[k] == 4750u) len_4750 = end - at[k];
            if (end > end_max) end_max = end;
        }
        bool stopped = x.sb.eng.crank == ENG_CRANK_COOL && x.sb.eng.stop_why == ENG_WHY_MAX_TIME &&
                       x.sb.eng.hold_ms <= ENG_CRANK_MAX_MS;
        sbsim_run(&x, x.t + 400u, er_hook, &e);
        bool off = !sim_hal_relay_state(2u);
        snprintf(d, sizeof d, "set up %s; 25 CRANKs accepted %s; latest pulse end %u ms after the hold start (limit %u); "
                 "the CRANK at 4.75 s -> a %u ms pulse (250 ms left, whole units); the CRANK at 4.95 s (50 ms left) -> %d, "
                 "%u new pulses, hold ended MAX_TIME at %u ms %s; STARTER off %s",
                 set ? "yes" : "NO", ok_all ? "yes" : "NO", (unsigned)end_max, (unsigned)ENG_CRANK_MAX_MS,
                 (unsigned)len_4750, (int)r_last, (unsigned)p_late, (unsigned)x.sb.eng.hold_ms, stopped ? "yes" : "NO",
                 off ? "yes" : "NO");
        test_report("ER7 the crank never passes 5 s: pulses cut to the hold's rest (whole 0.1 s), < 100 ms left = the hold ends",
                    set && ok_all && end_max <= ENG_CRANK_MAX_MS && len_4750 == 200u && r_last != SB_OK && p_late == 0u &&
                    stopped && off, d);
    }

    /* ER8 (step 11b) the ENGINE KILL latch survives a watchdog reset and a software reset (still killed, alarm,
     * STANDBY only); it goes only with a real power-on reset or the HELM station's RESTART (after which a reset
     * keeps it off); a RESTART of a killed AND tripped boat clears the kill before its own software reset */
    {
        static sbsim_t x;
        static llsim_t l;
        static ersim_t e;
        remove("nv7.bin");
        bool drv = ll_drive(&x, &l, 1816u, false, 50u);
        ll_cut(&x, &l);
        sbsim_run(&x, x.t + 4000u, ll_hook, &l);
        bool killed = x.sb.ll.state == SB_LL_KILLED && sim_hal_engine_killed() && !x.sb.ll.kept;
        /* a watchdog reset (core 0 frozen) */
        uint32_t r0 = x.resets;
        x.frozen = true;
        sbsim_run(&x, x.t + 600u, ll_hook, &l);
        bool wdt = x.resets == r0 + 1u && hal_reset_reason(NULL) == HAL_RESET_SYSTEM;
        bool kept_wdt = x.sb.ll.state == SB_LL_KILLED && x.sb.ll.kept && sim_hal_engine_killed() &&
                        (x.sb.alarm_bits & SB_ALM_ENGINE_KILLED) && sb_mode(&x.sb) == MODE_STANDBY;
        /* a software reset */
        sim_hal_chip_reset(HAL_RESET_APPLICATION);
        x.boot_until = x.t + 200u;
        sbsim_run(&x, x.t + 600u, ll_hook, &l);
        bool kept_sw = x.sb.ll.state == SB_LL_KILLED && sim_hal_engine_killed() && hal_reset_reason(NULL) == HAL_RESET_APPLICATION;
        /* the HELM station's RESTART clears it; a watchdog reset afterwards keeps it OFF */
        memset(&e, 0, sizeof e);
        e.r = l.r;
        e.resets = sim_hal_sw_resets();
        uint8_t helm = pi_cmd(&x, &e.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        e.r.rj_on = true;
        /* (DroneCAN servos: after the reset the throttle position is unknown until the node is seen and the
         * worst-case travel time has passed - only then "idle" for the restart) */
        sbsim_run(&x, x.t + (sim_hal_servos_can() ? 8500u : 300u), er_hook, &e);
        bool stays = sb_mode(&x.sb) == MODE_STANDBY && x.sb.ll.state == SB_LL_KILLED;
        uint8_t rs = er_cmd(&x, &e, 2u, ENG_ACT_RESTART);
        bool cleared = !sim_hal_engine_killed() && x.sb.ll.state == SB_LL_OK && !x.sb.ll.kept;
        r0 = x.resets;
        x.frozen = true;
        sbsim_run(&x, x.t + 600u, er_hook, &e);
        bool off_after = x.resets == r0 + 1u && x.sb.ll.state == SB_LL_OK && !sim_hal_engine_killed();
        /* killed again, then a real power-on: gone */
        static sbsim_t y;
        static llsim_t m;
        bool drv2 = ll_drive(&y, &m, 1817u, false, 50u);
        ll_cut(&y, &m);
        sbsim_run(&y, y.t + 4000u, ll_hook, &m);
        bool k2 = y.sb.ll.state == SB_LL_KILLED;
        sim_hal_chip_reset(HAL_RESET_POWER_ON);
        y.boot_until = y.t + 200u;
        sbsim_run(&y, y.t + 600u, ll_hook, &m);
        bool gone = y.sb.ll.state == SB_LL_OK && !sim_hal_engine_killed() && !(y.sb.alarm_bits & SB_ALM_ENGINE_KILLED);
        /* killed AND tripped: one RESTART clears the kill, then the software reset (the trip goes); after it not killed */
        static sbsim_t z;
        static llsim_t n;
        static ersim_t f;
        bool drv3 = ll_drive(&z, &n, 1818u, false, 50u);
        ll_cut(&z, &n);
        sbsim_run(&z, z.t + 4000u, ll_hook, &n);
        bool k3 = z.sb.ll.state == SB_LL_KILLED;
        g_mon.test_trip = MON_TEST_TRIP;
        sbsim_run(&z, z.t + 600u, ll_hook, &n);
        bool tr3 = z.sb.mon_tripped && hal_outputs_forced();
        memset(&f, 0, sizeof f);
        f.r = n.r;
        f.resets = sim_hal_sw_resets();
        pi_cmd(&z, &f.r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        f.r.rj_on = true;
        sbsim_run(&z, z.t + 300u, er_hook, &f);
        uint32_t sw0 = sim_hal_sw_resets();
        uint8_t rs3 = er_cmd(&z, &f, 2u, ENG_ACT_RESTART);
        bool kc = z.sb.eng.kill_clears == 1u && !sim_hal_engine_killed();
        sbsim_run(&z, z.t + 2000u, er_hook, &f);
        bool both = sim_hal_sw_resets() == sw0 + 1u && !hal_outputs_forced() && z.sb.ll.state == SB_LL_OK &&
                    !sim_hal_engine_killed();
        memset((void *)&g_mon, 0, sizeof g_mon);
        snprintf(d, sizeof d, "driving %s, killed %s; watchdog reset %s: still killed (kept, alarm, STANDBY) %s; software reset: "
                 "still killed %s; HELM %u (still STANDBY + killed %s), RESTART %u -> cleared %s, a watchdog reset then keeps it "
                 "off %s; second boat (driving %s, killed %s): power-on reset -> gone %s; third boat (driving %s) killed %s + "
                 "tripped %s: RESTART %u clears the kill %s, then the software reset: no trip, not killed %s",
                 drv ? "yes" : "NO", killed ? "yes" : "NO", wdt ? "yes" : "NO", kept_wdt ? "yes" : "NO", kept_sw ? "yes" : "NO",
                 helm, stays ? "yes" : "NO", rs, cleared ? "yes" : "NO", off_after ? "yes" : "NO", drv2 ? "yes" : "NO",
                 k2 ? "yes" : "NO", gone ? "yes" : "NO", drv3 ? "yes" : "NO", k3 ? "yes" : "NO", tr3 ? "yes" : "NO", rs3,
                 kc ? "yes" : "NO", both ? "yes" : "NO");
        test_report("ER8 the ENGINE KILL latch survives a watchdog / software reset; only a power-on or the HELM RESTART clears it",
                    drv && killed && wdt && kept_wdt && kept_sw && stays && helm == PROTO_R_OK && rs == PROTO_R_OK && cleared &&
                    off_after && drv2 && k2 && gone && drv3 && k3 && tr3 && rs3 == PROTO_R_OK && kc && both, d);
    }
    remove("nv7.bin");
    sim_hal_relay_board_reset();
}

/* ------------------------------------------- DP thrusters (CLAUDE.md P, step 9) -- */

/* What the thruster outputs did, measured after every control step. */
typedef struct {
    pcsim_t *pc;                /* the USB laptop (NULL = no link) */
    uint32_t steps;
    float    max_off;           /* largest |output| while DP is not active */
    float    max_rise;          /* fastest growth of |output|, %/s */
    float    max_over;          /* largest |output| - cap */
    float    prev_b, prev_s;
    uint32_t hot_on;            /* steps with a HOT thruster's output != 0 */
    bool     clear_dp;          /* "core 0 forgets dp_active" right after every step (monitor test) */
} dpw_t;

static void dp_hook(sbsim_t *x, void *ctx)
{
    dpw_t *w = (dpw_t *)ctx;
    if (w->pc) pc_hook(x, w->pc);
    float b, s;
    sim_hal_thrusters(&b, &s);
    const thruster_t *t = &x->sb.thrust;
    w->steps++;
    if (!t->active) w->max_off = fmaxf(w->max_off, fmaxf(fabsf(b), fabsf(s)));
    float rb = (fabsf(b) - fabsf(w->prev_b)) / 0.05f, rs = (fabsf(s) - fabsf(w->prev_s)) / 0.05f;
    w->max_rise = fmaxf(w->max_rise, fmaxf(rb, rs));
    w->max_over = fmaxf(w->max_over, fmaxf(fabsf(b) - thruster_cap(t, THR_BOW), fabsf(s) - thruster_cap(t, THR_STERN)));
    if ((t->hot[THR_BOW] && b != 0.0f) || (t->hot[THR_STERN] && s != 0.0f)) w->hot_on++;
    w->prev_b = b;
    w->prev_s = s;
    if (w->clear_dp) g_mon.dp_active = 0u;
}

/* PROTO_T_THRUSTER_CFG body */
static void dp_body(uint8_t *b, uint8_t bmax, uint16_t brun, uint16_t brest, uint8_t smax, uint16_t srun,
                    uint16_t srest, uint8_t jog10, uint8_t spot)
{
    b[0] = bmax; b[1] = (uint8_t)brun; b[2] = (uint8_t)(brun >> 8); b[3] = (uint8_t)brest; b[4] = (uint8_t)(brest >> 8);
    b[5] = smax; b[6] = (uint8_t)srun; b[7] = (uint8_t)(srun >> 8); b[8] = (uint8_t)srest; b[9] = (uint8_t)(srest >> 8);
    b[10] = jog10; b[11] = spot;
}

/* an armed boat in STANDBY, the USB laptop connected (heartbeat), optionally the settings sent */
static bool dp_setup(pcsim_t *p, uint32_t seed, bool settings, uint8_t bmax, uint16_t brun, uint16_t brest,
                     uint8_t smax, uint16_t srun, uint16_t srest)
{
    pcsim_init(p, seed);
    p->heartbeat = true;
    pc_run(p, 2500u);
    sb_alarm_ack(&p->x.sb);
    if (!settings) return p->x.sb.armed;
    uint8_t b[THR_FRAME_BYTES];
    dp_body(b, bmax, brun, brest, smax, srun, srest, 15u, 15u);
    return p->x.sb.armed && pc_cmd(p, PROTO_T_THRUSTER_CFG, b, sizeof b) == PROTO_R_OK;
}

/* one monitor case on its own shared block: true = tripped (reason in *why) */
static bool dp_mon_case(uint8_t dp, float bow, float stern, float cap_b, float cap_s, mon_reason_t *why)
{
    static mon_shared_t sh;
    mon_t m;
    memset((void *)&sh, 0, sizeof sh);
    monitor_init(&m, 0u);
    sh.running = MON_RUNNING;
    sh.fwd_limit = 30.0f;
    sh.rev_limit = 5.0f;
    sh.idle_max_pct = 1.0f;
    sh.gear_pwm_us = 1500u;
    sh.gear_n_us = 1500u;
    sh.thr_idle_us = 1000u;
    sh.dp_active = dp;
    sh.thr_bow_pct = bow;
    sh.thr_stern_pct = stern;
    sh.thr_bow_cap = cap_b;
    sh.thr_stern_cap = cap_s;
    for (uint32_t t = 0u; t <= 300u; t++) {
        if (t % 50u == 0u) sh.heartbeat++;
        (void)monitor_step(&m, &sh, t);
    }
    *why = m.reason;
    sim_hal_unforce();                      /* the case tripped the sim's latch: undo it for the next case */
    return m.tripped;
}

static void dp_tests(void)
{
    char d[1400];
    printf("\n=== DP thrusters part 1 (section P, step 9): app/thruster.c, CPU1, safe outputs ===\n");

    /* DP1 settings frame (HELM, STANDBY only, checked values, RAM only, hash in TELEM2 N); thruster.c the
     * only writer - every step, 0 outside DP; clamp to max_pct, rate limit, a stop is 0 at once */
    {
        static pcsim_t p;
        static dpw_t w;
        bool armed = dp_setup(&p, 1901u, false, 0u, 0u, 0u, 0u, 0u, 0u);
        sbsim_t *x = &p.x;
        memset(&w, 0, sizeof w);
        w.pc = &p;
        /* no settings = no thruster fitted: DP asking for full thrust gets nothing */
        bool none0 = !x->sb.thrust.have_cfg && p.t2_len >= T2_N && (p.t2[p.t2_len - T2_N] & 1u) == 0u;
        sb_test_thrust(&x->sb, true, 100.0f, 100.0f);
        sbsim_run(x, x->t + 1000u, dp_hook, &w);
        float b, s;
        sim_hal_thrusters(&b, &s);
        bool unfitted = b == 0.0f && s == 0.0f && x->sb.thrust.active;
        float ub = b, us = s;
        sb_test_thrust(&x->sb, false, 0.0f, 0.0f);
        /* refused frames */
        uint8_t body[THR_FRAME_BYTES];
        dp_body(body, 40u, 10u, 20u, 80u, 60u, 60u, 15u, 15u);
        uint8_t r_len = pc_cmd(&p, PROTO_T_THRUSTER_CFG, body, 11u);
        uint8_t bad[THR_FRAME_BYTES];
        int n_bad = 0;
        dp_body(bad, 101u, 10u, 20u, 80u, 60u, 60u, 15u, 15u); n_bad += pc_cmd(&p, PROTO_T_THRUSTER_CFG, bad, 12u) == PROTO_R_BAD_VALUE;
        dp_body(bad, 40u, 0u, 20u, 80u, 60u, 60u, 15u, 15u);   n_bad += pc_cmd(&p, PROTO_T_THRUSTER_CFG, bad, 12u) == PROTO_R_BAD_VALUE;
        dp_body(bad, 40u, 10u, 3601u, 80u, 60u, 60u, 15u, 15u); n_bad += pc_cmd(&p, PROTO_T_THRUSTER_CFG, bad, 12u) == PROTO_R_BAD_VALUE;
        dp_body(bad, 40u, 10u, 20u, 80u, 60u, 60u, 2u, 15u);   n_bad += pc_cmd(&p, PROTO_T_THRUSTER_CFG, bad, 12u) == PROTO_R_BAD_VALUE;
        dp_body(bad, 40u, 10u, 20u, 80u, 60u, 60u, 15u, 101u); n_bad += pc_cmd(&p, PROTO_T_THRUSTER_CFG, bad, 12u) == PROTO_R_BAD_VALUE;
        x->sb.helm = 3u;                                        /* a gateway station holds HELM */
        uint8_t r_helm = pc_cmd(&p, PROTO_T_THRUSTER_CFG, body, sizeof body);
        x->sb.helm = SB_HELM_USB;
        joy_set(0.0f, 0.5f, false, false);                      /* MANUAL, driving ahead */
        sbsim_run(x, x->t + 3000u, dp_hook, &w);
        bool manual = sb_mode(&x->sb) == MODE_MANUAL;
        uint8_t r_mode = pc_cmd(&p, PROTO_T_THRUSTER_CFG, body, sizeof body);
        bool none1 = !x->sb.thrust.have_cfg;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(x, x->t + 4000u, dp_hook, &w);
        press(x, HAL_BTN_STANDBY);
        sbsim_run(x, x->t + 3000u, dp_hook, &w);
        sb_alarm_ack(&x->sb);
        /* taken: STANDBY, HELM */
        uint8_t r_ok = pc_cmd(&p, PROTO_T_THRUSTER_CFG, body, sizeof body);
        pc_run(&p, 300u);
        uint16_t want = proto_crc16(body, sizeof body);
        const uint8_t *n = &p.t2[p.t2_len - T2_N];
        uint16_t t2_hash = (uint16_t)(n[7] | (n[8] << 8));
        bool taken = r_ok == PROTO_R_OK && x->sb.thrust.have_cfg && x->sb.thrust.cfg_hash == want && t2_hash == want &&
                     (n[0] & 0x31u) == 0x31u && n[5] == 40u && n[6] == 80u && fabsf(x->sb.thrust.cfg.jog_m - 1.5f) < 0.01f;
        /* the only writer: one hal_set_thrusters per control step, 0 the whole time without DP */
        uint32_t w0 = sim_hal_thruster_writes(), s0 = w.steps;
        joy_set(0.0f, 0.6f, false, false);
        sbsim_run(x, x->t + 4000u, dp_hook, &w);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(x, x->t + 4000u, dp_hook, &w);
        sim_hal_press(HAL_BTN_STANDBY);                         /* (counted: the hook runs every step) */
        sbsim_run(x, x->t + 3000u, dp_hook, &w);
        uint32_t writes = sim_hal_thruster_writes() - w0, steps = w.steps - s0;
        float off = w.max_off;
        sb_alarm_ack(&x->sb);
        /* DP "active" (test hook): clamped to 40 / 80 %, rate limited, a reversal through 0 */
        memset(&w, 0, sizeof w);
        w.pc = &p;
        sb_test_thrust(&x->sb, true, 100.0f, -100.0f);
        uint32_t t0 = x->t, t_cap = 0u;
        while (x->t < t0 + 4000u) {
            sbsim_run(x, x->t + 50u, dp_hook, &w);
            sim_hal_thrusters(&b, &s);
            if (!t_cap && b >= 40.0f) t_cap = x->t - t0;
        }
        sim_hal_thrusters(&b, &s);
        bool capped = b == 40.0f && s == -80.0f;
        sb_test_thrust(&x->sb, true, -100.0f, 100.0f);
        sbsim_run(x, x->t + 4000u, dp_hook, &w);
        float b2, s2;
        sim_hal_thrusters(&b2, &s2);
        bool reversed = b2 == -40.0f && s2 == 80.0f;
        sb_test_thrust(&x->sb, false, 0.0f, 0.0f);              /* DP ends: 0 within one step */
        sbsim_run(x, x->t + AP_PERIOD_MS, dp_hook, &w);
        float b3, s3;
        sim_hal_thrusters(&b3, &s3);
        bool stop = b3 == 0.0f && s3 == 0.0f;
        snprintf(d, sizeof d, "armed %s; no settings: TELEM2 N flag clear %s, DP asks 100 %% -> %.0f / %.0f (not fitted); "
                 "11 bytes -> %u, %d of 5 bad values refused, not HELM -> %u, MANUAL %s -> %u (nothing taken %s); STANDBY -> %u, "
                 "hash 0x%04x (TELEM2 0x%04x, want 0x%04x), max %u / %u; %lu writes in %lu steps, max |thrust| without DP %.1f; "
                 "DP: 40 %% after %lu ms, at the end %.0f / %.0f, reversed %.0f / %.0f, fastest rise %.1f %%/s (limit %.0f), "
                 "over the cap %.2f, off -> %.0f / %.0f in one step; monitor %s",
                 armed ? "yes" : "NO", none0 ? "yes" : "NO", ub, us, r_len, n_bad, r_helm, manual ? "yes" : "NO", r_mode,
                 none1 ? "yes" : "NO", r_ok, x->sb.thrust.cfg_hash, t2_hash, want, n[5], n[6], (unsigned long)writes,
                 (unsigned long)steps, off, (unsigned long)t_cap, b, s, b2, s2, w.max_rise, THR_RATE_PCT_S, w.max_over,
                 b3, s3, x->mon.tripped ? monitor_reason_name(x->mon.reason) : "quiet");
        test_report("DP1 thruster settings (HELM, STANDBY, checked); thruster.c the only writer: every step, 0 outside DP, "
                    "clamp, rate limit, stop at once",
                    armed && none0 && unfitted && r_len == PROTO_R_BAD_LENGTH && n_bad == 5 && r_helm == PROTO_R_NOT_HELM &&
                    manual && r_mode == PROTO_R_STANDBY_ONLY && none1 && taken && writes == steps && steps > 100u &&
                    off == 0.0f && capped && t_cap >= 750u && t_cap <= 1000u && reversed &&
                    w.max_rise <= THR_RATE_PCT_S + 0.5f && w.max_over <= 0.0f && stop && !x->mon.tripped, d);
    }

    /* DP2 CPU1 + hal_safe_outputs: thrust without dp_active / above the cap trips the monitor; a trip sets
     * both thrusters to 0 at once and keeps them there (latched); a reset forgets the settings (RAM only) */
    {
        mon_reason_t why[8];
        bool c1 = dp_mon_case(0u, 10.0f, 0.0f, 40.0f, 80.0f, &why[0]);        /* thrust without DP */
        bool c2 = dp_mon_case(1u, 30.0f, -70.0f, 40.0f, 80.0f, &why[1]);      /* inside the caps: fine */
        bool c3 = dp_mon_case(1u, 45.0f, 0.0f, 40.0f, 80.0f, &why[2]);        /* above the bow cap */
        bool c4 = dp_mon_case(1u, 0.0f, -85.0f, 40.0f, 80.0f, &why[3]);       /* above the stern cap (astern) */
        bool c5 = dp_mon_case(1u, NAN, 0.0f, 40.0f, 80.0f, &why[4]);          /* a broken value */
        bool c6 = dp_mon_case(0u, 0.3f, -0.3f, 0.0f, 0.0f, &why[5]);          /* below the "thrust" threshold */
        bool c7 = dp_mon_case(1u, 5.0f, 0.0f, 0.0f, 0.0f, &why[6]);           /* not fitted (cap 0) */
        bool mon_ok = c1 && why[0] == MON_TRIP_THRUSTER && !c2 && c3 && why[2] == MON_TRIP_THRUSTER && c4 &&
                      why[3] == MON_TRIP_THRUSTER && c5 && why[4] == MON_TRIP_THRUSTER && !c6 && c7;
        /* the big boat publishes no thrust */
        static mon_shared_t bb;
        memset((void *)&bb, 0, sizeof bb);
        bb.thr_bow_pct = 50.0f; bb.dp_active = 1u; bb.thr_stern_cap = 9.0f;
        monitor_publish_steering_only(&bb, 1000u, 1500u);
        bool big = bb.dp_active == 0u && bb.thr_bow_pct == 0.0f && bb.thr_stern_pct == 0.0f && bb.thr_bow_cap == 0.0f &&
                   bb.thr_stern_cap == 0.0f;
        /* core 0 thrusting "forgets" dp_active (the shared block right after every step): trip */
        static pcsim_t p;
        static dpw_t w;
        bool set1 = dp_setup(&p, 1902u, true, 60u, 120u, 120u, 60u, 120u, 120u);
        sbsim_t *x = &p.x;
        memset(&w, 0, sizeof w);
        w.pc = &p;
        sb_test_thrust(&x->sb, true, 40.0f, 40.0f);
        sbsim_run(x, x->t + 2000u, dp_hook, &w);
        bool running = !x->mon.tripped;
        uint32_t t0 = x->t;
        w.clear_dp = true;
        while (!x->mon.tripped && x->t < t0 + 1000u) sbsim_run(x, x->t + 10u, dp_hook, &w);
        uint32_t t_forget = x->t - t0;
        float b, s;
        sim_hal_thrusters(&b, &s);
        bool forget = x->mon.tripped && x->mon.reason == MON_TRIP_THRUSTER && b == 0.0f && s == 0.0f;
        /* core 0 frozen while thrusting: heartbeat trip, both 0 (hal_safe_outputs), latched afterwards */
        bool set2 = dp_setup(&p, 1903u, true, 60u, 120u, 120u, 60u, 120u, 120u);
        memset(&w, 0, sizeof w);
        w.pc = &p;
        sb_test_thrust(&x->sb, true, 50.0f, -50.0f);
        sbsim_run(x, x->t + 2500u, dp_hook, &w);
        sim_hal_thrusters(&b, &s);
        bool thrusting = b == 50.0f && s == -50.0f;
        x->wdt_ms = 0u;                                         /* (no watchdog reset: the monitor must do it) */
        x->frozen = true;
        t0 = x->t;
        float b_mid = 0.0f, s_mid = 0.0f;
        while (!x->mon.tripped && x->t < t0 + 1000u) sbsim_run(x, x->t + 10u, NULL, NULL);
        uint32_t t_trip = x->t - t0;
        sim_hal_thrusters(&b_mid, &s_mid);
        x->frozen = false;                                      /* core 0 back, still asking for thrust */
        sbsim_run(x, x->t + 2000u, dp_hook, &w);
        sim_hal_thrusters(&b, &s);
        /* (DroneCAN servos: the actuator task stops with core 0 - MON_TRIP_CAN_TX is the first trip) */
        bool latched = x->mon.tripped && (x->mon.reason == MON_TRIP_HEARTBEAT ||
                                          (s_can_all && x->mon.reason == MON_TRIP_CAN_TX)) && b_mid == 0.0f && s_mid == 0.0f &&
                       b == 0.0f && s == 0.0f && !x->sb.thrust.active && x->sb.mon_tripped && hal_outputs_forced();
        /* a chip reset (watchdog): the settings are gone (RAM only), the thrusters 0 */
        x->wdt_ms = 100u;
        x->frozen = true;
        uint32_t resets0 = x->resets;
        sbsim_run(x, x->t + 600u, NULL, NULL);
        x->frozen = false;
        sbsim_run(x, x->t + 1500u, NULL, NULL);
        sim_hal_thrusters(&b, &s);
        bool reset = x->resets > resets0 && !x->sb.thrust.have_cfg && b == 0.0f && s == 0.0f;
        sb_test_thrust(&x->sb, false, 0.0f, 0.0f);
        snprintf(d, sizeof d, "monitor cases: no DP + 10 %% %s (%s), inside the caps %s, bow 45 > 40 %s, stern -85 > 80 %s, "
                 "NaN %s, 0.3 %% %s, cap 0 %s; big boat publishes 0 %s; core 0 forgets dp_active (set up %s, running %s): "
                 "trip after %lu ms %s; frozen core 0 while thrusting (set up %s, %s): trip after %lu ms, at the trip %.0f / %.0f, "
                 "core 0 back asking for thrust -> %.0f / %.0f latched %s; watchdog reset: settings gone + 0 %s",
                 c1 ? "trip" : "NO TRIP", monitor_reason_name(why[0]), c2 ? "TRIP" : "ok", c3 ? "trip" : "NO TRIP",
                 c4 ? "trip" : "NO TRIP", c5 ? "trip" : "NO TRIP", c6 ? "TRIP" : "ok", c7 ? "trip" : "NO TRIP",
                 big ? "yes" : "NO", set1 ? "yes" : "NO", running ? "yes" : "NO", (unsigned long)t_forget,
                 forget ? "ok" : "FAILED", set2 ? "yes" : "NO", thrusting ? "50 / -50" : "NOT THRUSTING",
                 (unsigned long)t_trip, b_mid, s_mid, b, s, latched ? "yes" : "NO", reset ? "yes" : "NO");
        test_report("DP2 CPU1: thrust without DP / above the cap -> trip; hal_safe_outputs sets both thrusters 0 (latched); "
                    "settings RAM only",
                    mon_ok && big && set1 && running && forget && t_forget <= 100u && set2 && thrusting &&
                    t_trip <= MON_HB_MS + 60u && latched && reset, d);
    }

    /* DP3 heat budget per thruster (run_s / rest_s): used up -> that thruster 0 + DP THRUSTER HOT (an alarm,
     * kept by the ACK while hot), the other runs on; rested -> runs again. The simulated boat: the bow
     * thruster sways + turns it, the stern motor drives it ahead, the bank sags with the thrust */
    {
        static pcsim_t p;
        static dpw_t w;
        bool set = dp_setup(&p, 1904u, true, 50u, 10u, 20u, 50u, 60u, 30u);
        sbsim_t *x = &p.x;
        memset(&w, 0, sizeof w);
        w.pc = &p;
        float v0 = 0.0f;
        (void)hal_read_supply_voltage(&v0);
        double n0 = x->w.north_m, e0 = x->w.east_m;
        float h0 = x->w.heading_deg;
        sb_mode_t m0 = sb_mode(&x->sb);
        sb_test_thrust(&x->sb, true, 50.0f, 50.0f);
        sbsim_run(x, x->t + 8000u, dp_hook, &w);
        float v1 = 0.0f;
        (void)hal_read_supply_voltage(&v1);
        float turned = x->w.heading_deg - h0;
        if (turned < -180.0f) turned += 360.0f;
        /* the boat (heading 90 = east): ahead = east, to starboard = south */
        double ahead = x->w.east_m - e0, starboard = -(x->w.north_m - n0);
        float sway = x->w.sway_ms, kn = x->w.speed_kn;
        bool cool8 = !x->sb.thrust.hot[THR_BOW] && (x->sb.alarm_bits2 & SB_ALM2_DP_HEAT) == 0u;
        uint32_t t0 = x->t, t_hot = 0u;
        while (x->t < t0 + 5000u && !t_hot) {
            sbsim_run(x, x->t + 50u, dp_hook, &w);
            if (x->sb.thrust.hot[THR_BOW]) t_hot = x->t - t0 + 8000u;
        }
        float b, s;
        sim_hal_thrusters(&b, &s);
        float hb = b, hs = s;
        bool hot = t_hot >= 9800u && t_hot <= 10600u && b == 0.0f && s == 50.0f && (x->sb.alarm_bits2 & SB_ALM2_DP_HEAT) &&
                   x->sb.alarm && sim_hal_alarm() && sb_mode(&x->sb) == m0;
        sb_alarm_ack(&x->sb);
        bool kept = (x->sb.alarm_bits2 & SB_ALM2_DP_HEAT) != 0u;
        /* rest: 20 s (budget 10 s used, cooled at 10 / 20 s per s) */
        t0 = x->t;
        uint32_t t_cool = 0u;
        while (x->t < t0 + 25000u && !t_cool) {
            sbsim_run(x, x->t + 50u, dp_hook, &w);
            if (!x->sb.thrust.hot[THR_BOW]) t_cool = x->t - t0;
        }
        sbsim_run(x, x->t + 2000u, dp_hook, &w);
        sim_hal_thrusters(&b, &s);
        bool again = t_cool >= 19500u && t_cool <= 20500u && b == 50.0f;
        sb_alarm_ack(&x->sb);
        bool cleared = (x->sb.alarm_bits2 & SB_ALM2_DP_HEAT) == 0u;
        uint8_t heat_s = (uint8_t)(thruster_heat(&x->sb.thrust, THR_STERN) * 100.0f + 0.5f);
        sb_test_thrust(&x->sb, false, 0.0f, 0.0f);
        snprintf(d, sizeof d, "set up %s; 8 s at 50 / 50 %%: turned %+.1f deg, %.1f m to starboard (sway %.2f m/s), %.1f m ahead "
                 "(%.2f kn), bank %.2f -> %.2f V, not hot yet %s; bow HOT after %lu ms (budget 10 s): outputs %.0f / %.0f, "
                 "alarm DP HEAT + buzzer, mode unchanged %s; ACK while hot keeps it %s; cooled after %lu ms (rest 20 s), "
                 "bow at %.0f %% again %s; ACK then clears it %s; stern budget used %u %% (60 s); HOT outputs not 0: %lu steps",
                 set ? "yes" : "NO", turned, starboard, sway, ahead, kn, v0, v1, cool8 ? "yes" : "NO", (unsigned long)t_hot,
                 hb, hs, hot ? "yes" : "NO", kept ? "yes" : "NO", (unsigned long)t_cool, b, again ? "yes" : "NO",
                 cleared ? "yes" : "NO", heat_s, (unsigned long)w.hot_on);
        test_report("DP3 heat budget per thruster: used up -> 0 + DP THRUSTER HOT (alarm), rested -> again; the sim boat "
                    "sways / turns / moves ahead, the bank sags",
                    set && turned > 8.0f && starboard > 0.5 && sway > 0.1f && ahead > 1.0 && kn > 0.5f &&
                    v0 - v1 > 0.5f && cool8 && hot && kept && again && cleared && w.hot_on == 0u && !x->mon.tripped, d);
    }

    /* DP4 the e-stop (its contactor cuts the thruster power AND the software sends 0 within one step, while
     * pressed), FIRST TEST (never), outputs not armed (never) */
    {
        static pcsim_t p;
        static dpw_t w;
        bool set = dp_setup(&p, 1905u, true, 60u, 120u, 120u, 60u, 120u, 120u);
        sbsim_t *x = &p.x;
        memset(&w, 0, sizeof w);
        w.pc = &p;
        sb_test_thrust(&x->sb, true, 60.0f, -60.0f);
        sbsim_run(x, x->t + 3000u, dp_hook, &w);
        float b, s, tb, ts;
        sim_hal_thrusters(&b, &s);
        bool run = b == 60.0f && s == -60.0f;
        sim_hal_set_estop(true, true);
        sim_hal_thrust_truth(&tb, &ts);                         /* the contactor: at once, before any software */
        sbsim_run(x, x->t + AP_PERIOD_MS, dp_hook, &w);
        float b1, s1;
        sim_hal_thrusters(&b1, &s1);
        sbsim_run(x, x->t + 3000u, dp_hook, &w);
        float b2, s2;
        sim_hal_thrusters(&b2, &s2);
        uint8_t why_es = x->sb.thrust.why;
        bool es = tb == 0.0f && ts == 0.0f && b1 == 0.0f && s1 == 0.0f && b2 == 0.0f && s2 == 0.0f &&
                  x->sb.thrust.why == THR_WHY_BLOCKED && (x->sb.alarm_bits & SB_ALM_ESTOP);
        sim_hal_set_estop(true, false);
        sbsim_run(x, x->t + 500u, dp_hook, &w);
        sb_test_thrust(&x->sb, false, 0.0f, 0.0f);
        sim_hal_set_estop(false, false);
        /* FIRST TEST: never, whatever DP asks */
        sb_config_t c;
        sb_default_config(&c);
        c.first_test = true;
        static sbsim_t y;
        static dpw_t v;
        sbsim_init(&y, 1906u, 90.0f, 0.0f, &c);
        sbsim_run(&y, 2500u, NULL, NULL);
        thr_cfg_t tc;
        uint8_t body[THR_FRAME_BYTES];
        dp_body(body, 100u, 600u, 600u, 100u, 600u, 600u, 15u, 15u);
        bool dec = thruster_cfg_decode(body, sizeof body, &tc);
        sb_result_t fr = sb_thruster_cfg(&y.sb, &tc, 1u);
        memset(&v, 0, sizeof v);
        sb_test_thrust(&y.sb, true, 100.0f, 100.0f);
        sbsim_run(&y, y.t + 3000u, dp_hook, &v);
        float ft_max = v.max_off;
        bool ft = dec && fr == SB_OK && v.max_off == 0.0f && !y.sb.thrust.active && y.sb.thrust.why == THR_WHY_BLOCKED &&
                  !y.mon.tripped;
        sb_test_thrust(&y.sb, false, 0.0f, 0.0f);
        /* not armed (the joystick held at power-up): never */
        sbsim_init(&y, 1907u, 90.0f, 0.0f, NULL);
        joy_set(0.0f, 0.6f, false, false);
        sbsim_run(&y, 2500u, NULL, NULL);
        bool unarmed = !y.sb.armed;
        (void)sb_thruster_cfg(&y.sb, &tc, 1u);
        memset(&v, 0, sizeof v);
        sb_test_thrust(&y.sb, true, 100.0f, 100.0f);
        sbsim_run(&y, y.t + 2000u, dp_hook, &v);
        bool na = unarmed && v.max_off == 0.0f && !y.sb.thrust.active;
        sb_test_thrust(&y.sb, false, 0.0f, 0.0f);
        joy_set(0.0f, 0.0f, false, false);
        snprintf(d, sizeof d, "set up %s, thrusting 60 / -60 %s; e-stop: contactor %.0f / %.0f at once, software %.0f / %.0f "
                 "after one step, %.0f / %.0f after 3 s (why %u, alarm EMERGENCY STOP) %s; FIRST TEST (settings %s / %d): "
                 "max %.1f %% %s; not armed %s: max %.1f %% %s",
                 set ? "yes" : "NO", run ? "yes" : "NO", tb, ts, b1, s1, b2, s2, why_es, es ? "ok" : "FAILED",
                 dec ? "ok" : "BAD", (int)fr, ft_max, ft ? "ok" : "FAILED", unarmed ? "yes" : "NO", v.max_off,
                 na ? "ok" : "FAILED");
        test_report("DP4 e-stop: thrusters 0 (contactor + software within one step); never in FIRST TEST or not armed",
                    set && run && es && ft && na, d);
    }
}


/* ------------------------------------------- DP mode (CLAUDE.md P, step 10) -- */

/* A boat with the boat file's thruster settings, the gateway (Pi) on its UART, station 2 holds HELM
 * (CREWED: the onboard joystick confirms; UNCREWED: the shore joystick streams + confirms). Measured
 * after every control step: what the outboard / thrusters do in and out of DP. */
typedef struct {
    sbsim_t  x;
    rjsim_t  r;
    bool     crewed;
    uint32_t steps, dp_steps;
    uint32_t settle_until;      /* SPOT / CRUISE judged from here */
    float    max_err;           /* SPOT / JOG: true distance boat -> spot, m */
    float    max_hdg;           /* |true heading - target| (not TRIP) */
    float    max_outboard;      /* outboard throttle servo % in DP (generator not active) */
    uint32_t gear_off;          /* steps in DP with the gear pulse not N */
    uint32_t steer_on;          /* steps in DP with the outboard steering motor on */
    uint32_t thrust_outside;    /* steps NOT in DP with a thruster output != 0 */
    uint32_t dp_no_gps;         /* steps in DP without a trusted GPS */
    float    max_over;          /* |output| above its cap */
    uint32_t hot_on;            /* a HOT thruster's output != 0 */
    float    max_kn, min_kn;    /* true SOG (judged) */
    float    sum_kn;
    uint32_t n_kn;
} dpx_t;

static float dp_true_err(const sbsim_t *x)
{
    double n, e;
    world_from_latlon(x->sb.dp.alat, x->sb.dp.alon, &n, &e);
    double dn = x->w.north_m - n, de = x->w.east_m - e;
    return (float)sqrt(dn * dn + de * de);
}

static float dp_wrap180(float a)
{
    a = fmodf(a, 360.0f);
    if (a > 180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

static void dpx_hook(sbsim_t *x, void *ctx)
{
    dpx_t *d = (dpx_t *)ctx;
    rj_hook(x, &d->r);
    d->steps++;
    float b, s;
    sim_hal_thrusters(&b, &s);
    const thruster_t *t = &x->sb.thrust;
    d->max_over = fmaxf(d->max_over, fmaxf(fabsf(b) - thruster_cap(t, THR_BOW), fabsf(s) - thruster_cap(t, THR_STERN)));
    if ((t->hot[THR_BOW] && b != 0.0f) || (t->hot[THR_STERN] && s != 0.0f)) d->hot_on++;
    if (sb_mode(&x->sb) != MODE_DP) {
        if (b != 0.0f || s != 0.0f) d->thrust_outside++;
        return;
    }
    d->dp_steps++;
    if (!sb_gps_ok(&x->sb)) d->dp_no_gps++;
    if (!x->sb.gen.active) d->max_outboard = fmaxf(d->max_outboard, sim_hal_throttle());
    if (sim_hal_gear_pwm() != x->sb.cfg.gear.pwm_n_us) d->gear_off++;
    if (fabsf(sim_hal_motor_command()) > 0.01f) d->steer_on++;
    if (x->t >= d->settle_until) {
        uint8_t sub = x->sb.dp.sub;
        if (sub == DP_SPOT || sub == DP_JOG) d->max_err = fmaxf(d->max_err, dp_true_err(x));
        if (sub != DP_TRIP) d->max_hdg = fmaxf(d->max_hdg, fabsf(dp_wrap180(x->w.heading_deg - x->sb.dp.hdg_target)));
        float kn = world_sog_kn(&x->w);
        d->max_kn = fmaxf(d->max_kn, kn);
        d->min_kn = fminf(d->min_kn, kn);
        d->sum_kn += kn;
        d->n_kn++;
    }
}

static void dpx_run(dpx_t *d, uint32_t ms)
{
    sbsim_run(&d->x, d->x.t + ms, dpx_hook, d);
}

/* judge from now + settle_ms (the measures start again) */
static void dpx_judge(dpx_t *d, uint32_t settle_ms)
{
    d->settle_until = d->x.t + settle_ms;
    d->max_err = d->max_hdg = 0.0f;
    d->max_kn = 0.0f;
    d->min_kn = 99.0f;
    d->sum_kn = 0.0f;
    d->n_kn = 0u;
}

static uint8_t dpx_cmd_st(dpx_t *d, uint8_t st, uint8_t act, uint8_t arg, int16_t value)
{
    uint8_t pl[4] = { act, arg, (uint8_t)value, (uint8_t)((uint16_t)value >> 8) };
    return pi_cmd(&d->x, &d->r.p, st, false, PROTO_T_DP, pl, sizeof pl);
}

static uint8_t dpx_cmd(dpx_t *d, uint8_t act, uint8_t arg, int16_t value)
{
    return dpx_cmd_st(d, 2u, act, arg, value);
}

static bool dpx_confirm(dpx_t *d)
{
    return d->crewed ? onboard_confirm(&d->x, &d->r) : shore_confirm(&d->x, &d->r);
}

/* the boat file's thrusters (bow / stern max %, run_s, rest_s, spot_alarm_m) */
static uint8_t dpx_settings(dpx_t *d, uint8_t bmax, uint16_t brun, uint16_t brest, uint8_t smax, uint16_t srun,
                            uint16_t srest, uint8_t spot)
{
    uint8_t b[THR_FRAME_BYTES];
    dp_body(b, bmax, brun, brest, smax, srun, srest, 15u, spot);
    return pi_cmd(&d->x, &d->r.p, 2u, false, PROTO_T_THRUSTER_CFG, b, sizeof b);
}

/* armed in STANDBY, HELM station 2, the settings taken (a big heat budget) unless settings is false */
static bool dpx_init(dpx_t *d, uint32_t seed, bool crewed, const sb_config_t *cfg, bool settings)
{
    const uint8_t TAKE = 0u;
    memset(d, 0, sizeof *d);
    d->crewed = crewed;
    crew_init(&d->x, &d->r, seed, crewed, cfg);
    if (!crewed) d->r.rj_on = true;                 /* the shore captain's joystick streams (the CONFIRM source) */
    bool helm = pi_cmd(&d->x, &d->r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u) == PROTO_R_OK;
    dpx_run(d, 500u);
    sb_alarm_ack(&d->x.sb);
    bool set = !settings || dpx_settings(d, 100u, 1800u, 600u, 100u, 1800u, 600u, 15u) == PROTO_R_OK;
    dpx_run(d, 200u);
    d->min_kn = 99.0f;
    return helm && set && d->x.sb.armed && sb_mode(&d->x.sb) == MODE_STANDBY;
}

/* DP START (sub, speed kn) + the CONFIRM: true = in MODE_DP with that sub-state */
static bool dpx_start(dpx_t *d, uint8_t sub, float kn)
{
    if (dpx_cmd(d, DP_ACT_START, sub, (int16_t)lroundf(kn * 100.0f)) != PROTO_R_OK) return false;
    dpx_run(d, 800u);
    dpx_confirm(d);
    dpx_run(d, 100u);
    return sb_mode(&d->x.sb) == MODE_DP && d->x.sb.dp.sub == (sub & 0x7Fu);
}

/* the thruster outputs right now */
static void dpx_thr(float *b, float *s)
{
    sim_hal_thrusters(b, s);
}

static void dp_mode_tests(void)
{
    char d[1800];
    printf("\n=== DP mode (section P, step 10): SPOT / JOG / CRUISE / TRIP ===\n");

    /* DP5 every entry refusal, then PC + CONFIRM -> DP SPOT; UNCREWED: the shore CONFIRM enters */
    {
        static dpx_t a;
        int n_ok = 0, n_all = 0, len = 0;
        d[0] = 0;
#define DP5_CASE(name, cond) do { bool c_ = (cond); n_all++; if (c_) n_ok++; \
        len += snprintf(d + len, sizeof d - (size_t)len, "%s %s; ", name, c_ ? "ok" : "FAILED"); } while (0)
        bool set = dpx_init(&a, 2001u, true, NULL, false);
        DP5_CASE("set up", set);
        DP5_CASE("no thruster settings -> PRECOND", dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_PRECOND);
        DP5_CASE("settings taken", dpx_settings(&a, 100u, 1800u, 600u, 100u, 1800u, 600u, 15u) == PROTO_R_OK);
        DP5_CASE("bad sub -> BAD VALUE", dpx_cmd(&a, DP_ACT_START, 7u, 0) == PROTO_R_BAD_VALUE);
        DP5_CASE("3.1 kn -> BAD VALUE", dpx_cmd(&a, DP_ACT_START, DP_CRUISE, 310) == PROTO_R_BAD_VALUE);
        DP5_CASE("JOG outside DP -> WRONG MODE", dpx_cmd(&a, DP_ACT_JOG, DP_JOG_AHEAD, 0) == PROTO_R_WRONG_MODE);
        DP5_CASE("not the HELM station -> NOT HELM", dpx_cmd_st(&a, 3u, DP_ACT_START, DP_SPOT, 0) == PROTO_R_NOT_HELM);
        DP5_CASE("TRIP without a program -> NO PROGRAM", dpx_cmd(&a, DP_ACT_START, DP_TRIP, 0) == PROTO_R_NO_PROGRAM);
        joy_set(0.0f, 0.6f, false, false);                       /* MANUAL, driving ahead */
        dpx_run(&a, 1500u);
        DP5_CASE("MANUAL -> JOYSTICK", sb_mode(&a.x.sb) == MODE_MANUAL &&
                 dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_JOYSTICK);
        joy_set(0.0f, 0.0f, false, false);
        dpx_run(&a, 6000u);
        press(&a.x, HAL_BTN_STANDBY);
        dpx_run(&a, 15000u);                                     /* (the boat slows down) */
        sb_alarm_ack(&a.x.sb);
        press(&a.x, HAL_BTN_HDG_LOCK);
        dpx_run(&a, 300u);
        DP5_CASE("HDG LOCK -> STANDBY ONLY", sb_mode(&a.x.sb) == MODE_HDG_LOCK &&
                 dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_STANDBY_ONLY);
        press(&a.x, HAL_BTN_STANDBY);
        dpx_run(&a, 1000u);
        joy_set(0.5f, 0.0f, false, false);                       /* the stick held -> MANUAL; STANDBY: ignored */
        dpx_run(&a, 500u);
        press(&a.x, HAL_BTN_STANDBY);
        dpx_run(&a, 2000u);
        DP5_CASE("stick held -> NOT CENTRED", sb_mode(&a.x.sb) == MODE_STANDBY &&
                 dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_NOT_CENTERED);
        joy_set(0.0f, 0.0f, false, false);
        dpx_run(&a, 1000u);
        sb_alarm_ack(&a.x.sb);
        a.x.w.gps_fail = true;
        dpx_run(&a, 4000u);
        DP5_CASE("GPS lost -> PRECOND", dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_PRECOND);
        a.x.w.gps_fail = false;
        dpx_run(&a, 5000u);
        sim_hal_fail_heading(true);
        dpx_run(&a, 2000u);
        DP5_CASE("compass lost -> PRECOND", dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_PRECOND);
        sim_hal_fail_heading(false);
        dpx_run(&a, 2000u);
        a.x.w.current_e = 1.3f;                                  /* drifting 2.5 kn: too fast to start */
        dpx_run(&a, 5000u);
        DP5_CASE("SOG 2.5 kn -> PRECOND", dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0) == PROTO_R_PRECOND);
        a.x.w.current_e = 0.0f;
        dpx_run(&a, 3000u);
        sb_alarm_ack(&a.x.sb);
        /* step 11g (review finding 6): the MODELLED throttle not at idle (thr_est) -> refused, at the request and
         * at the CONFIRM (the precondition again) */
        a.x.sb.servo.est_pct = 40.0f;
        bool ref_req = sb_dp_cmd(&a.x.sb, a.x.sb.helm, DP_ACT_START, DP_SPOT, 0) == SB_ERR_PRECOND;
        dpx_run(&a, 4000u);
        bool ok_req = sb_dp_cmd(&a.x.sb, a.x.sb.helm, DP_ACT_START, DP_SPOT, 0) == SB_OK && a.x.sb.confirm.kind == CONF_DP;
        a.x.sb.servo.est_pct = 100.0f;                           /* (as after a CAN boot: unknown = not idle) */
        bool ref_conf = !onboard_confirm(&a.x, &a.r) || sb_mode(&a.x.sb) == MODE_STANDBY;
        ref_conf = ref_conf && sb_mode(&a.x.sb) == MODE_STANDBY;
        a.x.sb.confirm.kind = CONF_NONE;
        dpx_run(&a, 8000u);
        DP5_CASE("throttle not at the modelled idle -> PRECOND (request + CONFIRM)", ref_req && ok_req && ref_conf);
        /* no CONFIRM within 10 s: nothing */
        uint8_t r_req = dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0);
        bool pend = a.x.sb.confirm.kind == CONF_DP;
        dpx_run(&a, 10500u);
        bool late = onboard_confirm(&a.x, &a.r);
        DP5_CASE("no CONFIRM in 10 s: stays STANDBY", r_req == PROTO_R_OK && pend && !late && sb_mode(&a.x.sb) == MODE_STANDBY);
        /* the request + CONFIRM: DP SPOT, outboard idle + neutral, its steering off */
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        dpx_run(&a, 5000u);
        DP5_CASE("PC + CONFIRM -> DP SPOT", in && sb_mode(&a.x.sb) == MODE_DP && a.x.sb.dp.sub == DP_SPOT &&
                 a.x.sb.thrust.active && a.max_outboard <= 0.01f && a.gear_off == 0u && a.steer_on == 0u &&
                 a.x.sb.ap.mode == AP_MODE_STANDBY);
        DP5_CASE("a mode request from the PC can't enter DP", (sb_request_mode(&a.x.sb, MODE_STANDBY, WHY_PC),
                 !sb_request_mode(&a.x.sb, MODE_DP, WHY_PC) && sb_mode(&a.x.sb) == MODE_STANDBY));
        dpx_run(&a, 1000u);
        /* FIRST TEST: never */
        {
            sb_config_t c;
            sb_default_config(&c);
            c.first_test = true;
            static dpx_t f;
            (void)dpx_init(&f, 2002u, true, &c, true);
            DP5_CASE("FIRST TEST -> WRONG MODE", dpx_cmd(&f, DP_ACT_START, DP_SPOT, 0) == PROTO_R_WRONG_MODE &&
                     !sb_request_mode(&f.x.sb, MODE_DP, WHY_DP));
        }
        /* UNCREWED: the shore CONFIRM; the onboard CONFIRM does nothing */
        {
            static dpx_t u;
            bool us = dpx_init(&u, 2003u, false, NULL, true);
            bool prof = u.x.sb.profile == SB_PROFILE_UNCREWED;
            uint8_t r1 = dpx_cmd(&u, DP_ACT_START, DP_SPOT, 0);
            dpx_run(&u, 500u);
            sim_hal_joystick_confirm(true);                       /* (no onboard joystick: nothing) */
            dpx_run(&u, 200u);
            sim_hal_joystick_confirm(false);
            bool still = sb_mode(&u.x.sb) == MODE_STANDBY;
            bool sc = shore_confirm(&u.x, &u.r);
            dpx_run(&u, 500u);
            DP5_CASE("UNCREWED: shore CONFIRM -> DP", us && prof && r1 == PROTO_R_OK && still && sc &&
                     sb_mode(&u.x.sb) == MODE_DP);
        }
#undef DP5_CASE
        test_report("DP5 DP entry: HELM + PC request + CONFIRM (onboard / shore) from STANDBY only; refused without "
                    "settings, in FIRST TEST, MANUAL / HDG LOCK, stick held, GPS / compass lost, moving, no program, throttle not idle",
                    n_ok == n_all, d);
    }

    /* DP6 SPOT holds in a current from ahead (and a little from the side): the true position within a few metres
     * for 3 min, the heading held, the outboard idle + neutral with its steering off the whole time, the thrusters
     * within their caps; the heading moves with HEADING_ADJ / the course buttons */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2004u, true, NULL, true);
        a.x.w.current_e = -0.25f;                                /* heading 90: 0.5 kn from ahead (a current from the
                                                                    side drifts the boat: the bow can't sway without
                                                                    turning it - heading first) */
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        dpx_judge(&a, 60000u);
        dpx_run(&a, 240000u);
        float err = a.max_err, hdg = a.max_hdg;
        float b, s;
        dpx_thr(&b, &s);
        bool still_dp = sb_mode(&a.x.sb) == MODE_DP && a.x.sb.dp.sub == DP_SPOT;
        uint32_t alarms2 = a.x.sb.alarm_bits2;
        /* heading +10 (HEADING_ADJ) and -1 (the course button) */
        int8_t ten = 10;
        uint8_t r_adj = pi_cmd(&a.x, &a.r.p, 2u, false, PROTO_T_HEADING_ADJ, (const uint8_t *)&ten, 1u);
        float t1 = a.x.sb.dp.hdg_target;
        press(&a.x, HAL_BTN_MINUS_1);
        float t2 = a.x.sb.dp.hdg_target;
        dpx_judge(&a, 40000u);
        dpx_run(&a, 60000u);
        float hdg2 = a.max_hdg, err2 = a.max_err;
        snprintf(d, sizeof d, "set up %s, DP %s; current 0.5 kn from ahead: after 1 min for 3 min the "
                 "spot within %.1f m, heading within %.1f deg, still SPOT %s, stern %.0f %% (I %.0f %%), bow %.0f %%; outboard max "
                 "%.1f %%, gear off N %lu steps, steering on %lu steps, over the cap %.2f, no DP alarm (word 2 0x%lx) %s; "
                 "HEADING_ADJ +10 -> %u, target %.0f, course button -1 -> %.0f; then heading within %.1f deg, spot %.1f m",
                 set ? "yes" : "NO", in ? "SPOT" : "NOT ENTERED", err, hdg, still_dp ? "yes" : "NO", s, a.x.sb.dp.ix, b,
                 a.max_outboard,
                 (unsigned long)a.gear_off, (unsigned long)a.steer_on, a.max_over, (unsigned long)alarms2,
                 (alarms2 & (SB_ALM2_DP_OFFSPOT | SB_ALM2_DP_OVERPOWER | SB_ALM2_DP_HEAT)) == 0u ? "yes" : "NO", r_adj, t1, t2,
                 hdg2, err2);
        test_report("DP6 SPOT holds position + heading in a current; outboard idle + neutral, steering off; "
                    "heading +-1 / +-10 (HEADING_ADJ, course buttons)",
                    set && in && err < 4.0f && hdg < 4.0f && still_dp && a.x.sb.dp.ix > 10.0f && a.max_outboard <= 0.01f &&
                    a.gear_off == 0u && a.steer_on == 0u && a.max_over <= 0.0f &&
                    (alarms2 & (SB_ALM2_DP_OFFSPOT | SB_ALM2_DP_OVERPOWER | SB_ALM2_DP_HEAT)) == 0u &&
                    r_adj == PROTO_R_OK && fabsf(dp_wrap180(t1 - 100.0f)) < 1.5f && fabsf(dp_wrap180(t2 - (t1 - 1.0f))) < 0.01f &&
                    hdg2 < 4.0f && err2 < 4.0f && a.thrust_outside == 0u, d);
    }

    /* DP7 JOG: 1.5 m (boat file jog_m) ahead / to starboard / astern / to port of the anchor heading; JOG -> SPOT
     * once there; refused in CRUISE */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2005u, true, NULL, true);
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        dpx_run(&a, 20000u);
        static const uint8_t DIR[4] = { DP_JOG_AHEAD, DP_JOG_STBD, DP_JOG_ASTERN, DP_JOG_PORT };
        static const char *const NM[4] = { "ahead", "starboard", "astern", "port" };
        int good = 0, len = 0;
        char t[700] = "";
        for (int k = 0; k < 4; k++) {
            double n0 = a.x.w.north_m, e0 = a.x.w.east_m;
            double an0 = 0, ae0 = 0, an1 = 0, ae1 = 0;
            world_from_latlon(a.x.sb.dp.alat, a.x.sb.dp.alon, &an0, &ae0);
            uint8_t r = dpx_cmd(&a, DP_ACT_JOG, DIR[k], 0);
            bool jog = a.x.sb.dp.sub == DP_JOG;
            world_from_latlon(a.x.sb.dp.alat, a.x.sb.dp.alon, &an1, &ae1);
            dpx_run(&a, 40000u);
            /* heading 90 (east): ahead = east, starboard = south */
            float h = a.x.sb.dp.hdg_target * 3.14159265f / 180.0f;
            float sf = (float)((an1 - an0) * cos(h) + (ae1 - ae0) * sin(h));
            float ss = (float)(-(an1 - an0) * sin(h) + (ae1 - ae0) * cos(h));
            float bf = (float)((a.x.w.north_m - n0) * cos(h) + (a.x.w.east_m - e0) * sin(h));
            float bs = (float)(-(a.x.w.north_m - n0) * sin(h) + (a.x.w.east_m - e0) * cos(h));
            float wf = (k == 0) ? 1.5f : (k == 2) ? -1.5f : 0.0f, ws = (k == 1) ? 1.5f : (k == 3) ? -1.5f : 0.0f;
            bool ok = r == PROTO_R_OK && jog && fabsf(sf - wf) < 0.05f && fabsf(ss - ws) < 0.05f &&
                      a.x.sb.dp.sub == DP_SPOT && dp_true_err(&a.x) < 3.0f &&
                      (wf != 0.0f ? bf * wf > 0.0f : bs * ws > 0.0f);
            good += ok;
            len += snprintf(t + len, sizeof t - (size_t)len, "%s%s: spot moved %+.2f / %+.2f m, boat %+.1f / %+.1f m, "
                            "back in SPOT %s (%.1f m off)", k ? "; " : "", NM[k], sf, ss, bf, bs,
                            a.x.sb.dp.sub == DP_SPOT ? "yes" : "NO", dp_true_err(&a.x));
        }
        uint8_t r_cr = 0xFFu;
        bool cr = dpx_start(&a, DP_CRUISE, 0.5f);
        if (cr) r_cr = dpx_cmd(&a, DP_ACT_JOG, DP_JOG_AHEAD, 0);
        snprintf(d, sizeof d, "set up %s, SPOT %s; %s; in CRUISE (%s) a JOG -> %u", set ? "yes" : "NO", in ? "yes" : "NO",
                 t, cr ? "yes" : "NO", r_cr);
        test_report("DP7 JOG 1.5 m fore / aft / port / starboard of the anchor heading, back to SPOT; not in CRUISE",
                    set && in && good == 4 && cr && r_cr == PROTO_R_WRONG_MODE && a.thrust_outside == 0u, d);
    }

    /* DP8 CRUISE: heading + speed set point on the stern motor (<= dp_max_kn); SPEED / HEADING change it; a
     * SPOT command stops the boat there (no CONFIRM) */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2006u, true, NULL, true);
        bool in = dpx_start(&a, DP_CRUISE, 2.0f);
        dpx_judge(&a, 30000u);
        dpx_run(&a, 60000u);
        float avg1 = a.n_kn ? a.sum_kn / (float)a.n_kn : 0.0f, hdg1 = a.max_hdg;
        uint8_t r_fast = dpx_cmd(&a, DP_ACT_SPEED, 0u, 350);
        uint8_t r_sp = dpx_cmd(&a, DP_ACT_SPEED, 0u, 100);
        uint8_t r_hd = dpx_cmd(&a, DP_ACT_HEADING, 0u, 1200);           /* 120 deg */
        dpx_judge(&a, 40000u);
        dpx_run(&a, 60000u);
        float avg2 = a.n_kn ? a.sum_kn / (float)a.n_kn : 0.0f, hdg2 = a.max_hdg;
        float cog = world_cog_deg(&a.x.w);
        uint8_t r_spot = dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0);
        bool spot = a.x.sb.dp.sub == DP_SPOT && sb_mode(&a.x.sb) == MODE_DP;
        dpx_run(&a, 60000u);
        float kn3 = world_sog_kn(&a.x.w);
        snprintf(d, sizeof d, "set up %s, CRUISE %s; 2 kn ahead at 90 deg: average %.2f kn, heading within %.1f deg; "
                 "3.5 kn -> %u, 1 kn -> %u, heading 120 -> %u: average %.2f kn, heading within %.1f deg, COG %.0f; "
                 "SPOT (no CONFIRM) -> %u %s, 60 s later %.2f kn, %.1f m off the spot; outboard max %.1f %%, gear off N %lu",
                 set ? "yes" : "NO", in ? "yes" : "NO", avg1, hdg1, r_fast, r_sp, r_hd, avg2, hdg2, cog, r_spot,
                 spot ? "SPOT" : "NOT SPOT", kn3, dp_true_err(&a.x), a.max_outboard, (unsigned long)a.gear_off);
        test_report("DP8 CRUISE: heading + speed (<= 3 kn) on the stern motor; SPEED / HEADING; SPOT at once",
                    set && in && fabsf(avg1 - 2.0f) < 0.3f && hdg1 < 4.0f && r_fast == PROTO_R_BAD_VALUE &&
                    r_sp == PROTO_R_OK && r_hd == PROTO_R_OK && fabsf(avg2 - 1.0f) < 0.25f && hdg2 < 4.0f &&
                    fabsf(dp_wrap180(cog - 120.0f)) < 10.0f && r_spot == PROTO_R_OK && spot && kn3 < 0.4f &&
                    dp_true_err(&a.x) < 15.0f && a.max_outboard <= 0.01f && a.gear_off == 0u && a.thrust_outside == 0u, d);
    }

    /* DP9 TRIP: the stored program at <= 3 kn, the navigation request's cap applies (as in AUTO), it ends in
     * SPOT at the last waypoint; a navigation heading in SPOT is NOT applied (alarm only); a new program is
     * refused while a TRIP runs */
    {
        static dpx_t a;
        static program_t p;
        bool set = dpx_init(&a, 2007u, true, NULL, true);
        prog_wp_t w[3];
        w[0] = wp_world(&a.x, 0.0, 120.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        w[1] = wp_world(&a.x, -100.0, 200.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        w[2] = wp_world(&a.x, -100.0, 320.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        prog_make(&p, w, 3, PROG_END_STOP);
        prog_error_t pe;
        uint8_t bad;
        bool up = sb_program_set(&a.x.sb, &p, &pe, &bad) == SB_OK;
        bool in = dpx_start(&a, DP_TRIP, 3.0f);
        dpx_judge(&a, 25000u);
        dpx_run(&a, 50000u);
        float avg1 = a.n_kn ? a.sum_kn / (float)a.n_kn : 0.0f;
        bool refused = sb_program_set(&a.x.sb, &p, &pe, &bad) == SB_ERR_WRONG_MODE;
        /* the gateway: speed cap 1.5 kn (shallow water ahead), lease 3 s, 2 Hz for 40 s */
        static navsim_t nv;
        memset(&nv, 0, sizeof nv);
        uint32_t t0 = a.x.t;
        float cap_min = 99.0f, cap_max = 0.0f;
        while (a.x.t < t0 + 40000u) {
            nv.seq++;
            nav_push(&nv, SB_NAV_CAP, 2u, 4u, 0.0f, 1.5f, 3000u);
            dpx_run(&a, 500u);
            if (a.x.t > t0 + 20000u) {
                float kn = world_sog_kn(&a.x.w);
                cap_min = fminf(cap_min, kn);
                cap_max = fmaxf(cap_max, kn);
            }
        }
        bool shallow = (a.x.sb.alarm_bits & SB_ALM_SHALLOW) != 0u;
        uint8_t wp_mid = a.x.sb.dp.wp;
        /* let the lease run out mid-trip: DRIVER DECISION, the lower of nav_lost_kn and the last cap */
        dpx_run(&a, 4000u);
        bool lost = a.x.sb.nav.lost && (a.x.sb.alarm_bits & SB_ALM_DECISION);
        /* the gateway back with "nothing" (the driver decided): the route again */
        nv.seq++;
        nav_push(&nv, SB_NAV_NONE, 0u, 0u, 0.0f, -1.0f, 3000u);
        dpx_run(&a, 500u);
        bool back = !a.x.sb.nav.lost;
        sb_alarm_ack(&a.x.sb);
        /* to the end (the lost lease keeps the slow speed: give it time) */
        uint32_t t1 = a.x.t;
        while (a.x.t < t1 + 600000u && !(a.x.sb.dp.sub == DP_SPOT && a.x.sb.dp.trip_done)) dpx_run(&a, 1000u);
        bool end = a.x.sb.dp.sub == DP_SPOT && a.x.sb.dp.trip_done && sb_mode(&a.x.sb) == MODE_DP;
        double ln, le;
        world_from_latlon(program_lat(&p.wp[2]), program_lon(&p.wp[2]), &ln, &le);
        float at_wp = (float)sqrt((a.x.w.north_m - ln) * (a.x.w.north_m - ln) + (a.x.w.east_m - le) * (a.x.w.east_m - le));
        dpx_run(&a, 30000u);
        float held = dp_true_err(&a.x);
        /* in SPOT: a navigation heading request is an alarm only */
        float tgt0 = a.x.sb.dp.hdg_target;
        uint32_t t2 = a.x.t;
        while (a.x.t < t2 + 5000u) {
            nv.seq++;
            nav_push(&nv, SB_NAV_HEADING, 1u, 1u, fmodf(tgt0 + 90.0f, 360.0f), 1.0f, 3000u);
            dpx_run(&a, 500u);
        }
        bool not_steered = fabsf(dp_wrap180(a.x.sb.dp.hdg_target - tgt0)) < 0.01f && (a.x.sb.alarm_bits & SB_ALM_COLLISION);
        snprintf(d, sizeof d, "set up %s, program %s, TRIP %s; 3 kn legs: average %.2f kn; new program while it runs refused %s; "
                 "navreq cap 1.5 kn: SOG %.2f .. %.2f kn, SHALLOW alarm %s (waypoint %u); lease lost: DRIVER DECISION %s, back %s; "
                 "the end: SPOT %s, %.1f m from the last waypoint, 30 s later %.1f m off the spot; a heading request in SPOT "
                 "not applied + COLLISION alarm %s; outboard max %.1f %%, gear off N %lu, steering on %lu",
                 set ? "yes" : "NO", up ? "ok" : "REFUSED", in ? "yes" : "NO", avg1, refused ? "yes" : "NO", cap_min, cap_max,
                 shallow ? "yes" : "NO", wp_mid, lost ? "yes" : "NO", back ? "yes" : "NO", end ? "yes" : "NO", at_wp, held,
                 not_steered ? "yes" : "NO", a.max_outboard, (unsigned long)a.gear_off, (unsigned long)a.steer_on);
        test_report("DP9 TRIP: the program at <= 3 kn, navreq cap / lost lease as in AUTO, ends in SPOT at the last "
                    "waypoint; navreq heading in SPOT = alarm only",
                    set && up && in && avg1 > 2.4f && avg1 < 3.3f && refused && cap_max < 1.9f && cap_min > 0.8f && shallow &&
                    lost && back && end && at_wp < 15.0f && held < 5.0f && not_steered && a.max_outboard <= 0.01f &&
                    a.gear_off == 0u && a.steer_on == 0u && a.thrust_outside == 0u, d);
    }

    /* DP10 the onboard joystick (CREWED) takes over: thrusters 0 in the same step, MANUAL on the outboard (the
     * existing takeover: alarm JOYSTICK, chirp, the stick steers); TAKE too */
    {
        int ok = 0, len = 0;
        char t[700] = "";
        for (int k = 0; k < 2; k++) {
            static dpx_t a;
            bool set = dpx_init(&a, 2008u + (uint32_t)k, true, NULL, true);
            a.x.w.current_e = -0.2f;
            bool in = dpx_start(&a, DP_SPOT, 0.0f);
            dpx_run(&a, 30000u);
            float b0, s0;
            dpx_thr(&b0, &s0);
            uint32_t t0 = a.x.t;
            if (k == 0) joy_set(0.6f, 0.0f, false, false);         /* X deflected */
            else joy_set(0.0f, 0.0f, true, false);                  /* TAKE */
            while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + 1000u) dpx_run(&a, AP_PERIOD_MS);
            float b1, s1;
            dpx_thr(&b1, &s1);
            uint32_t dt = a.x.t - t0;
            bool manual = sb_mode(&a.x.sb) == MODE_MANUAL && b1 == 0.0f && s1 == 0.0f && (a.x.sb.alarm_bits & SB_ALM_JOYSTICK) &&
                          a.x.sb.dp.stop_why == DP_STOP_JOYSTICK && !a.x.sb.thrust.active;
            joy_set(k == 0 ? 0.6f : 0.0f, 0.0f, false, false);
            dpx_run(&a, 3000u);
            bool steers = k == 1 || fabsf(a.x.w.rud.angle_deg) > 3.0f;
            bool good = set && in && (b0 != 0.0f || s0 != 0.0f) && manual && dt <= 300u && steers && a.thrust_outside == 0u;
            ok += good;
            len += snprintf(t + len, sizeof t - (size_t)len, "%s%s: thrusting %.0f / %.0f, MANUAL after %lu ms with thrusters "
                            "%.0f / %.0f %s, outboard %.1f deg", k ? "; " : "", k == 0 ? "stick X" : "TAKE", b0, s0,
                            (unsigned long)dt, b1, s1, manual ? "ok" : "FAILED", a.x.w.rud.angle_deg);
            joy_set(0.0f, 0.0f, false, false);
        }
        test_report("DP10 CREWED joystick (X / TAKE) in DP -> thrusters 0 in the same step + MANUAL on the outboard",
                    ok == 2, t);
    }

    /* DP11 GPS lost / suspect (jump) in DP: thrusters 0 + alarm + STANDBY (no dead reckoning in DP) */
    {
        int ok = 0, len = 0;
        char t[600] = "";
        for (int k = 0; k < 2; k++) {
            static dpx_t a;
            bool set = dpx_init(&a, 2010u + (uint32_t)k, true, NULL, true);
            a.x.w.current_e = -0.2f;
            bool in = dpx_start(&a, DP_SPOT, 0.0f);
            dpx_run(&a, 20000u);
            uint32_t t0 = a.x.t;
            if (k == 0) a.x.w.gps_fail = true;
            else a.x.w.gps_off_n += 300.0;                           /* a jump: the integrity check rejects it */
            while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + 8000u) dpx_run(&a, AP_PERIOD_MS);
            float b, s;
            dpx_thr(&b, &s);
            uint32_t why = a.x.sb.alarm_bits & (SB_ALM_GPS_LOST | SB_ALM_GPS_SUSPECT);
            bool good = set && in && sb_mode(&a.x.sb) == MODE_STANDBY && b == 0.0f && s == 0.0f && why &&
                        a.x.sb.dp.stop_why == DP_STOP_GPS && a.dp_no_gps == 0u && a.thrust_outside == 0u;
            ok += good;
            len += snprintf(t + len, sizeof t - (size_t)len, "%s%s: STANDBY after %lu ms, thrusters %.0f / %.0f, alarm 0x%lx, "
                            "DP steps without a trusted GPS %lu %s", k ? "; " : "", k == 0 ? "GPS lost" : "GPS jump 300 m",
                            (unsigned long)(a.x.t - t0), b, s, (unsigned long)why, (unsigned long)a.dp_no_gps,
                            good ? "ok" : "FAILED");
            a.x.w.gps_fail = false;
        }
        test_report("DP11 GPS lost / suspect in DP -> thrusters 0 + alarm + STANDBY (never a DP step without a trusted GPS)",
                    ok == 2, t);
    }

    /* DP12 off the spot: pushed beyond spot_alarm_m = DP OFF SPOT (advisory: DP keeps trying, the power limit
     * stays); beyond 50 m -> STANDBY (thrusters 0); off it for 2 min -> STANDBY too */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2012u, true, NULL, true);
        power_full(&a.x);
        float pf0 = a.x.sb.plim.fwd_pct;
        a.x.w.current_n = -0.6f;                                 /* heading 90: 1.2 kn from the port side */
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        uint32_t t0 = a.x.t, t_alarm = 0u;
        while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + 400000u) {
            dpx_run(&a, 200u);
            if (!t_alarm && (a.x.sb.alarm_bits2 & SB_ALM2_DP_OFFSPOT)) t_alarm = a.x.t;
        }
        bool advisory = t_alarm != 0u && a.x.sb.plim.fwd_pct == pf0;
        float err_end = dp_true_err(&a.x);
        float b, s;
        dpx_thr(&b, &s);
        bool stop50 = sb_mode(&a.x.sb) == MODE_STANDBY && a.x.sb.dp.stop_why == DP_STOP_OFFSPOT && b == 0.0f && s == 0.0f;
        uint32_t t_stop = a.x.t - t0;
        /* 2 min rule: a push that keeps the boat between 15 and 50 m */
        static dpx_t c;
        bool set2 = dpx_init(&c, 2013u, true, NULL, true);
        (void)dpx_settings(&c, 100u, 1800u, 600u, 100u, 1800u, 600u, 5u);   /* spot_alarm_m 5 m */
        c.x.w.current_n = -0.05f;                                 /* a gentle side push: some metres off */
        bool in2 = dpx_start(&c, DP_SPOT, 0.0f);
        uint32_t t1 = c.x.t, t_off = 0u;
        float max2 = 0.0f;
        bool was_off = false;
        while (sb_mode(&c.x.sb) == MODE_DP && c.x.t < t1 + 400000u) {
            dpx_run(&c, 200u);
            if (c.x.sb.dp.offspot && !was_off) t_off = c.x.t;            /* (the latest onset: noise may reset it) */
            was_off = c.x.sb.dp.offspot;
            max2 = fmaxf(max2, dp_true_err(&c.x));
        }
        bool stop2 = sb_mode(&c.x.sb) == MODE_STANDBY && c.x.sb.dp.stop_why == DP_STOP_OFFSPOT && t_off &&
                     c.x.t - t_off >= DP_OFF_STOP_MS - 1000u && c.x.t - t_off <= DP_OFF_STOP_MS + 5000u && max2 < 50.0f;
        snprintf(d, sizeof d, "set up %s / %s; 1.2 kn from the side: DP %s, OFF SPOT after %.0f s (advisory, power limit "
                 "%.0f %% kept %s), STANDBY after %.0f s at %.1f m (thrusters %.0f / %.0f) %s; spot alarm 5 m, a gentle push "
                 "(DP %s, max %.1f m): OFF SPOT at %.0f s, STANDBY %.0f s later %s",
                 set ? "yes" : "NO", set2 ? "yes" : "NO", in ? "yes" : "NO", t_alarm ? (t_alarm - t0) * 0.001 : -1.0, pf0,
                 advisory ? "yes" : "NO", t_stop * 0.001, err_end, b, s, stop50 ? "ok" : "FAILED", in2 ? "yes" : "NO", max2,
                 t_off ? (t_off - t1) * 0.001 : -1.0, t_off ? (c.x.t - t_off) * 0.001 : -1.0, stop2 ? "ok" : "FAILED");
        test_report("DP12 off the spot: DP OFF SPOT advisory beyond spot_alarm_m; > 50 m or 2 min -> thrusters 0, STANDBY",
                    set && in && advisory && stop50 && err_end > 49.0f && set2 && in2 && stop2 && a.thrust_outside == 0u &&
                    c.thrust_outside == 0u, d);
    }

    /* DP13 a thruster HOT in DP (heat budget used up) -> DP ends: thrusters 0, STANDBY, DP THRUSTER HOT; both
     * thrusters at their limit for 10 s -> DP OVERPOWER (advisory, DP goes on) */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2014u, true, NULL, true);
        (void)dpx_settings(&a, 100u, 1800u, 600u, 100u, 40u, 40u, 15u);      /* stern: 40 s budget */
        a.x.w.current_e = -0.3f;                                  /* the stern works all the time */
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        uint32_t t0 = a.x.t;
        while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + 120000u) dpx_run(&a, AP_PERIOD_MS);
        float b, s;
        dpx_thr(&b, &s);
        bool hot = sb_mode(&a.x.sb) == MODE_STANDBY && a.x.sb.thrust.hot[THR_STERN] && b == 0.0f && s == 0.0f &&
                   (a.x.sb.alarm_bits2 & SB_ALM2_DP_HEAT) && a.x.sb.dp.stop_why == DP_STOP_THRUSTER && a.hot_on == 0u;
        uint32_t t_hot = a.x.t - t0;
        /* a hot thruster: no new DP */
        sb_alarm_ack(&a.x.sb);
        uint8_t r_hot = dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0);
        /* overpower: 30 % caps, a current the stern can't hold + a big heading change for the bow */
        static dpx_t c;
        bool set2 = dpx_init(&c, 2015u, true, NULL, true);
        (void)dpx_settings(&c, 30u, 1800u, 600u, 30u, 1800u, 600u, 100u);
        c.x.w.current_e = -0.6f;
        bool in2 = dpx_start(&c, DP_SPOT, 0.0f);
        dpx_run(&c, 5000u);
        uint8_t r_h = dpx_cmd(&c, DP_ACT_HEADING, 0u, 2700);
        uint32_t t1 = c.x.t, t_op = 0u;
        while (sb_mode(&c.x.sb) == MODE_DP && c.x.t < t1 + 60000u && !t_op) {
            dpx_run(&c, 200u);
            if (c.x.sb.alarm_bits2 & SB_ALM2_DP_OVERPOWER) t_op = c.x.t;
        }
        bool op = t_op && sb_mode(&c.x.sb) == MODE_DP && t_op - t1 >= DP_OVERPOWER_MS;
        snprintf(d, sizeof d, "set up %s, DP %s: stern budget 40 s in a current -> STANDBY after %.1f s, thrusters %.0f / %.0f, "
                 "DP THRUSTER HOT %s; while hot DP START -> %u; 30 %% caps + 1.2 kn current + heading 270 (-> %u, DP %s / %s): "
                 "OVERPOWER after %.1f s, still DP %s",
                 set ? "yes" : "NO", in ? "yes" : "NO", t_hot * 0.001, b, s, hot ? "ok" : "FAILED", r_hot, r_h,
                 set2 ? "yes" : "NO", in2 ? "yes" : "NO", t_op ? (t_op - t1) * 0.001 : -1.0, op ? "yes" : "NO");
        test_report("DP13 a thruster HOT in DP -> thrusters 0 + STANDBY + DP THRUSTER HOT (no new DP while hot); both "
                    "saturated 10 s -> DP OVERPOWER (advisory)",
                    set && in && hot && t_hot >= 35000u && r_hot == PROTO_R_PRECOND && set2 && in2 && r_h == PROTO_R_OK && op &&
                    a.thrust_outside == 0u && c.thrust_outside == 0u, d);
    }

    /* DP14 every other way out: thrusters 0 FIRST (the same step), then STANDBY - the STANDBY button, the e-stop, DP
     * STOP (also while a request waits), compass lost (gyro hold, STANDBY after 10 s; no gyro: at once), UNCREWED
     * link lost; the generator may run in SPOT (neutral confirmed, gear N) and stops for CRUISE; the stern fighting a
     * forward push = GEAR FAULT, DP ends */
    {
        int ok = 0, n = 0, len = 0;
        char t[1500] = "";
        for (int k = 0; k < 6; k++) {
            static dpx_t a;
            bool set = dpx_init(&a, 2016u + (uint32_t)k, k != 5, NULL, true);
            a.x.w.current_e = -0.2f;
            if (k == 5) {                                        /* UNCREWED: the Pi tells "open sea" */
                a.r.p.shore_on = true;
                a.r.p.shore_flags = SB_SHORE_VALID;
                a.r.p.shore_m = 900u;
            }
            bool in = dpx_start(&a, DP_SPOT, 0.0f);
            dpx_run(&a, 20000u);
            uint32_t t0 = a.x.t;
            const char *what = "?";
            uint8_t want = DP_STOP_STANDBY;
            uint32_t limit = 200u;
            switch (k) {
            case 0: what = "STANDBY button"; sim_hal_press(HAL_BTN_STANDBY); break;
            case 1: what = "e-stop"; sim_hal_set_estop(true, true); break;
            case 2: what = "DP STOP"; (void)dpx_cmd(&a, DP_ACT_STOP, 0u, 0); want = DP_STOP_CMD; t0 = a.x.t - 200u; limit = 400u; break;
            case 3: what = "compass lost (gyro hold 10 s)"; sim_hal_fail_heading(true); want = DP_STOP_COMPASS;
                    limit = DP_COMPASS_MS + 3000u; break;          /* (+ the compass timeout of the steering core) */
            case 4: what = "compass + gyro lost"; sim_hal_fail_heading(true); sim_hal_set_gyro(false, 0.0f, 0.0f);
                    want = DP_STOP_COMPASS; limit = 3000u; break;
            default: what = "UNCREWED link lost"; a.r.p.pi_on = false; a.r.rj_on = false; want = DP_STOP_STANDBY;
                     limit = 5000u; break;
            }
            float hdg0 = a.x.w.heading_deg, hdg_max = 0.0f;
            while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + limit + 1000u) {
                dpx_run(&a, AP_PERIOD_MS);
                hdg_max = fmaxf(hdg_max, fabsf(dp_wrap180(a.x.w.heading_deg - hdg0)));
            }
            float b, s;
            dpx_thr(&b, &s);
            uint32_t dt = a.x.t - t0;
            bool good = set && in && sb_mode(&a.x.sb) == MODE_STANDBY && b == 0.0f && s == 0.0f && dt <= limit + 60u &&
                        a.x.sb.dp.stop_why == want && a.thrust_outside == 0u;
            if (k == 3) good = good && dt >= DP_COMPASS_MS - 100u && hdg_max < 5.0f;    /* the gyro held it meanwhile */
            if (k == 5) good = good && a.x.sb.profile == SB_PROFILE_UNCREWED && a.x.sb.ll.state != SB_LL_OK;
            ok += good;
            n++;
            len += snprintf(t + len, sizeof t - (size_t)len, "%s%s: STANDBY after %lu ms (why %u), thrusters %.0f / %.0f%s %s",
                            k ? "; " : "", what, (unsigned long)dt, a.x.sb.dp.stop_why, b, s,
                            k == 3 ? (hdg_max < 5.0f ? ", heading held on the gyro" : ", HEADING NOT HELD") : "",
                            good ? "ok" : "FAILED");
            sim_hal_set_estop(false, false);
            sim_hal_fail_heading(false);
            sim_hal_set_gyro(true, 0.05f, 0.02f);
        }
        /* DP STOP while a DP request waits for its CONFIRM: cancelled */
        {
            static dpx_t a;
            bool set = dpx_init(&a, 2022u, true, NULL, true);
            uint8_t r1 = dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0);
            uint8_t r2 = dpx_cmd(&a, DP_ACT_STOP, 0u, 0);
            bool late = onboard_confirm(&a.x, &a.r);
            bool good = set && r1 == PROTO_R_OK && r2 == PROTO_R_OK && !late && sb_mode(&a.x.sb) == MODE_STANDBY;
            ok += good;
            n++;
            len += snprintf(t + len, sizeof t - (size_t)len, "; DP STOP cancels a waiting request %s", good ? "ok" : "FAILED");
        }
        test_report("DP14 STANDBY button / e-stop / DP STOP / compass lost (gyro 10 s) / UNCREWED link lost: thrusters 0 "
                    "first, then STANDBY", ok == n, t);
    }

    /* GN9 the generator in DP (section K): it may run while DP holds the spot (neutral confirmed by the switch, gear
     * N, the cap), stops idle-first when CRUISE starts; the stern fighting an unexplained forward push (gear in?) =
     * GEAR FAULT: the generator idles, DP ends (a fault), generator auto off */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2023u, true, NULL, true);
        a.x.w.neutral_switch = true;
        a.x.w.neutral_sw_force = 0;
        sim_hal_set_supply(true, 11.5f);
        dpx_run(&a, 2000u);
        sb_alarm_ack(&a.x.sb);
        bool on = sb_generator_set(&a.x.sb, true) == SB_OK;
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        uint32_t t0 = a.x.t;
        while (a.x.sb.gen.state != GEN_RUNNING && a.x.t < t0 + 90000u) dpx_run(&a, 200u);
        dpx_run(&a, 3000u);
        bool runs = a.x.sb.gen.state == GEN_RUNNING && sb_mode(&a.x.sb) == MODE_DP && gear_is(&a.x.sb.gear, GEAR_N) &&
                    sim_hal_throttle() > 1.0f && sim_hal_throttle() <= fminf(a.x.sb.gen.cap_pct, a.x.sb.plim.fwd_pct) + 0.01f &&
                    a.gear_off == 0u;
        float thr_run = sim_hal_throttle();
        /* CRUISE asked: a DP cruise start = a stop request (idle first) */
        uint8_t r_cr = dpx_cmd(&a, DP_ACT_START, DP_CRUISE, 100);
        dpx_run(&a, 200u);
        bool stopping = a.x.sb.gen.state == GEN_STOPPING || a.x.sb.gen.state == GEN_READY;
        dpx_confirm(&a);
        dpx_run(&a, 5000u);
        bool idle_cr = a.x.sb.dp.sub == DP_CRUISE && sim_hal_throttle() <= 0.6f && !a.x.sb.gen.active;
        /* back to SPOT: it may start again; then the gearbox slips into forward (the switch still says N) */
        (void)dpx_cmd(&a, DP_ACT_START, DP_SPOT, 0);
        t0 = a.x.t;
        while (a.x.sb.gen.state != GEN_RUNNING && a.x.t < t0 + 40000u) dpx_run(&a, 200u);
        bool again = a.x.sb.gen.state == GEN_RUNNING;
        a.x.w.neutral_sw_force = 2;                               /* the switch stuck "neutral" ... */
        a.x.w.servo_stuck = true;
        a.x.w.gear_pos_us = 1900.0f;                              /* ... the gearbox in forward: the boat is pushed ahead */
        a.x.w.engaged = GEAR_F;
        t0 = a.x.t;
        while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + 60000u) dpx_run(&a, AP_PERIOD_MS);
        float b, s;
        dpx_thr(&b, &s);
        bool fault = sb_mode(&a.x.sb) == MODE_STANDBY && a.x.sb.gen.stop_why == GEN_STOP_DP_FIGHT &&
                     (a.x.sb.alarm_bits & SB_ALM_GEAR) && !a.x.sb.gen.enabled && b == 0.0f && s == 0.0f;
        dpx_run(&a, 3000u);
        bool idle = sim_hal_throttle() <= 0.6f;
        snprintf(d, sizeof d, "set up %s, generator on %s, DP %s; in SPOT the generator runs at %.1f %% in neutral %s; "
                 "CRUISE asked (-> %u): stop begins %s, in CRUISE idle %s; back in SPOT it runs again %s; the gearbox slips "
                 "into forward: GEAR FAULT after %.1f s, DP ended, thrusters %.0f / %.0f, auto off %s, idle %s",
                 set ? "yes" : "NO", on ? "yes" : "NO", in ? "yes" : "NO", thr_run, runs ? "ok" : "FAILED", r_cr,
                 stopping ? "yes" : "NO", idle_cr ? "yes" : "NO", again ? "yes" : "NO", (a.x.t - t0) * 0.001 - 3.0, b, s,
                 fault ? "yes" : "NO", idle ? "yes" : "NO");
        test_report("GN9 generator in DP: runs while SPOT holds (neutral confirmed), stops idle-first for CRUISE; the stern "
                    "fighting a forward push = GEAR FAULT, DP ends",
                    set && on && in && runs && r_cr == PROTO_R_OK && stopping && idle_cr && again && fault && idle, d);
        a.x.w.servo_stuck = false;
        sim_hal_set_supply(true, 12.6f);
    }

    /* DP15 (step 11b) a thruster settings push keeps the heat budget: a HOT thruster stays HOT for its own rest
     * time (the same settings again, settings without it and with it again, a shorter rest_s) - never cleared
     * early; a part-used budget is kept too (not reset by a push without the thruster) */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2024u, true, NULL, false);
        bool s1 = dpx_settings(&a, 50u, 10u, 20u, 50u, 60u, 30u, 15u) == PROTO_R_OK;
        sb_test_thrust(&a.x.sb, true, 50.0f, 50.0f);
        uint32_t t0 = a.x.t;
        while (!a.x.sb.thrust.hot[THR_BOW] && a.x.t < t0 + 15000u) dpx_run(&a, AP_PERIOD_MS);
        uint32_t t_hot = a.x.t;
        sb_test_thrust(&a.x.sb, false, 0.0f, 0.0f);
        dpx_run(&a, 200u);
        /* re-pushes while it rests (20 s): same, without both thrusters, with them again, then a 1 s rest_s */
        uint8_t r_same = dpx_settings(&a, 50u, 10u, 20u, 50u, 60u, 30u, 15u);
        dpx_run(&a, 1000u);
        bool hot1 = a.x.sb.thrust.hot[THR_BOW];
        uint8_t r_none = dpx_settings(&a, 0u, 0u, 0u, 0u, 0u, 0u, 15u);
        float stern0 = a.x.sb.thrust.heat_s[THR_STERN];
        dpx_run(&a, 1000u);
        bool hot2 = a.x.sb.thrust.hot[THR_BOW];
        float stern1 = a.x.sb.thrust.heat_s[THR_STERN];
        uint8_t r_back = dpx_settings(&a, 50u, 10u, 20u, 50u, 60u, 30u, 15u);
        dpx_run(&a, 1000u);
        bool hot3 = a.x.sb.thrust.hot[THR_BOW];
        uint8_t r_short = dpx_settings(&a, 50u, 10u, 1u, 50u, 60u, 30u, 15u);
        dpx_run(&a, 3000u);
        bool hot4 = a.x.sb.thrust.hot[THR_BOW] && (a.x.sb.alarm_bits2 & SB_ALM2_DP_HEAT);
        uint32_t t_cool = 0u;
        while (!t_cool && a.x.t < t_hot + 30000u) {
            dpx_run(&a, AP_PERIOD_MS);
            if (!a.x.sb.thrust.hot[THR_BOW]) t_cool = a.x.t - t_hot;
        }
        bool rest_kept = t_cool >= 19500u && t_cool <= 20800u;
        snprintf(d, sizeof d, "set up %s / %s; bow HOT after %.1f s; settings again %u: HOT %s; without thrusters %u: HOT %s, "
                 "stern budget %.1f -> %.1f s kept; with them again %u: HOT %s; rest 1 s %u: HOT + alarm 3 s later %s; "
                 "cooled %.1f s after it went HOT (its own rest 20 s) %s",
                 set ? "yes" : "NO", s1 ? "yes" : "NO", (t_hot - t0) * 0.001, r_same, hot1 ? "yes" : "NO", r_none,
                 hot2 ? "yes" : "NO", stern0, stern1, r_back, hot3 ? "yes" : "NO", r_short, hot4 ? "yes" : "NO",
                 t_cool * 0.001, rest_kept ? "yes" : "NO");
        test_report("DP15 a thruster settings push keeps heat_s / HOT (same, without, with again, shorter rest): never "
                    "cleared early",
                    set && s1 && r_same == PROTO_R_OK && hot1 && r_none == PROTO_R_OK && hot2 && stern0 > 5.0f &&
                    fabsf(stern1 - stern0) < 0.01f && r_back == PROTO_R_OK && hot3 && r_short == PROTO_R_OK && hot4 &&
                    rest_kept && a.hot_on == 0u, d);
    }

    /* DP16 (step 11b) a thruster going HOT in DP: DP ends AND both thrusters are 0 in that very step (the other
     * one never runs on for a step) */
    {
        static dpx_t a;
        bool set = dpx_init(&a, 2025u, true, NULL, true);
        (void)dpx_settings(&a, 100u, 1800u, 600u, 100u, 30u, 30u, 15u);      /* stern: 30 s budget */
        a.x.w.current_e = -0.3f;                                  /* the stern works all the time ... */
        a.x.w.current_n = -0.15f;                                 /* ... the bow too (a side push) */
        bool in = dpx_start(&a, DP_SPOT, 0.0f);
        uint32_t t0 = a.x.t;
        float b_prev = 0.0f, b = 0.0f, s = 0.0f;
        bool hot = false;
        while (sb_mode(&a.x.sb) == MODE_DP && a.x.t < t0 + 90000u) {
            float pb, ps;
            dpx_thr(&pb, &ps);
            b_prev = pb;
            /* one control step */
            uint32_t n0 = a.steps;
            while (a.steps == n0) sbsim_run(&a.x, a.x.t, dpx_hook, &a);
            if (a.x.sb.thrust.hot[THR_STERN]) { hot = true; break; }
        }
        dpx_thr(&b, &s);
        bool same = hot && b == 0.0f && s == 0.0f && sb_mode(&a.x.sb) == MODE_STANDBY && !a.x.sb.thrust.active &&
                    a.x.sb.dp.stop_why == DP_STOP_THRUSTER;
        snprintf(d, sizeof d, "set up %s, DP %s; stern HOT after %.1f s (bow running %.1f %% the step before): in that step "
                 "thrusters %.1f / %.1f, STANDBY %s",
                 set ? "yes" : "NO", in ? "yes" : "NO", (a.x.t - t0) * 0.001, b_prev, b, s, same ? "yes" : "NO");
        test_report("DP16 a thruster HOT in DP: DP ends and BOTH thrusters 0 in the same step",
                    set && in && hot && fabsf(b_prev) > 0.5f && same && a.hot_on == 0u, d);
    }

    /* DP17 (step 13b review H1) a DP TRIP after an AUTO RETURN (the way back still latched in RAM): the trip drives the
     * STORED program (the one the gateway checked), from its own copy; an AUTO START CONFIRM during the trip (refused:
     * DP -> AUTO is not allowed) changes nothing - the way back, the trip's route and waypoint stay */
    {
        static dpx_t a;
        static program_t p, back, back_keep, route0;
        bool set = dpx_init(&a, 2031u, true, NULL, true);
        prog_wp_t w[3], b[2];
        w[0] = wp_world(&a.x, 0.0, 120.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        w[1] = wp_world(&a.x, -100.0, 200.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        w[2] = wp_world(&a.x, -100.0, 320.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        prog_make(&p, w, 3, PROG_END_STOP);
        b[0] = wp_world(&a.x, 250.0, -150.0, 3.0f, 10.0f, WP_ACT_CONTINUE);     /* a way back somewhere else */
        b[1] = wp_world(&a.x, 400.0, -150.0, 3.0f, 10.0f, WP_ACT_CONTINUE);
        prog_make(&back, b, 2, PROG_END_STOP);
        prog_error_t pe;
        uint8_t bad;
        bool up = sb_program_set(&a.x.sb, &p, &pe, &bad) == SB_OK;
        /* as after a finished AUTO RETURN: the stored program in orig, the way back active in RAM */
        a.x.sb.autop.orig = a.x.sb.autop.prog;
        a.x.sb.autop.prog = back;
        a.x.sb.autop.returning = true;
        a.x.sb.autop.ret_base = 1u;
        back_keep = back;
        bool in = dpx_start(&a, DP_TRIP, 3.0f);
        route0 = a.x.sb.dp.route;
        bool route_ok = memcmp(&route0, &p, sizeof p) == 0;
        /* drive: the first waypoint reached must be the stored one (120 m east), never the way back's */
        double n0, e0, nb, eb;
        world_from_latlon(program_lat(&p.wp[0]), program_lon(&p.wp[0]), &n0, &e0);
        world_from_latlon(program_lat(&back.wp[0]), program_lon(&back.wp[0]), &nb, &eb);
        uint32_t t0 = a.x.t;
        float at0 = 1e9f;
        while (a.x.t < t0 + 300000u && sb_mode(&a.x.sb) == MODE_DP && a.x.sb.dp.wp == 0u) dpx_run(&a, 500u);
        at0 = (float)sqrt((a.x.w.north_m - n0) * (a.x.w.north_m - n0) + (a.x.w.east_m - e0) * (a.x.w.east_m - e0));
        float atb = (float)sqrt((a.x.w.north_m - nb) * (a.x.w.north_m - nb) + (a.x.w.east_m - eb) * (a.x.w.east_m - eb));
        bool reached = sb_mode(&a.x.sb) == MODE_DP && a.x.sb.dp.wp == 1u && at0 < 25.0f;
        /* AUTO START + CONFIRM during the trip: refused, nothing changes */
        uint8_t wp_before = a.x.sb.dp.wp;
        sb_result_t rs = sb_auto_start(&a.x.sb);
        if (rs == SB_OK) { dpx_run(&a, 300u); dpx_confirm(&a); }
        dpx_run(&a, 300u);
        bool kept = sb_mode(&a.x.sb) == MODE_DP && a.x.sb.dp.sub == DP_TRIP && a.x.sb.autop.returning &&
                    memcmp(&a.x.sb.autop.prog, &back_keep, sizeof back_keep) == 0 &&
                    memcmp(&a.x.sb.dp.route, &route0, sizeof route0) == 0 && a.x.sb.dp.wp >= wp_before;
        snprintf(d, sizeof d, "set up %s, program %s, way back latched, TRIP %s; the trip's route = the stored program %s; "
                 "first waypoint reached %s after %.0f s: %.1f m from the stored waypoint 1 (%.0f m from the way back's); "
                 "AUTO START %d + CONFIRM in DP: still DP TRIP with the way back + route kept %s",
                 set ? "yes" : "NO", up ? "ok" : "REFUSED", in ? "yes" : "NO", route_ok ? "yes" : "NO",
                 reached ? "yes" : "NO", (a.x.t - t0) * 0.001, at0, atb, (int)rs, kept ? "yes" : "NO");
        test_report("DP17 DP TRIP after an AUTO RETURN: the stored program (own copy); AUTO START CONFIRM in DP changes nothing",
                    set && up && in && route_ok && reached && kept, d);
    }
}


/* ================================================================ DroneCAN (CN) ==
 * Step 11d (CLAUDE.md T, docs/dronecan_design.md sections 4 + 5): the real hal/dronecan.c + libcanard
 * on the simulated bus (hal_sim.c). The servos are NOT on CAN yet (11e): nothing is commanded over
 * CAN by the app; the node table / bus faults give the alarms + the STANDBY interlock. */

static mode_reason_t cn_why(const sb_t *s)          /* the reason of the last accepted mode change */
{
    uint32_t kept = s->modes.log_count < MODE_LOG_N ? s->modes.log_count : MODE_LOG_N;
    for (uint32_t k = kept; k > 0u; k--) {
        const mode_log_entry_t *e = mode_log_get(&s->modes, k - 1u);
        if (e && e->verdict == MODE_OK) return e->why;
    }
    return WHY_COUNT;
}

static uint8_t cn_p(const pcsim_t *p, uint32_t i)     /* TELEM2 group P byte i (P = the last 6 bytes) */
{
    return p->t2_len >= T2_P ? p->t2[p->t2_len - T2_P + i] : 0xEEu;     /* (P = 6 bytes before Q) */
}

/* a boat with the simulated CAN bus fitted (every node powered, OK, OPERATIONAL), armed (CREWED) */
static void cn_setup(pcsim_t *p, uint32_t seed, uint32_t fault)
{
    s_can_on = true;
    s_can_fault = fault;
    pcsim_init(p, seed);
    s_can_on = false;
    s_can_fault = 0u;
    pc_run(p, 8000u);
}

static bool cn_hdg(pcsim_t *p)                       /* HDG LOCK on the panel */
{
    sim_hal_press(HAL_BTN_HDG_LOCK);
    pc_run(p, 200u);
    return sb_mode(&p->x.sb) == MODE_HDG_LOCK;
}

static void can_tests(void)
{
    char d[900];
    printf("\n=== CN DroneCAN bus (dronecan.c + libcanard on the simulated bus): heartbeats, node table, "
           "CAN NODE LOST / CAN BUS FAULT -> STANDBY ===\n");

    /* CN1 healthy bus: our NodeStatus 1 Hz on the bus, every node seen, no alarm, modes work; step 11e: the
     * servos ARE commanded over CAN (ArrayCommand), still no hardpoint (the relay board is 11f); not fitted =
     * nothing at all */
    {
        static pcsim_t p;
        cn_setup(&p, 1101u, 0u);
        uint32_t per = sim_hal_can_status_period_ms();
        bool seen = p.x.sb.can.fitted && p.x.sb.can.seen_mask == 0x0Fu && p.x.sb.can.lost_mask == 0u &&
                    !p.x.sb.can.node_block && !p.x.sb.can.bus_bad;
        bool quiet = p.x.sb.alarm_bits2 == 0u && p.x.sb.armed;
        bool hdg = cn_hdg(&p);
        pc_run(&p, 3000u);
        uint32_t acmd = sim_hal_can_bus_frames(1u), hp = sim_hal_can_bus_frames(2u), st = sim_hal_can_bus_frames(0u);
        bool t2 = cn_p(&p, 0) == 1u && cn_p(&p, 1) == HAL_CAN_ST_RUN && cn_p(&p, 4) == 0x0Fu;
        uint32_t nst = sim_hal_can_node_statuses(DC_NODE_STEERING);
        static pcsim_t q;
        bool keep_all = s_can_all;
        s_can_all = false;                             /* (also in the "all --can" run) */
        pcsim_init(&q, 1102u);                         /* default: no CAN fitted (like HAL_CAN_FITTED 0) */
        s_can_all = keep_all;
        pc_run(&q, 8000u);
        bool none = !q.x.sb.can.fitted && q.x.sb.can.seen_mask == 0u && q.x.sb.alarm_bits2 == 0u &&
                    sim_hal_can_bus_frames(0u) == 0u && q.t2_len >= T2_P && cn_p(&q, 0) == 0u && q.can_frames == 0;
        snprintf(d, sizeof d, "our NodeStatus every %u ms (%u on the bus), node statuses (steering) %u, all seen + none "
                 "lost %s, no alarm + armed %s, HDG LOCK %s, ArrayCommand %u (servos, 11e) / hardpoint %u frames (want 0), TELEM2 P %s, "
                 "CAN_NODES frames %d; not fitted: nothing %s",
                 (unsigned)per, (unsigned)st, (unsigned)nst, seen ? "yes" : "NO", quiet ? "yes" : "NO", hdg ? "yes" : "NO",
                 (unsigned)acmd, (unsigned)hp, t2 ? "ok" : "WRONG", p.can_frames, none ? "yes" : "NO");
        test_report("CN1 healthy bus: heartbeats both ways, no alarm, modes work, the servos commanded over CAN (11e), "
                    "no relay command (11f); not fitted = nothing", per == 1000u && st >= 10u && nst >= 10u && seen &&
                    quiet && hdg && acmd > 0u && hp == 0u && t2 && p.can_frames >= 8 && none, d);
    }

    /* CN2 a required node goes silent while driving (HDG LOCK): CAN NODE LOST after 1.5 s, STANDBY, the
     * power limit at the startup values, no mode but STANDBY while lost; back = the interlock goes, the
     * acknowledge clears the alarm */
    {
        static pcsim_t p;
        cn_setup(&p, 1103u, 0u);
        bool hdg = cn_hdg(&p);
        uint32_t t0 = p.x.t;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        while (sb_mode(&p.x.sb) == MODE_HDG_LOCK && p.x.t < t0 + 5000u) pc_run(&p, AP_PERIOD_MS);
        uint32_t dt = p.x.t - t0;
        bool alarm = (p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) && p.x.sb.alarm && sb_mode(&p.x.sb) == MODE_STANDBY &&
                     cn_why(&p.x.sb) == WHY_CAN_NODE && p.x.sb.can.node[HAL_CAN_ROLE_THROTTLE].why == HAL_CAN_SILENT;
        bool plim = p.x.sb.plim.fwd_pct == 30.0f;
        pc_run(&p, 1000u);
        bool refused = !cn_hdg(&p);
        sb_alarm_ack(&p.x.sb);
        bool kept = (p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) != 0u;          /* still lost: the ack can't clear it */
        bool t2 = (cn_p(&p, 0) & 32u) && (cn_p(&p, 2) & 1u);
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        pc_run(&p, 1500u);
        bool back = !p.x.sb.can.node_lost && !p.x.sb.can.node_block;
        sb_alarm_ack(&p.x.sb);
        pc_run(&p, 100u);
        bool cleared = !(p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE);
        bool hdg2 = cn_hdg(&p);
        snprintf(d, sizeof d, "HDG LOCK %s; throttle node silent: STANDBY after %u ms (CAN NODE LOST, why CAN node, "
                 "SILENT) %s, power limit 30 %% %s, HDG LOCK refused %s, ack keeps it %s, TELEM2 P %s; back: interlock "
                 "off %s, ack clears %s, HDG LOCK %s",
                 hdg ? "yes" : "NO", (unsigned)dt, alarm ? "yes" : "NO", plim ? "yes" : "NO", refused ? "yes" : "NO",
                 kept ? "yes" : "NO", t2 ? "ok" : "WRONG", back ? "yes" : "NO", cleared ? "yes" : "NO", hdg2 ? "yes" : "NO");
        test_report("CN2 required node silent: CAN NODE LOST 1.5 s after its last NodeStatus -> STANDBY, power limit "
                    "down, STANDBY-only interlock, ack only when back", hdg && dt >= 400u && dt <= 1700u && alarm && plim && refused &&
                    kept && t2 && back && cleared && hdg2, d);
    }

    /* CN3 health / mode: gear node MAINTENANCE = lost, steering node ERROR = lost, CRITICAL = lost; WARNING =
     * a notice only (no alarm, no mode change: OD-6) */
    {
        static pcsim_t p;
        cn_setup(&p, 1104u, 0u);
        bool hdg = cn_hdg(&p);
        sim_hal_can_node(DC_NODE_STEERING, true, DC_HEALTH_WARNING, DC_MODE_OPERATIONAL);
        pc_run(&p, 2500u);
        bool warn = sb_mode(&p.x.sb) == MODE_HDG_LOCK && p.x.sb.alarm_bits2 == 0u && !p.x.sb.alarm &&
                    (p.x.sb.can.warn_mask & 2u) && (cn_p(&p, 3) & 2u) && !p.x.sb.can.node_lost;
        sim_hal_can_node(DC_NODE_GEAR, true, DC_HEALTH_OK, DC_MODE_MAINTENANCE);
        pc_run(&p, 1500u);
        bool maint = sb_mode(&p.x.sb) == MODE_STANDBY && (p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) &&
                     p.x.sb.can.node[HAL_CAN_ROLE_GEAR].why == HAL_CAN_NOT_OPER;
        sim_hal_can_node(DC_NODE_GEAR, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sim_hal_can_node(DC_NODE_STEERING, true, DC_HEALTH_ERROR, DC_MODE_OPERATIONAL);
        pc_run(&p, 1500u);
        bool err = p.x.sb.can.node[HAL_CAN_ROLE_STEERING].why == HAL_CAN_HEALTH && p.x.sb.can.node_block &&
                   !cn_hdg(&p);
        sim_hal_can_node(DC_NODE_STEERING, true, DC_HEALTH_CRITICAL, DC_MODE_OPERATIONAL);
        pc_run(&p, 1500u);
        bool crit = p.x.sb.can.node[HAL_CAN_ROLE_STEERING].why == HAL_CAN_HEALTH;
        sim_hal_can_node(DC_NODE_STEERING, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        pc_run(&p, 1500u);
        sb_alarm_ack(&p.x.sb);
        pc_run(&p, 100u);
        bool ok = !p.x.sb.can.node_block && !(p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) && cn_hdg(&p);
        snprintf(d, sizeof d, "HDG LOCK %s; steering WARNING: notice only (stays in HDG LOCK, no alarm) %s; gear "
                 "MAINTENANCE: STANDBY + CAN NODE LOST %s; steering ERROR: lost + refused %s; CRITICAL: lost %s; all OK: "
                 "ack + HDG LOCK %s", hdg ? "yes" : "NO", warn ? "yes" : "NO", maint ? "yes" : "NO", err ? "yes" : "NO",
                 crit ? "yes" : "NO", ok ? "yes" : "NO");
        test_report("CN3 node health / mode: not OPERATIONAL, ERROR, CRITICAL = lost -> STANDBY; WARNING = notice only",
                    hdg && warn && maint && err && crit && ok, d);
    }

    /* CN4 a node reboots (uptime back): lost + LATCHED, STANDBY; an ack before 2 s OPERATIONAL keeps it,
     * after 2 s in STANDBY it clears; an ack outside STANDBY never clears it */
    {
        static pcsim_t p;
        cn_setup(&p, 1105u, 0u);
        bool hdg = cn_hdg(&p);
        sim_hal_can_node_reboot(DC_NODE_STEERING);
        pc_run(&p, 600u);
        bool lost = sb_mode(&p.x.sb) == MODE_STANDBY && p.x.sb.can.node[HAL_CAN_ROLE_STEERING].rebooted &&
                    p.x.sb.can.node[HAL_CAN_ROLE_STEERING].restarts == 1u && (p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE);
        pc_run(&p, 1500u);                            /* the node OPERATIONAL again since ~1 s */
        sb_alarm_ack(&p.x.sb);
        bool early = p.x.sb.can.node[HAL_CAN_ROLE_STEERING].rebooted && (p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE);
        pc_run(&p, 2500u);
        bool still = p.x.sb.can.node[HAL_CAN_ROLE_STEERING].rebooted && p.x.sb.can.node_block;   /* no ack: latched */
        sb_alarm_ack(&p.x.sb);
        pc_run(&p, 100u);
        bool cleared = !p.x.sb.can.node[HAL_CAN_ROLE_STEERING].rebooted && !(p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) &&
                       cn_hdg(&p);
        /* in HDG LOCK: a reboot -> STANDBY first, the ack happens only in STANDBY (an ack while driving would
         * be outside STANDBY: the mode is already STANDBY by then - checked: the latch needs STANDBY) */
        snprintf(d, sizeof d, "HDG LOCK %s; steering node reboot: STANDBY + latched + CAN NODE LOST %s; ack after ~1 s "
                 "OPERATIONAL keeps it %s; without an ack still latched %s; ack after 2 s: cleared + HDG LOCK %s",
                 hdg ? "yes" : "NO", lost ? "yes" : "NO", early ? "yes" : "NO", still ? "yes" : "NO", cleared ? "yes" : "NO");
        test_report("CN4 node reboot: lost + latched until OPERATIONAL 2 s AND the acknowledge in STANDBY",
                    hdg && lost && early && still && cleared, d);
    }

    /* CN5 the relay node (30) lost: not a required node - no alarm, no STANDBY (the relay board rules stay
     * those of step 6 until 11f) */
    {
        static pcsim_t p;
        cn_setup(&p, 1106u, 0u);
        bool hdg = cn_hdg(&p);
        sim_hal_can_node(DC_NODE_RELAY, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        pc_run(&p, 3000u);
        bool ok = sb_mode(&p.x.sb) == MODE_HDG_LOCK && (p.x.sb.can.lost_mask & 8u) && !p.x.sb.can.node_lost &&
                  !(p.x.sb.alarm_bits2 & (SB_ALM2_CAN_NODE | SB_ALM2_CAN_BUS)) && (cn_p(&p, 2) & 8u);
        snprintf(d, sizeof d, "HDG LOCK %s; relay node unpowered: shown lost (mask 0x%02x), no alarm, still HDG LOCK %s",
                 hdg ? "yes" : "NO", p.x.sb.can.lost_mask, ok ? "yes" : "NO");
        test_report("CN5 relay node lost: shown, not required (no alarm, no mode change)", hdg && ok, d);
    }

    /* CN6 bus-off: CAN BUS FAULT after 100 ms (every node lost: CAN NODE LOST too), STANDBY, interlock;
     * the recovery brings it back by itself, the ack then clears both */
    {
        static pcsim_t p;
        cn_setup(&p, 1107u, 0u);
        bool hdg = cn_hdg(&p);
        uint32_t t0 = p.x.t;
        sim_hal_can_fault(SIM_CAN_BUSOFF);
        while (sb_mode(&p.x.sb) == MODE_HDG_LOCK && p.x.t < t0 + 2000u) pc_run(&p, AP_PERIOD_MS);
        uint32_t dt = p.x.t - t0;
        bool fault = sb_mode(&p.x.sb) == MODE_STANDBY && (p.x.sb.alarm_bits2 & SB_ALM2_CAN_BUS) &&
                     (p.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) && cn_why(&p.x.sb) == WHY_CAN_BUS &&
                     p.x.sb.can.bus.state == HAL_CAN_ST_BUSOFF && p.x.sb.can.lost_mask == 0x0Fu &&
                     p.x.sb.can.node[0].why == HAL_CAN_BUS;
        pc_run(&p, 2000u);
        bool refused = !cn_hdg(&p) && (cn_p(&p, 0) & 2u) && cn_p(&p, 1) == HAL_CAN_ST_BUSOFF;
        uint32_t boffs = p.x.sb.can.bus.busoffs;
        sim_hal_can_fault(0u);
        pc_run(&p, 2500u);
        bool back = !p.x.sb.can.bus_bad && !p.x.sb.can.node_block && p.x.sb.can.bus.state == HAL_CAN_ST_RUN;
        sb_alarm_ack(&p.x.sb);
        pc_run(&p, 100u);
        bool cleared = !(p.x.sb.alarm_bits2 & (SB_ALM2_CAN_BUS | SB_ALM2_CAN_NODE)) && cn_hdg(&p);
        snprintf(d, sizeof d, "HDG LOCK %s; bus-off: STANDBY after %u ms with CAN BUS FAULT + CAN NODE LOST, every node "
                 "lost (BUS) %s; HDG LOCK refused, TELEM2 P %s (bus-offs %u); fault gone: recovered by itself %s, ack "
                 "clears + HDG LOCK %s", hdg ? "yes" : "NO", (unsigned)dt, fault ? "yes" : "NO", refused ? "ok" : "WRONG",
                 (unsigned)boffs, back ? "yes" : "NO", cleared ? "yes" : "NO");
        test_report("CN6 bus-off > 100 ms: CAN BUS FAULT + every node lost -> STANDBY; recovery by itself, ack clears",
                    hdg && dt >= 100u && dt <= 300u && fault && refused && boffs >= 1u && back && cleared, d);
    }

    /* CN7 every node unpowered (nobody acknowledges): error passive > 100 ms = bus fault; our frames wait
     * in the controller (never a flood), the stale queue is dropped; power back = the bus recovers */
    {
        static pcsim_t p;
        cn_setup(&p, 1108u, 0u);
        bool hdg = cn_hdg(&p);
        for (uint8_t id = 20u; id <= 22u; id++) sim_hal_can_node(id, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sim_hal_can_node(DC_NODE_RELAY, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        pc_run(&p, 4000u);
        bool passive = sb_mode(&p.x.sb) == MODE_STANDBY && p.x.sb.can.bus.state == HAL_CAN_ST_PASSIVE &&
                       p.x.sb.can.bus.fault && (p.x.sb.alarm_bits2 & SB_ALM2_CAN_BUS) &&
                       sim_hal_can_tx_waiting() <= 16u;
        for (uint8_t id = 20u; id <= 22u; id++) sim_hal_can_node(id, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        pc_run(&p, 4000u);
        /* the nodes were power-cycled: their uptime went back = REBOOTED (latched until the ack in STANDBY) */
        bool back = !p.x.sb.can.bus.fault && p.x.sb.can.bus.state == HAL_CAN_ST_RUN && p.x.sb.can.reboot_mask == 0x0Fu &&
                    p.x.sb.can.node_block;
        sb_alarm_ack(&p.x.sb);
        pc_run(&p, 100u);
        bool cleared = !(p.x.sb.alarm_bits2 & (SB_ALM2_CAN_BUS | SB_ALM2_CAN_NODE)) && !p.x.sb.can.node_block &&
                       cn_hdg(&p);
        snprintf(d, sizeof d, "HDG LOCK %s; every node unpowered: error passive (TEC %u) -> bus fault + STANDBY %s, "
                 "controller FIFO %u frames (<= 16); power back (node boot: INITIALIZATION 1 s): bus OK, every node "
                 "REBOOTED (latched) %s, ack clears + HDG LOCK %s", hdg ? "yes" : "NO", p.x.sb.can.bus.tec,
                 passive ? "yes" : "NO", (unsigned)sim_hal_can_tx_waiting(), back ? "yes" : "NO", cleared ? "yes" : "NO");
        test_report("CN7 nobody acknowledges: error passive -> bus fault -> STANDBY; power back = recovered",
                    hdg && passive && back && cleared, d);
    }

    /* CN8 OUR node ID 10 on the bus (the Pixhawk connected, OA-4): CAN BUS FAULT + STANDBY + interlock while
     * it is there; gone for 3 s = the interlock goes, the ack clears; log event CAN 7 */
    {
        static pcsim_t p;
        cn_setup(&p, 1109u, 0u);
        bool hdg = cn_hdg(&p);
        uint16_t seq0 = p.x.sb.can.log_seq;
        sim_hal_can_fault(SIM_CAN_FOREIGN10);
        pc_run(&p, 1500u);
        bool conflict = sb_mode(&p.x.sb) == MODE_STANDBY && p.x.sb.can.bus.id_conflict &&
                        (p.x.sb.alarm_bits2 & SB_ALM2_CAN_BUS) && cn_why(&p.x.sb) == WHY_CAN_BUS &&
                        p.x.sb.can.log_seq != seq0 && (cn_p(&p, 0) & 4u);
        pc_run(&p, 3000u);
        bool refused = !cn_hdg(&p) && p.x.sb.can.bus_bad;
        sim_hal_can_fault(0u);
        pc_run(&p, 3500u);
        bool gone = !p.x.sb.can.bus_bad;
        sb_alarm_ack(&p.x.sb);
        pc_run(&p, 100u);
        bool cleared = !(p.x.sb.alarm_bits2 & SB_ALM2_CAN_BUS) && cn_hdg(&p);
        snprintf(d, sizeof d, "HDG LOCK %s; a node 10 on the bus: STANDBY + CAN BUS FAULT + log event %s; still there: "
                 "refused %s; gone 3.5 s: interlock off %s, ack clears + HDG LOCK %s",
                 hdg ? "yes" : "NO", conflict ? "yes" : "NO", refused ? "yes" : "NO", gone ? "yes" : "NO",
                 cleared ? "yes" : "NO");
        test_report("CN8 our own node ID 10 seen on the bus (Pixhawk): CAN BUS FAULT + STANDBY interlock while seen",
                    hdg && conflict && refused && gone && cleared, d);
    }

    /* CN9 a node asking for a dynamic node ID (no static ID set): a notice only (no alarm, no mode change,
     * never answered: no ID server on the AURIX) */
    {
        static pcsim_t p;
        cn_setup(&p, 1110u, 0u);
        bool hdg = cn_hdg(&p);
        uint32_t other0 = sim_hal_can_bus_frames(3u);
        sim_hal_can_fault(SIM_CAN_ANON);
        pc_run(&p, 3000u);
        bool notice = sb_mode(&p.x.sb) == MODE_HDG_LOCK && p.x.sb.can.bus.anon_node && p.x.sb.alarm_bits2 == 0u &&
                      (cn_p(&p, 0) & 8u) && sim_hal_can_bus_frames(3u) == other0;
        sim_hal_can_fault(0u);
        pc_run(&p, 6000u);
        bool gone = !p.x.sb.can.bus.anon_node && !(cn_p(&p, 0) & 8u);
        snprintf(d, sizeof d, "HDG LOCK %s; dynamic-ID requests: notice only, never answered %s; gone after 5 s %s",
                 hdg ? "yes" : "NO", notice ? "yes" : "NO", gone ? "yes" : "NO");
        test_report("CN9 a node without a static ID: notice only (never answered)", hdg && notice && gone, d);
    }

    /* CN10 the CAN controller does not start: CAN BUS FAULT, nothing leaves STANDBY; a required node never
     * seen since the boot blocks from the start (alarm only after 1.5 s) */
    {
        static pcsim_t p;
        cn_setup(&p, 1111u, SIM_CAN_START_FAIL);
        bool fault = p.x.sb.can.bus.state == HAL_CAN_ST_FAULT && (p.x.sb.alarm_bits2 & SB_ALM2_CAN_BUS) &&
                     sim_hal_can_bus_frames(0u) == 0u && !cn_hdg(&p);
        static pcsim_t q;
        s_can_on = true;
        pcsim_init(&q, 1112u);
        s_can_on = false;
        sim_hal_can_node(DC_NODE_GEAR, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);   /* the gear node never powered */
        pc_run(&q, 1000u);
        bool early = q.x.sb.can.node_block && !(q.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE);
        pc_run(&q, 7000u);
        bool later = (q.x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) && q.x.sb.can.node[HAL_CAN_ROLE_GEAR].why == HAL_CAN_NEVER &&
                     !cn_hdg(&q);
        snprintf(d, sizeof d, "controller start failed: FAULT + CAN BUS FAULT, no frame, HDG LOCK refused %s; gear node "
                 "never seen: blocked from the start without an alarm %s, CAN NODE LOST (NEVER) after the limit %s",
                 fault ? "yes" : "NO", early ? "yes" : "NO", later ? "yes" : "NO");
        test_report("CN10 controller start failure / a required node never seen: STANDBY only (alarm after 1.5 s)",
                    fault && early && later, d);
    }

    /* CN11 the simulated servo / relay nodes (for steps 11e / 11f), driven by dronecan.c's own encoder:
     * a command moves the servo at its rate limit (steering 12 s lock to lock), the play (backlash) shows on
     * a reversal, lost commands = HOLD (OA-1); hardpoint commands reach the relay node; the app (step 11e)
     * commands the throttle (IDLE) and, after the boot hold, the gear - no steering in STANDBY (OD-7) */
    {
        static pcsim_t p;
        cn_setup(&p, 1113u, 0u);
        sim_hal_can_servo_speed(DC_NODE_STEERING, 12.0f, 0.03f);     /* (the worst-case speed, also in "--can-fast") */
        bool q1 = dc_send_actuator(2u, DC_CMD_UNITLESS, 1.0f);       /* steering full right */
        pc_run(&p, 3000u);
        float c, pos, out;
        uint32_t n;
        bool got = sim_hal_can_servo(DC_NODE_STEERING, &c, &pos, &out, &n);
        bool rate = got && fabsf(c - 1.0f) < 1e-3f && pos > 0.40f && pos < 0.60f;   /* 2 / 12 s x 3 s = 0.5 */
        float pos3 = pos;
        pc_run(&p, 4000u);
        (void)sim_hal_can_servo(DC_NODE_STEERING, &c, &pos, &out, &n);
        float hold = pos;                                            /* no more commands: holds there */
        bool held = pos >= 0.99f;
        bool q2 = dc_send_actuator(2u, DC_CMD_UNITLESS, 0.9f);       /* a small reversal: inside the play */
        pc_run(&p, 1000u);
        float pos2, out2;
        (void)sim_hal_can_servo(DC_NODE_STEERING, &c, &pos2, &out2, &n);
        bool play = fabsf(pos2 - 0.9f) < 0.02f && out2 > pos2 + 0.01f;   /* the output lags by half the play */
        bool q3 = dc_send_hardpoint(4u, 1u);
        pc_run(&p, 200u);
        uint32_t hpc;
        uint16_t hp = sim_hal_can_hardpoint(4u, &hpc);
        float tc;
        bool thr = sim_hal_can_servo(DC_NODE_THROTTLE, &tc, NULL, NULL, NULL) && tc <= -0.999f;  /* the app: IDLE */
        snprintf(d, sizeof d, "steering: queued %d, after 3 s at 12 s lock-to-lock pos %.2f (want ~0.5), then holds at "
                 "%.2f without commands, reversal 0.9: servo %.2f, output %.2f (play) ; hardpoint 4 = %u (%u commands); "
                 "throttle node commanded idle (-1.0) by the app %s",
                 q1 && q2, (double)pos3, (double)hold, (double)pos2, (double)out2, hp, (unsigned)hpc, thr ? "yes" : "NO");
        test_report("CN11 simulated servo nodes (rate limit, backlash, HOLD on lost commands) + relay node decode",
                    q1 && q2 && q3 && rate && held && play && hp == 1u && hpc == 1u && thr, d);
    }

    /* CN12 telemetry + log: CAN_NODES frame (USB, 1 Hz) with the node table, TELEM2 group P, log event CAN */
    {
        static pcsim_t p;
        cn_setup(&p, 1114u, 0u);
        int f0 = p.can_frames;
        pc_run(&p, 3000u);
        int fn = p.can_frames - f0;
        proto_rd_t r;
        proto_rd_init(&r, p.can, p.can_len);
        uint8_t st = proto_rd_u8(&r), fl = proto_rd_u8(&r);
        (void)proto_rd_u8(&r); (void)proto_rd_u8(&r);
        uint32_t rx = proto_rd_u32(&r);
        r.n += 16u;
        uint8_t nn = proto_rd_u8(&r), id0 = proto_rd_u8(&r), role0 = proto_rd_u8(&r), nf0 = proto_rd_u8(&r);
        bool frame = st == HAL_CAN_ST_RUN && (fl & 1u) && rx > 0u && nn == HAL_CAN_NODES && id0 == DC_NODE_THROTTLE &&
                     role0 == HAL_CAN_ROLE_THROTTLE && (nf0 & 3u) == 3u && p.can_len == 25u + HAL_CAN_NODES * 20u;
        uint16_t s0 = p.x.sb.can.log_seq;
        sim_hal_can_node_silent(DC_NODE_GEAR, true);
        pc_run(&p, 2000u);
        bool ev = p.x.sb.can.log_seq != s0 && p.x.sb.can.log_what == 1u && p.x.sb.can.log_node == DC_NODE_GEAR &&
                  p.x.sb.can.log_why == HAL_CAN_SILENT;
        sim_hal_can_node_silent(DC_NODE_GEAR, false);
        pc_run(&p, 1500u);
        bool ev2 = p.x.sb.can.log_what == 2u;
        snprintf(d, sizeof d, "CAN_NODES frames in 3 s: %d (len %u), state %u flags 0x%02x rx %u n %u first id %u role %u "
                 "flags 0x%02x; log event: gear lost (SILENT) %s, back %s",
                 fn, p.can_len, st, fl, (unsigned)rx, nn, id0, role0, nf0, ev ? "yes" : "NO", ev2 ? "yes" : "NO");
        test_report("CN12 CAN_NODES frame (USB 1 Hz) + TELEM2 group P + log event CAN (29)",
                    fn >= 2 && fn <= 4 && frame && ev && ev2, d);
    }

    /* CN13 MANUAL with the stick held: a node lost -> STANDBY anyway (a fault always wins); the held stick
     * can't take MANUAL back while the node is lost (no mode but STANDBY) */
    {
        static pcsim_t p;
        cn_setup(&p, 1115u, 0u);
        joy_set(0.5f, 0.0f, false, false);
        pc_run(&p, 1000u);
        bool manual = sb_mode(&p.x.sb) == MODE_MANUAL;
        sim_hal_can_node_silent(DC_NODE_STEERING, true);
        uint32_t t0 = p.x.t;
        while (sb_mode(&p.x.sb) == MODE_MANUAL && p.x.t < t0 + 3000u) pc_run(&p, AP_PERIOD_MS);
        bool standby = sb_mode(&p.x.sb) == MODE_STANDBY && cn_why(&p.x.sb) == WHY_CAN_NODE;
        pc_run(&p, 1000u);
        joy_set(0.0f, 0.0f, false, false);
        pc_run(&p, 500u);
        joy_set(0.6f, 0.0f, false, false);
        pc_run(&p, 1000u);
        bool stays = sb_mode(&p.x.sb) == MODE_STANDBY;
        joy_set(0.0f, 0.0f, false, false);
        sim_hal_can_node_silent(DC_NODE_STEERING, false);
        pc_run(&p, 1500u);
        joy_set(0.6f, 0.0f, false, false);
        pc_run(&p, 1000u);
        bool again = sb_mode(&p.x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        snprintf(d, sizeof d, "MANUAL %s; steering node silent: STANDBY (CAN node) %s; stick takeover refused while lost "
                 "%s; node back: the stick takes MANUAL %s", manual ? "yes" : "NO", standby ? "yes" : "NO",
                 stays ? "yes" : "NO", again ? "yes" : "NO");
        test_report("CN13 MANUAL: a node lost -> STANDBY (forced), no takeover while lost, back = MANUAL possible",
                    manual && standby && stays && again, d);
    }
}

/* ======================== SM: the DroneCAN steering position servo (step 11e) ======================= */

/* a boat with the DroneCAN servos (the real boot order: position unknown, no gear frame for 6 s) */
static void sm_boot(sbsim_t *x, uint32_t seed, float steer_travel_s, float backlash, const sb_config_t *cfg)
{
    bool keep = s_can_on;
    s_can_on = true;
    sbsim_init(x, seed, 90.0f, 0.0f, cfg);
    s_can_on = keep;
    sim_hal_can_servo_speed(DC_NODE_STEERING, steer_travel_s, backlash);
    if (steer_travel_s < 6.0f) sim_hal_can_servo_speed(DC_NODE_THROTTLE, 2.0f, 0.0f);   /* the fast set */
}

typedef struct {
    bool     result, accepted, aborted;
    uint8_t  why;               /* tr_result_t */
    float    v_train;
    float    k_true, tau_true;
    trs_t    t;                 /* the TRAINING state at the result */
    float    max_u, max_yaw;
    uint32_t dur_s;
} smrun_t;

/* TRAINING on the position servo: spin-up at the training throttle, the phases, the result (accept = 1,
 * reject = 0, -1 = leave it). The truth: the world's K per servo unit = 0.05 x v x half (sim_world.c) */
static void sm_train(sbsim_t *x, smrun_t *o, int confirm)
{
    memset(o, 0, sizeof *o);
    sbsim_run(x, x->t + 10000u, NULL, NULL);
    sb_request_mode(&x->sb, MODE_TRAINING, WHY_TEST);
    uint32_t t0 = x->t;
    while (x->t < t0 + 20u * 60u * 1000u) {
        sbsim_run(x, x->t + 50u, NULL, NULL);
        float su = fabsf(sim_hal_steer_cmd(NULL));
        if (su > o->max_u) o->max_u = su;
        if (fabsf(x->w.yaw_dps) > o->max_yaw) o->max_yaw = fabsf(x->w.yaw_dps);
        if (sb_mode(&x->sb) != MODE_TRAINING) { o->aborted = true; break; }
        if (x->sb.train.stage == TS_RESULT) { o->result = true; break; }
    }
    o->dur_s = (x->t - t0) / 1000u;
    o->t = x->sb.ap.trs;
    o->why = (uint8_t)x->sb.ap.train.result;
    o->v_train = x->w.speed_kn;
    o->k_true = 0.05f * fabsf(x->w.speed_kn) * x->w.steer_half_deg;
    o->tau_true = 12.0f / fmaxf(fabsf(x->w.speed_kn), 0.3f);
    if (o->result && confirm >= 0) o->accepted = sb_confirm_training(&x->sb, confirm == 1, 0.0f);
}

static void steer_model_tests(void)
{
    char d[900];
    printf("\n=== SM: DroneCAN steering position servo - model, TRAINING, adaptation (step 11e) ===\n");
    /* SM1 TRAINING finds r / b / K within 15 % on the model (slow 12 s and fast 3 s servos) */
    {
        static sbsim_t x;
        static smrun_t a[2];
        const float travel[2] = { 12.0f, 3.0f };
        int good = 0;
        int len = 0;
        for (int k = 0; k < 2; k++) {
            sm_boot(&x, 2101u + (uint32_t)k, travel[k], 0.03f, NULL);
            sm_train(&x, &a[k], 1);
            const trs_t *t = &a[k].t;
            float r_true = 2.0f / travel[k];
            float er = fmaxf(fabsf(t->r_right / r_true - 1.0f), fabsf(t->r_left / r_true - 1.0f));
            float eb = fmaxf(fabsf(t->b_lr / 0.03f - 1.0f), fabsf(t->b_rl / 0.03f - 1.0f));
            float ek = fabsf(t->k / a[k].k_true - 1.0f);
            bool ok = a[k].result && a[k].accepted && er <= 0.15f && eb <= 0.15f && ek <= 0.15f;
            good += ok;
            len += snprintf(d + len, sizeof d - (size_t)len, "%s%.0f s servo: %s in %u s, r %.3f/%.3f (true %.3f), b %.4f/%.4f "
                            "(true 0.03; small %.3f large %.3f), K %.2f (true %.2f), tau %.2f (true %.2f), R2 %.3f, conf %.2f, "
                            "max |u| %.2f, max yaw %.1f, accepted %s", k ? "; " : "", travel[k],
                            a[k].result ? "result" : (a[k].aborted ? "ABORTED" : "timeout"), (unsigned)a[k].dur_s,
                            t->r_right, t->r_left, r_true, t->b_lr, t->b_rl, t->b_small, t->b_large, t->k, a[k].k_true,
                            t->tau, a[k].tau_true, t->r2, t->confidence, a[k].max_u, a[k].max_yaw,
                            a[k].accepted ? "yes" : "NO");
        }
        test_report("SM1 TRAINING (position servo) finds r / b / K within 15 % - slow 12 s and fast 3 s servos", good == 2, d);
    }
}


/* the fastest change of a value per CONTROL STEP (a sbsim_run of 50 ms may hold one or two steps) */
typedef struct { float prev, max; uint32_t prev_ms; bool have; } ratew_t;
static void ratew(ratew_t *r, float v, uint32_t now_ms)
{
    if (r->have && now_ms == r->prev_ms) return;
    if (r->have) {
        float rr = fabsf(v - r->prev) / ((float)(now_ms - r->prev_ms) * 0.001f);
        if (rr > r->max) r->max = rr;
    }
    r->prev = v;
    r->prev_ms = now_ms;
    r->have = true;
}

/* a trained boat in HDG LOCK at the TRAINING throttle (the adaptation's speed band) */
static bool sm_trained_hdg(sbsim_t *x, smrun_t *o, uint32_t seed, float travel_s, float backlash)
{
    sm_boot(x, seed, travel_s, backlash, NULL);
    sm_train(x, o, 1);
    sbsim_run(x, x->t + 15000u, NULL, NULL);
    bool hdg = hdg_lock(x);
    sb_test_drive(&x->sb, true, GEAR_F, x->sb.cfg.training_throttle_pct);
    sbsim_run(x, x->t + 40000u, NULL, NULL);
    return o->accepted && hdg && sb_mode(&x->sb) == MODE_HDG_LOCK && x->sb.ap.sm.stored.trained;
}

/* SM3: every control step - the command that moved stays within target + the half play of its direction */
typedef struct { float over, prev, play_max; uint32_t moving; bool have; } sm3w_t;
static void sm3_hook(sbsim_t *x, void *ctx)
{
    sm3w_t *w = (sm3w_t *)ctx;
    const steer_t *m = &x->sb.ap.sm;
    float h = 0.5f * sm_play(m);
    if (w->have && x->sb.ap.mode == AP_MODE_AUTO) {
        if (m->u > w->prev + 1e-6f) { w->over = fmaxf(w->over, m->u - (m->target + h)); w->moving++; }
        if (m->u < w->prev - 1e-6f) { w->over = fmaxf(w->over, (m->target - h) - m->u); w->moving++; }
    }
    w->play_max = fmaxf(w->play_max, sm_play(m));
    w->prev = m->u;
    w->have = true;
}

static void steer_model_tests2(void)
{
    char d[1000];

    /* SM2 abort paths: yaw > 17 deg/s, STANDBY, joystick, compass lost -> the servo back to the centre (at most the
     * measured / worst-case speed), then the STANDBY rules (hold); the joystick takes MANUAL; alarm unless the user */
    {
        int good = 0, len = 0;
        for (int k = 0; k < 4; k++) {
            static sbsim_t x;
            sm_boot(&x, 2401u + (uint32_t)k, 12.0f, 0.03f, NULL);
            sbsim_run(&x, 10000u, NULL, NULL);
            sb_request_mode(&x.sb, MODE_TRAINING, WHY_TEST);
            uint32_t t0 = x.t;
            /* into the SMALL steps (servo away from 0) */
            while (x.t < t0 + 200000u && !(x.sb.ap.trs.phase == TRS_SMALL && x.sb.ap.trs.sub >= 3u))
                sbsim_run(&x, x.t + 50u, NULL, NULL);
            bool reached = x.sb.ap.trs.phase == TRS_SMALL;
            if (k == 0) x.w.trim_deg = 80.0f;                       /* a huge yaw */
            if (k == 1) sim_hal_press(HAL_BTN_STANDBY);
            if (k == 2) joy_set(0.6f, 0.0f, false, false);
            if (k == 3) sim_hal_fail_heading(true);
            ratew_t rw;
            memset(&rw, 0, sizeof rw);
            uint32_t t1 = x.t;
            while (x.t < t1 + 25000u) {
                sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
                if (k != 2) ratew(&rw, sim_hal_steer_cmd(NULL), x.sb.now_ms);
                if (k == 0 && sb_mode(&x.sb) != MODE_TRAINING) x.w.trim_deg = 0.0f;   /* (the gust is over) */
            }
            float rate_max = rw.max;
            uint8_t why = (uint8_t)x.sb.ap.train.result;
            bool ok;
            if (k == 2) ok = reached && sb_mode(&x.sb) == MODE_MANUAL && why == TR_ABORT_USER;
            else ok = reached && sb_mode(&x.sb) == MODE_STANDBY && !x.sb.stopping && fabsf(sim_hal_steer_cmd(NULL)) < 0.02f &&
                      rate_max <= SM_RATE_WORST * 1.05f + 0.01f &&
                      (k == 0 ? (why == TR_ABORT_YAW && x.sb.alarm) : k == 1 ? why == TR_ABORT_USER : why == TR_ABORT_SENSOR);
            good += ok;
            static const char *nm[4] = { "yaw > 17", "STANDBY", "joystick", "compass lost" };
            len += snprintf(d + len, sizeof d - (size_t)len, "%s%s: %s, mode %s, result %s, steering %.3f, fastest %.3f/s %s",
                            k ? "; " : "", nm[k], reached ? "in SMALL" : "NOT REACHED", mode_name(sb_mode(&x.sb)),
                            training_result_text((tr_result_t)why), (double)sim_hal_steer_cmd(NULL), (double)rate_max,
                            ok ? "ok" : "FAIL");
            joy_set(0.0f, 0.0f, false, false);
            sim_hal_fail_heading(false);
        }
        test_report("SM2 TRAINING aborts (yaw > 17 deg/s, STANDBY, joystick, compass): servo back to the centre at the "
                    "worst-case speed, then hold; the joystick takes MANUAL", good == 4, d);
    }

    /* SM3 play compensation on every direction change, never more than TRAINING's play; the command never moves faster
     * than the measured servo speed (HDG LOCK in a little sea) */
    {
        static sbsim_t x;
        static smrun_t o;
        bool ok0 = sm_trained_hdg(&x, &o, 2405u, 12.0f, 0.03f);
        x.w.sea_state = 1.5f;
        const steer_t *m = &x.sb.ap.sm;
        float bs = 0.5f * (m->stored.b_lr + m->stored.b_rl);
        ratew_t rw;
        memset(&rw, 0, sizeof rw);
        sm3w_t w3;
        memset(&w3, 0, sizeof w3);
        uint16_t rev0 = m->reversals;
        uint32_t t0 = x.t;
        while (x.t < t0 + 120000u) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, sm3_hook, &w3);   /* (checked after every control step) */
            ratew(&rw, m->u, x.sb.now_ms);
        }
        float over = w3.over, play_max = w3.play_max, rate = rw.max;
        uint32_t settled = w3.moving;
        uint16_t revs = (uint16_t)(m->reversals - rev0);
        float rmax = fmaxf(m->use.r_right, m->use.r_left);
        snprintf(d, sizeof d, "trained %s; 120 s HDG LOCK sea 1.5: %u reversals; moving steps %u, compensation beyond b/2 "
                 "max %.4f; play in use max %.4f <= TRAINING %.4f; command rate max %.3f/s <= servo %.3f/s",
                 ok0 ? "yes" : "NO", revs, (unsigned)settled, (double)over, (double)play_max, (double)bs, (double)rate,
                 (double)rmax);
        test_report("SM3 play compensation on every reversal, never more than TRAINING's play; the command never "
                    "faster than the servo", ok0 && revs >= 5u && over <= 1e-3f && play_max <= bs + 1e-5f &&
                    rate <= rmax * 1.01f + 1e-3f, d);
    }

    /* SM4 slow adaptation inside the bounds (RAM only): K follows a changed boat but never beyond 0.7..1.3 x TRAINING
     * (a boat far out = a bound hit = back to the stored values); the play may only shrink, never below 0.5 x;
     * the servo speed is never adapted */
    {
        static sbsim_t x;
        static smrun_t o;
        bool ok0 = sm_trained_hdg(&x, &o, 2406u, 12.0f, 0.03f);
        const steer_t *m = &x.sb.ap.sm;
        float k0 = m->stored.k;
        /* the boat turns 15 % harder (load / trim): K moves toward it - calm water, the helmsman nudging the course
         * +-1 deg every 15 s (the steering is the excitation; still "straight": no turn > 5 deg in 30 s) */
        x.w.K_override = 1.15f * 0.05f * fabsf(x.w.speed_kn);
        x.w.T_override = 12.0f / fmaxf(fabsf(x.w.speed_kn), 0.3f);
        x.w.sea_state = 0.0f;
        bool bounds = true;
        float kmax = 0.0f;
        uint32_t t0 = x.t, nudge = 0u;
        while (x.t < t0 + 900000u) {
            sbsim_run(&x, x.t + 500u, NULL, NULL);
            if (x.t - t0 >= nudge * 15000u) { sim_hal_press((nudge & 1u) ? HAL_BTN_MINUS_1 : HAL_BTN_PLUS_1); nudge++; }
            bounds &= sm_within_bounds(m);
            kmax = fmaxf(kmax, m->use.k);
        }
        bool follows = kmax > k0 * 1.03f && kmax <= k0 * SM_ADAPT_K_HI + 1e-3f && m->use.r_right == m->stored.r_right;
        /* far out (x 2): reaches the bound -> reset to the stored values + notice */
        uint16_t resets0 = m->resets;
        x.w.K_override = 2.0f * 0.05f * fabsf(x.w.speed_kn);
        t0 = x.t;
        nudge = 0u;
        while (x.t < t0 + 1800000u && m->resets == resets0) {
            sbsim_run(&x, x.t + 500u, NULL, NULL);
            if (x.t - t0 >= nudge * 15000u) { sim_hal_press((nudge & 1u) ? HAL_BTN_MINUS_1 : HAL_BTN_PLUS_1); nudge++; }
            bounds &= sm_within_bounds(m);
        }
        bool limit = m->resets > resets0 && x.sb.steer.reset_why != SM_RESET_NONE && !m->adapted;
        x.w.K_override = 0.0f;
        x.w.T_override = 0.0f;
        /* the play shrinks (the linkage was tightened) but never below 0.5 x */
        sim_hal_can_servo_speed(DC_NODE_STEERING, 12.0f, 0.004f);
        float bmin = sm_play(m);
        t0 = x.t;
        nudge = 0u;
        while (x.t < t0 + 1800000u) {
            sbsim_run(&x, x.t + 500u, NULL, NULL);
            if (x.t - t0 >= nudge * 15000u) { sim_hal_press((nudge & 1u) ? HAL_BTN_MINUS_1 : HAL_BTN_PLUS_1); nudge++; }
            bounds &= sm_within_bounds(m);
            bmin = fminf(bmin, sm_play(m));
        }
        float bs = 0.5f * (m->stored.b_lr + m->stored.b_rl);
        bool shrink = bmin < bs * 0.99f && bmin >= SM_ADAPT_B_LO * bs - 1e-5f;
        snprintf(d, sizeof d, "trained %s (K %.2f); boat +15 %%: K up to %.2f (bound %.2f) %s; boat x 2: bound -> reset (why "
                 "%u) %s; play tightened to 0.004: the play in use down to %.4f (stored %.4f, floor %.4f) %s; inside the "
                 "bounds every step %s", ok0 ? "yes" : "NO", (double)k0, (double)kmax, (double)(k0 * SM_ADAPT_K_HI),
                 follows ? "yes" : "NO", x.sb.steer.reset_why, limit ? "yes" : "NO", (double)bmin, (double)bs,
                 (double)(SM_ADAPT_B_LO * bs), shrink ? "yes" : "NO", bounds ? "yes" : "NO");
        test_report("SM4 slow adaptation inside the TRAINING bounds: K follows a changed boat, a bound hit = the stored "
                    "values, the play only shrinks (>= 0.5 x), the speed never adapted", ok0 && follows && limit &&
                    shrink && bounds, d);
    }

    /* SM5 doubt -> the stored values at once (notice STEERING MODEL RESET, log event 30): a sensor fault, the steering
     * node lost; nothing adapted = nothing to reset */
    {
        static sbsim_t x;
        static smrun_t o;
        bool ok0 = sm_trained_hdg(&x, &o, 2407u, 12.0f, 0.03f);
        steer_t *m = &x.sb.ap.sm;
        m->use.k = 1.1f * m->stored.k;                      /* (as if adapted) */
        m->adapted = true;
        uint16_t lg = x.sb.steer.log_seq;
        sim_hal_set_gyro(false, 0.05f, 0.02f);              /* the gyro gone */
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool sensor = !m->adapted && m->use.k == m->stored.k && x.sb.steer.reset_why == SM_RESET_SENSOR &&
                      x.sb.steer.log_seq != lg && x.sb.steer.log_what == 1u;
        sim_hal_set_gyro(true, 0.05f, 0.02f);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool hdg2 = sb_mode(&x.sb) == MODE_HDG_LOCK || hdg_lock(&x);
        m->use.k = 1.1f * m->stored.k;
        m->adapted = true;
        sim_hal_can_node_silent(DC_NODE_STEERING, true);
        sbsim_run(&x, x.t + 2500u, NULL, NULL);
        bool node = !m->adapted && x.sb.steer.reset_why == SM_RESET_NODE;
        sim_hal_can_node_silent(DC_NODE_STEERING, false);
        uint16_t r0 = m->resets;
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool quiet = m->resets == r0;                       /* (nothing adapted now: no more resets) */
        snprintf(d, sizeof d, "trained %s; adapted + gyro lost: reset to the stored values (SENSOR), log event 30 %s; "
                 "HDG LOCK again %s; adapted + steering node lost: reset (NODE) %s; nothing adapted: no more resets %s",
                 ok0 ? "yes" : "NO", sensor ? "yes" : "NO", hdg2 ? "yes" : "NO", node ? "yes" : "NO", quiet ? "yes" : "NO");
        test_report("SM5 doubt (sensor fault, node lost) -> the stored values at once + notice + log event 30",
                    ok0 && sensor && node && quiet, d);
    }

    /* SM6 NV slot 5 record version 2: a version-1 (DC-motor) record = defaults + NV DATA RESET + TRAINING needed; an
     * accepted TRAINING is saved (v2) in STANDBY and loaded after a reboot; the ADAPTED values are never saved */
    {
        static sbsim_t x;
        calib_data_t c1;
        memset(&c1, 0, sizeof c1);
        c1.gain_dps_per_ms = 0.01f; c1.time_const_s = 2.0f; c1.travel_right_ms = 3000.0f; c1.travel_left_ms = 3000.0f;
        c1.gap_ms = 100.0f;
        calib_seal(&c1);
        sm_boot(&x, 2408u, 12.0f, 0.03f, NULL);
        FILE *f = fopen("nv5.bin", "wb");
        if (f) { fwrite(&c1, 1, sizeof c1, f); fclose(f); }
        hal_init();
        sb_init(&x.sb, &x.cfg);
        bool v1 = x.sb.steer.v1_refused && x.sb.steer.needs_training && !x.sb.ap.sm.stored.trained &&
                  (nv_status()->reset_mask & (1u << HAL_NV_SLOT_CALIB)) &&
                  x.sb.ap.sm.stored.r_right == SM_RATE_WORST && sm_play(&x.sb.ap.sm) == 0.0f;
        static smrun_t o;
        sm_train(&x, &o, 1);
        sbsim_run(&x, x.t + 15000u, NULL, NULL);
        long sz = -1;
        f = fopen("nv5.bin", "rb");
        if (f) { fseek(f, 0, SEEK_END); sz = ftell(f); fclose(f); }
        bool saved = o.accepted && sz == (long)sizeof(calib2_t) && !x.sb.steer.needs_training;
        float k_st = x.sb.ap.sm.stored.k;
        x.sb.ap.sm.use.k = 1.2f * k_st;                     /* adapted ... */
        x.sb.ap.sm.adapted = true;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        x.frozen = true;                                    /* ... then a reset */
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool loaded = x.resets >= 1u && x.sb.ap.sm.stored.trained && fabsf(x.sb.ap.sm.use.k - k_st) < 1e-4f &&
                      !x.sb.ap.sm.adapted && !x.sb.steer.needs_training && x.sb.ap.calibrated;
        snprintf(d, sizeof d, "a v1 (DC-motor) record: refused, defaults (worst-case speed, no play), NV DATA RESET, "
                 "TRAINING needed %s; TRAINING accepted %s, saved as v2 (%ld bytes) %s; adapted + reset: the stored values "
                 "again %s", v1 ? "yes" : "NO", o.accepted ? "yes" : "NO", sz, saved ? "yes" : "NO", loaded ? "yes" : "NO");
        test_report("SM6 NV slot 5 v2: a v1 record = defaults + NV DATA RESET + TRAINING needed; TRAINING saved + loaded "
                    "after a reset; adapted values never saved", v1 && saved && loaded, d);
        remove("nv5.bin");
    }

    /* SM7 course keeping on the position servo, slow (12 s) and fast (3 s): with the defaults and with the TRAINING
     * values (a 30 deg course change + holding); the command never faster than the servo speed in use */
    {
        const float travel[2] = { 12.0f, 3.0f };
        int good = 0, len = 0;
        for (int k = 0; k < 2; k++) {
            static sbsim_t x;
            static smrun_t o;
            bool ok0 = sm_trained_hdg(&x, &o, 2409u + (uint32_t)k, travel[k], 0.03f);
            x.w.sea_state = 1.0f;
            sb_test_drive(&x.sb, true, GEAR_F, 40.0f);
            sbsim_run(&x, x.t + 20000u, NULL, NULL);
            press(&x, HAL_BTN_PLUS_10); press(&x, HAL_BTN_PLUS_10); press(&x, HAL_BTN_PLUS_10);
            float hold = 0.0f;
            ratew_t rw;
            memset(&rw, 0, sizeof rw);
            uint32_t t0 = x.t;
            while (x.t < t0 + 90000u) {
                sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
                if (x.t > t0 + 40000u) hold = fmaxf(hold, fabsf(hdg_err(x.w.heading_deg, x.sb.ap.target_heading)));
                ratew(&rw, x.sb.ap.sm.u, x.sb.now_ms);
            }
            float rate = rw.max;
            float rmax = fmaxf(x.sb.ap.sm.use.r_right, x.sb.ap.sm.use.r_left);
            bool ok = ok0 && hold < 5.0f && rate <= rmax * 1.01f + 1e-3f;
            good += ok;
            len += snprintf(d + len, sizeof d - (size_t)len, "%s%.0f s servo: trained %s, after +30 deg held within %.1f deg, "
                            "command rate max %.3f/s (servo %.3f/s)", k ? "; " : "", (double)travel[k], ok0 ? "yes" : "NO",
                            (double)hold, (double)rate, (double)rmax);
        }
        test_report("SM7 course keeping on the position servo (slow 12 s / fast 3 s): turns and holds, the command never "
                    "faster than the servo", good == 2, d);
    }

    /* SM8 the steering node lost during TRAINING: CAN NODE LOST, TRAINING aborted, STANDBY */
    {
        static sbsim_t x;
        sm_boot(&x, 2411u, 12.0f, 0.03f, NULL);
        sbsim_run(&x, 10000u, NULL, NULL);
        sb_request_mode(&x.sb, MODE_TRAINING, WHY_TEST);
        sbsim_run(&x, x.t + 80000u, NULL, NULL);
        bool running = sb_mode(&x.sb) == MODE_TRAINING && x.sb.train.stage == TS_RUN;
        sim_hal_can_node_silent(DC_NODE_STEERING, true);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool stopped = sb_mode(&x.sb) == MODE_STANDBY && (x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) &&
                       x.sb.ap.train.phase == TR_ABORTED && x.sb.ap.mode != AP_MODE_TRAINING;
        sim_hal_can_node_silent(DC_NODE_STEERING, false);
        snprintf(d, sizeof d, "TRAINING running %s; steering node silent: CAN NODE LOST + STANDBY, TRAINING aborted %s",
                 running ? "yes" : "NO", stopped ? "yes" : "NO");
        test_report("SM8 steering node lost during TRAINING: aborted, STANDBY + CAN NODE LOST", running && stopped, d);
    }

    /* SM9 telemetry + log: TELEM2 group Q (servo, known, commanded, trained, play, K, confidence, the throttle model,
     * thr_travel_s) and log event 30 STEERMODEL when the TRAINING values were stored */
    {
        static pcsim_t p;
        bool keep = s_can_on;
        s_can_on = true;
        pcsim_init(&p, 2412u);
        s_can_on = keep;
        pc_run(&p, 3000u);
        const uint8_t *q = &p.t2[p.t2_len - T2_Q];
        bool before = p.t2_len >= T2_Q && (q[0] & 1u) && !(q[0] & 2u) && !(q[0] & 4u) && (q[0] & 32u) &&
                      (int8_t)q[2] == -128 && q[14] == 60u;
        snprintf(d, sizeof d, "boot: TELEM2 Q flags 0x%02x (servo, unknown, no command, TRAINING needed), position %d, "
                 "travel %.1f s, throttle model %u %% %s", q[0], (int8_t)q[2], q[14] * 0.1, q[13], before ? "ok" : "WRONG");
        test_report("SM9 TELEM2 group Q: the position servo state + the throttle model", before, d);
    }
}

/* ====================== CN14..CN23: the servos on DroneCAN (step 11e) ====================== */

static float wire_of(uint16_t us) { return ((float)us - 1000.0f) / 500.0f - 1.0f; }

/* a CAN boat (the real boot order) in HDG LOCK at a raw throttle, gear F */
static bool can_drive(sbsim_t *x, uint32_t seed, float travel_s, float thr)
{
    sm_boot(x, seed, travel_s, 0.03f, NULL);
    sbsim_run(x, 12000u, NULL, NULL);
    bool hdg = hdg_lock(x);
    sb_test_drive(&x->sb, true, GEAR_F, thr);
    sbsim_run(x, x->t + 15000u, NULL, NULL);
    return hdg && sb_mode(&x->sb) == MODE_HDG_LOCK && gear_is(&x->sb.gear, GEAR_F) && sim_hal_thr_truth() > thr - 2.0f;
}

/* REMOTE driving (CREWED or UNCREWED) on the DroneCAN servos: like ll_drive, plus the boot hold */
static bool can_remote_drive(sbsim_t *x, llsim_t *l, uint32_t seed, bool crewed)
{
    const uint8_t TAKE = 0u, ASK = 0u;
    memset(l, 0, sizeof *l);
    bool keep = s_can_on;
    s_can_on = true;
    crew_init(x, &l->r, seed, crewed, NULL);
    s_can_on = keep;
    l->r.p.shore_on = true;
    l->r.p.shore_flags = SB_SHORE_VALID;
    l->r.p.shore_m = 800u;
    sbsim_run(x, x->t + 4000u, ll_hook, l);         /* the boot hold: no gear frame before the throttle travel time */
    pi_cmd(x, &l->r.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
    l->r.rj_on = true;
    sbsim_run(x, x->t + 300u, ll_hook, l);
    sb_alarm_ack(&x->sb);
    pi_cmd(x, &l->r.p, 2u, false, PROTO_T_REMOTE, &ASK, 1u);
    if (crewed) onboard_confirm(x, &l->r); else shore_confirm(x, &l->r);
    rj_lever(x, &l->r, 1.0f, 1.0f);
    sbsim_run(x, x->t + 10000u, ll_hook, l);
    return sb_mode(&x->sb) == MODE_REMOTE && sim_hal_throttle() > 10.0f && gear_is(&x->sb.gear, GEAR_F);
}

static void can_servo_tests(void)
{
    char d[1000];
    printf("\n=== CN14-CN27: the servos on DroneCAN - boot order, node loss, CPU1 safe frames, dead-man, rates (step 11e) ===\n");

    /* CN14 boot order (OD-7 / OA-2): the first frame is throttle IDLE; no gear frame before thr_travel_s with the
     * throttle node OPERATIONAL, then NEUTRAL; no steering frame until a mode commands steering (no auto-centre:
     * the node holds where it was); STANDBY = HOLD; after a WARM reset the position is known (reset-surviving RAM)
     * but still no frame; after a POWER-ON it is unknown */
    {
        static sbsim_t x;
        sm_boot(&x, 2301u, 12.0f, 0.03f, NULL);
        sim_hal_can_servo_set(DC_NODE_STEERING, 0.3f);                  /* the outboard was left at +0.3 */
        sbsim_run(&x, 5000u, NULL, NULL);
        sim_can_act_t a1, a2, a3;
        sim_hal_can_act(1u, &a1); sim_hal_can_act(2u, &a2); sim_hal_can_act(3u, &a3);
        bool first = sim_hal_can_first_act() == 1u && a1.any && a1.first <= -0.999f;
        bool no_gear_early = !a3.any, no_steer = !a2.any;
        sbsim_run(&x, 11000u, NULL, NULL);
        sim_hal_can_act(3u, &a3); sim_hal_can_act(2u, &a2);
        bool gear_later = a3.any && a3.first_ms >= 6000u && fabsf(a3.first - wire_of(x.sb.cfg.gear.pwm_n_us)) < 1e-3f;
        float sp;
        (void)sim_hal_can_servo(DC_NODE_STEERING, NULL, &sp, NULL, NULL);
        bool held = !a2.any && fabsf(sp - 0.3f) < 1e-3f && !sm_known(&x.sb.ap.sm) && x.sb.armed;
        bool hdg = hdg_lock(&x);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        sim_hal_can_act(2u, &a2);
        bool steer_now = a2.any && a2.n_core0 > 0u;
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        sim_hal_can_act(2u, &a2);
        float v_hold = a2.last;
        uint32_t n_hold = a2.n_core0;
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        sim_hal_can_act(2u, &a2);
        bool hold = sb_mode(&x.sb) == MODE_STANDBY && a2.n_core0 > n_hold + 100u && fabsf(a2.last - v_hold) < 1e-4f;
        /* a watchdog reset: no frame during the boot, the nodes hold; afterwards known, still no steering frame */
        float u_before = x.sb.ap.servo_u;
        x.frozen = true;
        sbsim_run(&x, x.t + 150u, NULL, NULL);
        sim_hal_can_act_clear();
        sbsim_run(&x, x.t + 9000u, NULL, NULL);
        sim_hal_can_act(2u, &a2); sim_hal_can_act(1u, &a1);
        bool warm = x.resets >= 1u && !a2.any && a1.any && sim_hal_can_first_act() == 1u && sm_known(&x.sb.ap.sm) &&
                    fabsf(x.sb.ap.sm.u - u_before) < 1e-3f && !x.sb.ap.servo_have;
        /* a power-on reset: the position unknown again */
        sim_hal_chip_reset(HAL_RESET_POWER_ON);
        x.boot_until = x.t + 200u;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool cold = !sm_known(&x.sb.ap.sm) && !x.sb.ap.servo_have;
        snprintf(d, sizeof d, "first frame throttle idle %s (actuator %u, %.2f); no gear frame in 5 s %s, no steering %s; gear "
                 "first after %u ms = N %s; steering node held at +0.3 (no auto-centre), position unknown %s; HDG LOCK %s -> "
                 "steering frames %s; STANDBY = hold (same value, still sent) %s; watchdog reset: throttle idle first, no "
                 "steering frame, position known from reset-surviving RAM %s; power-on: unknown %s",
                 first ? "yes" : "NO", sim_hal_can_first_act(), (double)a1.first, no_gear_early ? "yes" : "NO",
                 no_steer ? "yes" : "NO", (unsigned)a3.first_ms, gear_later ? "yes" : "NO", held ? "yes" : "NO",
                 hdg ? "yes" : "NO", steer_now ? "yes" : "NO", hold ? "yes" : "NO", warm ? "yes" : "NO", cold ? "yes" : "NO");
        test_report("CN14 boot order: throttle IDLE first, no gear frame before the throttle travel time, no steering "
                    "frame until a mode steers (no auto-centre), STANDBY = hold, warm reset known / power-on unknown",
                    first && no_gear_early && no_steer && gear_later && held && hdg && steer_now && hold && warm && cold, d);
    }

    /* CN15 throttle node lost (OA-2 / OD-3): CREWED - CAN NODE LOST + STANDBY, the gear HELD (no gear frame), no kill;
     * back: the gear moves again only after the throttle travel time. UNCREWED - ENGINE KILL in the same step, no gear frame */
    {
        static sbsim_t x;
        bool drv = can_drive(&x, 2302u, 12.0f, 50.0f);
        sim_can_act_t g0, g1;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        uint32_t t0 = x.t;
        while (!x.sb.can.node[HAL_CAN_ROLE_THROTTLE].lost && x.t < t0 + 3000u) sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
        sim_hal_can_act(3u, &g0);
        sbsim_run(&x, x.t + 8000u, NULL, NULL);
        sim_hal_can_act(3u, &g1);
        float gout;
        (void)sim_hal_can_servo(DC_NODE_GEAR, NULL, NULL, &gout, NULL);
        bool crewed = sb_mode(&x.sb) == MODE_STANDBY && (x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) &&
                      g1.n_core0 == g0.n_core0 && gout > 0.5f && !sim_hal_engine_killed() &&
                      x.sb.ll.state != SB_LL_KILLED && x.sb.servo.est_pct >= 99.0f;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        uint32_t tb = x.t;
        uint32_t t_gear = 0u;
        float f_wire = wire_of(x.sb.cfg.gear.pwm_f_us);
        while (x.t < tb + 12000u) {
            sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
            sim_hal_can_act(3u, &g1);
            if (!t_gear && g1.n_core0 > g0.n_core0 && fabsf(g1.last - f_wire) > 1e-3f) t_gear = x.t - tb;   /* a CHANGE */
        }
        (void)sim_hal_can_servo(DC_NODE_GEAR, NULL, NULL, &gout, NULL);
        bool back = t_gear >= 6000u && fabsf(gout) < 0.05f && x.w.shift_under_load == 0;
        static sbsim_t y;
        static llsim_t l;
        bool rdrv = can_remote_drive(&y, &l, 2303u, false);
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        uint32_t t_lost = 0u, t_kill = 0u;
        t0 = y.t;
        while (y.t < t0 + 4000u) {
            sbsim_run(&y, y.t + AP_PERIOD_MS, ll_hook, &l);
            if (!t_lost && y.sb.can.node[HAL_CAN_ROLE_THROTTLE].lost) { t_lost = y.t; sim_hal_can_act(3u, &g0); }
            if (!t_kill && sim_hal_engine_killed()) t_kill = y.t;
        }
        sim_hal_can_act(3u, &g1);
        bool unc = rdrv && t_lost && t_kill == t_lost && y.sb.ll.node_kill && (y.sb.alarm_bits & SB_ALM_ENGINE_KILLED) &&
                   g1.n_core0 == g0.n_core0;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        snprintf(d, sizeof d, "CREWED driving %s; throttle node silent: STANDBY + CAN NODE LOST, gear held in F (no gear frame "
                 "in 8 s), no kill, throttle position unknown %s; back: the first gear CHANGE after %u ms (>= 6000), N, no shift "
                 "under load %s; UNCREWED driving %s: lost at +%u ms, ENGINE KILL at +%u ms (same step), no gear frame %s",
                 drv ? "yes" : "NO", crewed ? "yes" : "NO", (unsigned)t_gear, back ? "yes" : "NO", rdrv ? "yes" : "NO",
                 (unsigned)(t_lost - t0), (unsigned)(t_kill - t0), unc ? "yes" : "NO");
        test_report("CN15 throttle node lost: gear HOLD (never an automatic neutral), CREWED no kill; UNCREWED ENGINE KILL "
                    "at once (OD-3)", drv && crewed && back && unc, d);
    }

    /* CN16 gear node / steering node lost: idle COMMANDED at once (the same step as STANDBY), the steering holds */
    {
        static sbsim_t x;
        bool drv = can_drive(&x, 2304u, 12.0f, 60.0f);
        sim_hal_can_node_silent(DC_NODE_GEAR, true);
        bool idle_step = false;
        uint32_t t0 = x.t;
        while (sb_mode(&x.sb) != MODE_STANDBY && x.t < t0 + 3000u) sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
        idle_step = sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f;
        bool gear_alarm = (x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) && x.sb.can.node[HAL_CAN_ROLE_GEAR].lost;
        sim_hal_can_node_silent(DC_NODE_GEAR, false);
        static sbsim_t y;
        bool drv2 = can_drive(&y, 2305u, 12.0f, 40.0f);
        y.w.trim_deg = 2.0f;
        sbsim_run(&y, y.t + 3000u, NULL, NULL);
        sim_hal_can_node_silent(DC_NODE_STEERING, true);
        t0 = y.t;
        while (sb_mode(&y.sb) != MODE_STANDBY && y.t < t0 + 3000u) sbsim_run(&y, y.t + AP_PERIOD_MS, NULL, NULL);
        bool idle2 = sb_mode(&y.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f;
        sbsim_run(&y, y.t + 10000u, NULL, NULL);            /* the stop finishes: the steering holds */
        float h1 = sim_hal_steer_cmd(NULL);
        sbsim_run(&y, y.t + 3000u, NULL, NULL);
        bool have;
        float h2 = sim_hal_steer_cmd(&have);
        bool hold = have && fabsf(h1 - h2) < 1e-4f && !y.sb.stopping;
        sim_hal_can_node_silent(DC_NODE_STEERING, false);
        snprintf(d, sizeof d, "gear node silent while driving %s: STANDBY with the throttle idle commanded in that step %s, "
                 "CAN NODE LOST (gear) %s; steering node silent (driving %s): idle at once %s, the steering command held "
                 "(%.3f -> %.3f) %s", drv ? "yes" : "NO", idle_step ? "yes" : "NO", gear_alarm ? "yes" : "NO",
                 drv2 ? "yes" : "NO", idle2 ? "yes" : "NO", (double)h1, (double)h2, hold ? "yes" : "NO");
        test_report("CN16 gear / steering node lost: STANDBY with idle commanded at once, the steering holds",
                    drv && idle_step && gear_alarm && drv2 && idle2 && hold, d);
    }

    /* CN17 the CPU1 safety path (design 6.3): a trip latches, core 0's frames stop, CPU1's safe frames within 20 ms:
     * throttle IDLE, steering = the last command (hold), gear HOLD until thr_travel_s then NEUTRAL (throttle node OK);
     * with the throttle node lost the gear is held for good; core 0 frozen / its CAN path stuck = MON_TRIP_CAN_TX */
    {
        static sbsim_t x;
        inject_t j = { 1, 0u, false, 0u };
        bool drv = can_drive(&x, 2306u, 12.0f, 50.0f);
        float steer_last = x.sb.ap.servo_u;
        sb_power_lower(&x.sb);                              /* (30 %: the injected 90 % is above it) */
        sbsim_run(&x, x.t + 600u, NULL, NULL);
        j.at_ms = x.t + 1u;
        uint32_t t0 = x.t;
        while (!hal_outputs_forced() && x.t < t0 + 500u) sbsim_run(&x, x.t + 10u, inject_hook, &j);
        uint32_t t_trip = x.t;
        steer_last = CA_STEER_SIGN * x.sb.ap.servo_u;
        sim_can_act_t a1, a2, a3, b1;
        sbsim_run(&x, x.t + 30u, inject_hook, &j);
        sim_hal_can_act(1u, &a1);
        sbsim_run(&x, x.t + 3000u, inject_hook, &j);
        sim_hal_can_act(1u, &b1); sim_hal_can_act(2u, &a2); sim_hal_can_act(3u, &a3);
        bool fast = a1.n_cpu1 > 0u && a1.cpu1_ms <= t_trip + 30u && a1.last_cpu1 <= -0.999f;
        bool core0_stopped = b1.core0_ms <= t_trip + 20u;
        bool steer_hold = a2.n_cpu1 > 0u && fabsf(a2.last_cpu1 - steer_last) < 0.01f;
        bool gear_hold = a3.n_cpu1 > 0u && fabsf(a3.last_cpu1 - wire_of(x.sb.cfg.gear.pwm_f_us)) < 1e-3f;
        sbsim_run(&x, t_trip + 7000u, inject_hook, &j);
        sim_hal_can_act(3u, &a3);
        bool gear_n = fabsf(a3.last_cpu1 - wire_of(x.sb.cfg.gear.pwm_n_us)) < 1e-3f && x.w.shift_under_load == 0 &&
                      sim_hal_thr_truth() <= 1.0f;
        /* the throttle node lost before the trip: the gear is held for good */
        static sbsim_t y;
        inject_t k = { 1, 0u, false, 0u };
        bool drv2 = can_drive(&y, 2307u, 12.0f, 50.0f);
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        sbsim_run(&y, y.t + 1700u, NULL, NULL);
        k.at_ms = y.t + 1u;
        t0 = y.t;
        while (!hal_outputs_forced() && y.t < t0 + 500u) sbsim_run(&y, y.t + 10u, inject_hook, &k);
        sbsim_run(&y, y.t + 9000u, inject_hook, &k);
        sim_hal_can_act(3u, &a3);
        bool held_forever = hal_outputs_forced() && a3.n_cpu1 > 0u && fabsf(a3.last_cpu1 - wire_of(y.sb.cfg.gear.pwm_f_us)) < 1e-3f;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        /* core 0 frozen (watchdog off): MON_TRIP_CAN_TX ~100 ms, CPU1's frames */
        static sbsim_t z;
        bool drv3 = can_drive(&z, 2308u, 12.0f, 50.0f);
        z.wdt_ms = 0u;
        z.frozen = true;
        t0 = z.t;
        while (!z.mon.tripped && z.t < t0 + 1000u) sbsim_run(&z, z.t + 10u, NULL, NULL);
        uint32_t t_fr = z.t - t0;
        sbsim_run(&z, z.t + 100u, NULL, NULL);
        sim_hal_can_act(1u, &a1);
        bool frozen = z.mon.reason == MON_TRIP_CAN_TX && t_fr >= 90u && t_fr <= 140u && a1.n_cpu1 > 0u && a1.last_cpu1 <= -0.999f;
        z.frozen = false;
        /* core 0 alive, its actuator task stuck */
        static sbsim_t q;
        bool drv4 = can_drive(&q, 2309u, 12.0f, 50.0f);
        sim_hal_can_task_stuck(true);
        t0 = q.t;
        while (!q.mon.tripped && q.t < t0 + 1000u) sbsim_run(&q, q.t + 10u, NULL, NULL);
        uint32_t t_st = q.t - t0;
        sbsim_run(&q, q.t + 300u, NULL, NULL);
        bool stuck = q.mon.reason == MON_TRIP_CAN_TX && t_st <= 140u && q.sb.mon_tripped && sb_mode(&q.sb) == MODE_STANDBY;
        sim_hal_can_task_stuck(false);
        snprintf(d, sizeof d, "driving %s; injected 90 %%: trip after %u ms, CPU1 throttle IDLE frame %s, core 0's frames "
                 "stopped %s, steering held at %.3f %s, gear held in F %s, then N after the travel time (servo idle, no shift "
                 "under load) %s; throttle node lost (driving %s): gear held for good %s; core 0 frozen (%s): MON_TRIP_CAN_TX "
                 "after %u ms + CPU1 frames %s; actuator task stuck (%s): trip after %u ms, STANDBY %s",
                 drv ? "yes" : "NO", (unsigned)(t_trip - j.at_ms), fast ? "yes" : "NO", core0_stopped ? "yes" : "NO",
                 (double)steer_last, steer_hold ? "yes" : "NO", gear_hold ? "yes" : "NO", gear_n ? "yes" : "NO",
                 drv2 ? "yes" : "NO", held_forever ? "yes" : "NO", drv3 ? "yes" : "NO", (unsigned)t_fr, frozen ? "yes" : "NO",
                 drv4 ? "yes" : "NO", (unsigned)t_st, stuck ? "yes" : "NO");
        test_report("CN17 CPU1 safe frames: latch, core 0 cancelled, throttle idle within 20 ms, steering hold, gear hold "
                    "then N (node OK), held for good with the throttle node lost; MON_TRIP_CAN_TX",
                    drv && fast && core0_stopped && steer_hold && gear_hold && gear_n && drv2 && held_forever && drv3 &&
                    frozen && drv4 && stuck, d);
    }

    /* CN18 the UNCREWED dead-man (OA-3, TO CONFIRM): CPU1 pulses only while UNCREWED + no trip + core 0 alive + the
     * throttle node OPERATIONAL + frames going out + no kill latch; a trip / frozen core 0 / the throttle node lost =
     * no pulses = the timer relay stops the engine (~1.5 s); CREWED: never a pulse */
    {
        static sbsim_t x;
        static llsim_t l;
        inject_t j = { 1, 0u, false, 0u };
        bool drv = can_remote_drive(&x, &l, 2310u, false);
        sim_hal_deadman_arm(true);
        uint32_t e0 = sim_hal_deadman_edges();
        sbsim_run(&x, x.t + 5000u, ll_hook, &l);
        bool pulsing = sim_hal_deadman_edges() > e0 + 50u && !sim_hal_deadman_killed();
        j.at_ms = x.t + 1u;
        uint32_t t0 = x.t, t_trip = 0u, t_dead = 0u;
        while (x.t < t0 + 3000u) {
            sbsim_run(&x, x.t + 10u, inject_hook, &j);
            if (!t_trip && hal_outputs_forced()) t_trip = x.t;
            if (!t_dead && sim_hal_deadman_killed()) t_dead = x.t;
        }
        bool trip = t_trip && t_dead && t_dead - t_trip <= 1700u && t_dead - t_trip >= 1400u;
        sim_hal_deadman_arm(false);
        /* the throttle node lost: the kill latch (OD-3) and no more pulses */
        static sbsim_t y;
        static llsim_t m;
        bool drv2 = can_remote_drive(&y, &m, 2311u, false);
        sim_hal_deadman_arm(true);
        sbsim_run(&y, y.t + 1000u, ll_hook, &m);
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        sbsim_run(&y, y.t + 4000u, ll_hook, &m);
        bool node = sim_hal_engine_killed() && sim_hal_deadman_killed();
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        sim_hal_deadman_arm(false);
        /* frozen core 0 (watchdog on): the reset + the detection window = no pulses: the relay stops the engine */
        static sbsim_t z;
        static llsim_t n;
        bool drv3 = can_remote_drive(&z, &n, 2312u, false);
        sim_hal_deadman_arm(true);
        sbsim_run(&z, z.t + 1000u, ll_hook, &n);
        z.frozen = true;
        sbsim_run(&z, z.t + 3000u, ll_hook, &n);
        bool frozen = sim_hal_deadman_killed() && z.resets >= 1u;
        sim_hal_deadman_arm(false);
        /* CREWED: never a pulse */
        static sbsim_t c;
        bool drv4 = can_drive(&c, 2313u, 12.0f, 30.0f);
        e0 = sim_hal_deadman_edges();
        sbsim_run(&c, c.t + 5000u, NULL, NULL);
        bool crewed = drv4 && sim_hal_deadman_edges() == e0 && c.sb.profile == SB_PROFILE_CREWED;
        snprintf(d, sizeof d, "UNCREWED driving %s: pulses %s; a trip: engine stopped by the relay %u ms after the trip %s; "
                 "throttle node lost (driving %s): kill latch + relay %s; frozen core 0 (driving %s): reset, relay %s; CREWED "
                 "(driving %s): no pulse %s", drv ? "yes" : "NO", pulsing ? "yes" : "NO",
                 (unsigned)(t_dead ? t_dead - t_trip : 0u), trip ? "yes" : "NO", drv2 ? "yes" : "NO", node ? "yes" : "NO",
                 drv3 ? "yes" : "NO", frozen ? "yes" : "NO", drv4 ? "yes" : "NO", crewed ? "yes" : "NO");
        test_report("CN18 UNCREWED dead-man: pulses only while everything is right; trip / frozen core 0 / throttle node "
                    "lost = engine stopped by the timer relay; CREWED never pulses",
                    drv && pulsing && trip && drv2 && node && drv3 && frozen && crewed, d);
    }

    /* CN19 BENCH TEST on the position servo: steering steps (the command moves at the worst-case speed for the step
     * time), no frame before the first step, throttle idle + gear N the whole time */
    {
        static sbsim_t x;
        static link_t usb;
        sm_boot(&x, 2314u, 12.0f, 0.03f, NULL);
        link_init(&usb, &x.sb);
        sbsim_run(&x, 12000u, NULL, NULL);
        uint8_t r_in = bench_cmd(&x, &usb, SB_BENCH_ENTER, 0u);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool have0;
        (void)sim_hal_steer_cmd(&have0);
        uint8_t r_l = bench_cmd(&x, &usb, SB_BENCH_LEFT, 300u);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool have1;
        float u1 = sim_hal_steer_cmd(&have1);
        float want = -SM_RATE_WORST * 0.3f;
        uint8_t r_r = bench_cmd(&x, &usb, SB_BENCH_RIGHT, 300u);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        float u2 = sim_hal_steer_cmd(NULL);
        bool safe = sim_hal_throttle() <= 0.0f && sim_hal_gear_pwm() == x.sb.cfg.gear.pwm_n_us;
        snprintf(d, sizeof d, "enter %u; no steering frame before the first step %s; left 300 ms: %u -> command %.4f (want "
                 "%.4f); right 300 ms: %u -> %.4f; throttle idle + N %s", r_in, !have0 ? "yes" : "NO", r_l, (double)u1,
                 (double)want, r_r, (double)u2, safe ? "yes" : "NO");
        test_report("CN19 BENCH TEST on the DroneCAN position servo: steering steps at the worst-case speed, nothing "
                    "before the first step, idle + neutral",
                    r_in == PROTO_R_OK && !have0 && r_l == PROTO_R_OK && have1 && fabsf(u1 - want) < 0.012f &&
                    r_r == PROTO_R_OK && fabsf(u2) < 0.012f && safe, d);
        bench_cmd(&x, &usb, SB_BENCH_LEAVE, 0u);
    }

    /* CN20 e-stop (OD-1): the engine stops through the kill circuit in EVERY profile (the sim world: no thrust); the
     * software ENGINE KILL latch is NOT set by it; STANDBY with idle commanded at once */
    {
        static sbsim_t x;
        bool drv = can_drive(&x, 2315u, 12.0f, 60.0f);
        float v0 = x.w.speed_kn;
        sim_hal_set_estop(true, true);
        uint32_t t0 = x.t;
        while (sb_mode(&x.sb) != MODE_STANDBY && x.t < t0 + 1000u) sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
        bool at_once = sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f;
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        bool crewed = at_once && x.w.speed_kn < 0.5f && !sim_hal_engine_killed() && x.sb.ll.state != SB_LL_KILLED;
        sim_hal_set_estop(true, false);
        static sbsim_t y;
        static llsim_t l;
        bool drv2 = can_remote_drive(&y, &l, 2316u, false);
        sim_hal_set_estop(true, true);
        sbsim_run(&y, y.t + 3000u, ll_hook, &l);
        bool unc = sb_mode(&y.sb) == MODE_STANDBY && !sim_hal_engine_killed() && y.sb.ll.state != SB_LL_KILLED &&
                   sim_hal_estop_pressed();
        sim_hal_set_estop(false, false);
        snprintf(d, sizeof d, "CREWED driving %s at %.1f kn: e-stop -> STANDBY with idle at once %s, the engine stopped "
                 "(%.1f kn after 20 s), the software kill latch not set %s; UNCREWED (driving %s): STANDBY, no software kill %s",
                 drv ? "yes" : "NO", (double)v0, at_once ? "yes" : "NO", (double)x.w.speed_kn, crewed ? "yes" : "NO",
                 drv2 ? "yes" : "NO", unc ? "yes" : "NO");
        test_report("CN20 e-stop: the engine stops (kill circuit) in every profile, never the software kill latch; "
                    "STANDBY with idle at once", drv && crewed && drv2 && unc, d);
    }

    /* CN21 asymmetric throttle rates + the model (slow 6 s and fast 2 s servos): up <= 15 %/s, down 50 %/s in normal
     * driving, an emergency = idle at once; the gear moves only when the SIMULATED servo is at idle */
    {
        const float travel[2] = { 12.0f, 3.0f };
        int good = 0, len = 0;
        for (int k = 0; k < 2; k++) {
            static sbsim_t x;
            bool drv = can_drive(&x, 2317u + (uint32_t)k, travel[k], 10.0f);
            ratew_t ru, rd;
            memset(&ru, 0, sizeof ru);
            memset(&rd, 0, sizeof rd);
            sb_test_drive(&x.sb, true, GEAR_F, 80.0f);
            uint32_t t0 = x.t;
            while (x.t < t0 + 8000u) {
                sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
                ratew(&ru, sim_hal_throttle(), x.sb.now_ms);
            }
            sb_test_drive(&x.sb, true, GEAR_F, 20.0f);
            t0 = x.t;
            while (x.t < t0 + 3000u) {
                sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
                ratew(&rd, sim_hal_throttle(), x.sb.now_ms);
            }
            float up_max = ru.max, dn_max = rd.max;
            sb_test_drive(&x.sb, true, GEAR_F, 80.0f);
            sbsim_run(&x, x.t + 6000u, NULL, NULL);
            float thr0 = sim_hal_throttle();
            events_t ev;
            ev_reset(&ev, x.t, thr0, x.sb.cfg.gear.pwm_n_us);
            sim_hal_press(HAL_BTN_STANDBY);
            t0 = x.t;
            while (x.t < t0 + 12000u) {
                sbsim_run(&x, x.t + AP_PERIOD_MS, NULL, NULL);
                ev_watch(&x, &ev);
            }
            float model = (thr0 - 1.0f) * 0.01f * x.sb.servo.cfg.travel_s;
            /* (the pulse is whole microseconds: 0.1 % per step = 2 %/s of quantisation) */
            bool ok = drv && up_max <= x.sb.cfg.throttle.rate_up_pct_s + 2.1f && up_max >= 14.0f &&
                      dn_max <= x.sb.cfg.throttle.rate_down_pct_s + 2.1f && dn_max >= 45.0f && ev.t_idle <= 0.1f &&
                      ev.t_neutral >= model + 1.0f - 0.1f && ev.t_true_idle >= 0.0f && ev.t_true_idle < ev.t_neutral &&
                      x.w.shift_under_load == 0;
            good += ok;
            len += snprintf(d + len, sizeof d - (size_t)len, "%sservos %.0f s steering: up max %.1f %%/s, down max %.1f %%/s; "
                            "STANDBY at %.0f %%: idle commanded %.2f s, the real servo idle %.2f s, neutral %.2f s (model %.1f s "
                            "+ wait), under load %d", k ? "; " : "", (double)travel[k], (double)up_max, (double)dn_max,
                            (double)thr0, (double)ev.t_idle, (double)ev.t_true_idle, (double)ev.t_neutral, (double)model,
                            x.w.shift_under_load);
        }
        test_report("CN21 asymmetric rates (up 15 / down 50 %/s, emergency at once) + the gear only at the modelled idle "
                    "- slow and fast servos", good == 2, d);
    }

    /* CN22 thr_travel_s (OD-4): longer at once; SHORTER only as an owner-signed value - STANDBY + the joystick CONFIRM,
     * logged; RAM only (a reboot = the worst case 6 s); nothing measures it shorter by itself */
    {
        static pcsim_t p;
        bool keep = s_can_on;
        s_can_on = true;
        pcsim_init(&p, 2318u);
        s_can_on = keep;
        pc_run(&p, 12000u);
        uint8_t id = 0xFFu;
        for (uint8_t i = 0; i < link_param_count(); i++)
            if (strcmp(link_param_info(i)->name, "thr_trav_s") == 0) id = i;
        uint8_t ps[5];
        proto_wr_t w;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 8.0f);
        uint8_t r8 = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u);
        bool longer = r8 == PROTO_R_OK && p.x.sb.servo.cfg.travel_s == 8.0f;
        uint16_t lg0 = p.x.sb.steer.log_seq;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 3.0f);
        uint8_t r3 = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u);
        bool pending = r3 == PROTO_R_OK && p.x.sb.servo.cfg.travel_s == 8.0f && p.x.sb.confirm.kind == CONF_THR_TRAVEL &&
                       p.x.sb.steer.log_what == 5u && p.x.sb.steer.log_why == 30u && p.x.sb.steer.log_seq != lg0;   /* 11g */
        lg0 = p.x.sb.steer.log_seq;
        pc_run(&p, 11000u);
        bool late = p.x.sb.servo.cfg.travel_s == 8.0f && p.x.sb.confirm.kind == CONF_NONE;
        (void)pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u);
        bool conf = confirm_press(&p.x);
        bool signed_ok = conf && p.x.sb.servo.cfg.travel_s == 3.0f && p.x.sb.steer.log_seq != lg0 &&
                         p.x.sb.steer.log_what == 4u && sb_mode(&p.x.sb) == MODE_STANDBY;
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 0.5f);
        uint8_t r05 = pc_cmd(&p, PROTO_T_PARAM_SET, ps, 5u);
        bool hdg = hdg_lock(&p.x);
        pc_run(&p, 1000u);                                  /* (the link's telemetry backlog out first) */
        proto_wr_init(&w, ps, sizeof ps); proto_wr_u8(&w, id); proto_wr_f32(&w, 10.0f);
        uint8_t rh = (uint8_t)sb_thr_travel_set(&p.x.sb, 10.0f);      /* (driving: refused) */
        press(&p.x, HAL_BTN_STANDBY);
        pc_run(&p, 2000u);
        p.x.frozen = true;                                  /* a reset: the worst case again */
        pc_run(&p, 300u);
        pc_run(&p, 2000u);
        bool reboot = p.x.sb.servo.cfg.travel_s == SERVO_THR_TRAVEL_S;
        snprintf(d, sizeof d, "param id %u: 8 s -> %u (taken at once %s); 3 s -> %u (CONFIRM pending, 8 s kept %s); no "
                 "CONFIRM in 10 s: still 8 s %s; again + CONFIRM: 3 s, logged (STEERMODEL 5 asked, 4 taken) %s; 0.5 s -> %u (bad value); in "
                 "HDG LOCK (%s) -> %u (STANDBY only); after a reset %.1f s (worst case) %s",
                 id, r8, longer ? "yes" : "NO", r3, pending ? "yes" : "NO", late ? "yes" : "NO", signed_ok ? "yes" : "NO",
                 r05, hdg ? "yes" : "NO", rh, (double)p.x.sb.servo.cfg.travel_s, reboot ? "yes" : "NO");
        test_report("CN22 thr_travel_s: longer at once, shorter only with the CONFIRM (owner-signed, logged), STANDBY "
                    "only, RAM only", id != 0xFFu && longer && pending && late && signed_ok && r05 == PROTO_R_BAD_VALUE &&
                    hdg && rh == (uint8_t)SB_ERR_STANDBY_ONLY && reboot, d);
    }

    /* CN23 lost commands = every servo HOLDS (OA-1, the default SRV_CMD_TIME_OUT 0): during a watchdog reset no frame
     * goes out and the nodes stay where they are (throttle too); after the boot throttle idle first, the gear only
     * after the throttle travel time (the throttle node OPERATIONAL) - then the real throttle is idle */
    {
        static sbsim_t x;
        bool drv = can_drive(&x, 2319u, 12.0f, 50.0f);
        float t_pos, s_pos, g_out;
        (void)sim_hal_can_servo(DC_NODE_THROTTLE, NULL, &t_pos, NULL, NULL);
        (void)sim_hal_can_servo(DC_NODE_STEERING, NULL, &s_pos, NULL, NULL);
        (void)sim_hal_can_servo(DC_NODE_GEAR, NULL, NULL, &g_out, NULL);
        x.frozen = true;
        uint32_t t0 = x.t;
        while (!x.boot_until && x.t < t0 + 500u) sbsim_run(&x, x.t + 10u, NULL, NULL);
        sbsim_run(&x, x.t + 150u, NULL, NULL);              /* (in the boot) */
        float t2, s2, g2;
        (void)sim_hal_can_servo(DC_NODE_THROTTLE, NULL, &t2, NULL, NULL);
        (void)sim_hal_can_servo(DC_NODE_STEERING, NULL, &s2, NULL, NULL);
        (void)sim_hal_can_servo(DC_NODE_GEAR, NULL, NULL, &g2, NULL);
        bool hold = x.boot_until && fabsf(t2 - t_pos) < 0.02f && fabsf(s2 - s_pos) < 1e-3f && fabsf(g2 - g_out) < 1e-3f;
        sim_hal_can_act_clear();
        uint32_t tb = x.t;
        sbsim_run(&x, x.t + 12000u, NULL, NULL);
        sim_can_act_t a3;
        sim_hal_can_act(3u, &a3);
        bool order = sim_hal_can_first_act() == 1u && a3.any && a3.first_ms >= tb + 6000u && x.w.shift_under_load == 0 &&
                     fabsf(a3.first - wire_of(x.sb.cfg.gear.pwm_n_us)) < 1e-3f;
        snprintf(d, sizeof d, "driving %s at 50 %%; watchdog reset: throttle %.2f -> %.2f, steering %.3f -> %.3f, gear %.2f -> "
                 "%.2f during the boot (hold) %s; after the boot: throttle first, gear N %u ms later (>= 6000), no shift "
                 "under load %s", drv ? "yes" : "NO", (double)t_pos, (double)t2, (double)s_pos, (double)s2, (double)g_out,
                 (double)g2, hold ? "yes" : "NO", (unsigned)(a3.any ? a3.first_ms - tb : 0u), order ? "yes" : "NO");
        test_report("CN23 lost commands: every servo holds (the throttle too); after the boot throttle idle first, the gear "
                    "after the throttle travel time", drv && hold && order, d);
    }

    /* CN24 (storm seed 22): a node interlock holds a deflected stick in STANDBY; the STANDBY button's ACK clears the
     * interlock - the held stick must NOT get MANUAL from it (the STANDBY button always wins: a held stick is
     * ignored until centred); centred, then deflected again = MANUAL as usual */
    {
        static pcsim_t p;
        cn_setup(&p, 1124u, 0u);
        sim_hal_can_node_reboot(DC_NODE_STEERING);
        pc_run(&p, 3500u);                              /* lost + latched; OPERATIONAL again > 2 s */
        bool latched = sb_mode(&p.x.sb) == MODE_STANDBY && p.x.sb.can.node_block;
        joy_set(-0.9f, 0.0f, false, false);
        pc_run(&p, 1000u);
        bool refused = sb_mode(&p.x.sb) == MODE_STANDBY;
        sim_hal_press(HAL_BTN_STANDBY);                 /* = the ACK in STANDBY */
        pc_run(&p, 1500u);
        bool acked = !p.x.sb.can.node_block, stays = sb_mode(&p.x.sb) == MODE_STANDBY;
        joy_set(0.0f, 0.0f, false, false);
        pc_run(&p, 1000u);
        joy_set(-0.9f, 0.0f, false, false);
        pc_run(&p, 1000u);
        bool manual = sb_mode(&p.x.sb) == MODE_MANUAL;
        joy_set(0.0f, 0.0f, false, false);
        snprintf(d, sizeof d, "node rebooted: STANDBY + interlock %s; stick held: still STANDBY %s; STANDBY button = ACK: "
                 "interlock cleared %s, the held stick ignored (STANDBY) %s; centred + deflected: MANUAL %s",
                 latched ? "yes" : "NO", refused ? "yes" : "NO", acked ? "yes" : "NO", stays ? "yes" : "NO",
                 manual ? "yes" : "NO");
        test_report("CN24 the STANDBY button's ACK never hands MANUAL to a stick held through a node interlock",
                    latched && refused && acked && stays && manual, d);
    }

    /* CN25 (step 11g, owner review finding 1) a planned arrival stop (WHY_ARRIVED) comes down at rate_down; an
     * emergency while it runs - the STANDBY button, the e-stop, a required node lost - = idle AT ONCE in that step
     * (the boat is already in STANDBY: no mode change carries it) and the stop is no longer "planned" */
    {
        int n_ok = 0, len = 0;
        d[0] = 0;
        for (int k = 0; k < 3; k++) {
            static sbsim_t x;
            bool drv = can_drive(&x, 2325u + (uint32_t)k, 12.0f, 40.0f);
            sb_test_drive(&x.sb, false, GEAR_F, 0.0f);
            float o0 = x.sb.thr.out_pct;
            bool req = sb_request_mode(&x.sb, MODE_STANDBY, WHY_ARRIVED);
            sbsim_run(&x, x.t + 3u * AP_PERIOD_MS - 1u, NULL, NULL);
            float o1 = x.sb.thr.out_pct;
            bool planned = req && x.sb.stop_planned && o1 > 1.0f && o1 < o0 && !x.sb.thr.stop;
            if (k == 0) sim_hal_press(HAL_BTN_STANDBY);
            else if (k == 1) sim_hal_set_estop(true, true);
            else sim_hal_can_node_silent(DC_NODE_GEAR, true);
            uint32_t t0 = x.t;
            /* (a node is lost only after can_node_lost_ms: wait for it, then the very next step) */
            while (k == 2 && !x.sb.can.node_lost && x.t < t0 + 3000u) {
                o1 = x.sb.thr.out_pct;
                sbsim_run(&x, x.t + AP_PERIOD_MS - 1u, NULL, NULL);
            }
            if (k != 2) sbsim_run(&x, x.t + AP_PERIOD_MS - 1u, NULL, NULL);
            float o2 = x.sb.thr.out_pct;
            bool at_once = o2 == 0.0f && x.sb.thr.stop && !x.sb.stop_planned;
            sim_hal_set_estop(false, false);
            if (k == 2) sim_hal_can_node_silent(DC_NODE_GEAR, false);
            if (drv && planned && at_once) n_ok++;
            len += snprintf(d + len, sizeof d - (size_t)len, "%s: driving %s, arrival stop %.1f -> %.1f %% %s, emergency -> "
                            "%.1f %% %s; ", k == 0 ? "STANDBY button" : k == 1 ? "e-stop" : "gear node lost",
                            drv ? "yes" : "NO", (double)o0, (double)o1, planned ? "ok" : "NO", (double)o2,
                            at_once ? "ok" : "FAILED");
        }
        test_report("CN25 an emergency during a planned arrival stop: idle at once (STANDBY button, e-stop, node lost)",
                    n_ok == 3, d);
    }

    /* CN26 (step 11g, owner review finding 5) DroneCAN servos with a throttle calibration whose idle is not 1000 us:
     * the wire idle -1.0 (the boot frame, CPU1's safe frames) would not be its idle - the outputs never arm (NOT
     * ARMED), core 0 sends only the wire idle -1.0, a driving request is refused */
    {
        static sbsim_t x;
        static sb_config_t c;
        sb_default_config(&c);
        c.servo.idle_us = 1100u;
        sm_boot(&x, 2330u, 12.0f, 0.03f, &c);
        sim_hal_can_act_clear();
        sbsim_run(&x, 15000u, NULL, NULL);
        sim_can_act_t a;
        sim_hal_can_act(CA_ACT_THROTTLE, &a);
        bool hdg = hdg_lock(&x);
        bool unarmed = !x.sb.armed && (x.sb.alarm_bits & SB_ALM_NOT_ARMED) && x.sb.servo.cfg_bad;
        bool wire = a.any && a.n_core0 > 100u && a.first <= -0.999f && a.last <= -0.999f;
        snprintf(d, sizeof d, "idle_us 1100 on CAN: armed %s, NOT ARMED %s, cfg_bad %s; throttle frames %u, first %.3f last "
                 "%.3f (wire idle -1.0); HDG LOCK %s (mode %u)", x.sb.armed ? "YES" : "no",
                 (x.sb.alarm_bits & SB_ALM_NOT_ARMED) ? "yes" : "NO", x.sb.servo.cfg_bad ? "yes" : "NO",
                 (unsigned)a.n_core0, (double)a.first, (double)a.last, hdg ? "TAKEN" : "refused", (unsigned)sb_mode(&x.sb));
        /* (step 13b) a full_us that is not 2000 us: the same (the wire map would under-command the node) */
        static sbsim_t y;
        sb_default_config(&c);
        c.servo.full_us = 1800u;
        sm_boot(&y, 2332u, 12.0f, 0.03f, &c);
        sbsim_run(&y, 15000u, NULL, NULL);
        bool full_bad = !y.sb.armed && (y.sb.alarm_bits & SB_ALM_NOT_ARMED) && y.sb.servo.cfg_bad;
        size_t dl = strlen(d);
        snprintf(d + dl, sizeof d - dl, "; full_us 1800 on CAN: never armed %s", full_bad ? "yes" : "NO");
        test_report("CN26 DroneCAN throttle idle_us != 1000 / full_us != 2000: never armed, the wire stays at idle -1.0",
                    unarmed && wire && sb_mode(&x.sb) == MODE_STANDBY && full_bad, d);
    }

    /* CN27 (step 11g, found by the new storm I51 on the gear node, seed 14) after a trip CPU1 shifted to NEUTRAL (thr_travel_s,
     * throttle node OK); the throttle node lost AFTER that: the gear stays NEUTRAL - CPU1 never sends the pre-trip gear
     * (forward) again */
    {
        static sbsim_t x;
        inject_t j = { 1, 0u, false, 0u };
        bool drv = can_drive(&x, 2331u, 12.0f, 50.0f);
        sb_power_lower(&x.sb);
        sbsim_run(&x, x.t + 600u, NULL, NULL);
        j.at_ms = x.t + 1u;
        uint32_t t0 = x.t;
        while (!hal_outputs_forced() && x.t < t0 + 500u) sbsim_run(&x, x.t + 10u, inject_hook, &j);
        uint32_t t_trip = x.t;
        sbsim_run(&x, t_trip + 8000u, inject_hook, &j);
        float g1 = 0.0f, g2 = 0.0f;
        (void)sim_hal_can_servo(DC_NODE_GEAR, &g1, NULL, NULL, NULL);
        bool neutral = fabsf(g1 - wire_of(x.sb.cfg.gear.pwm_n_us)) < 1e-3f;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        sbsim_run(&x, x.t + 5000u, inject_hook, &j);
        (void)sim_hal_can_servo(DC_NODE_GEAR, &g2, NULL, NULL, NULL);
        bool lost = (x.sb.can.lost_mask & (1u << HAL_CAN_ROLE_THROTTLE)) != 0u;
        bool stays = fabsf(g2 - wire_of(x.sb.cfg.gear.pwm_n_us)) < 1e-3f && hal_outputs_forced();
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        snprintf(d, sizeof d, "driving %s; trip: the gear node at %.3f after the travel time (neutral %s); throttle node lost "
                 "%s: the gear node at %.3f (stays neutral %s)", drv ? "yes" : "NO", (double)g1, neutral ? "yes" : "NO",
                 lost ? "yes" : "NO", (double)g2, stays ? "yes" : "NO");
        test_report("CN27 CPU1 after a trip: once in NEUTRAL the gear stays NEUTRAL when the throttle node is lost",
                    drv && neutral && lost && stays, d);
    }

    /* CN28 (step 13b review) a trip, the throttle node lost at once (CPU1's idle frames can't reach it: it holds the open
     * throttle), back after more than the travel time: CPU1 shifts to NEUTRAL only thr_travel_s after the node's
     * RETURN (not after the trip) - the gear never shifts under a throttle that only now starts to close */
    {
        static sbsim_t x;
        inject_t j = { 1, 0u, false, 0u };
        bool drv = can_drive(&x, 2341u, 12.0f, 50.0f);
        sb_power_lower(&x.sb);
        sbsim_run(&x, x.t + 600u, NULL, NULL);
        j.at_ms = x.t + 1u;
        uint32_t t0 = x.t;
        while (!hal_outputs_forced() && x.t < t0 + 500u) sbsim_run(&x, x.t + 10u, inject_hook, &j);
        bool tripped = hal_outputs_forced();
        sim_hal_can_node_silent(DC_NODE_THROTTLE, true);
        float gf = 0.0f, g = 0.0f;
        (void)sim_hal_can_servo(DC_NODE_GEAR, &gf, NULL, NULL, NULL);
        bool fwd0 = fabsf(gf - wire_of(x.sb.cfg.gear.pwm_n_us)) > 0.05f;
        sbsim_run(&x, x.t + 10000u, inject_hook, &j);
        (void)sim_hal_can_servo(DC_NODE_GEAR, &g, NULL, NULL, NULL);
        bool held = fabsf(g - gf) < 1e-3f && !x.mon.node_ok;
        sim_hal_can_node_silent(DC_NODE_THROTTLE, false);
        uint32_t t_back = 0u, t_neutral = 0u, t1 = x.t;
        while (x.t < t1 + 30000u && !t_neutral) {
            sbsim_run(&x, x.t + 10u, inject_hook, &j);
            if ((x.t / 10u) % 50u == 0u) sb_alarm_ack(&x.sb);
            if (!t_back && x.mon.node_ok) t_back = x.t;
            (void)sim_hal_can_servo(DC_NODE_GEAR, &g, NULL, NULL, NULL);
            if (fabsf(g - wire_of(x.sb.cfg.gear.pwm_n_us)) < 1e-3f) t_neutral = x.t;
        }
        int32_t after = (t_back && t_neutral) ? (int32_t)(t_neutral - t_back) : -1;
        snprintf(d, sizeof d, "driving %s, trip %s, the gear forward %s; throttle node lost at once: the gear held %s for 10 s; "
                 "the node back (CPU1 sees it OK) %s; NEUTRAL %d ms after its return (travel %u ms)",
                 drv ? "yes" : "NO", tripped ? "yes" : "NO", fwd0 ? "yes" : "NO", held ? "yes" : "NO",
                 t_back ? "yes" : "NO", (int)after, (unsigned)x.mon.travel_ms);
        test_report("CN28 CPU1 after a trip: NEUTRAL only thr_travel_s after the throttle node's return (not the trip)",
                    drv && tripped && fwd0 && held && t_back && t_neutral && after + 30 >= (int32_t)x.mon.travel_ms, d);
    }
}

void sb_steer_tests(void)
{
    steer_model_tests();
    steer_model_tests2();
    can_servo_tests();
}

/* ------------------------------------------- AUTO RETURN (step 13, AR) -- */

/* AUTO due east from the start: waypoints 1-4 at 400 / 800 / 1200 / 1600 m, legs 10 / 12 / 14 / 16 kn,
 * radius 25 m, CONTINUE; the boat runs until waypoint 3 is the active one, 1000 m out (on leg 2 -> 3). */
static bool ar_setup(sbsim_t *x, uint32_t seed, program_t *orig, double *e0)
{
    static const float kn[4] = { 10.0f, 12.0f, 14.0f, 16.0f };
    prog_wp_t w[4];
    sbsim_init(x, seed, 90.0f, 0.0f, NULL);
    x->w.sea_state = 1.0f;
    sbsim_run(x, 3000u, NULL, NULL);
    *e0 = x->w.east_m;
    for (int i = 0; i < 4; i++)
        w[i] = wp_world(x, x->w.north_m, *e0 + 400.0 * (i + 1), kn[i], 25.0f, WP_ACT_CONTINUE);
    if (!auto_go(x, w, 4, PROG_END_STOP)) return false;
    while (x->t < 400000u && !(x->sb.autop.wp == 2u && x->w.east_m > *e0 + 1000.0)) sbsim_run(x, x->t + 100u, NULL, NULL);
    *orig = *sb_program(&x->sb);
    return sb_mode(&x->sb) == MODE_AUTO && x->sb.autop.wp == 2u;
}

static void return_tests(void)
{
    char d[700];
    printf("\n=== AUTO RETURN (step 13) ===\n");

    /* AR1 the way back: waypoint 2 then 1 at the speeds of the legs it drives back over, stop in neutral at
     * waypoint 1; nothing before the CONFIRM; the stored program never changes */
    {
        sbsim_t x;
        program_t orig;
        double e0;
        bool set = ar_setup(&x, 1301u, &orig, &e0);
        sb_result_t r = sb_auto_return(&x.sb);
        bool pend = x.sb.confirm.kind == CONF_AUTO_RETURN && !x.sb.autop.returning;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool still_fwd = !x.sb.autop.returning && x.sb.autop.wp == 2u && sb_mode(&x.sb) == MODE_AUTO;
        bool conf = confirm_press(&x);
        bool ret = x.sb.autop.returning && sb_auto_wp_stored(&x.sb) == 1u && x.sb.autop.prog.n == 2u &&
                   sb_mode(&x.sb) == MODE_AUTO && x.sb.autop.state == AUTO_RUN;
        float set1 = x.sb.cruise.set_kn;
        float min_e = 1e9f, set0 = -1.0f;
        bool passed2 = false;
        while (x.t < 900000u && sb_mode(&x.sb) == MODE_AUTO) {
            sbsim_run(&x, x.t + 100u, NULL, NULL);
            float e = (float)(x.w.east_m - e0);
            if (e < min_e) min_e = e;
            if (x.sb.autop.returning && sb_auto_wp_stored(&x.sb) == 0u) {
                if (!passed2) set0 = x.sb.cruise.set_kn;
                passed2 = true;
            }
        }
        float end_e = (float)(x.w.east_m - e0);
        bool same = memcmp(sb_program(&x.sb), &orig, sizeof orig) == 0;
        bool nv_same = false;
        {
            static program_t nvp;
            if (hal_nv_load(HAL_NV_SLOT_PROGRAM, &nvp, (uint32_t)sizeof nvp))
                nv_same = memcmp(&nvp, &orig, sizeof orig) == 0;
        }
        snprintf(d, sizeof d, "on leg 2 -> 3 %s; request %d, pending CONFIRM %s, before it still forward %s; CONFIRM %s -> "
                 "returning to waypoint 2 %s at %.0f kn (leg 2 -> 3: 14); waypoint 2 passed %s, then to waypoint 1 at "
                 "%.0f kn (leg 1 -> 2: 12); end %s (%s) %.0f m east of the start (waypoint 1 at 400 m), at %.1f kn, gear %s; "
                 "stored program unchanged %s, saved one unchanged %s",
                 set ? "yes" : "NO", (int)r, pend ? "yes" : "NO", still_fwd ? "yes" : "NO", conf ? "yes" : "NO",
                 ret ? "yes" : "NO", set1, passed2 ? "yes" : "NO", set0, mode_name(sb_mode(&x.sb)),
                 mode_reason_name(auto_end_why(&x)), end_e, x.w.speed_kn, x.w.engaged == GEAR_N ? "N" : "not N",
                 same ? "yes" : "NO", nv_same ? "yes" : "NO");
        test_report("AR1 AUTO RETURN: back over the legs to waypoint 1 after the CONFIRM, stop there; stored program kept",
                    set && r == SB_OK && pend && still_fwd && conf && ret && fabsf(set1 - 14.0f) < 0.01f && passed2 &&
                    fabsf(set0 - 12.0f) < 0.01f && sb_mode(&x.sb) == MODE_STANDBY && auto_end_why(&x) == WHY_ARRIVED &&
                    fabsf(end_e - 400.0f) < 80.0f && min_e > 300.0f && same && nv_same, d);
    }

    /* AR2 refusals: no program, FIRST TEST, a held stick, already returning, DP / TRAINING; a CONFIRM after 10 s
     * does nothing; the STANDBY button cancels a pending request */
    {
        sbsim_t x;
        program_t orig;
        double e0;
        sbsim_init(&x, 1302u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        sb_result_t r_noprog = sb_auto_return(&x.sb);
        sb_config_t c = cfg_first_test();
        sbsim_t y;
        sbsim_init(&y, 1303u, 90.0f, 0.0f, &c);
        sbsim_run(&y, 3000u, NULL, NULL);
        sb_result_t r_ft = sb_auto_return(&y.sb);

        bool set = ar_setup(&x, 1304u, &orig, &e0);
        joy_set(0.0f, 0.5f, false, false);                  /* stick held: MANUAL */
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        sb_result_t r_held = sb_auto_return(&x.sb);
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        sb_mode_t m_man = sb_mode(&x.sb);
        /* in MANUAL with the stick centred: the request is allowed (CONFIRM leaves MANUAL like AUTO START) */
        sb_result_t r_ok = sb_auto_return(&x.sb);
        sbsim_run(&x, x.t + 10500u, NULL, NULL);                    /* the CONFIRM window passes */
        bool expired = x.sb.confirm.kind == CONF_NONE;
        bool late = confirm_press(&x);
        bool not_ret = !x.sb.autop.returning && sb_mode(&x.sb) == MODE_MANUAL;
        sb_result_t r2 = sb_auto_return(&x.sb);
        press(&x, HAL_BTN_STANDBY);                                 /* STANDBY: the pending request goes */
        bool cancelled = x.sb.confirm.kind == CONF_NONE;
        sbsim_run(&x, x.t + 15000u, NULL, NULL);
        bool late2 = confirm_press(&x);
        /* in STANDBY: a new request, CONFIRM -> AUTO on the way back; then a second request is refused */
        sb_result_t r3 = sb_auto_return(&x.sb);
        bool c3 = confirm_press(&x);
        bool ret = x.sb.autop.returning && sb_mode(&x.sb) == MODE_AUTO;
        sb_result_t r_again = sb_auto_return(&x.sb);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 15000u, NULL, NULL);
        sb_result_t r_ret_stby = sb_auto_return(&x.sb);              /* a stopped return: still the way back */
        snprintf(d, sizeof d, "no program -> %d (%d), FIRST TEST -> %d (%d); stick held -> %d (%d), MANUAL %s; centred "
                 "-> %d, CONFIRM window passed %s, a late CONFIRM accepted %s, still forward in MANUAL %s; request %d, "
                 "STANDBY button cancels it %s, CONFIRM after it %s; from STANDBY %d + CONFIRM %s -> AUTO back %s; "
                 "again -> %d (%d); stopped return asked again -> %d (%d)",
                 (int)r_noprog, (int)SB_ERR_NO_PROGRAM, (int)r_ft, (int)SB_ERR_WRONG_MODE, (int)r_held,
                 (int)SB_ERR_NOT_CENTERED, m_man == MODE_MANUAL ? "yes" : "NO", (int)r_ok, expired ? "yes" : "NO",
                 late ? "YES" : "no", not_ret ? "yes" : "NO", (int)r2, cancelled ? "yes" : "NO", late2 ? "YES" : "no",
                 (int)r3, c3 ? "yes" : "NO", ret ? "yes" : "NO", (int)r_again, (int)SB_ERR_WRONG_MODE,
                 (int)r_ret_stby, (int)SB_ERR_WRONG_MODE);
        test_report("AR2 AUTO RETURN refusals: no program, FIRST TEST, held stick, CONFIRM window, STANDBY cancels, once",
                    r_noprog == SB_ERR_NO_PROGRAM && r_ft == SB_ERR_WRONG_MODE && set && r_held == SB_ERR_NOT_CENTERED &&
                    m_man == MODE_MANUAL && r_ok == SB_OK && expired && !late && not_ret && r2 == SB_OK && cancelled &&
                    !late2 && r3 == SB_OK && c3 && ret && r_again == SB_ERR_WRONG_MODE &&
                    r_ret_stby == SB_ERR_WRONG_MODE, d);
    }

    /* AR3 the joystick wins: takeover on the way back -> MANUAL (suspended), RESUME + CONFIRM goes on BACK;
     * then STANDBY + AUTO START + CONFIRM = the stored program forward from waypoint 1 again */
    {
        sbsim_t x;
        program_t orig;
        double e0;
        bool set = ar_setup(&x, 1305u, &orig, &e0);
        bool r = sb_auto_return(&x.sb) == SB_OK && confirm_press(&x);
        sbsim_run(&x, x.t + 20000u, NULL, NULL);
        joy_set(0.6f, 0.0f, false, false);
        sbsim_run(&x, x.t + 600u, NULL, NULL);
        bool man = sb_mode(&x.sb) == MODE_MANUAL && x.sb.autop.suspended && x.sb.autop.returning;
        joy_set(0.0f, 0.0f, false, false);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        sb_result_t rr = sb_auto_resume(&x.sb);
        bool c1 = confirm_press(&x);
        bool back = sb_mode(&x.sb) == MODE_AUTO && x.sb.autop.returning && sb_auto_wp_stored(&x.sb) <= 1u;
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 15000u, NULL, NULL);
        sb_result_t rs = sb_auto_start(&x.sb);
        bool c2 = confirm_press(&x);
        bool fwd = sb_mode(&x.sb) == MODE_AUTO && !x.sb.autop.returning && x.sb.autop.wp == 0u &&
                   memcmp(&x.sb.autop.prog, &orig, sizeof orig) == 0 &&
                   x.sb.autop.prog.wp[0].lat_e7 == orig.wp[0].lat_e7;
        snprintf(d, sizeof d, "return started %s; stick -> MANUAL, return suspended %s; RESUME %d + CONFIRM %s -> on the "
                 "way back again %s (target waypoint %u); STANDBY, AUTO START %d + CONFIRM %s -> the stored program from "
                 "waypoint 1 %s",
                 set && r ? "yes" : "NO", man ? "yes" : "NO", (int)rr, c1 ? "yes" : "NO", back ? "yes" : "NO",
                 (unsigned)sb_auto_wp_stored(&x.sb) + 1u, (int)rs, c2 ? "yes" : "NO", fwd ? "yes" : "NO");
        test_report("AR3 AUTO RETURN: the joystick wins; RESUME goes on back; AUTO START = the stored program again",
                    set && r && man && rr == SB_OK && c1 && back && rs == SB_OK && c2 && fwd, d);
    }

    /* AR6 (step 13b review) an AUTO RETURN confirmed inside AUTO keeps the rough-water slow-down and its memory:
     * the AUTO speed never jumps back to the full leg speed in the sea it just slowed down for */
    {
        sbsim_t x;
        program_t orig;
        double e0;
        bool set = ar_setup(&x, 1307u, &orig, &e0);
        /* as after rough water: nervous, the target stepped down to 8 kn, remembered (14 kn leg) */
        x.sb.nerv.nervous = true;
        x.sb.nerv.ceiling_kn = 8.0f;
        x.sb.nerv.ceiling_ms = x.sb.cfg.nerv_memory_ms;
        x.sb.cruise.target_kn = 8.0f;
        sbsim_run(&x, x.t + 200u, NULL, NULL);
        bool r = sb_auto_return(&x.sb) == SB_OK && confirm_press(&x);
        bool ret = x.sb.autop.returning && sb_mode(&x.sb) == MODE_AUTO;
        float tmax = 0.0f;
        uint32_t t0 = x.t;
        while (x.t < t0 + 5000u) {
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            tmax = fmaxf(tmax, x.sb.cruise.target_kn);
        }
        bool kept = x.sb.nerv.ceiling_ms > 0u && x.sb.nerv.ceiling_kn == 8.0f;
        snprintf(d, sizeof d, "AUTO on the 14 kn leg %s, slowed to 8 kn by rough water; RETURN + CONFIRM %s, returning %s "
                 "(leg speed %.0f kn); the target in the next 5 s at most %.1f kn; the rough-water memory kept %s",
                 set ? "yes" : "NO", r ? "yes" : "NO", ret ? "yes" : "NO", x.sb.cruise.set_kn, tmax, kept ? "yes" : "NO");
        test_report("AR6 AUTO RETURN inside AUTO keeps the rough-water slow-down + its memory (no jump to the leg speed)",
                    set && r && ret && x.sb.cruise.set_kn > 12.0f && tmax <= 8.01f && kept, d);
    }

    /* AR4 the same validation as an upload: a leg speed above today's max_leg_kn = refused (BAD_PROGRAM);
     * a new program upload ends a stopped return (the new one is the program) */
    {
        sbsim_t x;
        program_t orig;
        double e0;
        bool set = ar_setup(&x, 1306u, &orig, &e0);
        float keep = x.sb.cfg.prog.max_speed_kn;
        x.sb.cfg.prog.max_speed_kn = 13.0f;                 /* the way back to waypoint 2 runs at 14 kn */
        sb_result_t r_lim = sb_auto_return(&x.sb);
        x.sb.cfg.prog.max_speed_kn = keep;
        sb_result_t r_ok = sb_auto_return(&x.sb);
        bool c = confirm_press(&x);
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 15000u, NULL, NULL);
        prog_error_t err;
        uint8_t bad;
        sb_result_t up = sb_program_set(&x.sb, &orig, &err, &bad);
        bool cleared = !x.sb.autop.returning && memcmp(&x.sb.autop.prog, &orig, sizeof orig) == 0;
        snprintf(d, sizeof d, "max_leg_kn 13 -> %d (%d = refused: the way back to waypoint 2 at 14 kn); back to %.0f -> "
                 "%d, CONFIRM %s; STANDBY, a program upload %d ends the return %s",
                 (int)r_lim, (int)SB_ERR_BAD_PROGRAM, keep, (int)r_ok, c ? "yes" : "NO", (int)up, cleared ? "yes" : "NO");
        test_report("AR4 AUTO RETURN: the way back is validated like an upload; a new program ends it",
                    set && r_lim == SB_ERR_BAD_PROGRAM && r_ok == SB_OK && c && up == SB_OK && cleared, d);
    }

    /* AR5 the link: PROTO_T_AUTO_RETURN -> the request; telemetry = the stored program's count, the active waypoint
     * as its index in it, auto_flags b4; PROG_GET reads the stored program */
    {
        pcsim_t *p = (pcsim_t *)calloc(1, sizeof *p);
        pcsim_init(p, 1307u);
        p->heartbeat = true;
        pc_run(p, 3000u);
        double e0 = p->x.w.east_m;
        prog_wp_t w[4];
        for (int i = 0; i < 4; i++)
            w[i] = wp_world(&p->x, p->x.w.north_m, e0 + 400.0 * (i + 1), 10.0f, 25.0f, WP_ACT_CONTINUE);
        bool go = auto_go(&p->x, w, 4, PROG_END_STOP);
        while (p->x.t < 400000u && !(p->x.sb.autop.wp == 2u && p->x.w.east_m > e0 + 1000.0)) pc_run(p, 100u);
        uint8_t r = pc_cmd(p, PROTO_T_AUTO_RETURN, NULL, 0u);
        uint8_t kind = p->tm_len > 130u ? p->tm[130] : 0xFFu;
        bool c = confirm_press(&p->x);
        pc_run(p, 500u);
        uint8_t n = p->tm[126], wp = p->tm[127], flags = p->tm[129];
        int before = p->acks;
        uint8_t i3 = 3u;
        uint8_t rg = pc_cmd(p, PROTO_T_PROG_GET, &i3, 1u);
        snprintf(d, sizeof d, "AUTO %s, AUTO_RETURN -> %u, confirm kind %u (%d); CONFIRM %s; telemetry: program count %u "
                 "(4), active waypoint %u (1 = waypoint 2), flags 0x%02X (b4 returning); PROG_GET 4 -> %u (acks %d)",
                 go ? "yes" : "NO", r, kind, (int)CONF_AUTO_RETURN, c ? "yes" : "NO", n, wp, flags, rg, p->acks - before);
        test_report("AR5 AUTO RETURN over the link: request, telemetry in stored-program terms, PROG_GET",
                    go && r == PROTO_R_OK && kind == (uint8_t)CONF_AUTO_RETURN && c && n == 4u && wp == 1u &&
                    (flags & 16u) && rg == PROTO_R_OK, d);
        free(p);
    }
}

/* ------------------------------------- paddlewheel pulses (step 13, PW) -- */

/* hal/tc_pulse.c on synthetic pulses: the board's GPT12 counter = the integral of speed x K, 16 bits */
typedef struct {
    pulse_speed_t p;
    double edges;               /* exact edge count (fractional) */
    uint32_t t;
    int valid_steps, steps;
    float last;
    bool ok_now;
} pwsim_t;

static void pw_init(pwsim_t *w, double start_count)
{
    memset(w, 0, sizeof *w);
    pulse_init(&w->p, 4.9f, 40.0f, 1.0f, 1.0f);
    w->edges = start_count;
    w->t = 1000u;
}

/* run ms at speed kn (hz_override > 0: that pulse rate instead, e.g. noise; cut: no pulses) */
static void pw_run(pwsim_t *w, uint32_t ms, float kn, float hz_override, bool cut)
{
    for (uint32_t k = 0; k < ms; k += 50u) {
        w->t += 50u;
        if (!cut) w->edges += (hz_override > 0.0f ? hz_override : kn * 4.9f) * 0.05;
        float v;
        w->ok_now = pulse_update(&w->p, (uint16_t)((uint32_t)w->edges & 0xFFFFu), w->t, &v);
        w->steps++;
        if (w->ok_now) { w->valid_steps++; w->last = v; }
    }
}

static void pulse_tests(void)
{
    char d[400];
    printf("\n=== Paddlewheel pulses -> knots (hal/tc_pulse.c, step 13) ===\n");

    /* PW1 steady speeds, the 16-bit counter wraps */
    {
        pwsim_t w;
        float got[3];
        static const float kn[3] = { 10.0f, 0.5f, 25.0f };
        pw_init(&w, 65000.0);
        bool ok = true;
        for (int i = 0; i < 3; i++) {
            pw_run(&w, 6000u, kn[i], 0.0f, false);
            got[i] = w.ok_now ? w.last : -1.0f;
            ok = ok && w.ok_now && fabsf(got[i] - kn[i]) <= 0.03f * kn[i] + 0.15f;
        }
        snprintf(d, sizeof d, "10 kn -> %.2f, 0.5 kn -> %.2f, 25 kn -> %.2f (counter from 65000: wrapped %s)",
                 got[0], got[1], got[2], w.edges > 65536.0 ? "yes" : "NO");
        test_report("PW1 paddlewheel: steady speeds measured, 16-bit counter wrap", ok && w.edges > 65536.0, d);
    }
    /* PW2 a boat that stops (5 kn -> 0 over seconds): valid all the way, ends at 0 kn */
    {
        pwsim_t w;
        pw_init(&w, 0.0);
        pw_run(&w, 4000u, 5.0f, 0.0f, false);
        int bad = 0;
        float v = 5.0f;
        for (int k = 0; k < 600; k++) {          /* 30 s, tau 6 s */
            v += (0.0f - v) * 0.05f / 6.0f;
            pw_run(&w, 50u, v < 0.05f ? 0.0f : v, 0.0f, false);
            if (!w.ok_now) bad++;
        }
        pw_run(&w, 3000u, 0.0f, 0.0f, true);
        snprintf(d, sizeof d, "no-data steps while stopping %d; at rest %s %.2f kn; no-data events %lu",
                 bad, w.ok_now ? "valid" : "NO DATA", w.last, (unsigned long)w.p.no_data);
        test_report("PW2 paddlewheel: a stopping boat reads down to 0 kn (no false NO DATA)",
                    bad == 0 && w.ok_now && w.last < 0.05f && w.p.no_data == 0u, d);
    }
    /* PW3 the cable cut at 8 kn: NO DATA within two windows, stays so; pulses back: valid again */
    {
        pwsim_t w;
        pw_init(&w, 0.0);
        pw_run(&w, 4000u, 8.0f, 0.0f, false);
        bool before = w.ok_now;
        uint32_t t_cut = w.t, t_lost = 0u;
        for (int k = 0; k < 200 && !t_lost; k++) {
            pw_run(&w, 50u, 0.0f, 0.0f, true);
            if (!w.ok_now) t_lost = w.t - t_cut;
        }
        pw_run(&w, 10000u, 0.0f, 0.0f, true);
        bool stays = !w.ok_now;
        uint32_t t_back0 = w.t, t_back = 0u;
        for (int k = 0; k < 100 && !t_back; k++) {
            pw_run(&w, 50u, 8.0f, 0.0f, false);
            if (w.ok_now) t_back = w.t - t_back0;
        }
        snprintf(d, sizeof d, "valid before %s; NO DATA %u ms after the cut, still after 10 s %s; back %u ms after the "
                 "pulses returned (%.1f kn)", before ? "yes" : "NO", t_lost, stays ? "yes" : "NO", t_back, w.last);
        test_report("PW3 paddlewheel: cut cable / stuck wheel at speed = NO DATA (not 0 kn); back when pulses return",
                    before && t_lost > 0u && t_lost <= 2u * PULSE_WINDOW_MS && stays && t_back > 0u &&
                    t_back <= 2u * PULSE_WINDOW_MS && fabsf(w.last - 8.0f) < 0.5f, d);
    }
    /* PW4 implausible pulse rates (noise: 100 kn) and a long gap between the calls = NO DATA */
    {
        pwsim_t w;
        pw_init(&w, 0.0);
        pw_run(&w, 3000u, 6.0f, 0.0f, false);
        pw_run(&w, 1500u, 0.0f, 490.0f, false);
        bool noise = !w.ok_now;
        pw_run(&w, 3000u, 6.0f, 0.0f, false);
        bool back = w.ok_now;
        w.t += 3000u;                              /* the loop stalled 3 s */
        w.edges += 6.0 * 4.9 * 3.0;
        float v;
        bool gap = !pulse_update(&w.p, (uint16_t)((uint32_t)w.edges & 0xFFFFu), w.t, &v);
        pw_run(&w, 1500u, 6.0f, 0.0f, false);
        snprintf(d, sizeof d, "100 kn of pulses -> %s; 6 kn again -> %s; a 3 s gap -> %s, then %s (%.1f kn)",
                 noise ? "NO DATA" : "VALID", back ? "valid" : "NO DATA", gap ? "NO DATA" : "VALID",
                 w.ok_now ? "valid" : "NO DATA", w.last);
        test_report("PW4 paddlewheel: implausible pulse rates and a stalled reading = NO DATA, then back",
                    noise && back && gap && w.ok_now && fabsf(w.last - 6.0f) < 0.4f, d);
    }
    /* PW5 (step 13b review) a wheel cut / fouled AT REST at power-up: never a valid 0 kn; pulses later = valid */
    {
        pwsim_t w;
        pw_init(&w, 0.0);
        pw_run(&w, 20000u, 0.0f, 0.0f, true);          /* 20 s at rest, no pulse at all */
        int valid_at_rest = w.valid_steps;
        pw_run(&w, 3000u, 4.0f, 0.0f, false);          /* the wheel turns */
        bool back = w.ok_now && fabsf(w.last - 4.0f) < 0.4f;
        snprintf(d, sizeof d, "valid steps with no edge ever %d (want 0); pulses at 4 kn -> %s %.1f kn",
                 valid_at_rest, back ? "valid" : "NO DATA", w.last);
        test_report("PW5 paddlewheel: cut / fouled at rest = NO DATA until a window with edges (not a valid 0 kn)",
                    valid_at_rest == 0 && back, d);
    }
    /* PW6 (step 13b review) dr.c: a wheel that reads a valid 0 kn while the boat clearly moves is DEAD:
     * (a) trusted GPS SOG 6 kn, (b) dead reckoning with a gear and 30 % throttle; it is used again when it reads */
    {
        static dr_t dd;
        dr_config_t dc;
        dr_default_config(&dc);
        dr_init(&dd, &dc);
        dr_input_t in;
        memset(&in, 0, sizeof in);
        in.heading_ok = true;
        in.water_ok = true;
        in.water_kn = 0.0f;
        in.gps.fix = true;
        in.gps.sats = 10u;
        in.gps.lat_deg = 37.8;
        in.gps.lon_deg = 23.6;
        in.gps.sog_kn = 6.0f;
        in.gps.cog_deg = 0.0f;
        uint32_t t = 1000u, dead_a = 0u;
        bool gps_state = false;
        for (int k = 0; k < 200 && !dead_a; k++) {     /* (a) 10 s: GPS says 6 kn north, wheel 0 */
            t += 50u;
            in.now_ms = t;
            in.gps_new = (k % 4) == 0;
            in.gps.lat_deg += 6.0 * 0.5144 * 0.05 / 111320.0;
            dr_step(&dd, &in);
            gps_state = gps_state || dd.state == DR_GPS;
            if (dd.pw_dead && !dd.speed_sensor) dead_a = t - 1000u;
        }
        in.gps_new = false;
        in.water_kn = 5.8f;                            /* pulses again */
        t += 50u; in.now_ms = t; dr_step(&dd, &in);
        bool back_a = dd.speed_sensor && !dd.pw_dead;
        /* (b) no GPS (fix off), gear F at 30 %, wheel 0 */
        dr_init(&dd, &dc);
        in.gps.fix = false;
        in.gear = 1;
        in.throttle_pct = 30.0f;
        in.water_kn = 0.0f;
        uint32_t dead_b = 0u;
        int sensor_steps_after = 0;
        for (int k = 0; k < 400; k++) {
            t += 50u;
            in.now_ms = t;
            dr_step(&dd, &in);
            if (!dead_b && dd.pw_dead) dead_b = (uint32_t)(k + 1) * 50u;
            if (dead_b && dd.speed_sensor) sensor_steps_after++;
        }
        /* (c) at rest (gear N, throttle 0, no GPS): a 0 kn wheel stays in use */
        dr_init(&dd, &dc);
        in.gear = 0;
        in.throttle_pct = 0.0f;
        for (int k = 0; k < 400; k++) { t += 50u; in.now_ms = t; dr_step(&dd, &in); }
        bool rest_ok = dd.speed_sensor && !dd.pw_dead;
        snprintf(d, sizeof d, "(a) GPS 6 kn (trusted %s): dead after %u ms, back when it reads %s; (b) gear F 30 %%: "
                 "dead after %u ms, used after that %d steps; (c) at rest still used %s",
                 gps_state ? "yes" : "NO", dead_a, back_a ? "yes" : "NO", dead_b, sensor_steps_after,
                 rest_ok ? "yes" : "NO");
        test_report("PW6 paddlewheel: reads 0 while the boat moves (GPS SOG / throttle in gear) = DEAD, not used",
                    gps_state && dead_a >= dc.pw_dead_ms && dead_a <= dc.pw_dead_ms + 1000u && back_a &&
                    dead_b >= dc.pw_dead_ms && dead_b <= dc.pw_dead_ms + 200u && sensor_steps_after == 0 && rest_ok, d);
    }
}

/* -------------------- CONFIRM_ONLY (academy practical test, ACADEMY_SPEC G, step 21) -- */

/* what a CONFIRM_ONLY must never change: the mode, the power limit, every output, the alarms, the config */
typedef struct {
    sb_mode_t mode;
    float fwd, rev, thr, steer, target;
    uint16_t gear;
    uint32_t alarms, alarms2, cfg_version;
    bool prog_ok, raised;
    uint8_t helm;
} cosnap_t;

static cosnap_t co_snap(const sbsim_t *x)
{
    cosnap_t c;
    memset(&c, 0, sizeof c);
    c.mode = sb_mode(&x->sb);
    c.fwd = x->sb.plim.fwd_pct;
    c.rev = x->sb.plim.rev_pct;
    c.thr = sim_hal_throttle();
    c.steer = sim_hal_motor_command();
    c.target = x->sb.ap.target_heading;
    c.gear = sim_hal_gear_pwm();
    c.alarms = x->sb.alarm_bits;
    c.alarms2 = x->sb.alarm_bits2;
    c.cfg_version = x->sb.cfg_version;
    c.prog_ok = x->sb.autop.prog_ok;
    c.raised = plim_raised(&x->sb.plim);
    c.helm = x->sb.helm;
    return c;
}

static bool co_same(const cosnap_t *a, const cosnap_t *b)
{
    return a->mode == b->mode && a->fwd == b->fwd && a->rev == b->rev && fabsf(a->thr - b->thr) < 0.01f &&
           a->steer == b->steer && fabsf(a->target - b->target) < 0.01f && a->gear == b->gear &&
           a->alarms == b->alarms && a->alarms2 == b->alarms2 && a->cfg_version == b->cfg_version &&
           a->prog_ok == b->prog_ok && a->raised == b->raised && a->helm == b->helm;
}

/* TELEM2 group R = the last T2_R bytes: u8 state | u16 token */
static void co_t2(const pcsim_t *p, uint8_t *state, uint16_t *token)
{
    *state = p->t2_len >= T2_R ? p->t2[p->t2_len - T2_R] : 0xFFu;
    *token = p->t2_len >= T2_R ? (uint16_t)(p->t2[p->t2_len - 2u] | (p->t2[p->t2_len - 1u] << 8)) : 0u;
}

static void confirm_only_tests(void)
{
    char d[900];
    printf("\n=== CONFIRM_ONLY (academy practical test, step 21) ===\n");

    /* CO1 asked + answered by the onboard CONFIRM: NOTHING changes (STANDBY and HDG LOCK); TELEM2 group R shows
     * pending, then confirmed with the token; a second press (nothing pending) does nothing */
    {
        static pcsim_t p;
        pcsim_init(&p, 2101u);
        pc_run(&p, 12000u);
        const uint8_t TAKE = 0u;
        uint8_t r_helm = pc_cmd(&p, PROTO_T_HELM, &TAKE, 1u);      /* the laptop holds HELM before the snapshot */
        cosnap_t a = co_snap(&p.x);
        uint8_t tk[2] = { 0x34u, 0x12u };
        uint8_t r = pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        pc_run(&p, 300u);
        uint8_t st1; uint16_t tok1;
        co_t2(&p, &st1, &tok1);
        bool pend = r == PROTO_R_OK && p.x.sb.confirm.kind == CONF_CHECK && st1 == SB_CO_PENDING && tok1 == 0x1234u;
        bool conf = confirm_press(&p.x);
        pc_run(&p, 300u);
        uint8_t st2; uint16_t tok2;
        co_t2(&p, &st2, &tok2);
        cosnap_t b = co_snap(&p.x);
        bool same1 = co_same(&a, &b) && sb_mode(&p.x.sb) == MODE_STANDBY;
        uint32_t ign = p.x.sb.confirm.ignored;
        (void)confirm_press(&p.x);
        bool again = p.x.sb.confirm.ignored == ign + 1u && p.x.sb.conf_only.confirmed == 1u;
        bool hdg = hdg_lock(&p.x);
        pc_run(&p, 2000u);
        tk[0] = 0x35u;
        uint8_t r2 = pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        cosnap_t c = co_snap(&p.x);
        bool conf2 = confirm_press(&p.x);
        cosnap_t e = co_snap(&p.x);
        bool same2 = e.mode == MODE_HDG_LOCK && e.fwd == c.fwd && e.rev == c.rev && e.raised == c.raised &&
                     fabsf(e.target - c.target) < 0.01f && e.gear == c.gear && e.alarms == c.alarms &&
                     e.cfg_version == c.cfg_version && p.x.sb.conf_only.token == 0x1235u &&
                     p.x.sb.conf_only.state == SB_CO_CONFIRMED;
        press(&p.x, HAL_BTN_STANDBY);
        snprintf(d, sizeof d, "STANDBY: asked -> %u, pending + TELEM2 R (%u, 0x%04x) %s; CONFIRM -> %s, R (%u, 0x%04x); "
                 "mode / limits / throttle / gear / steering / alarms / config unchanged %s; a second press ignored %s; "
                 "HDG LOCK (%s): asked -> %u, CONFIRM %s, still HDG LOCK, target + limits unchanged %s",
                 r, st1, tok1, pend ? "yes" : "NO", conf ? "accepted" : "NO", st2, tok2, same1 ? "yes" : "NO",
                 again ? "yes" : "NO", hdg ? "yes" : "NO", r2, conf2 ? "accepted" : "NO", same2 ? "yes" : "NO");
        test_report("CO1 CONFIRM_ONLY answered by the onboard CONFIRM changes nothing on the boat (TELEM2 group R)",
                    r_helm == PROTO_R_OK && pend && conf && st2 == SB_CO_CONFIRMED && tok2 == 0x1234u && same1 && again && hdg &&
                    r2 == PROTO_R_OK && conf2 && same2, d);
    }

    /* CO2 refusals: never replaces another pending CONFIRM (that one stays), never in FIRST TEST, never UNCREWED
     * (no onboard joystick), only the HELM station, the length */
    {
        static pcsim_t p;
        pcsim_init(&p, 2102u);
        pc_run(&p, 12000u);
        uint8_t tk[2] = { 7u, 0u };
        sb_result_t rt = sb_thr_travel_set(&p.x.sb, 3.0f);         /* a shorter throttle travel: CONFIRM pending */
        uint8_t r_other = pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        bool kept = p.x.sb.confirm.kind == CONF_THR_TRAVEL && p.x.sb.conf_only.state == SB_CO_NONE;
        p.x.sb.confirm.kind = CONF_NONE;
        uint8_t r_len = pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 1u);
        static sbsim_t y;
        sb_config_t c = cfg_first_test();
        sbsim_init(&y, 2103u, 90.0f, 0.0f, &c);
        sbsim_run(&y, 8000u, NULL, NULL);
        sb_result_t r_ft = sb_confirm_only(&y.sb, SB_HELM_USB, 7u);
        static sbsim_t z;
        static rjsim_t rj;
        crew_init(&z, &rj, 2104u, false, NULL);
        const uint8_t TAKE = 0u;
        uint8_t r_nohelm = pi_cmd(&z, &rj.p, 3u, false, PROTO_T_CONFIRM_ONLY, tk, 2u);
        (void)pi_cmd(&z, &rj.p, 2u, false, PROTO_T_HELM, &TAKE, 1u);
        rj.rj_on = true;
        uint8_t r_unc = pi_cmd(&z, &rj.p, 2u, false, PROTO_T_CONFIRM_ONLY, tk, 2u);
        bool unc = z.sb.profile == SB_PROFILE_UNCREWED && z.sb.confirm.kind == CONF_NONE;
        crew_plug();
        snprintf(d, sizeof d, "another CONFIRM pending (thr_travel %d): -> %u, the other kept %s; 1 byte -> %u; FIRST TEST "
                 "-> %d; not the HELM station -> %u; UNCREWED (%s) -> %u", (int)rt, r_other, kept ? "yes" : "NO", r_len,
                 (int)r_ft, r_nohelm, unc ? "yes" : "NO", r_unc);
        test_report("CO2 CONFIRM_ONLY refused: another CONFIRM pending, FIRST TEST, UNCREWED, not HELM, bad length",
                    rt == SB_OK && r_other == PROTO_R_REFUSED && kept && r_len == PROTO_R_BAD_LENGTH &&
                    r_ft == SB_ERR_WRONG_MODE && r_nohelm == PROTO_R_NOT_HELM && unc && r_unc == PROTO_R_NOT_ALLOWED, d);
    }

    /* CO3 not answered in 10 s = LATE (a press after it does nothing); STANDBY entry / another request cancel it;
     * a CONFIRM with the stick off centre is refused (the question stays) */
    {
        static pcsim_t p;
        pcsim_init(&p, 2105u);
        pc_run(&p, 12000u);
        uint8_t tk[2] = { 9u, 0u };
        (void)pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        pc_run(&p, 10500u);
        bool late = p.x.sb.conf_only.state == SB_CO_LATE && p.x.sb.confirm.kind == CONF_NONE;
        cosnap_t a = co_snap(&p.x);
        (void)confirm_press(&p.x);
        cosnap_t b = co_snap(&p.x);
        bool after = p.x.sb.conf_only.state == SB_CO_LATE && co_same(&a, &b);
        bool hdg = hdg_lock(&p.x);
        (void)pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        press(&p.x, HAL_BTN_STANDBY);
        pc_run(&p, 300u);
        bool cancel1 = p.x.sb.conf_only.state == SB_CO_CANCELLED && sb_mode(&p.x.sb) == MODE_STANDBY;
        pc_run(&p, 3000u);
        (void)pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        uint16_t code = 0u;
        sb_result_t rc = sb_power_code(&p.x.sb, 0u, &code);
        sb_result_t rr = sb_power_raise(&p.x.sb, 0u, code);         /* a power raise request replaces the question */
        pc_run(&p, 300u);
        bool cancel2 = p.x.sb.conf_only.state == SB_CO_CANCELLED && p.x.sb.confirm.kind == CONF_RAISE_FWD;
        sb_power_lower(&p.x.sb);
        (void)pc_cmd(&p, PROTO_T_CONFIRM_ONLY, tk, 2u);
        bool off = true;
        if (!s_usb.on) {
            joy_set(0.0f, 0.6f, false, false);                      /* stick off centre */
            pc_run(&p, 300u);
            uint32_t ref = p.x.sb.confirm.refused;
            sim_hal_joystick_confirm(true);
            pc_run(&p, 120u);
            sim_hal_joystick_confirm(false);
            pc_run(&p, 100u);
            off = p.x.sb.confirm.refused == ref + 1u && p.x.sb.conf_only.state == SB_CO_PENDING;
            joy_set(0.0f, 0.0f, false, false);
        }
        press(&p.x, HAL_BTN_STANDBY);
        pc_run(&p, 2000u);
        snprintf(d, sizeof d, "no CONFIRM in 10 s: LATE %s, a press after it changes nothing %s; HDG LOCK (%s) + asked, "
                 "STANDBY button: CANCELLED %s; asked, then a power raise request (%d, %d): CANCELLED, the raise pending %s; "
                 "CONFIRM with the stick off centre: refused, still pending %s",
                 late ? "yes" : "NO", after ? "yes" : "NO", hdg ? "yes" : "NO", cancel1 ? "yes" : "NO", (int)rc, (int)rr,
                 cancel2 ? "yes" : "NO", s_usb.on ? "(USB: not run)" : (off ? "yes" : "NO"));
        test_report("CO3 CONFIRM_ONLY late / cancelled by STANDBY or another request / refused off centre",
                    late && after && hdg && cancel1 && rc == SB_OK && rr == SB_OK && cancel2 && off, d);
    }
}

/* only the CONFIRM_ONLY tests (debugging: `autopilot_sim co`) */
void sb_confirm_only_tests(void)
{
    confirm_only_tests();
}

/* only the paddlewheel pulse tests (debugging: `autopilot_sim pw`) */
void sb_pulse_tests(void)
{
    pulse_tests();
}

/* only the AUTO RETURN tests (debugging: `autopilot_sim ar`) */
void sb_return_tests(void)
{
    return_tests();
}

/* only the dead-reckoning / GPS integrity tests (debugging: `autopilot_sim dr`) */
void sb_dr_tests(void)
{
    gps_denied();
    gps_invalid_tests();
    gps_retrust_tests();
}

void sb_phase4_tests(void)
{
    phase4();
}

void sb_phase5_tests(void)
{
    phase5();
}


/* -------------------------------- relay board on DroneCAN (step 11f, CLAUDE.md T, design section 9) -- */

/* RB15-RB22: the DroneCAN relay board (can_relay.c + the simulated node 30) - in every suite (the RB1-RB14 run
 * on the Modbus board in "all" / "all --bridge" and on the DroneCAN board in "all --can" / "--can-fast"; "rb"
 * runs both) - and the STARTER's hardware time-limit relay (OD-9) on both buses */
static void relay_bus_tests(void)
{
    char d[1400];
    bool keep_rb_can = s_rb_can;
    printf("\n=== RB15-RB21 relay board on DroneCAN (can_relay.c, node 30) + the STARTER time-limit relay (OD-9) ===\n");

    /* RB15 the STARTER's hardware time-limit relay (6 s, OD-9 / W8), on BOTH buses: a normal 5 s hold is never
     * cut by it (6 s > crank_max_s 5 s); a welded STARTER contact (the AURIX's OFF has no effect) - the starter
     * motor is cut by the time-limit relay after 6 s */
    {
        bool good[2] = { false, false };
        char part[2][300];
        for (int b = 0; b < 2; b++) {
            static sbsim_t x;
            static ersim_t e;
            s_rb_can = b == 1;
            bool set = er_setup(&x, &e, 1725u + (uint32_t)b, true, NULL);
            bool can = rb_can();
            uint8_t ign[4] = { 1u, RELAY_CMD_ON, 0u, 0u };
            (void)pi_cmd(&x, &e.r.p, 2u, false, PROTO_T_RELAY, ign, 4u);
            sbsim_run(&x, x.t + 300u, er_hook, &e);
            bool cut_early = false;
            uint32_t t0 = x.t;
            while (x.t < t0 + 4800u) {
                (void)er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
                if (x.t > t0 + 400u && sim_hal_relay_state(2u) && !sim_hal_starter_cranking()) cut_early = true;
            }
            (void)er_cmd(&x, &e, 2u, ENG_ACT_RELEASE);
            sbsim_run(&x, x.t + 11000u, er_hook, &e);                          /* the cool-down */
            uint32_t t1 = x.t;
            (void)er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
            (void)er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
            bool on = sim_hal_relay_state(2u);
            sim_hal_relay_weld(2u, true);                                      /* the contact welds */
            (void)er_cmd(&x, &e, 2u, ENG_ACT_RELEASE);
            sbsim_run(&x, t1 + 5500u, er_hook, &e);
            bool still = sim_hal_relay_state(2u) && sim_hal_starter_cranking();
            sbsim_run(&x, t1 + 6400u, er_hook, &e);
            bool cut = sim_hal_relay_state(2u) && !sim_hal_starter_cranking();
            sim_hal_relay_weld(2u, false);
            sim_hal_relay_board_reset();
            good[b] = set && !cut_early && on && still && cut;
            snprintf(part[b], sizeof part[b], "%s: set up %s, 4.8 s hold never cut %s; welded: on %s, cranking at 5.5 s %s, "
                     "cut by the time-limit relay at 6.4 s %s", can ? "DroneCAN" : "Modbus", set ? "yes" : "NO",
                     !cut_early ? "yes" : "NO", on ? "yes" : "NO", still ? "yes" : "NO", cut ? "yes" : "NO");
            remove("nv7.bin");
        }
        s_rb_can = keep_rb_can;
        snprintf(d, sizeof d, "%s; %s", part[0], part[1]);
        test_report("RB15 STARTER hardware time-limit relay (OD-9, both buses): a 5 s hold never cut, a welded contact cut at 6 s",
                    good[0] && good[1], d);
    }

    s_rb_can = true;

    /* RB16 the wire: nothing after the boot; a new state = a 50 Hz burst of 10 frames, then the board holds it
     * (R-2) - no more frames, a hand switch at the board is never overwritten; a pulse = ON at 50 Hz until its
     * end (AURIX-timed), then the OFF burst */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1720u, NULL, &t) && rb_can();
        uint32_t f0 = sim_hal_can_bus_frames(2u);
        bool none_boot = f0 == 0u;
        bool cmd = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 400u, NULL, NULL);
        uint32_t burst = sim_hal_can_bus_frames(2u) - f0;
        bool on = sim_hal_relay_state(6u) && (x.sb.relays.states & rb_bit(6)) && (relays_flags(&x.sb.relays) & 64u) == 0u;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool quiet = sim_hal_can_bus_frames(2u) - f0 == burst && sim_hal_relay_state(6u);
        sim_hal_relay_force(6u, false);                                    /* switched off by hand at the board */
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool hand = !sim_hal_relay_state(6u) && !(x.sb.relays.states & rb_bit(6)) &&
                    sim_hal_can_bus_frames(2u) - f0 == burst;
        uint32_t f1 = sim_hal_can_bus_frames(2u);
        bool pulse = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 1000u) == SB_OK;
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool p_on = sim_hal_relay_state(4u);
        uint32_t half = sim_hal_can_bus_frames(2u) - f1;
        sbsim_run(&x, x.t + 900u, NULL, NULL);                             /* (before relays.c's own OFF at 1.5 s) */
        uint32_t pf = sim_hal_can_bus_frames(2u) - f1;
        bool p_off = !sim_hal_relay_state(4u);
        snprintf(d, sizeof d, "table %s; frames after the boot %u; ON: %u frames, on + readback %s, then quiet 2 s %s; hand "
                 "switch kept %s; 1 s pulse %s: %u frames in 0.5 s, on %s, %u frames in all (50 + the OFF burst 10), off %s",
                 loaded ? "yes" : "NO", (unsigned)f0, (unsigned)burst, on ? "yes" : "NO", quiet ? "yes" : "NO",
                 hand ? "yes" : "NO", pulse ? "ok" : "NO", (unsigned)half, p_on ? "yes" : "NO", (unsigned)pf,
                 p_off ? "yes" : "NO");
        test_report("RB16 DroneCAN wire: nothing after a boot, 10-frame burst at 50 Hz then the board holds, pulse = ON at 50 Hz",
                    loaded && none_boot && cmd && burst == CR_BURST_FRAMES && on && quiet && hand && pulse && p_on &&
                    half >= 22u && half <= 27u && pf >= 55u && pf <= 64u && p_off, d);
        remove("nv7.bin");
    }

    /* RB17 a board WITHOUT readback (no hardpoint.Status, R-4 not met): the COMMANDED states (TELEM2 M flag 64 "no
     * readback"), unknown after an AURIX power-on while the board kept running; an interlocked partner that is
     * unknown and not momentary blocks; a warm reset keeps the commanded states (no-clear RAM) and sends nothing;
     * a board reboot = every relay OFF (R-2), nothing re-sent; a board booting with the AURIX = all OFF known, the
     * power_up states applied */
    {
        static sbsim_t x;
        static relay_table_t t, tp;
        rb_table(&t, false, false);
        t.r[5].pair = 7u;                                                  /* NAV LIGHTS / BILGE PUMP interlocked */
        t.r[6].pair = 6u;
        relay_table_seal(&t);
        tp = t;
        tp.r[5].flags = RELAY_F_POWER_ON;
        relay_table_seal(&tp);
        bool loaded = rb_setup(&x, 1721u, NULL, &t) && rb_can();
        sim_hal_relay_board(true, 5u, SIM_RB_NO_READBACK);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool cmd_only = x.sb.relays.board_ok && (relays_flags(&x.sb.relays) & 64u) && x.sb.relays.known == 0u;
        bool pair_unknown = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_ERR_REFUSED;
        bool trim = sb_relay_cmd(&x.sb, 4u, RELAY_CMD_PULSE, 500u) == SB_OK;   /* momentary partner: off by R-3 */
        bool off7 = sb_relay_cmd(&x.sb, 7u, RELAY_CMD_OFF, 0u) == SB_OK;
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        bool on6 = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool shown = sim_hal_relay_state(6u) && (x.sb.relays.states & rb_bit(6)) && (x.sb.relays.known & 0x60u) == 0x60u;
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        /* a warm reset: the kept states, nothing sent */
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        uint32_t f0 = sim_hal_can_bus_frames(2u);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool kept = (x.sb.relays.known & 0x60u) == 0x60u && (x.sb.relays.states & rb_bit(6)) && sim_hal_relay_state(6u) &&
                    sim_hal_can_bus_frames(2u) == f0 && x.sb.relays.power_ups == 0u;
        /* the board reboots: every relay OFF, nothing re-sent */
        uint32_t e6 = sim_hal_relay_edges(6u);
        sim_hal_can_node_reboot(DC_NODE_RELAY);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool reboot = !sim_hal_relay_state(6u) && sim_hal_relay_edges(6u) == e6 && (x.sb.relays.known & 0x7Fu) == 0x7Fu &&
                      x.sb.relays.states == 0u && sim_hal_can_bus_frames(2u) == f0 && cr_stats()->board_reboots == 1u;
        /* the power_up table, then a power-on with the board booting at the same time: all OFF known, power_up ON */
        bool tp_ok = sb_relay_table(&x.sb, &tp) == SB_OK && confirm_press(&x);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        sim_hal_can_node_reboot(DC_NODE_RELAY);
        rb_power_cycle(&x, HAL_RESET_POWER_ON);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool po = tp_ok && x.sb.relays.power_ups == 1u && sim_hal_relay_state(6u) && (x.sb.relays.known & 0x7Fu) == 0x7Fu;
        /* a power-on while the board kept running: unknown, no power_up */
        rb_power_cycle(&x, HAL_RESET_POWER_ON);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool po2 = x.sb.relays.power_ups == 0u && x.sb.relays.known == 0u && sim_hal_relay_state(6u);
        sim_hal_relay_board(true, 5u, 0u);
        snprintf(d, sizeof d, "table %s; no Status: board ok, flag 64, all unknown %s; NAV LIGHTS refused while the BILGE PUMP "
                 "(not momentary) is unknown %s; TRIM pulse ok %s; BILGE OFF %s -> NAV LIGHTS ON %s, shown as commanded %s; "
                 "warm reset: kept, nothing sent %s; board reboot: OFF, not re-sent, all known OFF %s; power-on with the board "
                 "booting: all OFF known + power_up %s; power-on, board kept running: unknown, no power_up %s",
                 loaded ? "yes" : "NO", cmd_only ? "yes" : "NO", pair_unknown ? "yes" : "NO", trim ? "yes" : "NO",
                 off7 ? "ok" : "NO", on6 ? "ok" : "NO", shown ? "yes" : "NO", kept ? "yes" : "NO", reboot ? "yes" : "NO",
                 po ? "yes" : "NO", po2 ? "yes" : "NO");
        test_report("RB17 DroneCAN board without readback: commanded states, unknown partner blocks, kept over a warm reset, "
                    "reboot = OFF", loaded && cmd_only && pair_unknown && trim && off7 && on6 && shown && kept && reboot && po &&
                    po2, d);
        remove("nv7.bin");
        sim_hal_relay_board_reset();
    }

    /* RB18 the board's node health: not OPERATIONAL / ERROR = RELAY BOARD SILENT (advisory: no mode change, the
     * relay node is not a required node), WARNING = a notice only (the relays work) */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1722u, NULL, &t) && rb_can();
        bool hdg = hdg_lock(&x);
        uint32_t alm0 = x.sb.alarm_bits;
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_MAINTENANCE);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool maint = !x.sb.relays.board_ok && (x.sb.alarm_bits2 & SB_ALM2_RELAY_SILENT) && sb_mode(&x.sb) == MODE_HDG_LOCK &&
                     !(x.sb.alarm_bits2 & SB_ALM2_CAN_NODE) && x.sb.alarm_bits == alm0 &&
                     sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_ERR_REFUSED;
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_WARNING, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool warn = x.sb.relays.board_ok && sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        warn = warn && sim_hal_relay_state(6u) && x.sb.can.node[HAL_CAN_ROLE_RELAY].warning && sb_mode(&x.sb) == MODE_HDG_LOCK;
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_ERROR, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool err = !x.sb.relays.board_ok && relays_silent(&x.sb.relays) && sb_mode(&x.sb) == MODE_HDG_LOCK &&
                   x.sb.alarm_bits == alm0;
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool back = x.sb.relays.board_ok;
        press(&x, HAL_BTN_STANDBY);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        snprintf(d, sizeof d, "table %s, HDG LOCK %s; MAINTENANCE: SILENT advisory, mode / alarm word 1 kept, no CAN NODE LOST, "
                 "ON refused %s; WARNING: notice, the relay works %s; ERROR: SILENT, still HDG LOCK %s; OPERATIONAL: back %s",
                 loaded ? "yes" : "NO", hdg ? "yes" : "NO", maint ? "yes" : "NO", warn ? "yes" : "NO", err ? "yes" : "NO",
                 back ? "yes" : "NO");
        test_report("RB18 DroneCAN board health: not OPERATIONAL / ERROR = RELAY BOARD SILENT advisory only, WARNING = notice",
                    loaded && hdg && maint && warn && err && back, d);
        remove("nv7.bin");
    }

    /* RB19 bus loss and board power: bus-off = SILENT, an OFF asked meanwhile goes when the bus is back; the board
     * unpowered = its relays drop OFF (states unknown); powered again = it rebooted: all OFF known, nothing switched
     * back ON by itself */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1723u, NULL, &t) && rb_can();
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool on = sim_hal_relay_state(6u);
        sim_hal_can_fault(SIM_CAN_BUSOFF);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool off_asked = !x.sb.relays.board_ok && sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u) == SB_OK;
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool held = sim_hal_relay_state(6u) && (x.sb.relays.off_pending & rb_bit(6)) && relays_silent(&x.sb.relays);
        sim_hal_can_fault(0u);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool done = !sim_hal_relay_state(6u) && x.sb.relays.off_pending == 0u && x.sb.relays.board_ok;
        sb_alarm_ack(&x.sb);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool on2 = sim_hal_relay_state(6u);
        uint32_t e6 = sim_hal_relay_edges(6u);
        sim_hal_can_node(DC_NODE_RELAY, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool dropped = !sim_hal_relay_state(6u) && !x.sb.relays.board_ok && x.sb.relays.known == 0u &&
                       relays_silent(&x.sb.relays);
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 5000u, NULL, NULL);
        bool rebooted = x.sb.relays.board_ok && (x.sb.relays.known & 0x7Fu) == 0x7Fu && x.sb.relays.states == 0u &&
                        !sim_hal_relay_state(6u) && sim_hal_relay_edges(6u) == e6 && cr_stats()->board_reboots == 1u;
        snprintf(d, sizeof d, "table %s, ON %s; bus-off: SILENT, OFF asked %s, held + pending %s, done when the bus is back %s; "
                 "ON again %s; board unpowered: relay OFF, unknown, SILENT %s; powered again (reboot): all OFF known, "
                 "never back ON %s", loaded ? "yes" : "NO", on ? "yes" : "NO", off_asked ? "yes" : "NO",
                 held ? "yes" : "NO", done ? "yes" : "NO", on2 ? "yes" : "NO", dropped ? "yes" : "NO",
                 rebooted ? "yes" : "NO");
        test_report("RB19 DroneCAN bus loss + board power: SILENT, the OFF waits for the bus; unpowered / rebooted = OFF, "
                    "never back ON", loaded && on && off_asked && held && done && on2 && dropped && rebooted, d);
        remove("nv7.bin");
    }

    /* RB20 the STARTER on DroneCAN: hold-to-crank = ON at 50 Hz, the AURIX stops sending when core 0 freezes - the
     * board's command timeout (R-3, 300 ms) ends the crank; a watchdog reset while cranking: the STARTER off by
     * the board, NO frame after the boot, no power_up */
    {
        static sbsim_t x;
        static ersim_t e;
        bool set = er_setup(&x, &e, 1724u, true, NULL) && rb_can();
        uint8_t ign[4] = { 1u, RELAY_CMD_ON, 0u, 0u };
        (void)pi_cmd(&x, &e.r.p, 2u, false, PROTO_T_RELAY, ign, 4u);
        sbsim_run(&x, x.t + 300u, er_hook, &e);
        int ok = er_hold(&x, &e, 1000u);
        (void)er_cmd(&x, &e, 2u, ENG_ACT_CRANK);
        bool cranking = sim_hal_relay_state(2u);
        x.wdt_ms = 0u;                                   /* core 0 frozen, no watchdog */
        x.frozen = true;
        uint32_t t0 = x.t, t_off = 0u;
        while (x.t < t0 + 1500u) {
            sbsim_run(&x, x.t + 10u, NULL, NULL);
            if (!t_off && !sim_hal_relay_state(2u)) t_off = x.t - t0;
        }
        bool tripped = x.sb.eng.crank == ENG_CRANK_OFF || hal_outputs_forced();   /* (CPU1 trips on the frozen loop) */
        x.frozen = false;
        x.wdt_ms = 100u;
        x.next_ctrl = x.t;
        x.last_kick = x.t;
        remove("nv7.bin");
        /* a watchdog reset while cranking (a fresh boat) */
        static sbsim_t y;
        static ersim_t f;
        bool set2 = er_setup(&y, &f, 1726u, true, NULL);
        (void)pi_cmd(&y, &f.r.p, 2u, false, PROTO_T_RELAY, ign, 4u);
        sbsim_run(&y, y.t + 300u, er_hook, &f);
        int ok2 = er_hold(&y, &f, 600u);
        (void)er_cmd(&y, &f, 2u, ENG_ACT_CRANK);
        bool cranking2 = sim_hal_relay_state(2u);
        y.frozen = true;                                 /* frozen with the watchdog: a reset */
        uint32_t t1 = y.t, t_off2 = 0u, r0 = y.resets;
        while (y.t < t1 + 1500u) {
            sbsim_run(&y, y.t + 10u, NULL, NULL);
            if (!t_off2 && !sim_hal_relay_state(2u)) t_off2 = y.t - t1;
        }
        uint32_t f0 = sim_hal_can_bus_frames(2u);
        sbsim_run(&y, y.t + 3000u, NULL, NULL);
        bool after = y.resets == r0 + 1u && sim_hal_can_bus_frames(2u) == f0 && !sim_hal_relay_state(2u) &&
                     y.sb.relays.power_ups == 0u && sim_hal_relay_state(1u);
        snprintf(d, sizeof d, "set up %s; hold %d CRANKs, STARTER on %s; core 0 frozen: off after %u ms (R-3), the crank over "
                 "(monitor trip) %s; a fresh boat %s: hold %d, on %s; watchdog reset: off after %u ms, IGNITION kept, "
                 "nothing sent after the boot, no power_up %s",
                 set ? "yes" : "NO", ok, cranking ? "yes" : "NO", (unsigned)t_off, tripped ? "yes" : "NO",
                 set2 ? "yes" : "NO", ok2, cranking2 ? "yes" : "NO", (unsigned)t_off2, after ? "yes" : "NO");
        test_report("RB20 DroneCAN STARTER: AURIX-timed, the board's timeout ends it when core 0 freezes / resets; nothing "
                    "sent after the boot", set && ok >= 3 && cranking && t_off > 0u && t_off <= 400u && tripped && set2 &&
                    ok2 >= 2 && cranking2 && t_off2 > 0u && t_off2 <= 400u && after, d);
        remove("nv7.bin");
        sim_hal_relay_board_reset();
    }

    /* RB21 (step 11g, owner review findings 3 + 9) DroneCAN: an ON never waits for an absent board - a bus-off right
     * after an ON drops its burst (the relay never switches ON late when the bus is back; only an OFF waits); an ON
     * to a board that is not there is refused; the keep store follows every command at once (cr_set / cr_pulse);
     * the uptime test also after a WARM reset: a board that rebooted during the AURIX's warm reset = all OFF known
     * (the kept states are stale) */
    {
        static sbsim_t x;
        static relay_table_t t;
        rb_table(&t, false, false);
        bool loaded = rb_setup(&x, 1725u, NULL, &t) && rb_can();
        sim_hal_relay_board(true, 5u, SIM_RB_NO_READBACK);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        /* the keep store at once (no control step in between) */
        uint32_t kk = 0u, ko = 0u;
        bool c1 = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        bool store = c1 && sim_hal_cr_keep(&kk, &ko) && (kk & rb_bit(6)) && (ko & rb_bit(6));
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool on6 = sim_hal_relay_state(6u);
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_OFF, 0u);
        store = store && sim_hal_cr_keep(&kk, &ko) && (kk & rb_bit(6)) && !(ko & rb_bit(6));
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        /* ON, then the bus goes at once: the ON burst is dropped, never sent late */
        uint32_t e6 = sim_hal_relay_edges(6u), dr0 = cr_stats()->dropped;
        bool c2 = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sim_hal_can_fault(SIM_CAN_BUSOFF);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        bool absent_ref = sb_relay_cmd(&x.sb, 7u, RELAY_CMD_ON, 0u) != SB_OK;    /* (SILENT: refused) */
        bool absent_hal = !cr_set(7u, true) && !cr_pulse(4u, 300u);              /* the back-end refuses too */
        sim_hal_can_fault(0u);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool dropped = c2 && cr_stats()->dropped > dr0 && !sim_hal_relay_state(6u) && sim_hal_relay_edges(6u) == e6 &&
                       !sim_hal_relay_state(7u);
        sb_alarm_ack(&x.sb);
        /* NAV LIGHTS ON, then the board reboots DURING a warm reset of the AURIX: all OFF known after the boot */
        (void)sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        bool on_again = sim_hal_relay_state(6u) && (x.sb.relays.states & rb_bit(6));
        sim_hal_can_node_reboot(DC_NODE_RELAY);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        /* (step 13b, review LOW-2: a reboot AROUND the warm reset can't be told from one just before it - the kept ON
         * is UNKNOWN then, never "OFF"; RB22 has the proven case) */
        bool warm = !sim_hal_relay_state(6u) && !(x.sb.relays.known & rb_bit(6)) && x.sb.relays.states == 0u &&
                    x.sb.relays.power_ups == 0u;
        sim_hal_relay_board(true, 5u, 0u);
        snprintf(d, sizeof d, "table %s; keep store at once on ON / OFF %s; ON %s; ON + bus-off at once: burst dropped, never "
                 "ON late %s (dropped %u); ON to an absent board refused (relays %s, back-end %s); board rebooted during a "
                 "warm reset: ON before %s, the kept ON unknown after (not OFF) %s",
                 loaded ? "yes" : "NO", store ? "yes" : "NO", on6 ? "yes" : "NO", dropped ? "yes" : "NO",
                 (unsigned)(cr_stats()->dropped - dr0), absent_ref ? "yes" : "NO", absent_hal ? "yes" : "NO",
                 on_again ? "yes" : "NO", warm ? "yes" : "NO");
        test_report("RB21 DroneCAN: an ON never waits for an absent board, the keep store at every command, the uptime "
                    "test after a warm reset", loaded && store && on6 && dropped && absent_ref && absent_hal && on_again &&
                    warm, d);
        remove("nv7.bin");
        sim_hal_relay_board_reset();
    }

    /* RB22 (step 13b, reviews of 11g) DroneCAN relay board: (a) an ON burst cut by a bus loss after some frames
     * reached the board (it holds what it got: R-2) - an OFF burst is queued and goes when the bus is back, then the
     * state is known OFF; (b) SHUTOFF ON in the first second after a warm reset (the board not seen yet) is never
     * lost: sent when the board appears; queued longer than RELAY_SHUTOFF_RETRY_MS = dropped, never late; (c) a board
     * that rebooted shortly BEFORE a warm reset: the kept ON is UNKNOWN (the pair partner blocked), never "OFF";
     * a board that booted clearly AFTER the warm reset = all OFF */
    {
        static sbsim_t x;
        static relay_table_t t;
        s_rb_can = true;
        rb_table(&t, false, false);
        t.r[5].pair = 7u;                                                  /* NAV LIGHTS / BILGE PUMP interlocked */
        t.r[6].pair = 6u;
        relay_table_seal(&t);
        bool loaded = rb_setup(&x, 1727u, NULL, &t) && rb_can();
        sim_hal_relay_board(true, 5u, SIM_RB_NO_READBACK);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        /* (a) (the partner OFF first: unknown + not momentary blocks); the ON just before the board's next
         * NodeStatus says health ERROR: the burst is cut after its first frames */
        (void)sb_relay_cmd(&x.sb, 7u, RELAY_CMD_OFF, 0u);
        sbsim_run(&x, x.t + 500u, NULL, NULL);
        hal_can_node_t nn;
        uint32_t tw = x.t;
        do {
            sbsim_run(&x, x.t + 5u, NULL, NULL);
            (void)dc_node(HAL_CAN_ROLE_RELAY, &nn);
        } while (nn.age_ms > 10u && x.t < tw + 3000u);
        sbsim_run(&x, x.t + 900u, NULL, NULL);
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_ERROR, DC_MODE_OPERATIONAL);
        bool c1 = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        uint32_t t0 = x.t, dr0 = cr_stats()->dropped;
        while (!sim_hal_relay_state(6u) && x.t < t0 + 300u) sbsim_run(&x, x.t + 5u, NULL, NULL);
        bool got = sim_hal_relay_state(6u);
        sbsim_run(&x, x.t + 2000u, NULL, NULL);
        /* (the board in ERROR still takes frames: the queued OFF burst, sent while it is absent, switches it off;
         * after a bus loss it goes when the bus is back) */
        bool still_on = cr_stats()->dropped > dr0 && !x.sb.relays.board_ok;
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool a_ok = c1 && got && still_on && !sim_hal_relay_state(6u) && (x.sb.relays.known & rb_bit(6)) &&
                    !(x.sb.relays.states & rb_bit(6));
        sb_alarm_ack(&x.sb);
        /* (b) SHUTOFF ON at once after a warm reset (just after a NodeStatus: the next one ~1 s later) */
        tw = x.t;
        do {
            sbsim_run(&x, x.t + 5u, NULL, NULL);
            (void)dc_node(HAL_CAN_ROLE_RELAY, &nn);
        } while (nn.age_ms > 10u && x.t < tw + 3000u);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 50u, NULL, NULL);
        bool not_yet = !dc_node(HAL_CAN_ROLE_RELAY, &nn) || !nn.seen;      /* the board not seen yet */
        sb_result_t r_sh = sb_relay_cmd(&x.sb, 3u, RELAY_CMD_ON, 0u);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool b_ok = not_yet && r_sh == SB_OK && sim_hal_relay_state(3u) && x.sb.relays.shutoff_pending == 0u &&
                    x.sb.relays.shutoff_retried == 1u;
        (void)sb_relay_cmd(&x.sb, 3u, RELAY_CMD_OFF, 0u);
        sbsim_run(&x, x.t + 1000u, NULL, NULL);
        /* (b2) the board unpowered: SHUTOFF ON queued, dropped after the retry time, never sent late */
        sim_hal_can_node(DC_NODE_RELAY, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        sb_result_t r_q = sb_relay_cmd(&x.sb, 3u, RELAY_CMD_ON, 0u);
        bool queued = r_q == SB_OK && (x.sb.relays.shutoff_pending & rb_bit(3));
        sbsim_run(&x, x.t + RELAY_SHUTOFF_RETRY_MS + 500u, NULL, NULL);
        bool gone = x.sb.relays.shutoff_pending == 0u && x.sb.relays.shutoff_dropped == 1u;
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 5000u, NULL, NULL);
        bool b2_ok = queued && gone && !sim_hal_relay_state(3u);
        sb_alarm_ack(&x.sb);
        /* (c) the board reboots, NAV LIGHTS ON, a warm reset ~1.5 s later */
        sim_hal_can_node_reboot(DC_NODE_RELAY);
        sbsim_run(&x, x.t + 1500u, NULL, NULL);
        bool c_on = sb_relay_cmd(&x.sb, 6u, RELAY_CMD_ON, 0u) == SB_OK;
        sbsim_run(&x, x.t + 300u, NULL, NULL);
        c_on = c_on && sim_hal_relay_state(6u);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 3000u, NULL, NULL);
        bool unknown = sim_hal_relay_state(6u) && !(x.sb.relays.known & rb_bit(6));
        bool pair_blocks = sb_relay_cmd(&x.sb, 7u, RELAY_CMD_ON, 0u) == SB_ERR_REFUSED && !sim_hal_relay_state(7u);
        /* (c2) a warm reset with the board unpowered, the board powered 4 s later (booted clearly after us): all OFF */
        sim_hal_can_node(DC_NODE_RELAY, false, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        rb_power_cycle(&x, HAL_RESET_SYSTEM);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        sim_hal_can_node(DC_NODE_RELAY, true, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
        sbsim_run(&x, x.t + 4000u, NULL, NULL);
        bool proven = (x.sb.relays.known & 0x7Fu) == 0x7Fu && x.sb.relays.states == 0u && !sim_hal_relay_state(6u);
        sim_hal_relay_board(true, 5u, 0u);
        snprintf(d, sizeof d, "table %s; (a) ON reached the board %s, the board in ERROR: burst dropped %s, OK again: OFF delivered, "
                 "known OFF %s; (b) SHUTOFF ON 50 ms after a warm reset (board not seen: %s) -> %d, ON when it appears %s; "
                 "(b2) board unpowered: queued %s, dropped after %u ms %s, never ON late %s; (c) ON after a board reboot "
                 "%s, warm reset 0.3 s later: kept ON unknown (still on) %s, partner ON refused %s; (c2) board booted 4 s "
                 "after a warm reset: all OFF known %s",
                 loaded ? "yes" : "NO", got ? "yes" : "NO", still_on ? "yes" : "NO", a_ok ? "yes" : "NO",
                 not_yet ? "yes" : "NO", (int)r_sh, b_ok ? "yes" : "NO", queued ? "yes" : "NO",
                 (unsigned)RELAY_SHUTOFF_RETRY_MS, gone ? "yes" : "NO", b2_ok ? "yes" : "NO", c_on ? "yes" : "NO",
                 unknown ? "yes" : "NO", pair_blocks ? "yes" : "NO", proven ? "yes" : "NO");
        test_report("RB22 DroneCAN relays: a cut ON burst gets its OFF, SHUTOFF ON survives a warm reset start, a reboot "
                    "around a warm reset = unknown", loaded && a_ok && b_ok && b2_ok && c_on && unknown && pair_blocks &&
                    proven, d);
        remove("nv7.bin");
        sim_hal_relay_board_reset();
    }
    s_rb_can = keep_rb_can;
}

void sb_can_tests(void)
{
    can_tests();
}

void sb_dp_tests(void)
{
    dp_tests();
    dp_mode_tests();
}

void sb_engine_tests(void)
{
    engine_tests();
}

void sb_nv_tests(void)
{
    nv_tests();
}

void sb_battery_tests(void)
{
    battery_tests();
}

void sb_generator_tests(void)
{
    generator_tests();
}

void sb_tests(void)
{
    can_tests();
    can_servo_tests();
    steer_model_tests();
    steer_model_tests2();
    dp_tests();
    dp_mode_tests();
    generator_tests();
    relay_tests();
    engine_tests();
    relay_bus_tests();
    battery_tests();
    nv_tests();
    gateway_tests();
    helm_tests();
    remote_tests();
    crew_tests();
    lostlink_tests();
    save_tests();
    ais_tests();
    nav_tests();
    return_tests();
    confirm_only_tests();
    pulse_tests();
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
    alarm_ack_tests();
    telem_size_tests();
    link_burst_tests();
    servo_clamp_tests();
    first_test_tests();
    bench_tests();
    phase4();
    phase5();
    phase6();
}
