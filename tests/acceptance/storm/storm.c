/*
 * storm.c - Acceptance test (Part 4.4): random fault storm on the AURIX
 * controller in the PC simulation, much faster than real time.
 *
 * Every episode = a seed -> a SCHEDULE of random events (joystick moves and
 * buttons, panel buttons, PC commands, remote driving data with dropouts,
 * sensor failures, corrupted / random frames on the USB link and the
 * gateway UART, gateway loss, navigation requests incl. obstacle caps,
 * slamming, chip resets, frozen core 0, injected unsafe outputs for the
 * CPU1 monitor). After EVERY control step the safety invariants are checked.
 * A failing episode is replayed with fewer events until the shortest
 * schedule that still fails remains (the "minimal reproduction").
 *
 *   storm.exe [episodes] [first_seed] [--min-min 5] [--max-min 30] [--quiet]
 *   storm.exe --replay SEED          one episode, every event printed
 *   storm.exe --wrap                 the 32-bit time wraparound run (Part 4.3)
 *   storm.exe --soak HOURS SEED      one long mixed run (Part 4.3)
 *
 * Built from app/ + sim/ (not changed) by build_storm.bat. Output: one line
 * per invariant (PASS / FAIL counts), worst reaction times, failing seeds.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "smallboat.h"
#include "powerlim.h"
#include "link.h"
#include "proto.h"
#include "monitor.h"
#include "timing.h"
#include "logq.h"
#include "logger.h"
#include "sim_world.h"
#include "hal_sim.h"
#include "logcore0.h"
void sim_hal_real_micros(bool on);   /* hal_sim.c: the PC clock for hal_micros() (timing) */
#include "sim_run.h"

/* sim_sb.c / sim_train.c need these from sim_main.c (not linked here) */
void test_report(const char *name, bool ok, const char *detail) { (void)name; (void)ok; (void)detail; }
void test_na(const char *name, const char *why) { (void)name; (void)why; }
results_t run(const scenario_t *sc) { results_t r; (void)sc; memset(&r, 0, sizeof r); return r; }   /* sim_train.c */

/* ------------------------------------------------------------------ rng -- */
static uint32_t rng_next(uint32_t *s)
{
    uint32_t x = *s ? *s : 0x9E3779B9u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}
static float rnd(uint32_t *s) { return (float)(rng_next(s) >> 8) / 16777216.0f; }
static int rndi(uint32_t *s, int n) { return (int)(rng_next(s) % (uint32_t)n); }

/* ------------------------------------------------------------- events -- */
typedef enum {
    E_JOY, E_BTN, E_TAKE, E_AUTOBTN, E_CONFIRM,
    E_PC_MODE, E_PC_AUTO_START, E_PC_AUTO_RESUME, E_PC_PAUSE, E_PC_PROG, E_PC_RAISE, E_PC_LOWER,
    E_REMOTE_REQ, E_REMOTE_ON, E_REMOTE_OFF, E_REMOTE_STALE, E_REMOTE_RELEASE,
    E_COMPASS, E_GYRO, E_GPS, E_GPS_JUMP, E_NSW, E_CURRENT, E_LIMITSW, E_JOY_WIRE, E_ACCEL, E_SEA,
    E_GW, E_NAVREQ, E_FUZZ_USB, E_FUZZ_GW,
    E_FREEZE, E_RESET, E_INJ_THR, E_INJ_GEAR,
    E_COUNT
} ev_kind_t;

static const char *EV_NAMES[E_COUNT] = {
    "joystick", "panel button", "TAKE", "AUTO button", "CONFIRM",
    "PC mode request", "PC AUTO start", "PC AUTO resume", "PC AUTO pause", "PC program", "PC power raise",
    "PC power lower", "remote request", "remote data on", "remote data off (dropout)", "remote data stale",
    "remote release", "compass", "gyro", "GPS", "GPS jump (spoof)", "neutral switch", "motor current sense",
    "limit switches", "joystick wire", "slamming sensor", "sea state", "gateway status", "navigation request",
    "USB link fuzz", "gateway UART fuzz", "core 0 frozen", "chip reset", "inject throttle over the limit",
    "inject gear change under load"
};

typedef struct {
    uint32_t t;                 /* ms */
    uint8_t  kind;
    int32_t  a, b, c;           /* parameters */
    float    f, g;
} ev_t;

#define MAX_EV 4000
typedef struct {
    uint32_t seed, dur_ms;
    float    sea0;
    bool     boot_deflected;    /* joystick held at power-up (I11) */
    int      n;
    ev_t     ev[MAX_EV];
    bool     on[MAX_EV];        /* used while shrinking */
} sched_t;

static void gen_schedule(sched_t *s, uint32_t seed, uint32_t dur_ms)
{
    uint32_t r = seed * 2654435761u + 1u;
    memset(s, 0, sizeof *s);
    s->seed = seed;
    s->dur_ms = dur_ms;
    s->sea0 = (float)rndi(&r, 5);
    s->boot_deflected = rnd(&r) < 0.15f;
    uint32_t t = 2000u;
    /* the boat should be doing something: a program and a power raise early on */
    while (t < dur_ms && s->n < MAX_EV) {
        t += 200u + (uint32_t)rndi(&r, 9000);
        ev_t *e = &s->ev[s->n];
        e->t = t;
        float p = rnd(&r);
        int k;
        if (p < 0.20f) k = E_JOY;
        else if (p < 0.27f) k = E_BTN;
        else if (p < 0.30f) k = E_TAKE;
        else if (p < 0.32f) k = E_AUTOBTN;
        else if (p < 0.38f) k = E_CONFIRM;
        else if (p < 0.42f) k = E_PC_MODE;
        else if (p < 0.45f) k = E_PC_AUTO_START;
        else if (p < 0.46f) k = E_PC_AUTO_RESUME;
        else if (p < 0.47f) k = E_PC_PAUSE;
        else if (p < 0.49f) k = E_PC_PROG;
        else if (p < 0.53f) k = E_PC_RAISE;
        else if (p < 0.54f) k = E_PC_LOWER;
        else if (p < 0.56f) k = E_REMOTE_REQ;
        else if (p < 0.58f) k = E_REMOTE_ON;
        else if (p < 0.595f) k = E_REMOTE_OFF;
        else if (p < 0.605f) k = E_REMOTE_STALE;
        else if (p < 0.61f) k = E_REMOTE_RELEASE;
        else if (p < 0.63f) k = E_COMPASS;
        else if (p < 0.64f) k = E_GYRO;
        else if (p < 0.66f) k = E_GPS;
        else if (p < 0.665f) k = E_GPS_JUMP;
        else if (p < 0.675f) k = E_NSW;
        else if (p < 0.68f) k = E_CURRENT;
        else if (p < 0.685f) k = E_LIMITSW;
        else if (p < 0.695f) k = E_JOY_WIRE;
        else if (p < 0.70f) k = E_ACCEL;
        else if (p < 0.72f) k = E_SEA;
        else if (p < 0.74f) k = E_GW;
        else if (p < 0.82f) k = E_NAVREQ;
        else if (p < 0.87f) k = E_FUZZ_USB;
        else if (p < 0.92f) k = E_FUZZ_GW;
        else if (p < 0.935f) k = E_FREEZE;
        else if (p < 0.95f) k = E_RESET;
        else if (p < 0.975f) k = E_INJ_THR;
        else k = E_INJ_GEAR;
        e->kind = (uint8_t)k;
        e->a = rndi(&r, 1000);
        e->b = rndi(&r, 1000);
        e->c = rndi(&r, 1000);
        e->f = rnd(&r) * 2.0f - 1.0f;
        e->g = rnd(&r) * 2.0f - 1.0f;
        s->on[s->n] = true;
        s->n++;
        /* what a crew does: a program before AUTO, the remote data before its request, and the
           joystick CONFIRM 1-3 s after a request (mostly) */
        if ((k == E_PC_AUTO_START || k == E_PC_RAISE || k == E_REMOTE_REQ) && s->n + 3 < MAX_EV) {
            if (k == E_PC_AUTO_START && rnd(&r) < 0.6f) {
                ev_t pe = *e;
                pe.kind = E_PC_PROG;
                pe.a = 1 + rndi(&r, 5);
                *e = pe;                                   /* the program first ... */
                e->t = t;
                ev_t *st = &s->ev[s->n++];                 /* ... then the start */
                memset(st, 0, sizeof *st);
                st->t = t + 100u;
                st->kind = E_PC_AUTO_START;
                s->on[s->n - 1] = true;
            }
            if (k == E_REMOTE_REQ) {
                ev_t req = *e;
                if (rnd(&r) < 0.6f) {                      /* stick to the centre + STANDBY first */
                    ev_t *jc = e;
                    memset(jc, 0, sizeof *jc);
                    jc->t = t;
                    jc->kind = E_JOY;
                    jc->a = 0;                             /* a < 400: centre */
                    ev_t *sb = &s->ev[s->n++];
                    memset(sb, 0, sizeof *sb);
                    sb->t = t + 100u;
                    sb->kind = E_BTN;
                    sb->a = 0;                             /* a < 350: STANDBY */
                    s->on[s->n - 1] = true;
                    ev_t *rq = &s->ev[s->n++];
                    *rq = req;
                    rq->t = t + 1600u;
                    s->on[s->n - 1] = true;
                    t += 1600u;
                }
                ev_t *on = &s->ev[s->n++];
                *on = req;
                on->t = t + 50u;
                on->kind = E_REMOTE_ON;
                s->on[s->n - 1] = true;
            }
            if (rnd(&r) < 0.75f) {
                ev_t *cf = &s->ev[s->n++];
                memset(cf, 0, sizeof *cf);
                cf->t = t + 1000u + (uint32_t)rndi(&r, 2000);
                cf->kind = E_CONFIRM;
                s->on[s->n - 1] = true;
                t = cf->t;
                if (k == E_REMOTE_REQ && rnd(&r) < 0.5f && s->n + 1 < MAX_EV) {   /* a dropout / old data */
                    ev_t *dr = &s->ev[s->n++];
                    memset(dr, 0, sizeof *dr);
                    dr->t = t + 3000u + (uint32_t)rndi(&r, 17000);
                    dr->kind = rnd(&r) < 0.6f ? E_REMOTE_OFF : E_REMOTE_STALE;
                    s->on[s->n - 1] = true;
                    t = dr->t;
                }
            }
        }
    }
}

/* -------------------------------------------------------------- the sim -- */
#define AT_OR_AFTER(a, b)  ((int32_t)((uint32_t)(a) - (uint32_t)(b)) >= 0)
typedef struct {
    world_t     w;
    sb_t        sb;
    sb_config_t cfg;
    link_t      usb, gw;
    proto_rx_t  usb_rx, gw_rx;
    mon_t       mon;
    uint32_t    t, next_ctrl, last_kick, boot_until, wdt_ms, frozen_until, resets, boots;
    bool        frozen;
    bool        gw_on;
    uint32_t    gw_next;
    /* the storm's "hands" */
    float       jx, jy;
    uint32_t    joy_hold_until;
    bool        joy_fitted;
    bool        joy_broken;         /* a broken wire: the reading goes out of range */
    bool        confirm_held;
    uint32_t    confirm_release;
    bool        take_held, auto_held;
    uint32_t    btn_release;
    bool        rem_stream, rem_stale;
    uint16_t    rem_seq;
    uint8_t     rem_station;
    float       rx_x, rx_lever;
    uint32_t    inj_kind, inj_until;
    uint32_t    nav_seq;
    uint32_t    sched_seed;
    bool        pc_on;              /* a ground station on the USB link: heartbeats like the real one */
    uint32_t    pc_next;
} sim_t;

static sim_t S;
static bool s_log;              /* soak: core 0 log producer + core 2 logger (SD card in test_results/soak_sd) */
static logger_t s_lg;
static uint32_t s_ctrl_max_us, s_ctrl_n;
static uint32_t s_hist[64];                         /* control step time histogram, 100 us bins */
static double   s_ctrl_sum_us;

static uint16_t joy_raw(float v)
{
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (uint16_t)lrintf(2048.0f + v * 1800.0f);
}

static void joy_apply(void)
{
    sim_hal_set_joystick(S.joy_fitted, S.joy_broken ? 20u : joy_raw(S.jx), joy_raw(S.jy), S.take_held, S.auto_held);
    sim_hal_joystick_confirm(S.confirm_held);
}

static void boot(void)
{
    hal_init();
    sb_init(&S.sb, &S.cfg);
    link_init(&S.usb, &S.sb);
    link_init_port(&S.gw, &S.sb, LINK_PORT_GW);
    monitor_init(&S.mon, S.t);
    S.next_ctrl = S.t;
    S.last_kick = S.t;
    S.boots++;
    if (s_log) {
        g_logstat.boot = S.boots;
        logger_init(&s_lg, S.t);
        logcore0_init(&S.sb, S.boots);
    }
}

static void sim_init(const sched_t *sc)
{
    memset(&S, 0, sizeof S);
    S.sched_seed = sc->seed;
    world_init(&S.w, sc->seed | 1u, 0.0f, 0.0f);
    S.w.sea_state = sc->sea0;
    sim_hal_set_time_ms(0u);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, 0.05f, 0.02f);
    sim_hal_set_accel(true);
    sim_hal_set_attitude(true);
    sim_hal_set_neutral_switch(false, true);
    sim_hal_set_motor_current(false, 0.0f);
    sim_hal_set_limit_switches(false, false, false);
    sim_hal_reset_engine();
    sim_hal_bridge_mode(false);
    sim_hal_unforce();
    remove("nv1.bin");
    remove("nv2.bin");
    remove("nv4.bin");
    S.joy_fitted = true;
    S.jx = sc->boot_deflected ? 0.8f : 0.0f;
    S.jy = 0.0f;
    joy_apply();
    memset((void *)&g_mon, 0, sizeof g_mon);
    world_publish(&S.w, 0u);
    sb_default_config(&S.cfg);
    timing_init_all();
    logq_init_all();
    sim_hal_link_reset();
    sim_hal_gw_reset(921600u);
    proto_rx_init(&S.usb_rx);
    proto_rx_init(&S.gw_rx);
    S.wdt_ms = 100u;
    S.gw_on = true;
    if (s_log) {
        (void)system("mkdir soak_sd 2>nul & del /q soak_sd\\*.TLG 2>nul");
        sim_hal_sd_setup("soak_sd", true, 64000000u, false);
        memset((void *)&g_logstat, 0, sizeof g_logstat);
    }
    boot();
}

static void push_frame(bool gw, uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint8_t fr[PROTO_MAX_FRAME];
    size_t n = proto_encode(fr, type, (uint8_t)S.nav_seq, pl, len);
    if (gw) sim_hal_gw_rx_push(fr, (uint32_t)n);
    else sim_hal_link_rx_push(fr, (uint32_t)n);
}

static void navreq(uint8_t kind, uint8_t reason, uint8_t alarm, float hdg, float cap, uint16_t lease)
{
    uint8_t pl[11];
    uint16_t h = (uint16_t)lrintf(hdg * 100.0f), c = cap < 0.0f ? 0xFFFFu : (uint16_t)lrintf(cap * 10.0f);
    pl[0] = (uint8_t)(++S.nav_seq);
    pl[1] = kind; pl[2] = reason; pl[3] = alarm;
    pl[4] = (uint8_t)h; pl[5] = (uint8_t)(h >> 8);
    pl[6] = (uint8_t)c; pl[7] = (uint8_t)(c >> 8);
    pl[8] = (uint8_t)lease; pl[9] = (uint8_t)(lease >> 8);
    push_frame(true, PROTO_T_GW_NAVREQ, pl, 10u);
}

static void prog_random(int n, uint32_t *r)
{
    static program_t p;
    program_clear(&p);
    p.n = (uint16_t)n;
    p.final_action = (uint8_t)rndi(r, 2);
    double north = S.w.north_m, east = S.w.east_m;
    for (int i = 0; i < n; i++) {
        double br = rnd(r) * 6.283, d = 300.0 + rnd(r) * 1500.0;
        north += d * cos(br);
        east += d * sin(br);
        world_t tmp = S.w;
        double lat, lon;
        tmp.north_m = north;
        tmp.east_m = east;
        world_latlon(&tmp, &lat, &lon);
        p.wp[i].lat_e7 = (int32_t)lround(lat * 1e7);
        p.wp[i].lon_e7 = (int32_t)lround(lon * 1e7);
        p.wp[i].speed_kn = 4.0f + rnd(r) * 16.0f;
        p.wp[i].radius_m = 25.0f;
        p.wp[i].action = 0;
    }
    program_seal(&p);
    prog_error_t err;
    uint8_t bad;
    (void)sb_program_set(&S.sb, &p, &err, &bad);
}

/* --------------------------------------------------------- invariants -- */
#define NINV 22
static const char *INV_NAMES[NINV] = { "",
    "I1  throttle never above the power limit (except coming down at the closing rate) / remote cap",
    "I2  after power-up / reset the power limit is 30 % fwd / 5 % rev",
    "I3  gear never changes with the throttle open; idle -> neutral -> gear -> throttle",
    "I4  the STANDBY button always wins: idle + neutral within the time",
    "I5  the onboard joystick wins over autopilot / PC / remote / vision within the time",
    "I6  the PC / remote never take control from the onboard joystick",
    "I7  power raise, AUTO start, remote driving only with an onboard CONFIRM within 10 s",
    "I8  CONFIRM alone never changes the mode",
    "I9  fault / STANDBY: idle + neutral commanded before (or with) steering off",
    "I10 no mode change outside the allowed-transition table",
    "I11 outputs disarmed at power-up until neutral + joystick centred",
    "I12 remote data old / missing -> idle + neutral in time, never resumes by itself",
    "I13 slamming / caps never increase the throttle (never above what the mode asks)",
    "I14 (gateway: storm_gateway.py)", "I15 (gateway)", "I16 (gateway)", "I17 (gateway)",
    "I18 logging never delays the control step (20 Hz kept)",
    "I19 the CPU1 monitor forces the safe state on every injected violation in time",
    "I20 navigation / vision caps never steer the boat in other modes, never raise the throttle",
    "I21 (gateway)" };

typedef struct {
    uint32_t checks[NINV], fails[NINV];
    float    worst[NINV];           /* reaction times (s) */
    uint32_t first_fail_seed[NINV];
    char     first_fail_text[NINV][400];
    uint32_t auto_by_button, auto_by_confirm;
    uint32_t rem_req[16], remote_entries;
    uint32_t engaged_ignored;       /* HDG LOCK / AUTO / REMOTE entered while the joystick is ignored (not centred) */
    uint32_t engaged_ignored_seed;
} stats_t;
static stats_t ST;

/* the allowed transitions per CLAUDE.md (written here independently of app/modes.c) */
static bool spec_allowed(sb_mode_t f, sb_mode_t t)
{
    if (f == t) return true;
    switch (f) {
    case MODE_STANDBY:  return true;                                       /* every mode starts in STANDBY */
    case MODE_MANUAL:   return t == MODE_STANDBY || t == MODE_HDG_LOCK || t == MODE_AUTO;   /* never REMOTE / TRAINING */
    case MODE_HDG_LOCK: return t == MODE_STANDBY || t == MODE_MANUAL || t == MODE_AUTO || t == MODE_REMOTE;
    case MODE_AUTO:     return t == MODE_STANDBY || t == MODE_MANUAL || t == MODE_HDG_LOCK || t == MODE_REMOTE;
    case MODE_TRAINING: return t == MODE_STANDBY || t == MODE_MANUAL;
    case MODE_REMOTE:   return t == MODE_STANDBY || t == MODE_MANUAL;
    default:            return false;
    }
}

typedef struct {
    bool     failed;
    int      inv;
    uint32_t t;
    char     text[200];
    /* tracking */
    sb_mode_t mode_prev;
    float    thr_prev, env;
    int      gear_prev;
    uint32_t gear_change_t;
    uint32_t standby_press_t;       /* 0 = none pending */
    float    standby_thr;
    uint32_t defl_t;                /* joystick deflection from centre: when */
    bool     defl_pending;
    bool     centred_prev;
    uint32_t confirm_t, autobtn_t, raise_req_t;
    uint32_t confirms_prev;
    uint32_t confirm_free_t;        /* a CONFIRM with nothing pending */
    sb_mode_t confirm_free_mode;
    uint32_t rem_last_fresh;        /* last fresh remote packet (0 none) */
    bool     rem_check;
    uint32_t inj_t;
    bool     inj_seen_forced;
    uint32_t freeze_t;
    float    plim_f_prev, plim_r_prev;
    uint32_t boot_seen;
    bool     armed_ok_seen;         /* centred + neutral seen since the boot */
    uint32_t steps, steps_t0;
    float    target_prev;
    bool     rem_lost_session;      /* this REMOTE session went LOST: it must never drive again by itself */
    uint32_t last_action_t;         /* the last event that may legitimately change the mode */
} chk_t;

static chk_t C;
static bool s_verbose;

static void fail(int inv, const char *fmt, double a, double b)
{
    ST.checks[inv]++;
    if (C.failed) return;
    C.failed = true;
    C.inv = inv;
    C.t = S.t;
    snprintf(C.text, sizeof C.text, fmt, a, b);
}

static void ok(int inv) { ST.checks[inv]++; }

static int gear_pos(uint16_t us)
{
    if (us == 0u) return -1;                                      /* no pulses (reset) */
    if (abs((int)us - (int)S.cfg.gear.pwm_n_us) < 60) return 0;
    if (abs((int)us - (int)S.cfg.gear.pwm_f_us) < 60) return 1;
    if (abs((int)us - (int)S.cfg.gear.pwm_r_us) < 60) return 2;
    return 3;                                                     /* in between (moving) */
}

static void worst(int inv, float v)
{
    if (v > ST.worst[inv]) ST.worst[inv] = v;
}

/* after every control step */
static void check_step(void)
{
    sb_t *s = &S.sb;
    sb_mode_t m = sb_mode(s);
    float thr = sim_hal_throttle();
    int gp = gear_pos(sim_hal_gear_pwm());
    bool forced = hal_outputs_forced();
    bool inj = S.inj_kind && !AT_OR_AFTER(S.t, S.inj_until + 51u);
    float dt = AP_PERIOD_MS * 0.001f;
    float down = s->cfg.throttle.rate_down_pct_s;
    C.steps++;

    /* I2: right after a boot */
    if (C.boot_seen != S.boots) {
        C.boot_seen = S.boots;
        C.armed_ok_seen = false;
        C.env = 0.0f;
        if (s->plim.fwd_pct == 30.0f && s->plim.rev_pct == 5.0f) ok(2);
        else fail(2, "power limit after the boot %.0f / %.0f", s->plim.fwd_pct, s->plim.rev_pct);
        C.plim_f_prev = s->plim.fwd_pct;
        C.plim_r_prev = s->plim.rev_pct;
        C.mode_prev = m;
        C.thr_prev = thr;
        C.gear_prev = gp;
        C.confirms_prev = s->confirm.accepted;
        return;
    }

    /* I10: transitions */
    if (m != C.mode_prev) {
        if (spec_allowed(C.mode_prev, m)) ok(10);
        else fail(10, "mode %.0f -> %.0f not in the table", (double)C.mode_prev, (double)m);
    }

    /* I11: not armed -> nothing moves */
    float tol = s->joy.cfg.center_tol * 0.9f;              /* "centred" as the controller sees it */
    bool near_centre = fabsf(S.jx) < tol * 1.4f && fabsf(S.jy) < tol * 1.4f;   /* (calibration differs a little) */
    if (near_centre && (gp == 0 || gp == -1)) C.armed_ok_seen = true;
    if (!s->armed && !forced && !inj) {
        if (thr <= 0.6f && (gp == 0 || gp == -1) && fabsf(sim_hal_motor_command()) < 0.01f) ok(11);
        else fail(11, "not armed but throttle %.1f %% / gear %.0f", thr, (double)gp);
    }
    if (s->armed && !C.armed_ok_seen) fail(11, "armed although the joystick was never centred (%.1f, %.1f)",
                                             S.jx, S.jy);

    /* I1: under the limit, or coming down at the closing rate */
    if (!forced && !inj && gp >= 0) {
        float lim = (gp == 2) ? s->plim.rev_pct : s->plim.fwd_pct;
        if (m == MODE_REMOTE) lim = fminf(lim, gp == 2 ? s->cfg.remote_cap_rev_pct : s->cfg.remote_cap_fwd_pct);
        C.env = fmaxf(lim, C.env - down * dt);
        if (thr <= C.env + 1.0f) ok(1);
        else fail(1, "throttle %.1f %% above the limit envelope %.1f %%", thr, C.env);
        C.env = fmaxf(C.env, fminf(thr, C.env));
    }

    /* I3: gear change only at idle; throttle only after the engage wait */
    if (!forced && !inj && gp >= 0 && C.gear_prev >= 0 && gp != C.gear_prev && gp != 3 && C.gear_prev != 3) {
        if (thr <= 0.6f && C.thr_prev <= 0.6f) ok(3);
        else fail(3, "gear moved with the throttle at %.1f %% (before %.1f %%)", thr, C.thr_prev);
        C.gear_change_t = S.t;
    }
    if (!forced && !inj && C.gear_change_t && S.t - C.gear_change_t < (uint32_t)(s->cfg.gear.engage_wait_ms * 0.7f)) {
        if (thr <= 0.6f) ok(3);
        else fail(3, "throttle %.1f %% only %.0f ms after the gear engaged", thr, (double)(S.t - C.gear_change_t));
    }

    /* I4: the STANDBY button */
    if (C.standby_press_t && C.last_action_t > C.standby_press_t) {
        if (m == MODE_STANDBY || S.t - C.standby_press_t < 150u) ok(4);
        C.standby_press_t = 0u;                          /* a new action after STANDBY: it may change the mode */
    }
    if (C.standby_press_t) {
        int32_t dms = (int32_t)(S.t - C.standby_press_t);
        float el = (dms > 0 ? dms : 0) * 0.001f;
        float bound = C.standby_thr / down + s->cfg.gear.idle_wait_ms * 0.001f + 0.8f;
        if (el > 0.15f && m != MODE_STANDBY && !forced) fail(4, "%.2f s after STANDBY still mode %.0f", el, (double)m);
        if (thr <= 0.6f && (gp == 0 || gp == -1)) {
            ok(4);
            worst(4, el);
            C.standby_press_t = 0u;
        } else if (el > bound) {
            fail(4, "%.2f s after STANDBY: not yet idle + neutral (limit %.2f s)", el, bound);
            C.standby_press_t = 0u;
        }
    }

    /* I5 / I6: the onboard joystick */
    bool joy_ok = S.joy_fitted && !S.joy_broken && s->armed && !forced && !s->joy.fault && !s->mon_tripped;
    if (C.defl_pending && joy_ok) {
        float el = (S.t - C.defl_t) * 0.001f;
        if (m == MODE_MANUAL) {
            ok(5);
            worst(5, el);
            C.defl_pending = false;
        } else if (C.standby_press_t || (m == MODE_STANDBY && el < 0.12f)) {
            /* wait */
        } else if (el > 0.40f) {
            if (m == MODE_STANDBY && !s->joy.armed) {
                C.defl_pending = false;                  /* held through a STANDBY press: ignored until centred */
            } else {
                fail(5, "joystick deflected %.2f s ago, mode still %.0f", el, (double)m);
                C.defl_pending = false;
            }
        }
    } else if (C.defl_pending && !joy_ok) {
        C.defl_pending = false;
    }
    bool held = fabsf(S.jx) > tol * 1.5f || fabsf(S.jy) > tol * 1.5f;   /* clearly not centred */
    if (held && C.mode_prev == MODE_MANUAL && m != C.mode_prev) {
        if (m == MODE_STANDBY) ok(6);                    /* STANDBY / a fault may stop it */
        else fail(6, "stick held in MANUAL, mode went to %.0f (from %.0f)", (double)m, (double)C.mode_prev);
    }

    if (m != C.mode_prev && (m == MODE_HDG_LOCK || m == MODE_AUTO || m == MODE_REMOTE) && !s->joy.armed &&
        S.joy_fitted && !S.joy_broken && held) {
        if (!ST.engaged_ignored) ST.engaged_ignored_seed = S.sched_seed;
        ST.engaged_ignored++;
    }

    /* I7: raise / AUTO / REMOTE need an onboard CONFIRM within 10 s */
    bool conf_now = s->confirm.accepted != C.confirms_prev;
    if (conf_now) C.confirm_t = S.t;
    if (s->plim.fwd_pct > C.plim_f_prev + 0.1f || s->plim.rev_pct > C.plim_r_prev + 0.1f) {
        if (conf_now && C.raise_req_t && S.t - C.raise_req_t <= 10000u + 200u) ok(7);
        else fail(7, "power limit raised to %.0f / %.0f without a fresh request + CONFIRM",
                  s->plim.fwd_pct, s->plim.rev_pct);
    }
    if (m == MODE_AUTO && C.mode_prev != MODE_AUTO) {
        if (C.confirm_t && S.t - C.confirm_t <= 200u) { ok(7); ST.auto_by_confirm++; }
        else if (C.autobtn_t && S.t - C.autobtn_t <= 400u) { ok(7); ST.auto_by_button++; }
        else fail(7, "AUTO entered from mode %.0f without CONFIRM (last %.0f ms ago)", (double)C.mode_prev,
                  C.confirm_t ? (double)(S.t - C.confirm_t) : -1.0);
    }
    if (m == MODE_REMOTE && C.mode_prev != MODE_REMOTE) {
        ST.remote_entries++;
        if (C.confirm_t && S.t - C.confirm_t <= 200u) ok(7);
        else fail(7, "REMOTE entered without CONFIRM (mode before %.0f)%.0f", (double)C.mode_prev, 0.0);
    }

    /* I8: a CONFIRM with nothing pending */
    if (C.confirm_free_t && C.last_action_t > C.confirm_free_t) C.confirm_free_t = 0u;
    if (C.confirm_free_t && S.t - C.confirm_free_t >= 200u) {
        if (m == C.confirm_free_mode || m == MODE_STANDBY) ok(8);
        else fail(8, "CONFIRM alone changed the mode %.0f -> %.0f", (double)C.confirm_free_mode, (double)m);
        C.confirm_free_t = 0u;
    }

    /* I9: STANDBY by a fault / the button: idle + neutral commanded at once */
    if (m == MODE_STANDBY && C.mode_prev != MODE_STANDBY) {
        if (s->throttle_mode <= 0.01f && s->gear_demand == GEAR_N) ok(9);
        else fail(9, "STANDBY with throttle asked %.1f %% / gear %.0f", s->throttle_mode, (double)s->gear_demand);
    }
    if (s->stopping) {
        if (s->throttle_mode <= 0.01f) ok(9);
        else fail(9, "safe stop running but throttle asked %.1f %%%.0f", s->throttle_mode, 0.0);
    }

    /* I12: remote data stops / gets old */
    if (m != MODE_REMOTE) { C.rem_check = false; C.rem_lost_session = false; }
    if (m == MODE_REMOTE && C.mode_prev != MODE_REMOTE) {
        C.rem_last_fresh = S.t;
        if (!S.rem_stream || S.rem_stale) C.rem_check = true;     /* entered without fresh data */
    }
    if (m == MODE_REMOTE && s->remote.state == SB_REMOTE_LOST) C.rem_lost_session = true;
    if (C.rem_check && m == MODE_REMOTE) {
        float el = (S.t - C.rem_last_fresh) * 0.001f;
        if (s->remote.state == SB_REMOTE_LOST) {
            if (s->throttle_mode <= 0.01f && s->gear_demand == GEAR_N) {
                ok(12);
                worst(12, el);
                C.rem_check = false;
            } else if (el > 0.6f) {
                fail(12, "remote LOST %.2f s ago but throttle asked %.1f %%", el, s->throttle_mode);
                C.rem_check = false;
            }
        } else if (el > 0.5f) {
            fail(12, "remote data %.2f s old, state %.0f (not LOST)", el, (double)s->remote.state);
            C.rem_check = false;
        }
    }
    if (m == MODE_REMOTE && C.rem_lost_session) {
        if (s->remote.state == SB_REMOTE_LOST) ok(12);
        else fail(12, "remote LOST, then driving again by itself (state %.0f, mode %.0f)", (double)s->remote.state,
                  (double)m);
    }

    /* I13 / I20: never more throttle than the mode asks (slamming, caps, limits only reduce) */
    if (!forced && !inj) {
        float ask = fabsf(s->throttle_mode), out = fabsf(s->throttle_demand);
        if (out <= ask + 0.5f || (ask == 0.0f && out <= 0.6f)) ok(13);
        else fail(13, "throttle demand %.1f %% above what the mode asked (%.1f %%)", out, ask);
        bool changed = fabsf(s->ap.target_heading - C.target_prev) > 0.01f;
        if (changed && m != MODE_AUTO && m != MODE_HDG_LOCK && s->nav.kind == SB_NAV_HEADING &&
            fabsf(s->ap.target_heading - s->nav.heading) < 0.01f) {
            fail(20, "navigation heading %.1f applied in mode %.0f", s->nav.heading, (double)m);
        } else ok(20);
    }

    /* I19: injected violations / frozen core 0 -> the monitor's safe state */
    if (C.inj_t && forced && !C.inj_seen_forced) {
        float el = (S.t - C.inj_t) * 0.001f;
        C.inj_seen_forced = true;
        worst(19, el);
        if (el <= 0.15f) ok(19);
        else fail(19, "monitor safe state %.2f s after the injected violation%.0f", el, 0.0);
    }
    if (C.inj_t && !C.inj_seen_forced && S.t - C.inj_t > 300u) {
        fail(19, "no safe state %.2f s after the injected violation (kind %.0f)", (S.t - C.inj_t) * 0.001,
             (double)S.inj_kind);
        C.inj_t = 0u;
    }

    C.mode_prev = m;
    C.thr_prev = thr;
    C.gear_prev = gp;
    C.confirms_prev = s->confirm.accepted;
    C.plim_f_prev = s->plim.fwd_pct;
    C.plim_r_prev = s->plim.rev_pct;
    C.target_prev = s->ap.target_heading;
}

/* during a frozen core 0: the monitor must trip by itself */
static void check_frozen(void)
{
    if (C.freeze_t && hal_outputs_forced()) {
        float el = (S.t - C.freeze_t) * 0.001f;
        worst(19, el);
        if (el <= (MON_HB_MS + 60u) * 0.001f) ok(19);
        else fail(19, "core 0 frozen: safe state after %.2f s (limit %.2f s)", el, (MON_HB_MS + 60u) * 0.001);
        C.freeze_t = 0u;
    }
}

/* ------------------------------------------------------------- events -- */
static uint32_t s_er;       /* rng for event details (from the event's own numbers) */

static void apply(const ev_t *e)
{
    sb_t *s = &S.sb;
    switch (e->kind) {
    case E_JOY: case E_BTN: case E_TAKE: case E_AUTOBTN: case E_CONFIRM: case E_PC_MODE: case E_PC_AUTO_START:
    case E_PC_AUTO_RESUME: case E_REMOTE_REQ: case E_REMOTE_RELEASE: case E_RESET: case E_FREEZE: case E_COMPASS:
    case E_JOY_WIRE: case E_GPS: case E_NSW: case E_INJ_THR: case E_INJ_GEAR: case E_FUZZ_USB: case E_FUZZ_GW:
        C.last_action_t = S.t + 1u;
        break;
    default:
        break;
    }
    uint32_t r = (uint32_t)(e->a * 7919 + e->b * 104729 + e->c + 1);
    switch (e->kind) {
    case E_JOY: {
        float tl = s->joy.cfg.center_tol * 0.7f;
        bool was_c = fabsf(S.jx) < tl && fabsf(S.jy) < tl;
        if (e->a < 400) { S.jx = 0.0f; S.jy = 0.0f; }                       /* back to centre */
        else { S.jx = e->f; S.jy = (e->a < 700) ? 0.0f : e->g; }
        joy_apply();
        if (was_c && fabsf(S.jx) >= 0.6f) { C.defl_pending = true; C.defl_t = S.t; }
        break;
    }
    case E_BTN: {
        static const hal_button_t B[] = { HAL_BTN_STANDBY, HAL_BTN_HDG_LOCK, HAL_BTN_PLUS_1, HAL_BTN_MINUS_1,
                                          HAL_BTN_PLUS_10, HAL_BTN_MINUS_10 };
        hal_button_t b = B[e->a < 350 ? 0 : 1 + e->a % 5];
        sim_hal_press(b);
        if (b == HAL_BTN_STANDBY) { C.standby_press_t = S.t + 1u; C.standby_thr = sim_hal_throttle(); }
        break;
    }
    case E_TAKE: S.take_held = true; S.btn_release = S.t + 150u; joy_apply(); break;
    case E_AUTOBTN: S.auto_held = true; S.btn_release = S.t + 150u; C.autobtn_t = S.t; joy_apply(); break;
    case E_CONFIRM:
        if (s->confirm.kind == CONF_NONE && fabsf(S.jx) < 0.05f && fabsf(S.jy) < 0.05f) {
            C.confirm_free_t = S.t + 1u;
            C.confirm_free_mode = sb_mode(s);
        }
        S.confirm_held = true; S.confirm_release = S.t + 120u; joy_apply();
        break;
    case E_PC_MODE: (void)sb_request_mode(s, (sb_mode_t)(e->a % MODE_COUNT), WHY_USER); break;
    case E_PC_AUTO_START: (void)sb_auto_start(s); break;
    case E_PC_AUTO_RESUME: (void)sb_auto_resume(s); break;
    case E_PC_PAUSE: (void)sb_auto_pause(s); break;
    case E_PC_PROG: prog_random(1 + e->a % 6, &r); break;
    case E_PC_RAISE: {
        uint16_t code = 0;
        uint8_t dir = (uint8_t)(e->a % 2);
        if (sb_power_code(s, dir, &code) == SB_OK) {
            if (e->b < 200) code = (uint16_t)(code + 1u);                   /* typed wrong */
            if (sb_power_raise(s, dir, code) == SB_OK) C.raise_req_t = S.t;
        }
        break;
    }
    case E_PC_LOWER: sb_power_lower(s); break;
    case E_REMOTE_REQ:
        S.rem_station = (uint8_t)(2 + e->a % 3);
        s->helm = S.rem_station;                          /* the shore station took HELM first (gateway) */
        s->helm_local = false;
        ST.rem_req[(int)sb_remote_request(s, S.rem_station) & 15]++;
        break;
    case E_REMOTE_ON: S.rem_stream = true; S.rem_stale = false; S.rx_x = e->f; S.rx_lever = 0.5f + 0.5f * e->g; break;
    case E_REMOTE_OFF:
        if (S.rem_stream && sb_mode(s) == MODE_REMOTE && s->remote.state == 1u) C.rem_check = true;
        S.rem_stream = false;
        break;
    case E_REMOTE_STALE:
        if (S.rem_stream && sb_mode(s) == MODE_REMOTE && s->remote.state == 1u) C.rem_check = true;
        S.rem_stale = true;
        break;
    case E_REMOTE_RELEASE: (void)sb_remote_release(s, S.rem_station); break;
    case E_COMPASS: sim_hal_fail_heading(e->a < 500); break;
    case E_GYRO: sim_hal_set_gyro(e->a >= 300, 0.05f, 0.02f); break;
    case E_GPS: S.w.gps_fail = e->a < 500; break;
    case E_GPS_JUMP: S.w.gps_off_n += (e->f * 3000.0); S.w.gps_off_e += (e->g * 3000.0); break;
    case E_NSW: sim_hal_set_neutral_switch(e->a < 600, e->b < 900); break;
    case E_CURRENT: sim_hal_set_motor_current(e->a < 500, (float)e->b * 0.02f); break;
    case E_LIMITSW: sim_hal_set_limit_switches(e->a < 500, e->b < 100, e->c < 100); break;
    case E_JOY_WIRE: S.joy_broken = e->a < 300; joy_apply(); break;
    case E_ACCEL: sim_hal_set_accel(e->a >= 300); break;
    case E_SEA: S.w.sea_state = (float)(e->a % 6); S.w.slam_extra_g = e->b < 200 ? 2.0f : 0.0f; break;
    case E_GW: S.gw_on = e->a >= 300; break;
    case E_NAVREQ: {
        uint8_t kind = (uint8_t)(e->a % 4), reason = (uint8_t)(1u << (e->b % 4));
        float cap = (e->c < 300) ? -1.0f : (float)(e->c % 160) * 0.1f;
        if (reason == 8u) kind = (uint8_t)(e->a % 2);                      /* vision: none / cap only */
        uint16_t lease = (uint16_t)(e->b < 50 ? 60000u : 500u + (uint32_t)e->c * 9u);   /* some invalid */
        navreq(kind, reason, (uint8_t)(1u << (e->c % 4)), fabsf(e->f) * 359.0f, cap, lease);
        break;
    }
    case E_FUZZ_USB:
    case E_FUZZ_GW: {
        bool gw = e->kind == E_FUZZ_GW;
        uint8_t pl[PROTO_MAX_PAYLOAD];
        uint8_t len = (uint8_t)(e->b % 64);
        for (int i = 0; i < len; i++) pl[i] = (uint8_t)rng_next(&r);
        if (e->a < 300) {                                                  /* garbage bytes */
            uint8_t g[200];
            uint32_t n = 1u + (uint32_t)e->c % 200u;
            for (uint32_t i = 0; i < n; i++) g[i] = (uint8_t)rng_next(&r);
            if (gw) sim_hal_gw_rx_push(g, n); else sim_hal_link_rx_push(g, n);
        } else if (e->a < 600) {                                           /* a valid frame, random type */
            push_frame(gw, (uint8_t)(rng_next(&r) % 0x40u), pl, len);
            if (e->c < 300) push_frame(gw, (uint8_t)(rng_next(&r) % 0x40u), pl, len);   /* duplicated */
        } else {                                                           /* a corrupted frame */
            uint8_t fr[PROTO_MAX_FRAME];
            size_t n = proto_encode(fr, (uint8_t)(rng_next(&r) % 0x40u), 0u, pl, len);
            fr[rng_next(&r) % n] ^= (uint8_t)(1u + rng_next(&r) % 255u);
            if (gw) sim_hal_gw_rx_push(fr, (uint32_t)n); else sim_hal_link_rx_push(fr, (uint32_t)n);
        }
        break;
    }
    case E_FREEZE:
        if (!S.frozen && !S.boot_until) {
            S.frozen = true;
            S.wdt_ms = e->a < 500 ? 100u : 0u;
            S.frozen_until = S.t + 400u + (uint32_t)e->b * 3u;
            C.freeze_t = S.wdt_ms ? 0u : S.t;                 /* without the watchdog: the monitor must trip */
        }
        break;
    case E_RESET:
        sim_hal_chip_reset(HAL_RESET_POWER_ON);
        S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
        break;
    case E_INJ_THR:
    case E_INJ_GEAR:
        if (!hal_outputs_forced() && !S.boot_until && !S.frozen &&
            (e->kind == E_INJ_THR || (sim_hal_throttle() > 5.0f && gear_pos(sim_hal_gear_pwm()) == 1))) {
            S.inj_kind = e->kind == E_INJ_THR ? 1u : 2u;
            S.inj_until = S.t + 300u;
            C.inj_t = S.t;
            C.inj_seen_forced = false;
        }
        break;
    default: break;
    }
    (void)s_er;
}

/* --------------------------------------------------------- the run -- */
static void run_until(uint32_t until)
{
    while (!AT_OR_AFTER(S.t, until)) {
        sim_hal_set_time_ms(S.t);
        world_publish(&S.w, S.t);
        if (S.boot_until && AT_OR_AFTER(S.t, S.boot_until)) {
            S.boot_until = 0u;
            S.frozen = false;
            S.inj_kind = 0u;
            memset((void *)&g_mon, 0, sizeof g_mon);
            sim_hal_unforce();
            boot();
        }
        if (S.frozen && AT_OR_AFTER(S.t, S.frozen_until) && !S.wdt_ms) {      /* no watchdog: it stays frozen until reset */
            S.frozen = false;
            sim_hal_chip_reset(HAL_RESET_POWER_ON);
            S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
            C.freeze_t = 0u;
        }
        if (!S.boot_until && AT_OR_AFTER(S.t, S.next_ctrl)) {
            S.next_ctrl += AP_PERIOD_MS;
            if (!S.frozen) {
                /* the hands: release buttons, remote stream */
                if (S.confirm_held && AT_OR_AFTER(S.t, S.confirm_release)) { S.confirm_held = false; joy_apply(); }
                if ((S.take_held || S.auto_held) && AT_OR_AFTER(S.t, S.btn_release)) { S.take_held = S.auto_held = false; joy_apply(); }
                if (S.rem_stream && sb_mode(&S.sb) == MODE_REMOTE) {
                    uint16_t age = S.rem_stale ? 900u : 40u;
                    sb_remote_input(&S.sb, S.rem_station, ++S.rem_seq, age, 30u, S.rx_x, 0.0f, S.rx_lever, true);
                    if (!S.rem_stale) C.rem_last_fresh = S.t;
                } else if (sb_mode(&S.sb) == MODE_REMOTE && C.rem_last_fresh == 0u) {
                    C.rem_last_fresh = S.t;
                }
                uint32_t t0 = hal_micros();
                sb_step(&S.sb);
                if (s_log) logcore0_step(&S.sb);
                uint32_t us = hal_micros() - t0;
                if (us > s_ctrl_max_us) s_ctrl_max_us = us;
                s_ctrl_n++;
                s_ctrl_sum_us += us;
                s_hist[us / 100u < 63u ? us / 100u : 63u]++;
                link_step(&S.usb, &S.sb);
                link_step(&S.gw, &S.sb);
                if (S.inj_kind && !AT_OR_AFTER(S.t, S.inj_until + 1u)) {                   /* a "bug" after the normal step */
                    if (S.inj_kind == 1u) { hal_set_throttle_pwm(1900u); g_mon.throttle_pct = 90.0f; }
                    else { hal_set_gear_pwm(S.cfg.gear.pwm_r_us); g_mon.gear_pwm_us = S.cfg.gear.pwm_r_us; }
                } else if (S.inj_kind && AT_OR_AFTER(S.t, S.inj_until + 3001u)) {
                    sim_hal_chip_reset(HAL_RESET_POWER_ON);               /* the crew power-cycles after a trip */
                    S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
                    S.inj_kind = 0u;
                    C.inj_t = 0u;
                }
                S.last_kick = S.t;
                check_step();
            }
        }
        if (S.frozen && !S.boot_until) check_frozen();
        if (S.wdt_ms && !S.boot_until && S.t - S.last_kick > S.wdt_ms) {
            S.resets++;
            sim_hal_chip_reset(HAL_RESET_SYSTEM);
            S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
            S.frozen = false;
        }
        if (!S.boot_until) monitor_step(&S.mon, &g_mon, S.t);
        if (s_log && !S.boot_until && S.t % 10u == 0u) logger_step(&s_lg, S.t);
        if (S.pc_on && AT_OR_AFTER(S.t, S.pc_next)) {             /* the ground station's heartbeat */
            S.pc_next = S.t + 500u;
            push_frame(false, PROTO_T_HEARTBEAT, NULL, 0u);
        }
        /* the Pi: status once a second, read what the AURIX sends */
        if (S.gw_on && AT_OR_AFTER(S.t, S.gw_next)) {
            uint8_t pl[12] = { 1u, 0u, 1u, 1u };
            S.gw_next = S.t + 1000u;
            push_frame(true, PROTO_T_GW_STATUS, pl, sizeof pl);
        }
        uint8_t buf[512];
        uint32_t n;
        proto_frame_t f;
        while ((n = sim_hal_gw_tx_pop(buf, sizeof buf)) > 0u) {
            proto_rx_push(&S.gw_rx, buf, n);
            while (proto_rx_next(&S.gw_rx, &f)) { }
        }
        while ((n = sim_hal_link_tx_pop(buf, sizeof buf)) > 0u) {
            proto_rx_push(&S.usb_rx, buf, n);
            while (proto_rx_next(&S.usb_rx, &f)) { }
        }
        world_step(&S.w, PHYSICS_DT_MS * 0.001f);
        S.t += PHYSICS_DT_MS;
    }
}

/* one episode; returns true if an invariant failed (C.inv, C.text) */
static bool episode(const sched_t *sc, bool print)
{
    sim_init(sc);
    memset(&C, 0, sizeof C);
    C.boot_seen = 0u;
    for (int i = 0; i < sc->n; i++) {
        if (!sc->on[i]) continue;
        run_until(sc->ev[i].t);
        if (C.failed) break;
        if (print) printf("  [mode %d joy %.2f %.2f armed %d fault %d] ", (int)sb_mode(&S.sb), S.sb.joy.x, S.sb.joy.y,
                          (int)S.sb.joy.armed, (int)S.sb.joy.fault);
        if (print) printf("  %8.1f s  %-28s a=%d b=%d c=%d f=%.2f g=%.2f\n", sc->ev[i].t * 0.001,
                          EV_NAMES[sc->ev[i].kind], sc->ev[i].a, sc->ev[i].b, sc->ev[i].c, sc->ev[i].f, sc->ev[i].g);
        apply(&sc->ev[i]);
    }
    if (!C.failed) run_until(sc->dur_ms);
    return C.failed;
}

/* the shortest event list that still fails the same invariant */
static int shrink(sched_t *sc, int inv)
{
    int changed = 1, kept = 0;
    while (changed) {
        changed = 0;
        for (int i = sc->n - 1; i >= 0; i--) {
            if (!sc->on[i]) continue;
            sc->on[i] = false;
            stats_t save = ST;
            bool f = episode(sc, false) && C.inv == inv;
            ST = save;                                                   /* shrinking doesn't count */
            if (!f) sc->on[i] = true; else changed = 1;
        }
    }
    for (int i = 0; i < sc->n; i++) kept += sc->on[i];
    return kept;
}

static sched_t SC;

static void report(int episodes, double hours)
{
    printf("\n=== FAULT STORM: %d episodes, %.1f simulated hours ===\n", episodes, hours);
    for (int i = 1; i < NINV; i++) {
        if (i >= 14 && i <= 17) continue;
        if (i == 21) continue;
        if (i == 18) continue;
        printf("%-4s %-96s checks %9lu  fails %lu%s", ST.fails[i] ? "FAIL" : "PASS", INV_NAMES[i],
               (unsigned long)ST.checks[i], (unsigned long)ST.fails[i], "");
        if (i == 4 || i == 5 || i == 12 || i == 19) printf("  worst %.2f s", ST.worst[i]);
        printf("\n");
        if (ST.fails[i]) printf("     first failing seed %lu: %s\n", (unsigned long)ST.first_fail_seed[i],
                                ST.first_fail_text[i]);
    }
    for (int i = 1; i < NINV; i++)                      /* for run_storm.py (several processes merged) */
        printf("DATA %d %lu %lu %.3f %lu\n", i, (unsigned long)ST.checks[i], (unsigned long)ST.fails[i], ST.worst[i],
               (unsigned long)ST.first_fail_seed[i]);
    printf("DATA_RUN %d %.3f %lu %lu %lu %lu %lu\n", episodes, hours, (unsigned long)ST.auto_by_confirm,
           (unsigned long)ST.auto_by_button, (unsigned long)ST.remote_entries, (unsigned long)ST.engaged_ignored,
           (unsigned long)ST.engaged_ignored_seed);
    printf("NOTE: HDG LOCK / AUTO / REMOTE entered while the onboard joystick was held and ignored (not centred since "
           "STANDBY): %lu times (first seed %lu)\n", (unsigned long)ST.engaged_ignored,
           (unsigned long)ST.engaged_ignored_seed);
    printf("AUTO entries: %lu by PC + CONFIRM, %lu by the joystick AUTO button; REMOTE entries %lu "
           "(requests: ok %lu, refused", (unsigned long)ST.auto_by_confirm, (unsigned long)ST.auto_by_button,
           (unsigned long)ST.remote_entries, (unsigned long)ST.rem_req[0]);
    for (int i = 1; i < 16; i++) if (ST.rem_req[i]) printf(" code %d: %lu", i, (unsigned long)ST.rem_req[i]);
    printf(")\n");
}

/* 32-bit time wraparound: the clock starts 3 minutes before 2^32 ms */
static int wrap_test(void)
{
    sched_t *sc = &SC;
    gen_schedule(sc, 4242u, 10u * 60000u);
    sim_init(sc);
    memset(&C, 0, sizeof C);
    uint32_t start = 0xFFFFFFFFu - 180000u;
    S.t = start;
    S.next_ctrl = start;
    S.last_kick = start;
    S.gw_next = start;
    sim_hal_set_time_ms(start);
    boot();
    uint32_t overruns0 = g_timing.core0.overruns, steps0 = 0u;
    for (int i = 0; i < sc->n; i++) {
        uint32_t when = start + sc->ev[i].t;
        while ((int32_t)(S.t - when) < 0) {
            run_until(S.t + 10u);
            steps0++;
        }
        apply(&sc->ev[i]);
        if (C.failed) break;
    }
    printf("\n=== TIME WRAPAROUND: started at %lu ms (3 min before 2^32), ran 10 min across the wrap ===\n",
           (unsigned long)start);
    printf("%s invariants across the wrap (%s)\n", C.failed ? "FAIL" : "PASS", C.failed ? C.text : "all held");
    printf("     end time %lu ms (wrapped), control steps %lu, boots %lu, overruns %lu, mode %d\n",
           (unsigned long)S.t, (unsigned long)C.steps, (unsigned long)S.boots,
           (unsigned long)(g_timing.core0.overruns - overruns0), (int)sb_mode(&S.sb));
    (void)steps0;
    return C.failed ? 1 : 0;
}


/* ===================================================== a day at sea (Part 4.2, AURIX part) == */
typedef struct { double t; char what[90], did[150], exp[90]; bool pass; } row_t;
static row_t R[64];
static int nR;
static FILE *s_md;

static void row(const char *what, bool pass, const char *exp, const char *fmt, ...)
{
    va_list ap;
    row_t *r = &R[nR < 63 ? nR++ : 63];
    r->t = S.t * 0.001;
    snprintf(r->what, sizeof r->what, "%s", what);
    snprintf(r->exp, sizeof r->exp, "%s", exp);
    va_start(ap, fmt);
    vsnprintf(r->did, sizeof r->did, fmt, ap);
    va_end(ap);
    r->pass = pass && !C.failed;
    printf("| %7.0f s | %s | %s | %s | %s |\n", r->t, r->what, r->did, r->exp, r->pass ? "PASS" : "FAIL");
    if (s_md) fprintf(s_md, "| %.0f s | %s | %s | %s | %s |\n", r->t, r->what, r->did, r->exp, r->pass ? "PASS" : "FAIL");
}

static void run_s(double s) { run_until(S.t + (uint32_t)(s * 1000.0)); }
static void sync_mode(void) { C.mode_prev = sb_mode(&S.sb); }     /* a mode change by an API call (not a step) */
/* the USB laptop: take HELM, acknowledge the alarms (through the real link parser) */
static void ack_alarms(void)
{
    uint8_t take = 0u;
    S.pc_on = true;
    push_frame(false, PROTO_T_HELM, &take, 1u);
    run_until(S.t + 1200u);
    push_frame(false, PROTO_T_ALARM_ACK, NULL, 0u);
    run_until(S.t + 300u);
}
static void press_btn(hal_button_t b) { sim_hal_press(b); run_until(S.t + 120u); }
static bool do_confirm(void)
{
    uint32_t c0 = S.sb.confirm.accepted;
    S.confirm_held = true; S.confirm_release = S.t + 150u; joy_apply();
    run_until(S.t + 400u);
    return S.sb.confirm.accepted > c0;
}
static bool wait_for_c(bool (*cond)(void), double max_s)
{
    uint32_t end = S.t + (uint32_t)(max_s * 1000.0);
    while (!cond() && AT_OR_AFTER(end, S.t)) run_until(S.t + 100u);
    return cond();
}
static bool c_training_result(void) { return S.sb.train.stage == TS_RESULT; }
static bool c_dr_dead(void) { return S.sb.dr.state == DR_DEAD; }
static bool c_dr_gps(void) { return S.sb.dr.state == DR_GPS; }
static bool c_not_gps(void) { return S.sb.dr.state != DR_GPS; }
static bool c_lost(void) { return S.sb.remote.state == SB_REMOTE_LOST; }
static bool c_standby(void) { return sb_mode(&S.sb) == MODE_STANDBY; }
static bool c_arrived(void) { return (S.sb.alarm_bits & SB_ALM_ARRIVED) != 0u; }

static void wp_at(prog_wp_t *w, double north, double east, float kn, int act)
{
    world_t tmp = S.w;
    double la, lo;
    tmp.north_m = north;
    tmp.east_m = east;
    world_latlon(&tmp, &la, &lo);
    memset(w, 0, sizeof *w);
    w->lat_e7 = (int32_t)lround(la * 1e7);
    w->lon_e7 = (int32_t)lround(lo * 1e7);
    w->speed_kn = kn;
    w->radius_m = 30.0f;
    w->action = (uint8_t)act;
}

static int day_at_sea(void)
{
    sched_t *sc = &SC;
    memset(sc, 0, sizeof *sc);
    sc->seed = 2026u;
    sc->boot_deflected = true;                                  /* the stick is held at power-up */
    sim_init(sc);
    memset(&C, 0, sizeof C);
    s_md = fopen("day_at_sea_A.md", "w");
    if (s_md) fprintf(s_md, "| time | what happened | what the system did | expected | result |\n|---|---|---|---|---|\n");
    printf("| time | what happened | what the system did | expected | result |\n|---|---|---|---|---|\n");
    sb_t *b = &S.sb;
    char note[160];

    /* 1. power-up */
    run_s(3.0);
    row("power-up with the joystick held", !b->armed && sim_hal_throttle() < 0.6f && gear_pos(sim_hal_gear_pwm()) == 0,
        "outputs not armed, idle, neutral", "armed %d, throttle %.1f %%, gear %s", b->armed, sim_hal_throttle(),
        gear_pos(sim_hal_gear_pwm()) == 0 ? "N" : "?");
    S.jx = 0.0f; joy_apply(); run_s(1.5);
    row("joystick centred, neutral confirmed", b->armed && b->plim.fwd_pct == 30.0f && b->plim.rev_pct == 5.0f,
        "armed; power limit 30 / 5", "armed %d, power limit %.0f / %.0f %%", b->armed, b->plim.fwd_pct, b->plim.rev_pct);

    /* 2. TRAINING in calm water */
    S.w.sea_state = 0.0f;
    bool tr = sb_request_mode(b, MODE_TRAINING, WHY_USER);
    sync_mode();
    bool res = tr && wait_for_c(c_training_result, 25.0 * 60.0);
    run_s(5.0);
    bool acc = res && sb_confirm_training(b, true, 0.0f);
    sync_mode();
    row("TRAINING (PC, calm water), result accepted", tr && res && acc && sb_mode(b) == MODE_STANDBY,
        "calibration runs, idle + neutral, accept -> STANDBY", "started %d, result %s (code %d), accepted %d, mode %d",
        tr, res ? "reached" : "NOT reached", (int)b->ap.train.result, acc, (int)sb_mode(b));

    /* 3. HEADING LOCK out of the harbour, throttle on the joystick, +-1 / +-10 */
    press_btn(HAL_BTN_HDG_LOCK);
    S.jy = 0.45f; joy_apply(); run_s(60.0);
    float t0 = b->ap.target_heading, d1, d2, d3, d4;
    press_btn(HAL_BTN_PLUS_1);  d1 = b->ap.target_heading - t0;
    press_btn(HAL_BTN_PLUS_10); d2 = b->ap.target_heading - t0;
    press_btn(HAL_BTN_MINUS_1); d3 = b->ap.target_heading - t0;
    press_btn(HAL_BTN_MINUS_10); d4 = b->ap.target_heading - t0;
    row("HEADING LOCK, joystick throttle, +1 +10 -1 -10", sb_mode(b) == MODE_HDG_LOCK && S.w.speed_kn > 3.0f &&
        fabsf(d1 - 1.0f) < 0.01f && fabsf(d2 - 11.0f) < 0.01f && fabsf(d3 - 10.0f) < 0.01f && fabsf(d4) < 0.01f,
        "mode HDG LOCK, speed > 3 kn, target +1 +11 +10 0", "mode %d, %.1f kn, target %+.0f %+.0f %+.0f %+.0f",
        (int)sb_mode(b), S.w.speed_kn, d1, d2, d3, d4);

    /* 4. power limit 50 then 75 (PC request + code + CONFIRM each step); alarms acknowledged first.
     *    CONFIRM counts only with the stick centred: in HDG LOCK the throttle (stick Y) goes back first */
    S.jy = 0.0f; joy_apply(); run_s(3.0);
    ack_alarms();
    uint16_t code = 0;
    (void)sb_power_code(b, 0u, &code);
    (void)sb_power_raise(b, 0u, code);
    C.raise_req_t = S.t;
    run_s(11.0);                                                /* no CONFIRM */
    float no_conf = b->plim.fwd_pct;
    (void)sb_power_code(b, 0u, &code); (void)sb_power_raise(b, 0u, code); C.raise_req_t = S.t; bool c50 = do_confirm();
    float p50 = b->plim.fwd_pct;
    (void)sb_power_code(b, 0u, &code); (void)sb_power_raise(b, 0u, code); C.raise_req_t = S.t; bool c75 = do_confirm();
    row("power limit raised 30 -> 50 -> 75 %", no_conf == 30.0f && c50 && p50 == 50.0f && c75 && b->plim.fwd_pct == 75.0f,
        "no CONFIRM: stays 30; each step needs CONFIRM", "without CONFIRM %.0f %%; then %.0f %%, %.0f %%", no_conf, p50,
        b->plim.fwd_pct);

    /* 5. AUTO program: 6 waypoints, different leg speeds */
    static program_t pg;
    program_clear(&pg);
    double n0 = S.w.north_m, e0 = S.w.east_m;
    static const double L[6][2] = { {1200, 0}, {1200, 1200}, {2400, 1500}, {2400, 300}, {1200, -600}, {300, -200} };
    static const float KN6[6] = { 8, 12, 16, 10, 14, 6 };
    for (int i = 0; i < 6; i++) wp_at(&pg.wp[i], n0 + L[i][0], e0 + L[i][1], KN6[i], 0);
    pg.n = 6; pg.final_action = PROG_END_STOP;
    program_seal(&pg);
    prog_error_t err; uint8_t bad;
    sb_result_t pr = sb_program_set(b, &pg, &err, &bad);
    S.jy = 0.0f; joy_apply(); run_s(2.0);
    sb_result_t as = sb_auto_start(b);
    bool ca = do_confirm();
    row("AUTO program (6 waypoints, 6-16 kn) started with CONFIRM", pr == SB_OK && as == SB_OK && ca &&
        sb_mode(b) == MODE_AUTO, "program accepted, AUTO after CONFIRM", "program %d, start %d, CONFIRM %d, mode %d",
        (int)pr, (int)as, ca, (int)sb_mode(b));
    run_s(90.0);
    row("AUTO on leg 1", sb_mode(b) == MODE_AUTO && b->autop.state == AUTO_RUN && S.w.speed_kn > 5.0f,
        "running, about the leg speed (limit 75 %)", "mode %d, state %d, %.1f kn (target %.1f kn)", (int)sb_mode(b),
        (int)b->autop.state, S.w.speed_kn, b->cruise.target_kn);

    /* 6. sea state rising to 4 on a fast leg: slamming, step reductions, the power limit stays (STEP) */
    for (int i = 0; i < 6000 && b->cruise.target_kn < 12.0f; i++) run_until(S.t + 100u);
    run_s(30.0);
    S.w.sea_state = 4.0f;
    S.w.slam_scale = getenv("STORM_SLAM") ? (float)atof(getenv("STORM_SLAM")) : 2.6f;
    uint32_t imp0 = b->slam.impacts;
    float cap_min = 100.0f;
    for (int i = 0; i < 1800; i++) { run_until(S.t + 100u); cap_min = fminf(cap_min, b->slam.cap_pct); }
    row("sea state 4, slamming on the fast leg", b->slam.impacts > imp0 && cap_min < 100.0f && b->plim.fwd_pct == 75.0f,
        "impacts, throttle reduced in steps, power limit kept", "impacts %lu, lowest cap %.0f %%, power limit %.0f %%, "
        "state %d", (unsigned long)(b->slam.impacts - imp0), cap_min, b->plim.fwd_pct, (int)b->slam.state);
    S.w.sea_state = 2.0f;

    /* 7. GPS lost for 4 minutes: dead reckoning, AUTO goes on */
    S.w.gps_fail = true;
    bool dead = wait_for_c(c_dr_dead, 10.0);
    run_s(240.0);
    float unc = b->dr.unc_m;
    bool still = sb_mode(b) == MODE_AUTO;
    S.w.gps_fail = false;
    bool back = wait_for_c(c_dr_gps, 120.0);
    row("GPS lost for 4 min", dead && still && back, "dead reckoning, AUTO continues (slower), GPS trusted again",
        "DR %d, after 4 min mode %d, uncertainty %.0f m; GPS trusted again %d", dead, (int)sb_mode(b), unc, back);

    /* 8. GPS spoof attempt: the position jumps 800 m */
    S.w.gps_off_n += 800.0;
    bool caught = wait_for_c(c_not_gps, 15.0);
    bool alarm_s = (b->alarm_bits & SB_ALM_GPS_SUSPECT) != 0u;
    S.w.gps_off_n -= 800.0;
    bool rec = false;
    for (int k = 0; k < 120 && !rec; k++) {                     /* up to 10 min, traced every 5 s */
        run_s(5.0);
        rec = c_dr_gps();
        if (getenv("STORM_DEBUG"))
            printf("debug spoof +%3d s: dr state %d why %d unc %.0f m, mode %d, sog %.1f, gps fail %d\n", (k + 1) * 5,
                   (int)b->dr.state, (int)b->dr.why, b->dr.unc_m, (int)sb_mode(b), S.w.speed_kn, (int)S.w.gps_fail);
    }
    row("GPS spoof attempt (800 m jump)", caught && alarm_s && rec && sb_mode(b) == MODE_AUTO,
        "rejected, GPS SUSPECT, dead reckoning, recovers", "rejected %d, alarm %d, recovered %d, mode %d", caught, alarm_s,
        rec, (int)sb_mode(b));

    /* 9. compass glitch (0.8 s, below the 2 s timeout) */
    uint32_t a0 = b->alarm_bits;
    sim_hal_fail_heading(true); run_s(0.8); sim_hal_fail_heading(false); run_s(5.0);
    row("compass glitch 0.8 s", sb_mode(b) == MODE_AUTO && !((b->alarm_bits & ~a0) & SB_ALM_COMPASS),
        "no reaction (below the 2 s timeout)", "mode %d, new compass alarm %d", (int)sb_mode(b),
        ((b->alarm_bits & ~a0) & SB_ALM_COMPASS) != 0u);

    /* 10. onboard joystick takeover during AUTO -> MANUAL + SAVE; resume with PC + CONFIRM */
    uint16_t sv0 = b->save.seq;
    S.jx = 0.7f; joy_apply();
    run_s(0.5);
    bool man = sb_mode(b) == MODE_MANUAL;
    bool saved = b->save.seq != sv0 && b->save.trig == SB_SAVE_TAKEOVER;
    S.jx = 0.0f; joy_apply(); run_s(2.0);
    ack_alarms();
    sb_result_t rr = sb_auto_resume(b);
    bool cr = do_confirm();
    row("joystick takeover in AUTO, then resume", man && saved && rr == SB_OK && cr && sb_mode(b) == MODE_AUTO,
        "MANUAL at once + SAVE; resume = PC + CONFIRM", "MANUAL %d, SAVE %d, resume %d + CONFIRM %d -> mode %d", man, saved,
        (int)rr, cr, (int)sb_mode(b));

    /* 11. remote driving (HELM + onboard CONFIRM), a 2 s link dropout */
    S.rem_station = 3u; b->helm = 3u; b->helm_local = false;
    sb_result_t rq = sb_remote_request(b, 3u);
    S.rem_stream = true; S.rem_stale = false; S.rx_x = 0.0f; S.rx_lever = 0.8f;
    bool crm = do_confirm();
    run_s(10.0);
    bool driving = sb_mode(b) == MODE_REMOTE && b->remote.state == SB_REMOTE_DRIVING;
    S.rem_stream = false;
    uint32_t td = S.t;
    bool lost = wait_for_c(c_lost, 1.0);
    float t_lost = (S.t - td) * 0.001f;
    run_s(2.0 - t_lost);
    S.rem_stream = true;                                        /* data back after 2 s */
    run_s(1.0);
    bool no_resume = b->remote.state == SB_REMOTE_LOST && b->throttle_mode <= 0.01f && b->gear_demand == GEAR_N;
    bool sb5 = wait_for_c(c_standby, 6.0);
    row("remote driving, 2 s link dropout", rq == SB_OK && crm && driving && lost && t_lost < 0.6f && no_resume && sb5,
        "LOST within 0.4 s: idle + neutral + gyro hold, never resumes, STANDBY after 5 s",
        "request %d, CONFIRM %d, driving %d; LOST after %.2f s, data back -> still LOST/idle %d, STANDBY %d", (int)rq, crm,
        driving, t_lost, no_resume, sb5);
    S.rem_stream = false;

    /* 12. back to AUTO, arrival, stop in neutral, STANDBY */
    ack_alarms();
    rr = sb_auto_resume(b);
    cr = do_confirm();
    if (sb_mode(b) != MODE_AUTO) {                             /* after the remote STANDBY: AUTO start again */
        rr = sb_auto_resume(b) == SB_OK ? SB_OK : sb_auto_start(b);
        cr = do_confirm();
    }
    bool arr = wait_for_c(c_arrived, 40.0 * 60.0);
    run_s(10.0);
    snprintf(note, sizeof note, "%s", "");
    row("arrival at the last waypoint (STOP)", rr == SB_OK && cr && arr && sim_hal_throttle() < 0.6f &&
        gear_pos(sim_hal_gear_pwm()) == 0, "ARRIVED, idle + neutral", "resume %d + CONFIRM %d, arrived %d, throttle %.1f %%, "
        "gear %s, mode %d%s", (int)rr, cr, arr, sim_hal_throttle(), gear_pos(sim_hal_gear_pwm()) == 0 ? "N" : "?",
        (int)sb_mode(b), note);
    press_btn(HAL_BTN_STANDBY);
    row("STANDBY", sb_mode(b) == MODE_STANDBY, "STANDBY", "mode %d", (int)sb_mode(b));
    int fails = 0;
    for (int i = 0; i < nR; i++) fails += !R[i].pass;
    printf("\nDAY AT SEA (AURIX part): %d steps, %d failed; invariants during the whole day: %s\n", nR, fails,
           C.failed ? C.text : "all held");
    if (s_md) {
        fprintf(s_md, "\nInvariants checked after every control step during the whole day: %s\n",
                C.failed ? C.text : "all held");
        fclose(s_md);
    }
    return fails || C.failed;
}

/* Proof that the checks catch real violations: each case breaks one rule behind the
 * controller's back and the matching invariant must FAIL. */
static int selftest(void)
{
    static const struct { int inv; const char *what; } T[] = {
        { 7,  "power limit raised directly (no request, no CONFIRM)" },
        { 10, "mode forced MANUAL -> REMOTE (not in the table)" },
        { 1,  "throttle servo written above the limit (not by the servo clamp)" },
    };
    int bad = 0;
    printf("=== SELF-TEST: the checks must catch these ===\n");
    for (int k = 0; k < 3; k++) {
        sched_t *sc = &SC;
        memset(sc, 0, sizeof *sc);
        sc->seed = 77u;
        sc->dur_ms = 60000u;
        sim_init(sc);
        memset(&C, 0, sizeof C);
        S.jx = 0.0f;
        joy_apply();
        run_until(5000u);
        if (k == 0) {
            (void)plim_step_up(&S.sb.plim, 0u, S.sb.now_ms);
        } else if (k == 1) {
            S.jx = 0.8f;
            joy_apply();
            run_until(S.t + 1000u);                                    /* MANUAL */
            S.sb.modes.mode = MODE_REMOTE;
            check_step();                                              /* look before the next step */
        } else {
            S.sb.test_drive.on = false;
            run_until(S.t + 100u);
            hal_set_throttle_pwm(1800u);                               /* ~80 % at the 30 % limit */
            check_step();
        }
        run_until(S.t + 2000u);
        bool caught = C.failed && C.inv == T[k].inv;
        printf("%s  %-62s -> %s\n", caught ? "PASS" : "FAIL", T[k].what,
               C.failed ? C.text : "NOT DETECTED");
        bad += !caught;
    }
    return bad;
}

int main(int argc, char **argv)
{
    int episodes = 200;
    uint32_t seed0 = 1u;
    int min_min = 5, max_min = 30;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--quiet")) continue;
        if (!strcmp(argv[i], "--wrap")) return wrap_test();
        if (!strcmp(argv[i], "--selftest")) return selftest();
        if (!strcmp(argv[i], "--dayatsea")) return day_at_sea();
        if (!strcmp(argv[i], "--replay") && i + 1 < argc) {
            uint32_t sd = (uint32_t)strtoul(argv[i + 1], NULL, 10);
            uint32_t r = sd * 7u + 3u;
            gen_schedule(&SC, sd, (uint32_t)(5 + rndi(&r, 26)) * 60000u);
            s_verbose = true;
            bool f = episode(&SC, true);
            printf("seed %lu: %s\n", (unsigned long)sd, f ? C.text : "all invariants held");
            return f ? 1 : 0;
        }
        if (!strcmp(argv[i], "--soak") && i + 2 < argc) {
            double h = atof(argv[i + 1]);
            uint32_t sd = (uint32_t)strtoul(argv[i + 2], NULL, 10);
            gen_schedule(&SC, sd, (uint32_t)(h * 3600000.0));
            s_log = (i + 3 < argc && !strcmp(argv[i + 3], "--log"));
            sim_hal_real_micros(true);                  /* I18: real PC time of each control step */
            bool f = episode(&SC, false);
            printf("=== SOAK %.1f simulated hours, seed %lu: %s; control steps %lu, boots %lu, core 0 overruns %lu, "
                   "log queue drops %lu ===\n", h, (unsigned long)sd, f ? C.text : "all invariants held",
                   (unsigned long)C.steps, (unsigned long)S.boots, (unsigned long)g_timing.core0.overruns,
                   (unsigned long)g_logq_gw.dropped);
            printf("I18 control step with%s logging: %lu steps, longest step %lu us of the 50000 us budget (PC time); "
                   "log records %lu, files %lu, bytes %lu, queue drops core0 %lu / pi %lu (counted, never waited)\n",
                   s_log ? "" : "out", (unsigned long)s_ctrl_n, (unsigned long)s_ctrl_max_us,
                   (unsigned long)g_logstat.records, (unsigned long)g_logstat.files, (unsigned long)g_logstat.bytes,
                   (unsigned long)g_logq0.dropped, (unsigned long)g_logq_gw.dropped);
            {
                uint32_t acc = 0u, p999 = 0u;
                for (int b = 0; b < 64; b++) {
                    acc += s_hist[b];
                    if (acc >= s_ctrl_n - s_ctrl_n / 1000u) { p999 = (uint32_t)(b + 1) * 100u; break; }
                }
                printf("I18 control step time: average %.0f us, 99.9 %% of the steps below %lu us, steps over 1 ms: %lu "
                       "(PC; the TC375 must be measured on the board)\n", s_ctrl_n ? s_ctrl_sum_us / s_ctrl_n : 0.0,
                       (unsigned long)p999, (unsigned long)(s_ctrl_n - (s_hist[0] + s_hist[1] + s_hist[2] + s_hist[3] +
                       s_hist[4] + s_hist[5] + s_hist[6] + s_hist[7] + s_hist[8] + s_hist[9])));
            }
            report(1, h);
            return f ? 1 : 0;
        }
        if (!strcmp(argv[i], "--min-min") && i + 1 < argc) { min_min = atoi(argv[++i]); continue; }
        if (!strcmp(argv[i], "--max-min") && i + 1 < argc) { max_min = atoi(argv[++i]); continue; }
        if (i == 1) episodes = atoi(argv[i]);
        if (i == 2) seed0 = (uint32_t)strtoul(argv[i], NULL, 10);
    }
    double hours = 0.0;
    int failing = 0;
    for (int k = 0; k < episodes; k++) {
        uint32_t sd = seed0 + (uint32_t)k;
        uint32_t r = sd * 7u + 3u;
        uint32_t dur = (uint32_t)(min_min + rndi(&r, max_min - min_min + 1)) * 60000u;
        gen_schedule(&SC, sd, dur);
        hours += dur / 3600000.0;
        if (episode(&SC, false)) {
            int inv = C.inv;
            char text[200];
            snprintf(text, sizeof text, "%s", C.text);
            uint32_t at = C.t;
            ST.fails[inv]++;
            failing++;
            int kept = shrink(&SC, inv);
            if (ST.fails[inv] == 1u) {
                ST.first_fail_seed[inv] = sd;
                snprintf(ST.first_fail_text[inv], sizeof ST.first_fail_text[inv], "%s at %.1f s; minimal "
                         "reproduction: %d of %d events (storm.exe --replay %lu)", text, at * 0.001, kept, SC.n,
                         (unsigned long)sd);
            }
            printf("FAIL seed %lu %s: %s at %.1f s - minimal %d events:\n", (unsigned long)sd, INV_NAMES[inv],
                   text, at * 0.001, kept);
            for (int i = 0; i < SC.n; i++)
                if (SC.on[i]) printf("      %8.1f s  %s a=%d b=%d c=%d f=%.2f g=%.2f\n", SC.ev[i].t * 0.001,
                                     EV_NAMES[SC.ev[i].kind], SC.ev[i].a, SC.ev[i].b, SC.ev[i].c, SC.ev[i].f,
                                     SC.ev[i].g);
        }
        if ((k + 1) % 100 == 0) {
            printf("... %d episodes, %.0f simulated hours, %d failing\n", k + 1, hours, failing);
            fflush(stdout);
        }
    }
    report(episodes, hours);
    return failing ? 1 : 0;
}
