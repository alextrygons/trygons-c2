/*
 * sim_sb.c - Small-boat feature tests (one block per phase).
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hal_sim.h"
#include "sim_run.h"
#include "sim_world.h"
#include "smallboat.h"

/* ------------------------------------------------------------ harness -- */

typedef struct sbsim sbsim_t;
typedef void (*sb_hook_t)(sbsim_t *x, void *ctx);

struct sbsim {
    world_t  w;
    sb_t     sb;
    uint32_t t;             /* ms */
    uint32_t next_ctrl;
};

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
    world_publish(&x->w, 0);
    hal_init();
    if (cfg) c = *cfg; else sb_default_config(&c);
    sb_init(&x->sb, &c);
}

/* Run until absolute time until_ms; hook after every control step. */
static void sbsim_run(sbsim_t *x, uint32_t until_ms, sb_hook_t hook, void *ctx)
{
    while (x->t <= until_ms) {
        world_publish(&x->w, x->t);
        if (x->t >= x->next_ctrl) {
            x->next_ctrl += AP_PERIOD_MS;
            sb_step(&x->sb);
            if (hook) hook(x, ctx);
        }
        world_step(&x->w, PHYSICS_DT_MS * 0.001f);
        x->t += PHYSICS_DT_MS;
    }
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
        snprintf(d, sizeof d, "%d pairs, %d illegal rejected, %d wrong, %d/25 logged correctly",
                 MODE_COUNT * MODE_COUNT, illegal, wrong, logged_ok);
        test_report("P2 mode table: every illegal transition rejected and logged",
                    wrong == 0 && logged_ok == 25, d);
    }

    /* 2.2 preconditions: CRUISE without compass, NAV without GPS */
    {
        sbsim_t x;
        sbsim_init(&x, 11u, 90.0f, 0.0f, NULL);
        x.w.gps_fail = true;
        sbsim_run(&x, 3000u, NULL, NULL);
        bool nav = sb_request_mode(&x.sb, MODE_NAV, WHY_USER);
        sim_hal_fail_heading(true);
        sbsim_run(&x, 6000u, NULL, NULL);
        bool cruise = sb_request_mode(&x.sb, MODE_CRUISE, WHY_USER);
        const mode_log_entry_t *e = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        snprintf(d, sizeof d, "NAV without GPS %s, CRUISE without compass %s, last log: %s->%s %s",
                 nav ? "ACCEPTED" : "rejected", cruise ? "ACCEPTED" : "rejected",
                 mode_name(e->from), mode_name(e->to), e->verdict == MODE_REJECT_PRECOND ? "precondition" : "?");
        test_report("P2 mode preconditions (sensors) enforced and logged",
                    !nav && !cruise && e->verdict == MODE_REJECT_PRECOND, d);
    }

    /* 2.3 gear shift request at high throttle (F 80 % -> R) */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 12u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_TEST);
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
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_TEST);
        sb_test_drive(&x.sb, true, GEAR_F, 100.0f);
        sbsim_run(&x, 60000u, NULL, NULL);
        float v0 = x.w.speed_kn, h0 = x.w.heading_deg, hmax = 0.0f;
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sim_hal_press(HAL_BTN_STANDBY_AUTO);
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
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_TEST);
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
        test_report("P2 compass failure at full throttle: STANDBY, idle + neutral, alarm",
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
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_TEST);
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
        snprintf(d, sizeof d, "fwd 100 %% -> %.1f kn, rev (limited to 30 %%) -> %.1f kn, GPS %d msgs in 59 s, "
                 "position noise %.1f m rms, worst SOG error %.2f kn",
                 vf, vr, msgs, sqrt(err_sq / (msgs ? msgs : 1)), sog_err);
        test_report("P2 sim: engine thrust, speed fwd/rev, GPS 1 Hz with noise",
                    vf > 19.0f && vr < -2.0f && vr > -4.0f && msgs >= 57 && msgs <= 61, d);
    }
}

/* ------------------------------------------------------------ phase 3 -- */

/* A scripted operator: control positions from time t (s after start). */
typedef struct {
    float          t;
    float          steer, thr;
    hal_gear_sel_t gear;
} ctl_step_t;

typedef struct {
    const ctl_step_t *steps;
    int      n;
    uint32_t t0;
    /* watch */
    float est_max, est_min, ang_max, ang_min, v_max, v_min;
    float thr_in_neutral;       /* worst throttle while selector is N */
    float hdg_min, hdg_max, hdg_unwrap, hdg_prev;
} script_t;

static void script_hook(sbsim_t *x, void *ctx)
{
    script_t *s = (script_t *)ctx;
    float ts = (float)(x->t - s->t0) * 0.001f;
    int k = -1;
    for (int i = 0; i < s->n; i++) if (ts >= s->steps[i].t) k = i;
    if (k >= 0) sim_hal_set_controls(s->steps[k].steer, s->steps[k].thr, s->steps[k].gear);

    float est = x->sb.ap.est_angle, ang = x->w.rud.angle_deg;
    if (est > s->est_max) s->est_max = est;
    if (est < s->est_min) s->est_min = est;
    if (ang > s->ang_max) s->ang_max = ang;
    if (ang < s->ang_min) s->ang_min = ang;
    if (x->w.speed_kn > s->v_max) s->v_max = x->w.speed_kn;
    if (x->w.speed_kn < s->v_min) s->v_min = x->w.speed_kn;
    if (k >= 0 && s->steps[k].gear == HAL_SEL_N && gear_is(&x->sb.gear, GEAR_N) &&
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

static void phase3(void)
{
    char d[300];
    printf("\n=== Phase 3: MANEUVER ===\n");

    /* 3.1 docking-style sequence */
    {
        static const ctl_step_t dock[] = {
            {  0.0f,  0.0f, 1.0f, HAL_SEL_F },   /* ahead, lever full (-> 30 %) */
            { 15.0f,  1.0f, 1.0f, HAL_SEL_F },   /* full right */
            { 27.0f, -1.0f, 1.0f, HAL_SEL_F },   /* back left */
            { 37.0f,  0.0f, 0.5f, HAL_SEL_N },   /* neutral, lever left half open */
            { 45.0f,  0.0f, 1.0f, HAL_SEL_R },   /* astern, lever full */
            { 50.0f, -1.0f, 1.0f, HAL_SEL_R },   /* full left while reversing */
            { 62.0f,  1.0f, 1.0f, HAL_SEL_R },   /* back right */
            { 72.0f,  0.0f, 0.0f, HAL_SEL_N },
            { 80.0f,  0.0f, 0.3f, HAL_SEL_F },
        };
        sbsim_t x;
        script_t s;
        sbsim_init(&x, 21u, 45.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        bool ok_mode = sb_request_mode(&x.sb, MODE_MANEUVER, WHY_USER);
        script_start(&s, &x, dock, (int)(sizeof dock / sizeof dock[0]));
        sbsim_run(&x, 91000u, script_hook, &s);
        sim_hal_press(HAL_BTN_STANDBY_AUTO);
        sbsim_run(&x, 105000u, script_hook, &s);
        float lim = x.sb.cfg.steer.outboard.max_angle_deg;
        snprintf(d, sizeof d, "max throttle fwd %.1f %% / rev %.1f %% (limit %.0f), speed %+.1f..%+.1f kn, "
                 "outboard est %+.1f..%+.1f deg, true %+.1f..%+.1f deg (stops +-35), end-stop push %.1f s, "
                 "heading swing %+.0f..%+.0f deg, throttle in N %.1f %%, under-load %d, F/R w/o N %d",
                 x.w.max_thr_fwd, x.w.max_thr_rev, x.sb.cfg.maneuver_throttle_pct, s.v_min, s.v_max,
                 s.est_min, s.est_max, s.ang_min, s.ang_max, x.w.rud.stall_s, s.hdg_min, s.hdg_max,
                 s.thr_in_neutral, x.w.shift_under_load, x.w.fr_without_neutral);
        bool ok = ok_mode && x.w.max_thr_fwd <= x.sb.cfg.maneuver_throttle_pct + 0.01f &&
                  x.w.max_thr_rev <= x.sb.cfg.maneuver_throttle_pct + 0.01f &&
                  s.v_max > 3.0f && s.v_min < -1.5f &&
                  s.est_max >= lim - 0.01f && s.est_min <= -lim + 0.01f &&
                  s.est_max <= lim + 0.01f && s.est_min >= -lim - 0.01f &&
                  x.w.rud.stall_s == 0.0f && s.thr_in_neutral <= 0.0f &&
                  x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0 &&
                  (s.hdg_max - s.hdg_min) > 30.0f &&
                  sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f &&
                  gear_is(&x.sb.gear, GEAR_N) && sim_hal_motor_command() == 0.0f;
        test_report("P3 docking sequence: fwd, neutral, rev, full-angle turns, throttle <= 30 %", ok, d);
    }

    /* 3.2 fast gear toggling by the user (every 200 ms, lever full) */
    {
        static ctl_step_t tog[52];
        static const hal_gear_sel_t seq[3] = { HAL_SEL_F, HAL_SEL_R, HAL_SEL_N };
        for (int i = 0; i < 50; i++) {
            tog[i].t = 0.2f * (float)i;
            tog[i].steer = 0.0f;
            tog[i].thr = 1.0f;
            tog[i].gear = seq[i % 3];
        }
        tog[50].t = 10.0f; tog[50].steer = 0.0f; tog[50].thr = 1.0f; tog[50].gear = HAL_SEL_R;
        tog[51].t = 30.0f; tog[51].steer = 0.0f; tog[51].thr = 1.0f; tog[51].gear = HAL_SEL_R;
        sbsim_t x;
        script_t s;
        sbsim_init(&x, 22u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_request_mode(&x.sb, MODE_MANEUVER, WHY_USER);
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
                 "under-load %d, F/R w/o N %d; afterwards gear %s, throttle %.1f %%, speed %+.1f kn",
                 thr_toggling, engage_changes, x.w.shift_under_load, x.w.fr_without_neutral,
                 gear_name(x.w.engaged), sim_hal_throttle(), x.w.speed_kn);
        test_report("P3 fast gear toggling: always via neutral at idle, ends in last request",
                    thr_toggling <= 1.0f && x.w.shift_under_load == 0 && x.w.fr_without_neutral == 0 &&
                    x.w.engaged == GEAR_R && gear_is(&x.sb.gear, GEAR_R) &&
                    sim_hal_throttle() <= 30.01f && x.w.speed_kn < -1.0f, d);
    }

    /* 3.3 STANDBY while reversing with full lever: idle + neutral, steering off */
    {
        static const ctl_step_t rev[] = { { 0.0f, -1.0f, 1.0f, HAL_SEL_R } };
        sbsim_t x;
        script_t s;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 23u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_request_mode(&x.sb, MODE_MANEUVER, WHY_USER);
        script_start(&s, &x, rev, 1);
        sbsim_run(&x, 20000u, script_hook, &s);
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sim_hal_press(HAL_BTN_STANDBY_AUTO);
        sbsim_run(&x, 40000u, ev_watch, &ev);        /* operator still holds the controls */
        snprintf(d, sizeof d, "throttle down after %.2f s, idle %.2f s, neutral %.2f s, steering off %.2f s, "
                 "motor after off %d, under-load %d, speed now %+.1f kn",
                 ev.t_thr_down, ev.t_idle, ev.t_neutral, ev.t_steer_off, ev.steer_after_off,
                 x.w.shift_under_load, x.w.speed_kn);
        test_report("P3 STANDBY while reversing: idle + neutral, steering off, controls ignored",
                    ev.t_thr_down >= 0.0f && ev.t_thr_down <= 0.1f && ev.t_neutral > ev.t_idle &&
                    ev.steer_after_off == 0 && x.w.shift_under_load == 0 &&
                    sb_mode(&x.sb) == MODE_STANDBY && sim_hal_throttle() <= 0.0f, d);
    }

    /* 3.4 compass failure in MANEUVER -> STANDBY (every fault -> STANDBY) */
    {
        static const ctl_step_t fwd[] = { { 0.0f, 0.0f, 1.0f, HAL_SEL_F } };
        sbsim_t x;
        script_t s;
        sbsim_init(&x, 24u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_request_mode(&x.sb, MODE_MANEUVER, WHY_USER);
        script_start(&s, &x, fwd, 1);
        sbsim_run(&x, 20000u, script_hook, &s);
        sim_hal_fail_heading(true);
        sbsim_run(&x, 35000u, script_hook, &s);
        sb_mode_t after = sb_mode(&x.sb);
        bool back = sb_request_mode(&x.sb, MODE_MANEUVER, WHY_USER);   /* manual still possible */
        snprintf(d, sizeof d, "mode after fault %s, throttle %.1f %%, gear %s, alarm %s, under-load %d; "
                 "MANEUVER again without compass: %s",
                 mode_name(after), sim_hal_throttle(), gear_name(x.w.engaged),
                 sim_hal_alarm() ? "on" : "OFF", x.w.shift_under_load, back ? "allowed" : "refused");
        test_report("P3 compass failure in MANEUVER: STANDBY, idle + neutral, alarm",
                    after == MODE_STANDBY && x.w.engaged == GEAR_N && sim_hal_alarm() &&
                    x.w.shift_under_load == 0 && back, d);
    }
}

/* ---------------------------------------------------------------- all -- */

void sb_tests(void)
{
    phase2();
    phase3();
}
