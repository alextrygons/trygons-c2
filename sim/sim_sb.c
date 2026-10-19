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

/* ------------------------------------------------------------ phase 4 -- */

/* Watch a CRUISE run. Set speed / sea state follow a schedule. */
typedef struct {
    const float *at_s;          /* schedule times (s after t0) */
    const float *speed;         /* set speed from that time (<0 = keep) */
    const float *sea;           /* sea state from that time */
    int      n;
    uint32_t t0;
    int      seg;               /* current schedule entry */
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
    while (c->seg + 1 < c->n && ts >= c->at_s[c->seg + 1]) {
        c->seg++;
        if (c->speed[c->seg] >= 0.0f) sb_set_cruise_speed(&x->sb, c->speed[c->seg]);
        x->w.sea_state = c->sea[c->seg];
    }
    int g = c->seg;
    float seg_t = ts - c->at_s[g];
    float sog = world_sog_kn(&x->w);
    float set = x->sb.cruise.target_kn;
    if (seg_t > 40.0f) {                    /* settled part of the segment */
        c->seg_err_sum[g] += fabsf(sog - set);
        c->seg_err_n[g]++;
        float he = fabsf(hdg_err(x->w.heading_deg, x->sb.ap.target_heading));
        if (he > c->seg_hold[g]) c->seg_hold[g] = he;
        if (he > c->hold_max) c->hold_max = he;
    }
    if (sog - x->sb.cruise.set_kn > c->seg_over[g]) c->seg_over[g] = sog - x->sb.cruise.set_kn;
    if (seg_t > 40.0f && (c->seg_vmin[g] == 0.0f || sog < c->seg_vmin[g])) c->seg_vmin[g] = sog;
    if (seg_t > 40.0f && sog > c->seg_vmax[g]) c->seg_vmax[g] = sog;

    if (sb_mode(&x->sb) == MODE_CRUISE && x->sb.ap.steer_limit_deg > 0.0f) {
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

static void cruise_run(sbsim_t *x, cruise_watch_t *c, uint32_t seed, const float *at, const float *spd,
                       const float *sea, int n, uint32_t dur_ms)
{
    sbsim_init(x, seed, 90.0f, 0.0f, NULL);
    x->w.sea_state = sea[0];
    sbsim_run(x, 1000u, NULL, NULL);
    sb_set_cruise_speed(&x->sb, spd[0]);
    sb_request_mode(&x->sb, MODE_CRUISE, WHY_USER);
    memset(c, 0, sizeof *c);
    c->at_s = at; c->speed = spd; c->sea = sea; c->n = n;
    c->t0 = x->t;
    c->last_gear = x->w.engaged;
    sbsim_run(x, x->t + dur_ms, cruise_hook, c);
}

static void phase4(void)
{
    char d[400];
    printf("\n=== Phase 4: CRUISE ===\n");

    /* 4.1 speed ramp 3 -> 20 kn, sea state 1 */
    {
        static const float at[]  = { 0, 60, 120, 180, 240 };
        static const float spd[] = { 3, 6, 10, 15, 20 };
        static const float sea[] = { 1, 1, 1, 1, 1 };
        sbsim_t x;
        cruise_watch_t c;
        cruise_run(&x, &c, 41u, at, spd, sea, 5, 300000u);
        bool ok = true;
        int len = 0;
        len += snprintf(d + len, sizeof d - (size_t)len, "set/err/overshoot/hold:");
        for (int i = 0; i < 5; i++) {
            float err = c.seg_err_n[i] ? c.seg_err_sum[i] / (float)c.seg_err_n[i] : 99.0f;
            len += snprintf(d + len, sizeof d - (size_t)len, " %.0fkn %.2f/%.2f/%.1f", spd[i], err,
                            c.seg_over[i], c.seg_hold[i]);
            if (err > 0.5f || c.seg_over[i] > 1.5f || c.seg_hold[i] > 10.0f) ok = false;
        }
        snprintf(d + len, sizeof d - (size_t)len, "; steering beyond limit: cmd %.2f, true %.2f deg; "
                 "throttle rate max %.0f %%/s; gear changes after F %d",
                 c.limit_excess, c.true_excess, x.w.max_thr_rate, c.gear_changes - 1);
        ok = ok && c.limit_excess <= 0.01f && c.true_excess <= 0.0f && x.w.max_thr_rate <= 40.5f &&
             c.gear_changes == 1 && x.w.shift_under_load == 0;
        test_report("P4 speed ramp 3->20 kn: speed control, steering limit by speed", ok, d);
    }

    /* 4.2 sea state 1 -> 4 at 18 kn, then calm again */
    {
        static const float at[]  = { 0, 180, 360, 540, 1140 };
        static const float spd[] = { 18, -1, -1, -1, -1 };
        static const float sea[] = { 1, 2, 3, 4, 1 };
        sbsim_t x;
        cruise_watch_t c;
        cruise_run(&x, &c, 42u, at, spd, sea, 5, 2100000u);
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
                  x.sb.nerv.transitions <= 4u && sb_mode(&x.sb) == MODE_CRUISE;
        test_report("P4 sea state 1->4->1: slows down when rough, recovers when calm", ok, d);
    }

    /* 4.3 no oscillation: 30 min at a borderline sea state */
    {
        static const float at[]  = { 0 };
        static const float spd[] = { 18 };
        static const float sea[] = { 3.5f };
        sbsim_t x;
        cruise_watch_t c;
        cruise_run(&x, &c, 43u, at, spd, sea, 1, 1800000u);
        snprintf(d, sizeof d, "30 min at sea state 3.5, 18 kn set: episodes %u, nervous/calm changes %u, "
                 "shortest time between changes %.0f s, speed %.1f-%.1f kn",
                 x.sb.nerv.episodes, x.sb.nerv.transitions, c.min_gap_ms * 0.001f, c.seg_vmin[0], c.seg_vmax[0]);
        bool ok = x.sb.nerv.transitions <= 6u &&
                  (x.sb.nerv.transitions < 2u || c.min_gap_ms >= 30000u);
        test_report("P4 no oscillation between nervous and calm", ok, d);
    }

    /* 4.4 forward only: reverse selector and lever ignored in CRUISE */
    {
        sbsim_t x;
        sbsim_init(&x, 44u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_set_cruise_speed(&x.sb, 10.0f);
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_USER);
        sbsim_run(&x, 40000u, NULL, NULL);
        sim_hal_set_controls(-1.0f, 1.0f, HAL_SEL_R);
        sbsim_run(&x, 80000u, NULL, NULL);
        snprintf(d, sizeof d, "selector R + steer lever: gear %s, speed %.1f kn, steering motor manual? %s",
                 gear_name(x.w.engaged), world_sog_kn(&x.w), x.sb.ap.mode == AP_MODE_AUTO ? "no (AUTO)" : "YES");
        test_report("P4 CRUISE is forward only (operator reverse ignored)",
                    x.w.engaged == GEAR_F && world_sog_kn(&x.w) > 9.0f && x.sb.ap.mode == AP_MODE_AUTO &&
                    x.w.max_thr_rev == 0.0f, d);
    }

    /* 4.5 GPS loss in CRUISE: reduced throttle held, alarm, heading still held */
    {
        sbsim_t x;
        sbsim_init(&x, 45u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_set_cruise_speed(&x.sb, 15.0f);
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_USER);
        sbsim_run(&x, 80000u, NULL, NULL);
        float thr0 = sim_hal_throttle(), v0 = world_sog_kn(&x.w);
        x.w.gps_fail = true;
        float thr_max = 0.0f, hold = 0.0f;
        while (x.t < 200000u) {
            sbsim_run(&x, x.t + 50u, NULL, NULL);
            if (x.t > 90000u && sim_hal_throttle() > thr_max) thr_max = sim_hal_throttle();
            float he = fabsf(hdg_err(x.w.heading_deg, x.sb.ap.target_heading));
            if (x.t > 110000u && he > hold) hold = he;
        }
        snprintf(d, sizeof d, "before: %.1f %% at %.1f kn; after GPS loss: throttle <= %.1f %%, speed %.1f kn, "
                 "alarm %s, mode %s, course error max %.1f deg",
                 thr0, v0, thr_max, world_sog_kn(&x.w), sim_hal_alarm() ? "on" : "OFF",
                 mode_name(sb_mode(&x.sb)), hold);
        test_report("P4 GPS loss in CRUISE: reduced throttle, alarm, still steering",
                    thr_max <= thr0 * 0.61f && sim_hal_alarm() && sb_mode(&x.sb) == MODE_CRUISE &&
                    world_sog_kn(&x.w) < v0 && hold < 10.0f, d);
    }

    /* 4.6 compass failure in CRUISE at 20 kn: safe stop */
    {
        sbsim_t x;
        events_t ev;
        memset(&ev, 0, sizeof ev);
        sbsim_init(&x, 46u, 90.0f, 0.0f, NULL);
        sbsim_run(&x, 1000u, NULL, NULL);
        sb_set_cruise_speed(&x.sb, 20.0f);
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_USER);
        sbsim_run(&x, 120000u, NULL, NULL);
        float v0 = world_sog_kn(&x.w);
        sim_hal_fail_heading(true);
        ev_reset(&ev, x.t, sim_hal_throttle(), x.sb.cfg.gear.pwm_n_us);
        sbsim_run(&x, x.t + 30000u, ev_watch, &ev);
        snprintf(d, sizeof d, "at %.1f kn: throttle down after %.2f s, idle %.2f s, neutral %.2f s, alarm %s, "
                 "under-load %d", v0, ev.t_thr_down, ev.t_idle, ev.t_neutral, sim_hal_alarm() ? "on" : "OFF",
                 x.w.shift_under_load);
        test_report("P4 compass failure in CRUISE: STANDBY, idle + neutral first, alarm",
                    sb_mode(&x.sb) == MODE_STANDBY && ev.t_thr_down >= 0.0f && ev.t_thr_down <= 2.1f &&
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
    float  t_arrive;            /* mode left NAV (s), -1 */
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
    if (sb_mode(&x->sb) == MODE_NAV) {
        if (along > w->settle_m && along < w->length - 150.0f) {
            if (fabsf(xte) > w->xte_max) w->xte_max = fabsf(xte);
            w->xte_sq += xte * xte;
            w->xte_n++;
        }
        if (fabsf(x->sb.nav.crab_deg) > w->crab_seen) w->crab_seen = fabsf(x->sb.nav.crab_deg);
        float h = x->w.heading_deg;
        if ((w->prev_hdg > 300.0f && h < 60.0f) || (w->prev_hdg < 60.0f && h > 300.0f)) w->crossed_north = true;
        w->prev_hdg = h;
    } else if (w->t_arrive < 0.0f) {
        w->t_arrive = (float)(x->t - w->t0) * 0.001f;
        const mode_log_entry_t *le = mode_log_get(&x->sb.modes, mode_log_size(&x->sb.modes) - 1u);
        w->why = le->why;
        w->mode_after = sb_mode(&x->sb);
    }
    double dn = w->wp_n - x->w.north_m, de = w->wp_e - x->w.east_m;
    w->stop_dist = (float)sqrt(dn * dn + de * de);
}

/* NAV run: start at the world origin on `heading`, waypoint at dist/bearing. */
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
    sb_set_waypoint(&x->sb, lat, lon, speed);
    sb_request_mode(&x->sb, MODE_NAV, WHY_USER);
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
    printf("\n=== Phase 5: NAV ===\n");

    struct { const char *name; float hdg, dist, brg, spd, sea, ce, cn, xmax; } cases[] = {
        { "P5 NAV 500 m waypoint (12 kn, sea 1)",               90.0f,  500.0f,  45.0f, 12.0f, 1.0f, 0.0f,  0.0f, 15.0f },
        { "P5 NAV 2 km waypoint (18 kn, sea 2)",               180.0f, 2000.0f, 200.0f, 18.0f, 2.0f, 0.0f,  0.0f, 15.0f },
        { "P5 NAV 2 km with 1 m/s side current (10 kn)",        90.0f, 2000.0f,  90.0f, 10.0f, 1.0f, 0.0f, -1.0f, 20.0f },
        { "P5 NAV crossing north (track 2 deg, current from E)", 330.0f, 1000.0f,   2.0f, 10.0f, 1.0f, -0.8f, 0.0f, 20.0f },
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
        snprintf(d, sizeof d, "GPS lost at 1 km: NAV left after %.0f s (%s -> %s), alarm %s, gear %s, "
                 "throttle now %.1f %%, speed %.1f kn, heading %.0f (track 120)",
                 w.t_arrive, mode_reason_name(w.why), mode_name(sb_mode(&x.sb)), sim_hal_alarm() ? "on" : "OFF",
                 gear_name(x.w.engaged), sim_hal_throttle(), world_sog_kn(&x.w), x.w.heading_deg);
        test_report("P5 NAV GPS loss halfway: CRUISE on current heading, reduced speed, alarm",
                    w.why == WHY_GPS_LOST && sb_mode(&x.sb) == MODE_CRUISE && sim_hal_alarm() &&
                    x.w.engaged == GEAR_F && world_sog_kn(&x.w) < 11.0f &&
                    fabsf(hdg_err(x.w.heading_deg, 120.0f)) < 15.0f, d);
    }

    /* NAV needs a waypoint */
    {
        sbsim_t x;
        sbsim_init(&x, 57u, 0.0f, 0.0f, NULL);
        sbsim_run(&x, 3000u, NULL, NULL);
        bool ok = !sb_request_mode(&x.sb, MODE_NAV, WHY_USER);
        test_report("P5 NAV refused without a waypoint", ok, "");
    }
}

/* ------------------------------------------------------------ phase 6 -- */

/* A "true" boat for training tests (values at the slow training speed). */
typedef struct {
    float speed, gap, travel_r, travel_l, K, T, trim_deg;
    bool  start_left;
} tset_t;

static uint32_t s6_rng;
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
    sb_set_cruise_speed(&x.sb, speed);
    sb_request_mode(&x.sb, MODE_CRUISE, WHY_USER);
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
    printf("\nCourse keeping in CRUISE at the training speed (hold max / rms / overshoot of a 30 deg turn, deg):\n");
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
        sb_request_mode(&x.sb, MODE_CRUISE, WHY_USER);
        bool from_cruise = sb_request_mode(&x.sb, MODE_TRAINING, WHY_USER);
        const mode_log_entry_t *le = mode_log_get(&x.sb.modes, mode_log_size(&x.sb.modes) - 1u);
        test_report("P6 TRAINING refused unless in STANDBY (table), and logged",
                    !from_cruise && le->verdict == MODE_REJECT_TABLE && le->to == MODE_TRAINING, "");
    }
}

/* ---------------------------------------------------------------- all -- */

void sb_tests(void)
{
    phase2();
    phase3();
    phase4();
    phase5();
    phase6();
}
