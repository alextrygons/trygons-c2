/*
 * sim_main.c - Runs the autopilot against the boat model.
 *
 * Usage:
 *   autopilot_sim [sea_state 1..5 [wave_seed]]   big boat, rudder feedback
 *                                                (defaults 3, 2024)
 *   autopilot_sim outboard [wave_seed]           small boat test table,
 *                                                sea state 3
 *
 * Big-boat scenario (5 minutes, written to sim_output.csv):
 *   t =   0 s  boat on 90 deg, press Standby/Auto -> AUTO, holds 90
 *   t =  60 s  press +10 three times -> new course 120
 *   t = 240 s  compass stops sending data
 *              -> rudder frozen at ~240.4 s, STANDBY + alarm at ~242 s
 *   t = 300 s  end
 * Extra test (no CSV): compass drops out for 1.2 s in the middle of the
 * turn while the motor is busy -> motor must stop, then AUTO continues.
 *
 * Outboard: same scenarios plus a turn across north and a 10-minute
 * straight run, for several TRUE motor speeds and gaps, while the
 * autopilot keeps its default config and gets no angle feedback.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "autopilot.h"
#include "boat_model.h"
#include "hal_sim.h"

#define PHYSICS_DT_MS   10u         /* boat model runs at 100 Hz */
#define TURN_WINDOW_MS  40000u      /* turn metrics: 40 s after the course change */
#define BOAT_SPEED_KN   6.0f

typedef struct {
    uint8_t  sea_state;
    uint32_t seed;              /* wave pattern */
    float    start_heading;
    uint32_t end_ms;
    uint32_t turn_at_ms;        /* +30 deg (three +10 presses), 0 = no turn */
    uint32_t fail_at_ms;        /* compass dies for good (0 = never) */
    uint32_t dropout_at_ms;     /* short compass dropout (0 = none)  */
    uint32_t dropout_len_ms;
    bool     outboard;          /* small boat, no rudder feedback */
    float    true_speed_pos;    /* outboard only: TRUE actuator values */
    float    true_speed_neg;
    float    true_gap;
    FILE    *csv;               /* NULL = no CSV */
} scenario_t;

typedef struct {
    int   motor_switches;       /* motor on<->off changes, whole run */
    int   pulses;               /* motor starts (or reversals), whole run */
    float hold_max, hold_rms;   /* course error while holding (outside turn/failure) */
    float hold_max_last5;       /* same, last 5 minutes only */
    float reach_s, overshoot;   /* course change */
    float frozen_after_s;       /* fail -> motor stopped for good */
    float standby_s;            /* fail -> STANDBY + alarm */
    int   motor_while_stale;    /* steps with motor on while compass stale */
    bool  auto_at_end;          /* still AUTO, no alarm at the end */
    float est_err_max;          /* outboard: worst |estimate - true angle| */
    float est_err_max_first5;   /* ... first 5 minutes */
    float est_err_max_last5;    /* ... last 5 minutes */
    float est_err_end;          /* ... at the end (signed) */
    float stall_s;              /* motor pushing against the mechanical stop */
    float integrator_end;
    float alarm_s;              /* first time the alarm came on in AUTO (-1 = never) */
} results_t;

static int self_test(void)
{
    struct { float target, heading, expect; } cases[] = {
        { 350.0f,  10.0f, -20.0f },
        {  10.0f, 350.0f,  20.0f },
        {  90.0f,  90.0f,   0.0f },
        {   0.0f, 270.0f,  90.0f },
        { 180.0f,   0.0f, 180.0f },
    };
    int fails = 0;
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        float e = autopilot_heading_error(cases[i].target, cases[i].heading);
        if (fabsf(e - cases[i].expect) > 1e-3f) {
            printf("SELF-TEST FAIL: target %.0f heading %.0f -> %.1f (expected %.1f)\n",
                   cases[i].target, cases[i].heading, e, cases[i].expect);
            fails++;
        }
    }
    if (fails == 0) printf("Self-test: heading error wraparound OK\n");
    return fails;
}

static results_t run(const scenario_t *sc)
{
    results_t r;
    memset(&r, 0, sizeof r);
    r.reach_s = -1.0f;
    r.frozen_after_s = -1.0f;
    r.standby_s = -1.0f;
    r.alarm_s = -1.0f;

    boat_t boat;
    rudder_actuator_t rudder;
    autopilot_t ap;
    ap_config_t cfg;

    boat_init(&boat, sc->start_heading, BOAT_SPEED_KN, sc->seed);
    boat.wave_amp_deg = (float)sc->sea_state;   /* simulated sea matches the setting */
    if (sc->outboard)
        rudder_init_outboard(&rudder, sc->true_speed_pos, sc->true_speed_neg, sc->true_gap);
    else
        rudder_init(&rudder);

    sim_hal_set_time_ms(0);
    sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(!sc->outboard);
    hal_init();
    if (sc->outboard) autopilot_default_config_outboard(&cfg);
    else              autopilot_default_config(&cfg);
    cfg.sea_state = sc->sea_state;
    autopilot_init(&ap, &cfg);

    double hold_sq = 0.0;
    int hold_n = 0;
    float prev_cmd = 0.0f;
    uint32_t last_fix_ms = 0;   /* time of the last compass sample */
    uint32_t next_ap_ms = 0;
    uint32_t last5_ms = sc->end_ms > 300000u ? sc->end_ms - 300000u : 0u;

    if (sc->csv) {
        fprintf(sc->csv, "time_s,mode,target_deg,heading_deg,rudder_deg,motor_cmd,alarm%s\n",
                sc->outboard ? ",estimate_deg" : "");
    }

    for (uint32_t t = 0; t <= sc->end_ms; t += PHYSICS_DT_MS) {
        float ts = (float)t * 0.001f;

        sim_hal_set_time_ms(t);
        sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);

        /* Scenario events */
        if (t == 0) sim_hal_press(HAL_BTN_STANDBY_AUTO);
        if (sc->turn_at_ms &&
            (t == sc->turn_at_ms || t == sc->turn_at_ms + 300u || t == sc->turn_at_ms + 600u))
            sim_hal_press(HAL_BTN_PLUS_10);
        bool compass_dead =
            (sc->fail_at_ms && t >= sc->fail_at_ms) ||
            (sc->dropout_at_ms && t >= sc->dropout_at_ms &&
             t < sc->dropout_at_ms + sc->dropout_len_ms);
        sim_hal_fail_heading(compass_dead);
        if (!compass_dead && t % 100u == 0u) last_fix_ms = t;   /* compass is 10 Hz */

        /* Autopilot at a fixed 20 Hz */
        if (t == next_ap_ms) {
            next_ap_ms += AP_PERIOD_MS;
            autopilot_step(&ap);

            float cmd = sim_hal_motor_command();
            bool on = cmd != 0.0f;
            if (t > 0 && on != (prev_cmd != 0.0f)) r.motor_switches++;
            if (on && (prev_cmd == 0.0f || (cmd > 0.0f) != (prev_cmd > 0.0f))) r.pulses++;
            prev_cmd = cmd;

            if (sc->csv) {
                fprintf(sc->csv, "%.2f,%s,%.1f,%.2f,%.2f,%.3f,%d",
                        ts, ap.mode == AP_MODE_AUTO ? "AUTO" : "STANDBY",
                        ap.target_heading, boat.heading_deg, rudder.angle_deg,
                        ap.motor_cmd, ap.alarm ? 1 : 0);
                if (sc->outboard) fprintf(sc->csv, ",%.2f", ap.est_angle);
                fprintf(sc->csv, "\n");
            }

            bool in_turn = sc->turn_at_ms && t >= sc->turn_at_ms &&
                           t < sc->turn_at_ms + TURN_WINDOW_MS;
            bool failed = sc->fail_at_ms && t >= sc->fail_at_ms;

            /* Course holding (true heading vs target), outside turn/failure */
            if (t >= 10000u && !in_turn && !failed &&
                ap.mode == AP_MODE_AUTO && t < sc->end_ms) {
                float e = fabsf(autopilot_heading_error(ap.target_heading, boat.heading_deg));
                hold_sq += (double)e * e; hold_n++;
                if (e > r.hold_max) r.hold_max = e;
                if (t >= last5_ms && e > r.hold_max_last5) r.hold_max_last5 = e;
            }
            /* Course change */
            if (in_turn) {
                float e = autopilot_heading_error(ap.target_heading, boat.heading_deg);
                float tt = ts - sc->turn_at_ms * 0.001f;
                if (r.reach_s < 0.0f && fabsf(e) <= 2.0f) r.reach_s = tt;
                if (r.reach_s >= 0.0f && -e > r.overshoot) r.overshoot = -e;
            }
            /* Outboard angle estimate vs truth */
            if (sc->outboard && ap.mode == AP_MODE_AUTO) {
                float ee = ap.est_angle - rudder.angle_deg;
                float ae = fabsf(ee);
                if (ae > r.est_err_max) r.est_err_max = ae;
                if (t < 300000u && ae > r.est_err_max_first5) r.est_err_max_first5 = ae;
                if (t >= last5_ms && ae > r.est_err_max_last5) r.est_err_max_last5 = ae;
                r.est_err_end = ee;
            }
            /* Stale compass: once the last fix is >= 0.5 s old the motor must be off */
            if (compass_dead && t - last_fix_ms >= AP_SENSOR_STALE_MS && on)
                r.motor_while_stale++;
            if (r.alarm_s < 0.0f && ap.alarm && !failed) r.alarm_s = ts;
            if (failed) {
                if (on) r.frozen_after_s = -1.0f;
                else if (r.frozen_after_s < 0.0f) r.frozen_after_s = ts - sc->fail_at_ms * 0.001f;
                if (r.standby_s < 0.0f && ap.mode == AP_MODE_STANDBY && ap.alarm)
                    r.standby_s = ts - sc->fail_at_ms * 0.001f;
            }
        }
        rudder_step(&rudder, sim_hal_motor_command(), PHYSICS_DT_MS * 0.001f);
        boat_step(&boat, rudder.angle_deg, ts, PHYSICS_DT_MS * 0.001f);
    }

    r.hold_rms = hold_n ? (float)sqrt(hold_sq / hold_n) : 0.0f;
    r.auto_at_end = (ap.mode == AP_MODE_AUTO && !ap.alarm);
    r.stall_s = rudder.stall_s;
    r.integrator_end = ap.integrator;
    return r;
}

/* ------------------------------------------------------------ big boat -- */

static int big_boat(int ss, uint32_t seed)
{
    FILE *csv = fopen("sim_output.csv", "w");
    if (!csv) {
        printf("Cannot open sim_output.csv for writing\n");
        return 1;
    }
    scenario_t main_sc = { (uint8_t)ss, seed, 90.0f, 300000u, 60000u, 240000u, 0u, 0u,
                           false, 0.0f, 0.0f, 0.0f, csv };
    results_t m = run(&main_sc);
    fclose(csv);

    /* Short dropout during the turn, while the motor is working hard */
    scenario_t drop_sc = { (uint8_t)ss, seed, 90.0f, 70000u, 60000u, 0u, 61000u, 1200u,
                           false, 0.0f, 0.0f, 0.0f, NULL };
    results_t d = run(&drop_sc);

    printf("\n=== Sea state %d, simulated waves %d deg (true boat heading) ===\n", ss, ss);
    printf("Motor on/off switches (300 s): %d\n", m.motor_switches);
    printf("Course hold (10-60 s, 100-240 s): RMS %.2f deg, max %.2f deg\n",
           m.hold_rms, m.hold_max);
    if (m.reach_s >= 0.0f)
        printf("Turn 90 -> 120: within 2 deg after %.1f s, overshoot %.2f deg\n",
               m.reach_s, m.overshoot);
    else
        printf("Turn 90 -> 120: NEVER reached within 2 deg\n");
    printf("Compass failure at 240 s: motor stopped after %.2f s, "
           "STANDBY + alarm after %.2f s\n", m.frozen_after_s, m.standby_s);
    printf("1.2 s compass dropout mid-turn: motor steps while stale = %d, "
           "afterwards %s\n", d.motor_while_stale,
           d.auto_at_end ? "back in AUTO, no alarm" : "NOT in AUTO / alarm");
    printf("Wrote sim_output.csv\n");

    bool ok = m.reach_s >= 0.0f && m.standby_s >= 0.0f &&
              m.motor_while_stale == 0 && d.motor_while_stale == 0 &&
              d.auto_at_end;
    return ok ? 0 : 2;
}

/* ------------------------------------------------------------ outboard -- */

typedef struct {
    const char *name;
    float speed_pos, speed_neg, gap;
} actuator_case_t;

static int outboard(uint32_t seed)
{
    const uint8_t ss = 3u;
    const actuator_case_t cases[] = {
        { "A correct: 3 deg/s, gap 2", 3.0f, 3.0f, 2.0f },
        { "B speed -50%: 1.5 deg/s",   1.5f, 1.5f, 2.0f },
        { "C speed +50%: 4.5 deg/s",   4.5f, 4.5f, 2.0f },
        { "D gap 0",                   3.0f, 3.0f, 0.0f },
        { "E gap 5",                   3.0f, 3.0f, 5.0f },
        { "F uneven: R 3.6 / L 2.4",   3.6f, 2.4f, 2.0f },
    };
    const int n = (int)(sizeof cases / sizeof cases[0]);
    int unsafe_total = 0;

    printf("\n=== Outboard (no angle feedback), sea state 3, waves 3 deg, seed %u ===\n",
           (unsigned)seed);
    printf("Autopilot config (all runs): turn_speed 3 deg/s, gap 2 deg, "
           "max 30 deg, min pulse 100 ms\n");
    printf("All angles in degrees, times in seconds. Case A gap 2 is also the 'true gap 2' row. F is an extra test "
           "(motor slower one way).\n\n");

    printf("5-min run: hold 90, +30 at 60 s, compass fails at 240 s\n");
    printf("%-27s %8s %8s %7s %8s %9s %8s %8s %s\n", "case", "hold max", "hold rms",
           "reach", "oversh.", "pulses/5m", "est err", "stall s", "fail->STBY");
    results_t main_r[8];
    for (int i = 0; i < n; i++) {
        FILE *csv = (i == 0) ? fopen("sim_outboard.csv", "w") : NULL;
        scenario_t sc = { ss, seed, 90.0f, 300000u, 60000u, 240000u, 0u, 0u,
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, csv };
        results_t r = run(&sc);
        if (csv) fclose(csv);
        main_r[i] = r;
        printf("%-27s %7.2f %7.2f %6.1fs %7.2f %9d %7.2f %8.1f %5.2fs\n",
               cases[i].name, r.hold_max, r.hold_rms, r.reach_s, r.overshoot,
               r.pulses, r.est_err_max, r.stall_s, r.standby_s);
    }

    printf("\nTurn across north: hold 350, +30 at 60 s -> 20\n");
    printf("%-27s %7s %8s %8s %8s\n", "case", "reach", "oversh.", "est err", "stall s");
    results_t north_r[8];
    for (int i = 0; i < n; i++) {
        scenario_t sc = { ss, seed, 350.0f, 120000u, 60000u, 0u, 0u, 0u,
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, NULL };
        results_t r = run(&sc);
        north_r[i] = r;
        printf("%-27s %6.1fs %7.2f %7.2f %8.1f\n",
               cases[i].name, r.reach_s, r.overshoot, r.est_err_max, r.stall_s);
    }

    printf("\n10-minute straight run on 90 (drift check)\n");
    printf("%-27s %12s %12s %10s %13s %9s %10s %s\n", "case", "est err 0-5m",
           "est err 5-10m", "est at end", "hold max 5-10m", "pulses/5m", "integrator", "alarm");
    results_t long_r[8];
    for (int i = 0; i < n; i++) {
        scenario_t sc = { ss, seed, 90.0f, 600000u, 0u, 0u, 0u, 0u,
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, NULL };
        results_t r = run(&sc);
        long_r[i] = r;
        printf("%-27s %11.2f %12.2f %+9.2f %13.2f %9.0f %+9.2f %s\n",
               cases[i].name, r.est_err_max_first5, r.est_err_max_last5, r.est_err_end,
               r.hold_max_last5, r.pulses / 2.0f, r.integrator_end,
               r.alarm_s < 0.0f ? "none" : "ON");
    }

    printf("\nSafety checks and flags\n");
    for (int i = 0; i < n; i++) {
        scenario_t sc = { ss, seed, 90.0f, 70000u, 60000u, 0u, 61000u, 1200u,
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, NULL };
        results_t d = run(&sc);
        const results_t *m = &main_r[i], *no = &north_r[i], *lo = &long_r[i];

        char flags[512] = "";
        int unsafe = 0;
        float stall = m->stall_s + no->stall_s + lo->stall_s;
        if (m->standby_s < 0.0f) { strcat(flags, " UNSAFE: no STANDBY on compass failure;"); unsafe++; }
        if (m->motor_while_stale || d.motor_while_stale) { strcat(flags, " UNSAFE: motor ran on stale compass;"); unsafe++; }
        if (!d.auto_at_end) { strcat(flags, " UNSAFE: dropout did not recover;"); unsafe++; }
        if (stall > 0.0f) { strcat(flags, " UNSAFE: motor pushed against the end stop;"); unsafe++; }
        if (m->reach_s < 0.0f || no->reach_s < 0.0f) { strcat(flags, " UNSAFE: course change not reached;"); unsafe++; }
        if (m->hold_max > 10.0f || lo->hold_max > 10.0f) { strcat(flags, " UNSAFE: course error > 10 deg;"); unsafe++; }
        else if (m->hold_max > 5.0f || lo->hold_max > 5.0f) strcat(flags, " warn: course error > 5 deg;");
        if (m->overshoot > 10.0f || no->overshoot > 10.0f) { strcat(flags, " UNSAFE: overshoot > 10 deg;"); unsafe++; }
        else if (m->overshoot > 5.0f || no->overshoot > 5.0f) strcat(flags, " warn: overshoot > 5 deg;");
        if (lo->est_err_max > 10.0f) strcat(flags, " warn: estimate off by > 10 deg;");
        if (lo->est_err_max_last5 > lo->est_err_max_first5 + 2.0f) strcat(flags, " warn: estimate drifts;");

        printf("%-27s dropout: motor while stale %d, %s.%s\n", cases[i].name,
               m->motor_while_stale + d.motor_while_stale,
               d.auto_at_end ? "AUTO resumes" : "NO RECOVERY",
               flags[0] ? flags : " OK");
        unsafe_total += unsafe;
    }

    /* Steering failure: motor runs but the outboard never moves. Without
     * angle feedback only the off-course alarm can notice this. */
    scenario_t dead = { ss, seed, 90.0f, 180000u, 0u, 0u, 0u, 0u,
                        true, 0.0f, 0.0f, 2.0f, NULL };
    results_t dr = run(&dead);
    if (dr.alarm_s >= 0.0f) {
        printf("Steering failure (outboard never moves): off-course alarm at %.1f s\n",
               dr.alarm_s);
    } else {
        printf("Steering failure (outboard never moves): UNSAFE, no alarm in 180 s\n");
        unsafe_total++;
    }
    printf("\nWrote sim_outboard.csv (case A, 5-min run)\n");
    return unsafe_total ? 2 : 0;
}

int main(int argc, char **argv)
{
    if (self_test() != 0) return 1;

    if (argc > 1 && strcmp(argv[1], "outboard") == 0) {
        uint32_t seed = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 2024u;
        return outboard(seed);
    }

    int ss = (argc > 1) ? atoi(argv[1]) : (int)AP_SEA_STATE_DEFAULT;
    uint32_t seed = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 2024u;
    if (ss < (int)AP_SEA_STATE_MIN || ss > (int)AP_SEA_STATE_MAX) {
        printf("Sea state must be %u..%u (or 'outboard')\n", AP_SEA_STATE_MIN, AP_SEA_STATE_MAX);
        return 1;
    }
    return big_boat(ss, seed);
}
