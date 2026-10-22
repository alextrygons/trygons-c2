/*
 * sim_main.c - Runs the autopilot against the boat model.
 *
 * Usage:
 *   autopilot_sim [sea_state 1..5 [wave_seed]]   big boat, rudder feedback
 *                                                (defaults 3, 2024)
 *   autopilot_sim all                            every test, PASS/FAIL list
 *   autopilot_sim train [seed]                   outboard TRAINING tests
 *   autopilot_sim train SEED BOAT SS             trace one training (BOAT 1..5, SS 1..2)
 *   autopilot_sim timing                         PC timing reference of each part
 *   autopilot_sim logtest DIR                    log + reference values (round-trip test)
 *   autopilot_sim server [port [speed]]          simulated boat for the ground
 *                                                station (TCP 5760, speed 1)
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
#include "sim_run.h"
#include "sim_world.h"


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

results_t run(const scenario_t *sc)
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
    if (sc->boat_K > 0.0f) boat.K_override = sc->boat_K;
    if (sc->boat_T > 0.0f) boat.T_override = sc->boat_T;
    if (sc->set_current)   boat.current_deg = sc->current_deg;
    if (sc->outboard)
        rudder_init_outboard(&rudder, sc->true_speed_pos, sc->true_speed_neg, sc->true_gap);
    else
        rudder_init(&rudder);
    if (sc->stop_right > 0.0f) rudder.max_angle_deg = sc->stop_right;
    if (sc->stop_left > 0.0f)  rudder.max_angle_neg_deg = sc->stop_left;

    sim_hal_set_time_ms(0);
    sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(!sc->outboard);
    sim_hal_set_gyro(false, 0.0f, 0.0f);
    hal_init();
    if (sc->outboard) autopilot_default_config_outboard(&cfg);
    else              autopilot_default_config(&cfg);
    if (sc->cfg) cfg = *sc->cfg;
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
                           false, 0.0f, 0.0f, 0.0f, csv,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
    results_t m = run(&main_sc);
    fclose(csv);

    /* Short dropout during the turn, while the motor is working hard */
    scenario_t drop_sc = { (uint8_t)ss, seed, 90.0f, 70000u, 60000u, 0u, 61000u, 1200u,
                           false, 0.0f, 0.0f, 0.0f, NULL,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
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
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, csv,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
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
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, NULL,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
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
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, NULL,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
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
                          true, cases[i].speed_pos, cases[i].speed_neg, cases[i].gap, NULL,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
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
                        true, 0.0f, 0.0f, 2.0f, NULL,
                          0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
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

/* ---------------------------------------------------------------- all -- */

static int g_pass, g_fail, g_na;

void test_report(const char *name, bool ok, const char *detail)
{
    printf("[%s] %s%s%s\n", ok ? "PASS" : "FAIL", name,
           (detail && detail[0]) ? " - " : "", detail ? detail : "");
    if (ok) g_pass++; else g_fail++;
}

/* Test that does not apply to this configuration (shown, not counted). */
void test_na(const char *name, const char *why)
{
    printf("[N/A ] %s - %s\n", name, why);
    g_na++;
}

/* Every test of every phase. Exit code 0 only if all pass. */
static int run_all(void)
{
    char d[200];
    g_pass = g_fail = g_na = 0;

    /* Big boat: must stay exactly as before (regression) */
    scenario_t m_sc = { 3, 2024u, 90.0f, 300000u, 60000u, 240000u, 0u, 0u,
                        false, 0.0f, 0.0f, 0.0f, NULL,
                        0.0f, 0.0f, false, 0.0f, 0.0f, 0.0f, NULL };
    results_t m = run(&m_sc);
    scenario_t d_sc = m_sc;
    d_sc.end_ms = 70000u; d_sc.fail_at_ms = 0u; d_sc.dropout_at_ms = 61000u; d_sc.dropout_len_ms = 1200u;
    results_t dr = run(&d_sc);
    snprintf(d, sizeof d, "switches %d, hold max %.2f, overshoot %.2f", m.motor_switches, m.hold_max, m.overshoot);
    test_report("big boat: results unchanged (50 switches, 3.77 deg)",
                m.motor_switches == 50 && fabsf(m.hold_max - 3.77f) < 0.01f, d);
    snprintf(d, sizeof d, "STANDBY after %.2f s, motor on while stale %d", m.standby_s, m.motor_while_stale);
    test_report("big boat: compass failure -> STANDBY + alarm, motor off",
                m.standby_s > 0.0f && m.standby_s <= 2.1f && m.motor_while_stale == 0, d);
    test_report("big boat: compass dropout -> rudder frozen, AUTO resumes",
                dr.motor_while_stale == 0 && dr.auto_at_end, "");

    /* Outboard table and TRAINING (earlier work) */
    test_report("outboard: no unsafe case in the test table", outboard(2024u) == 0, "");
    test_report("training: all trainings OK, all safety tests OK", training_tests(2024u) == 0, "");

    /* Small-boat phases */
    sb_tests();

    printf("\nTOTAL: %d passed, %d failed", g_pass, g_fail);
    if (g_na) printf(", %d not applicable", g_na);
    printf("\n");
    return g_fail ? 2 : 0;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);     /* no lost output if a test crashes */
    if (self_test() != 0) return 1;

    if (argc > 1 && strcmp(argv[1], "all") == 0) {
        if (argc > 2 && strcmp(argv[2], "--bridge") == 0) {
            sb_set_bridge(true);
            printf("JOYSTICK SOURCE: USB bridge (all tests through bridge frames)\n");
        }
        return run_all();
    }

    if (argc > 1 && strcmp(argv[1], "usb") == 0) { sb_usb_tests(); return 0; }
    if (argc > 1 && strcmp(argv[1], "gw") == 0) { sb_gateway_tests(); return 0; }
    if (argc > 1 && strcmp(argv[1], "slam") == 0) { sb_slam_tests(); return 0; }
    if (argc > 1 && strcmp(argv[1], "timing") == 0) return sb_timing_report();
    if (argc > 2 && strcmp(argv[1], "logtest") == 0) return sb_logtest(argv[2]);

    if (argc > 1 && strcmp(argv[1], "server") == 0) {
        int port = (argc > 2) ? atoi(argv[2]) : 5760;
        double speed = (argc > 3) ? atof(argv[3]) : 1.0;
        if (port <= 0 || port > 65535 || !(speed > 0.0 && speed <= 50.0)) {
            printf("usage: autopilot_sim server [port 1..65535] [speed 0..50] [usb] [gwbaud=N] [origin=lat,lon]\n"
                   "       (the gateway UART for the Raspberry Pi software is on port + 2)\n");
            return 1;
        }
        bool usb = false;                       /* "usb": start with the USB joystick map */
        uint32_t gw_baud = 921600u;             /* "gwbaud=N": slower gateway UART (tests) */
        for (int i = 4; i < argc; i++) {
            if (strcmp(argv[i], "usb") == 0) usb = true;
            else if (strncmp(argv[i], "gwbaud=", 7) == 0) gw_baud = (uint32_t)strtoul(argv[i] + 7, NULL, 10);
            else if (strncmp(argv[i], "origin=", 7) == 0) {     /* "origin=37.75,23.70": start position */
                double la = 0.0, lo = 0.0;
                if (sscanf(argv[i] + 7, "%lf,%lf", &la, &lo) == 2 && la > -80.0 && la < 80.0 &&
                    lo >= -180.0 && lo <= 180.0) {
                    world_set_origin(la, lo);
                } else {
                    printf("origin=lat,lon: bad value \"%s\"\n", argv[i] + 7);
                    return 1;
                }
            }
        }
        return sim_server(port, speed, usb, gw_baud);
    }

    if (argc > 1 && strcmp(argv[1], "train") == 0) {
        uint32_t seed = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 2024u;
        if (argc > 4) return training_trace(seed, atoi(argv[3]), atoi(argv[4]));
        return training_tests(seed);
    }
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
