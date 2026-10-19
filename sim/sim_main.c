/*
 * sim_main.c - Runs the autopilot against the boat model.
 *
 * Usage:  autopilot_sim [sea_state 1..5 [wave_seed]]   (defaults 3, 2024)
 *
 * Main scenario (5 minutes, written to sim_output.csv):
 *   t =   0 s  boat on 90 deg, press Standby/Auto -> AUTO, holds 90
 *   t =  60 s  press +10 three times -> new course 120
 *   t = 240 s  compass stops sending data
 *              -> rudder frozen at ~240.4 s, STANDBY + alarm at ~242 s
 *   t = 300 s  end
 *
 * Extra test (no CSV): compass drops out for 1.2 s in the middle of the
 * turn while the motor is busy -> motor must stop, then AUTO continues.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "autopilot.h"
#include "boat_model.h"
#include "hal_sim.h"

#define PHYSICS_DT_MS   10u         /* boat model runs at 100 Hz */
#define TURN_AT_MS      60000u
#define BOAT_SPEED_KN   6.0f

typedef struct {
    uint8_t  sea_state;
    uint32_t seed;              /* wave pattern */
    uint32_t end_ms;
    uint32_t fail_at_ms;        /* compass dies for good (0 = never) */
    uint32_t dropout_at_ms;     /* short compass dropout (0 = none)  */
    uint32_t dropout_len_ms;
    FILE    *csv;               /* NULL = no CSV */
} scenario_t;

typedef struct {
    int   motor_switches;       /* motor on<->off changes, whole run */
    float hold_max, hold_rms;   /* course error while holding (10-60, 100-240 s) */
    float reach_s, overshoot;   /* turn 90 -> 120 */
    float frozen_after_s;       /* fail -> motor stopped for good */
    float standby_s;            /* fail -> STANDBY + alarm */
    int   motor_while_stale;    /* steps with motor on while compass stale */
    bool  auto_after_dropout;   /* still AUTO, no alarm after short dropout */
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
    results_t r = { 0, 0.0f, 0.0f, -1.0f, 0.0f, -1.0f, -1.0f, 0, false };
    boat_t boat;
    rudder_actuator_t rudder;
    autopilot_t ap;
    ap_config_t cfg;

    boat_init(&boat, 90.0f, BOAT_SPEED_KN, sc->seed);
    boat.wave_amp_deg = (float)sc->sea_state;   /* simulated sea matches the setting */
    rudder_init(&rudder);

    sim_hal_set_time_ms(0);
    sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);
    sim_hal_fail_heading(false);
    hal_init();
    autopilot_default_config(&cfg);
    cfg.sea_state = sc->sea_state;
    autopilot_init(&ap, &cfg);

    double hold_sq = 0.0;
    int hold_n = 0;
    bool prev_on = false;
    uint32_t last_fix_ms = 0;   /* time of the last compass sample */
    uint32_t next_ap_ms = 0;

    if (sc->csv)
        fprintf(sc->csv, "time_s,mode,target_deg,heading_deg,rudder_deg,motor_cmd,alarm\n");

    for (uint32_t t = 0; t <= sc->end_ms; t += PHYSICS_DT_MS) {
        float ts = (float)t * 0.001f;

        sim_hal_set_time_ms(t);
        sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);

        /* Scenario events */
        if (t == 0)                    sim_hal_press(HAL_BTN_STANDBY_AUTO);
        if (t == TURN_AT_MS)           sim_hal_press(HAL_BTN_PLUS_10);
        if (t == TURN_AT_MS + 300u)    sim_hal_press(HAL_BTN_PLUS_10);
        if (t == TURN_AT_MS + 600u)    sim_hal_press(HAL_BTN_PLUS_10);
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

            bool on = sim_hal_motor_command() != 0.0f;
            if (t > 0 && on != prev_on) r.motor_switches++;
            prev_on = on;

            if (sc->csv)
                fprintf(sc->csv, "%.2f,%s,%.1f,%.2f,%.2f,%.3f,%d\n",
                        ts, ap.mode == AP_MODE_AUTO ? "AUTO" : "STANDBY",
                        ap.target_heading, boat.heading_deg, rudder.angle_deg,
                        ap.motor_cmd, ap.alarm ? 1 : 0);

            /* Course holding (true heading vs target), outside the turn */
            if ((t >= 10000u && t < TURN_AT_MS) || (t >= 100000u && t < 240000u)) {
                float e = fabsf(autopilot_heading_error(ap.target_heading, boat.heading_deg));
                if (ap.mode == AP_MODE_AUTO && t < sc->end_ms) {
                    hold_sq += (double)e * e; hold_n++;
                    if (e > r.hold_max) r.hold_max = e;
                }
            }
            /* Turn to 120 */
            if (t >= TURN_AT_MS && t < 100000u) {
                float e = autopilot_heading_error(ap.target_heading, boat.heading_deg);
                float tt = ts - TURN_AT_MS * 0.001f;
                if (r.reach_s < 0.0f && fabsf(e) <= 2.0f) r.reach_s = tt;
                if (r.reach_s >= 0.0f && -e > r.overshoot) r.overshoot = -e;
            }
            /* Stale compass: once the last fix is >= 0.5 s old the motor must be off */
            if (compass_dead && t - last_fix_ms >= AP_SENSOR_STALE_MS && on)
                r.motor_while_stale++;
            if (sc->fail_at_ms && t >= sc->fail_at_ms) {
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
    r.auto_after_dropout = (ap.mode == AP_MODE_AUTO && !ap.alarm);
    return r;
}

int main(int argc, char **argv)
{
    int ss = (argc > 1) ? atoi(argv[1]) : (int)AP_SEA_STATE_DEFAULT;
    uint32_t seed = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 10) : 2024u;
    if (ss < (int)AP_SEA_STATE_MIN || ss > (int)AP_SEA_STATE_MAX) {
        printf("Sea state must be %u..%u\n", AP_SEA_STATE_MIN, AP_SEA_STATE_MAX);
        return 1;
    }
    if (self_test() != 0) return 1;

    /* Main 5-minute scenario */
    FILE *csv = fopen("sim_output.csv", "w");
    if (!csv) {
        printf("Cannot open sim_output.csv for writing\n");
        return 1;
    }
    scenario_t main_sc = { (uint8_t)ss, seed, 300000u, 240000u, 0u, 0u, csv };
    results_t m = run(&main_sc);
    fclose(csv);

    /* Short dropout during the turn, while the motor is working hard */
    scenario_t drop_sc = { (uint8_t)ss, seed, 70000u, 0u, 61000u, 1200u, NULL };
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
           d.auto_after_dropout ? "back in AUTO, no alarm" : "NOT in AUTO / alarm");
    printf("Wrote sim_output.csv\n");

    bool ok = m.reach_s >= 0.0f && m.standby_s >= 0.0f &&
              m.motor_while_stale == 0 && d.motor_while_stale == 0 &&
              d.auto_after_dropout;
    return ok ? 0 : 2;
}
