/*
 * sim_main.c - Runs the autopilot against the boat model for 5 minutes.
 *
 * Scenario:
 *   t =   0 s  boat on 90 deg, press Standby/Auto -> AUTO, holds 90
 *   t =  60 s  press +10 three times -> new course 120
 *   t = 240 s  compass stops sending data -> expect STANDBY + alarm at ~242 s
 *   t = 300 s  end
 *
 * Output: sim_output.csv (in the current folder) + a summary on screen.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "autopilot.h"
#include "boat_model.h"
#include "hal_sim.h"

#define SIM_END_MS      300000u
#define PHYSICS_DT_MS   10u         /* boat model runs at 100 Hz */
#define FAIL_AT_MS      240000u
#define TURN_AT_MS      60000u
#define BOAT_SPEED_KN   6.0f

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

int main(void)
{
    if (self_test() != 0) return 1;

    FILE *csv = fopen("sim_output.csv", "w");
    if (!csv) {
        printf("Cannot open sim_output.csv for writing\n");
        return 1;
    }
    fprintf(csv, "time_s,mode,target_deg,heading_deg,rudder_deg,motor_cmd,alarm\n");

    boat_t boat;
    rudder_actuator_t rudder;
    autopilot_t ap;
    ap_config_t cfg;

    boat_init(&boat, 90.0f, BOAT_SPEED_KN, 2024u);
    rudder_init(&rudder);

    sim_hal_set_time_ms(0);
    sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);
    hal_init();
    autopilot_default_config(&cfg);
    autopilot_init(&ap, &cfg);

    /* Measurements for the summary */
    double hold_sq = 0.0; int hold_n = 0; float hold_max = 0.0f;
    float reach_s = -1.0f, settle_s = -1.0f, overshoot = 0.0f;
    float standby_s = -1.0f;
    int   motor_on_after_standby = 0;
    const float new_course = 120.0f, settle_band = 3.0f;

    uint32_t next_ap_ms = 0;
    for (uint32_t t = 0; t <= SIM_END_MS; t += PHYSICS_DT_MS) {
        float ts = (float)t * 0.001f;

        sim_hal_set_time_ms(t);
        sim_hal_set_truth(boat.heading_deg, rudder.angle_deg);

        /* Scenario events */
        if (t == 0)                    sim_hal_press(HAL_BTN_STANDBY_AUTO);
        if (t == TURN_AT_MS)           sim_hal_press(HAL_BTN_PLUS_10);
        if (t == TURN_AT_MS + 300u)    sim_hal_press(HAL_BTN_PLUS_10);
        if (t == TURN_AT_MS + 600u)    sim_hal_press(HAL_BTN_PLUS_10);
        if (t == FAIL_AT_MS)           sim_hal_fail_heading(true);

        /* Autopilot at a fixed 20 Hz */
        if (t == next_ap_ms) {
            next_ap_ms += AP_PERIOD_MS;
            autopilot_step(&ap);

            fprintf(csv, "%.2f,%s,%.1f,%.2f,%.2f,%.3f,%d\n",
                    ts, ap.mode == AP_MODE_AUTO ? "AUTO" : "STANDBY",
                    ap.target_heading, boat.heading_deg, rudder.angle_deg,
                    ap.motor_cmd, ap.alarm ? 1 : 0);

            /* Course holding quality before the turn (skip first 10 s) */
            if (t >= 10000u && t < TURN_AT_MS) {
                float e = autopilot_heading_error(90.0f, boat.heading_deg);
                hold_sq += (double)e * e; hold_n++;
                if (fabsf(e) > hold_max) hold_max = fabsf(e);
            }
            /* Turn to 120 deg */
            if (t >= TURN_AT_MS && t < FAIL_AT_MS) {
                float e = autopilot_heading_error(new_course, boat.heading_deg);
                float tt = ts - TURN_AT_MS * 0.001f;
                if (reach_s < 0.0f && fabsf(e) <= 2.0f) reach_s = tt;
                if (reach_s >= 0.0f && -e > overshoot) overshoot = -e;
                if (fabsf(e) > settle_band) settle_s = -1.0f;
                else if (settle_s < 0.0f) settle_s = tt;
            }
            /* Sensor failure */
            if (t >= FAIL_AT_MS) {
                if (standby_s < 0.0f && ap.mode == AP_MODE_STANDBY && ap.alarm)
                    standby_s = ts - FAIL_AT_MS * 0.001f;
                if (standby_s >= 0.0f && sim_hal_motor_command() != 0.0f)
                    motor_on_after_standby++;
            }
        }

        rudder_step(&rudder, sim_hal_motor_command(), PHYSICS_DT_MS * 0.001f);
        boat_step(&boat, rudder.angle_deg, ts, PHYSICS_DT_MS * 0.001f);
    }
    fclose(csv);

    printf("\n=== Summary (true boat heading) ===\n");
    printf("Course hold on 90 deg (10-60 s): RMS error %.2f deg, max %.2f deg\n",
           hold_n ? sqrt(hold_sq / hold_n) : 0.0, hold_max);
    if (reach_s >= 0.0f)
        printf("Turn 90 -> 120: within 2 deg after %.1f s\n", reach_s);
    else
        printf("Turn 90 -> 120: NEVER reached within 2 deg\n");
    printf("Overshoot past 120: %.2f deg\n", overshoot);
    if (settle_s >= 0.0f)
        printf("Settled (stays within +/-%.0f deg) after %.1f s\n", settle_band, settle_s);
    else
        printf("Did not settle within +/-%.0f deg\n", settle_band);
    if (standby_s >= 0.0f)
        printf("Sensor failure at 240 s: STANDBY + alarm after %.2f s, "
               "motor commands after standby: %d\n", standby_s, motor_on_after_standby);
    else
        printf("Sensor failure at 240 s: STANDBY NOT triggered!\n");
    printf("Final mode: %s, alarm %s\n",
           ap.mode == AP_MODE_AUTO ? "AUTO" : "STANDBY", ap.alarm ? "ON" : "off");
    printf("Wrote sim_output.csv\n");

    return (standby_s >= 0.0f && reach_s >= 0.0f) ? 0 : 2;
}
