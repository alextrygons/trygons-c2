/*
 * sim_run.h - One simulated autopilot run (shared by sim_main.c and
 * sim_train.c).
 */
#ifndef SIM_RUN_H
#define SIM_RUN_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "autopilot.h"

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

    /* Optional (0 / false / NULL = default) */
    float    boat_K;            /* true turn gain, deg/s yaw per deg */
    float    boat_T;            /* true response time constant, s */
    bool     set_current;
    float    current_deg;       /* steady push, equivalent outboard deg */
    float    stop_right;        /* true end stops, deg */
    float    stop_left;
    const ap_config_t *cfg;     /* autopilot config to use */
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

results_t run(const scenario_t *sc);

/* Training tests (sim_train.c). Returns 0 if nothing unsafe. */
int training_tests(uint32_t seed);
int training_trace(uint32_t seed, int set, int ss);

/* PASS/FAIL line for the "all" run (sim_main.c). */
void test_report(const char *name, bool ok, const char *detail);
void test_na(const char *name, const char *why);

/* Small-boat tests, all phases (sim_sb.c). */
void sb_tests(void);
void sb_usb_tests(void);
void sb_gateway_tests(void);      /* gateway link, HELM, REMOTE, SAVE requests */
void sb_nv_tests(void);           /* DF: data flash / non-volatile storage */
void sb_battery_tests(void);      /* BV: supply voltage of the 12 V bank, LOW BATTERY */
void sb_relay_tests(void);        /* RB: RS485 Modbus relay board (tc_modbus.c + relays.c) */
void sb_engine_tests(void);       /* ER: engine controls + remote restart (engine.c, section N) */
void sb_dp_tests(void);           /* DP: thrusters part 1 (thruster.c, CPU1, safe outputs; section P step 9) */
void sb_generator_tests(void);    /* GN: generator mode (throttle in neutral to charge the bank) */
void sb_slam_tests(void);         /* only the slamming tests (section H) */
void sb_set_bridge(bool on);      /* all tests through the USB joystick bridge */

/* Python log round-trip test data (sim_sb.c) */
int sb_logtest(const char *dir);

/* PC timing reference (sim_sb.c) */
int sb_timing_report(void);

/* sim_server.c: simulated boat behind TCP for the ground station */
int sim_server(int port, double speed, bool usb, uint32_t gw_baud);   /* gateway UART on port + 2 */
void sim_server_nojoy(bool on);     /* before sim_server: start without the onboard joystick (UNCREWED) */

#endif /* SIM_RUN_H */
