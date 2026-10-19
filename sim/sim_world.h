/*
 * sim_world.h - Simulated small boat for the small-boat tests: engine and
 * speed (forward / reverse), yaw (speed dependent), position with current,
 * waves, outboard steering actuator, gear servo, GPS (1 Hz, noise).
 *
 * It also watches the safety rules from the outside (gear changes under
 * load, F <-> R without neutral pause, throttle limits) so the tests can
 * check them.
 */
#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include <stdbool.h>
#include <stdint.h>

#include "boat_model.h"
#include "gear.h"

typedef struct {
    /* environment / boat (set before or during the run) */
    float sea_state;            /* wave size, equivalent outboard deg */
    float current_e, current_n; /* water current, m/s (east, north) */
    float trim_deg;             /* steady yaw push, equivalent outboard deg */
    float K_override, T_override; /* >0: fixed boat turn gain / time constant */
    float vmax_fwd_kn;          /* full throttle forward (20) */
    float vmax_rev_kn;          /* full throttle reverse (8) */
    float idle_fwd_kn;          /* in gear at idle (1.5) */
    float speed_tau_s;          /* speed response (6 s) */
    bool  gps_fail;             /* no GPS messages */
    float gps_pos_noise_m;      /* 1.5 */
    bool  neutral_switch;       /* a neutral switch is fitted */
    bool  servo_stuck;          /* gear servo does not move any more */
    bool  current_sensor;       /* steering motor current sense fitted */
    bool  limit_switches;       /* steering end-stop switches fitted */

    /* state */
    float heading_deg, yaw_dps, wave_filt;
    float speed_kn;             /* through water, + forward */
    double north_m, east_m;
    rudder_actuator_t rud;
    float gear_pos_us;          /* gear servo position (moves at finite speed) */
    gear_t engaged;             /* gear the gearbox is in */
    uint32_t neutral_since_ms;
    float throttle;             /* servo % */
    uint32_t rng, gps_rng, cur_rng;

    /* safety watch */
    int   shift_under_load;     /* gear engaged / left with throttle open */
    int   fr_without_neutral;   /* F <-> R with < 0.5 s in neutral */
    float max_thr_fwd, max_thr_rev;
    float max_thr_rate;         /* fastest throttle change seen, %/s */
    float last_throttle;
    uint32_t t_ms;
} world_t;

void world_init(world_t *w, uint32_t seed, float heading_deg, float speed_kn);

/* Advance by dt using the commands currently in the sim HAL. */
void world_step(world_t *w, float dt_s);

/* Give the sensors their values for this time (compass, gyro, GPS). */
void world_publish(world_t *w, uint32_t t_ms);

/* Ground speed (kn) and course (deg) incl. current. */
float world_sog_kn(const world_t *w);
float world_cog_deg(const world_t *w);

/* Position as lat/lon (origin 60 N, 25 E). */
void world_latlon(const world_t *w, double *lat, double *lon);
void world_from_latlon(double lat, double lon, double *north_m, double *east_m);

#endif /* SIM_WORLD_H */
