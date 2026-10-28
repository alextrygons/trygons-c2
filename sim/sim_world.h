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
    int8_t neutral_sw_force;    /* tests: 0 the switch follows the gearbox, 1 stuck open, 2 stuck closed */
    bool  servo_stuck;          /* gear servo does not move any more */
    bool  current_sensor;       /* steering motor current sense fitted */
    bool  limit_switches;       /* steering end-stop switches fitted */
    bool  speed_sensor;         /* paddlewheel fitted */
    float speed_sensor_scale;   /* its calibration error (1.02 = reads 2 % high) */
    /* GPS faults: offset added to the reported position; a drag grows the
     * offset at gps_drag_n/e m/s up to gps_drag_max_m (spoofer) */
    double gps_off_n, gps_off_e;
    float gps_drag_n, gps_drag_e, gps_drag_max_m;
    bool  gps_dragging;
    bool  gps_flag;             /* receiver spoof/jam flag */
    /* slamming: impacts (Poisson) above ~8 kn, more and harder with speed
     * and sea state; slam_scale 0 = smooth water */
    float slam_scale;           /* 1.0 */
    float slam_extra_g;         /* tests: add this to every impact */
    uint32_t slam_rng;
    float slam_left[4];         /* impact pulse still to play out (samples) */
    uint32_t slam_impacts;      /* impacts of 1 g or more made (truth) */
    float slam_dead;            /* s until the next impact can come */
    float slam_last_g;
    /* FIRST TEST: the engine is driven by hand (throttle / gear servos not connected) */
    bool  hand_drive;
    gear_t hand_gear;
    float hand_throttle;        /* % */

    /* DP thrusters (CLAUDE.md P): full thrust (100 %) gives, at rest, ... */
    float bow_sway_ms;          /* ... this sideways speed from the bow tunnel thruster (0.5 m/s) */
    float bow_yaw_dps;          /* ... and this yaw rate (it pushes at the bow: 6 deg/s) */
    float stern_kn;             /* ... this speed ahead / astern from the transom motor (2.5 kn) */
    float thr_tau_s;            /* their response time (3 s) */
    float steer_half_deg;       /* DroneCAN steering position servo: outboard degrees at servo +1 (30) */

    /* state */
    float heading_deg, yaw_dps, wave_filt;
    float sway_ms;              /* sideways speed through water, + = to starboard */
    float thr_yaw_dps;          /* yaw rate part from the bow thruster */
    float speed_kn;             /* through water, + forward */
    double north_m, east_m;
    rudder_actuator_t rud;
    float gear_pos_us;          /* gear servo position (moves at finite speed) */
    gear_t engaged;             /* gear the gearbox is in */
    uint32_t neutral_since_ms;
    float throttle;             /* servo % */
    uint32_t rng, gps_rng, cur_rng, pw_rng;

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
/* Start position (default 60 N 25 E); call before the server starts. */
void world_set_origin(double lat, double lon);

#endif /* SIM_WORLD_H */
