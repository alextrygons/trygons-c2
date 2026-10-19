/*
 * boat_model.h - Simple boat + rudder actuator model for PC simulation.
 *
 * Boat: first-order "Nomoto" steering model
 *     T * d(yaw_rate)/dt + yaw_rate = K * (rudder + disturbance)
 * with K and T depending on boat speed. Disturbance = waves + steady
 * current, expressed as "equivalent rudder degrees".
 */
#ifndef BOAT_MODEL_H
#define BOAT_MODEL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float heading_deg;      /* 0..360 */
    float yaw_rate_dps;     /* deg/s, positive = turning to starboard */
    float speed_kn;
    float current_deg;      /* steady disturbance, equivalent rudder deg */
    float wave_amp_deg;     /* wave disturbance size, equivalent rudder deg */
    float wave_filt;        /* internal: filtered random wave state */
    uint32_t rng;           /* internal: random generator state */
    float K_override;       /* >0: use this turn gain (deg/s per deg) instead of the speed model */
    float T_override;       /* >0: use this response time constant, s */
} boat_t;

/* Rudder / outboard actuator. These are the TRUE values; the autopilot
 * only knows its own config (and, for the outboard, gets no angle). */
typedef struct {
    float angle_deg;        /* true rudder / outboard angle */
    float drive_deg;        /* motor-side position (differs by the backlash) */
    float rate_pos_dps;     /* true speed at full command, turning right */
    float rate_neg_dps;     /* true speed at full command, turning left */
    float gap_deg;          /* true backlash (0 = none) */
    float max_angle_deg;    /* mechanical stop, right */
    float max_angle_neg_deg;/* mechanical stop, left (positive number) */
    float stall_s;          /* time the motor pushed against a mechanical stop */
    float stall_run_s;      /* current continuous push */
    float stall_max_s;      /* longest continuous push */
} rudder_actuator_t;

void boat_init(boat_t *b, float heading_deg, float speed_kn, uint32_t seed);
void boat_step(boat_t *b, float rudder_deg, float t_s, float dt_s);

/* Small random number helpers (deterministic, so runs are repeatable). */
float boat_rand_gauss(uint32_t *state);

void rudder_init(rudder_actuator_t *r);          /* big boat: 6 deg/s, no gap */
void rudder_init_outboard(rudder_actuator_t *r, float speed_pos_dps,
                          float speed_neg_dps, float gap_deg);
void rudder_step(rudder_actuator_t *r, float motor_cmd, float dt_s);

#endif /* BOAT_MODEL_H */
