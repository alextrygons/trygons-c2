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

#include <stdint.h>

typedef struct {
    float heading_deg;      /* 0..360 */
    float yaw_rate_dps;     /* deg/s, positive = turning to starboard */
    float speed_kn;
    float current_deg;      /* steady disturbance, equivalent rudder deg */
    float wave_amp_deg;     /* wave disturbance size, equivalent rudder deg */
    float wave_filt;        /* internal: filtered random wave state */
    uint32_t rng;           /* internal: random generator state */
} boat_t;

typedef struct {
    float angle_deg;        /* actual rudder angle */
    float max_rate_dps;     /* rudder speed at full motor command */
    float max_angle_deg;    /* mechanical stop */
} rudder_actuator_t;

void boat_init(boat_t *b, float heading_deg, float speed_kn, uint32_t seed);
void boat_step(boat_t *b, float rudder_deg, float t_s, float dt_s);

/* Small random number helpers (deterministic, so runs are repeatable). */
float boat_rand_gauss(uint32_t *state);

void rudder_init(rudder_actuator_t *r);
void rudder_step(rudder_actuator_t *r, float motor_cmd, float dt_s);

#endif /* BOAT_MODEL_H */
