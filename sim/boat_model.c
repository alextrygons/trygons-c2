/*
 * boat_model.c - Simple boat + rudder actuator model for PC simulation.
 */
#include "boat_model.h"

#include <math.h>

#define PI_F 3.14159265f

/* xorshift32: tiny, deterministic random generator */
static float rand_uniform(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return (float)(x >> 8) / 16777216.0f;   /* 0..1 */
}

float boat_rand_gauss(uint32_t *state)
{
    /* Box-Muller */
    float u1 = rand_uniform(state);
    float u2 = rand_uniform(state);
    if (u1 < 1e-7f) u1 = 1e-7f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}

/* ---------------------------------------------------------------- boat -- */

void boat_init(boat_t *b, float heading_deg, float speed_kn, uint32_t seed)
{
    b->heading_deg = heading_deg;
    b->yaw_rate_dps = 0.0f;
    b->speed_kn = speed_kn;
    b->current_deg = 3.0f;      /* steady push to starboard: needs ~-3 deg rudder */
    b->wave_amp_deg = 3.0f;
    b->wave_filt = 0.0f;
    b->rng = seed ? seed : 1u;
}

void boat_step(boat_t *b, float rudder_deg, float t_s, float dt_s)
{
    /* Faster boat = turns harder (K up) and responds quicker (T down).
     * At 6 kn: K = 0.3 1/s  (10 deg rudder -> 3 deg/s turn), T = 2 s. */
    float v = b->speed_kn > 0.5f ? b->speed_kn : 0.5f;
    float K = 0.05f * v;
    float T = 12.0f / v;

    /* Waves: a regular swell (period 7 s) + filtered random chop. */
    float tau = 1.5f;
    b->wave_filt += (dt_s / tau) *
        (boat_rand_gauss(&b->rng) * sqrtf(2.0f * tau / dt_s) - b->wave_filt);
    float waves = b->wave_amp_deg *
        (0.6f * sinf(2.0f * PI_F * t_s / 7.0f) + 0.6f * b->wave_filt);

    float input = rudder_deg + b->current_deg + waves;

    b->yaw_rate_dps += (K * input - b->yaw_rate_dps) * (dt_s / T);
    b->heading_deg += b->yaw_rate_dps * dt_s;
    b->heading_deg = fmodf(b->heading_deg, 360.0f);
    if (b->heading_deg < 0.0f) b->heading_deg += 360.0f;
}

/* -------------------------------------------------------------- rudder -- */

void rudder_init(rudder_actuator_t *r)
{
    r->angle_deg = 0.0f;
    r->max_rate_dps = 6.0f;     /* hard over to hard over (70 deg) in ~12 s */
    r->max_angle_deg = 35.0f;
}

void rudder_step(rudder_actuator_t *r, float motor_cmd, float dt_s)
{
    if (motor_cmd > 1.0f) motor_cmd = 1.0f;
    if (motor_cmd < -1.0f) motor_cmd = -1.0f;

    r->angle_deg += motor_cmd * r->max_rate_dps * dt_s;
    if (r->angle_deg > r->max_angle_deg) r->angle_deg = r->max_angle_deg;
    if (r->angle_deg < -r->max_angle_deg) r->angle_deg = -r->max_angle_deg;
}
