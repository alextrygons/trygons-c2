/*
 * sim_world.c - Simulated small boat. See sim_world.h.
 */
#include "sim_world.h"
#include "hal_sim.h"

#include <math.h>
#include <string.h>

#define PI_F        3.14159265f
#define KN_TO_MS    0.514444f
#define LAT0        60.0
#define LON0        25.0
#define M_PER_DEG   111320.0

void world_init(world_t *w, uint32_t seed, float heading_deg, float speed_kn)
{
    memset(w, 0, sizeof *w);
    w->vmax_fwd_kn = 20.0f;
    w->vmax_rev_kn = 8.0f;
    w->idle_fwd_kn = 1.5f;
    w->speed_tau_s = 6.0f;
    w->gps_pos_noise_m = 1.5f;
    w->heading_deg = heading_deg;
    w->speed_kn = speed_kn;
    w->rng = seed ? seed : 1u;
    w->gps_rng = (seed * 7919u) | 1u;
    w->cur_rng = (seed * 104729u) | 1u;
    w->pw_rng = (seed * 1299709u) | 1u;
    w->speed_sensor_scale = 1.02f;
    rudder_init_outboard(&w->rud, 3.0f, 3.0f, 2.0f);
    w->gear_pos_us = 1500.0f;
    w->engaged = GEAR_N;
}

static float wrap360(float d)
{
    d = fmodf(d, 360.0f);
    return (d < 0.0f) ? d + 360.0f : d;
}

float world_sog_kn(const world_t *w)
{
    float h = w->heading_deg * PI_F / 180.0f;
    float ve = w->speed_kn * KN_TO_MS * sinf(h) + w->current_e;
    float vn = w->speed_kn * KN_TO_MS * cosf(h) + w->current_n;
    return sqrtf(ve * ve + vn * vn) / KN_TO_MS;
}

float world_cog_deg(const world_t *w)
{
    float h = w->heading_deg * PI_F / 180.0f;
    float ve = w->speed_kn * KN_TO_MS * sinf(h) + w->current_e;
    float vn = w->speed_kn * KN_TO_MS * cosf(h) + w->current_n;
    return wrap360(atan2f(ve, vn) * 180.0f / PI_F);
}

void world_latlon(const world_t *w, double *lat, double *lon)
{
    *lat = LAT0 + w->north_m / M_PER_DEG;
    *lon = LON0 + w->east_m / (M_PER_DEG * cos(LAT0 * 3.14159265358979 / 180.0));
}

void world_from_latlon(double lat, double lon, double *north_m, double *east_m)
{
    *north_m = (lat - LAT0) * M_PER_DEG;
    *east_m = (lon - LON0) * M_PER_DEG * cos(LAT0 * 3.14159265358979 / 180.0);
}

/* Gear servo: moves at 2000 us/s; the gearbox engages near the ends. */
static void gear_model(world_t *w, float dt_s)
{
    /* no pulses (chip in reset): a normal RC servo holds where it is */
    float cmd = (w->servo_stuck || sim_hal_gear_pwm() == 0u) ? w->gear_pos_us : (float)sim_hal_gear_pwm();
    float step = 2000.0f * dt_s;
    if (cmd > w->gear_pos_us + step) w->gear_pos_us += step;
    else if (cmd < w->gear_pos_us - step) w->gear_pos_us -= step;
    else w->gear_pos_us = cmd;

    gear_t now = (w->gear_pos_us >= 1800.0f) ? GEAR_F
               : (w->gear_pos_us <= 1200.0f) ? GEAR_R : GEAR_N;
    if (now != w->engaged) {
        if (w->throttle > 5.0f) w->shift_under_load++;
        if (now != GEAR_N && (w->t_ms - w->neutral_since_ms) < 500u) w->fr_without_neutral++;
        if (now == GEAR_N) w->neutral_since_ms = w->t_ms;
        w->engaged = now;
    }
}

void world_step(world_t *w, float dt_s)
{
    w->t_ms += (uint32_t)lrintf(dt_s * 1000.0f);

    /* throttle servo + watch */
    float thr = sim_hal_throttle();
    if (w->t_ms % 50u == 0u) {          /* the controller updates every 50 ms */
        float rate = fabsf(thr - w->last_throttle) / 0.05f;
        if (rate > w->max_thr_rate) w->max_thr_rate = rate;
        w->last_throttle = thr;
    }
    w->throttle = thr;
    gear_model(w, dt_s);
    if (w->engaged == GEAR_F && thr > w->max_thr_fwd) w->max_thr_fwd = thr;
    if (w->engaged == GEAR_R && thr > w->max_thr_rev) w->max_thr_rev = thr;

    /* speed through water */
    float vss = 0.0f;
    if (w->engaged == GEAR_F) vss = w->idle_fwd_kn + (w->vmax_fwd_kn - w->idle_fwd_kn) * thr / 100.0f;
    if (w->engaged == GEAR_R) vss = -(0.8f + (w->vmax_rev_kn - 0.8f) * thr / 100.0f);
    w->speed_kn += (vss - w->speed_kn) * dt_s / w->speed_tau_s;

    /* steering actuator */
    rudder_step(&w->rud, sim_hal_motor_command(), dt_s);

    /* yaw: turn gain and time constant follow speed; reversed in reverse */
    float v = w->speed_kn;
    float av = fabsf(v) > 0.3f ? fabsf(v) : 0.3f;
    float K = (w->K_override > 0.0f) ? w->K_override : 0.05f * av;
    float T = (w->T_override > 0.0f) ? w->T_override : 12.0f / av;
    if (T < 0.4f) T = 0.4f;
    if (v < 0.0f) K = -K;
    if (fabsf(v) < 0.3f) K *= fabsf(v) / 0.3f;      /* no steering at rest */

    float tau = 1.5f;
    w->wave_filt += (dt_s / tau) *
        (boat_rand_gauss(&w->rng) * sqrtf(2.0f * tau / dt_s) - w->wave_filt);
    float waves = w->sea_state * (0.6f * sinf(2.0f * PI_F * (float)w->t_ms * 0.001f / 7.0f)
                                  + 0.6f * w->wave_filt);
    float input = w->rud.angle_deg + w->trim_deg + waves;
    w->yaw_dps += (K * input - w->yaw_dps) * (dt_s / T);
    w->heading_deg = wrap360(w->heading_deg + w->yaw_dps * dt_s);

    /* GPS drag (spoofer): offset grows until the maximum */
    if (w->gps_dragging) {
        w->gps_off_n += (double)(w->gps_drag_n * dt_s);
        w->gps_off_e += (double)(w->gps_drag_e * dt_s);
        if (sqrt(w->gps_off_n * w->gps_off_n + w->gps_off_e * w->gps_off_e) >= w->gps_drag_max_m)
            w->gps_dragging = false;
    }

    /* position */
    float h = w->heading_deg * PI_F / 180.0f;
    w->east_m += (double)((v * KN_TO_MS * sinf(h) + w->current_e) * dt_s);
    w->north_m += (double)((v * KN_TO_MS * cosf(h) + w->current_n) * dt_s);
}

void world_publish(world_t *w, uint32_t t_ms)
{
    sim_hal_set_time_ms(t_ms);
    sim_hal_set_truth(w->heading_deg, w->rud.angle_deg);
    sim_hal_set_yaw_truth(w->yaw_dps);
    sim_hal_set_neutral_switch(w->neutral_switch,
                               w->gear_pos_us > 1200.0f && w->gear_pos_us < 1800.0f);
    /* steering motor: ~2 A running, ~8 A pushing against an end stop */
    float mot = fabsf(sim_hal_motor_command());
    float amps = mot * (w->rud.stall_run_s > 0.0f ? 8.0f : 2.0f) + 0.1f * boat_rand_gauss(&w->cur_rng);
    sim_hal_set_motor_current(w->current_sensor, fmaxf(0.0f, amps));
    sim_hal_set_limit_switches(w->limit_switches,
                               w->rud.angle_deg <= -w->rud.max_angle_neg_deg + 0.05f,
                               w->rud.angle_deg >= w->rud.max_angle_deg - 0.05f);

    sim_hal_set_water_speed(w->speed_sensor,
                            w->speed_kn * w->speed_sensor_scale + 0.05f * boat_rand_gauss(&w->pw_rng));

    if (t_ms % 1000u == 0u && !w->gps_fail) {
        hal_gps_t g;
        double n = w->north_m + w->gps_pos_noise_m * boat_rand_gauss(&w->gps_rng) + w->gps_off_n;
        double e = w->east_m + w->gps_pos_noise_m * boat_rand_gauss(&w->gps_rng) + w->gps_off_e;
        double lat, lon;
        world_t tmp = *w;
        tmp.north_m = n; tmp.east_m = e;
        world_latlon(&tmp, &lat, &lon);
        float sog = world_sog_kn(w), cog = world_cog_deg(w);
        if (w->gps_dragging) {
            /* a spoofer makes speed and course match its fake track */
            float h = w->heading_deg * PI_F / 180.0f;
            float vn = w->speed_kn * KN_TO_MS * cosf(h) + w->current_n + w->gps_drag_n;
            float ve = w->speed_kn * KN_TO_MS * sinf(h) + w->current_e + w->gps_drag_e;
            sog = sqrtf(vn * vn + ve * ve) / KN_TO_MS;
            cog = wrap360(atan2f(ve, vn) * 180.0f / PI_F);
        }
        g.lat_deg = lat;
        g.lon_deg = lon;
        g.sog_kn = fmaxf(0.0f, sog + 0.1f * boat_rand_gauss(&w->gps_rng));
        g.cog_deg = wrap360(cog + (0.1f / fmaxf(sog, 0.2f)) * 57.3f * boat_rand_gauss(&w->gps_rng));
        g.fix = true;
        g.jam_spoof = w->gps_flag;
        g.time_ms = (36000000u + t_ms) % 86400000u;     /* the sim starts at 10:00 UTC ... */
        g.date = 20261020u;                             /* ... on 20 Oct 2026 */
        g.sats = 11u;
        sim_hal_gps_message(&g);
    }
}
