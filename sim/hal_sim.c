/*
 * hal_sim.c - hal.h implemented on top of the PC boat simulation.
 */
#include "hal_sim.h"
#include "boat_model.h"

#include <stdio.h>

#define COMPASS_PERIOD_MS   100u    /* 10 Hz */
#define RUDDER_PERIOD_MS     50u    /* 20 Hz */
#define GYRO_PERIOD_MS       50u    /* 20 Hz */
#define COMPASS_NOISE_DEG   0.2f
#define RUDDER_NOISE_DEG    0.1f
#define CALIB_FILE          "calib.bin"

static uint32_t s_time_ms;
static float    s_true_heading;
static float    s_true_rudder;
static float    s_true_yaw;
static bool     s_heading_failed;
static bool     s_rudder_feedback = true;  /* false = no angle sensor fitted */
static bool     s_gyro_fitted = false;
static float    s_gyro_bias;
static float    s_gyro_noise;

static uint32_t s_next_compass_ms;
static uint32_t s_next_rudder_ms;
static uint32_t s_next_gyro_ms;
static uint32_t s_buttons;
static float    s_motor;
static bool     s_alarm;
static uint32_t s_rng = 12345u;
static uint32_t s_gyro_rng = 777u;

/* ---- hal.h ------------------------------------------------------------- */

void hal_init(void)
{
    s_next_compass_ms = s_time_ms;
    s_next_rudder_ms = s_time_ms;
    s_next_gyro_ms = s_time_ms;
    s_buttons = 0;
    s_motor = 0.0f;
    s_alarm = false;
    s_rng = 12345u;         /* same sensor noise every run */
    s_gyro_rng = 777u;
}

uint32_t hal_millis(void)
{
    return s_time_ms;
}

bool hal_read_heading(float *heading_deg)
{
    if (s_heading_failed || (int32_t)(s_time_ms - s_next_compass_ms) < 0) {
        return false;
    }
    s_next_compass_ms += COMPASS_PERIOD_MS;

    float h = s_true_heading + COMPASS_NOISE_DEG * boat_rand_gauss(&s_rng);
    if (h < 0.0f) h += 360.0f;
    if (h >= 360.0f) h -= 360.0f;
    *heading_deg = h;
    return true;
}

bool hal_read_rudder_angle(float *rudder_deg)
{
    if (!s_rudder_feedback || (int32_t)(s_time_ms - s_next_rudder_ms) < 0) {
        return false;
    }
    s_next_rudder_ms += RUDDER_PERIOD_MS;
    *rudder_deg = s_true_rudder + RUDDER_NOISE_DEG * boat_rand_gauss(&s_rng);
    return true;
}

bool hal_read_yaw_rate(float *yaw_dps)
{
    if (!s_gyro_fitted || (int32_t)(s_time_ms - s_next_gyro_ms) < 0) {
        return false;
    }
    s_next_gyro_ms += GYRO_PERIOD_MS;
    /* separate random stream, so fitting a gyro doesn't change the compass noise */
    *yaw_dps = s_true_yaw + s_gyro_bias + s_gyro_noise * boat_rand_gauss(&s_gyro_rng);
    return true;
}

void hal_set_rudder_motor(float command)
{
    if (command > 1.0f) command = 1.0f;
    if (command < -1.0f) command = -1.0f;
    s_motor = command;
}

uint32_t hal_get_button_presses(void)
{
    uint32_t b = s_buttons;
    s_buttons = 0;
    return b;
}

void hal_set_alarm(bool on)
{
    s_alarm = on;
}

bool hal_calib_save(const void *data, uint32_t size)
{
    FILE *f = fopen(CALIB_FILE, "wb");
    if (!f) return false;
    bool ok = fwrite(data, 1, size, f) == size;
    return (fclose(f) == 0) && ok;
}

bool hal_calib_load(void *data, uint32_t size)
{
    FILE *f = fopen(CALIB_FILE, "rb");
    if (!f) return false;
    bool ok = fread(data, 1, size, f) == size;
    fclose(f);
    return ok;
}

/* ---- simulation control ------------------------------------------------ */

void sim_hal_set_time_ms(uint32_t t_ms)  { s_time_ms = t_ms; }

void sim_hal_set_truth(float heading_deg, float rudder_deg)
{
    s_true_heading = heading_deg;
    s_true_rudder = rudder_deg;
}

void sim_hal_set_yaw_truth(float yaw_dps) { s_true_yaw = yaw_dps; }

void sim_hal_set_gyro(bool fitted, float bias_dps, float noise_dps)
{
    s_gyro_fitted = fitted;
    s_gyro_bias = bias_dps;
    s_gyro_noise = noise_dps;
}

void sim_hal_fail_heading(bool failed)   { s_heading_failed = failed; }
void sim_hal_press(hal_button_t button)  { s_buttons |= HAL_BTN_BIT(button); }
float sim_hal_motor_command(void)        { return s_motor; }
bool  sim_hal_alarm(void)                { return s_alarm; }
void sim_hal_set_rudder_feedback(bool fitted) { s_rudder_feedback = fitted; }
