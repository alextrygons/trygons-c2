/*
 * hal_sim.h - Extra functions the simulation uses to drive the fake HAL.
 * (The autopilot itself only sees hal.h.)
 */
#ifndef HAL_SIM_H
#define HAL_SIM_H

#include <stdbool.h>
#include <stdint.h>
#include "hal.h"

/* Advance simulated time. */
void sim_hal_set_time_ms(uint32_t t_ms);

/* Feed the true values from the models. The fake sensors sample these
 * at their own rate (compass 10 Hz, rudder sensor 20 Hz) and add noise. */
void sim_hal_set_truth(float heading_deg, float rudder_deg);

/* True yaw rate for the simulated gyro, deg/s. */
void sim_hal_set_yaw_truth(float yaw_dps);

/* Fit a gyro (default: none) with a constant bias and random noise. */
void sim_hal_set_gyro(bool fitted, float bias_dps, float noise_dps);

/* Simulate a dead compass (no new samples while true). */
void sim_hal_fail_heading(bool failed);

/* Is there a rudder angle sensor? (false = outboard, no feedback) */
void sim_hal_set_rudder_feedback(bool fitted);

/* Queue a button press; the autopilot sees it on its next cycle. */
void sim_hal_press(hal_button_t button);

/* What the autopilot last commanded. */
float sim_hal_motor_command(void);
bool  sim_hal_alarm(void);

/* Small boat: what the controller commanded, and GPS messages. */
float    sim_hal_throttle(void);
uint16_t sim_hal_gear_pwm(void);
void     sim_hal_gps_message(const hal_gps_t *g);
void     sim_hal_reset_engine(void);

#endif /* HAL_SIM_H */
