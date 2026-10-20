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
float    sim_hal_throttle(void);          /* % of full travel, from the servo pulse */
uint16_t sim_hal_throttle_us(void);
uint16_t sim_hal_gear_pwm(void);
void     sim_hal_gps_message(const hal_gps_t *g);
void     sim_hal_reset_engine(void);

/* Optional roll-rate sensor. */
void     sim_hal_set_roll(bool fitted, float roll_dps);

/* Optional neutral switch. */
void     sim_hal_set_neutral_switch(bool fitted, bool in_neutral);

/* Optional steering motor current sense and limit switches. */
void     sim_hal_set_motor_current(bool fitted, float amps);
void     sim_hal_set_limit_switches(bool fitted, bool left, bool right);

/* Optional paddlewheel (speed through water). */
void     sim_hal_set_water_speed(bool fitted, float kn);

/* Joystick (raw 12-bit ADC values and buttons). */
void     sim_hal_set_joystick(bool fitted, uint16_t x_raw, uint16_t y_raw, bool take, bool autob);
void     sim_hal_joystick_confirm(bool pressed);     /* CONFIRM button held */

/* Ground-station link byte streams (PC side of hal_link_read / write). */
uint32_t sim_hal_link_rx_push(const uint8_t *buf, uint32_t len);   /* PC -> boat */
uint32_t sim_hal_link_tx_pop(uint8_t *buf, uint32_t max);          /* boat -> PC */
void     sim_hal_link_reset(void);

/* Hardening: safety relay state, simulated chip reset, clear the monitor latch. */
bool     sim_hal_safety_relay(void);
void     sim_hal_chip_reset(hal_reset_t why);
void     sim_hal_unforce(void);

/* hal_micros(): false = simulated time (tests), true = the real PC clock. */
void     sim_hal_real_micros(bool on);
uint64_t sim_hal_real_ns(void);                  /* PC clock in ns (timing report) */

/* microSD card in a folder: card present, free space, every write failing. */
void     sim_hal_sd_setup(const char *dir, bool present, uint32_t free_kb, bool fail_writes);
void     sim_hal_sd_power_cut(void);               /* unsynced data is lost */

#endif /* HAL_SIM_H */
