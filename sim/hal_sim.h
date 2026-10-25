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
void sim_hal_push_accel(float g);               /* one BNO085 vertical acceleration sample */
void sim_hal_set_accel(bool fitted);            /* the BNO085 is fitted / works */
void sim_hal_push_attitude(float roll_deg, float pitch_deg);   /* BNO085 roll / pitch (waves) */
void sim_hal_set_attitude(bool fitted);

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

/* USB joystick bridge. Bridge mode: no analog joystick; the joystick state
 * above is sent as bridge frames (100 Hz) - or a state set directly. */
#include "jsframe.h"
#define SIM_BR_STOP    0x1u     /* no frames (cable cut / bridge dead) */
#define SIM_BR_CORRUPT 0x2u     /* every frame has a flipped bit (CRC error) */
#define SIM_BR_FREEZE  0x4u     /* sequence counter stuck (bridge repeating) */
#define SIM_BR_CORRUPT_SOME 0x8u /* one frame in 50 corrupted */
void     sim_hal_bridge_mode(bool on);
bool     sim_hal_bridge_on(void);
void     sim_hal_bridge_set(const jsf_t *f);          /* direct state (seq ignored) */
void     sim_hal_bridge_analog(void);                 /* back to the analog joystick state */
void     sim_hal_bridge_fault(uint32_t bits);
/* USB poll + hub delay, UART transfer time, phase of the next frame tick */
void     sim_hal_bridge_timing(uint32_t usb_ms, uint32_t xfer_ms, uint32_t phase_ms);
uint32_t sim_hal_bridge_frames(void);

/* Ground-station link byte streams (PC side of hal_link_read / write). */
uint32_t sim_hal_link_rx_push(const uint8_t *buf, uint32_t len);   /* PC -> boat */
uint32_t sim_hal_link_tx_pop(uint8_t *buf, uint32_t max);          /* boat -> PC */
void     sim_hal_link_reset(void);

/* Gateway UART byte streams (Pi side of hal_gw_read / write). The boat ->
 * Pi side is limited to the baud rate (bytes/s = baud / 10). */
uint32_t sim_hal_gw_rx_push(const uint8_t *buf, uint32_t len);     /* Pi -> boat */
uint32_t sim_hal_gw_tx_pop(uint8_t *buf, uint32_t max);            /* boat -> Pi */
void     sim_hal_gw_reset(uint32_t baud);

/* Hardening: safety relay state, simulated chip reset, clear the monitor latch. */
bool     sim_hal_safety_relay(void);
bool     sim_hal_engine_killed(void);       /* ENGINE KILL output on (engine stopped) */
uint32_t sim_hal_engine_kills(void);        /* times it was switched on */
void     sim_hal_chip_reset(hal_reset_t why);
void     sim_hal_unforce(void);

/* hal_micros(): false = simulated time (tests), true = the real PC clock. */
void     sim_hal_real_micros(bool on);
uint64_t sim_hal_real_ns(void);                  /* PC clock in ns (timing report) */

/* microSD card in a folder: card present, free space, every write failing. */
void     sim_hal_sd_setup(const char *dir, bool present, uint32_t free_kb, bool fail_writes);
void     sim_hal_sd_power_cut(void);               /* unsynced data is lost */
uint32_t sim_hal_core_kicks(void);
void     sim_hal_set_estop(bool fitted, bool pressed);    /* emergency stop input */                 /* hal_watchdog_kick_core() calls (logger between SD ops) */
uint32_t sim_hal_sd_ops(void);                     /* hal_sd_open / write / sync / close calls */

#endif /* HAL_SIM_H */
