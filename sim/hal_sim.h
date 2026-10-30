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

/* Supply voltage of the 12 V bank (default: measured, 12.6 V). */
void     sim_hal_set_supply(bool fitted, float volts);
/* DP thrusters: what hal_set_thrusters got (0 after a trip), what really pushes (0 while the e-stop
 * contactor is open), calls so far, the bank sag (V per % of |bow| + |stern|, 0.006) */
void     sim_hal_thrusters(float *bow_pct, float *stern_pct);
void     sim_hal_thrust_truth(float *bow_pct, float *stern_pct);
uint32_t sim_hal_thruster_writes(void);
void     sim_hal_thruster_sag(float v_per_pct);

/* RS485 Modbus relay board (16 coils, address 1): present with a reply
 * latency (default 5 ms), faults; its true states; what it received. The
 * board keeps its states across an AURIX reboot (hal_init); board_reset =
 * its own power cycle (every relay off). force = a relay changed by itself
 * (a hand switch on the board). */
#define SIM_RB_SILENT       0x1u    /* never answers (bus cut / board dead) */
#define SIM_RB_CORRUPT      0x2u    /* every reply has a bad CRC */
#define SIM_RB_CORRUPT_SOME 0x4u    /* one reply in 7 has a bad CRC */
#define SIM_RB_NO_PULSE_END 0x8u    /* the board's pulse timer is broken: a "flash on" stays ON (DroneCAN board: its
                                       command timeout R-3 is broken - a momentary relay stays as last commanded) */
#define SIM_RB_NO_READBACK  0x10u   /* DroneCAN board: sends no hardpoint.Status (the AURIX shows the commanded states) */
/* Step 11f: the SAME calls drive the DroneCAN relay board (node 30, hardpoints 0..15 = relays 1..16) when the
 * relay bus is DroneCAN (sim_hal_relay_bus, taken at the next hal_init; needs the CAN bus fitted). There:
 * present false = the board's CAN drop cable cut (it hears nothing, sends nothing, its relays hold); SILENT =
 * it acts on commands but sends no NodeStatus / Status; CORRUPT(_SOME) = its frames arrive broken (refused by
 * the transport); latency = how late a command takes effect; requests = hardpoint frames received; writes =
 * relay changes made by commands; pulses = relays of the timeout mask switched on; max_pulse = the longest
 * pulse the AURIX asked. The board's command timeout (R-3, 300 ms) applies to the timeout mask only (default
 * relays 2 / 4 / 5 = the momentary ones of the sample table); every other relay HOLDS (R-2). Unpowered
 * (sim_hal_can_node(30, false ...)) or rebooted = every relay OFF. */
#define SIM_RELAY_MODBUS    1u
#define SIM_RELAY_DRONECAN  2u
void     sim_hal_relay_bus(uint8_t bus);
uint8_t  sim_hal_relay_bus_now(void);       /* the back-end the AURIX started with (0 = no board) */
void     sim_hal_relay_timeout_mask(uint32_t mask_n);   /* bit n-1: relay n has the board's command timeout */
void     sim_hal_relay_weld(uint8_t n, bool welded);    /* relay n's contact welded: stays as it is whatever comes */
/* The STARTER's hardware time-limit relay (OD-9, both buses): the starter motor is cut when the STARTER relay
 * (default relay 2) has been on longer than limit_ms (default 6000) in one go. cranking = the motor turns. */
void     sim_hal_starter_limit(uint8_t n, uint32_t limit_ms);
bool     sim_hal_starter_cranking(void);
void     sim_hal_relay_board(bool present, uint32_t latency_ms, uint32_t fault);
void     sim_hal_relay_board_reset(void);
bool     sim_hal_relay_state(uint8_t n);
void     sim_hal_relay_force(uint8_t n, bool on);
uint32_t sim_hal_relay_requests(void);     /* frames the board received */
uint32_t sim_hal_relay_writes(void);       /* coil writes + pulses carried out */
uint32_t sim_hal_relay_pulses(void);       /* board-timed pulses started */
uint32_t sim_hal_relay_max_pulse_ms(void); /* the longest board-timed pulse ever asked (0 = none) */
uint32_t sim_hal_relay_edges(uint8_t n);   /* coil n: off -> on changes on the board (any cause) */
/* step 13h2 (review 3bef4c8 LOW-1, storm W42): per coil the last ACCEPTED relays.c command (hal_relay_set ON / OFF,
 * hal_relay_pulse = ON with last_on_pulse), the board's own R-3 timeout OFF and the board switching every coil OFF by
 * itself (DroneCAN reboot / power loss); times in sim ms, 0 = never */
typedef struct {
    uint32_t on_cmd_ms, off_cmd_ms, r3_off_ms, board_off_ms;
    bool     last_on_pulse;
} sim_relay_w42_t;
bool     sim_hal_relay_w42(uint8_t n, sim_relay_w42_t *w);
/* the board fitted with this many coils (hal_relay_channels; default 16); 0 = no board
 * fitted (like HAL_RELAY_FITTED 0 on the TC375: the driver is never started). Takes
 * effect at the next hal_init (the AURIX's boot). */
void     sim_hal_relay_fitted(uint8_t channels);

/* DroneCAN bus (step 11d): the real hal/dronecan.c on a simulated controller + bus + nodes
 * (throttle 20 / steering 21 / gear 22 = servo nodes, actuators 1 / 2 / 3, relay node 30 =
 * hardpoints 0..5; NodeStatus 1 Hz each). fitted: taken at the next hal_init (default off,
 * like HAL_CAN_FITTED 0). reset: every node powered, OK, OPERATIONAL, worst-case servo speeds. */
#define SIM_CAN_BUSOFF      0x1u    /* our controller goes bus-off (recovers 50 ms after the fault ends) */
#define SIM_CAN_FOREIGN10   0x2u    /* a node with OUR ID 10 sends NodeStatus (the Pixhawk on the bus) */
#define SIM_CAN_ANON        0x4u    /* a node asks for a dynamic node ID (no static ID set) */
#define SIM_CAN_START_FAIL  0x8u    /* the controller does not start (at the next hal_init) */
void     sim_hal_can_fitted(bool on);
void     sim_hal_can_reset(void);
void     sim_hal_can_fault(uint32_t bits);
/* a node powered / its health + mode in NodeStatus (power back = a node boot: uptime 0, INIT 1 s) */
void     sim_hal_can_node(uint8_t id, bool present, uint8_t health, uint8_t mode);
void     sim_hal_can_node_silent(uint8_t id, bool silent);   /* powered, acts on commands, no NodeStatus */
void     sim_hal_can_node_reboot(uint8_t id);                /* uptime back to 0 (brown-out) */
/* step 13k (W44) fault: the first NodeStatus after a quiet time (silent / cut off; not after a power-up) is
 * delivered late_ms late (its uptime is of late_ms ago: it waited in the node's TX mailbox); stale = then also an
 * OLDER status (uptime 1 s less) right after it. late_ms 0 + stale false = off. late_sent / stale_sent: counts */
void     sim_hal_can_node_late(uint8_t id, uint32_t late_ms, bool stale);
uint32_t sim_hal_can_node_late_sent(uint8_t id, uint32_t *stale_sent);
void     sim_hal_can_servo_speed(uint8_t id, float travel_s, float backlash);
/* servo node: last command, servo position, output after the play (units -1..1), commands taken;
 * returns false while it never got a command (it holds its power-up position) */
bool     sim_hal_can_servo(uint8_t id, float *cmd, float *pos, float *out, uint32_t *cmds);
uint32_t sim_hal_can_servo_changes(uint8_t id);  /* commands with a new value the node got (step 13b, storm I51) */
uint16_t sim_hal_can_hardpoint(uint8_t hp, uint32_t *cmds);  /* relay node: last command of hardpoint hp */
uint32_t sim_hal_can_bus_frames(uint32_t kind);  /* frames from the AURIX: 0 NodeStatus 1 ArrayCommand 2 hardpoint 3 other */
uint32_t sim_hal_can_status_period_ms(void);    /* between our last two NodeStatus on the bus */
uint32_t sim_hal_can_tx_waiting(void);          /* frames in our controller's TX FIFO (not acknowledged) */
uint32_t sim_hal_can_node_statuses(uint8_t id);

/* DroneCAN servos (step 11e): a fitted bus carries the throttle / steering / gear servos (default on;
 * off = the bus without servos, the old 11d case). What the servos REALLY do (the world uses it): the
 * throttle node's position %, the gear node's output as an RC pulse, the steering node's outboard after the
 * play in the app's sign (+ = starboard); PWM hardware: the commands. */
void     sim_hal_can_servos(bool on);
bool     sim_hal_servos_can(void);          /* the servos are DroneCAN nodes since the last hal_init */
void     sim_hal_board_boot_begin(void);    /* step 13e: hal_servos_can() FALSE until the next hal_init (board bss) */
float    sim_hal_thr_truth(void);
float    sim_hal_gear_truth_us(void);
float    sim_hal_steer_truth(void);
float    sim_hal_steer_cmd(bool *have);     /* what the app commanded (have = a steering command at all) */
void     sim_hal_can_servo_set(uint8_t id, float pos);   /* a servo's position before the boot (tests) */
uint32_t sim_hal_can_safe_frames(void);     /* CPU1's safe frames on the bus */
bool     sim_hal_cr_keep(uint32_t *known, uint32_t *on);   /* the DroneCAN relay keep store (no-clear RAM, step 11g) */
/* the ArrayCommands that went onto the wire, per actuator (1 throttle, 2 steering, 3 gear): from core 0
 * (priority 8) and from CPU1 (priority 0), the first and the last value + time; cleared by hand */
typedef struct {
    bool     any;
    uint32_t n_core0, n_cpu1;
    float    first, last, last_cpu1;
    uint32_t first_ms, last_ms, core0_ms, cpu1_ms;
} sim_can_act_t;
void     sim_hal_can_act_clear(void);
void     sim_hal_can_act(uint8_t act, sim_can_act_t *a);
uint8_t  sim_hal_can_first_act(void);       /* the actuator of the first ArrayCommand since the clear (0 none) */
void     sim_hal_can_task_stuck(bool stuck);/* core 0's actuator task stops (its CAN path stuck) */
/* the UNCREWED dead-man timer relay (OA-3): armed = in the kill circuit (the on-board UNCREWED switch);
 * no pulse edge for 1.5 s = the engine stops (latched until sim_hal_reset_engine / restart) */
void     sim_hal_deadman_arm(bool armed);
bool     sim_hal_deadman_killed(void);
uint32_t sim_hal_deadman_edges(void);
void     sim_hal_deadman_restart(void);
bool     sim_hal_estop_pressed(void);       /* (OD-1: the e-stop also stops the engine, every profile) */

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
bool     sim_hal_reset_requested(void);     /* hal_system_reset() was called (read clears); the harness reboots */
uint32_t sim_hal_sw_resets(void);           /* hal_system_reset() calls so far */
void     sim_hal_chip_reset(hal_reset_t why);
void     sim_hal_unforce(void);

/* hal_micros(): false = simulated time (tests), true = the real PC clock. */
void     sim_hal_real_micros(bool on);
uint64_t sim_hal_real_ns(void);                  /* PC clock in ns (timing report) */

/* microSD card in a folder: card present, free space, every write failing. */
void     sim_hal_sd_setup(const char *dir, bool present, uint32_t free_kb, bool fail_writes);
void     sim_hal_sd_power_cut(void);               /* unsynced data is lost */
uint32_t sim_hal_core_kicks(void);
uint32_t sim_hal_wdt_kicks(void);             /* step 13h: hal_watchdog_kick() calls (core 0: the NV boot loop, CN34 / CN35) */
void     sim_hal_set_estop(bool fitted, bool pressed);    /* emergency stop input */                 /* hal_watchdog_kick_core() calls (logger between SD ops) */
uint32_t sim_hal_sd_ops(void);                     /* hal_sd_open / write / sync / close calls */

/* Non-volatile storage (files nv<slot>.bin): faults for the tests. */
#define SIM_NV_POWER_LOSS 0x1u   /* the next flash write is cut: the file keeps the old record, the save is lost */
#define SIM_NV_STUCK      0x2u   /* the flash never finishes: saves stay pending, NV FAULT after the time limit */
void     sim_hal_nv_fault(uint32_t bits);
uint32_t sim_hal_nv_writes(void);          /* records written to the files */
uint32_t sim_hal_nv_write_ms(void);        /* sim time of the last one */
uint32_t sim_hal_nv_lost(void);            /* saves lost to a simulated power loss */

#endif /* HAL_SIM_H */
