/*
 * can_act.h - The CAN OUTPUT LAYER of the servos (CLAUDE.md T, docs/dronecan_design.md
 * sections 4-6; step 11e). Pure C99 over hal/dronecan.c (no iLLD): the same code on
 * the TC375 (hal_tc375.c) and in the PC simulation (sim/hal_sim.c).
 *
 * Core 0 only. app/servo.c (the ONLY writer of the throttle / gear / steering outputs)
 * reaches it through hal_set_throttle_pwm / hal_set_gear_pwm / hal_set_steer_pos; the
 * HAL turns those into the DroneCAN "wire" values here. ca_tick is the 10 ms actuator
 * task (core 0's main loop between the 20 Hz control steps; the sim harness every
 * 10 ms): it encodes ONE ArrayCommand per commanded actuator (UNITLESS, one command per
 * transfer = one CAN frame) and hands them to the controller - never a blocking wait.
 * Only this file calls dc_send_actuator (tools/check.sh).
 *
 * Wire values (docs/dronecan_design.md 2 / 4, the ArduPilot mapping, TO CONFIRM C7 on the
 * bench): unitless = (pulse_us - 1000) / 500 - 1, so throttle idle 1000 us = -1.0 and
 * 0.0 = HALF throttle; the gear positions are the configured gear pulses in the same
 * mapping (TO CONFIRM C4); steering wire = CA_STEER_SIGN x the app's position (+ = to
 * starboard; ArduPilot had SERVO2_REVERSED = 1: TO CONFIRM C6).
 *
 * Boot order (design 6.4, owner decisions OD-7 / OA-2): after ca_init the ONLY frame is
 * throttle IDLE (-1.0); the gear has no frame until servo.c sends one (it holds them until
 * the modelled throttle is idle with the throttle node OPERATIONAL); the steering has no
 * frame until a mode commands steering (hal_set_steer_pos with command = true).
 *
 * The monitor latch (hal_outputs_forced, CPU1 trip): checked BEFORE and AFTER queueing; a
 * latch set in between cancels the frames just queued (dc_cancel_tx) - core 0 can never
 * overwrite CPU1's safe frames (the existing "re-check the latch after the write" rule).
 */
#ifndef CAN_ACT_H
#define CAN_ACT_H

#include <stdbool.h>
#include <stdint.h>

/* actuator IDs on the wire = ArduPilot's servo channel numbers (TO CONFIRM C2) */
#define CA_ACT_THROTTLE   1u
#define CA_ACT_STEERING   2u
#define CA_ACT_GEAR       3u
/* steering wire sign (ArduPilot SERVO2_REVERSED = 1): wire = CA_STEER_SIGN x position (TO CONFIRM C6) */
#define CA_STEER_SIGN     (-1.0f)

/* unitless wire value of an RC pulse (us), clamped to -1..+1 */
float ca_us_to_wire(uint16_t us);
/* ... and back (the sim nodes / the tests) */
float ca_wire_to_us(float wire);

/* boot: throttle idle is the only demand (the first frame), gear and steering: none */
void ca_init(void);
/* core 0's demands (servo.c through the HAL) */
void ca_set_throttle_us(uint16_t us);
void ca_set_gear_us(uint16_t us);            /* 0 = no gear frame (the node HOLDS) */
void ca_set_steer(bool command, float pos);  /* command false = no steering frame (the node HOLDS); pos -1..+1 */

/* the 10 ms actuator task (never blocks): queues the commanded frames + hands them to the controller */
void ca_tick(void);

/* ticks that ran (CPU1 reads it: no tick for MON_CAN_TX_MS while core 0 runs = MON_TRIP_CAN_TX) */
uint32_t ca_ticks(void);

typedef struct {
    float    thr_wire;          /* last throttle wire value */
    bool     gear_have;         /* a gear frame was sent since the boot */
    float    gear_wire;         /* ... the last one */
    bool     steer_have;        /* a steering frame was sent since the boot (OD-7) */
    float    steer_wire;        /* ... the last one */
    uint32_t frames;            /* ArrayCommands queued */
    uint32_t refused;           /* not queued (bus fault / no room) */
    uint32_t cancelled;         /* ticks whose frames were cancelled: the latch came in between */
    uint32_t ticks;
} ca_state_t;
void ca_get(ca_state_t *st);

#endif /* CAN_ACT_H */
