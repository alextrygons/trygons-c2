/*
 * can_safe.h - CPU1's own encoder for its SAFE frames (CLAUDE.md T, docs/dronecan_design.md
 * 6.3; step 11e). Pure C99, NO libcanard (CPU1 stays small: its RAM grows by a few bytes,
 * tools/mapreport.py keeps the 60 % rule).
 *
 * One single-frame uavcan.equipment.actuator.ArrayCommand (data type 1010) with ONE
 * command: payload = actuator_id (u8), command_type UNITLESS (u8 0), value (float16, little
 * endian), then the tail byte (start + end of transfer, toggle 0, the transfer ID); the
 * identifier = priority DC_PRIO_SAFE (0: wins the arbitration) | type 1010 | source = OUR
 * node ID 10. A single-frame transfer has no transfer CRC. CPU1 continues core 0's
 * ArrayCommand transfer-ID counter (dc_cmd_tid) so a node never drops its frames as
 * duplicates (TO CONFIRM C19 on the bench). Checked against the independent encoder of
 * tests/hal_pc/dronecan_check.py (D16).
 */
#ifndef CAN_SAFE_H
#define CAN_SAFE_H

#include <stdbool.h>
#include <stdint.h>

#include "dronecan.h"

/* IEEE half precision of v (round to nearest even; NaN -> 0x7E00, overflow -> infinity) */
uint16_t cs_float16(float v);

/* one safe ArrayCommand frame: value clamped to -1..+1 (NaN -> -1 = throttle idle) */
void cs_frame(dc_frame_t *f, uint8_t actuator_id, float value, uint8_t tid);

#endif /* CAN_SAFE_H */
