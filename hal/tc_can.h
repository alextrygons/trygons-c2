/*
 * tc_can.h - The board's CAN controller for DroneCAN (MCMCAN module CAN0,
 * node 0, iLLD IfxCan_Can; pins / STB / bit rate in hal_pins.h, all TO CONFIRM:
 * HAL_CAN_FITTED 0 = never started). Board only, core 0 only.
 *
 * It is the port hal/dronecan.c drives (dc_port_t): no blocking wait after the
 * start - send = one frame into the hardware TX FIFO (full = false at once),
 * recv = one frame out of RX FIFO 0 (empty = false), state = the error
 * counters and bus-off / error-passive bits, recover = clear INIT after a
 * bus-off (the controller then waits 128 x 11 recessive bits by itself).
 * Message RAM: 1 dedicated TX buffer (buffer 0, RESERVED for CPU1's safety
 * frames from step 11e - core 0 never writes it) + a 16-entry TX FIFO + a
 * 32-entry RX FIFO 0; every extended-ID frame is accepted, standard-ID frames
 * are rejected (DroneCAN uses only 29-bit IDs).
 */
#ifndef TC_CAN_H
#define TC_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "dronecan.h"

#define CAN_TX_FIFO     16u
#define CAN_RX_FIFO     32u
#define CAN_TX_RESERVED 1u      /* dedicated TX buffer 0: CPU1 (step 11e) */

bool    can_start(void);
bool    can_send(const dc_frame_t *f);
bool    can_recv(dc_frame_t *f);
uint8_t can_state(uint8_t *tec, uint8_t *rec);
void    can_recover(void);

#endif /* TC_CAN_H */
