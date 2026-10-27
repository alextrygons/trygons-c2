/*
 * tc_modbus.h - Modbus RTU master for the RS485 relay board (pure C99, no
 * iLLD: the serial port is reached through a small port, so the same code
 * runs on the TC375 (hal_tc375.c: an ASCLIN + the RS485 DE pin) and in the
 * PC simulation against a simulated board (sim/hal_sim.c, sim tests RB1-RB8)).
 *
 * ONE request in flight, a state machine driven by mb_service() once per
 * control step: IDLE -> SEND (the request frame goes out) -> WAIT (the reply
 * is collected, at most timeout_ms) -> IDLE. Never a blocking wait. Writes
 * (set / pulse / multiple coils) wait in a small queue and go first; while
 * nothing waits the coils are read back (function 0x01) every poll_ms.
 *   0x01 read coils   -> the states shown to the driver (readback)
 *   0x05 write coil   -> ON (0xFF00) / OFF (0x0000), or the board-timed
 *                        pulse "flash on" (0x02xx: xx = time in 100 ms units,
 *                        Waveshare-style Modbus RTU relay boards - TBD per the
 *                        board chosen, PARTS.md)
 * CRC-16/MODBUS (0xA001 reflected, init 0xFFFF) on every frame; a reply with
 * a bad CRC, a wrong address / function, an exception, or no reply within
 * timeout_ms counts as a failed request (the write is dropped - the caller
 * sees the real state in the readback). No valid reply for lost_ms = the
 * board is SILENT (mb_ok false, states unknown); the master keeps polling, so
 * a board that comes back is found by itself.
 */
#ifndef TC_MODBUS_H
#define TC_MODBUS_H

#include <stdbool.h>
#include <stdint.h>

#define MB_COILS_MAX   32u      /* coils the master tracks (relay channels 1..32) */
#define MB_QUEUE       8u       /* writes waiting */
#define MB_FRAME_MAX   16u      /* longest request / reply we use */

typedef struct {
    /* queue a whole frame for transmission (returns at once; false = no room) */
    bool (*write)(const uint8_t *buf, uint32_t len);
    /* bytes received since the last call (non-blocking; the board port also
     * releases the RS485 driver pin here once the request has left the wire) */
    uint32_t (*read)(uint8_t *buf, uint32_t max);
    uint32_t (*millis)(void);
} mb_port_t;

typedef struct {
    uint8_t  slave;             /* Modbus address of the board (1) */
    uint8_t  n_coils;           /* coils read back per poll (the board's channel count, <= MB_COILS_MAX) */
    uint32_t timeout_ms;        /* reply must be complete within this (100) */
    uint32_t lost_ms;           /* no valid reply this long = SILENT (3000) */
    uint32_t poll_ms;           /* readback period while nothing is queued (100) */
} mb_config_t;

typedef struct {
    uint32_t requests, replies, timeouts, crc_errors, exceptions, bad_replies, dropped;
    uint32_t last_reply_ms;     /* 0 = never */
    uint8_t  state;             /* 0 idle, 1 sending, 2 waiting */
    uint8_t  last_exception;    /* exception code of the last exception reply */
} mb_stats_t;

void     mb_init(const mb_port_t *port, const mb_config_t *cfg);
/* one bounded step of the state machine (every control step) */
void     mb_service(void);
/* queue: coil n (1..n_coils, the board's channel count) on / off; a board-timed
 * pulse of ms (100 ms units, 100..25500). false = no room / bad argument / n
 * beyond the board (nothing queued). */
bool     mb_set_coil(uint8_t n, bool on);
bool     mb_pulse_coil(uint8_t n, uint16_t ms);
/* the last readback: true + state if coil n was read within lost_ms */
bool     mb_coil(uint8_t n, bool *on);
/* the board answered within lost_ms (since the start: within lost_ms of mb_init
 * the board is given time to answer; false = SILENT) */
bool     mb_ok(void);
bool     mb_idle(void);                 /* nothing in flight and nothing queued */
const mb_stats_t *mb_stats(void);
uint16_t mb_crc16(const uint8_t *p, uint32_t n);

#endif /* TC_MODBUS_H */
