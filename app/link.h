/*
 * link.h - Ground-station link: telemetry out, commands in (proto.h frames).
 *
 * Call link_step() once per control step, right after sb_step().
 *
 *   - 10 Hz telemetry frame (PROTO_T_TELEMETRY)
 *   - every command is checked by the same rules as the buttons (mode
 *     manager, logged as WHY_PC) and answered with ACK or NACK + reason
 *   - the PC can never take control from the joystick: while JOYSTICK is
 *     active, mode commands are refused (PROTO_R_JOYSTICK); only the
 *     hand-back command, with the stick centred, ends the override
 *   - heartbeat: no good frame from the PC for pc_timeout_s (3 s) after it
 *     was connected -> alarm "PC link lost" (SB_ALM_PC_LINK). The mode is
 *     NOT changed: the boat carries on as before.
 *   - parameters (config editor): a small table of live settings, read at
 *     any time, written in STANDBY only, range checked. RAM only (lost at
 *     power-off).
 */
#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "proto.h"
#include "smallboat.h"

/* Simulator control frames (0x70..0x7F); returns a PROTO_R_ result. */
typedef uint8_t (*link_sim_fn)(void *ctx, const proto_frame_t *f);

typedef struct {
    proto_rx_t  rx;
    uint8_t     tx_seq;
    bool        connected;      /* a good frame was received at least once */
    bool        lost;           /* connected, then silent for > timeout */
    uint32_t    last_rx_ms;
    uint32_t    rx_idle_steps;  /* steps with bytes waiting and nothing new */
    uint32_t    next_telem_ms;
    float       pc_timeout_s;   /* 3 */
    uint32_t    commands, nacks, tx_dropped;
    uint8_t     last_result;    /* last ACK result sent (tests) */
    link_sim_fn sim_fn;         /* NULL on the boat */
    void       *sim_ctx;
} link_t;

void link_init(link_t *l, sb_t *s);
void link_step(link_t *l, sb_t *s);

/* Parameter table (config editor). */
#define LINK_PARAM_NAME 16
typedef struct {
    const char *name;
    float       min, max;
} link_param_info_t;

uint8_t link_param_count(void);
const link_param_info_t *link_param_info(uint8_t id);   /* NULL if no such id */

#endif /* LINK_H */
