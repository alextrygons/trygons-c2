/*
 * link.h - Ground-station link: telemetry out, commands in (proto.h frames).
 *
 * Call link_step() once per control step, right after sb_step().
 *
 *   - 10 Hz telemetry frame (PROTO_T_TELEMETRY)
 *   - every command is checked by the same rules as the buttons (mode
 *     manager, logged as WHY_PC) and answered with ACK or NACK + reason
 *   - the PC can never take control from the joystick: in MANUAL every mode
 *     command is refused (PROTO_R_JOYSTICK). The PC can only request
 *     STANDBY (from HDG_LOCK / AUTO / TRAINING) and TRAINING (from
 *     STANDBY); AUTO needs PROTO_T_AUTO_START + the joystick CONFIRM.
 *     HDG_LOCK is engaged with the panel button only.
 *   - AUTO program upload: PROG_BEGIN, PROG_WP x n, PROG_COMMIT (validated
 *     and stored by the boat; a refusal also sends PROG_ERROR)
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

#define LINK_PORT_USB 0u      /* ground station on the USB cable (local laptop) */
#define LINK_PORT_GW  1u      /* Raspberry Pi gateway UART */

typedef struct {
    uint8_t     port;           /* LINK_PORT_USB / LINK_PORT_GW */
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
    program_t   staging;        /* program being uploaded */
    uint8_t     staging_n;      /* waypoints announced by PROG_BEGIN (0 = none) */
    uint64_t    staging_have;   /* bit per waypoint received */
    /* gateway port only */
    uint32_t    next_time_ms;   /* GPS time to the Pi, 1 Hz */
    uint32_t    log_len, log_off;   /* log record being sent to the Pi (bytes incl. the u32 number) */
    uint32_t    log_chunks;
    uint32_t    gw_used_after;  /* UART TX bytes waiting after the last step */
    uint32_t    gw_room;        /* bytes allowed to wait in the UART (follows its real speed) */
} link_t;

void link_init(link_t *l, sb_t *s);                         /* USB port */
void link_init_port(link_t *l, sb_t *s, uint8_t port);
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
