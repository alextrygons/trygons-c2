/*
 * dronecan.h - DroneCAN (UAVCAN v0) on the boat's CAN bus (CLAUDE.md section T,
 * docs/dronecan_design.md sections 4 + 5; step 11d). Pure C99, no iLLD: the
 * CAN controller is reached through a small port (dc_port_t), so the SAME code
 * runs on the TC375 (hal_tc375.c + hal/tc_can.c = MCMCAN CAN0 node 0) and in
 * the PC simulation (sim/hal_sim.c: a simulated controller + bus + nodes) and
 * in the host tests (tests/hal_pc/dronecan_check.py).
 *
 * What it does (core 0 only, one bounded step per control step: dc_service):
 *   - the bus state machine: RUN / PASSIVE / BUSOFF (recovery requested at
 *     once and every DC_RECOVER_MS) / FAULT (the controller did not start);
 *     off or error passive for longer than DC_BUS_FAULT_MS = bus fault = every
 *     node lost, and the software TX queue is dropped (counted) so no stale
 *     frame goes out later;
 *   - RX: at most DC_RX_PER_STEP frames per step into libcanard (MIT,
 *     hal/libcanard/, static memory pool, no malloc); a frame FROM our own node
 *     ID = ID conflict (the Pixhawk is node 10 too: OA-4);
 *   - TX: our NodeStatus at 1 Hz; the GetNodeInfo server (the DroneCAN GUI
 *     Tool sees the AURIX); at most DC_TX_PER_STEP frames per step into the
 *     controller FIFO (full = kept for the next step);
 *   - the node table (compile-time static IDs, TO CONFIRM, never from a boat
 *     file or the PC - R7): heartbeat age, health, mode, uptime, reboots; LOST
 *     rules of docs/dronecan_design.md section 5 (hal.h hal_can_node_t).
 *   - encoders / decoders of the messages of section 4: NodeStatus, ArrayCommand
 *     (UNITLESS, one command per transfer = one CAN frame), hardpoint.Command.
 *     Step 11d: dc_send_actuator / dc_send_hardpoint exist and are tested (sim
 *     nodes, host tests) but NO app code calls them yet (tools/check.sh) - the
 *     servos move to CAN in step 11e, the relay board in 11f.
 * Signatures and IDs are checked against values computed from the DSDL text by
 * tests/hal_pc/dronecan_check.py (CRC-64-WE, the DroneCAN rule).
 */
#ifndef DRONECAN_H
#define DRONECAN_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

/* --- the bus and the node table (TO CONFIRM: docs/dronecan_design.md section 12) */
#define DC_OWN_NODE_ID      10u     /* the AURIX (ArduPilot's CAN_D1_UC_NODE) */
#define DC_NODE_THROTTLE    20u     /* proposal, TO CONFIRM (set on the node with the DroneCAN GUI Tool) */
#define DC_NODE_STEERING    21u
#define DC_NODE_GEAR        22u
#define DC_NODE_RELAY       30u
#define DC_BITRATE          1000000u
#define DC_BUS_FAULT_MS     100u    /* bus-off / error passive longer than this = every node lost */
#define DC_RECOVER_MS       1000u   /* bus-off: recovery requested again after this */
#define DC_REBOOT_OK_MS     2000u   /* a rebooted node: OPERATIONAL this long before the ack clears the latch */
#define DC_CONFLICT_MS      3000u   /* our own node ID seen within this = ID conflict */
#define DC_ANON_MS          5000u   /* a dynamic-ID request seen within this = notice */
#define DC_STATUS_MS        1000u   /* our NodeStatus period (MAX_BROADCASTING_PERIOD_MS) */
#define DC_RX_PER_STEP      64u     /* frames taken per dc_service */
#define DC_TX_PER_STEP      32u     /* frames given to the controller per dc_service */
#define DC_POOL_BYTES       2048u   /* libcanard memory pool (TX queue + multi-frame RX) */

/* --- DroneCAN data types (standard IDs; signatures checked by dronecan_check.py) */
#define DC_NODESTATUS_ID    341u
#define DC_NODESTATUS_SIG   0x0F0868D0C1A7C6F1ull
#define DC_ARRAYCMD_ID      1010u
#define DC_ARRAYCMD_SIG     0xD8A7486238EC3AF3ull
#define DC_HARDPOINT_ID     1070u
#define DC_HARDPOINT_SIG    0xA1A036268B0C3455ull
#define DC_GETNODEINFO_ID   1u      /* service */
#define DC_GETNODEINFO_SIG  0xEE468A8121C46A9Eull
#define DC_ALLOCATION_ID    1u      /* message (anonymous dynamic node ID request) */
#define DC_ALLOCATION_SIG   0x0B2A812620A11D40ull

/* NodeStatus values */
#define DC_HEALTH_OK        0u
#define DC_HEALTH_WARNING   1u
#define DC_HEALTH_ERROR     2u
#define DC_HEALTH_CRITICAL  3u
#define DC_MODE_OPERATIONAL 0u
#define DC_MODE_INIT        1u
#define DC_MODE_MAINTENANCE 2u
#define DC_MODE_SW_UPDATE   3u
#define DC_MODE_OFFLINE     7u

/* actuator command types (uavcan.equipment.actuator.Command) */
#define DC_CMD_UNITLESS     0u
#define DC_CMD_POSITION     1u
#define DC_CMD_PWM          4u

/* transfer priorities (0 = highest; the CPU1 safety frames of step 11e get the top) */
#define DC_PRIO_ACTUATOR    8u
#define DC_PRIO_HARDPOINT   16u
#define DC_PRIO_STATUS      24u

/* one classic CAN frame, 29-bit identifier */
typedef struct {
    uint32_t id;            /* 29-bit extended ID (no flag bits) */
    uint8_t  len;           /* 0..8 */
    uint8_t  data[8];
} dc_frame_t;

/* controller states the port reports */
#define DC_CTRL_ACTIVE      0u      /* error active or warning */
#define DC_CTRL_PASSIVE     1u
#define DC_CTRL_BUSOFF      2u

typedef struct {
    bool     (*start)(void);                        /* start the controller (at dc_init): false = failed */
    bool     (*send)(const dc_frame_t *f);          /* into the TX FIFO, returns at once: false = full */
    bool     (*recv)(dc_frame_t *f);                /* one frame from the RX FIFO: false = empty */
    uint8_t  (*state)(uint8_t *tec, uint8_t *rec);  /* DC_CTRL_ */
    void     (*recover)(void);                      /* bus-off: start the recovery (returns at once) */
    uint32_t (*millis)(void);
} dc_port_t;

/* the compile-time node table (hal_can_node(i)): i = HAL_CAN_ROLE_ */
void dc_init(const dc_port_t *port);
void dc_service(void);
void dc_ack(void);
void dc_bus(hal_can_bus_t *b);
bool dc_node(uint8_t i, hal_can_node_t *n);
/* our own NodeStatus health / mode (default OK / OPERATIONAL) */
void dc_set_own_status(uint8_t health, uint8_t mode);

/* --- sending (step 11d: tests only; 11e servos, 11f relays) ------------------ */
/* one ArrayCommand with ONE command (4 bytes + tail = a single CAN frame):
 * false = not queued (no room / bus fault / bad value) */
bool dc_send_actuator(uint8_t actuator_id, uint8_t type, float value);
/* one hardpoint.Command (relay board): false = not queued */
bool dc_send_hardpoint(uint8_t hardpoint_id, uint16_t command);

/* --- pure encode / decode helpers (also used by the sim nodes + host tests) -- */
/* payload (no tail byte) sizes */
#define DC_NODESTATUS_LEN   7u
#define DC_HARDPOINT_LEN    3u
#define DC_NODEINFO_NAME    "org.trygons.c2.aurix"
typedef struct {
    uint32_t uptime_s;
    uint8_t  health, mode, sub_mode;
    uint16_t vendor;
} dc_status_t;
typedef struct {
    uint8_t actuator_id, type;
    float   value;
} dc_cmd_t;
void     dc_encode_status(uint8_t *out7, const dc_status_t *st);
bool     dc_decode_status(const uint8_t *p, uint32_t len, dc_status_t *st);
/* n commands (<= 15), 4 bytes each (tail array: no length prefix) */
uint32_t dc_encode_array_command(uint8_t *out, const dc_cmd_t *c, uint32_t n);
uint32_t dc_decode_array_command(const uint8_t *p, uint32_t len, dc_cmd_t *c, uint32_t max);
void     dc_encode_hardpoint(uint8_t *out3, uint8_t id, uint16_t command);
bool     dc_decode_hardpoint(const uint8_t *p, uint32_t len, uint8_t *id, uint16_t *command);
/* the GetNodeInfo response payload: returns its length (<= out size 128) */
uint32_t dc_encode_node_info(uint8_t *out, uint32_t max, const dc_status_t *st);
/* identifier of a single message frame + its tail byte (sim nodes) */
uint32_t dc_msg_id(uint8_t priority, uint16_t type_id, uint8_t src);
uint8_t  dc_tail(bool sot, bool eot, bool toggle, uint8_t tid);
/* source node of a frame identifier (0 = anonymous) */
uint8_t  dc_src(uint32_t id);
/* CRC-16-CCITT-FALSE of DroneCAN multi-frame transfers (seeded with the signature) */
uint16_t dc_crc16(uint16_t crc, const uint8_t *p, uint32_t n);
uint16_t dc_crc_signature(uint64_t sig);

/* counters for the host tests / the bench page */
typedef struct {
    uint32_t rx, tx, tx_drops, rx_errors, busoffs, recoveries, conflicts, anon;
    uint32_t nodeinfo_replies, pool_peak;
    uint8_t  state;         /* HAL_CAN_ST_ */
} dc_stats_t;
const dc_stats_t *dc_stats(void);

#endif /* DRONECAN_H */
