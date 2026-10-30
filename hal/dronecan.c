/*
 * dronecan.c - DroneCAN on the boat's CAN bus (see dronecan.h). Pure C99:
 * the controller is reached only through the port given to dc_init.
 */
#include "dronecan.h"

#include <math.h>
#include <string.h>

#include "libcanard/canard.h"

typedef struct {
    hal_can_node_t pub;
    uint32_t last_ms;
    uint32_t op_since_ms;       /* OPERATIONAL + healthy since (valid while op) */
    bool     op;
} dc_node_t;

static const dc_port_t *P;
static CanardInstance s_ins;
static union { uint8_t b[DC_POOL_BYTES]; uint64_t a; void *p; } s_pool;   /* aligned for the pool blocks */
static dc_node_t s_node[HAL_CAN_NODES];
static dc_stats_t s_st;
static bool     s_started;
static uint64_t s_us;                   /* monotonic microseconds (from the port's ms) */
static uint32_t s_last_ms, s_t0_ms;
static uint32_t s_status_ms, s_cleanup_ms;
static uint32_t s_bad_since_ms;         /* bus not RUN since (0 = RUN) */
static bool     s_bad;
static bool     s_was_busoff;
static bool     s_fault_cancelled;       /* the controller FIFO was cancelled at the start of this bus fault */
static uint32_t s_recover_ms;
static uint32_t s_last_rx_ms;
static uint32_t s_conflict_ms, s_anon_ms;
static bool     s_conflict_seen, s_anon_seen;
static uint8_t  s_tec, s_rec;
static uint8_t  s_own_health = DC_HEALTH_OK, s_own_mode = DC_MODE_OPERATIONAL;
static uint8_t  s_tid_status, s_tid_cmd, s_tid_hp;
static uint16_t s_hp_status[DC_HP_MAX];  /* the relay board's readback (hardpoint.Status from DC_NODE_RELAY) */
static uint32_t s_hp_ms[DC_HP_MAX];
static uint32_t s_hp_seen;               /* bit hp: a Status arrived since dc_init */

static const uint8_t k_ids[HAL_CAN_NODES]  = { DC_NODE_THROTTLE, DC_NODE_STEERING, DC_NODE_GEAR, DC_NODE_RELAY };
static const bool    k_req[HAL_CAN_NODES]  = { true, true, true, false };

/* --- pure helpers -------------------------------------------------------- */

uint16_t dc_crc16(uint16_t crc, const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint16_t dc_crc_signature(uint64_t sig)
{
    uint8_t b[8];
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)(sig >> (8 * i));
    return dc_crc16(0xFFFFu, b, 8u);
}

uint32_t dc_msg_id(uint8_t priority, uint16_t type_id, uint8_t src)
{
    return ((uint32_t)(priority & 0x1Fu) << 24) | ((uint32_t)type_id << 8) | (uint32_t)(src & 0x7Fu);
}

uint8_t dc_tail(bool sot, bool eot, bool toggle, uint8_t tid)
{
    return (uint8_t)((sot ? 0x80u : 0u) | (eot ? 0x40u : 0u) | (toggle ? 0x20u : 0u) | (tid & 0x1Fu));
}

uint8_t dc_src(uint32_t id)
{
    return (uint8_t)(id & 0x7Fu);
}

void dc_encode_status(uint8_t *out7, const dc_status_t *st)
{
    uint8_t h = (uint8_t)(st->health & 3u), m = (uint8_t)(st->mode & 7u), sm = (uint8_t)(st->sub_mode & 7u);
    memset(out7, 0, DC_NODESTATUS_LEN);
    canardEncodeScalar(out7, 0, 32, &st->uptime_s);
    canardEncodeScalar(out7, 32, 2, &h);
    canardEncodeScalar(out7, 34, 3, &m);
    canardEncodeScalar(out7, 37, 3, &sm);
    canardEncodeScalar(out7, 40, 16, &st->vendor);
}

bool dc_decode_status(const uint8_t *p, uint32_t len, dc_status_t *st)
{
    if (len < DC_NODESTATUS_LEN) return false;
    st->uptime_s = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    st->health = (uint8_t)(p[4] >> 6);
    st->mode = (uint8_t)((p[4] >> 3) & 7u);
    st->sub_mode = (uint8_t)(p[4] & 7u);
    st->vendor = (uint16_t)(p[5] | (p[6] << 8));
    return true;
}

uint32_t dc_encode_array_command(uint8_t *out, const dc_cmd_t *c, uint32_t n)
{
    if (n > 15u) n = 15u;
    memset(out, 0, 4u * n);
    for (uint32_t i = 0; i < n; i++) {
        uint16_t f16 = canardConvertNativeFloatToFloat16(c[i].value);
        canardEncodeScalar(out, 32u * i, 8, &c[i].actuator_id);
        canardEncodeScalar(out, 32u * i + 8u, 8, &c[i].type);
        canardEncodeScalar(out, 32u * i + 16u, 16, &f16);
    }
    return 4u * n;
}

uint32_t dc_decode_array_command(const uint8_t *p, uint32_t len, dc_cmd_t *c, uint32_t max)
{
    uint32_t n = len / 4u;
    if (n > 15u) n = 15u;
    if (n > max) n = max;
    for (uint32_t i = 0; i < n; i++) {
        c[i].actuator_id = p[4u * i];
        c[i].type = p[4u * i + 1u];
        c[i].value = canardConvertFloat16ToNativeFloat((uint16_t)(p[4u * i + 2u] | (p[4u * i + 3u] << 8)));
    }
    return n;
}

void dc_encode_hardpoint(uint8_t *out3, uint8_t id, uint16_t command)
{
    memset(out3, 0, DC_HARDPOINT_LEN);
    canardEncodeScalar(out3, 0, 8, &id);
    canardEncodeScalar(out3, 8, 16, &command);
}

bool dc_decode_hardpoint(const uint8_t *p, uint32_t len, uint8_t *id, uint16_t *command)
{
    if (len < DC_HARDPOINT_LEN) return false;
    *id = p[0];
    *command = (uint16_t)(p[1] | (p[2] << 8));
    return true;
}

void dc_encode_hp_status(uint8_t *out7, uint8_t id, uint16_t status)
{
    uint16_t w = canardConvertNativeFloatToFloat16(0.0f);
    memset(out7, 0, DC_HPSTATUS_LEN);
    canardEncodeScalar(out7, 0, 8, &id);
    canardEncodeScalar(out7, 8, 16, &w);
    canardEncodeScalar(out7, 24, 16, &w);
    canardEncodeScalar(out7, 40, 16, &status);
}

bool dc_decode_hp_status(const uint8_t *p, uint32_t len, uint8_t *id, uint16_t *status)
{
    if (len < DC_HPSTATUS_LEN) return false;
    *id = p[0];
    *status = (uint16_t)(p[5] | (p[6] << 8));
    return true;
}

uint32_t dc_encode_node_info(uint8_t *out, uint32_t max, const dc_status_t *st)
{
    static const char name[] = DC_NODEINFO_NAME;
    uint32_t nl = (uint32_t)sizeof name - 1u;
    if (max < 41u + nl) return 0u;
    memset(out, 0, 41u);
    dc_encode_status(out, st);              /* status */
    out[7] = 1u;                            /* software_version.major */
    out[8] = 0u;                            /* .minor; .optional_field_flags 0, vcs_commit 0, image_crc 0 */
    /* hardware_version: major 0, minor 0, unique_id[16] zeros (TO CONFIRM: the chip's UID), certificate
     * length 0 (a dynamic array that is not the last field: 8-bit length) */
    memcpy(out + 41, name, nl);             /* name: the last field (tail array, no length) */
    return 41u + nl;
}

const dc_stats_t *dc_stats(void)
{
    return &s_st;
}

/* --- libcanard callbacks --------------------------------------------------- */

static uint32_t now_ms(void)
{
    return s_last_ms;
}

static void on_status(uint8_t src, const dc_status_t *st)
{
    for (uint32_t i = 0; i < HAL_CAN_NODES; i++) {
        dc_node_t *n = &s_node[i];
        if (n->pub.id != src) continue;
        if (n->pub.seen && st->uptime_s < n->pub.uptime_s) {    /* the node rebooted: its outputs went through
                                                                   its own boot state - latched */
            n->pub.rebooted = true;
            n->pub.restarts++;
            n->pub.back_s = n->pub.uptime_s - st->uptime_s;     /* step 13k (W44): can_relay.c tells a small step
                                                                   (maybe a stale status) from a proven reboot */
            n->op = false;
        }
        n->pub.seen = true;
        n->pub.health = st->health;
        n->pub.mode = st->mode;
        n->pub.uptime_s = st->uptime_s;
        n->pub.statuses++;
        n->last_ms = now_ms();
        bool ok = st->mode == DC_MODE_OPERATIONAL && st->health <= DC_HEALTH_WARNING;
        if (ok && !n->op) n->op_since_ms = now_ms();
        n->op = ok;
        return;
    }
}

static bool should_accept(const CanardInstance *ins, uint64_t *sig, uint16_t type_id, CanardTransferType tt,
                          uint8_t src)
{
    (void)ins;
    if (tt == CanardTransferTypeBroadcast && type_id == DC_NODESTATUS_ID) {
        *sig = DC_NODESTATUS_SIG;
        return true;
    }
    if (tt == CanardTransferTypeBroadcast && type_id == DC_ALLOCATION_ID && src == 0u) {
        *sig = DC_ALLOCATION_SIG;
        return true;
    }
    if (tt == CanardTransferTypeRequest && type_id == DC_GETNODEINFO_ID) {
        *sig = DC_GETNODEINFO_SIG;
        return true;
    }
    if (tt == CanardTransferTypeBroadcast && type_id == DC_HPSTATUS_ID && src == DC_NODE_RELAY) {
        *sig = DC_HPSTATUS_SIG;             /* the relay board's readback only (step 11f) */
        return true;
    }
    return false;
}

static void own_status(dc_status_t *st)
{
    st->uptime_s = (uint32_t)(s_us / 1000000u);
    st->health = s_own_health;
    st->mode = s_own_mode;
    st->sub_mode = 0u;
    st->vendor = 0u;
}

static void on_reception(CanardInstance *ins, CanardRxTransfer *t)
{
    if (t->transfer_type == CanardTransferTypeBroadcast && t->data_type_id == DC_NODESTATUS_ID) {
        dc_status_t st;
        if (t->payload_len >= DC_NODESTATUS_LEN && dc_decode_status(t->payload_head, t->payload_len, &st))
            on_status(t->source_node_id, &st);
        return;
    }
    if (t->transfer_type == CanardTransferTypeBroadcast && t->data_type_id == DC_HPSTATUS_ID) {
        uint8_t hp;
        uint16_t v;
        if (t->source_node_id == DC_NODE_RELAY && t->payload_len >= DC_HPSTATUS_LEN &&
            dc_decode_hp_status(t->payload_head, t->payload_len, &hp, &v) && hp < DC_HP_MAX) {
            s_hp_status[hp] = v;
            s_hp_ms[hp] = now_ms();
            s_hp_seen |= 1u << hp;
            s_st.hp_status++;
        }
        return;
    }
    if (t->transfer_type == CanardTransferTypeBroadcast && t->data_type_id == DC_ALLOCATION_ID) {
        s_anon_ms = now_ms();           /* a node without a static ID: never answered (no ID server) */
        s_anon_seen = true;
        s_st.anon++;
        return;
    }
    if (t->transfer_type == CanardTransferTypeRequest && t->data_type_id == DC_GETNODEINFO_ID) {
        uint8_t buf[128];
        dc_status_t st;
        own_status(&st);
        uint32_t n = dc_encode_node_info(buf, sizeof buf, &st);
        uint8_t tid = t->transfer_id;
        uint8_t src = t->source_node_id, prio = t->priority;
        canardReleaseRxTransferPayload(ins, t);
        if (n && canardRequestOrRespond(ins, src, DC_GETNODEINFO_SIG, DC_GETNODEINFO_ID, &tid, prio, CanardResponse,
                                        buf, (uint16_t)n) >= 0)
            s_st.nodeinfo_replies++;
        else
            s_st.tx_drops++;
    }
}

/* --- the bus ---------------------------------------------------------------- */

static void drop_queue(void)
{
    while (canardPeekTxQueue(&s_ins) != NULL) {
        canardPopTxQueue(&s_ins);
        s_st.tx_drops++;
    }
}

/* a dead bus / a trip: nothing stale in the software queue AND nothing in the controller's TX FIFO
 * (step 11e: the hardware FIFO is cancelled too, so no old ArrayCommand goes out late) */
static void drop_all(void)
{
    drop_queue();
    if (P != NULL && P->cancel != NULL) P->cancel();
}

static bool bus_fault(void)
{
    return s_st.state == HAL_CAN_ST_FAULT || s_st.state == HAL_CAN_ST_OFF ||
           (s_bad && s_last_ms - s_bad_since_ms > DC_BUS_FAULT_MS);
}

void dc_init(const dc_port_t *port)
{
    P = port;
    memset(&s_st, 0, sizeof s_st);
    memset(s_node, 0, sizeof s_node);
    for (uint32_t i = 0; i < HAL_CAN_NODES; i++) {
        s_node[i].pub.id = k_ids[i];
        s_node[i].pub.role = (uint8_t)i;
        s_node[i].pub.required = k_req[i];
        s_node[i].pub.why = HAL_CAN_OK;
        s_node[i].pub.age_ms = 0xFFFFFFFFu;
    }
    canardInit(&s_ins, s_pool.b, sizeof s_pool.b, on_reception, should_accept, NULL);
    canardSetLocalNodeID(&s_ins, (uint8_t)DC_OWN_NODE_ID);
    s_last_ms = s_t0_ms = P->millis();
    s_us = 0u;
    s_status_ms = s_t0_ms - DC_STATUS_MS;       /* our first NodeStatus at the first service */
    s_cleanup_ms = s_t0_ms;
    s_bad = false;
    s_was_busoff = false;
    s_fault_cancelled = false;
    s_last_rx_ms = s_t0_ms;
    s_conflict_seen = s_anon_seen = false;
    s_tec = s_rec = 0u;
    s_own_health = DC_HEALTH_OK;
    s_own_mode = DC_MODE_OPERATIONAL;
    memset(s_hp_status, 0, sizeof s_hp_status);
    memset(s_hp_ms, 0, sizeof s_hp_ms);
    s_hp_seen = 0u;
    s_started = P->start();
    s_st.state = s_started ? HAL_CAN_ST_RUN : HAL_CAN_ST_FAULT;
}

void dc_set_own_status(uint8_t health, uint8_t mode)
{
    s_own_health = (uint8_t)(health & 3u);
    s_own_mode = (uint8_t)(mode & 7u);
}

static void bus_state(void)
{
    if (!s_started) {
        s_st.state = HAL_CAN_ST_FAULT;
        return;
    }
    uint8_t c = P->state(&s_tec, &s_rec);
    if (c == DC_CTRL_BUSOFF) {
        if (!s_was_busoff) {                    /* recovery at once (the controller waits 128 x 11 recessive bits) */
            s_st.busoffs++;
            s_recover_ms = s_last_ms;
            P->recover();
        } else if (s_last_ms - s_recover_ms >= DC_RECOVER_MS) {
            s_recover_ms = s_last_ms;
            s_st.recoveries++;
            P->recover();
        }
        s_was_busoff = true;
        s_st.state = HAL_CAN_ST_BUSOFF;
    } else {
        s_was_busoff = false;
        s_st.state = (c == DC_CTRL_PASSIVE) ? HAL_CAN_ST_PASSIVE : HAL_CAN_ST_RUN;
    }
    bool bad = s_st.state != HAL_CAN_ST_RUN;
    if (bad && !s_bad) s_bad_since_ms = s_last_ms;
    s_bad = bad;
}

static void nodes_eval(void)
{
    bool fault = bus_fault();
    for (uint32_t i = 0; i < HAL_CAN_NODES; i++) {
        dc_node_t *n = &s_node[i];
        hal_can_node_t *p = &n->pub;
        uint32_t age = p->seen ? s_last_ms - n->last_ms : 0xFFFFFFFFu;
        p->age_ms = age;
        uint8_t why = HAL_CAN_OK;
        if (fault)                                      why = HAL_CAN_BUS;
        else if (!p->seen)                              why = (s_last_ms - s_t0_ms >= HAL_CAN_NODE_LOST_MS) ? HAL_CAN_NEVER
                                                                                                         : HAL_CAN_OK;
        else if (age > HAL_CAN_NODE_LOST_MS)            why = HAL_CAN_SILENT;
        else if (p->rebooted)                           why = HAL_CAN_REBOOTED;
        else if (p->mode != DC_MODE_OPERATIONAL)        why = HAL_CAN_NOT_OPER;
        else if (p->health >= DC_HEALTH_ERROR)          why = HAL_CAN_HEALTH;
        if (age > HAL_CAN_NODE_LOST_MS) n->op = false;
        p->why = why;
        p->lost = why != HAL_CAN_OK;
        p->warning = !p->lost && p->seen && p->health == DC_HEALTH_WARNING;
    }
}

void dc_service(void)
{
    if (P == NULL) return;
    uint32_t t = P->millis();
    s_us += (uint64_t)(uint32_t)(t - s_last_ms) * 1000u;
    s_last_ms = t;
    bus_state();

    /* RX: a bounded number of frames per step */
    if (s_started) {
        dc_frame_t f;
        for (uint32_t k = 0; k < DC_RX_PER_STEP && P->recv(&f); k++) {
            s_st.rx++;
            s_last_rx_ms = t;
            if (f.len == 0u || f.len > 8u) {
                s_st.rx_errors++;
                continue;
            }
            if (dc_src(f.id) == DC_OWN_NODE_ID) {   /* another node with OUR static ID (the Pixhawk: OA-4) */
                s_conflict_ms = t;
                s_conflict_seen = true;
                s_st.conflicts++;
                continue;
            }
            CanardCANFrame cf;
            memset(&cf, 0, sizeof cf);
            cf.id = (f.id & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
            cf.data_len = f.len;
            memcpy(cf.data, f.data, f.len);
            int16_t r = canardHandleRxFrame(&s_ins, &cf, s_us);
            if (r < 0 && r != -CANARD_ERROR_RX_NOT_WANTED && r != -CANARD_ERROR_RX_WRONG_ADDRESS)
                s_st.rx_errors++;
        }
    }
    if (t - s_cleanup_ms >= 1000u) {
        s_cleanup_ms = t;
        canardCleanupStaleTransfers(&s_ins, s_us);
    }

    nodes_eval();

    /* our NodeStatus, 1 Hz (also while the bus is bad: it is the frame that brings the TEC back down) */
    if (s_started && t - s_status_ms >= DC_STATUS_MS) {
        s_status_ms = t;
        uint8_t buf[DC_NODESTATUS_LEN];
        dc_status_t st;
        own_status(&st);
        dc_encode_status(buf, &st);
        if (bus_fault()) drop_queue();          /* a dead bus: nothing stale waits in the queue */
        if (canardBroadcast(&s_ins, DC_NODESTATUS_SIG, DC_NODESTATUS_ID, &s_tid_status, DC_PRIO_STATUS, buf,
                            DC_NODESTATUS_LEN) < 0)
            s_st.tx_drops++;
    }

    /* TX: queued frames into the controller FIFO (full = they wait for the next step) */
    dc_tx_flush();
    if (bus_fault()) {
        drop_queue();
        /* the bus just went bad: cancel what waits in the controller's TX FIFO once (step 11e: no stale
         * ArrayCommand goes out late; dc_send_actuator refuses while the fault lasts, only our NodeStatus is
         * queued meanwhile - it is the frame that brings the error counter down again) */
        if (!s_fault_cancelled && P->cancel != NULL) P->cancel();
        s_fault_cancelled = true;
    } else {
        s_fault_cancelled = false;
    }
    CanardPoolAllocatorStatistics ps = canardGetPoolAllocatorStatistics(&s_ins);
    if (ps.peak_usage_blocks > s_st.pool_peak) s_st.pool_peak = ps.peak_usage_blocks;
}

void dc_ack(void)
{
    for (uint32_t i = 0; i < HAL_CAN_NODES; i++) {
        dc_node_t *n = &s_node[i];
        if (n->pub.rebooted && n->op && s_last_ms - n->last_ms <= HAL_CAN_NODE_LOST_MS &&
            s_last_ms - n->op_since_ms >= DC_REBOOT_OK_MS)
            n->pub.rebooted = false;
    }
    nodes_eval();
}

void dc_bus(hal_can_bus_t *b)
{
    memset(b, 0, sizeof *b);
    b->state = s_st.state;
    b->fault = bus_fault();
    b->id_conflict = s_conflict_seen && s_last_ms - s_conflict_ms < DC_CONFLICT_MS;
    b->anon_node = s_anon_seen && s_last_ms - s_anon_ms < DC_ANON_MS;
    b->silent = s_last_ms - s_last_rx_ms > HAL_CAN_NODE_LOST_MS;
    b->tec = s_tec;
    b->rec = s_rec;
    b->rx = s_st.rx;
    b->tx = s_st.tx;
    b->tx_drops = s_st.tx_drops;
    b->rx_errors = s_st.rx_errors;
    b->busoffs = s_st.busoffs;
}

bool dc_node(uint8_t i, hal_can_node_t *n)
{
    if (i >= HAL_CAN_NODES) return false;
    *n = s_node[i].pub;
    return true;
}

void dc_tx_flush(void)
{
    if (P == NULL || !s_started || s_st.state == HAL_CAN_ST_BUSOFF) return;
    for (uint32_t k = 0; k < DC_TX_PER_STEP; k++) {
        const CanardCANFrame *q = canardPeekTxQueue(&s_ins);
        if (q == NULL) break;
        dc_frame_t f;
        f.id = q->id & CANARD_CAN_EXT_ID_MASK;
        f.len = q->data_len;
        memcpy(f.data, q->data, q->data_len);
        if (!P->send(&f)) break;
        canardPopTxQueue(&s_ins);
        s_st.tx++;
    }
}

void dc_cancel_tx(void)
{
    if (P != NULL) drop_all();
}

uint8_t dc_cmd_tid(void)
{
    return (uint8_t)(s_tid_cmd & 0x1Fu);
}

static bool can_send(void)
{
    return P != NULL && s_started && !bus_fault();
}

bool dc_send_actuator(uint8_t actuator_id, uint8_t type, float value)
{
    if (!can_send() || !isfinite(value)) return false;
    if (type == DC_CMD_UNITLESS) value = value > 1.0f ? 1.0f : (value < -1.0f ? -1.0f : value);
    dc_cmd_t c = { actuator_id, type, value };
    uint8_t buf[4];
    (void)dc_encode_array_command(buf, &c, 1u);
    if (canardBroadcast(&s_ins, DC_ARRAYCMD_SIG, DC_ARRAYCMD_ID, &s_tid_cmd, DC_PRIO_ACTUATOR, buf, 4u) < 0) {
        s_st.tx_drops++;
        return false;
    }
    return true;
}

bool dc_send_hardpoint(uint8_t hardpoint_id, uint16_t command)
{
    if (!can_send()) return false;
    uint8_t buf[DC_HARDPOINT_LEN];
    dc_encode_hardpoint(buf, hardpoint_id, command);
    if (canardBroadcast(&s_ins, DC_HARDPOINT_SIG, DC_HARDPOINT_ID, &s_tid_hp, DC_PRIO_HARDPOINT, buf,
                        DC_HARDPOINT_LEN) < 0) {
        s_st.tx_drops++;
        return false;
    }
    return true;
}

bool dc_hardpoint_status(uint8_t hp, uint16_t *status, uint32_t *age_ms)
{
    if (hp >= DC_HP_MAX || !(s_hp_seen & (1u << hp))) return false;
    if (status) *status = s_hp_status[hp];
    if (age_ms) *age_ms = s_last_ms - s_hp_ms[hp];
    return true;
}
