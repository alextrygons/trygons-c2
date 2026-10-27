/*
 * relays.c - The relay board as the boat sees it. See relays.h.
 * The ONLY caller of hal_relay_* (tools/check.sh checks it).
 */
#include "relays.h"
#include "hal.h"
#include "proto.h"

#include <stddef.h>
#include <string.h>

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

const relay_def_t *relay_table_find(const relay_table_t *t, uint8_t n)
{
    for (uint8_t i = 0; i < t->count && i < RELAY_TABLE_MAX; i++) if (t->r[i].n == n) return &t->r[i];
    return NULL;
}

/* the checks of the boat file (groundstation/boats.py), again on the AURIX (R7) */
static bool table_consistent(const relay_table_t *t)
{
    if (t->count > RELAY_TABLE_MAX) return false;
    for (uint8_t i = 0; i < t->count; i++) {
        const relay_def_t *d = &t->r[i];
        if (d->n == 0u || d->n > RELAY_N_MAX || d->type > RELAY_TIMED || d->group > RELAY_GRP_EXTRA) return false;
        if ((d->flags & ~(RELAY_F_SHOW_HUD | RELAY_F_POWER_ON | RELAY_F_KEEP)) != 0u || d->pad != 0u) return false;
        for (uint8_t j = 0; j < i; j++) if (t->r[j].n == d->n) return false;          /* n twice */
        if (d->type == RELAY_MOMENTARY && (d->max_ds == 0u || d->max_ds > RELAY_PULSE_MAX_MS / 100u)) return false;
        if (d->type == RELAY_TIMED && d->max_ds == 0u) return false;
        if (d->group == RELAY_GRP_ENGINE && (d->flags & (RELAY_F_POWER_ON | RELAY_F_KEEP))) return false;
        if (d->pair != 0u) {                                                           /* mutual, another relay */
            const relay_def_t *p = relay_table_find(t, d->pair);
            if (p == NULL || p == d || p->pair != d->n) return false;
        }
    }
    return true;
}

static uint32_t table_crc(const relay_table_t *t)
{
    return crc32((const uint8_t *)t, offsetof(relay_table_t, crc));
}

bool relay_table_valid(const relay_table_t *t)
{
    return t->version == RELAY_TABLE_VERSION && t->pad[0] == 0u && t->pad[1] == 0u && table_consistent(t) &&
           t->crc == table_crc(t);
}

void relay_table_seal(relay_table_t *t)
{
    t->version = RELAY_TABLE_VERSION;
    t->pad[0] = t->pad[1] = 0u;
    for (uint8_t i = 0; i < RELAY_TABLE_MAX; i++) {
        if (i >= t->count) memset(&t->r[i], 0, sizeof t->r[i]);     /* unused entries all zero: one encoding */
        else t->r[i].pad = 0u;
    }
    t->crc = table_crc(t);
}

uint32_t relay_table_encode(const relay_table_t *t, uint8_t *out, uint32_t max)
{
    uint32_t n = 1u + (uint32_t)t->count * RELAY_FRAME_BYTES;
    if (t->count > RELAY_TABLE_MAX || n > max) return 0u;
    out[0] = t->count;
    for (uint8_t i = 0; i < t->count; i++) {
        uint8_t *p = out + 1u + (uint32_t)i * RELAY_FRAME_BYTES;
        p[0] = t->r[i].n; p[1] = t->r[i].type; p[2] = t->r[i].group; p[3] = t->r[i].pair;
        p[4] = (uint8_t)t->r[i].max_ds; p[5] = (uint8_t)(t->r[i].max_ds >> 8); p[6] = t->r[i].flags;
    }
    return n;
}

bool relay_table_decode(const uint8_t *in, uint32_t len, relay_table_t *t)
{
    memset(t, 0, sizeof *t);
    if (len < 1u || in[0] > RELAY_TABLE_MAX || len != 1u + (uint32_t)in[0] * RELAY_FRAME_BYTES) return false;
    t->count = in[0];
    for (uint8_t i = 0; i < t->count; i++) {
        const uint8_t *p = in + 1u + (uint32_t)i * RELAY_FRAME_BYTES;
        t->r[i].n = p[0]; t->r[i].type = p[1]; t->r[i].group = p[2]; t->r[i].pair = p[3];
        t->r[i].max_ds = (uint16_t)(p[4] | ((uint16_t)p[5] << 8)); t->r[i].flags = p[6];
    }
    relay_table_seal(t);
    return table_consistent(t);
}

uint16_t relay_table_hash(const relay_table_t *t)
{
    uint8_t buf[1u + RELAY_TABLE_MAX * RELAY_FRAME_BYTES];
    uint32_t n = relay_table_encode(t, buf, sizeof buf);
    return proto_crc16(buf, n);
}

static void table_activate(relays_t *r, const relay_table_t *t, bool from_nv)
{
    r->table = *t;
    r->have_table = true;
    r->from_nv = from_nv;
    r->hash = relay_table_hash(t);
    r->cmd_on = 0u;
    r->off_pending = 0u;
    memset(r->until_ms, 0, sizeof r->until_ms);
    r->last_what = 2u;
    r->log_seq++;
}

void relays_init(relays_t *r, const relay_table_t *stored, bool power_on_reset, bool first_test)
{
    relay_table_t empty;
    memset(r, 0, sizeof *r);
    r->first_test = first_test;
    r->power_on_reset = power_on_reset;
    /* power_up only after the AURIX's own power-on reset; never in FIRST TEST */
    r->power_up_done = !power_on_reset || first_test;
    memset(&empty, 0, sizeof empty);
    relay_table_seal(&empty);
    r->hash = relay_table_hash(&empty);                 /* "no table" = the empty table's hash */
    if (stored && relay_table_valid(stored)) table_activate(r, stored, true);
}

relay_offer_t relays_offer(relays_t *r, const relay_table_t *t, bool standby_idle)
{
    if (!relay_table_valid(t)) return RELAY_OFFER_BAD;
    uint16_t h = relay_table_hash(t);
    if (h == r->hash && (r->have_table || t->count == 0u) && memcmp(&r->table.r, &t->r, sizeof t->r) == 0 &&
        r->table.count == t->count) {
        r->offer_pending = false;                        /* the same table: nothing to confirm */
        return RELAY_OFFER_SAME;
    }
    if (!r->have_table && t->count == 0u) { r->offer_pending = false; return RELAY_OFFER_SAME; }
    if (!standby_idle) return RELAY_OFFER_STANDBY;
    r->offer = *t;
    r->offer_pending = true;
    r->last_what = 2u;
    r->log_seq++;
    return RELAY_OFFER_PENDING;
}

bool relays_accept(relays_t *r, bool standby_idle)
{
    if (!r->offer_pending || !standby_idle) return false;
    r->offer_pending = false;
    table_activate(r, &r->offer, false);
    r->table_new = true;
    return true;
}

void relays_cancel_offer(relays_t *r)
{
    if (r->offer_pending) { r->last_what = 2u; r->log_seq++; }
    r->offer_pending = false;
}

static bool relay_on(const relays_t *r, uint8_t n)
{
    uint32_t bit = 1u << (n - 1u);
    return (r->cmd_on & bit) != 0u || ((r->known & bit) && (r->states & bit));
}

static relay_result_t cmd_result(relays_t *r, uint8_t n, uint8_t action, relay_result_t res)
{
    r->last_n = n;
    r->last_action = action;
    r->last_result = (uint8_t)res;
    r->last_what = 1u;
    r->log_seq++;
    if (res == RELAY_R_OK) r->commands++; else r->refused++;
    return res;
}

relay_result_t relays_cmd(relays_t *r, uint8_t n, uint8_t action, uint16_t ms, uint32_t now_ms)
{
    if (r->first_test) return cmd_result(r, n, action, RELAY_R_FIRST_TEST);
    if (!r->have_table || r->table.count == 0u) return cmd_result(r, n, action, RELAY_R_NO_TABLE);
    const relay_def_t *d = relay_table_find(&r->table, n);
    if (d == NULL || action > RELAY_CMD_PULSE) return cmd_result(r, n, action, RELAY_R_UNKNOWN);
    uint8_t idx = (uint8_t)(d - r->table.r);
    uint32_t bit = 1u << (n - 1u);
    if (action == RELAY_CMD_OFF) {
        /* OFF is always allowed to be asked (a silent board: queued, goes when it is back) */
        r->cmd_on &= ~bit;
        r->until_ms[idx] = 0u;
        r->off_pending |= bit;                          /* re-sent until the readback shows it off */
        if (!hal_relay_set(n, false)) return cmd_result(r, n, action, RELAY_R_HAL);
        return cmd_result(r, n, action, RELAY_R_OK);
    }
    if (d->group == RELAY_GRP_ENGINE && d->type == RELAY_MOMENTARY)      /* the STARTER: section N only */
        return cmd_result(r, n, action, RELAY_R_ENGINE);
    if (!r->board_ok) return cmd_result(r, n, action, RELAY_R_SILENT);
    if (d->pair != 0u && relay_on(r, d->pair)) return cmd_result(r, n, action, RELAY_R_PAIR);
    if (d->type == RELAY_MOMENTARY) {
        uint32_t len = (uint32_t)d->max_ds * 100u;
        if (action == RELAY_CMD_PULSE && ms != 0u && ms < len) len = ms;
        if (len > RELAY_PULSE_MAX_MS) len = RELAY_PULSE_MAX_MS;
        if (len < 100u) len = 100u;
        if (!hal_relay_pulse(n, (uint16_t)len)) return cmd_result(r, n, action, RELAY_R_HAL);
        r->cmd_on |= bit;
        r->off_pending &= ~bit;
        r->until_ms[idx] = (now_ms + len) ? now_ms + len : 1u;    /* the AURIX's own copy of the board timer */
        r->pulses++;
        return cmd_result(r, n, action, RELAY_R_OK);
    }
    if (!hal_relay_set(n, true)) return cmd_result(r, n, action, RELAY_R_HAL);
    r->cmd_on |= bit;
    r->off_pending &= ~bit;
    r->until_ms[idx] = (d->type == RELAY_TIMED) ? ((now_ms + (uint32_t)d->max_ds * 100u) ? now_ms + (uint32_t)d->max_ds * 100u : 1u)
                                                : 0u;
    return cmd_result(r, n, action, RELAY_R_OK);
}

void relays_step(relays_t *r, uint32_t now_ms)
{
    hal_relay_service();
    r->board_ok = hal_relay_ok();
    /* readback of every relay of the table */
    r->known = 0u;
    r->states = 0u;
    for (uint8_t i = 0; r->have_table && i < r->table.count; i++) {
        bool on = false;
        uint8_t n = r->table.r[i].n;
        if (hal_relay_read(n, &on)) {
            r->known |= 1u << (n - 1u);
            if (on) r->states |= 1u << (n - 1u);
        }
    }
    /* time limits: a timed relay OFF after max_s (also while silent: queued);
     * a momentary relay's pulse is over (the board switched it off) */
    for (uint8_t i = 0; r->have_table && i < r->table.count; i++) {
        const relay_def_t *d = &r->table.r[i];
        uint32_t bit = 1u << (d->n - 1u);
        if (r->until_ms[i] == 0u || (int32_t)(now_ms - r->until_ms[i]) < 0) continue;
        r->until_ms[i] = 0u;
        r->cmd_on &= ~bit;
        if (d->type == RELAY_TIMED) {
            r->off_pending |= bit;                      /* also while the board is silent: re-sent when it is back */
            (void)hal_relay_set(d->n, false);
            r->timed_offs++;
            r->last_n = d->n;
            r->last_action = RELAY_CMD_OFF;
            r->last_result = (uint8_t)RELAY_R_OK;
            r->last_what = 1u;
            r->log_seq++;
        }
    }
    /* pending OFFs: done once the readback shows the relay off; re-sent while the
     * board answers and still shows it on (a write lost while it was silent) */
    for (uint8_t i = 0; r->have_table && r->off_pending && i < r->table.count; i++) {
        uint32_t bit = 1u << (r->table.r[i].n - 1u);
        if ((r->off_pending & bit) && (r->known & bit) && !(r->states & bit)) r->off_pending &= ~bit;
    }
    if (r->off_pending && r->board_ok && (int32_t)(now_ms - r->off_retry_ms) >= 0) {
        r->off_retry_ms = now_ms + RELAY_OFF_RETRY_MS;
        for (uint8_t i = 0; r->have_table && i < r->table.count; i++) {
            uint8_t n = r->table.r[i].n;
            uint32_t bit = 1u << (n - 1u);
            if ((r->off_pending & bit) && (r->known & bit) && (r->states & bit) && !(r->cmd_on & bit))
                (void)hal_relay_set(n, false);
        }
    }
    /* power_up: once, with the first active table after a power-on reset, once the board
     * really answered (a valid readback of every relay of the table - mb_ok alone is also
     * true in the first seconds after the start, before any reply) */
    uint32_t mask = 0u;
    for (uint8_t i = 0; r->have_table && i < r->table.count; i++) mask |= 1u << (r->table.r[i].n - 1u);
    if (!r->power_up_done && r->have_table && r->board_ok && (r->known & mask) == mask) {
        r->power_up_done = true;
        for (uint8_t i = 0; i < r->table.count; i++) {
            const relay_def_t *d = &r->table.r[i];
            if (d->group == RELAY_GRP_ENGINE || !(d->flags & RELAY_F_POWER_ON)) continue;   /* never the engine */
            if (d->type == RELAY_MOMENTARY) continue;                    /* a pulse at power-up makes no sense */
            if (d->pair != 0u && relay_on(r, d->pair)) continue;         /* never both of a pair */
            if (r->states & (1u << (d->n - 1u))) continue;              /* already on: nothing to do */
            if (hal_relay_set(d->n, true)) {
                r->cmd_on |= 1u << (d->n - 1u);
                if (d->type == RELAY_TIMED) r->until_ms[i] = (now_ms + (uint32_t)d->max_ds * 100u) ? now_ms + (uint32_t)d->max_ds * 100u : 1u;
                r->power_ups++;
            }
        }
        r->last_what = 2u;
        r->log_seq++;
    }
}

bool relays_silent(const relays_t *r)
{
    return r->have_table && r->table.count > 0u && !r->board_ok;
}

uint8_t relays_flags(const relays_t *r)
{
    return (uint8_t)((r->board_ok ? 1u : 0u) | (r->have_table ? 2u : 0u) | (r->from_nv ? 4u : 0u) |
                     (r->offer_pending ? 8u : 0u) | (r->power_ups ? 16u : 0u) | (r->first_test ? 32u : 0u));
}

uint8_t relays_starter(const relays_t *r)
{
    for (uint8_t i = 0; r->have_table && i < r->table.count; i++)
        if (r->table.r[i].group == RELAY_GRP_ENGINE && r->table.r[i].type == RELAY_MOMENTARY) return r->table.r[i].n;
    return 0u;
}

relay_result_t relays_starter_pulse(relays_t *r, uint16_t ms, uint32_t now_ms)
{
    uint8_t n = relays_starter(r);
    if (r->first_test) return RELAY_R_FIRST_TEST;
    if (n == 0u) return RELAY_R_NO_TABLE;
    if (!r->board_ok) return RELAY_R_SILENT;
    const relay_def_t *d = relay_table_find(&r->table, n);
    uint32_t len = ms;
    if (len > (uint32_t)d->max_ds * 100u) len = (uint32_t)d->max_ds * 100u;
    if (len > RELAY_PULSE_MAX_MS) len = RELAY_PULSE_MAX_MS;
    if (len < 100u) len = 100u;
    if (!hal_relay_pulse(n, (uint16_t)len)) return RELAY_R_HAL;
    uint32_t bit = 1u << (n - 1u);
    r->cmd_on |= bit;
    r->off_pending &= ~bit;
    r->until_ms[d - r->table.r] = (now_ms + len) ? now_ms + len : 1u;
    r->pulses++;
    return RELAY_R_OK;                                  /* (logged by engine.c: start / stop, not every refresh) */
}

void relays_starter_off(relays_t *r)
{
    uint8_t n = relays_starter(r);
    if (n == 0u) return;
    uint32_t bit = 1u << (n - 1u);
    const relay_def_t *d = relay_table_find(&r->table, n);
    r->cmd_on &= ~bit;
    r->until_ms[d - r->table.r] = 0u;
    r->off_pending |= bit;                              /* re-sent until the readback shows it off */
    (void)hal_relay_set(n, false);
}
