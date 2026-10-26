/*
 * tc_nv.c - Append-only record log on the data flash (see tc_nv.h).
 * Pure C99: the flash is reached only through the port given to nvlog_init.
 */
#include "tc_nv.h"

#include <string.h>

#define NV_MAGIC   0x4E595254u          /* "TRYN" */
#define NV_FMT     1u                   /* record format version */
#define NV_HDR     16u                  /* pages 0 + 1 */
#define NV_TRL     8u                   /* trailer page */
#define NV_MAX_TRIES 3u                 /* writes of one record before NV FAULT */
#define NV_CHUNK   64u                  /* read size while scanning */

typedef struct {
    bool     valid;
    uint8_t  region;
    uint32_t off, len, seq;
} nv_dir_t;

typedef struct {
    bool     used, inflight;
    uint8_t  slot, tries;
    uint32_t len;
    uint8_t  data[NV_MAX_LEN];
} nv_q_t;

enum { J_IDLE, J_ERASE, J_WRITE, J_VERIFY };

static const nvlog_port_t *P;
static nv_dir_t   s_dir[NV_SLOTS];
static nv_q_t     s_q[NV_QUEUE];
static uint8_t    s_copy[NV_MAX_LEN];   /* a live record being copied to the other region */
static uint32_t   s_copy_len;
static uint8_t    s_copy_slot, s_copy_tries;
static uint8_t    s_active;             /* region appended to */
static uint32_t   s_tail[2];            /* used bytes (end of the highest programmed page) */
static bool       s_dirty[2];           /* region has programmed pages */
static uint32_t   s_seq;                /* next record sequence number */
static uint16_t   s_copy_mask;          /* slots whose live record sits in the stale region */
static bool       s_fault;
static nvlog_stats_t s_st;

static struct {
    uint8_t  state;
    uint8_t  region;                    /* region of the erase / write */
    uint32_t sector;                    /* erase: next sector */
    bool     inflight, is_erase;        /* a flash command is running */
    uint32_t t0;                        /* when it was issued (us) */
    /* the record being written */
    const uint8_t *src;
    uint32_t len, off, size, seq, crc, k;   /* k = pages issued so far */
    uint8_t  slot;
    int8_t   qi;                        /* queue entry, -1 = the copy buffer */
} J;

/* --- helpers ------------------------------------------------------------- */

uint32_t nvlog_crc32(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static uint32_t rd_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static void wr_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}

static uint32_t rec_size(uint32_t len)
{
    return NV_HDR + ((len + NV_PAGE - 1u) & ~(NV_PAGE - 1u)) + NV_TRL;
}

static void fault(uint8_t why)
{
    if (!s_fault) s_st.fault_why = why;
    s_fault = true;
    J.state = J_IDLE;
}

static bool page_erased(const uint8_t *p)
{
    return rd_u32(p) == P->erased_word && rd_u32(p + 4) == P->erased_word;
}

/* CRC of a record's covered bytes: meta, seq, payload (read from the flash). */
static bool rec_crc(uint8_t region, uint32_t off, uint32_t meta, uint32_t seq, uint32_t len, uint32_t *crc_out)
{
    uint8_t buf[NV_CHUNK];
    uint32_t crc = 0xFFFFFFFFu;
    wr_u32(buf, meta);
    wr_u32(buf + 4, seq);
    crc = nvlog_crc32(crc, buf, 8u);
    for (uint32_t i = 0; i < len; i += NV_CHUNK) {
        uint32_t n = (len - i < NV_CHUNK) ? len - i : NV_CHUNK;
        if (!P->read(region, off + NV_HDR + i, buf, n)) return false;
        crc = nvlog_crc32(crc, buf, n);
    }
    *crc_out = ~crc;
    return true;
}

/* Parse the record at `off`: true + fields when complete and its CRC matches. */
static bool rec_parse(uint8_t region, uint32_t off, uint32_t limit, uint8_t *slot, uint32_t *len, uint32_t *seq)
{
    uint8_t h[NV_HDR], t[NV_TRL];
    uint32_t crc;
    if (off + NV_HDR + NV_TRL > limit || !P->read(region, off, h, NV_HDR)) return false;
    if (rd_u32(h) != NV_MAGIC) return false;
    uint32_t meta = rd_u32(h + 4);
    uint8_t sl = (uint8_t)(meta & 0xFFu);
    uint32_t fmt = (meta >> 8) & 0xFFu, ln = meta >> 16;
    if (sl == 0u || sl >= NV_SLOTS || fmt != NV_FMT || ln == 0u || ln > NV_MAX_LEN) return false;
    uint32_t size = rec_size(ln);
    if (off + size > limit) return false;
    uint32_t sq = rd_u32(h + 8);
    if (rd_u32(h + 12) != ~sq) return false;
    if (!rec_crc(region, off, meta, sq, ln, &crc)) return false;
    if (!P->read(region, off + size - NV_TRL, t, NV_TRL)) return false;
    if (rd_u32(t) != crc || rd_u32(t + 4) != ~crc) return false;
    *slot = sl;
    *len = ln;
    *seq = sq;
    return true;
}

/* End of the used part of a region = after its highest programmed page. */
static uint32_t scan_tail(uint8_t region)
{
    uint8_t buf[NV_CHUNK];
    uint32_t off = P->region_bytes;
    while (off > 0u) {
        off -= NV_CHUNK;
        if (!P->read(region, off, buf, NV_CHUNK)) { fault(5u); return P->region_bytes; }
        for (uint32_t p = NV_CHUNK; p > 0u; p -= NV_PAGE) {
            if (!page_erased(buf + p - NV_PAGE)) return off + p;
        }
    }
    return 0u;
}

/* Every complete record of a region -> the directory (newest seq per slot wins). */
static void scan_records(uint8_t region)
{
    uint32_t off = 0u, limit = s_tail[region];
    while (off + NV_HDR + NV_TRL <= limit) {
        uint8_t slot;
        uint32_t len, seq;
        if (rec_parse(region, off, limit, &slot, &len, &seq)) {
            nv_dir_t *d = &s_dir[slot];
            if (!d->valid || (int32_t)(seq - d->seq) > 0) {
                d->valid = true;
                d->region = region;
                d->off = off;
                d->len = len;
                d->seq = seq;
            }
            if ((int32_t)(seq + 1u - s_seq) > 0) s_seq = seq + 1u;
            off += rec_size(len);
        } else {
            uint8_t h[NV_PAGE];
            if (P->read(region, off, h, NV_PAGE) && !page_erased(h) && rd_u32(h) == NV_MAGIC) s_st.bad_records++;
            off += NV_PAGE;
        }
    }
}

void nvlog_init(const nvlog_port_t *port)
{
    P = port;
    memset(s_dir, 0, sizeof s_dir);
    memset(s_q, 0, sizeof s_q);
    memset(&J, 0, sizeof J);
    memset(&s_st, 0, sizeof s_st);
    s_seq = 1u;
    s_fault = false;
    s_copy_mask = 0u;
    s_copy_tries = 0u;
    for (uint8_t r = 0; r < 2u; r++) {
        s_tail[r] = scan_tail(r);
        s_dirty[r] = s_tail[r] > 0u;
    }
    if (s_fault) return;
    scan_records(0u);
    scan_records(1u);
    /* the active region holds the newest record; the other one is stale (a
     * swap that was cut by a power loss): its live records are copied over
     * again, then it is erased */
    uint32_t best = 0u;
    bool any = false;
    s_active = 0u;
    for (uint8_t s = 1; s < NV_SLOTS; s++) {
        if (s_dir[s].valid && (!any || (int32_t)(s_dir[s].seq - best) > 0)) {
            best = s_dir[s].seq;
            s_active = s_dir[s].region;
            any = true;
        }
    }
    if (!any && s_dirty[0] && !s_dirty[1]) s_active = 1u;     /* A = garbage only: append to B, erase A */
    for (uint8_t s = 1; s < NV_SLOTS; s++) {
        if (s_dir[s].valid && s_dir[s].region != s_active) s_copy_mask |= (uint16_t)(1u << s);
    }
    s_st.active = s_active;
    s_st.tail[0] = s_tail[0];
    s_st.tail[1] = s_tail[1];
}

/* --- API --------------------------------------------------------------- */

bool nvlog_save(uint8_t slot, const void *data, uint32_t len)
{
    if (P == NULL || slot == 0u || slot >= NV_SLOTS || len == 0u || len > NV_MAX_LEN || s_fault) return false;
    nv_q_t *e = NULL;
    for (uint32_t i = 0; i < NV_QUEUE; i++) {
        if (s_q[i].used && !s_q[i].inflight && s_q[i].slot == slot) { e = &s_q[i]; break; }
    }
    if (e == NULL) {
        for (uint32_t i = 0; i < NV_QUEUE; i++) if (!s_q[i].used) { e = &s_q[i]; break; }
    }
    if (e == NULL) return false;
    e->used = true;
    e->inflight = false;
    e->slot = slot;
    e->tries = 0u;
    e->len = len;
    memcpy(e->data, data, len);
    s_st.saves++;
    return true;
}

bool nvlog_load(uint8_t slot, void *data, uint32_t len)
{
    if (P == NULL || slot == 0u || slot >= NV_SLOTS || len == 0u) return false;
    /* a queued save is the newest value (a replacement first, then the one in flight) */
    const nv_q_t *q = NULL;
    for (uint32_t i = 0; i < NV_QUEUE; i++) {
        if (s_q[i].used && s_q[i].slot == slot && (q == NULL || q->inflight)) q = &s_q[i];
    }
    if (q != NULL) {
        if (q->len != len) return false;
        memcpy(data, q->data, len);
        return true;
    }
    const nv_dir_t *d = &s_dir[slot];
    if (!d->valid || d->len != len) return false;
    if (J.inflight) {                   /* no flash read while a command runs: wait, bounded */
        uint32_t t0 = P->micros();
        s_st.load_waits++;
        while (P->busy()) {
            if (P->micros() - t0 > P->cmd_timeout_us) return false;
        }
    }
    return P->read(d->region, d->off + NV_HDR, data, len);
}

bool nvlog_pending(void)
{
    if (s_copy_mask) return true;
    for (uint32_t i = 0; i < NV_QUEUE; i++) if (s_q[i].used) return true;
    return J.state != J_IDLE;
}

bool nvlog_ok(void)
{
    return P != NULL && !s_fault;
}

const nvlog_stats_t *nvlog_stats(void)
{
    s_st.active = s_active;
    s_st.tail[0] = s_tail[0];
    s_st.tail[1] = s_tail[1];
    return &s_st;
}

/* --- the state machine ----------------------------------------------------- */

static void issue(bool erase)
{
    J.inflight = true;
    J.is_erase = erase;
    J.t0 = P->micros();
}

/* Start writing `len` bytes of `src` as a record of `slot` at the active tail. */
static bool write_begin(const uint8_t *src, uint32_t len, uint8_t slot, int8_t qi)
{
    uint8_t b[8];
    J.src = src;
    J.len = len;
    J.slot = slot;
    J.qi = qi;
    J.size = rec_size(len);
    J.region = s_active;
    J.off = s_tail[s_active];
    J.seq = s_seq++;
    J.k = 0u;
    wr_u32(b, (uint32_t)slot | (NV_FMT << 8) | (len << 16));
    wr_u32(b + 4, J.seq);
    J.crc = ~nvlog_crc32(nvlog_crc32(0xFFFFFFFFu, b, 8u), src, len);
    J.state = J_WRITE;
    return true;
}

/* The next page of the record in write order: trailer, payload..., seq, header. */
static void write_page(void)
{
    uint32_t npay = (J.size - NV_HDR - NV_TRL) / NV_PAGE, pg, w0, w1;
    if (J.k == 0u) {                                    /* trailer first */
        pg = J.size / NV_PAGE - 1u;
        w0 = J.crc; w1 = ~J.crc;
    } else if (J.k <= npay) {                           /* payload */
        uint8_t b[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        uint32_t i = (J.k - 1u) * NV_PAGE, n = (J.len - i < NV_PAGE) ? J.len - i : NV_PAGE;
        memcpy(b, J.src + i, n);
        pg = 2u + J.k - 1u;
        w0 = rd_u32(b); w1 = rd_u32(b + 4);
    } else if (J.k == npay + 1u) {                      /* sequence page */
        pg = 1u;
        w0 = J.seq; w1 = ~J.seq;
    } else {                                            /* header page = commit */
        pg = 0u;
        w0 = NV_MAGIC; w1 = (uint32_t)J.slot | (NV_FMT << 8) | (J.len << 16);
    }
    if (!P->write_start(J.region, J.off + pg * NV_PAGE, w0, w1)) { fault(3u); return; }
    issue(false);
    J.k++;
    if (J.k == npay + 3u) J.state = J_VERIFY;
}

static void write_failed(void)
{
    s_st.retries++;
    s_tail[J.region] = J.off + J.size;                  /* the garbage stays; append after it */
    s_dirty[J.region] = true;
    uint8_t *tries = (J.qi >= 0) ? &s_q[J.qi].tries : &s_copy_tries;
    if (++*tries >= NV_MAX_TRIES) { fault(3u); return; }
    if (J.qi >= 0) s_q[J.qi].inflight = false;
    J.state = J_IDLE;                                   /* picked up again (new tail / after a swap) */
}

static void verify(void)
{
    uint8_t slot;
    uint32_t len, seq;
    bool ok = rec_parse(J.region, J.off, J.off + J.size, &slot, &len, &seq) &&
              slot == J.slot && len == J.len && seq == J.seq;
    if (!ok) { write_failed(); return; }
    nv_dir_t *d = &s_dir[J.slot];
    d->valid = true;
    d->region = J.region;
    d->off = J.off;
    d->len = J.len;
    d->seq = J.seq;
    s_tail[J.region] = J.off + J.size;
    s_dirty[J.region] = true;
    s_st.writes++;
    if (J.qi >= 0) { s_q[J.qi].used = false; s_q[J.qi].inflight = false; }
    else { s_copy_mask &= (uint16_t)~(1u << J.slot); s_copy_tries = 0u; }
    J.state = J_IDLE;
}

/* The command that was running has finished: react to its result. */
static void finished(void)
{
    bool err = P->failed();
    J.inflight = false;
    if (J.state == J_ERASE) {
        if (err) { fault(2u); return; }
        J.sector++;
        if (J.sector * P->sector_bytes >= P->region_bytes) {
            s_tail[J.region] = 0u;
            s_dirty[J.region] = false;
            for (uint8_t s = 1; s < NV_SLOTS; s++) {
                if (s_dir[s].valid && s_dir[s].region == J.region) s_dir[s].valid = false;
            }
            s_st.erases++;
            J.state = J_IDLE;
        }
    } else if (J.state == J_WRITE || J.state == J_VERIFY) {
        if (err) write_failed();
    }
}

static void erase_begin(uint8_t region)
{
    J.state = J_ERASE;
    J.region = region;
    J.sector = 0u;
}

/* Move the append point to the other region: the live records follow, the
 * old region is erased once they are over. */
static bool swap_begin(void)
{
    uint8_t other = (uint8_t)(1u - s_active);
    if (s_dirty[other]) { fault(4u); return false; }    /* can't happen: cleaned before any write */
    /* every live record follows (also one with a newer save queued: a power
     * loss before that save is written must still keep the old record) */
    for (uint8_t s = 1; s < NV_SLOTS; s++) {
        if (s_dir[s].valid && s_dir[s].region == s_active) s_copy_mask |= (uint16_t)(1u << s);
    }
    s_active = other;
    s_st.swaps++;
    return true;
}

/* Choose the next piece of work (called with no command running, state IDLE). */
static void next_work(void)
{
    uint8_t other = (uint8_t)(1u - s_active);
    /* 1. live records still only in the stale region: copy them over */
    if (s_copy_mask) {
        uint8_t s = 1u;
        while (!(s_copy_mask & (1u << s))) s++;
        const nv_dir_t *d = &s_dir[s];
        if (!d->valid || d->region == s_active) { s_copy_mask &= (uint16_t)~(1u << s); return; }
        if (s_tail[s_active] + rec_size(d->len) > P->region_bytes) { fault(4u); return; }
        if (!P->read(d->region, d->off + NV_HDR, s_copy, d->len)) { fault(5u); return; }
        s_copy_len = d->len;
        s_copy_slot = s;
        (void)write_begin(s_copy, s_copy_len, s_copy_slot, -1);
        return;
    }
    /* 2. the other region holds garbage / stale records: erase it */
    if (s_dirty[other]) { erase_begin(other); return; }
    /* 3. a queued save */
    for (uint32_t i = 0; i < NV_QUEUE; i++) {
        nv_q_t *e = &s_q[i];
        if (!e->used) continue;
        if (s_tail[s_active] + rec_size(e->len) > P->region_bytes) {
            if (!swap_begin()) return;
            return;                                     /* the copies first (next call) */
        }
        e->inflight = true;
        (void)write_begin(e->data, e->len, e->slot, (int8_t)i);
        return;
    }
}

void nvlog_service(bool allowed, uint32_t budget_us)
{
    if (P == NULL || s_fault) return;
    uint32_t start = P->micros(), issued = 0u;
    for (;;) {
        if (J.inflight) {
            if (P->busy()) {
                if (P->micros() - J.t0 > P->cmd_timeout_us) { fault(1u); return; }
                if (J.is_erase || P->micros() - start >= budget_us) return;
                continue;                               /* a page program: short, spin within the budget */
            }
            finished();
            if (s_fault) return;
        }
        /* from here on a new flash command may be issued: only when allowed */
        switch (J.state) {
        case J_IDLE:
            if (!allowed) return;
            next_work();
            if (J.state == J_IDLE) return;              /* nothing to do (or a fault) */
            if (J.state == J_ERASE) continue;
            break;
        case J_ERASE:
            if (!allowed) return;
            if (!P->erase_start(J.region, J.sector)) { fault(2u); return; }
            issue(true);
            return;                                     /* erases take ms: poll next time */
        case J_WRITE:
            if (!allowed) return;                       /* the record goes on in STANDBY */
            break;
        case J_VERIFY:
            verify();                                   /* reads only */
            if (s_fault) return;
            continue;
        default:
            J.state = J_IDLE;
            return;
        }
        /* J_WRITE: at least one page per call, more while the budget lasts */
        if (issued > 0u && P->micros() - start >= budget_us) return;
        write_page();
        issued++;
        if (s_fault) return;
    }
}
