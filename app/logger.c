/*
 * logger.c - The logger (core 2). See logger.h.
 */
#include "logger.h"

#include <stdio.h>
#include <string.h>

#include "hal.h"

/* Every SD card call, then CPU2's own watchdog: one slow call is bounded by
 * the HAL (HAL_SD_MAX_MS), several in one step can't add up to a reset. */
static int sd_open(const char *name) { int h = hal_sd_open(name); hal_watchdog_kick_core(); return h; }
static bool sd_write(int h, const void *d, uint32_t n) { bool ok = hal_sd_write(h, d, n); hal_watchdog_kick_core(); return ok; }
static bool sd_sync(int h) { bool ok = hal_sd_sync(h); hal_watchdog_kick_core(); return ok; }
static bool sd_close(int h) { bool ok = hal_sd_close(h); hal_watchdog_kick_core(); return ok; }

#include "logq.h"
#include "logrec.h"
#include "modes.h"

log_status_t g_logstat;

/* The black box ring (1200 x 184 B = 216 KB) is too big for any one RAM
 * block that is free. It is NOT in CPU1's RAM (CPU1 keeps only the small
 * safety monitor): it is split into 4 chunks of 300 records (54 KB each):
 *   chunk 0  core 2's own data RAM (DSPR2, where the logger runs)
 *   chunk 1  LMU block 0 (cpu0_dlmu)    shared RAM, core 2 reads / writes it
 *   chunk 2  LMU block 1 (cpu1_dlmu)    (only the logger ever touches these,
 *   chunk 3  LMU block 2 (cpu2_dlmu)     so no cache coherency issue)
 * Section names: Lcf_Tasking_Tricore_Tc.lsl / Lcf_Gnuc_Tricore_Tc.lsl.
 * The TASKING pragma spelling is UNVERIFIED until the first ADS build:
 * check with tools/mapreport.py that DSPR1 stays below 60 %. */
#define BB_CHUNK (LOG_BB_RECORDS / LOG_BB_CHUNKS)
#if defined(__TASKING__)
#define BB_SECTION(tasking, gnu) _Pragma(tasking)
#define BB_END() _Pragma("section farbss restore")
#elif defined(__tricore__)
#define BB_SECTION(tasking, gnu) _Pragma(gnu)
#define BB_END() _Pragma("section")
#else
#define BB_SECTION(tasking, gnu)
#define BB_END()
#endif
BB_SECTION("section farbss bss_cpu2", "section .bss_cpu2 aw 4")
static uint8_t s_bb0[BB_CHUNK][LOG_BB_SLOT];
BB_END()
BB_SECTION("section farbss lmubss_cpu0", "section .lmubss_cpu0 aw 4")
static uint8_t s_bb1[BB_CHUNK][LOG_BB_SLOT];
BB_END()
BB_SECTION("section farbss lmubss_cpu1", "section .lmubss_cpu1 aw 4")
static uint8_t s_bb2[BB_CHUNK][LOG_BB_SLOT];
BB_END()
BB_SECTION("section farbss lmubss_cpu2", "section .lmubss_cpu2 aw 4")
static uint8_t s_bb3[BB_CHUNK][LOG_BB_SLOT];
BB_END()

/* Black-box record i (0 .. LOG_BB_RECORDS-1), whatever chunk it lives in. */
static uint8_t *bb_slot(uint32_t i)
{
    static uint8_t (*const chunk[LOG_BB_CHUNKS])[LOG_BB_SLOT] = { s_bb0, s_bb1, s_bb2, s_bb3 };
    i %= LOG_BB_RECORDS;
    return chunk[i / BB_CHUNK][i % BB_CHUNK];
}

static uint8_t s_bb_ev[LOG_BB_EVENTS][LOG_BB_EV_SLOT];      /* events: logger's own RAM */
static uint8_t s_rec[4096 + 64];
static uint8_t s_gw[4096 + 64 + 4];

/* Copy for the Raspberry Pi (core 0 forwards it, link.c): u32 number +
 * the record. Never waits: a full queue drops the record (counted). */
static void gw_push(logger_t *lg, const uint8_t *rec, uint16_t len)
{
    uint32_t seq = lg->gw_seq++;
    if ((uint32_t)len + 4u > sizeof s_gw) return;
    s_gw[0] = (uint8_t)seq;
    s_gw[1] = (uint8_t)(seq >> 8);
    s_gw[2] = (uint8_t)(seq >> 16);
    s_gw[3] = (uint8_t)(seq >> 24);
    memcpy(&s_gw[4], rec, len);
    (void)logq_push(&g_logq_gw, s_gw, (uint16_t)(len + 4u));
}

/* --- file helpers ------------------------------------------------------------- */

static void fail(logger_t *lg)
{
    if (lg->h >= 0) (void)sd_close(lg->h);
    lg->h = -1;
    lg->buf_n = 0u;
    lg->failed = true;
    lg->retry_at = lg->now + LOG_RETRY_MS;
    g_logstat.failure = 1u;
}

static bool raw_write(int h, uint8_t *buf, uint32_t *n, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    while (len) {
        uint32_t room = LOG_BUF_BYTES - *n;
        uint32_t k = len < room ? len : room;
        memcpy(buf + *n, p, k);
        *n += k;
        p += k;
        len -= k;
        if (*n == LOG_BUF_BYTES) {                  /* full block -> card */
            if (!sd_write(h, buf, LOG_BUF_BYTES)) return false;
            *n = 0u;
        }
    }
    return true;
}

static bool flush(int h, uint8_t *buf, uint32_t *n)
{
    if (*n && !sd_write(h, buf, *n)) return false;
    *n = 0u;
    return sd_sync(h);
}

static void file_name(const logger_t *lg, char *out, size_t max, const char *prefix, uint32_t no)
{
    if (lg->gps_date) {
        uint32_t m = lg->gps_tod_ms / 60000u;
        snprintf(out, max, "%s%05lu_%03lu_%08lu-%02lu%02lu.TLG", prefix, (unsigned long)g_logstat.boot,
                 (unsigned long)no, (unsigned long)lg->gps_date, (unsigned long)(m / 60u), (unsigned long)(m % 60u));
    } else {
        snprintf(out, max, "%s%05lu_%03lu.TLG", prefix, (unsigned long)g_logstat.boot, (unsigned long)no);
    }
}

/* magic, header, boot event, config: every file can be read on its own */
static bool file_start(logger_t *lg, int h, uint8_t *buf, uint32_t *n, uint32_t no)
{
    static char txt[3072];
    uint16_t tl = logrec_header_text(txt, sizeof txt, g_logstat.boot, no);
    uint16_t rl = logrec_make(s_rec, LOGR_HEADER, lg->now, txt, tl);
    if (h == lg->h) gw_push(lg, s_rec, rl);         /* the Pi starts a file too */
    bool ok = raw_write(h, buf, n, "TLG1", 4u) && raw_write(h, buf, n, s_rec, rl);
    if (ok && lg->boot_len) ok = raw_write(h, buf, n, lg->boot_rec, lg->boot_len);
    if (ok && lg->config_len) ok = raw_write(h, buf, n, lg->config_rec, lg->config_len);
    return ok;
}

static void open_next(logger_t *lg)
{
    char name[64];
    if (!hal_sd_present()) {
        g_logstat.card = 0u;
        fail(lg);
        return;
    }
    g_logstat.card = 1u;
    lg->file_no++;
    file_name(lg, name, sizeof name, "B", lg->file_no);
    lg->h = sd_open(name);
    lg->buf_n = 0u;
    if (lg->h < 0 || !file_start(lg, lg->h, lg->buf, &lg->buf_n, lg->file_no)) {
        fail(lg);
        return;
    }
    lg->file_t0 = lg->now;
    lg->file_bytes = 0u;
    lg->last_sync = lg->now;
    lg->failed = false;
    g_logstat.failure = 0u;
    g_logstat.files = g_logstat.files + 1u;
}

/* --- black box ----------------------------------------------------------------- */

static void bb_store(logger_t *lg, const uint8_t *rec, uint16_t len)
{
    if (!lg->bb_ok) return;
    if (rec[1] == LOGR_DATA && len <= LOG_BB_SLOT) {
        memcpy(bb_slot(lg->bb_head), rec, len);
        lg->bb_head++;
        if (lg->bb_count < LOG_BB_RECORDS) lg->bb_count++;
    } else if (rec[1] == LOGR_EVENT && len <= LOG_BB_EV_SLOT) {
        memcpy(s_bb_ev[lg->ev_head % LOG_BB_EVENTS], rec, len);
        lg->ev_head++;
        if (lg->ev_count < LOG_BB_EVENTS) lg->ev_count++;
    }
}

static uint16_t rec_len(const uint8_t *r)
{
    return (uint16_t)((r[2] | (r[3] << 8)) + LOGR_OVERHEAD);
}

static uint32_t rec_t(const uint8_t *r)
{
    return (uint32_t)r[4] | ((uint32_t)r[5] << 8) | ((uint32_t)r[6] << 16) | ((uint32_t)r[7] << 24);
}

static void bb_start(logger_t *lg)
{
    char name[64];
    if (lg->bb_active || !lg->bb_ok || !hal_sd_present()) return;
    lg->bb_no++;
    file_name(lg, name, sizeof name, "BB", lg->bb_no);
    lg->bbh = sd_open(name);
    lg->bb_buf_n = 0u;
    if (lg->bbh < 0 || !file_start(lg, lg->bbh, lg->bb_buf, &lg->bb_buf_n, lg->bb_no)) {
        if (lg->bbh >= 0) (void)sd_close(lg->bbh);
        lg->bbh = -1;
        return;
    }
    /* the last 60 s: data and events merged in time order */
    uint32_t di = lg->bb_head - lg->bb_count, ei = lg->ev_head - lg->ev_count;
    bool ok = true;
    while (ok && (di != lg->bb_head || ei != lg->ev_head)) {
        const uint8_t *d = (di != lg->bb_head) ? bb_slot(di) : 0;
        const uint8_t *e = (ei != lg->ev_head) ? s_bb_ev[ei % LOG_BB_EVENTS] : 0;
        if (d && (!e || rec_t(d) <= rec_t(e))) { ok = raw_write(lg->bbh, lg->bb_buf, &lg->bb_buf_n, d, rec_len(d)); di++; }
        else { ok = raw_write(lg->bbh, lg->bb_buf, &lg->bb_buf_n, e, rec_len(e)); ei++; }
    }
    if (!ok) {
        (void)sd_close(lg->bbh);
        lg->bbh = -1;
        return;
    }
    lg->bb_active = true;
    lg->bb_until = lg->now + LOG_BB_AFTER_MS;
    g_logstat.blackboxes = g_logstat.blackboxes + 1u;
}

static void bb_end(logger_t *lg)
{
    if (!lg->bb_active) return;
    (void)flush(lg->bbh, lg->bb_buf, &lg->bb_buf_n);
    (void)sd_close(lg->bbh);
    lg->bbh = -1;
    lg->bb_active = false;
}

/* An alarm, a fault, a monitor trip or STANDBY because of a fault. */
static bool bb_trigger(const uint8_t *rec)
{
    if (rec[1] != LOGR_EVENT) return false;
    uint16_t code = (uint16_t)(rec[LOGR_HDR] | (rec[LOGR_HDR + 1] << 8));
    if (code == EV_ALARM_ON || code == EV_FAULT || code == EV_MONITOR) return true;
    if (code == EV_MODE) {
        int32_t to = (int32_t)((uint32_t)rec[LOGR_HDR + 6] | ((uint32_t)rec[LOGR_HDR + 7] << 8));
        int32_t why = (int32_t)((uint32_t)rec[LOGR_HDR + 10] | ((uint32_t)rec[LOGR_HDR + 11] << 8));
        int32_t verdict = (int32_t)rec[LOGR_HDR + 14];
        return to == MODE_STANDBY && verdict == 0 &&
               (why == WHY_COMPASS_FAULT || why == WHY_GEAR_FAULT || why == WHY_JOY_FAULT ||
                why == WHY_MONITOR || why == WHY_TRAINING_ABORT || why == WHY_GPS_LOST);
    }
    return false;
}

/* --- one record ---------------------------------------------------------------- */

static void handle(logger_t *lg, const uint8_t *rec, uint16_t len)
{
    uint8_t type = rec[1];
    g_logstat.records = g_logstat.records + 1u;
    if (type == LOGR_CONFIG && len <= sizeof lg->config_rec) {
        memcpy(lg->config_rec, rec, len);
        lg->config_len = len;
    }
    if (type == LOGR_EVENT && len <= sizeof lg->boot_rec &&
        (uint16_t)(rec[LOGR_HDR] | (rec[LOGR_HDR + 1] << 8)) == EV_BOOT) {
        memcpy(lg->boot_rec, rec, len);
        lg->boot_len = len;
    }
    if (type == LOGR_EVENT && (uint16_t)(rec[LOGR_HDR] | (rec[LOGR_HDR + 1] << 8)) == EV_TIME) {
        /* GPS date / time (core 0, once a minute): used in the file names */
        memcpy(&lg->gps_date, &rec[LOGR_HDR + 2], 4);
        memcpy(&lg->gps_tod_ms, &rec[LOGR_HDR + 6], 4);
    }
    gw_push(lg, rec, len);
    bb_store(lg, rec, len);
    bool trig = bb_trigger(rec);
    if (lg->h >= 0) {
        if (!raw_write(lg->h, lg->buf, &lg->buf_n, rec, len)) fail(lg);
        else lg->file_bytes += len;
    }
    if (lg->bb_active && !raw_write(lg->bbh, lg->bb_buf, &lg->bb_buf_n, rec, len)) bb_end(lg);
    if (trig) bb_start(lg);
    g_logstat.bytes = g_logstat.bytes + len;
}

/* --- public -------------------------------------------------------------------- */

void logger_init(logger_t *lg, uint32_t now_ms)
{
    memset(lg, 0, sizeof *lg);
    lg->h = -1;
    lg->bbh = -1;
    lg->now = now_ms;
    lg->bb_ok = (uint32_t)logrec_data_size() + LOGR_OVERHEAD <= LOG_BB_SLOT;
    g_logstat.failure = 0u;
    g_logstat.files = 0u;
    g_logstat.blackboxes = 0u;
    g_logstat.bytes = 0u;
    g_logstat.records = 0u;
    {
        /* the Pi copy starts with the field description even without an SD card */
        static char txt[3072];
        uint16_t tl = logrec_header_text(txt, sizeof txt, g_logstat.boot, 0u);
        gw_push(lg, s_rec, logrec_make(s_rec, LOGR_HEADER, now_ms, txt, tl));
    }
}

void logger_step(logger_t *lg, uint32_t now_ms)
{
    lg->now = now_ms;
    if (g_logstat.gw_resend) {
        /* the Pi (re)connected: give its copy a fresh start it can read on its own */
        static char txt[3072];
        g_logstat.gw_resend = 0u;
        uint16_t tl = logrec_header_text(txt, sizeof txt, g_logstat.boot, lg->file_no);
        gw_push(lg, s_rec, logrec_make(s_rec, LOGR_HEADER, now_ms, txt, tl));
        if (lg->boot_len) gw_push(lg, lg->boot_rec, lg->boot_len);
        if (lg->config_len) gw_push(lg, lg->config_rec, lg->config_len);
    }
    if (lg->h < 0 && (!lg->failed || (int32_t)(now_ms - lg->retry_at) >= 0)) open_next(lg);

    /* drain: bounded work per step */
    for (uint32_t k = 0; k < LOG_STEP_RECORDS; k++) {
        uint16_t len = logq_pop(&g_logq0, s_rec, sizeof s_rec);
        if (!len) len = logq_pop(&g_logq1, s_rec, sizeof s_rec);
        if (!len) break;
        if (s_rec[0] != LOGR_SYNC || len < LOGR_OVERHEAD) continue;
        handle(lg, s_rec, len);
    }

    /* records dropped by a full queue: counted, and the count logged */
    if (g_logq0.dropped != lg->drop0 || g_logq1.dropped != lg->drop1) {
        lg->drop0 = g_logq0.dropped;
        lg->drop1 = g_logq1.dropped;
        uint8_t ev[18 + LOGR_OVERHEAD];
        uint8_t pl[18];
        memset(pl, 0, sizeof pl);
        pl[0] = (uint8_t)EV_LOG;
        pl[2] = 6u;
        memcpy(&pl[6], &lg->drop0, 4);
        memcpy(&pl[10], &lg->drop1, 4);
        uint16_t n = logrec_make(ev, LOGR_EVENT, now_ms, pl, sizeof pl);
        handle(lg, ev, n);
    }

    if (lg->h >= 0 && (int32_t)(now_ms - lg->last_sync) >= (int32_t)LOG_SYNC_MS) {
        lg->last_sync = now_ms;
        if (!flush(lg->h, lg->buf, &lg->buf_n)) fail(lg);
        if (lg->bb_active && !flush(lg->bbh, lg->bb_buf, &lg->bb_buf_n)) bb_end(lg);
    }
    if (lg->bb_active && (int32_t)(now_ms - lg->bb_until) >= 0) bb_end(lg);
    if (lg->h >= 0 && ((now_ms - lg->file_t0) >= LOG_ROTATE_MS || lg->file_bytes >= LOG_ROTATE_BYTES)) {
        if (!flush(lg->h, lg->buf, &lg->buf_n)) fail(lg);
        if (lg->h >= 0) (void)sd_close(lg->h);
        lg->h = -1;
        lg->failed = false;
        open_next(lg);
    }
}

void logger_close(logger_t *lg)
{
    bb_end(lg);
    if (lg->h >= 0) {
        (void)flush(lg->h, lg->buf, &lg->buf_n);
        (void)sd_close(lg->h);
        lg->h = -1;
    }
}
