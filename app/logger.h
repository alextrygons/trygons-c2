/*
 * logger.h - The logger (core 2 on the board; in the sim a separate step).
 *
 * Drains the lock-free queues (core 0: g_logq0, CPU1: g_logq1) and writes
 * .TLG files to the microSD card (hal_sd_*):
 *   - RAM buffer, 4 KB block writes, sync at least every 2 s
 *     -> a power cut loses at most ~2 s and never older files
 *   - new file every 10 min or 20 MB; names "Bbbbbb_fff.TLG" (boot number,
 *     file number), with "_YYYYMMDD-HHMM" once GPS time is known
 *   - every file starts with the header (field list), the boot event and
 *     the latest config / calibration -> each file is self-contained
 *   - black box: the last 60 s of 20 Hz data (and recent events) always in
 *     RAM; on an alarm / fault it goes to a separate "BBbbbbb_nnn.TLG"
 *     file together with the following 30 s
 *   - card missing / full / failing: g_logstat.failure (core 0 raises
 *     "LOG FAILURE"), retry every 10 s; control is never affected
 *   - records dropped by a full queue are counted and logged
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_BUF_BYTES       4096u
#define LOG_SYNC_MS         2000u
#define LOG_ROTATE_MS       600000u
#define LOG_ROTATE_BYTES    (20u * 1024u * 1024u)
#define LOG_RETRY_MS        10000u
#define LOG_BB_RECORDS      1200u       /* 60 s at 20 Hz */
#define LOG_BB_SLOT         172u        /* one DATA record (checked at init) */
#define LOG_BB_EVENTS       128u
#define LOG_BB_EV_SLOT      64u
#define LOG_BB_AFTER_MS     30000u
#define LOG_STEP_RECORDS    64u         /* records handled per logger step (bounded time) */

/* Shared status, read by core 0 (alarm) and shown in telemetry. */
typedef struct {
    volatile uint8_t  failure;          /* card missing / full / write error */
    volatile uint8_t  card;             /* card present */
    volatile uint32_t files;            /* files written since boot */
    volatile uint32_t blackboxes;
    volatile uint32_t bytes;            /* bytes written since boot */
    volatile uint32_t records;
    volatile uint32_t boot;             /* boot number (set by core 0 before the logger starts) */
} log_status_t;

extern log_status_t g_logstat;

typedef struct {
    int      h;                         /* main log file, -1 = none */
    uint32_t file_no, file_t0, file_bytes;
    uint8_t  buf[LOG_BUF_BYTES];
    uint32_t buf_n;
    uint32_t last_sync;
    uint32_t retry_at;
    bool     failed;
    /* copies written at the start of every file */
    uint8_t  config_rec[4096 + 16];
    uint16_t config_len;
    uint8_t  boot_rec[32];
    uint16_t boot_len;
    /* GPS time for file names */
    uint32_t gps_date, gps_tod_ms;
    /* black box */
    bool     bb_ok;
    uint32_t bb_head, bb_count, ev_head, ev_count;
    int      bbh;
    bool     bb_active;
    uint32_t bb_until, bb_no;
    uint8_t  bb_buf[LOG_BUF_BYTES];
    uint32_t bb_buf_n;
    /* drop counters already logged */
    uint32_t drop0, drop1;
    uint32_t now;
} logger_t;

void logger_init(logger_t *lg, uint32_t now_ms);
/* One logger step (core 2 loop: every ~10 ms). */
void logger_step(logger_t *lg, uint32_t now_ms);
/* Close everything cleanly (e.g. before a planned power-off). */
void logger_close(logger_t *lg);

#endif /* LOGGER_H */
