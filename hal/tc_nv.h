/*
 * tc_nv.h - Non-volatile record log for the data flash (pure C99, no iLLD:
 * the flash itself is reached through a small port, so the same code runs on
 * the TC375 DFLASH0 (hal_tc375.c) and on a PC flash image
 * (tests/hal_pc/nv_check.py, sim test DF4)).
 *
 * Two regions A / B of the data flash are used as an APPEND-ONLY record log:
 * a save appends the next record, a load takes the newest record of the slot
 * with a valid CRC. When the active region is full the live records (the
 * newest of every slot) are copied to the other region, then the full one is
 * erased (wear levelling: every page is written once per erase cycle). A
 * power loss during any write keeps the previous record: a record counts only
 * once its header page (written last) is complete and its CRC matches.
 *
 * Record (8-byte flash pages, little-endian words):
 *   page 0      {magic "TRYN", slot | format << 8 | len << 16}   written LAST (commit)
 *   page 1      {seq, ~seq}                                       seq grows with every record
 *   pages 2..   payload, padded to 8 bytes with 0
 *   last page   {crc32, ~crc32}                                   written FIRST
 * The trailer is written first so the highest programmed page of a region
 * always marks the end of the used part, whatever a cut write left behind
 * (erased pages read as port->erased_word; a payload page may look like one).
 *
 * Flash commands are a state machine: nvlog_service() issues at most a few page
 * programs (within its time budget) or ONE sector erase per call and polls
 * "busy" the next time - never a blocking wait. The flash is only READ while
 * no command runs (TC3xx: a read of a busy DFLASH bank is not allowed).
 */
#ifndef TC_NV_H
#define TC_NV_H

#include <stdbool.h>
#include <stdint.h>

#define NV_PAGE      8u
#define NV_SLOTS     16u        /* slots 1..15 (0 unused) */
#define NV_MAX_LEN   1536u      /* payload bytes per record (= HAL_NV_MAX_SIZE) */
#define NV_QUEUE     4u         /* saves waiting for the flash */

typedef struct {
    uint32_t region_bytes;      /* bytes per region (A and B), a multiple of sector_bytes */
    uint32_t sector_bytes;      /* erase unit (TC3xx DFLASH logical sector: 4096) */
    uint32_t erased_word;       /* what an erased word reads as (TC3xx DFLASH: 0) */
    uint32_t cmd_timeout_us;    /* a command still busy after this = NV FAULT */
    /* read `len` bytes at `off` of the region (never called while a command runs) */
    bool (*read)(uint8_t region, uint32_t off, void *buf, uint32_t len);
    /* issue the erase of one sector of the region (returns at once) */
    bool (*erase_start)(uint8_t region, uint32_t sector);
    /* issue the programming of one 8-byte page (returns at once) */
    bool (*write_start)(uint8_t region, uint32_t off, uint32_t w0, uint32_t w1);
    bool (*busy)(void);         /* a command is still running */
    bool (*failed)(void);       /* the last command reported an error (checked when not busy) */
    uint32_t (*micros)(void);
} nvlog_port_t;

typedef struct {
    uint32_t saves, writes, swaps, erases, retries, bad_records, load_waits;
    uint8_t  active;            /* region records are appended to */
    uint32_t tail[2];           /* used bytes per region */
    uint8_t  fault_why;         /* 0 none, 1 command timeout, 2 erase failed, 3 write failed, 4 no room, 5 read */
} nvlog_stats_t;

/* Scan both regions (reads only; no command is issued). */
void nvlog_init(const nvlog_port_t *port);
/* Queue a save (the data is copied): false = bad slot / length or queue full. */
bool nvlog_save(uint8_t slot, const void *data, uint32_t len);
/* Newest valid record of the slot (a queued one first). Needs exactly `len` bytes. */
bool nvlog_load(uint8_t slot, void *data, uint32_t len);
/* One bounded step; allowed = false only polls (no new command is issued). */
void nvlog_service(bool allowed, uint32_t budget_us);
bool nvlog_pending(void);          /* saves not yet committed to the flash */
bool nvlog_ok(void);               /* false = NV FAULT: nothing is saved any more */
const nvlog_stats_t *nvlog_stats(void);
uint32_t nvlog_crc32(uint32_t crc, const void *data, uint32_t len);   /* reflected, 0xEDB88320; start with 0xFFFFFFFF, finish with ~ */

#endif /* TC_NV_H */
