/*
 * logq.h - Lock-free single-producer / single-consumer record queue.
 *
 * Core 0 (and CPU1) push log records; the logger on core 2 pops them.
 * One writer per queue, one reader: no locks, no waiting. A push that
 * doesn't fit is DROPPED and counted - logging never blocks or slows
 * control.
 *
 * Records are stored whole: u16 length, then the bytes. The writer only
 * moves `head`, the reader only moves `tail`; a memory barrier makes the
 * bytes visible before the new head.
 */
#ifndef LOGQ_H
#define LOGQ_H

#include <stdbool.h>
#include <stdint.h>

#define LOGQ0_SIZE  16384u      /* core 0 queue (~2.5 s of 20 Hz data) */
#define LOGQ1_SIZE  1024u       /* CPU1 (monitor events) */
#define LOGQGW_SIZE 16384u      /* logger (core 2) -> gateway link (core 0): the log copy for the Pi */

typedef struct {
    uint8_t          *buf;
    uint32_t          size;     /* power of 2 */
    volatile uint32_t head;     /* written by the producer only */
    volatile uint32_t tail;     /* written by the consumer only */
    volatile uint32_t dropped;  /* records that didn't fit (producer) */
    volatile uint32_t pushed;
} logq_t;

extern logq_t g_logq0, g_logq1;
/* Every record the logger handles, prefixed with its u32 sequence number;
 * core 0 forwards it to the Raspberry Pi (link.c). Full -> dropped and
 * counted (the Pi sees the gap in the numbers); the SD card is unaffected. */
extern logq_t g_logq_gw;

void logq_init(logq_t *q, uint8_t *buf, uint32_t size);
void logq_init_all(void);       /* g_logq0, g_logq1, g_logq_gw with their buffers */
/* Producer: whole record or nothing. Returns false if dropped. */
bool logq_push(logq_t *q, const void *rec, uint16_t len);
/* Consumer: next record into out (max bytes). Returns its length, 0 if empty. */
uint16_t logq_pop(logq_t *q, void *out, uint16_t max);
uint32_t logq_used(const logq_t *q);

#endif /* LOGQ_H */
