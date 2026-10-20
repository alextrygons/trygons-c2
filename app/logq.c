/*
 * logq.c - Lock-free SPSC record queue. See logq.h.
 */
#include "logq.h"

#include "hal.h"

logq_t g_logq0, g_logq1, g_logq_gw;
static uint8_t s_q0[LOGQ0_SIZE];
static uint8_t s_q1[LOGQ1_SIZE];
static uint8_t s_qgw[LOGQGW_SIZE];

void logq_init(logq_t *q, uint8_t *buf, uint32_t size)
{
    q->buf = buf;
    q->size = size;
    q->head = 0u;
    q->tail = 0u;
    q->dropped = 0u;
    q->pushed = 0u;
}

void logq_init_all(void)
{
    logq_init(&g_logq0, s_q0, LOGQ0_SIZE);
    logq_init(&g_logq1, s_q1, LOGQ1_SIZE);
    logq_init(&g_logq_gw, s_qgw, LOGQGW_SIZE);
}

uint32_t logq_used(const logq_t *q)
{
    return q->head - q->tail;
}

bool logq_push(logq_t *q, const void *rec, uint16_t len)
{
    uint32_t head = q->head;
    uint32_t free_bytes = q->size - (head - q->tail);
    if (q->buf == 0 || (uint32_t)len + 2u > free_bytes) {
        q->dropped = q->dropped + 1u;
        return false;
    }
    const uint8_t *p = (const uint8_t *)rec;
    q->buf[head % q->size] = (uint8_t)(len & 0xFFu);
    q->buf[(head + 1u) % q->size] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) q->buf[(head + 2u + i) % q->size] = p[i];
    hal_mem_barrier();                  /* bytes visible before the new head */
    q->head = head + 2u + len;
    q->pushed = q->pushed + 1u;
    return true;
}

uint16_t logq_pop(logq_t *q, void *out, uint16_t max)
{
    uint32_t tail = q->tail;
    if (q->head == tail) return 0u;
    hal_mem_barrier();
    uint16_t len = (uint16_t)(q->buf[tail % q->size] | ((uint16_t)q->buf[(tail + 1u) % q->size] << 8));
    uint8_t *p = (uint8_t *)out;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = q->buf[(tail + 2u + i) % q->size];
        if (i < max) p[i] = b;
    }
    hal_mem_barrier();
    q->tail = tail + 2u + len;          /* the space is free again */
    return len <= max ? len : max;
}
