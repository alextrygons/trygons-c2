/*
 * proto.c - Ground-station frames: CRC, encode, receive ring. See proto.h.
 */
#include "proto.h"

#include <math.h>
#include <string.h>

uint16_t proto_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t proto_encode(uint8_t *out, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    if (len > PROTO_MAX_PAYLOAD) return 0u;
    out[0] = PROTO_SYNC1;
    out[1] = PROTO_SYNC2;
    out[2] = len;
    out[3] = type;
    out[4] = seq;
    if (len > 0u) memcpy(&out[5], payload, len);
    uint16_t crc = proto_crc16(&out[2], 3u + (size_t)len);
    out[5u + len] = (uint8_t)(crc & 0xFFu);
    out[6u + len] = (uint8_t)(crc >> 8);
    return (size_t)len + PROTO_OVERHEAD;
}

/* --- receive -------------------------------------------------------------- */

void proto_rx_init(proto_rx_t *r)
{
    memset(r, 0, sizeof *r);
}

void proto_rx_push(proto_rx_t *r, const uint8_t *data, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (r->head - r->tail >= PROTO_RX_BUF) {    /* full: drop the oldest byte */
            r->tail++;
            r->overflows++;
        }
        r->buf[r->head++ % PROTO_RX_BUF] = data[i];
    }
}

static uint8_t at(const proto_rx_t *r, uint32_t i)
{
    return r->buf[(r->tail + i) % PROTO_RX_BUF];
}

bool proto_rx_next(proto_rx_t *r, proto_frame_t *f)
{
    for (;;) {
        uint32_t used = r->head - r->tail;
        if (used < 2u) return false;
        if (at(r, 0) != PROTO_SYNC1 || at(r, 1) != PROTO_SYNC2) {
            r->tail++;
            r->junk++;
            continue;
        }
        if (used < 5u) return false;
        uint8_t len = at(r, 2);
        if (len > PROTO_MAX_PAYLOAD) {          /* impossible length: not a frame start */
            r->tail++;
            r->crc_errors++;
            continue;
        }
        uint32_t total = (uint32_t)len + PROTO_OVERHEAD;
        if (used < total) return false;         /* wait for the rest */

        uint8_t tmp[PROTO_MAX_FRAME];
        for (uint32_t i = 0; i < total; i++) tmp[i] = at(r, i);
        uint16_t crc = proto_crc16(&tmp[2], 3u + (size_t)len);
        if ((uint8_t)(crc & 0xFFu) != tmp[5u + len] || (uint8_t)(crc >> 8) != tmp[6u + len]) {
            r->tail++;                          /* drop only the first byte, rescan */
            r->crc_errors++;
            continue;
        }
        f->len = len;
        f->type = tmp[3];
        f->seq = tmp[4];
        if (len > 0u) memcpy(f->payload, &tmp[5], len);
        r->tail += total;
        r->frames++;
        return true;
    }
}

/* --- payload writer / reader ---------------------------------------------- */

void proto_wr_init(proto_wr_t *w, uint8_t *buf, size_t max)
{
    w->p = buf;
    w->n = 0u;
    w->max = max;
    w->overflow = false;
}

void proto_wr_u8(proto_wr_t *w, uint8_t v)
{
    if (w->n >= w->max) { w->overflow = true; return; }
    w->p[w->n++] = v;
}

void proto_wr_u16(proto_wr_t *w, uint16_t v)
{
    proto_wr_u8(w, (uint8_t)(v & 0xFFu));
    proto_wr_u8(w, (uint8_t)(v >> 8));
}

void proto_wr_u32(proto_wr_t *w, uint32_t v)
{
    proto_wr_u16(w, (uint16_t)(v & 0xFFFFu));
    proto_wr_u16(w, (uint16_t)(v >> 16));
}

void proto_wr_i32(proto_wr_t *w, int32_t v)
{
    proto_wr_u32(w, (uint32_t)v);
}

void proto_wr_f32(proto_wr_t *w, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof u);
    proto_wr_u32(w, u);
}

void proto_wr_latlon(proto_wr_t *w, double deg)
{
    proto_wr_i32(w, (int32_t)lround(deg * 1e7));
}

void proto_rd_init(proto_rd_t *r, const uint8_t *buf, size_t len)
{
    r->p = buf;
    r->n = 0u;
    r->len = len;
    r->underflow = false;
}

uint8_t proto_rd_u8(proto_rd_t *r)
{
    if (r->n >= r->len) { r->underflow = true; return 0u; }
    return r->p[r->n++];
}

int8_t proto_rd_i8(proto_rd_t *r)
{
    return (int8_t)proto_rd_u8(r);
}

uint32_t proto_rd_u32(proto_rd_t *r)
{
    uint32_t v = 0u;
    for (int i = 0; i < 4; i++) v |= (uint32_t)proto_rd_u8(r) << (8 * i);
    return v;
}

int32_t proto_rd_i32(proto_rd_t *r)
{
    return (int32_t)proto_rd_u32(r);
}

float proto_rd_f32(proto_rd_t *r)
{
    uint32_t u = proto_rd_u32(r);
    float v;
    memcpy(&v, &u, sizeof v);
    return v;
}

double proto_rd_latlon(proto_rd_t *r)
{
    return (double)proto_rd_i32(r) * 1e-7;
}

uint32_t proto_rx_pending(const proto_rx_t *r)
{
    return r->head - r->tail;
}

void proto_rx_stale(proto_rx_t *r)
{
    if (r->head != r->tail) {
        r->tail++;
        r->crc_errors++;
    }
}
