/*
 * jsframe.c - USB joystick bridge frame. See jsframe.h.
 */
#include "jsframe.h"

#include <string.h>

uint16_t jsf_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

void jsf_dev_clear(jsf_dev_t *d)
{
    for (int i = 0; i < JSF_AXES; i++) d->axis[i] = JSF_AXIS_NONE;
    d->buttons = 0u;
    d->hats = (uint8_t)((JSF_HAT_CENTRE << 4) | JSF_HAT_CENTRE);
    d->vid = 0u;
    d->pid = 0u;
}

int16_t jsf_norm(int32_t raw, int32_t lo, int32_t hi, bool invert)
{
    if (lo >= hi) return JSF_AXIS_NONE;
    int64_t span = (int64_t)hi - lo;
    int64_t slack = span / 50;
    if ((int64_t)raw < lo - slack || (int64_t)raw > hi + slack) return JSF_AXIS_NONE;
    int64_t v = (((int64_t)raw - lo) * 65534) / span - 32767;
    if (v > 32767) v = 32767;
    if (v < -32767) v = -32767;
    return (int16_t)(invert ? -v : v);
}

bool jsf_code_pressed(const jsf_t *f, uint8_t code)
{
    if (code == JSF_CODE_NONE || code > JSF_CODE_MAX) return false;
    if (code <= 64u) {
        unsigned c = code - 1u;
        return (f->dev[c / 32u].buttons >> (c % 32u)) & 1u;
    }
    unsigned c = code - 65u;
    unsigned dev = c / 8u, hat = (c / 4u) % 2u, dir = c % 4u;
    return jsf_hat(&f->dev[dev], (int)hat) == dir * 2u;   /* 0 up, 2 right, 4 down, 6 left */
}

static uint8_t *put16(uint8_t *p, uint16_t v) { *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); return p; }

size_t jsf_encode(const jsf_t *f, uint8_t *out)
{
    uint8_t *p = out;
    *p++ = JSF_SYNC1;
    *p++ = JSF_SYNC2;
    *p++ = (uint8_t)JSF_PAYLOAD;
    *p++ = f->seq;
    *p++ = f->flags;
    for (int d = 0; d < JSF_DEVS; d++) {
        const jsf_dev_t *v = &f->dev[d];
        for (int a = 0; a < JSF_AXES; a++) p = put16(p, (uint16_t)v->axis[a]);
        p = put16(p, (uint16_t)(v->buttons & 0xFFFFu));
        p = put16(p, (uint16_t)(v->buttons >> 16));
        *p++ = v->hats;
        p = put16(p, v->vid);
        p = put16(p, v->pid);
    }
    uint16_t crc = jsf_crc16(&out[2], (size_t)(p - &out[2]));
    p = put16(p, crc);
    return (size_t)(p - out);
}

void jsf_rx_init(jsf_rx_t *r)
{
    memset(r, 0, sizeof *r);
}

void jsf_rx_push(jsf_rx_t *r, const uint8_t *data, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (r->head - r->tail >= sizeof r->buf) r->tail++;      /* full: drop the oldest */
        r->buf[r->head++ % sizeof r->buf] = data[i];
    }
}

static uint8_t at(const jsf_rx_t *r, uint32_t i)
{
    return r->buf[(r->tail + i) % sizeof r->buf];
}

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

bool jsf_rx_next(jsf_rx_t *r, jsf_t *f)
{
    for (;;) {
        uint32_t used = r->head - r->tail;
        if (used < 3u) return false;
        if (at(r, 0) != JSF_SYNC1 || at(r, 1) != JSF_SYNC2 || at(r, 2) != JSF_PAYLOAD) {
            r->tail++;
            r->junk++;
            continue;
        }
        if (used < JSF_FRAME_LEN) return false;
        uint8_t tmp[JSF_FRAME_LEN];
        for (uint32_t i = 0; i < JSF_FRAME_LEN; i++) tmp[i] = at(r, i);
        if (jsf_crc16(&tmp[2], 1u + JSF_PAYLOAD) != get16(&tmp[3 + JSF_PAYLOAD])) {
            r->tail++;
            r->crc_errors++;
            continue;
        }
        const uint8_t *p = &tmp[3];
        f->seq = *p++;
        f->flags = *p++;
        for (int d = 0; d < JSF_DEVS; d++) {
            jsf_dev_t *v = &f->dev[d];
            for (int a = 0; a < JSF_AXES; a++, p += 2) v->axis[a] = (int16_t)get16(p);
            v->buttons = (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16);
            p += 4;
            v->hats = *p++;
            v->vid = get16(p);
            v->pid = get16(p + 2);
            p += 4;
        }
        r->tail += JSF_FRAME_LEN;
        r->frames++;
        return true;
    }
}
