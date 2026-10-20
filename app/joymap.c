/*
 * joymap.c - Joystick source and USB button / axis map. See joymap.h.
 */
#include "joymap.h"

#include <stddef.h>
#include <string.h>

#include "jsframe.h"

void joymap_default(joymap_t *m)
{
    memset(m, 0, sizeof *m);
    m->source = JOY_SRC_ANALOG;
    m->thr_input = JOY_THR_LEVER;
    m->axis_steer = JSF_AXIS_CODE(0u, 0u);      /* T.16000M X */
    m->axis_thr = JSF_AXIS_CODE(1u, 2u);        /* TWCS main lever (Z) */
    m->inv_steer = 0u;
    m->inv_thr = 1u;                            /* TWCS: lever forward = low raw value */
    m->btn[JM_TAKE]     = JSF_CODE_BTN(0u, 0u);                 /* stick trigger */
    m->btn[JM_STANDBY]  = JSF_CODE_BTN(0u, 1u);                 /* stick button 2 (thumb) */
    m->btn[JM_AUTO]     = JSF_CODE_BTN(1u, 0u);                 /* TWCS button 1 */
    m->btn[JM_CONFIRM]  = JSF_CODE_BTN(1u, 3u);                 /* TWCS button 4 */
    m->btn[JM_HDG_LOCK] = JSF_CODE_BTN(1u, 2u);                 /* TWCS button 3 */
    m->btn[JM_MINUS_1]  = JSF_CODE_HAT(1u, 0u, JSF_HAT_LEFT);
    m->btn[JM_PLUS_1]   = JSF_CODE_HAT(1u, 0u, JSF_HAT_RIGHT);
    m->btn[JM_MINUS_10] = JSF_CODE_HAT(1u, 0u, JSF_HAT_DOWN);
    m->btn[JM_PLUS_10]  = JSF_CODE_HAT(1u, 0u, JSF_HAT_UP);
    m->nz_lo = 0.45f;
    m->nz_hi = 0.55f;
    m->lever_delta = 0.10f;
    m->pickup_win = 0.03f;
    joymap_seal(m);
}

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

jm_error_t joymap_check(const joymap_t *m)
{
    if (m->source > JOY_SRC_BRIDGE || m->thr_input > JOY_THR_LEVER) return JM_ERR_CODE;
    if (m->axis_steer >= JSF_DEVS * JSF_AXES || m->axis_thr >= JSF_DEVS * JSF_AXES ||
        m->axis_steer == m->axis_thr || m->inv_steer > 1u || m->inv_thr > 1u) return JM_ERR_CODE;
    for (int i = 0; i < JM_COUNT; i++)
        if (m->btn[i] > JSF_CODE_MAX) return JM_ERR_CODE;
    if (m->btn[JM_TAKE] == 0u || m->btn[JM_AUTO] == 0u || m->btn[JM_CONFIRM] == 0u) return JM_ERR_MISSING;
    for (int i = 0; i < JM_COUNT; i++)
        for (int k = i + 1; k < JM_COUNT; k++)
            if (m->btn[i] != 0u && m->btn[i] == m->btn[k]) return JM_ERR_DUPLICATE;
    /* neutral zone in the middle part of the travel, at least 4 % wide */
    if (!(m->nz_lo >= 0.2f && m->nz_hi <= 0.8f && m->nz_hi - m->nz_lo >= 0.04f)) return JM_ERR_ZONE;
    if (!(m->lever_delta >= 0.03f && m->lever_delta <= 0.5f)) return JM_ERR_ZONE;
    if (!(m->pickup_win >= 0.01f && m->pickup_win <= 0.1f)) return JM_ERR_ZONE;
    return JM_OK;
}

void joymap_seal(joymap_t *m)
{
    m->magic = JOYMAP_MAGIC;
    m->version = JOYMAP_VERSION;
    m->size = (uint32_t)sizeof *m;
    m->checksum = crc32((const uint8_t *)m, offsetof(joymap_t, checksum));
}

bool joymap_valid(const joymap_t *m)
{
    return m->magic == JOYMAP_MAGIC && m->version == JOYMAP_VERSION &&
           m->size == (uint32_t)sizeof *m &&
           m->checksum == crc32((const uint8_t *)m, offsetof(joymap_t, checksum)) &&
           joymap_check(m) == JM_OK;
}

const char *joymap_error_name(jm_error_t e)
{
    switch (e) {
    case JM_OK:            return "ok";
    case JM_ERR_MISSING:   return "TAKE, AUTO and CONFIRM must be mapped";
    case JM_ERR_DUPLICATE: return "two functions share a button";
    case JM_ERR_CODE:      return "bad code";
    case JM_ERR_ZONE:      return "bad neutral zone / delta / window";
    default:               return "?";
    }
}

const char *joymap_button_name(jm_button_t b)
{
    static const char *const n[JM_COUNT] = { "TAKE", "AUTO", "CONFIRM", "HDG LOCK", "STANDBY",
                                             "-1", "+1", "-10", "+10" };
    return (b < JM_COUNT) ? n[b] : "?";
}

static uint8_t code_dev(uint8_t code)
{
    if (code == 0u || code > JSF_CODE_MAX) return 0u;
    if (code <= 64u) return (uint8_t)(1u << ((code - 1u) / 32u));
    return (uint8_t)(1u << ((code - 65u) / 8u));
}

uint8_t joymap_devices(const joymap_t *m)
{
    uint8_t d = (uint8_t)((1u << (m->axis_steer / JSF_AXES)) | (1u << (m->axis_thr / JSF_AXES)));
    for (int i = 0; i < JM_COUNT; i++) d |= code_dev(m->btn[i]);
    return d;
}
