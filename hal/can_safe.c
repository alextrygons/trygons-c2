/*
 * can_safe.c - CPU1's encoder for its safe frames (see can_safe.h).
 */
#include "can_safe.h"

#include <string.h>

uint16_t cs_float16(float v)
{
    union { float f; uint32_t u; } x;
    x.f = v;
    uint32_t sign = (x.u >> 16) & 0x8000u;
    int32_t  exp = (int32_t)((x.u >> 23) & 0xFFu);
    uint32_t man = x.u & 0x7FFFFFu;
    if (exp == 0xFF) return (uint16_t)(man ? 0x7E00u : (sign | 0x7C00u));     /* NaN / infinity */
    int32_t e = exp - 127 + 15;
    if (e >= 31) return (uint16_t)(sign | 0x7C00u);                           /* too large: infinity */
    if (e <= 0) {                                                             /* subnormal / zero */
        if (e < -10) return (uint16_t)sign;
        man |= 0x800000u;
        uint32_t shift = (uint32_t)(14 - e);
        uint32_t h = man >> shift;
        uint32_t rem = man & ((1u << shift) - 1u), half = 1u << (shift - 1u);
        if (rem > half || (rem == half && (h & 1u))) h++;
        return (uint16_t)(sign | h);
    }
    uint32_t h = ((uint32_t)e << 10) | (man >> 13);
    uint32_t rem = man & 0x1FFFu;
    if (rem > 0x1000u || (rem == 0x1000u && (h & 1u))) h++;                   /* (a carry rounds into the exponent) */
    return (uint16_t)(sign | h);
}

void cs_frame(dc_frame_t *f, uint8_t actuator_id, float value, uint8_t tid)
{
    if (!(value >= -1.0f)) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    uint16_t h = cs_float16(value);
    memset(f, 0, sizeof *f);
    f->data[0] = actuator_id;
    f->data[1] = DC_CMD_UNITLESS;
    f->data[2] = (uint8_t)(h & 0xFFu);
    f->data[3] = (uint8_t)(h >> 8);
    f->data[4] = (uint8_t)(0xC0u | (tid & 0x1Fu));                            /* start + end, toggle 0 */
    f->len = 5u;
    f->id = ((uint32_t)DC_PRIO_SAFE << 24) | ((uint32_t)DC_ARRAYCMD_ID << 8) | (uint32_t)DC_OWN_NODE_ID;
}
