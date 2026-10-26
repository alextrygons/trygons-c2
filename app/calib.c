/*
 * calib.c - Outboard calibration record: checksum, validation, storage.
 */
#include "calib.h"
#include "hal.h"
#include "nv.h"

#include <math.h>
#include <stddef.h>

uint32_t calib_checksum(const calib_data_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    size_t n = offsetof(calib_data_t, checksum);
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

void calib_seal(calib_data_t *c)
{
    c->magic = CALIB_MAGIC;
    c->version = CALIB_VERSION;
    c->size = (uint32_t)sizeof(calib_data_t);
    c->checksum = calib_checksum(c);
}

static bool in_range(float x, float lo, float hi)
{
    return isfinite(x) && x >= lo && x <= hi;
}

bool calib_valid(const calib_data_t *c)
{
    if (c->magic != CALIB_MAGIC || c->version != CALIB_VERSION ||
        c->size != (uint32_t)sizeof(calib_data_t) ||
        c->checksum != calib_checksum(c)) {
        return false;
    }
    float travel = c->travel_left_ms + c->travel_right_ms;
    return in_range(c->gain_dps_per_ms, 1e-6f, 1.0f)
        && in_range(c->time_const_s, 0.1f, 60.0f)
        && in_range(c->travel_right_ms, 200.0f, 120000.0f)
        && in_range(c->travel_left_ms, 200.0f, 120000.0f)
        && in_range(c->gap_ms, 0.0f, 0.2f * travel)
        && in_range(c->trim_dps, -10.0f, 10.0f)
        && in_range(c->full_angle_deg, 0.0f, 120.0f);
}

bool calib_save(calib_data_t *c)
{
    calib_seal(c);
    return nv_save(HAL_NV_SLOT_CALIB, c, (uint32_t)sizeof *c);     /* queued; written in STANDBY */
}

bool calib_load(calib_data_t *c)
{
    if (!nv_load(HAL_NV_SLOT_CALIB, c, (uint32_t)sizeof *c)) return false;
    if (calib_valid(c)) return true;
    nv_invalid(HAL_NV_SLOT_CALIB);              /* stored record unusable: NV DATA RESET notice */
    return false;
}
