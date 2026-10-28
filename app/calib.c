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

/* --- version 2: the DroneCAN steering position servo (step 11e) ------------------ */

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

uint32_t calib2_checksum(const calib2_t *c)
{
    return crc32((const uint8_t *)c, offsetof(calib2_t, checksum));
}

void calib2_seal(calib2_t *c)
{
    c->magic = CALIB_MAGIC;
    c->version = CALIB2_VERSION;
    c->size = (uint32_t)sizeof(calib2_t);
    c->checksum = calib2_checksum(c);
}

bool calib2_valid(const calib2_t *c)
{
    if (c->magic != CALIB_MAGIC || c->version != CALIB2_VERSION || c->size != (uint32_t)sizeof(calib2_t) ||
        c->checksum != calib2_checksum(c))
        return false;
    return in_range(c->r_right, 0.02f, 20.0f) && in_range(c->r_left, 0.02f, 20.0f)
        && in_range(c->b_lr, 0.0f, 0.3f) && in_range(c->b_rl, 0.0f, 0.3f)
        && in_range(c->k, 0.3f, 500.0f) && in_range(c->tau, 0.1f, 60.0f)
        && in_range(c->a0, -0.5f, 0.5f) && in_range(c->confidence, 0.0f, 1.0f)
        && in_range(c->v_ref_kn, 0.0f, 40.0f) && in_range(c->res_dps, 0.0f, 20.0f)
        && in_range(c->full_angle_deg, 0.0f, 180.0f);
}

bool calib2_save(calib2_t *c)
{
    calib2_seal(c);
    return nv_save(HAL_NV_SLOT_CALIB, c, (uint32_t)sizeof *c);
}

bool calib2_load(calib2_t *c, bool *v1)
{
    *v1 = false;
    if (nv_load(HAL_NV_SLOT_CALIB, c, (uint32_t)sizeof *c)) {
        if (calib2_valid(c)) return true;
        nv_invalid(HAL_NV_SLOT_CALIB);          /* a v2-sized record that fails: NV DATA RESET */
        return false;
    }
    calib_data_t old;
    if (nv_load(HAL_NV_SLOT_CALIB, &old, (uint32_t)sizeof old)) {
        /* a version-1 (DC-motor) record: not converted - defaults, TRAINING needed; any other record of that size
         * is unusable as well: NV DATA RESET either way */
        *v1 = old.magic == CALIB_MAGIC && old.version == CALIB_VERSION;
        nv_invalid(HAL_NV_SLOT_CALIB);
    }
    return false;
}
