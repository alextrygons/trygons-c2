/*
 * nv.c - App side of the non-volatile storage (see nv.h).
 */
#include "nv.h"
#include "hal.h"

#include <string.h>

static nv_status_t s_nv;

void nv_init(void)
{
    memset(&s_nv, 0, sizeof s_nv);
    s_nv.fault = !hal_nv_ok();
}

bool nv_load(uint8_t slot, void *data, uint32_t size)
{
    return hal_nv_load(slot, data, size);
}

void nv_invalid(uint8_t slot)
{
    if (slot < 16u) s_nv.reset_mask |= (uint16_t)(1u << slot);
    s_nv.resets++;
}

bool nv_save(uint8_t slot, const void *data, uint32_t size)
{
    if (s_nv.fault) return false;
    bool ok = hal_nv_save(slot, data, size);
    if (ok) s_nv.pending = true;
    return ok;
}

void nv_step(bool standby_idle, uint32_t dt_ms)
{
    hal_nv_service(standby_idle);
    s_nv.pending = hal_nv_pending();
    s_nv.pending_ms = s_nv.pending ? s_nv.pending_ms + dt_ms : 0u;
    if (!hal_nv_ok()) s_nv.fault = true;
}

const nv_status_t *nv_status(void)
{
    return &s_nv;
}
