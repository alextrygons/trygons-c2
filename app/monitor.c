/*
 * monitor.c - Independent safety monitor. See monitor.h.
 */
#include "monitor.h"

#include <string.h>

#include "hal.h"

mon_shared_t g_mon;

void monitor_init(mon_t *m, uint32_t now_ms)
{
    memset(m, 0, sizeof *m);
    m->last_hb_ms = now_ms;
    m->last_ms = now_ms;
}

static void trip(mon_t *m, mon_shared_t *sh, mon_reason_t r, uint32_t now_ms)
{
    if (!m->tripped) {
        m->tripped = true;
        m->reason = r;
        m->trip_ms = now_ms;
        sh->reason = (uint8_t)r;
        sh->tripped = 1u;
    }
}

bool monitor_step(mon_t *m, mon_shared_t *sh, uint32_t now_ms)
{
    uint32_t dt = now_ms - m->last_ms;
    m->last_ms = now_ms;
    m->checks++;

    if (!m->tripped) {
        uint32_t hb = sh->heartbeat;
        float thr = sh->throttle_pct;
        uint16_t gear = sh->gear_pwm_us;
        float lim = sh->fwd_engaged ? sh->fwd_limit : sh->rev_limit;

        /* 1. core 0 alive? */
        if (hb != m->last_hb) {
            m->last_hb = hb;
            m->last_hb_ms = now_ms;
            if (!m->started) {
                m->started = true;
                m->last_thr = thr;
                m->last_gear = gear;
            }
        } else if (now_ms - m->last_hb_ms > MON_HB_MS) {
            trip(m, sh, MON_TRIP_HEARTBEAT, now_ms);
        }

        if (m->started && !m->tripped) {
            /* 2. throttle inside the power limit (may only come down to it) */
            if (thr > lim + MON_MARGIN_PCT) {
                if (thr > m->last_thr + 0.5f) trip(m, sh, MON_TRIP_THROTTLE, now_ms);     /* going up: never */
                m->over_ms += dt;
                if (m->over_ms > MON_OVER_MS) trip(m, sh, MON_TRIP_THROTTLE, now_ms);
            } else {
                m->over_ms = 0u;
            }
            /* 3. gear only moves at idle */
            int32_t dg = (int32_t)gear - (int32_t)m->last_gear;
            if ((dg > (int32_t)MON_GEAR_DELTA || dg < -(int32_t)MON_GEAR_DELTA) &&
                thr > sh->idle_max_pct + 0.5f) trip(m, sh, MON_TRIP_GEAR, now_ms);
            m->last_thr = thr;
            m->last_gear = gear;
        }
    }

    /* tripped: force the safe outputs (every time, latched in the HAL) */
    if (m->tripped) hal_safe_outputs(sh->thr_idle_us, sh->gear_n_us);
    return m->tripped;
}

const char *monitor_reason_name(mon_reason_t r)
{
    static const char *const n[] = { "ok", "throttle above the power limit", "gear moved with throttle open",
                                     "core 0 heartbeat missing" };
    return ((unsigned)r <= (unsigned)MON_TRIP_HEARTBEAT) ? n[r] : "?";
}
