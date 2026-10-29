/*
 * tc_pulse.c - Paddlewheel pulses -> speed through water. See tc_pulse.h. Pure C: also in the PC
 * simulation build (sim tests PW1-PW6).
 */
#include "tc_pulse.h"

#include <string.h>

void pulse_init(pulse_speed_t *p, float hz_per_kn, float max_kn, float stop_kn, float tau_s)
{
    memset(p, 0, sizeof *p);
    p->hz_per_kn = hz_per_kn > 0.01f ? hz_per_kn : 0.01f;
    p->max_kn = max_kn;
    p->stop_kn = stop_kn;
    p->tau_s = tau_s > 0.05f ? tau_s : 0.05f;
}

bool pulse_update(pulse_speed_t *p, uint16_t count, uint32_t now_ms, float *kn)
{
    if (!p->have) {
        p->have = true;
        p->win_count = count;
        p->win_ms = now_ms;
        return false;                           /* no window yet */
    }
    uint32_t dt_ms = now_ms - p->win_ms;
    if (dt_ms >= PULSE_WINDOW_MS) {
        uint16_t edges = (uint16_t)(count - p->win_count);     /* (16-bit wrap) */
        float dt = (float)dt_ms * 0.001f;
        float raw = (float)edges / dt / p->hz_per_kn;
        p->win_count = count;
        p->win_ms = now_ms;
        p->windows++;
        if (dt_ms > 4u * PULSE_WINDOW_MS) {
            /* a long gap between calls: no judgement, start again */
            p->valid = false;
        } else if (edges == 0u) {
            if (p->valid && p->kn < p->stop_kn) {
                p->kn = 0.0f;                   /* already slow: stopped */
            } else if (p->valid || p->kn >= p->stop_kn) {
                /* from speed to nothing within one window: the cable / the wheel, not the boat */
                if (p->valid) p->no_data++;
                p->valid = false;
            } else {
                /* no edge yet (power-up at rest, or after NO DATA): not trusted until a window WITH edges
                 * (step 13b: a wheel cut / fouled at rest would otherwise read a valid 0 kn for ever) */
                p->kn = 0.0f;
            }
        } else if (!(raw <= p->max_kn)) {
            if (p->valid) p->no_data++;
            p->valid = false;                   /* implausible: noise on the line or a wrong K */
        } else {
            if (!p->valid) p->kn = raw;         /* (back: no filter run-up from an old value) */
            float a = dt / (p->tau_s + dt);
            p->kn += a * (raw - p->kn);
            p->valid = true;
        }
    }
    if (!p->valid) return false;
    *kn = p->kn;
    return true;
}
