/*
 * accel.c - Step FW-TR: 3-axis acceleration windows for the telemetry and the log (see accel.h).
 * DISPLAY / LOGGING ONLY. A few float operations per control step, no loop over samples, no wait.
 */
#include "accel.h"

#include <math.h>
#include <string.h>

void accel_clear(hal_accel3_t *w)
{
    memset(w, 0, sizeof *w);
}

void accel_fold(hal_accel3_t *w, const hal_accel3_t *src)
{
    if (src->n == 0u) return;
    for (unsigned i = 0; i < 3u; i++)
        if (!isfinite(src->sum[i]) || !isfinite(src->peak[i])) return;
    if (w->n > 0xFFFF0000u) return;                     /* never wraps */
    for (unsigned i = 0; i < 3u; i++) {
        w->sum[i] += src->sum[i];
        if (w->n == 0u || fabsf(src->peak[i]) > fabsf(w->peak[i])) w->peak[i] = src->peak[i];
    }
    w->n += src->n;
}

float accel_avg(const hal_accel3_t *w, unsigned i)
{
    return (w->n && i < 3u) ? w->sum[i] / (float)w->n : 0.0f;
}

void accel_step(accel_t *a, uint32_t now_ms)
{
    uint32_t now = now_ms ? now_ms : 1u;
    if (a->run_ms == 0u) a->run_ms = now;
    accel_fold(&a->run, &a->step);
    if ((uint32_t)(now - a->run_ms) >= ACCEL_SEC_MS) {
        a->sec = a->run;
        a->sec_ms = now;
        a->sec_seq++;
        accel_clear(&a->run);
        a->run_ms = now;
    }
}

int16_t accel_mg(float g)
{
    if (!isfinite(g)) return 0;
    float mg = g * 1000.0f;
    if (mg > 32767.0f) return 32767;
    if (mg < -32767.0f) return -32767;
    return (int16_t)lroundf(mg);
}
