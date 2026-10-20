/*
 * timing.c - Cycle execution time statistics. See timing.h.
 */
#include "timing.h"

#include <string.h>

timing_t g_timing;

void cyc_init(cyc_t *c, uint32_t period_us, uint32_t window)
{
    memset((void *)c, 0, sizeof *c);
    c->period_us = period_us;
    c->window = window ? window : 1u;
    c->w_min = 0xFFFFFFFFu;
}

void timing_init_all(void)
{
    cyc_init(&g_timing.core0, TIMING_CORE0_PERIOD_US, 100u);       /* 5 s windows */
    cyc_init(&g_timing.monitor, TIMING_MONITOR_PERIOD_US, 1000u);  /* 1 s */
    cyc_init(&g_timing.logger, TIMING_LOGGER_PERIOD_US, 100u);     /* 1 s */
}

bool cyc_record(cyc_t *c, uint32_t exec_us)
{
    if (exec_us < c->w_min) c->w_min = exec_us;
    if (exec_us > c->w_max) c->w_max = exec_us;
    c->w_sum += exec_us;
    c->w_n++;
    c->cycles = c->cycles + 1u;
    c->last_us = exec_us;
    if (exec_us > c->max_us) c->max_us = exec_us;
    if (c->w_n >= c->window) {
        c->pub_min_us = c->w_min;
        c->pub_avg_us = (uint32_t)(c->w_sum / c->w_n);
        c->pub_max_us = c->w_max;
        c->w_min = 0xFFFFFFFFu;
        c->w_max = 0u;
        c->w_sum = 0u;
        c->w_n = 0u;
    }
    if (exec_us > c->period_us) {
        c->overruns = c->overruns + 1u;
        return true;
    }
    return false;
}

float cyc_max_pct(const cyc_t *c)
{
    return c->period_us ? 100.0f * (float)c->pub_max_us / (float)c->period_us : 0.0f;
}

float cyc_avg_pct(const cyc_t *c)
{
    return c->period_us ? 100.0f * (float)c->pub_avg_us / (float)c->period_us : 0.0f;
}
