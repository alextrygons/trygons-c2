/*
 * timing.h - Execution time of each cycle (STM timer, microseconds).
 *
 * One cyc_t per loop: core 0 control step (50 ms period), CPU1 safety
 * monitor (1 ms), core 2 logger. For every window of `window` cycles the
 * min / average / max are published (pub_*), the all-time max is kept, and
 * a cycle longer than its period counts as an OVERRUN (core 0: alarm).
 *
 * The three counters live in shared memory (g_timing) so core 0 can put
 * all of them into the telemetry and the log.
 */
#ifndef TIMING_H
#define TIMING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t period_us;         /* the loop period */
    uint32_t window;            /* cycles per published window */
    /* running window */
    uint32_t w_min, w_max, w_n;
    uint64_t w_sum;
    /* published (last full window) */
    volatile uint32_t pub_min_us, pub_avg_us, pub_max_us;
    volatile uint32_t max_us;       /* all-time max */
    volatile uint32_t overruns;     /* cycles longer than period_us */
    volatile uint32_t cycles;
} cyc_t;

typedef struct {
    cyc_t core0;                /* control step (sb_step + link_step) */
    cyc_t monitor;              /* CPU1 safety monitor */
    cyc_t logger;               /* core 2 logger */
} timing_t;

extern timing_t g_timing;

#define TIMING_CORE0_PERIOD_US   50000u     /* 20 Hz control loop */
#define TIMING_MONITOR_PERIOD_US 1000u      /* 1 kHz safety monitor */
#define TIMING_LOGGER_PERIOD_US  10000u     /* logger loop */

/* All three with their periods (core 0 calls it before the others start). */
void timing_init_all(void);
void cyc_init(cyc_t *c, uint32_t period_us, uint32_t window);
/* Record one cycle that took exec_us. Returns true if it overran. */
bool cyc_record(cyc_t *c, uint32_t exec_us);
/* Load in % of the period (published max / average). */
float cyc_max_pct(const cyc_t *c);
float cyc_avg_pct(const cyc_t *c);

#endif /* TIMING_H */
