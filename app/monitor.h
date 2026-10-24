/*
 * monitor.h - Independent safety monitor (runs on CPU1 on the board; in
 * the simulator as a separate check step every 10 ms).
 *
 * Core 0 publishes what it commands (throttle %, limits, gear pulse) and a
 * heartbeat into g_mon after every control step. The monitor checks, with
 * its own simple rules and its own clock:
 *   - throttle above the power limit of its direction: rising above it ->
 *     trip at once; still above it after OVER_MS (only allowed while
 *     coming down after a limit drop) -> trip
 *   - gear servo moved while the throttle is not idle -> trip
 *   - heartbeat missing for more than HB_MS (200 ms) -> trip (core 0 frozen)
 *     - only once core 0 says "running" (MON_RUNNING, written by its control
 *       loop): a slow boot (flash, calibration, links) can't trip it; core 0
 *       that never starts trips after BOOT_MS (3 s)
 *   - bench trip test: core 0 writes MON_TEST_TRIP into test_trip (BENCH TEST
 *     mode, operator confirmed) -> a real trip, latched like any other, so the
 *     safe outputs can be checked with a scope (FIRST_TEST.md 3g)
 * Trip: hal_safe_outputs() forces throttle idle + gear neutral + steering
 * motor off + alarm + safety relay open, latched in the HAL until reset
 * (core 0 can't override it), and g_mon.tripped tells core 0 (-> STANDBY).
 *
 * Kept small and simple on purpose: no floats in loops, no dependencies on
 * the rest of the app.
 */
#ifndef MONITOR_H
#define MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#define MON_HB_MS        200u     /* heartbeat missing this long -> trip */
#define MON_BOOT_MS      3000u    /* core 0 not "running" this long after the monitor started -> trip */
#define MON_RUNNING      0xC0DE0A11u   /* g_mon.running: core 0's control loop runs */
#define MON_OVER_MS      2500u    /* above the limit (coming down) at most this long */
#define MON_MARGIN_PCT   1.0f     /* tolerance on the limit */
#define MON_GEAR_DELTA   20u      /* gear pulse change that counts as "gear moved", us */
#define MON_TEST_TRIP    0x7E57B3C1u   /* g_mon.test_trip: bench trip test requested */

typedef enum {
    MON_OK = 0,
    MON_TRIP_THROTTLE,          /* throttle above the power limit */
    MON_TRIP_GEAR,              /* gear moved with the throttle open */
    MON_TRIP_HEARTBEAT,         /* core 0 stopped */
    MON_TRIP_BOOT,              /* core 0 never started its control loop */
    MON_TRIP_TEST               /* bench trip test (operator request in BENCH TEST) */
} mon_reason_t;

/* Shared memory core 0 -> CPU1 (and the trip flag back). On the TC375
 * this lives in core 0's DSPR (memplace.h), which other cores read uncached. */
typedef struct {
    volatile uint32_t seq;              /* seqlock: odd while core 0 writes the values below (monitor_begin/end) */
    volatile uint32_t running;          /* MON_RUNNING once core 0's control loop runs (CPU1 clears it at start) */
    volatile uint32_t heartbeat;        /* +1 every control step */
    volatile float    throttle_pct;     /* throttle sent to the servo */
    volatile float    fwd_limit, rev_limit;
    volatile uint8_t  fwd_engaged;      /* gearbox confirmed in forward */
    volatile uint16_t gear_pwm_us;      /* gear servo pulse sent */
    volatile float    idle_max_pct;     /* "idle" for the gear check */
    volatile uint16_t thr_idle_us;      /* safe outputs to force */
    volatile uint16_t gear_n_us;
    volatile uint8_t  tripped;          /* monitor -> core 0 */
    volatile uint8_t  reason;           /* mon_reason_t */
    volatile uint32_t test_trip;        /* MON_TEST_TRIP = bench trip test (core 0 -> monitor) */
} mon_shared_t;

extern mon_shared_t g_mon;

typedef struct {
    uint32_t     last_hb;
    uint32_t     last_hb_ms;    /* when the heartbeat last changed */
    float        last_thr;
    uint16_t     last_gear;
    uint32_t     over_ms;
    uint32_t     last_ms;
    bool         started;       /* core 0 said "running" (heartbeat watched from then) */
    uint32_t     init_ms;       /* monitor start (boot timeout) */
    bool         tripped;
    mon_reason_t reason;
    uint32_t     trip_ms;
    uint32_t     checks;
    uint32_t     torn;          /* checks skipped: core 0 was writing (seqlock) */
} mon_t;

void monitor_init(mon_t *m, uint32_t now_ms);
/* One check (CPU1: every 1 ms; sim: every 10 ms). Returns true once tripped. */
bool monitor_step(mon_t *m, mon_shared_t *sh, uint32_t now_ms);
const char *monitor_reason_name(mon_reason_t r);

/* Core 0: around writing the values (seqlock): the monitor never checks a
 * half-written set (a new throttle with the old limit = false trip). */
void monitor_write_begin(mon_shared_t *sh);
void monitor_write_end(mon_shared_t *sh);

/* Core 0 of the big boat (rudder only, no throttle / gear servo): publish
 * after every control step - heartbeat + "running", throttle 0, limits
 * 100 %, gear pulse unchanged. The monitor then watches for a frozen core 0
 * (a trip stops the rudder motor). */
void monitor_publish_steering_only(mon_shared_t *sh, uint16_t thr_idle_us, uint16_t gear_n_us);

#endif /* MONITOR_H */
