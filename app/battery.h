/*
 * battery.h - Supply voltage of the 12 V bank (CLAUDE.md section J).
 * Pure C99, no HAL.
 *
 * The bank = 3 x 12 V batteries in parallel, one supply for everything
 * (the starter battery is cut from it only while cranking). The HAL gives
 * the raw measurement every control step (hal_read_supply_voltage: a
 * divider on an EVADC input; false = not measured); this module filters it:
 *   bank_v       first order, tau ~30 s  - the alarm and the displays
 *   bank_v_fast  first order, tau ~1 s   - the generator checks (section K)
 * and judges LOW BATTERY: bank_v below low_v for low_ms (60 s) -> the
 * condition `low` (alarm SB_ALM2_LOW_BATTERY in smallboat.c: ADVISORY - no
 * mode change, the power limit stays: the way home may need the power);
 * it clears above low_v + hyst_v. Not measured for no_data_ms (5 s) = the
 * notice NO VOLTAGE DATA (never an alarm); the filters restart from the
 * first sample when the data comes back. A sample that is not finite or
 * out of range counts as "not measured".
 * Nothing here ever touches the throttle, the gear or the steering.
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float    low_v;         /* LOW BATTERY below this (11.8 V TO CONFIRM; parameter batt_low_v) */
    float    hyst_v;        /* ... and clears above low_v + this (0.3 V) */
    uint32_t low_ms;        /* below low_v this long before the condition holds (60 s) */
    float    tau_slow_s;    /* bank_v filter time constant (30 s) */
    float    tau_fast_s;    /* bank_v_fast filter time constant (1 s) */
    uint32_t no_data_ms;    /* not measured this long -> NO VOLTAGE DATA notice (5 s) */
    float    max_v;         /* a sample above this is not a measurement (60 V) */
} batt_config_t;

typedef struct {
    batt_config_t cfg;
    bool     valid;         /* bank_v / bank_v_fast mean something (measured, not stale) */
    bool     no_data;       /* notice: not measured for no_data_ms (no alarm) */
    float    raw_v;         /* the last measurement (0 while none) */
    float    bank_v;        /* slow filter (tau_slow_s) */
    float    bank_v_fast;   /* fast filter (tau_fast_s) */
    uint32_t gap_ms;        /* since the last measurement */
    uint32_t low_ms;        /* how long bank_v has been below low_v (while not low yet) */
    bool     low;           /* LOW BATTERY condition (with hysteresis) */
    bool     low_edge;      /* the condition started this step */
    uint32_t samples, lows; /* counters (tests / info) */
} batt_t;

void batt_default_config(batt_config_t *c);
void batt_init(batt_t *b, const batt_config_t *c);
/* One control step: measured = the HAL had a sample (volts), dt_ms = step. */
void batt_step(batt_t *b, bool measured, float volts, uint32_t dt_ms);

#endif /* BATTERY_H */
