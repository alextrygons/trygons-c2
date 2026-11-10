/*
 * accel.h - Step FW-TR: the boat's acceleration (BNO085 linear acceleration, gravity removed) on three axes for the
 * telemetry (PROTO_T_ACCEL) and the log (event 31 ACCEL). Pure C99, no HAL calls.
 *
 * DISPLAY / LOGGING ONLY: nothing in the control path (throttle, limits, modes, alarms, slamming) reads it.
 * Axes [0] longitudinal (+ forward), [1] lateral (+ starboard), [2] vertical (+ up), g.
 * Per window: average = sum / n, peak = the sample with the largest magnitude (signed).
 *   step  the samples of this control step (filled by smallboat.c from hal_read_accel3)
 *   run   the 1 s window being filled; sec = the last COMPLETE 1 s window (sec_seq + 1 each time)
 * Each link folds `step` into its own telemetry-period window (link.c).
 */
#ifndef ACCEL_H
#define ACCEL_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#define ACCEL_SEC_MS 1000u

typedef struct {
    hal_accel3_t step;          /* this step's samples */
    hal_accel3_t run;           /* the 1 s window being filled */
    hal_accel3_t sec;           /* the last complete 1 s window (n 0 = no sample in it) */
    uint32_t     run_ms;        /* when `run` began (0 = not yet) */
    uint32_t     sec_ms;        /* when `sec` was completed (0 = never) */
    uint16_t     sec_seq;       /* + 1 per completed window (the log event) */
} accel_t;

void accel_clear(hal_accel3_t *w);
/* Fold src into w (sums, counts, signed max-magnitude peaks); a non-finite src is ignored. */
void accel_fold(hal_accel3_t *w, const hal_accel3_t *src);
/* Average of axis i (0 if n is 0). */
float accel_avg(const hal_accel3_t *w, unsigned i);
/* One control step: `a->step` already holds this step's samples. */
void accel_step(accel_t *a, uint32_t now_ms);
/* milli-g for a frame / log field: rounded, clamped to -32767..32767, non-finite = 0. */
int16_t accel_mg(float g);

#endif /* ACCEL_H */
