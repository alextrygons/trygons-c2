/*
 * tc_pulse.h - Paddlewheel speed sensor: pulses -> speed through water. Pure C (no iLLD): the board
 * (hal_tc375.c, HAL_PADDLE_FITTED) feeds it the free-running 16-bit edge counter of GPT12 timer T2 in
 * counter mode (T2INA, pin TBD in hal_pins.h); the PC simulation tests it directly (sim tests PW1-PW4).
 *
 * Every pulse_update call does a few float operations, never waits. The speed is measured over a
 * window of at least PULSE_WINDOW_MS (edges / time), filtered (tau), and judged:
 *   - edges in the window: speed = Hz / hz_per_kn (the sensor's K factor, e.g. ~4.9 Hz per kn for a
 *     common paddlewheel - TBD with the real sensor), above max_kn = NO DATA (noise / a wrong K)
 *   - no edge for a whole window: a boat that was already slow (below stop_kn) has stopped = 0 kn;
 *     from above stop_kn the pulses can't stop within one window (the boat slows over seconds): the
 *     cable is cut / the wheel is stuck = NO DATA, until pulses come again
 *   - a gap between two calls longer than 4 windows: no judgement, NO DATA until the next window
 * NO DATA = pulse_update returns false: the dead reckoning uses its throttle table instead (dr.c).
 */
#ifndef TC_PULSE_H
#define TC_PULSE_H

#include <stdbool.h>
#include <stdint.h>

#define PULSE_WINDOW_MS 500u

typedef struct {
    /* configuration */
    float    hz_per_kn;         /* pulses per second at 1 kn (K factor, TBD with the sensor) */
    float    max_kn;            /* above this = implausible = no data (40) */
    float    stop_kn;           /* below this no pulses within a window = stopped (1.0) */
    float    tau_s;             /* speed filter (1.0) */
    /* state */
    bool     have;              /* a first counter value seen */
    bool     valid;             /* the speed means something */
    uint16_t win_count;         /* counter at the window start */
    uint32_t win_ms;            /* window start */
    float    kn;                /* filtered speed */
    uint32_t windows, no_data;  /* (info / tests) */
} pulse_speed_t;

void pulse_init(pulse_speed_t *p, float hz_per_kn, float max_kn, float stop_kn, float tau_s);
/* count: the hardware edge counter (wraps at 65536), now_ms: hal_millis(). true + *kn while valid. */
bool pulse_update(pulse_speed_t *p, uint16_t count, uint32_t now_ms, float *kn);

#endif /* TC_PULSE_H */
