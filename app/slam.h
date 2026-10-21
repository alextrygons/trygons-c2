/*
 * slam.h - Slamming: hull impacts measured with the BNO085 (vertical linear
 * acceleration, gravity removed, ~100-400 samples/s) and the throttle
 * reduction that follows from them. Pure C99, no HAL.
 *
 * Measured (every control step, all logged):
 *   peak      largest |vertical acceleration| this step, g
 *   impacts   a pulse above impact_g (one count per pulse, refractory time)
 *   rate      impacts in the last 60 s
 *   level     running average of the impact peaks (decays when calm), g
 *
 * State and reaction (the throttle is only ever REDUCED, never raised):
 *   CALM     no reduction; after calm_ms the cap comes back up slowly
 *            (+recover_pct every recover_ms) until it is gone
 *   YELLOW   level >= yellow_frac x limit: warning; the forward throttle
 *            cap steps down (step_pct) every step_ms while it stays yellow
 *   RED      one impact >= limit: red alarm + SAVE + the cap steps down at
 *            once. red_reset = 0 (STEP, default): that is all - the power
 *            limit stays (a sudden drop to 30 % can be dangerous in a
 *            following sea). red_reset = 1 (LIMIT_RESET): the power limit
 *            also drops to its start value like for every other alarm.
 * The cap never goes below min_cap_pct (the boat must stay controllable).
 * Override ("kick-down"): after a reduction the onboard driver eases off
 * (below 85 %) and then pushes to full (>= 95 %) for override_ms: the cap
 * is lifted (warning) until the throttle goes below the cap again. Just
 * staying at full throttle never overrides. Never the remote driver; the
 * power limit always stays.
 */
#ifndef SLAM_H
#define SLAM_H

#include <stdbool.h>
#include <stdint.h>

#define SLAM_CALM    0u
#define SLAM_YELLOW  1u
#define SLAM_RED     2u

#define SLAM_RING    64u

typedef struct {
    float    limit_g;           /* red: one impact this hard (3.0 g); settable (captain + driver) */
    float    red_reset;         /* RED action: 0 = STEP (default), 1 = LIMIT_RESET (power limit to 30 %) */
    float    yellow_frac;       /* yellow: level above this part of the limit (0.7) */
    float    impact_g;          /* a pulse above this counts as an impact (1.0 g) */
    uint32_t refractory_ms;     /* one impact per pulse (150 ms) */
    float    level_alpha;       /* level += alpha x (peak - level) per impact (0.25) */
    float    level_tau_s;       /* level decays with this time constant when no impacts (30 s) */
    float    step_pct;          /* cap step down (10 % of full throttle) */
    uint32_t step_ms;           /* while yellow: another step every (5 s) */
    float    min_cap_pct;       /* never below (30 %) */
    uint32_t calm_ms;           /* calm this long before recovering (30 s) */
    float    recover_pct;       /* then up by this (5 %) ... */
    uint32_t recover_ms;        /* ... every this long (10 s) */
    uint32_t override_ms;       /* kick-down held this long = override (1 s) */
    uint32_t lost_ms;           /* no samples this long after having some = sensor lost (1 s) */
} slam_config_t;

typedef struct {
    slam_config_t cfg;
    bool     seen;              /* the sensor has sent samples since power-up */
    bool     lost;              /* ... and then stopped (warning; no new reduction) */
    uint32_t last_sample_ms;
    float    peak;              /* this step */
    float    peak_hold;         /* since the telemetry last read it (link.c clears it) */
    float    level;
    uint16_t rate_pm;           /* impacts in the last 60 s */
    uint32_t impacts, reds;
    uint32_t ring[SLAM_RING];   /* impact times for the rate */
    uint8_t  ring_n, ring_i;
    bool     in_pulse;
    float    pulse_max;
    uint32_t refractory_until;
    uint8_t  state;             /* SLAM_CALM / YELLOW / RED */
    uint32_t red_until;         /* RED shown until (calm_ms after the last red impact) */
    bool     red_edge;          /* a red impact this step */
    bool     yellow_edge;       /* became yellow this step */
    float    cap_pct;           /* forward throttle cap, 100 = no reduction */
    uint32_t step_timer, calm_timer, recover_timer;
    bool     override;          /* the onboard driver overrides the cap */
    bool     kick_armed;        /* eased off below 85 % since the last reduction step */
    uint32_t over_ms;
    uint32_t steps, overrides;
} slam_t;

void slam_default_config(slam_config_t *c);
void slam_init(slam_t *s, const slam_config_t *c);

/* One control step: n new samples (g, + up), demand = the forward throttle
 * the driving mode wants (%), onboard = the onboard joystick is driving
 * (MANUAL / HDG LOCK). */
void slam_step(slam_t *s, const float *samples, uint32_t n, uint32_t now_ms, uint32_t dt_ms,
               float demand_pct, bool onboard);

/* Forward throttle limit from slamming (100 = none; override lifts it). */
float slam_cap(const slam_t *s);

#endif /* SLAM_H */
