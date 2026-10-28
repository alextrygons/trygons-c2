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
 *   - generator mode (CLAUDE.md K): while core 0 says gen_active the throttle
 *     may be above idle with the gear in NEUTRAL, at most min(gen_cap_pct,
 *     forward limit) - instead of the reverse limit that applies out of
 *     forward gear; the gear pulse not at neutral with gen_active -> trip
 *     (MON_TRIP_GENERATOR). Without gen_active the existing rules apply.
 *   - DP thrusters (CLAUDE.md P): a thrust output (|bow| or |stern| above
 *     MON_THR_ZERO_PCT) while core 0 does not say dp_active, or above the
 *     published cap of that thruster (+ MON_MARGIN_PCT) -> trip
 *     (MON_TRIP_THRUSTER). The safe state sets both thrusters to 0.
 *   - its OWN throttle envelope (step 11e): the last throttle command decayed at
 *     the worst-case servo speed (100 % per thr_travel_s, published by core 0,
 *     kept inside MON_THR_TRAVEL_MIN..MAX_MS): "gear moved with the throttle
 *     up" uses the envelope, not the command (a slow servo is still open after
 *     the command went to idle)
 *   - DroneCAN servos (CLAUDE.md T, step 11e): core 0's actuator task stopped
 *     (no tick for MON_CAN_TX_MS while running) -> trip (MON_TRIP_CAN_TX: its
 *     loop alive, its CAN path stuck)
 * Trip: hal_safe_outputs() forces throttle idle + gear neutral + steering
 * motor off + both thrusters 0 + alarm + safety relay open, latched in the HAL until reset
 * (core 0 can't override it), and g_mon.tripped tells core 0 (-> STANDBY).
 * DroneCAN servos: the latch FIRST, core 0's pending frames cancelled, then
 * CPU1's safe frames every MON_SAFE_FRAME_MS (hal_can_safe_send): throttle IDLE
 * at once; steering = core 0's last steering command (hold; none if it never
 * commanded one, OD-7); gear = HOLD its last command until thr_travel_s after
 * the trip, then NEUTRAL - only while the throttle node is OPERATIONAL and core
 * 0 alive to say so (else HOLD for good: OA-2). The servo power is never cut.
 * UNCREWED dead-man (OA-3, TO CONFIRM): CPU1 toggles hal_deadman_pulse every
 * MON_DM_TOGGLE_MS only while UNCREWED + no trip + core 0 alive + the throttle
 * node OPERATIONAL + actuator frames going out + no ENGINE KILL latched; any of
 * them false = no pulses = the timer relay stops the engine (~1.5 s).
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
#define MON_THR_ZERO_PCT 0.5f     /* a thruster output above this counts as "thrust" */
#define MON_CAN_TX_MS    100u     /* DroneCAN: no actuator task tick this long while running -> trip */
#define MON_SAFE_FRAME_MS 20u     /* CPU1's safe frames: this often after a trip */
#define MON_DM_TOGGLE_MS 50u      /* the dead-man pulse: an edge this often while allowed */
#define MON_THR_TRAVEL_MS     6000u   /* worst-case throttle servo travel (core 0 publishes the one in use) */
#define MON_THR_TRAVEL_MIN_MS 1000u
#define MON_THR_TRAVEL_MAX_MS 30000u
#define MON_PROFILE_UNCREWED  2u  /* = SB_PROFILE_UNCREWED (smallboat.h) */

typedef enum {
    MON_OK = 0,
    MON_TRIP_THROTTLE,          /* throttle above the power limit */
    MON_TRIP_GEAR,              /* gear moved with the throttle open */
    MON_TRIP_HEARTBEAT,         /* core 0 stopped */
    MON_TRIP_BOOT,              /* core 0 never started its control loop */
    MON_TRIP_TEST,              /* bench trip test (operator request in BENCH TEST) */
    MON_TRIP_GENERATOR,         /* generator mode: gear not in neutral while gen_active */
    MON_TRIP_THRUSTER,          /* DP thrusters: thrust without dp_active, or above the cap */
    MON_TRIP_CAN_TX             /* DroneCAN: core 0's actuator task stopped (its CAN path stuck) */
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
    volatile uint8_t  gen_active;       /* generator mode runs / stops: throttle above idle in NEUTRAL allowed ... */
    volatile float    gen_cap_pct;      /* ... up to this (never above the forward limit) */
    volatile uint8_t  dp_active;        /* DP runs: thruster outputs allowed ... */
    volatile float    thr_bow_pct, thr_stern_pct;   /* thruster outputs sent (-100..100) */
    volatile float    thr_bow_cap, thr_stern_cap;   /* ... at most these (boat file max_pct; 0 = not fitted) */
    /* DroneCAN servos (step 11e) */
    volatile uint8_t  can_servos;       /* the servos are DroneCAN nodes */
    volatile uint8_t  steer_have;       /* a steering frame went out since the boot (OD-7) ... */
    volatile float    steer_wire;       /* ... the last one (CPU1 replays it: hold) */
    volatile uint8_t  gear_have;        /* a gear frame went out since the boot ... */
    volatile float    gear_wire;        /* ... the last one (hold) */
    volatile float    gear_n_wire;      /* neutral (after thr_travel_s, with the throttle node OPERATIONAL) */
    volatile uint8_t  thr_node_ok;      /* the throttle node OPERATIONAL */
    volatile uint32_t thr_travel_ms;    /* thr_travel_s in use (CPU1's throttle envelope) */
    volatile uint8_t  profile;          /* the latched profile (MON_PROFILE_UNCREWED: the dead-man) */
    volatile uint8_t  killed;           /* the software ENGINE KILL is latched */
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
    /* step 11e */
    float        env;           /* CPU1's own throttle envelope (worst-case servo speed), % */
    uint32_t     act_prev, act_ms;      /* core 0's actuator task: last tick count / when it changed */
    uint32_t     safe_ms;       /* the last safe frame set */
    uint32_t     safe_sets;     /* safe frame sets sent */
    bool         safe_neutral;  /* CPU1 has sent NEUTRAL since the trip: it stays neutral (step 11g - a throttle node
                                 * lost afterwards must never bring back the old pre-trip gear) */
    bool         dm_level;      /* the dead-man pulse level */
    uint32_t     dm_ms;
    uint32_t     dm_edges;
    bool         dm_on;         /* pulsing now (the conditions held) */
    /* the last consistent snapshot of the DroneCAN values */
    bool         can;
    bool         steer_have, gear_have, node_ok, killed;
    float        steer_wire, gear_wire, gear_n_wire;
    uint8_t      profile;
    uint32_t     travel_ms;
    bool         boot_known;    /* sim tests only ("all --can", sb_test_boot_idle): the throttle known at idle at
                                   the boot - never set on the board (CPU1 can't know it there) */
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
