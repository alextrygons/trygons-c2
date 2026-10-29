/*
 * monitor.c - Independent safety monitor. See monitor.h.
 */
#include "monitor.h"

#include <math.h>
#include <string.h>

#include "hal.h"
#include "logrec.h"
#include "memplace.h"

SHARED_BEGIN()          /* core 0 writes, CPU1 reads: not cached (memplace.h) */
mon_shared_t g_mon;
SHARED_END()

void monitor_init(mon_t *m, uint32_t now_ms)
{
    memset(m, 0, sizeof *m);
    m->last_hb_ms = now_ms;
    m->last_ms = now_ms;
    m->init_ms = now_ms;
    m->node_bad_ms = now_ms;
    m->can_hw = hal_servos_can();       /* step 13d (HIGH-2): the board truth, before any publish of core 0 */
}

uint16_t monitor_safe_us(uint16_t us, uint16_t fallback)
{
    return (us >= MON_SAFE_MIN_US && us <= MON_SAFE_MAX_US) ? us : fallback;
}

void monitor_publish_safe(mon_shared_t *sh, uint16_t thr_idle_us, uint16_t gear_n_us)
{
    monitor_write_begin(sh);
    sh->thr_idle_us = thr_idle_us;
    sh->gear_n_us = gear_n_us;
    monitor_write_end(sh);
}

static void trip(mon_t *m, mon_shared_t *sh, mon_reason_t r, uint32_t now_ms)
{
    if (!m->tripped) {
        m->tripped = true;
        m->reason = r;
        m->trip_ms = now_ms;
        sh->reason = (uint8_t)r;
        sh->tripped = 1u;
        float lim = sh->fwd_engaged ? sh->fwd_limit : sh->rev_limit;
        (void)logrec_event(&g_logq1, now_ms, EV_MONITOR, (int32_t)r, (int32_t)(sh->throttle_pct * 10.0f),
                           (int32_t)(lim * 10.0f), 0);          /* CPU1's own log queue */
    }
}

void monitor_write_begin(mon_shared_t *sh)
{
    sh->seq = sh->seq + 1u;             /* odd: writing */
    hal_mem_barrier();
}

void monitor_write_end(mon_shared_t *sh)
{
    hal_mem_barrier();
    sh->seq = sh->seq + 1u;             /* even: a consistent set */
}

bool monitor_step(mon_t *m, mon_shared_t *sh, uint32_t now_ms)
{
    uint32_t dt = now_ms - m->last_ms;
    m->last_ms = now_ms;
    m->checks++;

    if (!m->tripped) {
        /* a consistent snapshot (seqlock): core 0 writing right now -> no
         * throttle / gear check this time (1 ms later again), heartbeat as always */
        uint32_t s1 = sh->seq;
        hal_mem_barrier();
        uint32_t hb = sh->heartbeat;
        float thr = sh->throttle_pct;
        uint16_t gear = sh->gear_pwm_us;
        float lim = sh->fwd_engaged ? sh->fwd_limit : sh->rev_limit;
        float idle_max = sh->idle_max_pct;
        bool gen = sh->gen_active != 0u;
        float gen_cap = sh->gen_cap_pct, fwd_lim = sh->fwd_limit;
        uint16_t gear_n = sh->gear_n_us;
        bool dp = sh->dp_active != 0u;
        float tb = sh->thr_bow_pct, ts = sh->thr_stern_pct, cb = sh->thr_bow_cap, cs = sh->thr_stern_cap;
        bool can = sh->can_servos != 0u, sth = sh->steer_have != 0u, gh = sh->gear_have != 0u;
        float sw = sh->steer_wire, gw = sh->gear_wire, gnw = sh->gear_n_wire;
        bool nok = sh->thr_node_ok != 0u, kil = sh->killed != 0u;
        uint8_t prof = sh->profile;
        uint32_t trav = sh->thr_travel_ms;
        hal_mem_barrier();
        bool consistent = (s1 & 1u) == 0u && sh->seq == s1;
        if (!consistent) m->torn++;
        if (consistent) {                       /* the DroneCAN values of the last consistent set */
            m->can = can;
            m->steer_have = sth;
            m->steer_wire = sw;
            m->gear_have = gh;
            m->gear_wire = gw;
            m->gear_n_wire = gnw;
            m->node_ok = nok;
            m->killed = kil;
            m->profile = prof;
            m->travel_ms = (trav < MON_THR_TRAVEL_MIN_MS) ? MON_THR_TRAVEL_MIN_MS
                         : (trav > MON_THR_TRAVEL_MAX_MS) ? MON_THR_TRAVEL_MAX_MS : trav;
        }

        /* 1. core 0 alive? Before its control loop runs only the boot
         *    timeout counts (a slow start must not trip), then the heartbeat */
        if (!m->started) {
            if (sh->running == MON_RUNNING) {
                m->started = true;
                m->last_hb = hb;
                m->last_hb_ms = now_ms;
                m->last_thr = thr;
                m->last_gear = gear;
                /* DroneCAN: after a boot the servo position is unknown - the envelope starts at full */
                m->env = (can && !m->boot_known) ? 100.0f : (thr > 0.0f ? thr : 0.0f);
                m->act_prev = hal_can_act_count();
                m->act_ms = now_ms;
            } else if (now_ms - m->init_ms > MON_BOOT_MS) {
                trip(m, sh, MON_TRIP_BOOT, now_ms);
            }
        } else if (hb != m->last_hb) {
            m->last_hb = hb;
            m->last_hb_ms = now_ms;
        } else if (now_ms - m->last_hb_ms > MON_HB_MS) {
            trip(m, sh, MON_TRIP_HEARTBEAT, now_ms);
        }
        if (!m->tripped && sh->test_trip == MON_TEST_TRIP) trip(m, sh, MON_TRIP_TEST, now_ms);

        /* 1c. DroneCAN: core 0's actuator task must keep running (its loop alive, its CAN path stuck = trip) */
        if (m->started && !m->tripped && m->can) {
            uint32_t ac = hal_can_act_count();
            if (ac != m->act_prev) {
                m->act_prev = ac;
                m->act_ms = now_ms;
            } else if (now_ms - m->act_ms > MON_CAN_TX_MS) {
                trip(m, sh, MON_TRIP_CAN_TX, now_ms);
            }
        }

        if (m->started && !m->tripped && consistent) {
            /* 1b. generator mode: throttle above idle in NEUTRAL only while core 0
             *     says gen_active, then at most min(generator cap, forward limit)
             *     (the limit below); the gear must be at neutral */
            if (gen) {
                int32_t dn = (int32_t)gear - (int32_t)gear_n;
                if (dn > (int32_t)MON_GEAR_DELTA || dn < -(int32_t)MON_GEAR_DELTA)
                    trip(m, sh, MON_TRIP_GENERATOR, now_ms);
                if (!sh->fwd_engaged) lim = (gen_cap < fwd_lim) ? gen_cap : fwd_lim;
                if (!(lim >= 0.0f)) lim = 0.0f;
            }
            /* 2. throttle inside the power limit (may only come down to it) */
            if (thr > lim + MON_MARGIN_PCT) {
                if (thr > m->last_thr + 0.5f) trip(m, sh, MON_TRIP_THROTTLE, now_ms);     /* going up: never */
                m->over_ms += dt;
                if (m->over_ms > MON_OVER_MS) trip(m, sh, MON_TRIP_THROTTLE, now_ms);
            } else {
                m->over_ms = 0u;
            }
            /* 2b. DP thrusters: thrust only while core 0 says dp_active, within each cap
             *     (NaN counts as thrust: !(|x| <= limit)) */
            {
                float ab = fabsf(tb), as = fabsf(ts);
                bool thrust = !(ab <= MON_THR_ZERO_PCT) || !(as <= MON_THR_ZERO_PCT);
                bool over = !(ab <= cb + MON_MARGIN_PCT) || !(as <= cs + MON_MARGIN_PCT);
                if (thrust && (!dp || over)) trip(m, sh, MON_TRIP_THRUSTER, now_ms);
            }
            /* 3. gear only moves at idle - of CPU1's own envelope: the command decayed at the worst-case servo
             *    speed (a slow servo is still open after the command went to idle) */
            {
                float down = m->env - 100.0f * (float)dt / (float)(m->travel_ms ? m->travel_ms : MON_THR_TRAVEL_MS);
                m->env = (thr > down) ? thr : down;
                if (m->env < 0.0f) m->env = 0.0f;
            }
            int32_t dg = (int32_t)gear - (int32_t)m->last_gear;
            if ((dg > (int32_t)MON_GEAR_DELTA || dg < -(int32_t)MON_GEAR_DELTA) &&
                (thr > idle_max + 0.5f || m->env > idle_max + 0.5f)) trip(m, sh, MON_TRIP_GEAR, now_ms);
            m->last_thr = thr;
            m->last_gear = gear;
        }
    }

    /* tripped: core 0 may still run (it is in STANDBY now): its heartbeat and the throttle node state it
     * publishes decide the gear frame (NEUTRAL only while it can still say the throttle node is OPERATIONAL) */
    if (m->tripped) {
        uint32_t s1 = sh->seq;
        hal_mem_barrier();
        uint32_t hb = sh->heartbeat;
        bool nok = sh->thr_node_ok != 0u;
        hal_mem_barrier();
        if ((s1 & 1u) == 0u && sh->seq == s1) m->node_ok = nok;
        if (hb != m->last_hb) {
            m->last_hb = hb;
            m->last_hb_ms = now_ms;
        }
    }

    if (!m->node_ok) m->node_bad_ms = now_ms;           /* step 13b (see node_bad_ms) */

    /* tripped: force the safe outputs (every time, latched in the HAL) */
    if (m->tripped) {
        hal_safe_outputs(monitor_safe_us(sh->thr_idle_us, MON_SAFE_THR_US),     /* never 0 (step 13d, HIGH-1) */
                         monitor_safe_us(sh->gear_n_us, MON_SAFE_GEAR_US));
        /* DroneCAN servos: CPU1's own safe frames (core 0's are cancelled by now) */
        /* step 13d (HIGH-2): a trip before core 0's control loop ran (MON_TRIP_BOOT: nothing published yet) uses the
         * HAL's own word (hal_servos_can) - the throttle node HOLDS the pre-reset command (OA-1) until CPU1's idle */
        bool can_now = m->can || (!m->started && m->can_hw);
        if (can_now && (m->safe_sets == 0u || now_ms - m->safe_ms >= MON_SAFE_FRAME_MS)) {
            hal_can_safe_t f;
            bool alive = m->started && now_ms - m->last_hb_ms <= MON_HB_MS;     /* core 0 can still say the node is OK */
            bool neutral = alive && m->node_ok && now_ms - m->trip_ms >= m->travel_ms &&
                           now_ms - m->node_bad_ms >= m->travel_ms;    /* (step 13b: also since the node's return) */
            /* once CPU1 sent neutral it HOLDS neutral (hold = what the node has now, never the pre-trip gear again;
             * found by storm I51 on the gear node, step 11g) */
            if (neutral) m->safe_neutral = true;
            neutral = m->safe_neutral;
            f.thr_wire = -1.0f;     /* IDLE at once: -1.0 = SERVO_CAN_IDLE_US (servo.c refuses another idle_us on CAN) */
            f.steer = m->steer_have;                                             /* hold (none if never commanded) */
            f.steer_wire = m->steer_wire;
            f.gear = neutral || m->gear_have;                                    /* hold, then neutral (OA-2) */
            f.gear_wire = neutral ? m->gear_n_wire : m->gear_wire;
            hal_can_safe_send(&f);
            m->safe_ms = now_ms;
            m->safe_sets++;
        }
    }

    /* the UNCREWED dead-man (OA-3): edges only while everything is right - else the timer relay stops the
     * engine by itself (also when CPU1 / the chip is dead or in reset: no edges either) */
    {
        bool alive = m->started && now_ms - m->last_hb_ms <= MON_HB_MS;
        bool frames = m->can && now_ms - m->act_ms <= MON_CAN_TX_MS;
        m->dm_on = m->profile == MON_PROFILE_UNCREWED && m->can && !m->tripped && alive && m->node_ok && frames &&
                   !m->killed;
        if (m->dm_on && now_ms - m->dm_ms >= MON_DM_TOGGLE_MS) {
            m->dm_ms = now_ms;
            m->dm_level = !m->dm_level;
            m->dm_edges++;
            hal_deadman_pulse(m->dm_level);
        }
    }
    return m->tripped;
}

void monitor_publish_steering_only(mon_shared_t *sh, uint16_t thr_idle_us, uint16_t gear_n_us)
{
    monitor_write_begin(sh);
    sh->throttle_pct = 0.0f;
    sh->fwd_limit = 100.0f;
    sh->rev_limit = 100.0f;
    sh->fwd_engaged = 0u;
    sh->gear_pwm_us = gear_n_us;
    sh->idle_max_pct = 1.0f;
    sh->thr_idle_us = thr_idle_us;
    sh->gear_n_us = gear_n_us;
    sh->gen_active = 0u;                /* big boat: no generator mode */
    sh->gen_cap_pct = 0.0f;
    sh->dp_active = 0u;                 /* big boat: no thrusters */
    sh->thr_bow_pct = 0.0f;
    sh->thr_stern_pct = 0.0f;
    sh->thr_bow_cap = 0.0f;
    sh->thr_stern_cap = 0.0f;
    sh->can_servos = 0u;                /* big boat: no DroneCAN servos, no dead-man (OD-12: unchanged) */
    sh->steer_have = 0u;
    sh->gear_have = 0u;
    sh->thr_node_ok = 0u;
    sh->thr_travel_ms = MON_THR_TRAVEL_MS;
    sh->profile = 0u;
    sh->killed = 0u;
    monitor_write_end(sh);
    sh->heartbeat = sh->heartbeat + 1u;
    sh->running = MON_RUNNING;
}

const char *monitor_reason_name(mon_reason_t r)
{
    static const char *const n[] = { "ok", "throttle above the power limit", "gear moved with throttle open",
                                     "core 0 heartbeat missing", "core 0 never started", "bench trip test",
                                     "generator: gear not in neutral", "thruster: thrust without DP / above the cap",
                                     "DroneCAN: actuator frames stopped" };
    return ((unsigned)r <= (unsigned)MON_TRIP_CAN_TX) ? n[r] : "?";
}
