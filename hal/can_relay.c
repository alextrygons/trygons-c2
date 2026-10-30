/*
 * can_relay.c - The relay board on DroneCAN (see can_relay.h). Only this file sends
 * hardpoint.Command frames (tools/check.sh).
 */
#include "can_relay.h"

#include <string.h>

#include "dronecan.h"
#include "hal.h"

static cr_config_t s_cfg;
static bool        s_up;
static uint32_t    s_mask;                  /* bit n-1 for every hardpoint of the board */
static uint32_t    s_on, s_known;           /* the COMMANDED states (known = commanded / kept / the board booted OFF) */
static uint8_t     s_burst[CR_CH_MAX];      /* frames of the new state still to send */
static uint32_t    s_until[CR_CH_MAX];      /* pulse end (0 = no pulse) */
static uint32_t    s_t0, s_last_tx;
static bool        s_seen;                  /* the board's NodeStatus seen since the start */
static uint16_t    s_restarts;
static uint32_t    s_cut;                   /* step 13d: ON bursts cut (bit n-1), for relays.c (cr_take_cut) */
static uint32_t    s_hold;                  /* step 13d: no OFF burst for a cut ON on these (cr_hold_mask) */
static uint32_t    s_retry;                 /* step 13h (W42): cut ON bursts that may be finished when the board is back */
static uint32_t    s_want;                  /* step 13h: relays.c still wants these ON (cr_on_want) */
static uint32_t    s_odrop;                 /* step 13h: retries given up (window / board reboot), for relays.c */
static uint32_t    s_on_ms[CR_CH_MAX];      /* step 13h: the time of the ON command (the retry window counts from it) */
static uint32_t    s_ref_up_s, s_ref_ms, s_ref_stat;   /* step 13h2: the last NodeStatus handled: its uptime, our time
                                                          of its reception, the status count */
static cr_stats_t  s_st;

static uint32_t bit(uint8_t i)
{
    return 1u << i;
}

void cr_init(const cr_config_t *c)
{
    s_cfg = *c;
    if (s_cfg.channels > CR_CH_MAX) s_cfg.channels = CR_CH_MAX;
    if (s_cfg.channels > DC_HP_MAX) s_cfg.channels = DC_HP_MAX;
    s_mask = (s_cfg.channels >= 32u) ? 0xFFFFFFFFu : ((1u << s_cfg.channels) - 1u);
    s_on = s_known = 0u;
    if (s_cfg.keep_valid && !s_cfg.power_on) {          /* a warm reset: what was commanded before it (held by the board) */
        s_known = s_cfg.keep_known & s_mask;
        s_on = s_cfg.keep_on & s_known;
    }
    memset(s_burst, 0, sizeof s_burst);                 /* nothing is sent after a boot (R-2: the board holds) */
    memset(s_until, 0, sizeof s_until);
    memset(&s_st, 0, sizeof s_st);
    s_t0 = hal_millis();
    s_last_tx = s_t0 - CR_PERIOD_MS;
    s_seen = false;
    s_restarts = 0u;
    s_cut = 0u;
    s_hold = 0u;
    s_retry = s_want = s_odrop = 0u;
    memset(s_on_ms, 0, sizeof s_on_ms);
    s_ref_up_s = s_ref_ms = s_ref_stat = 0u;
    s_up = s_cfg.channels != 0u;
}

static bool node(hal_can_node_t *n)
{
    return dc_node(HAL_CAN_ROLE_RELAY, n);
}

static bool bus_fault(void)
{
    hal_can_bus_t b;
    dc_bus(&b);
    return b.fault;
}

/* the board is there: its NodeStatus young enough, OPERATIONAL, health below ERROR (WARNING = notice) */
static bool present(void)
{
    hal_can_node_t n;
    if (!s_up || bus_fault() || !node(&n) || !n.seen) return false;
    return n.age_ms <= s_cfg.lost_ms && n.mode == DC_MODE_OPERATIONAL && n.health < DC_HEALTH_ERROR;
}

bool cr_ok(void)
{
    hal_can_node_t n;
    if (!s_up || bus_fault()) return false;
    if (!node(&n) || !n.seen) return (uint32_t)(hal_millis() - s_t0) < s_cfg.lost_ms;   /* time to be seen */
    return present();
}

static void end_pulses(uint32_t now)
{
    for (uint8_t i = 0; i < s_cfg.channels; i++) {
        if (s_until[i] == 0u || (int32_t)(now - s_until[i]) < 0) continue;
        s_until[i] = 0u;                                /* the pulse is over: OFF (a burst), then the board holds it */
        s_on &= ~bit(i);
        s_burst[i] = CR_BURST_FRAMES;
    }
}

/* step 13h (W42, owner 2026-10-29): the cut ON bursts waiting for the board - dropped silently when relays.c no longer
 * wants them ON (an operator OFF / a new command clears them in cr_set / cr_pulse at once; a table change or a timer
 * through the want mask), given up after CR_ON_RETRY_MS from the ON command (the state stays UNKNOWN, nothing sent,
 * relays.c is told: cr_take_on_drop) */
static void retry_service(uint32_t now)
{
    s_retry &= s_want;
    for (uint8_t i = 0; s_retry && i < s_cfg.channels; i++) {
        if (!(s_retry & bit(i)) || (uint32_t)(now - s_on_ms[i]) <= CR_ON_RETRY_MS) continue;
        s_retry &= ~bit(i);
        s_odrop |= bit(i);
        s_st.on_given_up++;
    }
}

static uint32_t popcount(uint32_t v)
{
    uint32_t c = 0u;
    for (; v; v &= v - 1u) c++;
    return c;
}

/* step 13h2 (review 3bef4c8 MED-1): a NodeStatus newer than the last one handled whose uptime did not grow with our
 * clock = the board rebooted (also when the uptime is not below the old one: a young board, silent after its reboot
 * for longer than it had been up). Margins: CR_UPTIME_* in can_relay.h (a false "rebooted" only makes ON states
 * unknown and drops cut ONs = fail-safe) */
static bool uptime_lag(const hal_can_node_t *n, uint32_t now)
{
    if (!s_seen || !n->seen || n->statuses == s_ref_stat || n->age_ms == 0xFFFFFFFFu) return false;
    uint64_t el = (uint32_t)((now - n->age_ms) - s_ref_ms);
    uint64_t got = (uint64_t)n->uptime_s * 1000u + CR_UPTIME_RES_MS + CR_UPTIME_JITTER_MS + el / CR_UPTIME_DRIFT_DIV;
    return got < (uint64_t)s_ref_up_s * 1000u + el;
}

static void ref_take(const hal_can_node_t *n, uint32_t now)
{
    s_ref_up_s = n->uptime_s;
    s_ref_ms = now - n->age_ms;
    s_ref_stat = n->statuses;
}

/* a board reboot: no burst / pulse / cut ON goes on; EVERY channel that was ON (or a cut ON waiting) is reported
 * through cr_take_on_drop (review 3bef4c8 LOW-2: relays.c clears its command for all of them) */
static void board_rebooted(void)
{
    uint32_t was = (s_on | s_retry) & s_mask;
    memset(s_burst, 0, sizeof s_burst);
    memset(s_until, 0, sizeof s_until);
    s_odrop |= was;
    s_st.on_given_up += popcount(s_retry);              /* (review LOW-3: per relay, like the other places) */
    s_retry = 0u;
}

void cr_service(void)
{
    if (!s_up) return;
    uint32_t now = hal_millis();
    hal_can_node_t n;
    if (node(&n) && n.seen) {
        if (!s_seen) {
            s_seen = true;
            s_restarts = n.restarts;
            /* the board booted after the AURIX's start - its power-on OR a warm reset (step 11g: the states kept
             * from before a warm reset are stale then) - its uptime is not longer than ours: it powered up with
             * every relay OFF (R-2). Nothing ON can have been commanded meanwhile (cr_set / cr_pulse refuse ON
             * while the board is not there), so every state is known OFF. */
            uint64_t up_ms = (uint64_t)n.uptime_s * 1000u, since = (uint64_t)(uint32_t)(now - s_t0);
            bool warm = s_cfg.keep_valid && !s_cfg.power_on;
            if (!warm && up_ms <= since + CR_BOOT_MARGIN_MS) {
                s_known = s_mask;
                s_on = 0u;
                memset(s_until, 0, sizeof s_until);
            } else if (warm && up_ms + 1000u < since) {
                /* step 13b: after a warm reset only a reboot PROVEN after our start (the uptime is whole seconds:
                 * the real one is below uptime_s + 1 s) clears the kept states to OFF */
                s_known = s_mask;
                s_on = 0u;
                memset(s_until, 0, sizeof s_until);
            } else if (warm && up_ms <= since + CR_BOOT_MARGIN_MS) {
                /* step 13b (review LOW-2): it booted around our warm reset - before or after can't be told: a kept
                 * OFF is OFF either way, a kept ON is UNKNOWN (the pair rule blocks its partner), never "OFF" */
                s_known &= ~s_on;
                s_on = 0u;
                memset(s_until, 0, sizeof s_until);
            }
        } else if (n.restarts != s_restarts) {
            /* the board rebooted (its uptime went back): it powered up OFF (R-2) - every state OFF, nothing re-sent
             * (a relay is never switched back ON by itself) */
            s_restarts = n.restarts;
            /* step 13h (W42): a cut ON is never finished on a board that rebooted meanwhile - given up, nothing sent
             * (the relay is known OFF like every other one; the operator presses again) */
            board_rebooted();
            s_on = 0u;
            s_known = s_mask;
            s_st.board_reboots++;
        } else if (uptime_lag(&n, now)) {
            /* step 13h2 (review 3bef4c8 MED-1): its uptime did not grow with our clock = it rebooted although the
             * uptime did not go back. Not PROVEN like an uptime back (a clock fault could look alike): a commanded
             * ON is UNKNOWN, an OFF stays OFF (OFF either way); no cut ON is finished, nothing sent */
            board_rebooted();
            s_known &= ~s_on;
            s_on = 0u;
            s_st.board_reboots_t++;
        }
        ref_take(&n, now);
    }
    retry_service(now);
    end_pulses(now);
}

void cr_tick(void)
{
    if (!s_up) return;
    uint32_t now = hal_millis();
    s_st.ticks++;
    end_pulses(now);
    if ((uint32_t)(now - s_last_tx) < CR_PERIOD_MS) return;
    s_last_tx = now;
    bool there = present();
    /* step 13h: the board came back with a new uptime-went-back mark cr_service has not handled yet = it rebooted:
     * no cut ON is finished (cr_service gives them up) */
    hal_can_node_t nd;
    bool rebooted = node(&nd) && nd.seen && s_seen && (nd.restarts != s_restarts || uptime_lag(&nd, now));
    if (there && !rebooted && s_retry) {
        retry_service(now);
        for (uint8_t i = 0; s_retry && i < s_cfg.channels; i++) {
            if (!(s_retry & bit(i))) continue;
            /* W42 (b): still wanted ON by relays.c, within CR_ON_RETRY_MS of the command, no board reboot: the ON
             * burst is sent again in full */
            s_retry &= ~bit(i);
            s_on |= bit(i);
            s_known |= bit(i);
            s_burst[i] = CR_BURST_FRAMES;
            s_st.on_retried++;
        }
    }
    for (uint8_t i = 0; i < s_cfg.channels; i++) {
        if (!there && s_until[i] != 0u) {
            /* step 11g: a pulse (ON at 50 Hz) to a board that is not there (bus fault, silent, not OPERATIONAL)
             * ends NOW - never an ON that arrives late when the board is back; the OFF burst follows */
            s_until[i] = 0u;
            s_on &= ~bit(i);
            s_burst[i] = CR_BURST_FRAMES;
        }
        if (!there && s_burst[i] != 0u && (s_on & bit(i))) {
            /* step 11g: the state is unknown (some frames may have reached it and the board holds what it got, R-2).
             * Step 13h (W42, owner 2026-10-29): NEVER an OFF by itself (the 13b OFF burst is gone) - nothing is
             * sent; the ON burst is finished when the board is back ONLY while relays.c still wants it ON, within
             * CR_ON_RETRY_MS of the command and without a board reboot in between (cr_tick above / retry_service /
             * cr_service), else given up: UNKNOWN, nothing sent. A hold channel (the SHUTOFF relay, step 13d) is
             * never retried here: relays.c queues its ON again (cr_take_cut, RELAY_SHUTOFF_RETRY_MS) */
            s_on &= ~bit(i);
            s_known &= ~bit(i);
            s_burst[i] = 0u;
            if (!(s_hold & bit(i)) && (s_want & bit(i))) s_retry |= bit(i);
            else if (!(s_hold & bit(i))) { s_odrop |= bit(i); s_st.on_given_up++; }
            s_st.dropped++;
            s_cut |= bit(i);
            continue;
        }
        bool pulse = s_until[i] != 0u;
        if (!pulse && s_burst[i] == 0u) continue;
        if (dc_send_hardpoint(i, (s_on & bit(i)) ? 1u : 0u)) {
            s_st.frames++;
            if (!pulse && there && s_burst[i] > 0u) {               /* (an absent board: sent again when it is back) */
                s_burst[i]--;
                if (s_burst[i] == 0u && !(s_on & bit(i))) s_known |= bit(i);   /* an OFF delivered: known OFF */
            }
        } else {
            s_st.refused++;
        }
    }
}

bool cr_set(uint8_t n, bool on)
{
    if (!s_up || n < 1u || n > s_cfg.channels) return false;
    if (on && !present()) return false;                 /* step 11g: never an ON to a board that is not there */
    uint8_t i = (uint8_t)(n - 1u);
    s_until[i] = 0u;
    s_retry &= ~bit(i);                                 /* step 13h: a new command replaces a cut ON's retry */
    s_odrop &= ~bit(i);
    if (on) s_on_ms[i] = hal_millis();
    if (on) s_on |= bit(i); else s_on &= ~bit(i);
    s_known |= bit(i);
    s_burst[i] = CR_BURST_FRAMES;
    s_st.sets++;
    return true;
}

bool cr_take_cut(uint8_t n)
{
    if (!s_up || n < 1u || n > s_cfg.channels) return false;
    uint32_t b = bit((uint8_t)(n - 1u));
    if (!(s_cut & b)) return false;
    s_cut &= ~b;
    return true;
}

void cr_hold_mask(uint32_t mask)
{
    s_hold = mask & s_mask;
}

uint32_t cr_hold(void)
{
    return s_hold;
}

uint32_t cr_cuts(void)
{
    return s_cut;
}

void cr_on_want(uint32_t mask)
{
    s_want = mask & s_mask;
    s_retry &= s_want;                                  /* (no longer wanted: dropped, nothing sent) */
}

bool cr_take_on_drop(uint8_t n)
{
    if (!s_up || n < 1u || n > s_cfg.channels) return false;
    uint32_t b = bit((uint8_t)(n - 1u));
    if (!(s_odrop & b)) return false;
    s_odrop &= ~b;
    return true;
}

uint32_t cr_retries(void)
{
    return s_retry;
}

bool cr_pulse(uint8_t n, uint16_t ms)
{
    if (!s_up || n < 1u || n > s_cfg.channels || ms == 0u) return false;
    if (!present()) return false;                       /* step 11g: a pulse only to a board that is there */
    uint8_t i = (uint8_t)(n - 1u);
    uint32_t end = hal_millis() + ms;
    s_retry &= ~bit(i);                                 /* step 13h: a pulse is never retried (the STARTER never late) */
    s_odrop &= ~bit(i);
    s_until[i] = end ? end : 1u;                        /* ON at 50 Hz until then (AURIX-timed), then OFF */
    s_on |= bit(i);
    s_known |= bit(i);
    s_burst[i] = 0u;
    s_st.pulses++;
    if (ms > s_st.max_pulse_ms) s_st.max_pulse_ms = ms;
    return true;
}

bool cr_readback(void)
{
    for (uint8_t i = 0; s_up && i < s_cfg.channels; i++) {
        uint32_t age;
        if (dc_hardpoint_status(i, NULL, &age) && age <= s_cfg.lost_ms) return true;
    }
    return false;
}

bool cr_read(uint8_t n, bool *on)
{
    if (!s_up || n < 1u || n > s_cfg.channels) return false;
    uint8_t i = (uint8_t)(n - 1u);
    uint16_t st;
    uint32_t age;
    /* step 11g: known only while the board is there (its NodeStatus, OPERATIONAL) - the same moment an ON can be
     * sent (cr_set / cr_pulse refuse it otherwise), so relays.c's power_up never meets a refused ON */
    if (!present()) return false;
    if (dc_hardpoint_status(i, &st, &age) && age <= s_cfg.lost_ms) {   /* the board's readback (R-4) */
        *on = st != 0u;
        return true;
    }
    if (cr_readback()) return false;                    /* the board reads back, but not this one: unknown */
    /* no readback at all: the commanded state, while the board is there */
    if (!(s_known & bit(i))) return false;
    *on = (s_on & bit(i)) != 0u;
    return true;
}

void cr_keep(uint32_t *known, uint32_t *on)
{
    uint32_t pulsing = 0u;
    for (uint8_t i = 0; i < s_cfg.channels; i++) if (s_until[i]) pulsing |= bit(i);
    *known = s_known;
    *on = s_on & s_known & ~pulsing;                    /* a running pulse: the board's own timeout ends it (R-3) */
}

const cr_stats_t *cr_stats(void)
{
    return &s_st;
}
