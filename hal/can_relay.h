/*
 * can_relay.h - The relay board on DroneCAN: the second back-end of the hal_relay_*
 * API (CLAUDE.md T "Relay board over DroneCAN (11f)", docs/dronecan_design.md
 * section 9; step 11f). Pure C99 over hal/dronecan.c (no iLLD): the same code on the
 * TC375 (hal_tc375.c with RELAY_BUS == RELAY_BUS_DRONECAN) and in the PC simulation
 * (sim/hal_sim.c, the simulated relay node 30). The RS485 Modbus back-end
 * (hal/tc_modbus.c) stays as the other build option (owner decision OD-9); app/relays.c
 * sees the SAME hal_relay_* calls and keeps EVERY relay rule (it does not know the bus).
 *
 * Core 0 only. The board: node DC_NODE_RELAY (30, TO CONFIRM C3 / C17), relay n =
 * hardpoint n - 1 (ArduPilot RELAY1..6_PIN 1000..1005 = hardpoints 0-5), command 1 = on,
 * 0 = off (uavcan.equipment.hardpoint.Command).
 *
 * What is sent, and when (ArduPilot CAN_D1_UC_RLY_RT 50 Hz):
 *   - NOTHING after a boot (power-on or warm reset): no frame for a relay until relays.c
 *     commands it - the board HOLDS its states (requirement R-2), so a reset never
 *     switches anything (the rules: never OFF by itself, power_up only ON and only after
 *     the AURIX's own power-on reset - relays.c decides that, not this file).
 *   - set (on / off): the new state CR_BURST_FRAMES times at 50 Hz, then no more frames
 *     (the board holds it, R-2; a hand switch on the board is never overwritten). The
 *     burst counts only frames sent while the board is there: an OFF asked while the board
 *     is silent goes when it is back. An ON is never queued for a board that is not there
 *     (step 11g): cr_set ON / cr_pulse refuse it, and an ON burst or a running pulse is
 *     dropped when the board goes (bus fault / silent / not OPERATIONAL) - the pulse ends
 *     with an OFF burst, a dropped ON leaves the state unknown and sends nothing (step 13h,
 *     W42: finished when the board is back only while still wanted, within 10 s, no reboot).
 *   - pulse (momentary relays, the STARTER's hold-to-crank): ON at 50 Hz until the pulse
 *     end (AURIX-timed: hardpoint.Command has no board-timed pulse), then the OFF burst.
 *     When the AURIX stops sending (frozen core 0, reset, cable) the BOARD switches the
 *     relay off by its own command timeout (<= 300 ms, requirement R-3); the STARTER is
 *     always also behind a hardware time-limit relay (OD-9).
 * Readback: the board's uavcan.equipment.hardpoint.Status (status != 0 = on) if it sends
 *   it (R-4, TO CONFIRM C17) - then exactly like the Modbus coil readback (valid for
 *   HAL_RELAY_LOST_MS). A board that sends no Status at all: cr_readback() false and the
 *   COMMANDED states are the readback (the screens say "no readback"): known only once
 *   commanded since the boot, kept across a warm reset (no-clear RAM, cr_keep: stored by
 *   the HAL after every cr_set / cr_pulse and every step), or all OFF when the board
 *   itself booted after the AURIX's start - a power-on or a warm reset (R-2: it powers
 *   up OFF; step 11g: the uptime test runs after a warm reset too).
 * ok: the bus works and the relay node's NodeStatus is younger than HAL_RELAY_LOST_MS,
 *   mode OPERATIONAL, health below ERROR (WARNING = ok: a notice, OD-6). Like the Modbus
 *   master, the first HAL_RELAY_LOST_MS after the start give the board time to be seen.
 *   A board that rebooted (its uptime went back) powered up OFF (R-2): every state OFF,
 *   nothing re-sent.
 */
#ifndef CAN_RELAY_H
#define CAN_RELAY_H

#include <stdbool.h>
#include <stdint.h>

#define CR_CH_MAX        32u     /* relays 1..32 = hardpoints 0..31 */
#define CR_PERIOD_MS     20u     /* 50 Hz (ArduPilot CAN_D1_UC_RLY_RT) */
#define CR_BURST_FRAMES  10u     /* a new on / off state: 10 frames (200 ms), then the board holds it (R-2) */
#define CR_BOOT_MARGIN_MS 2000u  /* the board booted after us when its uptime <= ours + this */
#define CR_ON_RETRY_MS   10000u  /* step 13h (W42, owner 2026-10-29 = RELAY_ON_RETRY_MS): a cut ON burst is finished
                                    only within this time of the ON command */
/* step 13h2 (review 3bef4c8 MED-1): a board whose uptime did not grow with our clock between two NodeStatus rebooted
 * too (also when its new uptime is not below the old one: a young board silent after its reboot for longer than it
 * had been up). Rebooted when uptime_new*1000 + CR_UPTIME_RES_MS + CR_UPTIME_JITTER_MS + elapsed/CR_UPTIME_DRIFT_DIV
 * < uptime_old*1000 + elapsed (elapsed = our time between the two receptions): the 1 s resolution of uptime_s, the
 * reception jitter and a 3 % clock difference never read as a reboot */
#define CR_UPTIME_RES_MS    1000u
#define CR_UPTIME_JITTER_MS  300u
#define CR_UPTIME_DRIFT_DIV   32u

typedef struct {
    uint8_t  channels;           /* the board's hardpoints (hal_pins.h CAN_RELAY_CHANNELS, TO CONFIRM; sim: fitted) */
    uint32_t lost_ms;            /* HAL_RELAY_LOST_MS */
    bool     power_on;           /* the AURIX's own POWER-ON reset (a board booting after it is all OFF) */
    bool     keep_valid;         /* the commanded states of before a warm reset (no-clear RAM) */
    uint32_t keep_known, keep_on;
} cr_config_t;

void cr_init(const cr_config_t *c);
/* every control step (hal_relay_service): the board's node state, pulse ends */
void cr_service(void);
/* core 0's 10 ms task (hal_can_tick): the frames at 50 Hz; never blocks */
void cr_tick(void);
/* the hal_relay_* back-end (n = 1..channels; false = bad n / not started) */
bool cr_set(uint8_t n, bool on);
bool cr_pulse(uint8_t n, uint16_t ms);
bool cr_read(uint8_t n, bool *on);
bool cr_ok(void);
bool cr_readback(void);          /* the board sends hardpoint.Status (else: the commanded states, "no readback") */
/* step 13d (review 49353b2 MED-1): an ON burst the driver had taken and then cut (the board went away): cr_take_cut
 * is true once for that relay (relays.c queues a SHUTOFF ON again). The channels of cr_hold_mask (bit n-1; relays.c:
 * the SHUTOFF relay) get NO OFF burst for such a cut - the state stays unknown, nothing is sent (an automatic OFF
 * would undo an engine stop); every other channel: see cr_on_want (step 13h, W42). */
bool cr_take_cut(uint8_t n);
void cr_hold_mask(uint32_t mask);
uint32_t cr_hold(void);          /* step 13e: the hold mask now (tests) */
uint32_t cr_cuts(void);          /* step 13e: the cut bits not taken yet (tests) */
/* step 13h (W42, owner 2026-10-29 - replaces the 13b OFF burst): a cut ON burst of any other channel sends NOTHING
 * (never an OFF by itself); it is finished when the board is back ONLY while (1) relays.c still wants it ON
 * (cr_on_want: bit n-1 = commanded ON, not momentary, not the engine group; a new cr_set / cr_pulse replaces it),
 * (2) within CR_ON_RETRY_MS of the ON command and (3) the board did not reboot meanwhile (its uptime went back:
 * every state OFF, nothing sent; step 13h2: or its uptime did not grow with our clock: every ON state UNKNOWN,
 * nothing sent; either way EVERY channel that was ON is reported through cr_take_on_drop). Given up (window over / reboot / not wanted at the cut) = the state UNKNOWN (OFF
 * after a reboot), nothing sent, cr_take_on_drop true once so relays.c clears its command (and logs it). */
void cr_on_want(uint32_t mask);
bool cr_take_on_drop(uint8_t n);
uint32_t cr_retries(void);       /* the cut ON bursts waiting for the board (tests) */
/* the commanded states to keep across a warm reset (a running pulse = off: the board's timeout ends it) */
void cr_keep(uint32_t *known, uint32_t *on);

typedef struct {
    uint32_t frames;             /* hardpoint.Command frames queued */
    uint32_t refused;            /* not queued (bus fault / no room) */
    uint32_t dropped;            /* ON bursts dropped: the board was not there (step 11g; only OFF waits for it) */
    uint32_t sets, pulses;       /* commands from relays.c */
    uint32_t max_pulse_ms;       /* the longest pulse ever asked */
    uint32_t board_reboots;      /* the board's uptime went back: every state OFF */
    uint32_t board_reboots_t;    /* step 13h2: its uptime did not grow with our clock: every ON state unknown */
    uint32_t on_retried;         /* step 13h: cut ON bursts finished when the board was back */
    uint32_t on_given_up;        /* step 13h: cut ON bursts given up (window / reboot / not wanted) */
    uint32_t ticks;
} cr_stats_t;
const cr_stats_t *cr_stats(void);

#endif /* CAN_RELAY_H */
