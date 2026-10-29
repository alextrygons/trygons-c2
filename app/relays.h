/*
 * relays.h - The relay board as the boat sees it (CLAUDE.md section M + owner
 * clarifications; step 11f: the RS485 Modbus board OR the DroneCAN relay node -
 * RELAY_BUS in hal_pins.h, the same hal_relay_* calls, every rule here unchanged). Pure C99; this module is the ONLY
 * caller of hal_relay_* (tools/check.sh greps for it, like servo.c for the
 * throttle). No relay is in any control path: nothing here can change the
 * mode, the throttle, the gear, the steering or the thrust (storm I36).
 *
 * The relay TABLE (which channel is what) comes from the boat file through
 * the HELM station (PROTO_T_RELAY_CFG) and lives in the data flash (NV slot
 * 7): a table that differs from the stored one is taken only in STANDBY and
 * only after the joystick CONFIRM (onboard in CREWED, the shore joystick in
 * UNCREWED) - until then the stored table stays in use (R7: a wrong boat
 * file must not move the STARTER out of the engine rules). The same table
 * again = accepted at once, in any mode. Without a table no relay can be
 * switched.
 *
 * Rules carried out here:
 *   - onoff / latching: set / clear (driven the same way on the AURIX; the
 *     screens draw a latching relay as a toggle that keeps its state)
 *   - momentary: a pulse <= max_s (<= 5 s) that ends by itself: Modbus =
 *     BOARD-timed; DroneCAN = AURIX-timed + the board's own command timeout
 *     (<= 300 ms, R-3) when the AURIX stops; the AURIX only refreshes / releases it
 *   - timed: on, and OFF by the AURIX after max_s (also while the board is
 *     silent: the OFF stays pending and is re-sent until the readback shows it off)
 *   - interlocked pair: never both - the second request is refused while
 *     the other is on (commanded or read back) or its state is unknown (a
 *     non-momentary partner on a DroneCAN board without readback)
 *   - a silent board (no valid reply for HAL_RELAY_LOST_MS): no new command
 *     (the OFF of a timed relay still goes), states shown unknown, the
 *     advisory RELAY BOARD SILENT (raised by smallboat.c)
 *   - engine group: every relay has an explicit ROLE (STARTER / IGNITION /
 *     SHUTOFF, at most one each; a table without it, or a STARTER that is not
 *     a momentary relay of <= 5 s, is INVALID - the rules never depend on the
 *     type alone). NO engine-group relay is switched ON by the plain relay
 *     command: the STARTER only by the hold-to-crank rules of section N
 *     (engine.c: relays_starter_pulse / _off), IGNITION / SHUTOFF only through
 *     engine.c (engine_relay: idle + neutral confirmed, e-stop released, not
 *     killed / tripped -> relays_engine_set). OFF is always allowed.
 *   - momentary: the AURIX's own copy of the board timer; RELAY_PULSE_MARGIN_MS
 *     after the pulse should have ended it sends an OFF itself and keeps it
 *     pending until the readback shows the relay off (a board whose timer
 *     failed never keeps a relay on)
 *   - channels: n must be within the fitted board's channels (hal_relay_channels);
 *     no board fitted = nothing is switched and no RELAY BOARD SILENT
 *   - power_up: ONLY switching ON, only once, only with the FIRST active
 *     table after the AURIX's own POWER-ON reset (never after a watchdog /
 *     software reset or a reconnect: then the board is read back and its
 *     states kept), never the engine group, only once the board answers
 *   - FIRST TEST: no relay changes at all (commands refused, no power_up);
 *     the table itself may still be stored (bench set-up)
 *   - monitor trip / e-stop: relays are left as they are (the board is not a
 *     safety path: the engine kill pin and the e-stop contactor are)
 */
#ifndef RELAYS_H
#define RELAYS_H

#include <stdbool.h>
#include <stdint.h>

#define RELAY_TABLE_MAX   16u       /* relays in a table (8-16 channel boards) */
#define RELAY_N_MAX       32u       /* channel numbers 1..32 */
#define RELAY_TABLE_VERSION 2u      /* 2: the engine roles (a version-1 record = defaults: the station pushes again) */
#define RELAY_PULSE_MAX_MS 5000u    /* a momentary relay: never longer */
#define RELAY_PULSE_MARGIN_MS 500u  /* after a pulse should have ended: the AURIX sends the OFF itself */
#define RELAY_FRAME_BYTES 8u        /* per relay in PROTO_T_RELAY_CFG */
#define RELAY_OFF_RETRY_MS 500u     /* a pending OFF is sent again this often until the readback shows it off */
#define RELAY_SHUTOFF_RETRY_MS 10000u /* step 13b: a SHUTOFF ON (stop the engine, always allowed) the board could not take
                                       yet (silent, or not seen yet after a warm reset) is retried this long, then
                                       dropped (a late engine stop is never sent minutes later); the STARTER is
                                       never queued */

/* relay_def_t.type */
#define RELAY_ONOFF       0u
#define RELAY_MOMENTARY   1u
#define RELAY_LATCHING    2u
#define RELAY_TIMED       3u
/* relay_def_t.group */
#define RELAY_GRP_ENGINE  0u
#define RELAY_GRP_TRIM    1u
#define RELAY_GRP_EXTRA   2u
/* relay_def_t.flags */
#define RELAY_F_SHOW_HUD  0x01u
#define RELAY_F_POWER_ON  0x02u     /* power_up "on" */
#define RELAY_F_KEEP      0x04u     /* power_up "keep" (= nothing is done: the board keeps its state) */
/* relay_def_t.role: engine group = one of these (each at most once), every other group = NONE */
#define RELAY_ROLE_NONE     0u
#define RELAY_ROLE_STARTER  1u      /* momentary, <= 5 s: hold-to-crank only (section N) */
#define RELAY_ROLE_IGNITION 2u      /* onoff / latching: ON only through engine.c */
#define RELAY_ROLE_SHUTOFF  3u      /* onoff / latching: ON only through engine.c */

typedef struct {
    uint8_t  n;                 /* channel 1..32 */
    uint8_t  type;              /* RELAY_ */
    uint8_t  group;             /* RELAY_GRP_ */
    uint8_t  pair;              /* the other relay of an interlocked pair (its n), 0 = none */
    uint16_t max_ds;            /* momentary: pulse length; timed: on at most this (0.1 s units) */
    uint8_t  flags;             /* RELAY_F_ */
    uint8_t  role;              /* RELAY_ROLE_ */
} relay_def_t;

/* NV slot 7 record (version + CRC-32 like the other records) */
typedef struct {
    uint8_t     version;
    uint8_t     count;
    uint8_t     pad[2];
    relay_def_t r[RELAY_TABLE_MAX];
    uint32_t    crc;
} relay_table_t;

/* the relay command (PROTO_T_RELAY: u8 n, u8 action, u16 ms) */
#define RELAY_CMD_OFF     0u
#define RELAY_CMD_ON      1u        /* momentary: a pulse of max_s */
#define RELAY_CMD_PULSE   2u        /* momentary: a pulse of min(ms, max_s); others: like ON */

typedef enum {
    RELAY_R_OK = 0,
    RELAY_R_NO_TABLE,       /* no relay table (boat file not pushed / confirmed) */
    RELAY_R_UNKNOWN,        /* n is not in the table / bad action */
    RELAY_R_SILENT,         /* the board is silent: no new command */
    RELAY_R_PAIR,           /* the interlocked partner is on */
    RELAY_R_ENGINE,         /* engine group ON: only through engine.c (section N rules) */
    RELAY_R_FIRST_TEST,     /* FIRST TEST: no relay changes */
    RELAY_R_HAL,            /* the driver refused (queue full) */
    RELAY_R_NOT_FITTED      /* no relay board fitted (hal_relay_channels() == 0) */
} relay_result_t;

typedef enum {
    RELAY_OFFER_SAME = 0,   /* the same table as the active one: nothing to do */
    RELAY_OFFER_PENDING,    /* differs: stored as the offer, waiting for the joystick CONFIRM */
    RELAY_OFFER_STANDBY,    /* differs, but not in STANDBY (not stopping): refused */
    RELAY_OFFER_BAD,        /* the table fails its checks */
    RELAY_OFFER_STOP        /* step 13g: differs, but a relay stop / timer is pending (relays_stop_pending): refused
                               up front (NACK "relay stop pending"), the station waits (TELEM2 M flag 128) */
} relay_offer_t;

typedef struct {
    relay_table_t table;        /* the active table (= NV slot 7) */
    bool     have_table;
    bool     from_nv;           /* the active table was loaded at boot */
    relay_table_t offer;        /* waiting for CONFIRM */
    bool     offer_pending;
    bool     first_test;
    bool     power_on_reset;    /* the AURIX's own power-on reset (not a watchdog / software reset) */
    bool     power_up_done;     /* the one-shot power_up is over (done, or never allowed) */
    bool     board_ok;          /* hal_relay_ok() this step */
    bool     readback;          /* the states are the board's own readback (false: a DroneCAN board without
                                   hardpoint.Status - the COMMANDED states, TELEM2 M flag 64 "no readback") */
    uint8_t  channels;          /* hal_relay_channels(): the fitted board's coils, 0 = no board */
    uint32_t pulse_offs;        /* OFFs the AURIX sent itself after a pulse should have ended */
    uint32_t states, known;     /* readback: bit n-1 (known = a valid readback) */
    uint32_t cmd_on;            /* bit n-1: commanded on by the AURIX (pulse running / on / timed on) */
    uint32_t off_pending;       /* bit n-1: an OFF not yet confirmed by the readback (re-sent while the board
                                   answers and still shows the relay on: a silent board loses writes) */
    uint32_t off_retry_ms;      /* next re-send of the pending OFFs */
    uint32_t shutoff_pending;   /* bit n-1: a SHUTOFF ON waiting for the board (step 13b) ... */
    uint32_t shutoff_until_ms;  /* ... until then (RELAY_SHUTOFF_RETRY_MS) */
    uint32_t shutoff_retried, shutoff_dropped;
    uint32_t shutoff_cut;       /* step 13d: a SHUTOFF ON burst cut by the bus driver, queued again */
    uint32_t on_dropped;        /* step 13h (W42): a cut ON the driver gave up (10 s / board reboot): command cleared */
    uint32_t until_ms[RELAY_TABLE_MAX];  /* pulse end (momentary) / OFF deadline (timed); 0 = none */
    uint16_t hash;              /* CRC-16 of the active table's frame encoding (TELEM2; the station compares) */
    bool     table_new;         /* the active table changed: save it (NV slot 7) */
    uint32_t commands, refused, pulses, timed_offs, power_ups;
    uint8_t  last_n, last_action, last_result;
    uint8_t  last_what;         /* 1 a command (also a timed OFF), 2 a table event (loaded / offered / accepted /
                                   dropped / power_up) - for the log producer */
    uint32_t log_seq;           /* +1 per command / table event (log producer) */
} relays_t;

/* table checks + sealing (version, CRC) */
bool     relay_table_valid(const relay_table_t *t);
void     relay_table_seal(relay_table_t *t);
/* the frame encoding (PROTO_T_RELAY_CFG payload): u8 count, then per relay
 * {n, type, group, pair, max_ds lo, max_ds hi, flags}. Returns the length. */
uint32_t relay_table_encode(const relay_table_t *t, uint8_t *out, uint32_t max);
bool     relay_table_decode(const uint8_t *in, uint32_t len, relay_table_t *t);
uint16_t relay_table_hash(const relay_table_t *t);
const relay_def_t *relay_table_find(const relay_table_t *t, uint8_t n);

/* every n of the table within the fitted board's channels (always true without a board) */
bool     relays_table_fits(const relay_table_t *t);
/* stored = the NV record (NULL = none / unusable) */
void relays_init(relays_t *r, const relay_table_t *stored, bool power_on_reset, bool first_test);
/* a table from the HELM station */
relay_offer_t relays_offer(relays_t *r, const relay_table_t *t, bool standby_idle);
/* t is the active table already (an offer of it changes nothing) */
bool     relays_offer_same(const relays_t *r, const relay_table_t *t);
/* the joystick CONFIRM (only in STANDBY, not stopping): the offer becomes the active table
 * (refused while relays_stop_pending: a table change would drop the stop / timer) */
bool     relays_accept(relays_t *r, bool standby_idle);
/* step 13g (review facf265 MED-1): a stop or a timer the AURIX still owes the board - a queued SHUTOFF ON, an OFF
 * not yet confirmed by the readback, a timed relay's OFF deadline or a pulse's end. A table change (table_activate)
 * would forget it, so no table is offered / accepted while it is true (TELEM2 M flag 128) */
bool     relays_stop_pending(const relays_t *r);
void     relays_cancel_offer(relays_t *r);
/* the relay command of the HELM station */
relay_result_t relays_cmd(relays_t *r, uint8_t n, uint8_t action, uint16_t ms, uint32_t now_ms);
/* IGNITION / SHUTOFF (engine group, not the STARTER): ONLY engine.c calls this
 * (engine_relay checks idle + neutral + e-stop first; tools/check.sh) */
relay_result_t relays_engine_set(relays_t *r, uint8_t n, bool on);
/* the log record of a command refused before relays_cmd / relays_engine_set (engine conditions) */
relay_result_t relays_note(relays_t *r, uint8_t n, uint8_t action, relay_result_t res);
/* every control step: the driver's step, readback, time limits, power_up */
void     relays_step(relays_t *r, uint32_t now_ms);
/* the advisory condition: a table with relays and the board silent */
bool     relays_silent(const relays_t *r);
/* The STARTER (= the relay with RELAY_ROLE_STARTER; 0 = none).
 * Driven ONLY by the remote-restart rules (engine.c, section N; tools/check.sh):
 * a board-timed pulse of ms (<= the relay's max_s, <= RELAY_PULSE_MAX_MS), and
 * off at once. Refused in FIRST TEST, without a table / STARTER, on a silent board. */
uint8_t  relays_starter(const relays_t *r);
relay_result_t relays_starter_pulse(relays_t *r, uint16_t ms, uint32_t now_ms);
void     relays_starter_off(relays_t *r);
uint8_t  relays_flags(const relays_t *r);           /* TELEM2 group M flags (128 = relays_stop_pending) */

#endif /* RELAYS_H */
