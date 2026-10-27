/*
 * proto.h - Ground-station frame format (same on USB, UART and sim TCP).
 *
 *   0xAA 0x55 | len | type | seq | payload[len] | crc_lo crc_hi
 *
 *   len   payload length, 0..PROTO_MAX_PAYLOAD
 *   type  message type (PROTO_T_...)
 *   seq   sender's frame counter (ACK/NACK echo the command's seq)
 *   crc   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over len, type,
 *         seq and the payload
 *
 * All numbers little-endian. Floats are IEEE-754 32 bit, lat/lon are
 * int32 in 1e-7 degrees.
 *
 * The receiver keeps unread bytes in a ring: a frame with a bad CRC only
 * drops its first byte, so a good frame hidden inside garbage or right
 * after a corrupted one is still found.
 *
 * The Python side (groundstation/proto.py) must match this file.
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTO_SYNC1        0xAAu
#define PROTO_SYNC2        0x55u
#define PROTO_MAX_PAYLOAD  200u
#define PROTO_OVERHEAD     7u          /* sync 2 + len + type + seq + crc 2 */
#define PROTO_MAX_FRAME    (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)
#define PROTO_RX_BUF       512u        /* power of 2, >= 2 frames */

/* PC -> boat */
#define PROTO_T_HEARTBEAT     0x01u   /* -                                    */
#define PROTO_T_SET_MODE      0x10u   /* u8 mode: only STANDBY and TRAINING   */
#define PROTO_T_HEADING_ADJ   0x12u   /* i8 delta: -10, -1, +1, +10 (HDG_LOCK) */
#define PROTO_T_SEA_STATE     0x15u   /* u8 1..5                              */
#define PROTO_T_TRAINING      0x16u   /* u8 0 start / 1 accept / 2 reject, f32 full angle (0 = unknown) */
#define PROTO_T_PARAM_GET     0x17u   /* u8 id                                */
#define PROTO_T_PARAM_SET     0x18u   /* u8 id, f32 value                     */
#define PROTO_T_JOY_CAL       0x19u   /* u8 0 start / 1 finish                */
#define PROTO_T_ALARM_ACK     0x1Au   /* -  acknowledge alarms (any mode)      */
#define PROTO_T_GW_STATUS     0x31u   /* gateway only, 1 Hz heartbeat, 12 bytes (link.c gw_status) */
#define PROTO_T_GW_CMD        0x32u   /* gateway only: u8 station, u8 flags (b0 local), u16 seq, u8 type, payload */
#define PROTO_T_HELM          0x33u   /* u8 0 take / 1 release (any station; local always wins) /
                                         u8 2, u8 station, u8 local: the holder hands it over in one step */
#define PROTO_T_GW_STATION    0x34u   /* gateway only: u8 station, u8 event (0 joined, 1 left / lost), u8 local */
#define PROTO_T_GW_RJOY       0x36u   /* gateway only, 30 Hz, no ACK: remote joystick (link.c gw_rjoy) */
#define PROTO_T_REMOTE        0x37u   /* u8 0 ask for remote driving (then joystick CONFIRM on board) / 1 release */
#define PROTO_T_MANEUVER      0x38u   /* u8 0 ask for MANEUVER (captain; then CONFIRM) / 1 release */
#define PROTO_T_GW_GPS2       0x39u   /* gateway only: i32 lat*1e7, i32 lon*1e7, u16 age ms (second GPS, AIS device) */
#define PROTO_T_GW_SHORE      0x3Bu   /* gateway only, 1 Hz: u8 flags (b0 near, b1 valid), u16 distance to the shore m */
#define PROTO_T_GW_NAVREQ     0x3Au   /* gateway only, 2 Hz: navigation request (link.c gw_navreq, smallboat.h SB_NAV_) */
#define PROTO_T_JOYMAP_GET    0x1Bu   /* -  -> JOYMAP_VALUE                     */
#define PROTO_T_JOYMAP_SET    0x1Cu   /* joymap body (see link.c), u8 save; STANDBY only */
#define PROTO_T_JOY_RAW_GET   0x1Du   /* -  -> JOY_RAW (USB bridge, for the button mapping) */
#define PROTO_T_RELAY         0x1Eu   /* u8 n (1..32), u8 action (0 off / 1 on / 2 pulse), u16 pulse ms: switch a
                                       * relay of the table (HELM station; relays.h rules: interlock, max time,
                                       * never the STARTER, nothing in FIRST TEST / without a table / silent board) */
#define PROTO_T_RELAY_CFG     0x2Fu   /* u8 count (0..16), count x {u8 n, u8 type, u8 group, u8 pair, u16 max_s*10,
                                       * u8 flags, u8 role (0 none, 1 starter, 2 ignition, 3 shutoff)}: the boat file's relay table (HELM station). The same table as the
                                       * active one = OK; a different one: STANDBY only, then joystick CONFIRM
                                       * (CONF_RELAY_TABLE) - the stored table (NV slot 7) stays in use until then */
#define PROTO_T_THRUSTER_CFG  0x1Fu   /* 12 bytes: bow {u8 max_pct, u16 run_s, u16 rest_s}, stern {the same},
                                       * u8 jog_m*10, u8 spot_alarm_m: the boat file's DP thruster settings
                                       * (HELM station, STANDBY only, RAM only; thruster.h) */
#define PROTO_T_PROG_BEGIN    0x20u   /* u8 n (1..50), u8 final action        */
#define PROTO_T_PROG_WP       0x21u   /* u8 index, i32 lat, i32 lon, f32 kn, f32 radius m, u8 action */
#define PROTO_T_PROG_COMMIT   0x22u   /* -  validate + store; NACK -> PROG_ERROR frame too */
#define PROTO_T_PROG_GET      0x23u   /* u8 index -> PROG_WP_VALUE            */
#define PROTO_T_AUTO_START    0x24u   /* -  then joystick CONFIRM within 10 s */
#define PROTO_T_AUTO_PAUSE    0x25u   /* -                                    */
#define PROTO_T_AUTO_RESUME   0x26u   /* -  then joystick CONFIRM within 10 s */
#define PROTO_T_AUTO_SKIP     0x27u   /* -                                    */
#define PROTO_T_POWER_CODE    0x28u   /* u8 dir (0 fwd, 1 rev): boat makes a code (telemetry) */
#define PROTO_T_POWER_RAISE   0x29u   /* u8 dir, u16 typed code; then joystick CONFIRM within 10 s */
#define PROTO_T_POWER_LOWER   0x2Au   /* -  back to 30 % / 5 % at once        */
#define PROTO_T_BENCH         0x2Bu   /* u8 action (SB_BENCH_), u16 arg (ms / trip code); USB laptop ONLY */
#define PROTO_T_GENERATOR     0x2Cu   /* u8 0 off / 1 on: generator auto (HELM station; a copilot's ON needs the
                                       * captain's ACCEPT on the gateway; the STANDBY button switches it off) */
#define PROTO_T_RESTART       0x2Du   /* u8 action: 0 RESTART (clear the ENGINE KILL / software reset after a
                                       * monitor trip), 1 CRANK (START held: 5 Hz, refreshes a 300 ms STARTER
                                       * pulse, <= 5 s per hold, 10 s cool-down), 2 RELEASE. HELM station only;
                                       * no code / CONFIRM; every time idle + neutral confirmed, e-stop released,
                                       * not FIRST TEST (engine.h; a copilot's RESTART needs the captain's ACCEPT,
                                       * CRANK is the captain's only - gateway) */
#define PROTO_T_SIM_FIRST     0x70u   /* 0x70..0x7F: simulator control only   */
#define PROTO_T_SIM_JOYSTICK  0x70u   /* u8 fitted, f32 x, f32 y, u8 take, u8 auto, u8 confirm */
#define PROTO_T_SIM_GPS       0x71u   /* u8 1 = GPS fails, 0 = GPS back       */
#define PROTO_T_SIM_BUTTON    0x72u   /* u8 panel button (hal_button_t)       */
#define PROTO_T_SIM_BRIDGE    0x73u   /* u8 flags, 2 x (i16 axis[8], u32 buttons, u8 hats), u8 fault bits */
#define PROTO_T_SIM_LAST      0x7Fu

/* boat -> PC */
#define PROTO_T_TELEMETRY     0x80u   /* see link.c telemetry()                */
#define PROTO_T_ACK           0x81u   /* u8 cmd type, u8 cmd seq, u8 result (PROTO_R_...) */
#define PROTO_T_PARAM_VALUE   0x82u   /* u8 id, f32 value, f32 min, f32 max, char name[16] */
#define PROTO_T_PROG_WP_VALUE 0x83u   /* u8 index, u8 n, u8 final, i32 lat, i32 lon, f32 kn, f32 radius, u8 action */
#define PROTO_T_PROG_ERROR    0x84u   /* u8 prog_error_t, u8 waypoint index (0xFF = none) */
#define PROTO_T_JOYMAP_VALUE  0x85u   /* joymap body (see link.c)             */
#define PROTO_T_JOY_RAW       0x86u   /* u8 flags, u8 seq, 2 x (i16 axis[8], u32 buttons, u8 hats) */
#define PROTO_T_LOG_CHUNK     0x89u   /* gateway only: u32 record no, u16 offset, u16 length, bytes */
#define PROTO_T_TIME          0x8Au   /* gateway only, 1 Hz: u8 valid, u32 date, u32 UTC ms of day, u32 t_ms */
#define PROTO_T_TELEM2        0x8Bu   /* with every telemetry: gateway / helm / remote / slamming (link.c) */
#define PROTO_T_GW_ACK        0x8Cu   /* gateway only: u8 station, u16 command seq, u8 type, u8 result */
#define PROTO_T_BENCH_DIAG    0x8Du   /* USB only, with every telemetry in BENCH TEST: every sensor (link.c bench_diag) */

/* ACK result: 0 = accepted, else why refused (NACK) */
#define PROTO_R_OK            0u
#define PROTO_R_BAD_LENGTH    1u      /* payload length wrong for this type   */
#define PROTO_R_UNKNOWN       2u      /* unknown message type                 */
#define PROTO_R_BAD_VALUE     3u      /* value out of range                   */
#define PROTO_R_JOYSTICK      4u      /* joystick is in control               */
#define PROTO_R_NOT_ALLOWED   5u      /* mode change not in the table         */
#define PROTO_R_PRECOND       6u      /* conditions not met (GPS, waypoint, sensors ...) */
#define PROTO_R_WRONG_MODE    7u      /* command not possible in this mode    */
#define PROTO_R_NOT_CENTERED  8u      /* centre the joystick: stick held (hand-back, HDG LOCK, AUTO, REMOTE, TRAINING) */
#define PROTO_R_STANDBY_ONLY  9u      /* only in STANDBY                      */
#define PROTO_R_BAD_PARAM     10u     /* no such parameter                    */
#define PROTO_R_REFUSED       11u     /* refused by the function (e.g. implausible calibration) */
#define PROTO_R_BAD_PROGRAM   12u     /* program refused (details in PROG_ERROR) */
#define PROTO_R_NO_PROGRAM    13u     /* no valid program stored              */
#define PROTO_R_JOY_NOT_READY 14u     /* joystick missing / faulted / not armed: CONFIRM impossible */
#define PROTO_R_WRONG_CODE    15u     /* power raise: code wrong / expired (ask for a new one) */
#define PROTO_R_AT_MAX        16u     /* power raise: already at the top step */
#define PROTO_R_ALARM_ACTIVE  17u     /* power raise: acknowledge the alarm first */
#define PROTO_R_MAP_DUPLICATE 18u     /* button map: two functions share a button */
#define PROTO_R_MAP_MISSING   19u     /* button map: TAKE, AUTO and CONFIRM must be mapped */
#define PROTO_R_MAP_BAD       20u     /* button map: bad code / neutral zone */
#define PROTO_R_NOT_HELM      21u     /* this station does not hold HELM */
#define PROTO_R_HELM_TAKEN    22u     /* another station holds HELM (only a local station can take it) */

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

typedef struct {
    uint8_t  buf[PROTO_RX_BUF];
    uint32_t head, tail;            /* free-running; used = head - tail */
    uint32_t frames;                /* good frames */
    uint32_t crc_errors;            /* bad CRC (or impossible length) */
    uint32_t junk;                  /* bytes skipped while looking for sync */
    uint32_t overflows;             /* bytes lost because the ring was full */
} proto_rx_t;

uint16_t proto_crc16(const uint8_t *p, size_t n);

/* Build a frame into out (>= PROTO_OVERHEAD + len bytes). Returns the
 * frame length, 0 if len is too big. */
size_t proto_encode(uint8_t *out, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);

void proto_rx_init(proto_rx_t *r);
void proto_rx_push(proto_rx_t *r, const uint8_t *data, size_t n);
/* Next good frame, if one is complete. Call until it returns false. */
bool proto_rx_next(proto_rx_t *r, proto_frame_t *f);
/* Bytes waiting (an incomplete frame). */
uint32_t proto_rx_pending(const proto_rx_t *r);
/* Call when bytes have waited too long without completing a frame (e.g. a
 * corrupted length byte promised a longer frame): drops the first byte so
 * the next proto_rx_next() rescans. */
void proto_rx_stale(proto_rx_t *r);

/* Little-endian payload writer / reader with bounds checks. */
typedef struct {
    uint8_t *p;
    size_t   n, max;
    bool     overflow;
} proto_wr_t;

typedef struct {
    const uint8_t *p;
    size_t         n, len;
    bool           underflow;
} proto_rd_t;

void proto_wr_init(proto_wr_t *w, uint8_t *buf, size_t max);
void proto_wr_u8(proto_wr_t *w, uint8_t v);
void proto_wr_u16(proto_wr_t *w, uint16_t v);
void proto_wr_u32(proto_wr_t *w, uint32_t v);
void proto_wr_i32(proto_wr_t *w, int32_t v);
void proto_wr_f32(proto_wr_t *w, float v);
void proto_wr_latlon(proto_wr_t *w, double deg);     /* int32, 1e-7 deg */

void     proto_rd_init(proto_rd_t *r, const uint8_t *buf, size_t len);
uint8_t  proto_rd_u8(proto_rd_t *r);
int8_t   proto_rd_i8(proto_rd_t *r);
uint16_t proto_rd_u16(proto_rd_t *r);
uint32_t proto_rd_u32(proto_rd_t *r);
int32_t  proto_rd_i32(proto_rd_t *r);
float    proto_rd_f32(proto_rd_t *r);
double   proto_rd_latlon(proto_rd_t *r);

#endif /* PROTO_H */
