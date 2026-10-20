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
#define PROTO_T_SET_MODE      0x10u   /* u8 mode (sb_mode_t)                  */
#define PROTO_T_JOY_HANDBACK  0x11u   /* -  (only with the stick centred)     */
#define PROTO_T_HEADING_ADJ   0x12u   /* i8 delta: -10, -1, +1, +10 (CRUISE)  */
#define PROTO_T_WAYPOINT      0x13u   /* i32 lat, i32 lon (1e-7 deg), f32 kn  */
#define PROTO_T_CRUISE_SPEED  0x14u   /* f32 kn                               */
#define PROTO_T_SEA_STATE     0x15u   /* u8 1..5                              */
#define PROTO_T_TRAINING      0x16u   /* u8 0 start / 1 accept / 2 reject, f32 full angle (0 = unknown) */
#define PROTO_T_PARAM_GET     0x17u   /* u8 id                                */
#define PROTO_T_PARAM_SET     0x18u   /* u8 id, f32 value                     */
#define PROTO_T_JOY_CAL       0x19u   /* u8 0 start / 1 finish                */
#define PROTO_T_ALARM_ACK     0x1Au   /* -  (STANDBY only, like the button)   */
#define PROTO_T_SIM_FIRST     0x70u   /* 0x70..0x7F: simulator control only   */
#define PROTO_T_SIM_JOYSTICK  0x70u   /* u8 fitted, f32 x, f32 y, u8 take, u8 auto */
#define PROTO_T_SIM_GPS       0x71u   /* u8 1 = GPS fails, 0 = GPS back       */
#define PROTO_T_SIM_LAST      0x7Fu

/* boat -> PC */
#define PROTO_T_TELEMETRY     0x80u   /* see link.c telemetry()                */
#define PROTO_T_ACK           0x81u   /* u8 cmd type, u8 cmd seq, u8 result (PROTO_R_...) */
#define PROTO_T_PARAM_VALUE   0x82u   /* u8 id, f32 value, f32 min, f32 max, char name[16] */

/* ACK result: 0 = accepted, else why refused (NACK) */
#define PROTO_R_OK            0u
#define PROTO_R_BAD_LENGTH    1u      /* payload length wrong for this type   */
#define PROTO_R_UNKNOWN       2u      /* unknown message type                 */
#define PROTO_R_BAD_VALUE     3u      /* value out of range                   */
#define PROTO_R_JOYSTICK      4u      /* joystick is in control               */
#define PROTO_R_NOT_ALLOWED   5u      /* mode change not in the table         */
#define PROTO_R_PRECOND       6u      /* conditions not met (GPS, waypoint, sensors ...) */
#define PROTO_R_WRONG_MODE    7u      /* command not possible in this mode    */
#define PROTO_R_NOT_CENTERED  8u      /* hand-back refused: stick not centred */
#define PROTO_R_STANDBY_ONLY  9u      /* only in STANDBY                      */
#define PROTO_R_BAD_PARAM     10u     /* no such parameter                    */
#define PROTO_R_REFUSED       11u     /* refused by the function (e.g. implausible calibration) */

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
uint32_t proto_rd_u32(proto_rd_t *r);
int32_t  proto_rd_i32(proto_rd_t *r);
float    proto_rd_f32(proto_rd_t *r);
double   proto_rd_latlon(proto_rd_t *r);

#endif /* PROTO_H */
