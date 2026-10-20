/*
 * jsframe.h - USB joystick bridge -> AURIX frame (UART, 3.3 V, 230400 8N1).
 *
 * Shared by the bridge firmware (bridge/, Teensy 4.1) and the AURIX
 * (joystick.c), so both sides always agree. Pure C99.
 *
 * The bridge sends one frame every 10 ms (100 Hz); the AURIX sends nothing
 * safety-critical back.
 *
 *   0xA5 0x5A | len (=JSF_PAYLOAD) | payload | crc_lo crc_hi
 *   CRC-16/CCITT-FALSE over len + payload.
 *
 * Payload (little-endian):
 *   u8 seq          +1 every frame (a stuck counter = a stuck bridge)
 *   u8 flags        b0 device 0 connected, b1 device 1 connected,
 *                   b2 learn mode on, b3 USB hub seen
 *   2 x device:     i16 axis[8]  normalised -32767..+32767, -32768 = no such
 *                                axis / invalid value
 *                   u32 buttons  bit n = button n+1 pressed
 *                   u8  hats     low nibble hat 0, high nibble hat 1:
 *                                0 up, 1 up-right, 2 right ... 7 up-left,
 *                                15 centred / no hat
 *                   u16 vid, u16 pid (USB ids, 0 = none)
 *
 * Device 0 = the stick (e.g. Thrustmaster T.16000M), device 1 = the
 * throttle (e.g. TWCS). See bridge/README.md.
 */
#ifndef JSFRAME_H
#define JSFRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JSF_SYNC1       0xA5u
#define JSF_SYNC2       0x5Au
#define JSF_DEVS        2
#define JSF_AXES        8
#define JSF_AXIS_NONE   (-32768)
#define JSF_HAT_CENTRE  15u
#define JSF_DEV_BYTES   (JSF_AXES * 2 + 4 + 1 + 4)
#define JSF_PAYLOAD     (2 + JSF_DEVS * JSF_DEV_BYTES)          /* 52 */
#define JSF_FRAME_LEN   (3 + JSF_PAYLOAD + 2)                    /* 57 */
#define JSF_BAUD        230400u
#define JSF_PERIOD_MS   10u

#define JSF_F_DEV0      0x01u
#define JSF_F_DEV1      0x02u
#define JSF_F_LEARN     0x04u
#define JSF_F_HUB       0x08u

typedef struct {
    int16_t  axis[JSF_AXES];
    uint32_t buttons;
    uint8_t  hats;
    uint16_t vid, pid;
} jsf_dev_t;

typedef struct {
    uint8_t   seq;
    uint8_t   flags;
    jsf_dev_t dev[JSF_DEVS];
} jsf_t;

typedef struct {
    uint8_t  buf[512];
    uint32_t head, tail;
    uint32_t frames, crc_errors, junk;
} jsf_rx_t;

uint16_t jsf_crc16(const uint8_t *p, size_t n);
/* Frame into out[JSF_FRAME_LEN]; returns JSF_FRAME_LEN. */
size_t jsf_encode(const jsf_t *f, uint8_t *out);
void jsf_dev_clear(jsf_dev_t *d);           /* no axes, no buttons, hats centred */

void jsf_rx_init(jsf_rx_t *r);
void jsf_rx_push(jsf_rx_t *r, const uint8_t *data, size_t n);
/* Next good frame. A bad CRC drops one byte and rescans. */
bool jsf_rx_next(jsf_rx_t *r, jsf_t *f);

/* Control codes - one number per button or hat direction; used by the
 * button map (AURIX config), the bridge learn mode and the ground station.
 *   0           none
 *   1..64       button: 1 + dev*32 + index (index 0 = "button 1")
 *   65..80      hat:    65 + dev*8 + hat*4 + dir (dir 0 up, 1 right, 2 down, 3 left;
 *                       only the exact direction counts, diagonals do not)
 * Axis codes (axis map): dev*8 + axis, 0xFF = none. */
#define JSF_CODE_NONE           0u
#define JSF_CODE_BTN(dev, i)    ((uint8_t)(1u + (dev) * 32u + (i)))
#define JSF_CODE_HAT(dev, h, d) ((uint8_t)(65u + (dev) * 8u + (h) * 4u + (d)))
#define JSF_CODE_MAX            80u
#define JSF_HAT_UP 0u
#define JSF_HAT_RIGHT 1u
#define JSF_HAT_DOWN 2u
#define JSF_HAT_LEFT 3u
#define JSF_AXIS_CODE(dev, a)   ((uint8_t)((dev) * 8u + (a)))
#define JSF_AXIS_CODE_NONE      0xFFu

bool jsf_code_pressed(const jsf_t *f, uint8_t code);

/* Raw USB axis value -> -32767..+32767 over the range [lo, hi].
 * JSF_AXIS_NONE if the range is unknown (lo >= hi) or raw is outside it
 * by more than 2 % (a wrong range table must not look like a valid value). */
int16_t jsf_norm(int32_t raw, int32_t lo, int32_t hi, bool invert);

/* Hat direction (0..7, 15) of hat h. */
static inline uint8_t jsf_hat(const jsf_dev_t *d, int h)
{
    return (uint8_t)((h ? (d->hats >> 4) : d->hats) & 0x0Fu);
}

#ifdef __cplusplus
}
#endif

#endif /* JSFRAME_H */
