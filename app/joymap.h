/*
 * joymap.h - Joystick source and USB button / axis map.
 *
 * Which joystick drives the boat (hard-wired analog or the USB bridge),
 * and for the USB bridge which button / hat / axis does what. Stored in NV
 * slot 4 with version + CRC-32; edited from the ground station (STANDBY
 * only). The rest of the app sees the same joystick (joystick.h) whatever
 * the source.
 *
 * Codes: app/jsframe.h (1..64 buttons, 65..80 hat directions, 0 = none;
 * axes dev*8 + axis). TAKE, AUTO and CONFIRM must be set and every mapped
 * button must be different - a map that shares a button is refused.
 */
#ifndef JOYMAP_H
#define JOYMAP_H

#include <stdbool.h>
#include <stdint.h>

#define JOYMAP_MAGIC    0x4A4D4150u     /* "JMAP" */
#define JOYMAP_VERSION  1u

typedef enum { JOY_SRC_ANALOG = 0, JOY_SRC_BRIDGE = 1 } joy_source_t;

/* Throttle input: spring-centred stick Y (centre = idle + neutral) or a
 * non-returning lever (neutral zone in the middle). */
typedef enum { JOY_THR_STICK_Y = 0, JOY_THR_LEVER = 1 } joy_thr_input_t;

typedef enum {
    JM_TAKE, JM_AUTO, JM_CONFIRM, JM_HDG_LOCK, JM_STANDBY,
    JM_MINUS_1, JM_PLUS_1, JM_MINUS_10, JM_PLUS_10,
    JM_COUNT
} jm_button_t;

typedef struct {
    uint32_t magic, version, size;
    uint8_t  source;            /* joy_source_t */
    uint8_t  thr_input;         /* joy_thr_input_t (USB bridge; analog is always STICK_Y) */
    uint8_t  axis_steer;        /* axis code: steering (stick X) */
    uint8_t  axis_thr;          /* axis code: lever or stick Y */
    uint8_t  inv_steer, inv_thr;
    uint8_t  btn[JM_COUNT];     /* control codes, 0 = not mapped */
    uint8_t  pad;
    float    nz_lo, nz_hi;      /* lever neutral zone, 0..1 of travel (0.45 / 0.55) */
    float    lever_delta;       /* lever moved this far from its hand-back position = override (0.10) */
    float    pickup_win;        /* throttle pickup window around the target (0.03) */
    uint32_t checksum;          /* CRC-32 of everything above */
} joymap_t;

typedef enum {
    JM_OK = 0,
    JM_ERR_MISSING,     /* TAKE, AUTO or CONFIRM not mapped */
    JM_ERR_DUPLICATE,   /* two functions share a button */
    JM_ERR_CODE,        /* code / axis / source / input out of range */
    JM_ERR_ZONE         /* neutral zone / delta / window implausible */
} jm_error_t;

/* Factory default: analog joystick; USB map for T.16000M + TWCS. */
void joymap_default(joymap_t *m);
jm_error_t joymap_check(const joymap_t *m);
void joymap_seal(joymap_t *m);
bool joymap_valid(const joymap_t *m);           /* checksum + check */
const char *joymap_error_name(jm_error_t e);
const char *joymap_button_name(jm_button_t b);

/* Bit mask of the USB devices (bit 0 stick, bit 1 throttle) the map uses -
 * those must be connected. */
uint8_t joymap_devices(const joymap_t *m);

#endif /* JOYMAP_H */
