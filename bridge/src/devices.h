/*
 * devices.h - known USB joysticks: which role (stick / throttle) and the
 * raw range of each axis, so the bridge can normalise to -32767..+32767.
 *
 * Axis index = HID usage - 0x30:  0 X, 1 Y, 2 Z, 3 Rx, 4 Ry, 5 Rz,
 * 6 Slider, 7 Dial.  (Usage 0x39 = hat switch, read separately.)
 *
 * Any other joystick: run learn mode and calibrate ('c'), see README.md.
 * A calibration saved in EEPROM always replaces the table values.
 * An axis with no range (lo >= hi) is sent as "invalid" - the AURIX then
 * refuses to arm the joystick if that axis is mapped to steering/throttle.
 */
#ifndef DEVICES_H
#define DEVICES_H

#include <stdint.h>

#define ROLE_STICK    0
#define ROLE_THROTTLE 1
#define ROLE_ANY      0xFF

typedef struct {
    uint16_t vid, pid;
    uint8_t  role;
    const char *name;
    int32_t  lo[8], hi[8];
} known_dev_t;

static const known_dev_t KNOWN_DEVS[] = {
    /* Thrustmaster T.16000M FCS stick: X/Y 14 bit, twist (Rz) and the
     * small base slider 8 bit. */
    { 0x044F, 0xB10A, ROLE_STICK, "Thrustmaster T.16000M",
      { 0, 0, 0, 0, 0, 0, 0, 0 },
      { 16383, 16383, 0, 0, 0, 255, 255, 0 } },
    /* Thrustmaster TWCS throttle. UNVERIFIED ranges (no unit on the bench
     * yet): main lever and rocker assumed 16 bit, mini-stick 10 bit.
     * Check with learn mode and calibrate ('c') before first use. */
    { 0x044F, 0xB687, ROLE_THROTTLE, "Thrustmaster TWCS",
      { 0, 0, 0, 0, 0, 0, 0, 0 },
      { 1023, 1023, 65535, 0, 0, 65535, 0, 0 } },
};

#define KNOWN_DEV_COUNT (sizeof KNOWN_DEVS / sizeof KNOWN_DEVS[0])

#endif /* DEVICES_H */
