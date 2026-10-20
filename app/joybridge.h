/*
 * joybridge.h - USB joystick source: frames from the Teensy bridge
 * (hal_bridge_read, app/jsframe.h) -> joy_input_t, mapped by joymap.
 *
 * JOYSTICK FAULT (same as a wire break on the analog joystick) when:
 *   - no valid frame for timeout_ms (150)
 *   - more than crc_max (5) CRC errors within crc_window_ms (1000)
 *   - the sequence counter repeats seq_stuck (3) frames in a row
 *   - a USB device the map uses is unplugged
 *   - a mapped axis is invalid / out of range
 *   - the bridge is in learn mode
 * During the first boot_grace_ms (5000) after power-up these only mean
 * "not ready yet" (the bridge enumerates USB) - not armed, no alarm. The
 * grace ends for good the first time everything is OK.
 */
#ifndef JOYBRIDGE_H
#define JOYBRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "joystick.h"
#include "joymap.h"
#include "jsframe.h"

typedef struct {
    uint32_t timeout_ms;        /* 150 */
    uint32_t crc_window_ms;     /* 1000 */
    uint32_t crc_max;           /* 5 */
    uint8_t  seq_stuck;         /* 3 */
    uint32_t boot_grace_ms;     /* 5000 */
} jb_config_t;

typedef struct {
    jb_config_t cfg;
    jsf_rx_t rx;
    jsf_t    last;              /* last good frame */
    bool     have;              /* at least one good frame since power-up */
    uint32_t start_ms, last_ok_ms;
    uint8_t  same_seq;          /* frames in a row with the same counter */
    uint32_t crc_seen;          /* rx.crc_errors already counted */
    uint32_t crc_win_start, crc_in_win;
    uint32_t fps_t0, fps_n;
    uint16_t fps;               /* good frames in the last second */
    uint8_t  why;               /* current problem (JOY_WHY_*), 0 = none */
    bool     grace;             /* boot grace: not yet seen working, < boot_grace_ms */
} jb_t;

void jb_default_config(jb_config_t *c);
void jb_init(jb_t *b, const jb_config_t *c, uint32_t now_ms);
/* Read the UART, check, map. */
void jb_step(jb_t *b, const joymap_t *m, uint32_t now_ms, joy_input_t *out);

#endif /* JOYBRIDGE_H */
