/*
 * logrec.h - Log record format (".TLG" files) and the core 0 producer.
 *
 * File = "TLG1" magic, then records:
 *   0xA5 | type | len (u16) | t_ms (u32) | payload[len] | crc (u16, CRC-16/CCITT over type..payload)
 * A record cut off by a power loss fails its CRC: readers stop there.
 *
 * Record types:
 *   LOGR_HEADER  text: format version, boot / file number, the list of data
 *                fields (name:type:scale) and of event codes (code:NAME:args),
 *                and the names of the modes / reasons / alarms
 *   LOGR_CONFIG  text: config + calibration (key=value lines); written at
 *                the start of every file, and again when it changes
 *   LOGR_DATA    one 20 Hz sample, binary, laid out as the header says
 *   LOGR_EVENT   u16 code + 4 x i32 arguments (mode change, alarm, ...)
 *   LOGR_PCCMD   u8 type, u8 seq, u8 result, u8 len, payload (full PC command)
 *   LOGR_TEXT    free text (logger notes)
 * Little-endian. Types in the field list: B u8, c i8, H u16, h i16, I u32,
 * i i32, f f32; a stored integer = value x scale.
 *
 * Core 0 only pushes into the lock-free queue (logq.c); it never waits.
 */
#ifndef LOGREC_H
#define LOGREC_H

#include <stdbool.h>
#include <stdint.h>

#include "logq.h"

#define LOG_FORMAT_VERSION  1u
#define LOGR_SYNC           0xA5u
#define LOGR_HDR            8u      /* sync, type, len(2), t(4) */
#define LOGR_OVERHEAD       10u     /* + crc(2) */
#define LOGR_MAX            4096u   /* largest record (config text) */

enum {
    LOGR_HEADER = 1, LOGR_CONFIG = 2, LOGR_DATA = 0x10, LOGR_EVENT = 0x20, LOGR_PCCMD = 0x22, LOGR_TEXT = 0x30
};

/* Event codes (args a, b, c, d) - names / arg names go into the header. */
typedef enum {
    EV_BOOT = 1,        /* reset reason, raw reset status, boot number, format version */
    EV_MODE,            /* from, to, why, verdict (0 ok, 1 table, 2 precondition) */
    EV_ALARM_ON,        /* new bits, all bits, ap alarm, 0 */
    EV_ALARM_OFF,       /* cleared bits, all bits, 0, 0 */
    EV_BUTTON,          /* panel button mask, 0, 0, 0 */
    EV_JOY_BUTTON,      /* bit0 TAKE, bit1 AUTO, bit2 CONFIRM pressed, stick x*1000, y*1000, armed */
    EV_PLIM,            /* source | result<<8 | attempt<<16 | dir<<24, fwd old*10, fwd new*10, rev old*10<<16 | rev new*10 */
    EV_TRAINING,        /* stage, phase, result, 0 */
    EV_CONFIG,          /* what (1 param, 2 training, 3 joystick cal, 4 program), id, old*1000, new*1000 */
    EV_GPS,             /* position source (dr_state), reason, uncertainty*10, gps losses */
    EV_FAULT,           /* kind (1 gear, 2 compass, 3 joystick, 4 monitor, 5 overrun, 6 log), a, b, c */
    EV_PROGRAM,         /* action (0 stored, 1 started, 2 paused, 3 resumed, 4 skipped, 5 finished,
                           6 waypoint reached), waypoints / wp, state, legs done */
    EV_CONFIRM,         /* kind, result (0 accepted, 1 refused, 2 ignored), counts */
    EV_MONITOR,         /* reason (CPU1), throttle*10, limit*10, 0 */
    EV_LOG,             /* what (1 file, 2 sync, 3 black box start, 4 black box end, 5 failure, 6 drops),
                           a, b, c */
    EV_TIME,            /* GPS date YYYYMMDD, GPS time of day ms, local t_ms, 0 (once a minute) */
    EV_HELM,            /* station, action (0 take, 1 release, 2 station lost, 3 automatic take by the
                           USB laptop, 4 gateway lost), result (0 granted, 1 refused), new holder */
    EV_SAVE,            /* video SAVE request: trigger (SB_SAVE_), arg, seq, 0 */
    EV_SLAM,            /* impact / slamming change: peak g*100 (0 = no impact), level g*100, impacts per min,
                           cap %*10 | state<<16 | override<<20 */
    EV_NAV,             /* navigation request change: kind | reason<<8 | alarm<<16 | lost<<24 | applied<<25,
                           heading*100, cap kn*10 (-1 none), seq */
    EV_PROFILE,         /* operating profile change: profile (0 detecting 1 CREWED 2 UNCREWED), onboard joystick
                           detected, changes, 0 */
    EV_LINKLOST,        /* UNCREWED lost link: state (0 ok 1 lost 2 killed), near shore, shore distance m, losses */
    EV_NV,              /* non-volatile storage: what (1 record unusable -> defaults, 2 NV FAULT), slot, resets, 0 */
    EV_GENERATOR,       /* generator mode: state (GEN_), stop why | enabled << 8, bank V*100, throttle demand %*10 */
    EV_RELAY,           /* relay board: n | action<<8 | result<<16 | what<<24 (1 command, 2 table, 3 states, 4 board),
                           states (readback bits), known bits | board ok<<31, table hash | flags<<16 | count<<24 */
    /* 26: not used (kept free) */
    EV_RESTART = 27,    /* engine controls / remote restart (section N): action (1 kill cleared, 2 reset after a trip
                           coming, 3 crank start, 4 crank stop, 5 restart refused, 6 crank refused, 7 reset cancelled),
                           why (engine.h eng_why_t), hold ms, restarts | cranks << 16 */
    EV_DP = 28,         /* DP (section P; step 9: the thrusters): what (1 settings taken, 2 thruster HOT, 3 cooled
                           down), unit (0 bow 1 stern), bow max % | stern max % << 8, heat used % | settings hash << 8 */
    EV_COUNT
} log_event_t;

/* Build one record (with sync / len / crc) into out; returns its length. */
uint16_t logrec_make(uint8_t *out, uint8_t type, uint32_t t_ms, const void *payload, uint16_t len);

/* Event into any queue (core 0: g_logq0, CPU1: g_logq1). */
bool logrec_event(logq_t *q, uint32_t t_ms, uint16_t code, int32_t a, int32_t b, int32_t c, int32_t d);

/* Header text (fields, events, names) for a new file. */
uint16_t logrec_header_text(char *out, uint16_t max, uint32_t boot, uint32_t file_no);

/* Size of one DATA payload (fixed). */
uint16_t logrec_data_size(void);

#endif /* LOGREC_H */
