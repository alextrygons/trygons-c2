/*
 * calib.h - Outboard calibration record (learned by TRAINING mode).
 *
 * Without an outboard angle sensor, motor speed and boat response can't be
 * told apart from heading data, so everything is stored in motor-time
 * units (ms of motor run). Degrees are only known if the user enters one
 * measured value: the full lock-to-lock outboard angle.
 *
 * Stored with a version and a CRC-32 checksum in NV slot 5 (app/nv.c:
 * queued, written in STANDBY; an unusable record = defaults + NV DATA RESET).
 */
#ifndef CALIB_H
#define CALIB_H

#include <stdbool.h>
#include <stdint.h>

#define CALIB_MAGIC    0x43595254u   /* "TRYC" */
#define CALIB_VERSION  1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;              /* sizeof(calib_data_t) */

    /* Learned, in motor-time units */
    float gain_dps_per_ms;      /* steady yaw rate per ms of motor run (+ = right) */
    float time_const_s;         /* boat response time constant (63 % of a change) */
    float gap_ms;               /* backlash: average of both directions */
    float gap_rl_ms;            /* ... reversing from right to left */
    float gap_lr_ms;            /* ... reversing from left to right */
    float travel_right_ms;      /* hand-centred position -> right end stop */
    float travel_left_ms;       /* hand-centred position -> left end stop */
    float trim_dps;             /* yaw rate with the outboard centred by hand */
    float gyro_bias_dps;        /* gyro offset found during CENTER (info) */

    /* Quality */
    float fit_r2;               /* 0..1: how well the model explains the log */
    float confidence;           /* 0..1: overall (fit + agreement of phases) */

    /* Entered by the user, 0 = not entered */
    float full_angle_deg;       /* measured lock-to-lock outboard angle */

    uint32_t checksum;          /* CRC-32 of all bytes above */
} calib_data_t;

/* CRC-32 over everything before the checksum field. */
uint32_t calib_checksum(const calib_data_t *c);

/* Fill magic, version, size and checksum (call after editing values). */
void calib_seal(calib_data_t *c);

/* Magic, version, size, checksum and plausible values all OK? */
bool calib_valid(const calib_data_t *c);

/* Save / load through the HAL. Load returns false if missing or invalid. */
bool calib_save(calib_data_t *c);
bool calib_load(calib_data_t *c);

#endif /* CALIB_H */
