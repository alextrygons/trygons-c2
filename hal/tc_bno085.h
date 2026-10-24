/*
 * tc_bno085.h - BNO085 IMU over SPI (SHTP protocol): heading (rotation vector,
 * magnetometer), yaw rate (calibrated gyro), roll / pitch (game rotation
 * vector), vertical acceleration (linear acceleration, 200 Hz).
 * Serviced from the control loop (CPU0) once per step; no report for 0.5 s
 * = "no data" and the sensor is reset (at most every 2 s).
 */
#ifndef TC_BNO085_H
#define TC_BNO085_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     ok;                    /* a report within the last 0.5 s */
    uint32_t reports, packets, resets, spi_errors;
    uint32_t last_report_ms;
    float    heading_deg, yaw_dps, roll_deg, pitch_deg, accel_z_g;
    uint8_t  heading_accuracy;      /* 0 unreliable .. 3 high (magnetometer calibration) */
} bno_stat_t;

void bno_init(void);
void bno_service(void);                         /* once per control step */
bool bno_heading(float *deg);                   /* true = a new value since the last call */
bool bno_yaw_rate(float *dps);
bool bno_attitude(float *roll_deg, float *pitch_deg);   /* true = fresh (< 0.5 s) */
uint32_t bno_accel_z(float *g, uint32_t max);   /* samples since the last call */
const bno_stat_t *bno_stat(void);

#endif /* TC_BNO085_H */
