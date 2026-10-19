/*
 * hal_tc375.c - hal.h implemented for the KIT_A2G_TC375_LITE board (iLLD).
 *
 * Status: first version, no sensors or motor connected yet.
 *   - hal_millis()        : real time from STM0
 *   - sensors             : always "no new data"  -> autopilot raises the
 *                           sensor-timeout alarm 2 s after startup
 *   - motor               : not connected (TODO)
 *   - buttons             : no presses (TODO)
 *   - alarm               : LED1 (P00.5), lit = alarm
 *
 * This file is only built by ADS. The PC simulation uses sim/hal_sim.c.
 */
#include "hal.h"

#include "Ifx_Types.h"
#include "IfxStm_reg.h"   /* STM0 registers (the IfxStm driver module is   */
#include "IfxScuCcu.h"    /* not enabled in this ADS project, so we read   */
#include "IfxPort.h"      /* the counter directly - same as IfxStm_get())  */

/* LED1 on the Lite Kit. The LEDs are active LOW: pin low = LED on. */
#define ALARM_LED_PORT  (&MODULE_P00)
#define ALARM_LED_PIN   5u

static uint32 s_stm_ticks_per_ms = 0u;

/* 64-bit STM0 counter. Reading TIM0 latches the upper 32 bits into CAP,
 * so the two halves always belong together. */
static uint64 stm0_ticks(void)
{
    uint64 t = MODULE_STM0.TIM0.U;
    t |= ((uint64)MODULE_STM0.CAP.U) << 32;
    return t;
}

void hal_init(void)
{
    /* STM0 runs from reset; we only need its frequency (100 MHz by default). */
    s_stm_ticks_per_ms = (uint32)(IfxScuCcu_getStmFrequency() / 1000.0f);

    /* Alarm LED: push-pull output, start OFF (high). */
    IfxPort_setPinHigh(ALARM_LED_PORT, ALARM_LED_PIN);
    IfxPort_setPinModeOutput(ALARM_LED_PORT, ALARM_LED_PIN,
                             IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    /* TODO: rudder motor driver - make sure it is OFF here. */
    /* TODO: compass interface (e.g. NMEA 0183 on ASCLIN UART). */
    /* TODO: rudder angle sensor (e.g. potentiometer on EVADC). */
    /* TODO: button inputs with debounce. */
}

uint32_t hal_millis(void)
{
    if (s_stm_ticks_per_ms == 0u) {
        return 0u;   /* hal_init() not called yet; avoid dividing by zero */
    }
    /* 64-bit STM counter / ticks per ms; the result wraps at 2^32 ms like
     * the HAL contract says. */
    return (uint32_t)(stm0_ticks() / s_stm_ticks_per_ms);
}

bool hal_read_heading(float *heading_deg)
{
    (void)heading_deg;
    return false;    /* TODO: real compass */
}

bool hal_read_rudder_angle(float *rudder_deg)
{
    (void)rudder_deg;
    return false;    /* TODO: real rudder angle sensor */
}

void hal_set_rudder_motor(float command)
{
    (void)command;   /* TODO: PWM + direction to the motor driver */
}

bool hal_read_yaw_rate(float *yaw_dps)
{
    (void)yaw_dps;
    return false;    /* TODO: gyro (e.g. IMU on QSPI/I2C) */
}

bool hal_calib_save(const void *data, uint32_t size)
{
    (void)data;
    (void)size;
    return false;    /* TODO: write to data flash (DFLASH0, IfxFlash) */
}

bool hal_calib_load(void *data, uint32_t size)
{
    (void)data;
    (void)size;
    return false;    /* TODO: read from data flash */
}

void hal_set_throttle(float percent)
{
    (void)percent;   /* TODO: throttle servo PWM (GTM TOM) */
}

void hal_set_gear_pwm(uint16_t pulse_us)
{
    (void)pulse_us;  /* TODO: gear servo PWM (GTM TOM, 50 Hz) */
}

bool hal_read_gps(hal_gps_t *gps)
{
    (void)gps;
    return false;    /* TODO: NMEA parser on ASCLIN UART */
}

bool hal_read_controls(hal_controls_t *c)
{
    (void)c;
    return false;    /* TODO: control panel (ADC levers / buttons) */
}

uint32_t hal_get_button_presses(void)
{
    return 0u;       /* TODO: read and debounce the buttons */
}

void hal_set_alarm(bool on)
{
    if (on) {
        IfxPort_setPinLow(ALARM_LED_PORT, ALARM_LED_PIN);    /* LED on  */
    } else {
        IfxPort_setPinHigh(ALARM_LED_PORT, ALARM_LED_PIN);   /* LED off */
    }
}
