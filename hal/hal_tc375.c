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
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"
#include "IfxCpu.h"        /* __dsync() */

/* LED1 on the Lite Kit. The LEDs are active LOW: pin low = LED on. */
#define ALARM_LED_PORT  (&MODULE_P00)
#define ALARM_LED_PIN   5u

/* Actuator outputs (WIRING.md). Driven LOW = no servo pulses / motor off
 * as the very first thing in hal_init(); before that (during reset) the
 * pins are inputs and the external pull-down resistors keep them low. */
#define THROTTLE_PWM_PORT (&MODULE_P02)
#define THROTTLE_PWM_PIN  7u        /* D9  */
#define GEAR_PWM_PORT     (&MODULE_P02)
#define GEAR_PWM_PIN      6u        /* D8  */
#define MOTOR_PWM_PORT    (&MODULE_P02)
#define MOTOR_PWM_PIN     5u        /* D6  */
#define MOTOR_DIR_PORT    (&MODULE_P02)
#define MOTOR_DIR_PIN     4u        /* D7  */
#define RELAY_PORT        (&MODULE_P10)
#define RELAY_PIN         5u        /* D10: safety relay, high = closed (actuators powered) */

static uint32 s_stm_ticks_per_ms = 0u;

/* Monitor trip latch (written by CPU1): normal output calls can't undo it. */
static volatile boolean s_forced = FALSE;
static volatile uint16  s_safe_thr_us = 1000u, s_safe_gear_us = 1500u;

static void pin_low_output(Ifx_P *port, uint8 pin)
{
    IfxPort_setPinLow(port, pin);
    IfxPort_setPinModeOutput(port, pin, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
}

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
    /* FIRST: actuator pins to a defined safe state (no pulses, motor off,
     * safety relay open) - before anything else can go wrong. */
    pin_low_output(THROTTLE_PWM_PORT, THROTTLE_PWM_PIN);
    pin_low_output(GEAR_PWM_PORT, GEAR_PWM_PIN);
    pin_low_output(MOTOR_PWM_PORT, MOTOR_PWM_PIN);
    pin_low_output(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    pin_low_output(RELAY_PORT, RELAY_PIN);

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

void hal_mem_barrier(void)
{
    __dsync();      /* TriCore: all data writes done before going on */
}

bool hal_read_supply_voltage(float *volts)
{
    (void)volts;
    return false;    /* TODO: battery voltage divider on EVADC (A2, AN37) */
}

/* microSD on the mikroBUS socket (microSD click): QSPI2, FatFs. */
bool hal_sd_present(void)
{
    return false;    /* TODO: card detect (mikroBUS INT, P10.7) + f_mount() */
}

int hal_sd_open(const char *name)
{
    (void)name;
    return -1;       /* TODO: f_open(FA_CREATE_ALWAYS | FA_WRITE) */
}

bool hal_sd_write(int h, const void *data, uint32_t len)
{
    (void)h; (void)data; (void)len;
    return false;    /* TODO: f_write */
}

bool hal_sd_sync(int h)
{
    (void)h;
    return false;    /* TODO: f_sync */
}

bool hal_sd_close(int h)
{
    (void)h;
    return false;    /* TODO: f_close */
}

uint32_t hal_sd_free_kb(void)
{
    return 0u;       /* TODO: f_getfree */
}

uint32_t hal_micros(void)
{
    if (s_stm_ticks_per_ms < 1000u) return 0u;
    return (uint32_t)(stm0_ticks() / (s_stm_ticks_per_ms / 1000u));
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
    if (s_forced) command = 0.0f;   /* monitor tripped: motor stays off */
    (void)command;   /* TODO: PWM (D6 = P02.5) + direction (D7 = P02.4) to the motor driver */
}

void hal_safe_outputs(uint16_t throttle_idle_us, uint16_t gear_neutral_us)
{
    s_safe_thr_us = throttle_idle_us;
    s_safe_gear_us = gear_neutral_us;
    s_forced = TRUE;
    /* TODO (with the GTM PWM): write the idle / neutral pulses directly here */
    IfxPort_setPinLow(MOTOR_PWM_PORT, MOTOR_PWM_PIN);
    IfxPort_setPinLow(RELAY_PORT, RELAY_PIN);            /* relay open */
    IfxPort_setPinLow(ALARM_LED_PORT, ALARM_LED_PIN);    /* LED on */
}

bool hal_outputs_forced(void)
{
    return s_forced != FALSE;
}

void hal_set_safety_relay(bool closed)
{
    if (closed && !s_forced) IfxPort_setPinHigh(RELAY_PORT, RELAY_PIN);
    else IfxPort_setPinLow(RELAY_PORT, RELAY_PIN);
}

/* WDT counts from RELOAD up to 0xFFFF at fSPB / 16384 (default input
 * divider): timeout = (0x10000 - reload) * 16384 / fSPB. */
void hal_watchdog_init(uint32_t timeout_ms)
{
    float32 tick_hz = IfxScuCcu_getSpbFrequency() / 16384.0f;
    uint32 ticks = (uint32)((float32)timeout_ms * tick_hz / 1000.0f);
    if (ticks < 2u) ticks = 2u;
    if (ticks > 0xFFF0u) ticks = 0xFFF0u;
    uint16 reload = (uint16)(0x10000u - ticks);
    uint16 pw = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_changeCpuWatchdogReload(pw, reload);
    IfxScuWdt_enableCpuWatchdog(pw);
    uint16 spw = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_changeSafetyWatchdogReload(spw, reload);
    IfxScuWdt_enableSafetyWatchdog(spw);
}

void hal_watchdog_kick(void)
{
    IfxScuWdt_serviceCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_serviceSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
}

hal_reset_t hal_reset_reason(uint32_t *raw)
{
    IfxScuRcu_ResetCode rc = IfxScuRcu_evaluateReset();
    if (raw) *raw = IfxScuRcu_readRawResetStatus();
    switch (rc.resetType) {
    case IfxScuRcu_ResetType_coldpoweron:
    case IfxScuRcu_ResetType_warmpoweron: return HAL_RESET_POWER_ON;
    case IfxScuRcu_ResetType_system:      return HAL_RESET_SYSTEM;
    case IfxScuRcu_ResetType_application: return HAL_RESET_APPLICATION;
    case IfxScuRcu_ResetType_debug:       return HAL_RESET_DEBUG;
    default:                              return HAL_RESET_OTHER;
    }
}

bool hal_read_yaw_rate(float *yaw_dps)
{
    (void)yaw_dps;
    return false;    /* TODO: gyro (e.g. IMU on QSPI/I2C) */
}

bool hal_read_motor_current(float *amps)
{
    (void)amps;
    return false;    /* TODO: motor driver current sense on EVADC */
}

bool hal_read_limit_switches(bool *left, bool *right)
{
    (void)left;
    (void)right;
    return false;    /* TODO: digital inputs, if limit switches are fitted */
}

bool hal_read_roll_rate(float *roll_dps)
{
    (void)roll_dps;
    return false;    /* TODO: roll axis of the IMU, if fitted */
}

uint32_t hal_read_accel_z(float *g, uint32_t max)
{
    (void)g;
    (void)max;
    return 0u;       /* TODO: BNO085 (SPI, linear acceleration report at 200 Hz) into a ring buffer */
}

bool hal_read_attitude(float *roll_deg, float *pitch_deg)
{
    (void)roll_deg;
    (void)pitch_deg;
    return false;    /* TODO: BNO085 game rotation vector report (50 Hz) -> roll / pitch */
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

void hal_set_throttle_pwm(uint16_t pulse_us)
{
    if (s_forced) pulse_us = s_safe_thr_us;     /* monitor tripped: idle, latched */
    if (pulse_us < HAL_THROTTLE_PWM_MIN) pulse_us = HAL_THROTTLE_PWM_MIN;
    if (pulse_us > HAL_THROTTLE_PWM_MAX) pulse_us = HAL_THROTTLE_PWM_MAX;
    (void)pulse_us;  /* TODO: throttle servo PWM, 50 Hz (GTM TOM, D9 = P02.7, see WIRING.md) */
}

void hal_set_gear_pwm(uint16_t pulse_us)
{
    if (s_forced) pulse_us = s_safe_gear_us;    /* monitor tripped: neutral, latched */
    (void)pulse_us;  /* TODO: gear servo PWM, 50 Hz (GTM TOM, D8 = P02.6) */
}

bool hal_read_neutral_switch(bool *in_neutral)
{
    (void)in_neutral;
    return false;    /* TODO: digital input, if a neutral switch is fitted */
}

bool hal_read_gps(hal_gps_t *gps)
{
    (void)gps;
    return false;    /* TODO: NMEA parser on ASCLIN UART */
}

bool hal_read_joystick(hal_joystick_t *j)
{
    (void)j;
    return false;    /* TODO: EVADC AN39 (P40.9, Arduino A0) = X, AN38 (P40.8, A1) = Y;
                        P02.0 (D2) = TAKE, P02.1 (D3) = AUTO, P10.4 (D4) = CONFIRM, inputs with pull-up,
                        pressed = low. See WIRING.md. */
}

uint32_t hal_bridge_read(uint8_t *buf, uint32_t max)
{
    (void)buf; (void)max;
    return 0u;       /* TODO: ASCLIN3, RX = P20.3 (IfxAsclin3_RXC_P20_3_IN, Shield2Go 2 RX),
                        230400 8N1, RX FIFO / ring filled by interrupt; TX P20.0 unused.
                        See WIRING.md section 6. */
}

bool hal_nv_save(uint8_t slot, const void *data, uint32_t size)
{
    (void)slot; (void)data; (void)size;
    return false;    /* TODO: data flash (DFLASH0), one sector per slot */
}

bool hal_nv_load(uint8_t slot, void *data, uint32_t size)
{
    (void)slot; (void)data; (void)size;
    return false;    /* TODO: data flash */
}

bool hal_read_water_speed(float *kn)
{
    (void)kn;
    return false;    /* TODO: paddlewheel pulse input (GPT12 / GTM TIM), if fitted */
}

uint32_t hal_get_button_presses(void)
{
    return 0u;       /* TODO: read and debounce the buttons */
}

void hal_set_alarm(bool on)
{
    if (s_forced) on = true;                    /* monitor tripped: alarm stays on */
    if (on) {
        IfxPort_setPinLow(ALARM_LED_PORT, ALARM_LED_PIN);    /* LED on  */
    } else {
        IfxPort_setPinHigh(ALARM_LED_PORT, ALARM_LED_PIN);   /* LED off */
    }
}

uint32_t hal_link_read(uint8_t *buf, uint32_t max)
{
    (void)buf; (void)max;
    return 0u;       /* TODO: ASCLIN0 (P14.0 TX / P14.1 RX = USB virtual COM via the
                        on-board miniWiggler), 921600 8N1, RX FIFO / interrupt
                        ring buffer (IfxAsclin_Asc_read, non-blocking) */
}

bool hal_link_write(const uint8_t *buf, uint32_t len)
{
    (void)buf; (void)len;
    return false;    /* TODO: IfxAsclin_Asc_write into the TX ring buffer; all or nothing */
}

uint32_t hal_gw_read(uint8_t *buf, uint32_t max)
{
    (void)buf; (void)max;
    return 0u;       /* TODO: ASCLIN1, RX = P15.1 (IfxAsclin1_RXA_P15_1_IN), TX = P15.0
                        (IfxAsclin1_TX_P15_0_OUT), 921600 8N1, interrupt-filled rings
                        (RX 2 KB, TX 8 KB). To the Raspberry Pi 5 UART0 (GPIO14/15).
                        See WIRING.md section 7. */
}

bool hal_gw_write(const uint8_t *buf, uint32_t len)
{
    (void)buf; (void)len;
    return false;    /* TODO: into the ASCLIN1 TX ring; all or nothing */
}

uint32_t hal_gw_tx_free(void)
{
    return 0u;       /* TODO: free bytes in the ASCLIN1 TX ring */
}
