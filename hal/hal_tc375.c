/*
 * hal_tc375.c - hal.h implemented for the KIT_A2G_TC375_LITE board (iLLD).
 * Every pin is in hal_pins.h (and in WIRING.md).
 *
 * Stage 1 parts (FIRST_TEST.md):
 *   - BNO085 IMU (SPI)          : heading, yaw rate, roll / pitch, vertical
 *                                 acceleration (tc_bno085.c)
 *   - u-blox MAX-M10S GPS (UART): fix, SOG / COG, spoofing / jamming (tc_gps.c)
 *   - Pololu G2 18v17           : steering motor PWM (on / off) + DIR + SLP,
 *                                 current sense on the ADC (tc_adc.c)
 *   - 2 limit switches, e-stop contact, panel buttons (debounced)
 *   - buzzer + LED1             : alarm
 *   - microSD click             : log files, FAT32 (tc_sd.c, CPU2 only)
 *   - USB link (ASCLIN0), gateway UART (ASCLIN1), USB joystick bridge (ASCLIN3)
 *     (tc_uart.c)
 *   - throttle / gear           : 50 Hz servo PWM on GTM TOM0 ch 15 / 14
 * Each driver has a time limit and "no data" detection: a sensor that stops
 * answering simply returns no new data, and the app raises its timeout alarm.
 *
 *   - data flash DFLASH0       : NV record log (tc_nv.c: TRAINING result,
 *                                 joystick calibration + map, AUTO program,
 *                                 boot counter), written only in STANDBY
 *   - supply voltage           : divider on A2 / AN37 (tc_adc.c, HAL_SUPPLY_FITTED
 *                                 0 until the divider is wired: "not measured")
 * Not yet: analog joystick, paddlewheel, roll rate - they return "not fitted".
 *
 * This file is only built by ADS. The PC simulation uses sim/hal_sim.c.
 */
#include "hal.h"

#include "Ifx_Types.h"
#include "IfxStm_reg.h"   /* STM0 counter, read directly (same as IfxStm_get()) */
#include "IfxScuCcu.h"
#include "IfxPort.h"
#include "IfxScuWdt.h"
#include "IfxScuRcu.h"
#include "IfxCpu.h"        /* __dsync() */
#include "IfxGtm.h"        /* servo PWM: GTM TOM (low-level driver in Gtm/Std) */
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Tom.h"
#include "IfxGtm_PinMap.h"

#include "hal_pins.h"
#include "tc_adc.h"
#include "tc_bno085.h"
#include "tc_gps.h"
#include "tc_sd.h"
#include "tc_uart.h"
#include "tc_nv.h"
#include "IfxFlash.h"      /* data flash command sequences (inline; Flash/Std built by ADS) */

#include <string.h>

static void nv_port_init(void);      /* below, with the data flash port */

#define LINK_BAUD    921600.0f
#define GW_BAUD      921600.0f
#define BRIDGE_BAUD  230400.0f
#define CURRENT_FRESH_MS 100u      /* a motor current sample older than this = no data */

static uint32 s_stm_ticks_per_ms = 0u;

/* Monitor trip latch (written by CPU1): normal output calls can't undo it. */
static volatile boolean s_forced = FALSE;
static volatile uint16  s_safe_thr_us = 1000u, s_safe_gear_us = 1500u;

/* --- servo PWM (GTM TOM0, 50 Hz) --------------------------------------------
 * Each channel counts up to CM0 (period) and its output is high until CM1
 * (pulse width). New pulse widths go into the shadow register SR1 and the
 * hardware takes them over at the end of the period, so a pulse is never cut
 * short (a short pulse could look like "full throttle" to a reversed servo).
 * The pins are routed to the TOM only at the first hal_set_*_pwm(): until
 * then they stay LOW (no pulses = servo fail-safe), as hal_init() set them. */
#define SERVO_TOM         IfxGtm_Tom_0
#define THROTTLE_TOM_CH   IfxGtm_Tom_Ch_15   /* P02.7 = TOM0_15 TOUT7 */
#define GEAR_TOM_CH       IfxGtm_Tom_Ch_14   /* P02.6 = TOM0_14 TOUT6 */
#define SERVO_PERIOD_US   20000.0f
#define SERVO_PULSE_MIN   500u              /* any servo pulse is limited to this range */
#define SERVO_PULSE_MAX   2500u

static Ifx_GTM_TOM     *s_tom = NULL;
static Ifx_GTM_TOM_TGC *s_tgc = NULL;
static float32          s_tom_ticks_per_us = 0.0f;
static volatile boolean s_thr_routed = FALSE, s_gear_routed = FALSE;

static uint16 servo_ticks(uint16 pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN) pulse_us = SERVO_PULSE_MIN;
    if (pulse_us > SERVO_PULSE_MAX) pulse_us = SERVO_PULSE_MAX;
    return (uint16)((float32)pulse_us * s_tom_ticks_per_us + 0.5f);
}

static void servo_channel_init(IfxGtm_Tom_Ch ch, uint16 period, uint16 pulse)
{
    IfxGtm_Tom_Tgc_enableChannelUpdate(s_tgc, ch, FALSE);
    IfxGtm_Tom_Ch_setClockSource(s_tom, ch, IfxGtm_Tom_Ch_ClkSrc_cmuFxclk2);
    IfxGtm_Tom_Ch_setSignalLevel(s_tom, ch, Ifx_ActiveState_high);
    IfxGtm_Tom_Ch_setResetSource(s_tom, ch, IfxGtm_Tom_Ch_ResetEvent_onCm0);
    IfxGtm_Tom_Ch_setCompareZero(s_tom, ch, period);
    IfxGtm_Tom_Ch_setCompareOne(s_tom, ch, pulse);
    IfxGtm_Tom_Ch_setCompareZeroShadow(s_tom, ch, period);
    IfxGtm_Tom_Ch_setCompareOneShadow(s_tom, ch, pulse);
    IfxGtm_Tom_Tgc_enableChannelUpdate(s_tgc, ch, TRUE);           /* SR -> CM at each period end */
    IfxGtm_Tom_Tgc_enableChannel(s_tgc, ch, TRUE, FALSE);
    IfxGtm_Tom_Tgc_enableChannelOutput(s_tgc, ch, TRUE, FALSE);
}

static void servo_pwm_init(void)
{
    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, IfxGtm_Cmu_getModuleFrequency(&MODULE_GTM));
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);
    float32 f = IfxGtm_Cmu_getFxClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Fxclk_2, TRUE);   /* GTM clock / 256 */
    float32 period = SERVO_PERIOD_US * f / 1e6f;
    if (!(period > 1000.0f && period < 65535.0f)) return;   /* clock not as expected: no PWM, pins stay low */
    s_tom_ticks_per_us = f / 1e6f;
    s_tom = &MODULE_GTM.TOM[SERVO_TOM];
    s_tgc = IfxGtm_Tom_Ch_getTgcPointer(s_tom, (uint32)THROTTLE_TOM_CH / 8u);   /* ch 8..15: TGC1 */
    servo_channel_init(THROTTLE_TOM_CH, (uint16)period, servo_ticks(s_safe_thr_us));
    servo_channel_init(GEAR_TOM_CH, (uint16)period, servo_ticks(s_safe_gear_us));
    IfxGtm_Tom_Tgc_trigger(s_tgc);                                  /* start both, in step */
}

/* Write one pulse width (shadow register, taken at the period end). */
static void servo_write(IfxGtm_Tom_Ch ch, uint16 pulse_us)
{
    if (s_tom == NULL) return;
    IfxGtm_Tom_Ch_setCompareOneShadow(s_tom, ch, servo_ticks(pulse_us));
}

static void servo_route(IfxGtm_Tom_Ch ch, volatile boolean *routed)
{
    if (*routed || s_tom == NULL) return;
    *routed = TRUE;
    IfxGtm_PinMap_setTomTout(ch == THROTTLE_TOM_CH ? &IfxGtm_TOM0_15_TOUT7_P02_7_OUT : &IfxGtm_TOM0_14_TOUT6_P02_6_OUT,
                             IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed1);
}

/* G2 PWM low with SLP high = both low-side switches on = motor braked (holds
 * the outboard); SLP low = driver asleep, outputs open (monitor trip). */
static void motor_off(void)
{
    IfxPort_setPinLow(PIN_MOTOR_PWM);
}

static void pin_low_output(Ifx_P *port, uint8 pin)
{
    IfxPort_setPinLow(port, pin);
    IfxPort_setPinModeOutput(port, pin, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
}

static void pin_input_pullup(Ifx_P *port, uint8 pin)
{
    IfxPort_setPinModeInput(port, pin, IfxPort_InputMode_pullUp);
}

static boolean pin_high(Ifx_P *port, uint8 pin)
{
    return IfxPort_getPinState(port, pin);
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
     * motor driver asleep, safety relay open, buzzer off) - before anything
     * else can go wrong. */
    pin_low_output(PIN_THROTTLE_PWM);
    pin_low_output(PIN_GEAR_PWM);
    pin_low_output(PIN_MOTOR_PWM);
    pin_low_output(PIN_MOTOR_DIR);
    pin_low_output(PIN_MOTOR_SLP);
    pin_low_output(PIN_RELAY);
    pin_low_output(PIN_BUZZER);

    /* STM0 runs from reset; we only need its frequency (100 MHz by default). */
    s_stm_ticks_per_ms = (uint32)(IfxScuCcu_getStmFrequency() / 1000.0f);

    /* Alarm LED: push-pull output, start OFF (high). */
    IfxPort_setPinHigh(PIN_ALARM_LED);
    IfxPort_setPinModeOutput(PIN_ALARM_LED, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    /* inputs: buttons (to GND = pressed), limit switches + e-stop contact
     * (normally closed to GND: high = at the stop / pressed / wire broken) */
    pin_input_pullup(PIN_BTN_STANDBY);
    pin_input_pullup(PIN_BTN_HDG_LOCK);
    pin_input_pullup(PIN_BTN_MINUS_1);
    pin_input_pullup(PIN_BTN_PLUS_1);
    pin_input_pullup(PIN_BTN_MINUS_10);
    pin_input_pullup(PIN_BTN_PLUS_10);
    pin_input_pullup(PIN_LIMIT_LEFT);
    pin_input_pullup(PIN_LIMIT_RIGHT);
    pin_input_pullup(PIN_ESTOP);
    pin_input_pullup(PIN_NEUTRAL_SW);

    /* servo PWM generators running (idle / neutral pulses), pins not yet routed */
    servo_pwm_init();

    adc_init();                                 /* motor current sense */
    uart_init(UART_LINK, LINK_BAUD);
    uart_init(UART_GW, GW_BAUD);
    uart_init(UART_BRIDGE, BRIDGE_BAUD);
    gps_init();                                 /* its UART + configuration (state machine) */
    bno_init();                                 /* SPI + reset pulse (state machine) */
    nv_port_init();                             /* data flash record log: scan only (no command) */
    /* the SD card is started by CPU2 (hal_sd_present), the only core that uses it */
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

/* Supply voltage of the 12 V bank: the divider on A2 / AN37 (hal_pins.h: the
 * pin and the divider are TBD, HAL_SUPPLY_FITTED 0 until wired). The raw
 * value only: app/battery.c filters it and judges LOW BATTERY. A result
 * older than SUPPLY_FRESH_MS = no data (the converter is dead). */
#define SUPPLY_FRESH_MS 100u
bool hal_read_supply_voltage(float *volts)
{
#if HAL_SUPPLY_FITTED
    static float last_v = 0.0f;
    static uint32_t last_ms = 0u;
    static boolean have = FALSE;
    uint16_t raw;
    uint32_t now = hal_millis();
    if (adc_read(ADC_SUPPLY, &raw)) {
        last_v = adc_volts(raw) * SUPPLY_DIV_RATIO;
        last_ms = now;
        have = TRUE;
    }
    if (!have || (uint32_t)(now - last_ms) > SUPPLY_FRESH_MS) return false;
    *volts = last_v;
    return true;
#else
    (void)volts;
    return false;    /* divider not wired yet (HAL_SUPPLY_FITTED 0): NO VOLTAGE DATA, no alarm */
#endif
}

/* microSD on the mikroBUS socket (microSD click): QSPI2, FAT32 (tc_sd.c, tc_fat32.c).
 * CPU2 only. Every call does bounded work (HAL_SD_MAX_MS). */
bool hal_sd_present(void)
{
    return sd_present();
}

int hal_sd_open(const char *name)
{
    return sd_open(name);
}

bool hal_sd_write(int h, const void *data, uint32_t len)
{
    return sd_write(h, data, len);
}

bool hal_sd_sync(int h)
{
    return sd_sync(h);
}

bool hal_sd_close(int h)
{
    return sd_close(h);
}

uint32_t hal_sd_free_kb(void)
{
    return sd_free_kb();
}

uint32_t hal_micros(void)
{
    if (s_stm_ticks_per_ms < 1000u) return 0u;
    return (uint32_t)(stm0_ticks() / (s_stm_ticks_per_ms / 1000u));
}

/* --- BNO085: serviced at most once per control step, by whichever read comes first */
static void imu_service(void)
{
    static uint32_t last_ms;
    static boolean once = FALSE;
    uint32_t now = hal_millis();
    if (once && (uint32_t)(now - last_ms) < 10u) return;
    once = TRUE;
    last_ms = now;
    bno_service();
}

bool hal_read_heading(float *heading_deg)
{
    imu_service();
    return bno_heading(heading_deg);
}

bool hal_read_yaw_rate(float *yaw_dps)
{
    imu_service();
    return bno_yaw_rate(yaw_dps);
}

bool hal_read_rudder_angle(float *rudder_deg)
{
    (void)rudder_deg;
    return false;    /* big boat only: rudder angle sensor (not fitted on the small boat) */
}

void hal_set_rudder_motor(float command)
{
    /* the outboard steering runs as -1 / 0 / +1 (estimate from the run time) */
    if (s_forced || !(command > 0.5f || command < -0.5f)) {    /* also NaN -> off */
        motor_off();
    } else {
        IfxPort_setPinHigh(PIN_MOTOR_SLP);                      /* driver awake */
        if (command > 0.0f) IfxPort_setPinHigh(PIN_MOTOR_DIR);
        else IfxPort_setPinLow(PIN_MOTOR_DIR);
        IfxPort_setPinHigh(PIN_MOTOR_PWM);
    }
    __dsync();
    if (s_forced) {                                             /* tripped meanwhile (CPU1): off again */
        motor_off();
        IfxPort_setPinLow(PIN_MOTOR_SLP);
    }
}

/* Called by CPU1 (monitor trip): flag FIRST, then the hardware. A core-0
 * output call that slipped in between sees the flag after its own write and
 * writes the safe value again, so the safe state always ends up in the
 * hardware - also when core 0 is frozen and never writes again. */
void hal_safe_outputs(uint16_t throttle_idle_us, uint16_t gear_neutral_us)
{
    s_safe_thr_us = throttle_idle_us;
    s_safe_gear_us = gear_neutral_us;
    s_forced = TRUE;
    __dsync();
    motor_off();
    IfxPort_setPinLow(PIN_MOTOR_SLP);                   /* motor driver asleep */
    servo_write(THROTTLE_TOM_CH, throttle_idle_us);     /* idle, from the next period (<= 20 ms) */
    servo_write(GEAR_TOM_CH, gear_neutral_us);          /* neutral */
    IfxPort_setPinLow(PIN_RELAY);                       /* relay open */
    IfxPort_setPinLow(PIN_ALARM_LED);                   /* LED on */
    IfxPort_setPinHigh(PIN_BUZZER);                     /* buzzer on */
}

/* E-stop contact: normally closed to GND, so high = pressed (or the wire is
 * broken - also "pressed"). Pressed counts at once; released only after 3
 * calls (~150 ms) in a row, so contact bounce can't "release" it. */
bool hal_read_estop(bool *pressed)
{
#if HAL_ESTOP_FITTED
    static uint8_t released_n = 0u;
    static bool state = false;                          /* pressed at power-up: seen at the first read */
    if (pin_high(PIN_ESTOP)) {
        state = true;
        released_n = 0u;
    } else if (state && ++released_n >= 3u) {
        state = false;
    }
    *pressed = state;
    return true;
#else
    (void)pressed;
    return false;
#endif
}

bool hal_outputs_forced(void)
{
    return s_forced != FALSE;
}

void hal_set_safety_relay(bool closed)
{
    if (closed && !s_forced) IfxPort_setPinHigh(PIN_RELAY);
    else IfxPort_setPinLow(PIN_RELAY);
}

/* ENGINE KILL relay: the pin is TBD (hal_pins.h). Until it is chosen nothing
 * is driven - the board can't kill the engine yet (the FIRST TEST build is
 * always CREWED, where the kill is never used). */
static volatile boolean s_engine_kill;

void hal_set_engine_kill(bool kill)
{
    s_engine_kill = kill ? TRUE : FALSE;
#ifdef PIN_ENGINE_KILL
#ifdef ENGINE_KILL_ACTIVE_HIGH
    if (kill) IfxPort_setPinHigh(PIN_ENGINE_KILL); else IfxPort_setPinLow(PIN_ENGINE_KILL);
#else
    if (kill) IfxPort_setPinLow(PIN_ENGINE_KILL); else IfxPort_setPinHigh(PIN_ENGINE_KILL);
#endif
#endif
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

void hal_watchdog_kick_core(void)
{
    IfxScuWdt_serviceCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());   /* the calling CPU's watchdog */
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

/* G2 current sense: V = offset + 20 mV/A. The newest ADC result, as long as
 * it is fresh (the link reads it too, so "new since the last call" would
 * starve one of the readers). */
bool hal_read_motor_current(float *amps)
{
    static float last_a = 0.0f;
    static uint32_t last_ms = 0u;
    static boolean have = FALSE;
    uint16_t raw;
    uint32_t now = hal_millis();
    if (adc_read(ADC_MOTOR_CS, &raw)) {
        float a = (adc_volts(raw) - MOTOR_CS_OFFSET_V) / MOTOR_CS_V_PER_A;
        last_a = a > 0.0f ? a : 0.0f;
        last_ms = now;
        have = TRUE;
    }
    if (!have || (uint32_t)(now - last_ms) > CURRENT_FRESH_MS) return false;
    *amps = last_a;
    return true;
}

/* Steering end stops: normally closed to GND, high = at the stop (or wire broken). */
bool hal_read_limit_switches(bool *left, bool *right)
{
#if HAL_LIMITS_FITTED
    *left = pin_high(PIN_LIMIT_LEFT) != FALSE;
    *right = pin_high(PIN_LIMIT_RIGHT) != FALSE;
    return true;
#else
    (void)left;
    (void)right;
    return false;
#endif
}

bool hal_read_roll_rate(float *roll_dps)
{
    (void)roll_dps;
    return false;    /* not used: the BNO085 gives the attitude (hal_read_attitude) */
}

uint32_t hal_read_accel_z(float *g, uint32_t max)
{
    imu_service();
    return bno_accel_z(g, max);
}

bool hal_read_attitude(float *roll_deg, float *pitch_deg)
{
    imu_service();
    return bno_attitude(roll_deg, pitch_deg);
}

void hal_set_throttle_pwm(uint16_t pulse_us)
{
    if (s_forced) pulse_us = s_safe_thr_us;     /* monitor tripped: idle, latched */
    if (pulse_us < HAL_THROTTLE_PWM_MIN) pulse_us = HAL_THROTTLE_PWM_MIN;
    if (pulse_us > HAL_THROTTLE_PWM_MAX) pulse_us = HAL_THROTTLE_PWM_MAX;
    servo_write(THROTTLE_TOM_CH, pulse_us);
    servo_route(THROTTLE_TOM_CH, &s_thr_routed);
    __dsync();
    if (s_forced) servo_write(THROTTLE_TOM_CH, s_safe_thr_us);   /* tripped meanwhile: idle again */
}

void hal_set_gear_pwm(uint16_t pulse_us)
{
    if (s_forced) pulse_us = s_safe_gear_us;    /* monitor tripped: neutral, latched */
    servo_write(GEAR_TOM_CH, pulse_us);
    servo_route(GEAR_TOM_CH, &s_gear_routed);
    __dsync();
    if (s_forced) servo_write(GEAR_TOM_CH, s_safe_gear_us);      /* tripped meanwhile: neutral again */
}

bool hal_read_neutral_switch(bool *in_neutral)
{
#if HAL_NEUTRAL_SW_FITTED
    *in_neutral = pin_high(PIN_NEUTRAL_SW) == FALSE;    /* closed to GND in neutral */
    return true;
#else
    (void)in_neutral;
    return false;
#endif
}

bool hal_read_gps(hal_gps_t *gps)
{
    return gps_poll(gps);
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
    return uart_read(UART_BRIDGE, buf, max);     /* ASCLIN3, RX P20.3, 230400 (WIRING.md section 6) */
}

/* --- non-volatile storage: tc_nv.c record log on DFLASH0 ---------------------
 * DFLASH0 (0xAF000000, 64 logical sectors of 4 KB, 8-byte pages, erased pages
 * read as 0 - IfxFlash.h "Verify Erased Page": erased = 0 data and ECC bits)
 * regions A = sectors 0-7, B = sectors 8-15 (32 KB each; the live records are
 * ~1.3 KB). The flash commands come from the iLLD Flash/Std inline sequences
 * (IfxFlash.h, Libraries un-excluded in .cproject); every command is ISSUED
 * here and polled by tc_nv.c through nv_busy() - no waitUnbusy loop, the
 * control loop and the CPU1 heartbeat are untouched. The DFLASH is read only
 * while no command runs (tc_nv.c keeps that rule: a read of a busy DFLASH
 * bank is not allowed on the TC3xx). DFLASH commands need no ENDINIT and no
 * write-protection unlock (UCB_DFLASH is not programmed). Core 0 only. */
#define NV_DFLASH_BASE   IFXFLASH_DFLASH_START
#define NV_SECTOR_BYTES  4096u
#define NV_REGION_BYTES  (8u * NV_SECTOR_BYTES)
#define NV_BUDGET_US     300u             /* page programs per control step: within this */
#define NV_ERR_MASK      0x7Fu            /* DMU_HF_ERRSR: OPER SQER PROER PVER EVER ADER ORIER */
#define NV_PAGEMODE_SPIN 10000u           /* "enter page mode" is immediate; bounded anyway */

static uint32 nv_addr(uint8_t region, uint32_t off)
{
    return (uint32)(NV_DFLASH_BASE + (uint32)region * NV_REGION_BYTES + off);
}

static bool nv_port_read(uint8_t region, uint32_t off, void *buf, uint32_t len)
{
    if (region > 1u || off + len > NV_REGION_BYTES) return false;
    memcpy(buf, (const void *)nv_addr(region, off), len);
    return true;
}

static bool nv_port_busy(void)
{
    return DMU_HF_STATUS.B.D0BUSY != 0u;
}

static bool nv_port_failed(void)
{
    return (DMU_HF_ERRSR.U & NV_ERR_MASK) != 0u;
}

static bool nv_port_erase_start(uint8_t region, uint32_t sector)
{
    if (region > 1u || sector * NV_SECTOR_BYTES >= NV_REGION_BYTES || nv_port_busy()) return false;
    IfxFlash_clearStatus(0u);
    IfxFlash_eraseSector(nv_addr(region, sector * NV_SECTOR_BYTES));
    return true;
}

static bool nv_port_write_start(uint8_t region, uint32_t off, uint32_t w0, uint32_t w1)
{
    if (region > 1u || (off & 7u) != 0u || off + 8u > NV_REGION_BYTES || nv_port_busy()) return false;
    uint32 page = nv_addr(region, off);
    IfxFlash_clearStatus(0u);
    if (IfxFlash_enterPageMode(page) != 0u) return false;
    for (uint32_t i = 0; nv_port_busy(); i++) {          /* page mode is entered at once */
        if (i >= NV_PAGEMODE_SPIN) return false;
    }
    if (nv_port_failed()) return false;
    IfxFlash_loadPage2X32(page, w0, w1);
    IfxFlash_writePage(page);
    return true;
}

static const nvlog_port_t s_nv_port = {
    NV_REGION_BYTES, NV_SECTOR_BYTES, 0u /* erased word */, 1000000u /* command time limit, us */,
    nv_port_read, nv_port_erase_start, nv_port_write_start, nv_port_busy, nv_port_failed, hal_micros
};

static void nv_port_init(void)
{
    nvlog_init(&s_nv_port);
}

bool hal_nv_save(uint8_t slot, const void *data, uint32_t size)
{
    return nvlog_save(slot, data, size);
}

bool hal_nv_load(uint8_t slot, void *data, uint32_t size)
{
    return nvlog_load(slot, data, size);
}

void hal_nv_service(bool allowed)
{
    nvlog_service(allowed, NV_BUDGET_US);
}

bool hal_nv_pending(void)
{
    return nvlog_pending();
}

bool hal_nv_ok(void)
{
    return nvlog_ok();
}

bool hal_read_water_speed(float *kn)
{
    (void)kn;
    return false;    /* TODO: paddlewheel pulse input (GPT12 / GTM TIM), if fitted */
}

/* Panel buttons, sampled once per control step (50 ms): a level that is the
 * same in two samples in a row is the debounced state; a press = its edge. */
uint32_t hal_get_button_presses(void)
{
    static uint32_t prev_raw = 0u, stable = 0u;
    uint32_t raw = 0u;
    if (!pin_high(PIN_BTN_STANDBY))  raw |= HAL_BTN_BIT(HAL_BTN_STANDBY);
    if (!pin_high(PIN_BTN_HDG_LOCK)) raw |= HAL_BTN_BIT(HAL_BTN_HDG_LOCK);
    if (!pin_high(PIN_BTN_MINUS_1))  raw |= HAL_BTN_BIT(HAL_BTN_MINUS_1);
    if (!pin_high(PIN_BTN_PLUS_1))   raw |= HAL_BTN_BIT(HAL_BTN_PLUS_1);
    if (!pin_high(PIN_BTN_MINUS_10)) raw |= HAL_BTN_BIT(HAL_BTN_MINUS_10);
    if (!pin_high(PIN_BTN_PLUS_10))  raw |= HAL_BTN_BIT(HAL_BTN_PLUS_10);
    uint32_t agree = ~(raw ^ prev_raw);                 /* bits equal in both samples */
    uint32_t next = (stable & ~agree) | (raw & agree);
    uint32_t pressed = next & ~stable;
    stable = next;
    prev_raw = raw;
    return pressed;
}

void hal_set_alarm(bool on)
{
    if (s_forced) on = true;                    /* monitor tripped: alarm stays on */
    if (on) {
        IfxPort_setPinLow(PIN_ALARM_LED);       /* LED on  */
        IfxPort_setPinHigh(PIN_BUZZER);
    } else {
        IfxPort_setPinHigh(PIN_ALARM_LED);      /* LED off */
        IfxPort_setPinLow(PIN_BUZZER);
    }
}

/* BENCH TEST screen: what the drivers know (read from CPU0; the SD numbers
 * are written by CPU2 - plain words, a torn value only shows wrong once). */
bool hal_get_diag(hal_diag_t *d)
{
    const bno_stat_t *b = bno_stat();
    const gps_stat_t *g = gps_stat();
    const sd_stat_t *c = sd_stat();
    d->imu_ok = b->ok;
    d->imu_accuracy = b->heading_accuracy;
    d->imu_reports = b->reports;
    d->imu_resets = b->resets;
    d->accel_z_g = b->accel_z_g;
    d->gps_ok = g->ok;
    d->gps_ubx = g->ubx;
    d->gps_fix_type = g->fix_type;
    d->gps_spoof = g->spoof_state;
    d->gps_jam = g->jam_state;
    d->sd_card = c->card;
    d->sd_mounted = c->mounted;
    d->sd_state = c->state;
    d->sd_free_kb = c->mounted ? (uint32_t)(((uint64_t)c->free_clusters * c->cluster_bytes) / 1024u) : 0xFFFFFFFFu;
    d->uart_errors = 0u;
    for (unsigned i = 0; i < (unsigned)UART_COUNT; i++) d->uart_errors += uart_stat((uart_id_t)i)->errors;
    return true;
}

/* Ground station: ASCLIN0 = USB virtual COM of the on-board debugger, 921600. */
uint32_t hal_link_read(uint8_t *buf, uint32_t max)
{
    return uart_read(UART_LINK, buf, max);
}

bool hal_link_write(const uint8_t *buf, uint32_t len)
{
    return uart_write(UART_LINK, buf, len);
}

/* Gateway (Raspberry Pi 5): ASCLIN1, TX P15.0 / RX P15.1, 921600, TX FIFO = HAL_GW_TX_RING. */
uint32_t hal_gw_read(uint8_t *buf, uint32_t max)
{
    return uart_read(UART_GW, buf, max);
}

bool hal_gw_write(const uint8_t *buf, uint32_t len)
{
    return uart_write(UART_GW, buf, len);
}

uint32_t hal_gw_tx_free(void)
{
    return uart_tx_free(UART_GW);
}
