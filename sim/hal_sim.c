/*
 * hal_sim.c - hal.h implemented on top of the PC boat simulation.
 */
#include "hal_sim.h"
#include "boat_model.h"

#include <stdio.h>
#include <string.h>

#define COMPASS_PERIOD_MS   100u    /* 10 Hz */
#define RUDDER_PERIOD_MS     50u    /* 20 Hz */
#define GYRO_PERIOD_MS       50u    /* 20 Hz */
#define COMPASS_NOISE_DEG   0.2f
#define RUDDER_NOISE_DEG    0.1f
#define CALIB_FILE          "calib.bin"

/* safety monitor trip latch (hal_safe_outputs) */
static bool     s_forced;
static uint16_t s_safe_thr_us = 1000u, s_safe_gear_us = 1500u;

static uint32_t s_time_ms;
static float    s_true_heading;
static float    s_true_rudder;
static float    s_true_yaw;
static bool     s_heading_failed;
static bool     s_rudder_feedback = true;  /* false = no angle sensor fitted */
static bool     s_gyro_fitted = false;
static float    s_gyro_bias;
static float    s_gyro_noise;

static uint32_t s_next_compass_ms;
static uint32_t s_next_rudder_ms;
static uint32_t s_next_gyro_ms;
static uint32_t s_buttons;
static float    s_motor;
static bool     s_alarm;
static uint32_t s_rng = 12345u;
static uint32_t s_gyro_rng = 777u;

/* ---- hal.h ------------------------------------------------------------- */

void hal_init(void)
{
    s_next_compass_ms = s_time_ms;
    s_next_rudder_ms = s_time_ms;
    s_next_gyro_ms = s_time_ms;
    s_buttons = 0;
    s_motor = 0.0f;
    s_alarm = false;
    s_rng = 12345u;         /* same sensor noise every run */
    s_gyro_rng = 777u;
}

uint32_t hal_millis(void)
{
    return s_time_ms;
}

bool hal_read_heading(float *heading_deg)
{
    if (s_heading_failed || (int32_t)(s_time_ms - s_next_compass_ms) < 0) {
        return false;
    }
    s_next_compass_ms += COMPASS_PERIOD_MS;

    float h = s_true_heading + COMPASS_NOISE_DEG * boat_rand_gauss(&s_rng);
    if (h < 0.0f) h += 360.0f;
    if (h >= 360.0f) h -= 360.0f;
    *heading_deg = h;
    return true;
}

bool hal_read_rudder_angle(float *rudder_deg)
{
    if (!s_rudder_feedback || (int32_t)(s_time_ms - s_next_rudder_ms) < 0) {
        return false;
    }
    s_next_rudder_ms += RUDDER_PERIOD_MS;
    *rudder_deg = s_true_rudder + RUDDER_NOISE_DEG * boat_rand_gauss(&s_rng);
    return true;
}

bool hal_read_yaw_rate(float *yaw_dps)
{
    if (!s_gyro_fitted || (int32_t)(s_time_ms - s_next_gyro_ms) < 0) {
        return false;
    }
    s_next_gyro_ms += GYRO_PERIOD_MS;
    /* separate random stream, so fitting a gyro doesn't change the compass noise */
    *yaw_dps = s_true_yaw + s_gyro_bias + s_gyro_noise * boat_rand_gauss(&s_gyro_rng);
    return true;
}

void hal_set_rudder_motor(float command)
{
    if (command > 1.0f) command = 1.0f;
    if (command < -1.0f) command = -1.0f;
    s_motor = s_forced ? 0.0f : command;
}

uint32_t hal_get_button_presses(void)
{
    uint32_t b = s_buttons;
    s_buttons = 0;
    return b;
}

void hal_set_alarm(bool on)
{
    s_alarm = on || s_forced;
}

bool hal_calib_save(const void *data, uint32_t size)
{
    FILE *f = fopen(CALIB_FILE, "wb");
    if (!f) return false;
    bool ok = fwrite(data, 1, size, f) == size;
    return (fclose(f) == 0) && ok;
}

bool hal_calib_load(void *data, uint32_t size)
{
    FILE *f = fopen(CALIB_FILE, "rb");
    if (!f) return false;
    bool ok = fread(data, 1, size, f) == size;
    fclose(f);
    return ok;
}

/* ---- simulation control ------------------------------------------------ */

void sim_hal_set_time_ms(uint32_t t_ms)  { s_time_ms = t_ms; }

void sim_hal_set_truth(float heading_deg, float rudder_deg)
{
    s_true_heading = heading_deg;
    s_true_rudder = rudder_deg;
}

void sim_hal_set_yaw_truth(float yaw_dps) { s_true_yaw = yaw_dps; }

void sim_hal_set_gyro(bool fitted, float bias_dps, float noise_dps)
{
    s_gyro_fitted = fitted;
    s_gyro_bias = bias_dps;
    s_gyro_noise = noise_dps;
}

void sim_hal_fail_heading(bool failed)   { s_heading_failed = failed; }
void sim_hal_press(hal_button_t button)  { s_buttons |= HAL_BTN_BIT(button); }
float sim_hal_motor_command(void)        { return s_motor; }
bool  sim_hal_alarm(void)                { return s_alarm; }
void sim_hal_set_rudder_feedback(bool fitted) { s_rudder_feedback = fitted; }

/* ---- small boat: engine and GPS ---------------------------------------- */

static uint16_t  s_throttle_us = 1000u;
static uint16_t  s_gear_pwm = 1500u;
static hal_gps_t s_gps;
static bool      s_gps_new;

void hal_set_throttle_pwm(uint16_t pulse_us)
{
    if (s_forced) pulse_us = s_safe_thr_us;     /* monitor tripped: latched */
    if (pulse_us < HAL_THROTTLE_PWM_MIN) pulse_us = HAL_THROTTLE_PWM_MIN;
    if (pulse_us > HAL_THROTTLE_PWM_MAX) pulse_us = HAL_THROTTLE_PWM_MAX;
    s_throttle_us = pulse_us;
}

void hal_set_gear_pwm(uint16_t pulse_us) { s_gear_pwm = s_forced ? s_safe_gear_us : pulse_us; }

bool hal_read_gps(hal_gps_t *gps)
{
    if (!s_gps_new) return false;
    *gps = s_gps;
    s_gps_new = false;
    return true;
}

/* The simulated throttle servo: 1000 us = idle, 2000 us = full (the
 * default calibration), beyond that the servo is at its mechanical end. */
float sim_hal_throttle(void)
{
    float pct = ((float)s_throttle_us - 1000.0f) * 0.1f;
    return pct < 0.0f ? 0.0f : (pct > 100.0f ? 100.0f : pct);
}
uint16_t sim_hal_throttle_us(void) { return s_throttle_us; }
uint16_t sim_hal_gear_pwm(void)  { return s_gear_pwm; }
void     sim_hal_gps_message(const hal_gps_t *g) { s_gps = *g; s_gps_new = true; }
void     sim_hal_reset_engine(void)
{
    s_throttle_us = 1000u;
    s_gear_pwm = 1500u;
    s_gps_new = false;
}

static bool  s_roll_fitted = false;
static float s_true_roll_rate;

bool hal_read_roll_rate(float *roll_dps)
{
    if (!s_roll_fitted) return false;
    *roll_dps = s_true_roll_rate;
    return true;
}

void sim_hal_set_roll(bool fitted, float roll_dps)
{
    s_roll_fitted = fitted;
    s_true_roll_rate = roll_dps;
}

/* BNO085 vertical acceleration: the boat model pushes one sample per
 * physics step (100 Hz); the controller takes them in batches. */
#define ACC_RING 256u
static float    s_acc[ACC_RING];
static uint32_t s_acc_head, s_acc_tail;
static bool     s_acc_fitted = true;

uint32_t hal_read_accel_z(float *g, uint32_t max)
{
    uint32_t n = 0u;
    while (s_acc_tail != s_acc_head && n < max) {
        g[n++] = s_acc[s_acc_tail % ACC_RING];
        s_acc_tail++;
    }
    return n;
}

void sim_hal_push_accel(float g)
{
    if (!s_acc_fitted) return;
    s_acc[s_acc_head % ACC_RING] = g;
    s_acc_head++;
    if (s_acc_head - s_acc_tail > ACC_RING) s_acc_tail = s_acc_head - ACC_RING;
}

/* BNO085 attitude: the boat model's wave roll / pitch */
static bool  s_att_fitted = true, s_att_new;
static float s_roll_deg, s_pitch_deg;

bool hal_read_attitude(float *roll_deg, float *pitch_deg)
{
    if (!s_att_fitted || !s_att_new) return false;
    *roll_deg = s_roll_deg;
    *pitch_deg = s_pitch_deg;
    s_att_new = false;
    return true;
}

void sim_hal_push_attitude(float roll_deg, float pitch_deg)
{
    s_roll_deg = roll_deg;
    s_pitch_deg = pitch_deg;
    s_att_new = true;
}

void sim_hal_set_attitude(bool fitted)
{
    s_att_fitted = fitted;
}

void sim_hal_set_accel(bool fitted)
{
    s_acc_fitted = fitted;
    s_acc_tail = s_acc_head;
}

static bool s_neutral_fitted = false;
static bool s_neutral_state;

bool hal_read_neutral_switch(bool *in_neutral)
{
    if (!s_neutral_fitted) return false;
    *in_neutral = s_neutral_state;
    return true;
}

void sim_hal_set_neutral_switch(bool fitted, bool in_neutral)
{
    s_neutral_fitted = fitted;
    s_neutral_state = in_neutral;
}

static bool  s_current_fitted = false;
static float s_current_a;
static bool  s_limits_fitted = false;
static bool  s_limit_left, s_limit_right;

bool hal_read_motor_current(float *amps)
{
    if (!s_current_fitted) return false;
    *amps = s_current_a;
    return true;
}

bool hal_read_limit_switches(bool *left, bool *right)
{
    if (!s_limits_fitted) return false;
    *left = s_limit_left;
    *right = s_limit_right;
    return true;
}

void sim_hal_set_motor_current(bool fitted, float amps)
{
    s_current_fitted = fitted;
    s_current_a = amps;
}

void sim_hal_set_limit_switches(bool fitted, bool left, bool right)
{
    s_limits_fitted = fitted;
    s_limit_left = left;
    s_limit_right = right;
}

static bool  s_water_fitted = false;
static float s_water_kn;

bool hal_read_water_speed(float *kn)
{
    if (!s_water_fitted) return false;
    *kn = s_water_kn;
    return true;
}

void sim_hal_set_water_speed(bool fitted, float kn)
{
    s_water_fitted = fitted;
    s_water_kn = kn;
}

static bool           s_joy_fitted = false;
static hal_joystick_t s_joy;

/* --- USB joystick bridge (simulated Teensy + hub) ---
 * Bridge mode: the analog joystick state (sim_hal_set_joystick) is sent as
 * USB bridge frames instead (stick X/Y -> device 0 axes 0/1, TAKE -> stick
 * button 1, AUTO -> throttle button 1, CONFIRM -> throttle button 4), or a
 * state set directly with sim_hal_bridge_set. Timing model:
 *   joystick change -> USB poll (1 ms) + hub (1 ms) -> bridge samples at its
 *   10 ms frame tick -> UART transfer 57 bytes at 230400 baud (2.5 ms, 3 ms
 *   here) -> hal_bridge_read. */
#define BR_HIST 16
static bool     s_br_on, s_br_direct;
static struct { uint32_t t_ms; jsf_t f; } s_br_hist[BR_HIST];   /* state changes, newest last */
static int      s_br_n;
static uint32_t s_br_next_ms;           /* next frame tick */
static uint8_t  s_br_seq;
static uint32_t s_br_fault;
static uint32_t s_br_usb_ms = 2u, s_br_xfer_ms = 3u, s_br_period_ms = JSF_PERIOD_MS;
static uint8_t  s_br_out[2048];
static uint32_t s_br_head, s_br_tail;
static uint32_t s_br_frames;

static int16_t br_axis(uint16_t raw)
{
    if (raw < 100u || raw > 3995u) return JSF_AXIS_NONE;        /* wire break -> invalid value */
    float v = ((float)raw - 2048.0f) / (raw >= 2048u ? (3754.0f - 2048.0f) : (2048.0f - 341.0f));
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (int16_t)(v * 32767.0f);
}

static void br_record(const jsf_t *f)
{
    if (s_br_n == BR_HIST) {
        memmove(&s_br_hist[0], &s_br_hist[1], sizeof s_br_hist[0] * (BR_HIST - 1));
        s_br_n--;
    }
    s_br_hist[s_br_n].t_ms = s_time_ms;
    s_br_hist[s_br_n].f = *f;
    s_br_n++;
}

static void br_from_analog(void)
{
    if (!s_br_on || s_br_direct) return;
    jsf_t f;
    memset(&f, 0, sizeof f);
    jsf_dev_clear(&f.dev[0]);
    jsf_dev_clear(&f.dev[1]);
    if (s_joy_fitted) {
        f.flags = JSF_F_DEV0 | JSF_F_DEV1 | JSF_F_HUB;
        f.dev[0].axis[0] = br_axis(s_joy.x_raw);
        f.dev[0].axis[1] = br_axis(s_joy.y_raw);
        f.dev[0].buttons = s_joy.take ? 1u : 0u;
        f.dev[1].axis[2] = 0;
        f.dev[1].buttons = (s_joy.autob ? 1u : 0u) | (s_joy.confirm ? 8u : 0u);
        f.dev[0].vid = f.dev[1].vid = 0x044Fu;
        f.dev[0].pid = 0xB10Au;
        f.dev[1].pid = 0xB687u;
    }
    br_record(&f);
}

void sim_hal_bridge_mode(bool on)
{
    s_br_on = on;
    s_br_direct = false;
    s_br_n = 0;
    s_br_fault = 0u;
    s_br_head = s_br_tail = 0u;
    s_br_next_ms = s_time_ms;
    s_br_frames = 0u;
    br_from_analog();
}

bool sim_hal_bridge_on(void) { return s_br_on; }

void sim_hal_bridge_set(const jsf_t *f)
{
    s_br_direct = true;
    br_record(f);
}

void sim_hal_bridge_analog(void)
{
    s_br_direct = false;
    br_from_analog();
}

void sim_hal_bridge_fault(uint32_t bits) { s_br_fault = bits; }

void sim_hal_bridge_timing(uint32_t usb_ms, uint32_t xfer_ms, uint32_t phase_ms)
{
    s_br_usb_ms = usb_ms;
    s_br_xfer_ms = xfer_ms;
    s_br_next_ms = s_time_ms + phase_ms;
}

uint32_t sim_hal_bridge_frames(void) { return s_br_frames; }

static const jsf_t *br_state_at(uint32_t t_ms)
{
    static jsf_t none;
    for (int i = s_br_n - 1; i >= 0; i--)
        if ((int32_t)(t_ms - s_br_hist[i].t_ms) >= 0) return &s_br_hist[i].f;
    if (s_br_n > 0) return &s_br_hist[0].f;
    memset(&none, 0, sizeof none);
    jsf_dev_clear(&none.dev[0]);
    jsf_dev_clear(&none.dev[1]);
    return &none;
}

uint32_t hal_bridge_read(uint8_t *buf, uint32_t max)
{
    if (!s_br_on) return 0u;
    /* frames whose last byte has arrived by now */
    while ((int32_t)(s_time_ms - (s_br_next_ms + s_br_xfer_ms)) >= 0) {
        uint32_t tick = s_br_next_ms;
        s_br_next_ms += s_br_period_ms;
        jsf_t f = *br_state_at(tick - s_br_usb_ms);     /* what the bridge had read over USB */
        if (!(s_br_fault & SIM_BR_FREEZE)) s_br_seq++;
        f.seq = s_br_seq;
        if (s_br_fault & SIM_BR_STOP) continue;
        uint8_t b[JSF_FRAME_LEN];
        size_t n = jsf_encode(&f, b);
        if ((s_br_fault & SIM_BR_CORRUPT) || ((s_br_fault & SIM_BR_CORRUPT_SOME) && s_br_seq % 50u == 0u)) b[10] ^= 0x40u;
        for (size_t i = 0; i < n; i++) {
            if (s_br_head - s_br_tail >= sizeof s_br_out) s_br_tail++;
            s_br_out[s_br_head++ % sizeof s_br_out] = b[i];
        }
        s_br_frames++;
    }
    uint32_t k = 0u;
    while (k < max && s_br_tail != s_br_head) buf[k++] = s_br_out[s_br_tail++ % sizeof s_br_out];
    return k;
}

bool hal_read_joystick(hal_joystick_t *j)
{
    if (!s_joy_fitted || s_br_on) return false;     /* bridge mode: no analog joystick */
    *j = s_joy;
    return true;
}

void sim_hal_set_joystick(bool fitted, uint16_t x_raw, uint16_t y_raw, bool take, bool autob)
{
    s_joy_fitted = fitted;
    s_joy.x_raw = x_raw;
    s_joy.y_raw = y_raw;
    s_joy.take = take;
    s_joy.autob = autob;
    br_from_analog();
}

void sim_hal_joystick_confirm(bool pressed)
{
    s_joy.confirm = pressed;
    br_from_analog();
}
bool hal_nv_save(uint8_t slot, const void *data, uint32_t size)
{
    char name[32];
    snprintf(name, sizeof name, "nv%u.bin", (unsigned)slot);
    FILE *f = fopen(name, "wb");
    if (!f) return false;
    bool ok = fwrite(data, 1, size, f) == size;
    return (fclose(f) == 0) && ok;
}

bool hal_nv_load(uint8_t slot, void *data, uint32_t size)
{
    char name[32];
    snprintf(name, sizeof name, "nv%u.bin", (unsigned)slot);
    FILE *f = fopen(name, "rb");
    if (!f) return false;
    bool ok = fread(data, 1, size, f) == size;
    fclose(f);
    return ok;
}

/* --- ground-station link: two byte rings, filled / emptied by the sim
 *     TCP server (sim_server.c) or directly by the tests --- */
#define LINK_BUF 8192u
typedef struct { uint8_t b[LINK_BUF]; uint32_t head, tail; } link_ring_t;
static link_ring_t s_link_rx, s_link_tx;   /* rx: PC -> boat, tx: boat -> PC */

static uint32_t ring_used(const link_ring_t *r) { return r->head - r->tail; }

uint32_t hal_link_read(uint8_t *buf, uint32_t max)
{
    uint32_t n = 0u;
    while (n < max && ring_used(&s_link_rx) > 0u) buf[n++] = s_link_rx.b[s_link_rx.tail++ % LINK_BUF];
    return n;
}

bool hal_link_write(const uint8_t *buf, uint32_t len)
{
    if (LINK_BUF - ring_used(&s_link_tx) < len) return false;
    for (uint32_t i = 0u; i < len; i++) s_link_tx.b[s_link_tx.head++ % LINK_BUF] = buf[i];
    return true;
}

uint32_t sim_hal_link_rx_push(const uint8_t *buf, uint32_t len)
{
    uint32_t n = 0u;
    while (n < len && ring_used(&s_link_rx) < LINK_BUF) s_link_rx.b[s_link_rx.head++ % LINK_BUF] = buf[n++];
    return n;
}

uint32_t sim_hal_link_tx_pop(uint8_t *buf, uint32_t max)
{
    uint32_t n = 0u;
    while (n < max && ring_used(&s_link_tx) > 0u) buf[n++] = s_link_tx.b[s_link_tx.tail++ % LINK_BUF];
    return n;
}

void sim_hal_link_reset(void)
{
    s_link_rx.head = s_link_rx.tail = 0u;
    s_link_tx.head = s_link_tx.tail = 0u;
}

/* --- gateway UART (Raspberry Pi): two rings like the PC link, plus a
 *     baud-rate model: the boat -> Pi side only moves as many bytes as the
 *     UART could have sent since the last pop (921600 baud = 92 bytes/ms) --- */
static link_ring_t s_gw_rx, s_gw_tx;
static uint32_t s_gw_bytes_per_s = 92160u;
static uint32_t s_gw_last_ms;
static uint64_t s_gw_credit_mb;            /* milli-bytes the UART may still send */

uint32_t hal_gw_read(uint8_t *buf, uint32_t max)
{
    uint32_t n = 0u;
    while (n < max && ring_used(&s_gw_rx) > 0u) buf[n++] = s_gw_rx.b[s_gw_rx.tail++ % LINK_BUF];
    return n;
}

bool hal_gw_write(const uint8_t *buf, uint32_t len)
{
    if (LINK_BUF - ring_used(&s_gw_tx) < len) return false;
    for (uint32_t i = 0u; i < len; i++) s_gw_tx.b[s_gw_tx.head++ % LINK_BUF] = buf[i];
    return true;
}

uint32_t hal_gw_tx_free(void)
{
    return LINK_BUF - ring_used(&s_gw_tx);
}

uint32_t sim_hal_gw_rx_push(const uint8_t *buf, uint32_t len)
{
    uint32_t n = 0u;
    while (n < len && ring_used(&s_gw_rx) < LINK_BUF) s_gw_rx.b[s_gw_rx.head++ % LINK_BUF] = buf[n++];
    return n;
}

uint32_t sim_hal_gw_tx_pop(uint8_t *buf, uint32_t max)
{
    uint32_t dt = s_time_ms - s_gw_last_ms;
    s_gw_last_ms = s_time_ms;
    s_gw_credit_mb += (uint64_t)dt * s_gw_bytes_per_s;
    if (s_gw_credit_mb > (uint64_t)LINK_BUF * 1000u) s_gw_credit_mb = (uint64_t)LINK_BUF * 1000u;
    uint32_t n = 0u;
    while (n < max && ring_used(&s_gw_tx) > 0u && s_gw_credit_mb >= 1000u) {
        buf[n++] = s_gw_tx.b[s_gw_tx.tail++ % LINK_BUF];
        s_gw_credit_mb -= 1000u;
    }
    return n;
}

void sim_hal_gw_reset(uint32_t baud)
{
    s_gw_rx.head = s_gw_rx.tail = 0u;
    s_gw_tx.head = s_gw_tx.tail = 0u;
    s_gw_bytes_per_s = baud / 10u;
    s_gw_last_ms = s_time_ms;
    s_gw_credit_mb = 0u;
}

/* --- hardening: monitor latch, safety relay, watchdog, reset reason --- */
static bool     s_relay;
static hal_reset_t s_reset_reason = HAL_RESET_POWER_ON;

void hal_safe_outputs(uint16_t throttle_idle_us, uint16_t gear_neutral_us)
{
    s_forced = true;
    s_safe_thr_us = throttle_idle_us;
    s_safe_gear_us = gear_neutral_us;
    s_throttle_us = throttle_idle_us;
    s_gear_pwm = gear_neutral_us;
    s_motor = 0.0f;
    s_alarm = true;
    s_relay = false;
}

bool hal_outputs_forced(void) { return s_forced; }
static bool s_estop_fitted, s_estop_pressed;
void sim_hal_set_estop(bool fitted, bool pressed) { s_estop_fitted = fitted; s_estop_pressed = pressed; }
bool hal_read_estop(bool *pressed)
{
    if (!s_estop_fitted) return false;
    *pressed = s_estop_pressed;
    return true;
}
void hal_set_safety_relay(bool closed) { s_relay = closed && !s_forced; }
bool sim_hal_safety_relay(void) { return s_relay; }

/* The simulator's watchdog is modelled by the test harness (sim_sb.c). */
void hal_watchdog_init(uint32_t timeout_ms) { (void)timeout_ms; }
void hal_watchdog_kick(void) {}
static uint32_t s_core_kicks;
void hal_watchdog_kick_core(void) { s_core_kicks++; }
uint32_t sim_hal_core_kicks(void) { return s_core_kicks; }

hal_reset_t hal_reset_reason(uint32_t *raw)
{
    if (raw) *raw = (uint32_t)s_reset_reason;
    return s_reset_reason;
}

/* Simulated chip reset: the monitor latch goes, outputs stop (no pulses:
 * the servos hold where they are), the next start reports `why`. */
void sim_hal_chip_reset(hal_reset_t why)
{
    s_forced = false;
    s_relay = false;
    s_throttle_us = 0u;
    s_gear_pwm = 0u;
    s_motor = 0.0f;
    s_reset_reason = why;
}

void sim_hal_unforce(void)
{
    s_forced = false;
    s_reset_reason = HAL_RESET_POWER_ON;
}

/* --- microseconds: simulated time, or the real PC clock (timing report) --- */
static bool s_real_us;

#ifdef _WIN32
#include <windows.h>
static uint32_t real_us(void)
{
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (f.QuadPart == 0) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (uint32_t)(c.QuadPart * 1000000ULL / (unsigned long long)f.QuadPart);
}
#else
#include <time.h>
static uint32_t real_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000ULL + (unsigned long long)ts.tv_nsec / 1000ULL);
}
#endif

uint32_t hal_micros(void)
{
    return s_real_us ? real_us() : s_time_ms * 1000u;
}

void sim_hal_real_micros(bool on) { s_real_us = on; }

uint64_t sim_hal_real_ns(void)
{
#ifdef _WIN32
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (f.QuadPart == 0) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (uint64_t)((double)c.QuadPart * 1e9 / (double)f.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
#endif
}

/* --- memory barrier, supply voltage ----------------------------------------- */
void hal_mem_barrier(void) { __sync_synchronize(); }

bool hal_read_supply_voltage(float *volts)
{
    *volts = 12.6f;
    return true;
}

/* --- microSD card: files in the folder "sdcard" ------------------------------
 * Written data is kept in RAM until hal_sd_sync / hal_sd_close, like the
 * card's cache: sim_hal_sd_power_cut() throws away what was not synced.
 * Faults: card missing, card full (free space), every write failing. */
#define SD_PENDING_MAX (256u * 1024u)
typedef struct { FILE *f; uint8_t pend[SD_PENDING_MAX]; uint32_t n; bool open; } sd_file_t;
static sd_file_t s_sd[HAL_SD_FILES];
static bool     s_sd_present = true, s_sd_fail;
static uint32_t s_sd_free_kb = 4000000u;     /* 4 GB */
static char     s_sd_dir[260] = "sdcard";

static bool sd_flush(sd_file_t *f)
{
    if (!f->open) return false;
    if (f->n) {
        if (fwrite(f->pend, 1, f->n, f->f) != f->n) return false;
        s_sd_free_kb -= (f->n / 1024u < s_sd_free_kb) ? f->n / 1024u : s_sd_free_kb;
        f->n = 0u;
    }
    return fflush(f->f) == 0;
}

bool hal_sd_present(void) { return s_sd_present; }

static uint32_t s_sd_ops;
uint32_t sim_hal_sd_ops(void) { return s_sd_ops; }

int hal_sd_open(const char *name)
{
    s_sd_ops++;
    char path[400];
    if (!s_sd_present || s_sd_fail || s_sd_free_kb < 64u) return -1;
    for (int h = 0; h < HAL_SD_FILES; h++) {
        if (s_sd[h].open) continue;
        snprintf(path, sizeof path, "%s/%s", s_sd_dir, name);
        s_sd[h].f = fopen(path, "wb");
        if (!s_sd[h].f) return -1;
        s_sd[h].n = 0u;
        s_sd[h].open = true;
        return h;
    }
    return -1;
}

bool hal_sd_write(int h, const void *data, uint32_t len)
{
    s_sd_ops++;
    if (h < 0 || h >= HAL_SD_FILES || !s_sd[h].open || !s_sd_present || s_sd_fail) return false;
    if (s_sd_free_kb * 1024ull < (uint64_t)s_sd[h].n + len + 4096u) return false;     /* full */
    if (s_sd[h].n + len > SD_PENDING_MAX && !sd_flush(&s_sd[h])) return false;
    memcpy(s_sd[h].pend + s_sd[h].n, data, len);
    s_sd[h].n += len;
    return true;
}

bool hal_sd_sync(int h)
{
    s_sd_ops++;
    if (h < 0 || h >= HAL_SD_FILES || !s_sd_present || s_sd_fail) return false;
    return sd_flush(&s_sd[h]);
}

bool hal_sd_close(int h)
{
    s_sd_ops++;
    if (h < 0 || h >= HAL_SD_FILES || !s_sd[h].open) return false;
    bool ok = s_sd_present && !s_sd_fail && sd_flush(&s_sd[h]);
    fclose(s_sd[h].f);
    s_sd[h].open = false;
    return ok;
}

uint32_t hal_sd_free_kb(void) { return s_sd_present ? s_sd_free_kb : 0u; }

void sim_hal_sd_setup(const char *dir, bool present, uint32_t free_kb, bool fail_writes)
{
    if (dir) snprintf(s_sd_dir, sizeof s_sd_dir, "%s", dir);
    s_sd_present = present;
    s_sd_free_kb = free_kb;
    s_sd_fail = fail_writes;
}

/* Power cut: unsynced data is gone, the files stay as they were on the card. */
void sim_hal_sd_power_cut(void)
{
    for (int h = 0; h < HAL_SD_FILES; h++) {
        if (!s_sd[h].open) continue;
        s_sd[h].n = 0u;
        fclose(s_sd[h].f);
        s_sd[h].open = false;
    }
}
