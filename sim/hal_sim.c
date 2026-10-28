/*
 * hal_sim.c - hal.h implemented on top of the PC boat simulation.
 */
#include "hal_sim.h"
#include "boat_model.h"
#include "tc_modbus.h"
#include "dronecan.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define COMPASS_PERIOD_MS   100u    /* 10 Hz */
#define RUDDER_PERIOD_MS     50u    /* 20 Hz */
#define GYRO_PERIOD_MS       50u    /* 20 Hz */
#define COMPASS_NOISE_DEG   0.2f
#define RUDDER_NOISE_DEG    0.1f

static void nv_sim_reset(void);         /* non-volatile storage queue (below) */
static void rb_driver_init(void);       /* Modbus relay board: the master restarts (below) */
static void sc_driver_init(void);       /* DroneCAN: the AURIX side restarts, the nodes keep running (below) */

/* safety monitor trip latch (hal_safe_outputs) */
static bool     s_forced;
static uint16_t s_safe_thr_us = 1000u, s_safe_gear_us = 1500u;
static float    s_thr_bow, s_thr_stern;      /* DP thrusters commanded, -100..100 (below) */

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
    nv_sim_reset();         /* a reboot: queued saves are gone, the files stay */
    rb_driver_init();       /* the Modbus master restarts; the relay board keeps its states */
    sc_driver_init();       /* DroneCAN: our controller + dronecan.c restart; the nodes keep running */
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
static bool      s_engine_kill;     /* the kill relay: the simulated engine stops (no thrust) */
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
    s_engine_kill = false;                  /* a new boat / the engine restarted on board */
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
/* --- non-volatile storage: files nv<slot>.bin, written like the board's data
 *     flash: a save only queues, the file is written by hal_nv_service(true)
 *     (STANDBY), one record per call; test faults: a power loss cuts the next
 *     write (the file keeps the old record, the queued save is lost), a stuck
 *     flash never finishes (NV FAULT after the time limit, like the board) --- */
#define NV_SIM_Q          4u
#define NV_SIM_TIMEOUT_MS 1000u
typedef struct { bool used; uint8_t slot; uint32_t len; uint8_t data[HAL_NV_MAX_SIZE]; } nv_sim_q_t;
static nv_sim_q_t s_nvq[NV_SIM_Q];
static uint32_t s_nv_fault, s_nv_writes, s_nv_write_ms, s_nv_lost, s_nv_busy_since;
static bool     s_nv_ok = true, s_nv_busy;

static void nv_sim_reset(void)
{
    memset(s_nvq, 0, sizeof s_nvq);
    s_nv_fault = 0u;
    s_nv_ok = true;
    s_nv_busy = false;
}

bool hal_nv_save(uint8_t slot, const void *data, uint32_t size)
{
    if (slot == 0u || slot > HAL_NV_MAX_SLOT || size == 0u || size > HAL_NV_MAX_SIZE || !s_nv_ok) return false;
    nv_sim_q_t *e = NULL;
    for (uint32_t i = 0; i < NV_SIM_Q; i++) if (s_nvq[i].used && s_nvq[i].slot == slot) { e = &s_nvq[i]; break; }
    if (!e) for (uint32_t i = 0; i < NV_SIM_Q; i++) if (!s_nvq[i].used) { e = &s_nvq[i]; break; }
    if (!e) return false;
    e->used = true;
    e->slot = slot;
    e->len = size;
    memcpy(e->data, data, size);
    return true;
}

bool hal_nv_load(uint8_t slot, void *data, uint32_t size)
{
    char name[32];
    if (slot == 0u || slot > HAL_NV_MAX_SLOT || size == 0u) return false;
    for (uint32_t i = 0; i < NV_SIM_Q; i++) {
        if (s_nvq[i].used && s_nvq[i].slot == slot) {     /* the queued save is the newest value */
            if (s_nvq[i].len != size) return false;
            memcpy(data, s_nvq[i].data, size);
            return true;
        }
    }
    snprintf(name, sizeof name, "nv%u.bin", (unsigned)slot);
    FILE *f = fopen(name, "rb");
    if (!f) return false;
    bool ok = fread(data, 1, size, f) == size && fgetc(f) == EOF;   /* exactly `size` bytes */
    fclose(f);
    return ok;
}

void hal_nv_service(bool allowed)
{
    if (!s_nv_ok) return;
    if (s_nv_busy) {                                    /* a "flash command" that never ends */
        if (s_time_ms - s_nv_busy_since > NV_SIM_TIMEOUT_MS) s_nv_ok = false;
        return;
    }
    if (!allowed) return;
    for (uint32_t i = 0; i < NV_SIM_Q; i++) {
        nv_sim_q_t *e = &s_nvq[i];
        if (!e->used) continue;
        if (s_nv_fault & SIM_NV_STUCK) { s_nv_busy = true; s_nv_busy_since = s_time_ms; return; }
        if (s_nv_fault & SIM_NV_POWER_LOSS) {           /* cut: the file stays as it was */
            s_nv_fault &= ~SIM_NV_POWER_LOSS;
            e->used = false;
            s_nv_lost++;
            return;
        }
        char name[32];
        snprintf(name, sizeof name, "nv%u.bin", (unsigned)e->slot);
        FILE *f = fopen(name, "wb");
        bool ok = f != NULL && fwrite(e->data, 1, e->len, f) == e->len;
        if (f) ok = (fclose(f) == 0) && ok;
        if (!ok) { s_nv_ok = false; return; }
        e->used = false;
        s_nv_writes++;
        s_nv_write_ms = s_time_ms;
        return;                                         /* one record per step */
    }
}

bool hal_nv_pending(void)
{
    if (s_nv_busy) return true;
    for (uint32_t i = 0; i < NV_SIM_Q; i++) if (s_nvq[i].used) return true;
    return false;
}

bool hal_nv_ok(void)
{
    return s_nv_ok;
}

void     sim_hal_nv_fault(uint32_t bits) { s_nv_fault = bits; }
uint32_t sim_hal_nv_writes(void)         { return s_nv_writes; }
uint32_t sim_hal_nv_write_ms(void)       { return s_nv_write_ms; }
uint32_t sim_hal_nv_lost(void)           { return s_nv_lost; }

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
    s_thr_bow = 0.0f;                   /* both thrusters stopped (latched) */
    s_thr_stern = 0.0f;
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


static uint32_t s_engine_kills; /* times the output went to "kill" */
void hal_set_engine_kill(bool kill)
{
    if (kill && !s_engine_kill) s_engine_kills++;
    s_engine_kill = kill;
}
bool sim_hal_engine_killed(void) { return s_engine_kill; }

/* the kill latch kept across a reset: sim_hal_chip_reset / hal_init never touch it
 * (a real chip's no-clear RAM); sb_init ignores it after a power-on */
static bool s_kill_keep;
void hal_kill_keep_set(bool killed) { s_kill_keep = killed; }
bool hal_kill_keep_get(void) { return s_kill_keep; }

/* hal_system_reset (remote restart after a trip): the harness sees the
 * request after the step and reboots the chip (HAL_RESET_APPLICATION). */
static uint32_t s_sw_resets;
static bool     s_sw_reset_req;
void hal_system_reset(void) { s_sw_reset_req = true; s_sw_resets++; }
bool sim_hal_reset_requested(void)
{
    bool r = s_sw_reset_req;
    s_sw_reset_req = false;
    return r;
}
uint32_t sim_hal_sw_resets(void) { return s_sw_resets; }
uint32_t sim_hal_engine_kills(void) { return s_engine_kills; }

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
    s_thr_bow = 0.0f;                   /* no pulses: the thruster controllers stop */
    s_thr_stern = 0.0f;
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

/* Supply voltage of the 12 V bank: measured (12.6 V) unless a test says
 * otherwise (sim_hal_set_supply: not fitted / any value, also NaN). */
static bool  s_supply_fitted = true;
static float s_supply_v = 12.6f;

/* --- DP thrusters: the outputs (latched to 0 by the monitor trip), the e-stop
 *     contactor (cuts their power), the bank voltage sag per % thrust --- */
static uint32_t s_thr_writes;                    /* hal_set_thrusters calls */
static float    s_thr_sag_v_per_pct = 0.006f;    /* both at 100 % = 1.2 V below the bank's rest voltage */

static float thr_clamp(float v)
{
    if (!(v == v)) return 0.0f;                  /* NaN = stop */
    return v > 100.0f ? 100.0f : (v < -100.0f ? -100.0f : v);
}

void hal_set_thrusters(float bow_pct, float stern_pct)
{
    s_thr_writes++;
    s_thr_bow = s_forced ? 0.0f : thr_clamp(bow_pct);
    s_thr_stern = s_forced ? 0.0f : thr_clamp(stern_pct);
}

void sim_hal_thrusters(float *bow_pct, float *stern_pct)
{
    *bow_pct = s_thr_bow;
    *stern_pct = s_thr_stern;
}

/* what really pushes the boat: nothing while the e-stop contactor is open */
void sim_hal_thrust_truth(float *bow_pct, float *stern_pct)
{
    bool cut = s_estop_fitted && s_estop_pressed;
    *bow_pct = cut ? 0.0f : s_thr_bow;
    *stern_pct = cut ? 0.0f : s_thr_stern;
}

uint32_t sim_hal_thruster_writes(void) { return s_thr_writes; }
void sim_hal_thruster_sag(float v_per_pct) { s_thr_sag_v_per_pct = v_per_pct; }

bool hal_read_supply_voltage(float *volts)
{
    if (!s_supply_fitted) return false;
    float b, st;
    sim_hal_thrust_truth(&b, &st);
    *volts = s_supply_v - s_thr_sag_v_per_pct * (fabsf(b) + fabsf(st));    /* the thrusters load the bank */
    return true;
}

void sim_hal_set_supply(bool fitted, float volts)
{
    s_supply_fitted = fitted;
    s_supply_v = volts;
}

/* --- RS485 Modbus RTU relay board: the real master (hal/tc_modbus.c) talks
 *     to a simulated SLAVE here, byte for byte: the request is parsed (CRC
 *     checked), the reply is ready after latency_ms, board-timed pulses are
 *     switched off by the board itself. The board keeps its coil states
 *     across an AURIX reboot (hal_init), like the real one on its own supply;
 *     sim_hal_relay_board() with present = false models a board that is gone
 *     (SILENT), faults corrupt the replies or make the board mute. --- */
#define RB_COILS 16u
static struct {
    bool     present;
    uint32_t latency_ms, fault;
    uint8_t  addr;
    uint32_t coils;                 /* true states, bit n-1 */
    uint32_t pulse_end[RB_COILS];   /* 0 = no pulse running */
    uint8_t  reply[32];
    uint8_t  reply_len;
    uint32_t reply_at_ms;
    bool     reply_pending;
    uint32_t requests, bad_requests, writes, pulses, polls, corrupted;
    uint32_t max_pulse_ms;          /* the longest "flash on" asked */
    uint32_t edges[RB_COILS];       /* off -> on per coil */
    bool     inited;
} s_rb;
static uint8_t s_rb_channels = RB_COILS;     /* hal_relay_channels(); 0 = no board fitted */
static uint8_t s_rb_channels_boot = RB_COILS; /* what the driver was started with (hal_init) */

static void rb_set(uint32_t i, bool on)
{
    if (on && !(s_rb.coils & (1u << i))) s_rb.edges[i]++;
    if (on) s_rb.coils |= 1u << i; else s_rb.coils &= ~(1u << i);
}

static void rb_tick(void)
{
    for (uint32_t i = 0; i < RB_COILS; i++) {
        if (s_rb.pulse_end[i] && (int32_t)(s_time_ms - s_rb.pulse_end[i]) >= 0) {
            s_rb.pulse_end[i] = 0u;
            if (!(s_rb.fault & SIM_RB_NO_PULSE_END)) s_rb.coils &= ~(1u << i);   /* (broken timer: stays on) */
        }
    }
}

static void rb_reply(const uint8_t *f, uint8_t n)
{
    uint16_t c = mb_crc16(f, n);
    memcpy(s_rb.reply, f, n);
    s_rb.reply[n] = (uint8_t)c;
    s_rb.reply[n + 1u] = (uint8_t)(c >> 8);
    s_rb.reply_len = (uint8_t)(n + 2u);
    if (s_rb.fault & SIM_RB_CORRUPT || ((s_rb.fault & SIM_RB_CORRUPT_SOME) && s_rb.requests % 7u == 3u)) {
        s_rb.reply[n] ^= 0x5Au;                                    /* a bad CRC */
        s_rb.corrupted++;
    }
    s_rb.reply_at_ms = s_time_ms + s_rb.latency_ms;
    s_rb.reply_pending = !(s_rb.fault & SIM_RB_SILENT);
}

static bool rb_port_write(const uint8_t *buf, uint32_t len)
{
    if (!s_rb.present || len < 4u || len > 32u) return s_rb.present;   /* a missing board: the bytes just go */
    rb_tick();
    s_rb.requests++;
    uint16_t c = (uint16_t)(buf[len - 2u] | ((uint16_t)buf[len - 1u] << 8));
    if (c != mb_crc16(buf, len - 2u) || buf[0] != s_rb.addr) { s_rb.bad_requests++; return true; }
    uint8_t f = buf[1];
    if (f == 0x01u && len == 8u) {                                 /* read coils */
        uint16_t start = (uint16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        uint16_t qty = (uint16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        if (start + qty > RB_COILS || qty == 0u) {
            uint8_t e[3] = { s_rb.addr, 0x81u, 0x02u };            /* illegal data address */
            rb_reply(e, 3u);
            return true;
        }
        uint8_t r[8] = { s_rb.addr, 0x01u, (uint8_t)((qty + 7u) / 8u) };
        uint32_t v = s_rb.coils >> start;
        for (uint8_t i = 0; i < r[2]; i++) r[3u + i] = (uint8_t)(v >> (8u * i));
        s_rb.polls++;
        rb_reply(r, (uint8_t)(3u + r[2]));
    } else if (f == 0x05u && len == 8u) {                          /* write / pulse one coil */
        uint16_t a = (uint16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        uint16_t v = (uint16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        if (a >= RB_COILS) {
            uint8_t e[3] = { s_rb.addr, 0x85u, 0x02u };
            rb_reply(e, 3u);
            return true;
        }
        if (v == 0xFF00u) { rb_set(a, true); s_rb.pulse_end[a] = 0u; s_rb.writes++; }
        else if (v == 0x0000u) { rb_set(a, false); s_rb.pulse_end[a] = 0u; s_rb.writes++; }
        else if ((v & 0xFF00u) == 0x0200u && (v & 0xFFu) != 0u) {  /* flash on: the board times it */
            rb_set(a, true);
            if (100u * (v & 0xFFu) > s_rb.max_pulse_ms) s_rb.max_pulse_ms = 100u * (v & 0xFFu);
            s_rb.pulse_end[a] = s_time_ms + 100u * (v & 0xFFu);
            if (s_rb.pulse_end[a] == 0u) s_rb.pulse_end[a] = 1u;
            s_rb.pulses++;
        } else {
            uint8_t e[3] = { s_rb.addr, 0x85u, 0x03u };            /* illegal data value */
            rb_reply(e, 3u);
            return true;
        }
        rb_reply(buf, 6u);                                         /* echo */
    } else if (f == 0x0Fu && len >= 10u) {                         /* write several coils */
        uint16_t start = (uint16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        uint16_t qty = (uint16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        uint8_t nb = buf[6];
        if (start + qty > RB_COILS || qty == 0u || nb != (qty + 7u) / 8u || len != 9u + nb) {
            uint8_t e[3] = { s_rb.addr, 0x8Fu, 0x02u };
            rb_reply(e, 3u);
            return true;
        }
        uint32_t v = 0u;
        for (uint8_t i = 0; i < nb && i < 4u; i++) v |= (uint32_t)buf[7u + i] << (8u * i);
        uint32_t mask = (qty >= 32u) ? 0xFFFFFFFFu : ((1u << qty) - 1u) << start;
        for (uint32_t i = 0; i < RB_COILS; i++) if (mask & (1u << i)) rb_set(i, ((v << start) >> i) & 1u);
        for (uint32_t i = 0; i < RB_COILS; i++) if (mask & (1u << i)) s_rb.pulse_end[i] = 0u;
        s_rb.writes++;
        rb_reply(buf, 6u);
    } else {
        uint8_t e[3] = { s_rb.addr, (uint8_t)(f | 0x80u), 0x01u };   /* illegal function */
        rb_reply(e, 3u);
    }
    return true;
}

static uint32_t rb_port_read(uint8_t *buf, uint32_t max)
{
    rb_tick();
    if (!s_rb.present || !s_rb.reply_pending || (int32_t)(s_time_ms - s_rb.reply_at_ms) < 0) return 0u;
    uint32_t n = s_rb.reply_len < max ? s_rb.reply_len : max;
    memcpy(buf, s_rb.reply, n);
    s_rb.reply_pending = false;
    return n;
}

static const mb_port_t s_rb_port = { rb_port_write, rb_port_read, hal_millis };

static void rb_driver_init(void)                 /* the AURIX side: at every boot */
{
    mb_config_t c = { 1u, s_rb_channels ? s_rb_channels : RB_COILS, 100u, HAL_RELAY_LOST_MS, 100u };
    s_rb_channels_boot = s_rb_channels;
    mb_init(&s_rb_port, &c);
    if (!s_rb.inited) {                          /* the board itself: once (it keeps its states across AURIX resets) */
        s_rb.inited = true;
        s_rb.present = true;
        s_rb.latency_ms = 5u;
        s_rb.addr = 1u;
    }
    s_rb.reply_pending = false;                  /* a reply in flight is lost with the reboot */
}

/* (no board fitted: like the TC375 with HAL_RELAY_FITTED 0 - the driver never runs) */
void hal_relay_service(void)      { if (s_rb_channels_boot) mb_service(); }
bool hal_relay_set(uint8_t n, bool on)        { return s_rb_channels_boot && mb_set_coil(n, on); }
bool hal_relay_pulse(uint8_t n, uint16_t ms)  { return s_rb_channels_boot && mb_pulse_coil(n, ms); }
bool hal_relay_read(uint8_t n, bool *on)      { return s_rb_channels_boot && mb_coil(n, on); }
bool hal_relay_ok(void)                        { return s_rb_channels_boot && mb_ok(); }
uint8_t hal_relay_channels(void)               { return s_rb_channels_boot; }
void sim_hal_relay_fitted(uint8_t channels)    { s_rb_channels = channels > RB_COILS ? RB_COILS : channels; }
uint32_t sim_hal_relay_max_pulse_ms(void)      { return s_rb.max_pulse_ms; }
uint32_t sim_hal_relay_edges(uint8_t n)        { return (n >= 1u && n <= RB_COILS) ? s_rb.edges[n - 1u] : 0u; }

void sim_hal_relay_board(bool present, uint32_t latency_ms, uint32_t fault)
{
    s_rb.inited = true;
    s_rb.present = present;
    s_rb.latency_ms = latency_ms;
    s_rb.fault = fault;
    s_rb.addr = 1u;
    s_rb.reply_pending = false;
}

void sim_hal_relay_board_reset(void)
{
    s_rb.coils = 0u;
    s_rb.max_pulse_ms = 0u;
    memset(s_rb.edges, 0, sizeof s_rb.edges);
    memset(s_rb.pulse_end, 0, sizeof s_rb.pulse_end);
    s_rb.reply_pending = false;
}

bool sim_hal_relay_state(uint8_t n)
{
    rb_tick();
    return n >= 1u && n <= RB_COILS && ((s_rb.coils >> (n - 1u)) & 1u);
}

void sim_hal_relay_force(uint8_t n, bool on)
{
    if (n < 1u || n > RB_COILS) return;
    rb_set(n - 1u, on);
    s_rb.pulse_end[n - 1u] = 0u;
}

uint32_t sim_hal_relay_requests(void) { return s_rb.requests; }
uint32_t sim_hal_relay_writes(void)   { return s_rb.writes + s_rb.pulses; }
uint32_t sim_hal_relay_pulses(void)   { return s_rb.pulses; }

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

/* BENCH TEST screen: the simulated drivers */
bool hal_get_diag(hal_diag_t *d)
{
    memset(d, 0, sizeof *d);
    d->imu_ok = !s_heading_failed;
    d->imu_accuracy = s_heading_failed ? 0u : 3u;
    d->gps_ok = s_gps.fix;
    d->gps_fix_type = s_gps.fix ? 3u : 0u;
    d->gps_spoof = s_gps.jam_spoof ? 2u : 1u;
    d->gps_jam = 1u;
    d->sd_card = s_sd_present;
    d->sd_mounted = s_sd_present;
    d->sd_state = s_sd_present ? 8u : 0u;
    d->sd_free_kb = s_sd_present ? 1000000u : 0xFFFFFFFFu;
    return true;
}

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

/* ---- DroneCAN bus (step 11d): a simulated MCMCAN controller + bus + nodes ----------------------------
 * The AURIX side is the REAL hal/dronecan.c (+ libcanard) on a simulated controller port: a TX FIFO, an
 * RX FIFO, error counters, bus-off. The nodes decode / encode raw frames themselves: servo nodes
 * (throttle 20 = actuator 1, steering 21 = actuator 2, gear 22 = actuator 3: ArrayCommand UNITLESS ->
 * a servo moving at its rate limit, then a backlash play; lost commands = HOLD, OA-1), the relay node
 * 30 (hardpoint.Command 0..5). Every node sends NodeStatus at 1 Hz. Faults: a node unpowered / silent
 * / not OPERATIONAL / health WARNING..CRITICAL / rebooted; the bus off; nobody acknowledging (every
 * node unpowered -> error passive); a node with OUR ID 10 (the Pixhawk, OA-4); a node asking for a
 * dynamic ID; the controller not starting. Not fitted by default (like HAL_CAN_FITTED 0). */
#define SC_TX_FIFO   16u
#define SC_RX_FIFO   32u
#define SC_NODES     4u
typedef struct {
    uint8_t  id, actuator;          /* actuator 0 = the relay node (hardpoints) */
    bool     present, silent;
    uint8_t  health, mode;
    uint32_t boot_ms, next_status_ms, init_until_ms;
    uint8_t  tid;
    float    rate, backlash;        /* servo units / s (2 / full travel s), play (units) */
    float    cmd, pos, out;         /* last command, servo position, output after the play */
    bool     have_cmd;
    uint32_t cmds, statuses;
    uint16_t hp[6];
    uint32_t hp_cmds;
} sc_node_t;
static sc_node_t s_sc[SC_NODES];
static bool      s_sc_fitted, s_sc_boot_fitted;
static uint32_t  s_sc_fault;
static dc_frame_t s_sc_tx[SC_TX_FIFO], s_sc_rx[SC_RX_FIFO];
static uint32_t  s_sc_tx_n, s_sc_rx_head, s_sc_rx_n, s_sc_rx_lost;
static uint32_t  s_sc_last_ms, s_sc_foreign_ms, s_sc_anon_ms;
static uint8_t   s_sc_tec, s_sc_rec, s_sc_foreign_tid, s_sc_anon_tid;
static bool      s_sc_busoff, s_sc_started;
static uint32_t  s_sc_recover_at;
static uint32_t  s_sc_types[4];     /* frames from the AURIX on the bus: NodeStatus, ArrayCommand, hardpoint, other */
static uint32_t  s_sc_status_ms[2]; /* our NodeStatus: the last two times seen on the bus */

static sc_node_t *sc_find(uint8_t id)
{
    for (uint32_t i = 0; i < SC_NODES; i++) if (s_sc[i].id == id) return &s_sc[i];
    return NULL;
}

void sim_hal_can_reset(void)
{
    static const uint8_t ids[SC_NODES] = { DC_NODE_THROTTLE, DC_NODE_STEERING, DC_NODE_GEAR, DC_NODE_RELAY };
    static const float travel[SC_NODES] = { 6.0f, 12.0f, 2.0f, 1.0f };     /* the worst-case servo speeds (11c) */
    memset(s_sc, 0, sizeof s_sc);
    for (uint32_t i = 0; i < SC_NODES; i++) {
        sc_node_t *n = &s_sc[i];
        n->id = ids[i];
        n->actuator = (i < 3u) ? (uint8_t)(i + 1u) : 0u;
        n->present = true;
        n->health = DC_HEALTH_OK;
        n->mode = DC_MODE_OPERATIONAL;
        n->boot_ms = s_time_ms - 60000u;            /* running for a minute already */
        n->next_status_ms = s_time_ms + 100u * i;
        n->rate = 2.0f / travel[i];
        n->backlash = (i == 1u) ? 0.03f : 0.0f;     /* the steering linkage has play */
        n->cmd = n->pos = n->out = (i == 0u) ? -1.0f : 0.0f;
    }
    s_sc_fault = 0u;
    s_sc_tx_n = s_sc_rx_n = s_sc_rx_head = s_sc_rx_lost = 0u;
    s_sc_tec = s_sc_rec = 0u;
    s_sc_busoff = false;
    s_sc_recover_at = 0u;
    memset(s_sc_types, 0, sizeof s_sc_types);
    memset(s_sc_status_ms, 0, sizeof s_sc_status_ms);
    s_sc_last_ms = s_time_ms;
}

void sim_hal_can_fitted(bool on) { s_sc_fitted = on; }
void sim_hal_can_fault(uint32_t bits) { s_sc_fault = bits; }

void sim_hal_can_node(uint8_t id, bool present, uint8_t health, uint8_t mode)
{
    sc_node_t *n = sc_find(id);
    if (!n) return;
    if (present && !n->present) {               /* power comes back = a node boot */
        n->boot_ms = s_time_ms;
        n->init_until_ms = s_time_ms + 1000u;
        n->next_status_ms = s_time_ms + 50u;
    }
    n->present = present;
    n->health = health;
    n->mode = mode;
}

void sim_hal_can_node_silent(uint8_t id, bool silent)
{
    sc_node_t *n = sc_find(id);
    if (n) n->silent = silent;
}

void sim_hal_can_node_reboot(uint8_t id)
{
    sc_node_t *n = sc_find(id);
    if (!n) return;
    n->boot_ms = s_time_ms;                      /* uptime back to 0, INITIALIZATION for 1 s */
    n->init_until_ms = s_time_ms + 1000u;
    n->next_status_ms = s_time_ms + 50u;
}

void sim_hal_can_servo_speed(uint8_t id, float travel_s, float backlash)
{
    sc_node_t *n = sc_find(id);
    if (!n || travel_s <= 0.0f) return;
    n->rate = 2.0f / travel_s;
    n->backlash = backlash;
}

bool sim_hal_can_servo(uint8_t id, float *cmd, float *pos, float *out, uint32_t *cmds)
{
    sc_node_t *n = sc_find(id);
    if (!n) return false;
    if (cmd) *cmd = n->cmd;
    if (pos) *pos = n->pos;
    if (out) *out = n->out;
    if (cmds) *cmds = n->cmds;
    return n->have_cmd;
}

uint16_t sim_hal_can_hardpoint(uint8_t hp, uint32_t *cmds)
{
    sc_node_t *n = sc_find(DC_NODE_RELAY);
    if (cmds) *cmds = n ? n->hp_cmds : 0u;
    return (n && hp < 6u) ? n->hp[hp] : 0u;
}

uint32_t sim_hal_can_bus_frames(uint32_t kind) { return kind < 4u ? s_sc_types[kind] : 0u; }
uint32_t sim_hal_can_status_period_ms(void) { return s_sc_status_ms[1] - s_sc_status_ms[0]; }
uint32_t sim_hal_can_tx_waiting(void) { return s_sc_tx_n; }
uint32_t sim_hal_can_node_statuses(uint8_t id) { sc_node_t *n = sc_find(id); return n ? n->statuses : 0u; }

static void sc_rx_push(const dc_frame_t *f)      /* a frame from the bus into our RX FIFO (full: the oldest goes) */
{
    if (s_sc_busoff) return;
    if (s_sc_rx_n == SC_RX_FIFO) {
        s_sc_rx_head = (s_sc_rx_head + 1u) % SC_RX_FIFO;
        s_sc_rx_n--;
        s_sc_rx_lost++;
    }
    s_sc_rx[(s_sc_rx_head + s_sc_rx_n) % SC_RX_FIFO] = *f;
    s_sc_rx_n++;
}

static void sc_status_frame(uint8_t src, uint8_t *tid, uint32_t uptime_s, uint8_t health, uint8_t mode)
{
    dc_status_t st = { uptime_s, health, mode, 0u, 0u };
    dc_frame_t f;
    dc_encode_status(f.data, &st);
    f.data[DC_NODESTATUS_LEN] = dc_tail(true, true, false, *tid);
    *tid = (uint8_t)((*tid + 1u) & 0x1Fu);
    f.len = DC_NODESTATUS_LEN + 1u;
    f.id = dc_msg_id(DC_PRIO_STATUS, DC_NODESTATUS_ID, src);
    sc_rx_push(&f);
}

static bool sc_anyone_acks(void)
{
    for (uint32_t i = 0; i < SC_NODES; i++) if (s_sc[i].present) return true;
    return false;
}

static void sc_deliver(const dc_frame_t *f)      /* one of our frames on the wire: every powered node sees it */
{
    uint16_t type = (uint16_t)((f->id >> 8) & 0xFFFFu);
    bool msg = !(f->id & 0x80u);
    s_sc_types[(msg && type == DC_NODESTATUS_ID) ? 0 : (msg && type == DC_ARRAYCMD_ID) ? 1 :
               (msg && type == DC_HARDPOINT_ID) ? 2 : 3]++;
    if (msg && type == DC_NODESTATUS_ID) {
        s_sc_status_ms[0] = s_sc_status_ms[1];
        s_sc_status_ms[1] = s_time_ms;
    }
    if (f->len < 1u) return;
    uint8_t tail = f->data[f->len - 1u];
    if ((tail & 0xC0u) != 0xC0u) return;         /* the nodes take single-frame transfers only (all we send them) */
    for (uint32_t i = 0; i < SC_NODES; i++) {
        sc_node_t *n = &s_sc[i];
        if (!n->present) continue;
        if (msg && type == DC_ARRAYCMD_ID && n->actuator) {
            dc_cmd_t c[15];
            uint32_t k = dc_decode_array_command(f->data, f->len - 1u, c, 15u);
            for (uint32_t j = 0; j < k; j++) {
                if (c[j].actuator_id != n->actuator || c[j].type != DC_CMD_UNITLESS) continue;
                float v = c[j].value;
                n->cmd = v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
                n->have_cmd = true;
                n->cmds++;
            }
        } else if (msg && type == DC_HARDPOINT_ID && !n->actuator) {
            uint8_t hp;
            uint16_t cmd;
            if (dc_decode_hardpoint(f->data, f->len - 1u, &hp, &cmd) && hp < 6u) {
                n->hp[hp] = cmd;
                n->hp_cmds++;
            }
        }
    }
}

static void sc_tick(void)
{
    uint32_t now = s_time_ms;
    float dt = (float)(uint32_t)(now - s_sc_last_ms) * 0.001f;
    s_sc_last_ms = now;
    /* the bus: bus-off (fault) / recovery; frames out only with someone to acknowledge them */
    if (s_sc_fault & SIM_CAN_BUSOFF) {
        if (!s_sc_busoff) s_sc_tec = 255u;
        s_sc_busoff = true;
        s_sc_recover_at = 0u;
    } else if (s_sc_busoff && s_sc_recover_at && now >= s_sc_recover_at) {
        s_sc_busoff = false;                     /* 128 x 11 recessive bits seen: error active again */
        s_sc_tec = s_sc_rec = 0u;
        s_sc_recover_at = 0u;
    }
    if (!s_sc_busoff && s_sc_tx_n) {
        if (sc_anyone_acks()) {
            for (uint32_t i = 0; i < s_sc_tx_n; i++) sc_deliver(&s_sc_tx[i]);
            s_sc_tx_n = 0u;
            s_sc_tec = (uint8_t)(s_sc_tec > 8u ? s_sc_tec - 8u : 0u);
        } else if (s_sc_tec < 136u) {
            s_sc_tec = (uint8_t)(s_sc_tec + 8u);  /* ACK errors: up to error passive, never bus-off (CAN rule) */
        }
    }
    /* the nodes */
    for (uint32_t i = 0; i < SC_NODES; i++) {
        sc_node_t *n = &s_sc[i];
        if (n->actuator) {                       /* servo: rate limit, then the play; no commands = HOLD */
            float step = n->rate * dt, d = n->cmd - n->pos;
            if (n->present) n->pos += (d > step) ? step : ((d < -step) ? -step : d);
            float h = 0.5f * n->backlash;
            if (n->pos > n->out + h) n->out = n->pos - h;
            if (n->pos < n->out - h) n->out = n->pos + h;
        }
        if (!n->present || n->silent || (int32_t)(now - n->next_status_ms) < 0) continue;
        n->next_status_ms = now + 1000u;
        uint8_t mode = ((int32_t)(now - n->init_until_ms) < 0) ? DC_MODE_INIT : n->mode;
        sc_status_frame(n->id, &n->tid, (now - n->boot_ms) / 1000u, n->health, mode);
        n->statuses++;
    }
    if ((s_sc_fault & SIM_CAN_FOREIGN10) && now - s_sc_foreign_ms >= 1000u) {   /* the Pixhawk: node 10 too */
        s_sc_foreign_ms = now;
        sc_status_frame(DC_OWN_NODE_ID, &s_sc_foreign_tid, now / 1000u, DC_HEALTH_OK, DC_MODE_OPERATIONAL);
    }
    if ((s_sc_fault & SIM_CAN_ANON) && now - s_sc_anon_ms >= 1000u) {          /* a node without a static ID */
        s_sc_anon_ms = now;
        dc_frame_t f;
        memset(&f, 0, sizeof f);
        f.data[0] = 0x01u;                       /* node_id 0 (any), first part of the unique ID */
        for (uint32_t k = 1; k < 7u; k++) f.data[k] = (uint8_t)(0x10u + k);
        f.data[7] = dc_tail(true, true, false, s_sc_anon_tid);
        s_sc_anon_tid = (uint8_t)((s_sc_anon_tid + 1u) & 0x1Fu);
        f.len = 8u;
        /* anonymous message: priority, a discriminator in bits 10..23, type ID 1 in bits 8..9, source 0 */
        f.id = ((uint32_t)DC_PRIO_STATUS << 24) | (0x1234u << 10) | ((uint32_t)DC_ALLOCATION_ID << 8);
        sc_rx_push(&f);
    }
}

/* the controller port dronecan.c drives (the same calls as hal/tc_can.c on the board) */
static bool sc_start(void)
{
    s_sc_started = !(s_sc_fault & SIM_CAN_START_FAIL);
    s_sc_tx_n = 0u;                              /* the controller restarts: its FIFOs are empty */
    s_sc_rx_n = s_sc_rx_head = 0u;
    return s_sc_started;
}

static bool sc_send(const dc_frame_t *f)
{
    if (!s_sc_started || s_sc_busoff || s_sc_tx_n >= SC_TX_FIFO) return false;
    s_sc_tx[s_sc_tx_n++] = *f;
    return true;
}

static bool sc_recv(dc_frame_t *f)
{
    if (!s_sc_started || s_sc_rx_n == 0u) return false;
    *f = s_sc_rx[s_sc_rx_head];
    s_sc_rx_head = (s_sc_rx_head + 1u) % SC_RX_FIFO;
    s_sc_rx_n--;
    return true;
}

static uint8_t sc_state(uint8_t *tec, uint8_t *rec)
{
    *tec = s_sc_tec;
    *rec = s_sc_rec;
    if (s_sc_busoff) return DC_CTRL_BUSOFF;
    return s_sc_tec >= 128u ? DC_CTRL_PASSIVE : DC_CTRL_ACTIVE;
}

static void sc_recover(void)
{
    if (s_sc_busoff && !(s_sc_fault & SIM_CAN_BUSOFF) && !s_sc_recover_at) s_sc_recover_at = s_time_ms + 50u;
}

static uint32_t sc_millis(void) { return s_time_ms; }

static const dc_port_t s_sc_port = { sc_start, sc_send, sc_recv, sc_state, sc_recover, sc_millis };

static void sc_driver_init(void)
{
    s_sc_boot_fitted = s_sc_fitted;
    if (s_sc_fitted) {
        s_sc_last_ms = s_time_ms;
        dc_init(&s_sc_port);
    }
}

void hal_can_service(void)
{
    if (!s_sc_boot_fitted) return;
    sc_tick();
    dc_service();
}

bool hal_can_bus(hal_can_bus_t *b)
{
    if (!s_sc_boot_fitted) return false;
    dc_bus(b);
    return true;
}

bool hal_can_node(uint8_t i, hal_can_node_t *n)
{
    return s_sc_boot_fitted && dc_node(i, n);
}

void hal_can_ack(void)
{
    if (s_sc_boot_fitted) dc_ack();
}
