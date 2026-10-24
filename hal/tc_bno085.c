/*
 * tc_bno085.c - BNO085 over SPI (SHTP, Hillcrest SH-2). See tc_bno085.h.
 *
 * SHTP packet: u16 length (bit 15 = continuation) | u8 channel | u8 sequence,
 * then the cargo. The host reads while H_INTN is low (sending zeros: what it
 * sends is a packet header for the sensor too); to send, it pulls WAKE low,
 * waits for H_INTN and exchanges (full duplex) in one chip-select phase.
 * Nothing here waits for the sensor across a control step: the reset pulse
 * and each Set Feature command are steps of a small state machine.
 * Channels: 0 command, 1 executable, 2 sensor hub control, 3 input reports,
 * 4 wake input reports, 5 gyro-integrated rotation vector.
 */
#include "tc_bno085.h"

#include <math.h>
#include <string.h>

#include "Ifx_Types.h"
#include "IfxPort.h"

#include "hal.h"
#include "hal_pins.h"
#include "tc_spi.h"

#define PKT_MAX     384u
#define STALE_MS    500u
#define RESET_GAP   2000u
#define ACC_RING    128u

static bno_stat_t s_st;
static uint8_t s_rx[PKT_MAX], s_tx[PKT_MAX];
static uint8_t s_seq[6];
static bool s_inited, s_need_features, s_new_hdg, s_new_yaw;
static uint32_t s_reset_ms, s_att_ms;
static uint8_t s_feat;               /* next report to enable (index into k_feat) */
static uint8_t s_phase;              /* 0 run, 1 reset pulse low, 2 waiting for the advertisement */
static bool s_waking;
static uint32_t s_wake_ms;
static const struct { uint8_t id; uint32_t us; } k_feat[4] = {
    { 0x05, 50000u },                /* rotation vector 20 Hz: heading */
    { 0x02, 20000u },                /* calibrated gyro 50 Hz: yaw rate */
    { 0x08, 50000u },                /* game rotation vector 20 Hz: roll / pitch */
    { 0x04, 5000u },                 /* linear acceleration 200 Hz: slamming */
};
static float s_acc[ACC_RING];
static uint32_t s_acc_head, s_acc_tail;

static void cs(bool active) { if (active) IfxPort_setPinLow(PIN_BNO_CS); else IfxPort_setPinHigh(PIN_BNO_CS); }
static bool int_low(void) { return !IfxPort_getPinState(PIN_BNO_INT); }

static int16_t s16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static float wrap360(float d)
{
    while (d < 0.0f) d += 360.0f;
    while (d >= 360.0f) d -= 360.0f;
    return d;
}

/* --- reports ------------------------------------------------------------------- */
static uint32_t report_len(uint8_t id)
{
    switch (id) {
    case 0xFB: case 0xFA: return 5u;                         /* base timestamp / rebase */
    case 0x01: case 0x02: case 0x03: case 0x04: case 0x06: return 10u;
    case 0x08: return 12u;                                   /* game rotation vector */
    case 0x05: case 0x09: return 14u;                        /* rotation vector (+ accuracy) */
    case 0x07: return 16u;                                   /* uncalibrated gyro */
    default: return 0u;                                      /* unknown: stop parsing this cargo */
    }
}

static void quat(const uint8_t *p, float *i, float *j, float *k, float *r)
{
    const float q14 = 1.0f / 16384.0f;
    *i = s16(p) * q14; *j = s16(p + 2) * q14; *k = s16(p + 4) * q14; *r = s16(p + 6) * q14;
}

static void input_reports(const uint8_t *p, uint32_t n)
{
    uint32_t now = hal_millis();
    for (uint32_t o = 0u; o < n;) {
        uint32_t len = report_len(p[o]);
        if (len == 0u || o + len > n) break;
        const uint8_t *r = &p[o];
        switch (r[0]) {
        case 0x05: {                                         /* rotation vector: heading of the X axis */
            float i, j, k, w;
            quat(&r[4], &i, &j, &k, &w);
            float east = 1.0f - 2.0f * (j * j + k * k);      /* the sensor X axis in the East-North-Up frame */
            float north = 2.0f * (i * j + w * k);
            float hdg = atan2f(east, north) * 57.29578f;
            s_st.heading_deg = wrap360(BNO_HEADING_SIGN * hdg + BNO_HEADING_OFFSET);
            s_st.heading_accuracy = (uint8_t)(r[2] & 3u);
            s_new_hdg = true;
            break;
        }
        case 0x08: {                                         /* game rotation vector: roll / pitch */
            float i, j, k, w;
            quat(&r[4], &i, &j, &k, &w);
            float sp = 2.0f * (w * j - k * i);
            if (sp > 1.0f) sp = 1.0f;
            if (sp < -1.0f) sp = -1.0f;
            s_st.roll_deg = BNO_ROLL_SIGN * atan2f(2.0f * (w * i + j * k), 1.0f - 2.0f * (i * i + j * j)) * 57.29578f;
            s_st.pitch_deg = BNO_PITCH_SIGN * asinf(sp) * 57.29578f;
            s_att_ms = now ? now : 1u;
            break;
        }
        case 0x02:                                           /* calibrated gyro, rad/s Q9 */
            s_st.yaw_dps = BNO_YAWRATE_SIGN * (s16(&r[8]) / 512.0f) * 57.29578f;
            s_new_yaw = true;
            break;
        case 0x04: {                                         /* linear acceleration, m/s^2 Q8 */
            float g = (s16(&r[8]) / 256.0f) / 9.80665f;
            s_st.accel_z_g = g;
            if (s_acc_head - s_acc_tail < ACC_RING) s_acc[s_acc_head++ % ACC_RING] = g;
            break;
        }
        default:
            break;
        }
        if (r[0] != 0xFB && r[0] != 0xFA) {
            s_st.reports++;
            s_st.last_report_ms = now ? now : 1u;
        }
        o += len;
    }
}

static void cargo(const uint8_t *pkt, uint32_t len)
{
    uint8_t ch = pkt[2];
    s_st.packets++;
    if (ch == 3u || ch == 4u) input_reports(&pkt[4], len - 4u);
    else if (ch == 0u || ch == 1u) s_need_features = true;    /* advertisement / reset complete */
}

/* One chip-select phase: send tx_len bytes of s_tx (0 = read only, zeros) and
 * read the sensor's packet at the same time. */
static bool transfer(uint32_t tx_len)
{
    uint8_t hdr_tx[4] = { 0, 0, 0, 0 };
    if (tx_len >= 4u) memcpy(hdr_tx, s_tx, 4u);
    cs(true);
    bool ok = spi_xfer(SPI_BNO, hdr_tx, s_rx, 4u, 5u);
    uint32_t rx_len = ok ? (((uint32_t)s_rx[0] | ((uint32_t)s_rx[1] << 8)) & 0x7FFFu) : 0u;
    if (rx_len == 0x7FFFu || (rx_len > 0u && rx_len < 4u)) rx_len = 0u;   /* nothing / invalid */
    uint32_t total = rx_len > tx_len ? rx_len : tx_len;
    if (ok && total > 4u) {
        uint32_t rest = total - 4u, got = 0u;
        static uint8_t zeros[PKT_MAX];
        while (ok && got < rest) {
            uint32_t k = rest - got;
            if (4u + got + k > PKT_MAX) k = PKT_MAX - 4u - got;
            if (k == 0u) {                                   /* longer than our buffer: drain and drop */
                uint8_t junk[32];
                uint32_t d = rest - got < sizeof junk ? rest - got : (uint32_t)sizeof junk;
                ok = spi_xfer(SPI_BNO, zeros, junk, d, 5u);
                got += d;
                rx_len = 0u;
                continue;
            }
            const uint8_t *src = (4u + got < tx_len) ? &s_tx[4u + got] : zeros;
            ok = spi_xfer(SPI_BNO, src, &s_rx[4u + got], k, 10u);
            got += k;
        }
    }
    cs(false);
    if (!ok) { s_st.spi_errors++; return false; }
    if (rx_len >= 4u && rx_len <= PKT_MAX) cargo(s_rx, rx_len);
    return true;
}

/* Prepare a Set Feature Command (channel 2) in s_tx. */
static uint32_t build_feature(uint8_t report, uint32_t interval_us)
{
    uint8_t *c = &s_tx[4];
    memset(c, 0, 17u);
    c[0] = 0xFD;                                             /* Set Feature Command */
    c[1] = report;
    c[5] = (uint8_t)interval_us; c[6] = (uint8_t)(interval_us >> 8);
    c[7] = (uint8_t)(interval_us >> 16); c[8] = (uint8_t)(interval_us >> 24);
    s_tx[0] = 21u; s_tx[1] = 0u; s_tx[2] = 2u; s_tx[3] = s_seq[2]++;
    return 21u;
}

/* Enable the reports one by one: pull WAKE, and on a later step (H_INTN low:
 * the sensor listens) send the command. A sensor that doesn't answer in
 * 100 ms: WAKE released, tried again. */
static void features_step(uint32_t now)
{
    if (!s_waking) {
        IfxPort_setPinLow(PIN_BNO_WAKE);
        s_waking = true;
        s_wake_ms = now;
        return;
    }
    if (int_low()) {
        IfxPort_setPinHigh(PIN_BNO_WAKE);
        s_waking = false;
        if (transfer(build_feature(k_feat[s_feat].id, k_feat[s_feat].us)) && ++s_feat >= 4u) {
            s_feat = 0;
            s_need_features = false;
        }
    } else if ((uint32_t)(now - s_wake_ms) > 100u) {
        IfxPort_setPinHigh(PIN_BNO_WAKE);
        s_waking = false;
        s_st.spi_errors++;
    }
}

/* Reset pulse: low now, high on the next step (> 10 ms later). */
static void hw_reset(void)
{
    IfxPort_setPinLow(PIN_BNO_RST);
    IfxPort_setPinHigh(PIN_BNO_WAKE);
    s_waking = false;
    s_phase = 1u;
    memset(s_seq, 0, sizeof s_seq);
    s_feat = 0;
    s_need_features = true;
    s_reset_ms = hal_millis() ? hal_millis() : 1u;
    s_st.resets++;
}

void bno_init(void)
{
    IfxPort_setPinHigh(PIN_BNO_CS);
    IfxPort_setPinModeOutput(PIN_BNO_CS, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(PIN_BNO_WAKE);
    IfxPort_setPinModeOutput(PIN_BNO_WAKE, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(PIN_BNO_RST);
    IfxPort_setPinModeOutput(PIN_BNO_RST, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(PIN_BNO_INT, IfxPort_InputMode_pullUp);
    spi_init(SPI_BNO, BNO_SPI_HZ);
    hw_reset();
    s_inited = true;
}

void bno_service(void)
{
    if (!s_inited) return;
    uint32_t now = hal_millis();
    if (s_phase == 1u) {                                     /* end of the reset pulse */
        if ((uint32_t)(now - s_reset_ms) >= 10u) {
            IfxPort_setPinHigh(PIN_BNO_RST);
            s_phase = 2u;
        }
        return;
    }
    /* the sensor's messages that are waiting (bounded: <= 4 packets, ~6 ms) */
    for (int k = 0; k < 4 && int_low() && !s_waking; k++) {
        if (!transfer(0u)) break;
    }
    /* after a reset (advertisement read, ~0.2 s): enable the reports */
    if (s_need_features && (uint32_t)(now - s_reset_ms) > 200u) features_step(now);
    if (!s_need_features) s_phase = 0u;
    s_st.ok = s_st.last_report_ms != 0u && (uint32_t)(now - s_st.last_report_ms) < STALE_MS;
    /* no data: reset the sensor (at most every 2 s) */
    if (!s_st.ok && (uint32_t)(now - s_reset_ms) > RESET_GAP) {
        spi_init(SPI_BNO, BNO_SPI_HZ);
        hw_reset();
    }
}

bool bno_heading(float *deg)
{
    if (!s_new_hdg || !s_st.ok) return false;
    s_new_hdg = false;
    *deg = s_st.heading_deg;
    return true;
}

bool bno_yaw_rate(float *dps)
{
    if (!s_new_yaw || !s_st.ok) return false;
    s_new_yaw = false;
    *dps = s_st.yaw_dps;
    return true;
}

bool bno_attitude(float *roll_deg, float *pitch_deg)
{
    if (s_att_ms == 0u || (uint32_t)(hal_millis() - s_att_ms) > STALE_MS) return false;
    *roll_deg = s_st.roll_deg;
    *pitch_deg = s_st.pitch_deg;
    return true;
}

uint32_t bno_accel_z(float *g, uint32_t max)
{
    uint32_t n = 0u;
    while (s_acc_tail != s_acc_head && n < max) g[n++] = s_acc[s_acc_tail++ % ACC_RING];
    return n;
}

const bno_stat_t *bno_stat(void)
{
    return &s_st;
}
