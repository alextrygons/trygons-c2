/*
 * tc_gps.c - u-blox MAX-M10S (UBX protocol, NMEA fallback). See tc_gps.h.
 *
 * Start-up: the module comes at 9600 baud (factory) - or still at 38400 if
 * only the AURIX was reset. UBX-CFG-VALSET (RAM layer) is sent at both baud
 * rates; it switches the module to 38400 baud, UBX output only, NAV-PVT at
 * 1 Hz, NAV-STATUS + MON-RF at 1 Hz, sea dynamic model, jamming detection on.
 * Repeated every 10 s while no UBX message comes in. Never waits: phase 1
 * sends at 9600, phase 2 (>= 150 ms later, the bytes are out) at 38400.
 */
#include "tc_gps.h"

#include <stdlib.h>
#include <string.h>

#include "hal_pins.h"
#include "tc_uart.h"

#define NO_DATA_MS   3000u
#define RECONFIG_MS  10000u

static gps_stat_t s_st;
static uint8_t s_buf[128];           /* UBX message being collected */
static uint32_t s_n, s_need;
static char s_nmea[100];
static uint32_t s_nl;
static bool s_in_nmea, s_have_fix;
static hal_gps_t s_fix;
static uint32_t s_cfg_ms;
static uint8_t s_cfg_phase;          /* 0 idle, 1 sent at 9600 (switch + send at 38400 next) */
static uint8_t s_cfg_msg[104];
static uint16_t s_cfg_len;

/* --- configuration ------------------------------------------------------------- */
static uint32_t put_key(uint8_t *p, uint32_t key, uint32_t val, uint32_t size)
{
    p[0] = (uint8_t)key; p[1] = (uint8_t)(key >> 8); p[2] = (uint8_t)(key >> 16); p[3] = (uint8_t)(key >> 24);
    for (uint32_t i = 0; i < size; i++) p[4 + i] = (uint8_t)(val >> (8u * i));
    return 4u + size;
}

static uint16_t build_ubx(uint8_t *m, uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
    m[0] = 0xB5; m[1] = 0x62; m[2] = cls; m[3] = id; m[4] = (uint8_t)len; m[5] = (uint8_t)(len >> 8);
    memcpy(&m[6], pl, len);
    uint8_t a = 0, b = 0;
    for (uint32_t i = 2; i < 6u + len; i++) { a = (uint8_t)(a + m[i]); b = (uint8_t)(b + a); }
    m[6 + len] = a; m[7 + len] = b;
    return (uint16_t)(8u + len);
}

/* phase 1: at the factory 9600 baud */
static void configure(void)
{
    uint8_t p[96];
    uint32_t n = 4;
    p[0] = 0; p[1] = 0x01; p[2] = 0; p[3] = 0;                     /* version 0, layer RAM */
    n += put_key(&p[n], 0x40520001u, 38400u, 4);                    /* CFG-UART1-BAUDRATE */
    n += put_key(&p[n], 0x10740001u, 1u, 1);                        /* CFG-UART1OUTPROT-UBX */
    n += put_key(&p[n], 0x10740002u, 0u, 1);                        /* CFG-UART1OUTPROT-NMEA off */
    n += put_key(&p[n], 0x20910007u, 1u, 1);                        /* CFG-MSGOUT-UBX_NAV_PVT_UART1 */
    n += put_key(&p[n], 0x2091001Bu, 1u, 1);                        /* CFG-MSGOUT-UBX_NAV_STATUS_UART1 */
    n += put_key(&p[n], 0x2091035Au, 1u, 1);                        /* CFG-MSGOUT-UBX_MON_RF_UART1 */
    n += put_key(&p[n], 0x30210001u, 1000u, 2);                     /* CFG-RATE-MEAS 1000 ms (1 Hz) */
    n += put_key(&p[n], 0x20110021u, 5u, 1);                        /* CFG-NAVSPG-DYNMODEL = sea */
    n += put_key(&p[n], 0x1041000Du, 1u, 1);                        /* CFG-ITFM-ENABLE (jamming detection) */
    s_cfg_len = build_ubx(s_cfg_msg, 0x06, 0x8A, p, (uint16_t)n);  /* UBX-CFG-VALSET */
    uart_set_baud(UART_GPS, GPS_BAUD_DEFAULT);
    (void)uart_write(UART_GPS, s_cfg_msg, s_cfg_len);
    s_cfg_phase = 1u;
    s_cfg_ms = hal_millis() ? hal_millis() : 1u;
    s_st.configs++;
}

/* phase 2 (the 9600 bytes are out): our own 38400, and the message once more in
 * case the module already was at 38400 (only the AURIX was reset) */
static void configure_phase2(void)
{
    uart_set_baud(UART_GPS, GPS_BAUD);
    (void)uart_write(UART_GPS, s_cfg_msg, s_cfg_len);
    s_cfg_phase = 0u;
}

void gps_init(void)
{
    uart_init(UART_GPS, GPS_BAUD_DEFAULT);
    configure();
}

/* --- UBX -------------------------------------------------------------------------- */
static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static bool ubx_message(uint8_t cls, uint8_t id, const uint8_t *p, uint32_t len)
{
    uint32_t now = hal_millis() ? hal_millis() : 1u;
    if (cls == 0x01 && id == 0x07 && len >= 92u) {                  /* NAV-PVT */
        uint8_t valid = p[11], fix_type = p[20], flags = p[21];
        memset(&s_fix, 0, sizeof s_fix);
        s_fix.fix = (fix_type == 2u || fix_type == 3u) && (flags & 1u);    /* 2D / 3D and gnssFixOK */
        s_fix.lon_deg = (int32_t)u32(&p[24]) * 1e-7;
        s_fix.lat_deg = (int32_t)u32(&p[28]) * 1e-7;
        s_fix.sog_kn = (float)((int32_t)u32(&p[60])) * 0.001f * 1.943844f;   /* mm/s */
        float cog = (float)((int32_t)u32(&p[64])) * 1e-5f;
        while (cog < 0.0f) cog += 360.0f;
        while (cog >= 360.0f) cog -= 360.0f;
        s_fix.cog_deg = cog;
        s_fix.sats = p[23];
        if (valid & 2u) {                                           /* validTime */
            int32_t nano = (int32_t)u32(&p[16]);
            int32_t ms = ((int32_t)p[8] * 3600 + (int32_t)p[9] * 60 + (int32_t)p[10]) * 1000 + nano / 1000000;
            s_fix.time_ms = (uint32_t)(ms < 0 ? 0 : ms);
        }
        if (valid & 1u) s_fix.date = ((uint32_t)p[4] | ((uint32_t)p[5] << 8)) * 10000u + p[6] * 100u + p[7];
        s_fix.jam_spoof = s_st.spoof_state >= 2u || s_st.jam_state >= 2u;
        s_st.sats = p[23];
        s_st.fix_type = fix_type;
        s_st.pvt++;
        s_st.ubx = true;
        s_st.last_msg_ms = now;
        return true;
    }
    if (cls == 0x01 && id == 0x03 && len >= 16u) {                  /* NAV-STATUS: flags2 bits 3..4 */
        s_st.spoof_state = (uint8_t)((p[7] >> 3) & 3u);
        s_st.status++;
    } else if (cls == 0x0A && id == 0x38 && len >= 4u) {            /* MON-RF: worst block */
        uint8_t nb = p[1], worst = 0;
        for (uint32_t b = 0; b < nb && 4u + 24u * (b + 1u) <= len; b++) {
            uint8_t js = (uint8_t)(p[4 + 24u * b + 1] & 3u);
            if (js > worst) worst = js;
        }
        s_st.jam_state = worst;
        s_st.monrf++;
    }
    return false;
}

/* --- NMEA RMC fallback --------------------------------------------------------- */
static double nmea_deg(const char *f, int deg_digits)
{
    if (!f || !*f) return 0.0;
    char d[4] = { 0 };
    memcpy(d, f, (size_t)deg_digits);
    return atof(d) + atof(f + deg_digits) / 60.0;
}

static bool nmea_line(char *s)
{
    if (strlen(s) < 7 || strncmp(s + 3, "RMC", 3) != 0) return false;
    char *star = strchr(s, '*');
    if (!star) return false;
    uint8_t cs = 0;
    for (char *c = s + 1; c < star; c++) cs ^= (uint8_t)*c;
    if (strtol(star + 1, NULL, 16) != cs) { s_st.bad_checksum++; return false; }
    *star = 0;
    char *f[13] = { 0 };
    int nf = 0;
    for (char *c = s; nf < 13; ) {
        f[nf++] = c;
        c = strchr(c, ',');
        if (!c) break;
        *c++ = 0;
    }
    if (nf < 10) return false;
    memset(&s_fix, 0, sizeof s_fix);
    s_fix.fix = f[2][0] == 'A';
    double lat = nmea_deg(f[3], 2), lon = nmea_deg(f[5], 3);
    s_fix.lat_deg = (f[4][0] == 'S') ? -lat : lat;
    s_fix.lon_deg = (f[6][0] == 'W') ? -lon : lon;
    s_fix.sog_kn = (float)atof(f[7]);
    s_fix.cog_deg = (float)atof(f[8]);
    if (strlen(f[1]) >= 6) {
        int hh = (f[1][0] - '0') * 10 + (f[1][1] - '0'), mm = (f[1][2] - '0') * 10 + (f[1][3] - '0');
        double ss = atof(f[1] + 4);
        s_fix.time_ms = (uint32_t)((hh * 3600 + mm * 60) * 1000 + (int)(ss * 1000.0));
    }
    if (strlen(f[9]) == 6) {
        int dd = (f[9][0] - '0') * 10 + (f[9][1] - '0'), mo = (f[9][2] - '0') * 10 + (f[9][3] - '0');
        int yy = (f[9][4] - '0') * 10 + (f[9][5] - '0');
        s_fix.date = (uint32_t)((2000 + yy) * 10000 + mo * 100 + dd);
    }
    s_fix.jam_spoof = false;                                        /* not known without UBX */
    s_st.rmc++;
    s_st.last_msg_ms = hal_millis() ? hal_millis() : 1u;
    return true;
}

/* --- byte stream ------------------------------------------------------------------- */
bool gps_poll(hal_gps_t *fix)
{
    uint8_t in[256];
    uint32_t n;
    bool got = false;
    while ((n = uart_read(UART_GPS, in, sizeof in)) > 0u) {
        for (uint32_t i = 0; i < n; i++) {
            uint8_t c = in[i];
            if (s_n == 0u && !s_in_nmea && c == '$') { s_in_nmea = true; s_nl = 0; }
            if (s_in_nmea) {
                if (c == '\r' || c == '\n') {
                    s_nmea[s_nl < sizeof s_nmea ? s_nl : sizeof s_nmea - 1] = 0;
                    if (s_nl > 0 && s_nl < sizeof s_nmea && nmea_line(s_nmea)) got = true;
                    s_in_nmea = false;
                } else if (s_nl < sizeof s_nmea - 1) {
                    s_nmea[s_nl++] = (char)c;
                } else {
                    s_in_nmea = false;                              /* too long: drop */
                }
                continue;
            }
            /* UBX: B5 62 class id len16 payload ck_a ck_b */
            if (s_n == 0u && c != 0xB5) continue;
            if (s_n == 1u && c != 0x62) { s_n = 0; continue; }
            if (s_n < sizeof s_buf) s_buf[s_n] = c;
            s_n++;
            if (s_n == 6u) {
                s_need = 8u + ((uint32_t)s_buf[4] | ((uint32_t)s_buf[5] << 8));
                if (s_need > sizeof s_buf) s_need = 0x7FFFFFFFu;    /* too long for us: skip it */
            }
            if (s_n >= 6u && s_n == s_need) {
                uint8_t a = 0, b = 0;
                for (uint32_t k = 2; k < s_need - 2u; k++) { a = (uint8_t)(a + s_buf[k]); b = (uint8_t)(b + a); }
                if (a == s_buf[s_need - 2u] && b == s_buf[s_need - 1u]) {
                    if (ubx_message(s_buf[2], s_buf[3], &s_buf[6], s_need - 8u)) got = true;
                } else {
                    s_st.bad_checksum++;
                }
                s_n = 0;
            } else if (s_n > 6u && s_need == 0x7FFFFFFFu && s_n > 1024u) {
                s_n = 0;
            }
        }
    }
    uint32_t now = hal_millis();
    s_st.ok = s_st.last_msg_ms != 0u && (uint32_t)(now - s_st.last_msg_ms) < NO_DATA_MS;
    if (s_cfg_phase == 1u && (uint32_t)(now - s_cfg_ms) >= 150u) configure_phase2();
    else if (s_cfg_phase == 0u && !s_st.ubx && (uint32_t)(now - s_cfg_ms) > RECONFIG_MS) configure();
    if (got) {
        s_have_fix = true;
        *fix = s_fix;
    }
    return got;
}

const gps_stat_t *gps_stat(void)
{
    return &s_st;
}
