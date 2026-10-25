/*
 * sim_server.c - Simulated small boat behind a TCP socket, for the ground
 * station. Same frames (proto.h) as the USB link on the real board.
 *
 *   autopilot_sim server [port] [speed]
 *       port   TCP port (default 5760), listens on 127.0.0.1 only
 *       speed  simulation speed, 1 = real time (default), 10 = 10x faster
 *
 * One PC at a time. Sim control frames 0x70.. (simulated joystick, GPS
 * failure) are only understood here, never on the boat.
 */
#if !defined(_WIN32)
#define _POSIX_C_SOURCE 200112L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
typedef SOCKET sock_t;
#define SOCK_BAD INVALID_SOCKET
#define sock_close closesocket
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
typedef int sock_t;
#define SOCK_BAD (-1)
#define sock_close close
#endif

#include "hal_sim.h"
#include "joystick.h"
#include "link.h"
#include "monitor.h"
#include "timing.h"
#include "logq.h"
#include "logcore0.h"
#include "logger.h"
#include "sim_run.h"
#include "sim_world.h"
#include "smallboat.h"

typedef struct {
    world_t w;
    sb_t    sb;
    link_t  link;
    link_t  gwlink;         /* the Raspberry Pi gateway UART */
    mon_t   mon;            /* independent safety monitor (CPU1 on the board) */
    uint32_t t, next_ctrl;
} server_t;

static server_t g_srv;      /* large: keep it off the stack */
static logger_t g_lg;       /* core 2 logger: writes TLG files into ./sdcard */

/* --- platform helpers ------------------------------------------------------ */

static double wall_s(void)
{
#ifdef _WIN32
    static LARGE_INTEGER f;
    LARGE_INTEGER c;
    if (f.QuadPart == 0) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

static void sleep_ms(unsigned ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec ts = { 0, (long)ms * 1000000L };
    nanosleep(&ts, NULL);
#endif
}

static void set_nonblocking(sock_t s)
{
#ifdef _WIN32
    u_long on = 1;
    ioctlsocket(s, FIONBIO, &on);
#else
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
}

static bool would_block(void)
{
#ifdef _WIN32
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

/* --- sim control frames ---------------------------------------------------- */

static uint16_t joy_raw(const joy_cal_t *c, int a, float v)
{
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    float r = (v >= 0.0f) ? c->center[a] + v * (float)(c->max[a] - c->center[a])
                          : c->center[a] + v * (float)(c->center[a] - c->min[a]);
    return (uint16_t)(r + 0.5f);
}

static uint8_t sim_cmd(void *ctx, const proto_frame_t *f)
{
    server_t *x = (server_t *)ctx;
    proto_rd_t r;
    proto_rd_init(&r, f->payload, f->len);
    switch (f->type) {
    case PROTO_T_SIM_JOYSTICK: {
        if (f->len != 11u && f->len != 12u) return PROTO_R_BAD_LENGTH;
        uint8_t fitted = proto_rd_u8(&r);
        float jx = proto_rd_f32(&r), jy = proto_rd_f32(&r);
        uint8_t take = proto_rd_u8(&r), autob = proto_rd_u8(&r);
        uint8_t confirm = (f->len == 12u) ? proto_rd_u8(&r) : 0u;
        if (!(jx >= -1.0f && jx <= 1.0f && jy >= -1.0f && jy <= 1.0f)) return PROTO_R_BAD_VALUE;
        joy_cal_t c;
        joy_default_cal(&c);        /* the sim stick matches the default calibration */
        sim_hal_set_joystick(fitted != 0u, joy_raw(&c, 0, jx), joy_raw(&c, 1, jy), take != 0u, autob != 0u);
        sim_hal_joystick_confirm(confirm != 0u);
        return PROTO_R_OK;
    }
    case PROTO_T_SIM_BUTTON: {
        if (f->len != 1u) return PROTO_R_BAD_LENGTH;
        uint8_t b = proto_rd_u8(&r);
        if (b >= (uint8_t)HAL_BTN_COUNT) return PROTO_R_BAD_VALUE;
        sim_hal_press((hal_button_t)b);
        return PROTO_R_OK;
    }
    case PROTO_T_SIM_BRIDGE: {
        /* the USB joysticks behind the simulated bridge (used when the boat's
         * joystick source is "USB bridge") */
        if (f->len != 1u + 2u * (JSF_AXES * 2u + 5u) + 1u) return PROTO_R_BAD_LENGTH;
        jsf_t j;
        memset(&j, 0, sizeof j);
        j.flags = proto_rd_u8(&r);
        for (int d = 0; d < JSF_DEVS; d++) {
            for (int a = 0; a < JSF_AXES; a++) j.dev[d].axis[a] = (int16_t)proto_rd_u16(&r);
            j.dev[d].buttons = proto_rd_u32(&r);
            j.dev[d].hats = proto_rd_u8(&r);
        }
        uint8_t fault = proto_rd_u8(&r);
        sim_hal_bridge_set(&j);
        sim_hal_bridge_fault(fault);
        return PROTO_R_OK;
    }
    case PROTO_T_SIM_GPS:
        if (f->len != 1u) return PROTO_R_BAD_LENGTH;
        x->w.gps_fail = proto_rd_u8(&r) != 0u;
        return PROTO_R_OK;
    default:
        return PROTO_R_UNKNOWN;
    }
}

/* --- main loop ------------------------------------------------------------- */

static bool s_nojoy;            /* "nojoy": no onboard joystick at power-up (UNCREWED profile) */
void sim_server_nojoy(bool on) { s_nojoy = on; }

static void sim_init(server_t *x, uint32_t seed, bool usb, uint32_t gw_baud)
{
    sb_config_t c;
    memset(x, 0, sizeof *x);
    remove("nv1.bin");                          /* default joystick calibration */
    remove("nv4.bin");                          /* factory joystick map ... */
    if (usb) {                                  /* ... or the USB T.16000M + TWCS */
        joymap_t m;
        joymap_default(&m);
        m.source = JOY_SRC_BRIDGE;
        joymap_seal(&m);
        (void)hal_nv_save(HAL_NV_SLOT_JOYMAP, &m, (uint32_t)sizeof m);
    }
    world_init(&x->w, seed, 90.0f, 0.0f);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, 0.05f, 0.02f);
    sim_hal_reset_engine();
    sim_hal_link_reset();
    sim_hal_gw_reset(gw_baud);
    sim_hal_set_joystick(!s_nojoy, 2048u, 2048u, false, false);    /* fitted, centred (nojoy: none) */
    world_publish(&x->w, 0);
    hal_init();
    sb_default_config(&c);
    sim_hal_unforce();
    sim_hal_real_micros(true);                  /* timing: the real PC clock */
    timing_init_all();
    memset((void *)&g_mon, 0, sizeof g_mon);
    sb_init(&x->sb, &c);
    monitor_init(&x->mon, 0u);
    link_init(&x->link, &x->sb);
    link_init_port(&x->gwlink, &x->sb, LINK_PORT_GW);
    /* logging like the board: boot counter, queues, logger into ./sdcard */
    {
        uint32_t boot = 0u;
        if (!hal_nv_load(HAL_NV_SLOT_BOOT, &boot, sizeof boot)) boot = 0u;
        boot++;
        (void)hal_nv_save(HAL_NV_SLOT_BOOT, &boot, sizeof boot);
        (void)system("mkdir sdcard 2>nul");
        sim_hal_sd_setup("sdcard", true, 4000000u, false);
        logq_init_all();
        g_logstat.boot = boot;
        logger_init(&g_lg, 0u);
        logcore0_init(&x->sb, boot);
    }
    x->link.sim_fn = sim_cmd;
    x->link.sim_ctx = x;
}

static void sim_advance(server_t *x, uint32_t until_ms)
{
    while (x->t < until_ms) {
        world_publish(&x->w, x->t);
        /* the simulated bridge sends frames while the boat uses the USB source */
        bool usb = x->sb.jmap.source == JOY_SRC_BRIDGE;
        if (usb != sim_hal_bridge_on()) sim_hal_bridge_mode(usb);
        if (x->t >= x->next_ctrl) {
            x->next_ctrl += AP_PERIOD_MS;
            uint32_t t0 = hal_micros();
            sb_step(&x->sb);
            link_step(&x->link, &x->sb);
            link_step(&x->gwlink, &x->sb);
            logcore0_step(&x->sb);
            (void)cyc_record(&g_timing.core0, hal_micros() - t0);
        }
        if (x->t % 10u == 0u) {                     /* the logger (core 2) */
            uint32_t t2 = hal_micros();
            logger_step(&g_lg, x->t);
            (void)cyc_record(&g_timing.logger, hal_micros() - t2);
        }
        uint32_t t1 = hal_micros();
        monitor_step(&x->mon, &g_mon, x->t);        /* separate check step */
        (void)cyc_record(&g_timing.monitor, hal_micros() - t1);
        world_step(&x->w, PHYSICS_DT_MS * 0.001f);
        x->t += PHYSICS_DT_MS;
    }
}

/* One TCP endpoint: the ground station (USB link) or the gateway (the Pi's
 * UART). Each takes one client at a time. */
typedef struct {
    const char *name;
    sock_t ls, cs;
    uint32_t (*push)(const uint8_t *buf, uint32_t len);     /* client -> boat */
    uint32_t (*pop)(uint8_t *buf, uint32_t max);            /* boat -> client */
} endpoint_t;

static bool ep_listen(endpoint_t *e, int port)
{
    int yes = 1;
    e->cs = SOCK_BAD;
    e->ls = socket(AF_INET, SOCK_STREAM, 0);
    if (e->ls == SOCK_BAD) return false;
    setsockopt(e->ls, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof yes);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(e->ls, (struct sockaddr *)&a, sizeof a) != 0 || listen(e->ls, 1) != 0) {
        printf("cannot listen on 127.0.0.1:%d (already in use?)\n", port);
        sock_close(e->ls);
        return false;
    }
    set_nonblocking(e->ls);
    return true;
}

static void ep_poll_in(endpoint_t *e, const server_t *x)
{
    int yes = 1;
    if (e->cs == SOCK_BAD) {
        e->cs = accept(e->ls, NULL, NULL);
        if (e->cs != SOCK_BAD) {
            set_nonblocking(e->cs);
            setsockopt(e->cs, IPPROTO_TCP, TCP_NODELAY, (const char *)&yes, sizeof yes);
            printf("[%7.1f s] %s connected\n", x->t * 0.001, e->name);
            fflush(stdout);
        }
    }
    if (e->cs == SOCK_BAD) return;
    char buf[1024];
    for (;;) {
        int n = (int)recv(e->cs, buf, sizeof buf, 0);
        if (n > 0) { e->push((const uint8_t *)buf, (uint32_t)n); continue; }
        if (n == 0 || !would_block()) {
            printf("[%7.1f s] %s disconnected\n", x->t * 0.001, e->name);
            fflush(stdout);
            sock_close(e->cs);
            e->cs = SOCK_BAD;
        }
        break;
    }
}

static void ep_flush_out(endpoint_t *e)
{
    uint8_t out[4096];
    uint32_t n;
    while ((n = e->pop(out, sizeof out)) > 0u) {
        if (e->cs == SOCK_BAD) continue;            /* nobody listening: drop */
        uint32_t sent = 0u;
        while (sent < n) {
            int k = (int)send(e->cs, (const char *)out + sent, (int)(n - sent), 0);
            if (k > 0) { sent += (uint32_t)k; continue; }
            if (k < 0 && would_block()) { sleep_ms(1); continue; }
            break;
        }
    }
}

int sim_server(int port, double speed, bool usb, uint32_t gw_baud)
{
    server_t *x = &g_srv;
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { printf("WSAStartup failed\n"); return 1; }
#endif
    endpoint_t gs = { "ground station", SOCK_BAD, SOCK_BAD, sim_hal_link_rx_push, sim_hal_link_tx_pop };
    endpoint_t gw = { "gateway (Pi UART)", SOCK_BAD, SOCK_BAD, sim_hal_gw_rx_push, sim_hal_gw_tx_pop };
    if (!ep_listen(&gs, port) || !ep_listen(&gw, port + 2)) return 1;

    sim_init(x, 4242u, usb, gw_baud);
    printf("Simulated boat: ground station on 127.0.0.1:%d, gateway UART on 127.0.0.1:%d (%lu baud), "
           "speed x%.1f. Ctrl+C to stop.\n", port, port + 2, (unsigned long)gw_baud, speed);
    fflush(stdout);

    double t0 = wall_s();
    uint32_t next_print = 0u;
    for (;;) {
        ep_poll_in(&gs, x);
        ep_poll_in(&gw, x);

        /* advance the boat to "now" (at most 1 s of catch-up) */
        uint32_t target = (uint32_t)((wall_s() - t0) * speed * 1000.0);
        if (target > x->t + 1000u) { t0 += (double)(target - x->t - 1000u) / (speed * 1000.0); target = x->t + 1000u; }
        sim_advance(x, target);

        ep_flush_out(&gs);
        ep_flush_out(&gw);

        if (x->t >= next_print) {
            next_print = x->t + 10000u;
            sb_status_t st = sb_status(&x->sb);
            printf("[%7.1f s] %-8s hdg %5.1f  %4.1f kn  thr %4.1f %%  alarms 0x%05x\n", x->t * 0.001,
                   mode_name(sb_mode(&x->sb)), x->sb.ap.heading, x->w.speed_kn, x->sb.thr.out_pct,
                   (unsigned)st.alarm_bits);
            fflush(stdout);
        }
        sleep_ms(2);
    }
}
