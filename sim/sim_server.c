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
    case PROTO_T_SIM_GPS:
        if (f->len != 1u) return PROTO_R_BAD_LENGTH;
        x->w.gps_fail = proto_rd_u8(&r) != 0u;
        return PROTO_R_OK;
    default:
        return PROTO_R_UNKNOWN;
    }
}

/* --- main loop ------------------------------------------------------------- */

static void sim_init(server_t *x, uint32_t seed)
{
    sb_config_t c;
    memset(x, 0, sizeof *x);
    remove("nv1.bin");                          /* default joystick calibration */
    world_init(&x->w, seed, 90.0f, 0.0f);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, 0.05f, 0.02f);
    sim_hal_reset_engine();
    sim_hal_link_reset();
    sim_hal_set_joystick(true, 2048u, 2048u, false, false);    /* fitted, centred */
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
        if (x->t >= x->next_ctrl) {
            x->next_ctrl += AP_PERIOD_MS;
            uint32_t t0 = hal_micros();
            sb_step(&x->sb);
            link_step(&x->link, &x->sb);
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

int sim_server(int port, double speed)
{
    server_t *x = &g_srv;
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { printf("WSAStartup failed\n"); return 1; }
#endif
    sock_t ls = socket(AF_INET, SOCK_STREAM, 0);
    if (ls == SOCK_BAD) { printf("socket() failed\n"); return 1; }
    int yes = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof yes);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(ls, (struct sockaddr *)&a, sizeof a) != 0 || listen(ls, 1) != 0) {
        printf("cannot listen on 127.0.0.1:%d (already in use?)\n", port);
        sock_close(ls);
        return 1;
    }
    set_nonblocking(ls);

    sim_init(x, 4242u);
    printf("Simulated boat: listening on 127.0.0.1:%d, speed x%.1f. Ctrl+C to stop.\n", port, speed);
    fflush(stdout);

    sock_t cs = SOCK_BAD;
    double t0 = wall_s();
    uint32_t next_print = 0u;
    for (;;) {
        if (cs == SOCK_BAD) {
            cs = accept(ls, NULL, NULL);
            if (cs != SOCK_BAD) {
                set_nonblocking(cs);
                setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, (const char *)&yes, sizeof yes);
                printf("[%7.1f s] ground station connected\n", x->t * 0.001);
                fflush(stdout);
            }
        }
        if (cs != SOCK_BAD) {
            char buf[1024];
            for (;;) {
                int n = (int)recv(cs, buf, sizeof buf, 0);
                if (n > 0) { sim_hal_link_rx_push((const uint8_t *)buf, (uint32_t)n); continue; }
                if (n == 0 || !would_block()) {
                    printf("[%7.1f s] ground station disconnected\n", x->t * 0.001);
                    fflush(stdout);
                    sock_close(cs);
                    cs = SOCK_BAD;
                }
                break;
            }
        }

        /* advance the boat to "now" (at most 1 s of catch-up) */
        uint32_t target = (uint32_t)((wall_s() - t0) * speed * 1000.0);
        if (target > x->t + 1000u) { t0 += (double)(target - x->t - 1000u) / (speed * 1000.0); target = x->t + 1000u; }
        sim_advance(x, target);

        uint8_t out[4096];
        uint32_t n;
        while ((n = sim_hal_link_tx_pop(out, sizeof out)) > 0u) {
            if (cs == SOCK_BAD) continue;              /* nobody listening: drop */
            uint32_t sent = 0u;
            while (sent < n) {
                int k = (int)send(cs, (const char *)out + sent, (int)(n - sent), 0);
                if (k > 0) { sent += (uint32_t)k; continue; }
                if (k < 0 && would_block()) { sleep_ms(1); continue; }
                break;
            }
        }

        if (x->t >= next_print) {
            next_print = x->t + 10000u;
            sb_status_t st = sb_status(&x->sb);
            printf("[%7.1f s] %-8s hdg %5.1f  %4.1f kn  thr %4.1f %%  alarms 0x%03x\n", x->t * 0.001,
                   mode_name(sb_mode(&x->sb)), x->sb.ap.heading, x->w.speed_kn, x->sb.thr.out_pct,
                   (unsigned)st.alarm_bits);
            fflush(stdout);
        }
        sleep_ms(2);
    }
}
