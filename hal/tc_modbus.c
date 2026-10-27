/*
 * tc_modbus.c - Modbus RTU master for the RS485 relay board. See tc_modbus.h.
 * Pure C99 over a port: the board (hal_tc375.c) and the PC simulation
 * (sim/hal_sim.c) both use it.
 */
#include "tc_modbus.h"

#include <string.h>

#define MB_IDLE   0u
#define MB_SEND   1u
#define MB_WAIT   2u

#define F_READ_COILS   0x01u
#define F_WRITE_COIL   0x05u

typedef struct { uint8_t kind; uint8_t n; uint16_t val; } mb_req_t;
#define Q_SET    1u
#define Q_PULSE  2u

static struct {
    const mb_port_t *port;
    mb_config_t cfg;
    uint8_t  state;
    uint8_t  req[MB_FRAME_MAX];
    uint8_t  req_len, func, expect;
    uint32_t sent_ms;
    uint8_t  rx[64];
    uint8_t  rx_len;
    uint32_t init_ms, next_poll_ms;
    mb_req_t q[MB_QUEUE];
    uint8_t  q_head, q_tail;           /* free running, used = head - tail */
    uint32_t coils;                     /* last readback */
    uint32_t coils_ms;                  /* when (0 = never) */
    mb_stats_t st;
} s;

uint16_t mb_crc16(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

void mb_init(const mb_port_t *port, const mb_config_t *cfg)
{
    memset(&s, 0, sizeof s);
    s.port = port;
    s.cfg = *cfg;
    if (s.cfg.n_coils == 0u || s.cfg.n_coils > MB_COILS_MAX) s.cfg.n_coils = MB_COILS_MAX;
    if (s.cfg.slave == 0u) s.cfg.slave = 1u;
    s.init_ms = port ? port->millis() : 0u;
    s.next_poll_ms = s.init_ms;
}

static bool q_push(const mb_req_t *r)
{
    if (!s.port || (uint8_t)(s.q_head - s.q_tail) >= MB_QUEUE) { s.st.dropped++; return false; }
    s.q[s.q_head % MB_QUEUE] = *r;
    s.q_head++;
    return true;
}

bool mb_set_coil(uint8_t n, bool on)
{
    mb_req_t r = { Q_SET, n, on ? 0xFF00u : 0x0000u };
    if (n == 0u || n > s.cfg.n_coils) return false;
    return q_push(&r);
}

bool mb_pulse_coil(uint8_t n, uint16_t ms)
{
    uint32_t units = ((uint32_t)ms + 50u) / 100u;             /* 100 ms units, at least 1 */
    if (n == 0u || n > s.cfg.n_coils || ms == 0u) return false;
    if (units < 1u) units = 1u;
    if (units > 255u) units = 255u;
    mb_req_t r = { Q_PULSE, n, (uint16_t)(0x0200u | units) };
    return q_push(&r);
}

bool mb_coil(uint8_t n, bool *on)
{
    if (!s.port || n == 0u || n > s.cfg.n_coils || s.coils_ms == 0u) return false;
    if (s.port->millis() - s.coils_ms >= s.cfg.lost_ms) return false;
    *on = (s.coils >> (n - 1u)) & 1u;
    return true;
}

bool mb_ok(void)
{
    if (!s.port) return false;
    uint32_t ref = s.st.last_reply_ms ? s.st.last_reply_ms : s.init_ms;
    return s.port->millis() - ref < s.cfg.lost_ms;
}

bool mb_idle(void)
{
    return s.state == MB_IDLE && s.q_head == s.q_tail;
}

const mb_stats_t *mb_stats(void) { return &s.st; }

static void frame_crc(uint8_t *f, uint8_t n)
{
    uint16_t c = mb_crc16(f, n);
    f[n] = (uint8_t)c;
    f[n + 1u] = (uint8_t)(c >> 8);
}

/* build the next request from the queue, or a readback if one is due */
static bool next_request(uint32_t now)
{
    uint8_t *f = s.req;
    if (s.q_head != s.q_tail) {
        mb_req_t r = s.q[s.q_tail % MB_QUEUE];
        s.q_tail++;
        f[0] = s.cfg.slave;
        f[1] = F_WRITE_COIL;
        f[2] = 0u; f[3] = (uint8_t)(r.n - 1u);
        f[4] = (uint8_t)(r.val >> 8); f[5] = (uint8_t)r.val;
        frame_crc(f, 6u);
        s.req_len = 8u;
        s.expect = 8u;                                              /* echo */
        s.func = f[1];
        return true;
    }
    if ((int32_t)(now - s.next_poll_ms) >= 0) {
        s.next_poll_ms = now + s.cfg.poll_ms;
        f[0] = s.cfg.slave;
        f[1] = F_READ_COILS;
        f[2] = 0u; f[3] = 0u;
        f[4] = 0u; f[5] = s.cfg.n_coils;
        frame_crc(f, 6u);
        s.req_len = 8u;
        s.expect = (uint8_t)(5u + (s.cfg.n_coils + 7u) / 8u);
        s.func = F_READ_COILS;
        return true;
    }
    return false;
}

static void flush_rx(void)
{
    uint8_t junk[64];
    while (s.port->read(junk, sizeof junk) > 0u) { }
}

static void reply_good(uint32_t now)
{
    s.st.replies++;
    s.st.last_reply_ms = now ? now : 1u;
    if (s.func == F_READ_COILS) {
        uint8_t nb = s.rx[2];
        uint32_t v = 0u;
        for (uint8_t i = 0; i < nb && i < 4u; i++) v |= (uint32_t)s.rx[3u + i] << (8u * i);
        s.coils = v;
        s.coils_ms = now ? now : 1u;
    } else if (s.func == F_WRITE_COIL) {
        uint8_t n = (uint8_t)(s.rx[3] + 1u);
        uint16_t val = (uint16_t)(((uint16_t)s.rx[4] << 8) | s.rx[5]);
        if (n <= s.cfg.n_coils) {                /* keep the shadow current until the next readback */
            if (val == 0xFF00u || (val & 0xFF00u) == 0x0200u) s.coils |= 1u << (n - 1u);
            else if (val == 0u) s.coils &= ~(1u << (n - 1u));
        }
    }
}

void mb_service(void)
{
    if (!s.port) return;
    uint32_t now = s.port->millis();
    s.st.state = s.state;
    switch (s.state) {
    case MB_IDLE:
        if (!next_request(now)) return;
        flush_rx();
        s.st.requests++;
        s.rx_len = 0u;
        s.sent_ms = now;
        if (!s.port->write(s.req, s.req_len)) { s.st.dropped++; return; }
        s.state = MB_WAIT;
        return;
    case MB_WAIT: {
        uint32_t n = s.port->read(s.rx + s.rx_len, (uint32_t)(sizeof s.rx - s.rx_len));
        s.rx_len = (uint8_t)(s.rx_len + n);
        /* garbage before the reply: drop until the slave address */
        while (s.rx_len > 0u && s.rx[0] != s.cfg.slave) {
            memmove(s.rx, s.rx + 1, s.rx_len - 1u);
            s.rx_len--;
        }
        if (s.rx_len >= 5u && s.rx[1] == (uint8_t)(s.func | 0x80u)) {        /* exception reply */
            uint16_t c = (uint16_t)(s.rx[3] | ((uint16_t)s.rx[4] << 8));
            if (c == mb_crc16(s.rx, 3u)) {
                s.st.exceptions++;
                s.st.last_exception = s.rx[2];
                s.st.last_reply_ms = now ? now : 1u;        /* the board is there, it just refused */
                s.state = MB_IDLE;
                return;
            }
        }
        if (s.rx_len >= s.expect) {
            uint16_t c = (uint16_t)(s.rx[s.expect - 2u] | ((uint16_t)s.rx[s.expect - 1u] << 8));
            if (c != mb_crc16(s.rx, (uint32_t)s.expect - 2u)) {
                s.st.crc_errors++;
            } else if (s.rx[1] != s.func || (s.func != F_READ_COILS && memcmp(s.rx, s.req, 6u) != 0) ||
                       (s.func == F_READ_COILS && s.rx[2] != (uint8_t)((s.cfg.n_coils + 7u) / 8u))) {
                s.st.bad_replies++;
            } else {
                reply_good(now);
            }
            s.state = MB_IDLE;
            return;
        }
        if (now - s.sent_ms >= s.cfg.timeout_ms) {
            s.st.timeouts++;
            s.state = MB_IDLE;
        }
        return;
    }
    default:
        s.state = MB_IDLE;
        return;
    }
}
