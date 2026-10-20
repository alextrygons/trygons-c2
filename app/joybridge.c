/*
 * joybridge.c - USB joystick source (Teensy bridge). See joybridge.h.
 */
#include "joybridge.h"

#include <string.h>

#include "hal.h"

void jb_default_config(jb_config_t *c)
{
    c->timeout_ms = 150u;
    c->crc_window_ms = 1000u;
    c->crc_max = 5u;
    c->seq_stuck = 3u;
    c->boot_grace_ms = 5000u;
}

void jb_init(jb_t *b, const jb_config_t *c, uint32_t now_ms)
{
    memset(b, 0, sizeof *b);
    b->cfg = *c;
    jsf_rx_init(&b->rx);
    b->start_ms = b->crc_win_start = b->fps_t0 = now_ms;
    b->grace = true;
    jsf_dev_clear(&b->last.dev[0]);
    jsf_dev_clear(&b->last.dev[1]);
    {
        uint8_t junk[128];      /* bytes from before now are stale: drop them */
        while (hal_bridge_read(junk, (uint32_t)sizeof junk) == sizeof junk) {}
    }
}

static float axis_value(const jsf_t *f, uint8_t code, bool invert, bool *ok)
{
    int16_t v = f->dev[code / JSF_AXES].axis[code % JSF_AXES];
    if (v == JSF_AXIS_NONE || v < -32767) {
        *ok = false;
        return 0.0f;
    }
    float x = (float)v / 32767.0f;
    return invert ? -x : x;
}

static uint32_t panel_buttons(const jsf_t *f, const joymap_t *m)
{
    static const struct { jm_button_t jm; hal_button_t hal; } MAP[] = {
        { JM_STANDBY, HAL_BTN_STANDBY }, { JM_HDG_LOCK, HAL_BTN_HDG_LOCK },
        { JM_MINUS_1, HAL_BTN_MINUS_1 }, { JM_PLUS_1, HAL_BTN_PLUS_1 },
        { JM_MINUS_10, HAL_BTN_MINUS_10 }, { JM_PLUS_10, HAL_BTN_PLUS_10 },
    };
    uint32_t b = 0u;
    for (unsigned i = 0; i < sizeof MAP / sizeof MAP[0]; i++)
        if (jsf_code_pressed(f, m->btn[MAP[i].jm])) b |= HAL_BTN_BIT(MAP[i].hal);
    return b;
}

void jb_step(jb_t *b, const joymap_t *m, uint32_t now_ms, joy_input_t *out)
{
    const jb_config_t *c = &b->cfg;
    uint8_t buf[128];
    uint32_t n;
    jsf_t f;

    /* read and parse in chunks: ~5 frames (285 bytes) arrive per 50 ms step */
    do {
        n = hal_bridge_read(buf, (uint32_t)sizeof buf);
        jsf_rx_push(&b->rx, buf, n);
        while (jsf_rx_next(&b->rx, &f)) {
            if (b->have && f.seq == b->last.seq) {
                if (b->same_seq < 255u) b->same_seq++;
            } else {
                b->same_seq = 0u;
                b->last_ok_ms = now_ms;     /* fresh frame (a repeated counter is not fresh) */
            }
            b->last = f;
            b->have = true;
            b->fps_n++;
        }
    } while (n == sizeof buf);

    /* frames per second (display) */
    if (now_ms - b->fps_t0 >= 1000u) {
        b->fps = (uint16_t)(b->fps_n > 65535u ? 65535u : b->fps_n);
        b->fps_n = 0u;
        b->fps_t0 = now_ms;
    }

    /* CRC errors in a window */
    if (now_ms - b->crc_win_start >= c->crc_window_ms) {
        b->crc_win_start = now_ms;
        b->crc_in_win = 0u;
    }
    b->crc_in_win += b->rx.crc_errors - b->crc_seen;
    b->crc_seen = b->rx.crc_errors;

    /* checks, most basic first */
    const jsf_t *L = &b->last;
    uint8_t need = joymap_devices(m);
    uint8_t why = 0u;
    if (!b->have || now_ms - b->last_ok_ms > c->timeout_ms) why = JOY_WHY_TIMEOUT;
    else if (b->crc_in_win > c->crc_max) why = JOY_WHY_CRC;
    else if (b->same_seq >= c->seq_stuck) why = JOY_WHY_SEQ;
    else if (L->flags & JSF_F_LEARN) why = JOY_WHY_LEARN;
    else if ((need & 1u) && !(L->flags & JSF_F_DEV0)) why = JOY_WHY_DEV0;
    else if ((need & 2u) && !(L->flags & JSF_F_DEV1)) why = JOY_WHY_DEV1;

    memset(out, 0, sizeof *out);
    out->present = true;
    out->has_lever = (m->thr_input == JOY_THR_LEVER);
    out->lever = 0.5f * (m->nz_lo + m->nz_hi);
    if (why == 0u) {
        bool ok = true;
        out->x = axis_value(L, m->axis_steer, m->inv_steer != 0u, &ok);
        float t = axis_value(L, m->axis_thr, m->inv_thr != 0u, &ok);
        if (!ok) why = JOY_WHY_RANGE;
        else if (out->has_lever) out->lever = 0.5f * (t + 1.0f);
        else out->y = t;
    }
    if (why != 0u) out->x = out->y = 0.0f;

    /* grace only until the bridge has once been seen fully working */
    if (why == 0u || now_ms - b->start_ms >= c->boot_grace_ms) b->grace = false;
    b->why = why;
    out->ready = !(why != 0u && b->grace);
    out->bad = why != 0u;
    out->why = why;
    if (why == 0u) {
        out->take = jsf_code_pressed(L, m->btn[JM_TAKE]);
        out->autob = jsf_code_pressed(L, m->btn[JM_AUTO]);
        out->confirm = jsf_code_pressed(L, m->btn[JM_CONFIRM]);
        out->save = jsf_code_pressed(L, m->btn[JM_SAVE]);
        out->buttons = panel_buttons(L, m);
    }
}
