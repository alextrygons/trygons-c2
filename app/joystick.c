/*
 * joystick.c - Hard-wired joystick: calibration, faults, arming, engage.
 * See joystick.h.
 */
#include "joystick.h"

#include <math.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

void joy_default_config(joy_config_t *c)
{
    c->deadband = 0.15f;
    c->engage_ms = 100u;
    c->center_tol = 0.10f;
    c->center_ms = 200u;
    c->fault_low = 100u;
    c->fault_high = 3995u;
    c->jump_max = 2600u;
    c->fault_clear_ms = 2000u;
}

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

void joy_cal_seal(joy_cal_t *cal)
{
    cal->magic = JOY_CAL_MAGIC;
    cal->version = JOY_CAL_VERSION;
    cal->size = (uint32_t)sizeof *cal;
    cal->checksum = crc32((const uint8_t *)cal, offsetof(joy_cal_t, checksum));
}

bool joy_cal_valid(const joy_cal_t *cal)
{
    if (cal->magic != JOY_CAL_MAGIC || cal->version != JOY_CAL_VERSION ||
        cal->size != (uint32_t)sizeof *cal ||
        cal->checksum != crc32((const uint8_t *)cal, offsetof(joy_cal_t, checksum))) return false;
    for (int a = 0; a < 2; a++) {
        /* centre must sit well inside the range, each side at least 300 counts */
        if (!(cal->min[a] + 300u < cal->center[a] && cal->center[a] + 300u < cal->max[a])) return false;
    }
    return true;
}

void joy_default_cal(joy_cal_t *cal)
{
    memset(cal, 0, sizeof *cal);
    for (int a = 0; a < 2; a++) {
        cal->center[a] = 2048u;
        cal->min[a] = 341u;         /* 1 k + 10 k pot + 1 k at 3.3 V: 1/12 .. 11/12 */
        cal->max[a] = 3754u;
    }
    joy_cal_seal(cal);
}

void joy_init(joy_t *j, const joy_config_t *c, const joy_cal_t *cal)
{
    memset(j, 0, sizeof *j);
    j->cfg = *c;
    if (cal && joy_cal_valid(cal)) j->cal = *cal;
    else joy_default_cal(&j->cal);
}

static float norm_axis(const joy_cal_t *cal, int a, uint16_t raw)
{
    float v = (float)raw - (float)cal->center[a];
    float span = (v >= 0.0f) ? (float)(cal->max[a] - cal->center[a]) : (float)(cal->center[a] - cal->min[a]);
    if (span < 1.0f) return 0.0f;
    v /= span;
    return (v > 1.0f) ? 1.0f : (v < -1.0f) ? -1.0f : v;
}

bool joy_centered(const joy_t *j)
{
    return j->present && !j->fault &&
           fabsf(j->x) < j->cfg.center_tol && fabsf(j->y) < j->cfg.center_tol;
}

void joy_step(joy_t *j, const hal_joystick_t *in, uint32_t dt_ms)
{
    const joy_config_t *c = &j->cfg;
    bool was_take = j->take, was_auto = j->autob, was_confirm = j->confirm;

    j->engage_x = j->engage_y = false;
    j->take_edge = j->auto_edge = j->confirm_edge = false;
    j->present = (in != NULL);
    if (!in) {
        j->armed = false;
        j->x = j->y = 0.0f;
        return;
    }

    /* wire break / short: raw at the rails; impossible jump between steps */
    bool bad = false;
    uint16_t raw[2] = { in->x_raw, in->y_raw };
    for (int a = 0; a < 2; a++) {
        bool out = raw[a] < c->fault_low || raw[a] > c->fault_high;
        bool jump = j->have_last && (uint16_t)abs((int)raw[a] - (int)j->raw[a]) > c->jump_max;
        if (out || jump) {
            bad = true;
            if (!j->fault) j->fault_axis = (uint8_t)(a + 1);
        }
        if (j->cal_active) {
            if (raw[a] < j->cal_min[a]) j->cal_min[a] = raw[a];
            if (raw[a] > j->cal_max[a]) j->cal_max[a] = raw[a];
        }
    }
    j->raw[0] = raw[0];
    j->raw[1] = raw[1];
    j->have_last = true;
    j->take = in->take;
    j->autob = in->autob;
    j->confirm = in->confirm;
    j->take_edge = in->take && !was_take;
    j->auto_edge = in->autob && !was_auto;
    j->confirm_edge = in->confirm && !was_confirm;

    float x = norm_axis(&j->cal, 0, raw[0]);
    float y = norm_axis(&j->cal, 1, raw[1]);
    bool centered_now = !bad && fabsf(x) < c->center_tol && fabsf(y) < c->center_tol;

    if (bad) {
        j->fault = true;
        j->armed = false;
        j->valid_ms = 0u;
    } else if (j->fault) {
        /* clears only after valid and centred for a while */
        j->valid_ms = centered_now ? j->valid_ms + dt_ms : 0u;
        if (j->valid_ms >= c->fault_clear_ms) j->fault = false;
    }

    if (!j->fault) {
        if (!j->armed) {
            j->centered_ms = centered_now ? j->centered_ms + dt_ms : 0u;
            if (j->centered_ms >= c->center_ms) {
                j->armed = true;
                j->not_centered = false;
            } else if (!centered_now) {
                j->not_centered = true;     /* power-up / re-arm: stick not centred */
            }
        }
    }

    if (j->fault || !j->armed) {
        j->x = j->y = 0.0f;
        j->deflect_x_ms = j->deflect_y_ms = 0u;
        j->take_edge = false;                   /* buttons only count when armed */
        return;
    }
    j->x = x;
    j->y = y;

    j->deflect_x_ms = (fabsf(x) > c->deadband) ? j->deflect_x_ms + dt_ms : 0u;
    j->deflect_y_ms = (fabsf(y) > c->deadband) ? j->deflect_y_ms + dt_ms : 0u;
    j->engage_x = j->deflect_x_ms >= c->engage_ms;
    j->engage_y = j->deflect_y_ms >= c->engage_ms;
}

bool joy_confirm(const joy_t *j)
{
    return j->confirm_edge && j->armed && joy_centered(j);
}

void joy_disarm(joy_t *j)
{
    j->armed = false;
    j->centered_ms = 0u;
    j->x = j->y = 0.0f;
    j->deflect_x_ms = j->deflect_y_ms = 0u;
}

void joy_cal_start(joy_t *j)
{
    j->cal_active = true;
    for (int a = 0; a < 2; a++) {
        j->cal_min[a] = j->raw[a];
        j->cal_max[a] = j->raw[a];
    }
}

bool joy_cal_finish(joy_t *j, joy_cal_t *out)
{
    if (!j->cal_active) return false;
    j->cal_active = false;
    joy_cal_t c;
    memset(&c, 0, sizeof c);
    for (int a = 0; a < 2; a++) {
        c.center[a] = j->raw[a];            /* stick released now */
        c.min[a] = j->cal_min[a];
        c.max[a] = j->cal_max[a];
    }
    joy_cal_seal(&c);
    if (!joy_cal_valid(&c)) return false;
    j->cal = c;
    *out = c;
    return true;
}
