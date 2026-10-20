/*
 * joystick.c - Joystick: analog source (calibration, wire break), and the
 * common part for every source (faults, arming, engage, lever, buttons).
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
    c->nz_lo = 0.45f;
    c->nz_hi = 0.55f;
    c->lever_delta = 0.10f;
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

static float nz_mid(const joy_t *j)
{
    return 0.5f * (j->cfg.nz_lo + j->cfg.nz_hi);
}

bool joy_lever_neutral(const joy_t *j, float lever)
{
    return lever >= j->cfg.nz_lo && lever <= j->cfg.nz_hi;
}

bool joy_stick_centered(const joy_t *j)
{
    return j->present && !j->fault &&
           fabsf(j->x) < j->cfg.center_tol && fabsf(j->y) < j->cfg.center_tol;
}

bool joy_centered(const joy_t *j)
{
    return joy_stick_centered(j) && (!j->has_lever || joy_lever_neutral(j, j->lever));
}

void joy_lever_ref_here(joy_t *j)
{
    j->lever_ref = j->lever_in;
    j->deflect_l_ms = 0u;
    j->lever_moved = false;
}

/* Analog source: wire break / short (raw at the rails), impossible jump
 * between steps, calibration -> joy_update. */
void joy_step(joy_t *j, const hal_joystick_t *in, uint32_t dt_ms)
{
    const joy_config_t *c = &j->cfg;
    joy_input_t u;

    memset(&u, 0, sizeof u);
    u.present = (in != NULL);
    u.ready = true;
    if (in) {
        uint16_t raw[2] = { in->x_raw, in->y_raw };
        for (int a = 0; a < 2; a++) {
            bool out = raw[a] < c->fault_low || raw[a] > c->fault_high;
            bool jump = j->have_last && (uint16_t)abs((int)raw[a] - (int)j->raw[a]) > c->jump_max;
            if ((out || jump) && !u.bad) {
                u.bad = true;
                u.why = (uint8_t)(a == 0 ? JOY_WHY_X : JOY_WHY_Y);
            }
            if (j->cal_active) {
                if (raw[a] < j->cal_min[a]) j->cal_min[a] = raw[a];
                if (raw[a] > j->cal_max[a]) j->cal_max[a] = raw[a];
            }
        }
        j->raw[0] = raw[0];
        j->raw[1] = raw[1];
        j->have_last = true;
        u.x = norm_axis(&j->cal, 0, raw[0]);
        u.y = norm_axis(&j->cal, 1, raw[1]);
        u.take = in->take;
        u.autob = in->autob;
        u.confirm = in->confirm;
    }
    joy_update(j, &u, dt_ms);
}

void joy_update(joy_t *j, const joy_input_t *in, uint32_t dt_ms)
{
    const joy_config_t *c = &j->cfg;
    bool was_take = j->take, was_auto = j->autob, was_confirm = j->confirm, was_save = j->save;
    uint32_t was_buttons = j->buttons;

    j->engage_x = j->engage_y = false;
    j->take_edge = j->auto_edge = j->confirm_edge = j->save_edge = false;
    j->buttons_edge = 0u;
    j->lever_moved = false;
    j->present = in->present;
    j->has_lever = in->present && in->has_lever;
    if (!in->present || !in->ready) {
        /* not fitted / still starting: not usable, no fault (yet) */
        j->armed = false;
        j->centered_ms = 0u;
        j->x = j->y = 0.0f;
        j->lever = nz_mid(j);
        j->lever_in = in->has_lever ? in->lever : nz_mid(j);
        j->take = j->autob = j->confirm = j->save = false;
        j->buttons = 0u;
        j->deflect_x_ms = j->deflect_y_ms = j->deflect_l_ms = 0u;
        return;
    }

    bool bad = in->bad;
    if (bad && !j->fault) j->fault_axis = in->why;
    j->take = in->take;
    j->autob = in->autob;
    j->confirm = in->confirm;
    j->save = in->save;
    j->save_edge = j->save && !was_save;
    j->buttons = in->buttons;
    j->take_edge = j->take && !was_take;
    j->auto_edge = j->autob && !was_auto;
    j->confirm_edge = j->confirm && !was_confirm;
    j->buttons_edge = j->buttons & ~was_buttons;
    if (!bad) j->lever_in = j->has_lever ? in->lever : nz_mid(j);

    float x = in->x, y = in->y;
    bool centered_now = !bad && fabsf(x) < c->center_tol && fabsf(y) < c->center_tol &&
                        (!j->has_lever || joy_lever_neutral(j, in->lever));

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
                joy_lever_ref_here(j);
            } else if (!centered_now) {
                j->not_centered = true;     /* power-up / re-arm: stick / lever not in neutral */
            }
        }
    }

    if (j->fault || !j->armed) {
        j->x = j->y = 0.0f;
        j->lever = nz_mid(j);
        j->deflect_x_ms = j->deflect_y_ms = j->deflect_l_ms = 0u;
        j->take_edge = false;                   /* buttons only count when armed ... */
        j->buttons_edge &= HAL_BTN_BIT(HAL_BTN_STANDBY);   /* ... except STANDBY (always safe) */
        return;
    }
    j->x = x;
    j->y = y;
    j->lever = j->has_lever ? in->lever : nz_mid(j);

    j->deflect_x_ms = (fabsf(x) > c->deadband) ? j->deflect_x_ms + dt_ms : 0u;
    j->deflect_y_ms = (fabsf(y) > c->deadband) ? j->deflect_y_ms + dt_ms : 0u;
    j->engage_x = j->deflect_x_ms >= c->engage_ms;
    j->engage_y = j->deflect_y_ms >= c->engage_ms;
    j->deflect_l_ms = (j->has_lever && fabsf(j->lever - j->lever_ref) > c->lever_delta) ? j->deflect_l_ms + dt_ms : 0u;
    j->lever_moved = j->deflect_l_ms >= c->engage_ms;
}

bool joy_confirm(const joy_t *j)
{
    return j->confirm_edge && j->armed && joy_stick_centered(j);
}
void joy_disarm(joy_t *j)
{
    j->armed = false;
    j->centered_ms = 0u;
    j->x = j->y = 0.0f;
    j->deflect_x_ms = j->deflect_y_ms = j->deflect_l_ms = 0u;
    j->lever = 0.5f * (j->cfg.nz_lo + j->cfg.nz_hi);
    j->lever_moved = false;
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
