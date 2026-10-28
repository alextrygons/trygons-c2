/*
 * can_act.c - The CAN output layer of the servos (see can_act.h).
 */
#include "can_act.h"

#include <math.h>
#include <string.h>

#include "dronecan.h"
#include "hal.h"

static float    s_thr = -1.0f;          /* throttle wire value: IDLE from the boot */
static bool     s_gear_on, s_steer_on;  /* a frame is commanded */
static float    s_gear, s_steer;
static ca_state_t s_st;

static float clamp1(float v)
{
    if (!(v >= -1.0f)) return -1.0f;    /* NaN / below: -1 (for the throttle that is IDLE) */
    if (v > 1.0f) return 1.0f;
    return v;
}

float ca_us_to_wire(uint16_t us)
{
    return clamp1(((float)us - 1000.0f) / 500.0f - 1.0f);
}

float ca_wire_to_us(float wire)
{
    return (clamp1(wire) + 1.0f) * 500.0f + 1000.0f;
}

void ca_init(void)
{
    s_thr = -1.0f;
    s_gear_on = s_steer_on = false;
    s_gear = s_steer = 0.0f;
    memset(&s_st, 0, sizeof s_st);
    s_st.thr_wire = -1.0f;
}

void ca_set_throttle_us(uint16_t us)
{
    s_thr = ca_us_to_wire(us);
}

void ca_set_gear_us(uint16_t us)
{
    if (us == 0u) {                     /* hold: no frame (the node keeps its last position) */
        s_gear_on = false;
        return;
    }
    s_gear_on = true;
    s_gear = ca_us_to_wire(us);
}

void ca_set_steer(bool command, float pos)
{
    if (!command || !isfinite(pos)) {
        s_steer_on = false;
        return;
    }
    s_steer_on = true;
    s_steer = clamp1(CA_STEER_SIGN * pos);
}

static void send(uint8_t act, float v)
{
    if (dc_send_actuator(act, DC_CMD_UNITLESS, v)) s_st.frames++;
    else s_st.refused++;
}

void ca_tick(void)
{
    s_st.ticks++;
    if (hal_outputs_forced()) return;   /* CPU1 tripped: its safe frames own the actuators (latched) */
    send(CA_ACT_THROTTLE, s_thr);       /* (the first frame after a boot: throttle IDLE) */
    s_st.thr_wire = s_thr;
    if (s_gear_on) {
        send(CA_ACT_GEAR, s_gear);
        s_st.gear_have = true;
        s_st.gear_wire = s_gear;
    }
    if (s_steer_on) {
        send(CA_ACT_STEERING, s_steer);
        s_st.steer_have = true;
        s_st.steer_wire = s_steer;
    }
    dc_tx_flush();
    if (hal_outputs_forced()) {         /* the latch came in between: our frames must not overwrite CPU1's */
        dc_cancel_tx();
        s_st.cancelled++;
    }
}

uint32_t ca_ticks(void)
{
    return s_st.ticks;
}

void ca_get(ca_state_t *st)
{
    *st = s_st;
}
