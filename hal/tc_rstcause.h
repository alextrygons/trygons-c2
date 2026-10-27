/*
 * tc_rstcause.h - Why the TC375 started: RSTSTAT (+ RSTCON) -> hal_reset_t.
 * Pure C99 (no iLLD): hal_tc375.c calls it once at boot and latches the
 * answer, then clears the cold-reset sticky bits (IfxScuRcu_clearColdResetStatus);
 * the sim tests feed it register patterns (RB13).
 *
 * The rule that matters (relay power_up, owner clarification): a WARM trigger
 * (ESR, SMU = the watchdogs, SW = the software reset, STM, Cerberus, HSM)
 * always wins over a cold bit. The iLLD's IfxScuRcu_evaluateReset turns any
 * reset into "warm power-on" while PORST is still set from the power-on, so a
 * watchdog reset would read as a power-on; here it never does.
 *
 * Bit positions = TC37x RSTSTAT / RSTCON (IfxScu_bf.h; hal_tc375.c checks them
 * at compile time). RSTCON: 2 bits per configurable trigger, 1 = system reset,
 * 2 = application reset.
 */
#ifndef TC_RSTCAUSE_H
#define TC_RSTCAUSE_H

#include <stdint.h>
#include "hal.h"

#define RST_ESR0   0u
#define RST_ESR1   1u
#define RST_SMU    3u
#define RST_SW     4u
#define RST_STM0   5u
#define RST_STM1   6u
#define RST_STM2   7u
#define RST_PORST  16u
#define RST_CB0    18u
#define RST_CB1    19u
#define RST_CB3    20u
#define RST_EVRC   23u
#define RST_EVR33  24u
#define RST_SWD    25u
#define RST_HSMS   26u
#define RST_HSMA   27u
#define RST_STBYR  28u
#define RST_LBPORST 29u
/* RSTCON field offsets of the configurable triggers */
#define RSTCON_ESR0 0u
#define RSTCON_ESR1 2u
#define RSTCON_SMU  6u
#define RSTCON_SW   8u
#define RSTCON_STM0 10u
#define RSTCON_STM1 12u
#define RSTCON_STM2 14u

static inline hal_reset_t rstcause_classify(uint32_t rststat, uint32_t rstcon)
{
    static const uint8_t trig[7][2] = {
        { RST_ESR0, RSTCON_ESR0 }, { RST_ESR1, RSTCON_ESR1 }, { RST_SMU, RSTCON_SMU }, { RST_SW, RSTCON_SW },
        { RST_STM0, RSTCON_STM0 }, { RST_STM1, RSTCON_STM1 }, { RST_STM2, RSTCON_STM2 },
    };
    for (unsigned i = 0; i < 7u; i++) {
        if ((rststat >> trig[i][0]) & 1u)
            return ((rstcon >> trig[i][1]) & 3u) == 2u ? HAL_RESET_APPLICATION : HAL_RESET_SYSTEM;
    }
    if ((rststat >> RST_CB0) & 1u) return HAL_RESET_SYSTEM;
    if ((rststat >> RST_CB1) & 1u) return HAL_RESET_DEBUG;
    if ((rststat >> RST_CB3) & 1u) return HAL_RESET_APPLICATION;
    if ((rststat >> RST_HSMS) & 1u) return HAL_RESET_SYSTEM;
    if ((rststat >> RST_HSMA) & 1u) return HAL_RESET_APPLICATION;
    if (rststat & ((1u << RST_PORST) | (1u << RST_EVRC) | (1u << RST_EVR33) | (1u << RST_SWD) |
                   (1u << RST_STBYR) | (1u << RST_LBPORST)))
        return HAL_RESET_POWER_ON;
    return HAL_RESET_OTHER;
}

#endif /* TC_RSTCAUSE_H */
