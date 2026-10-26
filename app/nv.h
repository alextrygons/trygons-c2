/*
 * nv.h - App side of the non-volatile storage (hal_nv_*: data flash on the
 * board, files in the sim).
 *
 * Every record type keeps its own version + checksum (calib.c CRC-32,
 * joymap.c CRC-32, program.c CRC-32, joystick.c calibration check): a record
 * that loads but fails that check = DEFAULTS, never a half-read record, and
 * the notice NV DATA RESET (TELEM2 group I, log event 23 NV). A save only
 * queues; the flash write runs in STANDBY (not stopping) through nv_step().
 * NV FAULT (the flash does not work) = notice too; nothing is saved, loads
 * keep working. The power limit is NEVER saved.
 */
#ifndef NV_H
#define NV_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t reset_mask;    /* bit per slot: its stored record was unusable -> defaults in use */
    uint16_t resets;        /* how many records were refused since power-up */
    bool     fault;         /* NV FAULT: the data flash does not work (nothing is saved) */
    bool     pending;       /* a save waits for STANDBY / the flash */
    uint32_t pending_ms;    /* how long the oldest wait lasts (0 = none) */
} nv_status_t;

void nv_init(void);                                         /* power-up (sb_init) */
bool nv_load(uint8_t slot, void *data, uint32_t size);      /* true = a record of that size was read */
void nv_invalid(uint8_t slot);                              /* its version / checksum check failed: defaults */
bool nv_save(uint8_t slot, const void *data, uint32_t size);/* queue (false: refused / NV FAULT) */
void nv_step(bool standby_idle, uint32_t dt_ms);            /* every control step */
const nv_status_t *nv_status(void);

#endif /* NV_H */
