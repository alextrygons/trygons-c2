/*
 * logcore0.h - What core 0 logs (producer side, never blocks).
 *
 * logcore0_step() after every control step: one LOGR_DATA sample (the
 * fields in logfields.h) and events for everything that changed since the
 * last step (mode changes and refused requests, alarms on / off, buttons,
 * power-limit changes and raise attempts, training, GPS state, faults,
 * AUTO program, CONFIRM, config changes). All of it goes into g_logq0;
 * a full queue drops and counts.
 */
#ifndef LOGCORE0_H
#define LOGCORE0_H

#include <stdint.h>

#include "smallboat.h"

/* Boot: reset reason + boot number event, config record. */
void logcore0_init(const sb_t *s, uint32_t boot_no);
void logcore0_step(const sb_t *s);
/* The ground station sent a command (link.c). */
/* Every PC / station command with its answer; station: 1 = USB laptop, 2.. = via the gateway. */
void logcore0_pc(uint32_t t_ms, uint8_t type, uint8_t seq, uint8_t result, const uint8_t *payload, uint8_t len,
                 uint8_t station);
void logcore0_helm(uint32_t t_ms, uint8_t station, uint8_t action, uint8_t refused, uint8_t holder);
/* A parameter changed (link.c, before it is written). */
void logcore0_param(const sb_t *s, uint8_t id, float old_v, float new_v);
/* The config / calibration as text (key=value lines). */
uint16_t logcore0_config_text(const sb_t *s, char *out, uint16_t max);

#endif /* LOGCORE0_H */
