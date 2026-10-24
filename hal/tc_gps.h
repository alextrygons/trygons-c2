/*
 * tc_gps.h - u-blox MAX-M10S on a UART: UBX NAV-PVT (1 Hz, the fix),
 * NAV-STATUS (spoofing detection), MON-RF (jamming); NMEA RMC as fallback if
 * the module ignores the configuration. No message for 3 s = "no data".
 */
#ifndef TC_GPS_H
#define TC_GPS_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

typedef struct {
    bool     ok;                    /* a navigation message within the last 3 s */
    bool     ubx;                   /* UBX messages seen (configured), else NMEA */
    uint32_t pvt, rmc, status, monrf, bad_checksum, configs;
    uint32_t last_msg_ms;
    uint8_t  spoof_state;           /* NAV-STATUS spoofDetState: 0 unknown 1 none 2 indicated 3 multiple */
    uint8_t  jam_state;             /* MON-RF jammingState: 0 unknown 1 ok 2 warning 3 critical */
    uint8_t  sats, fix_type;
} gps_stat_t;

void gps_init(void);
bool gps_poll(hal_gps_t *fix);      /* true = a new fix message (call once per step) */
const gps_stat_t *gps_stat(void);

#endif /* TC_GPS_H */
