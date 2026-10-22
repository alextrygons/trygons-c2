/*
 * logrec.c - Log record format. See logrec.h.
 */
#include "logrec.h"

#include <stdio.h>
#include <string.h>

#include "modes.h"
#include "proto.h"

uint16_t logrec_make(uint8_t *out, uint8_t type, uint32_t t_ms, const void *payload, uint16_t len)
{
    out[0] = LOGR_SYNC;
    out[1] = type;
    out[2] = (uint8_t)(len & 0xFFu);
    out[3] = (uint8_t)(len >> 8);
    out[4] = (uint8_t)(t_ms & 0xFFu);
    out[5] = (uint8_t)((t_ms >> 8) & 0xFFu);
    out[6] = (uint8_t)((t_ms >> 16) & 0xFFu);
    out[7] = (uint8_t)(t_ms >> 24);
    if (len) memcpy(&out[LOGR_HDR], payload, len);
    uint16_t crc = proto_crc16(&out[1], (size_t)len + 7u);
    out[LOGR_HDR + len] = (uint8_t)(crc & 0xFFu);
    out[LOGR_HDR + len + 1u] = (uint8_t)(crc >> 8);
    return (uint16_t)(len + LOGR_OVERHEAD);
}

static void put_i32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24);
}

bool logrec_event(logq_t *q, uint32_t t_ms, uint16_t code, int32_t a, int32_t b, int32_t c, int32_t d)
{
    uint8_t pl[18], rec[18 + LOGR_OVERHEAD];
    pl[0] = (uint8_t)code;
    pl[1] = (uint8_t)(code >> 8);
    put_i32(&pl[2], a);
    put_i32(&pl[6], b);
    put_i32(&pl[10], c);
    put_i32(&pl[14], d);
    uint16_t n = logrec_make(rec, LOGR_EVENT, t_ms, pl, sizeof pl);
    return logq_push(q, rec, n);
}

/* --- header text ------------------------------------------------------------ */

#define FIELD_NAME(n, t, sc, e) #n ":" #t ":" #sc ","
#include "logfields.h"
static const char k_fields[] = LOG_FIELDS(FIELD_NAME);

#define FIELD_SIZE(n, t, sc, e) + SZ_##t
#define SZ_B 1
#define SZ_c 1
#define SZ_H 2
#define SZ_h 2
#define SZ_I 4
#define SZ_i 4
#define SZ_f 4
static const uint16_t k_data_size = 0 LOG_FIELDS(FIELD_SIZE);

uint16_t logrec_data_size(void)
{
    return k_data_size;
}

static const char k_events[] =
    "1:BOOT:reset,raw,boot,format;"
    "2:MODE:from,to,why,verdict;"
    "3:ALARM_ON:new_bits,all_bits,ap_alarm,-;"
    "4:ALARM_OFF:cleared_bits,all_bits,-,-;"
    "5:BUTTON:panel_mask,-,-,-;"
    "6:JOY_BUTTON:pressed(1 TAKE 2 AUTO 4 CONFIRM),x*1000,y*1000,armed;"
    "7:POWER_LIMIT:src|res<<8|attempt<<16|dir<<24,fwd_old*10,fwd_new*10,rev_old*10<<16|rev_new*10;"
    "8:TRAINING:stage,phase,result,-;"
    "9:CONFIG:what,id,old*1000,new*1000;"
    "10:GPS:pos_source,reason,unc*10,losses;"
    "11:FAULT:kind,a,b,c;"
    "12:PROGRAM:action,a,b,c;"
    "13:CONFIRM:kind,result,count,-;"
    "14:MONITOR:reason,throttle*10,limit*10,-;"
    "15:LOG:what,a,b,c;"
    "16:TIME:date,time_of_day_ms,t_ms,-;"
    "17:HELM:station,action(0 take 1 release 2 lost 3 auto 4 gw lost),refused,holder;"
    "18:SAVE:trigger(1 button 2 alarm 3 fault 4 takeover 5 standby fault 6 slam),arg,seq,-;"
    "19:SLAM:peak_g*100,level_g*100,impacts_per_min,cap%*10|state<<16|override<<20;"
    "20:NAV:kind|reason<<8|alarm<<16|lost<<24|applied<<25,heading*100,cap_kn*10,seq;";

static const char k_alarms[] =
    "1:COMPASS,2:GPS_LOST,4:GPS_SUSPECT,8:DR_LIMIT,16:ARRIVAL_UNCERTAIN,32:ARRIVED,64:GEAR,128:TRAINING,"
    "256:JOY_FAULT,512:JOY_NOT_CENTERED,1024:JOYSTICK,2048:PC_LINK,4096:NOT_ARMED,8192:MONITOR,"
    "16384:OVERRUN,32768:LOG_FAILURE,65536:GATEWAY_LOST,131072:GATEWAY_STORAGE,262144:REMOTE_LINK_LOST,524288:CHANNEL_FALLBACK,1048576:REMOTE_DRIVING_LINK_LOST,2097152:SLAMMING,4194304:AIS_RECEIVER_SILENT,8388608:COLLISION_RISK,16777216:DRIVER_DECISION,33554432:SHALLOW_WATER,67108864:OBSTACLE_AHEAD";

uint16_t logrec_header_text(char *out, uint16_t max, uint32_t boot, uint32_t file_no)
{
    int n = snprintf(out, max, "TLG header\nformat=%u\nboot=%lu\nfile=%lu\nfields=%s\nevents=%s\nalarms=%s\nmodes=",
                     (unsigned)LOG_FORMAT_VERSION, (unsigned long)boot, (unsigned long)file_no, k_fields, k_events,
                     k_alarms);
    for (int m = 0; m < (int)MODE_COUNT && n > 0 && n < (int)max; m++)
        n += snprintf(out + n, (size_t)(max - n), "%s%s", m ? "," : "", mode_name((sb_mode_t)m));
    if (n > 0 && n < (int)max) n += snprintf(out + n, (size_t)(max - n), "\nwhy=");
    for (int w = 0; w < (int)WHY_COUNT && n > 0 && n < (int)max; w++)
        n += snprintf(out + n, (size_t)(max - n), "%s%s", w ? "," : "", mode_reason_name((mode_reason_t)w));
    if (n > 0 && n < (int)max) n += snprintf(out + n, (size_t)(max - n), "\n");
    return (n < 0) ? 0u : (uint16_t)((n < (int)max) ? n : (int)max - 1);
}
