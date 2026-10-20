/*
 * program.c - AUTO waypoint program. See program.h.
 */
#include "program.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

void program_clear(program_t *p)
{
    memset(p, 0, sizeof *p);
}

void program_seal(program_t *p)
{
    p->magic = PROG_MAGIC;
    p->version = PROG_VERSION;
    p->size = (uint32_t)sizeof *p;
    p->checksum = crc32((const uint8_t *)p, offsetof(program_t, checksum));
}

bool program_checksum_ok(const program_t *p)
{
    return p->magic == PROG_MAGIC && p->version == PROG_VERSION && p->size == (uint32_t)sizeof *p &&
           p->checksum == crc32((const uint8_t *)p, offsetof(program_t, checksum));
}

double program_lat(const prog_wp_t *w) { return (double)w->lat_e7 * 1e-7; }
double program_lon(const prog_wp_t *w) { return (double)w->lon_e7 * 1e-7; }

float program_dist_m(double lat1, double lon1, double lat2, double lon2)
{
    const double m_per_deg = 111320.0;
    double dn = (lat2 - lat1) * m_per_deg;
    double de = (lon2 - lon1) * m_per_deg * cos(0.5 * (lat1 + lat2) * 3.14159265358979 / 180.0);
    return (float)sqrt(dn * dn + de * de);
}

prog_error_t program_validate(const program_t *p, const prog_limits_t *lim, uint8_t *bad_index)
{
    *bad_index = 0xFFu;
    if (!program_checksum_ok(p)) return PROG_BAD_CHECKSUM;
    if (p->n == 0u || p->n > PROG_MAX_WP) return PROG_BAD_COUNT;
    if (p->final_action > (uint8_t)PROG_END_HOLD) return PROG_BAD_ACTION;
    for (uint16_t i = 0; i < p->n; i++) {
        const prog_wp_t *w = &p->wp[i];
        *bad_index = (uint8_t)i;
        if (w->lat_e7 < -900000000 || w->lat_e7 > 900000000 ||
            w->lon_e7 < -1800000000 || w->lon_e7 > 1800000000 ||
            (w->lat_e7 == 0 && w->lon_e7 == 0)) return PROG_BAD_COORD;       /* 0,0 = not filled in */
        if (!(w->speed_kn >= lim->min_speed_kn && w->speed_kn <= lim->max_speed_kn)) return PROG_BAD_SPEED;
        if (!(w->radius_m >= lim->min_radius_m && w->radius_m <= lim->max_radius_m)) return PROG_BAD_RADIUS;
        if (w->action > (uint8_t)WP_ACT_STOP) return PROG_BAD_ACTION;
        if (i > 0u) {
            const prog_wp_t *a = &p->wp[i - 1u];
            float d = program_dist_m(program_lat(a), program_lon(a), program_lat(w), program_lon(w));
            if (d < lim->min_leg_m || d <= w->radius_m) return PROG_LEG_SHORT;
            if (d > lim->max_leg_m) return PROG_LEG_LONG;
        }
    }
    *bad_index = 0xFFu;
    return PROG_OK;
}

const char *program_error_text(prog_error_t e)
{
    static const char *const t[] = {
        "ok", "bad number of waypoints", "bad coordinates", "leg speed out of range",
        "arrival radius out of range", "bad action", "leg too short", "leg too long", "bad checksum"
    };
    return ((unsigned)e <= (unsigned)PROG_BAD_CHECKSUM) ? t[e] : "?";
}
