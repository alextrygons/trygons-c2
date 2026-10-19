/*
 * nav.c - Waypoint line following geometry. See nav.h.
 */
#include "nav.h"

#include <math.h>

#define M_PER_DEG_LAT  111320.0
#define DEG2RAD        0.017453292519943295
#define RAD2DEG_F      57.29578f

static float wrap360(float d)
{
    d = fmodf(d, 360.0f);
    return (d < 0.0f) ? d + 360.0f : d;
}

void nav_to_grid(const nav_leg_t *l, double lat, double lon, float *n, float *e)
{
    *n = (float)((lat - l->lat0) * M_PER_DEG_LAT);
    *e = (float)((lon - l->lon0) * M_PER_DEG_LAT * l->cos_lat0);
}

void nav_leg_init(nav_leg_t *l, double lat0, double lon0, double wp_lat, double wp_lon)
{
    l->lat0 = lat0;
    l->lon0 = lon0;
    l->cos_lat0 = cos(lat0 * DEG2RAD);
    nav_to_grid(l, wp_lat, wp_lon, &l->wp_n, &l->wp_e);
    l->length_m = sqrtf(l->wp_n * l->wp_n + l->wp_e * l->wp_e);
    l->track_deg = wrap360(atan2f(l->wp_e, l->wp_n) * RAD2DEG_F);
    nav_leg_update(l, lat0, lon0);
}

void nav_leg_update(nav_leg_t *l, double lat, double lon)
{
    float n, e;
    nav_to_grid(l, lat, lon, &n, &e);
    float tn = (l->length_m > 0.1f) ? l->wp_n / l->length_m : 1.0f;   /* unit track vector */
    float te = (l->length_m > 0.1f) ? l->wp_e / l->length_m : 0.0f;
    l->along_m = n * tn + e * te;
    l->xte_m = e * tn - n * te;            /* right of track positive */
    float dn = l->wp_n - n, de = l->wp_e - e;
    l->dist_m = sqrtf(dn * dn + de * de);
    l->bearing_deg = wrap360(atan2f(de, dn) * RAD2DEG_F);
}

float nav_leg_course(const nav_leg_t *l, float lookahead_m)
{
    if (lookahead_m < 1.0f) lookahead_m = 1.0f;
    return wrap360(l->track_deg - atan2f(l->xte_m, lookahead_m) * RAD2DEG_F);
}
