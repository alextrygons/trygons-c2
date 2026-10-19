/*
 * nav.h - Waypoint line following (NAV mode geometry).
 *
 * Positions are turned into a flat local grid (metres north / east)
 * around the start point; fine for legs of a few km.
 *
 *   start ---------------------------> waypoint      (the track)
 *            |  cross-track error (+ = boat right of the track)
 *            boat
 *
 * Desired course over ground = track bearing - atan(xte / lookahead):
 * the boat aims at a point `lookahead` metres ahead on the line.
 */
#ifndef NAV_H
#define NAV_H

#include <stdbool.h>

typedef struct {
    double lat0, lon0;          /* start (grid origin) */
    double cos_lat0;
    float  wp_n, wp_e;          /* waypoint in the grid, m */
    float  track_deg;           /* start -> waypoint bearing */
    float  length_m;
    /* last update */
    float  xte_m;               /* cross-track error, + = right of track */
    float  along_m;             /* distance along the track from start */
    float  dist_m;              /* straight distance to the waypoint */
    float  bearing_deg;         /* bearing boat -> waypoint */
} nav_leg_t;

void nav_leg_init(nav_leg_t *l, double lat0, double lon0, double wp_lat, double wp_lon);

/* Update with the boat position. */
void nav_leg_update(nav_leg_t *l, double lat, double lon);

/* Course over ground that brings the boat onto the track. */
float nav_leg_course(const nav_leg_t *l, float lookahead_m);

/* Grid position of a lat/lon (m north, m east of the start). */
void nav_to_grid(const nav_leg_t *l, double lat, double lon, float *n, float *e);

#endif /* NAV_H */
