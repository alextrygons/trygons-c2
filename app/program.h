/*
 * program.h - AUTO waypoint program: list, validation, storage.
 *
 * A program is an ordered list of up to PROG_MAX_WP waypoints. Each has a
 * position, a leg speed (speed on the way TO it), an arrival radius and an
 * action when reached (continue / slow down / stop in neutral). The final
 * action says what happens at the last waypoint (default: stop in neutral
 * + alarm).
 *
 * Uploaded from the PC (link.c), validated (program_validate), stored in
 * non-volatile memory with a CRC-32 (survives power-off).
 */
#ifndef PROGRAM_H
#define PROGRAM_H

#include <stdbool.h>
#include <stdint.h>

#define PROG_MAX_WP     50u
#define PROG_MAGIC      0x50524F47u     /* "PROG" */
#define PROG_VERSION    1u

typedef enum {
    WP_ACT_CONTINUE = 0,    /* pass through, next leg at once */
    WP_ACT_SLOW,            /* slow to the approach speed near it, then continue */
    WP_ACT_STOP             /* stop in neutral there; resume = PC + CONFIRM */
} wp_action_t;

typedef enum {
    PROG_END_STOP = 0,      /* last waypoint: idle + neutral (STANDBY) + alarm */
    PROG_END_HOLD           /* last waypoint: hold the heading at approach speed + alarm */
} prog_end_t;

typedef struct {
    int32_t lat_e7, lon_e7;     /* degrees x 1e7 */
    float   speed_kn;           /* leg speed to this waypoint */
    float   radius_m;           /* arrival radius */
    uint8_t action;             /* wp_action_t */
    uint8_t pad[3];
} prog_wp_t;

typedef struct {
    uint32_t  magic, version, size;
    uint16_t  n;                /* number of waypoints, 1..PROG_MAX_WP */
    uint8_t   final_action;     /* prog_end_t */
    uint8_t   pad;
    prog_wp_t wp[PROG_MAX_WP];
    uint32_t  checksum;         /* CRC-32 of everything above */
} program_t;

typedef struct {
    float max_speed_kn;         /* leg speeds above this are refused */
    float min_speed_kn;         /* (1) */
    float min_radius_m;         /* (5) */
    float max_radius_m;         /* (500) */
    float min_leg_m;            /* legs shorter than this (or than the radius) refused (10) */
    float max_leg_m;            /* legs longer than this refused (50 km) */
} prog_limits_t;

/* Why a program was refused. */
typedef enum {
    PROG_OK = 0,
    PROG_BAD_COUNT,             /* 0 or more than PROG_MAX_WP waypoints */
    PROG_BAD_COORD,             /* latitude / longitude out of range */
    PROG_BAD_SPEED,             /* leg speed below min or above the configured max */
    PROG_BAD_RADIUS,
    PROG_BAD_ACTION,
    PROG_LEG_SHORT,
    PROG_LEG_LONG,
    PROG_BAD_CHECKSUM
} prog_error_t;

void program_clear(program_t *p);
void program_seal(program_t *p);                    /* sets magic, size, CRC */
bool program_checksum_ok(const program_t *p);

/* Check everything; *bad_index = waypoint at fault (or 0xFF). */
prog_error_t program_validate(const program_t *p, const prog_limits_t *lim, uint8_t *bad_index);

/* Distance between two waypoints / points, m (flat earth, fine for legs < 50 km). */
float program_dist_m(double lat1, double lon1, double lat2, double lon2);
double program_lat(const prog_wp_t *w);
double program_lon(const prog_wp_t *w);

const char *program_error_text(prog_error_t e);

#endif /* PROGRAM_H */
