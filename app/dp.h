/*
 * dp.h - Dynamic positioning (CLAUDE.md section P, step 10): the control of
 * MODE_DP. Minn Kota style with a bow TUNNEL thruster (sideways: + = push the
 * bow to starboard; it sways AND yaws the boat) and a fixed transom electric
 * motor (fore / aft: + = ahead). Pure C99, no HAL: smallboat.c owns the mode
 * (entry with PC + CONFIRM, every exit, the faults) and sends the demands
 * through app/thruster.c (the only writer: clamp, rate limit, heat budget).
 *
 * Sub-states:
 *   SPOT    Spot-Lock: a GPS anchor at the position where it started + heading
 *           hold on the anchor heading (+-1 / +-10 from HEADING_ADJ / the
 *           course buttons, or a set heading).
 *   JOG     the anchor moved jog_m (boat file, 1.5 m) ahead / astern / to port
 *           / to starboard of the anchor heading; back to SPOT once there
 *           (within DP_JOG_DONE_M) or after DP_JOG_MAX_MS.
 *   CRUISE  heading + speed over ground set point on the stern motor
 *           (0 .. dp_max_kn, 3 kn TO CONFIRM).
 *   TRIP    the stored AUTO program (program.c validated) at a set speed (<=
 *           dp_max_kn): the line to each waypoint, the gateway's navigation
 *           request (speed cap / avoidance heading) applied as in AUTO; ends
 *           in SPOT at the last waypoint (or at a STOP waypoint: SPOT there,
 *           "continue" goes on from the next one).
 *
 * Control (every step, 20 Hz; GPS 1 Hz, propagated between fixes with the
 * GPS velocity):
 *   - position PI in the boat frame (+ velocity damping): stern = fore / aft
 *     error, bow = sideways error;
 *   - heading PD on the gyro: the bow thruster; HEADING FIRST in the bow
 *     allocation - the sideways part only gets what the heading leaves
 *     (at most DP_SWAY_SHARE of the cap) and fades out above
 *     DP_SWAY_HDG_DEG of heading error (the tunnel thruster both sways and
 *     yaws: it can't do both at once);
 *   - small dead bands (DP_DB_*) so a boat that sits on the spot doesn't
 *     use up the thrusters' heat budget for nothing.
 * Compass lost: the heading is carried on by the gyro (yaw-rate hold) - the
 * caller ends DP after DP_COMPASS_MS; no gyro either = at once.
 */
#ifndef DP_H
#define DP_H

#include <stdbool.h>
#include <stdint.h>

#include "nav.h"
#include "program.h"

/* sub-states (dp_t.sub, TELEM2 group N "dp_sub") */
#define DP_OFF       0u
#define DP_SPOT      1u
#define DP_JOG       2u
#define DP_CRUISE    3u
#define DP_TRIP      4u

/* PROTO_T_DP 0x2E (4 bytes): u8 action | u8 arg | i16 value */
#define DP_ACT_START    0u  /* arg = sub (DP_SPOT / DP_CRUISE / DP_TRIP; DP_TRIP | DP_TRIP_CONTINUE = from the active
                               waypoint), value = speed kn*100 (CRUISE / TRIP; TRIP 0 = dp_max_kn). From STANDBY:
                               joystick CONFIRM within 10 s. In DP: SPOT at once, CRUISE / TRIP again with CONFIRM */
#define DP_ACT_JOG      1u  /* arg = DP_JOG_AHEAD / ASTERN / PORT / STBD (SPOT / JOG only) */
#define DP_ACT_SPEED    2u  /* value = kn*100, 0 .. dp_max_kn (CRUISE / TRIP) */
#define DP_ACT_HEADING  3u  /* arg 0: value = heading deg*10 (0..3599); arg 1: value = change deg*10 (-1800..1800);
                               SPOT / JOG / CRUISE (TRIP steers the route) */
#define DP_ACT_STOP     4u  /* DP off: thrusters 0, STANDBY (a copilot may do it without the captain's ACCEPT) */
#define DP_TRIP_CONTINUE 0x80u
#define DP_JOG_AHEAD    0u
#define DP_JOG_ASTERN   1u
#define DP_JOG_PORT     2u
#define DP_JOG_STBD     3u

/* why DP ended (dp_t.stop_why, log event 28 what 5, TELEM2 group N) */
#define DP_STOP_NONE      0u
#define DP_STOP_CMD       1u    /* DP STOP from the station */
#define DP_STOP_STANDBY   2u    /* STANDBY button / e-stop / link lost / any other STANDBY */
#define DP_STOP_JOYSTICK  3u    /* the onboard joystick took over (CREWED): MANUAL on the outboard */
#define DP_STOP_GPS       4u    /* GPS lost / suspect (no dead reckoning in DP) */
#define DP_STOP_COMPASS   5u    /* compass lost for DP_COMPASS_MS (gyro hold meanwhile), or compass + gyro lost */
#define DP_STOP_THRUSTER  6u    /* a thruster HOT (heat budget used up) or blocked (trip / not allowed) */
#define DP_STOP_OFFSPOT   7u    /* pushed more than DP_OFF_STOP_M off the spot, or off it for DP_OFF_STOP_MS */
#define DP_STOP_FAULT     8u    /* another fault (gear, joystick, monitor ...) */

/* tuning (pure numbers, the sim tests prove them; TO CONFIRM on the water) */
#define DP_JOG_DONE_M     1.0f      /* JOG -> SPOT within this of the new spot */
#define DP_JOG_MAX_MS     30000u    /* ... or after this */
#define DP_OFF_STOP_M     50.0f     /* off spot farther than this -> STANDBY (TO CONFIRM) */
#define DP_OFF_STOP_MS    120000u   /* off spot (beyond spot_alarm_m) this long -> STANDBY (TO CONFIRM) */
#define DP_OVERPOWER_MS   10000u    /* both thrusters saturated this long -> DP OVERPOWER (advisory) */
#define DP_COMPASS_MS     10000u    /* compass lost this long -> STANDBY (gyro hold meanwhile) */
#define DP_SWAY_SHARE     0.3f      /* the sideways part of the bow: at most this share of the cap */
#define DP_SWAY_HDG_DEG   5.0f      /* ... fading to 0 at this heading error (heading first) */
#define DP_FIGHT_PCT      30.0f     /* generator check: stern astern above this ... */
#define DP_FIGHT_MS       5000u     /* ... this long while the spot error grows ahead = GEAR FAULT */
#define DP_FIGHT_GROW_M   1.0f      /* ("grows": the boat this much farther ahead of the spot than at the start) */

typedef struct {
    float    dt;                /* s */
    bool     fix_new;           /* a new trusted GPS fix this step (lat / lon / sog / cog valid) */
    double   lat, lon;
    float    sog_kn, cog_deg;
    bool     compass_ok;
    float    heading;           /* compass, deg */
    bool     gyro_ok;
    float    yaw_dps;           /* gyro, + = turning to starboard */
    float    cap_bow, cap_stern;    /* thruster caps (thruster_cap) */
    float    out_bow, out_stern;    /* last outputs (saturation, generator check) */
    float    max_kn;            /* dp_max_kn */
    float    spot_alarm_m;      /* boat file (thr_cfg_t) */
    float    nav_cap_kn;        /* TRIP: the navigation request's speed cap (< 0 none) */
    bool     nav_heading_on;    /* TRIP: the navigation request steers (avoidance / lease lost) */
    float    nav_heading;
    const program_t *prog;      /* TRIP */
} dp_in_t;

/* dp_step result */
#define DP_R_OK         0u
#define DP_R_COMPASS    1u      /* compass lost too long / compass + gyro lost: the caller ends DP */
#define DP_R_OFFSPOT    2u      /* too far / too long off the spot: the caller ends DP */
#define DP_R_TRIP_END   3u      /* TRIP reached its last (or a STOP) waypoint: now SPOT there (no mode change) */

typedef struct {
    uint8_t  sub;               /* DP_ (DP_OFF outside MODE_DP) */
    double   alat, alon;        /* the spot (SPOT / JOG) */
    double   cos_alat;
    float    hdg_target;        /* deg */
    float    set_kn;            /* CRUISE / TRIP speed */
    float    trip_kn;           /* TRIP: the wanted speed after caps / slowing (telemetry) */
    float    en, ee;            /* position error boat -> spot, m north / east (filtered, propagated) */
    float    vn, ve;            /* ground velocity m/s (filtered) */
    bool     have_fix;
    float    err_fwd, err_stbd; /* boat frame (+ = the spot is ahead / to starboard) */
    float    err_m;
    float    v_fwd, v_stbd;
    float    ix, iv;            /* integrators: position fore / aft, CRUISE speed (stern %) */
    float    hdg_est;           /* compass, or carried on by the gyro while the compass is lost */
    float    hdg_err;
    uint32_t compass_lost_ms;
    uint32_t off_ms;            /* beyond spot_alarm_m since */
    uint32_t sat_ms;            /* both thrusters saturated since */
    bool     offspot;           /* DP OFF SPOT condition (advisory alarm) */
    bool     overpower;         /* DP OVERPOWER condition (advisory alarm) */
    uint32_t jog_ms;
    /* TRIP */
    nav_leg_t leg;
    uint8_t  wp;                /* the waypoint being driven to */
    bool     trip_done;         /* the last waypoint was reached */
    /* generator neutral check (section K): stern astern > DP_FIGHT_PCT for DP_FIGHT_MS while the spot error grows ahead */
    uint32_t fight_ms;
    float    fight_ref;
    bool     fight;
    /* demands for thruster.c */
    float    bow, stern;
    /* result of the last stop */
    uint8_t  stop_why;
    uint32_t entries, jogs, trips_done, stops;
    uint32_t log_seq;           /* +1 per event for the log (start / sub change / jog / stop / trip end) */
    uint8_t  log_what;          /* 4 start / sub change, 5 stopped, 6 jog, 7 trip end */
} dp_t;

void dp_init(dp_t *d);

/* Start (or change to) a sub-state here: SPOT = the anchor at lat / lon, the anchor heading = heading;
 * CRUISE = this heading, speed_kn (0..max_kn); TRIP = the program from waypoint wp_from at speed_kn. */
void dp_start(dp_t *d, uint8_t sub, double lat, double lon, float heading, float speed_kn, float max_kn,
              const program_t *prog, uint8_t wp_from);
/* JOG one step (SPOT / JOG only): false = refused (not in SPOT / JOG, bad direction) */
bool dp_jog(dp_t *d, uint8_t dir, float jog_m);
/* CRUISE / TRIP speed: false = not in CRUISE / TRIP */
bool dp_speed(dp_t *d, float kn, float max_kn);
/* SPOT / JOG / CRUISE heading: absolute (rel false) or a change; false = in TRIP / off */
bool dp_heading(dp_t *d, float deg, bool rel);
/* DP off (thrusters 0 now: the caller's thruster step sees dp_active false) */
void dp_stop(dp_t *d, uint8_t why);

/* One control step: d->bow / d->stern for thruster.c. Returns DP_R_. */
uint8_t dp_step(dp_t *d, const dp_in_t *in);

const char *dp_sub_name(uint8_t sub);

#endif /* DP_H */
