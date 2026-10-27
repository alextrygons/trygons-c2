/*
 * dp.c - Dynamic positioning control (SPOT / JOG / CRUISE / TRIP). See dp.h.
 */
#include "dp.h"

#include <math.h>
#include <string.h>

#define PI_F      3.14159265f
#define DEG2RAD   (PI_F / 180.0f)
#define M_PER_DEG 111320.0
#define KN_MS     0.514444f

/* gains (sim-tuned; TO CONFIRM on the water) */
#define KP_H   4.0f     /* bow % per deg of heading error */
#define KD_H   10.0f    /* bow % per deg/s of yaw rate */
#define KP_X   8.0f     /* stern % per m fore / aft */
#define KD_X   30.0f    /* stern % per m/s fore / aft */
#define KI_X   0.4f     /* stern % per m*s */
#define KP_Y   6.0f     /* bow % per m sideways */
#define KD_Y   20.0f    /* bow % per m/s sideways */
#define KP_V   15.0f    /* CRUISE / TRIP: stern % per kn of speed error */
#define KI_V   4.0f     /* ... per kn*s */
#define DB_HDG_DEG  1.5f    /* heading dead band ... */
#define DB_YAW_DPS  0.4f    /* ... while the yaw rate is small */
#define DB_POS_M    1.0f    /* position dead band ... */
#define DB_VEL_MS   0.15f   /* ... while the speed is small */
#define FIX_GAIN    0.5f    /* blend of a new GPS fix into the propagated estimate */
#define TRIP_SLOW_M 30.0f   /* TRIP: slow down before the last / a STOP waypoint ... */
#define TRIP_SLOW_KN 1.0f   /* ... to this */

static float clampf(float x, float lo, float hi)
{
    if (!(x >= lo)) return lo;          /* (NaN -> lo) */
    if (x > hi) return hi;
    return x;
}

static float wrap360(float d)
{
    d = fmodf(d, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d;
}

static float wrap180(float d)
{
    d = wrap360(d);
    return d > 180.0f ? d - 360.0f : d;
}

void dp_init(dp_t *d)
{
    memset(d, 0, sizeof *d);
    d->sub = DP_OFF;
}

static void reset_loop(dp_t *d)
{
    d->ix = 0.0f;
    d->iv = 0.0f;
    d->off_ms = 0u;
    d->sat_ms = 0u;
    d->offspot = false;
    d->overpower = false;
    d->fight_ms = 0u;
    d->fight = false;
    d->jog_ms = 0u;
}

static void set_anchor(dp_t *d, double lat, double lon)
{
    d->alat = lat;
    d->alon = lon;
    d->cos_alat = cos(lat * 3.14159265358979 / 180.0);
    d->en = d->ee = 0.0f;               /* at the spot */
    d->err_fwd = d->err_stbd = d->err_m = 0.0f;
}

static void trip_leg(dp_t *d, const program_t *p, double lat, double lon)
{
    const prog_wp_t *w = &p->wp[d->wp];
    nav_leg_init(&d->leg, lat, lon, program_lat(w), program_lon(w));
    nav_leg_update(&d->leg, lat, lon);
}

void dp_start(dp_t *d, uint8_t sub, double lat, double lon, float heading, float speed_kn, float max_kn,
              const program_t *prog, uint8_t wp_from)
{
    bool was_off = d->sub == DP_OFF;
    if (was_off) {
        d->vn = d->ve = 0.0f;
        d->have_fix = true;             /* the caller only starts with a trusted fix */
        d->hdg_est = wrap360(heading);
        d->compass_lost_ms = 0u;
        d->stop_why = DP_STOP_NONE;
        d->entries++;
    }
    reset_loop(d);
    d->hdg_target = wrap360(heading);
    d->set_kn = clampf(speed_kn, 0.0f, max_kn);
    d->trip_done = false;
    set_anchor(d, lat, lon);            /* (CRUISE / TRIP: the error is only used for the generator check) */
    if (sub == DP_TRIP && prog != NULL && prog->n > 0u) {
        d->wp = (wp_from < prog->n) ? wp_from : (uint8_t)(prog->n - 1u);
        trip_leg(d, prog, lat, lon);
        d->sub = DP_TRIP;
    } else if (sub == DP_CRUISE) {
        d->sub = DP_CRUISE;
    } else {
        d->sub = DP_SPOT;
    }
    d->log_what = 4u;
    d->log_seq++;
}

bool dp_jog(dp_t *d, uint8_t dir, float jog_m)
{
    if ((d->sub != DP_SPOT && d->sub != DP_JOG) || dir > DP_JOG_STBD || !(jog_m > 0.0f)) return false;
    /* the step relative to the ANCHOR heading (where the bow is held), not the heading of the moment */
    float h = d->hdg_target * DEG2RAD;
    float f = (dir == DP_JOG_AHEAD) ? jog_m : (dir == DP_JOG_ASTERN) ? -jog_m : 0.0f;
    float st = (dir == DP_JOG_STBD) ? jog_m : (dir == DP_JOG_PORT) ? -jog_m : 0.0f;
    float dn = f * cosf(h) - st * sinf(h), de = f * sinf(h) + st * cosf(h);
    d->alat += (double)dn / M_PER_DEG;
    d->alon += (double)de / (M_PER_DEG * d->cos_alat);
    d->en += dn;                        /* the spot moved: the error with it */
    d->ee += de;
    d->sub = DP_JOG;
    d->jog_ms = 0u;
    d->off_ms = 0u;
    d->jogs++;
    d->log_what = 6u;
    d->log_seq++;
    return true;
}

bool dp_speed(dp_t *d, float kn, float max_kn)
{
    if (d->sub != DP_CRUISE && d->sub != DP_TRIP) return false;
    d->set_kn = clampf(kn, 0.0f, max_kn);
    return true;
}

bool dp_heading(dp_t *d, float deg, bool rel)
{
    if (d->sub == DP_OFF || d->sub == DP_TRIP || !isfinite(deg)) return false;
    d->hdg_target = wrap360(rel ? d->hdg_target + deg : deg);
    return true;
}

void dp_stop(dp_t *d, uint8_t why)
{
    if (d->sub == DP_OFF) return;
    d->sub = DP_OFF;
    d->bow = d->stern = 0.0f;
    d->stop_why = why;
    d->offspot = d->overpower = d->fight = false;
    d->stops++;
    d->log_what = 5u;
    d->log_seq++;
}

/* heading PD on the gyro (+ dead band): the bow's first job */
static float heading_term(dp_t *d, const dp_in_t *in)
{
    float r = in->gyro_ok ? in->yaw_dps : 0.0f;
    d->hdg_err = wrap180(d->hdg_target - d->hdg_est);
    if (fabsf(d->hdg_err) < DB_HDG_DEG && fabsf(r) < DB_YAW_DPS) return 0.0f;
    return clampf(KP_H * d->hdg_err - KD_H * r, -in->cap_bow, in->cap_bow);
}

/* the speed loop of CRUISE / TRIP on the stern motor (ahead only) */
static float speed_term(dp_t *d, const dp_in_t *in, float want_kn)
{
    float e = want_kn - d->v_fwd / KN_MS;
    float u = KP_V * e + d->iv;
    bool sat_hi = u >= in->cap_stern && e > 0.0f, sat_lo = u <= -0.3f * in->cap_stern && e < 0.0f;
    if (!sat_hi && !sat_lo) d->iv = clampf(d->iv + KI_V * e * in->dt, -0.3f * in->cap_stern, in->cap_stern);
    return clampf(KP_V * e + d->iv, -0.3f * in->cap_stern, in->cap_stern);
}

uint8_t dp_step(dp_t *d, const dp_in_t *in)
{
    uint8_t res = DP_R_OK;
    float dt = (in->dt > 0.0f && in->dt <= 1.0f) ? in->dt : 0.05f;
    if (d->sub == DP_OFF) {
        d->bow = d->stern = 0.0f;
        return DP_R_OK;
    }

    /* 1. heading: the compass, or carried on by the gyro while it is lost */
    if (in->compass_ok && isfinite(in->heading)) {
        d->hdg_est = wrap360(in->heading);
        d->compass_lost_ms = 0u;
    } else {
        if (!in->gyro_ok || !isfinite(in->yaw_dps)) {
            d->bow = d->stern = 0.0f;
            return DP_R_COMPASS;                    /* nothing to hold the heading with */
        }
        d->hdg_est = wrap360(d->hdg_est + in->yaw_dps * dt);
        d->compass_lost_ms += (uint32_t)(dt * 1000.0f + 0.5f);
        if (d->compass_lost_ms >= DP_COMPASS_MS) res = DP_R_COMPASS;
    }

    /* 2. position + velocity: propagated with the GPS velocity, a new fix blended in */
    d->en -= d->vn * dt;
    d->ee -= d->ve * dt;
    if (in->fix_new && isfinite(in->lat) && isfinite(in->lon)) {
        float bn = (float)((in->lat - d->alat) * M_PER_DEG);
        float be = (float)((in->lon - d->alon) * M_PER_DEG * d->cos_alat);
        float c = in->cog_deg * DEG2RAD, v = in->sog_kn * KN_MS;
        float vn = isfinite(c) && isfinite(v) ? v * cosf(c) : 0.0f, ve = isfinite(c) && isfinite(v) ? v * sinf(c) : 0.0f;
        d->en += FIX_GAIN * (-bn - d->en);
        d->ee += FIX_GAIN * (-be - d->ee);
        d->vn += FIX_GAIN * (vn - d->vn);
        d->ve += FIX_GAIN * (ve - d->ve);
        if (d->sub == DP_TRIP) nav_leg_update(&d->leg, in->lat, in->lon);
    }
    float h = d->hdg_est * DEG2RAD, ch = cosf(h), sh = sinf(h);
    d->err_fwd = d->en * ch + d->ee * sh;
    d->err_stbd = -d->en * sh + d->ee * ch;
    d->err_m = sqrtf(d->en * d->en + d->ee * d->ee);
    d->v_fwd = d->vn * ch + d->ve * sh;
    d->v_stbd = -d->vn * sh + d->ve * ch;

    /* 3. TRIP: the leg, the arrival, the heading + speed of the route */
    float want_kn = d->set_kn;
    if (d->sub == DP_TRIP && in->prog != NULL && d->wp < in->prog->n) {
        const prog_wp_t *w = &in->prog->wp[d->wp];
        bool last = (uint16_t)(d->wp + 1u) >= in->prog->n;
        bool stop_wp = last || w->action == (uint8_t)WP_ACT_STOP;
        if (in->fix_new && (d->leg.dist_m <= w->radius_m || d->leg.along_m >= d->leg.length_m)) {
            if (stop_wp) {
                /* the end (or a STOP waypoint): SPOT here, heading as it is */
                double lat = in->lat, lon = in->lon;
                d->trip_done = last;
                if (!last) d->wp++;             /* "continue" goes on from the next one */
                d->trips_done += last ? 1u : 0u;
                reset_loop(d);
                set_anchor(d, lat, lon);
                d->hdg_target = d->hdg_est;
                d->sub = DP_SPOT;
                d->log_what = 7u;
                d->log_seq++;
                res = (res == DP_R_OK) ? DP_R_TRIP_END : res;
            } else {
                d->wp++;
                const prog_wp_t *nw = &in->prog->wp[d->wp];
                nav_leg_init(&d->leg, program_lat(w), program_lon(w), program_lat(nw), program_lon(nw));
                nav_leg_update(&d->leg, in->lat, in->lon);
            }
        }
        if (d->sub == DP_TRIP) {
            const prog_wp_t *cw = &in->prog->wp[d->wp];
            bool slows = (uint16_t)(d->wp + 1u) >= in->prog->n || cw->action != (uint8_t)WP_ACT_CONTINUE;
            want_kn = fminf(fminf(d->set_kn, cw->speed_kn), in->max_kn);
            if (slows && d->leg.dist_m < TRIP_SLOW_M + cw->radius_m) {
                float f = clampf((d->leg.dist_m - cw->radius_m) / TRIP_SLOW_M, 0.0f, 1.0f);
                want_kn = fminf(want_kn, TRIP_SLOW_KN + (want_kn - TRIP_SLOW_KN) * f);
            }
            if (in->nav_cap_kn >= 0.0f && in->nav_cap_kn < want_kn) want_kn = in->nav_cap_kn;
            float look = fmaxf(20.0f, 8.0f * want_kn * KN_MS);
            d->hdg_target = in->nav_heading_on ? wrap360(in->nav_heading) : wrap360(nav_leg_course(&d->leg, look));
        }
    }
    d->trip_kn = (d->sub == DP_TRIP) ? want_kn : 0.0f;

    /* 4. the thrusters */
    float uh = heading_term(d, in);
    if (d->sub == DP_SPOT || d->sub == DP_JOG) {
        bool still = d->err_m < DB_POS_M && fabsf(d->v_fwd) < DB_VEL_MS && fabsf(d->v_stbd) < DB_VEL_MS;
        /* stern: fore / aft PI + damping */
        float px = still ? 0.0f : KP_X * d->err_fwd - KD_X * d->v_fwd;
        float ux = px + d->ix;
        bool sat_hi = ux >= in->cap_stern && d->err_fwd > 0.0f, sat_lo = ux <= -in->cap_stern && d->err_fwd < 0.0f;
        if (!sat_hi && !sat_lo && d->err_m < 15.0f)
            d->ix = clampf(d->ix + KI_X * d->err_fwd * dt, -0.6f * in->cap_stern, 0.6f * in->cap_stern);
        d->stern = clampf(px + d->ix, -in->cap_stern, in->cap_stern);
        /* bow: the heading first, the sideways part with what is left */
        float uy = still ? 0.0f : KP_Y * d->err_stbd - KD_Y * d->v_stbd;
        float fade = clampf(1.0f - fabsf(d->hdg_err) / DP_SWAY_HDG_DEG, 0.0f, 1.0f);
        float room = fminf(DP_SWAY_SHARE * in->cap_bow, in->cap_bow - fabsf(uh));
        if (room < 0.0f) room = 0.0f;
        uy = clampf(uy * fade, -room, room);
        d->bow = clampf(uh + uy, -in->cap_bow, in->cap_bow);
        /* JOG done -> SPOT */
        if (d->sub == DP_JOG) {
            d->jog_ms += (uint32_t)(dt * 1000.0f + 0.5f);
            if (d->err_m < DP_JOG_DONE_M || d->jog_ms >= DP_JOG_MAX_MS) {
                d->sub = DP_SPOT;
                d->log_what = 4u;
                d->log_seq++;
            }
        }
        /* off the spot: advisory beyond spot_alarm_m, the end beyond DP_OFF_STOP_M or after DP_OFF_STOP_MS */
        float alarm_m = (in->spot_alarm_m > 0.0f) ? in->spot_alarm_m : 15.0f;
        if (d->err_m > alarm_m) {
            d->offspot = true;
        } else if (d->err_m < 0.8f * alarm_m) {
            d->offspot = false;                 /* (hysteresis: back within 80 % of the alarm distance) */
            d->off_ms = 0u;
        }
        if (d->offspot) d->off_ms += (uint32_t)(dt * 1000.0f + 0.5f);
        if (d->err_m > DP_OFF_STOP_M || d->off_ms >= DP_OFF_STOP_MS) res = DP_R_OFFSPOT;
        /* generator neutral check: the stern astern hard while the boat is pushed ahead of the spot */
        if (in->out_stern < -DP_FIGHT_PCT) {
            if (d->fight_ms == 0u) d->fight_ref = -d->err_fwd;
            d->fight_ms += (uint32_t)(dt * 1000.0f + 0.5f);
            if (d->fight_ms >= DP_FIGHT_MS && -d->err_fwd - d->fight_ref >= DP_FIGHT_GROW_M) d->fight = true;
        } else {
            d->fight_ms = 0u;
            d->fight = false;
        }
    } else {
        d->offspot = false;
        d->off_ms = 0u;
        d->fight_ms = 0u;
        d->fight = false;
        d->stern = speed_term(d, in, want_kn);
        d->bow = uh;
    }

    /* 5. both thrusters at their limit for a while: DP OVERPOWER (advisory) */
    bool sat = in->cap_bow > 0.0f && in->cap_stern > 0.0f && fabsf(in->out_bow) >= in->cap_bow - 0.5f &&
               fabsf(in->out_stern) >= in->cap_stern - 0.5f;
    d->sat_ms = sat ? d->sat_ms + (uint32_t)(dt * 1000.0f + 0.5f) : 0u;
    d->overpower = d->sat_ms >= DP_OVERPOWER_MS;
    if (!isfinite(d->bow)) d->bow = 0.0f;
    if (!isfinite(d->stern)) d->stern = 0.0f;
    return res;
}

const char *dp_sub_name(uint8_t sub)
{
    static const char *const n[] = { "off", "SPOT", "JOG", "CRUISE", "TRIP" };
    return sub <= DP_TRIP ? n[sub] : "?";
}
