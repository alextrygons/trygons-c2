/*
 * harbour.h - Harbour autonomy (CLAUDE.md section Z, docs/harbour_design.md 2 + 3; step HX2) - the AURIX side.
 *
 * Harbour autonomy is a SUB-STATE of MODE_AUTO (sb_t.harb), never a mode of its own: every AUTO exit (the STANDBY
 * button / command, the e-stop, the CREWED joystick takeover, any mode change, every fault STANDBY) ends it in that
 * step. The gateway's planner only PROPOSES (GW_NAVREQ kind SB_NAV_HARBOUR = 4: a heading + a speed cap + flags,
 * a lease); this module VALIDATES each proposal every control step:
 *   - the lease: a kind-4 request fresher than min(the frame's lease, harb_lease_ms) - lapsed / never / the last
 *     request of another kind = HOLD (why LEASE), never AUTO's "carry on at nav_lost_kn";
 *   - the STOP flag (hflags b0) or a cap below SB_SHDG_STOP_KN = HOLD (why PLANNER: the planner is unsure);
 *   - the speed: min(leg speed, the start command's max, harb_max_kn (firmware ceiling HARB_MAX_KN_CEIL), the cap) -
 *     only ever lowered (nothing raises the throttle above the route's speed);
 *   - the heading: within harb_max_dev_deg of the leg track AND (inside the corridor minus harb_corr_margin_m OR not
 *     increasing |xte|); else the AURIX's own leg guidance (refused; 2 s of refusals in a row = HOLD why HEADING);
 *   - the corridor: |xte| > corr = the own guidance back to the line at <= harb_back_kn; > corr + harb_corr_stop_m
 *     = HOLD (why CORRIDOR);
 *   - the turn rate: the heading target given to the steering core moves at most harb_rot_dps (a slew);
 *   - the integrity: GPS untrusted / its uncertainty > harb_unc_m = HOLD (why GPS: no dead reckoning in a harbour).
 * HOLD = idle + neutral (only why USER / ARRIVED at the normal closing rate; every other reason = idle AT ONCE),
 * the heading held until then, then the steering holds; the mode stays AUTO until the captain's RESUME + CONFIRM
 * or another mode. Pure C99, no HAL (smallboat.c feeds it).
 */
#ifndef HARBOUR_H
#define HARBOUR_H

#include <stdbool.h>
#include <stdint.h>

/* harb.state (HARBOUR_STAT byte 0) */
#define HARB_OFF   0u       /* not running */
#define HARB_RUN   1u       /* following the harbour route, the planner's proposals validated + applied */
#define HARB_SLOW  2u       /* as RUN, a cap below the route speed (or a degraded sensor) in force */
#define HARB_HOLD  3u       /* stopped: idle + neutral, heading held until then, then the steering holds */
#define HARB_DONE  4u       /* arrived at the last waypoint (final STOP: STANDBY) */

/* why (HARBOUR_STAT byte 1): the last HOLD / refusal reason (docs/harbour_design.md 4.3) */
#define HARB_WHY_NONE         0u
#define HARB_WHY_USER         1u    /* the station's HOLD / AUTO PAUSE (normal closing rate) */
#define HARB_WHY_LEASE        2u    /* the kind-4 lease lapsed / never / another kind */
#define HARB_WHY_PLANNER      3u    /* the planner's STOP flag / a cap below SB_SHDG_STOP_KN */
#define HARB_WHY_CORRIDOR     4u    /* |xte| > corr + harb_corr_stop_m */
#define HARB_WHY_HEADING      5u    /* proposals refused for HARB_REFUSE_HOLD_MS in a row */
#define HARB_WHY_GPS          6u    /* GPS not trusted / uncertainty > harb_unc_m */
#define HARB_WHY_COMPASS      7u    /* compass lost */
#define HARB_WHY_LINK         8u    /* lost link (the mode ends: STANDBY) */
#define HARB_WHY_FAULT        9u    /* any other fault */
#define HARB_WHY_NOT_HARBOUR  10u   /* entry: the stored program is not flagged HARBOUR (or has < 2 waypoints) */
#define HARB_WHY_NOT_CHECKED  11u   /* entry: not loaded + checked over the gateway (121) */
#define HARB_WHY_OUT_OF_CORR  12u   /* entry: the boat outside the leg's corridor */
#define HARB_WHY_TOO_FAST     13u   /* entry: SOG > max + 0.5 kn */
#define HARB_WHY_NO_LEASE     14u   /* entry: no fresh kind-4 request (or its STOP flag) */
#define HARB_WHY_ARRIVED      15u   /* a STOP waypoint / the final HOLD reached (planned: normal closing rate) */
#define HARB_WHY_COUNT        16u

/* PROTO_T_HARBOUR actions */
#define HARB_ACT_START   0u
#define HARB_ACT_RESUME  1u
#define HARB_ACT_HOLD    2u

/* GW_NAVREQ kind 4 hflags */
#define HARB_HF_STOP      0x01u     /* the planner: STOP and hold (unsure) */
#define HARB_HF_DEGRADED  0x02u     /* a sensor lost (planner degraded) */
#define HARB_HF_GIVEWAY   0x04u     /* giving way (COLREGs) */
#define HARB_HF_STATIC    0x08u     /* static obstacles only */
#define HARB_HF_CAMONLY   0x10u     /* a camera-only obstacle slowing */
#define HARB_HF_SHALLOW   0x20u     /* folded caps: shallow water */
#define HARB_HF_COLL      0x40u     /* ... collision */
#define HARB_HF_VISION    0x80u     /* ... vision */

#define HARB_MAX_KN_CEIL      8.0f      /* firmware ceiling of harb_max_kn (no parameter / command goes above) */
#define HARB_CORR_MIN_M       4.0f      /* the narrowest corridor a START / RESUME may ask (below = NACK) */
#define HARB_LEASE_FRAME_MIN  500u      /* kind-4 frame lease limits (gw_navreq) */
#define HARB_LEASE_FRAME_MAX  3000u
#define HARB_REFUSE_HOLD_MS   2000u     /* proposals refused this long in a row = HOLD why HEADING */
#define HARB_SOG_MARGIN_KN    0.5f      /* entry: SOG <= max + this */
/* W147 (owner, TO CONFIRM): the lost-link grace ships DISABLED - the parameter range is 0..0 until the owner signs
 * a value (then a separate commit raises this ceiling; a change needs STANDBY + CONFIRM like thr_trav_s, OD-4) */
#define HARB_LL_GRACE_MAX_S   0.0f
#define HARB_LL_DRIFT_MS      10000u    /* during the grace: trusted SOG > harb_ll_drift_kn this long at idle + N = kill */

typedef struct {
    float max_kn;           /* harb_max_kn: 5 (1..8, W144) */
    float corr_m;           /* harb_corr_m: 10 (4..20, W146) - the widest corridor a start may ask */
    float corr_margin_m;    /* harb_corr_margin_m: 2 - near the edge a proposal must not increase |xte| */
    float corr_stop_m;      /* harb_corr_stop_m: 5 (2..10) - beyond corr + this = HOLD */
    float max_dev_deg;      /* harb_max_dev_deg: 60 (20..90) - a proposal at most this far off the leg track */
    float rot_dps;          /* harb_rot_dps: 8 (2..15) - the heading target slew */
    float lease_ms;         /* harb_lease_ms: 1000 (500..3000) - the effective lease = min(frame, this) */
    float unc_m;            /* harb_unc_m: 5 (2..10) - GPS uncertainty limit (no DR in a harbour) */
    float back_kn;          /* harb_back_kn: 2 - back to the line at most this fast */
    float ll_grace_s;       /* harb_ll_grace_s: 0 (range 0..HARB_LL_GRACE_MAX_S until W147) */
    float ll_drift_kn;      /* harb_ll_drift_kn: 1 - during the grace, a drift above this = a stuck gear = kill */
} harb_cfg_t;

typedef struct {
    uint8_t  state;         /* HARB_ */
    uint8_t  why;           /* HARB_WHY_ (last HOLD / refusal) */
    float    corr_m;        /* this run's corridor (the station's value clamped to 4 m .. harb_corr_m) */
    float    max_kn;        /* this run's speed limit (min(station, harb_max_kn, HARB_MAX_KN_CEIL)) */
    float    tgt_deg;       /* the slewed heading target given to the steering core */
    float    cap_kn;        /* the speed target applied now */
    float    xte_m;         /* the leg's cross-track error (+ = right of the track) */
    uint32_t refuse_ms;     /* the first refused step of the current burst (0 = none) */
    uint8_t  refused;       /* refused proposals (count mod 256, HARBOUR_STAT) */
    uint8_t  refused_seq;   /* the request seq last counted as refused */
    bool     hold_emerg;    /* this step: the HOLD closes at once (not USER / ARRIVED, or a fault in force) */
    bool     hold_safe;     /* the HOLD reached idle + neutral (then the steering holds) */
    float    hold_hdg;      /* the heading held during the HOLD */
    bool     back;          /* this step: outside the corridor, own guidance back at harb_back_kn */
    bool     prop_used;     /* this step: the planner's heading steers */
    /* the request waiting for the CONFIRM (CONF_HARBOUR) */
    uint8_t  req_act;
    uint8_t  req_station;
    float    req_corr_m, req_kn;
    /* black box: log event 32 HARBOUR (logcore0.c) */
    uint8_t  log_what, log_why;
    uint16_t log_seq;
    /* counters (info / tests) */
    uint32_t starts, resumes, holds, refusals, backs, entry_refused;
    uint32_t k4_idle;       /* kind-4 requests received while harbour autonomy was OFF (never applied) */
} harb_t;

/* What smallboat.c knows this step (inputs of the validation). */
typedef struct {
    bool     lease_ok;      /* the last request is kind 4 and fresher than the effective lease */
    bool     stop_flag;     /* its hflags b0 */
    float    cap_kn;        /* its cap (< 0 = none) */
    float    prop_deg;      /* its heading */
    uint8_t  seq;           /* its seq (refusal count per proposal) */
    bool     gps_ok;        /* trusted GPS */
    float    unc_m;         /* the position uncertainty */
    bool     compass_ok;
    float    xte_m;         /* the leg's cross-track error */
    float    track_deg;     /* the leg's track */
    float    own_deg;       /* the AURIX's own leg guidance heading (line following + drift) */
    float    leg_kn;        /* the leg / cruise speed target before the harbour limits */
    uint32_t now_ms;
} harb_in_t;

typedef struct {
    uint8_t  hold_why;      /* HARB_WHY_NONE = carry on; else HOLD for this reason */
    float    want_deg;      /* the heading wanted (before the slew) */
    float    kn;            /* the speed target */
    bool     back;          /* outside the corridor: own guidance back */
    bool     prop_used;     /* the proposal's heading is used */
    bool     refused;       /* the proposal was refused this step */
} harb_out_t;

void  harb_default_config(harb_cfg_t *c);
float harb_wrap180(float a);
/* the heading target moved toward want by at most max_step degrees (shortest way round) */
float harb_slew(float cur, float want, float max_step);
/* the effective lease (ms): the gateway can only shorten it, never lengthen it */
uint32_t harb_eff_lease_ms(const harb_cfg_t *c, uint16_t frame_lease_ms);
/* the station's values clamped; false = refused (corridor below HARB_CORR_MIN_M, speed not > 0 / not finite) */
bool  harb_clamp_request(const harb_cfg_t *c, float corr_m, float kn, float *corr_out, float *kn_out);
/* does a heading of hdg (deg) increase |xte| (right of the track = +)? */
bool  harb_grows_xte(float track_deg, float hdg_deg, float xte_m);
/* the per-step judgement (2.3). Updates the refusal burst (h->refuse_ms / refused). */
void  harb_judge(const harb_cfg_t *c, harb_t *h, const harb_in_t *in, harb_out_t *o);
/* a hold reason that closes at the normal rate (only the user's HOLD and a planned stop) */
bool  harb_why_planned(uint8_t why);

#endif /* HARBOUR_H */
