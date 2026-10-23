/*
 * dr.h - Position estimate: GPS when it can be trusted, dead reckoning
 * (DR) when it can't. Pure C99.
 *
 * Always running (once there was a first fix):
 *   - predicts the position from heading + speed through water + learned
 *     water current, and corrects it with each trusted GPS fix
 *   - learns the current vector and a throttle -> speed table while GPS is
 *     good (or uses a paddlewheel speed sensor when fitted)
 *   - checks every fix: receiver spoof/jam flag, jump away from the
 *     prediction, impossible speed, course that disagrees with heading +
 *     drift, and a slow drag (GPS drifting away from an independent DR
 *     "shadow" over 60 s / 300 s windows)
 *   - on loss or a suspect fix: DR, uncertainty grows with time and distance
 *   - GPS comes back: trusted again only after N consistent fixes, then the
 *     estimate is blended over to GPS smoothly (no jump)
 */
#ifndef DR_H
#define DR_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#define DR_TABLE_N 11       /* throttle 0, 10, ... 100 % */

typedef enum {
    DR_NO_FIX = 0,          /* never had a fix: no position */
    DR_GPS,                 /* GPS trusted */
    DR_DEAD,                /* dead reckoning (GPS lost or suspect) */
    DR_RECOVER              /* dead reckoning, GPS fixes being checked again */
} dr_state_t;

typedef enum {
    DR_WHY_NONE = 0,
    DR_WHY_LOST,            /* no fixes */
    DR_WHY_FLAG,            /* receiver reports spoofing / jamming */
    DR_WHY_JUMP,            /* fix too far from the prediction */
    DR_WHY_SPEED,           /* impossible speed */
    DR_WHY_COURSE,          /* COG disagrees with heading + drift */
    DR_WHY_DRAG,            /* GPS slowly drifting away from DR */
    DR_WHY_GPS2,            /* disagrees with the second GPS (AIS device; off unless configured) */
    DR_WHY_INVALID          /* fix with NaN / infinite / impossible values (lat, lon, SOG, COG) */
} dr_reason_t;

typedef struct {
    /* uncertainty (radius, m) */
    float unc_gps_m;            /* right after a trusted fix (5) */
    float unc_per_s;            /* growth per second of DR (0.1) */
    float unc_frac_dist;        /* growth per metre travelled, throttle table (0.04) */
    float unc_frac_dist_pw;     /* ... with a speed sensor (0.02) */
    float unc_frac_unlearned;   /* extra per metre when the table isn't learned there (0.15) */
    /* learning */
    float current_tau_s;        /* current vector filter (30); checks that use the
                                   current start after 4 x this */
    float table_tau_s;          /* throttle -> speed filter (15) */
    float speed_tau_s;          /* boat speed response to throttle (6) */
    float default_v0_kn;        /* table before learning: v = v0 + k * throttle (1.0) */
    float default_kn_per_pct;   /* (0.15) */
    /* integrity */
    float jump_m;               /* fix more than this + 3 x uncertainty from the prediction (40) */
    float gps2_max_m;           /* fix more than this from the second GPS = suspect; 0 = check off (default) */
    float max_sog_kn;           /* impossible speed (40) */
    float course_tol_deg;       /* COG vs heading + drift (30) */
    float drag_window_s;        /* shadow DR windows (60 / 300 s) ... */
    float drag_limit_m;         /* ... and allowed disagreement (25 / 60 m) */
    float drag_window2_s;
    float drag_limit2_m;
    float drag_cur_limit;       /* learned current moved more than this (m/s) within a
                                   shadow window = GPS dragging (0.25) */
    float turn_reset_deg;       /* heading change that restarts the shadows (20) */
    uint32_t gps_timeout_ms;    /* no fix this long = lost (3000) */
    /* recovery */
    uint8_t recover_fixes;      /* consistent fixes before GPS is trusted again (10) */
    float recover_gate_m;       /* fix within max(this, 3 x uncertainty) of DR (30) */
    float recover_step_m;       /* fix-to-fix motion vs prediction (8) */
    float recover_drift_m;      /* GPS - DR offset may change at most this + rate x time */
    float recover_drift_rate;   /* while recovering (5 m + 0.3 m/s; a drag is faster) */
    float recover_along_frac;   /* ALONG the heading only, after a suspect fix: DR's own speed error
                                 * may add up to this fraction of the distance run (gate) / of the
                                 * water speed (drift) - ONLY when something independent backs it:
                                 * the second GPS agrees (0.25), or a paddlewheel measures the speed
                                 * (recover_along_frac_pw 0.05). Without either: 0 (the old rules - a
                                 * slow drag along the course looks exactly like a speed-table error).
                                 * Across the track nothing changes. */
    float recover_along_frac_pw;
    uint8_t recover_fixes_suspect; /* after an integrity alarm: fixes needed (60), and the
                                   gate is 1 x uncertainty instead of 3 x */
    float blend_tau_s;          /* estimate -> GPS blend time after recovery (15) */
} dr_config_t;

typedef struct {
    uint32_t now_ms;
    float    heading_deg;       /* compass */
    bool     heading_ok;
    float    yaw_dps;           /* gyro or compass rate (learning only when calm) */
    float    throttle_pct;      /* sent to the throttle servo */
    int8_t   gear;              /* +1 F, 0 N, -1 R (settled) */
    bool     water_ok;          /* paddlewheel fitted and read */
    float    water_kn;
    bool     gps_new;           /* a GPS message arrived this step */
    bool     gps_bad;           /* ... and it was a fix with invalid values (counts as a rejected fix) */
    hal_gps_t gps;
    bool     gps2_ok;           /* a fresh position from the second GPS (gateway, AIS device) */
    double   gps2_lat, gps2_lon;
} dr_input_t;

typedef struct {
    float n, e;                 /* position, m */
    float cur_n, cur_e;         /* current frozen at the anchor */
    float water_kn;
    uint32_t t0_ms;
    bool  valid;                /* window usable (learned, forward, no gaps) */
    float heading_deg;          /* heading at the anchor (a real turn restarts it) */
} dr_shadow_t;

typedef struct {
    dr_config_t cfg;
    dr_state_t  state;
    dr_reason_t why;            /* why DR started (last time) */

    bool     origin_set;
    double   lat0, lon0, cos_lat0;
    float    n, e;              /* estimated position, m from the origin */
    float    unc_m;             /* uncertainty radius, m */
    float    vn, ve;            /* estimated ground velocity, m/s */
    float    cur_n, cur_e;      /* learned water current, m/s */
    float    cur_learn_s;       /* seconds of current learning so far */
    float    table_kn[DR_TABLE_N];
    float    table_w[DR_TABLE_N];   /* how well each point is learned (0..1) */
    float    water_kn;          /* speed through water used now */
    float    thr_lp;            /* throttle delayed like the boat speed */
    bool     speed_sensor;      /* paddlewheel in use */
    float    heading_now;

    uint32_t last_ms;
    uint32_t last_msg_ms;       /* last GPS message with a fix */
    uint32_t last_fix_ms;       /* last TRUSTED fix */
    uint32_t dr_start_ms;
    float    dr_dist_m;         /* distance travelled in DR */

    /* integrity / recovery */
    uint32_t suspects;          /* integrity rejections (events) */
    uint32_t invalid_fixes;     /* fixes with NaN / impossible values (rejected) */
    uint8_t  good_count;
    bool     have_prev;
    float    prev_n, prev_e;    /* previous raw fix (recovery consistency) */
    float    pred_since_prev_n, pred_since_prev_e;
    dr_shadow_t sh[2];          /* slow-drag shadows (short / long window) */
    float    blend_n, blend_e;  /* estimate - GPS, blended out after recovery */
    float    rec_off_n, rec_off_e;  /* GPS - DR offset at the first good recovery fix */
    uint32_t rec_t0_ms;
} dr_t;

void dr_default_config(dr_config_t *cfg);
void dr_init(dr_t *d, const dr_config_t *cfg);

/* Every control step. */
void dr_step(dr_t *d, const dr_input_t *in);

static inline bool dr_trusted(const dr_t *d) { return d->state == DR_GPS; }
static inline bool dr_have_position(const dr_t *d) { return d->state != DR_NO_FIX; }

void  dr_latlon(const dr_t *d, double *lat, double *lon);
float dr_sog_kn(const dr_t *d);
float dr_cog_deg(const dr_t *d);
float dr_time_since_fix_s(const dr_t *d, uint32_t now_ms);
float dr_dr_time_s(const dr_t *d, uint32_t now_ms);   /* 0 if GPS trusted */

/* Learned throttle -> speed through water, and back. */
float dr_speed_for(const dr_t *d, float throttle_pct);
float dr_throttle_for(const dr_t *d, float water_kn);

const char *dr_state_name(dr_state_t s);
const char *dr_reason_name(dr_reason_t r);

/* A fix whose numbers can be used at all: finite, latitude -90..90,
 * longitude -180..180, SOG 0..100 kn, COG 0..360. Anything else is a bad
 * fix (DR_WHY_INVALID), never a position. */
bool dr_fix_valid(const hal_gps_t *g);

#endif /* DR_H */
