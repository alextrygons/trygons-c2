/*
 * dr.c - Position estimate with dead reckoning and GPS integrity checks.
 * See dr.h.
 */
#include "dr.h"

#include <math.h>
#include <string.h>

#define KN_MS        0.514444f
#define DEG2RAD_F    0.017453293f
#define RAD2DEG_F    57.29578f
#define M_PER_DEG    111320.0

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float wrap180(float d)
{
    d = fmodf(d, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

const char *dr_state_name(dr_state_t s)
{
    static const char *const n[] = { "no fix", "GPS", "dead reckoning", "GPS recovering" };
    return ((unsigned)s < 4u) ? n[s] : "?";
}

/* Distance (m) between a fix (grid fn / fe) and the second GPS; -1 when the
 * check is off or there is no fresh second-GPS position. */
static void to_grid(const dr_t *d, double lat, double lon, float *n, float *e);
static float gps2_off(const dr_t *d, const dr_input_t *in, float fn, float fe)
{
    if (d->cfg.gps2_max_m <= 0.0f || !in->gps2_ok) return -1.0f;
    float gn, ge;
    to_grid(d, in->gps2_lat, in->gps2_lon, &gn, &ge);
    return sqrtf((fn - gn) * (fn - gn) + (fe - ge) * (fe - ge));
}

const char *dr_reason_name(dr_reason_t r)
{
    static const char *const n[] = { "-", "GPS lost", "receiver spoof/jam flag", "position jump",
                                     "impossible speed", "course disagrees", "slow drag",
                                     "second GPS disagrees", "invalid fix values" };
    return ((unsigned)r < 8u) ? n[r] : "?";
}

void dr_default_config(dr_config_t *c)
{
    c->unc_gps_m = 5.0f;
    c->unc_per_s = 0.08f;
    c->unc_frac_dist = 0.03f;
    c->unc_frac_dist_pw = 0.02f;
    c->unc_frac_unlearned = 0.06f;
    c->current_tau_s = 30.0f;
    c->table_tau_s = 15.0f;
    c->speed_tau_s = 6.0f;
    c->default_v0_kn = 1.0f;
    c->default_kn_per_pct = 0.15f;
    c->learn_thr_dev_pct = 3.0f;
    c->jump_m = 40.0f;
    c->gps2_max_m = 0.0f;               /* second GPS check off until configured */
    c->max_sog_kn = 40.0f;
    c->course_tol_deg = 30.0f;
    c->drag_window_s = 60.0f;
    c->drag_limit_m = 25.0f;
    c->drag_window2_s = 300.0f;
    c->drag_limit2_m = 60.0f;
    c->drag_cur_limit = 0.25f;
    c->turn_reset_deg = 20.0f;
    c->gps_timeout_ms = 3000u;
    c->recover_fixes = 10u;
    c->recover_gate_m = 30.0f;
    c->recover_step_m = 8.0f;
    c->recover_drift_m = 5.0f;
    c->recover_drift_rate = 0.3f;
    c->recover_along_frac = 0.25f;
    c->recover_along_frac_pw = 0.05f;
    c->recover_fixes_suspect = 60u;
    c->blend_tau_s = 15.0f;
}

void dr_init(dr_t *d, const dr_config_t *cfg)
{
    memset(d, 0, sizeof *d);
    d->cfg = *cfg;
    for (int i = 0; i < DR_TABLE_N; i++)
        d->table_kn[i] = cfg->default_v0_kn + cfg->default_kn_per_pct * 10.0f * (float)i;
}

/* --- throttle -> speed table ------------------------------------------------ */

static float model_kn(const dr_t *d, float thr)
{
    return d->cfg.default_v0_kn + d->cfg.default_kn_per_pct * thr;
}

/* Value of table point b: learned value, mixed with an estimate scaled
 * from the nearest well-learned point (model shape) while not learned. */
static float bin_value(const dr_t *d, int b)
{
    float w = d->table_w[b];
    if (w >= 0.95f) return d->table_kn[b];
    int best = -1;
    for (int k = 1; k < DR_TABLE_N && best < 0; k++) {
        if (b - k >= 0 && d->table_w[b - k] >= 0.5f) best = b - k;
        else if (b + k < DR_TABLE_N && d->table_w[b + k] >= 0.5f) best = b + k;
    }
    float guess = d->table_kn[b];
    if (best >= 0) {
        float mb = model_kn(d, 10.0f * (float)best), m = model_kn(d, 10.0f * (float)b);
        guess = (mb > 0.1f) ? d->table_kn[best] * m / mb : m;
    }
    return w * d->table_kn[b] + (1.0f - w) * guess;
}

float dr_speed_for(const dr_t *d, float thr)
{
    float x = clampf(thr, 0.0f, 100.0f) / 10.0f;
    int i = (int)x;
    if (i >= DR_TABLE_N - 1) return bin_value(d, DR_TABLE_N - 1);
    float f = x - (float)i;
    return (1.0f - f) * bin_value(d, i) + f * bin_value(d, i + 1);
}

float dr_throttle_for(const dr_t *d, float water_kn)
{
    for (float t = 0.0f; t <= 100.0f; t += 0.5f)
        if (dr_speed_for(d, t) >= water_kn) return t;
    return 100.0f;
}

static float learned_at(const dr_t *d, float thr)
{
    float x = clampf(thr, 0.0f, 100.0f) / 10.0f;
    int i = (int)x;
    if (i >= DR_TABLE_N - 1) return d->table_w[DR_TABLE_N - 1];
    float f = x - (float)i;
    return (1.0f - f) * d->table_w[i] + f * d->table_w[i + 1];
}

static void table_learn(dr_t *d, float thr, float kn, float a)
{
    float x = clampf(thr, 0.0f, 100.0f) / 10.0f;
    int i = (int)x;
    if (i >= DR_TABLE_N - 1) i = DR_TABLE_N - 2;
    float f = x - (float)i;
    float w[2] = { 1.0f - f, f };
    for (int k = 0; k < 2; k++) {
        int b = i + k;
        float g = clampf(a * w[k], 0.0f, 1.0f);
        if (d->table_w[b] < 0.05f) d->table_kn[b] = bin_value(d, b);   /* start from the best guess */
        d->table_kn[b] += g * (kn - d->table_kn[b]);
        d->table_w[b] += g * (1.0f - d->table_w[b]);
    }
}

/* --- helpers -------------------------------------------------------------------- */

static void to_grid(const dr_t *d, double lat, double lon, float *n, float *e)
{
    *n = (float)((lat - d->lat0) * M_PER_DEG);
    *e = (float)((lon - d->lon0) * M_PER_DEG * d->cos_lat0);
}

void dr_latlon(const dr_t *d, double *lat, double *lon)
{
    *lat = d->lat0 + (double)d->n / M_PER_DEG;
    *lon = d->lon0 + (double)d->e / (M_PER_DEG * d->cos_lat0);
}

float dr_sog_kn(const dr_t *d)
{
    return sqrtf(d->vn * d->vn + d->ve * d->ve) / KN_MS;
}

float dr_cog_deg(const dr_t *d)
{
    float c = atan2f(d->ve, d->vn) * RAD2DEG_F;
    return (c < 0.0f) ? c + 360.0f : c;
}

float dr_time_since_fix_s(const dr_t *d, uint32_t now_ms)
{
    return (d->state == DR_NO_FIX) ? -1.0f : (float)(now_ms - d->last_fix_ms) * 0.001f;
}

float dr_dr_time_s(const dr_t *d, uint32_t now_ms)
{
    return (d->state == DR_DEAD || d->state == DR_RECOVER)
           ? (float)(now_ms - d->dr_start_ms) * 0.001f : 0.0f;
}

static void anchor(dr_t *d, dr_shadow_t *s, float n, float e, uint32_t now, bool usable)
{
    s->n = n;
    s->e = e;
    s->cur_n = d->cur_n;
    s->cur_e = d->cur_e;
    s->t0_ms = now;
    s->valid = usable;
    s->heading_deg = d->heading_now;
}

static void enter_dr(dr_t *d, dr_reason_t why, uint32_t now)
{
    d->state = DR_DEAD;
    d->why = why;
    d->dr_start_ms = now;
    d->dr_dist_m = 0.0f;
    d->good_count = 0u;
    d->have_prev = false;
    d->blend_n = d->blend_e = 0.0f;
    d->sh[0].valid = d->sh[1].valid = false;
    if (why != DR_WHY_LOST) d->suspects++;
}

bool dr_fix_valid(const hal_gps_t *g)
{
    /* written so that NaN fails every comparison */
    return g->lat_deg >= -90.0 && g->lat_deg <= 90.0 && g->lon_deg >= -180.0 && g->lon_deg <= 180.0 &&
           g->sog_kn >= 0.0f && g->sog_kn <= 100.0f && g->cog_deg >= 0.0f && g->cog_deg <= 360.0f;
}

/* A fix with unusable numbers: a GPS failure like an integrity rejection
 * (GPS trusted again only after the consistent fixes of a suspect GPS). */
static void reject_invalid(dr_t *d, uint32_t now)
{
    if (d->state == DR_GPS) {
        enter_dr(d, DR_WHY_INVALID, now);
    } else if (d->state != DR_NO_FIX) {
        if (d->why == DR_WHY_LOST) d->why = DR_WHY_INVALID;     /* recover like after a suspect fix */
        d->suspects++;
        d->good_count = 0u;
        d->have_prev = false;
        d->state = DR_DEAD;
    }
    d->invalid_fixes++;
}

/* --- step ----------------------------------------------------------------------- */

void dr_step(dr_t *d, const dr_input_t *in_raw)
{
    const dr_config_t *c = &d->cfg;
    /* inputs that are not finite are not used (never NaN in the estimate) */
    dr_input_t v = *in_raw;
    const dr_input_t *in = &v;
    if (!isfinite(v.heading_deg)) { v.heading_deg = d->heading_now; v.heading_ok = false; }
    if (!isfinite(v.yaw_dps)) v.yaw_dps = 0.0f;
    if (!isfinite(v.throttle_pct)) v.throttle_pct = 0.0f;
    if (v.water_ok && !(v.water_kn >= -50.0f && v.water_kn <= 100.0f)) v.water_ok = false;
    if (v.gps2_ok && !(v.gps2_lat >= -90.0 && v.gps2_lat <= 90.0 && v.gps2_lon >= -180.0 && v.gps2_lon <= 180.0))
        v.gps2_ok = false;
    float dt = (d->last_ms == 0u) ? 0.05f : (float)(in->now_ms - d->last_ms) * 0.001f;
    d->last_ms = in->now_ms;
    dt = clampf(dt, 0.0f, 1.0f);

    /* speed through water: paddlewheel, or table at the (delayed) throttle */
    float target = (in->gear > 0) ? dr_speed_for(d, in->throttle_pct)
                 : (in->gear < 0) ? -0.4f * dr_speed_for(d, in->throttle_pct) : 0.0f;
    float a_v = dt / (c->speed_tau_s + dt);
    if (in->gear > 0) d->thr_lp += a_v * (in->throttle_pct - d->thr_lp);
    else d->thr_lp = 0.0f;
    float model_v = d->water_kn + a_v * (target - d->water_kn);
    d->speed_sensor = in->water_ok;
    d->water_kn = in->water_ok ? in->water_kn : model_v;

    d->heading_now = in->heading_deg;
    float h = in->heading_deg * DEG2RAD_F;
    float wn = d->water_kn * KN_MS * cosf(h), we = d->water_kn * KN_MS * sinf(h);
    d->vn = wn + d->cur_n;
    d->ve = we + d->cur_e;

    if (d->state == DR_NO_FIX) {
        /* nothing to predict yet */
    } else {
        d->n += d->vn * dt;
        d->e += d->ve * dt;
        d->pred_since_prev_n += d->vn * dt;
        d->pred_since_prev_e += d->ve * dt;
        bool usable = (in->gear > 0) && (d->speed_sensor || learned_at(d, d->thr_lp) >= 0.5f) &&
                      d->cur_learn_s > 4.0f * c->current_tau_s;
        for (int k = 0; k < 2; k++) {
            d->sh[k].n += (wn + d->sh[k].cur_n) * dt;
            d->sh[k].e += (we + d->sh[k].cur_e) * dt;
            if (!usable || fabsf(wrap180(in->heading_deg - d->sh[k].heading_deg)) > c->turn_reset_deg)
                d->sh[k].valid = false;     /* a real turn reveals hidden current: restart */
        }
    }

    if (d->state == DR_DEAD || d->state == DR_RECOVER) {
        float dist = sqrtf(d->vn * d->vn + d->ve * d->ve) * dt;
        float frac = d->speed_sensor ? c->unc_frac_dist_pw
                   : c->unc_frac_dist + (1.0f - clampf(learned_at(d, d->thr_lp), 0.0f, 1.0f)) * c->unc_frac_unlearned;
        if (!in->heading_ok) frac += 0.2f;
        d->dr_dist_m += dist;
        d->unc_m += c->unc_per_s * dt + frac * dist;
    }

    if (d->state == DR_GPS && (in->now_ms - d->last_msg_ms) > c->gps_timeout_ms) {
        enter_dr(d, DR_WHY_LOST, in->now_ms);
    }

    if (!in->gps_new) return;
    if (in->gps_bad || (in->gps.fix && !dr_fix_valid(&in->gps))) {
        reject_invalid(d, in->now_ms);
        return;
    }
    if (!in->gps.fix) return;

    /* --- a GPS fix --- */
    if (!d->origin_set) {
        d->origin_set = true;
        d->lat0 = in->gps.lat_deg;
        d->lon0 = in->gps.lon_deg;
        d->cos_lat0 = cos(in->gps.lat_deg * 0.017453292519943295);
        d->n = d->e = 0.0f;
        d->state = DR_GPS;
        d->unc_m = c->unc_gps_m;
        d->last_fix_ms = d->last_msg_ms = in->now_ms;
        anchor(d, &d->sh[0], 0.0f, 0.0f, in->now_ms, false);
        anchor(d, &d->sh[1], 0.0f, 0.0f, in->now_ms, false);
        d->have_prev = true;
        d->prev_n = d->prev_e = 0.0f;
        return;
    }

    float fn, fe;
    to_grid(d, in->gps.lat_deg, in->gps.lon_deg, &fn, &fe);
    float fix_dt = clampf((float)(in->now_ms - d->last_msg_ms) * 0.001f, 0.05f, 5.0f);
    d->last_msg_ms = in->now_ms;

    if (d->state == DR_GPS) {
        /* integrity checks against the prediction */
        dr_reason_t bad = DR_WHY_NONE;
        int drag_k = -1;
        float pn = d->n - d->blend_n, pe = d->e - d->blend_e;      /* where GPS should be */
        float jump = sqrtf((fn - pn) * (fn - pn) + (fe - pe) * (fe - pe));
        float sog = in->gps.sog_kn;

        if (in->gps.jam_spoof) bad = DR_WHY_FLAG;
        else if (sog > c->max_sog_kn) bad = DR_WHY_SPEED;
        else if (jump > c->jump_m + 3.0f * d->unc_m) bad = DR_WHY_JUMP;
        else if (gps2_off(d, in, fn, fe) > c->gps2_max_m) bad = DR_WHY_GPS2;
        else if (sog > 3.0f && in->gear > 0 && in->heading_ok && d->cur_learn_s > 4.0f * c->current_tau_s &&
                 d->water_kn > 2.0f && fabsf(in->yaw_dps) < 5.0f &&
                 fabsf(wrap180(in->gps.cog_deg - dr_cog_deg(d))) > c->course_tol_deg) bad = DR_WHY_COURSE;
        else {
            const float win[2] = { c->drag_window_s, c->drag_window2_s };
            const float lim[2] = { c->drag_limit_m, c->drag_limit2_m };
            for (int k = 0; k < 2 && bad == DR_WHY_NONE; k++) {
                dr_shadow_t *s = &d->sh[k];
                if (!s->valid) continue;
                /* a real current hardly changes within a few minutes: if the
                 * learned one runs away, the GPS is dragging it along */
                float dcn = d->cur_n - s->cur_n, dce = d->cur_e - s->cur_e;
                if (sqrtf(dcn * dcn + dce * dce) > c->drag_cur_limit) { bad = DR_WHY_DRAG; drag_k = k; break; }
                if ((float)(in->now_ms - s->t0_ms) * 0.001f < win[k]) continue;
                float disc = sqrtf((fn - s->n) * (fn - s->n) + (fe - s->e) * (fe - s->e));
                if (disc > lim[k]) { bad = DR_WHY_DRAG; drag_k = k; }
            }
            /* restore from the oldest clean shadow (anchored before the drag) */
            if (drag_k == 0 && d->sh[1].valid) drag_k = 1;
        }

        if (bad != DR_WHY_NONE) {
            if (drag_k >= 0) {
                /* the GPS was already leading us astray: go back to the
                 * independent shadow estimate and the current from then */
                d->n = d->sh[drag_k].n;
                d->e = d->sh[drag_k].e;
                d->cur_n = d->sh[drag_k].cur_n;
                d->cur_e = d->sh[drag_k].cur_e;
                d->unc_m = fmaxf(d->unc_m, (drag_k == 0) ? c->drag_limit_m : c->drag_limit2_m);
            }
            enter_dr(d, bad, in->now_ms);
            return;
        }

        /* trusted fix: learn, then correct the estimate */
        float h2 = in->heading_deg * DEG2RAD_F;
        /* (W18: no learning in a fast throttle slow-down without a paddlewheel - see learn_thr_dev_pct) */
        bool steady = d->speed_sensor || d->thr_lp - in->throttle_pct < c->learn_thr_dev_pct;
        if (in->gear > 0 && sog > 1.0f && in->heading_ok && fabsf(in->yaw_dps) < 5.0f && steady &&
            fabsf(d->blend_n) + fabsf(d->blend_e) < 10.0f) {
            float cg = in->gps.cog_deg * DEG2RAD_F;
            float gn = sog * KN_MS * cosf(cg), ge = sog * KN_MS * sinf(cg);
            float a_c = clampf(fix_dt / c->current_tau_s, 0.0f, 1.0f);
            float hn = cosf(h2), he = sinf(h2);
            float swn = d->water_kn * KN_MS * hn, swe = d->water_kn * KN_MS * he;
            float rn = gn - swn - d->cur_n, re = ge - swe - d->cur_e;   /* unexplained motion */
            if (d->speed_sensor) {
                /* speed through water is measured: all of it is current */
                d->cur_n += a_c * rn;
                d->cur_e += a_c * re;
            } else {
                /* without a speed sensor, current along the heading looks
                 * exactly like a different boat speed: the part across the
                 * heading is current, the part along it goes (mostly) into
                 * the speed table; turns reveal the rest over time */
                float along = rn * hn + re * he;
                d->cur_n += a_c * (rn - along * hn) + 0.1f * a_c * along * hn;
                d->cur_e += a_c * (re - along * he) + 0.1f * a_c * along * he;
            }
            d->cur_learn_s += fix_dt;
            if (!d->speed_sensor && d->thr_lp > 0.0f) {
                /* speed through water seen by GPS: ground velocity minus current, along the heading */
                float thw = ((gn - d->cur_n) * cosf(h2) + (ge - d->cur_e) * sinf(h2)) / KN_MS;
                table_learn(d, d->thr_lp, thw, fix_dt / c->table_tau_s);
            }
        }
        float decay = expf(-fix_dt / c->blend_tau_s);
        d->blend_n *= decay;
        d->blend_e *= decay;
        d->n = fn + d->blend_n;
        d->e = fe + d->blend_e;
        d->unc_m = c->unc_gps_m + sqrtf(d->blend_n * d->blend_n + d->blend_e * d->blend_e);
        d->last_fix_ms = in->now_ms;
        bool usable = (in->gear > 0) && (d->speed_sensor || learned_at(d, d->thr_lp) >= 0.5f) &&
                      d->cur_learn_s > 4.0f * c->current_tau_s;
        const float win[2] = { c->drag_window_s, c->drag_window2_s };
        for (int k = 0; k < 2; k++) {
            if (!d->sh[k].valid || (float)(in->now_ms - d->sh[k].t0_ms) * 0.001f >= win[k])
                anchor(d, &d->sh[k], fn, fe, in->now_ms, usable);
        }
        d->prev_n = fn;
        d->prev_e = fe;
        d->pred_since_prev_n = d->pred_since_prev_e = 0.0f;
        d->have_prev = true;
        return;
    }

    /* --- dead reckoning: is the GPS believable again? --- */
    /* after an integrity alarm the GPS has to come back to where DR says we
     * are (no widening with the uncertainty: a spoofer that stays away is
     * never trusted again), and for longer */
    bool after_suspect = (d->why != DR_WHY_LOST);
    float gate = fmaxf(c->recover_gate_m, (after_suspect ? 1.0f : 3.0f) * d->unc_m);
    uint8_t need = after_suspect ? c->recover_fixes_suspect : c->recover_fixes;
    float off = sqrtf((fn - d->n) * (fn - d->n) + (fe - d->e) * (fe - d->e));
    /* GPS - DR offset split along / across the heading. Across: the gate as
     * always. Along: DR's speed estimate can be off, so that much of the
     * distance run is allowed on top - but only with an independent check:
     * the second GPS agrees (a spoofer must fool both receivers), or a
     * paddlewheel measures the speed. Without one, a slow drag along the
     * course can't be told from a speed-table error: the old rules apply. */
    float hr = d->heading_now * DEG2RAD_F, hc = cosf(hr), hs = sinf(hr);
    float g2 = gps2_off(d, in, fn, fe);
    bool gps2_agrees = g2 >= 0.0f && g2 <= c->gps2_max_m;
    float along_frac = gps2_agrees ? c->recover_along_frac : d->speed_sensor ? c->recover_along_frac_pw : 0.0f;
    float off_along = (fn - d->n) * hc + (fe - d->e) * hs;
    float off_cross = -(fn - d->n) * hs + (fe - d->e) * hc;
    bool in_gate = off <= gate ||
                   (fabsf(off_cross) <= gate && fabsf(off_along) <= gate + along_frac * d->dr_dist_m);
    bool ok = !in->gps.jam_spoof && in->gps.sog_kn <= c->max_sog_kn && in_gate &&
              gps2_off(d, in, fn, fe) <= c->gps2_max_m;      /* the second GPS must agree too */
    if (ok && d->have_prev) {
        float mn = fn - d->prev_n - d->pred_since_prev_n, me = fe - d->prev_e - d->pred_since_prev_e;
        ok = sqrtf(mn * mn + me * me) <= c->recover_step_m;
    }
    if (ok) {
        /* the GPS - DR offset must stay put while recovering (a drag moves it) */
        if (d->good_count == 0u) {
            d->rec_off_n = fn - d->n;
            d->rec_off_e = fe - d->e;
            d->rec_t0_ms = in->now_ms;
        } else {
            float dn = fn - d->n - d->rec_off_n, de = fe - d->e - d->rec_off_e;
            float t = (float)(in->now_ms - d->rec_t0_ms) * 0.001f;
            float allow = c->recover_drift_m + c->recover_drift_rate * t;
            /* along the heading the drift may also be DR's speed error */
            float allow_along = allow + along_frac * fabsf(d->water_kn) * KN_MS * t;
            float drift_along = dn * hc + de * hs, drift_cross = -dn * hs + de * hc;
            ok = sqrtf(dn * dn + de * de) <= allow ||
                 (fabsf(drift_cross) <= allow && fabsf(drift_along) <= allow_along);
        }
    }
    d->prev_n = fn;
    d->prev_e = fe;
    d->pred_since_prev_n = d->pred_since_prev_e = 0.0f;
    d->have_prev = true;
    d->good_count = ok ? (uint8_t)(d->good_count + 1u) : 0u;
    d->state = d->good_count ? DR_RECOVER : DR_DEAD;

    if (d->good_count >= need) {
        /* trusted again: keep the estimate where it is and blend over to
         * GPS (no position / cross-track jump) */
        d->state = DR_GPS;
        d->blend_n = d->n - fn;
        d->blend_e = d->e - fe;
        d->unc_m = c->unc_gps_m + off;
        d->last_fix_ms = in->now_ms;
        d->good_count = 0u;
        anchor(d, &d->sh[0], fn, fe, in->now_ms, false);
        anchor(d, &d->sh[1], fn, fe, in->now_ms, false);
    }
}
