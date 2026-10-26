#include "nav.h"
#include <math.h>
#include <string.h>

#define PI      3.14159265358979323846
#define D2R     (PI / 180.0)
#define G       9.80665f
#define DEG2M   111320.0

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static float wrap_pi(float a)
{
    while (a >  (float)PI) a -= 2.0f * (float)PI;
    while (a < -(float)PI) a += 2.0f * (float)PI;
    return a;
}

/* Flat-earth conversion about A: fine for legs of a few tens of km. */
static void to_ne(const nav_t *s, double lat, double lon, double *n, double *e)
{
    *n = (lat - s->ref_lat) * DEG2M;
    *e = (lon - s->ref_lon) * DEG2M * cos(s->ref_lat * D2R);
}

void nav_cfg_default(nav_cfg_t *c)
{
    /* Starting values for a ~2-3 kg RC-class airframe. TUNE IN SIM FIRST. */
    c->cruise_airspeed = 18.0f;
    c->min_airspeed    = 11.0f;
    c->max_airspeed    = 30.0f;
    c->cruise_throttle = 0.55f;
    c->max_bank        = 30.0f * (float)D2R;
    c->max_pitch_up    = 15.0f * (float)D2R;
    c->max_pitch_down  = 10.0f * (float)D2R;
    c->accept_radius   = 30.0f;
    c->loiter_radius   = 80.0f;
    c->l1_period       = 17.0f;
    c->l1_damping      = 0.75f;
    c->k_hdg           = 1.2f;
    c->k_alt           = 0.02f;
    c->k_bank_pitch    = 0.10f;
    c->kp_roll         = 2.0f;
    c->kd_roll         = 0.15f;
    c->kp_pitch        = 2.5f;
    c->kd_pitch        = 0.20f;
    c->k_rudder        = 0.30f;
    c->kp_speed        = 0.05f;
    c->ki_speed        = 0.01f;
    c->k_climb_thr     = 1.0f;
    c->gps_timeout_s   = 2.0f;
}

void nav_init(nav_t *s, const nav_cfg_t *cfg)
{
    memset(s, 0, sizeof *s);
    s->cfg   = *cfg;
    s->state = NAV_IDLE;
}

bool nav_set_mission(nav_t *s, const nav_wp_t *via, uint8_t n_via, nav_wp_t dest)
{
    if (n_via > NAV_MAX_VIA || (n_via > 0 && via == NULL)) return false;
    for (uint8_t i = 0; i < n_via; i++) s->via[i] = via[i];
    s->n_via = n_via;
    s->dest  = dest;
    s->mission_loaded = true;
    return true;
}

uint8_t nav_current_leg(const nav_t *s) { return s->leg; }

static void engage(nav_t *s, const nav_input_t *in)
{
    s->ref_lat = in->lat_deg;
    s->ref_lon = in->lon_deg;

    s->route[0] = (nav_wp_t){ in->lat_deg, in->lon_deg, in->alt_m };   /* A */
    for (uint8_t i = 0; i < s->n_via; i++) s->route[1 + i] = s->via[i];
    s->route[1 + s->n_via] = s->dest;                                  /* B */
    s->n_pts = (uint8_t)(s->n_via + 2);

    for (uint8_t i = 0; i < s->n_pts; i++)
        to_ne(s, s->route[i].lat_deg, s->route[i].lon_deg, &s->rn[i], &s->re[i]);

    s->leg = 0;
    s->spd_int = 0.0f;
    s->gps_age = 0.0f;
    s->fs_latched = false;
    s->state = NAV_TRACK;
}

/* ---- lateral guidance: returns commanded bank (rad) ----------------------- */

static float l1_bank(const nav_t *s, float gs, float track,
                     double pn, double pe, double cn, double ce)
{
    /* carrot (cn,ce) -> L1 lateral acceleration -> bank */
    if (gs < 2.0f) return 0.0f;
    float l1  = fmaxf(15.0f, (1.0f / (float)PI) * s->cfg.l1_damping * s->cfg.l1_period * gs);
    double dn = cn - pn, de = ce - pe;
    float brg = (float)atan2(de, dn);
    float eta = wrap_pi(brg - track);
    float lat_acc = 2.0f * gs * gs / l1 * sinf(eta);
    return atanf(lat_acc / G);
}

static float l1_dist(const nav_t *s, float gs)
{
    return fmaxf(15.0f, (1.0f / (float)PI) * s->cfg.l1_damping * s->cfg.l1_period * gs);
}

void nav_update(nav_t *s, const nav_input_t *in, float dt, nav_output_t *o)
{
    const nav_cfg_t *c = &s->cfg;
    memset(o, 0, sizeof *o);
    o->throttle = c->cruise_throttle;

    /* ---- mode handling --------------------------------------------------- */
    if (!in->auto_enable) {
        s->state = NAV_IDLE;
        s->fs_latched = false;
        return;                                   /* caller uses manual path */
    }
    if (s->state == NAV_IDLE) {
        if (s->mission_loaded && in->gps_valid) engage(s, in);
        else return;
    }

    if (in->gps_valid) s->gps_age = 0.0f; else s->gps_age += dt;
    if (s->gps_age > c->gps_timeout_s && s->state != NAV_FAILSAFE) {
        s->state = NAV_FAILSAFE;
        s->fs_alt = in->alt_m;
    }

    float gs    = hypotf(in->vn_mps, in->ve_mps);
    float track = atan2f(in->ve_mps, in->vn_mps);
    float v     = in->airspeed_mps;

    /* Envelope scaling at low airspeed (starts 15 % above min, full at min) */
    float v_prot = 1.15f * c->min_airspeed;
    float k_low  = clampf((v_prot - v) / (v_prot - c->min_airspeed), 0.0f, 1.0f);
    float bank_max  = c->max_bank * (1.0f - 0.5f * k_low);
    float pitch_hi  = c->max_pitch_up * (1.0f - k_low);

    float bank_cmd = 0.0f, alt_tgt = in->alt_m;

    /* ---- guidance -------------------------------------------------------- */
    if (s->state == NAV_TRACK) {
        double pn, pe;
        to_ne(s, in->lat_deg, in->lon_deg, &pn, &pe);

        uint8_t i0 = s->leg, i1 = (uint8_t)(s->leg + 1);
        double  ln = s->rn[i1] - s->rn[i0], le = s->re[i1] - s->re[i0];
        double  len = sqrt(ln * ln + le * le);
        if (len < 1.0) len = 1.0;
        double  dn_ = ln / len, de_ = le / len;

        double  an = pn - s->rn[i0], ae = pe - s->re[i0];
        double  along = an * dn_ + ae * de_;                /* along-track  */
        double  xn = an - along * dn_, xe = ae - along * de_;
        float   xt  = (float)sqrt(xn * xn + xe * xe);       /* |cross-track| */

        float   l1    = l1_dist(s, gs);
        float   ahead = sqrtf(fmaxf(l1 * l1 - xt * xt, 0.0f));
        double  cs    = along + ahead;
        if (cs > len) cs = len;
        if (cs < 0.0) cs = 0.0;
        double  cn = s->rn[i0] + dn_ * cs, ce = s->re[i0] + de_ * cs;

        bank_cmd = l1_bank(s, gs, track, pn, pe, cn, ce);

        /* climb/descend linearly along the leg */
        float f = clampf((float)(along / len), 0.0f, 1.0f);
        alt_tgt = s->route[i0].alt_m + f * (s->route[i1].alt_m - s->route[i0].alt_m);

        /* waypoint reached: inside radius, or crossed the waypoint plane */
        double rn_ = s->rn[i1] - pn, re_ = s->re[i1] - pe;
        double d_wp = sqrt(rn_ * rn_ + re_ * re_);
        if (d_wp < c->accept_radius || along > len) {
            if (i1 == s->n_pts - 1) s->state = NAV_LOITER;   /* reached B */
            else                    s->leg++;
        }
    }
    else if (s->state == NAV_LOITER) {
        double pn, pe, cn, ce;
        to_ne(s, in->lat_deg, in->lon_deg, &pn, &pe);
        cn = s->rn[s->n_pts - 1];
        ce = s->re[s->n_pts - 1];
        alt_tgt = s->dest.alt_m;

        double rn_ = pn - cn, re_ = pe - ce;
        float  r   = (float)sqrt(rn_ * rn_ + re_ * re_);
        float  R   = c->loiter_radius;
        float  theta = (float)atan2(re_, rn_);          /* bearing centre->aircraft */
        float  e   = r - R;                             /* + = outside circle        */
        float  l1  = l1_dist(s, gs);
        /* clockwise orbit: tangent = theta + 90 deg, bent inward when outside */
        float  des = theta + 0.5f * (float)PI + atan2f(e, l1);
        float  err = wrap_pi(des - track);
        float  ff  = (gs > 2.0f) ? atanf(gs * gs / (R * G)) : 0.0f;
        bank_cmd = c->k_hdg * err + ff;
    }
    else { /* NAV_FAILSAFE: wings level, hold altitude, cruise speed */
        bank_cmd = 0.0f;
        alt_tgt  = s->fs_alt;
    }

    bank_cmd = clampf(bank_cmd, -bank_max, bank_max);

    /* ---- vertical -------------------------------------------------------- */
    float pitch_cmd = c->k_alt * (alt_tgt - in->alt_m);
    pitch_cmd += c->k_bank_pitch * (1.0f / fmaxf(cosf(in->roll_rad), 0.5f) - 1.0f);
    pitch_cmd = clampf(pitch_cmd, -c->max_pitch_down, pitch_hi);

    /* ---- throttle: speed PI + climb feed-forward ------------------------- */
    float v_err = c->cruise_airspeed - v;
    s->spd_int  = clampf(s->spd_int + c->ki_speed * v_err * dt, -0.3f, 0.3f);
    float thr   = c->cruise_throttle + c->kp_speed * v_err + s->spd_int
                + c->k_climb_thr * pitch_cmd;
    if (v > c->max_airspeed) thr = 0.0f;
    if (v < c->min_airspeed) thr = 1.0f;
    o->throttle = clampf(thr, 0.0f, 1.0f);

    /* ---- inner loops (attitude -> surfaces) ------------------------------ */
    o->aileron  = clampf(c->kp_roll  * (bank_cmd  - in->roll_rad)  - c->kd_roll  * in->p_rps, -1.0f, 1.0f);
    o->elevator = clampf(c->kp_pitch * (pitch_cmd - in->pitch_rad) - c->kd_pitch * in->q_rps, -1.0f, 1.0f);
    o->rudder   = clampf(c->k_rudder * sinf(in->roll_rad), -1.0f, 1.0f);   /* turn coordination */
    o->flaps    = 0.0f;
}
