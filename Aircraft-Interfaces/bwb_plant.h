/*
 * bwb_plant.h -- blended-wing-body plant model used by the simulator and by the
 * training-label generator (so both see exactly the same aircraft).
 *
 * Effectors (all normalised):
 *   eL, eR   two elevons, + = trailing edge down.  Symmetric part pitches,
 *            differential part rolls.
 *            u_pitch(+nose up) = -(eL+eR)/2        u_roll(+right wing down) = (eL-eR)/2
 *   sL, sR   spoilers 0..1.  Symmetric = airbrake (drag, sink, small nose-up);
 *            differential = roll assist with proverse yaw.
 *   fl       flaps 0..1: lift/drag up, nose-down pitching moment.
 *   dr       rudder (split / winglet rudder), + = nose right.
 *
 * Axes: roll and pitch are first-order rate responses with actuator lag; the
 * yaw axis is a lightly damped Dutch-roll mode driven by rudder, adverse yaw from
 * elevon differential and spoiler drag.  Authority scales with dynamic pressure.
 * Motion is planar (x east, y north) plus altitude.
 */
#ifndef BWB_PLANT_H
#define BWB_PLANT_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define BWB_G   9.80665f
#define BWB_DT  0.02f
#define BWB_V0  18.0f              /* reference airspeed for authority scaling */

/* ---- types shared with the controller ------------------------------------ */
typedef struct { float eL, eR, sL, sR, flaps, rudder, throttle; } bwb_surf_t;

typedef struct {                   /* what the sensors report                  */
    float roll, pitch, p, q, r;    /* attitude, body rates (r = total yaw rate)*/
    float V, alt, heading;
} bwb_meas_t;

typedef struct {                   /* true (hidden) aircraft parameters        */
    float V0;
    float ka, ke;                  /* roll / pitch authority (nominal 2.5 / 2.0) */
    float nr, na, ns;              /* rudder, adverse-yaw, spoiler-yaw (nominal 0.20 / -0.08 / 0.05) */
    float zeta, wd0;               /* Dutch-roll damping, frequency at 18 m/s   */
    float lnu;                     /* dihedral effect                           */
    float gust;                    /* 0..1                                      */
    float wind_n, wind_e;          /* m/s                                       */
    float pbias, qbias;            /* trim rate biases rad/s                    */
    float drag_scale, trim_scale;  /* hidden drag and flap/spoiler moment scale */
    float gyro_noise, att_noise;
} bwb_pp_t;

typedef struct {
    double x, y;                   /* east, north                              */
    float  psi, phi, theta, V, h;
    float  p, q, nu, nu_d;
    float  eL, eR, sL, sR, fl, dr; /* actuator positions                       */
    float  gp, gq, gn;             /* gust states                              */
    uint32_t rng;
} bwb_ps_t;

/* ---- random numbers ------------------------------------------------------- */
static inline float bwb_rand(uint32_t *s)
{
    uint32_t x = *s; x ^= x << 13; x ^= x >> 17; x ^= x << 5; *s = x;
    return (float)(x >> 8) * (1.0f / 16777216.0f);
}
static inline float bwb_randn(uint32_t *s)
{
    float u1 = bwb_rand(s) + 1e-7f, u2 = bwb_rand(s);
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static inline void bwb_pp_nominal(bwb_pp_t *pp, float V0)
{
    memset(pp, 0, sizeof *pp);
    pp->V0 = V0; pp->ka = 2.5f; pp->ke = 2.0f;
    pp->nr = 0.20f; pp->na = -0.08f; pp->ns = 0.05f;
    pp->zeta = 0.08f; pp->wd0 = 2.0f; pp->lnu = 0.6f;
    pp->drag_scale = 1.0f; pp->trim_scale = 1.0f;
    pp->gyro_noise = 0.02f; pp->att_noise = 0.002f;
}

static inline void bwb_ps_init(bwb_ps_t *s, const bwb_pp_t *pp, double x, double y, float psi, float h, uint32_t seed)
{
    memset(s, 0, sizeof *s);
    s->x = x; s->y = y; s->psi = psi; s->V = pp->V0; s->h = h;
    s->rng = seed ? seed : 2463534242u;
}

/* sensors (with noise) */
static inline void bwb_measure(bwb_ps_t *s, const bwb_pp_t *pp, bwb_meas_t *m)
{
    m->roll  = s->phi   + pp->att_noise * bwb_randn(&s->rng);
    m->pitch = s->theta + pp->att_noise * bwb_randn(&s->rng);
    m->p = s->p + s->gp + pp->gyro_noise * bwb_randn(&s->rng);
    m->q = s->q + s->gq + pp->gyro_noise * bwb_randn(&s->rng);
    m->r = BWB_G * tanf(s->phi) / s->V + s->nu_d + pp->gyro_noise * bwb_randn(&s->rng);
    m->V = s->V; m->alt = s->h; m->heading = s->psi;
}

/* one Euler step */
static inline void bwb_plant_step(bwb_ps_t *s, const bwb_pp_t *pp, const bwb_surf_t *c, float dt)
{
    const float vr = s->V / BWB_V0;

    s->eL += (c->eL - s->eL) * (dt / 0.06f);
    s->eR += (c->eR - s->eR) * (dt / 0.06f);
    s->sL += (c->sL - s->sL) * (dt / 0.15f);
    s->sR += (c->sR - s->sR) * (dt / 0.15f);
    s->fl += (c->flaps - s->fl) * (dt / 0.80f);
    s->dr += (c->rudder - s->dr) * (dt / 0.08f);

    /* gusts: Ornstein-Uhlenbeck, correlation time 0.5 s */
    const float a = 0.9607894f, b = 0.2774f;
    s->gp = a * s->gp + b * 0.25f * pp->gust * bwb_randn(&s->rng);
    s->gq = a * s->gq + b * 0.10f * pp->gust * bwb_randn(&s->rng);
    s->gn = a * s->gn + b * 0.03f * pp->gust * bwb_randn(&s->rng);

    const float u_ra  = 0.5f * (s->eL - s->eR);              /* + roll right        */
    const float u_pe  = -0.5f * (s->eL + s->eR);             /* + nose up           */
    const float s_sym = 0.5f * (s->sL + s->sR);
    const float u_rs  = 0.5f * (s->sR - s->sL);              /* spoiler roll assist */

    /* roll */
    float p_ss = pp->ka * vr * (u_ra + u_rs) - pp->lnu * vr * s->nu + pp->pbias;
    s->p += (p_ss - s->p) * (dt / 0.12f);
    s->phi += (s->p + s->gp) * dt;

    /* pitch */
    float q_ss = pp->ke * vr * vr * (u_pe - 0.25f * pp->trim_scale * s->fl + 0.12f * pp->trim_scale * s_sym) + pp->qbias;
    s->q += (q_ss - s->q) * (dt / 0.12f);
    s->theta += (s->q + s->gq) * dt;

    /* Dutch roll */
    float wd = pp->wd0 * vr;
    float force = pp->nr * s->dr + pp->na * u_ra + pp->ns * 2.0f * u_rs + s->gn;
    float nu_dd = wd * wd * (force - s->nu) - 2.0f * pp->zeta * wd * s->nu_d;
    s->nu_d += nu_dd * dt;
    s->nu   += s->nu_d * dt;

    s->psi += (BWB_G * tanf(s->phi) / s->V + s->nu_d) * dt;

    /* speed and altitude */
    float thr = c->throttle < 0.0f ? 0.0f : (c->throttle > 1.0f ? 1.0f : c->throttle);
    float drag = 0.55f * pp->drag_scale * vr * vr + 0.20f * s_sym * vr * vr + 0.10f * s->fl * vr * vr;
    s->V += (12.0f * (thr - drag) - BWB_G * sinf(s->theta)) * dt;
    if (s->V < 5.0f) s->V = 5.0f;
    if (s->V > 60.0f) s->V = 60.0f;
    s->h += (s->V * sinf(s->theta) - 2.5f * s_sym * vr) * dt;

    s->x += ((double)s->V * sin(s->psi) + pp->wind_e) * dt;
    s->y += ((double)s->V * cos(s->psi) + pp->wind_n) * dt;
}

#endif /* BWB_PLANT_H */
