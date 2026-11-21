/*
 * BWB_MRAC.c -- Model Reference Adaptive Control for a blended wing body
 *
 * Same aircraft, same outer loops, protections and allocation as BWB_PID.c (this file
 * includes it and reuses bwb_pid_outer / bwb_pid_alloc).  What changes is the inner loops:
 * instead of fixed PID gains, each loop is forced to behave like a chosen reference model
 * and its controller parameters are adapted online from the model-following error.
 *
 *   roll, pitch  (2nd order)   plant      x' = A x + B (u):  x = [angle, rate],  B = [0, b]
 *                              reference  xm' = Am xm + Bm r   (natural freq wm, damping zm)
 *                              control    u = kx1*angle + kx2*rate + kr*r + kb
 *                              adaptation kx' = -G x s,  kr' = -G r s,  kb' = -G s
 *                                         s = e' P B,  e = x - xm,  Am'P + P Am = -Q  (Lyapunov)
 *   speed        (1st order)   regulation form around the speed demand, parameters kv, kvb
 *
 * The unknown is the control authority b (elevon effectiveness: damage, speed, CG ...), so
 * the loops keep their closed-loop shape when the aircraft changes.  Robustness additions:
 * dead zone on s, sigma-modification (leakage toward the nominal parameters), projection of
 * every parameter into a box, adaptation frozen while the effectors are saturated, and the
 * known dynamic-pressure scaling divided out of the control so only the unknown part adapts.
 * The adaptation uses the Lyapunov-based direct MRAC form (Lavretsky & Wise; Ioannou & Sun).
 *
 * Sign conventions and types are those of bwb_plant.h / BWB_PID.c.  C99, -lm.
 */
#ifndef BWB_MRAC_C
#define BWB_MRAC_C

#include "BWB_PID.c"

typedef struct {
    float wm_roll, zm_roll, wm_pitch, zm_pitch;   /* reference models                      */
    float q1, q2;                                  /* Lyapunov Q = diag(q1, q2)             */
    float ka0, ke0, tau;                           /* nominal authority and rate lag (starting parameters) */
    float gx1, gx2, gr, gb;                        /* adaptation rates                      */
    float sigma, dz, freeze_u;                     /* leakage, dead zone on s, freeze limit */
    float am_v, alpha_v, b_v, gv, gvb, dz_v;       /* speed loop                            */
} bwb_mrac_cfg_t;

typedef struct { float xm1, xm2, kx1, kx2, kr, kb, kx1_0, kx2_0, kr_0, s; } mr_axis_t;

typedef struct {
    bwb_pid_state_t base;                          /* flap and airbrake state (integrators unused) */
    mr_axis_t roll, pitch;
    float dm, kv, kv_0, kvb, vt_prev;
    bool  init, sat_roll, sat_pitch;
} bwb_mrac_state_t;

void bwb_mrac_cfg_default(bwb_mrac_cfg_t *g)
{
    g->wm_roll = 7.0f;  g->zm_roll = 0.7f;
    g->wm_pitch = 7.0f; g->zm_pitch = 0.85f;
    g->q1 = 50.0f; g->q2 = 1.0f;                  /* weight the angle error: small static errors adapt away */
    g->ka0 = 2.5f; g->ke0 = 2.0f; g->tau = 0.12f;
    g->gx1 = 200.0f; g->gx2 = 40.0f; g->gr = 200.0f; g->gb = 20.0f;
    g->sigma = 0.02f; g->dz = 0.001f; g->freeze_u = 1.0f;
    g->am_v = 2.5f; g->alpha_v = 0.73f; g->b_v = 12.0f; g->gv = 1.0f; g->gvb = 0.5f; g->dz_v = 0.004f;
}

void bwb_mrac_reset(bwb_mrac_state_t *s) { memset(s, 0, sizeof *s); }

static float mr_clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* starting parameters: the matching conditions evaluated for the nominal plant */
static void mr_axis_init(mr_axis_t *a, const bwb_mrac_cfg_t *g, float b_nom, float w, float z, float x1, float x2)
{
    float c = w * w, d = 2.0f * z * w, an = 1.0f / g->tau, b0 = b_nom / g->tau;
    a->kx1_0 = a->kx1 = -c / b0;
    a->kx2_0 = a->kx2 = (an - d) / b0;
    a->kr_0  = a->kr  = c / b0;
    a->kb = 0.0f; a->s = 0.0f;
    a->xm1 = x1; a->xm2 = x2;
}

/* one adaptive 2nd-order loop.  Returns the normalised control (known V scaling removed). */
static float mr_axis_step(mr_axis_t *a, const bwb_mrac_cfg_t *g, float w, float z, float x1, float x2,
                          float r, float gscale, bool adapt, float dt)
{
    float c = w * w, d = 2.0f * z * w;
    float p12 = g->q1 / (2.0f * c), p22 = (p12 + 0.5f * g->q2) / d;     /* solves Am'P + P Am = -Q */

    /* reference model */
    a->xm2 += (-c * a->xm1 - d * a->xm2 + c * r) * dt;
    a->xm1 += a->xm2 * dt;

    float e1 = x1 - a->xm1, e2 = x2 - a->xm2;
    float s = p12 * e1 + p22 * e2;                                       /* e' P B / b          */
    a->s = s;
    float u = a->kx1 * x1 + a->kx2 * x2 + a->kr * r + a->kb;

    float sd = fabsf(s) > g->dz ? s - copysignf(g->dz, s) : 0.0f;       /* dead zone           */
    if (adapt) {
        a->kx1 -= dt * g->gx1 * x1 * sd;
        a->kx2 -= dt * g->gx2 * x2 * sd;
        a->kr  -= dt * g->gr  * r  * sd;
        a->kb  -= dt * g->gb  * sd;
    }
    a->kx1 -= dt * g->sigma * (a->kx1 - a->kx1_0);                       /* sigma-modification  */
    a->kx2 -= dt * g->sigma * (a->kx2 - a->kx2_0);
    a->kr  -= dt * g->sigma * (a->kr  - a->kr_0);
    a->kb  -= dt * g->sigma * a->kb;
    a->kx1 = mr_clamp(a->kx1, a->kx1_0 * 6.0f, a->kx1_0 * 0.15f);        /* projection          */
    a->kx2 = mr_clamp(a->kx2, a->kx2_0 - 1.5f, a->kx2_0 + 1.5f);
    a->kr  = mr_clamp(a->kr,  a->kr_0 * 0.15f, a->kr_0 * 6.0f);
    a->kb  = mr_clamp(a->kb, -0.4f, 0.4f);
    return u / gscale;
}

/* speed loop, regulation form: delta = V - V_demand, reference delta_m' = -am delta_m */
static float mr_speed(bwb_mrac_state_t *s, const bwb_mrac_cfg_t *g, float V, float vt, float dt)
{
    float delta = V - vt;
    if (fabsf(vt - s->vt_prev) > 0.05f) { s->dm = delta; s->vt_prev = vt; }   /* new demand = step */
    s->dm -= g->am_v * s->dm * dt;
    float e = delta - s->dm, sv = e / (2.0f * g->am_v);
    float u = s->kv * delta + s->kvb;
    float sd = fabsf(sv) > g->dz_v ? sv - copysignf(g->dz_v, sv) : 0.0f;
    s->kv  -= dt * g->gv  * delta * sd;
    s->kvb -= dt * g->gvb * sd;
    s->kv  -= dt * g->sigma * (s->kv - s->kv_0);
    s->kvb -= dt * g->sigma * s->kvb;
    s->kv  = mr_clamp(s->kv, s->kv_0 * 8.0f, s->kv_0 * 0.2f);
    s->kvb = mr_clamp(s->kvb, -0.3f, 0.3f);
    return u;
}

/* ---------------------------------------------------------------------- */
/*  Inner loops (drop-in for bwb_pid_inner)                                */
/* ---------------------------------------------------------------------- */
void bwb_mrac_inner(bwb_mrac_state_t *st, const bwb_pid_cfg_t *c, const bwb_mrac_cfg_t *g,
                    const bwb_meas_t *z, float bank_cmd, float pitch_cmd, float v_tgt,
                    float dt, bwb_surf_t *o)
{
    if (!st->init) {
        mr_axis_init(&st->roll,  g, g->ka0, g->wm_roll,  g->zm_roll,  z->roll,  z->p);
        mr_axis_init(&st->pitch, g, g->ke0, g->wm_pitch, g->zm_pitch, z->pitch, z->q);
        st->kv_0 = st->kv = (g->alpha_v - g->am_v) / g->b_v;
        st->kvb = 0.0f; st->vt_prev = v_tgt; st->dm = z->V - v_tgt;
        st->init = true;
    }

    /* ---- speed: adaptive throttle on top of the trim feed-forward ------------- */
    float vr = v_tgt / BWB_V0;
    float u_v = mr_speed(st, g, z->V, v_tgt, dt);
    float thr_u = c->thr_trim * vr * vr + u_v + c->k_climb_thr * pitch_cmd;
    float thr = mr_clamp(thr_u, 0.0f, 1.0f);
    if (z->V > c->v_max) thr = 0.0f;
    if (z->V < c->v_min) thr = 1.0f;
    float v_err = v_tgt - z->V, s_req = 0.0f;
    if (thr_u <= 0.05f) s_req = mr_clamp(c->k_spoil_v * (-v_err - c->spoil_v_on), 0.0f, c->spoil_max);
    st->base.s_sym += (s_req - st->base.s_sym) * (dt / 0.4f);

    /* ---- roll and pitch: model-reference adaptive loops ------------------------ */
    float gs_r = z->V / BWB_V0, gs_p = gs_r * gs_r;                  /* known authority scaling */
    float u_roll = mr_axis_step(&st->roll, g, g->wm_roll, g->zm_roll, z->roll, z->p, bank_cmd, gs_r, !st->sat_roll, dt);
    float u_pitch = mr_axis_step(&st->pitch, g, g->wm_pitch, g->zm_pitch, z->pitch, z->q, pitch_cmd, gs_p, !st->sat_pitch, dt)
                  + c->flap_trim * st->base.flap - c->spoil_trim * st->base.s_sym;
    st->sat_roll  = fabsf(u_roll)  > g->freeze_u + 0.5f;             /* elevons + spoilers spent */
    st->sat_pitch = fabsf(u_pitch) > g->freeze_u;

    bwb_pid_alloc(c, 1.0f, z, u_roll, u_pitch, st->base.s_sym, st->base.flap, thr, o);
}

/* ---------------------------------------------------------------------- */
/*  Full chain: demands -> outer loops -> protections -> adaptive inner     */
/* ---------------------------------------------------------------------- */
void bwb_mrac_step(bwb_mrac_state_t *st, const bwb_pid_cfg_t *c, const bwb_mrac_cfg_t *g,
                   const bwb_meas_t *z, const bwb_pid_demand_t *d, float dt, bwb_surf_t *o,
                   float *bank_out, float *pitch_out)
{
    float bank, pitch;
    bwb_pid_outer(&st->base, c, NULL, z, d, dt, &bank, &pitch);
    if (bank_out) *bank_out = bank;
    if (pitch_out) *pitch_out = pitch;
    bwb_mrac_inner(st, c, g, z, bank, pitch, d->speed, dt, o);
}

#endif /* BWB_MRAC_C */
