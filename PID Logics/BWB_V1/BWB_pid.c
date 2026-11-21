/*
 * BWB_PID.c -- plain PID flight control for a blended wing body
 *
 * Aircraft: 2 elevons, 2 spoilers, flaps, rudder, one throttle.
 * No planner, no neural network: just the loops everything else is built on.
 *
 *   demands (heading, altitude, speed)
 *        |  outer loops        heading -> bank,   altitude -> pitch
 *        v
 *   FCS protections            low-speed limits, floor / ceiling, flap schedule
 *        |
 *        v  inner PID loops    roll PID, pitch PID, speed PI, yaw damper
 *        |
 *        v  allocation         elevon mix (pitch priority) + spoiler roll assist + airbrake
 *   eL eR sL sR flaps rudder throttle
 *
 * Built to be extended: every inner-loop gain is multiplied by a slot of the optional
 * array m[PID_NM] (pass NULL for 1.0).  BWB_FCS.c  fills
 * that array from a network, with the same slot order.
 *
 * Sign conventions: roll + = right wing down, pitch + = nose up, elevon + = trailing edge
 * down, rudder + = nose right, heading 0 = north, clockwise positive.
 * Types (bwb_meas_t, bwb_surf_t) and constants come from bwb_plant.h.  C99, -lm.
 */
#ifndef BWB_PID_C
#define BWB_PID_C

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "bwb_plant.h"

/* gain-multiplier slots .  Slot 4 scales the heading loop. */
enum { M_ROLL_PI, M_ROLL_D, M_PITCH_PI, M_PITCH_D, M_OUTER, M_SPEED, M_YAW, PID_NM };

typedef struct {
    /* inner loops */
    float kp_roll, ki_roll, kd_roll;
    float kp_pitch, ki_pitch, kd_pitch;
    float kp_speed, ki_speed, thr_trim;     /* thr_trim: throttle that holds 18 m/s level */
    float k_yaw_damp, k_rud_ff;             /* yaw damper, adverse-yaw feed-forward       */
    float i_lim, i_zone_roll, i_zone_pitch; /* integrators only act when the error is inside the zone (rad) */
    /* outer loops */
    float kp_hdg, kp_alt, k_bank_pitch, k_climb_thr;
    float bank_max, pitch_up_max, pitch_dn_max;
    /* envelope */
    float v_min, v_max;
    float alt_floor, alt_ceiling;
    /* effectors */
    float flap_trim, spoil_trim;            /* nominal pitching moment of flaps / airbrake  */
    float k_spoil_v, spoil_v_on, spoil_max; /* airbrake when fast at idle throttle          */
    float flap_max, flap_rate;
} bwb_pid_cfg_t;

typedef struct { float i_roll, i_pitch, i_speed, s_sym, flap; } bwb_pid_state_t;
typedef struct { float heading, altitude, speed; } bwb_pid_demand_t;

void bwb_pid_cfg_default(bwb_pid_cfg_t *c)
{
    c->kp_roll = 2.2f;  c->ki_roll = 0.35f;  c->kd_roll = 0.11f;
    c->kp_pitch = 2.6f; c->ki_pitch = 0.30f; c->kd_pitch = 0.16f;
    c->kp_speed = 0.40f; c->ki_speed = 0.08f; c->thr_trim = 0.55f;
    c->k_yaw_damp = 0.20f; c->k_rud_ff = 0.40f;
    c->i_lim = 0.3f; c->i_zone_roll = 0.10f; c->i_zone_pitch = 0.08f;

    c->kp_hdg = 1.2f; c->kp_alt = 0.02f; c->k_bank_pitch = 0.10f; c->k_climb_thr = 1.0f;
    c->bank_max = 30.0f * 0.0174533f;
    c->pitch_up_max = 15.0f * 0.0174533f;
    c->pitch_dn_max = 10.0f * 0.0174533f;

    c->v_min = 11.0f; c->v_max = 30.0f;
    c->alt_floor = 40.0f; c->alt_ceiling = 400.0f;

    c->flap_trim = 0.25f; c->spoil_trim = 0.12f;
    c->k_spoil_v = 0.15f; c->spoil_v_on = 2.0f; c->spoil_max = 0.6f;
    c->flap_max = 0.6f; c->flap_rate = 0.15f;
}

void bwb_pid_reset(bwb_pid_state_t *s) { memset(s, 0, sizeof *s); }

static float pid_clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float pid_wrap(float a)
{
    while (a >  3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

/* ---------------------------------------------------------------------- */
/*  Allocation: roll / pitch demands -> elevons, spoilers, rudder          */
/* ---------------------------------------------------------------------- */
void bwb_pid_alloc(const bwb_pid_cfg_t *c, float k_yaw, const bwb_meas_t *z, float u_roll, float u_pitch,
                   float s_sym, float flap, float thr, bwb_surf_t *o)
{
    /* pitch has priority on the elevons; roll beyond the remaining travel goes to
     * differential spoilers (roll assist); the airbrake adds equally to both spoilers */
    float up   = pid_clamp(u_pitch, -1.0f, 1.0f);
    float room = 1.0f - fabsf(up);
    float ura  = pid_clamp(u_roll, -room, room);
    float diff = pid_clamp(2.0f * (u_roll - ura), -1.0f, 1.0f);                   /* (sR - sL)  */
    o->eL = -up + ura;
    o->eR = -up - ura;
    o->sR = pid_clamp(s_sym + fmaxf(diff, 0.0f), 0.0f, 1.0f);
    o->sL = pid_clamp(s_sym + fmaxf(-diff, 0.0f), 0.0f, 1.0f);

    /* rudder: yaw damper on the non-coordinated yaw rate + adverse-yaw feed-forward */
    float r_pert = z->r - BWB_G * tanf(z->roll) / fmaxf(z->V, 5.0f);
    o->rudder = pid_clamp(-c->k_yaw_damp * k_yaw * r_pert + c->k_rud_ff * ura, -1.0f, 1.0f);

    o->flaps = flap;
    o->throttle = thr;
}

/* ---------------------------------------------------------------------- */
/*  Inner loops: attitude and speed PID + allocation to the effectors      */
/* ---------------------------------------------------------------------- */
void bwb_pid_inner(bwb_pid_state_t *st, const bwb_pid_cfg_t *c, const float *m,
                   const bwb_meas_t *z, float bank_cmd, float pitch_cmd, float v_tgt,
                   float dt, bwb_surf_t *o)
{
    float k[PID_NM];
    for (int i = 0; i < PID_NM; i++) k[i] = m ? m[i] : 1.0f;

    /* ---- speed: throttle PI with trim feed-forward; airbrake if fast at idle -- */
    float v_err = v_tgt - z->V;
    float vr = v_tgt / BWB_V0;
    float thr_u = c->thr_trim * vr * vr + c->kp_speed * k[M_SPEED] * v_err + st->i_speed
                + c->k_climb_thr * pitch_cmd;
    if ((thr_u < 1.0f || v_err < 0.0f) && (thr_u > 0.0f || v_err > 0.0f))        /* anti-windup */
        st->i_speed = pid_clamp(st->i_speed + c->ki_speed * k[M_SPEED] * v_err * dt, -c->i_lim, c->i_lim);
    float thr = pid_clamp(thr_u, 0.0f, 1.0f);
    if (z->V > c->v_max) thr = 0.0f;                                              /* overspeed  */
    if (z->V < c->v_min) thr = 1.0f;                                              /* underspeed */
    float s_req = 0.0f;
    if (thr_u <= 0.05f) s_req = pid_clamp(c->k_spoil_v * (-v_err - c->spoil_v_on), 0.0f, c->spoil_max);
    st->s_sym += (s_req - st->s_sym) * (dt / 0.4f);

    /* ---- roll PID (derivative on the gyro) ---------------------------------- */
    float e_r = bank_cmd - z->roll;
    float u_roll = c->kp_roll * k[M_ROLL_PI] * e_r - c->kd_roll * k[M_ROLL_D] * z->p + st->i_roll;
    if (fabsf(u_roll) < 1.0f && fabsf(e_r) < c->i_zone_roll)
        st->i_roll = pid_clamp(st->i_roll + c->ki_roll * k[M_ROLL_PI] * e_r * dt, -c->i_lim, c->i_lim);

    /* ---- pitch PID, with flap / airbrake pitching-moment trim -------------- */
    float e_p = pitch_cmd - z->pitch;
    float u_pitch = c->kp_pitch * k[M_PITCH_PI] * e_p - c->kd_pitch * k[M_PITCH_D] * z->q + st->i_pitch
                  + c->flap_trim * st->flap - c->spoil_trim * st->s_sym;
    if (fabsf(u_pitch) < 1.0f && fabsf(e_p) < c->i_zone_pitch)
        st->i_pitch = pid_clamp(st->i_pitch + c->ki_pitch * k[M_PITCH_PI] * e_p * dt, -c->i_lim, c->i_lim);

    bwb_pid_alloc(c, k[M_YAW], z, u_roll, u_pitch, st->s_sym, st->flap, thr, o);
}

/* ---------------------------------------------------------------------- */
/*  Outer loops + FCS protections: demands -> bank and pitch demands       */
/* ---------------------------------------------------------------------- */
void bwb_pid_outer(bwb_pid_state_t *st, const bwb_pid_cfg_t *c, const float *m,
                   const bwb_meas_t *z, const bwb_pid_demand_t *d, float dt,
                   float *bank_out, float *pitch_out)
{
    float k4 = m ? m[M_OUTER] : 1.0f;

    /* flaps: scheduled on airspeed (they lower the stall speed), rate limited */
    float flap_tgt = c->flap_max * pid_clamp((1.35f * c->v_min - z->V) / (0.35f * c->v_min), 0.0f, 1.0f);
    st->flap += pid_clamp(flap_tgt - st->flap, -c->flap_rate * dt, c->flap_rate * dt);

    /* low-speed envelope: shrink the bank and pitch-up limits as the stall speed nears */
    float vmin_eff = c->v_min * (1.0f - 0.12f * st->flap);
    float v_prot   = 1.15f * vmin_eff;
    float k_low    = pid_clamp((v_prot - z->V) / (v_prot - vmin_eff), 0.0f, 1.0f);
    float bank_max = c->bank_max * (1.0f - 0.5f * k_low);
    float pitch_hi = c->pitch_up_max * (1.0f - k_low);

    /* outer loops */
    float bank  = pid_clamp(c->kp_hdg * k4 * pid_wrap(d->heading - z->heading), -bank_max, bank_max);
    float pitch = c->kp_alt * (d->altitude - z->alt)
                + c->k_bank_pitch * (1.0f / fmaxf(cosf(z->roll), 0.5f) - 1.0f);
    pitch = pid_clamp(pitch, -c->pitch_dn_max, pitch_hi);
    if (z->alt < c->alt_floor)   pitch = fmaxf(pitch, 0.05f);        /* floor protection   */
    if (z->alt > c->alt_ceiling) pitch = fminf(pitch, -0.02f);       /* ceiling protection */

    *bank_out = bank; *pitch_out = pitch;
}

/* ---------------------------------------------------------------------- */
/*  Full chain: demands -> outer loops -> protections -> inner loops       */
/* ---------------------------------------------------------------------- */
void bwb_pid_step(bwb_pid_state_t *st, const bwb_pid_cfg_t *c, const float *m,
                  const bwb_meas_t *z, const bwb_pid_demand_t *d, float dt, bwb_surf_t *o,
                  float *bank_out, float *pitch_out)
{
    float bank, pitch;
    bwb_pid_outer(st, c, m, z, d, dt, &bank, &pitch);
    if (bank_out) *bank_out = bank;
    if (pitch_out) *pitch_out = pitch;
    bwb_pid_inner(st, c, m, z, bank, pitch, d->speed, dt, o);
}

#endif /* BWB_PID_C */
