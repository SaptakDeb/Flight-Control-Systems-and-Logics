/*
 * Inputs: the receiver, an airspeed sensor (pitot) and an altitude sensor (baro/AGL).
 * Pitch is now partly MEASURED through the flight-path angle: gamma = asin(vertical speed / airspeed),
 * with vertical speed derived from the altitude signal.
 *
 *   RC channels (index = channel - 1)
 *     CH1 aileron   CH2 elevator   CH3 throttle   CH4 rudder
 *     CH5 HOLD (latches the inputs given)   CH6 flaps (up / half / full)
 *
 *   MODES
 *     DISARMED : throttle forced to zero until the stick has been at idle once after power-up.
 *     GROUND   : armed, not yet flying (airspeed below flying speed). Direct control, no protections.
 *     NORMAL   : airborne. Shaped sticks + full protection set below.
 *     HOLD     : CH5 high. Latches aileron/elevator/rudder/throttle/flaps at engagement.
 *                Protections stay active on top of the held inputs.
 *     FAILSAFE : RC link lost. Gentle turn, airspeed-keeping elevator loop, flaps up.
 *
 *   AIRSPEED-BASED PROTECTION
 *     - "Flying" latch from airspeed, so the ground is never treated as flight.
 *     - Control gains scheduled by 1/V^2: smaller deflections when fast, larger when slow.
 *     - Stall protection: stall speed is scaled for flaps and dead-reckoned bank. Near it,
 *       up-elevator and aileron authority fade out, a nose-down bias is applied, and a
 *       throttle floor is applied.
 *     - Overspeed protection: throttle cut, nose-down authority fade, gentle pull-up bias near Vne.
 *     - Flap speed limits, and no flap retraction at low speed.
 *   ALTITUDE-BASED PROTECTION
 *     - Ceiling: up-elevator fades out below the ceiling, a nose-down bias and a throttle cap
 *       apply at and above it.
 *     - Floor (optional, needs AGL altitude): no further descent commands when low, and a
 *       pull-up bias plus throttle floor if the sink rate is too high. Bypassed when the aircraft
 *       looks like it is landing (flaps out or throttle at idle).
 *   FLIGHT-PATH PROTECTION
 *     - Elevator tapers to zero as the measured climb/descent angle nears its limits.
 *   STILL OPEN-LOOP
 *     - Bank limit and the fast pitch limiter use dead-reckoned estimates from stick history.
 *       They drift with wind, trim and CG. Tune them or loosen them.
 *   SENSOR FAULTS
 *     - Stale or implausible airspeed: speed protections off, gains nominal.
 *     - Stale or implausible altitude: altitude and flight-path protections off.
 *     - Baro spikes are rejected, and the filters re-initialise when a sensor returns.
 *
 * Conventions: roll right = +, nose up = +, yaw right = +. SI units, angles in radians.
 * Positive elevator = nose up, positive aileron = roll right.
 *
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DEG(x) ((x) * 3.14159265358979f / 180.0f)

enum { CH_AIL = 0, CH_ELE, CH_THR, CH_RUD, CH_HOLD, CH_FLAP, NUM_CH };

/* ------------------------------------------------------------------------- */
typedef struct {
    /* stick shaping */
    float deadband, expo;
    float ele_stick_sign;               /* +1: pulse > 1500 = nose up; flip if your radio differs */

    /* surface authority (fraction of travel) */
    float ail_max, ail_max_flaps, ele_up_max, ele_dn_max, rud_max;

    /* dead-reckoned bank / pitch limiters */
    float bank_max, pitch_up_max, pitch_dn_max;
    float bank_rate_full, pitch_rate_full, tau_bank, tau_pitch, taper;

    /* airspeed (m/s) */
    float vs_clean, vs_full;            /* 1 g stall speed, flaps up / full */
    float vfe_full, vfe_mid;            /* max speed with full / half flaps */
    float vne, v_ref, v_min_sched, sched_min, sched_max;
    float stall_push;                   /* nose-down elevator bias below stall margin */
    float stall_ail_scale;              /* aileron scale at deep stall margin */
    float over_pullup;                  /* pull-up bias at Vne */

    /* altitude (m) and flight path */
    float alt_ceiling, alt_ceiling_margin, ceil_push, ceil_thr_cap;
    float alt_floor;                    /* 0 = disabled; needs AGL altitude */
    float sink_max, floor_pullup, floor_landing_thr;
    float gamma_up_max, gamma_dn_max;   /* rad */

    /* mixing */
    float ail_rud_mix, flap_ele_comp;

    /* throttle */
    float arm_thr_max, thr_max, thr_recovery, hold_breakout;

    /* failsafe */
    float fs_ail, fs_ele, fs_thr, fs_v_target, fs_kv;

    /* sensor processing */
    uint32_t air_timeout_ms;
    float airspeed_plaus_max;
    float tau_v, tau_alt, tau_vz;
    float alt_spike_m; int alt_rej_max;

    /* link monitoring */
    uint32_t rc_timeout_ms;
    int rc_bad_ticks, rc_good_ticks;

    /* output */
    float slew_surf, slew_thr_up, slew_thr_dn, slew_flap, servo_travel_us;
    bool  rev_ail, rev_ele, rev_rud;
} cfg_t;

static const cfg_t CFG = {
    .deadband = 0.03f, .expo = 0.25f, .ele_stick_sign = 1.0f,

    .ail_max = 0.80f, .ail_max_flaps = 0.55f,
    .ele_up_max = 0.70f, .ele_dn_max = 0.55f, .rud_max = 0.85f,

    .bank_max = DEG(45), .pitch_up_max = DEG(20), .pitch_dn_max = DEG(15),
    .bank_rate_full = DEG(100), .pitch_rate_full = DEG(25),
    .tau_bank = 1.5f, .tau_pitch = 2.5f, .taper = 0.25f,

    .vs_clean = 11.0f, .vs_full = 9.5f, .vfe_full = 16.0f, .vfe_mid = 19.0f, .vne = 30.0f,
    .v_ref = 18.0f, .v_min_sched = 8.0f, .sched_min = 0.5f, .sched_max = 1.8f,
    .stall_push = 0.25f, .stall_ail_scale = 0.5f, .over_pullup = 0.10f,

    .alt_ceiling = 120.0f, .alt_ceiling_margin = 20.0f, .ceil_push = 0.10f, .ceil_thr_cap = 0.40f,
    .alt_floor = 0.0f, .sink_max = 4.0f, .floor_pullup = 0.20f, .floor_landing_thr = 0.20f,
    .gamma_up_max = DEG(15), .gamma_dn_max = DEG(15),

    .ail_rud_mix = 0.25f, .flap_ele_comp = -0.05f,

    .arm_thr_max = 0.05f, .thr_max = 1.0f, .thr_recovery = 0.85f, .hold_breakout = 0.8f,

    .fs_ail = 0.10f, .fs_ele = 0.0f, .fs_thr = 0.35f, .fs_v_target = 15.0f, .fs_kv = 0.04f,

    .air_timeout_ms = 200, .airspeed_plaus_max = 60.0f,
    .tau_v = 0.20f, .tau_alt = 0.25f, .tau_vz = 0.60f,
    .alt_spike_m = 10.0f, .alt_rej_max = 50,

    .rc_timeout_ms = 250, .rc_bad_ticks = 10, .rc_good_ticks = 30,

    .slew_surf = 8.0f, .slew_thr_up = 2.0f, .slew_thr_dn = 3.0f, .slew_flap = 0.4f,
    .servo_travel_us = 500.0f,
    .rev_ail = false, .rev_ele = false, .rev_rud = false,
};

/* ------------------------------------------------------------------------- */
typedef struct {
    uint16_t us[NUM_CH];
    uint32_t stamp_ms;
} rc_frame_t;

typedef struct {
    float airspeed;                     /* m/s */
    float alt;                          /* m */
    bool  airspeed_valid, alt_valid;
    uint32_t stamp_ms;
} air_t;

typedef enum { MODE_DISARMED, MODE_GROUND, MODE_NORMAL, MODE_HOLD, MODE_FAILSAFE } fcs_mode_t;

typedef struct { float ail, ele, thr, rud, flap; } surf_t;

typedef struct {
    fcs_mode_t mode;
    bool  armed, airborne;
    float t_slow;

    uint16_t rc_last[NUM_CH];
    int   rc_bad, rc_good;
    bool  rc_ok;

    /* filtered air data */
    float v, alt, vz;                   /* airspeed, altitude, vertical speed */
    bool  asp_ok, alt_ok, v_init, h_init;
    int   alt_rej;

    bool  hold, hold_sw_prev, hold_lockout;
    float hold_ail, hold_ele, hold_rud, hold_thr, hold_flap;

    float bank_est, pitch_est;          /* dead-reckoned */
    surf_t out;                         /* post-slew outputs */
} fcs_t;

/* ------------------------------------------------------------------------- */
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float lerpf(float a, float b, float t)    { return a + (b - a) * t; }
static inline float slewf(float cur, float tgt, float rate, float dt)
{
    float d = rate * dt;
    return cur + clampf(tgt - cur, -d, d);
}

static float rc_norm(uint16_t us) { return clampf(((float)us - 1500.0f) / 500.0f, -1.0f, 1.0f); }
static float rc_unit(uint16_t us) { return clampf(((float)us - 1000.0f) / 1000.0f, 0.0f, 1.0f); }

static float shape_stick(float x)
{
    float ax = fabsf(x);
    if (ax < CFG.deadband) return 0.0f;
    float y = (ax - CFG.deadband) / (1.0f - CFG.deadband);
    y = (1.0f - CFG.expo) * y + CFG.expo * y * y * y;
    return copysignf(y, x);
}

static float flap_selector(uint16_t us) { return us < 1300 ? 0.0f : (us < 1700 ? 0.5f : 1.0f); }

/* Fade a command to zero as the estimate nears the limit in the direction the command pushes. */
static float taper_cmd(float est, float lim_pos, float lim_neg, float cmd)
{
    if (cmd > 0.0f) return cmd * clampf((lim_pos - est) / (CFG.taper * lim_pos), 0.0f, 1.0f);
    if (cmd < 0.0f) return cmd * clampf((lim_neg + est) / (CFG.taper * lim_neg), 0.0f, 1.0f);
    return cmd;
}

static void fcs_init(fcs_t *f)
{
    memset(f, 0, sizeof *f);
    f->mode = MODE_DISARMED;
    for (int i = 0; i < NUM_CH; i++) f->rc_last[i] = 1500;
    f->rc_last[CH_THR] = 1000; f->rc_last[CH_HOLD] = 1000; f->rc_last[CH_FLAP] = 1000;
}

/* ------------------------------------------------------------------------- */
/* Air data validation and filtering                                         */
/* ------------------------------------------------------------------------- */
static void air_process(fcs_t *f, const air_t *a, uint32_t now_ms, float dt)
{
    const cfg_t *c = &CFG;
    bool fresh = (uint32_t)(now_ms - a->stamp_ms) < c->air_timeout_ms;
    bool v_ok = fresh && a->airspeed_valid && isfinite(a->airspeed) &&
                a->airspeed >= 0.0f && a->airspeed < c->airspeed_plaus_max;
    bool h_ok = fresh && a->alt_valid && isfinite(a->alt) && a->alt > -100.0f && a->alt < 10000.0f;

    if (v_ok) {
        if (!f->v_init) { f->v = a->airspeed; f->v_init = true; }
        else            f->v += (a->airspeed - f->v) * dt / (c->tau_v + dt);
    } else {
        f->v_init = false;
    }
    f->asp_ok = v_ok;

    if (h_ok) {
        if (!f->h_init || f->alt_rej >= c->alt_rej_max) {   /* first sample, or the jump persisted: accept it */
            f->alt = a->alt; f->vz = 0.0f; f->h_init = true; f->alt_rej = 0;
        } else if (fabsf(a->alt - f->alt) > c->alt_spike_m) {
            f->alt_rej++;                                   /* spike: ignore this sample */
        } else {
            f->alt_rej = 0;
            float alt_new = f->alt + (a->alt - f->alt) * dt / (c->tau_alt + dt);
            float vz_raw  = (alt_new - f->alt) / dt;
            f->vz = clampf(f->vz + (vz_raw - f->vz) * dt / (c->tau_vz + dt), -15.0f, 15.0f);
            f->alt = alt_new;
        }
    } else {
        f->h_init = false; f->vz = 0.0f; f->alt_rej = 0;
    }
    f->alt_ok = h_ok;
}

/* ------------------------------------------------------------------------- */
void fcs_step(fcs_t *f, const rc_frame_t *rc, const air_t *air, uint32_t now_ms,
              float dt, uint16_t out_us[NUM_CH])
{
    const cfg_t *c = &CFG;

    /* ---- 1. RC link monitoring (debounced); last good frame is used meanwhile ---- */
    bool frame_ok = (uint32_t)(now_ms - rc->stamp_ms) < c->rc_timeout_ms;
    for (int i = 0; i < NUM_CH; i++)
        if (rc->us[i] < 880 || rc->us[i] > 2120) frame_ok = false;
    if (frame_ok) {
        memcpy(f->rc_last, rc->us, sizeof f->rc_last);
        f->rc_bad = 0;
        if (f->rc_good < 10000) f->rc_good++;
    } else {
        f->rc_good = 0;
        if (f->rc_bad < 10000) f->rc_bad++;
    }
    if (f->rc_ok && f->rc_bad >= c->rc_bad_ticks)         f->rc_ok = false;
    else if (!f->rc_ok && f->rc_good >= c->rc_good_ticks) f->rc_ok = true;

    /* ---- 2. Air data ---- */
    air_process(f, air, now_ms, dt);

    /* ---- 3. "Flying" latch from airspeed (frozen if airspeed is invalid) ---- */
    if (f->asp_ok) {
        if (!f->airborne) {
            if (f->v > 1.2f * c->vs_clean) { f->airborne = true; f->t_slow = 0.0f; }
        } else if (f->v < 0.5f * c->vs_clean) {
            f->t_slow += dt;
            if (f->t_slow > 2.0f) { f->airborne = false; f->armed = false; }
        } else {
            f->t_slow = 0.0f;
        }
    }

    /* ---- 4. Pilot inputs ---- */
    float st_ail = shape_stick(rc_norm(f->rc_last[CH_AIL]));
    float st_ele = shape_stick(rc_norm(f->rc_last[CH_ELE])) * c->ele_stick_sign;
    float st_rud = shape_stick(rc_norm(f->rc_last[CH_RUD]));
    float st_thr = rc_unit(f->rc_last[CH_THR]);
    float flap_sel = flap_selector(f->rc_last[CH_FLAP]);
    bool  hold_sw = f->rc_last[CH_HOLD] > 1700;

    /* ---- 5. Arm interlock: link good and throttle at idle once after power-up ---- */
    if (f->airborne) f->armed = true;
    else if (!f->armed && f->rc_ok && st_thr < c->arm_thr_max) f->armed = true;

    /* ---- 6. HOLD switch: rising edge latches the stick inputs ---- */
    if (!f->rc_ok || !f->armed || !f->airborne || !hold_sw) {
        f->hold = false;
        if (!hold_sw) f->hold_lockout = false;
    } else if (!f->hold_sw_prev && !f->hold_lockout) {
        f->hold = true;
        f->hold_ail = st_ail; f->hold_ele = st_ele; f->hold_rud = st_rud;
        f->hold_thr = st_thr; f->hold_flap = flap_sel;
    }
    if (f->hold && (fabsf(st_ail) > c->hold_breakout || fabsf(st_ele) > c->hold_breakout)) {
        f->hold = false;                          /* pilot override: toggle the switch to re-engage */
        f->hold_lockout = true;
    }
    f->hold_sw_prev = hold_sw;

    /* ---- 7. Mode ---- */
    if (!f->rc_ok)           f->mode = MODE_FAILSAFE;
    else if (!f->armed)      f->mode = MODE_DISARMED;
    else if (!f->airborne)   f->mode = MODE_GROUND;
    else if (f->hold)        f->mode = MODE_HOLD;
    else                     f->mode = MODE_NORMAL;

    bool prot = f->airborne && f->mode != MODE_DISARMED && f->mode != MODE_GROUND;

    /* ---- 8. Command selection (before protection) ---- */
    float ail, ele, rud, thr, flap;
    switch (f->mode) {
    case MODE_FAILSAFE:
        if (!f->airborne) { ail = ele = rud = thr = flap = 0.0f; break; }
        ail = c->fs_ail;
        ele = f->asp_ok ? clampf(c->fs_kv * (f->v - c->fs_v_target), -0.3f, 0.3f) : c->fs_ele;
        rud = 0.0f; thr = c->fs_thr; flap = 0.0f;
        break;
    case MODE_HOLD:
        ail = f->hold_ail; ele = f->hold_ele; rud = f->hold_rud;
        thr = fminf(f->hold_thr, st_thr);         /* pilot can always pull power back */
        flap = f->hold_flap;
        break;
    case MODE_NORMAL:
    case MODE_GROUND:
        ail = st_ail; ele = st_ele; rud = st_rud; thr = st_thr; flap = flap_sel;
        break;
    default:
        ail = st_ail; ele = st_ele; rud = st_rud; thr = 0.0f; flap = flap_sel;
        break;
    }

    float flap_now = f->out.flap;
    float v = f->asp_ok ? f->v : c->v_ref;
    bool use_v = prot && f->asp_ok;
    bool use_h = prot && f->alt_ok;

    /* ---- 9. Airspeed protection terms ---- */
    float cb = fmaxf(cosf(f->bank_est), 0.5f);
    float vs_eff = lerpf(c->vs_clean, c->vs_full, flap_now) / sqrtf(cb);
    float sched = 1.0f, k_stall = 1.0f, b_stall = 0.0f, k_over_dn = 1.0f, b_over = 0.0f, ko = 0.0f;
    if (use_v) {
        float r = c->v_ref / fmaxf(v, c->v_min_sched);
        sched = clampf(r * r, c->sched_min, c->sched_max);
        float v_lo = 1.15f * vs_eff, v_hi = 1.30f * vs_eff;
        k_stall = clampf((v - v_lo) / (v_hi - v_lo), 0.0f, 1.0f);
        b_stall = -c->stall_push * clampf((v_lo - v) / (0.10f * vs_eff), 0.0f, 1.0f);
        ko = clampf((v - 0.90f * c->vne) / (0.10f * c->vne), 0.0f, 1.0f);
        k_over_dn = 1.0f - ko;
        b_over = c->over_pullup * ko;
    }

    /* ---- 10. Altitude / flight-path protection terms ---- */
    float k_ceil = 1.0f, kc = 0.0f, b_ceil = 0.0f, k_floor_dn = 1.0f, b_floor = 0.0f, thr_floor = 0.0f;
    float gamma = 0.0f;
    if (use_h) {
        float h0 = c->alt_ceiling - c->alt_ceiling_margin;
        kc = clampf((f->alt - h0) / c->alt_ceiling_margin, 0.0f, 1.0f);
        k_ceil = 1.0f - kc;
        b_ceil = -c->ceil_push * clampf((f->alt - c->alt_ceiling) / c->alt_ceiling_margin, 0.0f, 1.0f);

        bool landing_cfg = flap_now >= 0.45f || thr <= c->floor_landing_thr;
        if (c->alt_floor > 0.0f && f->alt < c->alt_floor && !landing_cfg) {
            k_floor_dn = clampf((f->alt - 0.5f * c->alt_floor) / (0.5f * c->alt_floor), 0.0f, 1.0f);
            if (f->vz < -c->sink_max) {
                float k = clampf((-f->vz - c->sink_max) / 2.0f, 0.0f, 1.0f);
                b_floor = c->floor_pullup * k;
                thr_floor = 0.7f * k;
            }
        }
        if (use_v) gamma = asinf(clampf(f->vz / fmaxf(v, c->v_min_sched), -0.9f, 0.9f));
    }

    /* ---- 11. Ailerons / elevator / rudder ---- */
    ail *= sched * lerpf(c->stall_ail_scale, 1.0f, k_stall);
    ele *= sched;
    rud *= sched;

    float ail_lim = lerpf(c->ail_max, c->ail_max_flaps, flap_now);
    ail = clampf(ail, -ail_lim, ail_lim);
    ele = clampf(ele, -c->ele_dn_max, c->ele_up_max);
    rud = clampf(rud, -c->rud_max, c->rud_max);

    if (prot) {
        ele *= (ele > 0.0f) ? (k_stall * k_ceil) : (k_over_dn * k_floor_dn);
        ail = taper_cmd(f->bank_est,  c->bank_max,     c->bank_max,     ail);
        ele = taper_cmd(f->pitch_est, c->pitch_up_max, c->pitch_dn_max, ele);
        if (use_h && use_v) ele = taper_cmd(gamma, c->gamma_up_max, c->gamma_dn_max, ele);
        ele += b_stall + b_over + b_ceil + b_floor;
        ele = clampf(ele, -1.0f, 1.0f);
        f->bank_est  += (c->bank_rate_full  * ail - f->bank_est  / c->tau_bank)  * dt;
        f->pitch_est += (c->pitch_rate_full * ele - f->pitch_est / c->tau_pitch) * dt;
    } else {
        f->bank_est = f->pitch_est = 0.0f;
    }

    /* ---- 12. Throttle ---- */
    float thr_out = thr;
    if (prot) {
        thr_out = fminf(thr_out, lerpf(1.0f, c->ceil_thr_cap, kc));         /* ceiling cap */
        if (use_v) {
            float kt = clampf((1.25f * vs_eff - v) / (0.25f * vs_eff), 0.0f, 1.0f);
            thr_out = fmaxf(thr_out, kt * c->thr_recovery);                 /* low-speed power floor */
        }
        thr_out = fmaxf(thr_out, thr_floor);                                /* sink-rate power floor */
        thr_out *= (1.0f - ko);                                             /* overspeed cut */
    }
    thr_out = f->armed ? thr_out * c->thr_max : 0.0f;

    /* ---- 13. Flaps ---- */
    if (use_v && f->mode != MODE_FAILSAFE) {
        float allowed = v > c->vfe_mid ? 0.0f : (v > c->vfe_full ? 0.5f : 1.0f);
        flap = fminf(flap, allowed);
        if (flap < flap_now && v < 1.3f * c->vs_clean && allowed >= flap_now)
            flap = flap_now;                                    /* don't retract when slow */
    }

    /* ---- 14. Mixing, clamp, slew, scale to servo pulses ---- */
    surf_t tgt;
    tgt.ail  = clampf(ail, -1.0f, 1.0f);
    tgt.ele  = clampf(ele + c->flap_ele_comp * flap_now, -1.0f, 1.0f);
    tgt.rud  = clampf(rud + c->ail_rud_mix * ail, -1.0f, 1.0f);
    tgt.thr  = clampf(thr_out, 0.0f, 1.0f);
    tgt.flap = clampf(flap, 0.0f, 1.0f);

    f->out.ail  = slewf(f->out.ail,  tgt.ail,  c->slew_surf, dt);
    f->out.ele  = slewf(f->out.ele,  tgt.ele,  c->slew_surf, dt);
    f->out.rud  = slewf(f->out.rud,  tgt.rud,  c->slew_surf, dt);
    f->out.thr  = slewf(f->out.thr,  tgt.thr,  tgt.thr > f->out.thr ? c->slew_thr_up : c->slew_thr_dn, dt);
    f->out.flap = slewf(f->out.flap, tgt.flap, c->slew_flap, dt);

    float a = c->rev_ail ? -f->out.ail : f->out.ail;
    float e = c->rev_ele ? -f->out.ele : f->out.ele;
    float r = c->rev_rud ? -f->out.rud : f->out.rud;
    out_us[CH_AIL]  = (uint16_t)(1500.0f + a * c->servo_travel_us + 0.5f);
    out_us[CH_ELE]  = (uint16_t)(1500.0f + e * c->servo_travel_us + 0.5f);
    out_us[CH_THR]  = (uint16_t)(1000.0f + f->out.thr * 1000.0f + 0.5f);
    out_us[CH_RUD]  = (uint16_t)(1500.0f + r * c->servo_travel_us + 0.5f);
    out_us[CH_HOLD] = f->hold ? 2000 : 1000;      /* status output (LED / telemetry): HOLD active */
    out_us[CH_FLAP] = (uint16_t)(1000.0f + f->out.flap * 1000.0f + 0.5f);
}

/* ------------------------------------------------------------------------- */
#ifndef FCS_SIM
extern uint32_t hal_millis(void);
extern void     hal_wait_next_tick(void);          /* blocks until next 100 Hz tick */
extern void     hal_read_rc(rc_frame_t *rc);
extern void     hal_read_air(air_t *air);          /* pitot + baro/AGL, with valid flags and stamp */
extern void     hal_write_servos(const uint16_t us[NUM_CH]);

void fcs_run(void)
{
    static fcs_t fcs;
    rc_frame_t rc; air_t air; uint16_t out[NUM_CH];
    fcs_init(&fcs);
    for (;;) {
        hal_wait_next_tick();
        hal_read_rc(&rc);
        hal_read_air(&air);
        fcs_step(&fcs, &rc, &air, hal_millis(), 0.01f, out);
        hal_write_servos(out);
    }
}
#endif

/* ------------------------------------------------------------------------- */
/* Demo with a toy plant (pitch / speed / altitude only; NOT an aero model)  */
/* ------------------------------------------------------------------------- */
#ifdef FCS_SIM
typedef struct { float theta, q, v, alt; } plant_t;

static void plant_step(plant_t *x, const surf_t *u, float dt)
{
    float stall_m = x->v < 10.5f ? -6.0f * (10.5f - x->v) : 0.0f;   /* nose drops when very slow */
    x->q     += (8.0f * u->ele - 5.0f * x->q - 12.0f * x->theta + stall_m) * dt;
    x->theta += x->q * dt;
    x->v     += (0.4f * (30.0f * u->thr - x->v) - 6.0f * sinf(x->theta)) * dt;
    if (x->v < 0.0f) x->v = 0.0f;
    x->alt   += x->v * sinf(x->theta) * dt;
}

static uint16_t us_from(float s) { return (uint16_t)(1500.0f + 500.0f * s); }

int main(void)
{
    fcs_t f; fcs_init(&f);
    f.armed = true; f.rc_ok = true; f.airborne = true;            /* start in flight */
    plant_t x = { .v = 18.0f, .alt = 60.0f };
    const float dt = 0.01f;
    const char *names[] = { "DISARM", "GROUND", "NORMAL", "HOLD  ", "FAILSF" };
    printf("  t   mode     V    alt   vz  gamma | ail  ele  thr  rud flap | pitch\n");

    for (int k = 0; k < 7000; k++) {
        float t = k * dt;
        uint32_t now = (uint32_t)(k * 10);
        float ail = 0, ele = 0, rud = 0, thr = 0.60f, flap = 0, hold = 0;

        if (t >= 3  && t < 20)  ele = 1.0f;                       /* full back stick: climb, ceiling, stall margin */
        if (t >= 20 && t < 32) { ele = -1.0f; thr = 1.0f; }       /* full forward, full power: overspeed */
        if (t >= 38 && t < 46)  flap = 1.0f;                      /* flaps: speed limited */
        if (t >= 48 && t < 50)  ele = 0.25f;                      /* gentle nose-up... */
        if (t >= 49 && t < 60)  hold = 1;                         /* ...latched, sticks released at t=50 */
        bool link_lost = (t >= 62 && t < 66);

        rc_frame_t rc = { .stamp_ms = link_lost ? (uint32_t)(62 * 1000) : now };
        rc.us[CH_AIL] = us_from(ail); rc.us[CH_ELE] = us_from(ele); rc.us[CH_RUD] = us_from(rud);
        rc.us[CH_THR] = (uint16_t)(1000 + 1000 * thr);
        rc.us[CH_HOLD] = hold > 0 ? 2000 : 1000;
        rc.us[CH_FLAP] = flap > 0 ? 2000 : 1000;

        air_t air = { .airspeed = x.v, .alt = x.alt, .airspeed_valid = true, .alt_valid = true, .stamp_ms = now };
        if (t >= 70 && t < 71) air.alt += 25.0f;                  /* baro spike: must be rejected */

        uint16_t out[NUM_CH];
        fcs_step(&f, &rc, &air, now, dt, out);
        plant_step(&x, &f.out, dt);

        if (k % 100 == 0) {
            float g = asinf(clampf(f.vz / fmaxf(f.v, 5.0f), -0.9f, 0.9f)) * 180.0f / 3.14159265f;
            printf("%5.1f  %s %5.1f %6.1f %5.1f %5.1f | %4d %4d %4d %4d %4d | %5.1f\n", t, names[f.mode],
                   f.v, f.alt, f.vz, g, out[CH_AIL], out[CH_ELE], out[CH_THR], out[CH_RUD], out[CH_FLAP],
                   x.theta * 180.0f / 3.14159265f);
        }
    }
    return 0;
}
#endif
