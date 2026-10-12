/*
 *   RC channels (index = channel - 1)
 *     CH1 aileron   CH2 elevator   CH3 throttle   CH4 rudder
 *     CH5 HOLD (latches the inputs given)   CH6 flaps (up / half / full)
 *
 *   MODES
 *     DISARMED : throttle forced to zero until the throttle stick has been at idle
 *                once after power-up. Surfaces follow the sticks (ground checks).
 *     NORMAL   : shaped stick inputs, authority limits, mixing, dead-reckoned
 *                bank/pitch limiters, slew limits.
 *     HOLD     : CH5 high. Latches aileron/elevator/rudder/throttle/flaps as they were
 *                at the moment of engagement. The same limiters still apply.
 *     FAILSAFE : RC link lost. Preset gentle turn, flaps up, throttle per config.
 *
 *   PROTECTIONS (all open-loop)
 *     - Surface authority limits (less aileron with flaps out) -> caps turn rate,
 *       pitch rate and control-induced overstress.
 *     - Dead-reckoned bank and pitch limiters: the code integrates the commanded
 *       roll/pitch rate and tapers the command to zero as the ESTIMATE nears the limit.
 *       This is an ESTIMATE built from stick history and an assumed self-levelling
 *       tendency (dihedral, static stability). It is not a measurement, so it drifts
 *       with wind, trim, CG and airspeed. Tune it on your airframe or set the limits
 *       huge to disable it.
 *     - Throttle arm interlock, throttle slew limit, pilot can always pull throttle
 *       back while in HOLD.
 *     - Flap slew limit and elevator compensation for flap pitch change.
 *     - Aileron-to-rudder mix to reduce adverse yaw.
 *     - Debounced link-loss detection with failsafe.
 *
 * Conventions: roll right = +, nose up = +, yaw right = +. Angles in radians.
 * Positive aileron = roll right, positive elevator = nose up, positive rudder = nose right.
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

    /* surface authority (fraction of full travel) */
    float ail_max, ail_max_flaps;       /* aileron limit, flaps up / flaps full */
    float ele_up_max, ele_dn_max;
    float rud_max;

    /* dead-reckoned attitude limiters */
    float bank_max;                     /* rad */
    float pitch_up_max, pitch_dn_max;   /* rad, magnitudes */
    float bank_rate_full;               /* rad/s of roll per full aileron (assumed) */
    float pitch_rate_full;              /* rad/s of pitch per full elevator (assumed) */
    float tau_bank, tau_pitch;          /* s, assumed self-levelling time constants */
    float taper;                        /* fraction of the limit over which command fades to 0 */

    /* mixing */
    float ail_rud_mix;                  /* rudder per unit aileron */
    float flap_ele_comp;                /* elevator per unit flap (sign: nose-up positive) */

    /* throttle */
    float arm_thr_max;                  /* throttle must be below this to arm */
    float thr_max;                      /* overall throttle cap, 0..1 */
    float hold_min_thr;                 /* HOLD refuses to engage below this throttle */
    float hold_breakout;                /* stick deflection that cancels HOLD (>1 disables) */

    /* failsafe */
    float fs_ail, fs_ele, fs_thr;       /* fs_thr = 0 is safe on the ground; raise for glide power */

    /* link monitoring */
    uint32_t rc_timeout_ms;
    int rc_bad_ticks, rc_good_ticks;

    /* output */
    float slew_surf, slew_thr_up, slew_thr_dn, slew_flap;  /* units per second */
    float servo_travel_us;
    bool  rev_ail, rev_ele, rev_rud;
} cfg_t;

static const cfg_t CFG = {
    .deadband = 0.03f, .expo = 0.25f, .ele_stick_sign = 1.0f,

    .ail_max = 0.80f, .ail_max_flaps = 0.55f,
    .ele_up_max = 0.70f, .ele_dn_max = 0.55f, .rud_max = 0.85f,

    .bank_max = DEG(45), .pitch_up_max = DEG(20), .pitch_dn_max = DEG(15),
    .bank_rate_full = DEG(100), .pitch_rate_full = DEG(25),
    .tau_bank = 1.5f, .tau_pitch = 2.5f, .taper = 0.25f,

    .ail_rud_mix = 0.25f, .flap_ele_comp = -0.05f,

    .arm_thr_max = 0.05f, .thr_max = 1.0f, .hold_min_thr = 0.15f, .hold_breakout = 0.8f,

    .fs_ail = 0.10f, .fs_ele = 0.0f, .fs_thr = 0.0f,

    .rc_timeout_ms = 250, .rc_bad_ticks = 10, .rc_good_ticks = 30,

    .slew_surf = 8.0f, .slew_thr_up = 2.0f, .slew_thr_dn = 3.0f, .slew_flap = 0.4f,
    .servo_travel_us = 500.0f,
    .rev_ail = false, .rev_ele = false, .rev_rud = false,
};

/* ------------------------------------------------------------------------- */
typedef struct {
    uint16_t us[NUM_CH];                /* pulse widths, nominal 1000..2000 */
    uint32_t stamp_ms;
} rc_frame_t;

typedef enum { MODE_DISARMED, MODE_NORMAL, MODE_HOLD, MODE_FAILSAFE } fcs_mode_t;

typedef struct { float ail, ele, thr, rud, flap; } surf_t;

typedef struct {
    fcs_mode_t mode;
    bool  armed;

    uint16_t rc_last[NUM_CH];
    int   rc_bad, rc_good;
    bool  rc_ok;

    bool  hold, hold_sw_prev, hold_lockout;
    float hold_ail, hold_ele, hold_rud, hold_thr, hold_flap;
    float cmd_ail, cmd_ele;             /* last limited commands (what HOLD latches) */

    float bank_est, pitch_est;          /* dead-reckoned, see header */
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
void fcs_step(fcs_t *f, const rc_frame_t *rc, uint32_t now_ms, float dt, uint16_t out_us[NUM_CH])
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

    /* ---- 2. Pilot inputs ---- */
    float st_ail = shape_stick(rc_norm(f->rc_last[CH_AIL]));
    float st_ele = shape_stick(rc_norm(f->rc_last[CH_ELE])) * c->ele_stick_sign;
    float st_rud = shape_stick(rc_norm(f->rc_last[CH_RUD]));
    float st_thr = rc_unit(f->rc_last[CH_THR]);
    float flap_sel = flap_selector(f->rc_last[CH_FLAP]);
    bool  hold_sw = f->rc_last[CH_HOLD] > 1700;

    /* ---- 3. Arm interlock: link good and throttle at idle once after power-up ---- */
    if (!f->armed && f->rc_ok && st_thr < c->arm_thr_max) f->armed = true;

    /* ---- 4. HOLD switch ---- */
    if (!f->rc_ok || !f->armed || !hold_sw) {
        f->hold = false;
        if (!hold_sw) f->hold_lockout = false;
    } else if (!f->hold_sw_prev && !f->hold_lockout && st_thr >= c->hold_min_thr) {
        f->hold = true;                                     /* rising edge: latch inputs */
        f->hold_ail = f->cmd_ail; f->hold_ele = f->cmd_ele;
        f->hold_rud = st_rud;     f->hold_thr = st_thr;  f->hold_flap = flap_sel;
    }
    if (f->hold && (fabsf(st_ail) > c->hold_breakout || fabsf(st_ele) > c->hold_breakout)) {
        f->hold = false;                                    /* pilot override: toggle switch to re-engage */
        f->hold_lockout = true;
    }
    f->hold_sw_prev = hold_sw;

    /* ---- 5. Mode ---- */
    if (!f->rc_ok)        f->mode = MODE_FAILSAFE;
    else if (!f->armed)   f->mode = MODE_DISARMED;
    else if (f->hold)     f->mode = MODE_HOLD;
    else                  f->mode = MODE_NORMAL;

    /* ---- 6. Command selection ---- */
    float ail, ele, rud, thr, flap;
    switch (f->mode) {
    case MODE_FAILSAFE: ail = c->fs_ail; ele = c->fs_ele; rud = 0.0f; thr = c->fs_thr; flap = 0.0f; break;
    case MODE_HOLD:     ail = f->hold_ail; ele = f->hold_ele; rud = f->hold_rud;
                        thr = fminf(f->hold_thr, st_thr);   /* pilot can always pull power back */
                        flap = f->hold_flap; break;
    case MODE_NORMAL:   ail = st_ail; ele = st_ele; rud = st_rud; thr = st_thr; flap = flap_sel; break;
    default:            ail = st_ail; ele = st_ele; rud = st_rud; thr = 0.0f;   flap = flap_sel; break;
    }

    /* ---- 7. Authority limits ---- */
    float ail_lim = lerpf(c->ail_max, c->ail_max_flaps, f->out.flap);
    ail = clampf(ail, -ail_lim, ail_lim);
    ele = clampf(ele, -c->ele_dn_max, c->ele_up_max);
    rud = clampf(rud, -c->rud_max, c->rud_max);

    /* ---- 8. Dead-reckoned bank / pitch limiters ---- */
    if (f->mode == MODE_DISARMED) {
        f->bank_est = f->pitch_est = 0.0f;
    } else {
        ail = taper_cmd(f->bank_est,  c->bank_max,     c->bank_max,     ail);
        ele = taper_cmd(f->pitch_est, c->pitch_up_max, c->pitch_dn_max, ele);
        f->bank_est  += (c->bank_rate_full  * ail - f->bank_est  / c->tau_bank)  * dt;
        f->pitch_est += (c->pitch_rate_full * ele - f->pitch_est / c->tau_pitch) * dt;
    }
    f->cmd_ail = ail; f->cmd_ele = ele;

    /* ---- 9. Mixing ---- */
    surf_t tgt;
    tgt.ail  = ail;
    tgt.ele  = ele + c->flap_ele_comp * f->out.flap;
    tgt.rud  = clampf(rud + c->ail_rud_mix * ail, -1.0f, 1.0f);
    tgt.thr  = clampf(thr * c->thr_max, 0.0f, 1.0f);
    tgt.flap = flap;

    /* ---- 10. Clamp, slew, scale to servo pulses ---- */
    tgt.ail = clampf(tgt.ail, -1.0f, 1.0f);
    tgt.ele = clampf(tgt.ele, -1.0f, 1.0f);

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
extern void     hal_write_servos(const uint16_t us[NUM_CH]);

void fcs_run(void)
{
    static fcs_t fcs;
    rc_frame_t rc; uint16_t out[NUM_CH];
    fcs_init(&fcs);
    for (;;) {
        hal_wait_next_tick();
        hal_read_rc(&rc);
        fcs_step(&fcs, &rc, hal_millis(), 0.01f, out);
        hal_write_servos(out);
    }
}
#endif

/* ------------------------------------------------------------------------- */
#ifdef FCS_SIM
static uint16_t us_from(float s) { return (uint16_t)(1500.0f + 500.0f * s); }

int main(void)
{
    fcs_t f; fcs_init(&f);
    const float dt = 0.01f;
    const char *names[] = { "DISARM", "NORMAL", "HOLD  ", "FAILSF" };
    printf("  t   mode   bank_est pitch_est | ail  ele  thr  rud flap\n");

    for (int k = 0; k < 5000; k++) {
        float t = k * dt;
        uint32_t now = (uint32_t)(k * 10);
        float ail = 0, ele = 0, rud = 0, thr = 0.0f, flap = 0, hold = 0;

        if (t >= 1.0f)               thr = 0.60f;             /* arms during first second at idle */
        if (t >= 3  && t < 9)        ail = 1.0f;              /* sustained full right: bank limiter */
        if (t >= 11 && t < 17)       ele = 1.0f;              /* sustained full back: pitch limiter */
        if (t >= 18 && t < 21)       { ail = 0.3f; ele = 0.2f; }   /* hold a gentle turn... */
        if (t >= 19 && t < 27)         hold = 1;                    /* ...latch it, sticks released at t=21 */
        if (t >= 31 && t < 37)       flap = 1.0f;
        if (t >= 38 && t < 40)       thr = 0.30f;
        if (t >= 41 && t < 45) { hold = 1; if (t >= 43) ail = 0.9f; }               /* breakout test */
        bool link_lost = (t >= 47 && t < 51);

        rc_frame_t rc = { .stamp_ms = link_lost ? (uint32_t)(47 * 1000) : now };
        rc.us[CH_AIL] = us_from(ail); rc.us[CH_ELE] = us_from(ele); rc.us[CH_RUD] = us_from(rud);
        rc.us[CH_THR] = (uint16_t)(1000 + 1000 * thr);
        rc.us[CH_HOLD] = hold > 0 ? 2000 : 1000;
        rc.us[CH_FLAP] = flap > 0 ? 2000 : 1000;

        uint16_t out[NUM_CH];
        fcs_step(&f, &rc, now, dt, out);

        if (k % 100 == 0)
            printf("%5.1f  %s  %7.1f  %8.1f | %4d %4d %4d %4d %4d\n", t, names[f.mode],
                   f.bank_est * 180 / 3.14159265f, f.pitch_est * 180 / 3.14159265f,
                   out[CH_AIL], out[CH_ELE], out[CH_THR], out[CH_RUD], out[CH_FLAP]);
    }
    return 0;
}
#endif
