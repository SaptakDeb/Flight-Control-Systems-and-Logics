/*
 * BWB_PID_SIM.c -- step-response test bench for BWB_PID.c
 *
 *   gcc -std=c99 -O2 -o bwb_pid_sim BWB_PID_SIM.c -lm
 *   ./bwb_pid_sim [--csv out.csv] [--report pid_results.txt]
 *
 * Flies the controller on the BWB plant (bwb_plant.h) and checks, with PASS / FAIL:
 *   1  roll step 20 deg            2  pitch step 6 deg           3  speed step +3 m/s
 *   4  heading +60 deg and altitude +30 m (outer loops, all channels at once)
 *   5  same steps on weak, strong and gusty aircraft, with sensor noise
 *   6  saturation: large roll and pitch demand together (pitch priority, spoiler assist)
 *   7  yaw damper on vs off (Dutch-roll excursion after a roll doublet)
 * Planar x-y motion plus altitude; roll, pitch and Dutch-roll dynamics with actuator lags.
 */
#include <stdio.h>
#include <stdlib.h>
#include "BWB_PID.c"

#define MAXN 4000
static float T_[MAXN], Y_[MAXN];

typedef struct { float rise, over, settle, sse, tail; bool valid; } step_t;

/* standard step metrics on y(t) going from y0 to target */
static step_t metrics(const float *t, const float *y, int n, float y0, float target)
{
    step_t r = {0, 0, 0, 0, 0, true};
    float A = target - y0;
    if (fabsf(A) < 1e-6f) { r.valid = false; return r; }
    float t10 = -1, t90 = -1, mx = -1e9f, last_out = 0;
    for (int i = 0; i < n; i++) {
        float yn = (y[i] - y0) / A;
        if (t10 < 0 && yn >= 0.1f) t10 = t[i];
        if (t90 < 0 && yn >= 0.9f) t90 = t[i];
        if (yn > mx) mx = yn;
        if (fabsf(yn - 1.0f) > 0.02f) last_out = t[i];
    }
    r.rise = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1.0f;
    r.over = mx > 1.0f ? 100.0f * (mx - 1.0f) : 0.0f;
    r.settle = last_out;
    int k0 = n - n / 10; float s = 0; for (int i = k0; i < n; i++) s += (y[i] - y0) / A;
    r.sse = 100.0f * (s / (n - k0) - 1.0f);
    int k1 = n - n / 4; float ta = 0; for (int i = k1; i < n; i++) ta += fabsf((y[i] - y0) / A - 1.0f);
    r.tail = 100.0f * ta / (n - k1);                  /* mean |error| over the last quarter, % of the step */
    return r;
}

static bwb_pp_t plant_clean(void)
{
    bwb_pp_t p; bwb_pp_nominal(&p, 18.0f);
    p.gyro_noise = 0.0f; p.att_noise = 0.0f; p.gust = 0.0f;
    return p;
}

typedef enum { CH_ROLL, CH_PITCH, CH_SPEED, CH_HDG, CH_ALT } chan_t;

typedef struct {
    float bank_cmd, pitch_cmd, v_tgt;       /* used when use_outer = false */
    bool  use_outer;
    float hdg_cmd, alt_cmd;                 /* used when use_outer = true  */
    float T;
    float peak_e, peak_sp, max_nu;
} run_t;

/* run one scenario; log channel `ch` into T_/Y_; return sample count and start value */
static int run(const bwb_pp_t *pp, const bwb_pid_cfg_t *c, run_t *r, chan_t ch, float *y0, uint32_t seed, FILE *csv)
{
    bwb_ps_t st; bwb_ps_init(&st, pp, 0, 0, 0.0f, 100.0f, seed);
    bwb_pid_state_t ps; bwb_pid_reset(&ps);
    bwb_meas_t z; bwb_surf_t o = {0};
    int n = 0, N = (int)(r->T / BWB_DT);
    r->peak_e = r->peak_sp = r->max_nu = 0;
    *y0 = ch == CH_ROLL ? 0 : ch == CH_PITCH ? 0 : ch == CH_SPEED ? pp->V0 : ch == CH_HDG ? 0 : 100.0f;
    for (int k = 0; k < N; k++) {
        bwb_measure(&st, pp, &z);
        float bank, pitch;
        if (r->use_outer) {
            bwb_pid_demand_t d = { r->hdg_cmd, r->alt_cmd, pp->V0 };
            bwb_pid_step(&ps, c, NULL, &z, &d, BWB_DT, &o, &bank, &pitch);
        } else {
            bwb_pid_inner(&ps, c, NULL, &z, r->bank_cmd, r->pitch_cmd, r->v_tgt, BWB_DT, &o);
            bank = r->bank_cmd; pitch = r->pitch_cmd;
        }
        bwb_plant_step(&st, pp, &o, BWB_DT);
        if (fabsf(o.eL) > r->peak_e) r->peak_e = fabsf(o.eL);
        if (fabsf(o.eR) > r->peak_e) r->peak_e = fabsf(o.eR);
        float sp = fabsf(o.sR - o.sL); if (sp > r->peak_sp) r->peak_sp = sp;
        if (fabsf(st.nu) > r->max_nu) r->max_nu = fabsf(st.nu);
        if (!isfinite(st.phi) || fabsf(st.phi) > 2.5f) return -1;           /* diverged */
        float y = ch == CH_ROLL ? st.phi : ch == CH_PITCH ? st.theta : ch == CH_SPEED ? st.V : ch == CH_HDG ? st.psi : st.h;
        if (n < MAXN) { T_[n] = k * BWB_DT; Y_[n] = y; n++; }
        if (csv && k % 5 == 0) fprintf(csv, "%.2f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f\n", k * BWB_DT, st.phi, st.theta, st.V, st.h, st.psi, st.nu, o.eL, o.eR, 0.5f * (o.sL + o.sR), o.rudder);
    }
    return n;
}

static int fails = 0, tests = 0;
static FILE *rep;

static void check(const char *name, const step_t *m, float max_over, float max_settle, float max_sse, bool diverged)
{
    tests++;
    /* max_settle < 0 means 'noisy case': judge by the mean tail error (max_sse, %) instead of a 2 % settling band */
    bool noisy = max_settle < 0.0f;
    bool ok = !diverged && m->valid && m->over <= max_over &&
              (noisy ? m->tail <= max_sse : (m->settle <= max_settle && fabsf(m->sse) <= max_sse));
    if (!ok) fails++;
    char line[256];
    if (diverged) snprintf(line, sizeof line, "%-40s  DIVERGED                                         FAIL", name);
    else if (noisy) snprintf(line, sizeof line, "%-40s  rise %5.2f s  overshoot %5.1f %%  tail error %5.1f %% of step            %s",
                  name, m->rise, m->over, m->tail, ok ? "PASS" : "FAIL");
    else snprintf(line, sizeof line, "%-40s  rise %5.2f s  overshoot %5.1f %%  settle %5.2f s  ss-err %5.2f %%   %s",
                  name, m->rise, m->over, m->settle, m->sse, ok ? "PASS" : "FAIL");
    puts(line); fprintf(rep, "%s\n", line);
}

int main(int argc, char **argv)
{
    const char *csvp = NULL, *repp = "pid_results.txt";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--csv") && i + 1 < argc) csvp = argv[++i];
        else if (!strcmp(argv[i], "--report") && i + 1 < argc) repp = argv[++i];
    }
    rep = fopen(repp, "w");
    if (!rep) return 2;
    bwb_pid_cfg_t c; bwb_pid_cfg_default(&c);
    bwb_pp_t pp = plant_clean();
    float y0; run_t r; step_t m; int n;
    const float D2R = 0.0174533f;

    const char *hdr = "BWB PID TEST BENCH -- step responses on the BWB plant (BWB_PID.c, gains from bwb_pid_cfg_default)";
    puts(hdr); fprintf(rep, "%s\n\n", hdr);

    puts("-- 1-3: inner loops, clean aircraft (no noise, no gusts) --");
    fprintf(rep, "-- 1-3: inner loops, clean aircraft (no noise, no gusts) --\n");
    memset(&r, 0, sizeof r); r.T = 8; r.bank_cmd = 20 * D2R; r.v_tgt = pp.V0;
    n = run(&pp, &c, &r, CH_ROLL, &y0, 1, NULL);  m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.bank_cmd);
    check("roll step 0 -> 20 deg", &m, 10.0f, 1.5f, 1.0f, n < 0);
    memset(&r, 0, sizeof r); r.T = 8; r.pitch_cmd = 6 * D2R; r.v_tgt = pp.V0;
    n = run(&pp, &c, &r, CH_PITCH, &y0, 1, NULL); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.pitch_cmd);
    check("pitch step 0 -> 6 deg", &m, 10.0f, 1.5f, 1.0f, n < 0);
    memset(&r, 0, sizeof r); r.T = 25; r.v_tgt = pp.V0 + 3.0f;
    n = run(&pp, &c, &r, CH_SPEED, &y0, 1, NULL); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.v_tgt);
    check("speed step 18 -> 21 m/s", &m, 15.0f, 10.0f, 2.0f, n < 0);

    puts("-- 4: outer loops, heading +60 deg and altitude +30 m together --");
    fprintf(rep, "-- 4: outer loops, heading +60 deg and altitude +30 m together --\n");
    FILE *csv = csvp ? fopen(csvp, "w") : NULL;
    if (csv) fprintf(csv, "t,roll,pitch,airspeed,altitude,heading,nu,elevon_L,elevon_R,spoiler_sym,rudder\n");
    memset(&r, 0, sizeof r); r.T = 40; r.use_outer = true; r.hdg_cmd = 60 * D2R; r.alt_cmd = 130.0f;
    n = run(&pp, &c, &r, CH_HDG, &y0, 1, csv); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.hdg_cmd);
    check("heading step 0 -> 60 deg", &m, 10.0f, 12.0f, 1.0f, n < 0);
    n = run(&pp, &c, &r, CH_ALT, &y0, 1, NULL); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.alt_cmd);
    check("altitude step 100 -> 130 m", &m, 15.0f, 30.0f, 2.0f, n < 0);
    if (csv) fclose(csv);

    puts("-- 5: robustness: sensor noise, gusts, weak and strong aircraft --");
    fprintf(rep, "-- 5: robustness: sensor noise, gusts, weak and strong aircraft --\n");
    struct { const char *nm; float ka, ke, V, gust; } cs[3] = {
        { "weak authority (ka 1.2, ke 1.0), gusty", 1.2f, 1.0f, 15.0f, 0.5f },
        { "nominal, gusty (0.5)",                   2.5f, 2.0f, 18.0f, 0.5f },
        { "strong authority, 24 m/s, gusty",        4.0f, 3.5f, 24.0f, 0.5f } };
    for (int i = 0; i < 3; i++) {
        bwb_pp_t q; bwb_pp_nominal(&q, cs[i].V); q.ka = cs[i].ka; q.ke = cs[i].ke; q.gust = cs[i].gust;
        char nm[96];
        memset(&r, 0, sizeof r); r.T = 8; r.bank_cmd = 20 * D2R; r.v_tgt = q.V0;
        n = run(&q, &c, &r, CH_ROLL, &y0, 7, NULL); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.bank_cmd);
        snprintf(nm, sizeof nm, "roll 20 deg, %s", cs[i].nm); check(nm, &m, 45.0f, -1.0f, 15.0f, n < 0);
        memset(&r, 0, sizeof r); r.T = 8; r.pitch_cmd = 6 * D2R; r.v_tgt = q.V0;
        n = run(&q, &c, &r, CH_PITCH, &y0, 7, NULL); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.pitch_cmd);
        snprintf(nm, sizeof nm, "pitch 6 deg, %s", cs[i].nm); check(nm, &m, 45.0f, -1.0f, 15.0f, n < 0);
    }

    puts("-- 6: saturation, roll 30 deg and pitch 14 deg demanded together --");
    fprintf(rep, "-- 6: saturation, roll 30 deg and pitch 14 deg demanded together --\n");
    {
        bwb_pp_t q = plant_clean(); q.ka = 1.6f;
        memset(&r, 0, sizeof r); r.T = 6; r.bank_cmd = 30 * D2R; r.pitch_cmd = 14 * D2R; r.v_tgt = q.V0;
        n = run(&q, &c, &r, CH_PITCH, &y0, 3, NULL);
        step_t mp = metrics(T_, Y_, n > 0 ? n : 1, y0, r.pitch_cmd);
        bool ok = n > 0 && r.peak_e <= 1.001f && r.peak_sp > 0.05f && mp.over <= 20.0f;
        tests++; if (!ok) fails++;
        char line[256];
        snprintf(line, sizeof line, "%-40s  peak elevon %.2f (limit 1.00)  spoiler roll-assist peak %.2f  pitch overshoot %.1f %%   %s",
                 "elevon saturation, pitch priority", r.peak_e, r.peak_sp, mp.over, ok ? "PASS" : "FAIL");
        puts(line); fprintf(rep, "%s\n", line);
    }

    puts("-- 7: yaw damper, Dutch-roll excursion after a 30 deg roll step --");
    fprintf(rep, "-- 7: yaw damper, Dutch-roll excursion after a 30 deg roll step --\n");
    {
        bwb_pp_t q = plant_clean(); q.zeta = 0.05f;
        bwb_pid_cfg_t off = c; off.k_yaw_damp = 0.0f; off.k_rud_ff = 0.0f;
        run_t a, b; memset(&a, 0, sizeof a); a.T = 12; a.bank_cmd = 30 * D2R; a.v_tgt = q.V0; b = a;
        run(&q, &off, &a, CH_ROLL, &y0, 5, NULL);
        run(&q, &c, &b, CH_ROLL, &y0, 5, NULL);
        bool ok = b.max_nu < a.max_nu;
        tests++; if (!ok) fails++;
        char line[256];
        snprintf(line, sizeof line, "%-40s  peak sideslip excursion %.4f rad (damper off) -> %.4f rad (damper on)   %s",
                 "yaw damper reduces Dutch roll", a.max_nu, b.max_nu, ok ? "PASS" : "FAIL");
        puts(line); fprintf(rep, "%s\n", line);
    }

    char tail[128]; snprintf(tail, sizeof tail, "\n%d of %d checks passed", tests - fails, tests);
    puts(tail); fprintf(rep, "%s\n", tail);
    fclose(rep);
    return fails ? 1 : 0;
}
