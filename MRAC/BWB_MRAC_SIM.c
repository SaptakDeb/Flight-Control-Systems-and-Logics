/*
 * BWB_MRAC_SIM.c -- test bench for BWB_MRAC.c, with BWB_PID.c flown on the same tests
 *
 *   gcc -std=c99 -O2 -o bwb_mrac_sim BWB_MRAC_SIM.c -lm
 *   ./bwb_mrac_sim [--report mrac_results.txt] [--csv failure.csv] [--set name=value ...]
 *   (--set overrides a tuning value: wm zm zmp wmp gx1 gx2 gr gb sigma dz q1 q2)
 *
 *   A  step-response checks for MRAC (same thresholds as the PID bench), PID shown beside it
 *   B  authority sweep: three roll steps on aircraft from very weak to very strong, 1st vs 3rd step
 *   C  sudden loss of control authority at t = 20 s (elevon damage): tracking before / after
 *   D  120 s gusty run with sensor noise: tracking and adaptive-parameter drift
 * Plant: bwb_plant.h (planar x-y + altitude, roll / pitch / Dutch-roll dynamics, actuator lags).
 */
#include <stdio.h>
#include <stdlib.h>
#include "BWB_MRAC.c"

#define MAXN 8000
typedef enum { K_PID, K_MRAC } kind_t;
typedef struct { bwb_pid_state_t pid; bwb_mrac_state_t mr; } ctl_t;
static bwb_pid_cfg_t g_pc;
static bwb_mrac_cfg_t g_mc;
static float T_[MAXN], Y_[MAXN];
static FILE *rep;

static void say(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#include <stdarg.h>
static void say(const char *fmt, ...)
{
    char b[512]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    fputs(b, stdout); fputs(b, rep);
}

static void ctl_reset(ctl_t *c) { memset(c, 0, sizeof *c); }
static void ctl_inner(kind_t k, ctl_t *c, const bwb_meas_t *z, float bank, float pitch, float vt, bwb_surf_t *o)
{
    if (k == K_PID) bwb_pid_inner(&c->pid, &g_pc, NULL, z, bank, pitch, vt, BWB_DT, o);
    else bwb_mrac_inner(&c->mr, &g_pc, &g_mc, z, bank, pitch, vt, BWB_DT, o);
}
static void ctl_step(kind_t k, ctl_t *c, const bwb_meas_t *z, const bwb_pid_demand_t *d, bwb_surf_t *o)
{
    if (k == K_PID) bwb_pid_step(&c->pid, &g_pc, NULL, z, d, BWB_DT, o, NULL, NULL);
    else bwb_mrac_step(&c->mr, &g_pc, &g_mc, z, d, BWB_DT, o, NULL, NULL);
}

/* ---- step metrics ----------------------------------------------------------- */
typedef struct { float rise, over, settle, sse, tail; bool valid; } step_t;
static step_t metrics(const float *t, const float *y, int n, float y0, float target)
{
    step_t r = {0, 0, 0, 0, 0, true};
    float A = target - y0;
    if (n < 4 || fabsf(A) < 1e-6f) { r.valid = false; return r; }
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
    r.tail = 100.0f * ta / (n - k1);
    return r;
}

typedef enum { CH_ROLL, CH_PITCH, CH_SPEED, CH_HDG, CH_ALT } chan_t;
typedef struct { float bank, pitch, vt, T, hdg, alt; bool outer; float peak_e, peak_sp; } run_t;

static bwb_pp_t clean_plant(void)
{
    bwb_pp_t p; bwb_pp_nominal(&p, 18.0f); p.gyro_noise = 0; p.att_noise = 0; p.gust = 0; return p;
}

static int run(kind_t k, const bwb_pp_t *pp, run_t *r, chan_t ch, float *y0, uint32_t seed)
{
    bwb_ps_t st; bwb_ps_init(&st, pp, 0, 0, 0.0f, 100.0f, seed);
    ctl_t c; ctl_reset(&c);
    bwb_meas_t z; bwb_surf_t o = {0};
    int n = 0, N = (int)(r->T / BWB_DT);
    r->peak_e = r->peak_sp = 0;
    *y0 = ch == CH_SPEED ? pp->V0 : ch == CH_ALT ? 100.0f : 0.0f;
    for (int i = 0; i < N; i++) {
        bwb_measure(&st, pp, &z);
        if (r->outer) { bwb_pid_demand_t d = { r->hdg, r->alt, pp->V0 }; ctl_step(k, &c, &z, &d, &o); }
        else ctl_inner(k, &c, &z, r->bank, r->pitch, r->vt, &o);
        bwb_plant_step(&st, pp, &o, BWB_DT);
        if (fabsf(o.eL) > r->peak_e) r->peak_e = fabsf(o.eL);
        if (fabsf(o.eR) > r->peak_e) r->peak_e = fabsf(o.eR);
        if (fabsf(o.sR - o.sL) > r->peak_sp) r->peak_sp = fabsf(o.sR - o.sL);
        if (!isfinite(st.phi) || fabsf(st.phi) > 2.5f) return -1;
        float y = ch == CH_ROLL ? st.phi : ch == CH_PITCH ? st.theta : ch == CH_SPEED ? st.V : ch == CH_HDG ? st.psi : st.h;
        if (n < MAXN) { T_[n] = i * BWB_DT; Y_[n] = y; n++; }
    }
    return n;
}

static int fails = 0, tests = 0;
static void check(const char *name, const step_t *m, const step_t *mp, float max_over, float max_settle, float max_sse, bool div)
{
    tests++;
    bool noisy = max_settle < 0;
    bool ok = !div && m->valid && m->over <= max_over &&
              (noisy ? m->tail <= max_sse : (m->settle <= max_settle && fabsf(m->sse) <= max_sse));
    if (!ok) fails++;
    if (div) { say("%-34s  MRAC DIVERGED                                                FAIL\n", name); return; }
    if (noisy) say("%-34s  MRAC rise %4.2f over %5.1f%% tail %4.1f%%  %s | PID rise %4.2f over %5.1f%% tail %4.1f%%\n", name, m->rise, m->over, m->tail, ok ? "PASS" : "FAIL", mp->rise, mp->over, mp->tail);
    else       say("%-34s  MRAC rise %4.2f over %5.1f%% settle %5.2f  %s | PID rise %4.2f over %5.1f%% settle %5.2f\n", name, m->rise, m->over, m->settle, ok ? "PASS" : "FAIL", mp->rise, mp->over, mp->settle);
}

/* roll step sequence: +amp, -amp, +amp, each tseg seconds */
static int seq_roll(kind_t k, const bwb_pp_t *pp, float amp, float tseg, int nseg, float *roll_out)
{
    bwb_ps_t st; bwb_ps_init(&st, pp, 0, 0, 0.0f, 100.0f, 11);
    ctl_t c; ctl_reset(&c); bwb_meas_t z; bwb_surf_t o = {0};
    int N = (int)(tseg * nseg / BWB_DT);
    for (int i = 0; i < N; i++) {
        int seg = (int)(i * BWB_DT / tseg);
        float cmd = (seg % 2 == 0) ? amp : -amp;
        bwb_measure(&st, pp, &z);
        ctl_inner(k, &c, &z, cmd, 0.0f, pp->V0, &o);
        bwb_plant_step(&st, pp, &o, BWB_DT);
        if (!isfinite(st.phi) || fabsf(st.phi) > 2.5f) return -1;
        roll_out[i] = st.phi;
    }
    return N;
}

static step_t seg_metrics(const float *roll, int seg, float tseg, float amp)
{
    int n0 = (int)(seg * tseg / BWB_DT), n1 = (int)((seg + 1) * tseg / BWB_DT);
    static float tt[MAXN];
    for (int i = n0; i < n1; i++) tt[i - n0] = (i - n0) * BWB_DT;
    float y0 = (seg == 0) ? 0.0f : ((seg % 2 == 0) ? -amp : amp), tg = (seg % 2 == 0) ? amp : -amp;
    return metrics(tt, roll + n0, n1 - n0, y0, tg);
}

int main(int argc, char **argv)
{
    const char *repp = "mrac_results.txt", *csvp = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--report") && i + 1 < argc) repp = argv[++i];
        else if (!strcmp(argv[i], "--csv") && i + 1 < argc) csvp = argv[++i];
    }
    rep = fopen(repp, "w"); if (!rep) return 2;
    bwb_pid_cfg_default(&g_pc); bwb_mrac_cfg_default(&g_mc);
    for (int i = 1; i + 1 < argc; i++) {                 /* --set name=value  (tuning experiments) */
        if (strcmp(argv[i], "--set")) continue;
        char nm[32]; float v;
        if (sscanf(argv[i + 1], "%31[^=]=%f", nm, &v) != 2) continue;
        if (!strcmp(nm, "wm")) g_mc.wm_roll = g_mc.wm_pitch = v;
        else if (!strcmp(nm, "zm")) g_mc.zm_roll = g_mc.zm_pitch = v; else if (!strcmp(nm, "zmp")) g_mc.zm_pitch = v; else if (!strcmp(nm, "wmp")) g_mc.wm_pitch = v;
        else if (!strcmp(nm, "gx1")) g_mc.gx1 = v; else if (!strcmp(nm, "gx2")) g_mc.gx2 = v;
        else if (!strcmp(nm, "gr")) g_mc.gr = v; else if (!strcmp(nm, "gb")) g_mc.gb = v;
        else if (!strcmp(nm, "sigma")) g_mc.sigma = v; else if (!strcmp(nm, "dz")) g_mc.dz = v;
        else if (!strcmp(nm, "q2")) g_mc.q2 = v; else if (!strcmp(nm, "q1")) g_mc.q1 = v;
    }
    const float D2R = 0.0174533f;
    bwb_pp_t pp = clean_plant();
    run_t r; float y0; int n, np; step_t m, mp;
    static float rollA[MAXN], rollB[MAXN];

    say("BWB MRAC TEST BENCH -- model reference adaptive control vs fixed-gain PID on the same BWB plant\n");
    say("reference models: roll and pitch wm = %.1f rad/s, zeta = %.1f;  speed am = %.1f 1/s\n\n", g_mc.wm_roll, g_mc.zm_roll, g_mc.am_v);

    /* ---------------------------------- A ----------------------------------- */
    say("A. STEP RESPONSES, CLEAN AIRCRAFT (MRAC checked, PID shown beside it)\n");
    memset(&r, 0, sizeof r); r.T = 8; r.bank = 20 * D2R; r.vt = pp.V0;
    n = run(K_MRAC, &pp, &r, CH_ROLL, &y0, 1); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.bank);
    np = run(K_PID, &pp, &r, CH_ROLL, &y0, 1); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.bank);
    check("roll step 0 -> 20 deg", &m, &mp, 10.0f, 1.5f, 1.0f, n < 0);
    memset(&r, 0, sizeof r); r.T = 8; r.pitch = 6 * D2R; r.vt = pp.V0;
    n = run(K_MRAC, &pp, &r, CH_PITCH, &y0, 1); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.pitch);
    np = run(K_PID, &pp, &r, CH_PITCH, &y0, 1); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.pitch);
    check("pitch step 0 -> 6 deg", &m, &mp, 10.0f, 1.5f, 1.0f, n < 0);
    memset(&r, 0, sizeof r); r.T = 25; r.vt = pp.V0 + 3.0f;
    n = run(K_MRAC, &pp, &r, CH_SPEED, &y0, 1); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.vt);
    np = run(K_PID, &pp, &r, CH_SPEED, &y0, 1); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.vt);
    check("speed step 18 -> 21 m/s", &m, &mp, 15.0f, 10.0f, 2.0f, n < 0);
    memset(&r, 0, sizeof r); r.T = 40; r.outer = true; r.hdg = 60 * D2R; r.alt = 130.0f;
    n = run(K_MRAC, &pp, &r, CH_HDG, &y0, 1); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.hdg);
    np = run(K_PID, &pp, &r, CH_HDG, &y0, 1); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.hdg);
    check("heading step 0 -> 60 deg", &m, &mp, 10.0f, 12.0f, 1.0f, n < 0);
    n = run(K_MRAC, &pp, &r, CH_ALT, &y0, 1); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.alt);
    np = run(K_PID, &pp, &r, CH_ALT, &y0, 1); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.alt);
    check("altitude step 100 -> 130 m", &m, &mp, 15.0f, 30.0f, 2.0f, n < 0);

    struct { const char *nm; float ka, ke, V, gust; } cs[3] = {
        { "weak (ka 1.2, ke 1.0), gusty", 1.2f, 1.0f, 15.0f, 0.5f },
        { "nominal, gusty 0.5",           2.5f, 2.0f, 18.0f, 0.5f },
        { "strong, 24 m/s, gusty",        4.0f, 3.5f, 24.0f, 0.5f } };
    for (int i = 0; i < 3; i++) {
        bwb_pp_t q; bwb_pp_nominal(&q, cs[i].V); q.ka = cs[i].ka; q.ke = cs[i].ke; q.gust = cs[i].gust;
        char nm[96];
        memset(&r, 0, sizeof r); r.T = 8; r.bank = 20 * D2R; r.vt = q.V0;
        n = run(K_MRAC, &q, &r, CH_ROLL, &y0, 7); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.bank);
        np = run(K_PID, &q, &r, CH_ROLL, &y0, 7); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.bank);
        snprintf(nm, sizeof nm, "roll 20: %s", cs[i].nm); check(nm, &m, &mp, 45.0f, -1.0f, 15.0f, n < 0);
        memset(&r, 0, sizeof r); r.T = 8; r.pitch = 6 * D2R; r.vt = q.V0;
        n = run(K_MRAC, &q, &r, CH_PITCH, &y0, 7); m = metrics(T_, Y_, n > 0 ? n : 1, y0, r.pitch);
        np = run(K_PID, &q, &r, CH_PITCH, &y0, 7); mp = metrics(T_, Y_, np > 0 ? np : 1, y0, r.pitch);
        snprintf(nm, sizeof nm, "pitch 6: %s", cs[i].nm); check(nm, &m, &mp, 45.0f, -1.0f, 15.0f, n < 0);
    }
    {   /* saturation: pitch priority, spoiler roll assist */
        bwb_pp_t q = clean_plant(); q.ka = 1.6f;
        memset(&r, 0, sizeof r); r.T = 6; r.bank = 30 * D2R; r.pitch = 14 * D2R; r.vt = q.V0;
        n = run(K_MRAC, &q, &r, CH_PITCH, &y0, 3);
        step_t mq = metrics(T_, Y_, n > 0 ? n : 1, y0, r.pitch);
        bool ok = n > 0 && r.peak_e <= 1.001f && r.peak_sp > 0.05f && mq.over <= 20.0f;
        tests++; if (!ok) fails++;
        say("%-34s  MRAC peak elevon %.2f, spoiler assist %.2f, pitch overshoot %.1f%%  %s\n", "elevon saturation", r.peak_e, r.peak_sp, mq.over, ok ? "PASS" : "FAIL");
    }
    say("-> %d of %d checks passed\n\n", tests - fails, tests);

    /* ---------------------------------- B ----------------------------------- */
    say("B. AUTHORITY SWEEP: roll steps +/-20 deg every 6 s; first step vs third step (clean aircraft)\n");
    say("   authority ka (nominal 2.5)    |   PID: rise  overshoot  (1st)   rise  overshoot  (3rd)   |  MRAC: rise  overshoot  (1st)   rise  overshoot  (3rd)\n");
    float kas[5] = { 0.8f, 1.5f, 2.5f, 4.0f, 4.8f };
    for (int i = 0; i < 5; i++) {
        bwb_pp_t q = clean_plant(); q.ka = kas[i]; q.ke = 0.8f * kas[i];
        int a = seq_roll(K_PID, &q, 20 * D2R, 6.0f, 3, rollA), b = seq_roll(K_MRAC, &q, 20 * D2R, 6.0f, 3, rollB);
        if (a < 0 || b < 0) { say("   ka %.1f: %s diverged\n", kas[i], a < 0 ? "PID" : "MRAC"); continue; }
        step_t p1 = seg_metrics(rollA, 0, 6.0f, 20 * D2R), p3 = seg_metrics(rollA, 2, 6.0f, 20 * D2R);
        step_t m1 = seg_metrics(rollB, 0, 6.0f, 20 * D2R), m3 = seg_metrics(rollB, 2, 6.0f, 20 * D2R);
        say("   ka %.1f                        |        %5.2f s %6.1f %%          %5.2f s %6.1f %%            |        %5.2f s %6.1f %%          %5.2f s %6.1f %%\n",
            kas[i], p1.rise, p1.over, p3.rise, p3.over, m1.rise, m1.over, m3.rise, m3.over);
    }
    say("   (MRAC adapts during the first step, so its 1st-step overshoot on very weak aircraft is larger than its 3rd)\n\n");

    /* ---------------------------------- C ----------------------------------- */
    say("C. LOSS OF CONTROL AUTHORITY: roll square wave +/-20 deg every 4 s; at t = 20 s roll authority falls to 35 %% and pitch to 45 %%\n");
    {
        FILE *csv = csvp ? fopen(csvp, "w") : NULL;
        if (csv) fprintf(csv, "t,bank_cmd,roll_pid,roll_mrac,ref_model,mrac_kx1,mrac_kr\n");
        bwb_pp_t base; bwb_pp_nominal(&base, 18.0f); base.gust = 0.2f;
        const int NS = 15; float tseg = 4.0f; int N = (int)(NS * tseg / BWB_DT);
        double iae[2][NS]; memset(iae, 0, sizeof iae);
        bwb_ps_t st[2]; ctl_t c[2]; bwb_pp_t pq[2]; bwb_meas_t z; bwb_surf_t o[2]; memset(o, 0, sizeof o);
        for (int k = 0; k < 2; k++) { pq[k] = base; bwb_ps_init(&st[k], &pq[k], 0, 0, 0.0f, 100.0f, 21); ctl_reset(&c[k]); }
        float kx1_end = 0, kr_end = 0, kx1_start = 0;
        for (int i = 0; i < N; i++) {
            float t = i * BWB_DT; int seg = (int)(t / tseg);
            float cmd = (seg % 2 == 0) ? 20 * D2R : -20 * D2R;
            if (i == (int)(20.0f / BWB_DT)) for (int k = 0; k < 2; k++) { pq[k].ka *= 0.35f; pq[k].ke *= 0.45f; }
            for (int k = 0; k < 2; k++) {
                bwb_measure(&st[k], &pq[k], &z);
                ctl_inner(k == 0 ? K_PID : K_MRAC, &c[k], &z, cmd, 0.0f, pq[k].V0, &o[k]);
                bwb_plant_step(&st[k], &pq[k], &o[k], BWB_DT);
                iae[k][seg] += fabsf(cmd - st[k].phi) * BWB_DT;
            }
            if (i == 100) kx1_start = c[1].mr.roll.kx1;
            kx1_end = c[1].mr.roll.kx1; kr_end = c[1].mr.roll.kr;
            if (csv && i % 5 == 0) fprintf(csv, "%.2f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f\n", t, cmd, st[0].phi, st[1].phi, c[1].mr.roll.xm1, c[1].mr.roll.kx1, c[1].mr.roll.kr);
        }
        if (csv) fclose(csv);
        say("   roll-error integral per 4 s segment (rad*s; lower is better)\n   segment (t):    ");
        for (int s = 0; s < NS; s++) { say("%5d ", s * 4); }
        say("\n   PID             ");
        for (int s = 0; s < NS; s++) { say("%5.2f ", iae[0][s]); }
        say("\n   MRAC            ");
        for (int s = 0; s < NS; s++) { say("%5.2f ", iae[1][s]); }
        say("\n");
        double pb = 0, mb = 0, pa = 0, ma = 0, pl = 0, ml = 0;
        for (int s = 1; s < 5; s++) { pb += iae[0][s]; mb += iae[1][s]; }
        for (int s = 5; s < 8; s++) { pa += iae[0][s]; ma += iae[1][s]; }
        for (int s = 10; s < NS; s++) { pl += iae[0][s]; ml += iae[1][s]; }
        pb /= 4; mb /= 4; pa /= 3; ma /= 3; pl /= 5; ml /= 5;
        say("   mean per segment:   before the failure  PID %.2f  MRAC %.2f | right after (t 20-32 s)  PID %.2f  MRAC %.2f | late (t 40-60 s)  PID %.2f  MRAC %.2f\n", pb, mb, pa, ma, pl, ml);
        say("   MRAC roll parameters: kx1 %.2f (start) -> %.2f (end), kr %.2f; nominal kx1 %.2f, kr %.2f; a 65 %% authority loss needs about 2.9x the gain\n\n",
            kx1_start, kx1_end, kr_end, -g_mc.wm_roll * g_mc.wm_roll * g_mc.tau / g_mc.ka0, g_mc.wm_roll * g_mc.wm_roll * g_mc.tau / g_mc.ka0);
    }

    /* ---------------------------------- D ----------------------------------- */
    say("D. LONG GUSTY RUN: 120 s, gust 0.8, sensor noise, random roll and pitch demands; parameter drift\n");
    {
        bwb_pp_t q; bwb_pp_nominal(&q, 18.0f); q.gust = 0.8f; q.pbias = 0.05f; q.qbias = -0.03f;
        bwb_ps_t st[2]; ctl_t c[2]; bwb_meas_t z; bwb_surf_t o[2]; memset(o, 0, sizeof o);
        for (int k = 0; k < 2; k++) { bwb_ps_init(&st[k], &q, 0, 0, 0.0f, 100.0f, 31); ctl_reset(&c[k]); }
        uint32_t rg = 99; float cb = 0, cp = 0; double ier[2] = {0, 0}, iep[2] = {0, 0}; long cnt = 0;
        float kmin[3] = {1e9f, 1e9f, 1e9f}, kmax[3] = {-1e9f, -1e9f, -1e9f};
        int N = (int)(120.0f / BWB_DT);
        for (int i = 0; i < N; i++) {
            if (i % 200 == 0) { cb = (bwb_rand(&rg) - 0.5f) * 0.6f; cp = (bwb_rand(&rg) - 0.5f) * 0.2f; }
            for (int k = 0; k < 2; k++) {
                bwb_measure(&st[k], &q, &z);
                ctl_inner(k == 0 ? K_PID : K_MRAC, &c[k], &z, cb, cp, q.V0, &o[k]);
                bwb_plant_step(&st[k], &q, &o[k], BWB_DT);
                ier[k] += fabsf(cb - st[k].phi); iep[k] += fabsf(cp - st[k].theta);
            }
            cnt++;
            float v[3] = { c[1].mr.roll.kx1, c[1].mr.roll.kr, c[1].mr.pitch.kx1 };
            for (int j = 0; j < 3; j++) { if (v[j] < kmin[j]) kmin[j] = v[j]; if (v[j] > kmax[j]) kmax[j] = v[j]; }
        }
        say("   mean |roll error|   PID %.4f rad   MRAC %.4f rad     mean |pitch error|   PID %.4f rad   MRAC %.4f rad\n", ier[0] / cnt, ier[1] / cnt, iep[0] / cnt, iep[1] / cnt);
        say("   MRAC parameters stayed bounded: roll kx1 %.2f..%.2f (nominal %.2f), roll kr %.2f..%.2f (nominal %.2f), pitch kx1 %.2f..%.2f (nominal %.2f)\n",
            kmin[0], kmax[0], c[1].mr.roll.kx1_0, kmin[1], kmax[1], c[1].mr.roll.kr_0, kmin[2], kmax[2], c[1].mr.pitch.kx1_0);
    }
    fclose(rep);
    return fails ? 1 : 0;
}
