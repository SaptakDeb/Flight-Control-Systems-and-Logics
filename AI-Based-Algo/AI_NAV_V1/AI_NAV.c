/*
 * AI_NAV.c  --  Algorithmic route planner + flight controller (tube-and-wing)
 *
 * "AI" here means classical graph search, not a learned model:
 *   1. The map (Zone1V1.map) is a network of nodes.  Links are built from
 *      line-of-sight between nodes, and every link gets a weight:
 *        length x corridor multiplier + hazard penalty + climb penalty
 *      No-fly zones (inflated by a safety margin) remove links entirely.
 *   2. Heading-aware A* searches over (previous node, current node) states, so
 *      sharp turns carry a cost and reversals are forbidden.
 *   3. A cost-aware string-pulling pass removes unnecessary waypoints.
 *   4. The resulting route is flown by the same L1 guidance / PD inner loops /
 *      speed-PI logic as nav.c, with turn anticipation at waypoints.
 *   5. If a pop-up no-fly zone blocks the remaining route, the planner
 *      re-plans from the aircraft's current position and heading.
 *
 * Coordinates are map-sheet metres: x = east, y = north.
 * Conventions: roll + = right wing down, pitch + = nose up, aileron + = roll
 * right, elevator + = nose up, heading 0 = north, clockwise positive.
 *
 * Outputs are normalised surface demands in the same channel space as the RC
 * map (ch1 aileron, ch2 elevator, ch3 throttle, ch4 rudder, ch6 flaps), so they
 * can be fed to the FCS as pilot input and still pass through its protection.
 *
 * Build: compile this file with your FCS, or #include it from the simulator
 * (AI_NAV_SIM.c does this).  It has no main().  C99, needs -lm.
 */
#ifndef AI_NAV_C
#define AI_NAV_C

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AI_MAX_NODES   48
#define AI_MAX_NOFLY   16
#define AI_MAX_HAZARD  16
#define AI_MAX_CORR    48
#define AI_MAX_ROUTE   32
#define AI_NAME_LEN    24
#define AI_PI          3.14159265358979323846
#define AI_G           9.80665f
#define AI_INF         1.0e30f

/* ======================================================================== */
/*  Data types                                                              */
/* ======================================================================== */

typedef struct { char name[AI_NAME_LEN]; double x, y; float alt; bool ok; } ai_node_t;

/* circle: a=cx b=cy c=r        box: a=x0 b=y0 c=x1 d=y1 */
typedef struct {
    char   name[AI_NAME_LEN];
    bool   is_box;
    double a, b, c, d;
    float  mult;                       /* hazard cost multiplier */
} ai_zone_t;

typedef struct { int a, b; float mult; } ai_corr_t;

typedef struct {
    char   name[AI_NAME_LEN];
    double width, height, margin, link;
    int    n_nodes, n_nofly, n_haz, n_corr;
    ai_node_t node[AI_MAX_NODES];
    ai_zone_t nofly[AI_MAX_NOFLY];
    ai_zone_t haz[AI_MAX_HAZARD];
    ai_corr_t corr[AI_MAX_CORR];
    char   def_start[AI_NAME_LEN], def_end[AI_NAME_LEN];
} ai_map_t;

typedef struct { double x, y; float alt; } ai_pt_t;

typedef struct {
    bool   gps_valid;
    double x_m, y_m;                   /* position on the sheet           */
    float  alt_m;
    float  vn_mps, ve_mps;             /* ground velocity                 */
    float  airspeed_mps;
    float  roll_rad, pitch_rad;
    float  p_rps, q_rps;
    bool   auto_enable;
} ai_input_t;

typedef struct { float aileron, elevator, rudder, throttle, flaps; } ai_output_t;

typedef struct {
    /* planner weights */
    float turn_w;                      /* cost (m) per radian of course change   */
    float max_turn;                    /* rad, sharpest allowed turn at a node   */
    float climb_w;                     /* cost (m) per metre of altitude change  */
    float sample_step;                 /* m, spacing of line-of-sight samples    */
    /* flight envelope */
    float cruise_airspeed, min_airspeed, max_airspeed, cruise_throttle;
    float max_bank, max_pitch_up, max_pitch_down;
    /* guidance */
    float accept_radius, loiter_radius, l1_period, l1_damping;
    float k_hdg, k_alt, k_bank_pitch;
    /* inner loops */
    float kp_roll, kd_roll, kp_pitch, kd_pitch, k_rudder;
    float kp_speed, ki_speed, k_climb_thr;
    float gps_timeout_s;
} ai_cfg_t;

typedef enum { AI_IDLE, AI_TRACK, AI_LOITER, AI_FAILSAFE } ai_state_t;
typedef enum { AI_PLAN_OK = 0, AI_PLAN_NO_ROUTE = 1, AI_PLAN_BAD_INPUT = 2, AI_PLAN_KEPT = 3 } ai_plan_status_t;

typedef struct {
    ai_cfg_t   cfg;
    ai_map_t   map;                    /* private copy: pop-ups are added here */
    ai_state_t state;

    ai_pt_t    route[AI_MAX_ROUTE];
    int        n_pts, leg;
    bool       planned;
    ai_pt_t    goal;

    double     loiter_x, loiter_y;
    float      loiter_alt;

    float      spd_int, gps_age, fs_alt;

    /* planner telemetry */
    float      route_len, route_cost;
    int        expanded, links_built, replans;
    bool       plan_failed;
} ai_nav_t;

/* ======================================================================== */
/*  Small helpers                                                           */
/* ======================================================================== */

static void ai_copy_name(char *dst, const char *src)
{
    size_t n = strlen(src); if (n > AI_NAME_LEN - 1) n = AI_NAME_LEN - 1;
    memcpy(dst, src, n); dst[n] = 0;
}

static float ai_clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static float ai_wrap_pi(float a)
{
    while (a >  (float)AI_PI) a -= 2.0f * (float)AI_PI;
    while (a < -(float)AI_PI) a += 2.0f * (float)AI_PI;
    return a;
}

/* signed distance to a zone boundary: negative inside */
static double ai_zone_sd(const ai_zone_t *z, double x, double y)
{
    if (!z->is_box) return hypot(x - z->a, y - z->b) - z->c;
    double cx = 0.5 * (z->a + z->c), cy = 0.5 * (z->b + z->d);
    double hx = 0.5 * (z->c - z->a), hy = 0.5 * (z->d - z->b);
    double qx = fabs(x - cx) - hx, qy = fabs(y - cy) - hy;
    return hypot(fmax(qx, 0.0), fmax(qy, 0.0)) + fmin(fmax(qx, qy), 0.0);
}

/* ======================================================================== */
/*  Map loading                                                             */
/* ======================================================================== */

int ai_map_find(const ai_map_t *m, const char *name)
{
    for (int i = 0; i < m->n_nodes; i++)
        if (strcmp(m->node[i].name, name) == 0) return i;
    return -1;
}

/* returns 0 on success, -1 file error, -2 syntax/limit error (message on stderr) */
int ai_map_load(ai_map_t *m, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    memset(m, 0, sizeof *m);
    m->margin = 120.0;
    m->link   = 1500.0;

    char line[256], kw[32], n1[AI_NAME_LEN], n2[AI_NAME_LEN];
    int  lineno = 0, rc = 0;
    double a, b, c, d;
    float  mult;

    while (fgets(line, sizeof line, f)) {
        lineno++;
        char *h = strchr(line, '#'); if (h) *h = 0;
        if (sscanf(line, "%31s", kw) != 1) continue;

        if (!strcmp(kw, "MAP")) {
            if (sscanf(line, "%*s %23s %lf %lf", m->name, &m->width, &m->height) != 3) rc = -2;
        } else if (!strcmp(kw, "MARGIN")) {
            if (sscanf(line, "%*s %lf", &m->margin) != 1) rc = -2;
        } else if (!strcmp(kw, "LINK")) {
            if (sscanf(line, "%*s %lf", &m->link) != 1) rc = -2;
        } else if (!strcmp(kw, "NODE")) {
            float alt;
            if (m->n_nodes >= AI_MAX_NODES || sscanf(line, "%*s %23s %lf %lf %f", n1, &a, &b, &alt) != 4) { rc = -2; }
            else {
                ai_node_t *n = &m->node[m->n_nodes++];
                ai_copy_name(n->name, n1);
                n->x = a; n->y = b; n->alt = alt; n->ok = true;
            }
        } else if (!strcmp(kw, "NOFLY_CIRCLE") || !strcmp(kw, "NOFLY_BOX")) {
            bool box = !strcmp(kw, "NOFLY_BOX");
            int  need = box ? 5 : 4;
            int  got = box ? sscanf(line, "%*s %23s %lf %lf %lf %lf", n1, &a, &b, &c, &d)
                           : sscanf(line, "%*s %23s %lf %lf %lf", n1, &a, &b, &c);
            if (m->n_nofly >= AI_MAX_NOFLY || got != need) rc = -2;
            else {
                ai_zone_t *z = &m->nofly[m->n_nofly++];
                memset(z, 0, sizeof *z);
                ai_copy_name(z->name, n1);
                z->is_box = box; z->a = a; z->b = b; z->c = c; z->d = box ? d : 0.0;
            }
        } else if (!strcmp(kw, "HAZARD_CIRCLE") || !strcmp(kw, "HAZARD_BOX")) {
            bool box = !strcmp(kw, "HAZARD_BOX");
            int  need = box ? 6 : 5;
            int  got = box ? sscanf(line, "%*s %23s %lf %lf %lf %lf %f", n1, &a, &b, &c, &d, &mult)
                           : sscanf(line, "%*s %23s %lf %lf %lf %f", n1, &a, &b, &c, &mult);
            if (m->n_haz >= AI_MAX_HAZARD || got != need) rc = -2;
            else {
                ai_zone_t *z = &m->haz[m->n_haz++];
                memset(z, 0, sizeof *z);
                ai_copy_name(z->name, n1);
                z->is_box = box; z->a = a; z->b = b; z->c = c; z->d = box ? d : 0.0; z->mult = mult;
            }
        } else if (!strcmp(kw, "CORRIDOR")) {
            if (m->n_corr >= AI_MAX_CORR || sscanf(line, "%*s %23s %23s %f", n1, n2, &mult) != 3) rc = -2;
            else {
                int i = ai_map_find(m, n1), j = ai_map_find(m, n2);
                if (i < 0 || j < 0) rc = -2;
                else { m->corr[m->n_corr].a = i; m->corr[m->n_corr].b = j; m->corr[m->n_corr].mult = mult; m->n_corr++; }
            }
        } else if (!strcmp(kw, "START")) {
            if (sscanf(line, "%*s %23s", m->def_start) != 1) rc = -2;
        } else if (!strcmp(kw, "END")) {
            if (sscanf(line, "%*s %23s", m->def_end) != 1) rc = -2;
        } else {
            rc = -2;
        }
        if (rc) { fprintf(stderr, "%s:%d: bad line\n", path, lineno); break; }
    }
    fclose(f);
    if (rc) return rc;

    /* a node that sits inside a no-fly zone (plus margin) can never be used */
    for (int i = 0; i < m->n_nodes; i++)
        for (int z = 0; z < m->n_nofly; z++)
            if (ai_zone_sd(&m->nofly[z], m->node[i].x, m->node[i].y) < m->margin) {
                m->node[i].ok = false;
                fprintf(stderr, "warning: node %s is inside the margin of %s and is disabled\n",
                        m->node[i].name, m->nofly[z].name);
            }
    return 0;
}

/* "NAME" or "x,y" or "x,y,alt"  ->  coordinates.  Returns 0 on success. */
int ai_map_resolve(const ai_map_t *m, const char *spec, double *x, double *y, float *alt)
{
    int i = ai_map_find(m, spec);
    if (i >= 0) { *x = m->node[i].x; *y = m->node[i].y; *alt = m->node[i].alt; return 0; }
    double a, b, c; int k = sscanf(spec, "%lf,%lf,%lf", &a, &b, &c);
    if (k < 2) return -1;
    *x = a; *y = b;
    if (k == 3) *alt = (float)c;
    else {                                    /* take the altitude of the nearest node */
        double best = 1e30; *alt = 100.0f;
        for (int n = 0; n < m->n_nodes; n++) {
            double dd = hypot(m->node[n].x - a, m->node[n].y - b);
            if (dd < best) { best = dd; *alt = m->node[n].alt; }
        }
    }
    return 0;
}

/* ======================================================================== */
/*  Config                                                                  */
/* ======================================================================== */

void ai_cfg_default(ai_cfg_t *c)
{
    c->turn_w = 120.0f;  c->max_turn = 140.0f * (float)(AI_PI / 180.0);
    c->climb_w = 2.0f;   c->sample_step = 10.0f;

    c->cruise_airspeed = 18.0f; c->min_airspeed = 11.0f; c->max_airspeed = 30.0f;
    c->cruise_throttle = 0.55f;
    c->max_bank = 30.0f * (float)(AI_PI / 180.0);
    c->max_pitch_up = 15.0f * (float)(AI_PI / 180.0);
    c->max_pitch_down = 10.0f * (float)(AI_PI / 180.0);

    c->accept_radius = 30.0f; c->loiter_radius = 80.0f;
    c->l1_period = 17.0f; c->l1_damping = 0.75f;
    c->k_hdg = 1.2f; c->k_alt = 0.02f; c->k_bank_pitch = 0.10f;

    c->kp_roll = 2.0f; c->kd_roll = 0.15f; c->kp_pitch = 2.5f; c->kd_pitch = 0.20f;
    c->k_rudder = 0.30f;
    c->kp_speed = 0.05f; c->ki_speed = 0.01f; c->k_climb_thr = 1.0f;
    c->gps_timeout_s = 2.0f;
}

int ai_nav_init(ai_nav_t *s, const ai_cfg_t *cfg, const ai_map_t *map)
{
    memset(s, 0, sizeof *s);
    s->cfg = *cfg;
    s->map = *map;
    s->state = AI_IDLE;
    return 0;
}

/* ======================================================================== */
/*  Network weights                                                         */
/* ======================================================================== */

/* Walk a straight segment.  Returns true if it is blocked by a no-fly zone
 * (inflated by the margin, or by zone_margin[] when given) or leaves the sheet.
 * On success *haz_extra is the extra cost added by hazard zones.            */
static bool ai_seg_eval(const ai_map_t *m, double x0, double y0, double x1, double y1,
                        double step, const double *zone_margin, double *haz_extra)
{
    double dx = x1 - x0, dy = y1 - y0, len = hypot(dx, dy);
    int    n  = (int)ceil(len / step); if (n < 1) n = 1;
    double ds = len / n, extra = 0.0;

    for (int k = 0; k <= n; k++) {
        double t = (double)k / n, x = x0 + dx * t, y = y0 + dy * t;
        if (k > 0 && (x < 0.0 || y < 0.0 || x > m->width || y > m->height)) return true;
        for (int z = 0; z < m->n_nofly; z++) {
            double mg = zone_margin ? zone_margin[z] : m->margin;
            if (ai_zone_sd(&m->nofly[z], x, y) < mg) return true;
        }
        if (k > 0)
            for (int h = 0; h < m->n_haz; h++)
                if (ai_zone_sd(&m->haz[h], x, y) < 0.0)
                    extra += (m->haz[h].mult - 1.0) * ds;
    }
    *haz_extra = extra;
    return false;
}

/* ======================================================================== */
/*  Heading-aware A* planner                                                */
/* ======================================================================== */

#define AI_MP (AI_MAX_NODES + 2)

static float         g_C[AI_MP][AI_MP];                     /* link weights   */
static float         g_g[(AI_MP + 1) * AI_MP];              /* cost so far    */
static int           g_parent[(AI_MP + 1) * AI_MP];
static unsigned char g_open[(AI_MP + 1) * AI_MP], g_closed[(AI_MP + 1) * AI_MP];

static ai_plan_status_t ai_plan_route(ai_nav_t *s,
        double sx, double sy, float salt, double shdg, bool hdg_valid,
        double gx, double gy, float galt)
{
    const ai_map_t *m = &s->map;
    const ai_cfg_t *c = &s->cfg;
    const int N = m->n_nodes, S = N, G = N + 1, M = N + 2;

    ai_pt_t P[AI_MP];
    for (int i = 0; i < N; i++) { P[i].x = m->node[i].x; P[i].y = m->node[i].y; P[i].alt = m->node[i].alt; }
    P[S] = (ai_pt_t){ sx, sy, salt };
    P[G] = (ai_pt_t){ gx, gy, galt };

    /* a start that is already inside a margin may only move away from it */
    double smargin[AI_MAX_NOFLY];
    for (int z = 0; z < m->n_nofly; z++)
        smargin[z] = fmin(m->margin, ai_zone_sd(&m->nofly[z], sx, sy) - 1.0);

    float min_mult = 1.0f;
    for (int k = 0; k < m->n_corr; k++) if (m->corr[k].mult < min_mult) min_mult = m->corr[k].mult;

    /* ---- 1. build the weighted network ---------------------------------- */
    for (int i = 0; i < M; i++) for (int j = 0; j < M; j++) g_C[i][j] = AI_INF;

    int links = 0;
    for (int i = 0; i < M; i++) {
        for (int j = i + 1; j < M; j++) {
            if (i < N && !m->node[i].ok) continue;
            if (j < N && !m->node[j].ok) continue;
            double d = hypot(P[i].x - P[j].x, P[i].y - P[j].y);
            float  cm = 1.0f; bool corridor = false;
            if (j < N)
                for (int k = 0; k < m->n_corr; k++)
                    if ((m->corr[k].a == i && m->corr[k].b == j) || (m->corr[k].a == j && m->corr[k].b == i)) {
                        corridor = true; if (m->corr[k].mult < cm || cm == 1.0f) cm = m->corr[k].mult;
                    }
            if (d > m->link && !corridor) continue;
            double extra;
            const double *zm = (i == S) ? smargin : NULL;
            if (ai_seg_eval(m, P[i].x, P[i].y, P[j].x, P[j].y, c->sample_step, zm, &extra)) continue;
            float w = (float)(d * cm + extra) + c->climb_w * fabsf(P[i].alt - P[j].alt);
            g_C[i][j] = g_C[j][i] = w;
            links++;
        }
    }
    s->links_built = links;

    /* ---- 2. A* over (previous node, current node) ----------------------- */
    const int NS = (M + 1) * M;
    for (int k = 0; k < NS; k++) { g_g[k] = AI_INF; g_open[k] = g_closed[k] = 0; g_parent[k] = -1; }

    int s0 = M * M + S;                            /* prev = "none" (index M) */
    g_g[s0] = 0.0f; g_open[s0] = 1;
    int expanded = 0, goal_state = -1;

    for (;;) {
        int best = -1; float bf = AI_INF;
        for (int k = 0; k < NS; k++) if (g_open[k]) {
            int cur = k % M;
            float h = (float)hypot(P[cur].x - P[G].x, P[cur].y - P[G].y) * min_mult;
            if (g_g[k] + h < bf) { bf = g_g[k] + h; best = k; }
        }
        if (best < 0) break;                       /* open list empty: no route */
        g_open[best] = 0; g_closed[best] = 1; expanded++;

        int prev = best / M, cur = best % M;
        if (cur == G) { goal_state = best; break; }

        double hin = 0.0; bool have_in = false;
        if (prev < M)           { hin = atan2(P[cur].x - P[prev].x, P[cur].y - P[prev].y); have_in = true; }
        else if (hdg_valid)     { hin = shdg; have_in = true; }

        for (int nx = 0; nx < M; nx++) {
            if (nx == S || nx == prev || g_C[cur][nx] >= AI_INF) continue;
            float turn = 0.0f;
            if (have_in) {
                double hout = atan2(P[nx].x - P[cur].x, P[nx].y - P[cur].y);
                float dpsi = fabsf(ai_wrap_pi((float)(hout - hin)));
                if (dpsi > c->max_turn) continue;
                turn = c->turn_w * dpsi;
            }
            int ns = cur * M + nx;
            if (g_closed[ns]) continue;
            float ng = g_g[best] + g_C[cur][nx] + turn;
            if (ng < g_g[ns]) { g_g[ns] = ng; g_parent[ns] = best; g_open[ns] = 1; }
        }
    }
    s->expanded = expanded;
    if (goal_state < 0) return AI_PLAN_NO_ROUTE;

    /* ---- 3. rebuild node list ------------------------------------------- */
    int rev[AI_MP], nr = 0;
    for (int k = goal_state; k >= 0 && nr < AI_MP; k = g_parent[k]) rev[nr++] = k % M;
    int path[AI_MP], np = 0;
    for (int k = nr - 1; k >= 0; k--) path[np++] = rev[k];
    float astar_cost = g_g[goal_state];

    /* ---- 4. cost-aware string pulling ------------------------------------ */
    int out[AI_MP], no = 0, i = 0;
    out[no++] = path[0];
    while (i < np - 1) {
        int take = i + 1;
        for (int j = np - 1; j > i + 1; j--) {
            double extra;
            const double *zm = (path[i] == S) ? smargin : NULL;
            if (ai_seg_eval(m, P[path[i]].x, P[path[i]].y, P[path[j]].x, P[path[j]].y,
                            c->sample_step, zm, &extra)) continue;
            double d = hypot(P[path[i]].x - P[path[j]].x, P[path[i]].y - P[path[j]].y);
            float direct = (float)(d + extra) + c->climb_w * fabsf(P[path[i]].alt - P[path[j]].alt);
            float chain = 0.0f;
            for (int k = i; k < j; k++) chain += g_C[path[k]][path[k + 1]];
            if (direct <= chain) { take = j; break; }
        }
        out[no++] = path[take];
        i = take;
    }
    if (no > AI_MAX_ROUTE) return AI_PLAN_NO_ROUTE;

    /* ---- 5. publish the route -------------------------------------------- */
    s->n_pts = no;
    s->route_len = 0.0f;
    for (int k = 0; k < no; k++) {
        s->route[k] = P[out[k]];
        if (k) s->route_len += (float)hypot(s->route[k].x - s->route[k - 1].x, s->route[k].y - s->route[k - 1].y);
    }
    s->route_cost = astar_cost;
    s->goal = P[G];
    s->leg = 0;
    s->planned = true;
    s->plan_failed = false;
    return AI_PLAN_OK;
}

/* Plan from `start_spec` to `end_spec` (node names or "x,y[,alt]").
 * start_heading_rad: course the aircraft will leave with, or NAN if unknown. */
ai_plan_status_t ai_nav_plan(ai_nav_t *s, const char *start_spec, const char *end_spec,
                             double start_heading_rad)
{
    double sx, sy, gx, gy; float sa, ga;
    if (ai_map_resolve(&s->map, start_spec, &sx, &sy, &sa)) return AI_PLAN_BAD_INPUT;
    if (ai_map_resolve(&s->map, end_spec,   &gx, &gy, &ga)) return AI_PLAN_BAD_INPUT;
    return ai_plan_route(s, sx, sy, sa, start_heading_rad, !isnan(start_heading_rad), gx, gy, ga);
}

/* ======================================================================== */
/*  Replanning on a pop-up no-fly zone                                      */
/* ======================================================================== */

ai_plan_status_t ai_nav_report_nofly(ai_nav_t *s, double cx, double cy, double r, const ai_input_t *in)
{
    ai_map_t *m = &s->map;
    if (m->n_nofly >= AI_MAX_NOFLY) return AI_PLAN_BAD_INPUT;
    ai_zone_t *z = &m->nofly[m->n_nofly++];
    memset(z, 0, sizeof *z);
    strcpy(z->name, "POPUP");
    z->a = cx; z->b = cy; z->c = r;

    if (s->state != AI_TRACK) return AI_PLAN_KEPT;

    /* does the remaining route still clear the new zone? */
    bool blocked = false;
    double extra, px = in->x_m, py = in->y_m;
    for (int k = s->leg + 1; k < s->n_pts && !blocked; k++) {
        blocked = ai_seg_eval(m, px, py, s->route[k].x, s->route[k].y, s->cfg.sample_step, NULL, &extra);
        px = s->route[k].x; py = s->route[k].y;
    }
    if (!blocked) return AI_PLAN_KEPT;

    double gs = hypot(in->vn_mps, in->ve_mps);
    double hdg = atan2(in->ve_mps, in->vn_mps);
    ai_plan_status_t st = ai_plan_route(s, in->x_m, in->y_m, in->alt_m, hdg, gs > 2.0,
                                        s->goal.x, s->goal.y, s->goal.alt);
    if (st == AI_PLAN_OK) { s->replans++; return st; }

    /* nowhere to go: orbit here and tell the caller */
    s->plan_failed = true;
    s->state = AI_LOITER;
    s->loiter_x = in->x_m; s->loiter_y = in->y_m; s->loiter_alt = in->alt_m;
    return st;
}

/* ======================================================================== */
/*  Flight controller (same logic as nav.c)                                 */
/* ======================================================================== */

const ai_pt_t *ai_nav_route(const ai_nav_t *s, int *n) { *n = s->n_pts; return s->route; }

void ai_nav_update(ai_nav_t *s, const ai_input_t *in, float dt, ai_output_t *o)
{
    const ai_cfg_t *c = &s->cfg;
    memset(o, 0, sizeof *o);
    o->throttle = c->cruise_throttle;

    /* ---- mode handling --------------------------------------------------- */
    if (!in->auto_enable) { s->state = AI_IDLE; return; }
    if (s->state == AI_IDLE) {
        if (s->planned && s->n_pts >= 2 && in->gps_valid) {
            s->route[0] = (ai_pt_t){ in->x_m, in->y_m, in->alt_m };
            s->leg = 0; s->spd_int = 0.0f; s->gps_age = 0.0f;
            s->state = AI_TRACK;
        } else return;
    }

    if (in->gps_valid) s->gps_age = 0.0f; else s->gps_age += dt;
    if (s->gps_age > c->gps_timeout_s && s->state != AI_FAILSAFE) {
        s->state = AI_FAILSAFE;
        s->fs_alt = in->alt_m;
    }

    float gs    = hypotf(in->vn_mps, in->ve_mps);
    float track = atan2f(in->ve_mps, in->vn_mps);
    float v     = in->airspeed_mps;

    /* low-speed envelope scaling */
    float v_prot   = 1.15f * c->min_airspeed;
    float k_low    = ai_clampf((v_prot - v) / (v_prot - c->min_airspeed), 0.0f, 1.0f);
    float bank_max = c->max_bank * (1.0f - 0.5f * k_low);
    float pitch_hi = c->max_pitch_up * (1.0f - k_low);

    float bank_cmd = 0.0f, alt_tgt = in->alt_m;
    float l1 = fmaxf(15.0f, (1.0f / (float)AI_PI) * c->l1_damping * c->l1_period * gs);

    /* ---- guidance -------------------------------------------------------- */
    if (s->state == AI_TRACK) {
        const ai_pt_t *p0 = &s->route[s->leg], *p1 = &s->route[s->leg + 1];
        double ln = p1->y - p0->y, le = p1->x - p0->x;
        double len = hypot(ln, le); if (len < 1.0) len = 1.0;
        double dn = ln / len, de = le / len;

        double an = in->y_m - p0->y, ae = in->x_m - p0->x;
        double along = an * dn + ae * de;
        double xn = an - along * dn, xe = ae - along * de;
        float  xt = (float)hypot(xn, xe);

        float  ahead = sqrtf(fmaxf(l1 * l1 - xt * xt, 0.0f));
        double cs = along + ahead; if (cs > len) cs = len; if (cs < 0.0) cs = 0.0;
        double cn = p0->y + dn * cs, ce = p0->x + de * cs;

        if (gs > 2.0f) {
            float eta = ai_wrap_pi((float)atan2(ce - in->x_m, cn - in->y_m) - track);
            bank_cmd = atanf(2.0f * gs * gs / l1 * sinf(eta) / AI_G);
        }

        float f = ai_clampf((float)(along / len), 0.0f, 1.0f);
        alt_tgt = p0->alt + f * (p1->alt - p0->alt);

        /* turn anticipation: start the next leg early by the turn's lead distance */
        float sw = c->accept_radius;
        if (s->leg + 2 < s->n_pts) {
            const ai_pt_t *p2 = &s->route[s->leg + 2];
            double ln2 = p2->y - p1->y, le2 = p2->x - p1->x, len2 = hypot(ln2, le2);
            float  dpsi = fabsf(ai_wrap_pi((float)(atan2(le2, ln2) - atan2(le, ln))));
            float  vr = fmaxf(gs, c->min_airspeed);
            float  r_turn = vr * vr / (AI_G * tanf(0.8f * c->max_bank));
            float  lead = r_turn * tanf(0.5f * fminf(dpsi, 2.6f));
            float  cap = 0.45f * (float)fmin(len, len2);
            if (lead > cap) lead = cap;
            if (lead > sw) sw = lead;
        }

        double rn_ = p1->y - in->y_m, re_ = p1->x - in->x_m;
        if (hypot(rn_, re_) < sw || along > len) {
            if (s->leg + 1 == s->n_pts - 1) {
                s->state = AI_LOITER;
                s->loiter_x = p1->x; s->loiter_y = p1->y; s->loiter_alt = p1->alt;
            } else s->leg++;
        }
    }
    else if (s->state == AI_LOITER) {
        double rn_ = in->y_m - s->loiter_y, re_ = in->x_m - s->loiter_x;
        float  r = (float)hypot(rn_, re_), R = c->loiter_radius;
        float  theta = (float)atan2(re_, rn_);
        float  e = r - R;
        alt_tgt = s->loiter_alt;
        float des = theta + 0.5f * (float)AI_PI + atan2f(e, l1);
        float err = ai_wrap_pi(des - track);
        float ff  = (gs > 2.0f) ? atanf(gs * gs / (R * AI_G)) : 0.0f;
        bank_cmd = c->k_hdg * err + ff;
    }
    else { bank_cmd = 0.0f; alt_tgt = s->fs_alt; }

    bank_cmd = ai_clampf(bank_cmd, -bank_max, bank_max);

    /* ---- vertical -------------------------------------------------------- */
    float pitch_cmd = c->k_alt * (alt_tgt - in->alt_m);
    pitch_cmd += c->k_bank_pitch * (1.0f / fmaxf(cosf(in->roll_rad), 0.5f) - 1.0f);
    pitch_cmd = ai_clampf(pitch_cmd, -c->max_pitch_down, pitch_hi);

    /* ---- throttle -------------------------------------------------------- */
    float v_err = c->cruise_airspeed - v;
    s->spd_int  = ai_clampf(s->spd_int + c->ki_speed * v_err * dt, -0.3f, 0.3f);
    float thr   = c->cruise_throttle + c->kp_speed * v_err + s->spd_int + c->k_climb_thr * pitch_cmd;
    if (v > c->max_airspeed) thr = 0.0f;
    if (v < c->min_airspeed) thr = 1.0f;
    o->throttle = ai_clampf(thr, 0.0f, 1.0f);

    /* ---- inner loops ----------------------------------------------------- */
    o->aileron  = ai_clampf(c->kp_roll  * (bank_cmd  - in->roll_rad)  - c->kd_roll  * in->p_rps, -1.0f, 1.0f);
    o->elevator = ai_clampf(c->kp_pitch * (pitch_cmd - in->pitch_rad) - c->kd_pitch * in->q_rps, -1.0f, 1.0f);
    o->rudder   = ai_clampf(c->k_rudder * sinf(in->roll_rad), -1.0f, 1.0f);
    o->flaps    = 0.0f;
}

#endif /* AI_NAV_C */
