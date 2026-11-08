/*
 * AI_NAV_SIM.c -- 2D (x,y) simulator for AI_NAV.c
 *
 * Plans a route on a map sheet, flies it with the real controller code against
 * a planar aircraft model (bank dynamics, heading from coordinated turn, simple
 * pitch/altitude and speed channels, steady wind), and scores the result.
 *
 *   gcc -std=c99 -O2 -o ai_nav_sim AI_NAV_SIM.c -lm
 *   ./ai_nav_sim Zone1V1.map [START] [END] [--wind SPEED FROM_DEG] [--no-popup] [--popup-r METRES]
 *                [--svg out.svg] [--csv out.csv]
 *
 * START / END: a node name (HOME, DEPOT, N14 ...) or coordinates "x,y".
 * It #includes AI_NAV.c so one compile line builds everything.
 */
#include "AI_NAV.c"

#define MAX_TRK 6000

typedef struct { double x, y, psi, phi, theta, V, h; } ac_t;

/* ---- helpers --------------------------------------------------------------- */

static double dist_to_line(double px, double py, const ai_pt_t *a, const ai_pt_t *b)
{
    double dx = b->x - a->x, dy = b->y - a->y, len = hypot(dx, dy);
    if (len < 1e-6) return hypot(px - a->x, py - a->y);
    return fabs((px - a->x) * dy - (py - a->y) * dx) / len;
}

/* point `ahead` metres along the remaining route from the aircraft; false if too short */
static bool point_ahead(const ai_nav_t *n, double px, double py, double ahead, double *ox, double *oy)
{
    double x = px, y = py, left = ahead;
    for (int k = n->leg + 1; k < n->n_pts; k++) {
        double seg = hypot(n->route[k].x - x, n->route[k].y - y);
        if (seg >= left) {
            double t = left / seg;
            *ox = x + (n->route[k].x - x) * t; *oy = y + (n->route[k].y - y) * t;
            return true;
        }
        left -= seg; x = n->route[k].x; y = n->route[k].y;
    }
    return false;
}

/* ---- SVG ------------------------------------------------------------------- */

static void svg_write(const char *path, const ai_map_t *m, const ai_pt_t *r0, int n0,
                      const ai_pt_t *r1, int n1, const double *tx, const double *ty, int nt,
                      const ai_zone_t *popup, bool have_popup, const char *title)
{
    FILE *f = fopen(path, "w"); if (!f) return;
    const double s = 0.2, pad = 30.0;
    double W = m->width * s + 2 * pad, H = m->height * s + 2 * pad + 62;
#define SX(x) (pad + (x) * s)
#define SY(y) (pad + (m->height - (y)) * s)
    fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 %.0f %.0f' width='%.0f' height='%.0f' font-family='sans-serif'>\n", W, H, W, H);
    fprintf(f, "<rect width='%.0f' height='%.0f' fill='white'/>\n", W, H);
    fprintf(f, "<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#f7f7f2' stroke='#444'/>\n", SX(0), SY(m->height), m->width * s, m->height * s);
    for (int g = 1000; g < m->width; g += 1000)
        fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#ddd'/><text x='%.1f' y='%.1f' font-size='9' fill='#999'>%d</text>\n", SX(g), SY(0), SX(g), SY(m->height), SX(g) - 8, SY(0) + 12, g);
    for (int g = 1000; g < m->height; g += 1000)
        fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#ddd'/><text x='%.1f' y='%.1f' font-size='9' fill='#999'>%d</text>\n", SX(0), SY(g), SX(m->width), SY(g), SX(0) - 26, SY(g) + 3, g);

    for (int i = 0; i < m->n_haz; i++) {
        const ai_zone_t *z = &m->haz[i];
        if (z->is_box) fprintf(f, "<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#f5a623' fill-opacity='0.22'/>\n", SX(z->a), SY(z->d), (z->c - z->a) * s, (z->d - z->b) * s);
        else fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='#f5a623' fill-opacity='0.22'/>\n", SX(z->a), SY(z->b), z->c * s);
        double lx = z->is_box ? 0.5 * (z->a + z->c) : z->a, ly = z->is_box ? 0.5 * (z->b + z->d) : z->b;
        fprintf(f, "<text x='%.1f' y='%.1f' font-size='9' fill='#a06000' text-anchor='middle'>%s x%.1f</text>\n", SX(lx), SY(ly), z->name, z->mult);
    }
    for (int i = 0; i < m->n_corr; i++) {
        const ai_node_t *a = &m->node[m->corr[i].a], *b = &m->node[m->corr[i].b];
        fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#6fa8dc' stroke-width='5' stroke-opacity='0.35'/>\n", SX(a->x), SY(a->y), SX(b->x), SY(b->y));
    }
    for (int i = 0; i < m->n_nofly; i++) {
        const ai_zone_t *z = &m->nofly[i];
        if (z->is_box) {
            fprintf(f, "<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='none' stroke='#c0392b' stroke-dasharray='4 3'/>\n", SX(z->a - m->margin), SY(z->d + m->margin), (z->c - z->a + 2 * m->margin) * s, (z->d - z->b + 2 * m->margin) * s);
            fprintf(f, "<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#c0392b' fill-opacity='0.45'/>\n", SX(z->a), SY(z->d), (z->c - z->a) * s, (z->d - z->b) * s);
        } else {
            fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='none' stroke='#c0392b' stroke-dasharray='4 3'/>\n", SX(z->a), SY(z->b), (z->c + m->margin) * s);
            fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='#c0392b' fill-opacity='0.45'/>\n", SX(z->a), SY(z->b), z->c * s);
        }
        double lx = z->is_box ? 0.5 * (z->a + z->c) : z->a, ly = z->is_box ? 0.5 * (z->b + z->d) : z->b;
        fprintf(f, "<text x='%.1f' y='%.1f' font-size='9' fill='#7b1d12' text-anchor='middle'>%s</text>\n", SX(lx), SY(ly) + 3, z->name);
    }
    if (have_popup) {
        fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='none' stroke='#8e24aa' stroke-dasharray='4 3'/>\n", SX(popup->a), SY(popup->b), (popup->c + m->margin) * s);
        fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='#8e24aa' fill-opacity='0.45'/>\n", SX(popup->a), SY(popup->b), popup->c * s);
        fprintf(f, "<text x='%.1f' y='%.1f' font-size='9' fill='#4a148c' text-anchor='middle'>POP-UP</text>\n", SX(popup->a), SY(popup->b) + 3);
    }
    for (int i = 0; i < m->n_nodes; i++) {
        fprintf(f, "<circle cx='%.1f' cy='%.1f' r='2.5' fill='%s'/>\n", SX(m->node[i].x), SY(m->node[i].y), m->node[i].ok ? "#555" : "#c0392b");
        fprintf(f, "<text x='%.1f' y='%.1f' font-size='8' fill='#555'>%s</text>\n", SX(m->node[i].x) + 4, SY(m->node[i].y) - 4, m->node[i].name);
    }
    if (r0 && n0 > 1) {
        fprintf(f, "<polyline fill='none' stroke='%s' stroke-width='%s' %s points='", n1 > 0 ? "#888" : "#1f5fbf", n1 > 0 ? "2" : "2.5", n1 > 0 ? "stroke-dasharray='6 4'" : "");
        for (int i = 0; i < n0; i++) fprintf(f, "%.1f,%.1f ", SX(r0[i].x), SY(r0[i].y));
        fprintf(f, "'/>\n");
        for (int i = 0; i < n0; i++) fprintf(f, "<circle cx='%.1f' cy='%.1f' r='3.5' fill='white' stroke='#1f5fbf' stroke-width='1.5'/>\n", SX(r0[i].x), SY(r0[i].y));
    }
    if (r1 && n1 > 1) {
        fprintf(f, "<polyline fill='none' stroke='#1f5fbf' stroke-width='2.5' points='");
        for (int i = 0; i < n1; i++) fprintf(f, "%.1f,%.1f ", SX(r1[i].x), SY(r1[i].y));
        fprintf(f, "'/>\n");
        for (int i = 0; i < n1; i++) fprintf(f, "<circle cx='%.1f' cy='%.1f' r='3.5' fill='white' stroke='#1f5fbf' stroke-width='1.5'/>\n", SX(r1[i].x), SY(r1[i].y));
    }
    if (nt > 1) {
        fprintf(f, "<polyline fill='none' stroke='#1b9e3e' stroke-width='1.8' points='");
        for (int i = 0; i < nt; i++) fprintf(f, "%.1f,%.1f ", SX(tx[i]), SY(ty[i]));
        fprintf(f, "'/>\n");
    }
    double ly = pad + m->height * s + 40;
    fprintf(f, "<text x='%.1f' y='%.1f' font-size='12' font-weight='bold'>%s</text>\n", pad, ly - 8, title);
    fprintf(f, "<text x='%.1f' y='%.1f' font-size='10'><tspan fill='#1f5fbf'>&#9632; planned route</tspan>  <tspan fill='#888'>&#9632; first plan (replaced)</tspan>  <tspan fill='#1b9e3e'>&#9632; flown track</tspan>  <tspan fill='#c0392b'>&#9632; no-fly (dashed = margin)</tspan>  <tspan fill='#f5a623'>&#9632; hazard</tspan>  <tspan fill='#6fa8dc'>&#9632; corridor</tspan></text>\n", pad, ly + 8);
    fprintf(f, "</svg>\n");
    fclose(f);
}

/* ---- main ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *map_path = "Zone1V1.map", *start = NULL, *end = NULL;
    const char *svg_path = "Zone1V1_result.svg", *csv_path = "flown.csv";
    double wind_spd = 0.0, wind_from = 270.0;
    bool popup_on = true;
    double popup_r = 280.0;
    int pos = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--wind") && i + 2 < argc) { wind_spd = atof(argv[++i]); wind_from = atof(argv[++i]); }
        else if (!strcmp(argv[i], "--no-popup")) popup_on = false;
        else if (!strcmp(argv[i], "--popup-r") && i + 1 < argc) popup_r = atof(argv[++i]);
        else if (!strcmp(argv[i], "--svg") && i + 1 < argc) svg_path = argv[++i];
        else if (!strcmp(argv[i], "--csv") && i + 1 < argc) csv_path = argv[++i];
        else if (argv[i][0] != '-') { if (pos == 0) map_path = argv[i]; else if (pos == 1) start = argv[i]; else if (pos == 2) end = argv[i]; pos++; }
        else { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
    }

    ai_map_t map;
    int rc = ai_map_load(&map, map_path);
    if (rc) { fprintf(stderr, "cannot load map %s (code %d)\n", map_path, rc); return 2; }
    if (!start) start = map.def_start;
    if (!end)   end   = map.def_end;

    ai_cfg_t cfg; ai_cfg_default(&cfg);
    static ai_nav_t nav;
    ai_nav_init(&nav, &cfg, &map);

    double sx, sy, gx, gy; float sa, ga;
    if (ai_map_resolve(&map, start, &sx, &sy, &sa) || ai_map_resolve(&map, end, &gx, &gy, &ga)) {
        fprintf(stderr, "bad start/end: use a node name or x,y\n"); return 2;
    }

    /* aircraft leaves facing the end point */
    double hdg0 = atan2(gx - sx, gy - sy);
    ai_plan_status_t st = ai_nav_plan(&nav, start, end, hdg0);
    printf("Map %s (%.0f x %.0f m), %d nodes, %d no-fly, %d hazard, %d corridors\n",
           map.name, map.width, map.height, map.n_nodes, map.n_nofly, map.n_haz, map.n_corr);
    printf("Plan %s -> %s : ", start, end);
    if (st != AI_PLAN_OK) { printf("NO ROUTE (status %d)\n", st); return 1; }
    printf("%d waypoints, %.0f m, cost %.0f, %d links, %d states expanded\n",
           nav.n_pts, nav.route_len, nav.route_cost, nav.links_built, nav.expanded);

    ai_pt_t first_route[AI_MAX_ROUTE]; int n_first = nav.n_pts;
    memcpy(first_route, nav.route, sizeof(ai_pt_t) * n_first);
    float first_len = nav.route_len;

    /* aircraft */
    ac_t ac = { sx, sy, hdg0, 0.0, 0.0, cfg.cruise_airspeed, sa };
    double wn = -wind_spd * cos(wind_from * AI_PI / 180.0), we = -wind_spd * sin(wind_from * AI_PI / 180.0);
    ai_input_t in; memset(&in, 0, sizeof in);
    ai_output_t out;
    float p_rate = 0.0f, q_rate = 0.0f;

    static double trk_x[MAX_TRK], trk_y[MAX_TRK]; int n_trk = 0;
    FILE *csv = fopen(csv_path, "w");
    if (csv) fprintf(csv, "t,x,y,alt,airspeed,roll_deg,leg,state\n");

    const float dt = 0.02f;
    double dist_flown = 0.0, loiter_t = 0.0, sum_xt2 = 0.0, max_xt = 0.0, min_sd = 1e9, min_margin_sd = 1e9;
    long   n_xt = 0, margin_hits = 0, raw_hits = 0;
    bool   popped = false; ai_zone_t popup; memset(&popup, 0, sizeof popup);
    ai_plan_status_t popup_status = AI_PLAN_KEPT;
    double t_end = 0.0, t_arrive = -1.0;
    int    replans_before = 0;

    for (long k = 0; k < 120000; k++) {
        double t = k * dt;

        in.gps_valid = true; in.x_m = ac.x; in.y_m = ac.y; in.alt_m = (float)ac.h;
        in.vn_mps = (float)(ac.V * cos(ac.psi) + wn); in.ve_mps = (float)(ac.V * sin(ac.psi) + we);
        in.airspeed_mps = (float)ac.V; in.roll_rad = (float)ac.phi; in.pitch_rad = (float)ac.theta;
        in.p_rps = p_rate; in.q_rps = q_rate; in.auto_enable = true;

        /* pop-up no-fly zone appears 35 % of the way along, centred ~1500 m ahead on the route */
        if (popup_on && !popped && nav.state == AI_TRACK && dist_flown > 0.35 * first_len) {
            double px, py;
            if (point_ahead(&nav, ac.x, ac.y, 1500.0, &px, &py)) {
                popup.a = px; popup.b = py; popup.c = popup_r; strcpy(popup.name, "POPUP");
                replans_before = nav.replans;
                popup_status = ai_nav_report_nofly(&nav, px, py, popup_r, &in);
                popped = true;
                printf("t=%.0fs pop-up no-fly at (%.0f, %.0f) r=%.0f -> ", t, px, py, popup_r);
                if (popup_status == AI_PLAN_OK) printf("replanned: %d waypoints, %.0f m remaining route\n", nav.n_pts, nav.route_len);
                else if (popup_status == AI_PLAN_KEPT) printf("route still clear, kept\n");
                else printf("NO ROUTE, orbiting in place\n");
                /* the controller restarts the leg list from the aircraft */
                if (popup_status == AI_PLAN_OK) { nav.route[0] = (ai_pt_t){ ac.x, ac.y, (float)ac.h }; nav.leg = 0; }
            } else popped = true;
        }

        ai_nav_update(&nav, &in, dt, &out);

        /* planar 2D aircraft with bank dynamics */
        p_rate = 2.5f * out.aileron; q_rate = 2.0f * out.elevator;
        ac.phi += p_rate * dt; ac.theta += q_rate * dt;
        ac.psi += AI_G * tan(ac.phi) / ac.V * dt;
        ac.h   += ac.V * sin(ac.theta) * dt;
        ac.V   += (12.0 * (out.throttle - 0.55 * (ac.V / 18.0) * (ac.V / 18.0)) - AI_G * sin(ac.theta)) * dt;
        double ox = ac.x, oy = ac.y;
        ac.x   += (ac.V * sin(ac.psi) + we) * dt;
        ac.y   += (ac.V * cos(ac.psi) + wn) * dt;
        dist_flown += hypot(ac.x - ox, ac.y - oy);

        /* metrics */
        if (nav.state == AI_TRACK && nav.leg + 1 < nav.n_pts) {
            double xt = dist_to_line(ac.x, ac.y, &nav.route[nav.leg], &nav.route[nav.leg + 1]);
            if (xt > max_xt) max_xt = xt;
            sum_xt2 += xt * xt; n_xt++;
        }
        for (int z = 0; z < nav.map.n_nofly; z++) {
            double sd = ai_zone_sd(&nav.map.nofly[z], ac.x, ac.y);
            if (sd < min_sd) min_sd = sd;
            if (sd < nav.map.margin) { margin_hits++; if (sd < min_margin_sd) min_margin_sd = sd; }
            if (sd < 0.0) raw_hits++;
        }

        if (k % 25 == 0) {
            if (n_trk < MAX_TRK) { trk_x[n_trk] = ac.x; trk_y[n_trk] = ac.y; n_trk++; }
            if (csv && k % 50 == 0) fprintf(csv, "%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%d,%d\n", t, ac.x, ac.y, ac.h, ac.V, ac.phi * 180.0 / AI_PI, nav.leg, (int)nav.state);
        }

        if (nav.state == AI_LOITER && t_arrive < 0.0) t_arrive = t;
        if (nav.state == AI_LOITER && (loiter_t += dt) > 30.0) { t_end = t; break; }
        t_end = t;
    }
    if (csv) fclose(csv);

    double rms_xt = n_xt ? sqrt(sum_xt2 / n_xt) : 0.0;
    double goal_err = hypot(ac.x - gx, ac.y - gy);
    bool reached = (nav.state == AI_LOITER) && !nav.plan_failed;
    bool pass = reached && raw_hits == 0 && max_xt < 100.0;

    printf("\n---------------- flight summary ----------------\n");
    printf("time to destination  %.0f s\n", t_arrive >= 0.0 ? t_arrive : t_end);
    printf("distance flown       %.0f m   (first plan %.0f m)\n", dist_flown, first_len);
    printf("replans              %d\n", nav.replans);
    printf("cross-track error    max %.1f m   rms %.1f m\n", max_xt, rms_xt);
    printf("no-fly clearance     min %.0f m  (margin %.0f m)  raw-zone hits %ld, margin intrusions %ld samples\n",
           min_sd, map.margin, raw_hits, margin_hits);
    if (reached) printf("reached destination  yes (orbiting it at %.0f m, loiter radius %.0f m)\n", goal_err, cfg.loiter_radius);
    else         printf("reached destination  NO (%.0f m from destination%s)\n", goal_err, nav.plan_failed ? ", no route after pop-up" : "");
    printf("result               %s\n", pass ? "PASS" : "FAIL");
    printf("(wind %.1f m/s from %.0f deg)\n", wind_spd, wind_from);

    char title[160];
    snprintf(title, sizeof title, "%s: %s to %s  |  %s  |  max xt %.0f m, replans %d", map.name, start, end, pass ? "PASS" : "FAIL", max_xt, nav.replans);
    if (popped && popup_status == AI_PLAN_OK)
        svg_write(svg_path, &map, first_route, n_first, nav.route, nav.n_pts, trk_x, trk_y, n_trk, &popup, true, title);
    else
        svg_write(svg_path, &map, first_route, n_first, NULL, 0, trk_x, trk_y, n_trk, &popup, popped, title);
    printf("wrote %s and %s\n", svg_path, csv_path);
    (void)replans_before;
    return pass ? 0 : 1;
}
