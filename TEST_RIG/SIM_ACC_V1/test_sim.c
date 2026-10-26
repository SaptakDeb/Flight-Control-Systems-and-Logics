/* test_sim.c - crude 3-DOF sim to sanity-check nav.c (not a flight-dynamics model) */
#include "nav.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI    3.14159265358979323846
#define DEG2M 111320.0
#define LAT0  20.0
#define LON0  0.0

static nav_wp_t wp(double north_m, double east_m, float alt)
{
    nav_wp_t w;
    w.lat_deg = LAT0 + north_m / DEG2M;
    w.lon_deg = LON0 + east_m / (DEG2M * cos(LAT0 * PI / 180.0));
    w.alt_m   = alt;
    return w;
}

int main(int argc, char **argv)
{
    int n_via = (argc > 1) ? atoi(argv[1]) : 4;          /* 0..5 */
    if (n_via < 0 || n_via > NAV_MAX_VIA) { fprintf(stderr, "n_via must be 0..%d\n", NAV_MAX_VIA); return 1; }

    /* candidate intermediate points (C, D, E, F, G) and destination B */
    nav_wp_t all_via[NAV_MAX_VIA] = {
        wp( 600,  200, 110), wp(1000, 900, 120), wp( 300, 1500, 100),
        wp(-400, 1200,  90), wp(-700,  500, 100)
    };
    nav_wp_t B = wp(-500, -300, 80);

    nav_cfg_t cfg;  nav_cfg_default(&cfg);
    nav_t nav;      nav_init(&nav, &cfg);
    if (!nav_set_mission(&nav, all_via, (uint8_t)n_via, B)) return 1;

    /* aircraft state: starts at A heading east, 100 m, cruise speed */
    double n = 0, e = 0;  float alt = 100, psi = (float)(PI / 2);
    float roll = 0, pitch = 0, v = cfg.cruise_airspeed;
    nav_input_t in = {0};  nav_output_t out;
    const float dt = 0.02f;
    float loiter_time = 0;

    printf("t,north,east,alt,speed,roll_deg,state,leg\n");
    for (int k = 0; k < 40000; k++) {
        float t = k * dt;

        in.gps_valid = true;
        in.lat_deg = LAT0 + n / DEG2M;
        in.lon_deg = LON0 + e / (DEG2M * cos(LAT0 * PI / 180.0));
        in.alt_m = alt;
        in.vn_mps = v * cosf(psi);  in.ve_mps = v * sinf(psi);
        in.airspeed_mps = v;
        in.roll_rad = roll;  in.pitch_rad = pitch;
        in.auto_enable = true;

        nav_update(&nav, &in, dt, &out);

        /* crude dynamics */
        float p = 2.5f * out.aileron;
        float q = 2.0f * out.elevator;
        in.p_rps = p;  in.q_rps = q;
        roll  += p * dt;
        pitch += q * dt;
        psi   += 9.80665f * tanf(roll) / v * dt;
        alt   += v * sinf(pitch) * dt;
        v     += (12.0f * (out.throttle - 0.55f * (v / 18.0f) * (v / 18.0f)) - 9.80665f * sinf(pitch)) * dt;
        n     += v * cosf(psi) * dt;
        e     += v * sinf(psi) * dt;

        if (k % 250 == 0)
            printf("%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%d,%d\n", t, n, e, alt, v,
                   roll * 180.0 / PI, (int)nav.state, nav_current_leg(&nav));

        if (nav.state == NAV_LOITER && (loiter_time += dt) > 60.0f) break;
    }
    return 0;
}
