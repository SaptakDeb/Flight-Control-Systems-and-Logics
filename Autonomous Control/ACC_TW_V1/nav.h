/*
 * nav.h - Autonomous waypoint navigation for a tube-and-wing aircraft
 *
 * Route:  A (captured at engage) -> via[0..n_via-1] (0..5 points) -> B (final)
 * At B the aircraft orbits (loiter) until auto is disabled.
 *
 * Output is normalised surface demand in the same channel space as the RC
 * map (ch1 aileron, ch2 elevator, ch3 throttle, ch4 rudder, ch6 flaps), so it
 * can be fed into your existing FCS as the "pilot input" and still pass
 * through its envelope protection.
 *
 * Sign conventions
 *   roll   + = right wing down      aileron  + = roll right
 *   pitch  + = nose up              elevator + = nose up
 *   heading/track: radians, 0 = North, clockwise positive
 *   alt: metres, same datum for the aircraft and all waypoints
 */
#ifndef NAV_H
#define NAV_H

#include <stdbool.h>
#include <stdint.h>

#define NAV_MAX_VIA 5                       /* intermediate points (C..F)  */
#define NAV_MAX_PTS (NAV_MAX_VIA + 2)       /* A + via + B                 */

typedef struct {
    double lat_deg, lon_deg;
    float  alt_m;
} nav_wp_t;

typedef struct {
    bool   gps_valid;
    double lat_deg, lon_deg;
    float  alt_m;
    float  vn_mps, ve_mps;          /* GPS ground velocity               */
    float  airspeed_mps;
    float  roll_rad, pitch_rad;
    float  p_rps, q_rps;            /* roll rate, pitch rate             */
    bool   auto_enable;             /* pilot switch / mode request       */
} nav_input_t;

typedef struct {
    float aileron, elevator, rudder;   /* -1 .. +1 */
    float throttle;                    /*  0 .. 1  */
    float flaps;                       /*  0 .. 1  (always 0 in cruise) */
} nav_output_t;

typedef struct {
    /* flight envelope */
    float cruise_airspeed, min_airspeed, max_airspeed;   /* m/s   */
    float cruise_throttle;                               /* 0..1  */
    float max_bank, max_pitch_up, max_pitch_down;        /* rad   */
    /* guidance */
    float accept_radius;      /* m, waypoint reached                    */
    float loiter_radius;      /* m, orbit at B (clockwise)              */
    float l1_period, l1_damping;
    float k_hdg;              /* rad bank per rad heading error (loiter)*/
    float k_alt;              /* rad pitch per metre altitude error     */
    float k_bank_pitch;       /* pitch added in turns: k*(1/cos-1)      */
    /* inner loops */
    float kp_roll, kd_roll, kp_pitch, kd_pitch, k_rudder;
    float kp_speed, ki_speed, k_climb_thr;
    float gps_timeout_s;
} nav_cfg_t;

typedef enum { NAV_IDLE, NAV_TRACK, NAV_LOITER, NAV_FAILSAFE } nav_state_t;

typedef struct {
    nav_cfg_t   cfg;
    nav_state_t state;

    nav_wp_t    via[NAV_MAX_VIA];
    nav_wp_t    dest;
    uint8_t     n_via;
    bool        mission_loaded;

    /* built at engage */
    nav_wp_t    route[NAV_MAX_PTS];
    double      rn[NAV_MAX_PTS], re[NAV_MAX_PTS];   /* local N/E (m) from A */
    uint8_t     n_pts, leg;                         /* leg i: route[i]->[i+1] */
    double      ref_lat, ref_lon;

    float       spd_int, gps_age, fs_alt;
    bool        fs_latched;
} nav_t;

void nav_cfg_default(nav_cfg_t *cfg);
void nav_init(nav_t *nav, const nav_cfg_t *cfg);

/* n_via 0..NAV_MAX_VIA. Returns false (and loads nothing) if n_via is too big. */
bool nav_set_mission(nav_t *nav, const nav_wp_t *via, uint8_t n_via, nav_wp_t dest);

/* Call every control cycle (dt in seconds). Engages automatically on the
 * rising edge of in->auto_enable when GPS is valid and a mission is loaded. */
void nav_update(nav_t *nav, const nav_input_t *in, float dt, nav_output_t *out);

/* Telemetry helpers */
uint8_t nav_current_leg(const nav_t *nav);

#endif
