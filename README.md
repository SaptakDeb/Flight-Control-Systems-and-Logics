# Flight-Control-Systems-and-Logics
FCS focusing on stability and autonomy for aircraft configurations.

## Tube-Wing Flight Control System — `FCS_TW_DL.c`

### Overview

A sensorless, receiver-input-only flight control system for a conventional tube-and-wing RC aircraft, implemented in C99. The controller processes pilot inputs, applies control limits and output smoothing, and manages multiple operating modes without relying on onboard attitude, airspeed, altitude, or position sensors.

### Features

- **Operating modes:** DISARMED, NORMAL, HOLD, and FAILSAFE.
- **Throttle arming interlock:** Requires a valid RC link and idle throttle before arming.
- **Input shaping:** Applies a deadband and exponential response to improve stick control.
- **Control authority limits:** Restricts aileron, elevator, and rudder commands, with reduced aileron authority as flaps deploy.
- **Bank and pitch limiters:** Uses dead-reckoned estimates and command tapering to reduce control inputs near configured attitude limits.
- **HOLD mode:** Latches selected control inputs and allows pilot override.
- **Failsafe:** Applies preset control inputs, zero throttle, and retracted flaps when the RC link is lost.
- **Control mixing:** Implements aileron-to-rudder mixing and flap–elevator compensation.
- **Output slew limiting:** Restricts the rate of change of control-surface, throttle, and flap commands.
- **Simulation harness:** Tests predefined control inputs, operating-mode transitions, and link-loss behaviour.

### Simulation Outcomes

- Sustained full aileron input caused the estimated bank angle to settle at approximately **41°**, below the configured 45° limit.
- Sustained full elevator input caused the estimated pitch angle to settle at approximately **18° nose-up**, below the configured 20° limit.
- HOLD retained latched control commands until disengagement or pilot override.
- Flap deployment was rate-limited, with corresponding elevator compensation applied.
- Throttle transitions were smoothed by configured slew-rate limits.
- Sustained RC-link loss triggered FAILSAFE, commanding zero throttle, preset aileron, neutral elevator and rudder, and retracted flaps.
---

# Tube-Wing Flight Control System with Airspeed and Altitude Protection — `FCS_TW_WD.c`

## Overview

An enhanced sensor-assisted flight control system for a conventional tube-and-wing RC aircraft, implemented in C99. The controller combines receiver inputs with airspeed and altitude measurements to provide flight-envelope protection, adaptive control authority, and automated responses to potentially unsafe flight conditions. Bank estimation remains dead-reckoned because the system does not use an inertial measurement unit (IMU).

## Features

- **Operating modes:** DISARMED, GROUND, NORMAL, HOLD, and FAILSAFE.
- **Throttle arming interlock:** Requires a valid RC link and idle throttle before arming.
- **Airspeed processing:** Filters airspeed measurements and rejects stale, invalid, or implausible readings.
- **Altitude processing:** Filters altitude and vertical-speed estimates, rejects transient barometric spikes, and reinitialises filtering after sensor recovery.
- **Adaptive control authority:** Scales aileron, elevator, and rudder commands according to airspeed using an inverse-square speed schedule.
- **Stall protection:** Reduces nose-up elevator and aileron authority near the stall margin, applies a nose-down bias, and enforces a throttle floor.
- **Overspeed protection:** Reduces throttle, limits nose-down elevator authority, and applies a gentle pull-up bias near the configured maximum speed.
- **Altitude ceiling protection:** Reduces nose-up authority near the ceiling, applies a nose-down bias above the ceiling, and limits throttle.
- **Altitude floor protection:** Supports descent-command limiting and recovery responses during excessive sink rates when configured with above-ground-level (AGL) altitude.
- **Flight-path protection:** Uses altitude-derived vertical speed and airspeed to estimate flight-path angle and limit climb or descent commands.
- **Flap protection:** Enforces flap speed limits and prevents flap retraction under specified low-speed conditions.
- **HOLD mode:** Latches pilot inputs while retaining active flight protections and allowing throttle reduction.
- **FAILSAFE:** Commands a gentle turn, a configured throttle level, retracted flaps, and airspeed-based elevator control when the RC link is lost in flight.
- **Output slew limiting:** Restricts the rate of change of control surfaces, throttle, and flap commands.
- **Simulation harness:** Includes a simplified pitch, airspeed, and altitude plant to evaluate controller responses to predefined flight scenarios.

## Simulation Outcomes

The built-in simulation exercises the controller under a sequence of flight conditions:

- **Climb command:** Sustained nose-up elevator input tests climb response, altitude-ceiling protection, and stall-margin management.
- **Overspeed scenario:** Full nose-down elevator and full throttle test overspeed-related control and throttle limiting.
- **Flap deployment:** Full flap deployment tests airspeed-based flap restrictions.
- **HOLD engagement:** A gentle nose-up command is latched and subsequently held after the pilot input is released.
- **RC-link loss:** Loss of the receiver signal triggers the FAILSAFE mode and its configured in-flight control response.
- **Barometric spike:** A temporary 25 m altitude disturbance tests rejection of implausible altitude changes.

The controller also computes filtered vertical speed and flight-path angle, schedules control authority according to airspeed, and applies protective corrections before generating servo outputs.

## Default Configuration

| Parameter | Value |
|---|---:|
| Clean stall speed | 11.0 m/s |
| Full-flap stall speed | 9.5 m/s |
| Half-flap maximum speed | 19.0 m/s |
| Full-flap maximum speed | 16.0 m/s |
| Maximum operating speed (Vne) | 30.0 m/s |
| Reference airspeed | 18.0 m/s |
| Altitude ceiling | 120 m |
| Ceiling protection margin | 20 m |
| Maximum climb flight-path angle | 15° |
| Maximum descent flight-path angle | 15° |
| Control-loop frequency | 100 Hz |
| RC-link timeout | 250 ms |
| Air-data timeout | 200 ms |

These values are initial configuration parameters and require adjustment for the target aircraft and sensor installation.

