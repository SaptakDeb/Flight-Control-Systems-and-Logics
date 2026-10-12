# Flight-Control-Systems-and-Logics
FCS focusing on stability and autonomy for aircraft configurations.

# Tube-Wing Flight Control System — `FCS_TW_DL.c`

## Overview

A sensorless, receiver-input-only flight control system for a conventional tube-and-wing RC aircraft, implemented in C99. The controller processes pilot inputs, applies control limits and output smoothing, and manages multiple operating modes without relying on onboard attitude, airspeed, altitude, or position sensors.

## Features

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

## Simulation Outcomes

- Sustained full aileron input caused the estimated bank angle to settle at approximately **41°**, below the configured 45° limit.
- Sustained full elevator input caused the estimated pitch angle to settle at approximately **18° nose-up**, below the configured 20° limit.
- HOLD retained latched control commands until disengagement or pilot override.
- Flap deployment was rate-limited, with corresponding elevator compensation applied.
- Throttle transitions were smoothed by configured slew-rate limits.
- Sustained RC-link loss triggered FAILSAFE, commanding zero throttle, preset aileron, neutral elevator and rudder, and retracted flaps.

## Limitations

The controller does not measure the aircraft's actual attitude or flight state. Bank and pitch estimates depend on commanded inputs and assumed aircraft dynamics, making them susceptible to drift and modelling errors. Consequently, the attitude limiters cannot guarantee actual bank or pitch limits, prevent stalls, or establish flight safety.

The simulation evaluates software behaviour only. Hardware integration, airframe-specific tuning, and controlled flight testing are required to assess real-world performance.

## Build and Run

Compile the built-in simulation using GCC:

```bash
gcc -O2 -Wall -Wextra -std=c99 \
    -DFCS_SIM tubewing_fcs_nosensor.c \
    -o fcs_ns -lm
