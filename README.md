# Flight-Control-Systems-and-Logics
FCS focusing on stability and autonomy for aircraft configurations.

## Tube-Wing Flight Control System — `FCS_TW_DL.c`

### Overview

A sensorless, receiver-input-only flight control system for a conventional tube-and-wing RC aircraft, implemented in C99. The controller processes pilot inputs, applies control limits and output smoothing, and manages multiple operating modes without relying on onboard attitude, airspeed, altitude, or position sensors.

Reference: `Flow Chart/FCS_TW_DL-Flow.pdf`
---
## Tube-Wing Flight Control System with Airspeed and Altitude Protection — `FCS_TW_WD.c`

### Overview

An enhanced sensor-assisted flight control system for a conventional tube-and-wing RC aircraft, implemented in C99. The controller combines receiver inputs with airspeed and altitude measurements to provide flight-envelope protection, adaptive control authority, and automated responses to potentially unsafe flight conditions. Bank estimation remains dead-reckoned because the system does not use an inertial measurement unit (IMU).

Reference: `Flow Chart/FCS_TW_WD-Flow.pdf`

