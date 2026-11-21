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
---
## Tube-Wing Autonomous Waypoint Navigation Controller — `nav.c`
 
### Overview
 
A GPS-based waypoint navigation controller for a conventional tube-and-wing RC aircraft, implemented in C99. The controller flies the aircraft from point A (the position captured when auto mode is engaged) to point B through up to five intermediate waypoints (C–G), then holds a clockwise orbit over B. Lateral guidance uses L1 path following along each leg, altitude is interpolated linearly between waypoint altitudes, and airspeed is held by a PI throttle loop with climb feed-forward. Low-airspeed envelope scaling, a GPS-loss failsafe and turn coordination are built in. The outputs are normalised surface demands in the same channel space as `FCS_TW_WD.c`, so they can be fed into the existing flight control system as the pilot input and still pass through its airspeed and altitude protection.
 
Reference: `Flow Charts/NAV-Flow.pdf`

---

## Tube-Wing AI Route Planning and Flight Controller — `AI-Based-Algo/AI_NAV_V1/AI_NAV.c`

### Overview

A map-based route planning and flight controller for a conventional tube-and-wing RC aircraft, implemented in C99. Given a start and end point on a coordinate-sheet map (`Zone1V1.map`, given as node names or x,y), the planner builds a weighted network of line-of-sight links, where each link's weight combines length (discounted along preferred corridors), hazard-zone penalties and climb cost, and no-fly zones plus a safety margin remove links entirely. A heading-aware A* search with a turn penalty finds the lowest-cost route, string-pulling removes unnecessary waypoints, and the route is flown with the same L1 path following, PI speed control, low-airspeed envelope scaling and GPS-loss failsafe as `nav.c`, ending in a clockwise orbit over the destination. If a pop-up no-fly zone blocks the remaining route, the planner re-plans from the aircraft's current position and heading. The outputs are normalised surface demands in the same channel space as `FCS_TW_WD.c`, so they can be fed into the existing flight control system as the pilot input and still pass through its airspeed and altitude protection.

Reference: `Flow Charts/AI_NAV-Flow.pdf`

---

## BWB PID Flight Control — `BWB_PID.c`
 
### Overview
 
A plain PID flight controller for a blended wing body (two elevons, two spoilers, flaps, rudder and throttle), implemented in C99 with no planner and no neural network. Heading and altitude demands pass through outer loops to bank and pitch demands, then through FCS protections (low-airspeed bank and pitch-up limits, altitude floor and ceiling, flap schedule on airspeed) into inner loops: roll PID, pitch PID with flap and airbrake trim, a speed PI with overspeed cut and airbrake, and a yaw damper with adverse-yaw feed-forward on the rudder. Control allocation gives pitch priority on the elevons and hands excess roll demand to differential spoilers. Integrators are zoned and anti-windup protected. Every inner-loop gain accepts an optional multiplier array `m[7]` (NULL = 1.0), which is the hook the neural-scheduled successor `BWB_FCS.c` plugs into, so this file is the base to build on.
 
Reference: `Flow Chart/BWB_PID-Flow.pdf`

---

## BWB Model Reference Adaptive Control — `BWB_MRAC.c`
 
### Overview
 
A Model Reference Adaptive Control (MRAC) flight controller for a blended wing body (two elevons, two spoilers, flaps, rudder and throttle), implemented in C99 as an alternative to the fixed-gain PID loops in `BWB_PID.c`. Heading and altitude demands, FCS protections and control allocation are shared with the PID version. The inner loops are different: roll and pitch are each forced to follow a second-order reference model, and the controller parameters (state feedback, reference feed-forward and a bias term) are adapted online from the model-following error using a Lyapunov-based direct adaptive law. The speed loop is a first-order adaptive regulator with an airbrake on top. Control authority is treated as unknown, so the loops keep their closed-loop shape when the aircraft changes (damage, speed, CG). Robustness measures are a dead zone, sigma-modification, parameter projection, adaptation frozen while the effectors saturate, and the known dynamic-pressure scaling divided out of the control.
 
Reference: `Flow Chart/BWB_MRAC-Flow.pdf`


 
