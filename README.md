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
 
