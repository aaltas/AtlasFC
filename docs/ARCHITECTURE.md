# Architecture Overview

## Design Philosophy

This project follows a **layered architecture** where each module builds on the ones
below it. No circular dependencies are allowed. Every function in `core/` must be
self-contained and testable independently.

## Layer Diagram

```
 ┌─────────────────────────────────────────────────┐
 │              PATH PLANNING (ch11-12)             │
 │         Waypoints, Dubins, RRT, Voronoi          │
 ├─────────────────────────────────────────────────┤
 │              GUIDANCE (ch10)                      │
 │         Line following, Orbit following           │
 ├─────────────────────────────────────────────────┤
 │              AUTOPILOT (ch06)                     │
 │     Lateral: roll→aileron, course→roll            │
 │     Longitudinal: pitch→elevator, alt→pitch       │
 ├─────────────────────────────────────────────────┤
 │         STATE ESTIMATION (ch08-09)                │
 │     EKF, Complementary filter, Sensor fusion      │
 ├─────────────────────────────────────────────────┤
 │              SENSORS (ch07)                       │
 │     IMU, GPS, Barometer, Pitot tube models        │
 ├─────────────────────────────────────────────────┤
 │         FORCES & MOMENTS (ch04)                   │
 │     Aerodynamic forces, Propulsion, Gravity       │
 ├─────────────────────────────────────────────────┤
 │       KINEMATICS & DYNAMICS (ch03)                │
 │     6-DOF EOM, Quaternion propagation             │
 ├─────────────────────────────────────────────────┤
 │        COORDINATE FRAMES (ch02)                   │
 │     Rotation matrices, Euler↔Quat, NED↔Body      │
 ├─────────────────────────────────────────────────┤
 │           PARAMETERS (params/)                    │
 │     Aircraft data, Environment, Simulation config │
 └─────────────────────────────────────────────────┘
```

## Naming Conventions

- Functions: `snake_case` (e.g., `euler_to_rotation.m`)
- Parameters: `snake_case` (e.g., `params.C_L_alpha`)
- Classes: `PascalCase` (when needed)
- Test files: `test_<module_name>.m`

## Future: C/C++ Port Strategy

When MATLAB code is mature:
1. Each `core/` module maps to a C header + source pair
2. MATLAB `params` structs become C `typedef struct`
3. Matrix operations translate to lightweight linear algebra (e.g., Eigen)
4. Unit tests port to Google Test or Unity (embedded C)
