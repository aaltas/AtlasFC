# AtlasFC — Flight Controller from First Principles

> Building a Pixhawk-class flight controller from theory to hardware, one chapter at a time.

## Vision

This project implements a complete small UAV flight control system based on the textbook
*Small Unmanned Aircraft: Theory and Practice* by Beard & McLain. Starting from coordinate
frames and rigid-body dynamics, we incrementally build every subsystem — aerodynamics,
sensors, state estimation, autopilot, guidance, and path planning — until we have a
flight-ready controller that can be ported to embedded hardware.

**Phase 1 — MATLAB Prototyping:** Implement and validate every algorithm in MATLAB,
building a reusable library where each layer depends on the ones below it.

**Phase 2 — C/C++ Port:** Translate the validated MATLAB code into efficient C/C++ for
real-time embedded execution.

**Phase 3 — Hardware Integration:** Deploy on a custom flight controller (Pixhawk-class)
with real sensors, actuators, and communication links.

## Repository Structure

```
AtlasFC/
│
├── chapters/              % Chapter-by-chapter study & exercises
│   ├── ch02_coordinate_frames/
│   ├── ch03_kinematics_dynamics/
│   ├── ch04_forces_moments/
│   ├── ch05_trim_linearization/
│   ├── ch06_autopilot_lateral/
│   ├── ch07_sensors/
│   ├── ch08_observers/
│   ├── ch09_design_estimation/
│   ├── ch10_path_following/
│   ├── ch11_path_planning/
│   ├── ch12_path_planning_advanced/
│   └── ch13_vision_guidance/
│
├── core/                  % Reusable flight controller library
│   ├── coordinate_frames/ % Rotation matrices, Euler, Quaternions
│   ├── kinematics/        % 6-DOF equations of motion
│   ├── aerodynamics/      % Lift, drag, moment coefficients
│   ├── propulsion/        % Motor & propeller models
│   ├── wind/              % Steady + stochastic wind (Dryden)
│   ├── sensors/           % IMU, GPS, barometer, pitot tube models
│   ├── state_estimation/  % EKF, complementary filter
│   ├── autopilot/         % PID loops (inner/outer), gain scheduling
│   ├── guidance/          % Path following (line, orbit)
│   ├── path_planning/     % Waypoints, Dubins, RRT
│   └── utilities/         % Common math helpers
│
├── simulation/            % Full closed-loop simulation harness
├── params/                % Aircraft parameters, environment constants
├── tests/                 % Unit tests and validation scripts
├── docs/                  % Notes, derivations, diagrams
├── tools/                 % Plotting, data logging, analysis utilities
│
├── hardware_interface/    % Future: embedded hardware bridge
│   ├── pixhawk_bridge/
│   ├── serial_comm/
│   └── sensor_drivers/
│
└── future/                % Placeholders for advanced phases
    ├── c_cpp_port/        % C/C++ translation of core library
    ├── sitl/              % Software-in-the-loop testing
    ├── hitl/              % Hardware-in-the-loop testing
    └── ros_integration/   % ROS/ROS2 node wrappers
```

## How the Library Builds Up

Each layer depends on the layers below it. This is the dependency chain:

```
Path Planning  (ch11-12)
      |
  Guidance     (ch10)
      |
  Autopilot    (ch06)
      |
State Estimation (ch08-09)
      |
   Sensors     (ch07)
      |
Forces & Moments (ch04)
      |
Kinematics & Dynamics (ch03)
      |
Coordinate Frames (ch02)
      |
  Parameters   (params/)
```

## Getting Started

### Prerequisites
- MATLAB R2022b or later (Simulink optional)

## License

MIT License — See [LICENSE](LICENSE) for details.

## Acknowledgments

- Beard, R.W. & McLain, T.W. — *Small Unmanned Aircraft: Theory and Practice*
- BYU MAGICC Lab — mavsim reference implementation
