% =========================================================================
%  TODO — System Identification (Aerodynamic Parameter Estimation)
% =========================================================================
%
%  GOAL:
%    Use real flight log data (PX4 .ulg) to estimate the aerodynamic
%    coefficients of the vehicle by comparing EKF state estimates with
%    the forces & moments model predictions.
%
%  APPROACH:
%    1. Load real flight log  →  load_ulg_sensors()  (already exists)
%    2. Run AtlasFC EKF       →  get state history x_hat(t)
%    3. Extract control inputs from log  (throttle, aileron, elevator, rudder)
%    4. Feed state + control into forces_moments()  →  predicted F, M
%    5. Compare predicted acceleration with IMU accel measurements
%    6. Minimize residuals  →  estimate CL, CD, Cm, etc.
%
%  KEY FUNCTIONS TO WRITE:
%    [ ] load_ulg_controls.m    — read actuator_outputs / actuator_controls
%                                 topic from PX4 log
%    [ ] sysid_residual.m       — compute  accel_measured - accel_predicted
%                                 given a candidate parameter set
%    [ ] run_sysid.m            — main script: calls lsqnonlin / fmincon
%                                 to minimize residuals over param set
%    [ ] sysid_params.m         — initial guess and bounds for aero params
%                                 (CL0, CLa, CD0, CDa, Cm0, Cma, ...)
%
%  METHODS TO CONSIDER:
%    - Equation-error method   (fast, linear, closed-form least squares)
%    - Output-error method     (slower, nonlinear, more accurate)
%    - Frequency-domain method (for structural modes, later)
%
%  REFERENCE:
%    Klein & Morelli, "Aircraft System Identification", AIAA 2006
%    Jategaonkar, "Flight Vehicle System Identification", AIAA 2006
%    Beard & McLain, Ch. 5 (aerodynamic model used here)
%
%  DEPENDENCIES (already in AtlasFC):
%    forces_moments.m    ← evaluate model at given state + control
%    mav_params.m        ← baseline parameter struct to be updated
%    run_flight_eval.m   ← EKF state history pipeline
%
%  STATUS: TODO — implement after Ch9/Ch10 guidance chapters
% =========================================================================
