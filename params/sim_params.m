% =========================================================================
%  SIM_PARAMS - Simulation configuration parameters
% =========================================================================
%  Contains time step, duration, and solver settings for the simulation.
%
%  Usage:
%    sim = sim_params();
% =========================================================================

function sim = sim_params()

    sim.ts   = 0.01;       % s   - simulation time step
    sim.t_end = 100;       % s   - simulation duration

    % --- Plotting ---
    sim.plot_interval = 0.1;  % s - how often to update plots
    sim.video_flag    = false;

    % --- Environment ---
    sim.rho   = 1.2682;    % kg/m^3 - air density at sea level (standard)
    sim.g     = 9.81;      % m/s^2

end
