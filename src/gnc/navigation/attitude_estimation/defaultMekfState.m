function state = defaultMekfState(cfg) %#codegen
%DEFAULTMEKFSTATE Create the initial MEKF state.

estimate.q_BI = [1; 0; 0; 0];
estimate.omega_BI_B_rad_s = zeros(3, 1);
estimate.initialized = false;
estimate.gyro_bias_B_rad_s = cfg.initial_bias_B_rad_s;
estimate.P_error = cfg.initial_covariance;
estimate.valid = false;
estimate.update_time_s = 0.0;

state.estimate = estimate;
state.previous_rate_B_rad_s = zeros(3, 1);
state.older_rate_B_rad_s = zeros(3, 1);
state.have_previous_rate = false;
state.have_two_rates = false;
state.previous_gap_adjacent = false;
state.last_correction_time_s = 0.0;

end
