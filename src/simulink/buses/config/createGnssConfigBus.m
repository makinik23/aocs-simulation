function GnssConfigBus = createGnssConfigBus(targetWorkspace)
% Description:
%   Defines numeric GNSS receiver configuration values consumed by the
%   simulated sensor model.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   GnssConfigBus - Simulink.Bus object for GNSS receiver configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("enabled", 1, "1", ...
    "GNSS receiver enable flag");
elems(2) = busElement("mode_id", 1, "1", ...
    "GNSS mode identifier; 1 = nominal, 2 = dropout, 3 = failure");
elems(3) = busElement("sample_time_s", 1, "s", ...
    "GNSS receiver output sample time");
elems(4) = busElement("acquisition_time_s", 1, "s", ...
    "Time after simulation start before the first nominal navigation fix");
elems(5) = busElement("dropout_probability_per_sample", 1, "1", ...
    "Independent probability of losing a nominal navigation fix per sample");
elems(6) = busElement("radial_ionosphere_bias_m", 1, "m", ...
    "Positive radial single-frequency ionosphere bias");
elems(7) = busElement("once_per_orbit_angular_rate_rad_s", 1, "rad/s", ...
    "Angular rate of the receiver once-per-orbit error");
elems(8) = busElement("once_per_orbit_phase_rad", 1, "rad", ...
    "Initial phase of the receiver once-per-orbit error");
elems(9) = busElement("position_periodic_amplitude_RTN_m", [3 1], "m", ...
    "Position amplitudes of the once-per-orbit RTN error");
elems(10) = busElement("velocity_periodic_amplitude_RTN_m_s", [3 1], "m/s", ...
    "Velocity amplitudes of the once-per-orbit RTN error");
elems(11) = busElement("gauss_markov_alpha", 1, "1", ...
    "Discrete first-order Gauss-Markov state transition coefficient");
elems(12) = busElement("position_gauss_markov_step_std_RTN_m", [3 1], "m", ...
    "RTN position Gauss-Markov innovation standard deviation");
elems(13) = busElement("velocity_gauss_markov_step_std_RTN_m_s", [3 1], "m/s", ...
    "RTN velocity Gauss-Markov innovation standard deviation");
elems(14) = busElement("position_white_noise_std_RTN_m", [3 1], "m", ...
    "RTN position white-noise standard deviation");
elems(15) = busElement("velocity_white_noise_std_RTN_m_s", [3 1], "m/s", ...
    "RTN velocity white-noise standard deviation");
elems(16) = busElement("position_resolution_m", 1, "m", ...
    "GNSS position quantization interval");
elems(17) = busElement("velocity_resolution_m_s", 1, "m/s", ...
    "GNSS velocity quantization interval");
elems(18) = busElement("position_gauss_markov_seed", 1, "1", ...
    "Deterministic seed for GNSS position Gauss-Markov innovations");
elems(19) = busElement("velocity_gauss_markov_seed", 1, "1", ...
    "Deterministic seed for GNSS velocity Gauss-Markov innovations");
elems(20) = busElement("position_white_noise_seed", 1, "1", ...
    "Deterministic seed for GNSS position white noise");
elems(21) = busElement("velocity_white_noise_seed", 1, "1", ...
    "Deterministic seed for GNSS velocity white noise");
elems(22) = busElement("dropout_seed", 1, "1", ...
    "Deterministic seed for stochastic navigation-fix dropout");
elems(23) = busElement("failure_r_I_m", [3 1], "m", ...
    "Position output used when mode_id selects failure mode");
elems(24) = busElement("failure_v_I_m_s", [3 1], "m/s", ...
    "Velocity output used when mode_id selects failure mode");
elems(25) = busElement("failure_valid", 1, "1", ...
    "Validity flag used when mode_id selects failure mode");

GnssConfigBus = Simulink.Bus;
GnssConfigBus.Description = ...
    "GNSS receiver configuration bus generated from config/sensors.json";
GnssConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "GnssConfigBus", GnssConfigBus);
end
end

function elem = busElement(name, dimensions, unit, description)
% Description:
%   Keeps bus element construction compact and consistent.
%
% Arguments:
%   name - Bus element name.
%   dimensions - Element dimensions.
%   unit - Physical unit string.
%   description - Human-readable element description.
%
% Outputs:
%   elem - Simulink.BusElement configured as a double.

elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = "double";
elem.Unit = unit;
elem.Description = description;
end
