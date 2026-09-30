function MekfConfigBus = createMekfConfigBus(targetWorkspace)
% Description:
%   Defines priors, process noise and vector-update tuning for the MEKF.
%
% Arguments:
%   targetWorkspace - "base" publishes the bus; otherwise returns it only.
%
% Outputs:
%   MekfConfigBus - Typed bias prior and mixed-unit initial covariance.

if nargin < 1
    targetWorkspace = "base";
end

elements(1) = Simulink.BusElement;
elements(1).Name = "initial_bias_B_rad_s";
elements(1).Dimensions = [3 1];
elements(1).DataType = "double";
elements(1).Unit = "rad/s";
elements(1).Description = "Initial estimated gyro bias, independent of sensor truth";
elements(2) = Simulink.BusElement;
elements(2).Name = "initial_covariance";
elements(2).Dimensions = [6 6];
elements(2).DataType = "double";
elements(2).Unit = "";
elements(2).Description = ...
    "P0 for [delta_theta; delta_bias], constructed from squared SI standard deviations";
names = ["gyro_noise_density_rad_s_sqrt_Hz", ...
    "bias_random_walk_std_rad_s_sqrt_s", "magnetometer_direction_std_rad", ...
    "css_direction_std_rad", "innovation_gate_squared", ...
    "maximum_correction_rad", "time_alignment_tolerance_s", "reference_max_age_s"];
descriptions = ["Independent continuous gyro noise amplitude", ...
    "Independent bias random-walk amplitude", ...
    "Isotropic normalized magnetic-vector noise standard deviation", ...
    "Isotropic normalized Sun-vector noise standard deviation", ...
    "Maximum normalized innovation squared", ...
    "Maximum accepted local attitude correction", ...
    "Maximum absolute report/state time mismatch", ...
    "Maximum age of the onboard SGP4 state used by the references"];
units = ["", "", "rad", "rad", "1", "rad", "s", "s"];
for index = 1:numel(names)
    element = Simulink.BusElement;
    element.Name = names(index);
    element.Dimensions = 1;
    element.DataType = "double";
    element.Unit = units(index);
    element.Description = descriptions(index);
    elements(index + 2) = element;
end

MekfConfigBus = Simulink.Bus;
MekfConfigBus.Description = "Six-error-state MEKF priors and independent filter tuning";
MekfConfigBus.Elements = elements;
if string(targetWorkspace) == "base"
    assignin("base", "MekfConfigBus", MekfConfigBus);
end
end
