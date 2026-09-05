function GyroConfigBus = createGyroConfigBus(targetWorkspace)
% Description:
%   Defines numeric gyroscope configuration values consumed by sensor models.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   GyroConfigBus - Simulink.Bus object for gyroscope configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("enabled", 1, "1", "Gyroscope sensor enable flag");
elems(2) = busElement("mode_id", 1, "1", "Gyroscope mode identifier; 1 = nominal, 2 = failure");
elems(3) = busElement("gyro_sample_time_s", 1, "s", "Gyroscope output sample time");
elems(4) = busElement("noise_density_rad_s_sqrt_Hz", 1, ...
    "", "Gyroscope white-noise density");
elems(5) = busElement("noise_std_rad_s", 1, "rad/s", ...
    "Discrete gyroscope white-noise standard deviation");
elems(6) = busElement("bias_initial_rad_s", [3 1], "rad/s", ...
    "Initial gyroscope bias vector");
elems(7) = busElement("bias_random_walk_std_rad_s_sqrt_s", 1, ...
    "", "Gyroscope bias random-walk standard deviation");
elems(8) = busElement("bias_random_walk_step_std_rad_s", 1, "rad/s", ...
    "Discrete gyroscope bias random-walk step standard deviation");
elems(9) = busElement("scale_factor", [3 1], "1", ...
    "Per-axis fractional gyroscope scale-factor error");
elems(10) = busElement("misalignment_matrix", [3 3], "1", ...
    "Gyroscope misalignment matrix premultiplying the scaled body rate");
elems(11) = busElement("range_rad_s", 1, "rad/s", ...
    "Symmetric gyroscope measurement range");
elems(12) = busElement("resolution_rad_s", 1, "rad/s", ...
    "Gyroscope quantization interval");
elems(13) = busElement("noise_seed", 1, "1", ...
    "Deterministic seed for gyroscope white noise");
elems(14) = busElement("bias_seed", 1, "1", ...
    "Deterministic seed for gyroscope bias random walk");
elems(15) = busElement("failure_output_rad_s", [3 1], "rad/s", ...
    "Gyroscope output used when mode_id selects failure mode");
elems(16) = busElement("failure_valid", 1, "1", ...
    "Validity flag used when mode_id selects failure mode");

GyroConfigBus = Simulink.Bus;
GyroConfigBus.Description = "Gyroscope configuration bus generated from config/sensors.json";
GyroConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "GyroConfigBus", GyroConfigBus);
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
