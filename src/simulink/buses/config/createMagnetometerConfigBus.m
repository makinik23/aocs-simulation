function MagnetometerConfigBus = createMagnetometerConfigBus(targetWorkspace)
% Description:
%   Defines numeric magnetometer configuration values consumed by sensor
%   models.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   MagnetometerConfigBus - Simulink.Bus object for magnetometer configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("enabled", 1, "1", "Magnetometer sensor enable flag");
elems(2) = busElement("mode_id", 1, "1", "Magnetometer mode identifier; 1 = nominal, 2 = failure");
elems(3) = busElement("sample_time_s", 1, "s", "Magnetometer output sample time");
elems(4) = busElement("noise_density_T_sqrt_Hz", 1, ...
    "", "Magnetometer white-noise density");
elems(5) = busElement("noise_std_T", 1, "T", ...
    "Discrete magnetometer white-noise standard deviation");
elems(6) = busElement("bias_initial_T", [3 1], "T", ...
    "Initial magnetometer bias vector");
elems(7) = busElement("bias_random_walk_std_T_sqrt_s", 1, ...
    "", "Magnetometer bias random-walk standard deviation");
elems(8) = busElement("bias_random_walk_step_std_T", 1, "T", ...
    "Discrete magnetometer bias random-walk step standard deviation");
elems(9) = busElement("scale_factor", [3 1], "1", ...
    "Per-axis fractional magnetometer scale-factor error");
elems(10) = busElement("misalignment_matrix", [3 3], "1", ...
    "Magnetometer misalignment matrix premultiplying the scaled field");
elems(11) = busElement("range_T", 1, "T", ...
    "Symmetric magnetometer measurement range");
elems(12) = busElement("resolution_T", 1, "T", ...
    "Magnetometer quantization interval");
elems(13) = busElement("noise_seed", 1, "1", ...
    "Deterministic seed for magnetometer white noise");
elems(14) = busElement("bias_seed", 1, "1", ...
    "Deterministic seed for magnetometer bias random walk");
elems(15) = busElement("failure_output_T", [3 1], "T", ...
    "Magnetometer output used when mode_id selects failure mode");
elems(16) = busElement("failure_valid", 1, "1", ...
    "Validity flag used when mode_id selects failure mode");

MagnetometerConfigBus = Simulink.Bus;
MagnetometerConfigBus.Description = "Magnetometer configuration bus generated from config/sensors.json";
MagnetometerConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "MagnetometerConfigBus", MagnetometerConfigBus);
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
