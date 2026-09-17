function CoarseSunSensorConfigBus = createCoarseSunSensorConfigBus(targetWorkspace)
% Description:
%   Defines numeric coarse sun sensor configuration values consumed by sensor
%   models.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   CoarseSunSensorConfigBus - Simulink.Bus object for CSS configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("enabled", 1, "1", ...
    "Coarse sun sensor enable flag");
elems(2) = busElement("mode_id", 1, "1", ...
    "Coarse sun sensor mode identifier; 1 = nominal, 2 = failure");
elems(3) = busElement("sample_time_s", 1, "s", ...
    "Coarse sun sensor output sample time");
elems(4) = busElement("panel_normals_B", [3 6], "1", ...
    "Panel normal vectors expressed in body axes");
elems(5) = busElement("fov_cos", 1, "1", ...
    "Cosine of panel half field-of-view angle");
elems(6) = busElement("min_valid_irradiance_W_m2", 1, "W/m^2", ...
    "Minimum irradiance required for a valid sun vector measurement");
elems(7) = busElement("noise_density_W_m2_sqrt_Hz", 1, ...
    "", "Coarse sun sensor white-noise density");
elems(8) = busElement("noise_std_W_m2", 1, "W/m^2", ...
    "Discrete coarse sun sensor white-noise standard deviation");
elems(9) = busElement("bias_W_m2", [6 1], "W/m^2", ...
    "Per-panel coarse sun sensor bias");
elems(10) = busElement("scale_factor", [6 1], "1", ...
    "Per-panel fractional coarse sun sensor scale-factor error");
elems(11) = busElement("range_W_m2", 1, "W/m^2", ...
    "Symmetric coarse sun sensor panel signal range");
elems(12) = busElement("resolution_W_m2", 1, "W/m^2", ...
    "Coarse sun sensor quantization interval");
elems(13) = busElement("noise_seed", 1, "1", ...
    "Deterministic seed for coarse sun sensor white noise");
elems(14) = busElement("failure_panel_signals_W_m2", [6 1], "W/m^2", ...
    "Panel signals used when mode_id selects failure mode");
elems(15) = busElement("failure_sun_B_unit", [3 1], "1", ...
    "Sun vector used when mode_id selects failure mode");
elems(16) = busElement("failure_irradiance_W_m2", 1, "W/m^2", ...
    "Irradiance used when mode_id selects failure mode");
elems(17) = busElement("failure_valid", 1, "1", ...
    "Validity flag used when mode_id selects failure mode");

CoarseSunSensorConfigBus = Simulink.Bus;
CoarseSunSensorConfigBus.Description = ...
    "Coarse sun sensor configuration bus generated from config/sensors.json";
CoarseSunSensorConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "CoarseSunSensorConfigBus", CoarseSunSensorConfigBus);
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
