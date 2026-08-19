function AOCS_EnvironmentConfigBus = createAocsEnvironmentConfigBus(targetWorkspace)
% Description:
%   Defines numeric environment and disturbance configuration values.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_EnvironmentConfigBus - Simulink.Bus object for environment config.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("rmm_enabled", 1, "1", "Residual magnetic moment disturbance enable flag");
elems(2) = busElement("gravity_gradient_enabled", 1, "1", "Gravity-gradient disturbance enable flag");
elems(3) = busElement("m_res_B_A_m2", [3 1], "A*m^2", "Residual magnetic dipole expressed in body axes");
elems(4) = busElement("solar_constant_W_m2", 1, "W/m^2", "Nominal solar irradiance at 1 AU");
elems(5) = busElement("eclipse_enabled", 1, "1", "Eclipse shadowing enable flag");
elems(6) = busElement("srp_enabled", 1, "1", "Solar radiation pressure model enable flag");
elems(7) = busElement("srp_area_ref_m2", 1, "m^2", "SRP reference illuminated area");
elems(8) = busElement("srp_coefficient_reflectivity", 1, "1", "SRP reflectivity coefficient");
elems(9) = busElement("srp_center_of_pressure_B_m", [3 1], "m", ...
    "SRP center-of-pressure offset from center of mass expressed in body axes");
elems(10) = busElement("atmosphere_enabled", 1, "1", "Atmosphere model enable flag");
elems(11) = busElement("atmosphere_model_id", 1, "1", "Atmosphere model identifier; 1 = DTM2020");
elems(12) = busElement("atmosphere_mode_id", 1, "1", ...
    "Atmosphere driver mode identifier; 1 = operational, 2 = research");
elems(13) = busElement("atmosphere_space_weather_source_id", 1, "1", ...
    "Space-weather source identifier; 1 = nominal constants, 2 = file time series");
elems(14) = busElement("atmosphere_uncertainty_enabled", 1, "1", ...
    "Atmosphere uncertainty output enable flag");
elems(15) = busElement("rho_scale_factor", 1, "1", ...
    "Multiplicative neutral-density scale factor applied to DTM2020 density");
elems(16) = busElement("f10_7_sfu", 1, "1", "Nominal daily F10.7 solar flux driver [sfu]");
elems(17) = busElement("f10_7_81d_sfu", 1, "1", "Nominal 81-day mean F10.7 solar flux driver [sfu]");
elems(18) = busElement("kp", 1, "1", "Nominal planetary Kp geomagnetic driver");
elems(19) = busElement("f30_sfu", 1, "1", "Nominal F30 solar flux driver for DTM2020 research mode [sfu]");
elems(20) = busElement("f30_81d_sfu", 1, "1", ...
    "Nominal 81-day mean F30 solar flux driver for DTM2020 research mode [sfu]");
elems(21) = busElement("hp60", 1, "1", "Nominal hourly Hp60 geomagnetic driver for DTM2020 research mode");

AOCS_EnvironmentConfigBus = Simulink.Bus;
AOCS_EnvironmentConfigBus.Description = "Environment configuration bus generated from config/AocsSimulationConfig.json";
AOCS_EnvironmentConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_EnvironmentConfigBus", AOCS_EnvironmentConfigBus);
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
