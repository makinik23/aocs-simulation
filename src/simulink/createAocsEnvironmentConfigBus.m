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

elems(1) = busElement("M_ext_B", [3 1], "N*m", "User-specified external torque expressed in body axes");
elems(2) = busElement("m_res_B_A_m2", [3 1], "A*m^2", "Residual magnetic dipole expressed in body axes");
elems(3) = busElement("solar_constant_W_m2", 1, "W/m^2", "Nominal solar irradiance at 1 AU");

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
