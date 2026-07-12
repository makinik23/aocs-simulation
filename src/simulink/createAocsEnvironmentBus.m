function AOCS_EnvironmentBus = createAocsEnvironmentBus(targetWorkspace)
% Description:
%   Defines runtime environment products consumed by disturbance and sensor
%   models.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_EnvironmentBus - Simulink.Bus object for environment products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("M_dist_B_Nm", [3 1], "N*m", "Total modeled disturbance torque expressed in body axes");
elems(2) = busElement("B_I_T", [3 1], "T", "Geomagnetic field vector expressed in inertial axes");
elems(3) = busElement("B_B_T", [3 1], "T", "Geomagnetic field vector expressed in body axes");
elems(4) = busElement("sun_B_unit", [3 1], "1", "Unit vector from spacecraft to Sun expressed in body axes");
elems(5) = busElement("sun_I_unit", [3 1], "1", "Unit vector from spacecraft to Sun expressed in inertial axes");
elems(6) = busElement("r_sun_I_m", [3 1], "m", "Approximate Earth-to-Sun position vector expressed in inertial axes");
elems(7) = busElement("sun_distance_m", 1, "m", "Approximate spacecraft-to-Sun distance");
elems(8) = busElement("solar_flux_W_m2", 1, "W/m^2", "Solar irradiance scaled by spacecraft-to-Sun distance");

AOCS_EnvironmentBus = Simulink.Bus;
AOCS_EnvironmentBus.Description = "Runtime environment products bus";
AOCS_EnvironmentBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_EnvironmentBus", AOCS_EnvironmentBus);
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
