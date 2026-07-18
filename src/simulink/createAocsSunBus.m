function AOCS_SunBus = createAocsSunBus(targetWorkspace)
% Description:
%   Defines Sun geometry and unshadowed solar flux products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("sun_B_unit", [3 1], "1", "Unit vector from spacecraft to Sun expressed in body axes");
elems(2) = busElement("sun_I_unit", [3 1], "1", "Unit vector from spacecraft to Sun expressed in inertial axes");
elems(3) = busElement("r_sun_I_m", [3 1], "m", "Approximate Earth-to-Sun position vector expressed in inertial axes");
elems(4) = busElement("sun_distance_m", [1 1], "m", "Approximate spacecraft-to-Sun distance");
elems(5) = busElement("solar_flux_W_m2", [1 1], "W/m^2", "Solar irradiance scaled by spacecraft-to-Sun distance");

AOCS_SunBus = Simulink.Bus;
AOCS_SunBus.Description = "Sun geometry and unshadowed flux product bus";
AOCS_SunBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_SunBus", AOCS_SunBus);
end
end

function elem = busElement(name, dimensions, unit, description)
elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = "double";
elem.Unit = unit;
elem.Description = description;
end
