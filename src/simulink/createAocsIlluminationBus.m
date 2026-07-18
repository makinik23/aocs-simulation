function AOCS_IlluminationBus = createAocsIlluminationBus(targetWorkspace)
% Description:
%   Defines eclipse and shadowed solar-flux products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("eclipse_fraction", [1 1], "1", "Fraction of direct solar illumination blocked by eclipse geometry");
elems(2) = busElement("sun_visibility", [1 1], "1", "Fraction of direct solar illumination in [0, 1]");
elems(3) = busElement("solar_flux_shadowed_W_m2", [1 1], "W/m^2", "Solar irradiance after eclipse shadowing");

AOCS_IlluminationBus = Simulink.Bus;
AOCS_IlluminationBus.Description = "Eclipse and shadowed solar-flux product bus";
AOCS_IlluminationBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_IlluminationBus", AOCS_IlluminationBus);
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
