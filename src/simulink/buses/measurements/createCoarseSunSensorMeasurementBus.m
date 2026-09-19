function CoarseSunSensorMeasurementBus = createCoarseSunSensorMeasurementBus(targetWorkspace)
% Description:
%   Defines coarse sun sensor measurement products consumed by downstream GNC.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   CoarseSunSensorMeasurementBus - Simulink.Bus object for CSS measurements.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("sun_B_unit", [3 1], "1", ...
    "Measured Sun direction expressed in body axes");
elems(2) = busElement("irradiance_W_m2", 1, "W/m^2", ...
    "Estimated solar irradiance from coarse sun sensor panels");
elems(3) = busElement("panel_signals_W_m2", [6 1], "W/m^2", ...
    "Raw coarse sun sensor panel irradiance readings");
elems(4) = busElement("valid", 1, "1", ...
    "Coarse sun sensor measurement validity flag; 1 = valid, 0 = invalid");

CoarseSunSensorMeasurementBus = Simulink.Bus;
CoarseSunSensorMeasurementBus.Description = ...
    "Coarse sun sensor measurement output bus";
CoarseSunSensorMeasurementBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "CoarseSunSensorMeasurementBus", ...
        CoarseSunSensorMeasurementBus);
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
