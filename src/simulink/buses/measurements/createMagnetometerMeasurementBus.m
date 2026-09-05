function MagnetometerMeasurementBus = createMagnetometerMeasurementBus(targetWorkspace)
% Description:
%   Defines magnetometer measurement products consumed by downstream GNC.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   MagnetometerMeasurementBus - Simulink.Bus object for magnetometer measurements.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("B_B_T", [3 1], "T", ...
    "Measured geomagnetic field vector expressed in body axes");
elems(2) = busElement("valid", 1, "1", ...
    "Magnetometer measurement validity flag; 1 = valid, 0 = invalid");

MagnetometerMeasurementBus = Simulink.Bus;
MagnetometerMeasurementBus.Description = "Magnetometer measurement output bus";
MagnetometerMeasurementBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "MagnetometerMeasurementBus", MagnetometerMeasurementBus);
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
