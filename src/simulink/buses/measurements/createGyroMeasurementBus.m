function GyroMeasurementBus = createGyroMeasurementBus(targetWorkspace)
% Description:
%   Defines gyroscope measurement products consumed by onboard drivers.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   GyroMeasurementBus - Simulink.Bus object for gyroscope measurements.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("omega_rad_s", [3 1], "rad/s", ...
    "Measured body angular rate from the gyroscope");
elems(2) = busElement("valid", 1, "1", ...
    "Gyroscope measurement validity flag; 1 = valid, 0 = invalid");

GyroMeasurementBus = Simulink.Bus;
GyroMeasurementBus.Description = "Gyroscope measurement output bus";
GyroMeasurementBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "GyroMeasurementBus", GyroMeasurementBus);
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
