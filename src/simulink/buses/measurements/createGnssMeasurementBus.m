function GnssMeasurementBus = createGnssMeasurementBus(targetWorkspace)
% Description:
%   Defines GNSS position and velocity measurement products consumed by GNC.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   GnssMeasurementBus - Simulink.Bus object for GNSS measurements.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("r_I_m", [3 1], "m", ...
    "Measured spacecraft inertial position from the GNSS receiver");
elems(2) = busElement("v_I_m_s", [3 1], "m/s", ...
    "Measured spacecraft inertial velocity from the GNSS receiver");
elems(3) = busElement("valid", 1, "1", ...
    "GNSS navigation-fix validity flag; 1 = valid, 0 = invalid");

GnssMeasurementBus = Simulink.Bus;
GnssMeasurementBus.Description = "GNSS receiver measurement output bus";
GnssMeasurementBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "GnssMeasurementBus", GnssMeasurementBus);
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
