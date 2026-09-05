function PlantStateBus = createPlantStateBus(targetWorkspace)
% Description:
%   Defines the truth-state output contract produced by Flight Dynamics.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   PlantStateBus - Simulink.Bus object for plant truth-state products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("time_s", 1, "double", "s", ...
    "Simulation elapsed time from scenario start");
elems(2) = busElement("OrbitState", 1, "Bus: OrbitStateBus", "", ...
    "Truth orbit state from the orbit propagator");
elems(3) = busElement("AttitudeState", 1, "Bus: AttitudeStateBus", "", ...
    "Truth attitude state from the rotational dynamics plant");
elems(4) = busElement("Environment", 1, "Bus: EnvironmentBus", "", ...
    "Truth environment products evaluated at the spacecraft state");

PlantStateBus = Simulink.Bus;
PlantStateBus.Description = ...
    "Flight dynamics truth-state output bus for sensor models";
PlantStateBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "PlantStateBus", PlantStateBus);
end
end

function elem = busElement(name, dimensions, dataType, unit, description)
% Description:
%   Keeps bus element construction compact and consistent.
%
% Arguments:
%   name - Bus element name.
%   dimensions - Element dimensions.
%   dataType - Simulink element data type.
%   unit - Physical unit string.
%   description - Human-readable element description.
%
% Outputs:
%   elem - Simulink.BusElement.

elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = dataType;
elem.Unit = unit;
elem.Description = description;
end
