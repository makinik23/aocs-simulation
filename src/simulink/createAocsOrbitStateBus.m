function AOCS_OrbitStateBus = createAocsOrbitStateBus(targetWorkspace)
% Description:
%   Defines the runtime orbit state produced by the orbit propagator.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_OrbitStateBus - Simulink.Bus object for orbit state signals.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("r_I_m", [3 1], "m", "Spacecraft inertial position vector");
elems(2) = busElement("v_I_m_s", [3 1], "m/s", "Spacecraft inertial velocity vector");

AOCS_OrbitStateBus = Simulink.Bus;
AOCS_OrbitStateBus.Description = "Orbit propagator state output bus";
AOCS_OrbitStateBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_OrbitStateBus", AOCS_OrbitStateBus);
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
