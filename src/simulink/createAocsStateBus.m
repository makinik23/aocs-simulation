function AOCS_StateBus = createAocsStateBus(targetWorkspace)
% Description:
%   Defines the plant output contract: Euler attitude, quaternion, DCM, and
%   body angular rate signals from the 6DOF block.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_StateBus - Simulink.Bus object for logged attitude state.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("euler_rad", [3 1], "rad", "Euler attitude angles [roll pitch yaw] from Aerospace Blockset 6DOF");
elems(2) = busElement("q_be", [4 1], "1", "Quaternion output from Aerospace Blockset 6DOF");
elems(3) = busElement("DCM_be", [3 3], "1", "Direction cosine matrix output from Aerospace Blockset 6DOF");
elems(4) = busElement("omega_b", [3 1], "rad/s", "Body angular rates [p q r]");

AOCS_StateBus = Simulink.Bus;
AOCS_StateBus.Description = "Attitude dynamics state output bus";
AOCS_StateBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_StateBus", AOCS_StateBus);
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
