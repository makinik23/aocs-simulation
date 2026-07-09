function AOCS_ConfigBus = createAocsConfigBus(targetWorkspace)
% Description:
%   Defines the public plant input contract: inertia, initial Euler attitude,
%   initial body rates, and external torque.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_ConfigBus - Simulink.Bus object for plant configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("I_B", [3 3], "kg*m^2", "Spacecraft inertia matrix expressed in body axes");
elems(2) = busElement("euler_BI_0_rad", [3 1], "rad", "Initial Euler attitude [roll pitch yaw] for Aerospace Blockset 6DOF");
elems(3) = busElement("omega_BI_B_0", [3 1], "rad/s", "Initial body angular velocity wrt inertial, expressed in body axes");
elems(4) = busElement("M_ext_B", [3 1], "N*m", "External torque expressed in body axes");

AOCS_ConfigBus = Simulink.Bus;
AOCS_ConfigBus.Description = "AOCS plant configuration bus generated from config/AocsSimulationConfig.json";
AOCS_ConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_ConfigBus", AOCS_ConfigBus);
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
