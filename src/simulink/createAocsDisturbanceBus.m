function AOCS_DisturbanceBus = createAocsDisturbanceBus(targetWorkspace)
% Description:
%   Defines modeled disturbance torque products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("M_dist_B_Nm", [3 1], "N*m", "Total modeled disturbance torque expressed in body axes");
elems(2) = busElement("M_srp_B_Nm", [3 1], "N*m", "Solar radiation pressure torque expressed in body axes");

AOCS_DisturbanceBus = Simulink.Bus;
AOCS_DisturbanceBus.Description = "Disturbance torque product bus";
AOCS_DisturbanceBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_DisturbanceBus", AOCS_DisturbanceBus);
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
