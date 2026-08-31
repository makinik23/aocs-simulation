function SrpBus = createSrpBus(targetWorkspace)
% Description:
%   Defines solar-radiation-pressure disturbance products.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("M_srp_B_Nm", [3 1], "N*m", "Solar radiation pressure torque expressed in body axes");
elems(2) = busElement("F_srp_B_N", [3 1], "N", "Solar radiation pressure force expressed in body axes");
elems(3) = busElement("P_srp_N_m2", 1, "N/m^2", "Eclipse-shadowed solar radiation pressure");

SrpBus = Simulink.Bus;
SrpBus.Description = "Solar radiation pressure product bus";
SrpBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "SrpBus", SrpBus);
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
