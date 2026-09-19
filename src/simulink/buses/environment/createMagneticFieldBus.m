function MagneticFieldBus = createMagneticFieldBus(targetWorkspace)
% Description:
%   Defines geomagnetic field products used by sensors and disturbance models.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("B_B_T", [3 1], "T", "Geomagnetic field vector expressed in body axes");
elems(2) = busElement("B_I_T", [3 1], "T", "Geomagnetic field vector expressed in inertial axes");

MagneticFieldBus = Simulink.Bus;
MagneticFieldBus.Description = "Geomagnetic field product bus";
MagneticFieldBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "MagneticFieldBus", MagneticFieldBus);
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
