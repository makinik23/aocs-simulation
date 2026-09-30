function AttitudeInitializationBus = createAttitudeInitializationBus(targetWorkspace)
% Description:
%   Defines the instantaneous TRIAD attitude-initialization result.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AttitudeInitializationBus - Simulink.Bus object for TRIAD output.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("q_BI", [4 1], "double", "1", ...
    "Scalar-first TRIAD quaternion mapping inertial axes to body axes");
elems(2) = busElement("DCM_BI", [3 3], "double", "1", ...
    "TRIAD direction cosine matrix mapping inertial vectors to body axes");
elems(3) = busElement("valid", 1, "double", "1", ...
    "TRIAD solution validity flag");
elems(4) = busElement("solution_time_s", 1, "double", "s", ...
    "Onboard time associated with the instantaneous TRIAD solution");
elems(5) = busElement("magnetometer_sequence_id", 1, "uint32", "1", ...
    "Magnetometer report sequence represented by the solution");
elems(6) = busElement("css_sequence_id", 1, "uint32", "1", ...
    "Coarse-Sun-sensor report sequence represented by the solution");
elems(7) = busElement("sgp4_sequence_id", 1, "uint32", "1", ...
    "SGP4 update sequence represented by the reference vectors");

AttitudeInitializationBus = Simulink.Bus;
AttitudeInitializationBus.Description = ...
    "Instantaneous TRIAD attitude-initialization result";
AttitudeInitializationBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AttitudeInitializationBus", ...
        AttitudeInitializationBus);
end
end

function elem = busElement(name, dimensions, dataType, unit, description)
elem = Simulink.BusElement;
elem.Name = name;
elem.Dimensions = dimensions;
elem.DataType = dataType;
elem.Unit = unit;
elem.Description = description;
end
