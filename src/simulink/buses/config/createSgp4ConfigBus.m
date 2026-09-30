function Sgp4ConfigBus = createSgp4ConfigBus(targetWorkspace)
% Description:
%   Defines the compact configuration interface consumed by the onboard
%   SGP4 propagator subsystem.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   Sgp4ConfigBus - Simulink.Bus object containing SGP4 epoch elements.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elements(1) = busElement("epoch_utc_jd", "1", ...
    "UTC Julian date of the SGP4 element epoch");
elements(2) = busElement("sgp4_semi_major_axis_m", "m", ...
    "Epoch semimajor axis");
elements(3) = busElement("sgp4_eccentricity", "1", ...
    "Epoch eccentricity");
elements(4) = busElement("sgp4_inclination_deg", "deg", ...
    "Epoch inclination");
elements(5) = busElement("sgp4_raan_deg", "deg", ...
    "Epoch right ascension of the ascending node");
elements(6) = busElement("sgp4_argument_of_periapsis_deg", "deg", ...
    "Epoch argument of periapsis");
elements(7) = busElement("sgp4_true_anomaly_deg", "deg", ...
    "Epoch true anomaly used to initialize mean elements");
elements(8) = busElement("sgp4_bstar", "1", ...
    "SGP4 B-star drag term");

Sgp4ConfigBus = Simulink.Bus;
Sgp4ConfigBus.Description = ...
    "Configuration input of the onboard SGP4 orbit propagator";
Sgp4ConfigBus.Elements = elements;

if targetWorkspace == "base"
    assignin("base", "Sgp4ConfigBus", Sgp4ConfigBus);
end
end

function element = busElement(name, unit, description)
% Description:
%   Creates one scalar double element of the SGP4 configuration bus.
%
% Arguments:
%   name - Bus element name.
%   unit - Physical unit string.
%   description - Human-readable element description.
%
% Outputs:
%   element - Configured Simulink.BusElement object.

element = Simulink.BusElement;
element.Name = name;
element.Dimensions = 1;
element.DataType = "double";
element.Unit = unit;
element.Description = description;
end
