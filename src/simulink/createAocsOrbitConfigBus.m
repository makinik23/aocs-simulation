function AOCS_OrbitConfigBus = createAocsOrbitConfigBus(targetWorkspace)
% Description:
%   Defines numeric orbit configuration consumed by orbit/environment
%   subsystems.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_OrbitConfigBus - Simulink.Bus object for orbit configuration.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("epoch_utc", [6 1], "1", "UTC epoch [year month day hour minute second]");
elems(2) = busElement("mu_m3_s2", [1 1], "m^3/s^2", "Central-body gravitational parameter");
elems(3) = busElement("central_body_radius_m", [1 1], "m", "Central-body reference radius");
elems(4) = busElement("semi_major_axis_m", [1 1], "m", "Initial Keplerian semi-major axis");
elems(5) = busElement("eccentricity", [1 1], "1", "Initial Keplerian eccentricity");
elems(6) = busElement("inclination_rad", [1 1], "rad", "Initial Keplerian inclination");
elems(7) = busElement("raan_rad", [1 1], "rad", "Initial Keplerian right ascension of ascending node");
elems(8) = busElement("argument_of_periapsis_rad", [1 1], "rad", "Initial Keplerian argument of periapsis");
elems(9) = busElement("true_anomaly_rad", [1 1], "rad", "Initial Keplerian true anomaly");

AOCS_OrbitConfigBus = Simulink.Bus;
AOCS_OrbitConfigBus.Description = "Orbit configuration bus generated from config/AocsSimulationConfig.json";
AOCS_OrbitConfigBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_OrbitConfigBus", AOCS_OrbitConfigBus);
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
