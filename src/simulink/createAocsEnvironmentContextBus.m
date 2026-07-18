function AOCS_EnvironmentContextBus = createAocsEnvironmentContextBus(targetWorkspace)
% Description:
%   Defines runtime timing and central-body context used by environment models.
%
% Arguments:
%   targetWorkspace - Optional workspace selector. Use "base" to assign the
%                     bus object to the MATLAB base workspace.
%
% Outputs:
%   AOCS_EnvironmentContextBus - Simulink.Bus object for environment context.

if nargin < 1
    targetWorkspace = "base";
end
targetWorkspace = string(targetWorkspace);

elems(1) = busElement("decimal_year", [1 1], "1", "Decimal UTC year used by IGRF");
elems(2) = busElement("mu_m3_s2", [1 1], "m^3/s^2", "Central-body gravitational parameter");
elems(3) = busElement("epoch_utc", [6 1], "1", "UTC epoch [year month day hour minute second]");
elems(4) = busElement("t_s", [1 1], "s", "Simulation elapsed time from epoch");
elems(5) = busElement("epoch_tdb_jd", [1 1], "1", "TDB Julian date corresponding to the simulation epoch");
elems(6) = busElement("delta_at_s", [1 1], "s", "TAI minus UTC offset used by high-accuracy ECI/ECEF transforms");
elems(7) = busElement("delta_ut1_s", [1 1], "s", "UT1 minus UTC offset used by high-accuracy ECI/ECEF transforms");
elems(8) = busElement("polar_motion_rad", [1 2], "rad", "Earth polar motion [xp yp] used by high-accuracy ECI/ECEF transforms");
elems(9) = busElement("d_cip_rad", [1 2], "rad", "IAU-2000/2006 celestial intermediate pole correction [dX dY]");

AOCS_EnvironmentContextBus = Simulink.Bus;
AOCS_EnvironmentContextBus.Description = "Runtime environment timing and central-body context bus";
AOCS_EnvironmentContextBus.Elements = elems;

if targetWorkspace == "base"
    assignin("base", "AOCS_EnvironmentContextBus", AOCS_EnvironmentContextBus);
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
